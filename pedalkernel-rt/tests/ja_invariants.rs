//! Jiles-Atherton transformer core invariant / property test suite.
//!
//! bead: pedalkernel-mnql
//!
//! Tests 8 physics invariants that golden-file comparison cannot catch:
//!
//! 1. [`ja_loop_closure`]        — steady-state loop closes (no drift over N cycles)
//! 2. [`ja_loop_area_positive`]  — energy dissipation >= 0; area scales with k
//! 3. [`ja_anhysteretic_properties`] — M_an monotonic, odd-symmetric, Langevin Taylor
//! 4. [`ja_m_bounded_by_ms`]     — |M| <= Ms under extreme drive and parameter corners
//! 5. [`ja_dc_bias_ampere_law`]  — DC-bias init matches Ampere law for gap 0..1 mm
//! 6. [`ja_sample_rate_invariance`] — 48k vs 192k loop areas agree within tolerance
//! 7. [`ja_nan_instability_corners`] — no NaN/inf across JaCoreModel parameter corners
//! 8. [`ja_thd_low_frequency_energy`] — THD energy concentrated below ~100 Hz
//!
//! IMPORTANT: failing invariants are documented with #[ignore] and a precise
//! finding.  Do NOT modify jiles_atherton.rs — findings belong to the user.

use pedalkernel_rt::elements::JaCoreModel;
use pedalkernel_rt::elements::JaMagnetizingRoot;

// ─── shared helpers ──────────────────────────────────────────────────────────

/// Drive a `JaMagnetizingRoot` with a pure sine for `periods` full cycles.
///
/// Returns `(h_samples, m_samples, b_reflected_samples)` collected across all
/// samples.  The wave amplitude `amp_a` is the WDF port-a wave (volts).
fn drive_sine(
    root: &mut JaMagnetizingRoot,
    amp_a: f64,
    f0: f64,
    fs: f64,
    periods: usize,
) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let samples_per_period = (fs / f0).round() as usize;
    let n = samples_per_period * periods;
    let mut hs = Vec::with_capacity(n);
    let mut ms = Vec::with_capacity(n);
    let mut bs = Vec::with_capacity(n);
    for k in 0..n {
        let a = amp_a * (std::f64::consts::TAU * f0 * k as f64 / fs).sin();
        let b = root.process(a);
        let st = root.state();
        hs.push(st.h);
        ms.push(st.m);
        bs.push(b);
    }
    (hs, ms, bs)
}

/// Compute the loop area of a (H, M) trace via the shoelace formula (trapezoidal).
/// A closed, dissipative loop has area > 0.
fn loop_area_hm(hs: &[f64], ms: &[f64]) -> f64 {
    assert_eq!(hs.len(), ms.len());
    let n = hs.len();
    let mut area = 0.0_f64;
    for i in 0..n {
        let j = (i + 1) % n;
        area += (hs[i] + hs[j]) * (ms[j] - ms[i]);
    }
    0.5 * area
}


// ─── invariant 1: hysteresis loop closure ────────────────────────────────────

/// Invariant 1 — Why: a physically correct J-A model must produce a periodic
/// steady-state B-H loop.  If the loop drifts over N cycles the numerical
/// integration has accumulated error that will corrupt long-running audio.
///
/// Method: drive 20 cycles, compare (H, M) at the start of cycle 19 and 20.
/// Both must be within 0.01 % of Ms.
#[test]
fn ja_loop_closure() {
    let model = JaCoreModel::si_steel_demo();
    let (fs, f0, amp) = (48_000.0, 50.0, 40.0);
    let mut root = JaMagnetizingRoot::new(model);
    root.prepare(fs, 1000.0);

    let spp = (fs / f0).round() as usize;
    let total = spp * 20;

    // Record (H, M) at each sample for the last two cycles.
    let mut cycle19: Option<(f64, f64)> = None;
    let mut cycle20: Option<(f64, f64)> = None;

    for k in 0..total {
        let a = amp * (std::f64::consts::TAU * f0 * k as f64 / fs).sin();
        root.process(a);
        if k == 18 * spp {
            let st = root.state();
            cycle19 = Some((st.h, st.m));
        }
        if k == 19 * spp {
            let st = root.state();
            cycle20 = Some((st.h, st.m));
        }
    }

    let (h19, m19) = cycle19.unwrap();
    let (h20, m20) = cycle20.unwrap();

    let tol = 1.0e-4 * model.ms; // 0.01% of Ms

    assert!(
        (h19 - h20).abs() < tol,
        "Invariant 1 FAIL — H drifts between cycle 19 and 20: delta={:.4e} (tol={:.4e})",
        (h19 - h20).abs(),
        tol
    );
    assert!(
        (m19 - m20).abs() < tol,
        "Invariant 1 FAIL — M drifts between cycle 19 and 20: delta={:.4e} (tol={:.4e})",
        (m19 - m20).abs(),
        tol
    );
}

// ─── invariant 2: loop area (energy dissipation) ─────────────────────────────

/// Invariant 2 — Why: the J-A model must be thermodynamically passive; the
/// area enclosed by a hysteresis loop equals the energy dissipated per cycle
/// and must be strictly positive.  Additionally, increasing k (coercivity)
/// must strictly widen the loop — this is the primary physical role of k.
///
/// Method: compare loop areas for k=200 (soft) vs k=800 (hard) at the same
/// drive level. Hard must be larger. Both must be positive.
#[test]
fn ja_loop_area_positive_and_scales_with_k() {
    let mut model_soft = JaCoreModel::si_steel_demo();
    model_soft.k = 200.0;

    let mut model_hard = JaCoreModel::si_steel_demo();
    model_hard.k = 800.0;

    let (fs, f0, amp, rp) = (48_000.0, 50.0, 40.0, 1000.0);
    let warmup = 8;
    let measure = 4;

    let area_for = |model: JaCoreModel| {
        let mut root = JaMagnetizingRoot::new(model);
        root.prepare(fs, rp);
        // warm up
        drive_sine(&mut root, amp, f0, fs, warmup);
        // measure
        let (hs, ms, _) = drive_sine(&mut root, amp, f0, fs, measure);
        loop_area_hm(&hs, &ms)
    };

    let area_soft = area_for(model_soft);
    let area_hard = area_for(model_hard);

    assert!(
        area_soft > 0.0,
        "Invariant 2 FAIL — soft loop area not positive: {area_soft:.4e}"
    );
    assert!(
        area_hard > 0.0,
        "Invariant 2 FAIL — hard loop area not positive: {area_hard:.4e}"
    );
    assert!(
        area_hard > area_soft,
        "Invariant 2 FAIL — hard loop (k=800, area={area_hard:.4e}) not wider than soft (k=200, area={area_soft:.4e})"
    );
}

// ─── invariant 3: anhysteretic properties ────────────────────────────────────

/// Invariant 3 — Why: M_an = Ms * L((H + α*M)/a) is the anhysteretic (no-loss)
/// magnetization. It must be (a) monotonically non-decreasing in H, (b)
/// odd-symmetric (M_an(-H) = -M_an(H)), and (c) match the Langevin
/// small-argument Taylor expansion L(x) ≈ x/3 for very small H.
///
/// These properties are guaranteed by the Langevin function itself but the
/// iterative solver `solve_anhysteretic_m` (internal) could in principle
/// converge to a wrong branch. We probe via the steady-state core trajectory:
/// M_an is evaluated implicitly by running the core at zero k (pure
/// anhysteretic) and reading M.
#[test]
fn ja_anhysteretic_properties() {
    // k → 0 removes hysteresis, M = M_an
    let mut model = JaCoreModel::si_steel_demo();
    model.k = 1.0e-6; // effectively zero irreversibility
    model.c = 1.0; // purely reversible
    model.alpha = 0.0; // no back-coupling for this test

    let fs = 48_000.0;
    let f0 = 50.0;
    let rp = 100.0; // low impedance so H closely tracks drive

    let mut root = JaMagnetizingRoot::new(model);
    root.prepare(fs, rp);
    // warm-up
    drive_sine(&mut root, 5.0, f0, fs, 4);

    let spp = (fs / f0).round() as usize;

    // Collect one cycle
    let mut pts: Vec<(f64, f64)> = Vec::with_capacity(spp);
    for k in 0..spp {
        let a = 5.0 * (std::f64::consts::TAU * f0 * k as f64 / fs).sin();
        root.process(a);
        let st = root.state();
        pts.push((st.h, st.m));
    }

    // (a) odd symmetry: M(-H) ≈ -M(H)
    // Match each point (H, M) to the point closest to (-H) and compare -M.
    // We use a coarse check over the mid-zero-crossing.
    let pos_half: Vec<_> = pts.iter().filter(|(h, _)| *h > 500.0).collect();
    let neg_half: Vec<_> = pts.iter().filter(|(h, _)| *h < -500.0).collect();
    let sym_ok = pos_half.iter().all(|(hp, mp)| {
        // find closest neg h
        neg_half
            .iter()
            .find(|(hn, _)| (hn + hp).abs() < 200.0)
            .map(|(_, mn)| (mp + mn).abs() < 0.05 * model.ms)
            .unwrap_or(true) // can't check if no pair found
    });
    assert!(sym_ok, "Invariant 3 FAIL — anhysteretic is not odd-symmetric");

    // (b) monotonic: sort by H, M must be non-decreasing
    let mut sorted = pts.clone();
    sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    let monotonic = sorted.windows(2).all(|w| w[1].1 >= w[0].1 - 1.0e-4 * model.ms);
    assert!(
        monotonic,
        "Invariant 3 FAIL — anhysteretic M is not monotonic in H"
    );

    // (c) Langevin small-x: for very small H, M ≈ Ms * (H/a) / 3
    // Use a tiny-drive run to test the near-zero region.
    let mut root2 = JaMagnetizingRoot::new(model);
    root2.prepare(fs, rp);
    // one sample with very small drive
    let tiny_a = 0.001; // tiny a-wave
    root2.process(tiny_a);
    let st = root2.state();
    let h_small = st.h;
    let m_small = st.m;
    if h_small.abs() > 1.0e-6 {
        // M_an ≈ Ms * (H/a) / 3
        let expected_m_an = model.ms * (h_small / model.a) / 3.0;
        let rel_err = (m_small - expected_m_an).abs() / (expected_m_an.abs().max(1.0));
        assert!(
            rel_err < 0.15,
            "Invariant 3 FAIL — small-H Langevin Taylor: m={m_small:.4e}, expected≈{expected_m_an:.4e}, rel_err={rel_err:.4e}"
        );
    }
}

// ─── invariant 4: |M| bounded by Ms ─────────────────────────────────────────

/// Invariant 4 — Why: the J-A model physics requires |M| ≤ Ms by construction
/// (Langevin function saturates).  Numerical instability or large Newton steps
/// can violate this, causing solver divergence or clipping artefacts.
///
/// Method: run extreme drive (100× nominal), parameter corners, and verify
/// peak |M| stays within 1% above Ms at all times.
///
/// FINDING (pedalkernel-mnql): FAILS for the "low_a" corner (a=100 A/m).
/// With a=100 the anhysteretic curve is extremely steep; at extreme drive
/// (400 V port-a) the Newton step limiter (STEP_LIMIT_A * a = 5000 A/m) is
/// too coarse to constrain M: peak |M| reaches 1.2e11 (>>Ms=1.6e6).
/// The STEP_LIMIT_A constant was tuned for a=1100 (the demo model); scaling
/// it relative to `a` would fix this, but that is a model change.
#[test]
#[ignore = "pedalkernel-mnql: |M|>>Ms at low_a corner — Newton step limiter \
            STEP_LIMIT_A not scaled relative to `a`; peak=1.2e11, limit=1.62e6"]
fn ja_m_bounded_by_ms() {
    let corners: &[(&str, JaCoreModel)] = &[
        ("default", JaCoreModel::si_steel_demo()),
        ("high_k", {
            let mut m = JaCoreModel::si_steel_demo();
            m.k = 4000.0;
            m
        }),
        ("low_a", {
            let mut m = JaCoreModel::si_steel_demo();
            m.a = 100.0;
            m
        }),
        ("high_c", {
            let mut m = JaCoreModel::si_steel_demo();
            m.c = 0.95;
            m
        }),
        ("high_alpha", {
            let mut m = JaCoreModel::si_steel_demo();
            m.alpha = 1.0e-2;
            m
        }),
    ];

    for (name, model) in corners {
        let ms = model.ms;
        let mut root = JaMagnetizingRoot::new(*model);
        root.prepare(48_000.0, 1000.0);

        let fs: f64 = 48_000.0;
        let f0: f64 = 50.0;
        let amp = 400.0_f64; // extreme drive

        let spp = (fs / f0).round() as usize;
        let n = spp * 6;

        let mut peak_m = 0.0_f64;
        for k in 0..n {
            let a = amp * (std::f64::consts::TAU * f0 * k as f64 / fs).sin();
            root.process(a);
            let st = root.state();
            assert!(
                st.m.is_finite(),
                "Invariant 4 FAIL [{name}] — M is not finite at sample {k}"
            );
            peak_m = peak_m.max(st.m.abs());
        }

        // Allow 1% numerical overshoot above Ms
        let limit = 1.01 * ms;
        assert!(
            peak_m <= limit,
            "Invariant 4 FAIL [{name}] — peak |M|={peak_m:.4e} exceeds 1.01*Ms={limit:.4e}"
        );
    }
}

// ─── invariant 5: DC bias init vs Ampere law ─────────────────────────────────

/// Invariant 5 — Why: `set_dc_bias_current` seeds the core from a DC current
/// by solving Ampere's law across the gap. If the result doesn't satisfy
/// H * le_eff + M * gap = N * I, the warm-start is wrong, causing an audible
/// transient when the circuit switches on.
///
/// Method: sweep gap from 0 to 1 mm, check Ampere law balance after init.
#[test]
fn ja_dc_bias_ampere_law() {
    let base = JaCoreModel::si_steel_demo();
    let dc_current = 45.0e-3; // 45 mA

    let gaps_mm = [0.0, 0.05, 0.1, 0.25, 0.5, 0.75, 1.0];

    for gap_mm in gaps_mm {
        let gap = gap_mm * 1.0e-3;
        let mut model = base;
        model.gap = gap;

        let mut root = JaMagnetizingRoot::new(model);
        root.prepare(48_000.0, 1000.0);
        root.set_dc_bias_current(dc_current);

        let st = root.state();
        assert!(
            st.h.is_finite() && st.m.is_finite(),
            "Invariant 5 FAIL [gap={gap_mm:.2}mm] — state is non-finite after DC bias init"
        );

        // Ampere law: N*I = H * (path_len + gap) + M * gap
        //                  = H * le_eff + M * gap
        let le_eff = model.path_len + gap;
        let ampere_turns = model.n_turns * dc_current;
        let reconstructed = st.h * le_eff + st.m * gap;
        let rel_err = (reconstructed - ampere_turns).abs()
            / ampere_turns.abs().max(1.0e-6 * model.ms);

        assert!(
            rel_err < 1.0e-4,
            "Invariant 5 FAIL [gap={gap_mm:.2}mm] — Ampere law mismatch: \
             reconstructed={reconstructed:.6e}, expected={ampere_turns:.6e}, rel_err={rel_err:.2e}"
        );

        // Magnetization must be non-zero for non-trivial current
        assert!(
            st.m.abs() > 1.0e-6 * model.ms,
            "Invariant 5 FAIL [gap={gap_mm:.2}mm] — M is near-zero after DC init: {:.4e}",
            st.m
        );
    }
}

// ─── invariant 6: sample-rate invariance ─────────────────────────────────────

/// Invariant 6 — Why: implicit trapezoidal integration is second-order
/// accurate; the B-H loop area should be sample-rate independent at audio
/// frequencies.  If 48k vs 192k areas diverge by more than a few percent,
/// the integration is leaking energy or adding artificial losses.
///
/// Method: run 8+4 cycles at both rates, compare final loop areas. Tolerance
/// is 5% relative (generous to accommodate real discretisation error).
#[test]
fn ja_sample_rate_invariance() {
    let model = JaCoreModel::si_steel_demo();
    let (f0, amp, rp) = (50.0, 40.0, 1000.0);
    let warmup = 8;
    let measure = 4;

    let area_at_fs = |fs: f64| {
        let mut root = JaMagnetizingRoot::new(model);
        root.prepare(fs, rp);
        drive_sine(&mut root, amp, f0, fs, warmup);
        let (hs, ms, _) = drive_sine(&mut root, amp, f0, fs, measure);
        loop_area_hm(&hs, &ms)
    };

    let area_48k = area_at_fs(48_000.0);
    let area_192k = area_at_fs(192_000.0);

    assert!(
        area_48k > 0.0,
        "Invariant 6 FAIL — 48k loop area non-positive: {area_48k:.4e}"
    );
    assert!(
        area_192k > 0.0,
        "Invariant 6 FAIL — 192k loop area non-positive: {area_192k:.4e}"
    );

    let rel_diff = (area_48k - area_192k).abs() / area_192k.abs();
    assert!(
        rel_diff < 0.05,
        "Invariant 6 FAIL — loop area diverges between 48k ({area_48k:.4e}) \
         and 192k ({area_192k:.4e}): rel_diff={rel_diff:.4e} > 5%"
    );
}

// ─── invariant 7: no NaN / instability at parameter corners ─────────────────

/// Invariant 7 — Why: production code receives arbitrary user parameters from
/// the circuit description language.  Any combination of valid (strictly
/// positive) J-A parameters must not produce NaN, infinity, or divergence.
///
/// Method: run 3 cycles of moderate drive across a grid of extreme but
/// physically valid parameter combinations. Any non-finite output is a
/// defect.
///
/// FINDING (pedalkernel-mnql): FAILS for 6 corners where alpha=5e-3.
/// All failures share alpha=5e-3 (high back-coupling). At these parameter
/// values the denominator `den = 1 - alpha * chi_reversible` in `mdot_eval`
/// can approach zero or go negative, causing the susceptibility chi to
/// diverge. The Jacobian determinant guard (`abs(det) < 1e-30`) fires but the
/// state is already corrupt. The chi formula needs a clamp or denominator
/// guard for alpha >= a/(Ms * L'_max) ≈ a * 3 / (Ms * alpha). Specifics:
///   ms=5.0e5 a=50  alpha=5e-3 k=400  c=0.2
///   ms=1.6e6 a=50  alpha=5e-3 k=4000 c=0.2
///   ms=1.6e6 a=500 alpha=5e-3 k=10   c=0.2
///   ms=1.6e6 a=500 alpha=5e-3 k=400  c=0.2
///   ms=2.0e6 a=500 alpha=5e-3 k=10   c=0.2
///   ms=2.0e6 a=500 alpha=5e-3 k=400  c=0.2
#[test]
#[ignore = "pedalkernel-mnql: NaN/inf at high alpha=5e-3 corners — `1 - alpha*chi` \
            denominator goes near-zero in mdot_eval; 6 failing corners documented above"]
fn ja_nan_instability_corners() {
    // (ms, a, alpha, k, c) — all combinations below must remain finite
    let ms_vals = [0.5e6, 1.6e6, 2.0e6];
    let a_vals = [50.0, 500.0, 5000.0];
    let alpha_vals = [0.0, 1.0e-4, 5.0e-3];
    let k_vals = [10.0, 400.0, 4000.0];
    let c_vals = [0.0, 0.2, 0.9];

    let fs: f64 = 48_000.0;
    let f0: f64 = 50.0;
    let amp = 40.0_f64;
    let rp = 1000.0_f64;
    let spp = (fs / f0).round() as usize;
    let n = spp * 3;

    let mut failures: Vec<String> = Vec::new();

    for &ms in &ms_vals {
        for &a in &a_vals {
            for &alpha in &alpha_vals {
                for &k in &k_vals {
                    for &c in &c_vals {
                        let model = JaCoreModel {
                            ms,
                            a,
                            alpha,
                            k,
                            c,
                            n_turns: 2000.0,
                            area: 2.0e-4,
                            path_len: 0.10,
                            gap: 0.0,
                        };
                        if !model.is_complete() {
                            continue;
                        }

                        let mut root = JaMagnetizingRoot::new(model);
                        root.prepare(fs, rp);

                        let mut bad = false;
                        for k_idx in 0..n {
                            let a_wave =
                                amp * (std::f64::consts::TAU * f0 * k_idx as f64 / fs).sin();
                            let b = root.process(a_wave);
                            let st = root.state();
                            if !b.is_finite() || !st.h.is_finite() || !st.m.is_finite() {
                                bad = true;
                                break;
                            }
                        }
                        if bad {
                            failures.push(format!(
                                "ms={ms:.1e} a={a} alpha={alpha:.1e} k={k} c={c}"
                            ));
                        }
                    }
                }
            }
        }
    }

    assert!(
        failures.is_empty(),
        "Invariant 7 FAIL — NaN/inf at {} parameter corners:\n  {}",
        failures.len(),
        failures.join("\n  ")
    );
}

// ─── invariant 8: THD energy concentrated below 100 Hz ──────────────────────

/// Invariant 8 — Why: at moderate drive (well below saturation), the
/// transformer nonlinearity produces harmonic distortion at integer multiples
/// of f0.  For f0 = 50 Hz, significant harmonic energy at 50, 100, 150 Hz is
/// expected.  Energy above ~1 kHz on a 50 Hz signal would indicate
/// high-frequency numerical noise or aliasing — a model quality concern.
///
/// Method: drive at moderate level, collect reflected wave b, compute discrete
/// Fourier amplitudes via Goertzel at key frequencies. Energy below 10*f0
/// (500 Hz) must represent at least 99% of the total spectral energy.
#[test]
fn ja_thd_low_frequency_energy() {
    let model = JaCoreModel::si_steel_demo();
    let fs: f64 = 48_000.0;
    let f0: f64 = 50.0;
    let amp = 20.0_f64;
    let rp = 1000.0_f64;
    let warmup = 8;

    let mut root = JaMagnetizingRoot::new(model);
    root.prepare(fs, rp);
    drive_sine(&mut root, amp, f0, fs, warmup);

    // Collect 4 cycles of signal
    let measure_periods = 4;
    let spp = (fs / f0).round() as usize;
    let n = spp * measure_periods;
    let mut signal: Vec<f64> = Vec::with_capacity(n);
    for k in 0..n {
        let a = amp * (std::f64::consts::TAU * f0 * k as f64 / fs).sin();
        let b = root.process(a);
        signal.push(b);
    }

    // Goertzel DFT amplitude at frequency f.
    let goertzel = |freq: f64| -> f64 {
        let omega = std::f64::consts::TAU * freq / fs;
        let coeff = 2.0 * omega.cos();
        let (mut s1, mut s2) = (0.0_f64, 0.0_f64);
        for &x in &signal {
            let s0 = x + coeff * s1 - s2;
            s2 = s1;
            s1 = s0;
        }
        // power
        s1 * s1 + s2 * s2 - coeff * s1 * s2
    };

    // Measure energy at harmonics 1..10 (50 Hz to 500 Hz) and 11..100
    let low_energy: f64 = (1..=10).map(|h| goertzel(f0 * h as f64)).sum();
    let high_energy: f64 = (11..=200).map(|h| goertzel(f0 * h as f64)).sum();

    let total = low_energy + high_energy;
    if total <= 0.0 {
        // degenerate: no energy; the loop above checks finiteness elsewhere
        return;
    }

    let low_fraction = low_energy / total;
    assert!(
        low_fraction >= 0.99,
        "Invariant 8 FAIL — only {:.1}% of harmonic energy is below 500 Hz \
         (expected ≥99%): low={low_energy:.4e} high={high_energy:.4e}",
        low_fraction * 100.0
    );
}
