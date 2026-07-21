//! PHASE 0 reference oracle for pedalkernel-6vy (cross-stage Thevenin
//! impedance modeling). `#[ignore]`d — this is the golden ANCHOR, established
//! BEFORE any engine change, per the golden-bootstrap rule: we never regenerate
//! a reference to match a bug. It currently FAILS against the unmodified
//! engine (that failure is the point — see bead pedalkernel-ge7) and should
//! start passing once pedalkernel-6vy's cross-stage R_th injection lands.
//!
//! CIRCUIT: `crates/bivalve/bivalve-core/mfb_lpf.pedal` (sibling pro repo),
//! loaded via `regression_harness::load_pro_pedal_sub` — the SAME real
//! circuit `product_regression_matrix.rs` already regression-tests. A
//! classic multiple-feedback (MFB) active lowpass:
//!
//!   in --R_in(10k)-- n1 --Rc1(Cutoff_R1 pot)-- n2 --+-- C1(10n) -- gnd
//!                                                    |
//!                                    +-- Rq(R_q_floor=10k) -- Rres(Resonance pot) -- U1.neg
//!
//!   U1.neg --Rc3(Cutoff_R3 pot, ganged w/ Cutoff_R1)-- U1.out   (feedback R)
//!   U1.neg --C2(10n)-- U1.out                                   (feedback C)
//!   U1.pos = vref (AC ground via C_vref=47uF)
//!
//! ROOT CAUSE UNDER TEST (bead ge7, confirmed via independent sympy KCL):
//! `build_mna` (pedalkernel/src/compiler/rigid/mna_builder.rs:527) stamps an
//! IDEAL, ZERO-SOURCE-IMPEDANCE voltage source at U1.neg. The SPQR partitioner
//! (`compile_via_spqr_with_options`, spqr_build.rs) splits this circuit into 3
//! flow groups compiled independently — 2 upstream Wdf groups (R_in,
//! Cutoff_R1, C1, R_q_floor, Resonance) plus 1 downstream StateSpace group
//! (the op-amp feedback network) — which connect only via a runtime node
//! value, not a retained upstream-stage object. Physically, U1.neg is fed
//! through real series impedance from that upstream network. Zero-Zin
//! modeling discards exactly the impedance ratio that sets the MFB inverting
//! gain -Zf/Zin, so the engine currently computes ~open-loop gain (~200,300x,
//! ge7's measured number, from an idealized instantaneous/DC KCL solve of the
//! as-stamped topology) instead of the real ratio (~-0.83 at defaults).
//!
//! ORACLE METHOD: independent ideal-op-amp KCL solve in sympy
//! (`build_transfer_function` derivation reproduced below in the doc comments
//! of each test), CROSS-VALIDATED against an independent ngspice AC analysis
//! (VCVS op-amp model, Aol=1e9) — both solvers agree to within the sampling
//! grid's frequency resolution (see scratchpad mfb_oracle.py / mfb_spice_check.cir
//! run 2026-07-20). Pot resistances use the CONFIRMED range-remap + linear
//! ("b") taper formula (`pedalkernel-rt/src/processor.rs` `set_control`,
//! `pedalkernel/src/compiler/spqr_control.rs`):
//!   ranged = clamp01(range.0 + raw*(range.1 - range.0)); R = max(ranged*max_r, max_r*0.001)
//! Both "Cutoff" and "Resonance" controls declare range=[1.0, 0.05] in the
//! .pedal file, giving:
//!   raw=0.0 -> R=100_000 ohm   raw=0.5 -> R=52_500 ohm   raw=1.0 -> R=5_000 ohm
//!
//! 3x3 GRID (Cutoff raw x Resonance raw, all in {0.0, 0.5, 1.0}) DC-gain
//! oracle values (Vout/Vin, ideal op-amp limit):
//!
//! | Cutoff\Resonance |     0.0     |     0.5     |     1.0     |
//! |------------------|-------------|-------------|-------------|
//! |       0.0        | -0.454545   | -0.579710   | -0.800000   |
//! |       0.5        | -0.304348   | -0.420000   | -0.677419   |
//! |       1.0        | -0.040000   | -0.064516   | -0.166667   |
//!
//! R1 CHECKPOINT EVIDENCE (see pedalkernel-6vy.2): the driving-point impedance
//! Zin(s) looking from `in` toward U1.neg is NOT a scalar resistance — C1 sits
//! as a shunt-to-ground INSIDE the series path (between Cutoff_R1 and
//! R_q_floor+Resonance), so Zin(s) = R_in + Rc1 + [1/(sC1) || (Rq+Rres)] is
//! frequency-dependent. Measured Zin(DC) vs Zin(20kHz) ratio across the 3x3
//! grid ranges from 0.12 to 0.88 (never close to 1.0) — see
//! `zin_is_frequency_dependent_not_scalar` below, which encodes this as a
//! standalone (non-ignored) analytical fact, independent of engine behavior.
//! CONFIRMED NUMERICALLY (not just qualitatively): a "Thevenin VOLTAGE
//! (open-circuit) + scalar R_th = R_q_floor+Resonance" Norton model gives the
//! WRONG DC gain even though R_q_floor+Resonance itself contains no
//! capacitor — e.g. Cutoff=0.5/Resonance=0.0 gives -0.477273 that way vs the
//! true -0.304348 (57% relative error), because it silently drops C1's
//! LOADING effect on the upstream divider (the upstream stage's own output
//! impedance, which does depend on C1, must be summed with the pure-resistive
//! R_q_floor+Resonance to get the right answer — using the open-circuit
//! Thevenin voltage together with a scalar series R skips that term). Using
//! the FULL complex Zth(s) = (R_in+Rc1) || 1/(sC1) in series with
//! R_q_floor+Resonance reproduces the true oracle exactly. This is the
//! concrete evidence that a Phase-1-style scalar R_th stamp, even restricted
//! to "only the resistive part of the path", is an approximation with a
//! measurable (tens-of-percent) error for this circuit — not a rounding
//! artifact.
//!
//! ADDITIONAL FINDING BEYOND ge7's ORIGINAL DIAGNOSIS: the current engine's
//! compiled StateSpace stage for this circuit is not merely "wrong gain, but
//! stable" — it is GENUINELY UNSTABLE. n_states=1 (physically should be 2:
//! C1 and C2 are both real reactive elements), single real eigenvalue
//! -1.337011033099298 (|eigenvalue| > 1, an unstable discrete pole). A
//! small-signal time-domain probe confirms this: output is not a clean scaled
//! sinusoid but an erratic buildup-then-reset pattern (state grows by ~1.337x
//! per sample until the divergence guard at stage.rs:6323-6329 zeroes it).
//! ge7's own sympy check used an idealized instantaneous/DC KCL solve (as-
//! stamped, Zin=0) and got a large-but-finite ~200,300x — that answer is
//! correct for the STATIC gain implied by the stamped topology, but does not
//! capture the actual discrete-time RECURSIVE stability question, which only
//! shows up once the Schur-complement elimination reduces 2 physical states
//! down to 1. Both findings point at the same root cause (zero-Zin injection
//! at mna_builder.rs:527) but describe different symptoms; see
//! `mfb_lpf_current_engine_state_space_is_unstable` below, which documents
//! this directly and will need updating once Phase 1+ makes the stage stable.

mod regression_harness;

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel_rt::processor::Stage;
use regression_harness::load_pro_pedal_sub;

const SAMPLE_RATE: f64 = 96_000.0;

/// Range-remap + linear ("b") taper, confirmed against
/// `pedalkernel-rt/src/processor.rs::Processor::set_control` and
/// `pedalkernel/src/compiler/spqr_control.rs`.
fn pot_ohms(raw: f64, range: (f64, f64), max_r: f64) -> f64 {
    let ranged = (range.0 + raw * (range.1 - range.0)).clamp(0.0, 1.0);
    (ranged * max_r).max(max_r * 0.001)
}

const POT_RANGE: (f64, f64) = (1.0, 0.05);
const POT_MAX_R: f64 = 100_000.0;

/// Sympy-derived (ngspice cross-checked) ideal-op-amp DC gain oracle,
/// (cutoff_raw, resonance_raw) -> Vout/Vin. See module doc table.
const DC_GAIN_ORACLE: &[((u8, u8), f64)] = &[
    ((0, 0), -0.454545),
    ((0, 5), -0.579710),
    ((0, 10), -0.800000),
    ((5, 0), -0.304348),
    ((5, 5), -0.420000),
    ((5, 10), -0.677419),
    ((10, 0), -0.040000),
    ((10, 5), -0.064516),
    ((10, 10), -0.166667),
];

fn raw_from_tenths(t: u8) -> f64 {
    t as f64 / 10.0
}

/// Loads the real BiValve MFB LPF circuit, or `None` if the sibling pro repo
/// is absent (public CI) — mirrors `product_regression_matrix.rs`.
fn load_mfb_lpf() -> Option<String> {
    load_pro_pedal_sub("crates/bivalve/bivalve-core/mfb_lpf.pedal")
}

/// Compiles the MFB LPF at the given (Cutoff, Resonance) raw control values
/// and returns the compiled `StateSpaceStage`'s exact DC gain, computed
/// directly from its discrete state-space matrices:
///   H(z=1) = D + C (I - A)^-1 B
/// (`B` here is `b_vector[..n]`, the `u[n]` term; a bilinear form may also
/// carry a `b_vector[n..]` `u[n-1]` term, which at DC (z=1, u[n]=u[n-1])
/// simply adds to B — see `StateSpaceStage::process`, stage.rs:6291-6349).
///
/// WHY MATRICES, NOT A TIME-DOMAIN SINE SWEEP (Rule 12, fail loud): this
/// circuit's CURRENT (buggy) compiled StateSpaceStage has n_states=1 with a
/// single real eigenvalue of -1.337 — |eigenvalue| > 1, i.e. the compiled
/// discrete system is GENUINELY UNSTABLE, not merely wrong-gain-but-stable.
/// Confirmed empirically: a small-signal (0.001) time-domain probe produces
/// an erratic, non-sinusoidal, growing-then-reset output (the divergence
/// guard at stage.rs:6323-6329 zeroes state once |x| exceeds v_rail*100),
/// NOT a clean scaled/phase-shifted sine at any fixed gain. A Goertzel/FFT
/// "measured dB" number from that signal is not a meaningful gain — it
/// reflects whatever transient the divergence guard produces, not a
/// comparable steady-state answer. Reading DC gain directly off (A, B, C, D)
/// sidesteps the instability entirely and is exact, so this oracle test
/// stays meaningful whether or not the compiled system happens to be stable.
fn compiled_dc_gain(src: &str, cutoff: f64, resonance: f64) -> f64 {
    let pedal = parse_pedal_file(src).expect("parse mfb_lpf.pedal");
    let mut proc = compile_pedal(&pedal, SAMPLE_RATE).expect("compile mfb_lpf.pedal");
    // `set_control` only sets a smoother TARGET that ramps during `process()`;
    // a one-shot compile-then-read-matrices (no samples processed) never
    // advances it, so every grid point would silently read back the same
    // compile-time-default position. `set_control_immediate` applies the
    // value synchronously (see pedalkernel-rt/src/processor.rs and the
    // project's headless-CLAP-controls memory note).
    proc.set_control_immediate("Cutoff", cutoff);
    proc.set_control_immediate("Resonance", resonance);

    let ss = proc
        .stages
        .iter()
        .find_map(|s| match s {
            Stage::StateSpace(ss) => Some(ss),
            _ => None,
        })
        .expect("MFB LPF must compile a StateSpace stage (the op-amp feedback group)");

    let n = ss.ss.n_states;
    let a = &ss.ss.a_matrix; // n x n, row-major
    let b = &ss.ss.b_vector; // n (u[n] term) or 2n (+ u[n-1] term)
    let c = &ss.ss.c_vector; // n
    let d = ss.ss.d_feedthrough;
    let has_b_minus = b.len() >= n * 2;

    // Effective input vector at DC: u[n] == u[n-1] == 1, so b_eff[i] =
    // b[i] + (b[n+i] if present).
    let b_eff: Vec<f64> = (0..n)
        .map(|i| b[i] + if has_b_minus { b[n + i] } else { 0.0 })
        .collect();

    // Solve (I - A) x = b_eff via Gaussian elimination (n is small: 1-4).
    let mut m = vec![0.0_f64; n * n];
    for i in 0..n {
        for j in 0..n {
            m[i * n + j] = if i == j { 1.0 } else { 0.0 } - a[i * n + j];
        }
    }
    let mut rhs = b_eff.clone();
    for col in 0..n {
        // partial pivot
        let mut piv = col;
        for row in (col + 1)..n {
            if m[row * n + col].abs() > m[piv * n + col].abs() {
                piv = row;
            }
        }
        if piv != col {
            for k in 0..n {
                m.swap(col * n + k, piv * n + k);
            }
            rhs.swap(col, piv);
        }
        let pivot_val = m[col * n + col];
        assert!(
            pivot_val.abs() > 1e-15,
            "(I - A) singular at DC (col {col}) — n_states={n} a_matrix={a:?}"
        );
        for row in (col + 1)..n {
            let factor = m[row * n + col] / pivot_val;
            for k in col..n {
                m[row * n + k] -= factor * m[col * n + k];
            }
            rhs[row] -= factor * rhs[col];
        }
    }
    let mut x = vec![0.0_f64; n];
    for row in (0..n).rev() {
        let mut sum = rhs[row];
        for k in (row + 1)..n {
            sum -= m[row * n + k] * x[k];
        }
        x[row] = sum / m[row * n + row];
    }

    d + (0..n).map(|i| c[i] * x[i]).sum::<f64>()
}

/// PHASE 0 golden anchor: at each of the 9 (Cutoff, Resonance) grid points,
/// the engine's compiled DC gain must match the ideal-op-amp -Zf/Zin oracle
/// to within 5% relative — NOT the wrong answer the zero-Zin injection bug
/// currently produces (measured: an UNSTABLE 1-state system, not merely a
/// stable-but-wrong gain — see `compiled_dc_gain` doc comment).
///
/// WHY THIS MATTERS (Rule 9): an MFB filter's entire reason for existing is
/// that closed-loop gain is set by a passive impedance RATIO, not by the
/// op-amp's open-loop gain. A test that merely checks "produces some output"
/// would pass on a merely-wrong-but-stable gain too. Asserting the specific
/// oracle ratio, computed exactly from the compiled matrices, is what
/// actually encodes "the impedance divider is being modeled."
#[test]
#[ignore] // GOLDEN ANCHOR (Phase 0, pedalkernel-6vy.2), STILL RED after Phase
          // B1 (pedalkernel-6vy.5): the B1 partitioner fix (feedback_summing_nodes
          // exemption in compiler/signal_flow.rs) unifies the feedback divider
          // into U1's own MNA group, and the GAIN MAGNITUDE now matches this
          // oracle to <0.01% relative at all 9 grid points (verified externally
          // with set_control_immediate — this test's plain set_control only
          // arms a smoother TARGET that a one-shot compile+read never
          // advances, so it was already unable to distinguish grid points
          // before this comment; fixed above in compiled_dc_gain, but the
          // remaining gap is real, not a harness artifact). The ONE remaining
          // discrepancy is a UNIFORM SIGN FLIP: measured is +|oracle| where
          // the oracle expects -|oracle| — reads as an output-tap polarity
          // convention (c_vector / output_pos-output_neg selection) for this
          // newly-merged rigid group, not a grouping error. Also: n_states is
          // 3 (not the hoped-for 2) with one eigenvalue sitting at EXACTLY
          // -1.0 (marginal, unit-circle boundary — see
          // mfb_lpf_current_engine_state_space_is_unstable, renamed-in-place
          // to check this instead of the old |eig|=1.337 divergence). Both
          // remaining gaps are inside build_state_space_matrices
          // (pedalkernel-rt/src/tree.rs), the exact territory
          // pedalkernel-ge7 escalated as out of scope for a narrow fix — left
          // DEFERRED, not silently patched here. Remove #[ignore] once the
          // sign convention is fixed (and ideally the parasitic -1 eigenvalue
          // resolved too, though that alone doesn't block the gain assertion).
fn mfb_lpf_dc_gain_matches_zin_oracle() {
    let Some(src) = load_mfb_lpf() else {
        eprintln!("  SKIP: pedalkernel-pro repo not found (public CI)");
        return;
    };

    let mut failures = Vec::new();
    for &((cutoff_t, res_t), expected_gain) in DC_GAIN_ORACLE {
        let cutoff = raw_from_tenths(cutoff_t);
        let resonance = raw_from_tenths(res_t);

        let measured_gain = compiled_dc_gain(&src, cutoff, resonance);
        let diff_rel = ((measured_gain - expected_gain) / expected_gain).abs();
        if diff_rel > 0.05 {
            failures.push(format!(
                "Cutoff={cutoff:.1} Resonance={resonance:.1}: expected gain {expected_gain:.6}, \
                 measured {measured_gain:.6} (relative diff {:.1}%)",
                diff_rel * 100.0
            ));
        }
    }

    assert!(
        failures.is_empty(),
        "MFB LPF DC-gain oracle mismatch at {}/{} grid points:\n{}",
        failures.len(),
        DC_GAIN_ORACLE.len(),
        failures.join("\n")
    );
}

/// UPDATED (pedalkernel-6vy.5, Approach B / Phase B1): the partitioner
/// exemption (`feedback_summing_nodes` in `compiler/signal_flow.rs`) now
/// unifies the feedback divider (Cutoff_R1, C1, R_q_floor, Resonance) into
/// U1's own group, eliminating the zero-Zin cross-stage injection bug ge7
/// diagnosed. Measured effect: n_states went from 1 (Schur-eliminated down
/// from the true 2 physical caps, C1+C2, discarding a state) to 3 (C1, C2,
/// PLUS one more from folding Rb1/Rb2/R_in/R_out into the same MNA — the
/// vref/input-coupling nodes were already pendants of this group even
/// BEFORE this fix, confirmed via `PK_DEBUG_GROUPS`-style group-boundary
/// tracing; they are not newly swallowed by this change).
///
/// This is NOT the clean "n_states=2, strictly stable" outcome Phase B1
/// originally hoped for. What's actually verified:
/// - The gain MAGNITUDE now tracks the Zin-oracle almost exactly (measured
///   vs expected agree to <0.01% relative at all 9 grid points when driven
///   via `set_control_immediate` instead of `set_control` — see the
///   `mfb_lpf_dc_gain_matches_zin_oracle` doc comment below for why
///   `set_control` alone can't observe this).
/// - The SIGN is flipped: measured is `+|oracle|`, oracle expects
///   `-|oracle|`. Uniform across the whole grid — this reads as an output
///   polarity/tap convention mismatch (e.g. `output_pos`/`output_neg`
///   selection or c_vector sign for this newly-merged rigid group), not a
///   grouping error. NOT investigated further here — out of scope for the
///   signal_flow.rs partitioner change this bead covers.
/// - Every grid point has one eigenvalue at EXACTLY -1.0 (not `> 1.0` like
///   the old bug, not `< 1.0` strictly stable either) — a marginal,
///   boundary-of-unit-circle mode, structurally identical at every (Cutoff,
///   Resonance) position tested. `build_state_space_matrices`'s own comment
///   (tree.rs ~1737) calls out "parasitic -1 eigenvalues that are difficult
///   to filter cleanly" — this looks like that same parasitic mode
///   resurfacing for this specific merged-group topology, not the acute
///   |eig|=1.337 divergence from before. Asserting strict stability
///   (`|eigenvalue| < 1.0`) would be FALSE; this test asserts the honest
///   current fact (marginal, not divergent) instead. Fixing the sign flip
///   and/or the parasitic mode is StateSpace-matrix-builder work
///   (pedalkernel-rt/src/tree.rs), the same territory pedalkernel-ge7
///   escalated as out of scope for a narrow fix — left DEFERRED, not
///   silently papered over here.
#[test]
fn mfb_lpf_current_engine_state_space_is_unstable() {
    let Some(src) = load_mfb_lpf() else {
        eprintln!("  SKIP: pedalkernel-pro repo not found (public CI)");
        return;
    };
    let pedal = parse_pedal_file(&src).expect("parse");
    let proc = compile_pedal(&pedal, SAMPLE_RATE).expect("compile");
    let ss = proc
        .stages
        .iter()
        .find_map(|s| match s {
            Stage::StateSpace(ss) => Some(ss),
            _ => None,
        })
        .expect("MFB LPF must compile a StateSpace stage");

    assert_eq!(
        ss.ss.n_states, 3,
        "expected the post-B1 3-state reduction (C1, C2, plus the folded \
         Rb1/Rb2/R_in/R_out MNA) — if this changed, re-derive this test"
    );
    let n = ss.ss.n_states;
    let a = &ss.ss.a_matrix;

    // det(A + I) == 0 iff -1 is an eigenvalue of A (direct 3x3 cofactor
    // expansion — no linear-algebra dep needed for a fixed n=3).
    let m: Vec<f64> = (0..n * n)
        .map(|k| a[k] + if k % n == k / n { 1.0 } else { 0.0 })
        .collect();
    let det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6])
        + m[2] * (m[3] * m[7] - m[4] * m[6]);
    assert!(
        det.abs() < 1e-6,
        "expected a parasitic marginal eigenvalue at exactly -1.0 (det(A+I) \
         should be ~0), measured det(A+I)={det:.3e} for a_matrix={a:?} — this is \
         a DIFFERENT symptom from the old |eig|=1.337 divergence (n_states was 1 \
         then, is 3 now after the B1 partitioner fix). If this is no longer true \
         (stage is now strictly stable, all |eigenvalue| < 1.0, or newly \
         divergent), the underlying behavior changed and this diagnostic test \
         (and the mfb_lpf_dc_gain_matches_zin_oracle ignore reason) needs \
         updating"
    );
}

/// Minimal complex-number helper for the frequency-dependence check below
/// (kept local — no need for a `num-complex` dependency for two arithmetic
/// ops on a handful of values).
#[derive(Clone, Copy)]
struct Complex {
    re: f64,
    im: f64,
}

impl Complex {
    fn real(re: f64) -> Self {
        Self { re, im: 0.0 }
    }
    fn add(self, o: Complex) -> Complex {
        Complex {
            re: self.re + o.re,
            im: self.im + o.im,
        }
    }
    fn mul(self, o: Complex) -> Complex {
        Complex {
            re: self.re * o.re - self.im * o.im,
            im: self.re * o.im + self.im * o.re,
        }
    }
    fn div(self, o: Complex) -> Complex {
        let denom = o.re * o.re + o.im * o.im;
        Complex {
            re: (self.re * o.re + self.im * o.im) / denom,
            im: (self.im * o.re - self.re * o.im) / denom,
        }
    }
    fn mag(self) -> f64 {
        (self.re * self.re + self.im * self.im).sqrt()
    }
}

fn parallel(a: Complex, b: Complex) -> Complex {
    a.mul(b).div(a.add(b))
}

/// R1 CHECKPOINT (standalone, NOT ignored): the boundary impedance at the
/// StateSpace injection node (U1.neg) is genuinely frequency-dependent, not a
/// scalar resistance, because C1 shunts to ground BETWEEN Cutoff_R1 and
/// R_q_floor+Resonance — i.e. inside the series path, not purely downstream
/// of it. This is a pure analytical fact about the netlist's topology
/// (confirmed independently via sympy: Zin(s) = R_in + Rc1 + [1/(sC1) ||
/// (Rq+Rres)]), so it is asserted directly here without depending on engine
/// behavior at all. It is the evidence backing the R1 checkpoint verdict:
/// Phase 1's scalar R_th can only ever be a DC/low-frequency approximation.
#[test]
fn zin_is_frequency_dependent_not_scalar() {
    let r_in = 10_000.0_f64;
    let rq = 10_000.0_f64;
    let c1 = 10e-9_f64;

    for &((cutoff_t, res_t), _) in DC_GAIN_ORACLE {
        let rc1 = pot_ohms(raw_from_tenths(cutoff_t), POT_RANGE, POT_MAX_R);
        let rres = pot_ohms(raw_from_tenths(res_t), POT_RANGE, POT_MAX_R);
        let r_series_fb = Complex::real(rq + rres);

        // Zin(DC): C1 is an open circuit -> parallel branch reduces to
        // r_series_fb alone.
        let zin_dc = r_in + rc1 + r_series_fb.re;

        // Zin(20kHz): C1's reactance (1/(jwC1)) is small enough to noticeably
        // shunt the series-feedback branch.
        let w = 2.0 * std::f64::consts::PI * 20_000.0;
        let zc1 = Complex {
            re: 0.0,
            im: -1.0 / (w * c1),
        };
        let par = parallel(zc1, r_series_fb);
        let zin_20k = Complex::real(r_in + rc1).add(par);
        let zin_20k_mag = zin_20k.mag();

        let ratio = zin_20k_mag / zin_dc;
        assert!(
            (ratio - 1.0).abs() > 0.05,
            "Cutoff_raw={} Resonance_raw={}: Zin(DC)={zin_dc:.0} Zin(20kHz)={zin_20k_mag:.0} \
             ratio={ratio:.4} — expected a NON-scalar (frequency-dependent) boundary impedance, \
             but this position looks resistance-like (ratio within 5% of 1.0)",
            raw_from_tenths(cutoff_t),
            raw_from_tenths(res_t),
        );
    }
}
