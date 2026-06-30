//! SPICE validation for the Neve 1073 — 500-series target (public spice / private .pedal).
//!
//! First milestone: the **BA283 discrete Class-A gain block** in isolation
//! (`pedals/500/1073/sub/ba283.pedal`), validated 1:1 against an ngspice golden.
//! The full mic preamp (with input/output transformers) and the program EQ are
//! layered on once this clean-signal block matches — see the 1073 README in the
//! pro repo.
//!
//! # Architecture (house pattern, mirrors `legends_spice.rs`)
//!
//! The `.pedal` lives in the **private** `pedalkernel-pro` repo and is loaded at
//! runtime via [`load_pro_pedal_sub`]; the test SKIPS (counts as pass) on CI when
//! the pro repo is absent. The ngspice deck is **public**, here in this crate.
//!
//! # Golden bootstrap
//!
//! ```
//! PEDALKERNEL_BOOTSTRAP_1073=1 cargo test -p pedalkernel-validate \
//!   --test neve1073_spice bootstrap_ba283_golden -- --nocapture
//! ```
//!
//! The golden is derived from the SPICE deck, NOT from WDF output, so the two
//! remain independent references.

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use pedalkernel_validate::npy;
use pedalkernel_validate::pro_pedal::load_pro_pedal_sub;
use pedalkernel_validate::spice::{SpiceConfig, SpiceRunner};
use pedalkernel_validate::{metrics, skip_if_missing};
use std::f64::consts::PI;
use std::path::PathBuf;

const SR: f64 = 48_000.0;
const OVERSAMPLE: u32 = 4;
const SINE_AMP: f64 = 0.1;
const TEST_FREQ: f64 = 1_000.0;

/// Steady-state settle window.  The BA283 has TWO 10 µF coupling caps (Cin and
/// Cout into a 10 k load → RC ≈ 100 ms), so the AC-coupled output needs ~10 τ ≈
/// 1 s to settle.  The previous golden was bootstrapped over only 0.1 s, so it
/// was measured mid-charge (golden mean ≈ +5 V instead of ≈ 0).
const SETTLE_S: f64 = 1.0;
/// Measurement window AFTER settling — the only span stored / compared.
const MEASURE_S: f64 = 0.1;

const PRO_PATH: &str = "pedals/500/1073/sub/ba283.pedal";
const DECK: &str = "neve1073_ba283";
const GOLDEN: &str = "neve1073_ba283";

fn spice_path(name: &str) -> PathBuf {
    let manifest = env!("CARGO_MANIFEST_DIR");
    PathBuf::from(format!("{manifest}/spice-circuits/active/{name}.spice"))
}

fn golden_dir(name: &str) -> PathBuf {
    let manifest = env!("CARGO_MANIFEST_DIR");
    PathBuf::from(format!("{manifest}/golden/active/{name}"))
}

/// Build the internal-rate (SR × OVERSAMPLE) input sine over `settle_s + MEASURE_S`.
fn internal_input(settle_s: f64) -> (SpiceRunner, Vec<f64>) {
    let config = SpiceConfig {
        sample_rate: SR as u32,
        oversample: OVERSAMPLE,
    };
    let internal_rate = config.internal_rate();
    let n = ((settle_s + MEASURE_S) * internal_rate) as usize;
    let input: Vec<f64> = (0..n)
        .map(|i| SINE_AMP * (2.0 * PI * TEST_FREQ * i as f64 / internal_rate).sin())
        .collect();
    (SpiceRunner::new(config), input)
}

/// Generate the SETTLED, base-rate (48 kHz) measurement window via ngspice.
fn generate_settled_golden(settle_s: f64) -> Vec<f64> {
    let circuit = spice_path(DECK);
    assert!(circuit.exists(), "{DECK}.spice not found at {circuit:?}");
    let (runner, input_internal) = internal_input(settle_s);
    runner
        .simulate_settled_window(&circuit, &input_internal, "v_out", settle_s)
        .unwrap_or_else(|e| panic!("ngspice simulation failed for {GOLDEN} bootstrap: {e}"))
}

/// Simulate the ngspice deck SETTLED and write a 48 kHz measurement-window golden.
///
/// PROVES (printed + asserted): the stored golden is at 48 kHz (len == MEASURE_S
/// × 48000, 48 samples/period at 1 kHz), its mean ≈ 0 (the AC-coupled output has
/// settled — no DC charge-up ramp), and the single-bin AC level is the same when
/// the settle is extended (steady state, not a transient artifact).
#[test]
fn bootstrap_ba283_golden() {
    if std::env::var("PEDALKERNEL_BOOTSTRAP_1073").as_deref() != Ok("1") {
        eprintln!("  SKIP bootstrap_ba283_golden: set PEDALKERNEL_BOOTSTRAP_1073=1 to run");
        return;
    }

    let golden = generate_settled_golden(SETTLE_S);

    // (i) Rate proof: base-rate 48 kHz, not the 192 kHz internal rate.
    let expected = (MEASURE_S * SR) as usize;
    let samples_per_period = SR / TEST_FREQ;
    eprintln!("  bootstrapped {GOLDEN} SETTLED golden:");
    eprintln!(
        "    len = {} (expected ≈ {expected} = {MEASURE_S}s × 48000); {samples_per_period} samples/period @ {TEST_FREQ} Hz",
        golden.len()
    );
    assert!(
        golden.len().abs_diff(expected) <= OVERSAMPLE as usize,
        "golden must be at 48 kHz base rate: len {} vs expected {expected}",
        golden.len()
    );

    // (ii) Settle proof: AC-coupled output mean ≈ 0 (was ≈ +5 V un-settled).
    let mean = metrics::dc_offset(&golden);
    let amp = metrics::single_bin_amplitude(&golden, TEST_FREQ, SR);
    let drift = metrics::ac_amplitude_drift_db(&golden, TEST_FREQ, SR);
    eprintln!("    mean = {mean:+.5} V   AC amp = {amp:.5} V   half-to-half drift = {drift:+.3} dB");
    assert!(
        mean.abs() < 0.05 * amp.max(1e-9) + 1e-3,
        "golden tail mean {mean:+.5} V is not ≈0 — output not settled (AC amp {amp:.5})"
    );
    assert!(drift.abs() < 0.5, "golden AC level still drifting ({drift:+.3} dB) — extend settle");

    // (iii) Stability proof: regenerate with 50% more settle; AC level must not move.
    let golden_long = generate_settled_golden(SETTLE_S * 1.5);
    let amp_long = metrics::single_bin_amplitude(&golden_long, TEST_FREQ, SR);
    let settle_delta = 20.0 * (amp_long / amp).log10();
    eprintln!(
        "    AC amp @1.5× settle = {amp_long:.5} V  → Δ = {settle_delta:+.3} dB (must be ≈0: steady state)"
    );
    assert!(
        settle_delta.abs() < 0.3,
        "AC level changed {settle_delta:+.3} dB when settle extended — still transient"
    );

    let dir = golden_dir(GOLDEN);
    std::fs::create_dir_all(&dir).unwrap_or_else(|e| panic!("create golden/active/{GOLDEN}: {e}"));
    let golden_path = dir.join("sine_1k.npy");
    npy::write_f64(&golden_path, &golden)
        .unwrap_or_else(|e| panic!("write {GOLDEN} golden: {e}"));
    eprintln!("    wrote {} samples → {golden_path:?}", golden.len());
}

/// Run the WDF BA283 at the BASE 48 kHz rate: warm up `settle_s`, then record a
/// `MEASURE_S` window.  Returns the settled measurement window — same rate, same
/// length, same phase as the golden.
fn wdf_settled_window(proc: &mut impl PedalProcessor, settle_s: f64) -> Vec<f64> {
    let settle_n = (settle_s * SR) as usize;
    let measure_n = (MEASURE_S * SR) as usize;
    for i in 0..settle_n {
        proc.process(SINE_AMP * (2.0 * PI * TEST_FREQ * i as f64 / SR).sin());
    }
    (0..measure_n)
        .map(|j| {
            let i = settle_n + j;
            proc.process(SINE_AMP * (2.0 * PI * TEST_FREQ * i as f64 / SR).sin())
        })
        .collect()
}

/// Compare the WDF BA283 block against the SETTLED ngspice golden (GAIN = 0.5).
///
/// Both signals are at 48 kHz, settled, same length/phase.  The trustworthy
/// number is the single-bin AC gain ratio at the test frequency — NOT
/// `normalized_rms_error_db`, which degenerates to ~0 dB whenever the WDF is far
/// below the golden.
#[test]
fn ba283_wdf_vs_spice() {
    let source = skip_if_missing!(load_pro_pedal_sub(PRO_PATH), PRO_PATH);

    let golden_path = golden_dir(GOLDEN).join("sine_1k.npy");
    if !golden_path.exists() {
        eprintln!(
            "  SKIP ba283_wdf_vs_spice: golden not found at {golden_path:?}; \
             run with PEDALKERNEL_BOOTSTRAP_1073=1 first"
        );
        return;
    }
    let golden = npy::read_f64(&golden_path).expect("read golden");

    let def = parse_pedal_file(&source).unwrap_or_else(|e| panic!("parse {PRO_PATH}: {e}"));
    let mut proc = compile_pedal(&def, SR).unwrap_or_else(|e| panic!("compile {PRO_PATH}: {e}"));
    proc.set_control("Gain", 0.5);

    let wdf = wdf_settled_window(&mut proc, SETTLE_S);
    let n = wdf.len().min(golden.len());
    let wdf = &wdf[..n];
    let golden_w = &golden[..n];

    // Settle guards on BOTH sides — an AC-coupled output that has settled has
    // mean ≈ 0 and ~0 dB half-to-half drift.
    let wdf_mean = metrics::dc_offset(wdf);
    let gold_mean = metrics::dc_offset(golden_w);
    let wdf_amp = metrics::single_bin_amplitude(wdf, TEST_FREQ, SR);
    let gold_amp = metrics::single_bin_amplitude(golden_w, TEST_FREQ, SR);
    let wdf_drift = metrics::ac_amplitude_drift_db(wdf, TEST_FREQ, SR);

    // THE steady-state, level-independent number.
    let ac_gain = metrics::ac_gain_db(wdf, golden_w, TEST_FREQ, SR);
    let nrms = metrics::normalized_rms_error_db(wdf, golden_w);

    eprintln!("  {GOLDEN} WDF vs ngspice — SETTLED, both 48 kHz, {n} samples:");
    eprintln!("    golden  : mean {gold_mean:+.5} V   AC amp {gold_amp:.5} V");
    eprintln!("    wdf     : mean {wdf_mean:+.5} V   AC amp {wdf_amp:.5} V   drift {wdf_drift:+.3} dB");
    eprintln!("    >>> single-bin AC gain (WDF/golden) = {ac_gain:+.2} dB  <<<  (steady-state, level-independent)");
    eprintln!("    normalized_rms_error = {nrms:+.2} dB  (DEGENERATE shape proxy — do not gate on this)");

    // Guard: a too-short settle window can never silently pass again.
    assert!(
        gold_mean.abs() < 0.05 * gold_amp.max(1e-9) + 1e-3,
        "golden not settled (mean {gold_mean:+.5} V vs AC amp {gold_amp:.5}); regenerate with PEDALKERNEL_BOOTSTRAP_1073=1"
    );
    assert!(
        wdf_mean.abs() < 0.05 * wdf_amp.max(1e-9) + 1e-3 && wdf_drift.abs() < 0.5,
        "WDF output not settled (mean {wdf_mean:+.5} V, drift {wdf_drift:+.3} dB) — extend SETTLE_S"
    );
    assert!(
        wdf_amp > 1e-6,
        "{GOLDEN} WDF output is silence (AC amp < 1e-6): compilation or processing failed"
    );

    // WDF-side steady-state stability: doubling the settle must not move the AC
    // level (proves the measured gain is steady state, not a transient sample).
    let mut proc2 = compile_pedal(&def, SR).unwrap();
    proc2.set_control("Gain", 0.5);
    let wdf_long = wdf_settled_window(&mut proc2, SETTLE_S * 2.0);
    let amp_long = metrics::single_bin_amplitude(&wdf_long, TEST_FREQ, SR);
    let stab = 20.0 * (amp_long / wdf_amp.max(1e-30)).log10();
    eprintln!("    WDF AC amp @2× settle = {amp_long:.5} V → Δ {stab:+.3} dB (steady-state check)");
    assert!(stab.abs() < 0.3, "WDF AC level moved {stab:+.3} dB on 2× settle — not steady state");

    // No hard accuracy gate yet — the BA283 has a known large gain deficit
    // (the adapted-input-port-impedance investigation). The point here is that
    // the AC gain number is now a TRUSTWORTHY steady-state figure.
}

/// PRIMARY DELIVERABLE (pedalkernel-9u6u.2 Phase B): the BA283 grouped-NR
/// transconductance diagnosis.
///
/// Drives the BA283 with the golden small sine and, after each metering block,
/// reads every BJT's live operating point (`Vbe`/`Vce`/`Ic`/`Ib`/`gm`) straight
/// out of the MultiNl stage via the Phase-B [`MultiNlStage::runtime_op_points`]
/// capture. Reports, and ASSERTS, the two things the static MNA snapshot cannot
/// show:
///
///  1. The grouped NR **converges** every block (low iters, small residual).
///  2. The transconductance **is applied**: across the signal swing the realized
///     `dIc ≈ gm·dVbe` for the conducting devices (ratio ≈ 1).
///
/// The diagnostic finding (logged, not asserted, since it is the bug under
/// investigation): the INPUT transistor TR1 settles near cutoff at runtime
/// (`Vbe ≈ 0.36 V`, `Ic ≈ 7 µA`, realized `gm ≈ 2.7e-4 S` — ~120× below the
/// design 0.033 S the static `cross_gm` shows). The −49.5 dB gain deficit is a
/// DC **operating-point** problem (TR1 starved), NOT a failure to apply the
/// transconductance in the solve.
///
/// Run: `cargo test -p pedalkernel-validate --features diag --test neve1073_spice
///       ba283_runtime_nr -- --nocapture`
#[cfg(feature = "diag")]
#[test]
fn ba283_runtime_nr() {
    use pedalkernel_rt::diag_ring::OpPointRecord;
    use pedalkernel_rt::processor::Stage;

    let source = skip_if_missing!(load_pro_pedal_sub(PRO_PATH), PRO_PATH);
    let def = parse_pedal_file(&source).unwrap_or_else(|e| panic!("parse {PRO_PATH}: {e}"));
    let mut proc = compile_pedal(&def, SR).unwrap_or_else(|e| panic!("compile {PRO_PATH}: {e}"));
    proc.set_control("Gain", 0.5);

    // Per-block sampling: drive `nblocks` blocks of `block` samples; after each
    // block read each BJT's operating point. NR stats come from the thread-local
    // grouped-NR snapshot reset each block.
    const BLOCK: usize = 128;
    const NBLOCKS: usize = 48;
    let warmup = 2 * BLOCK; // let the DC ramp + coupling caps settle.
    for i in 0..warmup {
        proc.process(SINE_AMP * (2.0 * PI * TEST_FREQ * i as f64 / SR).sin());
    }

    // Collect per-block op-point records keyed by group index.
    let mut series: Vec<Vec<OpPointRecord>> = Vec::new();
    let mut in_peak = 0.0f64;
    let mut out_peak = 0.0f64;
    let mut max_iters = 0u32;
    let mut max_resid = 0.0f64;
    let mut min_solves = u64::MAX;
    let mut sample = warmup;
    for _b in 0..NBLOCKS {
        pedalkernel_rt::elements::nonlinear::solver::reset_solver_stats();
        for _ in 0..BLOCK {
            let x = SINE_AMP * (2.0 * PI * TEST_FREQ * sample as f64 / SR).sin();
            let y = proc.process(x);
            in_peak = in_peak.max(x.abs());
            out_peak = out_peak.max((y as f64).abs());
            sample += 1;
        }
        let ss = pedalkernel_rt::elements::nonlinear::solver::solver_stats_snapshot();
        max_iters = max_iters.max(ss.max_iterations);
        max_resid = max_resid.max(ss.max_residual as f64);
        if ss.solves > 0 {
            min_solves = min_solves.min(ss.solves);
        }

        let mut recs = [OpPointRecord::default(); 8];
        let mut n = 0usize;
        for (si, st) in proc.stages.iter().enumerate() {
            if let Stage::MultiNl(mnl) = st {
                n += mnl.runtime_op_points(si, &mut recs[n..]);
            }
        }
        series.push(recs[..n].to_vec());
    }

    assert!(!series.is_empty() && !series[0].is_empty(), "no MultiNl op-points captured");
    let ngroups = series[0].len();

    eprintln!("\n  ═══ BA283 grouped-NR runtime diagnosis (Phase B) ═══");
    eprintln!(
        "  NR convergence over {NBLOCKS} blocks: max_iters={max_iters}, max_resid={max_resid:.3e}, min_solves/block={min_solves}"
    );
    // (1) Convergence health.
    assert!(max_iters < 40, "grouped NR not converging (max_iters={max_iters})");
    assert!(max_resid < 1e-2, "grouped NR residual too high ({max_resid:.3e})");

    // Per-device identity threaded from the compiler (ref + model type), so the
    // op-point is attributed to the PHYSICAL transistor — never a guessed group
    // index. This is the pedalkernel-3hsi fix that resolves the ej0v/kzla
    // port→transistor mapping ambiguity.
    eprintln!("\n  device identity (compiler-threaded ref/type, group order):");
    for g in 0..ngroups {
        let r = &series[0][g];
        eprintln!(
            "    group {g} = {:<6} ({:<8})  [build/port order — NO guessing]",
            if r.ref_str().is_empty() { "?" } else { r.ref_str() },
            if r.type_str().is_empty() { "?" } else { r.type_str() },
        );
    }

    // Per-group transconductance behavior across the swing.
    eprintln!("\n  per-BJT operating-point swing + transconductance check:");
    eprintln!(
        "  {:>10} {:>9} {:>9} {:>11} {:>11} {:>11} {:>11}",
        "ref/type", "Vbe~", "Ic~", "gm~", "dVbe", "dIc", "dIc/(gm·dVbe)"
    );
    for g in 0..ngroups {
        // Find the two blocks with the largest Vbe spread for this group.
        let (mut lo, mut hi) = (0usize, 0usize);
        for b in 1..series.len() {
            if series[b].len() <= g {
                continue;
            }
            if series[b][g].v_be < series[lo][g].v_be {
                lo = b;
            }
            if series[b][g].v_be > series[hi][g].v_be {
                hi = b;
            }
        }
        let a = &series[lo][g];
        let c = &series[hi][g];
        let dvbe = (c.v_be - a.v_be) as f64;
        let dic = (c.i_c - a.i_c) as f64;
        let gm_avg = ((a.gm + c.gm) / 2.0) as f64;
        let ic_avg = ((a.i_c + c.i_c) / 2.0) as f64;
        let ratio = if gm_avg * dvbe != 0.0 {
            dic / (gm_avg * dvbe)
        } else {
            f64::NAN
        };
        let ident = if a.ref_str().is_empty() {
            format!("g{g}")
        } else {
            format!("{}/{}", a.ref_str(), a.type_str())
        };
        eprintln!(
            "  {ident:>10} {:>9.4} {:>9.3e} {:>9.5} {:>11.3e} {:>11.3e} {:>13.3}",
            (a.v_be + c.v_be) / 2.0,
            ic_avg,
            gm_avg,
            dvbe,
            dic,
            ratio
        );
        // (2) Transconductance IS applied: for any group with a meaningful swing,
        // dIc tracks gm·dVbe (ratio ≈ 1). This is the core BA283 verdict.
        if dvbe.abs() > 1e-4 && gm_avg.abs() > 1e-6 {
            assert!(
                (ratio - 1.0).abs() < 0.35,
                "group {g}: dIc does not track gm·dVbe (ratio={ratio:.3}) — transconductance NOT applied"
            );
        }
    }

    // Realized chain gain over the window (the −49.5 dB level).
    let realized = if in_peak > 1e-12 { out_peak / in_peak } else { 0.0 };
    eprintln!(
        "\n  realized peak chain gain over window: {realized:.4}x ({:+.2} dB)  [design ~97x]",
        if realized > 0.0 { 20.0 * realized.log10() } else { f64::NEG_INFINITY }
    );

    // Diagnostic (logged, not asserted): flag any input-side BJT starved near
    // cutoff — the operating-point bug behind the gain deficit.
    let last = series.last().unwrap();
    for (g, r) in last.iter().enumerate() {
        if (r.v_be as f64) < 0.5 && (r.i_c as f64) < 1e-4 {
            eprintln!(
                "  ⚠ group {g} STARVED near cutoff at runtime: Vbe={:.3}V Ic={:.2e}A gm={:.5}S \
                 (a conducting Si BJT needs Vbe≈0.65V) → this is the gain deficit's DC root cause",
                r.v_be, r.i_c, r.gm
            );
        }
    }

    assert!(out_peak > 1e-9, "BA283 output is silence");
}

/// Prove the skip mechanism works when the pro repo is absent.
#[test]
fn ba283_skip_when_pro_absent() {
    let source = skip_if_missing!(
        load_pro_pedal_sub("pedals/500/1073/__nonexistent_ba283__.pedal"),
        "pedals/500/1073/__nonexistent_ba283__.pedal"
    );
    let _ = source;
}
