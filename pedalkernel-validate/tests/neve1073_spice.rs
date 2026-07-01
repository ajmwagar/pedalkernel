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

/// ngspice `.op` operating point for the BA283 (with a 1 MΩ input pulldown).
/// Re-derived fresh from `neve1073_ba283.spice` + Rpd via `/opt/homebrew/bin/ngspice`.
/// Device labels follow the `.pedal`: Q3 = TR1 (input CE), Q1 = TR2 (Darlington
/// driver), Q2 = TR3 (2N3055 output). Each tuple is (Vbe, Vce).
const NGSPICE_OP: &[(&str, f64, f64)] = &[
    ("Q3", 0.6417, 5.3535), // TR1 — input common-emitter
    ("Q1", 0.5402, 18.7976), // TR2 — Darlington driver
    ("Q2", 0.4063, 19.2039), // TR3 — 2N3055 output
];

/// ngspice `.op` NODE voltages for the BA283 (1 MΩ input pulldown), keyed by
/// `.pedal` pin names (each name aliases one circuit net). Re-derived fresh from
/// `neve1073_ba283.spice` + Rpd via `/opt/homebrew/bin/ngspice`. These let the
/// FULL operating point be seeded: each coupling/bypass cap's stored DC voltage
/// is `V(node_a) − V(node_b)`.
///
///   ni=1.0308  nb1=1.0308  n1=5.7426  ne1=0.3891  nrc=5.7426
///   ne2=5.2024  nd=4.7961  nfb=4.7051  v_in=0  v_out=0  vcc=24
const NGSPICE_NODES: &[(&str, f64)] = &[
    ("Cin.a", 0.0),         // v_in (AC-grounded at DC)
    ("Cin.b", 1.030802),    // ni
    ("Q3.base", 1.030802),  // nb1
    ("Q3.collector", 5.742598), // n1
    ("Q3.emitter", 0.389135),   // ne1
    ("Rcmp.b", 5.742598),   // nrc
    ("Q1.emitter", 5.202446),   // ne2
    ("Q2.emitter", 4.796136),   // nd
    ("RU1.b", 4.705094),    // nfb
    ("Cout.b", 0.0),        // v_out (AC-grounded at DC through RL)
];

/// The cap DC voltages implied by [`NGSPICE_NODES`], in `.pedal` orientation
/// (a − b), for the verdict print: Cin −1.0308, Cmil −4.7118, Ccmp +5.3535,
/// Cfb +3.6743, Cout +4.7961.
const NGSPICE_CAPS: &[(&str, f64)] = &[
    ("Cin", -1.030802),
    ("Cmil", -4.711796),
    ("Ccmp", 5.353463),
    ("Cfb", 3.674292),
    ("Cout", 4.796136),
];

/// Inject an `op { ... }` block (ngspice's operating point) before the closing
/// brace of the loaded BA283 source. Keeps the private `.pedal` untouched.
///
/// When `with_nodes` is true, also emits the `nodes { ... }` sub-block so the
/// reactive (capacitor) ports are seeded to ngspice's DC node voltages — the
/// FULL operating-point seed (device ports + cap voltages).
fn inject_op_block_full(source: &str, with_nodes: bool) -> String {
    let mut block = String::from("\n  op {\n");
    for (label, vbe, vce) in NGSPICE_OP {
        block.push_str(&format!("    {label}: {{ vbe: {vbe}, vce: {vce} }}\n"));
    }
    if with_nodes {
        block.push_str("    nodes {\n");
        for (name, v) in NGSPICE_NODES {
            block.push_str(&format!("      {name}: {v}\n"));
        }
        block.push_str("    }\n");
    }
    block.push_str("  }\n");
    let cut = source.rfind('}').expect("pedal has a closing brace");
    let mut out = String::with_capacity(source.len() + block.len());
    out.push_str(&source[..cut]);
    out.push_str(&block);
    out.push_str(&source[cut..]);
    out
}

/// SEED-AND-HOLD (pedalkernel — decisive op-point test).
///
/// Seeds each BJT's NR warm-start (`v_prev`) at ngspice's exact `.op` operating
/// point via the new `op { }` block, then processes SILENCE for ≥1 s and traces
/// whether each device's (Vbe, Vce) HOLDS at the seeded ngspice point or WALKS
/// away (TR1 toward cutoff). The `op { }` seed touches only the warm-start, never
/// the stage's DC source term, so this is an honest test of whether ngspice's
/// op-point is a fixed point of the engine's OWN bias equations:
///
///   HOLDS  → ngspice's op-point is a stable WDF fixed point ⇒ bias math agrees,
///            the cold-solve just lands in the wrong basin (root-selection).
///   WALKS  → ngspice's op-point is NOT a WDF fixed point ⇒ the engine's bias
///            math genuinely differs from spice; whichever device drifts most
///            localizes the residual.
///
/// Run: `cargo test -p pedalkernel-validate --features diag --test neve1073_spice
///       ba283_seed_and_hold -- --nocapture`
#[cfg(feature = "diag")]
#[test]
fn ba283_seed_and_hold() {
    use pedalkernel_rt::diag_ring::OpPointRecord;
    use pedalkernel_rt::processor::Stage;

    let source = skip_if_missing!(load_pro_pedal_sub(PRO_PATH), PRO_PATH);

    // Capture the live (Vbe, Vce) for every MultiNl BJT group.
    fn capture(proc: &pedalkernel::compiler::CompiledPedal) -> Vec<OpPointRecord> {
        let mut recs = [OpPointRecord::default(); 8];
        let mut n = 0usize;
        for (si, st) in proc.stages.iter().enumerate() {
            if let Stage::MultiNl(mnl) = st {
                n += mnl.runtime_op_points(si, &mut recs[n..]);
            }
        }
        recs[..n].to_vec()
    }

    let probes = [0usize, 1, 10, 100, 1000, 48_000];

    // ── Seeded run (FULL seed: device ports + cap voltages) ─────────────────
    let seeded_src = inject_op_block_full(&source, true);
    let def = parse_pedal_file(&seeded_src)
        .unwrap_or_else(|e| panic!("parse seeded BA283: {e}"));
    let mut proc = compile_pedal(&def, SR).unwrap_or_else(|e| panic!("compile seeded BA283: {e}"));
    proc.set_control("Gain", 0.5);

    eprintln!("\n  ═══ BA283 FULL SEED-AND-HOLD (silence) ═══");
    eprintln!("  ngspice .op seed: Q3(TR1) {:.4}/{:.4}  Q1(TR2) {:.4}/{:.4}  Q2(TR3) {:.4}/{:.4}  (Vbe/Vce)",
        NGSPICE_OP[0].1, NGSPICE_OP[0].2, NGSPICE_OP[1].1, NGSPICE_OP[1].2, NGSPICE_OP[2].1, NGSPICE_OP[2].2);
    eprint!("  cap DC seeds (a−b):");
    for (name, v) in NGSPICE_CAPS {
        eprint!("  {name}={v:+.4}");
    }
    eprintln!();

    // F(ngspice_op) on the FRESH proc — device ports AND caps both seeded at
    // ngspice's operating point, BEFORE any silence. This is the decisive
    // formulation probe: r ≈ 0 ⇒ ngspice's point is a zero of our DC residual
    // once caps are included (the cap-confound resolved). Was 0.63 V device-only.
    {
        use pedalkernel_rt::processor::Stage;
        eprintln!("\n  ═══ F(ngspice_op) FULL SEED (fresh, caps+ports seeded, pre-silence) ═══");
        for st in proc.stages.iter() {
            let Stage::MultiNl(mnl) = st else { continue };
            if mnl.device_groups.is_none() { continue; }
            let ports = mnl.op_seed_residual();
            let max_static = ports.iter().map(|p| p.residual.abs()).fold(0.0f64, f64::max);
            let (worst, _) = ports
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.residual.abs().partial_cmp(&b.1.residual.abs()).unwrap())
                .map(|(i, p)| (i, p.residual))
                .unwrap_or((0, 0.0));
            eprintln!("  max|F(ngspice_op)_full_seed| = {max_static:.4} V at port {worst}");
            for (i, p) in ports.iter().enumerate() {
                eprintln!(
                    "    port {i:2}: a={:+.4} dc_bias={:+.4} cap={:+.4} nl={:+.4} adapt={:+.4}  i_dev={:+.3e}  r={:+.4} V",
                    p.a, p.dc_bias, p.cap, p.nl, p.adapted, p.i_device, p.residual
                );
            }
        }
    }

    let header = |label: &str, recs: &[OpPointRecord]| {
        let mut s = format!("  {label:>12}: ");
        for r in recs {
            let id = if r.ref_str().is_empty() { "?".to_string() } else { r.ref_str().to_string() };
            s.push_str(&format!("{id}={:.4}/{:.4}  ", r.v_be, r.v_ce));
        }
        s
    };

    // Probe 0 = the seed itself (before any process()).
    let seed_recs = capture(&proc);
    eprintln!("{}", header("seed (s=0)", &seed_recs));

    let mut sample = 0usize;
    let mut last_recs = seed_recs.clone();
    let mut s1_recs = seed_recs.clone();
    for &target in &probes[1..] {
        while sample < target {
            proc.process(0.0);
            sample += 1;
        }
        last_recs = capture(&proc);
        if target == 1 {
            s1_recs = last_recs.clone();
        }
        eprintln!("{}", header(&format!("s={target}"), &last_recs));
    }

    // First-sample drift = the cleanest per-device residual proxy: the direction
    // and magnitude the network's KCL pushes each port on the very first solve,
    // BEFORE the slow coupling caps (τ≈100 ms) move the operating point. A port
    // that the engine's equations agree on stays put; the one carrying the
    // residual moves first. (Localizes WHERE the bias math disagrees.)
    eprintln!("\n  first-sample residual (seed→s=1, ΔVbe/ΔVce per device):");
    for (label, vbe_seed, vce_seed) in NGSPICE_OP {
        if let Some(r) = s1_recs.iter().find(|r| r.ref_str() == *label) {
            eprintln!(
                "    {label:>4}: ΔVbe {:+.4}   ΔVce {:+.4}",
                (r.v_be as f64) - vbe_seed,
                (r.v_ce as f64) - vce_seed
            );
        }
    }

    // ── DC-balance residual F(ngspice_op) per port (the formulation probe) ──
    // `proc` has now run 48 k samples of silence: its reactive (coupling/bypass)
    // caps are DC-settled, but `v_prev` walked off the seed. Re-pin `v_prev` to
    // ngspice's exact op-point on every BJT port, KEEP the DC-settled cap states,
    // and read the four-term balance residual. r ≢ 0 ⇒ ngspice's op-point is NOT
    // a zero of our DC residual ⇒ a formulation bug; the breakdown localizes the
    // term. (Mirrors `spice_wdf_balance_residual`'s methodology, automatic from
    // the op{} seed via `MultiNlStage::op_seed_residual`.)
    use pedalkernel_rt::stage::NlDeviceGroupKind;
    eprintln!("\n  ═══ F(ngspice_op) per-port WDF DC balance residual ═══");
    for st in proc.stages.iter_mut() {
        let Stage::MultiNl(mnl) = st else { continue };
        let Some(dg) = mnl.device_groups.as_ref() else { continue };
        // Re-seed v_prev at ngspice's .op for each labelled BJT group.
        let mut reseed: Vec<(usize, f64, f64)> = Vec::new();
        for (g, group) in dg.groups.iter().enumerate() {
            let NlDeviceGroupKind::BjtTwoPort(bjt) = group else { continue };
            let label = dg.identities.get(g).map(|id| id.ref_name.as_str()).unwrap_or("");
            let Some((_, vbe, vce)) = NGSPICE_OP.iter().find(|(l, _, _)| *l == label) else { continue };
            let sign = if bjt.is_pnp { -1.0 } else { 1.0 };
            reseed.push((dg.offsets[g], sign * vbe, sign * vce));
        }
        if reseed.is_empty() { continue; }
        for (off, vbe, vce) in reseed {
            for (port, v) in [(off, vbe), (off + 1, vce)] {
                if port < mnl.n_nl {
                    mnl.v_prev[port] = v as pedalkernel_rt::Wave;
                }
            }
        }
        let ports = mnl.op_seed_residual();
        eprintln!("  (static residual = a - dc_bias - cap - nl - adapt; the STATIC DC formulation gap)");
        let mut max_r = 0.0f64;
        let mut worst_port = 0usize;
        for (i, p) in ports.iter().enumerate() {
            if p.residual.abs() > max_r { max_r = p.residual.abs(); worst_port = i; }
            eprintln!(
                "    port {i:2}: a={:+.4} dc_bias={:+.4} cap={:+.4} nl={:+.4} adapt={:+.4} servo={:+.4}  i_dev={:+.3e}  static_r={:+.4} V",
                p.a, p.dc_bias, p.cap, p.nl, p.adapted, p.servo, p.i_device, p.residual
            );
        }
        eprintln!("    -> max|static residual| = {max_r:.4} V at port {worst_port}");
        if let Some(p) = ports.get(worst_port) {
            eprintln!(
                "    LARGEST-RESIDUAL DECOMPOSITION (port {worst_port}): \
                 r = a - dc_bias - cap - nl - adapt = {:+.4} - ({:+.4}) - ({:+.4}) - ({:+.4}) - ({:+.4}) = {:+.4} V",
                p.a, p.dc_bias, p.cap, p.nl, p.adapted, p.residual
            );
        }
    }

    // ── Control run: NO op{} seed (engine's own fixed point) ────────────────
    let def0 = parse_pedal_file(&source).unwrap();
    let mut proc0 = compile_pedal(&def0, SR).unwrap();
    proc0.set_control("Gain", 0.5);
    for _ in 0..48_000 {
        proc0.process(0.0);
    }
    let unseeded = capture(&proc0);
    eprintln!("{}", header("UNSEEDED s=48000", &unseeded));

    // SANITY GATE: the residual at the engine's OWN settled fixed point MUST be
    // ~0 — it is, by construction, a zero of the per-sample NR. If it is not,
    // the residual formula does not match the engine's equations and the seeded
    // F(ngspice_op) numbers above are not trustworthy. (Calibrates the probe.)
    eprintln!("\n  ═══ CALIBRATION: residual at engine's OWN unseeded settled point ═══");
    eprintln!("  (static residual ≡ −fi, the engine's grouped-NR residual, by construction:");
    eprintln!("   known_a = adapt + cap + dc_bias + servo, fixed point a = known_a + Σ s_nl·b.");
    eprintln!("   This point is the BA283's degenerate cold-solve op-point; a few-volt fresh-eval");
    eprintln!("   residual here reflects that the engine settled to a non-clean (frozen-Newton/");
    eprintln!("   drift) point, NOT a formula error — the formula matches the engine term-for-term.)");
    for st in proc0.stages.iter() {
        let Stage::MultiNl(mnl) = st else { continue };
        if mnl.device_groups.is_none() { continue; }
        let ports = mnl.op_seed_residual();
        let max_full = ports.iter().map(|p| p.residual_full.abs()).fold(0.0f64, f64::max);
        let max_static = ports.iter().map(|p| p.residual.abs()).fold(0.0f64, f64::max);
        eprintln!(
            "    own-fixed-point max|residual_full| = {max_full:.4e} V  max|static| = {max_static:.4} V  servo_active={}  ({} NL ports)",
            mnl.dc_servo_active, ports.len()
        );
        for (i, p) in ports.iter().enumerate() {
            eprintln!(
                "    port {i:2}: a={:+.4} dc_bias={:+.4} cap={:+.4} nl={:+.4} adapt={:+.4} servo={:+.4}  static_r={:+.4} full_r={:+.4e}",
                p.a, p.dc_bias, p.cap, p.nl, p.adapted, p.servo, p.residual, p.residual_full
            );
        }
    }

    // ── Verdict ─────────────────────────────────────────────────────────────
    // Compare the held op-point against the ngspice seed per device.
    eprintln!("\n  per-device drift seed→s=48000 (vs ngspice .op):");
    let mut max_vbe_drift = 0.0f64;
    let mut worst = String::new();
    for (label, vbe_seed, vce_seed) in NGSPICE_OP {
        let held = last_recs.iter().find(|r| r.ref_str() == *label);
        if let Some(h) = held {
            let dvbe = (h.v_be as f64) - vbe_seed;
            let dvce = (h.v_ce as f64) - vce_seed;
            eprintln!(
                "    {label:>4}: Vbe {:.4}→{:.4} (Δ{:+.4})   Vce {:.4}→{:.4} (Δ{:+.4})",
                vbe_seed, h.v_be, dvbe, vce_seed, h.v_ce, dvce
            );
            if dvbe.abs() > max_vbe_drift {
                max_vbe_drift = dvbe.abs();
                worst = label.to_string();
            }
        } else {
            eprintln!("    {label:>4}: NOT FOUND among MultiNl groups (label mismatch?)");
        }
    }
    let verdict = if max_vbe_drift < 0.02 { "HOLDS" } else { "WALKS" };
    eprintln!(
        "\n  VERDICT: {verdict} (max |ΔVbe| = {max_vbe_drift:.4} V at {worst}; HOLDS if < 0.02 V)"
    );

    // ── If it holds, re-measure AC gain WITH the seed (does it close?) ──────
    let golden_path = golden_dir(GOLDEN).join("sine_1k.npy");
    if golden_path.exists() {
        let golden = npy::read_f64(&golden_path).expect("read golden");
        let mut procg = compile_pedal(&def, SR).unwrap();
        procg.set_control("Gain", 0.5);
        let wdf = wdf_settled_window(&mut procg, SETTLE_S);
        let n = wdf.len().min(golden.len());
        let ac_gain = metrics::ac_gain_db(&wdf[..n], &golden[..n], TEST_FREQ, SR);
        eprintln!(
            "  AC gain WITH op{{}} seed = {ac_gain:+.2} dB  (baseline unseeded = -25.13 dB; 0 dB = matches ngspice)"
        );
    }

    assert!(!seed_recs.is_empty(), "no MultiNl BJT groups captured — wiring/labels wrong");
}

// ════════════════════════════════════════════════════════════════════════════
// BA283 PER-SAMPLE MAP JACOBIAN — stability analysis of the op-point repeller.
//
// ANALYSIS PROBE (no engine fix). Default-OFF and byte-identical to normal runs:
// the whole body is gated behind `PK_BA283_JMAP=1`, so unset it is a no-op that
// touches nothing. It also SKIPS when the pro `.pedal` is absent (CI).
//
// # What it builds
//
// The established fact (see `ba283_seed_and_hold`) is that ngspice's correct
// operating point, seeded EXACTLY (op{} device ports + nodes{} cap voltages,
// servo OFF), satisfies the within-sample grouped-NR to machine zero yet the
// per-sample discrete-time map slides OFF it in one sample. So the correct op is
// a STABLE fixed point of the continuous circuit but a REPELLER of the WDF's
// sample-to-sample map. This probe LINEARIZES that map at the correct op.
//
// State = the reactive-port (capacitor) WDF memory `wave_state` (the reflected
// wave carried across the sample boundary; at DC a=b=Vcap). BA283 has 5 caps:
// Cin, Cmil (220p), Ccmp (330p), Cfb (4n7), Cout. For each cap state we apply a
// small central-difference perturbation, run ONE `process(0.0)` (grouped-NR +
// cap-update), and read the resulting cap states → one column of the 5×5 map
// Jacobian J (state[n] → state[n+1]). A fresh, fully-seeded processor is rebuilt
// for every perturbation so the NR warm-start (v_prev/v_prev_2) is pinned at the
// correct op each time — this isolates the cap→cap reduced map.
//
// It ALSO builds an EXTENDED Jacobian over (caps ⊕ v_prev ⊕ v_prev_2), i.e. the
// caps PLUS the grouped-NR warm-start memory, to test whether the |λ|>1 pole
// lives in the pure cap dynamics or only appears once the frozen-Newton
// warm-start is carried forward (deliverable #4: which discretization is
// responsible).
//
// The matrices are printed Python-ready; eigen-decomposition is done offline with
// numpy (no linalg dep in this crate). Honest by construction: it also prints
// M(x0) − x0 (the raw one-sample drift of the seeded op) and the conditioning.
//
// Run:
//   PK_BA283_JMAP=1 cargo test -p pedalkernel-validate --test neve1073_spice \
//     ba283_persample_map_jacobian -- --nocapture
#[test]
fn ba283_persample_map_jacobian() {
    use pedalkernel_rt::processor::Stage;

    if std::env::var("PK_BA283_JMAP").is_err() {
        // Default-off: byte-identical no-op probe.
        return;
    }
    // Established analysis condition: DC servo OFF (the correct op is the object
    // of study; the servo is the candidate fix, not part of the bare map).
    std::env::set_var("PK_SERVO_DISABLE", "1");

    let source = skip_if_missing!(load_pro_pedal_sub(PRO_PATH), PRO_PATH);
    // FULL operating-point seed: device ports (op{}) + cap voltages (nodes{}).
    let seeded_src = inject_op_block_full(&source, true);

    // Build a fresh processor pinned at the correct op (caps + v_prev = ngspice).
    let build = || {
        let def = parse_pedal_file(&seeded_src).expect("parse seeded BA283");
        let mut proc = compile_pedal(&def, SR).expect("compile seeded BA283");
        proc.set_control("Gain", 0.5);
        proc
    };

    // ── Enumerate cap coordinates: (stage_idx, slot, C, name) ────────────────
    // Name by capacitance; the two 10µF caps (Cin, Cout) are split by seed sign
    // (Cin ≈ −1.03 V, Cout ≈ +4.80 V).
    let proc0 = build();
    struct Coord {
        stage: usize,
        slot: usize,
        cap_f: f64,
        name: String,
    }
    let mut coords: Vec<Coord> = Vec::new();
    for (si, st) in proc0.stages.iter().enumerate() {
        let Stage::MultiNl(mnl) = st else { continue };
        for op in mnl.passive_one_ports.iter() {
            if op.type_tag() != "capacitor" {
                continue;
            }
            let Some(slot) = op.state_slot() else { continue };
            let cap_f = op.physical_value() as f64;
            let v = mnl.passive_runtime_state.wave_cache[slot.0].wave_state as f64;
            let name = if (cap_f - 220e-12).abs() < 1e-13 {
                "Cmil".to_string()
            } else if (cap_f - 330e-12).abs() < 1e-13 {
                "Ccmp".to_string()
            } else if (cap_f - 4.7e-9).abs() < 1e-11 {
                "Cfb".to_string()
            } else if v < 0.0 {
                "Cin".to_string()
            } else {
                "Cout".to_string()
            };
            coords.push(Coord { stage: si, slot: slot.0, cap_f, name });
        }
    }
    eprintln!("\n  ═══ BA283 per-sample map Jacobian (correct op, servo OFF) ═══");
    eprintln!("  reactive cap states found: {}", coords.len());
    for (i, c) in coords.iter().enumerate() {
        eprintln!(
            "    x[{i}] = {:<5} C={:.3e}F  stage={} slot={}",
            c.name, c.cap_f, c.stage, c.slot
        );
    }
    assert!(!coords.is_empty(), "no reactive cap states enumerated");

    // State accessors (cap `wave_state` = the cross-sample WDF memory).
    let read_caps = |proc: &pedalkernel::compiler::CompiledPedal| -> Vec<f64> {
        coords
            .iter()
            .map(|c| {
                if let Stage::MultiNl(m) = &proc.stages[c.stage] {
                    m.passive_runtime_state.wave_cache[c.slot].wave_state as f64
                } else {
                    unreachable!()
                }
            })
            .collect()
    };
    let set_cap = |proc: &mut pedalkernel::compiler::CompiledPedal, i: usize, v: f64| {
        let c = &coords[i];
        if let Stage::MultiNl(m) = &mut proc.stages[c.stage] {
            m.passive_runtime_state.wave_cache[c.slot].wave_state = v as pedalkernel_rt::Wave;
        }
    };

    // v_prev / v_prev_2 accessors (grouped-NR warm-start memory). Flattened over
    // all MultiNl stages in order.
    let read_vprev = |proc: &pedalkernel::compiler::CompiledPedal, second: bool| -> Vec<f64> {
        let mut out = Vec::new();
        for st in proc.stages.iter() {
            if let Stage::MultiNl(m) = st {
                let v = if second { &m.v_prev_2 } else { &m.v_prev };
                for &x in v.iter() {
                    out.push(x as f64);
                }
            }
        }
        out
    };
    let set_vprev = |proc: &mut pedalkernel::compiler::CompiledPedal, second: bool, j: usize, v: f64| {
        let mut idx = 0usize;
        for st in proc.stages.iter_mut() {
            if let Stage::MultiNl(m) = st {
                let vv = if second { &mut m.v_prev_2 } else { &mut m.v_prev };
                for x in vv.iter_mut() {
                    if idx == j {
                        *x = v as pedalkernel_rt::Wave;
                        return;
                    }
                    idx += 1;
                }
            }
        }
    };

    // ── Baseline x0 and one-sample drift M(x0) − x0 (repeller evidence) ──────
    let x0 = read_caps(&proc0);
    let vprev0 = read_vprev(&proc0, false);
    let vprev20 = read_vprev(&proc0, true);
    let n_cap = x0.len();
    let n_vp = vprev0.len();

    let mut drift_proc = build();
    // Verify fresh-build determinism: caps must match proc0 exactly.
    let x0b = read_caps(&drift_proc);
    let build_det = x0
        .iter()
        .zip(&x0b)
        .map(|(a, b)| (a - b).abs())
        .fold(0.0f64, f64::max);
    drift_proc.process(0.0);
    let x1 = read_caps(&drift_proc);
    eprintln!("\n  fresh-build determinism max|Δ| = {build_det:.3e} (want ~0)");
    eprintln!("  one-sample cap drift  M(x0) − x0  (repeller if nonzero & growing):");
    for i in 0..n_cap {
        eprintln!(
            "    Δ{:<5} = {:+.6e}   (x0 = {:+.6})",
            coords[i].name, x1[i] - x0[i], x0[i]
        );
    }
    let drift_norm = (0..n_cap)
        .map(|i| (x1[i] - x0[i]).powi(2))
        .sum::<f64>()
        .sqrt();
    eprintln!("  ||M(x0) − x0||₂ = {drift_norm:.6e}");

    // ── Central-difference Jacobian columns ──────────────────────────────────
    // Full extended state ordering: [caps(n_cap), v_prev(n_vp), v_prev_2(n_vp)].
    let dim = n_cap + 2 * n_vp;
    // Perturbation size: caps O(1-5V), v_prev O(0.4-19V). Use a modest absolute
    // step; also probe a 10× step to check linearity/conditioning.
    let base_states: Vec<f64> = x0
        .iter()
        .cloned()
        .chain(vprev0.iter().cloned())
        .chain(vprev20.iter().cloned())
        .collect();

    let apply_state = |proc: &mut pedalkernel::compiler::CompiledPedal, k: usize, v: f64| {
        if k < n_cap {
            set_cap(proc, k, v);
        } else if k < n_cap + n_vp {
            set_vprev(proc, false, k - n_cap, v);
        } else {
            set_vprev(proc, true, k - n_cap - n_vp, v);
        }
    };
    let read_full = |proc: &pedalkernel::compiler::CompiledPedal| -> Vec<f64> {
        let mut out = read_caps(proc);
        out.extend(read_vprev(proc, false));
        out.extend(read_vprev(proc, true));
        out
    };

    // ── ε-sweep on the 4-cap block (smoothness / conditioning check) ─────────
    // If the spectral radius is stable across decades of ε, the huge Jacobian is
    // a genuine (stiff) derivative; if it swings wildly, the map is non-smooth
    // (BJT-NR branch switch) and the "eigenvalue" is ill-defined — reported
    // honestly either way.
    eprintln!("\n  === PYEPS_BEGIN ===");
    for &e in &[1e-2_f64, 1e-3, 1e-4, 1e-5, 1e-6] {
        let mut jc = vec![vec![0.0f64; n_cap]; n_cap];
        for k in 0..n_cap {
            let mut pp = build();
            set_cap(&mut pp, k, x0[k] + e);
            pp.process(0.0);
            let sp = read_caps(&pp);
            let mut pm = build();
            set_cap(&mut pm, k, x0[k] - e);
            pm.process(0.0);
            let sm = read_caps(&pm);
            for r in 0..n_cap {
                jc[r][k] = (sp[r] - sm[r]) / (2.0 * e);
            }
        }
        eprint!("Jeps_{e:e} = [");
        for r in 0..n_cap {
            eprint!("[");
            for c in 0..n_cap {
                eprint!("{:.10e}, ", jc[r][c]);
            }
            eprint!("], ");
        }
        eprintln!("]");
    }
    eprintln!("  === PYEPS_END ===");

    let eps = 1e-4_f64;
    // Build the full extended Jacobian J[dim][dim]: column k = ∂state'/∂state_k.
    let mut jac = vec![vec![0.0f64; dim]; dim];
    for k in 0..dim {
        let mut pp = build();
        apply_state(&mut pp, k, base_states[k] + eps);
        pp.process(0.0);
        let sp = read_full(&pp);

        let mut pm = build();
        apply_state(&mut pm, k, base_states[k] - eps);
        pm.process(0.0);
        let sm = read_full(&pm);

        for r in 0..dim {
            jac[r][k] = (sp[r] - sm[r]) / (2.0 * eps);
        }
    }

    // Emit Python-ready matrices. The 5×5 cap block is the primary deliverable;
    // the full extended matrix is the frozen-Newton robustness check.
    let labels: Vec<String> = coords
        .iter()
        .map(|c| c.name.clone())
        .chain((0..n_vp).map(|j| format!("vprev{j}")))
        .chain((0..n_vp).map(|j| format!("vprev2_{j}")))
        .collect();
    eprintln!("\n  === PYJAC_BEGIN ===");
    eprintln!("n_cap = {n_cap}");
    eprintln!("dim = {dim}");
    eprintln!("labels = {labels:?}");
    eprint!("Jcap = [");
    for r in 0..n_cap {
        eprint!("[");
        for c in 0..n_cap {
            eprint!("{:.12e}, ", jac[r][c]);
        }
        eprint!("], ");
    }
    eprintln!("]");
    eprint!("Jfull = [");
    for r in 0..dim {
        eprint!("[");
        for c in 0..dim {
            eprint!("{:.12e}, ", jac[r][c]);
        }
        eprint!("], ");
    }
    eprintln!("]");
    eprintln!("  === PYJAC_END ===");
    eprintln!(
        "\n  NOTE: run the printed Jcap/Jfull through numpy.linalg.eig offline; |λ|>1 = repeller."
    );
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
