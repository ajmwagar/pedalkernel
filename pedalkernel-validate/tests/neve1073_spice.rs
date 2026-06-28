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
const SINE_DURATION: f64 = 0.1;
const SINE_AMP: f64 = 0.1;
const TEST_FREQ: f64 = 1_000.0;

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

fn rms(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    (signal.iter().map(|&x| x * x).sum::<f64>() / signal.len() as f64).sqrt()
}

/// Simulate the ngspice deck and write `golden/active/<GOLDEN>/sine_1k.npy`.
#[test]
fn bootstrap_ba283_golden() {
    if std::env::var("PEDALKERNEL_BOOTSTRAP_1073").as_deref() != Ok("1") {
        eprintln!("  SKIP bootstrap_ba283_golden: set PEDALKERNEL_BOOTSTRAP_1073=1 to run");
        return;
    }

    let circuit = spice_path(DECK);
    assert!(circuit.exists(), "{DECK}.spice not found at {circuit:?}");

    let config = SpiceConfig {
        sample_rate: SR as u32,
        oversample: OVERSAMPLE,
    };
    let runner = SpiceRunner::new(config.clone());

    let internal_rate = config.internal_rate();
    let n_internal = (SINE_DURATION * internal_rate) as usize;
    let input_internal: Vec<f64> = (0..n_internal)
        .map(|i| SINE_AMP * (2.0 * PI * TEST_FREQ * i as f64 / internal_rate).sin())
        .collect();

    let golden_output = runner
        .simulate(&circuit, &input_internal, "v_out")
        .unwrap_or_else(|e| panic!("ngspice simulation failed for {GOLDEN} bootstrap: {e}"));

    let dir = golden_dir(GOLDEN);
    std::fs::create_dir_all(&dir).unwrap_or_else(|e| panic!("create golden/active/{GOLDEN}: {e}"));
    let golden_path = dir.join("sine_1k.npy");
    npy::write_f64(&golden_path, &golden_output)
        .unwrap_or_else(|e| panic!("write {GOLDEN} golden: {e}"));

    eprintln!(
        "  bootstrapped {GOLDEN} golden: {} samples → {golden_path:?}",
        golden_output.len()
    );
}

/// Compare the WDF BA283 block against the ngspice golden (GAIN = 0.5).
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

    let n = golden.len();
    let input: Vec<f64> = (0..n)
        .map(|i| SINE_AMP * (2.0 * PI * TEST_FREQ * i as f64 / SR).sin())
        .collect();

    // Warm up half a buffer before recording (DC-block / coupling-cap settling).
    let warmup = n / 2;
    for i in 0..warmup {
        proc.process(SINE_AMP * (2.0 * PI * TEST_FREQ * i as f64 / SR).sin());
    }
    let wdf_output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

    let trim = (0.01 * SR) as usize;
    let wdf_trim = &wdf_output[trim.min(wdf_output.len())..];
    let golden_trim = &golden[trim.min(golden.len())..];
    let result = metrics::compare(wdf_trim, golden_trim, SR, Some(TEST_FREQ));

    eprintln!("  {GOLDEN} WDF vs ngspice (after {trim}-sample trim):");
    eprintln!("    normalized RMS error : {:.1} dB", result.normalized_rms_error_db);
    eprintln!("    peak error           : {:.1} dB", result.peak_error_db);
    eprintln!("    WDF RMS amplitude    : {:.4} V", rms(wdf_trim));
    eprintln!("    ngspice RMS amplitude: {:.4} V", rms(golden_trim));
    if rms(golden_trim) > 1e-9 {
        eprintln!(
            "    WDF-vs-ngspice gap   : {:.1} dB (positive = WDF hotter)",
            20.0 * (rms(wdf_trim) / rms(golden_trim)).log10()
        );
    }
    if let Some(thd_err) = result.thd_error_db {
        eprintln!("    THD error            : {:.1} dB", thd_err);
    }

    assert!(
        rms(&wdf_output) > 1e-6,
        "{GOLDEN} WDF output is silence (RMS < 1e-6): compilation or processing failed"
    );

    // No hard gate yet — thresholds are set after the first measured run
    // (house style: measure, then gate at measured + ~3 dB margin).
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

    // Per-group transconductance behavior across the swing.
    eprintln!("\n  per-BJT operating-point swing + transconductance check:");
    eprintln!(
        "  {:>5} {:>9} {:>9} {:>11} {:>11} {:>11}",
        "grp", "Vbe~", "gm~", "dVbe", "dIc", "dIc/(gm·dVbe)"
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
        let ratio = if gm_avg * dvbe != 0.0 {
            dic / (gm_avg * dvbe)
        } else {
            f64::NAN
        };
        eprintln!(
            "  {g:>5} {:>9.4} {:>9.5} {:>11.3e} {:>11.3e} {:>13.3}",
            (a.v_be + c.v_be) / 2.0,
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

/// CONTROLLED EXPERIMENT (bead pedalkernel-ej0v): force the BA283 BJTs to the
/// SPICE/published DC operating point and measure whether the gain closes.
///
/// Decouples "is the op-point sufficient?" from "how do we compute it?".  The
/// `PK_FORCE_OPPOINT` build flag (default off) bypasses both the linear bias and
/// `solve_joint_dc_qpoint` (which converges to the WRONG, TR1-starved Vbe≈0.402)
/// and feeds `apply_dc_qpoint_seed` an externally-supplied per-port op-point.
///
/// Op-point source (ngspice `.op` on `neve1073_ba283.spice`, authoritative):
///   nodes: n1=1.0764 nb1=0.9365 ne1=0.3256 ne2=3.3301 nd=2.8786 vcc=24
///   TR1(Q3, grp0): Vbe = nb1−ne1 = 0.6109,  Vce = n1−ne1  = 0.7508
///   TR2(Q1, grp1): Vbe = n1−ne2  = −2.254,  Vce = vcc−ne2 = 20.67
///   TR3(Q2, grp2): Vbe = ne2−nd  = 0.4515,  Vce = vcc−nd  = 21.12
/// The SPICE Darlington (TR2) runs model-driven (negative Vbe — the .pedal header
/// flags it ~2 V high); the *published* Neve unit gives physical junctions, so we
/// also run a PUBLISHED variant (TR1 0.5/3.2, TR2 0.5/19.5, TR3 0.6/20.1).  The
/// decisive transistor is TR1 (grp0): target Vbe≈0.64, gm≈0.033 (bead).
///
/// Port order (build order, per `classify_nl_devices`):
///   [TR1_Vbe, TR1_Vce, TR2_Vbe, TR2_Vce, TR3_Vbe, TR3_Vce]
///
/// Run: `cargo test -p pedalkernel-validate --features diag --test neve1073_spice
///       ba283_forced_oppoint -- --nocapture --test-threads=1`
#[cfg(feature = "diag")]
#[test]
fn ba283_forced_oppoint() {
    use pedalkernel_rt::diag_ring::OpPointRecord;
    use pedalkernel_rt::processor::Stage;

    let source = skip_if_missing!(load_pro_pedal_sub(PRO_PATH), PRO_PATH);

    // Golden RMS (ngspice) for the dB-vs-golden verdict.
    let golden_path = golden_dir(GOLDEN).join("sine_1k.npy");
    let golden_rms = if golden_path.exists() {
        let g = npy::read_f64(&golden_path).expect("read golden");
        let trim = (0.01 * SR) as usize;
        rms(&g[trim.min(g.len())..])
    } else {
        eprintln!("  (golden absent — dB-vs-golden will use design 97x reference)");
        f64::NAN
    };

    // The forced op-points (per-NL-port voltage vectors, build order).
    // "spice"     — straight from the ngspice .op (Darlington model-driven).
    // "spice_tr1" — force ONLY TR1 (the starved input device), leave TR2/TR3 free.
    // "published" — published Neve DC voltages (physical junctions).
    let cases: &[(&str, &str)] = &[
        ("baseline (no force)", ""),
        ("spice_tr1_only", "0.6109,0.7508,_,_,_,_"),
        ("spice_full", "0.6109,0.7508,-2.254,20.67,0.4515,21.12"),
        ("published", "0.5,3.2,0.5,19.5,0.6,20.1"),
    ];

    eprintln!("\n  ═══ BA283 FORCED-OP-POINT experiment (bead ej0v) ═══");
    if golden_rms.is_finite() {
        eprintln!("  golden (ngspice) RMS = {golden_rms:.4} V  [design chain gain ~97x]");
    }

    for (label, spec) in cases {
        // SAFETY: single-threaded test (--test-threads=1); env set before compile.
        if spec.is_empty() {
            std::env::remove_var("PK_FORCE_OPPOINT");
        } else {
            std::env::set_var("PK_FORCE_OPPOINT", spec);
        }

        let def = parse_pedal_file(&source).unwrap_or_else(|e| panic!("parse {PRO_PATH}: {e}"));
        let mut proc =
            compile_pedal(&def, SR).unwrap_or_else(|e| panic!("compile {PRO_PATH}: {e}"));
        proc.set_control("Gain", 0.5);

        const BLOCK: usize = 128;
        const NBLOCKS: usize = 48;
        let warmup = 2 * BLOCK;
        for i in 0..warmup {
            proc.process(SINE_AMP * (2.0 * PI * TEST_FREQ * i as f64 / SR).sin());
        }

        let mut in_peak = 0.0f64;
        let mut out_peak = 0.0f64;
        let mut sum_sq = 0.0f64;
        let mut nsamp = 0usize;
        let mut last_recs: Vec<OpPointRecord> = Vec::new();
        let mut sample = warmup;
        for _b in 0..NBLOCKS {
            for _ in 0..BLOCK {
                let x = SINE_AMP * (2.0 * PI * TEST_FREQ * sample as f64 / SR).sin();
                let y = proc.process(x) as f64;
                in_peak = in_peak.max(x.abs());
                out_peak = out_peak.max(y.abs());
                sum_sq += y * y;
                nsamp += 1;
                sample += 1;
            }
            let mut recs = [OpPointRecord::default(); 8];
            let mut n = 0usize;
            for (si, st) in proc.stages.iter().enumerate() {
                if let Stage::MultiNl(mnl) = st {
                    n += mnl.runtime_op_points(si, &mut recs[n..]);
                }
            }
            last_recs = recs[..n].to_vec();
        }

        let realized = if in_peak > 1e-12 { out_peak / in_peak } else { 0.0 };
        let realized_db = if realized > 0.0 {
            20.0 * realized.log10()
        } else {
            f64::NEG_INFINITY
        };
        let out_rms = if nsamp > 0 {
            (sum_sq / nsamp as f64).sqrt()
        } else {
            0.0
        };
        let ba283_db_vs_golden = if golden_rms.is_finite() && out_rms > 1e-12 {
            20.0 * (out_rms / golden_rms).log10()
        } else {
            f64::NAN
        };

        eprintln!("\n  ── case: {label}  (spec={spec:?}) ──");
        eprintln!("    held op-points (diag ring, last block):");
        for (g, r) in last_recs.iter().enumerate() {
            let tag = match g {
                0 => "TR1",
                1 => "TR2",
                2 => "TR3",
                _ => "??",
            };
            eprintln!(
                "      grp{g} {tag}: Vbe={:.4}V Vce={:+.3}V Ic={:.3e}A gm={:.5}S",
                r.v_be, r.v_ce, r.i_c, r.gm
            );
        }
        eprintln!(
            "    realized chain gain: {realized:.4}x ({realized_db:+.2} dB)  out_rms={out_rms:.4}V",
        );
        if golden_rms.is_finite() {
            eprintln!(
                "    BA283 vs golden: {ba283_db_vs_golden:+.2} dB  (0 dB = closed; design ~97x)"
            );
        }
    }

    std::env::remove_var("PK_FORCE_OPPOINT");
    eprintln!("\n  ═══ end forced-op-point experiment ═══\n");
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
