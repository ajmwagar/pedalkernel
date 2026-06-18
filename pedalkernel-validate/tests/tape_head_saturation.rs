//! Tape-head saturation engine check + pending-reference gate mechanism.
//!
//! Two concerns:
//!
//!  1. The engine circuit `circuits/tape/tape_head_saturation.pedal` compiles,
//!     runs, and actually SATURATES (THD climbs with drive). This must hold NOW
//!     so the comparison is valid once the ngspice golden lands.
//!  2. The `pending_reference` gate mechanism: a pending test with a missing
//!     golden is reported PENDING and excluded from BOTH the passed count and
//!     the total (denominator), while a NON-pending test with a missing golden
//!     still FAILS loudly.

use std::collections::BTreeMap;
use std::fs;

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;

use pedalkernel_validate::config::{
    MetricConfig, PassCriteria, SignalConfig, TestCase, TestSuite, ValidationConfig,
};
use pedalkernel_validate::report::ValidationReport;
use pedalkernel_validate::runner::{RunnerConfig, ValidationRunner};

const FS: f64 = 96_000.0;
const PI: f64 = std::f64::consts::PI;

fn circuits_dir() -> std::path::PathBuf {
    std::path::PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/circuits"))
}

/// Goertzel magnitude at frequency `f`.
fn goertzel(w: &[f64], f: f64) -> f64 {
    let n = w.len() as f64;
    let k = (0.5 + n * f / FS).floor();
    let omega = 2.0 * PI * k / n;
    let coeff = 2.0 * omega.cos();
    let (mut s1, mut s2) = (0.0f64, 0.0f64);
    for &x in w {
        let s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    (s1 * s1 + s2 * s2 - coeff * s1 * s2).max(0.0).sqrt()
}

fn thd(w: &[f64], f0: f64) -> f64 {
    let fund = goertzel(w, f0);
    if fund <= 1.0e-18 {
        return 0.0;
    }
    let mut h = 0.0;
    for k in 2..=8 {
        let m = goertzel(w, k as f64 * f0);
        h += m * m;
    }
    h.sqrt() / fund
}

/// Drive the compiled tape-head circuit at amplitude `amp` and return the
/// steady-state (last 4 periods) output.
fn drive(amp: f64, f0: f64) -> Vec<f64> {
    let path = circuits_dir().join("tape/tape_head_saturation.pedal");
    let src = fs::read_to_string(&path).expect("read tape_head_saturation.pedal");
    let def = parse_pedal_file(&src).expect("parse tape_head_saturation.pedal");
    let mut proc = compile_pedal(&def, FS).expect("compile tape_head_saturation.pedal");

    let per = (FS / f0).round() as usize;
    let n = per * 16;
    let mut out = Vec::with_capacity(n);
    for k in 0..n {
        let a = amp * (2.0 * PI * f0 * k as f64 / FS).sin();
        out.push(proc.process(a));
    }
    let keep = per * 4;
    out[out.len() - keep..].to_vec()
}

#[test]
fn tape_head_circuit_compiles_runs_and_saturates() {
    let f0 = 120.0;

    // Sweep clean -> knee -> past-knee. The J-A knee sits at ~1 V (H = kv*V
    // reaches the Langevin saturation region), so output THD RISES into ~1 V and
    // then RECEDES at 3 V: past the knee the saturating magnetic current
    // (Isat*M/Ms) clamps to a near-constant offset, and in this stiff 470/2.2k
    // divider its already-light contribution to V(out) shrinks relative to the
    // fundamental. So the saturation witness is the KNEE THD (~1 V), not 3 V.
    // The absolute THD is modest (~1.2% peak) because the head is a light shunt
    // load here — this matches the ngspice ground truth for the same circuit
    // (see spice-circuits/tape/tape_head_saturation.spice + bead pedalkernel-x0mv
    // Phase 0). It is NOT the ~5%+ an earlier SERIES-miswired netlist showed by
    // forcing the full signal current through the head; that topology did not
    // match the shunt golden deck.
    let amps = [0.05_f64, 0.25, 1.0, 3.0];
    let mut curve = Vec::new();
    for &amp in &amps {
        let w = drive(amp, f0);
        assert!(
            w.iter().all(|x| x.is_finite()),
            "non-finite output at amp={amp}"
        );
        let t = thd(&w, f0);
        println!("[tape_head circuit] amp={amp:>4} V : THD={t:.5}");
        curve.push((amp, t));
    }

    let clean = curve[0].1; // 0.05 V — well below the knee
    let knee = curve[2].1; //  1.0  V — at the J-A knee (peak coloration)

    assert!(
        clean < 0.01,
        "low-level THD should be clean (<1%), got {clean:.5}"
    );
    assert!(
        knee > 0.01,
        "knee-level THD should show saturation (>1%), got {knee:.5}"
    );
    assert!(
        knee > clean * 3.0,
        "THD must climb clearly into the knee: clean {clean:.5} -> knee {knee:.5}"
    );
}

/// Build a one-suite config with `tape_head_saturation` flagged pending plus a
/// second NON-pending test, both with goldens that will be missing.
fn pending_test_config() -> ValidationConfig {
    let sine = vec![SignalConfig::Sine {
        frequency: 120.0,
        amplitude: 1.0,
        duration: 0.05,
        label: Some("hot".to_string()),
    }];
    let metrics = vec![
        MetricConfig::TimeDomain,
        MetricConfig::Thd { fundamental: 120.0 },
    ];

    let mut tests = BTreeMap::new();
    tests.insert(
        "tape_head_saturation".to_string(),
        TestCase {
            circuit: "tape/tape_head_saturation.pedal".to_string(),
            description: "pending tape head".to_string(),
            signals: sine.clone(),
            metrics: metrics.clone(),
            pass_criteria: PassCriteria::default(),
            warmup_trim_ms: None,
            pending_reference: true,
        },
    );
    tests.insert(
        "not_pending_missing_golden".to_string(),
        TestCase {
            circuit: "tape/tape_head_saturation.pedal".to_string(),
            description: "non-pending, golden missing -> must fail".to_string(),
            signals: sine,
            metrics,
            pass_criteria: PassCriteria::default(),
            warmup_trim_ms: None,
            pending_reference: false,
        },
    );

    let mut suites = BTreeMap::new();
    suites.insert(
        "tape".to_string(),
        TestSuite {
            description: "tape pending mechanism".to_string(),
            tests,
        },
    );

    ValidationConfig {
        global: Default::default(),
        suites,
    }
}

#[test]
fn pending_excluded_from_gate_but_missing_golden_still_fails() {
    // Point golden_dir at an empty temp dir so EVERY golden is missing.
    let tmp = tempfile::tempdir().expect("tempdir");
    let runner_cfg = RunnerConfig {
        circuits_dir: circuits_dir(),
        golden_dir: tmp.path().to_path_buf(),
        output_dir: tmp.path().join("out"),
        sample_rate: 96_000,
        oversample: 1,
        save_output: false,
        regenerate_golden: false,
        skip_k_tables: false,
    };

    let cfg = pending_test_config();
    let runner = ValidationRunner::new(runner_cfg, cfg);
    let results = runner.run_all().expect("run_all");

    let suite = &results["tape"];

    // The non-pending missing-golden test FAILS; the pending one is PENDING.
    assert_eq!(suite.failed, 1, "non-pending missing golden must fail");
    assert_eq!(suite.passed, 0, "no test should pass with goldens missing");
    assert_eq!(suite.pending, 1, "pending_reference test must be PENDING");

    let pending = &suite.tests["tape_head_saturation"];
    assert!(
        pending.pending,
        "tape_head_saturation must be marked pending"
    );
    assert!(!pending.passed, "pending is not a pass");

    let failing = &suite.tests["not_pending_missing_golden"];
    assert!(!failing.pending, "non-pending must NOT be marked pending");
    assert!(!failing.passed, "non-pending missing golden must fail");

    // Gate summary: pending is excluded from BOTH passed and total. Only the
    // single failing test counts toward the denominator (0/1), and pending=1.
    let report = ValidationReport::new(results, 96_000, 1);
    assert_eq!(report.summary.total_tests, 1, "pending excluded from total");
    assert_eq!(report.summary.passed, 0);
    assert_eq!(report.summary.failed, 1);
    assert_eq!(report.summary.pending, 1);
}
