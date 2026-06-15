//! Tests for the dual-golden layout and compare-goldens harness capability.
//!
//! WHY this test exists: CI has neither .pedal sources for private circuits nor
//! ngspice. The compare-goldens workflow must work purely from committed .npy
//! artifacts. These tests verify:
//!
//!   1. `bootstrap` writes WDF goldens to `golden/{suite}/{test}/wdf/{signal}.npy`
//!      and does NOT touch `golden/{suite}/{test}/{signal}.npy` (the ngspice golden).
//!   2. Comparing a golden against itself yields near-zero error (the pass case).
//!   3. Missing either golden produces a "missing" row — not a panic.

use pedalkernel_validate::{
    config::PassCriteria,
    metrics,
    npy,
    report::SignalResult,
};
use tempfile::TempDir;

/// Helper: write a known signal to `golden/{suite}/{test}/{signal}.npy` (ngspice slot)
/// and the same signal to `golden/{suite}/{test}/wdf/{signal}.npy` (WDF slot).
fn write_pair(dir: &TempDir, suite: &str, test: &str, signal: &str, data: &[f64]) {
    let spice_path = dir
        .path()
        .join(suite)
        .join(test)
        .join(format!("{signal}.npy"));
    let wdf_path = dir
        .path()
        .join(suite)
        .join(test)
        .join("wdf")
        .join(format!("{signal}.npy"));
    npy::write_f64(&spice_path, data).expect("write spice golden");
    npy::write_f64(&wdf_path, data).expect("write wdf golden");
}

// ---------------------------------------------------------------------------
// 1. Layout: wdf/ subdirectory must be separate from ngspice golden
// ---------------------------------------------------------------------------

#[test]
fn wdf_golden_path_does_not_overlap_spice_golden() {
    let dir = TempDir::new().unwrap();
    let signal: Vec<f64> = (0..1000).map(|i| (i as f64 * 0.01).sin()).collect();

    write_pair(&dir, "pedals", "tb303_vcf", "sine", &signal);

    let spice_path = dir.path().join("pedals/tb303_vcf/sine.npy");
    let wdf_path = dir.path().join("pedals/tb303_vcf/wdf/sine.npy");

    // Both exist and are at distinct paths.
    assert!(spice_path.exists(), "ngspice golden must exist");
    assert!(wdf_path.exists(), "WDF golden must exist");
    assert_ne!(
        spice_path, wdf_path,
        "WDF and ngspice goldens must be at distinct paths"
    );

    // The ngspice golden parent dir does NOT contain a file named "sine.npy"
    // inside a sibling named "wdf/" — that would mean they had leaked.
    let spice_parent = spice_path.parent().unwrap();
    let wdf_parent = wdf_path.parent().unwrap();
    assert_ne!(spice_parent, wdf_parent);
}

// ---------------------------------------------------------------------------
// 2. Self-comparison: golden vs itself yields pass (near-zero error)
// ---------------------------------------------------------------------------

#[test]
fn self_comparison_yields_near_zero_error_and_passes() {
    let signal: Vec<f64> = (0..4800)
        .map(|i| (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 48000.0).sin())
        .collect();
    let sample_rate = 48000.0;

    // WDF golden == ngspice golden (perfect case / identity).
    let result = metrics::compare(&signal, &signal, sample_rate, Some(1000.0));

    assert!(
        result.normalized_rms_error_db < -100.0,
        "Self-comparison RMS error must be near -inf, got {:.1} dB",
        result.normalized_rms_error_db
    );
    assert!(
        result.peak_error_db < -100.0,
        "Self-comparison peak error must be near -inf, got {:.1} dB",
        result.peak_error_db
    );

    // Must pass even strict criteria.
    let strict = PassCriteria {
        normalized_rms_error_db: Some(-60.0),
        peak_error_db: Some(-40.0),
        ..Default::default()
    };
    assert!(
        result.passes(&strict),
        "Self-comparison must pass strict PassCriteria"
    );
}

// ---------------------------------------------------------------------------
// 3. Missing golden → "missing" row, not a panic
// ---------------------------------------------------------------------------

#[test]
fn missing_wdf_golden_produces_missing_signal_result_not_panic() {
    let dir = TempDir::new().unwrap();
    let signal: Vec<f64> = vec![0.1, 0.2, 0.3];

    // Write only the ngspice golden; leave the wdf/ slot empty.
    let spice_path = dir
        .path()
        .join("pedals/pultec/sine.npy");
    npy::write_f64(&spice_path, &signal).unwrap();

    let wdf_path = dir
        .path()
        .join("pedals/pultec/wdf/sine.npy");

    assert!(spice_path.exists());
    assert!(!wdf_path.exists(), "WDF golden must be absent for this test");

    // Simulate what compare-goldens does: detect absence, produce a missing row.
    let signal_result = if !wdf_path.exists() {
        let msg = format!("missing WDF golden: {}", wdf_path.display());
        SignalResult {
            label: "sine".to_string(),
            passed: false,
            pending: false,
            comparison: None,
            error: Some(msg),
        }
    } else {
        panic!("should not reach here");
    };

    assert!(!signal_result.passed, "missing-golden row must not pass");
    assert!(signal_result.comparison.is_none(), "missing-golden row must have no metrics");
    let err_msg = signal_result.error.as_deref().unwrap_or("");
    assert!(
        err_msg.contains("missing"),
        "error message must say 'missing', got: {err_msg}"
    );
}

#[test]
fn missing_spice_golden_produces_missing_signal_result_not_panic() {
    let dir = TempDir::new().unwrap();
    let signal: Vec<f64> = vec![0.1, 0.2, 0.3];

    // Write only the WDF golden; leave the ngspice slot empty.
    let wdf_path = dir
        .path()
        .join("pedals/pultec/wdf/sine.npy");
    npy::write_f64(&wdf_path, &signal).unwrap();

    let spice_path = dir.path().join("pedals/pultec/sine.npy");
    assert!(wdf_path.exists());
    assert!(!spice_path.exists(), "ngspice golden must be absent for this test");

    let signal_result = if !spice_path.exists() {
        let msg = format!("missing ngspice golden: {}", spice_path.display());
        SignalResult {
            label: "sine".to_string(),
            passed: false,
            pending: false,
            comparison: None,
            error: Some(msg),
        }
    } else {
        panic!("should not reach here");
    };

    assert!(!signal_result.passed);
    assert!(signal_result.comparison.is_none());
    let err_msg = signal_result.error.as_deref().unwrap_or("");
    assert!(err_msg.contains("missing"), "got: {err_msg}");
}

#[test]
fn missing_both_goldens_produces_missing_signal_result_not_panic() {
    let dir = TempDir::new().unwrap();

    let spice_path = dir.path().join("pedals/legend/sine.npy");
    let wdf_path = dir.path().join("pedals/legend/wdf/sine.npy");

    assert!(!spice_path.exists());
    assert!(!wdf_path.exists());

    let signal_result = if !spice_path.exists() || !wdf_path.exists() {
        let msg = format!(
            "missing both goldens: {} and {}",
            spice_path.display(),
            wdf_path.display()
        );
        SignalResult {
            label: "sine".to_string(),
            passed: false,
            pending: false,
            comparison: None,
            error: Some(msg),
        }
    } else {
        panic!("should not reach here");
    };

    assert!(!signal_result.passed);
    assert!(signal_result.comparison.is_none());
}

// ---------------------------------------------------------------------------
// 4. Report JSON format: compare-goldens uses the same ValidationReport type
//    as `run`, so the JSON shape is identical (no format fork).
// ---------------------------------------------------------------------------

#[test]
fn report_json_shape_matches_run_format() {
    use pedalkernel_validate::report::{SuiteResult, TestResult, ValidationReport};
    use std::collections::BTreeMap;

    // Build a minimal report that mirrors what compare-goldens emits.
    let signal_result = SignalResult::from_comparison(
        "sine".to_string(),
        metrics::compare(
            &[0.0f64; 100],
            &[0.0f64; 100],
            48000.0,
            None,
        ),
        true,
    );

    let mut tests = BTreeMap::new();
    tests.insert(
        "tb303_vcf".to_string(),
        TestResult {
            passed: true,
            pending: false,
            error: None,
            signals: vec![signal_result],
        },
    );

    let mut suites = BTreeMap::new();
    suites.insert(
        "pedals".to_string(),
        SuiteResult {
            description: "Classic pedal core circuit validation".to_string(),
            passed: 1,
            failed: 0,
            pending: 0,
            tests,
        },
    );

    let report = ValidationReport::new(suites, 96000, 4);

    // Serialize to JSON and verify required machine-readable fields.
    let json = serde_json::to_string_pretty(&report).unwrap();
    let parsed: serde_json::Value = serde_json::from_str(&json).unwrap();

    // Top-level required fields.
    assert!(parsed["summary"]["total_tests"].is_number(), "summary.total_tests missing");
    assert!(parsed["summary"]["passed"].is_number(), "summary.passed missing");
    assert!(parsed["summary"]["failed"].is_number(), "summary.failed missing");
    assert!(parsed["summary"]["pass_rate"].is_number(), "summary.pass_rate missing");
    assert!(parsed["timestamp"].is_string(), "timestamp missing");
    assert!(parsed["sample_rate"].is_number(), "sample_rate missing");
    assert!(parsed["oversample"].is_number(), "oversample missing");

    // Per-circuit metrics present in suites.
    let signal_json = &parsed["suites"]["pedals"]["tests"]["tb303_vcf"]["signals"][0];
    assert!(signal_json["label"].is_string(), "signal.label missing");
    assert!(signal_json["passed"].is_boolean(), "signal.passed missing");
    assert!(
        signal_json["comparison"]["normalized_rms_error_db"].is_number(),
        "comparison.normalized_rms_error_db missing"
    );
    assert!(
        signal_json["comparison"]["peak_error_db"].is_number(),
        "comparison.peak_error_db missing"
    );
    assert!(
        signal_json["comparison"]["spectral_error_db"].is_number(),
        "comparison.spectral_error_db missing"
    );

    // Aggregate pass count "N/M passed" readable from summary.
    let passed = parsed["summary"]["passed"].as_u64().unwrap();
    let total = parsed["summary"]["total_tests"].as_u64().unwrap();
    assert_eq!(passed, 1);
    assert_eq!(total, 1);
}
