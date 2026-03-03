//! Deterministic regression tests that lock current WDF behavior for
//! representative circuits. When physics improve, rerun the ignored
//! `regenerate_golden_regressions` test to refresh fixtures.

mod audio_analysis;

use audio_analysis::*;
use std::fs;
use std::path::{Path, PathBuf};

struct GoldenCase {
    name: &'static str,
    pedal: &'static str,
    controls: &'static [(&'static str, f64)],
    input: fn() -> Vec<f64>,
    tolerance: f64,
}

const GOLDEN_CASES: &[GoldenCase] = &[
    GoldenCase {
        name: "clip_silicon",
        pedal: "clip_silicon.pedal",
        controls: &[],
        input: clip_silicon_input,
        tolerance: 3e-8,
    },
    GoldenCase {
        name: "triode_clean",
        pedal: "triode_clean.pedal",
        controls: &[],
        input: triode_clean_input,
        tolerance: 6e-8,
    },
    GoldenCase {
        name: "opamp_gain",
        pedal: "opamp_gain.pedal",
        controls: &[("Gain", 0.7)],
        input: opamp_gain_input,
        tolerance: 5e-8,
    },
];

#[test]
fn golden_regressions_hold() {
    for case in GOLDEN_CASES {
        assert_case_matches(case);
    }
}

#[test]
#[ignore]
fn regenerate_golden_regressions() {
    for case in GOLDEN_CASES {
        let output = evaluate_case(case);
        write_golden(case, &output);
        println!("Updated golden for {}", case.name);
    }
}

fn assert_case_matches(case: &GoldenCase) {
    let expected = read_golden(case);
    let actual = evaluate_case(case);

    assert_eq!(
        actual.len(),
        expected.len(),
        "Golden {} length mismatch",
        case.name
    );
    assert_healthy(&actual, case.name, 40.0);

    let diff: Vec<f64> = actual.iter().zip(&expected).map(|(a, b)| a - b).collect();
    let diff_rms = rms(&diff);
    assert!(
        diff_rms < case.tolerance,
        "Golden '{}' drifted: rms={diff_rms:.3e}, tol={}",
        case.name,
        case.tolerance
    );
}

fn evaluate_case(case: &GoldenCase) -> Vec<f64> {
    let input = (case.input)();
    compile_test_pedal_and_process(case.pedal, &input, SAMPLE_RATE, case.controls)
}

fn clip_silicon_input() -> Vec<f64> {
    sine_fixed_len(1000.0, 0.4, 4096)
}

fn triode_clean_input() -> Vec<f64> {
    pluck_fixed_len(196.0, 4096)
}

fn opamp_gain_input() -> Vec<f64> {
    sine_fixed_len(330.0, 0.3, 4096)
}

fn sine_fixed_len(freq_hz: f64, amplitude: f64, samples: usize) -> Vec<f64> {
    (0..samples)
        .map(|i| {
            let t = i as f64 / SAMPLE_RATE;
            amplitude * (2.0 * std::f64::consts::PI * freq_hz * t).sin()
        })
        .collect()
}

fn pluck_fixed_len(freq_hz: f64, samples: usize) -> Vec<f64> {
    (0..samples)
        .map(|i| {
            let t = i as f64 / SAMPLE_RATE;
            let env = (-3.0 * t).exp();
            let f = 2.0 * std::f64::consts::PI * freq_hz * t;
            0.4 * env * (f.sin() + 0.5 * (2.0 * f).sin() + 0.25 * (3.0 * f).sin())
        })
        .collect()
}

fn golden_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("golden")
}

fn read_golden(case: &GoldenCase) -> Vec<f64> {
    let path = golden_dir().join(format!("{}.bin", case.name));
    let bytes = fs::read(&path)
        .unwrap_or_else(|e| panic!("Missing golden {}: {} ({})", case.name, path.display(), e));
    bytes
        .chunks_exact(8)
        .map(|chunk| f64::from_le_bytes(chunk.try_into().unwrap()))
        .collect()
}

fn write_golden(case: &GoldenCase, samples: &[f64]) {
    let path = golden_dir().join(format!("{}.bin", case.name));
    fs::create_dir_all(path.parent().unwrap()).expect("create golden dir");
    let mut bytes = Vec::with_capacity(samples.len() * 8);
    for &s in samples {
        bytes.extend_from_slice(&s.to_le_bytes());
    }
    fs::write(&path, bytes).expect("write golden");
}
