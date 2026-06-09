//! Engine determinism: reset/rewind semantics and sample-rate invariance.

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::compiler::{compile_pedal, CompileOptions};
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::oversampling::OversamplingFactor;
use pedalkernel::PedalProcessor;
use std::path::Path;

#[test]
fn clip_silicon_reset_reproduces_output() {
    let src = read_test_pedal("clip_silicon.pedal");
    let pedal = parse_pedal_file(&src).expect("parse clip pedal");
    let mut proc = compile_pedal(&pedal, SAMPLE_RATE).expect("compile clip");

    let input = sine_at(660.0, 0.4, 0.1, SAMPLE_RATE);

    let first_run: Vec<f64> = input.iter().map(|&x| proc.process(x)).collect();
    proc.reset();
    let after_reset: Vec<f64> = input.iter().map(|&x| proc.process(x)).collect();

    assert_eq!(first_run.len(), after_reset.len());

    let diff: Vec<f64> = first_run
        .iter()
        .zip(&after_reset)
        .map(|(a, b)| a - b)
        .collect();
    let diff_rms = rms(&diff);
    assert!(
        diff_rms < 1e-10,
        "Reset should reproduce first run: diff_rms={diff_rms:e}"
    );
}

#[test]
fn triode_stage_is_sample_rate_consistent() {
    let duration = 0.2;
    let input48 = sine_at(523.25, 0.25, duration, 48_000.0);
    let input96 = sine_at(523.25, 0.25, duration, 96_000.0);

    // Use X1 oversampling so we compare raw sample rates, not oversampled rates
    let opts_48 = CompileOptions {
        oversampling: OversamplingFactor::X1,
        ..CompileOptions::default()
    };
    let opts_96 = CompileOptions {
        oversampling: OversamplingFactor::X1,
        ..CompileOptions::default()
    };
    let out48 =
        compile_test_pedal_with_options("triode_clean.pedal", &input48, 48_000.0, &[], opts_48);
    let out96 =
        compile_test_pedal_with_options("triode_clean.pedal", &input96, 96_000.0, &[], opts_96);
    let out96_down = decimate(&out96, 2);

    assert_eq!(out48.len(), out96_down.len());
    assert!(
        out48.iter().all(|x| x.is_finite()),
        "triode 48k output contains NaN/Inf"
    );
    assert!(
        out96_down.iter().all(|x| x.is_finite()),
        "triode 96k output contains NaN/Inf"
    );

    let corr = correlation(&out48, &out96_down).abs();
    assert!(
        corr > 0.985,
        "Downsampled 96k output should correlate: corr={corr:.6}"
    );

    let diff: Vec<f64> = out48.iter().zip(&out96_down).map(|(a, b)| a - b).collect();
    let diff_rms = rms(&diff);
    assert!(
        diff_rms < 0.01,
        "Sample-rate mismatch too large: rms={diff_rms}"
    );
}

fn decimate(buf: &[f64], factor: usize) -> Vec<f64> {
    buf.iter().step_by(factor).copied().collect()
}

fn read_test_pedal(filename: &str) -> String {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/test_pedals")
        .join(filename);
    std::fs::read_to_string(&path).expect("read test pedal")
}
