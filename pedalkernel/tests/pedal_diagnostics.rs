//! Targeted diagnostics for RAT and Klon reference pedals.
//! These cover aliasing, sample-rate scaling, solver convergence, and denormal behavior.

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::compiler::CompileOptions;
use pedalkernel::elements::{reset_solver_stats, solver_stats_snapshot};
use pedalkernel::oversampling::OversamplingFactor;

const RAT_PEDAL: &str = "proco_rat.pedal";
const RAT_CONTROLS: &[(&str, f64)] = &[("Distortion", 0.45), ("Filter", 0.35), ("Volume", 0.7)];
const KLON_PEDAL: &str = "klon_centaur.pedal";
const KLON_CONTROLS: &[(&str, f64)] = &[("Gain", 0.35), ("Treble", 0.5), ("Output", 0.7)];
const ALIAS_FREQ: f64 = 8000.0;

fn render_example(
    pedal: &str,
    controls: &[(&str, f64)],
    input: &[f64],
    sample_rate: f64,
    oversampling: OversamplingFactor,
) -> Vec<f64> {
    compile_example_with_options(
        pedal,
        input,
        sample_rate,
        controls,
        CompileOptions {
            oversampling,
            ..CompileOptions::default()
        },
    )
}

#[test]
fn rat_aliasing_reduced_by_oversampling() {
    let sample_rate = 24_000.0;
    for &amp in &[0.7, 0.9] {
        let input = sine_at(5_000.0, amp, 0.25, sample_rate);
        let no_os = render_example(
            RAT_PEDAL,
            RAT_CONTROLS,
            &input,
            sample_rate,
            OversamplingFactor::X1,
        );
        let with_os = render_example(
            RAT_PEDAL,
            RAT_CONTROLS,
            &input,
            sample_rate,
            OversamplingFactor::X4,
        );

        let aliased = alias_energy_summary(&no_os, sample_rate, ALIAS_FREQ, 50.0);
        let oversampled = alias_energy_summary(&with_os, sample_rate, ALIAS_FREQ, 50.0);

        assert!(
            aliased.alias_ratio > 1e-6,
            "Aliasing metric too small to compare (amp={amp}): ratio={}",
            aliased.alias_ratio
        );
        assert!(
            oversampled.alias_ratio < aliased.alias_ratio * 0.9,
            "Oversampling should cut aliasing (amp={amp}): no_os={:.6}, os={:.6}, peaks={:?}",
            aliased.alias_ratio,
            oversampled.alias_ratio,
            oversampled.top_bins,
        );
    }
}

#[test]
fn klon_aliasing_reduced_by_oversampling() {
    let sample_rate = 24_000.0;
    let input = sine_at(5_000.0, 0.8, 0.25, sample_rate);
    let no_os = render_example(
        KLON_PEDAL,
        KLON_CONTROLS,
        &input,
        sample_rate,
        OversamplingFactor::X1,
    );
    let with_os = render_example(
        KLON_PEDAL,
        KLON_CONTROLS,
        &input,
        sample_rate,
        OversamplingFactor::X4,
    );
    let aliased = alias_energy_summary(&no_os, sample_rate, ALIAS_FREQ, 50.0);
    let oversampled = alias_energy_summary(&with_os, sample_rate, ALIAS_FREQ, 50.0);
    assert!(
        aliased.alias_ratio > 1e-6,
        "Aliasing metric too small for Klon comparison: {}",
        aliased.alias_ratio
    );
    assert!(
        oversampled.alias_ratio < aliased.alias_ratio * 0.9,
        "Klon oversampling should suppress aliasing: no_os={:.6}, os={:.6}, peaks={:?}",
        aliased.alias_ratio,
        oversampled.alias_ratio,
        oversampled.top_bins
    );
}

fn compare_sample_rates(
    pedal: &str,
    controls: &[(&str, f64)],
    amp: f64,
    duration: f64,
) -> (f64, f64) {
    let base_rate = 48_000.0;
    let hi_rate = 96_000.0;
    let input48 = sine_at(1000.0, amp, duration, base_rate);
    let input96 = sine_at(1000.0, amp, duration, hi_rate);
    let out48 = render_example(pedal, controls, &input48, base_rate, OversamplingFactor::X2);
    let out96 = render_example(pedal, controls, &input96, hi_rate, OversamplingFactor::X2);
    let out96_down = resample_linear(&out96, hi_rate, base_rate);
    let mag48 = goertzel_mag(&out48, base_rate, 1000.0).max(1e-12);
    let mag96 = goertzel_mag(&out96_down, base_rate, 1000.0).max(1e-12);
    let mag_db = 20.0 * (mag48 / mag96).log10().abs();
    let rms_diff = (rms(&out48) - rms(&out96_down)).abs();
    (mag_db, rms_diff)
}

#[test]
fn rat_sample_rate_consistency_clean_setting() {
    let cmp = compare_sample_rates(RAT_PEDAL, RAT_CONTROLS, 0.05, 0.3);
    assert!(
        cmp.0 < 0.5,
        "RAT fundamental drifted across Fs: Δ={:.3} dB",
        cmp.0
    );
    assert!(cmp.1 < 0.02, "RAT RMS changed across Fs: diff={:.4}", cmp.1);
}

#[test]
fn klon_sample_rate_consistency_clean_setting() {
    let cmp = compare_sample_rates(KLON_PEDAL, KLON_CONTROLS, 0.05, 0.3);
    assert!(
        cmp.0 < 0.5,
        "Klon fundamental drifted across Fs: Δ={:.3} dB",
        cmp.0
    );
    assert!(
        cmp.1 < 0.02,
        "Klon RMS changed across Fs: diff={:.4}",
        cmp.1
    );
}

#[test]
fn rc_lowpass_cutoff_matches_prediction() {
    let circuit = r#"
pedal "RC Lowpass" {
  components {
    R1: resistor(10k)
    C1: cap(10n)
  }
  nets {
    in -> R1.a
    R1.b -> C1.a, out
    C1.b -> gnd
  }
}
"#;
    let fc = 1.0 / (2.0 * std::f64::consts::PI * 10_000.0 * 10e-9);
    let mut ratios = Vec::new();
    for &rate in &[48_000.0, 96_000.0] {
        let input = sine_at(fc, 0.5, 0.2, rate);
        let output = compile_and_process(circuit, &input, rate, &[]);
        let ratio = rms(&output) / rms(&input);
        ratios.push((rate, ratio));
    }
    let diff = (ratios[0].1 - ratios[1].1).abs();
    assert!(
        diff < 0.05,
        "RC cutoff response should match across Fs: ratios={:?}",
        ratios
    );
}

#[test]
fn solver_iterations_stable_for_rat_and_klon() {
    let sample_rate = 48_000.0;
    let input = sine_at(800.0, 0.4, 0.25, sample_rate);
    reset_solver_stats();
    let clip_input = sine_at(1000.0, 0.5, 0.1, sample_rate);
    let _ = compile_test_pedal_and_process("clip_silicon.pedal", &clip_input, sample_rate, &[]);
    let clip_stats = solver_stats_snapshot();
    eprintln!("  [diag] clip_silicon stats: {:?}", clip_stats);
    assert!(
        clip_stats.solves > 0,
        "Instrumentation should record solves for clip_silicon"
    );
    for &(pedal, controls) in &[(RAT_PEDAL, RAT_CONTROLS), (KLON_PEDAL, KLON_CONTROLS)] {
        reset_solver_stats();
        let _ = render_example(pedal, controls, &input, sample_rate, OversamplingFactor::X2);
        let first = solver_stats_snapshot();
        eprintln!("  [diag] {pedal} solver stats: {:?}", first);
        reset_solver_stats();
        let _ = render_example(pedal, controls, &input, sample_rate, OversamplingFactor::X2);
        let second = solver_stats_snapshot();

        assert_eq!(
            first, second,
            "Solver stats should be deterministic for {pedal}"
        );
        assert!(
            first.max_iterations <= 16,
            "{pedal} iterations hit limit: {:?}",
            first
        );
        assert!(
            first.max_residual < 1e-3,
            "{pedal} residual too large: {:?}",
            first
        );
        let avg_iter = first.total_iterations as f64 / first.solves as f64;
        assert!(
            avg_iter < 12.0,
            "{pedal} averages too many iterations: avg={avg_iter:.2}, stats={:?}",
            first
        );
    }
}

#[test]
fn rat_handles_near_silence_without_denormals() {
    let sample_rate = 48_000.0;
    let input = sine_at(200.0, 1e-8, 0.3, sample_rate);
    let output = render_example(
        RAT_PEDAL,
        RAT_CONTROLS,
        &input,
        sample_rate,
        OversamplingFactor::X2,
    );
    let subnormals = count_subnormals(&output);
    assert!(
        subnormals == 0,
        "RAT should not produce denormals: found {subnormals}"
    );
    assert!(
        peak(&output) < 1e-4,
        "Near-silence should stay tiny: peak={}",
        peak(&output)
    );
}
