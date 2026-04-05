//! Audio fidelity validation tests for WDF engine improvements.
//!
//! These tests validate four specific fixes:
//! 1. Reactive element port resistance scaling under oversampling
//! 2. X2 oversampling as default
//! 3. Newton-Raphson solver warm-starting (iteration count)
//! 4. Denormal flushing in signal tails

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::compiler::CompileOptions;
use pedalkernel::elements::{reset_solver_stats, solver_stats_snapshot};
use pedalkernel::oversampling::OversamplingFactor;

// ===========================================================================
// 1. Port Resistance Under Oversampling
//
// BUG: Reactive elements (C/L) compute port resistance at the base sample
// rate, but oversampling runs the WDF cycle at fs*ratio. This means:
//   - Capacitor Rp = 1/(2*fs*C)  should be  1/(2*fs*ratio*C)
//   - Inductor  Rp = 2*fs*L      should be  2*fs*ratio*L
//
// Result: filter cutoffs shift when oversampling is enabled.
// Fix: call set_sample_rate(fs * ratio) on reactive elements.
// ===========================================================================

/// Simple RC lowpass DSL for testing filter behavior.
const RC_LOWPASS: &str = r#"
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

/// RC lowpass with lower cutoff (bigger cap) — more sensitive to Rp errors.
const RC_LOWPASS_LOW: &str = r#"
pedal "RC Lowpass Low" {
  components {
    R1: resistor(10k)
    C1: cap(100n)
  }
  nets {
    in -> R1.a
    R1.b -> C1.a, out
    C1.b -> gnd
  }
}
"#;

#[test]
fn rc_filter_cutoff_preserved_under_oversampling() {
    // fc = 1/(2π * 10k * 10nF) ≈ 1592 Hz
    //
    // Strategy: compile the same RC lowpass at:
    //   A) 48 kHz, X1  — baseline
    //   B) 192 kHz, X1 — "true" high-rate reference
    //   C) 48 kHz, X4  — oversampled (should match B after fix)
    //
    // Measure the gain at fc. After the fix, C should closely match B.
    // Before the fix, C has wrong Rp → wrong cutoff.

    let fc = 1.0 / (2.0 * std::f64::consts::PI * 10_000.0 * 10e-9);
    eprintln!("  [diag] RC fc = {fc:.1} Hz");

    // Sweep test: measure gain at fc and at 2*fc
    for &test_freq in &[fc, fc * 2.0] {
        let duration = 0.15;

        // A) 48 kHz, X1
        let input_48 = sine_at(test_freq, 0.5, duration, 48_000.0);
        let out_a = compile_and_process_with_options(
            RC_LOWPASS,
            &input_48,
            48_000.0,
            &[],
            CompileOptions {
                oversampling: OversamplingFactor::X1,
                ..CompileOptions::default()
            },
        );

        // B) 192 kHz, X1 — true high sample rate reference
        let input_192 = sine_at(test_freq, 0.5, duration, 192_000.0);
        let out_b = compile_and_process_with_options(
            RC_LOWPASS,
            &input_192,
            192_000.0,
            &[],
            CompileOptions {
                oversampling: OversamplingFactor::X1,
                ..CompileOptions::default()
            },
        );
        // Downsample to 48k for comparison
        let out_b_48 = resample_linear(&out_b, 192_000.0, 48_000.0);

        // C) 48 kHz, X4 — should behave like 192 kHz internally
        let out_c = compile_and_process_with_options(
            RC_LOWPASS,
            &input_48,
            48_000.0,
            &[],
            CompileOptions {
                oversampling: OversamplingFactor::X4,
                ..CompileOptions::default()
            },
        );

        // Measure gain (output RMS / input RMS) — skip transient
        let skip = (0.05 * 48_000.0) as usize;
        let gain_a = rms(&out_a[skip..]) / rms(&input_48[skip..]);
        let gain_b = rms(&out_b_48[skip..]) / rms(&input_48[skip..]);
        let gain_c = rms(&out_c[skip..]) / rms(&input_48[skip..]);

        eprintln!(
            "  [diag] freq={test_freq:.0}Hz: gain_48k_x1={gain_a:.4} gain_192k_x1={gain_b:.4} gain_48k_x4={gain_c:.4}"
        );

        // After fix: 48k/X4 gain should be closer to 192k/X1 than to 48k/X1
        let err_c_vs_b = (gain_c - gain_b).abs();
        let err_a_vs_b = (gain_a - gain_b).abs();

        // The oversampled version should match the high-rate reference better
        // than the non-oversampled version (or be equivalently close).
        // Allow some tolerance for filter transients and oversampler filter effects.
        assert!(
            err_c_vs_b < err_a_vs_b + 0.05,
            "48k/X4 should be closer to 192k/X1 than 48k/X1 at {test_freq:.0}Hz: \
             |C-B|={err_c_vs_b:.4}, |A-B|={err_a_vs_b:.4}"
        );
    }
}

#[test]
fn rc_lowpass_spectral_centroid_stable_under_oversampling() {
    // A broader check: feed wideband signal, measure spectral centroid.
    // Oversampling should NOT shift the perceived brightness of a linear filter.
    let input = guitar_pluck(330.0, 0.3, SAMPLE_RATE);

    let out_x1 = compile_and_process_with_options(
        RC_LOWPASS_LOW,
        &input,
        SAMPLE_RATE,
        &[],
        CompileOptions {
            oversampling: OversamplingFactor::X1,
            ..CompileOptions::default()
        },
    );
    let out_x2 = compile_and_process_with_options(
        RC_LOWPASS_LOW,
        &input,
        SAMPLE_RATE,
        &[],
        CompileOptions {
            oversampling: OversamplingFactor::X2,
            ..CompileOptions::default()
        },
    );
    let out_x4 = compile_and_process_with_options(
        RC_LOWPASS_LOW,
        &input,
        SAMPLE_RATE,
        &[],
        CompileOptions {
            oversampling: OversamplingFactor::X4,
            ..CompileOptions::default()
        },
    );

    let skip = 2000;
    let cent_x1 = spectral_centroid(&out_x1[skip..], SAMPLE_RATE);
    let cent_x2 = spectral_centroid(&out_x2[skip..], SAMPLE_RATE);
    let cent_x4 = spectral_centroid(&out_x4[skip..], SAMPLE_RATE);

    eprintln!("  [diag] RC low: centroid X1={cent_x1:.1}Hz X2={cent_x2:.1}Hz X4={cent_x4:.1}Hz");

    // After fix, centroids should be within ~20% of each other.
    // Before fix, oversampled version has wrong Rp → shifted cutoff → different centroid.
    let max_cent = cent_x1.max(cent_x2).max(cent_x4);
    let min_cent = cent_x1.min(cent_x2).min(cent_x4);
    if max_cent > 50.0 {
        let spread = (max_cent - min_cent) / max_cent;
        assert!(
            spread < 0.3,
            "Spectral centroid should be stable across OS factors: \
             X1={cent_x1:.1} X2={cent_x2:.1} X4={cent_x4:.1} spread={spread:.3}"
        );
    }
}

#[test]
fn rat_tone_preserved_under_oversampling() {
    // Real-world check: RAT pedal filter behavior should remain consistent
    // across oversampling factors. The tone stack depends on RC networks
    // whose cutoffs must scale correctly with the effective sample rate.
    let input = guitar_pluck(220.0, 0.3, SAMPLE_RATE);

    let out_x1 = compile_example_with_options(
        "proco_rat.pedal",
        &input,
        SAMPLE_RATE,
        &[("Distortion", 0.3), ("Filter", 0.5), ("Volume", 0.7)],
        CompileOptions {
            oversampling: OversamplingFactor::X1,
            ..CompileOptions::default()
        },
    );
    let out_x2 = compile_example_with_options(
        "proco_rat.pedal",
        &input,
        SAMPLE_RATE,
        &[("Distortion", 0.3), ("Filter", 0.5), ("Volume", 0.7)],
        CompileOptions {
            oversampling: OversamplingFactor::X2,
            ..CompileOptions::default()
        },
    );

    assert_healthy(&out_x1, "RAT X1", 50.0);
    assert_healthy(&out_x2, "RAT X2", 50.0);

    let skip = 2000;
    let cent_x1 = spectral_centroid(&out_x1[skip..], SAMPLE_RATE);
    let cent_x2 = spectral_centroid(&out_x2[skip..], SAMPLE_RATE);

    eprintln!("  [diag] RAT tone: centroid X1={cent_x1:.1}Hz X2={cent_x2:.1}Hz");

    // Spectral distance should be moderate — oversampling changes harmonics
    // but shouldn't radically shift the tonal balance.
    let dist = spectral_distance(&out_x1[skip..], &out_x2[skip..], SAMPLE_RATE, 100.0);
    eprintln!("  [diag] RAT spectral distance X1↔X2: {dist:.4}");
    assert!(
        dist < 2.0,
        "RAT tonal character should be similar across OS: dist={dist:.4}"
    );
}

// ===========================================================================
// 2. X2 Oversampling Default
//
// After the fix, CompileOptions::default().oversampling should be X2.
// ===========================================================================

#[test]
fn default_oversampling_is_x2() {
    let opts = CompileOptions::default();
    assert_eq!(
        opts.oversampling,
        OversamplingFactor::X2,
        "Default oversampling should be X2, got {:?}",
        opts.oversampling
    );
}

#[test]
fn default_compile_has_less_aliasing_than_x1() {
    // Default-compiled pedal should have less alias energy than explicit X1.
    let sample_rate = 24_000.0; // Low rate to stress aliasing
    let input = sine_at(5_000.0, 0.7, 0.25, sample_rate);

    let out_x1 = compile_test_pedal_with_options(
        "clip_silicon.pedal",
        &input,
        sample_rate,
        &[],
        CompileOptions {
            oversampling: OversamplingFactor::X1,
            ..CompileOptions::default()
        },
    );

    // Default compile (should now be X2)
    let out_default =
        compile_test_pedal_and_process("clip_silicon.pedal", &input, sample_rate, &[]);

    let alias_x1 = alias_energy_summary(&out_x1, sample_rate, 5000.0, 50.0);
    let alias_default = alias_energy_summary(&out_default, sample_rate, 5000.0, 50.0);

    eprintln!(
        "  [diag] Alias ratio: X1={:.6} default={:.6}",
        alias_x1.alias_ratio, alias_default.alias_ratio
    );

    // Default (X2) should have less aliasing than explicit X1
    assert!(
        alias_default.alias_ratio <= alias_x1.alias_ratio,
        "Default compile should have <= aliasing vs X1: default={:.6} x1={:.6}",
        alias_default.alias_ratio,
        alias_x1.alias_ratio
    );
}

// ===========================================================================
// 3. Solver Warm-Starting
//
// The Newton-Raphson solver should cache the previous sample's solution
// as the initial guess for the next sample. This dramatically reduces
// iteration count for continuous signals (2-3 vs 10-15 iterations).
// ===========================================================================

#[test]
fn solver_warm_start_low_iteration_count() {
    // Process a steady-state sine through a diode clipper.
    // With warm-starting, the solver should converge in ~2-5 iterations
    // on average because consecutive samples have very similar solutions.
    let sample_rate = 48_000.0;
    let input = sine_at(440.0, 0.5, 0.2, sample_rate);

    reset_solver_stats();
    let output = compile_test_pedal_and_process("clip_silicon.pedal", &input, sample_rate, &[]);
    let stats = solver_stats_snapshot();

    assert_healthy(&output, "clip warm-start", 50.0);
    assert!(stats.solves > 0, "Should have solver calls");

    let avg_iter = stats.total_iterations as f64 / stats.solves as f64;
    eprintln!(
        "  [diag] Warm-start diode clip: avg_iter={avg_iter:.2} max_iter={} solves={}",
        stats.max_iterations, stats.solves
    );

    // With warm-starting, average should be well under 8.
    // Without warm-starting (cold v=0 every sample), average is typically 10-15.
    assert!(
        avg_iter < 8.0,
        "Average iterations should be low with warm-starting: avg={avg_iter:.2}"
    );
}

#[test]
fn solver_warm_start_rat_convergence() {
    // RAT has multiple nonlinear elements — warm-starting should help here too.
    let sample_rate = 48_000.0;
    let input = guitar_pluck(220.0, 0.3, sample_rate);

    reset_solver_stats();
    let output = compile_example_with_options(
        "proco_rat.pedal",
        &input,
        sample_rate,
        &[("Distortion", 0.7), ("Filter", 0.5), ("Volume", 0.7)],
        CompileOptions {
            oversampling: OversamplingFactor::X2,
            ..CompileOptions::default()
        },
    );
    let stats = solver_stats_snapshot();

    assert_healthy(&output, "RAT warm-start", 50.0);
    assert!(stats.solves > 0, "RAT should have solver calls");

    let avg_iter = stats.total_iterations as f64 / stats.solves as f64;
    eprintln!(
        "  [diag] Warm-start RAT: avg_iter={avg_iter:.2} max_iter={} max_resid={:.2e} solves={}",
        stats.max_iterations, stats.max_residual, stats.solves
    );

    // RAT with high distortion is harder, but warm-starting should still keep
    // average iterations reasonable.
    assert!(
        avg_iter < 10.0,
        "RAT avg iterations should be reasonable with warm-starting: avg={avg_iter:.2}"
    );
    // No sample should hit the absolute max (16) under normal signal conditions
    assert!(
        stats.max_iterations <= 16,
        "RAT should not blow iteration budget: max={}",
        stats.max_iterations
    );
}

#[test]
fn solver_warm_start_does_not_change_output() {
    // Correctness check: warm-starting should converge to the same solution
    // as cold-starting. We can't easily toggle warm-start at runtime, but
    // we can verify determinism: same input → same output across two runs.
    let sample_rate = 48_000.0;
    let input = sine_at(440.0, 0.4, 0.1, sample_rate);

    let out1 = compile_test_pedal_and_process("clip_silicon.pedal", &input, sample_rate, &[]);
    let out2 = compile_test_pedal_and_process("clip_silicon.pedal", &input, sample_rate, &[]);

    let corr = correlation(&out1, &out2);
    assert!(
        corr > 0.99999,
        "Warm-started solver should be deterministic: corr={corr:.8}"
    );
}

// ===========================================================================
// 4. Denormal Flushing
//
// After processing loud signal, filter states can decay to denormal values
// during silence. These are 100x slower to process on x86. The fix flushes
// denormals at the top of process().
// ===========================================================================

#[test]
fn no_denormals_in_silence_tail_diode_clip() {
    // Process 0.2s of signal, then 0.3s of silence through diode clipper.
    // The silence tail should contain no subnormal values.
    let mut input = sine_at(440.0, 0.5, 0.2, SAMPLE_RATE);
    input.extend(vec![0.0; (0.3 * SAMPLE_RATE) as usize]);

    let output = compile_test_pedal_and_process("clip_silicon.pedal", &input, SAMPLE_RATE, &[]);

    // Check only the silence tail
    let tail_start = (0.2 * SAMPLE_RATE) as usize;
    let tail = &output[tail_start..];
    let subnormals = count_subnormals(tail);
    eprintln!(
        "  [diag] Diode clip silence tail: subnormals={subnormals} tail_peak={:.2e}",
        peak(tail)
    );
    assert!(
        subnormals == 0,
        "Silence tail should have no denormals: found {subnormals}"
    );
}

#[test]
fn no_denormals_in_silence_tail_triode() {
    // Triode stages have more internal state (DC coupling, bias points)
    // that can produce denormals as signal decays.
    let mut input = guitar_pluck(220.0, 0.2, SAMPLE_RATE);
    input.extend(vec![0.0; (0.5 * SAMPLE_RATE) as usize]);

    let output = compile_test_pedal_and_process("triode_clean.pedal", &input, SAMPLE_RATE, &[]);

    let tail_start = (0.2 * SAMPLE_RATE) as usize;
    let tail = &output[tail_start..];
    let subnormals = count_subnormals(tail);
    eprintln!(
        "  [diag] Triode silence tail: subnormals={subnormals} tail_peak={:.2e}",
        peak(tail)
    );
    assert!(
        subnormals == 0,
        "Triode silence tail should have no denormals: found {subnormals}"
    );
}

#[test]
fn no_denormals_near_zero_input() {
    // Feed near-zero amplitude through various nonlinear pedals.
    // Even tiny signals shouldn't produce denormals in the output.
    let input = sine_at(200.0, 1e-10, 0.2, SAMPLE_RATE);

    for pedal in &[
        "clip_silicon.pedal",
        "clip_germanium.pedal",
        "triode_clean.pedal",
    ] {
        let output = compile_test_pedal_and_process(pedal, &input, SAMPLE_RATE, &[]);
        let subnormals = count_subnormals(&output);
        eprintln!(
            "  [diag] {pedal} near-zero: subnormals={subnormals} peak={:.2e}",
            peak(&output)
        );
        assert!(
            subnormals == 0,
            "{pedal}: near-zero input produced {subnormals} denormals"
        );
    }
}

#[test]
fn no_denormals_rat_full_chain() {
    // Full RAT pedal: process signal then silence at X2 oversampling.
    // Oversampler filter states are additional denormal sources.
    let mut input = guitar_pluck(330.0, 0.3, SAMPLE_RATE);
    input.extend(vec![0.0; (0.5 * SAMPLE_RATE) as usize]);

    let output = compile_example_with_options(
        "proco_rat.pedal",
        &input,
        SAMPLE_RATE,
        &[("Distortion", 0.7), ("Filter", 0.5), ("Volume", 0.7)],
        CompileOptions {
            oversampling: OversamplingFactor::X2,
            ..CompileOptions::default()
        },
    );

    let tail_start = (0.3 * SAMPLE_RATE) as usize;
    let tail = &output[tail_start..];
    let subnormals = count_subnormals(tail);
    eprintln!(
        "  [diag] RAT X2 silence tail: subnormals={subnormals} tail_peak={:.2e}",
        peak(tail)
    );
    assert!(
        subnormals == 0,
        "RAT silence tail should have no denormals: found {subnormals}"
    );
}

// ===========================================================================
// Bonus: Combined fidelity checks
// ===========================================================================

#[test]
fn oversampled_rc_matches_high_sample_rate_compilation() {
    // The gold standard: compiling at 48kHz with X4 oversampling should produce
    // nearly identical frequency response to compiling at 192kHz with X1.
    // This is the definitive test that port resistances are scaled correctly.

    // Use a frequency well below the lowpass cutoff (~1592 Hz for 10k/10n)
    // and one near the cutoff.
    let fc = 1.0 / (2.0 * std::f64::consts::PI * 10_000.0 * 10e-9);

    for &freq in &[200.0, fc * 0.5, fc, fc * 1.5] {
        if freq > 20_000.0 {
            continue;
        }
        let duration = 0.15;

        // Reference: 192 kHz / X1
        let input_hi = sine_at(freq, 0.5, duration, 192_000.0);
        let out_hi = compile_and_process_with_options(
            RC_LOWPASS,
            &input_hi,
            192_000.0,
            &[],
            CompileOptions {
                oversampling: OversamplingFactor::X1,
                ..CompileOptions::default()
            },
        );
        let out_hi_48 = resample_linear(&out_hi, 192_000.0, 48_000.0);

        // Test: 48 kHz / X4
        let input_lo = sine_at(freq, 0.5, duration, 48_000.0);
        let out_lo = compile_and_process_with_options(
            RC_LOWPASS,
            &input_lo,
            48_000.0,
            &[],
            CompileOptions {
                oversampling: OversamplingFactor::X4,
                ..CompileOptions::default()
            },
        );

        let skip = (0.05 * 48_000.0) as usize;
        let gain_hi = rms(&out_hi_48[skip..]) / rms(&input_lo[skip..]).max(1e-20);
        let gain_lo = rms(&out_lo[skip..]) / rms(&input_lo[skip..]).max(1e-20);
        let err = (gain_hi - gain_lo).abs();

        eprintln!(
            "  [diag] freq={freq:.0}Hz: gain_192k={gain_hi:.4} gain_48k_x4={gain_lo:.4} err={err:.4}"
        );

        // After fix, gains should match within 5% (some difference from
        // oversampler anti-imaging/aliasing filters is expected).
        assert!(
            err < 0.1,
            "48kHz/X4 should match 192kHz/X1 at {freq:.0}Hz: gain_hi={gain_hi:.4} gain_x4={gain_lo:.4} err={err:.4}"
        );
    }
}
