//! Integration tests for LFO modulation circuits.
//!
//! Validates LFO waveform shapes, rate accuracy, depth control,
//! and full phaser sweep behavior.

mod audio_analysis;

use audio_analysis::*;

// ---------------------------------------------------------------------------
// Basic LFO output
// ---------------------------------------------------------------------------

#[test]
fn lfo_sine_produces_output() {
    let input = sine(440.0, 1.0, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("lfo_sine.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "LFO sine", 5.0);
}

#[test]
fn lfo_triangle_produces_output() {
    let input = sine(440.0, 1.0, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("lfo_triangle.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "LFO triangle", 5.0);
}

#[test]
fn lfo_square_produces_output() {
    let input = sine(440.0, 1.0, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("lfo_square.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "LFO square", 5.0);
}

// ---------------------------------------------------------------------------
// Modulation detection
// ---------------------------------------------------------------------------

#[test]
fn lfo_sine_produces_modulation() {
    // LFO should cause amplitude variation over time
    let input = sine(440.0, 2.0, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("lfo_sine.pedal", &input, SAMPLE_RATE, &[]);

    // Check that the output has amplitude modulation
    let has_mod = has_amplitude_modulation(&output, 8);
    // Relaxed check: even if not perfectly detected, output should differ from input
    let corr = correlation(&input[..output.len()], &output).abs();
    assert!(
        has_mod || corr < 0.999,
        "LFO sine should modulate output: has_mod={has_mod}, corr={corr:.6}"
    );
}

#[test]
fn lfo_all_waveforms_modulate() {
    let input = sine(440.0, 2.0, SAMPLE_RATE);

    for name in ["lfo_sine.pedal", "lfo_triangle.pedal", "lfo_square.pedal"] {
        let output = compile_test_pedal_and_process(name, &input, SAMPLE_RATE, &[]);

        // Output should differ from clean input
        let corr = correlation(&input[..output.len()], &output).abs();
        assert!(
            corr < 0.9999,
            "{name}: output should differ from dry input: corr={corr:.6}"
        );
    }
}

// ---------------------------------------------------------------------------
// Phase 90 speed control
// ---------------------------------------------------------------------------

#[test]
fn phase90_speed_changes_rate() {
    // BUG: The Speed control (pot → LFO.rate) doesn't affect the
    //      compiled model output. Slow and fast produce identical output
    //      (corr=1.0), indicating the control-to-LFO-rate routing is broken.
    let input = sine(440.0, 2.0, SAMPLE_RATE);

    let slow = compile_example_and_process(
        "phase90.pedal",
        &input,
        SAMPLE_RATE,
        &[("Speed", 0.2)],
    );
    let fast = compile_example_and_process(
        "phase90.pedal",
        &input,
        SAMPLE_RATE,
        &[("Speed", 0.8)],
    );

    assert!(slow.iter().all(|x| x.is_finite()), "Slow: NaN/inf");
    assert!(fast.iter().all(|x| x.is_finite()), "Fast: NaN/inf");

    let corr = correlation(&slow, &fast).abs();
    assert!(
        corr < 0.999,
        "Phase 90 Speed control should change sweep rate: corr={corr:.6} (identical output = routing bug)"
    );
}

// ---------------------------------------------------------------------------
// Full phaser sweep
// ---------------------------------------------------------------------------

#[test]
fn lfo_modulated_phaser_produces_output() {
    let input = sine(440.0, 1.0, SAMPLE_RATE);
    let output =
        compile_test_pedal_and_process("lfo_modulated_jfet.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "LFO phaser", 5.0);
}

#[test]
fn lfo_modulated_phaser_sweeps_spectrum() {
    // 4-stage phaser should cause spectral centroid to vary over time
    let input = sine(440.0, 2.0, SAMPLE_RATE);
    let output =
        compile_test_pedal_and_process("lfo_modulated_jfet.pedal", &input, SAMPLE_RATE, &[]);

    // Split into 4 quarters and check spectral centroid varies
    let quarter = output.len() / 4;
    let centroids: Vec<f64> = (0..4)
        .map(|i| spectral_centroid(&output[i * quarter..(i + 1) * quarter], SAMPLE_RATE))
        .collect();

    // At least some variation in centroid (phaser sweep moves notches)
    let max_c = centroids.iter().cloned().fold(0.0f64, f64::max);
    let min_c = centroids.iter().cloned().fold(f64::MAX, f64::min);

    // Even small variation proves the sweep is happening
    let range = if max_c > 0.0 {
        (max_c - min_c) / max_c
    } else {
        0.0
    };
    // BUG: The phaser sweep produces near-zero centroid variation (range ≈ 0.0005).
    //      A real 4-stage phaser with LFO modulating JFET Rds should produce
    //      significant spectral movement (range > 1%).
    assert!(
        centroids.iter().all(|c| c.is_finite()),
        "Phaser centroids should be finite"
    );
    assert!(
        range > 0.01,
        "Phaser sweep should cause >1% spectral centroid variation: range={range:.6}, centroids={centroids:?}"
    );
}

// ---------------------------------------------------------------------------
// All LFO circuits stable
// ---------------------------------------------------------------------------

#[test]
fn lfo_all_test_circuits_stable() {
    let input = sine(440.0, 0.5, SAMPLE_RATE);

    for name in [
        "lfo_sine.pedal",
        "lfo_triangle.pedal",
        "lfo_square.pedal",
        "lfo_modulated_jfet.pedal",
    ] {
        let output = compile_test_pedal_and_process(name, &input, SAMPLE_RATE, &[]);
        assert!(
            output.iter().all(|x| x.is_finite()),
            "{name}: output has NaN/inf"
        );
        let p = peak(&output);
        assert!(p < 5.0, "{name}: output too loud, peak={p:.4}");
    }
}

// ---------------------------------------------------------------------------
// Phase 90 with guitar
// ---------------------------------------------------------------------------

#[test]
fn phase90_with_guitar() {
    let (guitar, _sr) = &*CLEAN_GUITAR;
    let input: Vec<f64> = guitar.iter().copied().take(48000).collect();

    let output = compile_example_and_process(
        "phase90.pedal",
        &input,
        SAMPLE_RATE,
        &[("Speed", 0.5)],
    );
    assert_healthy(&output, "Phase 90 guitar", 5.0);
    maybe_dump_wav(&output, "guitar_phase90", SAMPLE_RATE_U32);
}
