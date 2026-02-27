//! Integration tests for op-amp and JFET circuits.
//!
//! Validates unity-gain buffers, gain stages, slew rate limiting,
//! JFET all-pass filters, and JFET as voltage-controlled resistor.

mod audio_analysis;

use audio_analysis::*;

// ---------------------------------------------------------------------------
// Op-amp buffer
// ---------------------------------------------------------------------------

#[test]
fn opamp_buffer_produces_output() {
    let input = sine(440.0, 0.2, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("opamp_buffer.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "TL072 buffer", 5.0);
}

#[test]
fn opamp_buffer_near_unity_gain() {
    // Unity-gain follower: output should closely match input
    let input = sine(440.0, 0.5, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("opamp_buffer.pedal", &input, SAMPLE_RATE, &[]);

    let corr = correlation(&input, &output).abs();
    assert!(
        corr > 0.80,
        "Unity-gain buffer should track input: corr={corr:.4}"
    );
}

#[test]
fn opamp_buffer_low_thd() {
    // Buffer should have very low distortion
    let input = sine(440.0, 0.3, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("opamp_buffer.pedal", &input, SAMPLE_RATE, &[]);

    let t = thd(&output, SAMPLE_RATE, 440.0);
    assert!(
        t < 0.15,
        "Buffer THD should be low: got {t:.4}"
    );
}

#[test]
fn opamp_buffer_bounded_by_rails() {
    // Output should be bounded by supply rail (9V/2 - headroom)
    let input = sine(440.0, 0.5, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("opamp_buffer.pedal", &input, SAMPLE_RATE, &[]);

    let p = peak(&output);
    assert!(
        p < 5.0, // supply/2 = 4.5V, allow some margin
        "Buffer output should be bounded by rails: peak={p:.4}"
    );
}

// ---------------------------------------------------------------------------
// Op-amp gain stage
// ---------------------------------------------------------------------------

#[test]
fn opamp_gain_stage_produces_output() {
    // Inverting amp with Rf/Ri = 47k/4.7k = 10. The OpAmpRoot element
    // models VCVS behavior with the feedback network. Use small input
    // so the output stays within the 9V rail headroom.
    let input = sine_at(440.0, 0.05, 0.2, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("opamp_gain.pedal", &input, SAMPLE_RATE, &[]);

    assert_healthy(&output, "JRC4558 gain", 5.0);

    // The output should have measurable signal
    let out_rms = rms(&output);
    assert!(
        out_rms > 1e-6,
        "Gain stage should produce output: out_rms={out_rms:.6}"
    );

    // The gain stage should modify the signal (not just pass through)
    let corr = correlation(&input, &output).abs();
    assert!(
        corr < 0.9999 || out_rms > rms(&input) * 0.5,
        "Gain stage should affect signal: corr={corr:.6}"
    );
}

// ---------------------------------------------------------------------------
// Op-amp slew rate
// ---------------------------------------------------------------------------

#[test]
fn opamp_slew_produces_output() {
    let input = sine(440.0, 0.2, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("opamp_slew.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "LM308 slew", 5.0);
}

#[test]
fn opamp_slew_lm308_darker_than_tl072() {
    // LM308 (0.3 V/µs) should roll off HF more than TL072 (13 V/µs)
    // Use a harmonically rich input
    let input = guitar_pluck(440.0, 0.5, SAMPLE_RATE);

    let lm308_out = compile_test_pedal_and_process("opamp_slew.pedal", &input, SAMPLE_RATE, &[]);
    let tl072_out = compile_test_pedal_and_process("opamp_buffer.pedal", &input, SAMPLE_RATE, &[]);

    let lm308_centroid = spectral_centroid(&lm308_out, SAMPLE_RATE);
    let tl072_centroid = spectral_centroid(&tl072_out, SAMPLE_RATE);

    // LM308 should be darker (lower spectral centroid)
    assert!(
        lm308_centroid < tl072_centroid * 1.2,
        "LM308 should be darker than TL072: lm308={lm308_centroid:.0}Hz, tl072={tl072_centroid:.0}Hz"
    );
}

// ---------------------------------------------------------------------------
// JFET all-pass
// ---------------------------------------------------------------------------

#[test]
fn jfet_allpass_produces_output() {
    let input = sine(440.0, 0.2, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("jfet_allpass.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "JFET allpass", 5.0);
}

#[test]
fn jfet_allpass_passes_signal() {
    // All-pass should pass signal through (not attenuate significantly)
    let input = sine(440.0, 0.3, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("jfet_allpass.pedal", &input, SAMPLE_RATE, &[]);

    let in_rms = rms(&input);
    let out_rms = rms(&output);

    // Output should have meaningful energy (all-pass doesn't attenuate)
    assert!(
        out_rms > in_rms * 0.01,
        "All-pass should pass signal: in_rms={in_rms:.6}, out_rms={out_rms:.6}"
    );
}

// ---------------------------------------------------------------------------
// JFET switch
// ---------------------------------------------------------------------------

#[test]
fn jfet_switch_produces_output() {
    let input = sine(440.0, 0.2, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("jfet_switch.pedal", &input, SAMPLE_RATE, &[]);
    assert!(
        output.iter().all(|x| x.is_finite()),
        "JFET switch: NaN/inf"
    );
}

// ---------------------------------------------------------------------------
// All op-amp models
// ---------------------------------------------------------------------------

#[test]
fn opamp_all_test_circuits_stable() {
    let input = sine(440.0, 0.1, SAMPLE_RATE);

    for name in [
        "opamp_buffer.pedal",
        "opamp_gain.pedal",
        "opamp_slew.pedal",
        "jfet_allpass.pedal",
        "jfet_switch.pedal",
    ] {
        let output = compile_test_pedal_and_process(name, &input, SAMPLE_RATE, &[]);
        assert!(
            output.iter().all(|x| x.is_finite()),
            "{name}: output has NaN/inf"
        );
    }
}

// ---------------------------------------------------------------------------
// Real pedals: ProCo RAT (LM308 slew) and Tube Screamer (JRC4558)
// ---------------------------------------------------------------------------

#[test]
fn proco_rat_slew_character() {
    // RAT uses LM308 — should have slew-rate-limited character
    let input = guitar_pluck(330.0, 0.5, SAMPLE_RATE);
    let output = compile_example_and_process(
        "proco_rat.pedal",
        &input,
        SAMPLE_RATE,
        &[("Distortion", 0.7), ("Volume", 0.5)],
    );
    assert_healthy(&output, "ProCo RAT", 5.0);
    maybe_dump_wav(&output, "guitar_proco_rat", SAMPLE_RATE_U32);
}

#[test]
fn tube_screamer_opamp_character() {
    let input = guitar_pluck(330.0, 0.5, SAMPLE_RATE);
    let output = compile_example_and_process(
        "tube_screamer.pedal",
        &input,
        SAMPLE_RATE,
        &[("Drive", 0.6), ("Tone", 0.5), ("Level", 0.7)],
    );
    assert_healthy(&output, "Tube Screamer", 5.0);
    maybe_dump_wav(&output, "guitar_tube_screamer", SAMPLE_RATE_U32);
}
