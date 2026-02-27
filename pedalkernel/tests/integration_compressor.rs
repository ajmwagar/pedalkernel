//! Integration tests for compressor-related components:
//! envelope followers, OTA (CA3080), and photocouplers (Vactrols).
//!
//! These components form the compression signal chain used in pedals
//! like the MXR Dyna Comp and LA-2A-style compressors.

mod audio_analysis;

use audio_analysis::*;

// ===========================================================================
// OTA (CA3080) — current-controlled transconductance amplifier
// ===========================================================================

#[test]
fn ota_ca3080_produces_output() {
    let input = sine(440.0, 0.5, SAMPLE_RATE);
    let output =
        compile_test_pedal_and_process("ota_ca3080.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "CA3080 OTA", 50.0);
}

// ===========================================================================
// Photocouplers (Vactrols)
// ===========================================================================

#[test]
fn photocoupler_vtl5c3_produces_output() {
    let input = sine(440.0, 0.5, SAMPLE_RATE);
    let output =
        compile_test_pedal_and_process("photocoupler_vtl5c3.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "VTL5C3 photocoupler", 50.0);
}

#[test]
fn photocoupler_t4b_produces_output() {
    let input = sine(440.0, 0.5, SAMPLE_RATE);
    let output =
        compile_test_pedal_and_process("photocoupler_t4b.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "T4B photocoupler", 50.0);
}

// ===========================================================================
// Envelope follower
// ===========================================================================

#[test]
fn envelope_follower_produces_output() {
    let input = sine(440.0, 0.5, SAMPLE_RATE);
    let output =
        compile_test_pedal_and_process("envelope_follower.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "Envelope follower", 50.0);
}

// ===========================================================================
// Dyna Comp compressor — full signal chain
// ===========================================================================

#[test]
fn dyna_comp_compresses_dynamics() {
    // A compressor should reduce the difference between loud and quiet signals.
    // Feed a loud burst followed by a quiet burst and measure the RMS ratio.
    let loud = sine_at(440.0, 0.8, 0.25, SAMPLE_RATE);
    let quiet = sine_at(440.0, 0.1, 0.25, SAMPLE_RATE);
    let input: Vec<f64> = loud.into_iter().chain(quiet).collect();

    let output = compile_example_and_process(
        "dyna_comp.pedal",
        &input,
        SAMPLE_RATE,
        &[("Sensitivity", 0.8), ("Output", 0.7)],
    );
    assert_healthy(&output, "Dyna Comp compression", 50.0);

    // Measure RMS of first and second halves
    let half = output.len() / 2;
    let out_loud_rms = rms(&output[..half]);
    let out_quiet_rms = rms(&output[half..]);

    // Input ratio is 8:1, compressed ratio should be smaller
    let input_ratio = 0.8 / 0.1;
    let output_ratio = if out_quiet_rms > 1e-10 {
        out_loud_rms / out_quiet_rms
    } else {
        f64::INFINITY
    };

    eprintln!(
        "  [diag] Compression: input_ratio={input_ratio:.1}, output_ratio={output_ratio:.2}, \
         loud_rms={out_loud_rms:.6}, quiet_rms={out_quiet_rms:.6}"
    );

    // The output ratio should be less than the input ratio (compression occurred)
    assert!(
        output_ratio < input_ratio,
        "Compressor should reduce dynamic range: input_ratio={input_ratio:.1}, output_ratio={output_ratio:.2}"
    );
}
