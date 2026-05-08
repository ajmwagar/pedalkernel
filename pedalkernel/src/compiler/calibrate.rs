//! Output level calibration for compiled pedals.
//!
//! Runs a test signal through the pedal and adjusts output_gain
//! to normalize levels. Enabled by the `calibrate` keyword in `.pedal` files.

use super::compiled::CompiledPedal;
use crate::PedalProcessor;

/// Calibrate output gain by running a 1kHz test signal through the pedal.
///
/// Measures the actual output RMS over 500ms and sets `compiled.output_gain`
/// so the pedal produces levels proportional to the input. Resets state afterward.
pub(super) fn calibrate_output(compiled: &mut CompiledPedal) {
    // Input level matching removed — should be a UI slider, not hardcoded.
    // The circuit receives the raw input signal at instrument level.
    // pre_gain stays at 1.0.

    let output_gain = measure_output_gain(compiled);
    compiled.output_gain = output_gain;
    compiled.reset();
}

/// Conservative input attenuation to keep the circuit below saturation.
///
/// Single-coil average: ~50-80mV RMS, peak ~100mV.
/// Pedal circuits have internal gain stages (opamp, transistor) that amplify
/// before nonlinear elements. This keeps the full dynamic range of the gain
/// control by staying below saturation at minimum gain.
fn calibrate_input_level() -> f64 {
    0.03
}

/// Measure the output gain of a compiled pedal using a 1kHz sine test signal.
///
/// Returns the gain scalar needed to normalize output to match input level.
fn measure_output_gain(pedal: &mut CompiledPedal) -> f64 {
    use std::f64::consts::PI;

    let sr = pedal.sample_rate;
    let n = (0.5 * sr) as usize; // 500ms
    let two_pi_f = 2.0 * PI * 1000.0; // 1kHz
    let amp = 0.25; // -12dBFS

    // Warm up (128 samples — let caps settle)
    for i in 0..128usize {
        let _ = pedal.process(amp * (two_pi_f * i as f64 / sr).sin());
    }

    // Measure steady-state output RMS
    let mut sum_sq = 0.0;
    let measure_len = n - 128;
    for i in 128..n {
        let input = amp * (two_pi_f * i as f64 / sr).sin();
        let out = pedal.process(input);
        sum_sq += out * out;
    }
    let output_rms = (sum_sq / measure_len as f64).sqrt();
    let input_rms = amp / std::f64::consts::SQRT_2;

    if output_rms < 1e-10 {
        return 1.0; // Silent — don't amplify noise
    }

    (input_rms / output_rms).clamp(0.01, 100.0)
}
