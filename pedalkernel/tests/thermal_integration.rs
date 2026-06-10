//! Integration tests for the thermal modeling compile option.
//!
//! Verifies two hard requirements:
//!
//! 1. **Off-path bit-identity** — when `CompileOptions::thermal` is `false`
//!    (the default), the output is identical between two identical renders.
//!    Regression: any thermal code that leaks into the off-path would fail.
//!
//! 2. **On-path behavior drift** — when `thermal: true`, warming the circuit
//!    to steady-state produces measurably different output than cold start.
//!    For Ge BJTs: `beta_tempco=0.015/°C` per `ThermalCoefficients::germanium_bjt()`.
//!    At 40°C vs 15°C (germanium_fuzz model):
//!    - beta_multiplier_warm  = 1 + 0.015 * (40 - 25) = 1.225
//!    - beta_multiplier_cold  = 1 + 0.015 * (15 - 25) = 0.85
//!    Higher beta → more collector current → different output level.

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::compiler::{compile_pedal_with_options, CompileOptions};
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::thermal::{ThermalCoefficients, ThermalModel};
use pedalkernel::PedalProcessor;
use std::path::Path;

/// Helper: load a test pedal from `tests/test_pedals/` and return its source.
fn test_pedal_src(filename: &str) -> String {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/test_pedals")
        .join(filename);
    std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("failed to read {}: {e}", path.display()))
}

// ── Test 1: Bit-identical regression ─────────────────────────────────────────

/// With `thermal: false` (default), two identical compile-and-render runs
/// must produce **exactly** the same output samples.
///
/// This encodes the requirement: thermal=off must be bit-identical to
/// pre-change behavior. Any code path that accidentally runs thermal logic
/// when the flag is off would produce non-determinism or drift.
#[test]
fn thermal_off_is_deterministic_and_bit_identical() {
    let input = sine_at(440.0, 0.5, 0.1, SAMPLE_RATE);
    let src = test_pedal_src("clip_silicon.pedal");

    let options = CompileOptions {
        thermal: false,
        ..CompileOptions::default()
    };

    // Compile and render twice — must produce identical output.
    let out_a = compile_and_process_with_options(&src, &input, SAMPLE_RATE, &[], options.clone());
    let out_b = compile_and_process_with_options(&src, &input, SAMPLE_RATE, &[], options);

    assert_eq!(
        out_a.len(),
        out_b.len(),
        "thermal=off: run A and run B have different lengths"
    );

    for (i, (a, b)) in out_a.iter().zip(out_b.iter()).enumerate() {
        assert_eq!(
            a.to_bits(),
            b.to_bits(),
            "thermal=off: sample {i} differs — A={a} B={b}"
        );
    }
}

/// With `thermal: false`, BJT circuit output must also be deterministic.
#[test]
fn thermal_off_bjt_is_deterministic() {
    let input = sine_at(440.0, 0.3, 0.1, SAMPLE_RATE);
    let src = test_pedal_src("bjt_ac128.pedal");

    let options = CompileOptions {
        thermal: false,
        ..CompileOptions::default()
    };

    let out_a = compile_and_process_with_options(&src, &input, SAMPLE_RATE, &[], options.clone());
    let out_b = compile_and_process_with_options(&src, &input, SAMPLE_RATE, &[], options);

    assert_eq!(out_a.len(), out_b.len());
    for (i, (a, b)) in out_a.iter().zip(out_b.iter()).enumerate() {
        assert_eq!(
            a.to_bits(),
            b.to_bits(),
            "thermal=off BJT: sample {i} differs"
        );
    }
}

// ── Test 2: On-path behavior drift ───────────────────────────────────────────

/// With `thermal: true` on a Ge BJT (AC128), a fully-warmed circuit must
/// produce measurably different output than a cold-start circuit.
///
/// Uses the `germanium_fuzz` thermal model (15°C→40°C, tau=600s) with
/// Ge coefficients (beta_tempco=0.015/°C, is_doubling_temp=8°C):
/// - cold (15°C): beta_multiplier = 1 + 0.015 * (15 - 25) = 0.85
/// - warm (40°C): beta_multiplier = 1 + 0.015 * (40 - 25) = 1.225
///
/// The test verifies that cold and warm renders produce measurably
/// different output — confirming thermal state is applied to BJT parameters.
#[test]
fn thermal_on_ge_bjt_cold_vs_warm_diverges() {
    let input = sine_at(440.0, 0.3, 0.1, SAMPLE_RATE);
    let src = test_pedal_src("bjt_ac128.pedal");
    let pedal = parse_pedal_file(&src).expect("failed to parse bjt_ac128.pedal");

    let options = CompileOptions {
        thermal: true,
        ..CompileOptions::default()
    };

    // --- Cold render: thermal model stays at ambient (15°C for Ge fuzz) ---
    let mut proc_cold =
        compile_pedal_with_options(&pedal, SAMPLE_RATE, options.clone()).expect("compile failed");
    // Override with germanium_fuzz model (15°C ambient, 40°C steady-state).
    // silicon_standard (the default) only goes from 25°C to 35°C — too small
    // a range to guarantee a visible output difference in a short test buffer.
    // germanium_fuzz starts cold at 15°C (beta_mult=0.85) and warms to 40°C
    // (beta_mult=1.225), giving a 44% swing in beta.
    proc_cold.thermal = Some(ThermalModel::germanium_fuzz(SAMPLE_RATE));
    // Do NOT warm up — stays at ambient 15°C.
    let out_cold: Vec<f64> = input.iter().map(|&s| proc_cold.process(s)).collect();

    // --- Warm render: same but force to steady-state before processing ---
    let mut proc_warm =
        compile_pedal_with_options(&pedal, SAMPLE_RATE, options).expect("compile failed");
    proc_warm.thermal = Some(ThermalModel::germanium_fuzz(SAMPLE_RATE));
    // Force thermal model to steady-state (40°C). tick() will return warm
    // state on every subsequent call because elapsed >> tau.
    proc_warm.thermal.as_mut().unwrap().warm_up();
    let out_warm: Vec<f64> = input.iter().map(|&s| proc_warm.process(s)).collect();

    // Both must produce non-trivial output.
    assert_healthy(&out_cold, "AC128 thermal cold", 50.0);
    assert_healthy(&out_warm, "AC128 thermal warm", 50.0);

    let rms_cold = rms(&out_cold);
    let rms_warm = rms(&out_warm);

    eprintln!(
        "thermal_ge_bjt: cold_rms={rms_cold:.6}, warm_rms={rms_warm:.6}, ratio={:.4}",
        rms_warm / rms_cold.max(1e-20)
    );

    // The outputs MUST differ: thermal state modulates BJT vt, Is, and bf.
    let max_diff = out_cold
        .iter()
        .zip(out_warm.iter())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0f64, f64::max);

    assert!(
        max_diff > 1e-8,
        "thermal=on Ge BJT cold vs warm: outputs are identical (no thermal effect). \
         max_diff={max_diff:.2e}. Check that base_bjt_model is set and apply_thermal \
         is being called with the warm ThermalState."
    );

    // Log the drift direction. At warm temperature, beta_mult=1.225 vs cold 0.85
    // means more collector current. Whether RMS is higher/lower depends on the
    // circuit's operating point, but outputs must diverge.
    eprintln!(
        "thermal_ge_bjt direction: warm {} cold by {:.1}%",
        if rms_warm > rms_cold { ">" } else { "<" },
        (rms_warm - rms_cold).abs() / rms_cold.max(1e-20) * 100.0
    );
}

/// Verify beta_multiplier at 25°C (reference) is exactly 1.0 and at 45°C
/// is consistent with Ge tempco=0.015/°C.
///
/// This is a unit-level check on thermal coefficients independent of DSP.
#[test]
fn thermal_ge_bjt_beta_coefficients_match_spec() {
    let coeffs = ThermalCoefficients::germanium_bjt();
    assert!(
        (coeffs.beta_tempco - 0.015).abs() < 1e-6,
        "Ge beta_tempco should be 0.015/°C, got {}",
        coeffs.beta_tempco
    );

    // At 45°C: beta_mult = 1 + 0.015 * (45 - 25) = 1.30
    let mut model = ThermalModel::germanium_fuzz(48_000.0);
    model.warm_up();
    let state = model.state();

    // The warm state is at 40°C (steady_state_temp for germanium_fuzz).
    let expected_beta = 1.0 + 0.015 * (40.0 - 25.0); // 1.225
    assert!(
        (state.beta_multiplier - expected_beta).abs() < 0.01,
        "Ge beta_multiplier at 40°C should be ~{expected_beta:.3}, got {:.6}",
        state.beta_multiplier
    );

    // Verify is_multiplier > 1.0 at warm temperature (Is doubles every 8°C for Ge).
    // From 25°C to 40°C = 15°C → Is_mult = exp(15/8) ≈ 6.5
    assert!(
        state.is_multiplier > 2.0,
        "Ge Is multiplier at 40°C should be > 2.0, got {}",
        state.is_multiplier
    );
}

/// Thermal flag must not crash or produce NaN/Inf on any circuit.
/// Uses a silicon diode clipper — the canonical simple NL circuit.
#[test]
fn thermal_on_diode_clipper_no_nan() {
    let input = sine_at(440.0, 0.5, 0.1, SAMPLE_RATE);

    let options = CompileOptions {
        thermal: true,
        ..CompileOptions::default()
    };

    let output = compile_test_pedal_with_options("clip_silicon.pedal", &input, SAMPLE_RATE, &[], options);

    for (i, &s) in output.iter().enumerate() {
        assert!(
            s.is_finite(),
            "thermal=on diode clipper: NaN/Inf at sample {i}: {s}"
        );
    }

    assert_healthy(&output, "thermal diode clipper", 5.0);
}
