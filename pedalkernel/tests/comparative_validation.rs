//! Comparative/relative tests: validate physics through relative comparisons
//! rather than absolute thresholds. These are inherently robust because they
//! test the *direction* of an effect, not its exact magnitude.

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::compiler::CompileOptions;
use pedalkernel::oversampling::OversamplingFactor;

/// Silicon clipper clips harder than triode (lower crest factor).
///
/// TOLERANCE SOURCE: Comparative — physics guarantees Si diodes (sharp 0.6V knee)
/// produce harder clipping than triode (gradual polynomial curvature).
/// No absolute threshold; just tests the ordering.
#[test]
fn silicon_clips_harder_than_triode() {
    let input = sine_at(800.0, 0.6, 0.2, SAMPLE_RATE);

    let si_output = compile_test_pedal_and_process("clip_silicon.pedal", &input, SAMPLE_RATE, &[]);
    let triode_output =
        compile_test_pedal_and_process("triode_clean.pedal", &input, SAMPLE_RATE, &[]);

    let si_crest = crest_factor(&si_output);
    let triode_crest = crest_factor(&triode_output);

    assert!(
        si_crest < triode_crest,
        "Silicon diodes should clip harder (lower crest factor) than triode: \
         si_crest={si_crest:.4}, triode_crest={triode_crest:.4}"
    );
}

/// High-drive THD is much larger than low-drive THD.
///
/// TOLERANCE SOURCE: Comparative — nonlinear transfer functions produce
/// monotonically more harmonics with increasing drive level.
/// 5x ratio is conservative; actual is typically 10-100x.
#[test]
fn high_drive_thd_exceeds_low_drive() {
    let circuit = r#"
pedal "Silicon Clipper" {
  components {
    R1: resistor(4.7k)
    D1: diode_pair(silicon)
  }
  nets {
    in -> R1.a
    R1.b -> D1.a, out
    D1.b -> gnd
  }
}
"#;
    let freq = 1000.0;

    let low_input = sine_at(freq, 0.05, 0.2, SAMPLE_RATE);
    let high_input = sine_at(freq, 0.8, 0.2, SAMPLE_RATE);

    let low_output = compile_and_process(circuit, &low_input, SAMPLE_RATE, &[]);
    let high_output = compile_and_process(circuit, &high_input, SAMPLE_RATE, &[]);

    let low_thd = thd(&low_output, SAMPLE_RATE, freq);
    let high_thd = thd(&high_output, SAMPLE_RATE, freq);

    assert!(
        high_thd > low_thd * 5.0,
        "High-drive THD should be >5x low-drive THD: high={high_thd:.6}, low={low_thd:.6}, \
         ratio={:.1}",
        if low_thd > 1e-12 {
            high_thd / low_thd
        } else {
            f64::INFINITY
        }
    );
}

/// Oversampling reduces aliasing for both RAT and silicon clipper.
///
/// TOLERANCE SOURCE: Comparative — 4x oversampling with half-band filter
/// should reduce alias energy by at least 50% (0.5x). Theory predicts >20dB
/// (100x) reduction. Tests the direction, not exact magnitude.
#[test]
fn oversampling_reduces_aliasing_comparative() {
    let sample_rate = 24_000.0;
    let alias_freq = 8000.0;

    // Silicon clipper inline circuit
    let si_circuit = r#"
pedal "Silicon Clipper" {
  components {
    R1: resistor(4.7k)
    D1: diode_pair(silicon)
  }
  nets {
    in -> R1.a
    R1.b -> D1.a, out
    D1.b -> gnd
  }
}
"#;

    for (name, pedal_src) in [("silicon_clipper", si_circuit)] {
        let input = sine_at(5_000.0, 0.8, 0.25, sample_rate);

        let no_os = compile_and_process_with_options(
            pedal_src,
            &input,
            sample_rate,
            &[],
            CompileOptions {
                oversampling: OversamplingFactor::X1,
                ..CompileOptions::default()
            },
        );
        let with_os = compile_and_process_with_options(
            pedal_src,
            &input,
            sample_rate,
            &[],
            CompileOptions {
                oversampling: OversamplingFactor::X4,
                ..CompileOptions::default()
            },
        );

        let aliased = alias_energy_summary(&no_os, sample_rate, alias_freq, 50.0);
        let oversampled = alias_energy_summary(&with_os, sample_rate, alias_freq, 50.0);

        assert!(
            aliased.alias_ratio > 1e-6,
            "{name}: aliasing too small to measure: ratio={}",
            aliased.alias_ratio
        );
        assert!(
            oversampled.alias_ratio < aliased.alias_ratio * 0.5,
            "{name}: oversampling should halve aliasing: no_os={:.6}, os={:.6}",
            aliased.alias_ratio,
            oversampled.alias_ratio,
        );
    }

    // Also test RAT example pedal
    let rat_controls = &[("Distortion", 0.45), ("Filter", 0.35), ("Volume", 0.7)];
    let input = sine_at(5_000.0, 0.8, 0.25, sample_rate);

    let no_os = compile_example_with_options(
        "proco_rat.pedal",
        &input,
        sample_rate,
        rat_controls,
        CompileOptions {
            oversampling: OversamplingFactor::X1,
            ..CompileOptions::default()
        },
    );
    let with_os = compile_example_with_options(
        "proco_rat.pedal",
        &input,
        sample_rate,
        rat_controls,
        CompileOptions {
            oversampling: OversamplingFactor::X4,
            ..CompileOptions::default()
        },
    );

    let aliased = alias_energy_summary(&no_os, sample_rate, alias_freq, 50.0);
    let oversampled = alias_energy_summary(&with_os, sample_rate, alias_freq, 50.0);

    assert!(
        aliased.alias_ratio > 1e-6,
        "RAT aliasing too small to measure: ratio={}",
        aliased.alias_ratio
    );
    assert!(
        oversampled.alias_ratio < aliased.alias_ratio * 0.5,
        "RAT oversampling should halve aliasing: no_os={:.6}, os={:.6}",
        aliased.alias_ratio,
        oversampled.alias_ratio,
    );
}
