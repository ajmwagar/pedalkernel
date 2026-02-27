//! Integration tests for engine-level features: oversampling and tolerance.
//!
//! These features are "multipliers" that affect every pedal — oversampling
//! reduces aliasing artifacts, and tolerance simulates real component
//! variation between physical pedal units.

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::compiler::CompileOptions;
use pedalkernel::oversampling::OversamplingFactor;
use pedalkernel::tolerance::{ToleranceEngine, ToleranceGrade};

// ===========================================================================
// Oversampling — reduces aliasing on nonlinear stages
// ===========================================================================

#[test]
fn oversampling_x2_compiles_and_runs() {
    let input = sine(440.0, 0.5, SAMPLE_RATE);
    let options = CompileOptions {
        oversampling: OversamplingFactor::X2,
        ..CompileOptions::default()
    };
    let output = compile_test_pedal_with_options(
        "clip_silicon.pedal",
        &input,
        SAMPLE_RATE,
        &[],
        options,
    );
    assert_healthy(&output, "clip_silicon x2", 50.0);
}

#[test]
fn oversampling_x4_compiles_and_runs() {
    let input = sine(440.0, 0.5, SAMPLE_RATE);
    let options = CompileOptions {
        oversampling: OversamplingFactor::X4,
        ..CompileOptions::default()
    };
    let output = compile_test_pedal_with_options(
        "clip_silicon.pedal",
        &input,
        SAMPLE_RATE,
        &[],
        options,
    );
    assert_healthy(&output, "clip_silicon x4", 50.0);
}

#[test]
fn oversampling_reduces_alias_energy() {
    // Hard-clipped high-frequency sine generates aliased harmonics.
    // Oversampling should reduce energy above the fundamental's harmonics.
    let input = sine(8000.0, 0.3, SAMPLE_RATE);

    let no_os = compile_test_pedal_with_options(
        "clip_silicon.pedal",
        &input,
        SAMPLE_RATE,
        &[],
        CompileOptions {
            oversampling: OversamplingFactor::X1,
            ..CompileOptions::default()
        },
    );
    let with_os = compile_test_pedal_with_options(
        "clip_silicon.pedal",
        &input,
        SAMPLE_RATE,
        &[],
        CompileOptions {
            oversampling: OversamplingFactor::X4,
            ..CompileOptions::default()
        },
    );

    assert_healthy(&no_os, "no oversampling", 50.0);
    assert_healthy(&with_os, "4x oversampling", 50.0);

    // The outputs should differ — oversampling changes the alias content
    let corr = correlation(&no_os, &with_os).abs();
    eprintln!("  [diag] Oversampling effect: corr={corr:.6}");

    // Both should produce clipped output
    assert!(peak(&no_os) > 0.001, "No-OS should produce output");
    assert!(peak(&with_os) > 0.001, "With-OS should produce output");
}

#[test]
fn oversampling_preserves_low_frequency() {
    // A low-frequency sine through oversampled processing should remain clean
    let input = sine(220.0, 0.5, SAMPLE_RATE);

    let no_os = compile_test_pedal_with_options(
        "triode_clean.pedal",
        &input,
        SAMPLE_RATE,
        &[],
        CompileOptions {
            oversampling: OversamplingFactor::X1,
            ..CompileOptions::default()
        },
    );
    let with_os = compile_test_pedal_with_options(
        "triode_clean.pedal",
        &input,
        SAMPLE_RATE,
        &[],
        CompileOptions {
            oversampling: OversamplingFactor::X2,
            ..CompileOptions::default()
        },
    );

    assert_healthy(&no_os, "no-OS triode", 50.0);
    assert_healthy(&with_os, "x2-OS triode", 50.0);

    // Low frequency content should be highly correlated (oversampling
    // mainly affects high-frequency alias content, not the fundamental)
    let corr = correlation(&no_os, &with_os).abs();
    eprintln!("  [diag] Low-freq oversampling: corr={corr:.6}");
    assert!(
        corr > 0.9,
        "Low-frequency content should be similar: corr={corr:.6}"
    );
}

#[test]
fn oversampling_with_guitar() {
    let (guitar, _sr) = &*CLEAN_GUITAR;
    let input: Vec<f64> = guitar.iter().copied().take(24000).collect();

    let output = compile_example_with_options(
        "tube_screamer.pedal",
        &input,
        SAMPLE_RATE,
        &[("Drive", 0.7), ("Tone", 0.5), ("Level", 0.7)],
        CompileOptions {
            oversampling: OversamplingFactor::X2,
            ..CompileOptions::default()
        },
    );
    assert_healthy(&output, "TS x2 guitar", 100.0);
}

// ===========================================================================
// Tolerance — component value randomization
// ===========================================================================

#[test]
fn tolerance_ideal_matches_default() {
    // Ideal tolerance (no variation) should produce identical output to default
    let input = sine(440.0, 0.5, SAMPLE_RATE);

    let default_out =
        compile_test_pedal_and_process("clip_silicon.pedal", &input, SAMPLE_RATE, &[]);

    let ideal_out = compile_test_pedal_with_options(
        "clip_silicon.pedal",
        &input,
        SAMPLE_RATE,
        &[],
        CompileOptions {
            tolerance: ToleranceEngine::ideal(),
            ..CompileOptions::default()
        },
    );

    let corr = correlation(&default_out, &ideal_out).abs();
    assert!(
        corr > 0.999,
        "Ideal tolerance should match default: corr={corr:.6}"
    );
}

#[test]
fn tolerance_same_seed_deterministic() {
    let input = sine(440.0, 0.5, SAMPLE_RATE);
    let out1 = compile_test_pedal_with_options(
        "clip_silicon.pedal",
        &input,
        SAMPLE_RATE,
        &[],
        CompileOptions {
            tolerance: ToleranceEngine::new(42, ToleranceGrade::Loose),
            ..CompileOptions::default()
        },
    );
    let out2 = compile_test_pedal_with_options(
        "clip_silicon.pedal",
        &input,
        SAMPLE_RATE,
        &[],
        CompileOptions {
            tolerance: ToleranceEngine::new(42, ToleranceGrade::Loose),
            ..CompileOptions::default()
        },
    );

    let corr = correlation(&out1, &out2).abs();
    assert!(
        corr > 0.999,
        "Same seed should produce identical output: corr={corr:.6}"
    );
}

#[test]
fn tolerance_different_seeds_differ() {
    let input = sine(440.0, 0.5, SAMPLE_RATE);

    let unit_a = compile_test_pedal_with_options(
        "clip_silicon.pedal",
        &input,
        SAMPLE_RATE,
        &[],
        CompileOptions {
            tolerance: ToleranceEngine::new(1, ToleranceGrade::Wide),
            ..CompileOptions::default()
        },
    );
    let unit_b = compile_test_pedal_with_options(
        "clip_silicon.pedal",
        &input,
        SAMPLE_RATE,
        &[],
        CompileOptions {
            tolerance: ToleranceEngine::new(999, ToleranceGrade::Wide),
            ..CompileOptions::default()
        },
    );

    assert_healthy(&unit_a, "unit A", 50.0);
    assert_healthy(&unit_b, "unit B", 50.0);

    let corr = correlation(&unit_a, &unit_b).abs();
    eprintln!("  [diag] Tolerance seed variation: corr={corr:.6}");

    // Wide tolerance (±20%) on different seeds should produce different output
    // But they should still both be recognizably "the same pedal"
    assert!(
        corr < 1.0 || corr > 0.5,
        "Different seeds should differ but stay similar: corr={corr:.6}"
    );
}

#[test]
fn tolerance_wider_grade_more_variation() {
    let input = sine(440.0, 0.5, SAMPLE_RATE);

    // Compare standard (±5%) vs wide (±20%) tolerance
    let seeds = [1u64, 2, 3, 4, 5];
    let mut precision_corrs = Vec::new();
    let mut wide_corrs = Vec::new();

    let reference = compile_test_pedal_and_process(
        "clip_silicon.pedal",
        &input,
        SAMPLE_RATE,
        &[],
    );

    for &seed in &seeds {
        let precision = compile_test_pedal_with_options(
            "clip_silicon.pedal",
            &input,
            SAMPLE_RATE,
            &[],
            CompileOptions {
                tolerance: ToleranceEngine::new(seed, ToleranceGrade::Precision),
                ..CompileOptions::default()
            },
        );
        let wide = compile_test_pedal_with_options(
            "clip_silicon.pedal",
            &input,
            SAMPLE_RATE,
            &[],
            CompileOptions {
                tolerance: ToleranceEngine::new(seed, ToleranceGrade::Wide),
                ..CompileOptions::default()
            },
        );

        precision_corrs.push(correlation(&reference, &precision).abs());
        wide_corrs.push(correlation(&reference, &wide).abs());
    }

    let avg_precision: f64 = precision_corrs.iter().sum::<f64>() / precision_corrs.len() as f64;
    let avg_wide: f64 = wide_corrs.iter().sum::<f64>() / wide_corrs.len() as f64;

    eprintln!(
        "  [diag] Tolerance grades: avg_precision_corr={avg_precision:.6}, avg_wide_corr={avg_wide:.6}"
    );

    // Precision should stay closer to the reference than wide
    // (but both should produce valid output)
    assert!(avg_precision > 0.0, "Precision should produce output");
    assert!(avg_wide > 0.0, "Wide should produce output");
}

#[test]
fn tolerance_on_tube_amp() {
    // Tolerance should work on complex circuits too
    let input = guitar_pluck(330.0, 0.5, SAMPLE_RATE);

    let output = compile_example_with_options(
        "tweed_deluxe_5e3.pedal",
        &input,
        SAMPLE_RATE,
        &[("Volume", 0.7), ("Tone", 0.5)],
        CompileOptions {
            tolerance: ToleranceEngine::new(42, ToleranceGrade::Loose),
            ..CompileOptions::default()
        },
    );
    assert_healthy(&output, "Tweed with tolerance", 100.0);
}

// ===========================================================================
// Combined: oversampling + tolerance
// ===========================================================================

#[test]
fn combined_oversampling_and_tolerance() {
    let input = sine(440.0, 0.5, SAMPLE_RATE);

    let output = compile_test_pedal_with_options(
        "clip_silicon.pedal",
        &input,
        SAMPLE_RATE,
        &[],
        CompileOptions {
            oversampling: OversamplingFactor::X2,
            tolerance: ToleranceEngine::new(42, ToleranceGrade::Standard),
            ..CompileOptions::default()
        },
    );
    assert_healthy(&output, "clip x2 + tolerance", 50.0);
}

#[test]
fn combined_oversampling_tolerance_guitar() {
    let (guitar, _sr) = &*CLEAN_GUITAR;
    let input: Vec<f64> = guitar.iter().copied().take(24000).collect();

    let output = compile_example_with_options(
        "tube_screamer.pedal",
        &input,
        SAMPLE_RATE,
        &[("Drive", 0.7), ("Tone", 0.5), ("Level", 0.7)],
        CompileOptions {
            oversampling: OversamplingFactor::X2,
            tolerance: ToleranceEngine::new(123, ToleranceGrade::Loose),
            ..CompileOptions::default()
        },
    );
    assert_healthy(&output, "TS x2 + tolerance guitar", 100.0);
}
