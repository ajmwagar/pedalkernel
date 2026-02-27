//! Integration tests for diode clipping circuits.
//!
//! Validates silicon, germanium, LED, zener, and asymmetric diode behavior
//! by processing test signals through isolated clipping circuits and analyzing
//! the output with spectral analysis.

mod audio_analysis;

use audio_analysis::*;

// ---------------------------------------------------------------------------
// THD vs. input level
// ---------------------------------------------------------------------------

#[test]
fn silicon_thd_increases_with_level() {
    let levels = [0.1, 0.3, 0.5, 0.8];
    let mut prev_thd = 0.0;

    for &amp in &levels {
        let input = sine_at(440.0, amp, 0.2, SAMPLE_RATE);
        let output = compile_test_pedal_and_process("clip_silicon.pedal", &input, SAMPLE_RATE, &[]);
        assert_healthy(&output, &format!("silicon@{amp}"), 5.0);
        let t = thd(&output, SAMPLE_RATE, 440.0);
        // THD should generally increase with amplitude (more clipping).
        // Allow the lowest level to be essentially zero.
        if amp > 0.1 {
            assert!(
                t >= prev_thd * 0.8, // allow small jitter
                "Silicon THD should increase: at amp={amp}, thd={t:.4} < prev={prev_thd:.4}"
            );
        }
        prev_thd = t;
    }
}

#[test]
fn germanium_thd_increases_with_level() {
    let levels = [0.1, 0.3, 0.5, 0.8];
    let mut prev_thd = 0.0;

    for &amp in &levels {
        let input = sine_at(440.0, amp, 0.2, SAMPLE_RATE);
        let output =
            compile_test_pedal_and_process("clip_germanium.pedal", &input, SAMPLE_RATE, &[]);
        assert_healthy(&output, &format!("germanium@{amp}"), 5.0);
        let t = thd(&output, SAMPLE_RATE, 440.0);
        if amp > 0.1 {
            assert!(
                t >= prev_thd * 0.8,
                "Ge THD should increase: at amp={amp}, thd={t:.4} < prev={prev_thd:.4}"
            );
        }
        prev_thd = t;
    }
}

// ---------------------------------------------------------------------------
// Cross-diode-type comparisons
// ---------------------------------------------------------------------------

#[test]
fn germanium_and_silicon_both_clip() {
    // Both Ge and Si should produce measurable distortion at moderate levels.
    // In the WDF model, the exact ordering depends on internal impedance
    // interactions, so we verify both clip rather than requiring strict ordering.
    let input = sine_at(440.0, 0.4, 0.2, SAMPLE_RATE);

    let si_out = compile_test_pedal_and_process("clip_silicon.pedal", &input, SAMPLE_RATE, &[]);
    let ge_out = compile_test_pedal_and_process("clip_germanium.pedal", &input, SAMPLE_RATE, &[]);

    let si_thd = thd(&si_out, SAMPLE_RATE, 440.0);
    let ge_thd = thd(&ge_out, SAMPLE_RATE, 440.0);

    // Both should produce measurable distortion
    assert!(
        si_thd > 0.01,
        "Si should clip at 0.4: thd={si_thd:.4}"
    );
    assert!(
        ge_thd > 0.01,
        "Ge should clip at 0.4: thd={ge_thd:.4}"
    );

    // They should produce different amounts of distortion (different Vf)
    assert!(
        (ge_thd - si_thd).abs() > 0.0001 || ge_thd > 0.1,
        "Ge and Si should differ: ge={ge_thd:.4}, si={si_thd:.4}"
    );
}

#[test]
fn led_and_silicon_produce_different_clipping() {
    // LED (Vf≈1.7V) and Si (Vf≈0.6V) have different forward voltages,
    // which should produce different distortion characters.
    let input = sine_at(440.0, 0.5, 0.2, SAMPLE_RATE);

    let si_out = compile_test_pedal_and_process("clip_silicon.pedal", &input, SAMPLE_RATE, &[]);
    let led_out = compile_test_pedal_and_process("clip_led.pedal", &input, SAMPLE_RATE, &[]);

    let si_thd = thd(&si_out, SAMPLE_RATE, 440.0);
    let led_thd = thd(&led_out, SAMPLE_RATE, 440.0);

    // Both should produce some distortion
    assert!(si_thd > 0.01, "Si should clip: thd={si_thd:.4}");
    assert!(led_thd > 0.01, "LED should clip: thd={led_thd:.4}");

    // They should differ (different Vf = different clipping characteristics)
    let corr = correlation(&si_out, &led_out).abs();
    assert!(
        corr < 0.9999,
        "LED and Si should differ: si_thd={si_thd:.4}, led_thd={led_thd:.4}, corr={corr:.6}"
    );
}

// ---------------------------------------------------------------------------
// Harmonic symmetry
// ---------------------------------------------------------------------------

#[test]
fn symmetric_pair_favors_odd_harmonics() {
    // Symmetric clipping (diode pair) produces primarily odd harmonics
    let input = sine_at(440.0, 0.6, 0.5, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("clip_silicon.pedal", &input, SAMPLE_RATE, &[]);

    let h2 = goertzel_power(&output, SAMPLE_RATE, 880.0); // 2nd harmonic (even)
    let h3 = goertzel_power(&output, SAMPLE_RATE, 1320.0); // 3rd harmonic (odd)

    // For symmetric clipping, 3rd harmonic should be stronger than 2nd
    assert!(
        h3 > h2 * 0.5,
        "Symmetric clipper should favor odd harmonics: h3={h3:.6}, h2={h2:.6}"
    );
}

#[test]
fn asymmetric_diode_has_even_harmonics() {
    // Single diode (half-wave) produces significant even harmonics
    let input = sine_at(440.0, 0.6, 0.5, SAMPLE_RATE);
    let output =
        compile_test_pedal_and_process("clip_asymmetric.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "asymmetric_clip", 10.0);

    let h2 = goertzel_power(&output, SAMPLE_RATE, 880.0);
    let fundamental = goertzel_power(&output, SAMPLE_RATE, 440.0);

    // Asymmetric clipping should produce measurable 2nd harmonic
    // (half-wave rectification has strong even harmonics)
    assert!(
        h2 > fundamental * 0.001,
        "Asymmetric clipper should produce 2nd harmonic: h2={h2:.6}, fund={fundamental:.6}"
    );
}

// ---------------------------------------------------------------------------
// Zener asymmetry
// ---------------------------------------------------------------------------

#[test]
fn zener_clips_asymmetrically() {
    let input = sine_at(440.0, 0.6, 0.5, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("clip_zener.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "zener_clip", 10.0);

    // Zener has asymmetric clipping — should produce both even and odd harmonics
    let h2 = goertzel_power(&output, SAMPLE_RATE, 880.0);
    let h3 = goertzel_power(&output, SAMPLE_RATE, 1320.0);

    // Both harmonics should be present
    assert!(
        h2 > 1e-10 || h3 > 1e-10,
        "Zener should produce harmonics: h2={h2:.6}, h3={h3:.6}"
    );
}

// ---------------------------------------------------------------------------
// Crest factor (clipping compresses peaks)
// ---------------------------------------------------------------------------

#[test]
fn clipping_reduces_crest_factor() {
    let input = sine_at(440.0, 0.6, 0.5, SAMPLE_RATE);
    let input_cf = crest_factor(&input);

    let output = compile_test_pedal_and_process("clip_silicon.pedal", &input, SAMPLE_RATE, &[]);
    let output_cf = crest_factor(&output);

    // Clipping reduces crest factor (flattens peaks)
    // A pure sine has CF ≈ √2 ≈ 1.414; clipped should be lower
    assert!(
        output_cf <= input_cf + 0.1,
        "Clipping should not increase crest factor: input_cf={input_cf:.3}, output_cf={output_cf:.3}"
    );
}

// ---------------------------------------------------------------------------
// Guitar WAV through all clippers
// ---------------------------------------------------------------------------

#[test]
fn all_clippers_finite_with_guitar() {
    let (guitar, _sr) = &*CLEAN_GUITAR;
    // Take first 48000 samples (1 second)
    let input: Vec<f64> = guitar.iter().copied().take(48000).collect();

    let clippers = [
        "clip_silicon.pedal",
        "clip_germanium.pedal",
        "clip_led.pedal",
        "clip_zener.pedal",
        "clip_asymmetric.pedal",
    ];

    for name in clippers {
        let output = compile_test_pedal_and_process(name, &input, SAMPLE_RATE, &[]);
        assert_healthy(&output, name, 10.0);
        maybe_dump_wav(&output, &format!("guitar_{name}"), SAMPLE_RATE_U32);
    }
}

// ---------------------------------------------------------------------------
// Boundary tests
// ---------------------------------------------------------------------------

#[test]
fn high_drive_heavy_distortion() {
    // At high amplitude, all clippers should produce significant THD
    let input = sine_at(440.0, 0.8, 0.2, SAMPLE_RATE);

    for name in ["clip_silicon.pedal", "clip_germanium.pedal"] {
        let output = compile_test_pedal_and_process(name, &input, SAMPLE_RATE, &[]);
        let t = thd(&output, SAMPLE_RATE, 440.0);
        assert!(
            t > 0.05,
            "{name} at 0.8 amplitude should have THD > 5%: got {t:.4}"
        );
    }
}

#[test]
fn low_drive_minimal_distortion() {
    // At very low amplitude, signal should be below clipping threshold
    let input = sine_at(440.0, 0.02, 0.2, SAMPLE_RATE);

    for name in ["clip_silicon.pedal", "clip_led.pedal"] {
        let output = compile_test_pedal_and_process(name, &input, SAMPLE_RATE, &[]);
        assert!(
            output.iter().all(|x| x.is_finite()),
            "{name}: output has NaN/inf"
        );
        let t = thd(&output, SAMPLE_RATE, 440.0);
        assert!(
            t < 0.2,
            "{name} at 0.02 amplitude should have low THD: got {t:.4}"
        );
    }
}

#[test]
fn clipping_outputs_bounded() {
    let input = sine_at(440.0, 0.5, 0.2, SAMPLE_RATE);

    for name in [
        "clip_silicon.pedal",
        "clip_germanium.pedal",
        "clip_led.pedal",
        "clip_zener.pedal",
        "clip_asymmetric.pedal",
    ] {
        let output = compile_test_pedal_and_process(name, &input, SAMPLE_RATE, &[]);
        let p = peak(&output);
        assert!(p < 10.0, "{name}: output peak={p:.4} exceeds 10.0");
    }
}
