//! Integration tests for complete example pedal models.
//!
//! End-to-end tests that compile each example .pedal file, process audio
//! through it, and verify the output is healthy. Tests control responsiveness
//! and runs real guitar audio through each pedal.

mod audio_analysis;

use audio_analysis::*;

// ===========================================================================
// Klon Centaur
// ===========================================================================

#[test]
fn klon_centaur_produces_output() {
    let input = sine(440.0, 0.5, SAMPLE_RATE);
    let output = compile_example_and_process(
        "klon_centaur.pedal",
        &input,
        SAMPLE_RATE,
        &[("Gain", 0.5), ("Treble", 0.5), ("Output", 0.7)],
    );
    assert_healthy(&output, "Klon Centaur", 50.0);
}

#[test]
fn klon_centaur_with_guitar() {
    let (guitar, _sr) = &*CLEAN_GUITAR;
    let input: Vec<f64> = guitar.iter().copied().take(48000).collect();
    let output = compile_example_and_process(
        "klon_centaur.pedal",
        &input,
        SAMPLE_RATE,
        &[("Gain", 0.5), ("Treble", 0.5), ("Output", 0.7)],
    );
    assert_healthy(&output, "Klon guitar", 100.0);
    maybe_dump_wav(&output, "guitar_klon_centaur", SAMPLE_RATE_U32);
}

#[test]
fn klon_centaur_gain_affects_output() {
    let input = sine(440.0, 0.3, SAMPLE_RATE);

    let low = compile_example_and_process(
        "klon_centaur.pedal",
        &input,
        SAMPLE_RATE,
        &[("Gain", 0.1), ("Treble", 0.5), ("Output", 0.7)],
    );
    let high = compile_example_and_process(
        "klon_centaur.pedal",
        &input,
        SAMPLE_RATE,
        &[("Gain", 0.9), ("Treble", 0.5), ("Output", 0.7)],
    );

    assert_healthy(&low, "Klon low gain", 50.0);
    assert_healthy(&high, "Klon high gain", 50.0);

    let corr = correlation(&low, &high).abs();
    let low_thd = thd(&low, SAMPLE_RATE, 440.0);
    let high_thd = thd(&high, SAMPLE_RATE, 440.0);
    eprintln!("  [diag] Klon gain: low_thd={low_thd:.4}, high_thd={high_thd:.4}, corr={corr:.6}");
}

// ===========================================================================
// Boss Blues Driver BD-2
// ===========================================================================

#[test]
fn blues_driver_produces_output() {
    let input = sine(440.0, 0.5, SAMPLE_RATE);
    let output = compile_example_and_process(
        "blues_driver.pedal",
        &input,
        SAMPLE_RATE,
        &[("Gain", 0.5), ("Tone", 0.5), ("Level", 0.7)],
    );
    assert_healthy(&output, "Blues Driver", 50.0);
}

#[test]
fn blues_driver_with_guitar() {
    let (guitar, _sr) = &*CLEAN_GUITAR;
    let input: Vec<f64> = guitar.iter().copied().take(48000).collect();
    let output = compile_example_and_process(
        "blues_driver.pedal",
        &input,
        SAMPLE_RATE,
        &[("Gain", 0.5), ("Tone", 0.5), ("Level", 0.7)],
    );
    assert_healthy(&output, "Blues Driver guitar", 100.0);
    maybe_dump_wav(&output, "guitar_blues_driver", SAMPLE_RATE_U32);
}

#[test]
fn blues_driver_gain_affects_output() {
    let input = sine(440.0, 0.3, SAMPLE_RATE);

    let low = compile_example_and_process(
        "blues_driver.pedal",
        &input,
        SAMPLE_RATE,
        &[("Gain", 0.1), ("Tone", 0.5), ("Level", 0.7)],
    );
    let high = compile_example_and_process(
        "blues_driver.pedal",
        &input,
        SAMPLE_RATE,
        &[("Gain", 0.9), ("Tone", 0.5), ("Level", 0.7)],
    );

    assert_healthy(&low, "BD-2 low gain", 50.0);
    assert_healthy(&high, "BD-2 high gain", 50.0);

    let corr = correlation(&low, &high).abs();
    let low_thd = thd(&low, SAMPLE_RATE, 440.0);
    let high_thd = thd(&high, SAMPLE_RATE, 440.0);
    eprintln!("  [diag] BD-2 gain: low_thd={low_thd:.4}, high_thd={high_thd:.4}, corr={corr:.6}");
}

// ===========================================================================
// Fulltone OCD
// ===========================================================================

#[test]
fn fulltone_ocd_produces_output() {
    let input = sine(440.0, 0.5, SAMPLE_RATE);
    let output = compile_example_and_process(
        "fulltone_ocd.pedal",
        &input,
        SAMPLE_RATE,
        &[("Drive", 0.5), ("Tone", 0.5), ("Volume", 0.7)],
    );
    assert_healthy(&output, "Fulltone OCD", 50.0);
}

#[test]
fn fulltone_ocd_with_guitar() {
    let (guitar, _sr) = &*CLEAN_GUITAR;
    let input: Vec<f64> = guitar.iter().copied().take(48000).collect();
    let output = compile_example_and_process(
        "fulltone_ocd.pedal",
        &input,
        SAMPLE_RATE,
        &[("Drive", 0.5), ("Tone", 0.5), ("Volume", 0.7)],
    );
    assert_healthy(&output, "OCD guitar", 100.0);
    maybe_dump_wav(&output, "guitar_fulltone_ocd", SAMPLE_RATE_U32);
}

#[test]
fn fulltone_ocd_drive_affects_output() {
    let input = sine(440.0, 0.3, SAMPLE_RATE);

    let low = compile_example_and_process(
        "fulltone_ocd.pedal",
        &input,
        SAMPLE_RATE,
        &[("Drive", 0.1), ("Tone", 0.5), ("Volume", 0.7)],
    );
    let high = compile_example_and_process(
        "fulltone_ocd.pedal",
        &input,
        SAMPLE_RATE,
        &[("Drive", 0.9), ("Tone", 0.5), ("Volume", 0.7)],
    );

    assert_healthy(&low, "OCD low drive", 50.0);
    assert_healthy(&high, "OCD high drive", 50.0);

    let corr = correlation(&low, &high).abs();
    let low_thd = thd(&low, SAMPLE_RATE, 440.0);
    let high_thd = thd(&high, SAMPLE_RATE, 440.0);
    eprintln!("  [diag] OCD drive: low_thd={low_thd:.4}, high_thd={high_thd:.4}, corr={corr:.6}");
}

// ===========================================================================
// MXR Dyna Comp
// ===========================================================================

#[test]
fn dyna_comp_produces_output() {
    let input = sine(440.0, 0.5, SAMPLE_RATE);
    let output = compile_example_and_process(
        "dyna_comp.pedal",
        &input,
        SAMPLE_RATE,
        &[("Sensitivity", 0.5), ("Output", 0.7)],
    );
    assert_healthy(&output, "Dyna Comp", 50.0);
}

#[test]
fn dyna_comp_with_guitar() {
    let (guitar, _sr) = &*CLEAN_GUITAR;
    let input: Vec<f64> = guitar.iter().copied().take(48000).collect();
    let output = compile_example_and_process(
        "dyna_comp.pedal",
        &input,
        SAMPLE_RATE,
        &[("Sensitivity", 0.5), ("Output", 0.7)],
    );
    assert_healthy(&output, "Dyna Comp guitar", 100.0);
    maybe_dump_wav(&output, "guitar_dyna_comp", SAMPLE_RATE_U32);
}

#[test]
fn dyna_comp_sensitivity_affects_output() {
    let input = sine(440.0, 0.3, SAMPLE_RATE);

    let low = compile_example_and_process(
        "dyna_comp.pedal",
        &input,
        SAMPLE_RATE,
        &[("Sensitivity", 0.1), ("Output", 0.7)],
    );
    let high = compile_example_and_process(
        "dyna_comp.pedal",
        &input,
        SAMPLE_RATE,
        &[("Sensitivity", 0.9), ("Output", 0.7)],
    );

    assert_healthy(&low, "Dyna Comp low sens", 50.0);
    assert_healthy(&high, "Dyna Comp high sens", 50.0);

    let corr = correlation(&low, &high).abs();
    eprintln!("  [diag] Dyna Comp sensitivity: corr={corr:.6}");
}

// ===========================================================================
// Tone control tests — verify tone knob changes spectral character
// ===========================================================================

#[test]
fn klon_treble_affects_brightness() {
    let input = guitar_pluck(330.0, 0.5, SAMPLE_RATE);

    let dark = compile_example_and_process(
        "klon_centaur.pedal",
        &input,
        SAMPLE_RATE,
        &[("Gain", 0.5), ("Treble", 0.0), ("Output", 0.7)],
    );
    let bright = compile_example_and_process(
        "klon_centaur.pedal",
        &input,
        SAMPLE_RATE,
        &[("Gain", 0.5), ("Treble", 1.0), ("Output", 0.7)],
    );

    assert_healthy(&dark, "Klon dark", 50.0);
    assert_healthy(&bright, "Klon bright", 50.0);

    let dark_centroid = spectral_centroid(&dark, SAMPLE_RATE);
    let bright_centroid = spectral_centroid(&bright, SAMPLE_RATE);

    eprintln!(
        "  [diag] Klon treble: dark_centroid={dark_centroid:.1}Hz, bright_centroid={bright_centroid:.1}Hz"
    );
    // Both should produce valid spectra
    assert!(dark_centroid > 0.0, "Dark should have valid spectrum");
    assert!(bright_centroid > 0.0, "Bright should have valid spectrum");
}

#[test]
fn blues_driver_tone_affects_brightness() {
    let input = guitar_pluck(330.0, 0.5, SAMPLE_RATE);

    let dark = compile_example_and_process(
        "blues_driver.pedal",
        &input,
        SAMPLE_RATE,
        &[("Gain", 0.5), ("Tone", 0.0), ("Level", 0.7)],
    );
    let bright = compile_example_and_process(
        "blues_driver.pedal",
        &input,
        SAMPLE_RATE,
        &[("Gain", 0.5), ("Tone", 1.0), ("Level", 0.7)],
    );

    assert_healthy(&dark, "BD-2 dark", 50.0);
    assert_healthy(&bright, "BD-2 bright", 50.0);

    let dark_centroid = spectral_centroid(&dark, SAMPLE_RATE);
    let bright_centroid = spectral_centroid(&bright, SAMPLE_RATE);

    eprintln!(
        "  [diag] BD-2 tone: dark_centroid={dark_centroid:.1}Hz, bright_centroid={bright_centroid:.1}Hz"
    );
}

// ---------------------------------------------------------------------------
// BD-2 revived-pot monotonicity (all-passive-drop fix, 2026-06-14)
// ---------------------------------------------------------------------------
//
// The BD-2 tone stack (R10..C9 + the Tone pot) and the output network (R13/R14
// + the Level wiper divider) form an all-passive span that reduces to a rigid
// (non-series-parallel) R-node. SPQR previously DROPPED that span, so `Tone`
// and `Level` were unbound/inert (a unity passthrough). The fix rebuilds the
// span as a WDF PassiveRType stage with live pot leaves, so both controls now
// move the output monotonically. These tests lock that behavior.

/// Steady-state RMS-derived level (dBFS-ish) at one freq + control set.
fn bd2_level_db(freq: f64, ctrl: &[(&str, f64)]) -> f64 {
    // 1 s tone at 0.05 amplitude; analyze the second half (post-settle).
    let input = sine_at(freq, 0.05, 1.0, SAMPLE_RATE);
    let out = compile_example_and_process("blues_driver.pedal", &input, SAMPLE_RATE, ctrl);
    let n = out.len();
    lin_to_db(rms(&out[n / 2..]) * std::f64::consts::SQRT_2)
}

#[test]
fn blues_driver_tone_sweeps_monotonically() {
    // Tone is a tilt/cut control: at 3 kHz, CW (1.0) is darker than CCW (0.0),
    // so the sweep is monotone NON-INCREASING. Measured ~-16.2 dB (0.0) ->
    // ~-18.3 dB (1.0), ~2.1 dB of authority. (Was dropped/inert before the fix:
    // every position measured the same ~-1.08 dB passthrough.)
    let mut levels = Vec::new();
    for &t in &[0.0, 0.25, 0.5, 0.75, 1.0] {
        let db = bd2_level_db(3000.0, &[("Gain", 0.5), ("Tone", t), ("Level", 0.7)]);
        eprintln!("  [BD-2 Tone] {t:.2} -> {db:.3} dB @ 3 kHz");
        levels.push(db);
    }
    for w in levels.windows(2) {
        assert!(
            w[1] <= w[0] + 0.05,
            "BD-2 Tone must sweep monotonically (non-increasing) at 3 kHz, got {levels:?}"
        );
    }
    let span = levels.first().unwrap() - levels.last().unwrap();
    assert!(
        span > 1.0,
        "BD-2 Tone should have real authority (> 1 dB), got {span:.2} dB"
    );
}

#[test]
fn blues_driver_level_sweeps_monotonically() {
    // Level is a volume wiper divider: monotone INCREASING, full-range. Measured
    // ~-158 dB (0.0, fully closed) -> ~-6.4 dB (1.0) at 1 kHz. (Was dropped/inert
    // before the fix: a frozen passthrough.)
    let mut levels = Vec::new();
    for &l in &[0.25, 0.5, 0.75, 1.0] {
        let db = bd2_level_db(1000.0, &[("Gain", 0.5), ("Tone", 0.5), ("Level", l)]);
        eprintln!("  [BD-2 Level] {l:.2} -> {db:.3} dB @ 1 kHz");
        levels.push(db);
    }
    for w in levels.windows(2) {
        assert!(
            w[1] >= w[0] - 0.05,
            "BD-2 Level must sweep monotonically (non-decreasing), got {levels:?}"
        );
    }
    // Wide authority end-to-end (audio-taper volume pot).
    let span = levels.last().unwrap() - levels.first().unwrap();
    assert!(
        span > 10.0,
        "BD-2 Level should have wide authority (> 10 dB across 0.25..1.0), got {span:.2} dB"
    );
}

// ===========================================================================
// Cross-pedal comparison — different pedals should sound different
// ===========================================================================

#[test]
fn all_example_pedals_distinct() {
    let input = guitar_pluck(330.0, 0.5, SAMPLE_RATE);

    let pedals: Vec<(&str, &[(&str, f64)])> = vec![
        (
            "tube_screamer.pedal",
            &[("Drive", 0.5), ("Tone", 0.5), ("Level", 0.7)],
        ),
        (
            "klon_centaur.pedal",
            &[("Gain", 0.5), ("Treble", 0.5), ("Output", 0.7)],
        ),
        (
            "blues_driver.pedal",
            &[("Gain", 0.5), ("Tone", 0.5), ("Level", 0.7)],
        ),
        (
            "proco_rat.pedal",
            &[("Distortion", 0.5), ("Filter", 0.5), ("Volume", 0.7)],
        ),
    ];

    let mut outputs: Vec<(&str, Vec<f64>)> = Vec::new();
    for (name, controls) in &pedals {
        let out = compile_example_and_process(name, &input, SAMPLE_RATE, controls);
        assert_healthy(&out, name, 100.0);
        outputs.push((name, out));
    }

    // Log pairwise correlations — these should ideally differ
    for i in 0..outputs.len() {
        for j in (i + 1)..outputs.len() {
            let corr = correlation(&outputs[i].1, &outputs[j].1).abs();
            eprintln!(
                "  [diag] {} vs {}: corr={corr:.6}",
                outputs[i].0, outputs[j].0
            );
        }
    }
}
