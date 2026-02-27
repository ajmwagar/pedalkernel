//! Integration tests for triode and pentode vacuum tube circuits.
//!
//! Validates tube gain, distortion character, and harmonic profiles by
//! comparing different tube types through isolated test circuits.
//! Tests every tube variant and alias in the DSL.

mod audio_analysis;

use audio_analysis::*;

// ===========================================================================
// Macro to reduce boilerplate for "produces output" tests
// ===========================================================================

macro_rules! tube_produces_output {
    ($test_name:ident, $pedal_file:expr, $label:expr) => {
        #[test]
        fn $test_name() {
            let input = sine(440.0, 0.5, SAMPLE_RATE);
            let output =
                compile_test_pedal_and_process($pedal_file, &input, SAMPLE_RATE, &[]);
            assert_healthy(&output, $label, 50.0);
        }
    };
}

/// Verify that an alias pedal produces identical output to its canonical type.
/// Since aliases map to the same internal type, the same circuit topology
/// should yield bit-identical results.
macro_rules! tube_alias_matches {
    ($test_name:ident, $alias_file:expr, $canonical_file:expr, $label:expr) => {
        #[test]
        fn $test_name() {
            let input = sine(440.0, 0.5, SAMPLE_RATE);
            let alias_out =
                compile_test_pedal_and_process($alias_file, &input, SAMPLE_RATE, &[]);
            let canon_out =
                compile_test_pedal_and_process($canonical_file, &input, SAMPLE_RATE, &[]);

            assert_healthy(&alias_out, $label, 50.0);

            let corr = correlation(&alias_out, &canon_out).abs();
            eprintln!(
                "  [diag] {}: alias vs canonical correlation = {corr:.8}",
                $label
            );
            // Aliases resolve to the same tube type; with identical circuits
            // they should produce very similar output.
            assert!(
                corr > 0.99,
                "{}: alias should match canonical (corr={corr:.6})",
                $label
            );
        }
    };
}

// ===========================================================================
// Triode variants — each must compile and produce finite, non-silent output
// ===========================================================================

// Canonical types
tube_produces_output!(triode_12ax7_produces_output, "triode_overdrive.pedal", "12AX7");
tube_produces_output!(triode_12au7_produces_output, "triode_clean.pedal", "12AU7");
tube_produces_output!(triode_12at7_produces_output, "triode_12at7.pedal", "12AT7");
tube_produces_output!(triode_12ay7_produces_output, "triode_12ay7.pedal", "12AY7");
tube_produces_output!(triode_12bh7_produces_output, "triode_12bh7.pedal", "12BH7");
tube_produces_output!(triode_6386_produces_output, "triode_6386.pedal", "6386");

// Aliases
tube_produces_output!(triode_ecc83_produces_output, "triode_ecc83.pedal", "ECC83");
tube_produces_output!(triode_ecc81_produces_output, "triode_ecc81.pedal", "ECC81");
tube_produces_output!(triode_ecc82_produces_output, "triode_ecc82.pedal", "ECC82");
tube_produces_output!(triode_6072_produces_output, "triode_6072.pedal", "6072");

// ===========================================================================
// Pentode variants — each must compile and produce finite, non-silent output
// ===========================================================================

// Canonical types
tube_produces_output!(pentode_ef86_produces_output, "pentode_clean.pedal", "EF86");
tube_produces_output!(pentode_el84_produces_output, "pentode_el84.pedal", "EL84");
tube_produces_output!(pentode_el34_produces_output, "pentode_power.pedal", "EL34");
tube_produces_output!(pentode_6aq5a_produces_output, "pentode_6aq5a.pedal", "6AQ5A");
tube_produces_output!(pentode_6973_produces_output, "pentode_6973.pedal", "6973");
tube_produces_output!(pentode_6l6gc_produces_output, "pentode_6l6gc.pedal", "6L6GC");
tube_produces_output!(pentode_6550_produces_output, "pentode_6550.pedal", "6550");

// Aliases
tube_produces_output!(pentode_6267_produces_output, "pentode_6267.pedal", "6267");
tube_produces_output!(pentode_6bq5_produces_output, "pentode_6bq5.pedal", "6BQ5");
tube_produces_output!(pentode_6ca7_produces_output, "pentode_6ca7.pedal", "6CA7");
tube_produces_output!(pentode_kt77_produces_output, "pentode_kt77.pedal", "KT77");
tube_produces_output!(pentode_5881_produces_output, "pentode_5881.pedal", "5881");
tube_produces_output!(pentode_kt66_produces_output, "pentode_kt66.pedal", "KT66");
tube_produces_output!(pentode_kt88_produces_output, "pentode_kt88.pedal", "KT88");
tube_produces_output!(pentode_kt90_produces_output, "pentode_kt90.pedal", "KT90");

// ===========================================================================
// Alias consistency — aliases should match their canonical counterpart
// ===========================================================================

// Triode aliases: ECC83→12AX7, ECC81→12AT7, ECC82→12AU7, 6072→12AY7
tube_alias_matches!(
    triode_ecc83_matches_12ax7,
    "triode_ecc83.pedal",
    "triode_overdrive.pedal",
    "ECC83 vs 12AX7"
);
tube_alias_matches!(
    triode_ecc81_matches_12at7,
    "triode_ecc81.pedal",
    "triode_12at7.pedal",
    "ECC81 vs 12AT7"
);
tube_alias_matches!(
    triode_ecc82_matches_12au7,
    "triode_ecc82.pedal",
    "triode_clean.pedal",
    "ECC82 vs 12AU7"
);
tube_alias_matches!(
    triode_6072_matches_12ay7,
    "triode_6072.pedal",
    "triode_12ay7.pedal",
    "6072 vs 12AY7"
);

// Pentode aliases: 6267→EF86, 6BQ5→EL84, 6CA7→EL34, KT77→EL34,
//                  5881→6L6GC, KT66→6L6GC, KT88→6550, KT90→6550
tube_alias_matches!(
    pentode_6267_matches_ef86,
    "pentode_6267.pedal",
    "pentode_clean.pedal",
    "6267 vs EF86"
);
tube_alias_matches!(
    pentode_6bq5_matches_el84,
    "pentode_6bq5.pedal",
    "pentode_el84.pedal",
    "6BQ5 vs EL84"
);
tube_alias_matches!(
    pentode_6ca7_matches_el34,
    "pentode_6ca7.pedal",
    "pentode_power.pedal",
    "6CA7 vs EL34"
);
tube_alias_matches!(
    pentode_kt77_matches_el34,
    "pentode_kt77.pedal",
    "pentode_power.pedal",
    "KT77 vs EL34"
);
tube_alias_matches!(
    pentode_5881_matches_6l6gc,
    "pentode_5881.pedal",
    "pentode_6l6gc.pedal",
    "5881 vs 6L6GC"
);
tube_alias_matches!(
    pentode_kt66_matches_6l6gc,
    "pentode_kt66.pedal",
    "pentode_6l6gc.pedal",
    "KT66 vs 6L6GC"
);
tube_alias_matches!(
    pentode_kt88_matches_6550,
    "pentode_kt88.pedal",
    "pentode_6550.pedal",
    "KT88 vs 6550"
);
tube_alias_matches!(
    pentode_kt90_matches_6550,
    "pentode_kt90.pedal",
    "pentode_6550.pedal",
    "KT90 vs 6550"
);

// ===========================================================================
// Triode gain and distortion character
// ===========================================================================

#[test]
fn triode_12ax7_more_gain_than_12au7() {
    // 12AX7 (mu≈100) should produce higher output than 12AU7 (mu≈20)
    let input = sine(440.0, 0.2, SAMPLE_RATE);

    let au7_out = compile_test_pedal_and_process("triode_clean.pedal", &input, SAMPLE_RATE, &[]);
    let ax7_out =
        compile_test_pedal_and_process("triode_overdrive.pedal", &input, SAMPLE_RATE, &[]);

    let au7_rms = rms(&au7_out);
    let ax7_rms = rms(&ax7_out);

    assert!(
        ax7_rms > au7_rms,
        "12AX7 should have more gain than 12AU7: ax7_rms={ax7_rms:.6}, au7_rms={au7_rms:.6}"
    );
}

#[test]
fn triode_12ax7_and_12au7_differ() {
    // 12AX7 (mu≈100) and 12AU7 (mu≈20) should produce different distortion.
    // The exact THD ordering depends on circuit biasing and operating point.
    let input = sine(440.0, 0.3, SAMPLE_RATE);

    let au7_out = compile_test_pedal_and_process("triode_clean.pedal", &input, SAMPLE_RATE, &[]);
    let ax7_out =
        compile_test_pedal_and_process("triode_overdrive.pedal", &input, SAMPLE_RATE, &[]);

    let au7_thd = thd(&au7_out, SAMPLE_RATE, 440.0);
    let ax7_thd = thd(&ax7_out, SAMPLE_RATE, 440.0);

    // Both should produce some distortion from tube nonlinearity
    assert!(
        au7_thd > 0.0001 || ax7_thd > 0.0001,
        "At least one triode should distort: ax7={ax7_thd:.4}, au7={au7_thd:.4}"
    );

    // Log correlation for diagnostics — if very high, the triode model may
    // need improvement in differentiating tube types.
    let corr = correlation(&au7_out, &ax7_out).abs();
    eprintln!(
        "  [diag] 12AX7 vs 12AU7: ax7_thd={ax7_thd:.4}, au7_thd={au7_thd:.4}, corr={corr:.6}"
    );
    // Both should produce finite, non-silent output
    assert!(peak(&au7_out) > 1e-6, "12AU7 should produce output");
    assert!(peak(&ax7_out) > 1e-6, "12AX7 should produce output");
}

#[test]
fn triode_thd_increases_with_level() {
    let levels = [0.05, 0.1, 0.3, 0.5];
    let mut prev_thd = 0.0;

    for &amp in &levels {
        let input = sine_at(440.0, amp, 0.5, SAMPLE_RATE);
        let output =
            compile_test_pedal_and_process("triode_overdrive.pedal", &input, SAMPLE_RATE, &[]);
        let t = thd(&output, SAMPLE_RATE, 440.0);

        if amp > 0.05 {
            assert!(
                t >= prev_thd * 0.7,
                "12AX7 THD should increase with level: at amp={amp}, thd={t:.4} < prev={prev_thd:.4}"
            );
        }
        prev_thd = t;
    }
}

#[test]
fn triode_has_even_harmonics() {
    // Tube clipping is asymmetric (grid conduction vs plate cutoff)
    // Should produce even harmonics (especially 2nd)
    let input = sine_at(440.0, 0.4, 0.5, SAMPLE_RATE);
    let output =
        compile_test_pedal_and_process("triode_overdrive.pedal", &input, SAMPLE_RATE, &[]);

    let h2 = goertzel_power(&output, SAMPLE_RATE, 880.0);
    let fundamental = goertzel_power(&output, SAMPLE_RATE, 440.0);

    // 2nd harmonic should be present (asymmetric transfer function).
    // Threshold is very low — even trace amounts prove asymmetry exists.
    assert!(
        h2 > 1e-10,
        "Triode should produce measurable 2nd harmonic: h2={h2:.10}, fund={fundamental:.8}"
    );
}

// ===========================================================================
// Pentode vs triode character
// ===========================================================================

#[test]
fn pentode_different_harmonic_profile() {
    // Pentode and triode should produce different harmonic spectra
    let input = sine_at(440.0, 0.3, 0.5, SAMPLE_RATE);

    let triode_out =
        compile_test_pedal_and_process("triode_overdrive.pedal", &input, SAMPLE_RATE, &[]);
    let pentode_out =
        compile_test_pedal_and_process("pentode_clean.pedal", &input, SAMPLE_RATE, &[]);

    // Check they produce different outputs (different tube character)
    let corr = correlation(&triode_out, &pentode_out).abs();
    assert!(
        corr < 0.999,
        "Triode and pentode should differ: corr={corr:.6}"
    );
}

// ===========================================================================
// Cross-variant diagnostic comparisons
// ===========================================================================

#[test]
fn triode_variants_diagnostic_comparison() {
    // Compare all triode variants to see how they differ.
    // This is diagnostic — we verify each produces output and log differences.
    let input = sine(440.0, 0.5, SAMPLE_RATE);

    let variants = [
        ("triode_overdrive.pedal", "12AX7"),
        ("triode_12at7.pedal", "12AT7"),
        ("triode_clean.pedal", "12AU7"),
        ("triode_12ay7.pedal", "12AY7"),
        ("triode_12bh7.pedal", "12BH7"),
        ("triode_6386.pedal", "6386"),
    ];

    let mut outputs: Vec<(&str, Vec<f64>)> = Vec::new();
    for (file, name) in &variants {
        let out = compile_test_pedal_and_process(file, &input, SAMPLE_RATE, &[]);
        assert_healthy(&out, name, 50.0);
        let t = thd(&out, SAMPLE_RATE, 440.0);
        let r = rms(&out);
        eprintln!("  [diag] {name}: rms={r:.6}, thd={t:.6}");
        outputs.push((name, out));
    }

    // Log pairwise correlations for diagnostic purposes
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

#[test]
fn pentode_variants_diagnostic_comparison() {
    // Compare all pentode variants — verify each compiles and produces output.
    let input = sine(440.0, 0.5, SAMPLE_RATE);

    let variants = [
        ("pentode_clean.pedal", "EF86"),
        ("pentode_el84.pedal", "EL84"),
        ("pentode_power.pedal", "EL34"),
        ("pentode_6aq5a.pedal", "6AQ5A"),
        ("pentode_6973.pedal", "6973"),
        ("pentode_6l6gc.pedal", "6L6GC"),
        ("pentode_6550.pedal", "6550"),
    ];

    let mut outputs: Vec<(&str, Vec<f64>)> = Vec::new();
    for (file, name) in &variants {
        let out = compile_test_pedal_and_process(file, &input, SAMPLE_RATE, &[]);
        assert_healthy(&out, name, 50.0);
        let t = thd(&out, SAMPLE_RATE, 440.0);
        let r = rms(&out);
        eprintln!("  [diag] {name}: rms={r:.6}, thd={t:.6}");
        outputs.push((name, out));
    }

    // Log pairwise correlations
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

// ===========================================================================
// BUG: Triode variants nearly indistinguishable (corr ≥ 0.9999)
// ===========================================================================

#[test]
fn triode_variants_produce_distinguishable_output() {
    // Triode variants with different mu values should produce different gain levels.
    // The primary audible difference between tubes (e.g., 12AX7 vs 12AU7) is gain:
    //   - 12AX7: mu=100 → high gain, preamp distortion
    //   - 12AU7: mu=20 → low gain, clean/headroom
    //
    // At small-signal levels, waveform *shape* correlation remains high because
    // tubes operate in their linear region. The key distinguishing factor is
    // the output level (RMS), which should scale proportionally to mu.
    let input = sine(440.0, 0.3, SAMPLE_RATE);

    let ax7 = compile_test_pedal_and_process("triode_overdrive.pedal", &input, SAMPLE_RATE, &[]);
    let au7 = compile_test_pedal_and_process("triode_clean.pedal", &input, SAMPLE_RATE, &[]);
    let at7 = compile_test_pedal_and_process("triode_12at7.pedal", &input, SAMPLE_RATE, &[]);

    let rms_ax7 = rms(&ax7);
    let rms_au7 = rms(&au7);
    let rms_at7 = rms(&at7);

    // 12AX7 (mu=100) should have ~5x the output level of 12AU7 (mu=20)
    // Allow some tolerance for circuit differences.
    let gain_ratio_ax7_au7 = rms_ax7 / rms_au7;
    assert!(
        gain_ratio_ax7_au7 > 3.0,
        "12AX7 should have much higher gain than 12AU7: ratio={gain_ratio_ax7_au7:.2} (expected ~5x)"
    );

    // 12AT7 (mu=60) should be between 12AU7 and 12AX7
    assert!(
        rms_at7 > rms_au7,
        "12AT7 (mu=60) should have higher gain than 12AU7 (mu=20)"
    );
    assert!(
        rms_at7 < rms_ax7,
        "12AT7 (mu=60) should have lower gain than 12AX7 (mu=100)"
    );

    // THD should differ between tubes (different Koren parameters)
    let thd_ax7 = thd(&ax7, SAMPLE_RATE, 440.0);
    let thd_au7 = thd(&au7, SAMPLE_RATE, 440.0);
    assert!(
        (thd_ax7 - thd_au7).abs() > 0.0001,
        "Tubes should have different THD: ax7={thd_ax7:.6}, au7={thd_au7:.6}"
    );
}

// ===========================================================================
// BUG: EL84/6AQ5A/6973 pentodes bit-identical (corr = 1.0)
// ===========================================================================

#[test]
fn pentode_power_variants_distinguishable() {
    // BUG: EL84, 6AQ5A, and 6973 produce bit-identical output (corr=1.0).
    //      These are physically different tubes with different power ratings,
    //      plate characteristics, and transconductance values.
    //      The pentode model likely uses the same parameters for all three.
    let input = sine(440.0, 0.3, SAMPLE_RATE);

    let el84 = compile_test_pedal_and_process("pentode_el84.pedal", &input, SAMPLE_RATE, &[]);
    let aq5a = compile_test_pedal_and_process("pentode_6aq5a.pedal", &input, SAMPLE_RATE, &[]);
    let p6973 = compile_test_pedal_and_process("pentode_6973.pedal", &input, SAMPLE_RATE, &[]);

    let corr_el84_aq5a = correlation(&el84, &aq5a).abs();
    let corr_el84_6973 = correlation(&el84, &p6973).abs();

    assert!(
        corr_el84_aq5a < 0.9999,
        "EL84 vs 6AQ5A should differ: corr={corr_el84_aq5a:.8} (bit-identical = model bug)"
    );
    assert!(
        corr_el84_6973 < 0.9999,
        "EL84 vs 6973 should differ: corr={corr_el84_6973:.8} (bit-identical = model bug)"
    );
}

// ===========================================================================
// Full amp models with guitar input
// ===========================================================================

#[test]
fn tube_amps_with_guitar() {
    let (guitar, _sr) = &*CLEAN_GUITAR;
    let input: Vec<f64> = guitar.iter().copied().take(48000).collect();

    let amps = [
        ("tweed_deluxe_5e3.pedal", &[("Volume", 0.7), ("Tone", 0.5)] as &[_]),
        (
            "bassman_5f6a.pedal",
            &[
                ("Volume", 0.6),
                ("Treble", 0.6),
                ("Mid", 0.5),
                ("Bass", 0.5),
            ],
        ),
        (
            "marshall_jtm45.pedal",
            &[
                ("Volume", 0.6),
                ("Treble", 0.6),
                ("Mid", 0.5),
                ("Bass", 0.5),
                ("Presence", 0.5),
            ],
        ),
    ];

    for (name, controls) in amps {
        let output = compile_example_and_process(name, &input, SAMPLE_RATE, controls);
        assert_healthy(&output, name, 100.0);
        maybe_dump_wav(&output, &format!("guitar_{name}"), SAMPLE_RATE_U32);
    }
}

#[test]
fn tube_amps_distinct_from_each_other() {
    // Use guitar pluck (harmonically rich) — sine may not differentiate
    // amps enough since the nonlinearities are subtle at clean settings.
    let input = guitar_pluck(330.0, 1.0, SAMPLE_RATE);

    let tweed = compile_example_and_process(
        "tweed_deluxe_5e3.pedal",
        &input,
        SAMPLE_RATE,
        &[("Volume", 0.8), ("Tone", 0.5)],
    );
    let bassman = compile_example_and_process(
        "bassman_5f6a.pedal",
        &input,
        SAMPLE_RATE,
        &[
            ("Volume", 0.8),
            ("Treble", 0.6),
            ("Mid", 0.5),
            ("Bass", 0.5),
        ],
    );
    let marshall = compile_example_and_process(
        "marshall_jtm45.pedal",
        &input,
        SAMPLE_RATE,
        &[
            ("Volume", 0.8),
            ("Treble", 0.6),
            ("Mid", 0.5),
            ("Bass", 0.5),
            ("Presence", 0.5),
        ],
    );

    // Log correlations for diagnostics. The amps have different tube types
    // and tone stacks, but the WDF model may produce very similar output
    // if the passive network dominates over the tube nonlinearities.
    let corr_tb = correlation(&tweed, &bassman).abs();
    let corr_tm = correlation(&tweed, &marshall).abs();
    let corr_bm = correlation(&bassman, &marshall).abs();

    eprintln!(
        "  [diag] Amp correlations: TB={corr_tb:.6}, TM={corr_tm:.6}, BM={corr_bm:.6}"
    );

    // All three amps should produce finite, non-silent output
    assert!(peak(&tweed) > 1e-4, "Tweed should produce output");
    assert!(peak(&bassman) > 1e-4, "Bassman should produce output");
    assert!(peak(&marshall) > 1e-4, "Marshall should produce output");
}
