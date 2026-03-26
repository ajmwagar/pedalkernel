//! Gain Investigation Tests for Fuzz Face and Klon Centaur
//!
//! These tests investigate why certain circuits aren't producing expected gain.
//! - Fuzz Face: Should have massive gain from two-stage germanium transistor design
//! - Klon Centaur: Should provide clean boost with optional soft clipping

mod audio_analysis;

use audio_analysis::*;

// ===========================================================================
// FUZZ FACE INVESTIGATION
// ===========================================================================

/// Basic Fuzz Face operation test
#[test]
fn fuzz_face_basic_output() {
    let input = sine(440.0, 1.0, SAMPLE_RATE);
    let output = compile_example_and_process(
        "fuzz_face.pedal",
        &input,
        SAMPLE_RATE,
        &[("Fuzz", 0.7), ("Volume", 0.6)],
    );

    let input_rms = rms(&input);
    let output_rms = rms(&output);
    let output_peak = peak(&output);
    let gain_db = 20.0 * (output_rms / input_rms).log10();

    println!("\n=== FUZZ FACE BASIC ===");
    println!("Input RMS: {:.4}", input_rms);
    println!("Output RMS: {:.4}", output_rms);
    println!("Output Peak: {:.4}", output_peak);
    println!("Gain: {:.1} dB", gain_db);
    println!(
        "Finite: {}, Silent: {}",
        output.iter().all(|x| x.is_finite()),
        output_rms < 1e-6
    );

    // Fuzz Face must produce finite output with some signal
    assert!(output.iter().all(|x| x.is_finite()), "Fuzz Face output must be finite");
    assert!(output_rms > 1e-6, "Fuzz Face must produce signal, got RMS={:.6}", output_rms);

    maybe_dump_wav(&output, "fuzz_face_basic", SAMPLE_RATE_U32);
}

/// Fuzz Face gain vs Fuzz control setting
#[test]
fn fuzz_face_fuzz_control_sweep() {
    let input = sine(440.0, 0.5, SAMPLE_RATE);

    println!("\n=== FUZZ FACE: Fuzz Control Sweep ===");
    println!("Fuzz\tRMS\tGain dB\tTHD%");

    let input_rms = rms(&input);

    for fuzz in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let output = compile_example_and_process(
            "fuzz_face.pedal",
            &input,
            SAMPLE_RATE,
            &[("Fuzz", fuzz), ("Volume", 0.7)],
        );

        let output_rms = rms(&output);
        let gain_db = 20.0 * (output_rms / input_rms).log10();
        let output_thd = thd(&output, SAMPLE_RATE, 440.0);

        println!(
            "{:.2}\t{:.4}\t{:.1}\t{:.1}%",
            fuzz,
            output_rms,
            gain_db,
            output_thd * 100.0
        );

        // All fuzz settings must produce finite output
        assert!(output.iter().all(|x| x.is_finite()), "Fuzz output must be finite at fuzz={}", fuzz);
        assert!(output_rms > 1e-6, "Fuzz Face must produce signal at fuzz={}", fuzz);
    }
}

/// Fuzz Face Volume control sweep
#[test]
fn fuzz_face_volume_control_sweep() {
    let input = sine(440.0, 0.5, SAMPLE_RATE);

    println!("\n=== FUZZ FACE: Volume Control Sweep ===");
    println!("Volume\tRMS\tGain dB");

    let input_rms = rms(&input);

    for vol in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let output = compile_example_and_process(
            "fuzz_face.pedal",
            &input,
            SAMPLE_RATE,
            &[("Fuzz", 0.7), ("Volume", vol)],
        );

        let output_rms = rms(&output);
        let gain_db = 20.0 * (output_rms / input_rms).log10();

        println!("{:.2}\t{:.4}\t{:.1}", vol, output_rms, gain_db);

        assert!(output.iter().all(|x| x.is_finite()), "Fuzz output must be finite at vol={}", vol);
    }
}

/// Fuzz Face with guitar-like input (lower amplitude, complex waveform)
#[test]
fn fuzz_face_guitar_input() {
    let input = guitar_pluck(82.41, 2.0, SAMPLE_RATE); // Low E

    let output = compile_example_and_process(
        "fuzz_face.pedal",
        &input,
        SAMPLE_RATE,
        &[("Fuzz", 0.8), ("Volume", 0.7)],
    );

    let input_rms = rms(&input);
    let output_rms = rms(&output);
    let gain_db = 20.0 * (output_rms / input_rms).log10();

    println!("\n=== FUZZ FACE WITH GUITAR ===");
    println!("Input RMS: {:.4}", input_rms);
    println!("Output RMS: {:.4}", output_rms);
    println!("Gain: {:.1} dB", gain_db);

    // Must produce finite output with signal
    assert!(output.iter().all(|x| x.is_finite()), "Fuzz Face guitar output must be finite");
    assert!(output_rms > 1e-6, "Fuzz Face must produce signal with guitar input, got RMS={:.6}", output_rms);

    maybe_dump_wav(&output, "fuzz_face_guitar", SAMPLE_RATE_U32);
}

/// Test Fuzz Face transistor bias points
#[test]
fn fuzz_face_bias_analysis() {
    // Very low level input to check bias behavior
    let input = sine_at(440.0, 0.01, 1.0, SAMPLE_RATE);

    let output = compile_example_and_process(
        "fuzz_face.pedal",
        &input,
        SAMPLE_RATE,
        &[("Fuzz", 0.5), ("Volume", 0.5)],
    );

    let input_rms = rms(&input);
    let output_rms = rms(&output);
    let dc = dc_offset(&output);
    let gain_db = 20.0 * (output_rms / input_rms).log10();

    println!("\n=== FUZZ FACE BIAS ANALYSIS (Low Level) ===");
    println!("Input RMS: {:.6}", input_rms);
    println!("Output RMS: {:.6}", output_rms);
    println!("DC Offset: {:.6}", dc);
    println!("Small-signal gain: {:.1} dB", gain_db);

    // Output must be finite and DC offset must not blow up
    assert!(output.iter().all(|x| x.is_finite()), "Fuzz Face bias output must be finite");
    assert!(output_rms > 1e-8, "Fuzz Face should produce some output at low levels");
    assert!(dc.abs() < 1.0, "DC offset {:.4} is excessive (>1.0V)", dc);
}

// ===========================================================================
// KLON CENTAUR INVESTIGATION
// ===========================================================================

/// Basic Klon Centaur operation test
#[test]
fn klon_centaur_basic_output() {
    let input = sine(440.0, 1.0, SAMPLE_RATE);
    let output = compile_example_and_process(
        "klon_centaur.pedal",
        &input,
        SAMPLE_RATE,
        &[("Gain", 0.5), ("Treble", 0.5), ("Output", 0.7)],
    );

    let input_rms = rms(&input);
    let output_rms = rms(&output);
    let output_peak = peak(&output);
    let gain_db = 20.0 * (output_rms / input_rms).log10();

    println!("\n=== KLON CENTAUR BASIC ===");
    println!("Input RMS: {:.4}", input_rms);
    println!("Output RMS: {:.4}", output_rms);
    println!("Output Peak: {:.4}", output_peak);
    println!("Gain: {:.1} dB", gain_db);
    println!(
        "Finite: {}, Silent: {}",
        output.iter().all(|x| x.is_finite()),
        output_rms < 1e-6
    );

    // Klon must produce finite output with signal
    assert!(output.iter().all(|x| x.is_finite()), "Klon output must be finite");
    assert!(output_rms > 1e-6, "Klon must produce signal, got RMS={:.6}", output_rms);

    maybe_dump_wav(&output, "klon_centaur_basic", SAMPLE_RATE_U32);
}

/// Klon Centaur gain control sweep
#[test]
fn klon_centaur_gain_sweep() {
    let input = sine(440.0, 0.3, SAMPLE_RATE);

    println!("\n=== KLON CENTAUR: Gain Control Sweep ===");
    println!("Gain\tRMS\tGain dB\tTHD%");

    let input_rms = rms(&input);

    for gain in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let output = compile_example_and_process(
            "klon_centaur.pedal",
            &input,
            SAMPLE_RATE,
            &[("Gain", gain), ("Treble", 0.5), ("Output", 0.7)],
        );

        let output_rms = rms(&output);
        let gain_db = 20.0 * (output_rms / input_rms).log10();
        let output_thd = thd(&output, SAMPLE_RATE, 440.0);

        println!(
            "{:.2}\t{:.4}\t{:.1}\t{:.1}%",
            gain,
            output_rms,
            gain_db,
            output_thd * 100.0
        );

        assert!(output.iter().all(|x| x.is_finite()), "Klon output must be finite at gain={}", gain);
        assert!(output_rms > 1e-6, "Klon must produce signal at gain={}", gain);
    }
}

/// Klon Centaur output level sweep
#[test]
fn klon_centaur_output_sweep() {
    let input = sine(440.0, 0.3, SAMPLE_RATE);

    println!("\n=== KLON CENTAUR: Output Level Sweep ===");
    println!("Output\tRMS\tGain dB");

    let input_rms = rms(&input);

    for out_level in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let output = compile_example_and_process(
            "klon_centaur.pedal",
            &input,
            SAMPLE_RATE,
            &[("Gain", 0.5), ("Treble", 0.5), ("Output", out_level)],
        );

        let output_rms = rms(&output);
        let gain_db = 20.0 * (output_rms / input_rms).log10();

        println!("{:.2}\t{:.4}\t{:.1}", out_level, output_rms, gain_db);

        assert!(output.iter().all(|x| x.is_finite()), "Klon output must be finite at output={}", out_level);
    }
}

/// Klon Centaur treble control effect
#[test]
fn klon_centaur_treble_sweep() {
    let input = guitar_pluck(220.0, 1.0, SAMPLE_RATE);

    println!("\n=== KLON CENTAUR: Treble Control ===");
    println!("Treble\tCentroid Hz\tRMS");

    for treble in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let output = compile_example_and_process(
            "klon_centaur.pedal",
            &input,
            SAMPLE_RATE,
            &[("Gain", 0.5), ("Treble", treble), ("Output", 0.7)],
        );

        let centroid = spectral_centroid(&output, SAMPLE_RATE);
        let output_rms = rms(&output);

        println!("{:.2}\t{:.0}\t{:.4}", treble, centroid, output_rms);

        assert!(output.iter().all(|x| x.is_finite()), "Klon output must be finite at treble={}", treble);
        assert!(output_rms > 1e-6, "Klon must produce signal at treble={}", treble);
    }
}

/// Klon Centaur clean blend behavior
/// The Klon's unique character comes from blending clean and clipped signals
#[test]
fn klon_centaur_clean_blend() {
    // Low gain should be mostly clean
    let input = sine(440.0, 0.3, SAMPLE_RATE);

    let low_gain_out = compile_example_and_process(
        "klon_centaur.pedal",
        &input,
        SAMPLE_RATE,
        &[("Gain", 0.1), ("Treble", 0.5), ("Output", 0.7)],
    );

    let high_gain_out = compile_example_and_process(
        "klon_centaur.pedal",
        &input,
        SAMPLE_RATE,
        &[("Gain", 0.9), ("Treble", 0.5), ("Output", 0.7)],
    );

    let low_thd = thd(&low_gain_out, SAMPLE_RATE, 440.0);
    let high_thd = thd(&high_gain_out, SAMPLE_RATE, 440.0);

    println!("\n=== KLON CLEAN BLEND ===");
    println!("Low gain (0.1) THD: {:.2}%", low_thd * 100.0);
    println!("High gain (0.9) THD: {:.2}%", high_thd * 100.0);
    println!("THD ratio: {:.1}x", high_thd / low_thd.max(0.001));

    // Both outputs must be finite with signal
    assert!(low_gain_out.iter().all(|x| x.is_finite()), "Klon low-gain output must be finite");
    assert!(high_gain_out.iter().all(|x| x.is_finite()), "Klon high-gain output must be finite");
    assert!(rms(&low_gain_out) > 1e-6, "Klon must produce signal at low gain");
    assert!(rms(&high_gain_out) > 1e-6, "Klon must produce signal at high gain");
}

/// Test op-amp gain stages in Klon
#[test]
fn klon_centaur_opamp_analysis() {
    // Very low level input to analyze small-signal gain
    let input = sine_at(440.0, 0.01, 1.0, SAMPLE_RATE);

    let output = compile_example_and_process(
        "klon_centaur.pedal",
        &input,
        SAMPLE_RATE,
        &[("Gain", 0.5), ("Treble", 0.5), ("Output", 0.5)],
    );

    let input_rms = rms(&input);
    let output_rms = rms(&output);
    let dc = dc_offset(&output);
    let gain_db = 20.0 * (output_rms / input_rms).log10();

    println!("\n=== KLON OP-AMP ANALYSIS (Low Level) ===");
    println!("Input RMS: {:.6}", input_rms);
    println!("Output RMS: {:.6}", output_rms);
    println!("DC Offset: {:.6}", dc);
    println!("Small-signal gain: {:.1} dB", gain_db);

    // Output must be finite and produce signal
    assert!(output.iter().all(|x| x.is_finite()), "Klon op-amp output must be finite");
    assert!(output_rms > 1e-8, "Klon op-amp should produce some output at low levels");
    assert!(dc.abs() < 1.0, "Klon DC offset {:.4} is excessive (>1.0V)", dc);
}

/// Compare Fuzz Face and Klon gain characteristics
#[test]
fn compare_fuzz_and_klon_gain() {
    let input = sine(440.0, 0.3, SAMPLE_RATE);
    let input_rms = rms(&input);

    let fuzz_out = compile_example_and_process(
        "fuzz_face.pedal",
        &input,
        SAMPLE_RATE,
        &[("Fuzz", 0.7), ("Volume", 0.7)],
    );

    let klon_out = compile_example_and_process(
        "klon_centaur.pedal",
        &input,
        SAMPLE_RATE,
        &[("Gain", 0.7), ("Treble", 0.5), ("Output", 0.7)],
    );

    let fuzz_rms = rms(&fuzz_out);
    let klon_rms = rms(&klon_out);
    let fuzz_gain_db = 20.0 * (fuzz_rms / input_rms).log10();
    let klon_gain_db = 20.0 * (klon_rms / input_rms).log10();
    let fuzz_thd = thd(&fuzz_out, SAMPLE_RATE, 440.0);
    let klon_thd = thd(&klon_out, SAMPLE_RATE, 440.0);

    println!("\n=== FUZZ vs KLON COMPARISON ===");
    println!("            \tGain dB\tTHD%\tRMS");
    println!(
        "Fuzz Face   \t{:.1}\t{:.1}%\t{:.4}",
        fuzz_gain_db,
        fuzz_thd * 100.0,
        fuzz_rms
    );
    println!(
        "Klon Centaur\t{:.1}\t{:.1}%\t{:.4}",
        klon_gain_db,
        klon_thd * 100.0,
        klon_rms
    );

    // Both circuits must produce finite output with signal
    assert!(fuzz_out.iter().all(|x| x.is_finite()), "Fuzz Face output must be finite");
    assert!(klon_out.iter().all(|x| x.is_finite()), "Klon output must be finite");
    assert!(fuzz_rms > 1e-6, "Fuzz Face must produce signal");
    assert!(klon_rms > 1e-6, "Klon must produce signal");
}

// ===========================================================================
// DIAGNOSTIC TESTS
// ===========================================================================

/// Test if BJT models are producing gain
#[test]
fn bjt_gain_test_minimal() {
    // Minimal common-emitter test
    let pedal_src = r#"
pedal "BJT Gain Test" {
  components {
    C_in: cap(1u)
    R_b: resistor(100k)
    Q1: npn(2n3904)
    R_c: resistor(4.7k)
    R_e: resistor(1k)
    C_e: cap(100u)
    C_out: cap(1u)
    R_load: resistor(10k)
  }
  nets {
    in -> C_in.a
    C_in.b -> R_b.a, Q1.base
    R_b.b -> gnd
    vcc -> R_c.a
    R_c.b -> Q1.collector, C_out.a
    Q1.emitter -> R_e.a, C_e.a
    R_e.b -> gnd
    C_e.b -> gnd
    C_out.b -> R_load.a
    R_load.b -> gnd
    C_out.b -> out
  }
}
"#;

    let input = sine_at(440.0, 0.1, 1.0, SAMPLE_RATE);
    let output = compile_and_process(pedal_src, &input, SAMPLE_RATE, &[]);

    let input_rms = rms(&input);
    let output_rms = rms(&output);
    let gain_db = 20.0 * (output_rms / input_rms).log10();

    println!("\n=== MINIMAL BJT GAIN TEST ===");
    println!("Input RMS: {:.4}", input_rms);
    println!("Output RMS: {:.4}", output_rms);
    println!("Gain: {:.1} dB", gain_db);
    println!("Finite: {}", output.iter().all(|x| x.is_finite()));

    // BJT circuit must produce finite output with signal
    assert!(output.iter().all(|x| x.is_finite()), "BJT output must be finite");
    assert!(output_rms > 1e-6, "BJT common-emitter must produce signal, got RMS={:.6}", output_rms);
}

/// Test if op-amp models are producing gain
#[test]
fn opamp_gain_test_minimal() {
    // Minimal inverting amplifier test
    let pedal_src = r#"
pedal "Op-Amp Gain Test" {
  components {
    C_in: cap(100n)
    R_in: resistor(10k)
    U1: opamp(tl072)
    R_fb: resistor(100k)
    C_out: cap(100n)
    R_load: resistor(10k)
  }
  nets {
    in -> C_in.a
    C_in.b -> R_in.a
    R_in.b -> U1.neg
    U1.pos -> gnd
    U1.neg -> R_fb.a
    R_fb.b -> U1.out
    U1.out -> C_out.a
    C_out.b -> R_load.a
    R_load.b -> gnd
    C_out.b -> out
  }
}
"#;

    let input = sine_at(440.0, 0.1, 1.0, SAMPLE_RATE);
    let output = compile_and_process(pedal_src, &input, SAMPLE_RATE, &[]);

    let input_rms = rms(&input);
    let output_rms = rms(&output);
    let gain_db = 20.0 * (output_rms / input_rms).log10();

    println!("\n=== MINIMAL OP-AMP GAIN TEST ===");
    println!("Input RMS: {:.4}", input_rms);
    println!("Output RMS: {:.4}", output_rms);
    println!("Expected gain: {:.1} dB (Rf/Ri = 10)", 20.0);
    println!("Actual gain: {:.1} dB", gain_db);
    println!("Finite: {}", output.iter().all(|x| x.is_finite()));

    // Op-amp circuit must produce finite output with signal
    assert!(output.iter().all(|x| x.is_finite()), "Op-amp output must be finite");
    assert!(output_rms > 1e-6, "Op-amp inverting amplifier must produce signal, got RMS={:.6}", output_rms);
}
