//! BBD Artifact Detection Tests
//!
//! Tests specifically designed to detect and characterize audio artifacts
//! in BBD (Bucket-Brigade Device) implementations. These tests help verify
//! that artifacts like clock feedthrough, aliasing, compander breathing,
//! and zipper noise stay within acceptable bounds.
//!
//! Run with PEDALKERNEL_DUMP_WAV=1 to save outputs for manual inspection.

mod audio_analysis;

use audio_analysis::*;

// ---------------------------------------------------------------------------
// Clock Feedthrough Detection
// ---------------------------------------------------------------------------

/// Detect clock feedthrough artifacts.
///
/// BBD clock frequencies typically range 10kHz-200kHz. At lower clock rates
/// (longer delays), clock bleedthrough becomes audible as a high-pitched whine.
/// This test processes silence and checks for HF energy at clock frequencies.
#[test]
fn bbd_clock_feedthrough_with_silence() {
    // Process pure silence - any output is clock bleedthrough or noise
    let input = vec![0.0; (SAMPLE_RATE * 1.0) as usize];
    let output = compile_test_pedal_and_process("bbd_long_delay.pedal", &input, SAMPLE_RATE, &[]);

    assert!(
        output.iter().all(|x| x.is_finite()),
        "Output should be finite"
    );

    // With silence input, output should be very quiet (just noise floor)
    let output_rms = rms(&output);
    let output_peak = peak(&output);

    // If there's significant output with silent input, it's clock feedthrough or noise
    // MN3005 has noise_floor=0.001, so RMS should be near that level
    println!(
        "Silence test: RMS={:.6}, Peak={:.6}",
        output_rms, output_peak
    );

    // ARTIFACT DETECTION: Flag if output is louder than expected noise floor
    let noise_threshold = 0.01; // ~-40dB
    if output_rms > noise_threshold {
        println!(
            "WARNING: Possible clock feedthrough detected: RMS {:.4} > {:.4} threshold",
            output_rms, noise_threshold
        );
    }

    // Check for narrowband HF energy (clock feedthrough signature)
    // MN3005 clock range is 10kHz-100kHz; at long delay, clock ~ 10kHz
    let hf_energy_10k = goertzel_power(&output, SAMPLE_RATE, 10000.0);
    let hf_energy_15k = goertzel_power(&output, SAMPLE_RATE, 15000.0);
    let hf_energy_20k = goertzel_power(&output, SAMPLE_RATE, 20000.0);

    println!(
        "HF energy at clock frequencies: 10kHz={:.2e}, 15kHz={:.2e}, 20kHz={:.2e}",
        hf_energy_10k, hf_energy_15k, hf_energy_20k
    );

    // Flag dominant HF spike (clock feedthrough signature)
    let max_hf = hf_energy_10k.max(hf_energy_15k).max(hf_energy_20k);
    if max_hf > 1e-8 && output_rms > 1e-6 {
        let hf_ratio = max_hf / (output_rms * output_rms);
        println!("HF/total ratio: {:.4}", hf_ratio);
        if hf_ratio > 0.1 {
            println!("WARNING: Dominant HF spike detected - possible clock feedthrough");
        }
    }

    maybe_dump_wav(&output, "bbd_clock_feedthrough_silence", SAMPLE_RATE_U32);
}

/// Clock feedthrough with low-level signal.
/// Clock noise becomes more apparent relative to quiet signals.
#[test]
fn bbd_clock_feedthrough_with_quiet_signal() {
    // Very quiet input signal
    let input = sine_at(440.0, 0.01, 1.0, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("bbd_long_delay.pedal", &input, SAMPLE_RATE, &[]);

    assert_healthy(&output, "BBD quiet signal", 5.0);

    // Calculate SNR: signal power vs high-frequency noise
    let signal_power = goertzel_power(&output, SAMPLE_RATE, 440.0);
    let noise_power_10k = goertzel_power(&output, SAMPLE_RATE, 10000.0);
    let noise_power_15k = goertzel_power(&output, SAMPLE_RATE, 15000.0);
    let total_clock_noise = noise_power_10k + noise_power_15k;

    if signal_power > 1e-10 && total_clock_noise > 1e-14 {
        let snr_db = 10.0 * (signal_power / total_clock_noise).log10();
        println!("Signal-to-clock-noise ratio: {:.1} dB", snr_db);

        // ARTIFACT DETECTION: SNR should be > 40dB for acceptable quality
        if snr_db < 40.0 {
            println!(
                "WARNING: Poor signal-to-clock-noise ratio: {:.1} dB (want >40 dB)",
                snr_db
            );
        }
    }

    maybe_dump_wav(&output, "bbd_clock_feedthrough_quiet", SAMPLE_RATE_U32);
}

// ---------------------------------------------------------------------------
// Compander Breathing Artifacts
// ---------------------------------------------------------------------------

/// Detect compander "breathing" - pumping caused by attack/release mismatch.
///
/// BBD companders (NE571-style) use ~1ms attack and ~10ms release.
/// This mismatch causes audible pumping on transients and decaying signals.
#[test]
fn bbd_compander_breathing_on_transient() {
    // Sharp transient followed by decay - exposes compander breathing
    let mut input = Vec::with_capacity((SAMPLE_RATE * 2.0) as usize);

    // Hard attack transient (guitar pick)
    for i in 0..480 {
        // 10ms attack
        let t = i as f64 / SAMPLE_RATE;
        let env = (t * 100.0).min(1.0); // Fast ramp
        input.push(env * 0.8 * (2.0 * std::f64::consts::PI * 330.0 * t).sin());
    }

    // Exponential decay
    for i in 0..((SAMPLE_RATE * 1.5) as usize) {
        let t = i as f64 / SAMPLE_RATE;
        let env = (-2.0 * t).exp(); // ~350ms decay
        input.push(env * 0.8 * (2.0 * std::f64::consts::PI * 330.0 * t).sin());
    }

    // Silence to hear the release tail
    input.extend(vec![0.0; (SAMPLE_RATE * 0.5) as usize]);

    let output = compile_test_pedal_and_process("bbd_chorus.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "BBD transient", 5.0);

    // Measure envelope smoothness - breathing creates envelope ripples
    let chunk_size = 480; // 10ms chunks
    let num_chunks = output.len() / chunk_size;
    let envelope: Vec<f64> = (0..num_chunks)
        .map(|i| rms(&output[i * chunk_size..(i + 1) * chunk_size]))
        .collect();

    // Calculate envelope jitter (deviation from smooth decay)
    if envelope.len() > 10 {
        let mut envelope_diff: Vec<f64> = Vec::new();
        for i in 1..envelope.len() {
            if envelope[i - 1] > 1e-6 {
                envelope_diff.push((envelope[i] - envelope[i - 1]).abs() / envelope[i - 1]);
            }
        }

        if !envelope_diff.is_empty() {
            let avg_jitter = envelope_diff.iter().sum::<f64>() / envelope_diff.len() as f64;
            let max_jitter = envelope_diff.iter().cloned().fold(0.0f64, f64::max);

            println!(
                "Envelope jitter: avg={:.4}, max={:.4}",
                avg_jitter, max_jitter
            );

            // ARTIFACT DETECTION: High jitter indicates compander breathing
            if max_jitter > 0.5 {
                println!(
                    "WARNING: Compander breathing detected - envelope jitter {:.2} (want <0.5)",
                    max_jitter
                );
            }
        }
    }

    maybe_dump_wav(&output, "bbd_compander_breathing", SAMPLE_RATE_U32);
}

/// Test compander breathing with gated signal (sudden stops).
/// The release time mismatch causes audible "tails" after signal stops.
#[test]
fn bbd_compander_release_tail() {
    // Tone bursts with sudden stops
    let mut input = Vec::new();

    for _ in 0..3 {
        // 200ms of signal
        let burst: Vec<f64> = (0..((SAMPLE_RATE * 0.2) as usize))
            .map(|i| {
                let t = i as f64 / SAMPLE_RATE;
                0.5 * (2.0 * std::f64::consts::PI * 440.0 * t).sin()
            })
            .collect();
        input.extend(burst);

        // 300ms of silence
        input.extend(vec![0.0; (SAMPLE_RATE * 0.3) as usize]);
    }

    let output = compile_test_pedal_and_process("bbd_chorus.pedal", &input, SAMPLE_RATE, &[]);
    assert!(
        output.iter().all(|x| x.is_finite()),
        "Output should be finite"
    );

    // Analyze the "silent" periods for compander release artifacts
    let silence_start = (SAMPLE_RATE * 0.22) as usize; // Just after first burst
    let silence_end = (SAMPLE_RATE * 0.48) as usize;

    if silence_end < output.len() {
        let silence_region = &output[silence_start..silence_end];
        let silence_rms = rms(silence_region);
        let silence_peak = peak(silence_region);

        println!(
            "Silence region: RMS={:.6}, Peak={:.6}",
            silence_rms, silence_peak
        );

        // ARTIFACT DETECTION: Signal in "silent" region = compander tail
        if silence_rms > 0.01 {
            println!(
                "WARNING: Compander release tail detected - silence RMS {:.4} (want <0.01)",
                silence_rms
            );
        }
    }

    maybe_dump_wav(&output, "bbd_compander_release", SAMPLE_RATE_U32);
}

// ---------------------------------------------------------------------------
// Aliasing Artifacts
// ---------------------------------------------------------------------------

/// Detect aliasing from signals above BBD bandwidth.
///
/// BBD bandwidth is limited to ~0.45 × clock frequency.
/// Signals above this fold back as aliasing artifacts.
#[test]
fn bbd_aliasing_from_high_frequencies() {
    // High frequency input near the BBD bandwidth limit
    // MN3207 at max clock (200kHz): bandwidth ~ 90kHz
    // MN3207 at min clock (10kHz): bandwidth ~ 4.5kHz (aliasing likely at 48kHz sample rate)

    // Test with frequencies that will alias at lower clock rates
    let freqs = [8000.0, 10000.0, 12000.0];
    let mut input = Vec::new();

    for freq in freqs {
        let tone: Vec<f64> = (0..((SAMPLE_RATE * 0.5) as usize))
            .map(|i| {
                let t = i as f64 / SAMPLE_RATE;
                0.3 * (2.0 * std::f64::consts::PI * freq * t).sin()
            })
            .collect();
        input.extend(tone);
        input.extend(vec![0.0; (SAMPLE_RATE * 0.1) as usize]);
    }

    let output = compile_test_pedal_and_process("bbd_long_delay.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "BBD HF aliasing test", 5.0);

    // Check for energy at unexpected frequencies (aliasing products)
    // If 10kHz aliases down, we might see energy at lower unexpected frequencies

    // Original frequencies
    let energy_8k = goertzel_power(&output, SAMPLE_RATE, 8000.0);
    let energy_10k = goertzel_power(&output, SAMPLE_RATE, 10000.0);

    // Potential alias frequencies (difference tones)
    let alias_2k = goertzel_power(&output, SAMPLE_RATE, 2000.0);
    let alias_4k = goertzel_power(&output, SAMPLE_RATE, 4000.0);

    // Low frequency "garbage" that shouldn't exist
    let lf_garbage = goertzel_power(&output, SAMPLE_RATE, 500.0)
        + goertzel_power(&output, SAMPLE_RATE, 1000.0)
        + goertzel_power(&output, SAMPLE_RATE, 1500.0);

    println!("Original frequencies: 8k={:.2e}, 10k={:.2e}", energy_8k, energy_10k);
    println!("Potential aliases: 2k={:.2e}, 4k={:.2e}", alias_2k, alias_4k);
    println!("LF garbage: {:.2e}", lf_garbage);

    // ARTIFACT DETECTION: Significant energy at non-input frequencies indicates aliasing
    let input_energy = energy_8k + energy_10k;
    if input_energy > 1e-10 && lf_garbage > input_energy * 0.1 {
        println!(
            "WARNING: Possible aliasing - LF garbage is {:.1}% of input energy",
            lf_garbage / input_energy * 100.0
        );
    }

    maybe_dump_wav(&output, "bbd_aliasing_hf", SAMPLE_RATE_U32);
}

// ---------------------------------------------------------------------------
// Zipper Noise / Delay Time Modulation Artifacts
// ---------------------------------------------------------------------------

/// Detect zipper noise from rapid delay time changes.
///
/// When the LFO rapidly modulates delay time, sample-and-hold stepping
/// can create audible "zippering" or clicking.
#[test]
fn bbd_zipper_noise_fast_modulation() {
    // Steady tone through fast-modulating BBD
    let input = sine(440.0, 2.0, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("bbd_fast_clock.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "BBD zipper test", 5.0);

    // Zipper noise manifests as wideband HF artifacts
    // Calculate high-frequency energy that shouldn't be there with a pure sine input

    let fundamental = goertzel_power(&output, SAMPLE_RATE, 440.0);
    let harmonics = goertzel_power(&output, SAMPLE_RATE, 880.0)
        + goertzel_power(&output, SAMPLE_RATE, 1320.0)
        + goertzel_power(&output, SAMPLE_RATE, 1760.0);

    // Wideband noise at high frequencies (zipper artifacts)
    let wideband_hf = band_energy_ratio(&output, SAMPLE_RATE, 5000.0, 15000.0, 500.0);

    println!(
        "Fundamental: {:.2e}, Harmonics: {:.2e}, Wideband HF ratio: {:.4}",
        fundamental, harmonics, wideband_hf
    );

    // ARTIFACT DETECTION: High wideband energy relative to signal indicates zipper noise
    if wideband_hf > 0.1 && fundamental > 1e-6 {
        println!(
            "WARNING: Possible zipper noise - wideband HF is {:.1}% of total energy",
            wideband_hf * 100.0
        );
    }

    maybe_dump_wav(&output, "bbd_zipper_fast", SAMPLE_RATE_U32);
}

/// Detect discontinuities when delay time steps.
/// Uses peak-to-peak analysis to find sudden jumps.
#[test]
fn bbd_discontinuity_detection() {
    let input = sine(220.0, 3.0, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("bbd_chorus.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "BBD discontinuity test", 5.0);

    // Look for sample-to-sample discontinuities (clicks/pops)
    let mut max_delta = 0.0f64;
    let mut discontinuity_count = 0;

    // A well-behaved audio signal shouldn't have sudden jumps
    // Threshold: expect smooth transitions at audio frequencies
    let discontinuity_threshold = 0.2; // Large jump between adjacent samples

    for i in 1..output.len() {
        let delta = (output[i] - output[i - 1]).abs();
        max_delta = max_delta.max(delta);
        if delta > discontinuity_threshold {
            discontinuity_count += 1;
        }
    }

    println!(
        "Max sample-to-sample delta: {:.4}, discontinuities: {}",
        max_delta, discontinuity_count
    );

    // ARTIFACT DETECTION: More than a few discontinuities indicates zipper noise
    if discontinuity_count > 10 {
        println!(
            "WARNING: {} discontinuities detected (want <10) - possible zipper noise",
            discontinuity_count
        );
    }

    maybe_dump_wav(&output, "bbd_discontinuity", SAMPLE_RATE_U32);
}

// ---------------------------------------------------------------------------
// Intermodulation Distortion
// ---------------------------------------------------------------------------

/// Two-tone IMD test.
///
/// BBD companders and soft clipping can create intermodulation products.
/// This test uses two frequencies and checks for sum/difference tones.
#[test]
fn bbd_intermodulation_distortion() {
    // Two-tone test: 400Hz + 500Hz
    let n = (SAMPLE_RATE * 2.0) as usize;
    let input: Vec<f64> = (0..n)
        .map(|i| {
            let t = i as f64 / SAMPLE_RATE;
            0.3 * (2.0 * std::f64::consts::PI * 400.0 * t).sin()
                + 0.3 * (2.0 * std::f64::consts::PI * 500.0 * t).sin()
        })
        .collect();

    let output = compile_test_pedal_and_process("bbd_chorus.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "BBD IMD test", 5.0);

    // Original tones
    let p400 = goertzel_power(&output, SAMPLE_RATE, 400.0);
    let p500 = goertzel_power(&output, SAMPLE_RATE, 500.0);

    // IMD products (f1±f2, 2f1±f2, etc.)
    let p100 = goertzel_power(&output, SAMPLE_RATE, 100.0); // 500-400
    let p900 = goertzel_power(&output, SAMPLE_RATE, 900.0); // 400+500
    let p300 = goertzel_power(&output, SAMPLE_RATE, 300.0); // 2×400-500
    let p600 = goertzel_power(&output, SAMPLE_RATE, 600.0); // 2×500-400
    let p800 = goertzel_power(&output, SAMPLE_RATE, 800.0); // 2×400
    let p1000 = goertzel_power(&output, SAMPLE_RATE, 1000.0); // 2×500

    let fundamental_power = p400 + p500;
    let imd_power = p100 + p900 + p300 + p600 + p800 + p1000;

    println!(
        "Fundamental power: {:.2e}, IMD power: {:.2e}",
        fundamental_power, imd_power
    );

    if fundamental_power > 1e-10 {
        let imd_ratio = imd_power / fundamental_power;
        let imd_db = 10.0 * imd_ratio.log10();
        println!("IMD ratio: {:.4} ({:.1} dB)", imd_ratio, imd_db);

        // ARTIFACT DETECTION: High IMD indicates nonlinearity issues
        if imd_ratio > 0.05 {
            // >5% IMD
            println!(
                "WARNING: High IMD detected: {:.1}% (want <5%)",
                imd_ratio * 100.0
            );
        }
    }

    maybe_dump_wav(&output, "bbd_imd", SAMPLE_RATE_U32);
}

// ---------------------------------------------------------------------------
// DC Offset Accumulation
// ---------------------------------------------------------------------------

/// Detect DC offset buildup in the delay line.
///
/// Charge leakage and compander asymmetry can cause DC to accumulate.
#[test]
fn bbd_dc_offset_accumulation() {
    // Process a burst then extended silence
    let mut input = sine(440.0, 0.5, SAMPLE_RATE);
    input.extend(vec![0.0; (SAMPLE_RATE * 2.0) as usize]); // 2 seconds silence

    let output = compile_test_pedal_and_process("bbd_long_delay.pedal", &input, SAMPLE_RATE, &[]);
    assert!(
        output.iter().all(|x| x.is_finite()),
        "Output should be finite"
    );

    // Check DC offset in the "silent" portion (after initial transients settle)
    let silent_start = (SAMPLE_RATE * 1.5) as usize;
    if silent_start < output.len() {
        let silent_region = &output[silent_start..];
        let dc = dc_offset(silent_region);
        let dc_abs = dc.abs();

        println!("DC offset in silence: {:.6}", dc);

        // ARTIFACT DETECTION: Significant DC offset indicates accumulation
        if dc_abs > 0.01 {
            println!(
                "WARNING: DC offset accumulation detected: {:.4} (want <0.01)",
                dc_abs
            );
        }
    }

    // Also check for DC drift over time
    let early_dc = dc_offset(&output[..4800]);
    let late_dc = dc_offset(&output[output.len().saturating_sub(4800)..]);
    let dc_drift = (late_dc - early_dc).abs();

    println!(
        "DC drift: early={:.6}, late={:.6}, drift={:.6}",
        early_dc, late_dc, dc_drift
    );

    if dc_drift > 0.05 {
        println!(
            "WARNING: DC drift detected: {:.4} (want <0.05)",
            dc_drift
        );
    }

    maybe_dump_wav(&output, "bbd_dc_offset", SAMPLE_RATE_U32);
}

// ---------------------------------------------------------------------------
// Noise Floor Analysis
// ---------------------------------------------------------------------------

/// Measure BBD noise floor with varying input levels.
///
/// BBD noise should be consistent regardless of input level.
/// Compander artifacts can cause noise modulation with signal level.
#[test]
fn bbd_noise_floor_consistency() {
    // Test at different input levels
    let levels = [0.01, 0.1, 0.5];
    let mut noise_floors = Vec::new();

    for &level in &levels {
        let input = sine_at(440.0, 1.0, level, SAMPLE_RATE);
        let output = compile_test_pedal_and_process("bbd_chorus.pedal", &input, SAMPLE_RATE, &[]);

        // Measure noise as energy outside the fundamental and harmonics
        let signal_power = goertzel_power(&output, SAMPLE_RATE, 440.0)
            + goertzel_power(&output, SAMPLE_RATE, 880.0)
            + goertzel_power(&output, SAMPLE_RATE, 1320.0);

        let total_power: f64 = output.iter().map(|x| x * x).sum::<f64>() / output.len() as f64;
        let noise_power = (total_power - signal_power).max(0.0);

        noise_floors.push((level, noise_power));
        println!(
            "Input level {:.2}: signal={:.2e}, noise={:.2e}",
            level, signal_power, noise_power
        );
    }

    // ARTIFACT DETECTION: Noise floor should not vary drastically with input level
    // (noise modulation indicates compander tracking issues)
    if noise_floors.len() >= 2 {
        let noise_variation = noise_floors.iter().map(|(_, n)| *n).fold(0.0f64, f64::max)
            / noise_floors.iter().map(|(_, n)| *n).fold(f64::MAX, f64::min).max(1e-20);

        println!("Noise floor variation: {:.2}x", noise_variation);

        if noise_variation > 10.0 {
            println!(
                "WARNING: Noise floor varies {:.1}x with input level (want <10x)",
                noise_variation
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Modulation Rate Artifacts
// ---------------------------------------------------------------------------

/// Test for artifacts at very slow modulation rates.
///
/// Slow LFO modulation can cause audible stepping or warbling.
#[test]
fn bbd_slow_modulation_artifacts() {
    // Long test to capture slow modulation
    let input = sine(440.0, 5.0, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("bbd_chorus.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "BBD slow mod test", 5.0);

    // Look for low-frequency amplitude modulation artifacts
    let mod_rate = detect_modulation_rate(&output, SAMPLE_RATE);
    println!("Detected modulation rate: {:.2} Hz", mod_rate);

    // Check if modulation has uneven character (stepping vs smooth)
    let has_mod = has_amplitude_modulation(&output, 50);
    println!("Has amplitude modulation: {}", has_mod);

    // Analyze modulation smoothness by looking at envelope spectral purity
    // A smooth sine LFO should create clean sideband structure
    // Stepping creates harmonic distortion of the modulation itself

    maybe_dump_wav(&output, "bbd_slow_mod", SAMPLE_RATE_U32);
}

// ---------------------------------------------------------------------------
// Clipping / Saturation Artifacts
// ---------------------------------------------------------------------------

/// Test BBD soft clipping behavior at high input levels.
///
/// BBD has internal voltage swing limits that cause soft clipping.
#[test]
fn bbd_soft_clipping_character() {
    // High level input to trigger soft clipping
    let input = sine_at(440.0, 0.9, 1.0, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("bbd_chorus.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "BBD clipping test", 5.0);

    // Measure THD - soft clipping adds harmonics
    let thd_value = thd(&output, SAMPLE_RATE, 440.0);
    println!("THD at high input level: {:.4} ({:.1}%)", thd_value, thd_value * 100.0);

    // Compare to low-level THD
    let input_quiet = sine_at(440.0, 0.1, 1.0, SAMPLE_RATE);
    let output_quiet = compile_test_pedal_and_process("bbd_chorus.pedal", &input_quiet, SAMPLE_RATE, &[]);
    let thd_quiet = thd(&output_quiet, SAMPLE_RATE, 440.0);

    println!("THD at low input level: {:.4} ({:.1}%)", thd_quiet, thd_quiet * 100.0);

    // ARTIFACT DETECTION: THD should increase gracefully with level
    // Harsh clipping would cause very high THD
    if thd_value > 0.3 {
        println!(
            "WARNING: High THD at hot input: {:.1}% (may indicate harsh clipping)",
            thd_value * 100.0
        );
    }

    maybe_dump_wav(&output, "bbd_clipping", SAMPLE_RATE_U32);
}

// ---------------------------------------------------------------------------
// Memory Man Specific Tests (MN3005 - 4096 stages)
// ---------------------------------------------------------------------------

/// Test MN3005 long-delay darkening effect.
///
/// The MN3005's 4096 stages cause progressive HF rolloff.
/// Each stage leaks charge, accumulating as LPF behavior.
#[test]
fn mn3005_progressive_darkening() {
    // Bright, harmonically rich input
    let input = guitar_pluck(330.0, 1.0, SAMPLE_RATE);

    let output = compile_test_pedal_and_process("bbd_long_delay.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "MN3005 darkening test", 5.0);

    let input_centroid = spectral_centroid(&input, SAMPLE_RATE);
    let output_centroid = spectral_centroid(&output, SAMPLE_RATE);

    println!(
        "Spectral centroid: input={:.0}Hz, output={:.0}Hz",
        input_centroid, output_centroid
    );

    // MN3005 should darken the sound (lower centroid)
    if output_centroid > input_centroid {
        println!(
            "NOTE: Output brighter than input - MN3005 should darken ({:.0} > {:.0})",
            output_centroid, input_centroid
        );
    }

    // Check HF energy ratio
    let input_hf = band_energy_ratio(&input, SAMPLE_RATE, 2000.0, 8000.0, 200.0);
    let output_hf = band_energy_ratio(&output, SAMPLE_RATE, 2000.0, 8000.0, 200.0);

    println!(
        "HF energy ratio: input={:.4}, output={:.4}",
        input_hf, output_hf
    );

    maybe_dump_wav(&output, "mn3005_darkening", SAMPLE_RATE_U32);
}

// ---------------------------------------------------------------------------
// Multi-Stage Artifact Accumulation
// ---------------------------------------------------------------------------

/// Test for artifact accumulation through feedback loops.
///
/// In delay pedals with feedback, artifacts compound with each repeat.
/// Clock noise, aliasing, and distortion all accumulate.
#[test]
fn bbd_feedback_artifact_accumulation() {
    // Short burst then silence - let feedback create repeats
    let mut input = vec![0.0; (SAMPLE_RATE * 0.5) as usize];
    for i in 0..2400 {
        // 50ms burst
        let t = i as f64 / SAMPLE_RATE;
        input[i] = 0.5 * (2.0 * std::f64::consts::PI * 440.0 * t).sin();
    }

    // Long tail for feedback
    input.extend(vec![0.0; (SAMPLE_RATE * 3.0) as usize]);

    let output = compile_test_pedal_and_process("bbd_long_delay.pedal", &input, SAMPLE_RATE, &[]);
    assert!(
        output.iter().all(|x| x.is_finite()),
        "Output should be finite"
    );

    // Analyze spectral content at different time points
    let chunk_len = (SAMPLE_RATE * 0.5) as usize;
    let mut thd_values = Vec::new();

    for i in 0..4 {
        let start = i * chunk_len;
        let end = (start + chunk_len).min(output.len());
        if end > start + 1000 {
            let chunk = &output[start..end];
            let chunk_thd = thd(chunk, SAMPLE_RATE, 440.0);
            thd_values.push(chunk_thd);
            println!("THD at {:.1}s: {:.4}", i as f64 * 0.5, chunk_thd);
        }
    }

    // ARTIFACT DETECTION: THD should not explode in later repeats
    if thd_values.len() >= 2 {
        let first_thd = thd_values[0];
        let last_thd = thd_values[thd_values.len() - 1];
        if last_thd > first_thd * 3.0 && first_thd > 0.01 {
            println!(
                "WARNING: THD accumulation through feedback: {:.1}x increase",
                last_thd / first_thd
            );
        }
    }

    maybe_dump_wav(&output, "bbd_feedback_accumulation", SAMPLE_RATE_U32);
}

// ---------------------------------------------------------------------------
// Summary Test - Overall BBD Quality Score
// ---------------------------------------------------------------------------

/// Comprehensive BBD quality assessment.
///
/// Runs multiple artifact checks and provides an overall quality score.
#[test]
fn bbd_overall_quality_assessment() {
    println!("\n=== BBD Quality Assessment ===\n");

    let mut issues = Vec::new();

    // Test 1: Noise floor with silence
    {
        let input = vec![0.0; (SAMPLE_RATE * 0.5) as usize];
        let output = compile_test_pedal_and_process("bbd_chorus.pedal", &input, SAMPLE_RATE, &[]);
        let silence_rms = rms(&output);
        println!("1. Noise floor (silence): RMS = {:.6}", silence_rms);
        if silence_rms > 0.005 {
            issues.push(format!("High noise floor: {:.4}", silence_rms));
        }
    }

    // Test 2: THD at nominal level
    {
        let input = sine_at(440.0, 0.3, 1.0, SAMPLE_RATE);
        let output = compile_test_pedal_and_process("bbd_chorus.pedal", &input, SAMPLE_RATE, &[]);
        let thd_value = thd(&output, SAMPLE_RATE, 440.0);
        println!("2. THD at nominal level: {:.2}%", thd_value * 100.0);
        if thd_value > 0.1 {
            issues.push(format!("High THD: {:.1}%", thd_value * 100.0));
        }
    }

    // Test 3: DC offset
    {
        let input = sine(440.0, 1.0, SAMPLE_RATE);
        let output = compile_test_pedal_and_process("bbd_chorus.pedal", &input, SAMPLE_RATE, &[]);
        let dc = dc_offset(&output).abs();
        println!("3. DC offset: {:.6}", dc);
        if dc > 0.01 {
            issues.push(format!("DC offset: {:.4}", dc));
        }
    }

    // Test 4: Output stability
    {
        let input = sine(440.0, 2.0, SAMPLE_RATE);
        let output = compile_test_pedal_and_process("bbd_chorus.pedal", &input, SAMPLE_RATE, &[]);
        let all_finite = output.iter().all(|x| x.is_finite());
        let max_peak = peak(&output);
        println!("4. Stability: finite={}, peak={:.4}", all_finite, max_peak);
        if !all_finite {
            issues.push("Output contains NaN/inf".to_string());
        }
        if max_peak > 2.0 {
            issues.push(format!("Excessive peak: {:.2}", max_peak));
        }
    }

    // Report
    println!("\n=== Assessment Result ===");
    if issues.is_empty() {
        println!("PASS: No significant artifacts detected");
    } else {
        println!("ISSUES DETECTED:");
        for issue in &issues {
            println!("  - {}", issue);
        }
    }
    println!();

    // The test passes even with artifacts - it's for detection, not gating
}

// ===========================================================================
// Walrus Slo Specific Tests
// ===========================================================================

/// Walrus Slo - tests multi-tap BBD behavior.
///
/// The Slo uses cascaded BBDs which can compound artifacts.
#[test]
fn walrus_slo_basic_operation() {
    let input = sine(440.0, 2.0, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("walrus_slo.pedal", &input, SAMPLE_RATE, &[]);

    assert!(
        output.iter().all(|x| x.is_finite()),
        "Walrus Slo: output contains NaN/inf"
    );

    let out_rms = rms(&output);
    let out_peak = peak(&output);

    println!("Walrus Slo: RMS={:.6}, Peak={:.6}", out_rms, out_peak);

    // Should produce signal
    assert!(out_rms > 1e-6, "Walrus Slo: no signal output");

    maybe_dump_wav(&output, "walrus_slo_basic", SAMPLE_RATE_U32);
}

/// Walrus Slo - artifact accumulation through cascaded BBDs.
///
/// Multi-tap delays compound artifacts with each stage.
#[test]
fn walrus_slo_cascade_artifacts() {
    // Burst then silence to hear cascaded repeats
    let mut input = sine(330.0, 0.2, SAMPLE_RATE);
    input.extend(vec![0.0; (SAMPLE_RATE * 3.0) as usize]);

    let output = compile_test_pedal_and_process("walrus_slo.pedal", &input, SAMPLE_RATE, &[]);

    assert!(
        output.iter().all(|x| x.is_finite()),
        "Output should be finite"
    );

    // Analyze decay characteristics
    let chunk_size = (SAMPLE_RATE * 0.5) as usize;
    let mut rms_values = Vec::new();
    let mut thd_values = Vec::new();

    for i in 0..6 {
        let start = i * chunk_size;
        let end = (start + chunk_size).min(output.len());
        if end > start + 1000 {
            let chunk = &output[start..end];
            let chunk_rms = rms(chunk);
            let chunk_thd = thd(chunk, SAMPLE_RATE, 330.0);
            rms_values.push(chunk_rms);
            thd_values.push(chunk_thd);
            println!(
                "Slo chunk {}: RMS={:.6}, THD={:.4}",
                i, chunk_rms, chunk_thd
            );
        }
    }

    // ARTIFACT DETECTION: THD explosion in later repeats
    if thd_values.len() >= 3 {
        let early_thd = thd_values[0];
        let late_thd = thd_values.iter().skip(2).cloned().fold(0.0f64, f64::max);
        if late_thd > early_thd * 5.0 && early_thd > 0.01 {
            println!(
                "WARNING: Slo THD accumulation: {:.1}x increase through cascade",
                late_thd / early_thd
            );
        }
    }

    maybe_dump_wav(&output, "walrus_slo_cascade", SAMPLE_RATE_U32);
}

/// Walrus Slo - modulation interaction artifacts.
///
/// Multiple LFOs can create intermodulation and beating patterns.
#[test]
fn walrus_slo_lfo_interaction() {
    let input = sine(440.0, 5.0, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("walrus_slo.pedal", &input, SAMPLE_RATE, &[]);

    assert_healthy(&output, "Slo LFO interaction", 5.0);

    // Check for unexpected low-frequency modulation (LFO beating)
    let mod_rate = detect_modulation_rate(&output, SAMPLE_RATE);
    println!("Slo detected modulation rate: {:.2} Hz", mod_rate);

    // Check amplitude modulation depth
    let has_mod = has_amplitude_modulation(&output, 100);
    println!("Slo has amplitude modulation: {}", has_mod);

    // Analyze modulation spectrum for artifacts
    // Multiple LFOs should create sum/difference frequencies
    let lf_energy = band_energy_ratio(&output, SAMPLE_RATE, 0.5, 10.0, 0.5);
    println!("Slo LF modulation energy ratio: {:.4}", lf_energy);

    maybe_dump_wav(&output, "walrus_slo_lfo", SAMPLE_RATE_U32);
}

/// Walrus Slo - feedback oscillation check.
///
/// High feedback can cause self-oscillation or instability.
#[test]
fn walrus_slo_feedback_stability() {
    // Short burst then long silence to test feedback behavior
    let mut input = vec![0.3; 480]; // 10ms impulse
    input.extend(vec![0.0; (SAMPLE_RATE * 4.0) as usize]);

    let output = compile_test_pedal_and_process("walrus_slo.pedal", &input, SAMPLE_RATE, &[]);

    // Check for runaway oscillation
    let all_finite = output.iter().all(|x| x.is_finite());
    let max_peak = peak(&output);

    println!("Slo feedback test: finite={}, peak={:.4}", all_finite, max_peak);

    assert!(all_finite, "Slo: feedback caused NaN/inf");
    assert!(max_peak < 5.0, "Slo: feedback oscillation (peak={:.2})", max_peak);

    // Check for growing oscillation
    let early_rms = rms(&output[..48000.min(output.len())]);
    let late_rms = rms(&output[output.len().saturating_sub(48000)..]);

    println!("Slo feedback decay: early={:.6}, late={:.6}", early_rms, late_rms);

    // Signal should decay, not grow
    if late_rms > early_rms * 2.0 && late_rms > 0.01 {
        println!("WARNING: Slo feedback may be unstable (signal growing)");
    }

    maybe_dump_wav(&output, "walrus_slo_feedback", SAMPLE_RATE_U32);
}

// ===========================================================================
// EHX Memory Man Specific Tests
// ===========================================================================

/// Memory Man - basic operation test.
#[test]
fn memory_man_basic_operation() {
    let input = sine(440.0, 2.0, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("ehx_memory_man.pedal", &input, SAMPLE_RATE, &[]);

    assert!(
        output.iter().all(|x| x.is_finite()),
        "Memory Man: output contains NaN/inf"
    );

    let out_rms = rms(&output);
    let out_peak = peak(&output);

    println!("Memory Man: RMS={:.6}, Peak={:.6}", out_rms, out_peak);

    assert!(out_rms > 1e-6, "Memory Man: no signal output");

    maybe_dump_wav(&output, "memory_man_basic", SAMPLE_RATE_U32);
}

/// Memory Man - characteristic "dark" sound.
///
/// MN3005's 4096 stages should progressively roll off high frequencies.
#[test]
fn memory_man_dark_character() {
    // Bright input with harmonics
    let input = guitar_pluck(220.0, 1.0, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("ehx_memory_man.pedal", &input, SAMPLE_RATE, &[]);

    assert_healthy(&output, "Memory Man dark test", 5.0);

    let input_centroid = spectral_centroid(&input, SAMPLE_RATE);
    let output_centroid = spectral_centroid(&output, SAMPLE_RATE);

    println!(
        "Memory Man centroid: input={:.0}Hz, output={:.0}Hz",
        input_centroid, output_centroid
    );

    // Memory Man should darken the sound
    if output_centroid > input_centroid * 1.1 {
        println!(
            "NOTE: Output brighter than expected for Memory Man character"
        );
    }

    // Check HF rolloff
    let input_hf = band_energy_ratio(&input, SAMPLE_RATE, 3000.0, 10000.0, 500.0);
    let output_hf = band_energy_ratio(&output, SAMPLE_RATE, 3000.0, 10000.0, 500.0);

    println!(
        "Memory Man HF energy: input={:.4}, output={:.4}",
        input_hf, output_hf
    );

    if output_hf > input_hf && input_hf > 0.01 {
        println!("NOTE: HF rolloff less than expected for MN3005");
    }

    maybe_dump_wav(&output, "memory_man_dark", SAMPLE_RATE_U32);
}

/// Memory Man - delay time stability.
///
/// The Memory Man is known for stable, predictable delay times.
#[test]
fn memory_man_delay_consistency() {
    // Short burst followed by silence - analyze delay timing
    let mut input = Vec::new();

    // Click burst
    for i in 0..240 {
        let t = i as f64 / SAMPLE_RATE;
        input.push(0.5 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin());
    }

    // Silence to capture repeats
    input.extend(vec![0.0; (SAMPLE_RATE * 2.0) as usize]);

    let output = compile_test_pedal_and_process("ehx_memory_man.pedal", &input, SAMPLE_RATE, &[]);

    assert!(
        output.iter().all(|x| x.is_finite()),
        "Output should be finite"
    );

    // Find peaks (delay repeats)
    let mut peaks = Vec::new();
    for i in 1..output.len() - 1 {
        if output[i].abs() > output[i - 1].abs()
            && output[i].abs() > output[i + 1].abs()
            && output[i].abs() > 0.01
        {
            peaks.push(i);
        }
    }

    println!("Memory Man found {} peaks (potential repeats)", peaks.len());

    // Check delay consistency between repeats
    if peaks.len() >= 3 {
        let mut intervals: Vec<usize> = Vec::new();
        for i in 1..peaks.len() {
            intervals.push(peaks[i] - peaks[i - 1]);
        }

        let avg_interval = intervals.iter().sum::<usize>() as f64 / intervals.len() as f64;
        let interval_variance = intervals
            .iter()
            .map(|&i| (i as f64 - avg_interval).powi(2))
            .sum::<f64>()
            / intervals.len() as f64;

        let interval_std = interval_variance.sqrt();
        let delay_ms = avg_interval / SAMPLE_RATE * 1000.0;

        println!(
            "Memory Man delay: {:.1}ms avg, {:.1} samples std",
            delay_ms, interval_std
        );

        // ARTIFACT DETECTION: High variance indicates timing instability
        if interval_std > avg_interval * 0.1 {
            println!(
                "WARNING: Memory Man delay timing unstable (std={:.1} vs avg={:.0})",
                interval_std, avg_interval
            );
        }
    }

    maybe_dump_wav(&output, "memory_man_delay", SAMPLE_RATE_U32);
}

/// Memory Man - feedback pump/breathing.
///
/// Compander artifacts in the feedback path accumulate.
#[test]
fn memory_man_feedback_breathing() {
    // Burst with silence to hear feedback repeats
    let mut input = guitar_pluck(330.0, 0.3, SAMPLE_RATE);
    input.extend(vec![0.0; (SAMPLE_RATE * 3.0) as usize]);

    let output = compile_test_pedal_and_process("ehx_memory_man.pedal", &input, SAMPLE_RATE, &[]);

    assert!(
        output.iter().all(|x| x.is_finite()),
        "Output should be finite"
    );

    // Analyze envelope smoothness of the decay
    let chunk_size = (SAMPLE_RATE * 0.2) as usize;
    let mut envelope: Vec<f64> = Vec::new();

    for i in 0..15 {
        let start = i * chunk_size;
        let end = (start + chunk_size).min(output.len());
        if end > start {
            envelope.push(rms(&output[start..end]));
        }
    }

    // Calculate smoothness (should be monotonic decay with repeats)
    let mut jitter_score = 0.0;
    for i in 2..envelope.len() {
        // Compare local trend to expected decay
        let expected_decay = (envelope[i - 2] + envelope[i - 1]) / 2.0;
        let actual = envelope[i];
        if expected_decay > 1e-6 {
            let deviation = (actual - expected_decay * 0.9).abs() / expected_decay;
            jitter_score += deviation;
        }
    }

    println!(
        "Memory Man envelope jitter score: {:.4}",
        jitter_score / envelope.len() as f64
    );

    println!("Memory Man envelope: {:?}", envelope.iter().map(|x| format!("{:.4}", x)).collect::<Vec<_>>());

    maybe_dump_wav(&output, "memory_man_breathing", SAMPLE_RATE_U32);
}

/// Memory Man - chorus mode artifacts.
///
/// The Memory Man's chorus mode modulates delay time rapidly.
#[test]
fn memory_man_chorus_mode() {
    let input = sine(440.0, 3.0, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("ehx_memory_man.pedal", &input, SAMPLE_RATE, &[]);

    assert_healthy(&output, "Memory Man chorus", 5.0);

    // Check for chorus sidebands
    let fund_power = goertzel_power(&output, SAMPLE_RATE, 440.0);
    let sideband_lo = goertzel_power(&output, SAMPLE_RATE, 435.0)
        + goertzel_power(&output, SAMPLE_RATE, 430.0);
    let sideband_hi = goertzel_power(&output, SAMPLE_RATE, 445.0)
        + goertzel_power(&output, SAMPLE_RATE, 450.0);

    let total_sideband = sideband_lo + sideband_hi;

    println!(
        "Memory Man chorus: fund={:.2e}, sidebands={:.2e}",
        fund_power, total_sideband
    );

    // Check modulation rate
    let mod_rate = detect_modulation_rate(&output, SAMPLE_RATE);
    println!("Memory Man detected mod rate: {:.2} Hz", mod_rate);

    // Look for unwanted artifacts (clock noise, zipper)
    let hf_noise = band_energy_ratio(&output, SAMPLE_RATE, 8000.0, 16000.0, 1000.0);
    println!("Memory Man HF noise ratio: {:.4}", hf_noise);

    if hf_noise > 0.05 {
        println!(
            "WARNING: Memory Man chorus HF noise: {:.1}%",
            hf_noise * 100.0
        );
    }

    maybe_dump_wav(&output, "memory_man_chorus", SAMPLE_RATE_U32);
}

/// Memory Man - SNR at quiet levels.
///
/// Memory Man is known for relatively high noise floor.
#[test]
fn memory_man_snr() {
    // Test at different input levels
    let levels = [0.5, 0.1, 0.02];

    println!("\nMemory Man SNR analysis:");

    for &level in &levels {
        let input = sine_at(440.0, 1.0, level, SAMPLE_RATE);
        let output = compile_test_pedal_and_process("ehx_memory_man.pedal", &input, SAMPLE_RATE, &[]);

        let signal_power = goertzel_power(&output, SAMPLE_RATE, 440.0);
        let total_power: f64 = output.iter().map(|x| x * x).sum::<f64>() / output.len() as f64;
        let noise_power = (total_power - signal_power).max(1e-20);

        let snr_db = 10.0 * (signal_power / noise_power).log10();

        println!(
            "  Input level {:.2}: SNR = {:.1} dB",
            level, snr_db
        );

        // ARTIFACT DETECTION: SNR should be reasonable (>30dB for analog)
        if snr_db < 30.0 && level > 0.1 {
            println!(
                "  WARNING: Poor SNR at level {:.2}: {:.1} dB (want >30 dB)",
                level, snr_db
            );
        }
    }
}

// ===========================================================================
// Combined Stress Test
// ===========================================================================

/// Stress test: Complex musical signal through both pedals.
#[test]
fn bbd_stress_test_musical_content() {
    // Complex signal: multiple frequencies, dynamics
    let n = (SAMPLE_RATE * 3.0) as usize;
    let input: Vec<f64> = (0..n)
        .map(|i| {
            let t = i as f64 / SAMPLE_RATE;
            let env = if t < 0.1 {
                t * 10.0  // Attack
            } else {
                (-0.5 * (t - 0.1)).exp()  // Decay
            };

            // Guitar-like with chord
            let e = 82.41;  // E2
            let b = 123.47; // B2
            let g = 196.0;  // G3

            env * 0.3 * (
                (2.0 * std::f64::consts::PI * e * t).sin() +
                0.7 * (2.0 * std::f64::consts::PI * b * t).sin() +
                0.5 * (2.0 * std::f64::consts::PI * g * t).sin() +
                0.3 * (2.0 * std::f64::consts::PI * e * 2.0 * t).sin()
            )
        })
        .collect();

    // Process through both
    let slo_out = compile_test_pedal_and_process("walrus_slo.pedal", &input, SAMPLE_RATE, &[]);
    let mm_out = compile_test_pedal_and_process("ehx_memory_man.pedal", &input, SAMPLE_RATE, &[]);

    println!("\n=== BBD Stress Test ===");
    println!("Input: RMS={:.6}, Peak={:.6}", rms(&input), peak(&input));
    println!("Slo: RMS={:.6}, Peak={:.6}, finite={}",
        rms(&slo_out), peak(&slo_out), slo_out.iter().all(|x| x.is_finite()));
    println!("Memory Man: RMS={:.6}, Peak={:.6}, finite={}",
        rms(&mm_out), peak(&mm_out), mm_out.iter().all(|x| x.is_finite()));

    // Both should handle complex content
    assert!(slo_out.iter().all(|x| x.is_finite()), "Slo stress: NaN/inf");
    assert!(mm_out.iter().all(|x| x.is_finite()), "MM stress: NaN/inf");

    maybe_dump_wav(&input, "bbd_stress_input", SAMPLE_RATE_U32);
    maybe_dump_wav(&slo_out, "bbd_stress_slo", SAMPLE_RATE_U32);
    maybe_dump_wav(&mm_out, "bbd_stress_mm", SAMPLE_RATE_U32);
}
