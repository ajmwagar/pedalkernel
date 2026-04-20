//! 808 drum voice tests — bridged-T resonators at different tunings.
//! Each voice is the same circuit topology with different R/C values.
//! Verifies frequency, generates WAV files for listening.

use pedalkernel::PedalProcessor;

const SAMPLE_RATE: f64 = 48000.0;

/// 808 voice definition: name, R (series), C (shunt), R_fb, target frequency.
struct Voice {
    name: &'static str,
    note: &'static str,
    r: &'static str,      // R1=R2 value
    c: &'static str,      // C1=C2 value
    r_fb: &'static str,   // Feedback resistor (sets Q/decay)
    target_hz: f64,
    freq_tolerance: f64,  // ratio tolerance (0.2 = ±20%)
}

// Each voice has R_fb tuned for the right decay at its R value.
// Q = Rf / (Rf - 3R). The real 808 uses different physical resistors per voice.
// Toms: Q≈15-20 (sustained ring). Congas: Q≈8-10 (snappy, shorter).
// Rimshot: Q≈5 (very short). Claves: Q≈20 (clean ring).
const VOICES: &[Voice] = &[
    Voice { name: "Bass Drum",  note: "C3",  r: "150k", c: "8.2n", r_fb: "470k", target_hz: 130.0, freq_tolerance: 0.10 },
    Voice { name: "Low Tom",    note: "E3",  r: "120k", c: "8.2n", r_fb: "390k", target_hz: 162.0, freq_tolerance: 0.10 },
    Voice { name: "Mid Tom",    note: "A3",  r: "150k", c: "4.7n", r_fb: "470k", target_hz: 226.0, freq_tolerance: 0.10 },
    Voice { name: "High Tom",   note: "D4",  r: "120k", c: "4.7n", r_fb: "390k", target_hz: 282.0, freq_tolerance: 0.10 },
    Voice { name: "Low Conga",  note: "F3",  r: "120k", c: "8.2n", r_fb: "380k", target_hz: 162.0, freq_tolerance: 0.10 },
    Voice { name: "Mid Conga",  note: "A3",  r: "150k", c: "4.7n", r_fb: "470k", target_hz: 226.0, freq_tolerance: 0.10 },
    Voice { name: "High Conga", note: "D4",  r: "120k", c: "4.7n", r_fb: "380k", target_hz: 282.0, freq_tolerance: 0.10 },
    Voice { name: "Rimshot",    note: "A4",  r: "82k",  c: "4.7n", r_fb: "270k", target_hz: 413.0, freq_tolerance: 0.15 },
    Voice { name: "Claves",     note: "D#7", r: "150k", c: "470p", r_fb: "470k", target_hz: 2258.0, freq_tolerance: 0.10 },
];

/// Generate a .pedal source string for a bridged-T voice.
fn make_pedal_source(voice: &Voice) -> String {
    format!(
        r#"pedal "808 {name}" subtitle "{note}" {{
  supply 9V
  components {{
    U1: opamp(tl072)
    Rb1: resistor(100k)
    Rb2: resistor(100k)
    R1: resistor({r})
    R2: resistor({r})
    C1: cap({c})
    C2: cap({c})
    R_fb: resistor({r_fb})
    R_trig: resistor(100k)
    R_out: resistor(10k)
  }}
  nets {{
    vcc -> Rb1.a
    Rb1.b -> Rb2.a, U1.pos
    Rb2.b -> gnd
    U1.neg -> R1.a, C1.a
    R1.b -> R2.a, C2.a
    R2.b -> U1.out
    C1.b -> gnd
    C2.b -> gnd
    U1.neg -> R_fb.a
    R_fb.b -> U1.out
    in -> R_trig.a
    R_trig.b -> R1.b
    U1.out -> R_out.a
    R_out.b -> out
  }}
  controls {{
  }}
}}"#,
        name = voice.name,
        note = voice.note,
        r = voice.r,
        c = voice.c,
        r_fb = voice.r_fb,
    )
}

/// Compile a voice, fire an impulse, measure frequency, return (freq, peak, output).
fn test_voice(voice: &Voice) -> (f64, f64, Vec<f64>) {
    let source = make_pedal_source(voice);
    let pedal_def = pedalkernel::dsl::parse_pedal_file(&source)
        .unwrap_or_else(|e| panic!("{}: parse failed: {e}", voice.name));
    let mut proc = pedalkernel::compiler::compile_pedal(&pedal_def, SAMPLE_RATE)
        .unwrap_or_else(|e| panic!("{}: compile failed: {e}", voice.name));

    // Warmup
    for _ in 0..480 {
        proc.process(0.0);
    }

    // Process 1 second with impulse at sample 0
    let n_samples = SAMPLE_RATE as usize;
    let mut output = Vec::with_capacity(n_samples);
    for i in 0..n_samples {
        let input = if i == 0 { 1.0 } else { 0.0 };
        output.push(proc.process(input));
    }

    let peak = output.iter().map(|s| s.abs()).fold(0.0_f64, f64::max);

    // Count zero crossings in first 500ms
    let analysis = (SAMPLE_RATE * 0.5) as usize;
    let zc: u32 = (1..analysis.min(output.len()))
        .filter(|&i| output[i] * output[i - 1] < 0.0)
        .count() as u32;
    let freq = zc as f64 / 2.0 / 0.5;

    (freq, peak, output)
}

/// Write a mono WAV file.
fn write_wav(path: &std::path::Path, samples: &[f64], peak: f64) {
    let spec = hound::WavSpec {
        channels: 1,
        sample_rate: SAMPLE_RATE as u32,
        bits_per_sample: 32,
        sample_format: hound::SampleFormat::Float,
    };
    let mut writer = hound::WavWriter::create(path, spec).expect("create WAV");
    let normalize = if peak > 1e-10 { 0.707 / peak } else { 1.0 };
    for &s in samples {
        writer.write_sample((s * normalize) as f32).expect("write");
    }
    writer.finalize().expect("finalize");
}

#[test]
fn all_808_voices_produce_correct_frequency() {
    let wav_dir = std::env::temp_dir().join("808_voices");
    std::fs::create_dir_all(&wav_dir).ok();

    println!("\n{:>14} {:>5} {:>8} {:>8} {:>6} {}",
        "Voice", "Note", "Target", "Actual", "Peak", "Status");
    println!("{}", "-".repeat(60));

    let mut all_pass = true;
    let mut all_outputs: Vec<(&str, Vec<f64>, f64)> = Vec::new();

    for voice in VOICES {
        let (freq, peak, output) = test_voice(voice);
        let ratio = if voice.target_hz > 0.0 { freq / voice.target_hz } else { 0.0 };
        let freq_ok = (ratio - 1.0).abs() < voice.freq_tolerance;
        let peak_ok = peak > 0.0001;
        let pass = freq_ok && peak_ok;

        let status = if pass { "✓" } else { "✗" };
        println!(
            "{:>14} {:>5} {:>7.0}Hz {:>7.0}Hz {:>6.3} {}  (ratio {:.2})",
            voice.name, voice.note, voice.target_hz, freq, peak, status, ratio
        );

        // Write individual WAV
        let wav_path = wav_dir.join(format!("808_{}.wav",
            voice.name.to_lowercase().replace(' ', "_")));
        write_wav(&wav_path, &output, peak);

        all_outputs.push((voice.name, output, peak));

        if !pass {
            all_pass = false;
        }
    }

    // Write combined WAV: all voices in sequence with 0.3s gaps
    let gap = (SAMPLE_RATE * 0.3) as usize;
    let mut combined: Vec<f64> = Vec::new();
    let mut combined_peak = 0.0_f64;
    for (_, output, peak) in &all_outputs {
        combined.extend_from_slice(output);
        combined.extend(std::iter::repeat(0.0).take(gap));
        combined_peak = combined_peak.max(*peak);
    }
    let combined_path = wav_dir.join("808_all_voices.wav");
    write_wav(&combined_path, &combined, combined_peak);
    println!("\nCombined WAV: {}", combined_path.display());
    println!("Individual WAVs in: {}", wav_dir.display());

    assert!(all_pass, "Some 808 voices failed frequency check");
}

/// Generate a pedal source with a Decay pot (R_fb as pot) and optional Tuning pot.
fn make_pedal_with_controls(r: &str, c: &str, r_fb_max: &str, has_tuning: bool) -> String {
    let tuning_comp = if has_tuning {
        "    Tuning: pot(100k, linear)\n"
    } else {
        ""
    };
    let tuning_net = if has_tuning {
        "    U1.neg -> Tuning.a\n    Tuning.b -> R1.a, C1.a"
    } else {
        "    U1.neg -> R1.a, C1.a"
    };
    let tuning_ctrl = if has_tuning {
        "    Tuning.position -> \"Tuning\"\n"
    } else {
        ""
    };
    format!(
        r#"pedal "808 Test Voice" {{
  supply 9V
  components {{
    U1: opamp(tl072)
    Rb1: resistor(100k)
    Rb2: resistor(100k)
    R1: resistor({r})
    R2: resistor({r})
    C1: cap({c})
    C2: cap({c})
    Decay: pot({r_fb_max}, linear)
    R_trig: resistor(100k)
    R_out: resistor(10k)
{tuning_comp}  }}
  nets {{
    vcc -> Rb1.a
    Rb1.b -> Rb2.a, U1.pos
    Rb2.b -> gnd
{tuning_net}
    R1.b -> R2.a, C2.a
    R2.b -> U1.out
    C1.b -> gnd
    C2.b -> gnd
    U1.neg -> Decay.a
    Decay.b -> U1.out
    in -> R_trig.a
    R_trig.b -> R1.b
    U1.out -> R_out.a
    R_out.b -> out
  }}
  controls {{
    Decay.position -> "Decay"
{tuning_ctrl}  }}
}}"#
    )
}

/// Measure frequency and decay time from an impulse response.
fn measure_response(output: &[f64], sample_rate: f64) -> (f64, f64, f64) {
    let peak = output.iter().map(|s| s.abs()).fold(0.0_f64, f64::max);

    // Frequency from zero crossings in first 200ms
    let analysis = (sample_rate * 0.2) as usize;
    let zc: u32 = (1..analysis.min(output.len()))
        .filter(|&i| output[i] * output[i - 1] < 0.0)
        .count() as u32;
    let freq = zc as f64 / 2.0 / 0.2;

    // Decay time: find when amplitude drops to -20dB (0.1×) of peak
    let threshold = peak * 0.1;
    let mut decay_ms = 0.0;
    for i in 0..output.len() {
        if i > 10 && output[i].abs() < threshold {
            // Check it stays below threshold
            let still_below = output[i..].iter().take(100).all(|s| s.abs() < threshold * 2.0);
            if still_below {
                decay_ms = i as f64 / sample_rate * 1000.0;
                break;
            }
        }
    }

    (freq, peak, decay_ms)
}

/// Test: sweeping the Decay pot changes the ring time but NOT the frequency.
#[test]
#[ignore] // TODO: IIR recomputation on pot change
fn decay_pot_changes_ring_time() {
    let source = make_pedal_with_controls("150k", "8.2n", "470k", false);
    let pedal_def = pedalkernel::dsl::parse_pedal_file(&source).expect("parse");

    let mut results: Vec<(f64, f64, f64, f64)> = Vec::new(); // (pot_pos, freq, peak, decay_ms)

    for &pot_pos in &[0.95, 0.8, 0.6, 0.4, 0.2] {
        let mut proc = pedalkernel::compiler::compile_pedal(&pedal_def, SAMPLE_RATE).expect("compile");

        // Set decay pot position
        proc.set_control("Decay", pot_pos);

        // Warmup
        for _ in 0..480 {
            proc.process(0.0);
        }

        // Impulse + 1 second
        let mut output = Vec::with_capacity(SAMPLE_RATE as usize);
        for i in 0..SAMPLE_RATE as usize {
            let input = if i == 0 { 1.0 } else { 0.0 };
            output.push(proc.process(input));
        }

        let (freq, peak, decay_ms) = measure_response(&output, SAMPLE_RATE);
        results.push((pot_pos, freq, peak, decay_ms));
    }

    println!("\n{:>8} {:>8} {:>8} {:>10}", "Decay", "Freq", "Peak", "Decay(ms)");
    println!("{}", "-".repeat(40));
    for &(pos, freq, peak, decay) in &results {
        println!("{:>8.2} {:>7.0}Hz {:>8.3} {:>9.1}ms", pos, freq, peak, decay);
    }

    // Verify: frequency should stay roughly constant across decay settings
    let freqs: Vec<f64> = results.iter().map(|r| r.1).filter(|&f| f > 10.0).collect();
    if freqs.len() >= 2 {
        let f_min = freqs.iter().copied().fold(f64::MAX, f64::min);
        let f_max = freqs.iter().copied().fold(0.0_f64, f64::max);
        let f_ratio = f_max / f_min.max(1.0);
        assert!(
            f_ratio < 1.3,
            "Frequency should be stable across decay settings: min={f_min:.0}, max={f_max:.0}, ratio={f_ratio:.2}"
        );
    }

    // Verify: higher decay pot = longer ring (higher R_fb = higher Q)
    // pot_pos=0.95 should have longest decay, pot_pos=0.2 shortest
    let decays: Vec<f64> = results.iter().map(|r| r.3).collect();
    if decays.len() >= 2 && decays[0] > 0.0 && decays[decays.len() - 1] > 0.0 {
        assert!(
            decays[0] > decays[decays.len() - 1] * 0.5,
            "Higher decay pot should give longer ring: pos=0.95→{:.1}ms, pos=0.2→{:.1}ms",
            decays[0], decays[decays.len() - 1]
        );
    }
}

/// Test: sweeping Tuning pot changes frequency monotonically.
#[test]
#[ignore] // TODO: IIR recomputation on pot change
fn tuning_pot_sweeps_frequency() {
    let source = make_pedal_with_controls("150k", "8.2n", "470k", true);
    let pedal_def = pedalkernel::dsl::parse_pedal_file(&source).expect("parse");

    let mut results: Vec<(f64, f64, f64)> = Vec::new(); // (pot_pos, freq, peak)

    for &pot_pos in &[0.1, 0.3, 0.5, 0.7, 0.9] {
        let mut proc = pedalkernel::compiler::compile_pedal(&pedal_def, SAMPLE_RATE).expect("compile");

        // Set tuning pot — adds series resistance, should lower f0
        proc.set_control("Tuning", pot_pos);
        // Keep decay high for audible ring
        proc.set_control("Decay", 0.9);

        for _ in 0..480 {
            proc.process(0.0);
        }

        let mut output = Vec::with_capacity(SAMPLE_RATE as usize);
        for i in 0..SAMPLE_RATE as usize {
            let input = if i == 0 { 1.0 } else { 0.0 };
            output.push(proc.process(input));
        }

        let (freq, peak, _) = measure_response(&output, SAMPLE_RATE);
        results.push((pot_pos, freq, peak));
    }

    println!("\n{:>8} {:>8} {:>8}", "Tuning", "Freq", "Peak");
    println!("{}", "-".repeat(28));
    for &(pos, freq, peak) in &results {
        println!("{:>8.2} {:>7.0}Hz {:>8.3}", pos, freq, peak);
    }

    // Verify: frequency should change with tuning
    let freqs: Vec<f64> = results.iter().map(|r| r.1).filter(|&f| f > 10.0).collect();
    if freqs.len() >= 2 {
        let f_range = freqs.iter().copied().fold(0.0_f64, f64::max)
            - freqs.iter().copied().fold(f64::MAX, f64::min);
        assert!(
            f_range > 10.0,
            "Tuning pot should change frequency: range={f_range:.0}Hz (frequencies: {freqs:?})"
        );
    }

    // Verify all voices produced output
    for &(pos, _, peak) in &results {
        assert!(
            peak > 0.0001,
            "Tuning={pos:.1} should produce output, got peak={peak:.6}"
        );
    }
}
