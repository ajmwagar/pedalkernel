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

const VOICES: &[Voice] = &[
    Voice { name: "Bass Drum",  note: "C3",  r: "150k", c: "8.2n", r_fb: "470k", target_hz: 130.0, freq_tolerance: 0.15 },
    Voice { name: "Low Tom",    note: "E3",  r: "120k", c: "8.2n", r_fb: "470k", target_hz: 162.0, freq_tolerance: 0.15 },
    Voice { name: "Mid Tom",    note: "A3",  r: "150k", c: "4.7n", r_fb: "470k", target_hz: 226.0, freq_tolerance: 0.15 },
    Voice { name: "High Tom",   note: "D4",  r: "120k", c: "4.7n", r_fb: "470k", target_hz: 282.0, freq_tolerance: 0.15 },
    Voice { name: "Low Conga",  note: "F3",  r: "120k", c: "8.2n", r_fb: "330k", target_hz: 162.0, freq_tolerance: 0.15 },
    Voice { name: "Mid Conga",  note: "A3",  r: "150k", c: "4.7n", r_fb: "330k", target_hz: 226.0, freq_tolerance: 0.15 },
    Voice { name: "High Conga", note: "D4",  r: "120k", c: "4.7n", r_fb: "330k", target_hz: 282.0, freq_tolerance: 0.15 },
    Voice { name: "Rimshot",    note: "A4",  r: "82k",  c: "4.7n", r_fb: "330k", target_hz: 413.0, freq_tolerance: 0.35 },
    Voice { name: "Claves",     note: "D#7", r: "150k", c: "470p", r_fb: "470k", target_hz: 2258.0, freq_tolerance: 0.90 },
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
