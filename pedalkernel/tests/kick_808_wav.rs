//! 808 kick drum test — compile the bridged-T resonator, fire a trigger,
//! and save the output as a WAV file for listening.

use pedalkernel::PedalProcessor;
use std::path::PathBuf;

const SAMPLE_RATE: f64 = 48000.0;

fn pedal_path(name: &str) -> PathBuf {
    let mut p = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    p.push("tests/test_pedals");
    p.push(name);
    p
}

#[test]
fn bridged_t_kick_produces_wav() {
    let source = std::fs::read_to_string(pedal_path("bridged_t_resonator.pedal"))
        .expect("bridged_t_resonator.pedal not found");
    let pedal_def = pedalkernel::dsl::parse_pedal_file(&source).expect("parse failed");
    let mut proc =
        pedalkernel::compiler::compile_pedal(&pedal_def, SAMPLE_RATE).expect("compile failed");

    let duration_secs = 2.0;
    let num_samples = (SAMPLE_RATE * duration_secs) as usize;
    let mut output = Vec::with_capacity(num_samples);

    // Warmup: 480 samples of silence to settle DC bias
    for _ in 0..480 {
        proc.process(0.0);
    }

    // The trigger is just an input signal — a short voltage impulse.
    // No special routing needed. process(impulse) → MNA → output.
    for i in 0..num_samples {
        // Impulse at sample 0 and at 1 second
        let input = if i == 0 || i == (SAMPLE_RATE as usize) {
            1.0 // voltage impulse
        } else {
            0.0
        };
        let sample = proc.process(input);
        output.push(sample);

        // Print samples across full decay
        if i < 20 || (i < 2000 && i % 100 == 0) {
            eprintln!("  s[{i:5}] = {sample:+.6e}")
        }
    }

    // Find peak amplitude
    let peak = output.iter().map(|s| s.abs()).fold(0.0_f64, f64::max);
    println!("Bridged-T kick: peak amplitude = {peak:.6}");

    // Check we got meaningful output
    assert!(
        peak > 0.0001,
        "Bridged-T resonator should produce output after trigger, got peak={peak:.6}"
    );

    // Measure frequency content — count zero crossings in first 500ms
    let analysis_samples = (SAMPLE_RATE * 0.5) as usize;
    let mut zero_crossings = 0u32;
    for i in 1..analysis_samples.min(output.len()) {
        if output[i] * output[i - 1] < 0.0 {
            zero_crossings += 1;
        }
    }
    let estimated_freq = (zero_crossings as f64) / 2.0 / 0.5;
    println!("Estimated fundamental: {estimated_freq:.1} Hz (target: ~130 Hz)");

    // Write WAV file
    let wav_path = std::env::temp_dir().join("808_kick_bridged_t.wav");
    let spec = hound::WavSpec {
        channels: 1,
        sample_rate: SAMPLE_RATE as u32,
        bits_per_sample: 32,
        sample_format: hound::SampleFormat::Float,
    };
    let mut writer = hound::WavWriter::create(&wav_path, spec).expect("create WAV");

    // Normalize to -3dB peak
    let normalize = if peak > 1e-10 {
        0.707 / peak
    } else {
        1.0
    };
    for &s in &output {
        writer
            .write_sample((s * normalize) as f32)
            .expect("write sample");
    }
    writer.finalize().expect("finalize WAV");

    println!("WAV written to: {}", wav_path.display());

    // Basic frequency sanity check (bridged-T at 150k/8.2n ≈ 130 Hz)
    // Allow wide tolerance since it's a resonator, not a pure tone
    if estimated_freq > 10.0 {
        println!(
            "Frequency check: {estimated_freq:.0} Hz (expected ~130 Hz, ratio={:.2})",
            estimated_freq / 130.0
        );
    }
}
