//! Offline WAV processor — run a `.pedal` file on an input WAV.
//!
//! Usage:
//!   cargo run --example process -- <file.pedal> <input.wav> <output.wav> [knob=value ...]
//!
//! Examples:
//!   cargo run --example process -- examples/pedals/overdrive/tube_screamer.pedal in.wav out.wav
//!   cargo run --example process -- examples/pedals/fuzz/fuzz_face.pedal in.wav out.wav Fuzz=0.9 Volume=0.3
//!   cargo run --example process -- examples/pedals/fuzz/big_muff.pedal in.wav out.wav Sustain=1.0
//!   cargo run --example process -- examples/amps/tweed_deluxe_5e3.pedal in.wav out.wav Volume=0.7

use hound::{SampleFormat, WavReader, WavWriter};
use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use std::path::Path;
use std::process;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 4 {
        eprintln!(
            "Usage: {} <file.pedal> <input.wav> <output.wav> [knob=value ...]",
            args[0]
        );
        eprintln!();
        eprintln!("Examples:");
        eprintln!(
            "  {} examples/pedals/overdrive/tube_screamer.pedal input.wav output.wav",
            args[0]
        );
        eprintln!(
            "  {} examples/pedals/fuzz/fuzz_face.pedal input.wav output.wav Fuzz=0.9",
            args[0]
        );
        process::exit(1);
    }

    let pedal_path = &args[1];
    let input_path = &args[2];
    let output_path = &args[3];

    // Parse knob overrides from remaining args (e.g. "Drive=0.8")
    let overrides: Vec<(&str, f64)> = args[4..]
        .iter()
        .filter_map(|a| {
            let (k, v) = a.split_once('=')?;
            Some((k, v.parse::<f64>().ok()?))
        })
        .collect();

    // Parse the .pedal file
    let source = std::fs::read_to_string(pedal_path).unwrap_or_else(|e| {
        eprintln!("Error reading {pedal_path}: {e}");
        process::exit(1);
    });
    let pedal = parse_pedal_file(&source).unwrap_or_else(|e| {
        eprintln!("Parse error: {e}");
        process::exit(1);
    });

    // Read input WAV
    let mut reader = WavReader::open(input_path).unwrap_or_else(|e| {
        eprintln!("Error opening {input_path}: {e}");
        process::exit(1);
    });
    let spec = reader.spec();
    let sample_rate = spec.sample_rate;

    // Read all samples into f64 mono buffer
    let input_samples = read_samples_mono(&mut reader);
    eprintln!(
        "Input: {} ({} samples, {} Hz, {} ch, {}-bit {:?})",
        input_path,
        input_samples.len(),
        sample_rate,
        spec.channels,
        spec.bits_per_sample,
        spec.sample_format,
    );

    // Resolve control values: start from defaults, apply overrides
    let mut knob_values: Vec<(String, f64)> = pedal
        .controls
        .iter()
        .map(|c| (c.label.clone(), c.default))
        .collect();
    for (name, val) in &overrides {
        if let Some(kv) = knob_values
            .iter_mut()
            .find(|(l, _)| l.eq_ignore_ascii_case(name))
        {
            kv.1 = *val;
        } else {
            eprintln!(
                "Warning: unknown knob '{name}', available: {:?}",
                knob_values
                    .iter()
                    .map(|(l, _)| l.as_str())
                    .collect::<Vec<_>>()
            );
        }
    }

    // Compile the pedal's netlist into a WDF processor
    let mut proc = compile_pedal(&pedal, sample_rate as f64).unwrap_or_else(|e| {
        eprintln!("Compilation error: {e}");
        process::exit(1);
    });
    for (label, val) in &knob_values {
        proc.set_control(label, *val);
    }
    let output_samples = process_audio(&mut proc, &input_samples);

    // Write output WAV (same sample rate, mono, 32-bit float)
    let out_spec = hound::WavSpec {
        channels: 1,
        sample_rate,
        bits_per_sample: 32,
        sample_format: SampleFormat::Float,
    };
    let mut writer = WavWriter::create(Path::new(output_path), out_spec).unwrap_or_else(|e| {
        eprintln!("Error creating {output_path}: {e}");
        process::exit(1);
    });
    for &s in &output_samples {
        writer.write_sample(s as f32).unwrap();
    }
    writer.finalize().unwrap();

    // Print summary
    eprintln!("Pedal:  \"{}\"", pedal.name);
    for (label, val) in &knob_values {
        eprintln!("  {label}: {val:.2}");
    }
    eprintln!("Output: {} ({} samples)", output_path, output_samples.len());
}

fn process_audio(proc: &mut dyn PedalProcessor, input: &[f64]) -> Vec<f64> {
    input.iter().map(|&s| proc.process(s)).collect()
}

/// Read all samples from a WAV file into a mono f64 buffer, normalizing to [-1, 1].
fn read_samples_mono(reader: &mut WavReader<std::io::BufReader<std::fs::File>>) -> Vec<f64> {
    let spec = reader.spec();
    let channels = spec.channels as usize;

    match spec.sample_format {
        SampleFormat::Float => {
            let all: Vec<f32> = reader.samples::<f32>().map(|s| s.unwrap()).collect();
            mix_to_mono(&all.iter().map(|&s| s as f64).collect::<Vec<_>>(), channels)
        }
        SampleFormat::Int => {
            let max_val = (1_i64 << (spec.bits_per_sample - 1)) as f64;
            let all: Vec<i32> = reader.samples::<i32>().map(|s| s.unwrap()).collect();
            let normalized: Vec<f64> = all.iter().map(|&s| s as f64 / max_val).collect();
            mix_to_mono(&normalized, channels)
        }
    }
}

/// Mix interleaved multi-channel samples down to mono by averaging channels.
fn mix_to_mono(interleaved: &[f64], channels: usize) -> Vec<f64> {
    if channels == 1 {
        return interleaved.to_vec();
    }
    interleaved
        .chunks(channels)
        .map(|frame| frame.iter().sum::<f64>() / channels as f64)
        .collect()
}
