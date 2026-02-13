use hound::{SampleFormat, WavReader, WavWriter};
use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use std::path::Path;
use std::process;

pub fn run(file_path: &str, input_path: &str, output_path: &str, knob_args: &[String]) {
    if file_path.ends_with(".board") {
        run_board(file_path, input_path, output_path, knob_args);
    } else {
        run_pedal(file_path, input_path, output_path, knob_args);
    }
}

fn run_pedal(pedal_path: &str, input_path: &str, output_path: &str, knob_args: &[String]) {
    // Parse knob overrides (e.g. "Drive=0.8")
    let overrides: Vec<(&str, f64)> = knob_args
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
    let output_samples: Vec<f64> = input_samples.iter().map(|&s| proc.process(s)).collect();

    // Write output WAV (same sample rate, mono, 32-bit float)
    write_wav(output_path, sample_rate, &output_samples);

    eprintln!("Pedal:  \"{}\"", pedal.name);
    for (label, val) in &knob_values {
        eprintln!("  {label}: {val:.2}");
    }
    eprintln!(
        "Output: {} ({} samples)",
        output_path,
        output_samples.len()
    );
}

fn run_board(board_path: &str, input_path: &str, output_path: &str, knob_args: &[String]) {
    use pedalkernel::board::parse_board_file;
    use pedalkernel::pedalboard::PedalboardProcessor;

    // Parse knob overrides for boards: "pedal_id.Knob=value"
    let overrides: Vec<(&str, &str, f64)> = knob_args
        .iter()
        .filter_map(|a| {
            let (lhs, v) = a.split_once('=')?;
            let val = v.parse::<f64>().ok()?;
            if let Some((pedal_id, knob)) = lhs.split_once('.') {
                Some((pedal_id, knob, val))
            } else {
                eprintln!("Warning: board knob override must use 'pedal_id.Knob=value' format, got: {a}");
                None
            }
        })
        .collect();

    // Parse the .board file
    let source = std::fs::read_to_string(board_path).unwrap_or_else(|e| {
        eprintln!("Error reading {board_path}: {e}");
        process::exit(1);
    });
    let board = parse_board_file(&source).unwrap_or_else(|e| {
        eprintln!("Parse error: {e}");
        process::exit(1);
    });

    let board_dir = Path::new(board_path)
        .parent()
        .unwrap_or_else(|| Path::new("."));

    // Read input WAV
    let mut reader = WavReader::open(input_path).unwrap_or_else(|e| {
        eprintln!("Error opening {input_path}: {e}");
        process::exit(1);
    });
    let spec = reader.spec();
    let sample_rate = spec.sample_rate;

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

    // Build pedalboard processor
    let mut proc =
        PedalboardProcessor::from_board(&board, board_dir, sample_rate as f64).unwrap_or_else(
            |e| {
                eprintln!("Board compilation error: {e}");
                process::exit(1);
            },
        );

    // Apply CLI knob overrides using the "idx:label" control routing
    for (pedal_id, knob, val) in &overrides {
        if let Some(idx) = board.pedals.iter().position(|p| p.id == *pedal_id) {
            proc.set_control(&format!("{idx}:{knob}"), *val);
        } else {
            eprintln!(
                "Warning: unknown pedal id '{pedal_id}', available: {:?}",
                board.pedals.iter().map(|p| p.id.as_str()).collect::<Vec<_>>()
            );
        }
    }

    let output_samples: Vec<f64> = input_samples.iter().map(|&s| proc.process(s)).collect();

    // Write output WAV
    write_wav(output_path, sample_rate, &output_samples);

    eprintln!("Board:  \"{}\"", board.name);
    for entry in &board.pedals {
        eprintln!("  [{}] {}", entry.id, entry.path);
    }
    eprintln!(
        "Output: {} ({} samples)",
        output_path,
        output_samples.len()
    );
}

fn write_wav(output_path: &str, sample_rate: u32, samples: &[f64]) {
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
    for &s in samples {
        writer.write_sample(s as f32).unwrap();
    }
    writer.finalize().unwrap();
}

/// Read all samples from a WAV file into a mono f64 buffer, normalizing to [-1, 1].
fn read_samples_mono(reader: &mut WavReader<std::io::BufReader<std::fs::File>>) -> Vec<f64> {
    let spec = reader.spec();
    let channels = spec.channels as usize;

    match spec.sample_format {
        SampleFormat::Float => {
            let all: Vec<f32> = reader.samples::<f32>().map(|s| s.unwrap()).collect();
            mix_to_mono(
                &all.iter().map(|&s| s as f64).collect::<Vec<_>>(),
                channels,
            )
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
