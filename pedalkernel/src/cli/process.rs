use hound::{SampleFormat, WavWriter};
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
    let (input_samples, sample_rate) = pedalkernel::wav::read_wav_mono(Path::new(input_path))
        .unwrap_or_else(|e| {
            eprintln!("Error reading {input_path}: {e}");
            process::exit(1);
        });
    eprintln!(
        "Input: {} ({} samples, {} Hz)",
        input_path,
        input_samples.len(),
        sample_rate,
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
    eprintln!("Output: {} ({} samples)", output_path, output_samples.len());
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
                eprintln!(
                    "Warning: board knob override must use 'pedal_id.Knob=value' format, got: {a}"
                );
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
    let (input_samples, sample_rate) = pedalkernel::wav::read_wav_mono(Path::new(input_path))
        .unwrap_or_else(|e| {
            eprintln!("Error reading {input_path}: {e}");
            process::exit(1);
        });
    eprintln!(
        "Input: {} ({} samples, {} Hz)",
        input_path,
        input_samples.len(),
        sample_rate,
    );

    // Build pedalboard processor
    let mut proc = PedalboardProcessor::from_board(&board, board_dir, sample_rate as f64)
        .unwrap_or_else(|e| {
            eprintln!("Board compilation error: {e}");
            process::exit(1);
        });

    // Apply CLI knob overrides using the "idx:label" control routing
    for (pedal_id, knob, val) in &overrides {
        if let Some(idx) = board.pedals.iter().position(|p| p.id == *pedal_id) {
            proc.set_control(&format!("{idx}:{knob}"), *val);
        } else {
            eprintln!(
                "Warning: unknown pedal id '{pedal_id}', available: {:?}",
                board
                    .pedals
                    .iter()
                    .map(|p| p.id.as_str())
                    .collect::<Vec<_>>()
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
    eprintln!("Output: {} ({} samples)", output_path, output_samples.len());
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
