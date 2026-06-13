//! CLI handler for IR export from `.cab` files.

use std::path::Path;
use std::process;

use pedalkernel::cab::{compile_cab, ir_export, parse_cab_file, IrExportConfig};

pub fn run(cab_path: &str, output_path: &str, sample_rate: u32, length_samples: u32, bit_depth: u16) {
    let source = std::fs::read_to_string(cab_path).unwrap_or_else(|e| {
        eprintln!("Error reading {cab_path}: {e}");
        process::exit(1);
    });

    let cab = parse_cab_file(&source).unwrap_or_else(|e| {
        eprintln!("Parse error: {e}");
        process::exit(1);
    });

    let mut proc = compile_cab(&cab, sample_rate as f64).unwrap_or_else(|e| {
        eprintln!("Compilation error: {e}");
        process::exit(1);
    });

    let length = length_samples as usize;
    let config = IrExportConfig {
        sample_rate,
        length,
        bit_depth,
        normalize: true,
        fade_out: (sample_rate as usize / 24), // ~42ms fade
    };

    let ir = ir_export::capture_ir(&mut proc, &config);
    ir_export::export_ir_wav(&ir, Path::new(output_path), &config).unwrap_or_else(|e| {
        eprintln!("Error writing {output_path}: {e}");
        process::exit(1);
    });

    let peak_db = ir
        .iter()
        .map(|s| s.abs())
        .fold(0.0f64, f64::max)
        .max(1e-12);
    let rms = (ir.iter().map(|s| s * s).sum::<f64>() / ir.len() as f64).sqrt();

    eprintln!("Cabinet: \"{}\"", cab.name);
    eprintln!(
        "IR:      {} ({} samples, {} Hz, {}-bit)",
        output_path, length, sample_rate, bit_depth
    );
    eprintln!(
        "Peak:    {:.1} dB, RMS: {:.1} dB",
        20.0 * peak_db.log10(),
        20.0 * rms.max(1e-12).log10()
    );
}
