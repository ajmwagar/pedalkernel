//! Fuzz Face-style distortion → WAV output.
//!
//! Renders a guitar pluck through the compiled Fuzz Face at
//! several fuzz settings.
//!
//! Run: `cargo run --example fuzzface`

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::wav::{guitar_pluck, render_to_wav, write_stereo_wav};
use pedalkernel::PedalProcessor;
use std::path::Path;

fn main() {
    let sample_rate = 48_000;
    let duration = 2.0;
    let freq = 196.0; // G3 — classic fuzz tone

    println!("PedalKernel — Fuzz Face Example");
    println!("================================");
    println!("Sample rate: {} Hz", sample_rate);
    println!("Duration:    {} s", duration);
    println!("Input:       guitar pluck at {:.1} Hz (G3)", freq);
    println!();

    let src = std::fs::read_to_string("examples/fuzz_face.pedal")
        .expect("failed to read fuzz_face.pedal");
    let pedal_def = parse_pedal_file(&src).expect("failed to parse fuzz_face.pedal");
    let input = guitar_pluck(freq, duration, sample_rate);

    // Low fuzz
    let mut proc = compile_pedal(&pedal_def, sample_rate as f64).unwrap();
    proc.set_control("Fuzz", 0.3);
    proc.set_control("Volume", 0.7);
    let path = Path::new("fuzz_low.wav");
    render_to_wav(&mut proc, &input, path, sample_rate).unwrap();
    println!("Wrote low fuzz (0.3)  → {}", path.display());

    // Full fuzz
    let mut proc = compile_pedal(&pedal_def, sample_rate as f64).unwrap();
    proc.set_control("Fuzz", 1.0);
    proc.set_control("Volume", 0.6);
    let path = Path::new("fuzz_full.wav");
    render_to_wav(&mut proc, &input, path, sample_rate).unwrap();
    println!("Wrote full fuzz (1.0) → {}", path.display());

    // A/B comparison
    let mut proc = compile_pedal(&pedal_def, sample_rate as f64).unwrap();
    proc.set_control("Fuzz", 0.8);
    proc.set_control("Volume", 0.7);
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    let path = Path::new("fuzz_ab.wav");
    write_stereo_wav(&input, &output, path, sample_rate).unwrap();
    println!(
        "Wrote A/B stereo (L=clean, R=fuzz 0.8) → {}",
        path.display()
    );

    println!();
    println!("Done!");
}
