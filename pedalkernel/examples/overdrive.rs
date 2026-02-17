//! Tube Screamer-style overdrive → WAV output.
//!
//! Renders a 440 Hz guitar-like pluck through the compiled Tube Screamer
//! at three gain settings and writes each to a separate WAV file.
//!
//! Run: `cargo run --example overdrive`

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::wav::{guitar_pluck, render_to_wav, write_stereo_wav};
use pedalkernel::PedalProcessor;
use std::path::Path;

fn main() {
    let sample_rate = 48_000;
    let duration = 2.0; // seconds
    let freq = 82.41; // low E string

    println!("PedalKernel — Tube Screamer Overdrive Example");
    println!("==============================================");
    println!("Sample rate: {} Hz", sample_rate);
    println!("Duration:    {} s", duration);
    println!("Input:       guitar pluck at {:.1} Hz (low E)", freq);
    println!();

    let src = std::fs::read_to_string("examples/tube_screamer.pedal")
        .expect("failed to read tube_screamer.pedal");
    let pedal_def = parse_pedal_file(&src).expect("failed to parse tube_screamer.pedal");
    let input = guitar_pluck(freq, duration, sample_rate);

    // Clean signal
    let clean_path = Path::new("overdrive_clean.wav");
    pedalkernel::wav::write_wav(&input, clean_path, sample_rate).unwrap();
    println!("Wrote clean signal → {}", clean_path.display());

    // Low gain
    let mut proc = compile_pedal(&pedal_def, sample_rate as f64).unwrap();
    proc.set_control("Drive", 0.2);
    let path = Path::new("overdrive_low.wav");
    render_to_wav(&mut proc, &input, path, sample_rate).unwrap();
    println!("Wrote low gain (0.2) → {}", path.display());

    // Medium gain
    let mut proc = compile_pedal(&pedal_def, sample_rate as f64).unwrap();
    proc.set_control("Drive", 0.5);
    let path = Path::new("overdrive_mid.wav");
    render_to_wav(&mut proc, &input, path, sample_rate).unwrap();
    println!("Wrote mid gain (0.5) → {}", path.display());

    // High gain
    let mut proc = compile_pedal(&pedal_def, sample_rate as f64).unwrap();
    proc.set_control("Drive", 0.9);
    let path = Path::new("overdrive_high.wav");
    render_to_wav(&mut proc, &input, path, sample_rate).unwrap();
    println!("Wrote high gain (0.9) → {}", path.display());

    // A/B comparison — clean vs high gain in stereo
    let mut proc = compile_pedal(&pedal_def, sample_rate as f64).unwrap();
    proc.set_control("Drive", 0.9);
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    let path = Path::new("overdrive_ab.wav");
    write_stereo_wav(&input, &output, path, sample_rate).unwrap();
    println!(
        "Wrote A/B stereo (L=clean, R=high gain) → {}",
        path.display()
    );

    println!();
    println!("Done! Open the .wav files in your DAW or audio player to compare.");
}
