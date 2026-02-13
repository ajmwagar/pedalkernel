//! Fuzz Face-style distortion → WAV output.
//!
//! Renders a guitar pluck through the WDF germanium fuzz at
//! several fuzz settings.
//!
//! Run: `cargo run --example fuzzface`

use pedalkernel::pedals::FuzzFace;
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

    let input = guitar_pluck(freq, duration, sample_rate);

    // Low fuzz
    let mut ff = FuzzFace::new(sample_rate as f64);
    ff.set_fuzz(0.3);
    ff.set_volume(0.7);
    let path = Path::new("fuzz_low.wav");
    render_to_wav(&mut ff, &input, path, sample_rate).unwrap();
    println!("Wrote low fuzz (0.3)  → {}", path.display());

    // Full fuzz
    let mut ff = FuzzFace::new(sample_rate as f64);
    ff.set_fuzz(1.0);
    ff.set_volume(0.6);
    let path = Path::new("fuzz_full.wav");
    render_to_wav(&mut ff, &input, path, sample_rate).unwrap();
    println!("Wrote full fuzz (1.0) → {}", path.display());

    // A/B comparison
    let mut ff = FuzzFace::new(sample_rate as f64);
    ff.set_fuzz(0.8);
    ff.set_volume(0.7);
    ff.reset();
    let output: Vec<f64> = input.iter().map(|&s| ff.process(s)).collect();
    let path = Path::new("fuzz_ab.wav");
    write_stereo_wav(&input, &output, path, sample_rate).unwrap();
    println!("Wrote A/B stereo (L=clean, R=fuzz 0.8) → {}", path.display());

    println!();
    println!("Done!");
}
