//! Tube Screamer-style overdrive → WAV output.
//!
//! Renders a 440 Hz guitar-like pluck through the WDF overdrive at
//! three gain settings and writes each to a separate WAV file.
//!
//! Run: `cargo run --example overdrive`

use pedalkernel::pedals::Overdrive;
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

    let input = guitar_pluck(freq, duration, sample_rate);

    // Clean signal
    let clean_path = Path::new("overdrive_clean.wav");
    pedalkernel::wav::write_wav(&input, clean_path, sample_rate).unwrap();
    println!("Wrote clean signal → {}", clean_path.display());

    // Low gain
    let mut od_low = Overdrive::new(sample_rate as f64);
    od_low.set_gain(0.2);
    od_low.set_level(0.8);
    let path = Path::new("overdrive_low.wav");
    render_to_wav(&mut od_low, &input, path, sample_rate).unwrap();
    println!("Wrote low gain (0.2) → {}", path.display());

    // Medium gain
    let mut od_mid = Overdrive::new(sample_rate as f64);
    od_mid.set_gain(0.5);
    od_mid.set_level(0.8);
    let path = Path::new("overdrive_mid.wav");
    render_to_wav(&mut od_mid, &input, path, sample_rate).unwrap();
    println!("Wrote mid gain (0.5) → {}", path.display());

    // High gain
    let mut od_high = Overdrive::new(sample_rate as f64);
    od_high.set_gain(0.9);
    od_high.set_level(0.8);
    let path = Path::new("overdrive_high.wav");
    render_to_wav(&mut od_high, &input, path, sample_rate).unwrap();
    println!("Wrote high gain (0.9) → {}", path.display());

    // A/B comparison — clean vs high gain in stereo
    let mut od = Overdrive::new(sample_rate as f64);
    od.set_gain(0.9);
    od.set_level(0.8);
    od.reset();
    let output: Vec<f64> = input.iter().map(|&s| od.process(s)).collect();
    let path = Path::new("overdrive_ab.wav");
    write_stereo_wav(&input, &output, path, sample_rate).unwrap();
    println!(
        "Wrote A/B stereo (L=clean, R=high gain) → {}",
        path.display()
    );

    println!();
    println!("Done! Open the .wav files in your DAW or audio player to compare.");
}
