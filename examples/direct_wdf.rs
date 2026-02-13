//! Direct WDF tree usage — build a custom clipper from raw elements.
//!
//! Demonstrates using the WDF engine directly without the pedal
//! abstraction layer, comparing silicon, germanium, and LED diode models.
//!
//! Run: `cargo run --example direct_wdf`

use pedalkernel::elements::*;
use pedalkernel::tree::*;
use pedalkernel::wav::{sine_wave, write_wav, write_stereo_wav};
use std::path::Path;

fn main() {
    let sample_rate = 48_000;
    let duration = 1.0;
    let freq = 440.0;

    println!("PedalKernel — Direct WDF Tree Example");
    println!("======================================");
    println!();

    let input = sine_wave(freq, duration, sample_rate);

    // Silicon diode pair — smooth, warm clipping
    let mut si = WdfClipper::new(4700.0, 47e-9, DiodeModel::silicon(), sample_rate as f64);
    let si_out: Vec<f64> = input.iter().map(|&s| si.process(s)).collect();
    write_wav(&si_out, Path::new("wdf_silicon.wav"), sample_rate).unwrap();
    println!("Silicon diode pair  → wdf_silicon.wav");

    // Germanium diode pair — softer, earlier clipping
    let mut ge = WdfClipper::new(4700.0, 47e-9, DiodeModel::germanium(), sample_rate as f64);
    let ge_out: Vec<f64> = input.iter().map(|&s| ge.process(s)).collect();
    write_wav(&ge_out, Path::new("wdf_germanium.wav"), sample_rate).unwrap();
    println!("Germanium diode pair → wdf_germanium.wav");

    // LED diode pair — hard clipping, higher headroom
    let mut led = WdfClipper::new(4700.0, 47e-9, DiodeModel::led(), sample_rate as f64);
    let led_out: Vec<f64> = input.iter().map(|&s| led.process(s)).collect();
    write_wav(&led_out, Path::new("wdf_led.wav"), sample_rate).unwrap();
    println!("LED diode pair       → wdf_led.wav");

    // Single diode (asymmetric) — germanium
    let mut asym = WdfSingleDiodeClipper::new(4700.0, 47e-9, DiodeModel::germanium(), sample_rate as f64);
    let asym_out: Vec<f64> = input.iter().map(|&s| asym.process(s)).collect();
    write_wav(&asym_out, Path::new("wdf_asymmetric.wav"), sample_rate).unwrap();
    println!("Single diode (asym)  → wdf_asymmetric.wav");

    // Side-by-side: silicon vs germanium
    write_stereo_wav(&si_out, &ge_out, Path::new("wdf_si_vs_ge.wav"), sample_rate).unwrap();
    println!();
    println!("Stereo comparison (L=silicon, R=germanium) → wdf_si_vs_ge.wav");

    // Vary resistance to show gain effect
    println!();
    println!("Resistance sweep (lower R = more clipping):");
    for &r in &[10000.0, 4700.0, 1000.0, 470.0] {
        let mut c = WdfClipper::new(r, 47e-9, DiodeModel::silicon(), sample_rate as f64);
        let out: Vec<f64> = input.iter().map(|&s| c.process(s)).collect();
        let peak = out.iter().copied().fold(0.0_f64, |a, b| a.max(b.abs()));
        let filename = format!("wdf_r{}.wav", r as u32);
        write_wav(&out, Path::new(&filename), sample_rate).unwrap();
        println!("  R={:>7.0} Ω  peak={:.4}  → {}", r, peak, filename);
    }

    println!();
    println!("Done!");
}
