//! FET Leveler dynamics gap report: ngspice golden vs WDF engine.
//!
//! Purpose: quantify the WDF-vs-ngspice delta on the feedback-compressor
//! path.  The expected outcome is that ngspice shows real compression
//! (~4.2 dB GR, ratio >1.3:1, real attack/release) while the WDF engine
//! shows ~0 dB GR because the BJT makeup-gain + transformer losses starve
//! the feedback detector (audit gap G5 family, bead pedalkernel-mbte.1).
//!
//! This test ALWAYS prints the gap table and ALWAYS passes: it is a
//! measurement, not a gate.  No criteria are asserted beyond "the circuit
//! produced finite output" — the gap itself IS the deliverable.
//!
//! Run with:
//!   cargo test -p pedalkernel-validate --test compressor_dynamics_gap -- --nocapture

use pedalkernel_validate::{npy, signals};
use pedalkernel::PedalProcessor;
use std::path::PathBuf;

/// Internal rate used for the golden (96 kHz × 4×).
const INTERNAL_RATE: f64 = 384_000.0;
/// Output rate for WDF processing (no oversampling in the runner).
const SAMPLE_RATE: f64 = 384_000.0;

fn validate_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn golden_path(suite: &str, test: &str, label: &str) -> PathBuf {
    validate_dir()
        .join("golden")
        .join(suite)
        .join(test)
        .join(format!("{label}.npy"))
}

fn circuit_path(suite: &str, name: &str) -> PathBuf {
    validate_dir()
        .join("circuits")
        .join(suite)
        .join(name)
}

/// Compile a .pedal circuit at SAMPLE_RATE and return a processor closure.
fn compile_pedal(path: &std::path::Path, controls: &[(&str, f64)]) -> impl FnMut(f64) -> f64 {
    let src = std::fs::read_to_string(path)
        .unwrap_or_else(|e| panic!("Cannot read {}: {e}", path.display()));
    let pedal_def = pedalkernel::dsl::parse_pedal_file(&src)
        .unwrap_or_else(|e| panic!("Parse error in {}: {e}", path.display()));
    let options = pedalkernel::compiler::CompileOptions {
        oversampling: pedalkernel::oversampling::OversamplingFactor::X1,
        ..pedalkernel::compiler::CompileOptions::default()
    };
    let mut compiled = pedalkernel::compiler::compile_pedal_with_options(&pedal_def, SAMPLE_RATE, options)
        .unwrap_or_else(|e| panic!("Compile error in {}: {e}", path.display()));
    for &(name, val) in controls {
        compiled.set_control(name, val);
    }
    move |x| compiled.process(x)
}

// ---------------------------------------------------------------------------
// Dynamics helpers (mirrors audio_analysis in pedalkernel/tests/)
// ---------------------------------------------------------------------------

fn rms(buf: &[f64]) -> f64 {
    if buf.is_empty() { return 0.0; }
    (buf.iter().map(|x| x * x).sum::<f64>() / buf.len() as f64).sqrt()
}

fn db(linear: f64) -> f64 {
    20.0 * linear.log10()
}

/// Static gain at one level: fresh processor, 0.5 s settle, 0.25 s measure.
fn static_gain_db(
    circuit: &std::path::Path,
    controls: &[(&str, f64)],
    amplitude_dbvu: f64,
    freq_hz: f64,
) -> f64 {
    let settle_s = 0.5;
    let window_s = 0.25;
    let amplitude = signals::dbvu_to_peak(amplitude_dbvu);
    let total_s = settle_s + window_s;
    let _n = (total_s * SAMPLE_RATE) as usize;
    let settle = (settle_s * SAMPLE_RATE) as usize;

    let mut process = compile_pedal(circuit, controls);
    let input: Vec<f64> = signals::sine(SAMPLE_RATE, freq_hz, total_s, amplitude);
    let output: Vec<f64> = input.iter().map(|&x| process(x)).collect();

    let in_rms = rms(&input[settle..]);
    let out_rms = rms(&output[settle..]);
    if out_rms < 1e-30 || in_rms < 1e-30 { return f64::NEG_INFINITY; }
    db(out_rms / in_rms)
}

/// Gain reduction: difference in steady-state gain between lo_dbvu and hi_dbvu.
fn gain_reduction_from_curve(curve: &[(f64, f64)], lo_dbvu: f64, hi_dbvu: f64) -> f64 {
    let nearest = |target: f64| {
        curve.iter()
            .min_by(|a, b| (a.0 - target).abs().partial_cmp(&(b.0 - target).abs()).unwrap())
            .copied()
            .unwrap()
    };
    let (_, gain_lo) = nearest(lo_dbvu);
    let (_, gain_hi) = nearest(hi_dbvu);
    gain_lo - gain_hi
}

/// Compression ratio: 1 / slope of output dB vs input dB over the region.
fn compression_ratio_from_curve(curve: &[(f64, f64)], lo_dbvu: f64, hi_dbvu: f64) -> f64 {
    let pts: Vec<(f64, f64)> = curve.iter()
        .copied()
        .filter(|&(x, _)| x >= lo_dbvu && x <= hi_dbvu)
        .collect();
    if pts.len() < 2 { return 1.0; }
    let n = pts.len() as f64;
    let mx = pts.iter().map(|p| p.0).sum::<f64>() / n;
    let my = pts.iter().map(|p| p.1).sum::<f64>() / n;
    let (mut sxy, mut sxx) = (0.0, 0.0);
    for &(x, y) in &pts {
        sxy += (x - mx) * (y - my);
        sxx += (x - mx) * (x - mx);
    }
    let slope = sxy / sxx;
    if slope.abs() < 1e-9 { f64::INFINITY } else { 1.0 / slope }
}

/// Extract gain curve from ngspice golden level-sweep npy.
/// The golden is a level sweep: 9 levels × 0.5 s at 384 kHz internal rate.
/// Returns Vec<(in_dbvu, out_dbvu)>.
fn gain_curve_from_golden(golden: &[f64], _freq_hz: f64) -> Vec<(f64, f64)> {
    let levels_dbvu: Vec<f64> = (-40..=0).step_by(5).map(|l| l as f64).collect();
    let samples_per_level = (0.5 * INTERNAL_RATE) as usize;
    let settle_samples = (0.2 * INTERNAL_RATE) as usize;   // 200 ms warmup trim
    let mut curve = Vec::new();

    for (i, &in_db) in levels_dbvu.iter().enumerate() {
        let start = i * samples_per_level;
        let end = start + samples_per_level;
        if end > golden.len() { break; }
        let seg = &golden[start + settle_samples..end];
        let out_rms = rms(seg);
        let out_db = if out_rms > 1e-30 { db(out_rms * std::f64::consts::SQRT_2) } else { -200.0 };
        curve.push((in_db, out_db));
    }
    curve
}

/// RMS-envelope settling time from a step transition (shared logic for attack/release).
fn envelope_settle_seconds(output: &[f64], step_sample: usize) -> f64 {
    let window_samples = (0.002 * SAMPLE_RATE) as usize;
    if step_sample >= output.len() || window_samples == 0 { return 0.0; }
    let seg = &output[step_sample..];
    let env: Vec<f64> = seg.windows(window_samples)
        .map(rms)
        .collect();
    if env.is_empty() { return 0.0; }
    let tail_start = env.len() - env.len() / 4;
    let tail = &env[tail_start..];
    let target = tail.iter().sum::<f64>() / tail.len().max(1) as f64;
    if target < 1e-12 { return 0.0; }
    let lo = target * 10_f64.powf(-1.0 / 20.0);
    let hi = target * 10_f64.powf(1.0 / 20.0);
    let mut settle_idx = 0;
    for (i, &e) in env.iter().enumerate().take(tail_start) {
        if e < lo || e > hi { settle_idx = i + 1; }
    }
    settle_idx as f64 / SAMPLE_RATE
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// Gap measurement: prints WDF-vs-ngspice GR/ratio/attack/release table.
/// Never fails on dynamics — only asserts finite output.
#[test]
fn fet_leveler_dynamics_gap_report() {
    let circuit = circuit_path("compressor", "fet_leveler.pedal");
    let controls: &[(&str, f64)] = &[("Input", 0.7), ("Output", 0.7)];

    // ── 1. Static gain curve: WDF ────────────────────────────────────────────
    eprintln!("\n=== FET Leveler: Static Gain Curve (WDF) ===");
    let levels_dbvu: Vec<f64> = (-40..=0).step_by(5).map(|l| l as f64).collect();
    let wdf_curve: Vec<(f64, f64)> = levels_dbvu.iter().map(|&in_db| {
        let g = static_gain_db(&circuit, controls, in_db, 1000.0);
        eprintln!("  WDF: in {in_db:6.1} dBVU -> gain {g:8.3} dB");
        (in_db, g)
    }).collect();

    let wdf_gr = gain_reduction_from_curve(&wdf_curve, -40.0, 0.0);
    let wdf_ratio = compression_ratio_from_curve(&wdf_curve, -20.0, 0.0);
    eprintln!("  WDF GR (-40 vs 0 dBVU):    {wdf_gr:+.3} dB");
    eprintln!("  WDF ratio (-20..0 dBVU):   {wdf_ratio:.4}:1");

    // ── 2. Static gain curve: ngspice golden ────────────────────────────────
    let golden_sweep_path = golden_path("compressor", "fet_leveler_level_sweep", "level_sweep");
    let ngspice_curve: Vec<(f64, f64)> = if golden_sweep_path.exists() {
        let golden = npy::read_f64(&golden_sweep_path)
            .expect("Failed to read level_sweep golden");
        eprintln!("\n=== FET Leveler: Static Gain Curve (ngspice) ===");
        let curve = gain_curve_from_golden(&golden, 1000.0);
        for &(in_db, out_db) in &curve {
            eprintln!("  SPICE: in {in_db:6.1} dBVU -> out {out_db:8.3} dBVU (gain {:8.3} dB)",
                out_db - signals::dbvu_to_peak(in_db).log10() * 20.0);
        }
        curve
    } else {
        eprintln!("  WARN: level_sweep golden not found at {}", golden_sweep_path.display());
        Vec::new()
    };

    let (ngspice_gr, ngspice_ratio) = if ngspice_curve.len() >= 2 {
        let gr = gain_reduction_from_curve(&ngspice_curve, -40.0, 0.0);
        let ratio = compression_ratio_from_curve(&ngspice_curve, -20.0, 0.0);
        eprintln!("  SPICE GR (-40 vs 0 dBVU):  {gr:+.3} dB");
        eprintln!("  SPICE ratio (-20..0 dBVU): {ratio:.4}:1");
        (gr, ratio)
    } else {
        (f64::NAN, f64::NAN)
    };

    // ── 3. Attack / release: WDF ────────────────────────────────────────────
    eprintln!("\n=== FET Leveler: Attack/Release (WDF) ===");
    let burst_amplitude = signals::dbvu_to_peak(-6.0);
    let on_samples = (0.5 * SAMPLE_RATE) as usize;
    let off_samples = (2.0 * SAMPLE_RATE) as usize;
    let burst_signal = signals::tone_burst(SAMPLE_RATE, 1000.0, burst_amplitude, 500.0, 2000.0, 1);

    let mut process_wdf = compile_pedal(&circuit, controls);
    let wdf_burst_out: Vec<f64> = burst_signal.iter().map(|&x| process_wdf(x)).collect();

    assert!(
        wdf_burst_out.iter().all(|x| x.is_finite()),
        "WDF tone burst output contains NaN/inf"
    );

    let wdf_attack = envelope_settle_seconds(&wdf_burst_out, on_samples);
    let wdf_release = envelope_settle_seconds(&wdf_burst_out, on_samples + off_samples);
    eprintln!("  WDF attack:  {:.1} ms", wdf_attack * 1000.0);
    eprintln!("  WDF release: {:.1} ms", wdf_release * 1000.0);

    // ── 4. Attack / release: ngspice golden ─────────────────────────────────
    let golden_burst_path = golden_path("compressor", "fet_leveler_tone_burst", "tone_burst");
    let (ngspice_attack, ngspice_release) = if golden_burst_path.exists() {
        eprintln!("\n=== FET Leveler: Attack/Release (ngspice) ===");
        let golden_burst = npy::read_f64(&golden_burst_path)
            .expect("Failed to read tone_burst golden");
        let attack = envelope_settle_seconds(&golden_burst, on_samples);
        let release = envelope_settle_seconds(&golden_burst, on_samples + off_samples);
        eprintln!("  SPICE attack:  {:.1} ms", attack * 1000.0);
        eprintln!("  SPICE release: {:.1} ms", release * 1000.0);
        (attack, release)
    } else {
        eprintln!("  WARN: tone_burst golden not found at {}", golden_burst_path.display());
        (f64::NAN, f64::NAN)
    };

    // ── 5. Gap table ─────────────────────────────────────────────────────────
    eprintln!("\n╔══════════════════════════════════════════════════════╗");
    eprintln!("║   FET Leveler WDF-vs-ngspice Gap Report              ║");
    eprintln!("╠══════════════════════════════════════════════════════╣");
    eprintln!("║ Metric              │ WDF (engine) │ ngspice ref   ║");
    eprintln!("╠══════════════════════════════════════════════════════╣");
    eprintln!("║ GR (-40 vs 0 dBVU) │ {:>12.3} dB │ {:>12.3} dB ║",
        wdf_gr, ngspice_gr);
    eprintln!("║ Ratio (-20..0 dBVU)│ {:>12.4}:1 │ {:>12.4}:1 ║",
        wdf_ratio, ngspice_ratio);
    eprintln!("║ Attack (ms)        │ {:>12.1}    │ {:>12.1}    ║",
        wdf_attack * 1000.0, ngspice_attack * 1000.0);
    eprintln!("║ Release (ms)       │ {:>12.1}    │ {:>12.1}    ║",
        wdf_release * 1000.0, ngspice_release * 1000.0);
    eprintln!("╚══════════════════════════════════════════════════════╝");
    eprintln!("Root cause: BJT makeup-gain + transformer losses leave the");
    eprintln!("feedback detector at ~-58 dB, starving compression (audit G5).");

    // Only assert finite output — the gap is the deliverable, not a gate.
    assert!(
        wdf_burst_out.iter().all(|x| x.is_finite()),
        "WDF output must be finite"
    );
}
