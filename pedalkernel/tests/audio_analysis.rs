//! Shared audio analysis utilities for integration tests.
//!
//! Provides Goertzel-based spectral analysis, THD measurement, modulation
//! detection, and convenience wrappers for compiling + processing pedals.
//! No external FFT crate — pure Rust math.

#![allow(dead_code)]

use std::path::Path;
use std::sync::LazyLock;

use pedalkernel::compiler::{compile_pedal, compile_pedal_with_options, CompileOptions};
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::wav::{read_wav_mono, write_wav};
use pedalkernel::PedalProcessor;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

pub const SAMPLE_RATE: f64 = 48_000.0;
pub const SAMPLE_RATE_U32: u32 = 48_000;

// ---------------------------------------------------------------------------
// Cached clean guitar input (loaded once, shared across all tests)
// ---------------------------------------------------------------------------

pub static CLEAN_GUITAR: LazyLock<(Vec<f64>, u32)> = LazyLock::new(|| {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/wav/clean_guitar.wav");
    read_wav_mono(&path).expect("failed to load clean_guitar.wav")
});

// ---------------------------------------------------------------------------
// Signal generators
// ---------------------------------------------------------------------------

/// Generate a sine wave at `freq_hz` with amplitude 0.5.
pub fn sine(freq_hz: f64, duration_secs: f64, sample_rate: f64) -> Vec<f64> {
    let n = (duration_secs * sample_rate) as usize;
    (0..n)
        .map(|i| {
            let t = i as f64 / sample_rate;
            0.5 * (2.0 * std::f64::consts::PI * freq_hz * t).sin()
        })
        .collect()
}

/// Generate a sine wave at a given amplitude.
pub fn sine_at(freq_hz: f64, amplitude: f64, duration_secs: f64, sample_rate: f64) -> Vec<f64> {
    let n = (duration_secs * sample_rate) as usize;
    (0..n)
        .map(|i| {
            let t = i as f64 / sample_rate;
            amplitude * (2.0 * std::f64::consts::PI * freq_hz * t).sin()
        })
        .collect()
}

/// Multi-harmonic guitar pluck with exponential decay.
pub fn guitar_pluck(freq_hz: f64, duration_secs: f64, sample_rate: f64) -> Vec<f64> {
    let n = (duration_secs * sample_rate) as usize;
    (0..n)
        .map(|i| {
            let t = i as f64 / sample_rate;
            let env = (-3.0 * t).exp();
            let f = 2.0 * std::f64::consts::PI * freq_hz * t;
            0.4 * env * (f.sin() + 0.5 * (2.0 * f).sin() + 0.25 * (3.0 * f).sin())
        })
        .collect()
}

/// DC step signal: `value` for `duration_secs`, then zero for `zero_secs`.
pub fn step_then_zero(
    value: f64,
    duration_secs: f64,
    zero_secs: f64,
    sample_rate: f64,
) -> Vec<f64> {
    let n_on = (duration_secs * sample_rate) as usize;
    let n_off = (zero_secs * sample_rate) as usize;
    let mut buf = vec![value; n_on];
    buf.extend(vec![0.0; n_off]);
    buf
}

// ---------------------------------------------------------------------------
// Basic measurements
// ---------------------------------------------------------------------------

/// RMS level.
pub fn rms(buf: &[f64]) -> f64 {
    if buf.is_empty() {
        return 0.0;
    }
    (buf.iter().map(|x| x * x).sum::<f64>() / buf.len() as f64).sqrt()
}

/// Peak absolute amplitude.
pub fn peak(buf: &[f64]) -> f64 {
    buf.iter().fold(0.0f64, |m, x| m.max(x.abs()))
}

/// DC offset (mean).
pub fn dc_offset(buf: &[f64]) -> f64 {
    if buf.is_empty() {
        return 0.0;
    }
    buf.iter().sum::<f64>() / buf.len() as f64
}

/// Crest factor = peak / RMS. Lower means more compressed/clipped.
pub fn crest_factor(buf: &[f64]) -> f64 {
    let r = rms(buf);
    if r < 1e-20 {
        return 0.0;
    }
    peak(buf) / r
}

/// Normalized cross-correlation between two buffers.
pub fn correlation(a: &[f64], b: &[f64]) -> f64 {
    let n = a.len().min(b.len()) as f64;
    if n == 0.0 {
        return 0.0;
    }
    let ma = a.iter().sum::<f64>() / n;
    let mb = b.iter().sum::<f64>() / n;
    let (mut cov, mut va, mut vb) = (0.0, 0.0, 0.0);
    for i in 0..n as usize {
        let (da, db) = (a[i] - ma, b[i] - mb);
        cov += da * db;
        va += da * da;
        vb += db * db;
    }
    if va < 1e-30 || vb < 1e-30 {
        return 0.0;
    }
    cov / (va.sqrt() * vb.sqrt())
}

/// Energy of a buffer (sum of squares).
pub fn energy(buf: &[f64]) -> f64 {
    buf.iter().map(|x| x * x).sum()
}

// ---------------------------------------------------------------------------
// Goertzel algorithm — efficient single-bin DFT
// ---------------------------------------------------------------------------

/// Goertzel algorithm: returns magnitude² of DFT at `target_hz`.
/// O(N) per bin — much cheaper than full FFT when we only need a few bins.
pub fn goertzel_power(buf: &[f64], sample_rate: f64, target_hz: f64) -> f64 {
    let n = buf.len() as f64;
    let k = (target_hz * n / sample_rate).round();
    let w = 2.0 * std::f64::consts::PI * k / n;
    let coeff = 2.0 * w.cos();

    let mut s1 = 0.0;
    let mut s2 = 0.0;

    for &x in buf {
        let s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    // Power = s1² + s2² - coeff * s1 * s2
    let power = s1 * s1 + s2 * s2 - coeff * s1 * s2;
    power / (n * n / 4.0) // Normalize so a full-scale sine gives ~1.0
}

/// Goertzel magnitude (square root of power).
pub fn goertzel_mag(buf: &[f64], sample_rate: f64, target_hz: f64) -> f64 {
    goertzel_power(buf, sample_rate, target_hz).max(0.0).sqrt()
}

// ---------------------------------------------------------------------------
// Spectral analysis
// ---------------------------------------------------------------------------

/// THD: ratio of harmonic energy (harmonics 2-8) to fundamental energy.
/// Returns a value in [0, ∞). 0 = pure sine, >1 = harmonics dominate.
pub fn thd(buf: &[f64], sample_rate: f64, fundamental_hz: f64) -> f64 {
    let fund_power = goertzel_power(buf, sample_rate, fundamental_hz);
    if fund_power < 1e-30 {
        return 0.0;
    }

    let mut harmonic_power = 0.0;
    for h in 2..=8 {
        let freq = fundamental_hz * h as f64;
        if freq > sample_rate / 2.0 {
            break;
        }
        harmonic_power += goertzel_power(buf, sample_rate, freq);
    }

    (harmonic_power / fund_power).sqrt()
}

/// Energy in frequency band [lo_hz, hi_hz] relative to total wideband energy.
/// Steps through band at `step_hz` resolution.
pub fn band_energy_ratio(
    buf: &[f64],
    sample_rate: f64,
    lo_hz: f64,
    hi_hz: f64,
    step_hz: f64,
) -> f64 {
    let total_energy: f64 = buf.iter().map(|x| x * x).sum();
    if total_energy < 1e-30 {
        return 0.0;
    }

    let mut band = 0.0;
    let mut freq = lo_hz;
    while freq <= hi_hz {
        band += goertzel_power(buf, sample_rate, freq);
        freq += step_hz;
    }
    band / total_energy
}

/// Spectral centroid — perceptual "brightness" in Hz.
/// Computed over linearly-spaced bins from 50 Hz to Nyquist/2.
pub fn spectral_centroid(buf: &[f64], sample_rate: f64) -> f64 {
    let nyquist = sample_rate / 2.0;
    let step = 50.0;
    let mut weighted_sum = 0.0;
    let mut total_power = 0.0;

    let mut freq = 50.0;
    while freq < nyquist / 2.0 {
        let p = goertzel_power(buf, sample_rate, freq);
        weighted_sum += freq * p;
        total_power += p;
        freq += step;
    }

    if total_power < 1e-30 {
        return 0.0;
    }
    weighted_sum / total_power
}

// ---------------------------------------------------------------------------
// Modulation detection
// ---------------------------------------------------------------------------

/// Detect dominant modulation rate by amplitude-envelope analysis.
///
/// 1. Compute amplitude envelope (rectify + lowpass)
/// 2. Remove DC from envelope
/// 3. Find peak frequency in 0.1–20 Hz range via Goertzel
pub fn detect_modulation_rate(buf: &[f64], sample_rate: f64) -> f64 {
    if buf.len() < 1000 {
        return 0.0;
    }

    // Compute amplitude envelope: |x| smoothed with ~50 Hz lowpass
    let alpha = (-2.0 * std::f64::consts::PI * 50.0 / sample_rate).exp();
    let mut envelope = Vec::with_capacity(buf.len());
    let mut env = 0.0;
    for &x in buf {
        env = alpha * env + (1.0 - alpha) * x.abs();
        envelope.push(env);
    }

    // Remove DC
    let dc = envelope.iter().sum::<f64>() / envelope.len() as f64;
    for e in &mut envelope {
        *e -= dc;
    }

    // Scan 0.1–20 Hz for peak modulation frequency
    let mut best_freq = 0.0;
    let mut best_power = 0.0;
    let mut freq = 0.1;
    while freq <= 20.0 {
        let p = goertzel_power(&envelope, sample_rate, freq);
        if p > best_power {
            best_power = p;
            best_freq = freq;
        }
        freq += 0.05;
    }

    best_freq
}

/// Check whether amplitude envelope varies significantly over time.
/// Splits buffer into chunks and checks if RMS varies between them.
pub fn has_amplitude_modulation(buf: &[f64], num_chunks: usize) -> bool {
    if buf.len() < num_chunks * 100 {
        return false;
    }
    let chunk_size = buf.len() / num_chunks;
    let rms_values: Vec<f64> = (0..num_chunks)
        .map(|i| rms(&buf[i * chunk_size..(i + 1) * chunk_size]))
        .collect();

    let max_rms = rms_values.iter().cloned().fold(0.0f64, f64::max);
    let min_rms = rms_values.iter().cloned().fold(f64::MAX, f64::min);

    if max_rms < 1e-10 {
        return false;
    }
    // More than 10% variation = modulation detected
    (max_rms - min_rms) / max_rms > 0.10
}

// ---------------------------------------------------------------------------
// Compile + process helpers
// ---------------------------------------------------------------------------

/// Compile a .pedal source string and process input through it.
pub fn compile_and_process(
    pedal_src: &str,
    input: &[f64],
    sample_rate: f64,
    controls: &[(&str, f64)],
) -> Vec<f64> {
    let pedal = parse_pedal_file(pedal_src).expect("failed to parse pedal source");
    let mut proc = compile_pedal(&pedal, sample_rate).expect("failed to compile pedal");
    for &(label, val) in controls {
        proc.set_control(label, val);
    }
    input.iter().map(|&s| proc.process(s)).collect()
}

/// Compile a .pedal source string with custom options and process input.
pub fn compile_and_process_with_options(
    pedal_src: &str,
    input: &[f64],
    sample_rate: f64,
    controls: &[(&str, f64)],
    options: CompileOptions,
) -> Vec<f64> {
    let pedal = parse_pedal_file(pedal_src).expect("failed to parse pedal source");
    let mut proc =
        compile_pedal_with_options(&pedal, sample_rate, options).expect("failed to compile pedal");
    for &(label, val) in controls {
        proc.set_control(label, val);
    }
    input.iter().map(|&s| proc.process(s)).collect()
}

/// Load a .pedal file from tests/test_pedals/ and process with custom options.
pub fn compile_test_pedal_with_options(
    filename: &str,
    input: &[f64],
    sample_rate: f64,
    controls: &[(&str, f64)],
    options: CompileOptions,
) -> Vec<f64> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/test_pedals")
        .join(filename);
    let src = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("failed to read {}: {e}", path.display()));
    compile_and_process_with_options(&src, input, sample_rate, controls, options)
}

/// Load an example .pedal file and process with custom options.
pub fn compile_example_with_options(
    filename: &str,
    input: &[f64],
    sample_rate: f64,
    controls: &[(&str, f64)],
    options: CompileOptions,
) -> Vec<f64> {
    let examples_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
    let path = find_file_recursive(&examples_dir, filename)
        .unwrap_or_else(|| panic!("example file not found: {filename}"));
    let src = std::fs::read_to_string(&path).unwrap();
    compile_and_process_with_options(&src, input, sample_rate, controls, options)
}

/// Load a .pedal file from tests/test_pedals/ and process input through it.
pub fn compile_test_pedal_and_process(
    filename: &str,
    input: &[f64],
    sample_rate: f64,
    controls: &[(&str, f64)],
) -> Vec<f64> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/test_pedals")
        .join(filename);
    let src = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("failed to read {}: {e}", path.display()));
    compile_and_process(&src, input, sample_rate, controls)
}

/// Load a .pedal file from examples/ (searches subdirectories).
pub fn compile_example_and_process(
    filename: &str,
    input: &[f64],
    sample_rate: f64,
    controls: &[(&str, f64)],
) -> Vec<f64> {
    let examples_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
    let path = find_file_recursive(&examples_dir, filename)
        .unwrap_or_else(|| panic!("example file not found: {filename}"));
    let src = std::fs::read_to_string(&path).unwrap();
    compile_and_process(&src, input, sample_rate, controls)
}

fn find_file_recursive(dir: &Path, filename: &str) -> Option<String> {
    if let Ok(entries) = std::fs::read_dir(dir) {
        for entry in entries.flatten() {
            let p = entry.path();
            if p.is_dir() {
                if let Some(found) = find_file_recursive(&p, filename) {
                    return Some(found);
                }
            } else if p.file_name().map_or(false, |n| n == filename) {
                return Some(p.to_string_lossy().to_string());
            }
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Assertions
// ---------------------------------------------------------------------------

/// Assert that all samples are finite, non-silent, and bounded.
pub fn assert_healthy(buf: &[f64], name: &str, max_peak: f64) {
    assert!(
        buf.iter().all(|x| x.is_finite()),
        "{name}: output contains NaN/inf"
    );
    let p = peak(buf);
    assert!(p > 1e-6, "{name}: output is silent (peak={p:.8})");
    assert!(
        p < max_peak,
        "{name}: output too loud (peak={p:.4}, max={max_peak})"
    );
}

/// Assert one value is greater than another with context.
pub fn assert_greater(a: f64, b: f64, msg: &str) {
    assert!(a > b, "{msg}: expected {a:.6} > {b:.6}");
}

// ---------------------------------------------------------------------------
// WAV dump (optional, for manual inspection)
// ---------------------------------------------------------------------------

/// Write output WAV when PEDALKERNEL_DUMP_WAV=1 is set.
pub fn maybe_dump_wav(samples: &[f64], name: &str, sample_rate: u32) {
    if std::env::var("PEDALKERNEL_DUMP_WAV").is_ok() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("test_output");
        std::fs::create_dir_all(&dir).ok();
        let path = dir.join(format!("{name}.wav"));
        write_wav(samples, &path, sample_rate).unwrap();
    }
}
