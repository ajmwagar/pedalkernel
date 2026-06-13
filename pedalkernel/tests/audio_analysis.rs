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
use std::cmp::Ordering;

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

/// Tone-burst stimulus for attack/release measurement: a continuous-phase
/// sine at `amp_lo` for `lo_secs`, stepped to `amp_hi` for `hi_secs`,
/// then back to `amp_lo` for `tail_secs`.
pub fn tone_burst(
    freq_hz: f64,
    amp_lo: f64,
    amp_hi: f64,
    lo_secs: f64,
    hi_secs: f64,
    tail_secs: f64,
    sample_rate: f64,
) -> Vec<f64> {
    let n_lo = (lo_secs * sample_rate) as usize;
    let n_hi = (hi_secs * sample_rate) as usize;
    let n_tail = (tail_secs * sample_rate) as usize;
    (0..n_lo + n_hi + n_tail)
        .map(|i| {
            let amp = if i >= n_lo && i < n_lo + n_hi {
                amp_hi
            } else {
                amp_lo
            };
            let t = i as f64 / sample_rate;
            amp * (2.0 * std::f64::consts::PI * freq_hz * t).sin()
        })
        .collect()
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

/// Convert a linear amplitude to decibels (floored at -240 dB).
pub fn lin_to_db(lin: f64) -> f64 {
    20.0 * lin.max(1e-12).log10()
}

/// Convert decibels to a linear amplitude.
pub fn db_to_lin(db: f64) -> f64 {
    10f64.powf(db / 20.0)
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

/// Summary of alias energy in the upper band.
#[derive(Debug, Clone)]
pub struct AliasSummary {
    pub alias_ratio: f64,
    pub top_bins: Vec<(f64, f64)>,
}

/// Estimate aliasing by measuring energy between 45% of Nyquist and Nyquist.
/// Excludes bins near harmonics of `fundamental_hz`.
pub fn alias_energy_summary(
    buf: &[f64],
    sample_rate: f64,
    fundamental_hz: f64,
    guard_hz: f64,
) -> AliasSummary {
    let total_energy: f64 = buf.iter().map(|x| x * x).sum();
    if total_energy < 1e-30 {
        return AliasSummary {
            alias_ratio: 0.0,
            top_bins: Vec::new(),
        };
    }
    let nyquist = sample_rate / 2.0;
    let alias_start = nyquist * 0.45;
    let mut freq = alias_start;
    let mut alias_energy = 0.0;
    let mut bins: Vec<(f64, f64)> = Vec::new();

    let mut harmonics = Vec::new();
    let mut n = 1.0;
    while fundamental_hz * n <= nyquist + guard_hz {
        harmonics.push(fundamental_hz * n);
        n += 1.0;
    }

    while freq <= nyquist {
        if harmonics.iter().any(|&h| (h - freq).abs() < guard_hz) {
            freq += 200.0;
            continue;
        }
        let power = goertzel_power(buf, sample_rate, freq);
        alias_energy += power;
        bins.push((freq, power));
        freq += 200.0;
    }

    bins.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or_else(|| Ordering::Equal));
    bins.truncate(3);

    AliasSummary {
        alias_ratio: alias_energy / total_energy,
        top_bins: bins,
    }
}

/// Count subnormal (denormalized) floating point values in buffer.
pub fn count_subnormals(buf: &[f64]) -> usize {
    buf.iter().filter(|x| x.is_subnormal()).count()
}

/// Downsample by 2 with a simple 3-tap lowpass prefilter.
pub fn downsample_by_2(buf: &[f64]) -> Vec<f64> {
    if buf.len() < 2 {
        return Vec::new();
    }
    let mut out = Vec::with_capacity(buf.len() / 2 + 1);
    let mut i = 0;
    while i + 1 < buf.len() {
        let x0 = buf[i];
        let x1 = buf[i + 1];
        let x2 = if i + 2 < buf.len() { buf[i + 2] } else { x1 };
        out.push((x0 + 2.0 * x1 + x2) * 0.25);
        i += 2;
    }
    out
}

/// Resample buffer from `src_rate` to `dst_rate` using linear interpolation.
pub fn resample_linear(buf: &[f64], src_rate: f64, dst_rate: f64) -> Vec<f64> {
    if buf.is_empty() {
        return Vec::new();
    }
    let duration = buf.len() as f64 / src_rate;
    let dst_len = (duration * dst_rate).round() as usize;
    if dst_len == 0 {
        return Vec::new();
    }

    let mut out = Vec::with_capacity(dst_len);
    for n in 0..dst_len {
        let t = n as f64 / dst_rate;
        let src_pos = t * src_rate;
        let idx = src_pos.floor() as usize;
        let frac = src_pos - idx as f64;
        let a = buf.get(idx).copied().unwrap_or(*buf.last().unwrap());
        let b = buf.get(idx + 1).copied().unwrap_or(a);
        out.push(a + (b - a) * frac);
    }
    out
}

/// Spectral log-magnitude distance between two buffers.
pub fn spectral_distance(a: &[f64], b: &[f64], sample_rate: f64, step_hz: f64) -> f64 {
    let nyquist = sample_rate / 2.0;
    let mut freq = step_hz;
    let mut accum = 0.0;
    let mut bins = 0.0;
    while freq < nyquist {
        let pa = goertzel_power(a, sample_rate, freq).max(1e-18);
        let pb = goertzel_power(b, sample_rate, freq).max(1e-18);
        let diff = (pa.log10() - pb.log10()).abs();
        accum += diff;
        bins += 1.0;
        freq += step_hz;
    }
    if bins < 1.0 {
        0.0
    } else {
        accum / bins
    }
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

/// Compile a .pedal source string into a fresh sample-by-sample process
/// closure. Useful for measurement sweeps that recompile per data point so
/// reactive/envelope state cannot leak between measurements.
pub fn pedal_processor(
    pedal_src: &str,
    sample_rate: f64,
    controls: &[(&str, f64)],
) -> impl FnMut(f64) -> f64 {
    let pedal = parse_pedal_file(pedal_src).expect("failed to parse pedal source");
    let mut proc = compile_pedal(&pedal, sample_rate).expect("failed to compile pedal");
    for &(label, val) in controls {
        proc.set_control(label, val);
    }
    move |s| proc.process(s)
}

/// Read a .pedal source string from tests/test_pedals/.
pub fn test_pedal_source(filename: &str) -> String {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/test_pedals")
        .join(filename);
    std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("failed to read {}: {e}", path.display()))
}

/// Read a .pedal source string from examples/ (searches subdirectories).
pub fn example_pedal_source(filename: &str) -> String {
    let examples_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
    let path = find_file_recursive(&examples_dir, filename)
        .unwrap_or_else(|| panic!("example file not found: {filename}"));
    std::fs::read_to_string(&path).unwrap()
}

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
// Outboard-gear measurements: frequency response, gain curves, dynamics
// ---------------------------------------------------------------------------

/// Settle time used before measuring steady-state response.
const MEASURE_SETTLE_SECS: f64 = 0.5;
/// Minimum Goertzel/RMS analysis window for steady-state measurements.
const MEASURE_WINDOW_SECS: f64 = 0.25;

/// Measure small-signal frequency response in dB at each probe frequency.
///
/// `make_processor` is called once per frequency so each point starts from a
/// fresh, deterministic state (recompile-per-point — state never leaks
/// between probes). For each frequency: synthesize a sine at `amplitude`,
/// discard 0.5 s of settle time, then measure output/input magnitude via
/// Goertzel over an integer number of cycles (so the bin lands exactly on
/// the probe frequency). Returns `(freq_hz, gain_db)` pairs.
pub fn frequency_response_db<F, P>(
    mut make_processor: F,
    freqs_hz: &[f64],
    amplitude: f64,
    sample_rate: f64,
) -> Vec<(f64, f64)>
where
    F: FnMut() -> P,
    P: FnMut(f64) -> f64,
{
    freqs_hz
        .iter()
        .map(|&freq| {
            let mut process = make_processor();
            // Integer cycle count keeps the Goertzel bin exactly on the probe.
            let cycles = (MEASURE_WINDOW_SECS * freq).ceil().max(4.0);
            let window = (cycles * sample_rate / freq).round() as usize;
            let settle = (MEASURE_SETTLE_SECS * sample_rate) as usize;
            let input = sine_at(
                freq,
                amplitude,
                (settle + window) as f64 / sample_rate,
                sample_rate,
            );
            let output: Vec<f64> = input.iter().map(|&s| process(s)).collect();
            let in_mag = goertzel_mag(&input[settle..], sample_rate, freq);
            let out_mag = goertzel_mag(&output[settle..], sample_rate, freq);
            (freq, lin_to_db(out_mag) - lin_to_db(in_mag))
        })
        .collect()
}

/// Measure the static (steady-state) gain curve of a processor.
///
/// For each input level (in dB relative to 1.0 sine peak), a fresh processor
/// is built via `make_processor` so envelope/compression state cannot leak
/// between points. A sine at `freq_hz` runs for 0.5 s of settle time, then
/// output RMS is measured over 0.25 s and converted to peak-equivalent dB
/// (`20·log10(rms·√2)`), so a unity-gain device returns output_db ≈ input_db.
/// Returns `(input_db, output_db)` pairs.
pub fn static_gain_curve<F, P>(
    mut make_processor: F,
    input_levels_db: &[f64],
    freq_hz: f64,
    sample_rate: f64,
) -> Vec<(f64, f64)>
where
    F: FnMut() -> P,
    P: FnMut(f64) -> f64,
{
    let settle = (MEASURE_SETTLE_SECS * sample_rate) as usize;
    let window = (MEASURE_WINDOW_SECS * sample_rate) as usize;
    input_levels_db
        .iter()
        .map(|&in_db| {
            let mut process = make_processor();
            let input = sine_at(
                freq_hz,
                db_to_lin(in_db),
                (settle + window) as f64 / sample_rate,
                sample_rate,
            );
            let output: Vec<f64> = input.iter().map(|&s| process(s)).collect();
            let out_db = lin_to_db(rms(&output[settle..]) * std::f64::consts::SQRT_2);
            (in_db, out_db)
        })
        .collect()
}

/// Compression ratio over a region of a static gain curve.
///
/// Fits a least-squares line to the `(input_db, output_db)` points whose
/// input level lies within `[lo_db, hi_db]` and returns
/// ratio = Δinput_db / Δoutput_db = 1 / slope. A linear device gives 1.0,
/// a limiter approaches +∞ (returned as `f64::INFINITY` when the curve is
/// flat). Requires at least two points in the region.
pub fn compression_ratio(curve: &[(f64, f64)], lo_db: f64, hi_db: f64) -> f64 {
    let pts: Vec<(f64, f64)> = curve
        .iter()
        .copied()
        .filter(|&(x, _)| x >= lo_db && x <= hi_db)
        .collect();
    assert!(
        pts.len() >= 2,
        "compression_ratio: need >= 2 curve points in [{lo_db}, {hi_db}], got {}",
        pts.len()
    );
    let n = pts.len() as f64;
    let mx = pts.iter().map(|p| p.0).sum::<f64>() / n;
    let my = pts.iter().map(|p| p.1).sum::<f64>() / n;
    let (mut sxy, mut sxx) = (0.0, 0.0);
    for &(x, y) in &pts {
        sxy += (x - mx) * (y - my);
        sxx += (x - mx) * (x - mx);
    }
    let slope = sxy / sxx;
    if slope.abs() < 1e-9 {
        f64::INFINITY
    } else {
        1.0 / slope
    }
}

/// Gain reduction in dB between two points on a static gain curve.
///
/// GR = gain(lo) - gain(hi), where gain = output_db - input_db at the curve
/// point nearest each requested input level. Positive GR means the device
/// applies less gain at the high level (compression); 0 means linear.
pub fn gain_reduction_db(curve: &[(f64, f64)], lo_db: f64, hi_db: f64) -> f64 {
    assert!(!curve.is_empty(), "gain_reduction_db: empty curve");
    let nearest = |target: f64| {
        curve
            .iter()
            .min_by(|a, b| {
                (a.0 - target)
                    .abs()
                    .partial_cmp(&(b.0 - target).abs())
                    .unwrap_or(Ordering::Equal)
            })
            .unwrap()
    };
    let lo = nearest(lo_db);
    let hi = nearest(hi_db);
    (lo.1 - lo.0) - (hi.1 - hi.0)
}

/// Short-window RMS envelope (trailing window, same length as input).
/// Windows shorter than one sample are clamped to one sample.
pub fn rms_envelope(buf: &[f64], window_secs: f64, sample_rate: f64) -> Vec<f64> {
    let w = ((window_secs * sample_rate) as usize).max(1);
    let mut out = Vec::with_capacity(buf.len());
    let mut sum_sq = 0.0;
    for i in 0..buf.len() {
        sum_sq += buf[i] * buf[i];
        if i >= w {
            sum_sq -= buf[i - w] * buf[i - w];
        }
        let n = (i + 1).min(w);
        out.push((sum_sq.max(0.0) / n as f64).sqrt());
    }
    out
}

/// Envelope window for attack/release timing (2 ms — short enough to track
/// fast attacks, long enough to smooth a 1 kHz carrier).
const ENVELOPE_WINDOW_SECS: f64 = 0.002;
/// Settling tolerance for attack/release timing.
const SETTLE_TOLERANCE_DB: f64 = 1.0;

/// Shared settling measurement for attack/release.
///
/// Convention: the new steady state is the mean envelope over the final 25%
/// of the post-step segment; the settle time is measured from `step_sample`
/// to the moment the 2 ms RMS envelope last leaves the ±1 dB band around
/// that steady state (i.e. enters the band and stays there). Returns 0.0 if
/// the envelope never leaves the band (step too shallow to resolve).
fn envelope_settle_seconds(output: &[f64], step_sample: usize, sample_rate: f64) -> f64 {
    assert!(
        step_sample < output.len(),
        "envelope_settle_seconds: step_sample {step_sample} out of range"
    );
    let env = rms_envelope(output, ENVELOPE_WINDOW_SECS, sample_rate);
    let segment = &env[step_sample..];
    let tail_start = segment.len() - segment.len() / 4;
    let tail = &segment[tail_start..];
    let target = tail.iter().sum::<f64>() / tail.len().max(1) as f64;
    if target < 1e-12 {
        return 0.0;
    }
    let lo = target * db_to_lin(-SETTLE_TOLERANCE_DB);
    let hi = target * db_to_lin(SETTLE_TOLERANCE_DB);
    let mut settle_idx = 0;
    for (i, &e) in segment.iter().enumerate().take(tail_start) {
        if e < lo || e > hi {
            settle_idx = i + 1;
        }
    }
    settle_idx as f64 / sample_rate
}

/// Attack time: seconds from a level step-up at `step_sample` until the
/// short-window RMS envelope settles within ±1 dB of its new steady state.
/// Pass a slice that ends BEFORE any subsequent step-down (e.g.
/// `&output[..step_down]`), since the steady state is estimated from the
/// tail of the slice. See [`envelope_settle_seconds`] for the convention.
pub fn measure_attack_seconds(output: &[f64], step_sample: usize, sample_rate: f64) -> f64 {
    envelope_settle_seconds(output, step_sample, sample_rate)
}

/// Release time: seconds from a level step-down at `step_sample` until the
/// short-window RMS envelope settles within ±1 dB of its new steady state.
/// Same convention as [`measure_attack_seconds`].
pub fn measure_release_seconds(output: &[f64], step_sample: usize, sample_rate: f64) -> f64 {
    envelope_settle_seconds(output, step_sample, sample_rate)
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
