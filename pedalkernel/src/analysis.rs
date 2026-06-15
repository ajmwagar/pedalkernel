//! Pure-buffer audio analysis utilities.
//!
//! Provides Goertzel-based spectral analysis, THD measurement, RMS envelopes,
//! and health assertions for audio buffers. No external FFT crate — pure Rust math.
//!
//! # Feature gate
//!
//! This module is gated behind the `analysis` cargo feature, which is `std`-only
//! and **off by default**. This keeps it out of `no_std`/embedded builds (e.g.
//! Daisy targets) where the heap allocations and `std` types used here are
//! unavailable.
//!
//! Enable it with:
//!
//! ```toml
//! [dependencies]
//! pedalkernel = { version = "...", features = ["analysis"] }
//! ```
//!
//! # Example
//!
//! ```rust
//! use pedalkernel::analysis;
//!
//! let signal: Vec<f64> = (0..4800).map(|i| (i as f64 * 0.01).sin() * 0.5).collect();
//! assert!(analysis::rms(&signal) > 0.0);
//! ```

// ---------------------------------------------------------------------------
// Basic measurements
// ---------------------------------------------------------------------------

/// RMS level of a buffer.
///
/// Returns 0.0 for an empty buffer.
///
/// # Example
/// ```rust
/// assert!(pedalkernel::analysis::rms(&[1.0, -1.0]) > 0.0);
/// ```
pub fn rms(buf: &[f64]) -> f64 {
    if buf.is_empty() {
        return 0.0;
    }
    (buf.iter().map(|x| x * x).sum::<f64>() / buf.len() as f64).sqrt()
}

/// Peak absolute amplitude of a buffer.
pub fn peak(buf: &[f64]) -> f64 {
    buf.iter().fold(0.0f64, |m, x| m.max(x.abs()))
}

/// DC offset (mean) of a buffer.
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
///
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

/// THD: ratio of harmonic energy (harmonics 2–8) to fundamental energy.
///
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

/// Energy in frequency band \[lo_hz, hi_hz\] relative to total wideband energy.
///
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
///
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

/// Count subnormal (denormalized) floating point values in buffer.
pub fn count_subnormals(buf: &[f64]) -> usize {
    buf.iter().filter(|x| x.is_subnormal()).count()
}

// ---------------------------------------------------------------------------
// Envelope
// ---------------------------------------------------------------------------

/// Short-window RMS envelope (trailing window, same length as input).
///
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
