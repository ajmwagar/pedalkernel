//! Audio signal comparison metrics.
//!
//! This module provides functions for comparing audio signals and measuring
//! various quality metrics commonly used in audio circuit validation.
//!
//! # Available Metrics
//!
//! | Metric | Function | Description |
//! |--------|----------|-------------|
//! | Normalized RMS Error | [`normalized_rms_error_db`] | Overall signal difference (lower is better) |
//! | Peak Error | [`peak_error_db`] | Maximum sample-level difference |
//! | THD | [`thd_db`] | Total Harmonic Distortion |
//! | THD Error | [`thd_error_db`] | THD difference between two signals |
//! | Even/Odd Ratio | [`even_odd_ratio_db`] | Even vs odd harmonic balance |
//! | Spectral Error | [`spectral_error_db`] | Frequency domain difference |
//! | DC Drift | [`dc_drift_mv`] | Low-frequency offset measurement |
//!
//! # Example
//!
//! ```rust
//! use pedalkernel_validate::metrics;
//!
//! let wdf_output: Vec<f64> = vec![0.1, 0.2, 0.3, 0.2, 0.1];
//! let reference: Vec<f64> = vec![0.1, 0.2, 0.3, 0.2, 0.1];
//! let sample_rate = 48000.0;
//!
//! // Compare individual metrics
//! let rms_err = metrics::normalized_rms_error_db(&wdf_output, &reference);
//! let peak_err = metrics::peak_error_db(&wdf_output, &reference);
//!
//! // Or use the combined comparison function
//! let result = metrics::compare(&wdf_output, &reference, sample_rate, Some(1000.0));
//! println!("RMS error: {:.1} dB", result.normalized_rms_error_db);
//! println!("Peak error: {:.1} dB", result.peak_error_db);
//! ```
//!
//! # Interpreting Results
//!
//! - **RMS Error**: -60 dB means error is 0.1% of signal. -80 dB is excellent.
//! - **Peak Error**: Maximum instantaneous difference. Usually higher than RMS.
//! - **THD**: Pure sine should be < -80 dB. Audible distortion starts around -40 dB.
//! - **Even/Odd Ratio**: Push-pull amps should have < -40 dB (even harmonics suppressed)

use realfft::RealFftPlanner;
use rustfft::num_complex::Complex;

/// Compute normalized RMS error in dB.
///
/// `error_db = 20 * log10(rms(wdf - ref) / rms(ref))`
///
/// Lower is better. -60 dB means error is 0.1% of signal.
pub fn normalized_rms_error_db(wdf: &[f64], reference: &[f64]) -> f64 {
    let n = wdf.len().min(reference.len());
    if n == 0 {
        return f64::NEG_INFINITY;
    }

    let mut err_sum = 0.0;
    let mut ref_sum = 0.0;

    for i in 0..n {
        let diff = wdf[i] - reference[i];
        err_sum += diff * diff;
        ref_sum += reference[i] * reference[i];
    }

    let err_rms = (err_sum / n as f64).sqrt();
    let ref_rms = (ref_sum / n as f64).sqrt();

    if ref_rms < 1e-30 {
        return -200.0;
    }

    20.0 * (err_rms / ref_rms).log10()
}

/// Compute peak absolute error in dB relative to reference peak.
///
/// `error_db = 20 * log10(max|wdf - ref| / max|ref|)`
pub fn peak_error_db(wdf: &[f64], reference: &[f64]) -> f64 {
    let n = wdf.len().min(reference.len());
    if n == 0 {
        return f64::NEG_INFINITY;
    }

    let mut max_err = 0.0_f64;
    let mut max_ref = 0.0_f64;

    for i in 0..n {
        max_err = max_err.max((wdf[i] - reference[i]).abs());
        max_ref = max_ref.max(reference[i].abs());
    }

    if max_ref < 1e-30 {
        return -200.0;
    }

    20.0 * (max_err / max_ref).log10()
}

/// Compute THD (Total Harmonic Distortion) in dB.
///
/// THD = 10 * log10(sum(harmonic_powers) / fundamental_power)
///
/// Uses Blackman window for spectral analysis.
pub fn thd_db(signal: &[f64], fundamental_hz: f64, sample_rate: f64, n_harmonics: usize) -> f64 {
    let spectrum = compute_spectrum(signal);
    let bin_hz = sample_rate / signal.len() as f64;

    // Find fundamental bin
    let fund_bin = (fundamental_hz / bin_hz).round() as usize;
    if fund_bin >= spectrum.len() {
        return -200.0;
    }

    let fund_power = spectrum[fund_bin].powi(2);
    if fund_power < 1e-30 {
        return -200.0;
    }

    // Sum harmonic powers
    let mut harm_power = 0.0;
    for h in 2..=n_harmonics {
        let harm_freq = fundamental_hz * h as f64;
        if harm_freq > sample_rate / 2.0 {
            break;
        }
        let harm_bin = (harm_freq / bin_hz).round() as usize;
        if harm_bin < spectrum.len() {
            harm_power += spectrum[harm_bin].powi(2);
        }
    }

    10.0 * (harm_power / fund_power).log10()
}

/// Compute THD error between WDF and reference in dB.
pub fn thd_error_db(
    wdf: &[f64],
    reference: &[f64],
    fundamental_hz: f64,
    sample_rate: f64,
) -> f64 {
    let thd_wdf = thd_db(wdf, fundamental_hz, sample_rate, 10);
    let thd_ref = thd_db(reference, fundamental_hz, sample_rate, 10);
    (thd_wdf - thd_ref).abs()
}

/// Compute even/odd harmonic ratio in dB.
///
/// For push-pull amplifiers, even harmonics should be suppressed.
/// A good push-pull stage has ratio < -40 dB.
pub fn even_odd_ratio_db(signal: &[f64], fundamental_hz: f64, sample_rate: f64, n_harmonics: usize) -> f64 {
    let spectrum = compute_spectrum(signal);
    let bin_hz = sample_rate / signal.len() as f64;

    let mut even_power = 0.0;
    let mut odd_power = 0.0;

    for h in 2..=n_harmonics {
        let harm_freq = fundamental_hz * h as f64;
        if harm_freq > sample_rate / 2.0 {
            break;
        }
        let harm_bin = (harm_freq / bin_hz).round() as usize;
        if harm_bin < spectrum.len() {
            let power = spectrum[harm_bin].powi(2);
            if h % 2 == 0 {
                even_power += power;
            } else {
                odd_power += power;
            }
        }
    }

    if odd_power < 1e-30 {
        return 0.0;
    }

    10.0 * (even_power / odd_power).log10()
}

/// Compute maximum spectral magnitude error in dB.
///
/// Compares spectra up to `max_freq_hz`, only at bins where
/// reference has significant energy (within 80dB of peak).
pub fn spectral_error_db(
    wdf: &[f64],
    reference: &[f64],
    sample_rate: f64,
    max_freq_hz: Option<f64>,
) -> f64 {
    let n = wdf.len().min(reference.len());
    if n == 0 {
        return 0.0;
    }

    let wdf_spec = compute_spectrum(&wdf[..n]);
    let ref_spec = compute_spectrum(&reference[..n]);

    let bin_hz = sample_rate / n as f64;
    let max_freq = max_freq_hz.unwrap_or(sample_rate / 4.0);
    let max_bin = ((max_freq / bin_hz) as usize).min(wdf_spec.len());

    // Find reference peak for significance threshold
    let ref_peak_db = ref_spec[..max_bin]
        .iter()
        .map(|&x| 20.0 * (x + 1e-30).log10())
        .fold(f64::NEG_INFINITY, f64::max);

    let threshold_db = ref_peak_db - 80.0;

    let mut max_error = 0.0_f64;

    for i in 1..max_bin {
        let ref_db = 20.0 * (ref_spec[i] + 1e-30).log10();
        if ref_db > threshold_db {
            let wdf_db = 20.0 * (wdf_spec[i] + 1e-30).log10();
            let error = (wdf_db - ref_db).abs();
            max_error = max_error.max(error);
        }
    }

    max_error
}

/// Measure DC drift over time.
///
/// Returns the maximum DC offset observed (using a moving average).
pub fn dc_drift_mv(signal: &[f64], sample_rate: f64, window_ms: f64) -> f64 {
    let window_samples = (window_ms * sample_rate / 1000.0) as usize;
    if window_samples == 0 || signal.len() < window_samples {
        return 0.0;
    }

    let mut max_dc = 0.0_f64;
    let mut running_sum: f64 = signal[..window_samples].iter().sum();

    for i in window_samples..signal.len() {
        let dc = running_sum / window_samples as f64;
        max_dc = max_dc.max(dc.abs());
        running_sum += signal[i] - signal[i - window_samples];
    }

    max_dc * 1000.0 // Convert to mV
}

/// Compute magnitude spectrum using Blackman window.
fn compute_spectrum(signal: &[f64]) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return vec![];
    }

    // Apply Blackman window
    let mut windowed: Vec<f64> = signal
        .iter()
        .enumerate()
        .map(|(i, &x)| {
            let w = 0.42 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / (n - 1) as f64).cos()
                + 0.08 * (4.0 * std::f64::consts::PI * i as f64 / (n - 1) as f64).cos();
            x * w
        })
        .collect();

    // Perform real FFT
    let mut planner = RealFftPlanner::<f64>::new();
    let fft = planner.plan_fft_forward(n);

    let mut spectrum = vec![Complex::new(0.0, 0.0); n / 2 + 1];
    fft.process(&mut windowed, &mut spectrum).unwrap();

    // Return magnitudes
    spectrum.iter().map(|c| c.norm() / n as f64).collect()
}

/// Result of comparing WDF output to reference.
#[derive(Debug, Clone)]
pub struct ComparisonResult {
    pub normalized_rms_error_db: f64,
    pub peak_error_db: f64,
    pub thd_error_db: Option<f64>,
    pub spectral_error_db: f64,
    pub even_odd_ratio_db: Option<f64>,
    pub dc_drift_mv: Option<f64>,
}

impl ComparisonResult {
    /// Check if all metrics pass the given criteria.
    pub fn passes(&self, criteria: &crate::config::PassCriteria) -> bool {
        if self.normalized_rms_error_db > criteria.normalized_rms_error_db.unwrap_or(f64::INFINITY) {
            return false;
        }
        if self.peak_error_db > criteria.peak_error_db.unwrap_or(f64::INFINITY) {
            return false;
        }
        if let (Some(thd_err), Some(thresh)) = (self.thd_error_db, criteria.thd_error_db) {
            if thd_err > thresh {
                return false;
            }
        }
        if self.spectral_error_db > criteria.spectral_error_db.unwrap_or(f64::INFINITY) {
            return false;
        }
        if let (Some(dc), Some(thresh)) = (self.dc_drift_mv, criteria.max_dc_drift_mv) {
            if dc > thresh {
                return false;
            }
        }
        true
    }
}

/// Compare WDF output to reference with all metrics.
pub fn compare(
    wdf: &[f64],
    reference: &[f64],
    sample_rate: f64,
    fundamental_hz: Option<f64>,
) -> ComparisonResult {
    let thd_err = fundamental_hz.map(|f| thd_error_db(wdf, reference, f, sample_rate));
    let even_odd = fundamental_hz.map(|f| even_odd_ratio_db(wdf, f, sample_rate, 10));

    ComparisonResult {
        normalized_rms_error_db: normalized_rms_error_db(wdf, reference),
        peak_error_db: peak_error_db(wdf, reference),
        thd_error_db: thd_err,
        spectral_error_db: spectral_error_db(wdf, reference, sample_rate, None),
        even_odd_ratio_db: even_odd,
        dc_drift_mv: Some(dc_drift_mv(wdf, sample_rate, 100.0)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identical_signals_have_zero_error() {
        let sig: Vec<f64> = (0..1000).map(|i| (i as f64 * 0.1).sin()).collect();
        assert!(normalized_rms_error_db(&sig, &sig) < -100.0);
        assert!(peak_error_db(&sig, &sig) < -100.0);
    }

    #[test]
    fn thd_of_pure_sine_is_very_low() {
        let sr = 48000.0;
        let sig: Vec<f64> = (0..48000)
            .map(|i| (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / sr).sin())
            .collect();
        let thd = thd_db(&sig, 1000.0, sr, 10);
        assert!(thd < -60.0, "Pure sine THD should be < -60dB, got {thd}");
    }

    #[test]
    fn clipped_sine_has_high_thd() {
        let sr = 48000.0;
        let sig: Vec<f64> = (0..48000)
            .map(|i| {
                let x = (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / sr).sin();
                x.clamp(-0.5, 0.5) // Hard clip at ±0.5
            })
            .collect();
        let thd = thd_db(&sig, 1000.0, sr, 10);
        assert!(thd > -20.0, "Clipped sine THD should be > -20dB, got {thd}");
    }
}
