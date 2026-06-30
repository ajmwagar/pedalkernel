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

// ===========================================================================
// Steady-state, level-INDEPENDENT measurement
//
// `normalized_rms_error_db` is a DIFFERENCE metric: rms(wdf-ref)/rms(ref).  When
// the two levels differ by a factor k it computes |k-1|, NOT shape — and worse,
// when the WDF is far BELOW the golden (k→0) it reads ≈0 dB (because
// rms(wdf-ref) ≈ rms(ref)), falsely implying "shapes match" while the WDF is
// effectively silent.  These functions measure the magnitude of the test tone
// directly so the true gain gap is visible and is immune to any DC pedestal.
// ===========================================================================

/// Mean (DC offset) of a signal.  Use as the AC-coupling / settle guard: an
/// AC-coupled output that has reached steady state has |mean| ≈ 0.
pub fn dc_offset(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    signal.iter().sum::<f64>() / signal.len() as f64
}

/// Single-bin DFT amplitude (peak, in signal units) of the component at exactly
/// `freq_hz`.
///
/// Goertzel-style projection onto cos/sin at `freq_hz`.  The DC offset is
/// removed first, so the result is the AC amplitude AT THE TEST FREQUENCY and is
/// immune to any DC pedestal (a device sitting at its bias point) or slow drift
/// baseline.  This is the steady-state level measurement that
/// [`normalized_rms_error_db`] cannot provide.
pub fn single_bin_amplitude(signal: &[f64], freq_hz: f64, sample_rate: f64) -> f64 {
    let n = signal.len();
    if n == 0 || sample_rate <= 0.0 {
        return 0.0;
    }
    let mean = dc_offset(signal);
    let w = 2.0 * std::f64::consts::PI * freq_hz / sample_rate;
    let (mut re, mut im) = (0.0_f64, 0.0_f64);
    for (i, &x) in signal.iter().enumerate() {
        let a = w * i as f64;
        let v = x - mean;
        re += v * a.cos();
        im += v * a.sin();
    }
    2.0 * (re * re + im * im).sqrt() / n as f64
}

/// Steady-state, level-independent AC gain ratio in dB at `freq_hz`:
/// `20*log10(amp_wdf / amp_ref)`.  Positive = WDF louder than the reference.
///
/// Unlike [`normalized_rms_error_db`], this directly compares the magnitude of
/// the test tone, so it reports the TRUE level gap even when one signal rides on
/// a large DC bias or is far below the other.  Returns `-inf` when the reference
/// tone is silent.
pub fn ac_gain_db(wdf: &[f64], reference: &[f64], freq_hz: f64, sample_rate: f64) -> f64 {
    let aw = single_bin_amplitude(wdf, freq_hz, sample_rate);
    let ar = single_bin_amplitude(reference, freq_hz, sample_rate);
    if ar < 1e-30 {
        return f64::NEG_INFINITY;
    }
    20.0 * (aw / ar).log10()
}

/// AC-amplitude drift across a signal: dB difference between the single-bin
/// amplitude of the SECOND half vs the FIRST half of `signal`.
///
/// ≈0 dB means the tone has SETTLED (steady state).  A large magnitude means the
/// window is still inside a transient (e.g. a coupling cap still charging), so
/// any gain/shape number taken on it is unreliable.  This is the programmatic
/// guard that a measurement window is past the settling transient.
pub fn ac_amplitude_drift_db(signal: &[f64], freq_hz: f64, sample_rate: f64) -> f64 {
    let h = signal.len() / 2;
    if h == 0 {
        return 0.0;
    }
    let a1 = single_bin_amplitude(&signal[..h], freq_hz, sample_rate);
    let a2 = single_bin_amplitude(&signal[h..], freq_hz, sample_rate);
    if a1 < 1e-30 {
        return f64::INFINITY;
    }
    20.0 * (a2 / a1).log10()
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
pub fn thd_error_db(wdf: &[f64], reference: &[f64], fundamental_hz: f64, sample_rate: f64) -> f64 {
    let thd_wdf = thd_db(wdf, fundamental_hz, sample_rate, 10);
    let thd_ref = thd_db(reference, fundamental_hz, sample_rate, 10);
    (thd_wdf - thd_ref).abs()
}

/// Compute even/odd harmonic ratio in dB.
///
/// For push-pull amplifiers, even harmonics should be suppressed.
/// A good push-pull stage has ratio < -40 dB.
pub fn even_odd_ratio_db(
    signal: &[f64],
    fundamental_hz: f64,
    sample_rate: f64,
    n_harmonics: usize,
) -> f64 {
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
/// Compares spectra up to `max_freq_hz`, only at bins where **both** the
/// reference and WDF signals have significant energy.  The significance gate
/// has two components:
///
/// 1. **Relative gate** — bin must be within 80 dB of the reference peak.
/// 2. **Absolute floor** — both ref *and* WDF bins must be above −100 dBFS.
///
/// The absolute floor prevents false errors caused by comparing WDF's
/// pristine numerical noise floor (≈ −316 dB) against ngspice's broadband
/// numerical noise grass (≈ −95 dB).  Without the floor, a single weak
/// reference harmonic at −99 dBFS that the WDF places at −109 dBFS (below
/// the simulator floor) would contribute a ≈ 200 dB "error" even though the
/// real harmonics agree within 1–2 dB.  The floor at −100 dBFS sits 5 dB
/// *below* a typical 8-th harmonic of a well-simulated diode clipper
/// (≈ −80 dBFS) and 5 dB *above* ngspice's typical noise grass (≈ −95 dBFS),
/// so genuine circuit harmonics are still scored while simulator noise bins
/// are excluded.
pub fn spectral_error_db(
    wdf: &[f64],
    reference: &[f64],
    sample_rate: f64,
    max_freq_hz: Option<f64>,
) -> f64 {
    // Absolute noise-floor threshold: bins below this in *either* signal are
    // excluded from scoring.  Chosen to be above ngspice broadband noise
    // grass (≈ −95 dBFS) but well below real diode harmonics (≥ −80 dBFS).
    const ABS_FLOOR_DB: f64 = -100.0;

    let n = wdf.len().min(reference.len());
    if n == 0 {
        return 0.0;
    }

    let wdf_spec = compute_spectrum(&wdf[..n]);
    let ref_spec = compute_spectrum(&reference[..n]);

    let bin_hz = sample_rate / n as f64;
    let max_freq = max_freq_hz.unwrap_or(sample_rate / 4.0);
    let max_bin = ((max_freq / bin_hz) as usize).min(wdf_spec.len());

    // Relative threshold: within 80 dB of the reference peak.
    let ref_peak_db = ref_spec[..max_bin]
        .iter()
        .map(|&x| 20.0 * (x + 1e-30).log10())
        .fold(f64::NEG_INFINITY, f64::max);

    // Combined threshold: whichever is higher (less permissive).
    let threshold_db = (ref_peak_db - 80.0).max(ABS_FLOOR_DB);

    let mut max_error = 0.0_f64;

    for i in 1..max_bin {
        let ref_db = 20.0 * (ref_spec[i] + 1e-30).log10();
        // Gate 1 (relative + absolute): reference bin must be significant.
        if ref_db > threshold_db {
            let wdf_db = 20.0 * (wdf_spec[i] + 1e-30).log10();
            // Gate 2 (absolute): WDF bin must also be above the noise floor.
            // A WDF bin below ABS_FLOOR_DB compared to a reference bin just
            // above it is simulator-noise noise, not a circuit accuracy gap.
            if wdf_db > ABS_FLOOR_DB {
                let error = (wdf_db - ref_db).abs();
                max_error = max_error.max(error);
            }
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

/// Compute THD+N (Total Harmonic Distortion plus Noise) in dB.
///
/// THD+N = 10 * log10((total_power - fundamental_power) / fundamental_power)
///
/// Unlike [`thd_db`], which sums only discrete harmonic bins, THD+N includes
/// broadband noise and intermodulation products across ALL bins except the
/// fundamental.  Uses Blackman window for spectral analysis.
///
/// The fundamental exclusion zone is ±3 bins to prevent Blackman window
/// sidelobes from artificially inflating the noise estimate.
pub fn thd_plus_n_db(signal: &[f64], fundamental_hz: f64, sample_rate: f64) -> f64 {
    const FUND_EXCL_BINS: usize = 3;

    let spectrum = compute_spectrum(signal);
    let bin_hz = sample_rate / signal.len() as f64;

    let fund_bin = (fundamental_hz / bin_hz).round() as usize;
    if fund_bin >= spectrum.len() {
        return -200.0;
    }

    // Sum power in the exclusion zone around the fundamental to get
    // the reference fundamental power.
    let excl_lo = fund_bin.saturating_sub(FUND_EXCL_BINS);
    let excl_hi = (fund_bin + FUND_EXCL_BINS + 1).min(spectrum.len());
    let fund_power: f64 = spectrum[excl_lo..excl_hi].iter().map(|&x| x.powi(2)).sum();

    if fund_power < 1e-30 {
        return -200.0;
    }

    // Total power over all bins.
    let total_power: f64 = spectrum.iter().map(|&x| x.powi(2)).sum();
    // Noise+harmonic power = total minus the fundamental exclusion zone.
    let noise_power = total_power - fund_power;

    if noise_power <= 0.0 {
        return -200.0;
    }

    10.0 * (noise_power / fund_power).log10()
}

/// Compute THD+N error between WDF and reference in dB.
///
/// Returns the absolute difference in THD+N between the two signals.
pub fn thd_plus_n_error_db(
    wdf: &[f64],
    reference: &[f64],
    fundamental_hz: f64,
    sample_rate: f64,
) -> f64 {
    let wdf_tpn = thd_plus_n_db(wdf, fundamental_hz, sample_rate);
    let ref_tpn = thd_plus_n_db(reference, fundamental_hz, sample_rate);
    (wdf_tpn - ref_tpn).abs()
}

/// Compute the maximum harmonic magnitude error between WDF and reference in dB.
///
/// For each harmonic h in 2..=n_harmonics, reads both spectra at the h-th
/// harmonic bin and computes `|20*log10(wdf_h) - 20*log10(ref_h)|`.  Returns
/// the MAX error over harmonics where **both** bins exceed the −100 dBFS
/// absolute floor (same gate as [`spectral_error_db`]).
///
/// This is a targeted complement to [`spectral_error_db`]: it scores only
/// the discrete harmonic bins, making it meaningful at high-drive levels and
/// immune to broadband noise grass between harmonics.
pub fn harmonic_mag_error_db(
    wdf: &[f64],
    reference: &[f64],
    fundamental_hz: f64,
    sample_rate: f64,
    n_harmonics: usize,
) -> f64 {
    const ABS_FLOOR_DB: f64 = -100.0;

    let n = wdf.len().min(reference.len());
    if n == 0 {
        return 0.0;
    }

    let wdf_spec = compute_spectrum(&wdf[..n]);
    let ref_spec = compute_spectrum(&reference[..n]);
    let bin_hz = sample_rate / n as f64;

    let mut max_error = 0.0_f64;

    for h in 2..=n_harmonics {
        let harm_freq = fundamental_hz * h as f64;
        if harm_freq > sample_rate / 2.0 {
            break;
        }
        let harm_bin = (harm_freq / bin_hz).round() as usize;
        if harm_bin >= wdf_spec.len() || harm_bin >= ref_spec.len() {
            continue;
        }

        let wdf_db = 20.0 * (wdf_spec[harm_bin] + 1e-30).log10();
        let ref_db = 20.0 * (ref_spec[harm_bin] + 1e-30).log10();

        // Only score bins where both signals are above the noise floor.
        if wdf_db > ABS_FLOOR_DB && ref_db > ABS_FLOOR_DB {
            let error = (wdf_db - ref_db).abs();
            max_error = max_error.max(error);
        }
    }

    max_error
}

/// Compute even/odd harmonic ratio error between WDF and reference in dB.
///
/// Returns the absolute difference in [`even_odd_ratio_db`] between the two
/// signals.  Near zero for two signals with the same harmonic character;
/// large when one has asymmetric and the other symmetric distortion.
pub fn even_odd_ratio_error_db(
    wdf: &[f64],
    reference: &[f64],
    fundamental_hz: f64,
    sample_rate: f64,
    n_harmonics: usize,
) -> f64 {
    let wdf_ratio = even_odd_ratio_db(wdf, fundamental_hz, sample_rate, n_harmonics);
    let ref_ratio = even_odd_ratio_db(reference, fundamental_hz, sample_rate, n_harmonics);
    (wdf_ratio - ref_ratio).abs()
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
    /// THD+N error between WDF and reference in dB.  None when fundamental_hz
    /// is not provided.
    pub thd_plus_n_error_db: Option<f64>,
    /// Maximum per-harmonic magnitude error in dB.  None when fundamental_hz
    /// is not provided.
    pub harmonic_mag_error_db: Option<f64>,
    /// Even/odd ratio error between WDF and reference in dB.  None when
    /// fundamental_hz is not provided.
    pub even_odd_ratio_error_db: Option<f64>,
    /// Spectral error within the audio band (capped at the audio Nyquist by the
    /// production runners). This is the value checked against pass criteria.
    pub spectral_error_db: f64,
    /// Spectral error across the full data spectrum (up to sample_rate / 2, i.e.
    /// the oversampled Nyquist). Informational only — shows what the audio-band
    /// cap removes (ultrasonic content the engine anti-aliases away).
    pub spectral_error_full_db: f64,
    pub even_odd_ratio_db: Option<f64>,
    pub dc_drift_mv: Option<f64>,
}

impl ComparisonResult {
    /// Check if all metrics pass the given criteria.
    pub fn passes(&self, criteria: &crate::config::PassCriteria) -> bool {
        if self.normalized_rms_error_db > criteria.normalized_rms_error_db.unwrap_or(f64::INFINITY)
        {
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
        if let (Some(v), Some(t)) = (self.thd_plus_n_error_db, criteria.thd_plus_n_error_db) {
            if v > t {
                return false;
            }
        }
        if let (Some(v), Some(t)) = (self.harmonic_mag_error_db, criteria.harmonic_mag_error_db) {
            if v > t {
                return false;
            }
        }
        if let (Some(v), Some(t)) = (
            self.even_odd_ratio_error_db,
            criteria.even_odd_ratio_error_db,
        ) {
            if v > t {
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
    let tpn_err =
        fundamental_hz.map(|f| thd_plus_n_error_db(wdf, reference, f, sample_rate));
    let harm_mag_err =
        fundamental_hz.map(|f| harmonic_mag_error_db(wdf, reference, f, sample_rate, 10));
    let eo_ratio_err =
        fundamental_hz.map(|f| even_odd_ratio_error_db(wdf, reference, f, sample_rate, 10));
    let even_odd = fundamental_hz.map(|f| even_odd_ratio_db(wdf, f, sample_rate, 10));

    ComparisonResult {
        normalized_rms_error_db: normalized_rms_error_db(wdf, reference),
        peak_error_db: peak_error_db(wdf, reference),
        thd_error_db: thd_err,
        thd_plus_n_error_db: tpn_err,
        harmonic_mag_error_db: harm_mag_err,
        even_odd_ratio_error_db: eo_ratio_err,
        spectral_error_db: spectral_error_db(wdf, reference, sample_rate, None),
        // Full-band (raw) spectral error up to the data Nyquist. The production
        // runners overwrite `spectral_error_db` with the audio-band cap, but
        // leave this raw value intact for display.
        spectral_error_full_db: spectral_error_db(
            wdf,
            reference,
            sample_rate,
            Some(sample_rate / 2.0),
        ),
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
    fn single_bin_amplitude_recovers_tone_level() {
        let sr = 48000.0;
        let amp = 0.7;
        let sig: Vec<f64> = (0..48000)
            .map(|i| amp * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / sr).sin())
            .collect();
        let a = single_bin_amplitude(&sig, 1000.0, sr);
        assert!((a - amp).abs() < 1e-3, "expected ~{amp}, got {a}");
    }

    #[test]
    fn single_bin_amplitude_is_immune_to_dc_pedestal() {
        // A device output sitting on a +5 V DC bias with a small AC tone.
        let sr = 48000.0;
        let ac = 0.05;
        let sig: Vec<f64> = (0..48000)
            .map(|i| 5.0 + ac * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / sr).sin())
            .collect();
        let a = single_bin_amplitude(&sig, 1000.0, sr);
        assert!((a - ac).abs() < 1e-3, "DC pedestal must not affect AC amplitude; got {a}");
    }

    /// The core point of the new metric: when the WDF is far BELOW the golden,
    /// `normalized_rms_error_db` DEGENERATES to ~0 dB ("shape matches") while
    /// `ac_gain_db` correctly reports the true ~-40 dB level gap.
    #[test]
    fn ac_gain_db_exposes_what_normalized_rms_hides() {
        let sr = 48000.0;
        let golden: Vec<f64> = (0..48000)
            .map(|i| (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / sr).sin())
            .collect();
        // WDF is the same tone at 1% of the level (≈ -40 dB), e.g. an under-driven
        // input port.
        let wdf: Vec<f64> = golden.iter().map(|&x| 0.01 * x).collect();

        let nrms = normalized_rms_error_db(&wdf, &golden);
        let gain = ac_gain_db(&wdf, &golden, 1000.0, sr);

        // normalized RMS reads ~0 dB (degenerate: rms(wdf-ref) ≈ rms(ref)).
        assert!(
            nrms.abs() < 1.0,
            "normalized_rms_error_db is degenerate here (expected ~0 dB), got {nrms:.2}"
        );
        // ac_gain_db reports the real gap: 0.01 → -40 dB.
        assert!(
            (gain + 40.0).abs() < 0.5,
            "ac_gain_db must report the true -40 dB gap, got {gain:.2}"
        );
    }

    #[test]
    fn ac_gain_db_doubling_is_plus_six_db_regardless_of_dc() {
        let sr = 48000.0;
        let golden: Vec<f64> = (0..48000)
            .map(|i| 0.1 * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / sr).sin())
            .collect();
        // WDF: double the AC amplitude AND add an unrelated DC bias.
        let wdf: Vec<f64> = golden.iter().map(|&x| 3.3 + 2.0 * x).collect();
        let gain = ac_gain_db(&wdf, &golden, 1000.0, sr);
        assert!((gain - 6.02).abs() < 0.1, "2x AC → +6 dB, got {gain:.2}");
    }

    #[test]
    fn ac_amplitude_drift_zero_for_steady_large_for_ramp() {
        let sr = 48000.0;
        // Steady tone → ~0 dB drift.
        let steady: Vec<f64> = (0..48000)
            .map(|i| (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / sr).sin())
            .collect();
        assert!(
            ac_amplitude_drift_db(&steady, 1000.0, sr).abs() < 0.2,
            "steady tone must show ~0 dB drift"
        );
        // Amplitude ramp (transient): envelope grows from 0.1 to 1.0 over the
        // window → second half is much louder than the first.
        let n = 48000usize;
        let ramp: Vec<f64> = (0..n)
            .map(|i| {
                let env = 0.1 + 0.9 * (i as f64 / n as f64);
                env * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / sr).sin()
            })
            .collect();
        let drift = ac_amplitude_drift_db(&ramp, 1000.0, sr);
        assert!(drift > 3.0, "ramping (unsettled) tone must show large drift, got {drift:.2}");
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

    /// Verify that the noise-floor gate does not cause spurious large spectral
    /// errors when matching harmonics agree but one signal has a lower noise
    /// floor than the other (the "false 218 dB" single_diode scenario).
    ///
    /// Why this matters: ngspice's broadband numerical noise grass sits at
    /// roughly −95 dBFS.  The WDF engine's numerical floor is much lower
    /// (≈ −316 dBFS).  Without the absolute floor gate, bins in the ref that
    /// are just above ref_peak−80 dB (e.g. −99 dBFS) get compared against WDF
    /// bins at −316 dBFS, yielding ≈ 220 dB "error" — even though the real
    /// harmonics agree.  The fix requires both ref and WDF to be above
    /// −100 dBFS before scoring the bin's error.
    #[test]
    fn spectral_error_ignores_noise_floor_mismatch() {
        let sr = 48000.0;
        let n = sr as usize; // 1 second
        let freq = 1000.0_f64;

        // Reference: sine with the first several harmonics (simulating a
        // nonlinear SPICE output with noise grass at −95 dBFS).
        let ref_signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sr;
                let x = (2.0 * std::f64::consts::PI * freq * t).sin();
                // Simulate ngspice noise grass: 1e-5 amplitude ≈ −100 dBFS
                let noise_floor = 1e-5 * ((i as f64 * 7.13).sin());
                x + noise_floor
            })
            .collect();

        // WDF: same sine at same level but with effectively zero noise floor
        // (pristine numerical output — no added noise grass).
        let wdf_signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sr;
                (2.0 * std::f64::consts::PI * freq * t).sin()
            })
            .collect();

        let err = spectral_error_db(&wdf_signal, &ref_signal, sr, None);

        // The two signals are identical except for noise grass below −100 dBFS.
        // The gate must exclude those bins, so the error must be small (< 3 dB).
        // Pre-fix this would report ≈ 100–200 dB because WDF's −316 dBFS bins
        // were scored against ref's −100 dBFS noise grass.
        assert!(
            err < 3.0,
            "Noise-floor mismatch should produce < 3 dB spectral error, got {err:.1} dB. \
             Check that the absolute noise-floor gate in spectral_error_db is working."
        );
    }

    /// THD+N of a hard-clipped sine must be much higher than a clean sine.
    ///
    /// Why this matters: THD+N includes broadband noise and intermod across all
    /// bins, so a hard clipper that produces strong harmonics AND intermod
    /// noise should score significantly higher than a pure sine.
    #[test]
    fn thd_plus_n_clipped_sine_much_higher_than_clean() {
        let sr = 48000.0;
        let clean: Vec<f64> = (0..48000)
            .map(|i| (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / sr).sin())
            .collect();
        let clipped: Vec<f64> = clean.iter().map(|&x| x.clamp(-0.3, 0.3)).collect();

        let clean_tpn = thd_plus_n_db(&clean, 1000.0, sr);
        let clipped_tpn = thd_plus_n_db(&clipped, 1000.0, sr);

        assert!(
            clipped_tpn > clean_tpn + 10.0,
            "Clipped sine THD+N ({clipped_tpn:.1} dB) should be >10 dB higher than clean ({clean_tpn:.1} dB)"
        );
    }

    /// harmonic_mag_error_db ~0 when wdf == ref, large when one harmonic differs by ~20 dB.
    ///
    /// Why this matters: the metric must be sensitive to per-harmonic level
    /// mismatches that matter for circuit accuracy but immune to noise in bins
    /// where both signals are below −100 dBFS.
    #[test]
    fn harmonic_mag_error_zero_for_identical_large_for_mismatch() {
        let sr = 48000.0;
        let n = 48000_usize;
        let f = 1000.0_f64;

        // Shared base: fundamental + three harmonics at known levels.
        let base: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sr;
                (2.0 * std::f64::consts::PI * f * t).sin()
                    + 0.1 * (2.0 * std::f64::consts::PI * 2.0 * f * t).sin()
                    + 0.05 * (2.0 * std::f64::consts::PI * 3.0 * f * t).sin()
            })
            .collect();

        // Identical signals → error ~0.
        let err_same = harmonic_mag_error_db(&base, &base, f, sr, 10);
        assert!(
            err_same < 1.0,
            "Identical signals should give ~0 dB harmonic_mag_error, got {err_same:.2} dB"
        );

        // WDF with 2nd harmonic reduced by ~20 dB (0.1 → 0.01).
        let wdf_altered: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sr;
                (2.0 * std::f64::consts::PI * f * t).sin()
                    + 0.01 * (2.0 * std::f64::consts::PI * 2.0 * f * t).sin()
                    + 0.05 * (2.0 * std::f64::consts::PI * 3.0 * f * t).sin()
            })
            .collect();

        let err_altered = harmonic_mag_error_db(&wdf_altered, &base, f, sr, 10);
        assert!(
            err_altered >= 15.0,
            "~20 dB h2 level change should give >=15 dB harmonic_mag_error, got {err_altered:.2} dB"
        );
    }

    /// even_odd_ratio_error_db ~0 for identical signals, large when symmetry differs.
    ///
    /// Why this matters: push-pull vs single-ended topologies differ in their
    /// even/odd balance; this metric captures that difference.
    #[test]
    fn even_odd_ratio_error_zero_same_large_when_symmetry_differs() {
        let sr = 48000.0;
        let n = 48000_usize;
        let f = 1000.0_f64;

        // Asymmetric distortion: strong even harmonics (characteristic of single-ended).
        let asymmetric: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sr;
                (2.0 * std::f64::consts::PI * f * t).sin()
                    + 0.3 * (2.0 * std::f64::consts::PI * 2.0 * f * t).sin()
                    + 0.05 * (2.0 * std::f64::consts::PI * 3.0 * f * t).sin()
            })
            .collect();

        // Symmetric distortion: strong odd harmonics only (characteristic of push-pull).
        let symmetric: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sr;
                (2.0 * std::f64::consts::PI * f * t).sin()
                    + 0.01 * (2.0 * std::f64::consts::PI * 2.0 * f * t).sin()
                    + 0.3 * (2.0 * std::f64::consts::PI * 3.0 * f * t).sin()
            })
            .collect();

        // Identical pair → error ~0.
        let err_same = even_odd_ratio_error_db(&asymmetric, &asymmetric, f, sr, 10);
        assert!(
            err_same < 1.0,
            "Identical signals should give ~0 dB even_odd_ratio_error, got {err_same:.2} dB"
        );

        // Asymmetric vs symmetric → error must be large.
        let err_diff = even_odd_ratio_error_db(&symmetric, &asymmetric, f, sr, 10);
        assert!(
            err_diff > 10.0,
            "Asymmetric vs symmetric should give >10 dB even_odd_ratio_error, got {err_diff:.2} dB"
        );
    }

    /// Verify that a genuine harmonic level mismatch still scores a large spectral
    /// error even after the noise-floor fix (regression guard for the fix).
    ///
    /// Why this matters: the absolute floor at −100 dBFS should only exclude
    /// bins where *both* signals are near the noise floor.  When both ref and
    /// WDF have strong, above-floor content at the same harmonic but at
    /// significantly different levels, the error must still score large.
    #[test]
    fn spectral_error_detects_real_harmonic_divergence() {
        let sr = 48000.0;
        let n = sr as usize;
        let freq = 1000.0_f64;

        // Reference: fundamental + strong 2nd harmonic at −20 dBFS (amplitude 0.1).
        let ref_signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sr;
                let h1 = (2.0 * std::f64::consts::PI * freq * t).sin();
                let h2 = 0.1 * (2.0 * std::f64::consts::PI * 2.0 * freq * t).sin();
                h1 + h2
            })
            .collect();

        // WDF: fundamental + 2nd harmonic 30 dB lower than ref (a real accuracy gap
        // where both signals have measurable content above the noise floor).
        // WDF h2 amplitude ≈ 0.1 * 10^(-30/20) ≈ 0.0032.
        let wdf_signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sr;
                let h1 = (2.0 * std::f64::consts::PI * freq * t).sin();
                let h2 = 0.0032 * (2.0 * std::f64::consts::PI * 2.0 * freq * t).sin();
                h1 + h2
            })
            .collect();

        let err = spectral_error_db(&wdf_signal, &ref_signal, sr, None);

        // The 2nd harmonic differs by ~30 dB between ref and WDF, and both
        // are above the −100 dBFS noise floor, so the error must score large.
        assert!(
            err > 20.0,
            "30 dB h2 level gap should produce > 20 dB spectral error, got {err:.1} dB."
        );
    }
}
