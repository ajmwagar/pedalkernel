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

// ============================================================================
// DC operating-point (bias) accuracy metric
// ============================================================================
//
// This module adds the missing *primary* validation axis for active circuits:
// does our compile-time/settled DC bias agree with ngspice's `.op`? It answers
// two distinct questions that map to the literature's Layer A / Layer B
// (`docs/dc-operating-point-solve.md`):
//
//   1. **MATCH** (Layer B): per-device ΔVbe / ΔVce / ΔIc between OUR settled DC
//      operating point and ngspice's `.op`. A large MATCH error with a small
//      RESIDUAL is a *solver / root-selection* problem — our equations are right
//      but the cold solve lands in the wrong basin.
//
//   2. **RESIDUAL** (Layer A): `F(ngspice_op)` — is ngspice's `.op` a zero of OUR
//      DC equations? A large residual is a *formulation* bug — our bias math
//      genuinely differs from spice, independent of which root we converge to.
//
// The metric is plain-numeric so it stays decoupled from `pedalkernel-rt` (which
// is a dev-dependency only): the test harness reads the engine's per-device bias
// and per-port residual and feeds them in here.
pub mod op_point {
    use crate::spice::SpiceDeviceOp;

    /// Our (WDF) DC operating point for a single device, read at the engine's
    /// settled bias. Sign convention is the engine's *port* convention; the
    /// comparison normalizes against ngspice's junction-magnitude convention.
    #[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
    pub struct DeviceBias {
        /// Component reference, uppercased (e.g. `Q1`).
        pub reference: String,
        /// Base–emitter port voltage, volts.
        pub vbe: f64,
        /// Collector–emitter port voltage, volts.
        pub vce: f64,
        /// Collector current, amps.
        pub ic: f64,
    }

    /// Per-device MATCH comparison (ours vs ngspice).
    #[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
    pub struct DeviceMatch {
        pub reference: String,
        pub model: String,
        /// Our reading (Vbe, Vce, Ic).
        pub ours: (f64, f64, f64),
        /// ngspice `.op` (Vbe, Vce, Ic).
        pub spice: (f64, f64, f64),
        /// ΔVbe = ours − spice (volts), compared on junction magnitude.
        pub d_vbe: f64,
        /// ΔVce = ours − spice (volts), compared on magnitude.
        pub d_vce: f64,
        /// ΔIc as a percentage of |spice Ic| (∞-guarded: 0 when spice Ic ≈ 0).
        pub d_ic_pct: f64,
    }

    /// Per-port DC-balance residual `F(ngspice_op)` (from
    /// `MultiNlStage::op_seed_residual`, fed in as plain numbers).
    #[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
    pub struct PortResidual {
        /// Static formulation residual `a − dc_bias − cap − nl − adapt` (volts).
        pub residual: f64,
        /// Full residual (includes the runtime DC servo); ≈ 0 at the engine's own
        /// fixed point — calibrates the probe.
        pub residual_full: f64,
    }

    /// Thresholds for grading the bias-accuracy dimension. Informative, not
    /// punitive — tuned to localize Layer A vs Layer B, not to gate CI.
    #[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
    pub struct OpPassCriteria {
        /// ΔVbe warn threshold (volts). Default 10 mV.
        pub vbe_warn_v: f64,
        /// ΔVbe fail threshold (volts). Default 25 mV.
        pub vbe_fail_v: f64,
        /// ΔVce fail threshold (volts). Default 0.3 V.
        pub vce_fail_v: f64,
        /// ΔIc fail threshold (percent). Default 10 %.
        pub ic_fail_pct: f64,
        /// Static-residual threshold separating "formulation ≈ ok" from a
        /// formulation bug (volts). Default 0.25 V.
        ///
        /// This is a DC-balance error in volts at a device port. Once the reactive
        /// (capacitor) ports are seeded to ngspice's `.op` node voltages — the FULL
        /// operating-point seed, not the old cold-cap probe — a residual on the
        /// order of a junction drop means ngspice's `.op` is genuinely NOT a fixed
        /// point of our DC equations (a formulation bug, Layer A). The BA283 lands
        /// at ≈ 0.615 V (Q3 Vbe dominant) → Layer A. Tens-of-mV residuals still read
        /// "≈ ok" → Layer B (solver / root-selection). The prior 1.0 V default was
        /// set against a CONTAMINATED, cold-cap residual (≈ 0.04 V) and mis-called
        /// the BA283 Layer B; with the caps seeded the true 0.615 V exceeds this.
        pub residual_formulation_v: f64,
    }

    impl Default for OpPassCriteria {
        fn default() -> Self {
            Self {
                vbe_warn_v: 0.010,
                vbe_fail_v: 0.025,
                vce_fail_v: 0.3,
                ic_fail_pct: 10.0,
                residual_formulation_v: 0.25,
            }
        }
    }

    /// The formulation-vs-solver verdict for one circuit.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
    pub enum Verdict {
        /// Residual ≈ 0 AND MATCH within thresholds — bias is right.
        BiasOk,
        /// Residual ≈ 0 but MATCH off — ngspice's `.op` IS a fixed point of our
        /// equations, but the cold solve converged to a different root.
        /// (Layer B: solver / root-selection.)
        FormulationOkSolverOff,
        /// Residual far from 0 — ngspice's `.op` is NOT a fixed point of our
        /// equations. (Layer A: formulation bug.)
        FormulationBug,
        /// No active devices / no residual ports — nothing to grade.
        NoData,
    }

    impl Verdict {
        pub fn label(&self) -> &'static str {
            match self {
                Verdict::BiasOk => "BIAS OK",
                Verdict::FormulationOkSolverOff => "FORMULATION ~OK / SOLVER-OFF (Layer B)",
                Verdict::FormulationBug => "FORMULATION BUG (Layer A)",
                Verdict::NoData => "NO DATA",
            }
        }
    }

    /// Full bias-accuracy result for one circuit.
    #[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
    pub struct OpPointResult {
        pub circuit: String,
        pub devices: Vec<DeviceMatch>,
        /// Worst per-device readings (rolled up).
        pub max_abs_d_vbe: f64,
        pub max_abs_d_vce: f64,
        pub max_abs_d_ic_pct: f64,
        /// Worst static residual across ports (volts) — the formulation probe.
        pub max_abs_residual: f64,
        /// Worst full residual across ports (volts) — should be ≈ 0 (calibration).
        pub max_abs_residual_full: f64,
        pub n_residual_ports: usize,
        pub verdict: Verdict,
    }

    impl OpPointResult {
        /// True when MATCH is within all fail thresholds.
        pub fn match_ok(&self, c: &OpPassCriteria) -> bool {
            self.max_abs_d_vbe <= c.vbe_fail_v
                && self.max_abs_d_vce <= c.vce_fail_v
                && self.max_abs_d_ic_pct <= c.ic_fail_pct
        }
        /// True when the static residual is small enough to call the formulation
        /// approximately correct.
        pub fn formulation_ok(&self, c: &OpPassCriteria) -> bool {
            self.max_abs_residual <= c.residual_formulation_v
        }
    }

    /// Compute the per-device MATCH between our settled bias and ngspice `.op`.
    ///
    /// Devices are paired by uppercased reference. Comparison is on junction
    /// magnitude (|Vbe|, |Vce|, |Ic|) so PNP/NPN and port-vs-node sign
    /// conventions don't masquerade as errors.
    pub fn match_devices(ours: &[DeviceBias], spice: &[SpiceDeviceOp]) -> Vec<DeviceMatch> {
        let mut out = Vec::new();
        for s in spice {
            let key = s.device.to_uppercase();
            let Some(o) = ours.iter().find(|d| d.reference.to_uppercase() == key) else {
                continue;
            };
            let d_vbe = o.vbe.abs() - s.vbe.abs();
            let d_vce = o.vce.abs() - s.vce.abs();
            let d_ic_pct = if s.ic.abs() > 1e-12 {
                100.0 * (o.ic.abs() - s.ic.abs()) / s.ic.abs()
            } else {
                0.0
            };
            out.push(DeviceMatch {
                reference: o.reference.clone(),
                model: s.model.clone(),
                ours: (o.vbe, o.vce, o.ic),
                spice: (s.vbe, s.vce, s.ic),
                d_vbe,
                d_vce,
                d_ic_pct,
            });
        }
        out
    }

    /// Assemble the full bias-accuracy result + verdict for one circuit.
    pub fn evaluate(
        circuit: &str,
        ours: &[DeviceBias],
        spice: &[SpiceDeviceOp],
        residuals: &[PortResidual],
        criteria: &OpPassCriteria,
    ) -> OpPointResult {
        let devices = match_devices(ours, spice);
        let max_abs_d_vbe = devices.iter().map(|d| d.d_vbe.abs()).fold(0.0, f64::max);
        let max_abs_d_vce = devices.iter().map(|d| d.d_vce.abs()).fold(0.0, f64::max);
        let max_abs_d_ic_pct = devices.iter().map(|d| d.d_ic_pct.abs()).fold(0.0, f64::max);
        let max_abs_residual = residuals.iter().map(|p| p.residual.abs()).fold(0.0, f64::max);
        let max_abs_residual_full =
            residuals.iter().map(|p| p.residual_full.abs()).fold(0.0, f64::max);

        let mut result = OpPointResult {
            circuit: circuit.to_string(),
            devices,
            max_abs_d_vbe,
            max_abs_d_vce,
            max_abs_d_ic_pct,
            max_abs_residual,
            max_abs_residual_full,
            n_residual_ports: residuals.len(),
            verdict: Verdict::NoData,
        };

        result.verdict = if result.devices.is_empty() && residuals.is_empty() {
            Verdict::NoData
        } else if !result.formulation_ok(criteria) {
            Verdict::FormulationBug
        } else if result.match_ok(criteria) {
            Verdict::BiasOk
        } else {
            Verdict::FormulationOkSolverOff
        };
        result
    }
}

// ============================================================================
// AC-signal accuracy metric (the audio twin of the DC bias-accuracy dashboard)
// ============================================================================
//
// The DC dashboard (`op_point`) grades the operating point; this module grades
// the AC SIGNAL — WDF vs the ngspice-derived golden. Its whole reason to exist
// is that `normalized_rms_error_db` DEGENERATES for a pure-gain-offset circuit:
// when the WDF and golden differ only by a scalar level `k`, that difference
// metric reads |k−1| (and → 0 dB as the WDF falls far below the golden), so it
// can neither see the true level gap nor the shape. The BA283 is exactly such a
// circuit (a clean Class-A gain block sitting at a fixed +2.42 dB level offset
// with the DC servo on), so a single-bin gain scalar is all the old test could
// honestly report.
//
// The fix here is to DECOMPOSE the AC error into orthogonal dimensions so each
// is actually measured:
//
//   1. LEVEL   — `ac_gain_db` at the test tone (the scalar offset, e.g. +2.42).
//   2. SHAPE   — gain-normalize the WDF (× golden_amp/wdf_amp) so the level is
//                factored OUT, THEN measure `normalized_rms_error_db` (phase-
//                sensitive, time domain) and `spectral_error_db` (phase-immune,
//                magnitude) on the level-matched pair = the honest shape residual.
//   3. HARMONICS — `thd_db(wdf)` vs `thd_db(golden)` (a ratio, inherently level-
//                independent) plus per-harmonic (2nd–5th) magnitude error read in
//                dBc (relative to the fundamental), which is also level-immune.
//   4. RESPONSE — `ac_gain_db` across a frequency sweep. If the per-frequency
//                gain is FLAT the error is a pure scalar (a level bug); if it is
//                frequency-SHAPED it is a genuine response error.
//
// The verdict then states plainly whether the AC error is JUST the level offset
// (shape/THD/response all clean ⇒ a flat scalar bug) or ALSO a shape / harmonic /
// response error (⇒ more than one bug). Plain-numeric like `op_point`, so it
// stays decoupled from `pedalkernel-rt`.
pub mod ac_accuracy {
    use super::{
        ac_gain_db, normalized_rms_error_db, single_bin_amplitude, spectral_error_db, thd_db,
    };
    use serde::{Deserialize, Serialize};

    /// Harmonic orders scored per-tone (2nd through 5th).
    pub const HARMONIC_ORDERS: [usize; 4] = [2, 3, 4, 5];

    /// A golden harmonic weaker than this (relative to the fundamental) is treated
    /// as absent (below the simulator noise grass): its per-harmonic error is not
    /// scored for the verdict, so ngspice noise bins don't masquerade as shape
    /// error. −80 dBc sits above ngspice's typical noise grass yet below a real
    /// diode/BJT harmonic.
    pub const HARMONIC_FLOOR_DBC: f64 = -80.0;

    /// One harmonic's level, WDF vs golden, expressed in dBc (relative to the
    /// fundamental) so it is level-independent by construction.
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct HarmonicError {
        /// Harmonic order (2..=5).
        pub order: usize,
        /// WDF harmonic level relative to its own fundamental, dB.
        pub wdf_dbc: f64,
        /// Golden harmonic level relative to its own fundamental, dB.
        pub golden_dbc: f64,
        /// `|wdf_dbc − golden_dbc|`, the level-independent per-harmonic residual.
        pub error_db: f64,
        /// True when the GOLDEN harmonic is above [`HARMONIC_FLOOR_DBC`] (present,
        /// not noise). Only scored harmonics feed the verdict.
        pub scored: bool,
    }

    /// One frequency-sweep point: WDF-vs-golden AC gain at that frequency.
    #[derive(Debug, Clone, Copy, Serialize, Deserialize)]
    pub struct FreqPoint {
        pub freq_hz: f64,
        /// `ac_gain_db(wdf, golden)` at this frequency (WDF louder ⇒ positive).
        pub gain_db: f64,
    }

    /// Thresholds for grading the AC dimensions. Informative (localizes which
    /// dimension carries the error), not a hard CI gate.
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct AcThresholds {
        /// A |gain| above this at the test tone counts as a LEVEL offset. 1 dB.
        pub gain_warn_db: f64,
        /// Gain-normalized RMS shape residual (dB) above which SHAPE is bad. The
        /// residual is `20·log10(rms(wdf_norm − golden)/rms(golden))`, so −20 dB
        /// (≈ 10 % shape difference) is the boundary. NOTE: this is phase-
        /// SENSITIVE — a pure group-delay difference inflates it even when the
        /// magnitude shape agrees; cross-check with `shape_spectral`.
        pub shape_rms_fail_db: f64,
        /// Gain-normalized, phase-IMMUNE in-band magnitude shape error (dB) above
        /// which SHAPE is bad. 3 dB.
        pub shape_spectral_fail_db: f64,
        /// |THD_wdf − THD_golden| (dB) above which HARMONICS are bad. 6 dB.
        pub thd_error_fail_db: f64,
        /// Max per-harmonic dBc error above which HARMONICS are bad. 6 dB.
        pub harmonic_fail_db: f64,
        /// Response tilt (max−min gain across the sweep, dB) above which the error
        /// is frequency-SHAPED (a RESPONSE error, not a flat scalar). 3 dB.
        pub response_tilt_fail_db: f64,
    }

    impl Default for AcThresholds {
        fn default() -> Self {
            Self {
                gain_warn_db: 1.0,
                shape_rms_fail_db: -20.0,
                shape_spectral_fail_db: 3.0,
                thd_error_fail_db: 6.0,
                harmonic_fail_db: 6.0,
                response_tilt_fail_db: 3.0,
            }
        }
    }

    /// The AC-fidelity verdict for one circuit — which dimension carries the error.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    pub enum AcVerdict {
        /// Level, shape, harmonics AND response all within thresholds.
        Clean,
        /// ONLY the level tone is off; shape + THD + response are all clean ⇒ the
        /// AC error is a FLAT SCALAR (a pure-gain bug), the waveform shape matches.
        LevelOnly,
        /// The gain-normalized shape residual exceeds threshold ⇒ the waveform
        /// shape itself differs (beyond a scalar level).
        ShapeError,
        /// THD or a per-harmonic level differs ⇒ the nonlinearity/harmonic content
        /// differs (beyond a scalar level).
        HarmonicError,
        /// The per-frequency gain is not flat ⇒ a frequency-shaped RESPONSE error.
        ResponseError,
        /// No usable golden tone (silence) — nothing to grade.
        NoData,
    }

    impl AcVerdict {
        pub fn label(&self) -> &'static str {
            match self {
                AcVerdict::Clean => "AC OK",
                AcVerdict::LevelOnly => "LEVEL-ONLY (flat scalar; shape clean)",
                AcVerdict::ShapeError => "SHAPE ERROR (waveform differs)",
                AcVerdict::HarmonicError => "HARMONIC ERROR (THD/harmonics differ)",
                AcVerdict::ResponseError => "RESPONSE ERROR (frequency-shaped)",
                AcVerdict::NoData => "NO DATA",
            }
        }
    }

    /// Full AC-accuracy result for one circuit.
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct AcResult {
        pub circuit: String,
        /// Fundamental at which the level/shape/harmonic decomposition was taken.
        pub test_freq_hz: f64,
        /// LEVEL: `ac_gain_db(wdf, golden)` at the test tone (the scalar offset).
        pub gain_db: f64,
        /// WDF single-bin amplitude at the test tone (signal units).
        pub wdf_amp: f64,
        /// Golden single-bin amplitude at the test tone (signal units).
        pub golden_amp: f64,
        /// SHAPE (level factored out, phase-sensitive time domain), dB.
        pub shape_rms_db: f64,
        /// SHAPE (level factored out, phase-immune magnitude), dB.
        pub shape_spectral_db: f64,
        /// THD of the WDF tone, dB.
        pub thd_wdf_db: f64,
        /// THD of the golden tone, dB.
        pub thd_golden_db: f64,
        /// Per-harmonic (2nd–5th) dBc levels + error.
        pub harmonics: Vec<HarmonicError>,
        /// Frequency-response sweep (per-frequency WDF-vs-golden gain).
        pub response: Vec<FreqPoint>,
        /// Response tilt = max−min gain across the sweep, dB. ≈0 ⇒ flat (a scalar
        /// level offset); large ⇒ frequency-shaped response error.
        pub response_tilt_db: f64,
        pub verdict: AcVerdict,
    }

    impl AcResult {
        /// Worst per-harmonic error over SCORED harmonics (golden present).
        pub fn max_harmonic_error_db(&self) -> f64 {
            self.harmonics
                .iter()
                .filter(|h| h.scored)
                .map(|h| h.error_db)
                .fold(0.0, f64::max)
        }

        /// |THD_wdf − THD_golden|.
        pub fn thd_error_db(&self) -> f64 {
            (self.thd_wdf_db - self.thd_golden_db).abs()
        }
    }

    /// One harmonic's level relative to the fundamental, in dBc, via single-bin
    /// projection (DC-immune). Returns `NEG_INFINITY` if the fundamental is silent.
    fn harmonic_dbc(signal: &[f64], fund_hz: f64, harm_hz: f64, sample_rate: f64) -> f64 {
        let fund = single_bin_amplitude(signal, fund_hz, sample_rate);
        if fund < 1e-30 {
            return f64::NEG_INFINITY;
        }
        let harm = single_bin_amplitude(signal, harm_hz, sample_rate);
        20.0 * (harm / fund).max(1e-12).log10()
    }

    /// Response tilt: max−min over the finite per-frequency gains.
    pub fn response_tilt_db(response: &[FreqPoint]) -> f64 {
        let finite: Vec<f64> = response
            .iter()
            .map(|p| p.gain_db)
            .filter(|g| g.is_finite())
            .collect();
        if finite.len() < 2 {
            return 0.0;
        }
        let max = finite.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min = finite.iter().cloned().fold(f64::INFINITY, f64::min);
        max - min
    }

    /// Decompose the WDF-vs-golden AC error into level / shape / harmonics /
    /// response and reach a verdict.
    ///
    /// `wdf` and `golden` are settled, same-rate, same-length, same-phase
    /// windows of the SAME test tone at `test_freq_hz`. `response` is the
    /// per-frequency gain sweep (may be empty; then RESPONSE is not graded).
    pub fn evaluate(
        circuit: &str,
        wdf: &[f64],
        golden: &[f64],
        sample_rate: f64,
        test_freq_hz: f64,
        response: Vec<FreqPoint>,
        th: &AcThresholds,
    ) -> AcResult {
        let n = wdf.len().min(golden.len());
        let wdf = &wdf[..n];
        let golden = &golden[..n];

        let wdf_amp = single_bin_amplitude(wdf, test_freq_hz, sample_rate);
        let golden_amp = single_bin_amplitude(golden, test_freq_hz, sample_rate);
        let gain_db = ac_gain_db(wdf, golden, test_freq_hz, sample_rate);

        // SHAPE: gain-normalize the WDF so the scalar level is factored OUT, then
        // measure the residual. This is THE fix for the degeneracy — after
        // normalization a pure level offset contributes nothing, so what remains
        // is the honest shape difference.
        let k = if wdf_amp > 1e-30 {
            golden_amp / wdf_amp
        } else {
            0.0
        };
        let wdf_norm: Vec<f64> = wdf.iter().map(|&x| k * x).collect();
        let shape_rms_db = normalized_rms_error_db(&wdf_norm, golden);
        let shape_spectral_db = spectral_error_db(&wdf_norm, golden, sample_rate, None);

        // HARMONICS: THD ratio (level-independent) + per-harmonic dBc.
        let thd_wdf_db = thd_db(wdf, test_freq_hz, sample_rate, 10);
        let thd_golden_db = thd_db(golden, test_freq_hz, sample_rate, 10);
        let harmonics: Vec<HarmonicError> = HARMONIC_ORDERS
            .iter()
            .filter_map(|&order| {
                let hf = order as f64 * test_freq_hz;
                if hf >= sample_rate / 2.0 {
                    return None;
                }
                let wdf_dbc = harmonic_dbc(wdf, test_freq_hz, hf, sample_rate);
                let golden_dbc = harmonic_dbc(golden, test_freq_hz, hf, sample_rate);
                let scored = golden_dbc > HARMONIC_FLOOR_DBC;
                Some(HarmonicError {
                    order,
                    wdf_dbc,
                    golden_dbc,
                    error_db: (wdf_dbc - golden_dbc).abs(),
                    scored,
                })
            })
            .collect();

        let response_tilt_db = response_tilt_db(&response);

        let mut result = AcResult {
            circuit: circuit.to_string(),
            test_freq_hz,
            gain_db,
            wdf_amp,
            golden_amp,
            shape_rms_db,
            shape_spectral_db,
            thd_wdf_db,
            thd_golden_db,
            harmonics,
            response,
            response_tilt_db,
            verdict: AcVerdict::NoData,
        };

        // Classify. Priority: a genuine shape/harmonic/response error outranks a
        // pure level offset (which is the benign, expected BA283 signature).
        let level_off = gain_db.abs() > th.gain_warn_db;
        let shape_bad = shape_rms_db > th.shape_rms_fail_db
            || shape_spectral_db > th.shape_spectral_fail_db;
        let harm_bad = result.thd_error_db() > th.thd_error_fail_db
            || result.max_harmonic_error_db() > th.harmonic_fail_db;
        let resp_bad =
            !result.response.is_empty() && response_tilt_db > th.response_tilt_fail_db;

        result.verdict = if golden_amp < 1e-12 {
            AcVerdict::NoData
        } else if shape_bad {
            AcVerdict::ShapeError
        } else if harm_bad {
            AcVerdict::HarmonicError
        } else if resp_bad {
            AcVerdict::ResponseError
        } else if level_off {
            AcVerdict::LevelOnly
        } else {
            AcVerdict::Clean
        };
        result
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

    // ── op_point bias-accuracy metric (ngspice-free, synthetic) ──────────────
    mod op_point_logic {
        use crate::metrics::op_point::*;
        use crate::spice::SpiceDeviceOp;

        fn spice(dev: &str, vbe: f64, vce: f64, ic: f64) -> SpiceDeviceOp {
            SpiceDeviceOp {
                device: dev.into(),
                model: "qmod".into(),
                vbe,
                vce,
                ic,
                ib: ic / 100.0,
                gm: 0.03,
            }
        }
        fn ours(reference: &str, vbe: f64, vce: f64, ic: f64) -> DeviceBias {
            DeviceBias { reference: reference.into(), vbe, vce, ic }
        }

        #[test]
        fn residual_near_zero_and_match_ok_is_bias_ok() {
            let c = OpPassCriteria::default();
            let r = evaluate(
                "x",
                &[ours("Q1", 0.6505, 1.10, 1.0e-3)],
                &[spice("Q1", 0.650, 1.10, 1.0e-3)],
                &[PortResidual { residual: 0.01, residual_full: 0.01 }],
                &c,
            );
            assert_eq!(r.verdict, Verdict::BiasOk);
            assert!(r.match_ok(&c) && r.formulation_ok(&c));
        }

        #[test]
        fn residual_small_but_match_off_is_solver_off_layer_b() {
            // The BA283 signature: ngspice's .op IS a fixed point (small residual)
            // but our settled bias lands elsewhere (large ΔVbe / wrong Ic).
            let c = OpPassCriteria::default();
            let r = evaluate(
                "ba283-like",
                &[ours("Q1", 0.20, 19.9, 0.0)],
                &[spice("Q1", 0.61, 5.35, 0.26e-3)],
                &[PortResidual { residual: 0.027, residual_full: 0.027 }],
                &c,
            );
            assert_eq!(r.verdict, Verdict::FormulationOkSolverOff);
            assert!(r.formulation_ok(&c) && !r.match_ok(&c));
        }

        #[test]
        fn large_residual_is_formulation_bug_layer_a() {
            let c = OpPassCriteria::default();
            let r = evaluate(
                "y",
                &[ours("Q1", 0.0, 0.0, 0.0)],
                &[spice("Q1", 0.0, 0.0, 0.0)],
                &[PortResidual { residual: 8.5, residual_full: 8.5 }],
                &c,
            );
            assert_eq!(r.verdict, Verdict::FormulationBug);
        }

        #[test]
        fn match_compares_on_magnitude_for_pnp() {
            // PNP: our port Vbe is negative, ngspice reports positive magnitude.
            let dm = match_devices(
                &[ours("Q1", -0.62, -3.5, -1.0e-3)],
                &[spice("Q1", 0.62, 3.5, -1.0e-3)],
            );
            assert_eq!(dm.len(), 1);
            assert!(dm[0].d_vbe.abs() < 1e-9, "magnitude match ⇒ ΔVbe≈0");
            assert!(dm[0].d_ic_pct.abs() < 1e-6);
        }
    }

    // ── ac_accuracy AC-fidelity metric (ngspice-free, synthetic) ─────────────
    mod ac_accuracy_logic {
        use crate::metrics::ac_accuracy::*;

        const SR: f64 = 48_000.0;
        const F: f64 = 1_000.0;

        /// A pure 1 kHz tone (+ optional harmonics) as a golden.
        fn tone(amp: f64, h2: f64, h3: f64) -> Vec<f64> {
            (0..48_000)
                .map(|i| {
                    let t = i as f64 / SR;
                    amp * ((2.0 * std::f64::consts::PI * F * t).sin()
                        + h2 * (2.0 * std::f64::consts::PI * 2.0 * F * t).sin()
                        + h3 * (2.0 * std::f64::consts::PI * 3.0 * F * t).sin())
                })
                .collect()
        }

        /// THE core point: when the WDF differs from the golden ONLY by a scalar
        /// level, the verdict must be LevelOnly — the +N dB is a flat scalar and
        /// the gain-normalized shape residual is ~0 (what the degenerate
        /// `normalized_rms_error_db` could never separate).
        #[test]
        fn pure_level_offset_is_level_only_and_shape_clean() {
            let th = AcThresholds::default();
            let golden = tone(0.1, 0.02, 0.005);
            // WDF: same waveform SHAPE, +2.42 dB louder (the BA283 signature).
            let k = 10f64.powf(2.42 / 20.0);
            let wdf: Vec<f64> = golden.iter().map(|&x| k * x).collect();
            // Flat frequency response (same +2.42 dB at every frequency).
            let response: Vec<FreqPoint> = [50.0, 1000.0, 10000.0]
                .iter()
                .map(|&f| FreqPoint { freq_hz: f, gain_db: 2.42 })
                .collect();

            let r = evaluate("scaled", &wdf, &golden, SR, F, response, &th);
            assert!((r.gain_db - 2.42).abs() < 0.05, "level = +2.42 dB, got {:.3}", r.gain_db);
            // Gain factored out ⇒ shape residual collapses.
            assert!(r.shape_rms_db < -60.0, "shape RMS must be clean, got {:.1}", r.shape_rms_db);
            assert!(r.shape_spectral_db < 1.0, "shape spectral clean, got {:.2}", r.shape_spectral_db);
            assert!(r.thd_error_db() < 1.0, "THD matches, got {:.2}", r.thd_error_db());
            assert!(r.max_harmonic_error_db() < 1.0, "harmonics match, got {:.2}", r.max_harmonic_error_db());
            assert!(r.response_tilt_db < 0.5, "response flat, got {:.2}", r.response_tilt_db);
            assert_eq!(r.verdict, AcVerdict::LevelOnly);
        }

        /// A genuine shape difference (extra 2nd harmonic in the WDF) must NOT be
        /// hidden by gain normalization — verdict Shape or Harmonic error.
        #[test]
        fn added_harmonic_is_not_level_only() {
            let th = AcThresholds::default();
            let golden = tone(0.1, 0.0, 0.0); // clean sine
            let wdf = tone(0.1, 0.3, 0.0); // strong 2nd harmonic added
            let r = evaluate("distorted", &wdf, &golden, SR, F, vec![], &th);
            assert_ne!(r.verdict, AcVerdict::LevelOnly);
            assert_ne!(r.verdict, AcVerdict::Clean);
            assert!(
                matches!(r.verdict, AcVerdict::ShapeError | AcVerdict::HarmonicError),
                "added harmonic ⇒ Shape/Harmonic, got {:?}",
                r.verdict
            );
        }

        /// A frequency-SHAPED gain (flat level fine at 1 kHz, but tilted across the
        /// sweep) must classify as ResponseError, not LevelOnly.
        #[test]
        fn frequency_shaped_response_is_response_error() {
            let th = AcThresholds::default();
            let golden = tone(0.1, 0.0, 0.0);
            let wdf: Vec<f64> = golden.clone(); // identical at 1 kHz (level+shape clean)
            let response: Vec<FreqPoint> = vec![
                FreqPoint { freq_hz: 50.0, gain_db: -8.0 },
                FreqPoint { freq_hz: 1000.0, gain_db: 0.0 },
                FreqPoint { freq_hz: 10000.0, gain_db: 2.0 },
            ];
            let r = evaluate("tilted", &wdf, &golden, SR, F, response, &th);
            assert!(r.response_tilt_db > 3.0, "tilt = {:.1} dB", r.response_tilt_db);
            assert_eq!(r.verdict, AcVerdict::ResponseError);
        }

        /// Silence golden ⇒ NoData (no tone to grade against).
        #[test]
        fn silent_golden_is_no_data() {
            let th = AcThresholds::default();
            let golden = vec![0.0; 48_000];
            let wdf = tone(0.1, 0.0, 0.0);
            let r = evaluate("silent", &wdf, &golden, SR, F, vec![], &th);
            assert_eq!(r.verdict, AcVerdict::NoData);
        }
    }
}
