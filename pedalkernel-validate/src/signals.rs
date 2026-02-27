//! Deterministic test signal generators for audio validation.
//!
//! This module provides functions to generate standard test signals used in
//! audio circuit validation. All generators are deterministic (no randomness)
//! to ensure reproducible test results.
//!
//! # Available Signals
//!
//! | Signal | Use Case |
//! |--------|----------|
//! | [`impulse`] | Impulse response measurement, FIR characterization |
//! | [`sine`] | THD measurement, frequency response at single point |
//! | [`two_tone`] | IMD (Intermodulation Distortion) testing |
//! | [`exp_sweep`] | Frequency response measurement, system identification |
//! | [`silence`] | DC drift testing, noise floor measurement |
//! | [`tone_burst`] | Attack/release timing, transient response |
//! | [`level_sweep`] | Gain curves, compression characteristics |
//!
//! # Example
//!
//! ```rust
//! use pedalkernel_validate::signals;
//!
//! let sr = 48000.0;
//!
//! // Generate a 1kHz test tone
//! let tone = signals::sine(sr, 1000.0, 0.1, 1.0);
//!
//! // Generate a frequency sweep for response measurement
//! let sweep = signals::exp_sweep(sr, 20.0, 20000.0, 1.0, 0.8);
//! ```

use std::f64::consts::PI;

/// Generate an impulse (unit sample at t=0).
pub fn impulse(n_samples: usize, amplitude: f64) -> Vec<f64> {
    let mut signal = vec![0.0; n_samples];
    if !signal.is_empty() {
        signal[0] = amplitude;
    }
    signal
}

/// Generate a sine wave.
pub fn sine(sample_rate: f64, frequency: f64, duration: f64, amplitude: f64) -> Vec<f64> {
    let n_samples = (duration * sample_rate) as usize;
    (0..n_samples)
        .map(|i| {
            let t = i as f64 / sample_rate;
            amplitude * (2.0 * PI * frequency * t).sin()
        })
        .collect()
}

/// Generate a two-tone signal for IMD (intermodulation distortion) testing.
pub fn two_tone(
    sample_rate: f64,
    f1: f64,
    f2: f64,
    duration: f64,
    amplitude: f64,
) -> Vec<f64> {
    let n_samples = (duration * sample_rate) as usize;
    (0..n_samples)
        .map(|i| {
            let t = i as f64 / sample_rate;
            amplitude * 0.5 * ((2.0 * PI * f1 * t).sin() + (2.0 * PI * f2 * t).sin())
        })
        .collect()
}

/// Generate an exponential (logarithmic) frequency sweep.
///
/// Sweeps from `f_start` to `f_end` over `duration` seconds.
/// This is the standard sweep for frequency response measurement.
pub fn exp_sweep(
    sample_rate: f64,
    f_start: f64,
    f_end: f64,
    duration: f64,
    amplitude: f64,
) -> Vec<f64> {
    let n_samples = (duration * sample_rate) as usize;
    let log_ratio = (f_end / f_start).ln();

    (0..n_samples)
        .map(|i| {
            let t = i as f64 / sample_rate;
            // Instantaneous frequency: f(t) = f_start * exp(t * ln(f_end/f_start) / duration)
            // Phase integral: phi(t) = 2*pi * f_start * duration / ln(f_end/f_start) * (exp(...) - 1)
            let phase = 2.0 * PI * f_start * duration / log_ratio
                * ((t * log_ratio / duration).exp() - 1.0);
            amplitude * phase.sin()
        })
        .collect()
}

/// Generate silence (zeros).
pub fn silence(n_samples: usize) -> Vec<f64> {
    vec![0.0; n_samples]
}

/// Generate a tone burst for attack/release timing measurement.
///
/// Creates a signal with `repetitions` bursts of a sine wave,
/// each `on_ms` long followed by `off_ms` of silence.
pub fn tone_burst(
    sample_rate: f64,
    frequency: f64,
    amplitude: f64,
    on_ms: f64,
    off_ms: f64,
    repetitions: usize,
) -> Vec<f64> {
    let on_samples = (on_ms * sample_rate / 1000.0) as usize;
    let off_samples = (off_ms * sample_rate / 1000.0) as usize;
    let period_samples = on_samples + off_samples;
    let total_samples = period_samples * repetitions;

    let mut signal = vec![0.0; total_samples];

    for rep in 0..repetitions {
        let start = rep * period_samples;
        for i in 0..on_samples {
            let t = i as f64 / sample_rate;
            signal[start + i] = amplitude * (2.0 * PI * frequency * t).sin();
        }
        // off_samples remain zero
    }

    signal
}

/// Convert dBVU to peak voltage.
///
/// 0 dBVU ≈ +4 dBu ≈ 1.228 Vrms ≈ 1.736 Vpeak
/// We use 0.7746 as the reference (simplification).
pub fn dbvu_to_peak(dbvu: f64) -> f64 {
    10.0_f64.powf(dbvu / 20.0) * 0.7746
}

/// Generate a level sweep for static gain curve measurement.
///
/// Generates a series of sine wave segments at different amplitudes,
/// holding each level for `duration_per_level` seconds.
pub fn level_sweep(
    sample_rate: f64,
    frequency: f64,
    levels_dbvu: &[f64],
    duration_per_level: f64,
) -> Vec<f64> {
    let samples_per_level = (duration_per_level * sample_rate) as usize;
    let mut signal = Vec::with_capacity(levels_dbvu.len() * samples_per_level);

    for &level in levels_dbvu {
        let amplitude = dbvu_to_peak(level);
        for i in 0..samples_per_level {
            let t = i as f64 / sample_rate;
            signal.push(amplitude * (2.0 * PI * frequency * t).sin());
        }
    }

    signal
}

/// Signal specification from config.
#[derive(Debug, Clone)]
pub enum SignalSpec {
    Impulse { amplitude: f64 },
    Sine { frequency: f64, amplitude: f64, duration: f64 },
    TwoTone { f1: f64, f2: f64, amplitude: f64, duration: f64 },
    ExpSweep { f_start: f64, f_end: f64, amplitude: f64, duration: f64 },
    Silence { duration: f64 },
    ToneBurst { frequency: f64, amplitude: f64, on_ms: f64, off_ms: f64, repetitions: usize },
    LevelSweep { frequency: f64, levels_dbvu: Vec<f64>, duration_per_level: f64 },
}

impl SignalSpec {
    /// Generate the signal at the given sample rate.
    pub fn generate(&self, sample_rate: f64) -> Vec<f64> {
        match self {
            SignalSpec::Impulse { amplitude } => {
                impulse((sample_rate * 0.1) as usize, *amplitude) // 100ms default
            }
            SignalSpec::Sine { frequency, amplitude, duration } => {
                sine(sample_rate, *frequency, *duration, *amplitude)
            }
            SignalSpec::TwoTone { f1, f2, amplitude, duration } => {
                two_tone(sample_rate, *f1, *f2, *duration, *amplitude)
            }
            SignalSpec::ExpSweep { f_start, f_end, amplitude, duration } => {
                exp_sweep(sample_rate, *f_start, *f_end, *duration, *amplitude)
            }
            SignalSpec::Silence { duration } => {
                silence((duration * sample_rate) as usize)
            }
            SignalSpec::ToneBurst { frequency, amplitude, on_ms, off_ms, repetitions } => {
                tone_burst(sample_rate, *frequency, *amplitude, *on_ms, *off_ms, *repetitions)
            }
            SignalSpec::LevelSweep { frequency, levels_dbvu, duration_per_level } => {
                level_sweep(sample_rate, *frequency, levels_dbvu, *duration_per_level)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn impulse_has_correct_shape() {
        let sig = impulse(100, 1.0);
        assert_eq!(sig.len(), 100);
        assert_eq!(sig[0], 1.0);
        assert!(sig[1..].iter().all(|&x| x == 0.0));
    }

    #[test]
    fn sine_has_correct_frequency() {
        let sr = 48000.0;
        let freq = 1000.0;
        let sig = sine(sr, freq, 0.01, 1.0);

        // Should have ~10 cycles in 10ms at 1kHz
        let zero_crossings: usize = sig.windows(2)
            .filter(|w| w[0] * w[1] < 0.0)
            .count();
        // Each cycle has 2 zero crossings, 10 cycles = ~20 crossings
        assert!(zero_crossings >= 18 && zero_crossings <= 22);
    }

    #[test]
    fn exp_sweep_starts_and_ends_correctly() {
        let sr = 48000.0;
        let sig = exp_sweep(sr, 20.0, 20000.0, 1.0, 1.0);
        assert_eq!(sig.len(), 48000);
        // Just verify it doesn't explode
        assert!(sig.iter().all(|&x| x.is_finite()));
        assert!(sig.iter().map(|x| x.abs()).fold(0.0f64, |a, b| a.max(b)) <= 1.01);
    }

    #[test]
    fn tone_burst_has_correct_structure() {
        let sr = 48000.0;
        let sig = tone_burst(sr, 1000.0, 1.0, 10.0, 90.0, 2);

        // 10ms on + 90ms off = 100ms per rep, 2 reps = 200ms = 9600 samples
        assert_eq!(sig.len(), 9600);

        // First 480 samples (10ms) should have signal
        let first_burst_energy: f64 = sig[0..480].iter().map(|x| x * x).sum();
        assert!(first_burst_energy > 0.1);

        // Next 4320 samples (90ms) should be silent
        let first_silence: f64 = sig[480..4800].iter().map(|x| x * x).sum();
        assert!(first_silence < 1e-10);
    }
}
