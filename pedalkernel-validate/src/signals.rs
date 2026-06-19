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
//! | [`sawtooth`] | Per-harmonic linear/EQ accuracy (all harmonics) |
//! | [`triangle`] | Per-harmonic linear/EQ accuracy (odd harmonics) |
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
pub fn two_tone(sample_rate: f64, f1: f64, f2: f64, duration: f64, amplitude: f64) -> Vec<f64> {
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

/// Generate a band-limited sawtooth wave via additive synthesis.
///
/// Builds the wave from only the harmonics strictly below Nyquist, so it is
/// exactly band-limited by construction (no aliasing — unlike a naive ramp or a
/// PolyBLEP approximation). The harmonically-rich spectrum (energy at every
/// integer multiple of the fundamental) lets a single FFT localize per-harmonic
/// model error against a SPICE reference, which a pure sine cannot.
///
/// Series: `x(t) = (2/pi) * sum_{k=1..K} (-1)^(k+1) * sin(2*pi*k*f*t) / k`,
/// where `K = floor((sample_rate/2) / f)`. The final buffer is normalized so
/// `max |x| == amplitude`.
///
/// Returns a zero buffer of the expected length when `frequency <= 0` or when no
/// harmonic fits below Nyquist (`K == 0`).
pub fn sawtooth(sample_rate: f64, frequency: f64, duration: f64, amplitude: f64) -> Vec<f64> {
    let n_samples = (duration * sample_rate) as usize;
    if frequency <= 0.0 {
        return vec![0.0; n_samples];
    }
    let n_harmonics = ((sample_rate / 2.0) / frequency).floor() as usize;
    if n_harmonics == 0 {
        return vec![0.0; n_samples];
    }

    let mut signal: Vec<f64> = (0..n_samples)
        .map(|i| {
            let t = i as f64 / sample_rate;
            let mut acc = 0.0;
            for k in 1..=n_harmonics {
                let sign = if k % 2 == 1 { 1.0 } else { -1.0 };
                acc += sign * (2.0 * PI * k as f64 * frequency * t).sin() / k as f64;
            }
            (2.0 / PI) * acc
        })
        .collect();

    normalize_peak(&mut signal, amplitude);
    signal
}

/// Generate a band-limited triangle wave (odd harmonics only) via additive
/// synthesis.
///
/// Like [`sawtooth`], but uses only odd harmonics with `1/n^2` weighting and
/// alternating sign, giving the characteristic triangle spectrum. Exactly
/// band-limited by construction.
///
/// Series:
/// `x(t) = (8/pi^2) * sum_{n odd, n<=K} (-1)^((n-1)/2) * sin(2*pi*n*f*t) / n^2`,
/// where `K = floor((sample_rate/2) / f)`. The final buffer is normalized so
/// `max |x| == amplitude`.
///
/// Returns a zero buffer of the expected length when `frequency <= 0` or when no
/// harmonic fits below Nyquist (`K == 0`).
pub fn triangle(sample_rate: f64, frequency: f64, duration: f64, amplitude: f64) -> Vec<f64> {
    let n_samples = (duration * sample_rate) as usize;
    if frequency <= 0.0 {
        return vec![0.0; n_samples];
    }
    let n_harmonics = ((sample_rate / 2.0) / frequency).floor() as usize;
    if n_harmonics == 0 {
        return vec![0.0; n_samples];
    }

    let mut signal: Vec<f64> = (0..n_samples)
        .map(|i| {
            let t = i as f64 / sample_rate;
            let mut acc = 0.0;
            let mut n = 1usize;
            while n <= n_harmonics {
                // sign = (-1)^((n-1)/2): +,-,+,- for n = 1,3,5,7...
                let sign = if ((n - 1) / 2) % 2 == 0 { 1.0 } else { -1.0 };
                acc += sign * (2.0 * PI * n as f64 * frequency * t).sin() / (n as f64 * n as f64);
                n += 2;
            }
            (8.0 / (PI * PI)) * acc
        })
        .collect();

    normalize_peak(&mut signal, amplitude);
    signal
}

/// Scale `signal` in place so its peak absolute value equals `amplitude`.
///
/// No-op if the signal is silent (avoids divide-by-zero).
fn normalize_peak(signal: &mut [f64], amplitude: f64) {
    let peak = signal.iter().fold(0.0f64, |m, &x| m.max(x.abs()));
    if peak > 0.0 {
        let scale = amplitude / peak;
        for x in signal.iter_mut() {
            *x *= scale;
        }
    }
}

/// Signal specification from config.
#[derive(Debug, Clone)]
pub enum SignalSpec {
    Impulse {
        amplitude: f64,
    },
    Sine {
        frequency: f64,
        amplitude: f64,
        duration: f64,
    },
    TwoTone {
        f1: f64,
        f2: f64,
        amplitude: f64,
        duration: f64,
    },
    ExpSweep {
        f_start: f64,
        f_end: f64,
        amplitude: f64,
        duration: f64,
    },
    Silence {
        duration: f64,
    },
    ToneBurst {
        frequency: f64,
        amplitude: f64,
        on_ms: f64,
        off_ms: f64,
        repetitions: usize,
    },
    LevelSweep {
        frequency: f64,
        levels_dbvu: Vec<f64>,
        duration_per_level: f64,
    },
    Sawtooth {
        frequency: f64,
        amplitude: f64,
        duration: f64,
    },
    Triangle {
        frequency: f64,
        amplitude: f64,
        duration: f64,
    },
}

impl SignalSpec {
    /// Generate the signal at the given sample rate.
    pub fn generate(&self, sample_rate: f64) -> Vec<f64> {
        match self {
            SignalSpec::Impulse { amplitude } => {
                impulse((sample_rate * 0.1) as usize, *amplitude) // 100ms default
            }
            SignalSpec::Sine {
                frequency,
                amplitude,
                duration,
            } => sine(sample_rate, *frequency, *duration, *amplitude),
            SignalSpec::TwoTone {
                f1,
                f2,
                amplitude,
                duration,
            } => two_tone(sample_rate, *f1, *f2, *duration, *amplitude),
            SignalSpec::ExpSweep {
                f_start,
                f_end,
                amplitude,
                duration,
            } => exp_sweep(sample_rate, *f_start, *f_end, *duration, *amplitude),
            SignalSpec::Silence { duration } => silence((duration * sample_rate) as usize),
            SignalSpec::ToneBurst {
                frequency,
                amplitude,
                on_ms,
                off_ms,
                repetitions,
            } => tone_burst(
                sample_rate,
                *frequency,
                *amplitude,
                *on_ms,
                *off_ms,
                *repetitions,
            ),
            SignalSpec::LevelSweep {
                frequency,
                levels_dbvu,
                duration_per_level,
            } => level_sweep(sample_rate, *frequency, levels_dbvu, *duration_per_level),
            SignalSpec::Sawtooth {
                frequency,
                amplitude,
                duration,
            } => sawtooth(sample_rate, *frequency, *duration, *amplitude),
            SignalSpec::Triangle {
                frequency,
                amplitude,
                duration,
            } => triangle(sample_rate, *frequency, *duration, *amplitude),
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
        let zero_crossings: usize = sig.windows(2).filter(|w| w[0] * w[1] < 0.0).count();
        // Each cycle has 2 zero crossings, 10 cycles = ~20 crossings
        assert!((18..=22).contains(&zero_crossings));
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

    /// Single-bin Goertzel-style DFT magnitude at an arbitrary (possibly
    /// non-integer-bin) frequency. Deterministic; used by the band-limiting
    /// guard so we don't depend on FFT bin alignment.
    fn dft_mag(signal: &[f64], sample_rate: f64, freq: f64) -> f64 {
        let mut re = 0.0;
        let mut im = 0.0;
        for (i, &x) in signal.iter().enumerate() {
            let phase = 2.0 * PI * freq * (i as f64 / sample_rate);
            re += x * phase.cos();
            im -= x * phase.sin();
        }
        (re * re + im * im).sqrt() / signal.len() as f64
    }

    #[test]
    fn sawtooth_shape_and_length() {
        let sr = 48000.0;
        let amp = 0.8;
        let sig = sawtooth(sr, 1000.0, 0.1, amp);
        assert_eq!(sig.len(), 4800);
        assert!(sig.iter().all(|&x| x.is_finite()));
        let peak = sig.iter().fold(0.0f64, |m, &x| m.max(x.abs()));
        // Normalized: peak must equal amplitude within a few %.
        assert!((peak - amp).abs() / amp < 0.03, "peak {peak} vs amp {amp}");
    }

    #[test]
    fn triangle_shape_and_length() {
        let sr = 48000.0;
        let amp = 0.5;
        let sig = triangle(sr, 1000.0, 0.1, amp);
        assert_eq!(sig.len(), 4800);
        assert!(sig.iter().all(|&x| x.is_finite()));
        let peak = sig.iter().fold(0.0f64, |m, &x| m.max(x.abs()));
        assert!((peak - amp).abs() / amp < 0.03, "peak {peak} vs amp {amp}");
    }

    #[test]
    fn sawtooth_triangle_handle_degenerate_inputs() {
        let sr = 48000.0;
        // Zero / negative frequency -> silence of expected length.
        assert_eq!(sawtooth(sr, 0.0, 0.1, 1.0).len(), 4800);
        assert!(sawtooth(sr, 0.0, 0.1, 1.0).iter().all(|&x| x == 0.0));
        assert!(triangle(sr, -10.0, 0.1, 1.0).iter().all(|&x| x == 0.0));
        // Frequency above Nyquist -> K == 0 -> silence.
        assert!(sawtooth(sr, 30000.0, 0.1, 1.0).iter().all(|&x| x == 0.0));
        assert!(triangle(sr, 30000.0, 0.1, 1.0).iter().all(|&x| x == 0.0));
    }

    #[test]
    fn triangle_has_strong_fundamental_and_few_even_harmonics() {
        let sr = 48000.0;
        let freq = 1000.0;
        let dur = 1.0;
        let tri = triangle(sr, freq, dur, 1.0);

        // Zero-crossing count ~ 2 * freq * duration (2 per cycle).
        let zc: usize = tri.windows(2).filter(|w| w[0] * w[1] < 0.0).count();
        let expected = (2.0 * freq * dur) as usize;
        assert!(
            zc.abs_diff(expected) <= 4,
            "triangle zero-crossings {zc} vs expected {expected}"
        );

        // Triangle is odd-harmonic only: 2nd harmonic should be far weaker than
        // the same harmonic in a sawtooth (which has full even content).
        let fund = dft_mag(&tri, sr, freq);
        let tri_2nd = dft_mag(&tri, sr, 2.0 * freq);
        assert!(
            tri_2nd / fund < 1e-3,
            "triangle 2nd/fund = {}",
            tri_2nd / fund
        );

        let saw = sawtooth(sr, freq, dur, 1.0);
        let saw_fund = dft_mag(&saw, sr, freq);
        let saw_2nd = dft_mag(&saw, sr, 2.0 * freq);
        // Sawtooth has substantial even-harmonic content; triangle has ~none.
        assert!(
            saw_2nd / saw_fund > 0.1,
            "saw 2nd/fund = {}",
            saw_2nd / saw_fund
        );
        assert!(tri_2nd / fund < saw_2nd / saw_fund);
    }

    #[test]
    fn sawtooth_is_band_limited_no_aliasing() {
        // The key guard: a band-limited 1 kHz saw at 48 kHz must have negligible
        // energy above its highest included harmonic (which is just below
        // Nyquist). Any energy there would only appear via aliasing.
        let sr = 48000.0;
        let freq = 1000.0;
        let dur = 1.0;
        let sig = sawtooth(sr, freq, dur, 1.0);

        let n_harmonics = ((sr / 2.0) / freq).floor() as usize; // 23
        let highest_harmonic = n_harmonics as f64 * freq; // 23 kHz
        let nyquist = sr / 2.0; // 24 kHz

        let fund = dft_mag(&sig, sr, freq);
        assert!(fund > 0.0);

        // Probe several frequencies that lie strictly ABOVE the highest included
        // harmonic but below Nyquist. None of these are real harmonics, so any
        // energy is aliasing. Require >= 60 dB below the fundamental.
        let probes = [
            highest_harmonic + 500.0,
            (highest_harmonic + nyquist) / 2.0,
            nyquist - 100.0,
        ];
        for &p in &probes {
            let mag = dft_mag(&sig, sr, p);
            let rel_db = 20.0 * (mag / fund).log10();
            assert!(
                rel_db < -60.0,
                "aliasing energy at {p} Hz: {rel_db:.1} dB below fundamental (want < -60)"
            );
        }

        // Also confirm the included harmonics ARE present (sanity that the probe
        // method works): 5th harmonic should be ~ fund/5 for a saw.
        let fifth = dft_mag(&sig, sr, 5.0 * freq);
        assert!(
            fifth / fund > 0.1,
            "5th harmonic too weak: {}",
            fifth / fund
        );
    }
}
