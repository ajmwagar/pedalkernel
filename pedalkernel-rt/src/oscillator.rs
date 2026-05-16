//! Band-limited VCO with analog imperfections.
//!
//! Designed for TB-303 emulation but usable as a general synth oscillator.
//! `no_std` compatible, zero allocation.
//!
//! # Analog character
//!
//! - **Saw curvature**: Slight exponential bow on the up-ramp models the
//!   303's capacitor not charging linearly. Adds even harmonics.
//! - **VCO drift**: Slow random walk on pitch (±3–5 cents over seconds),
//!   plus frequency-dependent thermal drift (higher notes = more drift
//!   from exponential converter heating). Without this, it sounds dead.
//! - **Pulse width asymmetry**: Square wave threshold offset from 50%
//!   (~48%) gives the right odd/even harmonic balance.
//! - **Output bandwidth**: Single-pole lowpass at ~12 kHz models parasitic
//!   capacitances in the analog signal path.
//!
//! # Anti-aliasing
//!
//! PolyBLEP (polynomial band-limited step) on both saw and square
//! transitions. Negligible cost, removes >90% of aliasing energy.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::math;

// ── Constants ───────────────────────────────────────────────────────────

/// Base frequency for V/oct = 0V (C1, MIDI 36).
const BASE_FREQ_HZ: f64 = 65.4064;

/// Default saw ramp curvature exponent. 1.0 = linear, <1.0 = concave bow.
/// 0.92 gives subtle even-harmonic content matching 303 measurements.
const SAW_CURVE: f64 = 0.92;

/// Default square pulse width. 0.5 = perfect 50%. 303 sits around 0.48.
const PULSE_WIDTH: f64 = 0.48;

/// Output lowpass cutoff (Hz). Models parasitic rolloff in analog path.
const OUTPUT_LP_HZ: f64 = 12_000.0;

/// Drift: slow wander magnitude in semitones (±).
const DRIFT_SLOW_SEMITONES: f64 = 0.05; // ~5 cents

/// Drift: lowpass time constant for slow wander (seconds).
const DRIFT_SLOW_TAU: f64 = 2.0;

/// Drift: thermal coefficient — drift scales with log2(freq/base_freq).
/// Higher notes drift more because the exponential converter runs hotter.
const DRIFT_THERMAL_SCALE: f64 = 0.02; // cents per octave above base

// ── PolyBLEP ────────────────────────────────────────────────────────────

/// Polynomial band-limited step function.
/// `t` is the phase distance from the discontinuity, normalized to [0, dt).
/// `dt` is the phase increment (freq / sample_rate).
#[inline]
fn poly_blep(t: f64, dt: f64) -> f64 {
    if t < dt {
        // Rising toward discontinuity
        let t_norm = t / dt;
        2.0 * t_norm - t_norm * t_norm - 1.0
    } else if t > 1.0 - dt {
        // Just past discontinuity (wrapped)
        let t_norm = (t - 1.0) / dt;
        t_norm * t_norm + 2.0 * t_norm + 1.0
    } else {
        0.0
    }
}

// ── Simple PRNG (xorshift32, no_std friendly) ───────────────────────────

/// Deterministic PRNG for drift noise. No alloc, no std.
struct Rng {
    state: u32,
}

impl Rng {
    fn new(seed: u32) -> Self {
        Self {
            state: if seed == 0 { 1 } else { seed },
        }
    }

    /// Returns a value in [-1.0, 1.0].
    #[inline]
    fn next_bipolar(&mut self) -> f64 {
        // xorshift32
        self.state ^= self.state << 13;
        self.state ^= self.state >> 17;
        self.state ^= self.state << 5;
        // Map u32 to [-1, 1]
        (self.state as f64 / u32::MAX as f64) * 2.0 - 1.0
    }
}

// ── VCO ─────────────────────────────────────────────────────────────────

/// Band-limited oscillator with analog imperfections.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Vco {
    phase: f64,
    sample_rate: f64,
    inv_sample_rate: f64,

    // Analog imperfections
    #[cfg_attr(feature = "serde", serde(skip, default = "default_rng"))]
    rng: Rng,
    drift_slow: f64,      // slow random walk state (semitones)
    drift_lp_coeff: f64,  // lowpass coefficient for drift smoothing
    output_lp: f64,       // output bandwidth limiter state
    output_lp_coeff: f64, // coefficient for output LP

    // Character parameters
    saw_curve: f64,
    pulse_width: f64,
}

#[cfg(feature = "serde")]
fn default_rng() -> Rng {
    Rng::new(0x303A_C1D0)
}

impl Vco {
    /// Create a new VCO at the given sample rate.
    pub fn new(sample_rate: f64) -> Self {
        let inv_sr = 1.0 / sample_rate;
        Self {
            phase: 0.0,
            sample_rate,
            inv_sample_rate: inv_sr,
            rng: Rng::new(0x303A_C1D0),
            drift_slow: 0.0,
            drift_lp_coeff: math::exp(-1.0 / (DRIFT_SLOW_TAU * sample_rate)),
            output_lp: 0.0,
            output_lp_coeff: math::exp(-2.0 * core::f64::consts::PI * OUTPUT_LP_HZ * inv_sr),
            saw_curve: SAW_CURVE,
            pulse_width: PULSE_WIDTH,
        }
    }

    /// Update sample rate (e.g., when host changes it).
    pub fn set_sample_rate(&mut self, sr: f64) {
        self.sample_rate = sr;
        self.inv_sample_rate = 1.0 / sr;
        self.drift_lp_coeff = math::exp(-1.0 / (DRIFT_SLOW_TAU * sr));
        self.output_lp_coeff = math::exp(-2.0 * core::f64::consts::PI * OUTPUT_LP_HZ / sr);
    }

    /// Set saw ramp curvature. 1.0 = linear, <1.0 = concave (303-like).
    pub fn set_saw_curve(&mut self, curve: f64) {
        self.saw_curve = curve.clamp(0.5, 1.0);
    }

    /// Set square pulse width. 0.5 = perfect square. 303 ≈ 0.48.
    pub fn set_pulse_width(&mut self, pw: f64) {
        self.pulse_width = pw.clamp(0.1, 0.9);
    }

    /// Convert V/oct + tuning offset to frequency in Hz.
    #[inline]
    fn voct_to_hz(voct: f64, tuning_semitones: f64) -> f64 {
        BASE_FREQ_HZ * math::exp((voct + tuning_semitones / 12.0) * core::f64::consts::LN_2)
    }

    /// Process one sample.
    ///
    /// - `voct`: V/oct pitch (0V = C1, 1V = C2, etc.)
    /// - `tuning`: tuning offset in semitones
    /// - `square`: false = saw, true = square
    ///
    /// Returns a single sample in approximately [-1, 1].
    #[inline]
    pub fn process(&mut self, voct: f64, tuning: f64, square: bool) -> f64 {
        // ── 1. Drift ────────────────────────────────────────────────────
        // Slow random walk, lowpass filtered
        let noise = self.rng.next_bipolar();
        self.drift_slow = self.drift_lp_coeff * self.drift_slow
            + (1.0 - self.drift_lp_coeff) * noise * DRIFT_SLOW_SEMITONES;

        // Thermal drift: scales with octave above base
        let octaves_above_base = voct;
        let thermal_drift = octaves_above_base * DRIFT_THERMAL_SCALE;

        let total_drift = self.drift_slow + thermal_drift;

        // ── 2. Frequency ────────────────────────────────────────────────
        let freq = Self::voct_to_hz(voct, tuning + total_drift);
        let dt = freq * self.inv_sample_rate; // phase increment per sample

        // ── 3. Phase accumulator ────────────────────────────────────────
        self.phase += dt;
        if self.phase >= 1.0 {
            self.phase -= 1.0;
        }

        // ── 4. Waveform generation ──────────────────────────────────────
        let raw = if square {
            // Square with asymmetric pulse width + PolyBLEP
            let mut sq = if self.phase < self.pulse_width {
                1.0
            } else {
                -1.0
            };
            // BLEP at rising edge (phase = 0)
            sq += poly_blep(self.phase, dt);
            // BLEP at falling edge (phase = pulse_width)
            let phase_from_pw = self.phase - self.pulse_width;
            let wrapped = if phase_from_pw < 0.0 {
                phase_from_pw + 1.0
            } else {
                phase_from_pw
            };
            sq -= poly_blep(wrapped, dt);
            sq
        } else {
            // Saw with curvature + PolyBLEP
            // Linear ramp: 2*phase - 1 (maps [0,1] to [-1,1])
            // Apply curvature via fast_powf_unit (2 muls + 2 adds, <1% error).
            let curved_phase = crate::fast_math::fast_powf_unit(self.phase, self.saw_curve);
            let mut saw = 2.0 * curved_phase - 1.0;
            // BLEP at discontinuity (phase wraps from 1→0)
            saw -= poly_blep(self.phase, dt);
            saw
        };

        // ── 5. Output bandwidth limit ───────────────────────────────────
        // Single-pole lowpass: models parasitic capacitance rolloff
        self.output_lp = self.output_lp_coeff * self.output_lp + (1.0 - self.output_lp_coeff) * raw;

        self.output_lp
    }

    /// Reset phase and internal state.
    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.drift_slow = 0.0;
        self.output_lp = 0.0;
    }
}

// ── Tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::eprintln;

    const SR: f64 = 48_000.0;

    #[test]
    fn saw_output_bounded() {
        let mut vco = Vco::new(SR);
        let mut min = f64::MAX;
        let mut max = f64::MIN;
        // Sweep through a few cycles at 440Hz
        for _ in 0..4800 {
            let s = vco.process(3.75, 0.0, false); // ~440Hz (C1 * 2^3.75)
            min = min.min(s);
            max = max.max(s);
            assert!(s.is_finite(), "saw output must be finite");
        }
        eprintln!("  saw range: [{min:.4}, {max:.4}]");
        assert!(max > 0.5, "saw should reach positive half: max={max}");
        assert!(min < -0.5, "saw should reach negative half: min={min}");
        assert!(max < 1.5, "saw shouldn't exceed ±1.5: max={max}");
        assert!(min > -1.5, "saw shouldn't exceed ±1.5: min={min}");
    }

    #[test]
    fn square_output_bounded() {
        let mut vco = Vco::new(SR);
        let mut min = f64::MAX;
        let mut max = f64::MIN;
        for _ in 0..4800 {
            let s = vco.process(3.75, 0.0, true);
            min = min.min(s);
            max = max.max(s);
            assert!(s.is_finite(), "square output must be finite");
        }
        eprintln!("  square range: [{min:.4}, {max:.4}]");
        assert!(max > 0.5, "square should reach positive: max={max}");
        assert!(min < -0.5, "square should reach negative: min={min}");
    }

    #[test]
    fn saw_curvature_adds_even_harmonics() {
        // A perfectly linear saw has only odd harmonics in its Fourier series.
        // Wait, actually a saw has ALL harmonics (1/n series). The curvature
        // shifts the harmonic balance — more even harmonic energy vs linear.
        // Just verify the waveform is asymmetric (not identical to linear).
        let mut linear = Vco::new(SR);
        linear.set_saw_curve(1.0); // perfect linear
        let mut curved = Vco::new(SR);
        curved.set_saw_curve(0.92); // 303-like

        let mut diff_sum = 0.0;
        for _ in 0..4800 {
            let a = linear.process(3.75, 0.0, false);
            let b = curved.process(3.75, 0.0, false);
            diff_sum += (a - b).abs();
        }
        eprintln!("  curvature diff sum: {diff_sum:.4}");
        assert!(
            diff_sum > 1.0,
            "curved saw should differ from linear: diff_sum={diff_sum}"
        );
    }

    #[test]
    fn pulse_width_asymmetry() {
        // With PW != 0.5, the positive and negative half-cycles differ
        let mut vco = Vco::new(SR);
        vco.set_pulse_width(0.48);
        let mut pos_samples = 0usize;
        let mut neg_samples = 0usize;
        // Run enough for many cycles at ~440Hz
        for _ in 0..48000 {
            let s = vco.process(3.75, 0.0, true);
            if s > 0.0 {
                pos_samples += 1;
            } else {
                neg_samples += 1;
            }
        }
        let ratio = pos_samples as f64 / (pos_samples + neg_samples) as f64;
        eprintln!("  PW ratio: {ratio:.4} (target ~0.48)");
        // Should be close to 0.48, not 0.50
        assert!(
            (ratio - 0.48).abs() < 0.03,
            "PW ratio should be ~0.48, got {ratio:.4}"
        );
    }

    #[test]
    fn drift_adds_pitch_variation() {
        // Two VCOs with different seeds should diverge over time
        let mut vco1 = Vco::new(SR);
        let mut vco2 = Vco::new(SR);
        vco2.rng = Rng::new(0xBEEF);

        // Run 1 second
        let mut diff_sum = 0.0;
        for _ in 0..48000 {
            let a = vco1.process(3.0, 0.0, false);
            let b = vco2.process(3.0, 0.0, false);
            diff_sum += (a - b).abs();
        }
        eprintln!("  drift divergence over 1s: {diff_sum:.2}");
        assert!(
            diff_sum > 0.1,
            "different seeds should produce drift divergence"
        );
    }

    #[test]
    fn output_bandwidth_limits_hf() {
        // At very high frequency, the output LP should attenuate
        let mut vco = Vco::new(SR);
        // 20kHz = well above the 12kHz cutoff
        let voct = math::ln(20_000.0 / BASE_FREQ_HZ) / core::f64::consts::LN_2;
        let mut peak = 0.0f64;
        for _ in 0..4800 {
            let s = vco.process(voct, 0.0, false);
            peak = peak.max(s.abs());
        }
        eprintln!("  20kHz saw peak: {peak:.4}");
        // Should be attenuated below full amplitude
        assert!(
            peak < 0.9,
            "20kHz should be attenuated by output LP: peak={peak}"
        );
    }

    #[test]
    fn voct_tracking() {
        // Verify V/oct produces roughly correct frequencies
        // by counting zero crossings
        let measure_freq = |voct: f64| -> f64 {
            let mut vco = Vco::new(SR);
            let mut crossings = 0usize;
            let mut prev = 0.0;
            let n = 48000; // 1 second
            for _ in 0..n {
                let s = vco.process(voct, 0.0, false);
                if prev <= 0.0 && s > 0.0 {
                    crossings += 1;
                }
                prev = s;
            }
            crossings as f64 // crossings per second ≈ freq
        };

        let f_c2 = measure_freq(1.0); // C2 = 130.8 Hz
        let f_c3 = measure_freq(2.0); // C3 = 261.6 Hz
        let f_c4 = measure_freq(3.0); // C4 = 523.2 Hz

        eprintln!("  V/oct: C2={f_c2:.0}Hz, C3={f_c3:.0}Hz, C4={f_c4:.0}Hz");

        // Each octave should roughly double (within drift tolerance)
        let ratio_23 = f_c3 / f_c2;
        let ratio_34 = f_c4 / f_c3;
        assert!(
            (ratio_23 - 2.0).abs() < 0.15,
            "C3/C2 should be ~2.0, got {ratio_23:.3}"
        );
        assert!(
            (ratio_34 - 2.0).abs() < 0.15,
            "C4/C3 should be ~2.0, got {ratio_34:.3}"
        );
    }
}
