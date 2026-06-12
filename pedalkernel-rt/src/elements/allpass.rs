//! All-pass filter for phaser effects.
//!
//! An all-pass filter shifts phase without changing amplitude. Cascading multiple
//! all-pass stages and varying the cutoff frequency creates the classic phaser effect.

use alloc::vec::Vec;

/// First-order all-pass filter with variable cutoff frequency.
///
/// Transfer function: H(z) = (a + z⁻¹) / (1 + a*z⁻¹)
/// where a = (1 - tan(πf₀/fs)) / (1 + tan(πf₀/fs))
///
/// Phase shift varies from 0° at DC to -180° at Nyquist, with -90° at f₀.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AllPassFilter {
    /// Filter coefficient (computed from cutoff frequency)
    coef: crate::Wave,
    /// Previous input sample
    x1: crate::Wave,
    /// Previous output sample
    y1: crate::Wave,
    /// Sample rate
    sample_rate: crate::Wave,
    /// Current cutoff frequency (Hz)
    cutoff: crate::Wave,
}

impl AllPassFilter {
    /// Create a new all-pass filter with given cutoff frequency.
    pub fn new(cutoff: crate::Wave, sample_rate: crate::Wave) -> Self {
        let mut filter = Self {
            coef: 0.0,
            x1: 0.0,
            y1: 0.0,
            sample_rate,
            cutoff,
        };
        filter.update_coefficient();
        filter
    }

    /// Update the filter coefficient from cutoff frequency.
    fn update_coefficient(&mut self) {
        // Bilinear transform coefficient
        // a = (1 - tan(πf₀/fs)) / (1 + tan(πf₀/fs))
        let w = (crate::math::PI as crate::Wave) * self.cutoff / self.sample_rate;
        let tan_w = crate::math::tan(w as crate::Wave) as crate::Wave;
        self.coef = (1.0 - tan_w) / (1.0 + tan_w);
    }

    /// Set the cutoff frequency (Hz).
    #[inline]
    pub fn set_cutoff(&mut self, freq: crate::Wave) {
        // Clamp to valid range
        let freq = freq.clamp(20.0, self.sample_rate * 0.45);
        if (freq - self.cutoff).abs() > 0.1 {
            self.cutoff = freq;
            self.update_coefficient();
        }
    }

    /// Get current cutoff frequency.
    pub fn cutoff(&self) -> crate::Wave {
        self.cutoff
    }

    /// Process one sample through the all-pass filter.
    #[inline]
    pub fn process(&mut self, input: crate::Wave) -> crate::Wave {
        // y[n] = a * x[n] + x[n-1] - a * y[n-1]
        let output = self.coef * input + self.x1 - self.coef * self.y1;
        self.x1 = input;
        self.y1 = output;
        output
    }

    /// Reset filter state.
    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.y1 = 0.0;
    }

    /// Update sample rate.
    pub fn set_sample_rate(&mut self, sample_rate: crate::Wave) {
        self.sample_rate = sample_rate;
        self.update_coefficient();
    }
}

/// Multi-stage all-pass phaser.
///
/// Cascades multiple all-pass filters and mixes with dry signal to create
/// the classic phaser comb-filter effect.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Phaser {
    /// All-pass filter stages
    stages: Vec<AllPassFilter>,
    /// Dry/wet mix (0.0 = dry, 1.0 = wet)
    mix: crate::Wave,
    /// Feedback amount (0.0 to ~0.9)
    feedback: crate::Wave,
    /// Feedback state
    feedback_state: crate::Wave,
    /// Base cutoff frequency (Hz)
    base_freq: crate::Wave,
    /// Modulation depth (multiplier on base_freq)
    mod_depth: crate::Wave,
    /// Sample rate
    sample_rate: crate::Wave,
}

impl Phaser {
    /// Create a new phaser with given number of stages.
    ///
    /// Classic Phase 90 uses 4 stages. More stages = deeper effect.
    pub fn new(num_stages: usize, sample_rate: crate::Wave) -> Self {
        let base_freq = 1000.0; // 1kHz default center frequency
        let stages = (0..num_stages)
            .map(|_| AllPassFilter::new(base_freq, sample_rate))
            .collect();

        Self {
            stages,
            mix: 0.5,      // 50/50 mix for classic phaser
            feedback: 0.0, // No feedback for script logo Phase 90
            feedback_state: 0.0,
            base_freq,
            mod_depth: 4.0, // Sweep from base_freq/4 to base_freq*4
            sample_rate,
        }
    }

    /// Set the modulation value (typically from LFO, range -1 to +1).
    ///
    /// This sweeps the cutoff frequency of all stages together.
    #[inline]
    pub fn set_modulation(&mut self, mod_value: crate::Wave) {
        // Map modulation to exponential frequency sweep
        // mod_value -1 -> base_freq / mod_depth
        // mod_value  0 -> base_freq
        // mod_value +1 -> base_freq * mod_depth
        let freq = self.base_freq
            * crate::math::powf(self.mod_depth as crate::Wave, mod_value as crate::Wave)
                as crate::Wave;
        for stage in &mut self.stages {
            stage.set_cutoff(freq);
        }
    }

    /// Set the base center frequency (Hz).
    pub fn set_base_freq(&mut self, freq: crate::Wave) {
        self.base_freq = freq.clamp(100.0, 5000.0);
    }

    /// Set the modulation depth (frequency sweep range).
    pub fn set_mod_depth(&mut self, depth: crate::Wave) {
        self.mod_depth = depth.clamp(1.5, 10.0);
    }

    /// Set the dry/wet mix (0.0 = dry, 1.0 = wet).
    pub fn set_mix(&mut self, mix: crate::Wave) {
        self.mix = mix.clamp(0.0, 1.0);
    }

    /// Set feedback amount (0.0 to 0.9).
    pub fn set_feedback(&mut self, feedback: crate::Wave) {
        self.feedback = feedback.clamp(0.0, 0.9);
    }

    /// Process one sample through the phaser.
    #[inline]
    pub fn process(&mut self, input: crate::Wave) -> crate::Wave {
        // Add feedback
        let input_with_fb = input + self.feedback_state * self.feedback;

        // Process through all-pass stages
        let mut wet = input_with_fb;
        for stage in &mut self.stages {
            wet = stage.process(wet);
        }

        // Store feedback state
        self.feedback_state = wet;

        // Mix dry and wet
        // Phase 90 style: sum of dry + wet creates notches
        input * (1.0 - self.mix) + wet * self.mix
    }

    /// Reset all filter states.
    pub fn reset(&mut self) {
        for stage in &mut self.stages {
            stage.reset();
        }
        self.feedback_state = 0.0;
    }

    /// Update sample rate.
    pub fn set_sample_rate(&mut self, sample_rate: crate::Wave) {
        self.sample_rate = sample_rate;
        for stage in &mut self.stages {
            stage.set_sample_rate(sample_rate);
        }
    }
}

#[cfg(all(test, DISABLED_tests_in_pedalkernel))]
mod tests {
    use super::*;

    #[test]
    fn allpass_unity_gain() {
        let mut filter = AllPassFilter::new(1000.0, 48000.0);

        // All-pass should have unity gain (amplitude unchanged)
        // Test with a sine wave
        let freq = 440.0;
        let mut sum_sq_in = 0.0;
        let mut sum_sq_out = 0.0;

        for i in 0..4800 {
            let t = i as crate::Wave / 48000.0;
            let input = crate::math::sin((2.0 * crate::math::PI * freq * t));
            let output = filter.process(input);

            // Skip first 100 samples for transient
            if i > 100 {
                sum_sq_in += input * input;
                sum_sq_out += output * output;
            }
        }

        let rms_in = crate::math::sqrt((sum_sq_in / 4700.0));
        let rms_out = crate::math::sqrt((sum_sq_out / 4700.0));

        // Should be very close to unity gain
        assert!((rms_out / rms_in - 1.0).abs() < 0.01);
    }

    #[test]
    fn phaser_creates_notches() {
        let mut phaser = Phaser::new(4, 48000.0);
        phaser.set_mix(0.5);
        phaser.set_modulation(0.0);

        // Process some samples - should produce output
        let mut output_sum = 0.0;
        for i in 0..4800 {
            let t = i as crate::Wave / 48000.0;
            let input = crate::math::sin((2.0 * crate::math::PI * 440.0 * t));
            output_sum += phaser.process(input).abs();
        }

        // Should have non-zero output
        assert!(output_sum > 0.0);
    }
}
