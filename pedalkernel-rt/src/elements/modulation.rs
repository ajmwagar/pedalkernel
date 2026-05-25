//! Modulation sources: LFO, EnvelopeFollower.
//!
//! These elements generate control signals for modulating other elements
//! (tremolo depth, phaser sweep, auto-wah, etc.).

use super::Modulator;

// ---------------------------------------------------------------------------
// LFO
// ---------------------------------------------------------------------------

/// LFO waveform shapes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum LfoWaveform {
    /// Smooth sine wave - classic tremolo sound.
    Sine,
    /// Linear triangle wave - similar to sine but brighter.
    Triangle,
    /// Hard square wave - choppy, staccato effect.
    Square,
    /// Rising sawtooth - asymmetric, "ramping" feel.
    SawUp,
    /// Falling sawtooth - reverse ramp.
    SawDown,
    /// Random levels held for each cycle - sample-and-hold.
    SampleAndHold,
}

/// Low Frequency Oscillator for modulation effects.
///
/// Generates periodic control signals for tremolo, vibrato, phaser, etc.
/// Uses a phase accumulator for efficient, RT-safe operation.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Lfo {
    /// Current phase (0.0 - 1.0).
    phase: crate::Wave,
    /// Frequency in Hz.
    frequency: crate::Wave,
    /// Modulation depth (0.0 - 1.0).
    depth: crate::Wave,
    /// Waveform shape.
    waveform: LfoWaveform,
    /// Sample rate for phase increment calculation.
    sample_rate: crate::Wave,
    /// Phase increment per sample.
    phase_inc: crate::Wave,
    /// Held value for sample-and-hold mode.
    sh_value: crate::Wave,
    /// Simple RNG state for sample-and-hold.
    rng_state: u32,
}

impl Lfo {
    /// Create a new LFO with the given waveform and sample rate.
    pub fn new(waveform: LfoWaveform, sample_rate: crate::Wave) -> Self {
        let frequency = 5.0; // Default 5 Hz
        let phase_inc = frequency / sample_rate;
        Self {
            phase: 0.0,
            frequency,
            depth: 1.0,
            waveform,
            sample_rate,
            phase_inc,
            sh_value: 0.0,
            rng_state: 12345,
        }
    }

    /// Set modulation depth (0.0 = no modulation, 1.0 = full depth).
    #[inline]
    pub fn set_depth(&mut self, depth: crate::Wave) {
        self.depth = depth.clamp(0.0, 1.0);
    }

    /// Get the current depth.
    pub fn depth(&self) -> crate::Wave {
        self.depth
    }

    /// Set the waveform shape.
    pub fn set_waveform(&mut self, waveform: LfoWaveform) {
        self.waveform = waveform;
    }

    /// Get the current waveform.
    pub fn waveform(&self) -> LfoWaveform {
        self.waveform
    }

    /// Simple fast PRNG for sample-and-hold (xorshift).
    #[inline]
    fn next_random(&mut self) -> crate::Wave {
        let mut x = self.rng_state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng_state = x;
        // Convert to 0.0 - 1.0 range
        (x as crate::Wave) / (u32::MAX as crate::Wave)
    }

    /// Generate raw waveform value from phase (returns -1.0 to 1.0).
    #[inline]
    fn raw_waveform(&mut self, phase: crate::Wave) -> crate::Wave {
        match self.waveform {
            LfoWaveform::Sine => {
                crate::math::sin((phase * crate::math::TAU as crate::Wave) as crate::Wave)
                    as crate::Wave
            }
            LfoWaveform::Triangle => {
                // Triangle: rises from -1 to 1 in first half, falls in second
                if phase < 0.5 {
                    4.0 * phase - 1.0
                } else {
                    3.0 - 4.0 * phase
                }
            }
            LfoWaveform::Square => {
                if phase < 0.5 {
                    1.0
                } else {
                    -1.0
                }
            }
            LfoWaveform::SawUp => {
                // Rising saw: -1 at phase 0, +1 at phase 1
                2.0 * phase - 1.0
            }
            LfoWaveform::SawDown => {
                // Falling saw: +1 at phase 0, -1 at phase 1
                1.0 - 2.0 * phase
            }
            LfoWaveform::SampleAndHold => {
                // Value is held in sh_value, updated when phase wraps
                self.sh_value * 2.0 - 1.0 // Convert 0-1 to -1 to 1
            }
        }
    }

    /// Advance the LFO and return unipolar output (0.0 to depth).
    ///
    /// Useful for tremolo depth where 0 = no cut, depth = full cut.
    #[inline]
    pub fn tick_unipolar(&mut self) -> crate::Wave {
        (self.tick() + self.depth) * 0.5
    }

    /// Get current phase (0.0 - 1.0), useful for phase displays.
    pub fn phase(&self) -> crate::Wave {
        self.phase
    }

    /// Set phase directly (0.0 - 1.0), useful for sync.
    pub fn set_phase(&mut self, phase: crate::Wave) {
        self.phase =
            crate::math::rem_euclid(phase as crate::Wave, 1.0 as crate::Wave) as crate::Wave;
    }
}

impl Modulator for Lfo {
    /// Advance the LFO by one sample and return the output value.
    ///
    /// Returns a value in the range [-depth, +depth].
    /// For unipolar output (0 to depth), use `tick_unipolar()`.
    #[inline]
    fn tick(&mut self) -> crate::Wave {
        let value = self.raw_waveform(self.phase);

        // Advance phase
        self.phase += self.phase_inc;

        // Handle phase wrap
        if self.phase >= 1.0 {
            self.phase -= 1.0;
            // Update sample-and-hold on wrap
            if self.waveform == LfoWaveform::SampleAndHold {
                self.sh_value = self.next_random();
            }
        }

        value * self.depth
    }

    fn reset(&mut self) {
        self.phase = 0.0;
        self.sh_value = 0.0;
        self.rng_state = 12345;
    }

    #[inline]
    fn set_rate(&mut self, hz: crate::Wave) {
        self.frequency = hz.max(0.01);
        self.phase_inc = self.frequency / self.sample_rate;
    }

    fn rate(&self) -> crate::Wave {
        self.frequency
    }

    fn set_sample_rate(&mut self, sample_rate: crate::Wave) {
        self.sample_rate = sample_rate;
        self.phase_inc = self.frequency / sample_rate;
    }
}

// ---------------------------------------------------------------------------
// Envelope Follower
// ---------------------------------------------------------------------------

/// Envelope follower for dynamics-based modulation.
///
/// Tracks the amplitude envelope of an audio signal, producing a control
/// signal for auto-wah, compressor side-chains, or dynamic modulation.
/// Uses asymmetric attack/release smoothing for natural response.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct EnvelopeFollower {
    /// Current envelope value (0.0 - 1.0+).
    envelope: crate::Wave,
    /// Attack time constant coefficient.
    attack_coef: crate::Wave,
    /// Release time constant coefficient.
    release_coef: crate::Wave,
    /// Sample rate for coefficient calculation.
    sample_rate: crate::Wave,
    /// Attack time in milliseconds.
    attack_ms: crate::Wave,
    /// Release time in milliseconds.
    release_ms: crate::Wave,
    /// Input sensitivity/gain.
    sensitivity: crate::Wave,
}

impl EnvelopeFollower {
    /// Create a new envelope follower with default parameters.
    ///
    /// Default: 10ms attack, 100ms release, unity sensitivity.
    pub fn new(sample_rate: crate::Wave) -> Self {
        let mut ef = Self {
            envelope: 0.0,
            attack_coef: 0.0,
            release_coef: 0.0,
            sample_rate,
            attack_ms: 10.0,
            release_ms: 100.0,
            sensitivity: 1.0,
        };
        ef.update_coefficients();
        ef
    }

    /// Create an envelope follower from physical RC timing components.
    ///
    /// This mirrors real envelope detector circuits where attack and release
    /// time constants are set by RC networks, and sensitivity is set by
    /// an input gain resistor.
    ///
    /// - `attack_r`: Attack timing resistor (Ω)
    /// - `attack_c`: Attack timing capacitor (F)
    /// - `release_r`: Release timing resistor (Ω)
    /// - `release_c`: Release timing capacitor (F)
    /// - `sensitivity_r`: Sensitivity resistor (Ω) — gain = sensitivity_r / 10kΩ
    ///
    /// Time constants: τ_attack = R_attack × C_attack (in seconds),
    /// τ_release = R_release × C_release (in seconds).
    pub fn from_rc(
        attack_r: crate::Wave,
        attack_c: crate::Wave,
        release_r: crate::Wave,
        release_c: crate::Wave,
        sensitivity_r: crate::Wave,
        sample_rate: crate::Wave,
    ) -> Self {
        // τ = R × C gives seconds, convert to ms
        let attack_ms = attack_r * attack_c * 1000.0;
        let release_ms = release_r * release_c * 1000.0;
        // Sensitivity as a ratio against 10kΩ reference (like an OTA gain resistor)
        let sensitivity = sensitivity_r / 10_000.0;

        let mut ef = Self {
            envelope: 0.0,
            attack_coef: 0.0,
            release_coef: 0.0,
            sample_rate,
            attack_ms,
            release_ms,
            sensitivity,
        };
        ef.update_coefficients();
        ef
    }

    /// Compute smoothing coefficients from time constants.
    fn update_coefficients(&mut self) {
        // Convert ms to samples, then to exponential decay coefficient
        // coefficient = exp(-1 / (time_constant_in_samples))
        // For time constant τ: coef ≈ 1 - 1/(τ*fs/1000)
        let attack_samples = (self.attack_ms * self.sample_rate / 1000.0).max(1.0);
        let release_samples = (self.release_ms * self.sample_rate / 1000.0).max(1.0);

        self.attack_coef = crate::math::exp((-1.0 / attack_samples) as crate::Wave) as crate::Wave;
        self.release_coef =
            crate::math::exp((-1.0 / release_samples) as crate::Wave) as crate::Wave;
    }

    /// Set attack time in milliseconds.
    ///
    /// Shorter attack responds faster to transients but may track
    /// individual waveform cycles. Typical range: 1-50ms.
    pub fn set_attack(&mut self, ms: crate::Wave) {
        self.attack_ms = ms.max(0.1);
        self.update_coefficients();
    }

    /// Get current attack time in milliseconds.
    pub fn attack(&self) -> crate::Wave {
        self.attack_ms
    }

    /// Set release time in milliseconds.
    ///
    /// Longer release gives smoother output but slower response
    /// to level drops. Typical range: 50-500ms.
    pub fn set_release(&mut self, ms: crate::Wave) {
        self.release_ms = ms.max(0.1);
        self.update_coefficients();
    }

    /// Get current release time in milliseconds.
    pub fn release(&self) -> crate::Wave {
        self.release_ms
    }

    /// Set input sensitivity/gain.
    ///
    /// Higher sensitivity amplifies the input before envelope detection,
    /// useful for low-level signals. Typical range: 0.5-5.0.
    pub fn set_sensitivity(&mut self, sens: crate::Wave) {
        self.sensitivity = sens.max(0.0);
    }

    /// Get current sensitivity.
    pub fn sensitivity(&self) -> crate::Wave {
        self.sensitivity
    }

    /// Process one audio sample and return the envelope value.
    ///
    /// The envelope tracks the rectified (absolute value) input with
    /// asymmetric attack/release smoothing. Output range is 0.0 upward,
    /// typically staying below 1.0 for normalized audio input.
    #[inline]
    pub fn process(&mut self, input: crate::Wave) -> crate::Wave {
        // Full-wave rectify and apply sensitivity
        let rectified = input.abs() * self.sensitivity;

        // Asymmetric smoothing: fast attack, slow release
        let coef = if rectified > self.envelope {
            self.attack_coef
        } else {
            self.release_coef
        };

        // Exponential moving average
        self.envelope = coef * self.envelope + (1.0 - coef) * rectified;
        self.envelope
    }

    /// Get current envelope value without processing new input.
    pub fn envelope(&self) -> crate::Wave {
        self.envelope
    }

    /// Reset envelope to zero.
    pub fn reset(&mut self) {
        self.envelope = 0.0;
    }

    /// Update sample rate and recalculate coefficients.
    pub fn set_sample_rate(&mut self, sample_rate: crate::Wave) {
        self.sample_rate = sample_rate;
        self.update_coefficients();
    }
}
