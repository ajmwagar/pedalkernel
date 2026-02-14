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
pub struct Lfo {
    /// Current phase (0.0 - 1.0).
    phase: f64,
    /// Frequency in Hz.
    frequency: f64,
    /// Modulation depth (0.0 - 1.0).
    depth: f64,
    /// Waveform shape.
    waveform: LfoWaveform,
    /// Sample rate for phase increment calculation.
    sample_rate: f64,
    /// Phase increment per sample.
    phase_inc: f64,
    /// Held value for sample-and-hold mode.
    sh_value: f64,
    /// Simple RNG state for sample-and-hold.
    rng_state: u32,
}

impl Lfo {
    /// Create a new LFO with the given waveform and sample rate.
    pub fn new(waveform: LfoWaveform, sample_rate: f64) -> Self {
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
    pub fn set_depth(&mut self, depth: f64) {
        self.depth = depth.clamp(0.0, 1.0);
    }

    /// Get the current depth.
    pub fn depth(&self) -> f64 {
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
    fn next_random(&mut self) -> f64 {
        let mut x = self.rng_state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng_state = x;
        // Convert to 0.0 - 1.0 range
        (x as f64) / (u32::MAX as f64)
    }

    /// Generate raw waveform value from phase (returns -1.0 to 1.0).
    #[inline]
    fn raw_waveform(&mut self, phase: f64) -> f64 {
        match self.waveform {
            LfoWaveform::Sine => (phase * std::f64::consts::TAU).sin(),
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
    pub fn tick_unipolar(&mut self) -> f64 {
        (self.tick() + self.depth) * 0.5
    }

    /// Get current phase (0.0 - 1.0), useful for phase displays.
    pub fn phase(&self) -> f64 {
        self.phase
    }

    /// Set phase directly (0.0 - 1.0), useful for sync.
    pub fn set_phase(&mut self, phase: f64) {
        self.phase = phase.rem_euclid(1.0);
    }
}

impl Modulator for Lfo {
    /// Advance the LFO by one sample and return the output value.
    ///
    /// Returns a value in the range [-depth, +depth].
    /// For unipolar output (0 to depth), use `tick_unipolar()`.
    #[inline]
    fn tick(&mut self) -> f64 {
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
    fn set_rate(&mut self, hz: f64) {
        self.frequency = hz.max(0.01);
        self.phase_inc = self.frequency / self.sample_rate;
    }

    fn rate(&self) -> f64 {
        self.frequency
    }

    fn set_sample_rate(&mut self, sample_rate: f64) {
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
pub struct EnvelopeFollower {
    /// Current envelope value (0.0 - 1.0+).
    envelope: f64,
    /// Attack time constant coefficient.
    attack_coef: f64,
    /// Release time constant coefficient.
    release_coef: f64,
    /// Sample rate for coefficient calculation.
    sample_rate: f64,
    /// Attack time in milliseconds.
    attack_ms: f64,
    /// Release time in milliseconds.
    release_ms: f64,
    /// Input sensitivity/gain.
    sensitivity: f64,
}

impl EnvelopeFollower {
    /// Create a new envelope follower with default parameters.
    ///
    /// Default: 10ms attack, 100ms release, unity sensitivity.
    pub fn new(sample_rate: f64) -> Self {
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

    /// Create an envelope follower optimized for auto-wah.
    ///
    /// Fast attack (5ms) for quick response to pick attacks,
    /// medium release (150ms) for smooth filter sweep.
    pub fn auto_wah(sample_rate: f64) -> Self {
        let mut ef = Self::new(sample_rate);
        ef.set_attack(5.0);
        ef.set_release(150.0);
        ef.set_sensitivity(2.0); // Boost for typical guitar levels
        ef
    }

    /// Create an envelope follower optimized for compression.
    ///
    /// Very fast attack (1ms) to catch transients,
    /// slow release (300ms) for smooth gain reduction.
    pub fn compressor(sample_rate: f64) -> Self {
        let mut ef = Self::new(sample_rate);
        ef.set_attack(1.0);
        ef.set_release(300.0);
        ef.set_sensitivity(1.0);
        ef
    }

    /// Compute smoothing coefficients from time constants.
    fn update_coefficients(&mut self) {
        // Convert ms to samples, then to exponential decay coefficient
        // coefficient = exp(-1 / (time_constant_in_samples))
        // For time constant τ: coef ≈ 1 - 1/(τ*fs/1000)
        let attack_samples = (self.attack_ms * self.sample_rate / 1000.0).max(1.0);
        let release_samples = (self.release_ms * self.sample_rate / 1000.0).max(1.0);

        self.attack_coef = (-1.0 / attack_samples).exp();
        self.release_coef = (-1.0 / release_samples).exp();
    }

    /// Set attack time in milliseconds.
    ///
    /// Shorter attack responds faster to transients but may track
    /// individual waveform cycles. Typical range: 1-50ms.
    pub fn set_attack(&mut self, ms: f64) {
        self.attack_ms = ms.max(0.1);
        self.update_coefficients();
    }

    /// Get current attack time in milliseconds.
    pub fn attack(&self) -> f64 {
        self.attack_ms
    }

    /// Set release time in milliseconds.
    ///
    /// Longer release gives smoother output but slower response
    /// to level drops. Typical range: 50-500ms.
    pub fn set_release(&mut self, ms: f64) {
        self.release_ms = ms.max(0.1);
        self.update_coefficients();
    }

    /// Get current release time in milliseconds.
    pub fn release(&self) -> f64 {
        self.release_ms
    }

    /// Set input sensitivity/gain.
    ///
    /// Higher sensitivity amplifies the input before envelope detection,
    /// useful for low-level signals. Typical range: 0.5-5.0.
    pub fn set_sensitivity(&mut self, sens: f64) {
        self.sensitivity = sens.max(0.0);
    }

    /// Get current sensitivity.
    pub fn sensitivity(&self) -> f64 {
        self.sensitivity
    }

    /// Process one audio sample and return the envelope value.
    ///
    /// The envelope tracks the rectified (absolute value) input with
    /// asymmetric attack/release smoothing. Output range is 0.0 upward,
    /// typically staying below 1.0 for normalized audio input.
    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
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
    pub fn envelope(&self) -> f64 {
        self.envelope
    }

    /// Reset envelope to zero.
    pub fn reset(&mut self) {
        self.envelope = 0.0;
    }

    /// Update sample rate and recalculate coefficients.
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
        self.update_coefficients();
    }
}
