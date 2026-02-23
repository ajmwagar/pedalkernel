//! Synthesizer primitives: VCO, VCF, VCA.
//!
//! These are audio-rate synthesis building blocks modeled after classic
//! synth ICs (CEM3340, CEM3320, SSM2164).

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// VCO (Voltage-Controlled Oscillator)
// ---------------------------------------------------------------------------

/// VCO waveform outputs available from CEM3340-style oscillators.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VcoWaveform {
    Saw,
    Triangle,
    Pulse,
}

/// Audio-rate VCO modeled after the CEM3340/AS3340.
///
/// Features:
/// - 1V/octave exponential pitch tracking
/// - Saw, triangle, and pulse outputs
/// - Pulse width modulation
/// - Anti-aliased using PolyBLEP for saw/pulse
#[derive(Debug, Clone)]
pub struct Vco {
    /// Current phase (0.0 - 1.0)
    phase: f64,
    /// Base frequency in Hz (before CV modulation)
    base_freq: f64,
    /// Current frequency in Hz
    frequency: f64,
    /// Phase increment per sample
    phase_inc: f64,
    /// Sample rate
    sample_rate: f64,
    /// Pulse width (0.0 - 1.0, default 0.5 = square)
    pulse_width: f64,
    /// Last output values for each waveform
    last_saw: f64,
    last_tri: f64,
    last_pulse: f64,
}

impl Vco {
    /// Create a new VCO at the given sample rate.
    pub fn new(sample_rate: f64) -> Self {
        let base_freq = 440.0; // A4
        let phase_inc = base_freq / sample_rate;
        Self {
            phase: 0.0,
            base_freq,
            frequency: base_freq,
            phase_inc,
            sample_rate,
            pulse_width: 0.5,
            last_saw: 0.0,
            last_tri: 0.0,
            last_pulse: 0.0,
        }
    }

    /// Set the CV pitch input (1V/octave, 0V = base frequency).
    ///
    /// CV is in volts, with 0V = 440Hz (A4) by default.
    /// Each volt doubles the frequency.
    #[inline]
    pub fn set_cv_pitch(&mut self, cv: f64) {
        // 1V/octave: frequency = base_freq * 2^cv
        self.frequency = self.base_freq * (2.0_f64).powf(cv);
        self.phase_inc = self.frequency / self.sample_rate;
    }

    /// Set the base frequency (the frequency at 0V CV).
    pub fn set_base_freq(&mut self, freq: f64) {
        self.base_freq = freq.clamp(1.0, 20000.0);
        self.frequency = self.base_freq;
        self.phase_inc = self.frequency / self.sample_rate;
    }

    /// Set pulse width (0.0 - 1.0). 0.5 = square wave.
    pub fn set_pulse_width(&mut self, pw: f64) {
        self.pulse_width = pw.clamp(0.01, 0.99);
    }

    /// Get current frequency in Hz.
    pub fn frequency(&self) -> f64 {
        self.frequency
    }

    /// PolyBLEP correction for reducing aliasing at discontinuities.
    #[inline]
    fn poly_blep(&self, t: f64) -> f64 {
        let dt = self.phase_inc;
        if t < dt {
            // Rising edge
            let t_norm = t / dt;
            2.0 * t_norm - t_norm * t_norm - 1.0
        } else if t > 1.0 - dt {
            // Falling edge (wrap around)
            let t_norm = (t - 1.0) / dt;
            t_norm * t_norm + 2.0 * t_norm + 1.0
        } else {
            0.0
        }
    }

    /// Generate one sample and return all waveforms.
    ///
    /// Returns (saw, triangle, pulse) outputs in range [-1, 1].
    #[inline]
    pub fn tick(&mut self) -> (f64, f64, f64) {
        // Raw sawtooth (naive: 2*phase - 1)
        let mut saw = 2.0 * self.phase - 1.0;
        // Apply PolyBLEP antialiasing
        saw -= self.poly_blep(self.phase);

        // Triangle from integrated saw (leaky integrator approximation)
        // For proper triangle, we'd integrate the saw, but this is a
        // simple approximation that works well for synths
        let tri = if self.phase < 0.5 {
            4.0 * self.phase - 1.0
        } else {
            3.0 - 4.0 * self.phase
        };

        // Pulse wave with PolyBLEP
        let mut pulse = if self.phase < self.pulse_width {
            1.0
        } else {
            -1.0
        };
        // PolyBLEP at rising edge (phase = 0)
        pulse += self.poly_blep(self.phase);
        // PolyBLEP at falling edge (phase = pulse_width)
        let pw_phase = (self.phase - self.pulse_width).rem_euclid(1.0);
        pulse -= self.poly_blep(pw_phase);

        // Store outputs
        self.last_saw = saw;
        self.last_tri = tri;
        self.last_pulse = pulse;

        // Advance phase
        self.phase += self.phase_inc;
        if self.phase >= 1.0 {
            self.phase -= 1.0;
        }

        (saw, tri, pulse)
    }

    /// Get last saw output without advancing.
    pub fn saw(&self) -> f64 {
        self.last_saw
    }

    /// Get last triangle output without advancing.
    pub fn tri(&self) -> f64 {
        self.last_tri
    }

    /// Get last pulse output without advancing.
    pub fn pulse(&self) -> f64 {
        self.last_pulse
    }

    /// Reset oscillator state.
    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.last_saw = 0.0;
        self.last_tri = 0.0;
        self.last_pulse = 0.0;
    }

    /// Update sample rate.
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
        self.phase_inc = self.frequency / sample_rate;
    }
}

// ---------------------------------------------------------------------------
// VCF (Voltage-Controlled Filter)
// ---------------------------------------------------------------------------

/// 4-pole resonant lowpass filter modeled after the CEM3320/AS3320.
///
/// Uses a simplified Moog ladder topology with tanh saturation for
/// analog-like resonance and self-oscillation behavior.
#[derive(Debug, Clone)]
pub struct Vcf {
    /// Filter stages (4-pole = 4 one-pole sections)
    stage: [f64; 4],
    /// Cutoff frequency in Hz
    cutoff: f64,
    /// Resonance (0.0 - 1.0, self-oscillation above ~0.9)
    resonance: f64,
    /// Sample rate
    sample_rate: f64,
    /// Precomputed coefficient
    g: f64,
}

impl Vcf {
    /// Create a new VCF at the given sample rate.
    pub fn new(sample_rate: f64) -> Self {
        let mut vcf = Self {
            stage: [0.0; 4],
            cutoff: 1000.0,
            resonance: 0.0,
            sample_rate,
            g: 0.0,
        };
        vcf.update_coefficients();
        vcf
    }

    /// Update filter coefficients from cutoff frequency.
    fn update_coefficients(&mut self) {
        // Simplified coefficient calculation
        // g = tan(pi * fc / fs) for proper frequency warping
        let fc = self.cutoff.clamp(20.0, self.sample_rate * 0.45);
        self.g = (PI * fc / self.sample_rate).tan();
    }

    /// Set cutoff frequency via CV (1V/octave from 1kHz base).
    #[inline]
    pub fn set_cv_cutoff(&mut self, cv: f64) {
        // 1V/octave: cutoff = 1000 * 2^cv
        self.cutoff = 1000.0 * (2.0_f64).powf(cv);
        self.cutoff = self.cutoff.clamp(20.0, 20000.0);
        self.update_coefficients();
    }

    /// Set cutoff frequency directly in Hz.
    pub fn set_cutoff(&mut self, freq: f64) {
        self.cutoff = freq.clamp(20.0, 20000.0);
        self.update_coefficients();
    }

    /// Set resonance (0.0 - 1.0).
    pub fn set_resonance(&mut self, res: f64) {
        self.resonance = res.clamp(0.0, 1.0);
    }

    /// Get current cutoff frequency.
    pub fn cutoff(&self) -> f64 {
        self.cutoff
    }

    /// Process one sample through the filter.
    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
        // Resonance feedback (from 4th stage output back to input)
        let feedback = self.resonance * 4.0 * self.stage[3];

        // Input with resonance feedback, saturated
        let x = (input - feedback).tanh();

        // 4-pole ladder: each stage is a one-pole lowpass
        // y[n] = g * (x[n] - y[n-1]) + y[n-1]
        let g = self.g;
        let g1 = g / (1.0 + g);

        // Stage 1
        self.stage[0] += g1 * (x - self.stage[0]);
        // Stage 2
        self.stage[1] += g1 * (self.stage[0] - self.stage[1]);
        // Stage 3
        self.stage[2] += g1 * (self.stage[1] - self.stage[2]);
        // Stage 4
        self.stage[3] += g1 * (self.stage[2] - self.stage[3]);

        self.stage[3]
    }

    /// Reset filter state.
    pub fn reset(&mut self) {
        self.stage = [0.0; 4];
    }

    /// Update sample rate.
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
        self.update_coefficients();
    }
}

// ---------------------------------------------------------------------------
// VCA (Voltage-Controlled Amplifier)
// ---------------------------------------------------------------------------

/// Exponential VCA modeled after the SSM2164/V2164.
///
/// Features exponential CV response (dB/V) with soft saturation
/// at high levels.
#[derive(Debug, Clone)]
pub struct Vca {
    /// Current gain (0.0 - 1.0+)
    gain: f64,
    /// CV to gain conversion: dB per volt
    db_per_volt: f64,
}

impl Vca {
    /// Create a new VCA.
    pub fn new() -> Self {
        Self {
            gain: 1.0,
            db_per_volt: 20.0, // 20 dB/V is typical for SSM2164
        }
    }

    /// Set the CV control input.
    ///
    /// Exponential response: 0V = unity gain, negative = attenuation.
    /// Typical range: -5V (off) to 0V (unity).
    #[inline]
    pub fn set_cv(&mut self, cv: f64) {
        // Exponential: gain = 10^(cv * dB_per_volt / 20)
        // For SSM2164: -5V = -100dB (effectively off)
        let db = cv * self.db_per_volt;
        self.gain = (10.0_f64).powf(db / 20.0).clamp(0.0, 10.0);
    }

    /// Set gain directly (0.0 - 1.0+).
    pub fn set_gain(&mut self, gain: f64) {
        self.gain = gain.clamp(0.0, 10.0);
    }

    /// Get current gain.
    pub fn gain(&self) -> f64 {
        self.gain
    }

    /// Process one sample.
    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
        // Apply gain with soft saturation
        let output = input * self.gain;
        // Soft clip at high levels
        output.tanh()
    }

    /// Reset VCA state.
    pub fn reset(&mut self) {
        self.gain = 1.0;
    }
}

impl Default for Vca {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// ADSR Envelope Generator
// ---------------------------------------------------------------------------

/// ADSR envelope stages.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EnvelopeStage {
    Idle,
    Attack,
    Decay,
    Sustain,
    Release,
}

/// ADSR envelope generator for controlling VCA/VCF.
#[derive(Debug, Clone)]
pub struct AdsrEnvelope {
    /// Current stage
    stage: EnvelopeStage,
    /// Current envelope value (0.0 - 1.0)
    value: f64,
    /// Attack time in seconds
    attack: f64,
    /// Decay time in seconds
    decay: f64,
    /// Sustain level (0.0 - 1.0)
    sustain: f64,
    /// Release time in seconds
    release: f64,
    /// Sample rate
    sample_rate: f64,
    /// Gate state
    gate: bool,
    /// Attack coefficient
    attack_coef: f64,
    /// Decay coefficient
    decay_coef: f64,
    /// Release coefficient
    release_coef: f64,
}

impl AdsrEnvelope {
    /// Create a new ADSR envelope.
    pub fn new(sample_rate: f64) -> Self {
        let mut env = Self {
            stage: EnvelopeStage::Idle,
            value: 0.0,
            attack: 0.01,  // 10ms
            decay: 0.1,    // 100ms
            sustain: 0.7,
            release: 0.3,  // 300ms
            sample_rate,
            gate: false,
            attack_coef: 0.0,
            decay_coef: 0.0,
            release_coef: 0.0,
        };
        env.update_coefficients();
        env
    }

    /// Update coefficients from time values.
    fn update_coefficients(&mut self) {
        // Time constant: coef = exp(-1 / (time * sample_rate))
        self.attack_coef = (-1.0 / (self.attack.max(0.001) * self.sample_rate)).exp();
        self.decay_coef = (-1.0 / (self.decay.max(0.001) * self.sample_rate)).exp();
        self.release_coef = (-1.0 / (self.release.max(0.001) * self.sample_rate)).exp();
    }

    /// Set attack time in seconds.
    pub fn set_attack(&mut self, time: f64) {
        self.attack = time.max(0.001);
        self.update_coefficients();
    }

    /// Set decay time in seconds.
    pub fn set_decay(&mut self, time: f64) {
        self.decay = time.max(0.001);
        self.update_coefficients();
    }

    /// Set sustain level (0.0 - 1.0).
    pub fn set_sustain(&mut self, level: f64) {
        self.sustain = level.clamp(0.0, 1.0);
    }

    /// Set release time in seconds.
    pub fn set_release(&mut self, time: f64) {
        self.release = time.max(0.001);
        self.update_coefficients();
    }

    /// Trigger gate on (note on).
    pub fn gate_on(&mut self) {
        self.gate = true;
        self.stage = EnvelopeStage::Attack;
    }

    /// Trigger gate off (note off).
    pub fn gate_off(&mut self) {
        self.gate = false;
        if self.stage != EnvelopeStage::Idle {
            self.stage = EnvelopeStage::Release;
        }
    }

    /// Set gate state directly.
    pub fn set_gate(&mut self, gate: bool) {
        if gate && !self.gate {
            self.gate_on();
        } else if !gate && self.gate {
            self.gate_off();
        }
    }

    /// Get current envelope value.
    pub fn value(&self) -> f64 {
        self.value
    }

    /// Get current stage.
    pub fn stage(&self) -> EnvelopeStage {
        self.stage
    }

    /// Process one sample of the envelope.
    #[inline]
    pub fn tick(&mut self) -> f64 {
        match self.stage {
            EnvelopeStage::Idle => {
                self.value = 0.0;
            }
            EnvelopeStage::Attack => {
                // Exponential rise toward 1.0
                self.value = 1.0 - self.attack_coef * (1.0 - self.value);
                if self.value >= 0.999 {
                    self.value = 1.0;
                    self.stage = EnvelopeStage::Decay;
                }
            }
            EnvelopeStage::Decay => {
                // Exponential fall toward sustain level
                self.value = self.sustain + self.decay_coef * (self.value - self.sustain);
                if (self.value - self.sustain).abs() < 0.001 {
                    self.value = self.sustain;
                    self.stage = EnvelopeStage::Sustain;
                }
            }
            EnvelopeStage::Sustain => {
                self.value = self.sustain;
            }
            EnvelopeStage::Release => {
                // Exponential fall toward 0
                self.value = self.release_coef * self.value;
                if self.value < 0.001 {
                    self.value = 0.0;
                    self.stage = EnvelopeStage::Idle;
                }
            }
        }
        self.value
    }

    /// Reset envelope.
    pub fn reset(&mut self) {
        self.stage = EnvelopeStage::Idle;
        self.value = 0.0;
        self.gate = false;
    }

    /// Update sample rate.
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
        self.update_coefficients();
    }
}

// ---------------------------------------------------------------------------
// SynthProcessor - Complete synth voice (VCO + VCF + VCA + ADSR)
// ---------------------------------------------------------------------------

/// A complete monophonic synth voice combining VCO, VCF, VCA, and ADSR.
///
/// This provides a ready-to-use synth processor that can be controlled via
/// CV/gate signals (from MIDI or other sources).
#[derive(Debug, Clone)]
pub struct SynthProcessor {
    /// Oscillator
    vco: Vco,
    /// Filter
    vcf: Vcf,
    /// Amplifier
    vca: Vca,
    /// Amplitude envelope
    amp_env: AdsrEnvelope,
    /// Filter envelope
    filter_env: AdsrEnvelope,
    /// Filter envelope amount (how much the envelope affects cutoff)
    filter_env_amount: f64,
    /// Base filter cutoff frequency
    filter_base_cutoff: f64,
    /// Current note frequency (Hz)
    note_freq: f64,
    /// Current gate state
    gate: bool,
    /// Sample rate
    sample_rate: f64,
    /// Output level
    output_level: f64,
    /// Selected waveform (0=saw, 1=tri, 2=pulse)
    waveform_select: usize,
}

impl SynthProcessor {
    /// Create a new synth processor at the given sample rate.
    pub fn new(sample_rate: f64) -> Self {
        let mut vco = Vco::new(sample_rate);
        vco.set_base_freq(440.0);

        let mut vcf = Vcf::new(sample_rate);
        vcf.set_cutoff(2000.0);
        vcf.set_resonance(0.3);

        let mut amp_env = AdsrEnvelope::new(sample_rate);
        amp_env.set_attack(0.01);
        amp_env.set_decay(0.1);
        amp_env.set_sustain(0.7);
        amp_env.set_release(0.3);

        let mut filter_env = AdsrEnvelope::new(sample_rate);
        filter_env.set_attack(0.01);
        filter_env.set_decay(0.2);
        filter_env.set_sustain(0.3);
        filter_env.set_release(0.5);

        Self {
            vco,
            vcf,
            vca: Vca::new(),
            amp_env,
            filter_env,
            filter_env_amount: 3.0, // 3 octaves of filter sweep
            filter_base_cutoff: 500.0,
            note_freq: 440.0,
            gate: false,
            sample_rate,
            output_level: 0.8,
            waveform_select: 0, // Saw by default
        }
    }

    /// Set note on with MIDI note number.
    pub fn note_on(&mut self, midi_note: u8, _velocity: u8) {
        // Convert MIDI note to frequency: f = 440 * 2^((n-69)/12)
        self.note_freq = 440.0 * (2.0_f64).powf((midi_note as f64 - 69.0) / 12.0);
        self.vco.set_base_freq(self.note_freq);
        self.vco.set_cv_pitch(0.0); // Reset CV offset
        self.gate = true;
        self.amp_env.gate_on();
        self.filter_env.gate_on();
    }

    /// Set note off.
    pub fn note_off(&mut self) {
        self.gate = false;
        self.amp_env.gate_off();
        self.filter_env.gate_off();
    }

    /// Set CV pitch directly (1V/octave, 0V = 440Hz).
    pub fn set_cv_pitch(&mut self, cv: f64) {
        self.vco.set_cv_pitch(cv);
    }

    /// Set gate state directly.
    pub fn set_gate(&mut self, gate: bool) {
        if gate && !self.gate {
            self.gate = true;
            self.amp_env.gate_on();
            self.filter_env.gate_on();
        } else if !gate && self.gate {
            self.gate = false;
            self.amp_env.gate_off();
            self.filter_env.gate_off();
        }
    }

    /// Set filter cutoff frequency in Hz.
    pub fn set_filter_cutoff(&mut self, freq: f64) {
        self.filter_base_cutoff = freq.clamp(20.0, 20000.0);
    }

    /// Set filter resonance (0.0 - 1.0).
    pub fn set_filter_resonance(&mut self, res: f64) {
        self.vcf.set_resonance(res);
    }

    /// Set filter envelope amount in octaves.
    pub fn set_filter_env_amount(&mut self, octaves: f64) {
        self.filter_env_amount = octaves.clamp(-5.0, 5.0);
    }

    /// Set amplitude envelope attack time in seconds.
    pub fn set_attack(&mut self, time: f64) {
        self.amp_env.set_attack(time);
        self.filter_env.set_attack(time);
    }

    /// Set amplitude envelope decay time in seconds.
    pub fn set_decay(&mut self, time: f64) {
        self.amp_env.set_decay(time);
        self.filter_env.set_decay(time);
    }

    /// Set amplitude envelope sustain level (0.0 - 1.0).
    pub fn set_sustain(&mut self, level: f64) {
        self.amp_env.set_sustain(level);
    }

    /// Set amplitude envelope release time in seconds.
    pub fn set_release(&mut self, time: f64) {
        self.amp_env.set_release(time);
        self.filter_env.set_release(time);
    }

    /// Set output level (0.0 - 1.0).
    pub fn set_output_level(&mut self, level: f64) {
        self.output_level = level.clamp(0.0, 1.0);
    }

    /// Set waveform (0=saw, 1=tri, 2=pulse).
    pub fn set_waveform(&mut self, waveform: usize) {
        self.waveform_select = waveform.min(2);
    }

    /// Set pulse width (0.0 - 1.0, only affects pulse wave).
    pub fn set_pulse_width(&mut self, pw: f64) {
        self.vco.set_pulse_width(pw);
    }

    /// Process one sample and return the output.
    #[inline]
    pub fn process(&mut self) -> f64 {
        // Generate oscillator waveforms
        let (saw, tri, pulse) = self.vco.tick();

        // Select waveform
        let osc_out = match self.waveform_select {
            0 => saw,
            1 => tri,
            _ => pulse,
        };

        // Process filter envelope
        let filter_env_val = self.filter_env.tick();
        // Modulate filter cutoff: base_cutoff * 2^(env * amount)
        let cutoff = self.filter_base_cutoff * (2.0_f64).powf(filter_env_val * self.filter_env_amount);
        self.vcf.set_cutoff(cutoff);

        // Filter the oscillator
        let filtered = self.vcf.process(osc_out);

        // Process amplitude envelope
        let amp_env_val = self.amp_env.tick();

        // Apply VCA with envelope
        self.vca.set_gain(amp_env_val);
        let output = self.vca.process(filtered);

        output * self.output_level
    }

    /// Process audio input through the synth (for external audio processing).
    /// The synth VCF and VCA will process the input.
    #[inline]
    pub fn process_audio(&mut self, input: f64) -> f64 {
        // For audio input mode, just apply filter and envelope
        let filter_env_val = self.filter_env.tick();
        let cutoff = self.filter_base_cutoff * (2.0_f64).powf(filter_env_val * self.filter_env_amount);
        self.vcf.set_cutoff(cutoff);

        let filtered = self.vcf.process(input);
        let amp_env_val = self.amp_env.tick();
        self.vca.set_gain(amp_env_val);

        self.vca.process(filtered) * self.output_level
    }

    /// Reset the synth state.
    pub fn reset(&mut self) {
        self.vco.reset();
        self.vcf.reset();
        self.vca.reset();
        self.amp_env.reset();
        self.filter_env.reset();
        self.gate = false;
    }

    /// Update sample rate.
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
        self.vco.set_sample_rate(sample_rate);
        self.vcf.set_sample_rate(sample_rate);
        self.amp_env.set_sample_rate(sample_rate);
        self.filter_env.set_sample_rate(sample_rate);
    }

    /// Set a control by label (for compatibility with pedal interface).
    pub fn set_control(&mut self, label: &str, value: f64) {
        let value = value.clamp(0.0, 1.0);
        match label.to_ascii_lowercase().as_str() {
            "cutoff" | "filter" => {
                // Map 0-1 to 20Hz - 20kHz (exponential)
                let freq = 20.0 * (1000.0_f64).powf(value);
                self.set_filter_cutoff(freq);
            }
            "resonance" | "res" | "q" => {
                self.set_filter_resonance(value);
            }
            "attack" => {
                // Map 0-1 to 1ms - 2s (exponential)
                let time = 0.001 * (2000.0_f64).powf(value);
                self.set_attack(time);
            }
            "decay" => {
                let time = 0.001 * (2000.0_f64).powf(value);
                self.set_decay(time);
            }
            "sustain" => {
                self.set_sustain(value);
            }
            "release" => {
                let time = 0.001 * (5000.0_f64).powf(value);
                self.set_release(time);
            }
            "level" | "volume" | "output" => {
                self.set_output_level(value);
            }
            "waveform" | "wave" | "osc" => {
                self.set_waveform((value * 3.0) as usize);
            }
            "pulsewidth" | "pw" | "width" => {
                self.set_pulse_width(0.1 + value * 0.8); // 10% - 90%
            }
            "envamount" | "filterenv" | "envmod" => {
                // Map 0-1 to -5 to +5 octaves
                self.set_filter_env_amount(value * 10.0 - 5.0);
            }
            _ => {}
        }
    }

    /// Check if the synth is currently producing sound.
    pub fn is_active(&self) -> bool {
        self.gate || self.amp_env.stage() != EnvelopeStage::Idle
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vco_generates_waveforms() {
        let mut vco = Vco::new(48000.0);
        vco.set_base_freq(440.0);

        // Generate a few samples
        let mut saw_sum = 0.0;
        let mut tri_sum = 0.0;
        let mut pulse_sum = 0.0;

        for _ in 0..1000 {
            let (saw, tri, pulse) = vco.tick();
            saw_sum += saw.abs();
            tri_sum += tri.abs();
            pulse_sum += pulse.abs();
        }

        // All waveforms should have non-zero energy
        assert!(saw_sum > 0.0, "Saw waveform is silent");
        assert!(tri_sum > 0.0, "Triangle waveform is silent");
        assert!(pulse_sum > 0.0, "Pulse waveform is silent");
    }

    #[test]
    fn vco_cv_pitch_tracks_octaves() {
        let mut vco = Vco::new(48000.0);
        vco.set_base_freq(440.0);

        // 0V = 440Hz
        vco.set_cv_pitch(0.0);
        assert!((vco.frequency() - 440.0).abs() < 0.01);

        // +1V = 880Hz (one octave up)
        vco.set_cv_pitch(1.0);
        assert!((vco.frequency() - 880.0).abs() < 0.01);

        // -1V = 220Hz (one octave down)
        vco.set_cv_pitch(-1.0);
        assert!((vco.frequency() - 220.0).abs() < 0.01);
    }

    #[test]
    fn vcf_filters_signal() {
        let mut vcf = Vcf::new(48000.0);
        vcf.set_cutoff(200.0);
        vcf.set_resonance(0.0);

        // High frequency input should be attenuated
        let mut output_sum = 0.0;
        for i in 0..1000 {
            let input = (i as f64 * 0.5).sin(); // ~3.8kHz at 48kHz
            output_sum += vcf.process(input).abs();
        }

        // Should be significantly attenuated
        assert!(output_sum < 500.0, "High frequency not attenuated: {output_sum}");
    }

    #[test]
    fn vca_applies_gain() {
        let mut vca = Vca::new();

        // Unity gain
        vca.set_gain(1.0);
        assert!((vca.process(0.5) - 0.5_f64.tanh()).abs() < 0.01);

        // Half gain
        vca.set_gain(0.5);
        assert!((vca.process(1.0) - 0.5_f64.tanh()).abs() < 0.01);
    }

    #[test]
    fn adsr_envelope_stages() {
        let mut env = AdsrEnvelope::new(48000.0);
        env.set_attack(0.01);
        env.set_decay(0.01);
        env.set_sustain(0.5);
        env.set_release(0.01);

        // Initial state is idle
        assert_eq!(env.stage(), EnvelopeStage::Idle);
        assert_eq!(env.value(), 0.0);

        // Gate on triggers attack
        env.gate_on();
        assert_eq!(env.stage(), EnvelopeStage::Attack);

        // Run attack phase
        for _ in 0..500 {
            env.tick();
        }
        assert!(env.value() > 0.9, "Attack didn't reach peak: {}", env.value());

        // Run decay to sustain
        for _ in 0..500 {
            env.tick();
        }
        assert!(
            (env.value() - 0.5).abs() < 0.1,
            "Didn't reach sustain: {}",
            env.value()
        );

        // Gate off triggers release
        env.gate_off();
        assert_eq!(env.stage(), EnvelopeStage::Release);

        // Run release
        for _ in 0..1000 {
            env.tick();
        }
        assert!(env.value() < 0.01, "Didn't release: {}", env.value());
    }
}
