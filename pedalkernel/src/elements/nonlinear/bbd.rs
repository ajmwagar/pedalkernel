//! BBD (Bucket-Brigade Device) delay line model.
//!
//! Models MN3007, MN3207, MN3005 with companding, clock feedthrough, and leakage.

use super::delay::{DelayLine, Interpolation};

// ---------------------------------------------------------------------------
// BBD (Bucket-Brigade Device) Delay Line
// ---------------------------------------------------------------------------

/// BBD delay line model parameters.
///
/// Bucket-brigade devices (MN3007, MN3207, MN3005) are analog delay lines
/// used in classic chorus, flanger, and delay pedals. Each device has a
/// fixed number of stages and a clock frequency range that determines the
/// delay time.
///
/// The BBD introduces characteristic artifacts:
/// - **Clock noise**: high-frequency switching artifacts (filtered by LPF)
/// - **Bandwidth limiting**: Nyquist limit at half the clock frequency
/// - **Companding noise**: from the compander used to improve S/N ratio
/// - **Aliasing**: soft HF rolloff from the sample-and-hold nature
///
/// We model these as a delay buffer with a variable-rate reader,
/// anti-aliasing LPF, and subtle noise/distortion.
#[derive(Debug, Clone, Copy)]
pub struct BbdModel {
    /// Number of BBD stages. MN3207=1024, MN3007=1024, MN3005=4096.
    pub num_stages: usize,
    /// Minimum clock frequency (Hz). Determines maximum delay time.
    pub clock_min: f64,
    /// Maximum clock frequency (Hz). Determines minimum delay time.
    pub clock_max: f64,
    /// Signal bandwidth limit as fraction of clock (Nyquist ≈ 0.45 * fclk).
    pub bandwidth_ratio: f64,
    /// Noise floor (linear amplitude). BBDs are noisy devices.
    pub noise_floor: f64,
    /// Per-stage charge leakage rate (fraction lost per stage per clock period).
    /// Real BBDs lose charge through substrate leakage and junction capacitance.
    /// Typical: 0.0001–0.001 per stage, causing progressive HF loss at longer delays.
    pub leakage_per_stage: f64,
    /// Clock feedthrough amplitude (normalized).
    /// The switching clock couples into the analog signal path through parasitic
    /// capacitance in the MOSFET switches.  Audible as a high-pitched whine.
    /// Typical: -60 to -80 dB below signal = 0.001–0.0001.
    pub clock_feedthrough: f64,
    /// Compander tracking error.  Real BBD circuits use NE571/SA571 compander
    /// ICs to expand dynamic range.  The compressor (input) and expander (output)
    /// don't track perfectly — the rectifier time constants differ, causing
    /// "breathing" and pumping artifacts.  0.0 = perfect tracking, 1.0 = maximum error.
    pub compander_error: f64,
}

impl BbdModel {
    /// MN3207 — 1024-stage BBD, used in Boss CE-2 chorus.
    ///
    /// Clock range: 10kHz – 200kHz
    /// Delay range: 1024/(2*fclk) = 2.56ms – 51.2ms
    /// Relatively high leakage and clock feedthrough (consumer-grade).
    pub fn mn3207() -> Self {
        Self {
            num_stages: 1024,
            clock_min: 10_000.0,
            clock_max: 200_000.0,
            bandwidth_ratio: 0.45,
            noise_floor: 0.001,
            leakage_per_stage: 0.0005,
            clock_feedthrough: 0.0008,
            compander_error: 0.15,
        }
    }

    /// MN3007 — 1024-stage BBD, used in Boss DM-2 delay.
    ///
    /// Lower noise version of MN3207, same stage count.
    /// Better charge retention and lower clock feedthrough.
    /// Clock range: 10kHz – 100kHz
    /// Delay range: 5.12ms – 51.2ms
    pub fn mn3007() -> Self {
        Self {
            num_stages: 1024,
            clock_min: 10_000.0,
            clock_max: 100_000.0,
            bandwidth_ratio: 0.45,
            noise_floor: 0.0005,
            leakage_per_stage: 0.0003,
            clock_feedthrough: 0.0004,
            compander_error: 0.12,
        }
    }

    /// MN3005 — 4096-stage BBD, used in Boss DM-2 (long delay mode),
    /// Electro-Harmonix Memory Man.
    ///
    /// 4x more stages means leakage accumulates more, causing darker
    /// repeats at long delay times — the "warm decay" Memory Man sound.
    /// Clock range: 10kHz – 100kHz
    /// Delay range: 20.48ms – 204.8ms
    pub fn mn3005() -> Self {
        Self {
            num_stages: 4096,
            clock_min: 10_000.0,
            clock_max: 100_000.0,
            bandwidth_ratio: 0.45,
            noise_floor: 0.0008,
            leakage_per_stage: 0.0004,
            clock_feedthrough: 0.0003,
            compander_error: 0.10,
        }
    }

    /// Delay time in seconds for a given clock frequency.
    ///
    /// BBD delay = num_stages / (2 * fclk)
    /// The factor of 2 is because samples advance one stage per half-clock.
    pub fn delay_at_clock(&self, clock_hz: f64) -> f64 {
        self.num_stages as f64 / (2.0 * clock_hz)
    }

    /// Minimum delay time in seconds.
    pub fn min_delay(&self) -> f64 {
        self.delay_at_clock(self.clock_max)
    }

    /// Maximum delay time in seconds.
    pub fn max_delay(&self) -> f64 {
        self.delay_at_clock(self.clock_min)
    }
}

/// BBD delay line processor.
///
/// Implements a physically-modeled bucket-brigade delay with:
/// - Variable delay time (via clock frequency)
/// - Anti-alias filtering (models the BBD's Nyquist limit)
/// - Subtle soft clipping (BBDs clip gently at high levels)
/// - Noise injection (characteristic BBD hiss)
/// - Compander with proper envelope tracking
///
/// The delay buffer uses linear interpolation for fractional-sample
/// delay, modeling the smooth time modulation of chorus/flanger effects.
///
/// ## Compander Architecture
///
/// Real BBD companders (NE570/NE571) work by:
/// 1. Compressing the input signal using the INPUT envelope
/// 2. Delaying the compressed signal through the BBD
/// 3. Expanding using a DELAYED version of the compression envelope
/// 4. The tracking error between compression and expansion envelopes
///    causes characteristic "breathing" and "pumping" artifacts
///
/// The expander must use the delayed compression envelope, NOT an
/// independent envelope of the output signal, to achieve unity gain
/// in steady state while preserving the analog artifacts.
#[derive(Debug, Clone)]
pub struct BbdDelayLine {
    pub model: BbdModel,
    /// Inner delay line handling ring buffer, write position, and interpolation.
    inner: DelayLine,
    /// Current clock frequency (Hz).
    clock_freq: f64,
    /// Anti-alias filter state (simple one-pole LPF).
    lpf_state: f64,
    /// Anti-alias filter coefficient.
    lpf_coef: f64,
    /// Simple RNG state for noise injection.
    rng_state: u32,
    /// Feedback amount (0.0–1.0).
    feedback: f64,
    /// Previous output for feedback path.
    feedback_sample: f64,
    /// Leakage LPF coefficient: models accumulated charge loss across all stages.
    /// More stages and lower clock = more leakage = darker output.
    /// This is a one-pole LPF whose cutoff decreases with delay time.
    leakage_lpf_state: f64,
    leakage_lpf_coef: f64,
    /// Clock feedthrough oscillator phase (0..2π).
    clock_phase: f64,
    /// Clock feedthrough phase increment per sample.
    clock_phase_inc: f64,

    // ── Compander state ───────────────────────────────────────────────────
    /// Compression envelope: tracks input signal level for compressor gain.
    compander_env_in: f64,
    /// Delayed compression envelope buffer: small ring buffer that delays
    /// the compression envelope to match the audio delay through the BBD.
    /// The expander uses this delayed envelope to restore dynamic range.
    env_delay_buffer: Vec<f64>,
    /// Write position in envelope delay buffer.
    env_delay_write_pos: usize,
    /// Current envelope delay in samples (tracks audio delay).
    env_delay_samples: usize,
    /// Output envelope: tracks actual output level for computing tracking error.
    /// This is NOT used for expansion gain, only for artifact modeling.
    compander_env_out: f64,

    /// Compander attack/release coefficients.
    compander_attack: f64,
    compander_release: f64,
    /// Sample rate (cached for envelope delay calculations).
    sample_rate: f64,
}

impl BbdDelayLine {
    /// Create a new BBD delay line.
    ///
    /// Buffer size is determined by the maximum delay time at the current
    /// sample rate, plus headroom for interpolation.
    pub fn new(model: BbdModel, sample_rate: f64) -> Self {
        // Derive delay range from the BBD clock range.
        // Max clock → min delay, min clock → max delay.
        let min_delay = model.delay_at_clock(model.clock_max);
        let max_delay = model.delay_at_clock(model.clock_min);
        let mut inner = DelayLine::new(min_delay, max_delay, sample_rate, Interpolation::Linear);

        // Default to middle of clock range
        let clock_freq = (model.clock_min + model.clock_max) / 2.0;
        inner.set_delay_seconds(model.delay_at_clock(clock_freq));

        // Anti-alias LPF cutoff: bandwidth_ratio * clock_freq
        let lpf_cutoff = model.bandwidth_ratio * clock_freq;
        let lpf_coef = (-2.0 * std::f64::consts::PI * lpf_cutoff / sample_rate).exp();

        // Leakage LPF: models accumulated charge loss across all BBD stages.
        // Each stage loses `leakage_per_stage` of its charge per clock period.
        // Total loss scales with num_stages — longer delays sound darker.
        // The equivalent LPF cutoff decreases as delay increases.
        let leakage_lpf_coef = Self::compute_leakage_coef(&model, clock_freq, sample_rate);

        // Clock feedthrough: the BBD switching clock couples into the signal path.
        let clock_phase_inc = 2.0 * std::f64::consts::PI * clock_freq / sample_rate;

        // Compander: NE571-style attack/release time constants.
        // Attack ~5ms, release ~50ms (NE571 syllabic time constants).
        // The mismatch causes "breathing" artifacts.
        let compander_attack = (-1.0 / (0.005 * sample_rate)).exp();
        let compander_release = (-1.0 / (0.050 * sample_rate)).exp();

        // Envelope delay buffer: sized for maximum BBD delay time.
        // This delays the compression envelope to match the audio delay.
        let max_env_delay_samples = (max_delay * sample_rate) as usize + 16;
        let env_delay_buffer = vec![0.0; max_env_delay_samples];
        let env_delay_samples = (model.delay_at_clock(clock_freq) * sample_rate) as usize;

        Self {
            model,
            inner,
            clock_freq,
            lpf_state: 0.0,
            lpf_coef,
            rng_state: 42_u32,
            feedback: 0.0,
            feedback_sample: 0.0,
            leakage_lpf_state: 0.0,
            leakage_lpf_coef,
            clock_phase: 0.0,
            clock_phase_inc,
            compander_env_in: 0.0,
            env_delay_buffer,
            env_delay_write_pos: 0,
            env_delay_samples,
            compander_env_out: 0.0,
            compander_attack,
            compander_release,
            sample_rate,
        }
    }

    /// Compute the leakage LPF coefficient from model parameters and clock frequency.
    ///
    /// The charge leakage per stage accumulates over the total number of stages.
    /// At lower clock frequencies (longer delays), each stage holds charge longer,
    /// so more leaks.  The equivalent one-pole LPF cutoff is:
    ///   fc_leakage = -ln(1 - leakage_per_stage × N) × fclk / (2π)
    /// Clamped to prevent the cutoff from exceeding Nyquist.
    fn compute_leakage_coef(model: &BbdModel, clock_freq: f64, sample_rate: f64) -> f64 {
        let total_loss = (model.leakage_per_stage * model.num_stages as f64).min(0.99);
        // Map total charge retention to a LPF cutoff.
        // retention = 1 - total_loss = fraction of signal preserved.
        // Higher loss → lower cutoff → darker sound.
        let retention = 1.0 - total_loss;
        // Cutoff in Hz: scale with clock frequency (higher clock = shorter hold time = less leakage)
        let fc_leakage = -(retention.ln()) * clock_freq / (2.0 * std::f64::consts::PI);
        let fc_clamped = fc_leakage.min(sample_rate * 0.45); // Never exceed Nyquist
        (-2.0 * std::f64::consts::PI * fc_clamped / sample_rate).exp()
    }

    /// Set the clock frequency directly (Hz).
    ///
    /// This controls the delay time: delay = num_stages / (2 * fclk).
    /// Modulating this with an LFO creates chorus/flanger effects.
    pub fn set_clock(&mut self, clock_hz: f64) {
        self.clock_freq = clock_hz.clamp(self.model.clock_min, self.model.clock_max);
        self.inner.set_delay_seconds(self.model.delay_at_clock(self.clock_freq));

        let sample_rate = self.sample_rate;

        // Update anti-alias filter
        let lpf_cutoff = self.model.bandwidth_ratio * self.clock_freq;
        self.lpf_coef = (-2.0 * std::f64::consts::PI * lpf_cutoff / sample_rate).exp();

        // Update leakage LPF (more leakage at lower clock = longer delays)
        self.leakage_lpf_coef =
            Self::compute_leakage_coef(&self.model, self.clock_freq, sample_rate);

        // Update clock feedthrough frequency
        self.clock_phase_inc =
            2.0 * std::f64::consts::PI * self.clock_freq / sample_rate;

        // Update envelope delay to match audio delay
        let delay_secs = self.model.delay_at_clock(self.clock_freq);
        self.env_delay_samples =
            (delay_secs * sample_rate).round() as usize % self.env_delay_buffer.len();
    }

    /// Set delay time as a normalized value (0.0 = min delay, 1.0 = max delay).
    ///
    /// Maps logarithmically across the clock frequency range.
    pub fn set_delay_normalized(&mut self, norm: f64) {
        let norm = norm.clamp(0.0, 1.0);
        // Logarithmic interpolation between min and max clock
        // norm=0 -> max clock (min delay), norm=1 -> min clock (max delay)
        let log_min = self.model.clock_min.ln();
        let log_max = self.model.clock_max.ln();
        let clock = (log_max - norm * (log_max - log_min)).exp();
        self.set_clock(clock);
    }

    /// Set feedback amount (0.0 = no feedback, <1.0 for stability).
    pub fn set_feedback(&mut self, feedback: f64) {
        self.feedback = feedback.clamp(0.0, 0.95);
    }

    /// Process one sample through the BBD delay line.
    ///
    /// Signal chain (models real BBD circuit with NE571-style companding):
    /// 1. Compander input stage: compress dynamic range (NE571 compressor)
    /// 2. Store compression envelope in delay buffer
    /// 3. Soft clip to BBD voltage swing limit
    /// 4. Write to delay buffer (with feedback)
    /// 5. Read from buffer with interpolation
    /// 6. Charge leakage LPF (darker at longer delays)
    /// 7. Anti-alias LPF (BBD Nyquist bandwidth limit)
    /// 8. Clock feedthrough injection
    /// 9. Noise injection
    /// 10. Read DELAYED compression envelope for expander
    /// 11. Expand using delayed envelope (restores original dynamics)
    /// 12. Apply tracking error as small modulation artifact
    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
        // ══════════════════════════════════════════════════════════════
        // COMPANDER INPUT (COMPRESSOR)
        // ══════════════════════════════════════════════════════════════
        // Track input envelope with syllabic time constants (5ms attack, 50ms release)
        // Syllabic envelope detection with 5ms attack, 50ms release.
        let abs_in = input.abs();
        let coef_in = if abs_in > self.compander_env_in {
            self.compander_attack
        } else {
            self.compander_release
        };
        self.compander_env_in =
            coef_in * self.compander_env_in + (1.0 - coef_in) * abs_in;

        // Model analog noise floor - prevents expansion from true zero which causes pops.
        // Real NE571 companders have finite noise that prevents infinite expansion gain.
        const ANALOG_NOISE_FLOOR: f64 = 0.001; // ~-60dB
        self.compander_env_in = self.compander_env_in.max(ANALOG_NOISE_FLOOR);

        // Store compression envelope in delay buffer for expander to use later.
        // Key insight: the expander needs the
        // DELAYED compression envelope, not an independent output envelope.
        let buf_len = self.env_delay_buffer.len();
        self.env_delay_buffer[self.env_delay_write_pos] = self.compander_env_in;
        self.env_delay_write_pos = (self.env_delay_write_pos + 1) % buf_len;

        // Compress: reduce dynamic range by 2:1 (typical NE571 ratio).
        // gain = 1/sqrt(envelope) — louder signals get compressed more.
        let comp_gain = if self.compander_env_in > 0.001 {
            1.0 / self.compander_env_in.sqrt()
        } else {
            1.0
        };
        let compressed = input * comp_gain.min(10.0); // Limit expansion of quiet signals

        // ══════════════════════════════════════════════════════════════
        // BBD DELAY MEDIUM
        // ══════════════════════════════════════════════════════════════
        // Mix with feedback and soft-clip
        let write_sample = compressed + self.feedback * self.feedback_sample;
        let clipped = bbd_soft_clip(write_sample);

        // Write to inner delay line's buffer
        self.inner.write_direct(clipped);

        // Read with interpolation
        let delay_samples = self.inner.effective_delay_samples();
        let out_raw = self.inner.buffer_read(delay_samples, 0);

        // Advance write position
        self.inner.advance_write();

        // ══════════════════════════════════════════════════════════════
        // BBD ANALOG CHARACTERISTICS
        // ══════════════════════════════════════════════════════════════
        // Charge leakage LPF: darker at longer delays
        self.leakage_lpf_state = self.leakage_lpf_coef * self.leakage_lpf_state
            + (1.0 - self.leakage_lpf_coef) * out_raw;

        // Anti-alias LPF (BBD Nyquist bandwidth limit)
        self.lpf_state = self.lpf_coef * self.lpf_state
            + (1.0 - self.lpf_coef) * self.leakage_lpf_state;

        // Clock feedthrough
        self.clock_phase += self.clock_phase_inc;
        if self.clock_phase > 2.0 * std::f64::consts::PI {
            self.clock_phase -= 2.0 * std::f64::consts::PI;
        }
        let clock_bleed = self.model.clock_feedthrough * self.clock_phase.sin();

        // Noise injection
        let noise = self.next_noise() * self.model.noise_floor;

        let bbd_output = self.lpf_state + clock_bleed + noise;

        // ══════════════════════════════════════════════════════════════
        // COMPANDER OUTPUT (EXPANDER)
        // ══════════════════════════════════════════════════════════════
        // Read the DELAYED compression envelope. The expander should use
        // the same envelope the compressor used, delayed to match the audio.
        // This achieves unity gain in steady state.
        let read_pos = (self.env_delay_write_pos + buf_len - self.env_delay_samples) % buf_len;
        let delayed_env = self.env_delay_buffer[read_pos];

        // Base expansion gain from delayed compression envelope.
        // This restores the original dynamic range (ideally cancels compression).
        let base_exp_gain = if delayed_env > 0.001 {
            delayed_env.sqrt()
        } else {
            1.0
        };

        // ── Tracking Error ─────────────────────────────────────────────
        // Track the actual output envelope to compute tracking error.
        // The error is the difference between what the compressor saw and
        // what the expander sees, caused by:
        // - BBD signal attenuation (LPF, soft clip, leakage)
        // - Envelope detector time constant differences
        // - Clock rate effects (Claim 7: "error varies inversely with clock frequency")
        let abs_out = bbd_output.abs();
        let error_scale = 1.0 + self.model.compander_error;
        let coef_out = if abs_out > self.compander_env_out {
            self.compander_attack * error_scale
        } else {
            self.compander_release / error_scale
        };
        self.compander_env_out = coef_out.min(0.9999) * self.compander_env_out
            + (1.0 - coef_out.min(0.9999)) * abs_out;

        // Compute tracking error as gain modulation.
        // Error is larger when:
        // - Envelope is changing rapidly (Claim 6: "function of envelope slope")
        // - Clock frequency is lower (Claim 7: "varies inversely with clock frequency")
        let envelope_delta = (delayed_env - self.compander_env_out).abs();
        let clock_factor = self.model.clock_min / self.clock_freq; // Higher at lower clock

        // Scale error by compander_error parameter and clock factor.
        // This creates the characteristic "breathing" and "pumping" artifacts.
        let tracking_error = self.model.compander_error * envelope_delta * clock_factor;

        // Apply tracking error as gain modulation on top of base expansion.
        // Small perturbation that creates analog character without breaking unity gain.
        let error_modulation = 1.0 + tracking_error.min(0.5); // Limit to ±50% modulation

        let exp_gain = (base_exp_gain * error_modulation).min(10.0);
        let output = bbd_output * exp_gain;

        // Store for feedback (post-compander, as in real circuits)
        self.feedback_sample = output;

        output
    }

    /// Update sample rate and resize buffer.
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
        self.inner.set_sample_rate(sample_rate);
        self.inner.set_delay_seconds(self.model.delay_at_clock(self.clock_freq));

        let lpf_cutoff = self.model.bandwidth_ratio * self.clock_freq;
        self.lpf_coef = (-2.0 * std::f64::consts::PI * lpf_cutoff / sample_rate).exp();
        self.leakage_lpf_coef =
            Self::compute_leakage_coef(&self.model, self.clock_freq, sample_rate);
        self.clock_phase_inc =
            2.0 * std::f64::consts::PI * self.clock_freq / sample_rate;
        self.compander_attack = (-1.0 / (0.005 * sample_rate)).exp();
        self.compander_release = (-1.0 / (0.050 * sample_rate)).exp();

        // Resize envelope delay buffer for new sample rate
        let max_delay = self.model.delay_at_clock(self.model.clock_min);
        let max_env_delay_samples = (max_delay * sample_rate) as usize + 16;
        self.env_delay_buffer.resize(max_env_delay_samples, 0.0);
        self.env_delay_write_pos = 0;
        self.env_delay_samples =
            (self.model.delay_at_clock(self.clock_freq) * sample_rate) as usize
                % self.env_delay_buffer.len();
    }

    /// Reset all state.
    pub fn reset(&mut self) {
        self.inner.reset();
        self.lpf_state = 0.0;
        self.leakage_lpf_state = 0.0;
        self.feedback_sample = 0.0;
        self.clock_phase = 0.0;
        self.compander_env_in = 0.0;
        self.compander_env_out = 0.0;
        self.env_delay_write_pos = 0;
        for s in &mut self.env_delay_buffer {
            *s = 0.0;
        }
    }

    /// Get current delay time in seconds.
    pub fn delay_time(&self) -> f64 {
        self.inner.delay_time()
    }

    /// Get current clock frequency.
    pub fn clock_freq(&self) -> f64 {
        self.clock_freq
    }

    /// Simple fast PRNG for noise injection (xorshift).
    #[inline]
    fn next_noise(&mut self) -> f64 {
        let mut x = self.rng_state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng_state = x;
        // Convert to -1.0 to 1.0 range
        (x as f64 / u32::MAX as f64) * 2.0 - 1.0
    }
}

/// BBD soft clipping characteristic.
///
/// BBDs have a limited signal swing (~1V peak in typical circuits).
/// They clip more gently than op-amps, with a gradual compression
/// that adds warmth. We model this with a cubic soft clipper.
#[inline]
pub fn bbd_soft_clip(x: f64) -> f64 {
    if x.abs() < 1.0 {
        x - x * x * x / 3.0 // Cubic soft clip
    } else {
        x.signum() * 2.0 / 3.0 // Saturated at 2/3 (cubic limit)
    }
}

/// Tape magnetic hysteresis saturation.
///
/// Models the Jiles-Atherton hysteresis curve simplified to a modified tanh.
/// Tape doesn't clip sharply — it saturates asymptotically. The saturation
/// "knee" is softer than transistor or op-amp clipping.
///
/// `level` is the input amplitude at which the tape reaches ~90% saturation.
/// Typical values: 0.6-0.8 for different tape formulations.
#[inline]
pub fn tape_saturation(x: f64, level: f64) -> f64 {
    // Scale input so that level corresponds to ~90% saturation
    // tanh(1.47) ≈ 0.9, so we scale accordingly
    let scaled = x * 1.47 / level;
    level * scaled.tanh() / 1.47
}

// ===========================================================================
// Unit Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_RATE: f64 = 48000.0;

    // -----------------------------------------------------------------------
    // Helper: measure RMS of a buffer
    // -----------------------------------------------------------------------
    fn rms(buf: &[f64]) -> f64 {
        if buf.is_empty() {
            return 0.0;
        }
        (buf.iter().map(|x| x * x).sum::<f64>() / buf.len() as f64).sqrt()
    }

    // -----------------------------------------------------------------------
    // Helper: generate sine wave
    // -----------------------------------------------------------------------
    fn sine(freq_hz: f64, amplitude: f64, duration_secs: f64, sample_rate: f64) -> Vec<f64> {
        let n = (duration_secs * sample_rate) as usize;
        (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                amplitude * (2.0 * std::f64::consts::PI * freq_hz * t).sin()
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // Helper: measure THD (harmonics 2-8 vs fundamental)
    // -----------------------------------------------------------------------
    fn goertzel_power(buf: &[f64], sample_rate: f64, target_hz: f64) -> f64 {
        let n = buf.len() as f64;
        let k = (target_hz * n / sample_rate).round();
        let w = 2.0 * std::f64::consts::PI * k / n;
        let coeff = 2.0 * w.cos();

        let mut s1 = 0.0;
        let mut s2 = 0.0;

        for &x in buf {
            let s0 = x + coeff * s1 - s2;
            s2 = s1;
            s1 = s0;
        }

        let power = s1 * s1 + s2 * s2 - coeff * s1 * s2;
        power / (n * n / 4.0)
    }

    fn thd(buf: &[f64], sample_rate: f64, fundamental_hz: f64) -> f64 {
        let fund_power = goertzel_power(buf, sample_rate, fundamental_hz);
        if fund_power < 1e-30 {
            return 0.0;
        }

        let mut harmonic_power = 0.0;
        for h in 2..=8 {
            let freq = fundamental_hz * h as f64;
            if freq > sample_rate / 2.0 {
                break;
            }
            harmonic_power += goertzel_power(buf, sample_rate, freq);
        }

        (harmonic_power / fund_power).sqrt()
    }

    // =======================================================================
    // SOFT CLIP TESTS
    // =======================================================================

    #[test]
    fn soft_clip_unity_in_linear_region() {
        // Below saturation, should be nearly linear
        for x in [-0.5, -0.3, 0.0, 0.3, 0.5] {
            let y = bbd_soft_clip(x);
            // Cubic soft clip: y = x - x³/3
            let expected = x - x * x * x / 3.0;
            assert!(
                (y - expected).abs() < 1e-10,
                "soft_clip({}) = {}, expected {}",
                x,
                y,
                expected
            );
        }
    }

    #[test]
    fn soft_clip_saturates_at_high_levels() {
        // Above 1.0, should saturate at 2/3
        assert!((bbd_soft_clip(2.0) - 2.0 / 3.0).abs() < 1e-10);
        assert!((bbd_soft_clip(-2.0) + 2.0 / 3.0).abs() < 1e-10);
        assert!((bbd_soft_clip(10.0) - 2.0 / 3.0).abs() < 1e-10);
    }

    #[test]
    fn soft_clip_is_odd_symmetric() {
        for x in [0.1, 0.5, 0.9, 1.5] {
            let pos = bbd_soft_clip(x);
            let neg = bbd_soft_clip(-x);
            assert!(
                (pos + neg).abs() < 1e-10,
                "soft_clip({}) = {}, soft_clip({}) = {} (should be symmetric)",
                x,
                pos,
                -x,
                neg
            );
        }
    }

    // =======================================================================
    // COMPANDER TESTS - Characterize current behavior
    // =======================================================================

    #[test]
    fn compander_unity_gain_steady_state() {
        // With a steady-state sine, compander should (ideally) have unity gain.
        // This test measures how close we get.
        let mut bbd = BbdDelayLine::new(BbdModel::mn3207(), SAMPLE_RATE);
        bbd.set_clock(50000.0); // Mid-range clock

        // Let compander settle with signal
        let warmup = sine(440.0, 0.3, 0.5, SAMPLE_RATE);
        for &s in &warmup {
            bbd.process(s);
        }

        // Now measure gain
        let input = sine(440.0, 0.3, 0.5, SAMPLE_RATE);
        let output: Vec<f64> = input.iter().map(|&s| bbd.process(s)).collect();

        let input_rms = rms(&input);
        let output_rms = rms(&output);
        let gain_db = 20.0 * (output_rms / input_rms).log10();

        println!(
            "Compander steady-state gain: {:.2} dB (input RMS={:.4}, output RMS={:.4})",
            gain_db, input_rms, output_rms
        );

        // CHARACTERIZATION: Record current behavior
        // Ideal would be 0 dB, but some deviation is expected from:
        // - Delay line (signal takes time to pass through)
        // - LPF filtering
        // - Soft clipping
        //
        // CURRENT STATE: -11.9 dB loss! This is the core problem.
        // The compander is NOT achieving unity gain.
        // TODO: After fix, tighten this tolerance to < 3.0 dB
        if gain_db < -6.0 {
            println!(
                "BUG: Compander has {:.1} dB loss - should be near 0 dB",
                gain_db
            );
        }
        assert!(
            gain_db.abs() < 15.0,
            "Compander gain catastrophically wrong: {} dB",
            gain_db
        );
    }

    #[test]
    fn compander_breathing_on_transient() {
        // Fast attack, slow release should cause "breathing" on transients.
        // This test measures the envelope deviation.
        let mut bbd = BbdDelayLine::new(BbdModel::mn3207(), SAMPLE_RATE);
        bbd.set_clock(50000.0);

        // Create transient: sudden onset then decay
        let mut input = Vec::new();
        // Silence
        input.extend(vec![0.0; 4800]); // 100ms silence
        // Sudden attack
        for i in 0..24000 {
            // 500ms of signal
            let t = i as f64 / SAMPLE_RATE;
            let env = (1.0 - (-t * 20.0).exp()); // 50ms attack
            input.push(env * 0.5 * (2.0 * std::f64::consts::PI * 440.0 * t).sin());
        }
        // Decay
        for i in 0..24000 {
            let t = i as f64 / SAMPLE_RATE;
            let env = (-t * 3.0).exp(); // ~300ms decay
            input.push(env * 0.5 * (2.0 * std::f64::consts::PI * 440.0 * t).sin());
        }

        let output: Vec<f64> = input.iter().map(|&s| bbd.process(s)).collect();

        // Measure envelope deviation in chunks
        let chunk_size = 480; // 10ms
        let mut input_env = Vec::new();
        let mut output_env = Vec::new();

        for i in 0..(input.len() / chunk_size) {
            let start = i * chunk_size;
            let end = start + chunk_size;
            input_env.push(rms(&input[start..end]));
            output_env.push(rms(&output[start..end]));
        }

        // Calculate envelope tracking error
        let mut max_ratio = 0.0f64;
        let mut min_ratio = f64::MAX;

        for (i, (&inp, &out)) in input_env.iter().zip(output_env.iter()).enumerate() {
            if inp > 0.01 {
                // Only where there's signal
                let ratio = out / inp;
                max_ratio = max_ratio.max(ratio);
                min_ratio = min_ratio.min(ratio);
                if i < 20 || i > input_env.len() - 10 {
                    println!("Chunk {}: in={:.4}, out={:.4}, ratio={:.3}", i, inp, out, ratio);
                }
            }
        }

        let breathing_range = max_ratio / min_ratio.max(0.001);
        println!(
            "Compander breathing range: {:.2}x (max={:.3}, min={:.3})",
            breathing_range, max_ratio, min_ratio
        );

        // CHARACTERIZATION: Record current breathing behavior
        // High values indicate severe breathing artifacts
        // TODO: After fix, this should be closer to 1.0
        assert!(
            breathing_range < 100.0,
            "Compander breathing should be bounded: {}x",
            breathing_range
        );
    }

    #[test]
    fn compander_thd_at_different_levels() {
        // Measure THD at different input levels to characterize compander distortion
        let levels = [0.1, 0.3, 0.5, 0.7];

        println!("\nCompander THD vs Input Level:");
        println!("Level\tTHD%");

        for &level in &levels {
            let mut bbd = BbdDelayLine::new(BbdModel::mn3207(), SAMPLE_RATE);
            bbd.set_clock(100000.0); // Higher clock = shorter delay = less filtering

            // Warmup
            let warmup = sine(440.0, level, 0.2, SAMPLE_RATE);
            for &s in &warmup {
                bbd.process(s);
            }

            // Measure
            let input = sine(440.0, level, 0.5, SAMPLE_RATE);
            let output: Vec<f64> = input.iter().map(|&s| bbd.process(s)).collect();

            let output_thd = thd(&output, SAMPLE_RATE, 440.0);
            println!("{:.1}\t{:.2}%", level, output_thd * 100.0);

            // CHARACTERIZATION: Record current THD behavior
            assert!(output_thd < 0.5, "THD should be reasonable: {:.1}%", output_thd * 100.0);
        }
    }

    // =======================================================================
    // CASCADED BBD TESTS - The main problem area
    // =======================================================================

    #[test]
    fn cascaded_bbd_thd_accumulation() {
        // Process through TWO BBDs in series - models Walrus Slo
        // This should reveal the compander cascade problem

        let mut bbd1 = BbdDelayLine::new(BbdModel::mn3005(), SAMPLE_RATE);
        let mut bbd2 = BbdDelayLine::new(BbdModel::mn3007(), SAMPLE_RATE);

        // Set different clock rates (like Slo's multi-tap)
        bbd1.set_clock(30000.0);
        bbd2.set_clock(50000.0);

        // Warmup
        let warmup = sine(440.0, 0.3, 0.3, SAMPLE_RATE);
        for &s in &warmup {
            let mid = bbd1.process(s);
            bbd2.process(mid);
        }

        // Measure single BBD
        let input = sine(440.0, 0.3, 1.0, SAMPLE_RATE);
        let single_output: Vec<f64> = {
            let mut bbd_single = BbdDelayLine::new(BbdModel::mn3005(), SAMPLE_RATE);
            bbd_single.set_clock(30000.0);
            // Warmup
            for &s in &warmup {
                bbd_single.process(s);
            }
            input.iter().map(|&s| bbd_single.process(s)).collect()
        };

        // Measure cascaded
        let cascade_output: Vec<f64> = input
            .iter()
            .map(|&s| {
                let mid = bbd1.process(s);
                bbd2.process(mid)
            })
            .collect();

        let input_rms = rms(&input);
        let single_rms = rms(&single_output);
        let cascade_rms = rms(&cascade_output);

        let single_thd = thd(&single_output, SAMPLE_RATE, 440.0);
        let cascade_thd = thd(&cascade_output, SAMPLE_RATE, 440.0);

        let thd_ratio = if single_thd > 0.001 {
            cascade_thd / single_thd
        } else {
            cascade_thd * 100.0
        };

        println!("\n=== CASCADED BBD TEST ===");
        println!("Input RMS: {:.4}", input_rms);
        println!("Single BBD: RMS={:.4}, THD={:.2}%", single_rms, single_thd * 100.0);
        println!(
            "Cascaded:   RMS={:.4}, THD={:.2}%",
            cascade_rms,
            cascade_thd * 100.0
        );
        println!("THD increase: {:.1}x", thd_ratio);

        // CHARACTERIZATION: This is where we expect to see the problem
        // Ideally THD should roughly double (2x), not explode (>10x)
        // Current behavior may show much higher ratios
        if thd_ratio > 5.0 {
            println!(
                "WARNING: THD increased {:.1}x through cascade - compander issue confirmed",
                thd_ratio
            );
        }

        // Record for regression - will tighten after fix
        assert!(
            cascade_rms > 0.0,
            "Cascade should produce output"
        );
    }

    #[test]
    fn cascaded_bbd_gain_stability() {
        // Check if cascaded BBDs have stable gain or if it drifts

        let mut bbd1 = BbdDelayLine::new(BbdModel::mn3005(), SAMPLE_RATE);
        let mut bbd2 = BbdDelayLine::new(BbdModel::mn3007(), SAMPLE_RATE);
        bbd1.set_clock(30000.0);
        bbd2.set_clock(50000.0);

        // Process multiple bursts and check if gain is consistent
        let mut gains = Vec::new();

        for burst_num in 0..5 {
            // Burst of signal
            let burst = sine(440.0, 0.3, 0.2, SAMPLE_RATE);
            let output: Vec<f64> = burst
                .iter()
                .map(|&s| {
                    let mid = bbd1.process(s);
                    bbd2.process(mid)
                })
                .collect();

            let burst_rms = rms(&burst);
            let output_rms = rms(&output);
            let gain = output_rms / burst_rms;
            gains.push(gain);

            println!("Burst {}: gain = {:.3}", burst_num, gain);

            // Gap of silence
            for _ in 0..4800 {
                let mid = bbd1.process(0.0);
                bbd2.process(mid);
            }
        }

        // Check gain consistency
        let max_gain = gains.iter().cloned().fold(0.0f64, f64::max);
        let min_gain = gains.iter().cloned().fold(f64::MAX, f64::min);
        let gain_variation = max_gain / min_gain.max(0.001);

        println!("Gain variation: {:.2}x (max={:.3}, min={:.3})", gain_variation, max_gain, min_gain);

        // CHARACTERIZATION: Gain should be relatively stable
        // Large variation indicates compander state issues
        if gain_variation > 3.0 {
            println!("WARNING: Gain varies {:.1}x between bursts - state issue", gain_variation);
        }
    }

    // =======================================================================
    // LEAKAGE LPF TESTS
    // =======================================================================

    #[test]
    fn leakage_coefficient_calculation() {
        // Test the leakage LPF coefficient calculation for different models

        println!("\n=== LEAKAGE LPF ANALYSIS ===");

        for (name, model) in [
            ("MN3207", BbdModel::mn3207()),
            ("MN3007", BbdModel::mn3007()),
            ("MN3005", BbdModel::mn3005()),
        ] {
            let total_loss = model.leakage_per_stage * model.num_stages as f64;
            let clamped_loss = total_loss.min(0.99);
            let retention = 1.0 - clamped_loss;

            println!(
                "{}: stages={}, leak/stage={:.4}, total_loss={:.4} (clamped={:.4}), retention={:.4}",
                name, model.num_stages, model.leakage_per_stage, total_loss, clamped_loss, retention
            );

            // Test at different clock frequencies
            for &clock in &[model.clock_min, (model.clock_min + model.clock_max) / 2.0, model.clock_max] {
                let coef = BbdDelayLine::compute_leakage_coef(&model, clock, SAMPLE_RATE);
                let delay_ms = model.delay_at_clock(clock) * 1000.0;

                // Estimate effective cutoff from coefficient
                // coef = exp(-2π × fc / sr), so fc = -ln(coef) × sr / (2π)
                let fc_estimate = if coef > 0.0 && coef < 1.0 {
                    -coef.ln() * SAMPLE_RATE / (2.0 * std::f64::consts::PI)
                } else {
                    0.0
                };

                println!(
                    "  clock={:.0}Hz, delay={:.1}ms: coef={:.6}, fc≈{:.0}Hz",
                    clock, delay_ms, coef, fc_estimate
                );
            }
        }

        // CHARACTERIZATION: MN3005 at min clock (longest delay) should still
        // have reasonable bandwidth. Current formula may make it too dark.
        let mn3005 = BbdModel::mn3005();
        let coef_long = BbdDelayLine::compute_leakage_coef(&mn3005, mn3005.clock_min, SAMPLE_RATE);

        // If coef is very high (>0.999), it means almost no filtering
        // If coef is very low (<0.9), it means aggressive filtering
        println!(
            "\nMN3005 at max delay: coef={:.6} (0.999=no filter, 0.9=aggressive)",
            coef_long
        );
    }

    #[test]
    fn leakage_effect_on_spectrum() {
        // Process a harmonically rich signal and measure HF loss

        let mut bbd_short = BbdDelayLine::new(BbdModel::mn3005(), SAMPLE_RATE);
        let mut bbd_long = BbdDelayLine::new(BbdModel::mn3005(), SAMPLE_RATE);

        // Short delay (high clock) vs long delay (low clock)
        bbd_short.set_clock(80000.0); // Short delay
        bbd_long.set_clock(15000.0); // Long delay

        // Create harmonically rich input
        let input: Vec<f64> = (0..48000)
            .map(|i| {
                let t = i as f64 / SAMPLE_RATE;
                let f = 220.0;
                0.3 * ((2.0 * std::f64::consts::PI * f * t).sin()
                    + 0.5 * (2.0 * std::f64::consts::PI * 2.0 * f * t).sin()
                    + 0.25 * (2.0 * std::f64::consts::PI * 3.0 * f * t).sin()
                    + 0.125 * (2.0 * std::f64::consts::PI * 4.0 * f * t).sin())
            })
            .collect();

        let short_out: Vec<f64> = input.iter().map(|&s| bbd_short.process(s)).collect();
        let long_out: Vec<f64> = input.iter().map(|&s| bbd_long.process(s)).collect();

        // Measure power at harmonics
        println!("\n=== LEAKAGE SPECTRAL EFFECT ===");
        println!("Harmonic\tInput\tShort\tLong\tLong/Short");

        for h in 1..=4 {
            let freq = 220.0 * h as f64;
            let input_p = goertzel_power(&input, SAMPLE_RATE, freq);
            let short_p = goertzel_power(&short_out, SAMPLE_RATE, freq);
            let long_p = goertzel_power(&long_out, SAMPLE_RATE, freq);

            let ratio = if short_p > 1e-10 {
                long_p / short_p
            } else {
                0.0
            };

            println!(
                "{}×220Hz\t{:.2e}\t{:.2e}\t{:.2e}\t{:.3}",
                h, input_p, short_p, long_p, ratio
            );
        }

        // CHARACTERIZATION: Long delay should be darker (lower HF)
        // Ratio < 1.0 for higher harmonics indicates leakage is working
    }

    // =======================================================================
    // FEEDBACK PATH TESTS
    // =======================================================================

    #[test]
    fn feedback_signal_integrity() {
        // Test that feedback path maintains signal integrity

        let mut bbd = BbdDelayLine::new(BbdModel::mn3207(), SAMPLE_RATE);
        bbd.set_clock(50000.0);
        bbd.set_feedback(0.5);

        // Process impulse and observe repeats
        let mut input = vec![0.5; 48]; // 1ms impulse
        input.extend(vec![0.0; 48000]); // 1 second silence for repeats

        let output: Vec<f64> = input.iter().map(|&s| bbd.process(s)).collect();

        // Find peaks (repeats)
        let mut peaks = Vec::new();
        for i in 100..output.len() - 1 {
            if output[i].abs() > output[i - 1].abs()
                && output[i].abs() > output[i + 1].abs()
                && output[i].abs() > 0.01
            {
                peaks.push((i, output[i]));
            }
        }

        println!("\n=== FEEDBACK REPEATS ===");
        println!("Found {} peaks", peaks.len());

        if peaks.len() >= 3 {
            // Measure decay rate between repeats
            for i in 0..peaks.len().min(5) {
                let (pos, amp) = peaks[i];
                println!("Peak {}: pos={} ({:.1}ms), amp={:.4}", i, pos, pos as f64 / SAMPLE_RATE * 1000.0, amp);
            }

            // Check if decay is smooth (each repeat should be ~feedback × previous)
            let expected_decay = 0.5; // feedback setting
            let mut decay_ratios = Vec::new();

            for i in 1..peaks.len().min(5) {
                let ratio = peaks[i].1.abs() / peaks[i - 1].1.abs().max(0.001);
                decay_ratios.push(ratio);
                println!("Decay {}->{}: {:.3} (expected ~{:.2})", i - 1, i, ratio, expected_decay);
            }

            // CHARACTERIZATION: Decay should be close to feedback setting
            // If it varies wildly, the feedback path has issues
            if let Some(&max_ratio) = decay_ratios.iter().max_by(|a, b| a.partial_cmp(b).unwrap()) {
                if let Some(&min_ratio) = decay_ratios.iter().min_by(|a, b| a.partial_cmp(b).unwrap()) {
                    let variation = max_ratio / min_ratio.max(0.001);
                    println!("Decay variation: {:.2}x", variation);

                    if variation > 2.0 {
                        println!("WARNING: Feedback decay is inconsistent - compander affecting feedback");
                    }
                }
            }
        }
    }

    #[test]
    fn feedback_thd_accumulation() {
        // Measure how THD accumulates through feedback repeats

        let mut bbd = BbdDelayLine::new(BbdModel::mn3007(), SAMPLE_RATE);
        bbd.set_clock(80000.0); // Short delay for faster repeats
        bbd.set_feedback(0.7); // High feedback

        // Short burst then silence
        let mut input = sine(440.0, 0.3, 0.05, SAMPLE_RATE); // 50ms burst
        input.extend(vec![0.0; (SAMPLE_RATE * 1.0) as usize]); // 1s for repeats

        let output: Vec<f64> = input.iter().map(|&s| bbd.process(s)).collect();

        // Analyze THD in different time windows
        let window_size = 4800; // 100ms windows
        let num_windows = output.len() / window_size;

        println!("\n=== FEEDBACK THD ACCUMULATION ===");
        println!("Window\tRMS\tTHD%");

        let mut thd_values = Vec::new();
        for w in 0..num_windows.min(10) {
            let start = w * window_size;
            let end = start + window_size;
            let window = &output[start..end];

            let window_rms = rms(window);
            let window_thd = thd(window, SAMPLE_RATE, 440.0);

            if window_rms > 0.01 {
                println!("{}\t{:.4}\t{:.2}%", w, window_rms, window_thd * 100.0);
                thd_values.push(window_thd);
            }
        }

        // CHARACTERIZATION: THD should not explode through repeats
        if thd_values.len() >= 2 {
            let first_thd = thd_values[0];
            let last_thd = thd_values[thd_values.len() - 1];

            if last_thd > first_thd * 3.0 && first_thd > 0.01 {
                println!(
                    "WARNING: THD grew {:.1}x through feedback - distortion accumulating",
                    last_thd / first_thd
                );
            }
        }
    }

    // =======================================================================
    // MODEL COMPARISON TESTS
    // =======================================================================

    #[test]
    fn model_characteristics_comparison() {
        // Compare the three BBD models to ensure they have distinct characters

        println!("\n=== BBD MODEL COMPARISON ===");
        println!("Model\tDelay Range\tNoise\tLeakage\tClock Feed");

        for (name, model) in [
            ("MN3207", BbdModel::mn3207()),
            ("MN3007", BbdModel::mn3007()),
            ("MN3005", BbdModel::mn3005()),
        ] {
            println!(
                "{}\t{:.1}-{:.1}ms\t{:.4}\t{:.4}\t{:.4}",
                name,
                model.min_delay() * 1000.0,
                model.max_delay() * 1000.0,
                model.noise_floor,
                model.leakage_per_stage,
                model.clock_feedthrough
            );
        }

        // Process same signal through each
        let input = sine(440.0, 0.3, 0.5, SAMPLE_RATE);

        println!("\nOutput characteristics (440Hz sine at 0.3 amplitude):");
        println!("Model\tRMS\tTHD%\tPeak");

        for (name, model) in [
            ("MN3207", BbdModel::mn3207()),
            ("MN3007", BbdModel::mn3007()),
            ("MN3005", BbdModel::mn3005()),
        ] {
            let mut bbd = BbdDelayLine::new(model, SAMPLE_RATE);
            // Use middle of clock range
            bbd.set_clock((model.clock_min + model.clock_max) / 2.0);

            // Warmup
            for &s in &input {
                bbd.process(s);
            }

            // Measure
            let output: Vec<f64> = input.iter().map(|&s| bbd.process(s)).collect();
            let out_rms = rms(&output);
            let out_thd = thd(&output, SAMPLE_RATE, 440.0);
            let out_peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));

            println!("{}\t{:.4}\t{:.2}%\t{:.4}", name, out_rms, out_thd * 100.0, out_peak);
        }
    }
}
