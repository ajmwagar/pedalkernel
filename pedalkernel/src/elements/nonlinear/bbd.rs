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
///
/// The delay buffer uses linear interpolation for fractional-sample
/// delay, modeling the smooth time modulation of chorus/flanger effects.
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
    /// Compander envelope state: tracks the input RMS level for the
    /// compressor side.  The expander side sees a slightly delayed/smoothed
    /// version, causing tracking error.
    compander_env_in: f64,
    compander_env_out: f64,
    /// Compander attack/release coefficients.
    compander_attack: f64,
    compander_release: f64,
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
        // Attack ~1ms, release ~10ms — the mismatch causes "breathing" artifacts.
        let compander_attack = (-1.0 / (0.001 * sample_rate)).exp();
        let compander_release = (-1.0 / (0.010 * sample_rate)).exp();

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
            compander_env_out: 0.0,
            compander_attack,
            compander_release,
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

        let sample_rate = self.inner.sample_rate();

        // Update anti-alias filter
        let lpf_cutoff = self.model.bandwidth_ratio * self.clock_freq;
        self.lpf_coef = (-2.0 * std::f64::consts::PI * lpf_cutoff / sample_rate).exp();

        // Update leakage LPF (more leakage at lower clock = longer delays)
        self.leakage_lpf_coef =
            Self::compute_leakage_coef(&self.model, self.clock_freq, sample_rate);

        // Update clock feedthrough frequency
        self.clock_phase_inc =
            2.0 * std::f64::consts::PI * self.clock_freq / sample_rate;
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
    /// Signal chain (models real BBD circuit):
    /// 1. Compander input stage: compress dynamic range (NE571 compressor)
    /// 2. Soft clip to BBD voltage swing limit
    /// 3. Write to delay buffer (with feedback)
    /// 4. Read from buffer with interpolation
    /// 5. Charge leakage LPF (darker at longer delays)
    /// 6. Anti-alias LPF (BBD Nyquist bandwidth limit)
    /// 7. Clock feedthrough injection
    /// 8. Noise injection
    /// 9. Compander output stage: expand dynamic range (NE571 expander)
    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
        // ── Compander input (compressor) ──────────────────────────────
        // Track input envelope with fast attack, slow release.
        let abs_in = input.abs();
        let coef_in = if abs_in > self.compander_env_in {
            self.compander_attack
        } else {
            self.compander_release
        };
        self.compander_env_in =
            coef_in * self.compander_env_in + (1.0 - coef_in) * abs_in;

        // Compress: reduce dynamic range by 2:1 (typical NE571 ratio).
        // gain = 1/sqrt(envelope) — louder signals get compressed more.
        let comp_gain = if self.compander_env_in > 0.001 {
            1.0 / self.compander_env_in.sqrt()
        } else {
            1.0
        };
        let compressed = input * comp_gain.min(10.0); // Limit expansion of quiet signals

        // ── Mix with feedback and soft-clip ───────────────────────────
        let write_sample = compressed + self.feedback * self.feedback_sample;
        let clipped = bbd_soft_clip(write_sample);

        // Write to inner delay line's buffer (direct — feedback is handled above)
        self.inner.write_direct(clipped);

        // ── Read with interpolation ───────────────────────────────────
        let delay_samples = self.inner.effective_delay_samples();
        let out_raw = self.inner.buffer_read(delay_samples, 0);

        // Advance write position (and process medium if configured)
        self.inner.advance_write();

        // ── Charge leakage LPF ───────────────────────────────────────
        // Models accumulated charge loss across BBD stages.  More stages
        // and lower clock frequency → more leakage → darker output.
        // This is what gives long BBD delays their characteristic warmth.
        self.leakage_lpf_state = self.leakage_lpf_coef * self.leakage_lpf_state
            + (1.0 - self.leakage_lpf_coef) * out_raw;

        // ── Anti-alias LPF (BBD Nyquist bandwidth limit) ─────────────
        self.lpf_state = self.lpf_coef * self.lpf_state
            + (1.0 - self.lpf_coef) * self.leakage_lpf_state;

        // ── Clock feedthrough ─────────────────────────────────────────
        // The BBD switching clock couples into the signal through parasitic
        // capacitance.  Audible as a high-pitched whine, especially at
        // lower clock frequencies where it falls into the audio band.
        self.clock_phase += self.clock_phase_inc;
        if self.clock_phase > 2.0 * std::f64::consts::PI {
            self.clock_phase -= 2.0 * std::f64::consts::PI;
        }
        let clock_bleed = self.model.clock_feedthrough * self.clock_phase.sin();

        // ── Noise injection ───────────────────────────────────────────
        let noise = self.next_noise() * self.model.noise_floor;

        let bbd_output = self.lpf_state + clock_bleed + noise;

        // ── Compander output (expander) ───────────────────────────────
        // Track output envelope — deliberately uses slightly different
        // time constants to model NE571 tracking error.
        let abs_out = bbd_output.abs();
        // Expander envelope is intentionally sluggish compared to compressor,
        // scaled by compander_error to control the magnitude of the artifact.
        let error_scale = 1.0 + self.model.compander_error;
        let coef_out = if abs_out > self.compander_env_out {
            // Expander attack is slower than compressor attack
            self.compander_attack * error_scale
        } else {
            // Expander release is faster than compressor release
            self.compander_release / error_scale
        };
        self.compander_env_out =
            coef_out.min(0.9999) * self.compander_env_out + (1.0 - coef_out.min(0.9999)) * abs_out;

        // Expand: restore dynamic range.
        let exp_gain = if self.compander_env_out > 0.001 {
            self.compander_env_out.sqrt()
        } else {
            1.0
        };
        let output = bbd_output * exp_gain.min(10.0);

        // Store for feedback (post-compander, as in real circuits)
        self.feedback_sample = output;

        output
    }

    /// Update sample rate and resize buffer.
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.inner.set_sample_rate(sample_rate);
        self.inner.set_delay_seconds(self.model.delay_at_clock(self.clock_freq));

        let lpf_cutoff = self.model.bandwidth_ratio * self.clock_freq;
        self.lpf_coef = (-2.0 * std::f64::consts::PI * lpf_cutoff / sample_rate).exp();
        self.leakage_lpf_coef =
            Self::compute_leakage_coef(&self.model, self.clock_freq, sample_rate);
        self.clock_phase_inc =
            2.0 * std::f64::consts::PI * self.clock_freq / sample_rate;
        self.compander_attack = (-1.0 / (0.001 * sample_rate)).exp();
        self.compander_release = (-1.0 / (0.010 * sample_rate)).exp();
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
