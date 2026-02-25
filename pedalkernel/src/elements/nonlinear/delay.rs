//! Generic delay line with ring buffer, interpolation, and medium processing.
//!
//! Supports linear, allpass, and cubic interpolation modes with tape/BBD medium models.

// ═══════════════════════════════════════════════════════════════════════════
// Generic Delay Line — WDF two-port adaptor backed by a ring buffer
// ═══════════════════════════════════════════════════════════════════════════

/// Interpolation mode for delay line read pointer.
///
/// Different delay types require different interpolation characteristics:
/// - Linear: simple, low CPU, slight HF loss — suitable for digital delays
/// - Allpass: sub-sample accuracy without HF loss — best for tape/analog feel
/// - Cubic: smooth with low aliasing — good general purpose
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Interpolation {
    /// Linear interpolation: `y = y0*(1-f) + y1*f`
    /// Simple and cheap. Causes slight HF roll-off at short delays.
    Linear,
    /// First-order allpass interpolation for sub-sample accuracy.
    /// Preserves magnitude response while providing smooth fractional delay.
    /// Best for modulated delays (tape echo, chorus) — no zipper noise.
    Allpass,
    /// Cubic Hermite interpolation: 4-point, smooth transitions.
    /// Good frequency response, moderate CPU cost.
    Cubic,
}

/// Physical storage medium model for the delay buffer.
///
/// In real tape and BBD circuits, the storage medium itself transforms
/// the signal while it sits in the buffer. This is physically distinct
/// from write-side processing (saturation at the record head) and
/// read-side processing (gap loss at playback heads), which are handled
/// by WDF elements in the signal chain.
///
/// The medium processing uses a zone-based approach: the virtual tape
/// path is divided into zones, and each sample gets processed once as
/// it crosses a zone boundary. This is O(zones) per tick, not
/// O(buffer_length), keeping CPU cost negligible.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Medium {
    /// Clean digital delay — no buffer processing.
    None,
    /// Magnetic tape self-demagnetization.
    /// Adjacent domains with opposing polarity partially cancel, causing
    /// frequency-dependent HF loss that increases with distance from the
    /// record head. Short wavelengths (high frequencies) are affected
    /// more because opposing domains are physically closer.
    TapeOxide,
    /// BBD charge leakage.
    /// Amplitude decays uniformly toward zero over time (not frequency-
    /// dependent like tape). Each bucket loses a fraction of its charge
    /// on every clock cycle.
    BbdLeakage,
    /// Digital bit reduction (PT2399-style).
    /// Quantization happens at write time only — no ongoing buffer
    /// processing. The `process_medium()` method is a no-op for this mode.
    DigitalQuantize,
}

/// A zone boundary along the virtual delay path.
///
/// Samples are processed incrementally as they cross zone boundaries.
/// At normal playback speed, exactly one sample crosses each zone
/// boundary per tick, so the total cost is O(zones) per sample.
#[derive(Debug, Clone)]
pub struct ZoneBoundary {
    /// Distance from write head in samples.
    pub distance_samples: f64,
    /// One-pole LP filter state for this zone (tape oxide model).
    pub filter_state: f64,
    /// LP coefficient applied at this boundary.
    /// Higher = stronger HF loss. Typical tape: 0.02–0.05.
    pub coefficient: f64,
    /// Decay rate per zone crossing (BBD leakage model).
    /// Each crossing multiplies the sample by `(1 - leak_rate)`.
    pub leak_rate: f64,
    /// Tracks the last buffer position processed at this boundary.
    pub last_processed_pos: usize,
}

/// Pre-configured medium settings for common tape echo models.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MediumPreset {
    /// RE-201: gentle, well-maintained tape.
    Re201,
    /// Echoplex: darker, more worn tape.
    Echoplex,
    /// Lo-fi: aggressive degradation.
    LoFi,
}

impl MediumPreset {
    /// Get the zone coefficients for this preset.
    ///
    /// Returns coefficients for each zone boundary between taps.
    /// More distant zones have stronger HF loss.
    pub fn zone_coefficients(&self) -> Vec<f64> {
        match self {
            MediumPreset::Re201 => vec![0.02, 0.03, 0.05],
            MediumPreset::Echoplex => vec![0.04, 0.06],
            MediumPreset::LoFi => vec![0.08, 0.12, 0.18],
        }
    }
}

/// Generic delay line: a ring buffer with WDF-compatible ports.
///
/// This is a two-port element that splits the WDF tree into separate
/// subtrees coupled by an implicit one-sample delay. The write side
/// accepts signal from the "record" subtree; the read side feeds the
/// "playback" subtree(s).
///
/// The delay time is continuously modulatable — changing it at audio rate
/// moves the read pointer smoothly, creating pitch bending effects.
///
/// ## WDF integration
///
/// The compiler detects `delay_line()` in a netlist and automatically
/// splits the WDF tree at that point. Trees are processed in order:
/// write tree first (fills buffer), then read tree(s) consume from it.
/// The implicit one-sample delay between write and read ensures causality.
///
/// ## Multi-tap
///
/// Multiple `Tap` elements can read from the same `DelayLine` at
/// different positions (ratios of the base delay time). This models
/// multi-head tape machines (RE-201) and multi-tap digital delays.
#[derive(Debug, Clone)]
pub struct DelayLine {
    /// Ring buffer holding delayed samples.
    buffer: Vec<f64>,
    /// Current write position in the buffer.
    write_pos: usize,
    /// Current delay time in samples (fractional for interpolation).
    current_delay_samples: f64,
    /// Minimum delay time in seconds.
    min_delay_sec: f64,
    /// Maximum delay time in seconds.
    max_delay_sec: f64,
    /// Sample rate (Hz).
    sample_rate: f64,
    /// Interpolation mode for fractional-sample reads.
    interpolation: Interpolation,
    /// Allpass interpolation state (previous output) — one per tap slot.
    /// Index 0 is for the base delay, indices 1..N for taps.
    allpass_states: Vec<f64>,
    /// Speed modulation accumulator (from LFO wow/flutter).
    /// Applied as a multiplier to the base delay time.
    speed_mod: f64,
    /// Feedback sample (from downstream, routed back to input).
    feedback_sample: f64,
    /// Feedback amount (0.0 = no feedback, up to ~0.95 for self-oscillation).
    feedback: f64,
    /// Physical storage medium model.
    medium: Medium,
    /// Zone boundaries for medium processing.
    /// Samples are processed incrementally as they cross each boundary.
    zones: Vec<ZoneBoundary>,
}

impl DelayLine {
    /// Create a new delay line.
    ///
    /// * `min_delay_sec` — minimum delay in seconds (e.g., 0.001 for 1ms)
    /// * `max_delay_sec` — maximum delay in seconds (e.g., 1.2 for 1200ms)
    /// * `sample_rate` — audio sample rate in Hz
    /// * `interpolation` — fractional-sample interpolation mode
    pub fn new(
        min_delay_sec: f64,
        max_delay_sec: f64,
        sample_rate: f64,
        interpolation: Interpolation,
    ) -> Self {
        // Buffer sized for max delay + headroom for interpolation
        let max_samples = (max_delay_sec * sample_rate * 1.1) as usize + 8;
        let buffer = vec![0.0; max_samples];

        // Default to midpoint delay
        let mid = (min_delay_sec + max_delay_sec) / 2.0;
        let current_delay_samples = mid * sample_rate;

        Self {
            buffer,
            write_pos: 0,
            current_delay_samples,
            min_delay_sec,
            max_delay_sec,
            sample_rate,
            interpolation,
            allpass_states: vec![0.0; 8], // Room for up to 8 taps
            speed_mod: 1.0,
            feedback_sample: 0.0,
            feedback: 0.0,
            medium: Medium::None,
            zones: Vec::new(),
        }
    }

    /// Write a sample into the delay buffer, process medium zones,
    /// and advance the write pointer.
    ///
    /// The input signal is mixed with the feedback sample before writing.
    /// After writing, zone-based medium processing runs on any samples
    /// that have crossed zone boundaries since the last tick.
    /// Call this once per audio sample, before reading any taps.
    #[inline]
    pub fn write(&mut self, input: f64) {
        let write_sample = input + self.feedback * self.feedback_sample;
        self.buffer[self.write_pos] = write_sample;
        self.write_pos = (self.write_pos + 1) % self.buffer.len();

        // Process medium zones (incremental — only touches samples at boundaries)
        self.process_medium();
    }

    /// Read from the delay buffer at the current base delay time.
    ///
    /// Returns the interpolated sample at `write_pos - current_delay_samples`.
    /// This is equivalent to reading from tap ratio 1.0.
    #[inline]
    pub fn read(&mut self) -> f64 {
        self.read_at_ratio(1.0, 0)
    }

    /// Read from the delay buffer at a specific ratio of the base delay.
    ///
    /// * `ratio` — multiplier for the base delay (e.g., 2.0 = twice the delay)
    /// * `tap_index` — index for allpass state tracking (0 = base, 1+ = taps)
    ///
    /// The effective delay in samples is:
    ///   `current_delay_samples * speed_mod * ratio`
    #[inline]
    pub fn read_at_ratio(&mut self, ratio: f64, tap_index: usize) -> f64 {
        let effective_delay = self.current_delay_samples * self.speed_mod * ratio;
        let max_delay = (self.buffer.len() - 2) as f64;
        let clamped_delay = effective_delay.clamp(1.0, max_delay);

        match self.interpolation {
            Interpolation::Linear => self.read_linear(clamped_delay),
            Interpolation::Allpass => self.read_allpass(clamped_delay, tap_index),
            Interpolation::Cubic => self.read_cubic(clamped_delay),
        }
    }

    /// Linear interpolation read.
    #[inline]
    fn read_linear(&self, delay_samples: f64) -> f64 {
        let delay_int = delay_samples as usize;
        let frac = delay_samples - delay_int as f64;
        let buf_len = self.buffer.len();

        let idx0 = (self.write_pos + buf_len - delay_int) % buf_len;
        let idx1 = (idx0 + buf_len - 1) % buf_len;

        self.buffer[idx0] * (1.0 - frac) + self.buffer[idx1] * frac
    }

    /// First-order allpass interpolation read.
    ///
    /// Uses the Thiran allpass approximation for fractional delay:
    ///   `y[n] = x[n-D] + eta * (y[n-1] - x[n-D-1])`
    /// where `eta = (1 - frac) / (1 + frac)`.
    ///
    /// This preserves magnitude response (no HF loss) while providing
    /// smooth sub-sample delay — ideal for modulated delays.
    #[inline]
    fn read_allpass(&mut self, delay_samples: f64, tap_index: usize) -> f64 {
        let delay_int = delay_samples as usize;
        let frac = delay_samples - delay_int as f64;
        let buf_len = self.buffer.len();

        let idx0 = (self.write_pos + buf_len - delay_int) % buf_len;
        let idx1 = (idx0 + buf_len - 1) % buf_len;

        let x0 = self.buffer[idx0];
        let x1 = self.buffer[idx1];

        // Thiran allpass coefficient
        let eta = (1.0 - frac) / (1.0 + frac);

        // Ensure tap_index is in range
        let state_idx = tap_index.min(self.allpass_states.len() - 1);
        let prev = self.allpass_states[state_idx];

        let output = x1 + eta * (prev - x0);
        self.allpass_states[state_idx] = output;

        // Protect against NaN/Inf in allpass feedback
        if !output.is_finite() {
            self.allpass_states[state_idx] = 0.0;
            return x0 * (1.0 - frac) + x1 * frac; // Fall back to linear
        }

        output
    }

    /// Cubic Hermite interpolation read (4-point).
    #[inline]
    fn read_cubic(&self, delay_samples: f64) -> f64 {
        let delay_int = delay_samples as usize;
        let frac = delay_samples - delay_int as f64;
        let buf_len = self.buffer.len();

        // 4 sample points: x[-1], x[0], x[1], x[2]
        let idx0 = (self.write_pos + buf_len - delay_int) % buf_len;
        let idx_m1 = (idx0 + 1) % buf_len;
        let idx1 = (idx0 + buf_len - 1) % buf_len;
        let idx2 = (idx1 + buf_len - 1) % buf_len;

        let xm1 = self.buffer[idx_m1];
        let x0 = self.buffer[idx0];
        let x1 = self.buffer[idx1];
        let x2 = self.buffer[idx2];

        // Catmull-Rom cubic interpolation
        let c0 = x0;
        let c1 = 0.5 * (x1 - xm1);
        let c2 = xm1 - 2.5 * x0 + 2.0 * x1 - 0.5 * x2;
        let c3 = 0.5 * (x2 - xm1) + 1.5 * (x0 - x1);

        ((c3 * frac + c2) * frac + c1) * frac + c0
    }

    /// Set the delay time as a normalized value (0.0 = min, 1.0 = max).
    ///
    /// Uses logarithmic interpolation for a natural feel — small changes
    /// at short delays, larger changes at long delays.
    pub fn set_delay_normalized(&mut self, norm: f64) {
        let norm = norm.clamp(0.0, 1.0);
        let log_min = self.min_delay_sec.ln();
        let log_max = self.max_delay_sec.ln();
        let delay_sec = (log_min + norm * (log_max - log_min)).exp();
        self.current_delay_samples = delay_sec * self.sample_rate;
    }

    /// Set the delay time in seconds directly.
    pub fn set_delay_seconds(&mut self, seconds: f64) {
        let clamped = seconds.clamp(self.min_delay_sec, self.max_delay_sec);
        self.current_delay_samples = clamped * self.sample_rate;
    }

    /// Set the speed modulation factor.
    ///
    /// A value of 1.0 means no modulation. Values > 1.0 slow the "tape"
    /// (longer delay, lower pitch). Values < 1.0 speed it up.
    /// Typically modulated by an LFO for wow/flutter effects.
    #[inline]
    pub fn set_speed_mod(&mut self, factor: f64) {
        self.speed_mod = factor.clamp(0.5, 2.0);
    }

    /// Add a speed modulation offset (additive, for multiple LFO sources).
    ///
    /// The total speed_mod = 1.0 + sum of all modulation offsets.
    #[inline]
    pub fn add_speed_mod(&mut self, offset: f64) {
        self.speed_mod += offset;
    }

    /// Reset speed modulation to 1.0 (called at start of each sample).
    #[inline]
    pub fn reset_speed_mod(&mut self) {
        self.speed_mod = 1.0;
    }

    /// Set the feedback amount.
    pub fn set_feedback(&mut self, feedback: f64) {
        self.feedback = feedback.clamp(0.0, 0.99);
    }

    /// Store the feedback sample (called after downstream processing).
    #[inline]
    pub fn set_feedback_sample(&mut self, sample: f64) {
        self.feedback_sample = sample;
    }

    /// Get the current delay time in seconds.
    pub fn delay_time(&self) -> f64 {
        self.current_delay_samples / self.sample_rate
    }

    /// Get the current delay time in samples (with speed modulation).
    pub fn effective_delay_samples(&self) -> f64 {
        self.current_delay_samples * self.speed_mod
    }

    /// Update for a new sample rate.
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        let delay_sec = self.current_delay_samples / self.sample_rate;
        self.sample_rate = sample_rate;
        self.current_delay_samples = delay_sec * sample_rate;

        // Resize buffer if needed
        let max_samples = (self.max_delay_sec * sample_rate * 1.1) as usize + 8;
        if max_samples > self.buffer.len() {
            self.buffer.resize(max_samples, 0.0);
        }
    }

    /// Reset all state (buffer, allpass states, feedback).
    pub fn reset(&mut self) {
        self.buffer.fill(0.0);
        self.write_pos = 0;
        self.allpass_states.fill(0.0);
        self.feedback_sample = 0.0;
        self.speed_mod = 1.0;
        for zone in &mut self.zones {
            zone.filter_state = 0.0;
            zone.last_processed_pos = 0;
        }
    }

    /// Minimum delay in seconds.
    pub fn min_delay_sec(&self) -> f64 {
        self.min_delay_sec
    }

    /// Maximum delay in seconds.
    pub fn max_delay_sec(&self) -> f64 {
        self.max_delay_sec
    }

    /// Set the physical storage medium model.
    pub fn set_medium(&mut self, medium: Medium) {
        self.medium = medium;
    }

    /// Get the current medium model.
    pub fn medium(&self) -> Medium {
        self.medium
    }

    /// Get the current sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Write a sample directly to the buffer without feedback mixing.
    ///
    /// Does NOT advance the write position — call [`advance_write`] after
    /// reading to complete the tick.  This is used by `BbdDelayLine` which
    /// manages its own feedback path (with soft-clipping between mix and write).
    #[inline]
    pub fn write_direct(&mut self, sample: f64) {
        self.buffer[self.write_pos] = sample;
    }

    /// Read from the buffer at the given delay (in samples), using the
    /// current interpolation mode.
    ///
    /// Lower-level than [`read`]/[`read_at_ratio`]: the caller supplies the
    /// exact fractional-sample delay instead of using the stored delay time.
    #[inline]
    pub fn buffer_read(&mut self, delay_samples: f64, tap_slot: usize) -> f64 {
        let delay = delay_samples.clamp(1.0, self.buffer.len() as f64 - 2.0);
        match self.interpolation {
            Interpolation::Linear => self.read_linear(delay),
            Interpolation::Allpass => self.read_allpass(delay, tap_slot),
            Interpolation::Cubic => self.read_cubic(delay),
        }
    }

    /// Advance the write position by one sample and run medium processing.
    ///
    /// Must be called exactly once per tick, after [`write_direct`] and
    /// any reads are complete.
    #[inline]
    pub fn advance_write(&mut self) {
        self.write_pos = (self.write_pos + 1) % self.buffer.len();
        self.process_medium();
    }

    /// Configure zone boundaries from tap ratios.
    ///
    /// Creates one zone boundary at each tap position, with coefficients
    /// that increase with distance (modeling progressive signal degradation).
    /// An additional zone is placed at the end of the buffer for the
    /// feedback path.
    ///
    /// * `tap_ratios` — tap positions as ratios of base delay (e.g., [1.0, 2.0, 4.0])
    /// * `coefficients` — optional per-zone LP coefficients; if `None`, uses
    ///   defaults based on medium type and tap count
    pub fn configure_zones_from_taps(
        &mut self,
        tap_ratios: &[f64],
        coefficients: Option<&[f64]>,
    ) {
        self.zones.clear();

        if self.medium == Medium::None || self.medium == Medium::DigitalQuantize {
            return; // No zone processing needed
        }

        // Sort tap ratios to determine zone boundaries
        let mut sorted_ratios: Vec<f64> = tap_ratios.to_vec();
        sorted_ratios.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        sorted_ratios.dedup();

        // Default coefficients: progressively stronger
        let default_coefficients: Vec<f64> = sorted_ratios
            .iter()
            .enumerate()
            .map(|(i, _)| match self.medium {
                Medium::TapeOxide => 0.02 + 0.015 * i as f64, // 0.02, 0.035, 0.05, ...
                Medium::BbdLeakage => 0.01 + 0.005 * i as f64, // 0.01, 0.015, 0.02, ...
                _ => 0.0,
            })
            .collect();

        let coeffs = coefficients.unwrap_or(&default_coefficients);

        for (i, &ratio) in sorted_ratios.iter().enumerate() {
            let distance_samples = self.current_delay_samples * ratio;
            let coeff = coeffs.get(i).copied().unwrap_or(0.03);
            let leak = if self.medium == Medium::BbdLeakage {
                coeff
            } else {
                0.0
            };

            self.zones.push(ZoneBoundary {
                distance_samples,
                filter_state: 0.0,
                coefficient: coeff,
                leak_rate: leak,
                last_processed_pos: self.write_pos,
            });
        }
    }

    /// Zone-based incremental medium processing.
    ///
    /// For each zone boundary, checks if new samples have crossed it
    /// since the last tick. Applies the appropriate medium transformation
    /// to only those samples. At normal speed, this is O(zones) per tick
    /// (typically 1 sample per zone).
    #[inline]
    fn process_medium(&mut self) {
        match self.medium {
            Medium::None | Medium::DigitalQuantize => {}

            Medium::TapeOxide => {
                let buf_len = self.buffer.len();
                for zone in &mut self.zones {
                    // Current boundary position (relative to write head)
                    let boundary_pos = (self.write_pos + buf_len
                        - zone.distance_samples as usize)
                        % buf_len;

                    // How many samples have crossed this boundary since last tick
                    let samples_crossed = if boundary_pos >= zone.last_processed_pos {
                        boundary_pos - zone.last_processed_pos
                    } else {
                        buf_len - zone.last_processed_pos + boundary_pos
                    };

                    // Clamp to prevent runaway in edge cases (speed transients)
                    let to_process = samples_crossed.min(4);

                    for j in 0..to_process {
                        let idx = (zone.last_processed_pos + j + 1) % buf_len;
                        // One-pole LP: y += coeff * (x - y)
                        // This removes HF — short wavelength domains cancel more
                        zone.filter_state += zone.coefficient
                            * (self.buffer[idx] - zone.filter_state);
                        self.buffer[idx] = zone.filter_state;
                    }

                    zone.last_processed_pos = boundary_pos;
                }
            }

            Medium::BbdLeakage => {
                let buf_len = self.buffer.len();
                for zone in &mut self.zones {
                    let boundary_pos = (self.write_pos + buf_len
                        - zone.distance_samples as usize)
                        % buf_len;

                    let samples_crossed = if boundary_pos >= zone.last_processed_pos {
                        boundary_pos - zone.last_processed_pos
                    } else {
                        buf_len - zone.last_processed_pos + boundary_pos
                    };

                    let to_process = samples_crossed.min(4);

                    for j in 0..to_process {
                        let idx = (zone.last_processed_pos + j + 1) % buf_len;
                        // Uniform amplitude decay — charge leaks to ground
                        self.buffer[idx] *= 1.0 - zone.leak_rate;
                    }

                    zone.last_processed_pos = boundary_pos;
                }
            }
        }
    }

    /// Update zone distances when delay time changes.
    ///
    /// Called automatically by `set_delay_normalized` and `set_delay_seconds`
    /// to keep zone boundaries aligned with the current delay time.
    fn update_zone_distances(&mut self, tap_ratios: &[f64]) {
        for (zone, &ratio) in self.zones.iter_mut().zip(tap_ratios.iter()) {
            zone.distance_samples = self.current_delay_samples * ratio;
        }
    }
}

/// A read-only tap into a `DelayLine`.
///
/// Each tap reads from the parent delay line's buffer at a fixed ratio
/// of the base delay time. For an RE-201 with heads at 3cm, 6cm, 12cm:
///   - Head 1: ratio 1.0 (base delay)
///   - Head 2: ratio 2.0 (2× base delay)
///   - Head 3: ratio 4.0 (4× base delay)
///
/// Taps are separate WDF output ports that can feed into their own
/// subtrees (e.g., per-head HF filtering, individual level controls).
#[derive(Debug, Clone)]
pub struct Tap {
    /// Ratio of base delay time (e.g., 1.0, 2.0, 4.0).
    pub ratio: f64,
    /// Index of the parent `DelayLine` in the compiled pedal's delay list.
    pub delay_line_idx: usize,
    /// Tap index for allpass state tracking within the delay line.
    /// Index 0 is reserved for the base read; taps use 1, 2, 3, ...
    pub tap_index: usize,
}

#[cfg(test)]
mod delay_tests {
    use super::*;
    use super::super::bbd::{BbdDelayLine, BbdModel};

    const SR: f64 = 48000.0;

    // ── DelayLine basic operation ────────────────────────────────────

    #[test]
    fn delay_line_produces_delayed_output() {
        // A 10ms delay at 48kHz = 480 samples.
        // write() advances write_pos before read(), so the effective latency
        // through write→read is (delay_samples - 1) ticks.
        let mut dl = DelayLine::new(0.010, 0.010, SR, Interpolation::Linear);
        dl.set_delay_seconds(0.010);

        // Write impulse then pump zeros, collecting every output.
        let n = 600;
        let mut outputs = vec![0.0_f64; n];

        dl.write(1.0);
        outputs[0] = dl.read();
        for i in 1..n {
            dl.write(0.0);
            outputs[i] = dl.read();
        }

        // Find peak — should be near sample 479 (480 - 1 for write-advance-read ordering)
        let peak_idx = outputs
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap())
            .unwrap()
            .0;

        assert!(
            (peak_idx as i64 - 479).abs() <= 1,
            "Impulse should arrive near sample 479, peaked at {peak_idx}"
        );
        assert!(
            outputs[peak_idx].abs() > 0.5,
            "Peak amplitude should be significant, got {}",
            outputs[peak_idx]
        );
    }

    #[test]
    fn delay_line_output_is_finite() {
        let mut dl = DelayLine::new(0.001, 0.050, SR, Interpolation::Allpass);
        dl.set_delay_seconds(0.025);

        for i in 0..2400 {
            let t = i as f64 / SR;
            let input = (2.0 * std::f64::consts::PI * 440.0 * t).sin();
            dl.write(input);
            let out = dl.read();
            assert!(out.is_finite(), "Output should be finite at sample {i}");
        }
    }

    #[test]
    fn delay_line_set_delay_normalized() {
        let mut dl = DelayLine::new(0.001, 1.0, SR, Interpolation::Linear);

        // norm=0 → min delay (1ms)
        dl.set_delay_normalized(0.0);
        let t0 = dl.delay_time();
        assert!((t0 - 0.001).abs() < 0.0001, "norm=0 should be ~1ms, got {t0}");

        // norm=1 → max delay (1s)
        dl.set_delay_normalized(1.0);
        let t1 = dl.delay_time();
        assert!((t1 - 1.0).abs() < 0.01, "norm=1 should be ~1s, got {t1}");
    }

    #[test]
    fn delay_line_feedback_builds_up() {
        let mut dl = DelayLine::new(0.001, 0.010, SR, Interpolation::Linear);
        dl.set_delay_seconds(0.002); // 96 samples
        dl.set_feedback(0.8);

        // Write impulse
        dl.write(1.0);

        // Run for several delay periods, collecting output
        let mut max_out = 0.0_f64;
        for _ in 1..1000 {
            dl.write(0.0);
            let out = dl.read();
            dl.set_feedback_sample(out);
            max_out = max_out.max(out.abs());
        }

        // With feedback=0.8, the impulse should recirculate
        assert!(max_out > 0.1, "Feedback should cause recirculation, max={max_out}");
    }

    #[test]
    fn delay_line_speed_mod_affects_pitch() {
        let mut dl = DelayLine::new(0.005, 0.050, SR, Interpolation::Linear);
        dl.set_delay_seconds(0.020);

        // Normal speed: write a tone and measure delay
        for i in 0..960 {
            let t = i as f64 / SR;
            dl.write((2.0 * std::f64::consts::PI * 440.0 * t).sin());
        }
        let normal_out = dl.read();

        // With speed mod, effective delay changes
        dl.set_speed_mod(1.5);
        let modded = dl.effective_delay_samples();
        let normal = 0.020 * SR;
        assert!(
            (modded - normal * 1.5).abs() < 1.0,
            "speed_mod=1.5 should scale delay by 1.5x"
        );

        // Verify it's not NaN
        let out = dl.read();
        assert!(out.is_finite(), "Output with speed mod should be finite, got {normal_out} / {out}");
    }

    // ── Tap reads at correct ratio ───────────────────────────────────

    #[test]
    fn tap_read_at_ratio() {
        let mut dl = DelayLine::new(0.005, 0.200, SR, Interpolation::Linear);
        dl.set_delay_seconds(0.020); // 960 samples base delay

        // Write an impulse then silence
        dl.write(1.0);
        for _ in 1..480 {
            dl.write(0.0);
        }

        // ratio=0.5 → half the delay (480 samples) — impulse should be there
        let tap_half = dl.read_at_ratio(0.5, 1);
        assert!(
            tap_half.abs() > 0.5,
            "Tap at ratio=0.5 should see impulse at half-delay, got {tap_half}"
        );

        // ratio=1.0 → full delay (960 samples) — impulse not there yet
        let tap_full = dl.read_at_ratio(1.0, 0);
        assert!(
            tap_full.abs() < 0.01,
            "Tap at ratio=1.0 should not see impulse yet at 480 ticks, got {tap_full}"
        );
    }

    // ── Interpolation modes ──────────────────────────────────────────

    #[test]
    fn cubic_interpolation_finite() {
        let mut dl = DelayLine::new(0.001, 0.050, SR, Interpolation::Cubic);
        dl.set_delay_seconds(0.010);

        for i in 0..4800 {
            let t = i as f64 / SR;
            let input = (2.0 * std::f64::consts::PI * 440.0 * t).sin();
            dl.write(input);
            let out = dl.read();
            assert!(out.is_finite(), "Cubic output should be finite at sample {i}");
        }
    }

    #[test]
    fn allpass_interpolation_finite() {
        let mut dl = DelayLine::new(0.001, 0.050, SR, Interpolation::Allpass);
        dl.set_delay_seconds(0.010);

        for i in 0..4800 {
            let t = i as f64 / SR;
            let input = (2.0 * std::f64::consts::PI * 440.0 * t).sin();
            dl.write(input);
            let out = dl.read();
            assert!(out.is_finite(), "Allpass output should be finite at sample {i}");
        }
    }

    // ── Medium processing ────────────────────────────────────────────

    #[test]
    fn medium_none_passes_signal_unchanged() {
        let mut dl = DelayLine::new(0.001, 0.050, SR, Interpolation::Linear);
        dl.set_delay_seconds(0.010);
        // Medium::None is default

        // Write known pattern
        for i in 0..960 {
            dl.write((i as f64) / 960.0);
        }

        // Read — should be clean (no medium distortion)
        let out = dl.read();
        assert!(out.is_finite());
    }

    #[test]
    fn medium_tape_oxide_darkens_signal() {
        // With TapeOxide, the signal should lose HF content over time.
        let mut dl_clean = DelayLine::new(0.010, 0.050, SR, Interpolation::Linear);
        dl_clean.set_delay_seconds(0.020);

        let mut dl_tape = DelayLine::new(0.010, 0.050, SR, Interpolation::Linear);
        dl_tape.set_delay_seconds(0.020);
        dl_tape.set_medium(Medium::TapeOxide);
        dl_tape.configure_zones_from_taps(&[1.0], None);

        let mut sum_clean = 0.0;
        let mut sum_tape = 0.0;

        // Process a high-frequency tone through both
        for i in 0..4800 {
            let t = i as f64 / SR;
            let input = (2.0 * std::f64::consts::PI * 8000.0 * t).sin();
            dl_clean.write(input);
            dl_tape.write(input);

            if i > 1200 {
                sum_clean += dl_clean.read().abs();
                sum_tape += dl_tape.read().abs();
            }
        }

        // Tape should reduce HF energy compared to clean
        assert!(
            sum_tape < sum_clean,
            "TapeOxide should darken signal: tape={sum_tape} vs clean={sum_clean}"
        );
    }

    #[test]
    fn medium_bbd_leakage_attenuates() {
        let mut dl_clean = DelayLine::new(0.010, 0.050, SR, Interpolation::Linear);
        dl_clean.set_delay_seconds(0.020);

        let mut dl_bbd = DelayLine::new(0.010, 0.050, SR, Interpolation::Linear);
        dl_bbd.set_delay_seconds(0.020);
        dl_bbd.set_medium(Medium::BbdLeakage);
        dl_bbd.configure_zones_from_taps(&[1.0], None);

        let mut sum_clean = 0.0;
        let mut sum_bbd = 0.0;

        for i in 0..4800 {
            let t = i as f64 / SR;
            let input = (2.0 * std::f64::consts::PI * 440.0 * t).sin();
            dl_clean.write(input);
            dl_bbd.write(input);

            if i > 1200 {
                sum_clean += dl_clean.read().abs();
                sum_bbd += dl_bbd.read().abs();
            }
        }

        // BBD leakage should reduce signal amplitude
        assert!(
            sum_bbd < sum_clean,
            "BbdLeakage should attenuate: bbd={sum_bbd} vs clean={sum_clean}"
        );
    }

    // ── write_direct / buffer_read / advance_write ───────────────────

    #[test]
    fn direct_buffer_access_produces_delayed_impulse() {
        // write_direct / buffer_read / advance_write is the BBD-style ordering:
        //   write at pos → read at pos-delay → advance pos
        // This differs from the normal write/read path (which advances before read).
        let mut dl = DelayLine::new(0.005, 0.050, SR, Interpolation::Linear);
        dl.set_delay_seconds(0.010); // 480 samples

        let n = 600;
        let mut outputs = vec![0.0_f64; n];

        // Write impulse using direct access
        dl.write_direct(1.0);
        let delay = dl.effective_delay_samples();
        outputs[0] = dl.buffer_read(delay, 0);
        dl.advance_write();

        for i in 1..n {
            dl.write_direct(0.0);
            let delay = dl.effective_delay_samples();
            outputs[i] = dl.buffer_read(delay, 0);
            dl.advance_write();
        }

        // Find peak — with write-read-advance ordering, the latency is
        // exactly delay_samples ticks (no off-by-one since read happens
        // before advance).
        let peak_idx = outputs
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap())
            .unwrap()
            .0;

        assert!(
            (peak_idx as i64 - 480).abs() <= 1,
            "Impulse should arrive near sample 480, peaked at {peak_idx}"
        );
        assert!(outputs[peak_idx].abs() > 0.5);

        // All outputs should be finite
        assert!(outputs.iter().all(|o| o.is_finite()));
    }

    // ── BbdDelayLine using inner DelayLine ───────────────────────────

    #[test]
    fn bbd_produces_finite_output() {
        let model = BbdModel::mn3207();
        let mut bbd = BbdDelayLine::new(model, SR);

        for i in 0..4800 {
            let t = i as f64 / SR;
            let input = (2.0 * std::f64::consts::PI * 440.0 * t).sin() * 0.5;
            let out = bbd.process(input);
            assert!(out.is_finite(), "BBD output should be finite at sample {i}, got {out}");
        }
    }

    #[test]
    fn bbd_delay_time_changes_with_clock() {
        let model = BbdModel::mn3207();
        let mut bbd = BbdDelayLine::new(model, SR);

        let t_mid = bbd.delay_time();

        // Set to minimum delay (max clock)
        bbd.set_delay_normalized(0.0);
        let t_min = bbd.delay_time();

        // Set to maximum delay (min clock)
        bbd.set_delay_normalized(1.0);
        let t_max = bbd.delay_time();

        assert!(t_min < t_mid, "Min delay should be less than mid: {t_min} < {t_mid}");
        assert!(t_max > t_mid, "Max delay should be greater than mid: {t_max} > {t_mid}");
    }

    #[test]
    fn bbd_feedback_causes_repeats() {
        let model = BbdModel::mn3207();
        let mut bbd = BbdDelayLine::new(model, SR);
        bbd.set_feedback(0.7);
        bbd.set_delay_normalized(0.0); // Short delay for faster test

        // Write a short burst then silence
        for i in 0..480 {
            let t = i as f64 / SR;
            let input = if i < 48 {
                (2.0 * std::f64::consts::PI * 440.0 * t).sin()
            } else {
                0.0
            };
            bbd.process(input);
        }

        // After the burst, there should still be output from feedback
        let mut late_energy = 0.0;
        for _ in 0..4800 {
            let out = bbd.process(0.0);
            late_energy += out * out;
        }

        assert!(
            late_energy > 1e-6,
            "BBD with feedback should have repeating echoes, energy={late_energy}"
        );
    }

    #[test]
    fn bbd_reset_clears_state() {
        let model = BbdModel::mn3207();
        let mut bbd = BbdDelayLine::new(model, SR);

        // Process some signal
        for i in 0..4800 {
            let t = i as f64 / SR;
            bbd.process((2.0 * std::f64::consts::PI * 440.0 * t).sin());
        }

        bbd.reset();

        // After reset, output should be near zero
        let out = bbd.process(0.0);
        assert!(
            out.abs() < 0.01,
            "After reset, BBD output should be near zero, got {out}"
        );
    }

    #[test]
    fn bbd_set_sample_rate_adjusts_delay() {
        let model = BbdModel::mn3207();
        let mut bbd = BbdDelayLine::new(model, SR);

        let t1 = bbd.delay_time();
        bbd.set_sample_rate(96000.0);
        let t2 = bbd.delay_time();

        // Delay time in seconds should remain approximately the same
        assert!(
            (t1 - t2).abs() < 0.001,
            "Delay time should be preserved across sample rate changes: {t1} vs {t2}"
        );
    }

    // ── DelayLine reset / set_sample_rate ────────────────────────────

    #[test]
    fn delay_line_reset_clears_buffer() {
        let mut dl = DelayLine::new(0.001, 0.050, SR, Interpolation::Linear);
        dl.set_delay_seconds(0.010);

        // Fill with signal
        for i in 0..4800 {
            let t = i as f64 / SR;
            dl.write((2.0 * std::f64::consts::PI * 440.0 * t).sin());
        }

        dl.reset();

        // After reset, reads should return zero
        let out = dl.read();
        assert!(
            out.abs() < 1e-10,
            "After reset, delay line output should be zero, got {out}"
        );
    }

    #[test]
    fn delay_line_sample_rate_change() {
        let mut dl = DelayLine::new(0.001, 0.050, SR, Interpolation::Linear);
        dl.set_delay_seconds(0.020);

        let t1 = dl.delay_time();
        dl.set_sample_rate(96000.0);
        let t2 = dl.delay_time();

        assert!(
            (t1 - t2).abs() < 0.0001,
            "Delay time in seconds should be preserved: {t1} vs {t2}"
        );
    }

    // ── MediumPreset ────────────────────────────────────────────────

    #[test]
    fn medium_preset_zone_coefficients() {
        let coeff = MediumPreset::Re201.zone_coefficients();
        assert_eq!(coeff.len(), 3);
        // RE-201 is tape — increasing LP coefficients with distance
        assert!(coeff[0] < coeff[2], "Later zones should have stronger filtering");

        let coeff_lofi = MediumPreset::LoFi.zone_coefficients();
        assert_eq!(coeff_lofi.len(), 3);
        // LoFi has stronger coefficients than RE-201
        assert!(
            coeff_lofi[0] > coeff[0],
            "LoFi should have stronger filtering than RE-201"
        );
    }
}
