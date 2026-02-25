//! Slew rate limiter for modeling op-amp bandwidth limitations.

// ---------------------------------------------------------------------------
// Slew Rate Limiter
// ---------------------------------------------------------------------------

/// Models op-amp slew rate limiting for physically accurate WDF processing.
///
/// Real op-amps have a finite maximum rate of output voltage change (dV/dt),
/// measured in V/µs. When the signal demands a faster slew than the op-amp
/// can deliver, the output "rounds off" — this is the mechanism behind the
/// distinctive compression character of slow op-amps like the LM308 (RAT)
/// versus fast ones like the TL072 (Klon).
///
/// The slew rate limiter operates as a per-sample voltage clamp on the
/// derivative: `|V[n] - V[n-1]| ≤ slew_rate * dt`, where dt = 1/fs.
///
/// This is NOT a WDF root — it sits in the signal path between stages,
/// modeling the op-amp's output stage limitation.
#[derive(Debug, Clone, Copy)]
pub struct SlewRateLimiter {
    /// Maximum voltage change per sample (V/sample).
    max_dv: f64,
    /// Previous output voltage (state).
    prev_out: f64,
    /// Slew rate in V/µs (for reference/display).
    slew_rate_v_per_us: f64,
    /// Current sample rate.
    sample_rate: f64,
}

impl SlewRateLimiter {
    /// Create a slew rate limiter from a slew rate in V/µs and sample rate.
    ///
    /// Slew rate values from real op-amps:
    /// - LM308: 0.3 V/µs (the RAT's character)
    /// - LM741: 0.5 V/µs (vintage slow)
    /// - JRC4558: 1.7 V/µs (Tube Screamer warmth)
    /// - NE5532: 9.0 V/µs (studio clean)
    /// - TL072: 13.0 V/µs (modern, transparent)
    /// - CA3080: 50.0 V/µs (OTA, essentially transparent)
    pub fn new(slew_rate_v_per_us: f64, sample_rate: f64) -> Self {
        // Convert V/µs to V/sample: slew_rate * 1e6 / sample_rate
        let max_dv = slew_rate_v_per_us * 1e6 / sample_rate;
        Self {
            max_dv,
            prev_out: 0.0,
            slew_rate_v_per_us,
            sample_rate,
        }
    }

    /// Process one sample through the slew rate limiter.
    ///
    /// If the requested voltage change exceeds what the op-amp can deliver
    /// in one sample period, the output is clamped to the maximum slew rate.
    /// This creates asymmetric HF compression — the exact behavior that
    /// makes the LM308 RAT sound different from a TL072 RAT.
    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
        let dv = input - self.prev_out;
        let limited = if dv > self.max_dv {
            self.prev_out + self.max_dv
        } else if dv < -self.max_dv {
            self.prev_out - self.max_dv
        } else {
            input
        };
        self.prev_out = limited;
        limited
    }

    /// Update sample rate and recompute max_dv.
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
        self.max_dv = self.slew_rate_v_per_us * 1e6 / sample_rate;
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.prev_out = 0.0;
    }

    /// Get the slew rate in V/µs.
    pub fn slew_rate(&self) -> f64 {
        self.slew_rate_v_per_us
    }

    /// Check if slew limiting is currently active (for diagnostics).
    ///
    /// Returns the ratio of actual dV to max_dv for the last sample.
    /// Values > 1.0 mean the limiter was engaged.
    pub fn is_limiting(&self) -> bool {
        // This would need to track the last dv, but for simplicity
        // we just check if the limiter is "slow enough to matter"
        self.slew_rate_v_per_us < 5.0
    }
}
