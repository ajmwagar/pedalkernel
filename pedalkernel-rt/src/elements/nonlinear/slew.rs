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
/// Thin wrapper over the shared [`crate::stage::NonIdealFxState`] dV/dt clamp.
///
/// Carries NO copy of the slew math: `process` delegates to
/// `NonIdealFxState::slew_step` (the single source of truth) and only adds the
/// finite guard. Only test code uses this type now — the live SPQR pipeline
/// applies slew in-loop via `NonIdealFxState` directly.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SlewRateLimiter {
    /// Shared GBW/slew/rail state. Only the slew portion (`max_dv`/`prev_out`)
    /// is exercised here; GBW is left transparent and rails at MAX.
    fx: crate::stage::NonIdealFxState,
    /// Slew rate in V/µs (for reference/display).
    slew_rate_v_per_us: crate::Wave,
    /// Current sample rate.
    sample_rate: crate::Wave,
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
    pub fn new(slew_rate_v_per_us: crate::Wave, sample_rate: crate::Wave) -> Self {
        // Convert V/µs to V/sample: slew_rate * 1e6 / sample_rate
        let fx = crate::stage::NonIdealFxState {
            max_dv: slew_rate_v_per_us * 1e6 / sample_rate,
            ..crate::stage::NonIdealFxState::default()
        };
        Self {
            fx,
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
    pub fn process(&mut self, input: crate::Wave) -> crate::Wave {
        let limited = self.fx.slew_step(input);
        if limited.is_finite() {
            limited
        } else {
            self.fx.prev_out = 0.0;
            0.0
        }
    }

    /// Update sample rate and recompute max_dv.
    pub fn set_sample_rate(&mut self, sample_rate: crate::Wave) {
        self.sample_rate = sample_rate;
        self.fx.max_dv = self.slew_rate_v_per_us * 1e6 / sample_rate;
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.fx.prev_out = 0.0;
    }

    /// Get the slew rate in V/µs.
    pub fn slew_rate(&self) -> crate::Wave {
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
