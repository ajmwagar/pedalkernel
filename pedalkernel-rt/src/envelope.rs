//! One-pole exponential decay envelope with slide-aware triggering.
//!
//! Designed for TB-303 MEG/VEG but usable as a general-purpose envelope.
//! `no_std` compatible, zero allocation, one multiply per sample.
//!
//! # 303 behavior
//!
//! - **Normal note** (gate rise): envelope retriggers to 1.0, decays exponentially.
//! - **Slide note** (gate held, pitch changes): envelope does NOT retrigger.
//!   It continues decaying from wherever it was. This is what makes 303 slides
//!   feel musical — the filter sweep carries over between notes.
//! - **Accent**: overrides decay time to a fixed short value (~30ms),
//!   regardless of the Decay knob.
//!
//! # Usage
//!
//! ```
//! use pedalkernel_rt::envelope::DecayEnvelope;
//!
//! let mut env = DecayEnvelope::new(48_000.0);
//! env.set_decay_time_ms(200.0);
//!
//! // Normal note: retrigger
//! env.trigger();
//! assert!(env.value() > 0.99);
//!
//! // Process samples (envelope decays)
//! for _ in 0..4800 { env.process(); }
//! assert!(env.value() < 1.0);
//!
//! // Slide note: suppress retrigger
//! let old_val = env.value();
//! env.trigger_if(false); // slide active = suppress
//! assert!((env.value() - old_val).abs() < 1e-9);
//! ```

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::math;

// ── DecayEnvelope ───────────────────────────────────────────────────────

/// One-pole exponential decay envelope.
///
/// Each `process()` call multiplies the internal value by `decay_coeff`,
/// producing an exponential decay from 1.0 toward 0.0. The decay
/// coefficient is derived from a time constant.
#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct DecayEnvelope {
    value: f64,
    decay_coeff: f64,
    sample_rate: f64,
}

impl DecayEnvelope {
    /// Create a new envelope with a default 200ms decay time.
    pub fn new(sample_rate: f64) -> Self {
        let mut env = Self {
            value: 0.0,
            decay_coeff: 0.0,
            sample_rate,
        };
        env.set_decay_time_ms(200.0);
        env
    }

    /// Update sample rate, preserving the current decay time.
    pub fn set_sample_rate(&mut self, sr: f64) {
        // Recover the current time constant, then recompute for new SR
        let old_time_samples = if self.decay_coeff > 0.0 && self.decay_coeff < 1.0 {
            -1.0 / math::ln(self.decay_coeff)
        } else {
            0.01 * self.sample_rate // fallback: 10ms
        };
        let time_ms = (old_time_samples / self.sample_rate) * 1000.0;
        self.sample_rate = sr;
        self.set_decay_time_ms(time_ms);
    }

    // ── Decay time ──────────────────────────────────────────────────────

    /// Set decay time in milliseconds.
    ///
    /// This is the time for the envelope to decay to ~37% (1/e) of its
    /// initial value. Typical 303 range: 5ms (fast) to 2000ms (slow).
    pub fn set_decay_time_ms(&mut self, ms: f64) {
        let time_samples = ms.max(0.1) * 0.001 * self.sample_rate;
        self.decay_coeff = math::exp(-1.0 / time_samples);
    }

    /// Set decay time from a 0..1 knob value.
    ///
    /// Maps exponentially: 0.0 → 5ms (fast), 1.0 → 2000ms (slow).
    /// Matches the 303's Decay knob feel.
    pub fn set_decay_from_knob(&mut self, knob: f64) {
        let clamped = knob.clamp(0.0, 1.0);
        let ms = 5.0 * math::powf(2000.0 / 5.0, clamped);
        self.set_decay_time_ms(ms);
    }

    /// Set the decay coefficient directly.
    ///
    /// Use for accent override: the 303 forces a fixed short decay
    /// (~30ms) regardless of the Decay knob. Compute the coefficient
    /// externally and set it here.
    pub fn set_decay_coeff(&mut self, coeff: f64) {
        self.decay_coeff = coeff.clamp(0.0, 1.0 - 1e-12);
    }

    /// Get the current decay coefficient (for save/restore).
    pub fn decay_coeff(&self) -> f64 {
        self.decay_coeff
    }

    // ── Triggering ──────────────────────────────────────────────────────

    /// Unconditional trigger: snap envelope to 1.0.
    ///
    /// Use on normal (non-slide) note gate rising edges.
    #[inline]
    pub fn trigger(&mut self) {
        self.value = 1.0;
    }

    /// Conditional trigger: only retrigger if `should_trigger` is true.
    ///
    /// For 303 slide behavior, pass `!slide_active`:
    /// - Normal note: `trigger_if(true)` → snaps to 1.0
    /// - Slide note: `trigger_if(false)` → no retrigger, envelope continues decaying
    #[inline]
    pub fn trigger_if(&mut self, should_trigger: bool) {
        if should_trigger {
            self.value = 1.0;
        }
    }

    // ── Processing ──────────────────────────────────────────────────────

    /// Advance the envelope by one sample and return the current value.
    ///
    /// One multiply per sample. Returns value in [0.0, 1.0].
    #[inline]
    pub fn process(&mut self) -> f64 {
        self.value *= self.decay_coeff;
        self.value
    }

    /// Read the current envelope value without advancing.
    #[inline]
    pub fn value(&self) -> f64 {
        self.value
    }

    /// Reset envelope to zero (silence).
    pub fn reset(&mut self) {
        self.value = 0.0;
    }

    /// Check if the envelope has decayed below a threshold.
    #[inline]
    pub fn is_silent(&self, threshold: f64) -> bool {
        self.value < threshold
    }
}

// ── Accent helper ───────────────────────────────────────────────────────

/// Compute the decay coefficient for a fixed time in milliseconds.
///
/// Useful for accent override: `env.set_decay_coeff(decay_coeff_ms(30.0, sr))`.
pub fn decay_coeff_ms(ms: f64, sample_rate: f64) -> f64 {
    let time_samples = ms.max(0.1) * 0.001 * sample_rate;
    math::exp(-1.0 / time_samples)
}

// ── Tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::eprintln;

    const SR: f64 = 48_000.0;

    #[test]
    fn starts_at_zero() {
        let env = DecayEnvelope::new(SR);
        assert!((env.value() - 0.0).abs() < 1e-12, "should start at 0");
    }

    #[test]
    fn trigger_snaps_to_one() {
        let mut env = DecayEnvelope::new(SR);
        env.trigger();
        assert!(
            (env.value() - 1.0).abs() < 1e-12,
            "trigger should snap to 1.0"
        );
    }

    #[test]
    fn decays_after_trigger() {
        let mut env = DecayEnvelope::new(SR);
        env.set_decay_time_ms(100.0);
        env.trigger();

        let initial = env.value();
        for _ in 0..4800 {
            env.process();
        } // 100ms at 48kHz
        let after_100ms = env.value();

        eprintln!("  decay: initial={initial:.4}, after 100ms={after_100ms:.6}");
        assert!(after_100ms < initial, "should decay");
        assert!(after_100ms > 0.0, "should not reach zero instantly");

        // After one time constant (100ms), should be near 1/e ≈ 0.368
        assert!(
            (after_100ms - 1.0 / core::f64::consts::E).abs() < 0.05,
            "after one time constant: expected ~0.368, got {after_100ms:.4}"
        );
    }

    #[test]
    fn fast_decay_vs_slow_decay() {
        let n = 9600; // 200ms

        let mut fast = DecayEnvelope::new(SR);
        fast.set_decay_from_knob(0.1);
        fast.trigger();
        for _ in 0..n {
            fast.process();
        }

        let mut slow = DecayEnvelope::new(SR);
        slow.set_decay_from_knob(0.9);
        slow.trigger();
        for _ in 0..n {
            slow.process();
        }

        eprintln!("  knob 0.1 after 200ms: {:.6}", fast.value());
        eprintln!("  knob 0.9 after 200ms: {:.6}", slow.value());

        assert!(
            slow.value() > fast.value(),
            "slow decay ({:.6}) should retain more than fast ({:.6})",
            slow.value(),
            fast.value()
        );
    }

    #[test]
    fn knob_zero_is_fast() {
        let mut env = DecayEnvelope::new(SR);
        env.set_decay_from_knob(0.0); // fastest
        env.trigger();
        for _ in 0..4800 {
            env.process();
        } // 100ms
        eprintln!("  knob=0 after 100ms: {:.6}", env.value());
        assert!(
            env.value() < 0.01,
            "knob=0 should decay very fast: {:.6}",
            env.value()
        );
    }

    #[test]
    fn knob_one_is_slow() {
        let mut env = DecayEnvelope::new(SR);
        env.set_decay_from_knob(1.0); // slowest
        env.trigger();
        for _ in 0..4800 {
            env.process();
        } // 100ms
        eprintln!("  knob=1 after 100ms: {:.6}", env.value());
        assert!(
            env.value() > 0.8,
            "knob=1 should barely decay in 100ms: {:.6}",
            env.value()
        );
    }

    // ── Slide behavior ──────────────────────────────────────────────────

    #[test]
    fn trigger_if_true_retriggers() {
        let mut env = DecayEnvelope::new(SR);
        env.set_decay_time_ms(100.0);
        env.trigger();
        for _ in 0..9600 {
            env.process();
        } // decay to ~0.135
        let before = env.value();

        env.trigger_if(true); // should retrigger
        assert!(
            (env.value() - 1.0).abs() < 1e-12,
            "trigger_if(true) should snap to 1.0, got {:.6}",
            env.value()
        );
        assert!(env.value() > before, "should jump above decayed value");
    }

    #[test]
    fn trigger_if_false_does_not_retrigger() {
        let mut env = DecayEnvelope::new(SR);
        env.set_decay_time_ms(100.0);
        env.trigger();
        for _ in 0..9600 {
            env.process();
        } // decay
        let before = env.value();

        env.trigger_if(false); // slide — should NOT retrigger
        let after = env.value();

        eprintln!("  slide suppress: before={before:.6}, after={after:.6}");
        assert!(
            (after - before).abs() < 1e-12,
            "trigger_if(false) should not change value: before={before:.6}, after={after:.6}"
        );
    }

    #[test]
    fn slide_carries_envelope_state() {
        // Simulate: note1 triggers, decays 50ms, note2 slides (no retrigger),
        // continues decaying. The value should be continuous — no jump.
        let mut env = DecayEnvelope::new(SR);
        env.set_decay_time_ms(200.0);

        // Note 1: trigger and decay for 50ms
        env.trigger();
        for _ in 0..2400 {
            // 50ms
            env.process();
        }
        let at_slide = env.value();

        // Note 2: slide (no retrigger)
        env.trigger_if(false);
        let after_slide_trigger = env.value();
        assert!(
            (after_slide_trigger - at_slide).abs() < 1e-12,
            "slide should not change value"
        );

        // Continue decaying — should be monotonically decreasing
        let mut prev = after_slide_trigger;
        for _ in 0..2400 {
            // another 50ms
            let v = env.process();
            assert!(
                v <= prev + 1e-15,
                "should be monotonically decreasing during slide"
            );
            prev = v;
        }

        eprintln!("  slide continuity: at_slide={at_slide:.6}, after 50ms more={prev:.6}");
        assert!(
            prev < at_slide,
            "should have continued decaying after slide"
        );
    }

    // ── Accent override ─────────────────────────────────────────────────

    #[test]
    fn accent_override_shortens_decay() {
        let n = 4800; // 100ms

        // Normal: slow decay
        let mut normal = DecayEnvelope::new(SR);
        normal.set_decay_from_knob(0.9);
        normal.trigger();
        for _ in 0..n {
            normal.process();
        }

        // Accented: override to 30ms regardless of knob
        let mut accented = DecayEnvelope::new(SR);
        accented.set_decay_from_knob(0.9); // same slow knob
        accented.set_decay_coeff(decay_coeff_ms(30.0, SR)); // accent override
        accented.trigger();
        for _ in 0..n {
            accented.process();
        }

        eprintln!(
            "  accent: normal={:.6}, accented={:.6}",
            normal.value(),
            accented.value()
        );
        assert!(
            accented.value() < normal.value(),
            "accented ({:.6}) should decay faster than normal ({:.6})",
            accented.value(),
            normal.value()
        );
    }

    #[test]
    fn accent_decay_coeff_ms_helper() {
        let coeff = decay_coeff_ms(30.0, SR);
        assert!(
            coeff > 0.0 && coeff < 1.0,
            "coeff should be in (0,1): {coeff}"
        );

        // After 30ms (1440 samples), value should be near 1/e
        let mut val = 1.0;
        for _ in 0..1440 {
            val *= coeff;
        }
        eprintln!("  30ms coeff: {coeff:.8}, value after 30ms: {val:.4}");
        assert!(
            (val - 1.0 / core::f64::consts::E).abs() < 0.05,
            "after one time constant: expected ~0.368, got {val:.4}"
        );
    }

    // ── Reset and silent ────────────────────────────────────────────────

    #[test]
    fn reset_zeroes() {
        let mut env = DecayEnvelope::new(SR);
        env.trigger();
        assert!(env.value() > 0.99);
        env.reset();
        assert!(
            (env.value() - 0.0).abs() < 1e-12,
            "reset should zero the value"
        );
    }

    #[test]
    fn is_silent() {
        let mut env = DecayEnvelope::new(SR);
        assert!(env.is_silent(0.001), "should be silent at zero");
        env.trigger();
        assert!(!env.is_silent(0.001), "should not be silent after trigger");
        // Decay until silent
        env.set_decay_time_ms(5.0); // very fast
        for _ in 0..48000 {
            env.process();
        } // 1 second
        assert!(env.is_silent(1e-6), "should be silent after long decay");
    }

    // ── Process returns current value ───────────────────────────────────

    #[test]
    fn process_returns_value_after_decay() {
        let mut env = DecayEnvelope::new(SR);
        env.set_decay_time_ms(100.0);
        env.trigger();

        let v1 = env.process();
        let v2 = env.process();
        assert!(
            v1 > v2,
            "successive process() calls should decrease: {v1} > {v2}"
        );
        assert!(
            (v1 - env.decay_coeff()).abs() < 1e-12,
            "first process after trigger should be decay_coeff"
        );
    }

    // ── Sample rate change preserves behavior ───────────────────────────

    #[test]
    fn sample_rate_change_preserves_decay_time() {
        let mut env = DecayEnvelope::new(48_000.0);
        env.set_decay_time_ms(100.0);
        env.trigger();

        // Measure value after 100ms at 48kHz
        for _ in 0..4800 {
            env.process();
        }
        let val_48k = env.value();

        // Same thing at 96kHz
        let mut env96 = DecayEnvelope::new(96_000.0);
        env96.set_decay_time_ms(100.0);
        env96.trigger();
        for _ in 0..9600 {
            env96.process();
        } // 100ms at 96kHz
        let val_96k = env96.value();

        eprintln!("  48kHz after 100ms: {val_48k:.6}");
        eprintln!("  96kHz after 100ms: {val_96k:.6}");
        assert!(
            (val_48k - val_96k).abs() < 0.01,
            "same decay time should produce same value at different sample rates"
        );
    }
}
