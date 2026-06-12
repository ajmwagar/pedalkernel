//! One-pole pitch glide (portamento) with gate-aware activation.
//!
//! `no_std` compatible, zero allocation. One multiply + add per sample.
//!
//! The 303's slide behavior: when gate stays high between notes (legato),
//! pitch glides to the new target. On gate retrigger (staccato), pitch
//! snaps instantly.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::{math, Wave};

/// One-pole lowpass glide on pitch CV.
///
/// When `active`, the output pitch slews toward the target at a rate
/// determined by `glide_coeff`. When not active, output snaps to target.
#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Glide {
    current: Wave,
    target: Wave,
    glide_coeff: Wave, // per-sample smoothing coefficient (0..1, higher = slower)
    active: bool,
    sample_rate: crate::Wave,
}

impl Glide {
    pub fn new(sample_rate: crate::Wave) -> Self {
        Self {
            current: 0.0 as Wave,
            target: 0.0 as Wave,
            glide_coeff: 0.0 as Wave,
            active: false,
            sample_rate,
        }
    }

    pub fn set_sample_rate(&mut self, sr: crate::Wave) {
        self.sample_rate = sr;
    }

    /// Set glide time from a 0..1 knob.
    /// 0.0 = instant (no glide), 1.0 = ~360ms glide (Devil Fish Manual max).
    pub fn set_time_from_knob(&mut self, knob: crate::Wave) {
        let clamped = knob.clamp(0.0, 1.0);
        if clamped < 1e-4 {
            self.glide_coeff = 0.0 as Wave; // instant
        } else {
            let time_ms = 5.0 * math::powf(360.0 / 5.0, clamped);
            let time_samples = time_ms * 0.001 * self.sample_rate;
            self.glide_coeff = math::exp(-1.0 / time_samples) as Wave;
        }
    }

    /// Set target pitch (V/oct).
    pub fn set_target(&mut self, voct: crate::Wave) {
        self.target = voct as Wave;
    }

    /// Activate glide (legato: gate held, new pitch).
    pub fn activate(&mut self) {
        self.active = true;
    }

    /// Deactivate glide (slide gate went low). Pitch will snap on next process()
    /// call rather than gliding. Does not change the current pitch value.
    pub fn deactivate(&mut self) {
        self.active = false;
    }

    /// Snap to target and deactivate (staccato: gate retrigger).
    pub fn snap_to_target(&mut self) {
        self.current = self.target;
        self.active = false;
    }

    /// Process one sample. Returns the current (possibly gliding) pitch.
    #[inline]
    pub fn process(&mut self) -> Wave {
        if self.active && self.glide_coeff > 0.0 as Wave {
            // One-pole lowpass: current = coeff * current + (1-coeff) * target
            self.current =
                self.glide_coeff * self.current + (1.0 as Wave - self.glide_coeff) * self.target;
        } else {
            self.current = self.target;
        }
        self.current
    }

    /// Current pitch value (without advancing).
    pub fn value(&self) -> Wave {
        self.current
    }

    /// Returns true if glide is currently active (legato/slide mode).
    pub fn is_active(&self) -> bool {
        self.active
    }

    pub fn reset(&mut self) {
        self.current = self.target;
        self.active = false;
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::eprintln;

    const SR: crate::Wave = 48_000.0;

    #[test]
    fn snap_is_instant() {
        let mut g = Glide::new(SR);
        g.set_time_from_knob(0.9);
        g.set_target(3.0);
        g.snap_to_target();
        assert!((g.value() - 3.0).abs() < 1e-12);
    }

    #[test]
    fn glide_is_smooth() {
        let mut g = Glide::new(SR);
        g.set_time_from_knob(0.5);
        g.set_target(0.0);
        g.snap_to_target(); // start at 0

        g.set_target(1.0);
        g.activate();

        // After 10ms, should be partway there
        for _ in 0..480 {
            g.process();
        }
        let mid = g.value();
        eprintln!("  glide after 10ms: {mid:.4}");
        assert!(mid > 0.01, "should have started moving: {mid}");
        assert!(mid < 0.99, "should not have arrived yet: {mid}");
    }

    #[test]
    fn glide_converges() {
        let mut g = Glide::new(SR);
        g.set_time_from_knob(0.3);
        g.set_target(0.0);
        g.snap_to_target();

        g.set_target(2.0);
        g.activate();

        for _ in 0..48000 {
            g.process();
        } // 1 second
        eprintln!("  glide after 1s: {:.6}", g.value());
        assert!((g.value() - 2.0).abs() < 0.01, "should converge to target");
    }

    #[test]
    fn inactive_snaps() {
        let mut g = Glide::new(SR);
        g.set_time_from_knob(0.9);
        g.set_target(0.0);
        g.snap_to_target();

        // Don't activate — just set target and process
        g.set_target(5.0);
        let v = g.process();
        assert!((v - 5.0).abs() < 1e-12, "inactive glide should snap: {v}");
    }

    #[test]
    fn knob_zero_is_instant() {
        let mut g = Glide::new(SR);
        g.set_time_from_knob(0.0);
        g.set_target(0.0);
        g.snap_to_target();

        g.set_target(3.0);
        g.activate();
        let v = g.process();
        assert!(
            (v - 3.0).abs() < 1e-12,
            "knob=0 should be instant even when active: {v}"
        );
    }
}
