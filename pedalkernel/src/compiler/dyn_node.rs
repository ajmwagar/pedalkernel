//! Dynamic WDF tree node for runtime-constructed circuits.

use crate::elements::{Photocoupler, WdfLeaf};

// ═══════════════════════════════════════════════════════════════════════════
// Dynamic WDF tree node
// ═══════════════════════════════════════════════════════════════════════════

/// A node in a dynamically-constructed WDF binary tree.
///
/// Leaves are one-port elements; internal nodes are series/parallel adaptors.
/// All state is inline — zero heap allocation on the processing hot path.
pub(super) enum DynNode {
    Resistor {
        rp: f64,
    },
    Capacitor {
        capacitance: f64,
        rp: f64,
        state: f64,
    },
    Inductor {
        inductance: f64,
        rp: f64,
        state: f64,
    },
    VoltageSource {
        voltage: f64,
        rp: f64,
    },
    /// Variable resistor (potentiometer) — tagged with component ID for control binding.
    Pot {
        comp_id: String,
        max_resistance: f64,
        position: f64,
        rp: f64,
    },
    /// Photocoupler (Vactrol) — LDR with CdS carrier dynamics.
    Photocoupler {
        comp_id: String,
        inner: Photocoupler,
    },
    Series {
        left: Box<DynNode>,
        right: Box<DynNode>,
        rp: f64,
        gamma: f64,
        b1: f64,
        b2: f64,
    },
    Parallel {
        left: Box<DynNode>,
        right: Box<DynNode>,
        rp: f64,
        gamma: f64,
        b1: f64,
        b2: f64,
    },
}

impl DynNode {
    pub fn port_resistance(&self) -> f64 {
        match self {
            Self::Resistor { rp }
            | Self::Capacitor { rp, .. }
            | Self::Inductor { rp, .. }
            | Self::VoltageSource { rp, .. }
            | Self::Pot { rp, .. }
            | Self::Series { rp, .. }
            | Self::Parallel { rp, .. } => *rp,
            Self::Photocoupler { inner, .. } => inner.port_resistance(),
        }
    }

    /// Scatter-up: compute reflected wave (bottom → root), caching child waves.
    pub fn reflected(&mut self) -> f64 {
        match self {
            Self::Resistor { .. } | Self::Pot { .. } | Self::Photocoupler { .. } => 0.0,
            Self::Capacitor { state, .. } => *state,
            Self::Inductor { state, .. } => -*state,
            Self::VoltageSource { voltage, .. } => 2.0 * *voltage,
            Self::Series {
                left,
                right,
                b1,
                b2,
                ..
            } => {
                *b1 = left.reflected();
                *b2 = right.reflected();
                -(*b1 + *b2)
            }
            Self::Parallel {
                left,
                right,
                gamma,
                b1,
                b2,
                ..
            } => {
                *b1 = left.reflected();
                *b2 = right.reflected();
                *b1 + *gamma * (*b2 - *b1)
            }
        }
    }

    /// Scatter-down + state update: propagate incident wave (root → leaves).
    pub fn set_incident(&mut self, a: f64) {
        match self {
            Self::Resistor { .. }
            | Self::Pot { .. }
            | Self::VoltageSource { .. }
            | Self::Photocoupler { .. } => {}
            Self::Capacitor { state, .. } => *state = a,
            Self::Inductor { state, .. } => *state = a,
            Self::Series {
                left,
                right,
                gamma,
                b1,
                b2,
                ..
            } => {
                let sum = *b1 + *b2 + a;
                let a1 = *b1 - *gamma * sum;
                let a2 = *b2 - (1.0 - *gamma) * sum;
                left.set_incident(a1);
                right.set_incident(a2);
            }
            Self::Parallel {
                left,
                right,
                gamma,
                b1,
                b2,
                ..
            } => {
                let diff = *b2 - *b1;
                let a1 = a + (1.0 - *gamma) * diff;
                let a2 = a - *gamma * diff;
                left.set_incident(a1);
                right.set_incident(a2);
            }
        }
    }

    /// Set the voltage source value (searches recursively).
    pub fn set_voltage(&mut self, v: f64) {
        match self {
            Self::VoltageSource { voltage, .. } => *voltage = v,
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.set_voltage(v);
                right.set_voltage(v);
            }
            _ => {}
        }
    }

    /// Update a pot's position and resistance.  Returns true if found.
    pub fn set_pot(&mut self, target_id: &str, pos: f64) -> bool {
        match self {
            Self::Pot {
                comp_id,
                max_resistance,
                position,
                rp,
            } if comp_id == target_id => {
                *position = pos;
                *rp = (pos * *max_resistance).max(1.0);
                true
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.set_pot(target_id, pos) || right.set_pot(target_id, pos)
            }
            _ => false,
        }
    }

    /// Recompute all adaptor coefficients bottom-up (call after pot changes).
    pub fn recompute(&mut self) {
        match self {
            Self::Series {
                left,
                right,
                rp,
                gamma,
                ..
            } => {
                left.recompute();
                right.recompute();
                let r1 = left.port_resistance();
                let r2 = right.port_resistance();
                *rp = r1 + r2;
                *gamma = r1 / *rp;
            }
            Self::Parallel {
                left,
                right,
                rp,
                gamma,
                ..
            } => {
                left.recompute();
                right.recompute();
                let r1 = left.port_resistance();
                let r2 = right.port_resistance();
                *rp = r1 * r2 / (r1 + r2);
                *gamma = r2 / (r1 + r2);
            }
            _ => {}
        }
    }

    /// Reset all state (capacitor/inductor memory + adaptor caches).
    pub fn reset(&mut self) {
        match self {
            Self::Capacitor { state, .. } | Self::Inductor { state, .. } => *state = 0.0,
            Self::Photocoupler { inner, .. } => inner.reset(),
            Self::Series {
                left,
                right,
                b1,
                b2,
                ..
            }
            | Self::Parallel {
                left,
                right,
                b1,
                b2,
                ..
            } => {
                *b1 = 0.0;
                *b2 = 0.0;
                left.reset();
                right.reset();
            }
            _ => {}
        }
    }

    /// Update capacitor/inductor port resistances for a new sample rate.
    pub fn update_sample_rate(&mut self, fs: f64) {
        match self {
            Self::Capacitor {
                capacitance, rp, ..
            } => {
                *rp = 1.0 / (2.0 * fs * *capacitance);
            }
            Self::Inductor { inductance, rp, .. } => {
                *rp = 2.0 * fs * *inductance;
            }
            Self::Photocoupler { inner, .. } => {
                inner.set_sample_rate(fs);
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.update_sample_rate(fs);
                right.update_sample_rate(fs);
            }
            _ => {}
        }
    }

    /// Set LED drive level for a photocoupler. Returns true if found.
    pub fn set_photocoupler_led(&mut self, target_id: &str, led_drive: f64) -> bool {
        match self {
            Self::Photocoupler { comp_id, inner } if comp_id == target_id => {
                inner.set_led_drive(led_drive);
                true
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.set_photocoupler_led(target_id, led_drive)
                    || right.set_photocoupler_led(target_id, led_drive)
            }
            _ => false,
        }
    }
}

