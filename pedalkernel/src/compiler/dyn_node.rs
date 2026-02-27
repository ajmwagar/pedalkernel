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
    /// Capacitor with leakage resistance and/or dielectric absorption.
    ///
    /// Leakage is modeled as a state decay, representing the RC time constant
    /// of the capacitor discharging through the parallel leakage resistance.
    /// This causes the capacitor to slowly lose its charge over time.
    ///
    /// Dielectric absorption causes the cap to "remember" its previous charge
    /// and slowly creep back toward it after discharge — a subtle compression
    /// effect on low frequencies.
    LeakyCapacitor {
        capacitance: f64,
        /// Port resistance: 1/(2*fs*C)
        rp: f64,
        /// Main capacitor state (z^{-1} of incident wave)
        state: f64,
        /// Leakage decay factor per sample: exp(-1/(fs * R_leak * C))
        /// When no leakage, this is 1.0 (no decay)
        leakage_decay: f64,
        /// Dielectric absorption coefficient (0.0-1.0), None = no DA
        da_coef: Option<f64>,
        /// DA state: slowly tracks main state, releases charge back
        da_state: f64,
        /// DA rate: how fast da_state tracks state (per sample)
        /// Typical: 1/(fs * T_da) where T_da ≈ 0.1-1.0 seconds
        da_rate: f64,
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
    /// Switched resistor for fork() routing paths.
    /// When active (switch position matches path_index), uses low resistance.
    /// When inactive, uses high resistance (effectively open circuit).
    SwitchedResistor {
        /// ID of the controlling switch component
        switch_id: String,
        /// Which fork path this resistor represents (0, 1, 2, ...)
        path_index: usize,
        /// Total number of paths in this fork
        num_paths: usize,
        /// Resistance when this path is active (typically ~1Ω)
        r_active: f64,
        /// Resistance when this path is inactive (typically ~1MΩ)
        r_inactive: f64,
        /// Current switch position
        position: usize,
        /// Port resistance (changes based on active state)
        rp: f64,
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
            | Self::LeakyCapacitor { rp, .. }
            | Self::Inductor { rp, .. }
            | Self::VoltageSource { rp, .. }
            | Self::Pot { rp, .. }
            | Self::SwitchedResistor { rp, .. }
            | Self::Series { rp, .. }
            | Self::Parallel { rp, .. } => *rp,
            Self::Photocoupler { inner, .. } => inner.port_resistance(),
        }
    }

    /// Scatter-up: compute reflected wave (bottom → root), caching child waves.
    pub fn reflected(&mut self) -> f64 {
        match self {
            Self::Resistor { .. }
            | Self::Pot { .. }
            | Self::Photocoupler { .. }
            | Self::SwitchedResistor { .. } => 0.0,
            Self::Capacitor { state, .. } => *state,
            Self::LeakyCapacitor {
                state,
                da_coef,
                da_state,
                ..
            } => {
                // Reflected wave: same as ideal capacitor
                let b = *state;

                // Add dielectric absorption contribution if present
                // DA causes the cap to "creep" back toward previously absorbed voltage
                if let Some(da) = da_coef {
                    b + *da * (*da_state - *state)
                } else {
                    b
                }
            }
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
            | Self::Photocoupler { .. }
            | Self::SwitchedResistor { .. } => {}
            Self::Capacitor { state, .. } => *state = a,
            Self::LeakyCapacitor {
                state,
                leakage_decay,
                da_coef,
                da_state,
                da_rate,
                ..
            } => {
                // Update state with incident wave, then apply leakage decay
                // The leakage causes the capacitor to slowly discharge toward 0
                *state = a * *leakage_decay;

                // Update dielectric absorption state (slow tracking)
                if da_coef.is_some() {
                    // DA state slowly follows the main capacitor state
                    // This creates the "memory" effect
                    *da_state += (*state - *da_state) * *da_rate;
                }
            }
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
            Self::LeakyCapacitor {
                state, da_state, ..
            } => {
                *state = 0.0;
                *da_state = 0.0;
            }
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
            Self::LeakyCapacitor {
                capacitance,
                rp,
                leakage_decay,
                da_coef,
                da_rate,
                ..
            } => {
                // Update port resistance
                *rp = 1.0 / (2.0 * fs * *capacitance);

                // leakage_decay is set at construction time based on R_leak and C
                // It doesn't need to change with sample rate because it's based on
                // the RC time constant. However, we need to recalculate if fs changes.
                // For now, leakage_decay is fixed at construction.
                // TODO: Store leakage_r to allow sample rate changes
                let _ = leakage_decay; // Suppress unused warning

                // Update DA rate for new sample rate
                if da_coef.is_some() {
                    const DA_TIME_CONSTANT: f64 = 0.5; // 500ms
                    *da_rate = 1.0 / (fs * DA_TIME_CONSTANT);
                }
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

    /// Set switch position for all SwitchedResistor nodes controlled by this switch.
    /// Updates port resistance based on whether the path is active.
    /// Returns the number of switched resistors updated.
    pub fn set_switch_position(&mut self, target_switch: &str, new_position: usize) -> usize {
        match self {
            Self::SwitchedResistor {
                switch_id,
                path_index,
                position,
                r_active,
                r_inactive,
                rp,
                ..
            } if switch_id == target_switch => {
                *position = new_position;
                // Update port resistance based on whether this path is now active
                *rp = if *path_index == new_position {
                    *r_active
                } else {
                    *r_inactive
                };
                1
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.set_switch_position(target_switch, new_position)
                    + right.set_switch_position(target_switch, new_position)
            }
            _ => 0,
        }
    }

    /// Debug dump: print tree structure with port resistances and coefficients.
    /// Returns a multi-line string showing the WDF tree hierarchy.
    pub fn debug_dump(&self, indent: usize) -> String {
        let pad = "  ".repeat(indent);
        match self {
            Self::Resistor { rp } => {
                format!("{pad}Resistor(Rp={rp:.1}Ω)")
            }
            Self::Capacitor { capacitance, rp, state } => {
                format!("{pad}Capacitor(C={capacitance:.3e}F, Rp={rp:.1}Ω, state={state:.6})")
            }
            Self::LeakyCapacitor { capacitance, rp, state, leakage_decay, .. } => {
                format!("{pad}LeakyCapacitor(C={capacitance:.3e}F, Rp={rp:.1}Ω, state={state:.6}, decay={leakage_decay:.6})")
            }
            Self::Inductor { inductance, rp, state } => {
                format!("{pad}Inductor(L={inductance:.3e}H, Rp={rp:.1}Ω, state={state:.6})")
            }
            Self::VoltageSource { voltage, rp } => {
                format!("{pad}VoltageSource(V={voltage:.3}V, Rp={rp:.1}Ω)")
            }
            Self::Pot { comp_id, max_resistance, position, rp } => {
                format!("{pad}Pot(id=\"{comp_id}\", max={max_resistance:.1}Ω, pos={position:.3}, Rp={rp:.1}Ω)")
            }
            Self::Photocoupler { comp_id, inner } => {
                format!("{pad}Photocoupler(id=\"{comp_id}\", Rp={:.1}Ω)", inner.port_resistance())
            }
            Self::SwitchedResistor { switch_id, path_index, position, rp, .. } => {
                format!("{pad}SwitchedResistor(switch=\"{switch_id}\", path={path_index}, pos={position}, Rp={rp:.1}Ω)")
            }
            Self::Series { left, right, rp, gamma, b1, b2 } => {
                let mut s = format!("{pad}Series(Rp={rp:.1}Ω, γ={gamma:.6}, b1={b1:.6}, b2={b2:.6})\n");
                s.push_str(&left.debug_dump(indent + 1));
                s.push('\n');
                s.push_str(&right.debug_dump(indent + 1));
                s
            }
            Self::Parallel { left, right, rp, gamma, b1, b2 } => {
                let mut s = format!("{pad}Parallel(Rp={rp:.1}Ω, γ={gamma:.6}, b1={b1:.6}, b2={b2:.6})\n");
                s.push_str(&left.debug_dump(indent + 1));
                s.push('\n');
                s.push_str(&right.debug_dump(indent + 1));
                s
            }
        }
    }

    /// Count total nodes in tree (for statistics).
    pub fn node_count(&self) -> usize {
        match self {
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                1 + left.node_count() + right.node_count()
            }
            _ => 1,
        }
    }
}

