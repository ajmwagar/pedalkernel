//! Dynamic WDF tree node for runtime-constructed circuits.

use crate::dsl::PotTaper;
use crate::elements::{JfetVariableResistor, Photocoupler, WdfLeaf};
use crate::tree::RTypeAdaptor;

// ═══════════════════════════════════════════════════════════════════════════
// Dynamic WDF tree node
// ═══════════════════════════════════════════════════════════════════════════

/// A node in a dynamically-constructed WDF binary tree.
///
/// Leaves are one-port elements; internal nodes are series/parallel adaptors.
/// All state is inline — zero heap allocation on the processing hot path.
#[derive(Clone)]
pub(super) enum DynNode {
    Resistor {
        comp_id: Option<String>,
        rp: f64,
        /// Last incident wave for voltage extraction: V = last_a / 2.
        last_a: f64,
    },
    Capacitor {
        comp_id: Option<String>,
        capacitance: f64,
        rp: f64,
        state: f64,
        /// Last reflected wave (captured before state update, for voltage extraction)
        last_b: f64,
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
        comp_id: Option<String>,
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
        comp_id: Option<String>,
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
        taper: PotTaper,
        rp: f64,
        /// Last incident wave, for output voltage extraction: V = last_a / 2.
        last_a: f64,
    },
    /// Photocoupler (Vactrol) — LDR with CdS carrier dynamics.
    Photocoupler {
        comp_id: String,
        inner: Photocoupler,
        /// Previous sample's resistance for slew rate checking (runtime-warnings).
        #[allow(dead_code)]
        prev_resistance: f64,
    },
    /// JFET variable resistor — Rds controlled by Vgs (phaser applications).
    JfetVr {
        comp_id: String,
        inner: JfetVariableResistor,
        /// Previous sample's Rds for slew rate checking (runtime-warnings).
        #[allow(dead_code)]
        prev_rds: f64,
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
        #[allow(dead_code)]
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
    /// Ideal transformer adaptor connecting a secondary subtree to the main tree.
    ///
    /// The transformer scales waves by the turns ratio:
    /// - scatter_up: b_primary = n × b_secondary
    /// - scatter_down: a_secondary = a_primary / n
    ///
    /// Port resistance at primary = n² × R_secondary
    ///
    /// The `secondary` child is the subtree connected at the transformer's
    /// secondary winding. The primary side faces the parent (root direction).
    Transformer {
        /// Secondary subtree (the child of this adaptor)
        secondary: Box<DynNode>,
        /// Turns ratio: n = N_primary / N_secondary
        /// - n > 1: step-down (voltage decreases primary→secondary)
        /// - n < 1: step-up (voltage increases primary→secondary)
        turns_ratio: f64,
        /// Port resistance seen from primary = n² × R_secondary
        rp: f64,
        /// Cached secondary reflected wave
        b_sec: f64,
    },
    /// Fixed cathode bias voltage source for push-pull stages.
    ///
    /// Behaves like a VoltageSource but its `set_voltage()` is a no-op —
    /// it maintains a fixed DC level representing the cathode bias rail
    /// (e.g., -3.1V for the Fairchild 670's A_bal supply).
    /// This prevents the plate supply sweep (`set_voltage(vs=240)`) from
    /// overwriting the cathode bias.
    CathodeBiasSource {
        voltage: f64,
        rp: f64,
    },
    /// Unit delay for cross-coupled cathode exchange in push-pull stages.
    ///
    /// Each push/pull half has a UnitDelay at the cathode. After both halves
    /// process, the states are exchanged: push's outgoing wave becomes pull's
    /// incoming wave (and vice versa), modeling the bidirectional cathode
    /// bypass capacitor interaction with 1-sample latency.
    UnitDelay {
        rp: f64,
        /// Outgoing state: this half's incident wave, stored for the partner.
        state: f64,
        /// Incoming state: partner half's incident wave from the previous sample.
        partner_state: f64,
    },
    /// N-port R-type adaptor for non-series/parallel topologies.
    ///
    /// Used for 3-winding transformers, bridged-T networks, and other
    /// topologies that cannot be decomposed into binary adaptor trees.
    /// The scattering matrix is pre-computed at compile time from MNA.
    ///
    /// Children are ordered: child₀, child₁, ..., child_{n-2}.
    /// The parent (adapted, reflection-free) port is implicit (port n-1).
    RType {
        /// The R-type adaptor with pre-computed scattering matrix.
        adaptor: RTypeAdaptor,
        /// Child subtrees (one per non-adapted port).
        children: Vec<Box<DynNode>>,
    },
}

impl DynNode {
    pub fn port_resistance(&self) -> f64 {
        match self {
            Self::Resistor { rp, .. }
            | Self::Capacitor { rp, .. }
            | Self::LeakyCapacitor { rp, .. }
            | Self::Inductor { rp, .. }
            | Self::VoltageSource { rp, .. }
            | Self::CathodeBiasSource { rp, .. }
            | Self::UnitDelay { rp, .. }
            | Self::Pot { rp, .. }
            | Self::SwitchedResistor { rp, .. }
            | Self::Series { rp, .. }
            | Self::Parallel { rp, .. }
            | Self::Transformer { rp, .. } => *rp,
            Self::RType { adaptor, .. } => adaptor.port_resistance,
            Self::Photocoupler { inner, .. } => inner.port_resistance(),
            Self::JfetVr { inner, .. } => inner.port_resistance(),
        }
    }

    /// Returns true if the tree's root node is a Series adaptor.
    ///
    /// Used by the compiler to detect when the VS contribution to b_tree is
    /// negated by the Series adaptor formula `b = -(b1 + b2)`, which inverts
    /// the sign of the Thevenin voltage seen by the NL root.  When true,
    /// triode stages must negate VS so that `b_tree = 2*B+` (positive).
    pub fn is_series_root(&self) -> bool {
        matches!(self, Self::Series { .. })
    }

    /// Find a capacitor's state (DC voltage) by component ID.
    /// Searches the tree recursively. Returns None if not found.
    pub fn find_cap_state(&self, target_id: &str) -> Option<f64> {
        match self {
            Self::Capacitor { comp_id: Some(id), state, .. }
            | Self::LeakyCapacitor { comp_id: Some(id), state, .. } if id == target_id => {
                Some(*state)
            }
            Self::Series { left, right, .. }
            | Self::Parallel { left, right, .. } => {
                left.find_cap_state(target_id)
                    .or_else(|| right.find_cap_state(target_id))
            }
            Self::Transformer { secondary, .. } => secondary.find_cap_state(target_id),
            Self::RType { children, .. } => {
                children.iter().find_map(|c| c.find_cap_state(target_id))
            }
            _ => None,
        }
    }

    /// Scatter-up: compute reflected wave (bottom → root), caching child waves.
    pub fn reflected(&mut self) -> f64 {
        match self {
            Self::Resistor { .. }
            | Self::Pot { .. }
            | Self::Photocoupler { .. }
            | Self::JfetVr { .. }
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
            Self::VoltageSource { voltage, .. } | Self::CathodeBiasSource { voltage, .. } => {
                2.0 * *voltage
            }
            Self::UnitDelay { partner_state, .. } => *partner_state,
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
            Self::Transformer {
                secondary,
                turns_ratio,
                b_sec,
                ..
            } => {
                // Get secondary reflected wave and scale by turns ratio
                *b_sec = secondary.reflected();
                *turns_ratio * *b_sec
            }
            Self::RType { adaptor, children } => {
                // Collect reflected waves from all children
                let b_children: Vec<f64> = children.iter_mut().map(|c| c.reflected()).collect();
                adaptor.scatter_up(&b_children)
            }
        }
    }

    /// Scatter-down + state update: propagate incident wave (root → leaves).
    pub fn set_incident(&mut self, a: f64) {
        match self {
            Self::Resistor { last_a, .. } | Self::Pot { last_a, .. } => { *last_a = a; }
            Self::VoltageSource { .. }
            | Self::CathodeBiasSource { .. }
            | Self::Photocoupler { .. }
            | Self::JfetVr { .. }
            | Self::SwitchedResistor { .. } => {}
            Self::UnitDelay { state, .. } => *state = a,
            Self::Capacitor { state, last_b, .. } => {
                // Capture the reflected wave (current state) before updating
                // This allows voltage extraction: V = (a + last_b) / 2
                *last_b = *state;
                *state = a;
            }
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
                // Consistent with scatter_up b3 = (1-γ)·b1 + γ·b2:
                //   v1 = v3 → a1 = a3 + γ·(b2-b1)
                //   v2 = v3 → a2 = a3 - (1-γ)·(b2-b1)
                let a1 = a + *gamma * diff;
                let a2 = a - (1.0 - *gamma) * diff;
                left.set_incident(a1);
                right.set_incident(a2);
            }
            Self::Transformer {
                secondary,
                turns_ratio,
                ..
            } => {
                // Scale incident wave down by turns ratio and propagate to secondary
                let a_sec = a / *turns_ratio;
                secondary.set_incident(a_sec);
            }
            Self::RType { adaptor, children } => {
                // Scatter down: parent incident wave → child incident waves
                let a_children = adaptor.scatter_down(a);
                for (child, &a_i) in children.iter_mut().zip(a_children.iter()) {
                    child.set_incident(a_i);
                }
            }
        }
    }

    /// Set the voltage source value (searches recursively).
    /// Note: CathodeBiasSource is intentionally excluded — it maintains a
    /// fixed DC level and must not be overwritten by the plate supply sweep.
    pub fn set_voltage(&mut self, v: f64) {
        match self {
            Self::VoltageSource { voltage, .. } => *voltage = v,
            Self::CathodeBiasSource { .. } => { /* fixed DC — do not update */ }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.set_voltage(v);
                right.set_voltage(v);
            }
            Self::Transformer { secondary, .. } => {
                secondary.set_voltage(v);
            }
            Self::RType { children, .. } => {
                for child in children {
                    child.set_voltage(v);
                }
            }
            _ => {}
        }
    }

    /// Update a pot's position and resistance.  Returns true if found.
    /// The taper curve is applied to convert linear knob position to resistance.
    pub fn set_pot(&mut self, target_id: &str, pos: f64) -> bool {
        match self {
            Self::Pot {
                comp_id,
                max_resistance,
                position,
                taper,
                rp,
                ..
            } if comp_id == target_id
                || comp_id.starts_with(target_id)
                    && comp_id[target_id.len()..].starts_with("__") =>
            {
                *position = pos;
                // Apply taper curve to convert linear position to resistance ratio
                let tapered_pos = taper.apply(pos);
                *rp = (tapered_pos * *max_resistance).max(1.0);
                true
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                // Use | not || to avoid short-circuit: split pots (e.g. Level__aw + Level__wb)
                // need BOTH halves updated.
                let a = left.set_pot(target_id, pos);
                let b = right.set_pot(target_id, pos);
                a | b
            }
            Self::Transformer { secondary, .. } => secondary.set_pot(target_id, pos),
            Self::RType { children, .. } => {
                let mut found = false;
                for c in children.iter_mut() {
                    found |= c.set_pot(target_id, pos);
                }
                found
            }
            _ => false,
        }
    }

    /// Update a resistor's resistance by component ID. Returns true if found.
    pub fn set_resistor(&mut self, target_id: &str, new_ohms: f64) -> bool {
        match self {
            Self::Resistor { comp_id: Some(id), rp, .. } if id == target_id => {
                *rp = new_ohms;
                true
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                let a = left.set_resistor(target_id, new_ohms);
                let b = right.set_resistor(target_id, new_ohms);
                a | b
            }
            Self::Transformer { secondary, .. } => secondary.set_resistor(target_id, new_ohms),
            Self::RType { children, .. } => {
                let mut found = false;
                for c in children.iter_mut() {
                    found |= c.set_resistor(target_id, new_ohms);
                }
                found
            }
            _ => false,
        }
    }

    /// Update a capacitor's value and port resistance by component ID.
    /// Recalculates rp = 1/(2*fs*C) using the existing rp to derive fs.
    /// Returns true if found.
    pub fn set_capacitor(&mut self, target_id: &str, new_farads: f64, sample_rate: f64) -> bool {
        match self {
            Self::Capacitor { comp_id: Some(id), capacitance, rp, .. } if id == target_id => {
                *capacitance = new_farads;
                *rp = 1.0 / (2.0 * sample_rate * new_farads);
                true
            }
            Self::LeakyCapacitor { comp_id: Some(id), capacitance, rp, .. } if id == target_id => {
                *capacitance = new_farads;
                *rp = 1.0 / (2.0 * sample_rate * new_farads);
                true
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                let a = left.set_capacitor(target_id, new_farads, sample_rate);
                let b = right.set_capacitor(target_id, new_farads, sample_rate);
                a | b
            }
            Self::Transformer { secondary, .. } => secondary.set_capacitor(target_id, new_farads, sample_rate),
            Self::RType { children, .. } => {
                let mut found = false;
                for c in children.iter_mut() {
                    found |= c.set_capacitor(target_id, new_farads, sample_rate);
                }
                found
            }
            _ => false,
        }
    }

    /// Update an inductor's value and port resistance by component ID.
    /// Recalculates rp = 2*fs*L.
    /// Returns true if found.
    pub fn set_inductor(&mut self, target_id: &str, new_henries: f64, sample_rate: f64) -> bool {
        match self {
            Self::Inductor { comp_id: Some(id), inductance, rp, .. } if id == target_id => {
                *inductance = new_henries;
                *rp = 2.0 * sample_rate * new_henries;
                true
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                let a = left.set_inductor(target_id, new_henries, sample_rate);
                let b = right.set_inductor(target_id, new_henries, sample_rate);
                a | b
            }
            Self::Transformer { secondary, .. } => secondary.set_inductor(target_id, new_henries, sample_rate),
            Self::RType { children, .. } => {
                let mut found = false;
                for c in children.iter_mut() {
                    found |= c.set_inductor(target_id, new_henries, sample_rate);
                }
                found
            }
            _ => false,
        }
    }

    /// List all editable passive leaves in the tree.
    /// Returns (comp_id, kind, current_value) for each R/C/L with a comp_id.
    pub fn list_editable_leaves(&self) -> Vec<(String, &'static str, f64)> {
        let mut result = Vec::new();
        self.collect_editable_leaves(&mut result);
        result
    }

    fn collect_editable_leaves(&self, out: &mut Vec<(String, &'static str, f64)>) {
        match self {
            Self::Resistor { comp_id: Some(id), rp, .. } => {
                out.push((id.clone(), "resistor", *rp));
            }
            Self::Capacitor { comp_id: Some(id), capacitance, .. } => {
                out.push((id.clone(), "capacitor", *capacitance));
            }
            Self::LeakyCapacitor { comp_id: Some(id), capacitance, .. } => {
                out.push((id.clone(), "capacitor", *capacitance));
            }
            Self::Inductor { comp_id: Some(id), inductance, .. } => {
                out.push((id.clone(), "inductor", *inductance));
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.collect_editable_leaves(out);
                right.collect_editable_leaves(out);
            }
            Self::Transformer { secondary, .. } => {
                secondary.collect_editable_leaves(out);
            }
            Self::RType { children, .. } => {
                for c in children.iter() {
                    c.collect_editable_leaves(out);
                }
            }
            _ => {}
        }
    }

    /// Read a pot's current resistance by component ID.
    ///
    /// Recursively searches through Series/Parallel/Transformer/RType children.
    /// Returns `None` if the pot is not found in this subtree.
    pub fn get_pot_resistance(&self, target_id: &str) -> Option<f64> {
        match self {
            Self::Pot { comp_id, rp, .. }
                if comp_id == target_id
                    || comp_id.starts_with(target_id)
                        && comp_id[target_id.len()..].starts_with("__") =>
            {
                Some(*rp)
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.get_pot_resistance(target_id)
                    .or_else(|| right.get_pot_resistance(target_id))
            }
            Self::Transformer { secondary, .. } => secondary.get_pot_resistance(target_id),
            Self::RType { children, .. } => {
                children.iter().find_map(|c| c.get_pot_resistance(target_id))
            }
            _ => None,
        }
    }

    /// Read a pot's current position (0.0..1.0) by component ID.
    ///
    /// Recursively searches through Series/Parallel/Transformer/RType children.
    /// Returns `None` if the pot is not found in this subtree.
    pub fn get_pot_position(&self, target_id: &str) -> Option<f64> {
        match self {
            Self::Pot { comp_id, position, .. }
                if comp_id == target_id
                    || comp_id.starts_with(target_id)
                        && comp_id[target_id.len()..].starts_with("__") =>
            {
                Some(*position)
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.get_pot_position(target_id)
                    .or_else(|| right.get_pot_position(target_id))
            }
            Self::Transformer { secondary, .. } => secondary.get_pot_position(target_id),
            Self::RType { children, .. } => {
                children.iter().find_map(|c| c.get_pot_position(target_id))
            }
            _ => None,
        }
    }

    /// Extract voltage at a named leaf after the down-sweep.
    ///
    /// For a resistive leaf (Resistor, Pot), b=0, so V = last_a / 2.
    /// For a capacitor, V = (last_a + last_b) / 2.
    /// Returns None if the component is not found in this subtree.
    pub fn leaf_voltage(&self, target_id: &str) -> Option<f64> {
        match self {
            Self::Resistor { comp_id: Some(id), last_a, .. } if id == target_id => {
                Some(*last_a / 2.0)
            }
            Self::Pot { comp_id, last_a, .. }
                if comp_id == target_id
                    || comp_id.starts_with(target_id)
                        && comp_id[target_id.len()..].starts_with("__") =>
            {
                Some(*last_a / 2.0)
            }
            Self::Capacitor { comp_id: Some(id), last_b, state, .. } if id == target_id => {
                Some((*state + *last_b) / 2.0)
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.leaf_voltage(target_id)
                    .or_else(|| right.leaf_voltage(target_id))
            }
            Self::Transformer { secondary, .. } => secondary.leaf_voltage(target_id),
            Self::RType { children, .. } => {
                children.iter().find_map(|c| c.leaf_voltage(target_id))
            }
            _ => None,
        }
    }

    /// Return this leaf's component ID, if it is a leaf with a comp_id.
    /// For adaptors, returns None (they don't have a single component).
    pub fn leaf_comp_id(&self) -> Option<String> {
        match self {
            Self::Pot { comp_id, .. }
            | Self::Photocoupler { comp_id, .. }
            | Self::JfetVr { comp_id, .. }
            | Self::SwitchedResistor { switch_id: comp_id, .. } => Some(comp_id.clone()),
            Self::Resistor { comp_id: Some(id), .. }
            | Self::Capacitor { comp_id: Some(id), .. }
            | Self::LeakyCapacitor { comp_id: Some(id), .. }
            | Self::Inductor { comp_id: Some(id), .. } => Some(id.clone()),
            _ => None,
        }
    }

    /// Tag the deepest Resistor leaf as a Pot for output probe identification.
    ///
    /// Converts a plain Resistor (no comp_id) into a Pot with the given tag.
    /// The Pot has position=1.0 and the same rp, so it's electrically equivalent.
    /// Follows right branches of Series/Parallel to find the ground-side leaf.
    pub fn tag_as_probe(&mut self, tag: &str) -> bool {
        match self {
            Self::Resistor { rp, .. } => {
                *self = Self::Pot {
                    comp_id: tag.to_string(),
                    max_resistance: *rp,
                    position: 1.0,
                    taper: PotTaper::B,
                    rp: *rp,
                    last_a: 0.0,
                };
                true
            }
            Self::Series { right, .. } | Self::Parallel { right, .. } => {
                right.tag_as_probe(tag)
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
            Self::Transformer {
                secondary,
                turns_ratio,
                rp,
                ..
            } => {
                secondary.recompute();
                let r_sec = secondary.port_resistance();
                *rp = *turns_ratio * *turns_ratio * r_sec;
            }
            Self::RType { adaptor, children } => {
                for child in children.iter_mut() {
                    child.recompute();
                }
                // R-type scattering matrix depends on port resistances; if children
                // changed (pot adjustment), the matrix would need recomputation.
                // For now, the scattering matrix is fixed at compile time.
                // Update the parent port resistance to match the adaptor.
                let _ = adaptor; // port_resistance is immutable for R-type
            }
            _ => {}
        }
    }

    /// Update sample rate for all reactive elements (capacitors/inductors).
    ///
    /// Bilinear-transform port resistance depends on sample rate:
    ///   - Capacitor: Rp = 1 / (2 * fs * C)
    ///   - Inductor:  Rp = 2 * fs * L
    ///
    /// When oversampling is active, the WDF cycle runs at `fs * ratio`,
    /// so reactive elements must use the oversampled rate for correct
    /// frequency response. Call this with `base_rate * oversampling_ratio`,
    /// then call `recompute()` to propagate updated Rp through adaptors.
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        match self {
            Self::Capacitor {
                capacitance, rp, ..
            } => {
                *rp = 1.0 / (2.0 * sample_rate * *capacitance);
            }
            Self::LeakyCapacitor {
                capacitance, rp, ..
            } => {
                *rp = 1.0 / (2.0 * sample_rate * *capacitance);
            }
            Self::Inductor {
                inductance, rp, ..
            } => {
                *rp = 2.0 * sample_rate * *inductance;
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.set_sample_rate(sample_rate);
                right.set_sample_rate(sample_rate);
            }
            Self::Transformer { secondary, .. } => {
                secondary.set_sample_rate(sample_rate);
            }
            Self::RType { children, .. } => {
                for child in children.iter_mut() {
                    child.set_sample_rate(sample_rate);
                }
            }
            _ => {} // Resistors, pots, voltage sources: no fs dependence
        }
    }

    /// Reset all state (capacitor/inductor memory + adaptor caches).
    pub fn reset(&mut self) {
        match self {
            Self::Capacitor { state, last_b, .. } => {
                *state = 0.0;
                *last_b = 0.0;
            }
            Self::Inductor { state, .. } => *state = 0.0,
            Self::UnitDelay {
                state,
                partner_state,
                ..
            } => {
                *state = 0.0;
                *partner_state = 0.0;
            }
            Self::LeakyCapacitor {
                state, da_state, ..
            } => {
                *state = 0.0;
                *da_state = 0.0;
            }
            Self::Photocoupler { inner, .. } => inner.reset(),
            Self::JfetVr { inner, .. } => inner.reset(),
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
            Self::Transformer {
                secondary, b_sec, ..
            } => {
                *b_sec = 0.0;
                secondary.reset();
            }
            Self::RType { adaptor, children } => {
                adaptor.reset();
                for child in children {
                    child.reset();
                }
            }
            _ => {}
        }
    }

    /// Extract voltage from the first reactive element (capacitor/inductor) in the tree.
    ///
    /// For passive filter stages (RC lowpass, RL highpass, etc.), the output voltage
    /// should be taken from the reactive element, not the root port. This method
    /// finds the first capacitor or inductor and returns its voltage using:
    ///   V = (a + b) / 2
    /// where `a` is the last incident wave and `b` is the last reflected wave.
    ///
    /// Returns `None` if no reactive element exists in the tree.
    #[allow(dead_code)]
    pub fn reactive_voltage(&self) -> Option<f64> {
        match self {
            Self::Capacitor { state, last_b, .. } => {
                // state = last incident wave (a), last_b = last reflected wave (b)
                // Voltage = (a + b) / 2
                Some((*state + *last_b) / 2.0)
            }
            Self::LeakyCapacitor { state, .. } => {
                // For leaky capacitor, state is the incident wave; reflected = state
                // This is approximate; a more precise implementation would track last_b
                Some(*state)
            }
            Self::Inductor { state, .. } => {
                // For inductor: b = -state, so V = (a + (-state)) / 2 = (state - state) / 2
                // This is also approximate; ideally we'd track last incident wave
                // For now, inductors are rare in passive filters we're testing
                Some(*state / 2.0)
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                // Search children for first reactive element
                left.reactive_voltage().or_else(|| right.reactive_voltage())
            }
            Self::Transformer { secondary, .. } => secondary.reactive_voltage(),
            Self::RType { children, .. } => children.iter().find_map(|c| c.reactive_voltage()),
            _ => None,
        }
    }

    /// Compute output voltage for passive WDF stages using resistive termination.
    ///
    /// For passive filters with embedded voltage sources (Passthrough root), the
    /// standard open-circuit extraction V = (a + b) / 2 with a = b gives wrong results
    /// because it doesn't properly account for the voltage source's wave contribution.
    ///
    /// Instead, we use resistive termination (a = 0) at the root port, which models
    /// the output being taken across a matched load. This gives:
    ///   V_out = b_tree / 2
    ///
    /// Returns None if this optimization doesn't apply.
    pub fn resistive_termination_voltage(&self, b_tree: f64) -> Option<f64> {
        match self {
            // For parallel adaptor with resistor (load to ground) as one child,
            // the output voltage is b_tree / 2 (resistive termination)
            Self::Parallel { left, .. } => match left.as_ref() {
                Self::Resistor { .. } | Self::Pot { .. } => Some(b_tree / 2.0),
                _ => None,
            },
            // For series adaptor with resistor as one child
            Self::Series { left, .. } => match left.as_ref() {
                Self::Resistor { .. } | Self::Pot { .. } => Some(b_tree / 2.0),
                _ => None,
            },
            _ => None,
        }
    }

    /// Compute junction voltage for series filters (RL lowpass, LC filters, etc.).
    ///
    /// For a Series(L, R) tree driven by a voltage source at the root:
    /// - a_root = 2 * V_in (incident wave from VS)
    /// - b_tree = -(b1 + b2) (reflected wave from series adaptor)
    /// - The junction voltage (between L and R) is the voltage across R
    ///
    /// After scattering:
    /// - a2 = b2 - (1 - γ) * (b1 + b2 + a_root)
    /// - V_junction = (a2 + b2) / 2
    ///
    /// This is needed because (a_root + b_tree) / 2 gives the voltage at the root
    /// (input node), not at the internal junction (output node).
    ///
    /// Returns None if this is not a series filter topology.
    pub fn series_junction_voltage(&self, a_root: f64) -> Option<f64> {
        match self {
            Self::Series {
                gamma,
                b1,
                b2,
                right,
                ..
            } => {
                // Check if right child is a simple element (R, C, or L)
                // The junction voltage is the voltage at the right child's port
                match right.as_ref() {
                    Self::Resistor { .. }
                    | Self::Capacitor { .. }
                    | Self::Inductor { .. }
                    | Self::Pot { .. } => {
                        // Compute a2 (incident wave to right child) using series scattering
                        let sum = *b1 + *b2 + a_root;
                        let a2 = *b2 - (1.0 - *gamma) * sum;
                        // Junction voltage = voltage at right port
                        Some((a2 + *b2) / 2.0)
                    }
                    // Nested series: recurse into right child
                    Self::Series { .. } => right.series_junction_voltage(a_root),
                    _ => None,
                }
            }
            _ => None,
        }
    }

    /// Extract junction voltage for short-circuit terminated passive filters.
    ///
    /// For a tree like `Series(VS, Series(L, R))` with short-circuit at ground:
    /// 1. VS is embedded in tree, emits b_vs = 2 * Vin
    /// 2. Series(L, R) forms the filter network
    /// 3. Short-circuit root: a_root = -b_tree
    /// 4. Output is at junction between L and R (voltage across R)
    ///
    /// This method scatters down through the tree to compute the voltage
    /// at the load resistor R.
    pub fn short_circuit_junction_voltage(&self, a_root: f64) -> Option<f64> {
        match self {
            Self::Series {
                gamma,
                b1,
                b2,
                left,
                right,
                ..
            } => {
                // Check if this is Series(VS, inner) where inner contains the filter
                match (left.as_ref(), right.as_ref()) {
                    // VS on left, filter network on right
                    (Self::VoltageSource { .. }, right_node) => {
                        // Scatter: a_inner = b2 - (1 - γ) * (b1 + b2 + a_root)
                        let sum = *b1 + *b2 + a_root;
                        let a_inner = *b2 - (1.0 - *gamma) * sum;
                        // Recursively extract junction voltage from inner tree
                        right_node.extract_load_voltage(a_inner)
                    }
                    // Filter network on left, something else on right - try left
                    (left_node, Self::VoltageSource { .. }) => {
                        let sum = *b1 + *b2 + a_root;
                        let a_left = *b1 - *gamma * sum;
                        left_node.extract_load_voltage(a_left)
                    }
                    // Not Series(VS, _), try to find load directly
                    _ => {
                        // If right child is a load element, extract its voltage
                        if right.is_load_element() {
                            let sum = *b1 + *b2 + a_root;
                            let a2 = *b2 - (1.0 - *gamma) * sum;
                            Some((a2 + *b2) / 2.0)
                        } else if left.is_load_element() {
                            let sum = *b1 + *b2 + a_root;
                            let a1 = *b1 - *gamma * sum;
                            Some((a1 + *b1) / 2.0)
                        } else {
                            None
                        }
                    }
                }
            }
            // Direct resistor - voltage is (a + b) / 2
            Self::Resistor { .. } | Self::Pot { .. } => {
                // For resistor: b = 0, so V = a / 2
                Some(a_root / 2.0)
            }
            _ => None,
        }
    }

    /// Extract voltage at the load element (resistor) in a filter subtree.
    ///
    /// Used by short_circuit_junction_voltage to recursively find the load.
    fn extract_load_voltage(&self, a_parent: f64) -> Option<f64> {
        match self {
            // Direct load element - voltage = (a + b) / 2
            // For resistor: b = 0, so V = a / 2
            Self::Resistor { .. } | Self::Pot { .. } => Some(a_parent / 2.0),
            // Series adaptor - find the load resistor.
            //
            // For simple cases like Series(L, R) or Series(C, R), the direct
            // load check suffices. For nested chains like
            // Series(R1, Series(L, Series(C, RL))), we need to prefer the
            // deeper load RL over the immediate R1. The key case is when left
            // is a direct load but right is a subtree that may contain a
            // deeper (ground-side) load.
            Self::Series {
                gamma,
                b1,
                b2,
                left,
                right,
                ..
            } => {
                let sum = *b1 + *b2 + a_parent;
                // 1. Right child is a direct load element (e.g., Series(L, R))
                if right.is_load_element() {
                    let a2 = *b2 - (1.0 - *gamma) * sum;
                    Some((a2 + *b2) / 2.0)
                }
                // 2. Left is a load, but right is a subtree that may contain
                //    a deeper load. Prefer the deeper one (ground-side).
                //    Example: Series(R1, Series(L, Series(C, RL))) — pick RL
                else if left.is_load_element()
                    && matches!(right.as_ref(), Self::Series { .. } | Self::Parallel { .. })
                {
                    let a2 = *b2 - (1.0 - *gamma) * sum;
                    right.extract_load_voltage(a2).or_else(|| {
                        // Fallback: no load in right subtree, use left
                        let a1 = *b1 - *gamma * sum;
                        Some((a1 + *b1) / 2.0)
                    })
                }
                // 3. Left is a direct load (right is not a load or subtree)
                else if left.is_load_element() {
                    let a1 = *b1 - *gamma * sum;
                    Some((a1 + *b1) / 2.0)
                }
                // 4. Neither is a direct load — recurse into subtrees
                else {
                    if matches!(right.as_ref(), Self::Series { .. } | Self::Parallel { .. }) {
                        let a2 = *b2 - (1.0 - *gamma) * sum;
                        right.extract_load_voltage(a2)
                    } else if matches!(left.as_ref(), Self::Series { .. } | Self::Parallel { .. })
                    {
                        let a1 = *b1 - *gamma * sum;
                        left.extract_load_voltage(a1)
                    } else {
                        None
                    }
                }
            }
            // Parallel - try to find load in children
            Self::Parallel {
                gamma,
                b1,
                b2,
                left,
                right,
                ..
            } => {
                if right.is_load_element() {
                    let sum = *b1 + *b2 + a_parent;
                    let a2 = *b2 - (1.0 - *gamma) * sum;
                    Some((a2 + *b2) / 2.0)
                } else if left.is_load_element() {
                    let sum = *b1 + *b2 + a_parent;
                    let a1 = *b1 - *gamma * sum;
                    Some((a1 + *b1) / 2.0)
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    /// Check if this node is a load element (resistor or potentiometer).
    fn is_load_element(&self) -> bool {
        matches!(self, Self::Resistor { .. } | Self::Pot { .. })
    }

    /// Check if this tree contains any reactive elements (capacitors/inductors).
    #[allow(dead_code)]
    pub fn has_reactive_elements(&self) -> bool {
        match self {
            Self::Capacitor { .. } | Self::LeakyCapacitor { .. } | Self::Inductor { .. } => true,
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.has_reactive_elements() || right.has_reactive_elements()
            }
            Self::Transformer { secondary, .. } => secondary.has_reactive_elements(),
            Self::RType { children, .. } => children.iter().any(|c| c.has_reactive_elements()),
            _ => false,
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
            Self::JfetVr { inner, .. } => {
                inner.set_sample_rate(fs);
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.update_sample_rate(fs);
                right.update_sample_rate(fs);
            }
            Self::Transformer {
                secondary,
                turns_ratio,
                rp,
                ..
            } => {
                secondary.update_sample_rate(fs);
                // Recompute port resistance after secondary updates
                let r_sec = secondary.port_resistance();
                *rp = *turns_ratio * *turns_ratio * r_sec;
            }
            Self::RType { children, .. } => {
                for child in children {
                    child.update_sample_rate(fs);
                }
                // R-type scattering matrix is fixed at compile time.
                // Port resistance depends on children, but the R-type adaptor
                // computes it during construction. For sample rate changes,
                // the matrix would need recomputation (not yet supported).
            }
            _ => {}
        }
    }

    /// Set LED drive level for a photocoupler. Returns true if found.
    pub fn set_photocoupler_led(&mut self, target_id: &str, led_drive: f64) -> bool {
        match self {
            Self::Photocoupler { comp_id, inner, .. } if comp_id == target_id => {
                inner.set_led_drive(led_drive);
                true
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.set_photocoupler_led(target_id, led_drive)
                    || right.set_photocoupler_led(target_id, led_drive)
            }
            Self::RType { children, .. } => children
                .iter_mut()
                .any(|c| c.set_photocoupler_led(target_id, led_drive)),
            _ => false,
        }
    }

    /// Set Vgs for a JFET variable resistor leaf. Returns true if found.
    pub fn set_jfet_vr_vgs(&mut self, target_id: &str, vgs: f64) -> bool {
        match self {
            Self::JfetVr { comp_id, inner, .. } if comp_id == target_id => {
                inner.set_vgs(vgs);
                true
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.set_jfet_vr_vgs(target_id, vgs)
                    || right.set_jfet_vr_vgs(target_id, vgs)
            }
            Self::RType { children, .. } => children
                .iter_mut()
                .any(|c| c.set_jfet_vr_vgs(target_id, vgs)),
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
            Self::Transformer { secondary, .. } => {
                secondary.set_switch_position(target_switch, new_position)
            }
            Self::RType { children, .. } => children
                .iter_mut()
                .map(|c| c.set_switch_position(target_switch, new_position))
                .sum(),
            _ => 0,
        }
    }

    /// Debug dump: print tree structure with port resistances and coefficients.
    /// Returns a multi-line string showing the WDF tree hierarchy.
    pub fn debug_dump(&self, indent: usize) -> String {
        let pad = "  ".repeat(indent);
        match self {
            Self::Resistor { comp_id, rp, .. } => {
                let id_str = comp_id.as_deref().unwrap_or("?");
                format!("{pad}Resistor(id=\"{id_str}\", Rp={rp:.1}Ω)")
            }
            Self::Capacitor {
                comp_id,
                capacitance,
                rp,
                state,
                ..
            } => {
                let id_str = comp_id.as_deref().unwrap_or("?");
                format!("{pad}Capacitor(id=\"{id_str}\", C={capacitance:.3e}F, Rp={rp:.1}Ω, state={state:.6})")
            }
            Self::LeakyCapacitor {
                comp_id,
                capacitance,
                rp,
                state,
                leakage_decay,
                ..
            } => {
                let id_str = comp_id.as_deref().unwrap_or("?");
                format!("{pad}LeakyCapacitor(id=\"{id_str}\", C={capacitance:.3e}F, Rp={rp:.1}Ω, state={state:.6}, decay={leakage_decay:.6})")
            }
            Self::Inductor {
                comp_id,
                inductance,
                rp,
                state,
                ..
            } => {
                let id_str = comp_id.as_deref().unwrap_or("?");
                format!("{pad}Inductor(id=\"{id_str}\", L={inductance:.3e}H, Rp={rp:.1}Ω, state={state:.6})")
            }
            Self::VoltageSource { voltage, rp } => {
                format!("{pad}VoltageSource(V={voltage:.3}V, Rp={rp:.1}Ω)")
            }
            Self::CathodeBiasSource { voltage, rp } => {
                format!("{pad}CathodeBiasSource(V={voltage:.3}V, Rp={rp:.1}Ω)")
            }
            Self::UnitDelay {
                rp,
                state,
                partner_state,
            } => {
                format!("{pad}UnitDelay(Rp={rp:.1}Ω, state={state:.6}, partner={partner_state:.6})")
            }
            Self::Pot {
                comp_id,
                max_resistance,
                position,
                taper,
                rp,
                ..
            } => {
                format!("{pad}Pot(id=\"{comp_id}\", max={max_resistance:.1}Ω, pos={position:.3}, taper={taper:?}, Rp={rp:.1}Ω)")
            }
            Self::Photocoupler { comp_id, inner, .. } => {
                format!(
                    "{pad}Photocoupler(id=\"{comp_id}\", Rp={:.1}Ω)",
                    inner.port_resistance()
                )
            }
            Self::JfetVr { comp_id, inner, .. } => {
                format!(
                    "{pad}JfetVr(id=\"{comp_id}\", Vgs={:.3}V, Rds={:.1}Ω)",
                    inner.vgs(),
                    inner.rds()
                )
            }
            Self::SwitchedResistor {
                switch_id,
                path_index,
                position,
                rp,
                ..
            } => {
                format!("{pad}SwitchedResistor(switch=\"{switch_id}\", path={path_index}, pos={position}, Rp={rp:.1}Ω)")
            }
            Self::Series {
                left,
                right,
                rp,
                gamma,
                b1,
                b2,
            } => {
                let mut s =
                    format!("{pad}Series(Rp={rp:.1}Ω, γ={gamma:.6}, b1={b1:.6}, b2={b2:.6})\n");
                s.push_str(&left.debug_dump(indent + 1));
                s.push('\n');
                s.push_str(&right.debug_dump(indent + 1));
                s
            }
            Self::Parallel {
                left,
                right,
                rp,
                gamma,
                b1,
                b2,
            } => {
                let mut s =
                    format!("{pad}Parallel(Rp={rp:.1}Ω, γ={gamma:.6}, b1={b1:.6}, b2={b2:.6})\n");
                s.push_str(&left.debug_dump(indent + 1));
                s.push('\n');
                s.push_str(&right.debug_dump(indent + 1));
                s
            }
            Self::Transformer {
                secondary,
                turns_ratio,
                rp,
                ..
            } => {
                let mut s = format!("{pad}Transformer(n={turns_ratio:.3}, Rp={rp:.1}Ω)\n");
                s.push_str(&secondary.debug_dump(indent + 1));
                s
            }
            Self::RType { adaptor, children } => {
                let mut s = format!(
                    "{pad}RType(ports={}, Rp={:.1}Ω)\n",
                    adaptor.num_ports, adaptor.port_resistance
                );
                for (i, child) in children.iter().enumerate() {
                    s.push_str(&format!("{pad}  [port {i}]:\n"));
                    s.push_str(&child.debug_dump(indent + 2));
                    s.push('\n');
                }
                s
            }
        }
    }

    /// Get the UnitDelay's outgoing state (for cross-coupled cathode exchange).
    /// Searches recursively; returns 0.0 if no UnitDelay exists.
    pub fn get_unit_delay_state(&self) -> f64 {
        match self {
            Self::UnitDelay { state, .. } => *state,
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                let l = left.get_unit_delay_state();
                if l != 0.0 {
                    l
                } else {
                    right.get_unit_delay_state()
                }
            }
            Self::Transformer { secondary, .. } => secondary.get_unit_delay_state(),
            Self::RType { children, .. } => children
                .iter()
                .find_map(|c| {
                    let v = c.get_unit_delay_state();
                    if v != 0.0 {
                        Some(v)
                    } else {
                        None
                    }
                })
                .unwrap_or(0.0),
            _ => 0.0,
        }
    }

    /// Set the UnitDelay's incoming partner state (for cross-coupled cathode exchange).
    /// Searches recursively; returns true if a UnitDelay was found and updated.
    pub fn set_unit_delay_partner(&mut self, val: f64) -> bool {
        match self {
            Self::UnitDelay { partner_state, .. } => {
                *partner_state = val;
                true
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.set_unit_delay_partner(val) || right.set_unit_delay_partner(val)
            }
            Self::Transformer { secondary, .. } => secondary.set_unit_delay_partner(val),
            Self::RType { children, .. } => {
                children.iter_mut().any(|c| c.set_unit_delay_partner(val))
            }
            _ => false,
        }
    }

    /// Check hybrid linear/nonlinear devices for operating region violations.
    ///
    /// Recurses into Series/Parallel/RType/Transformer children, checking
    /// JfetVr and Photocoupler nodes. Updates prev_rds/prev_resistance for
    /// slew rate tracking.
    #[cfg(feature = "runtime-warnings")]
    pub fn check_hybrid_warnings(&mut self, v_port: f64, sample_index: u64) {
        match self {
            Self::JfetVr {
                comp_id,
                inner,
                prev_rds,
            } => {
                let prev = if *prev_rds > 0.0 {
                    Some(*prev_rds)
                } else {
                    None
                };
                inner.check_operating_region(v_port, prev, comp_id, sample_index);
                *prev_rds = inner.rds();
            }
            Self::Photocoupler {
                comp_id,
                inner,
                prev_resistance,
            } => {
                let prev = if *prev_resistance > 0.0 {
                    Some(*prev_resistance)
                } else {
                    None
                };
                inner.check_operating_region(prev, comp_id, sample_index);
                *prev_resistance = inner.port_resistance();
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.check_hybrid_warnings(v_port, sample_index);
                right.check_hybrid_warnings(v_port, sample_index);
            }
            Self::Transformer { secondary, .. } => {
                secondary.check_hybrid_warnings(v_port, sample_index);
            }
            Self::RType { children, .. } => {
                for child in children.iter_mut() {
                    child.check_hybrid_warnings(v_port, sample_index);
                }
            }
            _ => {}
        }
    }

    /// Count total nodes in tree (for statistics).
    pub fn node_count(&self) -> usize {
        match self {
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                1 + left.node_count() + right.node_count()
            }
            Self::Transformer { secondary, .. } => 1 + secondary.node_count(),
            Self::RType { children, .. } => {
                1 + children.iter().map(|c| c.node_count()).sum::<usize>()
            }
            _ => 1,
        }
    }
}
