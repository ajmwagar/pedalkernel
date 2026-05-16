//! Dynamic WDF tree node for runtime-constructed circuits.
//!
//! DynNode has 4 structural variants:
//! - `Leaf` — terminal one-port element (Box<dyn WdfLeaf>)
//! - `Binary` — series or parallel adaptor
//! - `Transformer` — ideal transformer adaptor
//! - `RType` — N-port R-type adaptor (MNA-derived scattering)

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::cell::Cell;

use crate::elements::{JfetVariableResistor, Photocoupler};
use crate::pot_taper::PotTaper;
use crate::tree::RTypeAdaptor;
use crate::wdf_leaf::{
    leaf_matches_id, LeafKind, WdfCapacitor, WdfInductor, WdfJfetVr, WdfLeaf, WdfLeakyCapacitor,
    WdfPhotocoupler, WdfPot, WdfResistor, WdfSwitchedResistor, WdfUnitDelay, WdfVoltageSource,
};

// ═══════════════════════════════════════════════════════════════════════════
// Dynamic WDF tree node
// ═══════════════════════════════════════════════════════════════════════════

/// Binary adaptor kind.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum BinaryKind {
    Series,
    Parallel,
}

/// A node in a dynamically-constructed WDF binary tree.
///
/// Leaves are one-port elements (trait objects); internal nodes are
/// series/parallel/transformer/R-type adaptors.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum DynNode {
    /// Terminal one-port element. Concrete enum for serialization support.
    Leaf(LeafKind),

    /// Binary adaptor (Series or Parallel).
    Binary {
        kind: BinaryKind,
        left: Box<DynNode>,
        right: Box<DynNode>,
        rp: f64,
        gamma: f64,
        b1: f64,
        b2: f64,
        /// True when a descendant leaf has changed and adaptor coefficients need recompute.
        dirty: bool,
        /// True when any descendant leaf is dynamic (pot, photocoupler, etc.).
        has_dynamic: bool,
    },

    /// Ideal transformer adaptor.
    Transformer {
        secondary: Box<DynNode>,
        turns_ratio: f64,
        rp: f64,
        b_sec: f64,
        /// True when a descendant leaf has changed and adaptor coefficients need recompute.
        dirty: bool,
        /// True when any descendant leaf is dynamic (pot, photocoupler, etc.).
        has_dynamic: bool,
    },

    /// N-port R-type adaptor (MNA-derived scattering).
    RType {
        adaptor: RTypeAdaptor,
        children: Vec<Box<DynNode>>,
    },
}

impl Clone for DynNode {
    fn clone(&self) -> Self {
        match self {
            Self::Leaf(leaf) => Self::Leaf(leaf.clone()), // LeafKind derives Clone
            Self::Binary {
                kind,
                left,
                right,
                rp,
                gamma,
                b1,
                b2,
                dirty,
                has_dynamic,
            } => Self::Binary {
                kind: *kind,
                left: left.clone(),
                right: right.clone(),
                rp: *rp,
                gamma: *gamma,
                b1: *b1,
                b2: *b2,
                dirty: *dirty,
                has_dynamic: *has_dynamic,
            },
            Self::Transformer {
                secondary,
                turns_ratio,
                rp,
                b_sec,
                dirty,
                has_dynamic,
            } => Self::Transformer {
                secondary: secondary.clone(),
                turns_ratio: *turns_ratio,
                rp: *rp,
                b_sec: *b_sec,
                dirty: *dirty,
                has_dynamic: *has_dynamic,
            },
            Self::RType { adaptor, children } => Self::RType {
                adaptor: adaptor.clone(),
                children: children.clone(),
            },
        }
    }
}

fn projected_leaf_voltage(leaf: &LeafKind, incident: f64) -> f64 {
    match leaf {
        LeafKind::Resistor(_)
        | LeafKind::Pot(_)
        | LeafKind::Inductor(_)
        | LeafKind::SwitchedResistor(_) => incident / 2.0,
        LeafKind::Capacitor(cap) => (incident + cap.state) / 2.0,
        LeafKind::LeakyCapacitor(cap) => incident * cap.leakage_decay,
        LeafKind::VoltageSource(_) | LeafKind::Photocoupler(_) | LeafKind::JfetVr(_) => 0.0,
        LeafKind::UnitDelay(_) => 0.0,
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Constructors — backward-compatible factory methods
// ═══════════════════════════════════════════════════════════════════════════

#[allow(non_snake_case)]
impl DynNode {
    pub fn Resistor(comp_id: Option<String>, rp: f64) -> Self {
        Self::Leaf(LeafKind::Resistor(WdfResistor {
            comp_id,
            rp,
            last_a: 0.0,
        }))
    }

    pub fn Capacitor(comp_id: Option<String>, capacitance: f64, rp: f64) -> Self {
        Self::Leaf(LeafKind::Capacitor(WdfCapacitor {
            comp_id,
            capacitance,
            rp,
            state: 0.0,
            last_b: 0.0,
        }))
    }

    pub fn LeakyCapacitor(
        comp_id: Option<String>,
        capacitance: f64,
        rp: f64,
        leakage_decay: f64,
        da_coef: Option<f64>,
        da_state: f64,
        da_rate: f64,
    ) -> Self {
        Self::Leaf(LeafKind::LeakyCapacitor(WdfLeakyCapacitor {
            comp_id,
            capacitance,
            rp,
            state: 0.0,
            leakage_decay,
            da_coef,
            da_state,
            da_rate,
        }))
    }

    pub fn Inductor(comp_id: Option<String>, inductance: f64, rp: f64) -> Self {
        Self::Leaf(LeafKind::Inductor(WdfInductor {
            comp_id,
            inductance,
            rp,
            state: 0.0,
        }))
    }

    pub fn VoltageSource(voltage: f64, rp: f64) -> Self {
        Self::Leaf(LeafKind::VoltageSource(WdfVoltageSource {
            voltage,
            rp,
            is_cathode_bias: false,
            port_name: None,
        }))
    }

    pub fn CathodeBiasSource(voltage: f64, rp: f64) -> Self {
        Self::Leaf(LeafKind::VoltageSource(WdfVoltageSource {
            voltage,
            rp,
            is_cathode_bias: true,
            port_name: None,
        }))
    }

    pub fn UnitDelay(rp: f64) -> Self {
        Self::Leaf(LeafKind::UnitDelay(WdfUnitDelay {
            rp,
            state: 0.0,
            partner_state: 0.0,
        }))
    }

    pub fn Pot(comp_id: String, max_resistance: f64, position: f64, taper: PotTaper) -> Self {
        let tapered_pos = taper.apply(position);
        let rp = (tapered_pos * max_resistance).max(max_resistance * 0.001);
        Self::Leaf(LeafKind::Pot(WdfPot {
            comp_id,
            max_resistance,
            position,
            taper,
            rp,
            last_a: 0.0,
            complement: false,
        }))
    }

    pub fn PhotocouplerNode(comp_id: String, inner: Photocoupler) -> Self {
        Self::Leaf(LeafKind::Photocoupler(WdfPhotocoupler {
            comp_id,
            inner,
            prev_resistance: 0.0,
        }))
    }

    pub fn JfetVrNode(comp_id: String, inner: JfetVariableResistor) -> Self {
        Self::Leaf(LeafKind::JfetVr(WdfJfetVr {
            comp_id,
            inner,
            prev_rds: 0.0,
        }))
    }

    pub fn SwitchedResistor(
        switch_id: String,
        path_index: usize,
        num_paths: usize,
        r_active: f64,
        r_inactive: f64,
    ) -> Self {
        let position = 0;
        let rp = if path_index == position {
            r_active
        } else {
            r_inactive
        };
        Self::Leaf(LeafKind::SwitchedResistor(WdfSwitchedResistor {
            switch_id,
            path_index,
            num_paths,
            r_active,
            r_inactive,
            position,
            rp,
            last_a: 0.0,
        }))
    }

    pub fn Series(left: Box<DynNode>, right: Box<DynNode>) -> Self {
        let r1 = left.port_resistance();
        let r2 = right.port_resistance();
        let rp = r1 + r2;
        let gamma = if rp > 0.0 { r1 / rp } else { 0.5 };
        Self::Binary {
            kind: BinaryKind::Series,
            left,
            right,
            rp,
            gamma,
            b1: 0.0,
            b2: 0.0,
            dirty: true,
            has_dynamic: true,
        }
    }

    pub fn Parallel(left: Box<DynNode>, right: Box<DynNode>) -> Self {
        let r1 = left.port_resistance();
        let r2 = right.port_resistance();
        let sum = r1 + r2;
        let rp = if sum > 0.0 { r1 * r2 / sum } else { 0.0 };
        let gamma = if sum > 0.0 { r2 / sum } else { 0.5 };
        Self::Binary {
            kind: BinaryKind::Parallel,
            left,
            right,
            rp,
            gamma,
            b1: 0.0,
            b2: 0.0,
            dirty: true,
            has_dynamic: true,
        }
    }

    pub fn TransformerNode(secondary: Box<DynNode>, turns_ratio: f64) -> Self {
        let r_sec = secondary.port_resistance();
        let rp = turns_ratio * turns_ratio * r_sec;
        Self::Transformer {
            secondary,
            turns_ratio,
            rp,
            b_sec: 0.0,
            dirty: true,
            has_dynamic: true,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Generic traversal
// ═══════════════════════════════════════════════════════════════════════════

impl DynNode {
    /// Find a leaf by predicate (first match, depth-first). Returns None if not found.
    pub fn find_leaf<T>(&self, f: &impl Fn(&dyn WdfLeaf) -> Option<T>) -> Option<T> {
        match self {
            Self::Leaf(leaf) => f(leaf),
            Self::Binary { left, right, .. } => left.find_leaf(f).or_else(|| right.find_leaf(f)),
            Self::Transformer { secondary, .. } => secondary.find_leaf(f),
            Self::RType { children, .. } => children.iter().find_map(|c| c.find_leaf(f)),
        }
    }

    /// Apply mutation to matching leaves. Uses bitwise OR (not short-circuit)
    /// to handle split pots where both halves must update.
    pub fn for_each_leaf_mut(&mut self, f: &mut impl FnMut(&mut dyn WdfLeaf) -> bool) -> bool {
        match self {
            Self::Leaf(leaf) => f(leaf),
            Self::Binary { left, right, .. } => {
                let a = left.for_each_leaf_mut(f);
                let b = right.for_each_leaf_mut(f);
                a | b
            }
            Self::Transformer { secondary, .. } => secondary.for_each_leaf_mut(f),
            Self::RType { children, .. } => {
                let mut found = false;
                for c in children.iter_mut() {
                    found |= c.for_each_leaf_mut(f);
                }
                found
            }
        }
    }

    /// Iterate all leaves immutably.
    pub fn for_each_leaf(&self, f: &mut impl FnMut(&dyn WdfLeaf)) {
        match self {
            Self::Leaf(leaf) => f(leaf),
            Self::Binary { left, right, .. } => {
                left.for_each_leaf(f);
                right.for_each_leaf(f);
            }
            Self::Transformer { secondary, .. } => secondary.for_each_leaf(f),
            Self::RType { children, .. } => {
                for c in children.iter() {
                    c.for_each_leaf(f);
                }
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Core WDF operations
// ═══════════════════════════════════════════════════════════════════════════

impl DynNode {
    pub fn port_resistance(&self) -> f64 {
        match self {
            Self::Leaf(leaf) => leaf.port_resistance(),
            Self::Binary { rp, .. } | Self::Transformer { rp, .. } => *rp,
            Self::RType { adaptor, .. } => adaptor.port_resistance,
        }
    }

    /// Returns true if the tree's root node is a Series adaptor.
    ///
    /// Used by the compiler to detect when the VS contribution to b_tree is
    /// negated by the Series adaptor formula `b = -(b1 + b2)`, which inverts
    /// the sign of the Thevenin voltage seen by the NL root.  When true,
    /// triode stages must negate VS so that `b_tree = 2*B+` (positive).
    pub fn is_series_root(&self) -> bool {
        matches!(
            self,
            Self::Binary {
                kind: BinaryKind::Series,
                ..
            }
        )
    }

    /// Find a capacitor's voltage by component ID.
    /// Searches the tree recursively via `find_leaf`. Returns None if not found.
    pub fn find_cap_state(&self, target_id: &str) -> Option<f64> {
        self.find_leaf(&|leaf| {
            if leaf.comp_id() == Some(target_id)
                && (leaf.type_tag() == "capacitor" || leaf.type_tag() == "leaky_capacitor")
            {
                Some(leaf.leaf_voltage())
            } else {
                None
            }
        })
    }

    /// Scatter-up: compute reflected wave (bottom → root), caching child waves.
    #[inline]
    pub fn reflected(&mut self) -> f64 {
        match self {
            Self::Leaf(leaf) => leaf.reflected(),
            Self::Binary {
                kind,
                left,
                right,
                gamma,
                b1,
                b2,
                ..
            } => {
                *b1 = left.reflected();
                *b2 = right.reflected();
                match kind {
                    BinaryKind::Series => -(*b1 + *b2),
                    BinaryKind::Parallel => *b1 + *gamma * (*b2 - *b1),
                }
            }
            Self::Transformer {
                secondary,
                turns_ratio,
                b_sec,
                ..
            } => {
                *b_sec = secondary.reflected();
                *turns_ratio * *b_sec
            }
            Self::RType { adaptor, children } => {
                let b_children: Vec<f64> = children.iter_mut().map(|c| c.reflected()).collect();
                adaptor.scatter_up(&b_children)
            }
        }
    }

    /// Scatter-down + state update: propagate incident wave (root → leaves).
    #[inline]
    pub fn set_incident(&mut self, a: f64) {
        match self {
            Self::Leaf(leaf) => leaf.set_incident(a),
            Self::Binary {
                kind,
                left,
                right,
                gamma,
                b1,
                b2,
                ..
            } => match kind {
                BinaryKind::Series => {
                    let sum = *b1 + *b2 + a;
                    let a1 = *b1 - *gamma * sum;
                    let a2 = *b2 - (1.0 - *gamma) * sum;
                    left.set_incident(a1);
                    right.set_incident(a2);
                }
                BinaryKind::Parallel => {
                    let diff = *b2 - *b1;
                    let a1 = a + *gamma * diff;
                    let a2 = a - (1.0 - *gamma) * diff;
                    left.set_incident(a1);
                    right.set_incident(a2);
                }
            },
            Self::Transformer {
                secondary,
                turns_ratio,
                ..
            } => {
                let a_sec = a / *turns_ratio;
                secondary.set_incident(a_sec);
            }
            Self::RType { adaptor, children } => {
                let a_children = adaptor.scatter_down(a);
                for (child, &a_i) in children.iter_mut().zip(a_children.iter()) {
                    child.set_incident(a_i);
                }
            }
        }
    }

    /// Set the main voltage source value.
    ///
    /// Fast path: the VS is always the left child of the root Binary (Series)
    /// node, placed there by `with_voltage_source()`. We access it directly
    /// instead of walking the entire tree via `for_each_leaf_mut`.
    ///
    /// Falls back to full tree walk if the fast path doesn't find a VS.
    pub fn set_voltage(&mut self, v: f64) {
        // Fast path: root is Binary(Series), left child is Leaf(VoltageSource)
        if let DynNode::Binary { ref mut left, .. } = self {
            if let DynNode::Leaf(ref mut leaf) = **left {
                if leaf.set_voltage(v) {
                    return;
                }
            }
        }
        // Fallback: walk the tree (handles unusual topologies)
        self.for_each_leaf_mut(&mut |leaf| leaf.set_voltage(v));
    }

    /// Set the VS leaf's port resistance (source impedance) and recompute
    /// all cached gamma/rp values up the tree.
    ///
    /// Used by blockwise K-method: the Thevenin impedance at each block's
    /// coupling port determines the VS source impedance. This affects the
    /// Series adaptor's gamma and thus the filter's frequency response.
    ///
    /// After calling this, the tree's port_resistance() will change,
    /// so K-tables must be regenerated.
    pub fn set_vs_port_resistance(&mut self, rp: f64) {
        // Fast path: root is Binary(Series), left child is Leaf(VoltageSource).
        // This is the standard WDF tree topology from with_voltage_source().
        if let DynNode::Binary { ref mut left, .. } = self {
            if let DynNode::Leaf(LeafKind::VoltageSource(ref mut vs)) = **left {
                vs.rp = rp;
            }
        }
        // Recompute all cached gamma/rp values in the tree
        self.recompute();
    }

    /// Set a specific port's voltage source by port name.
    /// Returns true if found. Used for CV input ports.
    /// Bypasses the set_voltage guard (which blocks port-named VS from
    /// being overwritten by the global set_voltage call).
    ///
    /// NOTE: Prefer `resolve_port_vs_ptr` + direct pointer write for hot paths.
    /// This method walks the tree recursively with string comparison.
    pub fn set_voltage_by_port(&mut self, port_name: &str, v: f64) -> bool {
        match self {
            DynNode::Leaf(LeafKind::VoltageSource(vs)) => {
                if vs.port_name.as_deref() == Some(port_name) {
                    vs.voltage = v; // Direct set, bypasses trait guard
                    true
                } else {
                    false
                }
            }
            DynNode::Leaf(_) => false,
            DynNode::Binary {
                ref mut left,
                ref mut right,
                ..
            } => left.set_voltage_by_port(port_name, v) || right.set_voltage_by_port(port_name, v),
            _ => false,
        }
    }

    /// Resolve the main (non-port) VS leaf's `voltage` field pointer.
    ///
    /// Walks the tree once to find the first `WdfVoltageSource` without a
    /// `port_name` and that is not a cathode bias source. Returns a raw
    /// pointer to its `voltage: f64` field. The pointer is stable because
    /// all non-root DynNode children are heap-allocated via `Box`.
    ///
    /// # Safety
    /// The returned pointer is valid as long as the tree is not dropped or
    /// structurally modified (which never happens after construction).
    pub fn resolve_main_vs_ptr(&mut self) -> Option<*mut f64> {
        match self {
            DynNode::Leaf(LeafKind::VoltageSource(vs)) => {
                if vs.port_name.is_none() && !vs.is_cathode_bias {
                    Some(&mut vs.voltage as *mut f64)
                } else {
                    None
                }
            }
            DynNode::Leaf(_) => None,
            DynNode::Binary {
                ref mut left,
                ref mut right,
                ..
            } => left
                .resolve_main_vs_ptr()
                .or_else(|| right.resolve_main_vs_ptr()),
            DynNode::Transformer {
                ref mut secondary, ..
            } => secondary.resolve_main_vs_ptr(),
            DynNode::RType {
                ref mut children, ..
            } => {
                for c in children.iter_mut() {
                    if let Some(p) = c.resolve_main_vs_ptr() {
                        return Some(p);
                    }
                }
                None
            }
        }
    }

    /// Resolve a named port VS leaf's `voltage` field pointer.
    ///
    /// Walks the tree once to find the `WdfVoltageSource` with matching
    /// `port_name`. Returns a raw pointer to its `voltage: f64` field.
    ///
    /// # Safety
    /// Same as `resolve_main_vs_ptr` — valid as long as tree is not dropped.
    pub fn resolve_port_vs_ptr(&mut self, port_name: &str) -> Option<*mut f64> {
        match self {
            DynNode::Leaf(LeafKind::VoltageSource(vs)) => {
                if vs.port_name.as_deref() == Some(port_name) {
                    Some(&mut vs.voltage as *mut f64)
                } else {
                    None
                }
            }
            DynNode::Leaf(_) => None,
            DynNode::Binary {
                ref mut left,
                ref mut right,
                ..
            } => left
                .resolve_port_vs_ptr(port_name)
                .or_else(|| right.resolve_port_vs_ptr(port_name)),
            DynNode::Transformer {
                ref mut secondary, ..
            } => secondary.resolve_port_vs_ptr(port_name),
            DynNode::RType {
                ref mut children, ..
            } => {
                for c in children.iter_mut() {
                    if let Some(p) = c.resolve_port_vs_ptr(port_name) {
                        return Some(p);
                    }
                }
                None
            }
        }
    }

    /// Wrap a specific leaf (found by comp_id) with a named VS in series.
    /// The VS represents a voltage port driving current through the leaf.
    /// Returns true if the leaf was found and wrapped.
    pub fn wrap_leaf_with_vs(&mut self, target_comp_id: &str, port_name: &str) -> bool {
        match self {
            DynNode::Leaf(ref leaf) => {
                if leaf.comp_id() == Some(target_comp_id) {
                    // Found the target leaf — wrap self with Series(VS, self)
                    let port_vs = DynNode::Leaf(LeafKind::VoltageSource(WdfVoltageSource {
                        voltage: 0.0,
                        rp: 1.0,
                        is_cathode_bias: false,
                        port_name: Some(alloc::string::String::from(port_name)),
                    }));
                    let old = core::mem::replace(self, DynNode::VoltageSource(0.0, 1.0));
                    *self = DynNode::Series(Box::new(port_vs), Box::new(old));
                    true
                } else {
                    false
                }
            }
            DynNode::Binary {
                ref mut left,
                ref mut right,
                ..
            } => {
                left.wrap_leaf_with_vs(target_comp_id, port_name)
                    || right.wrap_leaf_with_vs(target_comp_id, port_name)
            }
            _ => false, // Transformer, RType — not expected for port injection
        }
    }

    /// Update a pot's position. Returns true if found.
    pub fn set_pot(&mut self, target_id: &str, pos: f64) -> bool {
        self.for_each_leaf_mut(&mut |leaf| leaf.set_control(target_id, pos))
    }

    /// Update a resistor's resistance by component ID. Returns true if found.
    pub fn set_resistor(&mut self, target_id: &str, new_ohms: f64) -> bool {
        self.for_each_leaf_mut(&mut |leaf| {
            if leaf.comp_id() == Some(target_id) && leaf.type_tag() == "resistor" {
                leaf.set_resistance(new_ohms)
            } else {
                false
            }
        })
    }

    /// Update a capacitor's value and port resistance by component ID.
    pub fn set_capacitor(&mut self, target_id: &str, new_farads: f64, sample_rate: f64) -> bool {
        self.for_each_leaf_mut(&mut |leaf| {
            let tag = leaf.type_tag();
            if leaf.comp_id() == Some(target_id) && (tag == "capacitor" || tag == "leaky_capacitor")
            {
                leaf.set_control(target_id, new_farads);
                leaf.update_sample_rate(sample_rate);
                true
            } else {
                false
            }
        })
    }

    /// Update an inductor's value and port resistance by component ID.
    pub fn set_inductor(&mut self, target_id: &str, new_henries: f64, sample_rate: f64) -> bool {
        self.for_each_leaf_mut(&mut |leaf| {
            if leaf.comp_id() == Some(target_id) && leaf.type_tag() == "inductor" {
                // Update inductance value — inductors don't use set_control for value changes
                // but we need to update rp. We use set_resistance as a proxy for now.
                // In practice this would need a dedicated method, but for API compat
                // we use the fact that inductor rp = 2*fs*L.
                leaf.set_resistance(2.0 * sample_rate * new_henries);
                true
            } else {
                false
            }
        })
    }

    /// List all editable passive leaves in the tree.
    pub fn list_editable_leaves(&self) -> Vec<(String, &'static str, f64)> {
        let mut result = Vec::new();
        self.for_each_leaf(&mut |leaf| {
            if let Some((kind, value)) = leaf.editable_info() {
                if let Some(id) = leaf.comp_id() {
                    result.push((id.to_string(), kind, value));
                }
            }
        });
        result
    }

    /// Read a pot's current resistance by component ID.
    pub fn get_pot_resistance(&self, target_id: &str) -> Option<f64> {
        self.find_leaf(&|leaf| {
            if leaf.type_tag() == "pot" && leaf_matches_id(leaf, target_id) {
                Some(leaf.port_resistance())
            } else {
                None
            }
        })
    }

    /// Read a pot's current position (0.0..1.0) by component ID.
    pub fn get_pot_position(&self, target_id: &str) -> Option<f64> {
        self.find_leaf(&|leaf| {
            if leaf.type_tag() == "pot" && leaf_matches_id(leaf, target_id) {
                leaf.pot_position()
            } else {
                None
            }
        })
    }

    /// Extract voltage at a named leaf after the down-sweep.
    /// Visit all leaves (non-short-circuiting).
    fn visit_leaves(&self, f: &impl Fn(&dyn WdfLeaf)) {
        match self {
            Self::Leaf(leaf) => f(leaf),
            Self::Binary { left, right, .. } => {
                left.visit_leaves(f);
                right.visit_leaves(f);
            }
            Self::Transformer { secondary, .. } => secondary.visit_leaves(f),
            Self::RType { children, .. } => {
                for c in children {
                    c.visit_leaves(f);
                }
            }
        }
    }

    pub fn leaf_voltage(&self, target_id: &str) -> Option<f64> {
        // For pots: prefer the non-complement (GND-side) half.
        // Its voltage = V_wiper (voltage from wiper to GND).
        // The complement half gives V_in - V_wiper (wrong for output).
        let best: Cell<Option<(f64, bool)>> = Cell::new(None);
        self.visit_leaves(&|leaf| {
            if leaf_matches_id(leaf, target_id) {
                let v = leaf.leaf_voltage();
                let is_preferred = leaf.type_tag() == "pot" && !leaf.is_complement();
                match best.get() {
                    None => best.set(Some((v, is_preferred))),
                    Some((_, false)) if is_preferred => best.set(Some((v, true))),
                    _ => {}
                }
            }
        });
        best.get().map(|(v, _)| v)
    }

    /// Project the voltage at a named leaf for a root incident wave without
    /// mutating leaf state. This is used by implicit solvers that need a
    /// probe voltage during Newton iteration before the final down-sweep.
    pub fn leaf_voltage_for_incident(&self, target_id: &str, a_root: f64) -> Option<f64> {
        let best: Cell<Option<(f64, bool)>> = Cell::new(None);
        self.visit_leaf_voltage_for_incident(target_id, a_root, &best);
        best.get().map(|(v, _)| v)
    }

    fn visit_leaf_voltage_for_incident(
        &self,
        target_id: &str,
        a: f64,
        best: &Cell<Option<(f64, bool)>>,
    ) {
        match self {
            Self::Leaf(leaf) => {
                if leaf_matches_id(leaf, target_id) {
                    let v = projected_leaf_voltage(leaf, a);
                    let is_preferred = leaf.type_tag() == "pot" && !leaf.is_complement();
                    match best.get() {
                        None => best.set(Some((v, is_preferred))),
                        Some((_, false)) if is_preferred => best.set(Some((v, true))),
                        _ => {}
                    }
                }
            }
            Self::Binary {
                kind,
                left,
                right,
                gamma,
                b1,
                b2,
                ..
            } => match kind {
                BinaryKind::Series => {
                    let sum = *b1 + *b2 + a;
                    let a1 = *b1 - *gamma * sum;
                    let a2 = *b2 - (1.0 - *gamma) * sum;
                    left.visit_leaf_voltage_for_incident(target_id, a1, best);
                    right.visit_leaf_voltage_for_incident(target_id, a2, best);
                }
                BinaryKind::Parallel => {
                    let diff = *b2 - *b1;
                    let a1 = a + *gamma * diff;
                    let a2 = a - (1.0 - *gamma) * diff;
                    left.visit_leaf_voltage_for_incident(target_id, a1, best);
                    right.visit_leaf_voltage_for_incident(target_id, a2, best);
                }
            },
            Self::Transformer {
                secondary,
                turns_ratio,
                ..
            } => {
                secondary.visit_leaf_voltage_for_incident(target_id, a / *turns_ratio, best);
            }
            Self::RType { adaptor, children } => {
                let a_children = adaptor.scatter_down(a);
                for (child, &a_i) in children.iter().zip(a_children.iter()) {
                    child.visit_leaf_voltage_for_incident(target_id, a_i, best);
                }
            }
        }
    }

    /// Return this leaf's component ID (only if this node is a leaf).
    pub fn leaf_comp_id(&self) -> Option<String> {
        match self {
            Self::Leaf(leaf) => leaf.comp_id().map(|s| s.to_string()),
            _ => None,
        }
    }

    /// Tag the deepest Resistor leaf as a Pot for output probe identification.
    pub fn tag_as_probe(&mut self, tag: &str) -> bool {
        match self {
            Self::Leaf(leaf) => {
                if leaf.type_tag() == "resistor" {
                    let rp = leaf.port_resistance();
                    *self = DynNode::Leaf(LeafKind::Pot(WdfPot {
                        comp_id: tag.to_string(),
                        max_resistance: rp,
                        position: 1.0,
                        taper: PotTaper::B,
                        rp,
                        last_a: 0.0,
                        complement: false,
                    }));
                    true
                } else {
                    false
                }
            }
            Self::Binary { right, .. } => right.tag_as_probe(tag),
            _ => false,
        }
    }

    /// Recompute all adaptor coefficients bottom-up (call after pot changes).
    /// Also clears dirty flags so incremental recompute starts clean.
    pub fn recompute(&mut self) {
        match self {
            Self::Binary {
                kind,
                left,
                right,
                rp,
                gamma,
                dirty,
                ..
            } => {
                left.recompute();
                right.recompute();
                let r1 = left.port_resistance();
                let r2 = right.port_resistance();
                match kind {
                    BinaryKind::Series => {
                        *rp = r1 + r2;
                        // Guard: if both children are 0Ω (short circuit),
                        // gamma is undefined — default to 0.5 (equal split).
                        *gamma = if *rp > 0.0 { r1 / *rp } else { 0.5 };
                    }
                    BinaryKind::Parallel => {
                        let sum = r1 + r2;
                        // Guard: if both children are 0Ω, parallel is 0Ω
                        // and gamma is undefined — default to 0.5.
                        *rp = if sum > 0.0 { r1 * r2 / sum } else { 0.0 };
                        *gamma = if sum > 0.0 { r2 / sum } else { 0.5 };
                    }
                }
                *dirty = false;
            }
            Self::Transformer {
                secondary,
                turns_ratio,
                rp,
                dirty,
                ..
            } => {
                secondary.recompute();
                let r_sec = secondary.port_resistance();
                *rp = *turns_ratio * *turns_ratio * r_sec;
                *dirty = false;
            }
            Self::RType {
                adaptor: _,
                children,
            } => {
                for child in children.iter_mut() {
                    child.recompute();
                }
            }
            Self::Leaf(_) => {}
        }
    }

    /// Post-construction pass: sets `has_dynamic` based on whether any descendant
    /// leaf is dynamic. Call once after tree construction.
    pub fn compute_dynamic_flags(&mut self) -> bool {
        match self {
            Self::Leaf(leaf) => leaf.is_dynamic(),
            Self::Binary {
                left,
                right,
                has_dynamic,
                dirty,
                ..
            } => {
                let left_dyn = left.compute_dynamic_flags();
                let right_dyn = right.compute_dynamic_flags();
                *has_dynamic = left_dyn || right_dyn;
                *dirty = false;
                *has_dynamic
            }
            Self::Transformer {
                secondary,
                has_dynamic,
                dirty,
                ..
            } => {
                let sec_dyn = secondary.compute_dynamic_flags();
                *has_dynamic = sec_dyn;
                *dirty = false;
                *has_dynamic
            }
            Self::RType { children, .. } => {
                let mut any_dyn = false;
                for child in children.iter_mut() {
                    any_dyn |= child.compute_dynamic_flags();
                }
                any_dyn
            }
        }
    }

    /// Set a pot's position AND mark ancestors dirty on the way back up.
    /// Returns true if the pot was found (signals parent to mark dirty).
    ///
    /// Uses bitwise OR (not short-circuit) so split pots (__aw/__wb) that
    /// appear in both subtrees are both updated.
    pub fn set_pot_dirty(&mut self, target_id: &str, pos: f64) -> bool {
        match self {
            Self::Leaf(leaf) => leaf.set_control(target_id, pos),
            Self::Binary {
                left, right, dirty, ..
            } => {
                let a = left.set_pot_dirty(target_id, pos);
                let b = right.set_pot_dirty(target_id, pos);
                let found = a | b;
                if found {
                    *dirty = true;
                }
                found
            }
            Self::Transformer {
                secondary, dirty, ..
            } => {
                let found = secondary.set_pot_dirty(target_id, pos);
                if found {
                    *dirty = true;
                }
                found
            }
            Self::RType { children, .. } => {
                let mut found = false;
                for child in children.iter_mut() {
                    found |= child.set_pot_dirty(target_id, pos);
                }
                found
            }
        }
    }

    /// Recompute only dirty subtrees. Skips entirely-static subtrees.
    pub fn recompute_incremental(&mut self) {
        match self {
            Self::Leaf(_) => {}
            Self::Binary {
                has_dynamic: false, ..
            } => {
                // All static — nothing ever changes.
            }
            Self::Binary { dirty: false, .. } => {
                // Children haven't changed since last recompute.
            }
            Self::Binary {
                kind,
                left,
                right,
                rp,
                gamma,
                dirty,
                ..
            } => {
                // dirty == true here
                left.recompute_incremental();
                right.recompute_incremental();
                let r1 = left.port_resistance();
                let r2 = right.port_resistance();
                match kind {
                    BinaryKind::Series => {
                        *rp = r1 + r2;
                        *gamma = if *rp > 0.0 { r1 / *rp } else { 0.5 };
                    }
                    BinaryKind::Parallel => {
                        let sum = r1 + r2;
                        *rp = if sum > 0.0 { r1 * r2 / sum } else { 0.0 };
                        *gamma = if sum > 0.0 { r2 / sum } else { 0.5 };
                    }
                }
                *dirty = false;
            }
            Self::Transformer {
                has_dynamic: false, ..
            } => {
                // All static.
            }
            Self::Transformer { dirty: false, .. } => {
                // Children haven't changed.
            }
            Self::Transformer {
                secondary,
                turns_ratio,
                rp,
                dirty,
                ..
            } => {
                // dirty == true here
                secondary.recompute_incremental();
                let r_sec = secondary.port_resistance();
                *rp = *turns_ratio * *turns_ratio * r_sec;
                *dirty = false;
            }
            Self::RType { children, .. } => {
                for child in children.iter_mut() {
                    child.recompute_incremental();
                }
            }
        }
    }

    /// Update sample rate for all reactive elements.
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.for_each_leaf_mut(&mut |leaf| {
            leaf.update_sample_rate(sample_rate);
            false // don't stop
        });
    }

    /// Update sample rate for all elements (including photocoupler/JFET) with recompute.
    pub fn update_sample_rate(&mut self, fs: f64) {
        self.for_each_leaf_mut(&mut |leaf| {
            leaf.update_sample_rate(fs);
            false
        });
        // After updating leaf port resistances, recompute adaptor coefficients
        self.recompute();
    }

    /// Reset all state.
    pub fn reset(&mut self) {
        match self {
            Self::Leaf(leaf) => leaf.reset(),
            Self::Binary {
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
        }
    }

    /// Extract voltage from the first reactive element in the tree.
    #[allow(dead_code)]
    pub fn reactive_voltage(&self) -> Option<f64> {
        self.find_leaf(&|leaf| {
            if leaf.is_reactive() {
                Some(leaf.leaf_voltage())
            } else {
                None
            }
        })
    }

    /// Compute output voltage for passive WDF stages using resistive termination.
    pub fn resistive_termination_voltage(&self, b_tree: f64) -> Option<f64> {
        match self {
            Self::Binary {
                kind: BinaryKind::Parallel,
                left,
                ..
            } => {
                if let Self::Leaf(leaf) = left.as_ref() {
                    if leaf.type_tag() == "resistor" || leaf.type_tag() == "pot" {
                        return Some(b_tree / 2.0);
                    }
                }
                None
            }
            Self::Binary {
                kind: BinaryKind::Series,
                left,
                ..
            } => {
                if let Self::Leaf(leaf) = left.as_ref() {
                    if leaf.type_tag() == "resistor" || leaf.type_tag() == "pot" {
                        return Some(b_tree / 2.0);
                    }
                }
                None
            }
            _ => None,
        }
    }

    /// Compute junction voltage for series filters.
    pub fn series_junction_voltage(&self, a_root: f64) -> Option<f64> {
        match self {
            Self::Binary {
                kind: BinaryKind::Series,
                gamma,
                b1,
                b2,
                right,
                ..
            } => match right.as_ref() {
                Self::Leaf(leaf) => {
                    let tag = leaf.type_tag();
                    if tag == "resistor" || tag == "capacitor" || tag == "inductor" || tag == "pot"
                    {
                        let sum = *b1 + *b2 + a_root;
                        let a2 = *b2 - (1.0 - *gamma) * sum;
                        Some((a2 + *b2) / 2.0)
                    } else {
                        None
                    }
                }
                Self::Binary {
                    kind: BinaryKind::Series,
                    ..
                } => right.series_junction_voltage(a_root),
                _ => None,
            },
            _ => None,
        }
    }

    /// Extract junction voltage for short-circuit terminated passive filters.
    pub fn short_circuit_junction_voltage(&self, a_root: f64) -> Option<f64> {
        match self {
            Self::Binary {
                kind: BinaryKind::Series,
                gamma,
                b1,
                b2,
                left,
                right,
                ..
            } => {
                // Check for VS on left or right
                let left_is_vs = matches!(left.as_ref(), Self::Leaf(l) if l.type_tag() == "voltage_source" || l.type_tag() == "cathode_bias_source");
                let right_is_vs = matches!(right.as_ref(), Self::Leaf(l) if l.type_tag() == "voltage_source" || l.type_tag() == "cathode_bias_source");

                if left_is_vs {
                    // VS emits b = 2*V (WDF convention). The junction voltage
                    // needs to be halved to convert from wave to circuit voltage.
                    let sum = *b1 + *b2 + a_root;
                    let a_inner = *b2 - (1.0 - *gamma) * sum;
                    right.extract_load_voltage(a_inner).map(|v| v / 2.0)
                } else if right_is_vs {
                    let sum = *b1 + *b2 + a_root;
                    let a_left = *b1 - *gamma * sum;
                    left.extract_load_voltage(a_left).map(|v| v / 2.0)
                } else {
                    // Not Series(VS, _), try direct load
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
            Self::Leaf(leaf) if leaf.type_tag() == "resistor" || leaf.type_tag() == "pot" => {
                Some(a_root / 2.0)
            }
            _ => None,
        }
    }

    /// Extract voltage at the output element in a filter subtree.
    /// For short-circuit terminated filters, the output is at the element
    /// nearest to ground — the right child of the innermost series adaptor.
    fn extract_load_voltage(&self, a_parent: f64) -> Option<f64> {
        match self {
            Self::Leaf(leaf) => {
                let tag = leaf.type_tag();
                // Any passive leaf can be the output: resistor, pot, cap, inductor
                if tag == "resistor"
                    || tag == "pot"
                    || tag == "capacitor"
                    || tag == "leaky_capacitor"
                    || tag == "inductor"
                {
                    Some(a_parent / 2.0)
                } else {
                    None
                }
            }
            Self::Binary {
                kind: BinaryKind::Series,
                gamma,
                b1,
                b2,
                left,
                right,
                ..
            } => {
                let sum = *b1 + *b2 + a_parent;
                if right.is_load_element() {
                    let a2 = *b2 - (1.0 - *gamma) * sum;
                    Some((a2 + *b2) / 2.0)
                } else if left.is_load_element() && matches!(right.as_ref(), Self::Binary { .. }) {
                    let a2 = *b2 - (1.0 - *gamma) * sum;
                    right.extract_load_voltage(a2).or_else(|| {
                        let a1 = *b1 - *gamma * sum;
                        Some((a1 + *b1) / 2.0)
                    })
                } else if left.is_load_element() {
                    let a1 = *b1 - *gamma * sum;
                    Some((a1 + *b1) / 2.0)
                } else {
                    if matches!(right.as_ref(), Self::Binary { .. }) {
                        let a2 = *b2 - (1.0 - *gamma) * sum;
                        right.extract_load_voltage(a2)
                    } else if matches!(left.as_ref(), Self::Binary { .. }) {
                        let a1 = *b1 - *gamma * sum;
                        left.extract_load_voltage(a1)
                    } else {
                        None
                    }
                }
            }
            Self::Binary {
                kind: BinaryKind::Parallel,
                gamma,
                b1,
                b2,
                left,
                right,
                ..
            } => {
                let sum = *b1 + *b2 + a_parent;
                if right.is_load_element() {
                    let a2 = *b2 - (1.0 - *gamma) * sum;
                    Some((a2 + *b2) / 2.0)
                } else if left.is_load_element() {
                    let a1 = *b1 - *gamma * sum;
                    Some((a1 + *b1) / 2.0)
                } else {
                    // Recurse into children for complex parallel trees
                    let a2 = *b2 - (1.0 - *gamma) * sum;
                    let a1 = *b1 - *gamma * sum;
                    right
                        .extract_load_voltage(a2)
                        .or_else(|| left.extract_load_voltage(a1))
                }
            }
            _ => None,
        }
    }

    /// Check if this node is a passive element that can be an output point.
    fn is_load_element(&self) -> bool {
        matches!(self, Self::Leaf(l) if {
            let t = l.type_tag();
            t == "resistor" || t == "pot" || t == "capacitor" || t == "leaky_capacitor" || t == "inductor"
        })
    }

    /// Check if this tree contains any reactive elements.
    #[allow(dead_code)]
    pub fn has_reactive_elements(&self) -> bool {
        let mut found = false;
        self.for_each_leaf(&mut |leaf| {
            if leaf.is_reactive() {
                found = true;
            }
        });
        found
    }

    /// Set LED drive level for a photocoupler. Returns true if found.
    pub fn set_photocoupler_led(&mut self, target_id: &str, led_drive: f64) -> bool {
        self.for_each_leaf_mut(&mut |leaf| {
            if leaf.type_tag() == "photocoupler" && leaf.comp_id() == Some(target_id) {
                leaf.set_control(target_id, led_drive)
            } else {
                false
            }
        })
    }

    /// Set Vgs for a JFET variable resistor leaf. Returns true if found.
    pub fn set_jfet_vr_vgs(&mut self, target_id: &str, vgs: f64) -> bool {
        self.for_each_leaf_mut(&mut |leaf| {
            if leaf.type_tag() == "jfet_vr" && leaf.comp_id() == Some(target_id) {
                leaf.set_control(target_id, vgs)
            } else {
                false
            }
        })
    }

    /// Set switch position for SwitchedResistor nodes. Returns count of updated.
    pub fn set_switch_position(&mut self, target_switch: &str, new_position: usize) -> usize {
        let mut count = 0usize;
        self.for_each_leaf_mut(&mut |leaf| {
            if leaf.type_tag() == "switched_resistor" && leaf.comp_id() == Some(target_switch) {
                leaf.set_control(target_switch, new_position as f64);
                count += 1;
                true
            } else {
                false
            }
        });
        count
    }

    /// Debug dump: print tree structure.
    pub fn debug_dump(&self, indent: usize) -> String {
        let pad = "  ".repeat(indent);
        match self {
            Self::Leaf(leaf) => format!("{pad}{}", leaf.debug_info()),
            Self::Binary {
                kind,
                left,
                right,
                rp,
                gamma,
                b1,
                b2,
                ..
            } => {
                let kind_str = match kind {
                    BinaryKind::Series => "Series",
                    BinaryKind::Parallel => "Parallel",
                };
                let mut s =
                    format!("{pad}{kind_str}(Rp={rp:.1}Ω, γ={gamma:.6}, b1={b1:.6}, b2={b2:.6})\n");
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

    /// Get the UnitDelay's outgoing state.
    pub fn get_unit_delay_state(&self) -> f64 {
        self.find_leaf(&|leaf| leaf.unit_delay_state())
            .unwrap_or(0.0)
    }

    /// Set the UnitDelay's incoming partner state.
    pub fn set_unit_delay_partner(&mut self, val: f64) -> bool {
        self.for_each_leaf_mut(&mut |leaf| leaf.set_unit_delay_partner(val))
    }

    /// Check hybrid linear/nonlinear devices for operating region violations.
    #[cfg(feature = "runtime-warnings")]
    pub fn check_hybrid_warnings(&mut self, _v_port: f64, _sample_index: u64) {
        // Runtime warnings for JfetVr and Photocoupler need access to inner types.
        // This is feature-gated diagnostic code; will need concrete downcast support.
    }

    /// Count total nodes in tree (for statistics).
    pub fn node_count(&self) -> usize {
        match self {
            Self::Leaf(_) => 1,
            Self::Binary { left, right, .. } => 1 + left.node_count() + right.node_count(),
            Self::Transformer { secondary, .. } => 1 + secondary.node_count(),
            Self::RType { children, .. } => {
                1 + children.iter().map(|c| c.node_count()).sum::<usize>()
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::String;

    /// Build a typical WDF tree: Series(VS, Parallel(R, C))
    fn make_simple_tree() -> DynNode {
        let vs = DynNode::VoltageSource(0.0, 1.0);
        let r = DynNode::Resistor(Some(String::from("R1")), 1000.0);
        let c = DynNode::Capacitor(Some(String::from("C1")), 1e-6, 3316.0);
        let rc = DynNode::Parallel(Box::new(r), Box::new(c));
        let mut tree = DynNode::Series(Box::new(vs), Box::new(rc));
        tree.recompute();
        tree
    }

    /// Build a tree with a port VS: Series(VS_main, Series(VS_port, R))
    fn make_port_tree() -> DynNode {
        let vs_main = DynNode::VoltageSource(0.0, 1.0);
        let r = DynNode::Resistor(Some(String::from("R_cv")), 47000.0);
        // Wrap R_cv with a port VS
        let port_vs = DynNode::Leaf(LeafKind::VoltageSource(WdfVoltageSource {
            voltage: 0.0,
            rp: 1.0,
            is_cathode_bias: false,
            port_name: Some(String::from("cv_cutoff")),
        }));
        let inner = DynNode::Series(Box::new(port_vs), Box::new(r));
        let mut tree = DynNode::Series(Box::new(vs_main), Box::new(inner));
        tree.recompute();
        tree
    }

    #[test]
    fn resolve_main_vs_ptr_simple() {
        let mut tree = make_simple_tree();
        let ptr = tree.resolve_main_vs_ptr();
        assert!(ptr.is_some(), "should find main VS");

        // Write via pointer, verify via set_voltage
        let ptr = ptr.unwrap();
        unsafe {
            *ptr = 3.14;
        }

        // Verify the VS reflects the new voltage
        let b = tree.reflected();
        // reflected() on a VS returns 2*voltage, so with 3.14 the tree
        // should propagate a non-zero reflected wave
        assert!(
            b.abs() > 0.01,
            "reflected wave should be non-zero after ptr write, got {b}"
        );
    }

    #[test]
    fn resolve_main_vs_ptr_not_port() {
        let mut tree = make_port_tree();
        let ptr = tree.resolve_main_vs_ptr();
        assert!(ptr.is_some(), "should find main (non-port) VS");

        // The main VS is the one WITHOUT port_name
        unsafe {
            *ptr.unwrap() = 1.0;
        }

        // Verify port VS was NOT modified
        let port_ptr = tree.resolve_port_vs_ptr("cv_cutoff");
        assert!(port_ptr.is_some());
        let port_v = unsafe { *port_ptr.unwrap() };
        assert_eq!(port_v, 0.0, "port VS should be untouched");
    }

    #[test]
    fn resolve_port_vs_ptr_found() {
        let mut tree = make_port_tree();
        let ptr = tree.resolve_port_vs_ptr("cv_cutoff");
        assert!(ptr.is_some(), "should find port VS 'cv_cutoff'");

        // Write and read back
        unsafe {
            *ptr.unwrap() = 2.5;
        }
        let readback = unsafe { *ptr.unwrap() };
        assert_eq!(readback, 2.5);
    }

    #[test]
    fn resolve_port_vs_ptr_not_found() {
        let mut tree = make_port_tree();
        let ptr = tree.resolve_port_vs_ptr("nonexistent_port");
        assert!(ptr.is_none(), "should not find nonexistent port");
    }

    #[test]
    fn resolve_port_vs_ptr_independent_of_main() {
        let mut tree = make_port_tree();
        let main_ptr = tree.resolve_main_vs_ptr().unwrap();
        let port_ptr = tree.resolve_port_vs_ptr("cv_cutoff").unwrap();

        // They should point to different locations
        assert_ne!(
            main_ptr, port_ptr,
            "main and port VS should be different pointers"
        );

        // Write to both independently
        unsafe {
            *main_ptr = 1.0;
            *port_ptr = 5.0;
        }
        let main_v = unsafe { *main_ptr };
        let port_v = unsafe { *port_ptr };
        assert_eq!(main_v, 1.0);
        assert_eq!(port_v, 5.0);
    }

    #[test]
    fn set_voltage_by_port_matches_ptr() {
        let mut tree = make_port_tree();

        // Set via tree walk
        tree.set_voltage_by_port("cv_cutoff", 7.7);

        // Read via cached pointer
        let ptr = tree.resolve_port_vs_ptr("cv_cutoff").unwrap();
        let v = unsafe { *ptr };
        assert_eq!(v, 7.7, "ptr should read value set by tree walk");
    }

    #[test]
    fn leaf_voltage_for_incident_matches_downsweep_without_mutating() {
        let mut tree = make_simple_tree();
        tree.set_voltage(1.0);
        let _ = tree.reflected();

        let before = tree.leaf_voltage("C1").unwrap();
        let projected = tree.leaf_voltage_for_incident("C1", 0.25).unwrap();

        let mut clone = tree.clone();
        clone.set_incident(0.25);
        let expected = clone.leaf_voltage("C1").unwrap();

        assert!(
            (projected - expected).abs() < 1e-12,
            "projected voltage should match a real down-sweep: projected={projected}, expected={expected}"
        );
        assert_eq!(
            tree.leaf_voltage("C1").unwrap(),
            before,
            "projection must not update original leaf state"
        );
    }

    #[test]
    fn ptr_write_visible_via_tree_walk() {
        let mut tree = make_port_tree();
        let ptr = tree.resolve_port_vs_ptr("cv_cutoff").unwrap();

        // Write via pointer
        unsafe {
            *ptr = 3.3;
        }

        // Verify set_voltage_by_port (tree walk) overwrites what ptr sees
        tree.set_voltage_by_port("cv_cutoff", 99.0);
        let v = unsafe { *ptr };
        assert_eq!(v, 99.0, "tree walk write should be visible via ptr");

        // And ptr write should be visible via another tree walk
        unsafe {
            *ptr = 42.0;
        }
        // set_voltage_by_port with same name should find the updated value
        // (it overwrites, but the point is the memory is shared)
        tree.set_voltage_by_port("cv_cutoff", 0.0);
        let v2 = unsafe { *ptr };
        assert_eq!(v2, 0.0);
    }

    #[test]
    fn cathode_bias_excluded_from_main() {
        // Cathode bias VS should NOT be returned by resolve_main_vs_ptr
        let cb = DynNode::CathodeBiasSource(10.0, 100.0);
        let r = DynNode::Resistor(Some(String::from("R1")), 1000.0);
        let mut tree = DynNode::Series(Box::new(cb), Box::new(r));
        tree.recompute();

        let ptr = tree.resolve_main_vs_ptr();
        assert!(
            ptr.is_none(),
            "cathode bias should not be returned as main VS"
        );
    }
}
