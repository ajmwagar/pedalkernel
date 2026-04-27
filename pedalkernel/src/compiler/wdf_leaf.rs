//! WdfLeaf trait for dynamic WDF tree leaf nodes.
//!
//! Each leaf struct encapsulates the WDF scattering behavior of a single
//! circuit element. DynNode::Leaf wraps `Box<dyn WdfLeaf>` for polymorphism.

use crate::dsl::PotTaper;
// Import the elements::WdfLeaf trait to call port_resistance/set_sample_rate/reset
// on Photocoupler and JfetVariableResistor. Renamed to avoid conflict with our WdfLeaf.
use crate::elements::WdfLeaf as ElementsWdfLeaf;
use crate::elements::{JfetVariableResistor, Photocoupler as PhotocouplerInner};

// ═══════════════════════════════════════════════════════════════════════════
// WdfLeaf trait
// ═══════════════════════════════════════════════════════════════════════════

/// Trait for WDF leaf nodes in dynamically-constructed trees.
///
/// Each implementor is a one-port WDF element (R, C, L, pot, etc.)
/// that the DynNode tree wraps in a `Box<dyn WdfLeaf>`.
pub trait WdfLeaf: Send {
    // ── Hot path (per-sample) ────────────────────────────────────────────
    /// Scatter-up: compute reflected wave from current state.
    fn reflected(&mut self) -> f64;
    /// Scatter-down: accept incident wave and update state.
    fn set_incident(&mut self, a: f64);
    /// Port resistance (Ω).
    fn port_resistance(&self) -> f64;

    // ── Identity ─────────────────────────────────────────────────────────
    /// Component ID (e.g., "R1", "Gain__aw"). None for anonymous leaves.
    fn comp_id(&self) -> Option<&str> {
        None
    }
    /// Type tag string for introspection/matching.
    fn type_tag(&self) -> &'static str;

    /// Mark this pot half as the complement: uses 1-value on set_control.
    fn set_complement(&mut self) {}

    // ── Voltage extraction (after down-sweep) ────────────────────────────
    /// Voltage at this leaf after the WDF cycle.
    fn leaf_voltage(&self) -> f64 {
        0.0
    }

    // ── Control/mutation (not per-sample) ────────────────────────────────
    /// Generic control: pot position, LED drive, Vgs, switch position, etc.
    /// Returns true if this leaf handled the control.
    fn set_control(&mut self, _id: &str, _value: f64) -> bool {
        false
    }
    /// Direct resistance override. Returns true if handled.
    fn set_resistance(&mut self, _ohms: f64) -> bool {
        false
    }
    /// Voltage source update. Returns true if handled.
    fn set_voltage(&mut self, _v: f64) -> bool {
        false
    }
    /// Reactive element sample rate update.
    fn update_sample_rate(&mut self, _fs: f64) {}
    /// Reset state (caps/inductors clear to 0).
    fn reset(&mut self) {}

    // ── Introspection ────────────────────────────────────────────────────
    fn is_reactive(&self) -> bool {
        false
    }
    fn editable_info(&self) -> Option<(&'static str, f64)> {
        None
    }
    fn debug_info(&self) -> String;

    // ── Specialized queries ─────────────────────────────────────────────
    /// For UnitDelay: return the outgoing state.
    fn unit_delay_state(&self) -> Option<f64> {
        None
    }
    /// For UnitDelay: set the incoming partner state.
    fn set_unit_delay_partner(&mut self, _val: f64) -> bool {
        false
    }
    /// For Pot: return the current position (0.0..1.0).
    fn pot_position(&self) -> Option<f64> {
        None
    }
    /// For Pot: return the max resistance.
    fn pot_max_resistance(&self) -> Option<f64> {
        None
    }
    /// For Pot: return whether this half uses complement (1-position).
    fn is_complement(&self) -> bool {
        false
    }

    // ── Clone support ────────────────────────────────────────────────────
    fn clone_box(&self) -> Box<dyn WdfLeaf>;
}

impl Clone for Box<dyn WdfLeaf> {
    fn clone(&self) -> Self {
        self.clone_box()
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// WdfResistor
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Clone)]
pub struct WdfResistor {
    pub comp_id: Option<String>,
    pub rp: f64,
    pub last_a: f64,
}

impl WdfLeaf for WdfResistor {
    fn reflected(&mut self) -> f64 {
        0.0
    }
    fn set_incident(&mut self, a: f64) {
        self.last_a = a;
    }
    fn port_resistance(&self) -> f64 {
        self.rp
    }
    fn comp_id(&self) -> Option<&str> {
        self.comp_id.as_deref()
    }
    fn type_tag(&self) -> &'static str {
        "resistor"
    }
    fn leaf_voltage(&self) -> f64 {
        self.last_a / 2.0
    }
    fn set_resistance(&mut self, ohms: f64) -> bool {
        self.rp = ohms;
        true
    }
    fn editable_info(&self) -> Option<(&'static str, f64)> {
        if self.comp_id.is_some() {
            Some(("resistor", self.rp))
        } else {
            None
        }
    }
    fn debug_info(&self) -> String {
        let id = self.comp_id.as_deref().unwrap_or("?");
        format!("Resistor(id=\"{id}\", Rp={:.1}Ω)", self.rp)
    }
    fn clone_box(&self) -> Box<dyn WdfLeaf> {
        Box::new(self.clone())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// WdfCapacitor
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Clone)]
pub struct WdfCapacitor {
    pub comp_id: Option<String>,
    pub capacitance: f64,
    pub rp: f64,
    pub state: f64,
    /// Last reflected wave (captured before state update, for voltage extraction)
    pub last_b: f64,
}

impl WdfLeaf for WdfCapacitor {
    fn reflected(&mut self) -> f64 {
        self.state
    }
    fn set_incident(&mut self, a: f64) {
        self.last_b = self.state;
        self.state = a;
    }
    fn port_resistance(&self) -> f64 {
        self.rp
    }
    fn comp_id(&self) -> Option<&str> {
        self.comp_id.as_deref()
    }
    fn type_tag(&self) -> &'static str {
        "capacitor"
    }
    fn leaf_voltage(&self) -> f64 {
        (self.state + self.last_b) / 2.0
    }
    fn is_reactive(&self) -> bool {
        true
    }
    fn update_sample_rate(&mut self, fs: f64) {
        self.rp = 1.0 / (2.0 * fs * self.capacitance);
    }
    fn reset(&mut self) {
        self.state = 0.0;
        self.last_b = 0.0;
    }
    fn editable_info(&self) -> Option<(&'static str, f64)> {
        if self.comp_id.is_some() {
            Some(("capacitor", self.capacitance))
        } else {
            None
        }
    }
    fn debug_info(&self) -> String {
        let id = self.comp_id.as_deref().unwrap_or("?");
        format!(
            "Capacitor(id=\"{id}\", C={:.3e}F, Rp={:.1}Ω, state={:.6})",
            self.capacitance, self.rp, self.state
        )
    }
    fn clone_box(&self) -> Box<dyn WdfLeaf> {
        Box::new(self.clone())
    }

    fn set_control(&mut self, id: &str, value: f64) -> bool {
        // Support capacitance changes via set_control
        if self.comp_id.as_deref() == Some(id) {
            self.capacitance = value;
            // Note: rp must be updated via update_sample_rate after this
            true
        } else {
            false
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// WdfLeakyCapacitor
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Clone)]
pub struct WdfLeakyCapacitor {
    pub comp_id: Option<String>,
    pub capacitance: f64,
    pub rp: f64,
    pub state: f64,
    pub leakage_decay: f64,
    pub da_coef: Option<f64>,
    pub da_state: f64,
    pub da_rate: f64,
}

impl WdfLeaf for WdfLeakyCapacitor {
    fn reflected(&mut self) -> f64 {
        let b = self.state;
        if let Some(da) = self.da_coef {
            b + da * (self.da_state - self.state)
        } else {
            b
        }
    }
    fn set_incident(&mut self, a: f64) {
        self.state = a * self.leakage_decay;
        if self.da_coef.is_some() {
            self.da_state += (self.state - self.da_state) * self.da_rate;
        }
    }
    fn port_resistance(&self) -> f64 {
        self.rp
    }
    fn comp_id(&self) -> Option<&str> {
        self.comp_id.as_deref()
    }
    fn type_tag(&self) -> &'static str {
        "leaky_capacitor"
    }
    fn leaf_voltage(&self) -> f64 {
        self.state
    }
    fn is_reactive(&self) -> bool {
        true
    }
    fn update_sample_rate(&mut self, fs: f64) {
        self.rp = 1.0 / (2.0 * fs * self.capacitance);
        if self.da_coef.is_some() {
            const DA_TIME_CONSTANT: f64 = 0.5;
            self.da_rate = 1.0 / (fs * DA_TIME_CONSTANT);
        }
    }
    fn reset(&mut self) {
        self.state = 0.0;
        self.da_state = 0.0;
    }
    fn editable_info(&self) -> Option<(&'static str, f64)> {
        if self.comp_id.is_some() {
            Some(("capacitor", self.capacitance))
        } else {
            None
        }
    }
    fn debug_info(&self) -> String {
        let id = self.comp_id.as_deref().unwrap_or("?");
        format!(
            "LeakyCapacitor(id=\"{id}\", C={:.3e}F, Rp={:.1}Ω, state={:.6}, decay={:.6})",
            self.capacitance, self.rp, self.state, self.leakage_decay
        )
    }
    fn clone_box(&self) -> Box<dyn WdfLeaf> {
        Box::new(self.clone())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// WdfInductor
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Clone)]
pub struct WdfInductor {
    pub comp_id: Option<String>,
    pub inductance: f64,
    pub rp: f64,
    pub state: f64,
}

impl WdfLeaf for WdfInductor {
    fn reflected(&mut self) -> f64 {
        -self.state
    }
    fn set_incident(&mut self, a: f64) {
        self.state = a;
    }
    fn port_resistance(&self) -> f64 {
        self.rp
    }
    fn comp_id(&self) -> Option<&str> {
        self.comp_id.as_deref()
    }
    fn type_tag(&self) -> &'static str {
        "inductor"
    }
    fn leaf_voltage(&self) -> f64 {
        self.state / 2.0
    }
    fn is_reactive(&self) -> bool {
        true
    }
    fn update_sample_rate(&mut self, fs: f64) {
        self.rp = 2.0 * fs * self.inductance;
    }
    fn reset(&mut self) {
        self.state = 0.0;
    }
    fn editable_info(&self) -> Option<(&'static str, f64)> {
        if self.comp_id.is_some() {
            Some(("inductor", self.inductance))
        } else {
            None
        }
    }
    fn debug_info(&self) -> String {
        let id = self.comp_id.as_deref().unwrap_or("?");
        format!(
            "Inductor(id=\"{id}\", L={:.3e}H, Rp={:.1}Ω, state={:.6})",
            self.inductance, self.rp, self.state
        )
    }
    fn clone_box(&self) -> Box<dyn WdfLeaf> {
        Box::new(self.clone())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// WdfVoltageSource
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Clone)]
pub struct WdfVoltageSource {
    pub voltage: f64,
    pub rp: f64,
    /// If true, set_voltage is a no-op (cathode bias behavior).
    pub is_cathode_bias: bool,
}

impl WdfLeaf for WdfVoltageSource {
    fn reflected(&mut self) -> f64 {
        2.0 * self.voltage
    }
    fn set_incident(&mut self, _a: f64) {}
    fn port_resistance(&self) -> f64 {
        self.rp
    }
    fn type_tag(&self) -> &'static str {
        if self.is_cathode_bias {
            "cathode_bias_source"
        } else {
            "voltage_source"
        }
    }
    fn set_resistance(&mut self, ohms: f64) -> bool {
        self.rp = ohms;
        true
    }
    fn set_voltage(&mut self, v: f64) -> bool {
        if !self.is_cathode_bias {
            self.voltage = v;
            true
        } else {
            false
        }
    }
    fn debug_info(&self) -> String {
        if self.is_cathode_bias {
            format!(
                "CathodeBiasSource(V={:.3}V, Rp={:.1}Ω)",
                self.voltage, self.rp
            )
        } else {
            format!("VoltageSource(V={:.3}V, Rp={:.1}Ω)", self.voltage, self.rp)
        }
    }
    fn clone_box(&self) -> Box<dyn WdfLeaf> {
        Box::new(self.clone())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// WdfPot
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Clone)]
pub struct WdfPot {
    pub comp_id: String,
    pub max_resistance: f64,
    pub position: f64,
    pub taper: PotTaper,
    pub rp: f64,
    pub last_a: f64,
    /// When true, this half uses the complement of the control position
    /// (1 - value). For a 3-terminal pot split into two halves, one half
    /// tracks position and the other tracks 1-position so their
    /// resistances always sum to max_R.
    pub complement: bool,
}

impl WdfLeaf for WdfPot {
    fn reflected(&mut self) -> f64 {
        0.0
    }
    fn set_incident(&mut self, a: f64) {
        self.last_a = a;
    }
    fn port_resistance(&self) -> f64 {
        self.rp
    }
    fn comp_id(&self) -> Option<&str> {
        Some(&self.comp_id)
    }
    fn type_tag(&self) -> &'static str {
        "pot"
    }
    fn set_complement(&mut self) {
        self.complement = true;
        self.position = 1.0 - self.position;
        let tapered = self.taper.apply(self.position);
        self.rp = (tapered * self.max_resistance).max(1.0);
    }
    fn leaf_voltage(&self) -> f64 {
        self.last_a / 2.0
    }

    fn set_control(&mut self, id: &str, value: f64) -> bool {
        // Match by exact ID or by prefix with __ separator (split pots)
        if self.comp_id == id
            || (self.comp_id.starts_with(id) && self.comp_id[id.len()..].starts_with("__"))
        {
            // Apply complement (for split pots: one half tracks 1-position).
            let effective = if self.complement { 1.0 - value } else { value };
            self.position = effective;
            let tapered_pos = self.taper.apply(effective);
            self.rp = (tapered_pos * self.max_resistance).max(1.0);
            true
        } else {
            false
        }
    }

    fn set_resistance(&mut self, ohms: f64) -> bool {
        self.rp = ohms;
        true
    }

    fn editable_info(&self) -> Option<(&'static str, f64)> {
        Some(("pot", self.rp))
    }

    fn pot_position(&self) -> Option<f64> {
        Some(self.position)
    }
    fn pot_max_resistance(&self) -> Option<f64> {
        Some(self.max_resistance)
    }
    fn is_complement(&self) -> bool {
        self.complement
    }

    fn debug_info(&self) -> String {
        format!(
            "Pot(id=\"{}\", max={:.1}Ω, pos={:.3}, taper={:?}, Rp={:.1}Ω)",
            self.comp_id, self.max_resistance, self.position, self.taper, self.rp
        )
    }
    fn clone_box(&self) -> Box<dyn WdfLeaf> {
        Box::new(self.clone())
    }
}

impl WdfPot {
    /// Read the pot's current resistance.
    pub fn get_resistance(&self) -> f64 {
        self.rp
    }
    /// Read the pot's current position.
    pub fn get_position(&self) -> f64 {
        self.position
    }
    /// Read the pot's max resistance.
    pub fn max_resistance(&self) -> f64 {
        self.max_resistance
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// WdfPhotocoupler
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Clone)]
pub struct WdfPhotocoupler {
    pub comp_id: String,
    pub inner: PhotocouplerInner,
    #[allow(dead_code)]
    pub prev_resistance: f64,
}

impl WdfLeaf for WdfPhotocoupler {
    fn reflected(&mut self) -> f64 {
        0.0
    }
    fn set_incident(&mut self, _a: f64) {}
    fn port_resistance(&self) -> f64 {
        self.inner.port_resistance()
    }
    fn comp_id(&self) -> Option<&str> {
        Some(&self.comp_id)
    }
    fn type_tag(&self) -> &'static str {
        "photocoupler"
    }

    fn set_control(&mut self, id: &str, value: f64) -> bool {
        if self.comp_id == id {
            self.inner.set_led_drive(value);
            true
        } else {
            false
        }
    }

    fn update_sample_rate(&mut self, fs: f64) {
        self.inner.set_sample_rate(fs);
    }

    fn reset(&mut self) {
        self.inner.reset();
    }

    fn debug_info(&self) -> String {
        format!(
            "Photocoupler(id=\"{}\", Rp={:.1}Ω)",
            self.comp_id,
            self.inner.port_resistance()
        )
    }
    fn clone_box(&self) -> Box<dyn WdfLeaf> {
        Box::new(self.clone())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// WdfJfetVr
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Clone)]
pub struct WdfJfetVr {
    pub comp_id: String,
    pub inner: JfetVariableResistor,
    #[allow(dead_code)]
    pub prev_rds: f64,
}

impl WdfLeaf for WdfJfetVr {
    fn reflected(&mut self) -> f64 {
        0.0
    }
    fn set_incident(&mut self, _a: f64) {}
    fn port_resistance(&self) -> f64 {
        self.inner.port_resistance()
    }
    fn comp_id(&self) -> Option<&str> {
        Some(&self.comp_id)
    }
    fn type_tag(&self) -> &'static str {
        "jfet_vr"
    }

    fn set_control(&mut self, id: &str, value: f64) -> bool {
        if self.comp_id == id {
            self.inner.set_vgs(value);
            true
        } else {
            false
        }
    }

    fn update_sample_rate(&mut self, fs: f64) {
        self.inner.set_sample_rate(fs);
    }

    fn reset(&mut self) {
        self.inner.reset();
    }

    fn debug_info(&self) -> String {
        format!(
            "JfetVr(id=\"{}\", Vgs={:.3}V, Rds={:.1}Ω)",
            self.comp_id,
            self.inner.vgs(),
            self.inner.rds()
        )
    }
    fn clone_box(&self) -> Box<dyn WdfLeaf> {
        Box::new(self.clone())
    }
}

impl WdfJfetVr {
    pub fn vgs(&self) -> f64 {
        self.inner.vgs()
    }
    pub fn rds(&self) -> f64 {
        self.inner.rds()
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// WdfSwitchedResistor
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Clone)]
pub struct WdfSwitchedResistor {
    pub switch_id: String,
    pub path_index: usize,
    #[allow(dead_code)]
    pub num_paths: usize,
    pub r_active: f64,
    pub r_inactive: f64,
    pub position: usize,
    pub rp: f64,
    pub(crate) last_a: f64,
}

impl WdfLeaf for WdfSwitchedResistor {
    fn reflected(&mut self) -> f64 {
        0.0
    }
    fn set_incident(&mut self, a: f64) {
        self.last_a = a;
    }
    fn leaf_voltage(&self) -> f64 {
        self.last_a / 2.0
    }
    fn port_resistance(&self) -> f64 {
        self.rp
    }
    fn comp_id(&self) -> Option<&str> {
        Some(&self.switch_id)
    }
    fn type_tag(&self) -> &'static str {
        "switched_resistor"
    }

    fn set_control(&mut self, id: &str, value: f64) -> bool {
        if self.switch_id == id {
            let new_position = value as usize;
            self.position = new_position;
            self.rp = if self.path_index == new_position {
                self.r_active
            } else {
                self.r_inactive
            };
            true
        } else {
            false
        }
    }

    fn debug_info(&self) -> String {
        format!(
            "SwitchedResistor(switch=\"{}\", path={}, pos={}, Rp={:.1}Ω)",
            self.switch_id, self.path_index, self.position, self.rp
        )
    }
    fn clone_box(&self) -> Box<dyn WdfLeaf> {
        Box::new(self.clone())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// WdfUnitDelay
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Clone)]
pub struct WdfUnitDelay {
    pub rp: f64,
    pub state: f64,
    pub partner_state: f64,
}

impl WdfLeaf for WdfUnitDelay {
    fn reflected(&mut self) -> f64 {
        self.partner_state
    }
    fn set_incident(&mut self, a: f64) {
        self.state = a;
    }
    fn port_resistance(&self) -> f64 {
        self.rp
    }
    fn type_tag(&self) -> &'static str {
        "unit_delay"
    }
    fn reset(&mut self) {
        self.state = 0.0;
        self.partner_state = 0.0;
    }
    fn unit_delay_state(&self) -> Option<f64> {
        Some(self.state)
    }
    fn set_unit_delay_partner(&mut self, val: f64) -> bool {
        self.partner_state = val;
        true
    }
    fn debug_info(&self) -> String {
        format!(
            "UnitDelay(Rp={:.1}Ω, state={:.6}, partner={:.6})",
            self.rp, self.state, self.partner_state
        )
    }
    fn clone_box(&self) -> Box<dyn WdfLeaf> {
        Box::new(self.clone())
    }
}

impl WdfUnitDelay {
    pub fn get_state(&self) -> f64 {
        self.state
    }
    pub fn set_partner_state(&mut self, val: f64) {
        self.partner_state = val;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Downcast helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Helper to check if a leaf matches a comp_id with split-pot prefix matching.
pub fn leaf_matches_id(leaf: &dyn WdfLeaf, target_id: &str) -> bool {
    match leaf.comp_id() {
        Some(id) => {
            id == target_id
                || (id.starts_with(target_id) && id[target_id.len()..].starts_with("__"))
        }
        None => false,
    }
}
