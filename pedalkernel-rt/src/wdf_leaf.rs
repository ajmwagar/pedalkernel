//! WdfLeaf trait for dynamic WDF tree leaf nodes.
//!
//! Each leaf struct encapsulates the WDF scattering behavior of a single
//! circuit element. DynNode::Leaf wraps `Box<dyn WdfLeaf>` for polymorphism.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;

use crate::boundary_math::{
    OnePort, OnePortKind, OnePortState, PortTerminals, RuntimeOnePort, StateSlot,
};
use crate::pot_taper::PotTaper;
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
    fn reflected(&mut self) -> crate::Wave;
    /// Scatter-down: accept incident wave and update state.
    fn set_incident(&mut self, a: crate::Wave);
    /// Port resistance (Ω).
    fn port_resistance(&self) -> crate::Wave;

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
    fn leaf_voltage(&self) -> crate::Wave {
        0.0
    }

    // ── Control/mutation (not per-sample) ────────────────────────────────
    /// Generic control: pot position, LED drive, Vgs, switch position, etc.
    /// Returns true if this leaf handled the control.
    fn set_control(&mut self, _id: &str, _value: crate::Wave) -> bool {
        false
    }
    /// Direct resistance override. Returns true if handled.
    fn set_resistance(&mut self, _ohms: crate::Wave) -> bool {
        false
    }
    /// Voltage source update. Returns true if handled.
    fn set_voltage(&mut self, _v: crate::Wave) -> bool {
        false
    }
    /// Reactive element sample rate update.
    fn update_sample_rate(&mut self, _fs: crate::Wave) {}
    /// Reset state (caps/inductors clear to 0).
    fn reset(&mut self) {}

    // ── Introspection ────────────────────────────────────────────────────
    /// Whether this leaf's impedance can change at runtime (pots, modulated elements).
    /// Used by DynNode to skip recomputation of static subtrees.
    fn is_dynamic(&self) -> bool {
        false
    }
    fn is_reactive(&self) -> bool {
        false
    }

    /// Export this leaf's runtime one-port state, when it has explicit reactive memory.
    ///
    /// This is the bridge toward shared `Vec<OnePortState>` ownership. Some
    /// legacy WDF leaves still store wave-delay memory internally, so callers
    /// should treat this as the physical-state view of the current leaf state.
    fn one_port_state(&self) -> Option<OnePortState> {
        None
    }

    /// Import a shared one-port state into this leaf.
    fn set_one_port_state(&mut self, _state: OnePortState) -> bool {
        false
    }
    fn editable_info(&self) -> Option<(&'static str, crate::Wave)> {
        None
    }
    fn debug_info(&self) -> String;

    // ── Specialized queries ─────────────────────────────────────────────
    /// For UnitDelay: return the outgoing state.
    fn unit_delay_state(&self) -> Option<crate::Wave> {
        None
    }
    /// For UnitDelay: set the incoming partner state.
    fn set_unit_delay_partner(&mut self, _val: crate::Wave) -> bool {
        false
    }
    /// For Pot: return the current position (0.0..1.0).
    fn pot_position(&self) -> Option<crate::Wave> {
        None
    }
    /// For Pot: return the max resistance.
    fn pot_max_resistance(&self) -> Option<crate::Wave> {
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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WdfResistor {
    pub comp_id: Option<String>,
    pub rp: crate::Wave,
    pub last_a: crate::Wave,
}

impl WdfLeaf for WdfResistor {
    fn reflected(&mut self) -> crate::Wave {
        0.0
    }
    fn set_incident(&mut self, a: crate::Wave) {
        self.last_a = a;
    }
    fn port_resistance(&self) -> crate::Wave {
        self.rp
    }
    fn comp_id(&self) -> Option<&str> {
        self.comp_id.as_deref()
    }
    fn type_tag(&self) -> &'static str {
        "resistor"
    }
    fn leaf_voltage(&self) -> crate::Wave {
        self.last_a / 2.0
    }
    fn set_resistance(&mut self, ohms: crate::Wave) -> bool {
        self.rp = ohms;
        true
    }
    fn editable_info(&self) -> Option<(&'static str, crate::Wave)> {
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
// Runtime one-port WDF helpers
// ═══════════════════════════════════════════════════════════════════════════

fn runtime_one_port(kind: OnePortKind, state_slot: Option<StateSlot>) -> RuntimeOnePort<()> {
    RuntimeOnePort::new(OnePort::new(PortTerminals::grounded(), kind), state_slot)
}

// ═══════════════════════════════════════════════════════════════════════════
// WdfLeakyCapacitor
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WdfLeakyCapacitor {
    pub comp_id: Option<String>,
    pub capacitance: crate::Wave,
    pub rp: crate::Wave,
    pub state: crate::Wave,
    pub leakage_decay: crate::Wave,
    pub da_coef: Option<crate::Wave>,
    pub da_state: crate::Wave,
    pub da_rate: crate::Wave,
}

impl WdfLeaf for WdfLeakyCapacitor {
    fn reflected(&mut self) -> crate::Wave {
        let b = self.state;
        if let Some(da) = self.da_coef {
            b + da * (self.da_state - self.state)
        } else {
            b
        }
    }
    fn set_incident(&mut self, a: crate::Wave) {
        self.state = a * self.leakage_decay;
        if self.da_coef.is_some() {
            self.da_state += (self.state - self.da_state) * self.da_rate;
        }
    }
    fn port_resistance(&self) -> crate::Wave {
        self.rp
    }
    fn comp_id(&self) -> Option<&str> {
        self.comp_id.as_deref()
    }
    fn type_tag(&self) -> &'static str {
        "leaky_capacitor"
    }
    fn leaf_voltage(&self) -> crate::Wave {
        self.state
    }
    fn is_reactive(&self) -> bool {
        true
    }
    fn update_sample_rate(&mut self, fs: crate::Wave) {
        self.rp = 1.0 / (2.0 * fs * self.capacitance);
        if self.da_coef.is_some() {
            const DA_TIME_CONSTANT: crate::Wave = 0.5;
            self.da_rate = 1.0 / (fs * DA_TIME_CONSTANT);
        }
    }
    fn reset(&mut self) {
        self.state = 0.0;
        self.da_state = 0.0;
    }
    fn editable_info(&self) -> Option<(&'static str, crate::Wave)> {
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
// WdfVoltageSource
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WdfVoltageSource {
    pub voltage: crate::Wave,
    pub rp: crate::Wave,
    /// If true, set_voltage is a no-op (cathode bias behavior).
    pub is_cathode_bias: bool,
    /// Port name for named voltage ports (CV inputs).
    /// When set, this VS can be addressed by name via `set_voltage_by_port`.
    #[cfg_attr(feature = "serde", serde(default))]
    pub port_name: Option<alloc::string::String>,
}

impl WdfLeaf for WdfVoltageSource {
    fn reflected(&mut self) -> crate::Wave {
        2.0 * self.voltage
    }
    fn set_incident(&mut self, _a: crate::Wave) {}
    fn port_resistance(&self) -> crate::Wave {
        self.rp
    }
    fn comp_id(&self) -> Option<&str> {
        self.port_name.as_deref()
    }
    fn type_tag(&self) -> &'static str {
        if self.is_cathode_bias {
            "cathode_bias_source"
        } else {
            "voltage_source"
        }
    }
    fn set_resistance(&mut self, ohms: crate::Wave) -> bool {
        self.rp = ohms;
        true
    }
    fn set_voltage(&mut self, v: crate::Wave) -> bool {
        // Port-named VS leaves are set independently via set_voltage_by_port.
        // The global set_voltage() (main input) should not overwrite them.
        if self.port_name.is_some() {
            return false;
        }
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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WdfPot {
    pub comp_id: String,
    pub max_resistance: crate::Wave,
    pub position: crate::Wave,
    pub taper: PotTaper,
    pub rp: crate::Wave,
    pub last_a: crate::Wave,
    /// When true, this half uses the complement of the control position
    /// (1 - value). For a 3-terminal pot split into two halves, one half
    /// tracks position and the other tracks 1-position so their
    /// resistances always sum to max_R.
    pub complement: bool,
}

impl WdfLeaf for WdfPot {
    fn reflected(&mut self) -> crate::Wave {
        0.0
    }
    fn set_incident(&mut self, a: crate::Wave) {
        self.last_a = a;
    }
    fn port_resistance(&self) -> crate::Wave {
        self.rp
    }
    fn comp_id(&self) -> Option<&str> {
        Some(&self.comp_id)
    }
    fn type_tag(&self) -> &'static str {
        "pot"
    }
    fn is_dynamic(&self) -> bool {
        true
    }
    fn set_complement(&mut self) {
        self.complement = true;
        self.position = 1.0 - self.position;
        let tapered = self.taper.apply(self.position);
        self.rp = (tapered * self.max_resistance).max(1.0);
    }
    fn leaf_voltage(&self) -> crate::Wave {
        self.last_a / 2.0
    }

    fn set_control(&mut self, id: &str, value: crate::Wave) -> bool {
        // Match by exact ID or by prefix with __ separator (split pots)
        if self.comp_id == id
            || (self.comp_id.starts_with(id) && self.comp_id[id.len()..].starts_with("__"))
        {
            // Apply complement (for split pots: one half tracks 1-position).
            let effective = if self.complement { 1.0 - value } else { value };
            self.position = effective;
            let tapered_pos = self.taper.apply(effective);
            // Minimum resistance prevents gamma≈0 in series adaptors,
            // which causes numerically degenerate wave propagation.
            // Real pots have 10-100Ω contact resistance.
            self.rp = (tapered_pos * self.max_resistance).max(self.max_resistance * 0.001);
            true
        } else {
            false
        }
    }

    fn set_resistance(&mut self, ohms: crate::Wave) -> bool {
        self.rp = ohms;
        true
    }

    fn editable_info(&self) -> Option<(&'static str, crate::Wave)> {
        Some(("pot", self.rp))
    }

    fn pot_position(&self) -> Option<crate::Wave> {
        Some(self.position)
    }
    fn pot_max_resistance(&self) -> Option<crate::Wave> {
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
    pub fn get_resistance(&self) -> crate::Wave {
        self.rp
    }
    /// Read the pot's current position.
    pub fn get_position(&self) -> crate::Wave {
        self.position
    }
    /// Read the pot's max resistance.
    pub fn max_resistance(&self) -> crate::Wave {
        self.max_resistance
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// WdfPhotocoupler
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WdfPhotocoupler {
    pub comp_id: String,
    pub inner: PhotocouplerInner,
    #[allow(dead_code)]
    pub prev_resistance: crate::Wave,
}

impl WdfLeaf for WdfPhotocoupler {
    fn reflected(&mut self) -> crate::Wave {
        0.0
    }
    fn set_incident(&mut self, _a: crate::Wave) {}
    fn port_resistance(&self) -> crate::Wave {
        self.inner.port_resistance()
    }
    fn comp_id(&self) -> Option<&str> {
        Some(&self.comp_id)
    }
    fn type_tag(&self) -> &'static str {
        "photocoupler"
    }
    fn is_dynamic(&self) -> bool {
        true
    }

    fn set_control(&mut self, id: &str, value: crate::Wave) -> bool {
        if self.comp_id == id {
            self.inner.set_led_drive(value);
            true
        } else {
            false
        }
    }

    fn update_sample_rate(&mut self, fs: crate::Wave) {
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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WdfJfetVr {
    pub comp_id: String,
    pub inner: JfetVariableResistor,
    #[allow(dead_code)]
    pub prev_rds: crate::Wave,
}

impl WdfLeaf for WdfJfetVr {
    fn reflected(&mut self) -> crate::Wave {
        0.0
    }
    fn set_incident(&mut self, _a: crate::Wave) {}
    fn port_resistance(&self) -> crate::Wave {
        self.inner.port_resistance()
    }
    fn comp_id(&self) -> Option<&str> {
        Some(&self.comp_id)
    }
    fn type_tag(&self) -> &'static str {
        "jfet_vr"
    }
    fn is_dynamic(&self) -> bool {
        true
    }

    fn set_control(&mut self, id: &str, value: crate::Wave) -> bool {
        if self.comp_id == id {
            self.inner.set_vgs(value);
            true
        } else {
            false
        }
    }

    fn update_sample_rate(&mut self, fs: crate::Wave) {
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
    pub fn vgs(&self) -> crate::Wave {
        self.inner.vgs()
    }
    pub fn rds(&self) -> crate::Wave {
        self.inner.rds()
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// WdfSwitchedResistor
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WdfSwitchedResistor {
    pub switch_id: String,
    pub path_index: usize,
    #[allow(dead_code)]
    pub num_paths: usize,
    pub r_active: crate::Wave,
    pub r_inactive: crate::Wave,
    pub position: usize,
    pub rp: crate::Wave,
    pub last_a: crate::Wave,
}

impl WdfLeaf for WdfSwitchedResistor {
    fn reflected(&mut self) -> crate::Wave {
        0.0
    }
    fn set_incident(&mut self, a: crate::Wave) {
        self.last_a = a;
    }
    fn leaf_voltage(&self) -> crate::Wave {
        self.last_a / 2.0
    }
    fn port_resistance(&self) -> crate::Wave {
        self.rp
    }
    fn comp_id(&self) -> Option<&str> {
        Some(&self.switch_id)
    }
    fn type_tag(&self) -> &'static str {
        "switched_resistor"
    }
    fn is_dynamic(&self) -> bool {
        true
    }

    fn set_control(&mut self, id: &str, value: crate::Wave) -> bool {
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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WdfUnitDelay {
    pub rp: crate::Wave,
    pub state: crate::Wave,
    pub partner_state: crate::Wave,
}

impl WdfLeaf for WdfUnitDelay {
    fn reflected(&mut self) -> crate::Wave {
        self.partner_state
    }
    fn set_incident(&mut self, a: crate::Wave) {
        self.state = a;
    }
    fn port_resistance(&self) -> crate::Wave {
        self.rp
    }
    fn type_tag(&self) -> &'static str {
        "unit_delay"
    }
    fn reset(&mut self) {
        self.state = 0.0;
        self.partner_state = 0.0;
    }
    fn unit_delay_state(&self) -> Option<crate::Wave> {
        Some(self.state)
    }
    fn set_unit_delay_partner(&mut self, val: crate::Wave) -> bool {
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
    pub fn get_state(&self) -> crate::Wave {
        self.state
    }
    pub fn set_partner_state(&mut self, val: crate::Wave) {
        self.partner_state = val;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Downcast helpers
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
// LeafKind — concrete enum replacing Box<dyn WdfLeaf>
// ═══════════════════════════════════════════════════════════════════════════

/// Concrete leaf type — replaces Box<dyn WdfLeaf> for serialization support.
#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum LeafKind {
    Resistor(WdfResistor),
    OnePort {
        comp_id: Option<String>,
        runtime: RuntimeOnePort<()>,
        sample_rate: crate::Wave,
    },
    LeakyCapacitor(WdfLeakyCapacitor),
    VoltageSource(WdfVoltageSource),
    Pot(WdfPot),
    Photocoupler(WdfPhotocoupler),
    JfetVr(WdfJfetVr),
    SwitchedResistor(WdfSwitchedResistor),
    UnitDelay(WdfUnitDelay),
}

impl LeafKind {
    pub fn one_port(comp_id: Option<String>, kind: OnePortKind, sample_rate: crate::Wave) -> Self {
        Self::one_port_with_slot(comp_id, kind, sample_rate, None)
    }

    pub fn one_port_with_slot(
        comp_id: Option<String>,
        kind: OnePortKind,
        sample_rate: crate::Wave,
        state_slot: Option<StateSlot>,
    ) -> Self {
        let runtime = runtime_one_port(kind, state_slot);
        Self::OnePort {
            comp_id,
            runtime,
            sample_rate,
        }
    }

    pub fn capacitor(
        comp_id: Option<String>,
        capacitance: crate::Wave,
        sample_rate: crate::Wave,
    ) -> Self {
        Self::one_port(comp_id, OnePortKind::Capacitor(capacitance), sample_rate)
    }

    pub fn capacitor_from_rp(
        comp_id: Option<String>,
        capacitance: crate::Wave,
        rp: crate::Wave,
    ) -> Self {
        let kind = OnePortKind::Capacitor(capacitance);
        Self::one_port(comp_id, kind, kind.sample_rate_for_rp(rp))
    }

    pub fn inductor(
        comp_id: Option<String>,
        inductance: crate::Wave,
        sample_rate: crate::Wave,
    ) -> Self {
        Self::one_port(comp_id, OnePortKind::Inductor(inductance), sample_rate)
    }

    pub fn inductor_from_rp(
        comp_id: Option<String>,
        inductance: crate::Wave,
        rp: crate::Wave,
    ) -> Self {
        let kind = OnePortKind::Inductor(inductance);
        Self::one_port(comp_id, kind, kind.sample_rate_for_rp(rp))
    }

    pub fn one_port_value(&self) -> Option<crate::Wave> {
        match self {
            Self::OnePort { runtime, .. } => Some(runtime.physical_value()),
            _ => None,
        }
    }

    pub fn one_port_runtime(&self) -> Option<&RuntimeOnePort<()>> {
        match self {
            Self::OnePort { runtime, .. } => Some(runtime),
            _ => None,
        }
    }

    pub fn one_port_runtime_mut(&mut self) -> Option<&mut RuntimeOnePort<()>> {
        match self {
            Self::OnePort { runtime, .. } => Some(runtime),
            _ => None,
        }
    }
}

macro_rules! delegate_leaf {
    ($self:ident, $method:ident $(, $arg:expr)*) => {
        match $self {
            LeafKind::Resistor(x) => x.$method($($arg),*),
            LeafKind::OnePort { .. } => unreachable!(),
            LeafKind::LeakyCapacitor(x) => x.$method($($arg),*),
            LeafKind::VoltageSource(x) => x.$method($($arg),*),
            LeafKind::Pot(x) => x.$method($($arg),*),
            LeafKind::Photocoupler(x) => x.$method($($arg),*),
            LeafKind::JfetVr(x) => x.$method($($arg),*),
            LeafKind::SwitchedResistor(x) => x.$method($($arg),*),
            LeafKind::UnitDelay(x) => x.$method($($arg),*),
        }
    };
}

impl WdfLeaf for LeafKind {
    fn reflected(&mut self) -> crate::Wave {
        match self {
            LeafKind::OnePort { .. } => 0.0,
            _ => delegate_leaf!(self, reflected),
        }
    }
    fn set_incident(&mut self, a: crate::Wave) {
        match self {
            LeafKind::OnePort { .. } => {}
            _ => delegate_leaf!(self, set_incident, a),
        }
    }
    fn port_resistance(&self) -> crate::Wave {
        match self {
            LeafKind::OnePort {
                runtime,
                sample_rate,
                ..
            } => runtime.rp(*sample_rate),
            _ => delegate_leaf!(self, port_resistance),
        }
    }
    fn comp_id(&self) -> Option<&str> {
        match self {
            LeafKind::OnePort { comp_id, .. } => comp_id.as_deref(),
            _ => delegate_leaf!(self, comp_id),
        }
    }
    fn type_tag(&self) -> &'static str {
        match self {
            LeafKind::OnePort { runtime, .. } => runtime.type_tag(),
            _ => delegate_leaf!(self, type_tag),
        }
    }
    fn set_complement(&mut self) {
        match self {
            LeafKind::OnePort { .. } => {}
            _ => delegate_leaf!(self, set_complement),
        }
    }
    fn leaf_voltage(&self) -> crate::Wave {
        match self {
            LeafKind::OnePort { .. } => 0.0,
            _ => delegate_leaf!(self, leaf_voltage),
        }
    }
    fn set_control(&mut self, id: &str, value: crate::Wave) -> bool {
        match self {
            LeafKind::OnePort {
                comp_id, runtime, ..
            } => {
                if comp_id.as_deref() == Some(id) {
                    match runtime.spec.kind {
                        OnePortKind::Capacitor(_) => {
                            runtime.spec.kind = OnePortKind::Capacitor(value);
                            true
                        }
                        OnePortKind::Inductor(_) => {
                            runtime.spec.kind = OnePortKind::Inductor(value);
                            true
                        }
                        OnePortKind::Resistor(_) => {
                            runtime.spec.kind = OnePortKind::Resistor(value);
                            true
                        }
                    }
                } else {
                    false
                }
            }
            _ => delegate_leaf!(self, set_control, id, value),
        }
    }
    fn set_resistance(&mut self, ohms: crate::Wave) -> bool {
        match self {
            LeafKind::OnePort { runtime, .. } => match runtime.spec.kind {
                OnePortKind::Resistor(_) => {
                    runtime.spec.kind = OnePortKind::Resistor(ohms);
                    true
                }
                _ => false,
            },
            _ => delegate_leaf!(self, set_resistance, ohms),
        }
    }
    fn set_voltage(&mut self, v: crate::Wave) -> bool {
        match self {
            LeafKind::OnePort { .. } => false,
            _ => delegate_leaf!(self, set_voltage, v),
        }
    }
    fn update_sample_rate(&mut self, fs: crate::Wave) {
        match self {
            LeafKind::OnePort { sample_rate, .. } => *sample_rate = fs,
            _ => delegate_leaf!(self, update_sample_rate, fs),
        }
    }
    fn reset(&mut self) {
        match self {
            LeafKind::OnePort { .. } => {}
            _ => delegate_leaf!(self, reset),
        }
    }
    fn is_dynamic(&self) -> bool {
        match self {
            LeafKind::OnePort { .. } => false,
            _ => delegate_leaf!(self, is_dynamic),
        }
    }
    fn is_reactive(&self) -> bool {
        match self {
            LeafKind::OnePort { runtime, .. } => runtime.spec.kind.is_stateful(),
            _ => delegate_leaf!(self, is_reactive),
        }
    }
    fn one_port_state(&self) -> Option<OnePortState> {
        match self {
            LeafKind::OnePort { .. } => None,
            _ => delegate_leaf!(self, one_port_state),
        }
    }
    fn set_one_port_state(&mut self, state: OnePortState) -> bool {
        match self {
            LeafKind::OnePort { .. } => false,
            _ => delegate_leaf!(self, set_one_port_state, state),
        }
    }
    fn editable_info(&self) -> Option<(&'static str, crate::Wave)> {
        match self {
            LeafKind::OnePort {
                comp_id, runtime, ..
            } => comp_id
                .as_ref()
                .map(|_| (runtime.type_tag(), runtime.physical_value())),
            _ => delegate_leaf!(self, editable_info),
        }
    }
    fn debug_info(&self) -> String {
        match self {
            LeafKind::OnePort {
                comp_id,
                runtime,
                sample_rate,
            } => {
                let id = comp_id.as_deref().unwrap_or("?");
                let value = runtime.physical_value();
                match runtime.spec.kind {
                    OnePortKind::Resistor(_) => {
                        format!(
                            "OnePort::Resistor(id=\"{id}\", Rp={:.1}Ω)",
                            runtime.rp(*sample_rate)
                        )
                    }
                    OnePortKind::Capacitor(_) => format!(
                        "OnePort::Capacitor(id=\"{id}\", C={value:.3e}F, Rp={:.1}Ω)",
                        runtime.rp(*sample_rate)
                    ),
                    OnePortKind::Inductor(_) => format!(
                        "OnePort::Inductor(id=\"{id}\", L={value:.3e}H, Rp={:.1}Ω)",
                        runtime.rp(*sample_rate)
                    ),
                }
            }
            _ => delegate_leaf!(self, debug_info),
        }
    }
    fn unit_delay_state(&self) -> Option<crate::Wave> {
        match self {
            LeafKind::OnePort { .. } => None,
            _ => delegate_leaf!(self, unit_delay_state),
        }
    }
    fn set_unit_delay_partner(&mut self, val: crate::Wave) -> bool {
        match self {
            LeafKind::OnePort { .. } => false,
            _ => delegate_leaf!(self, set_unit_delay_partner, val),
        }
    }
    fn pot_position(&self) -> Option<crate::Wave> {
        match self {
            LeafKind::OnePort { .. } => None,
            _ => delegate_leaf!(self, pot_position),
        }
    }
    fn pot_max_resistance(&self) -> Option<crate::Wave> {
        match self {
            LeafKind::OnePort { .. } => None,
            _ => delegate_leaf!(self, pot_max_resistance),
        }
    }
    fn is_complement(&self) -> bool {
        match self {
            LeafKind::OnePort { .. } => false,
            _ => delegate_leaf!(self, is_complement),
        }
    }
    fn clone_box(&self) -> Box<dyn WdfLeaf> {
        Box::new(self.clone())
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dyn_node::DynNode;

    fn runtime_one_port_fixture(
        node: &DynNode,
    ) -> (&RuntimeOnePort<()>, &crate::boundary_math::RuntimeState) {
        match node {
            DynNode::Runtime {
                node: inner,
                runtime_state,
            } => match &**inner {
                DynNode::Leaf(LeafKind::OnePort { runtime, .. }) => (runtime, runtime_state),
                _ => panic!("runtime wrapper must own a one-port leaf"),
            },
            _ => panic!("reactive one-port must be wrapped with runtime state"),
        }
    }

    fn runtime_one_port_fixture_mut(
        node: &mut DynNode,
    ) -> (&RuntimeOnePort<()>, &mut crate::boundary_math::RuntimeState) {
        match node {
            DynNode::Runtime {
                node: inner,
                runtime_state,
            } => match &**inner {
                DynNode::Leaf(LeafKind::OnePort { runtime, .. }) => (runtime, runtime_state),
                _ => panic!("runtime wrapper must own a one-port leaf"),
            },
            _ => panic!("reactive one-port must be wrapped with runtime state"),
        }
    }

    #[test]
    fn capacitor_exports_and_imports_one_port_voltage_state() {
        let mut cap = DynNode::Capacitor(Some(String::from("C1")), 100e-9, 1_000.0);

        cap.set_incident(0.6);
        let (runtime, runtime_state) = runtime_one_port_fixture(&cap);
        let slot = runtime.state_slot().unwrap().0;
        assert_eq!(
            runtime_state.states[slot],
            Some(OnePortState::CapacitorVoltage(0.3)).unwrap()
        );
        assert_eq!(runtime_state.wave_cache[slot].wave_state, 0.6);
        assert_eq!(cap.leaf_voltage("C1"), Some(0.3));

        let (runtime, runtime_state) = runtime_one_port_fixture_mut(&mut cap);
        assert!(runtime.wdf_set_one_port_state(OnePortState::CapacitorVoltage(0.5), runtime_state));
        assert_eq!(
            runtime.wdf_one_port_state(runtime_state),
            Some(OnePortState::CapacitorVoltage(0.5))
        );
        assert!(!runtime
            .wdf_set_one_port_state(OnePortState::InductorScaledCurrent(0.1), runtime_state));
    }

    #[test]
    fn inductor_exports_and_imports_scaled_current_state() {
        let mut inductor = DynNode::Inductor(Some(String::from("L1")), 1e-3, 96.0);

        inductor.set_incident(0.25);
        let (runtime, runtime_state) = runtime_one_port_fixture(&inductor);
        let slot = runtime.state_slot().unwrap().0;
        assert_eq!(
            runtime_state.states[slot],
            OnePortState::InductorScaledCurrent(0.25)
        );
        assert_eq!(runtime_state.wave_cache[slot].wave_state, 0.25);
        assert_eq!(inductor.leaf_voltage("L1"), Some(0.125));

        let (runtime, runtime_state) = runtime_one_port_fixture_mut(&mut inductor);
        assert!(runtime
            .wdf_set_one_port_state(OnePortState::InductorScaledCurrent(-0.4), runtime_state));
        assert_eq!(
            runtime.wdf_one_port_state(runtime_state),
            Some(OnePortState::InductorScaledCurrent(-0.4))
        );
        assert!(!runtime.wdf_set_one_port_state(OnePortState::CapacitorVoltage(0.2), runtime_state));
    }

    #[test]
    fn resistor_has_no_one_port_runtime_state() {
        let resistor = WdfResistor {
            comp_id: Some(String::from("R1")),
            rp: 1_000.0,
            last_a: 0.0,
        };

        assert_eq!(resistor.one_port_state(), None);
    }
}
