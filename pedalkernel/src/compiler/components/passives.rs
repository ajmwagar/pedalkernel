//! Passive component structs: Resistor, Capacitor, Inductor, Potentiometer,
//! Tempco, CapSwitched, InductorSwitched, ResistorSwitched.

use crate::compiler::component::{
    Component, ComponentEdge, ControlParam, ControlParamKind, EdgeKind, GraphRole, PinConfig,
    PinDirection, StampResult, OPEN_CIRCUIT_R,
};
use crate::compiler::dyn_node::DynNode;
use crate::compiler::validate::Severity;
use crate::dsl::{CapConfig, PotTaper};
use crate::tree::MnaSystem;

use super::impl_component_dyn;

// ═══════════════════════════════════════════════════════════════════════════
// Resistor
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Resistor {
    pub value: f64,
}

impl Component for Resistor {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "resistor" }

    fn is_passive(&self) -> bool { true }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["a", "b"],
            aliases: &[],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Edge { pin_a: "a", pin_b: "b" }
    }

    fn stamp_mna(
        &self,
        _comp_id: &str,
        n1: Option<usize>,
        n2: Option<usize>,
        mna: &mut MnaSystem,
        _sample_rate: f64,
    ) -> StampResult {
        mna.stamp_resistor(n1, n2, self.value);
        StampResult::Stamped
    }

    fn make_leaf(&self, comp_id: &str, _sample_rate: f64) -> Option<DynNode> {
        if self.value.is_infinite() {
            Some(DynNode::Resistor(Some(comp_id.to_string()), OPEN_CIRCUIT_R))
        } else {
            Some(DynNode::Resistor(Some(comp_id.to_string()), self.value))
        }
    }

    fn edges(&self) -> Vec<ComponentEdge> {
        vec![ComponentEdge { pin_a: "a", pin_b: "b", kind: EdgeKind::Linear, port_group: None }]
    }

    fn resistance(&self) -> Option<f64> { Some(self.value) }

    fn validate_values(&self, comp_id: &str) -> Vec<(Severity, String)> {
        let mut w = Vec::new();
        if self.value <= 0.0 {
            w.push((
                Severity::Error,
                format!("Resistor '{}' has non-positive value {:.2} \u{2126}", comp_id, self.value),
            ));
        } else if self.value < 1.0 {
            w.push((
                Severity::Info,
                format!(
                    "Resistor '{}' is {:.2} \u{2126} \u{2014} extremely low, did you mean {}k?",
                    comp_id, self.value, self.value
                ),
            ));
        } else if self.value > 100e6 {
            w.push((
                Severity::Info,
                format!(
                    "Resistor '{}' is {:.1} M\u{2126} \u{2014} unusually high",
                    comp_id,
                    self.value / 1e6
                ),
            ));
        }
        w
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) { ("Device:R", "R") }

    fn symbol_name(&self) -> &'static str { "resistor" }
    fn layout_class(&self) -> &'static str { "resistor" }
    fn display_value(&self) -> Option<String> { Some(crate::kicad::format_eng(self.value, "Ω")) }
}

// ═══════════════════════════════════════════════════════════════════════════
// Capacitor
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Capacitor {
    pub config: CapConfig,
}

impl Component for Capacitor {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "capacitor" }

    fn is_passive(&self) -> bool { true }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["a", "b"],
            aliases: &[],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Edge { pin_a: "a", pin_b: "b" }
    }

    fn stamp_mna(
        &self,
        comp_id: &str,
        _n1: Option<usize>,
        _n2: Option<usize>,
        _mna: &mut MnaSystem,
        sample_rate: f64,
    ) -> StampResult {
        let rp = 1.0 / (2.0 * sample_rate * self.config.value);
        StampResult::Reactive {
            dyn_node: DynNode::Capacitor(Some(comp_id.to_string()), self.config.value, rp),
            rp,
        }
    }

    fn make_leaf(&self, comp_id: &str, sample_rate: f64) -> Option<DynNode> {
        if self.config.leakage.is_some() || self.config.da.is_some() {
            let rp = 1.0 / (2.0 * sample_rate * self.config.value);
            let leakage_decay = if let Some(r_leak) = self.config.leakage {
                let tau = r_leak * self.config.value;
                let dt = 1.0 / sample_rate;
                (-dt / tau).exp()
            } else {
                1.0
            };
            const DA_TIME_CONSTANT: f64 = 0.5;
            let da_rate = if self.config.da.is_some() {
                1.0 / (sample_rate * DA_TIME_CONSTANT)
            } else {
                0.0
            };
            Some(DynNode::LeakyCapacitor(Some(comp_id.to_string()), self.config.value, rp, leakage_decay, self.config.da, 0.0, da_rate))
        } else {
            Some(DynNode::Capacitor(Some(comp_id.to_string()), self.config.value, 1.0 / (2.0 * sample_rate * self.config.value)))
        }
    }

    fn edges(&self) -> Vec<ComponentEdge> {
        vec![ComponentEdge { pin_a: "a", pin_b: "b", kind: EdgeKind::Reactive, port_group: None }]
    }

    fn capacitance(&self) -> Option<f64> { Some(self.config.value) }

    fn validate_values(&self, comp_id: &str) -> Vec<(Severity, String)> {
        let mut w = Vec::new();
        if self.config.value <= 0.0 {
            w.push((
                Severity::Error,
                format!(
                    "Capacitor '{}' has non-positive value {:.2e} F",
                    comp_id, self.config.value
                ),
            ));
        } else if self.config.value > 1.0 {
            w.push((
                Severity::Warning,
                format!(
                    "Capacitor '{}' is {:.2} F \u{2014} over 1 farad, did you forget a unit suffix (n, u, p)?",
                    comp_id, self.config.value
                ),
            ));
        }
        if let Some(leak) = self.config.leakage {
            if leak <= 0.0 {
                w.push((
                    Severity::Error,
                    format!(
                        "Capacitor '{}' has non-positive leakage resistance {:.2} \u{2126}",
                        comp_id, leak
                    ),
                ));
            }
        }
        if let Some(da) = self.config.da {
            if !(0.0..=1.0).contains(&da) {
                w.push((
                    Severity::Warning,
                    format!(
                        "Capacitor '{}' has dielectric absorption {:.3} \u{2014} should be 0.0 to 1.0",
                        comp_id, da
                    ),
                ));
            }
        }
        w
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) { ("Device:C", "C") }

    fn symbol_name(&self) -> &'static str { "capacitor" }
    fn layout_class(&self) -> &'static str { "capacitor" }
    fn display_value(&self) -> Option<String> { Some(crate::kicad::format_eng(self.config.value, "F")) }
}

// ═══════════════════════════════════════════════════════════════════════════
// Inductor
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Inductor {
    pub value: f64,
}

impl Component for Inductor {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "inductor" }

    fn is_passive(&self) -> bool { true }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["a", "b"],
            aliases: &[],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Edge { pin_a: "a", pin_b: "b" }
    }

    fn stamp_mna(
        &self,
        comp_id: &str,
        _n1: Option<usize>,
        _n2: Option<usize>,
        _mna: &mut MnaSystem,
        sample_rate: f64,
    ) -> StampResult {
        let rp = 2.0 * sample_rate * self.value;
        StampResult::Reactive {
            dyn_node: DynNode::Inductor(Some(comp_id.to_string()), self.value, rp),
            rp,
        }
    }

    fn make_leaf(&self, comp_id: &str, sample_rate: f64) -> Option<DynNode> {
        Some(DynNode::Inductor(Some(comp_id.to_string()), self.value, 2.0 * sample_rate * self.value))
    }

    fn edges(&self) -> Vec<ComponentEdge> {
        vec![ComponentEdge { pin_a: "a", pin_b: "b", kind: EdgeKind::Reactive, port_group: None }]
    }

    fn inductance(&self) -> Option<f64> { Some(self.value) }

    fn validate_values(&self, comp_id: &str) -> Vec<(Severity, String)> {
        let mut w = Vec::new();
        if self.value <= 0.0 {
            w.push((
                Severity::Error,
                format!(
                    "Inductor '{}' has non-positive value {:.2e} H",
                    comp_id, self.value
                ),
            ));
        }
        w
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) { ("Device:L", "L") }

    fn symbol_name(&self) -> &'static str { "inductor" }
    fn layout_class(&self) -> &'static str { "inductor" }
    fn display_value(&self) -> Option<String> { Some(crate::kicad::format_eng(self.value, "H")) }
}

// ═══════════════════════════════════════════════════════════════════════════
// Potentiometer
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Potentiometer {
    pub max_r: f64,
    pub taper: PotTaper,
}

impl Component for Potentiometer {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "potentiometer" }

    fn is_passive(&self) -> bool { true }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["a", "b", "w", "wiper", "position"],
            aliases: &[],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Pot
    }

    fn stamp_mna(
        &self,
        comp_id: &str,
        n1: Option<usize>,
        n2: Option<usize>,
        mna: &mut MnaSystem,
        _sample_rate: f64,
    ) -> StampResult {
        let initial_pos = 0.5;
        let tapered_pos = self.taper.apply(initial_pos);
        let r = (tapered_pos * self.max_r).max(1.0);
        mna.stamp_resistor(n1, n2, r);
        StampResult::Pot {
            dyn_node: DynNode::Pot(comp_id.to_string(), self.max_r, initial_pos, self.taper),
            initial_conductance: 1.0 / r,
        }
    }

    fn make_leaf(&self, comp_id: &str, _sample_rate: f64) -> Option<DynNode> {
        let initial_pos = 0.5;
        let tapered_pos = self.taper.apply(initial_pos);
        Some(DynNode::Pot(comp_id.to_string(), self.max_r, initial_pos, self.taper))
    }

    fn is_variable(&self) -> bool { true }

    fn edges(&self) -> Vec<ComponentEdge> {
        vec![ComponentEdge { pin_a: "a", pin_b: "b", kind: EdgeKind::Linear, port_group: None }]
    }

    fn controls(&self) -> Vec<ControlParam> {
        vec![ControlParam { name: "position", kind: ControlParamKind::PotPosition }]
    }

    fn resistance(&self) -> Option<f64> { Some(self.max_r) }

    fn validate_values(&self, comp_id: &str) -> Vec<(Severity, String)> {
        let mut w = Vec::new();
        if self.max_r <= 0.0 {
            w.push((
                Severity::Error,
                format!(
                    "Potentiometer '{}' has non-positive max resistance {:.2} \u{2126}",
                    comp_id, self.max_r
                ),
            ));
        }
        w
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) { ("Device:R_Potentiometer", "RV") }

    fn symbol_name(&self) -> &'static str { "pot" }
    fn layout_class(&self) -> &'static str { "pot" }
    fn display_value(&self) -> Option<String> { Some(crate::kicad::format_eng(self.max_r, "\u{2126}")) }

    fn signal_adjacencies(&self) -> Vec<(&'static str, &'static str)> {
        vec![("a", "b"), ("a", "wiper"), ("wiper", "b"), ("a", "w"), ("w", "b")]
    }

    fn pin_direction(&self, pin: &str) -> PinDirection {
        match pin {
            "a" => PinDirection::Input,
            "wiper" | "b" | "w" => PinDirection::Output,
            _ => PinDirection::Bidirectional,
        }
    }

    fn is_pot(&self) -> bool { true }
    fn pot_taper(&self) -> Option<crate::dsl::PotTaper> { Some(self.taper) }
}

// ═══════════════════════════════════════════════════════════════════════════
// Tempco
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Tempco {
    pub resistance: f64,
    pub ppm: f64,
}

impl Component for Tempco {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "tempco resistor" }

    fn is_passive(&self) -> bool { true }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["a", "b"],
            aliases: &[],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Edge { pin_a: "a", pin_b: "b" }
    }

    fn stamp_mna(
        &self,
        _comp_id: &str,
        n1: Option<usize>,
        n2: Option<usize>,
        mna: &mut MnaSystem,
        _sample_rate: f64,
    ) -> StampResult {
        mna.stamp_resistor(n1, n2, self.resistance);
        StampResult::Stamped
    }

    fn make_leaf(&self, comp_id: &str, _sample_rate: f64) -> Option<DynNode> {
        Some(DynNode::Resistor(Some(comp_id.to_string()), self.resistance))
    }

    fn edges(&self) -> Vec<ComponentEdge> {
        vec![ComponentEdge { pin_a: "a", pin_b: "b", kind: EdgeKind::Linear, port_group: None }]
    }

    fn resistance(&self) -> Option<f64> { Some(self.resistance) }

    fn footprint_ref(&self) -> (&'static str, &'static str) { ("Device:R", "RT") }

    fn symbol_name(&self) -> &'static str { "resistor" }
    fn layout_class(&self) -> &'static str { "tempco" }
    fn display_value(&self) -> Option<String> { None }
}

// ═══════════════════════════════════════════════════════════════════════════
// CapSwitched
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct CapSwitched {
    pub values: Vec<f64>,
}

impl Component for CapSwitched {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "switched capacitor" }

    fn is_passive(&self) -> bool { true }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["a", "b"],
            aliases: &[],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Edge { pin_a: "a", pin_b: "b" }
    }

    fn stamp_mna(
        &self,
        comp_id: &str,
        _n1: Option<usize>,
        _n2: Option<usize>,
        _mna: &mut MnaSystem,
        sample_rate: f64,
    ) -> StampResult {
        if let Some(&c) = self.values.first() {
            if c.is_finite() && c > 0.0 {
                let rp = 1.0 / (2.0 * sample_rate * c);
                return StampResult::Reactive {
                    dyn_node: DynNode::Capacitor(Some(comp_id.to_string()), c, rp),
                    rp,
                };
            }
        }
        StampResult::Skip
    }

    fn edges(&self) -> Vec<ComponentEdge> {
        vec![ComponentEdge { pin_a: "a", pin_b: "b", kind: EdgeKind::Reactive, port_group: None }]
    }

    fn capacitance(&self) -> Option<f64> { self.values.first().copied() }

    fn footprint_ref(&self) -> (&'static str, &'static str) { ("Device:C", "C") }

    fn symbol_name(&self) -> &'static str { "capacitor" }
    fn layout_class(&self) -> &'static str { "cap_switched" }
}

// ═══════════════════════════════════════════════════════════════════════════
// InductorSwitched
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct InductorSwitched {
    pub values: Vec<f64>,
}

impl Component for InductorSwitched {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "switched inductor" }

    fn is_passive(&self) -> bool { true }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["a", "b"],
            aliases: &[],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Edge { pin_a: "a", pin_b: "b" }
    }

    fn stamp_mna(
        &self,
        comp_id: &str,
        _n1: Option<usize>,
        _n2: Option<usize>,
        _mna: &mut MnaSystem,
        sample_rate: f64,
    ) -> StampResult {
        if let Some(&l) = self.values.first() {
            if l.is_finite() && l > 0.0 {
                let rp = 2.0 * sample_rate * l;
                return StampResult::Reactive {
                    dyn_node: DynNode::Inductor(Some(comp_id.to_string()), l, rp),
                    rp,
                };
            }
        }
        StampResult::Skip
    }

    fn edges(&self) -> Vec<ComponentEdge> {
        vec![ComponentEdge { pin_a: "a", pin_b: "b", kind: EdgeKind::Reactive, port_group: None }]
    }

    fn inductance(&self) -> Option<f64> { self.values.first().copied() }

    fn footprint_ref(&self) -> (&'static str, &'static str) { ("Device:L", "L") }

    fn symbol_name(&self) -> &'static str { "inductor" }
    fn layout_class(&self) -> &'static str { "inductor_switched" }
}

// ═══════════════════════════════════════════════════════════════════════════
// ResistorSwitched
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct ResistorSwitched {
    pub values: Vec<f64>,
}

impl Component for ResistorSwitched {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "switched resistor" }

    fn is_passive(&self) -> bool { true }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["a", "b"],
            aliases: &[],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Edge { pin_a: "a", pin_b: "b" }
    }

    fn stamp_mna(
        &self,
        _comp_id: &str,
        n1: Option<usize>,
        n2: Option<usize>,
        mna: &mut MnaSystem,
        _sample_rate: f64,
    ) -> StampResult {
        if let Some(&r) = self.values.first() {
            if r.is_finite() && r > 0.0 {
                mna.stamp_resistor(n1, n2, r);
            }
        }
        StampResult::Stamped
    }

    fn edges(&self) -> Vec<ComponentEdge> {
        vec![ComponentEdge { pin_a: "a", pin_b: "b", kind: EdgeKind::Linear, port_group: None }]
    }

    fn resistance(&self) -> Option<f64> { self.values.first().copied() }

    fn footprint_ref(&self) -> (&'static str, &'static str) { ("Device:R", "R") }

    fn symbol_name(&self) -> &'static str { "resistor" }
    fn layout_class(&self) -> &'static str { "resistor_switched" }
}
