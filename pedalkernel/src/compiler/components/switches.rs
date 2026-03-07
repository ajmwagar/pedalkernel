//! Switch component structs: Switch, RotarySwitch, TriggerInputComp.

use crate::compiler::component::{Component, ControlParam, ControlParamKind, GraphRole, PinConfig, StampResult};
use crate::compiler::validate::Severity;
use crate::tree::MnaSystem;

use super::impl_component_dyn;

// ═══════════════════════════════════════════════════════════════════════════
// Switch
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Switch {
    pub positions: usize,
}

impl Component for Switch {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "switch" }

    fn is_passive(&self) -> bool { false }

    fn is_control_only(&self) -> bool { true }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &[],
            aliases: &[],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Virtual
    }

    fn stamp_mna(
        &self,
        _comp_id: &str,
        _n1: Option<usize>,
        _n2: Option<usize>,
        _mna: &mut MnaSystem,
        _sample_rate: f64,
    ) -> StampResult {
        StampResult::Skip
    }

    fn validate_values(&self, comp_id: &str) -> Vec<(Severity, String)> {
        let mut w = Vec::new();
        if self.positions < 2 {
            w.push((
                Severity::Warning,
                format!(
                    "Switch '{}' has {} position(s) \u{2014} a switch needs at least 2",
                    comp_id, self.positions
                ),
            ));
        }
        w
    }

    fn controls(&self) -> Vec<ControlParam> {
        vec![ControlParam {
            name: "position",
            kind: ControlParamKind::SwitchPosition { num_positions: self.positions },
        }]
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        match self.positions {
            2 => ("Switch:SW_SPDT", "SW"),
            3 => ("Switch:SW_Rotary4", "SW"),
            _ => ("Switch:SW_Rotary4", "SW"),
        }
    }

    fn symbol_name(&self) -> &'static str { "switch" }
    fn layout_class(&self) -> &'static str { "switch" }
}

// ═══════════════════════════════════════════════════════════════════════════
// RotarySwitch
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct RotarySwitch {
    pub linked_ids: Vec<String>,
}

impl Component for RotarySwitch {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "rotary switch" }

    fn is_passive(&self) -> bool { false }

    fn is_control_only(&self) -> bool { true }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &[],
            aliases: &[],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Virtual
    }

    fn stamp_mna(
        &self,
        _comp_id: &str,
        _n1: Option<usize>,
        _n2: Option<usize>,
        _mna: &mut MnaSystem,
        _sample_rate: f64,
    ) -> StampResult {
        StampResult::Skip
    }

    fn controls(&self) -> Vec<ControlParam> {
        vec![ControlParam {
            name: "position",
            kind: ControlParamKind::SwitchPosition { num_positions: self.linked_ids.len() },
        }]
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        let positions = self.linked_ids.len().max(2);
        match positions {
            2..=4 => ("Switch:SW_Rotary4", "SW"),
            5..=6 => ("Switch:SW_Rotary6", "SW"),
            7..=8 => ("Switch:SW_Rotary8", "SW"),
            _ => ("Switch:SW_Rotary12", "SW"),
        }
    }

    fn symbol_name(&self) -> &'static str { "switch" }
    fn layout_class(&self) -> &'static str { "rotary_switch" }
}

// ═══════════════════════════════════════════════════════════════════════════
// TriggerInput
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct TriggerInputComp;

impl Component for TriggerInputComp {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "trigger_input" }

    fn is_passive(&self) -> bool { false }

    fn is_control_only(&self) -> bool { true }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["out"],
            aliases: &[],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Virtual
    }

    fn stamp_mna(
        &self,
        _comp_id: &str,
        _n1: Option<usize>,
        _n2: Option<usize>,
        _mna: &mut MnaSystem,
        _sample_rate: f64,
    ) -> StampResult {
        StampResult::Skip
    }

    fn controls(&self) -> Vec<ControlParam> {
        vec![ControlParam { name: "trigger", kind: ControlParamKind::Trigger }]
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Connector:TestPoint", "TP")
    }

    fn symbol_name(&self) -> &'static str { "switch" }
    fn layout_class(&self) -> &'static str { "trigger_input" }

    fn is_trigger(&self) -> bool { true }
}
