//! Delay component structs: Bbd, DelayLine, Tap.

use crate::compiler::component::{
    Component, ComponentEdge, ControlParam, ControlParamKind, EdgeKind, GraphRole, ModulationSink,
    ModulationSinkKind, PinConfig, StampResult,
};
use crate::dsl::BbdType;
use crate::elements::{Interpolation, Medium};
use crate::tree::MnaSystem;

use super::impl_component_dyn;

// ═══════════════════════════════════════════════════════════════════════════
// Bbd
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Bbd {
    pub bbd_type: BbdType,
}

impl Component for Bbd {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "BBD delay" }

    fn is_passive(&self) -> bool { false }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["in", "input", "out", "output", "clock"],
            aliases: &[("in", "input"), ("out", "output")],
        }
    }

    fn modulation_pins(&self) -> &'static [&'static str] { &["clock"] }

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

    fn edges(&self) -> Vec<ComponentEdge> {
        vec![ComponentEdge { pin_a: "input", pin_b: "output", kind: EdgeKind::Behavioral, port_group: None }]
    }

    fn controls(&self) -> Vec<ControlParam> {
        vec![
            ControlParam { name: "clock", kind: ControlParamKind::BbdClockRate },
            ControlParam { name: "feedback", kind: ControlParamKind::BbdFeedback },
        ]
    }

    fn modulation_sink(&self, pin: &str) -> Option<ModulationSink> {
        match pin {
            "clock" => Some(ModulationSink {
                target_kind: ModulationSinkKind::BbdClock,
                bias: 0.15,
                range: 0.10,
            }),
            _ => None,
        }
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        match self.bbd_type {
            BbdType::Mn3207 => ("Analog_Delay:MN3207", "IC"),
            BbdType::Mn3007 => ("Analog_Delay:MN3007", "IC"),
            BbdType::Mn3005 => ("Analog_Delay:MN3005", "IC"),
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// DelayLineComp
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct DelayLineComp {
    pub min_delay: f64,
    pub max_delay: f64,
    pub interpolation: Interpolation,
    pub medium: Medium,
}

impl Component for DelayLineComp {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "delay line" }

    fn is_passive(&self) -> bool { false }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &[
                "input", "in", "output", "out",
                "rate", "speed_mod", "delay_time", "feedback",
            ],
            aliases: &[("in", "input"), ("out", "output")],
        }
    }

    fn modulation_pins(&self) -> &'static [&'static str] { &["speed_mod", "delay_time"] }

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

    fn edges(&self) -> Vec<ComponentEdge> {
        vec![ComponentEdge { pin_a: "input", pin_b: "output", kind: EdgeKind::Behavioral, port_group: None }]
    }

    fn controls(&self) -> Vec<ControlParam> {
        vec![
            ControlParam { name: "delay_time", kind: ControlParamKind::DelayTime },
            ControlParam { name: "feedback", kind: ControlParamKind::DelayFeedback },
        ]
    }

    fn modulation_sink(&self, pin: &str) -> Option<ModulationSink> {
        match pin {
            "speed_mod" => Some(ModulationSink {
                target_kind: ModulationSinkKind::DelaySpeed,
                bias: 0.0,
                range: 0.02,
            }),
            "delay_time" => Some(ModulationSink {
                target_kind: ModulationSinkKind::DelayTime,
                bias: 0.5,
                range: 0.5,
            }),
            _ => None,
        }
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) { ("", "DL") }
}

// ═══════════════════════════════════════════════════════════════════════════
// Tap
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Tap {
    pub parent_id: String,
    pub ratio: f64,
}

impl Component for Tap {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "tap" }

    fn is_passive(&self) -> bool { false }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["output", "out"],
            aliases: &[("out", "output")],
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

    fn footprint_ref(&self) -> (&'static str, &'static str) { ("", "TAP") }
}
