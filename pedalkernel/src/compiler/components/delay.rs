//! Delay component structs: Bbd, DelayLine, Tap.

use crate::compiler::component::{
    Component, ComponentEdge, ControlParam, ControlParamKind, EdgeKind, GraphRole, ModulationSink,
    ModulationSinkKind, PinConfig, StampResult,
};
use crate::dsl::{BbdType, SpringTankType};
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

    fn type_tag(&self) -> &'static str {
        "BBD delay"
    }

    fn is_passive(&self) -> bool {
        false
    }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["in", "input", "out", "output", "clock"],
            aliases: &[("in", "input"), ("out", "output")],
        }
    }

    fn modulation_pins(&self) -> &'static [&'static str] {
        &["clock"]
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

    fn edges(&self) -> Vec<ComponentEdge> {
        vec![ComponentEdge {
            pin_a: "input",
            pin_b: "output",
            kind: EdgeKind::Behavioral,
            port_group: None,
        }]
    }

    fn controls(&self) -> Vec<ControlParam> {
        vec![
            ControlParam {
                name: "clock",
                kind: ControlParamKind::BbdClockRate,
            },
            ControlParam {
                name: "feedback",
                kind: ControlParamKind::BbdFeedback,
            },
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

    fn symbol_name(&self) -> &'static str {
        "ic_chip"
    }
    fn layout_class(&self) -> &'static str {
        "bbd"
    }

    fn signal_adjacencies(&self) -> Vec<(&'static str, &'static str)> {
        vec![
            ("in", "out"),
            ("input", "output"),
            ("in", "input"),
            ("out", "output"),
        ]
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Spring (spring reverb tank — behavioral island)
// ═══════════════════════════════════════════════════════════════════════════

/// Spring-reverb tank: a `GraphRole::Virtual` behavioral island (like `Bbd`),
/// contributing one `Behavioral` in→out edge and `StampResult::Skip`. The
/// electromechanical spring pair is modeled by the runtime `SpringTank`; the
/// driver/recovery circuit stays OUTSIDE the island (real WDF stages).
#[derive(Debug, Clone, PartialEq)]
pub struct Spring {
    pub tank_type: SpringTankType,
}

impl Component for Spring {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str {
        "spring reverb"
    }

    fn is_passive(&self) -> bool {
        false
    }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &[
                "in", "input", "out", "output", "dwell", "decay", "damping", "mix",
            ],
            aliases: &[("in", "input"), ("out", "output")],
        }
    }

    fn modulation_pins(&self) -> &'static [&'static str] {
        &["dwell"]
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

    fn edges(&self) -> Vec<ComponentEdge> {
        vec![ComponentEdge {
            pin_a: "input",
            pin_b: "output",
            kind: EdgeKind::Behavioral,
            port_group: None,
        }]
    }

    fn controls(&self) -> Vec<ControlParam> {
        vec![
            ControlParam {
                name: "dwell",
                kind: ControlParamKind::SpringDwell,
            },
            ControlParam {
                name: "decay",
                kind: ControlParamKind::SpringDecay,
            },
            ControlParam {
                name: "damping",
                kind: ControlParamKind::SpringDamping,
            },
            ControlParam {
                name: "mix",
                kind: ControlParamKind::SpringMix,
            },
        ]
    }

    fn modulation_sink(&self, pin: &str) -> Option<ModulationSink> {
        match pin {
            "dwell" => Some(ModulationSink {
                target_kind: ModulationSinkKind::SpringDwell,
                bias: 0.5,
                range: 0.5,
            }),
            _ => None,
        }
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        match self.tank_type {
            SpringTankType::Type4 => ("Reverb:Accutronics_Type4", "RV"),
            SpringTankType::Type8 => ("Reverb:Accutronics_Type8", "RV"),
            SpringTankType::Type9 => ("Reverb:Accutronics_Type9", "RV"),
            SpringTankType::Re201Tank => ("Reverb:RE201_Tank", "RV"),
        }
    }

    fn symbol_name(&self) -> &'static str {
        "spring"
    }
    fn layout_class(&self) -> &'static str {
        "spring"
    }

    fn signal_adjacencies(&self) -> Vec<(&'static str, &'static str)> {
        vec![
            ("in", "out"),
            ("input", "output"),
            ("in", "input"),
            ("out", "output"),
        ]
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

    fn type_tag(&self) -> &'static str {
        "delay line"
    }

    fn is_passive(&self) -> bool {
        false
    }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &[
                "input",
                "in",
                "output",
                "out",
                "rate",
                "speed_mod",
                "delay_time",
                "feedback",
            ],
            aliases: &[("in", "input"), ("out", "output")],
        }
    }

    fn modulation_pins(&self) -> &'static [&'static str] {
        &["speed_mod", "delay_time"]
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

    fn edges(&self) -> Vec<ComponentEdge> {
        vec![ComponentEdge {
            pin_a: "input",
            pin_b: "output",
            kind: EdgeKind::Behavioral,
            port_group: None,
        }]
    }

    fn controls(&self) -> Vec<ControlParam> {
        vec![
            ControlParam {
                name: "delay_time",
                kind: ControlParamKind::DelayTime,
            },
            ControlParam {
                name: "feedback",
                kind: ControlParamKind::DelayFeedback,
            },
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

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("", "DL")
    }

    fn symbol_name(&self) -> &'static str {
        "delay"
    }
    fn layout_class(&self) -> &'static str {
        "delay"
    }

    fn signal_adjacencies(&self) -> Vec<(&'static str, &'static str)> {
        vec![
            ("in", "out"),
            ("input", "output"),
            ("in", "input"),
            ("out", "output"),
        ]
    }
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

    fn type_tag(&self) -> &'static str {
        "tap"
    }

    fn is_passive(&self) -> bool {
        false
    }

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

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("", "TAP")
    }

    fn symbol_name(&self) -> &'static str {
        "tap"
    }
    fn layout_class(&self) -> &'static str {
        "tap"
    }
}
