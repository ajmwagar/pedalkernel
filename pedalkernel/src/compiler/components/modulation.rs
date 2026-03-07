//! Modulation component structs: Lfo, EnvelopeFollower, Photocoupler.

use crate::compiler::component::{
    Component, ComponentEdge, ControlParam, ControlParamKind, EdgeKind, GraphRole, ModulationSink,
    ModulationSinkKind, PinConfig, StampResult,
};
use crate::compiler::dyn_node::DynNode;
use crate::compiler::validate::Severity;
use crate::dsl::{LfoWaveformDsl, PhotocouplerType};
use crate::elements::{Photocoupler as PhotocouplerElem, PhotocouplerModel};
use crate::tree::MnaSystem;

use super::impl_component_dyn;

// ═══════════════════════════════════════════════════════════════════════════
// Lfo
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Lfo {
    pub waveform: LfoWaveformDsl,
    pub timing_r: f64,
    pub timing_c: f64,
}

impl Component for Lfo {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "LFO" }

    fn is_passive(&self) -> bool { false }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["out", "output", "rate"],
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

    fn validate_values(&self, comp_id: &str) -> Vec<(Severity, String)> {
        let mut w = Vec::new();
        if self.timing_r <= 0.0 || self.timing_c <= 0.0 {
            w.push((
                Severity::Error,
                format!(
                    "LFO '{}' has non-positive timing values (R={:.2}, C={:.2e})",
                    comp_id, self.timing_r, self.timing_c
                ),
            ));
        }
        let freq = 1.0 / (2.0 * std::f64::consts::PI * self.timing_r * self.timing_c);
        if freq > 100.0 {
            w.push((
                Severity::Warning,
                format!(
                    "LFO '{}' base frequency is {:.1} Hz \u{2014} unusually fast for modulation",
                    comp_id, freq
                ),
            ));
        }
        w
    }

    fn controls(&self) -> Vec<ControlParam> {
        vec![
            ControlParam { name: "rate", kind: ControlParamKind::LfoRate },
            ControlParam { name: "depth", kind: ControlParamKind::LfoDepth },
        ]
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) { ("", "LFO") }

    fn symbol_name(&self) -> &'static str { "lfo" }
    fn layout_class(&self) -> &'static str { "lfo" }

    fn is_modulation_source(&self) -> bool { true }
}

// ═══════════════════════════════════════════════════════════════════════════
// EnvelopeFollower
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct EnvelopeFollower {
    pub attack_r: f64,
    pub attack_c: f64,
    pub release_r: f64,
    pub release_c: f64,
    pub sensitivity_r: f64,
}

impl Component for EnvelopeFollower {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "envelope follower" }

    fn is_passive(&self) -> bool { false }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["out", "output", "in", "input"],
            aliases: &[("in", "input"), ("out", "output")],
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
        if self.attack_r <= 0.0
            || self.attack_c <= 0.0
            || self.release_r <= 0.0
            || self.release_c <= 0.0
            || self.sensitivity_r <= 0.0
        {
            w.push((
                Severity::Error,
                format!(
                    "EnvelopeFollower '{}' has non-positive parameter value(s)",
                    comp_id
                ),
            ));
        }
        w
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) { ("", "ENV") }

    fn symbol_name(&self) -> &'static str { "envelope" }
    fn layout_class(&self) -> &'static str { "envelope" }

    fn is_modulation_source(&self) -> bool { true }
}

// ═══════════════════════════════════════════════════════════════════════════
// PhotocouplerComp
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct PhotocouplerComp {
    pub coupler_type: PhotocouplerType,
}

impl Component for PhotocouplerComp {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "photocoupler" }

    fn is_passive(&self) -> bool { false }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["a", "b", "led"],
            aliases: &[],
        }
    }

    fn modulation_pins(&self) -> &'static [&'static str] { &["led"] }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Edge { pin_a: "a", pin_b: "b" }
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

    fn make_leaf(&self, comp_id: &str, sample_rate: f64) -> Option<DynNode> {
        let model = match self.coupler_type {
            PhotocouplerType::Vtl5c3 => PhotocouplerModel::vtl5c3(),
            PhotocouplerType::Vtl5c1 => PhotocouplerModel::vtl5c1(),
            PhotocouplerType::Nsl32 => PhotocouplerModel::nsl32(),
            PhotocouplerType::T4b => PhotocouplerModel::t4b(),
        };
        Some(DynNode::Photocoupler {
            comp_id: comp_id.to_string(),
            inner: PhotocouplerElem::new(model, sample_rate),
            prev_resistance: 0.0,
        })
    }

    fn is_variable(&self) -> bool { true }

    fn edges(&self) -> Vec<ComponentEdge> {
        vec![ComponentEdge { pin_a: "a", pin_b: "b", kind: EdgeKind::Linear, port_group: None }]
    }

    fn modulation_sink(&self, pin: &str) -> Option<ModulationSink> {
        match pin {
            "led" => Some(ModulationSink {
                target_kind: ModulationSinkKind::PhotocouplerLed,
                bias: 0.5,
                range: 0.5,
            }),
            _ => None,
        }
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) { ("Isolator:PC817", "OC") }

    fn symbol_name(&self) -> &'static str { "photocoupler" }
    fn layout_class(&self) -> &'static str { "photocoupler" }
}
