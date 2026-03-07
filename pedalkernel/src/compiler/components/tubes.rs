//! Vacuum tube component structs: Triode, Pentode, VariMu.

use std::collections::HashMap;

use crate::compiler::classify::NonlinearKind;
use crate::compiler::component::{
    Component, ComponentEdge, EdgeKind, GraphRole, ModulationSink, ModulationSinkKind, PinConfig,
    StampResult,
};
use crate::compiler::graph::NodeId;
use crate::tree::MnaSystem;

use super::impl_component_dyn;

// ═══════════════════════════════════════════════════════════════════════════
// Triode
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Triode {
    pub model: String,
}

impl Component for Triode {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "triode" }

    fn is_passive(&self) -> bool { false }

    fn is_nonlinear(&self) -> bool { true }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["plate", "cathode", "grid", "vgk"],
            aliases: &[],
        }
    }

    fn modulation_pins(&self) -> &'static [&'static str] { &["vgk"] }

    fn graph_role(&self) -> GraphRole {
        GraphRole::CoupledEdge {
            edge_pin_a: "plate",
            edge_pin_b: "cathode",
            coupled_pins: &["plate", "cathode", "grid"],
        }
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
        vec![ComponentEdge { pin_a: "plate", pin_b: "cathode", kind: EdgeKind::Nonlinear, port_group: None }]
    }

    fn classify_nonlinear(
        &self,
        comp_id: &str,
        node_a: NodeId,
        node_b: NodeId,
        _gnd_node: NodeId,
        node_names: &HashMap<String, NodeId>,
    ) -> Option<(NonlinearKind, Vec<NodeId>)> {
        let grid_node = node_names.get(&format!("{}.grid", comp_id)).copied();
        Some((
            NonlinearKind::Triode {
                model_name: self.model.clone(),
                plate_node: node_a,
                cathode_node: node_b,
                grid_node,
                parallel_count: 1,
                is_vari_mu: false,
            },
            vec![node_a, node_b],
        ))
    }

    fn modulation_sink(&self, pin: &str) -> Option<ModulationSink> {
        match pin {
            "vgk" => Some(ModulationSink {
                target_kind: ModulationSinkKind::TriodeVgk,
                bias: -2.0,
                range: 2.0,
            }),
            _ => None,
        }
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Valve:Triode", "V")
    }

    fn symbol_name(&self) -> &'static str { "triode" }
    fn layout_class(&self) -> &'static str { "triode" }

    fn is_tube(&self) -> bool { true }
    fn model_name(&self) -> Option<&str> { Some(&self.model) }
}

// ═══════════════════════════════════════════════════════════════════════════
// Pentode
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Pentode {
    pub model: String,
}

impl Component for Pentode {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "pentode" }

    fn is_passive(&self) -> bool { false }

    fn is_nonlinear(&self) -> bool { true }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["plate", "cathode", "g1", "g2", "grid", "screen", "vg1k"],
            aliases: &[],
        }
    }

    fn modulation_pins(&self) -> &'static [&'static str] { &["vg1k"] }

    fn graph_role(&self) -> GraphRole {
        GraphRole::CoupledEdge {
            edge_pin_a: "plate",
            edge_pin_b: "cathode",
            coupled_pins: &["plate", "cathode", "grid", "screen"],
        }
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
        vec![ComponentEdge { pin_a: "plate", pin_b: "cathode", kind: EdgeKind::Nonlinear, port_group: None }]
    }

    fn classify_nonlinear(
        &self,
        _comp_id: &str,
        node_a: NodeId,
        node_b: NodeId,
        _gnd_node: NodeId,
        _node_names: &HashMap<String, NodeId>,
    ) -> Option<(NonlinearKind, Vec<NodeId>)> {
        Some((
            NonlinearKind::Pentode { model_name: self.model.clone() },
            vec![node_a, node_b],
        ))
    }

    fn modulation_sink(&self, pin: &str) -> Option<ModulationSink> {
        match pin {
            "vg1k" => Some(ModulationSink {
                target_kind: ModulationSinkKind::PentodeVg1k,
                bias: -2.0,
                range: 2.0,
            }),
            _ => None,
        }
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Valve:Triode", "V")
    }

    fn symbol_name(&self) -> &'static str { "pentode" }
    fn layout_class(&self) -> &'static str { "pentode" }

    fn is_tube(&self) -> bool { true }
    fn model_name(&self) -> Option<&str> { Some(&self.model) }
}

// ═══════════════════════════════════════════════════════════════════════════
// VariMu
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct VariMu {
    pub model: String,
}

impl Component for VariMu {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "variable-mu triode" }

    fn is_passive(&self) -> bool { false }

    fn is_nonlinear(&self) -> bool { true }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["plate", "cathode", "grid", "vgk"],
            aliases: &[],
        }
    }

    fn modulation_pins(&self) -> &'static [&'static str] { &["vgk"] }

    fn graph_role(&self) -> GraphRole {
        GraphRole::CoupledEdge {
            edge_pin_a: "plate",
            edge_pin_b: "cathode",
            coupled_pins: &["plate", "cathode", "grid"],
        }
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
        vec![ComponentEdge { pin_a: "plate", pin_b: "cathode", kind: EdgeKind::Nonlinear, port_group: None }]
    }

    fn classify_nonlinear(
        &self,
        comp_id: &str,
        node_a: NodeId,
        node_b: NodeId,
        _gnd_node: NodeId,
        node_names: &HashMap<String, NodeId>,
    ) -> Option<(NonlinearKind, Vec<NodeId>)> {
        let grid_node = node_names.get(&format!("{}.grid", comp_id)).copied();
        Some((
            NonlinearKind::Triode {
                model_name: self.model.clone(),
                plate_node: node_a,
                cathode_node: node_b,
                grid_node,
                parallel_count: 1,
                is_vari_mu: true,
            },
            vec![node_a, node_b],
        ))
    }

    fn modulation_sink(&self, pin: &str) -> Option<ModulationSink> {
        match pin {
            "vgk" => Some(ModulationSink {
                target_kind: ModulationSinkKind::VariMuVgk,
                bias: -2.0,
                range: 2.0,
            }),
            _ => None,
        }
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Valve:Triode", "V")
    }

    fn symbol_name(&self) -> &'static str { "triode" }
    fn layout_class(&self) -> &'static str { "vari_mu" }

    fn is_tube(&self) -> bool { true }
    fn model_name(&self) -> Option<&str> { Some(&self.model) }
}
