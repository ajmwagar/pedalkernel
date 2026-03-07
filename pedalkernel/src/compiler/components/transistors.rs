//! Transistor component structs: Npn, Pnp, NJfet, PJfet, Nmos, Pmos.

use std::collections::HashMap;

use crate::compiler::classify::NonlinearKind;
use crate::compiler::component::{
    Component, ComponentEdge, EdgeKind, GraphRole, ModulationSink, ModulationSinkKind, PinConfig,
    ResolveContext, StampResult,
};
use crate::compiler::dyn_node::DynNode;
use crate::compiler::graph::NodeId;
use crate::dsl::MosfetType;
use crate::elements::{JfetModel, JfetVariableResistor};
use crate::tree::MnaSystem;

use super::impl_component_dyn;

// ═══════════════════════════════════════════════════════════════════════════
// Npn
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Npn {
    pub model: String,
}

impl Component for Npn {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "NPN transistor" }

    fn is_passive(&self) -> bool { false }

    fn is_nonlinear(&self) -> bool { true }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["base", "collector", "emitter"],
            aliases: &[],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::ActiveEdge { pin_a: "collector", pin_b: "emitter" }
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
        vec![ComponentEdge { pin_a: "collector", pin_b: "emitter", kind: EdgeKind::Nonlinear, port_group: None }]
    }

    fn classify_nonlinear(
        &self,
        comp_id: &str,
        node_a: NodeId,
        node_b: NodeId,
        _gnd_node: NodeId,
        node_names: &HashMap<String, NodeId>,
    ) -> Option<(NonlinearKind, Vec<NodeId>)> {
        let base_node = node_names
            .get(&format!("{}.base", comp_id))
            .copied()
            .unwrap_or(node_a);
        Some((
            NonlinearKind::BjtNpn { model_name: self.model.clone(), base_node },
            vec![node_a, node_b],
        ))
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Device:Q_NPN_BCE", "Q")
    }

    fn symbol_name(&self) -> &'static str { "npn_bjt" }
    fn layout_class(&self) -> &'static str { "npn" }

    fn is_bjt(&self) -> bool { true }
    fn model_name(&self) -> Option<&str> { Some(&self.model) }
}

// ═══════════════════════════════════════════════════════════════════════════
// Pnp
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Pnp {
    pub model: String,
}

impl Component for Pnp {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "PNP transistor" }

    fn is_passive(&self) -> bool { false }

    fn is_nonlinear(&self) -> bool { true }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["base", "collector", "emitter"],
            aliases: &[],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::ActiveEdge { pin_a: "collector", pin_b: "emitter" }
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
        vec![ComponentEdge { pin_a: "collector", pin_b: "emitter", kind: EdgeKind::Nonlinear, port_group: None }]
    }

    fn classify_nonlinear(
        &self,
        comp_id: &str,
        node_a: NodeId,
        node_b: NodeId,
        _gnd_node: NodeId,
        node_names: &HashMap<String, NodeId>,
    ) -> Option<(NonlinearKind, Vec<NodeId>)> {
        let base_node = node_names
            .get(&format!("{}.base", comp_id))
            .copied()
            .unwrap_or(node_a);
        Some((
            NonlinearKind::BjtPnp { model_name: self.model.clone(), base_node },
            vec![node_a, node_b],
        ))
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Device:Q_PNP_BCE", "Q")
    }

    fn symbol_name(&self) -> &'static str { "pnp_bjt" }
    fn layout_class(&self) -> &'static str { "pnp" }

    fn is_bjt(&self) -> bool { true }
    fn model_name(&self) -> Option<&str> { Some(&self.model) }
}

// ═══════════════════════════════════════════════════════════════════════════
// NJfet
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct NJfet {
    pub model: String,
}

impl Component for NJfet {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "N-channel JFET" }

    fn is_passive(&self) -> bool { false }

    fn is_nonlinear(&self) -> bool { true }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["gate", "drain", "source", "vgs"],
            aliases: &[],
        }
    }

    fn modulation_pins(&self) -> &'static [&'static str] { &["vgs", "gate"] }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Edge { pin_a: "drain", pin_b: "source" }
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

    fn make_leaf(&self, comp_id: &str, _sample_rate: f64) -> Option<DynNode> {
        let model = JfetModel::by_name(&self.model);
        Some(DynNode::JfetVr {
            comp_id: comp_id.to_string(),
            inner: JfetVariableResistor::new(model),
            prev_rds: 0.0,
        })
    }

    fn is_variable(&self) -> bool { true }

    fn edges(&self) -> Vec<ComponentEdge> {
        vec![ComponentEdge { pin_a: "drain", pin_b: "source", kind: EdgeKind::Nonlinear, port_group: None }]
    }

    fn resolve_edges(&self, ctx: &ResolveContext) -> Option<Vec<ComponentEdge>> {
        if ctx.control_pin_is_modulated {
            Some(vec![ComponentEdge { pin_a: "drain", pin_b: "source", kind: EdgeKind::Linear, port_group: None }])
        } else {
            None
        }
    }

    fn classify_nonlinear(
        &self,
        _comp_id: &str,
        _node_a: NodeId,
        node_b: NodeId,
        gnd_node: NodeId,
        _node_names: &HashMap<String, NodeId>,
    ) -> Option<(NonlinearKind, Vec<NodeId>)> {
        let jn = if node_b == gnd_node { _node_a } else { node_b };
        Some((
            NonlinearKind::Jfet { model_name: self.model.clone(), is_n_channel: true },
            vec![jn],
        ))
    }

    fn modulation_sink(&self, pin: &str) -> Option<ModulationSink> {
        match pin {
            "vgs" | "gate" => Some(ModulationSink {
                target_kind: ModulationSinkKind::JfetVgs,
                bias: -0.45,
                range: 0.25,
            }),
            _ => None,
        }
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Device:Q_NJFET_DGS", "J")
    }

    fn symbol_name(&self) -> &'static str { "njfet" }
    fn layout_class(&self) -> &'static str { "njfet" }

    fn is_jfet(&self) -> bool { true }
    fn model_name(&self) -> Option<&str> { Some(&self.model) }
}

// ═══════════════════════════════════════════════════════════════════════════
// PJfet
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct PJfet {
    pub model: String,
}

impl Component for PJfet {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "P-channel JFET" }

    fn is_passive(&self) -> bool { false }

    fn is_nonlinear(&self) -> bool { true }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["gate", "drain", "source", "vgs"],
            aliases: &[],
        }
    }

    fn modulation_pins(&self) -> &'static [&'static str] { &["vgs", "gate"] }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Edge { pin_a: "drain", pin_b: "source" }
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

    fn make_leaf(&self, comp_id: &str, _sample_rate: f64) -> Option<DynNode> {
        let model = JfetModel::by_name(&self.model);
        Some(DynNode::JfetVr {
            comp_id: comp_id.to_string(),
            inner: JfetVariableResistor::new(model),
            prev_rds: 0.0,
        })
    }

    fn is_variable(&self) -> bool { true }

    fn edges(&self) -> Vec<ComponentEdge> {
        vec![ComponentEdge { pin_a: "drain", pin_b: "source", kind: EdgeKind::Nonlinear, port_group: None }]
    }

    fn resolve_edges(&self, ctx: &ResolveContext) -> Option<Vec<ComponentEdge>> {
        if ctx.control_pin_is_modulated {
            Some(vec![ComponentEdge { pin_a: "drain", pin_b: "source", kind: EdgeKind::Linear, port_group: None }])
        } else {
            None
        }
    }

    fn classify_nonlinear(
        &self,
        _comp_id: &str,
        _node_a: NodeId,
        node_b: NodeId,
        gnd_node: NodeId,
        _node_names: &HashMap<String, NodeId>,
    ) -> Option<(NonlinearKind, Vec<NodeId>)> {
        let jn = if node_b == gnd_node { _node_a } else { node_b };
        Some((
            NonlinearKind::Jfet { model_name: self.model.clone(), is_n_channel: false },
            vec![jn],
        ))
    }

    fn modulation_sink(&self, pin: &str) -> Option<ModulationSink> {
        match pin {
            "vgs" | "gate" => Some(ModulationSink {
                target_kind: ModulationSinkKind::JfetVgs,
                bias: -0.45,
                range: 0.25,
            }),
            _ => None,
        }
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Device:Q_PJFET_DGS", "J")
    }

    fn symbol_name(&self) -> &'static str { "pjfet" }
    fn layout_class(&self) -> &'static str { "pjfet" }

    fn is_jfet(&self) -> bool { true }
    fn model_name(&self) -> Option<&str> { Some(&self.model) }
}

// ═══════════════════════════════════════════════════════════════════════════
// Nmos
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Nmos {
    pub mosfet_type: MosfetType,
}

impl Component for Nmos {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "N-channel MOSFET" }

    fn is_passive(&self) -> bool { false }

    fn is_nonlinear(&self) -> bool { true }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["gate", "drain", "source", "vgs"],
            aliases: &[],
        }
    }

    fn modulation_pins(&self) -> &'static [&'static str] { &["vgs", "gate"] }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Edge { pin_a: "drain", pin_b: "source" }
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
        vec![ComponentEdge { pin_a: "drain", pin_b: "source", kind: EdgeKind::Nonlinear, port_group: None }]
    }

    fn classify_nonlinear(
        &self,
        _comp_id: &str,
        _node_a: NodeId,
        node_b: NodeId,
        gnd_node: NodeId,
        _node_names: &HashMap<String, NodeId>,
    ) -> Option<(NonlinearKind, Vec<NodeId>)> {
        let jn = if node_b == gnd_node { _node_a } else { node_b };
        Some((
            NonlinearKind::Mosfet { mosfet_type: self.mosfet_type, is_n_channel: true },
            vec![jn],
        ))
    }

    fn modulation_sink(&self, pin: &str) -> Option<ModulationSink> {
        match pin {
            "vgs" | "gate" => Some(ModulationSink {
                target_kind: ModulationSinkKind::MosfetVgs,
                bias: 3.0,
                range: 2.0,
            }),
            _ => None,
        }
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Device:Q_NMOS_DGS", "Q")
    }

    fn symbol_name(&self) -> &'static str { "nmos" }
    fn layout_class(&self) -> &'static str { "nmos" }

    fn is_mosfet(&self) -> bool { true }
}

// ═══════════════════════════════════════════════════════════════════════════
// Pmos
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Pmos {
    pub mosfet_type: MosfetType,
}

impl Component for Pmos {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "P-channel MOSFET" }

    fn is_passive(&self) -> bool { false }

    fn is_nonlinear(&self) -> bool { true }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["gate", "drain", "source", "vgs"],
            aliases: &[],
        }
    }

    fn modulation_pins(&self) -> &'static [&'static str] { &["vgs", "gate"] }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Edge { pin_a: "drain", pin_b: "source" }
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
        vec![ComponentEdge { pin_a: "drain", pin_b: "source", kind: EdgeKind::Nonlinear, port_group: None }]
    }

    fn classify_nonlinear(
        &self,
        _comp_id: &str,
        _node_a: NodeId,
        node_b: NodeId,
        gnd_node: NodeId,
        _node_names: &HashMap<String, NodeId>,
    ) -> Option<(NonlinearKind, Vec<NodeId>)> {
        let jn = if node_b == gnd_node { _node_a } else { node_b };
        Some((
            NonlinearKind::Mosfet { mosfet_type: self.mosfet_type, is_n_channel: false },
            vec![jn],
        ))
    }

    fn modulation_sink(&self, pin: &str) -> Option<ModulationSink> {
        match pin {
            "vgs" | "gate" => Some(ModulationSink {
                target_kind: ModulationSinkKind::MosfetVgs,
                bias: 3.0,
                range: 2.0,
            }),
            _ => None,
        }
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Device:Q_PMOS_DGS", "Q")
    }

    fn symbol_name(&self) -> &'static str { "pmos" }
    fn layout_class(&self) -> &'static str { "pmos" }

    fn is_mosfet(&self) -> bool { true }
}
