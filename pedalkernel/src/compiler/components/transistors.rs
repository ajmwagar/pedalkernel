//! Transistor component structs: Npn, Pnp, NJfet, PJfet, Nmos, Pmos.

use hashbrown::HashMap;

use crate::compiler::classify::NonlinearKind;
use crate::compiler::component::{
    Component, ComponentEdge, EdgeKind, GraphRole, KMethodSpec, ModulationSink, ModulationSinkKind,
    PinConfig, PinDirection, ResolveContext, SignalTerminals, StampResult,
    K_AXIS_INCIDENT_CONTROL_2D,
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

    fn type_tag(&self) -> &'static str {
        "NPN transistor"
    }

    fn ports(&self) -> Vec<(&'static str, &'static str)> {
        vec![("base", "emitter"), ("collector", "emitter")]
    }

    fn signal_terminals(&self) -> SignalTerminals {
        SignalTerminals::Amplifier {
            input: "base",
            output: "collector",
            control: None,
        }
    }

    fn is_passive(&self) -> bool {
        false
    }

    fn is_nonlinear(&self) -> bool {
        true
    }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["base", "collector", "emitter"],
            aliases: &[],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::ActiveEdge {
            pin_a: "collector",
            pin_b: "emitter",
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
        vec![ComponentEdge {
            pin_a: "collector",
            pin_b: "emitter",
            kind: EdgeKind::Nonlinear,
            port_group: None,
        }]
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
        let collector_node = node_names
            .get(&format!("{}.collector", comp_id))
            .copied()
            .unwrap_or(node_a);
        let emitter_node = node_names
            .get(&format!("{}.emitter", comp_id))
            .copied()
            .unwrap_or(node_b);
        Some((
            NonlinearKind::BjtNpn {
                model_name: self.model.clone(),
                base_node,
                collector_node,
                emitter_node,
            },
            vec![node_a, node_b],
        ))
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Device:Q_NPN_BCE", "Q")
    }

    fn symbol_name(&self) -> &'static str {
        "npn_bjt"
    }
    fn layout_class(&self) -> &'static str {
        "npn"
    }

    fn signal_adjacencies(&self) -> Vec<(&'static str, &'static str)> {
        vec![
            ("base", "collector"),
            ("collector", "emitter"),
            ("base", "emitter"),
        ]
    }

    fn pin_direction(&self, pin: &str) -> PinDirection {
        match pin {
            "base" => PinDirection::Input,
            "collector" => PinDirection::Output,
            "emitter" => PinDirection::Down,
            _ => PinDirection::Bidirectional,
        }
    }

    fn is_bjt(&self) -> bool {
        true
    }
    fn is_gain_device(&self) -> bool {
        true
    }
    fn k_method_spec(&self) -> Option<KMethodSpec> {
        Some(KMethodSpec {
            axes: K_AXIS_INCIDENT_CONTROL_2D,
            reason: "BJT: 2D memoryless I-V (Vbe, Vce)",
        })
    }
    fn model_name(&self) -> Option<&str> {
        Some(&self.model)
    }
    fn port_semantic(
        &self,
        _pin_a: &str,
        _pin_b: &str,
    ) -> crate::compiler::component::PortSemantic {
        // All BJT ports (B-E, C-E, B-C) are nonlinear junctions.
        crate::compiler::component::PortSemantic::Nonlinear
    }
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

    fn type_tag(&self) -> &'static str {
        "PNP transistor"
    }

    fn ports(&self) -> Vec<(&'static str, &'static str)> {
        vec![("base", "emitter"), ("collector", "emitter")]
    }

    fn signal_terminals(&self) -> SignalTerminals {
        SignalTerminals::Amplifier {
            input: "base",
            output: "collector",
            control: None,
        }
    }

    fn is_passive(&self) -> bool {
        false
    }

    fn is_nonlinear(&self) -> bool {
        true
    }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["base", "collector", "emitter"],
            aliases: &[],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::ActiveEdge {
            pin_a: "collector",
            pin_b: "emitter",
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
        vec![ComponentEdge {
            pin_a: "collector",
            pin_b: "emitter",
            kind: EdgeKind::Nonlinear,
            port_group: None,
        }]
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
        let collector_node = node_names
            .get(&format!("{}.collector", comp_id))
            .copied()
            .unwrap_or(node_a);
        let emitter_node = node_names
            .get(&format!("{}.emitter", comp_id))
            .copied()
            .unwrap_or(node_b);
        Some((
            NonlinearKind::BjtPnp {
                model_name: self.model.clone(),
                base_node,
                collector_node,
                emitter_node,
            },
            vec![node_a, node_b],
        ))
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Device:Q_PNP_BCE", "Q")
    }

    fn symbol_name(&self) -> &'static str {
        "pnp_bjt"
    }
    fn layout_class(&self) -> &'static str {
        "pnp"
    }

    fn signal_adjacencies(&self) -> Vec<(&'static str, &'static str)> {
        vec![
            ("base", "collector"),
            ("collector", "emitter"),
            ("base", "emitter"),
        ]
    }

    fn pin_direction(&self, pin: &str) -> PinDirection {
        match pin {
            "base" => PinDirection::Input,
            "collector" => PinDirection::Output,
            "emitter" => PinDirection::Down,
            _ => PinDirection::Bidirectional,
        }
    }

    fn is_bjt(&self) -> bool {
        true
    }
    fn is_gain_device(&self) -> bool {
        true
    }
    fn k_method_spec(&self) -> Option<KMethodSpec> {
        Some(KMethodSpec {
            axes: K_AXIS_INCIDENT_CONTROL_2D,
            reason: "BJT: 2D memoryless I-V (Vbe, Vce)",
        })
    }
    fn model_name(&self) -> Option<&str> {
        Some(&self.model)
    }
    fn port_semantic(
        &self,
        _pin_a: &str,
        _pin_b: &str,
    ) -> crate::compiler::component::PortSemantic {
        crate::compiler::component::PortSemantic::Nonlinear
    }
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

    fn type_tag(&self) -> &'static str {
        "N-channel JFET"
    }

    fn ports(&self) -> Vec<(&'static str, &'static str)> {
        vec![("drain", "source")] // gate is voltage-sense
    }

    fn signal_terminals(&self) -> SignalTerminals {
        SignalTerminals::Amplifier {
            input: "gate",
            output: "drain",
            control: None,
        }
    }

    fn is_passive(&self) -> bool {
        false
    }

    fn is_nonlinear(&self) -> bool {
        true
    }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["gate", "drain", "source", "vgs"],
            aliases: &[],
        }
    }

    fn modulation_pins(&self) -> &'static [&'static str] {
        &["vgs", "gate"]
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Edge {
            pin_a: "drain",
            pin_b: "source",
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

    fn make_leaf(&self, comp_id: &str, _sample_rate: f64) -> Option<DynNode> {
        let model = crate::model_lookup::jfet_model_by_name(&self.model);
        Some(DynNode::JfetVrNode(
            comp_id.to_string(),
            JfetVariableResistor::new(model),
        ))
    }

    fn is_variable(&self) -> bool {
        true
    }

    fn edges(&self) -> Vec<ComponentEdge> {
        vec![ComponentEdge {
            pin_a: "drain",
            pin_b: "source",
            kind: EdgeKind::Nonlinear,
            port_group: None,
        }]
    }

    fn resolve_edges(&self, ctx: &ResolveContext) -> Option<Vec<ComponentEdge>> {
        if ctx.control_pin_is_modulated {
            Some(vec![ComponentEdge {
                pin_a: "drain",
                pin_b: "source",
                kind: EdgeKind::Linear,
                port_group: None,
            }])
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
            NonlinearKind::Jfet {
                model_name: self.model.clone(),
                is_n_channel: true,
            },
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

    fn symbol_name(&self) -> &'static str {
        "njfet"
    }
    fn layout_class(&self) -> &'static str {
        "njfet"
    }

    fn pin_direction(&self, pin: &str) -> PinDirection {
        match pin {
            "gate" => PinDirection::Input,
            "drain" => PinDirection::Output,
            "source" => PinDirection::Down,
            _ => PinDirection::Bidirectional,
        }
    }

    fn is_jfet(&self) -> bool {
        true
    }
    fn is_gain_device(&self) -> bool {
        true
    }
    fn k_method_spec(&self) -> Option<KMethodSpec> {
        Some(KMethodSpec {
            axes: K_AXIS_INCIDENT_CONTROL_2D,
            reason: "JFET: 2D memoryless I-V (Vgs, Vds)",
        })
    }
    fn model_name(&self) -> Option<&str> {
        Some(&self.model)
    }
    fn port_semantic(&self, pin_a: &str, pin_b: &str) -> crate::compiler::component::PortSemantic {
        let pins = [pin_a, pin_b];
        if pins.contains(&"gate") {
            // Gate junction is nonlinear (diode)
            crate::compiler::component::PortSemantic::Nonlinear
        } else {
            // Drain-source: controlled conductance (Vgs modulates Rds)
            crate::compiler::component::PortSemantic::ControlledConductance
        }
    }
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

    fn type_tag(&self) -> &'static str {
        "P-channel JFET"
    }

    fn ports(&self) -> Vec<(&'static str, &'static str)> {
        vec![("drain", "source")]
    }

    fn signal_terminals(&self) -> SignalTerminals {
        SignalTerminals::Amplifier {
            input: "gate",
            output: "drain",
            control: None,
        }
    }

    fn is_passive(&self) -> bool {
        false
    }

    fn is_nonlinear(&self) -> bool {
        true
    }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["gate", "drain", "source", "vgs"],
            aliases: &[],
        }
    }

    fn modulation_pins(&self) -> &'static [&'static str] {
        &["vgs", "gate"]
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Edge {
            pin_a: "drain",
            pin_b: "source",
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

    fn make_leaf(&self, comp_id: &str, _sample_rate: f64) -> Option<DynNode> {
        let model = crate::model_lookup::jfet_model_by_name(&self.model);
        Some(DynNode::JfetVrNode(
            comp_id.to_string(),
            JfetVariableResistor::new(model),
        ))
    }

    fn is_variable(&self) -> bool {
        true
    }

    fn edges(&self) -> Vec<ComponentEdge> {
        vec![ComponentEdge {
            pin_a: "drain",
            pin_b: "source",
            kind: EdgeKind::Nonlinear,
            port_group: None,
        }]
    }

    fn resolve_edges(&self, ctx: &ResolveContext) -> Option<Vec<ComponentEdge>> {
        if ctx.control_pin_is_modulated {
            Some(vec![ComponentEdge {
                pin_a: "drain",
                pin_b: "source",
                kind: EdgeKind::Linear,
                port_group: None,
            }])
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
            NonlinearKind::Jfet {
                model_name: self.model.clone(),
                is_n_channel: false,
            },
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

    fn symbol_name(&self) -> &'static str {
        "pjfet"
    }
    fn layout_class(&self) -> &'static str {
        "pjfet"
    }

    fn pin_direction(&self, pin: &str) -> PinDirection {
        match pin {
            "gate" => PinDirection::Input,
            "drain" => PinDirection::Output,
            "source" => PinDirection::Down,
            _ => PinDirection::Bidirectional,
        }
    }

    fn is_jfet(&self) -> bool {
        true
    }
    fn is_gain_device(&self) -> bool {
        true
    }
    fn k_method_spec(&self) -> Option<KMethodSpec> {
        Some(KMethodSpec {
            axes: K_AXIS_INCIDENT_CONTROL_2D,
            reason: "JFET: 2D memoryless I-V (Vgs, Vds)",
        })
    }
    fn model_name(&self) -> Option<&str> {
        Some(&self.model)
    }
    fn port_semantic(&self, pin_a: &str, pin_b: &str) -> crate::compiler::component::PortSemantic {
        let pins = [pin_a, pin_b];
        if pins.contains(&"gate") {
            crate::compiler::component::PortSemantic::Nonlinear
        } else {
            crate::compiler::component::PortSemantic::ControlledConductance
        }
    }
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

    fn type_tag(&self) -> &'static str {
        "N-channel MOSFET"
    }

    fn ports(&self) -> Vec<(&'static str, &'static str)> {
        vec![("drain", "source")]
    }

    fn signal_terminals(&self) -> SignalTerminals {
        SignalTerminals::Amplifier {
            input: "gate",
            output: "drain",
            control: None,
        }
    }

    fn is_passive(&self) -> bool {
        false
    }

    fn is_nonlinear(&self) -> bool {
        true
    }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["gate", "drain", "source", "vgs"],
            aliases: &[],
        }
    }

    fn modulation_pins(&self) -> &'static [&'static str] {
        &["vgs", "gate"]
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Edge {
            pin_a: "drain",
            pin_b: "source",
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
        vec![ComponentEdge {
            pin_a: "drain",
            pin_b: "source",
            kind: EdgeKind::Nonlinear,
            port_group: None,
        }]
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
            NonlinearKind::Mosfet {
                mosfet_type: self.mosfet_type,
                is_n_channel: true,
            },
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

    fn symbol_name(&self) -> &'static str {
        "nmos"
    }
    fn layout_class(&self) -> &'static str {
        "nmos"
    }

    fn pin_direction(&self, pin: &str) -> PinDirection {
        match pin {
            "gate" => PinDirection::Input,
            "drain" => PinDirection::Output,
            "source" => PinDirection::Down,
            _ => PinDirection::Bidirectional,
        }
    }

    fn is_mosfet(&self) -> bool {
        true
    }
    fn is_gain_device(&self) -> bool {
        true
    }
    fn k_method_spec(&self) -> Option<KMethodSpec> {
        Some(KMethodSpec {
            axes: K_AXIS_INCIDENT_CONTROL_2D,
            reason: "MOSFET: 2D memoryless I-V (Vgs, Vds)",
        })
    }
    fn port_semantic(&self, pin_a: &str, pin_b: &str) -> crate::compiler::component::PortSemantic {
        let pins = [pin_a, pin_b];
        if pins.contains(&"gate") {
            crate::compiler::component::PortSemantic::Nonlinear
        } else {
            crate::compiler::component::PortSemantic::ControlledConductance
        }
    }
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

    fn type_tag(&self) -> &'static str {
        "P-channel MOSFET"
    }

    fn ports(&self) -> Vec<(&'static str, &'static str)> {
        vec![("drain", "source")]
    }

    fn signal_terminals(&self) -> SignalTerminals {
        SignalTerminals::Amplifier {
            input: "gate",
            output: "drain",
            control: None,
        }
    }

    fn is_passive(&self) -> bool {
        false
    }

    fn is_nonlinear(&self) -> bool {
        true
    }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["gate", "drain", "source", "vgs"],
            aliases: &[],
        }
    }

    fn modulation_pins(&self) -> &'static [&'static str] {
        &["vgs", "gate"]
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Edge {
            pin_a: "drain",
            pin_b: "source",
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
        vec![ComponentEdge {
            pin_a: "drain",
            pin_b: "source",
            kind: EdgeKind::Nonlinear,
            port_group: None,
        }]
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
            NonlinearKind::Mosfet {
                mosfet_type: self.mosfet_type,
                is_n_channel: false,
            },
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

    fn symbol_name(&self) -> &'static str {
        "pmos"
    }
    fn layout_class(&self) -> &'static str {
        "pmos"
    }

    fn pin_direction(&self, pin: &str) -> PinDirection {
        match pin {
            "gate" => PinDirection::Input,
            "drain" => PinDirection::Output,
            "source" => PinDirection::Down,
            _ => PinDirection::Bidirectional,
        }
    }

    fn is_mosfet(&self) -> bool {
        true
    }
    fn is_gain_device(&self) -> bool {
        true
    }
    fn k_method_spec(&self) -> Option<KMethodSpec> {
        Some(KMethodSpec {
            axes: K_AXIS_INCIDENT_CONTROL_2D,
            reason: "MOSFET: 2D memoryless I-V (Vgs, Vds)",
        })
    }
    fn port_semantic(&self, pin_a: &str, pin_b: &str) -> crate::compiler::component::PortSemantic {
        let pins = [pin_a, pin_b];
        if pins.contains(&"gate") {
            crate::compiler::component::PortSemantic::Nonlinear
        } else {
            crate::compiler::component::PortSemantic::ControlledConductance
        }
    }
}
