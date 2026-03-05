//! Transistor component structs: Npn, Pnp, NJfet, PJfet, Nmos, Pmos.

use crate::compiler::component::{Component, GraphRole, PinConfig, StampResult};
use crate::compiler::dyn_node::DynNode;
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

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Device:Q_NPN_BCE", "Q")
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

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Device:Q_PNP_BCE", "Q")
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

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Device:Q_NJFET_DGS", "J")
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

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Device:Q_PJFET_DGS", "J")
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

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Device:Q_NMOS_DGS", "Q")
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

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Device:Q_PMOS_DGS", "Q")
    }
}
