//! Vacuum tube component structs: Triode, Pentode, VariMu.

use crate::compiler::component::{Component, GraphRole, PinConfig, StampResult};
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

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Valve:Triode", "V")
    }
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

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Valve:Triode", "V")
    }
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

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Valve:Triode", "V")
    }
}
