//! Diode component structs: Diode, DiodePair, Zener, Neon.

use crate::compiler::component::{Component, GraphRole, PinConfig, StampResult};
use crate::compiler::validate::Severity;
use crate::dsl::{DiodeType, NeonType};
use crate::tree::MnaSystem;

use super::impl_component_dyn;

// ═══════════════════════════════════════════════════════════════════════════
// Diode
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Diode {
    pub diode_type: DiodeType,
}

impl Component for Diode {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "diode" }

    fn is_passive(&self) -> bool { false }

    fn is_nonlinear(&self) -> bool { true }

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
        _n1: Option<usize>,
        _n2: Option<usize>,
        _mna: &mut MnaSystem,
        _sample_rate: f64,
    ) -> StampResult {
        StampResult::Skip
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Device:D", "D")
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// DiodePair
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct DiodePair {
    pub diode_type: DiodeType,
}

impl Component for DiodePair {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "diode pair" }

    fn is_passive(&self) -> bool { false }

    fn is_nonlinear(&self) -> bool { true }

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
        _n1: Option<usize>,
        _n2: Option<usize>,
        _mna: &mut MnaSystem,
        _sample_rate: f64,
    ) -> StampResult {
        StampResult::Skip
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Device:D", "D")
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Zener
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Zener {
    pub breakdown_voltage: f64,
}

impl Component for Zener {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "zener diode" }

    fn is_passive(&self) -> bool { false }

    fn is_nonlinear(&self) -> bool { true }

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
        _n1: Option<usize>,
        _n2: Option<usize>,
        _mna: &mut MnaSystem,
        _sample_rate: f64,
    ) -> StampResult {
        StampResult::Skip
    }

    fn validate_values(&self, comp_id: &str) -> Vec<(Severity, String)> {
        let mut w = vec![];
        if self.breakdown_voltage <= 0.0 {
            w.push((
                Severity::Error,
                format!(
                    "Zener '{}' has non-positive breakdown voltage {:.2} V",
                    comp_id, self.breakdown_voltage
                ),
            ));
        }
        w
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Device:D_Zener", "D")
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Neon
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct Neon {
    pub neon_type: NeonType,
}

impl Component for Neon {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str { "neon bulb" }

    fn is_passive(&self) -> bool { false }

    fn is_nonlinear(&self) -> bool { true }

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
        _n1: Option<usize>,
        _n2: Option<usize>,
        _mna: &mut MnaSystem,
        _sample_rate: f64,
    ) -> StampResult {
        StampResult::Skip
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        match self.neon_type {
            NeonType::Ne2 | NeonType::Ne51 | NeonType::Ne83 => ("Device:Lamp_Neon", "NE"),
        }
    }
}
