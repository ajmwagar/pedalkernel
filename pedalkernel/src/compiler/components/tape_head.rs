//! Voltage-driven Jiles-Atherton tape head component.
//!
//! A record/playback head modelled as a WDF one-port whose magnetic field is
//! driven by PORT VOLTAGE (`H = kv*V`), so tape saturation tracks signal level
//! at line drive — unlike the transformer core, which is Ampere/current-driven
//! and only saturates at hundreds of mA of magnetizing current.
//!
//! Wiring: a simple 2-terminal `(a, b)` element that joins the surrounding
//! passive/reactive WDF tree as a leaf. Its `make_leaf` produces
//! `DynNode::TapeHeadVoltage`, whose per-sample Newton solves the rate-
//! independent J-A field-domain trajectory and the port KCL (see
//! `pedalkernel-rt/.../jiles_atherton.rs::TapeHeadVoltageRoot`).

use crate::compiler::component::{
    Component, ComponentEdge, EdgeKind, GraphRole, PinConfig, PinDirection, StampResult,
};
use crate::compiler::dyn_node::DynNode;
use crate::dsl::TapeHeadConfig;
use crate::tree::MnaSystem;
use pedalkernel_rt::elements::JaCoreModel;

use super::impl_component_dyn;

#[derive(Debug, Clone, PartialEq)]
pub struct TapeHeadComp {
    pub config: TapeHeadConfig,
}

impl TapeHeadComp {
    fn ja_model(&self) -> JaCoreModel {
        let c = &self.config;
        JaCoreModel {
            ms: c.ja_ms,
            a: c.ja_a,
            alpha: c.ja_alpha,
            k: c.ja_k,
            c: c.ja_c,
            // The voltage law uses kv (H = kv*V), not Ampere's law, so the
            // geometry only needs to be sane positives for `is_complete()`.
            n_turns: 100.0,
            area: 1.0e-6,
            path_len: 1.0e-3,
            gap: 0.0,
        }
    }
}

impl Component for TapeHeadComp {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str {
        "tape_head"
    }

    // Passive in the BFS sense: it is a self-contained WDF leaf (its own root
    // solver), so it collects into the surrounding passive/reactive WDF tree
    // exactly like the transformer core's J-A magnetizing leaf.
    fn is_passive(&self) -> bool {
        true
    }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &["a", "b"],
            aliases: &[],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Edge {
            pin_a: "a",
            pin_b: "b",
        }
    }

    fn edges(&self) -> Vec<ComponentEdge> {
        // Reactive: carries magnetic state between samples, so it must be a WDF
        // port (same classification the transformer winding uses).
        vec![ComponentEdge {
            pin_a: "a",
            pin_b: "b",
            kind: EdgeKind::Reactive,
            port_group: None,
        }]
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
        let c = &self.config;
        Some(DynNode::TapeHeadVoltage(
            Some(comp_id.to_string()),
            self.ja_model(),
            c.kv,
            c.isat,
            c.gp,
            c.h_bias,
            sample_rate,
            c.rp,
        ))
    }

    /// Report a representative inductance so the stage builder treats this as a
    /// reactive WDF port (the actual port resistance comes from the leaf's own
    /// `port_resistance()` — this value is only the reactive-port marker; see
    /// `spqr_build.rs` where `inductance().is_some()` selects the WDF-port path).
    fn inductance(&self) -> Option<f64> {
        Some(self.config.rp / (2.0 * 48_000.0))
    }

    fn pin_direction(&self, pin: &str) -> PinDirection {
        match pin {
            "a" => PinDirection::Input,
            "b" => PinDirection::Output,
            _ => PinDirection::Bidirectional,
        }
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        ("Device:L", "TH")
    }

    fn symbol_name(&self) -> &'static str {
        "inductor"
    }
    fn layout_class(&self) -> &'static str {
        "tape_head"
    }
}
