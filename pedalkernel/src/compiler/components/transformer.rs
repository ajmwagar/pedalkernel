//! Transformer component struct.

use crate::compiler::component::{
    Component, ComponentEdge, EdgeKind, GraphRole, PinConfig, PinDirection, StampResult,
};
use crate::compiler::dyn_node::DynNode;
use crate::dsl::{TransformerConfig, WindingType};
use crate::tree::MnaSystem;

use super::impl_component_dyn;

// ═══════════════════════════════════════════════════════════════════════════
// TransformerComp
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq)]
pub struct TransformerComp {
    pub config: TransformerConfig,
}

impl Component for TransformerComp {
    impl_component_dyn!();

    fn type_tag(&self) -> &'static str {
        "transformer"
    }

    fn is_passive(&self) -> bool {
        true
    }

    fn pin_config(&self) -> PinConfig {
        PinConfig {
            valid_pins: &[
                "a",
                "b",
                "c",
                "d",
                "e",
                "f",
                "primary.a",
                "primary.b",
                "secondary.a",
                "secondary.b",
                "tertiary.a",
                "tertiary.b",
                "pri.a",
                "pri.b",
                "sec.a",
                "sec.b",
                "ter.a",
                "ter.b",
                "pri_a",
                "pri_b",
                "sec_a",
                "sec_b",
                "ter_a",
                "ter_b",
                "pri_ct",
                "pri.ct",
                "sec_ct",
                "sec.ct",
                "ct",
                "primary.ct",
                "secondary.ct",
            ],
            aliases: &[],
        }
    }

    fn graph_role(&self) -> GraphRole {
        GraphRole::Transformer
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

    fn make_leaf(&self, _comp_id: &str, sample_rate: f64) -> Option<DynNode> {
        let l_primary = self.config.primary_inductance;
        let n = self.config.turns_ratio;
        let l_secondary = l_primary / (n * n);
        let secondary = Box::new(DynNode::Inductor(
            None,
            l_secondary,
            2.0 * sample_rate * l_secondary,
        ));
        if let Some(n_tertiary) = self.config.tertiary_turns_ratio {
            let l_tertiary = l_primary / (n_tertiary * n_tertiary);
            let tertiary = Box::new(DynNode::Inductor(
                None,
                l_tertiary,
                2.0 * sample_rate * l_tertiary,
            ));
            let r_sec = secondary.port_resistance();
            let r_ter = tertiary.port_resistance();
            let adaptor =
                crate::tree::RTypeAdaptor::three_winding_transformer(n, n_tertiary, r_sec, r_ter);
            Some(DynNode::RType {
                adaptor,
                children: vec![secondary, tertiary],
            })
        } else {
            Some(DynNode::TransformerNode(secondary, n))
        }
    }

    fn edges(&self) -> Vec<ComponentEdge> {
        vec![ComponentEdge {
            pin_a: "a",
            pin_b: "b",
            kind: EdgeKind::Reactive,
            port_group: None,
        }]
    }

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        match (self.config.primary_type, self.config.secondary_type) {
            (WindingType::PushPull, WindingType::Standard) => {
                ("Transformer:Transformer_1P_1S", "T")
            }
            (WindingType::Standard, WindingType::CenterTap) => {
                ("Transformer:Transformer_1P_1S_CT", "T")
            }
            (WindingType::PushPull, WindingType::CenterTap) => {
                ("Transformer:Transformer_1P_CT_1S_CT", "T")
            }
            _ => ("Transformer:Transformer_1P_1S", "T"),
        }
    }

    fn symbol_name(&self) -> &'static str {
        "transformer"
    }
    fn layout_class(&self) -> &'static str {
        "transformer"
    }

    fn signal_adjacencies(&self) -> Vec<(&'static str, &'static str)> {
        let mut adj = vec![("a", "b"), ("c", "d")];
        if self.config.has_tertiary() {
            adj.push(("e", "f"));
        }
        adj
    }

    fn pin_direction(&self, pin: &str) -> PinDirection {
        if pin.starts_with("pri") || pin == "a" || pin == "b" {
            PinDirection::Input
        } else if pin.starts_with("sec")
            || pin == "c"
            || pin == "d"
            || pin.starts_with("ter")
            || pin == "e"
            || pin == "f"
        {
            PinDirection::Output
        } else {
            PinDirection::Bidirectional
        }
    }

    fn is_transformer(&self) -> bool {
        true
    }
    fn transformer_config(&self) -> Option<&crate::dsl::TransformerConfig> {
        Some(&self.config)
    }
}
