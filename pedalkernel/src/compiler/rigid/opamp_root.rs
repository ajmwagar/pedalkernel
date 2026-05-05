//! Op-amp configuration extraction and root construction for Rigid stages.
//!
//! Extracts Rf/Ri from FlowGroup feedback/pendant edges, computes gain,
//! and creates OpAmpRoot with bias-derived rail limits.

use super::super::component::EdgeKind;
use super::super::signal_flow::FlowGroup;
use super::super::graph::{CircuitGraph, NodeId};
use crate::elements::{OpAmpModel, OpAmpRoot};

/// Shared op-amp extraction from classified FlowGroup.
pub(in crate::compiler) struct OpAmpConfig {
    pub model: OpAmpModel,
    pub rf: f64,
    pub ri: f64,
    pub gain: f64,
    pub inverting: bool,
}

/// Extract op-amp configuration from a classified FlowGroup.
///
/// Rf = sum of resistance on feedback_edges (the out→neg cycle).
/// Ri = sum of resistance on pendant_edges (input coupling).
/// No DFS — feedback.rs already did the classification.
pub(in crate::compiler) fn extract_opamp_config(
    group: &FlowGroup,
    inverting: bool,
    graph: &CircuitGraph,
) -> Result<OpAmpConfig, String> {
    // Find the VCVS edge → get OpAmpModel
    let vcvs_edge_idx = group
        .active_edges
        .iter()
        .find(|&&eidx| graph.effective_edge_kind(eidx) == EdgeKind::Vcvs)
        .ok_or("No VCVS edge found")?;

    let vcvs_comp = &graph.components[graph.edges[*vcvs_edge_idx].comp_idx];
    let op_type = vcvs_comp
        .kind
        .op_amp_type()
        .ok_or("VCVS component has no op_amp_type")?;
    let model = crate::model_lookup::opamp_model_from_type(&op_type);

    // Compute Rf and Ri from feedback edges.
    // For non-inverting: edges in the ground-return path are Ri (ground leg).
    // All other resistive feedback edges are Rf (neg→out).
    // For inverting: all feedback edges are Rf; Ri comes from pendant/graph.
    //
    // Ground-return detection: a resistor "touches ground" if either of its
    // nodes is GND, an AC ground, or connects to GND through a reactive
    // element (cap/inductor). Example: R_hp → C_hp → GND means R_hp is
    // in the ground-return path (Ri at mid frequencies).
    let reaches_gnd = |node: NodeId| -> bool {
        if node == graph.gnd_node || graph.ac_ground_nodes.contains(&node) {
            return true;
        }
        // Check if any reactive neighbor of this node connects to GND
        graph.edges.iter().any(|e2| {
            let other = if e2.node_a == node { e2.node_b }
                else if e2.node_b == node { e2.node_a }
                else { return false };
            let c2 = &graph.components[e2.comp_idx];
            (c2.kind.capacitance().is_some() || c2.kind.inductance().is_some())
                && (other == graph.gnd_node || graph.ac_ground_nodes.contains(&other))
        })
    };

    let mut rf = 0.0f64;
    let mut ri_from_feedback = 0.0f64;
    for &eidx in &group.feedback_edges {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        if let Some(r) = comp.kind.resistance() {
            let touches_gnd = reaches_gnd(e.node_a) || reaches_gnd(e.node_b);
            if !inverting && touches_gnd {
                ri_from_feedback += r;
            } else {
                rf += r;
            }
        }
    }

    // Compute Ri: the input coupling resistance at the op-amp's neg node.
    // Strategy: find resistors touching the neg node that are NOT in the
    // feedback set or active set. These are the input coupling path,
    // regardless of which FlowGroup they belong to.
    let vcvs_comp_idx = graph.edges[*vcvs_edge_idx].comp_idx;
    let neg_node = graph.nullor_pins.iter()
        .find(|p| p.comp_idx == vcvs_comp_idx)
        .map(|p| p.neg_node);

    // For non-inverting, prefer Ri from feedback edges touching GND
    let ri = if !inverting && ri_from_feedback > 0.0 {
        ri_from_feedback
    } else if let Some(neg) = neg_node {
        let feedback_set: std::collections::HashSet<usize> =
            group.feedback_edges.iter().copied().collect();
        let active_set: std::collections::HashSet<usize> =
            group.active_edges.iter().copied().collect();
        let ri_sum: f64 = graph.edges.iter().enumerate()
            .filter_map(|(eidx, e)| {
                if feedback_set.contains(&eidx) || active_set.contains(&eidx) {
                    return None;
                }
                let touches_neg = e.node_a == neg || e.node_b == neg;
                if !touches_neg { return None; }
                let comp = &graph.components[e.comp_idx];
                if comp.kind.is_pot() { return None; }
                comp.kind.resistance()
            })
            .sum();
        if ri_sum > 0.0 { ri_sum } else { f64::INFINITY }
    } else {
        f64::INFINITY
    };

    let gain = if ri.is_infinite() {
        1.0
    } else if inverting {
        rf / ri
    } else {
        1.0 + rf / ri
    };

    Ok(OpAmpConfig { model, rf, ri, gain, inverting })
}

/// Create an OpAmpRoot from config, with sample rate and supply voltage set.
///
/// `supply_voltage` comes from the PedalDef's supply declaration.
/// `bias_v_max` is (v_rail_pos, v_rail_neg) from `Component::apply_bias()`.
/// If None, falls back to symmetric supply/2 - headroom.
pub(in crate::compiler) fn make_opamp_root(
    config: &OpAmpConfig,
    sample_rate: f64,
    supply_voltage: f64,
    bias_v_max: Option<(f64, f64)>,
) -> OpAmpRoot {
    let mut root = if config.inverting {
        OpAmpRoot::new_inverting(config.model, config.gain)
    } else {
        OpAmpRoot::new_non_inverting(config.model, config.gain)
    };
    root.set_sample_rate(sample_rate);
    if let Some((pos, neg)) = bias_v_max {
        root.set_v_rails(pos, neg);
    } else {
        root.set_v_max((supply_voltage / 2.0 - 1.5).max(0.5));
    }
    root
}

// build_opamp_root() removed — zero callers. Use make_opamp_root() directly.
