//! Op-amp root stage building for Rigid stages.
//!
//! Uses the classified FlowGroup directly — no second DFS.
//! Rf comes from feedback_edges, Ri from pendant_edges.

use super::super::component::EdgeKind;
use super::super::dyn_node::DynNode;
use super::super::signal_flow::FlowGroup;
use super::super::graph::CircuitGraph;
use super::super::stage::{RootKind, WdfStage};
use super::super::wdf_leaf::WdfVoltageSource;
use crate::elements::{OpAmpModel, OpAmpRoot};
use crate::oversampling::{Oversampler, OversamplingFactor};

use super::super::spqr_build::with_voltage_source;
use super::super::graph::NodeId;

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
    let model = OpAmpModel::from_opamp_type(&op_type);

    // Compute Rf: sum resistance of ALL feedback edges (resistors + pots).
    // Pots contribute their current position's resistance. At compile time
    // this is the initial position (typically 50% of max_r).
    let rf: f64 = group
        .feedback_edges
        .iter()
        .filter_map(|&eidx| {
            let comp = &graph.components[graph.edges[eidx].comp_idx];
            comp.kind.resistance()
        })
        .sum();

    // Compute Ri: the input coupling resistance at the op-amp's neg node.
    // Strategy: find resistors touching the neg node that are NOT in the
    // feedback set or active set. These are the input coupling path,
    // regardless of which FlowGroup they belong to.
    let vcvs_comp_idx = graph.edges[*vcvs_edge_idx].comp_idx;
    let neg_node = graph.nullor_pins.iter()
        .find(|p| p.comp_idx == vcvs_comp_idx)
        .map(|p| p.neg_node);

    let ri = if let Some(neg) = neg_node {
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
pub(in crate::compiler) fn make_opamp_root(config: &OpAmpConfig, sample_rate: f64) -> OpAmpRoot {
    let supply_voltage: f64 = 9.0; // TODO: propagate from PedalDef
    let mut root = if config.inverting {
        OpAmpRoot::new_inverting(config.model, config.gain)
    } else {
        OpAmpRoot::new_non_inverting(config.model, config.gain)
    };
    root.set_sample_rate(sample_rate);
    root.set_v_max((supply_voltage / 2.0 - 1.5).max(0.5));
    root
}

/// Build an OpAmpRoot stage from a classified FlowGroup.
pub(in crate::compiler) fn build_opamp_root(
    group: &FlowGroup,
    inverting: bool,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<WdfStage, String> {
    let config = extract_opamp_config(group, inverting, graph)?;
    // Rf=0 is valid: unity-gain buffer (direct neg→out connection)
    let root = make_opamp_root(&config, sample_rate);

    // Build WDF tree from pendant edges (input coupling)
    let tree = if !group.pendant_edges.is_empty() {
        // Create DynNode from pendant passive components
        let pendant_leaf = group.pendant_edges.iter().find_map(|&eidx| {
            let comp = &graph.components[graph.edges[eidx].comp_idx];
            comp.kind.make_leaf(&comp.id, sample_rate)
        });
        if let Some(leaf) = pendant_leaf {
            with_voltage_source(leaf)
        } else {
            DynNode::Leaf(Box::new(WdfVoltageSource {
                voltage: 0.0,
                rp: 1.0,
                is_cathode_bias: false,
            }))
        }
    } else {
        DynNode::Leaf(Box::new(WdfVoltageSource {
            voltage: 0.0,
            rp: 1.0,
            is_cathode_bias: false,
        }))
    };

    let oversampler = Oversampler::new(OversamplingFactor::X1);
    Ok(WdfStage::new(tree, RootKind::OpAmp(root), oversampler))
}
