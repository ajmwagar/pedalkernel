//! Op-amp root stage building for Rigid stages.
//!
//! Uses the classified FeedbackGroup directly — no second DFS.
//! Rf comes from feedback_edges, Ri from pendant_edges.

use super::super::component::EdgeKind;
use super::super::dyn_node::DynNode;
use super::super::feedback::FeedbackGroup;
use super::super::graph::CircuitGraph;
use super::super::stage::{RootKind, WdfStage};
use super::super::wdf_leaf::WdfVoltageSource;
use crate::elements::{OpAmpModel, OpAmpRoot};
use crate::oversampling::{Oversampler, OversamplingFactor};

use super::super::spqr_build::with_voltage_source;
use super::super::graph::NodeId;

/// Shared op-amp extraction from classified FeedbackGroup.
pub(in crate::compiler) struct OpAmpConfig {
    pub model: OpAmpModel,
    pub rf: f64,
    pub ri: f64,
    pub gain: f64,
    pub inverting: bool,
}

/// Extract op-amp configuration from a classified FeedbackGroup.
///
/// Rf = sum of resistance on feedback_edges (the out→neg cycle).
/// Ri = sum of resistance on pendant_edges (input coupling).
/// No DFS — feedback.rs already did the classification.
pub(in crate::compiler) fn extract_opamp_config(
    group: &FeedbackGroup,
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

    // Rf: sum resistance of feedback_edges (already classified by feedback.rs)
    let rf: f64 = group
        .feedback_edges
        .iter()
        .filter_map(|&eidx| graph.components[graph.edges[eidx].comp_idx].kind.resistance())
        .sum();

    // Ri: sum resistance of pendant_edges (input coupling)
    let ri: f64 = if !group.pendant_edges.is_empty() {
        group
            .pendant_edges
            .iter()
            .filter_map(|&eidx| graph.components[graph.edges[eidx].comp_idx].kind.resistance())
            .sum::<f64>()
    } else {
        f64::INFINITY
    };
    // If pendant edges have no resistance (all caps?), fall back to infinity
    let ri = if ri <= 0.0 { f64::INFINITY } else { ri };

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

/// Build an OpAmpRoot stage from a classified FeedbackGroup.
pub(in crate::compiler) fn build_opamp_root(
    group: &FeedbackGroup,
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
