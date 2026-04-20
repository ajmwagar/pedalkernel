//! Op-amp root stage building for Rigid stages.
//!
//! Handles OpAmpConfig extraction and OpAmpRoot stage construction
//! for single-VCVS resistive feedback topologies.

use super::super::dyn_node::DynNode;
use super::super::graph::CircuitGraph;
use super::super::stage::{RootKind, WdfStage};
use super::super::wdf_leaf::WdfVoltageSource;
use crate::elements::{OpAmpModel, OpAmpRoot};
use crate::oversampling::{Oversampler, OversamplingFactor};

use super::super::spqr_build::with_voltage_source;
use super::super::graph::NodeId;

/// Shared op-amp extraction: finds VCVS edge, computes Rf, Ri, gain.
pub(in crate::compiler) struct OpAmpConfig {
    pub model: OpAmpModel,
    pub rf: f64,
    pub ri: f64,
    pub gain: f64,
    pub inverting: bool,
}

/// Extract op-amp configuration from a Rigid stage's edges + pendants.
///
/// Queries Component::op_amp_type() for the model, Component::resistance()
/// for Rf (linear edges in R-node), and pendant port_resistance() for Ri.
pub(in crate::compiler) fn extract_opamp_config(
    edge_indices: &[usize],
    pendant_trees: &[(DynNode, NodeId)],
    inverting: bool,
    graph: &CircuitGraph,
) -> Result<OpAmpConfig, String> {
    use super::super::component::EdgeKind;

    // Find the VCVS edge → get OpAmpModel
    let vcvs_edge_idx = edge_indices
        .iter()
        .find(|&&eidx| graph.effective_edge_kind(eidx) == EdgeKind::Vcvs)
        .ok_or("No VCVS edge found")?;

    let vcvs_comp = &graph.components[graph.edges[*vcvs_edge_idx].comp_idx];
    let op_type = vcvs_comp
        .kind
        .op_amp_type()
        .ok_or("VCVS component has no op_amp_type")?;
    let model = OpAmpModel::from_opamp_type(&op_type);

    // Sum feedback resistance (Rf): linear edges in the R-node
    let rf: f64 = edge_indices
        .iter()
        .filter(|&&eidx| graph.effective_edge_kind(eidx) == EdgeKind::Linear)
        .filter_map(|&eidx| graph.components[graph.edges[eidx].comp_idx].kind.resistance())
        .sum();

    // Input resistance (Ri): from pendant tree port resistance
    let ri: f64 = if !pendant_trees.is_empty() {
        pendant_trees.iter().map(|(tree, _)| tree.port_resistance()).sum()
    } else {
        f64::INFINITY // No pendant → unity gain
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

/// Build an OpAmpRoot stage (no NL elements — pure gain + GBW + slew).
pub(in crate::compiler) fn build_opamp_root(
    edge_indices: &[usize],
    pendant_trees: &[(DynNode, NodeId)],
    inverting: bool,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<WdfStage, String> {
    let config = extract_opamp_config(edge_indices, pendant_trees, inverting, graph)?;
    if config.rf <= 0.0 {
        return Err("OpAmpRoot: no feedback resistance found".to_string());
    }
    let root = make_opamp_root(&config, sample_rate);

    // Tree: VS + pendant Ri
    let tree = if let Some((pendant, _)) = pendant_trees.first() {
        with_voltage_source(pendant.clone())
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
