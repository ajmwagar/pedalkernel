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

    // Get VCVS terminals (neg, out) for classifying edges
    let vcvs_edge = &graph.edges[*vcvs_edge_idx];
    let neg = vcvs_edge.node_a;
    let out = vcvs_edge.node_b;

    let is_rail = |node: NodeId| -> bool {
        node == graph.gnd_node
            || graph.supply_nodes.contains(&node)
            || graph.ac_ground_nodes.contains(&node)
    };

    // Classify linear edges by VCVS terminal relationship:
    // - Feedback R (Rf): on path between neg and out (both ends touch neg/out/intermediate)
    // - Input R (Ri): touches neg but NOT out (pendant input coupling)
    let mut rf = 0.0f64;
    let mut ri_from_edges = 0.0f64;

    // Use DFS to find edges on paths from out→neg (feedback path)
    let mut feedback_edge_set = std::collections::HashSet::new();
    {
        // Build signal adjacency for just these edges
        let mut adj: std::collections::HashMap<NodeId, Vec<(usize, NodeId)>> =
            std::collections::HashMap::new();
        for &eidx in edge_indices {
            let e = &graph.edges[eidx];
            if is_rail(e.node_a) || is_rail(e.node_b) {
                continue;
            }
            if graph.effective_edge_kind(eidx) == EdgeKind::Vcvs {
                continue; // Skip the VCVS edge itself
            }
            adj.entry(e.node_a).or_default().push((eidx, e.node_b));
            adj.entry(e.node_b).or_default().push((eidx, e.node_a));
        }
        // DFS from out to neg, collecting feedback edges
        let mut visited = std::collections::HashSet::new();
        visited.insert(out);
        let mut path = Vec::new();
        super::super::feedback::dfs_find_feedback_edges(
            out, neg, &adj, &mut visited, &mut path, &mut feedback_edge_set,
        );
    }

    for &eidx in edge_indices {
        if graph.effective_edge_kind(eidx) != EdgeKind::Linear {
            continue;
        }
        if let Some(r) = graph.components[graph.edges[eidx].comp_idx].kind.resistance() {
            if feedback_edge_set.contains(&eidx) {
                rf += r; // On feedback path → Rf
            } else {
                ri_from_edges += r; // Not on feedback path → Ri
            }
        }
    }

    // Input resistance: from edge-based Ri or pendant trees
    let ri: f64 = if ri_from_edges > 0.0 {
        ri_from_edges
    } else if !pendant_trees.is_empty() {
        pendant_trees.iter().map(|(tree, _)| tree.port_resistance()).sum()
    } else {
        f64::INFINITY // No input coupling → unity gain
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
