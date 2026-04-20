//! General rigid stage building (NL feedback paths).
//!
//! Handles stages with op-amp gain driving a nonlinear root
//! (TS/RAT/Klon clipping pattern).

use super::super::build::create_root;
use super::super::dyn_node::DynNode;
use super::super::graph::{CircuitGraph, NodeId};
use super::super::stage::WdfStage;
use super::super::wdf_leaf::WdfVoltageSource;
use super::opamp_root::{extract_opamp_config, make_opamp_root};
use super::{is_inverting_topology, StageStats};
use crate::oversampling::{Oversampler, OversamplingFactor};

use super::super::spqr_build::with_voltage_source;

/// Build a stage with op-amp gain driving a nonlinear root (TS/RAT/Klon pattern).
///
/// The OpAmpRoot pre-amplifies the input (gain + GBW + slew), then the
/// NL root (diode) clips the amplified signal. This is the `feedback_opamp`
/// pattern from the existing pipeline.
pub(in crate::compiler) fn build_opamp_nl_feedback(
    edge_indices: &[usize],
    pendant_trees: &[(DynNode, NodeId)],
    stats: &StageStats,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<WdfStage, String> {
    use super::super::component::EdgeKind;

    // Extract op-amp config (gain from Rf/Ri)
    let inverting = is_inverting_topology(stats, graph);
    let config = extract_opamp_config(edge_indices, pendant_trees, inverting, graph)?;
    let opamp = make_opamp_root(&config, sample_rate);

    // Find first NL edge → build root
    let nl_edge_idx = edge_indices
        .iter()
        .find(|&&eidx| {
            let ek = graph.effective_edge_kind(eidx);
            ek == EdgeKind::Nonlinear
        })
        .ok_or("General stage has no NL edge")?;

    let e = &graph.edges[*nl_edge_idx];
    let comp = &graph.components[e.comp_idx];
    let (nl_kind, _) = comp
        .kind
        .classify_nonlinear(&comp.id, e.node_a, e.node_b, graph.gnd_node, &graph.node_names)
        .ok_or_else(|| format!("NL edge {} ({}) didn't classify", nl_edge_idx, comp.id))?;

    let (root, base_diode_model) = create_root(&nl_kind, false);

    // Tree: VS + pendant Ri (same as OpAmpRoot path)
    let tree = if let Some((pendant, _)) = pendant_trees.first() {
        with_voltage_source(pendant.clone())
    } else {
        DynNode::Leaf(Box::new(WdfVoltageSource {
            voltage: 0.0,
            rp: 1.0,
            is_cathode_bias: false,
        }))
    };

    let oversampler = Oversampler::new(OversamplingFactor::X2);
    let mut stage = WdfStage::new(tree, root, oversampler);
    stage.feedback_opamp = Some(opamp);
    stage.base_diode_model = base_diode_model;
    Ok(stage)
}
