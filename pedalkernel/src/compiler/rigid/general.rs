//! General rigid stage building (NL feedback paths).
//!
//! Handles stages with op-amp gain driving a nonlinear root
//! (TS/RAT/Klon clipping pattern). Uses classified FeedbackGroup.

use super::super::build::create_root;
use super::super::component::EdgeKind;
use super::super::dyn_node::DynNode;
use super::super::feedback::FeedbackGroup;
use super::super::graph::CircuitGraph;
use super::super::stage::WdfStage;
use super::super::wdf_leaf::WdfVoltageSource;
use super::opamp_root::{extract_opamp_config, make_opamp_root};
use super::{is_inverting_topology, StageStats};
use crate::oversampling::{Oversampler, OversamplingFactor};

use super::super::spqr_build::with_voltage_source;

/// Build a stage with op-amp gain driving a nonlinear root.
///
/// Uses classified FeedbackGroup: Rf from feedback_edges, Ri from
/// pendant_edges, NL root from active_edges.
pub(in crate::compiler) fn build_opamp_nl_feedback(
    group: &FeedbackGroup,
    stats: &StageStats,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<WdfStage, String> {
    // Extract op-amp config from classified group
    let inverting = is_inverting_topology(stats, graph);
    let config = extract_opamp_config(group, inverting, graph)?;
    let opamp = make_opamp_root(&config, sample_rate);

    // Find first NL active edge → build root
    let nl_edge_idx = group
        .active_edges
        .iter()
        .find(|&&eidx| graph.effective_edge_kind(eidx) == EdgeKind::Nonlinear)
        .ok_or("General stage has no NL edge")?;

    let e = &graph.edges[*nl_edge_idx];
    let comp = &graph.components[e.comp_idx];
    let (nl_kind, _) = comp
        .kind
        .classify_nonlinear(&comp.id, e.node_a, e.node_b, graph.gnd_node, &graph.node_names)
        .ok_or_else(|| format!("NL edge {} ({}) didn't classify", nl_edge_idx, comp.id))?;

    let (root, base_diode_model) = create_root(&nl_kind, false);

    // Tree from pendant edges (input coupling)
    let tree = if !group.pendant_edges.is_empty() {
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

    let oversampler = Oversampler::new(OversamplingFactor::X2);
    let mut stage = WdfStage::new(tree, root, oversampler);
    stage.feedback_opamp = Some(opamp);
    stage.base_diode_model = base_diode_model;
    Ok(stage)
}
