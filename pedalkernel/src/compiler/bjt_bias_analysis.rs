//! BJT bias pot analysis.
//!
//! Detects potentiometers that bridge emitter nodes of two coupled BJTs
//! (e.g., the Fuzz Face's 1kΩ "Fuzz" pot between Q2.emitter and Q1.emitter).
//!
//! These pots primarily control inter-stage feedback coupling and DC bias,
//! not signal-path impedance. Their small resistance (typically 1kΩ) makes
//! the scattering matrix insensitive to changes when treated as a passive
//! WDF port. Instead, we model them as parametric controls that adjust
//! `feedback_scale` and `veb_bias_offset` on the coupled BJT stage.
//!
//! This mirrors the `opamp_analysis.rs` pattern for op-amp feedback pots.

use std::collections::{HashMap, HashSet};

use crate::dsl::*;

use super::components::Potentiometer;
use super::graph::{CircuitGraph, NodeId};

/// Result of BJT bias pot analysis.
pub(super) struct BjtBiasAnalysis {
    /// Pot component ID → info for binding.
    pub(super) bias_pot_map: HashMap<String, BjtBiasPotInfo>,
    /// Edge indices of bias pots (to exclude from passive collection).
    pub(super) bias_pot_edge_indices: HashSet<usize>,
}

/// Info for a BJT bias pot binding.
pub(super) struct BjtBiasPotInfo {
    /// Maximum resistance of the pot (ohms).
    pub(super) max_pot_r: f64,
    /// Taper type.
    pub(super) taper: PotTaper,
}

/// Detect pots bridging emitter nodes of coupled BJTs.
///
/// For each coupled BJT cluster, scans the passive edges for pot edges
/// whose endpoints are both emitter nodes of different BJTs in the cluster.
///
/// `bjt_emitter_nodes` maps each BJT info index → emitter node ID.
/// `cluster_members` is a list of clusters, each cluster being a list of BJT info indices.
/// `all_passive_edges` is the set of passive edges for the multi-NL plan.
pub(super) fn detect_bias_pots(
    graph: &CircuitGraph,
    cluster_members: &[Vec<usize>],
    bjt_emitter_nodes: &[NodeId],
    all_passive_edge_sets: &[Vec<usize>],
) -> BjtBiasAnalysis {
    let mut bias_pot_map = HashMap::new();
    let mut bias_pot_edge_indices = HashSet::new();

    for (cluster_idx, members) in cluster_members.iter().enumerate() {
        if members.len() < 2 {
            continue;
        }

        // Collect emitter nodes for this cluster.
        let emitter_nodes: HashSet<NodeId> =
            members.iter().map(|&m| bjt_emitter_nodes[m]).collect();

        // Scan passive edges for this cluster's multi-NL plan.
        let passive_edges = &all_passive_edge_sets[cluster_idx];
        for &eidx in passive_edges {
            let edge = &graph.edges[eidx];
            let comp = &graph.components[edge.comp_idx];

            // Only consider pots.
            let (max_r, taper) = match comp.kind.as_any().downcast_ref::<Potentiometer>() {
                Some(pot) => (pot.max_r, pot.taper),
                None => continue,
            };

            // Check if both endpoints are emitter nodes of different BJTs in this cluster.
            let a_is_emitter = emitter_nodes.contains(&edge.node_a);
            let b_is_emitter = emitter_nodes.contains(&edge.node_b);

            if a_is_emitter && b_is_emitter && edge.node_a != edge.node_b {
                bias_pot_map.insert(
                    comp.id.clone(),
                    BjtBiasPotInfo {
                        max_pot_r: max_r,
                        taper,
                    },
                );
                bias_pot_edge_indices.insert(eidx);

                // Also exclude Baxandall-decomposed edges for the same pot.
                let aw_id = format!("{}__aw", comp.id);
                let wb_id = format!("{}__wb", comp.id);
                for (other_eidx, other_edge) in graph.edges.iter().enumerate() {
                    let other_comp = &graph.components[other_edge.comp_idx];
                    if other_comp.id == aw_id || other_comp.id == wb_id {
                        bias_pot_edge_indices.insert(other_eidx);
                    }
                }
            }
        }
    }

    BjtBiasAnalysis {
        bias_pot_map,
        bias_pot_edge_indices,
    }
}
