//! Op-amp configuration extraction and root construction for Rigid stages.
//!
//! Extracts Rf/Ri from FlowGroup feedback/pendant edges, computes gain,
//! and creates OpAmpRoot with bias-derived rail limits.

use super::super::component::EdgeKind;
use super::super::graph::{CircuitGraph, NodeId};
use super::super::signal_flow::FlowGroup;
use crate::elements::{OpAmpModel, OpAmpRoot};
use std::collections::{HashMap, HashSet};

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
            let other = if e2.node_a == node {
                e2.node_b
            } else if e2.node_b == node {
                e2.node_a
            } else {
                return false;
            };
            let c2 = &graph.components[e2.comp_idx];
            (c2.kind.capacitance().is_some() || c2.kind.inductance().is_some())
                && (other == graph.gnd_node || graph.ac_ground_nodes.contains(&other))
        })
    };

    let vcvs_comp_idx = graph.edges[*vcvs_edge_idx].comp_idx;
    let pins = graph
        .nullor_pins
        .iter()
        .find(|p| p.comp_idx == vcvs_comp_idx);

    let (rf, ri) = if let Some(pins) = pins {
        extract_feedback_resistances(group, graph, pins.neg_node, pins.out_node, inverting)
    } else {
        legacy_feedback_resistances(group, graph, inverting, &reaches_gnd)
    };

    let gain = if ri.is_infinite() {
        1.0
    } else if inverting {
        rf / ri
    } else {
        1.0 + rf / ri
    };

    Ok(OpAmpConfig {
        model,
        rf,
        ri,
        gain,
        inverting,
    })
}

fn legacy_feedback_resistances(
    group: &FlowGroup,
    graph: &CircuitGraph,
    inverting: bool,
    reaches_gnd: &dyn Fn(NodeId) -> bool,
) -> (f64, f64) {
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
    let ri = if !inverting && ri_from_feedback > 0.0 {
        ri_from_feedback
    } else {
        f64::INFINITY
    };
    (rf, ri)
}

fn extract_feedback_resistances(
    group: &FlowGroup,
    graph: &CircuitGraph,
    neg: NodeId,
    out: NodeId,
    inverting: bool,
) -> (f64, f64) {
    let forbidden_feedback_nodes: HashSet<NodeId> = graph
        .supply_nodes
        .iter()
        .copied()
        .chain(std::iter::once(graph.gnd_node))
        .chain(graph.ac_ground_nodes.iter().copied())
        .chain(std::iter::once(graph.in_node))
        .collect();

    let (rf, rf_edges) = shortest_resistive_path(
        group,
        graph,
        neg,
        |node| node == out,
        &forbidden_feedback_nodes,
        &HashSet::new(),
        false,
    )
    .unwrap_or_else(|| {
        let rf: f64 = group
            .feedback_edges
            .iter()
            .filter_map(|&eidx| {
                graph.components[graph.edges[eidx].comp_idx]
                    .kind
                    .resistance()
            })
            .sum();
        (rf, HashSet::new())
    });

    let ri = if inverting {
        input_leg_resistance(group, graph, neg, out, &rf_edges)
    } else {
        ground_leg_resistance(group, graph, neg, out, &rf_edges)
    };

    (rf, ri)
}

fn input_leg_resistance(
    group: &FlowGroup,
    graph: &CircuitGraph,
    neg: NodeId,
    out: NodeId,
    rf_edges: &HashSet<usize>,
) -> f64 {
    let active_edges: HashSet<usize> = group.active_edges.iter().copied().collect();
    let mut best = f64::INFINITY;

    for &eidx in &group.all_edges() {
        if rf_edges.contains(&eidx) || active_edges.contains(&eidx) {
            continue;
        }
        let e = &graph.edges[eidx];
        let Some(r) = edge_resistance(graph, eidx) else {
            continue;
        };
        let other = if e.node_a == neg {
            e.node_b
        } else if e.node_b == neg {
            e.node_a
        } else {
            continue;
        };
        if other == out {
            continue;
        }

        best = best.min(r);
    }

    best
}

fn ground_leg_resistance(
    group: &FlowGroup,
    graph: &CircuitGraph,
    neg: NodeId,
    out: NodeId,
    rf_edges: &HashSet<usize>,
) -> f64 {
    let mut forbidden = HashSet::new();
    forbidden.insert(out);
    shortest_resistive_path(
        group,
        graph,
        neg,
        |node| is_ground_like(graph, node),
        &forbidden,
        rf_edges,
        true,
    )
    .map(|(r, _)| r)
    .unwrap_or(f64::INFINITY)
}

fn shortest_resistive_path(
    group: &FlowGroup,
    graph: &CircuitGraph,
    start: NodeId,
    is_target: impl Fn(NodeId) -> bool,
    forbidden_nodes: &HashSet<NodeId>,
    forbidden_edges: &HashSet<usize>,
    allow_reactive_ground_short: bool,
) -> Option<(f64, HashSet<usize>)> {
    let edges = group.all_edges();
    let active_edges: HashSet<usize> = group.active_edges.iter().copied().collect();
    let mut dist: HashMap<NodeId, f64> = HashMap::new();
    let mut prev: HashMap<NodeId, (NodeId, usize)> = HashMap::new();
    let mut unsettled = HashSet::new();
    dist.insert(start, 0.0);
    unsettled.insert(start);

    while !unsettled.is_empty() {
        let node = unsettled
            .iter()
            .copied()
            .min_by(|a, b| {
                dist.get(a)
                    .unwrap_or(&f64::INFINITY)
                    .total_cmp(dist.get(b).unwrap_or(&f64::INFINITY))
            })
            .unwrap();
        unsettled.remove(&node);

        if node != start && is_target(node) {
            let mut path_edges = HashSet::new();
            let mut cursor = node;
            while let Some((previous, eidx)) = prev.get(&cursor).copied() {
                path_edges.insert(eidx);
                cursor = previous;
            }
            return Some((*dist.get(&node).unwrap_or(&0.0), path_edges));
        }

        for &eidx in &edges {
            if forbidden_edges.contains(&eidx) || active_edges.contains(&eidx) {
                continue;
            }
            let e = &graph.edges[eidx];
            let next = if e.node_a == node {
                e.node_b
            } else if e.node_b == node {
                e.node_a
            } else {
                continue;
            };
            if next != start && forbidden_nodes.contains(&next) {
                continue;
            }

            let Some(weight) = edge_path_weight(graph, eidx, allow_reactive_ground_short) else {
                continue;
            };
            let candidate = dist.get(&node).copied().unwrap_or(f64::INFINITY) + weight;
            if candidate < dist.get(&next).copied().unwrap_or(f64::INFINITY) {
                dist.insert(next, candidate);
                prev.insert(next, (node, eidx));
                unsettled.insert(next);
            }
        }
    }

    None
}

fn edge_resistance(graph: &CircuitGraph, eidx: usize) -> Option<f64> {
    graph.components[graph.edges[eidx].comp_idx]
        .kind
        .resistance()
        .filter(|r| r.is_finite() && *r > 0.0)
}

fn edge_path_weight(
    graph: &CircuitGraph,
    eidx: usize,
    allow_reactive_ground_short: bool,
) -> Option<f64> {
    if let Some(r) = edge_resistance(graph, eidx) {
        return Some(r);
    }
    if allow_reactive_ground_short {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        let reactive = comp.kind.capacitance().is_some() || comp.kind.inductance().is_some();
        if reactive && (is_ground_like(graph, e.node_a) || is_ground_like(graph, e.node_b)) {
            return Some(0.0);
        }
    }
    None
}

fn is_ground_like(graph: &CircuitGraph, node: NodeId) -> bool {
    node == graph.gnd_node || graph.ac_ground_nodes.contains(&node)
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
