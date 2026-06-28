//! Compile-time DC bias extraction from circuit graphs.
//!
//! Classifies each flow group as either:
//! - **StaticBias**: inputs are only supply rails (VCC, GND). DC operating
//!   point is computed from the resistor divider at compile time and applied
//!   to the connected active element's model parameters. The group is bypassed
//!   in the serial audio processing chain.
//! - **SignalPath**: has audio-rate inputs (connected to circuit in/out). Must
//!   be processed at runtime in the serial audio chain.
//! - **DynamicModulation**: has audio-rate input but doesn't carry the main
//!   signal (sidechains, envelope followers). Processed at runtime through
//!   modulation routing, not the serial chain. (Future — currently treated
//!   as SignalPath.)
//!
//! The detection is graph-level: "does this branch have an audio-rate input
//! or only DC supply?" No component-type special-casing.

use hashbrown::HashMap;
use std::collections::{HashSet, VecDeque};

use super::graph::{CircuitGraph, NodeId};
use super::signal_flow::FlowGroup;

/// Classification of a flow group's role in the circuit.
#[derive(Debug)]
pub(super) enum GroupBiasKind {
    /// Group is on the audio signal path (in_node → ... → out_node).
    /// Must be processed in the serial audio chain at runtime.
    SignalPath,

    /// Group is a static DC bias network (supply-only inputs).
    /// DC voltages are computed at compile time from resistor dividers.
    /// The group is bypassed in the serial audio chain.
    ///
    /// `dc_voltages` maps each non-rail junction node to its DC voltage.
    StaticBias { dc_voltages: HashMap<NodeId, f64> },
    // Future: DynamicModulation for sidechains, envelope followers, etc.
}

/// Classify a flow group as static bias, signal path, or dynamic modulation.
///
/// A group is **StaticBias** when every node it touches is either:
/// - A supply rail (VCC, GND, supply_nodes, ac_ground_nodes), or
/// - An interior node reachable ONLY through the group's own edges from
///   supply rails — i.e. not reachable from `in_node` or `out_node`
///   without passing through supply rails.
///
/// In other words: if you remove all supply rails from the graph, can the
/// group's nodes still reach `in_node` or `out_node`? If no → static bias.
///
/// When StaticBias, computes the DC voltage at each interior junction node
/// via nodal analysis of the resistor divider network.
pub(super) fn classify_group_bias(group: &FlowGroup, graph: &CircuitGraph) -> GroupBiasKind {
    // A group with any nonlinear edge is NEVER static bias.
    // Diodes/transistors to ground are signal clippers, not DC references.
    let has_nonlinear = group.all_edges().iter().any(|&eidx| {
        let comp = &graph.components[graph.edges[eidx].comp_idx];
        comp.kind.is_nonlinear()
    });
    if has_nonlinear {
        return GroupBiasKind::SignalPath;
    }

    let rail_set = build_rail_set(graph);

    // Collect all nodes this group touches.
    let group_nodes = collect_group_nodes(group, graph);

    // Find the group's non-rail nodes (interior junction nodes).
    let interior_nodes: HashSet<NodeId> = group_nodes
        .iter()
        .filter(|&&n| !rail_set.contains(&n))
        .copied()
        .collect();

    // If the group has NO interior nodes, it's entirely on rails — skip it.
    if interior_nodes.is_empty() {
        return GroupBiasKind::StaticBias {
            dc_voltages: HashMap::new(),
        };
    }

    // Check if any interior node can reach in_node or out_node through
    // non-supply, non-group-internal edges. If so, the group carries audio.
    //
    // Build adjacency from ALL circuit edges, but don't expand through
    // supply rail nodes (they're sinks, not signal bridges).
    let reaches = interior_reaches_signal(&interior_nodes, &rail_set, graph, group);
    #[cfg(test)]
    {
        let edge_names: Vec<_> = group
            .all_edges()
            .iter()
            .map(|&eidx| {
                let e = &graph.edges[eidx];
                let comp = &graph.components[e.comp_idx];
                format!("{}({:?}→{:?})", comp.id, e.node_a, e.node_b)
            })
            .collect();
        let interior_names: Vec<_> = interior_nodes.iter().map(|n| format!("{n:?}")).collect();
        eprintln!("  bias_analysis: edges={edge_names:?}");
        eprintln!("    interior={interior_names:?}, rails={:?}, in={:?}, out={:?}, reaches_signal={reaches}",
            rail_set.len(), graph.in_node, graph.out_node);
    }
    if reaches {
        return GroupBiasKind::SignalPath;
    }

    // Static bias: compute DC voltages from the resistor divider.
    let dc_voltages = compute_dc_voltages(group, &interior_nodes, &rail_set, graph);

    GroupBiasKind::StaticBias { dc_voltages }
}

// ═══════════════════════════════════════════════════════════════════════════
// Internal helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Build the set of all supply/rail nodes.
fn build_rail_set(graph: &CircuitGraph) -> HashSet<NodeId> {
    let mut rails = HashSet::new();
    rails.insert(graph.gnd_node);
    rails.insert(graph.vcc_node);
    for &n in &graph.supply_nodes {
        rails.insert(n);
    }
    for &n in &graph.ac_ground_nodes {
        rails.insert(n);
    }
    rails
}

/// Collect all nodes touched by a group's edges.
fn collect_group_nodes(group: &FlowGroup, graph: &CircuitGraph) -> HashSet<NodeId> {
    let mut nodes = HashSet::new();
    for &eidx in group.all_edges().iter() {
        let e = &graph.edges[eidx];
        nodes.insert(e.node_a);
        nodes.insert(e.node_b);
    }
    nodes
}

/// Check if the group carries audio signal based on its own edge topology.
///
/// A group is static bias if EVERY edge in it has at least one rail
/// terminal (i.e. all edges bridge interior↔rail). This means the group
/// only connects to the supply network, not to other signal-carrying nodes.
///
/// A group is on the signal path if ANY edge connects two non-rail nodes.
/// This means the group bridges between signal-carrying parts of the circuit.
///
/// This check is purely structural — it doesn't matter what other groups
/// connect to. Even if R_b1 bridges vb to U1.pos, the bias group's own
/// edges {R_v1, R_v2, C_v} only go rail↔vb↔rail → static.
fn interior_reaches_signal(
    _interior_nodes: &HashSet<NodeId>,
    rail_set: &HashSet<NodeId>,
    graph: &CircuitGraph,
    group: &FlowGroup,
) -> bool {
    // A group is on the signal path if ANY of its edges connects
    // two non-rail nodes. Rail→interior→rail = bias network.
    for &eidx in group.all_edges().iter() {
        let e = &graph.edges[eidx];
        let a_rail = rail_set.contains(&e.node_a);
        let b_rail = rail_set.contains(&e.node_b);
        if !a_rail && !b_rail {
            // Edge between two non-rail nodes → carries signal
            return true;
        }
    }

    // A plain two-port transformer couples its windings through magnetic flux,
    // NOT a graph edge — the link is in `coupled_nodes`, invisible to the edge
    // scan above. A transformer-primary group whose galvanic edge bridges
    // interior↔rail (e.g. LA-2A's `T_in`: primary across `in`↔gnd) would be
    // mis-classified StaticBias and bypassed, collapsing the forward path. The
    // broker's `Tight` coupled-link rule says that primary↔secondary pair is a
    // co-solved, traversable SIGNAL connection: if any of this group's winding
    // nodes is one end of a Tight link whose OTHER end is non-rail, the group
    // carries signal. (Consults the broker only; no transformer special-casing
    // here.)
    for &eidx in group.all_edges().iter() {
        let e = &graph.edges[eidx];
        for node in [e.node_a, e.node_b] {
            for other in super::boundary_rules::tight_coupled_neighbors(graph, node) {
                if !rail_set.contains(&other) {
                    return true;
                }
            }
        }
    }

    false
}

/// Compute DC voltages at interior junction nodes via resistor divider
/// nodal analysis.
///
/// For each interior node, sets up KCL: sum of currents = 0.
/// Current through each resistor: I = (V_a - V_b) / R.
/// Rail nodes have known voltages (VCC = supply, GND = 0).
/// Caps are open-circuit at DC (ignored).
///
/// For a simple two-resistor divider (the common case), this reduces to:
///   V_junction = VCC × R_bot / (R_top + R_bot)
///
/// For more complex networks, solves the linear system via Gaussian
/// elimination on the conductance matrix.
fn compute_dc_voltages(
    group: &FlowGroup,
    _interior_nodes: &HashSet<NodeId>,
    _rail_set: &HashSet<NodeId>,
    graph: &CircuitGraph,
) -> HashMap<NodeId, f64> {
    // ko5g.3 (pedalkernel-n07s): the resistor-divider DC solve is now the unified
    // `bias::solve_network_bias`.  It performs the same conductance-MNA Gaussian
    // elimination this function used to do, but additionally runs a rail-BFS to
    // restrict the solved node set to interior nodes that have a DC path back to a
    // rail through resistors.  That extra guard skips floating signal-path nodes
    // (cap-isolated junctions) that previously produced zero-conductance rows and
    // a singular matrix → an empty result.  For genuine resistor dividers (the
    // StaticBias case this function is only ever reached for) the two solvers are
    // numerically identical; the BFS only ever *removes* nodes the old code would
    // have failed on, so the StaticBias voltages this returns are unchanged.
    let supply_voltage = graph
        .supply_voltages
        .get(&graph.vcc_node)
        .copied()
        .unwrap_or(9.0);
    super::bias::solve_network_bias(&group.all_edges(), graph, supply_voltage).dc_voltages
}
