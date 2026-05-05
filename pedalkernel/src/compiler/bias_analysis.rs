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
    StaticBias {
        dc_voltages: HashMap<NodeId, f64>,
    },
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
pub(super) fn classify_group_bias(
    group: &FlowGroup,
    graph: &CircuitGraph,
) -> GroupBiasKind {
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
        let edge_names: Vec<_> = group.all_edges().iter()
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
    interior_nodes: &HashSet<NodeId>,
    rail_set: &HashSet<NodeId>,
    graph: &CircuitGraph,
) -> HashMap<NodeId, f64> {
    // Map interior nodes to matrix indices
    let node_list: Vec<NodeId> = interior_nodes.iter().copied().collect();
    let n = node_list.len();
    if n == 0 {
        return HashMap::new();
    }
    let node_idx: HashMap<NodeId, usize> = node_list
        .iter()
        .enumerate()
        .map(|(i, &n)| (n, i))
        .collect();

    // Known rail voltages
    let rail_voltage = |node: NodeId| -> f64 {
        if node == graph.gnd_node {
            0.0
        } else if node == graph.vcc_node {
            // Use the supply voltage from the graph
            graph.supply_voltages.get(&graph.vcc_node).copied().unwrap_or(9.0)
        } else if let Some(&v) = graph.supply_voltages.get(&node) {
            v
        } else if graph.ac_ground_nodes.contains(&node) {
            0.0
        } else {
            0.0
        }
    };

    // Build conductance matrix G and current vector I for KCL: G·V = I
    // For each resistor edge: G[a][a] += 1/R, G[b][b] += 1/R,
    //                         G[a][b] -= 1/R, G[b][a] -= 1/R
    // If one terminal is a rail: I[other] += V_rail / R
    let mut g_matrix = vec![0.0f64; n * n]; // Row-major
    let mut i_vector = vec![0.0f64; n];

    for &eidx in group.all_edges().iter() {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];

        // Only resistors contribute at DC. Caps are open-circuit.
        let r = match comp.kind.resistance() {
            Some(r) if r > 0.0 => r,
            _ => continue,
        };
        let g = 1.0 / r; // conductance

        let a_interior = node_idx.get(&e.node_a);
        let b_interior = node_idx.get(&e.node_b);
        let a_rail = rail_set.contains(&e.node_a);
        let b_rail = rail_set.contains(&e.node_b);

        match (a_interior, b_interior) {
            (Some(&ia), Some(&ib)) => {
                // Both interior: stamp mutual conductance
                g_matrix[ia * n + ia] += g;
                g_matrix[ib * n + ib] += g;
                g_matrix[ia * n + ib] -= g;
                g_matrix[ib * n + ia] -= g;
            }
            (Some(&ia), None) if b_rail => {
                // b is a rail with known voltage
                g_matrix[ia * n + ia] += g;
                i_vector[ia] += g * rail_voltage(e.node_b);
            }
            (None, Some(&ib)) if a_rail => {
                // a is a rail with known voltage
                g_matrix[ib * n + ib] += g;
                i_vector[ib] += g * rail_voltage(e.node_a);
            }
            _ => {
                // Both rails or both external — skip
            }
        }
    }

    // Solve G·V = I via Gaussian elimination (small system, n ≤ ~5)
    let voltages = solve_linear_system(&mut g_matrix, &mut i_vector, n);

    // Map back to node IDs
    let mut result = HashMap::new();
    if let Some(v) = voltages {
        for (i, &node) in node_list.iter().enumerate() {
            result.insert(node, v[i]);
        }
    }
    result
}

/// Solve a small linear system Ax = b via Gaussian elimination with
/// partial pivoting. Returns None if the system is singular.
fn solve_linear_system(a: &mut [f64], b: &mut [f64], n: usize) -> Option<Vec<f64>> {
    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_row = col;
        let mut max_val = a[col * n + col].abs();
        for row in (col + 1)..n {
            let val = a[row * n + col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }

        if max_val < 1e-15 {
            return None; // Singular
        }

        // Swap rows
        if max_row != col {
            for j in 0..n {
                a.swap(col * n + j, max_row * n + j);
            }
            b.swap(col, max_row);
        }

        // Eliminate
        let pivot = a[col * n + col];
        for row in (col + 1)..n {
            let factor = a[row * n + col] / pivot;
            for j in col..n {
                a[row * n + j] -= factor * a[col * n + j];
            }
            b[row] -= factor * b[col];
        }
    }

    // Back substitution
    let mut x = vec![0.0; n];
    for col in (0..n).rev() {
        let mut sum = b[col];
        for j in (col + 1)..n {
            sum -= a[col * n + j] * x[j];
        }
        x[col] = sum / a[col * n + col];
    }

    Some(x)
}
