//! Feedback group analysis for circuit stage partitioning.
//!
//! Determines which active elements (op-amps, BJTs, JFETs, diodes) are
//! coupled through feedback loops and must be simulated together.
//!
//! **Core principle**: two active elements are in the same stage if and
//! only if element A's output can reach element A's input through signal
//! edges, passing through element B. This is cycle detection on the
//! signal graph with rail nodes (GND, VCC, in, out) removed.
//!
//! Rail nodes are NOT shared signal nodes — they're references. Multiple
//! components connecting to GND doesn't couple them.

use std::collections::{HashMap, HashSet, VecDeque};

use super::component::{EdgeKind, SignalTerminals};
use super::graph::{CircuitGraph, NodeId};

/// An active element with identified input/output terminals.
#[derive(Debug, Clone)]
struct ActiveElement {
    /// Index into graph.edges for this element's primary edge.
    edge_idx: usize,
    /// Input terminal node (where signal controls the element).
    /// Op-amp: neg. BJT: base. Tube: grid. Diode: anode.
    input_node: NodeId,
    /// Output terminal node (where amplified signal leaves).
    /// Op-amp: out. BJT: collector. Tube: plate. Diode: cathode.
    output_node: NodeId,
}

/// A group of edges that share a feedback loop.
/// All edges in a group must be simulated as one stage.
#[derive(Debug, Clone)]
pub(in crate::compiler) struct FeedbackGroup {
    /// Edge indices in this group (active + passive feedback edges).
    pub edge_indices: Vec<usize>,
}

/// Set of nodes that are rails (not signal-carrying shared nodes).
/// BFS through these is blocked — they don't couple components.
fn rail_nodes(graph: &CircuitGraph) -> HashSet<NodeId> {
    let mut rails = HashSet::new();
    rails.insert(graph.gnd_node);
    rails.insert(graph.vcc_node);
    rails.extend(&graph.supply_nodes);
    rails.extend(&graph.ac_ground_nodes);
    rails
}

/// Identify active elements and their input/output terminals.
fn find_active_elements(
    edge_indices: &[usize],
    graph: &CircuitGraph,
) -> Vec<ActiveElement> {
    let mut elements = Vec::new();

    for &eidx in edge_indices {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];

        match comp.kind.signal_terminals() {
            SignalTerminals::Passive => {} // R, C, L — no active element
            SignalTerminals::TwoPort { input, output } => {
                // Diode: resolve pin names to node IDs
                let in_node = resolve_pin(&comp.id, input, graph).unwrap_or(e.node_a);
                let out_node = resolve_pin(&comp.id, output, graph).unwrap_or(e.node_b);
                elements.push(ActiveElement {
                    edge_idx: eidx,
                    input_node: in_node,
                    output_node: out_node,
                });
            }
            SignalTerminals::Amplifier { input, output, .. } => {
                // Op-amp, BJT, JFET, Tube: resolve input/output pins
                let in_node = resolve_pin(&comp.id, input, graph).unwrap_or(e.node_a);
                let out_node = resolve_pin(&comp.id, output, graph).unwrap_or(e.node_b);
                elements.push(ActiveElement {
                    edge_idx: eidx,
                    input_node: in_node,
                    output_node: out_node,
                });
            }
        }
    }

    elements
}

/// Resolve a component's pin name to a graph node ID.
fn resolve_pin(comp_id: &str, pin: &str, graph: &CircuitGraph) -> Option<NodeId> {
    let key = format!("{comp_id}.{pin}");
    graph.node_names.get(&key).copied()
}

/// Build signal-edge adjacency (excludes rail nodes).
///
/// Also adds virtual links for multi-terminal active elements:
/// BJT base→collector and base→emitter connections model the
/// transistor's input-output coupling for feedback detection.
fn build_signal_adjacency(
    edge_indices: &[usize],
    graph: &CircuitGraph,
    rails: &HashSet<NodeId>,
) -> HashMap<NodeId, Vec<(usize, NodeId)>> {
    let mut adj: HashMap<NodeId, Vec<(usize, NodeId)>> = HashMap::new();
    for &eidx in edge_indices {
        let e = &graph.edges[eidx];
        if rails.contains(&e.node_a) || rails.contains(&e.node_b) {
            continue; // Ground shunt or bias — not signal coupling
        }
        adj.entry(e.node_a).or_default().push((eidx, e.node_b));
        adj.entry(e.node_b).or_default().push((eidx, e.node_a));

        // For amplifiers (BJT, JFET, Tube): the graph edge typically spans
        // output→emitter/source/cathode, but the input pin (base/gate/grid)
        // controls the device. Add virtual links so BFS can traverse through.
        let comp = &graph.components[e.comp_idx];
        if let SignalTerminals::Amplifier { input, .. } = comp.kind.signal_terminals() {
            if let Some(input_node) = resolve_pin(&comp.id, input, graph) {
                if !rails.contains(&input_node) {
                    // input ↔ edge endpoints (virtual signal coupling)
                    adj.entry(input_node).or_default().push((eidx, e.node_a));
                    adj.entry(e.node_a).or_default().push((eidx, input_node));
                    adj.entry(input_node).or_default().push((eidx, e.node_b));
                    adj.entry(e.node_b).or_default().push((eidx, input_node));
                }
            }
        }
    }
    adj
}

/// BFS from `start` through signal edges. Returns all reachable nodes
/// and the edges traversed, excluding `skip_edge`.
fn bfs_reachable(
    start: NodeId,
    skip_edge: usize,
    adj: &HashMap<NodeId, Vec<(usize, NodeId)>>,
) -> (HashSet<NodeId>, HashSet<usize>) {
    let mut visited_nodes = HashSet::new();
    let mut visited_edges = HashSet::new();
    let mut queue = VecDeque::new();

    visited_nodes.insert(start);
    queue.push_back(start);

    while let Some(node) = queue.pop_front() {
        if let Some(neighbors) = adj.get(&node) {
            for &(eidx, next) in neighbors {
                if eidx == skip_edge {
                    continue;
                }
                if visited_nodes.insert(next) {
                    visited_edges.insert(eidx);
                    queue.push_back(next);
                }
            }
        }
    }

    (visited_nodes, visited_edges)
}

/// DFS: find all edges on any simple path from `current` to `target`.
/// When target is reached, all edges on the current path are added to `result`.
fn dfs_find_paths(
    current: NodeId,
    target: NodeId,
    skip_edge: usize,
    adj: &HashMap<NodeId, Vec<(usize, NodeId)>>,
    visited: &mut HashSet<NodeId>,
    path: &mut Vec<usize>,
    result: &mut HashSet<usize>,
) {
    if let Some(neighbors) = adj.get(&current) {
        for &(eidx, next) in neighbors {
            if eidx == skip_edge {
                continue;
            }
            if next == target {
                // Found a path! Add all edges on the current path + this edge.
                result.insert(eidx);
                result.extend(path.iter());
                continue;
            }
            if visited.contains(&next) {
                continue;
            }
            visited.insert(next);
            path.push(eidx);
            dfs_find_paths(next, target, skip_edge, adj, visited, path, result);
            path.pop();
            visited.remove(&next);
        }
    }
}

/// Find all edges on any simple path from `start` to `target` in the
/// given adjacency. Public wrapper for use by opamp_root edge classification.
pub(in crate::compiler) fn dfs_find_feedback_edges(
    start: NodeId,
    target: NodeId,
    adj: &HashMap<NodeId, Vec<(usize, NodeId)>>,
    visited: &mut HashSet<NodeId>,
    path: &mut Vec<usize>,
    result: &mut HashSet<usize>,
) {
    dfs_find_paths(start, target, usize::MAX, adj, visited, path, result);
}

/// Partition circuit edges into feedback groups.
///
/// Each group contains active elements that share feedback loops,
/// plus all passive edges in those loops. Edges not in any feedback
/// group are returned as individual singleton groups.
///
/// Rail nodes (GND, VCC, supply) are treated as boundaries — they
/// don't couple components. Two diodes both connected to GND are
/// NOT in the same feedback group just because they share GND.
pub(in crate::compiler) fn find_feedback_groups(
    edge_indices: &[usize],
    graph: &CircuitGraph,
) -> Vec<FeedbackGroup> {
    let rails = rail_nodes(graph);
    let adj = build_signal_adjacency(edge_indices, graph, &rails);
    let active_elements = find_active_elements(edge_indices, graph);

    if active_elements.is_empty() {
        // No active elements — everything is one passive group
        return vec![FeedbackGroup {
            edge_indices: edge_indices.to_vec(),
        }];
    }

    // Union-Find: group active elements by shared feedback loops
    let n = active_elements.len();
    let mut parent: Vec<usize> = (0..n).collect();

    fn find(parent: &mut [usize], i: usize) -> usize {
        if parent[i] != i {
            parent[i] = find(parent, parent[i]);
        }
        parent[i]
    }
    fn union(parent: &mut [usize], a: usize, b: usize) {
        let ra = find(parent, a);
        let rb = find(parent, b);
        if ra != rb {
            parent[rb] = ra;
        }
    }

    // For each active element: BFS from output, check if input is reachable
    // through signal edges (excluding the element's own edge).
    // Any other active element on the path → union with this one.
    for i in 0..n {
        let elem = &active_elements[i];

        // Skip if both terminals are rails (bias-only element)
        if rails.contains(&elem.input_node) || rails.contains(&elem.output_node) {
            continue;
        }

        let (reachable_nodes, _reachable_edges) =
            bfs_reachable(elem.output_node, elem.edge_idx, &adj);

        // Does the BFS reach this element's own input? → self-feedback
        // (Always true for op-amps with feedback resistors)
        let has_self_feedback = reachable_nodes.contains(&elem.input_node);

        if has_self_feedback {
            // Union with active elements whose BOTH non-rail terminals
            // are on the reachable path. Ground-shunt elements (one
            // terminal at rail) don't couple through the rail.
            for j in 0..n {
                if i == j {
                    continue;
                }
                let other = &active_elements[j];
                let in_ok = reachable_nodes.contains(&other.input_node)
                    && !rails.contains(&other.input_node);
                let out_ok = reachable_nodes.contains(&other.output_node)
                    && !rails.contains(&other.output_node);
                if in_ok && out_ok {
                    union(&mut parent, i, j);
                }
            }
        }
    }

    // Collect edges per group
    // Each active element maps to a group. Passive edges are assigned
    // to a group if they touch a signal node owned by that group.
    let mut group_map: HashMap<usize, Vec<usize>> = HashMap::new();
    for i in 0..n {
        let root = find(&mut parent, i);
        group_map
            .entry(root)
            .or_default()
            .push(active_elements[i].edge_idx);
    }

    // For each group, find passive edges on the FEEDBACK PATH:
    // edges on any simple path from input to output through signal edges.
    // This excludes downstream cascade and input pendant edges.
    let mut claimed: HashSet<usize> = HashSet::new();
    let mut groups: Vec<FeedbackGroup> = Vec::new();

    for (&_root, active_edge_indices) in &group_map {
        let mut group_edges: HashSet<usize> = HashSet::new();
        group_edges.extend(active_edge_indices);

        // Collect all (input, output) terminal pairs for this group
        let group_elem_indices: Vec<usize> = (0..n)
            .filter(|&i| find(&mut parent, i) == find(&mut parent, _root))
            .collect();

        // DFS from each input terminal to its output terminal.
        // All edges on successful paths → feedback edges.
        let mut has_feedback = false;
        for &elem_idx in &group_elem_indices {
            let elem = &active_elements[elem_idx];
            if rails.contains(&elem.input_node) || rails.contains(&elem.output_node) {
                continue;
            }
            // Find all edges on paths from output back to input
            // (excluding the active element's own edge)
            let mut feedback_edges: HashSet<usize> = HashSet::new();
            let mut path: Vec<usize> = Vec::new();
            let mut visited: HashSet<NodeId> = HashSet::new();
            visited.insert(elem.output_node);
            dfs_find_paths(
                elem.output_node,
                elem.input_node,
                elem.edge_idx,
                &adj,
                &mut visited,
                &mut path,
                &mut feedback_edges,
            );
            has_feedback = has_feedback || !feedback_edges.is_empty();
            group_edges.extend(feedback_edges);
        }

        // Only claim passive edges if this group has actual feedback.
        // Standalone NL elements (diodes to GND) don't get passive edges —
        // they're independent clipping stages.
        if !has_feedback {
            for &eidx in &group_edges {
                claimed.insert(eidx);
            }
            groups.push(FeedbackGroup {
                edge_indices: group_edges.into_iter().collect(),
            });
            continue;
        }

        // Claim two more edge categories (passive only):
        //
        // 1. GROUND SHUNTS: passive edge with one end at rail, other at a
        //    group node. Creates resonance (caps) or loading (resistors).
        //
        // 2. INPUT PENDANTS: passive edge touching an active element's
        //    input terminal. Determines Ri for gain computation.
        //
        // Everything else stays standalone (cascade, output load, etc.)
        let group_nodes: HashSet<NodeId> = group_edges
            .iter()
            .flat_map(|&eidx| {
                let e = &graph.edges[eidx];
                [e.node_a, e.node_b].into_iter().filter(|n| !rails.contains(n))
            })
            .collect();

        // Collect input terminal nodes for pendant detection
        let input_terminals: HashSet<NodeId> = group_elem_indices
            .iter()
            .filter_map(|&i| {
                let node = active_elements[i].input_node;
                if rails.contains(&node) { None } else { Some(node) }
            })
            .collect();

        for &eidx in edge_indices {
            if group_edges.contains(&eidx) || claimed.contains(&eidx) {
                continue;
            }
            let comp = &graph.components[graph.edges[eidx].comp_idx];
            if !matches!(comp.kind.signal_terminals(), SignalTerminals::Passive) {
                continue; // Only claim passive edges
            }
            let e = &graph.edges[eidx];
            let a_rail = rails.contains(&e.node_a);
            let b_rail = rails.contains(&e.node_b);

            // Ground shunt: one end rail, other end in group
            let is_ground_shunt = (a_rail && group_nodes.contains(&e.node_b))
                || (b_rail && group_nodes.contains(&e.node_a));

            // Input pendant: touches an active input terminal
            let is_input_pendant = input_terminals.contains(&e.node_a)
                || input_terminals.contains(&e.node_b);

            if is_ground_shunt || is_input_pendant {
                group_edges.insert(eidx);
            }
        }

        for &eidx in &group_edges {
            claimed.insert(eidx);
        }
        groups.push(FeedbackGroup {
            edge_indices: group_edges.into_iter().collect(),
        });
    }

    // Unclaimed edges → individual groups (cascaded passive stages)
    for &eidx in edge_indices {
        if !claimed.contains(&eidx) {
            let e = &graph.edges[eidx];
            if rails.contains(&e.node_a) || rails.contains(&e.node_b) {
                groups.push(FeedbackGroup {
                    edge_indices: vec![eidx],
                });
            } else {
                // Signal edge not in any feedback group — standalone
                groups.push(FeedbackGroup {
                    edge_indices: vec![eidx],
                });
            }
        }
    }

    groups
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    fn make_graph_all_edges(pedal_src: &str) -> (CircuitGraph, Vec<usize>) {
        let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse failed");
        let graph = CircuitGraph::from_pedal(&pedal);
        let active_set: std::collections::HashSet<usize> =
            graph.active_edge_indices.iter().copied().collect();
        let all_edges: Vec<usize> = (0..graph.edges.len())
            .filter(|i| !active_set.contains(i))
            .collect();
        (graph, all_edges)
    }

    fn group_summary(groups: &[FeedbackGroup], graph: &CircuitGraph) -> Vec<String> {
        groups
            .iter()
            .map(|g| {
                let names: Vec<String> = g
                    .edge_indices
                    .iter()
                    .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.clone())
                    .collect();
                let mut names = names;
                names.sort();
                names.join("+")
            })
            .collect()
    }

    #[test]
    fn rat_splits_into_stages() {
        // RAT: opamp gain → diodes to GND → tone → volume
        let (graph, edges) = make_graph_all_edges(r#"
            pedal "test" { supply 9V
                components {
                    R_in: resistor(1k)
                    U1: opamp(lm308)
                    R_fb: resistor(47k)
                    C_hpf: cap(100p)
                    D1: diode(silicon)
                    D2: diode(silicon)
                    C_tone: cap(3.3n)
                    R_tone: resistor(1.5k)
                    R_out: resistor(10k)
                }
                nets {
                    in -> R_in.a
                    R_in.b -> U1.neg
                    U1.neg -> R_fb.a
                    R_fb.b -> U1.out
                    U1.neg -> C_hpf.a
                    C_hpf.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> D1.a
                    D1.b -> gnd
                    U1.out -> D2.b
                    D2.a -> gnd
                    U1.out -> C_tone.a
                    C_tone.b -> gnd
                    U1.out -> R_tone.a
                    R_tone.b -> R_out.a
                    R_out.b -> out
                }
                controls {}
            }"#);

        let groups = find_feedback_groups(&edges, &graph);
        let summary = group_summary(&groups, &graph);
        eprintln!("RAT groups: {summary:?}");

        // The op-amp + feedback should be one group
        let opamp_group = groups.iter().find(|g| {
            g.edge_indices.iter().any(|&eidx| {
                graph.effective_edge_kind(eidx) == EdgeKind::Vcvs
            })
        });
        assert!(opamp_group.is_some(), "Should have an op-amp group");

        let opamp_edges = &opamp_group.unwrap().edge_indices;
        // Feedback group should contain: U1, R_fb, C_hpf, R_in
        // Should NOT contain: D1, D2, C_tone (ground shunts)
        let has_rfb = opamp_edges.iter().any(|&eidx| {
            graph.components[graph.edges[eidx].comp_idx].id == "R_fb"
        });
        let has_d1 = opamp_edges.iter().any(|&eidx| {
            graph.components[graph.edges[eidx].comp_idx].id == "D1"
        });
        assert!(has_rfb, "Op-amp group should include R_fb (feedback)");
        assert!(!has_d1, "Op-amp group should NOT include D1 (ground shunt)");

        // Should have more than 1 group (multi-stage)
        assert!(groups.len() >= 3, "RAT should split into ≥3 groups, got {}", groups.len());
    }

    #[test]
    fn fuzz_face_stays_coupled() {
        // Two BJTs with feedback between them → one group
        let (graph, edges) = make_graph_all_edges(r#"
            pedal "test" { supply 9V
                components {
                    R1: resistor(33k)
                    R2: resistor(8.2k)
                    R3: resistor(100k)
                    R4: resistor(470)
                    C1: cap(2.2u)
                    C2: cap(22u)
                    Q1: npn(2n3904)
                    Q2: npn(2n3904)
                }
                nets {
                    in -> C1.a
                    C1.b -> R1.a
                    R1.b -> Q1.base
                    Q1.collector -> R2.a, Q2.base
                    R2.b -> vcc
                    Q1.emitter -> gnd
                    Q2.collector -> R3.a
                    R3.b -> vcc
                    Q2.emitter -> R4.a, Q1.base
                    R4.b -> gnd
                    Q2.collector -> C2.a
                    C2.b -> out
                }
                controls {}
            }"#);

        let groups = find_feedback_groups(&edges, &graph);
        let summary = group_summary(&groups, &graph);
        eprintln!("Fuzz Face groups: {summary:?}");

        // Q1 and Q2 should be in the SAME group (feedback: Q2.emitter → Q1.base)
        let q1_group = groups.iter().position(|g| {
            g.edge_indices.iter().any(|&eidx| {
                graph.components[graph.edges[eidx].comp_idx].id == "Q1"
            })
        });
        let q2_group = groups.iter().position(|g| {
            g.edge_indices.iter().any(|&eidx| {
                graph.components[graph.edges[eidx].comp_idx].id == "Q2"
            })
        });
        assert_eq!(
            q1_group, q2_group,
            "Q1 and Q2 should be in same feedback group"
        );
    }

    #[test]
    fn cascaded_diodes_are_separate() {
        // Two diodes in series (no feedback) → separate groups
        let (graph, edges) = make_graph_all_edges(r#"
            pedal "test" { supply 9V
                components {
                    R1: resistor(4.7k)
                    D1: diode(silicon)
                    R2: resistor(4.7k)
                    D2: diode(silicon)
                }
                nets {
                    in -> R1.a
                    R1.b -> D1.a
                    D1.b -> R2.a
                    R2.b -> D2.a
                    D2.b -> out
                }
                controls {}
            }"#);

        let groups = find_feedback_groups(&edges, &graph);
        eprintln!("Cascaded diodes: {} groups", groups.len());

        // D1 output doesn't reach D1 input → separate groups
        let d1_group = groups.iter().position(|g| {
            g.edge_indices.iter().any(|&eidx| {
                graph.components[graph.edges[eidx].comp_idx].id == "D1"
            })
        });
        let d2_group = groups.iter().position(|g| {
            g.edge_indices.iter().any(|&eidx| {
                graph.components[graph.edges[eidx].comp_idx].id == "D2"
            })
        });
        assert_ne!(
            d1_group, d2_group,
            "Cascaded diodes should be in separate groups"
        );
    }

    #[test]
    fn ground_shunt_diodes_are_separate() {
        // Two diodes to GND from the same node — NOT coupled
        let (graph, edges) = make_graph_all_edges(r#"
            pedal "test" { supply 9V
                components {
                    R1: resistor(4.7k)
                    D1: diode(silicon)
                    D2: diode(silicon)
                }
                nets {
                    in -> R1.a
                    R1.b -> D1.a, D2.b
                    D1.b -> gnd
                    D2.a -> gnd
                }
                controls {}
            }"#);

        let groups = find_feedback_groups(&edges, &graph);
        eprintln!("Ground shunt diodes: {} groups", groups.len());

        // D1 and D2 both go to GND — GND is a rail, not a coupling node
        // They should NOT be in the same feedback group
        let d1_group = groups.iter().position(|g| {
            g.edge_indices.iter().any(|&eidx| {
                graph.components[graph.edges[eidx].comp_idx].id == "D1"
            })
        });
        let d2_group = groups.iter().position(|g| {
            g.edge_indices.iter().any(|&eidx| {
                graph.components[graph.edges[eidx].comp_idx].id == "D2"
            })
        });
        assert_ne!(
            d1_group, d2_group,
            "Ground shunt diodes should be separate (GND is a rail)"
        );
    }
}
