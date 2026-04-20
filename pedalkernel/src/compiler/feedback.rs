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

/// A group of edges that share a feedback loop, with classification.
///
/// The feedback analysis classifies each edge once. Downstream builders
/// (OpAmpRoot, IIR, General) read the classification directly — no
/// re-scanning or second DFS needed.
#[derive(Debug, Clone)]
pub(in crate::compiler) struct FeedbackGroup {
    /// Active element edges (VCVS, diode, BJT) — the core of the stage.
    pub active_edges: Vec<usize>,
    /// Passive edges on the feedback cycle (out→in path). Determines Rf.
    pub feedback_edges: Vec<usize>,
    /// Passive edges touching an active input terminal. Determines Ri.
    pub pendant_edges: Vec<usize>,
    /// Passive ground shunts touching group signal nodes. Loading/resonance.
    pub ground_shunt_edges: Vec<usize>,
}

impl FeedbackGroup {
    /// All edge indices in this group (all categories combined).
    pub fn all_edges(&self) -> Vec<usize> {
        let mut all = Vec::with_capacity(
            self.active_edges.len()
                + self.feedback_edges.len()
                + self.pendant_edges.len()
                + self.ground_shunt_edges.len(),
        );
        all.extend(&self.active_edges);
        all.extend(&self.feedback_edges);
        all.extend(&self.pendant_edges);
        all.extend(&self.ground_shunt_edges);
        all
    }

    /// Total number of edges.
    pub fn len(&self) -> usize {
        self.active_edges.len()
            + self.feedback_edges.len()
            + self.pendant_edges.len()
            + self.ground_shunt_edges.len()
    }

    /// Whether this group has feedback (cycle through active element).
    pub fn has_feedback(&self) -> bool {
        !self.feedback_edges.is_empty()
    }
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
            active_edges: Vec::new(),
            feedback_edges: Vec::new(),
            pendant_edges: edge_indices.to_vec(),
            ground_shunt_edges: Vec::new(),
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

    // Collect active edges per group (from union-find)
    let mut group_map: HashMap<usize, Vec<usize>> = HashMap::new();
    for i in 0..n {
        let root = find(&mut parent, i);
        group_map.entry(root).or_default().push(i);
    }

    // Build classified FeedbackGroup for each union-find group.
    // One DFS, one pass — all edge classification happens here.
    let mut claimed: HashSet<usize> = HashSet::new();
    let mut groups: Vec<FeedbackGroup> = Vec::new();

    for (&_root, elem_indices) in &group_map {
        let active_edges: Vec<usize> = elem_indices
            .iter()
            .map(|&i| active_elements[i].edge_idx)
            .collect();

        // DFS: find passive edges on the feedback cycle (out→in paths)
        let mut feedback_set: HashSet<usize> = HashSet::new();
        for &ei in elem_indices {
            let elem = &active_elements[ei];
            if rails.contains(&elem.input_node) || rails.contains(&elem.output_node) {
                continue;
            }
            let mut path = Vec::new();
            let mut visited = HashSet::new();
            visited.insert(elem.output_node);
            dfs_find_paths(
                elem.output_node,
                elem.input_node,
                elem.edge_idx,
                &adj,
                &mut visited,
                &mut path,
                &mut feedback_set,
            );
        }

        // Remove active element edges from feedback_set (they go in active_edges)
        for &ae in &active_edges {
            feedback_set.remove(&ae);
        }
        let feedback_edges: Vec<usize> = feedback_set.into_iter().collect();
        let has_feedback = !feedback_edges.is_empty();

        // If no feedback, this is a standalone NL element — no passive claiming
        if !has_feedback {
            for &eidx in &active_edges {
                claimed.insert(eidx);
            }
            groups.push(FeedbackGroup {
                active_edges,
                feedback_edges: Vec::new(),
                pendant_edges: Vec::new(),
                ground_shunt_edges: Vec::new(),
            });
            continue;
        }

        // Collect group signal nodes (from active + feedback edges)
        let mut group_nodes: HashSet<NodeId> = HashSet::new();
        for &eidx in active_edges.iter().chain(feedback_edges.iter()) {
            let e = &graph.edges[eidx];
            if !rails.contains(&e.node_a) { group_nodes.insert(e.node_a); }
            if !rails.contains(&e.node_b) { group_nodes.insert(e.node_b); }
        }

        // Input terminal nodes for pendant detection
        let input_terminals: HashSet<NodeId> = elem_indices
            .iter()
            .filter_map(|&i| {
                let node = active_elements[i].input_node;
                if rails.contains(&node) { None } else { Some(node) }
            })
            .collect();

        // Classify remaining passive edges: pendant or ground shunt
        let mut pendant_edges = Vec::new();
        let mut ground_shunt_edges = Vec::new();

        for &eidx in edge_indices {
            if claimed.contains(&eidx)
                || active_edges.contains(&eidx)
                || feedback_edges.contains(&eidx)
            {
                continue;
            }
            let comp = &graph.components[graph.edges[eidx].comp_idx];
            if !matches!(comp.kind.signal_terminals(), SignalTerminals::Passive) {
                continue;
            }
            let e = &graph.edges[eidx];
            let a_rail = rails.contains(&e.node_a);
            let b_rail = rails.contains(&e.node_b);

            if (a_rail && group_nodes.contains(&e.node_b))
                || (b_rail && group_nodes.contains(&e.node_a))
            {
                ground_shunt_edges.push(eidx);
            } else if input_terminals.contains(&e.node_a)
                || input_terminals.contains(&e.node_b)
            {
                pendant_edges.push(eidx);
            }
        }

        // Mark all claimed
        for &eidx in active_edges
            .iter()
            .chain(&feedback_edges)
            .chain(&pendant_edges)
            .chain(&ground_shunt_edges)
        {
            claimed.insert(eidx);
        }

        groups.push(FeedbackGroup {
            active_edges,
            feedback_edges,
            pendant_edges,
            ground_shunt_edges,
        });
    }

    // Unclaimed edges → individual standalone groups
    for &eidx in edge_indices {
        if !claimed.contains(&eidx) {
            {
                groups.push(FeedbackGroup {
                    active_edges: Vec::new(),
                    feedback_edges: Vec::new(),
                    pendant_edges: vec![eidx],
                    ground_shunt_edges: Vec::new(),
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
                let mut names: Vec<String> = g
                    .all_edges()
                    .iter()
                    .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.clone())
                    .collect();
                names.sort();
                names.dedup();
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
            g.active_edges.iter().any(|&eidx| {
                graph.effective_edge_kind(eidx) == EdgeKind::Vcvs
            })
        });
        assert!(opamp_group.is_some(), "Should have an op-amp group");
        let og = opamp_group.unwrap();

        // R_fb should be in feedback_edges (on the out→neg cycle)
        let has_rfb = og.feedback_edges.iter().any(|&eidx| {
            graph.components[graph.edges[eidx].comp_idx].id == "R_fb"
        });
        assert!(has_rfb, "R_fb should be in feedback_edges");

        // R_in should be in pendant_edges (input coupling, not feedback)
        let has_rin = og.pendant_edges.iter().any(|&eidx| {
            graph.components[graph.edges[eidx].comp_idx].id == "R_in"
        });
        assert!(has_rin, "R_in should be in pendant_edges");

        // D1 should NOT be in any category of this group
        let all = og.all_edges();
        let has_d1 = all.iter().any(|&eidx| {
            graph.components[graph.edges[eidx].comp_idx].id == "D1"
        });
        assert!(!has_d1, "D1 should NOT be in op-amp group");

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
            g.all_edges().iter().any(|&eidx| {
                graph.components[graph.edges[eidx].comp_idx].id == "Q1"
            })
        });
        let q2_group = groups.iter().position(|g| {
            g.all_edges().iter().any(|&eidx| {
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
            g.all_edges().iter().any(|&eidx| {
                graph.components[graph.edges[eidx].comp_idx].id == "D1"
            })
        });
        let d2_group = groups.iter().position(|g| {
            g.all_edges().iter().any(|&eidx| {
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
            g.all_edges().iter().any(|&eidx| {
                graph.components[graph.edges[eidx].comp_idx].id == "D1"
            })
        });
        let d2_group = groups.iter().position(|g| {
            g.all_edges().iter().any(|&eidx| {
                graph.components[graph.edges[eidx].comp_idx].id == "D2"
            })
        });
        assert_ne!(
            d1_group, d2_group,
            "Ground shunt diodes should be separate (GND is a rail)"
        );
    }

    // ── Distance-based direction stress tests ────────────────────────

    #[test]
    fn direction_simple_cascade_separates() {
        // Two op-amps in cascade: U1.out → R → U2.pos. No shared feedback.
        // Distance: U1 closer to input, U2 farther. Should be 2 groups.
        let (graph, edges) = make_graph_all_edges(r#"
            pedal "test" { supply 9V
                components {
                    R_in: resistor(10k)
                    U1: opamp(tl072)
                    Rf1: resistor(100k)
                    R_mid: resistor(10k)
                    U2: opamp(tl072)
                    Rf2: resistor(100k)
                    R_out: resistor(10k)
                }
                nets {
                    in -> R_in.a
                    R_in.b -> U1.neg
                    U1.neg -> Rf1.a
                    Rf1.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> R_mid.a
                    R_mid.b -> U2.neg
                    U2.neg -> Rf2.a
                    Rf2.b -> U2.out
                    U2.pos -> gnd
                    U2.out -> R_out.a
                    R_out.b -> out
                }
                controls {}
            }"#);
        let groups = find_feedback_groups(&edges, &graph);
        let summary = group_summary(&groups, &graph);
        eprintln!("Cascade: {summary:?}");

        // U1 and U2 should be in DIFFERENT groups
        let u1_group = groups.iter().position(|g| {
            g.all_edges().iter().any(|&eidx| {
                graph.components[graph.edges[eidx].comp_idx].id == "U1"
            })
        });
        let u2_group = groups.iter().position(|g| {
            g.all_edges().iter().any(|&eidx| {
                graph.components[graph.edges[eidx].comp_idx].id == "U2"
            })
        });
        assert_ne!(
            u1_group, u2_group,
            "Cascaded op-amps should be in separate groups"
        );
    }

    #[test]
    fn direction_bridged_t_stays_grouped() {
        // 808 bridged-T: R1/R2 form forward+backward paths. Must stay grouped.
        let (graph, edges) = make_graph_all_edges(r#"
            pedal "test" { supply 9V
                components {
                    U1: opamp(tl072)
                    R1: resistor(150k)
                    R2: resistor(150k)
                    C1: cap(8.2n)
                    C2: cap(8.2n)
                    R_fb: resistor(470k)
                    R_trig: resistor(100k)
                }
                nets {
                    U1.neg -> R1.a, C1.a
                    R1.b -> R2.a, C2.a
                    R2.b -> U1.out
                    C1.b -> gnd
                    C2.b -> gnd
                    U1.neg -> R_fb.a
                    R_fb.b -> U1.out
                    U1.pos -> gnd
                    in -> R_trig.a
                    R_trig.b -> R1.b
                    U1.out -> out
                }
                controls {}
            }"#);
        let groups = find_feedback_groups(&edges, &graph);
        let summary = group_summary(&groups, &graph);
        eprintln!("Bridged-T: {summary:?}");

        // U1, R1, R2, R_fb should all be in the same group
        let u1_group = groups.iter().find(|g| {
            g.all_edges().iter().any(|&eidx| {
                graph.components[graph.edges[eidx].comp_idx].id == "U1"
            })
        }).expect("Should have U1 group");

        let has_r1 = u1_group.all_edges().iter().any(|&eidx| {
            graph.components[graph.edges[eidx].comp_idx].id == "R1"
        });
        let has_r2 = u1_group.all_edges().iter().any(|&eidx| {
            graph.components[graph.edges[eidx].comp_idx].id == "R2"
        });
        assert!(has_r1, "R1 should be in U1's group (part of bridged-T)");
        assert!(has_r2, "R2 should be in U1's group (part of bridged-T)");
    }

    #[test]
    fn direction_equal_distance_parallel_paths() {
        // Two equal-length paths from neg to out: R_fb and (R1+R2 series).
        // Both should be found as feedback.
        let (graph, edges) = make_graph_all_edges(r#"
            pedal "test" { supply 9V
                components {
                    R_in: resistor(10k)
                    U1: opamp(tl072)
                    R_fb: resistor(100k)
                    R1: resistor(50k)
                    R2: resistor(50k)
                }
                nets {
                    in -> R_in.a
                    R_in.b -> U1.neg
                    U1.neg -> R_fb.a
                    R_fb.b -> U1.out
                    U1.neg -> R1.a
                    R1.b -> R2.a
                    R2.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> out
                }
                controls {}
            }"#);
        let groups = find_feedback_groups(&edges, &graph);

        let u1_group = groups.iter().find(|g| {
            g.all_edges().iter().any(|&eidx| {
                graph.components[graph.edges[eidx].comp_idx].id == "U1"
            })
        }).expect("Should have U1 group");

        // Both R_fb (direct) and R1+R2 (series chain) should be feedback
        let has_rfb = u1_group.feedback_edges.iter().any(|&eidx| {
            graph.components[graph.edges[eidx].comp_idx].id == "R_fb"
        });
        let has_r1 = u1_group.feedback_edges.iter().any(|&eidx| {
            graph.components[graph.edges[eidx].comp_idx].id == "R1"
        });
        let has_r2 = u1_group.feedback_edges.iter().any(|&eidx| {
            graph.components[graph.edges[eidx].comp_idx].id == "R2"
        });

        eprintln!("Equal-dist: fb={:?}", u1_group.feedback_edges.iter()
            .map(|&e| graph.components[graph.edges[e].comp_idx].id.clone())
            .collect::<Vec<_>>());

        assert!(has_rfb, "R_fb should be feedback");
        assert!(has_r1, "R1 should be feedback (series path)");
        assert!(has_r2, "R2 should be feedback (series path)");
    }
}
