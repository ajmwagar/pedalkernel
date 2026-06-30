//! Directed signal flow graph analysis for circuit stage partitioning.
//!
//! Replaces the undirected BFS approach in `feedback.rs` with directed
//! inter-element dependency analysis. This correctly handles multi-opamp
//! cascades (e.g., Klon Centaur) where undirected BFS incorrectly groups
//! all op-amps together.
//!
//! **Algorithm:**
//! 1. Each active element (op-amp, BJT, JFET, diode, tube) is a NODE in a
//!    directed flow graph.
//! 2. A directed edge A->B exists if A's output_node can reach B's input_node
//!    through passive signal edges (excluding rails).
//! 3. Strongly connected components (SCCs) define co-solved groups.
//! 4. Elements without mutual dependency are separate stages.

use hashbrown::HashMap;
use std::collections::{BTreeMap, HashSet, VecDeque};

use super::component::{EdgeKind, OutputImpedance, SignalTerminals};
use super::graph::{CircuitGraph, NodeId};

/// A group of mutually-dependent active elements (same stage).
#[derive(Debug, Clone)]
pub(in crate::compiler) struct FlowGroup {
    /// Active element edge indices in this group.
    pub active_edges: Vec<usize>,
    /// Passive edges classified as feedback (on cycle paths).
    pub feedback_edges: Vec<usize>,
    /// Passive edges touching input terminals (pendants / Ri).
    pub pendant_edges: Vec<usize>,
    /// Passive ground shunts touching group signal nodes.
    pub ground_shunt_edges: Vec<usize>,
}

impl FlowGroup {
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

    /// Whether this group has feedback (cycle through active element).
    pub fn has_feedback(&self) -> bool {
        !self.feedback_edges.is_empty()
    }
}

/// An active element with identified input/output terminals.
#[derive(Debug, Clone)]
struct ActiveElement {
    /// Index into graph.edges for this element's primary edge.
    edge_idx: usize,
    /// Input terminal node (where signal controls the element).
    input_node: NodeId,
    /// Output terminal node (where amplified signal leaves).
    output_node: NodeId,
    /// All output-side nodes (for BJT: both collector and emitter carry signal).
    /// Used for flow graph BFS starting points.
    output_nodes: Vec<NodeId>,
    /// Whether this element modulates an existing impedance in parallel with
    /// a passive element (NL/ControlledConductance shunt) rather than
    /// amplifying signal. Its input pin is a control pin outside the signal
    /// path, so signal-flow-distance bounds relative to it are meaningless.
    is_shunt_modulator: bool,
}

/// Set of nodes that are rails (not signal-carrying shared nodes).
pub(super) fn rail_nodes(graph: &CircuitGraph) -> HashSet<NodeId> {
    let mut rails = HashSet::new();
    rails.insert(graph.gnd_node);
    rails.insert(graph.vcc_node);
    rails.extend(&graph.supply_nodes);
    rails.extend(&graph.ac_ground_nodes);
    rails
}

/// Resolve a component's pin name to a graph node ID.
pub(super) fn resolve_pin(comp_id: &str, pin: &str, graph: &CircuitGraph) -> Option<NodeId> {
    let key = format!("{comp_id}.{pin}");
    graph.node_names.get(&key).copied()
}

/// Identify active elements and their input/output terminals.
fn find_active_elements(edge_indices: &[usize], graph: &CircuitGraph) -> Vec<ActiveElement> {
    let mut elements = Vec::new();

    for &eidx in edge_indices {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];

        match comp.kind.signal_terminals() {
            SignalTerminals::Passive => {}
            SignalTerminals::TwoPort { input, output } => {
                let in_node = resolve_pin(&comp.id, input, graph).unwrap_or(e.node_a);
                let out_node = resolve_pin(&comp.id, output, graph).unwrap_or(e.node_b);
                elements.push(ActiveElement {
                    edge_idx: eidx,
                    input_node: in_node,
                    output_node: out_node,
                    output_nodes: vec![out_node],
                    is_shunt_modulator: false,
                });
            }
            SignalTerminals::Amplifier { input, output, .. } => {
                let in_node = resolve_pin(&comp.id, input, graph).unwrap_or(e.node_a);
                let out_node = resolve_pin(&comp.id, output, graph).unwrap_or(e.node_b);
                // Detect shunt modulator: any NL/ControlledConductance device
                // whose port is in parallel with a passive element. Such devices
                // modulate an existing impedance rather than amplifying signal —
                // their secondary endpoint must not create directed SCC edges
                // that would hide the op-amp feedback topology they modulate.
                //
                // Generalizes the old BJT-specific exception to work for JFETs,
                // MOSFETs, and any future NL shunt device.
                let is_shunt_modulator = {
                    let sem = comp.kind.port_semantic(
                        comp.kind.edges().first().map_or("", |e| e.pin_a),
                        comp.kind.edges().first().map_or("", |e| e.pin_b),
                    );
                    matches!(
                        sem,
                        super::component::PortSemantic::Nonlinear
                            | super::component::PortSemantic::ControlledConductance
                    ) && device_parallels_passive(&comp.id, e, edge_indices, graph)
                };
                let mut output_nodes = vec![out_node];
                if !is_shunt_modulator {
                    // Add the other edge endpoint if it's different from output and input
                    if e.node_a != out_node && e.node_a != in_node {
                        output_nodes.push(e.node_a);
                    }
                    if e.node_b != out_node && e.node_b != in_node {
                        output_nodes.push(e.node_b);
                    }
                }
                elements.push(ActiveElement {
                    edge_idx: eidx,
                    input_node: in_node,
                    output_node: out_node,
                    output_nodes,
                    is_shunt_modulator,
                });
            }
        }
    }

    elements
}

/// Check if a device's edge is in parallel with a passive element.
///
/// A device "parallels passive" when another edge in the same group connects
/// the same two nodes and is a resistor (or other passive linear element).
/// This generalizes the old BJT-specific check to work for any NL/controlled
/// device used as a shunt modulator.
fn device_parallels_passive(
    comp_id: &str,
    edge: &super::graph::GraphEdge,
    edge_indices: &[usize],
    graph: &CircuitGraph,
) -> bool {
    let node_a = edge.node_a;
    let node_b = edge.node_b;

    edge_indices.iter().any(|&other_eidx| {
        let other = &graph.edges[other_eidx];
        let other_comp = &graph.components[other.comp_idx];
        // Skip self
        if other_comp.id == comp_id {
            return false;
        }
        // Must be a passive element (resistor, cap, inductor, pot)
        if !other_comp.kind.is_passive() {
            return false;
        }
        // Check same nodes (parallel connection)
        (other.node_a == node_a && other.node_b == node_b)
            || (other.node_a == node_b && other.node_b == node_a)
    })
}

/// Rail-shunt diode clamps are nonlinear loads, not active source boundaries.
///
/// They must remain separate stages from an op-amp, but their signal node
/// should not block passive reachability when the op-amp searches for its own
/// resistive feedback path. Otherwise a post-gain clipper connected at
/// `U1.out` hides `Rf: U1.out -> U1.neg` and the gain stage degenerates to
/// unity.
fn active_output_blocks_passive_reachability(
    elem: &ActiveElement,
    graph: &CircuitGraph,
    rails: &HashSet<NodeId>,
) -> bool {
    let edge = &graph.edges[elem.edge_idx];
    let comp = &graph.components[edge.comp_idx];
    if comp.kind.is_diode_family() && (rails.contains(&edge.node_a) || rails.contains(&edge.node_b))
    {
        return false;
    }
    true
}

/// Build undirected signal-edge adjacency (excludes rail nodes).
/// Same as feedback.rs but without virtual amplifier links — directionality
/// is handled by the flow graph, not the adjacency.
fn build_passive_adjacency(
    edge_indices: &[usize],
    graph: &CircuitGraph,
    rails: &HashSet<NodeId>,
    active_edge_set: &HashSet<usize>,
    cut_edges: &super::boundary_rules::DelayedCutSet,
) -> BTreeMap<NodeId, Vec<(usize, NodeId)>> {
    let mut adj: BTreeMap<NodeId, Vec<(usize, NodeId)>> = BTreeMap::new();
    for &eidx in edge_indices {
        // Phase 2a: skip broker-cut tap-mouth edges (mirrors the active-edge
        // skip below) so a delayed-coupling boundary can't bridge two groups.
        if cut_edges.cuts.contains_key(&eidx) {
            continue;
        }
        // Skip active element edges — we only traverse passive signal paths
        if active_edge_set.contains(&eidx) {
            continue;
        }
        let comp = &graph.components[graph.edges[eidx].comp_idx];
        if !matches!(comp.kind.signal_terminals(), SignalTerminals::Passive) {
            continue;
        }
        let e = &graph.edges[eidx];
        if rails.contains(&e.node_a) || rails.contains(&e.node_b) {
            continue;
        }
        adj.entry(e.node_a).or_default().push((eidx, e.node_b));
        adj.entry(e.node_b).or_default().push((eidx, e.node_a));
    }
    adj
}

/// BFS from `start` through passive signal edges. Returns all reachable nodes.
/// Blocks traversal through other active elements' output nodes (signal sources)
/// to prevent "backward" traversal through a cascade.
fn bfs_reachable_nodes(
    start: NodeId,
    adj: &BTreeMap<NodeId, Vec<(usize, NodeId)>>,
    blocked_outputs: &HashSet<NodeId>,
) -> HashSet<NodeId> {
    let mut visited = HashSet::new();
    let mut queue = VecDeque::new();
    visited.insert(start);
    if blocked_outputs.contains(&start) {
        return visited;
    }
    queue.push_back(start);

    while let Some(node) = queue.pop_front() {
        if let Some(neighbors) = adj.get(&node) {
            for &(_eidx, next) in neighbors {
                if visited.contains(&next) {
                    continue;
                }
                visited.insert(next);
                // Don't expand from blocked output nodes (they are signal
                // sources for other elements). We still mark them visited
                // so they appear in reachable set, but don't traverse further.
                if blocked_outputs.contains(&next) {
                    continue;
                }
                queue.push_back(next);
            }
        }
    }

    visited
}

fn edge_spans_any_output_and_input(
    edge_idx: usize,
    outputs: &[NodeId],
    input: NodeId,
    graph: &CircuitGraph,
) -> bool {
    let edge = &graph.edges[edge_idx];
    outputs.iter().any(|&out| {
        (edge.node_a == out && edge.node_b == input) || (edge.node_b == out && edge.node_a == input)
    })
}

fn edge_touches_output_and_returns_to_input(
    edge_idx: usize,
    outputs: &[NodeId],
    input: NodeId,
    adj: &BTreeMap<NodeId, Vec<(usize, NodeId)>>,
    blocked_outputs: &HashSet<NodeId>,
    graph: &CircuitGraph,
) -> bool {
    let edge = &graph.edges[edge_idx];
    outputs.iter().any(|&out| {
        let other = if edge.node_a == out {
            edge.node_b
        } else if edge.node_b == out {
            edge.node_a
        } else {
            return false;
        };
        let mut blocked = blocked_outputs.clone();
        blocked.remove(&input);
        bfs_reachable_nodes(other, adj, &blocked).contains(&input)
    })
}

/// Build the directed flow graph between active elements.
/// Returns adjacency list: flow_adj[i] = list of j where element i's output
/// can reach element j's input through passive edges.
///
/// Key insight: when checking reachability from element i's output nodes, we
/// block traversal through OTHER active elements' output nodes. This prevents
/// "backward" traversal through a cascade (e.g., U2.out -> Rf2 -> U2.neg
/// -> R_mid -> U1.out -> Rf1 -> U1.neg would incorrectly create U2->U1).
fn build_flow_graph(
    elements: &[ActiveElement],
    adj: &BTreeMap<NodeId, Vec<(usize, NodeId)>>,
    rails: &HashSet<NodeId>,
    graph: &CircuitGraph,
) -> Vec<Vec<usize>> {
    let n = elements.len();
    let mut flow_adj = vec![Vec::new(); n];

    for i in 0..n {
        let elem_i = &elements[i];

        // Block traversal through other elements' output nodes (ALL of them).
        // Additionally, block other VCVS (op-amp) elements' input nodes.
        // VCVS neg nodes act as summing junctions shared with interstage
        // coupling networks (pots, resistors). Without this blocking, BFS
        // from element i's output can traverse backward through i's own
        // feedback, through shared passives, and reach another op-amp's
        // neg node — creating a false cycle edge.
        //
        // We only block VCVS inputs, not diode/BJT inputs, because:
        // - Diodes in feedback (e.g., Screamer D1 between neg and out)
        //   MUST be reachable from the op-amp's BFS for correct coupling.
        // - BJTs with shared emitter coupling (Fuzz Face) need their
        //   input nodes reachable for mutual feedback detection.
        // - Op-amp neg nodes are the only ones that create false paths
        //   through feedback networks to upstream stages.
        // Block traversal through other elements' output nodes AND through
        // other elements whose input node acts as a summing junction
        // (feedback_input_is_barrier). Op-amp neg nodes are summing junctions
        // shared with interstage coupling — without blocking, BFS traverses
        // backward through i's feedback, across shared passives, and creates
        // false cycle edges.
        //
        // The barrier property comes from Component::feedback_input_is_barrier().
        // Don't block sibling ports of the SAME component. A BJT's B-E and C-E
        // ports share the device's output node, so blocking the sibling blocks
        // the device's own collector and severs resistor-coupled feedback
        // (Fuzz Face: Q1.collector -> R4 -> Q2.base never forms a flow edge,
        // so the loop can't become one SCC / co-solve group).
        let comp_i_idx = graph.edges[elem_i.edge_idx].comp_idx;
        let mut blocked_outputs: HashSet<NodeId> = elements
            .iter()
            .enumerate()
            .filter(|&(j, _)| j != i)
            .filter(|(_, e)| graph.edges[e.edge_idx].comp_idx != comp_i_idx)
            .filter(|(_, e)| active_output_blocks_passive_reachability(e, graph, rails))
            .flat_map(|(_, e)| {
                let mut nodes: Vec<NodeId> = e.output_nodes.clone();
                let comp = &graph.components[graph.edges[e.edge_idx].comp_idx];
                if comp.kind.feedback_input_is_barrier() {
                    nodes.push(e.input_node);
                }
                nodes
            })
            .filter(|node| !rails.contains(node))
            .collect();
        blocked_outputs.insert(graph.in_node);
        blocked_outputs.insert(graph.out_node);
        if elem_i.output_nodes.contains(&graph.out_node) {
            blocked_outputs.remove(&graph.out_node);
        }

        // BFS from ALL of element i's output nodes through passive edges
        let mut reachable = HashSet::new();
        for &out_node in &elem_i.output_nodes {
            if rails.contains(&out_node) {
                continue;
            }
            let r = bfs_reachable_nodes(out_node, adj, &blocked_outputs);
            reachable.extend(r);
        }

        // Also compute restricted reachability with i's own input blocked.
        // This prevents false backward paths where BFS traverses through i's
        // feedback network, across interstage coupling, and reaches another
        // barrier element's input (i.out → i.feedback → i.neg → pot → j.neg).
        // Computed for ALL elements — even non-barrier elements like diodes
        // can create false paths through shared feedback networks.
        let restricted_reachable = if !rails.contains(&elem_i.input_node) {
            let mut blocked_plus_self = blocked_outputs.clone();
            blocked_plus_self.insert(elem_i.input_node);
            let mut r = HashSet::new();
            for &out_node in &elem_i.output_nodes {
                if rails.contains(&out_node) {
                    continue;
                }
                r.extend(bfs_reachable_nodes(out_node, adj, &blocked_plus_self));
            }
            Some(r)
        } else {
            None
        };

        // Check if any other element's input is reachable.
        // For barrier targets (op-amps): use restricted reachability if
        // element i is also a barrier. This prevents false cycles through
        // shared feedback/interstage networks.
        for j in 0..n {
            if i == j {
                continue;
            }
            let elem_j = &elements[j];
            if rails.contains(&elem_j.input_node) {
                continue;
            }
            let comp_j = &graph.components[graph.edges[elem_j.edge_idx].comp_idx];
            let comp_i = &graph.components[graph.edges[elem_i.edge_idx].comp_idx];
            let target_is_direct_feedback_device = edge_spans_any_output_and_input(
                elem_j.edge_idx,
                &elem_i.output_nodes,
                elem_i.input_node,
                graph,
            );
            let target_is_series_feedback_device = edge_touches_output_and_returns_to_input(
                elem_j.edge_idx,
                &elem_i.output_nodes,
                elem_i.input_node,
                adj,
                &blocked_outputs,
                graph,
            );
            let source_is_direct_feedback_device = edge_spans_any_output_and_input(
                elem_i.edge_idx,
                &elem_j.output_nodes,
                elem_j.input_node,
                graph,
            );
            let source_is_series_feedback_device = edge_touches_output_and_returns_to_input(
                elem_i.edge_idx,
                &elem_j.output_nodes,
                elem_j.input_node,
                adj,
                &blocked_outputs,
                graph,
            );
            let use_restricted = restricted_reachable.is_some()
                && (comp_j.kind.feedback_input_is_barrier()
                    || (comp_i.kind.feedback_input_is_barrier()
                        && !target_is_direct_feedback_device
                        && !target_is_series_feedback_device));
            if source_is_direct_feedback_device || source_is_series_feedback_device {
                flow_adj[i].push(j);
                continue;
            }
            let reach = if use_restricted {
                restricted_reachable.as_ref().unwrap()
            } else {
                &reachable
            };
            if reach.contains(&elem_j.input_node) {
                flow_adj[i].push(j);
            }
        }
    }

    flow_adj
}

/// Find strongly connected components using Tarjan's algorithm.
/// Returns groups of element indices that form SCCs.
fn tarjan_scc(flow_adj: &[Vec<usize>]) -> Vec<Vec<usize>> {
    let n = flow_adj.len();
    let mut index_counter: usize = 0;
    let mut stack: Vec<usize> = Vec::new();
    let mut on_stack = vec![false; n];
    let mut index = vec![usize::MAX; n];
    let mut lowlink = vec![0usize; n];
    let mut sccs: Vec<Vec<usize>> = Vec::new();

    fn strongconnect(
        v: usize,
        flow_adj: &[Vec<usize>],
        index_counter: &mut usize,
        stack: &mut Vec<usize>,
        on_stack: &mut [bool],
        index: &mut [usize],
        lowlink: &mut [usize],
        sccs: &mut Vec<Vec<usize>>,
    ) {
        index[v] = *index_counter;
        lowlink[v] = *index_counter;
        *index_counter += 1;
        stack.push(v);
        on_stack[v] = true;

        for &w in &flow_adj[v] {
            if index[w] == usize::MAX {
                strongconnect(
                    w,
                    flow_adj,
                    index_counter,
                    stack,
                    on_stack,
                    index,
                    lowlink,
                    sccs,
                );
                lowlink[v] = lowlink[v].min(lowlink[w]);
            } else if on_stack[w] {
                lowlink[v] = lowlink[v].min(index[w]);
            }
        }

        if lowlink[v] == index[v] {
            let mut scc = Vec::new();
            loop {
                let w = stack.pop().unwrap();
                on_stack[w] = false;
                scc.push(w);
                if w == v {
                    break;
                }
            }
            sccs.push(scc);
        }
    }

    for v in 0..n {
        if index[v] == usize::MAX {
            strongconnect(
                v,
                flow_adj,
                &mut index_counter,
                &mut stack,
                &mut on_stack,
                &mut index,
                &mut lowlink,
                &mut sccs,
            );
        }
    }

    sccs
}

/// DFS: find all edges on any simple path from `current` to `target`.
fn dfs_find_paths(
    current: NodeId,
    target: NodeId,
    adj: &BTreeMap<NodeId, Vec<(usize, NodeId)>>,
    visited: &mut HashSet<NodeId>,
    path: &mut Vec<usize>,
    result: &mut HashSet<usize>,
) {
    if let Some(neighbors) = adj.get(&current) {
        for &(eidx, next) in neighbors {
            if next == target {
                result.insert(eidx);
                result.extend(path.iter());
                continue;
            }
            if visited.contains(&next) {
                continue;
            }
            visited.insert(next);
            path.push(eidx);
            dfs_find_paths(next, target, adj, visited, path, result);
            path.pop();
            visited.remove(&next);
        }
    }
}

/// DFS variant for feedback extraction. Barrier nodes are legal endpoints only
/// when they are the target input; otherwise they are stage boundaries and must
/// not be expanded through.
fn dfs_find_paths_blocked(
    current: NodeId,
    target: NodeId,
    adj: &BTreeMap<NodeId, Vec<(usize, NodeId)>>,
    blocked: &HashSet<NodeId>,
    visited: &mut HashSet<NodeId>,
    path: &mut Vec<usize>,
    result: &mut HashSet<usize>,
) {
    if current != target && blocked.contains(&current) {
        return;
    }

    if let Some(neighbors) = adj.get(&current) {
        for &(eidx, next) in neighbors {
            if next == target {
                result.insert(eidx);
                result.extend(path.iter());
                continue;
            }
            if blocked.contains(&next) || visited.contains(&next) {
                continue;
            }
            visited.insert(next);
            path.push(eidx);
            dfs_find_paths_blocked(next, target, adj, blocked, visited, path, result);
            path.pop();
            visited.remove(&next);
        }
    }
}

/// Merge SCCs whose active elements share a `comp_idx`.
///
/// Multi-terminal devices (BJTs, MOSFETs) have multiple edges in the flow graph.
/// Without feedback, Tarjan places them in separate SCCs. This merges them so
/// the device stays in one FlowGroup.
fn merge_same_component_sccs(
    sccs: Vec<Vec<usize>>,
    active_elements: &[ActiveElement],
    graph: &CircuitGraph,
) -> Vec<Vec<usize>> {
    if sccs.len() <= 1 {
        return sccs;
    }

    // Map each SCC to a merge-group ID via union-find on comp_idx
    let mut group_of: Vec<usize> = (0..sccs.len()).collect(); // identity

    fn find(group_of: &mut [usize], mut i: usize) -> usize {
        while group_of[i] != i {
            group_of[i] = group_of[group_of[i]];
            i = group_of[i];
        }
        i
    }
    fn union(group_of: &mut [usize], a: usize, b: usize) {
        let ra = find(group_of, a);
        let rb = find(group_of, b);
        if ra != rb {
            group_of[rb] = ra;
        }
    }

    // Build comp_idx → first SCC index map
    let mut comp_to_scc: std::collections::BTreeMap<usize, usize> =
        std::collections::BTreeMap::new();
    for (si, scc) in sccs.iter().enumerate() {
        for &elem_idx in scc {
            let eidx = active_elements[elem_idx].edge_idx;
            let comp_idx = graph.edges[eidx].comp_idx;
            if let Some(&prev_si) = comp_to_scc.get(&comp_idx) {
                union(&mut group_of, si, prev_si);
            } else {
                comp_to_scc.insert(comp_idx, si);
            }
        }
    }

    // Collect merged SCCs (BTreeMap for deterministic iteration order)
    let mut merged: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (si, scc) in sccs.into_iter().enumerate() {
        let root = find(&mut group_of, si);
        merged.entry(root).or_default().extend(scc);
    }

    merged.into_values().collect()
}

/// Merge SCCs whose active devices share a non-rail circuit node.
///
/// Directly node-coupled transistor structures such as current mirrors,
/// differential pairs, and Wilson mirrors exchange bias/control currents
/// through active terminals, not through passive signal edges. If those active
/// devices are split into serial stages, a bias pot can mutate one stage while
/// the audio-controlling device in the next stage never sees that operating
/// point change.
fn merge_shared_active_node_sccs(
    sccs: Vec<Vec<usize>>,
    active_elements: &[ActiveElement],
    graph: &CircuitGraph,
    rails: &HashSet<NodeId>,
) -> Vec<Vec<usize>> {
    if sccs.len() <= 1 {
        return sccs;
    }

    let mut group_of: Vec<usize> = (0..sccs.len()).collect();

    fn find(group_of: &mut [usize], mut i: usize) -> usize {
        while group_of[i] != i {
            group_of[i] = group_of[group_of[i]];
            i = group_of[i];
        }
        i
    }
    fn union(group_of: &mut [usize], a: usize, b: usize) {
        let ra = find(group_of, a);
        let rb = find(group_of, b);
        if ra != rb {
            group_of[rb] = ra;
        }
    }

    let mut elem_to_scc = vec![0usize; active_elements.len()];
    for (si, scc) in sccs.iter().enumerate() {
        for &elem_idx in scc {
            elem_to_scc[elem_idx] = si;
        }
    }

    let mut node_to_scc: BTreeMap<NodeId, usize> = BTreeMap::new();
    for (elem_idx, elem) in active_elements.iter().enumerate() {
        let comp = &graph.components[graph.edges[elem.edge_idx].comp_idx];
        if !comp.kind.is_bjt() {
            continue;
        }
        let si = elem_to_scc[elem_idx];
        for node in elem
            .output_nodes
            .iter()
            .copied()
            .chain(std::iter::once(elem.input_node))
        {
            if rails.contains(&node) || node == graph.in_node || node == graph.out_node {
                continue;
            }
            if let Some(&prev_si) = node_to_scc.get(&node) {
                union(&mut group_of, si, prev_si);
            } else {
                node_to_scc.insert(node, si);
            }
        }
    }

    let mut merged: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (si, scc) in sccs.into_iter().enumerate() {
        let root = find(&mut group_of, si);
        merged.entry(root).or_default().extend(scc);
    }

    merged.into_values().collect()
}

/// Signal-path BFS hop distance from `graph.in_node`.
///
/// Walks all circuit edges but never expands THROUGH rail nodes (GND, VCC,
/// supplies, AC grounds): rails reached from the input get a distance but do
/// not propagate it, so the result measures distance along signal paths
/// rather than through the ground plane.
///
/// Used to bound passive claiming (F10): a non-rail node strictly closer to
/// the circuit input than an active device's input pin lies upstream of the
/// device — claiming edges toward it would swallow the upstream network
/// (e.g. a passive EQ between `in` and a makeup stage) into the active
/// group, flattening its response and freezing its pots. Nodes with no
/// signal-path distance (disconnected subcircuits like LFOs, control pins
/// biased only from rails) are absent from the map and are never bounded.
pub(in crate::compiler) fn bfs_distances_from_in_node(
    graph: &CircuitGraph,
) -> HashMap<NodeId, usize> {
    let rails = rail_nodes(graph);
    let mut visited: HashMap<NodeId, usize> = HashMap::new();
    let mut queue: VecDeque<NodeId> = VecDeque::new();
    visited.insert(graph.in_node, 0);
    queue.push_back(graph.in_node);
    while let Some(node) = queue.pop_front() {
        if rails.contains(&node) && node != graph.in_node {
            continue;
        }
        let dist = visited[&node];
        for e in &graph.edges {
            let other = if e.node_a == node {
                e.node_b
            } else if e.node_b == node {
                e.node_a
            } else {
                continue;
            };
            if !visited.contains_key(&other) {
                visited.insert(other, dist + 1);
                queue.push_back(other);
            }
        }
    }
    visited
}

/// Rail-blocked, signal-DIRECTED hop distance from the circuit input.
///
/// Unlike [`bfs_distances_from_in_node`] (rail-blocked but UNDIRECTED — it walks
/// every edge both ways, including straight through an active device and along
/// supply-tied bias resistors), this traversal models the *signal* path:
///
/// * Rail nodes (gnd / supply / ac-ground) are dead-ends — never expanded.
/// * Passive edges conduct both directions (a passive network is reciprocal).
/// * Active devices conduct only `input_node -> output_nodes` (forward signal),
///   never output->input and never input->input.
///
/// This is the Defect-B fix base distance: plate-/collector-load resistors tie
/// to B+, so an undirected rail-unblocked walk reaches a tube's downstream plate
/// node in a couple of hops *through the supply rail*, giving the plate-load node
/// a tiny distance and scrambling serial stage order. With rails dead-ended and
/// devices crossed input->output only, a node downstream of a tube is reachable
/// only THROUGH that tube, so it gets a correctly larger distance.
///
/// Adjacency mirrors [`forward_signal_nodes_to_out`] but forward (in->out)
/// instead of reverse (out<-in), and over ALL active devices (no group exclusion
/// — every device is a legitimate forward conductor when measuring global flow).
pub(in crate::compiler) fn directed_signal_distances_from_in(
    graph: &CircuitGraph,
) -> HashMap<NodeId, usize> {
    let rails = rail_nodes(graph);
    let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
    let active_elements = find_active_elements(&all_edges, graph);

    // Forward signal adjacency: node -> nodes reachable in one directed hop.
    let mut fwd: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
    let mut add = |from: NodeId, to: NodeId| {
        if !rails.contains(&from) && !rails.contains(&to) {
            fwd.entry(from).or_default().push(to);
        }
    };
    let active_edge_set: HashSet<usize> = active_elements.iter().map(|e| e.edge_idx).collect();
    // Passive edges conduct both directions.
    for (eidx, e) in graph.edges.iter().enumerate() {
        if active_edge_set.contains(&eidx) {
            continue;
        }
        let comp = &graph.components[e.comp_idx];
        if !matches!(comp.kind.signal_terminals(), SignalTerminals::Passive) {
            continue;
        }
        add(e.node_a, e.node_b);
        add(e.node_b, e.node_a);
    }
    // Active devices conduct input -> each output node only.
    for elem in &active_elements {
        for &out in &elem.output_nodes {
            add(elem.input_node, out);
        }
    }
    // A plain two-port transformer couples primary↔secondary through magnetic
    // flux — a link recorded in `coupled_nodes`, NOT in `graph.edges`, so the
    // edge scan above never sees it. Without it, a mid-chain transformer whose
    // secondary feeds a SEPARATE downstream stage leaves that whole secondary
    // side unreachable from `in`: it gets no distance, so the stage-ordering
    // pass sorts it BEFORE the stages that actually feed it (LA-2A's output
    // group sorting ahead of V1). The broker's `Tight` coupled-link rule says
    // the winding pair is a traversable signal connection; cross it in both
    // directions so the secondary side is reachable from `in`. (Consults the
    // broker only; gated to plain two-ports whose secondary isn't `out`.)
    for (node, others) in graph.coupled_nodes.iter() {
        for &other in others {
            if super::boundary_rules::is_tight_coupled_link(graph, *node, other) {
                add(*node, other);
            }
        }
    }

    let mut visited: HashMap<NodeId, usize> = HashMap::new();
    let mut queue: VecDeque<NodeId> = VecDeque::new();
    visited.insert(graph.in_node, 0);
    queue.push_back(graph.in_node);
    while let Some(node) = queue.pop_front() {
        let dist = visited[&node];
        if let Some(neighbors) = fwd.get(&node) {
            for &next in neighbors {
                if !visited.contains_key(&next) {
                    visited.insert(next, dist + 1);
                    queue.push_back(next);
                }
            }
        }
    }
    visited
}

/// Set of non-rail nodes that have a directed signal path to `out` WITHOUT
/// passing back through the active group's own output nodes.
///
/// This is the formation-layer discriminator for mask 7
/// (`reports/signal-routing-formation-layer-2026-06-13.md` §1): it separates a
/// device's BIAS / operating-point branches (which terminate on a rail and
/// have no forward path to `out`) from the FORWARD-SIGNAL branch leaving the
/// same node (coupling cap -> output transformer -> load).
///
/// For a common-cathode/common-emitter stage the output node is the plate /
/// collector / drain; the cathode/emitter bias chain (Rk||Ck -> gnd) reaches
/// only rails, so none of its nodes land in this set and it is still claimed.
/// For a CATHODE FOLLOWER the output node IS the cathode, and the forward
/// branch (cathode -> coupling cap -> transformer -> load -> out) is the one
/// the heuristic could not previously distinguish; its interior nodes DO land
/// in this set, so the claim BFS treats them as a boundary and the downstream
/// transformer splits into its own stage.
///
/// The search runs BACKWARD from `out` over the directed signal graph:
/// passive edges conduct both ways; an active element conducts only
/// input_node -> output_node (so the reverse step is output_node -> input_node).
/// Traversal is seeded only at `out`/the circuit output pins and never expands
/// INTO the group's own output nodes — that boundary is what keeps the
/// device's own output node out of the set (it is the source, not downstream).
///
/// Crucially, the GROUP's OWN active elements are NOT traversed (their reverse
/// input<-output step is omitted): a branch counts as forward-signal only if
/// it reaches `out` WITHOUT re-entering this group's devices. Otherwise a
/// FEEDBACK branch (e.g. a TB303 resonance tap Q_last.emitter -> Q_first.base)
/// would be mis-classified as forward-signal — it only reaches `out` by going
/// back through the group's own transistors, which is a loop, not a forward
/// path. Such feedback branches must stay claimed into the group.
fn forward_signal_nodes_to_out(
    group_output_nodes: &HashSet<NodeId>,
    group_active_edges: &HashSet<usize>,
    active_elements: &[ActiveElement],
    graph: &CircuitGraph,
    rails: &HashSet<NodeId>,
) -> HashSet<NodeId> {
    // Reverse signal adjacency: for each node, the set of predecessor nodes
    // that can reach it in one directed signal hop.
    let mut rev: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
    let mut add = |to: NodeId, from: NodeId| {
        if !rails.contains(&to) && !rails.contains(&from) {
            rev.entry(to).or_default().push(from);
        }
    };
    let active_edge_set: HashSet<usize> = active_elements.iter().map(|e| e.edge_idx).collect();
    // Passive edges conduct both directions.
    for (eidx, e) in graph.edges.iter().enumerate() {
        if active_edge_set.contains(&eidx) {
            continue;
        }
        let comp = &graph.components[e.comp_idx];
        if !matches!(comp.kind.signal_terminals(), SignalTerminals::Passive) {
            continue;
        }
        add(e.node_a, e.node_b);
        add(e.node_b, e.node_a);
    }
    // Active elements conduct input -> output; reverse predecessor of an
    // output node is the input node. Skip this GROUP's own devices so a
    // feedback branch does not borrow them to reach `out`.
    for elem in active_elements {
        if group_active_edges.contains(&elem.edge_idx) {
            continue;
        }
        for &out in &elem.output_nodes {
            add(out, elem.input_node);
        }
    }

    // BFS backward from the circuit output(s). Seeds: out_node and output pins
    // — but NEVER a group output node. A group's own active output pin (e.g. a
    // BJT emitter in `output_pin_nodes`) is the SOURCE of the forward branch,
    // not a downstream sink; seeding from it would walk backward into the
    // device's feedback network (a TB303 resonance tap) and mis-mark it
    // forward-signal. Only true downstream sinks (`out` and OTHER devices'
    // output pins) seed the search.
    let mut seeds: Vec<NodeId> = Vec::new();
    if !rails.contains(&graph.out_node) && !group_output_nodes.contains(&graph.out_node) {
        seeds.push(graph.out_node);
    }
    for &node in &graph.output_pin_nodes {
        if !rails.contains(&node) && !group_output_nodes.contains(&node) {
            seeds.push(node);
        }
    }

    let mut visited: HashSet<NodeId> = HashSet::new();
    let mut queue: VecDeque<NodeId> = VecDeque::new();
    for s in seeds {
        if visited.insert(s) {
            queue.push_back(s);
        }
    }
    while let Some(node) = queue.pop_front() {
        if let Some(preds) = rev.get(&node) {
            for &p in preds {
                // Never expand INTO the group's own output nodes: the device
                // output is the SOURCE of the forward branch, not a downstream
                // node. Stopping here keeps the cathode/plate itself out of the
                // set so it stays a group node, while still marking everything
                // strictly downstream of it as forward-signal.
                if group_output_nodes.contains(&p) {
                    continue;
                }
                if visited.insert(p) {
                    queue.push_back(p);
                }
            }
        }
    }
    // The seeds (out/output pins) are not themselves "branch" nodes to exclude
    // from claiming, but they are already barriers, so leaving them in is
    // harmless. Group output nodes are never inserted (we never expand into
    // them, and they are not seeds in normal topologies).
    visited
}

/// Classify unclaimed passive edges as ground-shunts or pendants relative
/// to a set of active elements and their signal nodes.
///
/// Shared by both standalone (non-feedback) and feedback NL paths so that
/// passive claiming logic is not duplicated.
fn claim_passive_edges(
    scc: &[usize],
    active_elements: &[ActiveElement],
    active_edges: &[usize],
    feedback_edges: &[usize],
    edge_indices: &[usize],
    graph: &CircuitGraph,
    rails: &HashSet<NodeId>,
    claimed: &HashSet<usize>,
) -> (Vec<usize>, Vec<usize>) {
    // Collect group signal nodes from active + feedback edges
    let mut group_nodes: HashSet<NodeId> = HashSet::new();
    for &eidx in active_edges.iter().chain(feedback_edges.iter()) {
        let e = &graph.edges[eidx];
        if !rails.contains(&e.node_a) {
            group_nodes.insert(e.node_a);
        }
        if !rails.contains(&e.node_b) {
            group_nodes.insert(e.node_b);
        }
    }

    // Input terminal nodes for pendant detection.
    // Includes BOTH the feedback input (neg for op-amps) AND the signal
    // input (pos for non-inverting op-amps). The control pin carries signal
    // in non-inverting topologies — edges touching it are input coupling.
    let input_terminals: HashSet<NodeId> = scc
        .iter()
        .flat_map(|&i| {
            let elem = &active_elements[i];
            let comp = &graph.components[graph.edges[elem.edge_idx].comp_idx];
            let mut nodes = vec![elem.input_node];
            // Also include the control pin (pos for op-amps)
            if let SignalTerminals::Amplifier {
                control: Some(ctrl),
                ..
            } = comp.kind.signal_terminals()
            {
                if let Some(node) = resolve_pin(&comp.id, ctrl, graph) {
                    nodes.push(node);
                }
            }
            nodes
        })
        .filter(|node| !rails.contains(node))
        .collect();

    // Collect active element output nodes. Edges at these nodes that
    // shunt to ground are post-amplification components (tone caps,
    // coupling caps), NOT feedback network shunts. They should fall
    // through to passive grouping, not be claimed by the active group.
    let post_stage_output_nodes: HashSet<NodeId> = scc
        .iter()
        .filter_map(|&ei| {
            let elem = &active_elements[ei];
            let comp = &graph.components[graph.edges[elem.edge_idx].comp_idx];
            match comp.kind.signal_terminals() {
                SignalTerminals::Amplifier { .. } => Some(elem.output_node),
                SignalTerminals::TwoPort { .. } | SignalTerminals::Passive => None,
            }
        })
        .collect();
    let voltage_source_output_nodes: HashSet<NodeId> = scc
        .iter()
        .filter_map(|&ei| {
            let elem = &active_elements[ei];
            let comp = &graph.components[graph.edges[elem.edge_idx].comp_idx];
            if comp.kind.output_impedance() == OutputImpedance::VoltageSource
                && !rails.contains(&elem.output_node)
            {
                Some(elem.output_node)
            } else {
                None
            }
        })
        .collect();

    // ── Mask-7 forward-signal discriminator ─────────────────────────────
    //
    // A passive branch leaving an active device's OUTPUT node that has a
    // directed signal path toward `out` (coupling cap -> transformer -> load)
    // is the FORWARD-SIGNAL branch and must NOT be claimed — it is its own
    // downstream stage. Only BIAS / operating-point branches (rail-terminated,
    // no forward path to `out`) should be claimed.
    //
    // For a common-cathode/common-emitter stage the output node is the
    // plate/collector/drain, so the cathode/emitter Rk||Ck bias chain reaches
    // only rails and is NEVER in `forward_signal_nodes` — it stays claimed.
    // For a cathode follower the output node IS the cathode, so the branch
    // toward the output transformer lands in `forward_signal_nodes` and the
    // transformer splits into its own PassiveRType stage.
    // (`reports/signal-routing-formation-layer-2026-06-13.md` §1, mask 7.)
    let group_output_nodes: HashSet<NodeId> = scc
        .iter()
        .flat_map(|&ei| active_elements[ei].output_nodes.iter().copied())
        .filter(|n| !rails.contains(n))
        .collect();
    let group_active_edges: HashSet<usize> =
        active_edges.iter().chain(feedback_edges.iter()).copied().collect();
    let forward_signal_nodes = forward_signal_nodes_to_out(
        &group_output_nodes,
        &group_active_edges,
        active_elements,
        graph,
        rails,
    );
    // An edge is a forward-signal branch (do NOT claim) when its far endpoint
    // (the non-group end) is downstream toward `out`. A node IS allowed to be
    // a group node and forward at once (the device output) — we key off the
    // FAR endpoint, never the group node itself.
    let is_forward_signal_edge = |eidx: usize| -> bool {
        let e = &graph.edges[eidx];
        let a_group = group_output_nodes.contains(&e.node_a);
        let b_group = group_output_nodes.contains(&e.node_b);
        // Only branches that actually leave a group output node can be the
        // forward-signal branch in question; interior bias edges are handled
        // by the rail-termination logic below.
        if a_group && !b_group {
            forward_signal_nodes.contains(&e.node_b)
        } else if b_group && !a_group {
            forward_signal_nodes.contains(&e.node_a)
        } else {
            false
        }
    };

    // ── F10 upstream bound ──────────────────────────────────────────────
    //
    // Hop distance from the circuit input. A non-rail node strictly closer
    // to `in` than the device's input pins is part of the UPSTREAM network
    // (e.g. a passive EQ feeding a makeup stage) — claiming toward it would
    // swallow that network into this active group, flattening its response
    // and freezing its pots. Rail-terminated claims (vcc→plate, cathode→gnd,
    // grid→gnd) are never bounded — they are load-bearing bias edges for
    // every standalone tube/BJT/JFET fixture. Direct edges to `in_node`
    // itself (ordinary input coupling caps) are also never bounded: in/out
    // are already BFS barriers, so claiming them cannot swallow anything.
    // Unreachable nodes (disconnected subcircuits like LFOs) have no
    // distance and are never bounded.
    //
    // The bound is enabled only for groups made entirely of amplifier-class
    // non-BJT, non-shunt-modulator elements (op-amps, tubes, JFET/MOSFET
    // followers):
    // - BJT makeup stages compiled without `in` in their group go silent
    //   (the NlWdf voltage source sits at the global terminals), so
    //   swallowing the upstream network into a rigid whole-group stage is
    //   currently the only way such stages function (e.g. fet_leveler's
    //   class-A line amp behind the FET gain-reduction divider). Bounding
    //   them trades a working circuit for a dead one — keep old behavior
    //   until cascade injection for BJT stages lands (F10 link 2).
    // - Shunt modulators' input pins are control pins outside the signal
    //   path; distances relative to them are meaningless.
    // - Two-ports (transformers) legitimately claim their input coupling
    //   from upstream nodes.
    let bound_enabled = scc.iter().all(|&i| {
        let elem = &active_elements[i];
        let comp = &graph.components[graph.edges[elem.edge_idx].comp_idx];
        !comp.kind.is_bjt()
            && !elem.is_shunt_modulator
            && matches!(
                comp.kind.signal_terminals(),
                SignalTerminals::Amplifier { .. }
            )
    });
    let d_in = bfs_distances_from_in_node(graph);
    let input_pin_dist: Option<usize> = input_terminals
        .iter()
        .filter_map(|node| d_in.get(node).copied())
        .min();

    // Build barrier set: active input nodes of OTHER SCCs, plus circuit I/O
    // terminals. Built here (before the single-hop pass) because the F10
    // single-hop bound consults it; used below to prevent the BFS from
    // crossing into downstream active elements' domains (e.g., IC1a's BFS
    // reaching IC1b.neg through a pot).
    let scc_set: HashSet<usize> = scc.iter().copied().collect();
    let mut bfs_barriers: HashSet<NodeId> = HashSet::new();
    if !rails.contains(&graph.in_node) {
        bfs_barriers.insert(graph.in_node);
    }
    if !rails.contains(&graph.out_node) {
        bfs_barriers.insert(graph.out_node);
    }
    for &node in &graph.output_pin_nodes {
        if !voltage_source_output_nodes.contains(&node) && !rails.contains(&node) {
            bfs_barriers.insert(node);
        }
    }
    for pins in &graph.nullor_pins {
        for node in [pins.pos_node, pins.neg_node] {
            if !input_terminals.contains(&node) && !rails.contains(&node) {
                bfs_barriers.insert(node);
            }
        }
    }
    for (i, elem) in active_elements.iter().enumerate() {
        if !scc_set.contains(&i) && !rails.contains(&elem.input_node) {
            bfs_barriers.insert(elem.input_node);
        }
        // Also barrier on other elements' output nodes (voltage sources
        // like op-amp outputs are natural split points).
        if !scc_set.contains(&i) && !rails.contains(&elem.output_node) {
            bfs_barriers.insert(elem.output_node);
        }
    }

    // Feedback-loop interior nodes: reachable from this SCC's own output
    // nodes through passive edges WITHOUT passing through the device's input
    // terminals, rails, or BFS barriers. Such nodes belong to the device's
    // feedback network and are exempt from the F10 upstream bound even when
    // they sit closer to `in` than the input pins. Example: a Sallen-Key
    // follower's junction J (in→C1→J, J→C2→pos, J→R1→out) is one hop from
    // `in` but is part of the feedback loop — bounding it would split the SK
    // network into separate groups and flatten the filter. Inverting/makeup
    // stages are unaffected: their feedback resistor lands on the neg INPUT
    // terminal, where this BFS stops, so an upstream passive EQ stays
    // unreachable and remains bounded.
    let feedback_loop_nodes: HashSet<NodeId> = {
        let mut visited: HashSet<NodeId> = HashSet::new();
        let mut queue: VecDeque<NodeId> = VecDeque::new();
        let mut seed_outputs: HashSet<NodeId> = HashSet::new();
        for &i in scc {
            for &out in &active_elements[i].output_nodes {
                if !rails.contains(&out) {
                    seed_outputs.insert(out);
                    if visited.insert(out) {
                        queue.push_back(out);
                    }
                }
            }
        }
        while let Some(node) = queue.pop_front() {
            // Reachable, but never expand through barriers or the device's
            // input terminals. Seed output nodes are exempt from the
            // input-terminal stop: a unity follower has neg wired to out,
            // making its input_node coincide with the seed itself.
            if bfs_barriers.contains(&node)
                || (input_terminals.contains(&node) && !seed_outputs.contains(&node))
            {
                continue;
            }
            for &eidx in edge_indices {
                if active_edges.contains(&eidx) {
                    continue;
                }
                let e = &graph.edges[eidx];
                let other = if e.node_a == node {
                    e.node_b
                } else if e.node_b == node {
                    e.node_a
                } else {
                    continue;
                };
                if rails.contains(&other) {
                    continue;
                }
                if visited.insert(other) {
                    queue.push_back(other);
                }
            }
        }
        visited
    };

    let upstream_of_inputs = |node: NodeId| -> bool {
        if !bound_enabled {
            return false;
        }
        if rails.contains(&node) || node == graph.in_node {
            return false;
        }
        if feedback_loop_nodes.contains(&node) {
            return false;
        }
        match (d_in.get(&node), input_pin_dist) {
            (Some(&d), Some(d_pin)) => d < d_pin,
            _ => false,
        }
    };

    let mut pendant_edges = Vec::new();
    let mut ground_shunt_edges = Vec::new();

    for &eidx in edge_indices {
        if claimed.contains(&eidx) || active_edges.contains(&eidx) || feedback_edges.contains(&eidx)
        {
            continue;
        }
        let comp = &graph.components[graph.edges[eidx].comp_idx];
        if !matches!(comp.kind.signal_terminals(), SignalTerminals::Passive) {
            continue;
        }
        // Mask 7: a forward-signal branch leaving a group output node toward
        // `out` (coupling cap -> output transformer -> load) is its own
        // downstream stage — never claim it into this NL group.
        if is_forward_signal_edge(eidx) {
            continue;
        }
        let e = &graph.edges[eidx];
        let a_rail = rails.contains(&e.node_a);
        let b_rail = rails.contains(&e.node_b);

        // Ground shunt: one terminal on rail, other in group_nodes.
        // But NOT if the non-rail terminal is an active output node AND
        // the rail is ground — those are post-amp shunts (e.g. C_tone in
        // RAT), not part of the amplifier. Supply-side load resistors
        // (VCC → plate/drain/collector) ARE essential and must be claimed.
        let non_rail_node = if a_rail {
            e.node_b
        } else if b_rail {
            e.node_a
        } else {
            e.node_a
        };
        let rail_node = if a_rail { e.node_a } else { e.node_b };
        let rail_is_ground =
            rail_node == graph.gnd_node || graph.ac_ground_nodes.contains(&rail_node);
        let at_output = post_stage_output_nodes.contains(&non_rail_node) && rail_is_ground;

        if !at_output
            && ((a_rail && group_nodes.contains(&e.node_b))
                || (b_rail && group_nodes.contains(&e.node_a)))
        {
            ground_shunt_edges.push(eidx);
        } else if (input_terminals.contains(&e.node_a)
            && !voltage_source_output_nodes.contains(&e.node_a))
            || (input_terminals.contains(&e.node_b)
                && !voltage_source_output_nodes.contains(&e.node_b))
        {
            // F10: skip single-hop claims whose far endpoint is a strictly
            // upstream INTERIOR node — that edge is a purely passive
            // upstream network's output coupling, and claiming it seeds the
            // BFS expansion that swallows the network. Barrier far
            // endpoints (in/out, other elements' pins) are exempt: BFS can
            // never expand from them, so claiming such coupling edges is
            // the ordinary, safe behavior (input caps from `in`, input
            // resistors from a previous stage's output).
            let far = if input_terminals.contains(&e.node_a) {
                e.node_b
            } else {
                e.node_a
            };
            if bfs_barriers.contains(&far) || !upstream_of_inputs(far) {
                pendant_edges.push(eidx);
            }
        }
    }

    // ── BFS expansion: claim multi-hop chains from group_nodes ──────────
    //
    // The single-hop pass above only claims edges directly touching
    // group_nodes or input_terminals. In BJT ladders (e.g., TB303),
    // emitter components form chains: Q.emitter → R_e → Cutoff → GND.
    // R_e has no rail endpoint and Cutoff's non-rail side isn't a group_node.
    // Without BFS expansion, these chains escape into passive groups and
    // create degenerate IIR stages that kill signal.
    //
    // BFS from group_nodes through unclaimed passive edges:
    // - Edge to rail (GND/ac_ground) → claim as ground_shunt
    // - Edge to interior (non-rail, non-barrier) node → claim as pendant,
    //   add endpoint to frontier, continue BFS
    // - Stop at barrier nodes (other active inputs, circuit terminals);
    //   the barrier set is built above, before the single-hop pass.

    let mut already_claimed: HashSet<usize> = HashSet::new();
    for &eidx in pendant_edges.iter().chain(ground_shunt_edges.iter()) {
        already_claimed.insert(eidx);
    }

    // Expand group_nodes with nodes from already-claimed edges
    let mut expanded_nodes = group_nodes.clone();
    for &eidx in pendant_edges.iter().chain(ground_shunt_edges.iter()) {
        let e = &graph.edges[eidx];
        if !rails.contains(&e.node_a) {
            expanded_nodes.insert(e.node_a);
        }
        if !rails.contains(&e.node_b) {
            expanded_nodes.insert(e.node_b);
        }
    }

    // BFS frontier: start from all expanded group nodes
    let mut frontier: VecDeque<NodeId> = expanded_nodes.iter().copied().collect();
    let mut visited_bfs: HashSet<NodeId> = expanded_nodes.clone();

    while let Some(node) = frontier.pop_front() {
        // Barrier nodes may be reached by a directly claimed pendant edge
        // (for example an op-amp input resistor connected to the previous
        // buffer's output). The edge itself belongs to this group, but other
        // branches leaving that upstream output are independent feedforward or
        // tone paths and must not be swallowed by this active group.
        if bfs_barriers.contains(&node) {
            continue;
        }
        // F10: never expand FROM a node strictly upstream of the device's
        // input pins. Such a node (e.g. the far end of the input coupling
        // cap) may legitimately enter the frontier through a single-hop
        // pendant claim — the coupling edge itself belongs to this group —
        // but every other branch leaving it is the UPSTREAM network (a
        // passive EQ feeding this makeup stage) and must not be swallowed.
        if upstream_of_inputs(node) {
            continue;
        }
        for &eidx in edge_indices {
            if claimed.contains(&eidx)
                || active_edges.contains(&eidx)
                || feedback_edges.contains(&eidx)
                || already_claimed.contains(&eidx)
            {
                continue;
            }
            let comp = &graph.components[graph.edges[eidx].comp_idx];
            if !matches!(comp.kind.signal_terminals(), SignalTerminals::Passive) {
                continue;
            }
            // Mask 7: never claim or expand through a forward-signal branch
            // leaving a group output node toward `out` (coupling cap ->
            // output transformer -> load). It is its own downstream stage.
            if is_forward_signal_edge(eidx) {
                continue;
            }
            let e = &graph.edges[eidx];

            // Edge must touch the current BFS node
            let (touching, other) = if e.node_a == node {
                (e.node_a, e.node_b)
            } else if e.node_b == node {
                (e.node_b, e.node_a)
            } else {
                continue;
            };
            let _ = touching;

            // Voltage-source outputs are split boundaries. Feedback paths
            // were already claimed by the explicit out->in DFS, so any
            // remaining passive edge leaving this node is downstream load or
            // tone shaping and must not be swallowed by the active group.
            if voltage_source_output_nodes.contains(&node) {
                continue;
            }

            // Don't cross into barriers
            if bfs_barriers.contains(&other) {
                continue;
            }

            if rails.contains(&other) {
                // Ground/rail termination → ground shunt
                let rail_is_gnd = other == graph.gnd_node || graph.ac_ground_nodes.contains(&other);
                // Respect at_output exclusion for post-stage ground shunts.
                // Two-port nonlinear devices are not impedance boundaries:
                // their output-node caps can be part of a ladder rung.
                if rail_is_gnd && post_stage_output_nodes.contains(&node) {
                    continue;
                }
                ground_shunt_edges.push(eidx);
                already_claimed.insert(eidx);
            } else if !visited_bfs.contains(&other) {
                // F10: never expand to a non-rail node strictly closer to
                // `in_node` than the device's input pins — that direction is
                // upstream toward the circuit input, and walking it swallows
                // the upstream network (passive EQ → makeup stage flattening).
                if upstream_of_inputs(other) {
                    continue;
                }
                // Interior node → pendant, continue BFS
                pendant_edges.push(eidx);
                already_claimed.insert(eidx);
                visited_bfs.insert(other);
                frontier.push_back(other);
            } else {
                // Edge connects two already-known group nodes (e.g.,
                // R_fb_limit connecting resonance path back to Q1.base).
                // Claim it — it's internal to this group.
                pendant_edges.push(eidx);
                already_claimed.insert(eidx);
            }
        }
    }

    (pendant_edges, ground_shunt_edges)
}

/// Partition circuit edges into signal flow groups.
///
/// Uses directed signal flow analysis (SCC detection) to correctly
/// separate cascaded stages while grouping mutually-dependent elements.
pub(in crate::compiler) fn find_flow_groups(
    edge_indices: &[usize],
    graph: &CircuitGraph,
) -> Vec<FlowGroup> {
    let rails = rail_nodes(graph);
    let active_elements = find_active_elements(edge_indices, graph);

    if active_elements.is_empty() {
        return vec![FlowGroup {
            active_edges: Vec::new(),
            feedback_edges: Vec::new(),
            pendant_edges: edge_indices.to_vec(),
            ground_shunt_edges: Vec::new(),
        }];
    }

    // Phase 2a: consult the broker ONCE for the delayed-coupling cut set. The
    // rule logic lives entirely in `boundary_rules::delayed_cut_edges`; formation
    // only excludes the returned tap-mouth edges from grouping. Empty for every
    // circuit without a feedback-detector control electrode (the common case).
    let cut_edges = super::boundary_rules::delayed_cut_edges(graph);

    let active_edge_set: HashSet<usize> = active_elements.iter().map(|e| e.edge_idx).collect();
    let adj = build_passive_adjacency(edge_indices, graph, &rails, &active_edge_set, &cut_edges);

    // Build directed flow graph and find SCCs
    let flow_adj = build_flow_graph(&active_elements, &adj, &rails, graph);
    let sccs = tarjan_scc(&flow_adj);

    // Merge SCCs that contain edges from the same multi-edge component (e.g. BJT).
    // A BJT has 2 active elements (B-E, C-E) from the same comp_idx. Without
    // feedback they land in separate SCCs, but must be co-solved as one device.
    let sccs = merge_same_component_sccs(sccs, &active_elements, graph);
    let sccs = merge_shared_active_node_sccs(sccs, &active_elements, graph, &rails);

    // Build classified FlowGroup for each SCC
    let mut claimed: HashSet<usize> = HashSet::new();
    let mut groups: Vec<FlowGroup> = Vec::new();

    for scc in &sccs {
        let active_edges: Vec<usize> = scc.iter().map(|&i| active_elements[i].edge_idx).collect();

        // Determine if this SCC has self-feedback or mutual feedback
        // A single element has self-feedback if its output reaches its input
        // Multiple elements in SCC always have mutual feedback
        let has_mutual = scc.len() > 1;

        // Check self-feedback for single-element SCCs
        let has_self_feedback = if scc.len() == 1 {
            let elem = &active_elements[scc[0]];
            if rails.contains(&elem.input_node) || rails.contains(&elem.output_node) {
                false
            } else {
                // For self-feedback, block other elements' outputs but not our own
                let blocked_outputs: HashSet<NodeId> = active_elements
                    .iter()
                    .enumerate()
                    .filter(|&(j, _)| j != scc[0])
                    .filter(|(_, e)| active_output_blocks_passive_reachability(e, graph, &rails))
                    .filter_map(|(_, e)| {
                        if rails.contains(&e.output_node) {
                            None
                        } else {
                            Some(e.output_node)
                        }
                    })
                    .collect();
                let reachable = bfs_reachable_nodes(elem.output_node, &adj, &blocked_outputs);
                reachable.contains(&elem.input_node)
            }
        } else {
            false
        };

        let has_feedback = has_mutual || has_self_feedback;

        if !has_feedback {
            // Standalone device stages sometimes need adjacent local passive
            // networks to remain physically meaningful: transistor/tube
            // emitter/source/cathode chains are part of the device operating
            // point. Standalone diode clippers are different: downstream tone,
            // coupling, and feedforward networks should remain split so they
            // can compile as passive WDF/IIR stages with independent controls.
            let mut should_claim_local_passives = scc.iter().any(|&i| {
                let comp = &graph.components[graph.edges[active_elements[i].edge_idx].comp_idx];
                comp.kind.is_bjt()
                    || comp.kind.is_jfet()
                    || comp.kind.is_mosfet()
                    || comp.kind.is_tube()
            });

            // A *series* diode (both terminals are interior signal nodes, neither
            // tied to a rail) sits IN the through-path — e.g. a half-wave
            // rectifier `in -> R1 -> D1 -> RL -> gnd`. Like a transistor's signal
            // path, it must keep its adjacent series resistors in the same MNA
            // block: an isolated diode node sees only the GMIN leak (~1e9 Ω) for a
            // Thevenin source, so it can never develop a junction voltage and
            // fails to rectify (the stage degenerates to a unity pass-through).
            //
            // Shunt clippers (one diode terminal on a rail, the classic
            // diode-to-ground clipper) are deliberately left split so their
            // downstream tone/coupling networks compile as independent passive
            // WDF/IIR stages — that path is already correct and must not change.
            let has_series_diode = scc.iter().any(|&i| {
                let edge = &graph.edges[active_elements[i].edge_idx];
                let comp = &graph.components[edge.comp_idx];
                comp.kind.is_diode_family()
                    && !rails.contains(&edge.node_a)
                    && !rails.contains(&edge.node_b)
            });
            should_claim_local_passives |= has_series_diode;

            // F10: a tube fed from an upstream INTERIOR node (a passive
            // network between `in` and the grid, e.g. an EQ ahead of a
            // makeup stage) must NOT claim local passives here. Claiming
            // would make the group multi-edge and route it to the NlWdf
            // builder, whose voltage source sits at the global in/out
            // terminals — absent from such a group, the stage goes dead.
            // Leaving the group as the bare NL edge routes it to the
            // triode-context MNA path (spqr_build), which collects the
            // rail-terminated bias edges and injects the cascade input at
            // the grid. Tubes coupled directly from `in` (far endpoint ==
            // in_node) are unaffected.
            if should_claim_local_passives {
                let d_in = bfs_distances_from_in_node(graph);
                let tube_fed_from_interior = scc.iter().any(|&i| {
                    let elem = &active_elements[i];
                    let comp = &graph.components[graph.edges[elem.edge_idx].comp_idx];
                    if !comp.kind.is_tube() {
                        return false;
                    }
                    let Some(&d_pin) = d_in.get(&elem.input_node) else {
                        return false;
                    };
                    edge_indices.iter().any(|&eidx| {
                        if claimed.contains(&eidx) {
                            return false;
                        }
                        let e = &graph.edges[eidx];
                        let comp = &graph.components[e.comp_idx];
                        if !matches!(comp.kind.signal_terminals(), SignalTerminals::Passive) {
                            return false;
                        }
                        let far = if e.node_a == elem.input_node {
                            e.node_b
                        } else if e.node_b == elem.input_node {
                            e.node_a
                        } else {
                            return false;
                        };
                        if rails.contains(&far) || far == graph.in_node {
                            return false;
                        }
                        matches!(d_in.get(&far), Some(&d) if d < d_pin)
                    })
                });
                if tube_fed_from_interior {
                    should_claim_local_passives = false;
                }
            }

            let (pendant_edges, ground_shunt_edges) = if should_claim_local_passives {
                claim_passive_edges(
                    scc,
                    &active_elements,
                    &active_edges,
                    &[], // no feedback edges
                    edge_indices,
                    graph,
                    &rails,
                    &claimed,
                )
            } else {
                (Vec::new(), Vec::new())
            };
            for &eidx in active_edges
                .iter()
                .chain(&pendant_edges)
                .chain(&ground_shunt_edges)
            {
                claimed.insert(eidx);
            }
            groups.push(FlowGroup {
                active_edges,
                feedback_edges: Vec::new(),
                pendant_edges,
                ground_shunt_edges,
            });
            continue;
        }

        // DFS: find passive edges on feedback paths (out->in for each element in group).
        // Block circuit terminal nodes (in/out) from expansion — these are signal
        // entry/exit points, not part of feedback loops. Without this, the DFS
        // traverses through the input node and sweeps input coupling resistors
        // (R_in) into the feedback set.
        let mut feedback_set: HashSet<usize> = HashSet::new();
        let mut terminal_block: HashSet<NodeId> = HashSet::new();
        if !rails.contains(&graph.in_node) {
            terminal_block.insert(graph.in_node);
        }
        if !rails.contains(&graph.out_node) {
            terminal_block.insert(graph.out_node);
        }

        for &ei in scc {
            let elem = &active_elements[ei];
            if rails.contains(&elem.input_node) || rails.contains(&elem.output_node) {
                continue;
            }
            let mut feedback_block = terminal_block.clone();
            let own_comp_idx = graph.edges[elem.edge_idx].comp_idx;
            for &node in &graph.output_pin_nodes {
                if node != elem.output_node && !rails.contains(&node) {
                    feedback_block.insert(node);
                }
            }
            for pins in &graph.nullor_pins {
                if pins.comp_idx == own_comp_idx {
                    continue;
                }
                for node in [pins.pos_node, pins.neg_node] {
                    if node != elem.input_node && !rails.contains(&node) {
                        feedback_block.insert(node);
                    }
                }
            }
            for (j, other) in active_elements.iter().enumerate() {
                let other_comp = &graph.components[graph.edges[other.edge_idx].comp_idx];
                if j != ei {
                    if active_output_blocks_passive_reachability(other, graph, &rails) {
                        feedback_block.extend(
                            other
                                .output_nodes
                                .iter()
                                .copied()
                                .filter(|node| !rails.contains(node)),
                        );
                    }
                    if other_comp.kind.feedback_input_is_barrier()
                        && !rails.contains(&other.input_node)
                    {
                        feedback_block.insert(other.input_node);
                    }
                }
            }
            let comp = &graph.components[graph.edges[elem.edge_idx].comp_idx];
            if let SignalTerminals::Amplifier {
                control: Some(ctrl),
                ..
            } = comp.kind.signal_terminals()
            {
                if let Some(node) = resolve_pin(&comp.id, ctrl, graph) {
                    if node != elem.input_node && !rails.contains(&node) {
                        feedback_block.insert(node);
                    }
                }
            }
            feedback_block.remove(&elem.output_node);
            feedback_block.remove(&elem.input_node);
            let mut path = Vec::new();
            let mut visited = HashSet::new();
            visited.insert(elem.output_node);
            dfs_find_paths_blocked(
                elem.output_node,
                elem.input_node,
                &adj,
                &feedback_block,
                &mut visited,
                &mut path,
                &mut feedback_set,
            );
        }

        // For mutual feedback groups, also find paths between elements
        if has_mutual {
            for &ei in scc {
                for &ej in scc {
                    if ei == ej {
                        continue;
                    }
                    let elem_i = &active_elements[ei];
                    let elem_j = &active_elements[ej];
                    if rails.contains(&elem_i.output_node) || rails.contains(&elem_j.input_node) {
                        continue;
                    }
                    let mut feedback_block = terminal_block.clone();
                    for &node in &graph.output_pin_nodes {
                        if node != elem_i.output_node && !rails.contains(&node) {
                            feedback_block.insert(node);
                        }
                    }
                    for pins in &graph.nullor_pins {
                        for node in [pins.pos_node, pins.neg_node] {
                            if node != elem_i.input_node
                                && node != elem_j.input_node
                                && !rails.contains(&node)
                            {
                                feedback_block.insert(node);
                            }
                        }
                    }
                    for (k, other) in active_elements.iter().enumerate() {
                        let other_comp = &graph.components[graph.edges[other.edge_idx].comp_idx];
                        if k != ei && k != ej {
                            if active_output_blocks_passive_reachability(other, graph, &rails) {
                                feedback_block.extend(
                                    other
                                        .output_nodes
                                        .iter()
                                        .copied()
                                        .filter(|node| !rails.contains(node)),
                                );
                            }
                            if other_comp.kind.feedback_input_is_barrier()
                                && !rails.contains(&other.input_node)
                            {
                                feedback_block.insert(other.input_node);
                            }
                        }
                    }
                    // Don't block the DFS endpoints (mirrors the self-feedback
                    // path above, which removes its own output/input from the
                    // block). A sibling port of elem_i shares elem_i's output
                    // node, so without this the DFS can't leave the device's own
                    // collector — the resistor-coupled feedback (Fuzz Face:
                    // Q1.collector -> R4 -> Q2.base, Q2.collector -> R2 ->
                    // Q1.base) is never collected, leaving feedback_edges empty
                    // and the loop mis-routed as a non-feedback (split) group.
                    feedback_block.remove(&elem_i.output_node);
                    feedback_block.remove(&elem_j.input_node);
                    let mut path = Vec::new();
                    let mut visited = HashSet::new();
                    visited.insert(elem_i.output_node);
                    dfs_find_paths_blocked(
                        elem_i.output_node,
                        elem_j.input_node,
                        &adj,
                        &feedback_block,
                        &mut visited,
                        &mut path,
                        &mut feedback_set,
                    );
                }
            }
        }

        // Remove active element edges from feedback_set
        for &ae in &active_edges {
            feedback_set.remove(&ae);
        }

        let scc_input_nodes: HashSet<NodeId> = scc
            .iter()
            .map(|&ei| active_elements[ei].input_node)
            .filter(|node| !rails.contains(node))
            .collect();
        let scc_voltage_source_outputs: HashSet<NodeId> = scc
            .iter()
            .filter_map(|&ei| {
                let elem = &active_elements[ei];
                let comp = &graph.components[graph.edges[elem.edge_idx].comp_idx];
                if comp.kind.output_impedance() == OutputImpedance::VoltageSource
                    && !rails.contains(&elem.output_node)
                {
                    Some(elem.output_node)
                } else {
                    None
                }
            })
            .collect();

        if !scc_voltage_source_outputs.is_empty() {
            let feedback_return_nodes: HashSet<NodeId> = scc_input_nodes
                .difference(&scc_voltage_source_outputs)
                .copied()
                .collect();
            let mut blocked = terminal_block.clone();
            blocked.extend(scc_voltage_source_outputs.iter().copied());
            feedback_set.retain(|&eidx| {
                let e = &graph.edges[eidx];
                let touches_voltage_source_output = scc_voltage_source_outputs.contains(&e.node_a)
                    || scc_voltage_source_outputs.contains(&e.node_b);
                if !touches_voltage_source_output {
                    return true;
                }

                let start = if scc_voltage_source_outputs.contains(&e.node_a) {
                    e.node_b
                } else {
                    e.node_a
                };
                if feedback_return_nodes.contains(&start) {
                    return true;
                }
                if blocked.contains(&start) {
                    return false;
                }

                let mut visited = HashSet::new();
                let mut queue = VecDeque::new();
                visited.insert(start);
                queue.push_back(start);

                while let Some(node) = queue.pop_front() {
                    if let Some(neighbors) = adj.get(&node) {
                        for &(_next_eidx, next) in neighbors {
                            if feedback_return_nodes.contains(&next) {
                                return true;
                            }
                            if blocked.contains(&next) || !visited.insert(next) {
                                continue;
                            }
                            queue.push_back(next);
                        }
                    }
                }

                false
            });
        }

        let mut feedback_edges: Vec<usize> = feedback_set.into_iter().collect();
        feedback_edges.sort_unstable(); // deterministic ordering

        // Claim passive edges (ground shunts and pendants) adjacent to
        // the active + feedback edges.
        let (pendant_edges, ground_shunt_edges) = claim_passive_edges(
            scc,
            &active_elements,
            &active_edges,
            &feedback_edges,
            edge_indices,
            graph,
            &rails,
            &claimed,
        );

        // Mark all claimed
        for &eidx in active_edges
            .iter()
            .chain(&feedback_edges)
            .chain(&pendant_edges)
            .chain(&ground_shunt_edges)
        {
            claimed.insert(eidx);
        }

        groups.push(FlowGroup {
            active_edges,
            feedback_edges,
            pendant_edges,
            ground_shunt_edges,
        });
    }

    // ── Unclaimed passive edges: group by connectivity ──────────────────
    //
    // Barrier nodes prevent merging across stage boundaries:
    // - Rails (GND, VCC, supply, AC ground) — always barriers
    // - Voltage-source outputs (op-amp out pins) — safe split points
    //   per Harold Black's theorem: zero output impedance means
    //   downstream load doesn't affect upstream behavior.
    //
    // Edges sharing a non-barrier node are grouped → one WDF stage.
    // This prevents standalone caps from acting as high-pass filters.

    let mut barrier_nodes = rails.clone();

    // Add active AMPLIFIER input nodes as barriers.
    // Input nodes (neg, base, gate) separate upstream from downstream passive
    // groups — e.g., input coupling should not merge with feedback passives.
    //
    // Only Amplifier types (op-amps, BJTs, tubes) create barriers. TwoPort
    // elements (diodes, zeners) are nonlinear but don't create impedance
    // boundaries — their input nodes should NOT be barriers, or else
    // post-amp passive networks (tone caps, filter pots) can't merge.
    //
    // Output nodes are NOT barriers. The serial processing chain needs
    // post-amp passive components to group together for correct WDF filtering.
    for &eidx in edge_indices {
        let comp = &graph.components[graph.edges[eidx].comp_idx];
        match comp.kind.signal_terminals() {
            SignalTerminals::Amplifier { input, .. } => {
                if let Some(node) = resolve_pin(&comp.id, input, graph) {
                    barrier_nodes.insert(node);
                }
            }
            SignalTerminals::TwoPort { .. } | SignalTerminals::Passive => {}
        }
    }

    // Separate unclaimed edges into:
    // - signal_edges: at least one non-barrier endpoint (carry signal, groupable)
    // - isolated_edges: both endpoints are barriers (bypass caps, pull-downs)
    let mut signal_edges: Vec<usize> = Vec::new();
    let mut isolated_edges: Vec<usize> = Vec::new();
    for &eidx in edge_indices {
        if claimed.contains(&eidx) {
            continue;
        }
        // Phase 2a: a broker-cut tap-mouth edge is the delayed-coupling
        // boundary — it must NOT re-fuse the detector front-end to the forward
        // network through the unclaimed-passive connectivity grouping. Exclude
        // it (its endpoints become stage ports via the SPQR terminal set).
        if cut_edges.cuts.contains_key(&eidx) {
            claimed.insert(eidx);
            continue;
        }
        let e = &graph.edges[eidx];
        let a_barrier = barrier_nodes.contains(&e.node_a);
        let b_barrier = barrier_nodes.contains(&e.node_b);
        if a_barrier && b_barrier {
            // Both ends are barriers — this edge is isolated (bypass cap, etc.)
            isolated_edges.push(eidx);
        } else {
            signal_edges.push(eidx);
        }
    }

    // Group signal-carrying edges by connectivity (split at barriers)
    let passive_groups = group_by_connectivity(&signal_edges, graph, &barrier_nodes);
    for edges in passive_groups {
        groups.push(FlowGroup {
            active_edges: Vec::new(),
            feedback_edges: Vec::new(),
            pendant_edges: edges,
            ground_shunt_edges: Vec::new(),
        });
    }

    // Isolated edges → individual standalone stages
    for eidx in isolated_edges {
        groups.push(FlowGroup {
            active_edges: Vec::new(),
            feedback_edges: Vec::new(),
            pendant_edges: vec![eidx],
            ground_shunt_edges: Vec::new(),
        });
    }

    groups
}

/// Group edges by node connectivity using union-find.
///
/// Edges sharing a non-barrier node are merged into one group.
/// Barrier nodes (rails + voltage-source outputs) act as boundaries —
/// edges on opposite sides of a barrier stay in separate groups.
/// This preserves impedance relationships within each group.
fn group_by_connectivity(
    edges: &[usize],
    graph: &CircuitGraph,
    barrier_nodes: &HashSet<NodeId>,
) -> Vec<Vec<usize>> {
    if edges.is_empty() {
        return Vec::new();
    }

    // Build node → edge index mapping (skip barrier nodes)
    let mut node_to_edges: BTreeMap<NodeId, Vec<usize>> = BTreeMap::new();
    for &eidx in edges {
        let e = &graph.edges[eidx];
        if !barrier_nodes.contains(&e.node_a) {
            node_to_edges.entry(e.node_a).or_default().push(eidx);
        }
        if !barrier_nodes.contains(&e.node_b) {
            node_to_edges.entry(e.node_b).or_default().push(eidx);
        }
    }

    // Union-find: edges sharing a non-barrier node are in the same group
    let edge_pos: BTreeMap<usize, usize> = edges.iter().enumerate().map(|(i, &e)| (e, i)).collect();
    let mut parent: Vec<usize> = (0..edges.len()).collect();

    fn find(parent: &mut [usize], mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }

    for edge_list in node_to_edges.values() {
        if edge_list.len() > 1 {
            let first = edge_pos[&edge_list[0]];
            for &eidx in &edge_list[1..] {
                let pos = edge_pos[&eidx];
                let ra = find(&mut parent, first);
                let rb = find(&mut parent, pos);
                if ra != rb {
                    parent[rb] = ra;
                }
            }
        }
    }

    // Collect groups (BTreeMap for deterministic iteration order)
    let mut groups_map: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (i, &eidx) in edges.iter().enumerate() {
        let root = find(&mut parent, i);
        groups_map.entry(root).or_default().push(eidx);
    }

    groups_map.into_values().collect()
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

    fn group_active_names(group: &FlowGroup, graph: &CircuitGraph) -> Vec<String> {
        let mut names: Vec<String> = group
            .active_edges
            .iter()
            .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.clone())
            .collect();
        names.sort();
        names.dedup();
        names
    }

    fn find_group_containing(
        groups: &[FlowGroup],
        graph: &CircuitGraph,
        comp_id: &str,
    ) -> Option<usize> {
        groups.iter().position(|g| {
            g.all_edges()
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == comp_id)
        })
    }

    fn group_component_names(group: &FlowGroup, graph: &CircuitGraph) -> Vec<String> {
        let mut names: Vec<String> = group
            .all_edges()
            .iter()
            .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.clone())
            .collect();
        names.sort();
        names.dedup();
        names
    }

    // ── Test 1: Two op-amps in cascade, no inter-stage feedback ──────────

    #[test]
    fn flow_cascade_two_opamps_separate() {
        let (graph, edges) = make_graph_all_edges(
            r#"
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
            }"#,
        );

        let groups = find_flow_groups(&edges, &graph);
        let u1_group = find_group_containing(&groups, &graph, "U1");
        let u2_group = find_group_containing(&groups, &graph, "U2");
        assert!(u1_group.is_some(), "Should have U1 group");
        assert!(u2_group.is_some(), "Should have U2 group");
        assert_ne!(
            u1_group, u2_group,
            "Cascaded op-amps without inter-stage feedback should be in SEPARATE groups"
        );
    }

    #[test]
    fn flow_simple_inverting_opamp_feedback_reaches_global_output() {
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    R1: resistor(10k)
                    U1: opamp(tl072)
                    Rf: resistor(100k)
                }
                nets {
                    in -> R1.a
                    R1.b -> U1.neg
                    U1.neg -> Rf.a
                    Rf.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> out
                }
                controls {}
            }"#,
        );

        let groups = find_flow_groups(&edges, &graph);
        let u1_group = find_group_containing(&groups, &graph, "U1").expect("U1 group");
        let group = &groups[u1_group];

        assert!(
            group.has_feedback(),
            "op-amp feedback ending at global out must remain a feedback group"
        );
        assert!(
            group
                .feedback_edges
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "Rf"),
            "Rf should be classified as feedback"
        );
    }

    #[test]
    fn flow_claims_supply_side_bias_pot_chain_for_bjt_group() {
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 12V
                components {
                    Q_tail: pnp(2n3906)
                    Q_mirror: pnp(2n3906)
                    R_tail_floor: resistor(1k)
                    VCA_Gain: pot(100k, b)
                    R_emit: resistor(1k)
                }
                nets {
                    vcc -> VCA_Gain.a
                    VCA_Gain.b -> R_tail_floor.a
                    R_tail_floor.b -> Q_mirror.emitter
                    Q_mirror.base -> Q_mirror.collector
                    Q_mirror.collector -> Q_tail.base
                    vcc -> Q_tail.emitter
                    Q_tail.collector -> R_emit.a
                    R_emit.b -> gnd
                }
                controls {
                    VCA_Gain.position -> "VCA_Gain" [0.0, 1.0] = 1.0
                }
            }"#,
        );

        let groups = find_flow_groups(&edges, &graph);
        let mirror_group =
            find_group_containing(&groups, &graph, "Q_mirror").expect("Q_mirror group");
        let names = group_component_names(&groups[mirror_group], &graph);

        assert!(
            names.iter().any(|name| name == "VCA_Gain"),
            "supply-side bias pot must be claimed with current-mirror BJT group; group={names:?}"
        );
    }

    #[test]
    fn flow_keeps_tb303_vca_gain_pot_in_runtime_group() {
        const TB303_VCA: &str = r#"
            pedal "TB303 VCA" { supply 12V
                components {
                    Q_diff_p: npn(2n3904)
                    Q_diff_n: npn(2n3904)
                    Q_tail: pnp(2n3906)
                    Q_mirror: pnp(2n3906)
                    R_tail_floor: resistor(1k)
                    VCA_Gain: pot(100k, b)
                    Q_wilson_a: pnp(2n3906)
                    Q_wilson_b: pnp(2n3906)
                    R_load: resistor(47k)
                    R_in: resistor(10k)
                    C_in: cap(100n)
                    R_bias_top: resistor(100k)
                    R_bias_bot: resistor(100k)
                    C_out: cap(100n)
                    R_out: resistor(10k)
                }
                nets {
                    vcc -> R_bias_top.a
                    R_bias_top.b -> R_bias_bot.a
                    R_bias_bot.b -> gnd
                    R_bias_top.b -> Q_diff_p.base
                    in -> C_in.a
                    C_in.b -> R_in.a
                    R_in.b -> Q_diff_n.base
                    R_bias_top.b -> Q_diff_n.base
                    vcc -> VCA_Gain.a
                    VCA_Gain.b -> R_tail_floor.a
                    R_tail_floor.b -> Q_mirror.emitter
                    Q_mirror.base -> Q_mirror.collector
                    Q_mirror.collector -> Q_tail.base
                    vcc -> Q_tail.emitter
                    Q_tail.collector -> Q_diff_p.emitter, Q_diff_n.emitter
                    vcc -> Q_wilson_a.emitter
                    vcc -> Q_wilson_b.emitter
                    Q_wilson_a.base -> Q_wilson_b.collector, Q_diff_p.collector
                    Q_wilson_b.base -> Q_wilson_a.collector
                    Q_wilson_a.collector -> Q_diff_n.collector
                    Q_wilson_a.collector -> R_load.a
                    R_load.b -> gnd
                    Q_wilson_a.collector -> C_out.a
                    C_out.b -> R_out.a
                    R_out.b -> out
                }
                controls {
                    VCA_Gain.position -> "VCA_Gain" [0.0, 1.0] = 1.0
                }
            }"#;
        let (graph, edges) = make_graph_all_edges(TB303_VCA);

        let groups = find_flow_groups(&edges, &graph);
        let gain_group =
            find_group_containing(&groups, &graph, "VCA_Gain").expect("VCA_Gain group");
        let names = group_component_names(&groups[gain_group], &graph);

        assert!(
            names.iter().any(|name| name == "Q_mirror")
                || names.iter().any(|name| name == "Q_tail"),
            "VCA_Gain must stay with a nonlinear runtime group; group={names:?}"
        );

        let pedal = crate::dsl::parse_pedal_file(TB303_VCA).expect("parse failed");
        let compiled = crate::compiler::compile_pedal(&pedal, 48_000.0).expect("compile failed");
        assert!(
            compiled
                .controls
                .iter()
                .any(|control| control.label == "VCA_Gain"),
            "VCA_Gain must bind to a runtime stage"
        );
    }

    // ── Test 2: Fuzz Face — two BJTs with feedback ───────────────────────

    #[test]
    fn flow_fuzz_face_coupled() {
        let (graph, edges) = make_graph_all_edges(
            r#"
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
            }"#,
        );

        let groups = find_flow_groups(&edges, &graph);

        let q1_group = find_group_containing(&groups, &graph, "Q1");
        let q2_group = find_group_containing(&groups, &graph, "Q2");
        assert!(q1_group.is_some(), "Should have Q1 group");
        assert!(q2_group.is_some(), "Should have Q2 group");
        assert_eq!(
            q1_group, q2_group,
            "Fuzz Face Q1 and Q2 should be in SAME group (Q2.emitter -> Q1.base feedback)"
        );
    }

    // ── Test 3: RAT — op-amp + diodes to GND separate ───────────────────

    #[test]
    fn flow_rat_opamp_plus_diodes_to_gnd() {
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    R_in: resistor(1k)
                    U1: opamp(lm308)
                    R_fb: resistor(47k)
                    D1: diode(silicon)
                    D2: diode(silicon)
                    R_out: resistor(10k)
                }
                nets {
                    in -> R_in.a
                    R_in.b -> U1.neg
                    U1.neg -> R_fb.a
                    R_fb.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> D1.a
                    D1.b -> gnd
                    U1.out -> D2.b
                    D2.a -> gnd
                    U1.out -> R_out.a
                    R_out.b -> out
                }
                controls {}
            }"#,
        );

        let groups = find_flow_groups(&edges, &graph);

        let u1_group = find_group_containing(&groups, &graph, "U1");
        let d1_group = find_group_containing(&groups, &graph, "D1");
        let d2_group = find_group_containing(&groups, &graph, "D2");

        assert!(u1_group.is_some());
        assert!(d1_group.is_some());
        assert!(d2_group.is_some());

        // Op-amp is separate from diodes (diodes go to GND = rail)
        assert_ne!(
            u1_group, d1_group,
            "Op-amp and GND-shunt diode D1 should be separate groups"
        );
        assert_ne!(
            u1_group, d2_group,
            "Op-amp and GND-shunt diode D2 should be separate groups"
        );
        // Each diode is also separate from each other (GND doesn't couple)
        assert_ne!(
            d1_group, d2_group,
            "D1 and D2 to GND should be separate groups"
        );
    }

    // ── Test 4: Screamer — diode pair in feedback loop ───────────────────

    #[test]
    fn flow_screamer_diode_in_feedback() {
        // Op-amp with diodes in the feedback path (neg -> out through diode pair)
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    R_in: resistor(4.7k)
                    U1: opamp(tl072)
                    R_fb: resistor(51k)
                    D1: diode(silicon)
                    D2: diode(silicon)
                    R_out: resistor(10k)
                }
                nets {
                    in -> R_in.a
                    R_in.b -> U1.neg
                    U1.neg -> R_fb.a
                    R_fb.b -> U1.out
                    U1.neg -> D1.a
                    D1.b -> U1.out
                    U1.neg -> D2.b
                    D2.a -> U1.out
                    U1.pos -> gnd
                    U1.out -> R_out.a
                    R_out.b -> out
                }
                controls {}
            }"#,
        );

        let groups = find_flow_groups(&edges, &graph);

        let u1_group = find_group_containing(&groups, &graph, "U1");
        let d1_group = find_group_containing(&groups, &graph, "D1");
        let d2_group = find_group_containing(&groups, &graph, "D2");

        assert!(u1_group.is_some());
        assert!(d1_group.is_some());
        assert!(d2_group.is_some());

        // All in same group — diodes are in the feedback loop
        assert_eq!(
            u1_group, d1_group,
            "Op-amp and feedback diode D1 should be in SAME group"
        );
        assert_eq!(
            u1_group, d2_group,
            "Op-amp and feedback diode D2 should be in SAME group"
        );
    }

    #[test]
    fn flow_voltage_source_feedback_does_not_claim_downstream_tone_network() {
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    R_in: resistor(4.7k)
                    U1: opamp(tl072)
                    R_fb: resistor(51k)
                    Drive: pot(500k, a)
                    D1: diode(silicon)
                    D2: diode(silicon)
                    R_tone_in: resistor(1k)
                    Tone: pot(20k, b)
                    C_tone: cap(220n)
                    R_level_in: resistor(220)
                    C_level: cap(220n)
                    Level: pot(100k, a)
                    R_out: resistor(10k)
                }
                nets {
                    in -> R_in.a
                    R_in.b -> U1.neg
                    U1.neg -> R_fb.a
                    R_fb.b -> Drive.a
                    Drive.b -> U1.out
                    U1.neg -> D1.a
                    D1.b -> U1.out
                    U1.neg -> D2.b
                    D2.a -> U1.out
                    U1.pos -> gnd
                    U1.out -> R_tone_in.a
                    R_tone_in.b -> Tone.a
                    Tone.b -> C_tone.a
                    C_tone.b -> gnd
                    Tone.w -> R_level_in.a
                    R_level_in.b -> C_level.a
                    C_level.b -> gnd
                    Tone.w -> Level.a
                    Level.b -> gnd
                    Level.w -> R_out.a
                    R_out.b -> out
                }
                controls {}
            }"#,
        );

        let groups = find_flow_groups(&edges, &graph);

        let u1_group_idx =
            find_group_containing(&groups, &graph, "U1").expect("Should have U1 feedback group");
        let u1_group = &groups[u1_group_idx];

        for comp_id in ["R_fb", "Drive", "D1", "D2"] {
            assert!(
                u1_group
                    .all_edges()
                    .iter()
                    .any(|&eidx| { graph.components[graph.edges[eidx].comp_idx].id == comp_id }),
                "{comp_id} should stay in the voltage-source feedback group"
            );
        }

        for comp_id in ["R_tone_in", "Tone", "C_tone", "Level", "R_out"] {
            assert!(
                !u1_group
                    .all_edges()
                    .iter()
                    .any(|&eidx| { graph.components[graph.edges[eidx].comp_idx].id == comp_id }),
                "{comp_id} is downstream of the op-amp voltage-source output and should be split"
            );
        }
    }

    // ── Test 5: 808 bridged-T — single op-amp with T-network ────────────

    #[test]
    fn flow_808_bridged_t() {
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    U1: opamp(tl072)
                    R1: resistor(150k)
                    R2: resistor(150k)
                    C1: cap(8.2n)
                    C2: cap(8.2n)
                    R_fb: resistor(470k)
                    R_in: resistor(100k)
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
                    in -> R_in.a
                    R_in.b -> R1.b
                    U1.out -> out
                }
                controls {}
            }"#,
        );

        let groups = find_flow_groups(&edges, &graph);

        // Find the group containing U1
        let u1_group_idx = find_group_containing(&groups, &graph, "U1").expect("Should have U1");
        let u1_group = &groups[u1_group_idx];

        // Should have feedback (R_fb, R1, R2 form feedback paths)
        assert!(
            u1_group.has_feedback(),
            "808 bridged-T should have feedback"
        );

        // R1 and R2 should be in the feedback edges
        let has_r1 = u1_group
            .feedback_edges
            .iter()
            .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "R1");
        let has_r2 = u1_group
            .feedback_edges
            .iter()
            .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "R2");
        assert!(has_r1, "R1 should be in feedback edges (bridged-T path)");
        assert!(has_r2, "R2 should be in feedback edges (bridged-T path)");
    }

    #[test]
    fn flow_808_bridged_t_bjt_shunt_does_not_merge_with_opamp() {
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    U1: opamp(tl072)
                    Rb1: resistor(100k)
                    Rb2: resistor(100k)
                    R1: resistor(150k)
                    R2: resistor(150k)
                    C1: cap(8.2n)
                    C2: cap(8.2n)
                    R_fb: resistor(470k)
                    R_trig: resistor(1k)
                    Q_sweep: npn(2n3904)
                    R_base: resistor(47k)
                    C_env: cap(100n)
                    R_env: resistor(100k)
                    Decay: pot(500k, b)
                    U2: opamp(tl072)
                    Ri_out: resistor(10k)
                    Rf_out: resistor(470k)
                    R_click: resistor(470k)
                    C_click: cap(100p)
                }
                nets {
                    vcc -> Rb1.a
                    Rb1.b -> Rb2.a, U1.pos
                    Rb2.b -> gnd
                    U1.neg -> R1.a, C1.a
                    R1.b -> R2.a, C2.a
                    R2.b -> U1.out
                    C1.b -> gnd
                    C2.b -> gnd
                    U1.neg -> R_fb.a
                    R_fb.b -> U1.out
                    U1.neg -> Q_sweep.collector
                    Q_sweep.emitter -> R1.b
                    in -> C_env.a
                    C_env.b -> R_base.a
                    R_base.b -> Q_sweep.base
                    Q_sweep.base -> R_env.a
                    R_env.b -> gnd
                    in -> R_trig.a
                    R_trig.b -> R1.b
                    R1.b -> Decay.a
                    Decay.b -> gnd
                    in -> R_click.a
                    R_click.b -> C_click.a
                    C_click.b -> Ri_out.b
                    U1.out -> Ri_out.a
                    Ri_out.b -> U2.neg
                    U2.neg -> Rf_out.a
                    Rf_out.b -> U2.out
                    U2.pos -> gnd
                    U2.out -> out
                }
                controls {
                    Decay.position -> "Decay" [0.0, 1.0] = 1.0
                }
            }"#,
        );

        let groups = find_flow_groups(&edges, &graph);
        let u1_group = find_group_containing(&groups, &graph, "U1").expect("Should have U1");
        let q_group =
            find_group_containing(&groups, &graph, "Q_sweep").expect("Should have Q_sweep");

        assert_ne!(
            Some(u1_group),
            Some(q_group),
            "BJT shunt/modulator across bridged-T R1 should not merge into the opamp SCC"
        );
    }

    // ── Test 6: Ground shunt diodes independent ──────────────────────────

    #[test]
    fn flow_ground_shunt_diodes_independent() {
        let (graph, edges) = make_graph_all_edges(
            r#"
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
            }"#,
        );

        let groups = find_flow_groups(&edges, &graph);

        let d1_group = find_group_containing(&groups, &graph, "D1");
        let d2_group = find_group_containing(&groups, &graph, "D2");

        assert!(d1_group.is_some());
        assert!(d2_group.is_some());
        assert_ne!(
            d1_group, d2_group,
            "Ground shunt diodes should be in separate groups (GND is a rail)"
        );
    }

    // ── Test 7: Klon Centaur — four op-amps cascade ──────────────────────

    #[test]
    fn flow_klon_four_opamps_cascade() {
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    R_in: resistor(10k)
                    U1: opamp(tl072)
                    Rf1: resistor(100k)
                    Ri1: resistor(10k)
                    R12: resistor(10k)
                    U2: opamp(tl072)
                    Rf2: resistor(47k)
                    Ri2: resistor(10k)
                    R23: resistor(10k)
                    U3: opamp(tl072)
                    Rf3: resistor(100k)
                    Ri3: resistor(10k)
                    R34: resistor(10k)
                    U4: opamp(tl072)
                    Rf4: resistor(47k)
                    Ri4: resistor(10k)
                    R_out: resistor(10k)
                }
                nets {
                    in -> R_in.a
                    R_in.b -> Ri1.a
                    Ri1.b -> U1.neg
                    U1.neg -> Rf1.a
                    Rf1.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> R12.a
                    R12.b -> Ri2.a
                    Ri2.b -> U2.neg
                    U2.neg -> Rf2.a
                    Rf2.b -> U2.out
                    U2.pos -> gnd
                    U2.out -> R23.a
                    R23.b -> Ri3.a
                    Ri3.b -> U3.neg
                    U3.neg -> Rf3.a
                    Rf3.b -> U3.out
                    U3.pos -> gnd
                    U3.out -> R34.a
                    R34.b -> Ri4.a
                    Ri4.b -> U4.neg
                    U4.neg -> Rf4.a
                    Rf4.b -> U4.out
                    U4.pos -> gnd
                    U4.out -> R_out.a
                    R_out.b -> out
                }
                controls {}
            }"#,
        );

        let groups = find_flow_groups(&edges, &graph);

        let u1_group = find_group_containing(&groups, &graph, "U1");
        let u2_group = find_group_containing(&groups, &graph, "U2");
        let u3_group = find_group_containing(&groups, &graph, "U3");
        let u4_group = find_group_containing(&groups, &graph, "U4");

        assert!(u1_group.is_some(), "Should have U1 group");
        assert!(u2_group.is_some(), "Should have U2 group");
        assert!(u3_group.is_some(), "Should have U3 group");
        assert!(u4_group.is_some(), "Should have U4 group");

        // All four should be in DIFFERENT groups
        let all_groups = [u1_group, u2_group, u3_group, u4_group];
        for i in 0..4 {
            for j in (i + 1)..4 {
                assert_ne!(
                    all_groups[i],
                    all_groups[j],
                    "U{} and U{} should be in separate groups (cascade, no inter-stage feedback)",
                    i + 1,
                    j + 1
                );
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════════
    // Diode-in-feedback edge cases (TS/RAT/SD1/Klon pattern)
    // ═══════════════════════════════════════════════════════════════════

    #[test]
    fn flow_diode_in_opamp_feedback_same_group() {
        // RAT-style: diode between U1.neg and U1.out.
        // The diode IS in the op-amp's feedback loop.
        // They MUST be in the same group — splitting kills audio output.
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    R1: resistor(10k)
                    U1: opamp(tl072)
                    Rf: resistor(100k)
                    D1: diode(silicon)
                }
                nets {
                    in -> R1.a
                    R1.b -> U1.neg
                    U1.neg -> Rf.a
                    Rf.b -> U1.out
                    U1.neg -> D1.a
                    D1.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> out
                }
                controls {}
            }"#,
        );
        let groups = find_flow_groups(&edges, &graph);

        let u1_group = find_group_containing(&groups, &graph, "U1");
        let d1_group = find_group_containing(&groups, &graph, "D1");

        assert!(u1_group.is_some(), "Should find U1 group");
        assert!(d1_group.is_some(), "Should find D1 group");
        assert_eq!(
            u1_group, d1_group,
            "Diode in feedback MUST be in same group as op-amp"
        );

        // The combined group should have feedback
        let group = &groups[u1_group.unwrap()];
        assert!(
            group.has_feedback(),
            "Op-amp + diode feedback group should have feedback=true"
        );
    }

    #[test]
    fn flow_diode_pair_in_opamp_feedback() {
        // TS-style: anti-parallel diode pair in feedback.
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    R1: resistor(10k)
                    U1: opamp(tl072)
                    Rf: resistor(100k)
                    D1: diode(silicon)
                    D2: diode(silicon)
                }
                nets {
                    in -> R1.a
                    R1.b -> U1.neg
                    U1.neg -> Rf.a
                    Rf.b -> U1.out
                    U1.neg -> D1.a
                    D1.b -> U1.out
                    U1.neg -> D2.b
                    D2.a -> U1.out
                    U1.pos -> gnd
                    U1.out -> out
                }
                controls {}
            }"#,
        );
        let groups = find_flow_groups(&edges, &graph);

        let u1_group = find_group_containing(&groups, &graph, "U1");
        let d1_group = find_group_containing(&groups, &graph, "D1");
        let d2_group = find_group_containing(&groups, &graph, "D2");

        assert_eq!(u1_group, d1_group, "D1 must be in same group as U1");
        assert_eq!(u1_group, d2_group, "D2 must be in same group as U1");
    }

    #[test]
    fn flow_diode_after_opamp_not_in_feedback() {
        // Diode NOT in feedback — it's after the op-amp output, going to ground.
        // This should be a SEPARATE group (standalone clipper stage).
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    R1: resistor(10k)
                    U1: opamp(tl072)
                    Rf: resistor(100k)
                    D1: diode(silicon)
                }
                nets {
                    in -> R1.a
                    R1.b -> U1.neg
                    U1.neg -> Rf.a
                    Rf.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> D1.a
                    D1.b -> gnd
                }
                controls {}
            }"#,
        );
        let groups = find_flow_groups(&edges, &graph);

        let u1_group = find_group_containing(&groups, &graph, "U1");
        let d1_group = find_group_containing(&groups, &graph, "D1");

        assert!(u1_group.is_some(), "Should find U1");
        assert!(d1_group.is_some(), "Should find D1");
        // D1 goes to ground, NOT back to U1.neg — should be separate
        assert_ne!(
            u1_group, d1_group,
            "Diode to ground should be separate from op-amp (not in feedback)"
        );
    }

    #[test]
    fn flow_diode_in_feedback_with_pot() {
        // RAT with Drive pot: pot + Rf in parallel, diode in parallel too.
        // Pot/Rf/diode all span U1.neg↔U1.out (feedback path).
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    R1: resistor(1k)
                    U1: opamp(tl072)
                    Rf: resistor(100k)
                    Drive: pot(100k, a)
                    D1: diode(silicon)
                }
                nets {
                    in -> R1.a
                    R1.b -> U1.neg
                    U1.neg -> Rf.a
                    Rf.b -> U1.out
                    U1.neg -> Drive.a
                    Drive.b -> D1.a
                    D1.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> out
                }
                controls {}
            }"#,
        );
        let groups = find_flow_groups(&edges, &graph);
        let u1_group = find_group_containing(&groups, &graph, "U1");
        let d1_group = find_group_containing(&groups, &graph, "D1");

        assert_eq!(
            u1_group, d1_group,
            "Diode + pot in feedback must be in same group as op-amp"
        );
    }

    #[test]
    fn flow_cascade_opamp_then_diode_clipper_separate() {
        // Op-amp gain stage → separate diode clipper stage.
        // The diode clips the signal AFTER the op-amp, not in its feedback.
        // These should be in DIFFERENT groups.
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    R_in: resistor(10k)
                    U1: opamp(tl072)
                    Rf: resistor(100k)
                    R_mid: resistor(4.7k)
                    D1: diode(silicon)
                }
                nets {
                    in -> R_in.a
                    R_in.b -> U1.neg
                    U1.neg -> Rf.a
                    Rf.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> R_mid.a
                    R_mid.b -> D1.a
                    D1.b -> gnd
                    R_mid.b -> out
                }
                controls {}
            }"#,
        );
        let groups = find_flow_groups(&edges, &graph);

        let u1_group = find_group_containing(&groups, &graph, "U1");
        let d1_group = find_group_containing(&groups, &graph, "D1");

        assert!(u1_group.is_some(), "Should find U1");
        assert!(d1_group.is_some(), "Should find D1");
        assert_ne!(
            u1_group, d1_group,
            "Sequential op-amp then diode clipper should be separate groups"
        );
    }

    // ═══════════════════════════════════════════════════════════════════
    // Multi-VCVS cascade: op-amps sharing passive nodes must NOT
    // form false cycles. These are the regression cases.
    // ═══════════════════════════════════════════════════════════════════

    #[test]
    fn flow_two_opamp_cascade_via_pot_separate() {
        // Blues driver pattern: IC1a gain stage → Gain pot → IC1b clipping stage.
        // The Gain pot connects IC1a's feedback node to IC1b's input.
        // BFS from IC1b.neg can reach IC1a.neg through the pot — but that's
        // NOT feedback, it's just passive connectivity. They should be separate.
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    R_in: resistor(10k)
                    IC1a: opamp(tl072)
                    Rf1: resistor(100k)
                    Gain: pot(100k, a)
                    R4: resistor(4.7k)
                    IC1b: opamp(tl072)
                    Rf2: resistor(47k)
                }
                nets {
                    in -> R_in.a
                    R_in.b -> IC1a.neg
                    IC1a.neg -> Rf1.a
                    Rf1.b -> IC1a.out
                    IC1a.pos -> gnd
                    IC1a.neg -> Gain.a
                    Gain.b -> gnd
                    Gain.w -> R4.a
                    R4.b -> IC1b.neg
                    IC1b.neg -> Rf2.a
                    Rf2.b -> IC1b.out
                    IC1b.pos -> gnd
                    IC1b.out -> out
                }
                controls {}
            }"#,
        );
        let groups = find_flow_groups(&edges, &graph);

        let ic1a_group = find_group_containing(&groups, &graph, "IC1a");
        let ic1b_group = find_group_containing(&groups, &graph, "IC1b");

        assert!(ic1a_group.is_some(), "Should find IC1a");
        assert!(ic1b_group.is_some(), "Should find IC1b");
        assert_ne!(
            ic1a_group, ic1b_group,
            "Two cascaded op-amps via pot should be SEPARATE groups (not a false cycle)"
        );
    }

    #[test]
    fn flow_buffer_into_summing_amp_via_feedforward_separate() {
        // Goldenrod pattern: U1 (buffer) output feeds U3 (summing amp)
        // through two feedforward paths. BFS from U3.neg can reach U1.out
        // through the feedforward resistor chain — but signal flows U1→U3,
        // not U3→U1. They should be separate.
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    R_in: resistor(10k)
                    U1: opamp(tl072)
                    R3: resistor(10k)
                    U2: opamp(tl072)
                    Rf2: resistor(100k)
                    R_ff: resistor(15k)
                    U3: opamp(tl072)
                    Rf3: resistor(560k)
                }
                nets {
                    in -> U1.pos
                    U1.neg -> U1.out
                    U1.out -> R3.a
                    R3.b -> U2.pos
                    U2.neg -> Rf2.a
                    Rf2.b -> U2.out
                    U2.neg -> gnd
                    U2.out -> U3.neg
                    U1.out -> R_ff.a
                    R_ff.b -> U3.neg
                    U3.neg -> Rf3.a
                    Rf3.b -> U3.out
                    U3.pos -> gnd
                    U3.out -> out
                }
                controls {}
            }"#,
        );
        let groups = find_flow_groups(&edges, &graph);

        let u1_group = find_group_containing(&groups, &graph, "U1");
        let u3_group = find_group_containing(&groups, &graph, "U3");

        assert!(u1_group.is_some(), "Should find U1");
        assert!(u3_group.is_some(), "Should find U3");
        assert_ne!(
            u1_group, u3_group,
            "Buffer → summing amp via feedforward should be SEPARATE (not false cycle)"
        );
    }

    #[test]
    fn flow_two_opamp_true_feedback_same_group() {
        // When op-amp B's output DOES feed back to op-amp A's input,
        // they should be in the SAME group. This is true mutual feedback.
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    R_in: resistor(10k)
                    U1: opamp(tl072)
                    Rf1: resistor(100k)
                    R_mid: resistor(10k)
                    U2: opamp(tl072)
                    Rf2: resistor(47k)
                    R_fb: resistor(10k)
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
                    U2.out -> R_fb.a
                    R_fb.b -> U1.neg
                    U2.out -> out
                }
                controls {}
            }"#,
        );
        let groups = find_flow_groups(&edges, &graph);

        let u1_group = find_group_containing(&groups, &graph, "U1");
        let u2_group = find_group_containing(&groups, &graph, "U2");

        assert_eq!(
            u1_group, u2_group,
            "True mutual feedback (U2.out → R_fb → U1.neg) must be same group"
        );
    }

    #[test]
    fn flow_opamp_cascade_shared_bias_separate() {
        // Two op-amps sharing a bias node (Vref). This is NOT coupling —
        // bias nodes are rails, not signal paths. They should be separate.
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    R_in: resistor(10k)
                    U1: opamp(tl072)
                    Rf1: resistor(100k)
                    R_mid: resistor(10k)
                    U2: opamp(tl072)
                    Rf2: resistor(47k)
                    R_bias1: resistor(100k)
                    R_bias2: resistor(100k)
                }
                nets {
                    in -> R_in.a
                    R_in.b -> U1.neg
                    U1.neg -> Rf1.a
                    Rf1.b -> U1.out
                    U1.pos -> R_bias1.b
                    U1.out -> R_mid.a
                    R_mid.b -> U2.neg
                    U2.neg -> Rf2.a
                    Rf2.b -> U2.out
                    U2.pos -> R_bias1.b
                    vcc -> R_bias1.a
                    R_bias1.b -> R_bias2.a
                    R_bias2.b -> gnd
                    U2.out -> out
                }
                controls {}
            }"#,
        );
        let groups = find_flow_groups(&edges, &graph);

        let u1_group = find_group_containing(&groups, &graph, "U1");
        let u2_group = find_group_containing(&groups, &graph, "U2");

        assert!(u1_group.is_some(), "Should find U1");
        assert!(u2_group.is_some(), "Should find U2");
        assert_ne!(
            u1_group, u2_group,
            "Shared bias node should NOT create false coupling between cascaded op-amps"
        );
    }

    // ── Multi-VCVS splitting: extended edge cases ────────────────────────

    #[test]
    fn flow_cascade_with_diodes_in_second_stage_separate() {
        // Blues driver full pattern: IC1a (clean gain) → pot → IC1b (clipping).
        // IC1b has diode pair D1/D2 in its feedback. The diodes should be
        // grouped with IC1b (same feedback loop), but IC1a should be separate.
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    R_in: resistor(10k)
                    IC1a: opamp(tl072)
                    Rf1: resistor(100k)
                    Gain: pot(100k, a)
                    R4: resistor(4.7k)
                    IC1b: opamp(tl072)
                    Rf2: resistor(47k)
                    D1: diode(silicon)
                    D2: diode(silicon)
                }
                nets {
                    in -> R_in.a
                    R_in.b -> IC1a.neg
                    IC1a.neg -> Rf1.a
                    Rf1.b -> IC1a.out
                    IC1a.pos -> gnd
                    IC1a.neg -> Gain.a
                    Gain.b -> gnd
                    Gain.w -> R4.a
                    R4.b -> IC1b.neg
                    IC1b.neg -> Rf2.a
                    Rf2.b -> IC1b.out
                    IC1b.neg -> D1.a
                    D1.b -> IC1b.out
                    IC1b.neg -> D2.b
                    D2.a -> IC1b.out
                    IC1b.pos -> gnd
                    IC1b.out -> out
                }
                controls {}
            }"#,
        );
        let groups = find_flow_groups(&edges, &graph);

        let ic1a_group = find_group_containing(&groups, &graph, "IC1a");
        let ic1b_group = find_group_containing(&groups, &graph, "IC1b");
        let d1_group = find_group_containing(&groups, &graph, "D1");

        assert!(ic1a_group.is_some(), "Should find IC1a");
        assert!(ic1b_group.is_some(), "Should find IC1b");
        assert_ne!(
            ic1a_group, ic1b_group,
            "IC1a (clean gain) should be SEPARATE from IC1b (clipping)"
        );
        assert_eq!(
            ic1b_group, d1_group,
            "D1 (in IC1b's feedback) should be in SAME group as IC1b"
        );
    }

    #[test]
    fn flow_three_opamp_cascade_all_separate() {
        // Goldenrod-like: U1 (buffer) → U2 (gain) → U3 (tone).
        // All three should be in separate groups — no mutual feedback.
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    R1: resistor(10k)
                    U1: opamp(tl072)
                    Rf1: resistor(10k)
                    R2: resistor(10k)
                    U2: opamp(tl072)
                    Rf2: resistor(100k)
                    R3: resistor(10k)
                    U3: opamp(tl072)
                    Rf3: resistor(47k)
                }
                nets {
                    in -> R1.a
                    R1.b -> U1.neg
                    U1.neg -> Rf1.a
                    Rf1.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> R2.a
                    R2.b -> U2.neg
                    U2.neg -> Rf2.a
                    Rf2.b -> U2.out
                    U2.pos -> gnd
                    U2.out -> R3.a
                    R3.b -> U3.neg
                    U3.neg -> Rf3.a
                    Rf3.b -> U3.out
                    U3.pos -> gnd
                    U3.out -> out
                }
                controls {}
            }"#,
        );
        let groups = find_flow_groups(&edges, &graph);

        let u1_group = find_group_containing(&groups, &graph, "U1");
        let u2_group = find_group_containing(&groups, &graph, "U2");
        let u3_group = find_group_containing(&groups, &graph, "U3");

        assert!(u1_group.is_some() && u2_group.is_some() && u3_group.is_some());

        let all_different = u1_group != u2_group && u2_group != u3_group && u1_group != u3_group;
        assert!(
            all_different,
            "Three cascaded op-amps should all be in SEPARATE groups: U1={:?} U2={:?} U3={:?}",
            u1_group, u2_group, u3_group
        );
    }

    #[test]
    fn flow_global_negative_feedback_same_group() {
        // Global negative feedback: U1 (gain) → U2 (output), U2.out feeds
        // back to U1.neg via a resistor. This IS true inter-stage feedback.
        // They must stay in the same group.
        // (Same pattern as flow_two_opamp_true_feedback_same_group but with
        // a summing junction at U1.neg — the global NFB goes to the same
        // node as the local feedback Rf1.)
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    R_in: resistor(10k)
                    U1: opamp(tl072)
                    Rf1: resistor(100k)
                    R_mid: resistor(10k)
                    U2: opamp(tl072)
                    Rf2: resistor(100k)
                    R_nfb: resistor(470k)
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
                    U2.out -> R_nfb.a
                    R_nfb.b -> U1.neg
                    U2.out -> out
                }
                controls {}
            }"#,
        );
        let groups = find_flow_groups(&edges, &graph);

        let u1_group = find_group_containing(&groups, &graph, "U1");
        let u2_group = find_group_containing(&groups, &graph, "U2");

        assert_eq!(
            u1_group, u2_group,
            "Global NFB (U2.out → R_nfb → U1.neg) is true feedback — must stay SAME group"
        );
    }

    // ── Test: BJT ladder last-stage emitter chain claimed ────────────────
    //
    // In a diode-connected BJT cascade (TB303 ladder), each BJT's emitter
    // connects to the next BJT's base. For Q1-Q3, their emitter node IS an
    // input_terminal for the next BJT, so emitter components get claimed as
    // pendants. Q4 (last stage) has NO next BJT — its emitter is NOT an
    // input_terminal. Without BFS expansion, R_e_floor4 and Cutoff4 escape
    // into an unclaimed passive group, creating a degenerate IIR that kills
    // signal.

    #[test]
    fn flow_bjt_ladder_last_stage_emitter_components_claimed() {
        // Minimal 2-stage diode-connected BJT ladder.
        // Q2 is the last stage — R_e2 + Cutoff2 must be in Q2's group.
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    Q1: npn(2n3904)
                    Q2: npn(2n3904)
                    R_bias1: resistor(100k)
                    R_bias2: resistor(100k)
                    C1: cap(33n)
                    C2: cap(33n)
                    R_e1: resistor(220)
                    Cutoff1: resistor(5k)
                    R_e2: resistor(220)
                    Cutoff2: resistor(5k)
                    R_in: resistor(10k)
                    C_out: cap(100n)
                    R_out: resistor(100k)
                }
                nets {
                    vcc -> R_bias1.a
                    R_bias1.b -> Q1.base, Q1.collector
                    vcc -> R_bias2.a
                    R_bias2.b -> Q2.base, Q2.collector
                    in -> R_in.a
                    R_in.b -> Q1.base
                    Q1.emitter -> C1.a
                    C1.b -> gnd
                    Q1.emitter -> R_e1.a
                    R_e1.b -> Cutoff1.a
                    Cutoff1.b -> gnd
                    Q1.emitter -> Q2.base
                    Q2.emitter -> C2.a
                    C2.b -> gnd
                    Q2.emitter -> R_e2.a
                    R_e2.b -> Cutoff2.a
                    Cutoff2.b -> gnd
                    Q2.emitter -> C_out.a
                    C_out.b -> R_out.a
                    R_out.b -> gnd
                    C_out.b -> out
                }
                controls {}
            }"#,
        );

        let groups = find_flow_groups(&edges, &graph);

        // R_e2 and Cutoff2 must be in the same group as Q2
        let q2_group = find_group_containing(&groups, &graph, "Q2");
        let re2_group = find_group_containing(&groups, &graph, "R_e2");
        let cutoff2_group = find_group_containing(&groups, &graph, "Cutoff2");

        assert!(q2_group.is_some(), "Q2 should be in a group");
        assert_eq!(
            q2_group, re2_group,
            "R_e2 (last-stage emitter resistor) must be in Q2's group, not a separate passive group"
        );
        assert_eq!(
            q2_group, cutoff2_group,
            "Cutoff2 (last-stage emitter pot) must be in Q2's group, not a separate passive group"
        );
    }

    #[test]
    fn flow_bjt_ladder_output_coupling_separate() {
        // Output coupling (C_out + R_out) should NOT be in the BJT group.
        // They are post-amplification components in a separate passive stage.
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    Q1: npn(2n3904)
                    R_bias1: resistor(100k)
                    C1: cap(33n)
                    R_e1: resistor(220)
                    Cutoff1: resistor(5k)
                    R_in: resistor(10k)
                    C_out: cap(100n)
                    R_out: resistor(100k)
                }
                nets {
                    vcc -> R_bias1.a
                    R_bias1.b -> Q1.base, Q1.collector
                    in -> R_in.a
                    R_in.b -> Q1.base
                    Q1.emitter -> C1.a
                    C1.b -> gnd
                    Q1.emitter -> R_e1.a
                    R_e1.b -> Cutoff1.a
                    Cutoff1.b -> gnd
                    Q1.emitter -> C_out.a
                    C_out.b -> R_out.a
                    R_out.b -> gnd
                    C_out.b -> out
                }
                controls {}
            }"#,
        );

        let groups = find_flow_groups(&edges, &graph);

        let q1_group = find_group_containing(&groups, &graph, "Q1");
        let re1_group = find_group_containing(&groups, &graph, "R_e1");
        let cutoff1_group = find_group_containing(&groups, &graph, "Cutoff1");

        assert!(q1_group.is_some(), "Q1 should be in a group");
        // Emitter components must be claimed into BJT group
        assert_eq!(
            q1_group, re1_group,
            "R_e1 must be in Q1's group (emitter resistor chain)"
        );
        assert_eq!(
            q1_group, cutoff1_group,
            "Cutoff1 must be in Q1's group (emitter ground termination)"
        );
    }

    #[test]
    fn flow_bjt_ladder_resonance_feedback_claimed() {
        // In a TB303-style ladder, the Resonance pot feeds Q4.emitter
        // back to Q1.base via R_fb_limit. These components connect
        // the last stage back to the first — they must be in the
        // BJT feedback group.
        let (graph, edges) = make_graph_all_edges(
            r#"
            pedal "test" { supply 9V
                components {
                    Q1: npn(2n3904)
                    Q2: npn(2n3904)
                    R_bias1: resistor(100k)
                    R_bias2: resistor(100k)
                    C1: cap(33n)
                    C2: cap(33n)
                    R_e1: resistor(220)
                    R_e2: resistor(220)
                    Cutoff1: resistor(5k)
                    Cutoff2: resistor(5k)
                    R_in: resistor(10k)
                    Resonance: resistor(50k)
                    R_fb_limit: resistor(33k)
                    C_out: cap(100n)
                    R_out: resistor(100k)
                }
                nets {
                    vcc -> R_bias1.a
                    R_bias1.b -> Q1.base, Q1.collector
                    vcc -> R_bias2.a
                    R_bias2.b -> Q2.base, Q2.collector
                    in -> R_in.a
                    R_in.b -> Q1.base
                    Q1.emitter -> C1.a
                    C1.b -> gnd
                    Q1.emitter -> R_e1.a
                    R_e1.b -> Cutoff1.a
                    Cutoff1.b -> gnd
                    Q1.emitter -> Q2.base
                    Q2.emitter -> C2.a
                    C2.b -> gnd
                    Q2.emitter -> R_e2.a
                    R_e2.b -> Cutoff2.a
                    Cutoff2.b -> gnd
                    Q2.emitter -> C_out.a
                    C_out.b -> R_out.a
                    R_out.b -> gnd
                    C_out.b -> out
                    Q2.emitter -> Resonance.a
                    Resonance.b -> R_fb_limit.a
                    R_fb_limit.b -> Q1.base
                }
                controls {}
            }"#,
        );

        let groups = find_flow_groups(&edges, &graph);

        let q1_group = find_group_containing(&groups, &graph, "Q1");
        let res_group = find_group_containing(&groups, &graph, "Resonance");
        let fb_group = find_group_containing(&groups, &graph, "R_fb_limit");

        assert!(q1_group.is_some(), "Q1 should be in a group");
        assert_eq!(
            q1_group, res_group,
            "Resonance (Q2.emitter → Q1.base feedback path) must be in the BJT group"
        );
        assert_eq!(
            q1_group, fb_group,
            "R_fb_limit (feedback limiting resistor) must be in the BJT group"
        );
    }
}
