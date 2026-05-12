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

use super::component::{EdgeKind, SignalTerminals};
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
}

/// Set of nodes that are rails (not signal-carrying shared nodes).
fn rail_nodes(graph: &CircuitGraph) -> HashSet<NodeId> {
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
                        &comp.kind.edges().first().map_or("", |e| e.pin_a),
                        &comp.kind.edges().first().map_or("", |e| e.pin_b),
                    );
                    matches!(
                        sem,
                        super::component::PortSemantic::Nonlinear
                            | super::component::PortSemantic::ControlledConductance
                    ) && device_parallels_passive(&comp.id, &e, edge_indices, graph)
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

/// Build undirected signal-edge adjacency (excludes rail nodes).
/// Same as feedback.rs but without virtual amplifier links — directionality
/// is handled by the flow graph, not the adjacency.
fn build_passive_adjacency(
    edge_indices: &[usize],
    graph: &CircuitGraph,
    rails: &HashSet<NodeId>,
    active_edge_set: &HashSet<usize>,
) -> HashMap<NodeId, Vec<(usize, NodeId)>> {
    let mut adj: HashMap<NodeId, Vec<(usize, NodeId)>> = HashMap::new();
    for &eidx in edge_indices {
        // Skip active element edges — we only traverse passive signal paths
        if active_edge_set.contains(&eidx) {
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
    adj: &HashMap<NodeId, Vec<(usize, NodeId)>>,
    blocked_outputs: &HashSet<NodeId>,
) -> HashSet<NodeId> {
    let mut visited = HashSet::new();
    let mut queue = VecDeque::new();
    visited.insert(start);
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
    adj: &HashMap<NodeId, Vec<(usize, NodeId)>>,
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
        let mut blocked_outputs: HashSet<NodeId> = elements
            .iter()
            .enumerate()
            .filter(|&(j, _)| j != i)
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
            let use_restricted =
                comp_j.kind.feedback_input_is_barrier() && restricted_reachable.is_some();
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
    adj: &HashMap<NodeId, Vec<(usize, NodeId)>>,
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
    let mut comp_to_scc: HashMap<usize, usize> = HashMap::new();
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
    let active_output_nodes: HashSet<NodeId> = scc
        .iter()
        .map(|&ei| active_elements[ei].output_node)
        .collect();

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
        let rail_is_ground = rail_node == graph.gnd_node
            || graph.ac_ground_nodes.contains(&rail_node);
        let at_output = active_output_nodes.contains(&non_rail_node) && rail_is_ground;

        if !at_output
            && ((a_rail && group_nodes.contains(&e.node_b))
                || (b_rail && group_nodes.contains(&e.node_a)))
        {
            ground_shunt_edges.push(eidx);
        } else if input_terminals.contains(&e.node_a) || input_terminals.contains(&e.node_b) {
            pendant_edges.push(eidx);
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

    let active_edge_set: HashSet<usize> = active_elements.iter().map(|e| e.edge_idx).collect();
    let adj = build_passive_adjacency(edge_indices, graph, &rails, &active_edge_set);

    // Build directed flow graph and find SCCs
    let flow_adj = build_flow_graph(&active_elements, &adj, &rails, graph);
    let sccs = tarjan_scc(&flow_adj);

    // Merge SCCs that contain edges from the same multi-edge component (e.g. BJT).
    // A BJT has 2 active elements (B-E, C-E) from the same comp_idx. Without
    // feedback they land in separate SCCs, but must be co-solved as one device.
    let sccs = merge_same_component_sccs(sccs, &active_elements, graph);

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
            // Standalone NL element — claim adjacent passive edges
            // (ground shunts and pendants) so SPQR can build a complete
            // WDF tree with the NL element as root.
            let (pendant_edges, ground_shunt_edges) = claim_passive_edges(
                &scc,
                &active_elements,
                &active_edges,
                &[],  // no feedback edges
                edge_indices,
                graph,
                &rails,
                &claimed,
            );
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
            let mut path = Vec::new();
            let mut visited = HashSet::new();
            visited.insert(elem.output_node);
            // Block terminal nodes from expansion
            for &t in &terminal_block {
                visited.insert(t);
            }
            dfs_find_paths(
                elem.output_node,
                elem.input_node,
                &adj,
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
                    let mut path = Vec::new();
                    let mut visited = HashSet::new();
                    visited.insert(elem_i.output_node);
                    for &t in &terminal_block {
                        visited.insert(t);
                    }
                    dfs_find_paths(
                        elem_i.output_node,
                        elem_j.input_node,
                        &adj,
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
        let mut feedback_edges: Vec<usize> = feedback_set.into_iter().collect();
        feedback_edges.sort_unstable(); // deterministic ordering

        // Claim passive edges (ground shunts and pendants) adjacent to
        // the active + feedback edges.
        let (pendant_edges, ground_shunt_edges) = claim_passive_edges(
            &scc,
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
    use super::component::OutputImpedance;

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
    let mut node_to_edges: HashMap<NodeId, Vec<usize>> = HashMap::new();
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
    let edge_pos: HashMap<usize, usize> = edges.iter().enumerate().map(|(i, &e)| (e, i)).collect();
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
}
