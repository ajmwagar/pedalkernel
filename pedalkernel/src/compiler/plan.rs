//! Pass 3: Edge-based stage planning.
//!
//! Seven rules map circuit edges to processing stages:
//!
//! 1. **NL grouping**: BFS from each Nonlinear edge through Linear/Reactive
//!    edges. All NL edges reachable without crossing Behavioral → same group.
//! 2. **Lone NL → simple stage**: 1 NL edge, no VCCS → WDF tree with 1-port NR.
//! 3. **Multi-NL or VCCS → R-type**: Reactive → WDF ports, Linear → MNA
//!    conductances, NL → NR ports, VCCS → off-diagonal MNA stamps.
//! 4. **All-linear → passive**: No NL/VCCS/Behavioral → pure passive WDF tree.
//! 5. **Behavioral breaks**: BBD/delay_line creates stage boundaries.
//! 6. **Variable edges → recompute**: Stages with `is_variable()` edges store
//!    recompute state.
//! 7. **No orphans**: Every edge belongs to exactly one stage.

use std::collections::{HashMap, HashSet, VecDeque};

use crate::dsl::*;

use super::bjt_bias_analysis::{self, BjtBiasAnalysis};
use super::classify::{ClassifiedCircuit, NonlinearElement, NonlinearKind};
use super::component::{Component, EdgeKind};
use super::graph::{CircuitGraph, NodeId};

// ═══════════════════════════════════════════════════════════════════════════
// Stage plan
// ═══════════════════════════════════════════════════════════════════════════

/// Plan for a single WDF stage.
pub(super) struct StagePlan {
    /// Passive edge indices for the WDF tree.
    pub(super) passive_idxs: Vec<usize>,
    /// Injection node for the voltage source.
    pub(super) injection_node: NodeId,
    /// Terminal nodes for sp_reduce [source_node, junction/gnd].
    pub(super) terminals: Vec<NodeId>,
    /// Virtual source node ID (unique per element type).
    pub(super) source_node: NodeId,
    /// Virtual edge: (node_a, node_b, comp_idx, resistance, name).
    /// Used for BJT r_ce and triode r_p.
    pub(super) virtual_edge: Option<VirtualEdge>,
    /// Whether to skip voltage source in tree (source follower mode).
    pub(super) skip_vs: bool,
    /// Reference to the classified element.
    pub(super) element_idx: usize,
    /// DC-block filter coefficients: (a1, b0, 0, 0).
    /// Used for triode output coupling.
    pub(super) dc_block: Option<(f64, f64, f64, f64)>,
    /// Compensation factor (from triode mu, OTA feedback, etc.)
    pub(super) compensation: f64,
    /// Signal chain depth (boundary crossings from input). Used for stage ordering.
    pub(super) signal_chain_depth: Option<usize>,
}

/// Virtual edge connecting internal terminals of 3-terminal elements.
pub(super) struct VirtualEdge {
    pub(super) node_a: NodeId,
    pub(super) node_b: NodeId,
    pub(super) resistance: f64,
    pub(super) name: &'static str,
}

// ═══════════════════════════════════════════════════════════════════════════
// Push-pull plan
// ═══════════════════════════════════════════════════════════════════════════

/// Plan for a push-pull triode pair.
pub(super) struct PushPullPlan {
    /// Index into classified triodes for the push half.
    pub(super) push_triode_list_idx: usize,
    /// Index into classified triodes for the pull half.
    pub(super) pull_triode_list_idx: usize,
    /// Edge index of the output transformer in the circuit graph.
    pub(super) transformer_edge_idx: usize,
    /// Turns ratio of the CT transformer.
    pub(super) turns_ratio: f64,
}

// ═══════════════════════════════════════════════════════════════════════════
// Multi-NL plan (R-type adaptor approach)
// ═══════════════════════════════════════════════════════════════════════════

/// Plan for coupled nonlinear elements using R-type adaptor + multi-port NR.
pub(super) struct MultiNlPlan {
    /// Indices into `classified.nonlinear_elements` for the coupled NL elements.
    pub(super) nl_element_indices: Vec<usize>,
    /// Index of the NL element that produces the final output.
    pub(super) output_element_idx: usize,
    /// All passive edge indices from the coupled network.
    pub(super) passive_edge_indices: Vec<usize>,
    /// Node where the voltage source (input signal) is injected.
    pub(super) injection_node: NodeId,
    /// Terminal pairs for each NL element: (positive_node, negative_node).
    /// For BJTs: (collector, emitter). For triodes: (plate, cathode).
    pub(super) nl_terminals: Vec<(NodeId, NodeId)>,
    /// Compensation factor.
    pub(super) compensation: f64,
    /// Optional output node for passive-port output extraction.
    pub(super) output_node: Option<NodeId>,
    /// Linearized OTA VCCS stamps for the MNA.
    pub(super) ota_vccs: Vec<OtaVccsInfo>,
    /// Signal chain depth (boundary crossings from input). Used for stage ordering.
    pub(super) signal_chain_depth: Option<usize>,
}

/// Info for stamping a linearized OTA as a VCCS in the MNA.
pub(super) struct OtaVccsInfo {
    /// Non-inverting input node (signal input).
    pub(super) in_pos: NodeId,
    /// Inverting input node (feedback).
    pub(super) in_neg: NodeId,
    /// Output current node.
    pub(super) out_node: NodeId,
    /// Component index for looking up the OTA model.
    pub(super) comp_idx: usize,
}

// ═══════════════════════════════════════════════════════════════════════════
// Edge-kind helper
// ═══════════════════════════════════════════════════════════════════════════

/// Map a graph edge to its electrical classification via the Component trait.
fn edge_kind(graph: &CircuitGraph, edge_idx: usize) -> EdgeKind {
    let edge = &graph.edges[edge_idx];
    let comp = &graph.components[edge.comp_idx];
    comp.kind
        .edges()
        .first()
        .map(|e| e.kind)
        .unwrap_or(EdgeKind::Linear)
}

// ═══════════════════════════════════════════════════════════════════════════
// Reactive boundary detection
// ═══════════════════════════════════════════════════════════════════════════

/// Find reactive edges (coupling caps) that separate distinct NL islands.
///
/// Builds connected components using only Linear + Nonlinear edges (skipping
/// Reactive, Behavioral, Vccs, and sidechain edges). A Reactive edge whose
/// both non-hub endpoints lie in different components is a boundary.
///
/// Hub nodes (gnd, vcc, supply, in, out) are excluded from the "different
/// component" check — a cap from a hub to an island node is always internal
/// (e.g., input coupling cap or bypass cap to ground).
/// Result of reactive boundary analysis: boundary edge indices + island union-find.
struct ReactiveAnalysis {
    boundary_edges: HashSet<usize>,
    /// Union-find of non-hub nodes connected by Linear/Nonlinear edges.
    uf: UnionFind,
    n_nodes: usize,
}

fn find_reactive_boundaries(
    graph: &CircuitGraph,
    classified: &ClassifiedCircuit,
) -> ReactiveAnalysis {
    // Compute upper bound on node count from all edge endpoints.
    let n_nodes = graph
        .edges
        .iter()
        .flat_map(|e| [e.node_a, e.node_b])
        .max()
        .map(|m| m + 1)
        .unwrap_or(0);
    if n_nodes == 0 {
        return ReactiveAnalysis {
            boundary_edges: HashSet::new(),
            uf: UnionFind::new(0),
            n_nodes: 0,
        };
    }
    let mut uf = UnionFind::new(n_nodes);

    let is_hub = |node: NodeId| -> bool {
        node == graph.gnd_node
            || node == graph.vcc_node
            || node == graph.in_node
            || node == graph.out_node
            || graph.supply_nodes.contains(&node)
    };

    // Union nodes connected by Linear or Nonlinear edges only.
    // Skip edges touching hub nodes — hubs (gnd, vcc, supply, in, out) are
    // AC ground and should not propagate connectivity between islands.
    // E.g., Q1.collector→vcc via R3 and Q2.collector→vcc via R6 doesn't
    // make Q1 and Q2 the same island.
    for (eidx, e) in graph.edges.iter().enumerate() {
        if classified.sidechain_edge_set.contains(&eidx) {
            continue;
        }
        // Don't union through hub nodes.
        if is_hub(e.node_a) || is_hub(e.node_b) {
            continue;
        }
        let kind = edge_kind(graph, eidx);
        match kind {
            EdgeKind::Linear | EdgeKind::Nonlinear => {
                uf.union(e.node_a, e.node_b);
            }
            _ => {} // Reactive, Behavioral, Vccs — don't union
        }
    }

    // Identify boundary reactive edges.
    let mut boundaries = HashSet::new();
    for (eidx, e) in graph.edges.iter().enumerate() {
        if classified.sidechain_edge_set.contains(&eidx) {
            continue;
        }
        let kind = edge_kind(graph, eidx);
        if kind != EdgeKind::Reactive {
            continue;
        }
        // If either endpoint is a hub, the cap is internal (bypass/input cap).
        if is_hub(e.node_a) || is_hub(e.node_b) {
            continue;
        }
        // Both endpoints are non-hub — check if in different components.
        if uf.find(e.node_a) != uf.find(e.node_b) {
            boundaries.insert(eidx);
        }
    }

    ReactiveAnalysis {
        boundary_edges: boundaries,
        uf,
        n_nodes,
    }
}

/// Compute island depth for each NL element: number of boundary edges
/// between the element's island and the input node's island.
///
/// This gives correct signal chain ordering even when BFS distances
/// through hub nodes collapse to uniform values.
fn compute_island_depths(
    analysis: &mut ReactiveAnalysis,
    graph: &CircuitGraph,
    classified: &ClassifiedCircuit,
) -> HashMap<usize, usize> {
    if analysis.boundary_edges.is_empty() {
        return HashMap::new();
    }

    // Find the input node's island root. If in_node is a hub (which it is),
    // find the island connected to input via the first non-boundary edge from in_node.
    let input_island = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(eidx, e)| {
            !analysis.boundary_edges.contains(eidx)
                && (e.node_a == graph.in_node || e.node_b == graph.in_node)
        })
        .filter_map(|(_, e)| {
            let other = if e.node_a == graph.in_node {
                e.node_b
            } else {
                e.node_a
            };
            // Must be a non-hub node to have a valid island root.
            let is_hub = other == graph.gnd_node
                || other == graph.vcc_node
                || other == graph.out_node
                || graph.supply_nodes.contains(&other);
            if is_hub {
                None
            } else {
                Some(analysis.uf.find(other))
            }
        })
        .next();

    let Some(input_root) = input_island else {
        return HashMap::new();
    };

    // BFS on the island graph: nodes = island roots, edges = boundary edges.
    let mut island_depth: HashMap<usize, usize> = HashMap::new();
    island_depth.insert(input_root, 0);
    let mut queue = VecDeque::new();
    queue.push_back(input_root);

    while let Some(current) = queue.pop_front() {
        let depth = island_depth[&current];
        for &be in &analysis.boundary_edges {
            let e = &graph.edges[be];
            let root_a = analysis.uf.find(e.node_a);
            let root_b = analysis.uf.find(e.node_b);
            if root_a == current && !island_depth.contains_key(&root_b) {
                island_depth.insert(root_b, depth + 1);
                queue.push_back(root_b);
            } else if root_b == current && !island_depth.contains_key(&root_a) {
                island_depth.insert(root_a, depth + 1);
                queue.push_back(root_a);
            }
        }
    }

    // Map each NL element to its island depth.
    let mut elem_depths: HashMap<usize, usize> = HashMap::new();
    for (idx, elem) in classified.nonlinear_elements.iter().enumerate() {
        let edge = &graph.edges[elem.edge_idx];
        // Use whichever endpoint is non-hub.
        let node = if edge.node_a != graph.gnd_node
            && edge.node_a != graph.vcc_node
            && !graph.supply_nodes.contains(&edge.node_a)
        {
            edge.node_a
        } else {
            edge.node_b
        };
        if node < analysis.n_nodes {
            let root = analysis.uf.find(node);
            if let Some(&depth) = island_depth.get(&root) {
                elem_depths.insert(idx, depth);
            }
        }
    }

    elem_depths
}

// ═══════════════════════════════════════════════════════════════════════════
// NL grouping (Rule 1)
// ═══════════════════════════════════════════════════════════════════════════

/// Union-find (disjoint set) for coupling detection.
struct UnionFind {
    parent: Vec<usize>,
}

impl UnionFind {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
        }
    }

    fn find(&mut self, mut x: usize) -> usize {
        while self.parent[x] != x {
            self.parent[x] = self.parent[self.parent[x]]; // path compression
            x = self.parent[x];
        }
        x
    }

    fn union(&mut self, a: usize, b: usize) {
        let ra = self.find(a);
        let rb = self.find(b);
        if ra != rb {
            self.parent[rb] = ra;
        }
    }

    /// Group elements into clusters. Returns map from root → member indices.
    fn clusters(&mut self) -> HashMap<usize, Vec<usize>> {
        let n = self.parent.len();
        let mut clusters: HashMap<usize, Vec<usize>> = HashMap::new();
        for i in 0..n {
            let root = self.find(i);
            clusters.entry(root).or_default().push(i);
        }
        clusters
    }
}

/// Group nonlinear elements by connectivity through passive edges (Rule 1).
///
/// Check whether two NL elements have mutual resistive coupling that requires
/// simultaneous solving in an R-type.
///
/// Two NL elements need grouping only if there are ≥2 distinct resistive paths
/// between them (a cycle in the resistive graph containing both). A single shared
/// node gives one path. A feedback resistor gives the second. Two paths = loop =
/// mutual dependency → group. One path = feedforward → solve sequentially.
///
/// Examples:
/// - Fuzz Face Q1+Q2: shared emitter node + feedback resistor = 2 paths → group
/// - Big Muff Q2+D1: shared collector node only = 1 path → split
fn has_mutual_resistive_coupling(
    seed: &NonlinearElement,
    other: &NonlinearElement,
    graph: &CircuitGraph,
    classified: &ClassifiedCircuit,
    boundary_edges: &HashSet<usize>,
) -> bool {
    // Collect terminal nodes for both elements (non-hub only).
    let is_hub = |node: NodeId| -> bool {
        node == graph.gnd_node
            || node == graph.vcc_node
            || node == graph.in_node
            || node == graph.out_node
            || graph.supply_nodes.contains(&node)
    };

    let seed_nodes: HashSet<NodeId> = {
        let e = &graph.edges[seed.edge_idx];
        let mut nodes: HashSet<NodeId> = [e.node_a, e.node_b]
            .into_iter()
            .filter(|n| !is_hub(*n))
            .collect();
        // Include base/grid node.
        match &seed.kind {
            NonlinearKind::BjtNpn { base_node, .. }
            | NonlinearKind::BjtPnp { base_node, .. } => {
                if !is_hub(*base_node) {
                    nodes.insert(*base_node);
                }
            }
            NonlinearKind::Triode {
                grid_node: Some(gn),
                ..
            } => {
                if !is_hub(*gn) {
                    nodes.insert(*gn);
                }
            }
            _ => {}
        }
        nodes
    };

    let other_nodes: HashSet<NodeId> = {
        let e = &graph.edges[other.edge_idx];
        let mut nodes: HashSet<NodeId> = [e.node_a, e.node_b]
            .into_iter()
            .filter(|n| !is_hub(*n))
            .collect();
        match &other.kind {
            NonlinearKind::BjtNpn { base_node, .. }
            | NonlinearKind::BjtPnp { base_node, .. } => {
                if !is_hub(*base_node) {
                    nodes.insert(*base_node);
                }
            }
            NonlinearKind::Triode {
                grid_node: Some(gn),
                ..
            } => {
                if !is_hub(*gn) {
                    nodes.insert(*gn);
                }
            }
            _ => {}
        }
        nodes
    };

    // Count shared non-hub nodes — each shared node is one direct path.
    let shared_nodes: usize = seed_nodes.intersection(&other_nodes).count();

    // Count distinct resistive paths between any seed terminal and any other terminal.
    // BFS through Linear edges only (no NL, no Reactive, no boundary).
    // Each path found beyond the shared-node count indicates a resistive feedback loop.
    let mut path_count = shared_nodes;
    if path_count >= 2 {
        return true;
    }

    // BFS from seed's non-hub terminal nodes through Linear edges.
    // If we can reach any of other's terminal nodes via a path that doesn't
    // go through a shared node, that's an additional resistive path.
    for &start in &seed_nodes {
        if other_nodes.contains(&start) {
            continue; // Skip shared nodes — already counted.
        }
        let mut visited: HashSet<NodeId> = HashSet::new();
        visited.insert(start);
        // Block traversal through shared nodes — we want to find paths
        // that go around them (creating a cycle).
        for &shared in seed_nodes.intersection(&other_nodes) {
            visited.insert(shared);
        }
        let mut queue = VecDeque::new();
        queue.push_back(start);
        while let Some(node) = queue.pop_front() {
            for (eidx, e) in graph.edges.iter().enumerate() {
                if classified.all_nonlinear_edge_indices.contains(&eidx) {
                    continue;
                }
                if classified.sidechain_edge_set.contains(&eidx) {
                    continue;
                }
                if boundary_edges.contains(&eidx) {
                    continue;
                }
                let neighbor = if e.node_a == node {
                    Some(e.node_b)
                } else if e.node_b == node {
                    Some(e.node_a)
                } else {
                    None
                };
                let Some(n) = neighbor else { continue };
                if is_hub(n) || !visited.insert(n) {
                    continue;
                }
                if other_nodes.contains(&n) {
                    // Found a resistive path from seed to other that avoids
                    // shared nodes — this is a feedback loop.
                    path_count += 1;
                    if path_count >= 2 {
                        return true;
                    }
                }
                // Only traverse Linear edges (resistors, pots — not reactive/caps).
                let kind = edge_kind(graph, eidx);
                if matches!(kind, EdgeKind::Linear) {
                    queue.push_back(n);
                }
            }
        }
    }

    path_count >= 2
}

/// BFS from each NL element's terminal nodes through Linear/Reactive/NL edges.
/// All NL elements reachable from each other without crossing Behavioral edges
/// or global hub nodes (gnd, vcc, supply, in, out) → same group.
///
/// Two NL elements are only grouped if they have mutual resistive coupling
/// (≥2 resistive paths between them, forming a feedback cycle). Elements with
/// only a single shared node (feedforward topology) are kept in separate groups.
///
/// Returns groups as Vec<Vec<usize>> (each inner Vec is classified element indices).
fn group_nl_elements(
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    boundary_edges: &HashSet<usize>,
) -> Vec<Vec<usize>> {
    let n = classified.nonlinear_elements.len();
    if n == 0 {
        return Vec::new();
    }

    // Edge kind for each graph edge.
    let edge_kinds: Vec<EdgeKind> = (0..graph.edges.len())
        .map(|i| edge_kind(graph, i))
        .collect();

    // Map graph edge index → classified element index.
    let edge_to_elem: HashMap<usize, usize> = classified
        .nonlinear_elements
        .iter()
        .enumerate()
        .map(|(idx, elem)| (elem.edge_idx, idx))
        .collect();

    // Global hub nodes: BFS doesn't expand from these.
    let is_hub = |node: NodeId| -> bool {
        node == graph.gnd_node
            || node == graph.vcc_node
            || node == graph.in_node
            || node == graph.out_node
            || graph.supply_nodes.contains(&node)
    };

    // Build adjacency: node → [(edge_idx, neighbor)].
    let mut adj: HashMap<NodeId, Vec<(usize, NodeId)>> = HashMap::new();
    for (eidx, e) in graph.edges.iter().enumerate() {
        if classified.sidechain_edge_set.contains(&eidx) {
            continue;
        }
        adj.entry(e.node_a).or_default().push((eidx, e.node_b));
        adj.entry(e.node_b).or_default().push((eidx, e.node_a));
    }

    let mut uf = UnionFind::new(n);
    let mut visited_from: HashSet<usize> = HashSet::new(); // elem indices already seeded

    for seed_elem_idx in 0..n {
        if visited_from.contains(&seed_elem_idx) {
            continue;
        }
        let seed_edge = classified.nonlinear_elements[seed_elem_idx].edge_idx;
        if classified.sidechain_edge_set.contains(&seed_edge) {
            continue;
        }

        visited_from.insert(seed_elem_idx);
        let mut visited_nodes: HashSet<NodeId> = HashSet::new();
        let mut queue: VecDeque<NodeId> = VecDeque::new();

        // Seed BFS from this element's terminal nodes.
        let seed_edge_data = &graph.edges[seed_edge];
        for node in [seed_edge_data.node_a, seed_edge_data.node_b] {
            if visited_nodes.insert(node) && !is_hub(node) {
                queue.push_back(node);
            }
        }
        // Also seed from extra terminal nodes (base, grid).
        seed_extra_terminals(
            &classified.nonlinear_elements[seed_elem_idx],
            graph,
            &is_hub,
            &mut visited_nodes,
            &mut queue,
        );

        while let Some(node) = queue.pop_front() {
            if let Some(neighbors) = adj.get(&node) {
                for &(eidx, neighbor) in neighbors {
                    match edge_kinds[eidx] {
                        EdgeKind::Behavioral => continue, // Rule 5: don't cross
                        EdgeKind::Nonlinear => {
                            // Found another NL edge — union only if they have
                            // mutual resistive coupling (≥2 resistive paths
                            // forming a feedback cycle). Single shared node
                            // = feedforward → keep separate.
                            if let Some(&other_idx) = edge_to_elem.get(&eidx) {
                                if !visited_from.contains(&other_idx) {
                                    let seed_elem = &classified.nonlinear_elements[seed_elem_idx];
                                    let other_elem = &classified.nonlinear_elements[other_idx];
                                    if has_mutual_resistive_coupling(
                                        seed_elem,
                                        other_elem,
                                        graph,
                                        classified,
                                        boundary_edges,
                                    ) {
                                        visited_from.insert(other_idx);
                                        uf.union(seed_elem_idx, other_idx);
                                        // Expand BFS from the new element's terminals.
                                        let other_edge = &graph.edges[eidx];
                                        for n in [other_edge.node_a, other_edge.node_b] {
                                            if visited_nodes.insert(n) && !is_hub(n) {
                                                queue.push_back(n);
                                            }
                                        }
                                        seed_extra_terminals(
                                            &classified.nonlinear_elements[other_idx],
                                            graph,
                                            &is_hub,
                                            &mut visited_nodes,
                                            &mut queue,
                                        );
                                    }
                                    // Don't union feedforward elements, but still
                                    // traverse through the node for BFS connectivity.
                                }
                            }
                            // Traverse through NL edge nodes.
                            if visited_nodes.insert(neighbor) && !is_hub(neighbor) {
                                queue.push_back(neighbor);
                            }
                        }
                        _ => {
                            // Skip boundary reactive edges — they separate NL islands.
                            if boundary_edges.contains(&eidx) {
                                continue;
                            }
                            // Linear, Reactive, Vccs — traverse through.
                            if visited_nodes.insert(neighbor) && !is_hub(neighbor) {
                                queue.push_back(neighbor);
                            }
                        }
                    }
                }
            }
        }
    }

    let clusters = uf.clusters();
    let mut groups: Vec<Vec<usize>> = clusters.into_values().collect();
    // Sort groups by their smallest element index for deterministic ordering.
    groups.sort_by_key(|g| g.iter().copied().min().unwrap_or(0));
    groups
}

/// Seed BFS queue with extra terminal nodes (base for BJTs, grid for triodes).
fn seed_extra_terminals(
    elem: &NonlinearElement,
    graph: &CircuitGraph,
    is_hub: &impl Fn(NodeId) -> bool,
    visited: &mut HashSet<NodeId>,
    queue: &mut VecDeque<NodeId>,
) {
    match &elem.kind {
        NonlinearKind::BjtNpn { base_node, .. } | NonlinearKind::BjtPnp { base_node, .. } => {
            if visited.insert(*base_node) && !is_hub(*base_node) {
                queue.push_back(*base_node);
            }
        }
        NonlinearKind::Triode {
            grid_node: Some(gn),
            ..
        } => {
            if visited.insert(*gn) && !is_hub(*gn) {
                queue.push_back(*gn);
            }
        }
        _ => {}
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Planning (Rules 2–7)
// ═══════════════════════════════════════════════════════════════════════════

/// Plan all WDF stages from classified nonlinear elements.
///
/// Returns stage plans, push-pull plans, multi-NL plans, push-pull
/// transformer edges, and BJT bias analysis.
pub(super) fn plan_stages(
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    sample_rate: f64,
    envelope_controlled_otas: &HashSet<String>,
) -> (
    Vec<StagePlan>,
    Vec<PushPullPlan>,
    Vec<MultiNlPlan>,
    HashSet<usize>,
    BjtBiasAnalysis,
) {
    // ── Pre-compute reactive boundaries ────────────────────────────────
    let mut analysis = find_reactive_boundaries(graph, classified);
    let island_depths = compute_island_depths(&mut analysis, graph, classified);
    let boundary_edges = analysis.boundary_edges;

    // ── Step 1: Edge-based NL grouping ──────────────────────────────────
    let groups = group_nl_elements(classified, graph, &boundary_edges);

    // ── Step 2: Push-pull detection ─────────────────────────────────────
    // Triodes sharing a CT transformer get push-pull plans instead of
    // normal NL stages. Detect before per-group planning.
    let triode_elements: Vec<(usize, &NonlinearElement)> = classified
        .nonlinear_elements
        .iter()
        .enumerate()
        .filter(|(_, e)| matches!(&e.kind, NonlinearKind::Triode { .. }))
        .collect();

    use super::graph::TriodeInfo;
    let triode_infos: Vec<(usize, TriodeInfo)> = triode_elements
        .iter()
        .map(|(_, elem)| {
            if let NonlinearKind::Triode {
                model_name,
                plate_node,
                cathode_node,
                parallel_count,
                is_vari_mu,
                ..
            } = &elem.kind
            {
                (
                    elem.edge_idx,
                    TriodeInfo {
                        model_name: model_name.clone(),
                        plate_node: *plate_node,
                        cathode_node: *cathode_node,
                        junction_node: *cathode_node,
                        ground_node: graph.gnd_node,
                        parallel_count: *parallel_count,
                        is_vari_mu: *is_vari_mu,
                    },
                )
            } else {
                unreachable!()
            }
        })
        .collect();

    let (push_pull_pairs, mut pp_transformer_edges) =
        graph.find_push_pull_triode_pairs(&triode_infos, &classified.all_nonlinear_edge_indices);

    let pp_driven = graph.find_pp_driven_transformer_edges(
        &push_pull_pairs,
        &triode_infos,
        &classified.all_nonlinear_edge_indices,
    );
    pp_transformer_edges.extend(pp_driven);

    let paired_triode_indices: HashSet<usize> = push_pull_pairs
        .iter()
        .flat_map(|p| [p.push_triode_idx, p.pull_triode_idx])
        .collect();

    let triode_to_classified: Vec<usize> =
        triode_elements.iter().map(|(idx, _)| *idx).collect();

    let push_pull_plans: Vec<PushPullPlan> = push_pull_pairs
        .iter()
        .map(|p| PushPullPlan {
            push_triode_list_idx: triode_to_classified[p.push_triode_idx],
            pull_triode_list_idx: triode_to_classified[p.pull_triode_idx],
            transformer_edge_idx: p.transformer_edge_idx,
            turns_ratio: p.turns_ratio,
        })
        .collect();

    // Set of classified element indices that are push-pull paired.
    let pp_elem_indices: HashSet<usize> = paired_triode_indices
        .iter()
        .map(|&tli| triode_to_classified[tli])
        .collect();

    // ── Step 3: BJT bias pot detection ──────────────────────────────────
    // Collect BJT terminal info for bias pot analysis.
    struct BjtTerminals {
        elem_idx: usize,
        base_node: NodeId,
        collector_node: NodeId,
        emitter_node: NodeId,
    }
    let bjt_terminals: Vec<BjtTerminals> = classified
        .nonlinear_elements
        .iter()
        .enumerate()
        .filter_map(|(idx, e)| match &e.kind {
            NonlinearKind::BjtNpn { base_node, .. }
            | NonlinearKind::BjtPnp { base_node, .. } => Some(BjtTerminals {
                elem_idx: idx,
                base_node: *base_node,
                collector_node: e.junction_nodes[0],
                emitter_node: e.junction_nodes[1],
            }),
            _ => None,
        })
        .collect();

    // Find groups containing 2+ BJTs for bias pot detection.
    let mut bjt_cluster_members: Vec<Vec<usize>> = Vec::new();
    let mut bjt_cluster_passive_edges: Vec<Vec<usize>> = Vec::new();

    for group in &groups {
        let bjt_members: Vec<usize> = group
            .iter()
            .copied()
            .filter(|&idx| {
                matches!(
                    &classified.nonlinear_elements[idx].kind,
                    NonlinearKind::BjtNpn { .. } | NonlinearKind::BjtPnp { .. }
                )
            })
            .collect();

        if bjt_members.len() >= 2 {
            // Collect passive edges from all BJT junction nodes.
            let mut all_junction_nodes: HashSet<NodeId> = HashSet::new();
            for &idx in &bjt_members {
                if let Some(bt) = bjt_terminals.iter().find(|b| b.elem_idx == idx) {
                    all_junction_nodes.insert(bt.collector_node);
                    all_junction_nodes.insert(bt.emitter_node);
                }
            }
            let passive_edges = collect_passive_edges_from_nodes(
                &all_junction_nodes,
                graph,
                classified,
                true, // skip_out_node
                &pp_transformer_edges,
                &HashSet::new(),
            );

            // Convert bjt_members to indices into bjt_terminals for bias detection.
            let member_indices: Vec<usize> = bjt_members
                .iter()
                .filter_map(|&elem_idx| {
                    bjt_terminals.iter().position(|b| b.elem_idx == elem_idx)
                })
                .collect();

            bjt_cluster_members.push(member_indices);
            bjt_cluster_passive_edges.push(passive_edges);
        }
    }

    let bjt_emitter_nodes: Vec<NodeId> =
        bjt_terminals.iter().map(|b| b.emitter_node).collect();
    let bjt_bias_analysis = bjt_bias_analysis::detect_bias_pots(
        graph,
        &bjt_cluster_members,
        &bjt_emitter_nodes,
        &bjt_cluster_passive_edges,
    );

    // ── Step 4: Plan each group ──────────────────────────────────────────
    let mut plans: Vec<StagePlan> = Vec::new();
    let mut multi_nl_plans: Vec<MultiNlPlan> = Vec::new();
    let mut source_node_offset = 1000usize;

    for group in &groups {
        // Filter out sidechain and push-pull elements.
        let active_members: Vec<usize> = group
            .iter()
            .copied()
            .filter(|&idx| {
                let elem = &classified.nonlinear_elements[idx];
                !classified.sidechain_edge_set.contains(&elem.edge_idx)
                    && !pp_elem_indices.contains(&idx)
            })
            .collect();

        if active_members.is_empty() {
            continue;
        }

        if active_members.len() == 1 {
            // ── Rule 2: Lone NL → check for upgrades, else simple stage ──
            let elem_idx = active_members[0];
            let elem = &classified.nonlinear_elements[elem_idx];

            // Check: VariMu 3-port upgrade (grid-side passives → MultiNlPlan).
            if let Some(plan) = try_varimu_3port(
                elem,
                elem_idx,
                classified,
                graph,
                &pp_transformer_edges,
            ) {
                multi_nl_plans.push(plan);
                continue;
            }

            // Check: Linearized OTA (envelope-controlled → MultiNlPlan with VCCS).
            if let Some(plan) = try_linearized_ota(
                elem,
                elem_idx,
                classified,
                graph,
                envelope_controlled_otas,
                &pp_transformer_edges,
            ) {
                multi_nl_plans.push(plan);
                continue;
            }

            // Default: simple WDF stage.
            if let Some(plan) = plan_single_nl(
                elem,
                elem_idx,
                classified,
                graph,
                source_node_offset,
                sample_rate,
                &pp_transformer_edges,
                &boundary_edges,
            ) {
                plans.push(plan);
            }
            source_node_offset += 1000;
        } else {
            // ── Rule 3: Multi-NL → R-type stage ─────────────────────────
            if let Some(plan) = plan_multi_nl_group(
                &active_members,
                classified,
                graph,
                &pp_transformer_edges,
                &bjt_bias_analysis,
                &boundary_edges,
            ) {
                multi_nl_plans.push(plan);
            }
        }
    }

    // ── Set signal_chain_depth on all plans from island depths ─────────
    if !island_depths.is_empty() {
        for plan in plans.iter_mut() {
            if let Some(&depth) = island_depths.get(&plan.element_idx) {
                plan.signal_chain_depth = Some(depth);
            }
        }
        for plan in multi_nl_plans.iter_mut() {
            let min_depth = plan
                .nl_element_indices
                .iter()
                .filter_map(|idx| island_depths.get(idx))
                .min()
                .copied();
            plan.signal_chain_depth = min_depth;
        }
    }

    (
        plans,
        push_pull_plans,
        multi_nl_plans,
        pp_transformer_edges,
        bjt_bias_analysis,
    )
}

// ═══════════════════════════════════════════════════════════════════════════
// Single-NL planning (Rule 2)
// ═══════════════════════════════════════════════════════════════════════════

/// Plan a single nonlinear element as a WDF stage.
///
/// Consolidates diode/JFET/BJT/triode/pentode/MOSFET/zener/OTA planning
/// into one function parameterized by junction count and element kind.
fn plan_single_nl(
    elem: &NonlinearElement,
    elem_idx: usize,
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    source_node_offset: usize,
    sample_rate: f64,
    pp_transformer_edges: &HashSet<usize>,
    boundary_edges: &HashSet<usize>,
) -> Option<StagePlan> {
    let is_two_junction = elem.junction_nodes.len() == 2;

    if is_two_junction {
        plan_two_junction(
            elem,
            elem_idx,
            classified,
            graph,
            source_node_offset,
            sample_rate,
            pp_transformer_edges,
            boundary_edges,
        )
    } else {
        plan_one_junction(elem, elem_idx, classified, graph, source_node_offset, boundary_edges)
    }
}

/// Plan a 1-junction NL element (diode, MOSFET, zener, OTA, JFET).
fn plan_one_junction(
    elem: &NonlinearElement,
    elem_idx: usize,
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    source_node_offset: usize,
    boundary_edges: &HashSet<usize>,
) -> Option<StagePlan> {
    let junction = elem.junction_nodes[0];
    let is_jfet = matches!(&elem.kind, NonlinearKind::Jfet { .. });

    // Collect passive edges at junction, excluding boundary edges to avoid
    // pendants from downstream coupling caps.
    let mut junction_passives = graph.elements_at_junction(
        junction,
        &classified.all_nonlinear_edge_indices,
        &graph.active_edge_indices,
    );
    junction_passives.retain(|idx| !boundary_edges.contains(idx));

    if is_jfet {
        // JFET: check for source follower (junction connects to out_node).
        let junction_to_output: Vec<usize> = graph
            .edges
            .iter()
            .enumerate()
            .filter(|(idx, e)| {
                !classified.all_nonlinear_edge_indices.contains(idx)
                    && !graph.active_edge_indices.contains(idx)
                    && ((e.node_a == junction && e.node_b == graph.out_node)
                        || (e.node_a == graph.out_node && e.node_b == junction))
            })
            .map(|(idx, _)| idx)
            .collect();

        let output_passives = graph.elements_at_junction(
            graph.out_node,
            &classified.all_nonlinear_edge_indices,
            &graph.active_edge_indices,
        );

        let mut passive_idxs = junction_passives;
        extend_dedup(&mut passive_idxs, &junction_to_output);
        extend_dedup(&mut passive_idxs, &output_passives);

        if passive_idxs.is_empty() {
            return None;
        }

        let is_source_follower =
            !junction_to_output.is_empty() || junction == graph.out_node;

        if is_source_follower {
            return Some(StagePlan {
                passive_idxs,
                injection_node: graph.gnd_node,
                terminals: vec![graph.gnd_node, junction],
                source_node: 0,
                virtual_edge: None,
                skip_vs: true,
                element_idx: elem_idx,
                dc_block: None,
                compensation: 1.0,
                signal_chain_depth: None,
            });
        }

        let source_node = graph.edges.len() + source_node_offset;
        let injection_node = find_injection_node(
            &passive_idxs,
            junction,
            &classified.dist_from_in,
            graph,
        );

        Some(StagePlan {
            passive_idxs,
            injection_node,
            terminals: vec![source_node, junction],
            source_node,
            virtual_edge: None,
            skip_vs: false,
            element_idx: elem_idx,
            dc_block: None,
            compensation: 1.0,
            signal_chain_depth: None,
        })
    } else {
        // Simple 1-junction: diode, MOSFET, zener, OTA.
        if junction_passives.is_empty() {
            return None;
        }

        let injection_node = find_injection_node(
            &junction_passives,
            junction,
            &classified.dist_from_in,
            graph,
        );

        let source_node = graph.edges.len() + source_node_offset;
        let compensation = match &elem.kind {
            NonlinearKind::Ota => 0.08,
            _ => 1.0,
        };

        Some(StagePlan {
            passive_idxs: junction_passives,
            injection_node,
            terminals: vec![source_node, junction],
            source_node,
            virtual_edge: None,
            skip_vs: false,
            element_idx: elem_idx,
            dc_block: None,
            compensation,
            signal_chain_depth: None,
        })
    }
}

/// Plan a 2-junction NL element (BJT, triode, pentode).
fn plan_two_junction(
    elem: &NonlinearElement,
    elem_idx: usize,
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    source_node_offset: usize,
    sample_rate: f64,
    pp_transformer_edges: &HashSet<usize>,
    boundary_edges: &HashSet<usize>,
) -> Option<StagePlan> {
    let (node_a, node_b) = (elem.junction_nodes[0], elem.junction_nodes[1]);

    // Collect passives at junction A (plate/collector).
    // Exclude boundary edges — downstream coupling caps create pendants in the
    // upstream stage's tree (the downstream base_node is a dead-end in the
    // upstream context, causing SP reduce corruption).
    let mut a_passives = graph.elements_at_junction(
        node_a,
        &classified.all_nonlinear_edge_indices,
        &graph.active_edge_indices,
    );
    a_passives.retain(|idx| !boundary_edges.contains(idx));

    // Supply edges adjacent to node_a (plate load to vcc_sc, etc.).
    let a_supply_edges: Vec<usize> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(idx, e)| {
            !classified.all_nonlinear_edge_indices.contains(idx)
                && !graph.active_edge_indices.contains(idx)
                && (e.node_a == node_a || e.node_b == node_a)
                && {
                    let other = if e.node_a == node_a {
                        e.node_b
                    } else {
                        e.node_a
                    };
                    graph.supply_nodes.contains(&other)
                }
        })
        .map(|(idx, _)| idx)
        .collect();

    // Collect passives at junction B (cathode/emitter), unless grounded.
    let b_passives = if node_b == graph.gnd_node {
        Vec::new()
    } else {
        graph.elements_at_junction(
            node_b,
            &classified.all_nonlinear_edge_indices,
            &graph.active_edge_indices,
        )
    };

    // Base/grid passives — the input domain of the two-domain device.
    // BJTs and triodes have a control terminal (base/grid) whose voltage
    // determines the NL port current. The WDF tree needs these passives
    // to compute Vbe/Vgk. BFS from base/grid collects the input coupling
    // cap, bias resistors, and everything reachable within this stage.
    let base_passives = match &elem.kind {
        NonlinearKind::BjtNpn { base_node, .. }
        | NonlinearKind::BjtPnp { base_node, .. } => {
            if *base_node != graph.gnd_node {
                let exclude = classified.all_nonlinear_edge_indices.clone();
                // boundary edges are NOT excluded — BFS traverses through
                // coupling cap so it stays in the downstream stage's WDF tree
                graph.bfs_passive_edges(
                    *base_node,
                    &exclude,
                    &graph.active_edge_indices,
                    true,
                    true, // skip_out_node
                    pp_transformer_edges,
                )
            } else {
                Vec::new()
            }
        }
        NonlinearKind::Triode {
            grid_node: Some(gn),
            ..
        } => {
            if *gn != graph.gnd_node {
                let exclude = classified.all_nonlinear_edge_indices.clone();
                // boundary edges are NOT excluded — BFS traverses through
                // coupling cap so it stays in the downstream stage's WDF tree
                graph.bfs_passive_edges(
                    *gn,
                    &exclude,
                    &graph.active_edge_indices,
                    true,
                    true,
                    pp_transformer_edges,
                )
            } else {
                Vec::new()
            }
        }
        _ => Vec::new(),
    };

    // Node A to output edges.
    let a_to_output: Vec<usize> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(idx, e)| {
            !classified.all_nonlinear_edge_indices.contains(idx)
                && !graph.active_edge_indices.contains(idx)
                && ((e.node_a == node_a && e.node_b == graph.out_node)
                    || (e.node_a == graph.out_node && e.node_b == node_a))
        })
        .map(|(idx, _)| idx)
        .collect();

    // Output passives — skip if this stage has no direct connection to out_node.
    // Interior boundary stages and pentodes without output coupling are skipped.
    let is_pentode = matches!(&elem.kind, NonlinearKind::Pentode { .. });
    let output_passives = if (is_pentode || !boundary_edges.is_empty()) && a_to_output.is_empty() {
        Vec::new()
    } else {
        graph.elements_at_junction(
            graph.out_node,
            &classified.all_nonlinear_edge_indices,
            &graph.active_edge_indices,
        )
    };

    let mut passive_idxs = a_passives.clone();
    extend_dedup(&mut passive_idxs, &a_supply_edges);
    extend_dedup(&mut passive_idxs, &b_passives);
    // Always include base/grid passives — the coupling cap (if any) is a real
    // WDF reactive element in the downstream stage's base subtree.
    extend_dedup(&mut passive_idxs, &base_passives);
    extend_dedup(&mut passive_idxs, &a_to_output);
    extend_dedup(&mut passive_idxs, &output_passives);

    // Transformer injection (1-hop from node_a through plate passives).
    let xfmr_inject = find_plate_transformers(&a_passives, node_a, graph);
    extend_dedup(&mut passive_idxs, &xfmr_inject);
    if !xfmr_inject.is_empty() {
        let sec_inject =
            find_secondary_side_transformers(&passive_idxs, graph, &HashSet::new());
        extend_dedup(&mut passive_idxs, &sec_inject);
    }

    if passive_idxs.is_empty() {
        return None;
    }

    // ── Injection node ──────────────────────────────────────────────────
    // For boundary-split BJTs where a boundary edge touches the base node,
    // the injection node is the upstream side of the coupling cap — that's
    // where the inter-stage signal arrives.
    let boundary_injection = match &elem.kind {
        NonlinearKind::BjtNpn { base_node, .. }
        | NonlinearKind::BjtPnp { base_node, .. } => {
            boundary_edges.iter().find_map(|&eidx| {
                let e = &graph.edges[eidx];
                if e.node_a == *base_node {
                    Some(e.node_b) // upstream side
                } else if e.node_b == *base_node {
                    Some(e.node_a) // upstream side
                } else {
                    None
                }
            })
        }
        _ => None,
    };

    let injection_node = boundary_injection.unwrap_or_else(|| {
        find_two_junction_injection(
            &a_passives,
            &b_passives,
            node_a,
            node_b,
            is_pentode,
            classified,
            graph,
        )
    });

    // ── Virtual edge + compensation ─────────────────────────────────────
    let (virtual_edge, compensation, dc_block) = match &elem.kind {
        NonlinearKind::Triode {
            model_name,
            is_vari_mu,
            ..
        } => {
            let model = super::helpers::triode_model(model_name);
            let comp = if *is_vari_mu { 0.35 } else { 1.0 };
            let dc = compute_dc_block(&a_to_output, &output_passives, graph, sample_rate);
            (
                Some(VirtualEdge {
                    node_a,
                    node_b,
                    resistance: model.rp,
                    name: "__triode_rp__",
                }),
                comp,
                dc,
            )
        }
        NonlinearKind::Pentode { model_name } => {
            let model = super::helpers::pentode_model(model_name);
            let dc = compute_dc_block(&a_to_output, &output_passives, graph, sample_rate);
            (
                Some(VirtualEdge {
                    node_a,
                    node_b,
                    resistance: model.rp,
                    name: "__triode_rp__",
                }),
                1.0,
                dc,
            )
        }
        NonlinearKind::BjtNpn { model_name, .. }
        | NonlinearKind::BjtPnp { model_name, .. } => {
            let model = crate::elements::BjtModel::by_name(model_name);
            let r_ce = model.va * 1000.0; // Va / 1mA
            (
                Some(VirtualEdge {
                    node_a,
                    node_b,
                    resistance: r_ce,
                    name: "__bjt_rce__",
                }),
                1.0,
                None,
            )
        }
        _ => (None, 1.0, None),
    };

    let source_node = graph.edges.len() + source_node_offset;

    Some(StagePlan {
        passive_idxs,
        injection_node,
        terminals: vec![source_node, graph.gnd_node],
        source_node,
        virtual_edge,
        skip_vs: false,
        element_idx: elem_idx,
        dc_block,
        compensation,
        signal_chain_depth: None,
    })
}

/// Find injection node for a 2-junction element.
fn find_two_junction_injection(
    a_passives: &[usize],
    b_passives: &[usize],
    node_a: NodeId,
    node_b: NodeId,
    is_pentode: bool,
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
) -> NodeId {
    let mut injection_node = graph.in_node;
    let mut best_dist = usize::MAX;

    if is_pentode {
        // Pentode: check both plate and cathode passives, exclude transformer nodes.
        let is_transformer_node =
            |node: NodeId| -> bool { graph.transformer_info.contains_key(&node) };
        for &eidx in a_passives.iter().chain(b_passives.iter()) {
            let e = &graph.edges[eidx];
            for node in [e.node_a, e.node_b] {
                if node == node_a || node == node_b || is_transformer_node(node) {
                    continue;
                }
                if let Some(&d) = classified.dist_from_in.get(&node) {
                    if d < best_dist {
                        best_dist = d;
                        injection_node = node;
                    }
                }
            }
        }
    } else {
        // Triode/BJT: prefer node_a-side (plate/collector) first.
        for &eidx in a_passives {
            let e = &graph.edges[eidx];
            let other = if e.node_a == node_a {
                e.node_b
            } else {
                e.node_a
            };
            if let Some(&d) = classified.dist_from_in.get(&other) {
                if d < best_dist {
                    best_dist = d;
                    injection_node = other;
                }
            }
        }
        // Fallback: node_b-side (cathode/emitter).
        if best_dist == usize::MAX {
            for &eidx in b_passives {
                let e = &graph.edges[eidx];
                let other = if e.node_a == node_b {
                    e.node_b
                } else {
                    e.node_a
                };
                if let Some(&d) = classified.dist_from_in.get(&other) {
                    if d < best_dist {
                        best_dist = d;
                        injection_node = other;
                    }
                }
            }
        }
    }

    // Last resort: first non-special node from a_passives.
    if best_dist == usize::MAX {
        for &eidx in a_passives {
            let e = &graph.edges[eidx];
            let other = if e.node_a == node_a {
                e.node_b
            } else {
                e.node_a
            };
            if other != graph.gnd_node && other != node_b {
                return other;
            }
        }
        injection_node = graph.gnd_node;
    }

    injection_node
}

/// Compute DC-block filter from output coupling cap + load resistor.
fn compute_dc_block(
    a_to_output: &[usize],
    output_passives: &[usize],
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Option<(f64, f64, f64, f64)> {
    let c_out = a_to_output.iter().find_map(|&idx| {
        graph.components[graph.edges[idx].comp_idx]
            .kind
            .capacitance()
    });
    let r_load = output_passives.iter().find_map(|&idx| {
        graph.components[graph.edges[idx].comp_idx]
            .kind
            .resistance()
    });
    match (c_out, r_load) {
        (Some(c), Some(r)) => {
            let omega = (std::f64::consts::PI * 2.0 / sample_rate) / (r * c);
            let omega_tan = omega.tan();
            let a1 = (1.0 - omega_tan) / (1.0 + omega_tan);
            let b0 = 1.0 / (1.0 + omega_tan);
            Some((a1, b0, 0.0, 0.0))
        }
        _ => None,
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Multi-NL planning (Rule 3)
// ═══════════════════════════════════════════════════════════════════════════

/// Plan a group of coupled NL elements as an R-type (MultiNlPlan).
///
/// Detects diode-only groups and routes them to bridge rectifier planning
/// (which needs skip_out_node=false for RC time constant edges).
fn plan_multi_nl_group(
    elem_indices: &[usize],
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    pp_transformer_edges: &HashSet<usize>,
    bjt_bias_analysis: &BjtBiasAnalysis,
    boundary_edges: &HashSet<usize>,
) -> Option<MultiNlPlan> {
    if elem_indices.is_empty() {
        return None;
    }

    // Check if this is a diode-only group (bridge rectifier).
    let all_diodes = elem_indices.iter().all(|&idx| {
        matches!(
            &classified.nonlinear_elements[idx].kind,
            NonlinearKind::SingleDiode(_) | NonlinearKind::DiodePair(_)
        )
    });

    if all_diodes {
        return plan_diode_bridge(elem_indices, classified, graph, pp_transformer_edges);
    }

    // Order by distance from input (signal-flow order).
    let mut ordered = elem_indices.to_vec();
    ordered.sort_by_key(|&idx| classified.nonlinear_elements[idx].distance);

    // Collect all junction nodes + extra terminal nodes (base, grid).
    // Including base/grid ensures BFS collects bias resistors and allows
    // correct injection node detection.
    let mut all_junction_nodes: HashSet<NodeId> = HashSet::new();
    for &idx in &ordered {
        let elem = &classified.nonlinear_elements[idx];
        for &jn in &elem.junction_nodes {
            all_junction_nodes.insert(jn);
        }
        match &elem.kind {
            NonlinearKind::BjtNpn { base_node, .. }
            | NonlinearKind::BjtPnp { base_node, .. } => {
                all_junction_nodes.insert(*base_node);
            }
            NonlinearKind::Triode {
                grid_node: Some(gn),
                ..
            } => {
                all_junction_nodes.insert(*gn);
            }
            _ => {}
        }
    }

    // Collect passive edges from all junction nodes.
    let mut all_passive_edges = collect_passive_edges_from_nodes(
        &all_junction_nodes,
        graph,
        classified,
        true, // skip_out_node
        pp_transformer_edges,
        boundary_edges,
    );

    // BJT bias pots are now regular passive ports in the R-type adaptor.
    // They self-manage bias via update_bias_from_pot() on MultiNlStage.

    // Debug: show junction nodes and passive edges for boundary-split stages.
    if !boundary_edges.is_empty() {
        let mut jn_sorted: Vec<NodeId> = all_junction_nodes.iter().copied().collect();
        jn_sorted.sort();
        let comp_names: Vec<String> = all_passive_edges
            .iter()
            .map(|&eidx| {
                let comp = &graph.components[graph.edges[eidx].comp_idx];
                format!("{}({}→{})", comp.id, graph.edges[eidx].node_a, graph.edges[eidx].node_b)
            })
            .collect();
        let elem_names: Vec<String> = ordered.iter().map(|&idx| {
            graph.components[graph.edges[classified.nonlinear_elements[idx].edge_idx].comp_idx].id.clone()
        }).collect();
        eprintln!(
            "[plan-multi-nl] elems={:?} junctions={:?} passives={:?}",
            elem_names, jn_sorted, comp_names,
        );
    }

    // For boundary-split stages, the injection point is the first NL element's
    // input terminal (base for BJT, grid for triode) — this is where signal
    // enters the group from the upstream coupling cap. If no boundary touches
    // the input terminal, fall back to standard injection search.
    let injection_node = if !boundary_edges.is_empty() {
        // Get the first NL element's input terminal (base/grid).
        let first_elem = &classified.nonlinear_elements[ordered[0]];
        let input_terminal = match &first_elem.kind {
            NonlinearKind::BjtNpn { base_node, .. }
            | NonlinearKind::BjtPnp { base_node, .. } => Some(*base_node),
            NonlinearKind::Triode {
                grid_node: Some(gn),
                ..
            } => Some(*gn),
            _ => None,
        };

        // Check if any boundary edge connects to this input terminal.
        let boundary_at_input = input_terminal.and_then(|input_term| {
            boundary_edges.iter().any(|&be| {
                let be_edge = &graph.edges[be];
                be_edge.node_a == input_term || be_edge.node_b == input_term
            }).then_some(input_term)
        });

        boundary_at_input.unwrap_or_else(|| {
            find_injection_node_multi_nl(
                &all_passive_edges,
                graph,
                classified,
                &HashSet::new(),
                graph.in_node,
            )
        })
    } else {
        find_injection_node_multi_nl(
            &all_passive_edges,
            graph,
            classified,
            &HashSet::new(),
            graph.in_node,
        )
    };

    // NL terminals: (positive, negative) for each element.
    // BJTs emit 2 terminal pairs: port 0 = (base, emitter), port 1 = (collector, emitter).
    let nl_terminals: Vec<(NodeId, NodeId)> = ordered
        .iter()
        .flat_map(|&idx| {
            let elem = &classified.nonlinear_elements[idx];
            let edge = &graph.edges[elem.edge_idx];
            match &elem.kind {
                NonlinearKind::BjtNpn { base_node, emitter_node, .. }
                | NonlinearKind::BjtPnp { base_node, emitter_node, .. } => {
                    // 2-port: port 0 = (base, emitter), port 1 = (collector, emitter)
                    let collector_node = if edge.node_a == *emitter_node {
                        edge.node_b
                    } else {
                        edge.node_a
                    };
                    vec![(*base_node, *emitter_node), (collector_node, *emitter_node)]
                }
                _ => vec![(edge.node_a, edge.node_b)],
            }
        })
        .collect();

    // Output element: closest to output.
    let output_element_idx = *ordered
        .iter()
        .min_by_key(|&&idx| {
            let elem = &classified.nonlinear_elements[idx];
            let edge = &graph.edges[elem.edge_idx];
            let d_a = classified
                .dist_from_out
                .get(&edge.node_a)
                .copied()
                .unwrap_or(usize::MAX);
            let d_b = classified
                .dist_from_out
                .get(&edge.node_b)
                .copied()
                .unwrap_or(usize::MAX);
            d_a.min(d_b)
        })
        .unwrap();

    if !boundary_edges.is_empty() {
        let elem_names: Vec<String> = ordered.iter().map(|&idx| {
            graph.components[graph.edges[classified.nonlinear_elements[idx].edge_idx].comp_idx].id.clone()
        }).collect();
        eprintln!(
            "[plan-multi-nl] elems={:?} injection_node={} output_elem={}",
            elem_names, injection_node, output_element_idx,
        );
    }

    Some(MultiNlPlan {
        nl_element_indices: ordered,
        output_element_idx,
        passive_edge_indices: all_passive_edges,
        injection_node,
        nl_terminals,
        compensation: 1.0,
        output_node: None,
        ota_vccs: Vec::new(),

        signal_chain_depth: None,
    })
}

// ═══════════════════════════════════════════════════════════════════════════
// Special-case upgrades (VariMu 3-port, linearized OTA)
// ═══════════════════════════════════════════════════════════════════════════

/// Try upgrading a solo VariMu triode to a 3-port MultiNlPlan.
///
/// VariMu triodes with grid-side passives need 2 NL ports:
/// port 0 = (grid, cathode), port 1 = (plate, cathode).
fn try_varimu_3port(
    elem: &NonlinearElement,
    elem_idx: usize,
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    pp_transformer_edges: &HashSet<usize>,
) -> Option<MultiNlPlan> {
    let NonlinearKind::Triode {
        is_vari_mu: true,
        grid_node: Some(grid_node),
        plate_node,
        cathode_node,
        model_name,
        ..
    } = &elem.kind
    else {
        return None;
    };

    if classified.sidechain_edge_set.contains(&elem.edge_idx) {
        return None;
    }

    // Check for grid-side passive edges.
    let grid_passives = graph.bfs_passive_edges(
        *grid_node,
        &classified.all_nonlinear_edge_indices,
        &graph.active_edge_indices,
        true,
        true, // skip_out_node
        pp_transformer_edges,
    );

    if grid_passives.is_empty() {
        return None; // No grid-side passives — stays as 2-port
    }

    // Collect all passive edges from grid + plate + cathode.
    let junction_nodes: HashSet<NodeId> =
        [*grid_node, *plate_node, *cathode_node].into_iter().collect();
    let all_passive_edges = collect_passive_edges_from_nodes(
        &junction_nodes,
        graph,
        classified,
        true,
        pp_transformer_edges,
        &HashSet::new(),
    );

    let exclude: HashSet<NodeId> =
        [*plate_node, *cathode_node, *grid_node].into_iter().collect();
    let injection_node = find_injection_node_multi_nl(
        &all_passive_edges,
        graph,
        classified,
        &exclude,
        graph.in_node,
    );

    // NL terminals: port 0 = (grid, cathode), port 1 = (plate, cathode).
    let nl_terminals = vec![(*grid_node, *cathode_node), (*plate_node, *cathode_node)];

    let _model = super::helpers::triode_model(model_name);

    Some(MultiNlPlan {
        nl_element_indices: vec![elem_idx],
        output_element_idx: elem_idx,
        passive_edge_indices: all_passive_edges,
        injection_node,
        nl_terminals,
        compensation: 0.35, // VariMu
        output_node: None,
        ota_vccs: Vec::new(),

        signal_chain_depth: None,
    })
}

/// Try upgrading a solo envelope-controlled OTA to a linearized MultiNlPlan.
fn try_linearized_ota(
    elem: &NonlinearElement,
    elem_idx: usize,
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    envelope_controlled_otas: &HashSet<String>,
    pp_transformer_edges: &HashSet<usize>,
) -> Option<MultiNlPlan> {
    if !matches!(&elem.kind, NonlinearKind::Ota) {
        return None;
    }
    if classified.sidechain_edge_set.contains(&elem.edge_idx) {
        return None;
    }

    let edge = &graph.edges[elem.edge_idx];
    let comp = &graph.components[edge.comp_idx];
    if !envelope_controlled_otas.contains(&comp.id) {
        return None;
    }

    // Get the OTA's 3 nodes: pos, neg, out.
    let pos_key = format!("{}.pos", comp.id);
    let neg_key = format!("{}.neg", comp.id);
    let out_key = format!("{}.out", comp.id);
    let pos_node = graph
        .node_names
        .get(&pos_key)
        .copied()
        .unwrap_or(edge.node_a);
    let neg_node = graph
        .node_names
        .get(&neg_key)
        .copied()
        .unwrap_or(edge.node_b);
    let out_node = graph.node_names.get(&out_key).copied()?;

    // Collect passive edges from all 3 OTA nodes.
    let junction_nodes: HashSet<NodeId> =
        [pos_node, neg_node, out_node].into_iter().collect();
    let all_passive_edges = collect_passive_edges_from_nodes(
        &junction_nodes,
        graph,
        classified,
        false, // don't skip out_node — output coupling cap is in this network
        pp_transformer_edges,
        &HashSet::new(),
    );

    if all_passive_edges.is_empty() {
        return None;
    }

    let exclude: HashSet<NodeId> = [pos_node, neg_node, out_node].into_iter().collect();
    let injection_node = find_injection_node_multi_nl(
        &all_passive_edges,
        graph,
        classified,
        &exclude,
        graph.in_node,
    );

    Some(MultiNlPlan {
        nl_element_indices: Vec::new(),
        output_element_idx: elem_idx,
        passive_edge_indices: all_passive_edges,
        injection_node,
        nl_terminals: Vec::new(),
        compensation: 1.0,
        output_node: Some(graph.out_node),
        ota_vccs: vec![OtaVccsInfo {
            in_pos: pos_node,
            in_neg: neg_node,
            out_node,
            comp_idx: edge.comp_idx,
        }],

        signal_chain_depth: None,
    })
}

// ═══════════════════════════════════════════════════════════════════════════
// Coupled diode planning (bridge rectifier)
// ═══════════════════════════════════════════════════════════════════════════

/// Plan coupled diodes (bridge rectifier) as a MultiNlPlan.
///
/// Called from `plan_multi_nl_group` when the group contains only diodes
/// and has 3+ distinct terminal nodes (not anti-parallel).
fn plan_diode_bridge(
    elem_indices: &[usize],
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    pp_transformer_edges: &HashSet<usize>,
) -> Option<MultiNlPlan> {
    // Collect all terminal nodes from coupled diodes.
    let mut all_diode_nodes: HashSet<NodeId> = HashSet::new();
    for &idx in elem_indices {
        let edge = &graph.edges[classified.nonlinear_elements[idx].edge_idx];
        all_diode_nodes.insert(edge.node_a);
        all_diode_nodes.insert(edge.node_b);
    }

    // BFS passive edges from ALL terminal nodes, with skip_out_node=false
    // so that RC time constant edges touching out_node are collected.
    let all_passive_edges = collect_passive_edges_from_nodes(
        &all_diode_nodes,
        graph,
        classified,
        false, // skip_out_node=false (bridge rectifier needs RC at output)
        pp_transformer_edges,
        &HashSet::new(),
    );

    // Find injection node — prefer diode terminal nodes.
    let exclude: HashSet<NodeId> = [graph.out_node, graph.gnd_node].into_iter().collect();
    let injection_node = {
        let mut best = find_injection_node_multi_nl(
            &all_passive_edges,
            graph,
            classified,
            &exclude,
            graph.in_node,
        );
        let mut best_dist = classified
            .dist_from_in
            .get(&best)
            .copied()
            .unwrap_or(usize::MAX);

        // Refine: prefer diode terminal nodes.
        for &node in &all_diode_nodes {
            if exclude.contains(&node) {
                continue;
            }
            if let Some(&d) = classified.dist_from_in.get(&node) {
                if d < best_dist {
                    best_dist = d;
                    best = node;
                }
            }
        }
        // Fallback: farthest from output (AC input side).
        if best_dist == usize::MAX {
            let mut best_out_dist = 0usize;
            for &node in &all_diode_nodes {
                if exclude.contains(&node) {
                    continue;
                }
                if let Some(&d) = classified.dist_from_out.get(&node) {
                    if d > best_out_dist {
                        best_out_dist = d;
                        best = node;
                    }
                }
            }
        }
        best
    };

    // NL terminals: (anode, cathode) for each diode.
    let nl_terminals: Vec<(NodeId, NodeId)> = elem_indices
        .iter()
        .map(|&idx| {
            let edge = &graph.edges[classified.nonlinear_elements[idx].edge_idx];
            (edge.node_a, edge.node_b)
        })
        .collect();

    // Output element: diode closest to output.
    let output_element_idx = *elem_indices
        .iter()
        .min_by_key(|&&idx| {
            let edge = &graph.edges[classified.nonlinear_elements[idx].edge_idx];
            let d_a = classified
                .dist_from_out
                .get(&edge.node_a)
                .copied()
                .unwrap_or(usize::MAX);
            let d_b = classified
                .dist_from_out
                .get(&edge.node_b)
                .copied()
                .unwrap_or(usize::MAX);
            d_a.min(d_b)
        })
        .unwrap();

    // Check if any passive edge touches out_node → passive-port output.
    let output_node = if all_passive_edges.iter().any(|&eidx| {
        let e = &graph.edges[eidx];
        e.node_a == graph.out_node || e.node_b == graph.out_node
    }) {
        Some(graph.out_node)
    } else {
        None
    };

    Some(MultiNlPlan {
        nl_element_indices: elem_indices.to_vec(),
        output_element_idx,
        passive_edge_indices: all_passive_edges,
        injection_node,
        nl_terminals,
        compensation: 1.0,
        output_node,
        ota_vccs: Vec::new(),

        signal_chain_depth: None,
    })
}

// ═══════════════════════════════════════════════════════════════════════════
// Push-pull planning
// ═══════════════════════════════════════════════════════════════════════════

/// Plan a push-pull half (for building in build.rs).
pub(super) fn plan_push_pull_half(
    elem: &NonlinearElement,
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    pp_transformer_edges: &HashSet<usize>,
) -> Option<StagePlan> {
    if let NonlinearKind::Triode {
        plate_node,
        cathode_node,
        model_name,
        is_vari_mu,
        ..
    } = &elem.kind
    {
        let plate_passives = graph.bfs_passive_edges(
            *plate_node,
            &classified.all_nonlinear_edge_indices,
            &graph.active_edge_indices,
            true,
            true,
            pp_transformer_edges,
        );
        let cathode_passives = graph.bfs_passive_edges(
            *cathode_node,
            &classified.all_nonlinear_edge_indices,
            &graph.active_edge_indices,
            true,
            true,
            pp_transformer_edges,
        );

        let mut passive_idxs = plate_passives.clone();
        extend_dedup(&mut passive_idxs, &cathode_passives);

        if passive_idxs.is_empty() {
            return None;
        }

        // Find injection node, excluding plate/cathode.
        let exclude: HashSet<NodeId> =
            [*plate_node, *cathode_node].into_iter().collect();
        let mut injection_node = find_injection_node_multi_nl(
            &passive_idxs,
            graph,
            classified,
            &exclude,
            graph.gnd_node,
        );
        if injection_node == graph.gnd_node {
            'outer: for &eidx in &plate_passives {
                let e = &graph.edges[eidx];
                for candidate in [e.node_a, e.node_b] {
                    if candidate != graph.gnd_node
                        && candidate != *cathode_node
                        && candidate != *plate_node
                    {
                        injection_node = candidate;
                        break 'outer;
                    }
                }
            }
        }

        // Ground terminal: degree-1 leaf in cathode passives.
        let mut ground_terminal = graph.gnd_node;
        {
            let mut degree: HashMap<NodeId, usize> = HashMap::new();
            for &eidx in &cathode_passives {
                let e = &graph.edges[eidx];
                *degree.entry(e.node_a).or_insert(0) += 1;
                *degree.entry(e.node_b).or_insert(0) += 1;
            }
            for (&node, &deg) in &degree {
                if deg == 1
                    && node != *plate_node
                    && node != *cathode_node
                    && node != injection_node
                {
                    ground_terminal = node;
                    break;
                }
            }
        }

        let source_node = graph.edges.len() + 5000;
        let terminals = vec![source_node, ground_terminal];

        let model = super::helpers::triode_model(model_name);
        let compensation = if *is_vari_mu {
            find_input_transformer_gain_pp(graph, classified)
        } else {
            model.mu / 100.0
        };

        Some(StagePlan {
            passive_idxs,
            injection_node,
            terminals,
            source_node,
            virtual_edge: Some(VirtualEdge {
                node_a: *plate_node,
                node_b: *cathode_node,
                resistance: model.rp,
                name: "__triode_rp__",
            }),
            skip_vs: false,
            element_idx: 0,
            dc_block: None,
            compensation,
            signal_chain_depth: None,
        })
    } else {
        None
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Shared helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Extend a vec with items from another, skipping duplicates.
fn extend_dedup(dst: &mut Vec<usize>, src: &[usize]) {
    for &idx in src {
        if !dst.contains(&idx) {
            dst.push(idx);
        }
    }
}

/// Find the best injection node for the voltage source.
/// The non-junction endpoint closest to in_node.
fn find_injection_node(
    passive_idxs: &[usize],
    junction: NodeId,
    dist_from_in: &HashMap<NodeId, usize>,
    graph: &CircuitGraph,
) -> NodeId {
    let mut injection_node = graph.in_node;
    let mut best_dist = usize::MAX;
    for &eidx in passive_idxs {
        let e = &graph.edges[eidx];
        let other = if e.node_a == junction {
            e.node_b
        } else {
            e.node_a
        };
        if let Some(&d) = dist_from_in.get(&other) {
            if d < best_dist {
                best_dist = d;
                injection_node = other;
            }
        }
    }
    if best_dist == usize::MAX {
        injection_node = graph.gnd_node;
    }
    injection_node
}

/// Find the injection node closest to the circuit input among passive edge endpoints.
fn find_injection_node_multi_nl(
    passive_edges: &[usize],
    graph: &CircuitGraph,
    classified: &ClassifiedCircuit,
    exclude: &HashSet<NodeId>,
    fallback: NodeId,
) -> NodeId {
    let mut injection_node = fallback;
    let mut best_dist = usize::MAX;
    for &eidx in passive_edges {
        let e = &graph.edges[eidx];
        for node in [e.node_a, e.node_b] {
            if exclude.contains(&node) {
                continue;
            }
            if let Some(&d) = classified.dist_from_in.get(&node) {
                if d < best_dist {
                    best_dist = d;
                    injection_node = node;
                }
            }
        }
    }
    // Fallback: transformer barrier — pick farthest from output.
    if best_dist == usize::MAX {
        let mut best_out_dist = 0usize;
        for &eidx in passive_edges {
            let e = &graph.edges[eidx];
            for node in [e.node_a, e.node_b] {
                if exclude.contains(&node) {
                    continue;
                }
                if let Some(&d) = classified.dist_from_out.get(&node) {
                    if d > best_out_dist {
                        best_out_dist = d;
                        injection_node = node;
                    }
                }
            }
        }
    }
    injection_node
}

/// Collect passive edges via BFS from multiple junction nodes, deduplicating.
///
/// `boundary_edges` are excluded from BFS traversal (coupling caps that
/// separate NL islands). Pass `&HashSet::new()` when boundary-aware splitting
/// is not needed.
fn collect_passive_edges_from_nodes(
    junction_nodes: &HashSet<NodeId>,
    graph: &CircuitGraph,
    classified: &ClassifiedCircuit,
    skip_out_node: bool,
    pp_transformer_edges: &HashSet<usize>,
    boundary_edges: &HashSet<usize>,
) -> Vec<usize> {
    // Merge boundary edges into the NL exclusion set so BFS won't cross them.
    let excluded: Vec<usize> = if boundary_edges.is_empty() {
        classified.all_nonlinear_edge_indices.clone()
    } else {
        let mut excl = classified.all_nonlinear_edge_indices.clone();
        for &be in boundary_edges {
            if !excl.contains(&be) {
                excl.push(be);
            }
        }
        excl
    };

    let mut all_passive_edges: Vec<usize> = Vec::new();
    for &jn in junction_nodes {
        let edges = graph.bfs_passive_edges(
            jn,
            &excluded,
            &graph.active_edge_indices,
            true,
            skip_out_node,
            pp_transformer_edges,
        );
        extend_dedup(&mut all_passive_edges, &edges);
    }
    let xfmr_inject =
        find_secondary_side_transformers(&all_passive_edges, graph, pp_transformer_edges);
    extend_dedup(&mut all_passive_edges, &xfmr_inject);
    all_passive_edges
}

/// Find transformer edges within 1 extra hop from plate passives.
fn find_plate_transformers(
    plate_passives: &[usize],
    plate_node: NodeId,
    graph: &CircuitGraph,
) -> Vec<usize> {
    let mut result = Vec::new();
    for &eidx in plate_passives {
        let e = &graph.edges[eidx];
        let far_node = if e.node_a == plate_node {
            e.node_b
        } else {
            e.node_a
        };
        for (idx, edge) in graph.edges.iter().enumerate() {
            if edge.node_a != far_node && edge.node_b != far_node {
                continue;
            }
            if graph.components[edge.comp_idx].kind.is_transformer() {
                if !plate_passives.contains(&idx) && !result.contains(&idx) {
                    result.push(idx);
                }
            }
        }
    }
    result
}

/// Detect transformer secondary nodes in a passive set and inject primary edges.
fn find_secondary_side_transformers(
    passive_idxs: &[usize],
    graph: &CircuitGraph,
    pp_transformer_edges: &HashSet<usize>,
) -> Vec<usize> {
    let mut inject_edges: Vec<usize> = Vec::new();

    let passive_nodes: HashSet<NodeId> = passive_idxs
        .iter()
        .flat_map(|&eidx| {
            let e = &graph.edges[eidx];
            [e.node_a, e.node_b]
        })
        .collect();

    let mut seen_comp_idx: HashSet<usize> = HashSet::new();
    for &node in &passive_nodes {
        if let Some(info) = graph.transformer_info.get(&node) {
            if info.is_secondary && !seen_comp_idx.contains(&info.comp_idx) {
                seen_comp_idx.insert(info.comp_idx);

                for (eidx, e) in graph.edges.iter().enumerate() {
                    if e.comp_idx == info.comp_idx {
                        if pp_transformer_edges.contains(&eidx) {
                            continue;
                        }
                        if !passive_idxs.contains(&eidx) {
                            inject_edges.push(eidx);
                        }
                    }
                }
            }
        }
    }

    inject_edges
}

/// Find the input transformer voltage gain for a push-pull pair.
fn find_input_transformer_gain_pp(
    graph: &CircuitGraph,
    classified: &ClassifiedCircuit,
) -> f64 {
    let mut best_gain = None;
    let mut best_dist = usize::MAX;

    for (_edge_idx, edge) in graph.edges.iter().enumerate() {
        let comp = &graph.components[edge.comp_idx];
        if let Some(cfg) = comp.kind.transformer_config() {
            if matches!(
                cfg.primary_type,
                WindingType::CenterTap | WindingType::PushPull
            ) {
                continue;
            }

            let dist_a = classified
                .dist_from_in
                .get(&edge.node_a)
                .copied()
                .unwrap_or(usize::MAX);
            let dist_b = classified
                .dist_from_in
                .get(&edge.node_b)
                .copied()
                .unwrap_or(usize::MAX);
            let min_dist = dist_a.min(dist_b);

            if min_dist < best_dist {
                best_dist = min_dist;
                best_gain = Some((1.0 / cfg.turns_ratio) / 2.0);

                #[cfg(feature = "debug-trace")]
                eprintln!(
                    "[TX-COMP] input transformer: id={} ratio={:.4} gain={:.2} dist={min_dist}",
                    comp.id,
                    cfg.turns_ratio,
                    best_gain.unwrap(),
                );
            }
        }
    }

    best_gain.unwrap_or(1.0)
}
