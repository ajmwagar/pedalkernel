//! SPQR tree decomposition for circuit graphs.
//!
//! Every 2-connected graph decomposes uniquely into:
//! - **S (Series)**: components in series → WDF series adaptor
//! - **P (Parallel)**: components sharing endpoints → WDF parallel adaptor
//! - **Q (single edge)**: leaf component → WDF leaf element
//! - **R (Rigid)**: non-SP subgraph → MNA scattering matrix
//!
//! The decomposition drives the entire compilation pipeline:
//! - S/P/Q nodes become WDF tree nodes (O(1) per sample)
//! - R nodes become MNA stages (O(N²) per sample)
//! - Tree traversal order = signal routing (no sort needed)

use super::graph::{CircuitGraph, NodeId};

/// A node in the SPQR decomposition tree.
#[derive(Debug, Clone)]
pub(super) enum SpqrNode {
    /// Series: components in series between two cut vertices.
    S {
        children: Vec<SpqrNode>,
        cut_vertices: (NodeId, NodeId),
    },
    /// Parallel: components sharing both endpoints.
    P {
        children: Vec<SpqrNode>,
        endpoints: (NodeId, NodeId),
    },
    /// Single edge (leaf component).
    Q {
        edge_idx: usize,
        endpoints: (NodeId, NodeId),
    },
    /// Rigid (non-SP): requires MNA scattering matrix.
    R {
        edge_indices: Vec<usize>,
        boundary_nodes: Vec<NodeId>,
        children: Vec<SpqrNode>,
    },
}

impl SpqrNode {
    /// Count the total number of edges in this subtree.
    pub fn edge_count(&self) -> usize {
        match self {
            SpqrNode::Q { .. } => 1,
            SpqrNode::S { children, .. } | SpqrNode::P { children, .. } => {
                children.iter().map(|c| c.edge_count()).sum()
            }
            SpqrNode::R { edge_indices, children, .. } => {
                edge_indices.len() + children.iter().map(|c| c.edge_count()).sum::<usize>()
            }
        }
    }

    /// Get the endpoints (boundary nodes) of this SPQR node.
    pub fn endpoints(&self) -> (NodeId, NodeId) {
        match self {
            SpqrNode::S { cut_vertices, .. } => *cut_vertices,
            SpqrNode::P { endpoints, .. } => *endpoints,
            SpqrNode::Q { endpoints, .. } => *endpoints,
            SpqrNode::R { boundary_nodes, .. } => {
                // R-nodes may have more than 2 boundary nodes;
                // return the first two for compatibility.
                let a = boundary_nodes.first().copied().unwrap_or(0);
                let b = boundary_nodes.get(1).copied().unwrap_or(a);
                (a, b)
            }
        }
    }

    /// Check if this node is a leaf (Q node).
    pub fn is_leaf(&self) -> bool {
        matches!(self, SpqrNode::Q { .. })
    }

    /// Check if this node is rigid (R node).
    pub fn is_rigid(&self) -> bool {
        matches!(self, SpqrNode::R { .. })
    }

    /// Collect all edge indices in this subtree.
    pub fn all_edge_indices(&self) -> Vec<usize> {
        let mut result = Vec::new();
        self.collect_edges(&mut result);
        result
    }

    fn collect_edges(&self, result: &mut Vec<usize>) {
        match self {
            SpqrNode::Q { edge_idx, .. } => result.push(*edge_idx),
            SpqrNode::S { children, .. } | SpqrNode::P { children, .. } => {
                for child in children {
                    child.collect_edges(result);
                }
            }
            SpqrNode::R { edge_indices, children, .. } => {
                result.extend(edge_indices);
                for child in children {
                    child.collect_edges(result);
                }
            }
        }
    }
}

/// Decompose a set of edges into an SPQR tree.
///
/// Uses the existing `graph_reduce()` for S/P detection: edges that
/// survive series/parallel reduction form the R-node residual.
/// Pendant subtrees hanging off R-node boundary vertices are
/// recursively decomposed.
///
/// # Arguments
/// * `edge_indices` — edges to decompose
/// * `terminals` — boundary nodes (not collapsed during reduction)
/// * `graph` — the circuit graph
/// * `gnd_node` — ground node ID (treated as terminal)
pub(super) fn spqr_decompose(
    edge_indices: &[usize],
    terminals: &[NodeId],
    graph: &CircuitGraph,
    gnd_node: NodeId,
) -> SpqrNode {
    if edge_indices.is_empty() {
        return SpqrNode::R {
            edge_indices: Vec::new(),
            boundary_nodes: terminals.to_vec(),
            children: Vec::new(),
        };
    }

    // Single edge → Q node
    if edge_indices.len() == 1 {
        let e = &graph.edges[edge_indices[0]];
        return SpqrNode::Q {
            edge_idx: edge_indices[0],
            endpoints: (e.node_a, e.node_b),
        };
    }

    // Build adjacency: node → [(edge_idx, other_node)]
    let mut adj: std::collections::HashMap<NodeId, Vec<(usize, NodeId)>> =
        std::collections::HashMap::new();
    for &eidx in edge_indices {
        let e = &graph.edges[eidx];
        adj.entry(e.node_a).or_default().push((eidx, e.node_b));
        adj.entry(e.node_b).or_default().push((eidx, e.node_a));
    }

    let terminal_set: std::collections::HashSet<NodeId> =
        terminals.iter().copied().collect();

    // Check for parallel edges: multiple edges between the same pair
    let mut edge_pairs: std::collections::HashMap<(NodeId, NodeId), Vec<usize>> =
        std::collections::HashMap::new();
    for &eidx in edge_indices {
        let e = &graph.edges[eidx];
        let key = if e.node_a <= e.node_b {
            (e.node_a, e.node_b)
        } else {
            (e.node_b, e.node_a)
        };
        edge_pairs.entry(key).or_default().push(eidx);
    }

    // If ALL edges share the same endpoints → P node
    if edge_pairs.len() == 1 {
        let (&endpoints, indices) = edge_pairs.iter().next().unwrap();
        let children: Vec<SpqrNode> = indices
            .iter()
            .map(|&eidx| SpqrNode::Q {
                edge_idx: eidx,
                endpoints,
            })
            .collect();
        return SpqrNode::P {
            children,
            endpoints,
        };
    }

    // Try series detection: find internal nodes with degree exactly 2
    // that are not terminals. These can be eliminated (series reduction).
    let mut series_nodes: Vec<NodeId> = Vec::new();
    for (&node, neighbors) in &adj {
        if neighbors.len() == 2 && !terminal_set.contains(&node) && node != gnd_node {
            series_nodes.push(node);
        }
    }

    // If we found series nodes, try to build series chains
    if !series_nodes.is_empty() {
        // Find a chain: walk from a terminal or degree>2 node through
        // degree-2 nodes until hitting another terminal or branch point.
        let mut used_edges: std::collections::HashSet<usize> = std::collections::HashSet::new();
        let mut children: Vec<SpqrNode> = Vec::new();
        let mut chain_start = NodeId::MAX;
        let mut chain_end = NodeId::MAX;

        // Walk from the first series node to build the chain
        if let Some(&start_node) = series_nodes.first() {
            // Walk backward to find chain start (terminal or branch)
            let mut cur = start_node;
            let mut prev = NodeId::MAX;
            loop {
                if let Some(neighbors) = adj.get(&cur) {
                    if neighbors.len() != 2 || terminal_set.contains(&cur) || cur == gnd_node {
                        chain_start = cur;
                        break;
                    }
                    let next = if neighbors[0].1 == prev {
                        neighbors[1].1
                    } else {
                        neighbors[0].1
                    };
                    prev = cur;
                    cur = next;
                } else {
                    chain_start = cur;
                    break;
                }
            }

            // Walk forward collecting edges
            cur = chain_start;
            prev = NodeId::MAX;
            loop {
                if let Some(neighbors) = adj.get(&cur) {
                    // Find the next unused edge
                    let next_edge = neighbors
                        .iter()
                        .find(|&&(eidx, other)| !used_edges.contains(&eidx) && other != prev);
                    if let Some(&(eidx, next_node)) = next_edge {
                        used_edges.insert(eidx);
                        children.push(SpqrNode::Q {
                            edge_idx: eidx,
                            endpoints: (cur, next_node),
                        });
                        prev = cur;
                        cur = next_node;

                        // Stop if we hit a terminal, branch, or dead end
                        let is_end = terminal_set.contains(&cur)
                            || cur == gnd_node
                            || adj.get(&cur).map_or(true, |n| n.len() != 2);
                        if is_end {
                            chain_end = cur;
                            break;
                        }
                    } else {
                        chain_end = cur;
                        break;
                    }
                } else {
                    chain_end = cur;
                    break;
                }
            }
        }

        if children.len() >= 2 {
            // Remaining edges that weren't part of the series chain
            let remaining: Vec<usize> = edge_indices
                .iter()
                .copied()
                .filter(|e| !used_edges.contains(e))
                .collect();

            if remaining.is_empty() {
                return SpqrNode::S {
                    children,
                    cut_vertices: (chain_start, chain_end),
                };
            }

            // Recurse on remaining edges
            let remaining_node = spqr_decompose(&remaining, terminals, graph, gnd_node);
            children.push(remaining_node);
            return SpqrNode::S {
                children,
                cut_vertices: (chain_start, chain_end),
            };
        }
    }

    // Check for parallel subgroups within the edge set
    for ((a, b), indices) in &edge_pairs {
        if indices.len() >= 2 {
            let parallel_children: Vec<SpqrNode> = indices
                .iter()
                .map(|&eidx| SpqrNode::Q {
                    edge_idx: eidx,
                    endpoints: (*a, *b),
                })
                .collect();
            let remaining: Vec<usize> = edge_indices
                .iter()
                .copied()
                .filter(|e| !indices.contains(e))
                .collect();

            if remaining.is_empty() {
                return SpqrNode::P {
                    children: parallel_children,
                    endpoints: (*a, *b),
                };
            }

            // Mix of parallel and other — this is an R-node
            // (or the parallel is a sub-node of a larger structure)
            let mut r_children = vec![SpqrNode::P {
                children: parallel_children,
                endpoints: (*a, *b),
            }];
            if !remaining.is_empty() {
                r_children.push(spqr_decompose(&remaining, terminals, graph, gnd_node));
            }
            return SpqrNode::R {
                edge_indices: Vec::new(),
                boundary_nodes: terminals.to_vec(),
                children: r_children,
            };
        }
    }

    // No series or parallel reduction possible → R node (rigid)
    // Try to find pendant subtrees to extract as children
    let mut pendant_children: Vec<SpqrNode> = Vec::new();
    let mut residual_edges: Vec<usize> = Vec::new();
    let mut junction_nodes: std::collections::HashSet<NodeId> = terminal_set.clone();
    junction_nodes.insert(gnd_node);

    // Nodes with degree >= 3 or that are terminals are junctions
    for (&node, neighbors) in &adj {
        if neighbors.len() >= 3 || terminal_set.contains(&node) || node == gnd_node {
            junction_nodes.insert(node);
        }
    }

    // Classify edges: pendant (touches exactly 1 junction) vs bridging (2+ junctions)
    for &eidx in edge_indices {
        let e = &graph.edges[eidx];
        let a_is_junction = junction_nodes.contains(&e.node_a);
        let b_is_junction = junction_nodes.contains(&e.node_b);
        if a_is_junction && b_is_junction {
            residual_edges.push(eidx); // Bridging → stays in R-node
        } else if a_is_junction || b_is_junction {
            // Pendant — could be extracted as a child
            // For now, keep in residual (proper pendant extraction
            // would BFS from the non-junction end)
            residual_edges.push(eidx);
        } else {
            residual_edges.push(eidx);
        }
    }

    SpqrNode::R {
        edge_indices: residual_edges,
        boundary_nodes: junction_nodes.into_iter().collect(),
        children: pendant_children,
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SP subtree classification
// ═══════════════════════════════════════════════════════════════════════════

/// Component-level classification of an SP subtree.
///
/// Determined by walking Q leaves and checking `Component::is_passive()`
/// and `Component::is_nonlinear()`. Drives the choice of runtime strategy.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum SpClassification {
    /// All edges are passive → pure WDF tree (or IIR for rigid).
    AllPassive,
    /// Exactly one nonlinear edge → WDF tree + scalar NR/Wright Omega root.
    SingleNl { nl_edge_idx: usize },
    /// Multiple NL elements or mixed active — needs MNA or further decomposition.
    Complex,
}

/// Classify an S/P/Q subtree by its component content.
///
/// Walks Q leaves, counts nonlinear edges. Does NOT look at topology
/// (that's already captured by the SpqrNode structure).
pub(super) fn classify_sp_subtree(node: &SpqrNode, graph: &CircuitGraph) -> SpClassification {
    let mut nl_edges: Vec<usize> = Vec::new();
    collect_nl_edges(node, graph, &mut nl_edges);
    match nl_edges.len() {
        0 => SpClassification::AllPassive,
        1 => SpClassification::SingleNl { nl_edge_idx: nl_edges[0] },
        _ => SpClassification::Complex,
    }
}

fn collect_nl_edges(node: &SpqrNode, graph: &CircuitGraph, nl_edges: &mut Vec<usize>) {
    match node {
        SpqrNode::Q { edge_idx, .. } => {
            let e = &graph.edges[*edge_idx];
            let comp = &graph.components[e.comp_idx];
            if !comp.kind.is_passive() {
                nl_edges.push(*edge_idx);
            }
        }
        SpqrNode::S { children, .. } | SpqrNode::P { children, .. } => {
            for child in children {
                collect_nl_edges(child, graph, nl_edges);
            }
        }
        SpqrNode::R { edge_indices, children, .. } => {
            for &eidx in edge_indices {
                let e = &graph.edges[eidx];
                let comp = &graph.components[e.comp_idx];
                if !comp.kind.is_passive() {
                    nl_edges.push(eidx);
                }
            }
            for child in children {
                collect_nl_edges(child, graph, nl_edges);
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SPQR → DynNode (WDF tree) conversion
// ═══════════════════════════════════════════════════════════════════════════

use super::dyn_node::DynNode;

/// Fold a list of DynNodes into a binary tree using the given constructor.
///
/// Shared by series and parallel tree building. Folds right:
/// `fold([a, b, c], Series) → Series(a, Series(b, c))`
fn fold_binary(mut nodes: Vec<DynNode>, ctor: fn(Box<DynNode>, Box<DynNode>) -> DynNode) -> Option<DynNode> {
    match nodes.len() {
        0 => None,
        1 => Some(nodes.remove(0)),
        _ => {
            let mut tree = nodes.pop().unwrap();
            while let Some(left) = nodes.pop() {
                tree = ctor(Box::new(left), Box::new(tree));
            }
            Some(tree)
        }
    }
}

/// Convert an S/P/Q subtree to a WDF DynNode (all-passive).
///
/// Returns `None` if any edge is non-passive (NL element, op-amp, etc.).
pub(super) fn spqr_to_dyn_node(
    node: &SpqrNode,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Option<DynNode> {
    match node {
        SpqrNode::Q { edge_idx, .. } => {
            let e = &graph.edges[*edge_idx];
            let comp = &graph.components[e.comp_idx];
            comp.kind.make_leaf(&comp.id, sample_rate)
        }
        SpqrNode::S { children, .. } => {
            let nodes: Option<Vec<DynNode>> = children
                .iter()
                .map(|c| spqr_to_dyn_node(c, graph, sample_rate))
                .collect();
            fold_binary(nodes?, DynNode::Series)
        }
        SpqrNode::P { children, .. } => {
            let nodes: Option<Vec<DynNode>> = children
                .iter()
                .map(|c| spqr_to_dyn_node(c, graph, sample_rate))
                .collect();
            fold_binary(nodes?, DynNode::Parallel)
        }
        SpqrNode::R { .. } => None,
    }
}

/// Convert an S/P/Q subtree to a WDF DynNode, skipping one NL edge.
///
/// Builds the passive WDF tree for Case 2 (SingleNl). The excluded
/// NL edge becomes the root element solved by NR/Wright Omega.
pub(super) fn spqr_to_passive_dyn_node(
    node: &SpqrNode,
    graph: &CircuitGraph,
    sample_rate: f64,
    exclude_edge: usize,
) -> Option<DynNode> {
    match node {
        SpqrNode::Q { edge_idx, .. } => {
            if *edge_idx == exclude_edge {
                return None; // This is the NL root — skip it
            }
            let e = &graph.edges[*edge_idx];
            let comp = &graph.components[e.comp_idx];
            comp.kind.make_leaf(&comp.id, sample_rate)
        }
        SpqrNode::S { children, .. } => {
            let nodes: Vec<DynNode> = children
                .iter()
                .filter_map(|c| spqr_to_passive_dyn_node(c, graph, sample_rate, exclude_edge))
                .collect();
            fold_binary(nodes, DynNode::Series)
        }
        SpqrNode::P { children, .. } => {
            let nodes: Vec<DynNode> = children
                .iter()
                .filter_map(|c| spqr_to_passive_dyn_node(c, graph, sample_rate, exclude_edge))
                .collect();
            fold_binary(nodes, DynNode::Parallel)
        }
        SpqrNode::R { .. } => None,
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SPQR → stages
// ═══════════════════════════════════════════════════════════════════════════

/// A stage produced by SPQR tree conversion.
///
/// Three cases, matching topology × component classification:
/// - **PassiveWdf**: SP-reducible, all passive → pure WDF tree
/// - **NlWdf**: SP-reducible, single NL root → WDF tree + scalar NR solve
/// - **Rigid**: non-SP topology → MNA scattering (build layer decides IIR vs NR)
pub(super) enum SpqrStage {
    /// Case 1: all-passive SP tree → pure WDF (or IIR candidate).
    PassiveWdf {
        tree: DynNode,
        edge_indices: Vec<usize>,
        order: usize,
    },
    /// Case 2: SP tree with single NL root → WDF + scalar NR/Wright Omega.
    NlWdf {
        /// Passive WDF tree (NL edge excluded).
        tree: DynNode,
        /// The nonlinear edge index (becomes RootKind).
        nl_edge_idx: usize,
        /// All edge indices (passive + NL).
        edge_indices: Vec<usize>,
        order: usize,
    },
    /// Case 3: rigid subgraph → MNA scattering matrix.
    /// Build layer checks Component::is_passive() to pick IIR (3a) vs NR (3b).
    Rigid {
        edge_indices: Vec<usize>,
        boundary_nodes: Vec<NodeId>,
        /// Pendant WDF subtrees (SP-reduced children of the R-node).
        pendant_trees: Vec<(DynNode, NodeId)>,
        order: usize,
    },
}

impl std::fmt::Debug for SpqrStage {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SpqrStage::PassiveWdf { edge_indices, order, .. } => f
                .debug_struct("PassiveWdf")
                .field("edge_indices", edge_indices)
                .field("order", order)
                .finish(),
            SpqrStage::NlWdf { nl_edge_idx, edge_indices, order, .. } => f
                .debug_struct("NlWdf")
                .field("nl_edge_idx", nl_edge_idx)
                .field("edge_indices", edge_indices)
                .field("order", order)
                .finish(),
            SpqrStage::Rigid { edge_indices, boundary_nodes, pendant_trees, order } => f
                .debug_struct("Rigid")
                .field("edge_indices", edge_indices)
                .field("boundary_nodes", boundary_nodes)
                .field("pendant_count", &pendant_trees.len())
                .field("order", order)
                .finish(),
        }
    }
}

/// Convert an SPQR tree into a list of stages in signal-flow order.
///
/// Walks the SPQR tree depth-first. Classification at each S/P node
/// determines the stage type. Traversal order IS signal routing.
pub(super) fn spqr_to_stages(
    root: &SpqrNode,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Vec<SpqrStage> {
    let mut stages = Vec::new();
    let mut order = 0;
    collect_stages(root, graph, sample_rate, &mut stages, &mut order);
    stages
}

fn collect_stages(
    node: &SpqrNode,
    graph: &CircuitGraph,
    sample_rate: f64,
    stages: &mut Vec<SpqrStage>,
    order: &mut usize,
) {
    match node {
        SpqrNode::Q { edge_idx, .. } => {
            let e = &graph.edges[*edge_idx];
            let comp = &graph.components[e.comp_idx];
            if let Some(tree) = comp.kind.make_leaf(&comp.id, sample_rate) {
                stages.push(SpqrStage::PassiveWdf {
                    tree,
                    edge_indices: vec![*edge_idx],
                    order: *order,
                });
                *order += 1;
            }
            // Bare NL Q-nodes are absorbed by their parent stage
        }
        SpqrNode::S { children, .. } | SpqrNode::P { children, .. } => {
            match classify_sp_subtree(node, graph) {
                SpClassification::AllPassive => {
                    if let Some(tree) = spqr_to_dyn_node(node, graph, sample_rate) {
                        stages.push(SpqrStage::PassiveWdf {
                            tree,
                            edge_indices: node.all_edge_indices(),
                            order: *order,
                        });
                        *order += 1;
                    }
                }
                SpClassification::SingleNl { nl_edge_idx } => {
                    if let Some(tree) = spqr_to_passive_dyn_node(node, graph, sample_rate, nl_edge_idx) {
                        stages.push(SpqrStage::NlWdf {
                            tree,
                            nl_edge_idx,
                            edge_indices: node.all_edge_indices(),
                            order: *order,
                        });
                        *order += 1;
                    }
                }
                SpClassification::Complex => {
                    // Multiple NL or mixed → recurse into children
                    for child in children {
                        collect_stages(child, graph, sample_rate, stages, order);
                    }
                }
            }
        }
        SpqrNode::R { edge_indices, boundary_nodes, children, .. } => {
            let mut pendant_trees: Vec<(DynNode, NodeId)> = Vec::new();
            let mut remaining_edges = edge_indices.clone();

            for child in children {
                if let Some(tree) = spqr_to_dyn_node(child, graph, sample_rate) {
                    let (attach, _) = child.endpoints();
                    pendant_trees.push((tree, attach));
                } else {
                    // Non-passive child of R-node → recurse
                    collect_stages(child, graph, sample_rate, stages, order);
                    remaining_edges.extend(child.all_edge_indices());
                }
            }

            stages.push(SpqrStage::Rigid {
                edge_indices: remaining_edges,
                boundary_nodes: boundary_nodes.clone(),
                pendant_trees,
                order: *order,
            });
            *order += 1;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: build a CircuitGraph from a .pedal source string.
    /// Returns (graph, passive_edge_indices).
    fn make_graph(pedal_src: &str) -> (CircuitGraph, Vec<usize>) {
        let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse failed");
        let graph = CircuitGraph::from_pedal(&pedal);
        let active_set: std::collections::HashSet<usize> =
            graph.active_edge_indices.iter().copied().collect();
        let passive_edges: Vec<usize> = (0..graph.edges.len())
            .filter(|i| !active_set.contains(i))
            .filter(|&i| graph.components[graph.edges[i].comp_idx].kind.is_passive())
            .collect();
        (graph, passive_edges)
    }

    /// Helper: build a CircuitGraph and return ALL non-active-IC edges
    /// (passives + nonlinear elements like diodes). Used for NlWdf tests.
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

    #[test]
    fn spqr_single_resistor() {
        let (graph, edges) = make_graph(r#"
            pedal "test" { supply 9V
                components { R1: resistor(10k) }
                nets { in -> R1.a  R1.b -> out }
                controls {}
            }"#);
        let result = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
        assert!(result.is_leaf(), "Single resistor should be Q node");
        assert_eq!(result.edge_count(), 1);
    }

    #[test]
    fn spqr_series_two_resistors() {
        let (graph, edges) = make_graph(r#"
            pedal "test" { supply 9V
                components { R1: resistor(10k)  R2: resistor(10k) }
                nets { in -> R1.a  R1.b -> R2.a  R2.b -> out }
                controls {}
            }"#);
        let result = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
        match &result {
            SpqrNode::S { children, .. } => {
                assert!(children.len() >= 2, "Series should have ≥2 children, got {}", children.len());
            }
            other => panic!("Expected S node for series resistors, got {:?}", std::mem::discriminant(other)),
        }
        assert_eq!(result.edge_count(), 2);
    }

    #[test]
    fn spqr_parallel_two_resistors() {
        let (graph, edges) = make_graph(r#"
            pedal "test" { supply 9V
                components { R1: resistor(10k)  R2: resistor(20k) }
                nets { in -> R1.a, R2.a  R1.b -> out  R2.b -> out }
                controls {}
            }"#);
        let result = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
        match &result {
            SpqrNode::P { children, .. } => {
                assert_eq!(children.len(), 2, "Parallel should have 2 children");
            }
            other => panic!("Expected P node for parallel resistors, got {:?}", std::mem::discriminant(other)),
        }
        assert_eq!(result.edge_count(), 2);
    }

    #[test]
    fn spqr_wheatstone_bridge_is_rigid() {
        // Wheatstone bridge: 5 resistors, NOT series-parallel
        let (graph, edges) = make_graph(r#"
            pedal "test" { supply 9V
                components {
                    R1: resistor(10k)  R2: resistor(10k)
                    R3: resistor(10k)  R4: resistor(10k)
                    R5: resistor(10k)
                }
                nets {
                    in -> R1.a, R3.a
                    R1.b -> R2.a, R5.a
                    R3.b -> R4.a, R5.b
                    R2.b -> out  R4.b -> out
                }
                controls {}
            }"#);
        let result = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
        assert!(result.is_rigid(), "Wheatstone bridge should be R node, got {:?}", std::mem::discriminant(&result));
        assert_eq!(result.edge_count(), 5, "Must preserve all 5 edges");
    }

    #[test]
    fn spqr_rc_lowpass_is_series() {
        let (graph, edges) = make_graph(r#"
            pedal "test" { supply 9V
                components { R1: resistor(10k)  C1: cap(100n) }
                nets { in -> R1.a  R1.b -> C1.a  C1.b -> out }
                controls {}
            }"#);
        let result = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
        match &result {
            SpqrNode::S { children, .. } => {
                assert!(children.len() >= 2, "RC lowpass should be series with ≥2 children");
            }
            other => panic!("Expected S node for RC lowpass, got {:?}", std::mem::discriminant(other)),
        }
    }

    #[test]
    fn spqr_preserves_all_edges() {
        let (graph, edges) = make_graph(r#"
            pedal "test" { supply 9V
                components {
                    R1: resistor(10k)  R2: resistor(10k)
                    R3: resistor(10k)  R4: resistor(10k)
                    R5: resistor(10k)
                }
                nets {
                    in -> R1.a, R3.a
                    R1.b -> R2.a, R5.a
                    R3.b -> R4.a, R5.b
                    R2.b -> out  R4.b -> out
                }
                controls {}
            }"#);
        let n_passive = edges.len();
        let result = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
        assert_eq!(
            result.edge_count(),
            n_passive,
            "Decomposition must preserve all {} passive edges",
            n_passive
        );
    }

    // ── Phase 2: SPQR → stages tests ────────────────────────────────

    #[test]
    fn spqr_to_stages_passive_rc_single_wdf() {
        let (graph, edges) = make_graph(r#"
            pedal "test" { supply 9V
                components { R1: resistor(10k)  C1: cap(100n) }
                nets { in -> R1.a  R1.b -> C1.a  C1.b -> out }
                controls {}
            }"#);
        let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
        let stages = spqr_to_stages(&tree, &graph, 48000.0);
        assert_eq!(stages.len(), 1, "Passive RC should produce 1 stage");
        assert!(
            matches!(&stages[0], SpqrStage::PassiveWdf { edge_indices, .. } if edge_indices.len() == 2),
            "Passive RC should be PassiveWdf with 2 edges, got {:?}", stages[0]
        );
    }

    #[test]
    fn spqr_to_stages_parallel_rc_single_wdf() {
        let (graph, edges) = make_graph(r#"
            pedal "test" { supply 9V
                components { R1: resistor(10k)  R2: resistor(20k) }
                nets { in -> R1.a, R2.a  R1.b -> out  R2.b -> out }
                controls {}
            }"#);
        let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
        let stages = spqr_to_stages(&tree, &graph, 48000.0);
        assert_eq!(stages.len(), 1, "Parallel R should produce 1 stage");
        assert!(
            matches!(&stages[0], SpqrStage::PassiveWdf { .. }),
            "Parallel R should be PassiveWdf, got {:?}", stages[0]
        );
    }

    #[test]
    fn spqr_to_stages_bridge_becomes_rigid() {
        let (graph, edges) = make_graph(r#"
            pedal "test" { supply 9V
                components {
                    R1: resistor(10k)  R2: resistor(10k)
                    R3: resistor(10k)  R4: resistor(10k)
                    R5: resistor(10k)
                }
                nets {
                    in -> R1.a, R3.a
                    R1.b -> R2.a, R5.a
                    R3.b -> R4.a, R5.b
                    R2.b -> out  R4.b -> out
                }
                controls {}
            }"#);
        let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
        let stages = spqr_to_stages(&tree, &graph, 48000.0);

        let has_rigid = stages.iter().any(|s| matches!(s, SpqrStage::Rigid { .. }));
        assert!(has_rigid, "Wheatstone bridge should produce a Rigid stage");
    }

    #[test]
    fn spqr_to_stages_ordering_matches_traversal() {
        let (graph, edges) = make_graph(r#"
            pedal "test" { supply 9V
                components { R1: resistor(10k)  R2: resistor(10k)  R3: resistor(10k) }
                nets { in -> R1.a  R1.b -> R2.a  R2.b -> R3.a  R3.b -> out }
                controls {}
            }"#);
        let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
        let stages = spqr_to_stages(&tree, &graph, 48000.0);

        let get_order = |s: &SpqrStage| match s {
            SpqrStage::PassiveWdf { order, .. }
            | SpqrStage::NlWdf { order, .. }
            | SpqrStage::Rigid { order, .. } => *order,
        };
        for i in 1..stages.len() {
            assert!(
                get_order(&stages[i]) > get_order(&stages[i - 1]),
                "Stage ordering should be monotonic: stage[{}]={} <= stage[{}]={}",
                i - 1, get_order(&stages[i - 1]), i, get_order(&stages[i])
            );
        }
    }

    // ── Phase 2.5: classifier + NlWdf tests ────────────────────────

    #[test]
    fn classify_all_passive_rc() {
        let (graph, edges) = make_graph(r#"
            pedal "test" { supply 9V
                components { R1: resistor(10k)  C1: cap(100n) }
                nets { in -> R1.a  R1.b -> C1.a  C1.b -> out }
                controls {}
            }"#);
        let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
        assert_eq!(classify_sp_subtree(&tree, &graph), SpClassification::AllPassive);
    }

    #[test]
    fn classify_single_nl_diode_clipper() {
        // R in series with diode pair to ground — classic hard clipper
        let (graph, edges) = make_graph_all_edges(r#"
            pedal "test" { supply 9V
                components { R1: resistor(10k)  D1: diode(silicon) }
                nets { in -> R1.a  R1.b -> D1.a  D1.b -> out }
                controls {}
            }"#);
        let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
        match classify_sp_subtree(&tree, &graph) {
            SpClassification::SingleNl { .. } => {} // correct
            other => panic!("Diode clipper should be SingleNl, got {:?}", other),
        }
    }

    #[test]
    fn classify_complex_two_diodes() {
        let (graph, edges) = make_graph_all_edges(r#"
            pedal "test" { supply 9V
                components { D1: diode(silicon)  D2: diode(silicon) }
                nets { in -> D1.a  D1.b -> D2.a  D2.b -> out }
                controls {}
            }"#);
        let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
        assert_eq!(classify_sp_subtree(&tree, &graph), SpClassification::Complex);
    }

    #[test]
    fn spqr_to_stages_diode_clipper_is_nl_wdf() {
        let (graph, edges) = make_graph_all_edges(r#"
            pedal "test" { supply 9V
                components { R1: resistor(10k)  C1: cap(100n)  D1: diode(silicon) }
                nets { in -> R1.a  R1.b -> C1.a  C1.b -> D1.a  D1.b -> out }
                controls {}
            }"#);
        let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
        let stages = spqr_to_stages(&tree, &graph, 48000.0);
        assert_eq!(stages.len(), 1, "Diode clipper should produce 1 stage");
        match &stages[0] {
            SpqrStage::NlWdf { nl_edge_idx, edge_indices, tree, .. } => {
                assert_eq!(edge_indices.len(), 3, "Should have R + C + D");
                // The passive tree should have port_resistance for R + C
                let rp = tree.port_resistance();
                assert!(rp > 0.0, "Passive tree should have valid port resistance, got {rp}");
                // The NL edge should point to the diode
                let nl_edge = &graph.edges[*nl_edge_idx];
                let nl_comp = &graph.components[nl_edge.comp_idx];
                assert!(!nl_comp.kind.is_passive(), "NL edge should be non-passive");
            }
            other => panic!("Diode clipper should be NlWdf, got {:?}", other),
        }
    }

    #[test]
    fn spqr_to_stages_nl_wdf_passive_tree_excludes_diode() {
        let (graph, edges) = make_graph_all_edges(r#"
            pedal "test" { supply 9V
                components { R1: resistor(4.7k)  D1: diode(silicon) }
                nets { in -> R1.a  R1.b -> D1.a  D1.b -> out }
                controls {}
            }"#);
        let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
        let stages = spqr_to_stages(&tree, &graph, 48000.0);
        match &stages[0] {
            SpqrStage::NlWdf { tree, .. } => {
                // Passive tree should only be the resistor (4.7k)
                let rp = tree.port_resistance();
                assert!(
                    (rp - 4700.0).abs() < 100.0,
                    "Passive tree should be just R1=4.7k, got Rp={rp:.0}"
                );
            }
            other => panic!("Expected NlWdf, got {:?}", other),
        }
    }

    // ── Phase 2 continued ───────────────────────────────────────────

    #[test]
    fn spqr_dyn_node_series_has_correct_structure() {
        let (graph, edges) = make_graph(r#"
            pedal "test" { supply 9V
                components { R1: resistor(10k)  R2: resistor(10k) }
                nets { in -> R1.a  R1.b -> R2.a  R2.b -> out }
                controls {}
            }"#);
        let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
        let dyn_node = spqr_to_dyn_node(&tree, &graph, 48000.0);
        assert!(dyn_node.is_some(), "Series resistors should produce a DynNode");
        // The DynNode should have port_resistance = R1 + R2 = 20k
        let node = dyn_node.unwrap();
        let rp = node.port_resistance();
        assert!(
            (rp - 20_000.0).abs() < 100.0,
            "Series R1+R2 port resistance should be ~20k, got {rp:.0}"
        );
    }
}
