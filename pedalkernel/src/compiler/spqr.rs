//! # SPQR Tree Decomposition for Circuit Graphs
//!
//! This module implements SPQR decomposition — the algorithm at the heart of
//! PedalKernel's compilation strategy. It transforms a flat circuit graph into
//! a hierarchical tree that directly determines the runtime solver for each
//! sub-circuit.
//!
//! # What is SPQR Decomposition?
//!
//! Every 2-connected graph has a unique decomposition into four types of nodes,
//! named after their topological structure:
//!
//! - **S (Series)**: A chain of components connected end-to-end through degree-2
//!   internal nodes. In WDF terms, this becomes a **series adaptor** — the wave
//!   variables combine additively. Example: an RC lowpass (resistor in series
//!   with a capacitor to ground).
//!
//! - **P (Parallel)**: Multiple components sharing the same pair of endpoints.
//!   In WDF terms, this becomes a **parallel adaptor** — the wave variables
//!   combine via admittance weighting. Example: a resistor and capacitor
//!   both connected between the same two nodes.
//!
//! - **Q (Leaf)**: A single edge — one component between two nodes. This becomes
//!   a **WDF leaf element** (resistor, capacitor, inductor) or a nonlinear root
//!   (diode, transistor junction).
//!
//! - **R (Rigid)**: A triconnected subgraph that cannot be decomposed into series
//!   or parallel combinations. These require a **matrix-based solver** — either
//!   MNA scattering (for general circuits), IIR biquads (for all-passive cases
//!   like tone stacks), or Harold Black's feedback formula (for op-amp stages).
//!
//! # Why SPQR Matters for WDF
//!
//! Wave Digital Filters require a binary tree of series/parallel adaptors.
//! Arbitrary circuits do not naturally form binary trees — a Wheatstone bridge,
//! for instance, is triconnected and has no series/parallel decomposition.
//!
//! SPQR decomposition solves this by automatically identifying the maximal
//! series/parallel structure in any circuit. The S/P portions get the cheap
//! O(1)-per-sample WDF treatment; only the irreducible R portions pay the
//! cost of matrix solvers. For most guitar pedal circuits, the vast majority
//! of components fall into S/P subtrees.
//!
//! # The Signal Flow Pipeline
//!
//! The full compilation flow through this module:
//!
//! ```text
//! CircuitGraph ──> signal_flow::find_flow_groups()
//!                       │
//!                       v
//!               flow groups (split at VoltageSource nodes)
//!                       │
//!                       v
//!               spqr_decompose() ──> SpqrNode tree
//!                       │
//!                       v
//!               spqr_to_stages() ──> Vec<SpqrStage>
//!                       │
//!                       v
//!               build pass ──> runtime processors
//! ```
//!
//! ## Stage Classification
//!
//! Each SPQR subtree is classified by [`classify_sp_subtree`] based on the
//! [`EdgeKind`](super::component::EdgeKind) of its leaf components:
//!
//! - **AllPassive** (all edges are [`Linear`](super::component::EdgeKind::Linear) or
//!   [`Reactive`](super::component::EdgeKind::Reactive)): emitted as a
//!   [`SpqrStage::PassiveWdf`] — a pure WDF tree with O(1) per-sample cost.
//!
//! - **SingleNl** (exactly one [`Nonlinear`](super::component::EdgeKind::Nonlinear)
//!   edge): emitted as a [`SpqrStage::NlWdf`] — a WDF tree with the nonlinear
//!   element at the root, solved by scalar Newton-Raphson or Wright Omega.
//!
//! - **Vcvs** (contains a [`Vcvs`](super::component::EdgeKind::Vcvs) edge, i.e., an
//!   op-amp): the subtree is recursed into its children so the VCVS does not
//!   infect unrelated passive stages.
//!
//! - **Complex** (multiple nonlinear elements): recursed into children for
//!   independent classification.
//!
//! R-nodes always become [`SpqrStage::Rigid`]. The build layer then inspects
//! the component content to choose the solver:
//! - All passive → IIR biquad (O(1), e.g., Big Muff tone stack)
//! - Op-amp + passive feedback → BlackFeedback (O(1), Harold Black's formula)
//! - Reactive + active → StateSpace (O(N), state-space MNA integration)
//! - Multiple nonlinear → MultiNl (O(N^2), multi-dimensional Newton-Raphson)
//!
//! ## Pendant Extraction
//!
//! Before series/parallel reduction, [`extract_pendants`] removes dead-end edges
//! from the subgraph. A pendant is an edge where one endpoint has degree 1 —
//! it connects to only one junction node. Ground-terminated capacitors are the
//! most common example: the cap connects a signal node to GND, and within the
//! local subgraph GND has degree 1.
//!
//! Removing pendants reduces junction degrees, often enabling series detection
//! that would otherwise be blocked. For example, a T-junction with a cap to
//! ground: after extracting the pendant cap, the junction becomes degree 2,
//! revealing a series chain.
//!
//! ## Output Impedance and Stage Splitting
//!
//! Before SPQR decomposition begins, the signal flow analysis pass splits the
//! circuit at nodes where a component declares
//! [`OutputImpedance::VoltageSource`](super::component::OutputImpedance::VoltageSource).
//! This creates independent flow groups that are decomposed separately. The
//! physical justification: a voltage source output decouples upstream and
//! downstream impedances, so the sub-circuits on each side can be solved
//! independently without loss of accuracy.
//!
//! # Converting to Runtime Processors
//!
//! - [`spqr_to_dyn_node`] folds an all-passive S/P/Q subtree into a
//!   [`DynNode`](super::dyn_node::DynNode) WDF binary tree.
//! - [`spqr_to_passive_dyn_node`] does the same but excludes one nonlinear edge,
//!   which becomes the NR/Wright Omega root element.
//! - [`spqr_to_stages`] walks the full SPQR tree depth-first, classifying each
//!   subtree and emitting [`SpqrStage`] values in signal-flow order. The
//!   traversal order *is* the signal routing — no topological sort is needed.

use super::component::EdgeKind;
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
            SpqrNode::R {
                edge_indices,
                children,
                ..
            } => edge_indices.len() + children.iter().map(|c| c.edge_count()).sum::<usize>(),
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
            SpqrNode::R {
                edge_indices,
                children,
                ..
            } => {
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
    // Guard: detect infinite recursion from non-reducing decompositions
    if edge_indices.len() > 100 {
        eprintln!(
            "  SPQR: large edge set ({} edges), terminals={terminals:?}",
            edge_indices.len()
        );
    }
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
    let mut adj: std::collections::BTreeMap<NodeId, Vec<(usize, NodeId)>> =
        std::collections::BTreeMap::new();
    for &eidx in edge_indices {
        let e = &graph.edges[eidx];
        adj.entry(e.node_a).or_default().push((eidx, e.node_b));
        adj.entry(e.node_b).or_default().push((eidx, e.node_a));
    }

    let terminal_set: std::collections::HashSet<NodeId> = terminals.iter().copied().collect();

    // Extract pendant edges (dead-ends) before SP reduction.
    // Pendants reduce junction degree, enabling series detection.
    // E.g., T-junction: R1→junction→(C1→gnd, R2→out)
    // → extract C1 pendant → junction becomes degree 2 → series(R1, R2).
    let (remaining_edges, pendant_children) = extract_pendants(edge_indices, &terminal_set, graph);

    if remaining_edges.len() < edge_indices.len() {
        // Pendants found — decompose the core, then insert each pendant
        // as a parallel element at its junction node in the core tree.
        if remaining_edges.is_empty() {
            // All edges are pendants — group by junction, parallel at same junction
            if pendant_children.len() == 1 {
                return pendant_children.into_iter().next().unwrap().0;
            }
            // Multiple pendants: just wrap as S (they connect different junctions)
            let children: Vec<SpqrNode> = pendant_children.into_iter().map(|(n, _)| n).collect();
            return SpqrNode::S {
                children,
                cut_vertices: (
                    terminals.first().copied().unwrap_or(0),
                    terminals.last().copied().unwrap_or(0),
                ),
            };
        }
        let mut tree = spqr_decompose(&remaining_edges, terminals, graph, gnd_node);
        for (pendant, junction) in pendant_children {
            tree = insert_pendant_at_junction(tree, pendant, junction, graph);
        }
        return tree;
    }
    // No pendants — continue with SP reduction below
    // (adj was already built above for the full set, still valid if no pendants)

    // Check for parallel edges: multiple edges between the same pair
    // BTreeMap: deterministic iteration order prevents non-deterministic
    // SPQR tree structure, which would break pot binding stage indices.
    let mut edge_pairs: std::collections::BTreeMap<(NodeId, NodeId), Vec<usize>> =
        std::collections::BTreeMap::new();
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
            // Walk backward to find chain start (terminal or branch).
            // Use a visited set to prevent infinite loops in degree-2 cycles
            // where no terminal breaks the chain.
            let mut cur = start_node;
            let mut prev = NodeId::MAX;
            let mut visited_back: std::collections::HashSet<NodeId> =
                std::collections::HashSet::new();
            visited_back.insert(cur);
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
                    if visited_back.contains(&next) {
                        // Cycle detected — use current node as chain start
                        chain_start = cur;
                        break;
                    }
                    visited_back.insert(next);
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

            // Recurse on remaining edges — must be strictly fewer
            assert!(
                remaining.len() < edge_indices.len(),
                "SPQR series: remaining ({}) not smaller than input ({})",
                remaining.len(),
                edge_indices.len()
            );
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

            // Mix of parallel and other — recurse on remaining (strictly smaller)
            let mut r_children = vec![SpqrNode::P {
                children: parallel_children,
                endpoints: (*a, *b),
            }];
            assert!(
                remaining.len() < edge_indices.len(),
                "SPQR parallel: remaining ({}) not smaller than input ({})",
                remaining.len(),
                edge_indices.len()
            );
            r_children.push(spqr_decompose(&remaining, terminals, graph, gnd_node));
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

    // Sort boundary_nodes for deterministic ordering. The first two items are
    // used as the R-node's `endpoints()`, which drives SPQR series-chain
    // assembly. Non-deterministic HashSet iteration produced different endpoint
    // pairs across runs, causing different circuit topologies per compilation.
    // (Precedent: commit 0449cd6a fixed the same class of bug in blockwise.rs
    // and signal_flow.rs.)
    let mut boundary_nodes: Vec<NodeId> = junction_nodes.into_iter().collect();
    boundary_nodes.sort_unstable();
    SpqrNode::R {
        edge_indices: residual_edges,
        boundary_nodes,
        children: pendant_children,
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Pendant extraction
// ═══════════════════════════════════════════════════════════════════════════

/// Extract pendant edges (dead-ends) from a subgraph.
///
/// A pendant is an edge where one endpoint has degree 1 in the subgraph
/// (only connects to one junction node). Ground-terminated caps are
/// pendants: GND has degree 1 within the signal subgraph.
///
/// Iteratively removes pendants until no more exist, reducing junction
/// degrees so series detection can proceed.
///
/// Returns `(remaining_edges, pendant_q_nodes)`.
fn extract_pendants(
    edge_indices: &[usize],
    terminal_set: &std::collections::HashSet<NodeId>,
    graph: &CircuitGraph,
) -> (Vec<usize>, Vec<(SpqrNode, NodeId)>) {
    let mut remaining: Vec<usize> = edge_indices.to_vec();
    let mut pendants: Vec<(SpqrNode, NodeId)> = Vec::new();

    // Rail nodes are always considered pendant endpoints — multiple edges
    // connecting to GND/VCC don't create a signal path between them.
    // Excluding them from degree count ensures each ground-terminated
    // edge is recognized as a pendant independently.
    let rail_nodes: std::collections::HashSet<NodeId> = {
        let mut r = std::collections::HashSet::new();
        r.insert(graph.gnd_node);
        r.insert(graph.vcc_node);
        r.extend(&graph.supply_nodes);
        r.extend(&graph.ac_ground_nodes);
        r
    };

    loop {
        // Build degree map — exclude rail nodes (they're always "dead ends")
        let mut degree: std::collections::BTreeMap<NodeId, usize> =
            std::collections::BTreeMap::new();
        for &eidx in &remaining {
            let e = &graph.edges[eidx];
            if !rail_nodes.contains(&e.node_a) {
                *degree.entry(e.node_a).or_default() += 1;
            }
            if !rail_nodes.contains(&e.node_b) {
                *degree.entry(e.node_b).or_default() += 1;
            }
        }

        // Find edges with a degree-1 non-terminal endpoint, OR
        // edges where one endpoint is a rail node (always pendant).
        let mut found: Vec<usize> = Vec::new();
        for &eidx in &remaining {
            let e = &graph.edges[eidx];
            let a_is_rail = rail_nodes.contains(&e.node_a);
            let b_is_rail = rail_nodes.contains(&e.node_b);
            let a_pendant = (a_is_rail && !terminal_set.contains(&e.node_a))
                || (degree.get(&e.node_a).copied().unwrap_or(0) == 1
                    && !terminal_set.contains(&e.node_a));
            let b_pendant = (b_is_rail && !terminal_set.contains(&e.node_b))
                || (degree.get(&e.node_b).copied().unwrap_or(0) == 1
                    && !terminal_set.contains(&e.node_b));
            if a_pendant || b_pendant {
                found.push(eidx);
            }
        }

        if found.is_empty() {
            break;
        }

        let found_set: std::collections::HashSet<usize> = found.iter().copied().collect();
        for &eidx in &found {
            let e = &graph.edges[eidx];
            let a_is_rail = rail_nodes.contains(&e.node_a);
            let b_is_rail = rail_nodes.contains(&e.node_b);
            // Junction is the non-rail, non-degree-1 endpoint (the node
            // that connects to the rest of the circuit).
            let junction = if a_is_rail {
                e.node_b
            } else if b_is_rail {
                e.node_a
            } else if degree.get(&e.node_a).copied().unwrap_or(0) == 1 {
                e.node_b
            } else {
                e.node_a
            };
            pendants.push((
                SpqrNode::Q {
                    edge_idx: eidx,
                    endpoints: (e.node_a, e.node_b),
                },
                junction,
            ));
        }
        remaining.retain(|e| !found_set.contains(e));
    }

    (remaining, pendants)
}

/// Insert a pendant into an SPQR tree as a parallel element at its junction.
///
/// The pendant is a ground shunt — it branches off an internal node of the
/// core tree. This function finds the S-node child where `junction` is an
/// endpoint and wraps that child + pendant in a P-node.
fn insert_pendant_at_junction(
    core: SpqrNode,
    pendant: SpqrNode,
    junction: NodeId,
    graph: &CircuitGraph,
) -> SpqrNode {
    match core {
        SpqrNode::S {
            children,
            cut_vertices,
        } => {
            // In series chain [A→J, J→B, B→C], prefer the child where
            // junction is its "a" endpoint (J→B) — the downstream child.
            // Prefer passive children over NL children to avoid wrapping
            // NL edges in pendant P-nodes (which confuses find_nl_blocks).
            let is_passive_child = |c: &SpqrNode| -> bool {
                match c {
                    SpqrNode::Q { edge_idx, .. } => !matches!(
                        graph.effective_edge_kind(*edge_idx),
                        super::component::EdgeKind::Nonlinear | super::component::EdgeKind::Vcvs
                    ),
                    _ => true, // non-Q children (nested S/P) are fine to wrap
                }
            };

            // First: passive child with a == junction
            let target = children
                .iter()
                .position(|c| c.endpoints().0 == junction && is_passive_child(c))
                // Then: any child with a == junction
                .or_else(|| children.iter().position(|c| c.endpoints().0 == junction))
                // Then: passive child with b == junction
                .or_else(|| {
                    children
                        .iter()
                        .rposition(|c| c.endpoints().1 == junction && is_passive_child(c))
                })
                // Last resort: any child with b == junction
                .or_else(|| children.iter().rposition(|c| c.endpoints().1 == junction));

            let mut new_children: Vec<SpqrNode> = Vec::with_capacity(children.len());
            for (i, child) in children.into_iter().enumerate() {
                if Some(i) == target {
                    let ep = child.endpoints();
                    new_children.push(SpqrNode::P {
                        children: vec![child, pendant.clone()],
                        endpoints: ep,
                    });
                } else {
                    new_children.push(child);
                }
            }

            // Last resort: junction not found, append as series sibling
            if target.is_none() {
                new_children.push(pendant);
            }

            SpqrNode::S {
                children: new_children,
                cut_vertices,
            }
        }

        SpqrNode::P {
            mut children,
            endpoints,
        } => {
            children.push(pendant);
            SpqrNode::P {
                children,
                endpoints,
            }
        }

        SpqrNode::Q {
            edge_idx,
            endpoints,
        } => SpqrNode::P {
            children: vec![
                SpqrNode::Q {
                    edge_idx,
                    endpoints,
                },
                pendant,
            ],
            endpoints,
        },

        SpqrNode::R {
            edge_indices,
            boundary_nodes,
            children,
        } => {
            let mut all_edges = edge_indices;
            all_edges.extend(pendant.all_edge_indices());
            SpqrNode::R {
                edge_indices: all_edges,
                boundary_nodes,
                children,
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SP subtree classification
// ═══════════════════════════════════════════════════════════════════════════

/// Component-level classification of an SP subtree.
///
/// Uses `EdgeKind` from the Component trait to determine runtime strategy.
/// Priority: VCVS > SingleNl > AllPassive.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum SpClassification {
    /// All edges are passive → pure WDF tree (or IIR for rigid).
    AllPassive,
    /// Exactly one nonlinear edge → WDF tree + scalar NR/Wright Omega root.
    SingleNl { nl_edge_idx: usize },
    /// Contains a VCVS (op-amp) → entire subtree needs MNA, emit as Rigid.
    Vcvs,
    /// Multiple NL elements, no VCVS → recurse into children.
    Complex,
}

/// Classify an S/P/Q subtree by its component content.
///
/// Walks Q leaves, checks EdgeKind via the graph. VCVS edges take
/// priority — any VCVS forces the whole subtree to Rigid.
pub(super) fn classify_sp_subtree(node: &SpqrNode, graph: &CircuitGraph) -> SpClassification {
    let mut nl_edges: Vec<usize> = Vec::new();
    let mut has_vcvs = false;
    collect_non_passive_edges(node, graph, &mut nl_edges, &mut has_vcvs);

    if has_vcvs {
        return SpClassification::Vcvs;
    }
    if nl_edges.is_empty() {
        return SpClassification::AllPassive;
    }

    // Count NL *components*, not NL *edges*. A BJT has 2 NL edges
    // (B-E and C-E) from the same comp_idx — that's one NL device,
    // not two. Counting edges would classify it as Complex, preventing
    // it from becoming a WDF root with BjtRoot.
    let mut nl_comp_indices: Vec<usize> = nl_edges
        .iter()
        .map(|&eidx| graph.edges[eidx].comp_idx)
        .collect();
    nl_comp_indices.sort_unstable();
    nl_comp_indices.dedup();

    if nl_comp_indices.len() == 1 {
        // Single NL component (possibly multi-edge like BJT).
        // Use the first NL edge as the representative for create_root().
        SpClassification::SingleNl {
            nl_edge_idx: nl_edges[0],
        }
    } else {
        SpClassification::Complex
    }
}

fn collect_non_passive_edges(
    node: &SpqrNode,
    graph: &CircuitGraph,
    nl_edges: &mut Vec<usize>,
    has_vcvs: &mut bool,
) {
    match node {
        SpqrNode::Q { edge_idx, .. } => {
            let edge_kind = graph.effective_edge_kind(*edge_idx);
            match edge_kind {
                EdgeKind::Vcvs => *has_vcvs = true,
                EdgeKind::Linear | EdgeKind::Reactive => {} // passive
                EdgeKind::Nonlinear | EdgeKind::Vccs | EdgeKind::Behavioral => {
                    nl_edges.push(*edge_idx);
                }
            }
        }
        SpqrNode::S { children, .. } | SpqrNode::P { children, .. } => {
            for child in children {
                collect_non_passive_edges(child, graph, nl_edges, has_vcvs);
            }
        }
        SpqrNode::R {
            edge_indices,
            children,
            ..
        } => {
            for &eidx in edge_indices {
                let edge_kind = graph.effective_edge_kind(eidx);
                match edge_kind {
                    EdgeKind::Vcvs => *has_vcvs = true,
                    EdgeKind::Linear | EdgeKind::Reactive => {}
                    EdgeKind::Nonlinear | EdgeKind::Vccs | EdgeKind::Behavioral => {
                        nl_edges.push(eidx);
                    }
                }
            }
            for child in children {
                collect_non_passive_edges(child, graph, nl_edges, has_vcvs);
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SPQR → DynNode (WDF tree) conversion
// ═══════════════════════════════════════════════════════════════════════════

use super::dyn_node::DynNode;
use super::wdf_leaf::WdfLeaf;

/// Fold a list of DynNodes into a binary tree using the given constructor.
///
/// Shared by series and parallel tree building. Folds right:
/// `fold([a, b, c], Series) → Series(a, Series(b, c))`
fn fold_binary(
    mut nodes: Vec<DynNode>,
    ctor: fn(Box<DynNode>, Box<DynNode>) -> DynNode,
) -> Option<DynNode> {
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
            let mut leaf = comp.kind.make_leaf(&comp.id, sample_rate)?;
            // For 3-terminal pots (two edges, same comp_idx): the a-w half
            // is the complement (uses 1-position), and the w-b half tracks
            // position. Use terminal identity rather than "touches ground"
            // because tone/blend pots often connect b through a cap.
            if comp.kind.is_pot() {
                let is_3term = graph
                    .edges
                    .iter()
                    .filter(|other| other.comp_idx == e.comp_idx)
                    .count()
                    > 1;
                if is_3term && pot_edge_is_aw_half(graph, &comp.id, e.node_a, e.node_b) {
                    if let DynNode::Leaf(ref mut l) = leaf {
                        l.set_complement();
                    }
                }
            }
            Some(leaf)
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

fn pot_edge_is_aw_half(graph: &CircuitGraph, comp_id: &str, a: NodeId, b: NodeId) -> bool {
    let Some(&w_node) = graph
        .node_names
        .get(&format!("{comp_id}.w"))
        .or_else(|| graph.node_names.get(&format!("{comp_id}.wiper")))
    else {
        return false;
    };
    let Some(&a_node) = graph.node_names.get(&format!("{comp_id}.a")) else {
        return false;
    };
    (a == w_node && b == a_node) || (a == a_node && b == w_node)
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
            SpqrStage::PassiveWdf {
                edge_indices,
                order,
                ..
            } => f
                .debug_struct("PassiveWdf")
                .field("edge_indices", edge_indices)
                .field("order", order)
                .finish(),
            SpqrStage::NlWdf {
                nl_edge_idx,
                edge_indices,
                order,
                ..
            } => f
                .debug_struct("NlWdf")
                .field("nl_edge_idx", nl_edge_idx)
                .field("edge_indices", edge_indices)
                .field("order", order)
                .finish(),
            SpqrStage::Rigid {
                edge_indices,
                boundary_nodes,
                pendant_trees,
                order,
            } => f
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

/// Distinct component IDs for a set of edge indices (diagnostics).
fn component_ids_for_edges(graph: &CircuitGraph, edge_indices: &[usize]) -> Vec<String> {
    let mut ids: Vec<String> = edge_indices
        .iter()
        .map(|&i| graph.components[graph.edges[i].comp_idx].id.clone())
        .collect();
    ids.sort();
    ids.dedup();
    ids
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
            } else if comp.kind.is_passive() {
                // A passive component with no WDF leaf silently vanishes from
                // the signal path — the symptom is a unity passthrough with
                // dead controls (audit gap B4/G7).
                tracing::warn!(
                    "SPQR: passive component '{}' ({}) produced no WDF leaf; \
                     dropping its stage — output may collapse to passthrough",
                    comp.id,
                    comp.kind.type_tag()
                );
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
                    } else {
                        // One missing leaf poisons the whole S/P subtree
                        // (Option-collect in spqr_to_dyn_node), silently
                        // dropping every component in the group. Ideally a
                        // compile error, but spqr_to_stages is infallible at
                        // its call sites, so warn loudly instead.
                        tracing::warn!(
                            "SPQR: all-passive stage could not be converted to \
                             a WDF tree (a component produced no leaf); \
                             dropping stage with components [{}] — output may \
                             collapse to passthrough",
                            component_ids_for_edges(graph, &node.all_edge_indices()).join(", ")
                        );
                    }
                }
                SpClassification::SingleNl { nl_edge_idx } => {
                    if let Some(tree) =
                        spqr_to_passive_dyn_node(node, graph, sample_rate, nl_edge_idx)
                    {
                        stages.push(SpqrStage::NlWdf {
                            tree,
                            nl_edge_idx,
                            edge_indices: node.all_edge_indices(),
                            order: *order,
                        });
                        *order += 1;
                    }
                }
                SpClassification::Vcvs => {
                    // VCVS present — recurse into children. Each child is
                    // classified independently: VCVS children → Rigid,
                    // passive children → PassiveWdf, NL children → NlWdf.
                    // This prevents the VCVS from infecting unrelated
                    // stages (diodes to GND, tone stacks, volume pots).
                    for child in children {
                        collect_stages(child, graph, sample_rate, stages, order);
                    }
                }
                SpClassification::Complex => {
                    // Multiple NL, no VCVS → recurse into children
                    for child in children {
                        collect_stages(child, graph, sample_rate, stages, order);
                    }
                }
            }
        }
        SpqrNode::R {
            edge_indices,
            boundary_nodes,
            children,
            ..
        } => {
            // R-nodes are rigid: everything inside goes into one MNA stage.
            // Passive SP children become pendant WDF trees (port optimization).
            // Non-passive children are absorbed into the R-node's edge list.
            let mut pendant_trees: Vec<(DynNode, NodeId)> = Vec::new();
            let mut all_edges = edge_indices.clone();

            for child in children {
                if let Some(tree) = spqr_to_dyn_node(child, graph, sample_rate) {
                    let (attach, _) = child.endpoints();
                    pendant_trees.push((tree, attach));
                } else {
                    // Non-passive child → absorb into R-node MNA
                    all_edges.extend(child.all_edge_indices());
                }
            }

            stages.push(SpqrStage::Rigid {
                edge_indices: all_edges,
                boundary_nodes: boundary_nodes.clone(),
                pendant_trees,
                order: *order,
            });
            *order += 1;
        }
    }
}
