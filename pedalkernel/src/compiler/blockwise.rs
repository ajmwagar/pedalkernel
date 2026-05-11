//! Blockwise decomposition of large nonlinear circuits.
//!
//! Detects when a monolithic multi-NL system can be split into smaller
//! chained blocks with sparse inter-block coupling. Uses the SPQR tree
//! structure directly:
//!
//! 1. Find P-nodes containing NL edges → those are the blocks
//! 2. Note each P-node's endpoints → block port nodes
//! 3. Assign sibling Q-leaves to blocks by which port nodes they touch
//! 4. Validate: ≥2 blocks, each has reactive state, coupling is NL-free

use super::component::EdgeKind;
use super::graph::{CircuitGraph, NodeId};
use super::spqr::{spqr_decompose, spqr_to_stages, SpqrNode};
use super::spqr_build::BuiltStage;
use std::collections::{HashMap, HashSet};

/// A block in the blockwise decomposition — one NL device + its local state.
#[derive(Debug)]
pub struct Block {
    /// NL edge indices (the nonlinear device junctions).
    pub nl_edges: Vec<usize>,
    /// Linear edge indices interior to this block.
    pub linear_edges: Vec<usize>,
    /// Reactive edge indices interior to this block.
    pub reactive_edges: Vec<usize>,
    /// Port nodes — where this block connects to the coupling network.
    pub port_nodes: Vec<NodeId>,
}

impl Block {
    /// All edge indices in this block (NL + linear + reactive).
    pub fn all_edges(&self) -> Vec<usize> {
        let mut edges = Vec::with_capacity(
            self.nl_edges.len() + self.linear_edges.len() + self.reactive_edges.len(),
        );
        edges.extend(&self.nl_edges);
        edges.extend(&self.linear_edges);
        edges.extend(&self.reactive_edges);
        edges
    }
}

/// Result of blockwise decomposition analysis.
#[derive(Debug)]
pub struct BlockwisePlan {
    /// The chained blocks, in signal-flow order.
    pub blocks: Vec<Block>,
    /// Coupling edges — the residual connecting blocks.
    pub coupling_edges: Vec<usize>,
    /// Port nodes shared between blocks and coupling network.
    pub port_nodes: Vec<NodeId>,
}

impl BlockwisePlan {
    pub fn num_blocks(&self) -> usize {
        self.blocks.len()
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Step 1: Find NL P-nodes in the SPQR tree
// ═══════════════════════════════════════════════════════════════════════════

/// An NL block found in the SPQR tree: a P-node whose children are
/// NL Q-leaves from the same component.
struct NlBlock {
    /// The NL edge indices (from Q-leaf children of the P-node).
    nl_edges: Vec<usize>,
    /// The P-node's endpoints — the block's port nodes.
    endpoints: (NodeId, NodeId),
    /// Component index of the NL device.
    comp_idx: usize,
}

/// Walk the SPQR tree and collect NL blocks.
///
/// Two patterns:
/// 1. **P-node**: children are parallel NL edges from the same component
///    (diode-connected BJT: base=collector → same endpoints)
/// 2. **S-node siblings**: Q-leaf NL edges from the same component that
///    are siblings in an S-node (CE BJT: B-E and C-E share emitter node
///    but have different other endpoints)
fn find_nl_blocks(node: &SpqrNode, graph: &CircuitGraph, blocks: &mut Vec<NlBlock>) {
    match node {
        SpqrNode::P { children, endpoints } => {
            // Pattern 1: P-node with all-NL children from same component
            let mut nl_edges = Vec::new();
            let mut comp_idx = None;
            let mut all_nl_same_comp = true;

            for child in children {
                if let SpqrNode::Q { edge_idx, .. } = child {
                    if graph.effective_edge_kind(*edge_idx) == EdgeKind::Nonlinear {
                        let ci = graph.edges[*edge_idx].comp_idx;
                        if let Some(existing) = comp_idx {
                            if existing != ci { all_nl_same_comp = false; }
                        } else {
                            comp_idx = Some(ci);
                        }
                        nl_edges.push(*edge_idx);
                    } else {
                        all_nl_same_comp = false;
                    }
                } else {
                    all_nl_same_comp = false;
                }
            }

            if all_nl_same_comp && !nl_edges.is_empty() {
                if let Some(ci) = comp_idx {
                    blocks.push(NlBlock {
                        nl_edges,
                        endpoints: *endpoints,
                        comp_idx: ci,
                    });
                    return;
                }
            }

            for child in children { find_nl_blocks(child, graph, blocks); }
        }

        SpqrNode::S { children, .. } | SpqrNode::R { children, .. } => {
            // Pattern 2: group NL Q-leaf siblings by comp_idx
            let mut comp_groups: HashMap<usize, Vec<usize>> = HashMap::new();
            for child in children {
                if let SpqrNode::Q { edge_idx, .. } = child {
                    if graph.effective_edge_kind(*edge_idx) == EdgeKind::Nonlinear {
                        let ci = graph.edges[*edge_idx].comp_idx;
                        comp_groups.entry(ci).or_default().push(*edge_idx);
                    }
                }
            }

            for (ci, nl_edges) in comp_groups {
                if nl_edges.is_empty() { continue; }
                // Compute endpoints: union of all nodes from this comp's NL edges
                let mut all_nodes: Vec<NodeId> = Vec::new();
                for &eidx in &nl_edges {
                    let e = &graph.edges[eidx];
                    if !all_nodes.contains(&e.node_a) { all_nodes.push(e.node_a); }
                    if !all_nodes.contains(&e.node_b) { all_nodes.push(e.node_b); }
                }
                // Endpoints: the two outermost nodes (or first two if >2)
                let ep = if all_nodes.len() >= 2 {
                    (all_nodes[0], all_nodes[1])
                } else if all_nodes.len() == 1 {
                    (all_nodes[0], all_nodes[0])
                } else {
                    continue;
                };
                // For 3-node BJTs (base, collector, emitter), use base and collector
                // as endpoints (emitter is the shared internal node)
                let ep = if all_nodes.len() == 3 {
                    // Find the node that appears in ALL NL edges (the shared one = emitter)
                    // The other two are the endpoints
                    let shared = all_nodes.iter().find(|&&n| {
                        nl_edges.iter().all(|&eidx| {
                            let e = &graph.edges[eidx];
                            e.node_a == n || e.node_b == n
                        })
                    });
                    if let Some(&emitter) = shared {
                        let others: Vec<NodeId> = all_nodes.iter()
                            .filter(|&&n| n != emitter)
                            .copied()
                            .collect();
                        if others.len() == 2 {
                            (others[0], others[1])
                        } else {
                            ep
                        }
                    } else {
                        ep
                    }
                } else {
                    ep
                };

                blocks.push(NlBlock { nl_edges, endpoints: ep, comp_idx: ci });
            }

            // Also recurse into non-Q children (P-nodes, nested S/R)
            for child in children {
                match child {
                    SpqrNode::Q { .. } => {} // already handled above
                    _ => find_nl_blocks(child, graph, blocks),
                }
            }
        }

        SpqrNode::Q { .. } => {} // Single leaf — not a block
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Step 2: Collect sibling Q-leaves (non-NL edges)
// ═══════════════════════════════════════════════════════════════════════════

/// Collect all Q-leaf edge indices that are NOT part of any NL block.
fn collect_sibling_edges(
    node: &SpqrNode,
    nl_edge_set: &HashSet<usize>,
    siblings: &mut Vec<usize>,
) {
    match node {
        SpqrNode::Q { edge_idx, .. } => {
            if !nl_edge_set.contains(edge_idx) {
                siblings.push(*edge_idx);
            }
        }
        SpqrNode::S { children, .. }
        | SpqrNode::P { children, .. } => {
            // Check if this P-node is an NL block (all children are NL from same comp)
            // If so, skip — its edges are already in nl_edge_set
            let is_nl_p = matches!(node, SpqrNode::P { .. })
                && children.iter().all(|c| {
                    matches!(c, SpqrNode::Q { edge_idx, .. } if nl_edge_set.contains(edge_idx))
                });
            if !is_nl_p {
                for child in children {
                    collect_sibling_edges(child, nl_edge_set, siblings);
                }
            }
        }
        SpqrNode::R { children, edge_indices, .. } => {
            // R-node edges that aren't NL
            for &eidx in edge_indices {
                if !nl_edge_set.contains(&eidx) {
                    siblings.push(eidx);
                }
            }
            for child in children {
                collect_sibling_edges(child, nl_edge_set, siblings);
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Step 3: Assign siblings to blocks by port node adjacency
// ═══════════════════════════════════════════════════════════════════════════

// (assign_sibling replaced by iterative node_to_block expansion in analyze_blockwise)

// ═══════════════════════════════════════════════════════════════════════════
// Step 4: Validation
// ═══════════════════════════════════════════════════════════════════════════

fn validate_plan(plan: &BlockwisePlan, graph: &CircuitGraph) -> bool {
    if plan.blocks.len() < 2 {
        return false;
    }
    for block in &plan.blocks {
        if block.nl_edges.is_empty() {
            return false;
        }
        if block.reactive_edges.is_empty() {
            return false;
        }
    }
    // Coupling must not contain NL
    for &eidx in &plan.coupling_edges {
        if graph.effective_edge_kind(eidx) == EdgeKind::Nonlinear {
            return false;
        }
    }
    true
}

// ═══════════════════════════════════════════════════════════════════════════
// Public entry point: analyze
// ═══════════════════════════════════════════════════════════════════════════

/// Analyze a set of edges for blockwise decomposition using the SPQR tree.
///
/// Returns `Some(BlockwisePlan)` if the circuit splits into ≥2 NL blocks
/// with sparse coupling. Returns `None` otherwise.
pub(super) fn analyze_blockwise(
    edge_indices: &[usize],
    graph: &CircuitGraph,
) -> Option<BlockwisePlan> {
    // Build SPQR tree for these edges
    let terminals = vec![graph.in_node, graph.out_node];
    let tree = spqr_decompose(edge_indices, &terminals, graph, graph.gnd_node);

    // Step 1: find NL blocks (P-nodes or S-node siblings)
    let mut nl_blocks = Vec::new();
    find_nl_blocks(&tree, graph, &mut nl_blocks);

    #[cfg(test)]
    eprintln!("  analyze_blockwise: found {} NL blocks from SPQR tree", nl_blocks.len());

    if nl_blocks.len() < 2 {
        return None;
    }

    // Build NL edge set and port node map
    let nl_edge_set: HashSet<usize> = nl_blocks
        .iter()
        .flat_map(|b| b.nl_edges.iter().copied())
        .collect();

    let block_port_nodes: Vec<(usize, (NodeId, NodeId))> = nl_blocks
        .iter()
        .enumerate()
        .map(|(i, b)| (i, b.endpoints))
        .collect();

    let excluded: HashSet<NodeId> = {
        let mut s = HashSet::new();
        s.insert(graph.gnd_node);
        for &n in &graph.supply_nodes { s.insert(n); }
        for &n in &graph.ac_ground_nodes { s.insert(n); }
        s
    };

    // Step 2: collect non-NL edges — everything that isn't in an NL block
    let sibling_edges: Vec<usize> = edge_indices
        .iter()
        .filter(|e| !nl_edge_set.contains(e))
        .copied()
        .collect();

    #[cfg(test)]
    eprintln!("  siblings: {} edges (of {} total, {} NL)",
        sibling_edges.len(), edge_indices.len(), nl_edge_set.len());

    // Step 3: assign siblings to blocks.
    // First pass: assign edges directly touching a block's port nodes.
    // Then expand: edges touching nodes already claimed by a block get
    // assigned to that block too (captures pot chains: R_e→Cutoff→gnd).
    let mut node_to_block: HashMap<NodeId, usize> = HashMap::new();
    for &(bi, (ep_a, ep_b)) in &block_port_nodes {
        // Port nodes may be shared between blocks — assign to the block
        // with the most NL edges at that node (same heuristic as before)
        for &node in &[ep_a, ep_b] {
            if excluded.contains(&node) { continue; }
            let existing = node_to_block.get(&node).copied();
            if existing.is_none() {
                node_to_block.insert(node, bi);
            }
            // If contested, keep the one with more NL edges at this node
            // (already handled by find_nl_blocks ordering — first writer wins)
        }
    }

    let mut block_linear: Vec<Vec<usize>> = vec![Vec::new(); nl_blocks.len()];
    let mut block_reactive: Vec<Vec<usize>> = vec![Vec::new(); nl_blocks.len()];
    #[cfg(test)]
    eprintln!("  node_to_block seed: {:?}", node_to_block);

    let mut unassigned: Vec<usize> = sibling_edges.clone();
    let mut coupling_edges = Vec::new();

    // Iterative assignment: keep passing over unassigned edges until
    // no more can be claimed. Each assigned edge's "other" node becomes
    // available for the next pass.
    loop {
        let mut newly_assigned = Vec::new();
        let mut still_unassigned = Vec::new();

        for &eidx in &unassigned {
            let e = &graph.edges[eidx];
            let owner_a = if excluded.contains(&e.node_a) { None }
                else { node_to_block.get(&e.node_a).copied() };
            let owner_b = if excluded.contains(&e.node_b) { None }
                else { node_to_block.get(&e.node_b).copied() };

            match (owner_a, owner_b) {
                (Some(a), Some(b)) if a == b => {
                    // Both nodes owned by same block → interior
                    newly_assigned.push((eidx, a));
                }
                (Some(a), None) => {
                    newly_assigned.push((eidx, a));
                    if !excluded.contains(&e.node_b) {
                        node_to_block.insert(e.node_b, a);
                    }
                    #[cfg(test)]
                    eprintln!("    assign edge {eidx} {}({:?}) → block {a}",
                        graph.components[e.comp_idx].id, graph.effective_edge_kind(eidx));
                }
                (None, Some(b)) => {
                    newly_assigned.push((eidx, b));
                    if !excluded.contains(&e.node_a) {
                        node_to_block.insert(e.node_a, b);
                    }
                    #[cfg(test)]
                    eprintln!("    assign edge {eidx} {}({:?}) → block {b}",
                        graph.components[e.comp_idx].id, graph.effective_edge_kind(eidx));
                }
                (Some(a), Some(b)) if a != b => {
                    // Spans two blocks → coupling
                    coupling_edges.push(eidx);
                }
                _ => {
                    // Neither node owned yet → try again next pass
                    still_unassigned.push(eidx);
                }
            }
        }

        // Assign newly found edges
        for (eidx, bi) in &newly_assigned {
            match graph.effective_edge_kind(*eidx) {
                EdgeKind::Linear => block_linear[*bi].push(*eidx),
                EdgeKind::Reactive => block_reactive[*bi].push(*eidx),
                _ => coupling_edges.push(*eidx),
            }
        }

        if newly_assigned.is_empty() {
            // No progress — remaining are truly unconnected → coupling
            coupling_edges.extend(still_unassigned);
            break;
        }

        unassigned = still_unassigned;
    }

    // Build the plan
    let mut blocks = Vec::new();
    for (i, nl_block) in nl_blocks.into_iter().enumerate() {
        blocks.push(Block {
            nl_edges: nl_block.nl_edges,
            linear_edges: std::mem::take(&mut block_linear[i]),
            reactive_edges: std::mem::take(&mut block_reactive[i]),
            port_nodes: vec![nl_block.endpoints.0, nl_block.endpoints.1],
        });
    }

    let all_port_nodes: Vec<NodeId> = blocks
        .iter()
        .flat_map(|b| b.port_nodes.iter().copied())
        .collect::<HashSet<_>>()
        .into_iter()
        .collect();

    let plan = BlockwisePlan {
        blocks,
        coupling_edges,
        port_nodes: all_port_nodes,
    };

    if validate_plan(&plan, graph) {
        Some(plan)
    } else {
        #[cfg(test)]
        {
            for (i, b) in plan.blocks.iter().enumerate() {
                eprintln!("  validate fail: block {i}: {} NL, {} reactive, {} linear",
                    b.nl_edges.len(), b.reactive_edges.len(), b.linear_edges.len());
            }
        }
        None
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Public entry point: build
// ═══════════════════════════════════════════════════════════════════════════

/// Try blockwise decomposition and build. Returns `Some(stages)` if the
/// circuit decomposes, `None` to fall through to monolithic.
pub(super) fn try_build_blockwise(
    edge_indices: &[usize],
    graph: &CircuitGraph,
    terminals: &[NodeId],
    sample_rate: f64,
    bias_node_voltages: &hashbrown::HashMap<NodeId, f64>,
    supply_voltage: f64,
) -> Option<Vec<BuiltStage>> {
    let plan = analyze_blockwise(edge_indices, graph)?;

    #[cfg(test)]
    eprintln!(
        "  Blockwise: {} blocks, {} coupling edges",
        plan.num_blocks(),
        plan.coupling_edges.len()
    );

    let mut all_stages = Vec::new();

    for (bi, block) in plan.blocks.iter().enumerate() {
        let block_edges = block.all_edges();
        if block_edges.is_empty() {
            continue;
        }

        let block_terminals =
            super::spqr_build::compute_group_terminals(&block_edges, graph, terminals);
        let spqr_tree = spqr_decompose(&block_edges, &block_terminals, graph, graph.gnd_node);
        let spqr_stages = spqr_to_stages(&spqr_tree, graph, sample_rate);

        #[cfg(test)]
        {
            let class = super::spqr::classify_sp_subtree(&spqr_tree, graph);
            eprintln!(
                "  Block {bi}: {} edges → {} stages, class={:?}",
                block_edges.len(),
                spqr_stages.len(),
                class,
            );
        }

        for stage in spqr_stages {
            let mut built = super::spqr_build::build_spqr_stage(stage, graph, sample_rate)
                .ok()?;

            // Set BJT bias from circuit analysis
            if let BuiltStage::Wdf(ref mut wdf) = built {
                if let pedalkernel_rt::stage::RootKind::Bjt(ref mut bjt) = wdf.root {
                    let base_bias = block.nl_edges.iter().find_map(|&eidx| {
                        let e = &graph.edges[eidx];
                        let comp = &graph.components[e.comp_idx];
                        let base_key = format!("{}.base", comp.id);
                        let base_node = graph.node_names.get(&base_key)?;
                        bias_node_voltages.get(base_node).copied()
                    });

                    if let Some(v_base) = base_bias {
                        bjt.set_bias(v_base.min(0.8));
                        #[cfg(test)]
                        eprintln!("  Block {bi}: BJT bias = {v_base:.3}V (from circuit)");
                    } else {
                        let default_vbe = if supply_voltage > 1.0 { 0.6 } else { 0.3 };
                        bjt.set_bias(default_vbe);
                        #[cfg(test)]
                        eprintln!("  Block {bi}: BJT bias = {default_vbe:.1}V (default)");
                    }

                    bjt.set_v_max(supply_voltage.abs().max(1.0));
                }
            }

            all_stages.push(built);
        }
    }

    // Coupling edges → passive stages
    if !plan.coupling_edges.is_empty() {
        let coupling_terminals =
            super::spqr_build::compute_group_terminals(&plan.coupling_edges, graph, terminals);
        let spqr_tree = spqr_decompose(
            &plan.coupling_edges,
            &coupling_terminals,
            graph,
            graph.gnd_node,
        );
        let spqr_stages = spqr_to_stages(&spqr_tree, graph, sample_rate);

        for stage in spqr_stages {
            let built = super::spqr_build::build_spqr_stage(stage, graph, sample_rate)
                .ok()?;
            all_stages.push(built);
        }
    }

    Some(all_stages)
}
