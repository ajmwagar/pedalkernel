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
use std::collections::{BTreeMap, HashMap, HashSet};

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
    /// All nodes touched by this block's NL edges.
    nodes: Vec<NodeId>,
    /// Component index of the NL device.
    comp_idx: usize,
}

fn component_pin_node(graph: &CircuitGraph, comp_id: &str, pin: &str) -> Option<NodeId> {
    graph.node_names.get(&format!("{comp_id}.{pin}")).copied()
}

fn nl_block_direction_nodes(block: &NlBlock, graph: &CircuitGraph) -> Option<(NodeId, NodeId)> {
    let comp = &graph.components[block.comp_idx];
    match comp.kind.signal_terminals() {
        super::component::SignalTerminals::Amplifier { input, output, .. } => {
            let input_node = component_pin_node(graph, &comp.id, input)?;
            let output_node = component_pin_node(graph, &comp.id, output)?;
            let common_node = comp
                .kind
                .pin_config()
                .valid_pins
                .iter()
                .find(|&&p| p != input && p != output)
                .and_then(|&pin| component_pin_node(graph, &comp.id, pin));

            // Diode-connected transistor ladders short the declared input/output
            // pins and take the stage output from the common terminal. This makes
            // each block directional as base/collector -> emitter.
            if input_node == output_node {
                let common_node = common_node?;
                if block.nodes.contains(&input_node) && block.nodes.contains(&common_node) {
                    return Some((input_node, common_node));
                }
            }

            if block.nodes.contains(&input_node) && block.nodes.contains(&output_node) {
                Some((input_node, output_node))
            } else {
                None
            }
        }
        super::component::SignalTerminals::TwoPort { input, output } => {
            let input_node = component_pin_node(graph, &comp.id, input)?;
            let output_node = component_pin_node(graph, &comp.id, output)?;
            if block.nodes.contains(&input_node) && block.nodes.contains(&output_node) {
                Some((input_node, output_node))
            } else {
                None
            }
        }
        super::component::SignalTerminals::Passive => None,
    }
}

pub(super) fn block_coupling_port_node(
    nl_edges: &[usize],
    port_nodes: &[NodeId],
    graph: &CircuitGraph,
) -> Option<NodeId> {
    let comp_idx = nl_edges.first().map(|&eidx| graph.edges[eidx].comp_idx)?;
    let comp = &graph.components[comp_idx];
    let (input, output) = match comp.kind.signal_terminals() {
        super::component::SignalTerminals::Amplifier { input, output, .. }
        | super::component::SignalTerminals::TwoPort { input, output } => (input, output),
        super::component::SignalTerminals::Passive => return None,
    };

    let input_node = component_pin_node(graph, &comp.id, input)?;
    let output_node = component_pin_node(graph, &comp.id, output)?;

    // For diode-connected transistors, input and output pins are shorted.
    // This is still the driven side of the one-port; the common pin
    // (emitter/source/cathode) is the cascade output and must not be used as
    // the coupling matrix port.
    if input_node == output_node && port_nodes.contains(&input_node) {
        return Some(input_node);
    }

    if port_nodes.contains(&input_node) {
        Some(input_node)
    } else {
        None
    }
}

fn block_cascade_output_node(
    nl_edges: &[usize],
    port_nodes: &[NodeId],
    graph: &CircuitGraph,
) -> Option<NodeId> {
    let comp_idx = nl_edges.first().map(|&eidx| graph.edges[eidx].comp_idx)?;
    let comp = &graph.components[comp_idx];
    let (input, output, is_amplifier) = match comp.kind.signal_terminals() {
        super::component::SignalTerminals::Amplifier { input, output, .. } => (input, output, true),
        super::component::SignalTerminals::TwoPort { input, output } => (input, output, false),
        super::component::SignalTerminals::Passive => return None,
    };

    let input_node = component_pin_node(graph, &comp.id, input)?;
    let output_node = component_pin_node(graph, &comp.id, output)?;

    if is_amplifier && input_node == output_node {
        return comp
            .kind
            .pin_config()
            .valid_pins
            .iter()
            .find(|&&p| p != input && p != output)
            .and_then(|&pin| component_pin_node(graph, &comp.id, pin))
            .filter(|node| port_nodes.contains(node));
    }

    if port_nodes.contains(&output_node) {
        Some(output_node)
    } else {
        None
    }
}

fn signal_order_indices(nl_blocks: &[NlBlock], graph: &CircuitGraph) -> Option<Vec<usize>> {
    let dirs: Vec<Option<(NodeId, NodeId)>> = nl_blocks
        .iter()
        .map(|block| nl_block_direction_nodes(block, graph))
        .collect();
    if dirs.iter().any(Option::is_none) {
        return None;
    }
    let dirs: Vec<(NodeId, NodeId)> = dirs.into_iter().map(Option::unwrap).collect();

    let downstream_nodes: HashSet<NodeId> =
        dirs.iter().map(|(_, downstream)| *downstream).collect();
    let mut start_candidates: Vec<usize> = dirs
        .iter()
        .enumerate()
        .filter_map(|(i, (upstream, _))| (!downstream_nodes.contains(upstream)).then_some(i))
        .collect();

    if start_candidates.is_empty() {
        return None;
    }

    start_candidates.sort_by_key(|&i| {
        let (upstream, _) = dirs[i];
        let touches_input =
            nl_blocks[i].nodes.contains(&graph.in_node) || upstream == graph.in_node;
        (!touches_input, i)
    });

    let mut ordered = Vec::with_capacity(nl_blocks.len());
    let mut used = HashSet::new();
    let mut current = start_candidates[0];

    loop {
        ordered.push(current);
        used.insert(current);

        let (_, downstream) = dirs[current];
        let mut next: Vec<usize> = dirs
            .iter()
            .enumerate()
            .filter_map(|(i, (upstream, _))| {
                (!used.contains(&i) && *upstream == downstream).then_some(i)
            })
            .collect();
        if next.is_empty() {
            break;
        }
        next.sort_unstable();
        current = next[0];
    }

    if ordered.len() == nl_blocks.len() {
        Some(ordered)
    } else {
        None
    }
}

/// Recursively extract all NL Q-leaf edge indices from a subtree.
/// Skips P-nodes that are all-NL-same-comp (those are found by
/// find_nl_blocks pattern 1 and should not be broken up).
fn extract_nl_q_leaves(node: &SpqrNode, graph: &CircuitGraph, out: &mut Vec<usize>) {
    match node {
        SpqrNode::Q { edge_idx, .. } => {
            if graph.effective_edge_kind(*edge_idx) == EdgeKind::Nonlinear {
                out.push(*edge_idx);
            }
        }
        SpqrNode::P { children, .. } => {
            // Check if this is an all-NL-same-comp P-node (pattern 1 candidate)
            let all_nl_same = {
                let mut ci = None;
                let mut ok = true;
                for c in children {
                    if let SpqrNode::Q { edge_idx, .. } = c {
                        if graph.effective_edge_kind(*edge_idx) == EdgeKind::Nonlinear {
                            let c2 = graph.edges[*edge_idx].comp_idx;
                            if let Some(prev) = ci {
                                if prev != c2 {
                                    ok = false;
                                }
                            } else {
                                ci = Some(c2);
                            }
                        } else {
                            ok = false;
                        }
                    } else {
                        ok = false;
                    }
                }
                ok && ci.is_some()
            };
            if !all_nl_same {
                // Mixed P-node (pendant wrapper) — extract NL Q-leaves
                for c in children {
                    extract_nl_q_leaves(c, graph, out);
                }
            }
            // all-NL-same-comp → skip, handled by find_nl_blocks pattern 1
        }
        _ => {} // S/R nodes are structural, don't descend
    }
}

/// Scan children for NL Q-leaves, grouping by component. Recursively
/// looks through P-node pendant wrappers at any depth. Then recurse
/// into non-Q children that weren't consumed.
fn collect_nl_q_children(children: &[SpqrNode], graph: &CircuitGraph, blocks: &mut Vec<NlBlock>) {
    let mut comp_groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    let mut consumed: Vec<usize> = Vec::new(); // indices of children fully consumed

    for (i, child) in children.iter().enumerate() {
        let mut nl_leaves = Vec::new();
        extract_nl_q_leaves(child, graph, &mut nl_leaves);
        if !nl_leaves.is_empty() {
            consumed.push(i);
            for eidx in nl_leaves {
                let ci = graph.edges[eidx].comp_idx;
                comp_groups.entry(ci).or_default().push(eidx);
            }
        }
    }

    for (ci, nl_edges) in comp_groups {
        if nl_edges.is_empty() {
            continue;
        }
        let mut all_nodes: Vec<NodeId> = Vec::new();
        for &eidx in &nl_edges {
            let e = &graph.edges[eidx];
            if !all_nodes.contains(&e.node_a) {
                all_nodes.push(e.node_a);
            }
            if !all_nodes.contains(&e.node_b) {
                all_nodes.push(e.node_b);
            }
        }
        if all_nodes.is_empty() {
            continue;
        }
        blocks.push(NlBlock {
            nl_edges,
            nodes: all_nodes,
            comp_idx: ci,
        });
    }

    // Recurse into children that weren't consumed
    for (i, child) in children.iter().enumerate() {
        if consumed.contains(&i) {
            continue;
        }
        match child {
            SpqrNode::Q { .. } => {} // passive Q, not relevant
            _ => find_nl_blocks(child, graph, blocks),
        }
    }
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
        SpqrNode::P {
            children,
            endpoints,
        } => {
            // Pattern 1: P-node with all-NL children from same component
            let mut nl_edges = Vec::new();
            let mut comp_idx = None;
            let mut all_nl_same_comp = true;

            for child in children {
                if let SpqrNode::Q { edge_idx, .. } = child {
                    if graph.effective_edge_kind(*edge_idx) == EdgeKind::Nonlinear {
                        let ci = graph.edges[*edge_idx].comp_idx;
                        if let Some(existing) = comp_idx {
                            if existing != ci {
                                all_nl_same_comp = false;
                            }
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
                        nodes: vec![endpoints.0, endpoints.1],
                        comp_idx: ci,
                    });
                    return;
                }
            }

            // Fallback: mixed P-node. Scan Q children for NL edges
            // (same as S-node pattern), then recurse into non-Q children.
            collect_nl_q_children(children, graph, blocks);
        }

        SpqrNode::S { children, .. } | SpqrNode::R { children, .. } => {
            // Pattern 2: group NL Q-leaf siblings by comp_idx,
            // then recurse into non-Q children.
            collect_nl_q_children(children, graph, blocks);
        }

        SpqrNode::Q { .. } => {} // Single leaf — not a block
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Step 2: Collect sibling Q-leaves (non-NL edges)
// ═══════════════════════════════════════════════════════════════════════════

/// Collect all Q-leaf edge indices that are NOT part of any NL block.
fn collect_sibling_edges(node: &SpqrNode, nl_edge_set: &HashSet<usize>, siblings: &mut Vec<usize>) {
    match node {
        SpqrNode::Q { edge_idx, .. } => {
            if !nl_edge_set.contains(edge_idx) {
                siblings.push(*edge_idx);
            }
        }
        SpqrNode::S { children, .. } | SpqrNode::P { children, .. } => {
            // Check if this P-node is an NL block (all children are NL from same comp)
            // If so, skip — its edges are already in nl_edge_set
            let is_nl_p = matches!(node, SpqrNode::P { .. })
                && children.iter().all(
                    |c| matches!(c, SpqrNode::Q { edge_idx, .. } if nl_edge_set.contains(edge_idx)),
                );
            if !is_nl_p {
                for child in children {
                    collect_sibling_edges(child, nl_edge_set, siblings);
                }
            }
        }
        SpqrNode::R {
            children,
            edge_indices,
            ..
        } => {
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
    // Build SPQR tree for these edges.
    // Terminals may be graph.in_node/out_node (which might be port nodes
    // outside this subgraph). The SPQR series chain walk handles this safely
    // via visited-set cycle detection — no infinite loop even when terminals
    // are absent from the subgraph.
    let terminals = vec![graph.in_node, graph.out_node];
    eprintln!(
        "  [blockwise] analyze: SPQR decompose ({} edges, terminals={terminals:?})...",
        edge_indices.len()
    );
    let tree = spqr_decompose(edge_indices, &terminals, graph, graph.gnd_node);
    eprintln!("  [blockwise] analyze: SPQR done, finding NL blocks...");

    // Step 1: find NL blocks (P-nodes or S-node siblings)
    let mut nl_blocks = Vec::new();
    find_nl_blocks(&tree, graph, &mut nl_blocks);

    eprintln!("  [blockwise] analyze: found {} NL blocks", nl_blocks.len());

    if nl_blocks.len() < 2 {
        return None;
    }

    // Build NL edge set and port node map
    let nl_edge_set: HashSet<usize> = nl_blocks
        .iter()
        .flat_map(|b| b.nl_edges.iter().copied())
        .collect();

    let block_port_nodes: Vec<(usize, Vec<NodeId>)> = nl_blocks
        .iter()
        .enumerate()
        .map(|(i, b)| (i, b.nodes.clone()))
        .collect();

    let excluded: HashSet<NodeId> = {
        let mut s = HashSet::new();
        s.insert(graph.gnd_node);
        s.insert(graph.vcc_node);
        for &n in &graph.supply_nodes {
            s.insert(n);
        }
        for &n in &graph.ac_ground_nodes {
            s.insert(n);
        }
        s
    };

    // Step 2: collect non-NL edges — everything that isn't in an NL block
    let sibling_edges: Vec<usize> = edge_indices
        .iter()
        .filter(|e| !nl_edge_set.contains(e))
        .copied()
        .collect();

    #[cfg(test)]
    eprintln!(
        "  siblings: {} edges (of {} total, {} NL)",
        sibling_edges.len(),
        edge_indices.len(),
        nl_edge_set.len()
    );

    // Step 3: assign siblings to blocks.
    //
    // Node classification:
    // - A port node appearing in exactly 1 block → owned by that block
    // - A port node appearing in 2+ blocks → boundary (coupling interface)
    //   Neither block owns it. Edges here are assigned by their OTHER endpoint.
    // - Ground/supply → excluded (not owned, not boundary)
    //
    // Then iteratively expand: edges where one endpoint is owned and the
    // other is unclaimed → assign to the owning block, claim the other node.

    // Find boundary nodes (shared between blocks)
    let mut node_block_count: HashMap<NodeId, HashSet<usize>> = HashMap::new();
    for (bi, nodes) in &block_port_nodes {
        for &node in nodes {
            if excluded.contains(&node) {
                continue;
            }
            node_block_count.entry(node).or_default().insert(*bi);
        }
    }
    let boundary_nodes: HashSet<NodeId> = node_block_count
        .iter()
        .filter(|(_, blocks)| blocks.len() > 1)
        .map(|(&node, _)| node)
        .collect();

    // Seed node_to_block with non-boundary port nodes
    let mut node_to_block: HashMap<NodeId, usize> = HashMap::new();
    for (bi, nodes) in &block_port_nodes {
        for &node in nodes {
            if excluded.contains(&node) || boundary_nodes.contains(&node) {
                continue;
            }
            node_to_block.entry(node).or_insert(*bi);
        }
    }

    let mut block_linear: Vec<Vec<usize>> = vec![Vec::new(); nl_blocks.len()];
    let mut block_reactive: Vec<Vec<usize>> = vec![Vec::new(); nl_blocks.len()];
    #[cfg(test)]
    eprintln!("  node_to_block seed: {:?}", node_to_block);

    let mut coupling_edges = Vec::new();

    // Pre-classify: find edges on paths connecting two different NL blocks.
    // These are coupling by definition — the greedy expansion must not
    // claim them for any single block. Without this, multi-hop feedback
    // paths (e.g., Resonance → R_fb_limit between Q4 and Q1) get greedily
    // absorbed into the first block they touch.
    //
    // Algorithm: BFS from each NL block's port nodes through sibling edges.
    // If a path reaches another block's port node, all edges on that path
    // are coupling.
    let coupling_set: HashSet<usize> = {
        // Build adjacency for sibling edges only
        let mut adj: HashMap<NodeId, Vec<(usize, NodeId)>> = HashMap::new();
        for &eidx in &sibling_edges {
            let e = &graph.edges[eidx];
            adj.entry(e.node_a).or_default().push((eidx, e.node_b));
            adj.entry(e.node_b).or_default().push((eidx, e.node_a));
        }

        // Map: node → which block(s) own it (from NL port nodes)
        let mut node_blocks: HashMap<NodeId, Vec<usize>> = HashMap::new();
        for (bi, nodes) in &block_port_nodes {
            for &n in nodes {
                if !excluded.contains(&n) {
                    node_blocks.entry(n).or_default().push(*bi);
                }
            }
        }

        let mut inter_block_edges: HashSet<usize> = HashSet::new();

        // BFS from each block's port nodes through sibling edges
        for (bi, nodes) in &block_port_nodes {
            for &start in nodes {
                if excluded.contains(&start) {
                    continue;
                }
                // BFS to find paths to other blocks
                let mut visited: HashSet<NodeId> = HashSet::new();
                let mut queue: Vec<(NodeId, Vec<usize>)> = vec![(start, Vec::new())];
                visited.insert(start);

                while let Some((node, path)) = queue.pop() {
                    // Did we reach a different block's port node?
                    if let Some(blocks) = node_blocks.get(&node) {
                        if blocks.iter().any(|b| b != bi) && !path.is_empty() {
                            // This path connects two blocks → all edges are coupling
                            inter_block_edges.extend(path.iter());
                            continue; // don't expand further from this node
                        }
                    }

                    if let Some(neighbors) = adj.get(&node) {
                        for &(eidx, next) in neighbors {
                            if visited.contains(&next) {
                                continue;
                            }
                            if excluded.contains(&next) {
                                continue;
                            }
                            visited.insert(next);
                            let mut new_path = path.clone();
                            new_path.push(eidx);
                            queue.push((next, new_path));
                        }
                    }
                }
            }
        }

        #[cfg(test)]
        if !inter_block_edges.is_empty() {
            let names: Vec<&str> = inter_block_edges
                .iter()
                .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
                .collect();
            eprintln!("  inter-block coupling detected: {:?}", names);
        }

        inter_block_edges
    };
    eprintln!(
        "  [blockwise] analyze: inter-block coupling done ({} edges)",
        coupling_set.len()
    );

    // Step 3: Pendant-chain assignment.
    //
    // Each block claims only edges reachable from its NL device's
    // COMMON terminal (emitter/source/cathode) via pendant chains
    // to ground/rail. This produces identical blocks for symmetric
    // cascades — the "repeatable unit" — and pushes contextual
    // edges (bias resistors, input/output networks) to coupling.
    //
    // Algorithm per block:
    //   1. Find the NL device's common terminal node
    //   2. BFS through sibling edges from that node
    //   3. Claim edges that terminate at excluded nodes (GND/VCC/rail)
    //   4. Stop at boundary nodes — don't cross into other blocks

    let sibling_set: HashSet<usize> = sibling_edges.iter().copied().collect();
    let assigned_set: HashSet<usize> = coupling_set.clone(); // start with coupling pre-excluded

    // Build adjacency for sibling edges
    let mut sib_adj: HashMap<NodeId, Vec<(usize, NodeId)>> = HashMap::new();
    for &eidx in &sibling_edges {
        if coupling_set.contains(&eidx) {
            continue;
        }
        let e = &graph.edges[eidx];
        sib_adj.entry(e.node_a).or_default().push((eidx, e.node_b));
        sib_adj.entry(e.node_b).or_default().push((eidx, e.node_a));
    }

    // Build map: node → which block(s) it belongs to (from NL port nodes)
    let mut node_owner: HashMap<NodeId, Vec<usize>> = HashMap::new();
    for (bi, nodes) in &block_port_nodes {
        for &n in nodes {
            if !excluded.contains(&n) {
                node_owner.entry(n).or_default().push(*bi);
            }
        }
    }

    // For each block, find its common terminal and BFS for pendant chains
    let mut claimed: HashSet<usize> = coupling_set.clone();

    for (bi, nl_block) in nl_blocks.iter().enumerate() {
        // Find common terminal (emitter/source/cathode)
        let comp = &graph.components[nl_block.comp_idx];
        let common_node = match comp.kind.signal_terminals() {
            super::component::SignalTerminals::Amplifier { input, output, .. } => {
                let common_pin = comp
                    .kind
                    .pin_config()
                    .valid_pins
                    .iter()
                    .find(|&&p| p != input && p != output);
                common_pin.and_then(|&pin| {
                    let key = format!("{}.{}", comp.id, pin);
                    graph.node_names.get(&key).copied()
                })
            }
            super::component::SignalTerminals::TwoPort { output, .. } => {
                let key = format!("{}.{}", comp.id, output);
                graph.node_names.get(&key).copied()
            }
            super::component::SignalTerminals::Passive => None,
        };

        let start_node = match common_node {
            Some(n) => n,
            None => continue, // can't determine common terminal
        };

        // BFS from common terminal through sibling edges
        let mut visited: HashSet<NodeId> = HashSet::new();
        let mut queue: Vec<NodeId> = vec![start_node];
        visited.insert(start_node);

        while let Some(node) = queue.pop() {
            if let Some(neighbors) = sib_adj.get(&node) {
                for &(eidx, next) in neighbors {
                    if claimed.contains(&eidx) {
                        continue;
                    }
                    if visited.contains(&next) {
                        continue;
                    }

                    // Claim this edge if it terminates at ground
                    // OR leads to a non-boundary interior node.
                    // Don't claim VCC/supply-connected edges at boundary
                    // nodes — those are bias networks for the next stage.
                    let terminates_at_gnd =
                        next == graph.gnd_node || graph.ac_ground_nodes.contains(&next);
                    let terminates_at_vcc =
                        next == graph.vcc_node || graph.supply_nodes.contains(&next);
                    // Stop at nodes owned by other blocks, circuit terminals,
                    // or shared boundaries. The BFS is allowed to START at a
                    // boundary (the emitter IS the cascade junction) but
                    // shouldn't cross INTO other blocks or the I/O interface.
                    let next_owned_by_other = node_owner
                        .get(&next)
                        .map_or(false, |owners| owners.iter().any(|&o| o != bi));
                    let next_is_terminal = next == graph.in_node || next == graph.out_node;
                    let is_boundary =
                        boundary_nodes.contains(&next) || next_owned_by_other || next_is_terminal;

                    if terminates_at_gnd {
                        // Edge to ground → pendant, claim for this block
                        claimed.insert(eidx);
                        match graph.effective_edge_kind(eidx) {
                            EdgeKind::Linear => block_linear[bi].push(eidx),
                            EdgeKind::Reactive => block_reactive[bi].push(eidx),
                            _ => {}
                        }
                        #[cfg(test)]
                        eprintln!(
                            "    assign edge {eidx} {}({:?}) → block {bi} (pendant to gnd)",
                            graph.components[graph.edges[eidx].comp_idx].id,
                            graph.effective_edge_kind(eidx)
                        );
                    } else if terminates_at_vcc {
                        // VCC-connected at boundary → coupling (bias for next stage)
                        // VCC-connected at non-boundary → claim (only this block's bias)
                        if !boundary_nodes.contains(&node) {
                            claimed.insert(eidx);
                            match graph.effective_edge_kind(eidx) {
                                EdgeKind::Linear => block_linear[bi].push(eidx),
                                EdgeKind::Reactive => block_reactive[bi].push(eidx),
                                _ => {}
                            }
                            #[cfg(test)]
                            eprintln!("    assign edge {eidx} {}({:?}) → block {bi} (pendant to vcc, non-boundary)",
                                graph.components[graph.edges[eidx].comp_idx].id,
                                graph.effective_edge_kind(eidx));
                        }
                        // else: leave unclaimed → becomes coupling
                    } else if !is_boundary {
                        // Interior node (not shared) → claim and continue BFS
                        visited.insert(next);
                        claimed.insert(eidx);
                        match graph.effective_edge_kind(eidx) {
                            EdgeKind::Linear => block_linear[bi].push(eidx),
                            EdgeKind::Reactive => block_reactive[bi].push(eidx),
                            _ => {}
                        }
                        queue.push(next);
                        #[cfg(test)]
                        eprintln!(
                            "    assign edge {eidx} {}({:?}) → block {bi} (pendant interior)",
                            graph.components[graph.edges[eidx].comp_idx].id,
                            graph.effective_edge_kind(eidx)
                        );
                    }
                    // If boundary → don't claim, don't cross. Edge stays unassigned.
                }
            }
        }
    }

    // Inter-block coupling (from BFS pre-classification) + unclaimed → coupling
    for &eidx in &sibling_edges {
        if coupling_set.contains(&eidx) && !coupling_edges.contains(&eidx) {
            coupling_edges.push(eidx);
        }
    }
    for &eidx in &sibling_edges {
        if !claimed.contains(&eidx) {
            coupling_edges.push(eidx);
            #[cfg(test)]
            eprintln!(
                "    coupling: edge {eidx} {}({:?})",
                graph.components[graph.edges[eidx].comp_idx].id,
                graph.effective_edge_kind(eidx)
            );
        }
    }

    let block_order: Vec<usize> =
        signal_order_indices(&nl_blocks, graph).unwrap_or_else(|| (0..nl_blocks.len()).collect());

    #[cfg(test)]
    {
        let order_names: Vec<&str> = block_order
            .iter()
            .map(|&i| graph.components[nl_blocks[i].comp_idx].id.as_str())
            .collect();
        eprintln!("  block signal order: {:?}", order_names);
    }

    // Build the plan
    let mut blocks = Vec::new();
    for i in block_order {
        let nl_block = &nl_blocks[i];
        blocks.push(Block {
            nl_edges: nl_block.nl_edges.clone(),
            linear_edges: std::mem::take(&mut block_linear[i]),
            reactive_edges: std::mem::take(&mut block_reactive[i]),
            port_nodes: nl_block.nodes.clone(),
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
                eprintln!(
                    "  validate fail: block {i}: {} NL, {} reactive, {} linear",
                    b.nl_edges.len(),
                    b.reactive_edges.len(),
                    b.linear_edges.len()
                );
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
    bias_node_voltages: &std::collections::BTreeMap<NodeId, f64>,
    supply_voltage: f64,
    port_defs: &[crate::dsl::PortDef],
    force_serial: bool,
    disable_iir: bool,
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

        for (si, stage) in spqr_stages.into_iter().enumerate() {
            eprintln!("  [blockwise] block {bi} stage {si}: building...");
            let mut built = super::spqr_build::build_spqr_stage_with_options(
                stage,
                graph,
                sample_rate,
                disable_iir,
            )
            .ok()?;
            eprintln!("  [blockwise] block {bi} stage {si}: built");

            if let BuiltStage::Wdf(ref mut wdf) = built {
                if wdf.output_probe.is_none() {
                    if let Some(&reactive_edge) = block.reactive_edges.first() {
                        let comp = &graph.components[graph.edges[reactive_edge].comp_idx];
                        wdf.output_probe = Some(comp.id.clone());
                    }
                }
            }

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

    // ── Build coupling scattering matrix + package as BlockwiseKMethodStage ──
    //
    // Instead of building coupling edges as a separate passive WDF stage
    // (which has no feedback loop), we build a coupling scattering matrix
    // that connects all blocks' ports. The BlockwiseKMethodStage uses
    // wave-domain Newton iteration to solve the algebraic feedback loop
    // each sample — no unit delay, correct resonance tracking.
    if !plan.coupling_edges.is_empty() && !all_stages.is_empty() {
        let coupling_names: Vec<&str> = plan
            .coupling_edges
            .iter()
            .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
            .collect();
        eprintln!(
            "  [blockwise] coupling: {} edges {:?}",
            plan.coupling_edges.len(),
            coupling_names
        );

        // Extract KMethodBlocks from the built WDF stages.
        //
        // Each block's VS Rp must match the source impedance driving it:
        // - Block 0: driven by R_in from external input → VS Rp from port
        //   impedance (e.g. 10kΩ). Set by with_voltage_source() default.
        // - Blocks 1..N: driven by previous diode's output → VS Rp = 1/gm
        //   where gm = I_bias / Vt. The bias current comes from R_bias to VCC.
        //
        // Without this, VS Rp = 10kΩ for all blocks → gamma ≈ 0.97 → cap
        // can't filter → each stage adds gain → 12× total gain explosion.

        // Compute a fallback 1/gm from bias: I = (V_supply - V_be) / R_bias.
        // Individual blocks below prefer their own bias leg because cascaded
        // diode ladders often use different bias resistors per rung.
        let vt = 0.02585; // thermal voltage at 25°C
        let v_be = 0.6; // typical forward bias
                        // Find the typical R_bias value from coupling edges
        let r_bias_value = plan
            .coupling_edges
            .iter()
            .find_map(|&eidx| {
                let comp = &graph.components[graph.edges[eidx].comp_idx];
                if comp.id.contains("R_bias") || comp.id.contains("r_bias") {
                    comp.kind.resistance()
                } else {
                    None
                }
            })
            .unwrap_or(100_000.0);
        let i_bias = (supply_voltage - v_be).max(0.1) / r_bias_value;
        let gm = i_bias / vt;
        let r_source_cascade = (1.0 / gm).clamp(10.0, 10_000.0); // 1/gm, clamped

        #[cfg(test)]
        eprintln!(
            "  cascade source impedance: R_bias={r_bias_value:.0}Ω, I_bias={i_bias:.1e}A, \
                   gm={gm:.4}S, 1/gm={r_source_cascade:.1}Ω"
        );

        let first_block_source_r = port_defs
            .iter()
            .find(|port| port.direction == pedalkernel_rt::PortDirection::Input)
            .and_then(|port| port.impedance)
            .unwrap_or(10_000.0);

        let mut k_blocks = Vec::new();
        for (bi, built) in all_stages.iter_mut().enumerate() {
            if let BuiltStage::Wdf(ref mut wdf) = built {
                let block_port_node = plan.blocks.get(bi).and_then(|block| {
                    block_coupling_port_node(&block.nl_edges, &block.port_nodes, graph)
                });
                let block_bias_resistance = block_port_node
                    .and_then(|port_node| {
                        plan.coupling_edges.iter().find_map(|&eidx| {
                            let e = &graph.edges[eidx];
                            let comp = &graph.components[e.comp_idx];
                            let touches_port = e.node_a == port_node || e.node_b == port_node;
                            let touches_supply = e.node_a == graph.vcc_node
                                || e.node_b == graph.vcc_node
                                || graph.supply_nodes.contains(&e.node_a)
                                || graph.supply_nodes.contains(&e.node_b);
                            if touches_port && touches_supply {
                                comp.kind.resistance()
                            } else {
                                None
                            }
                        })
                    })
                    .unwrap_or(r_bias_value);
                // Block 0 is driven through the declared input impedance.
                // Downstream rungs are driven by the previous diode follower,
                // whose small-signal output impedance is set by that rung's
                // own DC bias current.
                let block_source_r = if bi == 0 {
                    first_block_source_r
                } else {
                    match &wdf.root {
                        pedalkernel_rt::stage::RootKind::ExplicitSingleDiode(root) => root
                            .model
                            .dynamic_resistance_from_sources(&[(
                                supply_voltage,
                                block_bias_resistance,
                            )])
                            .clamp(10.0, 10_000.0),
                        _ => {
                            let i_bias = (supply_voltage - v_be).max(0.1) / block_bias_resistance;
                            let gm = i_bias / vt;
                            (1.0 / gm).clamp(10.0, 10_000.0)
                        }
                    }
                };
                wdf.tree.set_vs_port_resistance(block_source_r);
                let diode_cutoff = match &wdf.root {
                    pedalkernel_rt::stage::RootKind::ExplicitSingleDiode(_) => {
                        Some(pedalkernel_rt::stage::DiodeCutoffCalibration {
                            bias_voltage: supply_voltage,
                            bias_resistance: block_bias_resistance,
                            cv_resistance: None,
                            min_rp: 10.0,
                            max_rp: 100_000.0,
                        })
                    }
                    _ => None,
                };

                // Generate K-table with the correct adapted tree impedance.
                // A diode-connected ladder block needs a bias-voltage axis:
                // the local reactive tree removes DC from b_tree, but the
                // nonlinear diode's small-signal conductance is set by the
                // raw coupling/cascade voltage.
                wdf.k_table = match &wdf.root {
                    pedalkernel_rt::stage::RootKind::ExplicitSingleDiode(root) => {
                        Some(super::k_method::generate_biased_single_diode_k_table(
                            root.model,
                            wdf.tree.port_resistance(),
                        ))
                    }
                    _ => super::k_method::generate_k_table(wdf),
                };

                if let Some(ref k_table) = wdf.k_table {
                    let vbe_bias = match &wdf.root {
                        pedalkernel_rt::stage::RootKind::Bjt(bjt) => bjt.vbe_bias(),
                        _ => 0.6,
                    };
                    // dc_offset initialized to the quiescent cascade voltage.
                    // Tracked at runtime via 1-pole LPF to extract the AC
                    // component for the K-table ctrl axis.
                    let mut dc_tree = wdf.tree.clone();
                    dc_tree.set_voltage(0.0);
                    let b0 = dc_tree.reflected();
                    let mut dc_kt = k_table.clone();
                    dc_kt.precompute_scales();
                    let a0 = if dc_kt.dims == 1 {
                        dc_kt.lookup_1d(b0)
                    } else {
                        dc_kt.lookup_2d(b0, 0.0)
                    };
                    dc_tree.set_incident(a0);
                    let dc_offset = wdf
                        .output_probe
                        .as_ref()
                        .and_then(|probe| dc_tree.leaf_voltage(probe))
                        .unwrap_or((a0 + b0) / 2.0);

                    #[cfg(test)]
                    eprintln!(
                        "  block {bi}: VS Rp={:.1}Ω, tree Rp={:.1}Ω",
                        block_source_r,
                        wdf.tree.port_resistance()
                    );
                    let root_polarity = match &wdf.root {
                        pedalkernel_rt::stage::RootKind::DiodePair(_)
                        | pedalkernel_rt::stage::RootKind::SingleDiode(_)
                        | pedalkernel_rt::stage::RootKind::ExplicitDiodePair(_)
                        | pedalkernel_rt::stage::RootKind::ExplicitSingleDiode(_)
                        | pedalkernel_rt::stage::RootKind::Zener(_) => -1.0,
                        _ => 1.0,
                    };
                    let source_polarity = if wdf.negate_vs {
                        -root_polarity
                    } else {
                        root_polarity
                    };
                    let k_table_control_polarity = match &wdf.root {
                        // The biased diode K-table axis is physical diode bias,
                        // not the voltage-source orientation used to drive the
                        // WDF tree.
                        pedalkernel_rt::stage::RootKind::ExplicitSingleDiode(_) => 1.0,
                        _ => source_polarity,
                    };
                    // The cascade tap is at the passive subtree (cap voltage),
                    // not the root port (diode residual). This is determined
                    // by the circuit: the cascade node (Q.emitter) is where
                    // the cap connects, which is the right child of the
                    // Series(VS, Cap) adaptor.
                    k_blocks.push(pedalkernel_rt::stage::KMethodBlock {
                        tree: wdf.tree.clone(),
                        k_table: k_table.clone(),
                        explicit_diode_root: match &wdf.root {
                            pedalkernel_rt::stage::RootKind::ExplicitSingleDiode(root) => {
                                Some(*root)
                            }
                            _ => None,
                        },
                        nominal_vs_rp: block_source_r,
                        diode_cutoff,
                        rp: wdf.tree.port_resistance(),
                        vbe_bias,
                        dc_offset,
                        cascade_from_passive: true,
                        cascade_probe_id: wdf.output_probe.clone(),
                        source_polarity,
                        k_table_control_polarity,
                    });
                }
            }
        }

        if force_serial {
            eprintln!(
                "  [blockwise] force_serial: returning {} rung stages without BKM coupling",
                all_stages.len()
            );
            return Some(all_stages);
        }

        let n_blocks = k_blocks.len();
        if n_blocks < 2 {
            // Not enough K-table blocks — fall back to separate stages
            eprintln!(
                "  [blockwise] only {n_blocks} K-table blocks, falling back to separate stages"
            );
            eprintln!("  [blockwise] done: {} total stages", all_stages.len());
            return Some(all_stages);
        }

        // ── Build MNA for coupling network ──────────────────────────────
        // Collect unique nodes from coupling edges.
        //
        // GND and AC ground are true ground references (MNA ground node).
        // VCC and supply nodes are NOT ground — they are DC voltage sources
        // that bias the circuit. They get included in the MNA as nodes,
        // and a VS port is added to inject the supply voltage.
        let mut node_set: Vec<NodeId> = Vec::new();
        let ground_rails: HashSet<NodeId> = {
            let mut r = HashSet::new();
            r.insert(graph.gnd_node);
            r.extend(&graph.ac_ground_nodes);
            r
        };
        // Track which supply nodes appear in the coupling (need VS ports)
        let mut supply_nodes_in_coupling: Vec<(NodeId, f64)> = Vec::new();

        for &eidx in &plan.coupling_edges {
            let e = &graph.edges[eidx];
            for &node in &[e.node_a, e.node_b] {
                if ground_rails.contains(&node) {
                    continue; // true ground — MNA reference
                }
                if !node_set.contains(&node) {
                    node_set.push(node);
                }
                // Track supply nodes that need VS ports
                if node == graph.vcc_node || graph.supply_nodes.contains(&node) {
                    if !supply_nodes_in_coupling.iter().any(|(n, _)| *n == node) {
                        supply_nodes_in_coupling.push((node, supply_voltage));
                    }
                }
            }
        }
        let node_to_mna: HashMap<NodeId, usize> =
            node_set.iter().enumerate().map(|(i, &n)| (n, i)).collect();
        let n_mna = node_set.len();

        eprintln!("  [blockwise] coupling MNA: {n_mna} nodes, {n_blocks} block ports");

        // Create MNA system and stamp coupling resistors
        let mut mna = pedalkernel_rt::tree::MnaSystem::new(n_mna, 0);
        let output_feedback_node = plan
            .blocks
            .last()
            .and_then(|block| block_cascade_output_node(&block.nl_edges, &block.port_nodes, graph));

        let mut coupling_elements = Vec::new();
        for &eidx in &plan.coupling_edges {
            let e = &graph.edges[eidx];
            let comp = &graph.components[e.comp_idx];
            let r = comp.kind.resistance().unwrap_or(10_000.0);
            let n1 = if ground_rails.contains(&e.node_a) {
                None
            } else {
                node_to_mna.get(&e.node_a).copied()
            };
            let n2 = if ground_rails.contains(&e.node_b) {
                None
            } else {
                node_to_mna.get(&e.node_b).copied()
            };
            mna.stamp_resistor(n1, n2, r);

            let pot_meta = comp
                .kind
                .as_any()
                .downcast_ref::<super::components::Potentiometer>()
                .map(|p| (p.max_r, p.taper));
            let invert_control = pot_meta.is_some()
                && output_feedback_node
                    .map(|node| e.node_a == node || e.node_b == node)
                    .unwrap_or(false);
            coupling_elements.push(pedalkernel_rt::stage::CouplingElement {
                comp_id: comp.id.clone(),
                node_a: n1,
                node_b: n2,
                resistance: r,
                pot_max_resistance: pot_meta.map(|(max_r, _)| max_r),
                taper: pot_meta
                    .map(|(_, taper)| taper)
                    .unwrap_or(pedalkernel_rt::pot_taper::PotTaper::B),
                invert_control,
            });
        }

        // GMIN regularization (prevent singular matrix)
        for i in 0..n_mna {
            mna.stamp_resistor(Some(i), None, 1e9);
        }

        // ── Define WDF ports ────────────────────────────────────────────
        // One port per block (at the block's coupling node) + one adapted VS port
        let mut ports = Vec::new();

        // Block ports: find each block's UNIQUE coupling node.
        //
        // For cascaded diode-connected BJTs (base=collector), adjacent blocks
        // share a node (Q1.emitter = Q2.base). Naively picking the first
        // port_node in the MNA can assign the same node to two blocks,
        // collapsing them in the scattering matrix.
        //
        // Heuristic: pick the port_node with the MOST coupling edges touching
        // it. This selects the base/collector node (where R_bias connects)
        // rather than the cascade junction.
        let mut used_ports: HashSet<NodeId> = HashSet::new();
        for (bi, block) in plan.blocks.iter().enumerate() {
            let preferred_node =
                block_coupling_port_node(&block.nl_edges, &block.port_nodes, graph)
                    .filter(|pn| node_to_mna.contains_key(pn) && !used_ports.contains(pn));

            let best_node = preferred_node.or_else(|| {
                block
                    .port_nodes
                    .iter()
                    .filter(|pn| node_to_mna.contains_key(pn) && !used_ports.contains(pn))
                    .max_by_key(|&&pn| {
                        plan.coupling_edges
                            .iter()
                            .filter(|&&eidx| {
                                let e = &graph.edges[eidx];
                                e.node_a == pn || e.node_b == pn
                            })
                            .count()
                    })
                    .copied()
            });

            if let Some(pn) = best_node {
                let mna_idx = node_to_mna[&pn];
                let rp = if bi < k_blocks.len() {
                    k_blocks[bi].nominal_vs_rp
                } else {
                    1000.0
                };
                ports.push(pedalkernel_rt::tree::WdfPort {
                    node_pos: Some(mna_idx),
                    node_neg: None,
                    resistance: rp,
                });
                used_ports.insert(pn);
                #[cfg(test)]
                eprintln!("    block {bi}: port_node=Some({pn}) → mna_node=Some({mna_idx})");
            } else {
                // Block has no unique node in coupling — use dummy
                ports.push(pedalkernel_rt::tree::WdfPort {
                    node_pos: None,
                    node_neg: None,
                    resistance: 1000.0,
                });
                #[cfg(test)]
                eprintln!("    block {bi}: port_node=None (no unique coupling node)");
            }
        }

        let mut feedback_port_map: Vec<(usize, usize)> = Vec::new();
        for (bi, block) in plan.blocks.iter().enumerate() {
            let Some(output_node) =
                block_cascade_output_node(&block.nl_edges, &block.port_nodes, graph)
            else {
                continue;
            };
            if used_ports.contains(&output_node) {
                continue;
            }
            if !plan.coupling_edges.iter().any(|&eidx| {
                let e = &graph.edges[eidx];
                e.node_a == output_node || e.node_b == output_node
            }) {
                continue;
            }
            let Some(&mna_idx) = node_to_mna.get(&output_node) else {
                continue;
            };
            let scattering_idx = ports.len();
            ports.push(pedalkernel_rt::tree::WdfPort {
                node_pos: Some(mna_idx),
                node_neg: None,
                resistance: r_source_cascade,
            });
            feedback_port_map.push((bi, scattering_idx));
            used_ports.insert(output_node);
            #[cfg(test)]
            eprintln!(
                "    feedback port block {bi}: node={output_node} → mna_node=Some({mna_idx}), scattering_port={scattering_idx}"
            );
        }

        // ── VS ports: one per input port that connects through a coupling edge ──
        //
        // The .pedal declares input ports (audio_in, cv_cutoff, etc.) with
        // net connections to components. If a port's component is a coupling
        // edge, we add a VS port at that edge's external node in the coupling
        // MNA. The scattering matrix distributes the port voltage through
        // the resistor network. At runtime, writing to the VS port is O(1)
        // — no recompute, just update b[vs_port] before the matrix multiply.
        let mut vs_port_map: Vec<(String, usize)> = Vec::new(); // (port_name, scattering_port_idx)

        // Collect all coupling edge nodes for quick lookup
        let coupling_edge_set: HashSet<usize> = plan.coupling_edges.iter().copied().collect();

        for port_def in port_defs {
            if port_def.direction != pedalkernel_rt::PortDirection::Input {
                continue;
            }
            // Resolve port name to graph node
            let port_node = graph.node_names.get(&port_def.name).copied();
            let port_node = match port_node {
                Some(n) => n,
                None => continue,
            };

            // Find the coupling edge connected to this port node.
            // The VS goes at the port node's end of the edge (the "external"
            // side that faces outside the coupling network).
            let edge_and_node = plan.coupling_edges.iter().find_map(|&eidx| {
                let e = &graph.edges[eidx];
                // The port node connects to one end of a coupling edge.
                // The VS port goes at the port node itself (the external side).
                if e.node_a == port_node || e.node_b == port_node {
                    // Find the port node in MNA. If it's a rail, use the other end.
                    let mna_node = node_to_mna.get(&port_node).copied();
                    if mna_node.is_some() {
                        return Some(mna_node);
                    }
                    // Port node is a rail/external — use the other end
                    let other = if e.node_a == port_node {
                        e.node_b
                    } else {
                        e.node_a
                    };
                    Some(node_to_mna.get(&other).copied())
                } else {
                    None
                }
            });

            if let Some(Some(mna_idx)) = edge_and_node {
                let rp = port_def.impedance.unwrap_or(1.0);
                let scattering_idx = ports.len();
                ports.push(pedalkernel_rt::tree::WdfPort {
                    node_pos: Some(mna_idx),
                    node_neg: None,
                    resistance: rp,
                });
                vs_port_map.push((port_def.name.clone(), scattering_idx));
                #[cfg(test)]
                eprintln!("    VS port '{}': mna_node={mna_idx}, Rp={rp:.0}Ω, scattering_port={scattering_idx}",
                    port_def.name);
            }
        }

        // Fallback: if no audio input port was found, add a default VS at graph.in_node
        if vs_port_map.is_empty() {
            let vs_node = node_to_mna.get(&graph.in_node).copied();
            ports.push(pedalkernel_rt::tree::WdfPort {
                node_pos: vs_node,
                node_neg: None,
                resistance: 1.0,
            });
            vs_port_map.push(("audio_in".to_string(), ports.len() - 1));
        }

        // Supply VS ports: VCC and other supply nodes that appear in coupling.
        // These establish DC bias current through R_bias → diodes.
        // Without them, R_bias grounds the block ports → diodes off → dead filter.
        for (supply_node, voltage) in &supply_nodes_in_coupling {
            if let Some(&mna_idx) = node_to_mna.get(supply_node) {
                let scattering_idx = ports.len();
                ports.push(pedalkernel_rt::tree::WdfPort {
                    node_pos: Some(mna_idx),
                    node_neg: None,
                    resistance: 1.0, // ideal supply
                });
                let name = format!("_supply_{}", supply_node);
                vs_port_map.push((name, scattering_idx));
                #[cfg(test)]
                eprintln!("    Supply VS: node={supply_node}, {voltage}V, scattering_port={scattering_idx}");
            }
        }

        let n_ports = ports.len();
        eprintln!("  [blockwise] coupling scattering: {n_ports} ports");

        // Derive scattering matrix
        let scattering = mna.derive_scattering_matrix_general(&ports);

        // Validate scattering matrix
        let all_finite = scattering.iter().all(|&v| v.is_finite());
        if !all_finite || scattering.len() != n_ports * n_ports {
            eprintln!("  [blockwise] WARNING: coupling scattering invalid, falling back");
            eprintln!("  [blockwise] done: {} total stages", all_stages.len());
            return Some(all_stages);
        }

        #[cfg(test)]
        {
            eprintln!("  [blockwise] scattering matrix ({n_ports}x{n_ports}):");
            for i in 0..n_ports {
                let row: Vec<f64> = (0..n_ports).map(|j| scattering[i * n_ports + j]).collect();
                eprintln!("    row {i}: {row:.4?}");
            }
        }

        let coupling_rp: Vec<f64> = ports.iter().map(|p| p.resistance).collect();

        // TODO: Compute per-block source impedance from circuit topology.
        //
        // The VS Rp = 1Ω (default) makes the source near-ideal, which
        // prevents the RC filter from working (gamma ≈ 0). The correct
        // source impedance comes from:
        // - Block 0: R_in from the DSL port impedance declaration
        // - Blocks 1..N: previous emitter follower output Z (~1/gm)
        //
        // The coupling scattering diagonal S[i][i] gives the TOTAL
        // Thevenin impedance including DC bias (R_bias → VCC), which
        // is wrong for AC source impedance. Need AC-only analysis.
        //
        // For now: use port impedance from DSL if available, else keep
        // default VS Rp = 1Ω. The DSL extension `audio_in: input(10k)`
        // is implemented and ready to use.

        // ── Package as BlockwiseKMethodStage ─────────────────────────────
        let output_block = n_blocks - 1; // Last block is the output
        let bkm = pedalkernel_rt::stage::BlockwiseKMethodStage {
            blocks: k_blocks,
            coupling_s: scattering,
            coupling_rp,
            coupling_n_mna: n_mna,
            coupling_ports: ports,
            coupling_elements,
            n_ports,
            output_block,
            supply_voltage,
            vs_port_map,
            cutoff_cv_port: None,
            feedback_port_map,
            compensation: 1.0,
            oversampler: pedalkernel_rt::oversampling::Oversampler::new(
                pedalkernel_rt::oversampling::OversamplingFactor::X1,
            ),
            // BKM stage should process BEFORE output coupling (C_out).
            // The feedback flow distance formula inflates distance for
            // op-amp cascades, but for ladders the feedback group IS the
            // primary signal path and should process first.
            signal_flow_distance: 0, // always first
            bypass_serial: false,
            b_warm: vec![0.0; n_ports],
            work_b: vec![0.0; n_ports],
            work_a: vec![0.0; n_ports],
            port_index_cache: Vec::new(),
        };

        eprintln!(
            "  [blockwise] built BlockwiseKMethodStage: {} blocks, {} ports",
            bkm.blocks.len(),
            bkm.n_ports
        );

        // Replace the individual block stages with the single BKM stage
        all_stages.clear();
        all_stages.push(BuiltStage::BlockwiseKMethod(bkm));
    }

    eprintln!("  [blockwise] done: {} total stages", all_stages.len());
    Some(all_stages)
}
