//! Blockwise decomposition of large nonlinear circuits.
//!
//! Detects when a monolithic multi-NL system can be split into smaller
//! chained blocks with sparse inter-block coupling. Each block becomes
//! its own WDF subtree with a local NL solve, and the blocks are connected
//! by a lightweight coupling network (K-method scattering or iteration).
//!
//! The algorithm:
//! 1. Find all NL components → group by shared circuit nodes (union-find)
//! 2. For each NL group → collect interior edges (linear + reactive)
//! 3. Edges touching multiple groups → coupling network
//! 4. Validate: ≥2 blocks, coupling is NL-free, each block has reactive state

use super::component::EdgeKind;
use super::graph::{CircuitGraph, NodeId};
use super::spqr::{spqr_decompose, spqr_to_stages};
use super::spqr_build::BuiltStage;
use std::collections::{HashMap, HashSet};

/// A block in the blockwise decomposition — one NL group + its local state.
#[derive(Debug)]
pub struct Block {
    /// NL edge indices (the nonlinear device junctions).
    pub nl_edges: Vec<usize>,
    /// Linear edge indices interior to this block (bias resistors, etc.).
    pub linear_edges: Vec<usize>,
    /// Reactive edge indices interior to this block (shunt caps, etc.).
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
    /// Coupling edges — the residual graph connecting blocks.
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
// Step 1: NL component grouping via union-find on shared nodes
// ═══════════════════════════════════════════════════════════════════════════

/// Simple union-find for grouping NL components by shared nodes.
struct UnionFind {
    parent: Vec<usize>,
    rank: Vec<usize>,
}

impl UnionFind {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
            rank: vec![0; n],
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
        if ra == rb {
            return;
        }
        if self.rank[ra] < self.rank[rb] {
            self.parent[ra] = rb;
        } else if self.rank[ra] > self.rank[rb] {
            self.parent[rb] = ra;
        } else {
            self.parent[rb] = ra;
            self.rank[ra] += 1;
        }
    }
}

/// Find NL edge groups: NL edges that share circuit nodes belong to the
/// same group. Ground and supply nodes are excluded from adjacency
/// (otherwise every NL component touching ground merges into one group).
///
/// Returns a map: group_id → Vec<nl_edge_index>.
fn group_nl_edges(
    edge_indices: &[usize],
    graph: &CircuitGraph,
) -> HashMap<usize, Vec<usize>> {
    // Collect NL edges
    let nl_edges: Vec<usize> = edge_indices
        .iter()
        .filter(|&&eidx| graph.effective_edge_kind(eidx) == EdgeKind::Nonlinear)
        .copied()
        .collect();

    if nl_edges.is_empty() {
        return HashMap::new();
    }

    // Nodes to exclude from adjacency: ground, supply, ac_ground
    let excluded_nodes: HashSet<NodeId> = {
        let mut s = HashSet::new();
        s.insert(graph.gnd_node);
        for &n in &graph.supply_nodes {
            s.insert(n);
        }
        for &n in &graph.ac_ground_nodes {
            s.insert(n);
        }
        s
    };

    // Union-find: merge NL edges that belong to the SAME component.
    // Two NL edges from the same BJT (B-E and C-E) share a comp_idx
    // and must be in the same block. But NL edges from different devices
    // that happen to share a circuit node (Q1.collector = Q2.base) must
    // NOT merge — that shared node is the inter-block coupling point.
    let mut uf = UnionFind::new(nl_edges.len());
    for i in 0..nl_edges.len() {
        for j in (i + 1)..nl_edges.len() {
            let ci = graph.edges[nl_edges[i]].comp_idx;
            let cj = graph.edges[nl_edges[j]].comp_idx;
            if ci == cj {
                uf.union(i, j);
            }
        }
    }

    // Collect groups
    let mut groups: HashMap<usize, Vec<usize>> = HashMap::new();
    for (i, &eidx) in nl_edges.iter().enumerate() {
        let root = uf.find(i);
        groups.entry(root).or_default().push(eidx);
    }

    groups
}

// ═══════════════════════════════════════════════════════════════════════════
// Step 2: Collect nodes for each NL group
// ═══════════════════════════════════════════════════════════════════════════

/// Collect all circuit nodes touched by a group's NL edges.
fn group_nodes(nl_edges: &[usize], graph: &CircuitGraph) -> HashSet<NodeId> {
    let mut nodes = HashSet::new();
    for &eidx in nl_edges {
        let e = &graph.edges[eidx];
        nodes.insert(e.node_a);
        nodes.insert(e.node_b);
    }
    nodes
}

// ═══════════════════════════════════════════════════════════════════════════
// Step 3: Assign non-NL edges to blocks or coupling
// ═══════════════════════════════════════════════════════════════════════════

/// Determine which NL group(s) each non-NL edge's endpoints belong to.
/// Returns: for each edge, the set of group IDs its endpoints touch.
///
/// An edge touching exactly one group → interior to that block.
/// An edge touching multiple groups → coupling network.
/// An edge touching no groups → coupling (linear glue between blocks).
fn classify_edge(
    eidx: usize,
    graph: &CircuitGraph,
    group_node_sets: &[(usize, HashSet<NodeId>)],
) -> HashSet<usize> {
    let e = &graph.edges[eidx];
    let mut touching_groups = HashSet::new();
    for &(gid, ref nodes) in group_node_sets {
        if nodes.contains(&e.node_a) || nodes.contains(&e.node_b) {
            touching_groups.insert(gid);
        }
    }
    touching_groups
}

// ═══════════════════════════════════════════════════════════════════════════
// Step 4: Build blocks and identify port nodes
// ═══════════════════════════════════════════════════════════════════════════

/// Build a Block from an NL group and its assigned interior edges.
fn build_block(
    nl_edges: Vec<usize>,
    interior_edges: &[usize],
    all_edges: &[usize],
    graph: &CircuitGraph,
) -> Block {
    let mut linear = Vec::new();
    let mut reactive = Vec::new();
    for &eidx in interior_edges {
        match graph.effective_edge_kind(eidx) {
            EdgeKind::Linear => linear.push(eidx),
            EdgeKind::Reactive => reactive.push(eidx),
            _ => {} // NL edges are in nl_edges already
        }
    }

    // Port nodes: nodes in this block that also touch edges NOT in the block
    let block_edge_set: HashSet<usize> = nl_edges
        .iter()
        .chain(linear.iter())
        .chain(reactive.iter())
        .copied()
        .collect();
    let block_nodes: HashSet<NodeId> = block_edge_set
        .iter()
        .flat_map(|&eidx| {
            let e = &graph.edges[eidx];
            vec![e.node_a, e.node_b]
        })
        .collect();

    let mut port_nodes = Vec::new();
    for &node in &block_nodes {
        // Skip ground/supply — they're implicit
        if node == graph.gnd_node || graph.supply_nodes.contains(&node) {
            continue;
        }
        let touches_outside = all_edges.iter().any(|&eidx| {
            if block_edge_set.contains(&eidx) {
                return false;
            }
            let e = &graph.edges[eidx];
            e.node_a == node || e.node_b == node
        });
        if touches_outside {
            port_nodes.push(node);
        }
    }

    Block {
        nl_edges,
        linear_edges: linear,
        reactive_edges: reactive,
        port_nodes,
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Step 5: Validation
// ═══════════════════════════════════════════════════════════════════════════

/// Check that a BlockwisePlan is well-formed:
/// - ≥2 blocks
/// - Each block has ≥1 NL edge
/// - Each block has ≥1 reactive edge (otherwise it's just a leaf NL root)
/// - Coupling network has no NL edges
fn validate_plan(
    plan: &BlockwisePlan,
    graph: &CircuitGraph,
) -> bool {
    if plan.blocks.len() < 2 {
        return false;
    }
    for block in &plan.blocks {
        if block.nl_edges.is_empty() {
            return false;
        }
        if block.reactive_edges.is_empty() {
            // A block without reactive state is just a memoryless NL root —
            // doesn't benefit from blockwise treatment.
            return false;
        }
    }
    // Coupling must not contain NL edges
    for &eidx in &plan.coupling_edges {
        if graph.effective_edge_kind(eidx) == EdgeKind::Nonlinear {
            return false;
        }
    }
    true
}

// ═══════════════════════════════════════════════════════════════════════════
// Public entry point
// ═══════════════════════════════════════════════════════════════════════════

/// Analyze a set of edges for blockwise decomposition.
///
/// Returns `Some(BlockwisePlan)` if the circuit can be split into ≥2
/// NL blocks with sparse coupling. Returns `None` if monolithic is
/// the only option.
pub(super) fn analyze_blockwise(
    edge_indices: &[usize],
    graph: &CircuitGraph,
) -> Option<BlockwisePlan> {
    // Step 1: group NL edges by shared nodes
    let nl_groups = group_nl_edges(edge_indices, graph);
    if nl_groups.len() < 2 {
        return None; // 0 or 1 NL groups → no decomposition
    }

    // Step 2: compute node sets for each group
    let group_node_sets: Vec<(usize, HashSet<NodeId>)> = nl_groups
        .iter()
        .map(|(&gid, nl_edges)| (gid, group_nodes(nl_edges, graph)))
        .collect();

    // Step 3: classify all non-NL edges
    let nl_edge_set: HashSet<usize> = nl_groups
        .values()
        .flat_map(|v| v.iter().copied())
        .collect();

    let mut group_interior_edges: HashMap<usize, Vec<usize>> = HashMap::new();
    let mut coupling_edges = Vec::new();

    for &eidx in edge_indices {
        if nl_edge_set.contains(&eidx) {
            continue; // NL edges are already assigned to groups
        }
        let touching = classify_edge(eidx, graph, &group_node_sets);
        if touching.len() == 1 {
            // Interior to exactly one group
            let gid = *touching.iter().next().unwrap();
            group_interior_edges.entry(gid).or_default().push(eidx);
        } else {
            // Touches 0 or 2+ groups → coupling
            coupling_edges.push(eidx);
        }
    }

    // Step 4: build blocks
    let mut blocks: Vec<Block> = Vec::new();
    for (&gid, nl_edges) in &nl_groups {
        let interior = group_interior_edges.get(&gid).map(|v| v.as_slice()).unwrap_or(&[]);
        blocks.push(build_block(
            nl_edges.clone(),
            interior,
            edge_indices,
            graph,
        ));
    }

    // Collect all port nodes
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

    // Step 5: validate
    if validate_plan(&plan, graph) {
        Some(plan)
    } else {
        None
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Build: convert a BlockwisePlan into BuiltStages
// ═══════════════════════════════════════════════════════════════════════════

/// Try blockwise decomposition and build. Returns `Some(stages)` if the
/// circuit decomposes into chained blocks, `None` to fall through to
/// the normal SPQR path.
///
/// Each block is SPQR-decomposed independently → small WDF trees.
/// Coupling edges are also SPQR-decomposed → passive stages.
pub(super) fn try_build_blockwise(
    edge_indices: &[usize],
    graph: &CircuitGraph,
    terminals: &[NodeId],
    sample_rate: f64,
) -> Option<Vec<BuiltStage>> {
    let plan = analyze_blockwise(edge_indices, graph)?;

    #[cfg(test)]
    eprintln!(
        "  Blockwise: {} blocks, {} coupling edges",
        plan.num_blocks(),
        plan.coupling_edges.len()
    );

    let mut all_stages = Vec::new();

    // Build each block via SPQR
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
        eprintln!(
            "  Block {bi}: {} edges → {} stages",
            block_edges.len(),
            spqr_stages.len()
        );

        for stage in spqr_stages {
            let built = super::spqr_build::build_spqr_stage(stage, graph, sample_rate)
                .ok()?; // If any block fails, fall through to monolithic
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

        #[cfg(test)]
        eprintln!(
            "  Coupling: {} edges → {} stages",
            plan.coupling_edges.len(),
            spqr_stages.len()
        );

        for stage in spqr_stages {
            let built = super::spqr_build::build_spqr_stage(stage, graph, sample_rate)
                .ok()?;
            all_stages.push(built);
        }
    }

    Some(all_stages)
}
