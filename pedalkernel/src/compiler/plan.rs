//! Pass 3: Unified stage planning.
//!
//! One loop over classified nonlinear elements. For each element, the same
//! algorithm with parameterized differences:
//! 1. Collect passive edges at junction node(s)
//! 2. Detect topology variants (source follower, feedback, push-pull)
//! 3. Find injection node (BFS-closest to input)
//! 4. Build virtual edges (if 3-terminal: r_ce for BJT, r_p for triode)
//! 5. Determine terminals

use std::collections::{HashMap, HashSet};

use crate::dsl::*;

use super::classify::{ClassifiedCircuit, NonlinearElement, NonlinearKind};
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
// Coupled BJT plan
// ═══════════════════════════════════════════════════════════════════════════

/// Plan for a tightly-coupled BJT pair (e.g., Fuzz Face).
///
/// Coupled BJTs share feedback paths (collector→base or shared emitter/collector
/// nodes via passives). They are processed as a single unit with 1-sample delay
/// feedback coupling, rather than as independent stages.
pub(super) struct CoupledBjtPlan {
    /// Element indices in signal-flow order (first BJT receives input).
    pub(super) bjt_element_indices: Vec<usize>,
    /// Index of the BJT that produces the final output.
    pub(super) output_bjt_idx: usize,
}

// ═══════════════════════════════════════════════════════════════════════════
// Multi-NL plan (R-type adaptor approach)
// ═══════════════════════════════════════════════════════════════════════════

/// Plan for coupled nonlinear elements using R-type adaptor + multi-port NR.
///
/// Instead of decoupling each NL element into its own WDF tree with 1-sample
/// feedback, this plan captures the full passive network connecting all coupled
/// NL elements as an MNA system, derives a scattering matrix, and solves
/// all NL ports simultaneously.
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
    /// When Some, the output is taken from a passive child port touching this node
    /// rather than from an NL port. Used for bridge rectifier + RC stages where
    /// the output is the smoothed DC voltage across a capacitor.
    pub(super) output_node: Option<NodeId>,
}

// ═══════════════════════════════════════════════════════════════════════════
// Planning
// ═══════════════════════════════════════════════════════════════════════════

/// Plan all WDF stages from classified nonlinear elements.
///
/// Returns a list of stage plans (one per nonlinear element that has passives),
/// a list of push-pull plans, a list of coupled BJT plans, and a list of
/// multi-NL plans (for R-type adaptor approach).
pub(super) fn plan_stages(
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> (Vec<StagePlan>, Vec<PushPullPlan>, Vec<CoupledBjtPlan>, Vec<MultiNlPlan>, HashSet<usize>) {
    // ── Push-pull detection for triodes ────────────────────────────────
    // Collect triode elements indices to detect push-pull pairs.
    let triode_elements: Vec<(usize, &NonlinearElement)> = classified
        .nonlinear_elements
        .iter()
        .enumerate()
        .filter(|(_, e)| matches!(&e.kind, NonlinearKind::Triode { .. }))
        .collect();

    // Build triode info for push-pull detection (needs TriodeInfo format).
    use super::graph::TriodeInfo;
    let triode_infos: Vec<(usize, TriodeInfo)> = triode_elements
        .iter()
        .map(|(_, elem)| {
            if let NonlinearKind::Triode { model_name, plate_node, cathode_node, parallel_count, is_vari_mu, .. } = &elem.kind {
                (elem.edge_idx, TriodeInfo {
                    model_name: model_name.clone(),
                    plate_node: *plate_node,
                    cathode_node: *cathode_node,
                    junction_node: *cathode_node,
                    ground_node: graph.gnd_node,
                    parallel_count: *parallel_count,
                    is_vari_mu: *is_vari_mu,
                })
            } else {
                unreachable!()
            }
        })
        .collect();

    let (push_pull_pairs, mut pp_transformer_edges) = graph.find_push_pull_triode_pairs(
        &triode_infos,
        &classified.all_nonlinear_edge_indices,
    );

    // Also detect Standard-type transformers driven push-pull (e.g., 670 T_sc_out).
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

    // Map from triode_infos index → classified element index.
    let triode_to_classified: Vec<usize> = triode_elements.iter().map(|(idx, _)| *idx).collect();

    let push_pull_plans: Vec<PushPullPlan> = push_pull_pairs
        .iter()
        .map(|p| PushPullPlan {
            push_triode_list_idx: triode_to_classified[p.push_triode_idx],
            pull_triode_list_idx: triode_to_classified[p.pull_triode_idx],
            transformer_edge_idx: p.transformer_edge_idx,
            turns_ratio: p.turns_ratio,
        })
        .collect();

    // ── BJT feedback detection ─────────────────────────────────────────
    // Collect all BJT base nodes for feedback path detection.
    let all_bjt_base_nodes: HashSet<NodeId> = classified
        .nonlinear_elements
        .iter()
        .filter_map(|e| match &e.kind {
            NonlinearKind::BjtNpn { base_node, .. }
            | NonlinearKind::BjtPnp { base_node, .. } => Some(*base_node),
            _ => None,
        })
        .collect();

    // ── Coupled BJT detection ────────────────────────────────────────
    // Collect all BJTs with their terminal nodes.
    struct BjtInfo {
        elem_idx: usize,
        base_node: NodeId,
        collector_node: NodeId,
        emitter_node: NodeId,
    }
    let bjt_infos: Vec<BjtInfo> = classified
        .nonlinear_elements
        .iter()
        .enumerate()
        .filter_map(|(idx, e)| match &e.kind {
            NonlinearKind::BjtNpn { base_node, .. }
            | NonlinearKind::BjtPnp { base_node, .. } => {
                Some(BjtInfo {
                    elem_idx: idx,
                    base_node: *base_node,
                    collector_node: e.junction_nodes[0],
                    emitter_node: e.junction_nodes[1],
                })
            }
            _ => None,
        })
        .collect();

    // Union-find for BJT coupling.
    let n_bjts = bjt_infos.len();
    let mut parent: Vec<usize> = (0..n_bjts).collect();
    fn find(parent: &mut Vec<usize>, mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }
    fn union(parent: &mut Vec<usize>, a: usize, b: usize) {
        let ra = find(parent, a);
        let rb = find(parent, b);
        if ra != rb {
            parent[rb] = ra;
        }
    }

    // Build coupling edges: BJT_i ↔ BJT_j if collector_i == base_j or
    // collector_j == base_i (DC feedback), or they share emitter/collector
    // nodes via passives.
    for i in 0..n_bjts {
        for j in (i + 1)..n_bjts {
            let bi = &bjt_infos[i];
            let bj = &bjt_infos[j];

            // Direct DC feedback: collector of one feeds base of other.
            let dc_coupled = bi.collector_node == bj.base_node
                || bj.collector_node == bi.base_node;

            // Shared emitter/collector nodes (via passives in between).
            let shared_nodes = bi.emitter_node == bj.emitter_node
                || bi.emitter_node == bj.collector_node
                || bi.collector_node == bj.emitter_node
                || bi.collector_node == bj.collector_node;

            if dc_coupled || shared_nodes {
                union(&mut parent, i, j);
            }
        }
    }

    // Group coupled BJTs into clusters.
    let mut clusters: HashMap<usize, Vec<usize>> = HashMap::new();
    for i in 0..n_bjts {
        let root = find(&mut parent, i);
        clusters.entry(root).or_default().push(i);
    }

    // Build CoupledBjtPlan + MultiNlPlan for clusters of 2+ BJTs.
    let mut coupled_bjt_plans: Vec<CoupledBjtPlan> = Vec::new();
    let mut multi_nl_plans: Vec<MultiNlPlan> = Vec::new();
    let mut coupled_bjt_elem_indices: HashSet<usize> = HashSet::new();

    for (_root, members) in &clusters {
        if members.len() < 2 {
            continue;
        }

        // Order by distance from input (signal-flow order).
        let mut ordered: Vec<usize> = members.clone();
        ordered.sort_by_key(|&m| {
            let info = &bjt_infos[m];
            classified.dist_from_in.get(&info.base_node).copied().unwrap_or(usize::MAX)
        });

        let elem_indices: Vec<usize> = ordered.iter().map(|&m| bjt_infos[m].elem_idx).collect();

        // Output BJT: the one closest to the output.
        let output_idx = *ordered.iter().min_by_key(|&&m| {
            let info = &bjt_infos[m];
            classified.dist_from_out.get(&info.collector_node).copied().unwrap_or(usize::MAX)
        }).unwrap();

        for &ei in &elem_indices {
            coupled_bjt_elem_indices.insert(ei);
        }

        coupled_bjt_plans.push(CoupledBjtPlan {
            bjt_element_indices: elem_indices.clone(),
            output_bjt_idx: bjt_infos[output_idx].elem_idx,
        });

        // Also build a MultiNlPlan for the R-type adaptor approach.
        // Collect all passive edges from all junction nodes of all members.
        let mut all_passive_edges: Vec<usize> = Vec::new();
        let mut all_junction_nodes: HashSet<NodeId> = HashSet::new();
        for &m in &ordered {
            let info = &bjt_infos[m];
            all_junction_nodes.insert(info.collector_node);
            all_junction_nodes.insert(info.emitter_node);
        }
        for &jn in &all_junction_nodes {
            let edges = graph.bfs_passive_edges(
                jn,
                &classified.all_nonlinear_edge_indices,
                &graph.active_edge_indices,
                true, // include supply-adjacent
                true, // skip_out_node
                &pp_transformer_edges,
            );
            for eidx in edges {
                if !all_passive_edges.contains(&eidx) {
                    all_passive_edges.push(eidx);
                }
            }
        }

        // Case B: inject primary edge for input transformers.
        let xfmr_inject = find_secondary_side_transformers(
            &all_passive_edges, graph, &pp_transformer_edges,
        );
        for eidx in xfmr_inject {
            if !all_passive_edges.contains(&eidx) {
                all_passive_edges.push(eidx);
            }
        }

        // Find injection node (BFS-closest to input among passive edge endpoints).
        let mut injection_node = graph.in_node;
        let mut best_dist = usize::MAX;
        for &eidx in &all_passive_edges {
            let e = &graph.edges[eidx];
            for node in [e.node_a, e.node_b] {
                if let Some(&d) = classified.dist_from_in.get(&node) {
                    if d < best_dist {
                        best_dist = d;
                        injection_node = node;
                    }
                }
            }
        }

        // NL terminals: (collector, emitter) for each BJT
        let nl_terminals: Vec<(NodeId, NodeId)> = ordered
            .iter()
            .map(|&m| {
                let info = &bjt_infos[m];
                (info.collector_node, info.emitter_node)
            })
            .collect();

        multi_nl_plans.push(MultiNlPlan {
            nl_element_indices: elem_indices,
            output_element_idx: bjt_infos[output_idx].elem_idx,
            passive_edge_indices: all_passive_edges,
            injection_node,
            nl_terminals,
            compensation: 1.0,
            output_node: None,
        });
    }

    // ── 3-port VariMu triode detection ──────────────────────────────
    // VariMu triodes with grid-side passives get a MultiNlPlan with
    // 2 NL ports: (grid, cathode) and (plate, cathode). The grid-side
    // passive network (threshold pot, bias resistors, etc.) becomes part
    // of the R-type adaptor, allowing the NR solver to see grid voltage
    // changes from pots.
    let mut three_port_triode_indices: HashSet<usize> = HashSet::new();

    for (elem_idx, elem) in classified.nonlinear_elements.iter().enumerate() {
        // Only VariMu triodes with a known grid node.
        if let NonlinearKind::Triode {
            is_vari_mu: true,
            grid_node: Some(grid_node),
            plate_node,
            cathode_node,
            model_name,
            ..
        } = &elem.kind
        {
            // Skip sidechain elements.
            if classified.sidechain_edge_set.contains(&elem.edge_idx) {
                continue;
            }

            // Skip push-pull paired triodes.
            let triode_list_idx = triode_elements
                .iter()
                .position(|(_, e)| std::ptr::eq(*e, elem));
            if let Some(tli) = triode_list_idx {
                if paired_triode_indices.contains(&tli) {
                    continue;
                }
            }

            // Check for grid-side passive edges.
            let grid_passives = graph.bfs_passive_edges(
                *grid_node,
                &classified.all_nonlinear_edge_indices,
                &graph.active_edge_indices,
                true,
                true, // skip_out_node
                &pp_transformer_edges,
            );

            if grid_passives.is_empty() {
                continue; // No grid-side passives — fall through to 2-port
            }

            // Collect plate + cathode passives too.
            let plate_passives = graph.bfs_passive_edges(
                *plate_node,
                &classified.all_nonlinear_edge_indices,
                &graph.active_edge_indices,
                true,
                true, // skip_out_node
                &pp_transformer_edges,
            );
            let cathode_passives = graph.bfs_passive_edges(
                *cathode_node,
                &classified.all_nonlinear_edge_indices,
                &graph.active_edge_indices,
                true,
                true, // skip_out_node
                &pp_transformer_edges,
            );

            // Merge all passive edges (grid + plate + cathode).
            let mut all_passive_edges = grid_passives.clone();
            extend_dedup(&mut all_passive_edges, &plate_passives);
            extend_dedup(&mut all_passive_edges, &cathode_passives);

            // Case B: inject primary edge for input transformers whose secondary
            // nodes are adjacent to the passive set.
            let xfmr_inject = find_secondary_side_transformers(
                &all_passive_edges, graph, &pp_transformer_edges,
            );
            extend_dedup(&mut all_passive_edges, &xfmr_inject);

            // Find injection node: BFS-closest to input among all passive edge endpoints,
            // excluding plate/cathode/grid nodes.
            let mut injection_node = graph.in_node;
            let mut best_dist = usize::MAX;
            for &eidx in &all_passive_edges {
                let e = &graph.edges[eidx];
                for node in [e.node_a, e.node_b] {
                    if node == *plate_node || node == *cathode_node || node == *grid_node {
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

            // NL terminals: port 0 = (grid, cathode), port 1 = (plate, cathode)
            let nl_terminals = vec![
                (*grid_node, *cathode_node),
                (*plate_node, *cathode_node),
            ];

            let _model = super::helpers::triode_model(model_name);
            let compensation = 0.35; // always VariMu in this branch

            multi_nl_plans.push(MultiNlPlan {
                nl_element_indices: vec![elem_idx],
                output_element_idx: elem_idx,
                passive_edge_indices: all_passive_edges,
                injection_node,
                nl_terminals,
                compensation,
                output_node: None,
            });

            three_port_triode_indices.insert(elem_idx);
        }
    }

    // ── Coupled diode detection (bridge rectifiers) ────────────────────
    // Diodes sharing terminal nodes are coupled (e.g., bridge rectifier).
    // Union-find mirrors the BJT coupling pass above.
    struct DiodeInfo {
        elem_idx: usize,
        node_a: NodeId, // anode
        node_b: NodeId, // cathode
    }
    let diode_infos: Vec<DiodeInfo> = classified
        .nonlinear_elements
        .iter()
        .enumerate()
        .filter_map(|(idx, e)| match &e.kind {
            NonlinearKind::SingleDiode(_) => {
                // Skip sidechain elements — they're compiled separately.
                if classified.sidechain_edge_set.contains(&e.edge_idx) {
                    return None;
                }
                let edge = &graph.edges[e.edge_idx];
                Some(DiodeInfo {
                    elem_idx: idx,
                    node_a: edge.node_a,
                    node_b: edge.node_b,
                })
            }
            _ => None,
        })
        .collect();

    let mut coupled_diode_indices: HashSet<usize> = HashSet::new();

    if diode_infos.len() >= 2 {
        // Union-find for diode coupling.
        let n_diodes = diode_infos.len();
        let mut d_parent: Vec<usize> = (0..n_diodes).collect();
        fn d_find(parent: &mut Vec<usize>, mut x: usize) -> usize {
            while parent[x] != x {
                parent[x] = parent[parent[x]];
                x = parent[x];
            }
            x
        }
        fn d_union(parent: &mut Vec<usize>, a: usize, b: usize) {
            let ra = d_find(parent, a);
            let rb = d_find(parent, b);
            if ra != rb {
                parent[rb] = ra;
            }
        }

        // Diodes sharing a non-global terminal node are coupled.
        // Exclude gnd and vcc — they are global hubs, not coupling points.
        let is_global = |n: NodeId| n == graph.gnd_node || n == graph.vcc_node;
        for i in 0..n_diodes {
            for j in (i + 1)..n_diodes {
                let di = &diode_infos[i];
                let dj = &diode_infos[j];
                let shared = (!is_global(di.node_a) && (di.node_a == dj.node_a || di.node_a == dj.node_b))
                    || (!is_global(di.node_b) && (di.node_b == dj.node_a || di.node_b == dj.node_b));
                if shared {
                    d_union(&mut d_parent, i, j);
                }
            }
        }

        // Group coupled diodes into clusters.
        let mut d_clusters: HashMap<usize, Vec<usize>> = HashMap::new();
        for i in 0..n_diodes {
            let root = d_find(&mut d_parent, i);
            d_clusters.entry(root).or_default().push(i);
        }

        for (_root, members) in &d_clusters {
            if members.len() < 2 {
                continue;
            }

            // Skip antiparallel pairs: if ALL diodes in the cluster connect the
            // same two nodes, they're antiparallel clipping diodes (e.g., Klon
            // Centaur). These work fine as independent WDF stages. Only couple
            // diodes that form a multi-node topology like a bridge rectifier.
            let all_diode_nodes: HashSet<NodeId> = members.iter()
                .flat_map(|&m| [diode_infos[m].node_a, diode_infos[m].node_b])
                .collect();
            if all_diode_nodes.len() <= 2 {
                continue; // All diodes share the same 2 nodes — antiparallel pair
            }

            let elem_indices: Vec<usize> = members.iter().map(|&m| diode_infos[m].elem_idx).collect();
            for &ei in &elem_indices {
                coupled_diode_indices.insert(ei);
            }

            // Collect all terminal nodes from coupled diodes.
            let mut all_diode_nodes: HashSet<NodeId> = HashSet::new();
            for &m in members {
                all_diode_nodes.insert(diode_infos[m].node_a);
                all_diode_nodes.insert(diode_infos[m].node_b);
            }

            // BFS passive edges from ALL terminal nodes, with skip_out_node=false
            // so that RC time constant edges touching out_node are collected.
            let mut all_passive_edges: Vec<usize> = Vec::new();
            for &dn in &all_diode_nodes {
                let edges = graph.bfs_passive_edges(
                    dn,
                    &classified.all_nonlinear_edge_indices,
                    &graph.active_edge_indices,
                    true,  // include supply-adjacent
                    false, // skip_out_node=false (bridge rectifier needs RC at output)
                    &pp_transformer_edges,
                );
                for eidx in edges {
                    if !all_passive_edges.contains(&eidx) {
                        all_passive_edges.push(eidx);
                    }
                }
            }

            // Case B: inject primary edge for input transformers.
            let xfmr_inject = find_secondary_side_transformers(
                &all_passive_edges, graph, &pp_transformer_edges,
            );
            for eidx in xfmr_inject {
                if !all_passive_edges.contains(&eidx) {
                    all_passive_edges.push(eidx);
                }
            }

            // Find injection node from diode terminal nodes (not passive
            // endpoints). The bridge's AC-side terminals have the correct
            // signal flow distance. Passive endpoints might reach back
            // toward the input through the RC network, giving a misleadingly
            // low distance.
            // Find injection node from diode terminal nodes. Use dist_from_in
            // when available; fall back to dist_from_out for nodes on the
            // disconnected side of transformers (secondary side).
            let mut injection_node = graph.in_node;
            let mut best_dist = usize::MAX;
            for &m in members {
                let di = &diode_infos[m];
                for node in [di.node_a, di.node_b] {
                    if node == graph.out_node || node == graph.gnd_node {
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
            // If no diode terminal is reachable from input (transformer barrier),
            // pick the terminal FARTHEST from output — this selects the AC
            // input side of the bridge (transformer secondary), not the DC
            // output side (RC time constant). Signal should be injected at
            // the AC side for correct rectification.
            if best_dist == usize::MAX {
                let mut best_out_dist = 0usize;
                for &m in members {
                    let di = &diode_infos[m];
                    for node in [di.node_a, di.node_b] {
                        if node == graph.out_node || node == graph.gnd_node {
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

            // NL terminals: (anode, cathode) for each diode.
            let nl_terminals: Vec<(NodeId, NodeId)> = members
                .iter()
                .map(|&m| (diode_infos[m].node_a, diode_infos[m].node_b))
                .collect();

            // Output element: diode closest to output.
            let output_elem_idx = *members
                .iter()
                .min_by_key(|&&m| {
                    let di = &diode_infos[m];
                    let d_a = classified.dist_from_out.get(&di.node_a).copied().unwrap_or(usize::MAX);
                    let d_b = classified.dist_from_out.get(&di.node_b).copied().unwrap_or(usize::MAX);
                    d_a.min(d_b)
                })
                .unwrap();

            // Check if any passive edge touches out_node — if so, output is a passive port.
            let output_node = if all_passive_edges.iter().any(|&eidx| {
                let e = &graph.edges[eidx];
                e.node_a == graph.out_node || e.node_b == graph.out_node
            }) {
                Some(graph.out_node)
            } else {
                None
            };

            multi_nl_plans.push(MultiNlPlan {
                nl_element_indices: elem_indices,
                output_element_idx: diode_infos[output_elem_idx].elem_idx,
                passive_edge_indices: all_passive_edges,
                injection_node,
                nl_terminals,
                compensation: 1.0,
                output_node,
            });
        }
    }

    // ── Plan each nonlinear element ────────────────────────────────────
    let mut plans: Vec<StagePlan> = Vec::new();
    let mut source_node_offset = 1000usize;

    for (elem_idx, elem) in classified.nonlinear_elements.iter().enumerate() {
        // Skip sidechain elements.
        if classified.sidechain_edge_set.contains(&elem.edge_idx) {
            continue;
        }

        // Skip push-pull paired triodes (handled separately).
        if let NonlinearKind::Triode { .. } = &elem.kind {
            let triode_list_idx = triode_elements
                .iter()
                .position(|(_, e)| std::ptr::eq(*e, elem));
            if let Some(tli) = triode_list_idx {
                if paired_triode_indices.contains(&tli) {
                    continue;
                }
            }
        }

        // Skip coupled BJTs (handled as CoupledBjtStage).
        if coupled_bjt_elem_indices.contains(&elem_idx) {
            continue;
        }

        // Skip 3-port VariMu triodes (handled as MultiNlStage).
        if three_port_triode_indices.contains(&elem_idx) {
            continue;
        }

        // Skip coupled diodes (handled as MultiNlStage bridge rectifier).
        if coupled_diode_indices.contains(&elem_idx) {
            continue;
        }

        let plan_result = plan_single_stage(
            elem,
            elem_idx,
            classified,
            graph,
            &all_bjt_base_nodes,
            source_node_offset,
            sample_rate,
        );
        #[cfg(feature = "debug-trace")]
        {
            let kind_name = match &elem.kind {
                NonlinearKind::Pentode { .. } => "Pentode",
                NonlinearKind::Triode { .. } => "Triode",
                _ => "Other",
            };
            eprintln!("[plan] elem[{elem_idx}] {kind_name} edge={} junctions={:?} -> {}",
                elem.edge_idx, elem.junction_nodes,
                if plan_result.is_some() { "OK" } else { "None" });
        }
        if let Some(plan) = plan_result {
            plans.push(plan);
        }

        source_node_offset += 1000;
    }

    (plans, push_pull_plans, coupled_bjt_plans, multi_nl_plans, pp_transformer_edges)
}

/// Plan a push-pull half (for building in build.rs).
pub(super) fn plan_push_pull_half(
    elem: &NonlinearElement,
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    pp_transformer_edges: &HashSet<usize>,
) -> Option<StagePlan> {
    if let NonlinearKind::Triode { plate_node, cathode_node, model_name, is_vari_mu, .. } = &elem.kind {
        // Multi-hop BFS: collect all passive edges reachable from plate/cathode,
        // stopping at transformers, gnd, and vcc.
        // include_supply_adjacent=true so edges to supply nodes (A_bal, B+, etc.)
        // ARE collected as plate load / cathode bias paths, but BFS doesn't
        // continue through them (they're boundary terminations).
        let plate_passives = graph.bfs_passive_edges(
            *plate_node,
            &classified.all_nonlinear_edge_indices,
            &graph.active_edge_indices,
            true, // include_supply_adjacent
            true, // skip_out_node
            pp_transformer_edges,
        );
        let cathode_passives = graph.bfs_passive_edges(
            *cathode_node,
            &classified.all_nonlinear_edge_indices,
            &graph.active_edge_indices,
            true, // include_supply_adjacent
            true, // skip_out_node
            pp_transformer_edges,
        );

        let mut passive_idxs: Vec<usize> = plate_passives.clone();
        extend_dedup(&mut passive_idxs, &cathode_passives);

        if passive_idxs.is_empty() {
            return None;
        }

        // Find injection node: scan ALL nodes across ALL collected plate passive
        // edges (multi-hop), pick the node with minimum dist_from_in, excluding
        // plate_node and cathode_node.
        let mut injection_node = graph.gnd_node;
        let mut best_dist = usize::MAX;
        for &eidx in &plate_passives {
            let e = &graph.edges[eidx];
            for candidate in [e.node_a, e.node_b] {
                if candidate == *plate_node || candidate == *cathode_node {
                    continue;
                }
                if let Some(&d) = classified.dist_from_in.get(&candidate) {
                    if d < best_dist {
                        best_dist = d;
                        injection_node = candidate;
                    }
                }
            }
        }
        // Fallback: try cathode passive edges if plate side yielded nothing.
        if best_dist == usize::MAX {
            for &eidx in &cathode_passives {
                let e = &graph.edges[eidx];
                for candidate in [e.node_a, e.node_b] {
                    if candidate == *plate_node || candidate == *cathode_node {
                        continue;
                    }
                    if let Some(&d) = classified.dist_from_in.get(&candidate) {
                        if d < best_dist {
                            best_dist = d;
                            injection_node = candidate;
                        }
                    }
                }
            }
        }
        // Last resort: first non-special node from plate passives.
        if best_dist == usize::MAX {
            for &eidx in &plate_passives {
                let e = &graph.edges[eidx];
                for candidate in [e.node_a, e.node_b] {
                    if candidate != graph.gnd_node && candidate != *cathode_node && candidate != *plate_node {
                        injection_node = candidate;
                        break;
                    }
                }
                if injection_node != graph.gnd_node { break; }
            }
        }

        // Ground terminal for push-pull halves: build degree map across
        // cathode passive edges, find a degree-1 leaf that isn't a special node.
        let mut ground_terminal = graph.gnd_node;
        {
            let mut degree: std::collections::HashMap<NodeId, usize> = std::collections::HashMap::new();
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

        // Push-pull halves are simple series chains (VS → plate_load → rp → cathode_bias → gnd).
        // Supply nodes (A_bal, B+) must NOT be terminals — that creates a 3-terminal T-network
        // that isn't SP-reducible. Instead, supply voltage is applied via CathodeBiasSource
        // leaves in build_push_pull_half (supply_leaf_voltages).
        let terminals = vec![source_node, ground_terminal];

        let model = super::helpers::triode_model(model_name);
        // For standard triodes, compensation scales by mu (e.g. 12AX7 mu=100 → 1.0).
        // For vari_mu tubes, compensation is the input transformer's voltage gain
        // per push-pull grid. The input transformer is not modeled as a WDF stage
        // (it's a passive coupling network), so its gain is folded into the
        // compensation factor here.
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
            element_idx: 0, // Not used for push-pull
            dc_block: None,
            compensation,
        })
    } else {
        None
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Single-element planning
// ═══════════════════════════════════════════════════════════════════════════

fn plan_single_stage(
    elem: &NonlinearElement,
    elem_idx: usize,
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    all_bjt_base_nodes: &HashSet<NodeId>,
    source_node_offset: usize,
    sample_rate: f64,
) -> Option<StagePlan> {
    match &elem.kind {
        // ── Simple 1-junction elements ─────────────────────────────────
        NonlinearKind::DiodePair(_) | NonlinearKind::SingleDiode(_) |
        NonlinearKind::Mosfet { .. } |
        NonlinearKind::Zener { .. } | NonlinearKind::Ota => {
            plan_simple_stage(elem, elem_idx, classified, graph, source_node_offset)
        }

        // ── Pentode (2-junction, triode-style planning) ────────────────
        NonlinearKind::Pentode { .. } => {
            plan_triode_stage(elem, elem_idx, classified, graph, source_node_offset, sample_rate)
        }

        // ── JFET (source follower detection) ───────────────────────────
        NonlinearKind::Jfet { .. } => {
            plan_jfet_stage(elem, elem_idx, classified, graph, source_node_offset)
        }

        // ── BJT (feedback detection, 2-junction) ───────────────────────
        NonlinearKind::BjtNpn { .. } | NonlinearKind::BjtPnp { .. } => {
            plan_bjt_stage(elem, elem_idx, classified, graph, all_bjt_base_nodes, source_node_offset)
        }

        // ── Triode (2-junction, DC block, push-pull) ───────────────────
        NonlinearKind::Triode { .. } => {
            plan_triode_stage(elem, elem_idx, classified, graph, source_node_offset, sample_rate)
        }
    }
}

/// Plan a simple 1-junction nonlinear stage (diode, pentode, MOSFET, zener, OTA).
fn plan_simple_stage(
    elem: &NonlinearElement,
    elem_idx: usize,
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    source_node_offset: usize,
) -> Option<StagePlan> {
    let junction = elem.junction_nodes[0];
    let passive_idxs = graph.elements_at_junction(
        junction,
        &classified.all_nonlinear_edge_indices,
        &graph.active_edge_indices,
    );

    if passive_idxs.is_empty() {
        return None;
    }

    let injection_node = find_injection_node(
        &passive_idxs,
        junction,
        &classified.dist_from_in,
        graph,
    );

    let source_node = graph.edges.len() + source_node_offset;
    let compensation = match &elem.kind {
        NonlinearKind::Ota => 0.08, // OTA feedback compensation
        _ => 1.0,
    };

    Some(StagePlan {
        passive_idxs,
        injection_node,
        terminals: vec![source_node, junction],
        source_node,
        virtual_edge: None,
        skip_vs: false,
        element_idx: elem_idx,
        dc_block: None,
        compensation,
    })
}

/// Plan a JFET stage with source follower detection.
fn plan_jfet_stage(
    elem: &NonlinearElement,
    elem_idx: usize,
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    source_node_offset: usize,
) -> Option<StagePlan> {
    let junction = elem.junction_nodes[0];

    let junction_passives = graph.elements_at_junction(
        junction,
        &classified.all_nonlinear_edge_indices,
        &graph.active_edge_indices,
    );

    // Find elements connecting junction to output (source follower detection).
    let junction_to_output: Vec<usize> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(idx, e)| {
            if classified.all_nonlinear_edge_indices.contains(idx) { return false; }
            if graph.active_edge_indices.contains(idx) { return false; }
            (e.node_a == junction && e.node_b == graph.out_node)
                || (e.node_a == graph.out_node && e.node_b == junction)
        })
        .map(|(idx, _)| idx)
        .collect();

    let output_passives = graph.elements_at_junction(
        graph.out_node,
        &classified.all_nonlinear_edge_indices,
        &graph.active_edge_indices,
    );

    let mut passive_idxs: Vec<usize> = junction_passives.clone();
    extend_dedup(&mut passive_idxs, &junction_to_output);
    extend_dedup(&mut passive_idxs, &output_passives);

    if passive_idxs.is_empty() {
        return None;
    }

    let is_source_follower = !junction_to_output.is_empty() || junction == graph.out_node;

    if is_source_follower {
        // Source follower: no voltage source, terminals = [gnd, junction].
        Some(StagePlan {
            passive_idxs,
            injection_node: graph.gnd_node,
            terminals: vec![graph.gnd_node, junction],
            source_node: 0, // Not used
            virtual_edge: None,

            skip_vs: true,
            element_idx: elem_idx,
            dc_block: None,
            compensation: 1.0,
        })
    } else {
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
        })
    }
}

/// Plan a BJT stage (simplified, matching triode pattern).
pub(super) fn plan_bjt_stage(
    elem: &NonlinearElement,
    elem_idx: usize,
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    _all_bjt_base_nodes: &HashSet<NodeId>,
    source_node_offset: usize,
) -> Option<StagePlan> {
    let (collector_node, emitter_node) = (elem.junction_nodes[0], elem.junction_nodes[1]);

    let collector_passives = graph.elements_at_junction(
        collector_node,
        &classified.all_nonlinear_edge_indices,
        &graph.active_edge_indices,
    );

    let emitter_passives = graph.elements_at_junction(
        emitter_node,
        &classified.all_nonlinear_edge_indices,
        &graph.active_edge_indices,
    );

    // Collector-to-output edges.
    let collector_to_output: Vec<usize> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(idx, e)| {
            if classified.all_nonlinear_edge_indices.contains(idx) { return false; }
            if graph.active_edge_indices.contains(idx) { return false; }
            (e.node_a == collector_node && e.node_b == graph.out_node)
                || (e.node_a == graph.out_node && e.node_b == collector_node)
        })
        .map(|(idx, _)| idx)
        .collect();

    // Output passives.
    let output_passives = graph.elements_at_junction(
        graph.out_node,
        &classified.all_nonlinear_edge_indices,
        &graph.active_edge_indices,
    );

    // Combine passive sets (matching triode pattern: always include all).
    let mut passive_idxs: Vec<usize> = collector_passives.clone();
    extend_dedup(&mut passive_idxs, &emitter_passives);
    extend_dedup(&mut passive_idxs, &collector_to_output);
    extend_dedup(&mut passive_idxs, &output_passives);

    if passive_idxs.is_empty() {
        return None;
    }

    // Find injection node.
    let mut injection_node = graph.in_node;
    let mut best_dist = usize::MAX;

    for &eidx in &collector_passives {
        let e = &graph.edges[eidx];
        let other = if e.node_a == collector_node { e.node_b } else { e.node_a };
        if let Some(&d) = classified.dist_from_in.get(&other) {
            if d < best_dist { best_dist = d; injection_node = other; }
        }
    }
    for &eidx in emitter_passives.iter().chain(collector_to_output.iter()).chain(output_passives.iter()) {
        let e = &graph.edges[eidx];
        for node in [e.node_a, e.node_b] {
            if node != collector_node && node != emitter_node {
                if let Some(&d) = classified.dist_from_in.get(&node) {
                    if d < best_dist { best_dist = d; injection_node = node; }
                }
            }
        }
    }
    if best_dist == usize::MAX {
        let mut found_supply = false;
        for &eidx in &collector_passives {
            let e = &graph.edges[eidx];
            if graph.supply_nodes.contains(&e.node_a) || graph.supply_nodes.contains(&e.node_b) {
                injection_node = if graph.supply_nodes.contains(&e.node_a) { e.node_a } else { e.node_b };
                found_supply = true;
                break;
            }
        }
        if !found_supply {
            injection_node = graph.gnd_node;
        }
    }

    let source_node = graph.edges.len() + source_node_offset;

    // Compute r_ce from Early voltage: r_ce = Va / Ic at quiescent Ic ≈ 1mA.
    let r_ce = match &elem.kind {
        NonlinearKind::BjtNpn { model_name, .. } | NonlinearKind::BjtPnp { model_name, .. } => {
            let model = crate::elements::BjtModel::by_name(model_name);
            model.va * 1000.0 // Va / 1mA
        }
        _ => 50000.0, // Fallback
    };

    Some(StagePlan {
        passive_idxs,
        injection_node,
        terminals: vec![source_node, graph.gnd_node],
        source_node,
        virtual_edge: Some(VirtualEdge {
            node_a: collector_node,
            node_b: emitter_node,
            resistance: r_ce,
            name: "__bjt_rce__",
        }),
        skip_vs: false,
        element_idx: elem_idx,
        dc_block: None,
        compensation: 1.0,
    })
}

/// Plan a triode stage with DC-block detection.
fn plan_triode_stage(
    elem: &NonlinearElement,
    elem_idx: usize,
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    source_node_offset: usize,
    sample_rate: f64,
) -> Option<StagePlan> {
    let (plate_node, cathode_node) = (elem.junction_nodes[0], elem.junction_nodes[1]);

    // Collect passives directly adjacent to plate and cathode junctions.
    let plate_passives = graph.elements_at_junction(
        plate_node,
        &classified.all_nonlinear_edge_indices,
        &graph.active_edge_indices,
    );

    // Additionally collect plate load edges to named supply rails (vcc_sc, etc.)
    // that elements_at_junction() excludes. These are plate load resistors
    // needed in the WDF tree — supply rails are AC ground (zero impedance),
    // so the plate load to vcc_sc is equivalent to a plate load to ground.
    let plate_supply_edges: Vec<usize> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(idx, e)| {
            if classified.all_nonlinear_edge_indices.contains(idx) {
                return false;
            }
            if graph.active_edge_indices.contains(idx) {
                return false;
            }
            if e.node_a != plate_node && e.node_b != plate_node {
                return false;
            }
            let other = if e.node_a == plate_node { e.node_b } else { e.node_a };
            graph.supply_nodes.contains(&other)
        })
        .map(|(idx, _)| idx)
        .collect();

    // For grounded-cathode tubes (cathode_node == gnd_node), return empty
    // to avoid collecting the entire ground network as "cathode passives."
    let cathode_passives = if cathode_node == graph.gnd_node {
        Vec::new()
    } else {
        graph.elements_at_junction(
            cathode_node,
            &classified.all_nonlinear_edge_indices,
            &graph.active_edge_indices,
        )
    };

    // Plate-to-output edges.
    let plate_to_output: Vec<usize> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(idx, e)| {
            if classified.all_nonlinear_edge_indices.contains(idx) { return false; }
            if graph.active_edge_indices.contains(idx) { return false; }
            (e.node_a == plate_node && e.node_b == graph.out_node)
                || (e.node_a == graph.out_node && e.node_b == plate_node)
        })
        .map(|(idx, _)| idx)
        .collect();

    let is_pentode = matches!(&elem.kind, NonlinearKind::Pentode { .. });

    // For pentodes: only collect output passives if the tube connects to
    // out_node. Without a plate_to_output edge, the output load belongs to a
    // different stage (e.g., bridge rectifier) and contaminates the passive set.
    // For triodes: always collect output passives (original behavior).
    let output_passives = if is_pentode && plate_to_output.is_empty() {
        Vec::new()
    } else {
        graph.elements_at_junction(
            graph.out_node,
            &classified.all_nonlinear_edge_indices,
            &graph.active_edge_indices,
        )
    };

    let mut passive_idxs: Vec<usize> = plate_passives.clone();
    extend_dedup(&mut passive_idxs, &plate_supply_edges);
    extend_dedup(&mut passive_idxs, &cathode_passives);
    extend_dedup(&mut passive_idxs, &plate_to_output);
    extend_dedup(&mut passive_idxs, &output_passives);

    // Detect transformers reachable from plate passives (1 extra hop).
    // For push-pull pentodes: plate → R_plate → transformer.primary.
    let xfmr_inject = find_plate_transformers(
        &plate_passives, plate_node, graph,
    );
    extend_dedup(&mut passive_idxs, &xfmr_inject);

    // If we found plate-side transformers, also check for secondary-side edges.
    if !xfmr_inject.is_empty() {
        let sec_inject = find_secondary_side_transformers(
            &passive_idxs, graph, &HashSet::new(),
        );
        extend_dedup(&mut passive_idxs, &sec_inject);
    }

    if passive_idxs.is_empty() {
        return None;
    }

    // Find injection node.
    // Pentodes: check both plate and cathode passives, exclude transformer
    // nodes (they create parallel VS/transformer topologies that cause
    // numerical issues).
    // Triodes: original behavior — prefer plate-side, then cathode-side.
    let mut injection_node = graph.in_node;
    let mut best_dist = usize::MAX;

    if is_pentode {
        let is_transformer_node = |node: NodeId| -> bool {
            graph.transformer_info.contains_key(&node)
        };
        for &eidx in plate_passives.iter().chain(cathode_passives.iter()) {
            let e = &graph.edges[eidx];
            for node in [e.node_a, e.node_b] {
                if node == plate_node || node == cathode_node { continue; }
                if is_transformer_node(node) { continue; }
                if let Some(&d) = classified.dist_from_in.get(&node) {
                    if d < best_dist { best_dist = d; injection_node = node; }
                }
            }
        }
    } else {
        // Original triode injection node search: plate-side first.
        for &eidx in &plate_passives {
            let e = &graph.edges[eidx];
            let other = if e.node_a == plate_node { e.node_b } else { e.node_a };
            if let Some(&d) = classified.dist_from_in.get(&other) {
                if d < best_dist { best_dist = d; injection_node = other; }
            }
        }
        if best_dist == usize::MAX {
            for &eidx in &cathode_passives {
                let e = &graph.edges[eidx];
                let other = if e.node_a == cathode_node { e.node_b } else { e.node_a };
                if let Some(&d) = classified.dist_from_in.get(&other) {
                    if d < best_dist { best_dist = d; injection_node = other; }
                }
            }
        }
    }
    if best_dist == usize::MAX {
        let mut found_supply = false;
        for &eidx in &plate_passives {
            let e = &graph.edges[eidx];
            let other = if e.node_a == plate_node { e.node_b } else { e.node_a };
            if other != graph.gnd_node && other != cathode_node {
                injection_node = other;
                found_supply = true;
                break;
            }
        }
        if !found_supply {
            injection_node = graph.gnd_node;
        }
    }

    // DC-block detection (coupling cap + load resistor).
    let dc_block = {
        let c_out = plate_to_output.iter().find_map(|&idx| {
            let e = &graph.edges[idx];
            match &graph.components[e.comp_idx].kind {
                ComponentKind::Capacitor(cap_cfg) => Some(cap_cfg.value),
                _ => None,
            }
        });
        let r_load = output_passives.iter().find_map(|&idx| {
            let e = &graph.edges[idx];
            match &graph.components[e.comp_idx].kind {
                ComponentKind::Resistor(r) => Some(*r),
                _ => None,
            }
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
    };

    // Compensation and plate resistance from triode/pentode model.
    let (compensation, rp) = match &elem.kind {
        NonlinearKind::Triode { model_name, is_vari_mu, .. } => {
            let model = super::helpers::triode_model(model_name);
            let comp = if *is_vari_mu { 0.35 } else { 1.0 };
            (comp, model.rp)
        }
        NonlinearKind::Pentode { model_name } => {
            let model = super::helpers::pentode_model(model_name);
            (1.0, model.rp)
        }
        _ => (1.0, 62500.0),
    };

    let source_node = graph.edges.len() + source_node_offset;

    Some(StagePlan {
        passive_idxs,
        injection_node,
        terminals: vec![source_node, graph.gnd_node],
        source_node,
        virtual_edge: Some(VirtualEdge {
            node_a: plate_node,
            node_b: cathode_node,
            resistance: rp,
            name: "__triode_rp__",
        }),
        skip_vs: false,
        element_idx: elem_idx,
        dc_block,
        compensation,
    })
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

/// Find the input transformer voltage gain for a push-pull pair.
///
/// Scans the circuit for Standard-winding (non-CT/PushPull) transformers
/// that are on the audio input path (not in the sidechain). Returns the
/// per-grid voltage gain: `(1/turns_ratio) / 2` because each push-pull
/// grid sees half the secondary swing.
///
/// Falls back to 1.0 if no input transformer is found.
fn find_input_transformer_gain_pp(
    graph: &CircuitGraph,
    classified: &ClassifiedCircuit,
) -> f64 {
    let mut best_gain = None;
    let mut best_dist = usize::MAX;

    for (_edge_idx, edge) in graph.edges.iter().enumerate() {
        let comp = &graph.components[edge.comp_idx];
        if let ComponentKind::Transformer(cfg) = &comp.kind {
            // Skip output transformers (CT or PushPull primary).
            if matches!(cfg.primary_type, WindingType::CenterTap | WindingType::PushPull) {
                continue;
            }

            // Pick the Standard transformer closest to the input.
            // Distance-based selection naturally prefers the input transformer
            // over sidechain transformers (which are further from input).
            let dist_a = classified.dist_from_in.get(&edge.node_a).copied().unwrap_or(usize::MAX);
            let dist_b = classified.dist_from_in.get(&edge.node_b).copied().unwrap_or(usize::MAX);
            let min_dist = dist_a.min(dist_b);

            if min_dist < best_dist {
                best_dist = min_dist;
                // Voltage gain = Ns/Np = 1/turns_ratio.
                // Push-pull halving: each grid sees half the secondary.
                best_gain = Some((1.0 / cfg.turns_ratio) / 2.0);

                #[cfg(feature = "debug-trace")]
                eprintln!(
                    "[TX-COMP] input transformer: id={} ratio={:.4} gain={:.2} dist={min_dist}",
                    comp.id, cfg.turns_ratio, best_gain.unwrap(),
                );
            }
        }
    }

    best_gain.unwrap_or(1.0)
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
        let other = if e.node_a == junction { e.node_b } else { e.node_a };
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

/// Find transformer edges within 1 extra hop from plate passives.
///
/// For push-pull pentodes, the plate load path is:
///   plate → R_plate → transformer.primary
/// `elements_at_junction` only collects direct adjacencies (R_plate), so the
/// transformer edge is missed. This helper walks far nodes of plate passives
/// and picks up any adjacent transformer edges.
fn find_plate_transformers(
    plate_passives: &[usize],
    plate_node: NodeId,
    graph: &CircuitGraph,
) -> Vec<usize> {
    let mut result = Vec::new();
    for &eidx in plate_passives {
        let e = &graph.edges[eidx];
        let far_node = if e.node_a == plate_node { e.node_b } else { e.node_a };
        for (idx, edge) in graph.edges.iter().enumerate() {
            if edge.node_a != far_node && edge.node_b != far_node {
                continue;
            }
            if matches!(graph.components[edge.comp_idx].kind, ComponentKind::Transformer(_)) {
                if !plate_passives.contains(&idx) && !result.contains(&idx) {
                    result.push(idx);
                }
            }
        }
    }
    result
}

/// Detect transformer secondary nodes in a stage's passive set and inject
/// the corresponding primary edge (Case B: input transformers).
///
/// When a stage's passive endpoints include a transformer's secondary-side
/// nodes, the primary edge (pri.a ↔ pri.b) should be added to the passive
/// set so that `sp_reduce` sees the transformer edge and `make_leaf` builds
/// a real transformer adaptor.
///
/// Returns the transformer edge indices to add (may be empty).
fn find_secondary_side_transformers(
    passive_idxs: &[usize],
    graph: &CircuitGraph,
    pp_transformer_edges: &HashSet<usize>,
) -> Vec<usize> {
    let mut inject_edges: Vec<usize> = Vec::new();

    // Collect all nodes touched by the passive set.
    let passive_nodes: HashSet<NodeId> = passive_idxs.iter()
        .flat_map(|&eidx| {
            let e = &graph.edges[eidx];
            [e.node_a, e.node_b]
        })
        .collect();

    // Check each node for secondary-side transformer info.
    let mut seen_comp_idx: HashSet<usize> = HashSet::new();
    for &node in &passive_nodes {
        if let Some(info) = graph.transformer_info.get(&node) {
            if info.is_secondary && !seen_comp_idx.contains(&info.comp_idx) {
                seen_comp_idx.insert(info.comp_idx);

                // Find the primary edge for this transformer.
                for (eidx, e) in graph.edges.iter().enumerate() {
                    if e.comp_idx == info.comp_idx {
                        // Skip push-pull transformer edges.
                        if pp_transformer_edges.contains(&eidx) {
                            continue;
                        }
                        // Don't add if already in the passive set.
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
