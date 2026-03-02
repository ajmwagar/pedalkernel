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
// Planning
// ═══════════════════════════════════════════════════════════════════════════

/// Plan all WDF stages from classified nonlinear elements.
///
/// Returns a list of stage plans (one per nonlinear element that has passives),
/// a list of push-pull plans, and a list of coupled BJT plans.
pub(super) fn plan_stages(
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> (Vec<StagePlan>, Vec<PushPullPlan>, Vec<CoupledBjtPlan>) {
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
            if let NonlinearKind::Triode { model_name, plate_node, cathode_node, parallel_count, is_vari_mu } = &elem.kind {
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

    let push_pull_pairs = graph.find_push_pull_triode_pairs(
        &triode_infos,
        &classified.all_nonlinear_edge_indices,
    );
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

    // Build CoupledBjtPlan for clusters of 2+ BJTs.
    let mut coupled_bjt_plans: Vec<CoupledBjtPlan> = Vec::new();
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
            bjt_element_indices: elem_indices,
            output_bjt_idx: bjt_infos[output_idx].elem_idx,
        });
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

        if let Some(plan) = plan_single_stage(
            elem,
            elem_idx,
            classified,
            graph,
            &all_bjt_base_nodes,
            source_node_offset,
            sample_rate,
        ) {
            plans.push(plan);
        }

        source_node_offset += 1000;
    }

    (plans, push_pull_plans, coupled_bjt_plans)
}

/// Plan a push-pull half (for building in build.rs).
pub(super) fn plan_push_pull_half(
    elem: &NonlinearElement,
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
) -> Option<StagePlan> {
    if let NonlinearKind::Triode { plate_node, cathode_node, model_name, is_vari_mu, .. } = &elem.kind {
        // For push-pull halves, collect passives WITHOUT filtering supply nodes.
        let collect_passives_at = |junction: NodeId| -> Vec<usize> {
            graph.edges
                .iter()
                .enumerate()
                .filter(|(idx, e)| {
                    if classified.all_nonlinear_edge_indices.contains(idx) { return false; }
                    if graph.active_edge_indices.contains(idx) { return false; }
                    e.node_a == junction || e.node_b == junction
                })
                .map(|(idx, _)| idx)
                .collect()
        };

        let plate_passives = collect_passives_at(*plate_node);
        let cathode_passives = collect_passives_at(*cathode_node);

        let mut passive_idxs: Vec<usize> = plate_passives.clone();
        extend_dedup(&mut passive_idxs, &cathode_passives);

        if passive_idxs.is_empty() {
            return None;
        }

        // Find injection node (prefer plate-side).
        let mut injection_node = graph.gnd_node;
        let mut best_dist = usize::MAX;
        for &eidx in &plate_passives {
            let e = &graph.edges[eidx];
            let other = if e.node_a == *plate_node { e.node_b } else { e.node_a };
            if let Some(&d) = classified.dist_from_in.get(&other) {
                if d < best_dist {
                    best_dist = d;
                    injection_node = other;
                }
            }
        }
        if best_dist == usize::MAX {
            for &eidx in &cathode_passives {
                let e = &graph.edges[eidx];
                let other = if e.node_a == *cathode_node { e.node_b } else { e.node_a };
                if let Some(&d) = classified.dist_from_in.get(&other) {
                    if d < best_dist {
                        best_dist = d;
                        injection_node = other;
                    }
                }
            }
        }
        if best_dist == usize::MAX {
            for &eidx in &plate_passives {
                let e = &graph.edges[eidx];
                let other = if e.node_a == *plate_node { e.node_b } else { e.node_a };
                if other != graph.gnd_node && other != *cathode_node {
                    injection_node = other;
                    break;
                }
            }
        }

        // Ground terminal for push-pull halves.
        let mut ground_terminal = graph.gnd_node;
        for &eidx in &cathode_passives {
            let e = &graph.edges[eidx];
            let far = if e.node_a == *cathode_node { e.node_b } else { e.node_a };
            if far != *plate_node && far != injection_node {
                ground_terminal = far;
                break;
            }
        }

        let source_node = graph.edges.len() + 5000;

        let compensation = if *is_vari_mu {
            0.35
        } else {
            let model = super::helpers::triode_model(model_name);
            model.mu / 100.0
        };

        Some(StagePlan {
            passive_idxs,
            injection_node,
            terminals: vec![source_node, ground_terminal],
            source_node,
            virtual_edge: Some(VirtualEdge {
                node_a: *plate_node,
                node_b: *cathode_node,
                resistance: 62500.0,
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
        NonlinearKind::Pentode { .. } | NonlinearKind::Mosfet { .. } |
        NonlinearKind::Zener { .. } | NonlinearKind::Ota => {
            plan_simple_stage(elem, elem_idx, classified, graph, source_node_offset)
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

    Some(StagePlan {
        passive_idxs,
        injection_node,
        terminals: vec![source_node, graph.gnd_node],
        source_node,
        virtual_edge: Some(VirtualEdge {
            node_a: collector_node,
            node_b: emitter_node,
            resistance: 50000.0, // BJT r_ce ≈ 50kΩ
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

    // Collect passives at plate and cathode.
    let plate_passives = graph.elements_at_junction(
        plate_node,
        &classified.all_nonlinear_edge_indices,
        &graph.active_edge_indices,
    );
    let cathode_passives = graph.elements_at_junction(
        cathode_node,
        &classified.all_nonlinear_edge_indices,
        &graph.active_edge_indices,
    );

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

    let output_passives = graph.elements_at_junction(
        graph.out_node,
        &classified.all_nonlinear_edge_indices,
        &graph.active_edge_indices,
    );

    let mut passive_idxs: Vec<usize> = plate_passives.clone();
    extend_dedup(&mut passive_idxs, &cathode_passives);
    extend_dedup(&mut passive_idxs, &plate_to_output);
    extend_dedup(&mut passive_idxs, &output_passives);

    if passive_idxs.is_empty() {
        return None;
    }

    // Find injection node (prefer plate-side).
    let mut injection_node = graph.in_node;
    let mut best_dist = usize::MAX;

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

    // Compensation from triode mu.
    let compensation = if let NonlinearKind::Triode { model_name, is_vari_mu, .. } = &elem.kind {
        if *is_vari_mu {
            0.35
        } else {
            let model = super::helpers::triode_model(model_name);
            model.mu / 100.0
        }
    } else {
        1.0
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
            resistance: 62500.0, // 12AX7 plate resistance
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
