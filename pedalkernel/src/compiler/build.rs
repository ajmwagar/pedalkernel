//! Pass 4: Unified tree builder.
//!
//! For each StagePlan:
//! 1. Build SP edges from plan (identical algorithm for all types)
//! 2. sp_reduce() → sp_to_dyn_with_vs() or sp_to_dyn()
//! 3. Create RootKind via factory function (single match on NonlinearKind)
//! 4. Balance VS impedance
//! 5. Package into WdfStage

use std::collections::{HashMap, HashSet};

use crate::dsl::*;
use crate::elements::*;
use crate::oversampling::{Oversampler, OversamplingFactor};
use crate::tree::{MnaSystem, RTypeAdaptor, WdfPort};

use super::classify::{ClassifiedCircuit, NonlinearKind};
use super::dyn_node::DynNode;
use super::graph::{CircuitGraph, NodeId, SpTree, sp_reduce, sp_to_dyn};
use super::helpers::*;
use super::opamp_analysis::OpAmpAnalysis;
use super::plan::{CoupledBjtPlan, MultiNlPlan, StagePlan, PushPullPlan};
use super::stage::{CoupledBjtStage, MultiNlDeviceGroups, MultiNlStage, NlDeviceGroupKind, NlDeviceKind, PushPullStage, RootKind, ScatteringRecomputeData, TubeRoot, WdfStage};

// ═══════════════════════════════════════════════════════════════════════════
// Transformer subtree building
// ═══════════════════════════════════════════════════════════════════════════

/// A real WDF subtree for the other side of a transformer.
///
/// When a stage's passive set includes a transformer edge, instead of
/// building a magnetizing-inductance stub, we BFS the opposite winding's
/// passives, reduce them to an SP tree, and attach them as the transformer's
/// child. This gives physical impedance reflection, LF rolloff, and coupling.
#[derive(Clone)]
pub(super) struct TransformerSubtree {
    pub(super) subtree: DynNode,
    /// Turns ratio from the parent port's perspective.
    /// - Case A (primary in tree): n = cfg.turns_ratio (pri:sec)
    /// - Case B (secondary in tree): n = 1/cfg.turns_ratio (sec:pri inverted)
    pub(super) turns_ratio: f64,
}

/// Build real secondary (or primary) subtrees for transformers in a stage's passive set.
///
/// For each non-PP transformer edge in `passive_idxs`:
/// 1. Find the "other side" nodes via `transformer_info`
/// 2. BFS from those nodes (skipping ALL transformers to prevent cascading)
/// 3. `sp_reduce` → `sp_to_dyn` to build a DynNode subtree
/// 4. Fall back to `find_secondary_load_resistance` → Resistor stub if SP fails
///
/// Returns a map from comp_idx → TransformerSubtree.
pub(super) fn build_transformer_subtrees(
    passive_idxs: &[usize],
    graph: &CircuitGraph,
    pp_transformer_edges: &HashSet<usize>,
    sample_rate: f64,
) -> HashMap<usize, TransformerSubtree> {
    let mut result: HashMap<usize, TransformerSubtree> = HashMap::new();

    // Identify transformer edges in this stage's passive set.
    for &eidx in passive_idxs {
        let edge = &graph.edges[eidx];
        let comp = &graph.components[edge.comp_idx];
        let cfg = match &comp.kind {
            ComponentKind::Transformer(cfg) => cfg,
            _ => continue,
        };

        // Skip push-pull transformers (handled separately).
        if pp_transformer_edges.contains(&eidx) {
            continue;
        }

        // Already built (duplicate edge for same component).
        if result.contains_key(&edge.comp_idx) {
            continue;
        }

        // Determine if the stage is on the primary or secondary side.
        // The edge always spans pri.a ↔ pri.b (primary pins).
        let primary_nodes = [edge.node_a, edge.node_b];
        let is_primary_side = primary_nodes.iter().any(|n| {
            graph.transformer_info.get(n)
                .map(|info| !info.is_secondary && info.comp_idx == edge.comp_idx)
                .unwrap_or(false)
        });

        if is_primary_side {
            // Case A: Stage is on the primary side (output transformer).
            // Build secondary subtree from BFS of secondary-side passives.
            if let Some(subtree) = build_other_side_subtree(
                edge.comp_idx, true, graph, sample_rate,
            ) {
                result.insert(edge.comp_idx, TransformerSubtree {
                    subtree,
                    turns_ratio: cfg.turns_ratio,
                });
            }
        } else {
            // Case B: Stage is on the secondary side (input transformer).
            // Build primary subtree from BFS of primary-side passives.
            // Turns ratio inverted: parent port faces secondary.
            if let Some(subtree) = build_other_side_subtree(
                edge.comp_idx, false, graph, sample_rate,
            ) {
                result.insert(edge.comp_idx, TransformerSubtree {
                    subtree,
                    turns_ratio: 1.0 / cfg.turns_ratio,
                });
            }
        }
    }

    result
}

/// BFS the opposite side of a transformer and build a DynNode subtree.
///
/// `build_secondary`: true = BFS from secondary nodes, false = BFS from primary nodes.
fn build_other_side_subtree(
    xfmr_comp_idx: usize,
    build_secondary: bool,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Option<DynNode> {
    // Find the target-side nodes from transformer_info.
    let target_nodes: Vec<NodeId> = graph.transformer_info.iter()
        .filter(|(_, info)| {
            info.comp_idx == xfmr_comp_idx && info.is_secondary == build_secondary
        })
        .map(|(node, _)| *node)
        .collect();

    if target_nodes.is_empty() {
        return None;
    }

    // BFS from target nodes through passives, skipping ALL transformer edges
    // (prevents cascading into other transformers) and NL/active edges.
    let all_transformer_edges: HashSet<usize> = graph.edges.iter().enumerate()
        .filter(|(_, e)| matches!(graph.components[e.comp_idx].kind, ComponentKind::Transformer(_)))
        .map(|(idx, _)| idx)
        .collect();

    let mut collected_edges: Vec<usize> = Vec::new();
    let mut visited: HashSet<NodeId> = HashSet::new();
    let mut queue = std::collections::VecDeque::new();

    for &node in &target_nodes {
        if visited.insert(node) {
            queue.push_back(node);
        }
    }

    while let Some(node) = queue.pop_front() {
        for (idx, e) in graph.edges.iter().enumerate() {
            // Skip NL and active edges.
            if graph.active_edge_indices.contains(&idx) {
                continue;
            }
            // Skip ALL transformer edges (prevent cascading).
            if all_transformer_edges.contains(&idx) {
                continue;
            }
            // Check if this edge touches the current node.
            let neighbor = if e.node_a == node {
                Some(e.node_b)
            } else if e.node_b == node {
                Some(e.node_a)
            } else {
                None
            };
            let Some(n) = neighbor else { continue };

            // Skip NL component edges (by checking component kind).
            match &graph.components[e.comp_idx].kind {
                ComponentKind::Resistor(_)
                | ComponentKind::Capacitor(_)
                | ComponentKind::Inductor(_)
                | ComponentKind::Potentiometer(_, _)
                | ComponentKind::Tempco(_, _) => {}
                _ => continue,
            }

            // Skip supply nodes (boundary).
            if graph.supply_nodes.contains(&n) {
                continue;
            }

            if visited.insert(n) {
                collected_edges.push(idx);
                // Don't traverse through global power rails.
                if n != graph.gnd_node && n != graph.vcc_node {
                    queue.push_back(n);
                }
            }
        }
    }

    if collected_edges.is_empty() {
        // No passives on the other side — fall back to resistor stub.
        let r_load = find_secondary_load_resistance(graph, xfmr_comp_idx);
        return Some(DynNode::Resistor { rp: r_load });
    }

    // Build SP edges for reduction.
    let sp_edges: Vec<(NodeId, NodeId, SpTree)> = collected_edges.iter()
        .map(|&eidx| {
            let e = &graph.edges[eidx];
            (e.node_a, e.node_b, SpTree::Leaf(e.comp_idx))
        })
        .collect();

    // Terminals: the two target nodes (secondary or primary winding pins).
    let terminals = if target_nodes.len() >= 2 {
        vec![target_nodes[0], target_nodes[1]]
    } else {
        // Single-node (one secondary pin connects to ground) — use gnd as second terminal.
        vec![target_nodes[0], graph.gnd_node]
    };

    match sp_reduce(sp_edges, &terminals) {
        Ok(sp_tree) => {
            Some(sp_to_dyn(&sp_tree, &graph.components, &graph.fork_paths, sample_rate))
        }
        Err(_) => {
            // SP reduction failed — fall back to resistor stub.
            let r_load = find_secondary_load_resistance(graph, xfmr_comp_idx);
            Some(DynNode::Resistor { rp: r_load })
        }
    }
}

/// Replace transformer magnetizing-inductance stubs with real subtrees.
///
/// Walks the DynNode tree. For each `Transformer` node, checks if the
/// transformer's comp_idx has a real subtree in the map. If so, replaces
/// the secondary child and recomputes port resistances up the tree.
///
/// This is a post-processing pass that runs after `sp_to_dyn` builds the
/// tree with stub secondaries.
fn replace_transformer_stubs(
    node: &mut DynNode,
    transformer_subtrees: &HashMap<usize, TransformerSubtree>,
    components: &[ComponentDef],
) {
    match node {
        DynNode::Transformer { secondary, turns_ratio, rp, .. } => {
            // First recurse into the current secondary subtree.
            replace_transformer_stubs(secondary, transformer_subtrees, components);

            // Try to find a matching TransformerSubtree by comp_idx.
            // We match by turns_ratio since Transformer DynNodes don't store comp_idx.
            // Each stage has at most one transformer, so this is unambiguous.
            for (_comp_idx, ts) in transformer_subtrees {
                if (ts.turns_ratio - *turns_ratio).abs() < 1e-10 {
                    *secondary = Box::new(ts.subtree.clone());
                    let rp_sec = secondary.port_resistance();
                    *rp = *turns_ratio * *turns_ratio * rp_sec;
                    return;
                }
            }
        }
        DynNode::Series { left, right, rp, gamma, .. } => {
            replace_transformer_stubs(left, transformer_subtrees, components);
            replace_transformer_stubs(right, transformer_subtrees, components);
            let r1 = left.port_resistance();
            let r2 = right.port_resistance();
            *rp = r1 + r2;
            if *rp > 0.0 { *gamma = r1 / *rp; }
        }
        DynNode::Parallel { left, right, rp, gamma, .. } => {
            replace_transformer_stubs(left, transformer_subtrees, components);
            replace_transformer_stubs(right, transformer_subtrees, components);
            let r1 = left.port_resistance();
            let r2 = right.port_resistance();
            *rp = r1 * r2 / (r1 + r2);
            let denom = r1 + r2;
            if denom > 0.0 { *gamma = r2 / denom; }
        }
        DynNode::RType { children, .. } => {
            for child in children {
                replace_transformer_stubs(child, transformer_subtrees, components);
            }
        }
        _ => {}
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Stage building
// ═══════════════════════════════════════════════════════════════════════════

/// Compute inter-stage transformer voltage gain for a stage.
///
/// If the transformer is now modeled in the WDF tree (has entry in
/// `transformer_subtrees`), returns 1.0 — the gain is handled physically
/// by the transformer adaptor's scattering.
///
/// Otherwise falls back to flat scalar gain: checks if any passive edge
/// endpoints are on a transformer secondary winding, returns step-up gain.
fn compute_transformer_gain(
    passive_edge_indices: &[usize],
    graph: &CircuitGraph,
    transformer_subtrees: &HashMap<usize, TransformerSubtree>,
) -> f64 {
    for &eidx in passive_edge_indices {
        let edge = &graph.edges[eidx];
        // If this transformer has a real subtree, gain is modeled by the WDF tree.
        if transformer_subtrees.contains_key(&edge.comp_idx) {
            return 1.0;
        }
        for node in [edge.node_a, edge.node_b] {
            if let Some(info) = graph.transformer_info.get(&node) {
                if info.is_secondary {
                    // Secondary side of a transformer — apply step-up gain.
                    // For 1:17 (n ≈ 0.059): gain = 1/0.059 ≈ 17.0
                    return 1.0 / info.turns_ratio;
                }
            }
        }
    }
    1.0
}

/// Build WDF stages from plans.
///
/// Each plan becomes one WdfStage through the same algorithm:
/// build SP edges → sp_reduce → sp_to_dyn → create root → package stage.
///
/// Triode plans that fail SP reduction are retried as single-NL MNA stages
/// (MultiNlStage with n_nl=1) using BFS passive collection, which handles
/// non-SP topologies like inter-stage coupling networks.
pub(super) fn build_stages(
    plans: &[StagePlan],
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    opamp_analysis: &OpAmpAnalysis,
    sample_rate: f64,
    oversampling: OversamplingFactor,
    pp_transformer_edges: &HashSet<usize>,
) -> (Vec<WdfStage>, Vec<MultiNlStage>) {
    let vs_comp_idx = graph.components.len();

    // Build unity-gain feedback op-amp queue for JFET pairing.
    let mut feedback_opamp_queue = super::opamp_analysis::build_unity_gain_queue(opamp_analysis, sample_rate);

    let mut stages: Vec<WdfStage> = Vec::new();
    let mut fallback_multi_nl: Vec<MultiNlStage> = Vec::new();

    for plan in plans {
        let elem = &classified.nonlinear_elements[plan.element_idx];

        // Build real transformer subtrees for any transformers in this stage's passives.
        let transformer_subtrees = build_transformer_subtrees(
            &plan.passive_idxs, graph, pp_transformer_edges, sample_rate,
        );

        // Pentode + inductive load (transformer) → skip SP, go straight to MNA.
        // High plate impedance (40kΩ+) interacting with transformer reflected
        // load creates a stiff system that causes SP wave reflections to diverge.
        // Pentodes without transformers may still SP-reduce successfully.
        let pentode_with_transformer = matches!(&elem.kind, NonlinearKind::Pentode { .. })
            && plan.passive_idxs.iter().any(|&idx| {
                matches!(graph.components[graph.edges[idx].comp_idx].kind, ComponentKind::Transformer(_))
            });
        if pentode_with_transformer {
            if let Some(mut multi_nl) = build_triode_mna_fallback(
                plan, elem, classified, graph, sample_rate, oversampling,
            ) {
                multi_nl.transformer_gain = compute_transformer_gain(
                    &plan.passive_idxs, graph, &transformer_subtrees,
                );
                // Use element's BFS distance for ordering (injection node may
                // default to in_node when plate/cathode passives lack dist_from_in).
                multi_nl.signal_flow_distance = elem.distance;
                fallback_multi_nl.push(multi_nl);
            }
            continue;
        }

        let stage = if plan.skip_vs {
            build_source_follower_stage(plan, elem, graph, sample_rate, oversampling)
        } else {
            build_vs_stage(plan, elem, graph, sample_rate, oversampling, vs_comp_idx)
        };

        if let Some(mut stage) = stage {
            // Replace transformer stubs with real subtrees if available.
            if !transformer_subtrees.is_empty() {
                replace_transformer_stubs(&mut stage.tree, &transformer_subtrees, &graph.components);
            }

            // Pair JFET stages with feedback op-amps (for all-pass filters).
            if matches!(&elem.kind, NonlinearKind::Jfet { .. }) {
                if !feedback_opamp_queue.is_empty() {
                    stage.paired_opamp = Some(feedback_opamp_queue.remove(0));
                }
            }

            stage.signal_flow_distance = classified
                .dist_from_in
                .get(&plan.injection_node)
                .copied()
                .unwrap_or(usize::MAX);
            stage.transformer_gain = compute_transformer_gain(
                &plan.passive_idxs, graph, &transformer_subtrees,
            );
            stages.push(stage);
        } else if matches!(&elem.kind, NonlinearKind::Triode { .. } | NonlinearKind::Pentode { .. }) {
            // SP reduction failed for this triode/pentode — try building as a
            // single-NL MNA stage. Uses plate+cathode adjacency to collect
            // passives, which handles non-SP topologies (inter-stage coupling,
            // Miller feedback, dangling nodes).
            if let Some(mut multi_nl) = build_triode_mna_fallback(
                plan, elem, classified, graph, sample_rate, oversampling,
            ) {
                multi_nl.transformer_gain = compute_transformer_gain(
                    &plan.passive_idxs, graph, &transformer_subtrees,
                );
                fallback_multi_nl.push(multi_nl);
            }
        }
    }

    // Handle remaining unity-gain op-amps that weren't paired with JFETs.
    for opamp in feedback_opamp_queue.drain(..) {
        let tree = DynNode::VoltageSource { voltage: 0.0, rp: 10_000.0 };
        stages.push(WdfStage {
            tree,
            root: RootKind::OpAmp(opamp),
            compensation: 1.0,
            oversampler: Oversampler::new(oversampling),
            base_diode_model: None,
            paired_opamp: None,
            dc_block: None,
            is_source_follower: false,
            prev_source_voltage: 0.0,
            signal_flow_distance: usize::MAX, // op-amp buffer — runs last
            transformer_gain: 1.0,
        });
    }

    // Balance voltage source impedance in each stage.
    for stage in &mut stages {
        stage.balance_vs_impedance();
    }

    (stages, fallback_multi_nl)
}

/// Build a single-NL MNA stage for a triode/pentode that failed SP reduction.
///
/// Uses BFS from plate and cathode to collect only the passives that are
/// topologically connected to the tube's circuit, avoiding output_passives
/// from other stages that contaminate the plan. MNA handles arbitrary
/// topologies (dangling nodes, non-SP branches, transformer dead-ends)
/// that SP reduction can't.
fn build_triode_mna_fallback(
    plan: &StagePlan,
    elem: &super::classify::NonlinearElement,
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    sample_rate: f64,
    oversampling: OversamplingFactor,
) -> Option<MultiNlStage> {
    let (plate_node, cathode_node) = (elem.junction_nodes[0], elem.junction_nodes[1]);

    // Extract grid_node if this is a triode (pentodes don't expose grid).
    let grid_node = if let NonlinearKind::Triode { grid_node: Some(gn), .. } = &elem.kind {
        Some(*gn)
    } else {
        None
    };

    // Collect passives directly adjacent to plate and cathode junctions,
    // plus supply-adjacent edges. This is the "core" passive set for this
    // tube — no output_passives from other stages.
    let mut passive_edges = graph.elements_at_junction(
        plate_node,
        &classified.all_nonlinear_edge_indices,
        &graph.active_edge_indices,
    );

    // Add supply-adjacent plate edges (plate loads to named supply rails).
    for (idx, e) in graph.edges.iter().enumerate() {
        if classified.all_nonlinear_edge_indices.contains(&idx) { continue; }
        if graph.active_edge_indices.contains(&idx) { continue; }
        if e.node_a != plate_node && e.node_b != plate_node { continue; }
        let other = if e.node_a == plate_node { e.node_b } else { e.node_a };
        if graph.supply_nodes.contains(&other) && !passive_edges.contains(&idx) {
            passive_edges.push(idx);
        }
    }

    // Discover transformers connected through plate passives (1 hop away).
    // Pentode sidechain topology: plate → R_plate → transformer.primary.
    let mut xfmr_edges = Vec::new();
    for &eidx in &passive_edges {
        let e = &graph.edges[eidx];
        let far_node = if e.node_a == plate_node { e.node_b } else { e.node_a };
        for (idx, edge) in graph.edges.iter().enumerate() {
            if edge.node_a != far_node && edge.node_b != far_node { continue; }
            if matches!(graph.components[edge.comp_idx].kind, ComponentKind::Transformer(_)) {
                if !passive_edges.contains(&idx) && !xfmr_edges.contains(&idx) {
                    xfmr_edges.push(idx);
                }
            }
        }
    }
    passive_edges.extend(xfmr_edges);

    // Cathode passives (skip for grounded-cathode tubes).
    if cathode_node != graph.gnd_node {
        let cathode_edges = graph.elements_at_junction(
            cathode_node,
            &classified.all_nonlinear_edge_indices,
            &graph.active_edge_indices,
        );
        for idx in cathode_edges {
            if !passive_edges.contains(&idx) {
                passive_edges.push(idx);
            }
        }
    }

    // Grid passives — BFS from the grid node to collect the full grid-side
    // passive network (bias resistors, threshold pots, input transformers, etc.).
    // Without these, the grid is isolated in the scattering matrix and the
    // triode produces zero output when supply nodes are properly grounded.
    // 1-hop collection is insufficient: pots like AC_Threshold are multiple hops
    // away through bias resistors (grid → R_bias → node → DC_Threshold → node → AC_Threshold).
    if let Some(gn) = grid_node {
        if gn != graph.gnd_node {
            let empty_pp = std::collections::HashSet::new();
            let grid_edges = graph.bfs_passive_edges(
                gn,
                &classified.all_nonlinear_edge_indices,
                &graph.active_edge_indices,
                true,   // include supply-adjacent
                false,  // don't skip out_node
                &empty_pp,
            );
            for idx in grid_edges {
                if !passive_edges.contains(&idx) {
                    passive_edges.push(idx);
                }
            }
        }
    }

    #[cfg(feature = "debug-trace")]
    {
        let kind = match &elem.kind {
            NonlinearKind::Pentode { .. } => "Pentode",
            NonlinearKind::Triode { .. } => "Triode",
            _ => "Other",
        };
        eprintln!("[MNA-fallback] {kind} plate={plate_node} cathode={cathode_node} grid={grid_node:?} passive_edges={passive_edges:?}");
        for &eidx in &passive_edges {
            let e = &graph.edges[eidx];
            let comp = &graph.components[e.comp_idx];
            eprintln!("  edge[{eidx}]: {} ({:?}) nodes=({}, {})", comp.id, std::mem::discriminant(&comp.kind), e.node_a, e.node_b);
        }
    }

    if passive_edges.is_empty() {
        return None;
    }

    // Recalculate injection_node from ALL passive edge endpoints (plate +
    // cathode + grid), preferring nodes closest to input. Exclude the tube's
    // own junction nodes (they are NL device terminals, not injection points).
    let mut injection_node = plan.injection_node;
    let mut best_dist = classified.dist_from_in.get(&injection_node).copied().unwrap_or(usize::MAX);
    for &eidx in &passive_edges {
        let e = &graph.edges[eidx];
        for candidate in [e.node_a, e.node_b] {
            if candidate == plate_node || candidate == cathode_node {
                continue;
            }
            if let Some(gn) = grid_node {
                if candidate == gn { continue; }
            }
            if candidate == graph.gnd_node || candidate == graph.vcc_node
                || graph.supply_nodes.contains(&candidate)
            {
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

    // For triodes with a grid node, use 2 NL terminal pairs:
    // port 0 = grid-cathode, port 1 = plate-cathode.
    // This creates a TriodeThreePort device group where the NR solver
    // handles grid-plate cross-coupling through transconductance (dIa/dVgk).
    // Without this, the grid-side and plate-side passive networks are
    // disconnected in the MNA, giving s_nl_adapted = 0 for the plate port.
    let nl_terminals = if let Some(gn) = grid_node {
        vec![(gn, cathode_node), (plate_node, cathode_node)]
    } else {
        vec![(plate_node, cathode_node)]
    };

    let multi_nl_plan = MultiNlPlan {
        nl_element_indices: vec![plan.element_idx],
        output_element_idx: plan.element_idx,
        passive_edge_indices: passive_edges,
        injection_node,
        nl_terminals,
        compensation: plan.compensation,
        output_node: None,
    };

    try_build_multi_nl_stage(&multi_nl_plan, classified, graph, sample_rate, oversampling)
}

/// Build push-pull stages from plans.
pub(super) fn build_push_pull_stages(
    push_pull_plans: &[PushPullPlan],
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    sample_rate: f64,
    oversampling: OversamplingFactor,
    pp_transformer_edges: &HashSet<usize>,
) -> Vec<PushPullStage> {
    let vs_comp_idx = graph.components.len();
    let mut stages = Vec::new();

    for pp_plan in push_pull_plans {
        let push_elem = &classified.nonlinear_elements[pp_plan.push_triode_list_idx];
        let pull_elem = &classified.nonlinear_elements[pp_plan.pull_triode_list_idx];

        let push_plan = super::plan::plan_push_pull_half(push_elem, classified, graph, pp_transformer_edges);
        let pull_plan = super::plan::plan_push_pull_half(pull_elem, classified, graph, pp_transformer_edges);

        if let (Some(push_plan), Some(pull_plan)) = (push_plan, pull_plan) {
            let push_half = build_push_pull_half(&push_plan, graph, sample_rate, vs_comp_idx);
            let pull_half = build_push_pull_half(&pull_plan, graph, sample_rate, vs_comp_idx);

            if let (Some((push_tree, push_comp)), Some((pull_tree, _))) = (push_half, pull_half) {
                // Look up transformer config and wrap each half with reflected load.
                let xfmr_edge = &graph.edges[pp_plan.transformer_edge_idx];
                let xfmr_cfg = match &graph.components[xfmr_edge.comp_idx].kind {
                    ComponentKind::Transformer(cfg) => cfg,
                    _ => {
                        // Shouldn't happen — plan already validated transformer.
                        continue;
                    }
                };
                let r_load = find_secondary_load_resistance(graph, xfmr_edge.comp_idx);
                let is_ct = matches!(
                    xfmr_cfg.primary_type,
                    crate::dsl::WindingType::CenterTap | crate::dsl::WindingType::PushPull
                );

                let mut push_tree = wrap_with_transformer_load(
                    push_tree,
                    xfmr_cfg.turns_ratio,
                    r_load,
                    xfmr_cfg.primary_inductance,
                    xfmr_cfg.primary_dcr,
                    is_ct,
                    sample_rate,
                );
                let mut pull_tree = wrap_with_transformer_load(
                    pull_tree,
                    xfmr_cfg.turns_ratio,
                    r_load,
                    xfmr_cfg.primary_inductance,
                    xfmr_cfg.primary_dcr,
                    is_ct,
                    sample_rate,
                );

                // Balance VS impedance and recompute adaptor coefficients.
                balance_parallel_vs(&mut push_tree);
                balance_parallel_vs(&mut pull_tree);
                push_tree.recompute();
                pull_tree.recompute();

                let (push_root, pull_root) = build_push_pull_roots(push_elem, pull_elem);

                // Determine grid bias from tube type.
                // Variable-mu (6386): -7.2V per Raffensperger's wavechild670 model.
                // Standard triodes: -2.0V (class A/AB operation).
                let is_vari_mu = matches!(&push_root, TubeRoot::VariMu(_));
                let grid_bias = if is_vari_mu { -7.2 } else { -2.0 };

                stages.push(PushPullStage {
                    push_tree,
                    pull_tree,
                    push_root,
                    pull_root,
                    push_oversampler: Oversampler::new(oversampling),
                    pull_oversampler: Oversampler::new(oversampling),
                    compensation: push_comp,
                    turns_ratio: pp_plan.turns_ratio,
                    grid_bias,
                });
            }
        }
    }

    stages
}

/// Build coupled BJT stages from plans.
///
/// Each CoupledBjtPlan groups 2+ tightly-coupled BJTs. For each plan:
/// 1. Plan each BJT independently using plan_bjt_stage()
/// 2. Build each tree with build_vs_stage()
/// 3. Assemble into a CoupledBjtStage with 1-sample delay feedback
///
/// If SP reduction fails for any member (non-SP topology), falls back to
/// building those BJTs as independent WdfStages instead.
///
/// Returns (coupled_stages, fallback_independent_stages).
pub(super) fn build_coupled_bjt_stages(
    coupled_plans: &[super::plan::CoupledBjtPlan],
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    opamp_analysis: &OpAmpAnalysis,
    sample_rate: f64,
    oversampling: OversamplingFactor,
) -> (Vec<super::stage::CoupledBjtStage>, Vec<WdfStage>) {
    use std::collections::HashSet;
    let vs_comp_idx = graph.components.len();
    let empty_base_nodes: HashSet<super::graph::NodeId> = HashSet::new();
    let mut coupled_stages = Vec::new();
    let mut fallback_stages = Vec::new();

    for coupled_plan in coupled_plans {
        let mut bjt_stages = Vec::new();
        let mut compensations = Vec::new();
        let mut source_node_offset = 8000usize;
        let mut all_built = true;

        for &elem_idx in &coupled_plan.bjt_element_indices {
            let elem = &classified.nonlinear_elements[elem_idx];

            // Plan this BJT independently using the simplified planner.
            let plan = super::plan::plan_bjt_stage(
                elem,
                elem_idx,
                classified,
                graph,
                &empty_base_nodes,
                source_node_offset,
            );

            if let Some(plan) = plan {
                // Build the WDF tree for this BJT.
                let stage = build_vs_stage(
                    &plan, elem, graph, sample_rate, oversampling, vs_comp_idx,
                );

                if let Some(mut wdf_stage) = stage {
                    wdf_stage.balance_vs_impedance();
                    compensations.push(plan.compensation);
                    bjt_stages.push((wdf_stage.tree, wdf_stage.root, wdf_stage.oversampler));
                } else {
                    all_built = false;
                }
            } else {
                all_built = false;
            }

            source_node_offset += 1000;
        }

        if all_built && bjt_stages.len() >= 2 {
            // All BJTs built successfully — create coupled stage.
            // Signal flow distance: use the first BJT's base node distance.
            let first_elem = &classified.nonlinear_elements[coupled_plan.bjt_element_indices[0]];
            let sfd = match &first_elem.kind {
                NonlinearKind::BjtNpn { base_node, .. }
                | NonlinearKind::BjtPnp { base_node, .. } => {
                    classified.dist_from_in.get(base_node).copied().unwrap_or(usize::MAX)
                }
                _ => usize::MAX,
            };
            coupled_stages.push(super::stage::CoupledBjtStage {
                bjt_stages,
                compensations,
                feedback_state: 0.0,
                feedback_scale: 0.1,
                signal_flow_distance: sfd,
            });
        } else {
            // SP reduction failed for at least one member — fall back to
            // independent stage planning via the standard build pipeline.
            let plans: Vec<_> = coupled_plan.bjt_element_indices.iter()
                .enumerate()
                .filter_map(|(i, &elem_idx)| {
                    let elem = &classified.nonlinear_elements[elem_idx];
                    super::plan::plan_bjt_stage(
                        elem, elem_idx, classified, graph,
                        &empty_base_nodes, 9000 + i * 1000,
                    )
                })
                .collect();
            let empty_pp = HashSet::new();
            let (indep_stages, _triode_fb) = super::build::build_stages(
                &plans, classified, graph, opamp_analysis, sample_rate, oversampling, &empty_pp,
            );
            fallback_stages.extend(indep_stages);
        }
    }

    (coupled_stages, fallback_stages)
}

// ═══════════════════════════════════════════════════════════════════════════
// Multi-NL stage building (R-type adaptor + multi-port NR solver)
// ═══════════════════════════════════════════════════════════════════════════

/// Build multi-NL stages from plans using R-type adaptor approach.
///
/// For each `MultiNlPlan`, attempts MNA-based construction:
/// 1. Map circuit nodes → MNA node indices
/// 2. Classify passive edges: resistors stamp into MNA, reactive elements become WDF ports
/// 3. Build MNA → derive scattering matrix → create RTypeAdaptor
/// 4. Extract sub-blocks (s_nl, s_nl_passive, s_nl_adapted)
/// 5. Create NL device roots, package as MultiNlStage
///
/// Falls back to old `build_coupled_bjt_stages()` on failure.
///
/// Returns (multi_nl_stages, coupled_bjt_fallback_stages, independent_fallback_stages).
pub(super) fn build_multi_nl_stages(
    multi_nl_plans: &[MultiNlPlan],
    coupled_bjt_plans: &[CoupledBjtPlan],
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    opamp_analysis: &OpAmpAnalysis,
    sample_rate: f64,
    oversampling: OversamplingFactor,
) -> (Vec<MultiNlStage>, Vec<CoupledBjtStage>, Vec<WdfStage>) {
    let mut multi_nl_stages = Vec::new();
    let mut needs_fallback = false;

    for plan in multi_nl_plans {
        match try_build_multi_nl_stage(plan, classified, graph, sample_rate, oversampling) {
            Some(mut stage) => {
                let empty_subtrees = HashMap::new();
                stage.transformer_gain = compute_transformer_gain(
                    &plan.passive_edge_indices, graph, &empty_subtrees,
                );
                multi_nl_stages.push(stage);
            }
            None => {
                needs_fallback = true;
            }
        }
    }

    // If any multi-NL plan failed, fall back to old coupled BJT approach for ALL plans.
    // This keeps things simple — either all succeed or we fall back entirely.
    if needs_fallback {
        multi_nl_stages.clear();
        let (coupled, fallback) = build_coupled_bjt_stages(
            coupled_bjt_plans, classified, graph, opamp_analysis, sample_rate, oversampling,
        );
        return (Vec::new(), coupled, fallback);
    }

    // All multi-NL stages built successfully. No coupled BJT fallback needed.
    (multi_nl_stages, Vec::new(), Vec::new())
}

/// Try to build a single MultiNlStage from a plan.
///
/// Returns `None` if MNA construction fails (e.g., singular matrix, no passive edges).
fn try_build_multi_nl_stage(
    plan: &MultiNlPlan,
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    sample_rate: f64,
    oversampling: OversamplingFactor,
) -> Option<MultiNlStage> {
    // n_nl = number of NL ports (terminal pairs), which may exceed the number
    // of NL elements (e.g., a 3-port VariMu triode has 1 element but 2 ports).
    let n_nl = plan.nl_terminals.len();
    if n_nl == 0 || plan.passive_edge_indices.is_empty() {
        return None;
    }

    // ── Step 1: Collect unique circuit nodes and map to MNA indices ────
    // Ground and supply nodes are excluded from MNA (implicit AC ground).
    // Supply nodes (VCC, B+, bias rails) are zero-impedance voltage sources,
    // so they are AC ground — same treatment as gnd_node. This ensures plate
    // load resistors (plate → VCC) stamp as plate → ground, properly loading
    // the tube's NL port.
    let mut node_set: Vec<NodeId> = Vec::new();
    let mut add_node = |node: NodeId| {
        if node == graph.gnd_node
            || node == graph.vcc_node
            || graph.supply_nodes.contains(&node)
        {
            return;
        }
        if !node_set.contains(&node) {
            node_set.push(node);
        }
    };

    // Collect nodes from passive edges
    for &eidx in &plan.passive_edge_indices {
        let e = &graph.edges[eidx];
        add_node(e.node_a);
        add_node(e.node_b);
    }

    // Collect nodes from NL terminals
    for &(pos, neg) in &plan.nl_terminals {
        add_node(pos);
        add_node(neg);
    }

    // Injection node (voltage source input)
    add_node(plan.injection_node);

    let num_mna_nodes = node_set.len();
    if num_mna_nodes == 0 {
        return None;
    }

    let node_to_mna = |node: NodeId| -> Option<usize> {
        if node == graph.gnd_node
            || node == graph.vcc_node
            || graph.supply_nodes.contains(&node)
        {
            None // AC ground reference
        } else {
            node_set.iter().position(|&n| n == node)
        }
    };

    // ── Step 2: Classify passive edges ──────────────────────────────
    // Resistors → stamp directly into MNA (no WDF port needed)
    // Capacitors, inductors, pots → WDF port + DynNode child
    let mut reactive_edges: Vec<(usize, DynNode)> = Vec::new(); // (edge_idx, dyn_node)

    let mut mna = MnaSystem::new(num_mna_nodes, 0);

    for &eidx in &plan.passive_edge_indices {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        let n1 = node_to_mna(e.node_a);
        let n2 = node_to_mna(e.node_b);

        match &comp.kind {
            ComponentKind::Resistor(r) => {
                // Stamp resistor directly into MNA conductance matrix
                mna.stamp_resistor(n1, n2, *r);
            }
            ComponentKind::Capacitor(cfg) => {
                let rp = 1.0 / (2.0 * sample_rate * cfg.value);
                let dyn_node = DynNode::Capacitor {
                    capacitance: cfg.value,
                    rp,
                    state: 0.0,
                    last_b: 0.0,
                };
                reactive_edges.push((eidx, dyn_node));
            }
            ComponentKind::Inductor(l) => {
                let dyn_node = DynNode::Inductor {
                    inductance: *l,
                    rp: 2.0 * sample_rate * *l,
                    state: 0.0,
                };
                reactive_edges.push((eidx, dyn_node));
            }
            ComponentKind::Potentiometer(max_r, taper) => {
                let initial_pos = 0.5;
                let tapered_pos = taper.apply(initial_pos);
                let dyn_node = DynNode::Pot {
                    comp_id: comp.id.clone(),
                    max_resistance: *max_r,
                    position: initial_pos,
                    taper: *taper,
                    rp: (tapered_pos * *max_r).max(1.0),
                };
                reactive_edges.push((eidx, dyn_node));
            }
            ComponentKind::Tempco(r, _ppm) => {
                // Temperature-compensated resistor — treat as fixed resistor in MNA
                mna.stamp_resistor(n1, n2, *r);
            }
            ComponentKind::ResistorSwitched(values) => {
                // Switched resistor — stamp with the first (default) value
                if let Some(&r) = values.first() {
                    if r.is_finite() && r > 0.0 {
                        mna.stamp_resistor(n1, n2, r);
                    }
                }
            }
            ComponentKind::CapSwitched(values) => {
                // Switched capacitor — reactive edge with first (default) value
                if let Some(&c) = values.first() {
                    if c.is_finite() && c > 0.0 {
                        let rp = 1.0 / (2.0 * sample_rate * c);
                        let dyn_node = DynNode::Capacitor {
                            capacitance: c,
                            rp,
                            state: 0.0,
                            last_b: 0.0,
                        };
                        reactive_edges.push((eidx, dyn_node));
                    }
                }
            }
            ComponentKind::InductorSwitched(values) => {
                // Switched inductor — reactive edge with first (default) value
                if let Some(&l) = values.first() {
                    if l.is_finite() && l > 0.0 {
                        let dyn_node = DynNode::Inductor {
                            inductance: l,
                            rp: 2.0 * sample_rate * l,
                            state: 0.0,
                        };
                        reactive_edges.push((eidx, dyn_node));
                    }
                }
            }
            ComponentKind::Transformer(cfg) => {
                // Transformer primary acts as magnetizing inductance.
                // The secondary load (e.g., bridge rectifier) is a separate stage;
                // from the primary side we only see the magnetizing inductance.
                let l = cfg.primary_inductance;
                if l > 0.0 && l.is_finite() {
                    let rp = 2.0 * sample_rate * l;
                    let dyn_node = DynNode::Inductor {
                        inductance: l,
                        rp,
                        state: 0.0,
                    };
                    reactive_edges.push((eidx, dyn_node));
                }
                // Also stamp primary DCR if non-zero
                if cfg.primary_dcr > 0.0 {
                    mna.stamp_resistor(n1, n2, cfg.primary_dcr);
                }
            }
            _ => {
                // Skip unknown edge types
            }
        }
    }

    // ── GMIN regularization ──────────────────────────────────────────
    // Standard SPICE technique: add a minimum conductance (GMIN) from every
    // MNA node to ground. This prevents singular matrices when nodes are
    // connected to the rest of the circuit only through NL elements (e.g.
    // transformer secondary nodes in a bridge rectifier). Without GMIN,
    // those nodes have zero passive conductance and the scattering matrix
    // shows zero coupling from the adapted port (s_nl_adapted = 0).
    const GMIN_RESISTANCE: f64 = 1e9; // 1 GΩ → 1 nS conductance
    for i in 0..num_mna_nodes {
        mna.stamp_resistor(Some(i), None, GMIN_RESISTANCE);
    }

    // ── Step 3: Build WDF ports ─────────────────────────────────────
    // Port ordering: [NL_0..NL_{n-1}, reactive_0..reactive_{m-1}, adapted]
    // The "adapted" port is the voltage source injection port.
    let n_passive = reactive_edges.len();
    let n_total = n_nl + n_passive + 1; // +1 for adapted (voltage source)

    let mut ports: Vec<WdfPort> = Vec::with_capacity(n_total);
    let mut port_node_pairs: Vec<(Option<usize>, Option<usize>)> = Vec::with_capacity(n_total);
    let mut has_pots = false;

    // NL ports: each spans collector-to-emitter (or plate-to-cathode)
    let mut nl_port_resistances = Vec::with_capacity(n_nl);
    for i in 0..n_nl {
        let (pos_node, neg_node) = plan.nl_terminals[i];
        let pos = node_to_mna(pos_node);
        let neg = node_to_mna(neg_node);

        // Default NL port resistance: 10kΩ (typical collector impedance)
        let r_nl = 10_000.0;
        ports.push(WdfPort {
            node_pos: pos,
            node_neg: neg,
            resistance: r_nl,
        });
        port_node_pairs.push((pos, neg));
        nl_port_resistances.push(r_nl);
    }

    // Reactive ports: each spans the edge's two nodes
    let reactive_edge_indices: Vec<usize> = reactive_edges.iter().map(|(eidx, _)| *eidx).collect();
    let mut passive_children: Vec<DynNode> = Vec::with_capacity(n_passive);
    for (eidx, dyn_node) in reactive_edges {
        let e = &graph.edges[eidx];
        let pos = node_to_mna(e.node_a);
        let neg = node_to_mna(e.node_b);
        let rp = dyn_node.port_resistance();
        if matches!(&dyn_node, DynNode::Pot { .. }) {
            has_pots = true;
        }
        ports.push(WdfPort {
            node_pos: pos,
            node_neg: neg,
            resistance: rp,
        });
        port_node_pairs.push((pos, neg));
        passive_children.push(dyn_node);
    }

    // Adapted port: voltage source at injection_node to ground
    // Port resistance will be set so S[n-1][n-1] ≈ 0 (reflection-free).
    // Start with a reasonable guess; the scattering derivation handles adaptation.
    let injection_mna = node_to_mna(plan.injection_node);
    let r_adapted = 1000.0; // Will be computed from Thévenin impedance
    ports.push(WdfPort {
        node_pos: injection_mna,
        node_neg: None, // ground
        resistance: r_adapted,
    });
    port_node_pairs.push((injection_mna, None));

    // ── Step 4: Derive scattering matrix ────────────────────────────
    let scattering = mna.derive_scattering_matrix_general(&ports);

    // Validate scattering matrix: check for NaN/inf
    if scattering.iter().any(|&s| !s.is_finite()) {
        return None;
    }

    // Check adapted port reflection: S[n-1][n-1] should be near 0
    let s_adapted_refl = scattering[(n_total - 1) * n_total + (n_total - 1)];
    if s_adapted_refl.abs() > 0.5 {
        // Poor adaptation — try to compute proper adapted resistance.
        // The Thévenin impedance at the adapted port determines the ideal resistance.
        // For now, accept imperfect adaptation (the solver still works, just less efficient).
    }

    // ── Step 5: Extract sub-blocks of scattering matrix ─────────────
    // s_nl: n_nl × n_nl (NL-to-NL coupling)
    let mut s_nl = vec![0.0; n_nl * n_nl];
    for i in 0..n_nl {
        for j in 0..n_nl {
            s_nl[i * n_nl + j] = scattering[i * n_total + j];
        }
    }

    // s_nl_passive: n_nl × n_passive (NL-to-passive coupling)
    let mut s_nl_passive = vec![0.0; n_nl * n_passive];
    for i in 0..n_nl {
        for k in 0..n_passive {
            s_nl_passive[i * n_passive + k] = scattering[i * n_total + (n_nl + k)];
        }
    }

    // s_nl_adapted: n_nl (NL-to-adapted column)
    let mut s_nl_adapted = vec![0.0; n_nl];
    for i in 0..n_nl {
        s_nl_adapted[i] = scattering[i * n_total + (n_total - 1)];
    }

    // ── Step 6: Create NL device roots ──────────────────────────────
    // Detect 3-port triodes: single element with 2 NL terminal pairs
    // (grid-cathode + plate-cathode). Create a grouped device for the
    // multi-port NR solver with cross-coupled Jacobian.
    let is_three_port_vari_mu = plan.nl_element_indices.len() == 1
        && n_nl == 2
        && matches!(
            &classified.nonlinear_elements[plan.nl_element_indices[0]].kind,
            NonlinearKind::Triode { is_vari_mu: true, .. }
        );

    let is_three_port_triode = plan.nl_element_indices.len() == 1
        && n_nl == 2
        && matches!(
            &classified.nonlinear_elements[plan.nl_element_indices[0]].kind,
            NonlinearKind::Triode { is_vari_mu: false, .. }
        );

    let (nl_devices, device_groups) = if is_three_port_vari_mu {
        let elem = &classified.nonlinear_elements[plan.nl_element_indices[0]];
        if let NonlinearKind::Triode { model_name, parallel_count, .. } = &elem.kind {
            let model = vari_mu_model(model_name);
            let three_port = VariMuThreePort::new(model).with_parallel_count(*parallel_count);
            let groups = MultiNlDeviceGroups {
                groups: vec![NlDeviceGroupKind::VariMuThreePort(three_port)],
                offsets: vec![0],
            };
            (Vec::new(), Some(groups))
        } else {
            unreachable!()
        }
    } else if is_three_port_triode {
        let elem = &classified.nonlinear_elements[plan.nl_element_indices[0]];
        if let NonlinearKind::Triode { model_name, parallel_count, .. } = &elem.kind {
            let model = TriodeModel::by_name(model_name);
            let three_port = TriodeThreePort::new(model).with_parallel_count(*parallel_count);
            let groups = MultiNlDeviceGroups {
                groups: vec![NlDeviceGroupKind::TriodeThreePort(three_port)],
                offsets: vec![0],
            };
            (Vec::new(), Some(groups))
        } else {
            unreachable!()
        }
    } else {
        // Standard case: one NlDeviceKind per element.
        let mut devices = Vec::with_capacity(n_nl);
        for &elem_idx in &plan.nl_element_indices {
            let elem = &classified.nonlinear_elements[elem_idx];
            let device = create_nl_device(&elem.kind)?;
            devices.push(device);
        }
        (devices, None)
    };

    // ── Step 7: Determine output port ───────────────────────────────
    // For passive-port output (bridge rectifier): find the reactive port
    // touching the output node.
    // For 3-port VariMu: output is port 1 (plate-cathode).
    // For standard multi-NL: output is the NL element closest to output.
    let output_port = if let Some(out_node) = plan.output_node {
        // Find reactive port touching out_node, or closest to it through resistors.
        // First try: direct match (reactive edge endpoint == out_node).
        let direct_idx = reactive_edge_indices.iter().position(|&eidx| {
            let e = &graph.edges[eidx];
            e.node_a == out_node || e.node_b == out_node
        });
        if let Some(idx) = direct_idx {
            n_nl + idx
        } else {
            // No reactive edge touches out_node directly. BFS from out_node
            // through resistor edges (already stamped in MNA) to find the
            // first node that has a reactive edge. This handles topologies
            // like: out_node → R_time (resistor) → C_time (capacitor).
            let mut visited: std::collections::HashSet<NodeId> = std::collections::HashSet::new();
            let mut queue = std::collections::VecDeque::new();
            visited.insert(out_node);
            queue.push_back(out_node);

            let reactive_nodes: std::collections::HashSet<NodeId> = reactive_edge_indices.iter()
                .flat_map(|&eidx| {
                    let e = &graph.edges[eidx];
                    [e.node_a, e.node_b]
                })
                .collect();

            let mut found_node = None;
            'bfs: while let Some(node) = queue.pop_front() {
                // Check if any reactive edge touches this node.
                if reactive_nodes.contains(&node) && node != out_node {
                    found_node = Some(node);
                    break 'bfs;
                }
                // Expand through resistor edges in the passive set.
                for &eidx in &plan.passive_edge_indices {
                    let e = &graph.edges[eidx];
                    let neighbor = if e.node_a == node { Some(e.node_b) }
                        else if e.node_b == node { Some(e.node_a) }
                        else { None };
                    if let Some(n) = neighbor {
                        if visited.insert(n) {
                            // Only follow through resistor edges (not reactive ones).
                            let comp = &graph.components[e.comp_idx];
                            if matches!(&comp.kind,
                                ComponentKind::Resistor(_)
                                | ComponentKind::ResistorSwitched(_)
                                | ComponentKind::Tempco(_, _)
                            ) {
                                queue.push_back(n);
                            }
                        }
                    }
                }
            }

            if let Some(target_node) = found_node {
                // Find the reactive edge that touches target_node.
                let passive_idx = reactive_edge_indices.iter().position(|&eidx| {
                    let e = &graph.edges[eidx];
                    e.node_a == target_node || e.node_b == target_node
                });
                match passive_idx {
                    Some(idx) => n_nl + idx,
                    None => {
                        plan.nl_element_indices.iter()
                            .position(|&idx| idx == plan.output_element_idx)
                            .unwrap_or(0)
                    }
                }
            } else {
                // Fallback: NL element closest to output
                plan.nl_element_indices.iter()
                    .position(|&idx| idx == plan.output_element_idx)
                    .unwrap_or(0)
            }
        }
    } else if is_three_port_vari_mu || is_three_port_triode {
        1 // plate-cathode port (port 0 = grid-cathode, port 1 = plate-cathode)
    } else {
        plan.nl_element_indices
            .iter()
            .position(|&idx| idx == plan.output_element_idx)
            .unwrap_or(0)
    };

    // ── Step 8: Create RTypeAdaptor and package ─────────────────────
    let port_resistances: Vec<f64> = ports.iter().map(|p| p.resistance).collect();
    let adaptor = RTypeAdaptor::new(scattering, &port_resistances);

    // Store recompute data only if there are pots (saves memory otherwise).
    let recompute_data = if has_pots {
        Some(ScatteringRecomputeData {
            mna,
            port_node_pairs,
            adapted_resistance: r_adapted,
        })
    } else {
        None
    };

    let signal_flow_distance = classified
        .dist_from_in
        .get(&plan.injection_node)
        .copied()
        .unwrap_or(usize::MAX);

    // Determine the circuit graph node IDs for node-based routing.
    let injection_node_id = plan.injection_node;
    let output_node_id = if let Some(out_node) = plan.output_node {
        // Passive-port output (e.g., bridge rectifier RC output)
        out_node
    } else if output_port < n_nl {
        // NL port output: use positive terminal (plate/collector)
        plan.nl_terminals[output_port].0
    } else {
        // Passive port output (fallback): use injection node
        plan.injection_node
    };

    Some(MultiNlStage {
        adaptor,
        nl_devices,
        nl_port_resistances,
        passive_children,
        n_nl,
        v_prev: vec![0.0; n_nl],
        s_nl,
        s_nl_passive,
        s_nl_adapted,
        oversampler: Oversampler::new(oversampling),
        compensation: plan.compensation,
        output_port,
        device_groups,
        recompute_data,
        signal_flow_distance,
        transformer_gain: 1.0, // set by caller if needed
        injection_node_id,
        output_node_id,
    })
}

/// Create an NlDeviceKind from a NonlinearKind classification.
fn create_nl_device(kind: &NonlinearKind) -> Option<NlDeviceKind> {
    match kind {
        NonlinearKind::BjtNpn { model_name, .. } => {
            let model = BjtModel::by_name(model_name);
            Some(NlDeviceKind::BjtNpn(BjtNpnRoot::new(model)))
        }
        NonlinearKind::BjtPnp { model_name, .. } => {
            let model = BjtModel::by_name(model_name);
            Some(NlDeviceKind::BjtPnp(BjtPnpRoot::new(model)))
        }
        NonlinearKind::Triode { model_name, parallel_count, is_vari_mu, .. } => {
            if *is_vari_mu {
                let model = vari_mu_model(model_name);
                Some(NlDeviceKind::VariMu(
                    VariMuTriodeRoot::new(model).with_parallel_count(*parallel_count),
                ))
            } else {
                let model = triode_model(model_name);
                Some(NlDeviceKind::Triode(
                    TriodeRoot::new(model).with_parallel_count(*parallel_count),
                ))
            }
        }
        NonlinearKind::SingleDiode(dt) => {
            let model = diode_model(*dt);
            Some(NlDeviceKind::Diode(DiodeRoot::new(model)))
        }
        NonlinearKind::Pentode { model_name } => {
            let model = pentode_model(model_name);
            Some(NlDeviceKind::Pentode(PentodeRoot::new(model)))
        }
        _ => None, // DiodePair, Jfet, Mosfet, Zener, Ota not yet supported in multi-NL
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Internal builders
// ═══════════════════════════════════════════════════════════════════════════

/// Collect SP edges from a plan.
///
/// Adds the voltage source edge (if `vs_comp_idx` is Some), passive edges,
/// and virtual edge (if present). Returns the virtual edge component index if applicable.
fn collect_sp_edges(
    plan: &StagePlan,
    graph: &CircuitGraph,
    vs_comp_idx: Option<usize>,
) -> (Vec<(usize, usize, SpTree)>, Option<usize>) {
    collect_sp_edges_inner(plan, graph, vs_comp_idx, false)
}

/// Like `collect_sp_edges` but remaps named supply rail nodes to gnd_node.
///
/// Named supply rails (vcc_sc, A_bal, etc.) are ideal voltage sources with
/// zero AC impedance, so a plate load to vcc_sc is AC-equivalent to ground.
/// This is needed for sidechain triode stages whose plate loads connect to
/// named supply rails rather than standard vcc.
///
/// NOT used for push-pull halves — those handle supply nodes via the
/// `supply_leaf_voltages` mechanism + `sp_to_dyn_with_vs_and_supplies`.
fn collect_sp_edges_with_supply_remap(
    plan: &StagePlan,
    graph: &CircuitGraph,
    vs_comp_idx: Option<usize>,
) -> (Vec<(usize, usize, SpTree)>, Option<usize>) {
    collect_sp_edges_inner(plan, graph, vs_comp_idx, true)
}

fn collect_sp_edges_inner(
    plan: &StagePlan,
    graph: &CircuitGraph,
    vs_comp_idx: Option<usize>,
    remap_supply_nodes: bool,
) -> (Vec<(usize, usize, SpTree)>, Option<usize>) {
    let remap = |node: NodeId| -> NodeId {
        if remap_supply_nodes && graph.supply_nodes.contains(&node) {
            graph.gnd_node
        } else {
            node
        }
    };

    let mut sp_edges: Vec<(usize, usize, SpTree)> = Vec::new();

    if let Some(vs_idx) = vs_comp_idx {
        sp_edges.push((plan.source_node, plan.injection_node, SpTree::Leaf(vs_idx)));
    }

    for &eidx in &plan.passive_idxs {
        let e = &graph.edges[eidx];
        let na = remap(e.node_a);
        let nb = remap(e.node_b);
        // Skip self-loops (both endpoints mapped to same node, e.g., gnd-to-gnd).
        if na != nb {
            sp_edges.push((na, nb, SpTree::Leaf(e.comp_idx)));
        }
    }

    let virtual_edge_idx = if let Some(ve) = &plan.virtual_edge {
        let ve_idx = vs_comp_idx.unwrap_or(graph.components.len()) + 1;
        sp_edges.push((ve.node_a, ve.node_b, SpTree::Leaf(ve_idx)));
        Some(ve_idx)
    } else {
        None
    };

    (sp_edges, virtual_edge_idx)
}

/// Build a component list with virtual voltage source and optional virtual edge.
fn build_components_with_virtuals(
    graph: &CircuitGraph,
    vs_comp_idx: usize,
    virtual_edge: Option<(&super::plan::VirtualEdge, usize)>,
) -> Vec<ComponentDef> {
    let max_idx = virtual_edge.map(|(_, idx)| idx).unwrap_or(vs_comp_idx);
    let mut components = graph.components.clone();
    while components.len() <= max_idx {
        components.push(ComponentDef {
            id: "__vs__".to_string(),
            kind: ComponentKind::Resistor(1.0),
        });
    }
    if let Some((ve, ve_idx)) = virtual_edge {
        components[ve_idx] = ComponentDef {
            id: ve.name.to_string(),
            kind: ComponentKind::Resistor(ve.resistance),
        };
    }
    components
}

/// Build a standard VS-driven stage (simple, virtual edge, or push-pull half).
fn build_vs_stage(
    plan: &StagePlan,
    elem: &super::classify::NonlinearElement,
    graph: &CircuitGraph,
    sample_rate: f64,
    oversampling: OversamplingFactor,
    vs_comp_idx: usize,
) -> Option<WdfStage> {
    // Use supply remap so triodes with plate loads to named supply rails
    // (vcc_sc, A_bal, etc.) can reduce to valid SP trees.
    let (sp_edges, virtual_edge_idx) = collect_sp_edges_with_supply_remap(plan, graph, Some(vs_comp_idx));
    let sp_tree = sp_reduce(sp_edges, &plan.terminals).ok()?;

    let ve_info = plan.virtual_edge.as_ref().zip(virtual_edge_idx);
    let components = build_components_with_virtuals(graph, vs_comp_idx, ve_info);

    let tree = sp_to_dyn_with_vs(&sp_tree, &components, &graph.fork_paths, sample_rate, vs_comp_idx);
    let (root, base_diode_model) = create_root(&elem.kind);

    Some(WdfStage {
        tree,
        root,
        compensation: plan.compensation,
        oversampler: Oversampler::new(oversampling),
        base_diode_model,
        paired_opamp: None,
        dc_block: plan.dc_block,
        is_source_follower: false,
        prev_source_voltage: 0.0,
        signal_flow_distance: 0, // set by caller
        transformer_gain: 1.0, // set by caller
    })
}

/// Build a source follower stage (no voltage source in tree).
fn build_source_follower_stage(
    plan: &StagePlan,
    elem: &super::classify::NonlinearElement,
    graph: &CircuitGraph,
    sample_rate: f64,
    oversampling: OversamplingFactor,
) -> Option<WdfStage> {
    let (sp_edges, _) = collect_sp_edges(plan, graph, None);
    let sp_tree = sp_reduce(sp_edges, &plan.terminals).ok()?;
    let tree = sp_to_dyn(&sp_tree, &graph.components, &graph.fork_paths, sample_rate);
    let (root, _) = create_root(&elem.kind);

    Some(WdfStage {
        tree,
        root,
        compensation: 1.0,
        oversampler: Oversampler::new(oversampling),
        base_diode_model: None,
        paired_opamp: None,
        dc_block: None,
        is_source_follower: true,
        prev_source_voltage: 0.0,
        signal_flow_distance: 0, // set by caller
        transformer_gain: 1.0, // set by caller
    })
}

/// Build a push-pull half tree.
///
/// Identifies edges that terminate at supply nodes and creates
/// `CathodeBiasSource` leaves for them instead of plain resistors.
fn build_push_pull_half(
    plan: &StagePlan,
    graph: &CircuitGraph,
    sample_rate: f64,
    vs_comp_idx: usize,
) -> Option<(DynNode, f64)> {
    // Identify which passive edges terminate at supply nodes.
    // Map: comp_idx → supply voltage for creating CathodeBiasSource leaves.
    let mut supply_leaf_voltages: std::collections::HashMap<usize, f64> = std::collections::HashMap::new();
    for &eidx in &plan.passive_idxs {
        let e = &graph.edges[eidx];
        for &node in &[e.node_a, e.node_b] {
            if let Some(&voltage) = graph.supply_voltages.get(&node) {
                supply_leaf_voltages.insert(e.comp_idx, voltage);
            }
        }
    }

    let (sp_edges, virtual_edge_idx) = collect_sp_edges(plan, graph, Some(vs_comp_idx));
    let sp_tree = sp_reduce(sp_edges, &plan.terminals).ok()?;

    let ve_info = plan.virtual_edge.as_ref().zip(virtual_edge_idx);
    let components = build_components_with_virtuals(graph, vs_comp_idx, ve_info);

    let tree = if supply_leaf_voltages.is_empty() {
        sp_to_dyn_with_vs(&sp_tree, &components, &graph.fork_paths, sample_rate, vs_comp_idx)
    } else {
        super::helpers::sp_to_dyn_with_vs_and_supplies(
            &sp_tree, &components, &graph.fork_paths, sample_rate, vs_comp_idx,
            &supply_leaf_voltages,
        )
    };
    Some((tree, plan.compensation))
}

/// Build push-pull tube roots from classified elements.
fn build_push_pull_roots(
    push_elem: &super::classify::NonlinearElement,
    pull_elem: &super::classify::NonlinearElement,
) -> (TubeRoot, TubeRoot) {
    let build_root = |elem: &super::classify::NonlinearElement| -> TubeRoot {
        if let NonlinearKind::Triode { model_name, parallel_count, is_vari_mu, .. } = &elem.kind {
            if *is_vari_mu {
                let model = vari_mu_model(model_name);
                TubeRoot::VariMu(
                    VariMuTriodeRoot::new(model).with_parallel_count(*parallel_count),
                )
            } else {
                let model = triode_model(model_name);
                TubeRoot::Koren(
                    TriodeRoot::new(model).with_parallel_count(*parallel_count),
                )
            }
        } else {
            // Fallback — shouldn't happen.
            TubeRoot::Koren(TriodeRoot::new(TriodeModel::by_name("12AX7")))
        }
    };

    (build_root(push_elem), build_root(pull_elem))
}

/// Find the load resistor on the transformer's secondary side.
///
/// Looks up the secondary pin nodes (`.c` and `.d`) via `graph.node_names`
/// and searches for a resistor edge between them. Falls back to 600Ω
/// (standard line-level termination) if not found.
fn find_secondary_load_resistance(
    graph: &CircuitGraph,
    xfmr_comp_idx: usize,
) -> f64 {
    let comp = &graph.components[xfmr_comp_idx];
    let sec_a_key = format!("{}.c", comp.id);
    let sec_b_key = format!("{}.d", comp.id);

    if let (Some(&node_a), Some(&node_b)) = (
        graph.node_names.get(&sec_a_key),
        graph.node_names.get(&sec_b_key),
    ) {
        for edge in &graph.edges {
            if (edge.node_a == node_a && edge.node_b == node_b)
                || (edge.node_a == node_b && edge.node_b == node_a)
            {
                if let ComponentKind::Resistor(r) = &graph.components[edge.comp_idx].kind {
                    return *r;
                }
            }
        }
    }
    600.0 // Default: standard 600Ω line-level termination
}

/// Build a reflected-impedance sub-tree for the output transformer and add
/// it in series with an existing WDF tree.
///
/// Models:
/// - **Reflected secondary load**: `n_eff² × R_load` (resistive, determines DC load line)
/// - **Magnetizing inductance**: `L_primary/2` per half → LF rolloff modeled as
///   a DC-blocking high-pass on the secondary (avoids WDF inductor transient issues)
/// - **Primary DCR**: winding resistance (small, ~2.5Ω per half)
///
/// The reflected load is wrapped in a polarity-inverting ideal transformer (n = -1)
/// to cancel the sign flip introduced by the Series adaptor at the root. This
/// preserves the tube root's expected sign convention for the reflected wave.
fn wrap_with_transformer_load(
    tree: DynNode,
    turns_ratio: f64,
    r_load: f64,
    _l_primary: f64,
    primary_dcr: f64,
    is_ct: bool,
    _sample_rate: f64,
) -> DynNode {
    let n_eff = if is_ct { turns_ratio / 2.0 } else { turns_ratio };

    // Secondary: just the load resistor (referred to secondary side).
    // The magnetizing inductance LF rolloff is handled as a post-process
    // high-pass filter rather than an in-tree WDF inductor, avoiding
    // the DC transient/initialization issue that causes tube cutoff.
    let secondary = DynNode::Resistor { rp: r_load };

    // Ideal transformer: reflects secondary impedance to primary by n².
    let xfmr_rp = n_eff * n_eff * r_load;
    let mut xfmr: DynNode = DynNode::Transformer {
        secondary: Box::new(secondary),
        turns_ratio: n_eff,
        rp: xfmr_rp,
        b_sec: 0.0,
    };

    // Add primary DCR in series if non-zero.
    let xfmr_total_rp;
    if primary_dcr > 1e-6 {
        let dcr = if is_ct { primary_dcr / 2.0 } else { primary_dcr };
        let combined_rp = dcr + xfmr_rp;
        xfmr = DynNode::Series {
            left: Box::new(DynNode::Resistor { rp: dcr }),
            right: Box::new(xfmr),
            rp: combined_rp,
            gamma: dcr / combined_rp,
            b1: 0.0,
            b2: 0.0,
        };
        xfmr_total_rp = combined_rp;
    } else {
        xfmr_total_rp = xfmr_rp;
    }

    // Add in series with existing tree.
    //
    // The Series adaptor's root port reflected wave is b = -(b_tree + b_xfmr),
    // which NEGATES the Thevenin voltage. The tube root expects a positive
    // reflected wave for a positive supply. To correct the polarity, we wrap
    // the Series in an ideal transformer with n = -1 (polarity inverter).
    // This cancels the sign flip: b_out = n * b_series = -1 * (-(b_tree + b_xfmr))
    //                                    = b_tree + b_xfmr (correct sign)
    // Port resistance is unchanged: n² * rp = 1 * rp.
    let tree_rp = tree.port_resistance();
    let total_rp = tree_rp + xfmr_total_rp;
    let inner_series = DynNode::Series {
        left: Box::new(tree),
        right: Box::new(xfmr),
        rp: total_rp,
        gamma: tree_rp / total_rp,
        b1: 0.0,
        b2: 0.0,
    };
    DynNode::Transformer {
        secondary: Box::new(inner_series),
        turns_ratio: -1.0,
        rp: total_rp, // n² = 1
        b_sec: 0.0,
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Root creation factory
// ═══════════════════════════════════════════════════════════════════════════

/// Create the RootKind for a nonlinear element.
/// Returns (root, base_diode_model) — diode stages store their model.
fn create_root(kind: &NonlinearKind) -> (RootKind, Option<DiodeModel>) {
    match kind {
        NonlinearKind::DiodePair(dt) => {
            let model = diode_model(*dt);
            (RootKind::DiodePair(DiodePairRoot::new(model)), Some(model))
        }
        NonlinearKind::SingleDiode(dt) => {
            let model = diode_model(*dt);
            (RootKind::SingleDiode(DiodeRoot::new(model)), Some(model))
        }
        NonlinearKind::Jfet { model_name, is_n_channel } => {
            let model = jfet_model(model_name, *is_n_channel);
            (RootKind::Jfet(JfetRoot::new(model)), None)
        }
        NonlinearKind::BjtNpn { model_name, .. } => {
            let model = BjtModel::by_name(model_name);
            (RootKind::BjtNpn(BjtNpnRoot::new(model)), None)
        }
        NonlinearKind::BjtPnp { model_name, .. } => {
            let model = BjtModel::by_name(model_name);
            (RootKind::BjtPnp(BjtPnpRoot::new(model)), None)
        }
        NonlinearKind::Triode { model_name, parallel_count, is_vari_mu, .. } => {
            if *is_vari_mu {
                let model = vari_mu_model(model_name);
                (
                    RootKind::VariMu(
                        VariMuTriodeRoot::new(model).with_parallel_count(*parallel_count),
                    ),
                    None,
                )
            } else {
                let model = triode_model(model_name);
                (
                    RootKind::Triode(
                        TriodeRoot::new(model).with_parallel_count(*parallel_count),
                    ),
                    None,
                )
            }
        }
        NonlinearKind::Pentode { model_name } => {
            let model = pentode_model(model_name);
            (RootKind::Pentode(PentodeRoot::new(model)), None)
        }
        NonlinearKind::Mosfet { mosfet_type, is_n_channel } => {
            let model = mosfet_model(*mosfet_type, *is_n_channel);
            (RootKind::Mosfet(MosfetRoot::new(model)), None)
        }
        NonlinearKind::Zener { voltage } => {
            let model = ZenerModel::new(*voltage);
            (RootKind::Zener(ZenerRoot::new(model)), None)
        }
        NonlinearKind::Ota => {
            let model = OtaModel::ca3080();
            (RootKind::Ota(OtaRoot::new(model)), None)
        }
    }
}
