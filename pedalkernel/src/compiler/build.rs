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
use crate::elements::nonlinear::solver::{
    multi_port_nr_solve, multi_port_nr_solve_grouped, NlDeviceGroupIv, NlDeviceIv,
};
use crate::elements::*;
use crate::oversampling::{Oversampler, OversamplingFactor};
use crate::tree::{MnaSystem, RTypeAdaptor, ScatteringInterpolationTable, WdfPort};

use super::classify::{ClassifiedCircuit, NonlinearKind};
use super::component::StampResult;
use super::components::{
    CapSwitched, Capacitor as CapacitorComp, Potentiometer as PotComp,
};
use super::dyn_node::DynNode;
use super::graph::{sp_decompose, graph_reduce, ExtraEdge, CircuitGraph, NodeId};
use super::helpers::*;
use super::opamp_analysis::OpAmpAnalysis;
use super::plan::{MultiNlPlan, PushPullPlan, StagePlan};
use super::stage::{
    MultiNlDeviceGroups, MultiNlStage, NlDeviceGroupKind, NlDeviceKind, PushPullStage, RootKind,
    ScatteringRecomputeData, TubeRoot, WdfStage,
};

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
        let cfg = match comp.kind.transformer_config() {
            Some(cfg) => cfg,
            None => continue,
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
            graph
                .transformer_info
                .get(n)
                .map(|info| !info.is_secondary && info.comp_idx == edge.comp_idx)
                .unwrap_or(false)
        });

        if is_primary_side {
            // Case A: Stage is on the primary side (output transformer).
            // Build secondary subtree from BFS of secondary-side passives.
            if let Some(subtree) = build_other_side_subtree(edge.comp_idx, true, graph, sample_rate)
            {
                result.insert(
                    edge.comp_idx,
                    TransformerSubtree {
                        subtree,
                        turns_ratio: cfg.turns_ratio,
                    },
                );
            }
        } else {
            // Case B: Stage is on the secondary side (input transformer).
            // Build primary subtree from BFS of primary-side passives.
            // Turns ratio inverted: parent port faces secondary.
            if let Some(subtree) =
                build_other_side_subtree(edge.comp_idx, false, graph, sample_rate)
            {
                result.insert(
                    edge.comp_idx,
                    TransformerSubtree {
                        subtree,
                        turns_ratio: 1.0 / cfg.turns_ratio,
                    },
                );
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
    let target_nodes: Vec<NodeId> = graph
        .transformer_info
        .iter()
        .filter(|(_, info)| info.comp_idx == xfmr_comp_idx && info.is_secondary == build_secondary)
        .map(|(node, _)| *node)
        .collect();

    if target_nodes.is_empty() {
        return None;
    }

    // BFS from target nodes through passives, skipping ALL transformer edges
    // (prevents cascading into other transformers) and NL/active edges.
    let all_transformer_edges: HashSet<usize> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(_, e)| graph.components[e.comp_idx].kind.is_transformer())
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

            // Only traverse through simple passive components (R, C, L, etc.).
            if !graph.components[e.comp_idx].kind.is_simple_passive() {
                continue;
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
        return Some(DynNode::Resistor { rp: r_load, last_a: 0.0 });
    }

    // Terminals: the two target nodes (secondary or primary winding pins).
    let terminals = if target_nodes.len() >= 2 {
        vec![target_nodes[0], target_nodes[1]]
    } else {
        // Single-node (one secondary pin connects to ground) — use gnd as second terminal.
        vec![target_nodes[0], graph.gnd_node]
    };

    match graph_reduce(
        &collected_edges, &[], &terminals,
        graph, sample_rate, &HashMap::new(), |n| n,
        None,
    ) {
        Ok((tree, _)) => Some(tree),
        Err(_) => {
            // SP reduction failed — fall back to resistor stub.
            let r_load = find_secondary_load_resistance(graph, xfmr_comp_idx);
            Some(DynNode::Resistor { rp: r_load, last_a: 0.0 })
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
        DynNode::Transformer {
            secondary,
            turns_ratio,
            rp,
            ..
        } => {
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
        DynNode::Series {
            left,
            right,
            rp,
            gamma,
            ..
        } => {
            replace_transformer_stubs(left, transformer_subtrees, components);
            replace_transformer_stubs(right, transformer_subtrees, components);
            let r1 = left.port_resistance();
            let r2 = right.port_resistance();
            *rp = r1 + r2;
            if *rp > 0.0 {
                *gamma = r1 / *rp;
            }
        }
        DynNode::Parallel {
            left,
            right,
            rp,
            gamma,
            ..
        } => {
            replace_transformer_stubs(left, transformer_subtrees, components);
            replace_transformer_stubs(right, transformer_subtrees, components);
            let r1 = left.port_resistance();
            let r2 = right.port_resistance();
            *rp = r1 * r2 / (r1 + r2);
            let denom = r1 + r2;
            if denom > 0.0 {
                *gamma = r2 / denom;
            }
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
    lfo_controlled_jfets: &HashSet<String>,
    supply_voltage: f64,
    node_island_depths: &HashMap<super::graph::NodeId, usize>,
) -> (Vec<WdfStage>, Vec<MultiNlStage>, Vec<usize>) {
    // Build unity-gain feedback op-amp queue for JFET pairing.
    // Also collect their out_nodes for signal_flow_distance.
    let mut feedback_opamp_queue =
        super::opamp_analysis::build_unity_gain_queue(opamp_analysis, sample_rate);
    let unity_gain_out_nodes: Vec<super::graph::NodeId> = opamp_analysis
        .feedback_loops
        .iter()
        .filter(|info| matches!(info.feedback_kind, super::graph::OpAmpFeedbackKind::UnityGain))
        .map(|info| info.out_node)
        .collect();

    // Build AllpassJfet map: JFET comp_id → (rf, cf) for inverting all-pass stages.
    let allpass_jfet_map = super::opamp_analysis::build_allpass_jfet_map(opamp_analysis);

    let mut stages: Vec<WdfStage> = Vec::new();
    let mut fallback_multi_nl: Vec<MultiNlStage> = Vec::new();
    let mut fallback_claimed_edges: Vec<usize> = Vec::new();

    for plan in plans {
        let elem = &classified.nonlinear_elements[plan.element_idx];

        // Build real transformer subtrees for any transformers in this stage's passives.
        let transformer_subtrees = build_transformer_subtrees(
            &plan.passive_idxs,
            graph,
            pp_transformer_edges,
            sample_rate,
        );

        // Pentode + inductive load (transformer) → skip SP, go straight to MNA.
        // High plate impedance (40kΩ+) interacting with transformer reflected
        // load creates a stiff system that causes SP wave reflections to diverge.
        // Pentodes without transformers may still SP-reduce successfully.
        let pentode_with_transformer = matches!(&elem.kind, NonlinearKind::Pentode { .. })
            && plan.passive_idxs.iter().any(|&idx| {
                graph.components[graph.edges[idx].comp_idx].kind.is_transformer()
            });
        if pentode_with_transformer {
            let multi_nl_plan = stage_plan_to_multi_nl(plan, elem, classified, graph);
            if let Some(mut multi_nl) = try_build_multi_nl_stage(&multi_nl_plan, classified, graph, sample_rate, oversampling, supply_voltage) {
                multi_nl.transformer_gain =
                    compute_transformer_gain(&plan.passive_idxs, graph, &transformer_subtrees);
                // Use island-depth when available (boundary-split stages),
                // else element's BFS distance for ordering.
                multi_nl.signal_flow_distance = plan.signal_chain_depth.unwrap_or(elem.distance);
                fallback_multi_nl.push(multi_nl);
                fallback_claimed_edges.extend(&plan.passive_idxs);
            }
            continue;
        }

        // Check if this JFET should use variable resistance mode.
        let use_jfet_vr = if matches!(&elem.kind, NonlinearKind::Jfet { .. }) {
            let comp_id = &graph.components[graph.edges[elem.edge_idx].comp_idx].id;
            lfo_controlled_jfets.contains(comp_id)
        } else {
            false
        };

        let stage = if plan.skip_vs {
            build_source_follower_stage(plan, elem, graph, sample_rate, oversampling, use_jfet_vr)
        } else {
            build_vs_stage(plan, elem, graph, sample_rate, oversampling, use_jfet_vr)
        };

        if let Some(mut stage) = stage {
            // Replace transformer stubs with real subtrees if available.
            if !transformer_subtrees.is_empty() {
                replace_transformer_stubs(
                    &mut stage.tree,
                    &transformer_subtrees,
                    &graph.components,
                );
            }

            // Pair JFET stages with all-pass feedback or unity-gain op-amp buffers.
            if matches!(&elem.kind, NonlinearKind::Jfet { .. }) {
                let comp_id = &graph.components[graph.edges[elem.edge_idx].comp_idx].id;
                if let Some(&(rf, cf)) = allpass_jfet_map.get(comp_id) {
                    // Phase 90 inverting all-pass: build AllpassFeedback IIR.
                    let r_ap = plan
                        .passive_idxs
                        .iter()
                        .filter_map(|&idx| {
                            graph.components[graph.edges[idx].comp_idx].kind.resistance()
                        })
                        .next()
                        .unwrap_or(22_000.0);
                    let k = 2.0 * sample_rate * rf * cf;
                    stage.allpass_feedback =
                        Some(super::stage::AllpassFeedback {
                            r_ap,
                            b0: rf / (1.0 + k),
                            a1: (k - 1.0) / (k + 1.0),
                            x_prev: 0.0,
                            y_prev: 0.0,
                        });
                } else if !feedback_opamp_queue.is_empty() {
                    stage.paired_opamp = Some(feedback_opamp_queue.remove(0));
                }
            }

            stage.signal_flow_distance = if let Some(depth) = plan.signal_chain_depth {
                depth
            } else {
                classified
                    .dist_from_in
                    .get(&plan.injection_node)
                    .copied()
                    .unwrap_or(usize::MAX)
            };
            stage.transformer_gain =
                compute_transformer_gain(&plan.passive_idxs, graph, &transformer_subtrees);
            stages.push(stage);
        } else if matches!(
            &elem.kind,
            NonlinearKind::Triode { .. } | NonlinearKind::Pentode { .. }
        ) {
            // SP reduction failed for this triode/pentode — build as a
            // single-NL MNA stage using the plan's already-collected passives.
            let multi_nl_plan = stage_plan_to_multi_nl(plan, elem, classified, graph);
            if let Some(mut multi_nl) = try_build_multi_nl_stage(&multi_nl_plan, classified, graph, sample_rate, oversampling, supply_voltage) {
                multi_nl.transformer_gain =
                    compute_transformer_gain(&plan.passive_idxs, graph, &transformer_subtrees);
                multi_nl.signal_flow_distance = plan.signal_chain_depth.unwrap_or(elem.distance);
                fallback_multi_nl.push(multi_nl);
                fallback_claimed_edges.extend(&plan.passive_idxs);
            }
        } else if matches!(
            &elem.kind,
            NonlinearKind::SingleDiode(_) | NonlinearKind::DiodePair(_)
        ) {
            // SP reduction failed for this diode — build as single-NL MNA
            // stage using the plan's already-collected passives.
            let multi_nl_plan = stage_plan_to_multi_nl(plan, elem, classified, graph);
            if let Some(mut multi_nl) = try_build_multi_nl_stage(&multi_nl_plan, classified, graph, sample_rate, oversampling, supply_voltage) {
                multi_nl.transformer_gain =
                    compute_transformer_gain(&plan.passive_idxs, graph, &transformer_subtrees);
                multi_nl.signal_flow_distance = plan.signal_chain_depth.unwrap_or(elem.distance);
                fallback_multi_nl.push(multi_nl);
                fallback_claimed_edges.extend(&plan.passive_idxs);
            }
        }
    }

    // Handle remaining unity-gain op-amps that weren't paired with JFETs.
    // Track which unity gain opamp we're consuming from the parallel list.
    let consumed_count = unity_gain_out_nodes.len() - feedback_opamp_queue.len();
    for (qi, opamp) in feedback_opamp_queue.drain(..).enumerate() {
        let out_node_idx = consumed_count + qi;
        let out_node = unity_gain_out_nodes.get(out_node_idx).copied();
        let sfd = out_node
            .and_then(|n| node_island_depths.get(&n).copied())
            .unwrap_or(usize::MAX);
        let tree = DynNode::VoltageSource {
            voltage: 0.0,
            rp: 10_000.0,
        };
        stages.push(WdfStage {
            tree,
            root: RootKind::OpAmp(opamp),
            compensation: 1.0,
            oversampler: Oversampler::new(oversampling),
            base_diode_model: None,
            paired_opamp: None,
            allpass_feedback: None,
            dc_block: None,
            is_source_follower: false,
            prev_source_voltage: 0.0,
            signal_flow_distance: sfd,
            transformer_gain: 1.0,
            injection_node_id: out_node.unwrap_or(usize::MAX),
            output_node_id: out_node.unwrap_or(usize::MAX),
            is_trigger_voice: false,
            is_feedforward: false,
            sample_counter: 0,
            root_comp_id: String::new(),
            feedback_pot_id: None,
            output_probe: None,
        });
    }

    // Balance voltage source impedance in each stage.
    for stage in &mut stages {
        stage.balance_vs_impedance();
    }

    // Adjust reactive element port resistances for oversampling.
    // Must be done AFTER balance_vs_impedance (which calls recompute).
    for stage in &mut stages {
        stage.apply_oversampling_rate(sample_rate);
    }
    for stage in &mut fallback_multi_nl {
        stage.apply_oversampling_rate(sample_rate);
    }

    (stages, fallback_multi_nl, fallback_claimed_edges)
}

/// Append `src` elements to `dst`, skipping duplicates.
fn extend_dedup_vec(dst: &mut Vec<usize>, src: &[usize]) {
    for &v in src {
        if !dst.contains(&v) {
            dst.push(v);
        }
    }
}

/// Convert a StagePlan (single-NL WDF plan) into a MultiNlPlan for MNA fallback.
///
/// When SP reduction fails, the plan's 1-hop passive_idxs are insufficient
/// for MNA — we need the full passive network around the NL element. This
/// function does multi-hop BFS from the NL terminals (like the old fallback
/// functions did) to collect all reachable passives for the MNA builder.
fn stage_plan_to_multi_nl(
    plan: &StagePlan,
    elem: &super::classify::NonlinearElement,
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
) -> MultiNlPlan {
    let empty_pp = HashSet::new();

    let (nl_terminals, passive_edges) = match &elem.kind {
        NonlinearKind::Triode {
            grid_node: Some(gn),
            ..
        } => {
            let (plate, cathode) = (elem.junction_nodes[0], elem.junction_nodes[1]);
            let terminals = vec![(*gn, cathode), (plate, cathode)];

            // BFS from plate, cathode, and grid to collect full passive network.
            let mut edges = graph.bfs_passive_edges(
                plate,
                &classified.all_nonlinear_edge_indices,
                &graph.active_edge_indices,
                true, false, &empty_pp, &HashSet::new(),
            );
            if cathode != graph.gnd_node {
                let cathode_edges = graph.bfs_passive_edges(
                    cathode,
                    &classified.all_nonlinear_edge_indices,
                    &graph.active_edge_indices,
                    true, false, &empty_pp, &HashSet::new(),
                );
                extend_dedup_vec(&mut edges, &cathode_edges);
            }
            if *gn != graph.gnd_node {
                let grid_edges = graph.bfs_passive_edges(
                    *gn,
                    &classified.all_nonlinear_edge_indices,
                    &graph.active_edge_indices,
                    true, false, &empty_pp, &HashSet::new(),
                );
                extend_dedup_vec(&mut edges, &grid_edges);
            }

            (terminals, edges)
        }
        NonlinearKind::Triode { .. } | NonlinearKind::Pentode { .. } => {
            let (plate, cathode) = (elem.junction_nodes[0], elem.junction_nodes[1]);
            let terminals = vec![(plate, cathode)];

            let mut edges = graph.bfs_passive_edges(
                plate,
                &classified.all_nonlinear_edge_indices,
                &graph.active_edge_indices,
                true, false, &empty_pp, &HashSet::new(),
            );
            if cathode != graph.gnd_node {
                let cathode_edges = graph.bfs_passive_edges(
                    cathode,
                    &classified.all_nonlinear_edge_indices,
                    &graph.active_edge_indices,
                    true, false, &empty_pp, &HashSet::new(),
                );
                extend_dedup_vec(&mut edges, &cathode_edges);
            }

            (terminals, edges)
        }
        NonlinearKind::SingleDiode(_) | NonlinearKind::DiodePair(_) => {
            let junction = elem.junction_nodes[0];
            let edge = &graph.edges[elem.edge_idx];
            let other = if edge.node_a == junction {
                edge.node_b
            } else {
                edge.node_a
            };
            let terminals = vec![(junction, other)];

            // BFS from both diode terminals to capture full passive network.
            let mut edges = graph.bfs_passive_edges(
                junction,
                &classified.all_nonlinear_edge_indices,
                &graph.active_edge_indices,
                true, false, &empty_pp, &HashSet::new(),
            );
            if other != graph.gnd_node && other != graph.vcc_node
                && !graph.supply_nodes.contains(&other)
            {
                let other_edges = graph.bfs_passive_edges(
                    other,
                    &classified.all_nonlinear_edge_indices,
                    &graph.active_edge_indices,
                    true, false, &empty_pp, &HashSet::new(),
                );
                extend_dedup_vec(&mut edges, &other_edges);
            }

            (terminals, edges)
        }
        _ => {
            let terminals = vec![(elem.junction_nodes[0], graph.gnd_node)];
            (terminals, plan.passive_idxs.clone())
        }
    };

    // Find injection node: closest to input among passive edge endpoints,
    // excluding NL terminal nodes and global nodes.
    let nl_nodes: HashSet<super::graph::NodeId> = nl_terminals
        .iter()
        .flat_map(|&(a, b)| [a, b])
        .collect();
    let mut injection_node = plan.injection_node;
    let mut best_dist = classified
        .dist_from_in
        .get(&injection_node)
        .copied()
        .unwrap_or(usize::MAX);
    for &eidx in &passive_edges {
        let e = &graph.edges[eidx];
        for candidate in [e.node_a, e.node_b] {
            if nl_nodes.contains(&candidate) {
                continue;
            }
            if candidate == graph.gnd_node
                || candidate == graph.vcc_node
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

    MultiNlPlan {
        nl_element_indices: vec![plan.element_idx],
        output_element_idx: plan.element_idx,
        passive_edge_indices: passive_edges,
        injection_node,
        nl_terminals,
        compensation: plan.compensation,
        output_node: None,
        ota_vccs: Vec::new(),
        signal_chain_depth: plan.signal_chain_depth,
    }
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
    let mut stages = Vec::new();

    for pp_plan in push_pull_plans {
        let push_elem = &classified.nonlinear_elements[pp_plan.push_triode_list_idx];
        let pull_elem = &classified.nonlinear_elements[pp_plan.pull_triode_list_idx];

        let push_plan =
            super::plan::plan_push_pull_half(push_elem, classified, graph, pp_transformer_edges);
        let pull_plan =
            super::plan::plan_push_pull_half(pull_elem, classified, graph, pp_transformer_edges);

        if let (Some(push_plan), Some(pull_plan)) = (push_plan, pull_plan) {
            let push_half = build_push_pull_half(&push_plan, graph, sample_rate);
            let pull_half = build_push_pull_half(&pull_plan, graph, sample_rate);

            if let (Some((push_tree, push_comp)), Some((pull_tree, _))) = (push_half, pull_half) {
                // Look up transformer config and wrap each half with reflected load.
                let xfmr_edge = &graph.edges[pp_plan.transformer_edge_idx];
                let xfmr_cfg = match graph.components[xfmr_edge.comp_idx].kind.transformer_config() {
                    Some(cfg) => cfg,
                    None => {
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

    // Adjust reactive element port resistances for oversampling.
    for stage in &mut stages {
        stage.apply_oversampling_rate(sample_rate);
    }

    stages
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
/// Plans that fail MNA construction are skipped with a warning.
pub(super) fn build_multi_nl_stages(
    multi_nl_plans: &[MultiNlPlan],
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    sample_rate: f64,
    oversampling: OversamplingFactor,
    supply_voltage: f64,
) -> Vec<MultiNlStage> {
    let mut multi_nl_stages = Vec::new();

    for plan in multi_nl_plans {
        match try_build_multi_nl_stage(plan, classified, graph, sample_rate, oversampling, supply_voltage) {
            Some(mut stage) => {
                let empty_subtrees = HashMap::new();
                stage.transformer_gain =
                    compute_transformer_gain(&plan.passive_edge_indices, graph, &empty_subtrees);
                multi_nl_stages.push(stage);
            }
            None => {
                eprintln!(
                    "Warning: multi-NL stage build failed for elements {:?}, skipping",
                    plan.nl_element_indices
                );
            }
        }
    }

    // Adjust reactive element port resistances for oversampling.
    for stage in &mut multi_nl_stages {
        stage.apply_oversampling_rate(sample_rate);
    }

    multi_nl_stages
}

/// Stamp a single passive edge into the MNA system or collect it as a reactive port.
///
/// Fixed-resistance components (resistors, tempcos, switched resistors) are stamped
/// directly into the MNA conductance matrix. Reactive components (capacitors,
/// inductors, switched caps/inductors) become WDF ports with their own DynNode.
/// Potentiometers are stamped into the G matrix as conductances (not WDF ports)
/// and tracked in `pot_entries` for the `set_pot()` API.
fn stamp_passive_edge(
    eidx: usize,
    graph: &CircuitGraph,
    mna: &mut MnaSystem,
    reactive_edges: &mut Vec<(usize, DynNode)>,
    pot_entries: &mut Vec<(usize, DynNode, Option<usize>, Option<usize>, f64)>,
    coupled_edge_indices: &HashSet<usize>,
    node_to_mna: &dyn Fn(NodeId) -> Option<usize>,
    sample_rate: f64,
) {
    let e = &graph.edges[eidx];
    let comp = &graph.components[e.comp_idx];
    let n1 = node_to_mna(e.node_a);
    let n2 = node_to_mna(e.node_b);

    // Transformer needs coupled_edge_indices context — handle specially.
    if let Some(cfg) = comp.kind.transformer_config() {
        if coupled_edge_indices.contains(&eidx) {
            if cfg.primary_dcr > 0.0 {
                mna.stamp_resistor(n1, n2, cfg.primary_dcr);
            }
        } else {
            let l = cfg.primary_inductance;
            if l > 0.0 && l.is_finite() {
                reactive_edges.push((
                    eidx,
                    DynNode::Inductor {
                        inductance: l,
                        rp: 2.0 * sample_rate * l,
                        state: 0.0,
                    },
                ));
            }
            if cfg.primary_dcr > 0.0 {
                mna.stamp_resistor(n1, n2, cfg.primary_dcr);
            }
        }
        return;
    }

    // All other passives: delegate to Component trait.
    match comp.kind.stamp_mna(&comp.id, n1, n2, mna, sample_rate) {
        StampResult::Stamped => {}
        StampResult::Reactive { dyn_node, .. } => {
            reactive_edges.push((eidx, dyn_node));
        }
        StampResult::Pot {
            dyn_node,
            initial_conductance,
        } => {
            pot_entries.push((eidx, dyn_node, n1, n2, initial_conductance));
        }
        StampResult::Skip => {}
    }
}

/// Build an R-type stage from a decomposed circuit.
///
/// This is the unified MNA builder that replaces `try_build_multi_nl_stage`,
/// `build_triode_mna_fallback`, and `build_diode_mna_fallback`. It takes the
/// output of `sp_decompose` which pre-extracts SP-reducible pendant subtrees
/// as WDF ports, reducing the size of the MNA system.
///
/// Port ordering: [NL_0..NL_{n-1}, subtree_0..subtree_{s-1}, reactive_0..reactive_{m-1}, (adapted)]
///
/// Returns `None` if MNA construction fails (e.g., singular matrix).
#[allow(dead_code)]
fn build_rtype_stage(
    decomposed: &super::graph::DecomposedCircuit,
    plan: &MultiNlPlan,
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    sample_rate: f64,
    oversampling: OversamplingFactor,
    supply_voltage: f64,
) -> Option<MultiNlStage> {
    let n_nl = plan.nl_terminals.len();
    let has_linearized_ota = !plan.ota_vccs.is_empty();

    // Compute effective sample rate accounting for oversampling.
    // All DynNodes (caps, inductors) must be created at this rate so that
    // their port resistances match the scattering matrix. Previously, the
    // scattering was built at base rate and oversampling applied after,
    // causing a mismatch between port R and the S matrix.
    let effective_rate = sample_rate * oversampling.ratio() as f64;

    // Need either NL ports or linearized OTA, and at least some passive content.
    if n_nl == 0 && !has_linearized_ota {
        return None;
    }
    if decomposed.residual_edges.is_empty() && decomposed.wdf_subtrees.is_empty() {
        return None;
    }

    // ── Step 1: Collect unique circuit nodes for MNA ────────────────────
    // Ground and supply nodes are AC ground (excluded from MNA).
    let mut node_set: Vec<NodeId> = Vec::new();
    {
        let mut add_node = |node: NodeId| {
            if node == graph.gnd_node || node == graph.vcc_node || graph.supply_nodes.contains(&node) {
                return;
            }
            if !node_set.contains(&node) {
                node_set.push(node);
            }
        };

        // Nodes from residual edges.
        for &eidx in &decomposed.residual_edges {
            let e = &graph.edges[eidx];
            add_node(e.node_a);
            add_node(e.node_b);
        }

        // Nodes from WDF subtree attachment points.
        for subtree in &decomposed.wdf_subtrees {
            add_node(subtree.attachment_node);
        }

        // Nodes from NL terminals.
        for &(pos, neg) in &plan.nl_terminals {
            add_node(pos);
            add_node(neg);
        }

        // Nodes from linearized OTA VCCS stamps.
        for ota in &plan.ota_vccs {
            add_node(ota.in_pos);
            add_node(ota.in_neg);
            add_node(ota.out_node);
        }

        // Injection node (voltage source input).
        add_node(plan.injection_node);
    }

    // ── Detect VCC-connected passive edges ───────────────────────────
    // If any residual edges or NL terminals touch VCC AND we have NL ports,
    // stamp VCC as an ideal voltage source in the MNA to inject DC supply bias.
    // Without this, VCC is treated as AC ground and transistor bias is lost.
    // Only needed for NL stages (n_nl > 0); OTA stages (n_nl == 0) treat VCC
    // as AC ground which is correct for small-signal linearized analysis.
    let needs_vcc_port = n_nl > 0 && (decomposed.residual_edges.iter().any(|&eidx| {
        let e = &graph.edges[eidx];
        e.node_a == graph.vcc_node || e.node_b == graph.vcc_node
    }) || plan.nl_terminals.iter().any(|&(p, n)| {
        p == graph.vcc_node || n == graph.vcc_node
    }));

    if needs_vcc_port {
        if !node_set.contains(&graph.vcc_node) {
            node_set.push(graph.vcc_node);
        }
    }

    // ── Pre-scan for coupled transformers ────────────────────────────
    struct CoupledXfmr {
        comp_idx: usize,
        primary_edge_idx: usize,
        sec_node_a: NodeId,
        sec_node_b: NodeId,
        turns_ratio: f64,
    }
    let mut coupled_transformers: Vec<CoupledXfmr> = Vec::new();
    {
        let mut seen_comp: HashSet<usize> = HashSet::new();
        for &eidx in &decomposed.residual_edges {
            let e = &graph.edges[eidx];
            let comp = &graph.components[e.comp_idx];
            if let Some(cfg) = comp.kind.transformer_config() {
                if seen_comp.contains(&e.comp_idx) {
                    continue;
                }
                seen_comp.insert(e.comp_idx);
                let sec_pin_names: &[(&str, &str)] = &[
                    ("c", "d"),
                    ("secondary.a", "secondary.b"),
                    ("sec.a", "sec.b"),
                ];
                let mut sec_a_node: Option<NodeId> = None;
                let mut sec_b_node: Option<NodeId> = None;
                for &(pin_a, pin_b) in sec_pin_names {
                    let key_a = format!("{}.{}", comp.id, pin_a);
                    let key_b = format!("{}.{}", comp.id, pin_b);
                    if sec_a_node.is_none() {
                        if let Some(&n) = graph.node_names.get(&key_a) {
                            sec_a_node = Some(n);
                        }
                    }
                    if sec_b_node.is_none() {
                        if let Some(&n) = graph.node_names.get(&key_b) {
                            sec_b_node = Some(n);
                        }
                    }
                }
                if let (Some(sna), Some(snb)) = (sec_a_node, sec_b_node) {
                    let sec_in_circuit = [sna, snb].iter().any(|&sn| {
                        decomposed.residual_edges.iter().any(|&ei| {
                            let pe = &graph.edges[ei];
                            ei != eidx && (pe.node_a == sn || pe.node_b == sn)
                        })
                        || plan.nl_terminals.iter().any(|&(p, n)| p == sn || n == sn)
                    });
                    if sec_in_circuit {
                        // Add secondary nodes (skip gnd/supply, like add_node would)
                        for &sn in &[sna, snb] {
                            if sn != graph.gnd_node && sn != graph.vcc_node
                                && !graph.supply_nodes.contains(&sn)
                                && !node_set.contains(&sn)
                            {
                                node_set.push(sn);
                            }
                        }
                        coupled_transformers.push(CoupledXfmr {
                            comp_idx: e.comp_idx,
                            primary_edge_idx: eidx,
                            sec_node_a: sna,
                            sec_node_b: snb,
                            turns_ratio: cfg.turns_ratio,
                        });
                    }
                }
            }
        }
    }
    let coupled_edge_indices: HashSet<usize> = coupled_transformers
        .iter()
        .map(|ct| ct.primary_edge_idx)
        .collect();
    let mut num_vsources = coupled_transformers.len() * 2;
    if has_linearized_ota && n_nl == 0 {
        num_vsources += 1;
    }
    // VCC voltage source: ideal supply with zero internal impedance.
    // Stamped into the MNA B/C matrices so VCC node voltage is exact.
    let vcc_vs_idx = if needs_vcc_port {
        let idx = num_vsources;
        num_vsources += 1;
        Some(idx)
    } else {
        None
    };

    let num_mna_nodes = node_set.len();
    if num_mna_nodes == 0 {
        return None;
    }

    let node_to_mna = |node: NodeId| -> Option<usize> {
        if node == graph.gnd_node || graph.supply_nodes.contains(&node) {
            None
        } else if node == graph.vcc_node && !needs_vcc_port {
            None
        } else {
            node_set.iter().position(|&n| n == node)
        }
    };

    // ── Step 2: Build MNA — stamp only residual (bridging) edges ────────
    let mut reactive_edges: Vec<(usize, DynNode)> = Vec::new();
    let mut mna = MnaSystem::new(num_mna_nodes, num_vsources);

    // Stamp coupled transformer constraints.
    for (ti, ct) in coupled_transformers.iter().enumerate() {
        let e = &graph.edges[ct.primary_edge_idx];
        let p_pos = node_to_mna(e.node_a);
        let p_neg = node_to_mna(e.node_b);
        let s_pos = node_to_mna(ct.sec_node_a);
        let s_neg = node_to_mna(ct.sec_node_b);
        let vsrc_p = ti * 2;
        let vsrc_s = ti * 2 + 1;
        let n = ct.turns_ratio;

        mna.stamp_voltage_source(p_pos, p_neg, vsrc_p);
        mna.stamp_voltage_source(s_pos, s_neg, vsrc_s);

        if let Some(sp) = s_pos {
            mna.c_matrix[vsrc_p * num_mna_nodes + sp] += -n;
        }
        if let Some(sn) = s_neg {
            mna.c_matrix[vsrc_p * num_mna_nodes + sn] += n;
        }
        if let Some(sp) = s_pos {
            mna.c_matrix[vsrc_s * num_mna_nodes + sp] = 0.0;
        }
        if let Some(sn) = s_neg {
            mna.c_matrix[vsrc_s * num_mna_nodes + sn] = 0.0;
        }
        mna.d_matrix[vsrc_s * num_vsources + vsrc_p] = n;
        mna.d_matrix[vsrc_s * num_vsources + vsrc_s] = 1.0;
    }

    // State-space mode for linearized OTA.
    let use_state_space = has_linearized_ota && n_nl == 0;
    let mut cap_stamps: Vec<(Option<usize>, Option<usize>, f64)> = Vec::new();
    let mut pot_entries: Vec<(usize, DynNode, Option<usize>, Option<usize>, f64)> = Vec::new();

    // Stamp only residual edges (bridging resistors between R-type nodes).
    for &eidx in &decomposed.residual_edges {
        if use_state_space {
            let e = &graph.edges[eidx];
            let comp = &graph.components[e.comp_idx];
            let n1 = node_to_mna(e.node_a);
            let n2 = node_to_mna(e.node_b);

            if let Some(cap) = comp.kind.as_any().downcast_ref::<CapacitorComp>() {
                cap_stamps.push((n1, n2, cap.config.value));
                continue;
            } else if let Some(cs) = comp.kind.as_any().downcast_ref::<CapSwitched>() {
                if let Some(&c) = cs.values.first() {
                    if c.is_finite() && c > 0.0 {
                        cap_stamps.push((n1, n2, c));
                    }
                }
                continue;
            } else if let Some(pot) = comp.kind.as_any().downcast_ref::<PotComp>() {
                let initial_pos = 0.5;
                let tapered_pos = pot.taper.apply(initial_pos);
                let r = (tapered_pos * pot.max_r).max(1.0);
                mna.stamp_resistor(n1, n2, r);
                pot_entries.push((
                    eidx,
                    DynNode::Pot {
                        comp_id: comp.id.clone(),
                        max_resistance: pot.max_r,
                        position: initial_pos,
                        taper: pot.taper,
                        rp: r,
                        last_a: 0.0,
                    },
                    n1,
                    n2,
                    1.0 / r,
                ));
                continue;
            }
        }
        stamp_passive_edge(
            eidx,
            graph,
            &mut mna,
            &mut reactive_edges,
            &mut pot_entries,
            &coupled_edge_indices,
            &node_to_mna,
            effective_rate,
        );
    }

    // ── GMIN regularization ─────────────────────────────────────────────
    const GMIN_RESISTANCE: f64 = 1e9;
    for i in 0..num_mna_nodes {
        mna.stamp_resistor(Some(i), None, GMIN_RESISTANCE);
    }

    // ── Stamp VCC as ideal voltage source ───────────────────────────────
    if let Some(vcc_idx) = vcc_vs_idx {
        let vcc_mna = node_to_mna(graph.vcc_node);
        mna.stamp_voltage_source(vcc_mna, None, vcc_idx);
    }

    // ── Stamp linearized OTA VCCS ──────────────────────────────────────
    use crate::elements::OtaModel;
    let mut linearized_ota_data: Option<super::stage::LinearizedOtaData> = None;
    if has_linearized_ota {
        let ota_info = &plan.ota_vccs[0];
        let model = OtaModel::ca3080();
        let gm_max = model.iabc_max / (2.0 * model.vt);
        let initial_gain = 1.0;

        let out_mna = node_to_mna(ota_info.out_node);
        let in_pos_mna = node_to_mna(ota_info.in_pos);
        let in_neg_mna = node_to_mna(ota_info.in_neg);

        let mut stamp_cells: Vec<(usize, usize, f64)> = Vec::new();
        if let Some(op) = out_mna {
            if let Some(ip) = in_pos_mna {
                stamp_cells.push((op, ip, 1.0));
            }
            if let Some(inn) = in_neg_mna {
                stamp_cells.push((op, inn, -1.0));
            }
        }

        mna.stamp_vccs(out_mna, None, in_pos_mna, in_neg_mna, gm_max * initial_gain);

        linearized_ota_data = Some(super::stage::LinearizedOtaData {
            model,
            gain: initial_gain,
            stamp_cells,
            num_mna_nodes,
        });
    }

    // ── State-space path (same as try_build_multi_nl_stage) ─────────────
    if use_state_space {
        let vs_idx = num_vsources - 1;
        let injection_mna = node_to_mna(plan.injection_node);
        mna.stamp_voltage_source(injection_mna, None, vs_idx);

        let out_circuit = plan
            .output_node
            .unwrap_or_else(|| plan.ota_vccs[0].out_node);
        let out_mna = node_to_mna(out_circuit);

        let (a_d, b_d, c_out, n_states) =
            mna.build_state_space_matrices(&cap_stamps, vs_idx, out_mna, None, effective_rate);

        if a_d.iter().any(|v| !v.is_finite()) || b_d.iter().any(|v| !v.is_finite()) {
            return None;
        }

        let has_pots = !pot_entries.is_empty();
        let mut pot_children: Vec<DynNode> = Vec::with_capacity(pot_entries.len());
        let mut pot_mna_stamps: Vec<(usize, Option<usize>, Option<usize>, f64)> =
            Vec::with_capacity(pot_entries.len());
        for (i, (_eidx, dyn_node, n1, n2, g)) in pot_entries.into_iter().enumerate() {
            pot_children.push(dyn_node);
            pot_mna_stamps.push((i, n1, n2, g));
        }

        let passive_children: Vec<DynNode> = Vec::new();
        let port_node_pairs: Vec<(Option<usize>, Option<usize>)> = Vec::new();
        let dummy_s = vec![1.0];
        let adaptor = RTypeAdaptor::new(dummy_s, &[1000.0]);
        let scattering_blocks =
            super::stage::MultiNlScattering::from_full_matrix(&[0.0; 0], 0, 0);
        let pot_stamps_ss: Vec<(usize, Option<usize>, Option<usize>, f64)> =
            pot_mna_stamps.clone();

        let recompute_data = if has_pots || has_linearized_ota {
            Some(ScatteringRecomputeData {
                mna,
                port_node_pairs,
                adapted_resistance: 1000.0,
                vs_source_index: Some(vs_idx),
                vcc_vs_index: None,
                extract_output_nodes: Some((out_mna, None)),
            })
        } else {
            None
        };

        let signal_flow_distance = plan.signal_chain_depth.unwrap_or_else(|| {
            classified
                .dist_from_in
                .get(&plan.injection_node)
                .copied()
                .unwrap_or(usize::MAX)
        });

        let state_space_data = super::stage::StateSpaceData {
            x: vec![0.0; n_states],
            a_matrix: a_d,
            b_vector: b_d,
            c_vector: c_out,
            n_states,
            cap_stamps,
            vs_idx,
            output_pos: out_mna,
            output_neg: None,
            sample_rate: effective_rate,
            pot_stamps: pot_stamps_ss,
        };

        return Some(MultiNlStage {
            adaptor,
            nl_devices: Vec::new(),
            nl_port_resistances: Vec::new(),
            passive_children,
            pot_children,
            pot_mna_stamps,
            n_nl: 0,
            v_prev: Vec::new(),
            scattering: scattering_blocks,
            oversampler: Oversampler::new(oversampling),
            compensation: plan.compensation,
            output_port: 0,
            device_groups: None,
            recompute_data,
            signal_flow_distance,
            transformer_gain: 1.0,
            injection_node_id: plan.injection_node,
            output_node_id: out_circuit,
            recompute_pending: false,
            veb_bias_offset: 0.0,
            feedback_scale: 0.1,
            linearized_ota: linearized_ota_data,
            vs_injection: None,
            extract_coeffs: None,
            extract_vs: 0.0,
            state_space: Some(state_space_data),
            bias_pot_id: None,
            bias_emitter_r: 470.0,
            interp_table: None, // state-space stages excluded from tables
            dc_bias: Vec::new(),
            vcc_bias_all: Vec::new(),
            vcc_vs_index: None,
            supply_voltage: 0.0,
            dc_blocker_x1: 0.0,
            dc_blocker_y1: 0.0,
            dc_ramp: 0,
            initial_v_prev: Vec::new(),
        });
    }

    // ── Step 3: Build WDF ports ─────────────────────────────────────────
    // Port ordering: [NL_0..NL_{n-1}, subtree_0..subtree_{s-1}, reactive_0..reactive_{m-1}, (adapted)]
    let n_subtrees = decomposed.wdf_subtrees.len();
    let n_reactive = reactive_edges.len();

    let mut ports: Vec<WdfPort> = Vec::with_capacity(n_nl + n_subtrees + n_reactive + 1);
    let mut port_node_pairs: Vec<(Option<usize>, Option<usize>)> =
        Vec::with_capacity(n_nl + n_subtrees + n_reactive + 1);
    let has_pots = !pot_entries.is_empty();

    // NL ports.
    // Initial NL port resistance: use geometric mean of resistive passive ports
    // (R > 10Ω to exclude bilinear-transformed capacitors which have sub-ohm R).
    // Falls back to 1kΩ if no resistive passives available.
    // The adaptive Thevenin step (step 4) will then refine this per-port.
    let r_nl_default = {
        let resistive_r: Vec<f64> = decomposed
            .wdf_subtrees
            .iter()
            .filter_map(|st| {
                let r = st.tree.port_resistance();
                if r > 10.0 { Some(r) } else { None }
            })
            .chain(reactive_edges.iter().filter_map(|(_, dn)| {
                let r = dn.port_resistance();
                if r > 10.0 { Some(r) } else { None }
            }))
            .collect();
        if !resistive_r.is_empty() {
            let log_sum: f64 = resistive_r.iter().map(|r| r.ln()).sum();
            (log_sum / resistive_r.len() as f64).exp()
        } else {
            1000.0
        }
    };
    let mut nl_port_resistances = Vec::with_capacity(n_nl);
    for i in 0..n_nl {
        let (pos_node, neg_node) = plan.nl_terminals[i];
        let pos = node_to_mna(pos_node);
        let neg = node_to_mna(neg_node);
        ports.push(WdfPort {
            node_pos: pos,
            node_neg: neg,
            resistance: r_nl_default,
        });
        port_node_pairs.push((pos, neg));
        nl_port_resistances.push(r_nl_default);
    }

    // WDF subtree ports — each pre-extracted pendant becomes a single port.
    let mut subtree_children: Vec<DynNode> = Vec::with_capacity(n_subtrees);
    for subtree in &decomposed.wdf_subtrees {
        let rp = subtree.tree.port_resistance();
        let pos = node_to_mna(subtree.attachment_node);
        let neg = node_to_mna(subtree.far_node);
        ports.push(WdfPort {
            node_pos: pos,
            node_neg: neg,
            resistance: rp,
        });
        port_node_pairs.push((pos, neg));
        subtree_children.push(subtree.tree.clone());
    }

    // Reactive ports from residual edges.
    let reactive_edge_indices: Vec<usize> = reactive_edges.iter().map(|(eidx, _)| *eidx).collect();
    let mut reactive_children: Vec<DynNode> = Vec::with_capacity(n_reactive);
    for (eidx, dyn_node) in reactive_edges {
        let e = &graph.edges[eidx];
        let pos = node_to_mna(e.node_a);
        let neg = node_to_mna(e.node_b);
        let rp = dyn_node.port_resistance();
        ports.push(WdfPort {
            node_pos: pos,
            node_neg: neg,
            resistance: rp,
        });
        port_node_pairs.push((pos, neg));
        reactive_children.push(dyn_node);
    }

    // Combine subtree + reactive children as passive_children.
    // Subtree children come first (matching port ordering).
    let mut passive_children: Vec<DynNode> =
        Vec::with_capacity(n_subtrees + n_reactive);
    passive_children.extend(subtree_children);
    passive_children.extend(reactive_children);
    let n_passive = passive_children.len();

    // Build pot_children and pot_mna_stamps.
    let mut pot_children: Vec<DynNode> = Vec::with_capacity(pot_entries.len());
    let mut pot_mna_stamps: Vec<(usize, Option<usize>, Option<usize>, f64)> =
        Vec::with_capacity(pot_entries.len());
    for (i, (_eidx, dyn_node, n1, n2, g)) in pot_entries.into_iter().enumerate() {
        pot_children.push(dyn_node);
        pot_mna_stamps.push((i, n1, n2, g));
    }

    // ── Step 3b: Voltage source injection ──────────────────────────────
    let injection_mna = node_to_mna(plan.injection_node);
    let mut r_adapted = 1000.0;
    let mut vs_injection_vec: Option<Vec<f64>> = None;

    if has_linearized_ota && n_nl == 0 {
        let vs_idx = num_vsources - 1;
        mna.stamp_voltage_source(injection_mna, None, vs_idx);
    } else {
        ports.push(WdfPort {
            node_pos: injection_mna,
            node_neg: None,
            resistance: r_adapted,
        });
        port_node_pairs.push((injection_mna, None));
    }

    // ── Step 4: Derive scattering matrix ────────────────────────────────
    let mut vcc_injection_vec: Option<Vec<f64>> = None;
    let mut scattering = if has_linearized_ota && n_nl == 0 {
        // OTA stage: signal input as VS
        let vs_idx = num_vsources - 1;
        let (s, vs_inj) = mna.derive_scattering_and_vs_injection(&ports, vs_idx);
        if s.iter().any(|&sv| !sv.is_finite()) {
            return None;
        }
        vs_injection_vec = Some(vs_inj);
        s
    } else if let Some(vcc_idx) = vcc_vs_idx {
        // NL stage with VCC supply: VCC as VS, signal input as adapted WDF port
        let (s, vcc_inj) = mna.derive_scattering_and_vs_injection(&ports, vcc_idx);
        if s.iter().any(|&sv| !sv.is_finite()) {
            return None;
        }
        vcc_injection_vec = Some(vcc_inj);
        s
    } else {
        let s = mna.derive_scattering_matrix_general(&ports);
        if s.iter().any(|&sv| !sv.is_finite()) {
            return None;
        }
        s
    };

    let n_total = ports.len();

    // ── Adaptive port resistance ────────────────────────────────────────
    // Helper closure: recompute scattering (and VCC injection if applicable)
    let recompute_scattering = |mna: &MnaSystem, ports: &[WdfPort], vcc_vs: Option<usize>| -> Option<(Vec<f64>, Option<Vec<f64>>)> {
        if let Some(vcc_idx) = vcc_vs {
            let (s, vcc_inj) = mna.derive_scattering_and_vs_injection(ports, vcc_idx);
            if s.iter().any(|&sv| !sv.is_finite()) { return None; }
            Some((s, Some(vcc_inj)))
        } else {
            let s = mna.derive_scattering_matrix_general(ports);
            if s.iter().any(|&sv| !sv.is_finite()) { return None; }
            Some((s, None))
        }
    };

    if vs_injection_vec.is_none() {
        let mut needs_recompute = false;

        // Adapt NL port resistances.
        for i in 0..n_nl {
            let s_refl = scattering[i * n_total + i];
            if s_refl.abs() > 0.1 && (1.0 - s_refl.abs()) > 1e-12 {
                let z_th = nl_port_resistances[i] * (1.0 + s_refl) / (1.0 - s_refl);
                if z_th.is_finite() && z_th > 1.0 {
                    nl_port_resistances[i] = z_th;
                    ports[i].resistance = z_th;
                    needs_recompute = true;
                }
            }
        }

        // Adapt the adapted (VS) port resistance.
        let s_adapted_refl = scattering[(n_total - 1) * n_total + (n_total - 1)];
        if s_adapted_refl.abs() > 0.05 && (1.0 - s_adapted_refl.abs()) > 1e-12 {
            let z_th = r_adapted * (1.0 + s_adapted_refl) / (1.0 - s_adapted_refl);
            if z_th.is_finite() && z_th > 0.0 {
                r_adapted = z_th;
                ports.last_mut().unwrap().resistance = r_adapted;
                needs_recompute = true;
            }
        }

        if needs_recompute {
            let (new_s, new_vcc) = recompute_scattering(&mna, &ports, vcc_vs_idx)?;
            scattering = new_s;
            if new_vcc.is_some() { vcc_injection_vec = new_vcc; }
        }
    }

    // ── Step 4b: Geometric mean scaling of NL port resistances ──────────
    // NL port resistances are free parameters in the R-type adaptor.
    // After adaptive Thevenin matching, normalize all NL ports to their
    // geometric mean. This balances impedance ratios across the NL-NL
    // sub-block of the scattering matrix, improving NR solver conditioning.
    // The adapted port keeps its Thévenin-matched value for VCC stages
    // to avoid impedance mismatch that attenuates the signal.
    if n_nl > 1 {
        let log_sum: f64 = nl_port_resistances.iter().map(|r| r.ln()).sum();
        let gmean = (log_sum / n_nl as f64).exp();
        #[cfg(feature = "debug-trace")]
        eprintln!(
            "[geo-scale] NL R before: {:?} → gmean={:.1}",
            &nl_port_resistances, gmean,
        );
        for i in 0..n_nl {
            nl_port_resistances[i] = gmean;
            ports[i].resistance = gmean;
        }
        // Only reset adapted port to gmean for non-VCC stages.
        // VCC stages keep the Thévenin-matched adapted port to preserve
        // signal coupling at the injection node.
        if vs_injection_vec.is_none() && vcc_vs_idx.is_none() {
            r_adapted = gmean;
            ports.last_mut().unwrap().resistance = gmean;
        }
        // Recompute scattering with balanced port resistances
        let (new_s, new_vcc) = recompute_scattering(&mna, &ports, vcc_vs_idx)?;
        scattering = new_s;
        if new_vcc.is_some() { vcc_injection_vec = new_vcc; }
    }

    // ── Step 5: Extract sub-blocks ──────────────────────────────────────
    let scattering_blocks =
        super::stage::MultiNlScattering::from_full_matrix(&scattering, n_nl, n_passive);

    // ── Step 5b: Extract DC bias from VCC injection vector ──────────────
    // dc_bias: NL ports only (for NR solver known_a)
    // vcc_bias_all: ALL ports (for scatter_all post-addition)
    let (dc_bias, vcc_bias_all) = if let Some(ref vcc_inj) = vcc_injection_vec {
        let mut bias: Vec<f64> = vcc_inj.iter().take(n_nl).map(|&k| k * supply_voltage).collect();
        bias.resize(n_nl, 0.0);
        let bias_all: Vec<f64> = vcc_inj.iter().map(|&k| k * supply_voltage).collect();
        #[cfg(feature = "debug-trace")]
        eprintln!(
            "[vcc-bias] supply={:.1}V dc_bias={:?} vcc_bias_all={:?}",
            supply_voltage, bias, bias_all
        );
        (bias, bias_all)
    } else {
        (vec![0.0; n_nl], Vec::new())
    };

    // ── Step 6: Create NL device roots ──────────────────────────────────
    let is_three_port_vari_mu = plan.nl_element_indices.len() == 1
        && n_nl == 2
        && matches!(
            &classified.nonlinear_elements[plan.nl_element_indices[0]].kind,
            NonlinearKind::Triode {
                is_vari_mu: true,
                ..
            }
        );

    let is_three_port_triode = plan.nl_element_indices.len() == 1
        && n_nl == 2
        && matches!(
            &classified.nonlinear_elements[plan.nl_element_indices[0]].kind,
            NonlinearKind::Triode {
                is_vari_mu: false,
                ..
            }
        );

    let (nl_devices, device_groups) = if is_three_port_vari_mu {
        let elem = &classified.nonlinear_elements[plan.nl_element_indices[0]];
        if let NonlinearKind::Triode {
            model_name,
            parallel_count,
            ..
        } = &elem.kind
        {
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
        if let NonlinearKind::Triode {
            model_name,
            parallel_count,
            ..
        } = &elem.kind
        {
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
    } else if plan.nl_element_indices.iter().all(|&idx| {
        matches!(
            &classified.nonlinear_elements[idx].kind,
            NonlinearKind::BjtNpn { .. } | NonlinearKind::BjtPnp { .. }
        )
    }) && !plan.nl_element_indices.is_empty() {
        // All-BJT group: each BJT is a 2-port device.
        let mut groups: Vec<NlDeviceGroupKind> = Vec::new();
        let mut offsets: Vec<usize> = Vec::new();
        let mut offset = 0usize;
        for &elem_idx in &plan.nl_element_indices {
            let elem = &classified.nonlinear_elements[elem_idx];
            offsets.push(offset);
            match &elem.kind {
                NonlinearKind::BjtNpn { model_name, .. } => {
                    let gp_model = gummel_poon_model(model_name);
                    groups.push(NlDeviceGroupKind::BjtTwoPort(BjtTwoPort::new(gp_model)));
                    offset += 2;
                }
                NonlinearKind::BjtPnp { model_name, .. } => {
                    let gp_model = gummel_poon_model(model_name);
                    groups.push(NlDeviceGroupKind::BjtTwoPort(BjtTwoPort::new_pnp(gp_model)));
                    offset += 2;
                }
                _ => unreachable!(),
            }
        }
        (Vec::new(), Some(MultiNlDeviceGroups { groups, offsets }))
    } else if plan.nl_element_indices.len() > 1 && has_mixed_device_types(plan, classified) {
        let mut groups: Vec<NlDeviceGroupKind> = Vec::new();
        let mut offsets: Vec<usize> = Vec::new();
        let mut offset = 0usize;

        for &elem_idx in &plan.nl_element_indices {
            let elem = &classified.nonlinear_elements[elem_idx];
            offsets.push(offset);

            match &elem.kind {
                NonlinearKind::Triode {
                    model_name,
                    parallel_count,
                    is_vari_mu: true,
                    grid_node: Some(_),
                    ..
                } => {
                    let model = vari_mu_model(model_name);
                    let tp = VariMuThreePort::new(model).with_parallel_count(*parallel_count);
                    groups.push(NlDeviceGroupKind::VariMuThreePort(tp));
                    offset += 2;
                }
                NonlinearKind::Triode {
                    model_name,
                    parallel_count,
                    is_vari_mu: false,
                    grid_node: Some(_),
                    ..
                } => {
                    let model = TriodeModel::by_name(model_name);
                    let tp = TriodeThreePort::new(model).with_parallel_count(*parallel_count);
                    groups.push(NlDeviceGroupKind::TriodeThreePort(tp));
                    offset += 2;
                }
                NonlinearKind::BjtNpn { model_name, .. } => {
                    let gp_model = gummel_poon_model(model_name);
                    groups.push(NlDeviceGroupKind::BjtTwoPort(BjtTwoPort::new(gp_model)));
                    offset += 2;
                }
                NonlinearKind::BjtPnp { model_name, .. } => {
                    let gp_model = gummel_poon_model(model_name);
                    groups.push(NlDeviceGroupKind::BjtTwoPort(BjtTwoPort::new_pnp(gp_model)));
                    offset += 2;
                }
                _ => {
                    let device = create_nl_device(&elem.kind)?;
                    groups.push(NlDeviceGroupKind::SinglePort(device));
                    offset += 1;
                }
            }
        }

        (Vec::new(), Some(MultiNlDeviceGroups { groups, offsets }))
    } else {
        let mut devices = Vec::with_capacity(n_nl);
        for &elem_idx in &plan.nl_element_indices {
            let elem = &classified.nonlinear_elements[elem_idx];
            let device = create_nl_device(&elem.kind)?;
            devices.push(device);
        }
        (devices, None)
    };

    // ── Step 7: Determine output port ───────────────────────────────────
    //
    // For BJT/triode stages: output from the collector/plate NL port of the
    // output device. Using a passive port (e.g., coupling cap) would give
    // V_across_cap instead of the actual collector signal.
    //
    // For passive-output stages (bridge rectifiers): BFS from output node
    // to find the nearest reactive port.
    let output_port = if is_three_port_vari_mu || is_three_port_triode {
        1 // plate-cathode port
    } else if let Some(ref dg) = device_groups {
        // For grouped devices (BJT 2-port, triode 3-port), output from the
        // collector/plate port of the output device.
        let output_group = plan
            .nl_element_indices
            .iter()
            .position(|&idx| idx == plan.output_element_idx)
            .unwrap_or(dg.groups.len().saturating_sub(1));
        if output_group < dg.groups.len() {
            let np = dg.groups[output_group].n_ports();
            // Collector/plate is the last port in each group (port 1 for 2-port BJT)
            dg.offsets[output_group] + np - 1
        } else {
            0
        }
    } else if n_nl > 0 {
        // Ungrouped NL devices: use the output element's position
        plan.nl_element_indices
            .iter()
            .position(|&idx| idx == plan.output_element_idx)
            .unwrap_or(0)
    } else if let Some(out_node) = plan.output_node {
        // No NL ports (linearized OTA, bridge rectifier): find passive port near output
        let direct_idx = reactive_edge_indices.iter().position(|&eidx| {
            let e = &graph.edges[eidx];
            e.node_a == out_node || e.node_b == out_node
        });
        if let Some(idx) = direct_idx {
            n_nl + n_subtrees + idx
        } else {
            // BFS through resistor edges to find reactive port.
            let mut visited: std::collections::HashSet<NodeId> = std::collections::HashSet::new();
            let mut queue = std::collections::VecDeque::new();
            visited.insert(out_node);
            queue.push_back(out_node);

            let reactive_nodes: std::collections::HashSet<NodeId> = reactive_edge_indices
                .iter()
                .flat_map(|&eidx| {
                    let e = &graph.edges[eidx];
                    [e.node_a, e.node_b]
                })
                .collect();

            let mut found_node = None;
            'bfs: while let Some(node) = queue.pop_front() {
                if reactive_nodes.contains(&node) && node != out_node {
                    found_node = Some(node);
                    break 'bfs;
                }
                for &eidx in &decomposed.residual_edges {
                    let e = &graph.edges[eidx];
                    let neighbor = if e.node_a == node {
                        Some(e.node_b)
                    } else if e.node_b == node {
                        Some(e.node_a)
                    } else {
                        None
                    };
                    if let Some(n) = neighbor {
                        if visited.insert(n) {
                            let comp = &graph.components[e.comp_idx];
                            if comp.kind.resistance().is_some() {
                                queue.push_back(n);
                            }
                        }
                    }
                }
            }

            if let Some(target_node) = found_node {
                reactive_edge_indices
                    .iter()
                    .position(|&eidx| {
                        let e = &graph.edges[eidx];
                        e.node_a == target_node || e.node_b == target_node
                    })
                    .map(|idx| n_nl + n_subtrees + idx)
                    .unwrap_or(0)
            } else {
                0
            }
        }
    } else {
        0
    };

    // ── Step 8: Create RTypeAdaptor and package ─────────────────────────
    let port_resistances: Vec<f64> = ports.iter().map(|p| p.resistance).collect();
    let adaptor = RTypeAdaptor::new(scattering, &port_resistances);

    // Node-voltage extraction for linearized OTA.
    let (extract_coeffs, extract_vs, extract_output_nodes) =
        if has_linearized_ota && n_nl == 0 && vs_injection_vec.is_some() {
            let vs_idx = num_vsources - 1;
            let ota_out_circuit = plan.ota_vccs[0].out_node;
            let ota_out_mna = node_to_mna(ota_out_circuit);
            let out_pos = ota_out_mna;
            let out_neg = None;
            let (coeffs, vs_coeff) =
                mna.derive_extraction_coeffs(&ports, vs_idx, out_pos, out_neg);
            (Some(coeffs), vs_coeff, Some((out_pos, out_neg)))
        } else {
            (None, 0.0, None)
        };

    let vs_source_index = if has_linearized_ota && n_nl == 0 {
        Some(num_vsources - 1)
    } else {
        None
    };
    // Always store recompute_data: needed for pot-driven scattering recompute
    // and for future sample-rate changes (oversampling adjustments).
    let recompute_data = Some(ScatteringRecomputeData {
        mna,
        port_node_pairs,
        adapted_resistance: r_adapted,
        vs_source_index,
        vcc_vs_index: vcc_vs_idx,
        extract_output_nodes,
    });

    // Use island-depth (boundary crossings from input) for reliable ordering
    // of boundary-split stages where hub shortcuts collapse dist_from_in.
    let signal_flow_distance = plan.signal_chain_depth.unwrap_or_else(|| {
        classified
            .dist_from_in
            .get(&plan.injection_node)
            .copied()
            .unwrap_or(usize::MAX)
    });

    let injection_node_id = plan.injection_node;
    let output_node_id = if let Some(out_node) = plan.output_node {
        out_node
    } else if output_port < n_nl {
        plan.nl_terminals[output_port].0
    } else {
        let passive_idx = output_port - n_nl;
        if passive_idx < n_subtrees {
            // Output from a subtree port — use its attachment node.
            decomposed.wdf_subtrees[passive_idx].attachment_node
        } else {
            let reactive_idx = passive_idx - n_subtrees;
            if reactive_idx < reactive_edge_indices.len() {
                let e = &graph.edges[reactive_edge_indices[reactive_idx]];
                if e.node_a != plan.injection_node {
                    e.node_a
                } else {
                    e.node_b
                }
            } else if !plan.nl_terminals.is_empty() {
                plan.nl_terminals.last().unwrap().0
            } else {
                plan.injection_node
            }
        }
    };

    // Build interpolation table for single-pot stages (no OTA, no state-space).
    let interp_table = if pot_mna_stamps.len() == 1
        && linearized_ota_data.is_none()
    {
        // Find the pot's max_resistance from the DynNode
        let max_r = match &pot_children[0] {
            DynNode::Pot { max_resistance, .. } => *max_resistance,
            _ => 0.0,
        };
        if max_r > 1.0 {
            if let Some(ref rd) = recompute_data {
                let (_, n1, n2, initial_g) = pot_mna_stamps[0];
                let vs_idx = rd.vs_source_index.or(rd.vcc_vs_index);
                Some(ScatteringInterpolationTable::build(
                    &rd.mna,
                    n1,
                    n2,
                    initial_g,
                    1.0,     // r_min: minimum pot resistance (clamped at 1Ω)
                    max_r,   // r_max: full pot resistance
                    &ports,
                    vs_idx,
                    32,      // 32 log-spaced entries
                ))
            } else {
                None
            }
        } else {
            None
        }
    } else {
        None
    };

    let mut initial_v = compute_initial_v_prev(n_nl, &device_groups, supply_voltage);

    // Pre-converge DC operating point using homotopy continuation.
    // Gradually ramp dc_bias from 0 → full and solve NR at each step,
    // using the previous solution as warm-start. This finds the true DC
    // operating point for initial_v_prev and avoids NR divergence at runtime.
    if n_nl > 0 && !dc_bias.iter().all(|&b| b == 0.0) {
        let n_steps = 20;
        let s_nl = &scattering_blocks.s_nl;
        for step in 1..=n_steps {
            let lambda = step as f64 / n_steps as f64;
            let known_a_dc: Vec<f64> = dc_bias.iter().map(|&b| b * lambda).collect();
            if let Some(ref dg) = device_groups {
                let groups: Vec<&dyn NlDeviceGroupIv> =
                    dg.groups.iter().map(|g| g.as_group_iv()).collect();
                let _ = multi_port_nr_solve_grouped(
                    n_nl, s_nl, &known_a_dc, &nl_port_resistances,
                    &groups, &dg.offsets, &mut initial_v, 50, 1e-6,
                );
            } else {
                let devices: Vec<&dyn NlDeviceIv> =
                    nl_devices.iter().map(|d| d.as_nl_device_iv()).collect();
                let _ = multi_port_nr_solve(
                    n_nl, s_nl, &known_a_dc, &nl_port_resistances,
                    &devices, &mut initial_v, 50, 1e-6,
                );
            }
        }
        #[cfg(feature = "debug-trace")]
        eprintln!("[vcc-preconverge] DC operating point: {:?}", initial_v);
    }

    Some(MultiNlStage {
        adaptor,
        nl_devices,
        nl_port_resistances,
        passive_children,
        pot_children,
        pot_mna_stamps,
        n_nl,
        v_prev: initial_v.clone(),
        scattering: scattering_blocks,
        oversampler: Oversampler::new(oversampling),
        compensation: plan.compensation,
        output_port,
        device_groups,
        recompute_data,
        signal_flow_distance,
        transformer_gain: 1.0,
        injection_node_id,
        output_node_id,
        recompute_pending: false,
        veb_bias_offset: 0.0,
        feedback_scale: 0.1,
        linearized_ota: linearized_ota_data,
        vs_injection: vs_injection_vec,
        extract_coeffs,
        extract_vs,
        state_space: None,
        bias_pot_id: None,
        bias_emitter_r: 470.0,
        interp_table,
        dc_bias,
        vcc_bias_all,
        vcc_vs_index: vcc_vs_idx,
        supply_voltage,
        dc_ramp: 0,
        initial_v_prev: initial_v,
        dc_blocker_x1: 0.0,
        dc_blocker_y1: 0.0,
    })
}


/// Compute physics-based initial voltage guesses for NR solver warm-start.
///
/// Using device-type-aware initial voltages (Vbe ≈ -0.15V for Ge PNP,
/// Vce ≈ -supply/2) avoids exponential blow-up from naive dc_bias/2 values.
fn compute_initial_v_prev(
    n_nl: usize,
    device_groups: &Option<MultiNlDeviceGroups>,
    supply_voltage: f64,
) -> Vec<f64> {
    let mut v_prev = vec![0.0; n_nl];
    if let Some(ref dg) = device_groups {
        for (g, group) in dg.groups.iter().enumerate() {
            let offset = dg.offsets[g];
            match group {
                NlDeviceGroupKind::BjtTwoPort(bjt) => {
                    let sign = if bjt.is_pnp { -1.0 } else { 1.0 };
                    // Port 0: base-emitter — near forward-bias threshold
                    if offset < n_nl {
                        v_prev[offset] = sign * 0.15; // Ge: 0.15V, Si: ~0.6V
                    }
                    // Port 1: collector-emitter — mid-supply for active region
                    // PNP: Vce_port < 0 (collector below emitter), NPN: > 0
                    if offset + 1 < n_nl {
                        v_prev[offset + 1] = sign * supply_voltage * 0.5;
                    }
                }
                NlDeviceGroupKind::TriodeThreePort(_) | NlDeviceGroupKind::VariMuThreePort(_) => {
                    // Port 0: grid-cathode — slight negative bias
                    if offset < n_nl {
                        v_prev[offset] = -2.0;
                    }
                    // Port 1: plate-cathode — mid-supply
                    if offset + 1 < n_nl {
                        v_prev[offset + 1] = supply_voltage * 0.5;
                    }
                }
                NlDeviceGroupKind::SinglePort(_) => {
                    // Single-port device (diode, etc.) — start near zero
                    if offset < n_nl {
                        v_prev[offset] = 0.0;
                    }
                }
            }
        }
    }
    v_prev
}

/// New pipeline: decompose passive network then build R-type stage.
///
/// Uses `sp_decompose` to greedily extract SP-reducible pendant subtrees,
/// then `build_rtype_stage` to build the MNA from the residual.
fn try_build_multi_nl_stage(
    plan: &MultiNlPlan,
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    sample_rate: f64,
    oversampling: OversamplingFactor,
    supply_voltage: f64,
) -> Option<MultiNlStage> {
    let n_nl = plan.nl_terminals.len();
    let has_linearized_ota = !plan.ota_vccs.is_empty();
    if (n_nl == 0 && !has_linearized_ota) || plan.passive_edge_indices.is_empty() {
        return None;
    }

    // Collect junction nodes: NL terminal nodes + VS injection node.
    let mut junction_nodes: Vec<NodeId> = Vec::new();
    for &(pos, neg) in &plan.nl_terminals {
        if !junction_nodes.contains(&pos) {
            junction_nodes.push(pos);
        }
        if !junction_nodes.contains(&neg) {
            junction_nodes.push(neg);
        }
    }
    if !junction_nodes.contains(&plan.injection_node) {
        junction_nodes.push(plan.injection_node);
    }
    // Add output node as junction so output tail edges stay in MNA residual
    // (not SP-reduced into WDF subtrees, which would hide pots from pot_children).
    if let Some(out_node) = plan.output_node {
        if !junction_nodes.contains(&out_node) {
            junction_nodes.push(out_node);
        }
    }
    // Add nodes adjacent to pot edges as junctions so pots stay in the MNA
    // residual (stamped into G matrix → pot_children) rather than being
    // SP-reduced into WDF subtrees where they'd be invisible to pot binding.
    for &eidx in &plan.passive_edge_indices {
        let e = &graph.edges[eidx];
        if graph.components[e.comp_idx].kind.is_pot() {
            if !junction_nodes.contains(&e.node_a) {
                junction_nodes.push(e.node_a);
            }
            if !junction_nodes.contains(&e.node_b) {
                junction_nodes.push(e.node_b);
            }
        }
    }

    // Also add OTA nodes as junctions.
    for ota in &plan.ota_vccs {
        for &n in &[ota.in_pos, ota.in_neg, ota.out_node] {
            if !junction_nodes.contains(&n) {
                junction_nodes.push(n);
            }
        }
    }

    // State-space path (linearized OTA, n_nl=0) requires ALL components in the
    // MNA — caps in cap_stamps, resistors/pots in G matrix. WDF subtrees bypass
    // MNA entirely, so skip subtree extraction for state-space mode.
    let use_state_space = has_linearized_ota && n_nl == 0;
    let decomposed = if use_state_space {
        // No subtree extraction — all edges go to residual.
        super::graph::DecomposedCircuit {
            wdf_subtrees: Vec::new(),
            residual_edges: plan.passive_edge_indices.clone(),
        }
    } else {
        sp_decompose(&plan.passive_edge_indices, &junction_nodes, graph, sample_rate * oversampling.ratio() as f64)
    };

    {
        let subtree_info: Vec<String> = decomposed
            .wdf_subtrees
            .iter()
            .map(|st| format!("attach={} far={}", st.attachment_node, st.far_node))
            .collect();
        let residual_names: Vec<String> = decomposed
            .residual_edges
            .iter()
            .map(|&eidx| {
                let comp = &graph.components[graph.edges[eidx].comp_idx];
                format!("{}({}→{})", comp.id, graph.edges[eidx].node_a, graph.edges[eidx].node_b)
            })
            .collect();
        let elem_names: Vec<String> = plan
            .nl_element_indices
            .iter()
            .map(|&idx| {
                let elem = &classified.nonlinear_elements[idx];
                graph.components[graph.edges[elem.edge_idx].comp_idx].id.clone()
            })
            .collect();
        eprintln!(
            "[sp-decompose] elems={:?} subtrees={:?} residual={:?}",
            elem_names, subtree_info, residual_names,
        );
    }

    build_rtype_stage(&decomposed, plan, classified, graph, sample_rate, oversampling, supply_voltage)
}

/// Check if a multi-element plan has mixed device types that require
/// the grouped solver (some elements have cross-coupled ports, others don't).
fn has_mixed_device_types(plan: &MultiNlPlan, classified: &ClassifiedCircuit) -> bool {
    let mut has_multi_port = false;
    let mut has_single_port = false;
    for &elem_idx in &plan.nl_element_indices {
        let elem = &classified.nonlinear_elements[elem_idx];
        match &elem.kind {
            NonlinearKind::Triode {
                grid_node: Some(_), ..
            } => {
                has_multi_port = true;
            }
            NonlinearKind::BjtNpn { .. } | NonlinearKind::BjtPnp { .. } => {
                has_multi_port = true;
            }
            _ => {
                has_single_port = true;
            }
        }
    }
    has_multi_port && has_single_port
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
        NonlinearKind::Triode {
            model_name,
            parallel_count,
            is_vari_mu,
            ..
        } => {
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
        NonlinearKind::DiodePair(dt) => {
            let model = diode_model(*dt);
            Some(NlDeviceKind::DiodePair(DiodePairRoot::new(model)))
        }
        NonlinearKind::Pentode { model_name } => {
            let model = pentode_model(model_name);
            Some(NlDeviceKind::Pentode(PentodeRoot::new(model)))
        }
        _ => None, // Jfet, Mosfet, Zener, Ota not yet supported in multi-NL
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Internal builders
// ═══════════════════════════════════════════════════════════════════════════

/// Build a standard VS-driven stage (simple, virtual edge, or push-pull half).
fn build_vs_stage(
    plan: &StagePlan,
    elem: &super::classify::NonlinearElement,
    graph: &CircuitGraph,
    sample_rate: f64,
    oversampling: OversamplingFactor,
    use_jfet_vr: bool,
) -> Option<WdfStage> {
    // Use supply remap so triodes with plate loads to named supply rails
    // (vcc_sc, A_bal, etc.) can reduce to valid SP trees.
    let mut extra = vec![ExtraEdge {
        node_a: plan.source_node,
        node_b: plan.injection_node,
        tree: DynNode::VoltageSource { voltage: 0.0, rp: 1.0 },
    }];
    if let Some(ve) = &plan.virtual_edge {
        extra.push(ExtraEdge {
            node_a: ve.node_a,
            node_b: ve.node_b,
            tree: DynNode::Resistor { rp: ve.resistance, last_a: 0.0 },
        });
    }
    let remap = |n: NodeId| -> NodeId {
        if graph.supply_nodes.contains(&n) { graph.gnd_node } else { n }
    };

    let (tree, output_probe) = graph_reduce(
        &plan.passive_idxs, &extra, &plan.terminals,
        graph, sample_rate, &HashMap::new(), remap,
        Some(graph.out_node),
    ).ok()?;
    let (root, base_diode_model) = create_root(&elem.kind, use_jfet_vr);

    Some(WdfStage {
        tree,
        root,
        compensation: plan.compensation,
        oversampler: Oversampler::new(oversampling),
        base_diode_model,
        paired_opamp: None,
        allpass_feedback: None,
        dc_block: plan.dc_block,
        is_source_follower: false,
        prev_source_voltage: 0.0,
        signal_flow_distance: 0, // set by caller
        transformer_gain: 1.0,   // set by caller
        injection_node_id: plan.injection_node,
        output_node_id: elem
            .junction_nodes
            .first()
            .copied()
            .unwrap_or(plan.injection_node),
        is_trigger_voice: false,
        is_feedforward: false,
        sample_counter: 0,
        root_comp_id: String::new(),
        feedback_pot_id: None,
        output_probe,
    })
}

/// Build a source follower stage (no voltage source in tree).
///
/// When `use_jfet_vr` is true, the JFET is a passive variable resistor
/// that needs signal injected via a voltage source.  We wrap the passive
/// tree in Series(VS, tree) so `set_voltage` + `reflected` work correctly.
fn build_source_follower_stage(
    plan: &StagePlan,
    elem: &super::classify::NonlinearElement,
    graph: &CircuitGraph,
    sample_rate: f64,
    oversampling: OversamplingFactor,
    use_jfet_vr: bool,
) -> Option<WdfStage> {
    let passive_tree = graph_reduce(
        &plan.passive_idxs, &[], &plan.terminals,
        graph, sample_rate, &HashMap::new(), |n| n,
        None,
    ).ok()?.0;
    let (root, _) = create_root(&elem.kind, use_jfet_vr);

    // JfetVr is a passive element — signal must enter via a voltage source.
    // Wrap the passive tree in Series(VS, passive) to inject signal.
    let (tree, is_sf) = if use_jfet_vr {
        let vs = DynNode::VoltageSource {
            voltage: 0.0,
            rp: 1.0,
        };
        let r_passive = passive_tree.port_resistance();
        let r_vs = 1.0;
        let rp = r_vs + r_passive;
        let tree = DynNode::Series {
            left: Box::new(vs),
            right: Box::new(passive_tree),
            rp,
            gamma: r_vs / rp,
            b1: 0.0,
            b2: 0.0,
        };
        (tree, false) // Not a source follower — VS drives signal
    } else {
        (passive_tree, true)
    };

    Some(WdfStage {
        tree,
        root,
        compensation: 1.0,
        oversampler: Oversampler::new(oversampling),
        base_diode_model: None,
        paired_opamp: None,
        allpass_feedback: None,
        dc_block: None,
        is_source_follower: is_sf,
        prev_source_voltage: 0.0,
        signal_flow_distance: 0, // set by caller
        transformer_gain: 1.0,   // set by caller
        injection_node_id: plan.injection_node,
        output_node_id: elem
            .junction_nodes
            .first()
            .copied()
            .unwrap_or(plan.injection_node),
        is_trigger_voice: false,
        is_feedforward: false,
        sample_counter: 0,
        root_comp_id: String::new(),
        feedback_pot_id: None,
        output_probe: None,
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
) -> Option<(DynNode, f64)> {
    // Identify which passive edges terminate at supply nodes.
    // Map: comp_idx → CathodeBiasSource DynNode for leaf overrides.
    let mut leaf_overrides: HashMap<usize, DynNode> = HashMap::new();
    for &eidx in &plan.passive_idxs {
        let e = &graph.edges[eidx];
        for &node in &[e.node_a, e.node_b] {
            if let Some(&voltage) = graph.supply_voltages.get(&node) {
                leaf_overrides.insert(e.comp_idx, DynNode::CathodeBiasSource { voltage, rp: 1.0 });
            }
        }
    }

    let mut extra = vec![ExtraEdge {
        node_a: plan.source_node,
        node_b: plan.injection_node,
        tree: DynNode::VoltageSource { voltage: 0.0, rp: 1.0 },
    }];
    if let Some(ve) = &plan.virtual_edge {
        extra.push(ExtraEdge {
            node_a: ve.node_a,
            node_b: ve.node_b,
            tree: DynNode::Resistor { rp: ve.resistance, last_a: 0.0 },
        });
    }
    let tree = graph_reduce(
        &plan.passive_idxs, &extra, &plan.terminals,
        graph, sample_rate, &leaf_overrides, |n| n,
        None,
    ).ok()?.0;
    Some((tree, plan.compensation))
}

/// Build push-pull tube roots from classified elements.
fn build_push_pull_roots(
    push_elem: &super::classify::NonlinearElement,
    pull_elem: &super::classify::NonlinearElement,
) -> (TubeRoot, TubeRoot) {
    let build_root = |elem: &super::classify::NonlinearElement| -> TubeRoot {
        if let NonlinearKind::Triode {
            model_name,
            parallel_count,
            is_vari_mu,
            ..
        } = &elem.kind
        {
            if *is_vari_mu {
                let model = vari_mu_model(model_name);
                TubeRoot::VariMu(VariMuTriodeRoot::new(model).with_parallel_count(*parallel_count))
            } else {
                let model = triode_model(model_name);
                TubeRoot::Koren(TriodeRoot::new(model).with_parallel_count(*parallel_count))
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
fn find_secondary_load_resistance(graph: &CircuitGraph, xfmr_comp_idx: usize) -> f64 {
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
                if let Some(r) = graph.components[edge.comp_idx].kind.resistance() {
                    return r;
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
    let n_eff = if is_ct {
        turns_ratio / 2.0
    } else {
        turns_ratio
    };

    // Secondary: just the load resistor (referred to secondary side).
    // The magnetizing inductance LF rolloff is handled as a post-process
    // high-pass filter rather than an in-tree WDF inductor, avoiding
    // the DC transient/initialization issue that causes tube cutoff.
    let secondary = DynNode::Resistor { rp: r_load, last_a: 0.0 };

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
        let dcr = if is_ct {
            primary_dcr / 2.0
        } else {
            primary_dcr
        };
        let combined_rp = dcr + xfmr_rp;
        xfmr = DynNode::Series {
            left: Box::new(DynNode::Resistor { rp: dcr, last_a: 0.0 }),
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
///
/// `use_jfet_vr` overrides JFET creation: when true, builds a
/// `JfetVr` (variable resistance, no NR) instead of `Jfet` (full NR solver).
fn create_root(kind: &NonlinearKind, use_jfet_vr: bool) -> (RootKind, Option<DiodeModel>) {
    match kind {
        NonlinearKind::DiodePair(dt) => {
            let model = diode_model(*dt);
            (RootKind::DiodePair(DiodePairRoot::new(model)), Some(model))
        }
        NonlinearKind::SingleDiode(dt) => {
            let model = diode_model(*dt);
            (RootKind::SingleDiode(DiodeRoot::new(model)), Some(model))
        }
        NonlinearKind::Jfet {
            model_name,
            is_n_channel,
        } => {
            let model = jfet_model(model_name, *is_n_channel);
            if use_jfet_vr {
                (
                    RootKind::JfetVr(JfetVariableResistor::new(model)),
                    None,
                )
            } else {
                (RootKind::Jfet(JfetRoot::new(model)), None)
            }
        }
        NonlinearKind::BjtNpn { model_name, .. } => {
            let model = BjtModel::by_name(model_name);
            (RootKind::BjtNpn(BjtNpnRoot::new(model)), None)
        }
        NonlinearKind::BjtPnp { model_name, .. } => {
            let model = BjtModel::by_name(model_name);
            (RootKind::BjtPnp(BjtPnpRoot::new(model)), None)
        }
        NonlinearKind::Triode {
            model_name,
            parallel_count,
            is_vari_mu,
            ..
        } => {
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
                    RootKind::Triode(TriodeRoot::new(model).with_parallel_count(*parallel_count)),
                    None,
                )
            }
        }
        NonlinearKind::Pentode { model_name } => {
            let model = pentode_model(model_name);
            (RootKind::Pentode(PentodeRoot::new(model)), None)
        }
        NonlinearKind::Mosfet {
            mosfet_type,
            is_n_channel,
        } => {
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
