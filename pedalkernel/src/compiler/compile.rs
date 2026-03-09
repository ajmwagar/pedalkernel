//! Main compiler entry point: PedalDef -> CompiledPedal.
//!
//! Orchestrates the 6-pass compilation pipeline:
//! - Pass 0: Graph construction (CircuitGraph::from_pedal)
//! - Pass 1: Element classification (classify.rs)
//! - Pass 2: Op-amp analysis (opamp_analysis.rs)
//! - Pass 3: Stage planning (plan.rs)
//! - Pass 4: Tree building (build.rs)
//! - Pass 5: Binding & assembly (bind.rs)

use std::collections::{HashMap, HashSet};

use crate::dsl::*;
use crate::PedalProcessor;
use crate::elements::{BbdDelayLine, BbdModel, SlewRateLimiter, TriodeModel};
use crate::oversampling::{Oversampler, OversamplingFactor};
use crate::thermal::ThermalModel;
use crate::tolerance::ToleranceEngine;

use super::compiled::*;
use super::component::Component;
use super::components::{
    Bbd as BbdComp, Capacitor as CapacitorComp, DelayLineComp, Inductor as InductorComp,
    Lfo as LfoComp, Pentode as PentodeComp, Potentiometer as PotComp,
    Resistor as ResistorComp, Tap as TapComp, Triode as TriodeComp, Vca as VcaComp,
    VariMu as VariMuComp, Vco as VcoComp,
};
use super::dyn_node::DynNode;
use super::graph::*;
use super::helpers::*;
use super::stage::{RootKind, WdfStage};
use crate::tree::{MnaSystem, ScatteringInterpolationTable, WdfPort};

// ═══════════════════════════════════════════════════════════════════════════
// Modulation-controlled element detection
// ═══════════════════════════════════════════════════════════════════════════

/// Scan PedalDef nets for OTAs whose `.iabc` pin is driven by an envelope
/// follower or LFO. These OTAs should be linearized (gm stamped into MNA)
/// rather than solved with Newton-Raphson at a WDF tree root.
fn detect_envelope_controlled_otas(pedal: &PedalDef) -> HashSet<String> {
    let mut result = HashSet::new();

    let modulator_ids: HashSet<&str> = pedal
        .components
        .iter()
        .filter(|c| c.kind.is_modulation_source())
        .map(|c| c.id.as_str())
        .collect();

    if modulator_ids.is_empty() {
        return result;
    }

    let ota_ids: HashSet<&str> = pedal
        .components
        .iter()
        .filter(|c| c.kind.op_amp_type().map_or(false, |ot| ot.is_ota()))
        .map(|c| c.id.as_str())
        .collect();

    for net in &pedal.nets {
        let from_is_modulator = match &net.from {
            Pin::ComponentPin { component, pin } => {
                modulator_ids.contains(component.as_str()) && pin == "out"
            }
            _ => false,
        };
        if !from_is_modulator {
            continue;
        }
        for to_pin in &net.to {
            if let Pin::ComponentPin { component, pin } = to_pin {
                if ota_ids.contains(component.as_str()) && pin == "iabc" {
                    result.insert(component.clone());
                }
            }
        }
    }

    result
}

/// Scan PedalDef nets for JFETs that have LFO or envelope follower connections
/// to their `.vgs` pin. These JFETs should use the variable resistance model
/// (simple Rds formula) instead of the full Newton-Raphson JFET solver.
fn detect_lfo_controlled_jfets(pedal: &PedalDef) -> HashSet<String> {
    let mut result = HashSet::new();

    // Collect LFO and envelope follower component IDs.
    let modulator_ids: HashSet<&str> = pedal
        .components
        .iter()
        .filter(|c| c.kind.is_modulation_source())
        .map(|c| c.id.as_str())
        .collect();

    if modulator_ids.is_empty() {
        return result;
    }

    // Collect JFET component IDs for cross-reference.
    let jfet_ids: HashSet<&str> = pedal
        .components
        .iter()
        .filter(|c| c.kind.is_jfet())
        .map(|c| c.id.as_str())
        .collect();

    // Scan nets for modulator.out -> jfet.vgs connections.
    for net in &pedal.nets {
        let from_is_modulator = match &net.from {
            Pin::ComponentPin { component, pin } => {
                modulator_ids.contains(component.as_str()) && pin == "out"
            }
            _ => false,
        };
        if !from_is_modulator {
            continue;
        }
        for to_pin in &net.to {
            if let Pin::ComponentPin { component, pin } = to_pin {
                if jfet_ids.contains(component.as_str()) && pin == "vgs" {
                    result.insert(component.clone());
                }
            }
        }
    }

    result
}

// ═══════════════════════════════════════════════════════════════════════════
// Passive circuit gain helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Compute the voltage divider gain for a resistor-only circuit.
fn compute_resistor_divider_gain(graph: &CircuitGraph) -> f64 {
    let r_series = find_resistance_between(graph, graph.in_node, graph.out_node);
    let r_shunt = find_resistance_between(graph, graph.out_node, graph.gnd_node);

    match (r_series, r_shunt) {
        (Some(rs), Some(rsh)) => rsh / (rs + rsh),
        _ => 1.0,
    }
}

/// Find total resistance between two nodes via BFS through resistive elements.
fn find_resistance_between(graph: &CircuitGraph, from: NodeId, to: NodeId) -> Option<f64> {
    if from == to {
        return Some(0.0);
    }
    let mut visited = std::collections::HashSet::new();
    let mut queue = std::collections::VecDeque::new();
    visited.insert(from);
    queue.push_back((from, 0.0));

    while let Some((node, r_so_far)) = queue.pop_front() {
        for (_, e) in graph.edges.iter().enumerate() {
            let (other, matches) = if e.node_a == node {
                (e.node_b, true)
            } else if e.node_b == node {
                (e.node_a, true)
            } else {
                (0, false)
            };
            if !matches {
                continue;
            }

            if let Some(r) = graph.components[e.comp_idx].kind.resistance() {
                if other == to {
                    return Some(r_so_far + r);
                }
                if visited.insert(other) {
                    queue.push_back((other, r_so_far + r));
                }
            }
        }
    }
    None
}

// ═══════════════════════════════════════════════════════════════════════════
// Passive WDF stage builder
// ═══════════════════════════════════════════════════════════════════════════

/// Build a WDF stage for a passive-only circuit.
///
/// Uses output-rooted tree decomposition: the load element (connected from
/// out_node to gnd) is placed at the root, and the remaining elements + VS
/// form the tree body. This gives correct filter behavior via the standard
/// WDF output extraction: V_out = (a_root + b_tree) / 2.
///
/// For simple topologies (1R + 1C, or 1R + 1L):
/// - RC lowpass (R in→out, C out→gnd): tree=Series(VS,R), root=CapacitorRoot
/// - RC highpass (C in→out, R out→gnd): tree=Series(VS,C), root=ResistiveTermination
/// - RL lowpass (L in→out, R out→gnd): tree=Series(VS,L), root=ResistiveTermination
/// - RL highpass (R in→out, L out→gnd): tree=Series(VS,R), root=InductorRoot
///
/// For complex circuits: falls back to ShortCircuit with all elements in tree.
fn build_passive_wdf_stage(
    graph: &CircuitGraph,
    sample_rate: f64,
    oversampling: OversamplingFactor,
) -> Option<WdfStage> {
    let vs_comp_idx = graph.components.len();
    let passive_edges: Vec<usize> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(_, e)| {
            graph.components[e.comp_idx].kind.is_simple_passive()
        })
        .map(|(i, _)| i)
        .collect();

    if passive_edges.is_empty() {
        return None;
    }

    // Try output-rooted decomposition for simple 2-element circuits.
    if let Some(stage) = build_output_rooted_stage(
        graph,
        &passive_edges,
        sample_rate,
        oversampling,
    ) {
        return Some(stage);
    }

    // For 3+ element passive circuits, prefer MNA R-type adaptor.
    // The ShortCircuit root's output extraction is unreliable for complex
    // multi-element trees where the VS position is non-deterministic.
    // MNA derives the exact scattering matrix from circuit equations.
    if passive_edges.len() >= 3 {
        if let Some(stage) =
            build_passive_mna_stage(graph, &passive_edges, sample_rate, oversampling)
        {
            return Some(stage);
        }
    }

    // Fallback: full tree with ShortCircuit root (all elements in tree, gnd at root).
    let source_node = graph.edges.len() + 1000;
    let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();
    sp_edges.push((source_node, graph.in_node, SpTree::Leaf(vs_comp_idx)));
    for &eidx in &passive_edges {
        let e = &graph.edges[eidx];
        sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
    }
    let terminals = vec![source_node, graph.gnd_node];
    match sp_reduce(sp_edges, &terminals) {
        Ok(sp_tree) => {
            let mut all_components = graph.components.clone();
            while all_components.len() <= vs_comp_idx {
                all_components.push(ComponentDef {
                    id: "__vs__".to_string(),
                    kind: Box::new(ResistorComp { value: 1.0 }),
                });
            }
            let tree = sp_to_dyn_with_vs(
                &sp_tree,
                &all_components,
                &graph.fork_paths,
                sample_rate,
                vs_comp_idx,
            );
            let mut stage = WdfStage {
                tree,
                root: RootKind::ShortCircuit,
                compensation: 1.0,
                oversampler: Oversampler::new(oversampling),
                base_diode_model: None,
                paired_opamp: None,
                allpass_feedback: None,
                dc_block: None,
                is_source_follower: false,
                prev_source_voltage: 0.0,
                signal_flow_distance: 0,
                transformer_gain: 1.0,
                injection_node_id: usize::MAX,
                output_node_id: usize::MAX,
                is_trigger_voice: false,
                sample_counter: 0,
                root_comp_id: String::new(),
                feedback_pot_id: None,
            };
            stage.balance_vs_impedance();
            Some(stage)
        }
        Err(_) => {
            // SP reduction failed (non-series-parallel topology).
            // Fall back to MNA-derived R-type adaptor.
            build_passive_mna_stage(graph, &passive_edges, sample_rate, oversampling)
        }
    }
}


/// Build a WDF stage using the unified sp_decompose + MNA pipeline for
/// non-series-parallel passive topologies.
///
/// Uses `sp_decompose` to extract SP-reducible pendant subtrees, then stamps
/// only the residual (bridging) edges into MNA. Each subtree becomes a single
/// WDF port with proper impedance, reducing the MNA size.
fn build_passive_mna_stage(
    graph: &CircuitGraph,
    passive_edges: &[usize],
    sample_rate: f64,
    _oversampling: OversamplingFactor,
) -> Option<WdfStage> {
    // Verify out_node is reachable from in_node through passive edges.
    {
        let mut adj: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
        for &eidx in passive_edges {
            let e = &graph.edges[eidx];
            adj.entry(e.node_a).or_default().push(e.node_b);
            adj.entry(e.node_b).or_default().push(e.node_a);
        }
        let mut visited = HashSet::new();
        let mut queue = std::collections::VecDeque::new();
        visited.insert(graph.in_node);
        queue.push_back(graph.in_node);
        while let Some(node) = queue.pop_front() {
            if node == graph.out_node {
                break;
            }
            if let Some(neighbors) = adj.get(&node) {
                for &n in neighbors {
                    if visited.insert(n) {
                        queue.push_back(n);
                    }
                }
            }
        }
        if !visited.contains(&graph.out_node) {
            return None;
        }
    }

    // Junction nodes for passive circuit: in_node and out_node.
    let junction_nodes = vec![graph.in_node, graph.out_node];
    let decomposed = super::graph::sp_decompose(passive_edges, &junction_nodes, graph);

    build_passive_rtype_from_decomposed(
        graph,
        &decomposed,
        passive_edges,
        graph.in_node,
        graph.out_node,
        1e9, // 1 GΩ probe
        sample_rate,
    )
}

/// Build a PassiveRType MNA stage for orphan output networks, using the unified pipeline.
fn build_orphan_output_mna_stage(
    graph: &CircuitGraph,
    passive_edges: &[usize],
    vs_node: NodeId,
    probe_node: NodeId,
    probe_resistance: f64,
    sample_rate: f64,
) -> Option<WdfStage> {
    // Verify probe_node is reachable from vs_node.
    {
        let mut adj: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
        for &eidx in passive_edges {
            let e = &graph.edges[eidx];
            adj.entry(e.node_a).or_default().push(e.node_b);
            adj.entry(e.node_b).or_default().push(e.node_a);
        }
        let mut visited = HashSet::new();
        let mut queue = std::collections::VecDeque::new();
        visited.insert(vs_node);
        queue.push_back(vs_node);
        while let Some(node) = queue.pop_front() {
            if node == probe_node {
                break;
            }
            if let Some(neighbors) = adj.get(&node) {
                for &n in neighbors {
                    if visited.insert(n) {
                        queue.push_back(n);
                    }
                }
            }
        }
        if !visited.contains(&probe_node) {
            return None;
        }
    }

    let junction_nodes = vec![vs_node, probe_node];
    let decomposed = super::graph::sp_decompose(passive_edges, &junction_nodes, graph);

    build_passive_rtype_from_decomposed(
        graph,
        &decomposed,
        passive_edges,
        vs_node,
        probe_node,
        probe_resistance,
        sample_rate,
    )
}

/// Build a PassiveRType WdfStage from a decomposed circuit.
///
/// Shared implementation for `build_passive_mna_stage` and
/// `build_orphan_output_mna_stage` in the unified pipeline.
fn build_passive_rtype_from_decomposed(
    graph: &CircuitGraph,
    decomposed: &super::graph::DecomposedCircuit,
    _passive_edges: &[usize],
    vs_node: NodeId,
    probe_node: NodeId,
    probe_resistance: f64,
    sample_rate: f64,
) -> Option<WdfStage> {
    // ── Step 1: Collect MNA nodes from residual edges + subtree attachments ──
    let mut node_set = std::collections::BTreeSet::new();
    for &eidx in &decomposed.residual_edges {
        let e = &graph.edges[eidx];
        if e.node_a != graph.gnd_node {
            node_set.insert(e.node_a);
        }
        if e.node_b != graph.gnd_node {
            node_set.insert(e.node_b);
        }
    }
    for subtree in &decomposed.wdf_subtrees {
        if subtree.attachment_node != graph.gnd_node {
            node_set.insert(subtree.attachment_node);
        }
    }
    if vs_node != graph.gnd_node {
        node_set.insert(vs_node);
    }
    if probe_node != graph.gnd_node {
        node_set.insert(probe_node);
    }

    let nodes: Vec<NodeId> = node_set.into_iter().collect();
    let num_mna_nodes = nodes.len();
    if num_mna_nodes == 0 {
        return None;
    }
    let node_to_idx: HashMap<NodeId, usize> =
        nodes.iter().enumerate().map(|(i, &n)| (n, i)).collect();

    let to_mna = |node: NodeId| -> Option<usize> {
        if node == graph.gnd_node {
            None
        } else {
            node_to_idx.get(&node).copied()
        }
    };

    // ── Step 2: Stamp residual edges into MNA ──────────────────────────
    let effective_rate = sample_rate;
    let mut mna = crate::tree::MnaSystem::new(num_mna_nodes, 0);
    let mut reactive_children: Vec<DynNode> = Vec::new();
    let mut reactive_ports: Vec<crate::tree::WdfPort> = Vec::new();
    let mut pot_children_pending: Vec<DynNode> = Vec::new();
    let mut pot_stamp_nodes: Vec<(Option<usize>, Option<usize>, f64)> = Vec::new();

    for &eidx in &decomposed.residual_edges {
        let e = &graph.edges[eidx];
        let n1 = to_mna(e.node_a);
        let n2 = to_mna(e.node_b);
        let comp = &graph.components[e.comp_idx];

        use super::component::StampResult;
        match comp.kind.stamp_mna(&comp.id, n1, n2, &mut mna, effective_rate) {
            StampResult::Stamped => {}
            StampResult::Reactive { dyn_node, rp } => {
                reactive_children.push(dyn_node);
                reactive_ports.push(crate::tree::WdfPort {
                    node_pos: n1,
                    node_neg: n2,
                    resistance: rp,
                });
            }
            StampResult::Pot {
                dyn_node,
                initial_conductance,
            } => {
                pot_children_pending.push(dyn_node);
                pot_stamp_nodes.push((n1, n2, initial_conductance));
            }
            StampResult::Skip => {}
        }
    }

    // ── Step 2b: Add WDF subtree ports ─────────────────────────────────
    for subtree in &decomposed.wdf_subtrees {
        let dyn_tree = super::graph::sp_to_dyn(
            &subtree.tree,
            &graph.components,
            &graph.fork_paths,
            sample_rate,
        );
        let rp = dyn_tree.port_resistance();
        let pos = to_mna(subtree.attachment_node);
        let neg = to_mna(subtree.far_node);
        reactive_children.push(dyn_tree);
        reactive_ports.push(crate::tree::WdfPort {
            node_pos: pos,
            node_neg: neg,
            resistance: rp,
        });
    }

    // ── Step 3: GMIN regularization ─────────────────────────────────────
    const GMIN_RESISTANCE: f64 = 1e9;
    for i in 0..num_mna_nodes {
        mna.stamp_resistor(Some(i), None, GMIN_RESISTANCE);
    }

    // ── Step 4: Output probe port ───────────────────────────────────────
    let out_mna = to_mna(probe_node);
    let output_port_idx = reactive_children.len();
    reactive_children.push(DynNode::Resistor { rp: probe_resistance });
    reactive_ports.push(crate::tree::WdfPort {
        node_pos: out_mna,
        node_neg: None,
        resistance: probe_resistance,
    });

    // Pot children after ports.
    let pot_stamps: Vec<(usize, Option<usize>, Option<usize>, f64)> = pot_stamp_nodes
        .into_iter()
        .enumerate()
        .map(|(i, (n1, n2, g))| {
            let child_idx = reactive_children.len() + i;
            (child_idx, n1, n2, g)
        })
        .collect();
    reactive_children.extend(pot_children_pending);

    // ── Step 5: VS as ideal voltage source ──────────────────────────────
    let in_mna = to_mna(vs_node);
    let mna = {
        let mut mna_vs = crate::tree::MnaSystem::new(num_mna_nodes, 1);
        mna_vs.g_matrix = mna.g_matrix;
        mna_vs.stamp_voltage_source(in_mna, None, 0);
        mna_vs
    };

    // ── Step 6: Derive scattering matrix + VS injection ─────────────────
    let ports: Vec<crate::tree::WdfPort> = reactive_ports;
    let n_ports = ports.len();

    let (scattering, vs_injection) = mna.derive_scattering_and_vs_injection(&ports, 0);
    if scattering.iter().any(|&s| !s.is_finite()) {
        return None;
    }
    if vs_injection.iter().any(|&v| !v.is_finite()) {
        return None;
    }

    // ── Step 7: Build WdfStage ──────────────────────────────────────────
    let dummy_tree = DynNode::Resistor { rp: 1000.0 };
    let has_pots = !pot_stamps.is_empty();
    let recompute_mna = if has_pots { Some(mna.clone()) } else { None };
    let recompute_ports = if has_pots { Some(ports.clone()) } else { None };

    // Build interpolation table for single-pot stages
    let interp_table = if pot_stamps.len() == 1 {
        // Find the pot's max_resistance from the DynNode
        let pot_child_idx = pot_stamps[0].0;
        let max_r = match &reactive_children[pot_child_idx] {
            DynNode::Pot { max_resistance, .. } => *max_resistance,
            _ => 0.0,
        };
        if max_r > 1.0 {
            let (_, n1, n2, initial_g) = pot_stamps[0];
            Some(ScatteringInterpolationTable::build(
                &mna,
                n1,
                n2,
                initial_g,
                1.0,     // r_min: minimum pot resistance (clamped at 1Ω)
                max_r,   // r_max: full pot resistance
                &ports,
                Some(0), // VS injection mode (VS index 0)
                32,      // 32 log-spaced entries
            ))
        } else {
            None
        }
    } else {
        None
    };

    Some(WdfStage {
        tree: dummy_tree,
        root: super::stage::RootKind::PassiveRType {
            scattering,
            vs_injection,
            n_ports,
            children: reactive_children,
            output_port: output_port_idx,
            recompute_mna,
            recompute_ports,
            pot_stamps,
            needs_recompute: false,
            interp_table,
        },
        compensation: 1.0,
        oversampler: Oversampler::new(OversamplingFactor::X1),
        base_diode_model: None,
        paired_opamp: None,
        allpass_feedback: None,
        dc_block: None,
        is_source_follower: false,
        prev_source_voltage: 0.0,
        signal_flow_distance: 0,
        transformer_gain: 1.0,
        injection_node_id: usize::MAX,
        output_node_id: usize::MAX,
        is_trigger_voice: false,
        sample_counter: 0,
        root_comp_id: String::new(),
        feedback_pot_id: None,
    })
}

/// Rescue orphan output pots that sit between the last processing stage and
/// `out_node`. These are skipped by `elements_at_junction()` and
/// `bfs_passive_edges()` because those functions don't traverse through
/// `out_node`. Build a PassiveRType stage so `PotInStage` can find them.
fn rescue_orphan_output_pots(
    graph: &CircuitGraph,
    classified: &super::classify::ClassifiedCircuit,
    stages: &[WdfStage],
    multi_nl_stages: &[super::stage::MultiNlStage],
    sample_rate: f64,
    oversampling: OversamplingFactor,
) -> Vec<WdfStage> {
    use super::stage::RootKind;

    // ── Step 1: Find all pot component IDs ────────────────────────────────
    let all_pot_ids: Vec<String> = graph
        .components
        .iter()
        .filter(|c| c.kind.is_pot())
        .map(|c| c.id.clone())
        .collect();
    if all_pot_ids.is_empty() {
        return Vec::new();
    }

    // ── Step 2: Find orphan pots (not in any WDF or multi-NL stage) ──────
    let orphan_pot_ids: Vec<&str> = all_pot_ids
        .iter()
        .filter(|id| {
            // Check WDF stages: tree + PassiveRType children
            for stage in stages {
                if has_pot(&stage.tree, id) {
                    return false;
                }
                if let RootKind::PassiveRType { children, .. } = &stage.root {
                    if children.iter().any(|c| has_pot(c, id)) {
                        return false;
                    }
                }
            }
            // Check multi-NL stages
            for mnl in multi_nl_stages {
                for child in &mnl.pot_children {
                    if has_pot(child, id) {
                        return false;
                    }
                }
            }
            true
        })
        .map(|s| s.as_str())
        .collect();
    if orphan_pot_ids.is_empty() {
        return Vec::new();
    }

    // ── Step 3: BFS from out_node to collect the output passive network ──
    // Walk through passive edges (R/C/L/Pot). Stop at NL junction nodes,
    // gnd_node, vcc_node, in_node (collect edge but don't continue).
    let nl_edge_set: HashSet<usize> = classified.all_nonlinear_edge_indices.iter().copied().collect();
    let active_edge_set: HashSet<usize> = graph.active_edge_indices.iter().copied().collect();

    // Build adjacency: node → [(edge_idx, neighbor_node)]
    let mut adj: HashMap<NodeId, Vec<(usize, NodeId)>> = HashMap::new();
    for (eidx, e) in graph.edges.iter().enumerate() {
        if nl_edge_set.contains(&eidx) || active_edge_set.contains(&eidx) {
            continue;
        }
        // Skip non-simple-passive components (transformers, nonlinear, etc.)
        let comp = &graph.components[e.comp_idx];
        if !comp.kind.is_simple_passive() {
            continue;
        }
        // Skip supply-node edges (except gnd — gnd is a valid termination)
        if graph.supply_nodes.contains(&e.node_a) && e.node_a != graph.gnd_node {
            continue;
        }
        if graph.supply_nodes.contains(&e.node_b) && e.node_b != graph.gnd_node {
            continue;
        }
        adj.entry(e.node_a).or_default().push((eidx, e.node_b));
        adj.entry(e.node_b).or_default().push((eidx, e.node_a));
    }

    // Nodes that have nonlinear edges (NL junction nodes)
    let mut nl_junction_nodes: HashSet<NodeId> = HashSet::new();
    for &eidx in &classified.all_nonlinear_edge_indices {
        let e = &graph.edges[eidx];
        nl_junction_nodes.insert(e.node_a);
        nl_junction_nodes.insert(e.node_b);
    }
    // Also treat active bridge edges as NL boundaries
    for &eidx in &graph.active_edge_indices {
        let e = &graph.edges[eidx];
        nl_junction_nodes.insert(e.node_a);
        nl_junction_nodes.insert(e.node_b);
    }

    let boundary_nodes: HashSet<NodeId> = [graph.gnd_node, graph.vcc_node, graph.in_node]
        .iter()
        .copied()
        .collect();

    let mut visited_nodes: HashSet<NodeId> = HashSet::new();
    let mut output_edges: Vec<usize> = Vec::new();
    let mut queue = std::collections::VecDeque::new();
    visited_nodes.insert(graph.out_node);
    queue.push_back(graph.out_node);

    while let Some(node) = queue.pop_front() {
        if let Some(neighbors) = adj.get(&node) {
            for &(eidx, neighbor) in neighbors {
                if output_edges.contains(&eidx) {
                    continue;
                }
                output_edges.push(eidx);
                if visited_nodes.contains(&neighbor) {
                    continue;
                }
                visited_nodes.insert(neighbor);
                // Stop BFS at boundary/NL nodes (edge collected, don't continue)
                if boundary_nodes.contains(&neighbor) || nl_junction_nodes.contains(&neighbor) {
                    continue;
                }
                queue.push_back(neighbor);
            }
        }
    }

    if output_edges.is_empty() {
        return Vec::new();
    }

    // ── Step 4: Check if any orphan pot is in the output network ──────────
    let output_comp_ids: HashSet<usize> = output_edges
        .iter()
        .map(|&eidx| graph.edges[eidx].comp_idx)
        .collect();
    let has_orphan_in_output = orphan_pot_ids.iter().any(|id| {
        graph
            .components
            .iter()
            .enumerate()
            .any(|(ci, c)| c.id == *id && output_comp_ids.contains(&ci))
    });
    if !has_orphan_in_output {
        return Vec::new();
    }

    // ── Step 5: Find VS injection node ────────────────────────────────────
    // The interior node (not out_node, gnd, vcc) with smallest dist_from_in.
    // This is where signal enters the output network from the processing chain.
    let exclude: HashSet<NodeId> = [graph.out_node, graph.gnd_node, graph.vcc_node]
        .iter()
        .copied()
        .collect();
    let vs_node = {
        let mut best_node = None;
        let mut best_dist = usize::MAX;
        for &node in &visited_nodes {
            if exclude.contains(&node) {
                continue;
            }
            if let Some(&d) = classified.dist_from_in.get(&node) {
                if d < best_dist {
                    best_dist = d;
                    best_node = Some(node);
                }
            }
        }
        match best_node {
            Some(n) => n,
            None => return Vec::new(),
        }
    };

    // ── Step 6: Build PassiveRType stage ──────────────────────────────────
    // Standard 1 GΩ probe — 3-terminal divider pots work at any load
    // impedance because the voltage division is internal to the pot.
    if let Some(mut stage) = build_orphan_output_mna_stage(
        graph,
        &output_edges,
        vs_node,
        graph.out_node,
        1e9, // standard high-Z probe
        sample_rate,
    ) {
        // Process last — after all NL stages, before the output probe.
        stage.signal_flow_distance = usize::MAX - 1;
        vec![stage]
    } else {
        Vec::new()
    }
}

/// Build a WDF stage for simple 2-element passive circuits (RC, RL, resistor divider).
///
/// Uses VoltageSourceDriver at root with Series(source, load) tree.
/// The corrected VS scattering `a_root = 2·V_in - b_tree` gives the exact
/// bilinear-transform pole, and the output is extracted (negated) at the
/// series junction (right child = load element).
fn build_output_rooted_stage(
    graph: &CircuitGraph,
    passive_edges: &[usize],
    sample_rate: f64,
    _oversampling: OversamplingFactor,
) -> Option<WdfStage> {
    if passive_edges.len() != 2 {
        return None;
    }

    // Find the load edge (connects out_node to gnd_node) and source edge.
    let mut load_idx = None;
    let mut source_idx = None;
    for &eidx in passive_edges {
        let e = &graph.edges[eidx];
        let connects_out_gnd = (e.node_a == graph.out_node && e.node_b == graph.gnd_node)
            || (e.node_a == graph.gnd_node && e.node_b == graph.out_node);
        if connects_out_gnd {
            load_idx = Some(eidx);
        } else {
            source_idx = Some(eidx);
        }
    }
    let load_eidx = load_idx?;
    let source_eidx = source_idx?;

    // Verify source edge connects in→out (through the circuit).
    let source_edge = &graph.edges[source_eidx];
    let connects_in_out = (source_edge.node_a == graph.in_node
        && source_edge.node_b == graph.out_node)
        || (source_edge.node_a == graph.out_node && source_edge.node_b == graph.in_node);
    if !connects_in_out {
        return None;
    }

    let load_edge = &graph.edges[load_eidx];
    let source_comp = &graph.components[source_edge.comp_idx];
    let load_comp = &graph.components[load_edge.comp_idx];

    // Verify this is a supported 2-element passive topology (RC, RL, or RR).
    // At least one element must be a resistor, and both must be basic passives (R/C/L).
    {
        let is_r_c_or_l = |k: &dyn Component| -> bool {
            k.as_any().downcast_ref::<ResistorComp>().is_some()
                || k.as_any().downcast_ref::<CapacitorComp>().is_some()
                || k.as_any().downcast_ref::<InductorComp>().is_some()
        };
        let has_resistor = source_comp.kind.as_any().downcast_ref::<ResistorComp>().is_some()
            || load_comp.kind.as_any().downcast_ref::<ResistorComp>().is_some();
        if !is_r_c_or_l(source_comp.kind.as_ref()) || !is_r_c_or_l(load_comp.kind.as_ref()) || !has_resistor {
            return None;
        }
    }

    // Build WDF tree: Series(source, load).
    // Source element (in→out) on left, load element (out→gnd) on right.
    // Output is extracted at right child (load port) via series_junction_voltage.
    let source_dyn = make_leaf(source_edge.comp_idx, source_comp, None, sample_rate);
    let load_dyn = make_leaf(load_edge.comp_idx, load_comp, None, sample_rate);
    let r1 = source_dyn.port_resistance();
    let r2 = load_dyn.port_resistance();
    let rp = r1 + r2;
    let tree = DynNode::Series {
        left: Box::new(source_dyn),
        right: Box::new(load_dyn),
        rp,
        gamma: r1 / rp,
        b1: 0.0,
        b2: 0.0,
    };

    Some(WdfStage {
        tree,
        root: RootKind::VoltageSourceDriver,
        compensation: 1.0,
        // Linear passive stage — no nonlinearity means no aliasing,
        // so X1 avoids double-counting the oversampling the runner already applied.
        oversampler: Oversampler::new(OversamplingFactor::X1),
        base_diode_model: None,
        paired_opamp: None,
        allpass_feedback: None,
        dc_block: None,
        is_source_follower: false,
        prev_source_voltage: 0.0,
        signal_flow_distance: 0,
        transformer_gain: 1.0,
        injection_node_id: usize::MAX,
        output_node_id: usize::MAX,
        is_trigger_voice: false,
        sample_counter: 0,
        root_comp_id: String::new(),
        feedback_pot_id: None,
    })
}

// ═══════════════════════════════════════════════════════════════════════════
// Compile options
// ═══════════════════════════════════════════════════════════════════════════

/// Options for pedal compilation.
pub struct CompileOptions {
    pub oversampling: OversamplingFactor,
    pub tolerance: ToleranceEngine,
    pub thermal: bool,
    /// When true, collapse ALL nonlinear elements into a single MultiNlStage
    /// instead of planning them as individual stages. Used for sidechain
    /// sub-circuits where the entire NL network should be solved as one
    /// multi-junction system (shared MNA + scattering matrix).
    pub collapse_nl: bool,
}

impl Default for CompileOptions {
    fn default() -> Self {
        Self {
            oversampling: OversamplingFactor::X4,
            tolerance: ToleranceEngine::ideal(),
            thermal: false,
            collapse_nl: false,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// ═══════════════════════════════════════════════════════════════════════════
// VCO / VCA binding builders
// ═══════════════════════════════════════════════════════════════════════════

/// Build VCO runtime bindings from the pedal definition.
///
/// For each VCO component, creates a `VcoBinding` with:
/// - An audio-rate oscillator set to the base frequency
/// - The default waveform output
/// - The output node resolved from net connections (Option A: use target's node)
fn build_vco_bindings(
    pedal: &PedalDef,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Vec<VcoBinding> {
    let mut vcos = Vec::new();

    for comp in &pedal.components {
        if let Some(vco_comp) = comp.kind.as_any().downcast_ref::<VcoComp>() {
            let mut vco = crate::elements::Vco::new(sample_rate);
            vco.set_base_freq(vco_comp.base_freq);

            // Map DSL waveform to runtime waveform
            let waveform = match &vco_comp.waveform {
                crate::dsl::VcoWaveformDsl::Saw => crate::elements::VcoWaveform::Saw,
                crate::dsl::VcoWaveformDsl::Triangle => crate::elements::VcoWaveform::Triangle,
                crate::dsl::VcoWaveformDsl::Pulse => crate::elements::VcoWaveform::Pulse,
            };

            // Resolve output node: find where {vco_id}.out (or .saw/.tri/.pulse) connects.
            // Use Option A: resolve to the *other* component's graph node ID.
            let mut output_node_id = usize::MAX;
            let mut resolved_waveform = waveform;

            for net in &pedal.nets {
                if let Pin::ComponentPin { component, pin } = &net.from {
                    if component == &comp.id {
                        // Check if the pin specifies a specific waveform
                        match pin.as_str() {
                            "saw" => resolved_waveform = crate::elements::VcoWaveform::Saw,
                            "tri" => resolved_waveform = crate::elements::VcoWaveform::Triangle,
                            "pulse" => resolved_waveform = crate::elements::VcoWaveform::Pulse,
                            "out" | "output" => {} // use default waveform
                            _ => continue,
                        }
                        // Resolve to the target's graph node
                        for to_pin in &net.to {
                            if let Pin::ComponentPin { component: target_comp, pin: target_pin } = to_pin {
                                if let Some(node) = resolve_pin_to_node(target_comp, target_pin, pedal, graph) {
                                    output_node_id = node;
                                    break;
                                }
                            }
                            if let Pin::Reserved(node_name) = to_pin {
                                if node_name == "out" || node_name == "output" {
                                    output_node_id = graph.out_node;
                                } else if node_name == "in" || node_name == "input" {
                                    output_node_id = graph.in_node;
                                }
                            }
                        }
                    }
                }
                // Also check if VCO is a target (reverse direction)
                for to_pin in &net.to {
                    if let Pin::ComponentPin { component, pin } = to_pin {
                        if component == &comp.id {
                            match pin.as_str() {
                                "saw" => resolved_waveform = crate::elements::VcoWaveform::Saw,
                                "tri" => resolved_waveform = crate::elements::VcoWaveform::Triangle,
                                "pulse" => resolved_waveform = crate::elements::VcoWaveform::Pulse,
                                "out" | "output" => {
                                    // Resolve the source end
                                    if let Pin::ComponentPin { component: src_comp, pin: src_pin } = &net.from {
                                        if let Some(node) = resolve_pin_to_node(src_comp, src_pin, pedal, graph) {
                                            output_node_id = node;
                                        }
                                    }
                                }
                                _ => {}
                            }
                        }
                    }
                }
            }

            // If no output node found, default to circuit output node
            if output_node_id == usize::MAX {
                output_node_id = graph.out_node;
            }

            vcos.push(VcoBinding {
                vco,
                waveform: resolved_waveform,
                output_node_id,
                comp_id: comp.id.clone(),
            });
        }
    }

    vcos
}

/// Build VCA runtime bindings from the pedal definition.
///
/// For each VCA component, creates a `VcaBinding` with:
/// - A VCA gain element
/// - An ADSR envelope for gating
/// - Input/output nodes resolved from net connections
/// - Trigger index if CV is connected to a trigger_input
fn build_vca_bindings(
    pedal: &PedalDef,
    graph: &CircuitGraph,
    trigger_id_to_idx: &HashMap<String, usize>,
    sample_rate: f64,
) -> Vec<VcaBinding> {
    let mut vcas = Vec::new();

    for comp in &pedal.components {
        if let Some(_vca_comp) = comp.kind.as_any().downcast_ref::<VcaComp>() {
            let vca = crate::elements::Vca::new();
            let mut envelope = crate::elements::AdsrEnvelope::new(sample_rate);
            // Percussive defaults: fast attack, short decay, no sustain.
            // The envelope naturally shapes the hit without needing gate_off.
            envelope.set_attack(0.001);  // 1ms
            envelope.set_decay(0.15);    // 150ms
            envelope.set_sustain(0.0);   // no sustain — percussive
            envelope.set_release(0.05);  // 50ms

            let mut input_node_id = usize::MAX;
            let mut output_node_id = usize::MAX;
            let mut trigger_idx = None;

            // Scan nets for VCA pin connections
            for net in &pedal.nets {
                // Check if VCA is the source
                if let Pin::ComponentPin { component, pin } = &net.from {
                    if component == &comp.id {
                        match pin.as_str() {
                            "out" | "output" => {
                                for to_pin in &net.to {
                                    if let Pin::ComponentPin { component: target_comp, pin: target_pin } = to_pin {
                                        if let Some(node) = resolve_pin_to_node(target_comp, target_pin, pedal, graph) {
                                            output_node_id = node;
                                            break;
                                        }
                                    }
                                    if let Pin::Reserved(node_name) = to_pin {
                                        if node_name == "out" || node_name == "output" {
                                            output_node_id = graph.out_node;
                                        }
                                    }
                                }
                            }
                            _ => {}
                        }
                    }
                }
                // Check if VCA is a target
                for to_pin in &net.to {
                    if let Pin::ComponentPin { component, pin } = to_pin {
                        if component == &comp.id {
                            match pin.as_str() {
                                "in" | "input" => {
                                    if let Pin::ComponentPin { component: src_comp, pin: src_pin } = &net.from {
                                        if let Some(node) = resolve_pin_to_node(src_comp, src_pin, pedal, graph) {
                                            input_node_id = node;
                                        }
                                    }
                                }
                                "out" | "output" => {
                                    if let Pin::ComponentPin { component: src_comp, pin: src_pin } = &net.from {
                                        if let Some(node) = resolve_pin_to_node(src_comp, src_pin, pedal, graph) {
                                            output_node_id = node;
                                        }
                                    }
                                }
                                "cv" => {
                                    // Check if CV is connected to a trigger_input
                                    if let Pin::ComponentPin { component: src_comp, .. } = &net.from {
                                        if let Some(&idx) = trigger_id_to_idx.get(src_comp.as_str()) {
                                            trigger_idx = Some(idx);
                                        }
                                    }
                                }
                                _ => {}
                            }
                        }
                    }
                }
            }

            // Default: if no output found, use circuit output
            if output_node_id == usize::MAX {
                output_node_id = graph.out_node;
            }

            vcas.push(VcaBinding {
                vca,
                envelope,
                trigger_idx,
                input_node_id,
                output_node_id,
                comp_id: comp.id.clone(),
            });
        }
    }

    vcas
}

/// Resolve a component pin to a circuit graph node ID.
/// Returns None if the component or pin can't be found in the graph.
fn resolve_pin_to_node(
    comp_id: &str,
    _pin: &str,
    _pedal: &PedalDef,
    graph: &CircuitGraph,
) -> Option<usize> {
    // Find edges in the graph that involve this component
    for edge in &graph.edges {
        if graph.components[edge.comp_idx].id == comp_id {
            // Return node_a as a reasonable default
            // (for Virtual components there won't be edges, but for components
            // connected to the VCO/VCA via nets, there will be)
            return Some(edge.node_a);
        }
    }
    // Check if the component is at a known node (in/out/gnd)
    // This handles cases where the target is a circuit terminal
    if comp_id == "in" || comp_id == "input" {
        return Some(graph.in_node);
    }
    if comp_id == "out" || comp_id == "output" {
        return Some(graph.out_node);
    }
    None
}

// Main compilation pipeline
// ═══════════════════════════════════════════════════════════════════════════

/// Compile a pedal definition with default options.
pub fn compile_pedal(pedal: &PedalDef, sample_rate: f64) -> Result<CompiledPedal, String> {
    compile_pedal_with_options(pedal, sample_rate, CompileOptions::default())
}

/// Compile a pedal definition with custom options.
///
/// Orchestrates the 6-pass compilation pipeline:
/// 1. Graph construction
/// 2. Element classification
/// 3. Op-amp analysis
/// 4. Stage planning
/// 5. Tree building
/// 6. Binding & assembly
pub fn compile_pedal_with_options(
    pedal: &PedalDef,
    sample_rate: f64,
    options: CompileOptions,
) -> Result<CompiledPedal, String> {
    let oversampling = options.oversampling;
    let tolerance = options.tolerance;
    let enable_thermal = options.thermal;
    let _collapse_nl = options.collapse_nl;

    // ══ Pass 0.5: Validation ═══════════════════════════════════════════
    // Run validation to catch pin errors early. Only hard-fail on
    // unknown-pin errors — other Error-severity warnings (e.g. no-signal-path)
    // may have false positives for sub-circuits and complex topologies.
    let warnings = super::validate::validate_pedal(pedal);
    for w in &warnings {
        if w.severity == super::validate::Severity::Error && w.code == "unknown-pin" {
            return Err(w.message.clone());
        }
    }
    for w in &warnings {
        if w.severity == super::validate::Severity::Warning {
            eprintln!("[pedalkernel] {}", w);
        }
    }

    // ══ Pass 0: Graph construction ════════════════════════════════════
    let mut graph = CircuitGraph::from_pedal(pedal);

    // Post-construction: resolve context-dependent edge kinds (JFET Vr, OTA linear).
    super::graph::resolve_components(&mut graph, pedal);

    // Apply component tolerance.
    for (i, comp) in graph.components.iter_mut().enumerate() {
        if let Some(r) = comp.kind.as_any_mut().downcast_mut::<ResistorComp>() {
            r.value = tolerance.apply_resistor(r.value, i);
        } else if let Some(c) = comp.kind.as_any_mut().downcast_mut::<CapacitorComp>() {
            c.config.value = tolerance.apply_capacitor(c.config.value, i);
        } else if let Some(p) = comp.kind.as_any_mut().downcast_mut::<PotComp>() {
            p.max_r = tolerance.apply_resistor(p.max_r, i);
        }
    }

    // Supply voltage (needed early for multi-NL VCC bias injection).
    let supply_voltage = pedal.supplies.first().map_or(9.0, |s| s.config.voltage);

    // ══ Pass 1: Element classification ════════════════════════════════
    let classified = super::classify::classify_circuit(&graph, pedal);

    // ══ Pass 2: Op-amp analysis ═══════════════════════════════════════
    let opamp_analysis = super::opamp_analysis::analyze_opamps(&graph, pedal);
    let opamp_feedback_gain = 1.0_f64;

    // Detect opamps whose feedback components share a graph node with
    // nonlinear element junctions. For these opamps, skip build_feedback_tree
    // so the NL stage keeps the feedback components (avoids double-counting
    // that kills signal level in circuits like RAT, Tube Screamer, etc.).
    let nl_junction_nodes: HashSet<super::graph::NodeId> = classified
        .nonlinear_elements
        .iter()
        .flat_map(|e| e.junction_nodes.iter().copied())
        .collect();

    let skip_feedback_tree_opamps: HashSet<String> = opamp_analysis
        .feedback_loops
        .iter()
        .filter(|info| {
            // Collect graph nodes that this opamp's feedback components touch.
            let fb_nodes: HashSet<super::graph::NodeId> = graph
                .edges
                .iter()
                .filter(|e| {
                    info.feedback_comp_ids
                        .contains(&graph.components[e.comp_idx].id)
                })
                .flat_map(|e| [e.node_a, e.node_b])
                .collect();

            // Skip feedback tree if any passive edge connects the feedback
            // node set to an NL junction (direct overlap OR 1-hop bridge).
            // - Direct: RAT feedback pot IS at diode junction
            // - 1-hop: Klon R_clip bridges diode junction to U3.neg
            graph.edges.iter().enumerate().any(|(idx, e)| {
                if classified.all_nonlinear_edge_indices.contains(&idx)
                    || graph.active_edge_indices.contains(&idx)
                {
                    return false;
                }
                (fb_nodes.contains(&e.node_a) && nl_junction_nodes.contains(&e.node_b))
                    || (fb_nodes.contains(&e.node_b) && nl_junction_nodes.contains(&e.node_a))
            })
        })
        .map(|info| info.comp_id.clone())
        .collect();

    // Build op-amp feedback stages (inverting, non-inverting).
    let mut stages: Vec<WdfStage> = Vec::new();
    let opamp_feedback_stages = super::opamp_analysis::build_opamp_feedback_stages(
        &opamp_analysis,
        pedal,
        &graph,
        stages.len(),
        sample_rate,
        oversampling,
        &skip_feedback_tree_opamps,
    );
    stages.extend(opamp_feedback_stages);

    // Build standalone op-amp stages (no feedback).
    let opamp_stages = super::opamp_analysis::build_standalone_opamp_stages(
        pedal,
        &opamp_analysis.feedback_opamp_ids,
        sample_rate,
    );

    // ══ Pass 3: Stage planning ════════════════════════════════════════
    // Detect modulation-controlled elements before planning.
    let envelope_controlled_otas = detect_envelope_controlled_otas(pedal);

    // Collect edge indices for opamp feedback path components so the
    // multi-NL planner won't BFS through them (prevents pot duplication).
    // Exclude opamps that share junctions with NL elements — their feedback
    // components stay with the NL stage.
    let opamp_feedback_edges: HashSet<usize> = {
        let mut comp_ids: HashSet<String> = HashSet::new();
        for info in &opamp_analysis.feedback_loops {
            if !skip_feedback_tree_opamps.contains(&info.comp_id) {
                comp_ids.extend(info.feedback_comp_ids.iter().cloned());
            }
        }
        graph
            .edges
            .iter()
            .enumerate()
            .filter(|(_, e)| comp_ids.contains(&graph.components[e.comp_idx].id))
            .map(|(idx, _)| idx)
            .collect()
    };

    let (stage_plans, push_pull_plans, multi_nl_plans, pp_transformer_edges, bjt_bias_analysis) =
        super::plan::plan_stages(&classified, &graph, sample_rate, &envelope_controlled_otas, &opamp_feedback_edges);

    // ══ Pass 4: Tree building ═════════════════════════════════════════
    // Detect JFETs that are LFO-controlled so they use variable-resistance mode
    // instead of full nonlinear NR solving.
    let lfo_controlled_jfets = detect_lfo_controlled_jfets(pedal);

    // Build nonlinear WDF stages from plans.
    // Triode plans that fail SP reduction fall back to single-NL MNA stages.
    let (nonlinear_stages, triode_fallback_stages) = super::build::build_stages(
        &stage_plans,
        &classified,
        &graph,
        &opamp_analysis,
        sample_rate,
        oversampling,
        &pp_transformer_edges,
        &lfo_controlled_jfets,
        supply_voltage,
    );
    stages.extend(nonlinear_stages);

    // Build push-pull stages.
    let push_pull_stages = super::build::build_push_pull_stages(
        &push_pull_plans,
        &classified,
        &graph,
        sample_rate,
        oversampling,
        &pp_transformer_edges,
    );

    // Build multi-NL stages (R-type adaptor approach).
    let mut multi_nl_stages = super::build::build_multi_nl_stages(
        &multi_nl_plans,
        &classified,
        &graph,
        sample_rate,
        oversampling,
        supply_voltage,
    );

    // Add triode fallback stages (SP-failed triodes built as single-NL MNA).
    multi_nl_stages.extend(triode_fallback_stages);

    // ══ Orphan output pot rescue ═════════════════════════════════════
    // Pots between the last processing stage and out_node are skipped
    // by elements_at_junction/bfs_passive_edges. Build a PassiveRType
    // stage so PotInStage can find and control them.
    if !stages.is_empty() || !multi_nl_stages.is_empty() {
        let orphan_stages = rescue_orphan_output_pots(
            &graph, &classified, &stages, &multi_nl_stages,
            sample_rate, oversampling,
        );
        stages.extend(orphan_stages);
    }

    // ══ Passive-only fallback ═════════════════════════════════════════
    let mut passive_attenuation = 1.0;
    // Maps trigger component ID → (WDF stage index, injection node ID).
    let mut trigger_stage_map: HashMap<String, (usize, NodeId)> = HashMap::new();

    if stages.is_empty() && multi_nl_stages.is_empty() {
        let has_reactive = pedal.components.iter().any(|c| {
            c.kind.as_any().downcast_ref::<CapacitorComp>().is_some()
                || c.kind.as_any().downcast_ref::<InductorComp>().is_some()
                || c.kind.is_pot()
        });

        if has_reactive && !graph.trigger_nodes.is_empty() {
            // ── Per-voice trigger stages ─────────────────────────────
            // Build one MNA stage per trigger, each with VS at the trigger's
            // injection node and output probe at out_node. Each voice stage
            // has independent reactive element state so voices decay independently.
            //
            // Each voice gets only the passive edges reachable from its trigger
            // node through the passive subgraph. This ensures voices with
            // different component values (e.g., different L/C for pitch) get
            // distinct scattering matrices.
            let all_passive_edges: Vec<usize> = graph
                .edges
                .iter()
                .enumerate()
                .filter(|(_, e)| {
                    graph.components[e.comp_idx].kind.is_simple_passive()
                })
                .map(|(i, _)| i)
                .collect();

            // Build adjacency from passive edges for reachability flood-fill.
            let mut passive_adj: HashMap<NodeId, Vec<(NodeId, usize)>> = HashMap::new();
            for &eidx in &all_passive_edges {
                let e = &graph.edges[eidx];
                passive_adj.entry(e.node_a).or_default().push((e.node_b, eidx));
                passive_adj.entry(e.node_b).or_default().push((e.node_a, eidx));
            }

            // Distance-based voice assignment: each non-gnd node belongs to
            // the nearest trigger. Equidistant nodes are shared (mix junctions).
            // Build gnd-free adjacency for distance computation.
            let mut adj_no_gnd: HashMap<NodeId, Vec<(NodeId, usize)>> = HashMap::new();
            for &eidx in &all_passive_edges {
                let e = &graph.edges[eidx];
                if e.node_a != graph.gnd_node && e.node_b != graph.gnd_node {
                    adj_no_gnd.entry(e.node_a).or_default().push((e.node_b, eidx));
                    adj_no_gnd.entry(e.node_b).or_default().push((e.node_a, eidx));
                }
            }

            // BFS from each trigger to compute distances.
            let trigger_list: Vec<(String, NodeId)> = graph.trigger_nodes.clone();
            let num_triggers = trigger_list.len();
            let mut node_distances: HashMap<NodeId, Vec<usize>> = HashMap::new();
            for (ti, (_, trigger_node)) in trigger_list.iter().enumerate() {
                let mut dist: HashMap<NodeId, usize> = HashMap::new();
                let mut queue = std::collections::VecDeque::new();
                dist.insert(*trigger_node, 0);
                queue.push_back(*trigger_node);
                while let Some(node) = queue.pop_front() {
                    let d = dist[&node];
                    if let Some(neighbors) = adj_no_gnd.get(&node) {
                        for &(neighbor, _) in neighbors {
                            if !dist.contains_key(&neighbor) {
                                dist.insert(neighbor, d + 1);
                                queue.push_back(neighbor);
                            }
                        }
                    }
                }
                for (&node, &d) in &dist {
                    let dists = node_distances
                        .entry(node)
                        .or_insert_with(|| vec![usize::MAX; num_triggers]);
                    dists[ti] = d;
                }
            }

            // Assign nodes to their closest trigger. Equidistant = shared.
            let mut voice_nodes: Vec<HashSet<NodeId>> =
                vec![HashSet::new(); num_triggers];
            let mut shared_nodes: HashSet<NodeId> = HashSet::new();
            for (node, dists) in &node_distances {
                let min_d = *dists.iter().min().unwrap_or(&usize::MAX);
                if min_d == usize::MAX {
                    continue;
                }
                let closest: Vec<usize> = dists
                    .iter()
                    .enumerate()
                    .filter(|(_, &d)| d == min_d)
                    .map(|(i, _)| i)
                    .collect();
                if closest.len() == 1 {
                    voice_nodes[closest[0]].insert(*node);
                } else {
                    shared_nodes.insert(*node);
                }
            }

            // Shared output edges: edges between shared nodes and/or output.
            let all_voice_nodes: HashSet<NodeId> =
                voice_nodes.iter().flatten().copied().collect();
            let mut shared_output_edges: HashSet<usize> = HashSet::new();
            for &eidx in &all_passive_edges {
                let e = &graph.edges[eidx];
                let a_shared = shared_nodes.contains(&e.node_a)
                    || (!all_voice_nodes.contains(&e.node_a)
                        && e.node_a != graph.gnd_node);
                let b_shared = shared_nodes.contains(&e.node_b)
                    || (!all_voice_nodes.contains(&e.node_b)
                        && e.node_b != graph.gnd_node);
                if a_shared && b_shared {
                    shared_output_edges.insert(eidx);
                }
            }

            for (voice_idx, (comp_id, trigger_node)) in
                trigger_list.iter().enumerate()
            {
                // Voice edges: edges with at least one non-shared endpoint
                // owned by this voice, plus shared output edges.
                let voice_set = &voice_nodes[voice_idx];
                let mut voice_edges: HashSet<usize> = HashSet::new();
                for &eidx in &all_passive_edges {
                    let e = &graph.edges[eidx];
                    if voice_set.contains(&e.node_a)
                        || voice_set.contains(&e.node_b)
                    {
                        voice_edges.insert(eidx);
                    }
                }
                voice_edges.extend(&shared_output_edges);
                let voice_passive_edges: Vec<usize> =
                    voice_edges.into_iter().collect();

                let stage_idx = stages.len();
                if let Some(mut voice_stage) = build_orphan_output_mna_stage(
                    &graph,
                    &voice_passive_edges,
                    *trigger_node,
                    graph.out_node,
                    100_000.0, // 100kΩ probe — matches typical mix bus impedance
                    sample_rate,
                ) {
                    voice_stage.injection_node_id = *trigger_node;
                    voice_stage.output_node_id = graph.out_node;
                    voice_stage.is_trigger_voice = true;
                    trigger_stage_map.insert(
                        comp_id.clone(),
                        (stage_idx, *trigger_node),
                    );
                    stages.push(voice_stage);
                }
            }
        } else if has_reactive {
            if let Some(stage) = build_passive_wdf_stage(&graph, sample_rate, oversampling) {
                stages.push(stage);
            }
        } else {
            passive_attenuation = compute_resistor_divider_gain(&graph);
        }
    }

    // ══ Transformer gain ══════════════════════════════════════════════
    let mut transformer_gain = 1.0;
    for comp in &pedal.components {
        if let Some(cfg) = comp.kind.transformer_config() {
            let pin_matches = |p: &Pin, comp_id: &str, pin_name: &str| -> bool {
                matches!(p, Pin::ComponentPin { component, pin } if component == comp_id && pin == pin_name)
            };
            let is_input_pin =
                |p: &Pin| -> bool { matches!(p, Pin::Reserved(name) if name == "in") };
            let is_output_pin =
                |p: &Pin| -> bool { matches!(p, Pin::Reserved(name) if name == "out") };

            let output_from_secondary = pedal.nets.iter().any(|net| {
                let has_secondary =
                    pin_matches(&net.from, &comp.id, "c") || pin_matches(&net.from, &comp.id, "d");
                let has_secondary_to = net
                    .to
                    .iter()
                    .any(|p| pin_matches(p, &comp.id, "c") || pin_matches(p, &comp.id, "d"));
                let has_output =
                    is_output_pin(&net.from) || net.to.iter().any(|p| is_output_pin(p));
                (has_secondary || has_secondary_to) && has_output
            });
            let input_to_primary = pedal.nets.iter().any(|net| {
                let has_primary =
                    pin_matches(&net.from, &comp.id, "a") || pin_matches(&net.from, &comp.id, "b");
                let has_primary_to = net
                    .to
                    .iter()
                    .any(|p| pin_matches(p, &comp.id, "a") || pin_matches(p, &comp.id, "b"));
                let has_input = is_input_pin(&net.from) || net.to.iter().any(|p| is_input_pin(p));
                (has_primary || has_primary_to) && has_input
            });

            if input_to_primary && output_from_secondary {
                transformer_gain *= 1.0 / cfg.turns_ratio;
            }
        }
    }

    // ══ Slew rate limiters ════════════════════════════════════════════
    let mut slew_limiters = Vec::new();
    for comp in &pedal.components {
        if let Some(ot) = comp.kind.op_amp_type() {
            if !ot.is_ota() {
                slew_limiters.push(SlewRateLimiter::new(ot.slew_rate(), sample_rate));
            }
        }
    }

    // ══ BBD delay lines ═══════════════════════════════════════════════
    let mut bbds = Vec::new();
    let mut bbd_id_to_idx: HashMap<String, usize> = HashMap::new();
    for comp in &pedal.components {
        if let Some(bbd_comp) = comp.kind.as_any().downcast_ref::<BbdComp>() {
            let idx = bbds.len();
            bbd_id_to_idx.insert(comp.id.clone(), idx);
            let model = match &bbd_comp.bbd_type {
                BbdType::Mn3207 => BbdModel::mn3207(),
                BbdType::Mn3007 => BbdModel::mn3007(),
                BbdType::Mn3005 => BbdModel::mn3005(),
            };
            bbds.push(BbdDelayLine::new(model, sample_rate));
        }
    }

    // ══ Generic delay lines ═══════════════════════════════════════════
    let mut delay_lines: Vec<DelayLineBinding> = Vec::new();
    let mut delay_id_to_idx: HashMap<String, usize> = HashMap::new();

    for comp in &pedal.components {
        if let Some(dl_comp) = comp.kind.as_any().downcast_ref::<DelayLineComp>() {
            let idx = delay_lines.len();
            delay_id_to_idx.insert(comp.id.clone(), idx);
            let mut dl =
                crate::elements::DelayLine::new(dl_comp.min_delay, dl_comp.max_delay, sample_rate, dl_comp.interpolation);
            dl.set_medium(dl_comp.medium);
            delay_lines.push(DelayLineBinding {
                delay_line: dl,
                taps: vec![1.0],
                comp_id: comp.id.clone(),
            });
        }
    }

    for comp in &pedal.components {
        if let Some(tap_comp) = comp.kind.as_any().downcast_ref::<TapComp>() {
            if let Some(&dl_idx) = delay_id_to_idx.get(&tap_comp.parent_id) {
                delay_lines[dl_idx].taps.push(tap_comp.ratio);
            }
        }
    }

    for dl_binding in &mut delay_lines {
        if dl_binding.delay_line.medium() != crate::elements::Medium::None {
            let taps = dl_binding.taps.clone();
            dl_binding.delay_line.configure_zones_from_taps(&taps, None);
        }
    }

    // ══ Rail saturation model ═════════════════════════════════════════
    let rail_saturation = {
        let mut has_opamp = false;
        let mut has_bjt = false;
        let mut has_fet = false;
        let mut has_tube = false;
        let mut tube_mu = 100.0_f64;
        let mut opamp_swing = 0.85_f64;
        for comp in &pedal.components {
            if let Some(ot) = comp.kind.op_amp_type() {
                if !ot.is_ota() {
                    has_opamp = true;
                    opamp_swing = match ot {
                        OpAmpType::Tl072 | OpAmpType::Tl082 | OpAmpType::Generic => 0.92,
                        OpAmpType::Ne5532 => 0.90,
                        OpAmpType::Jrc4558 | OpAmpType::Rc4558 => 0.87,
                        OpAmpType::Lm308 | OpAmpType::Lm741 | OpAmpType::Op07 => 0.85,
                        _ => 0.85,
                    };
                }
            } else if comp.kind.is_bjt() {
                has_bjt = true;
            } else if comp.kind.is_jfet() || comp.kind.is_mosfet() {
                has_fet = true;
            } else if let Some(triode) = comp.kind.as_any().downcast_ref::<TriodeComp>() {
                has_tube = true;
                tube_mu = TriodeModel::try_by_name(&triode.model)
                    .map(|m| m.mu)
                    .unwrap_or(100.0);
            } else if comp.kind.as_any().downcast_ref::<VariMuComp>().is_some() {
                has_tube = true;
                tube_mu = 35.0;
            } else if comp.kind.as_any().downcast_ref::<PentodeComp>().is_some() {
                has_tube = true;
                tube_mu = 200.0;
            }
        }
        let has_source_follower = stages.iter().any(|s| s.is_source_follower);
        if has_tube {
            RailSaturation::Tube { mu: tube_mu }
        } else if has_bjt {
            let vce_sat = if classified.has_germanium { 0.3 } else { 0.2 };
            RailSaturation::Bjt { vce_sat }
        } else if has_fet && !has_source_follower {
            RailSaturation::Fet
        } else if has_opamp {
            RailSaturation::OpAmp {
                output_swing_ratio: opamp_swing,
            }
        } else {
            RailSaturation::None
        }
    };

    // ══ Pass 5: Binding & assembly ════════════════════════════════════
    let lfo_ids: Vec<String> = pedal
        .components
        .iter()
        .filter_map(|c| {
            if c.kind.as_any().downcast_ref::<LfoComp>().is_some() {
                Some(c.id.clone())
            } else {
                None
            }
        })
        .collect();

    // Collect trigger inputs.
    let mut trigger_id_to_idx: HashMap<String, usize> = HashMap::new();
    let mut triggers: Vec<super::compiled::TriggerState> = pedal
        .components
        .iter()
        .filter(|c| c.kind.is_trigger())
        .enumerate()
        .map(|(idx, c)| {
            trigger_id_to_idx.insert(c.id.clone(), idx);
            let amplitude = pedal.supplies.first().map(|s| s.config.voltage).unwrap_or(9.0);
            super::compiled::TriggerState::new(amplitude)
        })
        .collect();

    // Wire per-voice trigger → stage mapping from the passive-only fallback.
    for (comp_id, (stage_idx, injection_node)) in &trigger_stage_map {
        if let Some(&trig_idx) = trigger_id_to_idx.get(comp_id) {
            triggers[trig_idx].target_stage = Some(*stage_idx);
            triggers[trig_idx].injection_node = *injection_node;
        }
    }

    // Build MIDI note → trigger index map from midi_bindings.
    let midi_trigger_map: HashMap<u8, usize> = pedal
        .midi_bindings
        .iter()
        .filter_map(|mb| {
            trigger_id_to_idx.get(&mb.component).map(|&idx| (mb.note, idx))
        })
        .collect();

    // Sidechain construction — must happen before build_controls so we have
    // the component → sidechain index mapping for control routing.
    let (sidechains, sidechain_comp_ids) =
        super::bind::build_sidechains(pedal, &graph, sample_rate);

    let (controls, bbd_mix_pot_id) = super::bind::build_controls(
        pedal,
        &stages,
        &multi_nl_stages,
        &lfo_ids,
        &delay_id_to_idx,
        delay_lines.is_empty(),
        &sidechain_comp_ids,
        &bbd_id_to_idx,
        &trigger_id_to_idx,
    );

    let physical_gain = opamp_feedback_gain * passive_attenuation * transformer_gain;
    // Gain range for initial pre-gain computation.
    // Maps the physical gain to a useful input-level sweep.
    let gain_range_final = (physical_gain.max(0.1), physical_gain.max(0.1) * 10.0);
    let pre_gain = physical_gain;

    let lfos = super::bind::build_lfo_bindings(
        pedal,
        &stages,
        &multi_nl_stages,
        &delay_id_to_idx,
        sample_rate,
    );
    let envelopes = super::bind::build_envelope_bindings(
        pedal,
        &stages,
        &multi_nl_stages,
        &delay_id_to_idx,
        sample_rate,
    );
    let vcos = build_vco_bindings(pedal, &graph, sample_rate);
    let vcas = build_vca_bindings(pedal, &graph, &trigger_id_to_idx, sample_rate);
    // Thermal model.
    let thermal = if enable_thermal && classified.has_germanium {
        Some(ThermalModel::germanium_fuzz(sample_rate))
    } else if enable_thermal {
        Some(ThermalModel::silicon_standard(sample_rate))
    } else {
        None
    };

    // Power supply.
    let primary_supply = pedal.supplies.first().map(|s| &s.config);

    let power_supply = primary_supply.filter(|s| s.has_sag()).map(|s| {
        crate::elements::PowerSupply::new(
            s.voltage,
            s.impedance.unwrap_or(0.0),
            s.filter_cap.unwrap_or(100e-6),
            s.rectifier,
            sample_rate,
        )
    });

    let base_grid_bias = push_pull_stages.first().map_or(-2.0, |pp| pp.grid_bias);

    // Build pot smoothers.
    let pot_smoothers: Vec<SmoothedParam> = controls
        .iter()
        .enumerate()
        .filter_map(|(i, ctrl)| {
            if matches!(
                ctrl.target,
                ControlTarget::PotInStage(_)
                    | ControlTarget::PotInMultiNlStage(_, _)
            ) {
                Some(SmoothedParam::new(0.5, i, sample_rate))
            } else {
                None
            }
        })
        .collect();

    // ══ Topological stage ordering ═════════════════════════════════════
    // Build a unified stage execution order sorted by signal_flow_distance.
    // This ensures stages process in signal-flow order regardless of type.
    let stage_order = {
        let mut order: Vec<(StageRef, usize)> = Vec::new();
        for (i, s) in stages.iter().enumerate() {
            order.push((StageRef::Wdf(i), s.signal_flow_distance));
        }
        for (i, s) in multi_nl_stages.iter().enumerate() {
            order.push((StageRef::MultiNl(i), s.signal_flow_distance));
        }
        order.sort_by_key(|(_, dist)| *dist);
        order.into_iter().map(|(sr, _)| sr).collect::<Vec<_>>()
    };

    // ══ Assembly ══════════════════════════════════════════════════════
    let mut compiled = CompiledPedal {
        stages,
        push_pull_stages,
        multi_nl_stages,
        pre_gain,
        output_gain: 1.0,
        rail_saturation,
        sample_rate,
        controls,
        gain_range: gain_range_final,
        supply_voltage: 9.0,
        lfos,
        envelopes,
        slew_limiters,
        bbds,
        delay_lines,
        vcos,
        vcas,
        thermal,
        tolerance_seed: tolerance.seed(),
        oversampling,
        opamp_stages,
        power_supply,
        #[cfg(debug_assertions)]
        debug_stats: None,
        metrics_accumulator: None,
        metrics_buffer: None,
        input_loading: None,
        output_loading: None,
        sidechains,
        pot_smoothers,
        pot_mirrors: {
            // Build reverse mapping: source_id → [mirrored_ids]
            let mut m: std::collections::HashMap<String, Vec<super::compiled::MirrorPot>> =
                std::collections::HashMap::new();
            for (mirrored, source) in &pedal.mirrors {
                m.entry(source.clone()).or_default().push(super::compiled::MirrorPot {
                    id: mirrored.clone(),
                    id_aw: format!("{}__aw", mirrored),
                    id_wb: format!("{}__wb", mirrored),
                });
            }
            m
        },
        base_grid_bias,
        multi_nl_recompute_counter: 0,
        stage_order,
        node_signals: Vec::new(),
        bbd_wet_mix: 0.5,
        bbd_mix_pot_id,
        triggers,
        midi_trigger_map,
    };

    let initial_voltage = match &compiled.power_supply {
        Some(psu) => psu.steady_state_voltage(),
        None => supply_voltage,
    };
    compiled.set_supply_voltage(initial_voltage);

    // Auto-calibrate output level if requested by pedal definition
    if pedal.calibrate {
        let output_gain = calibrate_output_gain(&mut compiled);
        compiled.output_gain = output_gain;
        compiled.reset();
    }

    Ok(compiled)
}

/// Process a reference signal through the pedal and compute a gain scalar
/// that normalizes output RMS to match input RMS.
fn calibrate_output_gain(pedal: &mut CompiledPedal) -> f64 {
    use std::f64::consts::PI;

    let sr = pedal.sample_rate;
    let n = (0.5 * sr) as usize; // 500ms
    let two_pi_f = 2.0 * PI * 1000.0; // 1kHz
    let amp = 0.25; // -12dBFS

    // Warm up (128 samples — let caps settle)
    for i in 0..128usize {
        let _ = pedal.process(amp * (two_pi_f * i as f64 / sr).sin());
    }

    // Measure steady-state output RMS
    let mut sum_sq = 0.0;
    let measure_len = n - 128;
    for i in 128..n {
        let input = amp * (two_pi_f * i as f64 / sr).sin();
        let out = pedal.process(input);
        sum_sq += out * out;
    }
    let output_rms = (sum_sq / measure_len as f64).sqrt();
    let input_rms = amp / std::f64::consts::SQRT_2;

    if output_rms < 1e-10 {
        return 1.0; // Silent — don't amplify noise
    }

    (input_rms / output_rms).clamp(0.01, 100.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::parse_pedal_file;

    #[test]
    fn calibrate_output_gain_reasonable() {
        // A simple resistor divider: output should be ~half input.
        // calibrate should produce a gain ~2.0 to compensate.
        let src = r#"
            pedal "Divider" {
                components {
                    R1: resistor(10k)
                    R2: resistor(10k)
                }
                nets {
                    in -> R1.a
                    R1.b -> R2.a
                    R2.b -> gnd
                    R1.b -> out
                }
                calibrate
            }
        "#;
        let pedal = parse_pedal_file(src).unwrap();
        assert!(pedal.calibrate);
        let compiled = compile_pedal(&pedal, 48000.0).unwrap();
        // Output gain should compensate for the ~6dB loss
        assert!(
            compiled.output_gain > 1.0,
            "expected output_gain > 1.0, got {}",
            compiled.output_gain
        );
        assert!(
            compiled.output_gain < 10.0,
            "expected output_gain < 10.0, got {}",
            compiled.output_gain
        );
    }

    #[test]
    fn no_calibrate_gives_unity_gain() {
        let src = r#"
            pedal "Pass" {
                components {
                    R1: resistor(10k)
                }
                nets {
                    in -> R1.a
                    R1.b -> out
                }
            }
        "#;
        let pedal = parse_pedal_file(src).unwrap();
        assert!(!pedal.calibrate);
        let compiled = compile_pedal(&pedal, 48000.0).unwrap();
        assert!(
            (compiled.output_gain - 1.0).abs() < 1e-10,
            "expected output_gain = 1.0, got {}",
            compiled.output_gain
        );
    }
}
