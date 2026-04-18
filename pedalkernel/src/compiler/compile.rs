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
use crate::elements::{BbdDelayLine, BbdModel, SlewRateLimiter, TriodeModel};
use crate::oversampling::{Oversampler, OversamplingFactor};
use crate::thermal::ThermalModel;
use crate::tolerance::ToleranceEngine;
use crate::PedalProcessor;

use super::compiled::*;
use super::component::Component;
use super::components::{
    Bbd as BbdComp, Capacitor as CapacitorComp, DelayLineComp, Inductor as InductorComp,
    Lfo as LfoComp, Pentode as PentodeComp, Potentiometer as PotComp, Resistor as ResistorComp,
    Tap as TapComp, Triode as TriodeComp, VariMu as VariMuComp, Vca as VcaComp, Vco as VcoComp,
};
use super::dyn_node::DynNode;
use super::graph::*;
use super::helpers::*;
use super::stage::{RootKind, WdfStage};
use crate::tree::ScatteringInterpolationTable;

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
        .filter(|c| c.kind.op_amp_type().is_some_and(|ot| ot.is_ota()))
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
        for e in graph.edges.iter() {
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
    let _vs_comp_idx = graph.components.len();
    let passive_edges: Vec<usize> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(_, e)| graph.components[e.comp_idx].kind.is_simple_passive())
        .map(|(i, _)| i)
        .collect();

    if passive_edges.is_empty() {
        return None;
    }

    // Try output-rooted decomposition for simple 2-element circuits.
    if let Some(stage) = build_output_rooted_stage(graph, &passive_edges, sample_rate, oversampling)
    {
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
    let extra = vec![super::graph::ExtraEdge {
        node_a: source_node,
        node_b: graph.in_node,
        tree: DynNode::VoltageSource(0.0, 1.0),
    }];
    let terminals = vec![source_node, graph.gnd_node];
    match super::graph::graph_reduce(
        &passive_edges,
        &extra,
        &terminals,
        graph,
        sample_rate,
        &HashMap::new(),
        |n| n,
        None,
    ) {
        Ok((tree, _)) => {
            let mut stage =
                WdfStage::new(tree, RootKind::ShortCircuit, Oversampler::new(oversampling));
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
    let decomposed = super::graph::sp_decompose(passive_edges, &junction_nodes, graph, sample_rate);

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
    let decomposed = super::graph::sp_decompose(passive_edges, &junction_nodes, graph, sample_rate);

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
        match comp
            .kind
            .stamp_mna(&comp.id, n1, n2, &mut mna, effective_rate)
        {
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
        let rp = subtree.tree.port_resistance();
        let pos = to_mna(subtree.attachment_node);
        let neg = to_mna(subtree.far_node);
        let dyn_tree = subtree.tree.clone();
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
    reactive_children.push(DynNode::Resistor(None, probe_resistance));
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
    let dummy_tree = DynNode::Resistor(None, 1000.0);
    let has_pots = !pot_stamps.is_empty();
    let recompute_mna = if has_pots { Some(mna.clone()) } else { None };
    let recompute_ports = if has_pots { Some(ports.clone()) } else { None };

    // Build interpolation table for single-pot stages
    let interp_table = if pot_stamps.len() == 1 {
        // Find the pot's max_resistance from the DynNode
        let pot_child_idx = pot_stamps[0].0;
        let max_r = match &reactive_children[pot_child_idx] {
            DynNode::Leaf(leaf) => leaf.pot_max_resistance().unwrap_or(0.0),
            _ => 0.0,
        };
        if max_r > 1.0 {
            let (_, n1, n2, initial_g) = pot_stamps[0];
            Some(ScatteringInterpolationTable::build(
                &mna,
                n1,
                n2,
                initial_g,
                1.0,   // r_min: minimum pot resistance (clamped at 1Ω)
                max_r, // r_max: full pot resistance
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

    Some(WdfStage::new(
        dummy_tree,
        super::stage::RootKind::PassiveRType {
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
        Oversampler::new(OversamplingFactor::X1),
    ))
}

/// Diagnostic-only check for orphan output pots.
///
/// Output pots are now handled by `OutputPot` on the WDF stage (post-WDF
/// gain), so this function no longer builds rescue stages. It only logs
/// warnings to help catch edge-collection gaps.
fn rescue_orphan_output_pots(
    graph: &CircuitGraph,
    _classified: &super::classify::ClassifiedCircuit,
    stages: &[WdfStage],
    multi_nl_stages: &[super::stage::MultiNlStage],
    _sample_rate: f64,
    _oversampling: OversamplingFactor,
) -> (Vec<WdfStage>, Vec<usize>) {
    use super::stage::RootKind;

    let all_pot_ids: Vec<String> = graph
        .components
        .iter()
        .filter(|c| c.kind.is_pot())
        .map(|c| c.id.clone())
        .collect();
    if all_pot_ids.is_empty() {
        return (Vec::new(), Vec::new());
    }

    let orphan_pot_ids: Vec<&str> = all_pot_ids
        .iter()
        .filter(|id| {
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
            for mnl in multi_nl_stages {
                for child in &mnl.pot_children {
                    if has_pot(child, id) {
                        return false;
                    }
                }
                for child in &mnl.passive_children {
                    if has_pot(child, id) {
                        return false;
                    }
                }
            }
            true
        })
        .map(|s| s.as_str())
        .collect();

    if !orphan_pot_ids.is_empty() {
        tracing::warn!(
            count = orphan_pot_ids.len(),
            pots = ?orphan_pot_ids,
            "unbound pot(s) — declared but not mapped in controls block",
        );
    }

    (Vec::new(), Vec::new())
}

// ═══════════════════════════════════════════════════════════════════════════
// Op-amp fast path: SP-decomposable Rf/Ri feedback → OpAmpWdfAdaptor
// ═══════════════════════════════════════════════════════════════════════════

/// Result from the op-amp fast path builder.
struct OpAmpFastPathResult {
    stage: WdfStage,
    /// Edge indices consumed by this stage (must be claimed).
    claimed: Vec<usize>,
}

/// Try to build a cheap `OpAmpWdfAdaptor`-based `WdfStage` for a simple
/// inverting op-amp feedback network.
///
/// Returns `Some` when:
/// - The op-amp is inverting (V+ grounded or AC-grounded)
/// - The Zf feedback network (neg↔out) is SP-reducible via `graph_reduce`
/// - The Zi input network (injection↔neg) is SP-reducible via `graph_reduce`
///
/// Returns `None` when the topology is too complex for the fast path
/// (non-SP feedback, reactive-dominant, or non-inverting). The caller
/// falls back to the MNA nullor path which handles complex/rigid topologies.
///
/// # Performance
///
/// `OpAmpWdfAdaptor::process()` costs ~20 ops/sample (2 up-sweeps + 3 mults +
/// GBW/slew). The MNA nullor path costs O(N²) for N ports (~100× slower for
/// simple Rf/Ri circuits that end up with 10+ MNA ports).
fn try_build_opamp_fast_path(
    rec: &super::graph::NullorPinRecord,
    graph: &CircuitGraph,
    claimed_edges: &HashSet<usize>,
    sample_rate: f64,
    oversampling: OversamplingFactor,
    supply_voltage: f64,
) -> Option<OpAmpFastPathResult> {
    use super::graph::{graph_reduce, ExtraEdge, NodeId};
    use super::stage::OpAmpWdfAdaptor;
    use crate::elements::nonlinear::{FeedbackConfig, OpAmpModel, OpAmpRoot};

    let neg_node = rec.neg_node;
    let out_node = rec.out_node;
    let pos_node = rec.pos_node;

    // Unity-gain buffers (neg == out) are not for this path.
    if neg_node == out_node {
        return None;
    }

    // ── Detect topology ─────────────────────────────────────────────────
    // Inverting: V+ is grounded (== gnd_node or an AC-ground node)
    let is_inverting = pos_node == graph.gnd_node || graph.ac_ground_nodes.contains(&pos_node);

    // For now only implement the inverting fast path.
    // Non-inverting requires a different Zi (neg→gnd) split.
    if !is_inverting {
        return None;
    }

    let injection_node = graph.in_node; // main signal injection

    // ── Collect Zf edges (neg ↔ out sub-network, feedback path) ─────────
    //
    // BFS from neg_node. Stop expansion at boundary nodes:
    // out_node, injection_node, gnd_node, graph.out_node.
    // This ensures load resistors and output-side elements are NOT included.
    let zf_edges: Vec<usize> = {
        let zf_stop_nodes: HashSet<NodeId> =
            [out_node, injection_node, graph.gnd_node, graph.out_node]
                .iter()
                .copied()
                .collect();

        let mut zf_nodes: HashSet<NodeId> = [neg_node, out_node].iter().copied().collect();
        let mut frontier = vec![neg_node];
        let mut seen: HashSet<usize> = HashSet::new();
        let mut found: Vec<usize> = Vec::new();
        while let Some(n) = frontier.pop() {
            for (eidx, e) in graph.edges.iter().enumerate() {
                if seen.contains(&eidx) || claimed_edges.contains(&eidx) {
                    continue;
                }
                if graph.active_edge_indices.contains(&eidx) {
                    continue;
                }
                let comp = &graph.components[e.comp_idx];
                if !comp.kind.is_passive() {
                    continue;
                }
                if graph.supply_nodes.contains(&e.node_a) || graph.supply_nodes.contains(&e.node_b)
                {
                    continue;
                }
                let other = if e.node_a == n {
                    e.node_b
                } else if e.node_b == n {
                    e.node_a
                } else {
                    continue;
                };
                seen.insert(eidx);
                found.push(eidx);
                // Expand only through non-boundary interior nodes.
                if zf_nodes.insert(other) && !zf_stop_nodes.contains(&other) {
                    frontier.push(other);
                }
            }
        }
        found
    };

    if zf_edges.is_empty() {
        return None;
    }

    // Try SP reduction of the Zf sub-network with terminals [neg_node, out_node].
    let zf_terminals = vec![neg_node, out_node];
    let zf_tree = graph_reduce(
        &zf_edges,
        &[],
        &zf_terminals,
        graph,
        sample_rate,
        &HashMap::new(),
        |n| n,
        None,
    )
    .ok()
    .map(|(t, _)| t)?;

    let zf_r = zf_tree.port_resistance();
    if zf_r <= 0.0 || !zf_r.is_finite() {
        return None;
    }

    // ── Collect Zi edges (injection → neg sub-network, input path) ──────
    //
    // BFS from injection_node, stopping at neg_node.
    // Exclude Zf edges to avoid cross-contamination.
    let zi_edges: Vec<usize> = {
        let zf_set: HashSet<usize> = zf_edges.iter().copied().collect();
        let zi_stop_nodes: HashSet<NodeId> = [neg_node, out_node, graph.gnd_node, graph.out_node]
            .iter()
            .copied()
            .collect();

        let mut zi_nodes: HashSet<NodeId> = [injection_node, neg_node].iter().copied().collect();
        let mut frontier = vec![injection_node];
        let mut seen: HashSet<usize> = HashSet::new();
        let mut found: Vec<usize> = Vec::new();
        while let Some(n) = frontier.pop() {
            for (eidx, e) in graph.edges.iter().enumerate() {
                if seen.contains(&eidx) || claimed_edges.contains(&eidx) || zf_set.contains(&eidx) {
                    continue;
                }
                if graph.active_edge_indices.contains(&eidx) {
                    continue;
                }
                let comp = &graph.components[e.comp_idx];
                if !comp.kind.is_passive() {
                    continue;
                }
                if graph.supply_nodes.contains(&e.node_a) || graph.supply_nodes.contains(&e.node_b)
                {
                    continue;
                }
                let other = if e.node_a == n {
                    e.node_b
                } else if e.node_b == n {
                    e.node_a
                } else {
                    continue;
                };
                seen.insert(eidx);
                found.push(eidx);
                if zi_nodes.insert(other) && !zi_stop_nodes.contains(&other) {
                    frontier.push(other);
                }
            }
        }
        found
    };

    if zi_edges.is_empty() {
        return None;
    }

    // Try SP reduction of the Zi sub-network.
    // Use a synthetic source node so graph_reduce can anchor the VS.
    let vs_syn_node = graph.edges.len() + 8000 + rec.comp_idx;
    let zi_extra = vec![ExtraEdge {
        node_a: vs_syn_node,
        node_b: injection_node,
        tree: DynNode::VoltageSource(0.0, 1.0),
    }];
    let zi_terminals_ext = vec![vs_syn_node, neg_node];

    let zi_tree = graph_reduce(
        &zi_edges,
        &zi_extra,
        &zi_terminals_ext,
        graph,
        sample_rate,
        &HashMap::new(),
        |n| n,
        None,
    )
    .ok()
    .map(|(t, _)| t)?;

    let zi_r = zi_tree.port_resistance();
    if zi_r <= 0.0 || !zi_r.is_finite() {
        return None;
    }

    // ── DC gain and feedback pot detection ───────────────────────────────
    let dc_gain = (zf_r / zi_r).max(0.01);

    let zf_pot_id: Option<String> = zf_edges.iter().find_map(|&eidx| {
        let comp = &graph.components[graph.edges[eidx].comp_idx];
        if comp.kind.is_pot() {
            Some(comp.id.clone())
        } else {
            None
        }
    });
    let zi_pot_id: Option<String> = zi_edges.iter().find_map(|&eidx| {
        let comp = &graph.components[graph.edges[eidx].comp_idx];
        if comp.kind.is_pot() {
            Some(comp.id.clone())
        } else {
            None
        }
    });

    let feedback_config: Option<FeedbackConfig> = if let Some(ref pot_id) = zf_pot_id {
        let fixed_series_r = zf_edges
            .iter()
            .filter_map(|&eidx| {
                let comp = &graph.components[graph.edges[eidx].comp_idx];
                if !comp.kind.is_pot() {
                    comp.kind.resistance()
                } else {
                    None
                }
            })
            .sum::<f64>();
        Some(FeedbackConfig {
            pot_comp_id: pot_id.clone(),
            other_leg_r: zi_r,
            fixed_series_r,
            parallel_r: None,
            pot_is_feedback: true,
            is_inverting: true,
        })
    } else if let Some(ref pot_id) = zi_pot_id {
        let fixed_series_r = zi_edges
            .iter()
            .filter_map(|&eidx| {
                let comp = &graph.components[graph.edges[eidx].comp_idx];
                if !comp.kind.is_pot() {
                    comp.kind.resistance()
                } else {
                    None
                }
            })
            .sum::<f64>();
        Some(FeedbackConfig {
            pot_comp_id: pot_id.clone(),
            other_leg_r: zf_r,
            fixed_series_r,
            parallel_r: None,
            pot_is_feedback: false,
            is_inverting: true,
        })
    } else {
        None
    };

    // ── Build OpAmpRoot ───────────────────────────────────────────────────
    let opamp_comp = &graph.components[rec.comp_idx];
    let opamp_type = opamp_comp
        .kind
        .op_amp_type()
        .unwrap_or(crate::dsl::OpAmpType::Generic);
    let model = OpAmpModel::from_opamp_type(&opamp_type);

    let mut opamp = OpAmpRoot::new_inverting(model, dc_gain);
    opamp.set_sample_rate(sample_rate);
    let v_max = (supply_voltage / 2.0 - 1.5).max(0.5);
    opamp.set_v_max(v_max);
    opamp.set_gbw_gain(dc_gain);
    if let Some(cfg) = feedback_config {
        opamp.set_feedback_config(cfg);
    }

    // ── Build WdfStage with OpAmpWdfAdaptor ───────────────────────────────
    let feedback_pot_id = zf_pot_id.or(zi_pot_id);

    let adaptor = OpAmpWdfAdaptor {
        zi: zi_tree,
        zf: zf_tree,
        is_inverting: true,
        opamp,
        gbw_state: 0.0,
        prev_out: 0.0,
        feedback_pot_id: feedback_pot_id.clone(),
    };

    // Dummy tree — not used when opamp_wdf_adaptor is Some.
    let dummy_tree = DynNode::VoltageSource(0.0, 1.0);

    let mut stage = WdfStage::new(
        dummy_tree,
        super::stage::RootKind::OpAmp(OpAmpRoot::new_inverting(
            OpAmpModel::from_opamp_type(&opamp_type),
            dc_gain,
        )),
        Oversampler::new(oversampling),
    );
    stage.opamp_wdf_adaptor = Some(adaptor);
    stage.feedback_pot_id = feedback_pot_id;
    stage.injection_node_id = injection_node;
    stage.output_node_id = out_node;
    stage.signal_flow_distance = 0;
    stage.root_comp_id = opamp_comp.id.clone();

    let mut all_claimed: Vec<usize> = zf_edges.clone();
    all_claimed.extend_from_slice(&zi_edges);

    Some(OpAmpFastPathResult {
        stage,
        claimed: all_claimed,
    })
}

/// Build feedforward passive stages for orphaned subgraphs that bridge
/// two active nodes (e.g., clean blend paths in the Klon Centaur).
fn build_feedforward_stages(
    graph: &CircuitGraph,
    classified: &super::classify::ClassifiedCircuit,
    claimed_edges: &HashSet<usize>,
    sample_rate: f64,
) -> Vec<WdfStage> {
    // ── Step 1: Collect unclaimed simple-passive edges ────────────────
    let nl_edge_set: HashSet<usize> = classified
        .all_nonlinear_edge_indices
        .iter()
        .copied()
        .collect();
    let active_edge_set: HashSet<usize> = graph.active_edge_indices.iter().copied().collect();

    let unclaimed: Vec<usize> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(eidx, e)| {
            let comp = &graph.components[e.comp_idx];
            if claimed_edges.contains(eidx) {
                return false;
            }
            if nl_edge_set.contains(eidx) || active_edge_set.contains(eidx) {
                return false;
            }
            let comp = &graph.components[e.comp_idx];
            if !comp.kind.is_simple_passive() {
                return false;
            }
            // Skip supply-adjacent edges (except gnd — gnd is valid termination)
            if graph.supply_nodes.contains(&e.node_a) && e.node_a != graph.gnd_node {
                return false;
            }
            if graph.supply_nodes.contains(&e.node_b) && e.node_b != graph.gnd_node {
                return false;
            }
            // Skip sidechain edges
            if classified.sidechain_edge_set.contains(eidx) {
                return false;
            }
            true
        })
        .map(|(eidx, _)| eidx)
        .collect();

    if unclaimed.is_empty() {
        return Vec::new();
    }

    // ── Step 2: Build connected components via union-find ─────────────
    let mut parent: HashMap<NodeId, NodeId> = HashMap::new();
    fn find(parent: &mut HashMap<NodeId, NodeId>, x: NodeId) -> NodeId {
        let p = *parent.get(&x).unwrap_or(&x);
        if p == x {
            return x;
        }
        let root = find(parent, p);
        parent.insert(x, root);
        root
    }
    fn union(parent: &mut HashMap<NodeId, NodeId>, a: NodeId, b: NodeId) {
        let ra = find(parent, a);
        let rb = find(parent, b);
        if ra != rb {
            parent.insert(ra, rb);
        }
    }

    // Global reference nodes: GND, VCC, and any named supply rails.
    // Edges that have one terminal at a global node are shunt-to-rail components
    // (e.g. R_out1 to GND, R_bias2 to GND). We must NOT union across these
    // global nodes or every passive sub-network in the circuit gets merged
    // into one giant component (output chain ↔ gain-stage ground leg, etc.).
    let global_junction_nodes: HashSet<NodeId> = {
        let mut g = graph.supply_nodes.clone();
        g.insert(graph.gnd_node);
        g.insert(graph.vcc_node);
        g
    };

    for &eidx in &unclaimed {
        let e = &graph.edges[eidx];
        // Only union when NEITHER endpoint is a global node.
        let a_is_global = global_junction_nodes.contains(&e.node_a);
        let b_is_global = global_junction_nodes.contains(&e.node_b);
        if !a_is_global && !b_is_global {
            union(&mut parent, e.node_a, e.node_b);
        }
    }

    // Group edges by their component root
    let mut components: HashMap<NodeId, Vec<usize>> = HashMap::new();
    for &eidx in &unclaimed {
        let e = &graph.edges[eidx];
        // Assign each edge to the component root of its non-global node.
        // For shunt-to-gnd/vcc edges, use the non-global side's root so the
        // edge appears in the correct sub-network (not in a global-only group).
        let root_node = if global_junction_nodes.contains(&e.node_a) {
            e.node_b
        } else {
            e.node_a
        };
        let root = find(&mut parent, root_node);
        components.entry(root).or_default().push(eidx);
    }

    // ── Step 3: Identify boundary nodes for each component ───────────
    // Boundary nodes are nodes that also touch a claimed edge, or are
    // NL junction / active / opamp nodes.
    let mut boundary_nodes_set: HashSet<NodeId> = HashSet::new();
    // Nodes on claimed edges
    for &eidx in claimed_edges {
        if eidx < graph.edges.len() {
            let e = &graph.edges[eidx];
            boundary_nodes_set.insert(e.node_a);
            boundary_nodes_set.insert(e.node_b);
        }
    }
    // NL junction nodes
    for &eidx in &classified.all_nonlinear_edge_indices {
        let e = &graph.edges[eidx];
        boundary_nodes_set.insert(e.node_a);
        boundary_nodes_set.insert(e.node_b);
    }
    // Active bridge nodes
    for &eidx in &graph.active_edge_indices {
        let e = &graph.edges[eidx];
        boundary_nodes_set.insert(e.node_a);
        boundary_nodes_set.insert(e.node_b);
    }
    // Global nodes
    boundary_nodes_set.insert(graph.in_node);
    boundary_nodes_set.insert(graph.out_node);
    boundary_nodes_set.insert(graph.gnd_node);
    boundary_nodes_set.insert(graph.vcc_node);

    let global_nodes: HashSet<NodeId> = [
        graph.in_node,
        graph.out_node,
        graph.gnd_node,
        graph.vcc_node,
    ]
    .iter()
    .copied()
    .collect();

    let mut result = Vec::new();

    for edges in components.values() {
        #[cfg(test)]
        {
            let edge_names: Vec<String> = edges
                .iter()
                .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.clone())
                .collect();
            eprintln!("[ff-stages] component edges={:?}", edge_names);
        }
        // Collect all nodes in this subgraph
        let mut subgraph_nodes: HashSet<NodeId> = HashSet::new();
        for &eidx in edges {
            let e = &graph.edges[eidx];
            subgraph_nodes.insert(e.node_a);
            subgraph_nodes.insert(e.node_b);
        }

        // Find boundary nodes for this subgraph
        let boundary: Vec<NodeId> = subgraph_nodes
            .iter()
            .copied()
            .filter(|n| boundary_nodes_set.contains(n))
            .collect();

        // Skip subgraphs with fewer than 2 boundary nodes (dangling/bias)
        if boundary.len() < 2 {
            continue;
        }
        // Skip subgraphs where ALL boundary nodes are global (bias networks)
        if boundary.iter().all(|n| global_nodes.contains(n)) {
            continue;
        }

        // ── Step 4: Find injection and output nodes ──────────────────
        // Prefer active stage output pins (output_pin_nodes) as injection —
        // these are where signal enters the orphan passive subgraph from an
        // upstream active stage. Prefer graph.out_node as output.
        // Fallback: dist_from_in heuristic (smallest = injection, largest = output).

        // Find best injection candidate: active output pin with largest
        // dist_from_in (deepest in signal chain = latest active stage before
        // this passive section).
        let mut best_active_inj: Option<(NodeId, usize)> = None;
        for &node in &boundary {
            if graph.output_pin_nodes.contains(&node) {
                let d = classified.dist_from_in.get(&node).copied().unwrap_or(0);
                match best_active_inj {
                    None => best_active_inj = Some((node, d)),
                    Some((_, prev_d)) if d > prev_d => best_active_inj = Some((node, d)),
                    _ => {}
                }
            }
        }

        let (injection_node, output_node);
        if let Some((inj_node, _)) = best_active_inj {
            injection_node = inj_node;
            // Output: prefer graph.out_node, else farthest-from-input
            // non-injection boundary node
            if boundary.contains(&graph.out_node) && graph.out_node != inj_node {
                output_node = graph.out_node;
            } else {
                let mut best: Option<(NodeId, usize)> = None;
                for &node in &boundary {
                    if node == inj_node {
                        continue;
                    }
                    let d = classified
                        .dist_from_in
                        .get(&node)
                        .copied()
                        .unwrap_or(usize::MAX);
                    match best {
                        None => best = Some((node, d)),
                        Some((_, pd)) if d > pd => best = Some((node, d)),
                        Some((_, pd)) if d == pd && node == graph.out_node => {
                            best = Some((node, d))
                        }
                        _ => {}
                    }
                }
                match best {
                    Some((n, _)) => output_node = n,
                    None => continue,
                }
            }
        } else {
            // No active output pin in boundary — fall back to dist_from_in.
            // Exclude GND/VCC/in from injection candidates — they are supply/reference
            // nodes, not signal injection points. GND in particular has dist=0 and
            // would always win the "smallest distance" heuristic.
            let signal_boundary: Vec<NodeId> = boundary
                .iter()
                .copied()
                .filter(|n| *n != graph.gnd_node && *n != graph.vcc_node && *n != graph.in_node)
                .collect();
            if signal_boundary.len() < 2 {
                // Not enough non-global boundary nodes to form injection→output
                continue;
            }

            // If out_node is in boundary, it's always the output (signal flows
            // toward circuit output). The injection is the other non-global node
            // with smallest dist_from_in.
            if signal_boundary.contains(&graph.out_node) {
                output_node = graph.out_node;
                let candidates: Vec<NodeId> = signal_boundary
                    .iter()
                    .copied()
                    .filter(|n| *n != graph.out_node)
                    .collect();
                if candidates.is_empty() {
                    continue;
                }
                injection_node = *candidates
                    .iter()
                    .min_by_key(|n| {
                        classified
                            .dist_from_in
                            .get(n)
                            .copied()
                            .unwrap_or(usize::MAX)
                    })
                    .unwrap();
            } else {
                let mut inj = signal_boundary[0];
                let mut inj_d = classified
                    .dist_from_in
                    .get(&signal_boundary[0])
                    .copied()
                    .unwrap_or(usize::MAX);
                let mut out = signal_boundary[0];
                let mut out_d = inj_d;
                for &node in &signal_boundary[1..] {
                    let d = classified
                        .dist_from_in
                        .get(&node)
                        .copied()
                        .unwrap_or(usize::MAX);
                    if d < inj_d {
                        inj_d = d;
                        inj = node;
                    }
                    if d > out_d {
                        out_d = d;
                        out = node;
                    }
                }
                injection_node = inj;
                output_node = out;
            }
        }

        let injection_dist = classified
            .dist_from_in
            .get(&injection_node)
            .copied()
            .unwrap_or(1);

        // injection and output must differ
        if injection_node == output_node {
            continue;
        }

        // ── Step 5: Try SP reduction first, fall back to PassiveRType ───
        // Use VoltageSourceDriver root: the VS drives the injection node from
        // outside the tree, and the tree is just the passive network from
        // injection_node to gnd. No embedded VS (avoids impedance balancing
        // artefacts and gives the exact voltage divider).
        let terminals = vec![injection_node, graph.gnd_node];
        let remap = |n: NodeId| -> NodeId {
            if graph.supply_nodes.contains(&n) {
                graph.gnd_node
            } else {
                n
            }
        };

        let mut stage = match super::graph::graph_reduce(
            edges,
            &[],
            &terminals,
            graph,
            sample_rate,
            &HashMap::new(),
            remap,
            Some(output_node),
        ) {
            Ok((tree, output_probe)) => WdfStage {
                injection_node_id: injection_node,
                output_node_id: output_node,
                is_feedforward: true,
                output_probe,
                ..WdfStage::new(
                    tree,
                    RootKind::VoltageSourceDriver,
                    Oversampler::new(OversamplingFactor::X1),
                )
            },
            Err(_) => {
                // SP reduction failed → fall back to PassiveRType MNA
                match build_orphan_output_mna_stage(
                    graph,
                    edges,
                    injection_node,
                    output_node,
                    1e9,
                    sample_rate,
                ) {
                    Some(s) => s,
                    None => continue,
                }
            }
        };
        // If the stage terminates at the circuit output, it should be in the
        // serial chain (not feedforward) so pots like Volume/Output actually
        // control the signal level.
        stage.is_feedforward = output_node != graph.out_node;
        stage.injection_node_id = injection_node;
        stage.output_node_id = output_node;
        stage.signal_flow_distance = injection_dist.saturating_add(1);
        result.push(stage);
    }

    result
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
        let has_resistor = source_comp
            .kind
            .as_any()
            .downcast_ref::<ResistorComp>()
            .is_some()
            || load_comp
                .kind
                .as_any()
                .downcast_ref::<ResistorComp>()
                .is_some();
        if !is_r_c_or_l(source_comp.kind.as_ref())
            || !is_r_c_or_l(load_comp.kind.as_ref())
            || !has_resistor
        {
            return None;
        }
    }

    // Build WDF tree: Series(source, load).
    // Source element (in→out) on left, load element (out→gnd) on right.
    // Output is extracted at right child (load port) via series_junction_voltage.
    let source_dyn = make_leaf(source_edge.comp_idx, source_comp, None, sample_rate);
    let load_dyn = make_leaf(load_edge.comp_idx, load_comp, None, sample_rate);
    let tree = DynNode::Series(Box::new(source_dyn), Box::new(load_dyn));

    // Linear passive stage — no nonlinearity means no aliasing,
    // so X1 avoids double-counting the oversampling the runner already applied.
    Some(WdfStage::new(
        tree,
        RootKind::VoltageSourceDriver,
        Oversampler::new(OversamplingFactor::X1),
    ))
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
fn build_vco_bindings(pedal: &PedalDef, graph: &CircuitGraph, sample_rate: f64) -> Vec<VcoBinding> {
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
                            if let Pin::ComponentPin {
                                component: target_comp,
                                pin: target_pin,
                            } = to_pin
                            {
                                if let Some(node) =
                                    resolve_pin_to_node(target_comp, target_pin, pedal, graph)
                                {
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
                                    if let Pin::ComponentPin {
                                        component: src_comp,
                                        pin: src_pin,
                                    } = &net.from
                                    {
                                        if let Some(node) =
                                            resolve_pin_to_node(src_comp, src_pin, pedal, graph)
                                        {
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
            envelope.set_attack(0.001); // 1ms
            envelope.set_decay(0.15); // 150ms
            envelope.set_sustain(0.0); // no sustain — percussive
            envelope.set_release(0.05); // 50ms

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
                                    if let Pin::ComponentPin {
                                        component: target_comp,
                                        pin: target_pin,
                                    } = to_pin
                                    {
                                        if let Some(node) = resolve_pin_to_node(
                                            target_comp,
                                            target_pin,
                                            pedal,
                                            graph,
                                        ) {
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
                                    if let Pin::ComponentPin {
                                        component: src_comp,
                                        pin: src_pin,
                                    } = &net.from
                                    {
                                        if let Some(node) =
                                            resolve_pin_to_node(src_comp, src_pin, pedal, graph)
                                        {
                                            input_node_id = node;
                                        }
                                    }
                                }
                                "out" | "output" => {
                                    if let Pin::ComponentPin {
                                        component: src_comp,
                                        pin: src_pin,
                                    } = &net.from
                                    {
                                        if let Some(node) =
                                            resolve_pin_to_node(src_comp, src_pin, pedal, graph)
                                        {
                                            output_node_id = node;
                                        }
                                    }
                                }
                                "cv" => {
                                    // Check if CV is connected to a trigger_input
                                    if let Pin::ComponentPin {
                                        component: src_comp,
                                        ..
                                    } = &net.from
                                    {
                                        if let Some(&idx) = trigger_id_to_idx.get(src_comp.as_str())
                                        {
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

// ─────────────────────────────────────────────────────────────────────────────
// Subcircuit equipment compiler
// ─────────────────────────────────────────────────────────────────────────────

/// Compile an equipment definition that uses subcircuit blocks.
///
/// Each subcircuit is compiled independently via `compile_pedal_with_options()`
/// (potentially at a reduced sample rate) and wired together via the routing
/// graph produced by `subcircuit::build_routing()`.
///
/// When all components live inside subcircuits (`pedal.components.is_empty()`),
/// the caller's normal 6-pass pipeline is entirely replaced by this function.
fn compile_subcircuit_equipment(
    pedal: &PedalDef,
    sample_rate: f64,
    _options: &CompileOptions,
) -> Result<CompiledPedal, String> {
    use super::stage::SubcircuitProcessor;
    use super::subcircuit;

    // 1. Resolve subcircuits to compilable sub-PedalDefs
    let resolved = subcircuit::resolve_subcircuits(pedal)
        .map_err(|e| format!("subcircuit resolution: {e}"))?;

    // 2. Compile each subcircuit
    let mut processors: Vec<SubcircuitProcessor> = Vec::with_capacity(resolved.len());
    for r in &resolved {
        let sc_rate = sample_rate / r.rate_divisor as f64;
        // Subcircuits use no oversampling by default to avoid N × M sample rate blowup.
        // Individual sub-circuits can opt into oversampling via their own options later.
        let sc_options = CompileOptions {
            collapse_nl: false,
            oversampling: crate::oversampling::OversamplingFactor::X1,
            ..CompileOptions::default()
        };
        let compiled = compile_pedal_with_options(&r.pedal_def, sc_rate, sc_options)
            .map_err(|e| format!("subcircuit '{}': {e}", r.name))?;
        processors.push(SubcircuitProcessor {
            circuit: compiled,
            name: r.name.clone(),
            rate_divisor: r.rate_divisor,
            rate_counter: r.rate_divisor.max(1),
            held_output: 0.0,
            prev_output: 0.0,
        });
    }

    // 3. Build routing graph
    let (routing, output_idx) = subcircuit::build_routing(pedal, &resolved)
        .map_err(|e| format!("subcircuit routing: {e}"))?;

    let n = processors.len();

    // 4. Assemble routing-only CompiledPedal shell
    Ok(CompiledPedal {
        stages: Vec::new(),
        push_pull_stages: Vec::new(),
        multi_nl_stages: Vec::new(),
        pre_gain: 1.0,
        output_gain: 1.0,
        rail_saturation: super::compiled::RailSaturation::None,
        rail_sat_oversampler: crate::oversampling::Oversampler::new(
            crate::oversampling::OversamplingFactor::X1,
        ),
        sample_rate,
        controls: Vec::new(),
        gain_range: (1.0, 1.0),
        supply_voltage: pedal.supplies.first().map_or(9.0, |s| s.config.voltage),
        lfos: Vec::new(),
        envelopes: Vec::new(),
        slew_limiters: Vec::new(),
        bbds: Vec::new(),
        delay_lines: Vec::new(),
        vcos: Vec::new(),
        vcas: Vec::new(),
        thermal: None,
        tolerance_seed: 0,
        oversampling: crate::oversampling::OversamplingFactor::X1,
        opamp_stages: Vec::new(),
        power_supply: None,
        #[cfg(debug_assertions)]
        debug_stats: None,
        metrics_accumulator: None,
        metrics_buffer: None,
        input_loading: None,
        output_loading: None,
        output_dc_block: None,
        sidechains: Vec::new(),
        subcircuit_processors: processors,
        subcircuit_routing: routing,
        subcircuit_output_idx: Some(output_idx),
        subcircuit_outputs: vec![0.0; n],
        pot_smoothers: Vec::new(),
        pot_mirrors: std::collections::HashMap::new(),
        base_grid_bias: 0.0,
        multi_nl_recompute_counter: 0,
        stage_order: Vec::new(),
        node_signals: Vec::new(),
        bbd_wet_mix: 0.5,
        bbd_mix_pot_id: None,
        triggers: Vec::new(),
        midi_trigger_map: std::collections::HashMap::new(),
        original_passive_values: std::collections::HashMap::new(),
    })
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
    // ══ Subcircuit pre-pass: compile & route subcircuits ═════════════
    // When the equipment has subcircuit blocks, each block is compiled as an
    // independent sub-PedalDef (potentially at a different sample rate) and
    // wired together via the routing graph.  When all components live in
    // subcircuits, the passes below are skipped and a routing-only shell is
    // returned immediately.
    if !pedal.subcircuits.is_empty() {
        return compile_subcircuit_equipment(pedal, sample_rate, &options);
    }

    let oversampling = options.oversampling;
    let tolerance = options.tolerance;
    let enable_thermal = options.thermal;
    let _collapse_nl = options.collapse_nl;

    // ══ Pass 0.5: Validation ═══════════════════════════════════════════
    // Run validation and hard-fail on all Error-severity warnings.
    // Warnings that may have false positives (e.g. no-signal-path) use
    // Severity::Warning instead so they don't block compilation.
    let warnings = super::validate::validate_pedal(pedal);
    for w in &warnings {
        if w.severity == super::validate::Severity::Error {
            return Err(w.message.clone());
        }
    }
    for w in &warnings {
        match w.severity {
            super::validate::Severity::Warning => tracing::warn!("{}", w),
            super::validate::Severity::Info => tracing::info!("{}", w),
            super::validate::Severity::Error => {} // hard-fail errors handled above
        }
    }

    // ══ Pass 0: Graph construction ════════════════════════════════════
    let mut graph = CircuitGraph::from_pedal(pedal);

    // Post-construction: resolve context-dependent edge kinds (JFET Vr, OTA linear).
    super::graph::resolve_components(&mut graph, pedal);

    // Compute AC ground nodes for BFS barriers.
    graph.compute_ac_ground_nodes(pedal);

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

    // ══ Pass 1.5 + 2 removed (nullor refactor) ═════════════════════════
    // Op-amp topology classification and feedback analysis are gone.
    // All op-amps are absorbed into R-type adaptors via VCVS stamps.
    // See Phase 1–6 commits on refactor/opamp-nullor.

    // ── Short unity-gain opamp pos→out in the graph ────────────────────
    // ── Unity-gain buffer pos→out merge ────────────────────────────────
    // Unity-gain buffers (neg==out) are transparent wires. In circuits
    // with NL elements (Klon, TS808), merge pos→out so NL plans see the
    // merged topology and collect output-tail edges through the buffer.
    //
    // In pure-linear circuits (standalone unity buffer), do NOT merge —
    // the synthetic plan picks up both sides and the VCVS stamp provides
    // the current sourcing that a passive wire can't.
    // Only merge unity buffers whose pos is in the SIGNAL path (reachable
    // from graph.in_node via passive edges). Buffers whose pos is on a
    // bias network (vref divider, gate bias) must NOT be merged — they'd
    // collapse bias nodes into the signal output. This distinguishes:
    //   Klon U1/U3 (pos on signal path) → merge ✓
    //   JFET allpass U1 (pos on vref) → don't merge ✗
    let has_nl_elements = !classified.nonlinear_elements.is_empty();
    if has_nl_elements {
        // Quick BFS from in_node through passive edges to find signal-path nodes.
        let active_set: HashSet<usize> = graph.active_edge_indices.iter().copied().collect();
        let mut signal_reachable: HashSet<super::graph::NodeId> = HashSet::new();
        signal_reachable.insert(graph.in_node);
        let mut frontier = vec![graph.in_node];
        while let Some(n) = frontier.pop() {
            for (eidx, e) in graph.edges.iter().enumerate() {
                if active_set.contains(&eidx) {
                    continue;
                }
                if !graph.components[e.comp_idx].kind.is_passive() {
                    continue;
                }
                let other = if e.node_a == n {
                    e.node_b
                } else if e.node_b == n {
                    e.node_a
                } else {
                    continue;
                };
                if signal_reachable.insert(other) {
                    frontier.push(other);
                }
            }
        }
        for rec in &graph.nullor_pins {
            if rec.neg_node == rec.out_node {
                let pos = rec.pos_node;
                let out = rec.out_node;
                if pos == out {
                    continue;
                }
                // Only merge if pos is on the signal path.
                if !signal_reachable.contains(&pos) {
                    continue;
                }
                for edge in &mut graph.edges {
                    if edge.node_a == pos {
                        edge.node_a = out;
                    }
                    if edge.node_b == pos {
                        edge.node_b = out;
                    }
                }
                if graph.output_pin_nodes.remove(&pos) {
                    graph.output_pin_nodes.insert(out);
                }
                if graph.in_node == pos {
                    graph.in_node = out;
                }
                if graph.out_node == pos {
                    graph.out_node = out;
                }
            }
        }
        for rec in &mut graph.nullor_pins {
            if rec.neg_node == rec.out_node && rec.pos_node != rec.out_node {
                // Check if this rec was actually merged (pos changed).
                // Simple: if pos isn't in any edge, it was merged.
                let pos_still_exists = graph
                    .edges
                    .iter()
                    .any(|e| e.node_a == rec.pos_node || e.node_b == rec.pos_node);
                if !pos_still_exists {
                    rec.pos_node = rec.out_node;
                }
            }
        }
    }

    let mut stages: Vec<WdfStage> = Vec::new();
    let mut claimed_edges: HashSet<usize> = HashSet::new();
    // Op-amp stages are now compiled via the nullor/VCVS path in build_rtype_stage.
    let opamp_stages: Vec<super::compiled::OpAmpStage> = Vec::new();

    // ══ Pass 3: Stage planning ════════════════════════════════════════
    // Detect modulation-controlled elements before planning.
    let envelope_controlled_otas = detect_envelope_controlled_otas(pedal);

    // Op-amp BFS barriers: these were populated from opamp_analysis (now deleted).
    // The nullor path handles op-amps directly via stamp_vcvs; BFS barriers
    // are no longer needed for op-amp feedback networks.
    let opamp_input_nodes: HashSet<super::graph::NodeId> = HashSet::new();
    let feedback_diode_neg_map: HashMap<super::graph::NodeId, super::graph::NodeId> =
        HashMap::new();

    let (
        stage_plans,
        push_pull_plans,
        mut multi_nl_plans,
        pp_transformer_edges,
        _bjt_bias_analysis,
        node_island_depths,
    ) = super::plan::plan_stages(
        &classified,
        &graph,
        sample_rate,
        &envelope_controlled_otas,
        &mut claimed_edges,
        &opamp_input_nodes,
        &feedback_diode_neg_map,
    );

    // ══ Synthesize op-amp-only plans (Phase 3c) ═══════════════════════════
    // For every op-amp whose pins are NOT already covered by an existing
    // multi_nl plan, generate a synthetic plan that carries a nullor VCVS
    // stamp. This allows pure-linear op-amp circuits (unity buffer,
    // inverting x10, integrator, …) to flow through `build_rtype_stage`
    // and produce R-type stages via the unified nullor path.
    {
        // Nodes already covered by existing multi_nl plans.
        let mut covered_nodes: HashSet<super::graph::NodeId> = HashSet::new();
        for p in &multi_nl_plans {
            for &(pos, neg) in &p.nl_terminals {
                covered_nodes.insert(pos);
                covered_nodes.insert(neg);
            }
            for &eidx in &p.passive_edge_indices {
                let e = &graph.edges[eidx];
                covered_nodes.insert(e.node_a);
                covered_nodes.insert(e.node_b);
            }
        }
        for rec in &graph.nullor_pins {
            // Skip op-amps whose OUTPUT node is already covered by an
            // existing plan — they'll pick up the VCVS stamp via
            // in_stage_nullors in build_rtype_stage. If the output node
            // is NOT covered, the op-amp needs its own synthetic plan
            // (even if input pins are in another plan, e.g. Klon U3
            // whose pos is in the D1 plan but out connects to R11/Output).
            let out_covered = covered_nodes.contains(&rec.out_node);
            eprintln!(
                "[nullor-check] opamp={} pos={} neg={} out={} out_covered={}",
                graph.components[rec.comp_idx].id,
                rec.pos_node,
                rec.neg_node,
                rec.out_node,
                out_covered
            );
            if out_covered {
                continue;
            }

            // ── FAST PATH: try SP-reduction for simple Rf/Ri topologies ──
            // Attempt OpAmpWdfAdaptor for inverting op-amps where both the
            // Zf (neg↔out) and Zi (injection↔neg) networks are SP-reducible.
            // Falls back to the MNA nullor slow path on failure.
            if let Some(fast) = try_build_opamp_fast_path(
                rec,
                &graph,
                &claimed_edges,
                sample_rate,
                oversampling,
                supply_voltage,
            ) {
                eprintln!(
                    "[opamp-fast-path] opamp={} using OpAmpWdfAdaptor (O(1) scatter)",
                    graph.components[rec.comp_idx].id,
                );
                for &eidx in &fast.claimed {
                    claimed_edges.insert(eidx);
                }
                stages.push(fast.stage);
                continue; // skip MNA nullor plan for this op-amp
            }

            // ── SLOW PATH: MNA nullor (fallback) ──────────────────────────
            // Gather all unclaimed passive edges that touch any of this
            // op-amp's pins transitively (simple BFS through passive edges).
            let mut pin_nodes: HashSet<super::graph::NodeId> =
                [rec.pos_node, rec.neg_node, rec.out_node]
                    .iter()
                    .copied()
                    .collect();
            // BFS seeds from the op-amp's 3 pins ONLY — do NOT add
            // graph.in_node or graph.out_node. Adding global I/O nodes
            // causes greedy BFS to claim edges belonging to other op-amps
            // (e.g., Klon U1 grabbing R11/Output from U3's territory).
            // The in/out connection happens via stage chaining instead.

            // Collect unclaimed passive edges between these nodes and
            // expand the node set by BFS through passive connectivity.
            let mut frontier: Vec<super::graph::NodeId> = pin_nodes.iter().copied().collect();
            let mut passive_edges: Vec<usize> = Vec::new();
            let mut seen_edges: HashSet<usize> = HashSet::new();
            while let Some(n) = frontier.pop() {
                for (eidx, e) in graph.edges.iter().enumerate() {
                    if seen_edges.contains(&eidx) || claimed_edges.contains(&eidx) {
                        continue;
                    }
                    if graph.active_edge_indices.contains(&eidx) {
                        continue; // skip virtual active-bridge edges
                    }
                    if e.node_a != n && e.node_b != n {
                        continue;
                    }
                    // Skip non-passive edges.
                    let comp = &graph.components[e.comp_idx];
                    if !comp.kind.is_passive() {
                        continue;
                    }
                    seen_edges.insert(eidx);
                    passive_edges.push(eidx);
                    // Expand frontier through the other endpoint.
                    let other = if e.node_a == n { e.node_b } else { e.node_a };
                    if pin_nodes.insert(other) {
                        frontier.push(other);
                    }
                }
            }

            // Build a synthetic plan. injection_node = graph.in_node,
            // output_node = graph.out_node (typical for single-stage
            // op-amp pedals).
            if !passive_edges.is_empty() {
                // Mark these edges as claimed so other plan passes don't
                // re-claim them.
                for &eidx in &passive_edges {
                    claimed_edges.insert(eidx);
                }
                eprintln!(
                    "[synthetic-nullor-plan] opamp={} pins=({},{},{}) edges={}",
                    graph.components[rec.comp_idx].id,
                    rec.pos_node,
                    rec.neg_node,
                    rec.out_node,
                    passive_edges.len()
                );
                multi_nl_plans.push(super::plan::MultiNlPlan {
                    nl_element_indices: Vec::new(),
                    output_element_idx: 0, // unused when n_nl == 0
                    passive_edge_indices: passive_edges,
                    injection_node: graph.in_node,
                    nl_terminals: Vec::new(),
                    compensation: 1.0,
                    output_node: Some(graph.out_node),
                    ota_vccs: Vec::new(),
                    signal_chain_depth: Some(0),
                    nullor_comp_indices: vec![rec.comp_idx],
                });
            }
        }
    }

    // ══ Pass 4: Tree building ═════════════════════════════════════════
    // Detect JFETs that are LFO-controlled so they use variable-resistance mode
    // instead of full nonlinear NR solving.
    let lfo_controlled_jfets = detect_lfo_controlled_jfets(pedal);

    // Build nonlinear WDF stages from plans.
    // Triode plans that fail SP reduction fall back to single-NL MNA stages.
    let (nonlinear_stages, triode_fallback_stages, fallback_claimed_edges) =
        super::build::build_stages(
            &stage_plans,
            &classified,
            &graph,
            sample_rate,
            oversampling,
            &pp_transformer_edges,
            &lfo_controlled_jfets,
            supply_voltage,
            &node_island_depths,
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
        supply_voltage,
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

    // ══ Orphan output pot diagnostic ═════════════════════════════════
    let mut orphan_output_edges: Vec<usize> = Vec::new();
    if !stages.is_empty() || !multi_nl_stages.is_empty() {
        let (orphan_stages, output_edges) = rescue_orphan_output_pots(
            &graph,
            &classified,
            &stages,
            &multi_nl_stages,
            sample_rate,
            oversampling,
        );
        stages.extend(orphan_stages);
        orphan_output_edges = output_edges;
    }

    // ══ Orphan feedforward path compilation ═══════════════════════════
    if !stages.is_empty() || !multi_nl_stages.is_empty() {
        // claimed_edges already contains: opamp feedback edges (from build_opamp_feedback_stages)
        // + StagePlan passive edges + MultiNlPlan passive edges (accumulated in plan_stages).
        // Extend with the remaining fixed sets.
        claimed_edges.extend(classified.all_nonlinear_edge_indices.iter().copied());
        claimed_edges.extend(graph.active_edge_indices.iter().copied());
        claimed_edges.extend(pp_transformer_edges.iter().copied());
        claimed_edges.extend(classified.sidechain_edge_set.iter().copied());
        claimed_edges.extend(orphan_output_edges.iter().copied());
        claimed_edges.extend(fallback_claimed_edges.iter().copied());

        let feedforward_stages =
            build_feedforward_stages(&graph, &classified, &claimed_edges, sample_rate);
        stages.extend(feedforward_stages);
    }

    // ══ Passive-only fallback ═════════════════════════════════════════
    let mut passive_attenuation = 1.0;
    // Maps trigger component ID → (WDF stage index, injection node ID).
    let mut trigger_stage_map: HashMap<String, (usize, NodeId)> = HashMap::new();

    if stages.is_empty() && multi_nl_stages.is_empty() && push_pull_plans.is_empty() {
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
                .filter(|(_, e)| graph.components[e.comp_idx].kind.is_simple_passive())
                .map(|(i, _)| i)
                .collect();

            // Build adjacency from passive edges for reachability flood-fill.
            let mut passive_adj: HashMap<NodeId, Vec<(NodeId, usize)>> = HashMap::new();
            for &eidx in &all_passive_edges {
                let e = &graph.edges[eidx];
                passive_adj
                    .entry(e.node_a)
                    .or_default()
                    .push((e.node_b, eidx));
                passive_adj
                    .entry(e.node_b)
                    .or_default()
                    .push((e.node_a, eidx));
            }

            // Distance-based voice assignment: each non-gnd node belongs to
            // the nearest trigger. Equidistant nodes are shared (mix junctions).
            // Build gnd-free adjacency for distance computation.
            let mut adj_no_gnd: HashMap<NodeId, Vec<(NodeId, usize)>> = HashMap::new();
            for &eidx in &all_passive_edges {
                let e = &graph.edges[eidx];
                if e.node_a != graph.gnd_node && e.node_b != graph.gnd_node {
                    adj_no_gnd
                        .entry(e.node_a)
                        .or_default()
                        .push((e.node_b, eidx));
                    adj_no_gnd
                        .entry(e.node_b)
                        .or_default()
                        .push((e.node_a, eidx));
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
                            if let std::collections::hash_map::Entry::Vacant(e) =
                                dist.entry(neighbor)
                            {
                                e.insert(d + 1);
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
            let mut voice_nodes: Vec<HashSet<NodeId>> = vec![HashSet::new(); num_triggers];
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
            let all_voice_nodes: HashSet<NodeId> = voice_nodes.iter().flatten().copied().collect();
            let mut shared_output_edges: HashSet<usize> = HashSet::new();
            for &eidx in &all_passive_edges {
                let e = &graph.edges[eidx];
                let a_shared = shared_nodes.contains(&e.node_a)
                    || (!all_voice_nodes.contains(&e.node_a) && e.node_a != graph.gnd_node);
                let b_shared = shared_nodes.contains(&e.node_b)
                    || (!all_voice_nodes.contains(&e.node_b) && e.node_b != graph.gnd_node);
                if a_shared && b_shared {
                    shared_output_edges.insert(eidx);
                }
            }

            for (voice_idx, (comp_id, trigger_node)) in trigger_list.iter().enumerate() {
                // Voice edges: edges with at least one non-shared endpoint
                // owned by this voice, plus shared output edges.
                let voice_set = &voice_nodes[voice_idx];
                let mut voice_edges: HashSet<usize> = HashSet::new();
                for &eidx in &all_passive_edges {
                    let e = &graph.edges[eidx];
                    if voice_set.contains(&e.node_a) || voice_set.contains(&e.node_b) {
                        voice_edges.insert(eidx);
                    }
                }
                voice_edges.extend(&shared_output_edges);
                let voice_passive_edges: Vec<usize> = voice_edges.into_iter().collect();

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
                    trigger_stage_map.insert(comp_id.clone(), (stage_idx, *trigger_node));
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

    // ══ Trigger → opamp stage mapping ═════════════════════════════════
    // For opamp-based synth pedals (e.g. sine_drum), each trigger connects
    // through an injection resistor to a specific opamp stage's WDF tree.
    // BFS from each trigger node through passive edges to find the associated
    // opamp stage, then mark it as a trigger voice for per-note isolation.
    if !graph.trigger_nodes.is_empty() && !stages.is_empty() && trigger_stage_map.is_empty() {
        let hub_nodes: HashSet<super::graph::NodeId> = {
            let mut h = HashSet::new();
            h.insert(graph.gnd_node);
            h.insert(graph.in_node);
            h.insert(graph.out_node);
            h.extend(graph.supply_nodes.iter().copied());
            h
        };

        // Build passive adjacency (resistors, caps, inductors, pots).
        let mut passive_adj: HashMap<super::graph::NodeId, Vec<super::graph::NodeId>> =
            HashMap::new();
        for e in &graph.edges {
            if graph.components[e.comp_idx].kind.is_simple_passive() {
                passive_adj.entry(e.node_a).or_default().push(e.node_b);
                passive_adj.entry(e.node_b).or_default().push(e.node_a);
            }
        }

        // Map injection_node_id → stage index for quick lookup.
        let inj_to_stage: HashMap<super::graph::NodeId, usize> = stages
            .iter()
            .enumerate()
            .filter(|(_, s)| s.injection_node_id != usize::MAX)
            .map(|(i, s)| (s.injection_node_id, i))
            .collect();

        // Treat opamp output nodes as BFS barriers — stop before crossing
        // into another voice's domain through the shared mix bus.
        let mut barrier_nodes = hub_nodes.clone();
        barrier_nodes.extend(graph.output_pin_nodes.iter().copied());

        for (comp_id, trigger_node) in &graph.trigger_nodes {
            // BFS from trigger node through passive edges, stopping at
            // opamp output pins and hub nodes (gnd/vcc/in/out).
            let mut visited = HashSet::new();
            let mut queue = std::collections::VecDeque::new();
            visited.insert(*trigger_node);
            queue.push_back(*trigger_node);

            let mut found_stage = None;
            while let Some(node) = queue.pop_front() {
                if let Some(&stage_idx) = inj_to_stage.get(&node) {
                    found_stage = Some(stage_idx);
                    break;
                }
                if let Some(neighbors) = passive_adj.get(&node) {
                    for &neighbor in neighbors {
                        if !visited.contains(&neighbor) && !barrier_nodes.contains(&neighbor) {
                            visited.insert(neighbor);
                            queue.push_back(neighbor);
                        }
                    }
                }
            }

            if let Some(stage_idx) = found_stage {
                let inj_node = stages[stage_idx].injection_node_id;
                #[cfg(debug_assertions)]
                eprintln!(
                    "[trigger→stage] {} (node={}) → stage {} (inj={})",
                    comp_id, trigger_node, stage_idx, inj_node
                );
                trigger_stage_map.insert(comp_id.clone(), (stage_idx, inj_node));
                stages[stage_idx].is_trigger_voice = true;
                stages[stage_idx].output_node_id = graph.out_node;
            } else {
                #[cfg(debug_assertions)]
                eprintln!(
                    "[trigger→stage] {} (node={}) → NO STAGE FOUND",
                    comp_id, trigger_node
                );
            }
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
                let has_output = is_output_pin(&net.from) || net.to.iter().any(&is_output_pin);
                (has_secondary || has_secondary_to) && has_output
            });
            let input_to_primary = pedal.nets.iter().any(|net| {
                let has_primary =
                    pin_matches(&net.from, &comp.id, "a") || pin_matches(&net.from, &comp.id, "b");
                let has_primary_to = net
                    .to
                    .iter()
                    .any(|p| pin_matches(p, &comp.id, "a") || pin_matches(p, &comp.id, "b"));
                let has_input = is_input_pin(&net.from) || net.to.iter().any(&is_input_pin);
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
            let mut dl = crate::elements::DelayLine::new(
                dl_comp.min_delay,
                dl_comp.max_delay,
                sample_rate,
                dl_comp.interpolation,
            );
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
            let amplitude = pedal
                .supplies
                .first()
                .map(|s| s.config.voltage)
                .unwrap_or(9.0);
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
            trigger_id_to_idx
                .get(&mb.component)
                .map(|&idx| (mb.note, idx))
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

    let physical_gain = passive_attenuation * transformer_gain;
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

    // Power supply — always create a PSU model for dynamic sag.
    // When impedance/filter_cap aren't specified, use defaults scaled by voltage:
    //   - 9V pedal: ~5Ω impedance (battery/adapter), 100µF cap
    //   - 300V+ tube amp: ~100Ω impedance (rectifier+transformer), 40µF cap
    let primary_supply = pedal.supplies.first().map(|s| &s.config);

    let power_supply = primary_supply.map(|s| {
        let default_impedance = if s.voltage > 100.0 { 100.0 } else { 5.0 };
        let default_cap = if s.voltage > 100.0 { 40e-6 } else { 100e-6 };
        crate::elements::PowerSupply::new(
            s.voltage,
            s.impedance.unwrap_or(default_impedance),
            s.filter_cap.unwrap_or(default_cap),
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
                ControlTarget::PotInStage(_) | ControlTarget::PotInMultiNlStage(_, _)
            ) {
                Some(SmoothedParam::new(0.5, i, sample_rate))
            } else {
                None
            }
        })
        .collect();

    // ══ Topological stage ordering ═════════════════════════════════════
    // Build a unified stage execution order sorted by signal_flow_distance,
    // then topologically refined within same-sfd groups using output→injection
    // passive-edge reachability.
    let stage_order = {
        let mut order: Vec<(StageRef, usize)> = Vec::new();
        for (i, s) in stages.iter().enumerate() {
            order.push((StageRef::Wdf(i), s.signal_flow_distance));
        }
        for (i, s) in multi_nl_stages.iter().enumerate() {
            order.push((StageRef::MultiNl(i), s.signal_flow_distance));
        }

        // Build hub node set (shared nodes that would create false dependencies).
        let hub_nodes: HashSet<NodeId> = {
            let mut h = HashSet::new();
            h.insert(graph.in_node);
            h.insert(graph.out_node);
            h.insert(graph.gnd_node);
            h.insert(graph.vcc_node);
            h.extend(graph.supply_nodes.iter());
            h
        };

        // Collect injection node for each stage.
        let injection_nodes: Vec<usize> = order
            .iter()
            .map(|(sr, _)| match sr {
                StageRef::Wdf(i) => stages[*i].injection_node_id,
                StageRef::MultiNl(i) => multi_nl_stages[*i].injection_node_id,
            })
            .collect();

        // Build injection barrier set: injection nodes act as BFS barriers to
        // prevent traversal through opamp feedback paths (out→Rf→neg→R→prev_stage)
        // which would create false reverse dependencies.
        let injection_barrier: HashSet<NodeId> = injection_nodes
            .iter()
            .filter(|&&n| n != usize::MAX)
            .copied()
            .collect();

        // For each stage, BFS passive-reachable nodes from its output.
        // Exclude the stage's OWN injection from barriers so the BFS can
        // traverse through the opamp's feedback path to downstream stages.
        // Example: Bluesbreaker IC1a.out → IC1a.neg → Gain → R4 → IC1b.neg
        // requires traversing through IC1a.neg (wdf[0]'s own injection).
        let reachable: Vec<HashSet<NodeId>> = order
            .iter()
            .enumerate()
            .map(|(idx, (sr, _))| {
                let out_id = match sr {
                    StageRef::Wdf(i) => stages[*i].output_node_id,
                    StageRef::MultiNl(i) => multi_nl_stages[*i].output_node_id,
                };
                if out_id == usize::MAX {
                    HashSet::new()
                } else {
                    // Remove own injection from barriers so BFS can traverse
                    // the stage's own feedback network to find downstream stages.
                    let own_inj = injection_nodes[idx];
                    if own_inj != usize::MAX && injection_barrier.contains(&own_inj) {
                        let mut local_barrier = injection_barrier.clone();
                        local_barrier.remove(&own_inj);
                        bfs_passive_reachable_nodes(&graph, out_id, &hub_nodes, &local_barrier)
                    } else {
                        bfs_passive_reachable_nodes(&graph, out_id, &hub_nodes, &injection_barrier)
                    }
                }
            })
            .collect();

        // Build dependency graph: deps[j] contains indices i where stage j
        // depends on stage i (i's output reaches j's injection).
        let mut deps: Vec<Vec<usize>> = vec![Vec::new(); order.len()];
        for (i, reach) in reachable.iter().enumerate() {
            if reach.is_empty() {
                continue;
            }
            for (j, &inj) in injection_nodes.iter().enumerate() {
                if i != j && inj != usize::MAX && reach.contains(&inj) {
                    deps[j].push(i);
                }
            }
        }

        // Main-chain priority: within the same sfd group, stages whose injection
        // matches a previous-sfd stage's output (main chain) should process before
        // stages whose injection doesn't (side branches like shunt clippers).
        {
            // Collect output_node_ids grouped by sfd.
            let mut outputs_by_sfd: std::collections::BTreeMap<usize, HashSet<NodeId>> =
                std::collections::BTreeMap::new();
            for &(ref sr, sfd) in order.iter() {
                let out_id = match sr {
                    StageRef::Wdf(i) => stages[*i].output_node_id,
                    StageRef::MultiNl(i) => multi_nl_stages[*i].output_node_id,
                };
                if out_id != usize::MAX {
                    outputs_by_sfd.entry(sfd).or_default().insert(out_id);
                }
            }

            // For each sfd group > 0, check if stages are "main chain" (injection
            // matches a previous-sfd output) or "side branch" (injection doesn't).
            // Side branches depend on all main chain stages in the same group.
            let mut groups_by_sfd: std::collections::BTreeMap<usize, Vec<usize>> =
                std::collections::BTreeMap::new();
            for (i, &(_, sfd)) in order.iter().enumerate() {
                groups_by_sfd.entry(sfd).or_default().push(i);
            }

            for (&sfd, group) in &groups_by_sfd {
                if sfd == 0 || group.len() <= 1 {
                    continue;
                }
                // Collect all outputs from previous sfd levels.
                let prev_outputs: HashSet<NodeId> = outputs_by_sfd
                    .range(..sfd)
                    .flat_map(|(_, outs)| outs.iter().copied())
                    .collect();
                if prev_outputs.is_empty() {
                    continue;
                }

                let main_chain: Vec<usize> = group
                    .iter()
                    .copied()
                    .filter(|&i| {
                        let inj = injection_nodes[i];
                        inj != usize::MAX && prev_outputs.contains(&inj)
                    })
                    .collect();
                let side_branch: Vec<usize> = group
                    .iter()
                    .copied()
                    .filter(|&i| {
                        let inj = injection_nodes[i];
                        inj == usize::MAX || !prev_outputs.contains(&inj)
                    })
                    .collect();

                // Side branches depend on all main chain stages.
                for &sb in &side_branch {
                    for &mc in &main_chain {
                        if !deps[sb].contains(&mc) {
                            deps[sb].push(mc);
                        }
                    }
                }
            }
        }

        topological_refine_stage_order(order, &deps)
    };

    // ══ Assembly ══════════════════════════════════════════════════════
    let mut compiled = CompiledPedal {
        stages,
        push_pull_stages,
        multi_nl_stages,
        pre_gain,
        output_gain: 1.0,
        rail_saturation,
        rail_sat_oversampler: crate::oversampling::Oversampler::new(oversampling),
        sample_rate,
        controls,
        gain_range: gain_range_final,
        supply_voltage,
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
        output_dc_block: None,
        sidechains,
        // Normal (non-subcircuit) pedals have no subcircuit routing
        subcircuit_processors: Vec::new(),
        subcircuit_routing: Vec::new(),
        subcircuit_output_idx: None,
        subcircuit_outputs: Vec::new(),
        pot_smoothers,
        pot_mirrors: {
            // Build reverse mapping: source_id → [mirrored_ids]
            let mut m: std::collections::HashMap<String, Vec<super::compiled::MirrorPot>> =
                std::collections::HashMap::new();
            for (mirrored, source) in &pedal.mirrors {
                m.entry(source.clone())
                    .or_default()
                    .push(super::compiled::MirrorPot {
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
        original_passive_values: HashMap::new(),
    };

    // Enable inter-stage grid DC blocker for tube stages that aren't the first
    // in the signal chain. Without this, plate DC from the previous stage
    // (~80-200V) is fed directly into the tube's Vgk, saturating the grid.
    // The WDF tree's coupling cap blocks DC for wave propagation, but the grid
    // voltage is set externally via set_control_voltage(input). This DC blocker
    // mimics the coupling cap's effect on the grid voltage path.
    //
    // Additionally, add an output DC block to all tube stages that don't already
    // have one. Triode stages output the full plate voltage (~80-300V DC + AC).
    // Without a DC block, this large voltage propagates to downstream stages,
    // disrupting passive tone stacks and subsequent tube grid voltages. The
    // real circuit uses an inter-stage coupling cap for DC blocking; here we
    // model that with an IIR highpass at ~20 Hz (fc << 20 Hz audio content).
    for stage in &mut compiled.stages {
        // Set up input DC blocker for inter-stage tube grids
        if stage.signal_flow_distance > 0 {
            match &stage.root {
                super::stage::RootKind::Triode(_)
                | super::stage::RootKind::VariMu(_)
                | super::stage::RootKind::Pentode(_) => {
                    // Initialize the HPF state so the first-sample grid output is
                    // negative (safe operating point) rather than tracking the large
                    // positive DC from the previous stage's plate voltage.
                    //
                    // y_prev = -10.0 ensures the HPF output at sample 0 is:
                    //   y = input - 0 + α × (-10) ≈ -9V (for small input)
                    // This keeps the tube below positive grid bias during the first
                    // few milliseconds before the HPF converges, preventing cathode
                    // bypass cap latch from saturation-level plate current.
                    stage.grid_dc_blocker = Some((0.0, -10.0));
                }
                _ => {}
            }
        }
        // Set up output DC block for all tube stages that don't already have one.
        // This models the inter-stage coupling cap that blocks plate DC from
        // reaching downstream passive stages and tube grids.
        if stage.dc_block.is_none() {
            match &stage.root {
                super::stage::RootKind::Triode(_)
                | super::stage::RootKind::VariMu(_)
                | super::stage::RootKind::Pentode(_) => {
                    // IIR highpass at fc ≈ 20 Hz: passes audio, blocks plate DC.
                    // This approximates the typical 0.022 µF × 470 kΩ = 10ms RC
                    // time constant found in inter-stage coupling caps.
                    let omega = (std::f64::consts::PI * 2.0 * 20.0) / sample_rate;
                    let omega_tan = omega.tan();
                    let a1 = (1.0 - omega_tan) / (1.0 + omega_tan);
                    let b0 = 1.0 / (1.0 + omega_tan);
                    stage.dc_block = Some((a1, b0, 0.0, 0.0));
                }
                _ => {}
            }
        }
    }

    let initial_voltage = match &compiled.power_supply {
        Some(psu) => psu.steady_state_voltage(),
        None => supply_voltage,
    };
    compiled.set_supply_voltage(initial_voltage);

    // Auto-calibrate input and output levels if requested by pedal definition
    if pedal.calibrate {
        let input_atten = calibrate_input_level(&mut compiled);
        compiled.pre_gain *= input_atten;
        compiled.reset();

        let output_gain = calibrate_output_gain(&mut compiled);
        compiled.output_gain = output_gain;
        compiled.reset();
    }

    // Snapshot original passive values for reset support.
    compiled.snapshot_original_values();

    Ok(compiled)
}

// ═══════════════════════════════════════════════════════════════════════════
// Topological stage ordering helpers
// ═══════════════════════════════════════════════════════════════════════════

/// BFS from `start` through passive edges, returning all reachable nodes.
///
/// - Only traverses edges whose component `is_passive()` (R, C, L, pot, transformer).
/// - Hub nodes (gnd, vcc, in, out, supply) are excluded to avoid false dependencies
///   through shared power/ground rails.
/// - `output_pin_nodes` (opamp out, BJT collector, etc.) and `injection_barriers`
///   (stage injection nodes) act as barriers: included in the result but not
///   traversed further. Injection barriers prevent feedback paths (out→Rf→neg)
///   from creating false reverse dependencies.
fn bfs_passive_reachable_nodes(
    graph: &CircuitGraph,
    start: NodeId,
    hub_nodes: &HashSet<NodeId>,
    injection_barriers: &HashSet<NodeId>,
) -> HashSet<NodeId> {
    let mut visited: HashSet<NodeId> = HashSet::new();
    let mut queue = std::collections::VecDeque::new();
    visited.insert(start);
    queue.push_back(start);

    while let Some(node) = queue.pop_front() {
        for e in &graph.edges {
            // Only traverse passive/reactive edges.
            if !graph.components[e.comp_idx].kind.is_passive() {
                continue;
            }
            let neighbor = if e.node_a == node {
                Some(e.node_b)
            } else if e.node_b == node {
                Some(e.node_a)
            } else {
                None
            };
            let Some(n) = neighbor else { continue };
            // Hub nodes: skip entirely (don't include or traverse).
            if hub_nodes.contains(&n) {
                continue;
            }
            if !visited.insert(n) {
                continue;
            }
            // Barrier nodes: include but don't traverse further.
            // - output_pin_nodes: active device outputs (opamp.out, BJT collector)
            // - injection_barriers: stage injection points (opamp.neg, etc.)
            if graph.output_pin_nodes.contains(&n) || injection_barriers.contains(&n) {
                continue;
            }
            queue.push_back(n);
        }
    }

    visited
}

/// Topologically refine stage ordering within same-sfd groups.
///
/// Groups stages by `signal_flow_distance`. Within each group, runs Kahn's
/// algorithm using the provided dependency edges. Zero-in-degree stages are
/// emitted first, using original insertion order as tiebreaker. Any cycle
/// members are appended in original order.
fn topological_refine_stage_order(
    order: Vec<(StageRef, usize)>,
    deps: &[Vec<usize>],
) -> Vec<StageRef> {
    use std::collections::BTreeMap;

    // Group indices by sfd value (BTreeMap for sorted iteration).
    let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (i, &(_, sfd)) in order.iter().enumerate() {
        groups.entry(sfd).or_default().push(i);
    }

    let mut result = Vec::with_capacity(order.len());

    for group in groups.values() {
        if group.len() <= 1 {
            for &i in group {
                result.push(order[i].0);
            }
            continue;
        }

        let group_set: HashSet<usize> = group.iter().copied().collect();

        // Compute in-degree for nodes within this sfd group.
        let mut in_degree: HashMap<usize, usize> = HashMap::new();
        let mut adj: HashMap<usize, Vec<usize>> = HashMap::new();
        for &i in group {
            in_degree.entry(i).or_insert(0);
            for &dep in &deps[i] {
                if group_set.contains(&dep) {
                    adj.entry(dep).or_default().push(i);
                    *in_degree.entry(i).or_insert(0) += 1;
                }
            }
        }

        // Kahn's algorithm: seed queue with zero-in-degree nodes in original order.
        let mut queue = std::collections::VecDeque::new();
        for &i in group {
            if in_degree[&i] == 0 {
                queue.push_back(i);
            }
        }

        let mut sorted = Vec::new();
        while let Some(i) = queue.pop_front() {
            sorted.push(i);
            if let Some(neighbors) = adj.get(&i) {
                for &n in neighbors {
                    let deg = in_degree.get_mut(&n).unwrap();
                    *deg -= 1;
                    if *deg == 0 {
                        queue.push_back(n);
                    }
                }
            }
        }

        // Append any cycle members in original order.
        let sorted_set: HashSet<usize> = sorted.iter().copied().collect();
        for &i in group {
            if !sorted_set.contains(&i) {
                sorted.push(i);
            }
        }

        for i in sorted {
            result.push(order[i].0);
        }
    }

    result
}

/// Input attenuation: map 0dBFS DAW signal to guitar pickup level.
///
/// Guitar pickups output ~100-200mV peak. Pedal circuits are designed
/// for this range. We attenuate so 0dBFS (1.0) maps to 100mV (0.1).
/// Output calibration then compensates to restore unity gain.
///
/// Returns attenuation factor to multiply pre_gain by.
fn calibrate_input_level(_pedal: &mut CompiledPedal) -> f64 {
    // Single-coil average: ~50-80mV RMS, peak ~100mV.
    // But pedal circuits have internal gain stages (opamp, transistor)
    // that amplify before nonlinear elements. To preserve the full
    // dynamic range of the gain control, use a conservative level
    // that keeps the circuit below saturation at minimum gain.
    0.03
}

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
