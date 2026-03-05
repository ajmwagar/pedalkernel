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
use crate::elements::*;
use crate::oversampling::{Oversampler, OversamplingFactor};
use crate::thermal::ThermalModel;
use crate::tolerance::ToleranceEngine;

use super::compiled::*;
use super::dyn_node::DynNode;
use super::graph::*;
use super::helpers::*;
use super::stage::{RootKind, WdfStage};
use crate::tree::{MnaSystem, WdfPort};

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
        .filter(|c| {
            matches!(
                &c.kind,
                ComponentKind::Lfo(..) | ComponentKind::EnvelopeFollower(..)
            )
        })
        .map(|c| c.id.as_str())
        .collect();

    if modulator_ids.is_empty() {
        return result;
    }

    let ota_ids: HashSet<&str> = pedal
        .components
        .iter()
        .filter(|c| matches!(&c.kind, ComponentKind::OpAmp(ot) if ot.is_ota()))
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
        .filter(|c| {
            matches!(
                &c.kind,
                ComponentKind::Lfo(..) | ComponentKind::EnvelopeFollower(..)
            )
        })
        .map(|c| c.id.as_str())
        .collect();

    if modulator_ids.is_empty() {
        return result;
    }

    // Collect JFET component IDs for cross-reference.
    let jfet_ids: HashSet<&str> = pedal
        .components
        .iter()
        .filter(|c| matches!(&c.kind, ComponentKind::NJfet(_) | ComponentKind::PJfet(_)))
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

            if let ComponentKind::Resistor(r) = &graph.components[e.comp_idx].kind {
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
            matches!(
                graph.components[e.comp_idx].kind,
                ComponentKind::Resistor(_)
                    | ComponentKind::Capacitor(_)
                    | ComponentKind::Inductor(_)
                    | ComponentKind::Potentiometer(_, _)
            )
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
                    kind: ComponentKind::Resistor(1.0),
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
                sample_counter: 0,
                root_comp_id: String::new(),
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

/// Build a WDF stage using MNA-derived R-type adaptor for non-series-parallel
/// passive topologies (e.g., Twin-T notch filter).
///
/// Resistors are stamped directly into the MNA conductance matrix.
/// Reactive elements (capacitors, inductors) become WDF child ports with state.
/// An adapted voltage source port injects signal at `in_node`, and a
/// high-impedance probe port extracts output voltage at `out_node`.
fn build_passive_mna_stage(
    graph: &CircuitGraph,
    passive_edges: &[usize],
    sample_rate: f64,
    _oversampling: OversamplingFactor,
) -> Option<WdfStage> {
    // Verify that out_node is reachable from in_node through passive edges.
    // If not, these passive edges don't form an in→out signal path and
    // shouldn't produce a stage (e.g., input coupling caps that feed into
    // active stages handled by other builders).
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

    // ── Step 1: Collect unique circuit nodes, map to MNA indices ─────────
    // Ground is the reference (not an MNA node).
    let mut node_set = std::collections::BTreeSet::new();
    for &eidx in passive_edges {
        let e = &graph.edges[eidx];
        if e.node_a != graph.gnd_node {
            node_set.insert(e.node_a);
        }
        if e.node_b != graph.gnd_node {
            node_set.insert(e.node_b);
        }
    }
    // Ensure in_node and out_node are included
    if graph.in_node != graph.gnd_node {
        node_set.insert(graph.in_node);
    }
    if graph.out_node != graph.gnd_node {
        node_set.insert(graph.out_node);
    }

    let nodes: Vec<NodeId> = node_set.into_iter().collect();
    let num_mna_nodes = nodes.len();
    if num_mna_nodes == 0 {
        return None;
    }
    let node_to_idx: HashMap<NodeId, usize> = nodes.iter().enumerate().map(|(i, &n)| (n, i)).collect();

    let to_mna = |node: NodeId| -> Option<usize> {
        if node == graph.gnd_node {
            None
        } else {
            node_to_idx.get(&node).copied()
        }
    };

    // ── Step 2: Create MNA system, stamp components ─────────────────────
    // Use the base sample rate for reactive port resistances. The PassiveRType
    // stage uses X1 oversampling (no internal oversampling), matching
    // build_output_rooted_stage. The caller (validation runner) already passes
    // the effective sample rate as the base rate.
    let effective_rate = sample_rate;

    let mut mna = MnaSystem::new(num_mna_nodes, 0);
    let mut reactive_children: Vec<DynNode> = Vec::new();
    let mut reactive_ports: Vec<WdfPort> = Vec::new();
    // Pot DynNodes are deferred — they go at the END of children (after ports)
    // so they don't interfere with the scattering matrix port indexing.
    let mut pot_children_pending: Vec<DynNode> = Vec::new();
    let mut pot_stamp_nodes: Vec<(Option<usize>, Option<usize>, f64)> = Vec::new();

    for &eidx in passive_edges {
        let e = &graph.edges[eidx];
        let n1 = to_mna(e.node_a);
        let n2 = to_mna(e.node_b);
        let comp = &graph.components[e.comp_idx];

        match &comp.kind {
            ComponentKind::Resistor(r) => {
                mna.stamp_resistor(n1, n2, *r);
            }
            ComponentKind::Capacitor(cfg) => {
                let rp = 1.0 / (2.0 * effective_rate * cfg.value);
                reactive_children.push(DynNode::Capacitor {
                    capacitance: cfg.value,
                    rp,
                    state: 0.0,
                    last_b: 0.0,
                });
                reactive_ports.push(WdfPort {
                    node_pos: n1,
                    node_neg: n2,
                    resistance: rp,
                });
            }
            ComponentKind::Inductor(l) => {
                let rp = 2.0 * effective_rate * l;
                reactive_children.push(DynNode::Inductor {
                    inductance: *l,
                    rp,
                    state: 0.0,
                });
                reactive_ports.push(WdfPort {
                    node_pos: n1,
                    node_neg: n2,
                    resistance: rp,
                });
            }
            ComponentKind::Potentiometer(max_r, taper) => {
                // Pot stays in G matrix as conductance (NOT a WDF port).
                // DynNode::Pot is stored in children for control binding + position tracking.
                let initial_pos = 0.5;
                let tapered_pos = taper.apply(initial_pos);
                let r = (tapered_pos * *max_r).max(1.0);
                mna.stamp_resistor(n1, n2, r);
                pot_children_pending.push(DynNode::Pot {
                    comp_id: comp.id.clone(),
                    max_resistance: *max_r,
                    position: initial_pos,
                    taper: *taper,
                    rp: r,
                });
                pot_stamp_nodes.push((n1, n2, 1.0 / r));
            }
            _ => {
                // Unsupported passive component type — skip
            }
        }
    }

    // ── Step 3: GMIN regularization ─────────────────────────────────────
    const GMIN_RESISTANCE: f64 = 1e9; // 1 GΩ → 1 nS conductance
    for i in 0..num_mna_nodes {
        mna.stamp_resistor(Some(i), None, GMIN_RESISTANCE);
    }

    // ── Step 4: Create output probe port ────────────────────────────────
    // High-impedance resistor at out_node → gnd to measure output voltage.
    let out_mna = to_mna(graph.out_node);
    let output_probe_r = 1e9; // 1 GΩ probe (minimal circuit loading)
    let output_port_idx = reactive_children.len();
    reactive_children.push(DynNode::Resistor { rp: output_probe_r });
    reactive_ports.push(WdfPort {
        node_pos: out_mna,
        node_neg: None,
        resistance: output_probe_r,
    });

    // ── Step 4b: Append pot children AFTER all ports ─────────────────────
    // Pot DynNodes go at children[n_ports..] so they don't affect scattering
    // matrix indexing. They're only here for control binding + position tracking.
    let pot_stamps: Vec<(usize, Option<usize>, Option<usize>, f64)> = pot_stamp_nodes
        .into_iter()
        .enumerate()
        .map(|(i, (n1, n2, g))| {
            let child_idx = reactive_children.len() + i;
            (child_idx, n1, n2, g)
        })
        .collect();
    reactive_children.extend(pot_children_pending);

    // ── Step 5: Stamp VS as ideal voltage source into MNA B/C/D ────────
    // Instead of making the VS a WDF port (which adds internal impedance),
    // stamp it directly into the MNA system as a zero-impedance source.
    let in_mna = to_mna(graph.in_node);
    let mna = {
        // Recreate MNA with 1 voltage source branch
        let mut mna_vs = MnaSystem::new(num_mna_nodes, 1);
        // Copy conductance stamps (already includes GMIN from step 3)
        mna_vs.g_matrix = mna.g_matrix;
        // Stamp VS: V(in_node) - V(gnd) = E
        mna_vs.stamp_voltage_source(in_mna, None, 0);
        mna_vs
    };

    // ── Step 6: Derive scattering matrix + VS injection vector ──────────
    let ports: Vec<WdfPort> = reactive_ports;
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

    let stage = WdfStage {
        tree: dummy_tree,
        root: RootKind::PassiveRType {
            scattering,
            vs_injection,
            n_ports,
            children: reactive_children,
            output_port: output_port_idx,
            recompute_mna,
            recompute_ports,
            pot_stamps,
            needs_recompute: false,
        },
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
        sample_counter: 0,
        root_comp_id: String::new(),
    };
    Some(stage)
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

    // Verify this is a supported 2-element passive topology.
    let _supported = match (&source_comp.kind, &load_comp.kind) {
        (ComponentKind::Resistor(_), ComponentKind::Capacitor(_))
        | (ComponentKind::Capacitor(_), ComponentKind::Resistor(_))
        | (ComponentKind::Inductor(_), ComponentKind::Resistor(_))
        | (ComponentKind::Resistor(_), ComponentKind::Inductor(_))
        | (ComponentKind::Resistor(_), ComponentKind::Resistor(_)) => true,
        _ => return None,
    };

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
        sample_counter: 0,
        root_comp_id: String::new(),
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
            oversampling: OversamplingFactor::X2,
            tolerance: ToleranceEngine::ideal(),
            thermal: false,
            collapse_nl: false,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
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

    // Apply component tolerance.
    for (i, comp) in graph.components.iter_mut().enumerate() {
        match &mut comp.kind {
            ComponentKind::Resistor(r) => {
                *r = tolerance.apply_resistor(*r, i);
            }
            ComponentKind::Capacitor(cfg) => {
                cfg.value = tolerance.apply_capacitor(cfg.value, i);
            }
            ComponentKind::Potentiometer(max_r, _) => {
                *max_r = tolerance.apply_resistor(*max_r, i);
            }
            _ => {}
        }
    }

    // ══ Pass 1: Element classification ════════════════════════════════
    let classified = super::classify::classify_circuit(&graph, pedal);

    // ══ Pass 2: Op-amp analysis ═══════════════════════════════════════
    let mut opamp_analysis = super::opamp_analysis::analyze_opamps(&graph, pedal);
    let opamp_feedback_gain = 1.0_f64;

    // Build op-amp feedback stages (inverting, non-inverting).
    let mut stages: Vec<WdfStage> = Vec::new();
    let opamp_feedback_stages = super::opamp_analysis::build_opamp_feedback_stages(
        &mut opamp_analysis,
        stages.len(),
        sample_rate,
        oversampling,
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

    let (stage_plans, push_pull_plans, multi_nl_plans, pp_transformer_edges, bjt_bias_analysis) =
        super::plan::plan_stages(&classified, &graph, sample_rate, &envelope_controlled_otas);

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
    );

    // Add triode fallback stages (SP-failed triodes built as single-NL MNA).
    multi_nl_stages.extend(triode_fallback_stages);

    // ══ Passive-only fallback ═════════════════════════════════════════
    let mut passive_attenuation = 1.0;

    if stages.is_empty() && multi_nl_stages.is_empty() {
        let has_reactive = pedal.components.iter().any(|c| {
            matches!(
                c.kind,
                ComponentKind::Capacitor(_)
                    | ComponentKind::Inductor(_)
                    | ComponentKind::Potentiometer(_, _)
            )
        });

        if has_reactive {
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
        if let ComponentKind::Transformer(cfg) = &comp.kind {
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
        if let ComponentKind::OpAmp(ot) = &comp.kind {
            if !ot.is_ota() {
                slew_limiters.push(SlewRateLimiter::new(ot.slew_rate(), sample_rate));
            }
        }
    }

    // ══ BBD delay lines ═══════════════════════════════════════════════
    let mut bbds = Vec::new();
    let mut bbd_id_to_idx: HashMap<String, usize> = HashMap::new();
    for comp in &pedal.components {
        if let ComponentKind::Bbd(bt) = &comp.kind {
            let idx = bbds.len();
            bbd_id_to_idx.insert(comp.id.clone(), idx);
            let model = match bt {
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
        if let ComponentKind::DelayLine(min_delay, max_delay, interp, medium) = &comp.kind {
            let idx = delay_lines.len();
            delay_id_to_idx.insert(comp.id.clone(), idx);
            let mut dl =
                crate::elements::DelayLine::new(*min_delay, *max_delay, sample_rate, *interp);
            dl.set_medium(*medium);
            delay_lines.push(DelayLineBinding {
                delay_line: dl,
                taps: vec![1.0],
                comp_id: comp.id.clone(),
            });
        }
    }

    for comp in &pedal.components {
        if let ComponentKind::Tap(parent_id, ratio) = &comp.kind {
            if let Some(&dl_idx) = delay_id_to_idx.get(parent_id) {
                delay_lines[dl_idx].taps.push(*ratio);
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
            match &comp.kind {
                ComponentKind::OpAmp(ot) if !ot.is_ota() => {
                    has_opamp = true;
                    opamp_swing = match ot {
                        OpAmpType::Tl072 | OpAmpType::Tl082 | OpAmpType::Generic => 0.92,
                        OpAmpType::Ne5532 => 0.90,
                        OpAmpType::Jrc4558 | OpAmpType::Rc4558 => 0.87,
                        OpAmpType::Lm308 | OpAmpType::Lm741 | OpAmpType::Op07 => 0.85,
                        _ => 0.85,
                    };
                }
                ComponentKind::Npn(_) | ComponentKind::Pnp(_) => {
                    has_bjt = true;
                }
                ComponentKind::NJfet(_)
                | ComponentKind::PJfet(_)
                | ComponentKind::Nmos(_)
                | ComponentKind::Pmos(_) => {
                    has_fet = true;
                }
                ComponentKind::Triode(name) => {
                    has_tube = true;
                    tube_mu = TriodeModel::try_by_name(name)
                        .map(|m| m.mu)
                        .unwrap_or(100.0);
                }
                ComponentKind::VariMu(_) => {
                    has_tube = true;
                    tube_mu = 35.0;
                }
                ComponentKind::Pentode(_) => {
                    has_tube = true;
                    tube_mu = 200.0;
                }
                _ => {}
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
            if matches!(c.kind, ComponentKind::Lfo(..)) {
                Some(c.id.clone())
            } else {
                None
            }
        })
        .collect();

    // Sidechain construction — must happen before build_controls so we have
    // the component → sidechain index mapping for control routing.
    let (sidechains, sidechain_comp_ids) =
        super::bind::build_sidechains(pedal, &graph, sample_rate);

    let controls = super::bind::build_controls(
        pedal,
        &stages,
        &multi_nl_stages,
        &opamp_analysis.pot_map,
        &bjt_bias_analysis.bias_pot_map,
        &lfo_ids,
        &delay_id_to_idx,
        delay_lines.is_empty(),
        &sidechain_comp_ids,
        &bbd_id_to_idx,
    );

    // Initialize BJT bias pots from their default positions.
    // This sets the initial feedback_scale and veb_bias_offset on
    // multi-NL stages before any user input arrives.
    for (pot_id, info) in &bjt_bias_analysis.bias_pot_map {
        let default_pos = pedal
            .controls
            .iter()
            .find(|c| c.component == *pot_id)
            .map(|c| c.default)
            .unwrap_or(0.5);
        let tapered = info.taper.apply(default_pos);
        for stage in &mut multi_nl_stages {
            stage.set_feedback_from_pot(tapered, info.max_pot_r);
        }
    }

    let level_default = pedal
        .controls
        .iter()
        .find(|c| is_level_label(&c.label))
        .map(|c| c.default)
        .unwrap_or(1.0);

    let physical_gain = opamp_feedback_gain * passive_attenuation * transformer_gain;
    // Gain range for PreGain controls ("Gain", "Drive", "Fuzz", etc.).
    // Maps the 0–1 control value to a useful input-level sweep:
    //   value=0 → near-unity (clean), value=1 → 10× drive (+20 dB).
    // This is a fallback for pots not found in any WDF stage tree.
    let gain_range_final = (physical_gain.max(0.1), physical_gain.max(0.1) * 10.0);
    let (pre_gain, output_gain) = (physical_gain, level_default);

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
    let supply_voltage = primary_supply.map_or(9.0, |s| s.voltage);

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
                    | ControlTarget::BjtBias { .. }
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
        output_gain,
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
            let mut m: std::collections::HashMap<String, Vec<String>> =
                std::collections::HashMap::new();
            for (mirrored, source) in &pedal.mirrors {
                m.entry(source.clone()).or_default().push(mirrored.clone());
            }
            m
        },
        base_grid_bias,
        multi_nl_recompute_counter: 0,
        stage_order,
        node_signals: Vec::new(),
        bbd_wet_mix: 0.5,
    };

    let initial_voltage = match &compiled.power_supply {
        Some(psu) => psu.steady_state_voltage(),
        None => supply_voltage,
    };
    compiled.set_supply_voltage(initial_voltage);

    Ok(compiled)
}
