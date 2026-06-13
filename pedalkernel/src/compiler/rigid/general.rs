//! General rigid stage building.
//!
//! Two public builders:
//! - `build_opamp_nl_feedback()` — Op-amp gain + NL root (TS/RAT/Klon)
//! - `build_general_mna()` / `build_general_mna_from_edges()` — Full MNA + NR
//!
//! The MNA path is decomposed into focused helpers:
//! 1. `collect_mna_nodes()` — unique circuit nodes (skipping GND/supply)
//! 2. `classify_nl_devices()` — NL edge classification + terminal pairs
//! 3. `stamp_passive_edges()` — resistors/caps/inductors into MNA
//! 4. `build_wdf_ports()` — NL + reactive + adapted input ports
//! 5. `derive_scattering()` — scattering matrix + Thevenin adaptation
//! 6. `compute_dc_bias()` — VCC injection → per-port DC bias
//! 7. `create_nl_devices()` — NlDeviceKind / BjtTwoPort groups

use std::collections::HashSet;

use super::super::build::create_root;
use super::super::classify::NonlinearKind;
use super::super::component::EdgeKind;
use super::super::dyn_node::DynNode;
use super::super::graph::{CircuitGraph, NodeId};
use super::super::helpers::{gummel_poon_model, triode_model, vari_mu_model};
use super::super::signal_flow::FlowGroup;
use super::super::stage::{
    MultiNlDeviceGroups, MultiNlScattering, MultiNlStage, NlDeviceGroupKind, NlDeviceKind,
    ScatteringRecomputeData, WdfStage, NR_ITERATION_BUDGET,
};
use super::super::wdf_leaf::{LeafKind, WdfVoltageSource};
use super::opamp_root::{extract_opamp_config, make_opamp_root};
use super::{is_inverting_topology, StageStats};
use crate::elements::*;
use crate::oversampling::{Oversampler, OversamplingFactor};
use crate::tree::{MnaSystem, RTypeAdaptor, WdfPort};
use pedalkernel_rt::boundary_math::{
    MnaNodeId, MnaOnePort, MnaPortTerminals, MnaVariableResistorBinding, OnePortKind,
    RuntimeOnePort, RuntimeState, WdfPortTerminals,
};
use pedalkernel_rt::wdf_leaf::WdfLeaf;

// ═══════════════════════════════════════════════════════════════════════════
// Op-amp + NL root (TS/RAT/Klon pattern)
// ═══════════════════════════════════════════════════════════════════════════

/// Build a stage with op-amp gain driving a nonlinear root.
///
/// Uses classified FlowGroup: Rf from feedback_edges, Ri from
/// pendant_edges, NL root from active_edges.
pub(in crate::compiler) fn build_opamp_nl_feedback(
    group: &FlowGroup,
    stats: &StageStats,
    graph: &CircuitGraph,
    sample_rate: f64,
    supply_voltage: f64,
    bias_v_max: Option<(f64, f64)>,
) -> Result<WdfStage, String> {
    let inverting = is_inverting_topology(stats, graph);
    let config = extract_opamp_config(group, inverting, graph)?;
    let mut opamp = make_opamp_root(&config, sample_rate, supply_voltage, bias_v_max);

    // Collect ALL NL active edges
    let nl_edge_indices: Vec<usize> = group
        .active_edges
        .iter()
        .filter(|&&eidx| graph.effective_edge_kind(eidx) == EdgeKind::Nonlinear)
        .copied()
        .collect();

    if nl_edge_indices.is_empty() {
        return Err("General stage has no NL edge".to_string());
    }

    // Classify all NL edges
    let mut nl_kinds: Vec<(NonlinearKind, usize)> = Vec::new(); // (kind, edge_idx)
    for &eidx in &nl_edge_indices {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        if let Some((kind, _)) = comp.kind.classify_nonlinear(
            &comp.id,
            e.node_a,
            e.node_b,
            graph.gnd_node,
            &graph.node_names,
        ) {
            nl_kinds.push((kind, eidx));
        }
    }

    // Detect antiparallel diode pairs from separate diode components.
    // Two SingleDiode edges between the same pair of nodes (with swapped polarity)
    // form an antiparallel pair → synthesize a DiodePair root using the first diode's model.
    let (root, base_diode_model) = if nl_kinds.len() >= 2 {
        let mut synthesized = false;
        let mut result_root = None;
        let mut result_model = None;

        // Check each pair of diode edges for antiparallel topology
        for i in 0..nl_kinds.len() {
            for j in (i + 1)..nl_kinds.len() {
                if let (
                    (NonlinearKind::SingleDiode(dt_a), eidx_a),
                    (NonlinearKind::SingleDiode(dt_b), eidx_b),
                ) = (&nl_kinds[i], &nl_kinds[j])
                {
                    let ea = &graph.edges[*eidx_a];
                    let eb = &graph.edges[*eidx_b];
                    // Antiparallel: node_a↔node_b swapped (same two nodes, opposite polarity)
                    let antiparallel = (ea.node_a == eb.node_b && ea.node_b == eb.node_a)
                        || (ea.node_a == eb.node_a && ea.node_b == eb.node_b);
                    if antiparallel {
                        // Use the higher-Vf diode's model for the pair.
                        // For asymmetric pairs (SD-1: silicon + germanium), this gives
                        // approximately correct symmetric clipping at the silicon threshold.
                        // TODO: implement AsymmetricDiodePairRoot for exact behavior.
                        use crate::dsl::DiodeType;
                        let dt = if dt_a == dt_b {
                            *dt_a
                        } else {
                            // Pick silicon (higher Vf) for now
                            match (dt_a, dt_b) {
                                (DiodeType::Silicon, _) | (_, DiodeType::Silicon) => {
                                    DiodeType::Silicon
                                }
                                _ => *dt_a,
                            }
                        };
                        let model = super::super::helpers::diode_model(dt);
                        result_root = Some(super::super::stage::RootKind::ExplicitDiodePair(
                            ExplicitDiodePairRoot::new(model),
                        ));
                        result_model = Some(model);
                        synthesized = true;
                        break;
                    }
                }
            }
            if synthesized {
                break;
            }
        }

        if synthesized {
            (result_root.unwrap(), result_model)
        } else {
            // Multiple NL edges but not antiparallel — use the first one
            create_root(&nl_kinds[0].0, false)
        }
    } else {
        create_root(&nl_kinds[0].0, false)
    };

    // VS with op-amp output impedance as source resistance.
    //
    // No pendant tree. The pendant edges (input coupling: Cin, R_b1, etc.)
    // are either in their own SPQR passive stage or provide DC bias handled
    // by bias_analysis. They don't affect the diode clipping — the diode
    // sees the op-amp's output impedance, not the input coupling impedance.
    //
    // The gain is already computed from Rf/Ri by extract_opamp_config and
    // applied by OpAmpRoot.compute_vs_voltage(). The VS output IS the
    // op-amp output voltage. The diode clips it.
    let tree = DynNode::Leaf(LeafKind::VoltageSource(WdfVoltageSource {
        voltage: 0.0,
        rp: config.model.output_impedance,
        is_cathode_bias: false,
        port_name: None,
    }));

    let oversampler = Oversampler::new(OversamplingFactor::X2);
    let mut stage = WdfStage::new(tree, root, oversampler);

    // Find feedback pot
    let feedback_pot = super::find_feedback_pot(group, graph);
    if let Some((pot_id, pot_leaf, fixed_r, _parallel_r)) = feedback_pot {
        let ri = if config.ri.is_finite() && config.ri > 0.0 {
            config.ri
        } else {
            stage.tree.port_resistance()
        };
        stage.feedback_pot_id = Some(pot_id);
        stage.feedback_series_r = fixed_r;
        stage.feedback_ri = ri;
        stage.opamp_children.push(pot_leaf);
    }

    stage.feedback_opamp = Some(opamp);
    stage.base_diode_model = base_diode_model;
    Ok(stage)
}

// ═══════════════════════════════════════════════════════════════════════════
// General MNA + NR solver
// ═══════════════════════════════════════════════════════════════════════════

/// Build from raw edge indices (no FlowGroup required).
pub(in crate::compiler) fn build_general_mna_from_edges(
    all_edges: &[usize],
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<MultiNlStage, String> {
    build_general_mna_from_edges_inner(all_edges, graph, sample_rate, &[], 9.0)
}

/// Build from raw edge indices with explicit init hints (branch rework).
pub(in crate::compiler) fn build_general_mna_from_edges_with_hints(
    all_edges: &[usize],
    graph: &CircuitGraph,
    sample_rate: f64,
    init_hints: &[crate::dsl::InitHint],
) -> Result<MultiNlStage, String> {
    build_general_mna_from_edges_inner(all_edges, graph, sample_rate, init_hints, 9.0)
}

/// Build from raw edge indices with explicit supply voltage (#85).
pub(in crate::compiler) fn build_general_mna_from_edges_with_supply(
    all_edges: &[usize],
    graph: &CircuitGraph,
    sample_rate: f64,
    supply_voltage: f64,
) -> Result<MultiNlStage, String> {
    build_general_mna_from_edges_inner(all_edges, graph, sample_rate, &[], supply_voltage)
}

/// Core builder carrying both branch features: init hints (warm-start /
/// nl_comp_labels matching) and an explicit supply voltage (triode v_max +
/// DC Q-point pre-charge from #85).
fn build_general_mna_from_edges_inner(
    all_edges: &[usize],
    graph: &CircuitGraph,
    sample_rate: f64,
    init_hints: &[crate::dsl::InitHint],
    supply_voltage: f64,
) -> Result<MultiNlStage, String> {
    let oversampling = OversamplingFactor::X2;
    let effective_rate = sample_rate * oversampling.ratio() as f64;

    // Step 1: Collect unique MNA nodes
    let mut node_set = collect_mna_nodes(all_edges, graph);

    let diode_ladder_filter = differential_diode_ladder_component_filter(all_edges, graph);

    // Step 2: Classify NL devices (also returns component labels for hint matching)
    let (nl_kinds, nl_comp_labels, nl_terminals) = classify_nl_devices(
        all_edges,
        graph,
        &mut node_set,
        diode_ladder_filter.as_ref(),
    )?;
    let n_nl = nl_terminals.len();

    // Step 3: Check VCC, build MNA, stamp passives
    let needs_vcc_port = check_vcc_needed(all_edges, &nl_terminals, graph);
    if needs_vcc_port && !node_set.contains(&graph.vcc_node) {
        node_set.push(graph.vcc_node);
    }

    let num_mna_nodes = node_set.len();
    let mut num_vsources = 0usize;
    let vcc_vs_idx = if needs_vcc_port {
        let idx = num_vsources;
        num_vsources += 1;
        Some(idx)
    } else {
        None
    };

    let node_to_mna = |node: NodeId| -> Option<usize> {
        if node == graph.gnd_node || graph.supply_nodes.contains(&node) {
            None
        } else if node == graph.vcc_node && !needs_vcc_port {
            None
        } else {
            node_set.iter().position(|&n| n == node)
        }
    };

    let nl_edge_set: HashSet<usize> = all_edges
        .iter()
        .filter(|&&eidx| graph.effective_edge_kind(eidx) == EdgeKind::Nonlinear)
        .copied()
        .collect();

    let (mut mna, reactive_edges, variable_resistor_candidates) = stamp_passive_edges(
        all_edges,
        &nl_edge_set,
        graph,
        &node_to_mna,
        num_mna_nodes,
        num_vsources,
        effective_rate,
        vcc_vs_idx,
    );

    // Step 4: Build WDF ports
    let (mut ports, port_node_pairs, passive_children, mut nl_port_resistances) = build_wdf_ports(
        &nl_terminals,
        &reactive_edges,
        graph,
        &node_to_mna,
        n_nl,
        all_edges,
        effective_rate,
    );
    let n_passive = passive_children.len();
    let extract_output_nodes = find_output_extract_node(all_edges, &node_set, graph)
        .and_then(node_to_mna)
        .map(WdfPortTerminals::single_ended);

    // Step 5: Derive scattering matrix + Thevenin adaptation
    let (scattering, vcc_injection_vec) =
        derive_scattering(&mna, &mut ports, &mut nl_port_resistances, n_nl, vcc_vs_idx)?;

    let n_total = ports.len();

    // Step 6: DC bias from VCC injection
    let (dc_bias, vcc_bias_all) = compute_dc_bias(
        vcc_injection_vec.as_deref(),
        &nl_kinds,
        n_nl,
        supply_voltage,
    );

    // Step 7: Create NL device groups
    let (nl_devices, device_groups) = create_nl_devices(&nl_kinds, supply_voltage)?;

    // Step 8: Assemble stage
    let mut stage = assemble_multi_nl_stage(
        mna,
        scattering,
        ports,
        port_node_pairs,
        passive_children,
        nl_devices,
        device_groups,
        nl_port_resistances,
        dc_bias,
        vcc_bias_all,
        vcc_vs_idx,
        n_nl,
        n_passive,
        supply_voltage,
        oversampling,
        effective_rate,
        graph,
        variable_resistor_candidates,
        extract_output_nodes,
        &nl_comp_labels,
        init_hints,
    )?;

    // Step 9: DC Q-point pre-charge for triode-with-grid stages.
    //
    // Problem: the cathode bypass cap (C_cathode) has a very small WDF port
    // resistance (rp ≈ 1/(2*fs*C) ≈ 0.2Ω for 25μF at 96kHz). In the MNA used
    // to derive the scattering matrix, this tiny rp effectively shorts the cathode
    // to GND, eliminating the cathode self-bias signal path. As a result:
    //   - dc_bias[0] (Vgk port) ≈ 0 — VCC has no direct linear path to grid
    //   - The NR solver converges to Vgk = 0 (no self-bias)
    //   - Gain is ~288x instead of expected ~50x
    //
    // Fix: compute the DC operating point (load-line intersection) from the
    // circuit resistances (R_plate, R_cathode) and pre-charge the cap to the
    // DC cathode voltage. This puts the system at the correct Q-point at t=0.
    if let Some(dc) = compute_triode_dc_qpoint(&nl_kinds, all_edges, graph, supply_voltage) {
        apply_triode_dc_qpoint(&mut stage, &dc, &nl_kinds, &reactive_edges, graph);
    }

    Ok(stage)
}

// ═══════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Step 1: Collect unique MNA nodes, excluding GND/VCC/supply.
fn collect_mna_nodes(all_edges: &[usize], graph: &CircuitGraph) -> Vec<NodeId> {
    let mut nodes: Vec<NodeId> = Vec::new();
    for &eidx in all_edges {
        let e = &graph.edges[eidx];
        for node in [e.node_a, e.node_b] {
            if node != graph.gnd_node
                && node != graph.vcc_node
                && !graph.supply_nodes.contains(&node)
                && !nodes.contains(&node)
            {
                nodes.push(node);
            }
        }
    }
    nodes
}

fn find_output_extract_node(
    all_edges: &[usize],
    node_set: &[NodeId],
    graph: &CircuitGraph,
) -> Option<NodeId> {
    if node_set.contains(&graph.out_node) {
        return Some(graph.out_node);
    }

    let edge_set: HashSet<usize> = all_edges.iter().copied().collect();
    graph.edges.iter().enumerate().find_map(|(eidx, edge)| {
        if edge_set.contains(&eidx) {
            return None;
        }
        if edge.node_a == graph.out_node && node_set.contains(&edge.node_b) {
            Some(edge.node_b)
        } else if edge.node_b == graph.out_node && node_set.contains(&edge.node_a) {
            Some(edge.node_a)
        } else {
            None
        }
    })
}

fn differential_diode_ladder_component_filter(
    all_edges: &[usize],
    graph: &CircuitGraph,
) -> Option<HashSet<usize>> {
    let mut seen = HashSet::new();
    let diode_connected_bjt_count = all_edges
        .iter()
        .filter(|&&eidx| graph.effective_edge_kind(eidx) == EdgeKind::Nonlinear)
        .filter(|&&eidx| {
            let edge = &graph.edges[eidx];
            if !seen.insert(edge.comp_idx) {
                return false;
            }
            let comp = &graph.components[edge.comp_idx];
            comp.kind
                .classify_nonlinear(
                    &comp.id,
                    edge.node_a,
                    edge.node_b,
                    graph.gnd_node,
                    &graph.node_names,
                )
                .is_some_and(|(kind, _)| is_diode_connected_bjt(&kind))
        })
        .count();
    if diode_connected_bjt_count < 8 {
        return None;
    }

    let plan = super::super::blockwise::analyze_blockwise(all_edges, graph)?;
    let mut rung_components = HashSet::new();
    let mut rung_count = 0usize;

    for block in &plan.blocks {
        let super::super::blockwise::BlockTopology::DifferentialDiodeRung {
            left_comp_idx,
            right_comp_idx,
            ..
        } = block.topology
        else {
            continue;
        };
        rung_count += 1;
        rung_components.insert(left_comp_idx);
        rung_components.insert(right_comp_idx);
    }

    (rung_count >= 4).then_some(rung_components)
}

/// Step 2: Classify NL edges into NonlinearKind + terminal pairs.
/// Deduplicates by comp_idx for multi-port devices (BJTs have 2 edges).
/// Also returns a parallel Vec of component labels (e.g. "Q1") for hint matching.
fn classify_nl_devices(
    all_edges: &[usize],
    graph: &CircuitGraph,
    node_set: &mut Vec<NodeId>,
    comp_filter: Option<&HashSet<usize>>,
) -> Result<(Vec<NonlinearKind>, Vec<String>, Vec<(NodeId, NodeId)>), String> {
    let mut nl_kinds = Vec::new();
    let mut nl_comp_labels = Vec::new();
    let mut nl_terminals = Vec::new();
    let mut seen: HashSet<usize> = HashSet::new();

    for &eidx in all_edges {
        if graph.effective_edge_kind(eidx) != EdgeKind::Nonlinear {
            continue;
        }
        let e = &graph.edges[eidx];
        if comp_filter.is_some_and(|filter| !filter.contains(&e.comp_idx)) {
            continue;
        }
        if !seen.insert(e.comp_idx) {
            continue;
        }
        let comp = &graph.components[e.comp_idx];
        let (kind, _) = comp
            .kind
            .classify_nonlinear(
                &comp.id,
                e.node_a,
                e.node_b,
                graph.gnd_node,
                &graph.node_names,
            )
            .ok_or_else(|| format!("NL edge {} ({}) didn't classify", eidx, comp.id))?;

        match &kind {
            NonlinearKind::BjtNpn {
                base_node,
                collector_node,
                emitter_node,
                ..
            }
            | NonlinearKind::BjtPnp {
                base_node,
                collector_node,
                emitter_node,
                ..
            } => {
                if base_node == collector_node {
                    nl_terminals.push((*base_node, *emitter_node));
                } else {
                    nl_terminals.push((*base_node, *emitter_node));
                    nl_terminals.push((*collector_node, *emitter_node));
                }
                for &n in &[*base_node, *collector_node, *emitter_node] {
                    if !node_set.contains(&n)
                        && n != graph.gnd_node
                        && !graph.supply_nodes.contains(&n)
                    {
                        node_set.push(n);
                    }
                }
            }
            // Triode with a connected grid: add grid-cathode port (port 0)
            // then plate-cathode port (port 1). This is what TriodeThreePort
            // and VariMuThreePort expect: [Vgk, Vpk].
            NonlinearKind::Triode {
                plate_node,
                cathode_node,
                grid_node: Some(grid),
                ..
            } => {
                let grid = *grid;
                let plate = *plate_node;
                let cathode = *cathode_node;
                nl_terminals.push((grid, cathode));   // port 0: grid-cathode
                nl_terminals.push((plate, cathode));  // port 1: plate-cathode
                for &n in &[grid, plate, cathode] {
                    if !node_set.contains(&n)
                        && n != graph.gnd_node
                        && !graph.supply_nodes.contains(&n)
                    {
                        node_set.push(n);
                    }
                }
            }
            _ => {
                nl_terminals.push((e.node_a, e.node_b));
            }
        }
        nl_comp_labels.push(comp.id.clone());
        nl_kinds.push(kind);
    }

    if nl_kinds.is_empty() {
        return Err("build_general_mna: no NL edges found".to_string());
    }
    Ok((nl_kinds, nl_comp_labels, nl_terminals))
}

/// Check if any edge or NL terminal touches VCC.
fn check_vcc_needed(
    all_edges: &[usize],
    nl_terminals: &[(NodeId, NodeId)],
    graph: &CircuitGraph,
) -> bool {
    all_edges.iter().any(|&eidx| {
        let e = &graph.edges[eidx];
        e.node_a == graph.vcc_node || e.node_b == graph.vcc_node
    }) || nl_terminals
        .iter()
        .any(|&(p, n)| p == graph.vcc_node || n == graph.vcc_node)
}

/// Step 3: Build MNA system and stamp passive edges (skip NL).
/// Returns (mna, reactive_edges).
/// A pot detected during passive stamping. Stamped into MNA as a fixed resistor
/// at its initial position, but also tracked for runtime delta-updating.
struct VariableResistorCandidate {
    /// WDF pot leaf for runtime position changes.
    leaf: DynNode,
    /// MNA node indices (pos, neg) for G-matrix delta updates.
    mna_pos: Option<usize>,
    mna_neg: Option<usize>,
    /// Initial conductance stamped into MNA.
    initial_conductance: f64,
}

fn stamp_passive_edges(
    all_edges: &[usize],
    nl_edge_set: &HashSet<usize>,
    graph: &CircuitGraph,
    node_to_mna: &dyn Fn(NodeId) -> Option<usize>,
    num_mna_nodes: usize,
    num_vsources: usize,
    effective_rate: f64,
    vcc_vs_idx: Option<usize>,
) -> (
    MnaSystem,
    Vec<(usize, OnePortKind)>,
    Vec<VariableResistorCandidate>,
) {
    let mut mna = MnaSystem::new(num_mna_nodes, num_vsources);
    let mut reactive_edges: Vec<(usize, OnePortKind)> = Vec::new();
    let mut variable_resistor_candidates: Vec<VariableResistorCandidate> = Vec::new();

    for &eidx in all_edges {
        if nl_edge_set.contains(&eidx) {
            continue;
        }
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        let n1 = node_to_mna(e.node_a);
        let n2 = node_to_mna(e.node_b);

        if comp.kind.is_pot() {
            // Pots: stamp initial resistance into MNA AND create a WDF leaf
            // for runtime updates. A 3-terminal pot appears as two graph
            // edges: a-w tracks position, w-b tracks 1-position.
            if let Some(mut leaf) = comp.kind.make_leaf(&comp.id, effective_rate) {
                if pot_edge_is_wb_half(graph, &comp.id, e.node_a, e.node_b) {
                    if let DynNode::Leaf(ref mut l) = leaf {
                        l.set_complement();
                    }
                }
                let initial_r = leaf.port_resistance();
                mna.stamp_resistor(n1, n2, initial_r);
                variable_resistor_candidates.push(VariableResistorCandidate {
                    leaf,
                    mna_pos: n1,
                    mna_neg: n2,
                    initial_conductance: 1.0 / initial_r,
                });
            }
        } else if let Some(r) = comp.kind.resistance() {
            mna.stamp_resistor(n1, n2, r);
        } else if let Some(c) = comp.kind.capacitance() {
            reactive_edges.push((eidx, OnePortKind::Capacitor(c)));
        } else if let Some(l) = comp.kind.inductance() {
            reactive_edges.push((eidx, OnePortKind::Inductor(l)));
        }
    }

    // GMIN regularization
    for i in 0..num_mna_nodes {
        mna.stamp_resistor(Some(i), None, 1e9);
    }

    // VCC as ideal voltage source
    if let Some(vcc_idx) = vcc_vs_idx {
        let vcc_mna = node_to_mna(graph.vcc_node);
        mna.stamp_voltage_source(vcc_mna, None, vcc_idx);
    }

    (mna, reactive_edges, variable_resistor_candidates)
}

fn pot_edge_is_wb_half(graph: &CircuitGraph, comp_id: &str, a: NodeId, b: NodeId) -> bool {
    let Some(&w_node) = graph
        .node_names
        .get(&format!("{comp_id}.w"))
        .or_else(|| graph.node_names.get(&format!("{comp_id}.wiper")))
    else {
        return false;
    };
    let Some(&b_node) = graph.node_names.get(&format!("{comp_id}.b")) else {
        return false;
    };
    (a == w_node && b == b_node) || (a == b_node && b == w_node)
}

/// Step 4: Build WDF ports (NL + reactive + adapted input).
fn build_wdf_ports(
    nl_terminals: &[(NodeId, NodeId)],
    reactive_edges: &[(usize, OnePortKind)],
    graph: &CircuitGraph,
    node_to_mna: &dyn Fn(NodeId) -> Option<usize>,
    n_nl: usize,
    edge_indices: &[usize],
    effective_rate: f64,
) -> (
    Vec<WdfPort>,
    Vec<WdfPortTerminals>,
    Vec<MnaOnePort>,
    Vec<f64>,
) {
    let r_nl_default = 1000.0;
    let r_adapted = 1000.0;
    let mut ports = Vec::with_capacity(n_nl + reactive_edges.len() + 1);
    let mut port_node_pairs = Vec::new();
    let mut nl_port_resistances = vec![r_nl_default; n_nl];

    // NL ports
    for (i, &(pos_node, neg_node)) in nl_terminals.iter().enumerate() {
        let pos = node_to_mna(pos_node);
        let neg = node_to_mna(neg_node);
        ports.push(WdfPort {
            node_pos: pos,
            node_neg: neg,
            resistance: nl_port_resistances[i],
        });
        port_node_pairs.push(WdfPortTerminals::maybe_differential(pos, neg));
    }

    // Reactive ports
    let mut passive_one_ports = Vec::with_capacity(reactive_edges.len());
    for (eidx, kind) in reactive_edges {
        let e = &graph.edges[*eidx];
        let pos = node_to_mna(e.node_a);
        let neg = node_to_mna(e.node_b);
        let rp = kind.rp(effective_rate);
        ports.push(WdfPort {
            node_pos: pos,
            node_neg: neg,
            resistance: rp,
        });
        port_node_pairs.push(WdfPortTerminals::maybe_differential(pos, neg));
        passive_one_ports.push(MnaOnePort::new(
            MnaPortTerminals::maybe_differential(pos.map(MnaNodeId::new), neg.map(MnaNodeId::new)),
            *kind,
        ));
    }

    // Adapted (input voltage source) port.
    // Use graph.in_node if it's in this MNA. Otherwise, find the active
    // element's signal input pin from Component::signal_terminals().
    // Last resort: the boundary node nearest the global input (NL groups
    // with no amplifier inside, e.g. a clipper + tone network split off an
    // op-amp gain stage). Without this the adapted VS port is grounded and
    // the stage is structurally silent (s_nl_adapted = 0).
    let injection_mna = node_to_mna(graph.in_node)
        .or_else(|| {
            edge_indices.iter().find_map(|&eidx| {
                let comp = &graph.components[graph.edges[eidx].comp_idx];
                match comp.kind.signal_terminals() {
                    super::super::component::SignalTerminals::Amplifier { input, .. } => {
                        let key = format!("{}.{}", comp.id, input);
                        graph.node_names.get(&key).and_then(|&n| node_to_mna(n))
                    }
                    _ => None,
                }
            })
        })
        .or_else(|| find_input_boundary_node(edge_indices, graph, node_to_mna));
    ports.push(WdfPort {
        node_pos: injection_mna,
        node_neg: None,
        resistance: r_adapted,
    });
    port_node_pairs.push(WdfPortTerminals::maybe_single_ended(injection_mna));

    (
        ports,
        port_node_pairs,
        passive_one_ports,
        nl_port_resistances,
    )
}

/// Injection fallback: the boundary node nearest the global input.
///
/// NL groups that contain neither `graph.in_node` nor an amplifier receive
/// their audio from an upstream stage at a *boundary* node — a node of this
/// group that is also touched by edges outside the group. Among those, pick
/// the one with the shortest hop distance from `graph.in_node` (BFS over the
/// full graph, not traversing GND/supply rails, ties broken by NodeId for
/// determinism) so the adapted VS port lands where the upstream stage
/// actually drives this group.
fn find_input_boundary_node(
    edge_indices: &[usize],
    graph: &CircuitGraph,
    node_to_mna: &dyn Fn(NodeId) -> Option<usize>,
) -> Option<usize> {
    let edge_set: HashSet<usize> = edge_indices.iter().copied().collect();

    // Boundary nodes: in this group's MNA, but also touched by outside edges.
    let mut boundary: Vec<NodeId> = Vec::new();
    for (eidx, e) in graph.edges.iter().enumerate() {
        if edge_set.contains(&eidx) {
            continue;
        }
        for n in [e.node_a, e.node_b] {
            if node_to_mna(n).is_some() && !boundary.contains(&n) {
                boundary.push(n);
            }
        }
    }
    if boundary.is_empty() {
        return None;
    }

    // BFS hop distances from the global input. GND and supply rails are
    // barriers — almost everything meets there, so passing through them
    // would make the distances meaningless.
    let blocked =
        |n: NodeId| n == graph.gnd_node || n == graph.vcc_node || graph.supply_nodes.contains(&n);
    let mut dist: std::collections::HashMap<NodeId, usize> = std::collections::HashMap::new();
    let mut queue = std::collections::VecDeque::new();
    dist.insert(graph.in_node, 0);
    queue.push_back(graph.in_node);
    while let Some(n) = queue.pop_front() {
        let d = dist[&n];
        for e in &graph.edges {
            for (from, to) in [(e.node_a, e.node_b), (e.node_b, e.node_a)] {
                if from == n && !blocked(to) && !dist.contains_key(&to) {
                    dist.insert(to, d + 1);
                    queue.push_back(to);
                }
            }
        }
    }

    boundary
        .iter()
        .filter_map(|&n| dist.get(&n).map(|&d| (d, n)))
        .min()
        .and_then(|(_, n)| node_to_mna(n))
}

/// Step 5: Derive scattering matrix + iterative Thevenin adaptation.
fn derive_scattering(
    mna: &MnaSystem,
    ports: &mut [WdfPort],
    nl_port_resistances: &mut [f64],
    n_nl: usize,
    vcc_vs_idx: Option<usize>,
) -> Result<(Vec<f64>, Option<Vec<f64>>), String> {
    let n_total = ports.len();
    let mut vcc_injection: Option<Vec<f64>> = None;

    let mut scattering = if let Some(vcc_idx) = vcc_vs_idx {
        let (s, vcc_inj) = mna.derive_scattering_and_vs_injection(ports, vcc_idx);
        if s.iter().any(|v| !v.is_finite()) {
            return Err("Scattering matrix contains NaN/Inf".to_string());
        }
        vcc_injection = Some(vcc_inj);
        s
    } else {
        let s = mna.derive_scattering_matrix_general(ports);
        if s.iter().any(|v| !v.is_finite()) {
            return Err("Scattering matrix contains NaN/Inf".to_string());
        }
        s
    };

    // Iterative Thevenin adaptation of NL port resistances
    for _iter in 0..5 {
        let mut needs_recompute = false;
        for i in 0..n_nl {
            let s_refl = scattering[i * n_total + i];
            if s_refl.abs() > 0.05 {
                let z_th = nl_port_resistances[i] * (1.0 + s_refl) / (1.0 - s_refl);
                if z_th.is_finite() && z_th > 1.0 {
                    nl_port_resistances[i] = z_th;
                    ports[i].resistance = z_th;
                    needs_recompute = true;
                }
            }
        }
        if !needs_recompute {
            break;
        }
        if let Some(vcc_idx) = vcc_vs_idx {
            let (s, inj) = mna.derive_scattering_and_vs_injection(ports, vcc_idx);
            if s.iter().any(|v| !v.is_finite()) {
                break;
            }
            scattering = s;
            vcc_injection = Some(inj);
        } else {
            let s = mna.derive_scattering_matrix_general(ports);
            if s.iter().any(|v| !v.is_finite()) {
                break;
            }
            scattering = s;
        }
    }

    Ok((scattering, vcc_injection))
}

/// Step 6: Compute DC bias from VCC injection vector.
fn compute_dc_bias(
    vcc_injection: Option<&[f64]>,
    nl_kinds: &[NonlinearKind],
    n_nl: usize,
    supply_voltage: f64,
) -> (Vec<f64>, Vec<f64>) {
    let (mut dc_bias, vcc_bias_all) = if let Some(inj) = vcc_injection {
        let bias: Vec<f64> = inj.iter().take(n_nl).map(|&k| k * supply_voltage).collect();
        let bias_all: Vec<f64> = inj.iter().map(|&k| k * supply_voltage).collect();
        (bias, bias_all)
    } else {
        (vec![0.0; n_nl], Vec::new())
    };

    // Per-device port bias corrections.
    // BJTs: clamp Vbe port to ±0.65V if the VCC injection gave zero.
    // Triodes with grid: 2 ports each (Vgk, Vpk) — no extra clamping needed,
    // VCC injection provides Vpk bias; Vgk starts at 0 (correct for cold start).
    let vbe_threshold = 0.65;
    let mut port_idx = 0usize;
    for kind in nl_kinds {
        match kind {
            NonlinearKind::BjtNpn { .. } => {
                if port_idx < n_nl && dc_bias[port_idx].abs() < vbe_threshold {
                    dc_bias[port_idx] = vbe_threshold;
                }
                port_idx += 2;
            }
            NonlinearKind::BjtPnp { .. } => {
                if port_idx < n_nl && dc_bias[port_idx].abs() < vbe_threshold {
                    dc_bias[port_idx] = -vbe_threshold;
                }
                port_idx += 2;
            }
            NonlinearKind::Triode {
                grid_node: Some(_),
                ..
            } => {
                // Port 0 = Vgk (grid-cathode), port 1 = Vpk (plate-cathode).
                // Warm-start Vpk at half supply so the NR solver converges near
                // the operating point on the first sample.
                if port_idx + 1 < n_nl && dc_bias[port_idx + 1].abs() < 1.0 {
                    dc_bias[port_idx + 1] = supply_voltage * 0.5;
                }
                port_idx += 2;
            }
            _ => {
                port_idx += 1;
            }
        }
    }

    (dc_bias, vcc_bias_all)
}

fn is_diode_connected_bjt(kind: &NonlinearKind) -> bool {
    match kind {
        NonlinearKind::BjtNpn {
            base_node,
            collector_node,
            ..
        }
        | NonlinearKind::BjtPnp {
            base_node,
            collector_node,
            ..
        } => base_node == collector_node,
        _ => false,
    }
}

/// Step 7: Create NL device kinds or grouped multi-port models.
///
/// Triodes with a connected grid node → `TriodeThreePort` / `VariMuThreePort`
/// grouped path (2 ports each: [Vgk, Vpk]).
/// BJTs → `BjtTwoPort` grouped path (2 ports each: [Vbe, Vce]).
/// All other devices → single-port `NlDeviceKind` vector.
///
/// `supply_voltage` sets the initial `v_max` on triode/vari-mu devices so that
/// the MNA solver's plate-voltage shift (`vpk = v[1] + v_max`) is correct from
/// the first Newton-Raphson iteration.
fn create_nl_devices(
    nl_kinds: &[NonlinearKind],
    supply_voltage: f64,
) -> Result<(Vec<NlDeviceKind>, Option<MultiNlDeviceGroups>), String> {
    // Check if all NL devices are triodes with a connected grid node.
    let all_triode_with_grid = !nl_kinds.is_empty()
        && nl_kinds.iter().all(|k| {
            matches!(
                k,
                NonlinearKind::Triode {
                    grid_node: Some(_),
                    ..
                }
            )
        });

    let all_bjt = nl_kinds.iter().all(|k| {
        matches!(
            k,
            NonlinearKind::BjtNpn { .. } | NonlinearKind::BjtPnp { .. }
        )
    });
    let any_bjt_two_port = nl_kinds.iter().any(|k| {
        matches!(
            k,
            NonlinearKind::BjtNpn { .. } | NonlinearKind::BjtPnp { .. }
        ) && !is_diode_connected_bjt(k)
    });

    if all_triode_with_grid {
        let mut groups = Vec::new();
        let mut offsets = Vec::new();
        let mut offset = 0usize;
        for kind in nl_kinds {
            offsets.push(offset);
            match kind {
                NonlinearKind::Triode {
                    model_name,
                    parallel_count,
                    is_vari_mu,
                    ..
                } => {
                    if *is_vari_mu {
                        let model = vari_mu_model(model_name);
                        groups.push(NlDeviceGroupKind::VariMuThreePort(
                            VariMuThreePort::new_gnd_referenced(model, supply_voltage)
                                .with_parallel_count(*parallel_count),
                        ));
                    } else {
                        let model = triode_model(model_name);
                        groups.push(NlDeviceGroupKind::TriodeThreePort(
                            TriodeThreePort::new_gnd_referenced(model, supply_voltage)
                                .with_parallel_count(*parallel_count),
                        ));
                    }
                    offset += 2;
                }
                _ => unreachable!(),
            }
        }
        Ok((Vec::new(), Some(MultiNlDeviceGroups { groups, offsets })))
    } else if all_bjt && any_bjt_two_port {
        let mut groups = Vec::new();
        let mut offsets = Vec::new();
        let mut offset = 0usize;
        for kind in nl_kinds {
            offsets.push(offset);
            match kind {
                NonlinearKind::BjtNpn { model_name, .. } if !is_diode_connected_bjt(kind) => {
                    groups.push(NlDeviceGroupKind::BjtTwoPort(BjtTwoPort::new(
                        gummel_poon_model(model_name),
                    )));
                    offset += 2;
                }
                NonlinearKind::BjtPnp { model_name, .. } if !is_diode_connected_bjt(kind) => {
                    groups.push(NlDeviceGroupKind::BjtTwoPort(BjtTwoPort::new_pnp(
                        gummel_poon_model(model_name),
                    )));
                    offset += 2;
                }
                NonlinearKind::BjtNpn { .. } | NonlinearKind::BjtPnp { .. } => {
                    let device = super::super::build::create_nl_device(kind).ok_or_else(|| {
                        "Unsupported diode-connected BJT device kind in general MNA".to_string()
                    })?;
                    groups.push(NlDeviceGroupKind::SinglePort(device));
                    offset += 1;
                }
                _ => unreachable!(),
            }
        }
        Ok((Vec::new(), Some(MultiNlDeviceGroups { groups, offsets })))
    } else {
        let mut devices = Vec::new();
        for kind in nl_kinds {
            let device = super::super::build::create_nl_device(kind)
                .ok_or_else(|| "Unsupported NL device kind in general MNA".to_string())?;
            devices.push(device);
        }
        Ok((devices, None))
    }
}

/// Resolve a named BJT init state to (Vbe, Vce) magnitudes (unsigned; PNP sign applied by caller).
///
/// Named states:
/// - `saturated`: BJT fully on, low Vce. [Vbe=0.75, Vce=0.1]
/// - `cutoff`:    BJT fully off, full supply across CE. [Vbe=0.0, Vce=supply]
/// - `active`:    BJT in linear region. [Vbe=0.65, Vce=supply/2]
/// - `forward`:   Treated as active (used for diodes, mapped to active for BJT).
/// - `reverse`:   Treated as cutoff (used for diodes, mapped to cutoff for BJT).
///
/// These constants match the table in INIT_BLOCK_DESIGN.md.
fn resolve_bjt_init_state(state_name: &str, supply_voltage: f64) -> (f64, f64) {
    match state_name {
        "saturated" => (0.75, 0.1),
        "cutoff" => (0.0, supply_voltage),
        "active" => (0.65, supply_voltage * 0.5),
        // Diode aliases — map to closest BJT state
        "forward" => (0.65, supply_voltage * 0.5),
        "reverse" => (0.0, supply_voltage),
        // Unknown states fall through to active (safest non-zero state)
        _ => (0.65, supply_voltage * 0.5),
    }
}

/// Step 8: Assemble the final MultiNlStage.
#[allow(clippy::too_many_arguments)]
fn assemble_multi_nl_stage(
    mna: MnaSystem,
    scattering: Vec<f64>,
    ports: Vec<WdfPort>,
    port_node_pairs: Vec<WdfPortTerminals>,
    passive_one_port_specs: Vec<MnaOnePort>,
    nl_devices: Vec<NlDeviceKind>,
    device_groups: Option<MultiNlDeviceGroups>,
    nl_port_resistances: Vec<f64>,
    dc_bias: Vec<f64>,
    vcc_bias_all: Vec<f64>,
    vcc_vs_idx: Option<usize>,
    n_nl: usize,
    n_passive: usize,
    supply_voltage: f64,
    oversampling: OversamplingFactor,
    effective_rate: f64,
    graph: &CircuitGraph,
    variable_resistor_candidates: Vec<VariableResistorCandidate>,
    extract_output_nodes: Option<WdfPortTerminals>,
    nl_comp_labels: &[String],
    init_hints: &[crate::dsl::InitHint],
) -> Result<MultiNlStage, String> {
    let scattering_blocks = MultiNlScattering::from_full_matrix(&scattering, n_nl, n_passive);
    let port_resistances: Vec<f64> = ports.iter().map(|p| p.resistance).collect();
    let adaptor = RTypeAdaptor::new(scattering, &port_resistances);
    let r_adapted = 1000.0;
    let extract_coeffs = extract_output_nodes.map(|out| {
        let (out_pos, out_neg) = out.as_tuple();
        mna.derive_node_extraction_coeffs(&ports, out_pos, out_neg)
    });

    // Output port: last CE port for BJTs, first NL port otherwise
    let output_port = if let Some(ref dg) = device_groups {
        let last = dg.groups.len().saturating_sub(1);
        dg.offsets[last] + dg.groups[last].n_ports() - 1
    } else {
        0
    };

    // Initial voltage state from device groups (physics-based defaults).
    let mut initial_v = vec![0.0; n_nl];
    let mut max_group_ports = 0usize;
    if let Some(ref dg) = device_groups {
        for (g, group) in dg.groups.iter().enumerate() {
            let off = dg.offsets[g];
            max_group_ports = max_group_ports.max(group.n_ports());
            match group {
                NlDeviceGroupKind::BjtTwoPort(bjt) => {
                    let sign = if bjt.is_pnp { -1.0 } else { 1.0 };
                    if off < n_nl {
                        let vbe = bjt.model.nf * bjt.model.vt * (1.0e-3_f64 / bjt.model.is).ln();
                        initial_v[off] = sign * vbe.clamp(0.1, 0.8);
                    }
                    if off + 1 < n_nl {
                        initial_v[off + 1] = sign * supply_voltage * 0.5;
                    }
                }
                NlDeviceGroupKind::TriodeThreePort(_) | NlDeviceGroupKind::VariMuThreePort(_) => {
                    // Port 0 = Vgk: start at 0 (cold grid, no bias signal yet).
                    // Port 1 = Vpk: warm-start at half supply for faster convergence.
                    if off + 1 < n_nl {
                        initial_v[off + 1] = supply_voltage * 0.5;
                    }
                }
                _ => {}
            }
        }
    }

    // Apply init hints: override physics-based defaults with author-specified states.
    // Only BJT two-port groups are supported (Vbe + Vce at port offsets).
    // Unrecognized hints are silently ignored (they may target diodes or JFETs
    // in circuits where no grouped solver applies — not an error).
    if !init_hints.is_empty() {
        if let Some(ref dg) = device_groups {
            for (g, group) in dg.groups.iter().enumerate() {
                let label = nl_comp_labels.get(g).map(|s| s.as_str()).unwrap_or("");
                let hint = init_hints.iter().find(|h| h.device_label == label);
                let Some(hint) = hint else { continue };
                let off = dg.offsets[g];
                if let NlDeviceGroupKind::BjtTwoPort(bjt) = group {
                    let sign = if bjt.is_pnp { -1.0 } else { 1.0 };
                    let crate::dsl::InitState::Named(ref state_name) = hint.state;
                    let (vbe, vce) = resolve_bjt_init_state(state_name, supply_voltage);
                    if off < n_nl {
                        initial_v[off] = sign * vbe;
                    }
                    if off + 1 < n_nl {
                        initial_v[off + 1] = sign * vce;
                    }
                    eprintln!(
                        "[init-hint] {label}: {state_name} → Vbe={:.3}, Vce={:.3} (sign={sign})",
                        vbe, vce
                    );
                }
            }
        }
    }

    let nr_workspace = if device_groups.is_some() {
        crate::elements::nonlinear::solver::NrWorkspace::new_grouped(n_nl, max_group_ports)
    } else {
        crate::elements::nonlinear::solver::NrWorkspace::new(n_nl)
    };

    let mut passive_runtime_state = RuntimeState::new();
    let passive_one_ports = passive_one_port_specs
        .into_iter()
        .map(|spec| {
            let state_slot = passive_runtime_state.allocate_one_port(spec.kind);
            RuntimeOnePort::new(spec, state_slot)
        })
        .collect();

    Ok(MultiNlStage {
        adaptor,
        nl_devices,
        nl_port_resistances,
        passive_one_ports,
        passive_runtime_state,
        passive_sample_rate: effective_rate,
        pot_children: variable_resistor_candidates
            .iter()
            .map(|ps| ps.leaf.clone())
            .collect(),
        variable_resistors: variable_resistor_candidates
            .iter()
            .enumerate()
            .map(|(i, ps)| MnaVariableResistorBinding {
                child_idx: i,
                terminals: MnaPortTerminals::maybe_differential(
                    ps.mna_pos.map(MnaNodeId::new),
                    ps.mna_neg.map(MnaNodeId::new),
                ),
                conductance: ps.initial_conductance,
            })
            .collect(),
        n_nl,
        v_prev: initial_v.clone(),
        scattering: scattering_blocks,
        oversampler: Oversampler::new(oversampling),
        compensation: 1.0,
        output_port,
        device_groups,
        recompute_data: Some(ScatteringRecomputeData {
            mna,
            port_node_pairs,
            adapted_resistance: r_adapted,
            vs_source_index: None,
            vcc_vs_index: vcc_vs_idx,
            extract_output_nodes,
        }),
        signal_flow_distance: 0,
        #[cfg(debug_assertions)]
        debug_label: String::new(),
        bypass_serial: false,
        transformer_gain: 1.0,
        injection_node_id: graph.in_node,
        output_node_id: graph.out_node,
        recompute_pending: false,
        veb_bias_offset: 0.0,
        feedback_scale: 0.1,
        feedback_opamp: None,
        feedback_pot_id: None,
        linearized_ota: None,
        vs_injection: None,
        extract_coeffs,
        extract_vs: 0.0,
        state_space: None,
        iir: None,
        bias_pot_id: None,
        bias_emitter_r: 470.0,
        interp_table: None,
        dc_bias,
        vcc_bias_all,
        vcc_vs_index: vcc_vs_idx,
        supply_voltage,
        dc_blocker_x1: 0.0,
        dc_blocker_y1: 0.0,
        dc_ramp: 0,
        initial_v_prev: initial_v.clone(),
        v_prev_2: vec![0.0; n_nl],
        nr_workspace,
        work_b_passive: vec![0.0; n_passive],
        work_known_a: vec![0.0; n_nl],
        work_b_all: vec![0.0; n_nl + n_passive + 1],
        work_a_all: vec![0.0; n_nl + n_passive + 1],
        adaptive_x2: false,
        subsample_counter: 0,
        iteration_budget_remaining: NR_ITERATION_BUDGET,
        prev_input: 0.0,
        opamp_post_fx: None,
    })
}

// ═══════════════════════════════════════════════════════════════════════════
// DC Q-point pre-charge for triode stages
// ═══════════════════════════════════════════════════════════════════════════

/// DC operating-point data for a single common-cathode triode stage.
struct TriodeDcQpoint {
    /// Grid-cathode bias voltage (negative for self-biased stages, e.g. -1.1V).
    vgk: f64,
    /// Plate-cathode voltage at the Q-point (e.g. 120V for a 12AX7 @ 250V supply).
    vpk: f64,
    /// Cathode voltage = -vgk = Ia × R_cathode.
    v_cathode: f64,
    /// Plate current at Q-point (A).
    ia: f64,
}

/// Compute the DC operating point for a triode-with-grid stage.
///
/// Uses the load-line equations:
///   Vgk = -Ia × R_cathode    (cathode self-bias)
///   Vpk = VCC - Ia × R_plate (plate load line)
///   Ia = Triode.plate_current(Vgk, Vpk)
///
/// Solves with a simple Newton-Raphson iteration on the 1-D residual in Ia.
/// Returns `None` if the circuit doesn't have exactly one triode-with-grid or
/// if R_plate/R_cathode cannot be found.
fn compute_triode_dc_qpoint(
    nl_kinds: &[NonlinearKind],
    all_edges: &[usize],
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Option<TriodeDcQpoint> {
    // Only handle single-triode-with-grid stages.
    if nl_kinds.len() != 1 {
        return None;
    }
    let (model_name, plate_node, cathode_node) = match &nl_kinds[0] {
        NonlinearKind::Triode {
            model_name,
            plate_node,
            cathode_node,
            grid_node: Some(_),
            is_vari_mu: false,
            ..
        } => (model_name.as_str(), *plate_node, *cathode_node),
        _ => return None,
    };

    // Find R_plate: linear resistor between vcc_node and plate_node.
    let r_plate = all_edges.iter().find_map(|&eidx| {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        if graph.effective_edge_kind(eidx) != EdgeKind::Linear {
            return None;
        }
        let (a, b) = (e.node_a, e.node_b);
        if (a == graph.vcc_node && b == plate_node) || (b == graph.vcc_node && a == plate_node) {
            comp.kind.resistance()
        } else {
            None
        }
    })?;

    // Find R_cathode: linear resistor between cathode_node and gnd_node.
    let r_cathode = all_edges.iter().find_map(|&eidx| {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        if graph.effective_edge_kind(eidx) != EdgeKind::Linear {
            return None;
        }
        let (a, b) = (e.node_a, e.node_b);
        if (a == cathode_node && b == graph.gnd_node)
            || (b == cathode_node && a == graph.gnd_node)
        {
            comp.kind.resistance()
        } else {
            None
        }
    })?;

    // Newton-Raphson on F(Ia) = Ia - plate_current(Vgk(Ia), Vpk(Ia)) = 0.
    let model = super::super::helpers::triode_model(model_name);
    let mut triode = TriodeRoot::new_with_v_max(model, supply_voltage);

    let mut ia = 1e-4_f64; // initial guess: 0.1mA
    for _iter in 0..50 {
        let vgk = -ia * r_cathode;
        let vpk = (supply_voltage - ia * r_plate).max(0.0);
        triode.set_vgk(vgk);
        let ia_model = triode.plate_current(vpk);
        let f = ia - ia_model;
        // Numerical Jacobian dF/dIa ≈ 1 (since dIa_model/dIa is small)
        // Refine with secant step if needed; simple relaxation converges here.
        ia = (ia - f * 0.5).max(0.0);
        if f.abs() < 1e-9 {
            break;
        }
    }

    let vgk = -ia * r_cathode;
    let vpk = (supply_voltage - ia * r_plate).max(0.0);
    let v_cathode = ia * r_cathode;

    // Sanity: Q-point should have negative Vgk and positive Vpk.
    if vgk >= 0.0 || vpk <= 0.0 || !vgk.is_finite() || !vpk.is_finite() {
        return None;
    }

    Some(TriodeDcQpoint {
        vgk,
        vpk,
        v_cathode,
        ia,
    })
}

/// Apply a pre-computed DC Q-point to a just-built triode MultiNlStage.
///
/// Sets `v_prev` and `initial_v_prev` to (Vgk_dc, Vpk_dc) so the NR solver
/// warm-starts at the correct operating point on the first sample.
///
/// Also pre-charges any cathode bypass capacitors among the passive children
/// to `dc.v_cathode`, so the WDF cap state immediately reflects the correct
/// DC cathode voltage and the `s_nl_passive` coupling term produces the right
/// Vgk bias from the very first sample.
fn apply_triode_dc_qpoint(
    stage: &mut MultiNlStage,
    dc: &TriodeDcQpoint,
    nl_kinds: &[NonlinearKind],
    reactive_edges: &[(usize, OnePortKind)],
    graph: &CircuitGraph,
) {
    // Set NR warm-start voltages.
    if stage.v_prev.len() >= 2 {
        stage.v_prev[0] = dc.vgk;
        stage.v_prev[1] = dc.vpk;
    }
    if stage.initial_v_prev.len() >= 2 {
        stage.initial_v_prev[0] = dc.vgk;
        stage.initial_v_prev[1] = dc.vpk;
    }
    if stage.v_prev_2.len() >= 2 {
        stage.v_prev_2[0] = dc.vgk;
        stage.v_prev_2[1] = dc.vpk;
    }

    // Find cathode_node for the triode.
    let cathode_node = match nl_kinds.first() {
        Some(NonlinearKind::Triode { cathode_node, .. }) => *cathode_node,
        _ => return,
    };

    // Pre-charge cathode bypass capacitors (caps between cathode and GND).
    //
    // MERGE NOTE: #85's original code set `cap.state`/`cap.last_b` directly on a
    // `LeafKind::Capacitor` inside `passive_children`. The branch refactored
    // passive one-ports into `passive_one_ports: Vec<RuntimeOnePort>` whose
    // capacitor memory lives in the shared `passive_runtime_state`. The
    // semantically-equivalent pre-charge is to seed the capacitor *voltage*
    // state to V_cathode via the branch-native runtime-state API, so the first
    // sample's reflected wave already encodes the DC self-bias.
    //
    // `reactive_edges[k]` corresponds 1:1 to `passive_one_ports[k]` (both are
    // built in the same order by `build_wdf_ports`).
    for (k, (eidx, _)) in reactive_edges.iter().enumerate() {
        let e = &graph.edges[*eidx];
        let is_cathode_cap = (e.node_a == cathode_node && e.node_b == graph.gnd_node)
            || (e.node_b == cathode_node && e.node_a == graph.gnd_node);
        if !is_cathode_cap {
            continue;
        }
        if let Some(&one_port) = stage.passive_one_ports.get(k) {
            one_port.wdf_set_one_port_state(
                pedalkernel_rt::boundary_math::OnePortState::CapacitorVoltage(dc.v_cathode),
                &mut stage.passive_runtime_state,
            );
        }
    }
}
