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
use super::super::helpers::gummel_poon_model;
use super::super::signal_flow::FlowGroup;
use super::super::spqr_build::with_voltage_source;
use super::super::stage::{
    MultiNlDeviceGroups, MultiNlScattering, MultiNlStage, NlDeviceGroupKind, NlDeviceKind,
    ScatteringRecomputeData, WdfStage, NR_ITERATION_BUDGET,
};
use super::super::wdf_leaf::WdfVoltageSource;
use super::opamp_root::{extract_opamp_config, make_opamp_root};
use super::{is_inverting_topology, StageStats};
use crate::elements::*;
use crate::oversampling::{Oversampler, OversamplingFactor};
use crate::tree::{MnaSystem, RTypeAdaptor, WdfPort};

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
) -> Result<WdfStage, String> {
    let inverting = is_inverting_topology(stats, graph);
    let config = extract_opamp_config(group, inverting, graph)?;
    let opamp = make_opamp_root(&config, sample_rate);

    // Find first NL active edge → build root
    let nl_edge_idx = group
        .active_edges
        .iter()
        .find(|&&eidx| graph.effective_edge_kind(eidx) == EdgeKind::Nonlinear)
        .ok_or("General stage has no NL edge")?;

    let e = &graph.edges[*nl_edge_idx];
    let comp = &graph.components[e.comp_idx];
    let (nl_kind, _) = comp
        .kind
        .classify_nonlinear(&comp.id, e.node_a, e.node_b, graph.gnd_node, &graph.node_names)
        .ok_or_else(|| format!("NL edge {} ({}) didn't classify", nl_edge_idx, comp.id))?;

    let (root, base_diode_model) = create_root(&nl_kind, false);

    // Tree from pendant edges (input coupling)
    let tree = build_pendant_tree(&group.pendant_edges, graph, sample_rate);

    let oversampler = Oversampler::new(OversamplingFactor::X2);
    let mut stage = WdfStage::new(tree, root, oversampler);
    stage.feedback_opamp = Some(opamp);
    stage.base_diode_model = base_diode_model;
    Ok(stage)
}

/// Build a WDF tree from pendant edges, or a bare voltage source if none.
fn build_pendant_tree(pendant_edges: &[usize], graph: &CircuitGraph, sample_rate: f64) -> DynNode {
    if !pendant_edges.is_empty() {
        if let Some(leaf) = pendant_edges.iter().find_map(|&eidx| {
            let comp = &graph.components[graph.edges[eidx].comp_idx];
            comp.kind.make_leaf(&comp.id, sample_rate)
        }) {
            return with_voltage_source(leaf);
        }
    }
    DynNode::Leaf(Box::new(WdfVoltageSource {
        voltage: 0.0,
        rp: 1.0,
        is_cathode_bias: false,
    }))
}

// ═══════════════════════════════════════════════════════════════════════════
// General MNA + NR solver
// ═══════════════════════════════════════════════════════════════════════════

/// Build from a classified FlowGroup.
pub(in crate::compiler) fn build_general_mna(
    group: &FlowGroup,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<MultiNlStage, String> {
    build_general_mna_from_edges(&group.all_edges(), graph, sample_rate)
}

/// Build from raw edge indices (no FlowGroup required).
pub(in crate::compiler) fn build_general_mna_from_edges(
    all_edges: &[usize],
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<MultiNlStage, String> {
    let oversampling = OversamplingFactor::X2;
    let effective_rate = sample_rate * oversampling.ratio() as f64;
    let supply_voltage = 9.0; // TODO: pass from PedalDef

    // Step 1: Collect unique MNA nodes
    let mut node_set = collect_mna_nodes(all_edges, graph);

    // Step 2: Classify NL devices
    let (nl_kinds, nl_terminals) = classify_nl_devices(all_edges, graph, &mut node_set)?;
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

    let (mut mna, reactive_edges) = stamp_passive_edges(
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
    let (mut ports, port_node_pairs, passive_children, mut nl_port_resistances) =
        build_wdf_ports(&nl_terminals, &reactive_edges, graph, &node_to_mna, n_nl);
    let n_passive = passive_children.len();

    // Step 5: Derive scattering matrix + Thevenin adaptation
    let (scattering, vcc_injection_vec) =
        derive_scattering(&mna, &mut ports, &mut nl_port_resistances, n_nl, vcc_vs_idx)?;

    let n_total = ports.len();

    // Step 6: DC bias from VCC injection
    let (dc_bias, vcc_bias_all) =
        compute_dc_bias(vcc_injection_vec.as_deref(), &nl_kinds, n_nl, supply_voltage);

    // Step 7: Create NL device groups
    let (nl_devices, device_groups) = create_nl_devices(&nl_kinds)?;

    // Step 8: Assemble stage
    assemble_multi_nl_stage(
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
        graph,
    )
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

/// Step 2: Classify NL edges into NonlinearKind + terminal pairs.
/// Deduplicates by comp_idx for multi-port devices (BJTs have 2 edges).
fn classify_nl_devices(
    all_edges: &[usize],
    graph: &CircuitGraph,
    node_set: &mut Vec<NodeId>,
) -> Result<(Vec<NonlinearKind>, Vec<(NodeId, NodeId)>), String> {
    let mut nl_kinds = Vec::new();
    let mut nl_terminals = Vec::new();
    let mut seen: HashSet<usize> = HashSet::new();

    for &eidx in all_edges {
        if graph.effective_edge_kind(eidx) != EdgeKind::Nonlinear {
            continue;
        }
        let e = &graph.edges[eidx];
        if !seen.insert(e.comp_idx) {
            continue;
        }
        let comp = &graph.components[e.comp_idx];
        let (kind, _) = comp
            .kind
            .classify_nonlinear(&comp.id, e.node_a, e.node_b, graph.gnd_node, &graph.node_names)
            .ok_or_else(|| format!("NL edge {} ({}) didn't classify", eidx, comp.id))?;

        match &kind {
            NonlinearKind::BjtNpn { base_node, collector_node, emitter_node, .. }
            | NonlinearKind::BjtPnp { base_node, collector_node, emitter_node, .. } => {
                nl_terminals.push((*base_node, *emitter_node));
                nl_terminals.push((*collector_node, *emitter_node));
                for &n in &[*base_node, *collector_node, *emitter_node] {
                    if !node_set.contains(&n) && n != graph.gnd_node && !graph.supply_nodes.contains(&n) {
                        node_set.push(n);
                    }
                }
            }
            _ => {
                nl_terminals.push((e.node_a, e.node_b));
            }
        }
        nl_kinds.push(kind);
    }

    if nl_kinds.is_empty() {
        return Err("build_general_mna: no NL edges found".to_string());
    }
    Ok((nl_kinds, nl_terminals))
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
    }) || nl_terminals.iter().any(|&(p, n)| p == graph.vcc_node || n == graph.vcc_node)
}

/// Step 3: Build MNA system and stamp passive edges (skip NL).
/// Returns (mna, reactive_edges).
fn stamp_passive_edges(
    all_edges: &[usize],
    nl_edge_set: &HashSet<usize>,
    graph: &CircuitGraph,
    node_to_mna: &dyn Fn(NodeId) -> Option<usize>,
    num_mna_nodes: usize,
    num_vsources: usize,
    effective_rate: f64,
    vcc_vs_idx: Option<usize>,
) -> (MnaSystem, Vec<(usize, DynNode)>) {
    let mut mna = MnaSystem::new(num_mna_nodes, num_vsources);
    let mut reactive_edges: Vec<(usize, DynNode)> = Vec::new();

    for &eidx in all_edges {
        if nl_edge_set.contains(&eidx) {
            continue;
        }
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        let n1 = node_to_mna(e.node_a);
        let n2 = node_to_mna(e.node_b);

        if let Some(r) = comp.kind.resistance() {
            mna.stamp_resistor(n1, n2, r);
        } else if let Some(c) = comp.kind.capacitance() {
            let rp = 1.0 / (2.0 * effective_rate * c);
            reactive_edges.push((eidx, DynNode::Capacitor(None, c, rp)));
        } else if let Some(l) = comp.kind.inductance() {
            let rp = 2.0 * effective_rate * l;
            reactive_edges.push((eidx, DynNode::Inductor(None, l, rp)));
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

    (mna, reactive_edges)
}

/// Step 4: Build WDF ports (NL + reactive + adapted input).
fn build_wdf_ports(
    nl_terminals: &[(NodeId, NodeId)],
    reactive_edges: &[(usize, DynNode)],
    graph: &CircuitGraph,
    node_to_mna: &dyn Fn(NodeId) -> Option<usize>,
    n_nl: usize,
) -> (Vec<WdfPort>, Vec<(Option<usize>, Option<usize>)>, Vec<DynNode>, Vec<f64>) {
    let r_nl_default = 1000.0;
    let r_adapted = 1000.0;
    let mut ports = Vec::with_capacity(n_nl + reactive_edges.len() + 1);
    let mut port_node_pairs = Vec::new();
    let mut nl_port_resistances = vec![r_nl_default; n_nl];

    // NL ports
    for (i, &(pos_node, neg_node)) in nl_terminals.iter().enumerate() {
        let pos = node_to_mna(pos_node);
        let neg = node_to_mna(neg_node);
        ports.push(WdfPort { node_pos: pos, node_neg: neg, resistance: nl_port_resistances[i] });
        port_node_pairs.push((pos, neg));
    }

    // Reactive ports
    let mut passive_children = Vec::with_capacity(reactive_edges.len());
    for (eidx, dyn_node) in reactive_edges {
        let e = &graph.edges[*eidx];
        let pos = node_to_mna(e.node_a);
        let neg = node_to_mna(e.node_b);
        let rp = dyn_node.port_resistance();
        ports.push(WdfPort { node_pos: pos, node_neg: neg, resistance: rp });
        port_node_pairs.push((pos, neg));
        passive_children.push(dyn_node.clone());
    }

    // Adapted (input voltage source) port
    let injection_mna = node_to_mna(graph.in_node);
    ports.push(WdfPort { node_pos: injection_mna, node_neg: None, resistance: r_adapted });
    port_node_pairs.push((injection_mna, None));

    (ports, port_node_pairs, passive_children, nl_port_resistances)
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
            if s.iter().any(|v| !v.is_finite()) { break; }
            scattering = s;
            vcc_injection = Some(inj);
        } else {
            let s = mna.derive_scattering_matrix_general(ports);
            if s.iter().any(|v| !v.is_finite()) { break; }
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

    // BJT VBE bias correction
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
            _ => { port_idx += 1; }
        }
    }

    (dc_bias, vcc_bias_all)
}

/// Step 7: Create NL device kinds or grouped BJT two-port models.
fn create_nl_devices(
    nl_kinds: &[NonlinearKind],
) -> Result<(Vec<NlDeviceKind>, Option<MultiNlDeviceGroups>), String> {
    let all_bjt = nl_kinds.iter().all(|k| {
        matches!(k, NonlinearKind::BjtNpn { .. } | NonlinearKind::BjtPnp { .. })
    });

    if all_bjt && !nl_kinds.is_empty() {
        let mut groups = Vec::new();
        let mut offsets = Vec::new();
        let mut offset = 0usize;
        for kind in nl_kinds {
            offsets.push(offset);
            match kind {
                NonlinearKind::BjtNpn { model_name, .. } => {
                    groups.push(NlDeviceGroupKind::BjtTwoPort(BjtTwoPort::new(gummel_poon_model(model_name))));
                    offset += 2;
                }
                NonlinearKind::BjtPnp { model_name, .. } => {
                    groups.push(NlDeviceGroupKind::BjtTwoPort(BjtTwoPort::new_pnp(gummel_poon_model(model_name))));
                    offset += 2;
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

/// Step 8: Assemble the final MultiNlStage.
#[allow(clippy::too_many_arguments)]
fn assemble_multi_nl_stage(
    mna: MnaSystem,
    scattering: Vec<f64>,
    ports: Vec<WdfPort>,
    port_node_pairs: Vec<(Option<usize>, Option<usize>)>,
    passive_children: Vec<DynNode>,
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
    graph: &CircuitGraph,
) -> Result<MultiNlStage, String> {
    let scattering_blocks = MultiNlScattering::from_full_matrix(&scattering, n_nl, n_passive);
    let port_resistances: Vec<f64> = ports.iter().map(|p| p.resistance).collect();
    let adaptor = RTypeAdaptor::new(scattering, &port_resistances);
    let r_adapted = 1000.0;

    // Output port: last CE port for BJTs, first NL port otherwise
    let output_port = if let Some(ref dg) = device_groups {
        let last = dg.groups.len().saturating_sub(1);
        dg.offsets[last] + dg.groups[last].n_ports() - 1
    } else {
        0
    };

    // Initial voltage state from device groups
    let mut initial_v = vec![0.0; n_nl];
    let mut max_group_ports = 0usize;
    if let Some(ref dg) = device_groups {
        for (g, group) in dg.groups.iter().enumerate() {
            let off = dg.offsets[g];
            max_group_ports = max_group_ports.max(group.n_ports());
            if let NlDeviceGroupKind::BjtTwoPort(bjt) = group {
                let sign = if bjt.is_pnp { -1.0 } else { 1.0 };
                if off < n_nl {
                    let vbe = bjt.model.nf * bjt.model.vt * (1.0e-3_f64 / bjt.model.is).ln();
                    initial_v[off] = sign * vbe.clamp(0.1, 0.8);
                }
                if off + 1 < n_nl {
                    initial_v[off + 1] = sign * supply_voltage * 0.5;
                }
            }
        }
    }

    let nr_workspace = if device_groups.is_some() {
        crate::elements::nonlinear::solver::NrWorkspace::new_grouped(n_nl, max_group_ports)
    } else {
        crate::elements::nonlinear::solver::NrWorkspace::new(n_nl)
    };

    Ok(MultiNlStage {
        adaptor,
        nl_devices,
        nl_port_resistances,
        passive_children,
        pot_children: Vec::new(),
        pot_mna_stamps: Vec::new(),
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
            extract_output_nodes: None,
        }),
        signal_flow_distance: 0,
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
        extract_coeffs: None,
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
