//! General rigid stage building (NL feedback paths).
//!
//! Handles stages with op-amp gain driving a nonlinear root
//! (TS/RAT/Klon clipping pattern). Uses classified FlowGroup.
//!
//! Also handles the General MNA path for circuits with nonlinear
//! elements (BJTs, coupled diodes) without a VCVS.

use std::collections::HashSet;

use super::super::build::create_root;
use super::super::classify::NonlinearKind;
use super::super::component::{EdgeKind, StampResult};
use super::super::components::{CapSwitched, Capacitor as CapacitorComp, Potentiometer as PotComp};
use super::super::dyn_node::DynNode;
use super::super::graph::{CircuitGraph, NodeId};
use super::super::helpers::gummel_poon_model;
use super::super::signal_flow::FlowGroup;
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

use super::super::spqr_build::with_voltage_source;

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
    // Extract op-amp config from classified group
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
    let tree = if !group.pendant_edges.is_empty() {
        let pendant_leaf = group.pendant_edges.iter().find_map(|&eidx| {
            let comp = &graph.components[graph.edges[eidx].comp_idx];
            comp.kind.make_leaf(&comp.id, sample_rate)
        });
        if let Some(leaf) = pendant_leaf {
            with_voltage_source(leaf)
        } else {
            DynNode::Leaf(Box::new(WdfVoltageSource {
                voltage: 0.0,
                rp: 1.0,
                is_cathode_bias: false,
            }))
        }
    } else {
        DynNode::Leaf(Box::new(WdfVoltageSource {
            voltage: 0.0,
            rp: 1.0,
            is_cathode_bias: false,
        }))
    };

    let oversampler = Oversampler::new(OversamplingFactor::X2);
    let mut stage = WdfStage::new(tree, root, oversampler);
    stage.feedback_opamp = Some(opamp);
    stage.base_diode_model = base_diode_model;
    Ok(stage)
}

/// Build a General MNA stage for circuits with NL elements (BJTs, diodes)
/// but no VCVS. Uses R-type adaptor + multi-port NR solver via `MultiNlStage`.
pub(in crate::compiler) fn build_general_mna(
    group: &FlowGroup,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<MultiNlStage, String> {
    let oversampling = OversamplingFactor::X2;
    let effective_rate = sample_rate * oversampling.ratio() as f64;
    let supply_voltage = 9.0; // TODO: pass from PedalDef

    // ── Step 1: Collect unique MNA nodes ─────────────────────────────────
    let all_edges = group.all_edges();
    let mut node_set: Vec<NodeId> = Vec::new();
    let mut add_node = |node: NodeId, ns: &mut Vec<NodeId>| {
        if node == graph.gnd_node
            || node == graph.vcc_node
            || graph.supply_nodes.contains(&node)
        {
            return;
        }
        if !ns.contains(&node) {
            ns.push(node);
        }
    };
    for &eidx in &all_edges {
        let e = &graph.edges[eidx];
        add_node(e.node_a, &mut node_set);
        add_node(e.node_b, &mut node_set);
    }

    // ── Step 2: Identify NL edges and classify them ──────────────────────
    let nl_edge_indices: Vec<usize> = all_edges
        .iter()
        .filter(|&&eidx| graph.effective_edge_kind(eidx) == EdgeKind::Nonlinear)
        .copied()
        .collect();
    if nl_edge_indices.is_empty() {
        return Err("build_general_mna: no NL edges found".to_string());
    }

    // Classify NL elements and collect terminal pairs.
    // Deduplicate by comp_idx — multi-port components (BJTs) have 2 edges
    // but are ONE device. classify_nonlinear() should be called once per device.
    let mut nl_kinds: Vec<NonlinearKind> = Vec::new();
    let mut nl_terminals: Vec<(NodeId, NodeId)> = Vec::new();
    let mut seen_comp_idx: HashSet<usize> = HashSet::new();
    for &eidx in &nl_edge_indices {
        let e = &graph.edges[eidx];
        if !seen_comp_idx.insert(e.comp_idx) {
            continue; // Already classified this component
        }
        let comp = &graph.components[e.comp_idx];
        let (kind, _junctions) = comp
            .kind
            .classify_nonlinear(&comp.id, e.node_a, e.node_b, graph.gnd_node, &graph.node_names)
            .ok_or_else(|| format!("NL edge {} ({}) didn't classify", eidx, comp.id))?;
        match &kind {
            NonlinearKind::BjtNpn { base_node, collector_node, emitter_node, .. }
            | NonlinearKind::BjtPnp { base_node, collector_node, emitter_node, .. } => {
                // BJT: 2 ports — BE (base-emitter) and CE (collector-emitter)
                nl_terminals.push((*base_node, *emitter_node));
                nl_terminals.push((*collector_node, *emitter_node));
                // Ensure BJT nodes are in node_set
                add_node(*base_node, &mut node_set);
                add_node(*collector_node, &mut node_set);
                add_node(*emitter_node, &mut node_set);
            }
            _ => {
                // Single-port NL (diode, etc.)
                nl_terminals.push((e.node_a, e.node_b));
            }
        }
        nl_kinds.push(kind);
    }
    let n_nl = nl_terminals.len();

    // ── Check for VCC-connected edges ────────────────────────────────────
    let needs_vcc_port = all_edges.iter().any(|&eidx| {
        let e = &graph.edges[eidx];
        e.node_a == graph.vcc_node || e.node_b == graph.vcc_node
    }) || nl_terminals
        .iter()
        .any(|&(p, n)| p == graph.vcc_node || n == graph.vcc_node);
    if needs_vcc_port && !node_set.contains(&graph.vcc_node) {
        node_set.push(graph.vcc_node);
    }

    // ── Build MNA ────────────────────────────────────────────────────────
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

    let mut mna = MnaSystem::new(num_mna_nodes, num_vsources);
    let mut reactive_edges: Vec<(usize, DynNode)> = Vec::new();
    let nl_edge_set: HashSet<usize> = nl_edge_indices.iter().copied().collect();

    // Stamp passive edges only (skip NL edges — they become WDF ports)
    for &eidx in &all_edges {
        if nl_edge_set.contains(&eidx) {
            continue;
        }
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        let n1 = node_to_mna(e.node_a);
        let n2 = node_to_mna(e.node_b);

        // Simple stamp: resistor, cap → reactive port, otherwise use trait
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

    // Stamp VCC as ideal voltage source
    if let Some(vcc_idx) = vcc_vs_idx {
        let vcc_mna = node_to_mna(graph.vcc_node);
        mna.stamp_voltage_source(vcc_mna, None, vcc_idx);
    }

    // ── Step 3: Build WDF ports ──────────────────────────────────────────
    // Port ordering: [NL_0..NL_{n-1}, reactive_0..reactive_{m-1}, adapted]
    let n_reactive = reactive_edges.len();
    let mut ports: Vec<WdfPort> = Vec::with_capacity(n_nl + n_reactive + 1);
    let mut port_node_pairs: Vec<(Option<usize>, Option<usize>)> = Vec::new();

    // NL ports
    let r_nl_default = 1000.0;
    let mut nl_port_resistances = vec![r_nl_default; n_nl];
    for (i, &(pos_node, neg_node)) in nl_terminals.iter().enumerate() {
        let pos = node_to_mna(pos_node);
        let neg = node_to_mna(neg_node);
        ports.push(WdfPort {
            node_pos: pos,
            node_neg: neg,
            resistance: nl_port_resistances[i],
        });
        port_node_pairs.push((pos, neg));
    }

    // Reactive ports
    let mut passive_children: Vec<DynNode> = Vec::with_capacity(n_reactive);
    for (eidx, dyn_node) in &reactive_edges {
        let e = &graph.edges[*eidx];
        let pos = node_to_mna(e.node_a);
        let neg = node_to_mna(e.node_b);
        let rp = dyn_node.port_resistance();
        ports.push(WdfPort {
            node_pos: pos,
            node_neg: neg,
            resistance: rp,
        });
        port_node_pairs.push((pos, neg));
    }
    for (_eidx, dyn_node) in reactive_edges {
        passive_children.push(dyn_node);
    }
    let n_passive = passive_children.len();

    // Adapted (input voltage source) port — find injection node
    let injection_node = graph.in_node;
    let r_adapted = 1000.0;
    let injection_mna = node_to_mna(injection_node);
    ports.push(WdfPort {
        node_pos: injection_mna,
        node_neg: None,
        resistance: r_adapted,
    });
    port_node_pairs.push((injection_mna, None));

    // ── Step 4: Derive scattering matrix ─────────────────────────────────
    let mut vcc_injection_vec: Option<Vec<f64>> = None;
    let mut scattering = if let Some(vcc_idx) = vcc_vs_idx {
        let (s, vcc_inj) = mna.derive_scattering_and_vs_injection(&ports, vcc_idx);
        if s.iter().any(|&sv| !sv.is_finite()) {
            return Err("Scattering matrix contains NaN/Inf".to_string());
        }
        vcc_injection_vec = Some(vcc_inj);
        s
    } else {
        let s = mna.derive_scattering_matrix_general(&ports);
        if s.iter().any(|&sv| !sv.is_finite()) {
            return Err("Scattering matrix contains NaN/Inf".to_string());
        }
        s
    };

    let n_total = ports.len();

    // Iterative Thévenin adaptation of NL port resistances
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
            let (s, vcc_inj) = mna.derive_scattering_and_vs_injection(&ports, vcc_idx);
            if s.iter().any(|&sv| !sv.is_finite()) {
                break;
            }
            scattering = s;
            vcc_injection_vec = Some(vcc_inj);
        } else {
            let s = mna.derive_scattering_matrix_general(&ports);
            if s.iter().any(|&sv| !sv.is_finite()) {
                break;
            }
            scattering = s;
        }
    }

    // ── Step 5: Extract sub-blocks and DC bias ───────────────────────────
    let scattering_blocks = MultiNlScattering::from_full_matrix(&scattering, n_nl, n_passive);

    let (mut dc_bias, vcc_bias_all) = if let Some(ref vcc_inj) = vcc_injection_vec {
        let bias: Vec<f64> = vcc_inj.iter().take(n_nl).map(|&k| k * supply_voltage).collect();
        let bias_all: Vec<f64> = vcc_inj.iter().map(|&k| k * supply_voltage).collect();
        (bias, bias_all)
    } else {
        (vec![0.0; n_nl], Vec::new())
    };

    // BJT BE bias correction
    {
        let vbe_threshold = 0.65;
        let mut port_idx = 0usize;
        for kind in &nl_kinds {
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
                _ => {
                    port_idx += 1;
                }
            }
        }
    }

    // ── Step 6: Create NL device groups ──────────────────────────────────
    let all_bjt = nl_kinds.iter().all(|k| {
        matches!(k, NonlinearKind::BjtNpn { .. } | NonlinearKind::BjtPnp { .. })
    });

    let (nl_devices, device_groups) = if all_bjt && !nl_kinds.is_empty() {
        let mut groups: Vec<NlDeviceGroupKind> = Vec::new();
        let mut offsets: Vec<usize> = Vec::new();
        let mut offset = 0usize;
        for kind in &nl_kinds {
            offsets.push(offset);
            match kind {
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
    } else {
        // Mixed or single-port devices
        let mut devices: Vec<NlDeviceKind> = Vec::new();
        for kind in &nl_kinds {
            let device = super::super::build::create_nl_device(kind)
                .ok_or_else(|| "Unsupported NL device kind in general MNA".to_string())?;
            devices.push(device);
        }
        (devices, None)
    };

    // ── Step 7: Determine output port ────────────────────────────────────
    // For BJTs: collector-emitter (port 1) of last device
    let output_port = if let Some(ref dg) = device_groups {
        let last_group = dg.groups.len().saturating_sub(1);
        let np = dg.groups[last_group].n_ports();
        dg.offsets[last_group] + np - 1
    } else if n_nl > 0 {
        0
    } else {
        0
    };

    // ── Step 8: Assemble MultiNlStage ────────────────────────────────────
    let port_resistances: Vec<f64> = ports.iter().map(|p| p.resistance).collect();
    let adaptor = RTypeAdaptor::new(scattering, &port_resistances);

    let recompute_data = Some(ScatteringRecomputeData {
        mna,
        port_node_pairs,
        adapted_resistance: r_adapted,
        vs_source_index: None,
        vcc_vs_index: vcc_vs_idx,
        extract_output_nodes: None,
    });

    // Compute initial v_prev from device groups
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
                _ => {}
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
        recompute_data,
        signal_flow_distance: 0,
        transformer_gain: 1.0,
        injection_node_id: injection_node,
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
