//! General rigid stage building.
//!
//! Public builder:
//! - `build_general_mna()` / `build_general_mna_from_edges()` — Full MNA + NR.
//!   Co-solves op-amp VCVS + in-loop NL (TS/SD-1 diode-across-feedback) and
//!   passive/BJT/tube groups in one grouped-NR system. (Retired the
//!   build_opamp_nl_feedback shortcut — pedalkernel-9xu1.)
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

use super::super::classify::NonlinearKind;
use super::super::component::EdgeKind;
use super::super::dyn_node::DynNode;
use super::super::graph::{CircuitGraph, NodeId};
use super::super::helpers::{gummel_poon_model, pentode_model, triode_model, vari_mu_model};
use super::super::stage::{
    MultiNlDeviceGroups, MultiNlScattering, MultiNlStage, NlDeviceGroupKind, NlDeviceKind,
    ScatteringRecomputeData, NR_ITERATION_BUDGET,
};
use super::super::wdf_leaf::LeafKind;
use super::StageStats;
use crate::elements::*;
use crate::oversampling::{Oversampler, OversamplingFactor};
use crate::tree::{MnaSystem, RTypeAdaptor, WdfPort};
use pedalkernel_rt::boundary_math::{
    MnaNodeId, MnaOnePort, MnaPortTerminals, MnaVariableResistorBinding, OnePortKind,
    RuntimeOnePort, RuntimeState, WdfPortTerminals,
};
use pedalkernel_rt::wdf_leaf::WdfLeaf;


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

    // Op-amp nullor (VCVS) support (pedalkernel-9xu1): an in-loop NL device
    // (TS/SD-1/RAT diode across the feedback) must co-solve against the op-amp's
    // CLOSED-LOOP impedance, not its output impedance. To do that the op-amp
    // VCVS must be a row in this MNA system. Ensure each in-stage nullor's
    // pos/neg/out nodes are in node_set (the `pos` bias node may be touched by
    // no other edge in this group, e.g. opamp_diode_clipper's U1.pos).
    for rec in &graph.nullor_pins {
        let in_stage = all_edges
            .iter()
            .any(|&eidx| graph.edges[eidx].comp_idx == rec.comp_idx);
        if in_stage {
            for node in [rec.pos_node, rec.neg_node, rec.out_node] {
                if node != graph.gnd_node
                    && node != graph.vcc_node
                    && !graph.supply_nodes.contains(&node)
                    && !node_set.contains(&node)
                {
                    node_set.push(node);
                }
            }
        }
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

    // Allocate one vsource per VCVS (op-amp nullor) component in this stage.
    // comp_vsrc_base maps comp_idx -> its vsource base index for stamp_mna_multi.
    let mut vcvs_vsrc_base: Vec<(usize, usize)> = Vec::new();
    {
        let mut seen_vcvs: HashSet<usize> = HashSet::new();
        for &eidx in all_edges {
            if graph.effective_edge_kind(eidx) != EdgeKind::Vcvs {
                continue;
            }
            let comp_idx = graph.edges[eidx].comp_idx;
            if !seen_vcvs.insert(comp_idx) {
                continue;
            }
            let count = graph.components[comp_idx].kind.mna_vsource_count();
            if count > 0 {
                vcvs_vsrc_base.push((comp_idx, num_vsources));
                num_vsources += count;
            }
        }
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
        &vcvs_vsrc_base,
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
    let extract_output_node_id = find_output_extract_node(all_edges, &node_set, graph);
    let extract_output_nodes = extract_output_node_id
        .and_then(node_to_mna)
        .map(WdfPortTerminals::single_ended);

    if std::env::var("PK9XU1_DEBUG").is_ok() {
        let node_names: std::collections::HashMap<NodeId, &String> =
            graph.node_names.iter().map(|(k, v)| (*v, k)).collect();
        let nm = |n: NodeId| node_names.get(&n).map(|s| s.as_str()).unwrap_or("?");
        eprintln!("[PK9XU1] === build_general_mna_from_edges_inner ===");
        eprintln!("[PK9XU1] num_mna_nodes={} num_vsources={} n_nl={}", num_mna_nodes, num_vsources, n_nl);
        eprintln!("[PK9XU1] vcvs_vsrc_base={:?}", vcvs_vsrc_base);
        for (i, &n) in node_set.iter().enumerate() {
            eprintln!("[PK9XU1]   mna node[{}] = graph {} ({})", i, n, nm(n));
        }
        eprintln!("[PK9XU1] in_node={} ({})  out_node={} ({})", graph.in_node, nm(graph.in_node), graph.out_node, nm(graph.out_node));
        eprintln!("[PK9XU1] extract_output_node={:?} ({})", extract_output_node_id, extract_output_node_id.map(nm).unwrap_or("none"));
        eprintln!("[PK9XU1] injection (in_node mna)={:?}", node_to_mna(graph.in_node));
        for rec in &graph.nullor_pins {
            let in_stage = all_edges.iter().any(|&e| graph.edges[e].comp_idx == rec.comp_idx);
            if in_stage {
                eprintln!("[PK9XU1] nullor pos={}({}) neg={}({}) out={}({})",
                    rec.pos_node, nm(rec.pos_node), rec.neg_node, nm(rec.neg_node), rec.out_node, nm(rec.out_node));
            }
        }
    }

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

    // Step 10: DC Q-point override for grouped-BJT stages (ko5g g725.2).
    //
    // Problem (BA283 trace y2wj / 685e): the runtime grouped-NR seeds each port's
    // DC from `dc_bias[i]`, built by `compute_dc_bias` as a LINEAR vcc-injection
    // superposition. That linear solve cannot model a nonlinear DC-coupled
    // feedback servo: the BA283 TR1 base is biased through R2 56k from the NFB
    // bus, whose DC level is established by the conducting output Darlington, so
    // the linear solve lands TR1's be-source at -2.43 V → the NR drives TR1 to
    // cutoff (Vbe≈0.36 V vs the validated 0.64 V) → ~120× low gain → -49.5 dB.
    //
    // Fix (mirrors the triode pair above): `compute_bjt_dc_qpoint` solves the
    // NONLINEAR DC operating point of the whole BJT group (Gummel-Poon junctions
    // co-solved against the group's resistor network, rails fixed) and
    // `apply_bjt_dc_qpoint` OVERRIDES `dc_bias[be]`/`dc_bias[ce]` (via an exact
    // wave-domain inversion against the stage's own scattering) plus the
    // warm-start `v_prev` with the device's true Q-point.
    //
    // STATUS (g725.2): the DC solve and the inversion are VERIFIED correct — the
    // BA283 TR1 lands at Vbe=0.608 V / base=0.996 V (matching the validated
    // 0.94/0.64) and `v*` is an exact fixed point of the runtime NR
    // (max|F(v*)| ≈ 9e-16).  HOWEVER seeding it makes the *grouped NR + reactive
    // ports* a JOINT equilibrium that the static seed does not yet stabilise:
    // the DC-coupled feedback caps drift off `passive_b`, the grouped NR stops
    // converging (residual ~27, budget-exhausted) and BA283 goes unstable.  That
    // is the banked deep-WDF reactive-feedback DC-equilibrium fix
    // (engine-adapted-input-port / cosolve family), not this bead's surface.
    //
    // To NOT regress every BJT circuit (HARD CONSTRAINT), the override is OFF by
    // default and opt-in via `PK_BJT_DCQPOINT=1`.  When off, the engine is
    // byte-identical to baseline; the solver + inversion + the BA283 RCA are
    // committed for the stabilisation follow-up.
    if std::env::var("PK_BJT_DCQPOINT").as_deref() == Ok("1") {
        if let Some(node_dc) = compute_bjt_dc_qpoint(&nl_kinds, all_edges, graph, supply_voltage) {
            apply_bjt_dc_qpoint(
                &mut stage,
                &node_dc,
                &nl_terminals,
                &nl_kinds,
                &reactive_edges,
                graph,
            );
        }
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
    let neighbor = graph.edges.iter().enumerate().find_map(|(eidx, edge)| {
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
    });
    if neighbor.is_some() {
        return neighbor;
    }

    // Op-amp feedback stage with the global `out` in a downstream stage: the
    // stage's true output is the op-amp's nullor output node (U1.out), where the
    // closed-loop signal appears. (pedalkernel-9xu1)
    graph
        .nullor_pins
        .iter()
        .find(|rec| {
            all_edges
                .iter()
                .any(|&eidx| graph.edges[eidx].comp_idx == rec.comp_idx)
        })
        .map(|rec| rec.out_node)
        .filter(|n| node_set.contains(n))
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
            // Pentode with a connected grid: same 2-port layout as triode.
            // Port 0 = grid-cathode (Vgk), port 1 = plate-cathode (Vpk).
            // This matches PentodeThreePort's NlDeviceGroupIv::eval() port ordering.
            NonlinearKind::Pentode {
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
    vcvs_vsrc_base: &[(usize, usize)],
) -> (
    MnaSystem,
    Vec<(usize, OnePortKind)>,
    Vec<VariableResistorCandidate>,
) {
    let mut mna = MnaSystem::new(num_mna_nodes, num_vsources);
    let mut reactive_edges: Vec<(usize, OnePortKind)> = Vec::new();
    let mut variable_resistor_candidates: Vec<VariableResistorCandidate> = Vec::new();
    let mut stamped_vcvs: HashSet<usize> = HashSet::new();

    for &eidx in all_edges {
        if nl_edge_set.contains(&eidx) {
            continue;
        }
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        let n1 = node_to_mna(e.node_a);
        let n2 = node_to_mna(e.node_b);

        // Op-amp nullor (VCVS): stamp the closed-loop constraint into the MNA so
        // an in-loop NL device (diode across the feedback) co-solves against the
        // op-amp's closed-loop impedance (pedalkernel-9xu1). Stamped once per
        // component via stamp_mna_multi (resolves pos/neg/out by pin name).
        if graph.effective_edge_kind(eidx) == EdgeKind::Vcvs {
            if let Some(&(_, vsrc_base)) =
                vcvs_vsrc_base.iter().find(|&&(ci, _)| ci == e.comp_idx)
            {
                if stamped_vcvs.insert(e.comp_idx) {
                    let pin_fn = |pin: &str| -> Option<usize> {
                        let key = format!("{}.{}", comp.id, pin);
                        let node = graph.node_names.get(&key)?;
                        node_to_mna(*node)
                    };
                    let mut ctx = super::super::component::StampContext {
                        pin_to_mna: &pin_fn,
                        vsrc_base,
                        internal_node_base: 0,
                        sample_rate: effective_rate,
                        reactive_one_ports: None,
                    };
                    comp.kind.stamp_mna_multi(&comp.id, &mut ctx, &mut mna);
                }
            }
            continue;
        }

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
        } else if comp.kind.is_jfet() || comp.kind.type_tag() == "photocoupler" {
            // Controlled resistors (JFET Rds, photocoupler CdS cell) are the
            // same shape as a pot: stamp the current resistance into the MNA G
            // matrix AND register a variable-resistor candidate so runtime
            // control (LFO/envelope, OR the LA-2A detector→LED GR coupling) can
            // delta-update the G matrix and re-derive scattering. Without this
            // the cell folds into the STATIC G matrix and the GR loop is inert
            // even when the LED is driven (the cell can never change R).
            if let Some(leaf) = comp.kind.make_leaf(&comp.id, effective_rate) {
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
/// Tubes (triodes + pentodes) with a connected grid node → `TriodeThreePort` /
/// `VariMuThreePort` / `PentodeThreePort` grouped path (2 ports each: [Vgk, Vpk]).
/// BJTs → `BjtTwoPort` grouped path (2 ports each: [Vbe, Vce]).
/// All other devices → single-port `NlDeviceKind` vector.
///
/// `supply_voltage` sets the initial `v_max` on tube devices so that
/// the MNA solver's plate-voltage interpretation is correct from
/// the first Newton-Raphson iteration.
fn create_nl_devices(
    nl_kinds: &[NonlinearKind],
    supply_voltage: f64,
) -> Result<(Vec<NlDeviceKind>, Option<MultiNlDeviceGroups>), String> {
    // Check if all NL devices are tubes (triodes or pentodes) with a connected grid node.
    // Triodes and pentodes both use the 3-port grouped path [Vgk, Vpk] — the
    // grid-context solve is essential for push-pull stages of either tube type.
    let all_tube_with_grid = !nl_kinds.is_empty()
        && nl_kinds.iter().all(|k| {
            matches!(
                k,
                NonlinearKind::Triode {
                    grid_node: Some(_),
                    ..
                } | NonlinearKind::Pentode {
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

    if all_tube_with_grid {
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
                NonlinearKind::Pentode { model_name, .. } => {
                    // Pentodes use the same 2-port grouped path as triodes: [Vgk, Vpk].
                    // parallel_count is not carried on NonlinearKind::Pentode (always 1
                    // per DSL element — the pedal def instantiates separate elements).
                    let model = pentode_model(model_name);
                    groups.push(NlDeviceGroupKind::PentodeThreePort(
                        PentodeThreePort::new_gnd_referenced(model, supply_voltage),
                    ));
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

    if std::env::var("PK9XU1_DEBUG").is_ok() {
        eprintln!("[PK9XU1] assemble: n_nl={} n_passive={} n_total_ports={} r_adapted={}",
            n_nl, n_passive, ports.len(), r_adapted);
        eprintln!("[PK9XU1] port resistances: {:?}", port_resistances);
        eprintln!("[PK9XU1] extract_output_nodes={:?}", extract_output_nodes);
        eprintln!("[PK9XU1] extract_coeffs={:?}", extract_coeffs);
        eprintln!("[PK9XU1] s_nl={:?}", scattering_blocks.s_nl);
        eprintln!("[PK9XU1] s_nl_passive={:?}", scattering_blocks.s_nl_passive);
        eprintln!("[PK9XU1] s_nl_adapted={:?}", scattering_blocks.s_nl_adapted);
        eprintln!("[PK9XU1] dc_bias={:?}", dc_bias);
    }

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
                NlDeviceGroupKind::TriodeThreePort(_)
                | NlDeviceGroupKind::VariMuThreePort(_)
                | NlDeviceGroupKind::PentodeThreePort(_)
                    if off + 1 < n_nl =>
                {
                    // Port 0 = Vgk: start at 0 (cold grid, no bias signal yet).
                    // Port 1 = Vpk: warm-start at half supply for faster NR convergence.
                    initial_v[off + 1] = supply_voltage * 0.5;
                }
                _ => {}
            }
        }
    }

    // Apply init hints: override physics-based defaults with author-specified states.
    //
    // Hints are matched per-DEVICE within device_groups. nl_comp_labels is a
    // parallel Vec to device_groups.groups with one label per device group,
    // so nl_comp_labels[g] == the component ID of groups[g]. For a cross-coupled
    // BJT pair (two separate BjtTwoPort groups), g=0→Q1 and g=1→Q2, so both
    // hints are applied to the correct port offsets.
    //
    // Unrecognized hints are silently ignored (they may target diodes or JFETs
    // in circuits where no grouped solver applies — not an error).
    let mut any_hint_applied = false;
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
                    any_hint_applied = true;
                    eprintln!(
                        "[init-hint] {label}: {state_name} → Vbe={:.3}, Vce={:.3} (sign={sign})",
                        vbe, vce
                    );
                }
            }
        }
    }

    // When init hints were applied, skip the DC ramp on reset() so the hinted
    // v_prev is used as the NR warm-start with full DC excitation (dc_scale=1.0).
    //
    // Without this, dc_ramp restores to 0 on reset(), making dc_scale ≈ 0 on
    // the first sample. With near-zero excitation, the NR solver converges to
    // v ≈ 0 regardless of the warm-start, erasing the asymmetric seed provided
    // by the init hints (see DC_RAMP_SAMPLES in MultiNlStage::process).
    //
    // For free-running oscillators (BJT astable multivibrators), this ensures
    // that after DAW reset() the NR warm-start starts at the hinted operating
    // point (e.g. Q1=saturated, Q2=cutoff) rather than a symmetric saddle.
    let initial_dc_ramp: u32 = if any_hint_applied { 256 } else { 0 };

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
        dc_ramp: initial_dc_ramp,
        initial_dc_ramp,
        initial_v_prev: initial_v.clone(),
        // Seed v_prev_2 from initial_v so the linear extrapolation warm-start
        // begins near the operating point rather than at zero.
        v_prev_2: initial_v.clone(),
        nr_workspace,
        work_b_passive: vec![0.0; n_passive],
        work_known_a: vec![0.0; n_nl],
        work_b_all: vec![0.0; n_nl + n_passive + 1],
        work_a_all: vec![0.0; n_nl + n_passive + 1],
        adaptive_x2: false,
        subsample_counter: 0,
        iteration_budget_remaining: NR_ITERATION_BUDGET,
        prev_input: 0.0,
        dc_qpoint_v: None,
        dc_qpoint_passive_b: Vec::new(),
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
    let (model_name, plate_node, cathode_node, parallel_count, is_vari_mu) = match &nl_kinds[0] {
        NonlinearKind::Triode {
            model_name,
            plate_node,
            cathode_node,
            grid_node: Some(_),
            parallel_count,
            is_vari_mu,
            ..
        } => (
            model_name.as_str(),
            *plate_node,
            *cathode_node,
            *parallel_count,
            *is_vari_mu,
        ),
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

    if is_vari_mu {
        // Variable-mu fixtures are fixed-bias devices: the grid control voltage
        // establishes Vgk directly, rather than via cathode self-bias. Use the
        // model's default bias (6386: -2 V) and solve the plate load line with
        // the Raffensperger model, not the Koren triode model.
        let model = super::super::helpers::vari_mu_model(model_name);
        let mut triode = VariMuTriodeRoot::new_with_v_max(model, supply_voltage)
            .with_parallel_count(parallel_count);
        let vgk = triode.vgk_bias();
        triode.set_vgk(vgk);

        let max_ia = (supply_voltage / r_plate.max(1.0)).max(1e-9);
        let mut lo = 0.0_f64;
        let mut hi = max_ia;
        let residual = |ia: f64, triode: &mut VariMuTriodeRoot| -> f64 {
            let vpk = (supply_voltage - ia * r_plate).max(0.0);
            ia - triode.plate_current(vpk)
        };

        let mut flo = residual(lo, &mut triode);
        let fhi = residual(hi, &mut triode);
        if !flo.is_finite() || !fhi.is_finite() || flo.signum() == fhi.signum() {
            return None;
        }
        for _ in 0..80 {
            let mid = 0.5 * (lo + hi);
            let fmid = residual(mid, &mut triode);
            if !fmid.is_finite() {
                return None;
            }
            if fmid.abs() < 1e-10 {
                lo = mid;
                hi = mid;
                break;
            }
            if flo.signum() == fmid.signum() {
                lo = mid;
                flo = fmid;
            } else {
                hi = mid;
            }
        }

        let ia = 0.5 * (lo + hi);
        let vpk = (supply_voltage - ia * r_plate).max(0.0);
        let v_cathode = ia * r_cathode;
        if vgk >= 0.0 || vpk <= 0.0 || !vgk.is_finite() || !vpk.is_finite() {
            return None;
        }
        return Some(TriodeDcQpoint {
            vgk,
            vpk,
            v_cathode,
            ia,
        });
    }

    // Newton-Raphson on F(Ia) = Ia - plate_current(Vgk(Ia), Vpk(Ia)) = 0.
    let model = super::super::helpers::triode_model(model_name);
    let mut triode = TriodeRoot::new_with_v_max(model, supply_voltage)
        .with_parallel_count(parallel_count);

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
    let is_vari_mu = matches!(
        nl_kinds.first(),
        Some(NonlinearKind::Triode {
            is_vari_mu: true,
            ..
        })
    );

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
    if is_vari_mu {
        if !stage.dc_bias.is_empty() {
            stage.dc_bias[0] = dc.vgk;
        }
        if !stage.vcc_bias_all.is_empty() {
            stage.vcc_bias_all[0] = dc.vgk;
        }
        // The Q-point has already been solved and seeded. Starting the DC ramp
        // at zero would pull the grid port back to cold 0 V for the first 256
        // samples, which is exactly the invalid startup this seeding avoids.
        stage.dc_ramp = 256;
        stage.initial_dc_ramp = 256;
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

// ═══════════════════════════════════════════════════════════════════════════
// BJT DC Q-point (ko5g g725.2) — nonlinear nodal DC solve for grouped BJTs
// ═══════════════════════════════════════════════════════════════════════════

/// Solve the **nonlinear** DC operating point of every BJT in the group.
///
/// This is the BJT analogue of [`compute_triode_dc_qpoint`].  Where the triode
/// path solves a single 1-D load line, a BJT group may contain a DC-coupled
/// feedback servo (BA283: TR1's base is biased through R2 from the NFB bus whose
/// level is set by the conducting Darlington), so the operating point of all
/// devices is mutually coupled and must be **co-solved**.
///
/// Method: full-network nodal Newton-Raphson over the group's interior
/// (non-rail) nodes.  Linear resistors contribute conductances; every BJT is
/// stamped via its Gummel-Poon `currents(Vbe, Vbc)` and a numerical 3×3 device
/// Jacobian.  Caps/inductors are open at DC and ignored.  Rails (GND / VCC /
/// other supplies) are held at their known voltages.
///
/// Returns a map of **DC node voltage** for every solved interior node (rails
/// excluded), or `None` if the group has no BJTs, the system is singular, the
/// solve fails to converge, or the result is non-physical (so non-BJT and
/// degenerate stages keep their existing linear `dc_bias` and stay
/// byte-identical to before).  The caller turns these node voltages into the
/// per-port DC operating point.
fn compute_bjt_dc_qpoint(
    nl_kinds: &[NonlinearKind],
    all_edges: &[usize],
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Option<std::collections::HashMap<NodeId, f64>> {
    // Gather the BJTs (skip diode-connected: those are handled as 1-port diodes).
    struct BjtRef<'a> {
        model_name: &'a str,
        is_npn: bool,
        base: NodeId,
        collector: NodeId,
        emitter: NodeId,
    }
    let mut bjts: Vec<BjtRef> = Vec::new();
    for kind in nl_kinds {
        match kind {
            NonlinearKind::BjtNpn {
                model_name,
                base_node,
                collector_node,
                emitter_node,
            } if base_node != collector_node => bjts.push(BjtRef {
                model_name,
                is_npn: true,
                base: *base_node,
                collector: *collector_node,
                emitter: *emitter_node,
            }),
            NonlinearKind::BjtPnp {
                model_name,
                base_node,
                collector_node,
                emitter_node,
            } if base_node != collector_node => bjts.push(BjtRef {
                model_name,
                is_npn: false,
                base: *base_node,
                collector: *collector_node,
                emitter: *emitter_node,
            }),
            _ => {}
        }
    }
    if bjts.is_empty() {
        return None;
    }

    // Known rail voltage, or None for an interior (solved) node.
    let rail_v = |node: NodeId| -> Option<f64> {
        if node == graph.gnd_node || graph.ac_ground_nodes.contains(&node) {
            Some(0.0)
        } else if node == graph.vcc_node {
            Some(
                graph
                    .supply_voltages
                    .get(&graph.vcc_node)
                    .copied()
                    .unwrap_or(supply_voltage),
            )
        } else if let Some(&v) = graph.supply_voltages.get(&node) {
            Some(v)
        } else if graph.supply_nodes.contains(&node) {
            Some(supply_voltage)
        } else {
            None
        }
    };

    // Resistor list (linear conductances): (node_a, node_b, g).
    let mut resistors: Vec<(NodeId, NodeId, f64)> = Vec::new();
    for &eidx in all_edges {
        if graph.effective_edge_kind(eidx) != EdgeKind::Linear {
            continue;
        }
        let e = &graph.edges[eidx];
        if let Some(r) = graph.components[e.comp_idx].kind.resistance() {
            if r > 0.0 {
                resistors.push((e.node_a, e.node_b, 1.0 / r));
            }
        }
    }

    // Only nodes with a real DC path participate: those incident to a resistor or
    // that are a BJT terminal.  Cap/inductor-only nodes (input coupling cap, output
    // tap behind Cout) are open at DC → excluded, else they produce zero-conductance
    // rows that make the Newton system singular.
    let mut dc_path: std::collections::HashSet<NodeId> = std::collections::HashSet::new();
    for &(na, nb, _) in &resistors {
        dc_path.insert(na);
        dc_path.insert(nb);
    }
    for b in &bjts {
        dc_path.insert(b.base);
        dc_path.insert(b.collector);
        dc_path.insert(b.emitter);
    }

    // Collect interior (non-rail) nodes from that DC-connected set.
    let mut interior: Vec<NodeId> = Vec::new();
    for &node in &dc_path {
        if rail_v(node).is_none() && !interior.contains(&node) {
            interior.push(node);
        }
    }
    interior.sort_unstable();
    let n = interior.len();
    if n == 0 {
        return None;
    }
    let idx: std::collections::HashMap<NodeId, usize> =
        interior.iter().enumerate().map(|(i, &nd)| (nd, i)).collect();

    // Pre-fetch device models.
    let models: Vec<GummelPoonModel> =
        bjts.iter().map(|b| gummel_poon_model(b.model_name)).collect();

    // Newton-Raphson on the interior node voltages.
    // Start interior nodes at a mild bias (½ supply) so junctions are near-on.
    let mut v = vec![supply_voltage * 0.5; n];

    // Helper: voltage at a node (rail or solved).
    let node_voltage = |node: NodeId, v: &[f64]| -> f64 {
        if let Some(rv) = rail_v(node) {
            rv
        } else {
            v[idx[&node]]
        }
    };

    // Stamp one BJT's currents into the KCL residual `f` and 3×3 Jacobian into
    // the system matrix `j`.  Signs: for an NPN, conventional current flows INTO
    // the collector and base, OUT of the emitter; KCL residual at a node is the
    // net current leaving the node, so a current flowing INTO a terminal is
    // subtracted from that node's residual.  For a PNP the device currents are
    // mirrored (Vbe/Vbc negated, currents reversed).
    // Small node-to-ground shunt conductance (SPICE `gmin`) regularizes the
    // Jacobian so the off-state cold start (all junctions at Vbe≈0, ~zero gm)
    // never produces a singular / wildly ill-scaled Newton system.
    let gmin = 1e-9_f64;

    let converged = {
        let mut converged = false;
        for _iter in 0..200 {
            let mut j = vec![0.0_f64; n * n];
            let mut f = vec![0.0_f64; n];

            // gmin shunt: every interior node leaks `gmin·V` to ground.
            for k in 0..n {
                f[k] += gmin * v[k];
                j[k * n + k] += gmin;
            }

            // Resistor stamps (KCL: current leaving node via R).
            for &(na, nb, g) in &resistors {
                let va = node_voltage(na, &v);
                let vb = node_voltage(nb, &v);
                let ia = idx.get(&na).copied();
                let ib = idx.get(&nb).copied();
                let i_ab = (va - vb) * g; // current a→b
                if let Some(a) = ia {
                    f[a] += i_ab;
                    j[a * n + a] += g;
                    if let Some(b) = ib {
                        j[a * n + b] -= g;
                    }
                }
                if let Some(b) = ib {
                    f[b] -= i_ab;
                    j[b * n + b] += g;
                    if let Some(a) = ia {
                        j[b * n + a] -= g;
                    }
                }
            }

            // BJT stamps.
            let h = 1e-6_f64;
            for (b, model) in bjts.iter().zip(models.iter()) {
                let vb_ = node_voltage(b.base, &v);
                let vc_ = node_voltage(b.collector, &v);
                let ve_ = node_voltage(b.emitter, &v);
                let sign = if b.is_npn { 1.0 } else { -1.0 };
                let vbe = sign * (vb_ - ve_);
                let vbc = sign * (vb_ - vc_);

                let (ic, ib_) = model.currents(vbe as pedalkernel_rt::Wave, vbc as pedalkernel_rt::Wave);
                let (ic, ib_) = (sign * ic as f64, sign * ib_ as f64);
                // Terminal currents flowing INTO the device (leaving the node):
                //   base: +Ib, collector: +Ic, emitter: -(Ib+Ic)
                let term = [
                    (b.base, ib_),
                    (b.collector, ic),
                    (b.emitter, -(ib_ + ic)),
                ];
                for &(node, i_term) in &term {
                    if let Some(&row) = idx.get(&node) {
                        f[row] += i_term;
                    }
                }

                // Numerical 3×3 Jacobian: ∂(terminal current)/∂(terminal V).
                let ctrl_nodes = [b.base, b.collector, b.emitter];
                for (k, &cn) in ctrl_nodes.iter().enumerate() {
                    // Only interior columns matter (rails are fixed).
                    let col = match idx.get(&cn) {
                        Some(&c) => c,
                        None => continue,
                    };
                    // Perturb terminal k's voltage.
                    let mut vbn = vb_;
                    let mut vcn = vc_;
                    let mut ven = ve_;
                    match k {
                        0 => vbn += h,
                        1 => vcn += h,
                        _ => ven += h,
                    }
                    let vbe_p = sign * (vbn - ven);
                    let vbc_p = sign * (vbn - vcn);
                    let (icp, ibp) =
                        model.currents(vbe_p as pedalkernel_rt::Wave, vbc_p as pedalkernel_rt::Wave);
                    let (icp, ibp) = (sign * icp as f64, sign * ibp as f64);
                    let dterm = [
                        (b.base, (ibp - ib_) / h),
                        (b.collector, (icp - ic) / h),
                        (b.emitter, (-(ibp + icp) - -(ib_ + ic)) / h),
                    ];
                    for &(node, d) in &dterm {
                        if let Some(&row) = idx.get(&node) {
                            j[row * n + col] += d;
                        }
                    }
                }
            }

            // Converge on the current residual (KCL must balance to a tiny
            // current at every node).  This is robust to ill-scaling where a
            // small residual still implies a large raw Newton step.
            let max_f = f.iter().fold(0.0_f64, |m, &x| m.max(x.abs()));
            if max_f < 1e-9 {
                converged = true;
                if std::env::var("PK_BJTDC_DEBUG").is_ok() {
                    eprintln!("[PK_BJTDC]   converged at iter {_iter} (max|f|={max_f:.3e})");
                }
                break;
            }

            // Solve J·Δ = -f.
            let mut neg_f: Vec<f64> = f.iter().map(|x| -x).collect();
            let delta = match solve_dense_linear(&mut j, &mut neg_f, n) {
                Some(d) => d,
                None => return None,
            };

            // Damped update: limit any single node move to 0.25 V so junction
            // exponentials never blow up between iterations (SPICE-style voltage
            // limiting).  Scale the whole step by one factor to preserve direction.
            let max_raw = delta.iter().fold(0.0_f64, |m, &x| m.max(x.abs()));
            let scale = if max_raw > 0.25 { 0.25 / max_raw } else { 1.0 };
            for k in 0..n {
                v[k] += delta[k] * scale;
            }
            if !v.iter().all(|x| x.is_finite()) {
                return None;
            }
        }
        converged
    };
    if !converged {
        if std::env::var("PK_BJTDC_DEBUG").is_ok() {
            eprintln!("[PK_BJTDC] no-converge interior_n={n} n_resistors={}", resistors.len());
        }
        return None;
    }

    // Physicality check + assemble the solved node-voltage map.  Reject if any
    // BJT lands non-physical (Vbe outside a plausible silicon window or Vce
    // saturated/negative) so we never seed a worse point than the linear
    // fallback.  Ceiling from the ACTUAL solved VCC rail (the `supply_voltage`
    // param is the builder's inner default, e.g. 9 V, which need not match the
    // graph's rail — BA283 runs on +24 V via `supply 24V`).
    let vcc_ceiling = rail_v(graph.vcc_node)
        .unwrap_or(supply_voltage)
        .max(supply_voltage);
    let mut any_active = false;
    for b in &bjts {
        let vb_ = node_voltage(b.base, &v);
        let vc_ = node_voltage(b.collector, &v);
        let ve_ = node_voltage(b.emitter, &v);
        let sign = if b.is_npn { 1.0 } else { -1.0 };
        let vbe = sign * (vb_ - ve_);
        let vce = sign * (vc_ - ve_);
        if std::env::var("PK_BJTDC_DEBUG").is_ok() {
            eprintln!(
                "[PK_BJTDC]   {} npn={} Vbe={vbe:.4} Vce={vce:.4} (V b={vb_:.4} c={vc_:.4} e={ve_:.4})",
                b.model_name, b.is_npn
            );
        }
        if !vbe.is_finite() || !vce.is_finite() {
            return None;
        }
        // Reject clearly non-physical points; a slightly-saturated output
        // Darlington is acceptable (its Vce can be small), but a junction biased
        // well past turn-on or reverse-biased Vce is a failed solve.
        if vbe > 1.2 || vce < -0.5 || vce > vcc_ceiling + 1.0 {
            return None;
        }
        if (0.3..=1.0).contains(&vbe) && vce > 0.05 {
            any_active = true;
        }
    }
    if !any_active {
        return None;
    }

    // Return the solved DC node voltages.  Include the rail nodes that BJT
    // terminals touch (VCC / GND / supplies) so the caller can resolve a port
    // like (collector=vcc, emitter=interior) to a true Vce.
    let mut node_dc: std::collections::HashMap<NodeId, f64> = std::collections::HashMap::new();
    for (i, &nd) in interior.iter().enumerate() {
        node_dc.insert(nd, v[i]);
    }
    for b in &bjts {
        for &nd in &[b.base, b.collector, b.emitter] {
            if let Some(rv) = rail_v(nd) {
                node_dc.insert(nd, rv);
            }
        }
    }
    Some(node_dc)
}

/// Apply the solved BJT DC operating point to a just-built grouped-BJT
/// `MultiNlStage`.
///
/// Two things are seeded so the runtime NR HOLDS the nonlinear Q-point instead
/// of the linear vcc-injection set-point:
///
/// 1. **NR warm-start** — `v_prev` / `initial_v_prev` / `v_prev_2` at each port
///    are set to the solved port voltage `v* = V(pos) − V(neg)` (from the DC
///    node-voltage map), so the first sample starts at the operating point.
///
/// 2. **DC source term** — `dc_bias[i]` is the DC component of the port's
///    *incident wave*, NOT the port voltage.  The runtime NR residual is
///    `F_i = known_a_i − v_i − R_i·i_i(v) + Σ_j S[i][j]·(v_j − R_j·i_j(v))`, and
///    at DC steady state `known_a_i = dc_bias[i]` (the AC drive and the open-at-DC
///    coupling caps contribute nothing).  Inverting at the solved operating point
///    `v*` gives the exact incident wave that makes the NR settle there:
///    `dc_bias[i] = v_i* + R_i·i_i(v*) − Σ_j S[i][j]·(v_j* − R_j·i_j(v*))`.
///    This is coupling-aware (uses the stage's own `s_nl` + `nl_port_resistances`
///    + the grouped device currents), so it reproduces the feedback-servo bias
///    that the linear superposition could not.
///
/// Port voltages are read from `nl_terminals` (built in port order alongside
/// `dc_bias`), so this is robust to the device-group ordering.  Ports whose
/// terminal nodes are not in the solved DC map (e.g. behind a coupling cap) are
/// left at their existing linear values.
fn apply_bjt_dc_qpoint(
    stage: &mut MultiNlStage,
    node_dc: &std::collections::HashMap<NodeId, f64>,
    nl_terminals: &[(NodeId, NodeId)],
    nl_kinds: &[NonlinearKind],
    reactive_edges: &[(usize, OnePortKind)],
    graph: &CircuitGraph,
) {
    let n_nl = stage.dc_bias.len();
    if n_nl == 0 || nl_terminals.len() < n_nl {
        return;
    }

    // Voltage at a node: solved DC map first, else fall back to the runtime
    // stage's recorded supply / ground rails via the graph-independent map.
    // `node_dc` already excludes rails, so a missing node is a rail (or floating);
    // we resolve rails from the values stamped on the BJT terminals during the
    // solve by reusing the map plus a 0-default for ground-like nodes.
    let solved = |node: NodeId| -> Option<f64> { node_dc.get(&node).copied() };

    // Target port voltage v* for every NL port that has a fully-solved terminal
    // pair.  `None` => leave that port's linear dc_bias untouched.
    let mut v_star: Vec<Option<f64>> = vec![None; n_nl];
    // We also need the absolute DC voltage at *both* terminals of a port to know
    // whether the port is a BJT port at all; only BJT be/ce ports are remapped.
    let mut is_bjt_port = vec![false; n_nl];

    // Mark BJT ports (be, ce) in port order, mirroring `classify_nl_devices`.
    {
        let mut port_idx = 0usize;
        for kind in nl_kinds {
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
                } if base_node != collector_node => {
                    if port_idx + 1 < n_nl {
                        is_bjt_port[port_idx] = true;
                        is_bjt_port[port_idx + 1] = true;
                    }
                    port_idx += 2;
                }
                NonlinearKind::Triode {
                    grid_node: Some(_),
                    ..
                }
                | NonlinearKind::Pentode {
                    grid_node: Some(_),
                    ..
                } => {
                    port_idx += 2;
                }
                _ => {
                    port_idx += 1;
                }
            }
        }
    }

    for i in 0..n_nl {
        if !is_bjt_port[i] {
            continue;
        }
        let (pos, neg) = nl_terminals[i];
        // Resolve each terminal: solved interior node, or a rail (gnd→0,
        // vcc/supply → solved value isn't stored, so treat as the port's existing
        // behaviour by requiring at least one solved terminal and using the map
        // delta).  We only remap when BOTH terminals resolve to a DC voltage.
        let vp = solved(pos);
        let vn = solved(neg);
        if let (Some(vp), Some(vn)) = (vp, vn) {
            v_star[i] = Some(vp - vn);
        } else if let Some(vp) = vp {
            // neg is a rail not in the map (gnd ⇒ 0).  Approximate with vp − 0.
            v_star[i] = Some(vp);
        } else if let Some(vn) = vn {
            v_star[i] = Some(-vn);
        }
    }

    if std::env::var("PK_BJTDC_DEBUG").is_ok() {
        eprintln!(
            "[PK_BJTDC] apply: n_nl={n_nl} nl_terminals.len()={} is_bjt_port={:?} v_star={:?}",
            nl_terminals.len(),
            is_bjt_port,
            v_star
        );
    }
    // The DC-bias seed (`dc_qpoint_v`) is a FULL per-port vector consumed by the
    // runtime inversion, which re-derives EVERY port.  To stay byte-identical on
    // mixed stages, only engage the seed when every NL port resolved a target
    // (true for an all-BJT group like the BA283); otherwise fall back to a
    // build-time-only override of just the BJT ports.
    let all_resolved = v_star.iter().all(|o| o.is_some());
    if !all_resolved {
        if v_star.iter().all(|o| o.is_none()) {
            return;
        }
        // Partial: write each resolved BJT port's warm-start; leave dc_bias as the
        // linear term (the safe pre-existing behaviour for unsupported mixes).
        for i in 0..n_nl {
            if let Some(v) = v_star[i] {
                let w = v as pedalkernel_rt::Wave;
                if i < stage.v_prev.len() {
                    stage.v_prev[i] = w;
                }
                if i < stage.initial_v_prev.len() {
                    stage.initial_v_prev[i] = w;
                }
                if i < stage.v_prev_2.len() {
                    stage.v_prev_2[i] = w;
                }
            }
        }
        return;
    }

    // 1. Warm-start the NR at the solved operating point.
    let v_seed: Vec<pedalkernel_rt::Wave> =
        v_star.iter().map(|o| o.unwrap() as pedalkernel_rt::Wave).collect();
    for i in 0..n_nl {
        let w = v_seed[i];
        if i < stage.v_prev.len() {
            stage.v_prev[i] = w;
        }
        if i < stage.initial_v_prev.len() {
            stage.initial_v_prev[i] = w;
        }
        if i < stage.v_prev_2.len() {
            stage.v_prev_2[i] = w;
        }
    }

    // 2a. Passive-port DC reflected waves: at DC each coupling cap settles to its
    //     node-voltage difference (rails resolved: gnd→0, vcc/supply→rail).  A cap
    //     whose terminal is a floating signal node (behind another coupling cap,
    //     e.g. the input/output) has no DC current and contributes 0.  These feed
    //     the runtime inversion's passive-subtraction term so the seeded dc_bias
    //     is exact.  `reactive_edges[k]` ↔ passive port k (build order).
    let resolve = |node: NodeId| -> Option<f64> {
        if node == graph.gnd_node || graph.ac_ground_nodes.contains(&node) {
            Some(0.0)
        } else if node == graph.vcc_node {
            graph
                .supply_voltages
                .get(&graph.vcc_node)
                .copied()
                .or(Some(0.0))
        } else if let Some(&v) = graph.supply_voltages.get(&node) {
            Some(v)
        } else if graph.supply_nodes.contains(&node) {
            None // unknown supply value here; treat as no DC contribution
        } else {
            node_dc.get(&node).copied()
        }
    };
    let mut passive_b: Vec<pedalkernel_rt::Wave> = Vec::with_capacity(reactive_edges.len());
    for (eidx, _) in reactive_edges {
        let e = &graph.edges[*eidx];
        let v = match (resolve(e.node_a), resolve(e.node_b)) {
            (Some(va), Some(vb)) => va - vb,
            // A terminal with no resolved DC voltage (floating) ⇒ cap carries no
            // DC ⇒ zero reflected-wave contribution.
            _ => 0.0,
        };
        passive_b.push(v as pedalkernel_rt::Wave);
    }

    // 2b. Pre-charge each coupling cap to its DC voltage so `b_passive[k]` starts
    //     at the value the inversion assumed (passive_b[k]).  Without this the caps
    //     start at 0 V, the first-sample `known_a` is wrong, and the NR falls into
    //     the cutoff fixed point before the caps can charge — exactly what the
    //     triode path does for the cathode bypass cap.  `reactive_edges[k]` ↔
    //     `passive_one_ports[k]` (same build order).
    for (k, _) in reactive_edges.iter().enumerate() {
        if let (Some(&one_port), Some(&bk)) =
            (stage.passive_one_ports.get(k), passive_b.get(k))
        {
            // Seed the cap's *reflected wave* directly to `passive_b[k]` (the DC
            // steady-state value the inversion assumed).  `wdf_set_incident` sets
            // `wave_state = incident`, and `wdf_reflected` returns `wave_state`, so
            // the first sample sees b_passive[k] = passive_b[k] exactly.  (The
            // `CapacitorVoltage` setter instead yields 2·v on a fresh cap, which
            // over-drives the first known_a and kicks the NR out of the active
            // basin.)
            one_port.wdf_set_incident(bk, &mut stage.passive_runtime_state);
        }
    }

    // 2c. Store the seed and derive dc_bias from it (coupling-aware inversion).
    //     The runtime re-runs the SAME derivation after any scattering recompute
    //     (e.g. a feedback-pot move), so the servo bias survives pot changes.
    stage.dc_qpoint_v = Some(v_seed);
    stage.dc_qpoint_passive_b = passive_b;
    stage.apply_dc_qpoint_seed();

    // Skip the DC ramp: the operating point is already seeded into v_prev and
    // dc_bias, so ramping dc_bias up from 0 would pull the NR back through the
    // cold/cutoff basin (the BA283 servo has both a cutoff and an active fixed
    // point — the warm-start must keep it in the active basin from sample 0).
    stage.dc_ramp = 256;
    stage.initial_dc_ramp = 256;

    if std::env::var("PK_BJTDC_DEBUG").is_ok() {
        eprintln!("[PK_BJTDC] v_star={v_star:?}");
        eprintln!("[PK_BJTDC] dc_bias after seed = {:?}", stage.dc_bias);
    }
}

/// Dense Gaussian elimination with partial pivoting (row-major `a`, in-place).
/// Returns the solution to `a·x = b`, or `None` if singular.
fn solve_dense_linear(a: &mut [f64], b: &mut [f64], n: usize) -> Option<Vec<f64>> {
    for col in 0..n {
        // Partial pivot.
        let mut pivot = col;
        let mut best = a[col * n + col].abs();
        for row in (col + 1)..n {
            let val = a[row * n + col].abs();
            if val > best {
                best = val;
                pivot = row;
            }
        }
        if best < 1e-18 {
            return None;
        }
        if pivot != col {
            for k in 0..n {
                a.swap(col * n + k, pivot * n + k);
            }
            b.swap(col, pivot);
        }
        let diag = a[col * n + col];
        for row in (col + 1)..n {
            let factor = a[row * n + col] / diag;
            if factor == 0.0 {
                continue;
            }
            for k in col..n {
                a[row * n + k] -= factor * a[col * n + k];
            }
            b[row] -= factor * b[col];
        }
    }
    // Back-substitution.
    let mut x = vec![0.0_f64; n];
    for row in (0..n).rev() {
        let mut sum = b[row];
        for k in (row + 1)..n {
            sum -= a[row * n + k] * x[k];
        }
        x[row] = sum / a[row * n + row];
    }
    if x.iter().all(|v| v.is_finite()) {
        Some(x)
    } else {
        None
    }
}
