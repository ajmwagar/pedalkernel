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
use std::collections::VecDeque;

use super::super::classify::NonlinearKind;
use super::super::component::EdgeKind;
use super::super::dyn_node::DynNode;
use super::super::graph::{CircuitGraph, NodeId};
use super::super::helpers::{gummel_poon_model, pentode_model, triode_model, vari_mu_model};
use super::super::stage::{
    DeviceIdentity, MultiNlDeviceGroups, MultiNlScattering, MultiNlStage, NlDeviceGroupKind,
    NlDeviceKind, ScatteringRecomputeData, NR_ITERATION_BUDGET,
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

    // Step 1b: Input-coupling-network inclusion (pedalkernel adapted-input-port).
    //
    // The WDF adapted (voltage-source) input port injects the signal at
    // `graph.in_node`. For a cap-coupled discrete amp the global input is the
    // OUTER plate of an input coupling cap (e.g. BA283 `Cin.a`), which is NOT a
    // member of this group's MNA — the coupling cap is a boundary edge that was
    // stripped during signal-flow partitioning. Injection then falls back to the
    // amplifier's base node, BYPASSING the input coupling cap + series base
    // resistor and injecting with a placeholder source impedance.
    //
    // Fix (Option B): pull the source node + its coupling cap INTO the group. We
    // locate the single passive (cap/resistor) edge that bridges `graph.in_node`
    // to a node already in this MNA, add `graph.in_node` as an MNA node, and add
    // that bridge edge to the passive edge set so it is stamped as a reactive
    // (or resistive) WDF port. The adapted VS port then lands at the TRUE source
    // node with the source's own (low) impedance, and the WDF scatters
    // source -> Cin -> Rin -> base by construction — correct for ANY input
    // network, not just a lumped series resistor.
    let mut passive_edges_owned: Vec<usize> = all_edges.to_vec();
    let mut input_coupling_included = false;
    if !node_set.contains(&graph.in_node)
        && graph.in_node != graph.gnd_node
        && graph.in_node != graph.vcc_node
    {
        let edge_set: HashSet<usize> = all_edges.iter().copied().collect();
        // The bridge edge: incident to in_node, other terminal already in MNA,
        // and a 2-terminal passive (capacitor preferred — the coupling cap).
        let bridge = graph
            .edges
            .iter()
            .enumerate()
            .filter(|(eidx, e)| {
                !edge_set.contains(eidx)
                    && (e.node_a == graph.in_node || e.node_b == graph.in_node)
            })
            .filter_map(|(eidx, e)| {
                let other = if e.node_a == graph.in_node {
                    e.node_b
                } else {
                    e.node_a
                };
                if !node_set.contains(&other) {
                    return None;
                }
                let comp = &graph.components[e.comp_idx];
                // Only coupling caps / series resistors — never an active device.
                let is_passive_bridge =
                    comp.kind.capacitance().is_some() || comp.kind.resistance().is_some();
                is_passive_bridge.then_some(eidx)
            })
            // Prefer a capacitor (the DC-blocking coupling cap) if several exist.
            .min_by_key(|&eidx| {
                let comp = &graph.components[graph.edges[eidx].comp_idx];
                if comp.kind.capacitance().is_some() {
                    0
                } else {
                    1
                }
            });
        if let Some(bridge_eidx) = bridge {
            node_set.push(graph.in_node);
            passive_edges_owned.push(bridge_eidx);
            input_coupling_included = true;
        }
    }
    let passive_edges: &[usize] = &passive_edges_owned;

    // Adapted (input voltage-source) port reference resistance = the source's own
    // (low/known) impedance. When we pulled the true global source node + its
    // coupling cap into the group (`input_coupling_included`), the adapted port
    // sits at the actual signal source, which is near-ideal (low Z) — use a small
    // line-source impedance instead of the legacy 1 kΩ placeholder, so the WDF
    // input divider (source -> Cin -> Rin -> base) is not artificially loaded.
    // Inter-stage / boundary-fallback adapted ports keep the legacy 1 kΩ (their
    // true driving-point impedance is the upstream stage, not a known source), so
    // no existing non-cap-coupled circuit changes.
    let adapted_source_r: f64 = if input_coupling_included {
        std::env::var("PK_RADAPT")
            .ok()
            .and_then(|s| s.parse::<f64>().ok())
            .unwrap_or(50.0)
    } else {
        1000.0
    };

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

    // ── Transformer stamp plan (pedalkernel-ffkl) ────────────────────────────
    // Plain two-port transformers among this build's passive edges. Before
    // this plan existed the transformer's primary edge fell through EVERY
    // branch of `stamp_passive_edges` (a transformer has no scalar
    // resistance/capacitance/inductance) and the component silently vanished
    // from the compiled stage — the LA-2A `T_out` drop.
    //
    // Each transformer is stamped one of two ways:
    // * FULL skeleton (`stamp_linear_transformer_skeleton`, the same
    //   machinery `build_passive_rtype_stage` uses: DCR + leakage +
    //   magnetizing + ideal turns-ratio branch) when its secondary
    //   PARTICIPATES in this MNA — both winding ends are ground or solved
    //   nodes (e.g. the fused secondary load of the C4 reflection). 4
    //   internal nodes + 2 voltage sources.
    // * MAGNETIZING-ONLY branch (primary DCR in series with the full primary
    //   inductance) when the secondary does not participate: with no
    //   secondary loop the ideal branch carries zero current, so from the
    //   primary's terminals the transformer IS its magnetizing inductance.
    //   Stamping the full skeleton here would instead SHORT the dangling
    //   winding to the MNA ground reference. 1 internal node, no sources.
    struct TransformerStampPlan {
        comp_idx: usize,
        primary_eidx: usize,
        internal_base: usize,
        /// `Some(vsrc_base)` → full skeleton; `None` → magnetizing-only.
        vsrc_base: Option<usize>,
    }
    let mut transformer_plans: Vec<TransformerStampPlan> = Vec::new();
    let mut transformer_edge_set: HashSet<usize> = HashSet::new();
    {
        let is_gnd =
            |n: NodeId| n == graph.gnd_node || graph.ac_ground_nodes.contains(&n);
        let mut seen: HashSet<usize> = HashSet::new();
        let mut internal_next = num_mna_nodes;
        for &eidx in passive_edges {
            let comp_idx = graph.edges[eidx].comp_idx;
            let comp = &graph.components[comp_idx];
            let Some(cfg) = comp.kind.transformer_config() else {
                continue;
            };
            if cfg.has_tertiary() {
                // Three-winding transformers are not supported by this
                // builder; the edge deliberately falls to the guard below
                // (loud) instead of being silently dropped.
                continue;
            }
            transformer_edge_set.insert(eidx);
            if !seen.insert(comp_idx) {
                continue;
            }
            let winding_in_mna = |pin: &str| -> bool {
                graph
                    .node_names
                    .get(&format!("{}.{pin}", comp.id))
                    .map(|&n| is_gnd(n) || node_to_mna(n).is_some())
                    .unwrap_or(false)
            };
            let full = winding_in_mna("c") && winding_in_mna("d");
            let (internal_base, vsrc_base) = if full {
                let base = internal_next;
                internal_next += 4;
                let vsrc = num_vsources;
                num_vsources += 2;
                (base, Some(vsrc))
            } else {
                let base = internal_next;
                internal_next += 1;
                (base, None)
            };
            transformer_plans.push(TransformerStampPlan {
                comp_idx,
                primary_eidx: eidx,
                internal_base,
                vsrc_base,
            });
        }
    }
    let total_mna_nodes = num_mna_nodes
        + transformer_plans
            .iter()
            .map(|p| if p.vsrc_base.is_some() { 4 } else { 1 })
            .sum::<usize>();

    let nl_edge_set: HashSet<usize> = all_edges
        .iter()
        .filter(|&&eidx| graph.effective_edge_kind(eidx) == EdgeKind::Nonlinear)
        .copied()
        .collect();

    let (mut mna, reactive_edges, variable_resistor_candidates, dropped_edges) =
        stamp_passive_edges(
            passive_edges,
            &nl_edge_set,
            graph,
            &node_to_mna,
            total_mna_nodes,
            num_vsources,
            effective_rate,
            vcc_vs_idx,
            &vcvs_vsrc_base,
            &transformer_edge_set,
        );

    // ── Edge guard (pedalkernel-ffkl): a builder must not silently drop ──
    // Every edge handed to this build is stamped into the MNA, lowered to a
    // WDF/one-port, registered as a variable resistor, or explicitly consumed
    // (NL ports, VCVS, transformer plan). Anything else is a formation bug.
    super::super::spqr_build::report_dropped_edges(
        "general MNA",
        &dropped_edges,
        graph,
        super::super::spqr_build::EdgeGuardMode::Error,
    )?;

    // Stamp the planned transformers + collect their synthetic reactive
    // one-ports (leakage / magnetizing inductors, inter-winding capacitance).
    let mut synthetic_one_ports: Vec<(WdfPort, OnePortKind)> = Vec::new();
    for plan in &transformer_plans {
        let comp = &graph.components[plan.comp_idx];
        let cfg = comp
            .kind
            .transformer_config()
            .expect("planned transformer has a config");
        let resolved_cfg = crate::model_lookup::transformer_config_from_dsl(cfg);
        let e = &graph.edges[plan.primary_eidx];
        let p1 = node_to_mna(e.node_a);
        let p2 = node_to_mna(e.node_b);
        match plan.vsrc_base {
            Some(vsrc_p) => {
                let sec_mna = |pin: &str| -> Option<usize> {
                    graph
                        .node_names
                        .get(&format!("{}.{pin}", comp.id))
                        .and_then(|&n| node_to_mna(n))
                };
                let internals = [
                    plan.internal_base,
                    plan.internal_base + 1,
                    plan.internal_base + 2,
                    plan.internal_base + 3,
                ];
                let dynamic = super::super::spqr_build::stamp_linear_transformer_skeleton(
                    &mut mna,
                    &comp.id,
                    &resolved_cfg,
                    p1,
                    p2,
                    sec_mna("c"),
                    sec_mna("d"),
                    internals,
                    vsrc_p,
                    effective_rate,
                    resolved_cfg.dc_bias_current,
                );
                for (port, child) in dynamic {
                    let Some(kind) =
                        one_port_kind_for_transformer_child(&child, effective_rate)
                    else {
                        return Err(format!(
                            "transformer {}: unsupported dynamic child in general MNA",
                            comp.id
                        ));
                    };
                    synthetic_one_ports.push((port, kind));
                }
            }
            None => {
                let l_p = resolved_cfg.primary_inductance.max(1.0e-12);
                let mid = Some(plan.internal_base);
                mna.stamp_resistor(p1, mid, resolved_cfg.primary_dcr.max(1.0e-6));
                let rp = 2.0 * effective_rate * l_p;
                synthetic_one_ports.push((
                    WdfPort {
                        node_pos: mid,
                        node_neg: p2,
                        resistance: rp,
                    },
                    OnePortKind::Inductor(l_p),
                ));
            }
        }
    }

    // Step 4: Build WDF ports
    let (
        mut ports,
        port_node_pairs,
        passive_children,
        mut nl_port_resistances,
        adapt_input_vs_port,
    ) = build_wdf_ports(
        &nl_terminals,
        &nl_kinds,
        &reactive_edges,
        &synthetic_one_ports,
        graph,
        &node_to_mna,
        n_nl,
        passive_edges,
        effective_rate,
        adapted_source_r,
    )?;
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
        eprintln!(
            "[PK9XU1] num_mna_nodes={} num_vsources={} n_nl={}",
            num_mna_nodes, num_vsources, n_nl
        );
        eprintln!("[PK9XU1] vcvs_vsrc_base={:?}", vcvs_vsrc_base);
        for (i, &n) in node_set.iter().enumerate() {
            eprintln!("[PK9XU1]   mna node[{}] = graph {} ({})", i, n, nm(n));
        }
        eprintln!(
            "[PK9XU1] in_node={} ({})  out_node={} ({})",
            graph.in_node,
            nm(graph.in_node),
            graph.out_node,
            nm(graph.out_node)
        );
        eprintln!(
            "[PK9XU1] extract_output_node={:?} ({})",
            extract_output_node_id,
            extract_output_node_id.map(nm).unwrap_or("none")
        );
        eprintln!(
            "[PK9XU1] injection (in_node mna)={:?}",
            node_to_mna(graph.in_node)
        );
        for rec in &graph.nullor_pins {
            let in_stage = all_edges
                .iter()
                .any(|&e| graph.edges[e].comp_idx == rec.comp_idx);
            if in_stage {
                eprintln!(
                    "[PK9XU1] nullor pos={}({}) neg={}({}) out={}({})",
                    rec.pos_node,
                    nm(rec.pos_node),
                    rec.neg_node,
                    nm(rec.neg_node),
                    rec.out_node,
                    nm(rec.out_node)
                );
            }
        }
    }

    // Step 5: Derive scattering matrix + Thevenin adaptation
    let (scattering, vcc_injection_vec) = derive_scattering(
        &mna,
        &mut ports,
        &mut nl_port_resistances,
        n_nl,
        vcc_vs_idx,
        adapt_input_vs_port,
    )?;

    let n_total = ports.len();

    // Step 6: DC bias from VCC injection
    let (dc_bias, vcc_bias_all) = compute_dc_bias(
        vcc_injection_vec.as_deref(),
        &nl_kinds,
        n_nl,
        supply_voltage,
    );

    // Step 7: Create NL device groups (threading per-device component identity
    // — ref + model type — so the runtime op-point readout names the physical
    // transistor, never a guessed port index).
    let (nl_devices, device_groups) =
        create_nl_devices(&nl_kinds, &nl_comp_labels, supply_voltage)?;

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
        adapted_source_r,
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
    // The DC operating-point COMPUTATION lives in `compiler::bias` — the single
    // bias-computation file (pedalkernel-kzla).  This module only APPLIES it.
    match super::super::bias::solve_triode_dc_qpoint(&nl_kinds, all_edges, graph, supply_voltage) {
        Ok(dc) => apply_triode_dc_qpoint(&mut stage, &dc, &nl_kinds, &reactive_edges, graph),
        // ko5g.3 warn-not-error: an undeterminable single-triode stage keeps
        // the legacy silent fallback — the LINEAR `compute_dc_bias`
        // superposition (Vgk ≈ 0 warm start) — but now says so loudly.
        // Multi-NL groups / pentodes / strapped triodes are NotApplicable
        // (no triode load line to solve) and stay silent.
        Err(skip) => skip.warn_if_undeterminable(
            "Stage keeps the linear dc-bias superposition (Vgk≈0 warm start, no Q-point precharge).",
        ),
    }

    // Step 9b: DC Q-point for single-pentode stages (ko5g.5 — the first
    // pentode bias solver).  Exactly the triode pattern above: solve the
    // self-bias load line (screen divider resolved at DC, shared-cathode
    // aware, OT-primary plate paths at DCR), then seed the NR warm-start,
    // the circuit-true Vg2 on the PentodeThreePort, and the cathode bypass
    // cap.  Undeterminable topologies — including the grounded-cathode
    // FIXED-BIAS deferral (the ko5g.2 la2a-V5 safeguard) — keep the legacy
    // defaults, loudly.
    match super::super::bias::solve_pentode_dc_qpoint(&nl_kinds, all_edges, graph, supply_voltage) {
        Ok(dc) => apply_pentode_dc_qpoint(&mut stage, &dc, &nl_kinds, &reactive_edges, graph),
        Err(skip) => skip.warn_if_undeterminable(
            "Stage keeps the linear dc-bias superposition (Vg1k≈0 warm start, \
             model-default Vg2, no Q-point precharge).",
        ),
    }

    // Step 10: DC Q-point for grouped-BJT stages (pedalkernel-kzla).
    //
    // Problem (BA283 trace y2wj / 685e / ej0v): the runtime grouped-NR seeds each
    // port's DC from `dc_bias[i]`, built by `compute_dc_bias` as a LINEAR
    // vcc-injection superposition. That linear solve cannot model a nonlinear
    // DC-coupled feedback servo: the BA283 TR1 base is biased through R2 56k from
    // the NFB bus, whose DC level is set by the conducting Darlington, so the
    // linear solve starves TR1 at the cutoff fixed point (Vbe≈0.36 V vs the
    // validated 0.61 V) → ~120× low gain → -49.5 dB.
    //
    // Fix (ej0v PROVED forcing TR1 to the SPICE op-point closes BA283 to ~0 dB):
    // `bias::solve_bjt_group_dc_qpoint` solves the NONLINEAR DC operating point of
    // the whole BJT group with a SOURCE-STEPPING HOMOTOPY that lands the
    // CONDUCTING fixed point (TR1 → Vbe≈0.61); `apply_bjt_dc_qpoint` injects it
    // via the exact wave-domain inversion (`apply_dc_qpoint_seed`).  Runs by
    // DEFAULT: non-BJT / degenerate groups return `None` and keep their linear
    // `dc_bias`, staying byte-identical.
    // pedalkernel-y9hz: a NAMED `init { Q: saturated/cutoff/... }` hint on any
    // device of this group is the asymmetric-OSCILLATOR seed path (cross-coupled
    // astables): the whole point is to start the NR AWAY from the network's
    // symmetric DC fixed point so the multivibrator breaks symmetry
    // deterministically. The group q-point seed would overwrite every port's
    // warm-start with that symmetric root (measured: bjt_astable initial_v_prev
    // with-hints == without-hints once the solver — post-129p — converges on
    // the astable's metastable symmetric root). Skip the seed and keep the
    // hinted state authoritative, mirroring the WDF call-site's has_hint guard.
    // Explicit `op { }` seeds re-override AFTER this in apply_op_seed (Step 11)
    // and need no gate.
    let has_named_bjt_hint = init_hints.iter().any(|h| {
        matches!(h.state, crate::dsl::InitState::Named(_))
            && nl_comp_labels.iter().any(|l| l == &h.device_label)
    });
    if !has_named_bjt_hint {
        // pedalkernel-onu2 (RCA GAPs 2a+4a on pedalkernel-a5ho): the DC solve
        // must see the whole DC-CLOSURE of the group's BJT terminals, not the
        // group-local edges alone — a bias network split across flow groups
        // (sunflower's downstream collector chain, the uberdrive/LGSM `vref`
        // divider) otherwise floats on gmin and the homotopy converges a
        // saturated / cutoff artifact. This mirrors the blockwise caller,
        // which already passes the whole plan's edges
        // (`solve_blockwise_bjt_group_qpoint`). vref-class nodes crossed by
        // the closure (ac-ground-classified bias dividers) are solved as
        // interior instead of a 0 V rail. A group whose bias network is
        // group-local gets the exact legacy edge list + empty overrides
        // (closure adds nothing) and stays bit-for-bit identical.
        let (dc_closure_edges, vref_overrides) = super::super::bias::bjt_dc_closure_edges(
            &nl_kinds,
            all_edges,
            graph,
            supply_voltage,
        );
        // The single-BJT seed-BAKE (below, in `apply_bjt_dc_qpoint`) is gated
        // NARROWLY on a vref-class override actually being needed — NOT merely
        // on the closure having grown. A grown closure alone is the common case
        // (any single CE stage whose collector load returns to a rail through a
        // pot/resistor in another flow group — e.g. the Big Muff's Sustain-pot
        // Q3 collector chain): there the group-local linear vcc-injection
        // `dc_bias` ALREADY biases the stage functionally, and overriding it
        // with the (correct-but-near-saturation) closure op-point kills the
        // small-signal gain the runtime relies on. The vref-follower case (GAP
        // 4a) is different in kind: the base sits at a bypassed-divider virtual
        // ground that is BOTH in another flow group AND unrepresentable in the
        // runtime stage's own MNA, so without baking the seed the follower can
        // only ever read cutoff. `vref_overrides` non-empty is precisely that
        // signal. (The DC-closure EDGE SET improvement still applies to every
        // group — it only fixes what the solve SEES; the bake gate decides
        // whether we overwrite a working linear bias with it.)
        let bake_cross_group_seed = !vref_overrides.is_empty();
        if let Some(node_dc) = super::super::bias::solve_bjt_group_dc_qpoint_with_overrides(
            &nl_kinds,
            &dc_closure_edges,
            graph,
            supply_voltage,
            &vref_overrides,
        ) {
            apply_bjt_dc_qpoint(
                &mut stage,
                &node_dc,
                &nl_terminals,
                &nl_kinds,
                &reactive_edges,
                graph,
                bake_cross_group_seed,
            );
        }
    }

    // Step 11: explicit `op { }` operating-point seed (pedalkernel — seed-and-hold).
    //
    // This is the AUTHORITATIVE warm-start override: it runs LAST, after the
    // engine's own `apply_bjt_dc_qpoint` (which otherwise sets `v_prev` from the
    // engine's homotopy DC solve). It overrides ONLY the NR warm-start
    // (`v_prev` / `initial_v_prev` / `v_prev_2`) at the named device's ports and
    // deliberately leaves the stage's DC source term (`dc_bias` / `dc_qpoint_v`)
    // exactly as the engine computed it.
    //
    // That is the whole point: the stage's `dc_bias` defines WHERE the runtime
    // NR fixed point sits. Seeding `v_prev` at an externally-supplied operating
    // point (e.g. ngspice's `.op`) and then running silence answers a precise
    // question — does the NR HOLD the seed (the seed IS a root of the engine's
    // residual ⇒ bias math agrees, cold-solve is a basin/root-selection issue)
    // or WALK back to the engine's own root (the seed is NOT a root ⇒ the
    // engine's bias math genuinely differs from spice). Re-deriving `dc_bias`
    // from the seed would force it to be a fixed point and make the test vacuous,
    // so we do not.
    apply_op_seed(&mut stage, init_hints, &nl_comp_labels);

    // Step 12: reactive-port (capacitor) seed from `op { nodes { ... } }`.
    // Seeds each WDF capacitor's stored DC voltage to `V(node_a) − V(node_b)`
    // from the supplied node-voltage map, completing a FULL operating-point seed
    // (device ports via Step 11 + cap voltages here). No-op without node hints.
    apply_cap_seed(&mut stage, init_hints, &reactive_edges, graph);

    Ok(stage)
}

/// Step 12 helper: seed reactive (capacitor) one-port voltages from the
/// `op { nodes { ... } }` node-voltage map.
///
/// `reactive_edges[k]` is 1:1 with `stage.passive_one_ports[k]` (both built in
/// the same order by `build_wdf_ports`). For each capacitor, the stored DC
/// voltage is `V(node_a) − V(node_b)`, read from the node-voltage hints keyed by
/// `.pedal` node NAME (a pin name like `Q3.base`, or a reserved node like
/// `in`/`vcc`). Ground defaults to 0 V. Caps whose terminals are not both in the
/// map are left at their existing (engine-DC-solved) state.
///
/// This is the reusable cap half of a full operating-point seed: a future
/// compile-time DC solver can call the same path with its solved node vector.
/// No-op when no `NodeVoltage` hints are present (byte-identical to before).
fn apply_cap_seed(
    stage: &mut MultiNlStage,
    init_hints: &[crate::dsl::InitHint],
    reactive_edges: &[(usize, OnePortKind)],
    graph: &CircuitGraph,
) {
    use std::collections::HashMap;
    // name → DC voltage, from the node-voltage hints only.
    let mut name_v: HashMap<&str, f64> = HashMap::new();
    for h in init_hints {
        if let crate::dsl::InitState::NodeVoltage { v } = &h.state {
            name_v.insert(h.device_label.as_str(), *v);
        }
    }
    if name_v.is_empty() {
        return;
    }

    // Resolve to NodeId → voltage via the graph's pin-name map.
    let mut node_v: HashMap<NodeId, f64> = HashMap::new();
    for (name, v) in &name_v {
        if let Some(&nid) = graph.node_names.get(*name) {
            node_v.insert(nid, *v);
        } else {
            eprintln!("[cap-seed] node name {name:?} not found in graph — ignored");
        }
    }

    let voltage_at = |node: NodeId| -> Option<f64> {
        if node == graph.gnd_node {
            Some(0.0)
        } else {
            node_v.get(&node).copied()
        }
    };

    let mut any_seeded = false;
    for (k, (eidx, kind)) in reactive_edges.iter().enumerate() {
        if !matches!(kind, OnePortKind::Capacitor(_)) {
            continue;
        }
        let e = &graph.edges[*eidx];
        let (Some(va), Some(vb)) = (voltage_at(e.node_a), voltage_at(e.node_b)) else {
            continue;
        };
        let v_cap = va - vb;
        if let Some(&one_port) = stage.passive_one_ports.get(k) {
            let seeded =
                one_port.wdf_seed_dc_voltage(v_cap as pedalkernel_rt::Wave, &mut stage.passive_runtime_state);
            if seeded {
                any_seeded = true;
                eprintln!(
                    "[cap-seed] reactive port {k} (edge {eidx}): V = {va:.4} − {vb:.4} = {v_cap:+.4} V"
                );
            }
        }
    }

    // Make the seed SELF-CONSISTENT (Part 2 for the seeded op): `dc_bias` was
    // derived (`apply_dc_qpoint_seed`) against the compiler's own DC cap solve
    // (`dc_qpoint_passive_b`). Overriding the cap wave_state to ngspice's
    // node voltages without re-deriving `dc_bias` leaves a spurious residual
    // `Σ_k s_nl_passive[i][k]·(b_ngspice[k] − b_compiler[k])` in the per-sample
    // `known_a` — e.g. the input coupling cap Cin resolves to 0 V in the
    // compiler solve but −1.03 V in ngspice, injecting ~0.6 V of bogus residual
    // that prevents the grouped-NR from converging at the seeded op. Re-point
    // `dc_qpoint_passive_b` at the seeded (ngspice) cap reflected waves and
    // re-run the wave-domain inversion so the Norton `dc_bias` source term makes
    // `F(ngspice_op) ≈ 0`. Uses only `i(v*)` (no device small-signal Jacobians),
    // per the documented hard constraint. Only fires when `nodes{}` is present,
    // so production (no seed) is byte-identical.
    if any_seeded && stage.dc_qpoint_v.is_some() && !stage.dc_qpoint_passive_b.is_empty() {
        let n_pb = stage.dc_qpoint_passive_b.len();
        for k in 0..n_pb {
            if let Some(&one_port) = stage.passive_one_ports.get(k) {
                stage.dc_qpoint_passive_b[k] = one_port.wdf_reflected(&stage.passive_runtime_state);
            }
        }
        stage.apply_dc_qpoint_seed();
    }
}

/// Step 11 helper: apply explicit `op { }` operating-point seeds as the final
/// NR warm-start, overriding `v_prev` / `initial_v_prev` / `v_prev_2` only.
///
/// Maps each `InitState::Explicit` device label to its BJT device group's port
/// offsets (`off` = base-emitter port, `off + 1` = collector-emitter port) and
/// writes the seeded `Vbe` / `Vce` (sign-flipped for PNP). Leaves `dc_bias`,
/// `dc_qpoint_v`, and every passive cap state untouched. Devices without an
/// explicit hint, and `Named` hints, are skipped. No-op when there are no
/// explicit hints (the common case ⇒ byte-identical).
fn apply_op_seed(
    stage: &mut MultiNlStage,
    init_hints: &[crate::dsl::InitHint],
    nl_comp_labels: &[String],
) {
    let has_explicit = init_hints
        .iter()
        .any(|h| matches!(h.state, crate::dsl::InitState::Explicit { .. }));
    if !has_explicit {
        return;
    }
    let Some(dg) = stage.device_groups.as_ref() else {
        return;
    };
    // Snapshot (offset, is_pnp) per group before mutably borrowing the stage's
    // warm-start vectors.
    let mut seeds: Vec<(usize, f64, f64)> = Vec::new();
    for (g, group) in dg.groups.iter().enumerate() {
        let NlDeviceGroupKind::BjtTwoPort(bjt) = group else {
            continue;
        };
        let label = nl_comp_labels.get(g).map(|s| s.as_str()).unwrap_or("");
        let Some(hint) = init_hints.iter().find(|h| h.device_label == label) else {
            continue;
        };
        let crate::dsl::InitState::Explicit { vbe, vce } = &hint.state else {
            continue;
        };
        let sign = if bjt.is_pnp { -1.0 } else { 1.0 };
        seeds.push((dg.offsets[g], sign * *vbe, sign * *vce));
    }
    let n_nl = stage.n_nl;
    for (off, vbe, vce) in seeds {
        for (port, v) in [(off, vbe), (off + 1, vce)] {
            if port >= n_nl {
                continue;
            }
            let w = v as pedalkernel_rt::Wave;
            stage.v_prev[port] = w;
            stage.initial_v_prev[port] = w;
            stage.v_prev_2[port] = w;
        }
        eprintln!(
            "[op-seed/hold] port {off}: Vbe={:.4} Vce={:.4} (v_prev overridden post-apply_bjt_dc_qpoint)",
            vbe, vce
        );
    }

    // Self-validation: the op{} seed is, by construction, an externally-supplied
    // operating point. Compute and report the per-port WDF DC balance residual
    // F(seed) = a − dc_bias − cap − nl right here, as a byproduct of seeding.
    // A nonzero residual proves the seed is NOT a fixed point of OUR DC equations
    // (a formulation bug), and the four-term breakdown localizes which term is
    // off. Always emitted when an explicit op{} seed is present; also honors
    // PK_OP_RESIDUAL=1 for ad-hoc probing.
    //
    // NOTE: at this compile-time call the reactive (cap) ports are at their
    // freshly-built state, not yet DC-settled, so the `cap` column reflects the
    // initial wave state. For a DC-consistent cap term, settle silence first and
    // re-read via `MultiNlStage::op_seed_residual` (the ba283_seed_and_hold test
    // does exactly this).
    let report = has_explicit
        || std::env::var("PK_OP_RESIDUAL").map(|v| v == "1").unwrap_or(false);
    if report {
        let ports = stage.op_seed_residual();
        eprintln!(
            "[op-seed/residual] per-port WDF DC balance residual F(seed)=a-dc_bias-cap-nl \
             (nonzero ⇒ seed is NOT a fixed point of our DC eqns ⇒ formulation bug):"
        );
        let mut max_r = 0.0f64;
        for (i, p) in ports.iter().enumerate() {
            max_r = max_r.max(p.residual.abs());
            eprintln!(
                "  port {i:2}: a={:+.4} dc_bias={:+.4} cap={:+.4} nl={:+.4} adapt={:+.4} servo={:+.4}  i_dev={:+.3e}  static_r={:+.4} V",
                p.a, p.dc_bias, p.cap, p.nl, p.adapted, p.servo, p.i_device, p.residual
            );
        }
        eprintln!("[op-seed/residual] max|static residual| = {max_r:.4} V");
    }
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
    //
    // LATENT RISK (pedalkernel-pmv1): this `.find()` returns the FIRST nullor
    // in `graph.nullor_pins` declaration order whose component has an edge in
    // `all_edges`, not necessarily the one actually wired to `graph.out_node`.
    // The identical shape of this bug in `mna_builder.rs`'s `output_mna`/
    // `injection_mna` resolution caused BiValve SVF's LPF/HPF taps to collapse
    // to the same node for a multi-opamp (KHN/state-variable) group — fixed
    // there by trying the group-boundary-aware `mid_chain_terminals()` first.
    // This function is currently unreachable for that scenario (multi-opamp
    // linear groups route to Iir/StateSpace, not General — see
    // `rigid/mod.rs::classify_rigid`), so it has no failing test and is left
    // unchanged here per strict-TDD (no red test, no change). Tracked as a
    // follow-up: harden this fallback the same way if/when a reachable
    // multi-nullor General-MNA case appears (e.g. a multi-opamp NL group).
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
                nl_terminals.push((grid, cathode)); // port 0: grid-cathode
                nl_terminals.push((plate, cathode)); // port 1: plate-cathode
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
                nl_terminals.push((grid, cathode)); // port 0: grid-cathode
                nl_terminals.push((plate, cathode)); // port 1: plate-cathode
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

#[allow(clippy::too_many_arguments)] // focused stamping helper of the MNA build
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
    transformer_edge_set: &HashSet<usize>,
) -> (
    MnaSystem,
    Vec<(usize, OnePortKind)>,
    Vec<VariableResistorCandidate>,
    Vec<usize>,
) {
    let mut mna = MnaSystem::new(num_mna_nodes, num_vsources);
    let mut reactive_edges: Vec<(usize, OnePortKind)> = Vec::new();
    let mut variable_resistor_candidates: Vec<VariableResistorCandidate> = Vec::new();
    let mut stamped_vcvs: HashSet<usize> = HashSet::new();
    // Edge guard (pedalkernel-ffkl): edges that match NO branch below. The
    // caller decides how loud to be (`report_dropped_edges`).
    let mut dropped_edges: Vec<usize> = Vec::new();

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
            if let Some(&(_, vsrc_base)) = vcvs_vsrc_base.iter().find(|&&(ci, _)| ci == e.comp_idx)
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
        } else if transformer_edge_set.contains(&eidx) {
            // Stamped by the transformer skeleton plan in the caller
            // (pedalkernel-ffkl) — accounted for, not dropped.
        } else {
            // No branch stamped this edge: without the guard the component
            // would silently vanish from the compiled stage (the LA-2A
            // T_out failure mode).
            dropped_edges.push(eidx);
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

    (mna, reactive_edges, variable_resistor_candidates, dropped_edges)
}

/// Convert a transformer-skeleton dynamic child (built for the WDF passive
/// path) into the linear one-port kind the MultiNl stage's passive set
/// understands. The Jiles-Atherton magnetizing core degrades to its LINEAR
/// magnetizing inductance here (`L = rp / (2·fs)`, the same L its port
/// resistance encodes) — the MultiNl passive one-ports are linear reactive
/// elements. Plain-config transformers (no JA parameters) never produce a JA
/// child, so nothing degrades for them.
fn one_port_kind_for_transformer_child(
    child: &DynNode,
    effective_rate: f64,
) -> Option<OnePortKind> {
    match child {
        DynNode::Leaf(LeafKind::OnePort { runtime, .. }) => Some(runtime.spec.kind),
        DynNode::Leaf(LeafKind::JaMagnetizing(_)) => {
            let rp = child.port_resistance();
            Some(OnePortKind::Inductor(rp / (2.0 * effective_rate)))
        }
        _ => None,
    }
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

/// `true` when the group's audio-entry node REACHES (through in-group passive
/// edges only — a coupling cap / input trim pot / grid-stopper) the HIGH-Z
/// control input (base / grid) of a NONLINEAR active device, AND the group holds
/// ≥2 such coupled active devices (a coupled-NL group solved by grouped NR — the
/// Fuzz-Face 2-Ge pair). This is the discharge condition that admits the in-node
/// VS-port Thevenin adaptation for sunflower (pedalkernel-guwp) without touching
/// the inverting-op-amp+diode cases, which inject into a low-Z VIRTUAL-GROUND
/// summing pin (an op-amp/nullor pin, never a BJT base or tube grid) — so this
/// returns `false` for them.
///
/// Passive-only reachability (never crossing an active device itself, matching
/// `boundary_supersedes_amp_pin`) is what separates the two: sunflower's `in`
/// reaches `Q1.base` through CLEAN→C1 (both passive); the op-amp summing node
/// has NO passive path to any base/grid (there are none), so the predicate is
/// false.
///
/// Scoped to base/grid because those are the ports the compiler records with a
/// node (`BjtNpn/BjtPnp.base_node`, `Triode/Pentode.grid_node`); JFET/MOSFET
/// carry no node here and never form a coupled ≥2 active-input group in the
/// corpus, so they are excluded (and stay byte-identical).
fn injection_feeds_coupled_high_z_active_input(
    nl_kinds: &[NonlinearKind],
    injection_mna: Option<usize>,
    edge_indices: &[usize],
    graph: &CircuitGraph,
    node_to_mna: &dyn Fn(NodeId) -> Option<usize>,
) -> bool {
    let Some(inj) = injection_mna else {
        return false;
    };
    // High-Z control-input MNA nodes of the group's active NL devices.
    let mut control_mnas: Vec<usize> = Vec::new();
    for kind in nl_kinds {
        let control_node = match kind {
            NonlinearKind::BjtNpn { base_node, .. }
            | NonlinearKind::BjtPnp { base_node, .. } => Some(*base_node),
            NonlinearKind::Triode {
                grid_node: Some(grid),
                ..
            }
            | NonlinearKind::Pentode {
                grid_node: Some(grid),
                ..
            } => Some(*grid),
            _ => None,
        };
        if let Some(cn) = control_node {
            if let Some(m) = node_to_mna(cn) {
                control_mnas.push(m);
            }
        }
    }
    // Coupled-NL group requirement: ≥2 active devices with a high-Z control pin.
    if control_mnas.len() < 2 {
        return false;
    }
    if control_mnas.contains(&inj) {
        return true;
    }
    // Passive-only BFS from the injection node: does it reach ANY control pin
    // through in-group Passive edges (coupling cap / input trim pot / grid
    // stopper)? Never crosses an active device (so we can't trivially reach a
    // base through the transistor itself).
    let edge_set: HashSet<usize> = edge_indices.iter().copied().collect();
    let mut visited: HashSet<usize> = HashSet::new();
    visited.insert(inj);
    let mut queue: VecDeque<usize> = VecDeque::new();
    queue.push_back(inj);
    while let Some(m) = queue.pop_front() {
        for &eidx in &edge_set {
            let e = &graph.edges[eidx];
            let comp = &graph.components[e.comp_idx];
            if !matches!(
                comp.kind.signal_terminals(),
                super::super::component::SignalTerminals::Passive
            ) {
                continue;
            }
            let (ma, mb) = (node_to_mna(e.node_a), node_to_mna(e.node_b));
            let next = if ma == Some(m) {
                mb
            } else if mb == Some(m) {
                ma
            } else {
                None
            };
            if let Some(n) = next {
                if control_mnas.contains(&n) {
                    return true;
                }
                if visited.insert(n) {
                    queue.push_back(n);
                }
            }
        }
    }
    false
}

/// Step 4: Build WDF ports (NL + reactive + adapted input).
#[allow(clippy::too_many_arguments)] // focused port-assembly helper of the MNA build
fn build_wdf_ports(
    nl_terminals: &[(NodeId, NodeId)],
    nl_kinds: &[NonlinearKind],
    reactive_edges: &[(usize, OnePortKind)],
    synthetic_one_ports: &[(WdfPort, OnePortKind)],
    graph: &CircuitGraph,
    node_to_mna: &dyn Fn(NodeId) -> Option<usize>,
    n_nl: usize,
    edge_indices: &[usize],
    effective_rate: f64,
    r_adapted: f64,
) -> Result<
    (
        Vec<WdfPort>,
        Vec<WdfPortTerminals>,
        Vec<MnaOnePort>,
        Vec<f64>,
        // `true` when `derive_scattering` should ALSO Thevenin-adapt the input
        // (VS) port at index `n_total-1`. Two topologies qualify (see the gate
        // at pedalkernel-ayjt / pedalkernel-guwp):
        //   1. the adapted input port landed on an input BOUNDARY node that
        //      SUPERSEDED the amplifier device pin — the divider-fed high-Z
        //      follower case (uberdrive/lgsm); and
        //   2. `in_node` injection whose node is the HIGH-Z control input
        //      (base/grid) of a coupled ≥2-active-device NL group (the
        //      Fuzz-Face 2-Ge pair — sunflower).
        // `false` for the inverting op-amp + diode cases (`in_node` into a
        // virtual-ground summing pin, `amplifier-pin`, `boundary-fallback`),
        // which keep the VS port pinned at the placeholder (byte-identical to
        // base).
        bool,
    ),
    String,
> {
    let r_nl_default = 1000.0;
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

    // Synthetic passive one-ports (pedalkernel-ffkl): reactive ports that do
    // not correspond 1:1 to a graph edge — the transformer skeleton's leakage
    // and magnetizing inductors (their terminals may be transformer-internal
    // MNA nodes). Appended AFTER the edge-backed reactive ports so the
    // `reactive_edges[k] ↔ passive_one_ports[k]` positional contract that
    // `apply_triode_dc_qpoint` / `apply_cap_seed` rely on stays intact for
    // k < reactive_edges.len(), and BEFORE the adapted port which must stay
    // last.
    for (port, kind) in synthetic_one_ports {
        let (kind, port) = (*kind, port.clone());
        port_node_pairs.push(WdfPortTerminals::maybe_differential(
            port.node_pos,
            port.node_neg,
        ));
        passive_one_ports.push(MnaOnePort::new(
            MnaPortTerminals::maybe_differential(
                port.node_pos.map(MnaNodeId::new),
                port.node_neg.map(MnaNodeId::new),
            ),
            kind,
        ));
        ports.push(port);
    }

    // Adapted (input voltage source) port — where the audio ENTERS this group.
    // Choice is ordered by how unambiguously each candidate identifies the
    // physical entry node:
    //
    // 1. `graph.in_node` when it is a member of this MNA (possibly pulled in
    //    by the Option-B input-coupling inclusion above) — the true source.
    // 2. MAGNETIC entry (pedalkernel-n0yy): the input-nearest boundary node
    //    (directed signal distance) when it is a transformer WINDING whose
    //    sibling winding lies outside the group. A winding is the one
    //    unambiguous entry: the upstream stage drives this group through the
    //    transformer, and injecting at the device input pin instead silently
    //    bypasses any in-group network between the winding and the pin (the
    //    Pultec program EQ between `n_lf` and `V1.grid` — stamped in the G
    //    matrix, contributing nothing: present-but-inert).
    // 3. GALVANIC entry — legacy order preserved: the active element's
    //    signal input pin from `Component::signal_terminals()`, then the
    //    input-nearest boundary node (NL groups with no amplifier inside,
    //    e.g. a clipper + tone network split off an op-amp gain stage).
    //    Preferring the boundary node here too would be the fully general
    //    fix, but the entry-node choice interacts with the cross-stage
    //    WiperDivider contract (straddling pots would double-count) and
    //    with sidechain/feed-forward taps — measured on the LA-2A as a
    //    −28 dB level shift. Scoped out; see the pedalkernel-n0yy follow-up
    //    bead before widening.
    //
    // Guard: when the amplifier-pin fallback engages while the group HAS
    // boundary nodes but NONE is reachable from `in`, that is a reachability
    // gap of the present-but-inert family — diagnose via PK_INJECT_GUARD
    // (warn by default).
    //
    // Without any of these the adapted VS port is grounded and the stage is
    // structurally silent (s_nl_adapted = 0).
    let inject_debug = std::env::var("PK_INJECT_DEBUG").as_deref() == Ok("1");
    let name_of = |mna: Option<usize>| -> String {
        let Some(m) = mna else {
            return "<gnd>".to_string();
        };
        let names: Vec<&str> = graph
            .node_names
            .iter()
            .filter(|(_, &n)| node_to_mna(n) == Some(m))
            .map(|(k, _)| k.as_str())
            .collect();
        format!("mna#{m}[{}]", names.join(","))
    };
    // Gates the input-VS-port Thevenin adaptation in `derive_scattering`. Set
    // true in TWO cases: (a) the `in_node` injection lands on the HIGH-Z control
    // input (base/grid) of a coupled ≥2-active-device NL group — the Fuzz-Face
    // 2-Ge pair, sunflower (pedalkernel-guwp); (b) injection resolves via the
    // boundary-supersedes-amp-pin arm below — the divider-fed high-Z follower
    // (uberdrive/lgsm).
    let mut adapt_input_vs_port = false;
    let injection_mna = if let Some(m) = node_to_mna(graph.in_node) {
        if inject_debug {
            eprintln!("[PK_INJECT] branch=in_node -> {}", name_of(Some(m)));
        }
        // pedalkernel-guwp: a coupled-NL group whose audio entry IS a high-Z
        // active control input (base/grid) needs the injected source impedance
        // adapted to what that high-Z node actually sees (else the coupled NR
        // gets a mis-scaled source and produces ~0 AC gain — sunflower was
        // -135 dB / silent). Inverting op-amp+diode groups inject `in_node`
        // into a virtual-ground summing pin (never a base/grid) → predicate
        // false → byte-identical.
        if injection_feeds_coupled_high_z_active_input(
            nl_kinds,
            Some(m),
            edge_indices,
            graph,
            node_to_mna,
        ) {
            if inject_debug {
                eprintln!("[PK_INJECT]   in_node feeds coupled high-Z active input -> adapt VS port");
            }
            adapt_input_vs_port = true;
        }
        Some(m)
    } else {
        let boundary = find_input_boundary_node(edge_indices, graph, node_to_mna);
        if let Some((m, true)) = boundary.winner {
            if inject_debug {
                eprintln!("[PK_INJECT] branch=winding-boundary -> {}", name_of(Some(m)));
            }
            Some(m)
        } else {
            let amp_pin = edge_indices.iter().find_map(|&eidx| {
                let comp = &graph.components[graph.edges[eidx].comp_idx];
                match comp.kind.signal_terminals() {
                    super::super::component::SignalTerminals::Amplifier { input, .. } => {
                        let key = format!("{}.{}", comp.id, input);
                        graph.node_names.get(&key).and_then(|&n| node_to_mna(n))
                    }
                    _ => None,
                }
            });
            // Defect-2 precedence (pedalkernel-xki7): a reachable input BOUNDARY
            // supersedes the amplifier device pin when injecting at the pin
            // would bypass an in-group passive signal network (a BJT/FET
            // follower fed through a volume divider — uberdrive/lgsm). Scoped
            // to leave calibrated tube/photocoupler groups on the device pin.
            let amp_pin_node = edge_indices.iter().find_map(|&eidx| {
                let comp = &graph.components[graph.edges[eidx].comp_idx];
                match comp.kind.signal_terminals() {
                    super::super::component::SignalTerminals::Amplifier { input, .. } => {
                        graph.node_names.get(&format!("{}.{}", comp.id, input)).copied()
                    }
                    _ => None,
                }
            });
            let boundary_wins = match (amp_pin_node, boundary.winner, boundary.winner_node) {
                (Some(pin_n), Some((bm, _)), Some(bnode)) => {
                    boundary_supersedes_amp_pin(edge_indices, graph, pin_n, bnode)
                        .then_some(bm)
                }
                _ => None,
            };
            match (boundary_wins, amp_pin, boundary.winner) {
                (Some(bm), _, _) => {
                    if inject_debug {
                        eprintln!(
                            "[PK_INJECT] branch=boundary-supersedes-amp-pin -> {}",
                            name_of(Some(bm))
                        );
                    }
                    // Divider-fed high-Z follower: a reachable input boundary
                    // superseded the amplifier device pin because injecting at
                    // the pin would bypass an in-group passive divider. The VS
                    // port is Thevenin-adapted (magnitude fix), gated in
                    // derive_scattering (see also the coupled-NL in_node case).
                    adapt_input_vs_port = true;
                    Some(bm)
                }
                (None, Some(m), _) => {
                    if inject_debug {
                        eprintln!("[PK_INJECT] branch=amplifier-pin -> {}", name_of(Some(m)));
                    }
                    // In-loop op-amp nullor groups (pedalkernel-9xu1) inject
                    // at the virtual-ground pin BY DESIGN — the NL device
                    // co-solves against the closed-loop impedance, and the
                    // feedback topology means no boundary node is forward-
                    // reachable. Not a present-but-inert hazard: skip the
                    // guard.
                    let is_nullor_pin = graph
                        .nullor_pins
                        .iter()
                        .any(|rec| {
                            node_to_mna(rec.pos_node) == Some(m)
                                || node_to_mna(rec.neg_node) == Some(m)
                        });
                    if boundary.winner.is_none() && !is_nullor_pin {
                        report_unverified_injection(
                            &name_of(Some(m)),
                            boundary.has_boundary_nodes,
                        )?;
                    }
                    Some(m)
                }
                (None, None, Some((m, _))) => {
                    if inject_debug {
                        eprintln!("[PK_INJECT] branch=boundary-fallback -> {}", name_of(Some(m)));
                    }
                    Some(m)
                }
                (None, None, None) => {
                    report_unverified_injection("<none>", boundary.has_boundary_nodes)?;
                    None
                }
            }
        }
    };
    ports.push(WdfPort {
        node_pos: injection_mna,
        node_neg: None,
        resistance: r_adapted,
    });
    port_node_pairs.push(WdfPortTerminals::maybe_single_ended(injection_mna));

    Ok((
        ports,
        port_node_pairs,
        passive_one_ports,
        nl_port_resistances,
        adapt_input_vs_port,
    ))
}

/// Injection-node guard (pedalkernel-n0yy): the adapted VS port should land
/// on the group's input *boundary* node — the node the upstream stage
/// actually drives. When the boundary search comes back empty-handed even
/// though the group HAS boundary nodes (they exist but are unreachable from
/// `graph.in_node`), the injection degrades to the amplifier-pin heuristic,
/// which silently bypasses any in-group network upstream of the device pin —
/// the "present-but-inert" failure the edge guard cannot see (everything is
/// stamped; nothing participates). Diagnose it:
///
/// * `PK_INJECT_GUARD=error` — fail the compile.
/// * `warn` (default) — loud eprintln, compile proceeds.
/// * `off` — silent.
fn report_unverified_injection(injection_desc: &str, has_boundary_nodes: bool) -> Result<(), String> {
    if !has_boundary_nodes {
        // No boundary at all (single-group circuit whose input node vanished,
        // or a truly isolated NL island) — the amplifier pin is the best and
        // only guess; nothing was bypassed.
        return Ok(());
    }
    let mode = match std::env::var("PK_INJECT_GUARD").as_deref() {
        Ok("off") => return Ok(()),
        Ok("error") => crate::compiler::spqr_build::EdgeGuardMode::Error,
        _ => crate::compiler::spqr_build::EdgeGuardMode::Warn,
    };
    let msg = format!(
        "injection guard [general-MNA]: group has boundary nodes but none is \
         reachable from the global input — adapted input port falls back to \
         the device input pin ({injection_desc}), bypassing any in-group \
         network upstream of it (components would stay stamped yet \
         contribute nothing to the transfer). \
         (Set PK_INJECT_GUARD=warn to compile anyway, PK_INJECT_GUARD=off to \
         silence.)"
    );
    if mode == crate::compiler::spqr_build::EdgeGuardMode::Error {
        Err(msg)
    } else {
        eprintln!("[pedalkernel] WARNING: {msg}");
        Ok(())
    }
}

/// The injection target: the group's input boundary node.
///
/// NL groups that do not contain `graph.in_node` receive their audio from an
/// upstream stage at a *boundary* node — a node of this group that is also
/// touched by edges outside the group, OR a transformer winding node whose
/// sibling winding lives outside this group (a transformer couples its
/// windings with no shared graph edge, so an input transformer's secondary —
/// the Pultec `n_lf` — is invisible to the edge-touch test). Among those,
/// pick the one with the smallest rail-blocked signal-DIRECTED distance from
/// `graph.in_node` ([`directed_signal_distances_from_in`], ties broken by
/// NodeId for determinism) so the adapted VS port lands where the upstream
/// stage actually drives this group.
///
/// The DIRECTED metric matters twice (pedalkernel-n0yy):
/// * it crosses Tight transformer windings and active devices forward, so a
///   group behind an input transformer or downstream of another tube sees
///   its true entry node instead of silently degrading to the amplifier-pin
///   heuristic (the Pultec program EQ stamped-but-bypassed);
/// * it never walks backward through device outputs, sidechain taps, or
///   feed-forward forks, so an output-side boundary node cannot measure
///   "closer" to `in` than the input-side one (an undirected metric put the
///   LA-2A output-transformer group's injection on `out`: −28 dB).
///
/// Returns the input-nearest winner tagged with whether it is a
/// transformer-winding boundary, plus whether ANY boundary node exists (the
/// PK_INJECT_GUARD diagnostic fires when the search fails despite boundaries
/// existing).
struct InputBoundary {
    /// `(mna_index, is_transformer_winding)` of the input-nearest boundary
    /// node, if any is reachable from `in` under the directed metric.
    winner: Option<(usize, bool)>,
    /// The graph `NodeId` of the winner (for tracing the in-group passive path
    /// from the boundary to the amplifier pin — Defect-2 precedence).
    winner_node: Option<NodeId>,
    /// The group has at least one boundary node (reachable or not).
    has_boundary_nodes: bool,
}

fn find_input_boundary_node(
    edge_indices: &[usize],
    graph: &CircuitGraph,
    node_to_mna: &dyn Fn(NodeId) -> Option<usize>,
) -> InputBoundary {
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

    // Injection-boundary reachability crosses amplifier control pins (op-amp
    // `pos`) forward (pedalkernel-xki7, GAP 4b): a non-inverting op-amp is
    // otherwise a directed dead-end and the downstream output-follower boundary
    // is never reachable. Tube/BJT groups have no control pin, so this metric is
    // byte-identical to the stage-ordering metric for them (LA-2A/pultec/neve
    // unaffected); only op-amp-fed groups (uberdrive/lgsm) gain reachability.
    let dist = super::super::signal_flow::boundary_reachable_distances_from_in(graph);

    // Transformer-winding ENTRY nodes: winding node in this MNA driven from
    // the outside — some sibling winding node of the same transformer lies
    // outside this MNA *and* sits strictly closer to `in` (the signal
    // arrives through the transformer INTO this group). The strict-distance
    // test rejects the group's own OUTPUT transformer: its outside/secondary
    // side is downstream (larger distance, or none at all when the winding
    // hop is gated at `out`), so e.g. the LA-2A `T_out` secondary node —
    // which is simultaneously `out` and a sidechain tap — can never be
    // classified as an entry.
    //
    // EXCLUSION: groups carrying a photocoupler variable resistor (LA-2A
    // T4B) keep the legacy device-pin injection. Their in-group divider is
    // control-plane coordinated (`DetectorLedCoupling` drive scale was
    // calibrated against pin-injection — pedalkernel-o5dg); re-basing the
    // injection would silently re-scale the whole calibrated GR loop
    // (measured: LA-2A quiet gain +16.6 → +5.0 dB). Widening magnetic-entry
    // to these groups is the pedalkernel-n0yy follow-up.
    let group_has_photocoupler = edge_indices.iter().any(|&eidx| {
        graph.components[graph.edges[eidx].comp_idx]
            .kind
            .type_tag()
            == "photocoupler"
    });
    let mut winding_entry: HashSet<NodeId> = HashSet::new();
    if !group_has_photocoupler {
        let mut by_comp: std::collections::HashMap<usize, Vec<NodeId>> =
            std::collections::HashMap::new();
        for (&n, info) in &graph.transformer_info {
            by_comp.entry(info.comp_idx).or_default().push(n);
        }
        for nodes in by_comp.values() {
            for &n in nodes {
                if node_to_mna(n).is_none() {
                    continue;
                }
                let Some(&d_n) = dist.get(&n) else {
                    continue;
                };
                let driven_from_outside = nodes.iter().any(|&sib| {
                    sib != n
                        && node_to_mna(sib).is_none()
                        && dist.get(&sib).is_some_and(|&d_sib| d_sib < d_n)
                });
                if driven_from_outside {
                    winding_entry.insert(n);
                    if !boundary.contains(&n) {
                        boundary.push(n);
                    }
                }
            }
        }
    }
    if boundary.is_empty() {
        return InputBoundary {
            winner: None,
            winner_node: None,
            has_boundary_nodes: false,
        };
    }

    if std::env::var("PK_INJECT_DEBUG").as_deref() == Ok("1") {
        let names = |n: NodeId| -> String {
            let v: Vec<&str> = graph
                .node_names
                .iter()
                .filter(|(_, &x)| x == n)
                .map(|(k, _)| k.as_str())
                .collect();
            format!("{n}[{}]", v.join(","))
        };
        eprintln!(
            "[PK_INJECT/bfs] in_node={} boundary={:?} dist_of_boundary={:?}",
            names(graph.in_node),
            boundary.iter().map(|&n| names(n)).collect::<Vec<_>>(),
            boundary.iter().map(|n| dist.get(n)).collect::<Vec<_>>()
        );
    }
    let winner_pick = boundary
        .iter()
        .filter_map(|&n| dist.get(&n).map(|&d| (d, n)))
        .min()
        .map(|(_, n)| n);
    let winner =
        winner_pick.and_then(|n| node_to_mna(n).map(|m| (m, winding_entry.contains(&n))));
    InputBoundary {
        winner,
        winner_node: winner_pick.filter(|&n| node_to_mna(n).is_some()),
        has_boundary_nodes: true,
    }
}

/// Defect-2 precedence (pedalkernel-xki7): does the reachable input BOUNDARY
/// node supersede the amplifier device pin as the injection target?
///
/// The legacy order (`(Some(m), _) => amplifier-pin`) always injected at the
/// device's signal input pin even when a boundary node was forward-reachable
/// from `in`. For a BJT emitter-follower output stage whose signal enters at an
/// upstream boundary and passes THROUGH an in-group passive network (the volume
/// pot + coupling: uberdrive `R11 -> VOLUME -> R12 -> C8 -> Q2.base`,
/// lgsm `VOLUME -> C8 -> Q2.base`) to reach the base, pin injection silently
/// bypasses that whole network — the Volume knob is stamped yet inert and the
/// stage is driven by an unattenuated copy of its own input (or, pre-Defect-1,
/// nothing at all). Injecting at the boundary instead lets the in-group MNA
/// carry the divider.
///
/// TRUE iff ALL hold:
/// 1. the amplifier input pin (`amp_pin_node`) is reachable from the boundary
///    node through IN-GROUP PASSIVE edges only — so pin injection provably
///    bypasses an in-group signal network, AND
/// 2. the group's amplifier is NOT a vacuum tube and the group carries NO
///    photocoupler variable-resistor. Tube stages (LA-2A V1..V4) and
///    photocoupler cells were CALIBRATED against pin injection: their GR loop
///    and the straddling-Gain-pot WiperDivider contract assume the device pin
///    (general.rs photocoupler exclusion / pedalkernel-n0yy). Re-basing them to
///    a boundary node re-scales the whole loop (measured +16.6 -> +5.0 dB on
///    the LA-2A) — out of scope here. The BJT/FET follower classes carry no
///    such calibration, so the physically-correct boundary injection is safe.
fn boundary_supersedes_amp_pin(
    edge_indices: &[usize],
    graph: &CircuitGraph,
    amp_pin_node: NodeId,
    boundary_node: NodeId,
) -> bool {
    let edge_set: HashSet<usize> = edge_indices.iter().copied().collect();

    // (2) calibration guard: skip tube and photocoupler groups.
    let calibrated_class = edge_indices.iter().any(|&eidx| {
        let comp = &graph.components[graph.edges[eidx].comp_idx];
        let tag = comp.kind.type_tag();
        tag == "triode"
            || tag == "pentode"
            || tag == "variable-mu triode"
            || tag == "photocoupler"
    });
    if calibrated_class {
        return false;
    }

    // (1) trace IN-GROUP passive edges from the boundary node; reach the amp
    //     pin? Passive-only so we never cross the device itself (which would
    //     make every group trivially "reachable" and defeat the point).
    let mut visited: HashSet<NodeId> = HashSet::new();
    visited.insert(boundary_node);
    let mut queue: VecDeque<NodeId> = VecDeque::new();
    queue.push_back(boundary_node);
    while let Some(node) = queue.pop_front() {
        if node == amp_pin_node {
            return true;
        }
        for &eidx in &edge_set {
            let e = &graph.edges[eidx];
            let comp = &graph.components[e.comp_idx];
            if !matches!(
                comp.kind.signal_terminals(),
                super::super::component::SignalTerminals::Passive
            ) {
                continue;
            }
            let next = if e.node_a == node {
                Some(e.node_b)
            } else if e.node_b == node {
                Some(e.node_a)
            } else {
                None
            };
            if let Some(n) = next {
                if visited.insert(n) {
                    queue.push_back(n);
                }
            }
        }
    }
    false
}

/// Step 5: Derive scattering matrix + iterative Thevenin adaptation.
fn derive_scattering(
    mna: &MnaSystem,
    ports: &mut [WdfPort],
    nl_port_resistances: &mut [f64],
    n_nl: usize,
    vcc_vs_idx: Option<usize>,
    // Gate for the input VS-port Thevenin adaptation (pedalkernel-ayjt /
    // pedalkernel-guwp). `true` for (a) divider-fed high-Z followers
    // (boundary-supersedes-amp-pin arm — uberdrive/lgsm) and (b) `in_node`
    // injection into the high-Z control input of a coupled ≥2-active-device NL
    // group (the Fuzz-Face 2-Ge pair — sunflower). See comment below.
    adapt_input_vs_port: bool,
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

    // Iterative Thevenin adaptation: drive each free port's self-reflection
    // S_ii -> 0 so its reference resistance equals the driving-point impedance
    // it actually sees. This covers the NL device ports AND the adapted input
    // (voltage-source) port — which the old loop left pinned at the 1000Ω
    // placeholder, mis-scaling the injected signal. Reactive ports keep their
    // physical rp (not adapted). Tight tolerance + more iters so the result no
    // longer depends on the seed; low-Z ports (e.g. an emitter follower) are
    // allowed to adapt below 1Ω.
    // pedalkernel-ayjt (GAP 4c magnitude) — GATED input-VS-port adaptation.
    // DESIGN-E/F proposed ALSO adapting the input (voltage-source) port at index
    // n_total-1 to correct a ~2x injection inflation into the output follower.
    // Applied UNCONDITIONALLY that is a net regression: it halves the small-
    // signal gain of a standard inverting amp (Rf/Ri=10 -> 4.97) and shifts the
    // asymmetric-diode clip levels — breaking mna_accuracy / tree_build::
    // adaptor_gain / voltage_extraction::wdf_asymmetric_diodes (all green on the
    // base). Those groups inject via `in_node` / `amplifier-pin` /
    // `boundary-fallback`; their VS port sits at a REAL matched source rp (the
    // Thevenin source resistance of the driving network), so adapting it a second
    // time double-counts the impedance and mis-scales the gain.
    //
    // The divider-fed high-Z FOLLOWER case (uberdrive/lgsm) is different: it
    // injects via the `boundary-supersedes-amp-pin` arm (`adapt_input_vs_port`),
    // where the VS port lands at an in-group divider boundary and was left pinned
    // at the 1000Ω placeholder rp — NOT the driving-point impedance the follower
    // actually sees. Adapting S_ii -> 0 there sets the reference resistance to the
    // real divider-source impedance, which is precisely what corrects the ~2x
    // (measured ~35x lift on lgsm, RMS 1e-4 -> 3.5e-3 V). The gate is topological,
    // not name-based: it fires iff the injection superseded the amplifier device
    // pin because a reachable passive divider feeds a high-Z active input.
    let adapt_ports: Vec<usize> = if adapt_input_vs_port {
        (0..n_nl).chain(core::iter::once(n_total - 1)).collect()
    } else {
        (0..n_nl).collect()
    };
    for _iter in 0..40 {
        let mut needs_recompute = false;
        for &i in &adapt_ports {
            let s_refl = scattering[i * n_total + i];
            if s_refl.abs() > 1e-5 {
                let z_th = ports[i].resistance * (1.0 + s_refl) / (1.0 - s_refl);
                if z_th.is_finite() && z_th > 1e-3 {
                    ports[i].resistance = z_th;
                    if i < n_nl {
                        nl_port_resistances[i] = z_th;
                    }
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
                grid_node: Some(_), ..
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
/// Human-readable device TYPE for a classified NL device — the model name when
/// the kind carries one (e.g. `2N3055`, `BC184C`, `12AX7`), else a generic
/// device-class label. This is the `device_type` half of the op-point identity.
fn nl_device_type(kind: &NonlinearKind) -> String {
    match kind {
        NonlinearKind::BjtNpn { model_name, .. } | NonlinearKind::BjtPnp { model_name, .. } => {
            model_name.clone()
        }
        NonlinearKind::Triode { model_name, .. } | NonlinearKind::Pentode { model_name, .. } => {
            model_name.clone()
        }
        NonlinearKind::Jfet { model_name, .. } => model_name.clone(),
        NonlinearKind::DiodePair(_) => "DiodePair".to_string(),
        NonlinearKind::SingleDiode(_) => "Diode".to_string(),
        NonlinearKind::Mosfet { .. } => "MOSFET".to_string(),
        NonlinearKind::Zener { .. } => "Zener".to_string(),
        NonlinearKind::Ota => "OTA".to_string(),
    }
}

fn create_nl_devices(
    nl_kinds: &[NonlinearKind],
    nl_comp_labels: &[String],
    supply_voltage: f64,
) -> Result<(Vec<NlDeviceKind>, Option<MultiNlDeviceGroups>), String> {
    // `nl_comp_labels[i]` is the `.pedal` component ref of `nl_kinds[i]` (1:1,
    // same build order); `nl_device_type(nl_kinds[i])` is its model type. The
    // grouped branches below iterate `nl_kinds` in this same order, so the
    // identity at index `i` attaches to the group built from `nl_kinds[i]`.
    let identity_for = |i: usize| DeviceIdentity {
        ref_name: nl_comp_labels.get(i).cloned().unwrap_or_default(),
        device_type: nl_device_type(&nl_kinds[i]),
    };
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
        let mut identities = Vec::new();
        let mut offset = 0usize;
        for (i, kind) in nl_kinds.iter().enumerate() {
            offsets.push(offset);
            identities.push(identity_for(i));
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
        Ok((
            Vec::new(),
            Some(MultiNlDeviceGroups {
                groups,
                offsets,
                identities,
            }),
        ))
    } else if all_bjt && any_bjt_two_port {
        let mut groups = Vec::new();
        let mut offsets = Vec::new();
        let mut identities = Vec::new();
        let mut offset = 0usize;
        for (i, kind) in nl_kinds.iter().enumerate() {
            offsets.push(offset);
            identities.push(identity_for(i));
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
        Ok((
            Vec::new(),
            Some(MultiNlDeviceGroups {
                groups,
                offsets,
                identities,
            }),
        ))
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
    r_adapted: f64,
) -> Result<MultiNlStage, String> {
    let scattering_blocks = MultiNlScattering::from_full_matrix(&scattering, n_nl, n_passive);
    let port_resistances: Vec<f64> = ports.iter().map(|p| p.resistance).collect();
    let adaptor = RTypeAdaptor::new(scattering, &port_resistances);
    let extract_coeffs = extract_output_nodes.map(|out| {
        let (out_pos, out_neg) = out.as_tuple();
        mna.derive_node_extraction_coeffs(&ports, out_pos, out_neg)
    });

    if std::env::var("PK9XU1_DEBUG").is_ok() {
        eprintln!(
            "[PK9XU1] assemble: n_nl={} n_passive={} n_total_ports={} r_adapted={}",
            n_nl,
            n_passive,
            ports.len(),
            r_adapted
        );
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
                    // ko5g.4: the model-derived conduction warm-start (signed
                    // Vbe at a nominal 1 mA, half-rail Vce) lives in bias.rs —
                    // `bjt_model_conduction_seed` — alongside the other BJT
                    // bias flavors.
                    let (vbe, vce) = super::super::bias::bjt_model_conduction_seed(
                        &bjt.model,
                        supply_voltage,
                        bjt.is_pnp,
                    );
                    if off < n_nl {
                        initial_v[off] = vbe;
                    }
                    if off + 1 < n_nl {
                        initial_v[off + 1] = vce;
                    }
                    if std::env::var("PK_BIAS_QPOINT_DEBUG").is_ok() {
                        eprintln!(
                            "[bias-qpoint] path=mna-seed bjt {}: vbe={vbe:.6} vce={vce:.6} \
                             supply={supply_voltage}",
                            nl_comp_labels.get(g).map(|s| s.as_str()).unwrap_or("?"),
                        );
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
                // Node-voltage seeds key on circuit nodes, not device refs — they
                // seed reactive ports (apply_cap_seed), never this BJT warm-start.
                let hint = init_hints.iter().find(|h| {
                    h.device_label == label
                        && !matches!(h.state, crate::dsl::InitState::NodeVoltage { .. })
                });
                let Some(hint) = hint else { continue };
                let off = dg.offsets[g];
                if let NlDeviceGroupKind::BjtTwoPort(bjt) = group {
                    let sign = if bjt.is_pnp { -1.0 } else { 1.0 };
                    match &hint.state {
                        crate::dsl::InitState::Named(state_name) => {
                            let (vbe, vce) = resolve_bjt_init_state(state_name, supply_voltage);
                            if off < n_nl {
                                initial_v[off] = sign * vbe;
                            }
                            if off + 1 < n_nl {
                                initial_v[off + 1] = sign * vce;
                            }
                            // Named states are the asymmetric-oscillator seed path:
                            // engaging the DC ramp protects the seed across reset().
                            any_hint_applied = true;
                            eprintln!(
                                "[init-hint] {label}: {state_name} → Vbe={:.3}, Vce={:.3} (sign={sign})",
                                vbe, vce
                            );
                        }
                        crate::dsl::InitState::Explicit { vbe, vce } => {
                            // Explicit `op { }` operating-point seed: a steady-state
                            // warm-start (the opposite of an oscillator asymmetry), so
                            // it deliberately does NOT set `any_hint_applied` and the
                            // stage keeps `dc_ramp = 0` (full DC excitation at t=0,
                            // mirroring ngspice's `.op`-then-`.tran`). This only sets
                            // the build-time `initial_v`; the authoritative override
                            // that survives `apply_bjt_dc_qpoint` is `apply_op_seed`
                            // (Step 11 in build_general_mna_from_edges_inner).
                            if off < n_nl {
                                initial_v[off] = sign * *vbe;
                            }
                            if off + 1 < n_nl {
                                initial_v[off + 1] = sign * *vce;
                            }
                            eprintln!(
                                "[op-seed] {label}: Vbe={:.4}, Vce={:.4} (sign={sign})",
                                vbe, vce
                            );
                        }
                        // Filtered out of the `find` above; unreachable here.
                        crate::dsl::InitState::NodeVoltage { .. } => {}
                    }
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
        // Default = usize::MAX: the stage reads the SERIAL chain signal (the
        // runtime injection read at processor.rs is gated on `!= usize::MAX`).
        // Only an explicitly-wired output follower (spqr_build GAP-4c pass) sets a
        // real boundary node here so it reads that node from `node_signals`
        // instead of the serial 0.0 it would otherwise get in the MAX ordering
        // band. Previously this was `graph.in_node`, which was never read; making
        // it a sentinel keeps the general-MNA read path on `signal` for every
        // non-follower stage (LA-2A/pultec/neve/tb303 byte-identical).
        injection_node_id: usize::MAX,
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
        // DC servo: off by default. `apply_bjt_dc_qpoint` enables and configures
        // it (physics-derived τ → bandwidth) only for the multi-BJT feedback
        // servo it seeds, leaving every other stage byte-identical.
        dc_servo_active: false,
        dc_servo_alpha: 0.0,
        dc_servo_gain: 0.0,
        dc_servo_tau: 0.0,
        dc_servo_v_lp: Vec::new(),
        dc_servo_corr: Vec::new(),
        dc_servo_mask: Vec::new(),
        stiff_fold: None,
    })
}

// ═══════════════════════════════════════════════════════════════════════════
// DC Q-point pre-charge for triode stages
// ═══════════════════════════════════════════════════════════════════════════

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
    dc: &super::super::bias::TriodeDcQpoint,
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

/// Apply the solved pentode DC operating point to a just-built single-pentode
/// `MultiNlStage` (ko5g.5 — the exact `apply_triode_dc_qpoint` mirror).
///
/// Port layout (see `build_general_mna`): port 0 = grid-cathode (Vg1k),
/// port 1 = plate-cathode (Vpk).  Additionally seeds the circuit-resolved
/// screen voltage on the `PentodeThreePort` group device (its `vg2k` is an
/// external parameter, previously always the model default).
fn apply_pentode_dc_qpoint(
    stage: &mut MultiNlStage,
    dc: &super::super::bias::PentodeDcQpoint,
    nl_kinds: &[NonlinearKind],
    reactive_edges: &[(usize, OnePortKind)],
    graph: &CircuitGraph,
) {
    // NR warm-start voltages at the solved op.
    if stage.v_prev.len() >= 2 {
        stage.v_prev[0] = dc.vg1k;
        stage.v_prev[1] = dc.vpk;
    }
    if stage.initial_v_prev.len() >= 2 {
        stage.initial_v_prev[0] = dc.vg1k;
        stage.initial_v_prev[1] = dc.vpk;
    }
    if stage.v_prev_2.len() >= 2 {
        stage.v_prev_2[0] = dc.vg1k;
        stage.v_prev_2[1] = dc.vpk;
    }

    // Circuit-true screen voltage (None = unwired screen, model default stands).
    if let Some(vg2) = dc.vg2k {
        if let Some(groups) = stage.device_groups.as_mut() {
            for g in &mut groups.groups {
                if let NlDeviceGroupKind::PentodeThreePort(p) = g {
                    p.set_vg2k(vg2 as pedalkernel_rt::Wave);
                }
            }
        }
        for d in &mut stage.nl_devices {
            if let NlDeviceKind::Pentode(p) = d {
                p.set_vg2k(vg2 as pedalkernel_rt::Wave);
            }
        }
    }

    let cathode_node = match nl_kinds.first() {
        Some(NonlinearKind::Pentode { cathode_node, .. }) => *cathode_node,
        _ => return,
    };

    // Pre-charge cathode bypass capacitors (caps between cathode and GND) —
    // same mechanism as the triode apply above.
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
    bake_cross_group_seed: bool,
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
    // Mask of the controlling base-emitter ports (first of each BJT pair) — the
    // only ports the DC servo pins.  The collector-emitter (load-line) ports are
    // left free so the natural AC swing is not over-constrained.
    let mut be_port_mask = vec![false; n_nl];

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
                        be_port_mask[port_idx] = true; // base-emitter port
                    }
                    port_idx += 2;
                }
                NonlinearKind::Triode {
                    grid_node: Some(_), ..
                }
                | NonlinearKind::Pentode {
                    grid_node: Some(_), ..
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

    // The consolidated nodal Q-point seed is reserved for **DC-coupled feedback
    // servos** — groups with more than one BJT whose operating point is mutually
    // established through the network (e.g. BA283's TR1↔Darlington servo).  For a
    // plain single-BJT common-emitter stage (Big Muff sustain/tone cells, fuzz
    // faces, etc.) the linear vcc-injection `dc_bias` already biases the stage and
    // the runtime holds it; seeding/re-inverting would only perturb a working
    // set-point.  So single-BJT groups are left untouched here — byte-identical to
    // the pre-consolidation default (which never ran a BJT seed).
    //
    // EXCEPTION (pedalkernel-onu2, RCA GAP 4a): a single-BJT group whose base
    // sits at a vref-class virtual ground (`bake_cross_group_seed` — set by the
    // caller ONLY when a vref-class interior override was needed: the bypassed
    // bias divider is in another flow group AND is unrepresentable in the
    // runtime stage's own MNA, e.g. the uberdrive/LGSM `vref`-biased follower).
    // There the premise above fails on both counts: the linear vcc-injection
    // over the group-local MNA never saw the divider, and the runtime stage
    // structurally CANNOT see it, so without baking the closure-solved op-point
    // via `dc_qpoint_v` + `apply_dc_qpoint_seed` the follower only ever reads
    // cutoff. This is deliberately NOT triggered by a merely-grown closure (a
    // CE stage whose collector load returns to a rail through another group —
    // the Big Muff Q3 Sustain chain): that stage's group-local linear `dc_bias`
    // already biases it, and overwriting it with the near-saturation closure op
    // would kill its small-signal gain.
    let n_bjt = nl_kinds
        .iter()
        .filter(|k| {
            matches!(
                k,
                NonlinearKind::BjtNpn { base_node, collector_node, .. }
                | NonlinearKind::BjtPnp { base_node, collector_node, .. }
                    if base_node != collector_node
            )
        })
        .count();
    if n_bjt < 2 && !bake_cross_group_seed {
        return;
    }

    // 1. Warm-start the NR at the solved operating point.
    let v_seed: Vec<pedalkernel_rt::Wave> = v_star
        .iter()
        .map(|o| o.unwrap() as pedalkernel_rt::Wave)
        .collect();

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
        } else if node == graph.in_node || node == graph.out_node {
            // AC-coupled I/O: the source / load network DC-references ground, so
            // a coupling cap facing `in`/`out` charges to the full interior DC
            // voltage.  Treating the input node as "floating ⇒ 0 V across the
            // cap" seeded the BA283's Cin at 0 V instead of its true −1.03 V:
            // the compile state was self-consistent (F(seed)=0) but NOT a steady
            // state of the cap dynamics, so Cin charged over ~0.2 s and dragged
            // the stage off the solved op (Q3 vbe 0.6417 → 0.663, ΔIc +46 %) —
            // the cold-path twin of the `nodes{}`-seed Cin bug fixed in
            // `apply_cap_seed`.
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
        if let (Some(&one_port), Some(&bk)) = (stage.passive_one_ports.get(k), passive_b.get(k)) {
            // Pin the cap at its DC fixed point (`a = b = v`, previous_reflected
            // = v): the first sample sees b_passive[k] = passive_b[k] AND the
            // cap's stored voltage/history are steady-state-consistent — the
            // same primitive the `op { nodes { … } }` seed path uses
            // (`apply_cap_seed`), which is proven to HOLD over 48 000 samples.
            // (`wdf_set_incident` left `previous_reflected` at the stale
            // pre-seed value; the `CapacitorVoltage` setter yields 2·v on a
            // fresh cap — both perturb the first samples.)
            one_port.wdf_seed_dc_voltage(bk, &mut stage.passive_runtime_state);
        }
    }
    // Re-read the actual reflected waves so the inversion below sees exactly the
    // state the runtime will produce (mirrors `apply_cap_seed`; for resistor /
    // non-stateful ports `wdf_reflected` is 0 rather than the analytic guess).
    for (k, b) in passive_b.iter_mut().enumerate() {
        if let Some(&one_port) = stage.passive_one_ports.get(k) {
            *b = one_port.wdf_reflected(&stage.passive_runtime_state);
        }
    }

    // 2c. Store the seed and derive dc_bias from it (coupling-aware inversion).
    //     The runtime re-runs the SAME derivation after any scattering recompute
    //     (e.g. a feedback-pot move), so the servo bias survives pot changes.
    stage.dc_qpoint_v = Some(v_seed);
    stage.dc_qpoint_passive_b = passive_b;
    stage.apply_dc_qpoint_seed();

    // 2d. PHYSICS-DERIVED DC SERVO — OFF BY DEFAULT since the output-load
    //     fusion (opt-in: PK_SERVO_ENABLE; PK_SERVO_DISABLE still forces off).
    //
    //     HISTORY: the servo was introduced when the seeded conducting operating
    //     point was DYNAMICALLY UNSTABLE in the per-sample cap-coupled dynamics
    //     — without it the runtime slid to the starved (cutoff) fixed point.
    //     It is a slow, proportional DC-restoring servo on the
    //     controlling (Vbe) ports, run at the bias network's natural bandwidth
    //     `f_c ≈ 1/(2π·τ)` with `τ = max_k (R_thévenin · C)` over the circuit's
    //     reactive elements (see `dominant_bias_tau`).  The rate FALLS OUT of the
    //     circuit's R/C — it is NOT a hardcoded constant.  The LP coefficient
    //     `α = dt/τ` rejects the audio swing so the servo senses only the slow DC
    //     drift; the proportional gain `k_p` restores the op-point without the
    //     windup / limit-cycle a pure integrator suffers around the unstable fixed
    //     point.
    //
    //     WHY THE DEFAULT FLIPPED TO OFF (output-load-fusion branch):
    //     (a) The stability job is done at the ROOT: the cold-path DC solve now
    //         lands AND holds ngspice's op exactly — the seed-and-hold probe is
    //         bit-stable over 48 000 samples with the servo OFF, and the
    //         per-sample map Jacobian is contractive (ρ = 0.9998 < 1, incl. the
    //         fused Cout·RL pole).  The knife-edge the servo guarded against no
    //         longer exists.
    //     (b) With the output load FUSED into the stage (trailing-output-group
    //         fusion in spqr_build), the servo actively SUPPRESSES PHYSICS: the
    //         loaded class-A stage clips by cutoff under large swing, and the
    //         clipping-rectified DC component on Vbe — a real behavior ngspice
    //         shows — looks like "drift" to the servo, which corrects it and
    //         linearizes the amp.  A/B vs the LOADED ngspice golden (identical
    //         fused topology): servo ON reads THD −30.3 dB / LEVEL +2.24 dB /
    //         tilt 4.32 dB; servo OFF reads THD −16.4 vs golden −16.1 dB
    //         (Δ 0.4 dB), LEVEL −0.05 dB, tilt 0.79 dB across 50 Hz–10 kHz.
    //         There is no such control loop in the physical circuit; keeping it
    //         on by default trades a solved stability problem for a real
    //         large-signal fidelity error.
    //
    //     STATUS (BA283 runtime-stability fix): fix #1 — the compile-time DC solve
    //     now applies the BJT parasitic drops (804f91e6, TR1 122→258 µA) — DISSOLVED
    //     the old knife-edge.  With the CORRECT (fully-conducting) seed the servo is
    //     ROBUST: a k_p sweep 8→45 ALL hold the op-point (drift 0.000, settled), and
    //     the BA283 seed-and-hold goes WALK→HOLD.  The old note (only k_p 20–22
    //     lands, collapses otherwise) was a property of the UNDER-CONDUCTING seed,
    //     not the servo.  The servo's job is STABILITY — hold the seeded op-point —
    //     NOT gain: as k_p→∞ it holds ever closer to the true seed and the gain
    //     asymptotes to ~+2.8 dB.  We therefore KEEP the bias-hold default k_p = 20
    //     (holds near the seed, gain ~+2.4 dB) and do NOT trim k_p toward 0 dB —
    //     reading 0 dB requires k_p≈6, which holds a STARVED bias whose lower gm
    //     cancels a real small-signal error.  That residual is BUG #3 below, a
    //     loop-gain-layer problem, NOT a bias/stability one.  Scoped: this block
    //     only runs when `solve_bjt_group_dc_qpoint` returned a seed (multi-BJT
    //     group) and only pins ports whose seed is a conducting Si Vbe — so
    //     non-feedback / single-BJT / degenerate groups are untouched and their
    //     goldens stay byte-identical.  Verified: `cargo test -p pedalkernel --lib`
    //     failing set is byte-identical with the servo ON vs OFF (no lib-corpus
    //     circuit reaches this path; BA283 lives in pedalkernel-pro).
    //
    //     BUG #3 (LOGGED RESIDUAL — next workstream, NOT fixed here): at the CORRECT
    //     operating point there is a ~+2.8 dB small-signal gain error (the BA283
    //     `ba283_wdf_vs_spice` reads ~+2.4 dB with the k_p=20 hold, asymptoting to
    //     ~+2.8 dB as k_p→∞).  ba283_runtime_nr confirms the mechanism: the realized
    //     dIc/(gm·dVbe) ratio ≈ 0.55 at the correct bias — the transconductance /
    //     loop gain is not fully realized.  This is the engine-adapted-input-port /
    //     loop-gain layer the servo's author flagged (685e) — NOT something k_p
    //     should paper over.  Do NOT tune k_p to hide it; fix the loop-gain layer
    //     instead.  Tracked: bd pedalkernel-opi6.
    let mut tau = dominant_bias_tau(reactive_edges, node_dc, graph);
    if let Ok(s) = std::env::var("PK_SERVO_TAU") {
        if let Ok(v) = s.parse::<f64>() {
            tau = v;
        }
    }
    // Default OFF for this dc_qpoint-seeded multi-BJT feedback path (see WHY THE
    // DEFAULT FLIPPED above): the hold is bit-stable without it and it suppresses
    // the real load-induced op-point dynamics.  `PK_SERVO_ENABLE` opts back in
    // (stability belt for experiments); `PK_SERVO_DISABLE` still forces it off
    // and wins over both.
    let servo_enabled =
        std::env::var("PK_SERVO_ENABLE").is_ok() && std::env::var("PK_SERVO_DISABLE").is_err();
    if servo_enabled && tau > 0.0 {
        let fs = stage.passive_sample_rate.max(1.0) as f64;
        // Low-pass coefficient α = dt/τ for the slow op-point DC estimate.
        let alpha = (1.0 / (tau * fs)).clamp(1e-7, 0.5);
        // Proportional DC-restoring gain.  k_p = 20 is the BIAS-HOLD default: it
        // holds the seeded conducting op-point near the true seed (gain ~+2.4 dB).
        // It is NOT a gain-trim — with fix #1's correct seed the whole k_p 8→45
        // band holds (drift 0.000), so 20 is a robust hold value, not a knife-edge
        // pick.  Do NOT lower it toward ~6 to read 0 dB: that holds a STARVED bias
        // and hides BUG #3 (the ~+2.8 dB loop-gain residual).  Overridable via
        // PK_SERVO_KP.
        let mut k_p = 20.0_f64;
        if let Ok(s) = std::env::var("PK_SERVO_KP") {
            if let Ok(v) = s.parse::<f64>() {
                k_p = v;
            }
        }
        stage.dc_servo_active = true;
        stage.dc_servo_alpha = alpha as pedalkernel_rt::Wave;
        stage.dc_servo_gain = k_p as pedalkernel_rt::Wave;
        stage.dc_servo_tau = tau as pedalkernel_rt::Wave;
        let seed = stage.dc_qpoint_v.clone().unwrap_or_default();
        stage.dc_servo_v_lp = seed.clone();
        stage.dc_servo_corr = vec![0.0; stage.dc_bias.len()];
        // Only servo a base-emitter port whose SEED is a CONDUCTING Si junction
        // (|Vbe| ∈ [0.3, 0.9] V).  A port whose seed is non-conducting (e.g. the
        // BA283 Darlington driver TR2, whose model-driven seed sits at cutoff) is
        // left FREE: pinning it at its own cutoff seed would starve the forward
        // chain.  The free port then finds conduction from the coupled solve once
        // the conducting ports are held.
        let mut mask = be_port_mask.clone();
        for (i, m) in mask.iter_mut().enumerate() {
            if *m {
                let vb = seed.get(i).copied().unwrap_or(0.0) as f64;
                if !(0.3..=0.9).contains(&vb.abs()) {
                    *m = false;
                }
            }
        }
        stage.dc_servo_mask = mask;
        if std::env::var("PK_SERVO_TRACE").is_ok() {
            eprintln!(
                "[PK_SERVO] dominant τ={tau:.6e}s  f_c={:.3}Hz  α={alpha:.3e}  k_p={k_p:.3}  fs={fs:.0}  n_nl={}  v_seed={:?}  servo_mask={:?}",
                1.0 / (2.0 * std::f64::consts::PI * tau),
                stage.dc_bias.len(),
                seed,
                stage.dc_servo_mask,
            );
        }
    }

    // Skip the DC ramp: the operating point is already seeded into v_prev and
    // dc_bias, so ramping dc_bias up from 0 would pull the NR back through the
    // cold/cutoff basin (the BA283 servo has both a cutoff and an active fixed
    // point — the warm-start must keep it in the active basin from sample 0).
    stage.dc_ramp = 256;
    stage.initial_dc_ramp = 256;
}

/// Dominant bias-network time constant `τ = max_k (C_k · R_thévenin_k)` over the
/// reactive elements of a DC-coupled feedback-servo group.
///
/// `R_thévenin` at a cap's terminal is the DC resistance looking back into the
/// resistor network (rails — gnd/vcc/supply — are AC-grounded), estimated as the
/// reciprocal of the total resistor conductance incident on that node.  For a cap
/// spanning nodes (a, b) the series RC pole sees `R_a + R_b`.  This is the
/// bias network's natural bandwidth — the rate at which the operating point can
/// physically move — and it differs per circuit because it is built from the
/// circuit's own R·C.
fn dominant_bias_tau(
    reactive_edges: &[(usize, OnePortKind)],
    node_dc: &std::collections::HashMap<NodeId, f64>,
    graph: &CircuitGraph,
) -> f64 {
    use std::collections::HashMap;
    // Sum of resistor conductances incident on each interior node.  Rails
    // (gnd/vcc/supply) and pots/JFETs/photocouplers all act as a finite
    // resistance to AC ground, so they add to the node's conductance just like a
    // grounded resistor — that is exactly what sets the Thévenin R the cap sees.
    let mut g_node: HashMap<NodeId, f64> = HashMap::new();
    let is_rail = |n: NodeId| -> bool {
        n == graph.gnd_node
            || n == graph.vcc_node
            || graph.supply_nodes.contains(&n)
            || graph.ac_ground_nodes.contains(&n)
    };
    for e in &graph.edges {
        let comp = &graph.components[e.comp_idx];
        let r = match comp.kind.resistance() {
            Some(r) if r.is_finite() && r > 0.0 => r,
            _ => continue,
        };
        let g = 1.0 / r;
        for n in [e.node_a, e.node_b] {
            if !is_rail(n) {
                *g_node.entry(n).or_insert(0.0) += g;
            }
        }
    }
    // Thévenin resistance at a node: 1/Σg.  A rail node is AC ground → R≈0.
    let r_thev = |n: NodeId| -> f64 {
        if is_rail(n) {
            0.0
        } else {
            match g_node.get(&n) {
                Some(&g) if g > 0.0 => 1.0 / g,
                _ => 0.0,
            }
        }
    };

    let _ = node_dc; // reserved for future per-node DC weighting

    // The servo's natural bandwidth is set by the DOMINANT (largest) `R·C` over
    // the FEEDBACK-SERVO GROUP's OWN reactive elements — i.e. this MultiNl stage's
    // `reactive_edges` (BA283: Cmil 220p, Cfb 4.7n, Ccmp 330p).  Cfb·R dominates
    // at ≈26 µs — that is the DC-coupled negative-feedback cap that physically
    // sets how fast the bias servo loop moves, and empirically the τ at which the
    // servo can re-pin the conducting Darlington.  (The big DC-blocking coupling
    // caps Cin/Cout, tens of µF, live in ADJACENT coupling groups and govern AC
    // coupling, not the in-loop bias servo, so they are deliberately excluded.)
    let mut tau_max = 0.0_f64;
    let trace = std::env::var("PK_SERVO_TRACE").is_ok();
    for (eidx, kind) in reactive_edges {
        let c = match kind {
            OnePortKind::Capacitor(c) if c.is_finite() && *c > 0.0 => *c,
            // Inductors are a short at DC → don't set the bias bandwidth; skip.
            _ => continue,
        };
        let e = &graph.edges[*eidx];
        let r_series = r_thev(e.node_a) + r_thev(e.node_b);
        let tau = c * r_series;
        if trace {
            let comp = &graph.components[e.comp_idx];
            eprintln!(
                "[PK_SERVO]   cap {:<6} C={c:.3e}F  Rthev(a)={:.1} Rthev(b)={:.1}  τ={tau:.4e}s",
                comp.id,
                r_thev(e.node_a),
                r_thev(e.node_b)
            );
        }
        if tau.is_finite() && tau > tau_max {
            tau_max = tau;
        }
    }
    tau_max
}

// pedalkernel-pmv1 note: `find_output_extract_node`'s final fallback (below,
// "Op-amp feedback stage with the global `out` in a downstream stage") has the
// same "first nullor in declaration order, not the one actually wired to
// `out`" shape as the bug fixed in `mna_builder.rs`'s `output_mna`/
// `injection_mna` resolution (pedalkernel-pmv1: BiValve SVF's LPF/HPF taps
// collapsed to the same node because a multi-opamp group's output extraction
// grabbed the first-declared nullor instead of the true downstream boundary).
//
// This function is NOT reachable by that scenario today: multi-opamp linear
// groups (no NL elements) are routed to Iir/StateSpace by
// `rigid/mod.rs::classify_rigid`, never to the General/MNA+NR path this
// function serves (its only caller is `build_general_mna_from_edges_inner`,
// used when `stats.nl_count > 0`). The tests below document and guard the
// CURRENT behavior for a multi-nullor General-MNA group (safe today, by
// construction of the fixture below, but only because the earlier
// `node_set`/direct-neighbor branches happen to resolve it correctly first —
// see `multi_nullor_general_group_correct_by_direct_neighbor` for the
// reachable-today shape, and
// `multi_nullor_general_group_would_hit_first_nullor_fallback` for
// confirmation that the *fallback itself* has not been hardened, so a future
// General-MNA circuit that reaches it via a different topology could still
// trip this bug. Do not harden the fallback here — strict TDD: no failing
// test on unmodified source, no change (see PR review discussion on
// pedalkernel-pmv1; follow-up tracked separately).
#[cfg(test)]
mod find_output_extract_node_tests {
    use super::*;

    /// Three op-amps (U_hp, U_bp, U_lp) form one signal-flow group (a
    /// KHN/state-variable feedback loop) with an NL element added so the
    /// group actually reaches the General/MNA+NR path (`stats.nl_count > 0`,
    /// per `classify_rigid`) — this is what it would take for
    /// `find_output_extract_node` to be reachable for a multi-opamp group.
    /// `all_edges`/`node_set` is the group's own internal loop edges — it
    /// does NOT include the final `R_out` edge that actually connects `out`
    /// to one specific op-amp's `.out` node. `out_node` is wired (via R_out,
    /// outside `all_edges`) to U_lp.out — the LAST-declared op-amp, not the
    /// first.
    fn khn_loop_graph() -> (CircuitGraph, Vec<usize>) {
        let pedal = crate::dsl::parse_pedal_file(
            r#"
            pedal "test" { supply 9V
                components {
                    U_hp: opamp(tl072)
                    U_bp: opamp(tl072)
                    U_lp: opamp(tl072)
                    R_in: resistor(47k)
                    R_hp_fb: resistor(47k)
                    R_q: resistor(150k)
                    C_bp: cap(4.7n)
                    C_lp: cap(4.7n)
                    R_out: resistor(10k)
                }
                nets {
                    in -> R_in.a
                    R_in.b -> U_hp.neg
                    U_hp.neg -> R_hp_fb.a
                    R_hp_fb.b -> U_hp.out
                    U_bp.out -> R_q.a
                    R_q.b -> U_hp.neg
                    U_hp.pos -> gnd

                    U_hp.out -> U_bp.neg
                    U_bp.neg -> C_bp.a
                    C_bp.b -> U_bp.out
                    U_bp.pos -> gnd

                    U_bp.out -> U_lp.neg
                    U_lp.neg -> C_lp.a
                    C_lp.b -> U_lp.out
                    U_lp.pos -> gnd

                    U_lp.out -> R_out.a
                    R_out.b -> out
                }
                controls {}
            }"#,
        )
        .expect("parse failed");
        let graph = CircuitGraph::from_pedal(&pedal);

        // The group under test is the internal feedback loop: every edge
        // EXCEPT the R_out edge that connects the loop to `out`. This
        // mirrors the real caller (build_general_mna_from_edges_inner),
        // where `all_edges` is the rigid/SPQR group's edge set and `out`
        // is reached only by tracing beyond it.
        let r_out_comp = graph
            .components
            .iter()
            .position(|c| c.id == "R_out")
            .expect("R_out component");
        let all_edges: Vec<usize> = (0..graph.edges.len())
            .filter(|&i| graph.edges[i].comp_idx != r_out_comp)
            .collect();
        (graph, all_edges)
    }

    #[test]
    fn khn_loop_out_node_reached_via_direct_neighbor_not_fallback() {
        // Documents WHICH branch resolves this fixture. out_node is not
        // directly in the loop's node_set (R_out is excluded from
        // all_edges)...
        let (graph, all_edges) = khn_loop_graph();
        let node_set = collect_mna_nodes(&all_edges, &graph);
        assert!(
            !node_set.contains(&graph.out_node),
            "fixture invalid: out_node should NOT be directly in the loop's node_set"
        );

        // ...but R_out's own edge (node_a=U_lp.out, excluded from all_edges
        // since R_out isn't part of the loop group, node_b=out_node) IS
        // exactly the "direct neighbor" branch's match: an edge outside
        // all_edges connecting out_node to a node_set member. So this
        // fixture resolves correctly via the SECOND branch (direct neighbor
        // trace), never reaching the first-nullor fallback at all. That
        // fallback remains untested for a genuinely-reachable multi-nullor
        // case — see the module doc comment above.
        let edge_set: HashSet<usize> = all_edges.iter().copied().collect();
        let has_direct_neighbor = graph.edges.iter().enumerate().any(|(eidx, edge)| {
            !edge_set.contains(&eidx)
                && ((edge.node_a == graph.out_node && node_set.contains(&edge.node_b))
                    || (edge.node_b == graph.out_node && node_set.contains(&edge.node_a)))
        });
        assert!(
            has_direct_neighbor,
            "expected R_out's edge (excluded from all_edges) to be the direct-neighbor \
             match for out_node; if this fails, the fixture's resolution path changed \
             and the module doc comment above needs re-checking"
        );
    }

    #[test]
    fn multi_nullor_general_group_extracts_correct_tap_today() {
        // Regression guard: for THIS multi-nullor shape (resolved via the
        // direct-neighbor branch, not the first-nullor fallback — see the
        // test above), find_output_extract_node correctly returns U_lp's
        // out_node (the nullor actually wired to `out` via R_out), not
        // U_hp's (first-declared). If this ever starts returning U_hp's
        // node, either the direct-neighbor branch broke, or the fallback
        // shape changed underneath it — investigate before assuming this
        // is the pmv1 bug resurfacing (this exact path is safe by
        // construction, not by a hardened fallback).
        let (graph, all_edges) = khn_loop_graph();
        let node_set = collect_mna_nodes(&all_edges, &graph);

        let u_hp_idx = graph.components.iter().position(|c| c.id == "U_hp").unwrap();
        let u_lp_idx = graph.components.iter().position(|c| c.id == "U_lp").unwrap();
        let u_hp_out = graph
            .nullor_pins
            .iter()
            .find(|p| p.comp_idx == u_hp_idx)
            .expect("U_hp nullor pins")
            .out_node;
        let u_lp_out = graph
            .nullor_pins
            .iter()
            .find(|p| p.comp_idx == u_lp_idx)
            .expect("U_lp nullor pins")
            .out_node;

        let extracted = find_output_extract_node(&all_edges, &node_set, &graph);

        eprintln!(
            "[khn_loop] U_hp.out={u_hp_out} (first declared) U_lp.out={u_lp_out} \
             (wired to out via R_out) extracted={extracted:?}"
        );

        assert_eq!(
            extracted,
            Some(u_lp_out),
            "find_output_extract_node should extract U_lp's out_node \
             (U_lp.out={u_lp_out}, actually wired to `out`), not U_hp's \
             (first-declared, out_node={u_hp_out}). Got {extracted:?}."
        );
    }
}
