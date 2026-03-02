//! Pass 4: Unified tree builder.
//!
//! For each StagePlan:
//! 1. Build SP edges from plan (identical algorithm for all types)
//! 2. sp_reduce() → sp_to_dyn_with_vs() or sp_to_dyn()
//! 3. Create RootKind via factory function (single match on NonlinearKind)
//! 4. Balance VS impedance
//! 5. Package into WdfStage

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
// Stage building
// ═══════════════════════════════════════════════════════════════════════════

/// Build WDF stages from plans.
///
/// Each plan becomes one WdfStage through the same algorithm:
/// build SP edges → sp_reduce → sp_to_dyn → create root → package stage.
pub(super) fn build_stages(
    plans: &[StagePlan],
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    opamp_analysis: &OpAmpAnalysis,
    sample_rate: f64,
    oversampling: OversamplingFactor,
) -> Vec<WdfStage> {
    let vs_comp_idx = graph.components.len();

    // Build unity-gain feedback op-amp queue for JFET pairing.
    let mut feedback_opamp_queue = super::opamp_analysis::build_unity_gain_queue(opamp_analysis, sample_rate);

    let mut stages: Vec<WdfStage> = Vec::new();

    for plan in plans {
        let elem = &classified.nonlinear_elements[plan.element_idx];

        let stage = if plan.skip_vs {
            build_source_follower_stage(plan, elem, graph, sample_rate, oversampling)
        } else {
            build_vs_stage(plan, elem, graph, sample_rate, oversampling, vs_comp_idx)
        };

        if let Some(mut stage) = stage {
            // Pair JFET stages with feedback op-amps (for all-pass filters).
            if matches!(&elem.kind, NonlinearKind::Jfet { .. }) {
                if !feedback_opamp_queue.is_empty() {
                    stage.paired_opamp = Some(feedback_opamp_queue.remove(0));
                }
            }

            stages.push(stage);
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
        });
    }

    // Balance voltage source impedance in each stage.
    for stage in &mut stages {
        stage.balance_vs_impedance();
    }

    stages
}

/// Build push-pull stages from plans.
pub(super) fn build_push_pull_stages(
    push_pull_plans: &[PushPullPlan],
    classified: &ClassifiedCircuit,
    graph: &CircuitGraph,
    sample_rate: f64,
    oversampling: OversamplingFactor,
) -> Vec<PushPullStage> {
    let vs_comp_idx = graph.components.len();
    let mut stages = Vec::new();

    for pp_plan in push_pull_plans {
        let push_elem = &classified.nonlinear_elements[pp_plan.push_triode_list_idx];
        let pull_elem = &classified.nonlinear_elements[pp_plan.pull_triode_list_idx];

        let push_plan = super::plan::plan_push_pull_half(push_elem, classified, graph);
        let pull_plan = super::plan::plan_push_pull_half(pull_elem, classified, graph);

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
            coupled_stages.push(super::stage::CoupledBjtStage {
                bjt_stages,
                compensations,
                feedback_state: 0.0,
                feedback_scale: 0.1,
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
            let indep_stages = super::build::build_stages(
                &plans, classified, graph, opamp_analysis, sample_rate, oversampling,
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
            Some(stage) => {
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
    if n_nl < 2 || plan.passive_edge_indices.is_empty() {
        return None;
    }

    // ── Step 1: Collect unique circuit nodes and map to MNA indices ────
    // Ground is excluded from MNA (implicit reference).
    let mut node_set: Vec<NodeId> = Vec::new();
    let mut add_node = |node: NodeId| {
        if node != graph.gnd_node && !node_set.contains(&node) {
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
        if node == graph.gnd_node {
            None
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
            _ => {
                // Skip unknown edge types
            }
        }
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
    // Detect 3-port VariMu triode: single element with 2 NL terminal pairs.
    // In this case, create a VariMuThreePort device group instead of
    // individual NlDeviceKind devices.
    let is_three_port_vari_mu = plan.nl_element_indices.len() == 1
        && n_nl == 2
        && matches!(
            &classified.nonlinear_elements[plan.nl_element_indices[0]].kind,
            NonlinearKind::Triode { is_vari_mu: true, .. }
        );

    let (nl_devices, device_groups) = if is_three_port_vari_mu {
        let elem = &classified.nonlinear_elements[plan.nl_element_indices[0]];
        if let NonlinearKind::Triode { model_name, parallel_count, .. } = &elem.kind {
            let model = vari_mu_model(model_name);
            let three_port = VariMuThreePort::new(model).with_parallel_count(*parallel_count);
            let groups = MultiNlDeviceGroups {
                groups: vec![NlDeviceGroupKind::VariMuThreePort(three_port)],
                offsets: vec![0], // Single group starting at port 0
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
    // For 3-port VariMu: output is port 1 (plate-cathode).
    // For standard multi-NL: output is the NL element closest to output.
    let output_port = if is_three_port_vari_mu {
        1 // plate-cathode port
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
        _ => None, // DiodePair, Jfet, Mosfet, Zener, Ota, Pentode not yet supported in multi-NL
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
    let mut sp_edges: Vec<(usize, usize, SpTree)> = Vec::new();

    if let Some(vs_idx) = vs_comp_idx {
        sp_edges.push((plan.source_node, plan.injection_node, SpTree::Leaf(vs_idx)));
    }

    for &eidx in &plan.passive_idxs {
        let e = &graph.edges[eidx];
        sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
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
    let (sp_edges, virtual_edge_idx) = collect_sp_edges(plan, graph, Some(vs_comp_idx));
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
