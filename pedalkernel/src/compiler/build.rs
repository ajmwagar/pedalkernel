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

use super::classify::{ClassifiedCircuit, NonlinearKind};
use super::dyn_node::DynNode;
use super::graph::{CircuitGraph, SpTree, sp_reduce, sp_to_dyn};
use super::helpers::*;
use super::opamp_analysis::OpAmpAnalysis;
use super::plan::{StagePlan, PushPullPlan};
use super::stage::{PushPullStage, RootKind, TubeRoot, WdfStage};

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

                let push_tree = wrap_with_transformer_load(
                    push_tree,
                    xfmr_cfg.turns_ratio,
                    r_load,
                    xfmr_cfg.primary_inductance,
                    xfmr_cfg.primary_dcr,
                    pp_plan.is_ct_primary,
                    sample_rate,
                );
                let pull_tree = wrap_with_transformer_load(
                    pull_tree,
                    xfmr_cfg.turns_ratio,
                    r_load,
                    xfmr_cfg.primary_inductance,
                    xfmr_cfg.primary_dcr,
                    pp_plan.is_ct_primary,
                    sample_rate,
                );

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
                    cathode_delay_state: 0.0,
                    is_ct_primary: pp_plan.is_ct_primary,
                });
            }
        }
    }

    stages
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
fn build_push_pull_half(
    plan: &StagePlan,
    graph: &CircuitGraph,
    sample_rate: f64,
    vs_comp_idx: usize,
) -> Option<(DynNode, f64)> {
    let (sp_edges, virtual_edge_idx) = collect_sp_edges(plan, graph, Some(vs_comp_idx));
    let sp_tree = sp_reduce(sp_edges, &plan.terminals).ok()?;

    let ve_info = plan.virtual_edge.as_ref().zip(virtual_edge_idx);
    let components = build_components_with_virtuals(graph, vs_comp_idx, ve_info);

    let tree = sp_to_dyn_with_vs(&sp_tree, &components, &graph.fork_paths, sample_rate, vs_comp_idx);
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
