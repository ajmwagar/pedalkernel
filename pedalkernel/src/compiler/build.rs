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
                let (push_root, pull_root) = build_push_pull_roots(push_elem, pull_elem);

                stages.push(PushPullStage {
                    push_tree,
                    pull_tree,
                    push_root,
                    pull_root,
                    push_oversampler: Oversampler::new(oversampling),
                    pull_oversampler: Oversampler::new(oversampling),
                    compensation: push_comp,
                    turns_ratio: pp_plan.turns_ratio,
                    grid_bias: -2.0,
                    cathode_delay_state: 0.0,
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
