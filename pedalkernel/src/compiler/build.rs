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
            // Source follower: no voltage source in tree.
            build_source_follower_stage(plan, elem, graph, sample_rate, oversampling)
        } else if plan.virtual_edge.is_some() {
            // 3-terminal element with virtual internal edge.
            build_virtual_edge_stage(plan, elem, graph, sample_rate, oversampling, vs_comp_idx)
        } else {
            // Simple 1-junction element with voltage source.
            build_simple_stage(plan, elem, graph, sample_rate, oversampling, vs_comp_idx)
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

        let push_plan = super::plan::plan_push_pull_half(push_elem, classified, graph, vs_comp_idx, sample_rate);
        let pull_plan = super::plan::plan_push_pull_half(pull_elem, classified, graph, vs_comp_idx, sample_rate);

        if let (Some(push_plan), Some(pull_plan)) = (push_plan, pull_plan) {
            let push_half = build_push_pull_half(&push_plan, push_elem, graph, sample_rate, vs_comp_idx);
            let pull_half = build_push_pull_half(&pull_plan, pull_elem, graph, sample_rate, vs_comp_idx);

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

// ═══════════════════════════════════════════════════════════════════════════
// Internal builders
// ═══════════════════════════════════════════════════════════════════════════

/// Build a simple stage with voltage source (diode, pentode, MOSFET, zener, OTA).
fn build_simple_stage(
    plan: &StagePlan,
    elem: &super::classify::NonlinearElement,
    graph: &CircuitGraph,
    sample_rate: f64,
    oversampling: OversamplingFactor,
    vs_comp_idx: usize,
) -> Option<WdfStage> {
    // Build SP edges.
    let mut sp_edges: Vec<(usize, usize, SpTree)> = Vec::new();
    sp_edges.push((plan.source_node, plan.injection_node, SpTree::Leaf(vs_comp_idx)));

    for &eidx in &plan.passive_idxs {
        let e = &graph.edges[eidx];
        sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
    }

    let sp_tree = sp_reduce(sp_edges, &plan.terminals).ok()?;

    // Build component list with virtual voltage source.
    let mut components = graph.components.clone();
    while components.len() <= vs_comp_idx {
        components.push(ComponentDef {
            id: "__vs__".to_string(),
            kind: ComponentKind::Resistor(1.0),
        });
    }

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
    let mut sp_edges: Vec<(usize, usize, SpTree)> = Vec::new();

    for &eidx in &plan.passive_idxs {
        let e = &graph.edges[eidx];
        sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
    }

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

/// Build a stage with a virtual internal edge (BJT r_ce, triode r_p).
fn build_virtual_edge_stage(
    plan: &StagePlan,
    elem: &super::classify::NonlinearElement,
    graph: &CircuitGraph,
    sample_rate: f64,
    oversampling: OversamplingFactor,
    vs_comp_idx: usize,
) -> Option<WdfStage> {
    let virtual_edge = plan.virtual_edge.as_ref().unwrap();
    let virtual_edge_idx = vs_comp_idx + 1;

    // Build SP edges.
    let mut sp_edges: Vec<(usize, usize, SpTree)> = Vec::new();
    sp_edges.push((plan.source_node, plan.injection_node, SpTree::Leaf(vs_comp_idx)));

    for &eidx in &plan.passive_idxs {
        let e = &graph.edges[eidx];
        sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
    }

    // Add virtual internal edge.
    sp_edges.push((
        virtual_edge.node_a,
        virtual_edge.node_b,
        SpTree::Leaf(virtual_edge_idx),
    ));

    let sp_tree = sp_reduce(sp_edges, &plan.terminals).ok()?;

    // Build component list with virtual elements.
    let mut components = graph.components.clone();
    while components.len() <= virtual_edge_idx {
        components.push(ComponentDef {
            id: "__vs__".to_string(),
            kind: ComponentKind::Resistor(1.0),
        });
    }
    components[virtual_edge_idx] = ComponentDef {
        id: virtual_edge.name.to_string(),
        kind: ComponentKind::Resistor(virtual_edge.resistance),
    };

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

/// Build a push-pull half tree.
fn build_push_pull_half(
    plan: &StagePlan,
    _elem: &super::classify::NonlinearElement,
    graph: &CircuitGraph,
    sample_rate: f64,
    vs_comp_idx: usize,
) -> Option<(DynNode, f64)> {
    let virtual_edge = plan.virtual_edge.as_ref()?;
    let virtual_rp_idx = vs_comp_idx + 1;

    let mut sp_edges: Vec<(usize, usize, SpTree)> = Vec::new();
    sp_edges.push((plan.source_node, plan.injection_node, SpTree::Leaf(vs_comp_idx)));

    for &eidx in &plan.passive_idxs {
        let e = &graph.edges[eidx];
        sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
    }
    sp_edges.push((virtual_edge.node_a, virtual_edge.node_b, SpTree::Leaf(virtual_rp_idx)));

    let sp_tree = sp_reduce(sp_edges, &plan.terminals).ok()?;

    let mut components = graph.components.clone();
    while components.len() <= virtual_rp_idx {
        components.push(ComponentDef {
            id: "__vs__".to_string(),
            kind: ComponentKind::Resistor(1.0),
        });
    }
    components[virtual_rp_idx] = ComponentDef {
        id: virtual_edge.name.to_string(),
        kind: ComponentKind::Resistor(virtual_edge.resistance),
    };

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
