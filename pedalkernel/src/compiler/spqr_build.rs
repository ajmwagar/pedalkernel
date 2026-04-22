//! SPQR stage builder: converts `SpqrStage` descriptors into runnable WDF stages,
//! and provides the `compile_via_spqr()` entry point for the full pipeline.
//!
//! Separation of concerns:
//! - `spqr.rs`: graph decomposition + classification (topology + component semantics)
//! - `rigid/`: StageStats + RigidOptimization decision rules + rigid stage builders
//! - `spqr_build.rs`: stage construction + full pipeline entry point

use super::build::create_root;
use super::compiled::{CompiledPedal, RailSaturation, Stage, StageRef};
use super::dyn_node::DynNode;
use super::graph::{CircuitGraph, NodeId};
use super::rigid::{build_rigid, build_rigid_from_group};
use super::spqr::{spqr_decompose, spqr_to_stages, SpqrStage};
use super::stage::{IirStage, MultiNlStage, RootKind, StateSpaceStage, WdfStage};
use super::wdf_leaf::WdfVoltageSource;
use crate::dsl::PedalDef;
use crate::oversampling::{Oversampler, OversamplingFactor};

/// A stage built from the SPQR pipeline.
pub(super) enum BuiltStage {
    Wdf(WdfStage),
    Iir(IirStage),
    StateSpace(StateSpaceStage),
    MultiNl(MultiNlStage),
    BlackFeedback(super::stage::BlackFeedbackStage),
}

impl BuiltStage {
    /// Extract as WdfStage, panicking if it's a different type. For tests.
    #[cfg(test)]
    pub(super) fn into_wdf(self) -> WdfStage {
        match self {
            BuiltStage::Wdf(w) => w,
            _ => panic!("Expected WdfStage"),
        }
    }

    /// Extract as IirStage, panicking if it's a different type. For tests.
    #[cfg(test)]
    pub(super) fn into_iir(self) -> IirStage {
        match self {
            BuiltStage::Iir(i) => i,
            _ => panic!("Expected IirStage"),
        }
    }

    /// Extract as StateSpaceStage, panicking if it's a different type. For tests.
    #[cfg(test)]
    pub(super) fn into_state_space(self) -> StateSpaceStage {
        match self {
            BuiltStage::StateSpace(s) => s,
            _ => panic!("Expected StateSpaceStage"),
        }
    }

    /// Extract as MultiNlStage, panicking if it's a different type. For tests.
    #[cfg(test)]
    pub(super) fn into_multi_nl(self) -> MultiNlStage {
        match self {
            BuiltStage::MultiNl(m) => m,
            _ => panic!("Expected MultiNlStage"),
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Full pipeline entry point
// ═══════════════════════════════════════════════════════════════════════════

/// Compile a `.pedal` circuit via the SPQR pipeline with default options.
pub fn compile_via_spqr(
    pedal: &PedalDef,
    sample_rate: f64,
) -> Result<CompiledPedal, String> {
    compile_via_spqr_with_options(pedal, sample_rate, super::compile::CompileOptions::default())
}

/// Compile a `.pedal` circuit via the SPQR pipeline.
///
/// Full flow: PedalDef → CircuitGraph → feedback groups → per-group
/// SPQR decompose → classify → build stages → CompiledPedal.
///
/// Returns `Err` if any stage can't be built (unsupported topology).
pub fn compile_via_spqr_with_options(
    pedal: &PedalDef,
    sample_rate: f64,
    options: super::compile::CompileOptions,
) -> Result<CompiledPedal, String> {
    use super::signal_flow::find_flow_groups;

    let graph = CircuitGraph::from_pedal(pedal);

    // Collect all non-bridge edges (passive + NL + VCVS)
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    if all_edges.is_empty() {
        return Err("No circuit edges found".to_string());
    }

    // Step 1: Partition edges into signal flow groups.
    // Each group = mutually-dependent components that must share a stage.
    // Uses directed dependency graph: cycles = co-solved, acyclic = sequential.
    let feedback_groups = super::signal_flow::find_flow_groups(&all_edges, &graph);

    // Step 2: SPQR decompose each group independently.
    let terminals = vec![graph.in_node, graph.out_node];
    let mut stages: Vec<Stage> = Vec::new();
    let mut stage_counter = 0usize;

    // Helper: push a BuiltStage into the unified stages vec.
    macro_rules! push_stage {
        ($built:expr, $counter:expr) => {
            match $built {
                BuiltStage::Wdf(mut wdf) => {
                    wdf.signal_flow_distance = $counter;
                    stages.push(Stage::Wdf(wdf));
                }
                BuiltStage::Iir(mut iir) => {
                    iir.signal_flow_distance = $counter;
                    stages.push(Stage::Iir(iir));
                }
                BuiltStage::StateSpace(mut ss) => {
                    ss.signal_flow_distance = $counter;
                    stages.push(Stage::StateSpace(ss));
                }
                BuiltStage::MultiNl(mut mnl) => {
                    mnl.signal_flow_distance = $counter;
                    stages.push(Stage::MultiNl(mnl));
                }
                BuiltStage::BlackFeedback(mut bf) => {
                    bf.signal_flow_distance = $counter;
                    stages.push(Stage::BlackFeedback(bf));
                }
            }
        };
    }

    for (gi, group) in feedback_groups.iter().enumerate() {
        if group.all_edges().is_empty() {
            continue;
        }

        #[cfg(test)]
        {
            let edges = group.all_edges();
            let edge_names: Vec<String> = edges.iter().map(|&eidx| {
                let comp = &graph.components[graph.edges[eidx].comp_idx];
                format!("{}({:?})", comp.id, graph.effective_edge_kind(eidx))
            }).collect();
            eprintln!("group {gi}: feedback={} edges={:?}", group.has_feedback(), edge_names);
        }

        if group.has_feedback() {
            // Feedback group → build directly as Rigid.
            // Classification flows from FeedbackGroup to builder.
            let built = build_rigid_from_group(
                group.all_edges(),
                &graph,
                sample_rate,
                Some(group),
            )
            .map_err(|e| format!("Stage {stage_counter}: {e}"))?;
            push_stage!(built, stage_counter);
            stage_counter += 1;
        } else if is_pot_divider_group(group, &graph) {
            #[cfg(test)]
            eprintln!("  → POT DIVIDER group: {:?}", group.all_edges());
            // Pot voltage divider: both halves in one stage.
            // Build directly as Parallel(aw, wb) with ShortCircuit root.
            let built = build_pot_divider(group, &graph, sample_rate);
            match built {
                Ok(built_stage) => {
                    push_stage!(built_stage, stage_counter);
                }
                Err(e) => return Err(format!("Stage {stage_counter} (pot): {e}")),
            }
            stage_counter += 1;
        } else {
            // No feedback, not a pot → SPQR decompose for WDF/NlWdf stages.
            // Compute per-group terminals: use the group's actual boundary nodes
            // (nodes shared with other groups or the global in/out).
            // Include GND as a terminal if any edge touches ground — this lets
            // SPQR recognize ground-shunt caps as part of the SP structure
            // instead of forcing the whole group to Rigid.
            let group_edges = group.all_edges();
            let group_terminals = compute_group_terminals(&group_edges, &graph, &terminals);
            #[cfg(test)]
            eprintln!("  SPQR terminals for group: {:?}", group_terminals);
            let spqr_tree = spqr_decompose(
                &group_edges,
                &group_terminals,
                &graph,
                graph.gnd_node,
            );
            let spqr_stages = spqr_to_stages(&spqr_tree, &graph, sample_rate);

            for stage in spqr_stages {
                let built = build_spqr_stage(stage, &graph, sample_rate)
                    .map_err(|e| format!("Stage {stage_counter}: {e}"))?;
                push_stage!(built, stage_counter);
                stage_counter += 1;
            }
        }
    }

    let supply_voltage = pedal.supplies.first().map_or(9.0, |s| s.config.voltage);

    let mut compiled = CompiledPedal {
        stages,
        push_pull_stages: Vec::new(),
        pre_gain: 1.0,
        output_gain: 1.0,
        rail_saturation: RailSaturation::None,
        rail_sat_oversampler: Oversampler::new(options.oversampling),
        sample_rate,
        controls: Vec::new(),
        gain_range: (0.0, 1.0),
        supply_voltage,
        oversampling: options.oversampling,
        lfos: Vec::new(),
        envelopes: Vec::new(),
        slew_limiters: Vec::new(),
        bbds: Vec::new(),
        delay_lines: Vec::new(),
        vcos: Vec::new(),
        vcas: Vec::new(),
        thermal: None,
        tolerance_seed: 0,
        opamp_stages: Vec::new(),
        power_supply: None,
        #[cfg(debug_assertions)]
        debug_stats: None,
        metrics_accumulator: None,
        metrics_buffer: None,
        input_loading: None,
        output_loading: None,
        output_dc_block: None,
        sidechains: Vec::new(),
        subcircuit_processors: Vec::new(),
        subcircuit_routing: Vec::new(),
        subcircuit_output_idx: None,
        subcircuit_outputs: Vec::new(),
        pot_smoothers: Vec::new(),
        pot_mirrors: std::collections::HashMap::new(),
        base_grid_bias: 0.0,
        multi_nl_recompute_counter: 0,
        node_signals: Vec::new(),
        triggers: Vec::new(),
        bbd_wet_mix: 0.5,
        bbd_mix_pot_id: None,
        original_passive_values: std::collections::HashMap::new(),
    };

    if pedal.calibrate {
        super::calibrate::calibrate_output(&mut compiled);
    }

    // Bind pot controls to their stages (WDF, IIR, MultiNl).
    super::spqr_control::bind_controls(pedal, &mut compiled);

    Ok(compiled)
}

// ═══════════════════════════════════════════════════════════════════════════
// Stage construction helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Check if a group is a merged pot pair (aw + wb of same component).
fn is_pot_divider_group(
    group: &super::signal_flow::FlowGroup,
    graph: &CircuitGraph,
) -> bool {
    let edges = group.all_edges();
    if edges.len() != 2 {
        return false;
    }
    let id0 = &graph.components[graph.edges[edges[0]].comp_idx].id;
    let id1 = &graph.components[graph.edges[edges[1]].comp_idx].id;
    // Both are pot halves of the same base component
    (id0.ends_with("__aw") && id1.ends_with("__wb"))
        || (id0.ends_with("__wb") && id1.ends_with("__aw"))
}

/// Build a pot voltage divider stage: Parallel(R_aw, R_wb) with ShortCircuit root.
///
/// The output is at the wiper (parallel junction). In WDF, the Parallel
/// adaptor's junction voltage IS the voltage divider output.
fn build_pot_divider(
    group: &super::signal_flow::FlowGroup,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<BuiltStage, String> {
    let edges = group.all_edges();

    // Build both pot leaf nodes
    let mut leaves: Vec<DynNode> = Vec::new();
    for &eidx in &edges {
        let comp = &graph.components[graph.edges[eidx].comp_idx];
        if let Some(leaf) = comp.kind.make_leaf(&comp.id, sample_rate) {
            leaves.push(leaf);
        }
    }

    if leaves.len() != 2 {
        return Err("Pot divider: expected 2 leaves".to_string());
    }

    // Series(R_aw, R_wb) — voltage divider from signal to ground.
    // Wiper is the junction between aw and wb.
    // ShortCircuit root = ground at the bottom of R_wb.
    // Output extracted at the junction (series_junction_voltage).
    let divider = DynNode::Series(Box::new(leaves.remove(0)), Box::new(leaves.remove(0)));

    // Tree: VS in series with the divider chain
    let tree = with_voltage_source(divider);

    let oversampler = Oversampler::new(OversamplingFactor::X1);
    Ok(BuiltStage::Wdf(WdfStage::new(tree, RootKind::ShortCircuit, oversampler)))
}

/// Wrap a passive DynNode tree with a voltage source input port.
///
/// The WDF tree needs a voltage source leaf to receive the input signal.
/// Creates `Series(VoltageSource, passive_tree)` — the standard WDF
/// topology where VS drives the tree and the root terminates it.
pub(super) fn with_voltage_source(passive_tree: DynNode) -> DynNode {
    let vs = DynNode::Leaf(Box::new(WdfVoltageSource {
        voltage: 0.0,
        rp: 1.0, // Small source impedance
        is_cathode_bias: false,
    }));
    DynNode::Series(Box::new(vs), Box::new(passive_tree))
}

/// Build a runnable `WdfStage` from an `SpqrStage`.
///
/// - **PassiveWdf**: VS + DynNode tree + Passthrough root
/// - **NlWdf**: VS + DynNode tree + NL root from Component::classify_nonlinear()
/// - **Rigid**: not yet supported (returns Err — build layer will handle IIR/OpAmpRoot/MNA)
pub(super) fn build_spqr_stage(
    stage: SpqrStage,
    graph: &CircuitGraph,
    _sample_rate: f64,
) -> Result<BuiltStage, String> {
    match stage {
        SpqrStage::PassiveWdf { tree, edge_indices, .. } => {
            let tree = with_voltage_source(tree);
            let oversampler = Oversampler::new(OversamplingFactor::X1);
            // Ground-terminated → ShortCircuit root. Floating → Passthrough.
            let touches_gnd = edge_indices.iter().any(|&eidx| {
                let e = &graph.edges[eidx];
                e.node_a == graph.gnd_node || e.node_b == graph.gnd_node
                    || graph.ac_ground_nodes.contains(&e.node_a)
                    || graph.ac_ground_nodes.contains(&e.node_b)
            });
            let root = if touches_gnd { RootKind::ShortCircuit } else { RootKind::Passthrough };
            Ok(BuiltStage::Wdf(WdfStage::new(tree, root, oversampler)))
        }
        SpqrStage::NlWdf { tree, nl_edge_idx, .. } => {
            let e = &graph.edges[nl_edge_idx];
            let comp = &graph.components[e.comp_idx];
            let (nl_kind, _junction_nodes) = comp
                .kind
                .classify_nonlinear(
                    &comp.id,
                    e.node_a,
                    e.node_b,
                    graph.gnd_node,
                    &graph.node_names,
                )
                .ok_or_else(|| {
                    format!("NL edge {} ({}) didn't classify", nl_edge_idx, comp.id)
                })?;

            let (root, base_diode_model) = create_root(&nl_kind, false);
            let tree = with_voltage_source(tree);
            let oversampler = Oversampler::new(OversamplingFactor::X1);
            let mut wdf_stage = WdfStage::new(tree, root, oversampler);
            wdf_stage.base_diode_model = base_diode_model;
            Ok(BuiltStage::Wdf(wdf_stage))
        }
        SpqrStage::Rigid {
            edge_indices,
            boundary_nodes,
            pendant_trees,
            ..
        } => build_rigid(edge_indices, boundary_nodes, pendant_trees, graph, _sample_rate),
    }
}

/// Compute SPQR terminals for a passive sub-group.
///
/// Instead of using the global [in_node, out_node], find the group's actual
/// boundary nodes — nodes that connect to edges outside the group or to the
/// global input/output. Includes GND if any edge touches ground, so SPQR
/// can recognize ground-shunt components in the SP structure.
pub(super) fn compute_group_terminals(
    group_edges: &[usize],
    graph: &CircuitGraph,
    global_terminals: &[NodeId],
) -> Vec<NodeId> {
    use std::collections::HashSet;

    // Collect all nodes touched by this group's edges
    let mut group_nodes: HashSet<NodeId> = HashSet::new();
    let mut touches_gnd = false;
    for &eidx in group_edges {
        let e = &graph.edges[eidx];
        group_nodes.insert(e.node_a);
        group_nodes.insert(e.node_b);
        if e.node_a == graph.gnd_node || e.node_b == graph.gnd_node
            || graph.ac_ground_nodes.contains(&e.node_a)
            || graph.ac_ground_nodes.contains(&e.node_b)
        {
            touches_gnd = true;
        }
    }

    // Terminals = nodes in this group that are also global terminals
    // or that connect to edges NOT in this group (boundary nodes).
    let group_edge_set: HashSet<usize> = group_edges.iter().copied().collect();
    let mut terminals: Vec<NodeId> = Vec::new();

    for &node in &group_nodes {
        if node == graph.gnd_node || graph.supply_nodes.contains(&node) {
            continue; // GND handled separately below
        }
        // Is this a global terminal?
        if global_terminals.contains(&node) {
            if !terminals.contains(&node) {
                terminals.push(node);
            }
            continue;
        }
        // Does this node connect to edges outside the group?
        let is_boundary = graph.edges.iter().enumerate().any(|(eidx, e)| {
            !group_edge_set.contains(&eidx)
                && (e.node_a == node || e.node_b == node)
        });
        if is_boundary && !terminals.contains(&node) {
            terminals.push(node);
        }
    }

    // Do NOT include GND as a terminal — the pendant extraction in SPQR
    // handles ground-terminated edges correctly (GND has degree 1, pendant
    // extraction removes the cap, reducing the junction's degree for series
    // detection).

    // Fallback: if no terminals found, use global
    if terminals.is_empty() {
        return global_terminals.to_vec();
    }

    terminals
}

