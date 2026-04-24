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

    // Step 1b: Compute signal flow distance for each group via BFS from in_node.
    // Each group's distance = min BFS hop count from in_node to any node in the group.
    // This gives the correct processing order (input coupling first, clipping second,
    // tone third, etc.) regardless of the order find_flow_groups returned them.
    let group_flow_distances = compute_group_flow_distances(&feedback_groups, &graph);

    // Step 1c: Classify each group as signal path or static bias.
    // Static bias groups (VCC dividers) are bypassed in the serial audio
    // chain — they still process and meter, but don't overwrite the signal.
    let group_bias: Vec<_> = feedback_groups.iter()
        .map(|g| super::bias_analysis::classify_group_bias(g, &graph))
        .collect();

    // Build a map from node → DC bias voltage across all StaticBias groups.
    // Used to set op-amp v_max from the circuit's actual bias network.
    let mut bias_node_voltages: std::collections::HashMap<super::graph::NodeId, f64> =
        std::collections::HashMap::new();
    for kind in &group_bias {
        if let super::bias_analysis::GroupBiasKind::StaticBias { dc_voltages } = kind {
            bias_node_voltages.extend(dc_voltages);
        }
    }

    let supply_voltage = pedal.supplies.first().map_or(9.0, |s| s.config.voltage);

    // Step 2: SPQR decompose each group independently.
    let terminals = vec![graph.in_node, graph.out_node];
    let mut stages: Vec<Stage> = Vec::new();

    // Helper: push a BuiltStage into the unified stages vec.
    // `flow_distance` comes from BFS-computed signal flow distance.
    // `label` is the debug component names (zero cost in release).
    // `bypass` is true for static bias groups (not on audio path).
    macro_rules! push_stage {
        ($built:expr, $flow_distance:expr, $label:expr, $bypass:expr) => {
            match $built {
                BuiltStage::Wdf(mut wdf) => {
                    wdf.signal_flow_distance = $flow_distance;
                    wdf.bypass_serial = $bypass;
                    #[cfg(debug_assertions)]
                    { wdf.debug_label = $label; }
                    stages.push(Stage::Wdf(wdf));
                }
                BuiltStage::Iir(mut iir) => {
                    iir.signal_flow_distance = $flow_distance;
                    iir.bypass_serial = $bypass;
                    #[cfg(debug_assertions)]
                    { iir.debug_label = $label; }
                    stages.push(Stage::Iir(iir));
                }
                BuiltStage::StateSpace(mut ss) => {
                    ss.signal_flow_distance = $flow_distance;
                    ss.bypass_serial = $bypass;
                    #[cfg(debug_assertions)]
                    { ss.debug_label = $label; }
                    stages.push(Stage::StateSpace(ss));
                }
                BuiltStage::MultiNl(mut mnl) => {
                    mnl.signal_flow_distance = $flow_distance;
                    mnl.bypass_serial = $bypass;
                    #[cfg(debug_assertions)]
                    { mnl.debug_label = $label; }
                    stages.push(Stage::MultiNl(mnl));
                }
                BuiltStage::BlackFeedback(mut bf) => {
                    bf.signal_flow_distance = $flow_distance;
                    bf.bypass_serial = $bypass;
                    #[cfg(debug_assertions)]
                    { bf.debug_label = $label; }
                    stages.push(Stage::BlackFeedback(bf));
                }
            }
        };
    }

    // Build order: non-feedback groups first so the clipping stage can read
    // input coupling impedance (Ri) from already-built preceding stages.
    // Signal flow order is preserved via `gi` (original group index from
    // find_flow_groups), which becomes signal_flow_distance on each stage.
    let mut build_order: Vec<(usize, &super::signal_flow::FlowGroup)> =
        feedback_groups.iter().enumerate().collect();
    // Track which ground-clip groups were merged into another group's stage
    let mut ground_clip_built: std::collections::HashSet<usize> = std::collections::HashSet::new();
    build_order.sort_by_key(|(_, g)| if g.has_feedback() { 1 } else { 0 });

    for &(gi, group) in &build_order {
        if group.all_edges().is_empty() {
            continue;
        }

        // Build debug label from component names (zero cost in release)
        #[cfg(debug_assertions)]
        let group_label = {
            let mut names: Vec<&str> = group.all_edges().iter()
                .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
                .collect();
            names.dedup();
            names.join(",")
        };
        #[cfg(not(debug_assertions))]
        let group_label = String::new();

        // Static bias groups bypass the serial audio chain
        let is_bypass = matches!(
            group_bias[gi],
            super::bias_analysis::GroupBiasKind::StaticBias { .. }
        );

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
            // Compute bias-derived v_max for op-amps in this group.
            // Find the VCVS component, look up its pos/neg pin nodes in the
            // bias voltage map, and call apply_bias to get rail limits.
            let bias_v_max = compute_bias_v_max_for_group(group, &graph, &bias_node_voltages, supply_voltage);

            let mut built = build_rigid_from_group(
                group.all_edges(),
                &graph,
                sample_rate,
                Some(group),
                supply_voltage,
                bias_v_max,
            )
            .map_err(|e| format!("Group {gi}: {e}"))?;

            // Fix gain for feedback_opamp stages where Ri is in a preceding
            // stage. Find the input coupling group's edges touching the active
            // element's input pin, sum their resistance as Ri, compute gain.
            if let BuiltStage::Wdf(ref mut wdf) = built {
                if let Some(ref mut opamp) = wdf.feedback_opamp {
                    if opamp.gain().abs() <= 1.01 {
                        let input_node = group.active_edges.iter().find_map(|&eidx| {
                            let comp = &graph.components[graph.edges[eidx].comp_idx];
                            if let super::component::SignalTerminals::Amplifier { input, .. } = comp.kind.signal_terminals() {
                                let key = format!("{}.{input}", comp.id);
                                graph.node_names.get(&key).copied()
                            } else {
                                None
                            }
                        });

                        if let Some(neg) = input_node {
                            // Find the non-feedback group whose edges touch neg
                            // and sum their resistance as Ri.
                            let ri: f64 = build_order.iter()
                                .filter(|(_, g)| !g.has_feedback())
                                .flat_map(|(_, g)| g.all_edges())
                                .filter_map(|eidx| {
                                    let e = &graph.edges[eidx];
                                    if e.node_a != neg && e.node_b != neg { return None; }
                                    graph.components[e.comp_idx].kind.resistance()
                                })
                                .sum();

                            if ri > 0.0 {
                                let rf: f64 = group.feedback_edges.iter()
                                    .filter_map(|&eidx| graph.components[graph.edges[eidx].comp_idx].kind.resistance())
                                    .sum();
                                if rf > 0.0 {
                                    opamp.set_gain(rf / ri);
                                    wdf.feedback_ri = ri;
                                }
                            }
                        }
                    }
                }
            }

            // Same Ri fix for BlackFeedback stages
            if let BuiltStage::BlackFeedback(ref mut bf) = built {
                if bf.gain().abs() <= 1.01 {
                    let input_node = group.active_edges.iter().find_map(|&eidx| {
                        let comp = &graph.components[graph.edges[eidx].comp_idx];
                        if let super::component::SignalTerminals::Amplifier { input, .. } = comp.kind.signal_terminals() {
                            let key = format!("{}.{input}", comp.id);
                            graph.node_names.get(&key).copied()
                        } else {
                            None
                        }
                    });
                    if let Some(neg) = input_node {
                        let ri: f64 = build_order.iter()
                            .filter(|(_, g)| !g.has_feedback())
                            .flat_map(|(_, g)| g.all_edges())
                            .filter_map(|eidx| {
                                let e = &graph.edges[eidx];
                                if e.node_a != neg && e.node_b != neg { return None; }
                                graph.components[e.comp_idx].kind.resistance()
                            })
                            .sum();
                        if ri > 0.0 {
                            bf.set_ri(ri);
                        }
                    }
                }
            }

            push_stage!(built, group_flow_distances[gi], group_label.clone(), is_bypass);
        } else if is_pot_divider_group(group, &graph) {
            #[cfg(test)]
            eprintln!("  → POT DIVIDER group: {:?}", group.all_edges());
            // Pot voltage divider: both halves in one stage.
            // Build directly as Parallel(aw, wb) with ShortCircuit root.
            let built = build_pot_divider(group, &graph, sample_rate);
            match built {
                Ok(built_stage) => {
                    push_stage!(built_stage, group_flow_distances[gi], group_label.clone(), is_bypass);
                }
                Err(e) => return Err(format!("Group {gi} (pot): {e}")),
            }
        } else if is_ground_clip_group(group, &graph) {
            // Standalone NL element to ground (RAT/Klon hard-clip diodes).
            // Collect ALL ground-clip groups at the same signal node and merge
            // them into a single DiodePair stage. Skip individual groups that
            // were already merged into a preceding group.
            if !ground_clip_built.contains(&gi) {
                // Find all other ground-clip groups sharing the same signal node
                let signal_node = get_ground_clip_signal_node(group, &graph);
                let mut merged_edges: Vec<usize> = group.all_edges();
                for &(gj, other_group) in &build_order {
                    if gj != gi && !ground_clip_built.contains(&gj)
                        && is_ground_clip_group(other_group, &graph)
                        && get_ground_clip_signal_node(other_group, &graph) == signal_node
                    {
                        merged_edges.extend(other_group.all_edges());
                        ground_clip_built.insert(gj);
                    }
                }
                ground_clip_built.insert(gi);
                // Rebuild label from all merged edges (not just first group)
                #[cfg(debug_assertions)]
                let merged_label = {
                    let mut names: Vec<&str> = merged_edges.iter()
                        .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
                        .collect();
                    names.dedup();
                    names.join(",")
                };
                #[cfg(not(debug_assertions))]
                let merged_label = String::new();
                if let Some(built) = build_ground_clip_stage(&merged_edges, &graph, sample_rate) {
                    push_stage!(built, group_flow_distances[gi], merged_label, is_bypass);
                }
            }
        } else {
            // No feedback, not a pot, not a ground clip →
            // SPQR decompose for WDF/NlWdf stages.
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

            #[cfg(test)]
            {
                let node_type = if spqr_tree.is_rigid() { "R" }
                    else if matches!(&spqr_tree, super::spqr::SpqrNode::S { .. }) { "S" }
                    else if matches!(&spqr_tree, super::spqr::SpqrNode::P { .. }) { "P" }
                    else { "Q" };
                eprintln!("  SPQR result: {node_type}-node → {} stages", spqr_stages.len());
                if spqr_stages.is_empty() {
                    eprintln!("    WARNING: 0 stages from {node_type}-node ({} edges, {} terminals)",
                        group_edges.len(), group_terminals.len());
                    // Show what stage types were produced
                    for stage in &spqr_stages {
                        match stage {
                            super::spqr::SpqrStage::PassiveWdf { .. } => eprintln!("      → PassiveWdf"),
                            super::spqr::SpqrStage::NlWdf { .. } => eprintln!("      → NlWdf"),
                            super::spqr::SpqrStage::Rigid { .. } => eprintln!("      → Rigid"),
                        }
                    }
                }
            }

            for stage in spqr_stages {
                let built = build_spqr_stage(stage, &graph, sample_rate)
                    .map_err(|e| format!("Group {gi}: {e}"))?;
                push_stage!(built, group_flow_distances[gi], group_label.clone(), is_bypass);
            }
        }
    }

    // Sort stages by signal_flow_distance (original group index from
    // find_flow_groups) so processing order matches the circuit's signal chain.
    // Build order was different (non-feedback first for Ri extraction).
    stages.sort_by_key(|s| match s {
        Stage::Wdf(w) => w.signal_flow_distance,
        Stage::Iir(i) => i.signal_flow_distance,
        Stage::StateSpace(ss) => ss.signal_flow_distance,
        Stage::MultiNl(m) => m.signal_flow_distance,
        Stage::BlackFeedback(b) => b.signal_flow_distance,
    });

    // ── Feedforward detection ─────────────────────────────────────────────
    // Detect non-feedback stages that fan out from a shared source node
    // and converge at a summing point. These stages read from node_signals
    // instead of the serial chain, and add their output to the signal.
    //
    // Phase 1: collect (stage_index, injection_node) pairs.
    // Phase 2: apply flags to stages.
    {
        // Main path output nodes: active element outputs + in_node
        let mut main_path_output_nodes: std::collections::HashSet<super::graph::NodeId> =
            std::collections::HashSet::new();
        main_path_output_nodes.insert(graph.in_node);
        for group in feedback_groups.iter() {
            if !group.has_feedback() { continue; }
            for &eidx in group.active_edges.iter() {
                let e = &graph.edges[eidx];
                if let Some(pins) = graph.nullor_pins.iter().find(|p| p.comp_idx == e.comp_idx) {
                    main_path_output_nodes.insert(pins.out_node);
                }
            }
        }

        // Phase 1: collect feedforward candidates as (flow_distance, injection_node, first_comp_name)
        let mut ff_candidates: Vec<(usize, usize, String)> = Vec::new();
        for (gi, group) in feedback_groups.iter().enumerate() {
            if group.has_feedback() { continue; }

            let group_nodes: std::collections::HashSet<super::graph::NodeId> = group.all_edges().iter()
                .flat_map(|&eidx| {
                    let e = &graph.edges[eidx];
                    vec![e.node_a, e.node_b]
                })
                .collect();

            let source_node = group_nodes.iter()
                .find(|&&n| main_path_output_nodes.contains(&n))
                .copied();

            if let Some(src) = source_node {
                let dist = group_flow_distances[gi];
                let first_comp = group.all_edges().first()
                    .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.clone())
                    .unwrap_or_default();
                ff_candidates.push((dist, src, first_comp));
            }
        }

        // Check which distances have a feedback (main-path) stage.
        // A non-feedback stage is only feedforward if there's a feedback
        // stage at the same distance carrying the serial signal. If ALL
        // stages at a distance are non-feedback, they're serial, not
        // feedforward — marking them all would leave nobody on the chain.
        let mut dist_has_feedback: std::collections::HashSet<usize> =
            std::collections::HashSet::new();
        for stage in &stages {
            let (d, bp, is_fb) = match stage {
                Stage::BlackFeedback(b) => (b.signal_flow_distance, b.bypass_serial, true),
                Stage::Wdf(w) => (w.signal_flow_distance, w.bypass_serial, false),
                Stage::Iir(i) => (i.signal_flow_distance, i.bypass_serial, false),
                Stage::StateSpace(ss) => (ss.signal_flow_distance, ss.bypass_serial, false),
                Stage::MultiNl(m) => (m.signal_flow_distance, m.bypass_serial, false),
            };
            if !bp && is_fb {
                dist_has_feedback.insert(d);
            }
        }

        // Phase 2: apply feedforward flags — only at distances with a feedback stage
        for (dist, inj_node, comp_name) in &ff_candidates {
            if !dist_has_feedback.contains(dist) {
                continue;
            }
            for stage in &mut stages {
                if let Stage::Wdf(w) = stage {
                    if w.signal_flow_distance == *dist && !w.bypass_serial && !w.is_feedforward {
                        #[cfg(debug_assertions)]
                        let matches = w.debug_label.contains(comp_name.as_str());
                        #[cfg(not(debug_assertions))]
                        let matches = true;
                        if matches {
                            w.is_feedforward = true;
                            w.injection_node_id = *inj_node;
                            #[cfg(test)]
                            eprintln!("  → feedforward: [{}] inj_node={}", w.debug_label, inj_node);
                            break;
                        }
                    }
                }
            }
        }

        // Set output_node_id on upstream stages that feed feedforward stages.
        // The upstream stage is the one whose output node matches the feedforward's
        // injection_node_id. It must write to node_signals so feedforward stages
        // can read from it.
        let ff_inj_nodes: std::collections::HashSet<usize> = stages.iter()
            .filter_map(|s| {
                if let Stage::Wdf(w) = s {
                    if w.is_feedforward { Some(w.injection_node_id) } else { None }
                } else { None }
            })
            .collect();

        if !ff_inj_nodes.is_empty() {
            for stage in &mut stages {
                match stage {
                    Stage::Wdf(w) if !w.is_feedforward && !w.bypass_serial => {
                        // Check if this stage's group outputs to a feedforward injection node.
                        // The output node is the last node in the stage's signal path.
                        if w.output_node_id == usize::MAX {
                            // Find the group's output node from its debug label (match nullor_pins)
                            for pins in &graph.nullor_pins {
                                if ff_inj_nodes.contains(&pins.out_node) {
                                    // Check if this stage contains the active element
                                    #[cfg(debug_assertions)]
                                    {
                                        let comp_id = &graph.components[pins.comp_idx].id;
                                        if w.debug_label.contains(comp_id.as_str()) {
                                            w.output_node_id = pins.out_node;
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    Stage::BlackFeedback(b) if !b.bypass_serial => {
                        // BlackFeedback stages also need output_node_id for feedforward
                        if b.output_node_id == usize::MAX {
                            for pins in &graph.nullor_pins {
                                if ff_inj_nodes.contains(&pins.out_node) {
                                    #[cfg(debug_assertions)]
                                    {
                                        let comp_id = &graph.components[pins.comp_idx].id;
                                        if b.debug_label.contains(comp_id.as_str()) {
                                            b.output_node_id = pins.out_node;
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    _ => {}
                }
            }
        }
    }

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
    let comp0 = &graph.components[graph.edges[edges[0]].comp_idx];
    let comp1 = &graph.components[graph.edges[edges[1]].comp_idx];

    // Check for synthetic __aw/__wb split (legacy 3-terminal pot encoding)
    let is_aw_wb = (comp0.id.ends_with("__aw") && comp1.id.ends_with("__wb"))
        || (comp0.id.ends_with("__wb") && comp1.id.ends_with("__aw"));

    // Check for 2-edge pot (same component, both edges are the same pot)
    let is_same_pot = graph.edges[edges[0]].comp_idx == graph.edges[edges[1]].comp_idx
        && comp0.kind.is_pot();

    is_aw_wb || is_same_pot
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

    // Build both pot leaf nodes. Use the component's actual name (no __aw/__wb).
    // The signal-side half (NOT touching ground) gets marked as complement
    // so set_control applies 1-value: R_aw = (1-pos)*max_R.
    let mut leaves: Vec<DynNode> = Vec::new();
    for &eidx in &edges {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        if let Some(mut leaf) = comp.kind.make_leaf(&comp.id, sample_rate) {
            let touches_gnd = e.node_a == graph.gnd_node
                || e.node_b == graph.gnd_node
                || graph.ac_ground_nodes.contains(&e.node_a)
                || graph.ac_ground_nodes.contains(&e.node_b);
            if !touches_gnd {
                if let DynNode::Leaf(ref mut l) = leaf {
                    l.set_complement();
                }
            }
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
            let mut wdf = WdfStage::new(tree, root, oversampler);

            // Set output_probe for complex ground-terminated trees where
            // short_circuit_junction_voltage can't find the output node.
            // Simple trees (≤4 edges) work with junction extraction.
            // Complex trees (5+ edges, e.g. Screamer tone+output) need
            // explicit probing at the leaf connecting out_node toward GND.
            if touches_gnd && edge_indices.len() > 4 {
                let out_node = graph.out_node;
                let is_gnd = |n: super::graph::NodeId| -> bool {
                    n == graph.gnd_node || graph.ac_ground_nodes.contains(&n)
                };
                // Find edge at out_node whose other terminal goes toward GND
                // (is GND itself, or connects to GND through other edges).
                for &eidx in &edge_indices {
                    let e = &graph.edges[eidx];
                    let (touches_out, other) = if e.node_a == out_node {
                        (true, e.node_b)
                    } else if e.node_b == out_node {
                        (true, e.node_a)
                    } else {
                        (false, out_node) // dummy
                    };
                    if !touches_out { continue; }
                    // The GND-side edge: other terminal is GND or connects
                    // to GND through other group edges
                    let other_reaches_gnd = is_gnd(other) || edge_indices.iter().any(|&eidx2| {
                        let e2 = &graph.edges[eidx2];
                        (e2.node_a == other || e2.node_b == other)
                            && (is_gnd(e2.node_a) || is_gnd(e2.node_b))
                    });
                    if other_reaches_gnd {
                        let comp = &graph.components[e.comp_idx];
                        // Use the component ID as probe. For pots, make_leaf
                        // creates leaves with the base comp_id — leaf_matches_id
                        // will match "Volume" against "Volume" or "Volume__*".
                        wdf.output_probe = Some(comp.id.clone());
                        break;
                    }
                }
            }

            Ok(BuiltStage::Wdf(wdf))
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

/// Determine which groups are on the audio signal path (in_node → ... → out_node).
///
/// Check if a group is a ground-clip pattern: diode edges connecting signal to GND.
/// Only matches diode-family components (diode, diode_pair, zener), not BJTs/JFETs
/// which also have NL edges to ground but serve a different purpose.
fn is_ground_clip_group(
    group: &super::signal_flow::FlowGroup,
    graph: &super::graph::CircuitGraph,
) -> bool {
    use super::component::EdgeKind;
    let nl_edges: Vec<usize> = group.all_edges().iter()
        .filter(|&&eidx| graph.effective_edge_kind(eidx) == EdgeKind::Nonlinear)
        .copied()
        .collect();
    if nl_edges.is_empty() { return false; }
    nl_edges.iter().all(|&eidx| {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        let is_diode = comp.kind.is_diode_family();
        let to_gnd = e.node_a == graph.gnd_node || e.node_b == graph.gnd_node
            || graph.ac_ground_nodes.contains(&e.node_a)
            || graph.ac_ground_nodes.contains(&e.node_b);
        is_diode && to_gnd
    })
}

/// Get the non-GND signal node from a ground-clip group.
fn get_ground_clip_signal_node(
    group: &super::signal_flow::FlowGroup,
    graph: &super::graph::CircuitGraph,
) -> super::graph::NodeId {
    for &eidx in group.all_edges().iter() {
        let e = &graph.edges[eidx];
        if e.node_a != graph.gnd_node && !graph.ac_ground_nodes.contains(&e.node_a) {
            return e.node_a;
        }
        if e.node_b != graph.gnd_node && !graph.ac_ground_nodes.contains(&e.node_b) {
            return e.node_b;
        }
    }
    graph.gnd_node // fallback
}

/// Build a ground-clip WdfStage from merged NL edges (all to GND).
///
/// If two SingleDiode edges exist, synthesizes a DiodePair for antiparallel
/// clipping. Otherwise uses a single diode root.
fn build_ground_clip_stage(
    nl_edge_indices: &[usize],
    graph: &super::graph::CircuitGraph,
    _sample_rate: f64,
) -> Option<BuiltStage> {
    // Classify all NL edges
    let mut nl_kinds = Vec::new();
    for &eidx in nl_edge_indices {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        if let Some((kind, _)) = comp.kind.classify_nonlinear(
            &comp.id, e.node_a, e.node_b, graph.gnd_node, &graph.node_names,
        ) {
            nl_kinds.push(kind);
        }
    }
    if nl_kinds.is_empty() { return None; }

    // Synthesize DiodePair from two SingleDiode edges (antiparallel to ground)
    let (root, base_diode_model) = if nl_kinds.len() >= 2 {
        let mut pair_dt = None;
        for i in 0..nl_kinds.len() {
            for j in (i+1)..nl_kinds.len() {
                if let (
                    super::classify::NonlinearKind::SingleDiode(dt_a),
                    super::classify::NonlinearKind::SingleDiode(_),
                ) = (&nl_kinds[i], &nl_kinds[j]) {
                    pair_dt = Some(*dt_a);
                    break;
                }
            }
            if pair_dt.is_some() { break; }
        }
        if let Some(dt) = pair_dt {
            let model = super::helpers::diode_model(dt);
            (
                super::stage::RootKind::ExplicitDiodePair(
                    crate::elements::ExplicitDiodePairRoot::new(model),
                ),
                Some(model),
            )
        } else {
            super::build::create_root(&nl_kinds[0], false)
        }
    } else {
        super::build::create_root(&nl_kinds[0], false)
    };

    // VS with op-amp output impedance as source resistance.
    // In the real circuit, the diode sees the driving stage's output Z.
    // Typical op-amp output impedance: 50-200Ω. Using 100Ω as default.
    // This gives the WDF diode root the correct source impedance for
    // computing junction voltage and clipping behavior.
    let tree = DynNode::Leaf(Box::new(super::wdf_leaf::WdfVoltageSource {
        voltage: 0.0,
        rp: 100.0,
        is_cathode_bias: false,
    }));

    let oversampler = crate::oversampling::Oversampler::new(
        crate::oversampling::OversamplingFactor::X2,
    );
    let mut stage = super::stage::WdfStage::new(tree, root, oversampler);
    stage.base_diode_model = base_diode_model;

    Some(BuiltStage::Wdf(stage))
}

/// Compute SPQR terminals for a passive sub-group.
///
/// Compute signal flow distance for each group via BFS from in_node.
///
/// Each group's distance = minimum BFS hop count from `graph.in_node` to any
/// node touched by the group's edges. Groups closer to the circuit input get
/// Compute bias-derived v_max for an op-amp in a flow group.
///
/// Looks up the VCVS component's pos/neg pin nodes in the bias voltage
/// map, then calls `Component::apply_bias()` to get rail limits.
/// Returns the symmetric v_max (min of positive and negative rail).
/// Returns (v_rail_pos, v_rail_neg) for asymmetric rail clipping.
fn compute_bias_v_max_for_group(
    group: &super::signal_flow::FlowGroup,
    graph: &super::graph::CircuitGraph,
    bias_node_voltages: &std::collections::HashMap<super::graph::NodeId, f64>,
    supply_voltage: f64,
) -> Option<(f64, f64)> {
    use super::component::BiasResult;

    // Find the VCVS component in this group
    for &eidx in group.active_edges.iter() {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        if comp.kind.op_amp_type().is_none() {
            continue;
        }

        // Build bias_voltages map from pin names to DC voltages.
        let mut pin_voltages: std::collections::HashMap<String, f64> = std::collections::HashMap::new();

        if let Some(pins) = graph.nullor_pins.iter().find(|p| p.comp_idx == e.comp_idx) {
            if let Some(&v) = bias_node_voltages.get(&pins.pos_node) {
                pin_voltages.insert("pos".to_string(), v);
            }
            if let Some(&v) = bias_node_voltages.get(&pins.neg_node) {
                pin_voltages.insert("neg".to_string(), v);
            }
        }

        let result = comp.kind.apply_bias(&pin_voltages, supply_voltage);
        if let BiasResult::Applied { v_rail_pos, v_rail_neg } = result {
            return Some((v_rail_pos, v_rail_neg));
        }
    }

    None
}

/// lower distance values, giving the correct processing order regardless of
/// the order `find_flow_groups` returned them.
fn compute_group_flow_distances(
    groups: &[super::signal_flow::FlowGroup],
    graph: &super::graph::CircuitGraph,
) -> Vec<usize> {
    use std::collections::{HashMap, HashSet, VecDeque};
    use super::graph::NodeId;

    // BFS from in_node through all circuit edges to compute node distances.
    let mut node_dist: HashMap<NodeId, usize> = HashMap::new();
    let mut queue: VecDeque<NodeId> = VecDeque::new();
    node_dist.insert(graph.in_node, 0);
    queue.push_back(graph.in_node);

    // Build adjacency from ALL edges (not just group edges)
    let mut adj: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
    for e in &graph.edges {
        adj.entry(e.node_a).or_default().push(e.node_b);
        adj.entry(e.node_b).or_default().push(e.node_a);
    }

    while let Some(node) = queue.pop_front() {
        let dist = node_dist[&node];
        if let Some(neighbors) = adj.get(&node) {
            for &next in neighbors {
                if !node_dist.contains_key(&next) {
                    node_dist.insert(next, dist + 1);
                    queue.push_back(next);
                }
            }
        }
    }

    // For each group, find the minimum distance of any node it touches.
    groups
        .iter()
        .map(|group| {
            let mut min_dist = usize::MAX;
            for &eidx in group.all_edges().iter() {
                let e = &graph.edges[eidx];
                if let Some(&d) = node_dist.get(&e.node_a) {
                    min_dist = min_dist.min(d);
                }
                if let Some(&d) = node_dist.get(&e.node_b) {
                    min_dist = min_dist.min(d);
                }
            }
            if min_dist == usize::MAX { groups.len() } else { min_dist }
        })
        .collect()
}

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
        // Skip rail and AC ground nodes — they're DC references, not
        // signal boundaries. AC ground nodes (VB+ bias junctions) are
        // detected by the graph builder from capacitor-bypassed dividers.
        if node == graph.gnd_node
            || graph.supply_nodes.contains(&node)
            || graph.ac_ground_nodes.contains(&node)
        {
            continue;
        }
        // Global terminals (in_node, out_node) are only terminals for
        // THIS group if the group actually uses them as signal entry/exit.
        // A mid-chain group that happens to touch in_node via one edge
        // (e.g. R1 connecting to the global input) shouldn't get in_node
        // as a terminal — that edge is a pendant, not a signal boundary.
        // Check: is this global terminal also a boundary node (connects
        // to edges outside the group)? If not, it's pendant — skip it.
        if global_terminals.contains(&node) {
            let is_also_boundary = graph.edges.iter().enumerate().any(|(eidx, e)| {
                !group_edge_set.contains(&eidx)
                    && (e.node_a == node || e.node_b == node)
            });
            if is_also_boundary && !terminals.contains(&node) {
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

