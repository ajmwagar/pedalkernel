//! SPQR stage builder: converts `SpqrStage` descriptors into runnable WDF stages,
//! and provides the `compile_via_spqr()` entry point for the full pipeline.
//!
//! Separation of concerns:
//! - `spqr.rs`: graph decomposition + classification (topology + component semantics)
//! - `rigid/`: StageStats + RigidOptimization decision rules + rigid stage builders
//! - `spqr_build.rs`: stage construction + full pipeline entry point

use super::build::create_root;
use super::classify::NonlinearKind;
use super::compiled::{CompiledPedal, RailSaturation, Stage, StageRef};
use super::dyn_node::DynNode;
use super::graph::{CircuitGraph, NodeId};
use super::rigid::{
    build_general_mna_from_edges, build_general_mna_from_edges_with_hints, build_rigid,
    build_rigid_from_group, build_rigid_from_group_with_hints, build_rigid_without_iir,
};
use super::spqr::{spqr_decompose, spqr_to_stages, SpqrStage};
use super::stage::{IirStage, MultiNlStage, RootKind, StateSpaceStage, WdfStage};
use super::wdf_leaf::{LeafKind, WdfLeaf, WdfVoltageSource};
use crate::dsl::PedalDef;
use crate::oversampling::{Oversampler, OversamplingFactor};
use pedalkernel_rt::boundary_math::{PotStamp as RuntimePotStamp, WdfPortTerminals};
use pedalkernel_rt::tree::{MnaSystem, WdfPort};

fn output_coupling_dc_block(
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Option<(f64, f64, f64, f64)> {
    let output_node = graph.out_node;
    let mut output_cap = None;
    let mut output_load = None;

    for edge in &graph.edges {
        let comp = &graph.components[edge.comp_idx];
        let touches_output = edge.node_a == output_node || edge.node_b == output_node;
        if !touches_output {
            continue;
        }

        if let Some(c) = comp.kind.capacitance() {
            output_cap = Some(c);
        }

        if let Some(r) = comp.kind.resistance() {
            let other = if edge.node_a == output_node {
                edge.node_b
            } else {
                edge.node_a
            };
            if other == graph.gnd_node || graph.ac_ground_nodes.contains(&other) {
                output_load = Some(r);
            }
        }
    }

    let (Some(c), Some(r)) = (output_cap, output_load) else {
        return None;
    };
    if !(c.is_finite() && c > 0.0 && r.is_finite() && r > 0.0 && sample_rate > 0.0) {
        return None;
    }

    let rc = r * c;
    let alpha = rc / (rc + 1.0 / sample_rate);
    Some((alpha, alpha, 0.0, 0.0))
}

/// A stage built from the SPQR pipeline.
pub(super) enum BuiltStage {
    Wdf(WdfStage),
    Iir(IirStage),
    StateSpace(StateSpaceStage),
    MultiNl(MultiNlStage),
    BlackFeedback(super::stage::BlackFeedbackStage),
    Blockwise(pedalkernel_rt::stage::BlockwiseStage),
    SerialDelayedFeedback(pedalkernel_rt::stage::SerialDelayedFeedbackStage),
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
pub fn compile_via_spqr(pedal: &PedalDef, sample_rate: f64) -> Result<CompiledPedal, String> {
    compile_via_spqr_with_options(
        pedal,
        sample_rate,
        super::compile::CompileOptions::default(),
    )
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

    let mut graph = CircuitGraph::from_pedal(pedal);
    let supply_voltage = pedal.supplies.first().map_or(9.0, |s| s.config.voltage);

    // When ports are declared, the first input port replaces `in` and
    // the first output port replaces `out` as the circuit's I/O nodes.
    if !pedal.ports.is_empty() {
        if let Some(first_in) = pedal
            .ports
            .iter()
            .find(|p| p.direction == pedalkernel_rt::PortDirection::Input)
        {
            if let Some(&node_id) = graph.node_names.get(&first_in.name) {
                graph.in_node = node_id;
            }
        }
        if let Some(first_out) = pedal
            .ports
            .iter()
            .find(|p| p.direction == pedalkernel_rt::PortDirection::Output)
        {
            if let Some(&node_id) = graph.node_names.get(&first_out.name) {
                graph.out_node = node_id;
            }
        }
    }

    // Collect all non-bridge edges (passive + NL + VCVS)
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    if all_edges.is_empty() {
        return Err("No circuit edges found".to_string());
    }

    if options.collapse_nl {
        let stage = build_general_mna_from_edges_with_hints(
            &all_edges,
            &graph,
            sample_rate,
            &pedal.init_hints,
        )?;
        let mut compiled = CompiledPedal {
            stages: vec![Stage::MultiNl(stage)],
            stage_graph: pedalkernel_rt::processor::StageGraph::default(),
            stage_route_plan: pedalkernel_rt::processor::StageRoutePlan::default(),
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
            metrics_accumulator: None,
            metrics_buffer: None,
            input_loading: None,
            output_loading: None,
            output_dc_block: output_coupling_dc_block(&graph, sample_rate),
            sidechains: Vec::new(),
            subcircuit_processors: Vec::new(),
            subcircuit_routing: Vec::new(),
            subcircuit_output_idx: None,
            subcircuit_outputs: Vec::new(),
            pot_smoothers: Vec::new(),
            wiper_dividers: Vec::new(),
            pot_mirrors: hashbrown::HashMap::new(),
            base_grid_bias: 0.0,
            multi_nl_recompute_counter: 0,
            node_signals: Vec::new(),
            triggers: Vec::new(),
            bbd_wet_mix: 0.5,
            bbd_mix_pot_id: None,
            original_passive_values: hashbrown::HashMap::new(),
            ports: Vec::new(),
            port_values: Vec::new(),
            initialized: false,
        };
        super::spqr_control::bind_controls(pedal, &mut compiled);
        return Ok(compiled);
    }

    // Step 1: Partition edges into signal flow groups.
    // Each group = mutually-dependent components that must share a stage.
    // Uses directed dependency graph: cycles = co-solved, acyclic = sequential.
    eprintln!(
        "  [compile] Step 1: find_flow_groups ({} edges)...",
        all_edges.len()
    );
    let mut feedback_groups = super::signal_flow::find_flow_groups(&all_edges, &graph);
    merge_cross_reactive_groups_into_active_groups(&mut feedback_groups, &graph);
    eprintln!("  [compile] Step 1 done: {} groups", feedback_groups.len());

    // Step 1b: Compute signal flow distance for each group via BFS from in_node.
    // Each group's distance = min BFS hop count from in_node to any node in the group.
    // This gives the correct processing order (input coupling first, clipping second,
    // tone third, etc.) regardless of the order find_flow_groups returned them.
    let group_flow_distances = compute_group_flow_distances(&feedback_groups, &graph);

    // Step 1c: Classify each group as signal path or static bias.
    // Static bias groups (VCC dividers) are bypassed in the serial audio
    // chain — they still process and meter, but don't overwrite the signal.
    let group_bias: Vec<_> = feedback_groups
        .iter()
        .map(|g| super::bias_analysis::classify_group_bias(g, &graph))
        .collect();

    // Build a map from node → DC bias voltage across all StaticBias groups.
    // Used to set op-amp v_max from the circuit's actual bias network.
    let mut bias_node_voltages: std::collections::BTreeMap<super::graph::NodeId, f64> =
        std::collections::BTreeMap::new();
    for kind in &group_bias {
        if let super::bias_analysis::GroupBiasKind::StaticBias { dc_voltages } = kind {
            bias_node_voltages.extend(dc_voltages);
        }
    }

    // Step 2: SPQR decompose each group independently.
    let terminals = vec![graph.in_node, graph.out_node];
    let mut stages: Vec<Stage> = Vec::new();
    let mut stage_comp_ids: Vec<Vec<String>> = Vec::new();
    let mut bkm_consumed_comp_ids: std::collections::HashSet<String> =
        std::collections::HashSet::new();

    // Helper: push a BuiltStage into the unified stages vec.
    // `flow_distance` comes from BFS-computed signal flow distance.
    // `label` is the debug component names (zero cost in release).
    // `bypass` is true for static bias groups (not on audio path).
    macro_rules! push_stage {
        ($built:expr, $flow_distance:expr, $label:expr, $bypass:expr, $comp_ids:expr) => {
            match $built {
                BuiltStage::Wdf(mut wdf) => {
                    wdf.signal_flow_distance = $flow_distance;
                    wdf.bypass_serial = $bypass;
                    #[cfg(debug_assertions)]
                    {
                        wdf.debug_label = $label;
                    }
                    // Generate K-method lookup table for NL roots
                    // Skipped when skip_k_tables is set (debug builds).
                    if wdf.k_table.is_none()
                        && !options.skip_k_tables
                        && root_supports_k_table(&wdf.root)
                    {
                        let stage_n = stages.len();
                        eprintln!("  K-table: generating for stage {stage_n}...");
                        wdf.k_table = super::k_method::generate_k_table(&mut wdf);
                        if let Some(ref kt) = wdf.k_table {
                            eprintln!(
                                "  K-table generated: {}D, {} entries",
                                kt.dims,
                                kt.entries.len()
                            );
                        }
                    }
                    stages.push(Stage::Wdf(wdf));
                    stage_comp_ids.push($comp_ids.clone());
                }
                BuiltStage::Iir(mut iir) => {
                    iir.signal_flow_distance = $flow_distance;
                    iir.bypass_serial = $bypass;
                    #[cfg(debug_assertions)]
                    {
                        iir.debug_label = $label;
                    }
                    stages.push(Stage::Iir(iir));
                    stage_comp_ids.push($comp_ids.clone());
                }
                BuiltStage::StateSpace(mut ss) => {
                    ss.signal_flow_distance = $flow_distance;
                    ss.bypass_serial = $bypass;
                    #[cfg(debug_assertions)]
                    {
                        ss.debug_label = $label;
                    }
                    stages.push(Stage::StateSpace(ss));
                    stage_comp_ids.push($comp_ids.clone());
                }
                BuiltStage::MultiNl(mut mnl) => {
                    mnl.signal_flow_distance = $flow_distance;
                    mnl.bypass_serial = $bypass;
                    #[cfg(debug_assertions)]
                    {
                        mnl.debug_label = $label;
                    }
                    stages.push(Stage::MultiNl(mnl));
                    stage_comp_ids.push($comp_ids.clone());
                }
                BuiltStage::BlackFeedback(mut bf) => {
                    bf.signal_flow_distance = $flow_distance;
                    bf.bypass_serial = $bypass;
                    #[cfg(debug_assertions)]
                    {
                        bf.debug_label = $label;
                    }
                    stages.push(Stage::BlackFeedback(bf));
                    stage_comp_ids.push($comp_ids.clone());
                }
                BuiltStage::Blockwise(mut bkm) => {
                    let mut consumed: std::collections::HashSet<String> = bkm
                        .coupling_elements
                        .iter()
                        .map(|element| element.comp_id.clone())
                        .chain(
                            bkm.coupling_passives
                                .iter()
                                .map(|passive| passive.comp_id.clone()),
                        )
                        .collect();
                    consumed.remove("");
                    if !consumed.is_empty() {
                        for idx in (0..stages.len()).rev() {
                            let ids = &stage_comp_ids[idx];
                            if !ids.is_empty() && ids.iter().all(|id| consumed.contains(id)) {
                                stages.remove(idx);
                                stage_comp_ids.remove(idx);
                            }
                        }
                        bkm_consumed_comp_ids.extend(consumed);
                    }
                    // Blockwise stages set their own flow distance (0 = primary
                    // signal path, processes before output coupling stages).
                    // Don't override with the feedback group's inflated distance.
                    bkm.bypass_serial = $bypass;
                    bkm.init_buffers();
                    stages.push(Stage::Blockwise(bkm));
                    stage_comp_ids.push($comp_ids.clone());
                }
                BuiltStage::SerialDelayedFeedback(mut serial) => {
                    serial.signal_flow_distance = $flow_distance;
                    serial.bypass_serial = $bypass;
                    #[cfg(debug_assertions)]
                    {
                        serial.debug_label = $label;
                    }
                    stages.push(Stage::SerialDelayedFeedback(serial));
                    stage_comp_ids.push($comp_ids.clone());
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

        let group_comp_ids: Vec<String> = {
            let mut names: Vec<String> = group
                .all_edges()
                .iter()
                .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.clone())
                .collect();
            names.sort();
            names.dedup();
            names
        };
        if !group_comp_ids.is_empty()
            && group_comp_ids
                .iter()
                .all(|id| bkm_consumed_comp_ids.contains(id))
        {
            #[cfg(test)]
            eprintln!("  → group {gi} already consumed by BKM coupling");
            continue;
        }

        // Build debug label from component names (zero cost in release)
        #[cfg(debug_assertions)]
        let group_label = group_comp_ids.join(",");
        #[cfg(not(debug_assertions))]
        let group_label = String::new();

        // Static bias groups bypass the serial audio chain. Transistor-only
        // nonlinear modulator groups that do not reach the output also bypass:
        // they are solved for their local operating point but must not become
        // an independent serial audio stage.
        let is_bypass = matches!(
            group_bias[gi],
            super::bias_analysis::GroupBiasKind::StaticBias { .. }
        ) || is_nonlinear_modulator_group(group, &graph);

        #[cfg(test)]
        {
            let edges = group.all_edges();
            let edge_names: Vec<String> = edges
                .iter()
                .map(|&eidx| {
                    let comp = &graph.components[graph.edges[eidx].comp_idx];
                    format!("{}({:?})", comp.id, graph.effective_edge_kind(eidx))
                })
                .collect();
            eprintln!(
                "group {gi}: feedback={} edges={:?}",
                group.has_feedback(),
                edge_names
            );
        }

        if is_nonlinear_modulator_group(group, &graph) {
            #[cfg(test)]
            eprintln!("  → nonlinear modulator group consumed by control analysis");
            continue;
        }

        if group.has_feedback() {
            // ── Blockwise check for feedback groups (e.g. ladder with resonance)
            let group_edges = group.all_edges();
            if !options.skip_blockwise {
                eprintln!(
                    "  [compile] group {gi}: blockwise check ({} edges)...",
                    group_edges.len()
                );
                if let Some(built_stages) = super::blockwise::try_build_blockwise(
                    &group_edges,
                    &graph,
                    &terminals,
                    sample_rate,
                    &bias_node_voltages,
                    supply_voltage,
                    &pedal.ports,
                    options.force_serial_blockwise,
                    options.force_serial_blockwise_feedback_gain,
                    options.disable_iir,
                    options.coupled_blockwise_newton,
                    &pedal.init_hints,
                ) {
                    for built in built_stages {
                        push_stage!(
                            built,
                            group_flow_distances[gi],
                            group_label.clone(),
                            is_bypass,
                            group_comp_ids.clone()
                        );
                    }
                    continue;
                }
            }

            // Compute bias-derived v_max for op-amps in this group.
            // Find the VCVS component, look up its pos/neg pin nodes in the
            // bias voltage map, and call apply_bias to get rail limits.
            let bias_v_max =
                compute_bias_v_max_for_group(group, &graph, &bias_node_voltages, supply_voltage);

            let mut built = build_rigid_from_group_with_hints(
                group.all_edges(),
                &graph,
                sample_rate,
                Some(group),
                supply_voltage,
                bias_v_max,
                !options.disable_iir,
                &pedal.init_hints,
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
                            if let super::component::SignalTerminals::Amplifier { input, .. } =
                                comp.kind.signal_terminals()
                            {
                                let key = format!("{}.{input}", comp.id);
                                graph.node_names.get(&key).copied()
                            } else {
                                None
                            }
                        });

                        if let Some(neg) = input_node {
                            // Follow the ground-leg chain from neg through resistors
                            // to GND across ALL non-feedback groups. This captures
                            // multi-hop impedance like R5→R6→Gain_A→GND.
                            let non_fb_edges: Vec<usize> = build_order
                                .iter()
                                .filter(|(_, g)| !g.has_feedback())
                                .flat_map(|(_, g)| g.all_edges())
                                .collect();

                            // BFS from neg through resistive edges to GND.
                            // Track fixed resistors and pot components separately.
                            let is_gnd = |n: super::graph::NodeId| -> bool {
                                n == graph.gnd_node || graph.ac_ground_nodes.contains(&n)
                            };
                            let mut ri_fixed = 0.0f64;
                            let mut ri_pot_id: Option<String> = None;
                            let mut ri_pot_max_r = 0.0f64;
                            let mut ri_pot_taper = crate::dsl::PotTaper::B;
                            let mut ri_pot_initial_r = 0.0f64;
                            let mut visited = std::collections::HashSet::new();
                            let mut frontier = vec![neg];
                            visited.insert(neg);

                            while let Some(node) = frontier.pop() {
                                for &eidx in &non_fb_edges {
                                    let e = &graph.edges[eidx];
                                    let (touches, other) = if e.node_a == node {
                                        (true, e.node_b)
                                    } else if e.node_b == node {
                                        (true, e.node_a)
                                    } else {
                                        (false, node)
                                    };
                                    if !touches || visited.contains(&other) {
                                        continue;
                                    }
                                    let comp = &graph.components[e.comp_idx];
                                    if comp.kind.is_pot() {
                                        if let Some(max_r) = comp.kind.resistance() {
                                            ri_pot_id = Some(comp.id.clone());
                                            ri_pot_max_r = max_r;
                                            ri_pot_taper = comp
                                                .kind
                                                .pot_taper()
                                                .unwrap_or(crate::dsl::PotTaper::B);
                                            ri_pot_initial_r = max_r * 0.5; // default position
                                            ri_fixed += ri_pot_initial_r; // add initial pot R to total
                                            visited.insert(other);
                                            if !is_gnd(other) {
                                                frontier.push(other);
                                            }
                                        }
                                    } else if let Some(r) = comp.kind.resistance() {
                                        ri_fixed += r;
                                        visited.insert(other);
                                        if !is_gnd(other) {
                                            frontier.push(other);
                                        }
                                    }
                                }
                            }

                            // Also include pendant resistors from the feedback group
                            for &eidx in group.pendant_edges.iter() {
                                let e = &graph.edges[eidx];
                                if e.node_a != neg && e.node_b != neg {
                                    continue;
                                }
                                if let Some(r) = graph.components[e.comp_idx].kind.resistance() {
                                    ri_fixed += r;
                                }
                            }

                            let ri = ri_fixed;
                            if ri > 0.0 {
                                let rf: f64 = group
                                    .feedback_edges
                                    .iter()
                                    .filter_map(|&eidx| {
                                        graph.components[graph.edges[eidx].comp_idx]
                                            .kind
                                            .resistance()
                                    })
                                    .sum();
                                if rf > 0.0 {
                                    opamp.set_gain(rf / ri);
                                    wdf.feedback_ri = ri;
                                    if let Some(pot_id) = ri_pot_id {
                                        wdf.feedback_ri_pot_id = Some(pot_id);
                                        wdf.feedback_ri_fixed_r = ri_fixed - ri_pot_max_r * 0.5;
                                        wdf.feedback_ri_pot_max_r = ri_pot_max_r;
                                        wdf.feedback_ri_pot_taper = ri_pot_taper;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // Ground-leg Ri fix for BlackFeedback stages — BFS chain from neg to GND.
            // Always run (not just for gain≈1) to find pot-controlled Ri.
            if let BuiltStage::BlackFeedback(ref mut bf) = built {
                {
                    let input_node = group.active_edges.iter().find_map(|&eidx| {
                        let comp = &graph.components[graph.edges[eidx].comp_idx];
                        if let super::component::SignalTerminals::Amplifier { input, .. } =
                            comp.kind.signal_terminals()
                        {
                            let key = format!("{}.{input}", comp.id);
                            graph.node_names.get(&key).copied()
                        } else {
                            None
                        }
                    });
                    if let Some(neg) = input_node {
                        // Compute Ri: BFS from neg through passive edges to
                        // GND. Finds the full ground-leg impedance chain
                        // (e.g. R5 → R6 → Gain_A → GND) regardless of which
                        // FlowGroup the edges belong to.
                        //
                        // Exclude:
                        // - Feedback edges (Rf path, neg→out)
                        // - Active edges (opamp itself)
                        // - Edges at opamp output nodes (summing inputs)
                        // - Edges at opamp input nodes (don't cross to pos bias)
                        // - Edges at the global in/out nodes
                        let feedback_set: std::collections::HashSet<usize> =
                            group.feedback_edges.iter().copied().collect();
                        let active_set: std::collections::HashSet<usize> =
                            group.active_edges.iter().copied().collect();
                        let mut barrier_nodes: std::collections::HashSet<super::graph::NodeId> =
                            graph
                                .nullor_pins
                                .iter()
                                .flat_map(|p| vec![p.out_node, p.pos_node, p.neg_node])
                                .collect();
                        barrier_nodes.insert(graph.in_node);
                        barrier_nodes.insert(graph.out_node);
                        // Allow traversal FROM neg (the starting point)
                        barrier_nodes.remove(&neg);

                        let is_gnd = |n: super::graph::NodeId| -> bool {
                            n == graph.gnd_node || graph.ac_ground_nodes.contains(&n)
                        };

                        let mut ri_fixed = 0.0f64;
                        let mut ri_pot_id: Option<String> = None;
                        let mut ri_pot_max_r = 0.0f64;
                        let mut visited = std::collections::HashSet::new();
                        let mut frontier = vec![neg];
                        visited.insert(neg);

                        while let Some(node) = frontier.pop() {
                            // Search ALL graph edges (not just non-feedback)
                            for (eidx, e) in graph.edges.iter().enumerate() {
                                if feedback_set.contains(&eidx) || active_set.contains(&eidx) {
                                    continue;
                                }
                                let (touches, other) = if e.node_a == node {
                                    (true, e.node_b)
                                } else if e.node_b == node {
                                    (true, e.node_a)
                                } else {
                                    (false, node)
                                };
                                if !touches {
                                    continue;
                                }
                                // Allow multiple edges to GND (it's a shared
                                // rail), but skip already-visited non-rail nodes
                                if visited.contains(&other) && !is_gnd(other) {
                                    continue;
                                }
                                // Don't cross into barrier nodes (opamp pins,
                                // global in/out — not ground-leg paths)
                                if barrier_nodes.contains(&other) {
                                    continue;
                                }
                                let comp = &graph.components[e.comp_idx];
                                // Only follow passive edges with resistance
                                if comp.kind.is_pot() {
                                    if let Some(max_r) = comp.kind.resistance() {
                                        ri_pot_id = Some(comp.id.clone());
                                        ri_pot_max_r = max_r;
                                        ri_fixed += max_r * 0.5; // default position
                                        visited.insert(other);
                                        if !is_gnd(other) {
                                            frontier.push(other);
                                        }
                                    }
                                } else if let Some(r) = comp.kind.resistance() {
                                    ri_fixed += r;
                                    visited.insert(other);
                                    if !is_gnd(other) {
                                        frontier.push(other);
                                    }
                                } else if comp.kind.capacitance().is_some() {
                                    // Reactive elements: traverse through
                                    // (C_gnd bypass) but don't add resistance
                                    visited.insert(other);
                                    if !is_gnd(other) {
                                        frontier.push(other);
                                    }
                                }
                            }
                        }

                        if ri_fixed > 0.0 {
                            bf.set_ri(ri_fixed);
                        }
                        // Store ground-leg pot mapping for runtime Ri updates
                        if let Some(pot_id) = ri_pot_id {
                            let fixed_without_pot = ri_fixed - ri_pot_max_r * 0.5;
                            bf.ri_pot_comp_id = Some(pot_id);
                            bf.ri_fixed_r = fixed_without_pot;
                            bf.ri_pot_max_r = ri_pot_max_r;
                        }
                    }
                }
            }

            push_stage!(
                built,
                group_flow_distances[gi],
                group_label.clone(),
                is_bypass,
                group_comp_ids.clone()
            );
        } else if is_pot_divider_group(group, &graph) {
            #[cfg(test)]
            eprintln!("  → POT DIVIDER group: {:?}", group.all_edges());
            // Pot voltage divider: both halves in one stage.
            // Build directly as Parallel(aw, wb) with ShortCircuit root.
            let built = build_pot_divider(group, &graph, sample_rate);
            match built {
                Ok(built_stage) => {
                    push_stage!(
                        built_stage,
                        group_flow_distances[gi],
                        group_label.clone(),
                        is_bypass,
                        group_comp_ids.clone()
                    );
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
                    if gj != gi
                        && !ground_clip_built.contains(&gj)
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
                    let mut names: Vec<&str> = merged_edges
                        .iter()
                        .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
                        .collect();
                    names.dedup();
                    names.join(",")
                };
                #[cfg(not(debug_assertions))]
                let merged_label = String::new();
                if let Some(built) = build_ground_clip_stage(&merged_edges, &graph, sample_rate) {
                    let merged_comp_ids: Vec<String> = {
                        let mut names: Vec<String> = merged_edges
                            .iter()
                            .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.clone())
                            .collect();
                        names.sort();
                        names.dedup();
                        names
                    };
                    #[cfg(test)]
                    eprintln!(
                        "  Ground-clip stage built: {:?}",
                        std::mem::discriminant(&built)
                    );
                    push_stage!(
                        built,
                        group_flow_distances[gi],
                        merged_label,
                        is_bypass,
                        merged_comp_ids
                    );
                }
            }
        } else {
            // Extract pot dividers from the group BEFORE SPQR processing.
            // Pots need dedicated pot-divider stages with correct complement
            // handling. If left inside a larger SPQR tree, the WDF wave
            // propagation produces V-shaped output at extreme positions.
            let group_edges = group.all_edges();
            let mut remaining_edges = group_edges.clone();

            // Find pot component indices with 2 edges in this group.
            // Only extract pots that are OUTPUT dividers (wiper → out or
            // downstream group). Pots whose wiper stays internal (gain
            // controls, impedance elements) must NOT be extracted.
            let group_edge_set: std::collections::HashSet<usize> =
                group_edges.iter().copied().collect();
            let mut pot_edge_pairs: Vec<(usize, Vec<usize>)> = Vec::new();
            {
                let mut pot_edges: std::collections::BTreeMap<usize, Vec<usize>> =
                    std::collections::BTreeMap::new();
                for &eidx in &group_edges {
                    let comp = &graph.components[graph.edges[eidx].comp_idx];
                    if comp.kind.is_pot() {
                        pot_edges
                            .entry(graph.edges[eidx].comp_idx)
                            .or_default()
                            .push(eidx);
                    }
                }
                for (comp_idx, edges) in pot_edges {
                    if edges.len() != 2 {
                        continue;
                    }
                    // Check if the pot's wiper node connects to edges OUTSIDE
                    // this group (it's an output divider) or only to edges
                    // INSIDE this group (it's an internal impedance element).
                    // Find the wiper node: the node shared by both pot edges.
                    let e0 = &graph.edges[edges[0]];
                    let e1 = &graph.edges[edges[1]];
                    let wiper_node = if e0.node_a == e1.node_a || e0.node_a == e1.node_b {
                        Some(e0.node_a)
                    } else if e0.node_b == e1.node_a || e0.node_b == e1.node_b {
                        Some(e0.node_b)
                    } else {
                        None
                    };
                    let is_output_divider = if let Some(wiper) = wiper_node {
                        // Wiper connects to edges outside this group?
                        graph.edges.iter().enumerate().any(|(eidx, e)| {
                            !group_edge_set.contains(&eidx)
                                && (e.node_a == wiper || e.node_b == wiper)
                        })
                    } else {
                        false
                    };
                    if is_output_divider {
                        pot_edge_pairs.push((comp_idx, edges));
                    }
                }
            }

            // Build each pot as a standalone pot-divider stage
            for (_, pot_edges) in &pot_edge_pairs {
                // Create a temporary FlowGroup with just the pot edges
                let pot_group = super::signal_flow::FlowGroup {
                    active_edges: Vec::new(),
                    feedback_edges: Vec::new(),
                    pendant_edges: pot_edges.clone(),
                    ground_shunt_edges: Vec::new(),
                };
                if let Ok(built) = build_pot_divider(&pot_group, &graph, sample_rate) {
                    let pot_comp_ids: Vec<String> = {
                        let mut names: Vec<String> = pot_edges
                            .iter()
                            .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.clone())
                            .collect();
                        names.sort();
                        names.dedup();
                        names
                    };
                    #[cfg(debug_assertions)]
                    let pot_label = {
                        let comp = &graph.components[graph.edges[pot_edges[0]].comp_idx];
                        comp.id.clone()
                    };
                    #[cfg(not(debug_assertions))]
                    let pot_label = String::new();
                    push_stage!(
                        built,
                        group_flow_distances[gi],
                        pot_label,
                        is_bypass,
                        pot_comp_ids
                    );
                    // Remove pot edges from remaining
                    for &eidx in pot_edges {
                        remaining_edges.retain(|&e| e != eidx);
                    }
                }
            }

            // Process remaining non-pot edges through SPQR
            let group_edges = remaining_edges;
            if group_edges.is_empty() {
                continue;
            } // All edges were pots

            let group_has_runtime_pot = group_edges
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].kind.is_pot());
            let group_has_nonlinear = group_edges.iter().any(|&eidx| {
                graph.effective_edge_kind(eidx) == super::component::EdgeKind::Nonlinear
            });
            if group_has_runtime_pot && group_has_nonlinear {
                #[cfg(test)]
                eprintln!("  → rigid whole-group: NL group contains runtime pot");
                let built = build_rigid_from_group_with_hints(
                    group_edges,
                    &graph,
                    sample_rate,
                    Some(group),
                    supply_voltage,
                    None,
                    !options.disable_iir,
                    &pedal.init_hints,
                )
                .map_err(|e| format!("Group {gi}: {e}"))?;
                push_stage!(
                    built,
                    group_flow_distances[gi],
                    group_label.clone(),
                    is_bypass,
                    group_comp_ids.clone()
                );
                continue;
            }

            if !group.has_feedback() {
                let feedback_nodes: std::collections::HashSet<super::graph::NodeId> =
                    feedback_groups
                        .iter()
                        .filter(|g| g.has_feedback())
                        .flat_map(|g| g.all_edges())
                        .flat_map(|eidx| {
                            let e = &graph.edges[eidx];
                            [e.node_a, e.node_b].into_iter()
                        })
                        .collect();
                let main_outputs: std::collections::HashSet<super::graph::NodeId> = graph
                    .nullor_pins
                    .iter()
                    .map(|pins| pins.out_node)
                    .chain(std::iter::once(graph.in_node))
                    .collect();

                let mut built_feedforward = false;
                for &eidx in &group_edges {
                    let e = &graph.edges[eidx];
                    let comp = &graph.components[e.comp_idx];
                    let Some(leaf) = comp.kind.make_leaf(&comp.id, sample_rate) else {
                        continue;
                    };

                    let source = if main_outputs.contains(&e.node_a)
                        && feedback_nodes.contains(&e.node_b)
                    {
                        Some(e.node_a)
                    } else if main_outputs.contains(&e.node_b) && feedback_nodes.contains(&e.node_a)
                    {
                        Some(e.node_b)
                    } else {
                        None
                    };

                    let Some(source) = source else {
                        continue;
                    };

                    let mut wdf = WdfStage::new(
                        with_voltage_source(leaf),
                        RootKind::Passthrough,
                        Oversampler::new(OversamplingFactor::X1),
                    );
                    wdf.signal_flow_distance = group_flow_distances[gi];
                    wdf.is_feedforward = true;
                    wdf.injection_node_id = source;
                    #[cfg(debug_assertions)]
                    {
                        wdf.debug_label = comp.id.clone();
                    }
                    stages.push(Stage::Wdf(wdf));
                    stage_comp_ids.push(vec![comp.id.clone()]);
                    built_feedforward = true;
                }

                if built_feedforward {
                    continue;
                }
            }

            // ── Blockwise check: can this group be split into chained NL blocks?
            if !options.skip_blockwise {
                if let Some(built_stages) = super::blockwise::try_build_blockwise(
                    &group_edges,
                    &graph,
                    &terminals,
                    sample_rate,
                    &bias_node_voltages,
                    supply_voltage,
                    &pedal.ports,
                    options.force_serial_blockwise,
                    options.force_serial_blockwise_feedback_gain,
                    options.disable_iir,
                    options.coupled_blockwise_newton,
                    &pedal.init_hints,
                ) {
                    for built in built_stages {
                        push_stage!(
                            built,
                            group_flow_distances[gi],
                            group_label.clone(),
                            is_bypass,
                            group_comp_ids.clone()
                        );
                    }
                    continue; // Skip normal SPQR path
                }
            } // !skip_blockwise

            let group_terminals = compute_group_terminals(&group_edges, &graph, &terminals);
            #[cfg(test)]
            eprintln!("  SPQR terminals for group: {:?}", group_terminals);

            {
                let spqr_tree =
                    spqr_decompose(&group_edges, &group_terminals, &graph, graph.gnd_node);
                let spqr_stages = spqr_to_stages(&spqr_tree, &graph, sample_rate);

                #[cfg(test)]
                {
                    let node_type = if spqr_tree.is_rigid() {
                        "R"
                    } else if matches!(&spqr_tree, super::spqr::SpqrNode::S { .. }) {
                        "S"
                    } else if matches!(&spqr_tree, super::spqr::SpqrNode::P { .. }) {
                        "P"
                    } else {
                        "Q"
                    };
                    eprintln!(
                        "  SPQR result: {node_type}-node → {} stages",
                        spqr_stages.len()
                    );
                }

                // Count NL stages from the SPQR decomposition.
                // Passive WDF stages (ShortCircuit/Passthrough roots) do not
                // process nonlinear devices. If the group has NL components but
                // SPQR produces no NlWdf or Rigid stages, the NL devices were
                // silently dropped as Q-nodes. Fall back to build_rigid_from_group
                // to ensure they're processed through the general MNA + NR path.
                let spqr_has_nl_stage = spqr_stages
                    .iter()
                    .any(|s| matches!(s, SpqrStage::NlWdf { .. } | SpqrStage::Rigid { .. }));

                if group_has_nonlinear && !spqr_has_nl_stage {
                    // NL devices were dropped by SPQR (cross-coupled topology with
                    // no R-node, e.g. BJT astable multivibrator). Use the general
                    // MNA path with init hints so all NL devices are compiled.
                    eprintln!(
                        "  [compile] group {gi}: SPQR dropped NL devices, falling back to rigid MNA"
                    );
                    let built = build_rigid_from_group_with_hints(
                        group_edges,
                        &graph,
                        sample_rate,
                        Some(group),
                        supply_voltage,
                        None,
                        !options.disable_iir,
                        &pedal.init_hints,
                    )
                    .map_err(|e| format!("Group {gi} (rigid fallback): {e}"))?;
                    push_stage!(
                        built,
                        group_flow_distances[gi],
                        group_label.clone(),
                        is_bypass,
                        group_comp_ids.clone()
                    );
                    continue;
                }

                for stage in spqr_stages {
                    let built = build_spqr_stage_with_options(
                        stage,
                        &graph,
                        sample_rate,
                        options.disable_iir,
                        &pedal.init_hints,
                        supply_voltage,
                    )
                    .map_err(|e| format!("Group {gi}: {e}"))?;
                    push_stage!(
                        built,
                        group_flow_distances[gi],
                        group_label.clone(),
                        is_bypass,
                        group_comp_ids.clone()
                    );
                }
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
        Stage::Blockwise(k) => k.signal_flow_distance,
        Stage::SerialDelayedFeedback(s) => s.signal_flow_distance,
    });

    // ── Feedforward detection ─────────────────────────────────────────────
    // Detect non-feedback stages that fan out from a shared source node
    // and converge at a summing point. These stages read from node_signals
    // instead of the serial chain, and add their output to the signal.
    //
    // Phase 1: collect (stage_index, injection_node) pairs.
    // Phase 2: apply flags to stages.
    {
        // BFS node distances for feedforward stage ordering
        let node_dist = {
            use std::collections::{BTreeMap, VecDeque};
            let mut dist: BTreeMap<usize, usize> = BTreeMap::new();
            let mut queue: VecDeque<usize> = VecDeque::new();
            dist.insert(graph.in_node, 0);
            queue.push_back(graph.in_node);
            let mut adj: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
            for e in &graph.edges {
                adj.entry(e.node_a).or_default().push(e.node_b);
                adj.entry(e.node_b).or_default().push(e.node_a);
            }
            while let Some(node) = queue.pop_front() {
                let d = dist[&node];
                if let Some(neighbors) = adj.get(&node) {
                    for &next in neighbors {
                        if !dist.contains_key(&next) {
                            dist.insert(next, d + 1);
                            queue.push_back(next);
                        }
                    }
                }
            }
            dist
        };

        // Main path output nodes: ALL active element outputs + in_node.
        // This includes both feedback group op-amps AND unity followers
        // (which have nullor_pins but no feedback group).
        let mut main_path_output_nodes: std::collections::HashSet<super::graph::NodeId> =
            std::collections::HashSet::new();
        main_path_output_nodes.insert(graph.in_node);
        for pins in &graph.nullor_pins {
            main_path_output_nodes.insert(pins.out_node);
        }

        // Collect feedback group input nodes (where feedforward paths converge).
        // A feedback group's input node is where non-feedback groups can feed into.
        let mut feedback_input_nodes: std::collections::HashSet<super::graph::NodeId> =
            std::collections::HashSet::new();
        let is_audio_convergence_node = |node: super::graph::NodeId| -> bool {
            node != graph.gnd_node
                && node != graph.vcc_node
                && !graph.supply_nodes.contains(&node)
                && !graph.ac_ground_nodes.contains(&node)
        };
        for group in feedback_groups.iter() {
            if !group.has_feedback() {
                continue;
            }
            for &eidx in group.active_edges.iter() {
                let e = &graph.edges[eidx];
                if let Some(pins) = graph.nullor_pins.iter().find(|p| p.comp_idx == e.comp_idx) {
                    if is_audio_convergence_node(pins.neg_node) {
                        feedback_input_nodes.insert(pins.neg_node);
                    }
                    if is_audio_convergence_node(pins.pos_node) {
                        feedback_input_nodes.insert(pins.pos_node);
                    }
                }
            }
        }

        // Phase 1: A non-feedback group is feedforward if:
        //   - It shares an input node with a main-path output (taps from main path)
        //   - It shares an output node with a feedback group's input (converges)
        // This uses actual node connectivity, not distance heuristics.
        let mut ff_candidates: Vec<(usize, usize, String)> = Vec::new();
        for (gi, group) in feedback_groups.iter().enumerate() {
            if group.has_feedback() {
                continue;
            }

            let group_nodes: std::collections::HashSet<super::graph::NodeId> = group
                .all_edges()
                .iter()
                .flat_map(|&eidx| {
                    let e = &graph.edges[eidx];
                    vec![e.node_a, e.node_b]
                })
                .collect();

            // Does this group tap from a main-path output?
            // Prefer op-amp output nodes over in_node — in_node is only
            // the source for the very first stage, not mid-chain feedforward.
            let source_node = group_nodes
                .iter()
                .find(|&&n| main_path_output_nodes.contains(&n) && n != graph.in_node)
                .or_else(|| group_nodes.iter().find(|&&n| n == graph.in_node))
                .copied();

            // Does this group converge at a feedback group's input?
            let converges = group_nodes.iter().any(|n| feedback_input_nodes.contains(n));

            if let Some(src) = source_node {
                if converges {
                    // This is a feedforward path: taps from main path AND
                    // converges at a feedback input. It's parallel to the
                    // main serial chain.
                    let dist = group_flow_distances[gi];
                    let first_comp = group
                        .all_edges()
                        .first()
                        .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.clone())
                        .unwrap_or_default();
                    ff_candidates.push((dist, src, first_comp));
                }
            }
        }

        // Phase 2: apply feedforward flags.
        // A candidate is feedforward if a feedback stage ALSO taps from the same
        // source. This means there's a main path (through the feedback stage) and
        // a parallel path (through this non-feedback group). Without a feedback
        // stage at the same source, the non-feedback group IS the serial chain.
        let sources_with_feedback: std::collections::HashSet<usize> = ff_candidates
            .iter()
            .filter_map(|(_, inj, _)| {
                // Check if any feedback group also taps from this source node
                let has_fb = feedback_groups.iter().any(|g| {
                    g.has_feedback()
                        && g.all_edges().iter().any(|&eidx| {
                            let e = &graph.edges[eidx];
                            e.node_a == *inj || e.node_b == *inj
                        })
                });
                if has_fb {
                    Some(*inj)
                } else {
                    None
                }
            })
            .collect();
        for (dist, inj_node, comp_name) in &ff_candidates {
            if !sources_with_feedback.contains(inj_node) {
                continue; // No feedback stage at this source — this is serial, not feedforward
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
                            // Set flow distance based on the injection node's
                            // BFS distance from in_node. The feedforward stage
                            // must process AFTER signal arrives at the tap point.
                            if let Some(&node_d) = node_dist.get(inj_node) {
                                w.signal_flow_distance = node_d;
                            }
                            #[cfg(test)]
                            eprintln!(
                                "  → feedforward: [{}] inj_node={} dist→{}",
                                w.debug_label, inj_node, w.signal_flow_distance
                            );
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
        let ff_inj_nodes: std::collections::HashSet<usize> = stages
            .iter()
            .filter_map(|s| {
                if let Stage::Wdf(w) = s {
                    if w.is_feedforward {
                        Some(w.injection_node_id)
                    } else {
                        None
                    }
                } else {
                    None
                }
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

    // Re-sort: feedforward stages may have changed distance.
    // Secondary key: feedforward stages sort AFTER non-feedforward at same distance.
    stages.sort_by_key(|s| {
        let (d, ff) = match s {
            Stage::Wdf(w) => (w.signal_flow_distance, w.is_feedforward),
            Stage::Iir(i) => (i.signal_flow_distance, false),
            Stage::StateSpace(ss) => (ss.signal_flow_distance, false),
            Stage::MultiNl(m) => (m.signal_flow_distance, false),
            Stage::BlackFeedback(b) => (b.signal_flow_distance, false),
            Stage::Blockwise(k) => (k.signal_flow_distance, false),
            Stage::SerialDelayedFeedback(s) => (s.signal_flow_distance, false),
        };
        (d, ff as u8) // false=0 sorts before true=1
    });

    stages.sort_by_key(|s| {
        let (d, ff) = match s {
            Stage::Wdf(w) => (w.signal_flow_distance, w.is_feedforward),
            Stage::Iir(i) => (i.signal_flow_distance, false),
            Stage::StateSpace(ss) => (ss.signal_flow_distance, false),
            Stage::MultiNl(m) => (m.signal_flow_distance, false),
            Stage::BlackFeedback(b) => (b.signal_flow_distance, false),
            Stage::Blockwise(k) => (k.signal_flow_distance, false),
            Stage::SerialDelayedFeedback(s) => (s.signal_flow_distance, false),
        };
        (d, ff as u8)
    });

    let mut compiled = CompiledPedal {
        stages,
        stage_graph: pedalkernel_rt::processor::StageGraph::default(),
        stage_route_plan: pedalkernel_rt::processor::StageRoutePlan::default(),
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
        metrics_accumulator: None,
        metrics_buffer: None,
        input_loading: None,
        output_loading: None,
        output_dc_block: output_coupling_dc_block(&graph, sample_rate),
        sidechains: Vec::new(),
        subcircuit_processors: Vec::new(),
        subcircuit_routing: Vec::new(),
        subcircuit_output_idx: None,
        subcircuit_outputs: Vec::new(),
        pot_smoothers: Vec::new(),
        wiper_dividers: Vec::new(),
        pot_mirrors: hashbrown::HashMap::new(),
        base_grid_bias: 0.0,
        multi_nl_recompute_counter: 0,
        node_signals: Vec::new(),
        triggers: Vec::new(),
        bbd_wet_mix: 0.5,
        bbd_mix_pot_id: None,
        original_passive_values: hashbrown::HashMap::new(),
        ports: Vec::new(),
        port_values: Vec::new(),
        initialized: false,
    };

    // Bind pot controls to their stages (WDF, IIR, MultiNl).
    super::spqr_control::bind_controls(pedal, &mut compiled);

    if pedal.calibrate {
        super::calibrate::calibrate_output(&mut compiled);
    }

    // Bind named ports: resolve port names to graph NodeIds.
    if !pedal.ports.is_empty() {
        let mut port_bindings = Vec::new();
        let mut port_values = Vec::new();
        for (i, port_def) in pedal.ports.iter().enumerate() {
            let node_id = graph
                .node_names
                .get(&port_def.name)
                .copied()
                .unwrap_or(usize::MAX);
            #[cfg(test)]
            if node_id == usize::MAX {
                eprintln!(
                    "  WARNING: port '{}' not found in graph node_names",
                    port_def.name
                );
            }
            port_bindings.push(pedalkernel_rt::processor::PortBinding {
                name: port_def.name.clone(),
                direction: port_def.direction,
                index: i,
                node_id,
                default_value: 0.0,
                stage_idx: usize::MAX, // resolved below for input ports
            });
            port_values.push(0.0);
        }
        // For each input port, find the component connected to the port
        // node and wrap its WDF leaf with a named VS in series.
        // The VS drives current through the component into the circuit,
        // modelling a voltage source at the port jack.
        for port_binding in &mut port_bindings {
            if port_binding.direction != pedalkernel_rt::PortDirection::Input {
                continue;
            }
            if port_binding.node_id == graph.in_node {
                continue; // Main input already has VS from with_voltage_source()
            }
            // Find the component connected to this port node
            let port_comp_id: Option<String> = graph
                .edges
                .iter()
                .find(|e| e.node_a == port_binding.node_id || e.node_b == port_binding.node_id)
                .map(|e| graph.components[e.comp_idx].id.clone());

            if let Some(comp_id) = port_comp_id {
                // Find the WDF stage containing this component and wrap
                // its leaf with a named VS in series. Record stage index
                // so runtime only touches this stage for this port.
                for (si, stage) in compiled.stages.iter_mut().enumerate() {
                    if let pedalkernel_rt::processor::Stage::Wdf(ref mut wdf) = stage {
                        let found = wdf.tree.wrap_leaf_with_vs(&comp_id, &port_binding.name);
                        if found {
                            wdf.tree.recompute();
                            port_binding.stage_idx = si;
                            #[cfg(test)]
                            eprintln!(
                                "  Injected port VS '{}' at leaf '{}' in stage {si}",
                                port_binding.name, comp_id
                            );
                            break;
                        }
                    }
                }
            }
        }

        compiled.ports = port_bindings;
        compiled.port_values = port_values;
    }

    // Cache raw pointers to all VS leaves for zero-cost runtime access.
    // Must be after port binding (wrap_leaf_with_vs) and recompute.
    compiled.cache_all_vs_pointers();
    compiled.stage_graph =
        build_compiled_stage_graph(&compiled.stages, &stage_comp_ids, &compiled.ports);
    compiled.stage_route_plan = pedalkernel_rt::processor::StageRoutePlan::from_compiled_parts(
        &compiled.stage_graph,
        &compiled.ports,
        &compiled.stages,
    );

    Ok(compiled)
}

// ═══════════════════════════════════════════════════════════════════════════
// Stage construction helpers
// ═══════════════════════════════════════════════════════════════════════════

fn build_compiled_stage_graph(
    stages: &[Stage],
    stage_comp_ids: &[Vec<String>],
    external_ports: &[pedalkernel_rt::processor::PortBinding],
) -> pedalkernel_rt::processor::StageGraph {
    use pedalkernel_rt::processor::{
        StageGraph, StageGraphConnection, StageGraphNode, StageGraphPort, StageGraphPortDirection,
    };

    fn wdf_dir(direction: pedalkernel_rt::stage::WdfBoundaryDirection) -> StageGraphPortDirection {
        match direction {
            pedalkernel_rt::stage::WdfBoundaryDirection::Input => StageGraphPortDirection::Input,
            pedalkernel_rt::stage::WdfBoundaryDirection::Output => StageGraphPortDirection::Output,
            pedalkernel_rt::stage::WdfBoundaryDirection::Control => {
                StageGraphPortDirection::Control
            }
        }
    }

    let mut graph = StageGraph::default();

    for (stage_idx, stage) in stages.iter().enumerate() {
        let component_ids = stage_comp_ids.get(stage_idx).cloned().unwrap_or_default();
        let mut ports = Vec::new();
        let (kind, label) = match stage {
            Stage::Wdf(wdf) => {
                for binding in &wdf.boundary_bindings {
                    ports.push(StageGraphPort {
                        label: binding.label.clone(),
                        node_id: binding.node_id,
                        direction: wdf_dir(binding.direction),
                    });
                }
                if ports.is_empty() {
                    if wdf.injection_node_id != usize::MAX {
                        ports.push(StageGraphPort {
                            label: "input".into(),
                            node_id: wdf.injection_node_id,
                            direction: StageGraphPortDirection::Input,
                        });
                    }
                    if wdf.output_node_id != usize::MAX {
                        ports.push(StageGraphPort {
                            label: "output".into(),
                            node_id: wdf.output_node_id,
                            direction: StageGraphPortDirection::Output,
                        });
                    }
                }
                ("Wdf".to_string(), wdf.root_comp_id.clone())
            }
            Stage::MultiNl(mnl) => {
                if mnl.injection_node_id != usize::MAX {
                    ports.push(StageGraphPort {
                        label: "input".into(),
                        node_id: mnl.injection_node_id,
                        direction: StageGraphPortDirection::Input,
                    });
                }
                if mnl.output_node_id != usize::MAX {
                    ports.push(StageGraphPort {
                        label: "output".into(),
                        node_id: mnl.output_node_id,
                        direction: StageGraphPortDirection::Output,
                    });
                }
                ("MultiNl".to_string(), "multi-nl".to_string())
            }
            Stage::BlackFeedback(bf) => {
                if bf.output_node_id != usize::MAX {
                    ports.push(StageGraphPort {
                        label: "output".into(),
                        node_id: bf.output_node_id,
                        direction: StageGraphPortDirection::Output,
                    });
                }
                ("BlackFeedback".to_string(), "black-feedback".to_string())
            }
            Stage::Blockwise(bkm) => {
                let mut labels: Vec<String> = (0..bkm.n_ports)
                    .map(|idx| format!("coupling_{idx}"))
                    .collect();
                for (block_idx, owned_ports) in bkm.block_ports.iter().enumerate() {
                    for (local_idx, binding) in owned_ports.iter().enumerate() {
                        let port_idx = binding.port_idx;
                        if let Some(label) = labels.get_mut(port_idx) {
                            *label = format!("block{block_idx}_{:?}_{local_idx}", binding.role);
                        }
                    }
                }
                for (name, port_idx) in &bkm.vs_port_map {
                    if let Some(label) = labels.get_mut(*port_idx) {
                        *label = format!("vs:{name}");
                    }
                }
                for passive in &bkm.coupling_passives {
                    if let Some(label) = labels.get_mut(passive.port_idx) {
                        *label = format!("passive:{}", passive.comp_id);
                    }
                }
                for port_idx in 0..bkm.coupling_ports.len() {
                    let Some(terminals) = bkm.coupling_port_graph_terminals(port_idx) else {
                        continue;
                    };
                    let (node_pos, node_neg) = terminals.as_tuple();
                    if let Some(node_id) = node_pos {
                        ports.push(StageGraphPort {
                            label: labels
                                .get(port_idx)
                                .cloned()
                                .unwrap_or_else(|| format!("coupling_{port_idx}")),
                            node_id,
                            direction: StageGraphPortDirection::Bidirectional,
                        });
                    }
                    if let Some(node_id) = node_neg {
                        ports.push(StageGraphPort {
                            label: format!(
                                "{}:neg",
                                labels
                                    .get(port_idx)
                                    .cloned()
                                    .unwrap_or_else(|| format!("coupling_{port_idx}"))
                            ),
                            node_id,
                            direction: StageGraphPortDirection::Bidirectional,
                        });
                    }
                }
                ("Blockwise".to_string(), "blockwise".to_string())
            }
            Stage::Iir(_) => ("Iir".to_string(), "iir".to_string()),
            Stage::StateSpace(_) => ("StateSpace".to_string(), "state-space".to_string()),
            Stage::SerialDelayedFeedback(_) => (
                "SerialDelayedFeedback".to_string(),
                "serial-feedback".to_string(),
            ),
        };

        graph.stages.push(StageGraphNode {
            stage_idx,
            kind,
            label,
            component_ids,
            ports,
        });
    }

    for port in external_ports {
        graph.stages.push(StageGraphNode {
            stage_idx: usize::MAX,
            kind: "ExternalPort".to_string(),
            label: port.name.clone(),
            component_ids: Vec::new(),
            ports: vec![StageGraphPort {
                label: port.name.clone(),
                node_id: port.node_id,
                direction: match port.direction {
                    pedalkernel_rt::PortDirection::Input => StageGraphPortDirection::Output,
                    pedalkernel_rt::PortDirection::Output => StageGraphPortDirection::Input,
                },
            }],
        });
    }

    let mut endpoints_by_node: std::collections::BTreeMap<usize, Vec<(usize, usize)>> =
        std::collections::BTreeMap::new();
    for (graph_stage_idx, stage) in graph.stages.iter().enumerate() {
        for (port_idx, port) in stage.ports.iter().enumerate() {
            if port.node_id != usize::MAX {
                endpoints_by_node
                    .entry(port.node_id)
                    .or_default()
                    .push((graph_stage_idx, port_idx));
            }
        }
    }

    for (node_id, endpoints) in endpoints_by_node {
        for i in 0..endpoints.len() {
            for j in (i + 1)..endpoints.len() {
                let (from_stage, from_port) = endpoints[i];
                let (to_stage, to_port) = endpoints[j];
                graph.connections.push(StageGraphConnection {
                    node_id,
                    from_stage,
                    from_port,
                    to_stage,
                    to_port,
                });
            }
        }
    }

    graph
}

/// Check if a group is a merged pot pair (aw + wb of same component).
fn is_pot_divider_group(group: &super::signal_flow::FlowGroup, graph: &CircuitGraph) -> bool {
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
    let is_same_pot =
        graph.edges[edges[0]].comp_idx == graph.edges[edges[1]].comp_idx && comp0.kind.is_pot();

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
    Ok(BuiltStage::Wdf(WdfStage::new(
        tree,
        RootKind::ShortCircuit,
        oversampler,
    )))
}

fn root_supports_k_table(root: &RootKind) -> bool {
    matches!(
        root,
        RootKind::DiodePair(_)
            | RootKind::SingleDiode(_)
            | RootKind::ExplicitDiodePair(_)
            | RootKind::ExplicitSingleDiode(_)
            | RootKind::Zener(_)
            | RootKind::Jfet(_)
            | RootKind::JfetVr(_)
            | RootKind::Triode(_)
            | RootKind::VariMu(_)
            | RootKind::Pentode(_)
            | RootKind::Mosfet(_)
            | RootKind::Bjt(_)
            | RootKind::DiffPair(_)
            | RootKind::Ota(_)
            | RootKind::OpAmp(_)
    )
}

/// Wrap a passive DynNode tree with a voltage source input port.
///
/// The WDF tree needs a voltage source leaf to receive the input signal.
/// Creates `Series(VoltageSource, passive_tree)` — the standard WDF
/// topology where VS drives the tree and the root terminates it.
/// Default VS source impedance (Ω). Used when no port impedance is declared.
/// 10kΩ is a reasonable general-purpose value — high enough for RC filters
/// to work but not so high that it dominates the circuit.
const DEFAULT_VS_RP: f64 = 10_000.0;

pub(super) fn with_voltage_source(passive_tree: DynNode) -> DynNode {
    with_voltage_source_rp(passive_tree, DEFAULT_VS_RP)
}

pub(super) fn with_voltage_source_rp(passive_tree: DynNode, rp: f64) -> DynNode {
    let vs = DynNode::Leaf(LeafKind::VoltageSource(WdfVoltageSource {
        voltage: 0.0,
        rp,
        is_cathode_bias: false,
        port_name: None,
    }));
    DynNode::Series(Box::new(vs), Box::new(passive_tree))
}

fn build_grounded_series_reactive_load(
    edge_indices: &[usize],
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Option<WdfStage> {
    if edge_indices.len() != 2 {
        return None;
    }

    let is_gnd = |n: NodeId| n == graph.gnd_node || graph.ac_ground_nodes.contains(&n);

    let mut reactive_edge = None;
    let mut load_edge = None;
    for &eidx in edge_indices {
        match graph.effective_edge_kind(eidx) {
            super::component::EdgeKind::Reactive => reactive_edge = Some(eidx),
            super::component::EdgeKind::Linear => {
                let e = &graph.edges[eidx];
                if is_gnd(e.node_a) || is_gnd(e.node_b) {
                    load_edge = Some(eidx);
                }
            }
            _ => return None,
        }
    }

    let reactive_edge = reactive_edge?;
    let load_edge = load_edge?;
    let load = &graph.edges[load_edge];
    let load_node = if is_gnd(load.node_a) {
        load.node_b
    } else {
        load.node_a
    };
    let reactive = &graph.edges[reactive_edge];
    if reactive.node_a != load_node && reactive.node_b != load_node {
        return None;
    }

    let reactive_comp = &graph.components[graph.edges[reactive_edge].comp_idx];
    let load_comp = &graph.components[graph.edges[load_edge].comp_idx];
    let reactive_leaf = reactive_comp
        .kind
        .make_leaf(&reactive_comp.id, sample_rate)?;
    let load_leaf = load_comp.kind.make_leaf(&load_comp.id, sample_rate)?;

    let passive_tree = DynNode::Series(Box::new(reactive_leaf), Box::new(load_leaf));
    let mut wdf = WdfStage::new(
        with_voltage_source(passive_tree),
        RootKind::ShortCircuit,
        Oversampler::new(OversamplingFactor::X1),
    );
    wdf.output_probe = Some(load_comp.id.clone());
    Some(wdf)
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
    build_spqr_stage_with_options(stage, graph, _sample_rate, false, &[], 9.0)
}

pub(super) fn build_spqr_stage_with_options(
    stage: SpqrStage,
    graph: &CircuitGraph,
    _sample_rate: f64,
    disable_iir: bool,
    init_hints: &[crate::dsl::InitHint],
    supply_voltage: f64,
) -> Result<BuiltStage, String> {
    match stage {
        SpqrStage::PassiveWdf {
            tree, edge_indices, ..
        } => {
            if passive_pot_reactive_needs_rtype(&edge_indices, graph) {
                if let Some(wdf) = build_passive_rtype_stage(&edge_indices, graph, _sample_rate) {
                    return Ok(BuiltStage::Wdf(wdf));
                }
            }

            if let Some(wdf) =
                build_grounded_series_reactive_load(&edge_indices, graph, _sample_rate)
            {
                return Ok(BuiltStage::Wdf(wdf));
            }

            let tree = with_voltage_source(tree);
            let oversampler = Oversampler::new(OversamplingFactor::X1);
            // Ground-terminated → ShortCircuit root. Floating → Passthrough.
            let touches_gnd = edge_indices.iter().any(|&eidx| {
                let e = &graph.edges[eidx];
                e.node_a == graph.gnd_node
                    || e.node_b == graph.gnd_node
                    || graph.ac_ground_nodes.contains(&e.node_a)
                    || graph.ac_ground_nodes.contains(&e.node_b)
            });
            let root = if touches_gnd {
                RootKind::ShortCircuit
            } else {
                RootKind::Passthrough
            };
            let mut wdf = WdfStage::new(tree, root, oversampler);

            // Set output_probe: find the GND-side leaf at the output boundary.
            // For ShortCircuit stages, short_circuit_junction_voltage fails when
            // VS rp << passive rp (gamma ≈ 0 → near-zero junction voltage).
            // Instead, probe the leaf directly — leaf_voltage returns (a+b)/2
            // which is correct regardless of gamma.
            //
            // The output boundary is the group's non-input terminal. Find the
            // edge at that node whose other end reaches GND.
            if touches_gnd {
                // Find the group's output boundary node: any terminal that isn't
                // in_node and isn't GND.
                let group_terminals = compute_group_terminals(
                    &edge_indices,
                    graph,
                    &vec![graph.in_node, graph.out_node],
                );
                let is_gnd = |n: super::graph::NodeId| -> bool {
                    n == graph.gnd_node || graph.ac_ground_nodes.contains(&n)
                };
                let terminal_has_ground_load = |terminal: super::graph::NodeId| -> bool {
                    edge_indices.iter().any(|&eidx| {
                        let e = &graph.edges[eidx];
                        (e.node_a == terminal && is_gnd(e.node_b))
                            || (e.node_b == terminal && is_gnd(e.node_a))
                    })
                };
                // Prefer the local output/load boundary. This matters for
                // coupling stages like C_out -> R_out -> gnd where the group
                // input is not the global graph.in_node; picking the wrong
                // terminal probes the coupling cap instead of the load.
                let output_boundary = group_terminals
                    .iter()
                    .find(|&&t| terminal_has_ground_load(t))
                    .or_else(|| group_terminals.iter().find(|&&t| t == graph.out_node))
                    .or_else(|| group_terminals.iter().find(|&&t| t != graph.in_node))
                    .or_else(|| group_terminals.first())
                    .copied()
                    .unwrap_or(graph.out_node);
                // Find the edge at the output boundary that goes toward GND
                for &eidx in &edge_indices {
                    let e = &graph.edges[eidx];
                    let (touches_out, other) = if e.node_a == output_boundary {
                        (true, e.node_b)
                    } else if e.node_b == output_boundary {
                        (true, e.node_a)
                    } else {
                        (false, output_boundary)
                    };
                    if !touches_out {
                        continue;
                    }
                    let other_reaches_gnd = is_gnd(other)
                        || edge_indices.iter().any(|&eidx2| {
                            let e2 = &graph.edges[eidx2];
                            (e2.node_a == other || e2.node_b == other)
                                && (is_gnd(e2.node_a) || is_gnd(e2.node_b))
                        });
                    if other_reaches_gnd {
                        let comp = &graph.components[e.comp_idx];
                        wdf.output_probe = Some(comp.id.clone());
                        break;
                    }
                }
            }

            Ok(BuiltStage::Wdf(wdf))
        }
        SpqrStage::NlWdf {
            tree,
            nl_edge_idx,
            edge_indices,
            ..
        } => {
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
                .ok_or_else(|| format!("NL edge {} ({}) didn't classify", nl_edge_idx, comp.id))?;

            // BJTs now use BjtRoot (single-port WDF root with external Vbe),
            // same as triodes use TriodeRoot. No MultiNL fallback needed.
            let (mut root, base_diode_model) = create_root(&nl_kind, false);
            eprintln!(
                "[NlWdf] comp.id={:?} hints={:?}",
                comp.id,
                init_hints
                    .iter()
                    .map(|h| &h.device_label)
                    .collect::<Vec<_>>()
            );
            // Apply init hint to BjtRoot: set asymmetric initial Vce warm-start.
            // This is the mechanism for free-running BJT oscillators (e.g. astable
            // multivibrators). Without asymmetric initial conditions, the NR solver
            // can trap at the symmetric DC fixed point and produce no oscillation.
            if let RootKind::Bjt(ref mut bjt) = root {
                if let Some(hint) = init_hints.iter().find(|h| h.device_label == comp.id) {
                    let crate::dsl::InitState::Named(ref state_name) = hint.state;
                    let vce = bjt_hint_vce(state_name, supply_voltage, bjt.is_pnp);
                    bjt.set_initial_prev_v(vce);
                }
            }
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
        } => {
            // Use build_rigid_from_group_with_hints so init_hints flow through
            // when this Rigid stage is reached via the SPQR or blockwise path.
            // boundary_nodes and pendant_trees are not used by build_rigid_from_group*
            // (they were only meaningful for the legacy MNA pendant tree path).
            let _ = (boundary_nodes, pendant_trees);
            super::rigid::build_rigid_from_group_with_hints(
                edge_indices,
                graph,
                _sample_rate,
                None,
                supply_voltage,
                None,
                !disable_iir,
                init_hints,
            )
        }
    }
}

fn passive_pot_reactive_needs_rtype(edge_indices: &[usize], graph: &CircuitGraph) -> bool {
    let mut has_pot = false;
    let mut has_reactive = false;
    let mut has_split_pot = false;
    let mut split_pot_nodes = std::collections::HashSet::new();
    let mut seen = std::collections::HashSet::new();

    for &eidx in edge_indices {
        let comp_idx = graph.edges[eidx].comp_idx;
        let comp = &graph.components[comp_idx];
        if comp.kind.capacitance().is_some() || comp.kind.inductance().is_some() {
            has_reactive = true;
        }
        if !seen.insert(comp_idx) {
            continue;
        }
        if !comp.kind.is_pot() {
            continue;
        }
        has_pot = true;

        let pot_edges = edge_indices
            .iter()
            .filter(|&&other| graph.edges[other].comp_idx == comp_idx)
            .count();
        if pot_edges > 1 {
            has_split_pot = true;
            for pin in ["a", "w", "wiper", "b"] {
                if let Some(&node) = graph.node_names.get(&format!("{}.{}", comp.id, pin)) {
                    split_pot_nodes.insert(node);
                }
            }
        }
    }

    if has_pot && has_reactive {
        return true;
    }

    if !has_split_pot {
        return false;
    }

    edge_indices.iter().any(|&eidx| {
        let edge = &graph.edges[eidx];
        let comp = &graph.components[edge.comp_idx];
        (comp.kind.capacitance().is_some() || comp.kind.inductance().is_some())
            && (split_pot_nodes.contains(&edge.node_a) || split_pot_nodes.contains(&edge.node_b))
    })
}

fn build_passive_rtype_stage(
    edge_indices: &[usize],
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Option<WdfStage> {
    let is_ground = |n: NodeId| n == graph.gnd_node || graph.ac_ground_nodes.contains(&n);

    let terminals =
        compute_group_terminals(edge_indices, graph, &vec![graph.in_node, graph.out_node]);
    let output_node = terminals
        .iter()
        .copied()
        .find(|&t| t == graph.out_node)
        .or_else(|| {
            terminals
                .iter()
                .copied()
                .find(|&t| !is_ground(t) && t != graph.in_node)
        })?;
    let input_node = terminals
        .iter()
        .copied()
        .find(|&t| t == graph.in_node)
        .or_else(|| {
            terminals
                .iter()
                .copied()
                .find(|&t| !is_ground(t) && t != output_node)
        })?;

    let mut nodes: Vec<NodeId> = edge_indices
        .iter()
        .flat_map(|&eidx| {
            let e = &graph.edges[eidx];
            [e.node_a, e.node_b]
        })
        .filter(|&n| !is_ground(n))
        .collect();
    nodes.sort_unstable();
    nodes.dedup();

    let node_to_mna =
        |node: NodeId, nodes: &[NodeId]| -> Option<usize> { nodes.iter().position(|&n| n == node) };

    let mut mna = MnaSystem::new(nodes.len(), 1);
    let vs_node = node_to_mna(input_node, &nodes);
    mna.stamp_voltage_source(vs_node, None, 0);

    let mut children = Vec::new();
    let mut ports = Vec::new();
    let mut pot_stamps = Vec::new();
    let mut seen_pots = std::collections::HashSet::new();

    for &eidx in edge_indices {
        let edge = &graph.edges[eidx];
        let comp = &graph.components[edge.comp_idx];
        let n1 = node_to_mna(edge.node_a, &nodes);
        let n2 = node_to_mna(edge.node_b, &nodes);

        if comp.kind.is_pot() {
            if let Some(pot) = comp
                .kind
                .as_any()
                .downcast_ref::<super::components::Potentiometer>()
            {
                let mut child = DynNode::Pot(comp.id.clone(), pot.max_r, 0.5, pot.taper);
                if pot_edge_is_aw_half_for_build(graph, &comp.id, edge.node_a, edge.node_b) {
                    if let DynNode::Leaf(ref mut leaf) = child {
                        leaf.set_complement();
                    }
                }
                let r = child.port_resistance();
                mna.stamp_resistor(n1, n2, r);
                pot_stamps.push(RuntimePotStamp {
                    child_idx: children.len(),
                    terminals: WdfPortTerminals::maybe_differential(n1, n2),
                    conductance: 1.0 / r,
                });
                children.push(child);
                seen_pots.insert(edge.comp_idx);
            }
            continue;
        }

        if let Some(r) = comp.kind.resistance() {
            mna.stamp_resistor(n1, n2, r);
        } else if comp.kind.capacitance().is_some() || comp.kind.inductance().is_some() {
            let child = comp.kind.make_leaf(&comp.id, sample_rate)?;
            let rp = child.port_resistance();
            ports.push(WdfPort {
                node_pos: n1,
                node_neg: n2,
                resistance: rp,
            });
            children.insert(ports.len() - 1, child);
            for stamp in &mut pot_stamps {
                if stamp.child_idx >= ports.len() - 1 {
                    stamp.child_idx += 1;
                }
            }
        }
    }

    if ports.is_empty() || seen_pots.is_empty() {
        return None;
    }

    let output_mna = node_to_mna(output_node, &nodes);
    let (scattering, vs_injection) = mna.derive_scattering_and_vs_injection(&ports, 0);
    let (extraction_coeffs, extraction_vs) =
        mna.derive_extraction_coeffs(&ports, 0, output_mna, None);

    if scattering.iter().any(|v| !v.is_finite())
        || vs_injection.iter().any(|v| !v.is_finite())
        || extraction_coeffs.iter().any(|v| !v.is_finite())
        || !extraction_vs.is_finite()
    {
        return None;
    }

    let mut wdf = WdfStage::new(
        DynNode::Resistor(Some("__passive_rtype_dummy".to_string()), 1.0),
        RootKind::PassiveRType {
            scattering,
            vs_injection,
            n_ports: ports.len(),
            children,
            output_port: 0,
            extraction_coeffs,
            extraction_vs,
            extraction_output_pos: output_mna,
            extraction_output_neg: None,
            recompute_mna: Some(mna),
            recompute_ports: Some(ports),
            pot_stamps,
            needs_recompute: false,
            interp_table: None,
        },
        Oversampler::new(OversamplingFactor::X1),
    );
    wdf.output_probe = None;
    Some(wdf)
}

fn pot_edge_is_aw_half_for_build(
    graph: &CircuitGraph,
    comp_id: &str,
    a: NodeId,
    b: NodeId,
) -> bool {
    let Some(&w_node) = graph
        .node_names
        .get(&format!("{comp_id}.w"))
        .or_else(|| graph.node_names.get(&format!("{comp_id}.wiper")))
    else {
        return false;
    };
    let Some(&a_node) = graph.node_names.get(&format!("{comp_id}.a")) else {
        return false;
    };
    (a == w_node && b == a_node) || (a == a_node && b == w_node)
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
    let nl_edges: Vec<usize> = group
        .all_edges()
        .iter()
        .filter(|&&eidx| graph.effective_edge_kind(eidx) == EdgeKind::Nonlinear)
        .copied()
        .collect();
    if nl_edges.is_empty() {
        return false;
    }
    let mut pot_edge_counts = std::collections::HashMap::new();
    for eidx in group.all_edges() {
        let comp = &graph.components[graph.edges[eidx].comp_idx];
        if comp.kind.is_pot() {
            *pot_edge_counts
                .entry(graph.edges[eidx].comp_idx)
                .or_insert(0usize) += 1;
        }
    }
    if pot_edge_counts.values().any(|&count| count > 1) {
        return false;
    }

    nl_edges.iter().all(|&eidx| {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        let is_diode = comp.kind.is_diode_family();
        let to_gnd = e.node_a == graph.gnd_node
            || e.node_b == graph.gnd_node
            || graph.ac_ground_nodes.contains(&e.node_a)
            || graph.ac_ground_nodes.contains(&e.node_b);
        is_diode && to_gnd
    })
}

fn merge_cross_reactive_groups_into_active_groups(
    groups: &mut Vec<super::signal_flow::FlowGroup>,
    graph: &super::graph::CircuitGraph,
) {
    let mut active_terminal_nodes: Vec<std::collections::HashSet<super::graph::NodeId>> =
        Vec::with_capacity(groups.len());
    for group in groups.iter() {
        let mut nodes = std::collections::HashSet::new();
        for &eidx in &group.active_edges {
            let edge = &graph.edges[eidx];
            nodes.insert(edge.node_a);
            nodes.insert(edge.node_b);
        }
        active_terminal_nodes.push(nodes);
    }

    let mut merge_into: Vec<Option<usize>> = vec![None; groups.len()];
    for (source_idx, group) in groups.iter().enumerate() {
        if !group.active_edges.is_empty() {
            continue;
        }
        let reactive_edges: Vec<usize> = group
            .all_edges()
            .into_iter()
            .filter(|&eidx| graph.effective_edge_kind(eidx) == super::component::EdgeKind::Reactive)
            .collect();
        if reactive_edges.is_empty() {
            continue;
        }

        for (target_idx, nodes) in active_terminal_nodes.iter().enumerate() {
            if target_idx == source_idx || groups[target_idx].active_edges.is_empty() {
                continue;
            }
            let bridges_active_terminals = reactive_edges.iter().any(|&eidx| {
                let edge = &graph.edges[eidx];
                nodes.contains(&edge.node_a) && nodes.contains(&edge.node_b)
            });
            if bridges_active_terminals {
                merge_into[source_idx] = Some(target_idx);
                break;
            }
        }
    }

    for source_idx in 0..groups.len() {
        let Some(target_idx) = merge_into[source_idx] else {
            continue;
        };
        let edges = groups[source_idx].all_edges();
        if edges.is_empty() {
            continue;
        }
        #[cfg(test)]
        {
            let names: Vec<&str> = edges
                .iter()
                .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
                .collect();
            eprintln!(
                "  [compile] merged cross-reactive passive group {source_idx} into active group {target_idx}: {names:?}"
            );
        }
        for eidx in edges {
            if !groups[target_idx].feedback_edges.contains(&eidx) {
                groups[target_idx].feedback_edges.push(eidx);
            }
        }
    }

    let mut idx = 0;
    groups.retain(|_| {
        let keep = merge_into.get(idx).and_then(|target| *target).is_none();
        idx += 1;
        keep
    });
}

fn is_nonlinear_modulator_group(
    group: &super::signal_flow::FlowGroup,
    graph: &super::graph::CircuitGraph,
) -> bool {
    let mut has_transistor_nl = false;
    let mut touches_output = false;
    let mut has_reactive = false;
    let mut has_vcvs = false;
    for &eidx in group.all_edges().iter() {
        let e = &graph.edges[eidx];
        touches_output |= e.node_a == graph.out_node || e.node_b == graph.out_node;
        match graph.effective_edge_kind(eidx) {
            super::component::EdgeKind::Reactive => {
                has_reactive = true;
                continue;
            }
            super::component::EdgeKind::Vcvs => {
                has_vcvs = true;
                continue;
            }
            super::component::EdgeKind::Nonlinear => {}
            _ => continue,
        }
        if graph.effective_edge_kind(eidx) != super::component::EdgeKind::Nonlinear {
            continue;
        }
        let comp = &graph.components[e.comp_idx];
        if comp.kind.is_bjt() || comp.kind.is_jfet() || comp.kind.is_mosfet() {
            has_transistor_nl = true;
        } else {
            return false;
        }
    }

    has_transistor_nl && !touches_output && !has_reactive && !has_vcvs
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
            &comp.id,
            e.node_a,
            e.node_b,
            graph.gnd_node,
            &graph.node_names,
        ) {
            nl_kinds.push(kind);
        }
    }
    if nl_kinds.is_empty() {
        return None;
    }

    // Synthesize DiodePair from two SingleDiode edges (antiparallel to ground)
    let (root, base_diode_model) = if nl_kinds.len() >= 2 {
        let mut pair_dt = None;
        for i in 0..nl_kinds.len() {
            for j in (i + 1)..nl_kinds.len() {
                if let (
                    super::classify::NonlinearKind::SingleDiode(dt_a),
                    super::classify::NonlinearKind::SingleDiode(_),
                ) = (&nl_kinds[i], &nl_kinds[j])
                {
                    pair_dt = Some(*dt_a);
                    break;
                }
            }
            if pair_dt.is_some() {
                break;
            }
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
    let tree = DynNode::Leaf(super::wdf_leaf::LeafKind::VoltageSource(
        super::wdf_leaf::WdfVoltageSource {
            voltage: 0.0,
            rp: 100.0,
            is_cathode_bias: false,
            port_name: None,
        },
    ));

    let oversampler =
        crate::oversampling::Oversampler::new(crate::oversampling::OversamplingFactor::X2);
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
    bias_node_voltages: &std::collections::BTreeMap<super::graph::NodeId, f64>,
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
        let mut pin_voltages: hashbrown::HashMap<String, f64> = hashbrown::HashMap::new();

        if let Some(pins) = graph.nullor_pins.iter().find(|p| p.comp_idx == e.comp_idx) {
            if let Some(&v) = bias_node_voltages.get(&pins.pos_node) {
                pin_voltages.insert("pos".to_string(), v);
            }
            if let Some(&v) = bias_node_voltages.get(&pins.neg_node) {
                pin_voltages.insert("neg".to_string(), v);
            }
        }

        let result = comp.kind.apply_bias(&pin_voltages, supply_voltage);
        if let BiasResult::Applied {
            v_rail_pos,
            v_rail_neg,
        } = result
        {
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
    use super::component::EdgeKind;
    use super::graph::NodeId;
    use hashbrown::HashMap;
    use std::collections::{HashSet, VecDeque};

    fn bfs_distances(start: NodeId, graph: &super::graph::CircuitGraph) -> HashMap<NodeId, usize> {
        let mut node_dist: HashMap<NodeId, usize> = HashMap::new();
        let mut queue: VecDeque<NodeId> = VecDeque::new();
        node_dist.insert(start, 0);
        queue.push_back(start);

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

        node_dist
    }

    fn passive_path_exists(
        start: NodeId,
        target: NodeId,
        graph: &super::graph::CircuitGraph,
    ) -> bool {
        if start == target {
            return true;
        }

        let mut queue: VecDeque<NodeId> = VecDeque::new();
        let mut visited: HashSet<NodeId> = HashSet::new();
        visited.insert(start);
        queue.push_back(start);

        while let Some(node) = queue.pop_front() {
            for (eidx, e) in graph.edges.iter().enumerate() {
                let next = if e.node_a == node {
                    e.node_b
                } else if e.node_b == node {
                    e.node_a
                } else {
                    continue;
                };

                match graph.effective_edge_kind(eidx) {
                    EdgeKind::Linear | EdgeKind::Reactive => {}
                    EdgeKind::Vcvs
                    | EdgeKind::Vccs
                    | EdgeKind::Behavioral
                    | EdgeKind::Nonlinear => continue,
                }

                if next == target {
                    return true;
                }
                if next == graph.gnd_node
                    || graph.supply_nodes.contains(&next)
                    || graph.ac_ground_nodes.contains(&next)
                {
                    continue;
                }
                if visited.insert(next) {
                    queue.push_back(next);
                }
            }
        }

        false
    }

    let mut node_dist: HashMap<NodeId, usize> = HashMap::new();
    node_dist.extend(bfs_distances(graph.in_node, graph));
    let out_dist = bfs_distances(graph.out_node, graph);
    let span = graph.edges.len() + graph.components.len() + groups.len() + 1;

    let min_dists: Vec<usize> = groups
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
            min_dist
        })
        .collect();

    let mut distances: Vec<usize> = groups
        .iter()
        .enumerate()
        .map(|(gi, group)| {
            let min_dist = min_dists[gi];
            if min_dist == usize::MAX {
                return groups.len() * span;
            }
            let touches_output = group.all_edges().iter().any(|&eidx| {
                let e = &graph.edges[eidx];
                e.node_a == graph.out_node || e.node_b == graph.out_node
            });

            if touches_output && !group.has_feedback() {
                // A non-feedback group touching the circuit output is a sink
                // network: output pot, coupling cap, load, or final pad. Its
                // nearest node may be an upstream op-amp output, so min-hop
                // BFS can place it before the active stages that feed it.
                // Keep it after upstream feedback/active groups while still
                // preserving deterministic ordering among multiple sinks.
                groups.len() * span + min_dist
            } else if group.has_feedback() {
                let mut max_active_out_to_output = 0usize;
                for &eidx in group.active_edges.iter() {
                    let comp_idx = graph.edges[eidx].comp_idx;
                    if let Some(pins) = graph.nullor_pins.iter().find(|p| p.comp_idx == comp_idx) {
                        if let Some(&d) = out_dist.get(&pins.out_node) {
                            max_active_out_to_output = max_active_out_to_output.max(d);
                        }
                    }
                }
                min_dist * span + (span - max_active_out_to_output.min(span))
            } else {
                min_dist
            }
        })
        .collect();

    for (gi, group) in groups.iter().enumerate() {
        if !is_ground_clip_group(group, graph) {
            continue;
        }

        let clip_node = get_ground_clip_signal_node(group, graph);
        let mut driven_by_active = None;
        for (driver_gi, driver_group) in groups.iter().enumerate() {
            if !driver_group.has_feedback() {
                continue;
            }
            for &active_eidx in &driver_group.active_edges {
                let comp_idx = graph.edges[active_eidx].comp_idx;
                let Some(pins) = graph.nullor_pins.iter().find(|p| p.comp_idx == comp_idx) else {
                    continue;
                };
                if passive_path_exists(pins.out_node, clip_node, graph) {
                    driven_by_active = Some(
                        driven_by_active
                            .map_or(distances[driver_gi], |d: usize| d.min(distances[driver_gi])),
                    );
                }
            }
        }

        if let Some(driver_dist) = driven_by_active {
            distances[gi] = distances[gi].max(driver_dist.saturating_add(1));
        }
    }

    distances
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
        if e.node_a == graph.gnd_node
            || e.node_b == graph.gnd_node
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
        // Global terminals (in_node, out_node): always add if the group
        // touches them. They're the circuit's signal ports. For mid-chain
        // groups that don't touch in_node/out_node, this is a no-op.
        if global_terminals.contains(&node) {
            if !terminals.contains(&node) {
                terminals.push(node);
            }
            continue;
        }
        // Does this node connect to edges outside the group?
        let is_boundary = graph.edges.iter().enumerate().any(|(eidx, e)| {
            !group_edge_set.contains(&eidx) && (e.node_a == node || e.node_b == node)
        });
        if is_boundary && !terminals.contains(&node) {
            terminals.push(node);
        }
    }

    // Merge terminals that converge at the same downstream node.
    // If two boundary nodes both connect (via external edges) to the same
    // destination node, they're effectively one output port — keep only one.
    // Example: nodes 49 and 51 both connect to U3.neg → merge to one terminal.
    if terminals.len() > 2 {
        // For each terminal, find the set of downstream nodes it connects to
        let downstream: Vec<(NodeId, std::collections::HashSet<NodeId>)> = terminals
            .iter()
            .map(|&t| {
                let dests: std::collections::HashSet<NodeId> = graph
                    .edges
                    .iter()
                    .enumerate()
                    .filter(|(eidx, e)| {
                        !group_edge_set.contains(eidx) && (e.node_a == t || e.node_b == t)
                    })
                    .map(|(_, e)| if e.node_a == t { e.node_b } else { e.node_a })
                    .collect();
                (t, dests)
            })
            .collect();

        // Find terminals that share downstream destinations and merge them
        let mut merged = vec![false; terminals.len()];
        for i in 0..downstream.len() {
            if merged[i] {
                continue;
            }
            for j in (i + 1)..downstream.len() {
                if merged[j] {
                    continue;
                }
                // If they share ANY downstream node, merge (keep i, drop j)
                if !downstream[i].1.is_disjoint(&downstream[j].1) {
                    merged[j] = true;
                }
            }
        }

        terminals = terminals
            .into_iter()
            .enumerate()
            .filter(|(i, _)| !merged[*i])
            .map(|(_, t)| t)
            .collect();
    }

    // Fallback: if no terminals found, use global
    if terminals.is_empty() {
        return global_terminals.to_vec();
    }

    terminals
}

/// Resolve a BJT init state name to the initial Vce warm-start for BjtRoot.
///
/// Mirrors the table in `rigid/general.rs:resolve_bjt_init_state` but returns
/// only Vce (the scalar that BjtRoot uses as `prev_v`). Sign is applied for PNP.
fn bjt_hint_vce(state_name: &str, supply_voltage: f64, is_pnp: bool) -> f64 {
    let vce = match state_name {
        "saturated" => 0.1,
        "cutoff" => supply_voltage,
        "active" | "forward" => supply_voltage * 0.5,
        "reverse" => supply_voltage,
        _ => supply_voltage * 0.5,
    };
    if is_pnp {
        -vce
    } else {
        vce
    }
}
