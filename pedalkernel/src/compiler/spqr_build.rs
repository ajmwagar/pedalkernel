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
    build_general_mna_from_edges, build_general_mna_from_edges_with_hints,
    build_general_mna_from_edges_with_supply, build_rigid, build_rigid_from_group,
    build_rigid_from_group_with_hints, build_rigid_without_iir,
};
use super::spqr::{spqr_decompose, spqr_to_stages, SpqrStage};
use super::stage::{IirStage, MultiNlStage, RootKind, StateSpaceStage, WdfStage};
use super::wdf_leaf::{LeafKind, WdfLeaf, WdfVoltageSource};
use crate::dsl::{PedalDef, TransformerConfig};
use crate::oversampling::{Oversampler, OversamplingFactor};
use pedalkernel_rt::boundary_math::{MnaNodeId, MnaPortTerminals, MnaVariableResistorBinding};
use pedalkernel_rt::elements::JaCoreModel;
use pedalkernel_rt::thermal::ThermalModel;
use pedalkernel_rt::tree::{MnaSystem, WdfPort};

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
    // Resolve envelope-modulated component edge kinds (audit gap G2):
    // `EF.out -> J.vgs` reclassifies the JFET's drain–source edge to Linear
    // so it compiles as a `jfet_vr` variable-resistor leaf the envelope
    // binding can reach. Restricted to envelope followers so LFO-modulated
    // circuits keep their existing compilation behavior.
    super::graph::resolve_components_by(&mut graph, pedal, |c| {
        c.kind
            .as_any()
            .downcast_ref::<super::components::EnvelopeFollower>()
            .is_some()
    });
    // Connectivity-based completeness check: every active device that declares
    // `terminal_requirements()` must have its Required neighbour roles present
    // (e.g. a triode must have a Load reachable from an output terminal). This
    // is additive and behaviour-neutral — it only rejects circuits that are
    // genuinely missing a required neighbour; it never changes how complete
    // circuits compile. Gated false-positive-free on the working corpus.
    super::neighbor_roles::validate_completeness(&graph)?;
    let supply_voltage = pedal.supplies.first().map_or(9.0, |s| s.config.voltage);
    let delay_lines = build_delay_line_bindings(pedal, sample_rate);
    // Behavioral islands (bbd(), vca(), ...) lower to per-instance runtime DSP
    // blocks via the DspBlock registry, not the WDF/MNA core. The mandatory-
    // lowering gate (architecture debt §4) rejects any registered-island
    // type_tag with no DspBlock to lower it (currently vco()). Runtime
    // instances are built + bound by `dsp_block::bind_runtime_all` after the
    // CompiledPedal is constructed; `has_blocks` is the presence predicate
    // that gates group splitting and terminal injection below.
    super::dsp_block::reject_unlowered_behavioral(pedal)?;
    let has_blocks = super::dsp_block::any_block_has_components(pedal);

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

    if all_edges.is_empty() && delay_lines.is_empty() && !has_blocks {
        return Err("No circuit edges found".to_string());
    }

    if all_edges.is_empty() {
        let mut compiled = CompiledPedal {
            stages: Vec::new(),
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
            bbds: Vec::new(),
            delay_lines,
            springs: Vec::new(),
            vcos: Vec::new(),
            vcas: Vec::new(),
            thermal: if options.thermal {
                Some(ThermalModel::silicon_standard(sample_rate))
            } else {
                None
            },
            tolerance_seed: 0,
            opamp_stages: Vec::new(),
            power_supply: None,
            metrics_accumulator: None,
            metrics_buffer: None,
            #[cfg(feature = "diag")]
            diag_ring: None,
            input_loading: None,
            output_loading: None,
            output_dc_block: None,
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
            internal_ports: Vec::new(),
            boundary_loads: Vec::new(),
            detector_led_coupling: None,
            initialized: false,
        };
        compiled.set_supply_voltage(supply_voltage);
        super::spqr_control::bind_controls(pedal, &mut compiled);
        super::dsp_block::bind_runtime_all(pedal, &mut compiled, sample_rate)?;
        return Ok(compiled);
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
            bbds: Vec::new(),
            delay_lines,
            springs: Vec::new(),
            vcos: Vec::new(),
            vcas: Vec::new(),
            thermal: None,
            tolerance_seed: 0,
            opamp_stages: Vec::new(),
            power_supply: None,
            metrics_accumulator: None,
            metrics_buffer: None,
            #[cfg(feature = "diag")]
            diag_ring: None,
            input_loading: None,
            output_loading: None,
            output_dc_block: None,
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
            internal_ports: Vec::new(),
            boundary_loads: Vec::new(),
            detector_led_coupling: None,
            initialized: false,
        };
        compiled.set_supply_voltage(supply_voltage);
        super::spqr_control::bind_controls(pedal, &mut compiled);
        super::dsp_block::bind_runtime_all(pedal, &mut compiled, sample_rate)?;
        return Ok(compiled);
    }

    // Step 1: Partition edges into signal flow groups.
    // Each group = mutually-dependent components that must share a stage.
    // Uses directed dependency graph: cycles = co-solved, acyclic = sequential.
    eprintln!(
        "  [compile] Step 1: find_flow_groups ({} edges)...",
        all_edges.len()
    );
    // Phase 2a: the same broker cut set find_flow_groups consulted. Threaded
    // into the merge passes so a cut tap-mouth edge can never be re-absorbed
    // back across the delayed-coupling boundary.
    let cut_edges = super::boundary_rules::delayed_cut_edges(&graph);
    // Detector control-electrode nodes (DelayedSense sinks) — used below to mark
    // the de-fused detector group bypass_serial so it can't hijack the chain.
    let detector_seed_nodes: std::collections::HashSet<super::graph::NodeId> =
        super::boundary_rules::detector_control_nodes(&graph)
            .into_iter()
            .collect();
    let mut feedback_groups = super::signal_flow::find_flow_groups(&all_edges, &graph);
    merge_cross_reactive_groups_into_active_groups(&mut feedback_groups, &graph, &cut_edges);
    merge_input_coupling_into_active_groups(&mut feedback_groups, &graph, &cut_edges);
    // DSP-block pedals: bbd(), vca(), ... are GraphRole::Virtual, so the
    // netlist is galvanically cut at their in/out pins. Split any group that
    // spans a behavioral gap (the sides stay "connected" through ground only)
    // into separate serial stages — a stage built across the gap probes 0.
    if has_blocks {
        super::dsp_block::split_groups_at_behavioral_gaps(&mut feedback_groups, &graph, pedal);
    }
    eprintln!("  [compile] Step 1 done: {} groups", feedback_groups.len());

    // Step 1b: Compute signal flow distance for each group via BFS from in_node.
    // Each group's distance = min BFS hop count from in_node to any node in the group.
    // This gives the correct processing order (input coupling first, clipping second,
    // tone third, etc.) regardless of the order find_flow_groups returned them.
    let group_flow_distances = compute_group_flow_distances(&feedback_groups, &graph);

    // Broker (d-2): does this circuit have a mid-chain Tight transformer coupling
    // (plain two-port, secondary != out)? If so, the rail-crossing grid-hop walk
    // used to order triode stages under-counts everything behind the magnetic gap,
    // and the corrected broker-coupled-link-aware `group_flow_distances` must be
    // used for those stages instead. Computed once (cheap; consults the broker).
    let has_tight_coupled_transformer =
        super::boundary_rules::has_tight_coupled_transformer(&graph);

    // Step 1c: Classify each group as signal path or static bias.
    // Static bias groups (VCC dividers) are bypassed in the serial audio
    // chain — they still process and meter, but don't overwrite the signal.
    let group_bias: Vec<_> = feedback_groups
        .iter()
        .map(|g| super::bias::classify_group_bias(g, &graph))
        .collect();

    // Build a map from node → DC bias voltage across all StaticBias groups.
    // Used to set op-amp v_max from the circuit's actual bias network.
    let mut bias_node_voltages: std::collections::BTreeMap<super::graph::NodeId, f64> =
        std::collections::BTreeMap::new();
    for kind in &group_bias {
        if let super::bias::GroupBiasKind::StaticBias { dc_voltages } = kind {
            bias_node_voltages.extend(dc_voltages);
        }
    }

    // Step 2: SPQR decompose each group independently.
    // DSP-block signal pins are behavioral stage boundaries: treat them like
    // global terminals so the group on each side of a galvanic gap gets a port
    // there (otherwise the dangling side has no entry/exit node and its stage
    // probes 0). Boundary nodes are gathered from every registered DspBlock.
    let mut terminals = vec![graph.in_node, graph.out_node];
    for node in super::dsp_block::all_boundary_nodes(pedal, &graph) {
        if !terminals.contains(&node) {
            terminals.push(node);
        }
    }
    // Cross-network couplers (photocoupler LED/LDR) are galvanically isolated:
    // each side's port nodes must be SPQR terminals so the side that does not
    // carry the global in/out signal still gets a stage port (otherwise its
    // dangling group probes 0). Kept as a sibling of `all_boundary_nodes` so
    // the DspBlock registry and the coupler concern stay cleanly separated.
    for node in coupler_boundary_nodes(&graph) {
        if !terminals.contains(&node) {
            terminals.push(node);
        }
    }
    // Phase 2a: each side of a broker-cut delayed-coupling boundary becomes a
    // stage port — register the tap-mouth nodes as SPQR terminals (sibling of
    // `coupler_boundary_nodes`) so the de-fused detector group and the forward
    // group each get an entry/exit node at the cut.
    for &node in &cut_edges.boundary_nodes {
        if !terminals.contains(&node) {
            terminals.push(node);
        }
    }
    let mut stages: Vec<Stage> = Vec::new();
    let mut stage_comp_ids: Vec<Vec<String>> = Vec::new();
    let mut bkm_consumed_comp_ids: std::collections::HashSet<String> =
        std::collections::HashSet::new();
    // Component ids of transformer-SECONDARY load groups consumed by the
    // transformer-reflection fusion (their edges are solved inside the fused
    // primary-side PassiveRType MNA). Groups made entirely of these ids are
    // skipped — the magnetic-coupling analogue of `bkm_consumed_comp_ids`.
    let mut xfmr_consumed_comp_ids: std::collections::HashSet<String> =
        std::collections::HashSet::new();
    // Boundary-load decision table rows recorded during stage building;
    // resolved to final stage indices after the last stage sort.
    let mut pending_boundary_loads: Vec<super::boundary_load::PendingBoundaryLoad> = Vec::new();

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
                        && !wdf.is_source_follower
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
    // Track which passive edges were absorbed into a triode-context MNA stage.
    // Their containing groups will be skipped to avoid double-processing.
    let mut triode_absorbed_edges: std::collections::HashSet<usize> =
        std::collections::HashSet::new();
    // Edge guard (pedalkernel-ffkl): edges DELIBERATELY excluded from stage
    // assembly, with the exclusion reason logged at the exclusion site.
    let mut guard_excluded_edges: std::collections::HashSet<usize> =
        std::collections::HashSet::new();
    build_order.sort_by_key(|(_, g)| if g.has_feedback() { 1 } else { 0 });

    for &(gi, group) in &build_order {
        if group.all_edges().is_empty() {
            continue;
        }

        // Skip passive groups whose edges were ALL absorbed into a triode-context MNA stage.
        // Groups with only some absorbed edges: their group_edges will be filtered below.
        {
            let group_edges = group.all_edges();
            if !group_edges.is_empty()
                && group_edges
                    .iter()
                    .all(|eidx| triode_absorbed_edges.contains(eidx))
            {
                #[cfg(test)]
                eprintln!("  group {gi}: all edges absorbed into triode stage, skipping");
                continue;
            }
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
        if !group_comp_ids.is_empty()
            && group_comp_ids
                .iter()
                .all(|id| xfmr_consumed_comp_ids.contains(id))
        {
            #[cfg(test)]
            eprintln!("  → group {gi} already consumed by transformer-secondary reflection");
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
        let has_signal_transformer =
            group_has_signal_transformer_boundary(group, &graph, graph.in_node, graph.out_node);
        // Phase 2a: the de-fused detector group (holds the DelayedSense control
        // electrode, no non-cut path to in/out after the tap-mouth cut) computes
        // state/metering but must NOT hijack the forward serial signal — bypass
        // it. Its input is floating in 2a (driven by internal delayed ports in
        // 2b); de-fusing + feeding the forward path is all 2a proves.
        let is_detector_bypass = is_delayed_detector_group(group, &graph, &detector_seed_nodes);
        let is_bypass = ((matches!(
            group_bias[gi],
            super::bias::GroupBiasKind::StaticBias { .. }
        ) || is_nonlinear_modulator_group(group, &graph))
            && !has_signal_transformer)
            || is_detector_bypass;

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
            // Edge guard: deliberate exclusion, with a reason. (When the
            // modulator classifier MISfires — e.g. the fuzz-face family's
            // audio-path BJT core consumed here, leaving a unity passthrough —
            // that is the classifier's known bug, pedalkernel-129p family,
            // not an unaccounted edge.)
            guard_excluded_edges.extend(group.all_edges());
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

            // ── Output-load fusion (BA283-class, narrow + gated) ──────────
            // A multi-BJT DC-coupled-feedback group about to be built as ONE
            // general-MNA stage (the dc_qpoint path) never feels a load placed
            // in a downstream stage — downstream impedance does not reflect
            // back into this stage's scattering. Fuse the trailing output
            // group (coupling cap + grounded load at the global `out`) into
            // this build so the load-current demand is part of the solved
            // network. ANALYSIS (what hangs off the output boundary) and
            // POLICY (whether it may fuse — gated exactly like the dc_qpoint
            // seed + servo) live in `boundary_load`; everything else is
            // byte-identical. Analysis + disposition are recorded on
            // `CompiledPedal::boundary_loads` for dashboards, fused or not.
            let mut build_edges = group.all_edges();
            let boundary_analysis = super::boundary_load::analyze_trailing_output_load(
                gi,
                &feedback_groups,
                &graph,
                &cut_edges,
            );
            let load_disposition = match &boundary_analysis {
                Some(load) => {
                    match super::boundary_load::gate_fusion(load, group, &graph, supply_voltage) {
                        Ok(()) => super::boundary_load::LoadDisposition::FusedUpstream {
                            consumed_group: load.group,
                        },
                        Err(reason) => super::boundary_load::LoadDisposition::Unloaded { reason },
                    }
                }
                None => super::boundary_load::LoadDisposition::Unloaded {
                    reason: super::boundary_load::REASON_NO_TRAILING_LOAD,
                },
            };
            let fused_output_group = load_disposition.fused_group();
            if let Some(src_gi) = fused_output_group {
                // Consume the analyzed load model: its edges ARE the trailing
                // group's edges (same vec, same order).
                let extra = boundary_analysis
                    .as_ref()
                    .map(|load| load.model.edges().to_vec())
                    .unwrap_or_default();
                #[cfg(test)]
                {
                    let names: Vec<&str> = extra
                        .iter()
                        .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
                        .collect();
                    eprintln!(
                        "  [compile] group {gi}: fused trailing output load group {src_gi} \
                         {names:?} into multi-BJT DC-feedback MNA"
                    );
                }
                for eidx in extra {
                    if !build_edges.contains(&eidx) {
                        build_edges.push(eidx);
                    }
                }
            }
            // Refresh the stage label / comp ids so the fused stage owns the
            // load components (stage_comp_ids drives later stage lookups).
            let (group_label, group_comp_ids) = if fused_output_group.is_some() {
                let mut names: Vec<String> = build_edges
                    .iter()
                    .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.clone())
                    .collect();
                names.sort();
                names.dedup();
                #[cfg(debug_assertions)]
                let label = names.join(",");
                #[cfg(not(debug_assertions))]
                let label = String::new();
                (label, names)
            } else {
                (group_label, group_comp_ids)
            };

            let mut built = build_rigid_from_group_with_hints(
                build_edges,
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
                        let mut ri_pot_taper = crate::dsl::PotTaper::B;
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
                                        ri_pot_taper = comp
                                            .kind
                                            .pot_taper()
                                            .unwrap_or(crate::dsl::PotTaper::B);
                                        ri_fixed += ri_pot_taper.apply(0.5) * max_r;
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
                            let fixed_without_pot =
                                ri_fixed - ri_pot_taper.apply(0.5) * ri_pot_max_r;
                            bf.ri_pot_comp_id = Some(pot_id);
                            bf.ri_fixed_r = fixed_without_pot;
                            bf.ri_pot_max_r = ri_pot_max_r;
                            bf.ri_pot_taper = ri_pot_taper;
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

            // Output-load fusion: the trailing `{Cout, RL}` group was already
            // built as its own passive stage (non-feedback groups build
            // first). Its components are now solved INSIDE the fused MNA —
            // remove the standalone stage so the load isn't applied twice
            // (same consumption mechanism as blockwise coupling above).
            if let Some(src_gi) = fused_output_group {
                let mut src_ids: Vec<String> = feedback_groups[src_gi]
                    .all_edges()
                    .iter()
                    .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.clone())
                    .collect();
                src_ids.sort();
                src_ids.dedup();
                // Exclude the fused stage just pushed (last index).
                for idx in (0..stages.len().saturating_sub(1)).rev() {
                    if stage_comp_ids[idx] == src_ids {
                        #[cfg(test)]
                        eprintln!(
                            "  [compile] group {gi}: removed standalone trailing \
                             output stage {idx} ({src_ids:?}) — consumed by fusion"
                        );
                        stages.remove(idx);
                        stage_comp_ids.remove(idx);
                        break;
                    }
                }
            }

            // Record the boundary-load decision (fused or declined) for this
            // stage's output boundary. `stage_key` = the stage's minimum
            // stable component id — the same deterministic identity the final
            // stage sort uses — resolved to a stage index once ordering is
            // final. Purely descriptive: nothing downstream of compilation
            // reads it.
            pending_boundary_loads.push(super::boundary_load::PendingBoundaryLoad {
                stage_key: group_comp_ids
                    .iter()
                    .min()
                    .cloned()
                    .unwrap_or_else(|| "~".to_string()),
                boundary_node: boundary_analysis
                    .as_ref()
                    .map(|load| load.boundary_node)
                    .unwrap_or(graph.out_node),
                model: boundary_analysis
                    .as_ref()
                    .map(|load| load.model.summarize(&graph))
                    .unwrap_or(pedalkernel_rt::processor::BoundaryLoadSummary::Unloaded),
                disposition: load_disposition.summarize(),
            });
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
            // Remove any edges already absorbed into a triode-context MNA stage.
            let mut remaining_edges: Vec<usize> = group_edges
                .iter()
                .copied()
                .filter(|eidx| !triode_absorbed_edges.contains(eidx))
                .collect();

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

            // Linear (gate-modulated) JFET groups: keep only edges that are
            // graph-connected to the JFET's drain–source path through
            // non-ground nodes. Signal-flow grouping claims the gate-bias
            // network into the JFET's group (the gate is the amplifier
            // "input"), but a JFET resolved to a variable resistor conducts
            // audio only drain–source; gate-bias edges share no signal node
            // and would compile into a bogus series chain that silences the
            // stage (audit gap G2, fet_leveler).
            {
                let linear_jfet_edges: Vec<usize> = remaining_edges
                    .iter()
                    .copied()
                    .filter(|&eidx| {
                        graph.effective_edge_kind(eidx) == super::component::EdgeKind::Linear
                            && graph.components[graph.edges[eidx].comp_idx].kind.is_jfet()
                    })
                    .collect();
                if !linear_jfet_edges.is_empty() {
                    let is_ground = |n: super::graph::NodeId| -> bool {
                        n == graph.gnd_node || graph.ac_ground_nodes.contains(&n)
                    };
                    let mut keep_nodes: std::collections::HashSet<super::graph::NodeId> =
                        linear_jfet_edges
                            .iter()
                            .flat_map(|&eidx| {
                                let e = &graph.edges[eidx];
                                [e.node_a, e.node_b]
                            })
                            .filter(|&n| !is_ground(n))
                            .collect();
                    // Grow the connected component over non-ground nodes.
                    loop {
                        let mut grew = false;
                        for &eidx in &remaining_edges {
                            let e = &graph.edges[eidx];
                            let touches = (keep_nodes.contains(&e.node_a) && !is_ground(e.node_a))
                                || (keep_nodes.contains(&e.node_b) && !is_ground(e.node_b));
                            if touches {
                                for n in [e.node_a, e.node_b] {
                                    if !is_ground(n) && keep_nodes.insert(n) {
                                        grew = true;
                                    }
                                }
                            }
                        }
                        if !grew {
                            break;
                        }
                    }
                    remaining_edges.retain(|&eidx| {
                        let e = &graph.edges[eidx];
                        linear_jfet_edges.contains(&eidx)
                            || (!is_ground(e.node_a) && keep_nodes.contains(&e.node_a))
                            || (!is_ground(e.node_b) && keep_nodes.contains(&e.node_b))
                    });
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

            // ── Triode-with-grid detection: common-cathode triodes need the
            // full MNA context path for correct DC self-bias.
            //
            // Signal flow analysis may classify a standalone common-cathode triode
            // either as a group with only the triode NL edge (no passive claiming,
            // no feedback) OR as a group that already includes the triode's passive
            // context (plate load, cathode bias, cathode bypass cap, grid leak).
            //
            // In both cases SPQR would produce a bare WdfStage with a TriodeRoot.
            // While the WDF tree correctly models the passive network, the K-table
            // interpolation introduces ~4 dB RMS error versus SPICE for high-voltage
            // stages because the K-table sweep doesn't capture the dynamic cathode
            // self-bias feedback accurately at large signal levels.
            //
            // Fix: detect ANY non-feedback group containing exactly ONE triode NL
            // edge (with a grid node). Route it through the full MNA path
            // (build_general_mna_from_edges_with_supply + apply_triode_dc_qpoint)
            // which gives ~0.1 dB RMS accuracy versus SPICE.
            //
            // For 1-edge groups: collect context edges via BFS.
            // For n-edge groups: use group_edges directly (already has context).
            {
                let nl_triode_edges: Vec<usize> = group_edges
                    .iter()
                    .copied()
                    .filter(|&eidx| {
                        if graph.effective_edge_kind(eidx) != super::component::EdgeKind::Nonlinear
                        {
                            return false;
                        }
                        let e = &graph.edges[eidx];
                        let comp = &graph.components[e.comp_idx];
                        matches!(
                            comp.kind.classify_nonlinear(
                                &comp.id,
                                e.node_a,
                                e.node_b,
                                graph.gnd_node,
                                &graph.node_names,
                            ),
                            Some((
                                super::classify::NonlinearKind::Triode {
                                    grid_node: Some(_),
                                    ..
                                },
                                _
                            ))
                        )
                    })
                    .collect();

                if nl_triode_edges.len() == 1 {
                    let nl_edge_idx = nl_triode_edges[0];
                    let e = &graph.edges[nl_edge_idx];
                    let comp = &graph.components[e.comp_idx];
                    if let Some((
                        super::classify::NonlinearKind::Triode {
                            grid_node: Some(grid_node),
                            ..
                        },
                        _,
                    )) = comp.kind.classify_nonlinear(
                        &comp.id,
                        e.node_a,
                        e.node_b,
                        graph.gnd_node,
                        &graph.node_names,
                    ) {
                        // For 1-edge groups collect context; for multi-edge groups
                        // the context is already in group_edges — use them directly.
                        let mut context_edges = if group_edges.len() == 1 {
                            collect_triode_context_edges(nl_edge_idx, &graph, &all_edges)
                        } else {
                            group_edges.clone()
                        };
                        #[cfg(test)]
                        eprintln!(
                            "  group {gi}: triode-with-grid ({} group edges → {} context edges)",
                            group_edges.len(),
                            context_edges.len(),
                        );

                        // ── Transformer-secondary load reflection at the
                        // triode-context MNA (pedalkernel-ffkl / GAP F) ──
                        // If this build set owns a transformer primary edge
                        // (LA-2A: V3's cathode follower drives T_out), find
                        // the grounded resistive load on its secondary (the
                        // C4 analysis) and fuse those edges into the build so
                        // the general-MNA transformer stamp solves the
                        // reflection in-stage: the load current flows through
                        // the ideal turns-ratio branch and the follower feels
                        // n²·R. The standalone load group is consumed
                        // (absorbed-edge skip + built-stage removal below)
                        // and the decision is recorded on the boundary-load
                        // table either way.
                        let mut xfmr_fused: Option<(
                            super::boundary_load::TransformerSecondaryLoad,
                            Vec<String>,
                        )> = None;
                        if let Some(load) =
                            super::boundary_load::analyze_transformer_secondary_load_in_edges(
                                &context_edges,
                                gi,
                                &feedback_groups,
                                &graph,
                                &cut_edges,
                            )
                        {
                            let load_already_consumed =
                                load.model.edges().iter().any(|&eidx| {
                                    xfmr_consumed_comp_ids.contains(
                                        &graph.components[graph.edges[eidx].comp_idx].id,
                                    )
                                });
                            let mut fused_load_ids: Option<Vec<String>> = None;
                            let disposition = if load_already_consumed {
                                super::boundary_load::LoadDisposition::Unloaded {
                                    reason: "gate:load-group-already-consumed",
                                }
                            } else {
                                match super::boundary_load::
                                    gate_triode_context_transformer_reflection()
                                {
                                    Ok(()) => {
                                        for &eidx in load.model.edges() {
                                            if !context_edges.contains(&eidx) {
                                                context_edges.push(eidx);
                                            }
                                        }
                                        let mut load_ids: Vec<String> = load
                                            .model
                                            .edges()
                                            .iter()
                                            .map(|&eidx| {
                                                graph.components
                                                    [graph.edges[eidx].comp_idx]
                                                    .id
                                                    .clone()
                                            })
                                            .collect();
                                        load_ids.sort();
                                        load_ids.dedup();
                                        #[cfg(test)]
                                        eprintln!(
                                            "  [compile] group {gi}: reflected transformer-\
                                             secondary load group {} (n={}, r_reflected={:.1}) \
                                             into triode-context MNA",
                                            load.load_group, load.turns_ratio, load.r_reflected
                                        );
                                        fused_load_ids = Some(load_ids);
                                        super::boundary_load::
                                            LoadDisposition::ReflectedThroughTransformer {
                                            consumed_group: load.load_group,
                                            turns_ratio: load.turns_ratio,
                                            r_reflected: load.r_reflected,
                                        }
                                    }
                                    Err(reason) => {
                                        super::boundary_load::LoadDisposition::Unloaded {
                                            reason,
                                        }
                                    }
                                }
                            };
                            match fused_load_ids {
                                // Fused: the row is recorded after the stage
                                // push (its stage_key needs the fused comp-id
                                // set).
                                Some(ids) => xfmr_fused = Some((load, ids)),
                                // Analyzed but declined: record for the
                                // dashboard now — build proceeds unchanged.
                                None => pending_boundary_loads.push(
                                    super::boundary_load::PendingBoundaryLoad {
                                        stage_key: group_comp_ids
                                            .iter()
                                            .min()
                                            .cloned()
                                            .unwrap_or_else(|| "~".to_string()),
                                        boundary_node: load.boundary_node,
                                        model: load.model.summarize(&graph),
                                        disposition: disposition.summarize(),
                                    },
                                ),
                            }
                        }

                        // Mark all non-NL context edges as absorbed so their
                        // groups are skipped later (with a fused load this
                        // also consumes the standalone secondary-load group).
                        for &eidx in &context_edges {
                            if graph.effective_edge_kind(eidx)
                                != super::component::EdgeKind::Nonlinear
                            {
                                triode_absorbed_edges.insert(eidx);
                            }
                        }
                        let built = super::rigid::build_general_mna_from_edges_with_supply(
                            &context_edges,
                            &graph,
                            sample_rate,
                            supply_voltage,
                        )
                        .map_err(|e| format!("Group {gi} (triode-context MNA): {e}"))?;
                        // A triode's serial position is normally its grid's hop
                        // distance from `in` (`bfs_dist_from_in_node`). That walk
                        // is undirected AND crosses rails, so once a circuit has a
                        // mid-chain plain two-port transformer (whose magnetic
                        // primary↔secondary coupling is NOT a graph edge), every
                        // triode downstream of the gap is reached only via rail
                        // shortcuts and gets spuriously SMALL distances — scrambling
                        // the serial order (LA-2A's output group sorting ahead of
                        // V1/V2). `group_flow_distances[gi]` is the corrected
                        // rail-blocked, directed, broker-coupled-link-aware distance
                        // (d-2, now honoring the Tight link). Defer to it ONLY when
                        // the circuit actually has such a Tight coupling — which is
                        // exactly the case the rail-crossing grid walk gets wrong.
                        // Circuits without a mid-chain two-port (ordinary tube amps:
                        // their output transformer is its OWN group and its
                        // secondary is `out`, which the broker excludes; cap-coupled
                        // amps with no transformer at all) keep the existing grid
                        // distance, byte-for-byte. (Broker consult only.)
                        let triode_flow_dist = if has_tight_coupled_transformer {
                            group_flow_distances[gi]
                        } else {
                            bfs_dist_from_in_node(grid_node, &graph)
                                .unwrap_or(group_flow_distances[gi])
                        };
                        if let Some((load, load_ids)) = xfmr_fused {
                            // Fused stage owns the reflected load components:
                            // label / comp ids come from the full fused edge
                            // set so later stage lookups (and the boundary-
                            // load stage_key) see them. Only the transformer
                            // family takes this arm — everyone else keeps the
                            // group ids byte-identical.
                            let mut fused_comp_ids: Vec<String> = context_edges
                                .iter()
                                .map(|&eidx| {
                                    graph.components[graph.edges[eidx].comp_idx].id.clone()
                                })
                                .collect();
                            fused_comp_ids.sort();
                            fused_comp_ids.dedup();
                            #[cfg(debug_assertions)]
                            let fused_label = fused_comp_ids.join(",");
                            #[cfg(not(debug_assertions))]
                            let fused_label = String::new();
                            push_stage!(
                                BuiltStage::MultiNl(built),
                                triode_flow_dist,
                                fused_label,
                                is_bypass,
                                fused_comp_ids.clone()
                            );

                            // Consume the standalone load group: the absorbed-
                            // edge skip handles the not-yet-built order; if it
                            // already built (group enumeration order), remove
                            // its stage. Same de-duplication contract as the
                            // C4 passive-path fusion.
                            for id in &load_ids {
                                xfmr_consumed_comp_ids.insert(id.clone());
                            }
                            for idx in (0..stages.len().saturating_sub(1)).rev() {
                                if stage_comp_ids[idx] == load_ids {
                                    #[cfg(test)]
                                    eprintln!(
                                        "  [compile] group {gi}: removed standalone \
                                         secondary-load stage {idx} ({load_ids:?}) — consumed \
                                         by triode-context transformer reflection"
                                    );
                                    stages.remove(idx);
                                    stage_comp_ids.remove(idx);
                                    break;
                                }
                            }
                            pending_boundary_loads.push(
                                super::boundary_load::PendingBoundaryLoad {
                                    stage_key: fused_comp_ids
                                        .iter()
                                        .min()
                                        .cloned()
                                        .unwrap_or_else(|| "~".to_string()),
                                    boundary_node: load.boundary_node,
                                    model: load.model.summarize(&graph),
                                    disposition:
                                        super::boundary_load::LoadDisposition::
                                            ReflectedThroughTransformer {
                                            consumed_group: load.load_group,
                                            turns_ratio: load.turns_ratio,
                                            r_reflected: load.r_reflected,
                                        }
                                        .summarize(),
                                },
                            );
                        } else {
                            push_stage!(
                                BuiltStage::MultiNl(built),
                                triode_flow_dist,
                                group_label.clone(),
                                is_bypass,
                                group_comp_ids.clone()
                            );
                        }
                        continue;
                    }
                }
            }

            // MERGE NOTE: #85 added a blockwise check here, but the branch's
            // restructured flow already performs the blockwise split for both
            // feedback groups (above) and non-feedback groups (the
            // `if !group.has_feedback()` block below, guarded by
            // `!options.skip_blockwise`). The #85 copy was a duplicate against
            // the pre-rework structure and is dropped to avoid double-processing.

            if !group.has_feedback() {
                // Restrict to ONLY the op-amp neg/pos input nodes — not all
                // intermediate nodes of the feedback group. Using all nodes was
                // too broad: a series input resistor (e.g. R_in feeding the
                // mid-node of a bridged-T) would be flagged as feedforward
                // because its output node touches the feedback group's interior.
                // The intent is to detect direct-connect modulator paths
                // (LED→LDR, photocoupler) from an op-amp output to an op-amp
                // input pin, not ordinary series input resistors.
                // Exclude gnd_node from feedback_nodes: op-amps whose pos
                // (or neg) input ties directly to ground are not meaningful
                // feedforward targets — gnd is universally reachable and would
                // cause false positives (e.g. C_tone.b → gnd flagged as
                // feedforward from U1.out → gnd).
                let feedback_nodes: std::collections::HashSet<super::graph::NodeId> = graph
                    .nullor_pins
                    .iter()
                    .flat_map(|pins| [pins.neg_node, pins.pos_node].into_iter())
                    .filter(|&n| n != graph.gnd_node)
                    .collect();
                let main_outputs: std::collections::HashSet<super::graph::NodeId> = graph
                    .nullor_pins
                    .iter()
                    .map(|pins| pins.out_node)
                    .chain(std::iter::once(graph.in_node))
                    .collect();

                // F2 (B2/B3a): an edge spanning {in / op-amp out} ↔
                // {feedback-group node} is NOT automatically a parallel
                // feedforward branch. Series couplers (input cap before an
                // inverting input; R→C interstage coupling) match that shape
                // too, and lowering one to an additive Passthrough bridge
                // cancels the signal: the open-circuited stage emits −x and
                // the blend computes x + (−x) = 0. Only build the bridge when
                // the branch is genuinely parallel:
                //   (a) the convergence node is an op-amp input pin (nullor
                //       neg/pos) of a feedback group, AND
                //   (b) a second serial path from the source into that same
                //       feedback group exists — the group's own claimed edges
                //       also reach the source node (mirrors the
                //       sources_with_feedback check in the phase-2 detector
                //       below).
                // When either fails, fall through to the normal serial
                // (blockwise/SPQR) build.
                let is_audio_node = |n: super::graph::NodeId| -> bool {
                    n != graph.gnd_node
                        && n != graph.vcc_node
                        && !graph.supply_nodes.contains(&n)
                        && !graph.ac_ground_nodes.contains(&n)
                };
                let genuinely_parallel =
                    |source: super::graph::NodeId, converge: super::graph::NodeId| -> bool {
                        if !is_audio_node(converge) {
                            return false;
                        }
                        feedback_groups.iter().any(|fg| {
                            if !fg.has_feedback() {
                                return false;
                            }
                            // (a) converge is a nullor input pin of this group.
                            let pin_match = fg.active_edges.iter().any(|&ae| {
                                let ae_comp = graph.edges[ae].comp_idx;
                                graph.nullor_pins.iter().any(|p| {
                                    p.comp_idx == ae_comp
                                        && (p.neg_node == converge || p.pos_node == converge)
                                })
                            });
                            if !pin_match {
                                return false;
                            }
                            // (b) the group's own edges reach the source node.
                            fg.all_edges().iter().any(|&fe| {
                                let f = &graph.edges[fe];
                                f.node_a == source || f.node_b == source
                            })
                        })
                    };

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

                    let converge = if source == e.node_a {
                        e.node_b
                    } else {
                        e.node_a
                    };
                    if !genuinely_parallel(source, converge) {
                        continue;
                    }

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

            // ── Transformer-secondary load reflection (GAP F, narrow + gated) ──
            // A transformer's PRIMARY-side stage never feels a load on its
            // SECONDARY: the windings couple magnetically (`coupled_nodes`),
            // not conductively, so the loaded secondary always splits into its
            // own flow group. The primary-side solve then terminates the
            // transformer with an OPEN secondary (bare magnetizing branch) and
            // the standalone load stage divides by an arbitrary source
            // resistance — measured +20…+26 dB vs ngspice on the CE → 10:1 OT
            // → 1 kΩ probes (the LA-2A GAP F step-down family).
            //
            // Fix: fuse the secondary load-group edges into THIS group's build
            // and lower via `build_passive_rtype_stage`. Its MNA stamps the
            // full transformer skeleton (DCR + leakage + magnetizing/JA core +
            // ideal turns-ratio branch) with REAL secondary nodes, so the load
            // current flows through the turns-ratio stamp and the primary
            // feels `n²·R` — the reflection is performed by the same model
            // that matches the ngspice coupled-inductor fixtures to ~−90 dB
            // when transformer and load share a stage. The standalone load
            // stage is consumed (skip-set + built-stage removal) so the load
            // is never applied twice. ANALYSIS (the exact recognized shape)
            // and POLICY (passive-only target, `PK_XFMR_REFLECT_DISABLE`
            // opt-out) live in `boundary_load`; the decision — reflected or
            // declined — is recorded on `CompiledPedal::boundary_loads`.
            {
                let xfmr_analysis = super::boundary_load::analyze_transformer_secondary_load(
                    gi,
                    &feedback_groups,
                    &graph,
                    &cut_edges,
                );
                if let Some(load) = xfmr_analysis {
                    // A load group can only be consumed ONCE. Two transformers
                    // whose secondaries share a single load group (pathological
                    // but constructible) must not both fuse it — the second
                    // fusion would stamp the load edges into two stages
                    // (double loading) with no standalone stage left to remove.
                    let load_already_consumed = load.model.edges().iter().any(|&eidx| {
                        xfmr_consumed_comp_ids
                            .contains(&graph.components[graph.edges[eidx].comp_idx].id)
                    });
                    let mut fused: Option<(WdfStage, Vec<usize>)> = None;
                    let disposition = if load_already_consumed {
                        super::boundary_load::LoadDisposition::Unloaded {
                            reason: "gate:load-group-already-consumed",
                        }
                    } else {
                        match super::boundary_load::gate_transformer_reflection(group, &graph) {
                            Ok(()) => {
                                let mut fused_edges = group_edges.clone();
                                for &eidx in load.model.edges() {
                                    if !fused_edges.contains(&eidx) {
                                        fused_edges.push(eidx);
                                    }
                                }
                                match build_passive_rtype_stage(
                                    &fused_edges,
                                    &graph,
                                    sample_rate,
                                    &bias_node_voltages,
                                ) {
                                    Some(wdf) => {
                                        #[cfg(test)]
                                        eprintln!(
                                            "  [compile] group {gi}: reflected transformer-secondary \
                                             load group {} (n={}, r_reflected={:.1}) into primary-side \
                                             PassiveRType MNA",
                                            load.load_group, load.turns_ratio, load.r_reflected
                                        );
                                        fused = Some((wdf, fused_edges));
                                        super::boundary_load::LoadDisposition::ReflectedThroughTransformer {
                                            consumed_group: load.load_group,
                                            turns_ratio: load.turns_ratio,
                                            r_reflected: load.r_reflected,
                                        }
                                    }
                                    // Lowering failed (degenerate terminals) — leave
                                    // the boundary open and fall through to the
                                    // normal build path; nothing is consumed.
                                    None => super::boundary_load::LoadDisposition::Unloaded {
                                        reason: "gate:passive-rtype-lowering-failed",
                                    },
                                }
                            }
                            Err(reason) => {
                                super::boundary_load::LoadDisposition::Unloaded { reason }
                            }
                        }
                    };
                    if let Some((wdf, fused_edges)) = fused {
                        // Fused stage owns the load components: refresh label /
                        // comp ids so later stage lookups see them.
                        let mut fused_comp_ids: Vec<String> = fused_edges
                            .iter()
                            .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.clone())
                            .collect();
                        fused_comp_ids.sort();
                        fused_comp_ids.dedup();
                        #[cfg(debug_assertions)]
                        let fused_label = fused_comp_ids.join(",");
                        #[cfg(not(debug_assertions))]
                        let fused_label = String::new();
                        push_stage!(
                            BuiltStage::Wdf(wdf),
                            group_flow_distances[gi],
                            fused_label,
                            is_bypass,
                            fused_comp_ids.clone()
                        );

                        // Consume the standalone load group: skip it if it has
                        // not built yet; remove its stage if it already has
                        // (non-feedback build order is group-enumeration order,
                        // so both orders occur). Same de-duplication contract
                        // as the C1 trailing-output fusion.
                        let mut load_ids: Vec<String> = load
                            .model
                            .edges()
                            .iter()
                            .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.clone())
                            .collect();
                        load_ids.sort();
                        load_ids.dedup();
                        for id in &load_ids {
                            xfmr_consumed_comp_ids.insert(id.clone());
                        }
                        // Exclude the fused stage just pushed (last index).
                        for idx in (0..stages.len().saturating_sub(1)).rev() {
                            if stage_comp_ids[idx] == load_ids {
                                #[cfg(test)]
                                eprintln!(
                                    "  [compile] group {gi}: removed standalone secondary-load \
                                     stage {idx} ({load_ids:?}) — consumed by transformer \
                                     reflection"
                                );
                                stages.remove(idx);
                                stage_comp_ids.remove(idx);
                                break;
                            }
                        }
                        pending_boundary_loads.push(super::boundary_load::PendingBoundaryLoad {
                            stage_key: fused_comp_ids
                                .iter()
                                .min()
                                .cloned()
                                .unwrap_or_else(|| "~".to_string()),
                            boundary_node: load.boundary_node,
                            model: load.model.summarize(&graph),
                            disposition: disposition.summarize(),
                        });
                        continue;
                    }
                    // Analyzed but declined (gate / lowering failure): record
                    // the row for the dashboard and fall through to the normal
                    // build path — byte-identical behavior.
                    pending_boundary_loads.push(super::boundary_load::PendingBoundaryLoad {
                        stage_key: group_comp_ids
                            .iter()
                            .min()
                            .cloned()
                            .unwrap_or_else(|| "~".to_string()),
                        boundary_node: load.boundary_node,
                        model: load.model.summarize(&graph),
                        disposition: disposition.summarize(),
                    });
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
                } else if !group_has_nonlinear && spqr_stages.is_empty() && group_has_runtime_pot {
                    // All-passive group that reduced to a rigid (non-series-
                    // parallel) R-node AND carries a runtime pot: spqr_to_dyn_node
                    // returns None for the R-node, so the AllPassive arm of
                    // collect_stages warns and drops the entire stage — and its
                    // declared pots vanish with it (symptom: dead Tone/Level
                    // controls, unity passthrough).
                    //
                    // The pot guard is load-bearing: an all-passive group with NO
                    // pot (e.g. a bare input/output coupling RC feeding an active
                    // device's pin) is HARMLESS to drop — the serial chain carries
                    // the signal through as a passthrough, which is correct. Only a
                    // dropped group that owns a CONTROL needs rebuilding, and only
                    // then is it worth the risk of mis-terminating a coupling
                    // network as a standalone 2-port (which silences it — observed
                    // on dyna_comp's pot-less input-coupling group).
                    //
                    // Rebuild it exactly the way the normal `SpqrStage::Rigid`
                    // all-passive branch does — via `build_passive_rtype_stage`,
                    // which lowers the group into a WDF PassiveRType stage with
                    // terminal-derived input/output ports (compute_group_terminals)
                    // and live pot leaves (both 2-terminal rheostats like Tone and
                    // 3-terminal wiper dividers like Level). This preserves the
                    // mid-chain 2-port transfer; the generic rigid MNA builder does
                    // NOT — it models a 1-port (voltage-source-in / sample-at-`out`)
                    // and silences any mid-chain passive group that does not contain
                    // the global `out` (output port unbound -> zero c-vector).
                    // Narrow: only fires when SPQR produced ZERO stages for an
                    // all-passive group that owns a pot control.
                    let built = if let Some(wdf) = build_passive_rtype_stage(
                        &group_edges,
                        &graph,
                        sample_rate,
                        &bias_node_voltages,
                    ) {
                        eprintln!(
                            "  [compile] group {gi}: SPQR dropped all-passive stage (rigid R-node), rebuilt as PassiveRType WDF"
                        );
                        BuiltStage::Wdf(wdf)
                    } else {
                        // PassiveRType could not lower this group (e.g. degenerate
                        // terminals); last-resort rigid MNA so the stage at least
                        // exists rather than being silently dropped.
                        eprintln!(
                            "  [compile] group {gi}: SPQR dropped all-passive stage; PassiveRType lowering failed, using rigid MNA"
                        );
                        build_rigid_from_group_with_hints(
                            group_edges,
                            &graph,
                            sample_rate,
                            Some(group),
                            supply_voltage,
                            None,
                            !options.disable_iir,
                            &pedal.init_hints,
                        )
                        .map_err(|e| format!("Group {gi} (all-passive rigid fallback): {e}"))?
                    };
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
                        &bias_node_voltages,
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

    // ── Edge accounting guard (pedalkernel-ffkl) ─────────────────────────────
    // After stage assembly every graph edge must be accounted for: solved in
    // some stage (its component id is claimed by a stage's comp-id set),
    // consumed by an explicit mechanism (triode-context absorption, blockwise
    // coupling, transformer reflection, ground-clip merge, delayed-cut carry),
    // or deliberately excluded with a reason (op-amp nullor edges lower via
    // the op-amp pipeline; behavioral coupler edges bind at bind-time). An
    // edge that falls through means a component silently vanished from the
    // compiled circuit — the LA-2A T_out failure family. Mode is governed by
    // PK_EDGE_GUARD (error by default; see `report_dropped_edges`).
    //
    // NOTE this comp-id-granular sweep is the OUTER net; the INNER net is the
    // builder-level guard in `rigid::general::stamp_passive_edges`, which
    // catches edge-granular drops inside a build (a stage can claim a comp id
    // while dropping one of its edges — exactly how T_out was lost while V3's
    // stage label listed it).
    {
        use super::component::EdgeKind;
        let claimed_comp_ids: std::collections::HashSet<&String> =
            stage_comp_ids.iter().flatten().collect();
        let ground_clip_edges: std::collections::HashSet<usize> = ground_clip_built
            .iter()
            .flat_map(|&gi| feedback_groups[gi].all_edges())
            .collect();
        let mut unaccounted: Vec<usize> = Vec::new();
        for eidx in 0..graph.edges.len() {
            let comp = &graph.components[graph.edges[eidx].comp_idx];
            let kind = graph.effective_edge_kind(eidx);
            // Vcvs (op-amp nullor) and Vccs (OTA transconductance) edges
            // lower via the active-IC pipeline (opamp analysis / OTA
            // envelope resolution), Behavioral edges bind at bind-time —
            // deliberate exclusions with a home elsewhere.
            let accounted =
                matches!(kind, EdgeKind::Vcvs | EdgeKind::Vccs | EdgeKind::Behavioral)
                    || claimed_comp_ids.contains(&comp.id)
                    || triode_absorbed_edges.contains(&eidx)
                    || guard_excluded_edges.contains(&eidx)
                    || cut_edges.cuts.contains_key(&eidx)
                    || bkm_consumed_comp_ids.contains(&comp.id)
                    || xfmr_consumed_comp_ids.contains(&comp.id)
                    || ground_clip_edges.contains(&eidx);
            if !accounted {
                unaccounted.push(eidx);
            }
        }
        if !unaccounted.is_empty() && std::env::var("PK_EDGE_GUARD_DEBUG").is_ok() {
            eprintln!(
                "[edge-guard-debug] stages={} stage_comp_ids={stage_comp_ids:?} \
                 absorbed={triode_absorbed_edges:?} cuts={:?} bkm={bkm_consumed_comp_ids:?} \
                 xfmr={xfmr_consumed_comp_ids:?}",
                stages.len(),
                cut_edges.cuts.keys().collect::<Vec<_>>(),
            );
        }
        report_dropped_edges(
            "stage assembly",
            &unaccounted,
            &graph,
            // pedalkernel-x5ac (promoted in the y9hz batch): error by
            // default, like the builder-level guard — corpus is quiet now
            // that the blockwise Some(empty) ghost drop is fixed.
            EdgeGuardMode::Error,
        )?;
    }

    // ── Output-follower injection wiring (GAP 4c, pedalkernel-0lsv) ────────
    // Runs BEFORE the sorts, where `stage_comp_ids[si]` is still 1:1 with
    // `stages[si]` (the sorts move the fields we set along with each stage).
    //
    // A non-inverting output FOLLOWER (uberdrive/lgsm Q2 emitter follower, fed
    // IC2.out -> C7 -> volume divider -> Q2.base) is a directed dead-end under the
    // stage-ordering metric, so `compute_group_flow_distances` re-slots its group
    // into the MAX-band retry (3a) — but its MultiNl stage still reads the serial
    // `signal`, which is 0.0 there. Give it a value to read: set the MultiNl
    // stage's `injection_node_id` to the upstream op-amp output node (IC2.out, a
    // node the tone stage already produces) and make that upstream stage PUBLISH
    // to `node_signals` (`output_node_id`). The follower's own in-group MNA (which
    // spans C7 / R11 / VOLUME / R12 / C8) performs the volume division against the
    // injected value — no new node_signals writer is needed (design variant b).
    //
    // GATE: only groups that are a directed dead-end (`directed` reaches no group
    // node) yet reachable once op-amp control pins are crossed
    // (`boundary_reachable` finite) qualify — exactly the 3a retry set. LA-2A /
    // pultec / neve photocoupler and tube groups inject at a device pin (not a
    // control-crossing boundary; no op-amp control pin exists to cross) and do NOT
    // qualify, so their goldens stay byte-identical.
    {
        use super::graph::NodeId;
        let follower_debug = std::env::var("PK_FOLLOWER_DEBUG").is_ok();
        let directed = super::signal_flow::directed_signal_distances_from_in(&graph);
        let boundary = super::signal_flow::boundary_reachable_distances_from_in(&graph);

        let mut follower_injections: Vec<(NodeId, Vec<String>, Vec<String>)> = Vec::new();
        for (gi, group) in feedback_groups.iter().enumerate() {
            if group.has_feedback() || group.active_edges.is_empty() {
                continue;
            }
            let edges = group.all_edges();
            if edges.is_empty() {
                continue;
            }
            let edge_set: std::collections::HashSet<usize> = edges.iter().copied().collect();
            let group_nodes: std::collections::HashSet<NodeId> = edges
                .iter()
                .flat_map(|&eidx| {
                    let e = &graph.edges[eidx];
                    [e.node_a, e.node_b]
                })
                .collect();
            // Directed dead-end (no group node reachable under the base metric)
            // yet reachable once control pins are crossed.
            let directed_reachable = group_nodes.iter().any(|n| directed.contains_key(n));
            let boundary_reachable = group_nodes.iter().any(|n| boundary.contains_key(n));
            // Injection node = the group's INPUT BOUNDARY node: a group node also
            // touched by an edge OUTSIDE this group (so an upstream stage drives
            // it) and NOT a rail. Deterministic: nearest to `in` under the
            // control-crossing metric, ties broken by NodeId. This mirrors
            // general.rs `find_input_boundary_node`, so the runtime read node ==
            // the compile-time adapted-VS injection boundary (uberdrive/lgsm:
            // C7.b/R11.a).
            let is_rail = |n: NodeId| {
                n == graph.gnd_node
                    || n == graph.vcc_node
                    || graph.supply_nodes.contains(&n)
                    || graph.ac_ground_nodes.contains(&n)
            };
            let boundary_nodes: std::collections::HashSet<NodeId> = graph
                .edges
                .iter()
                .enumerate()
                .filter(|(eidx, _)| !edge_set.contains(eidx))
                .flat_map(|(_, e)| [e.node_a, e.node_b])
                .filter(|n| group_nodes.contains(n) && !is_rail(*n))
                .collect();
            let inj_node = boundary_nodes
                .iter()
                .copied()
                .filter(|n| boundary.contains_key(n) && *n != graph.in_node)
                .min_by_key(|n| (*boundary.get(n).unwrap_or(&usize::MAX), *n));
            if follower_debug {
                let names: Vec<String> = group_nodes
                    .iter()
                    .map(|&n| {
                        graph
                            .node_names
                            .iter()
                            .find(|(_, &id)| id == n)
                            .map(|(k, _)| k.clone())
                            .unwrap_or_else(|| format!("n{n}"))
                    })
                    .collect();
                let comps: Vec<String> = edges
                    .iter()
                    .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.clone())
                    .collect();
                eprintln!(
                    "[PK_FOLLOWER] group {gi}: directed_reachable={directed_reachable} \
                     boundary_reachable={boundary_reachable} inj_node={inj_node:?} \
                     comps={comps:?} nodes={names:?}"
                );
            }
            if directed_reachable || !boundary_reachable {
                continue;
            }
            let Some(inj_node) = inj_node else {
                continue;
            };
            let comp_ids: Vec<String> = edges
                .iter()
                .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.clone())
                .collect();
            // Producer comps: components of OUTSIDE edges touching the injection
            // boundary node — the upstream stage that drives it and must publish
            // its per-sample value at `inj_node` to the bus (e.g. C7, whose `.a`
            // is IC2.out and `.b` is the boundary). Ordered nearest-`in` first so
            // the true upstream (not a same-node sibling shunt) is tried first.
            let mut producers: Vec<(usize, String)> = graph
                .edges
                .iter()
                .enumerate()
                .filter(|(eidx, e)| {
                    !edge_set.contains(eidx) && (e.node_a == inj_node || e.node_b == inj_node)
                })
                .map(|(_, e)| {
                    let other = if e.node_a == inj_node { e.node_b } else { e.node_a };
                    let d = boundary.get(&other).copied().unwrap_or(usize::MAX);
                    (d, graph.components[e.comp_idx].id.clone())
                })
                .collect();
            producers.sort();
            let producer_ids: Vec<String> = producers.into_iter().map(|(_, id)| id).collect();
            follower_injections.push((inj_node, comp_ids, producer_ids));
        }

        for (inj_node, comp_ids, producer_ids) in &follower_injections {
            // Wire the follower's MultiNl stage to READ the injection node.
            let mut wired = false;
            for (si, stage) in stages.iter_mut().enumerate() {
                if let Stage::MultiNl(m) = stage {
                    if m.injection_node_id != usize::MAX {
                        continue;
                    }
                    if stage_comp_ids[si].iter().any(|id| comp_ids.contains(id)) {
                        m.injection_node_id = *inj_node;
                        wired = true;
                        if follower_debug {
                            eprintln!(
                                "  → output follower: MultiNl stage {si} reads node {inj_node} \
                                 (injection), was serial-in 0.0"
                            );
                        }
                        break;
                    }
                }
            }
            if !wired {
                if follower_debug {
                    eprintln!(
                        "[PK_FOLLOWER] NOT wired: inj_node={inj_node} follower_comps={comp_ids:?}"
                    );
                }
                continue;
            }
            // Make the upstream producer stage PUBLISH the injection node to the
            // bus. The producer owns an outside edge touching `inj_node` (e.g. the
            // tone-stage output coupling that terminates at C7.b).
            for prod_id in producer_ids {
                let mut published = false;
                for (si, stage) in stages.iter_mut().enumerate() {
                    if !stage_comp_ids[si].iter().any(|id| id == prod_id) {
                        continue;
                    }
                    let ok = match stage {
                        Stage::Wdf(w) if !w.bypass_serial && w.output_node_id == usize::MAX => {
                            w.output_node_id = *inj_node;
                            true
                        }
                        Stage::BlackFeedback(b)
                            if !b.bypass_serial && b.output_node_id == usize::MAX =>
                        {
                            b.output_node_id = *inj_node;
                            true
                        }
                        Stage::MultiNl(m)
                            if !m.bypass_serial && m.output_node_id == usize::MAX =>
                        {
                            m.output_node_id = *inj_node;
                            true
                        }
                        // The tone stage is typically a StateSpace (uberdrive
                        // IC2). Setting its output_node_id makes it PUBLISH the
                        // node to the bus; the runtime StateSpace path then routes
                        // to node_signals instead of the serial chain, which is
                        // correct here — the follower (the only consumer of the
                        // tone output) reads it from the bus and re-drives serial.
                        Stage::StateSpace(ss) if !ss.bypass_serial && ss.output_node_id == usize::MAX => {
                            ss.output_node_id = *inj_node;
                            true
                        }
                        // Already publishes some node (or is a bypass/serial-only
                        // stage): if it already writes `inj_node`, we are done.
                        Stage::Wdf(w) if w.output_node_id == *inj_node => true,
                        Stage::BlackFeedback(b) if b.output_node_id == *inj_node => true,
                        Stage::MultiNl(m) if m.output_node_id == *inj_node => true,
                        Stage::StateSpace(ss) if ss.output_node_id == *inj_node => true,
                        _ => false,
                    };
                    if ok {
                        published = true;
                        if follower_debug {
                            eprintln!(
                                "  → output follower: upstream stage {si} ({prod_id}) publishes \
                                 node {inj_node} to bus"
                            );
                        }
                        break;
                    }
                }
                if published {
                    break;
                }
            }
        }
    }

    // Defect B tertiary tiebreak: when two stages share a signal_flow_distance,
    // resolve their order DETERMINISTICALLY by the minimum stable component id
    // in each stage (netlist names are stable across compiles/processes). This
    // makes the serial chain reproducible even when `find_flow_groups` returns
    // tied groups in a process-dependent order. `stage_comp_ids` is populated in
    // both debug and release. A stage with no comp ids sorts last among its tie.
    let mut stage_min_comp_id: Vec<String> = stage_comp_ids
        .iter()
        .map(|ids| ids.iter().min().cloned().unwrap_or_else(|| "~".to_string()))
        .collect();
    let stage_dist = |s: &Stage| -> usize {
        match s {
            Stage::Wdf(w) => w.signal_flow_distance,
            Stage::Iir(i) => i.signal_flow_distance,
            Stage::StateSpace(ss) => ss.signal_flow_distance,
            Stage::MultiNl(m) => m.signal_flow_distance,
            Stage::BlackFeedback(b) => b.signal_flow_distance,
            Stage::Blockwise(k) => k.signal_flow_distance,
            Stage::KMethod { .. } => usize::MAX,
            Stage::SerialDelayedFeedback(s) => s.signal_flow_distance,
        }
    };

    // Sort stages by (signal_flow_distance, min_comp_id). Index-permutation
    // based so the parallel `stage_min_comp_id` table stays aligned through the
    // sort (the second, post-feedforward re-sort below reuses it).
    // (`stage_comp_ids` is intentionally left in its build order — the VCO
    // generator pass below indexes it by the same pre-sort positions it always
    // has, so we do not perturb that pre-existing behavior.)
    {
        let mut perm: Vec<usize> = (0..stages.len()).collect();
        perm.sort_by(|&a, &b| {
            stage_dist(&stages[a])
                .cmp(&stage_dist(&stages[b]))
                .then_with(|| stage_min_comp_id[a].cmp(&stage_min_comp_id[b]))
        });
        apply_permutation(&mut stages, &perm);
        apply_permutation(&mut stage_min_comp_id, &perm);
    }

    // ── Generator (VCO, N=0) source placement (spec §4) ───────────────────
    // A generator has no audio input — there is no galvanic gap to split.
    // Instead its wave-output node is a *source seed*: the runtime injects the
    // VCO sample into `node_signals` at that node (processor.rs VCO tick loop),
    // and the stage immediately downstream (e.g. the output protection resistor
    // VCO1.saw -> R_saw -> out) must READ that node rather than the serial
    // chain. Mark that consuming stage feedforward with injection_node_id =
    // the generator output node, reusing the existing port-write/feedforward
    // read path. No new per-sample machinery — this is the §4 N=0 branch made
    // real, the analogue of the input-port VS for a pure source.
    {
        // Generator output nodes from every block whose io() has no audio input.
        let mut gen_out_nodes: Vec<super::graph::NodeId> = Vec::new();
        for block in super::dsp_block::all_generator_output_nodes(pedal, &graph) {
            if !gen_out_nodes.contains(&block) {
                gen_out_nodes.push(block);
            }
        }
        for vn in gen_out_nodes {
            // The consuming stage is the one whose group has an edge touching
            // the generator output node (the first passive the VCO drives).
            let mut consumer_comp_ids: std::collections::HashSet<String> =
                std::collections::HashSet::new();
            for e in &graph.edges {
                if e.node_a == vn || e.node_b == vn {
                    consumer_comp_ids.insert(graph.components[e.comp_idx].id.clone());
                }
            }
            if consumer_comp_ids.is_empty() {
                continue;
            }
            for (si, stage) in stages.iter_mut().enumerate() {
                if let Stage::Wdf(w) = stage {
                    if w.is_feedforward || w.bypass_serial {
                        continue;
                    }
                    let ids = &stage_comp_ids[si];
                    if ids.iter().any(|id| consumer_comp_ids.contains(id)) {
                        w.is_feedforward = true;
                        w.injection_node_id = vn;
                        #[cfg(test)]
                        eprintln!(
                            "  → generator source: stage {si} reads VCO node {vn} (feedforward)"
                        );
                        break;
                    }
                }
            }
        }
    }

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
                        if let std::collections::btree_map::Entry::Vacant(e) = dist.entry(next) {
                            e.insert(d + 1);
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
                    Stage::BlackFeedback(b)
                        if !b.bypass_serial && b.output_node_id == usize::MAX =>
                    {
                        // BlackFeedback stages also need output_node_id for feedforward
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
                    _ => {}
                }
            }
        }
    }

    // Re-sort: feedforward stages may have changed distance.
    // Secondary key: feedforward stages sort AFTER non-feedforward at same
    // distance. Tertiary key (Defect B): min stable component id, for
    // deterministic ordering among otherwise-tied stages.
    {
        let ff_of = |s: &Stage| -> u8 {
            match s {
                Stage::Wdf(w) => w.is_feedforward as u8,
                _ => 0,
            }
        };
        let mut perm: Vec<usize> = (0..stages.len()).collect();
        perm.sort_by(|&a, &b| {
            stage_dist(&stages[a])
                .cmp(&stage_dist(&stages[b]))
                .then_with(|| ff_of(&stages[a]).cmp(&ff_of(&stages[b])))
                .then_with(|| stage_min_comp_id[a].cmp(&stage_min_comp_id[b]))
        });
        apply_permutation(&mut stages, &perm);
        apply_permutation(&mut stage_min_comp_id, &perm);
    }

    // F13b: wire parallel branches into the convergence mixer. The convergence
    // stage reads its driver voltages from `node_signals`, so each upstream
    // active branch whose output node is one of those drivers must PUBLISH its
    // per-sample output to the bus (and stop overwriting the serial chain — the
    // convergence stage owns the serial output). Branches keep reading the serial
    // `signal`, which is the shared drive (stage 0's conditioned trigger): because
    // branches publish to the bus and never to serial, sibling branches all see
    // the same drive instead of cascading.
    {
        let driver_nodes: std::collections::HashSet<NodeId> = stages
            .iter()
            .filter_map(|s| match s {
                Stage::Wdf(w) => w.convergence.as_ref(),
                _ => None,
            })
            .flat_map(|cs| cs.branches.iter().map(|b| b.source_node_id))
            .collect();
        if !driver_nodes.is_empty() {
            for stage in &mut stages {
                if let Stage::StateSpace(ss) = stage {
                    if let Some(out_node) = ss.output_binding.map(|b| b.binding_id.get()) {
                        if driver_nodes.contains(&out_node) {
                            ss.output_node_id = out_node;
                        }
                    }
                }
            }
        }
    }

    // When thermal is enabled, snapshot base BJT models so apply_thermal()
    // can modulate them without accumulating multipliers.
    if options.thermal {
        for stage in &mut stages {
            if let Stage::Wdf(wdf) = stage {
                if let RootKind::Bjt(bjt) = &wdf.root {
                    wdf.base_bjt_model = Some(bjt.model);
                }
            }
        }
    }

    // Envelope-follower → JFET-leaf modulation bindings (audit gap G2).
    let envelopes = super::bind::build_envelope_jfet_bindings(pedal, &stages, sample_rate);

    let mut compiled = CompiledPedal {
        stages,
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
        envelopes,
        bbds: Vec::new(),
        delay_lines,
        springs: Vec::new(),
        vcos: Vec::new(),
        vcas: Vec::new(),
        thermal: if options.thermal {
            Some(ThermalModel::silicon_standard(sample_rate))
        } else {
            None
        },
        tolerance_seed: 0,
        opamp_stages: Vec::new(),
        power_supply: None,
        metrics_accumulator: None,
        metrics_buffer: None,
        #[cfg(feature = "diag")]
        diag_ring: None,
        input_loading: None,
        output_loading: None,
        output_dc_block: None,
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
        internal_ports: Vec::new(),
        // Boundary-load decision table: pending rows recorded at the fusion
        // call site, resolved against the FINAL stage order via the
        // `stage_min_comp_id` table (kept aligned with `stages` through both
        // permutation sorts above).
        boundary_loads: super::boundary_load::resolve_boundary_load_bindings(
            pending_boundary_loads,
            &stage_min_comp_id,
        ),
        detector_led_coupling: None,
        initialized: false,
    };
    compiled.set_supply_voltage(supply_voltage);

    // Boundary-load decision table diagnostic (bead pedalkernel-lkf1.2) —
    // cfg(test)-gated like the other `[compile]` prints; purely additive, no
    // behavior. One line per analyzed output boundary so a silently-Unloaded
    // boundary is a visible, greppable fact with its reason. An EMPTY table is
    // itself a triage signal (no feedback group reached the general-MNA call
    // site — e.g. the whole pedal compiled via blockwise/other paths), distinct
    // from a populated row with `Unloaded{reason}`.
    #[cfg(test)]
    {
        if compiled.boundary_loads.is_empty() {
            eprintln!(
                "  [compile] boundary-load table: EMPTY — no feedback group reached the \
                 general-MNA call site (output boundary unanalyzed)"
            );
        }
        for b in &compiled.boundary_loads {
            let stage = if b.upstream_stage == usize::MAX {
                "?".to_string()
            } else {
                b.upstream_stage.to_string()
            };
            eprintln!(
                "  [compile] boundary-load: stage {stage} @ node {} — {:?} ⇒ {:?}",
                b.boundary_node, b.model, b.disposition
            );
        }
    }

    // Bind pot controls to their stages (WDF, IIR, MultiNl).
    super::spqr_control::bind_controls(pedal, &mut compiled);

    // Lower + bind every registered DSP block's runtime instances (BBD
    // clock/feedback/mix pots and clock LFOs; VCA gain + envelope-follower CV
    // bindings). Must run after the stage list is final: VCA binding resolves
    // detector taps against `compiled.stages`.
    super::dsp_block::bind_runtime_all(pedal, &mut compiled, sample_rate)?;

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
            port_bindings.push(pedalkernel_rt::processor::NamedPortBinding {
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
                let mut injected = false;
                for (si, stage) in compiled.stages.iter_mut().enumerate() {
                    if let pedalkernel_rt::processor::Stage::Wdf(ref mut wdf) = stage {
                        // Primary path: NL-root WDF tree leaf wrapping.
                        let found = wdf.tree.wrap_leaf_with_vs(&comp_id, &port_binding.name);
                        if found {
                            wdf.tree.recompute();
                            port_binding.stage_idx = si;
                            #[cfg(test)]
                            eprintln!(
                                "  Injected port VS '{}' at leaf '{}' in stage {si}",
                                port_binding.name, comp_id
                            );
                            injected = true;
                            break;
                        }
                        // Fallback: PassiveRType MNA superposition injection.
                        if wdf.register_port_vs_injection(&port_binding.name, port_binding.node_id)
                        {
                            port_binding.stage_idx = si;
                            #[cfg(test)]
                            eprintln!(
                                "  Injected port VS '{}' at node {} in stage {si} (PassiveRType MNA)",
                                port_binding.name, port_binding.node_id
                            );
                            injected = true;
                            break;
                        }
                    }
                }
                let _ = injected;
            }
        }

        compiled.ports = port_bindings;
        compiled.port_values = port_values;
    }

    // Phase 2b: build the internal delayed-port table for a DELAYED detector.
    // Compiler-synthesized (separate from the user `ports` Vec): cross-sample
    // (z⁻¹) carries that deliver the cut forward taps into the de-fused detector
    // sub-network one sample late, and store the detector output (`EL_drive`) for
    // Phase 3. NARROW: fires only when `detector_control_nodes` found a true
    // cross-network feedback detector (LA-2A opto leveler) AND the broker cut a
    // tap mouth — non-detector circuits get an empty table (byte-identical).
    populate_detector_internal_ports(
        &mut compiled,
        &graph,
        &cut_edges,
        &detector_seed_nodes,
        &stage_comp_ids,
    );

    // Cache raw pointers to all VS leaves for zero-cost runtime access.
    // Must be after port binding (wrap_leaf_with_vs) and recompute.
    compiled.cache_all_vs_pointers();

    Ok(compiled)
}

/// Phase 2b — populate `compiled.internal_ports` for a DELAYED feedback detector.
///
/// The broker (`delayed_cut_edges`) de-fuses the detector from the forward audio
/// path by cutting the tap-mouth edges (the `out -> C_sc` feedback mouth and the
/// `in -> fork`/`R_ff` feed-forward mouth — see step 4 + 4b of `delayed_cut_edges`).
/// After the cut the forward `in` is no longer shorted by the Compress fork arm,
/// and the de-fused detector sub-chain (front-end → V4 → V5 → `EL_drive`) solves
/// on the program signal it still sees through the forward serial routing (the
/// feed-forward tap), producing a program-dependent EL-drive value.
///
/// This routine wires the genuinely-new cross-sample (z⁻¹) piece — the EL-DRIVE
/// CARRY port — that captures and stores that detector output:
///
///   * EL-DRIVE CARRY port: `source = the Behavioral coupler's LED-anode node`
///     (the detector output, `EL_drive.b == PC1.led.a`), `consumer = usize::MAX`
///     (carry-only). The detector's solved EL-drive value is captured every
///     sample into `prev_value`, AVAILABLE for Phase 3 (which will set
///     `consumer = PC1.led` to apply gain reduction from the carried value) and
///     for the 2b tracking measurement (`la2a_detector_el_drive_tracks_program`).
///     NO gain reduction is applied here — 2b only COMPUTES and STORES it.
///
/// To make the EL-drive node observable for the carry, the stage that owns the
/// EL-drive driver component is told to PUBLISH its solved output node into
/// `node_signals` (`output_node_id = el_node`); this does NOT alter the serial
/// audio signal, it only ALSO exposes the node for the end-of-sample z⁻¹ capture.
///
/// The DELAYED feedback-tap mix (re-injecting the `out` node into the detector
/// front one sample late) belongs to Phase 3, where the GR loop is actually
/// closed; in 2b the detector is open-loop (no GR) so the feed-forward solve
/// suffices to demonstrate program tracking. The internal-port table + carry
/// generalize the `SidechainProcessor.cv_delayed` precedent and subsume it.
fn populate_detector_internal_ports(
    compiled: &mut CompiledPedal,
    graph: &CircuitGraph,
    cut_edges: &super::boundary_rules::DelayedCutSet,
    detector_seeds: &std::collections::HashSet<super::graph::NodeId>,
    stage_comp_ids: &[Vec<String>],
) {
    use super::component::EdgeKind;
    use pedalkernel_rt::processor::{InternalPortBinding, Stage};

    // Narrow gate: only a true delayed detector with a broker tap-mouth cut.
    if detector_seeds.is_empty() || cut_edges.cuts.is_empty() {
        return;
    }

    // The detector OUTPUT node = the cross-network coupler's LED-anode node
    // (the node that drives the photocoupler LED — `EL_drive.b == PC1.led.a`).
    // Derive it from the Behavioral coupling edge's `pin_a` (the driven side).
    // Also record the coupler component id so we can EXCLUDE it when finding the
    // detector-side driver component that owns that node.
    let mut el_drive_node: Option<super::graph::NodeId> = None;
    let mut coupler_comp_id: Option<String> = None;
    for comp in &graph.components {
        for edge in comp.kind.edges() {
            if edge.kind != EdgeKind::Behavioral {
                continue;
            }
            if let Some(&n) = graph.node_names.get(&format!("{}.{}", comp.id, edge.pin_a)) {
                el_drive_node = Some(n);
                coupler_comp_id = Some(comp.id.clone());
            }
        }
    }

    let Some(el_node) = el_drive_node else {
        return;
    };

    // The detector-side component that DRIVES el_node (e.g. the `EL_drive`
    // transformer winding `EL_drive.b`) — any conductive component, other than
    // the coupler, with a graph edge on el_node.
    let driver_comp_id: Option<String> = graph
        .edges
        .iter()
        .filter(|e| e.node_a == el_node || e.node_b == el_node)
        .map(|e| graph.components[e.comp_idx].id.clone())
        .find(|id| Some(id) != coupler_comp_id.as_ref());

    // Make the stage that owns the EL-drive driver PUBLISH its solved output node
    // value into `node_signals` (set `output_node_id = el_node`). The non-feed-
    // forward WDF stage publish path (`processor.rs`) then writes el_node every
    // sample, where the carry-only internal port captures it at end-of-sample.
    // This does NOT change the serial audio signal (the stage still drives the
    // chain as before) — it only ALSO publishes the node for the z⁻¹ capture.
    if let Some(ref drv) = driver_comp_id {
        for (si, comp_ids) in stage_comp_ids.iter().enumerate() {
            if comp_ids.iter().any(|c| c == drv) {
                if let Some(stage) = compiled.stages.get_mut(si) {
                    match stage {
                        Stage::Wdf(w) => w.output_node_id = el_node,
                        Stage::MultiNl(m) => m.output_node_id = el_node,
                        _ => {}
                    }
                }
                break;
            }
        }
    }

    // EL-drive carry-only port (the detector output, for Phase 3 + measurement).
    // carry_idx 0 = the EL-drive carry the Phase-3 LED coupling reads.
    let internal_ports = vec![InternalPortBinding {
        source_node_id: el_node,
        consumer_node_id: usize::MAX,
        gain: 1.0,
        prev_value: 0.0,
    }];

    #[cfg(test)]
    {
        eprintln!(
            "  [2b] detector internal-ports: el_node={el_node} driver={driver_comp_id:?} \
             coupler={coupler_comp_id:?} in={} out={} cut_boundary={:?}",
            graph.in_node, graph.out_node, cut_edges.boundary_nodes
        );
    }

    compiled.internal_ports = internal_ports;

    // ---- Phase 3: close the GR loop (LED -> cell -> forward shunt). --------
    // The carry above stores the detector's solved EL-drive value one sample
    // late. Synthesize the optical coupling that turns that value into actual
    // gain reduction: drive the photocoupler (`coupler_comp_id` = PC1) LED from
    // |EL_drive| each sample. The runtime reads `internal_ports[0].prev_value`
    // (the z⁻¹ closure), rectifies + normalizes it, and calls `set_led_drive`,
    // darkening the LDR shunt leaf in the FORWARD divider -> downward GR.
    if let Some(coupler) = coupler_comp_id {
        // Find the WDF stage that owns the photocoupler's LDR leaf (its
        // conductive forward-divider position — the shunt cell ahead of V1).
        let mut led_stage_idx: Option<usize> = None;
        for (si, comp_ids) in stage_comp_ids.iter().enumerate() {
            if comp_ids.iter().any(|c| c == &coupler) {
                led_stage_idx = Some(si);
                break;
            }
        }
        if let Some(stage_idx) = led_stage_idx {
            // Normalization: a loud-program detector drives EL_drive to ~15 V
            // peak (see la2a_detector_el_drive_tracks_program); quiet ~0.05 V.
            // scale = 1/15 maps loud -> ~full illumination, quiet -> near dark.
            // The T4B two-rate cell (set_led_drive) provides attack/release.
            compiled.detector_led_coupling = Some(pedalkernel_rt::processor::DetectorLedCoupling {
                carry_idx: 0,
                comp_id: coupler,
                stage_idx,
                scale: 1.0 / 15.0,
            });
            #[cfg(test)]
            eprintln!(
                "  [3] detector LED coupling: carry=0 comp={:?} stage={stage_idx} scale={}",
                compiled.detector_led_coupling.as_ref().map(|c| &c.comp_id),
                1.0 / 15.0
            );
        }
    }
}

/// Build the initial `DelayLineBinding`s (with real tap ratios and configured
/// medium zones). Delegates to the F15 `delay_lowering` [`DspBlock`], which is
/// the single owner of delay-line lowering; `dsp_block::bind_runtime_all` later
/// rebuilds + binds these idempotently. Called here so the early presence
/// guard (`delay_lines.is_empty()`) and the no-edge `CompiledPedal` branches
/// see the correct instances.
fn build_delay_line_bindings(
    pedal: &PedalDef,
    sample_rate: f64,
) -> Vec<pedalkernel_rt::processor::DelayLineBinding> {
    super::delay_lowering::build_delay_line_bindings(pedal, sample_rate)
}

/// SPQR terminal nodes contributed by cross-network couplers.
///
/// A cross-network coupler (today: `photocoupler`) declares a `Behavioral`
/// edge between two galvanically-isolated terminals (the LED side) ALONGSIDE a
/// conductive `graph_role` edge (the LDR side). The two sides live in different
/// electrical networks and must never be fused (see the photocoupler component
/// and `find_flow_groups`). For SPQR to emit a proper stage port on each side,
/// the (isolated) LED side must be a global terminal — the same treatment
/// `in_node`/`out_node` and DspBlock boundary pins receive.
///
/// Detection is generic (no component-type matching): a component qualifies
/// when its `graph_role` is a conductive edge role (so it is a coupler with a
/// real LDR edge, not a pure `GraphRole::Virtual` DSP island like a BBD, which
/// is handled by `all_boundary_nodes`).
///
/// Only the endpoints of the **Behavioral** edge (the galvanically-isolated
/// side that has NO conductive graph edge — the LED) are forced to terminals.
/// The conductive side (the LDR `a`/`b`) already gets a stage port from its
/// real graph edge, so it MUST NOT be forced: forcing it would fragment the
/// passive WDF tree of every existing photocoupler-as-LDR circuit (e.g.
/// `photocoupler_t4b.pedal`), which is a regression, not isolation.
fn coupler_boundary_nodes(graph: &CircuitGraph) -> Vec<NodeId> {
    use super::component::{EdgeKind, GraphRole};
    let mut nodes = Vec::new();
    for comp in &graph.components {
        let role_is_conductive = matches!(
            comp.kind.graph_role(),
            GraphRole::Edge { .. } | GraphRole::ActiveEdge { .. } | GraphRole::CoupledEdge { .. }
        );
        if !role_is_conductive {
            continue;
        }
        for edge in comp.kind.edges() {
            if edge.kind != EdgeKind::Behavioral {
                continue;
            }
            for pin in [edge.pin_a, edge.pin_b] {
                if let Some(&n) = graph.node_names.get(&format!("{}.{}", comp.id, pin)) {
                    if !nodes.contains(&n) {
                        nodes.push(n);
                    }
                }
            }
        }
    }
    nodes
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

    // Tree: VS in series with the divider chain. A dedicated pot divider is
    // normally driven by a low-impedance previous stage/output source; using
    // the generic 10k passive-filter source impedance turns a 100k midpoint
    // level pot into a 0.45x divider before control scaling.
    let tree = with_voltage_source_rp(divider, 1.0);

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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct FetWdfTopology {
    source_follower: bool,
}

fn build_fet_amplifier_passive_tree(
    _edge_indices: &[usize],
    nl_edge_idx: usize,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Option<(DynNode, FetWdfTopology)> {
    let nl_edge = &graph.edges[nl_edge_idx];
    let comp = &graph.components[nl_edge.comp_idx];
    if !(comp.kind.is_jfet() || comp.kind.is_mosfet()) {
        return None;
    }

    let drain_node = nl_edge.node_a;
    let source_node = nl_edge.node_b;
    let gate_node = graph.node_names.get(&format!("{}.gate", comp.id)).copied();

    let source_feeds_output =
        fet_terminal_reaches_output(source_node, drain_node, gate_node, graph);
    let drain_feeds_output = fet_terminal_reaches_output(drain_node, source_node, gate_node, graph);
    let source_follower = source_feeds_output && !drain_feeds_output;
    let passive_edge_indices: Vec<usize> = (0..graph.edges.len()).collect();

    let drain_leg = build_fet_ac_ground_leg(
        drain_node,
        &passive_edge_indices,
        nl_edge_idx,
        graph,
        sample_rate,
        &[source_node],
        gate_node,
    );
    let source_leg = build_fet_ac_ground_leg(
        source_node,
        &passive_edge_indices,
        nl_edge_idx,
        graph,
        sample_rate,
        &[drain_node],
        gate_node,
    );

    let tree = if source_follower {
        // The source-follower runtime root solves Vs against the source load
        // directly; including the AC-grounded drain load would inflate Rp.
        source_leg.or(drain_leg)?
    } else {
        match (drain_leg, source_leg) {
            (Some(drain), Some(source)) => DynNode::Series(Box::new(drain), Box::new(source)),
            (Some(drain), None) => drain,
            (None, Some(source)) => source,
            (None, None) => return None,
        }
    };

    Some((tree, FetWdfTopology { source_follower }))
}

fn build_fet_ac_ground_leg(
    start_node: NodeId,
    edge_indices: &[usize],
    nl_edge_idx: usize,
    graph: &CircuitGraph,
    sample_rate: f64,
    blocked_nodes: &[NodeId],
    gate_node: Option<NodeId>,
) -> Option<DynNode> {
    let mut visited_edges = std::collections::HashSet::new();
    build_fet_leg_from_node(
        start_node,
        edge_indices,
        nl_edge_idx,
        graph,
        sample_rate,
        blocked_nodes,
        gate_node,
        &mut visited_edges,
    )
}

fn build_fet_leg_from_node(
    node: NodeId,
    edge_indices: &[usize],
    nl_edge_idx: usize,
    graph: &CircuitGraph,
    sample_rate: f64,
    blocked_nodes: &[NodeId],
    gate_node: Option<NodeId>,
    visited_edges: &mut std::collections::HashSet<usize>,
) -> Option<DynNode> {
    if is_fet_ac_ground(node, graph) {
        return None;
    }

    let mut branches = Vec::new();
    for &eidx in edge_indices {
        if eidx == nl_edge_idx || visited_edges.contains(&eidx) {
            continue;
        }
        let edge = &graph.edges[eidx];
        let Some(next_node) = other_node(edge, node) else {
            continue;
        };
        if Some(next_node) == gate_node || blocked_nodes.contains(&next_node) {
            continue;
        }
        if !matches!(
            graph.effective_edge_kind(eidx),
            super::component::EdgeKind::Linear | super::component::EdgeKind::Reactive
        ) {
            continue;
        }

        let comp = &graph.components[edge.comp_idx];
        let leaf = comp.kind.make_leaf(&comp.id, sample_rate)?;
        let mut branch_visited = visited_edges.clone();
        branch_visited.insert(eidx);

        let branch = if is_fet_ac_ground(next_node, graph) {
            leaf
        } else if let Some(rest) = build_fet_leg_from_node(
            next_node,
            edge_indices,
            nl_edge_idx,
            graph,
            sample_rate,
            blocked_nodes,
            gate_node,
            &mut branch_visited,
        ) {
            DynNode::Series(Box::new(leaf), Box::new(rest))
        } else {
            continue;
        };

        branches.push(branch);
    }

    fold_dyn_nodes_parallel(branches)
}

fn fold_dyn_nodes_parallel(mut nodes: Vec<DynNode>) -> Option<DynNode> {
    match nodes.len() {
        0 => None,
        1 => Some(nodes.remove(0)),
        _ => {
            let mut tree = nodes.pop().unwrap();
            while let Some(left) = nodes.pop() {
                tree = DynNode::Parallel(Box::new(left), Box::new(tree));
            }
            Some(tree)
        }
    }
}

fn other_node(edge: &super::graph::GraphEdge, node: NodeId) -> Option<NodeId> {
    if edge.node_a == node {
        Some(edge.node_b)
    } else if edge.node_b == node {
        Some(edge.node_a)
    } else {
        None
    }
}

fn is_fet_ac_ground(node: NodeId, graph: &CircuitGraph) -> bool {
    node == graph.gnd_node
        || node == graph.vcc_node
        || graph.supply_nodes.contains(&node)
        || graph.ac_ground_nodes.contains(&node)
}

fn fet_terminal_reaches_output(
    start_node: NodeId,
    other_fet_terminal: NodeId,
    gate_node: Option<NodeId>,
    graph: &CircuitGraph,
) -> bool {
    let mut seen_nodes = std::collections::HashSet::new();
    let mut stack = vec![start_node];
    seen_nodes.insert(start_node);

    while let Some(node) = stack.pop() {
        if node == graph.out_node {
            return true;
        }
        for (eidx, edge) in graph.edges.iter().enumerate() {
            let Some(next) = other_node(edge, node) else {
                continue;
            };
            if next == other_fet_terminal
                || Some(next) == gate_node
                || is_fet_ac_ground(next, graph)
            {
                continue;
            }
            if !matches!(
                graph.effective_edge_kind(eidx),
                super::component::EdgeKind::Linear | super::component::EdgeKind::Reactive
            ) {
                continue;
            }
            if seen_nodes.insert(next) {
                stack.push(next);
            }
        }
    }

    false
}

fn find_fet_source_probe(
    source_node: NodeId,
    drain_node: NodeId,
    gate_node: Option<NodeId>,
    graph: &CircuitGraph,
) -> Option<String> {
    for (eidx, edge) in graph.edges.iter().enumerate() {
        let Some(other) = other_node(edge, source_node) else {
            continue;
        };
        if other == drain_node || Some(other) == gate_node || !is_fet_ac_ground(other, graph) {
            continue;
        }
        if !matches!(
            graph.effective_edge_kind(eidx),
            super::component::EdgeKind::Linear | super::component::EdgeKind::Reactive
        ) {
            continue;
        }
        let comp = &graph.components[edge.comp_idx];
        if comp.kind.make_leaf(&comp.id, 48_000.0).is_some() {
            return Some(comp.id.clone());
        }
    }
    None
}

/// Build a runnable `WdfStage` from an `SpqrStage`.
///
/// - **PassiveWdf**: VS + DynNode tree + Passthrough root
/// - **NlWdf**: VS + DynNode tree + NL root from Component::classify_nonlinear()
/// - **Rigid**: not yet supported (returns Err — build layer will handle IIR/OpAmpRoot/MNA)
///
/// `supply_voltage` is the circuit's B+ supply (e.g. 250V for tube stages, 9V for pedals).
/// It is used to configure tube roots with the correct `v_max` so the WDF voltage source
/// matches the actual plate supply rail.
pub(super) fn build_spqr_stage(
    stage: SpqrStage,
    graph: &CircuitGraph,
    _sample_rate: f64,
    supply_voltage: f64,
) -> Result<BuiltStage, String> {
    let bias_node_voltages = std::collections::BTreeMap::new();
    build_spqr_stage_with_options(
        stage,
        graph,
        _sample_rate,
        false,
        &[],
        supply_voltage,
        &bias_node_voltages,
    )
}

pub(super) fn build_spqr_stage_with_options(
    stage: SpqrStage,
    graph: &CircuitGraph,
    _sample_rate: f64,
    disable_iir: bool,
    init_hints: &[crate::dsl::InitHint],
    supply_voltage: f64,
    bias_node_voltages: &std::collections::BTreeMap<NodeId, f64>,
) -> Result<BuiltStage, String> {
    match stage {
        SpqrStage::PassiveWdf {
            tree, edge_indices, ..
        } => {
            if let Some(wdf) =
                build_passive_rtype_stage(&edge_indices, graph, _sample_rate, bias_node_voltages)
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
            let (tree, fet_topology) =
                build_fet_amplifier_passive_tree(&edge_indices, nl_edge_idx, graph, _sample_rate)
                    .map(|(tree, topology)| (tree, Some(topology)))
                    .unwrap_or((tree, None));

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
                // Node-voltage seeds key on circuit nodes, not BJT refs — skip
                // them here (they seed reactive ports, not this BjtRoot).
                if let Some(hint) = init_hints.iter().find(|h| {
                    h.device_label == comp.id
                        && !matches!(h.state, crate::dsl::InitState::NodeVoltage { .. })
                }) {
                    let sign = if bjt.is_pnp { -1.0 } else { 1.0 };
                    let vce = match &hint.state {
                        crate::dsl::InitState::Named(state_name) => {
                            bjt_hint_vce(state_name, supply_voltage, bjt.is_pnp)
                        }
                        // Explicit `op { }` seed: warm-start the single-port WDF
                        // BjtRoot at the supplied collector-emitter voltage.
                        crate::dsl::InitState::Explicit { vce, .. } => sign * *vce,
                        // Filtered out above; unreachable.
                        crate::dsl::InitState::NodeVoltage { .. } => 0.0,
                    };
                    bjt.set_initial_prev_v(vce);
                }
            }
            // Set the supply voltage on tube roots so the WDF voltage source (VS =
            // t.v_max()) matches the actual B+ rail.  Without this the triode/pentode
            // would default to 500 V, shifting the plate operating point and gain by
            // ~8 dB versus the correct 250 V operating point.
            match &mut root {
                RootKind::Triode(t) => t.set_v_max(supply_voltage.max(1.0)),
                RootKind::VariMu(t) => t.set_v_max(supply_voltage.max(1.0)),
                RootKind::Pentode(p) => p.set_v_max(supply_voltage.max(1.0)),
                RootKind::Bjt(b) => b.set_v_max(supply_voltage.abs().max(1.0)),
                _ => {}
            }
            // Seed the TriodeRoot with the DC Q-point from load-line analysis.
            //
            // bias::solve_wdf_triode_dc_qpoint (ko5g.3 — the unified solver
            // that replaced this file's duplicate `compute_wdf_triode_dc_qpoint`)
            // runs the shared 1-D load-line relaxation (Vgk = -Ia*Rk,
            // Vpk = VCC - Ia*Rp) using edge_indices to locate R_plate and
            // R_cathode.  NotApplicable when the triode lacks a grid node
            // (strapped triode) or is vari-mu; Undeterminable when the plate/
            // cathode resistors are not found.
            //
            // Two initialization targets:
            //   1. TriodeRoot::set_bias(vgk)  — NR warm-start + K-table centre
            //   2. Cathode bypass cap seed    — eliminates startup transient
            //
            // Without (1) the NR solver starts at vgk_bias=-2.0 V (the default)
            // which is wrong for high-voltage stages (e.g. 250 V 12AX7).
            // Without (2) the cathode cap takes τ=Rk*Ck time-constants to charge;
            // SPICE runs a .op before .tran so it starts at steady state.
            let dc_qpoint = match super::bias::solve_wdf_triode_dc_qpoint(
                &nl_kind,
                &edge_indices,
                graph,
                supply_voltage,
            ) {
                Ok(dc) => Some(dc),
                Err(skip) => {
                    // ko5g.3 warn-not-error: an undeterminable triode keeps the
                    // legacy -2.0 V TriodeRoot default (loudly); the fail-loud
                    // flip is ko5g.8.
                    skip.warn_if_undeterminable(
                        "Triode stage keeps the -2.0 V default Vgk bias.",
                    );
                    None
                }
            };
            // (1) Seed bias
            if let Some(ref dc) = dc_qpoint {
                if let RootKind::Triode(t) = &mut root {
                    t.set_bias(dc.vgk as pedalkernel_rt::Wave);
                }
            }

            // Seed the PentodeRoot with the DC Q-point (ko5g.5 — the FIRST
            // pentode bias solver; every pentode previously rode the −8.0 V
            // `vg1k_bias` default and the model-default `vg2k`, never seeded).
            //
            // bias::solve_wdf_pentode_dc_qpoint solves the self-bias load line
            // (shared-cathode aware, OT-primary-at-DCR plate paths) and
            // resolves the screen divider at DC.  Three initialization
            // targets:
            //   1. PentodeRoot::set_bias(vg1k)   — NR warm-start + K-table centre
            //   2. PentodeRoot::set_vg2k(vg2)    — circuit-true screen voltage
            //   3. Cathode bypass cap seed       — eliminates startup transient
            //
            // Undeterminable topologies (incl. the grounded-cathode FIXED-BIAS
            // guard — the ko5g.2 la2a-V5 safeguard) keep ALL defaults, loudly.
            let pentode_dc = if matches!(root, RootKind::Pentode(_)) {
                match super::bias::solve_wdf_pentode_dc_qpoint(
                    &nl_kind,
                    &edge_indices,
                    graph,
                    supply_voltage,
                ) {
                    Ok(dc) => Some(dc),
                    Err(skip) => {
                        skip.warn_if_undeterminable(
                            "Pentode stage keeps the -8.0 V default Vg1k bias and the \
                             model-default Vg2.",
                        );
                        None
                    }
                }
            } else {
                None
            };
            if let Some(ref dc) = pentode_dc {
                if let RootKind::Pentode(p) = &mut root {
                    p.set_bias(dc.vg1k as pedalkernel_rt::Wave);
                    if let Some(vg2) = dc.vg2k {
                        p.set_vg2k(vg2 as pedalkernel_rt::Wave);
                    }
                }
            }

            // Seed the BjtRoot DC operating point (Q-point).
            //
            // Without this the BjtRoot keeps its default vbe_bias = 0 V, which
            // sits the transistor in cutoff (Ic ≈ 0) so a common-emitter stage
            // produces almost no output (≈1% of the SPICE level).  The base-bias
            // divider (R1 to a rail, R2 to gnd) sets the DC base voltage; the
            // emitter degeneration resistor lowers the actual Vbe.  We seed the
            // root with the forward Vbe operating point so its Newton-Raphson
            // solve and K-table are centred at conduction, exactly mirroring the
            // semantics the blockwise multi-NL path already uses.
            //
            // We only seed when a Q-point is actually solvable (a determinable
            // base divider + emitter resistor).  When it is not — e.g. a
            // single-resistor self-bias circuit, or a topology we cannot resolve —
            // we deliberately LEAVE the existing vbe_bias untouched rather than
            // forcing a nominal default.  Forcing a default there would push such
            // a stage into conduction whose DC operating point cannot be
            // characterized (and may not be DC-blocked downstream), a strictly
            // larger blast radius than the cutoff-amplifier bug this fix targets.
            //
            // bias::solve_wdf_bjt_dc_qpoint (ko5g.4 — the unified solver that
            // replaced this file's duplicate `compute_wdf_bjt_dc_qpoint`) runs
            // the shared base-loop Newton with the legacy stage-set direct-only
            // finder breadth (`BjtFinderFlavor::WdfStageDirect`).
            let bjt_dc = if matches!(root, RootKind::Bjt(_)) {
                match super::bias::solve_wdf_bjt_dc_qpoint(
                    &nl_kind,
                    &edge_indices,
                    graph,
                    bias_node_voltages,
                    supply_voltage,
                ) {
                    Ok(dc) => Some(dc),
                    Err(skip) => {
                        // ko5g.4 warn-not-error: an undeterminable BJT keeps the
                        // legacy leave-at-cutoff fallback (vbe_bias = 0, no Vce
                        // warm-start) — loudly; the fail-loud flip is ko5g.8.
                        skip.warn_if_undeterminable(
                            "BJT stage keeps its cutoff default (vbe_bias = 0 V, no Vce warm-start).",
                        );
                        None
                    }
                }
            } else {
                None
            };
            if let RootKind::Bjt(b) = &mut root {
                if let Some(ref dc) = bjt_dc {
                    b.set_bias(dc.vbe as pedalkernel_rt::Wave);
                    // Warm-start the NR/K-table at the Q-point Vce (unless an
                    // explicit init { } hint already set an asymmetric state for
                    // an oscillator).  This removes the bias-settling startup
                    // transient so the steady-state output is reached immediately,
                    // matching ngspice's `.op`-then-`.tran` behaviour.
                    let has_hint = init_hints.iter().any(|h| h.device_label == comp.id);
                    if !has_hint && dc.vce.is_finite() {
                        b.set_initial_prev_v(dc.vce as pedalkernel_rt::Wave);
                    }
                }
            }

            // Seed FET gate-source DC operating point.  The gate is a high-Z
            // control terminal and is intentionally excluded from the WDF tree
            // above; this puts the nonlinear root at the same DC bias that the
            // omitted gate/source resistor network establishes in SPICE.
            let fet_dc = if matches!(root, RootKind::Jfet(_) | RootKind::Mosfet(_)) {
                compute_wdf_fet_dc_qpoint(&nl_kind, &edge_indices, graph, supply_voltage)
            } else {
                None
            };
            match &mut root {
                RootKind::Jfet(j) => {
                    if let Some(ref dc) = fet_dc {
                        j.set_operating_point(
                            dc.vgs as pedalkernel_rt::Wave,
                            dc.vds as pedalkernel_rt::Wave,
                        );
                    }
                }
                RootKind::Mosfet(m) => {
                    if let Some(ref dc) = fet_dc {
                        m.set_operating_point(
                            dc.vgs as pedalkernel_rt::Wave,
                            dc.vds as pedalkernel_rt::Wave,
                        );
                    }
                }
                _ => {}
            }

            let tree = if fet_topology.is_some() {
                tree
            } else {
                with_voltage_source(tree)
            };
            let oversampler = Oversampler::new(OversamplingFactor::X1);
            let mut wdf_stage = WdfStage::new(tree, root, oversampler);
            if let (Some(topology), Some(dc)) = (fet_topology, fet_dc.as_ref()) {
                wdf_stage.output_bias = if topology.source_follower {
                    dc.source_voltage as pedalkernel_rt::Wave
                } else {
                    dc.drain_voltage as pedalkernel_rt::Wave
                };
                let gate_node = graph.node_names.get(&format!("{}.gate", comp.id)).copied();
                wdf_stage.fet_source_probe =
                    find_fet_source_probe(e.node_b, e.node_a, gate_node, graph);
            }
            if fet_topology.is_some_and(|topology| topology.source_follower)
                && matches!(wdf_stage.root, RootKind::Jfet(_))
            {
                wdf_stage.is_source_follower = true;
            }
            wdf_stage.base_diode_model = base_diode_model;
            // (2) Pre-charge cathode bypass cap
            if let Some(dc) = dc_qpoint {
                if dc.v_cathode > 0.01 {
                    if let super::classify::NonlinearKind::Triode {
                        cathode_node: triode_cathode_node,
                        ..
                    } = &nl_kind
                    {
                        let v_cat = dc.v_cathode as pedalkernel_rt::Wave;
                        for eidx in 0..graph.edges.len() {
                            let e = &graph.edges[eidx];
                            let is_cathode_gnd = (e.node_a == *triode_cathode_node
                                && (e.node_b == graph.gnd_node
                                    || graph.ac_ground_nodes.contains(&e.node_b)))
                                || (e.node_b == *triode_cathode_node
                                    && (e.node_a == graph.gnd_node
                                        || graph.ac_ground_nodes.contains(&e.node_a)));
                            if !is_cathode_gnd {
                                continue;
                            }
                            let comp = &graph.components[e.comp_idx];
                            if comp.kind.capacitance().is_none() {
                                continue;
                            }
                            if let Some(port) =
                                wdf_stage.tree.one_port_runtime_binding_mut(&comp.id)
                            {
                                port.wdf_set_one_port_state(
                                    pedalkernel_rt::boundary_math::OnePortState::CapacitorVoltage(
                                        v_cat,
                                    ),
                                    &mut wdf_stage.runtime_state,
                                );
                            }
                        }
                    }
                }
            }

            // (2a) Pre-charge the pentode cathode bypass cap (ko5g.5) — the
            // exact mirror of the triode block above: the shared/self-bias
            // cathode resistor develops n·Ia·Rk and the bypass cap across it
            // must start charged or the stage spends τ = Rk·Ck settling.
            if let Some(ref dc) = pentode_dc {
                if dc.v_cathode > 0.01 {
                    if let super::classify::NonlinearKind::Pentode {
                        cathode_node: pentode_cathode_node,
                        ..
                    } = &nl_kind
                    {
                        let v_cat = dc.v_cathode as pedalkernel_rt::Wave;
                        for eidx in 0..graph.edges.len() {
                            let e = &graph.edges[eidx];
                            let is_cathode_gnd = (e.node_a == *pentode_cathode_node
                                && (e.node_b == graph.gnd_node
                                    || graph.ac_ground_nodes.contains(&e.node_b)))
                                || (e.node_b == *pentode_cathode_node
                                    && (e.node_a == graph.gnd_node
                                        || graph.ac_ground_nodes.contains(&e.node_a)));
                            if !is_cathode_gnd {
                                continue;
                            }
                            let comp = &graph.components[e.comp_idx];
                            if comp.kind.capacitance().is_none() {
                                continue;
                            }
                            if let Some(port) =
                                wdf_stage.tree.one_port_runtime_binding_mut(&comp.id)
                            {
                                port.wdf_set_one_port_state(
                                    pedalkernel_rt::boundary_math::OnePortState::CapacitorVoltage(
                                        v_cat,
                                    ),
                                    &mut wdf_stage.runtime_state,
                                );
                            }
                        }
                    }
                }
            }

            // (2b) Pre-charge the BJT emitter bypass cap to its DC drop.
            //
            // Mirrors the triode cathode-bypass-cap pre-charge: the emitter
            // resistor RE develops a DC drop (Ie·RE) and the bypass cap (CE)
            // across it charges to that voltage with τ = RE·CE.  Starting it at
            // 0 V produces a large bias-settling transient that drives the
            // collector to the rail for several τ.  Seeding the cap state to the
            // Q-point drop makes the stage start at steady state.
            if let Some(ref dc) = bjt_dc {
                if dc.v_emitter > 0.01 {
                    let emitter_node = match &nl_kind {
                        super::classify::NonlinearKind::BjtNpn { emitter_node, .. }
                        | super::classify::NonlinearKind::BjtPnp { emitter_node, .. } => {
                            Some(*emitter_node)
                        }
                        _ => None,
                    };
                    if let Some(emitter_node) = emitter_node {
                        let v_e = dc.v_emitter as pedalkernel_rt::Wave;
                        for eidx in 0..graph.edges.len() {
                            let e = &graph.edges[eidx];
                            let is_emitter_gnd = (e.node_a == emitter_node
                                && (e.node_b == graph.gnd_node
                                    || graph.ac_ground_nodes.contains(&e.node_b)))
                                || (e.node_b == emitter_node
                                    && (e.node_a == graph.gnd_node
                                        || graph.ac_ground_nodes.contains(&e.node_a)));
                            if !is_emitter_gnd {
                                continue;
                            }
                            let comp = &graph.components[e.comp_idx];
                            if comp.kind.capacitance().is_none() {
                                continue;
                            }
                            if let Some(port) =
                                wdf_stage.tree.one_port_runtime_binding_mut(&comp.id)
                            {
                                port.wdf_set_one_port_state(
                                    pedalkernel_rt::boundary_math::OnePortState::CapacitorVoltage(
                                        v_e,
                                    ),
                                    &mut wdf_stage.runtime_state,
                                );
                            }
                        }
                    }
                }
            }

            // Series-diode rectifier output extraction.
            //
            // For a diode root the stage normally reports the *junction* voltage
            // −(a_root + b_tree)/2, i.e. the voltage across the diode itself.
            // That is correct for the canonical cases the diode root was built
            // for: a shunt clipper (diode to ground — junction voltage IS the
            // output) and an op-amp feedback clipper (diode across the feedback
            // path). But in a *series* rectifier `in -> R1 -> D1 -> RL -> gnd`,
            // the circuit output is the load voltage at the diode's cathode
            // (D1.b / RL.a), NOT the diode junction voltage. Reporting the
            // junction voltage there yields an inverted, non-rectifying signal.
            //
            // Detect the series-rectifier shape: a single-diode whose cathode
            // (output terminal "b") drives exactly one resistor to ground (the
            // load RL), with one resistor between the anode and the source (R1).
            // The output load voltage follows from KVL: V_RL = (Vin - V_diode) *
            // RL/(R1+RL). Record that divider so the runtime computes the load
            // voltage. Shunt clippers (cathode == gnd) leave it unset and keep the
            // junction-voltage behaviour.
            if matches!(nl_kind, super::classify::NonlinearKind::SingleDiode(_)) {
                let is_gnd = |n: super::graph::NodeId| -> bool {
                    n == graph.gnd_node || graph.ac_ground_nodes.contains(&n)
                };
                // Diode terminals: anode = input "a" = e.node_a, cathode = "b".
                let anode = e.node_a;
                let cathode = e.node_b;
                // Total series resistance touching a node (excluding the diode).
                let series_r_at = |node: super::graph::NodeId| -> f64 {
                    edge_indices
                        .iter()
                        .filter(|&&eidx| eidx != nl_edge_idx)
                        .filter_map(|&eidx| {
                            let le = &graph.edges[eidx];
                            if le.node_a == node || le.node_b == node {
                                graph.components[le.comp_idx].kind.resistance()
                            } else {
                                None
                            }
                        })
                        .sum()
                };
                if !is_gnd(cathode) {
                    // Load resistance: cathode → gnd resistors (RL).
                    let r_load: f64 = edge_indices
                        .iter()
                        .filter(|&&eidx| eidx != nl_edge_idx)
                        .filter_map(|&eidx| {
                            let le = &graph.edges[eidx];
                            let touches_cathode = le.node_a == cathode || le.node_b == cathode;
                            let other = if le.node_a == cathode {
                                le.node_b
                            } else {
                                le.node_a
                            };
                            if touches_cathode && is_gnd(other) {
                                graph.components[le.comp_idx].kind.resistance()
                            } else {
                                None
                            }
                        })
                        .sum();
                    // Source-side series resistance feeding the anode (R1).
                    let r_series = series_r_at(anode);
                    if r_load > 0.0 && (r_series + r_load).is_finite() {
                        wdf_stage.series_rectifier_divider = Some(r_load / (r_series + r_load));
                    }
                }
            }

            Ok(BuiltStage::Wdf(wdf_stage))
        }
        SpqrStage::Rigid {
            edge_indices,
            boundary_nodes,
            pendant_trees,
            ..
        } => {
            let all_passive = edge_indices.iter().all(|&eidx| {
                let comp = &graph.components[graph.edges[eidx].comp_idx];
                comp.kind.is_passive()
            });
            if all_passive {
                if let Some(wdf) = build_passive_rtype_stage(
                    &edge_indices,
                    graph,
                    _sample_rate,
                    bias_node_voltages,
                ) {
                    return Ok(BuiltStage::Wdf(wdf));
                }
            }

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

fn build_passive_rtype_stage(
    edge_indices: &[usize],
    graph: &CircuitGraph,
    sample_rate: f64,
    bias_node_voltages: &std::collections::BTreeMap<NodeId, f64>,
) -> Option<WdfStage> {
    let is_ground = |n: NodeId| n == graph.gnd_node || graph.ac_ground_nodes.contains(&n);

    let terminals =
        compute_group_terminals(edge_indices, graph, &vec![graph.in_node, graph.out_node]);
    let mut output_node = terminals
        .iter()
        .copied()
        .find(|&t| t == graph.out_node)
        .or_else(|| {
            terminals
                .iter()
                .copied()
                .find(|&t| !is_ground(t) && t != graph.in_node)
        })?;
    let mut input_node = terminals
        .iter()
        .copied()
        .find(|&t| t == graph.in_node)
        .or_else(|| {
            terminals
                .iter()
                .copied()
                .find(|&t| !is_ground(t) && t != output_node)
        })?;
    // Interior groups (neither terminal is the global in/out) picked their
    // input/output above by terminal ITERATION order — i.e. by NodeId —
    // which flips with incidental node renumbering (e.g. a virtual
    // `EF.in` tap pin unioning into a nearby node shifts union-find roots).
    // An inverted orientation stamps the MNA voltage source on the
    // DOWNSTREAM terminal and extracts at the upstream one, erasing the
    // group's transfer (a JFET-shunt GR divider measured vs=1.0 — flat —
    // in exactly that case). Orient by rail-blocked signal-flow distance
    // from `in`: the upstream terminal drives, the downstream one extracts.
    // Swap only on PROVEN inversion (both distances known and ordered) so
    // unreachable/tied terminals keep the legacy order.
    if output_node != graph.out_node && input_node != graph.in_node {
        let d_in = super::signal_flow::bfs_distances_from_in_node(graph);
        if let (Some(&d_input), Some(&d_output)) = (d_in.get(&input_node), d_in.get(&output_node)) {
            if d_output < d_input {
                core::mem::swap(&mut input_node, &mut output_node);
            }
        }
    }

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

    let transformer_comp_indices: Vec<usize> = {
        let mut seen = std::collections::HashSet::new();
        edge_indices
            .iter()
            .filter_map(|&eidx| {
                let comp = &graph.components[graph.edges[eidx].comp_idx];
                let cfg = comp.kind.transformer_config()?;
                if cfg.has_tertiary() || !seen.insert(graph.edges[eidx].comp_idx) {
                    return None;
                }
                Some(graph.edges[eidx].comp_idx)
            })
            .collect()
    };
    let transformer_internal_base = nodes.len();
    let transformer_internals: std::collections::HashMap<usize, [usize; 4]> =
        transformer_comp_indices
            .iter()
            .enumerate()
            .map(|(i, &comp_idx)| {
                let base = transformer_internal_base + i * 4;
                (comp_idx, [base, base + 1, base + 2, base + 3])
            })
            .collect();
    let mut mna = MnaSystem::new(
        nodes.len() + transformer_comp_indices.len() * 4,
        1 + transformer_comp_indices.len() * 2,
    );
    let vs_node = node_to_mna(input_node, &nodes);
    mna.stamp_voltage_source(vs_node, None, 0);

    let mut children = Vec::new();
    let mut ports = Vec::new();
    let mut variable_resistors = Vec::new();
    let mut seen_pots = std::collections::HashSet::new();
    let mut stamped_transformers = std::collections::HashSet::new();

    let add_dynamic_port = |ports: &mut Vec<WdfPort>,
                            children: &mut Vec<DynNode>,
                            variable_resistors: &mut Vec<MnaVariableResistorBinding>,
                            port: WdfPort,
                            child: DynNode| {
        ports.push(port);
        let child_idx = ports.len() - 1;
        children.insert(child_idx, child);
        for binding in variable_resistors {
            if binding.child_idx >= child_idx {
                binding.child_idx += 1;
            }
        }
    };

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
                variable_resistors.push(MnaVariableResistorBinding {
                    child_idx: children.len(),
                    terminals: MnaPortTerminals::maybe_differential(
                        n1.map(MnaNodeId::new),
                        n2.map(MnaNodeId::new),
                    ),
                    conductance: 1.0 / r,
                });
                children.push(child);
                seen_pots.insert(edge.comp_idx);
            }
            continue;
        }

        if let Some(cfg) = comp.kind.transformer_config() {
            if !cfg.has_tertiary() && stamped_transformers.insert(edge.comp_idx) {
                let Some(&internals) = transformer_internals.get(&edge.comp_idx) else {
                    continue;
                };
                let vsrc_p = 1 + transformer_comp_indices
                    .iter()
                    .position(|&idx| idx == edge.comp_idx)
                    .unwrap()
                    * 2;
                let Some(sec_pos_node) = graph.node_names.get(&format!("{}.c", comp.id)).copied()
                else {
                    continue;
                };
                let Some(sec_neg_node) = graph.node_names.get(&format!("{}.d", comp.id)).copied()
                else {
                    continue;
                };
                let primary_pos_node = graph.node_names.get(&format!("{}.a", comp.id)).copied();
                let primary_neg_node = graph.node_names.get(&format!("{}.b", comp.id)).copied();
                let s1 = node_to_mna(sec_pos_node, &nodes);
                let s2 = node_to_mna(sec_neg_node, &nodes);
                let resolved_cfg = crate::model_lookup::transformer_config_from_dsl(cfg);
                let dc_bias_current = resolved_cfg.dc_bias_current.or_else(|| {
                    match (primary_pos_node, primary_neg_node) {
                        (Some(a), Some(b)) => infer_transformer_dc_bias_current(
                            &resolved_cfg,
                            a,
                            b,
                            bias_node_voltages,
                            graph,
                        ),
                        _ => None,
                    }
                });
                let dynamic = stamp_linear_transformer_skeleton(
                    &mut mna,
                    &comp.id,
                    &resolved_cfg,
                    n1,
                    n2,
                    s1,
                    s2,
                    internals,
                    vsrc_p,
                    sample_rate,
                    dc_bias_current,
                );
                for (port, child) in dynamic {
                    add_dynamic_port(
                        &mut ports,
                        &mut children,
                        &mut variable_resistors,
                        port,
                        child,
                    );
                }
            }
            continue;
        }

        // Gate-modulated JFETs reach this builder as Linear edges (variable
        // resistors). Stamp the current Rds and register the `jfet_vr` child
        // so runtime Vgs modulation (LFO/envelope) can update the G matrix
        // and re-derive scattering — same mechanism as pots. Without this
        // the JFET edge was silently dropped from the MNA (audit gap G2).
        // Photocoupler LDR cells are the same shape of controlled resistance
        // (LED drive instead of Vgs) and were dropped identically (gap G3).
        if comp.kind.is_jfet() || comp.kind.type_tag() == "photocoupler" {
            if let Some(child) = comp.kind.make_leaf(&comp.id, sample_rate) {
                let r = child.port_resistance();
                mna.stamp_resistor(n1, n2, r);
                variable_resistors.push(MnaVariableResistorBinding {
                    child_idx: children.len(),
                    terminals: MnaPortTerminals::maybe_differential(
                        n1.map(MnaNodeId::new),
                        n2.map(MnaNodeId::new),
                    ),
                    conductance: 1.0 / r,
                });
                children.push(child);
            }
            continue;
        }

        if let Some(r) = comp.kind.resistance() {
            mna.stamp_resistor(n1, n2, r);
        } else if comp.kind.capacitance().is_some() || comp.kind.inductance().is_some() {
            let child = comp.kind.make_leaf(&comp.id, sample_rate)?;
            let rp = child.port_resistance();
            let (port_pos, port_neg) = if edge.node_a == input_node {
                (n2, n1)
            } else if edge.node_b == input_node {
                (n1, n2)
            } else {
                (n1, n2)
            };
            ports.push(WdfPort {
                node_pos: port_pos,
                node_neg: port_neg,
                resistance: rp,
            });
            children.insert(ports.len() - 1, child);
            for binding in &mut variable_resistors {
                if binding.child_idx >= ports.len() - 1 {
                    binding.child_idx += 1;
                }
            }
        }
    }

    let transformer_voltage_gain = if stamped_transformers.is_empty() {
        passive_transformer_voltage_gain(input_node, output_node, graph)
    } else {
        None
    };
    let output_mna = node_to_mna(output_node, &nodes);
    let (scattering, vs_injection) = mna.derive_scattering_and_vs_injection(&ports, 0);
    let (extraction_coeffs, mut extraction_vs) =
        mna.derive_extraction_coeffs(&ports, 0, output_mna, None);
    if let Some(gain) = transformer_voltage_gain {
        extraction_vs = gain;
    }

    if scattering.iter().any(|v| !v.is_finite())
        || vs_injection.iter().any(|v| !v.is_finite())
        || extraction_coeffs.iter().any(|v| !v.is_finite())
        || !extraction_vs.is_finite()
    {
        return None;
    }

    let mut child_runtime_states = Vec::with_capacity(children.len());
    for child in &mut children {
        child_runtime_states.push(child.bind_runtime_state());
    }

    // `nodes` maps MNA index → circuit NodeId; needed for secondary port injection.
    let mna_node_map: Vec<usize> = nodes.clone();

    let mut wdf = WdfStage::new(
        DynNode::Resistor(Some("__passive_rtype_dummy".to_string()), 1.0),
        RootKind::PassiveRType {
            scattering,
            vs_injection,
            n_ports: ports.len(),
            children,
            child_runtime_states,
            output_port: 0,
            extraction_coeffs,
            extraction_vs,
            extraction_output_pos: output_mna,
            extraction_output_neg: None,
            recompute_mna: Some(mna),
            recompute_ports: Some(ports),
            variable_resistors,
            needs_recompute: false,
            interp_table: None,
            mna_node_map,
            port_vs_injections: Vec::new(),
        },
        Oversampler::new(OversamplingFactor::X1),
    );
    wdf.output_probe = None;

    // F13b: detect a parallel-branch convergence mixer and attach a superposition
    // summation. NARROW by construction — only resistive groups whose boundary
    // includes >=2 distinct op-amp-output nodes AND the global `out` qualify, so
    // every single-source passive group is completely unaffected (zero-diff).
    if let Some(cs) = try_build_convergence_sum(edge_indices, graph) {
        wdf.convergence = Some(cs);
    }

    Some(wdf)
}

/// Detect and build a parallel-branch convergence summation (F13b).
///
/// Predicate (intentionally narrow): the group's boundary nodes must include
/// **two or more distinct op-amp output nodes** (driving ideal voltage sources)
/// AND the group must touch the global `out` node. This is exactly the
/// "several active branches mixed to the output" topology (the 808 snare's two
/// bridged-T resonators into the blend pot). Single-source groups, bias
/// dividers, tone stacks, and ordinary serial passives all fail the predicate
/// and are untouched.
///
/// When it matches, the gains are solved by DC superposition (see
/// [`ConvergenceSum::recompute_gains`]).
fn try_build_convergence_sum(
    edge_indices: &[usize],
    graph: &CircuitGraph,
) -> Option<pedalkernel_rt::convergence::ConvergenceSum> {
    use pedalkernel_rt::convergence::{ConvergenceBranch, ConvergenceSum, FixedBranch, PotBranch};

    let is_ground = |n: NodeId| n == graph.gnd_node || graph.ac_ground_nodes.contains(&n);

    let terminals = compute_group_terminals(edge_indices, graph, &[graph.in_node, graph.out_node]);

    // Must reach the global output.
    if !terminals.contains(&graph.out_node) {
        return None;
    }

    // Driver nodes = boundary terminals that are an op-amp `out` node of an
    // op-amp NOT contained in this group (a genuine upstream voltage source).
    let group_comps: std::collections::HashSet<usize> = edge_indices
        .iter()
        .map(|&e| graph.edges[e].comp_idx)
        .collect();
    let mut driver_nodes: Vec<NodeId> = Vec::new();
    for &t in &terminals {
        if t == graph.out_node || is_ground(t) {
            continue;
        }
        let is_upstream_opamp_out = graph
            .nullor_pins
            .iter()
            .any(|p| p.out_node == t && !group_comps.contains(&p.comp_idx));
        if is_upstream_opamp_out && !driver_nodes.contains(&t) {
            driver_nodes.push(t);
        }
    }
    if driver_nodes.len() < 2 {
        return None;
    }

    // Build the resistive network over this group's local nodes (excl. ground).
    let mut local: Vec<NodeId> = Vec::new();
    let mut local_index = |node: NodeId, local: &mut Vec<NodeId>| -> Option<usize> {
        if is_ground(node) {
            return None;
        }
        if let Some(p) = local.iter().position(|&n| n == node) {
            Some(p)
        } else {
            local.push(node);
            Some(local.len() - 1)
        }
    };

    let mut fixed_branches: Vec<FixedBranch> = Vec::new();
    let mut pot_branches: Vec<PotBranch> = Vec::new();
    for &eidx in edge_indices {
        let edge = &graph.edges[eidx];
        let comp = &graph.components[edge.comp_idx];
        let na = local_index(edge.node_a, &mut local);
        let nb = local_index(edge.node_b, &mut local);
        if na.is_none() && nb.is_none() {
            continue;
        }
        if comp.kind.is_pot() {
            if let Some(pot) = comp
                .kind
                .as_any()
                .downcast_ref::<super::components::Potentiometer>()
            {
                let is_aw =
                    pot_edge_is_aw_half_for_build(graph, &comp.id, edge.node_a, edge.node_b);
                pot_branches.push(PotBranch {
                    comp_id: comp.id.clone(),
                    na,
                    nb,
                    max_r: pot.max_r,
                    taper: pot.taper,
                    is_aw,
                    position: 0.5,
                });
            }
        } else if let Some(r) = comp.kind.resistance() {
            if r > 0.0 {
                fixed_branches.push(FixedBranch {
                    na,
                    nb,
                    conductance: 1.0 / r,
                });
            }
        }
        // Reactive/NL elements are intentionally ignored: convergence groups are
        // resistive by the predicate above (a group containing an NL device would
        // not have been routed through `build_passive_rtype_stage`).
    }

    let branches: Vec<ConvergenceBranch> = driver_nodes
        .iter()
        .filter_map(|&node| {
            local
                .iter()
                .position(|&n| n == node)
                .map(|source_local| ConvergenceBranch {
                    source_node_id: node,
                    source_local,
                    gain: 0.0,
                })
        })
        .collect();
    if branches.len() < 2 {
        return None;
    }

    let output_local = local.iter().position(|&n| n == graph.out_node);

    let mut cs = ConvergenceSum {
        branches,
        output_node_id: graph.out_node,
        num_nodes: local.len(),
        output_local,
        fixed_branches,
        pot_branches,
    };
    cs.recompute_gains();
    Some(cs)
}

/// Edge-guard (pedalkernel-ffkl): what a builder does when a circuit edge
/// falls through every stamping branch — the failure mode that silently
/// dropped the LA-2A `T_out` primary from the triode-context MNA.
///
/// * `error`: the compile FAILS with the offending component list.
///   A circuit must not compile with a component missing and no trace.
/// * `warn`: loud eprintln diagnostic, compile proceeds.
/// * `off`: legacy silent behavior.
///
/// DEFAULTS are per-layer (each call site passes its own), overridable for
/// both layers at once via `PK_EDGE_GUARD=error|warn|off`:
/// * builder-level (`rigid::general` — an edge handed to a builder was not
///   stamped): **error**. Corpus-measured clean; any trip is a formation bug.
/// * assembly-level (a graph edge landed in NO stage and NO consumption
///   mechanism): **error** (pedalkernel-x5ac, promoted in the
///   pedalkernel-y9hz batch). The one real corpus drop that blocked
///   promotion — `try_build_blockwise` returning `Some(empty)` and
///   vaporizing the PNP fuzz family's BJT core — is FIXED (empty build
///   declines to the monolithic path), and the full lib corpus runs
///   guard-quiet (2 trips -> 0, measured). Any trip is now a compile error:
///   a circuit must not compile with a component missing and no trace.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(in crate::compiler) enum EdgeGuardMode {
    Off,
    Warn,
    Error,
}

pub(in crate::compiler) fn edge_guard_mode(default_mode: EdgeGuardMode) -> EdgeGuardMode {
    match std::env::var("PK_EDGE_GUARD").as_deref() {
        Ok("off") => EdgeGuardMode::Off,
        Ok("warn") => EdgeGuardMode::Warn,
        Ok("error") => EdgeGuardMode::Error,
        _ => default_mode,
    }
}

/// Report edges that a builder received but did not stamp, port, or
/// explicitly consume. Returns `Err` in [`EdgeGuardMode::Error`].
pub(in crate::compiler) fn report_dropped_edges(
    context: &str,
    dropped: &[usize],
    graph: &super::graph::CircuitGraph,
    default_mode: EdgeGuardMode,
) -> Result<(), String> {
    if dropped.is_empty() {
        return Ok(());
    }
    let mode = edge_guard_mode(default_mode);
    if mode == EdgeGuardMode::Off {
        return Ok(());
    }
    let mut descriptions: Vec<String> = dropped
        .iter()
        .map(|&eidx| {
            let e = &graph.edges[eidx];
            let comp = &graph.components[e.comp_idx];
            format!(
                "{} ({}, edge {eidx}, kind {:?})",
                comp.id,
                comp.kind.type_tag(),
                graph.effective_edge_kind(eidx)
            )
        })
        .collect();
    descriptions.sort();
    descriptions.dedup();
    let msg = format!(
        "edge guard [{context}]: {} edge(s) fell through the build without a \
         stamp, port, or explicit consumption — the compiled circuit would be \
         missing these components with no trace: {}. \
         (Set PK_EDGE_GUARD=warn to compile anyway, PK_EDGE_GUARD=off to \
         silence.)",
        descriptions.len(),
        descriptions.join(", ")
    );
    match mode {
        EdgeGuardMode::Error => Err(msg),
        _ => {
            eprintln!("[pedalkernel] WARNING: {msg}");
            Ok(())
        }
    }
}

pub(in crate::compiler) fn stamp_linear_transformer_skeleton(
    mna: &mut MnaSystem,
    comp_id: &str,
    cfg: &TransformerConfig,
    p_pos: Option<usize>,
    p_neg: Option<usize>,
    s_pos: Option<usize>,
    s_neg: Option<usize>,
    internals: [usize; 4],
    vsrc_p: usize,
    sample_rate: f64,
    dc_bias_current: Option<f64>,
) -> Vec<(WdfPort, DynNode)> {
    const SHORT_R: f64 = 1.0e-6;
    const MIN_L: f64 = 1.0e-12;
    const MIN_C: f64 = 1.0e-15;

    let [p_series, p_core, s_core, s_series] = internals;
    let n = cfg.turns_ratio;
    let l_primary = cfg.primary_inductance.max(MIN_L);
    let l_secondary = l_primary / (n * n);
    let k = cfg.coupling.clamp(0.0, 1.0);
    let l_leak_p = cfg.primary_leakage.unwrap_or(l_primary * (1.0 - k));
    let l_leak_s = cfg.secondary_leakage.unwrap_or(l_secondary * (1.0 - k));
    let l_mag = cfg
        .magnetizing_inductance
        .unwrap_or(l_primary * k.max(1.0e-6));
    let ja_core = transformer_ja_core_model(cfg);
    let dc_bias_current = dc_bias_current.unwrap_or(0.0);

    fn add_inductor_or_short(
        mna: &mut MnaSystem,
        dynamic: &mut Vec<(WdfPort, DynNode)>,
        comp_id: &str,
        name: &str,
        a: Option<usize>,
        b: Option<usize>,
        l: f64,
        sample_rate: f64,
    ) {
        const SHORT_R: f64 = 1.0e-6;
        const MIN_L: f64 = 1.0e-12;
        if l.is_finite() && l > MIN_L {
            let rp = 2.0 * sample_rate * l;
            dynamic.push((
                WdfPort {
                    node_pos: a,
                    node_neg: b,
                    resistance: rp,
                },
                DynNode::Inductor(Some(format!("{comp_id}.{name}")), l, rp),
            ));
        } else {
            mna.stamp_resistor(a, b, SHORT_R);
        }
    }

    fn add_magnetizing_branch(
        mna: &mut MnaSystem,
        dynamic: &mut Vec<(WdfPort, DynNode)>,
        comp_id: &str,
        a: Option<usize>,
        b: Option<usize>,
        l: f64,
        sample_rate: f64,
        ja_core: Option<JaCoreModel>,
        dc_bias_current: f64,
    ) {
        const SHORT_R: f64 = 1.0e-6;
        const MIN_L: f64 = 1.0e-12;
        if !(l.is_finite() && l > MIN_L) {
            mna.stamp_resistor(a, b, SHORT_R);
            return;
        }

        let rp = 2.0 * sample_rate * l;
        let node = if let Some(model) = ja_core {
            DynNode::JaMagnetizingWithDcBias(
                Some(format!("{comp_id}.Lm")),
                model,
                sample_rate,
                rp,
                dc_bias_current as pedalkernel_rt::Wave,
            )
        } else {
            DynNode::Inductor(Some(format!("{comp_id}.Lm")), l, rp)
        };
        dynamic.push((
            WdfPort {
                node_pos: a,
                node_neg: b,
                resistance: rp,
            },
            node,
        ));
    }

    let mut dynamic = Vec::new();

    mna.stamp_resistor(p_pos, Some(p_series), cfg.primary_dcr.max(SHORT_R));
    add_inductor_or_short(
        mna,
        &mut dynamic,
        comp_id,
        "Llp",
        Some(p_series),
        Some(p_core),
        l_leak_p,
        sample_rate,
    );

    add_inductor_or_short(
        mna,
        &mut dynamic,
        comp_id,
        "Lls",
        Some(s_core),
        Some(s_series),
        l_leak_s,
        sample_rate,
    );
    mna.stamp_resistor(Some(s_series), s_pos, cfg.secondary_dcr.max(SHORT_R));

    add_magnetizing_branch(
        mna,
        &mut dynamic,
        comp_id,
        Some(p_core),
        p_neg,
        l_mag,
        sample_rate,
        ja_core,
        dc_bias_current,
    );
    if let Some(rc) = cfg
        .core_loss_resistance
        .filter(|r| r.is_finite() && *r > 0.0)
    {
        mna.stamp_resistor(Some(p_core), p_neg, rc);
    }

    if cfg.capacitance.is_finite() && cfg.capacitance > MIN_C {
        let rp = 1.0 / (2.0 * sample_rate * cfg.capacitance);
        dynamic.push((
            WdfPort {
                node_pos: Some(p_core),
                node_neg: Some(s_core),
                resistance: rp,
            },
            DynNode::Capacitor(Some(format!("{comp_id}.Cp")), cfg.capacitance, rp),
        ));
    }

    mna.stamp_transformer(
        Some(p_core),
        p_neg,
        Some(s_core),
        s_neg,
        vsrc_p,
        vsrc_p + 1,
        n,
    );

    dynamic
}

fn transformer_ja_core_model(cfg: &TransformerConfig) -> Option<JaCoreModel> {
    let model = JaCoreModel {
        ms: cfg.ja_ms? as pedalkernel_rt::Wave,
        a: cfg.ja_a? as pedalkernel_rt::Wave,
        alpha: cfg.ja_alpha? as pedalkernel_rt::Wave,
        k: cfg.ja_k? as pedalkernel_rt::Wave,
        c: cfg.ja_c? as pedalkernel_rt::Wave,
        n_turns: cfg.core_primary_turns? as pedalkernel_rt::Wave,
        area: cfg.core_area? as pedalkernel_rt::Wave,
        path_len: cfg.core_path_length? as pedalkernel_rt::Wave,
        gap: cfg.core_gap.unwrap_or(0.0) as pedalkernel_rt::Wave,
    };
    model.is_complete().then_some(model)
}

fn infer_transformer_dc_bias_current(
    cfg: &TransformerConfig,
    primary_pos: NodeId,
    primary_neg: NodeId,
    bias_node_voltages: &std::collections::BTreeMap<NodeId, f64>,
    graph: &CircuitGraph,
) -> Option<f64> {
    let r_primary = cfg.primary_dcr;
    if !(r_primary.is_finite() && r_primary > 1.0e-9) {
        return None;
    }

    // Moved to bias.rs (ko5g.4) — the single legacy-arm-order node-voltage read.
    let vp = super::bias::node_dc_voltage(primary_pos, bias_node_voltages, graph)?;
    let vn = super::bias::node_dc_voltage(primary_neg, bias_node_voltages, graph)?;
    let current = (vp - vn) / r_primary;
    (current.is_finite() && current.abs() > 1.0e-12).then_some(current)
}

fn passive_transformer_voltage_gain(
    input_node: NodeId,
    output_node: NodeId,
    graph: &CircuitGraph,
) -> Option<f64> {
    let input = graph.transformer_info.get(&input_node)?;
    let output = graph.transformer_info.get(&output_node)?;
    if input.comp_idx != output.comp_idx || input.is_secondary == output.is_secondary {
        return None;
    }

    let n = input.turns_ratio;
    if !(n.is_finite() && n > 0.0) {
        return None;
    }

    if input.is_secondary && !output.is_secondary {
        Some(n)
    } else {
        Some(1.0 / n)
    }
}

fn group_has_signal_transformer_boundary(
    group: &super::signal_flow::FlowGroup,
    graph: &CircuitGraph,
    input_node: NodeId,
    output_node: NodeId,
) -> bool {
    if passive_transformer_voltage_gain(input_node, output_node, graph).is_none() {
        return false;
    }

    group.all_edges().iter().any(|&eidx| {
        graph.components[graph.edges[eidx].comp_idx]
            .kind
            .is_transformer()
    })
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

// NOTE: stage-ordering helper. Walks ALL edges (including through rails),
// unlike `signal_flow::bfs_distances_from_in_node`, which measures
// signal-path distance (rail-blocked) for the F10 claiming bound.
fn bfs_dist_from_in_node(
    target: super::graph::NodeId,
    graph: &super::graph::CircuitGraph,
) -> Option<usize> {
    use std::collections::VecDeque;
    let mut visited: hashbrown::HashMap<super::graph::NodeId, usize> = hashbrown::HashMap::new();
    let mut queue: VecDeque<super::graph::NodeId> = VecDeque::new();
    visited.insert(graph.in_node, 0);
    queue.push_back(graph.in_node);
    while let Some(node) = queue.pop_front() {
        let dist = visited[&node];
        if node == target {
            return Some(dist);
        }
        for e in &graph.edges {
            let (touches, other) = if e.node_a == node {
                (true, e.node_b)
            } else if e.node_b == node {
                (true, e.node_a)
            } else {
                (false, node)
            };
            if touches && !visited.contains_key(&other) {
                visited.insert(other, dist + 1);
                queue.push_back(other);
            }
        }
    }
    None
}

/// Collect all edges that belong to a triode's local subcircuit.
///
/// Starting from the triode's plate, cathode, and grid nodes, performs a BFS
/// through the graph collecting all passive (Linear/Reactive) edges. The BFS
/// stops at any node that is:
/// - `graph.gnd_node` or a supply node (VCC, global rail)
/// - `graph.in_node` or `graph.out_node` (global signal boundary)
///
/// This lets a standalone common-cathode triode (which gets no passive claiming
/// from signal_flow because it has no feedback path) gather its plate load,
/// cathode bias resistor, and cathode bypass capacitor into the same MNA stage.
///
/// Returns the collected edge indices (including the triode's own NL edge).
fn collect_triode_context_edges(
    triode_nl_edge_idx: usize,
    graph: &super::graph::CircuitGraph,
    all_graph_edges: &[usize],
) -> Vec<usize> {
    use super::component::EdgeKind;

    let e = &graph.edges[triode_nl_edge_idx];
    let comp = &graph.components[e.comp_idx];

    // Collect the triode's terminal nodes from its component pins.
    let mut frontier_nodes: std::collections::HashSet<NodeId> = std::collections::HashSet::new();
    for pin in ["plate", "cathode", "grid"] {
        let key = format!("{}.{pin}", comp.id);
        if let Some(&node) = graph.node_names.get(&key) {
            frontier_nodes.insert(node);
        }
    }
    // Also include the NL edge endpoints directly.
    frontier_nodes.insert(e.node_a);
    frontier_nodes.insert(e.node_b);

    // Boundary nodes: stop BFS here (don't pull in coupling caps / grid leak
    // that connects to the circuit's global input or output).
    let is_global = |n: NodeId| -> bool {
        n == graph.gnd_node
            || graph.supply_nodes.contains(&n)
            || n == graph.in_node
            || n == graph.out_node
    };

    // F10 upstream bound: never collect toward a non-global node strictly
    // closer to `in_node` than the triode's grid pin. Such a node belongs to
    // the UPSTREAM network (e.g. a passive EQ feeding this makeup stage);
    // collecting it would swallow the network into the triode's MNA stage,
    // flattening its response and freezing its pots. Rail-terminated bias
    // edges (vcc→plate, cathode→gnd, grid→gnd) are handled by the is_global
    // branch and stay collected. Unreachable nodes (disconnected
    // subcircuits) have no distance and are never bounded.
    let d_in = super::signal_flow::bfs_distances_from_in_node(graph);
    let grid_dist: Option<usize> = graph
        .node_names
        .get(&format!("{}.grid", comp.id))
        .and_then(|node| d_in.get(node))
        .copied();
    let upstream_of_grid = |n: NodeId| -> bool {
        match (d_in.get(&n), grid_dist) {
            (Some(&d), Some(d_grid)) => d < d_grid,
            _ => false,
        }
    };

    let mut collected_edges: std::collections::HashSet<usize> = std::collections::HashSet::new();
    collected_edges.insert(triode_nl_edge_idx);

    let mut visited_nodes: std::collections::HashSet<NodeId> = frontier_nodes.clone();
    let mut queue: std::collections::VecDeque<NodeId> = frontier_nodes.into_iter().collect();

    while let Some(node) = queue.pop_front() {
        if is_global(node) {
            continue;
        }
        for &eidx in all_graph_edges {
            if collected_edges.contains(&eidx) {
                continue;
            }
            let edge = &graph.edges[eidx];
            let kind = graph.effective_edge_kind(eidx);
            // Only follow passive (Linear/Reactive) edges — don't cross into
            // other NL devices.
            if kind == EdgeKind::Nonlinear {
                continue;
            }
            let (touches, other) = if edge.node_a == node {
                (true, edge.node_b)
            } else if edge.node_b == node {
                (true, edge.node_a)
            } else {
                (false, node)
            };
            if !touches {
                continue;
            }

            let comp = &graph.components[edge.comp_idx];

            // Coupling capacitors (reactive elements connecting to in/out ports)
            // are NOT part of the triode's local bias network. Exclude them.
            // R_plate (plate→VCC) and R_cathode/C_cathode (cathode→GND) ARE
            // included because they connect triode pins to supply rails.
            if is_global(other) {
                // other is a global node (VCC, GND, in_node, out_node).
                // Include the edge only if it is a resistor (bias/load resistor
                // connecting triode pin to a supply) OR if it is a reactive
                // element whose global end is VCC/GND (bypass capacitor to GND
                // is fine; coupling cap to in/out is not).
                let other_is_signal_port = other == graph.in_node || other == graph.out_node;
                if other_is_signal_port {
                    // This is a coupling element (C_in: in→grid, C_out: plate→out).
                    // Do NOT include — it would drag in/out signal domain into the MNA.
                    continue;
                }
                // other is VCC or GND — include (plate load, cathode bias, bypass cap)
                collected_edges.insert(eidx);
                continue;
            }

            // other is a non-global internal node.
            // F10: refuse to collect toward the upstream network (see the
            // upstream_of_grid doc above).
            if upstream_of_grid(other) {
                continue;
            }
            // Exclude coupling caps that lead toward the signal ports: if the
            // component is reactive and the other node eventually only connects
            // to in_node / out_node without passing through any resistor to a
            // supply, it's a coupling cap. Use a simple one-hop check: if the
            // only connections at 'other' (besides this edge) lead to in_node
            // or out_node, it's an isolated node that is the output of a
            // coupling cap chain — don't include.
            let other_only_connects_to_signal_port = comp.kind.capacitance().is_some()
                && graph.edges.iter().enumerate().all(|(other_eidx, other_e)| {
                    if other_eidx == eidx {
                        return true; // skip self
                    }
                    if other_e.node_a != other && other_e.node_b != other {
                        return true; // doesn't touch other node
                    }
                    let far = if other_e.node_a == other {
                        other_e.node_b
                    } else {
                        other_e.node_a
                    };
                    far == graph.in_node || far == graph.out_node || far == graph.gnd_node
                });
            if other_only_connects_to_signal_port {
                continue;
            }

            collected_edges.insert(eidx);
            if !visited_nodes.contains(&other) {
                visited_nodes.insert(other);
                queue.push_back(other);
            }
        }
    }

    collected_edges.into_iter().collect()
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

/// Merge purely-linear passive groups (series input resistors) into adjacent
/// active groups so the active group's MNA captures the source impedance.
///
/// Motivation: Sallen-Key and other active filter topologies have a series
/// input resistor R1 in its own FlowGroup (no feedback, no reactive elements).
/// Without merging, the active group's MNA treats its first node (junction J)
/// as an ideal voltage source, losing R1's contribution and producing a flat
/// frequency response instead of the correct filter shape.
///
/// Merge condition (all must hold):
/// 1. Source group is passive (no active edges).
/// 2. Source group's edges are ALL linear (no reactive, NL, or VCVS).
/// 3. One endpoint of every source edge connects the circuit `in_node` or the
///    output node of a preceding active group (i.e. an already-committed
///    signal source) to a node that is also an internal node of exactly one
///    active group.
/// 4. The target active group contains a VCVS (needs source impedance for
///    correct filter computation).
///
/// Only source groups with a single linear resistor touching the active
/// group's node-set qualify — broader merges are too risky.
pub(super) fn merge_input_coupling_into_active_groups(
    groups: &mut Vec<super::signal_flow::FlowGroup>,
    graph: &super::graph::CircuitGraph,
    cut_edges: &super::boundary_rules::DelayedCutSet,
) {
    // Build node sets for each active group (include all edges — active,
    // feedback, and pendant — so we can find the coupling junction J even
    // when the VCVS is a self-loop at node_out and the filter passives are
    // in pendant_edges rather than active_edges).
    let active_group_nodes: Vec<std::collections::HashSet<super::graph::NodeId>> = groups
        .iter()
        .map(|g| {
            let mut nodes = std::collections::HashSet::new();
            for &eidx in &g.all_edges() {
                let e = &graph.edges[eidx];
                nodes.insert(e.node_a);
                nodes.insert(e.node_b);
            }
            nodes
        })
        .collect();

    // Identify which active groups contain a VCVS AND are purely linear
    // (no NL/ControlledConductance). We must not merge input resistors into
    // General-class groups (those with pots, JFETs, etc.) — the General
    // builder handles source impedance differently and merging pendant edges
    // into it disrupts signal flow routing.
    let has_vcvs: Vec<bool> = groups
        .iter()
        .map(|g| {
            let all = g.all_edges();
            let has_v = g
                .active_edges
                .iter()
                .any(|&eidx| graph.effective_edge_kind(eidx) == super::component::EdgeKind::Vcvs);
            let is_linear = all.iter().all(|&eidx| {
                matches!(
                    graph.effective_edge_kind(eidx),
                    super::component::EdgeKind::Linear
                        | super::component::EdgeKind::Reactive
                        | super::component::EdgeKind::Vcvs
                )
            });
            has_v && is_linear
        })
        .collect();

    let mut merge_into: Vec<Option<usize>> = vec![None; groups.len()];

    for (source_idx, source) in groups.iter().enumerate() {
        // Must be a passive group.
        if !source.active_edges.is_empty() {
            continue;
        }

        let edges = source.all_edges();
        if edges.is_empty() {
            continue;
        }

        // All edges must be strictly linear (resistors/pots only).
        let all_linear = edges
            .iter()
            .all(|&eidx| graph.effective_edge_kind(eidx) == super::component::EdgeKind::Linear);
        if !all_linear {
            continue;
        }

        // For each edge, one endpoint must be `in_node` (or another "source"
        // node) and the other must be an internal node of exactly one VCVS
        // active group.
        let is_source_node = |n: super::graph::NodeId| -> bool {
            n == graph.in_node
                || n == graph.gnd_node
                || graph.supply_nodes.contains(&n)
                || graph.ac_ground_nodes.contains(&n)
        };

        let mut target: Option<usize> = None;
        for &eidx in &edges {
            // Phase 2a: a broker-cut tap-mouth edge can't be a merge bridge.
            if cut_edges.cuts.contains_key(&eidx) {
                continue;
            }
            let e = &graph.edges[eidx];
            // One side must be a "source" node.
            let (src_side, other_side) = if is_source_node(e.node_a) {
                (e.node_a, e.node_b)
            } else if is_source_node(e.node_b) {
                (e.node_b, e.node_a)
            } else {
                continue;
            };
            let _ = src_side;

            // The other side must be in exactly one VCVS active group's
            // node set (not the output node — we want an internal coupling node).
            let candidate = active_group_nodes
                .iter()
                .enumerate()
                .filter(|(gi, nodes)| has_vcvs[*gi] && nodes.contains(&other_side))
                .map(|(gi, _)| gi)
                .collect::<Vec<_>>();
            if candidate.len() == 1 {
                let t = candidate[0];
                match target {
                    None => target = Some(t),
                    Some(prev) if prev == t => {} // Same target
                    Some(_) => {
                        // Ambiguous — don't merge.
                        target = None;
                        break;
                    }
                }
            }
        }

        if let Some(t) = target {
            // Don't create a cycle (source must not be the target).
            if t != source_idx {
                merge_into[source_idx] = Some(t);
            }
        }
    }

    // Apply merges: move source edges into target's pendant_edges.
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
                "  [compile] merged input-coupling group {source_idx} into active group {target_idx}: {names:?}"
            );
        }
        for eidx in edges {
            if !groups[target_idx].pendant_edges.contains(&eidx) {
                groups[target_idx].pendant_edges.push(eidx);
            }
        }
    }

    // Remove merged-away groups.
    let mut idx = 0;
    groups.retain(|_| {
        let keep = merge_into.get(idx).and_then(|target| *target).is_none();
        idx += 1;
        keep
    });
}

pub(super) fn merge_cross_reactive_groups_into_active_groups(
    groups: &mut Vec<super::signal_flow::FlowGroup>,
    graph: &super::graph::CircuitGraph,
    cut_edges: &super::boundary_rules::DelayedCutSet,
) {
    let rails = super::signal_flow::rail_nodes(graph);
    let is_boundary_node = |node: super::graph::NodeId| -> bool {
        rails.contains(&node) || node == graph.in_node || node == graph.out_node
    };

    let mut active_terminal_nodes: Vec<std::collections::HashSet<super::graph::NodeId>> =
        Vec::with_capacity(groups.len());
    let mut active_reactive_nodes: Vec<std::collections::HashSet<super::graph::NodeId>> =
        Vec::with_capacity(groups.len());
    for group in groups.iter() {
        let mut terminal_nodes = std::collections::HashSet::new();
        let mut reactive_nodes = std::collections::HashSet::new();
        if !can_absorb_cross_reactive_passives(group, graph) {
            active_terminal_nodes.push(terminal_nodes);
            active_reactive_nodes.push(reactive_nodes);
            continue;
        }
        for &eidx in &group.active_edges {
            let edge = &graph.edges[eidx];
            terminal_nodes.insert(edge.node_a);
            terminal_nodes.insert(edge.node_b);
        }
        for eidx in group.all_edges() {
            if graph.effective_edge_kind(eidx) != super::component::EdgeKind::Reactive {
                continue;
            }
            let edge = &graph.edges[eidx];
            if !is_boundary_node(edge.node_a) {
                reactive_nodes.insert(edge.node_a);
            }
            if !is_boundary_node(edge.node_b) {
                reactive_nodes.insert(edge.node_b);
            }
        }
        active_terminal_nodes.push(terminal_nodes);
        active_reactive_nodes.push(reactive_nodes);
    }

    let mut merge_into: Vec<Option<usize>> = vec![None; groups.len()];
    for (source_idx, group) in groups.iter().enumerate() {
        if !group.active_edges.is_empty() {
            continue;
        }
        let reactive_edges: Vec<usize> = group
            .all_edges()
            .into_iter()
            // Phase 2a: a broker-cut tap-mouth edge can't be a merge bridge.
            .filter(|eidx| !cut_edges.cuts.contains_key(eidx))
            .filter(|&eidx| graph.effective_edge_kind(eidx) == super::component::EdgeKind::Reactive)
            .collect();
        if reactive_edges.is_empty() {
            continue;
        }

        let mut reactive_group_nodes = std::collections::HashSet::new();
        for &eidx in &reactive_edges {
            let edge = &graph.edges[eidx];
            if !is_boundary_node(edge.node_a) {
                reactive_group_nodes.insert(edge.node_a);
            }
            if !is_boundary_node(edge.node_b) {
                reactive_group_nodes.insert(edge.node_b);
            }
        }

        for (target_idx, terminal_nodes) in active_terminal_nodes.iter().enumerate() {
            if target_idx == source_idx || groups[target_idx].active_edges.is_empty() {
                continue;
            }
            let bridges_active_terminals = reactive_edges.iter().any(|&eidx| {
                let edge = &graph.edges[eidx];
                terminal_nodes.contains(&edge.node_a) && terminal_nodes.contains(&edge.node_b)
            });
            let shares_reactive_internal_node = reactive_group_nodes
                .iter()
                .any(|node| active_reactive_nodes[target_idx].contains(node));
            if bridges_active_terminals || shares_reactive_internal_node {
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

fn can_absorb_cross_reactive_passives(
    group: &super::signal_flow::FlowGroup,
    graph: &super::graph::CircuitGraph,
) -> bool {
    if group.active_edges.is_empty() {
        return false;
    }
    let rails = super::signal_flow::rail_nodes(graph);
    group.active_edges.iter().any(|&eidx| {
        let edge = &graph.edges[eidx];
        let comp = &graph.components[edge.comp_idx];
        !(comp.kind.is_diode_family()
            && (rails.contains(&edge.node_a) || rails.contains(&edge.node_b)))
    })
}

/// Phase 2a: true if `group` is the de-fused DELAYED detector group — it holds a
/// DelayedSense control electrode (a node in `detector_seeds`) AND, after the
/// broker tap-mouth cut, has NO edge touching the global `in`/`out` nodes (no
/// non-cut path to the forward chain). Such a group must compute (state/metering)
/// but NOT overwrite the forward serial signal, so it is marked `bypass_serial`.
fn is_delayed_detector_group(
    group: &super::signal_flow::FlowGroup,
    graph: &super::graph::CircuitGraph,
    detector_seeds: &std::collections::HashSet<super::graph::NodeId>,
) -> bool {
    if detector_seeds.is_empty() {
        return false;
    }
    let mut holds_seed = false;
    let mut touches_io = false;
    for &eidx in group.all_edges().iter() {
        let e = &graph.edges[eidx];
        for n in [e.node_a, e.node_b] {
            if detector_seeds.contains(&n) {
                holds_seed = true;
            }
            if n == graph.in_node || n == graph.out_node {
                touches_io = true;
            }
        }
    }
    holds_seed && !touches_io
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

    if !(has_transistor_nl && !touches_output && !has_reactive && !has_vcvs) {
        return false;
    }

    // pedalkernel-y9hz GHOST-DROP FIX (the pedalkernel-ffkl guard finding):
    // "doesn't touch `out` directly" is NOT "off the audio path" — the
    // fuzz-face family's BJT core reaches `out` THROUGH its output coupling
    // cap (a cut edge in a different group), so this classifier consumed the
    // whole audio core as a "modulator" and the pedal compiled as a unity
    // passthrough. A group that signal FLOWS THROUGH (reachable from `in`,
    // device outputs reach `out` over the directed signal graph) is an
    // audio-path core and must compile as real stages. True modulators
    // (envelope followers, LFO cores) fail the directed through-path test —
    // their device outputs dead-end at control electrodes.
    if super::signal_flow::group_is_on_forward_audio_path(graph, &group.all_edges()) {
        return false;
    }

    true
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

    // Synthesize pair roots from two antiparallel ground-clip edges. Otherwise
    // two opposite zeners collapse to one polarity and clip asymmetrically.
    let (root, base_diode_model) = if nl_kinds.len() >= 2 {
        let mut pair_dt = None;
        let mut zener_voltage = None;
        for i in 0..nl_kinds.len() {
            for j in (i + 1)..nl_kinds.len() {
                match (&nl_kinds[i], &nl_kinds[j]) {
                    (
                        super::classify::NonlinearKind::SingleDiode(dt_a),
                        super::classify::NonlinearKind::SingleDiode(_),
                    ) => {
                        pair_dt = Some(*dt_a);
                        break;
                    }
                    (
                        super::classify::NonlinearKind::Zener { voltage },
                        super::classify::NonlinearKind::Zener { .. },
                    ) => {
                        zener_voltage = Some(*voltage);
                        break;
                    }
                    _ => {}
                }
            }
            if pair_dt.is_some() || zener_voltage.is_some() {
                break;
            }
        }
        if let Some(voltage) = zener_voltage {
            let model = crate::elements::ZenerModel::new(voltage);
            (
                super::stage::RootKind::ZenerPair(crate::elements::ZenerPairRoot::new(model)),
                None,
            )
        } else if let Some(dt) = pair_dt {
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

/// Reorder `v` in place so that the new element `i` is the old element
/// `perm[i]`. `perm` must be a permutation of `0..v.len()`.
fn apply_permutation<T>(v: &mut Vec<T>, perm: &[usize]) {
    debug_assert_eq!(v.len(), perm.len());
    let mut taken: Vec<Option<T>> = v.drain(..).map(Some).collect();
    for &i in perm {
        v.push(taken[i].take().expect("permutation index reused"));
    }
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

    // Defect B (stage ordering): the base in->node distance is now a
    // RAIL-BLOCKED, signal-DIRECTED traversal. The old `bfs_distances` was
    // undirected and never blocked B+/gnd/ac-ground rails, so a plate-/
    // collector-load resistor tied to B+ let the walk reach a tube's downstream
    // plate node in a few hops THROUGH the supply rail — giving load nodes tiny
    // distances and scrambling serial stage order. Crossing active devices only
    // input->output and dead-ending rails fixes the order. The `out`-side
    // distance keeps the old undirected walk: it only ranks active-output->output
    // proximity for the feedback ordering special case, where reachability
    // (not signal direction) is what matters.
    let mut node_dist: HashMap<NodeId, usize> = HashMap::new();
    node_dist.extend(super::signal_flow::directed_signal_distances_from_in(graph));
    let out_dist = bfs_distances(graph.out_node, graph);
    let span = graph.edges.len() + graph.components.len() + groups.len() + 1;

    // GAP 4c (pedalkernel-0lsv) — MAX-band ordering retry. The base metric above
    // (`directed_signal_distances_from_in`) does NOT cross an op-amp control pin
    // (`pos`), so a non-inverting IC is a directed dead-end: a downstream output
    // FOLLOWER (uberdrive/lgsm Q2 emitter follower, fed IC2.out -> C7 -> volume
    // divider -> Q2.base) gets `min_dist == usize::MAX` and lands in the flat MAX
    // band, so it is never serially chained after the tone stage and runs with a
    // serial input of 0.0. The control-crossing metric
    // `boundary_reachable_distances_from_in` DOES reach it. We consult it ONLY as
    // a per-group RETRY for groups already stuck at MAX — the GLOBAL ordering
    // metric (`node_dist`) that screamer/sd1/muff depend on is untouched, so their
    // ordering is byte-identical.
    let boundary_dist: HashMap<NodeId, usize> =
        super::signal_flow::boundary_reachable_distances_from_in(graph);

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

    // Control-crossing distance for a group stuck at `usize::MAX` under the
    // directed metric. Finite only when a boundary node of the group is reachable
    // once op-amp control pins are crossed (uberdrive/lgsm followers). Returns
    // `None` for a group with no such node — keeping it in the flat MAX band.
    let boundary_retry_dist = |group: &super::signal_flow::FlowGroup| -> Option<usize> {
        let mut best = usize::MAX;
        for &eidx in group.all_edges().iter() {
            let e = &graph.edges[eidx];
            if let Some(&d) = boundary_dist.get(&e.node_a) {
                best = best.min(d);
            }
            if let Some(&d) = boundary_dist.get(&e.node_b) {
                best = best.min(d);
            }
        }
        (best != usize::MAX).then_some(best)
    };

    let mut distances: Vec<usize> = groups
        .iter()
        .enumerate()
        .map(|(gi, group)| {
            let min_dist = min_dists[gi];
            if min_dist == usize::MAX {
                // GAP 4c retry: an output follower behind a non-inverting op-amp
                // is a directed dead-end under the base metric but reachable once
                // control pins are crossed. Place it at the MAX-band base plus its
                // control-crossing distance (same band form as the touches-output
                // sink below) so it orders AFTER the reachable stages that feed it
                // and among MAX-band peers by real distance. Falls back to the flat
                // MAX band for groups the control-crossing metric still can't reach.
                if let Some(bdist) = boundary_retry_dist(group) {
                    return groups.len() * span + bdist;
                }
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

    // Collect all nodes touched by this group's edges. BTreeSet, not HashSet:
    // the iteration below fixes the terminal order, and downstream input-node
    // selection and MNA source stamping depend on that order — it must be
    // deterministic across compiles and processes.
    let mut group_nodes: std::collections::BTreeSet<NodeId> = std::collections::BTreeSet::new();
    for &eidx in group_edges {
        let e = &graph.edges[eidx];
        group_nodes.insert(e.node_a);
        group_nodes.insert(e.node_b);
    }
    // Sort to a deterministic iteration order before building terminals.
    // HashSet iteration is non-deterministic (random seed per HashMap instance).
    // The terminal ordering directly feeds spqr_decompose(), so any variation
    // here produces different SPQR trees and different compiled topologies.
    let mut group_nodes_sorted: Vec<NodeId> = group_nodes.into_iter().collect();
    group_nodes_sorted.sort_unstable();

    // Terminals = nodes in this group that are also global terminals
    // or that connect to edges NOT in this group (boundary nodes).
    let group_edge_set: HashSet<usize> = group_edges.iter().copied().collect();
    let mut terminals: Vec<NodeId> = Vec::new();

    for &node in &group_nodes_sorted {
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

// NOTE (ko5g.3): the duplicate `compute_wdf_triode_dc_qpoint` that lived here
// was deleted — the single-port WDF triode call-site now rides
// `super::bias::solve_wdf_triode_dc_qpoint` (same load-line core as the
// grouped MNA path, legacy `TriodeFinderFlavor::DirectOnly` finder breadth).

struct FetDcQpoint {
    vgs: f64,
    vds: f64,
    drain_voltage: f64,
    source_voltage: f64,
}

fn compute_wdf_fet_dc_qpoint(
    nl_kind: &NonlinearKind,
    edge_indices: &[usize],
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Option<FetDcQpoint> {
    let (gate_node, drain_node, source_node) = match nl_kind {
        NonlinearKind::Jfet { .. } | NonlinearKind::Mosfet { .. } => {
            let nl_edge_idx = edge_indices.iter().copied().find(|&eidx| {
                matches!(
                    graph.effective_edge_kind(eidx),
                    super::component::EdgeKind::Nonlinear
                ) && {
                    let comp = &graph.components[graph.edges[eidx].comp_idx];
                    comp.kind.is_jfet() || comp.kind.is_mosfet()
                }
            })?;
            let edge = &graph.edges[nl_edge_idx];
            let comp = &graph.components[edge.comp_idx];
            let gate = graph
                .node_names
                .get(&format!("{}.gate", comp.id))
                .copied()?;
            (gate, edge.node_a, edge.node_b)
        }
        _ => return None,
    };

    let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
    let gate_voltage = dc_thevenin_voltage(gate_node, &all_edges, graph, supply_voltage)?;
    let (drain_rail_voltage, drain_resistance) =
        dc_single_rail_resistance(drain_node, &all_edges, graph, supply_voltage)?;
    let (source_rail_voltage, source_resistance) =
        dc_single_rail_resistance(source_node, &all_edges, graph, supply_voltage)?;

    let model_current = |vgs: f64, vds: f64| -> f64 {
        match nl_kind {
            NonlinearKind::Jfet {
                model_name,
                is_n_channel,
            } => {
                let model = super::helpers::jfet_model(model_name, *is_n_channel);
                let mut root = pedalkernel_rt::elements::nonlinear::JfetRoot::new(model);
                root.set_vgs(vgs as pedalkernel_rt::Wave);
                root.drain_current(vds as pedalkernel_rt::Wave)
            }
            NonlinearKind::Mosfet {
                mosfet_type,
                is_n_channel,
            } => {
                let model = super::helpers::mosfet_model(*mosfet_type, *is_n_channel);
                model.ids(vgs as pedalkernel_rt::Wave, vds as pedalkernel_rt::Wave)
            }
            _ => 0.0,
        }
    };

    let point_for_ids = |ids: f64| -> (f64, f64, f64, f64, f64) {
        let drain_voltage = drain_rail_voltage - ids * drain_resistance;
        let source_voltage = source_rail_voltage + ids * source_resistance;
        let vgs = gate_voltage - source_voltage;
        let vds = drain_voltage - source_voltage;
        (
            vgs,
            vds,
            drain_voltage,
            source_voltage,
            model_current(vgs, vds),
        )
    };

    let residual = |ids: f64| -> f64 {
        let (_vgs, _vds, _vd, _vs, device_ids) = point_for_ids(ids);
        device_ids - ids
    };

    let resistance_sum = (drain_resistance + source_resistance).max(1.0);
    let search = (supply_voltage.abs().max(1.0) / resistance_sum * 4.0).max(1e-6);
    let mut best_ids = None;
    let mut best_abs = f64::INFINITY;
    let mut prev_i = -search;
    let mut prev_f = residual(prev_i);

    for i in 1..=480 {
        let ids = -search + (2.0 * search) * (i as f64 / 480.0);
        let f = residual(ids);
        if f.is_finite() && f.abs() < best_abs {
            best_abs = f.abs();
            best_ids = Some(ids);
        }
        if prev_f.is_finite() && f.is_finite() && prev_f.signum() != f.signum() {
            let mut lo = prev_i;
            let mut hi = ids;
            let mut flo = prev_f;
            for _ in 0..80 {
                let mid = 0.5 * (lo + hi);
                let fmid = residual(mid);
                if !fmid.is_finite() {
                    break;
                }
                if fmid.abs() < 1e-10 {
                    let (vgs, vds, drain_voltage, source_voltage, _ids) = point_for_ids(mid);
                    return Some(FetDcQpoint {
                        vgs,
                        vds,
                        drain_voltage,
                        source_voltage,
                    });
                }
                if flo.signum() == fmid.signum() {
                    lo = mid;
                    flo = fmid;
                } else {
                    hi = mid;
                }
            }
            let ids = 0.5 * (lo + hi);
            let (vgs, vds, drain_voltage, source_voltage, _ids) = point_for_ids(ids);
            return Some(FetDcQpoint {
                vgs,
                vds,
                drain_voltage,
                source_voltage,
            });
        }
        prev_i = ids;
        prev_f = f;
    }

    best_ids.filter(|_| best_abs.is_finite()).map(|ids| {
        let (vgs, vds, drain_voltage, source_voltage, _ids) = point_for_ids(ids);
        FetDcQpoint {
            vgs,
            vds,
            drain_voltage,
            source_voltage,
        }
    })
}

fn dc_thevenin_voltage(
    node: NodeId,
    edge_indices: &[usize],
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Option<f64> {
    let mut conductance_sum = 0.0;
    let mut weighted_voltage_sum = 0.0;
    for &eidx in edge_indices {
        if graph.effective_edge_kind(eidx) != super::component::EdgeKind::Linear {
            continue;
        }
        let edge = &graph.edges[eidx];
        let Some(other) = other_node(edge, node) else {
            continue;
        };
        let Some(rail_voltage) = dc_rail_voltage(other, graph, supply_voltage) else {
            continue;
        };
        let Some(r) = graph.components[edge.comp_idx].kind.resistance() else {
            continue;
        };
        if r <= 0.0 || !r.is_finite() {
            continue;
        }
        let g = 1.0 / r;
        conductance_sum += g;
        weighted_voltage_sum += rail_voltage * g;
    }
    (conductance_sum > 0.0).then_some(weighted_voltage_sum / conductance_sum)
}

fn dc_single_rail_resistance(
    node: NodeId,
    edge_indices: &[usize],
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Option<(f64, f64)> {
    edge_indices.iter().find_map(|&eidx| {
        if graph.effective_edge_kind(eidx) != super::component::EdgeKind::Linear {
            return None;
        }
        let edge = &graph.edges[eidx];
        let other = other_node(edge, node)?;
        let rail_voltage = dc_rail_voltage(other, graph, supply_voltage)?;
        let r = graph.components[edge.comp_idx].kind.resistance()?;
        (r > 0.0 && r.is_finite()).then_some((rail_voltage, r))
    })
}

fn dc_rail_voltage(node: NodeId, graph: &CircuitGraph, supply_voltage: f64) -> Option<f64> {
    // Delegates to THE shared rail resolver (pedalkernel-0stg): named rails
    // now resolve to their ACTUAL declared voltage (graph.supply_voltages)
    // instead of blanket `supply_voltage`. Single-supply pedals are
    // unchanged (their one rail's voltage IS supply_voltage).
    super::bias::rail_dc_voltage(node, graph, supply_voltage)
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
