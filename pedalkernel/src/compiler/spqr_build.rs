//! SPQR stage builder: converts `SpqrStage` descriptors into runnable WDF stages,
//! and provides the `compile_via_spqr()` entry point for the full pipeline.
//!
//! Separation of concerns:
//! - `spqr.rs`: graph decomposition + classification (topology + component semantics)
//! - `rigid/`: StageStats + RigidOptimization decision rules + rigid stage builders
//! - `spqr_build.rs`: stage construction + full pipeline entry point

use super::build::create_root;
use super::compiled::{CompiledPedal, StageRef};
use super::dyn_node::DynNode;
use super::graph::{CircuitGraph, NodeId};
use super::rigid::build_rigid;
use super::spqr::{spqr_decompose, spqr_to_stages, SpqrStage};
use super::compiled::RailSaturation;
use super::stage::{RootKind, WdfStage};
use super::wdf_leaf::WdfVoltageSource;
use crate::dsl::PedalDef;
use crate::oversampling::{Oversampler, OversamplingFactor};

// ═══════════════════════════════════════════════════════════════════════════
// Full pipeline entry point
// ═══════════════════════════════════════════════════════════════════════════

/// Compile a `.pedal` circuit via the SPQR pipeline.
///
/// Full flow: PedalDef → CircuitGraph → SPQR decompose → classify →
/// build stages → CompiledPedal.
///
/// Returns `Err` if any stage can't be built (unsupported topology).
pub fn compile_via_spqr(
    pedal: &PedalDef,
    sample_rate: f64,
) -> Result<CompiledPedal, String> {
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

    // Decompose → classify → build
    let terminals = vec![graph.in_node, graph.out_node];
    let spqr_tree = spqr_decompose(&all_edges, &terminals, &graph, graph.gnd_node);
    let spqr_stages = spqr_to_stages(&spqr_tree, &graph, sample_rate);

    let mut wdf_stages: Vec<WdfStage> = Vec::new();
    for (i, stage) in spqr_stages.into_iter().enumerate() {
        let mut wdf = build_spqr_stage(stage, &graph, sample_rate)
            .map_err(|e| format!("Stage {i}: {e}"))?;
        wdf.signal_flow_distance = i;
        wdf_stages.push(wdf);
    }

    // Build stage ordering (all WDF for now, in SPQR traversal order)
    let stage_order: Vec<StageRef> = (0..wdf_stages.len())
        .map(StageRef::Wdf)
        .collect();

    let supply_voltage = pedal.supplies.first().map_or(9.0, |s| s.config.voltage);

    Ok(CompiledPedal {
        stages: wdf_stages,
        push_pull_stages: Vec::new(),
        multi_nl_stages: Vec::new(),
        pre_gain: 1.0,
        output_gain: 1.0,
        rail_saturation: RailSaturation::None,
        rail_sat_oversampler: Oversampler::new(OversamplingFactor::X1),
        sample_rate,
        controls: Vec::new(),
        gain_range: (0.0, 1.0),
        supply_voltage,
        lfos: Vec::new(),
        envelopes: Vec::new(),
        slew_limiters: Vec::new(),
        bbds: Vec::new(),
        delay_lines: Vec::new(),
        vcos: Vec::new(),
        vcas: Vec::new(),
        thermal: None,
        tolerance_seed: 0,
        oversampling: OversamplingFactor::X1,
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
        stage_order,
        node_signals: Vec::new(),
        triggers: Vec::new(),
        bbd_wet_mix: 0.5,
        bbd_mix_pot_id: None,
        midi_trigger_map: std::collections::HashMap::new(),
        original_passive_values: std::collections::HashMap::new(),
    })
}

// ═══════════════════════════════════════════════════════════════════════════
// Stage construction helpers
// ═══════════════════════════════════════════════════════════════════════════

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
) -> Result<WdfStage, String> {
    match stage {
        SpqrStage::PassiveWdf { tree, .. } => {
            let tree = with_voltage_source(tree);
            let oversampler = Oversampler::new(OversamplingFactor::X1);
            Ok(WdfStage::new(tree, RootKind::Passthrough, oversampler))
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
            Ok(wdf_stage)
        }
        SpqrStage::Rigid {
            edge_indices,
            boundary_nodes,
            pendant_trees,
            ..
        } => build_rigid(edge_indices, boundary_nodes, pendant_trees, graph, _sample_rate),
    }
}

