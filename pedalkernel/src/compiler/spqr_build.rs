//! SPQR stage builder: converts `SpqrStage` descriptors into runnable WDF stages,
//! and provides the `compile_via_spqr()` entry point for the full pipeline.
//!
//! Separation of concerns:
//! - `spqr.rs`: graph decomposition + classification (topology + component semantics)
//! - `rigid_build.rs`: StageStats + RigidOptimization decision rules
//! - `spqr_build.rs`: stage construction + full pipeline entry point

use super::build::create_root;
use super::compiled::{CompiledPedal, StageRef};
use super::dyn_node::DynNode;
use super::graph::{CircuitGraph, NodeId};
use super::rigid_build::{classify_rigid, RigidOptimization, StageStats};
use super::spqr::{spqr_decompose, spqr_to_stages, SpqrStage};
use super::compiled::RailSaturation;
use super::stage::{RootKind, WdfStage};
use super::wdf_leaf::WdfVoltageSource;
use crate::dsl::PedalDef;
use crate::elements::{OpAmpModel, OpAmpRoot};
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

// ═══════════════════════════════════════════════════════════════════════════
// Rigid stage building
// ═══════════════════════════════════════════════════════════════════════════

/// Build a runnable stage from a Rigid SpqrStage.
///
/// Uses `StageStats` + `classify_rigid()` to select the cheapest strategy,
/// then constructs the appropriate stage type.
fn build_rigid(
    edge_indices: Vec<usize>,
    _boundary_nodes: Vec<NodeId>,
    pendant_trees: Vec<(DynNode, NodeId)>,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<WdfStage, String> {
    let stats = StageStats::from_edges(&edge_indices, graph);
    let optimization = classify_rigid(&stats, graph);

    match optimization {
        RigidOptimization::OpAmpRoot { inverting } => {
            build_opamp_root(&edge_indices, &pendant_trees, inverting, graph, sample_rate)
        }
        RigidOptimization::Iir => {
            Err("IIR rigid stages not yet implemented".to_string())
        }
        RigidOptimization::StateSpace => {
            Err("StateSpace rigid stages not yet implemented".to_string())
        }
        RigidOptimization::General => {
            // Common case: VCVS + NL (TS/RAT/Klon clipping stages).
            // OpAmpRoot handles gain+GBW+slew, NL root handles clipping.
            if stats.vcvs_count == 1 && stats.nl_count > 0 {
                build_opamp_nl_feedback(
                    &edge_indices,
                    &pendant_trees,
                    &stats,
                    graph,
                    sample_rate,
                )
            } else {
                Err(format!(
                    "General MNA not yet implemented (vcvs={}, nl={}, linear={}, reactive={})",
                    stats.vcvs_count, stats.nl_count, stats.linear_count, stats.reactive_count
                ))
            }
        }
    }
}

/// Shared op-amp extraction: finds VCVS edge, computes Rf, Ri, gain.
struct OpAmpConfig {
    model: OpAmpModel,
    rf: f64,
    ri: f64,
    gain: f64,
    inverting: bool,
}

/// Extract op-amp configuration from a Rigid stage's edges + pendants.
///
/// Queries Component::op_amp_type() for the model, Component::resistance()
/// for Rf (linear edges in R-node), and pendant port_resistance() for Ri.
fn extract_opamp_config(
    edge_indices: &[usize],
    pendant_trees: &[(DynNode, NodeId)],
    inverting: bool,
    graph: &CircuitGraph,
) -> Result<OpAmpConfig, String> {
    use super::component::EdgeKind;

    // Find the VCVS edge → get OpAmpModel
    let vcvs_edge_idx = edge_indices
        .iter()
        .find(|&&eidx| graph.effective_edge_kind(eidx) == EdgeKind::Vcvs)
        .ok_or("No VCVS edge found")?;

    let vcvs_comp = &graph.components[graph.edges[*vcvs_edge_idx].comp_idx];
    let op_type = vcvs_comp
        .kind
        .op_amp_type()
        .ok_or("VCVS component has no op_amp_type")?;
    let model = OpAmpModel::from_opamp_type(&op_type);

    // Sum feedback resistance (Rf): linear edges in the R-node
    let rf: f64 = edge_indices
        .iter()
        .filter(|&&eidx| graph.effective_edge_kind(eidx) == EdgeKind::Linear)
        .filter_map(|&eidx| graph.components[graph.edges[eidx].comp_idx].kind.resistance())
        .sum();

    // Input resistance (Ri): from pendant tree port resistance
    let ri: f64 = if !pendant_trees.is_empty() {
        pendant_trees.iter().map(|(tree, _)| tree.port_resistance()).sum()
    } else {
        f64::INFINITY // No pendant → unity gain
    };

    let gain = if ri.is_infinite() {
        1.0
    } else if inverting {
        rf / ri
    } else {
        1.0 + rf / ri
    };

    Ok(OpAmpConfig { model, rf, ri, gain, inverting })
}

/// Create an OpAmpRoot from config, with sample rate and supply voltage set.
fn make_opamp_root(config: &OpAmpConfig, sample_rate: f64) -> OpAmpRoot {
    let supply_voltage: f64 = 9.0; // TODO: propagate from PedalDef
    let mut root = if config.inverting {
        OpAmpRoot::new_inverting(config.model, config.gain)
    } else {
        OpAmpRoot::new_non_inverting(config.model, config.gain)
    };
    root.set_sample_rate(sample_rate);
    root.set_v_max((supply_voltage / 2.0 - 1.5).max(0.5));
    root
}

/// Build an OpAmpRoot stage (no NL elements — pure gain + GBW + slew).
fn build_opamp_root(
    edge_indices: &[usize],
    pendant_trees: &[(DynNode, NodeId)],
    inverting: bool,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<WdfStage, String> {
    let config = extract_opamp_config(edge_indices, pendant_trees, inverting, graph)?;
    if config.rf <= 0.0 {
        return Err("OpAmpRoot: no feedback resistance found".to_string());
    }
    let root = make_opamp_root(&config, sample_rate);

    // Tree: VS + pendant Ri
    let tree = if let Some((pendant, _)) = pendant_trees.first() {
        with_voltage_source(pendant.clone())
    } else {
        DynNode::Leaf(Box::new(WdfVoltageSource {
            voltage: 0.0,
            rp: 1.0,
            is_cathode_bias: false,
        }))
    };

    let oversampler = Oversampler::new(OversamplingFactor::X1);
    Ok(WdfStage::new(tree, RootKind::OpAmp(root), oversampler))
}

/// Build a stage with op-amp gain driving a nonlinear root (TS/RAT/Klon pattern).
///
/// The OpAmpRoot pre-amplifies the input (gain + GBW + slew), then the
/// NL root (diode) clips the amplified signal. This is the `feedback_opamp`
/// pattern from the existing pipeline.
fn build_opamp_nl_feedback(
    edge_indices: &[usize],
    pendant_trees: &[(DynNode, NodeId)],
    stats: &StageStats,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<WdfStage, String> {
    use super::component::EdgeKind;

    // Extract op-amp config (gain from Rf/Ri)
    let inverting = super::rigid_build::is_inverting_topology(stats, graph);
    let config = extract_opamp_config(edge_indices, pendant_trees, inverting, graph)?;
    let opamp = make_opamp_root(&config, sample_rate);

    // Find first NL edge → build root
    let nl_edge_idx = edge_indices
        .iter()
        .find(|&&eidx| {
            let ek = graph.effective_edge_kind(eidx);
            ek == EdgeKind::Nonlinear
        })
        .ok_or("General stage has no NL edge")?;

    let e = &graph.edges[*nl_edge_idx];
    let comp = &graph.components[e.comp_idx];
    let (nl_kind, _) = comp
        .kind
        .classify_nonlinear(&comp.id, e.node_a, e.node_b, graph.gnd_node, &graph.node_names)
        .ok_or_else(|| format!("NL edge {} ({}) didn't classify", nl_edge_idx, comp.id))?;

    let (root, base_diode_model) = create_root(&nl_kind, false);

    // Tree: VS + pendant Ri (same as OpAmpRoot path)
    let tree = if let Some((pendant, _)) = pendant_trees.first() {
        with_voltage_source(pendant.clone())
    } else {
        DynNode::Leaf(Box::new(WdfVoltageSource {
            voltage: 0.0,
            rp: 1.0,
            is_cathode_bias: false,
        }))
    };

    let oversampler = Oversampler::new(OversamplingFactor::X2);
    let mut stage = WdfStage::new(tree, root, oversampler);
    stage.feedback_opamp = Some(opamp);
    stage.base_diode_model = base_diode_model;
    Ok(stage)
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;
    use crate::compiler::spqr::{spqr_decompose, spqr_to_stages, SpqrStage};

    fn make_graph_all_edges(
        pedal_src: &str,
    ) -> (CircuitGraph, Vec<usize>) {
        let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse failed");
        let graph = CircuitGraph::from_pedal(&pedal);
        let active_set: std::collections::HashSet<usize> =
            graph.active_edge_indices.iter().copied().collect();
        let all_edges: Vec<usize> = (0..graph.edges.len())
            .filter(|i| !active_set.contains(i))
            .collect();
        (graph, all_edges)
    }

    fn make_graph_passive(
        pedal_src: &str,
    ) -> (CircuitGraph, Vec<usize>) {
        let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse failed");
        let graph = CircuitGraph::from_pedal(&pedal);
        let active_set: std::collections::HashSet<usize> =
            graph.active_edge_indices.iter().copied().collect();
        let passive_edges: Vec<usize> = (0..graph.edges.len())
            .filter(|i| !active_set.contains(i))
            .filter(|&i| graph.components[graph.edges[i].comp_idx].kind.is_passive())
            .collect();
        (graph, passive_edges)
    }

    #[test]
    fn spqr_diode_clipper_produces_audio() {
        let (graph, edges) = make_graph_all_edges(r#"
            pedal "test" { supply 9V
                components { R1: resistor(4.7k)  D1: diode(silicon) }
                nets { in -> R1.a  R1.b -> D1.a  D1.b -> gnd }
                controls {}
            }"#);
        let spqr = spqr_decompose(
            &edges,
            &[graph.in_node, graph.out_node],
            &graph,
            graph.gnd_node,
        );
        let spqr_stages = spqr_to_stages(&spqr, &graph, 48000.0);

        assert_eq!(spqr_stages.len(), 1);
        let mut stage = build_spqr_stage(
            spqr_stages.into_iter().next().unwrap(),
            &graph,
            48000.0,
        )
        .expect("Should build NlWdf stage");

        // DC test: 5V input should clip to ~0.6V (silicon diode forward voltage)
        let dc_out = stage.process(5.0);
        assert!(
            dc_out < 1.5,
            "5V DC should clip to <1.5V, got {dc_out:.4}V"
        );
        assert!(dc_out > 0.1, "5V DC should produce output, got {dc_out:.4}V");

        // Negative DC: single diode doesn't clip reverse bias → passes through
        let neg_out = stage.process(-5.0);
        assert!(
            neg_out.abs() > 2.0,
            "Reverse bias should pass through, got {neg_out:.4}V"
        );

        // Sine wave: positive peaks clipped, negative peaks pass through
        let mut pos_peak = 0.0f64;
        let mut neg_peak = 0.0f64;
        for i in 0..960 {
            let input =
                5.0 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin();
            let output = stage.process(input);
            pos_peak = pos_peak.max(output);
            neg_peak = neg_peak.min(output);
        }

        // SingleDiode clips forward (positive) direction only
        assert!(
            pos_peak < 1.5,
            "Forward bias should clip: pos_peak={pos_peak:.3}V"
        );
        assert!(
            neg_peak < -1.0,
            "Reverse bias should pass: neg_peak={neg_peak:.3}V"
        );
    }

    #[test]
    fn spqr_passive_rc_produces_audio() {
        let (graph, edges) = make_graph_passive(r#"
            pedal "test" { supply 9V
                components { R1: resistor(10k)  C1: cap(100n) }
                nets { in -> R1.a  R1.b -> C1.a  C1.b -> gnd }
                controls {}
            }"#);
        let spqr = spqr_decompose(
            &edges,
            &[graph.in_node, graph.out_node],
            &graph,
            graph.gnd_node,
        );
        let spqr_stages = spqr_to_stages(&spqr, &graph, 48000.0);

        assert_eq!(spqr_stages.len(), 1);
        let mut stage = build_spqr_stage(
            spqr_stages.into_iter().next().unwrap(),
            &graph,
            48000.0,
        )
        .expect("Should build PassiveWdf stage");

        // Process step input — capacitor should charge (lowpass)
        let mut output = 0.0;
        for _ in 0..480 {
            output = stage.process(1.0);
        }

        // After 10ms with RC = 1ms, cap should be mostly charged
        assert!(output.abs() > 0.001, "RC lowpass should pass DC, got {output:.6}");
    }

    #[test]
    fn spqr_inverting_opamp_gain() {
        // Inverting amp: R1=10k, Rf=100k → gain = -10
        let (graph, edges) = make_graph_all_edges(r#"
            pedal "test" { supply 9V
                components {
                    R1: resistor(10k)
                    Rf: resistor(100k)
                    U1: opamp(tl072)
                }
                nets {
                    in -> R1.a
                    R1.b -> U1.neg
                    Rf.a -> U1.neg
                    Rf.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> out
                }
                controls {}
            }"#);
        let spqr = spqr_decompose(
            &edges,
            &[graph.in_node, graph.out_node],
            &graph,
            graph.gnd_node,
        );
        let spqr_stages = spqr_to_stages(&spqr, &graph, 48000.0);

        assert_eq!(spqr_stages.len(), 1);
        let mut stage = build_spqr_stage(
            spqr_stages.into_iter().next().unwrap(),
            &graph,
            48000.0,
        )
        .expect("Should build OpAmpRoot stage");

        // Small signal: 0.1V input → should get ~1.0V output (gain=10, inverted)
        // Let the stage settle for a few samples
        for _ in 0..10 {
            stage.process(0.1);
        }
        let output = stage.process(0.1);
        let gain_measured = output.abs() / 0.1;

        assert!(
            gain_measured > 5.0,
            "Inverting gain should be ~10, got {gain_measured:.2}"
        );
        assert!(
            gain_measured < 15.0,
            "Inverting gain should be ~10, got {gain_measured:.2}"
        );
    }

    #[test]
    fn compile_via_spqr_diode_clipper_end_to_end() {
        use crate::PedalProcessor;

        let pedal = crate::dsl::parse_pedal_file(r#"
            pedal "test_clipper" { supply 9V
                components { R1: resistor(4.7k)  D1: diode(silicon) }
                nets { in -> R1.a  R1.b -> D1.a  D1.b -> gnd }
                controls {}
            }"#)
        .expect("parse");

        let mut compiled = compile_via_spqr(&pedal, 48000.0)
            .expect("Should compile via SPQR");

        // Process through PedalProcessor trait
        let dc_out = compiled.process(5.0);
        assert!(
            dc_out.abs() < 2.0,
            "Diode should clip 5V input, got {dc_out:.4}"
        );

        // Process sine
        let mut peak = 0.0f64;
        for i in 0..480 {
            let input =
                1.0 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin();
            let output = compiled.process(input);
            peak = peak.max(output.abs());
        }
        assert!(peak > 0.01, "Should produce output: {peak:.6}");
    }

    #[test]
    fn compile_via_spqr_inverting_opamp_end_to_end() {
        use crate::PedalProcessor;

        let pedal = crate::dsl::parse_pedal_file(r#"
            pedal "test_inv" { supply 9V
                components {
                    R1: resistor(10k)
                    Rf: resistor(100k)
                    U1: opamp(tl072)
                }
                nets {
                    in -> R1.a
                    R1.b -> U1.neg
                    Rf.a -> U1.neg
                    Rf.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> out
                }
                controls {}
            }"#)
        .expect("parse");

        let mut compiled = compile_via_spqr(&pedal, 48000.0)
            .expect("Should compile inverting amp via SPQR");

        // Settle
        for _ in 0..10 {
            compiled.process(0.1);
        }
        let output = compiled.process(0.1);
        let gain = output.abs() / 0.1;
        assert!(
            gain > 5.0 && gain < 15.0,
            "Inverting gain should be ~10, got {gain:.2}"
        );
    }

    #[test]
    fn compile_via_spqr_opamp_diode_feedback() {
        // TS-style: R1 → opamp(neg) ← Rf ← diode ← opamp(out)
        // OpAmpRoot pre-amplifies, diode clips
        use crate::PedalProcessor;

        let pedal = crate::dsl::parse_pedal_file(r#"
            pedal "test_ts" { supply 9V
                components {
                    R1: resistor(4.7k)
                    D1: diode(silicon)
                    Rf: resistor(51k)
                    U1: opamp(jrc4558)
                }
                nets {
                    in -> R1.a
                    R1.b -> U1.neg
                    D1.a -> U1.neg
                    D1.b -> U1.out
                    Rf.a -> U1.neg
                    Rf.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> out
                }
                controls {}
            }"#)
        .expect("parse");

        let mut compiled = compile_via_spqr(&pedal, 48000.0)
            .expect("Should compile TS-style circuit via SPQR");

        // Small signal should be amplified (gain ≈ Rf/Ri ≈ 51k/4.7k ≈ 10.8)
        for _ in 0..20 {
            compiled.process(0.01);
        }
        let output = compiled.process(0.01);
        assert!(
            output.abs() > 0.02,
            "Should amplify small signal, got {output:.6}"
        );

        // Large signal should clip (diode limits output)
        for _ in 0..20 {
            compiled.process(0.5);
        }
        let loud_output = compiled.process(0.5);
        // Gain of ~11 would give 5.5V, but diode clips at ~0.6V
        assert!(
            loud_output.abs() < 3.0,
            "Diode should clip large signal, got {loud_output:.4}V"
        );
    }

    #[test]
    fn spqr_noninverting_opamp_gain() {
        // Non-inverting amp: R1=10k, Rf=100k → gain = 1 + 100k/10k = 11
        let (graph, edges) = make_graph_all_edges(r#"
            pedal "test" { supply 9V
                components {
                    R1: resistor(10k)
                    Rf: resistor(100k)
                    U1: opamp(tl072)
                }
                nets {
                    in -> U1.pos
                    U1.neg -> R1.a
                    R1.b -> gnd
                    U1.neg -> Rf.a
                    Rf.b -> U1.out
                    U1.out -> out
                }
                controls {}
            }"#);
        let spqr = spqr_decompose(
            &edges,
            &[graph.in_node, graph.out_node],
            &graph,
            graph.gnd_node,
        );
        let spqr_stages = spqr_to_stages(&spqr, &graph, 48000.0);

        assert_eq!(spqr_stages.len(), 1);
        let mut stage = build_spqr_stage(
            spqr_stages.into_iter().next().unwrap(),
            &graph,
            48000.0,
        )
        .expect("Should build OpAmpRoot stage");

        for _ in 0..10 {
            stage.process(0.1);
        }
        let output = stage.process(0.1);
        let gain_measured = output.abs() / 0.1;

        assert!(
            gain_measured > 6.0,
            "Non-inverting gain should be ~11, got {gain_measured:.2}"
        );
        assert!(
            gain_measured < 16.0,
            "Non-inverting gain should be ~11, got {gain_measured:.2}"
        );
    }
}
