//! SPQR stage builder: converts `SpqrStage` descriptors into runnable WDF stages.
//!
//! Separation of concerns:
//! - `spqr.rs`: graph decomposition + classification (topology + component semantics)
//! - `spqr_build.rs`: stage construction (DynNode trees → WdfStage processors)
//!
//! The builder adds a voltage source input port, maps NL edges to RootKind
//! via Component::classify_nonlinear(), and wraps everything in WdfStage.

use super::build::create_root;
use super::dyn_node::DynNode;
use super::graph::{CircuitGraph, NodeId};
use super::rigid_build::{classify_rigid, RigidOptimization, StageStats};
use super::spqr::SpqrStage;
use super::stage::{RootKind, WdfStage};
use super::wdf_leaf::WdfVoltageSource;
use crate::elements::OpAmpModel;
use crate::elements::OpAmpRoot;
use crate::oversampling::{Oversampler, OversamplingFactor};

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
            Err("General MNA rigid stages not yet implemented".to_string())
        }
    }
}

/// Build an OpAmpRoot stage from a simple VCVS feedback topology.
///
/// Extracts Rf from linear edges in the R-node, Ri from pendant trees,
/// and the OpAmpModel from the VCVS edge's Component.
fn build_opamp_root(
    edge_indices: &[usize],
    pendant_trees: &[(DynNode, NodeId)],
    inverting: bool,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<WdfStage, String> {
    // Find the VCVS edge → get OpAmpModel
    let vcvs_edge_idx = edge_indices
        .iter()
        .find(|&&eidx| {
            graph.effective_edge_kind(eidx) == super::component::EdgeKind::Vcvs
        })
        .ok_or("No VCVS edge in OpAmpRoot stage")?;

    let vcvs_comp = &graph.components[graph.edges[*vcvs_edge_idx].comp_idx];
    let op_type = vcvs_comp
        .kind
        .op_amp_type()
        .ok_or("VCVS edge component has no op_amp_type")?;
    let model = OpAmpModel::from_opamp_type(&op_type);

    // Sum feedback resistance (Rf): linear edges in the R-node (between neg and out)
    let rf: f64 = edge_indices
        .iter()
        .filter(|&&eidx| {
            graph.effective_edge_kind(eidx) == super::component::EdgeKind::Linear
        })
        .filter_map(|&eidx| {
            let comp = &graph.components[graph.edges[eidx].comp_idx];
            comp.kind.resistance()
        })
        .sum();

    if rf <= 0.0 {
        return Err("OpAmpRoot: no feedback resistance found".to_string());
    }

    // Input resistance (Ri): from pendant tree port resistance
    let ri: f64 = if !pendant_trees.is_empty() {
        pendant_trees.iter().map(|(tree, _)| tree.port_resistance()).sum()
    } else {
        // No pendant → unity gain buffer (Ri = infinity, gain = 1)
        f64::INFINITY
    };

    let gain = if ri.is_infinite() {
        1.0
    } else if inverting {
        rf / ri
    } else {
        1.0 + rf / ri
    };

    let supply_voltage: f64 = 9.0; // TODO: from PedalDef
    let mut root = if inverting {
        OpAmpRoot::new_inverting(model, gain)
    } else {
        OpAmpRoot::new_non_inverting(model, gain)
    };
    root.set_sample_rate(sample_rate);
    root.set_v_max((supply_voltage / 2.0 - 1.5).max(0.5));

    // Build tree: VS (input port) in series with pendant Ri if present
    let tree = if let Some((pendant, _)) = pendant_trees.first() {
        let ri_tree = pendant.clone();
        with_voltage_source(ri_tree)
    } else {
        // Unity buffer: just a VS
        DynNode::Leaf(Box::new(WdfVoltageSource {
            voltage: 0.0,
            rp: 1.0,
            is_cathode_bias: false,
        }))
    };

    let oversampler = Oversampler::new(OversamplingFactor::X1);
    let mut stage = WdfStage::new(tree, RootKind::OpAmp(root), oversampler);
    stage.compensation = 1.0;
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
