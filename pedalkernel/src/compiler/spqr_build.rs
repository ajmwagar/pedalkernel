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
use super::graph::CircuitGraph;
use super::spqr::SpqrStage;
use super::stage::{RootKind, WdfStage};
use super::wdf_leaf::WdfVoltageSource;
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
        SpqrStage::Rigid { .. } => {
            Err("Rigid stages not yet supported in SPQR pipeline".to_string())
        }
    }
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
}
