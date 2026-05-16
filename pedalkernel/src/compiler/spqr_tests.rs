// Tests extracted from spqr.rs — SPQR decomposition + classification + stage conversion.

use super::graph::CircuitGraph;
use super::spqr::*;

/// Helper: build a CircuitGraph from a .pedal source string.
/// Returns (graph, passive_edge_indices).
fn make_graph(pedal_src: &str) -> (CircuitGraph, Vec<usize>) {
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

/// Helper: build a CircuitGraph and return ALL non-active-IC edges
/// (passives + nonlinear elements like diodes). Used for NlWdf tests.
fn make_graph_all_edges(pedal_src: &str) -> (CircuitGraph, Vec<usize>) {
    let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&pedal);
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();
    (graph, all_edges)
}

#[test]
fn spqr_single_resistor() {
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k) }
            nets { in -> R1.a  R1.b -> out }
            controls {}
        }"#,
    );
    let result = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    assert!(result.is_leaf(), "Single resistor should be Q node");
    assert_eq!(result.edge_count(), 1);
}

#[test]
fn spqr_series_two_resistors() {
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  R2: resistor(10k) }
            nets { in -> R1.a  R1.b -> R2.a  R2.b -> out }
            controls {}
        }"#,
    );
    let result = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    match &result {
        SpqrNode::S { children, .. } => {
            assert!(
                children.len() >= 2,
                "Series should have ≥2 children, got {}",
                children.len()
            );
        }
        other => panic!(
            "Expected S node for series resistors, got {:?}",
            std::mem::discriminant(other)
        ),
    }
    assert_eq!(result.edge_count(), 2);
}

#[test]
fn spqr_parallel_two_resistors() {
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  R2: resistor(20k) }
            nets { in -> R1.a, R2.a  R1.b -> out  R2.b -> out }
            controls {}
        }"#,
    );
    let result = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    match &result {
        SpqrNode::P { children, .. } => {
            assert_eq!(children.len(), 2, "Parallel should have 2 children");
        }
        other => panic!(
            "Expected P node for parallel resistors, got {:?}",
            std::mem::discriminant(other)
        ),
    }
    assert_eq!(result.edge_count(), 2);
}

#[test]
fn spqr_wheatstone_bridge_is_rigid() {
    // Wheatstone bridge: 5 resistors, NOT series-parallel
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)  R2: resistor(10k)
                R3: resistor(10k)  R4: resistor(10k)
                R5: resistor(10k)
            }
            nets {
                in -> R1.a, R3.a
                R1.b -> R2.a, R5.a
                R3.b -> R4.a, R5.b
                R2.b -> out  R4.b -> out
            }
            controls {}
        }"#,
    );
    let result = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    assert!(
        result.is_rigid(),
        "Wheatstone bridge should be R node, got {:?}",
        std::mem::discriminant(&result)
    );
    assert_eq!(result.edge_count(), 5, "Must preserve all 5 edges");
}

#[test]
fn spqr_rc_lowpass_is_series() {
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  C1: cap(100n) }
            nets { in -> R1.a  R1.b -> C1.a  C1.b -> out }
            controls {}
        }"#,
    );
    let result = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    match &result {
        SpqrNode::S { children, .. } => {
            assert!(
                children.len() >= 2,
                "RC lowpass should be series with ≥2 children"
            );
        }
        other => panic!(
            "Expected S node for RC lowpass, got {:?}",
            std::mem::discriminant(other)
        ),
    }
}

#[test]
fn spqr_preserves_all_edges() {
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)  R2: resistor(10k)
                R3: resistor(10k)  R4: resistor(10k)
                R5: resistor(10k)
            }
            nets {
                in -> R1.a, R3.a
                R1.b -> R2.a, R5.a
                R3.b -> R4.a, R5.b
                R2.b -> out  R4.b -> out
            }
            controls {}
        }"#,
    );
    let n_passive = edges.len();
    let result = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    assert_eq!(
        result.edge_count(),
        n_passive,
        "Decomposition must preserve all {} passive edges",
        n_passive
    );
}

// ── Phase 2: SPQR → stages tests ────────────────────────────────

#[test]
fn spqr_to_stages_passive_rc_single_wdf() {
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  C1: cap(100n) }
            nets { in -> R1.a  R1.b -> C1.a  C1.b -> out }
            controls {}
        }"#,
    );
    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    let stages = spqr_to_stages(&tree, &graph, 48000.0);
    assert_eq!(stages.len(), 1, "Passive RC should produce 1 stage");
    assert!(
        matches!(&stages[0], SpqrStage::PassiveWdf { edge_indices, .. } if edge_indices.len() == 2),
        "Passive RC should be PassiveWdf with 2 edges, got {:?}",
        stages[0]
    );
}

#[test]
fn spqr_to_stages_parallel_rc_single_wdf() {
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  R2: resistor(20k) }
            nets { in -> R1.a, R2.a  R1.b -> out  R2.b -> out }
            controls {}
        }"#,
    );
    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    let stages = spqr_to_stages(&tree, &graph, 48000.0);
    assert_eq!(stages.len(), 1, "Parallel R should produce 1 stage");
    assert!(
        matches!(&stages[0], SpqrStage::PassiveWdf { .. }),
        "Parallel R should be PassiveWdf, got {:?}",
        stages[0]
    );
}

#[test]
fn spqr_to_stages_bridge_becomes_rigid() {
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)  R2: resistor(10k)
                R3: resistor(10k)  R4: resistor(10k)
                R5: resistor(10k)
            }
            nets {
                in -> R1.a, R3.a
                R1.b -> R2.a, R5.a
                R3.b -> R4.a, R5.b
                R2.b -> out  R4.b -> out
            }
            controls {}
        }"#,
    );
    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    let stages = spqr_to_stages(&tree, &graph, 48000.0);

    let has_rigid = stages.iter().any(|s| matches!(s, SpqrStage::Rigid { .. }));
    assert!(has_rigid, "Wheatstone bridge should produce a Rigid stage");
}

#[test]
fn spqr_to_stages_ordering_matches_traversal() {
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  R2: resistor(10k)  R3: resistor(10k) }
            nets { in -> R1.a  R1.b -> R2.a  R2.b -> R3.a  R3.b -> out }
            controls {}
        }"#,
    );
    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    let stages = spqr_to_stages(&tree, &graph, 48000.0);

    let get_order = |s: &SpqrStage| match s {
        SpqrStage::PassiveWdf { order, .. }
        | SpqrStage::NlWdf { order, .. }
        | SpqrStage::Rigid { order, .. } => *order,
    };
    for i in 1..stages.len() {
        assert!(
            get_order(&stages[i]) > get_order(&stages[i - 1]),
            "Stage ordering should be monotonic: stage[{}]={} <= stage[{}]={}",
            i - 1,
            get_order(&stages[i - 1]),
            i,
            get_order(&stages[i])
        );
    }
}

// ── Phase 2.5: classifier + NlWdf tests ────────────────────────

#[test]
fn classify_all_passive_rc() {
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  C1: cap(100n) }
            nets { in -> R1.a  R1.b -> C1.a  C1.b -> out }
            controls {}
        }"#,
    );
    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    assert_eq!(
        classify_sp_subtree(&tree, &graph),
        SpClassification::AllPassive
    );
}

#[test]
fn classify_single_nl_diode_clipper() {
    // R in series with diode pair to ground — classic hard clipper
    let (graph, edges) = make_graph_all_edges(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  D1: diode(silicon) }
            nets { in -> R1.a  R1.b -> D1.a  D1.b -> out }
            controls {}
        }"#,
    );
    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    match classify_sp_subtree(&tree, &graph) {
        SpClassification::SingleNl { .. } => {} // correct
        other => panic!("Diode clipper should be SingleNl, got {:?}", other),
    }
}

#[test]
fn classify_complex_two_diodes() {
    let (graph, edges) = make_graph_all_edges(
        r#"
        pedal "test" { supply 9V
            components { D1: diode(silicon)  D2: diode(silicon) }
            nets { in -> D1.a  D1.b -> D2.a  D2.b -> out }
            controls {}
        }"#,
    );
    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    assert_eq!(
        classify_sp_subtree(&tree, &graph),
        SpClassification::Complex
    );
}

#[test]
fn spqr_to_stages_diode_clipper_is_nl_wdf() {
    let (graph, edges) = make_graph_all_edges(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  C1: cap(100n)  D1: diode(silicon) }
            nets { in -> R1.a  R1.b -> C1.a  C1.b -> D1.a  D1.b -> out }
            controls {}
        }"#,
    );
    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    let stages = spqr_to_stages(&tree, &graph, 48000.0);
    assert_eq!(stages.len(), 1, "Diode clipper should produce 1 stage");
    match &stages[0] {
        SpqrStage::NlWdf {
            nl_edge_idx,
            edge_indices,
            tree,
            ..
        } => {
            assert_eq!(edge_indices.len(), 3, "Should have R + C + D");
            // The passive tree should have port_resistance for R + C
            let rp = tree.port_resistance();
            assert!(
                rp > 0.0,
                "Passive tree should have valid port resistance, got {rp}"
            );
            // The NL edge should point to the diode
            let nl_edge = &graph.edges[*nl_edge_idx];
            let nl_comp = &graph.components[nl_edge.comp_idx];
            assert!(!nl_comp.kind.is_passive(), "NL edge should be non-passive");
        }
        other => panic!("Diode clipper should be NlWdf, got {:?}", other),
    }
}

#[test]
fn spqr_to_stages_nl_wdf_passive_tree_excludes_diode() {
    let (graph, edges) = make_graph_all_edges(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(4.7k)  D1: diode(silicon) }
            nets { in -> R1.a  R1.b -> D1.a  D1.b -> out }
            controls {}
        }"#,
    );
    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    let stages = spqr_to_stages(&tree, &graph, 48000.0);
    match &stages[0] {
        SpqrStage::NlWdf { tree, .. } => {
            // Passive tree should only be the resistor (4.7k)
            let rp = tree.port_resistance();
            assert!(
                (rp - 4700.0).abs() < 100.0,
                "Passive tree should be just R1=4.7k, got Rp={rp:.0}"
            );
        }
        other => panic!("Expected NlWdf, got {:?}", other),
    }
}

// ── Phase 3: Op-amp absorption via VCVS edges ─────────────────

#[test]
fn classify_vcvs_inverting_opamp() {
    // Inverting amp: R1 + Rf + U1(VCVS) — should classify as Vcvs
    let (graph, edges) = make_graph_all_edges(
        r#"
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
        }"#,
    );
    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    assert_eq!(
        classify_sp_subtree(&tree, &graph),
        SpClassification::Vcvs,
        "Inverting op-amp should classify as Vcvs"
    );
}

#[test]
fn spqr_opamp_inverting_emits_rigid() {
    let (graph, edges) = make_graph_all_edges(
        r#"
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
        }"#,
    );
    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    let stages = spqr_to_stages(&tree, &graph, 48000.0);

    assert_eq!(stages.len(), 1, "Inverting amp should produce 1 stage");
    match &stages[0] {
        SpqrStage::Rigid {
            edge_indices,
            pendant_trees,
            ..
        } => {
            // Rf and U1(VCVS) are in the MNA edge list (parallel, neg→out)
            // R1 is extracted as a pendant WDF tree (series, in→neg)
            assert_eq!(edge_indices.len(), 2, "MNA edges: Rf + U1(VCVS)");
            assert_eq!(pendant_trees.len(), 1, "R1 should be a pendant WDF port");
            // Verify the VCVS edge is present in MNA edges
            let has_vcvs = edge_indices
                .iter()
                .any(|&eidx| graph.effective_edge_kind(eidx) == super::component::EdgeKind::Vcvs);
            assert!(has_vcvs, "Rigid stage should contain the VCVS edge");
        }
        other => panic!("Inverting amp should be Rigid, got {:?}", other),
    }
}

#[test]
fn spqr_opamp_noninverting_emits_rigid() {
    // Non-inverting: signal to pos, voltage divider on neg
    let (graph, edges) = make_graph_all_edges(
        r#"
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
        }"#,
    );
    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    let stages = spqr_to_stages(&tree, &graph, 48000.0);

    let has_rigid = stages.iter().any(|s| matches!(s, SpqrStage::Rigid { .. }));
    assert!(has_rigid, "Non-inverting amp should produce a Rigid stage");
}

#[test]
fn spqr_diode_opamp_shared_rigid() {
    // TS-style: diode clipper in op-amp feedback loop
    // All three (R1, D1, Rf, U1) should land in one Rigid stage
    let (graph, edges) = make_graph_all_edges(
        r#"
        pedal "test" { supply 9V
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
        }"#,
    );
    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    let stages = spqr_to_stages(&tree, &graph, 48000.0);

    // Single Rigid: D1 + Rf + U1 in MNA, R1 as pendant WDF port
    assert_eq!(stages.len(), 1, "TS clipping should produce 1 stage");
    match &stages[0] {
        SpqrStage::Rigid {
            edge_indices,
            pendant_trees,
            ..
        } => {
            // D1, Rf, U1 are parallel (neg→out) → MNA edges
            // R1 is pendant (in→neg) → WDF port
            assert_eq!(edge_indices.len(), 3, "MNA edges: D1 + Rf + U1");
            assert_eq!(pendant_trees.len(), 1, "R1 should be a pendant WDF port");
        }
        other => panic!("TS clipping should be Rigid, got {:?}", other),
    }
}

#[test]
fn spqr_opamp_preserves_passive_context() {
    // Inverting amp with input RC filter.
    // VCVS edges are handled by signal_flow groups (not SPQR decomposition).
    // The SPQR decomposition only sees passive edges: C1, R1, Rf.
    // These form a series chain → single PassiveWdf or Rigid stage.
    let (graph, edges) = make_graph_all_edges(
        r#"
        pedal "test" { supply 9V
            components {
                C1: cap(100n)
                R1: resistor(10k)
                Rf: resistor(100k)
                U1: opamp(tl072)
            }
            nets {
                in -> C1.a
                C1.b -> R1.a
                R1.b -> U1.neg
                Rf.a -> U1.neg
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    );
    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    let stages = spqr_to_stages(&tree, &graph, 48000.0);

    // VCVS excluded → 3 passive edges decomposed
    let total_edges: usize = stages
        .iter()
        .map(|s| match s {
            SpqrStage::PassiveWdf { edge_indices, .. }
            | SpqrStage::NlWdf { edge_indices, .. }
            | SpqrStage::Rigid { edge_indices, .. } => edge_indices.len(),
        })
        .sum();
    assert_eq!(
        total_edges, 3,
        "3 passive edges (C1 + R1 + Rf), VCVS handled by signal flow"
    );

    // Full pipeline test: the compile_via_spqr path groups VCVS + passives
    // into a single FlowGroup → IIR stage with NonIdealFx.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                C1: cap(100n)
                R1: resistor(10k)
                Rf: resistor(100k)
                U1: opamp(tl072)
            }
            nets {
                in -> C1.a
                C1.b -> R1.a
                R1.b -> U1.neg
                Rf.a -> U1.neg
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");
    let compiled = super::spqr_build::compile_via_spqr(&pedal, 48000.0);
    assert!(compiled.is_ok(), "Should compile through full pipeline");
}

// ── Phase 2 continued ───────────────────────────────────────────

#[test]
fn spqr_dyn_node_series_has_correct_structure() {
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  R2: resistor(10k) }
            nets { in -> R1.a  R1.b -> R2.a  R2.b -> out }
            controls {}
        }"#,
    );
    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    let dyn_node = spqr_to_dyn_node(&tree, &graph, 48000.0);
    assert!(
        dyn_node.is_some(),
        "Series resistors should produce a DynNode"
    );
    // The DynNode should have port_resistance = R1 + R2 = 20k
    let node = dyn_node.unwrap();
    let rp = node.port_resistance();
    assert!(
        (rp - 20_000.0).abs() < 100.0,
        "Series R1+R2 port resistance should be ~20k, got {rp:.0}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// T-junction: passive network with a branch (the canary for HPF regression)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn spqr_t_junction_is_series_parallel() {
    // T-junction: R1 → junction → (C1 to gnd, R2 to out)
    // This is Series(R1, Parallel(C1, R2)) — SP-reducible.
    // SPQR should NOT classify this as Rigid.
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                C1: cap(22n)
                R2: resistor(10k)
            }
            nets {
                in -> R1.a
                R1.b -> C1.a
                C1.b -> gnd
                R1.b -> R2.a
                R2.b -> out
            }
            controls {}
        }"#,
    );

    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );

    // Should be S or P, NOT R
    let is_rigid = matches!(tree, SpqrNode::R { .. });
    eprintln!("T-junction SPQR: rigid={is_rigid}, tree={tree:?}");
    assert!(
        !is_rigid,
        "T-junction (R + parallel C/R) should be SP-reducible, not Rigid"
    );
}

#[test]
fn spqr_t_junction_with_wrong_terminals_is_rigid() {
    // Same T-junction, but with terminals that DON'T match the circuit's
    // boundary nodes. If terminals are [in, out] but the passive group's
    // edges don't span in→out, SPQR falls back to Rigid.
    //
    // This is the canary: when signal_flow creates a passive group that
    // doesn't touch in/out, SPQR misclassifies it.
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components {
                R_before: resistor(10k)
                R1: resistor(10k)
                C1: cap(22n)
                R2: resistor(10k)
                R_after: resistor(10k)
            }
            nets {
                in -> R_before.a
                R_before.b -> R1.a
                R1.b -> C1.a
                C1.b -> gnd
                R1.b -> R2.a
                R2.b -> R_after.a
                R_after.b -> out
            }
            controls {}
        }"#,
    );

    // Decompose ONLY the T-junction edges (R1, C1, R2) — not R_before/R_after.
    // Use the circuit's in/out as terminals (which the T-junction doesn't span).
    let t_edges: Vec<usize> = edges
        .iter()
        .copied()
        .filter(|&eidx| {
            let comp = &graph.components[graph.edges[eidx].comp_idx];
            comp.id == "R1" || comp.id == "C1" || comp.id == "R2"
        })
        .collect();

    eprintln!(
        "T-junction sub-group edges: {:?}",
        t_edges
            .iter()
            .map(|&e| { graph.components[graph.edges[e].comp_idx].id.clone() })
            .collect::<Vec<_>>()
    );

    // With global terminals [in, out] — T-junction doesn't touch these
    let tree_global = spqr_decompose(
        &t_edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    let rigid_with_global = matches!(tree_global, SpqrNode::R { .. });

    // With correct boundary terminals [R_before.b, R_after.a]
    let r_before_b = graph
        .node_names
        .get("R_before.b")
        .copied()
        .unwrap_or(graph.in_node);
    let r_after_a = graph
        .node_names
        .get("R_after.a")
        .copied()
        .unwrap_or(graph.out_node);
    let tree_local = spqr_decompose(&t_edges, &[r_before_b, r_after_a], &graph, graph.gnd_node);
    let rigid_with_local = matches!(tree_local, SpqrNode::R { .. });

    eprintln!("Global terminals: rigid={rigid_with_global}");
    eprintln!("Local terminals:  rigid={rigid_with_local}");

    // With local terminals, the T-junction should be SP-reducible
    assert!(
        !rigid_with_local,
        "T-junction with correct boundary terminals should be SP-reducible"
    );
}

#[test]
fn spqr_t_junction_produces_audio_as_stage() {
    // End-to-end: passive T-junction tone stack between op-amp and output.
    // When SPQR resolves the correct terminals, the passive group becomes
    // a WDF tree instead of an IIR with b=[0,0,0].
    use crate::PedalProcessor;

    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                R_tone: resistor(10k)
                C_tone: cap(22n)
                R_out: resistor(10k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> R_tone.a
                R_tone.b -> C_tone.a
                C_tone.b -> gnd
                R_tone.b -> R_out.a
                R_out.b -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled = super::spqr_build::compile_via_spqr(&pedal, 48000.0).expect("compile");

    // Settle
    for _ in 0..2000 {
        compiled.process(0.0);
    }

    // 1kHz should pass through tone stack
    let mut peak = 0.0f64;
    for s in 0..4800 {
        let input = 0.05 * (2.0 * std::f64::consts::PI * 1000.0 * s as f64 / 48000.0).sin();
        peak = peak.max(compiled.process(input).abs());
    }

    eprintln!("T-junction tone stack: 1kHz peak={peak:.6}");
    assert!(
        peak > 0.001,
        "Tone stack should pass signal: peak={peak:.6}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Pendant → stage pipeline: SPQR tree with pendants must produce
// PassiveWdf stages (not Rigid → IIR with b=[0,0,0])
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn spqr_t_junction_becomes_passive_wdf_stage() {
    // The T-junction SPQR tree (with pendant extraction) should convert
    // to a PassiveWdf stage, NOT a Rigid stage.
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                C1: cap(22n)
                R2: resistor(10k)
            }
            nets {
                in -> R1.a
                R1.b -> C1.a
                C1.b -> gnd
                R1.b -> R2.a
                R2.b -> out
            }
            controls {}
        }"#,
    );

    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    let stages = spqr_to_stages(&tree, &graph, 48000.0);

    eprintln!("T-junction stages:");
    for (i, s) in stages.iter().enumerate() {
        eprintln!("  [{i}]: {s:?}");
    }

    // Should produce PassiveWdf stage(s), NOT Rigid
    let has_passive_wdf = stages
        .iter()
        .any(|s| matches!(s, SpqrStage::PassiveWdf { .. }));
    let has_rigid = stages.iter().any(|s| matches!(s, SpqrStage::Rigid { .. }));

    assert!(
        has_passive_wdf,
        "T-junction should produce PassiveWdf stage"
    );
    assert!(!has_rigid, "T-junction should NOT produce Rigid stage");
}

#[test]
fn spqr_t_junction_dyn_node_has_all_components() {
    // The DynNode tree from a T-junction should contain all 3 components:
    // R1, C1, R2. None should be lost during pendant extraction.
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                C1: cap(22n)
                R2: resistor(10k)
            }
            nets {
                in -> R1.a
                R1.b -> C1.a
                C1.b -> gnd
                R1.b -> R2.a
                R2.b -> out
            }
            controls {}
        }"#,
    );

    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );

    // Convert to DynNode
    let dyn_node = spqr_to_dyn_node(&tree, &graph, 48000.0);
    assert!(
        dyn_node.is_some(),
        "T-junction should produce a valid DynNode"
    );

    let node = dyn_node.unwrap();
    // Count leaves
    let mut leaf_count = 0;
    node.for_each_leaf(&mut |_leaf| {
        leaf_count += 1;
    });
    eprintln!(
        "T-junction DynNode: {leaf_count} leaves, rp={:.0}",
        node.port_resistance()
    );

    assert_eq!(leaf_count, 3, "DynNode should have 3 leaves (R1, C1, R2)");
}

#[test]
fn spqr_t_junction_stage_edges_include_all() {
    // When spqr_to_stages converts a T-junction, the PassiveWdf stage's
    // edge_indices must include ALL edges (R1, C1, R2) — not just the
    // core series chain.
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                C1: cap(22n)
                R2: resistor(10k)
            }
            nets {
                in -> R1.a
                R1.b -> C1.a
                C1.b -> gnd
                R1.b -> R2.a
                R2.b -> out
            }
            controls {}
        }"#,
    );

    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    let stages = spqr_to_stages(&tree, &graph, 48000.0);

    let total_edges: usize = stages
        .iter()
        .map(|s| match s {
            SpqrStage::PassiveWdf { edge_indices, .. }
            | SpqrStage::NlWdf { edge_indices, .. }
            | SpqrStage::Rigid { edge_indices, .. } => edge_indices.len(),
        })
        .sum();

    assert_eq!(
        total_edges, 3,
        "All 3 edges should be accounted for in stages"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Ground-terminated caps in SP reduction
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn spqr_single_cap_to_ground_is_not_rigid() {
    // A cap from signal node to GND is a valid WDF leaf.
    // SPQR should treat it as a Q node (single edge), not Rigid.
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  C1: cap(100n) }
            nets { in -> R1.a  R1.b -> C1.a  C1.b -> gnd  R1.b -> out }
            controls {}
        }"#,
    );

    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    let is_rigid = matches!(tree, SpqrNode::R { .. });
    eprintln!("R + C-to-gnd: rigid={is_rigid}, tree={tree:?}");
    assert!(
        !is_rigid,
        "R + C-to-ground should be SP-reducible (parallel at junction)"
    );
}

#[test]
fn spqr_two_caps_to_ground_are_parallel() {
    // Two caps from the same node to GND are parallel.
    // Common pattern: bypass caps, decoupling.
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  C1: cap(100n)  C2: cap(10n) }
            nets {
                in -> R1.a
                R1.b -> C1.a
                C1.b -> gnd
                R1.b -> C2.a
                C2.b -> gnd
                R1.b -> out
            }
            controls {}
        }"#,
    );

    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    let is_rigid = matches!(tree, SpqrNode::R { .. });
    eprintln!("R + 2×C-to-gnd: rigid={is_rigid}");
    assert!(!is_rigid, "R + two ground caps should be SP-reducible");
}

#[test]
fn spqr_bridged_t_stays_rigid() {
    // Bridged-T: R1-R2 series with C1, C2 ground shunts AND Rf bridging.
    // The bridge resistor Rf makes it genuinely non-SP. Must stay Rigid.
    // (This is the 808 kick drum pattern — IIR handles it correctly.)
    let (graph, edges) = make_graph_all_edges(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                R2: resistor(10k)
                C1: cap(68n)
                C2: cap(68n)
                Rf: resistor(47k)
                U1: opamp(tl072)
            }
            nets {
                in -> R1.a
                R1.b -> R2.a
                R2.b -> U1.neg
                R1.b -> C1.a
                C1.b -> gnd
                R2.b -> C2.a
                C2.b -> gnd
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    );

    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    // Should have at least one Rigid node (the bridged-T can't SP-reduce)
    fn has_rigid(node: &SpqrNode) -> bool {
        match node {
            SpqrNode::R { children, .. } => true,
            SpqrNode::S { children, .. } | SpqrNode::P { children, .. } => {
                children.iter().any(has_rigid)
            }
            SpqrNode::Q { .. } => false,
        }
    }
    assert!(
        has_rigid(&tree),
        "Bridged-T must contain a Rigid node (bridge prevents SP reduction)"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Ground shunts must be parallel, not series
// ═══════════════════════════════════════════════════════════════════════════

/// Helper: check if an SpqrNode tree contains at least one P-node.
fn has_p_node(node: &SpqrNode) -> bool {
    match node {
        SpqrNode::P { .. } => true,
        SpqrNode::S { children, .. } => children.iter().any(has_p_node),
        SpqrNode::R { children, .. } => children.iter().any(has_p_node),
        SpqrNode::Q { .. } => false,
    }
}

#[test]
fn spqr_ground_shunt_is_parallel_not_series() {
    // T-junction: in --R1-- J --R2-- out, C1: J -> GND
    // C1 is a ground shunt at junction J. It must be PARALLEL to the
    // signal path at J, not in series with R1 and R2.
    //
    // Correct WDF tree: Series(R1, Parallel(R2, C1))
    // Wrong WDF tree:   Series(R1, R2, C1)
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                C1: cap(100n)
                R2: resistor(10k)
            }
            nets {
                in -> R1.a
                R1.b -> C1.a
                C1.b -> gnd
                R1.b -> R2.a
                R2.b -> out
            }
            controls {}
        }"#,
    );

    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );

    eprintln!("Ground shunt tree: {tree:?}");

    // Must contain a P-node (the ground shunt is parallel)
    assert!(
        has_p_node(&tree),
        "Ground shunt C1 must be in a P-node (parallel), not flat S-node (series)"
    );

    // The top-level should be S-node (R1 in series with the parallel group)
    assert!(
        matches!(tree, SpqrNode::S { .. }),
        "Top-level should be S-node: Series(R1, Parallel(R2, C1))"
    );

    // Convert to DynNode and check port resistance
    let dyn_node = spqr_to_dyn_node(&tree, &graph, 48000.0).unwrap();
    let rp = dyn_node.port_resistance();
    eprintln!("  rp = {rp:.1}");

    // Wrong (series): rp = R1 + R2 + Zc(100nF@48kHz) = 10k + 10k + 33.2 = ~20033
    // Correct (parallel): rp = R1 + (R2 || Zc) = 10k + (10k || 33.2) ≈ 10033
    // The parallel should bring rp well below 20k
    assert!(
        rp < 15_000.0,
        "rp={rp:.0} too high — C1 likely in series instead of parallel. Expected ~10033"
    );
}

#[test]
fn spqr_multiple_ground_shunts_same_junction() {
    // Two ground shunts at the same junction → both parallel
    // in --R1-- J --R2-- out, C1: J->GND, C2: J->GND
    // Expected: S(R1, P(R2, C1, C2))
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                C1: cap(100n)
                C2: cap(10n)
                R2: resistor(10k)
            }
            nets {
                in -> R1.a
                R1.b -> C1.a  C1.b -> gnd
                R1.b -> C2.a  C2.b -> gnd
                R1.b -> R2.a  R2.b -> out
            }
            controls {}
        }"#,
    );

    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );

    eprintln!("Two ground shunts: {tree:?}");
    assert!(has_p_node(&tree), "Both shunts must be in P-nodes");

    let dyn_node = spqr_to_dyn_node(&tree, &graph, 48000.0).unwrap();
    let rp = dyn_node.port_resistance();
    let mut leaf_count = 0;
    dyn_node.for_each_leaf(&mut |_| {
        leaf_count += 1;
    });

    eprintln!("  rp={rp:.1}, leaves={leaf_count}");
    assert_eq!(leaf_count, 4, "Should have 4 leaves: R1, R2, C1, C2");
    // Parallel of R2||C1||C2 is very small → rp ≈ R1 + small ≈ 10k
    assert!(rp < 15_000.0, "rp={rp:.0} too high");
}

#[test]
fn spqr_ground_shunts_at_different_junctions() {
    // Shunts at different junctions: each gets its own P-node
    // in --R1-- J1 --R2-- J2 --R3-- out
    //                C1: J1->GND, C2: J2->GND
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                R2: resistor(10k)
                R3: resistor(10k)
                C1: cap(100n)
                C2: cap(100n)
            }
            nets {
                in -> R1.a
                R1.b -> C1.a  C1.b -> gnd
                R1.b -> R2.a
                R2.b -> C2.a  C2.b -> gnd
                R2.b -> R3.a
                R3.b -> out
            }
            controls {}
        }"#,
    );

    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );

    eprintln!("Shunts at different junctions: {tree:?}");

    let dyn_node = spqr_to_dyn_node(&tree, &graph, 48000.0).unwrap();
    let rp = dyn_node.port_resistance();
    let mut leaf_count = 0;
    dyn_node.for_each_leaf(&mut |_| {
        leaf_count += 1;
    });

    eprintln!("  rp={rp:.1}, leaves={leaf_count}");
    assert_eq!(leaf_count, 5, "Should have 5 leaves: R1, R2, R3, C1, C2");
    // Series(R1, P(R2, C1), P(R3, C2))
    // rp = R1 + (R2||Zc1) + (R3||Zc2)
    // Each parallel is small → rp ≈ R1 + small + small ≈ 10k
    assert!(
        rp < 15_000.0,
        "rp={rp:.0} too high — shunts likely in series"
    );
}

#[test]
fn spqr_ground_shunt_port_resistance_matches_theory() {
    // Precise port resistance check for a simple T-junction
    // in --R1(10k)-- J --R2(20k)-- out, C1(100nF): J->GND
    //
    // At 48kHz: Zc = 1/(2π × 48000 × 100e-9) ≈ 33.16Ω
    // Parallel(R2, C1) = (20000 × 33.16) / (20000 + 33.16) ≈ 33.10Ω
    // Total rp = R1 + Parallel = 10000 + 33.10 ≈ 10033.1
    let (graph, edges) = make_graph(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                C1: cap(100n)
                R2: resistor(20k)
            }
            nets {
                in -> R1.a
                R1.b -> C1.a  C1.b -> gnd
                R1.b -> R2.a  R2.b -> out
            }
            controls {}
        }"#,
    );

    let tree = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );

    let dyn_node = spqr_to_dyn_node(&tree, &graph, 48000.0).unwrap();
    let rp = dyn_node.port_resistance();

    // WDF discretized cap: rp_cap = 1/(2*fs*C) = 1/(2*48000*100e-9) ≈ 104.17
    // (WDF uses bilinear: Z = 1/(2*fs*C), not 1/(2π*fs*C))
    let zc_wdf = 1.0 / (2.0 * 48000.0 * 100e-9);
    let r2 = 20000.0;
    let r1 = 10000.0;
    let parallel = (r2 * zc_wdf) / (r2 + zc_wdf);
    let expected_rp = r1 + parallel;

    eprintln!("  rp={rp:.2}, expected={expected_rp:.2}, Zc_wdf={zc_wdf:.2}");
    let err = (rp - expected_rp).abs() / expected_rp;
    assert!(
        err < 0.01,
        "rp={rp:.2} doesn't match theory={expected_rp:.2} (err={:.1}%)",
        err * 100.0
    );
}
