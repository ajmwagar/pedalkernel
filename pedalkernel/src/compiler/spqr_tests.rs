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
    let (graph, edges) = make_graph(r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k) }
            nets { in -> R1.a  R1.b -> out }
            controls {}
        }"#);
    let result = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
    assert!(result.is_leaf(), "Single resistor should be Q node");
    assert_eq!(result.edge_count(), 1);
}

#[test]
fn spqr_series_two_resistors() {
    let (graph, edges) = make_graph(r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  R2: resistor(10k) }
            nets { in -> R1.a  R1.b -> R2.a  R2.b -> out }
            controls {}
        }"#);
    let result = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
    match &result {
        SpqrNode::S { children, .. } => {
            assert!(children.len() >= 2, "Series should have ≥2 children, got {}", children.len());
        }
        other => panic!("Expected S node for series resistors, got {:?}", std::mem::discriminant(other)),
    }
    assert_eq!(result.edge_count(), 2);
}

#[test]
fn spqr_parallel_two_resistors() {
    let (graph, edges) = make_graph(r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  R2: resistor(20k) }
            nets { in -> R1.a, R2.a  R1.b -> out  R2.b -> out }
            controls {}
        }"#);
    let result = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
    match &result {
        SpqrNode::P { children, .. } => {
            assert_eq!(children.len(), 2, "Parallel should have 2 children");
        }
        other => panic!("Expected P node for parallel resistors, got {:?}", std::mem::discriminant(other)),
    }
    assert_eq!(result.edge_count(), 2);
}

#[test]
fn spqr_wheatstone_bridge_is_rigid() {
    // Wheatstone bridge: 5 resistors, NOT series-parallel
    let (graph, edges) = make_graph(r#"
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
        }"#);
    let result = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
    assert!(result.is_rigid(), "Wheatstone bridge should be R node, got {:?}", std::mem::discriminant(&result));
    assert_eq!(result.edge_count(), 5, "Must preserve all 5 edges");
}

#[test]
fn spqr_rc_lowpass_is_series() {
    let (graph, edges) = make_graph(r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  C1: cap(100n) }
            nets { in -> R1.a  R1.b -> C1.a  C1.b -> out }
            controls {}
        }"#);
    let result = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
    match &result {
        SpqrNode::S { children, .. } => {
            assert!(children.len() >= 2, "RC lowpass should be series with ≥2 children");
        }
        other => panic!("Expected S node for RC lowpass, got {:?}", std::mem::discriminant(other)),
    }
}

#[test]
fn spqr_preserves_all_edges() {
    let (graph, edges) = make_graph(r#"
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
        }"#);
    let n_passive = edges.len();
    let result = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
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
    let (graph, edges) = make_graph(r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  C1: cap(100n) }
            nets { in -> R1.a  R1.b -> C1.a  C1.b -> out }
            controls {}
        }"#);
    let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
    let stages = spqr_to_stages(&tree, &graph, 48000.0);
    assert_eq!(stages.len(), 1, "Passive RC should produce 1 stage");
    assert!(
        matches!(&stages[0], SpqrStage::PassiveWdf { edge_indices, .. } if edge_indices.len() == 2),
        "Passive RC should be PassiveWdf with 2 edges, got {:?}", stages[0]
    );
}

#[test]
fn spqr_to_stages_parallel_rc_single_wdf() {
    let (graph, edges) = make_graph(r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  R2: resistor(20k) }
            nets { in -> R1.a, R2.a  R1.b -> out  R2.b -> out }
            controls {}
        }"#);
    let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
    let stages = spqr_to_stages(&tree, &graph, 48000.0);
    assert_eq!(stages.len(), 1, "Parallel R should produce 1 stage");
    assert!(
        matches!(&stages[0], SpqrStage::PassiveWdf { .. }),
        "Parallel R should be PassiveWdf, got {:?}", stages[0]
    );
}

#[test]
fn spqr_to_stages_bridge_becomes_rigid() {
    let (graph, edges) = make_graph(r#"
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
        }"#);
    let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
    let stages = spqr_to_stages(&tree, &graph, 48000.0);

    let has_rigid = stages.iter().any(|s| matches!(s, SpqrStage::Rigid { .. }));
    assert!(has_rigid, "Wheatstone bridge should produce a Rigid stage");
}

#[test]
fn spqr_to_stages_ordering_matches_traversal() {
    let (graph, edges) = make_graph(r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  R2: resistor(10k)  R3: resistor(10k) }
            nets { in -> R1.a  R1.b -> R2.a  R2.b -> R3.a  R3.b -> out }
            controls {}
        }"#);
    let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
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
            i - 1, get_order(&stages[i - 1]), i, get_order(&stages[i])
        );
    }
}

// ── Phase 2.5: classifier + NlWdf tests ────────────────────────

#[test]
fn classify_all_passive_rc() {
    let (graph, edges) = make_graph(r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  C1: cap(100n) }
            nets { in -> R1.a  R1.b -> C1.a  C1.b -> out }
            controls {}
        }"#);
    let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
    assert_eq!(classify_sp_subtree(&tree, &graph), SpClassification::AllPassive);
}

#[test]
fn classify_single_nl_diode_clipper() {
    // R in series with diode pair to ground — classic hard clipper
    let (graph, edges) = make_graph_all_edges(r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  D1: diode(silicon) }
            nets { in -> R1.a  R1.b -> D1.a  D1.b -> out }
            controls {}
        }"#);
    let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
    match classify_sp_subtree(&tree, &graph) {
        SpClassification::SingleNl { .. } => {} // correct
        other => panic!("Diode clipper should be SingleNl, got {:?}", other),
    }
}

#[test]
fn classify_complex_two_diodes() {
    let (graph, edges) = make_graph_all_edges(r#"
        pedal "test" { supply 9V
            components { D1: diode(silicon)  D2: diode(silicon) }
            nets { in -> D1.a  D1.b -> D2.a  D2.b -> out }
            controls {}
        }"#);
    let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
    assert_eq!(classify_sp_subtree(&tree, &graph), SpClassification::Complex);
}

#[test]
fn spqr_to_stages_diode_clipper_is_nl_wdf() {
    let (graph, edges) = make_graph_all_edges(r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  C1: cap(100n)  D1: diode(silicon) }
            nets { in -> R1.a  R1.b -> C1.a  C1.b -> D1.a  D1.b -> out }
            controls {}
        }"#);
    let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
    let stages = spqr_to_stages(&tree, &graph, 48000.0);
    assert_eq!(stages.len(), 1, "Diode clipper should produce 1 stage");
    match &stages[0] {
        SpqrStage::NlWdf { nl_edge_idx, edge_indices, tree, .. } => {
            assert_eq!(edge_indices.len(), 3, "Should have R + C + D");
            // The passive tree should have port_resistance for R + C
            let rp = tree.port_resistance();
            assert!(rp > 0.0, "Passive tree should have valid port resistance, got {rp}");
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
    let (graph, edges) = make_graph_all_edges(r#"
        pedal "test" { supply 9V
            components { R1: resistor(4.7k)  D1: diode(silicon) }
            nets { in -> R1.a  R1.b -> D1.a  D1.b -> out }
            controls {}
        }"#);
    let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
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
    let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
    assert_eq!(
        classify_sp_subtree(&tree, &graph),
        SpClassification::Vcvs,
        "Inverting op-amp should classify as Vcvs"
    );
}

#[test]
fn spqr_opamp_inverting_emits_rigid() {
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
    let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
    let stages = spqr_to_stages(&tree, &graph, 48000.0);

    assert_eq!(stages.len(), 1, "Inverting amp should produce 1 stage");
    match &stages[0] {
        SpqrStage::Rigid { edge_indices, pendant_trees, .. } => {
            // Rf and U1(VCVS) are in the MNA edge list (parallel, neg→out)
            // R1 is extracted as a pendant WDF tree (series, in→neg)
            assert_eq!(edge_indices.len(), 2, "MNA edges: Rf + U1(VCVS)");
            assert_eq!(pendant_trees.len(), 1, "R1 should be a pendant WDF port");
            // Verify the VCVS edge is present in MNA edges
            let has_vcvs = edge_indices.iter().any(|&eidx| {
                graph.effective_edge_kind(eidx) == super::component::EdgeKind::Vcvs
            });
            assert!(has_vcvs, "Rigid stage should contain the VCVS edge");
        }
        other => panic!("Inverting amp should be Rigid, got {:?}", other),
    }
}

#[test]
fn spqr_opamp_noninverting_emits_rigid() {
    // Non-inverting: signal to pos, voltage divider on neg
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
    let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
    let stages = spqr_to_stages(&tree, &graph, 48000.0);

    let has_rigid = stages.iter().any(|s| matches!(s, SpqrStage::Rigid { .. }));
    assert!(has_rigid, "Non-inverting amp should produce a Rigid stage");
}

#[test]
fn spqr_diode_opamp_shared_rigid() {
    // TS-style: diode clipper in op-amp feedback loop
    // All three (R1, D1, Rf, U1) should land in one Rigid stage
    let (graph, edges) = make_graph_all_edges(r#"
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
        }"#);
    let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
    let stages = spqr_to_stages(&tree, &graph, 48000.0);

    // Single Rigid: D1 + Rf + U1 in MNA, R1 as pendant WDF port
    assert_eq!(stages.len(), 1, "TS clipping should produce 1 stage");
    match &stages[0] {
        SpqrStage::Rigid { edge_indices, pendant_trees, .. } => {
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
    // Inverting amp with input RC filter: the RC should be
    // part of the same Rigid stage (connected to op-amp pins)
    let (graph, edges) = make_graph_all_edges(r#"
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
        }"#);
    let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
    let stages = spqr_to_stages(&tree, &graph, 48000.0);

    // All edges in one Rigid (VCVS infects the whole SP subtree)
    let rigid_count = stages.iter().filter(|s| matches!(s, SpqrStage::Rigid { .. })).count();
    assert!(rigid_count >= 1, "Should have at least one Rigid stage");

    // Total edges across all stages should account for C1 + R1 + Rf + U1
    let total_edges: usize = stages.iter().map(|s| match s {
        SpqrStage::PassiveWdf { edge_indices, .. }
        | SpqrStage::NlWdf { edge_indices, .. }
        | SpqrStage::Rigid { edge_indices, .. } => edge_indices.len(),
    }).sum();
    assert_eq!(total_edges, 4, "All 4 edges should be accounted for");
}

// ── Phase 2 continued ───────────────────────────────────────────

#[test]
fn spqr_dyn_node_series_has_correct_structure() {
    let (graph, edges) = make_graph(r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  R2: resistor(10k) }
            nets { in -> R1.a  R1.b -> R2.a  R2.b -> out }
            controls {}
        }"#);
    let tree = spqr_decompose(&edges, &[graph.in_node, graph.out_node], &graph, graph.gnd_node);
    let dyn_node = spqr_to_dyn_node(&tree, &graph, 48000.0);
    assert!(dyn_node.is_some(), "Series resistors should produce a DynNode");
    // The DynNode should have port_resistance = R1 + R2 = 20k
    let node = dyn_node.unwrap();
    let rp = node.port_resistance();
    assert!(
        (rp - 20_000.0).abs() < 100.0,
        "Series R1+R2 port resistance should be ~20k, got {rp:.0}"
    );
}
