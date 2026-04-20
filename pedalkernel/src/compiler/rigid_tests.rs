// Tests extracted from rigid_build.rs — StageStats + RigidOptimization classification.

use super::graph::CircuitGraph;
use super::rigid::*;

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

fn make_graph_passive(pedal_src: &str) -> (CircuitGraph, Vec<usize>) {
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
fn stats_passive_bridge_is_iir() {
    let (graph, edges) = make_graph_passive(r#"
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
    let stats = StageStats::from_edges(&edges, &graph);
    assert!(stats.is_all_linear());
    assert_eq!(stats.linear_count, 5);
    assert_eq!(classify_rigid(&stats, &graph), RigidOptimization::Iir);
}

#[test]
fn stats_inverting_opamp_is_opamproot() {
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
    let stats = StageStats::from_edges(&edges, &graph);
    assert_eq!(stats.vcvs_count, 1);
    assert_eq!(stats.nl_count, 0);
    assert_eq!(stats.linear_count, 2);
    assert_eq!(
        classify_rigid(&stats, &graph),
        RigidOptimization::OpAmpRoot { inverting: true }
    );
}

#[test]
fn stats_noninverting_opamp_is_opamproot() {
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
    let stats = StageStats::from_edges(&edges, &graph);
    assert_eq!(
        classify_rigid(&stats, &graph),
        RigidOptimization::OpAmpRoot { inverting: false }
    );
}

#[test]
fn stats_opamp_with_feedback_cap_is_opamproot() {
    let (graph, edges) = make_graph_all_edges(r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                Rf: resistor(100k)
                Cf: cap(100p)
                U1: opamp(tl072)
            }
            nets {
                in -> R1.a
                R1.b -> U1.neg
                Rf.a -> U1.neg
                Rf.b -> U1.out
                Cf.a -> U1.neg
                Cf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#);
    let stats = StageStats::from_edges(&edges, &graph);
    assert_eq!(stats.vcvs_count, 1);
    assert_eq!(stats.reactive_count, 1);
    // Single VCVS → OpAmpRoot (reactive feedback cap absorbed by GBW model)
    assert_eq!(classify_rigid(&stats, &graph), RigidOptimization::OpAmpRoot { inverting: true });
}

#[test]
fn stats_diode_in_rigid_is_general() {
    let (graph, edges) = make_graph_all_edges(r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                D1: diode(silicon)
                Rf: resistor(100k)
                U1: opamp(tl072)
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
    let stats = StageStats::from_edges(&edges, &graph);
    assert!(stats.nl_count > 0, "Should detect diode as NL");
    assert_eq!(classify_rigid(&stats, &graph), RigidOptimization::General);
}

#[test]
fn stats_counts_are_correct() {
    let (graph, edges) = make_graph_all_edges(r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                C1: cap(100n)
                D1: diode(silicon)
                U1: opamp(tl072)
            }
            nets {
                in -> R1.a
                R1.b -> C1.a
                C1.b -> U1.neg
                D1.a -> U1.neg
                D1.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#);
    let stats = StageStats::from_edges(&edges, &graph);
    assert_eq!(stats.total, 4, "R + C + D + U1");
    assert_eq!(stats.linear_count, 1, "R1");
    assert_eq!(stats.reactive_count, 1, "C1");
    assert_eq!(stats.nl_count, 1, "D1");
    assert_eq!(stats.vcvs_count, 1, "U1");
}
