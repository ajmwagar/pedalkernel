// TDD tests for port-based graph construction.
//
// Every component declares its ports (terminal pairs carrying current).
// The graph creates one edge per port, all sharing the same comp_idx.
// No synthetic splits, no virtual bridges, no special-case GraphRole variants.

use super::graph::CircuitGraph;

fn make_graph(pedal_src: &str) -> CircuitGraph {
    let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse failed");
    CircuitGraph::from_pedal(&pedal)
}

fn edges_for_component(graph: &CircuitGraph, comp_id: &str) -> Vec<usize> {
    (0..graph.edges.len())
        .filter(|&i| {
            let id = &graph.components[graph.edges[i].comp_idx].id;
            id == comp_id || id.starts_with(&format!("{comp_id}__"))
        })
        .collect()
}

// ═══════════════════════════════════════════════════════════════════════════
// 1-port components: one terminal pair, one edge
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn resistor_one_port_one_edge() {
    let graph = make_graph(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k) }
            nets { in -> R1.a  R1.b -> out }
            controls {}
        }"#,
    );
    assert_eq!(edges_for_component(&graph, "R1").len(), 1);
}

#[test]
fn diode_one_port_one_edge() {
    let graph = make_graph(
        r#"
        pedal "test" { supply 9V
            components { D1: diode(silicon) }
            nets { in -> D1.a  D1.b -> out }
            controls {}
        }"#,
    );
    assert_eq!(edges_for_component(&graph, "D1").len(), 1);
}

#[test]
fn opamp_one_port_one_edge() {
    // Op-amp: 1 port (neg→out). Pos is voltage-sense, no current.
    let graph = make_graph(
        r#"
        pedal "test" { supply 9V
            components { U1: opamp(tl072)  R1: resistor(10k)  Rf: resistor(100k) }
            nets {
                in -> R1.a  R1.b -> U1.neg  U1.neg -> Rf.a  Rf.b -> U1.out
                U1.pos -> gnd  U1.out -> out
            }
            controls {}
        }"#,
    );
    // Count REAL edges for U1 (exclude virtual bridges with comp_idx=0 placeholder)
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let opamp_edges: Vec<_> = edges_for_component(&graph, "U1")
        .into_iter()
        .filter(|i| !active_set.contains(i))
        .collect();
    assert_eq!(
        opamp_edges.len(),
        1,
        "Op-amp has 1 real port (neg→out), got {}",
        opamp_edges.len()
    );
    // Pos pin accessible via node_names
    assert!(graph.node_names.contains_key("U1.pos"));
}

// ═══════════════════════════════════════════════════════════════════════════
// 2-port components: two terminal pairs, two edges, same comp_idx
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bjt_two_ports_two_edges() {
    // BJT: 2 ports (base→emitter, collector→emitter)
    let graph = make_graph(
        r#"
        pedal "test" { supply 9V
            components { Q1: npn(2n3904)  R_c: resistor(10k)  R_b: resistor(100k) }
            nets {
                in -> R_b.a  R_b.b -> Q1.base
                Q1.collector -> R_c.a  R_c.b -> vcc
                Q1.emitter -> gnd  Q1.collector -> out
            }
            controls {}
        }"#,
    );
    let bjt_edges = edges_for_component(&graph, "Q1");
    assert_eq!(
        bjt_edges.len(),
        2,
        "BJT has 2 ports (B-E + C-E), got {}",
        bjt_edges.len()
    );
    // Both edges share the same comp_idx (Q1)
    let comp_idx_0 = graph.edges[bjt_edges[0]].comp_idx;
    let comp_idx_1 = graph.edges[bjt_edges[1]].comp_idx;
    assert_eq!(comp_idx_0, comp_idx_1, "Both BJT ports same comp_idx");
    // All pins in node_names
    assert!(graph.node_names.contains_key("Q1.base"));
    assert!(graph.node_names.contains_key("Q1.collector"));
    assert!(graph.node_names.contains_key("Q1.emitter"));
}

#[test]
fn pot_3term_two_ports_two_edges() {
    // Pot: 2 ports (a→wiper, wiper→b). Same comp_idx, no __aw/__wb split.
    let graph = make_graph(
        r#"
        pedal "test" { supply 9V
            components { Volume: pot(100k, a) }
            nets { in -> Volume.a  Volume.w -> out  Volume.b -> gnd }
            controls { Volume.position -> "Volume" [0.0, 1.0] = 0.5 }
        }"#,
    );
    let pot_edges = edges_for_component(&graph, "Volume");
    assert_eq!(
        pot_edges.len(),
        2,
        "Pot has 2 ports (a-w + w-b), got {} (IDs: {:?})",
        pot_edges.len(),
        pot_edges
            .iter()
            .map(|&i| &graph.components[graph.edges[i].comp_idx].id)
            .collect::<Vec<_>>()
    );
    // Both use original comp_idx — no synthetic __aw/__wb
    let has_synthetic = graph
        .components
        .iter()
        .any(|c| c.id.contains("__aw") || c.id.contains("__wb"));
    assert!(!has_synthetic, "No synthetic __aw/__wb components");
    // Wiper in node_names
    assert!(
        graph.node_names.contains_key("Volume.w") || graph.node_names.contains_key("Volume.wiper"),
        "Wiper in node_names"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Component trait: ports() method
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn component_ports_resistor() {
    use super::component::Component;
    use super::components::Resistor;
    let r = Resistor { value: 10_000.0 };
    assert_eq!(r.ports(), vec![("a", "b")]);
}

#[test]
fn component_ports_bjt() {
    use super::component::Component;
    use super::components::Npn;
    let q = Npn {
        model: "2n3904".to_string(),
    };
    let ports = q.ports();
    assert_eq!(ports.len(), 2);
    assert!(ports.contains(&("base", "emitter")));
    assert!(ports.contains(&("collector", "emitter")));
}

#[test]
fn component_ports_opamp() {
    use super::component::Component;
    use super::components::OpAmp;
    use crate::dsl::OpAmpType;
    let u = OpAmp {
        op_type: OpAmpType::Tl072,
    };
    assert_eq!(u.ports(), vec![("neg", "out")]);
}

#[test]
fn component_ports_pot() {
    use super::component::Component;
    use super::components::Potentiometer;
    use crate::dsl::PotTaper;
    let p = Potentiometer {
        max_r: 100_000.0,
        taper: PotTaper::A,
    };
    let ports = p.ports();
    // 2-terminal pot: 1 port. 3-terminal pot: 2 ports.
    // The Component doesn't know if wiper is connected — that's determined
    // by the circuit nets. Default: 2 ports (a→w, w→b).
    assert_eq!(ports.len(), 2, "Pot default: 2 ports (a-w, w-b)");
}

// ═══════════════════════════════════════════════════════════════════════════
// No virtual bridge edges needed
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn no_virtual_bridge_edges() {
    // With real multi-port edges, virtual bridges are redundant.
    let graph = make_graph(
        r#"
        pedal "test" { supply 9V
            components { Q1: npn(2n3904)  R_c: resistor(10k)  R_b: resistor(100k) }
            nets {
                in -> R_b.a  R_b.b -> Q1.base
                Q1.collector -> R_c.a  R_c.b -> vcc
                Q1.emitter -> gnd  Q1.collector -> out
            }
            controls {}
        }"#,
    );
    // active_edge_indices should be empty — real edges provide connectivity
    assert!(
        graph.active_edge_indices.is_empty(),
        "No virtual bridge edges needed, got {}",
        graph.active_edge_indices.len()
    );
}
