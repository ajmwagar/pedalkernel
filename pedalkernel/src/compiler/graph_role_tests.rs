// TDD tests for unified GraphRole.
//
// Every component creates one Edge with two pins. No special variants.
// All other pin info comes from Component trait + node_names.

use super::graph::CircuitGraph;

fn make_graph(pedal_src: &str) -> CircuitGraph {
    let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse failed");
    CircuitGraph::from_pedal(&pedal)
}

#[test]
fn resistor_creates_one_edge() {
    let graph = make_graph(r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k) }
            nets { in -> R1.a  R1.b -> out }
            controls {}
        }"#);
    let edges: Vec<_> = (0..graph.edges.len())
        .filter(|&i| graph.components[graph.edges[i].comp_idx].id == "R1")
        .collect();
    assert_eq!(edges.len(), 1, "Resistor should create 1 edge");
}

#[test]
fn bjt_creates_one_edge() {
    let graph = make_graph(r#"
        pedal "test" { supply 9V
            components {
                Q1: npn(2n3904)
                R_c: resistor(10k)
                R_b: resistor(100k)
            }
            nets {
                in -> R_b.a
                R_b.b -> Q1.base
                Q1.collector -> R_c.a
                R_c.b -> vcc
                Q1.emitter -> gnd
                Q1.collector -> out
            }
            controls {}
        }"#);
    // BJT should have exactly 1 graph edge (collector-emitter)
    let bjt_edges: Vec<_> = (0..graph.edges.len())
        .filter(|&i| graph.components[graph.edges[i].comp_idx].id == "Q1")
        .collect();
    assert_eq!(bjt_edges.len(), 1, "BJT should create 1 edge");

    // Base should be resolvable via node_names
    assert!(
        graph.node_names.contains_key("Q1.base"),
        "Q1.base should be in node_names"
    );
}

#[test]
fn opamp_creates_one_edge() {
    let graph = make_graph(r#"
        pedal "test" { supply 9V
            components {
                U1: opamp(tl072)
                R1: resistor(10k)
                Rf: resistor(100k)
            }
            nets {
                in -> R1.a
                R1.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#);
    // Op-amp should have exactly 1 graph edge (neg-out)
    let opamp_edges: Vec<_> = (0..graph.edges.len())
        .filter(|&i| graph.components[graph.edges[i].comp_idx].id == "U1")
        .collect();
    assert_eq!(opamp_edges.len(), 1, "Op-amp should create 1 edge");

    // Pos should be resolvable via node_names
    assert!(
        graph.node_names.contains_key("U1.pos"),
        "U1.pos should be in node_names"
    );
}

#[test]
fn pot_3term_creates_one_edge_no_split() {
    // A 3-terminal pot should create 1 edge, NOT 2 synthetic __aw/__wb edges.
    let graph = make_graph(r#"
        pedal "test" { supply 9V
            components { Volume: pot(100k, a) }
            nets { in -> Volume.a  Volume.w -> out  Volume.b -> gnd }
            controls { Volume.position -> "Volume" [0.0, 1.0] = 0.5 }
        }"#);

    let pot_edges: Vec<_> = (0..graph.edges.len())
        .filter(|&i| {
            let id = &graph.components[graph.edges[i].comp_idx].id;
            id == "Volume" || id.starts_with("Volume__")
        })
        .collect();
    assert_eq!(
        pot_edges.len(), 1,
        "3-terminal pot should create 1 edge, got {} (IDs: {:?})",
        pot_edges.len(),
        pot_edges.iter().map(|&i| &graph.components[graph.edges[i].comp_idx].id).collect::<Vec<_>>()
    );

    // Wiper should be resolvable via node_names
    assert!(
        graph.node_names.contains_key("Volume.w") || graph.node_names.contains_key("Volume.wiper"),
        "Wiper should be in node_names"
    );

    // No synthetic __aw/__wb components should exist
    let has_synthetic = graph.components.iter().any(|c| {
        c.id.contains("__aw") || c.id.contains("__wb")
    });
    assert!(!has_synthetic, "No synthetic __aw/__wb components should exist");
}

#[test]
fn tube_creates_one_edge() {
    let graph = make_graph(r#"
        pedal "test" { supply 9V
            components {
                V1: triode(12ax7)
                R_p: resistor(100k)
                R_k: resistor(1.5k)
            }
            nets {
                in -> V1.grid
                V1.plate -> R_p.a
                R_p.b -> vcc
                V1.cathode -> R_k.a
                R_k.b -> gnd
                V1.plate -> out
            }
            controls {}
        }"#);
    let tube_edges: Vec<_> = (0..graph.edges.len())
        .filter(|&i| graph.components[graph.edges[i].comp_idx].id == "V1")
        .collect();
    assert_eq!(tube_edges.len(), 1, "Tube should create 1 edge");

    assert!(
        graph.node_names.contains_key("V1.grid"),
        "V1.grid should be in node_names"
    );
}

#[test]
fn no_nullor_pin_records_needed() {
    // With unified GraphRole, nullor_pins should be empty.
    // All pin info accessible via signal_terminals() + node_names.
    let graph = make_graph(r#"
        pedal "test" { supply 9V
            components {
                U1: opamp(tl072)
                R1: resistor(10k)
                Rf: resistor(100k)
            }
            nets {
                in -> R1.a
                R1.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#);

    // All op-amp pin info should be accessible without NullorPinRecord
    let comp = graph.components.iter().find(|c| c.id == "U1").unwrap();
    let terminals = comp.kind.signal_terminals();
    match terminals {
        super::component::SignalTerminals::Amplifier { input, output, control } => {
            assert_eq!(input, "neg");
            assert_eq!(output, "out");
            assert_eq!(control, Some("pos"));
            // All pins resolvable via node_names
            assert!(graph.node_names.contains_key("U1.neg"));
            assert!(graph.node_names.contains_key("U1.out"));
            assert!(graph.node_names.contains_key("U1.pos"));
        }
        _ => panic!("Op-amp should be Amplifier"),
    }
}
