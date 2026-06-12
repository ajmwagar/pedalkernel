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
    let (graph, edges) = make_graph_passive(
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
    let stats = StageStats::from_edges(&edges, &graph);
    assert!(stats.is_all_linear());
    assert_eq!(stats.linear_count, 5);
    assert_eq!(classify_rigid(&stats, &graph, None), RigidOptimization::Iir);
}

#[test]
fn stats_inverting_opamp_is_black_feedback() {
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
    let stats = StageStats::from_edges(&edges, &graph);
    assert_eq!(stats.vcvs_count, 1);
    assert_eq!(stats.nl_count, 0);
    assert_eq!(stats.linear_count, 2);
    assert_eq!(
        classify_rigid(&stats, &graph, None),
        RigidOptimization::BlackFeedback
    );
}

#[test]
fn stats_noninverting_opamp_is_black_feedback() {
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
    let stats = StageStats::from_edges(&edges, &graph);
    assert_eq!(
        classify_rigid(&stats, &graph, None),
        RigidOptimization::BlackFeedback
    );
}

#[test]
fn stats_opamp_with_feedback_cap_is_black_feedback() {
    let (graph, edges) = make_graph_all_edges(
        r#"
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
        }"#,
    );
    let stats = StageStats::from_edges(&edges, &graph);
    assert_eq!(stats.vcvs_count, 1);
    assert_eq!(stats.reactive_count, 1);
    // Single VCVS + reactive → BlackFeedback (closed-form Rf/Ri + NonIdealFx)
    assert_eq!(
        classify_rigid(&stats, &graph, None),
        RigidOptimization::BlackFeedback
    );
}

#[test]
fn stats_diode_in_rigid_is_general() {
    let (graph, edges) = make_graph_all_edges(
        r#"
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
        }"#,
    );
    let stats = StageStats::from_edges(&edges, &graph);
    assert!(stats.nl_count > 0, "Should detect diode as NL");
    assert_eq!(
        classify_rigid(&stats, &graph, None),
        RigidOptimization::General
    );
}

#[test]
fn stats_counts_are_correct() {
    let (graph, edges) = make_graph_all_edges(
        r#"
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
        }"#,
    );
    let stats = StageStats::from_edges(&edges, &graph);
    assert_eq!(stats.total, 4, "R + C + D + U1");
    assert_eq!(stats.linear_count, 1, "R1");
    assert_eq!(stats.reactive_count, 1, "C1");
    assert_eq!(stats.nl_count, 1, "D1");
    assert_eq!(stats.vcvs_count, 1, "U1");
}

// ═══════════════════════════════════════════════════════════════════════════
// is_inverting_topology: classifying op-amp signal input
// ═══════════════════════════════════════════════════════════════════════════

fn classify_inverting(pedal_src: &str) -> bool {
    let (graph, edges) = make_graph_all_edges(pedal_src);
    let stats = StageStats::from_edges(&edges, &graph);
    is_inverting_topology(&stats, &graph)
}

#[test]
fn inverting_pos_to_gnd() {
    // Simplest inverting: pos → GND directly.
    let src = r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#;

    let pedal = crate::dsl::parse_pedal_file(src).expect("parse");
    let graph = CircuitGraph::from_pedal(&pedal);
    let r_in_edges: Vec<_> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(_, e)| graph.components[e.comp_idx].id == "R_in")
        .map(|(i, e)| (i, e.node_a, e.node_b))
        .collect();
    eprintln!(
        "Simple inverting — R_in edges: {r_in_edges:?} (total edges: {})",
        graph.edges.len()
    );
    assert_eq!(
        r_in_edges.len(),
        1,
        "R_in should have exactly 1 edge in simple circuit too: {:?}",
        r_in_edges
    );

    assert!(classify_inverting(src), "pos→GND should be inverting");
}

#[test]
fn noninverting_pos_to_signal() {
    // Non-inverting: pos receives signal, neg has Ri→GND + Rf→out.
    assert!(
        !classify_inverting(
            r#"
        pedal "test" { supply 9V
            components {
                U1: opamp(tl072)
                Rf: resistor(100k)
                Ri: resistor(10k)
            }
            nets {
                in -> U1.pos
                U1.neg -> Ri.a
                Ri.b -> gnd
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.out -> out
            }
            controls {}
        }"#
        ),
        "pos→signal should be non-inverting"
    );
}

#[test]
fn inverting_pos_to_bias_with_cap() {
    // Inverting with Vref bias: pos → R-R divider + bypass cap to GND.
    // The cap makes pos an AC ground → inverting.
    // This is the Screamer / RAT / SD-1 pattern.
    let src = r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                R_bias1: resistor(100k)
                R_bias2: resistor(100k)
                C_bias: cap(10u)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                vcc -> R_bias1.a
                R_bias1.b -> U1.pos
                U1.pos -> R_bias2.a
                R_bias2.b -> gnd
                U1.pos -> C_bias.a
                C_bias.b -> gnd
                U1.out -> out
            }
            controls {}
        }"#;

    // Dump all graph edges to understand phantom edges
    let pedal = crate::dsl::parse_pedal_file(src).expect("parse");
    let graph = CircuitGraph::from_pedal(&pedal);
    eprintln!(
        "Graph: {} edges, {} components",
        graph.edges.len(),
        graph.components.len()
    );
    eprintln!(
        "  in_node={} out_node={} gnd={} vcc={}",
        graph.in_node, graph.out_node, graph.gnd_node, graph.vcc_node
    );
    for (i, e) in graph.edges.iter().enumerate() {
        let c = &graph.components[e.comp_idx];
        eprintln!(
            "  edge[{i}] comp[{}]={} ({}->{}) passive={}",
            e.comp_idx,
            c.id,
            e.node_a,
            e.node_b,
            c.kind.is_passive()
        );
    }

    // Edge 6 and 7 are PHANTOM edges — R_in should only have 1 edge (0→5),
    // not 3. This is a graph construction bug from nullor model stamping.
    let r_in_edges: Vec<_> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(_, e)| e.comp_idx == 0) // comp[0] = R_in
        .map(|(i, e)| (i, e.node_a, e.node_b))
        .collect();
    eprintln!("  R_in edges: {r_in_edges:?}");
    assert_eq!(
        r_in_edges.len(),
        1,
        "R_in should have exactly 1 edge, got {}: {:?}",
        r_in_edges.len(),
        r_in_edges
    );

    assert!(
        classify_inverting(src),
        "pos→bias(R+R+C) should be inverting"
    );
}

#[test]
fn inverting_pos_to_bias_no_cap() {
    // Inverting with Vref bias but NO bypass cap.
    // Without the cap, pos is NOT at AC ground — the bias divider
    // has finite impedance to both VCC and GND.
    // This is ambiguous but should still be inverting (signal enters at neg).
    assert!(
        classify_inverting(
            r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                R_bias1: resistor(100k)
                R_bias2: resistor(100k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                vcc -> R_bias1.a
                R_bias1.b -> U1.pos
                U1.pos -> R_bias2.a
                R_bias2.b -> gnd
                U1.out -> out
            }
            controls {}
        }"#
        ),
        "pos→bias(R+R, no cap) should still be inverting"
    );
}

#[test]
fn noninverting_pos_to_signal_with_bias_on_neg() {
    // Non-inverting where neg has feedback to out AND Ri to GND.
    // pos receives signal. Should NOT be classified as inverting
    // just because neg touches GND through Ri.
    assert!(
        !classify_inverting(
            r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                Ri: resistor(10k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.pos
                U1.neg -> Ri.a
                Ri.b -> gnd
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.out -> out
            }
            controls {}
        }"#
        ),
        "pos→signal (with Ri on neg) should be non-inverting"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Unity follower node union: pos and out should be the same node
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn unity_follower_pos_equals_out() {
    // A unity follower (U2.out -> U2.neg) is electrically a wire from
    // pos to out. The graph should union pos and out into the same node.
    let (graph, _) = make_graph_all_edges(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                U2: opamp(tl072)
                C_out: cap(10u)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> U2.pos
                U2.out -> U2.neg
                U2.out -> C_out.a
                C_out.b -> out
            }
            controls {}
        }"#,
    );

    // Find U2's nullor pins
    let u2_pins = graph
        .nullor_pins
        .iter()
        .find(|p| graph.components[p.comp_idx].id == "U2")
        .expect("Should find U2 nullor pins");

    eprintln!(
        "U2: pos={} neg={} out={}",
        u2_pins.pos_node, u2_pins.neg_node, u2_pins.out_node
    );

    // For a unity follower, pos_node and out_node should be the same
    // (unioned during graph construction)
    assert_eq!(
        u2_pins.pos_node, u2_pins.out_node,
        "Unity follower: pos and out should be the same node (pos={}, out={})",
        u2_pins.pos_node, u2_pins.out_node
    );
}

#[test]
fn unity_follower_does_not_union_inverting_opamp() {
    // An inverting op-amp (NOT a follower) should keep pos and out separate.
    let (graph, _) = make_graph_all_edges(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    );

    let u1_pins = graph
        .nullor_pins
        .iter()
        .find(|p| graph.components[p.comp_idx].id == "U1")
        .expect("Should find U1 nullor pins");

    // Inverting amp: pos (GND) and out should NOT be unioned
    assert_ne!(
        u1_pins.pos_node, u1_pins.out_node,
        "Inverting amp: pos and out should be different nodes"
    );
}

#[test]
fn unity_follower_signal_passes_through() {
    // With node union, signal should flow through the follower naturally.
    // U1 (gain=10) → U2 (follower) → out. Output should be ~10x input.
    use super::spqr_build::compile_via_spqr;
    use crate::PedalProcessor;

    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                U2: opamp(tl072)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> U2.pos
                U2.out -> U2.neg
                U2.out -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    // Warmup
    for s in 0..500 {
        let input = 0.01 * (std::f64::consts::TAU * 440.0 * s as f64 / 48000.0).sin();
        compiled.process(input);
    }
    let mut peak = 0.0f64;
    for s in 500..720 {
        let input = 0.01 * (std::f64::consts::TAU * 440.0 * s as f64 / 48000.0).sin();
        peak = peak.max(compiled.process(input).abs());
    }

    eprintln!("Unity follower passthrough: peak={peak:.6}");
    assert!(
        peak > 0.01,
        "Signal should pass through unity follower: peak={peak:.6}"
    );
}

#[test]
fn inverting_second_stage_with_bias() {
    // Two-stage pedal: U1 (inverting, pos→GND) → U2 (inverting, pos→Vref).
    // Both should classify as inverting. U2's pos connects to a bias divider,
    // NOT to the circuit input. This is the Screamer/SD-1 pattern.
    //
    // The fix must work per-stage (not BFS from global in_node) because
    // U2's signal enters through the serial chain, not from in_node.
    let (graph, edges) = make_graph_all_edges(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf1: resistor(100k)
                R_mid: resistor(10k)
                U2: opamp(tl072)
                Rf2: resistor(47k)
                R_bias1: resistor(100k)
                R_bias2: resistor(100k)
                C_bias: cap(10u)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf1.a
                Rf1.b -> U1.out
                U1.pos -> gnd
                U1.out -> R_mid.a
                R_mid.b -> U2.neg
                U2.neg -> Rf2.a
                Rf2.b -> U2.out
                vcc -> R_bias1.a
                R_bias1.b -> U2.pos
                U2.pos -> R_bias2.a
                R_bias2.b -> gnd
                U2.pos -> C_bias.a
                C_bias.b -> gnd
                U2.out -> out
            }
            controls {}
        }"#,
    );

    // Test U1 (pos→GND)
    let u1_edges: Vec<usize> = edges
        .iter()
        .copied()
        .filter(|&eidx| {
            let comp = &graph.components[graph.edges[eidx].comp_idx];
            // U1's group: R_in, U1, Rf1
            comp.id == "R_in" || comp.id == "U1" || comp.id == "Rf1"
        })
        .collect();
    let stats1 = StageStats::from_edges(&u1_edges, &graph);
    assert!(
        is_inverting_topology(&stats1, &graph),
        "U1 (pos→GND) should be inverting"
    );

    // Test U2 (pos→Vref bias)
    let u2_edges: Vec<usize> = edges
        .iter()
        .copied()
        .filter(|&eidx| {
            let comp = &graph.components[graph.edges[eidx].comp_idx];
            comp.id == "R_mid" || comp.id == "U2" || comp.id == "Rf2"
        })
        .collect();
    let stats2 = StageStats::from_edges(&u2_edges, &graph);
    assert!(
        is_inverting_topology(&stats2, &graph),
        "U2 (pos→bias divider) should be inverting in second stage"
    );
}
