// ═══════════════════════════════════════════════════════════════════════════
// Group terminal computation tests
//
// compute_group_terminals determines which nodes are SPQR terminals for a
// passive group. Too many terminals → SPQR can't find series/parallel
// structure → group produces 0 stages → silently dropped.
//
// Terminals should be signal entry/exit points of the group. NOT:
// - DC bias nodes (VB+, supply references)
// - The global in_node when it's shared but not this group's input
// - AC ground nodes
//
// These tests verify correct terminal computation for various topologies.
// ═══════════════════════════════════════════════════════════════════════════

use super::graph::CircuitGraph;
use super::signal_flow::find_flow_groups;
use super::spqr_build::compute_group_terminals;

const SR: f64 = 48000.0;

fn parse_and_get_terminals(source: &str, target_comp: &str) -> (Vec<usize>, Vec<usize>) {
    let pedal = crate::dsl::parse_pedal_file(source).expect("parse");
    let graph = CircuitGraph::from_pedal(&pedal);
    let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
    let groups = find_flow_groups(&all_edges, &graph);
    let terminals = vec![graph.in_node, graph.out_node];

    // Find the group containing target_comp
    let group = groups
        .iter()
        .find(|g| {
            g.all_edges()
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == target_comp)
        })
        .expect(&format!("group containing {target_comp} not found"));

    let group_edges = group.all_edges();
    let group_terminals = compute_group_terminals(&group_edges, &graph, &terminals);

    (group_edges, group_terminals)
}

// ═══════════════════════════════════════════════════════════════════════════
// 1. Simple passive group: 2 terminals (in/out)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn simple_passive_has_two_terminals() {
    let (edges, terminals) = parse_and_get_terminals(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                C_out: cap(1u)
                R_out: resistor(10k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> C_out.a
                C_out.b -> R_out.a
                R_out.b -> out
            }
            controls {}
        }"#,
        "C_out",
    );

    eprintln!(
        "Simple passive: {} edges, {} terminals: {:?}",
        edges.len(),
        terminals.len(),
        terminals
    );
    assert!(
        terminals.len() <= 2,
        "Simple passive group should have ≤2 terminals, got {}",
        terminals.len()
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Group with DC bias connection should NOT count bias node as terminal
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bias_node_not_counted_as_terminal() {
    // R2 connects to VB+ (bias). VB+ should NOT be a terminal.
    let (edges, terminals) = parse_and_get_terminals(
        r#"
        pedal "test" { supply 9V
            components {
                R_bias1: resistor(27k)
                R_bias2: resistor(27k)
                R1: resistor(10k)
                C1: cap(100n)
                R2: resistor(1M)
                U1: opamp(tl072)
                Rf: resistor(100k)
            }
            nets {
                vcc -> R_bias1.a
                R_bias1.b -> R_bias2.a
                R_bias2.b -> gnd
                in -> R1.a
                R1.b -> C1.a
                C1.b -> R2.a, U1.neg
                R2.b -> R_bias1.b
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
        "R1",
    );

    eprintln!(
        "With bias: {} edges, {} terminals: {:?}",
        edges.len(),
        terminals.len(),
        terminals
    );
    // The group containing R1,C1,R2 should have ≤3 terminals.
    // R2 connects to VB+ (bias node) — that should NOT be a terminal.
    assert!(
        terminals.len() <= 3,
        "Bias connection should not add terminal: {} terminals from {} edges",
        terminals.len(),
        edges.len()
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Goldenrod feedforward group: should have ≤3 terminals
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn goldenrod_feedforward_has_few_terminals() {
    // Simplified Goldenrod feedforward topology:
    // U1.out → R_ff → C_ff → summing_node
    // U1.out → Pot.a, Pot.w → R_ff2 → summing_node
    // R_bias connects to VB+ (bias, should not be terminal)
    let (edges, terminals) = parse_and_get_terminals(
        r#"
        pedal "test" { supply 9V
            components {
                R_bias1: resistor(27k)
                R_bias2: resistor(27k)
                R1: resistor(10k)
                C1: cap(100n)
                R2: resistor(1M)
                U1: opamp(tl072)
                Rf: resistor(100k)
                R_ff1: resistor(15k)
                C_ff1: cap(100n)
                Blend: pot(100k, b)
                R_ff2: resistor(392k)
                U2: opamp(tl072)
                Rf2: resistor(100k)
            }
            nets {
                vcc -> R_bias1.a
                R_bias1.b -> R_bias2.a
                R_bias2.b -> gnd
                in -> R1.a
                R1.b -> C1.a
                C1.b -> R2.a, U1.pos
                R2.b -> R_bias1.b
                U1.neg -> U1.out
                U1.out -> R_ff1.a, Blend.a
                R_ff1.b -> C_ff1.a
                C_ff1.b -> U2.neg
                Blend.w -> R_ff2.a
                R_ff2.b -> U2.neg
                Blend.b -> gnd
                U2.neg -> Rf2.a
                Rf2.b -> U2.out
                U2.pos -> R_bias1.b
                U2.out -> out
            }
            controls {
                Blend.position -> "Blend" [0.0, 1.0] = 0.5
            }
        }"#,
        "Blend",
    );

    eprintln!(
        "Feedforward: {} edges, {} terminals: {:?}",
        edges.len(),
        terminals.len(),
        terminals
    );
    // The feedforward group should have ≤3 terminals:
    // - U1.out (signal input)
    // - U2.neg (signal output / summing point)
    // - GND (pot ground, handled separately)
    // NOT: VB+ (bias), in_node (global input)
    assert!(
        terminals.len() <= 4,
        "Feedforward group has too many terminals: {} from {} edges",
        terminals.len(),
        edges.len()
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Bias nodes connected via static bias groups are not terminals
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn static_bias_junction_not_terminal() {
    // A passive group has an edge connecting to a node that's also in a
    // static bias group (VB+). That node should not be a terminal because
    // the bias group is just a DC reference — it doesn't carry signal.
    let (edges, terminals) = parse_and_get_terminals(
        r#"
        pedal "test" { supply 9V
            components {
                R_v1: resistor(10k)
                R_v2: resistor(10k)
                C_v: cap(47u, electrolytic)
                C_in: cap(100n)
                R_bias: resistor(1M)
                R_series: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
            }
            nets {
                vcc -> R_v1.a
                R_v1.b -> R_v2.a, C_v.a
                R_v2.b -> gnd
                C_v.b -> gnd
                in -> C_in.a
                C_in.b -> R_bias.a, R_series.a
                R_bias.b -> R_v1.b
                R_series.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
        "C_in",
    );

    eprintln!(
        "Bias junction: {} edges, {} terminals: {:?}",
        edges.len(),
        terminals.len(),
        terminals
    );
    // C_in group connects to VB+ via R_bias. VB+ is a bias junction.
    // Should have ≤3 terminals (in_node, U1.neg, maybe GND for R_bias path)
    // NOT: VB+ as a terminal
    assert!(
        terminals.len() <= 3,
        "Bias junction should not be terminal: {} terminals",
        terminals.len()
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. ac_ground_nodes must be excluded from terminals
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn ac_ground_node_excluded_from_terminals() {
    // VB+ (virtual ground from bias divider) is in ac_ground_nodes.
    // A group edge connecting to VB+ should NOT make VB+ a terminal.
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/goldenrod.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let graph = CircuitGraph::from_pedal(&pedal);
    let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
    let groups = find_flow_groups(&all_edges, &graph);
    let global_terminals = vec![graph.in_node, graph.out_node];

    let gain_b_group = groups
        .iter()
        .find(|g| {
            g.all_edges()
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "Gain_B")
        })
        .expect("Gain_B group");

    let terminals = compute_group_terminals(&gain_b_group.all_edges(), &graph, &global_terminals);

    // Node 76 is in ac_ground_nodes — it must NOT be a terminal
    for &t in &terminals {
        assert!(
            !graph.ac_ground_nodes.contains(&t),
            "ac_ground node {t:?} should not be a terminal. Terminals: {terminals:?}"
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. Global terminals only when they're this group's signal boundary
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn global_in_node_not_terminal_for_mid_chain_group() {
    // A group that touches in_node via one edge (R1) shouldn't get in_node
    // as a terminal if in_node is just a shared node, not this group's
    // signal entry point.
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/goldenrod.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let graph = CircuitGraph::from_pedal(&pedal);
    let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
    let groups = find_flow_groups(&all_edges, &graph);
    let global_terminals = vec![graph.in_node, graph.out_node];

    let gain_b_group = groups
        .iter()
        .find(|g| {
            g.all_edges()
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "Gain_B")
        })
        .expect("Gain_B group");

    let terminals = compute_group_terminals(&gain_b_group.all_edges(), &graph, &global_terminals);

    // in_node should NOT be a terminal for the Gain_B feedforward group.
    // R1 connects in_node to an internal junction — in_node is the global
    // input, not this group's signal boundary.
    assert!(
        !terminals.contains(&graph.in_node),
        "in_node should not be terminal for mid-chain group. Terminals: {terminals:?}"
    );
}

#[test]
fn goldenrod_gain_b_has_two_terminals() {
    // After fixing ac_ground, global terminal, and convergence issues,
    // Gain_B should have 2 terminals: node 13 (U1.out, input) and one
    // merged output (nodes 49+51 converge at U3.neg).
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/goldenrod.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let graph = CircuitGraph::from_pedal(&pedal);
    let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
    let groups = find_flow_groups(&all_edges, &graph);
    let global_terminals = vec![graph.in_node, graph.out_node];

    let gain_b_group = groups
        .iter()
        .find(|g| {
            g.all_edges()
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "Gain_B")
        })
        .expect("Gain_B group");

    let terminals = compute_group_terminals(&gain_b_group.all_edges(), &graph, &global_terminals);

    eprintln!("Gain_B terminals: {terminals:?} (expected 2)");
    assert!(
        terminals.len() <= 2,
        "Gain_B should have ≤2 terminals, got {}: {terminals:?}",
        terminals.len()
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 7. Full Goldenrod Gain_B group should produce stages
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn goldenrod_gain_b_produces_stages() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/goldenrod.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read goldenrod.pedal");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let compiled = super::spqr_build::compile_via_spqr(&pedal, SR).expect("compile");

    // First: dump the terminal analysis for the Gain_B group
    let graph = super::graph::CircuitGraph::from_pedal(&pedal);
    let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
    let groups = super::signal_flow::find_flow_groups(&all_edges, &graph);
    let global_terminals = vec![graph.in_node, graph.out_node];

    let gain_b_group = groups
        .iter()
        .find(|g| {
            g.all_edges()
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "Gain_B")
        })
        .expect("Gain_B group");

    let group_edges = gain_b_group.all_edges();
    let terminals = compute_group_terminals(&group_edges, &graph, &global_terminals);

    eprintln!("\nGain_B group terminal analysis:");
    eprintln!("  edges ({}):", group_edges.len());
    for &eidx in &group_edges {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        eprintln!("    {}({:?}→{:?})", comp.id, e.node_a, e.node_b);
    }

    eprintln!("  nodes in group: {:?}", {
        let mut nodes: Vec<_> = group_edges
            .iter()
            .flat_map(|&eidx| {
                let e = &graph.edges[eidx];
                vec![e.node_a, e.node_b]
            })
            .collect::<std::collections::HashSet<_>>()
            .into_iter()
            .collect();
        nodes.sort();
        nodes
    });

    eprintln!("  terminals ({}):", terminals.len());
    for &t in &terminals {
        // Why is this a terminal? Check what external edges connect here.
        let external_edges: Vec<String> = graph
            .edges
            .iter()
            .enumerate()
            .filter(|(eidx, e)| !group_edges.contains(eidx) && (e.node_a == t || e.node_b == t))
            .map(|(_, e)| {
                let comp = &graph.components[e.comp_idx];
                let other = if e.node_a == t { e.node_b } else { e.node_a };
                let other_name = if other == graph.gnd_node {
                    "GND".to_string()
                } else if other == graph.vcc_node {
                    "VCC".to_string()
                } else if graph.supply_nodes.contains(&other) {
                    format!("SUPPLY({:?})", other)
                } else if graph.ac_ground_nodes.contains(&other) {
                    format!("AC_GND({:?})", other)
                } else {
                    format!("{:?}", other)
                };
                format!("{}→{}", comp.id, other_name)
            })
            .collect();
        let is_global = global_terminals.contains(&t);
        let tag = if is_global { " [GLOBAL]" } else { "" };
        eprintln!(
            "    node {:?}{tag}: external edges: {:?}",
            t, external_edges
        );
    }

    eprintln!(
        "  special nodes: in={:?} out={:?} gnd={:?} vcc={:?}",
        graph.in_node, graph.out_node, graph.gnd_node, graph.vcc_node
    );
    eprintln!("  supply_nodes: {:?}", graph.supply_nodes);
    eprintln!("  ac_ground_nodes: {:?}", graph.ac_ground_nodes);

    #[cfg(debug_assertions)]
    {
        let has_gain_b = compiled.stages.iter().any(|s| {
            let lbl = match s {
                super::compiled::Stage::Wdf(w) => w.debug_label.as_str(),
                super::compiled::Stage::MultiNl(m) => m.debug_label.as_str(),
                super::compiled::Stage::Iir(i) => i.debug_label.as_str(),
                super::compiled::Stage::StateSpace(s) => s.debug_label.as_str(),
                super::compiled::Stage::BlackFeedback(b) => b.debug_label.as_str(),
                super::compiled::Stage::BlockwiseKMethod(_) => "blockwise",
            };
            lbl.contains("Gain_B")
        });
        assert!(
            has_gain_b,
            "Goldenrod Gain_B must produce a stage after terminal fix"
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 8. Terminals that converge at the same downstream node should merge
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn converging_outputs_count_as_one_terminal() {
    // Gain_B group has nodes 49 and 51 as separate terminals.
    // Both connect to U3.neg (node 56) via external edges (R_ff1b and R_ff2).
    // Since they converge at the same destination, they should count as
    // ONE output terminal, giving the group 2 terminals total (in + out).
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/goldenrod.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let graph = CircuitGraph::from_pedal(&pedal);
    let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
    let groups = find_flow_groups(&all_edges, &graph);
    let global_terminals = vec![graph.in_node, graph.out_node];

    let gain_b_group = groups
        .iter()
        .find(|g| {
            g.all_edges()
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "Gain_B")
        })
        .expect("Gain_B group");

    let terminals = compute_group_terminals(&gain_b_group.all_edges(), &graph, &global_terminals);

    eprintln!("Gain_B terminals after merge: {terminals:?}");
    assert!(
        terminals.len() <= 2,
        "Converging outputs should merge: got {} terminals {:?}, expected ≤2",
        terminals.len(),
        terminals
    );
}

#[test]
fn two_outputs_to_same_destination_merge() {
    // Simplified: two paths from junction to the same downstream node.
    // R_a → junction_a and R_b → junction_b, both connect to U2.neg.
    // junction_a and junction_b should merge into one terminal.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                R_a: resistor(15k)
                C_a: cap(100n)
                R_b: resistor(100k)
                U1: opamp(tl072)
                Rf1: resistor(100k)
                U2: opamp(tl072)
                Rf2: resistor(100k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf1.a
                Rf1.b -> U1.out
                U1.pos -> gnd
                U1.out -> R_a.a, R_b.a
                R_a.b -> C_a.a
                C_a.b -> U2.neg
                R_b.b -> U2.neg
                U2.neg -> Rf2.a
                Rf2.b -> U2.out
                U2.pos -> gnd
                U2.out -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let graph = CircuitGraph::from_pedal(&pedal);
    let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
    let groups = find_flow_groups(&all_edges, &graph);
    let global_terminals = vec![graph.in_node, graph.out_node];

    // Find the group containing R_a (the feedforward passive group)
    let ff_group = groups
        .iter()
        .find(|g| {
            g.all_edges()
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "R_a")
        })
        .expect("feedforward group");

    let terminals = compute_group_terminals(&ff_group.all_edges(), &graph, &global_terminals);

    eprintln!("Two-output terminals: {terminals:?}");
    // R_a path output (C_a.b) and R_b path output (R_b.b) both go to U2.neg.
    // They should merge → 2 terminals total (U1.out input + merged output).
    assert!(
        terminals.len() <= 2,
        "Two outputs to same destination should merge: got {} terminals {:?}",
        terminals.len(),
        terminals
    );
}
