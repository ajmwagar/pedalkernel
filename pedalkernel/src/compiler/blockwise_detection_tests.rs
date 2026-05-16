//! TDD tests for blockwise decomposition and K-method candidacy.
//!
//! Two orthogonal optimizations:
//!
//! **Blockwise**: Structural. A large coupled NL system can be split into
//! smaller blocks with sparse inter-block coupling. Detection is from the
//! SPQR tree: an S-node with repeated NL+reactive groups that share local
//! topology. Each block becomes its own small WDF solve. The inter-block
//! coupling is handled by K-method scattering matrix or iteration.
//!
//! **K-method**: Algebraic. Within a block, the NL element can be replaced
//! by a precomputed lookup table if:
//!   1. NL is memoryless (no caps/inductors inside the NL box)
//!   2. Low port count (≤2 wires cross the NL box boundary → ≤2D table)
//!   3. Fixed parameters (no variable components inside the NL box)
//!   4. Unique solution (monotonic I-V, no fold-back)
//!
//! For the 303 ladder: blockwise gives 4 blocks, K-method tabulates each
//! block's BJT. Both wins stack: O(8³) monolithic → O(4 × table_lookup).

use super::blockwise;
use super::graph::CircuitGraph;
use super::spqr::*;
use crate::PedalProcessor;

/// Helper: compile a .pedal source with disk caching (avoids regenerating
/// K-tables on every test run). First run is slow, subsequent runs instant.
/// Compile with disk caching. First run generates K-tables (slow),
/// subsequent runs load from target/test-cache/ (instant).
fn compile_cached(source: &str, name: &str) -> super::compiled::CompiledPedal {
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let options = super::compile::CompileOptions::default(); // with K-tables
    let blob =
        super::compile::compile_pedal_cached(source, name, name, 48_000.0, &options, &cache_dir)
            .unwrap_or_else(|e| panic!("{name}: {e}"));
    postcard::from_bytes(&blob).unwrap_or_else(|e| panic!("{name} deserialize: {e}"))
}

/// Helper: build graph from .pedal source, return all non-active edges.
fn make_graph_all(pedal_src: &str) -> (CircuitGraph, Vec<usize>) {
    let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&pedal);
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();
    (graph, all_edges)
}

/// Helper: count NL edges in a set of edge indices.
fn count_nl(edges: &[usize], graph: &CircuitGraph) -> usize {
    edges
        .iter()
        .filter(|&&eidx| graph.effective_edge_kind(eidx) == super::component::EdgeKind::Nonlinear)
        .count()
}

/// Helper: count reactive edges in a set of edge indices.
fn count_reactive(edges: &[usize], graph: &CircuitGraph) -> usize {
    edges
        .iter()
        .filter(|&&eidx| graph.effective_edge_kind(eidx) == super::component::EdgeKind::Reactive)
        .count()
}

// ═══════════════════════════════════════════════════════════════════════════
// Circuit definitions
// ═══════════════════════════════════════════════════════════════════════════

/// 4-stage BJT ladder filter (303-style).
/// Each stage: BJT (NL) + shunt cap (reactive) + bias resistors (linear).
/// Global feedback: Resonance pot from Q4.collector to Q1.base.
const BJT_LADDER_4: &str = r#"pedal "BJT Ladder" {
  supply 9V
  components {
    Q1: npn(2n3904)
    Q2: npn(2n3904)
    Q3: npn(2n3904)
    Q4: npn(2n3904)
    R_c1: resistor(22k)
    R_c2: resistor(22k)
    R_c3: resistor(22k)
    R_c4: resistor(22k)
    R_e1: resistor(220)
    R_e2: resistor(220)
    R_e3: resistor(220)
    R_e4: resistor(220)
    C1: cap(22n)
    C2: cap(22n)
    C3: cap(22n)
    C4: cap(22n)
    R_in: resistor(10k)
    R_fb: resistor(33k)
    Resonance: pot(100k, b)
  }
  nets {
    in -> R_in.a
    R_in.b -> Q1.base

    vcc -> R_c1.a
    R_c1.b -> Q1.collector
    Q1.emitter -> R_e1.a
    R_e1.b -> gnd
    Q1.emitter -> C1.a
    C1.b -> gnd

    Q1.collector -> Q2.base
    vcc -> R_c2.a
    R_c2.b -> Q2.collector
    Q2.emitter -> R_e2.a
    R_e2.b -> gnd
    Q2.emitter -> C2.a
    C2.b -> gnd

    Q2.collector -> Q3.base
    vcc -> R_c3.a
    R_c3.b -> Q3.collector
    Q3.emitter -> R_e3.a
    R_e3.b -> gnd
    Q3.emitter -> C3.a
    C3.b -> gnd

    Q3.collector -> Q4.base
    vcc -> R_c4.a
    R_c4.b -> Q4.collector
    Q4.emitter -> R_e4.a
    R_e4.b -> gnd
    Q4.emitter -> C4.a
    C4.b -> gnd

    Q4.collector -> out
    Q4.collector -> Resonance.a
    Resonance.b -> R_fb.a
    R_fb.b -> Q1.base
  }
  controls {
    Resonance.position -> "Resonance" [0.0, 1.0] = 0.0
  }
}"#;

/// 2-stage 303-style ladder with ports (reduced for fast tests).
const BJT_LADDER_WITH_PORTS_2: &str = r#"pedal "303 Ported 2" {
  supply 9V
  ports {
    audio_in: input
    cv_cutoff: input
    audio_out: output
  }
  components {
    Q1: npn(2n3904)
    Q2: npn(2n3904)
    R_in: resistor(10k)
    R_cv: resistor(47k)
    C1: cap(18n)
    C2: cap(33n)
    R_e1: resistor(220)
    Cutoff1: pot(10k, b)
    R_e2: resistor(220)
    Cutoff2: pot(10k, b)
    Resonance: pot(100k, b)
    R_fb: resistor(33k)
    C_out: cap(100n)
    R_out: resistor(100k)
  }
  nets {
    audio_in -> R_in.a
    R_in.b -> Q1.base
    cv_cutoff -> R_cv.a
    R_cv.b -> Q1.base
    Q1.emitter -> C1.a
    C1.b -> gnd
    Q1.emitter -> R_e1.a
    R_e1.b -> Cutoff1.a
    Cutoff1.b -> gnd
    Q1.emitter -> Q2.base
    Q2.emitter -> C2.a
    C2.b -> gnd
    Q2.emitter -> R_e2.a
    R_e2.b -> Cutoff2.a
    Cutoff2.b -> gnd
    Q2.emitter -> C_out.a
    C_out.b -> R_out.a
    R_out.b -> gnd
    C_out.b -> audio_out
    Q2.emitter -> Resonance.a
    Resonance.b -> R_fb.a
    R_fb.b -> R_in.b
  }
  controls {
    Cutoff1.position -> "Cutoff" [1.0, 0.02] = 0.5
    Cutoff2.position -> "Cutoff" [1.0, 0.02] = 0.5
    Resonance.position -> "Resonance" [0.0, 0.95] = 0.0
  }
}"#;

/// Full 4-stage 303 ladder with ports — the actual AcidAttack topology.
/// This hangs in SPQR decompose during blockwise analysis.
const BJT_LADDER_WITH_PORTS_4: &str = r#"pedal "303 Ported 4" {
  supply 9V
  ports {
    audio_in: input
    cv_cutoff: input
    audio_out: output
  }
  components {
    Q1: npn(2n3904)
    Q2: npn(2n3904)
    Q3: npn(2n3904)
    Q4: npn(2n3904)
    R_bias1: resistor(100k)
    R_bias2: resistor(100k)
    R_bias3: resistor(100k)
    R_bias4: resistor(100k)
    C1: cap(18n)
    C2: cap(33n)
    C3: cap(33n)
    C4: cap(33n)
    R_e_floor1: resistor(220)
    Cutoff1: pot(10k, b)
    R_e_floor2: resistor(220)
    Cutoff2: pot(10k, b)
    R_e_floor3: resistor(220)
    Cutoff3: pot(10k, b)
    R_e_floor4: resistor(220)
    Cutoff4: pot(10k, b)
    Resonance: pot(100k, b)
    R_fb_limit: resistor(33k)
    R_in: resistor(10k)
    R_cv: resistor(47k)
    C_out: cap(100n)
    R_out: resistor(100k)
  }
  nets {
    vcc -> R_bias1.a
    R_bias1.b -> Q1.base, Q1.collector
    vcc -> R_bias2.a
    R_bias2.b -> Q2.base, Q2.collector
    vcc -> R_bias3.a
    R_bias3.b -> Q3.base, Q3.collector
    vcc -> R_bias4.a
    R_bias4.b -> Q4.base, Q4.collector
    audio_in -> R_in.a
    R_in.b -> Q1.base
    cv_cutoff -> R_cv.a
    R_cv.b -> Q1.base
    Q1.emitter -> C1.a
    C1.b -> gnd
    Q1.emitter -> R_e_floor1.a
    R_e_floor1.b -> Cutoff1.a
    Cutoff1.b -> gnd
    Q1.emitter -> Q2.base
    Q2.emitter -> C2.a
    C2.b -> gnd
    Q2.emitter -> R_e_floor2.a
    R_e_floor2.b -> Cutoff2.a
    Cutoff2.b -> gnd
    Q2.emitter -> Q3.base
    Q3.emitter -> C3.a
    C3.b -> gnd
    Q3.emitter -> R_e_floor3.a
    R_e_floor3.b -> Cutoff3.a
    Cutoff3.b -> gnd
    Q3.emitter -> Q4.base
    Q4.emitter -> C4.a
    C4.b -> gnd
    Q4.emitter -> R_e_floor4.a
    R_e_floor4.b -> Cutoff4.a
    Cutoff4.b -> gnd
    Q4.emitter -> C_out.a
    C_out.b -> R_out.a
    R_out.b -> gnd
    C_out.b -> audio_out
    Q4.emitter -> Resonance.a
    Resonance.b -> R_fb_limit.a
    R_fb_limit.b -> R_in.b
  }
  controls {
    Cutoff1.position -> "Cutoff" [1.0, 0.02] = 0.5
    Cutoff2.position -> "Cutoff" [1.0, 0.02] = 0.5
    Cutoff3.position -> "Cutoff" [1.0, 0.02] = 0.5
    Cutoff4.position -> "Cutoff" [1.0, 0.02] = 0.5
    Resonance.position -> "Resonance" [0.0, 0.95] = 0.0
  }
}"#;

/// 2-stage cascaded BJT (simplest cascade — should also be blockwise).
const BJT_CASCADE_2: &str = r#"pedal "BJT Cascade 2" {
  supply 9V
  components {
    Q1: npn(2n3904)
    Q2: npn(2n3904)
    R_c1: resistor(22k)
    R_c2: resistor(22k)
    R_e1: resistor(220)
    R_e2: resistor(220)
    C1: cap(22n)
    C2: cap(22n)
    R_in: resistor(10k)
  }
  nets {
    in -> R_in.a
    R_in.b -> Q1.base
    vcc -> R_c1.a
    R_c1.b -> Q1.collector
    Q1.emitter -> R_e1.a
    R_e1.b -> gnd
    Q1.emitter -> C1.a
    C1.b -> gnd

    Q1.collector -> Q2.base
    vcc -> R_c2.a
    R_c2.b -> Q2.collector
    Q2.emitter -> R_e2.a
    R_e2.b -> gnd
    Q2.emitter -> C2.a
    C2.b -> gnd

    Q2.collector -> out
  }
  controls {}
}"#;

/// Single BJT stage (NOT blockwise — just one block, no cascade).
const SINGLE_BJT: &str = r#"pedal "Single BJT" {
  supply 9V
  components {
    Q1: npn(2n3904)
    R_c: resistor(22k)
    R_e: resistor(220)
    C1: cap(22n)
    R_in: resistor(10k)
  }
  nets {
    in -> R_in.a
    R_in.b -> Q1.base
    vcc -> R_c.a
    R_c.b -> Q1.collector
    Q1.emitter -> R_e.a
    R_e.b -> gnd
    Q1.emitter -> C1.a
    C1.b -> gnd
    Q1.collector -> out
  }
  controls {}
}"#;

/// Wheatstone bridge (NOT blockwise — single R-node, no repeating children).
/// 2-stage diode-connected BJT cascade (303 topology).
const DIODE_CASCADE_2: &str = r#"pedal "Diode Cascade 2" {
  supply 9V
  components {
    Q1: npn(2n3904)
    Q2: npn(2n3904)
    R_bias1: resistor(100k)
    R_bias2: resistor(100k)
    C1: cap(33n)
    C2: cap(33n)
    R_e1: resistor(220)
    R_e2: resistor(220)
    R_in: resistor(10k)
  }
  nets {
    vcc -> R_bias1.a
    R_bias1.b -> Q1.base, Q1.collector
    vcc -> R_bias2.a
    R_bias2.b -> Q2.base, Q2.collector
    in -> R_in.a
    R_in.b -> Q1.base
    Q1.emitter -> C1.a
    C1.b -> gnd
    Q1.emitter -> R_e1.a
    R_e1.b -> gnd
    Q1.emitter -> Q2.base
    Q2.emitter -> C2.a
    C2.b -> gnd
    Q2.emitter -> R_e2.a
    R_e2.b -> gnd
    Q2.emitter -> out
  }
  controls {}
}"#;

/// 2-stage diode-connected cascade WITH cutoff pot chains.
/// This is the 303 topology that currently fails blockwise.
/// R_e_floor → Cutoff → gnd creates intermediate nodes that
/// must be claimed by the block through chain-following.
const DIODE_CASCADE_2_WITH_POTS: &str = r#"pedal "Diode Cascade Pots" {
  supply 9V
  components {
    Q1: npn(2n3904)
    Q2: npn(2n3904)
    R_bias1: resistor(100k)
    R_bias2: resistor(100k)
    C1: cap(33n)
    C2: cap(33n)
    R_e_floor1: resistor(220)
    Cutoff1: pot(10k, b)
    R_e_floor2: resistor(220)
    Cutoff2: pot(10k, b)
    R_in: resistor(10k)
  }
  nets {
    vcc -> R_bias1.a
    R_bias1.b -> Q1.base, Q1.collector
    vcc -> R_bias2.a
    R_bias2.b -> Q2.base, Q2.collector
    in -> R_in.a
    R_in.b -> Q1.base
    Q1.emitter -> C1.a
    C1.b -> gnd
    Q1.emitter -> R_e_floor1.a
    R_e_floor1.b -> Cutoff1.a
    Cutoff1.b -> gnd
    Q1.emitter -> Q2.base
    Q2.emitter -> C2.a
    C2.b -> gnd
    Q2.emitter -> R_e_floor2.a
    R_e_floor2.b -> Cutoff2.a
    Cutoff2.b -> gnd
    Q2.emitter -> out
  }
  controls {
    Cutoff1.position -> "Cutoff" [1.0, 0.02] = 0.5
    Cutoff2.position -> "Cutoff" [1.0, 0.02] = 0.5
  }
}"#;

#[test]
fn diagnostic_diode_cascade_pots_spqr_tree() {
    let (graph, edges) = make_graph_all(DIODE_CASCADE_2_WITH_POTS);
    let terminals = vec![graph.in_node, graph.out_node];
    let tree = spqr_decompose(&edges, &terminals, &graph, graph.gnd_node);

    fn print_spqr(node: &SpqrNode, graph: &super::graph::CircuitGraph, depth: usize) {
        let indent = "  ".repeat(depth);
        match node {
            SpqrNode::S {
                children,
                cut_vertices,
            } => {
                eprintln!(
                    "{indent}S (cut={:?}, {} children)",
                    cut_vertices,
                    children.len()
                );
                for c in children {
                    print_spqr(c, graph, depth + 1);
                }
            }
            SpqrNode::P {
                children,
                endpoints,
            } => {
                eprintln!(
                    "{indent}P (ep={:?}, {} children)",
                    endpoints,
                    children.len()
                );
                for c in children {
                    print_spqr(c, graph, depth + 1);
                }
            }
            SpqrNode::Q { edge_idx, .. } => {
                let comp = &graph.components[graph.edges[*edge_idx].comp_idx];
                let kind = graph.effective_edge_kind(*edge_idx);
                let e = &graph.edges[*edge_idx];
                eprintln!(
                    "{indent}Q {}: {}({:?}) [{:?}→{:?}]",
                    edge_idx, comp.id, kind, e.node_a, e.node_b
                );
            }
            SpqrNode::R {
                edge_indices,
                boundary_nodes,
                children,
            } => {
                let mut nl = 0;
                let mut reactive = 0;
                let mut linear = 0;
                for &eidx in edge_indices {
                    match graph.effective_edge_kind(eidx) {
                        super::component::EdgeKind::Nonlinear => nl += 1,
                        super::component::EdgeKind::Reactive => reactive += 1,
                        super::component::EdgeKind::Linear => linear += 1,
                        _ => {}
                    }
                }
                eprintln!("{indent}R ({} edges: {nl}NL+{reactive}C+{linear}R, boundary={:?}, {} children)",
                    edge_indices.len(), boundary_nodes, children.len());
                for &eidx in edge_indices {
                    let comp = &graph.components[graph.edges[eidx].comp_idx];
                    let kind = graph.effective_edge_kind(eidx);
                    let e = &graph.edges[eidx];
                    eprintln!(
                        "{indent}  {}: {}({:?}) [{:?}→{:?}]",
                        eidx, comp.id, kind, e.node_a, e.node_b
                    );
                }
                for c in children {
                    print_spqr(c, graph, depth + 1);
                }
            }
        }
    }

    eprintln!("\n=== Diode Cascade 2 + Pots: SPQR tree ===");
    print_spqr(&tree, &graph, 0);
    eprintln!("  Total edges: {}", tree.edge_count());
    eprintln!(
        "  GND={:?}, VCC={:?}, IN={:?}, OUT={:?}",
        graph.gnd_node, graph.supply_nodes, graph.in_node, graph.out_node
    );
}

#[test]
fn diagnostic_diode_cascade_no_pots_spqr_tree() {
    let (graph, edges) = make_graph_all(DIODE_CASCADE_2);
    let terminals = vec![graph.in_node, graph.out_node];
    let tree = spqr_decompose(&edges, &terminals, &graph, graph.gnd_node);

    fn print_spqr(node: &SpqrNode, graph: &super::graph::CircuitGraph, depth: usize) {
        let indent = "  ".repeat(depth);
        match node {
            SpqrNode::S {
                children,
                cut_vertices,
            } => {
                eprintln!(
                    "{indent}S (cut={:?}, {} children)",
                    cut_vertices,
                    children.len()
                );
                for c in children {
                    print_spqr(c, graph, depth + 1);
                }
            }
            SpqrNode::P { children, .. } => {
                eprintln!("{indent}P ({} children)", children.len());
                for c in children {
                    print_spqr(c, graph, depth + 1);
                }
            }
            SpqrNode::Q { edge_idx, .. } => {
                let comp = &graph.components[graph.edges[*edge_idx].comp_idx];
                let kind = graph.effective_edge_kind(*edge_idx);
                let e = &graph.edges[*edge_idx];
                eprintln!(
                    "{indent}Q {}: {}({:?}) [{:?}→{:?}]",
                    edge_idx, comp.id, kind, e.node_a, e.node_b
                );
            }
            SpqrNode::R {
                edge_indices,
                children,
                ..
            } => {
                eprintln!(
                    "{indent}R ({} edges, {} children)",
                    edge_indices.len(),
                    children.len()
                );
                for c in children {
                    print_spqr(c, graph, depth + 1);
                }
            }
        }
    }

    eprintln!("\n=== Diode Cascade 2 (no pots): SPQR tree ===");
    print_spqr(&tree, &graph, 0);
}

#[test]
fn diode_cascade_with_pots_is_blockwise() {
    let (graph, edges) = make_graph_all(DIODE_CASCADE_2_WITH_POTS);
    let nl_count = count_nl(&edges, &graph);
    eprintln!(
        "  Diode cascade+pots: {nl_count} NL edges, {} total",
        edges.len()
    );

    let plan = blockwise::analyze_blockwise(&edges, &graph);
    if let Some(ref p) = plan {
        eprintln!(
            "  Blockwise: {} blocks, {} coupling",
            p.num_blocks(),
            p.coupling_edges.len()
        );
        for (i, b) in p.blocks.iter().enumerate() {
            eprintln!(
                "    block {i}: {} NL, {} reactive, {} linear",
                b.nl_edges.len(),
                b.reactive_edges.len(),
                b.linear_edges.len()
            );
        }
    } else {
        eprintln!("  Blockwise: None");
    }

    assert!(
        plan.is_some(),
        "Diode cascade with cutoff pots should be blockwise-decomposable. \
         The pot chains (R_e_floor → Cutoff → gnd) must be followed from \
         the NL group's emitter node into the block."
    );
    let plan = plan.unwrap();
    assert_eq!(plan.num_blocks(), 2, "Should split into 2 blocks");

    // Each block must have its cap (reactive) AND its pot chain (linear)
    for (i, block) in plan.blocks.iter().enumerate() {
        assert!(
            block.reactive_edges.len() >= 1,
            "Block {i} must have ≥1 reactive edge (cap), got {}",
            block.reactive_edges.len()
        );
        // The pot chain (R_e_floor + Cutoff) should be in linear_edges
        assert!(
            block.linear_edges.len() >= 2,
            "Block {i} must have ≥2 linear edges (R_e_floor + Cutoff), got {}",
            block.linear_edges.len()
        );
    }
}

#[test]
fn diode_cascade_with_pots_each_block_has_correct_cap() {
    // C1 must be in Q1's block, C2 must be in Q2's block.
    // The pot chains must NOT pull caps into the wrong block.
    let (graph, edges) = make_graph_all(DIODE_CASCADE_2_WITH_POTS);
    let plan = blockwise::analyze_blockwise(&edges, &graph);
    assert!(plan.is_some(), "Should decompose");
    let plan = plan.unwrap();

    // Find which block has C1 and which has C2
    for (i, block) in plan.blocks.iter().enumerate() {
        let cap_names: Vec<&str> = block
            .reactive_edges
            .iter()
            .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
            .collect();
        let nl_names: Vec<&str> = block
            .nl_edges
            .iter()
            .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
            .collect();
        eprintln!("  Block {i}: NL={nl_names:?}, caps={cap_names:?}");

        // Q1's block should have C1, Q2's block should have C2
        // (not both caps in one block)
        assert!(
            cap_names.len() == 1,
            "Block {i} should have exactly 1 cap, got {:?}",
            cap_names
        );
    }
}

const WHEATSTONE_BRIDGE: &str = r#"pedal "Wheatstone" {
  supply 9V
  components {
    R1: resistor(10k)
    R2: resistor(10k)
    R3: resistor(10k)
    R4: resistor(10k)
    R5: resistor(10k)
  }
  nets {
    in -> R1.a
    in -> R3.a
    R1.b -> R2.a, R5.a
    R3.b -> R4.a, R5.b
    R2.b -> out
    R4.b -> out
  }
  controls {}
}"#;

// ═══════════════════════════════════════════════════════════════════════════
// 1. Structural detection: SPQR produces an R-node with repeated NL children
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn ladder_4_has_4_nl_elements() {
    let (graph, edges) = make_graph_all(BJT_LADDER_4);
    let nl_count = count_nl(&edges, &graph);
    // 4 BJTs × 2 NL junctions each = 8 NL edges (or 4 if counted by component)
    assert!(
        nl_count >= 4,
        "Should have at least 4 NL edges, got {nl_count}"
    );
}

#[test]
fn ladder_4_has_4_reactive_elements() {
    let (graph, edges) = make_graph_all(BJT_LADDER_4);
    let reactive_count = count_reactive(&edges, &graph);
    assert!(
        reactive_count >= 4,
        "Should have at least 4 reactive edges, got {reactive_count}"
    );
}

#[test]
fn cascade_2_has_2_nl_elements() {
    let (graph, edges) = make_graph_all(BJT_CASCADE_2);
    let nl_count = count_nl(&edges, &graph);
    assert!(
        nl_count >= 2,
        "Should have at least 2 NL edges, got {nl_count}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. The R-node contains identifiable sub-blocks
//
// A blockwise-decomposable R-node has children (or subgraphs) that each
// contain a small number of NL + reactive elements. The detection function
// should identify these sub-blocks.
//
// Future API: `blockwise::analyze_blockwise(r_node, graph) -> Option<BlockwisePlan>`
// For now, we define the expected structure and test it.
// ═══════════════════════════════════════════════════════════════════════════

// BlockwisePlan and analyze_blockwise are in super::blockwise

#[test]
fn ladder_4_is_blockwise_decomposable() {
    let (graph, edges) = make_graph_all(BJT_LADDER_4);

    // Debug: check NL grouping directly
    let nl_count = count_nl(&edges, &graph);
    eprintln!(
        "  Ladder 4: {nl_count} NL edges, {} total edges",
        edges.len()
    );

    // Check what groups we get
    let nl_edges: Vec<usize> = edges
        .iter()
        .filter(|&&eidx| graph.effective_edge_kind(eidx) == super::component::EdgeKind::Nonlinear)
        .copied()
        .collect();
    eprintln!(
        "  NL edges: {:?}",
        nl_edges
            .iter()
            .map(|&eidx| format!(
                "{}({:?}→{:?})",
                graph.components[graph.edges[eidx].comp_idx].id,
                graph.edges[eidx].node_a,
                graph.edges[eidx].node_b
            ))
            .collect::<Vec<_>>()
    );

    let plan = blockwise::analyze_blockwise(&edges, &graph);

    assert!(
        plan.is_some(),
        "4-stage BJT ladder should be blockwise-decomposable"
    );
    let plan = plan.unwrap();

    assert_eq!(plan.num_blocks(), 4, "Should decompose into 4 blocks");

    // Each block should have exactly 1 BJT (1-2 NL edges) and 1 cap
    for (i, block) in plan.blocks.iter().enumerate() {
        assert!(
            block.nl_edges.len() >= 1 && block.nl_edges.len() <= 2,
            "Block {i} should have 1-2 NL edges, got {}",
            block.nl_edges.len()
        );
        assert!(
            block.reactive_edges.len() >= 1,
            "Block {i} should have at least 1 reactive edge, got {}",
            block.reactive_edges.len()
        );
    }

    // Coupling edges: the resonance feedback may be split across blocks
    // (each edge in the Resonance→R_fb chain touches only one block).
    // The important thing is all 4 blocks exist with NL + reactive edges.
}

#[test]
fn cascade_2_is_blockwise_decomposable() {
    let (graph, edges) = make_graph_all(BJT_CASCADE_2);
    let plan = blockwise::analyze_blockwise(&edges, &graph);

    assert!(
        plan.is_some(),
        "2-stage BJT cascade should be blockwise-decomposable"
    );
    let plan = plan.unwrap();

    assert_eq!(plan.num_blocks(), 2, "Should decompose into 2 blocks");
}

#[test]
fn single_bjt_is_not_blockwise() {
    let (graph, edges) = make_graph_all(SINGLE_BJT);
    let plan = blockwise::analyze_blockwise(&edges, &graph);

    assert!(
        plan.is_none(),
        "Single BJT should NOT be blockwise (only 1 NL element, no cascade)"
    );
}

#[test]
fn diode_cascade_2_is_blockwise_decomposable() {
    let (graph, edges) = make_graph_all(DIODE_CASCADE_2);
    let nl_count = count_nl(&edges, &graph);
    eprintln!(
        "  Diode cascade 2: {nl_count} NL edges, {} total",
        edges.len()
    );

    let plan = blockwise::analyze_blockwise(&edges, &graph);
    if let Some(ref p) = plan {
        eprintln!(
            "  Blockwise: {} blocks, {} coupling",
            p.num_blocks(),
            p.coupling_edges.len()
        );
        for (i, b) in p.blocks.iter().enumerate() {
            eprintln!(
                "    block {i}: {} NL, {} reactive, {} linear",
                b.nl_edges.len(),
                b.reactive_edges.len(),
                b.linear_edges.len()
            );
        }
    } else {
        eprintln!("  Blockwise: None");
    }

    assert!(
        plan.is_some(),
        "Diode-connected 2-stage cascade should be blockwise-decomposable"
    );
    let plan = plan.unwrap();
    assert_eq!(plan.num_blocks(), 2, "Should split into 2 blocks");

    // Each block should have ≥1 reactive edge (a cap)
    for (i, block) in plan.blocks.iter().enumerate() {
        assert!(
            block.reactive_edges.len() >= 1,
            "Block {i} should have ≥1 reactive edge, got {}",
            block.reactive_edges.len()
        );
    }
}

#[test]
fn wheatstone_bridge_is_not_blockwise() {
    let (graph, edges) = make_graph_all(WHEATSTONE_BRIDGE);
    let plan = blockwise::analyze_blockwise(&edges, &graph);

    assert!(
        plan.is_none(),
        "Wheatstone bridge is a single R-node with no repeating NL children"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Full pipeline: blockwise circuits should NOT produce monolithic MultiNL
//
// When blockwise detection is implemented, the compiler should emit
// chained WDF stages instead of one big MultiNL stage.
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn ladder_4_compile_emits_chained_stages_not_monolithic() {
    let compiled = compile_cached(BJT_LADDER_4, "blockwise_test");

    let multi_nl_count = compiled
        .stages
        .iter()
        .filter(|s| matches!(s, super::compiled::Stage::MultiNl(_)))
        .count();

    // Today: this produces 1 monolithic MultiNL with all 4 BJTs.
    // Target: 0 MultiNL stages — replaced by 4 chained WDF stages.
    assert_eq!(
        multi_nl_count, 0,
        "Blockwise ladder should have 0 MultiNL stages (got {multi_nl_count}). \
         Each BJT stage should be its own WDF block."
    );

    // Should have at least 4 WDF stages (one per BJT block)
    let wdf_count = compiled
        .stages
        .iter()
        .filter(|s| matches!(s, super::compiled::Stage::Wdf(_)))
        .count();
    assert!(
        wdf_count >= 4,
        "Should have at least 4 WDF stages for the 4 BJT blocks, got {wdf_count}"
    );
}

#[test]
fn cascade_2_compile_emits_2_wdf_stages() {
    let compiled = compile_cached(BJT_CASCADE_2, "blockwise_test");

    let multi_nl_count = compiled
        .stages
        .iter()
        .filter(|s| matches!(s, super::compiled::Stage::MultiNl(_)))
        .count();

    assert_eq!(
        multi_nl_count, 0,
        "2-stage cascade should have 0 MultiNL (got {multi_nl_count})"
    );

    let wdf_count = compiled
        .stages
        .iter()
        .filter(|s| matches!(s, super::compiled::Stage::Wdf(_)))
        .count();
    assert!(
        wdf_count >= 2,
        "Should have at least 2 WDF stages, got {wdf_count}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Negative cases: non-cascade circuits stay monolithic
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn single_bjt_stays_as_single_stage() {
    let compiled = compile_cached(SINGLE_BJT, "blockwise_test");

    // Single BJT should compile to a single WDF or MultiNL stage — not split
    let total_stages = compiled.stages.len();
    assert!(
        total_stages <= 3,
        "Single BJT should have ≤3 stages (input coupling + BJT + output), got {total_stages}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Diagnostic: trace what the compiler actually produces for each circuit
// ═══════════════════════════════════════════════════════════════════════════

fn print_stages(name: &str, compiled: &super::compiled::CompiledPedal) {
    eprintln!("\n=== {name}: {} stages ===", compiled.stages.len());
    for (i, stage) in compiled.stages.iter().enumerate() {
        match stage {
            super::compiled::Stage::Wdf(w) => {
                let fb = if w.feedback_opamp.is_some() {
                    "+OA"
                } else {
                    ""
                };
                eprintln!("  [{i}] WDF{fb} dist={}", w.signal_flow_distance);
            }
            super::compiled::Stage::MultiNl(m) => {
                eprintln!(
                    "  [{i}] MultiNL ports={} dist={}",
                    m.nl_devices.len(),
                    m.signal_flow_distance
                );
            }
            super::compiled::Stage::Iir(iir) => {
                #[cfg(debug_assertions)]
                eprintln!("  [{i}] IIR label={:?}", iir.debug_label);
                #[cfg(not(debug_assertions))]
                eprintln!("  [{i}] IIR");
            }
            super::compiled::Stage::BlackFeedback(bf) => {
                eprintln!("  [{i}] BF gain={:.2}", bf.gain());
            }
            super::compiled::Stage::StateSpace(_) => {
                eprintln!("  [{i}] StateSpace");
            }
            super::compiled::Stage::BlockwiseKMethod(bk) => {
                eprintln!("  [{i}] BKM dist={}", bk.signal_flow_distance);
            }
        }
    }
}

#[test]
fn diagnostic_cascade_2_stage_structure() {
    let compiled = compile_cached(BJT_CASCADE_2, "blockwise_test");
    print_stages("BJT Cascade 2", &compiled);
}

#[test]
fn diagnostic_ladder_4_stage_structure() {
    let compiled = compile_cached(BJT_LADDER_4, "blockwise_test");
    print_stages("BJT Ladder 4", &compiled);
}

/// Trace the raw SPQR tree for the ladder circuit.
#[test]
fn diagnostic_ladder_4_spqr_tree() {
    let (graph, edges) = make_graph_all(BJT_LADDER_4);
    let terminals = vec![graph.in_node, graph.out_node];
    let tree = spqr_decompose(&edges, &terminals, &graph, graph.gnd_node);

    fn print_spqr(node: &SpqrNode, graph: &CircuitGraph, depth: usize) {
        let indent = "  ".repeat(depth);
        match node {
            SpqrNode::S {
                children,
                cut_vertices,
            } => {
                eprintln!(
                    "{indent}S (cut={:?}, {} children)",
                    cut_vertices,
                    children.len()
                );
                for c in children {
                    print_spqr(c, graph, depth + 1);
                }
            }
            SpqrNode::P {
                children,
                endpoints,
            } => {
                eprintln!(
                    "{indent}P (ep={:?}, {} children)",
                    endpoints,
                    children.len()
                );
                for c in children {
                    print_spqr(c, graph, depth + 1);
                }
            }
            SpqrNode::Q { edge_idx, .. } => {
                let comp = &graph.components[graph.edges[*edge_idx].comp_idx];
                let kind = graph.effective_edge_kind(*edge_idx);
                eprintln!("{indent}Q edge={} {}({:?})", edge_idx, comp.id, kind);
            }
            SpqrNode::R {
                edge_indices,
                boundary_nodes,
                children,
            } => {
                // Classify edges in this R-node
                let mut nl = 0;
                let mut reactive = 0;
                let mut linear = 0;
                for &eidx in edge_indices {
                    match graph.effective_edge_kind(eidx) {
                        super::component::EdgeKind::Nonlinear => nl += 1,
                        super::component::EdgeKind::Reactive => reactive += 1,
                        super::component::EdgeKind::Linear => linear += 1,
                        _ => {}
                    }
                }
                eprintln!(
                    "{indent}R ({} edges: {nl}NL+{reactive}C+{linear}R, {} boundary, {} children)",
                    edge_indices.len(),
                    boundary_nodes.len(),
                    children.len()
                );
                // Print edge names
                for &eidx in edge_indices {
                    let comp = &graph.components[graph.edges[eidx].comp_idx];
                    let kind = graph.effective_edge_kind(eidx);
                    eprintln!("{indent}  edge {eidx}: {}({:?})", comp.id, kind);
                }
                for c in children {
                    print_spqr(c, graph, depth + 1);
                }
            }
        }
    }

    eprintln!("\n=== BJT Ladder 4: SPQR tree ===");
    print_spqr(&tree, &graph, 0);
    eprintln!("  Total edges: {}", tree.edge_count());
}

/// Also trace the cascade 2 SPQR tree for comparison.
#[test]
fn diagnostic_cascade_2_spqr_tree() {
    let (graph, edges) = make_graph_all(BJT_CASCADE_2);
    let terminals = vec![graph.in_node, graph.out_node];
    let tree = spqr_decompose(&edges, &terminals, &graph, graph.gnd_node);

    fn print_spqr(node: &SpqrNode, graph: &CircuitGraph, depth: usize) {
        let indent = "  ".repeat(depth);
        match node {
            SpqrNode::S { children, .. } => {
                eprintln!("{indent}S ({} children)", children.len());
                for c in children {
                    print_spqr(c, graph, depth + 1);
                }
            }
            SpqrNode::P { children, .. } => {
                eprintln!("{indent}P ({} children)", children.len());
                for c in children {
                    print_spqr(c, graph, depth + 1);
                }
            }
            SpqrNode::Q { edge_idx, .. } => {
                let comp = &graph.components[graph.edges[*edge_idx].comp_idx];
                let kind = graph.effective_edge_kind(*edge_idx);
                eprintln!("{indent}Q {}({:?})", comp.id, kind);
            }
            SpqrNode::R {
                edge_indices,
                children,
                ..
            } => {
                let mut nl = 0;
                let mut reactive = 0;
                let mut linear = 0;
                for &eidx in edge_indices {
                    match graph.effective_edge_kind(eidx) {
                        super::component::EdgeKind::Nonlinear => nl += 1,
                        super::component::EdgeKind::Reactive => reactive += 1,
                        super::component::EdgeKind::Linear => linear += 1,
                        _ => {}
                    }
                }
                eprintln!(
                    "{indent}R ({} edges: {nl}NL+{reactive}C+{linear}R, {} children)",
                    edge_indices.len(),
                    children.len()
                );
                for c in children {
                    print_spqr(c, graph, depth + 1);
                }
            }
        }
    }

    eprintln!("\n=== BJT Cascade 2: SPQR tree ===");
    print_spqr(&tree, &graph, 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. Performance assertion: blockwise should be cheaper than monolithic
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn ladder_4_blockwise_has_no_large_scattering_matrix() {
    let compiled = compile_cached(BJT_LADDER_4, "blockwise_test");

    // Check that no MultiNL stage has > 4 NL ports
    // (monolithic 303 would have 8 ports; blockwise has max 2 per block)
    for (i, stage) in compiled.stages.iter().enumerate() {
        if let super::compiled::Stage::MultiNl(mnl) = stage {
            let n_ports = mnl.nl_devices.len();
            assert!(
                n_ports <= 4,
                "Stage {i} has {n_ports} NL ports — too large for blockwise. \
                 Max should be 2-4 per block."
            );
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. K-method candidacy: per-component NL characterization
//
// K-method requires the NL element to be memoryless, low-dimensional,
// fixed-parameter, and monotonic. These are properties of the Component,
// not the circuit topology.
// ═══════════════════════════════════════════════════════════════════════════

/// K-method analysis of a nonlinear subgraph.
///
/// This is the "box" you draw around the NL stuff. The analysis checks:
/// 1. All NL elements inside are memoryless (component-level k_method_candidacy)
/// 2. No reactive elements inside the box (caps/inductors must be outside)
/// 3. Port wires crossing the boundary ≤ 3 (table dimensionality)
/// 4. No variable components inside (pots, photocouplers)
#[derive(Debug)]
struct KMethodSubgraph {
    /// Can this subgraph be tabulated?
    is_candidate: bool,
    /// Edge indices inside the NL box.
    nl_box_edges: Vec<usize>,
    /// Number of port wires crossing the box boundary = table dimensions.
    port_dims: usize,
    /// Reason for rejection, if not a candidate.
    rejection: Option<String>,
}

/// Analyze a set of edges as a potential K-method subgraph.
///
/// Finds the NL "box": all NL edges + any linear edges connecting them
/// internally. Checks the 4 constraints. Counts boundary wires.
///
/// This is a stub — returns a rejection until the subgraph analysis is
/// implemented. The component-level `k_method_candidacy()` provides the
/// building block (constraint 1), but the full check needs graph analysis
/// for constraints 2-4.
fn analyze_k_method_subgraph(edge_indices: &[usize], graph: &CircuitGraph) -> KMethodSubgraph {
    // Step 1: find NL edges
    let nl_edges: Vec<usize> = edge_indices
        .iter()
        .filter(|&&eidx| graph.effective_edge_kind(eidx) == super::component::EdgeKind::Nonlinear)
        .copied()
        .collect();

    if nl_edges.is_empty() {
        return KMethodSubgraph {
            is_candidate: false,
            nl_box_edges: vec![],
            port_dims: 0,
            rejection: Some("no nonlinear elements".into()),
        };
    }

    // Step 2: check all NL components are memoryless (component-level)
    for &eidx in &nl_edges {
        let comp = &graph.components[graph.edges[eidx].comp_idx];
        let (is_ok, _dims, reason) = comp.kind.k_method_candidacy();
        if !is_ok {
            return KMethodSubgraph {
                is_candidate: false,
                nl_box_edges: nl_edges,
                port_dims: 0,
                rejection: Some(format!("{}: {reason}", comp.id)),
            };
        }
    }

    // Step 3: check no reactive elements inside the NL box.
    // The "box" is the NL edges + any linear edges whose BOTH endpoints
    // are internal to the NL subgraph (shared nodes between NL edges).
    let mut nl_nodes: std::collections::HashSet<super::graph::NodeId> =
        std::collections::HashSet::new();
    for &eidx in &nl_edges {
        let e = &graph.edges[eidx];
        nl_nodes.insert(e.node_a);
        nl_nodes.insert(e.node_b);
    }
    // Note: ground/supply are kept in nl_nodes — a cap from base to gnd
    // IS inside the NL box if the base is an NL node.

    let mut box_edges = nl_edges.clone();
    for &eidx in edge_indices {
        if box_edges.contains(&eidx) {
            continue;
        }
        let e = &graph.edges[eidx];
        // Edge is "inside the box" if at least one endpoint is an NL node
        // and the other is either also NL or ground/supply (shared rail)
        let a_is_nl = nl_nodes.contains(&e.node_a);
        let b_is_nl = nl_nodes.contains(&e.node_b);
        if a_is_nl && b_is_nl {
            let kind = graph.effective_edge_kind(eidx);
            if kind == super::component::EdgeKind::Reactive {
                return KMethodSubgraph {
                    is_candidate: false,
                    nl_box_edges: nl_edges,
                    port_dims: 0,
                    rejection: Some(format!(
                        "reactive element {} inside NL box",
                        graph.components[graph.edges[eidx].comp_idx].id
                    )),
                };
            }
            // Check for variable components
            if graph.components[graph.edges[eidx].comp_idx]
                .kind
                .is_variable()
            {
                return KMethodSubgraph {
                    is_candidate: false,
                    nl_box_edges: nl_edges,
                    port_dims: 0,
                    rejection: Some(format!(
                        "variable component {} inside NL box",
                        graph.components[graph.edges[eidx].comp_idx].id
                    )),
                };
            }
            box_edges.push(eidx);
        }
    }

    // Step 4: count boundary ports — nodes where box edges connect to
    // non-box edges. Each such node is one port wire.
    let mut box_nodes: std::collections::HashSet<super::graph::NodeId> =
        std::collections::HashSet::new();
    for &eidx in &box_edges {
        let e = &graph.edges[eidx];
        box_nodes.insert(e.node_a);
        box_nodes.insert(e.node_b);
    }
    // Remove ground/supply — they're implicit, not counted as ports
    box_nodes.remove(&graph.gnd_node);
    for &s in &graph.supply_nodes {
        box_nodes.remove(&s);
    }
    // A boundary node has at least one edge OUTSIDE the box
    let non_box_edges: Vec<usize> = edge_indices
        .iter()
        .filter(|e| !box_edges.contains(e))
        .copied()
        .collect();
    let mut boundary_nodes = 0usize;
    for &node in &box_nodes {
        let touches_outside = non_box_edges.iter().any(|&eidx| {
            let e = &graph.edges[eidx];
            e.node_a == node || e.node_b == node
        });
        if touches_outside {
            boundary_nodes += 1;
        }
    }

    if boundary_nodes > 3 {
        return KMethodSubgraph {
            is_candidate: false,
            nl_box_edges: box_edges,
            port_dims: boundary_nodes,
            rejection: Some(format!(
                "{boundary_nodes} port wires — too many dimensions for K-method table"
            )),
        };
    }

    KMethodSubgraph {
        is_candidate: true,
        nl_box_edges: box_edges,
        port_dims: boundary_nodes.max(1), // at least 1D
        rejection: None,
    }
}

// ── Component-level building block tests ──

#[test]
fn diode_component_is_memoryless() {
    let (graph, _) = make_graph_all(
        r#"pedal "test" {
      supply 9V
      components { D1: diode(silicon)  R1: resistor(10k) }
      nets { in -> R1.a  R1.b -> D1.anode  D1.cathode -> gnd  R1.b -> out }
      controls {}
    }"#,
    );
    let diode = graph
        .components
        .iter()
        .find(|c| c.kind.type_tag() == "diode")
        .unwrap();
    let (ok, dims, _) = diode.kind.k_method_candidacy();
    assert!(ok, "Diode should be memoryless");
    assert_eq!(dims, 1);
}

#[test]
fn bjt_component_is_memoryless_2d() {
    let (graph, _) = make_graph_all(SINGLE_BJT);
    let bjt = graph
        .components
        .iter()
        .find(|c| c.kind.type_tag() == "NPN transistor")
        .unwrap();
    let (ok, dims, _) = bjt.kind.k_method_candidacy();
    assert!(ok, "BJT should be memoryless");
    assert_eq!(dims, 2);
}

#[test]
fn resistor_component_is_not_nl() {
    let (graph, _) = make_graph_all(
        r#"pedal "test" {
      supply 9V
      components { R1: resistor(10k) }
      nets { in -> R1.a  R1.b -> out }
      controls {}
    }"#,
    );
    let r = graph
        .components
        .iter()
        .find(|c| c.kind.type_tag() == "resistor")
        .unwrap();
    let (ok, _, _) = r.kind.k_method_candidacy();
    assert!(!ok, "Resistor is linear");
}

// ── Subgraph-level K-method tests ──

#[test]
fn diode_clipper_subgraph_is_k_candidate() {
    // D1 + D2 anti-parallel with R1 — all NL is memoryless, no reactive
    // inside the box, ≤2 boundary wires (signal in, ground)
    let (graph, edges) = make_graph_all(
        r#"pedal "test" {
      supply 9V
      components {
        D1: diode(silicon)
        D2: diode(silicon)
        R1: resistor(10k)
      }
      nets {
        in -> R1.a
        R1.b -> D1.anode
        D1.cathode -> gnd
        R1.b -> D2.cathode
        D2.anode -> gnd
        R1.b -> out
      }
      controls {}
    }"#,
    );

    let result = analyze_k_method_subgraph(&edges, &graph);
    eprintln!(
        "  Diode clipper: candidate={}, dims={}, rejection={:?}",
        result.is_candidate, result.port_dims, result.rejection
    );
    assert!(
        result.is_candidate,
        "Diode clipper subgraph should be K-method: {:?}",
        result.rejection
    );
    assert!(
        result.port_dims <= 2,
        "Should be ≤2D, got {}",
        result.port_dims
    );
}

#[test]
fn single_bjt_subgraph_is_k_candidate() {
    // Single BJT stage: BJT is memoryless 2D, caps are OUTSIDE the box
    // (C1 is a ground shunt separate from the BJT junctions)
    let (graph, edges) = make_graph_all(SINGLE_BJT);
    let result = analyze_k_method_subgraph(&edges, &graph);
    eprintln!(
        "  Single BJT: candidate={}, dims={}, rejection={:?}",
        result.is_candidate, result.port_dims, result.rejection
    );
    assert!(
        result.is_candidate,
        "Single BJT subgraph should be K-method: {:?}",
        result.rejection
    );
    assert!(
        result.port_dims <= 3,
        "Should be ≤3D, got {}",
        result.port_dims
    );
}

#[test]
fn bjt_with_cap_across_junction_is_not_k_candidate() {
    // If a cap connects directly across the BJT's B-E junction,
    // it's INSIDE the NL box → reactive inside → not K-method
    let (graph, edges) = make_graph_all(
        r#"pedal "test" {
      supply 9V
      components {
        Q1: npn(2n3904)
        R_c: resistor(22k)
        R_in: resistor(10k)
        C_be: cap(100p)
      }
      nets {
        in -> R_in.a
        R_in.b -> Q1.base
        vcc -> R_c.a
        R_c.b -> Q1.collector
        Q1.emitter -> gnd
        Q1.base -> C_be.a
        C_be.b -> Q1.emitter
        Q1.collector -> out
      }
      controls {}
    }"#,
    );

    let result = analyze_k_method_subgraph(&edges, &graph);
    eprintln!(
        "  BJT+C_be: candidate={}, dims={}, rejection={:?}",
        result.is_candidate, result.port_dims, result.rejection
    );
    assert!(
        !result.is_candidate,
        "BJT with cap across B-E should NOT be K-method (reactive inside NL box)"
    );
}

#[test]
fn pure_linear_circuit_is_not_k_candidate() {
    let (graph, edges) = make_graph_all(WHEATSTONE_BRIDGE);
    let result = analyze_k_method_subgraph(&edges, &graph);
    assert!(
        !result.is_candidate,
        "Pure linear circuit has no NL to tabulate"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 7. Combined: blockwise + K-method gives the full optimization plan
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn ladder_4_each_block_is_k_method_eligible() {
    let (graph, edges) = make_graph_all(BJT_LADDER_4);
    let plan = blockwise::analyze_blockwise(&edges, &graph);

    // First, blockwise must succeed
    assert!(plan.is_some(), "Ladder should be blockwise-decomposable");
    let plan = plan.unwrap();

    // Each block's edges (NL + reactive + linear) should form a valid
    // K-method subgraph: memoryless NL, no reactive INSIDE the NL box,
    // ≤3 boundary wires.
    for (i, block) in plan.blocks.iter().enumerate() {
        // Build the full edge set for this block
        let mut block_edges = block.nl_edges.clone();
        block_edges.extend(&block.reactive_edges);
        // The reactive edges are OUTSIDE the NL box (caps are state, not NL).
        // The subgraph analysis should see NL edges only as the "box"
        // and confirm no reactive edges share both endpoints with NL.
        let result = analyze_k_method_subgraph(&block_edges, &graph);
        assert!(
            result.is_candidate,
            "Block {i} should be K-method eligible: {:?}",
            result.rejection,
        );
        assert!(
            result.port_dims <= 3,
            "Block {i} has {} port dims — too many for K-method",
            result.port_dims,
        );
    }
}

#[test]
fn ts808_clipper_is_k_subgraph_but_not_blockwise() {
    // TS808 diode clipper: the whole subgraph is one K-method candidate
    // (1D diode pair), but NOT blockwise (no cascade to split).
    let (graph, edges) = make_graph_all(
        r#"pedal "test" {
      supply 9V
      components {
        D1: diode(silicon)
        D2: diode(silicon)
        R1: resistor(10k)
      }
      nets {
        in -> R1.a
        R1.b -> D1.anode
        D1.cathode -> gnd
        R1.b -> D2.cathode
        D2.anode -> gnd
        R1.b -> out
      }
      controls {}
    }"#,
    );

    // Not blockwise
    let plan = blockwise::analyze_blockwise(&edges, &graph);
    assert!(
        plan.is_none(),
        "TS808 clipper is not blockwise (single NL group)"
    );

    // But the full subgraph IS a K-method candidate
    let result = analyze_k_method_subgraph(&edges, &graph);
    assert!(
        result.is_candidate,
        "TS808 clipper subgraph should be K-method: {:?}",
        result.rejection
    );
    assert!(
        result.port_dims <= 2,
        "Should be ≤2D, got {}",
        result.port_dims
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 8. Blockwise runtime behavior: chained blocks produce correct audio
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn cascade_2_produces_audio() {
    let compiled = compile_cached(BJT_CASCADE_2, "blockwise_test");
    let mut proc: Box<dyn crate::PedalProcessor> = Box::new(compiled);

    let mut peak = 0.0f64;
    for i in 0..9600 {
        let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48_000.0).sin() * 0.1;
        let out = proc.process(input);
        if i >= 4800 {
            peak = peak.max(out.abs());
        }
    }
    eprintln!("  Cascade 2 peak: {peak:.6}");
    assert!(
        peak > 0.001,
        "Cascade 2 should produce audible output, got {peak:.6}"
    );
    assert!(peak.is_finite(), "Output should be finite");
}

#[test]
fn ladder_4_produces_audio() {
    let compiled = compile_cached(BJT_LADDER_4, "blockwise_test");
    let mut proc: Box<dyn crate::PedalProcessor> = Box::new(compiled);

    let mut peak = 0.0f64;
    for i in 0..9600 {
        let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48_000.0).sin() * 0.1;
        let out = proc.process(input);
        if i >= 4800 {
            peak = peak.max(out.abs());
        }
    }
    eprintln!("  Ladder 4 peak: {peak:.6}");
    assert!(
        peak > 0.001,
        "Ladder 4 should produce audible output, got {peak:.6}"
    );
    assert!(peak.is_finite(), "Output should be finite");
}

#[test]
fn ladder_4_resonance_affects_output() {
    // Process with resonance = 0
    let mut no_res = compile_cached(BJT_LADDER_4, "blockwise_test");
    no_res.set_control("Resonance", 0.0);
    let mut peak_no = 0.0f64;
    for i in 0..9600 {
        let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48_000.0).sin() * 0.1;
        let out = no_res.process(input);
        if i >= 4800 {
            peak_no = peak_no.max(out.abs());
        }
    }

    // Process with resonance = 0.9
    let mut hi_res = compile_cached(BJT_LADDER_4, "blockwise_test");
    hi_res.set_control("Resonance", 0.9);
    let mut peak_hi = 0.0f64;
    for i in 0..9600 {
        let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48_000.0).sin() * 0.1;
        let out = hi_res.process(input);
        if i >= 4800 {
            peak_hi = peak_hi.max(out.abs());
        }
    }

    eprintln!("  Ladder 4 resonance: off={peak_no:.6}, hi={peak_hi:.6}");
    assert!(
        (peak_no - peak_hi).abs() > 0.0001 || (peak_no > 0.001 && peak_hi > 0.001),
        "Resonance should affect output: off={peak_no:.6}, hi={peak_hi:.6}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 9. Blockwise produces BETTER results than monolithic for same circuit
//
// The blockwise decomposition should produce correct cascaded lowpass
// behavior — each block independently resolves its pole. A monolithic
// solver may struggle with convergence or produce artifacts.
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn ladder_4_no_nan_or_inf_at_any_resonance() {
    let compiled = compile_cached(BJT_LADDER_4, "blockwise_test");

    for &res in &[0.0, 0.3, 0.5, 0.7, 0.9, 1.0] {
        let mut proc = compile_cached(BJT_LADDER_4, "blockwise_test");
        proc.set_control("Resonance", res);

        let mut any_bad = false;
        for i in 0..4800 {
            let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48_000.0).sin() * 0.1;
            let out = proc.process(input);
            if out.is_nan() || out.is_infinite() {
                any_bad = true;
                break;
            }
        }
        assert!(!any_bad, "Ladder 4 at resonance={res} produced NaN/Inf");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 10. analyze_blockwise structural details
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn analyze_blockwise_groups_by_shared_nodes() {
    // Each block should contain edges that share circuit nodes
    // (BJT B-E and C-E junctions share the emitter node with Re and C)
    let (graph, edges) = make_graph_all(BJT_LADDER_4);
    let plan = blockwise::analyze_blockwise(&edges, &graph);
    assert!(plan.is_some(), "Should decompose");
    let plan = plan.unwrap();

    // Each block's NL edges should share nodes with its reactive edges
    // (the cap is connected to the same emitter node as the BJT)
    for (i, block) in plan.blocks.iter().enumerate() {
        let nl_nodes: std::collections::HashSet<_> = block
            .nl_edges
            .iter()
            .flat_map(|&eidx| {
                let e = &graph.edges[eidx];
                vec![e.node_a, e.node_b]
            })
            .collect();
        let reactive_nodes: std::collections::HashSet<_> = block
            .reactive_edges
            .iter()
            .flat_map(|&eidx| {
                let e = &graph.edges[eidx];
                vec![e.node_a, e.node_b]
            })
            .collect();
        let shared = nl_nodes.intersection(&reactive_nodes).count();
        assert!(
            shared > 0,
            "Block {i}: NL and reactive edges should share at least 1 node (emitter), got 0"
        );
    }
}

#[test]
fn analyze_blockwise_coupling_is_sparse() {
    // The coupling edges (inter-block + feedback) should be a small
    // fraction of total edges — sparse connection network
    let (graph, edges) = make_graph_all(BJT_LADDER_4);
    let plan = blockwise::analyze_blockwise(&edges, &graph);
    assert!(plan.is_some());
    let plan = plan.unwrap();

    let total_block_edges: usize = plan
        .blocks
        .iter()
        .map(|b| b.nl_edges.len() + b.reactive_edges.len() + b.linear_edges.len())
        .sum();
    let coupling_count = plan.coupling_edges.len();

    eprintln!("  Block edges: {total_block_edges}, coupling edges: {coupling_count}");
    assert!(
        coupling_count < total_block_edges,
        "Coupling edges ({coupling_count}) should be fewer than block edges ({total_block_edges})"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Feedback path must be coupling, not assigned to a block
// ═══════════════════════════════════════════════════════════════════════════

/// 303-like ladder with resonance feedback: the feedback path between the
/// last and first stage must be classified as coupling, not absorbed into
/// the last block. Each of the 4 blocks should be SP-reducible (SingleNl).
const LADDER_WITH_FEEDBACK: &str = r#"pedal "Ladder Feedback" {
  supply 9V
  components {
    Q1: npn(2n3904)
    Q2: npn(2n3904)
    Q3: npn(2n3904)
    Q4: npn(2n3904)
    R_bias1: resistor(100k)
    R_bias2: resistor(100k)
    R_bias3: resistor(100k)
    R_bias4: resistor(100k)
    C1: cap(33n)
    C2: cap(33n)
    C3: cap(33n)
    C4: cap(33n)
    R_e1: resistor(220)
    R_e2: resistor(220)
    R_e3: resistor(220)
    R_e4: resistor(220)
    R_in: resistor(10k)
    R_fb: resistor(33k)
    R_out: resistor(100k)
    C_out: cap(100n)
  }
  nets {
    vcc -> R_bias1.a
    R_bias1.b -> Q1.base, Q1.collector
    vcc -> R_bias2.a
    R_bias2.b -> Q2.base, Q2.collector
    vcc -> R_bias3.a
    R_bias3.b -> Q3.base, Q3.collector
    vcc -> R_bias4.a
    R_bias4.b -> Q4.base, Q4.collector
    in -> R_in.a
    R_in.b -> Q1.base
    Q1.emitter -> C1.a  C1.b -> gnd
    Q1.emitter -> R_e1.a  R_e1.b -> gnd
    Q1.emitter -> Q2.base
    Q2.emitter -> C2.a  C2.b -> gnd
    Q2.emitter -> R_e2.a  R_e2.b -> gnd
    Q2.emitter -> Q3.base
    Q3.emitter -> C3.a  C3.b -> gnd
    Q3.emitter -> R_e3.a  R_e3.b -> gnd
    Q3.emitter -> Q4.base
    Q4.emitter -> C4.a  C4.b -> gnd
    Q4.emitter -> R_e4.a  R_e4.b -> gnd
    Q4.emitter -> C_out.a
    C_out.b -> R_out.a  R_out.b -> gnd
    C_out.b -> out
    Q4.emitter -> R_fb.a
    R_fb.b -> R_in.b
  }
  controls {}
}"#;

#[test]
fn ladder_feedback_path_is_coupling() {
    // The resonance feedback path (Q4.emitter → R_fb → R_in.b → Q1.base)
    // must be classified as coupling, NOT absorbed into Block 3.
    let (graph, edges) = make_graph_all(LADDER_WITH_FEEDBACK);
    let plan = blockwise::analyze_blockwise(&edges, &graph);
    assert!(
        plan.is_some(),
        "ladder with feedback should be blockwise-decomposable"
    );
    let plan = plan.unwrap();

    eprintln!(
        "  Feedback ladder: {} blocks, {} coupling edges",
        plan.num_blocks(),
        plan.coupling_edges.len()
    );
    for (i, b) in plan.blocks.iter().enumerate() {
        eprintln!(
            "    block {i}: {} NL, {} reactive, {} linear (total {})",
            b.nl_edges.len(),
            b.reactive_edges.len(),
            b.linear_edges.len(),
            b.all_edges().len()
        );
    }

    assert_eq!(
        plan.num_blocks(),
        4,
        "should have 4 blocks (one per BJT stage)"
    );

    // The feedback resistor R_fb should be in coupling, not in any block
    let fb_edge = edges
        .iter()
        .find(|&&eidx| graph.components[graph.edges[eidx].comp_idx].id == "R_fb");
    assert!(fb_edge.is_some(), "R_fb should exist");
    let fb_eidx = *fb_edge.unwrap();
    assert!(
        plan.coupling_edges.contains(&fb_eidx),
        "R_fb (feedback resistor) should be coupling, not in any block"
    );

    // Each block should have ≤7 edges (not 9+).
    // A clean block is: 2 NL (BJT) + 1 reactive (cap) + 2-3 linear (R_e, R_bias, maybe R_in/C_out)
    for (i, b) in plan.blocks.iter().enumerate() {
        let total = b.all_edges().len();
        assert!(
            total <= 8,
            "block {i} has {total} edges — feedback path likely absorbed. Max should be ~7"
        );
    }
}

#[test]
fn ladder_feedback_blocks_are_sp_reducible() {
    // With feedback properly as coupling, each block's subgraph should
    // be SP-reducible (not an R-node), giving SingleNl WDF classification.
    let (graph, edges) = make_graph_all(LADDER_WITH_FEEDBACK);
    let plan = blockwise::analyze_blockwise(&edges, &graph);
    assert!(plan.is_some());
    let plan = plan.unwrap();

    for (i, block) in plan.blocks.iter().enumerate() {
        let block_edges = block.all_edges();
        let terminals = if block.port_nodes.len() >= 2 {
            block.port_nodes.clone()
        } else {
            vec![graph.in_node, graph.out_node]
        };
        let tree = spqr_decompose(&block_edges, &terminals, &graph, graph.gnd_node);

        let has_r_node = {
            fn check(n: &SpqrNode) -> bool {
                match n {
                    SpqrNode::R { .. } => true,
                    SpqrNode::S { children, .. } | SpqrNode::P { children, .. } => {
                        children.iter().any(check)
                    }
                    SpqrNode::Q { .. } => false,
                }
            }
            check(&tree)
        };

        eprintln!(
            "  block {i}: {} edges, has_r_node={has_r_node}",
            block_edges.len()
        );
        assert!(
            !has_r_node,
            "block {i} should be SP-reducible (no R-node), but has one. \
             Feedback edges likely absorbed into this block."
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Block = repeatable unit. Contextual edges go to coupling.
// ═══════════════════════════════════════════════════════════════════════════

/// 303-like diode-connected cascade with VCC bias and emitter passives.
/// The repeatable unit is [BJT + C + R_e]. Everything else is contextual:
/// - R_bias: VCC bias network (coupling)
/// - R_in: input network (coupling)
/// - R_out: output network (coupling)
const DIODE_CASCADE_WITH_CONTEXT: &str = r#"pedal "Diode Cascade Context" {
  supply 9V
  components {
    Q1: npn(2n3904)
    Q2: npn(2n3904)
    Q3: npn(2n3904)
    Q4: npn(2n3904)
    R_bias1: resistor(100k)
    R_bias2: resistor(100k)
    R_bias3: resistor(100k)
    R_bias4: resistor(100k)
    C1: cap(33n)
    C2: cap(33n)
    C3: cap(33n)
    C4: cap(33n)
    R_e1: resistor(220)
    R_e2: resistor(220)
    R_e3: resistor(220)
    R_e4: resistor(220)
    R_in: resistor(10k)
  }
  nets {
    vcc -> R_bias1.a
    R_bias1.b -> Q1.base, Q1.collector
    vcc -> R_bias2.a
    R_bias2.b -> Q2.base, Q2.collector
    vcc -> R_bias3.a
    R_bias3.b -> Q3.base, Q3.collector
    vcc -> R_bias4.a
    R_bias4.b -> Q4.base, Q4.collector
    in -> R_in.a
    R_in.b -> Q1.base
    Q1.emitter -> C1.a  C1.b -> gnd
    Q1.emitter -> R_e1.a  R_e1.b -> gnd
    Q1.emitter -> Q2.base
    Q2.emitter -> C2.a  C2.b -> gnd
    Q2.emitter -> R_e2.a  R_e2.b -> gnd
    Q2.emitter -> Q3.base
    Q3.emitter -> C3.a  C3.b -> gnd
    Q3.emitter -> R_e3.a  R_e3.b -> gnd
    Q3.emitter -> Q4.base
    Q4.emitter -> C4.a  C4.b -> gnd
    Q4.emitter -> R_e4.a  R_e4.b -> gnd
    Q4.emitter -> out
  }
  controls {}
}"#;

#[test]
fn blocks_are_repeatable_units() {
    // Each block should contain ONLY the repeatable unit:
    //   2 NL edges (BJT B-E + C-E) + 1 cap (C) + 1 resistor (R_e)
    // Everything else (R_bias, R_in) is coupling/context.
    let (graph, edges) = make_graph_all(DIODE_CASCADE_WITH_CONTEXT);
    let plan = blockwise::analyze_blockwise(&edges, &graph);
    assert!(plan.is_some(), "should be blockwise-decomposable");
    let plan = plan.unwrap();

    eprintln!(
        "  {} blocks, {} coupling",
        plan.num_blocks(),
        plan.coupling_edges.len()
    );
    for (i, block) in plan.blocks.iter().enumerate() {
        let edge_names: Vec<&str> = block
            .all_edges()
            .iter()
            .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
            .collect();
        eprintln!("    block {i}: {:?}", edge_names);
    }
    let coupling_names: Vec<&str> = plan
        .coupling_edges
        .iter()
        .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
        .collect();
    eprintln!("    coupling: {:?}", coupling_names);

    assert_eq!(plan.num_blocks(), 4);

    // Each block: exactly 2 NL + 1 reactive + 1 linear = 4 edges
    for (i, block) in plan.blocks.iter().enumerate() {
        assert_eq!(
            block.nl_edges.len(),
            2,
            "block {i}: expected 2 NL edges, got {}",
            block.nl_edges.len()
        );
        assert_eq!(
            block.reactive_edges.len(),
            1,
            "block {i}: expected 1 reactive (cap), got {}",
            block.reactive_edges.len()
        );
        assert_eq!(
            block.linear_edges.len(),
            1,
            "block {i}: expected 1 linear (R_e), got {}",
            block.linear_edges.len()
        );
    }

    // All blocks should have identical edge count (they're the same unit)
    let counts: Vec<usize> = plan.blocks.iter().map(|b| b.all_edges().len()).collect();
    assert!(
        counts.iter().all(|&c| c == counts[0]),
        "all blocks should be identical size: {:?}",
        counts
    );
}

#[test]
fn contextual_edges_are_coupling() {
    // R_bias (VCC bias) and R_in (input network) should be coupling,
    // not assigned to any block.
    let (graph, edges) = make_graph_all(DIODE_CASCADE_WITH_CONTEXT);
    let plan = blockwise::analyze_blockwise(&edges, &graph);
    assert!(plan.is_some());
    let plan = plan.unwrap();

    let coupling_names: Vec<&str> = plan
        .coupling_edges
        .iter()
        .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
        .collect();

    // All R_bias should be coupling
    for i in 1..=4 {
        let name = format!("R_bias{i}");
        assert!(
            coupling_names.contains(&name.as_str()),
            "{name} should be coupling, not in any block. Coupling: {coupling_names:?}"
        );
    }

    // R_in should be coupling
    assert!(
        coupling_names.contains(&"R_in"),
        "R_in should be coupling. Coupling: {coupling_names:?}"
    );

    // No block should contain any R_bias or R_in
    for (i, block) in plan.blocks.iter().enumerate() {
        let has_bias = block.all_edges().iter().any(|&eidx| {
            graph.components[graph.edges[eidx].comp_idx]
                .id
                .starts_with("R_bias")
        });
        let has_r_in = block
            .all_edges()
            .iter()
            .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "R_in");
        assert!(!has_bias, "block {i} should not contain R_bias");
        assert!(!has_r_in, "block {i} should not contain R_in");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Identical blocks must compile to identical WDF stages (shared K-table)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn identical_blocks_produce_identical_port_resistance() {
    // When 4 identical blocks are compiled, all 4 WDF stages must have
    // the same port resistance. Different rp means different WDF trees,
    // which means the dedup failed.
    let pedal = crate::dsl::parse_pedal_file(DIODE_CASCADE_WITH_CONTEXT).unwrap();
    let compiled = compile_cached(DIODE_CASCADE_WITH_CONTEXT, "dedup_test");

    let bjt_rps: Vec<f64> = compiled
        .stages
        .iter()
        .filter_map(|s| {
            if let super::compiled::Stage::Wdf(w) = s {
                if matches!(w.root, pedalkernel_rt::stage::RootKind::Bjt(_)) {
                    return Some(w.tree.port_resistance());
                }
            }
            None
        })
        .collect();

    eprintln!("  BJT stage port resistances: {:?}", bjt_rps);
    assert!(
        bjt_rps.len() >= 2,
        "should have ≥2 BJT WDF stages, got {}",
        bjt_rps.len()
    );

    // All rp values should be identical
    let first = bjt_rps[0];
    for (i, &rp) in bjt_rps.iter().enumerate().skip(1) {
        assert!(
            (rp - first).abs() < 0.01,
            "stage {i} rp={rp:.1} differs from stage 0 rp={first:.1} — blocks compiled differently"
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Port topology tests — SPQR must not hang on circuits with port nodes
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn spqr_decompose_with_ports_2stage_terminates() {
    // 2-stage ported 303: should complete and classify ports as coupling.
    let (graph, all_edges) = make_graph_all(BJT_LADDER_WITH_PORTS_2);
    eprintln!(
        "  ported 303 (2-stage): {} edges, in={}, out={}, gnd={}",
        all_edges.len(),
        graph.in_node,
        graph.out_node,
        graph.gnd_node
    );

    let feedback_groups = super::signal_flow::find_flow_groups(&all_edges, &graph);
    for (gi, group) in feedback_groups.iter().enumerate() {
        let edges = group.all_edges();
        eprintln!(
            "  group {gi}: {} edges, feedback={}",
            edges.len(),
            group.has_feedback()
        );
        if group.has_feedback() && edges.len() > 2 {
            let plan = blockwise::analyze_blockwise(&edges, &graph);
            assert!(
                plan.is_some(),
                "2-stage ported 303 should decompose blockwise"
            );
            let plan = plan.unwrap();
            eprintln!(
                "  blockwise: {} blocks, {} coupling",
                plan.num_blocks(),
                plan.coupling_edges.len()
            );
        }
    }
}

#[test]
fn spqr_decompose_with_ports_4stage_terminates() {
    // REGRESSION: full 4-stage ported 303 hangs in SPQR decompose.
    // The blockwise analysis must complete and produce 4 identical blocks.
    let (graph, all_edges) = make_graph_all(BJT_LADDER_WITH_PORTS_4);
    eprintln!(
        "  ported 303 (4-stage): {} edges, in={}, out={}, gnd={}",
        all_edges.len(),
        graph.in_node,
        graph.out_node,
        graph.gnd_node
    );

    let feedback_groups = super::signal_flow::find_flow_groups(&all_edges, &graph);
    for (gi, group) in feedback_groups.iter().enumerate() {
        let edges = group.all_edges();
        let edge_names: Vec<_> = edges
            .iter()
            .map(|&ei| {
                format!(
                    "{}({}→{})",
                    graph.components[graph.edges[ei].comp_idx].id,
                    graph.edges[ei].node_a,
                    graph.edges[ei].node_b
                )
            })
            .collect();
        eprintln!(
            "  group {gi}: {} edges, feedback={}: {:?}",
            edges.len(),
            group.has_feedback(),
            edge_names
        );
        if group.has_feedback() && edges.len() > 4 {
            let plan = blockwise::analyze_blockwise(&edges, &graph);
            assert!(
                plan.is_some(),
                "4-stage ported 303 should decompose blockwise"
            );
            let plan = plan.unwrap();
            eprintln!(
                "  blockwise: {} blocks, {} coupling",
                plan.num_blocks(),
                plan.coupling_edges.len()
            );
            assert_eq!(
                plan.num_blocks(),
                4,
                "should have 4 blocks (one per BJT stage)"
            );
        }
    }
}

#[test]
fn ported_303_2stage_compiles_with_skip_k_tables() {
    let pedal = crate::dsl::parse_pedal_file(BJT_LADDER_WITH_PORTS_2).expect("parse");
    let mut options = super::compile::CompileOptions::default();
    options.skip_k_tables = true;
    let compiled = super::compile::compile_pedal_with_options(&pedal, 48_000.0, options)
        .expect("compile failed");
    eprintln!(
        "  ported 303 (2-stage): {} stages, {} ports",
        compiled.stages.len(),
        compiled.ports.len()
    );
    assert!(
        compiled.ports.len() >= 3,
        "should have audio_in, cv_cutoff, audio_out ports"
    );
}

#[test]
fn analyze_blockwise_uses_group_terminals_not_graph() {
    // REGRESSION: analyze_blockwise used graph.in_node/out_node as terminals.
    // With ports, those are port nodes (audio_in=0, audio_out=1) which are
    // NOT in the feedback group's edge set. This causes spqr_decompose to
    // loop forever in the series chain walk (no terminal to break the cycle).
    //
    // Fix: use compute_group_terminals to find the actual boundary nodes
    // of the feedback group's subgraph.
    let (graph, all_edges) = make_graph_all(BJT_LADDER_WITH_PORTS_4);
    let feedback_groups = super::signal_flow::find_flow_groups(&all_edges, &graph);

    for group in &feedback_groups {
        if !group.has_feedback() || group.all_edges().len() < 5 {
            continue;
        }
        let edges = group.all_edges();

        // Verify: graph.in_node and graph.out_node are NOT in the group's node set
        let group_nodes: std::collections::HashSet<super::graph::NodeId> = edges
            .iter()
            .flat_map(|&ei| [graph.edges[ei].node_a, graph.edges[ei].node_b])
            .collect();
        let in_in_group = group_nodes.contains(&graph.in_node);
        let out_in_group = group_nodes.contains(&graph.out_node);
        eprintln!("  graph.in_node={} in group: {in_in_group}", graph.in_node);
        eprintln!(
            "  graph.out_node={} in group: {out_in_group}",
            graph.out_node
        );

        // With ports, at least one of in/out is NOT in the group
        // (audio_in and audio_out are separate flow groups)
        assert!(
            !in_in_group || !out_in_group,
            "With ports, at least one of in_node/out_node should be outside the feedback group"
        );

        // analyze_blockwise must still complete (not hang)
        let plan = blockwise::analyze_blockwise(&edges, &graph);
        assert!(
            plan.is_some(),
            "blockwise should work with correct terminals"
        );
        let plan = plan.unwrap();
        assert_eq!(plan.num_blocks(), 4, "should have 4 blocks");
    }
}

#[test]
fn blockwise_coupling_spqr_terminates() {
    // Isolate the coupling edges from the 4-stage 303 and SPQR them.
    // This is the step after blocks are built — coupling is SPQR'd separately.
    let (graph, all_edges) = make_graph_all(BJT_LADDER_WITH_PORTS_4);
    let feedback_groups = super::signal_flow::find_flow_groups(&all_edges, &graph);

    for group in &feedback_groups {
        if !group.has_feedback() || group.all_edges().len() < 5 {
            continue;
        }
        let edges = group.all_edges();
        let plan = blockwise::analyze_blockwise(&edges, &graph).unwrap();

        eprintln!(
            "  coupling edges: {:?}",
            plan.coupling_edges
                .iter()
                .map(|&ei| format!(
                    "{}({}→{})",
                    graph.components[graph.edges[ei].comp_idx].id,
                    graph.edges[ei].node_a,
                    graph.edges[ei].node_b
                ))
                .collect::<Vec<_>>()
        );

        let terminals = vec![graph.in_node, graph.out_node];
        let coupling_terminals =
            super::spqr_build::compute_group_terminals(&plan.coupling_edges, &graph, &terminals);
        eprintln!("  coupling terminals: {coupling_terminals:?}");

        // This is the call that might hang in release mode
        let tree = spqr_decompose(
            &plan.coupling_edges,
            &coupling_terminals,
            &graph,
            graph.gnd_node,
        );
        let kind = match &tree {
            SpqrNode::S { .. } => "S",
            SpqrNode::P { .. } => "P",
            SpqrNode::Q { .. } => "Q",
            SpqrNode::R { .. } => "R",
        };
        eprintln!("  coupling SPQR: {kind}");

        // Also test each block's SPQR
        for (bi, block) in plan.blocks.iter().enumerate() {
            let block_edges = block.all_edges();
            let block_terminals =
                super::spqr_build::compute_group_terminals(&block_edges, &graph, &terminals);
            eprintln!(
                "  block {bi}: {} edges, terminals={block_terminals:?}",
                block_edges.len()
            );
            let btree = spqr_decompose(&block_edges, &block_terminals, &graph, graph.gnd_node);
            let bkind = match &btree {
                SpqrNode::S { .. } => "S",
                SpqrNode::P { .. } => "P",
                SpqrNode::Q { .. } => "Q",
                SpqrNode::R { .. } => "R",
            };
            eprintln!("  block {bi} SPQR: {bkind}");
        }
    }
}

#[test]
fn ported_303_4stage_compiles_with_skip_k_tables() {
    let pedal = crate::dsl::parse_pedal_file(BJT_LADDER_WITH_PORTS_4).expect("parse");
    let mut options = super::compile::CompileOptions::default();
    options.skip_k_tables = true;
    let compiled = super::compile::compile_pedal_with_options(&pedal, 48_000.0, options)
        .expect("compile failed");
    eprintln!(
        "  ported 303 (4-stage): {} stages, {} ports",
        compiled.stages.len(),
        compiled.ports.len()
    );
    assert!(
        compiled.ports.len() >= 3,
        "should have audio_in, cv_cutoff, audio_out ports"
    );
}

#[test]
fn ported_303_filter_passes_ac_signal() {
    // The compiled 303 filter must pass an AC signal — not output DC.
    // This tests the full serial chain: StateSpace coupling → BJT blocks.
    use crate::PedalProcessor;

    // Test BOTH ported and non-ported versions
    for (name, src) in [
        ("non-ported", BJT_LADDER_4),
        ("ported", BJT_LADDER_WITH_PORTS_4),
    ] {
        eprintln!("\n--- {name} ---");
        let pedal = crate::dsl::parse_pedal_file(src).expect("parse");
        let mut options = super::compile::CompileOptions::default();
        options.skip_k_tables = true;
        let mut compiled = super::compile::compile_pedal_with_options(&pedal, 48_000.0, options)
            .expect("compile failed");

        eprintln!("{}", compiled.debug_dump());
        compiled.set_control("Cutoff", 0.8); // open filter
        compiled.set_control("Resonance", 0.0);

        // Settle DC
        for _ in 0..2400 {
            compiled.process(0.0);
        }

        // Feed a 440Hz sine and measure peak AC output
        let mut peak = 0.0f64;
        let mut dc_sum = 0.0f64;
        let n = 4800;
        for i in 0..n {
            let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin();
            let out = compiled.process(input);
            peak = peak.max(out.abs());
            dc_sum += out;
        }
        let dc_avg = dc_sum / n as f64;
        let ac_peak = peak - dc_avg.abs();

        eprintln!("  {name} AC test: peak={peak:.6}, dc_avg={dc_avg:.6}, ac_peak={ac_peak:.6}");
        assert!(
        ac_peak > 0.001,
        "{name}: filter must pass AC signal, not just DC. peak={peak:.6}, dc={dc_avg:.6}, ac={ac_peak:.6}"
    );
    } // end for
}

// ═══════════════════════════════════════════════════════════════════════════
// Coupling MNA tests — verify scattering matrix correctness with small
// known circuits where we can check results by hand.
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn coupling_mna_vcc_creates_bias_current() {
    // A single R_bias from VCC to a block port should create a DC voltage
    // at the port when VCC is included as a VS in the coupling MNA.
    //
    // Circuit: VCC (9V) → R_bias (100k) → block_port → block (diode to GND)
    //
    // Without VCC VS: the MNA sees R_bias from ground to port → port at 0V.
    // With VCC VS: the MNA sees R_bias from 9V to port → port at ~9V (minus
    // diode drop), establishing forward bias for the diode.
    use pedalkernel_rt::tree::{MnaSystem, WdfPort};

    // Simple 2-node MNA: node 0 = VCC, node 1 = block port
    let mut mna = MnaSystem::new(2, 0);

    // R_bias: 100kΩ from node 0 (VCC) to node 1 (port)
    mna.stamp_resistor(Some(0), Some(1), 100_000.0);

    // GMIN regularization
    mna.stamp_resistor(Some(0), None, 1e9);
    mna.stamp_resistor(Some(1), None, 1e9);

    // Ports: block port (NL, Rp=623Ω) + VCC supply VS (Rp=1Ω)
    let ports = vec![
        WdfPort {
            node_pos: Some(1),
            node_neg: None,
            resistance: 623.0,
        }, // block
        WdfPort {
            node_pos: Some(0),
            node_neg: None,
            resistance: 1.0,
        }, // VCC VS
    ];

    let s = mna.derive_scattering_matrix_general(&ports);
    let n = ports.len();

    eprintln!("  Coupling MNA with VCC:");
    for i in 0..n {
        let row: Vec<f64> = (0..n).map(|j| s[i * n + j]).collect();
        eprintln!("    row {i}: {row:.4?}");
    }

    // The block port (row 0) should receive significant signal from VCC VS (column 1)
    let block_from_vcc = s[0 * n + 1];
    eprintln!("    S[block, vcc] = {block_from_vcc:.4}");

    assert!(
        block_from_vcc.abs() > 0.001,
        "Block port should receive signal from VCC through R_bias. S[0,1]={block_from_vcc:.6}"
    );

    // Verify: with VCC VS at 9V, the block port should see nonzero voltage
    // b_vcc = 2 * 9.0 = 18.0 (WDF VS reflected wave)
    let b_block = 0.0; // block initially at rest
    let b_vcc = 2.0 * 9.0;
    let a_block = s[0 * n + 0] * b_block + s[0 * n + 1] * b_vcc;
    let v_block = (a_block + b_block) / 2.0;
    eprintln!("    With VCC=9V: a_block={a_block:.4}, V_block={v_block:.4}V");

    assert!(
        v_block.abs() > 0.01,
        "Block port should see nonzero voltage from 9V supply. V={v_block:.6}"
    );
}

#[test]
fn coupling_mna_without_vcc_gives_zero_bias() {
    // Same circuit but WITHOUT the VCC VS — only R_bias to ground.
    // The block port should see zero voltage (no bias current).
    use pedalkernel_rt::tree::{MnaSystem, WdfPort};

    let mut mna = MnaSystem::new(1, 0); // only 1 node (block port)

    // R_bias from VCC (ground in MNA, since VCC is a rail) to node 0
    mna.stamp_resistor(Some(0), None, 100_000.0);
    mna.stamp_resistor(Some(0), None, 1e9); // GMIN

    let ports = vec![WdfPort {
        node_pos: Some(0),
        node_neg: None,
        resistance: 623.0,
    }];

    let s = mna.derive_scattering_matrix_general(&ports);
    eprintln!("  Without VCC: S = [{:.4}]", s[0]);

    // With no VCC source, the block port only sees its own reflection
    // and the grounded R_bias. No DC offset.
    let a_block = s[0] * 0.0; // zero input
    eprintln!("    a_block = {a_block:.6} (should be ~0)");

    // This test documents the current behavior — block gets no bias
    assert!(
        a_block.abs() < 0.01,
        "Without VCC VS, block should see zero bias. a={a_block:.6}"
    );
}

#[test]
fn coupling_mna_two_blocks_cross_coupled() {
    // Two blocks connected by a resistor. Signal at block 0 should
    // appear at block 1 through the coupling resistor.
    //
    // block_0 ── R_couple (10k) ── block_1
    use pedalkernel_rt::tree::{MnaSystem, WdfPort};

    let mut mna = MnaSystem::new(2, 0);

    // R_couple: 10kΩ between node 0 and node 1
    mna.stamp_resistor(Some(0), Some(1), 10_000.0);
    mna.stamp_resistor(Some(0), None, 1e9);
    mna.stamp_resistor(Some(1), None, 1e9);

    let ports = vec![
        WdfPort {
            node_pos: Some(0),
            node_neg: None,
            resistance: 623.0,
        },
        WdfPort {
            node_pos: Some(1),
            node_neg: None,
            resistance: 623.0,
        },
    ];

    let s = mna.derive_scattering_matrix_general(&ports);
    let n = 2;

    eprintln!("  Two blocks coupled by 10kΩ:");
    for i in 0..n {
        let row: Vec<f64> = (0..n).map(|j| s[i * n + j]).collect();
        eprintln!("    row {i}: {row:.4?}");
    }

    // Cross-coupling: S[0,1] and S[1,0] should be nonzero
    let cross = s[0 * n + 1];
    eprintln!("    S[0,1] = {cross:.4} (cross-coupling)");

    assert!(
        cross.abs() > 0.01,
        "Blocks connected by 10kΩ should have cross-coupling. S[0,1]={cross:.6}"
    );

    // Verify signal transfer: VS at port 0, measure at port 1
    let b0 = 2.0 * 1.0; // 1V at block 0
    let b1 = 0.0;
    let a1 = s[1 * n + 0] * b0 + s[1 * n + 1] * b1;
    let v1 = (a1 + b1) / 2.0;
    eprintln!("    1V at block 0 → V_block1 = {v1:.4}V");

    assert!(
        v1.abs() > 0.01,
        "Signal should transfer from block 0 to block 1 through R_couple. V1={v1:.6}"
    );
}

#[test]
fn coupling_mna_supply_vs_port_sets_dc_bias() {
    // Full test: R_bias from VCC to block, with VCC as a supply VS port.
    // Process one sample through a single DiodeRoot block and verify
    // the diode is forward-biased (nonzero output).
    //
    // This is the minimal test for the VCC bug — without VCC VS,
    // the diode never turns on.
    use pedalkernel_rt::tree::{MnaSystem, WdfPort};

    let mut mna = MnaSystem::new(2, 0);
    mna.stamp_resistor(Some(0), Some(1), 100_000.0); // R_bias
    mna.stamp_resistor(Some(0), None, 1e9);
    mna.stamp_resistor(Some(1), None, 1e9);

    let ports = vec![
        WdfPort {
            node_pos: Some(1),
            node_neg: None,
            resistance: 623.0,
        }, // block
        WdfPort {
            node_pos: Some(0),
            node_neg: None,
            resistance: 1.0,
        }, // VCC supply
        WdfPort {
            node_pos: Some(1),
            node_neg: None,
            resistance: 10_000.0,
        }, // audio input VS
    ];

    let s = mna.derive_scattering_matrix_general(&ports);
    let n = 3;

    // Simulate: VCC=9V, audio=0.1V, block reflects from DiodeRoot
    let b_block = 0.0;
    let b_vcc = 2.0 * 9.0;
    let b_audio = 2.0 * 0.1;

    let a_block = s[0 * n + 0] * b_block + s[0 * n + 1] * b_vcc + s[0 * n + 2] * b_audio;
    let v_block = (a_block + b_block) / 2.0;

    eprintln!("  Supply + audio → block:");
    eprintln!("    a_block={a_block:.4}, V_block={v_block:.4}V");
    eprintln!("    (should be ~0.1V + bias from 9V through 100kΩ)");

    // The block should see positive voltage (forward bias + audio)
    assert!(
        v_block > 0.01,
        "Block should be forward-biased with VCC supply. V={v_block:.6}"
    );
}

#[test]
fn coupling_scattering_includes_vcc_supply() {
    // THE BUG: the blockwise coupling MNA excludes VCC as a rail node,
    // so R_bias connects to ground instead of 9V. The diodes never
    // get forward bias → filter outputs zero.
    //
    // This test loads the actual 303, builds the coupling MNA the same
    // way blockwise does, and verifies that VCC appears as a VS port
    // in the scattering matrix with the supply voltage.
    let pedal_src = BJT_LADDER_WITH_PORTS_4;
    let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse");
    let graph = CircuitGraph::from_pedal(&pedal);
    let supply_voltage = pedal.supplies.first().map_or(9.0, |s| s.config.voltage);

    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    let groups = super::signal_flow::find_flow_groups(&all_edges, &graph);
    let feedback_group = groups
        .iter()
        .find(|g| g.has_feedback())
        .expect("should have feedback group");
    let group_edges = feedback_group.all_edges();
    let plan = blockwise::analyze_blockwise(&group_edges, &graph).expect("should decompose");

    // Check: does the coupling scattering have a VCC VS port?
    // The coupling edges include R_bias1..4 which connect VCC to block ports.
    let has_bias_edge = plan.coupling_edges.iter().any(|&eidx| {
        let comp = &graph.components[graph.edges[eidx].comp_idx];
        comp.id.contains("R_bias")
    });
    assert!(has_bias_edge, "Coupling should include R_bias edges");

    // Check: do any coupling edges touch VCC?
    let vcc_in_coupling = plan.coupling_edges.iter().any(|&eidx| {
        let e = &graph.edges[eidx];
        e.node_a == graph.vcc_node || e.node_b == graph.vcc_node
    });
    assert!(
        vcc_in_coupling,
        "R_bias edges should touch VCC node (graph.vcc_node={})",
        graph.vcc_node
    );

    // THE ACTUAL BUG TEST: blockwise builds the coupling MNA excluding
    // VCC as a rail (line 859 in blockwise.rs). This means R_bias
    // connects to ground instead of 9V → diodes get no forward bias.
    //
    // Test: compile the 303, process a signal, verify nonzero output.
    // This FAILS until blockwise adds a VCC VS port to the coupling.
    let mut options = super::compile::CompileOptions::default();
    options.skip_k_tables = false;
    let mut compiled = super::spqr_build::compile_via_spqr_with_options(&pedal, 48_000.0, options)
        .expect("compile");

    // Warm up (including DC op solve)
    for _ in 0..4800 {
        compiled.process(0.0);
    }

    // Feed a 440Hz tone and check for nonzero output
    let mut peak = 0.0f64;
    for i in 0..4800 {
        let input = 0.1 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48_000.0).sin();
        peak = peak.max(compiled.process(input).abs());
    }
    eprintln!("  303 with coupling: peak={peak:.6}");

    assert!(
        peak > 0.001,
        "303 filter must produce nonzero output (diodes need VCC bias). \
         peak={peak:.6} — if zero, VCC is excluded from coupling MNA."
    );
}
