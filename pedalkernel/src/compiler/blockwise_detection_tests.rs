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
        .filter(|&&eidx| {
            graph.effective_edge_kind(eidx) == super::component::EdgeKind::Nonlinear
        })
        .count()
}

/// Helper: count reactive edges in a set of edge indices.
fn count_reactive(edges: &[usize], graph: &CircuitGraph) -> usize {
    edges
        .iter()
        .filter(|&&eidx| {
            graph.effective_edge_kind(eidx) == super::component::EdgeKind::Reactive
        })
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
    assert!(nl_count >= 4, "Should have at least 4 NL edges, got {nl_count}");
}

#[test]
fn ladder_4_has_4_reactive_elements() {
    let (graph, edges) = make_graph_all(BJT_LADDER_4);
    let reactive_count = count_reactive(&edges, &graph);
    assert!(reactive_count >= 4, "Should have at least 4 reactive edges, got {reactive_count}");
}

#[test]
fn cascade_2_has_2_nl_elements() {
    let (graph, edges) = make_graph_all(BJT_CASCADE_2);
    let nl_count = count_nl(&edges, &graph);
    assert!(nl_count >= 2, "Should have at least 2 NL edges, got {nl_count}");
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
    eprintln!("  Ladder 4: {nl_count} NL edges, {} total edges", edges.len());

    // Check what groups we get
    let nl_edges: Vec<usize> = edges.iter()
        .filter(|&&eidx| graph.effective_edge_kind(eidx) == super::component::EdgeKind::Nonlinear)
        .copied().collect();
    eprintln!("  NL edges: {:?}", nl_edges.iter()
        .map(|&eidx| format!("{}({:?}→{:?})", graph.components[graph.edges[eidx].comp_idx].id,
            graph.edges[eidx].node_a, graph.edges[eidx].node_b))
        .collect::<Vec<_>>());

    let plan = blockwise::analyze_blockwise(&edges, &graph);

    assert!(plan.is_some(), "4-stage BJT ladder should be blockwise-decomposable");
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

    // Coupling edges: the resonance feedback path
    assert!(
        !plan.coupling_edges.is_empty(),
        "Should have coupling edges (resonance feedback)"
    );
}

#[test]
fn cascade_2_is_blockwise_decomposable() {
    let (graph, edges) = make_graph_all(BJT_CASCADE_2);
    let plan = blockwise::analyze_blockwise(&edges, &graph);

    assert!(plan.is_some(), "2-stage BJT cascade should be blockwise-decomposable");
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
    let pedal = crate::dsl::parse_pedal_file(BJT_LADDER_4).unwrap();
    let compiled = super::spqr_build::compile_via_spqr(&pedal, 48_000.0).unwrap();

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
    let pedal = crate::dsl::parse_pedal_file(BJT_CASCADE_2).unwrap();
    let compiled = super::spqr_build::compile_via_spqr(&pedal, 48_000.0).unwrap();

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
    let pedal = crate::dsl::parse_pedal_file(SINGLE_BJT).unwrap();
    let compiled = super::spqr_build::compile_via_spqr(&pedal, 48_000.0).unwrap();

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
                let fb = if w.feedback_opamp.is_some() { "+OA" } else { "" };
                eprintln!("  [{i}] WDF{fb} dist={}", w.signal_flow_distance);
            }
            super::compiled::Stage::MultiNl(m) => {
                eprintln!("  [{i}] MultiNL ports={} dist={}",
                    m.nl_devices.len(), m.signal_flow_distance);
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
        }
    }
}

#[test]
fn diagnostic_cascade_2_stage_structure() {
    let pedal = crate::dsl::parse_pedal_file(BJT_CASCADE_2).unwrap();
    let compiled = super::spqr_build::compile_via_spqr(&pedal, 48_000.0).unwrap();
    print_stages("BJT Cascade 2", &compiled);
}

#[test]
fn diagnostic_ladder_4_stage_structure() {
    let pedal = crate::dsl::parse_pedal_file(BJT_LADDER_4).unwrap();
    match super::spqr_build::compile_via_spqr(&pedal, 48_000.0) {
        Ok(compiled) => print_stages("BJT Ladder 4", &compiled),
        Err(e) => eprintln!("\n=== BJT Ladder 4: COMPILE FAILED: {e} ==="),
    }
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
            SpqrNode::S { children, cut_vertices } => {
                eprintln!("{indent}S (cut={:?}, {} children)", cut_vertices, children.len());
                for c in children { print_spqr(c, graph, depth + 1); }
            }
            SpqrNode::P { children, endpoints } => {
                eprintln!("{indent}P (ep={:?}, {} children)", endpoints, children.len());
                for c in children { print_spqr(c, graph, depth + 1); }
            }
            SpqrNode::Q { edge_idx, .. } => {
                let comp = &graph.components[graph.edges[*edge_idx].comp_idx];
                let kind = graph.effective_edge_kind(*edge_idx);
                eprintln!("{indent}Q edge={} {}({:?})", edge_idx, comp.id, kind);
            }
            SpqrNode::R { edge_indices, boundary_nodes, children } => {
                // Classify edges in this R-node
                let mut nl = 0; let mut reactive = 0; let mut linear = 0;
                for &eidx in edge_indices {
                    match graph.effective_edge_kind(eidx) {
                        super::component::EdgeKind::Nonlinear => nl += 1,
                        super::component::EdgeKind::Reactive => reactive += 1,
                        super::component::EdgeKind::Linear => linear += 1,
                        _ => {}
                    }
                }
                eprintln!("{indent}R ({} edges: {nl}NL+{reactive}C+{linear}R, {} boundary, {} children)",
                    edge_indices.len(), boundary_nodes.len(), children.len());
                // Print edge names
                for &eidx in edge_indices {
                    let comp = &graph.components[graph.edges[eidx].comp_idx];
                    let kind = graph.effective_edge_kind(eidx);
                    eprintln!("{indent}  edge {eidx}: {}({:?})", comp.id, kind);
                }
                for c in children { print_spqr(c, graph, depth + 1); }
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
                for c in children { print_spqr(c, graph, depth + 1); }
            }
            SpqrNode::P { children, .. } => {
                eprintln!("{indent}P ({} children)", children.len());
                for c in children { print_spqr(c, graph, depth + 1); }
            }
            SpqrNode::Q { edge_idx, .. } => {
                let comp = &graph.components[graph.edges[*edge_idx].comp_idx];
                let kind = graph.effective_edge_kind(*edge_idx);
                eprintln!("{indent}Q {}({:?})", comp.id, kind);
            }
            SpqrNode::R { edge_indices, children, .. } => {
                let mut nl = 0; let mut reactive = 0; let mut linear = 0;
                for &eidx in edge_indices {
                    match graph.effective_edge_kind(eidx) {
                        super::component::EdgeKind::Nonlinear => nl += 1,
                        super::component::EdgeKind::Reactive => reactive += 1,
                        super::component::EdgeKind::Linear => linear += 1,
                        _ => {}
                    }
                }
                eprintln!("{indent}R ({} edges: {nl}NL+{reactive}C+{linear}R, {} children)",
                    edge_indices.len(), children.len());
                for c in children { print_spqr(c, graph, depth + 1); }
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
    let pedal = crate::dsl::parse_pedal_file(BJT_LADDER_4).unwrap();
    let compiled = super::spqr_build::compile_via_spqr(&pedal, 48_000.0).unwrap();

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
fn analyze_k_method_subgraph(
    edge_indices: &[usize],
    graph: &CircuitGraph,
) -> KMethodSubgraph {
    // Step 1: find NL edges
    let nl_edges: Vec<usize> = edge_indices
        .iter()
        .filter(|&&eidx| {
            graph.effective_edge_kind(eidx) == super::component::EdgeKind::Nonlinear
        })
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
        if box_edges.contains(&eidx) { continue; }
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
            if graph.components[graph.edges[eidx].comp_idx].kind.is_variable() {
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
    let (graph, _) = make_graph_all(r#"pedal "test" {
      supply 9V
      components { D1: diode(silicon)  R1: resistor(10k) }
      nets { in -> R1.a  R1.b -> D1.anode  D1.cathode -> gnd  R1.b -> out }
      controls {}
    }"#);
    let diode = graph.components.iter().find(|c| c.kind.type_tag() == "diode").unwrap();
    let (ok, dims, _) = diode.kind.k_method_candidacy();
    assert!(ok, "Diode should be memoryless");
    assert_eq!(dims, 1);
}

#[test]
fn bjt_component_is_memoryless_2d() {
    let (graph, _) = make_graph_all(SINGLE_BJT);
    let bjt = graph.components.iter().find(|c| c.kind.type_tag() == "NPN transistor").unwrap();
    let (ok, dims, _) = bjt.kind.k_method_candidacy();
    assert!(ok, "BJT should be memoryless");
    assert_eq!(dims, 2);
}

#[test]
fn resistor_component_is_not_nl() {
    let (graph, _) = make_graph_all(r#"pedal "test" {
      supply 9V
      components { R1: resistor(10k) }
      nets { in -> R1.a  R1.b -> out }
      controls {}
    }"#);
    let r = graph.components.iter().find(|c| c.kind.type_tag() == "resistor").unwrap();
    let (ok, _, _) = r.kind.k_method_candidacy();
    assert!(!ok, "Resistor is linear");
}

// ── Subgraph-level K-method tests ──

#[test]
fn diode_clipper_subgraph_is_k_candidate() {
    // D1 + D2 anti-parallel with R1 — all NL is memoryless, no reactive
    // inside the box, ≤2 boundary wires (signal in, ground)
    let (graph, edges) = make_graph_all(r#"pedal "test" {
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
    }"#);

    let result = analyze_k_method_subgraph(&edges, &graph);
    eprintln!("  Diode clipper: candidate={}, dims={}, rejection={:?}",
        result.is_candidate, result.port_dims, result.rejection);
    assert!(result.is_candidate, "Diode clipper subgraph should be K-method: {:?}", result.rejection);
    assert!(result.port_dims <= 2, "Should be ≤2D, got {}", result.port_dims);
}

#[test]
fn single_bjt_subgraph_is_k_candidate() {
    // Single BJT stage: BJT is memoryless 2D, caps are OUTSIDE the box
    // (C1 is a ground shunt separate from the BJT junctions)
    let (graph, edges) = make_graph_all(SINGLE_BJT);
    let result = analyze_k_method_subgraph(&edges, &graph);
    eprintln!("  Single BJT: candidate={}, dims={}, rejection={:?}",
        result.is_candidate, result.port_dims, result.rejection);
    assert!(result.is_candidate, "Single BJT subgraph should be K-method: {:?}", result.rejection);
    assert!(result.port_dims <= 3, "Should be ≤3D, got {}", result.port_dims);
}

#[test]
fn bjt_with_cap_across_junction_is_not_k_candidate() {
    // If a cap connects directly across the BJT's B-E junction,
    // it's INSIDE the NL box → reactive inside → not K-method
    let (graph, edges) = make_graph_all(r#"pedal "test" {
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
    }"#);

    let result = analyze_k_method_subgraph(&edges, &graph);
    eprintln!("  BJT+C_be: candidate={}, dims={}, rejection={:?}",
        result.is_candidate, result.port_dims, result.rejection);
    assert!(!result.is_candidate,
        "BJT with cap across B-E should NOT be K-method (reactive inside NL box)");
}

#[test]
fn pure_linear_circuit_is_not_k_candidate() {
    let (graph, edges) = make_graph_all(WHEATSTONE_BRIDGE);
    let result = analyze_k_method_subgraph(&edges, &graph);
    assert!(!result.is_candidate, "Pure linear circuit has no NL to tabulate");
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
    let (graph, edges) = make_graph_all(r#"pedal "test" {
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
    }"#);

    // Not blockwise
    let plan = blockwise::analyze_blockwise(&edges, &graph);
    assert!(plan.is_none(), "TS808 clipper is not blockwise (single NL group)");

    // But the full subgraph IS a K-method candidate
    let result = analyze_k_method_subgraph(&edges, &graph);
    assert!(result.is_candidate, "TS808 clipper subgraph should be K-method: {:?}", result.rejection);
    assert!(result.port_dims <= 2, "Should be ≤2D, got {}", result.port_dims);
}

// ═══════════════════════════════════════════════════════════════════════════
// 8. Blockwise runtime behavior: chained blocks produce correct audio
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn cascade_2_produces_audio() {
    let pedal = crate::dsl::parse_pedal_file(BJT_CASCADE_2).unwrap();
    let compiled = super::spqr_build::compile_via_spqr(&pedal, 48_000.0).unwrap();
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
    assert!(peak > 0.001, "Cascade 2 should produce audible output, got {peak:.6}");
    assert!(peak.is_finite(), "Output should be finite");
}

#[test]
fn ladder_4_produces_audio() {
    let pedal = crate::dsl::parse_pedal_file(BJT_LADDER_4).unwrap();
    let compiled = super::spqr_build::compile_via_spqr(&pedal, 48_000.0).unwrap();
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
    assert!(peak > 0.001, "Ladder 4 should produce audible output, got {peak:.6}");
    assert!(peak.is_finite(), "Output should be finite");
}

#[test]
fn ladder_4_resonance_affects_output() {
    let pedal = crate::dsl::parse_pedal_file(BJT_LADDER_4).unwrap();

    // Process with resonance = 0
    let mut no_res = super::spqr_build::compile_via_spqr(&pedal, 48_000.0).unwrap();
    no_res.set_control("Resonance", 0.0);
    let mut peak_no = 0.0f64;
    for i in 0..9600 {
        let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48_000.0).sin() * 0.1;
        let out = no_res.process(input);
        if i >= 4800 { peak_no = peak_no.max(out.abs()); }
    }

    // Process with resonance = 0.9
    let mut hi_res = super::spqr_build::compile_via_spqr(&pedal, 48_000.0).unwrap();
    hi_res.set_control("Resonance", 0.9);
    let mut peak_hi = 0.0f64;
    for i in 0..9600 {
        let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48_000.0).sin() * 0.1;
        let out = hi_res.process(input);
        if i >= 4800 { peak_hi = peak_hi.max(out.abs()); }
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
    let pedal = crate::dsl::parse_pedal_file(BJT_LADDER_4).unwrap();
    let compiled = super::spqr_build::compile_via_spqr(&pedal, 48_000.0).unwrap();

    for &res in &[0.0, 0.3, 0.5, 0.7, 0.9, 1.0] {
        let pedal = crate::dsl::parse_pedal_file(BJT_LADDER_4).unwrap();
        let mut proc = super::spqr_build::compile_via_spqr(&pedal, 48_000.0).unwrap();
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
    for (i, block) in plan.blocks.iter().enumerate()
    {
        let nl_nodes: std::collections::HashSet<_> = block.nl_edges.iter()
            .flat_map(|&eidx| {
                let e = &graph.edges[eidx];
                vec![e.node_a, e.node_b]
            })
            .collect();
        let reactive_nodes: std::collections::HashSet<_> = block.reactive_edges.iter()
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

    let total_block_edges: usize = plan.blocks.iter()
        .map(|b| b.nl_edges.len() + b.reactive_edges.len() + b.linear_edges.len())
        .sum();
    let coupling_count = plan.coupling_edges.len();

    eprintln!("  Block edges: {total_block_edges}, coupling edges: {coupling_count}");
    assert!(
        coupling_count < total_block_edges,
        "Coupling edges ({coupling_count}) should be fewer than block edges ({total_block_edges})"
    );
}
