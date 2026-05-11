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

use super::graph::CircuitGraph;
use super::spqr::*;

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
// Future API: `analyze_blockwise(r_node, graph) -> Option<BlockwisePlan>`
// For now, we define the expected structure and test it.
// ═══════════════════════════════════════════════════════════════════════════

/// A blockwise decomposition plan for an R-node.
/// This is the target data structure — doesn't exist yet.
#[derive(Debug)]
struct BlockwisePlan {
    /// Number of chained blocks.
    num_blocks: usize,
    /// NL edge indices per block (each block should have ≤ 2 NL ports).
    block_nl_edges: Vec<Vec<usize>>,
    /// Reactive edge indices per block.
    block_reactive_edges: Vec<Vec<usize>>,
    /// Coupling edges (feedback, resonance) — the residual graph.
    coupling_edges: Vec<usize>,
}

/// Placeholder for the detection function that will be implemented.
/// For now, returns None (no blockwise detected).
fn analyze_blockwise(
    _edge_indices: &[usize],
    _graph: &CircuitGraph,
) -> Option<BlockwisePlan> {
    // TODO: implement blockwise detection from SPQR structure
    None
}

#[test]
fn ladder_4_is_blockwise_decomposable() {
    let (graph, edges) = make_graph_all(BJT_LADDER_4);
    let plan = analyze_blockwise(&edges, &graph);

    assert!(plan.is_some(), "4-stage BJT ladder should be blockwise-decomposable");
    let plan = plan.unwrap();

    assert_eq!(plan.num_blocks, 4, "Should decompose into 4 blocks");

    // Each block should have exactly 1 BJT (1-2 NL edges) and 1 cap
    for (i, nl) in plan.block_nl_edges.iter().enumerate() {
        assert!(
            nl.len() >= 1 && nl.len() <= 2,
            "Block {i} should have 1-2 NL edges, got {}",
            nl.len()
        );
    }
    for (i, reactive) in plan.block_reactive_edges.iter().enumerate() {
        assert!(
            reactive.len() >= 1,
            "Block {i} should have at least 1 reactive edge, got {}",
            reactive.len()
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
    let plan = analyze_blockwise(&edges, &graph);

    assert!(plan.is_some(), "2-stage BJT cascade should be blockwise-decomposable");
    let plan = plan.unwrap();

    assert_eq!(plan.num_blocks, 2, "Should decompose into 2 blocks");
}

#[test]
fn single_bjt_is_not_blockwise() {
    let (graph, edges) = make_graph_all(SINGLE_BJT);
    let plan = analyze_blockwise(&edges, &graph);

    assert!(
        plan.is_none(),
        "Single BJT should NOT be blockwise (only 1 NL element, no cascade)"
    );
}

#[test]
fn wheatstone_bridge_is_not_blockwise() {
    let (graph, edges) = make_graph_all(WHEATSTONE_BRIDGE);
    let plan = analyze_blockwise(&edges, &graph);

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

/// K-method candidacy result for a nonlinear component.
#[derive(Debug, Clone, PartialEq)]
struct KMethodCandidacy {
    /// Can this NL be tabulated? (memoryless, monotonic, ≤3D)
    is_candidate: bool,
    /// Number of port dimensions (1D for diode, 2D for BJT, etc.)
    port_dims: usize,
    /// Reason for rejection, if not a candidate.
    rejection: Option<&'static str>,
}

/// Query K-method candidacy from the Component trait.
fn k_method_candidacy(comp_idx: usize, graph: &CircuitGraph) -> KMethodCandidacy {
    let comp = &graph.components[comp_idx];
    let (is_candidate, port_dims, reason) = comp.kind.k_method_candidacy();
    KMethodCandidacy {
        is_candidate,
        port_dims,
        rejection: if is_candidate { None } else { Some(reason) },
    }
}

#[test]
fn diode_is_k_method_candidate() {
    let (graph, edges) = make_graph_all(r#"pedal "test" {
      supply 9V
      components { D1: diode(silicon)  R1: resistor(10k) }
      nets { in -> R1.a  R1.b -> D1.anode  D1.cathode -> gnd  R1.b -> out }
      controls {}
    }"#);

    // Find the diode component
    let diode_comp = graph.components.iter().position(|c| c.kind.type_tag() == "diode");
    assert!(diode_comp.is_some(), "Should have a diode component");
    let result = k_method_candidacy(diode_comp.unwrap(), &graph);

    assert!(result.is_candidate, "Diode should be K-method candidate: {:?}", result.rejection);
    assert_eq!(result.port_dims, 1, "Diode is 1D (single junction voltage)");
}

#[test]
fn bjt_is_k_method_candidate_2d() {
    let (graph, _edges) = make_graph_all(SINGLE_BJT);

    let bjt_comp = graph.components.iter().position(|c| c.kind.type_tag() == "NPN transistor");
    assert!(bjt_comp.is_some(), "Should have an NPN component");
    let result = k_method_candidacy(bjt_comp.unwrap(), &graph);

    assert!(result.is_candidate, "BJT should be K-method candidate: {:?}", result.rejection);
    assert_eq!(result.port_dims, 2, "BJT is 2D (Vbe, Vce)");
}

#[test]
fn resistor_is_not_k_method_candidate() {
    let (graph, _edges) = make_graph_all(r#"pedal "test" {
      supply 9V
      components { R1: resistor(10k) }
      nets { in -> R1.a  R1.b -> out }
      controls {}
    }"#);

    let r_comp = graph.components.iter().position(|c| c.kind.type_tag() == "resistor");
    assert!(r_comp.is_some());
    let result = k_method_candidacy(r_comp.unwrap(), &graph);

    assert!(!result.is_candidate, "Resistor is linear, not NL — not a K-method target");
}

// ═══════════════════════════════════════════════════════════════════════════
// 7. Combined: blockwise + K-method gives the full optimization plan
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn ladder_4_each_block_is_k_method_eligible() {
    let (graph, edges) = make_graph_all(BJT_LADDER_4);
    let plan = analyze_blockwise(&edges, &graph);

    // First, blockwise must succeed
    assert!(plan.is_some(), "Ladder should be blockwise-decomposable");
    let plan = plan.unwrap();

    // Each block's NL edges should be K-method candidates
    for (i, nl_edges) in plan.block_nl_edges.iter().enumerate() {
        for &eidx in nl_edges {
            let comp_idx = graph.edges[eidx].comp_idx;
            let candidacy = k_method_candidacy(comp_idx, &graph);
            assert!(
                candidacy.is_candidate,
                "Block {i} NL edge {eidx} ({}) should be K-method candidate: {:?}",
                graph.components[comp_idx].id,
                candidacy.rejection,
            );
        }
    }
}

#[test]
fn ts808_clipper_is_k_method_without_blockwise() {
    // TS808 diode clipper: single NL element, no cascade.
    // K-method candidate but NOT blockwise (only 1 block).
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

    // Not blockwise (no cascade)
    let plan = analyze_blockwise(&edges, &graph);
    assert!(plan.is_none(), "TS808 clipper is not blockwise (single NL group)");

    // But diodes ARE K-method candidates
    for comp_idx in 0..graph.components.len() {
        if graph.components[comp_idx].kind.type_tag() == "diode" {
            let candidacy = k_method_candidacy(comp_idx, &graph);
            assert!(candidacy.is_candidate,
                "Diode {} should be K-method candidate",
                graph.components[comp_idx].id);
        }
    }
}
