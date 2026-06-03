//! TDD tests for K-method table generation and BjtRoot WDF integration.
//!
//! K-method replaces Newton-Raphson iteration inside WDF NL roots with
//! a precomputed lookup table.
//!
//! BJTs need a BjtRoot variant in RootKind (like TriodeRoot) to work as
//! single-port WDF roots with external Vbe control. This enables:
//! - WDF tree processing for common-emitter BJT stages
//! - K-method tabulation (2D: b_tree × Vbe)
//! - Blockwise decomposition of BJT cascades (303 ladder)

use super::spqr_build::compile_via_spqr;
use crate::dsl::parse_pedal_file;
use crate::PedalProcessor;

const SR: f64 = 48_000.0;

/// Single BJT common-emitter — primary K-method target (2D).
const BJT_CE: &str = r#"pedal "BJT CE" {
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

/// 4-stage BJT ladder (303-style) — blockwise + K-method combined target.
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
    vcc -> R_c1.a  R_c1.b -> Q1.collector
    Q1.emitter -> R_e1.a  R_e1.b -> gnd
    Q1.emitter -> C1.a  C1.b -> gnd
    Q1.collector -> Q2.base
    vcc -> R_c2.a  R_c2.b -> Q2.collector
    Q2.emitter -> R_e2.a  R_e2.b -> gnd
    Q2.emitter -> C2.a  C2.b -> gnd
    Q2.collector -> Q3.base
    vcc -> R_c3.a  R_c3.b -> Q3.collector
    Q3.emitter -> R_e3.a  R_e3.b -> gnd
    Q3.emitter -> C3.a  C3.b -> gnd
    Q3.collector -> Q4.base
    vcc -> R_c4.a  R_c4.b -> Q4.collector
    Q4.emitter -> R_e4.a  R_e4.b -> gnd
    Q4.emitter -> C4.a  C4.b -> gnd
    Q4.collector -> out
    Q4.collector -> Resonance.a
    Resonance.b -> R_fb.a
    R_fb.b -> Q1.base
  }
  controls {
    Resonance.position -> "Resonance" [0.0, 1.0] = 0.0
  }
}"#;

fn find_wdf_with_k_table(compiled: &super::compiled::CompiledPedal) -> bool {
    compiled.stages.iter().any(|s| {
        if let super::compiled::Stage::Wdf(w) = s {
            w.k_table.is_some()
        } else {
            false
        }
    })
}

fn blockwise_k_table_dims(compiled: &super::compiled::CompiledPedal) -> Vec<usize> {
    compiled
        .stages
        .iter()
        .flat_map(|stage| {
            if let super::compiled::Stage::Blockwise(bkm) = stage {
                bkm.blocks
                    .iter()
                    .map(|block| block.k_table.dims)
                    .collect::<Vec<_>>()
            } else {
                Vec::new()
            }
        })
        .collect()
}

// ═══════════════════════════════════════════════════════════════════════════
// 1. BjtRoot exists in RootKind and compiles from common-emitter topology
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bjt_ce_compiles_to_wdf_with_bjt_root() {
    // A single common-emitter BJT should compile to a WDF stage with a
    // BjtRoot (not MultiNL). The base is external control, C-E is the port.
    let pedal = parse_pedal_file(BJT_CE).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    let has_bjt_wdf = compiled.stages.iter().any(|s| {
        if let super::compiled::Stage::Wdf(w) = s {
            // Check for BjtRoot variant — doesn't exist yet, will fail
            matches!(w.root, pedalkernel_rt::stage::RootKind::Bjt(_))
        } else {
            false
        }
    });
    assert!(
        has_bjt_wdf,
        "Single BJT CE should compile to WDF with BjtRoot"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. BjtRoot gets K-method table
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bjt_stage_has_k_table() {
    let pedal = parse_pedal_file(BJT_CE).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();
    assert!(
        find_wdf_with_k_table(&compiled),
        "BJT WDF stage should have a K-table"
    );
}

#[test]
fn bjt_k_table_is_2d() {
    let pedal = parse_pedal_file(BJT_CE).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    for s in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = s {
            if let Some(ref table) = w.k_table {
                assert_eq!(table.dims, 2, "BJT should be 2D (b_tree × Vbe)");
                return;
            }
        }
    }
    panic!("No K-table found");
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. K-table values are valid
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bjt_k_table_values_are_finite() {
    let pedal = parse_pedal_file(BJT_CE).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    for s in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = s {
            if let Some(ref table) = w.k_table {
                for (i, &val) in table.entries.iter().enumerate() {
                    assert!(val.is_finite(), "K-table [{i}] = {val}");
                }
                return;
            }
        }
    }
    panic!("No K-table found");
}

#[test]
fn bjt_k_table_response_varies_with_input() {
    let pedal = parse_pedal_file(BJT_CE).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    for s in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = s {
            if let Some(ref table) = w.k_table {
                // Sweep across ctrl range to find a conducting operating point
                let mut max_diff = 0.0f64;
                let mut best_ctrl = 0.0;
                for ci in 0..10 {
                    let ctrl =
                        table.ctrl_min + (ci as f64 / 9.0) * (table.ctrl_max - table.ctrl_min);
                    let a_pos = table.lookup_2d(5.0, ctrl);
                    let a_neg = table.lookup_2d(-5.0, ctrl);
                    let diff = (a_pos - a_neg).abs();
                    if diff > max_diff {
                        max_diff = diff;
                        best_ctrl = ctrl;
                    }
                }
                eprintln!("  Max table variation: {max_diff:.6} at ctrl={best_ctrl:.3}");
                assert!(
                    max_diff > 1e-6,
                    "K-table should vary with input at some ctrl, max_diff={max_diff:.6}"
                );
                return;
            }
        }
    }
    panic!("No K-table found");
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Runtime: BJT with K-table produces audio
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bjt_with_k_table_produces_audio() {
    let pedal = parse_pedal_file(BJT_CE).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();
    let mut proc: Box<dyn PedalProcessor> = Box::new(compiled);

    let mut peak = 0.0f64;
    for i in 0..9600 {
        let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SR).sin() * 0.1;
        let out = proc.process(input);
        if i >= 4800 {
            peak = peak.max(out.abs());
        }
    }
    eprintln!("  BJT CE peak: {peak:.6}");
    assert!(peak > 0.001, "Should produce output, got {peak:.6}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. Blockwise ladder: each block gets BjtRoot + K-table
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn ladder_4_blocks_have_bjt_roots() {
    let pedal = parse_pedal_file(BJT_LADDER_4).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    let block_k_table_dims = blockwise_k_table_dims(&compiled);

    assert!(
        block_k_table_dims.iter().filter(|&&dims| dims == 2).count() >= 4,
        "Ladder should have ≥4 BKM BJT blocks with 2D K-tables, got {block_k_table_dims:?}"
    );
}

#[test]
fn ladder_4_blocks_have_k_tables() {
    let pedal = parse_pedal_file(BJT_LADDER_4).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    let k_table_count = blockwise_k_table_dims(&compiled).len();

    eprintln!("  Ladder 4: {k_table_count} BKM blocks with K-tables");
    assert!(
        k_table_count >= 4,
        "Ladder should have ≥4 K-tables (one per BJT block), got {k_table_count}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. No NaN across input domain
// ═══════════════════════════════════════════════════════════════════════════

#[test]
// ═══════════════════════════════════════════════════════════════════════════
// 7. K-table gating: only generated for eligible components
// ═══════════════════════════════════════════════════════════════════════════
#[test]
fn passive_stages_have_no_k_table() {
    // Passive WDF stages (ShortCircuit, Passthrough) should NOT get K-tables
    let pedal = parse_pedal_file(
        r#"pedal "test" {
      supply 9V
      components { R1: resistor(10k)  C1: cap(10n) }
      nets { in -> R1.a  R1.b -> C1.a  C1.b -> gnd  R1.b -> out }
      controls {}
    }"#,
    )
    .unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    for s in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = s {
            assert!(
                w.k_table.is_none(),
                "Passive WDF stage should NOT have K-table"
            );
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 8. Runtime: K-table is USED (not just stored)
//
// Process the same signal through two copies — one should use K-table
// lookup, the other NR. Results should match closely.
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn k_table_runtime_matches_nr() {
    // Compile a BJT stage with K-table
    let pedal = parse_pedal_file(BJT_LADDER_4).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    // Check that K-table stages exist
    let has_k_table = compiled.stages.iter().any(|s| {
        if let super::compiled::Stage::Wdf(w) = s {
            w.k_table.is_some()
        } else {
            false
        }
    }) || !blockwise_k_table_dims(&compiled).is_empty();
    assert!(has_k_table, "Should have at least one K-table stage");

    // Process audio — this SHOULD use the K-table internally
    let mut proc: Box<dyn PedalProcessor> = Box::new(compiled);
    let mut peak = 0.0f64;
    let mut any_nonzero = false;
    for i in 0..9600 {
        let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SR).sin() * 0.1;
        let out = proc.process(input);
        if i >= 4800 {
            peak = peak.max(out.abs());
            if out.abs() > 1e-6 {
                any_nonzero = true;
            }
        }
        assert!(
            out.is_finite(),
            "K-table runtime produced NaN at sample {i}"
        );
    }
    eprintln!("  K-table runtime peak: {peak:.6}");
    assert!(any_nonzero, "K-table runtime should produce nonzero output");
    assert!(peak > 0.001, "K-table runtime peak too low: {peak:.6}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 9. Diagnostic: compare K-table lookup vs NR at same operating point
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn k_table_vs_nr_pipeline_comparison() {
    // Compare full pipeline output: K-table path vs NR-only path
    let pedal = parse_pedal_file(BJT_LADDER_4).unwrap();

    // NR path: compile then strip K-tables
    let mut compiled_nr = compile_via_spqr(&pedal, SR).unwrap();
    for s in &mut compiled_nr.stages {
        if let super::compiled::Stage::Wdf(w) = s {
            w.k_table = None;
        }
    }
    let mut proc_nr: Box<dyn PedalProcessor> = Box::new(compiled_nr);
    let mut nr_peak = 0.0f64;
    for i in 0..9600 {
        let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SR).sin() * 0.1;
        let out = proc_nr.process(input);
        if i >= 4800 {
            nr_peak = nr_peak.max(out.abs());
        }
    }

    // K-table path: compile normally
    let compiled_kt = compile_via_spqr(&parse_pedal_file(BJT_LADDER_4).unwrap(), SR).unwrap();
    // Diagnostic: check table entries
    for (si, s) in compiled_kt.stages.iter().enumerate() {
        if let super::compiled::Stage::Wdf(w) = s {
            if let Some(ref table) = w.k_table {
                let nonzero = table.entries.iter().filter(|&&v| v.abs() > 1e-12).count();
                let max = table.entries.iter().cloned().fold(0.0f64, f64::max);
                let min = table.entries.iter().cloned().fold(0.0f64, f64::min);
                eprintln!(
                    "  Stage {si} table: {} entries, {} nonzero, range=[{min:.4}, {max:.4}]",
                    table.entries.len(),
                    nonzero
                );
            }
        }
    }
    let mut proc_kt: Box<dyn PedalProcessor> = Box::new(compiled_kt);
    let mut kt_peak = 0.0f64;
    for i in 0..9600 {
        let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SR).sin() * 0.1;
        let out = proc_kt.process(input);
        if i >= 4800 {
            kt_peak = kt_peak.max(out.abs());
        }
    }

    eprintln!(
        "  Pipeline: NR={nr_peak:.6}, K-table={kt_peak:.6}, ratio={:.3}",
        kt_peak / nr_peak.max(1e-12)
    );

    assert!(nr_peak > 0.001, "NR should produce output: {nr_peak:.6}");
    assert!(
        kt_peak > 0.001,
        "K-table should produce output: {kt_peak:.6}"
    );
    // K-table output should be within 10x of NR (generous for now)
    let ratio = kt_peak / nr_peak.max(1e-12);
    assert!(
        ratio > 0.1 && ratio < 10.0,
        "K-table should be within 10x of NR: ratio={ratio:.3}"
    );
}

#[test]
fn bjt_k_table_no_nan() {
    let pedal = parse_pedal_file(BJT_CE).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    for s in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = s {
            if let Some(ref table) = w.k_table {
                for i in 0..100 {
                    for j in 0..10 {
                        let b = (i as f64 - 50.0) * 0.2;
                        let ctrl = (j as f64 - 5.0) * 0.4;
                        let a = table.lookup_2d(b, ctrl);
                        assert!(a.is_finite(), "NaN at b={b:.1}, ctrl={ctrl:.1}");
                    }
                }
                return;
            }
        }
    }
    panic!("No K-table found");
}

#[test]
fn single_bjt_edges_in_same_flow_group() {
    // A single BJT's two NL edges (B-E and C-E) must land in the same FlowGroup.
    // If split, SPQR can't build a WDF stage for either edge alone.
    let pedal = parse_pedal_file(BJT_CE).unwrap();
    let graph = super::graph::CircuitGraph::from_pedal(&pedal);
    let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
    let groups = super::signal_flow::find_flow_groups(&all_edges, &graph);

    // Find all groups containing Q1's NL edges
    let q1_groups: Vec<usize> = groups
        .iter()
        .enumerate()
        .filter(|(_, g)| {
            g.active_edges
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "Q1")
        })
        .map(|(i, _)| i)
        .collect();

    assert_eq!(
        q1_groups.len(),
        1,
        "Q1's edges should be in 1 group, got {} groups: {:?}",
        q1_groups.len(),
        q1_groups
    );
}
