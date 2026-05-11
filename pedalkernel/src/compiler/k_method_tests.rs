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
    assert!(has_bjt_wdf, "Single BJT CE should compile to WDF with BjtRoot");
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. BjtRoot gets K-method table
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bjt_stage_has_k_table() {
    let pedal = parse_pedal_file(BJT_CE).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();
    assert!(find_wdf_with_k_table(&compiled), "BJT WDF stage should have a K-table");
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
                let a_pos = table.lookup_2d(5.0, -0.6);
                let a_neg = table.lookup_2d(-5.0, -0.6);
                assert!(
                    (a_pos - a_neg).abs() > 0.01,
                    "K-table should vary: +5V→{a_pos:.4}, -5V→{a_neg:.4}"
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
        if i >= 4800 { peak = peak.max(out.abs()); }
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

    let bjt_wdf_count = compiled.stages.iter().filter(|s| {
        if let super::compiled::Stage::Wdf(w) = s {
            matches!(w.root, pedalkernel_rt::stage::RootKind::Bjt(_))
        } else {
            false
        }
    }).count();

    assert!(bjt_wdf_count >= 4,
        "Ladder should have ≥4 WDF stages with BjtRoot, got {bjt_wdf_count}");
}

#[test]
fn ladder_4_blocks_have_k_tables() {
    let pedal = parse_pedal_file(BJT_LADDER_4).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    let k_table_count = compiled.stages.iter().filter(|s| {
        if let super::compiled::Stage::Wdf(w) = s {
            w.k_table.is_some()
        } else {
            false
        }
    }).count();

    eprintln!("  Ladder 4: {k_table_count} stages with K-tables");
    assert!(k_table_count >= 4,
        "Ladder should have ≥4 K-tables (one per BJT block), got {k_table_count}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. No NaN across input domain
// ═══════════════════════════════════════════════════════════════════════════

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
