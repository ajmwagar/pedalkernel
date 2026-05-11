//! TDD tests for K-method table generation and runtime lookup.
//!
//! K-method replaces Newton-Raphson iteration inside WDF NL roots with
//! a precomputed lookup table. Targets: triode, JFET, BJT roots in WDF stages.
//!
//! Diode-to-ground clippers use Wright Omega (already O(1)) and go through
//! a separate ground-clip path — they don't need K-tables.

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
// 1. Table generation
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bjt_stage_has_k_table() {
    let pedal = parse_pedal_file(BJT_CE).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    eprintln!("  Stages: {}", compiled.stages.len());
    for (i, s) in compiled.stages.iter().enumerate() {
        match s {
            super::compiled::Stage::Wdf(w) => {
                let has = w.k_table.is_some();
                eprintln!("  [{i}] WDF k_table={has}");
            }
            _ => eprintln!("  [{i}] other"),
        }
    }

    assert!(find_wdf_with_k_table(&compiled), "BJT WDF stage should have a K-method table");
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Table properties
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bjt_k_table_is_2d() {
    let pedal = parse_pedal_file(BJT_CE).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    for s in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = s {
            if let Some(ref table) = w.k_table {
                assert_eq!(table.dims, 2, "BJT should be 2D table");
                assert!(table.entries.len() >= 64,
                    "Should have ≥64 entries, got {}", table.entries.len());
                return;
            }
        }
    }
    panic!("No K-table found");
}

#[test]
fn k_table_values_are_finite() {
    let pedal = parse_pedal_file(BJT_CE).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    for s in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = s {
            if let Some(ref table) = w.k_table {
                for (i, &val) in table.entries.iter().enumerate() {
                    assert!(val.is_finite(), "K-table entry [{i}] = {val} is not finite");
                }
                return;
            }
        }
    }
    panic!("No K-table found");
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Table accuracy
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bjt_k_table_response_varies_with_input() {
    let pedal = parse_pedal_file(BJT_CE).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    for s in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = s {
            if let Some(ref table) = w.k_table {
                let a_zero = table.lookup_2d(0.0, 0.0);
                let a_pos = table.lookup_2d(5.0, 0.0);
                let a_neg = table.lookup_2d(-5.0, 0.0);
                eprintln!("  K-table: a(0)={a_zero:.4}, a(+5)={a_pos:.4}, a(-5)={a_neg:.4}");
                // Output should vary with input
                assert!(
                    (a_pos - a_neg).abs() > 0.01,
                    "K-table should produce different outputs for +5V vs -5V"
                );
                return;
            }
        }
    }
    panic!("No K-table found");
}

#[test]
fn k_table_no_nan_across_input_range() {
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
                        assert!(a.is_finite(), "K-table NaN at b={b:.1}, ctrl={ctrl:.1}");
                    }
                }
                return;
            }
        }
    }
    panic!("No K-table found");
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Runtime: K-table produces audio
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
    eprintln!("  BJT CE K-table peak: {peak:.6}");
    assert!(peak > 0.001, "Should produce output, got {peak:.6}");
    assert!(peak.is_finite(), "Output should be finite");
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. Serialization: table survives postcard roundtrip
// ═══════════════════════════════════════════════════════════════════════════

#[test]
#[cfg(feature = "serde")]
fn k_table_survives_serialization() {
    let pedal = parse_pedal_file(BJT_CE).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    // Only test if K-table was generated
    if !find_wdf_with_k_table(&compiled) {
        eprintln!("  SKIP: no K-table generated (not yet implemented for this root)");
        return;
    }

    let blob = postcard::to_allocvec(&compiled).unwrap();
    let roundtrip: super::compiled::CompiledPedal = postcard::from_bytes(&blob).unwrap();

    assert!(find_wdf_with_k_table(&roundtrip), "K-table should survive serialization");
}
