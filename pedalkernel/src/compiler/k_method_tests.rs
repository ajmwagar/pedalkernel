//! TDD tests for K-method table generation and runtime lookup.
//!
//! K-method replaces Newton-Raphson iteration inside WDF NL roots with
//! a precomputed lookup table. The table maps incident wave → reflected wave
//! for a specific port resistance and NL device.
//!
//! Interface: root.process(b_tree, rp) → a_root
//! K-method:  table.lookup(b_tree) → a_root  (rp baked in at build time)
//!
//! For 2D devices (BJT, triode): table is indexed by (b_tree, control_voltage).

use super::spqr_build::compile_via_spqr;
use crate::dsl::parse_pedal_file;
use crate::PedalProcessor;

const SR: f64 = 48_000.0;

/// Simple diode clipper — 1D K-method target.
const DIODE_CLIPPER: &str = r#"pedal "Diode Clipper" {
  supply 9V
  components {
    R1: resistor(10k)
    D1: diode(silicon)
    D2: diode(silicon)
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
}"#;

/// Single BJT common-emitter — 2D K-method target.
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

// ═══════════════════════════════════════════════════════════════════════════
// 1. Table generation: a compiled NL stage should have a K-table
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn diode_clipper_stage_has_k_table() {
    let pedal = parse_pedal_file(DIODE_CLIPPER).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    // Find the WDF stage with a diode root
    let has_diode_stage = compiled.stages.iter().any(|s| {
        matches!(s, super::compiled::Stage::Wdf(_))
    });
    assert!(has_diode_stage, "Should have a WDF stage");

    // The diode stage should have a K-method table
    // (currently it doesn't — this test drives the implementation)
    let has_k_table = compiled.stages.iter().any(|s| {
        if let super::compiled::Stage::Wdf(w) = s {
            w.k_table.is_some()
        } else {
            false
        }
    });
    assert!(has_k_table, "Diode WDF stage should have a K-method table");
}

#[test]
fn bjt_stage_has_k_table() {
    let pedal = parse_pedal_file(BJT_CE).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    let has_k_table = compiled.stages.iter().any(|s| {
        if let super::compiled::Stage::Wdf(w) = s {
            w.k_table.is_some()
        } else {
            false
        }
    });
    assert!(has_k_table, "BJT WDF stage should have a K-method table");
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Table properties: correct dimensions, finite values
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn diode_k_table_is_1d() {
    let pedal = parse_pedal_file(DIODE_CLIPPER).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    for s in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = s {
            if let Some(ref table) = w.k_table {
                assert_eq!(table.dims, 1, "Diode clipper should be 1D table");
                assert!(table.entries.len() >= 64,
                    "Should have ≥64 entries, got {}", table.entries.len());
                return;
            }
        }
    }
    panic!("No K-table found");
}

#[test]
fn bjt_k_table_is_2d() {
    let pedal = parse_pedal_file(BJT_CE).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    for s in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = s {
            if let Some(ref table) = w.k_table {
                assert_eq!(table.dims, 2, "BJT should be 2D table");
                return;
            }
        }
    }
    panic!("No K-table found");
}

#[test]
fn k_table_values_are_finite() {
    let pedal = parse_pedal_file(DIODE_CLIPPER).unwrap();
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
// 3. Table accuracy: lookup matches NR solve
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn diode_k_table_matches_nr_at_zero() {
    // At b_tree = 0 (no incident wave), reflected wave should be ~0
    let pedal = parse_pedal_file(DIODE_CLIPPER).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    for s in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = s {
            if let Some(ref table) = w.k_table {
                let a_root = table.lookup_1d(0.0);
                assert!(
                    a_root.abs() < 0.1,
                    "At b_tree=0, reflected wave should be ~0, got {a_root:.6}"
                );
                return;
            }
        }
    }
    panic!("No K-table found");
}

#[test]
fn diode_k_table_clips_large_input() {
    // At large positive b_tree, diode conducts → reflected wave is clamped
    let pedal = parse_pedal_file(DIODE_CLIPPER).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    for s in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = s {
            if let Some(ref table) = w.k_table {
                let a_small = table.lookup_1d(0.1);
                let a_large = table.lookup_1d(5.0);
                // Large input should produce larger reflected wave, but
                // growth should be sublinear (diode clips)
                assert!(
                    a_large.abs() > a_small.abs(),
                    "Larger input should produce larger output"
                );
                assert!(
                    a_large.abs() < 5.0,
                    "Diode should clip — reflected wave {:.3} should be < 5V",
                    a_large.abs()
                );
                return;
            }
        }
    }
    panic!("No K-table found");
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Runtime: K-table produces same audio as NR
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn diode_clipper_with_k_table_produces_audio() {
    let pedal = parse_pedal_file(DIODE_CLIPPER).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();
    let mut proc: Box<dyn PedalProcessor> = Box::new(compiled);

    let mut peak = 0.0f64;
    for i in 0..9600 {
        let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SR).sin();
        let out = proc.process(input);
        if i >= 4800 {
            peak = peak.max(out.abs());
        }
    }
    eprintln!("  Diode clipper K-table peak: {peak:.6}");
    assert!(peak > 0.01, "Should produce audible output, got {peak:.6}");
    assert!(peak < 2.0, "Should clip — peak {peak:.3} should be < 2V");
}

#[test]
fn bjt_ce_with_k_table_produces_audio() {
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
// 5. No regression: existing pedals still work with K-table enabled
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn k_table_no_nan_across_input_range() {
    let pedal = parse_pedal_file(DIODE_CLIPPER).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    for s in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = s {
            if let Some(ref table) = w.k_table {
                // Sweep the full input domain
                for i in 0..1000 {
                    let b_tree = (i as f64 - 500.0) * 0.02; // -10V to +10V
                    let a_root = table.lookup_1d(b_tree);
                    assert!(
                        a_root.is_finite(),
                        "K-table lookup at b_tree={b_tree:.3} returned {a_root}"
                    );
                }
                return;
            }
        }
    }
    panic!("No K-table found");
}
