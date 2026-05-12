//! K-method tests for non-BJT devices: diode clippers, triode stages.
//!
//! Verifies K-tables work correctly for 1D (diode) and 2D (triode) roots
//! that DON'T go through the blockwise path. These compile as standard
//! WDF stages via the normal SPQR pipeline.

use super::spqr_build::compile_via_spqr;
use crate::dsl::parse_pedal_file;
use crate::PedalProcessor;

const SR: f64 = 48_000.0;

/// Single triode preamp stage — 2D K-method target.
/// Grid is driven by input, plate-cathode is the WDF port.
const TRIODE_PREAMP: &str = r#"pedal "Triode Preamp" {
  supply 9V
  components {
    V1: triode(12ax7)
    R_g: resistor(1M)
    R_k: resistor(1.5k)
    C_k: cap(25u)
    R_p: resistor(100k)
    C_in: cap(22n)
    R_out: resistor(10k)
  }
  nets {
    in -> C_in.a
    C_in.b -> R_g.a
    R_g.b -> gnd
    C_in.b -> V1.grid
    V1.cathode -> R_k.a
    R_k.b -> gnd
    V1.cathode -> C_k.a
    C_k.b -> gnd
    vcc -> R_p.a
    R_p.b -> V1.plate
    V1.plate -> R_out.a
    R_out.b -> out
  }
  controls {}
}"#;

// ═══════════════════════════════════════════════════════════════════════════
// 1. Triode stage gets K-table
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn triode_stage_has_k_table() {
    let pedal = parse_pedal_file(TRIODE_PREAMP).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    let mut found = false;
    for (i, s) in compiled.stages.iter().enumerate() {
        if let super::compiled::Stage::Wdf(w) = s {
            let has_table = w.k_table.is_some();
            let root_name = match &w.root {
                pedalkernel_rt::stage::RootKind::Triode(_) => "Triode",
                pedalkernel_rt::stage::RootKind::Bjt(_) => "Bjt",
                pedalkernel_rt::stage::RootKind::ShortCircuit => "ShortCircuit",
                pedalkernel_rt::stage::RootKind::Passthrough => "Passthrough",
                _ => "other",
            };
            eprintln!("  [{i}] WDF root={root_name} k_table={has_table}");
            if has_table {
                let table = w.k_table.as_ref().unwrap();
                eprintln!("    dims={}, steps={}, entries={}",
                    table.dims, table.steps, table.entries.len());
                found = true;
            }
        }
    }
    assert!(found, "Triode stage should have a K-table");
}

#[test]
fn triode_k_table_is_2d() {
    let pedal = parse_pedal_file(TRIODE_PREAMP).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    for s in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = s {
            if let Some(ref table) = w.k_table {
                assert_eq!(table.dims, 2, "Triode should be 2D");
                return;
            }
        }
    }
    panic!("No K-table found");
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Triode K-table NR vs table comparison
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn triode_k_table_vs_nr_pipeline() {
    let pedal = parse_pedal_file(TRIODE_PREAMP).unwrap();

    // NR path
    let mut compiled_nr = compile_via_spqr(&pedal, SR).unwrap();
    for s in &mut compiled_nr.stages {
        if let super::compiled::Stage::Wdf(w) = s { w.k_table = None; }
    }
    let mut proc_nr: Box<dyn PedalProcessor> = Box::new(compiled_nr);
    let mut nr_peak = 0.0f64;
    for i in 0..9600 {
        let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SR).sin() * 0.1;
        let out = proc_nr.process(input);
        if i >= 4800 { nr_peak = nr_peak.max(out.abs()); }
    }

    // K-table path
    let compiled_kt = compile_via_spqr(&parse_pedal_file(TRIODE_PREAMP).unwrap(), SR).unwrap();
    let mut proc_kt: Box<dyn PedalProcessor> = Box::new(compiled_kt);
    let mut kt_peak = 0.0f64;
    for i in 0..9600 {
        let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SR).sin() * 0.1;
        let out = proc_kt.process(input);
        if i >= 4800 { kt_peak = kt_peak.max(out.abs()); }
    }

    let ratio = kt_peak / nr_peak.max(1e-12);
    eprintln!("  Triode pipeline: NR={nr_peak:.6}, KT={kt_peak:.6}, ratio={ratio:.3}");

    assert!(nr_peak > 0.001, "Triode NR should produce output: {nr_peak:.6}");
    assert!(kt_peak > 0.001, "Triode KT should produce output: {kt_peak:.6}");
    assert!(ratio > 0.5 && ratio < 2.0,
        "Triode K-table should be within 2x of NR: ratio={ratio:.3}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Triode K-table values are sane
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn triode_k_table_values_finite() {
    let pedal = parse_pedal_file(TRIODE_PREAMP).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    for s in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = s {
            if let Some(ref table) = w.k_table {
                for (i, &v) in table.entries.iter().enumerate() {
                    assert!(v.is_finite(), "Triode K-table[{i}] = {v}");
                }
                return;
            }
        }
    }
    panic!("No K-table found");
}

#[test]
fn triode_k_table_no_nan_across_range() {
    let pedal = parse_pedal_file(TRIODE_PREAMP).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    for s in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = s {
            if let Some(ref table) = w.k_table {
                for i in 0..50 {
                    for j in 0..20 {
                        let b = (i as f64 - 25.0) * 0.4;
                        let ctrl = (j as f64 - 10.0) * 0.2;
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

// ═══════════════════════════════════════════════════════════════════════════
// 4. 1D diode test — TS808-style opamp+diode feedback
// ═══════════════════════════════════════════════════════════════════════════

// Note: diode-to-ground clippers use Wright Omega (already O(1)) and
// go through the ground-clip path. They don't become WDF roots with K-tables.
// Diodes in opamp feedback loops go through the feedback_opamp path.
// So diode K-tables may not fire for standard pedal circuits.
// This test documents whether any diode WDF root exists.

#[test]
fn check_if_any_diode_root_gets_k_table() {
    // Simple diode shunt — may or may not become a WDF DiodePair root
    let source = r#"pedal "Diode Shunt" {
      supply 9V
      components {
        R1: resistor(10k)
        D1: diode(silicon)
        D2: diode(silicon)
        R2: resistor(10k)
      }
      nets {
        in -> R1.a
        R1.b -> D1.anode, D2.cathode, R2.a
        D1.cathode -> gnd
        D2.anode -> gnd
        R2.b -> out
      }
      controls {}
    }"#;

    let pedal = parse_pedal_file(source).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    let mut any_diode_table = false;
    for (i, s) in compiled.stages.iter().enumerate() {
        if let super::compiled::Stage::Wdf(w) = s {
            let root_name = match &w.root {
                pedalkernel_rt::stage::RootKind::DiodePair(_) => "DiodePair",
                pedalkernel_rt::stage::RootKind::ExplicitDiodePair(_) => "ExplicitDiodePair",
                pedalkernel_rt::stage::RootKind::SingleDiode(_) => "SingleDiode",
                pedalkernel_rt::stage::RootKind::ExplicitSingleDiode(_) => "ExplicitSingleDiode",
                pedalkernel_rt::stage::RootKind::ShortCircuit => "ShortCircuit",
                pedalkernel_rt::stage::RootKind::Passthrough => "Passthrough",
                _ => "other",
            };
            let has_table = w.k_table.is_some();
            eprintln!("  [{i}] root={root_name} k_table={has_table}");
            if has_table { any_diode_table = true; }
        }
    }
    eprintln!("  Any diode K-table: {any_diode_table}");
    // Don't assert — just document. Diodes may go through ground-clip path.
}
