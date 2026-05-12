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
// 4. Cascaded triode — goes through blockwise → gets K-tables
// ═══════════════════════════════════════════════════════════════════════════

/// Two cascaded triode preamp stages. Blockwise should split into 2 blocks,
/// each getting a Triode WDF root + 2D K-table.
const TRIODE_CASCADE_2: &str = r#"pedal "Triode Cascade" {
  supply 9V
  components {
    V1: triode(12ax7)
    V2: triode(12ax7)
    R_g1: resistor(1M)
    R_g2: resistor(1M)
    R_k1: resistor(1.5k)
    R_k2: resistor(1.5k)
    C_k1: cap(25u)
    C_k2: cap(25u)
    R_p1: resistor(100k)
    R_p2: resistor(100k)
    C_coup: cap(22n)
    R_in: resistor(10k)
    R_fb: resistor(1M)
  }
  nets {
    in -> R_in.a
    R_in.b -> R_g1.a
    R_g1.b -> gnd
    R_in.b -> V1.grid
    V1.cathode -> R_k1.a
    R_k1.b -> gnd
    V1.cathode -> C_k1.a
    C_k1.b -> gnd
    vcc -> R_p1.a
    R_p1.b -> V1.plate

    V1.plate -> C_coup.a
    C_coup.b -> R_g2.a
    R_g2.b -> gnd
    C_coup.b -> V2.grid
    V2.cathode -> R_k2.a
    R_k2.b -> gnd
    V2.cathode -> C_k2.a
    C_k2.b -> gnd
    vcc -> R_p2.a
    R_p2.b -> V2.plate
    V2.plate -> out

    V2.plate -> R_fb.a
    R_fb.b -> R_in.b
  }
  controls {}
}"#;

#[test]
fn triode_cascade_blockwise_has_k_tables() {
    let pedal = parse_pedal_file(TRIODE_CASCADE_2).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    let mut triode_k_count = 0;
    for (i, s) in compiled.stages.iter().enumerate() {
        if let super::compiled::Stage::Wdf(w) = s {
            let root_name = match &w.root {
                pedalkernel_rt::stage::RootKind::Triode(_) => "Triode",
                pedalkernel_rt::stage::RootKind::ShortCircuit => "ShortCircuit",
                pedalkernel_rt::stage::RootKind::Passthrough => "Passthrough",
                _ => "other",
            };
            let has_table = w.k_table.is_some();
            eprintln!("  [{i}] root={root_name} k_table={has_table}");
            if has_table && root_name == "Triode" {
                let t = w.k_table.as_ref().unwrap();
                eprintln!("    dims={} steps={} entries={}", t.dims, t.steps, t.entries.len());
                triode_k_count += 1;
            }
        }
    }
    assert!(triode_k_count >= 2,
        "Cascaded triode should have ≥2 Triode stages with K-tables, got {triode_k_count}");
}

#[test]
fn triode_cascade_k_table_vs_nr() {
    let pedal = parse_pedal_file(TRIODE_CASCADE_2).unwrap();

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
    let compiled_kt = compile_via_spqr(&parse_pedal_file(TRIODE_CASCADE_2).unwrap(), SR).unwrap();
    let mut proc_kt: Box<dyn PedalProcessor> = Box::new(compiled_kt);
    let mut kt_peak = 0.0f64;
    for i in 0..9600 {
        let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SR).sin() * 0.1;
        let out = proc_kt.process(input);
        if i >= 4800 { kt_peak = kt_peak.max(out.abs()); }
    }

    let ratio = kt_peak / nr_peak.max(1e-12);
    eprintln!("  Triode cascade: NR={nr_peak:.6}, KT={kt_peak:.6}, ratio={ratio:.3}");

    assert!(nr_peak > 0.001, "NR should produce output: {nr_peak:.6}");
    assert!(kt_peak > 0.001, "KT should produce output: {kt_peak:.6}");
    assert!(ratio > 0.3 && ratio < 3.0,
        "Triode K-table should be within 3x of NR: ratio={ratio:.3}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. Cascaded JFET — blockwise → K-tables
// ═══════════════════════════════════════════════════════════════════════════

/// Two cascaded JFET common-source stages.
const JFET_CASCADE_2: &str = r#"pedal "JFET Cascade" {
  supply 9V
  components {
    J1: njfet(2n5457)
    J2: njfet(2n5457)
    R_d1: resistor(10k)
    R_d2: resistor(10k)
    R_s1: resistor(1k)
    R_s2: resistor(1k)
    C_s1: cap(10u)
    C_s2: cap(10u)
    R_g1: resistor(1M)
    R_g2: resistor(1M)
    C_coup: cap(100n)
    R_in: resistor(10k)
    R_fb: resistor(1M)
  }
  nets {
    in -> R_in.a
    R_in.b -> R_g1.a
    R_g1.b -> gnd
    R_in.b -> J1.gate
    vcc -> R_d1.a
    R_d1.b -> J1.drain
    J1.source -> R_s1.a
    R_s1.b -> gnd
    J1.source -> C_s1.a
    C_s1.b -> gnd

    J1.drain -> C_coup.a
    C_coup.b -> R_g2.a
    R_g2.b -> gnd
    C_coup.b -> J2.gate
    vcc -> R_d2.a
    R_d2.b -> J2.drain
    J2.source -> R_s2.a
    R_s2.b -> gnd
    J2.source -> C_s2.a
    C_s2.b -> gnd

    J2.drain -> out

    J2.drain -> R_fb.a
    R_fb.b -> R_in.b
  }
  controls {}
}"#;

#[test]
fn jfet_cascade_blockwise_has_k_tables() {
    let pedal = parse_pedal_file(JFET_CASCADE_2).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();

    let mut jfet_k_count = 0;
    for (i, s) in compiled.stages.iter().enumerate() {
        if let super::compiled::Stage::Wdf(w) = s {
            let root_name = match &w.root {
                pedalkernel_rt::stage::RootKind::Jfet(_) => "Jfet",
                pedalkernel_rt::stage::RootKind::ShortCircuit => "ShortCircuit",
                pedalkernel_rt::stage::RootKind::Passthrough => "Passthrough",
                _ => "other",
            };
            let has_table = w.k_table.is_some();
            eprintln!("  [{i}] root={root_name} k_table={has_table}");
            if has_table && root_name == "Jfet" {
                let t = w.k_table.as_ref().unwrap();
                eprintln!("    dims={} steps={} entries={}", t.dims, t.steps, t.entries.len());
                jfet_k_count += 1;
            }
        }
    }
    assert!(jfet_k_count >= 2,
        "Cascaded JFET should have ≥2 Jfet stages with K-tables, got {jfet_k_count}");
}

#[test]
fn jfet_cascade_k_table_vs_nr() {
    let pedal = parse_pedal_file(JFET_CASCADE_2).unwrap();

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
    let compiled_kt = compile_via_spqr(&parse_pedal_file(JFET_CASCADE_2).unwrap(), SR).unwrap();
    let mut proc_kt: Box<dyn PedalProcessor> = Box::new(compiled_kt);
    let mut kt_peak = 0.0f64;
    for i in 0..9600 {
        let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SR).sin() * 0.1;
        let out = proc_kt.process(input);
        if i >= 4800 { kt_peak = kt_peak.max(out.abs()); }
    }

    let ratio = kt_peak / nr_peak.max(1e-12);
    eprintln!("  JFET cascade: NR={nr_peak:.6}, KT={kt_peak:.6}, ratio={ratio:.3}");

    assert!(nr_peak > 0.001, "NR should produce output: {nr_peak:.6}");
    assert!(kt_peak > 0.001, "KT should produce output: {kt_peak:.6}");
    assert!(ratio > 0.3 && ratio < 3.0,
        "JFET K-table should be within 3x of NR: ratio={ratio:.3}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. 1D diode test — TS808-style opamp+diode feedback
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
