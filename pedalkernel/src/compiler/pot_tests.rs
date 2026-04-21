// TDD tests for native 3-terminal pot handling.
//
// These test the CONTRACT: a pot is one component, creates one edge,
// produces a working voltage divider, and responds to set_control().
// Written BEFORE the implementation — they define what "correct" means.

use super::spqr_build::compile_via_spqr;
use super::spqr_control::bind_controls;
use crate::PedalProcessor;

// ═══════════════════════════════════════════════════════════════════════════
// Graph level: pot is one component, one edge
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn pot_halves_in_same_stage() {
    // Pot halves (aw + wb) should end up in the same stage, not split apart.
    // The wiper is a real circuit node — two edges is correct in the graph.
    // But the SPQR pipeline must keep them together for the divider to work.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                Volume: pot(100k, a)
            }
            nets {
                in -> R1.a
                R1.b -> Volume.a
                Volume.w -> out
                Volume.b -> gnd
            }
            controls { Volume.position -> "Volume" [0.0, 1.0] = 0.5 }
        }"#)
    .expect("parse");

    let compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    // Both pot halves should be findable in the SAME stage
    let mut aw_stage = None;
    let mut wb_stage = None;
    for (i, stage) in compiled.stages.iter().enumerate() {
        if stage.tree.get_pot_position("Volume__aw").is_some() {
            aw_stage = Some(i);
        }
        if stage.tree.get_pot_position("Volume__wb").is_some() {
            wb_stage = Some(i);
        }
    }

    assert!(aw_stage.is_some(), "Should find Volume__aw in a stage");
    assert!(wb_stage.is_some(), "Should find Volume__wb in a stage");
    assert_eq!(
        aw_stage, wb_stage,
        "Both pot halves must be in the SAME stage for voltage divider to work"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Stage level: pot creates a voltage divider
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn pot_creates_voltage_divider_stage() {
    // A standalone volume pot should compile into a stage that attenuates.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                Volume: pot(100k, a)
            }
            nets {
                in -> R1.a
                R1.b -> Volume.a
                Volume.w -> out
                Volume.b -> gnd
            }
            controls { Volume.position -> "Volume" [1.0, 0.0] = 0.5 }
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    eprintln!("Divider stages: {}", compiled.stages.len());
    for (i, s) in compiled.stages.iter().enumerate() {
        eprintln!("  stage {i}: rp={:.1}", s.tree.port_resistance());
    }

    // At default (0.5): should pass signal with some attenuation
    for _ in 0..100 {
        compiled.process(1.0);
    }
    let mid_output = compiled.process(1.0);
    assert!(
        mid_output.abs() > 0.01 && mid_output.abs() < 1.0,
        "Pot at 50% should attenuate: got {mid_output:.4}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Control level: set_control changes attenuation
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn pot_set_control_changes_level() {
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                Volume: pot(100k, a)
            }
            nets {
                in -> R1.a
                R1.b -> Volume.a
                Volume.w -> out
                Volume.b -> gnd
            }
            controls { Volume.position -> "Volume" [0.0, 1.0] = 0.5 }
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    bind_controls(&pedal, &mut compiled);

    // Full volume
    compiled.set_control("Volume", 1.0);
    for _ in 0..50 { compiled.process(1.0); }
    let full = compiled.process(1.0).abs();

    // Zero volume
    compiled.set_control("Volume", 0.0);
    for _ in 0..50 { compiled.process(1.0); }
    let zero = compiled.process(1.0).abs();

    eprintln!("Pot control: full={full:.4}, zero={zero:.4}");
    assert!(
        full > zero * 3.0,
        "Full volume should be much louder than zero: full={full:.4}, zero={zero:.4}"
    );
}

#[test]
fn pot_in_feedback_changes_gain() {
    // Drive pot in op-amp feedback: changing position changes gain.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(1k)
                U1: opamp(tl072)
                R_min: resistor(1k)
                Drive: pot(100k, a)
            }
            nets {
                in -> R1.a
                R1.b -> U1.neg
                U1.neg -> R_min.a
                R_min.b -> Drive.a
                Drive.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls { Drive.position -> "Drive" [0.0, 1.0] = 0.5 }
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    bind_controls(&pedal, &mut compiled);

    // Low gain
    compiled.set_control("Drive", 0.0);
    for _ in 0..50 { compiled.process(0.01); }
    let low_gain_out = compiled.process(0.01).abs();

    // High gain
    compiled.set_control("Drive", 1.0);
    for _ in 0..50 { compiled.process(0.01); }
    let high_gain_out = compiled.process(0.01).abs();

    eprintln!("Drive: low={low_gain_out:.4}, high={high_gain_out:.4}");
    assert!(
        high_gain_out > low_gain_out * 2.0,
        "High drive should amplify more: low={low_gain_out:.4}, high={high_gain_out:.4}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Direct WDF scatter math test
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn wdf_voltage_divider_scatter_math() {
    // Direct WDF test: Series(VS, Series(R_aw, R_wb)) + ShortCircuit
    // Expected: wiper voltage = Vin * R_wb / (R_aw + R_wb)
    use super::dyn_node::DynNode;
    use super::wdf_leaf::{WdfResistor, WdfVoltageSource};

    let r_aw = 50_000.0;
    let r_wb = 50_000.0;

    let vs = DynNode::Leaf(Box::new(WdfVoltageSource {
        voltage: 0.0,
        rp: 1.0,
        is_cathode_bias: false,
    }));
    let leaf_aw = DynNode::Leaf(Box::new(WdfResistor { rp: r_aw, comp_id: Some("aw".to_string()), last_a: 0.0 }));
    let leaf_wb = DynNode::Leaf(Box::new(WdfResistor { rp: r_wb, comp_id: Some("wb".to_string()), last_a: 0.0 }));

    // Series(R_aw, R_wb) — the divider chain
    let divider = DynNode::Series(Box::new(leaf_aw), Box::new(leaf_wb));
    // Series(VS, divider) — full tree
    let mut tree = DynNode::Series(Box::new(vs), Box::new(divider));

    let rp = tree.port_resistance();
    eprintln!("Tree port resistance: {rp:.1}");
    // Expected: VS(1) + R_aw(50k) + R_wb(50k) = 100001
    assert!((rp - 100001.0).abs() < 10.0, "rp should be ~100001, got {rp:.1}");

    // Set VS voltage and scatter
    tree.set_voltage(1.0);
    let b = tree.reflected();
    eprintln!("reflected b = {b:.6}");

    // ShortCircuit: a = -b
    let a = -b;
    tree.set_incident(a);

    // Read junction voltage at wiper (between R_aw and R_wb)
    // Method 1: series_junction_voltage
    let v_junction = tree.series_junction_voltage(a);
    eprintln!("series_junction_voltage = {v_junction:?}");

    // Method 2: leaf voltage of R_wb
    let v_wb = tree.leaf_voltage(""); // Can't easily get by ID on raw resistors
    eprintln!("leaf_voltage = {v_wb:?}");

    // Expected wiper voltage: 1.0 * 50k / 100k = 0.5V
    if let Some(v) = v_junction {
        let expected = 1.0 * r_wb / (r_aw + r_wb);
        assert!(
            (v.abs() - expected).abs() < 0.1,
            "Wiper voltage should be ~{expected:.2}V, got {v:.4}V"
        );
    } else {
        // If series_junction_voltage returns None, try manual extraction
        // Root voltage = (a + b) / 2 = 0 (ShortCircuit)
        let v_root = (a + b) / 2.0;
        eprintln!("Root voltage (should be 0): {v_root:.6}");

        // Let's try a different approach: output_probe on R_wb
        panic!("series_junction_voltage returned None — need alternative extraction");
    }
}

#[test]
fn wdf_simple_resistor_divider() {
    // Simplest possible test: Series(VS, R) with ShortCircuit.
    // Output should be 0 at root (short circuit) but VS drives current through R.
    use super::dyn_node::DynNode;
    use super::wdf_leaf::{WdfResistor, WdfVoltageSource};

    let vs = DynNode::Leaf(Box::new(WdfVoltageSource {
        voltage: 0.0,
        rp: 1.0,
        is_cathode_bias: false,
    }));
    let r = DynNode::Leaf(Box::new(WdfResistor { rp: 10_000.0, comp_id: Some("R".to_string()), last_a: 0.0 }));
    let mut tree = DynNode::Series(Box::new(vs), Box::new(r));

    tree.set_voltage(1.0);
    let b = tree.reflected();
    let a = -b; // ShortCircuit
    tree.set_incident(a);

    let v_root = (a + b) / 2.0;
    let v_junction = tree.series_junction_voltage(a);
    eprintln!("Simple: b={b:.6}, a={a:.6}, v_root={v_root:.6}, v_junction={v_junction:?}");

    // With ShortCircuit: all voltage drops across R.
    // V_R = Vin (VS drives 1V, ground absorbs it all through R)
    // series_junction_voltage should give voltage at R node
    assert!(v_junction.is_some(), "Should extract junction voltage");
}

// ═══════════════════════════════════════════════════════════════════════════
// No synthetic split leaking
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn no_synthetic_aw_wb_components() {
    // After compilation, no __aw/__wb synthetic component names should leak
    // into stage trees or control bindings.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                Volume: pot(100k, a)
            }
            nets {
                in -> R1.a
                R1.b -> Volume.a
                Volume.w -> out
                Volume.b -> gnd
            }
            controls { Volume.position -> "Volume" [0.0, 1.0] = 0.5 }
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    bind_controls(&pedal, &mut compiled);

    // Control binding should use base ID "Volume", not "Volume__aw"
    for ctrl in &compiled.controls {
        assert!(
            !ctrl.component_id.contains("__aw") && !ctrl.component_id.contains("__wb"),
            "Binding should use base ID, got '{}'", ctrl.component_id
        );
    }
}
