// Tests for SPQR control binding (spqr_control.rs).
//
// Verifies: parse .pedal with controls → compile → bind → set_control changes output.

use super::spqr_build::compile_via_spqr;
use super::spqr_control::bind_controls;
use crate::PedalProcessor;

#[test]
fn bind_pot_produces_binding() {
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(1k)
                U1: opamp(tl072)
                Drive: pot(100k, a)
            }
            nets {
                in -> R1.a
                R1.b -> U1.neg
                U1.neg -> Drive.a
                Drive.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {
                Drive.position -> "Gain" [0.0, 1.0] = 0.5
            }
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    bind_controls(&pedal, &mut compiled);

    assert!(
        !compiled.controls.is_empty(),
        "Should produce at least one control binding"
    );
    assert_eq!(compiled.controls[0].label, "Gain");
}

#[test]
fn bind_volume_pot_changes_output() {
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(4.7k)
                D1: diode(silicon)
                Volume: pot(100k, a)
            }
            nets {
                in -> R1.a
                R1.b -> D1.a
                D1.b -> Volume.a
                Volume.w -> out
                Volume.b -> gnd
            }
            controls {
                Volume.position -> "Volume" [1.0, 0.0] = 0.5
            }
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    bind_controls(&pedal, &mut compiled);

    eprintln!("Controls bound: {}", compiled.controls.len());
    for c in &compiled.controls {
        eprintln!("  {:?} -> {:?}", c.label, c.target);
    }

    // Full volume
    compiled.set_control("Volume", 1.0);
    for _ in 0..50 {
        compiled.process(0.5);
    }
    let full_output = compiled.process(0.5).abs();

    // Zero volume
    compiled.set_control("Volume", 0.0);
    for _ in 0..50 {
        compiled.process(0.5);
    }
    let zero_output = compiled.process(0.5).abs();

    eprintln!("Volume: full={full_output:.4}, zero={zero_output:.4}");
    assert!(
        full_output > zero_output * 2.0 || zero_output < 0.01,
        "Volume should affect output: full={full_output:.4}, zero={zero_output:.4}"
    );
}

#[test]
fn bind_empty_controls_produces_nothing() {
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k) }
            nets { in -> R1.a  R1.b -> out }
            controls {}
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    bind_controls(&pedal, &mut compiled);

    assert!(compiled.controls.is_empty());
}

#[test]
fn debug_volume_stages() {
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(4.7k)
                D1: diode(silicon)
                Volume: pot(100k, a)
            }
            nets {
                in -> R1.a
                R1.b -> D1.a
                D1.b -> Volume.a
                Volume.w -> out
                Volume.b -> gnd
            }
            controls { Volume.position -> "Volume" [1.0, 0.0] = 0.5 }
        }"#).expect("parse");

    let compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    eprintln!("WDF Stages: {}", compiled.stages.len());
    for (i, s) in compiled.stages.iter().enumerate() {
        eprintln!("  wdf {i}: rp={:.1}", s.tree.port_resistance());
    }
    eprintln!("IIR Stages: {}", compiled.iir_stages.len());
    eprintln!("Stage order: {:?}", compiled.stage_order);
}
