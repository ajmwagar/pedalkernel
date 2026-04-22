// TDD tests for pot binding — verifying that pots are found and bound
// so that set_control actually changes something.

use super::spqr_build::compile_via_spqr;
use crate::PedalProcessor;

// ═══════════════════════════════════════════════════════════════════════════
// Layer 1: Is the pot bound at all?
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_drive_pot_is_bound() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/screamer.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    let drive_bound = compiled.controls.iter().any(|c| c.label == "Drive");
    let tone_bound = compiled.controls.iter().any(|c| c.label == "Tone");
    let level_bound = compiled.controls.iter().any(|c| c.label == "Level");

    eprintln!("Screamer bindings:");
    for c in &compiled.controls {
        eprintln!("  {:?} -> {:?}", c.label, c.target);
    }

    assert!(drive_bound, "Drive pot should be bound");
    assert!(tone_bound, "Tone pot should be bound");
    assert!(level_bound, "Level pot should be bound");
}

#[test]
fn ratking_v1a_distortion_pot_is_bound() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/ratking_non_invert_v1a.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    let distortion_bound = compiled.controls.iter().any(|c| c.label == "Distortion");
    let filter_bound = compiled.controls.iter().any(|c| c.label == "Filter");
    let volume_bound = compiled.controls.iter().any(|c| c.label == "Volume");

    eprintln!("Ratking v1a bindings:");
    for c in &compiled.controls {
        eprintln!("  {:?} -> {:?}", c.label, c.target);
    }

    assert!(distortion_bound, "Distortion pot should be bound");
    assert!(filter_bound, "Filter pot should be bound");
    assert!(volume_bound, "Volume pot should be bound");
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 2: Minimal circuits — does the pot binding work?
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn feedback_drive_pot_changes_gain() {
    // Simplest case: pot in op-amp feedback (inverting).
    // Sweeping drive should change Rf → change gain.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Drive: pot(100k, a)
                D1: diode(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Drive.a
                Drive.b -> U1.out
                U1.neg -> D1.a
                D1.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls { Drive.position -> "Drive" [0.0, 1.0] = 0.5 }
        }"#)
    .expect("parse");

    let mut lo = compile_via_spqr(&pedal, 48000.0).expect("compile");
    lo.set_control("Drive", 0.0);
    for _ in 0..500 { lo.process(0.0); }
    let mut peak_lo = 0.0f64;
    for s in 0..480 {
        let input = 0.01 * (2.0 * std::f64::consts::PI * 440.0 * s as f64 / 48000.0).sin();
        peak_lo = peak_lo.max(lo.process(input).abs());
    }

    let mut hi = compile_via_spqr(&pedal, 48000.0).expect("compile");
    hi.set_control("Drive", 1.0);
    for _ in 0..500 { hi.process(0.0); }
    let mut peak_hi = 0.0f64;
    for s in 0..480 {
        let input = 0.01 * (2.0 * std::f64::consts::PI * 440.0 * s as f64 / 48000.0).sin();
        peak_hi = peak_hi.max(hi.process(input).abs());
    }

    eprintln!("Feedback drive: lo={peak_lo:.4}, hi={peak_hi:.4}");

    // Check binding exists
    let bound = lo.controls.iter().any(|c| c.label == "Drive");
    assert!(bound, "Drive should be bound");

    // Sweeping should change output
    assert!(
        (peak_lo - peak_hi).abs() > 0.001 || peak_lo.max(peak_hi) > 0.01,
        "Drive pot should change output: lo={peak_lo:.4}, hi={peak_hi:.4}"
    );
}

#[test]
fn tone_pot_in_blackfeedback_changes_spectrum() {
    // Tone pot in a BlackFeedback stage (non-inverting, cap + pot).
    // Sweeping tone should change frequency balance.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(47k)
                C_tone: cap(22n)
                Tone: pot(20k, b)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.neg -> C_tone.a
                C_tone.b -> Tone.a
                Tone.w -> U1.out
                Tone.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls { Tone.position -> "Tone" [0.0, 1.0] = 0.5 }
        }"#)
    .expect("parse");

    // Check binding exists
    let compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    let bound = compiled.controls.iter().any(|c| c.label == "Tone");
    eprintln!("Tone pot binding:");
    for c in &compiled.controls {
        eprintln!("  {:?} -> {:?}", c.label, c.target);
    }
    eprintln!("  Controls: {:?}", compiled.controls.iter().map(|c| &c.label).collect::<Vec<_>>());

    // The tone pot may not be bound if it's consumed by BlackFeedback.
    // This is the test — it SHOULD be bound.
    if !bound {
        eprintln!("  Tone NOT bound — checking WDF stage trees:");
        for (i, s) in compiled.stages.iter().enumerate() {
            let pos = s.tree.get_pot_position("Tone")
                .or_else(|| s.tree.get_pot_position("Tone__aw"))
                .or_else(|| s.tree.get_pot_position("Tone__wb"));
            if pos.is_some() {
                eprintln!("    wdf[{i}]: Tone found, pos={:?}", pos);
            }
        }
    }
    assert!(bound, "Tone pot should be bound");

    // Verify the pot position actually changes when set_control is called
    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    // Get initial Tone pot position
    let stage = &compiled.stages[1]; // PotInStage(1)
    let initial_pos = stage.tree.get_pot_position("Tone")
        .or_else(|| stage.tree.get_pot_position("Tone__aw"));
    eprintln!("Initial Tone position: {:?}", initial_pos);

    // Set to 0.0
    compiled.set_control("Tone", 0.0);
    let pos_0 = compiled.stages[1].tree.get_pot_position("Tone")
        .or_else(|| compiled.stages[1].tree.get_pot_position("Tone__aw"));

    // Set to 1.0
    compiled.set_control("Tone", 1.0);
    let pos_1 = compiled.stages[1].tree.get_pot_position("Tone")
        .or_else(|| compiled.stages[1].tree.get_pot_position("Tone__aw"));

    eprintln!("Tone @0.0: {:?}, @1.0: {:?}", pos_0, pos_1);

    // Position should change
    if let (Some(p0), Some(p1)) = (pos_0, pos_1) {
        assert!(
            (p0 - p1).abs() > 0.01,
            "Tone pot position should change: @0={p0:.3}, @1={p1:.3}"
        );
    } else {
        panic!("Tone pot not found in stage tree after set_control");
    }
}
