// ═══════════════════════════════════════════════════════════════════════════
// Pot binding investigation tests
//
// The Screamer/SD-1 Level pot doesn't respond to sweeps. The pot is in
// the tone+output WDF stage but find_pot_binding can't find it.
// These tests isolate where the binding breaks.
// ═══════════════════════════════════════════════════════════════════════════

use super::spqr_build::compile_via_spqr;
use crate::PedalProcessor;

const SR: f64 = 48000.0;
const FREQ: f64 = 440.0;

fn measure_peak(compiled: &mut impl PedalProcessor, amp: f64) -> f64 {
    for s in 0..4000 {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }
    let mut peak = 0.0f64;
    for s in 0..500 {
        let out = compiled.process(
            amp * (std::f64::consts::TAU * FREQ * (4000 + s) as f64 / SR).sin()
        );
        peak = peak.max(out.abs());
    }
    peak
}

// ═══════════════════════════════════════════════════════════════════════════
// 1. Simple pot in standalone stage — does binding work at all?
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn standalone_pot_binding_works() {
    let source = r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                Vol: pot(100k, a)
            }
            nets {
                in -> R_in.a
                R_in.b -> Vol.a
                Vol.b -> gnd
                Vol.w -> out
            }
            controls {
                Vol.position -> "Volume" [0.0, 1.0] = 0.5
            }
        }"#;

    let pedal = crate::dsl::parse_pedal_file(source).expect("parse");
    let mut low = compile_via_spqr(&pedal, SR).expect("compile");
    low.set_control("Volume", 0.2);
    let peak_low = measure_peak(&mut low, 0.1);

    let mut high = compile_via_spqr(&pedal, SR).expect("compile");
    high.set_control("Volume", 0.8);
    let peak_high = measure_peak(&mut high, 0.1);

    let ratio = peak_high / peak_low.max(0.0001);
    eprintln!("Standalone pot: low={peak_low:.4}V high={peak_high:.4}V ratio={ratio:.2}");
    assert!(ratio > 1.5, "Standalone pot should change output: ratio={ratio:.2}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Pot in multi-edge passive group — does binding find it?
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn pot_in_passive_group_binding_works() {
    // Pot inside a passive group with other components (tone-like)
    let source = r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(1k)
                C_tone: cap(220n)
                Vol: pot(100k, a)
                R_out: resistor(10k)
            }
            nets {
                in -> R_in.a
                R_in.b -> C_tone.a, Vol.a
                C_tone.b -> gnd
                Vol.b -> gnd
                Vol.w -> R_out.a
                R_out.b -> out
            }
            controls {
                Vol.position -> "Volume" [0.0, 1.0] = 0.5
            }
        }"#;

    let pedal = crate::dsl::parse_pedal_file(source).expect("parse");
    let mut low = compile_via_spqr(&pedal, SR).expect("compile");
    low.set_control("Volume", 0.2);
    let peak_low = measure_peak(&mut low, 0.1);

    let mut high = compile_via_spqr(&pedal, SR).expect("compile");
    high.set_control("Volume", 0.8);
    let peak_high = measure_peak(&mut high, 0.1);

    let ratio = peak_high / peak_low.max(0.0001);
    eprintln!("Pot in passive group: low={peak_low:.4}V high={peak_high:.4}V ratio={ratio:.2}");
    assert!(ratio > 1.5, "Pot in passive group should change output: ratio={ratio:.2}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Pot after gain stage — Screamer-like topology
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn pot_after_gain_stage_binding_works() {
    let source = r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D: diode_pair(silicon)
                Level: pot(100k, a)
                R_out: resistor(10k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.out -> D.a
                D.b -> U1.neg
                U1.pos -> gnd
                U1.out -> Level.a
                Level.b -> gnd
                Level.w -> R_out.a
                R_out.b -> out
            }
            controls {
                Level.position -> "Level" [0.0, 1.0] = 0.5
            }
        }"#;

    let pedal = crate::dsl::parse_pedal_file(source).expect("parse");

    // Check if binding exists
    let compiled = compile_via_spqr(&pedal, SR).expect("compile");
    let level_binding = compiled.controls.iter().find(|c| c.label == "Level");
    eprintln!("Level binding: {:?}", level_binding.map(|b| (&b.label, &b.component_id)));
    assert!(level_binding.is_some(), "Level pot should be bound");

    let mut low = compile_via_spqr(&pedal, SR).expect("compile");
    low.set_control("Level", 0.2);
    let peak_low = measure_peak(&mut low, 0.1);

    let mut high = compile_via_spqr(&pedal, SR).expect("compile");
    high.set_control("Level", 0.8);
    let peak_high = measure_peak(&mut high, 0.1);

    let ratio = peak_high / peak_low.max(0.0001);
    eprintln!("Level after gain: low={peak_low:.4}V high={peak_high:.4}V ratio={ratio:.2}");
    assert!(ratio > 1.5, "Level pot after gain should change output: ratio={ratio:.2}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Screamer Level pot — the actual failing case
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_level_pot_is_bound() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/screamer.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // List all bindings
    for ctrl in &compiled.controls {
        eprintln!("  binding: '{}' → comp={}", ctrl.label, ctrl.component_id);
    }

    let level_binding = compiled.controls.iter().find(|c| c.label == "Level");
    assert!(level_binding.is_some(),
        "Screamer Level pot must be bound. Controls: {:?}",
        compiled.controls.iter().map(|c| &c.label).collect::<Vec<_>>());
}

#[test]
fn screamer_level_pot_changes_output() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/screamer.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read");

    let pedal_low = crate::dsl::parse_pedal_file(&source).expect("parse");
    let mut low = compile_via_spqr(&pedal_low, SR).expect("compile");
    low.set_control("Level", 0.2);
    let peak_low = measure_peak(&mut low, 0.1);

    let pedal_high = crate::dsl::parse_pedal_file(&source).expect("parse");
    let mut high = compile_via_spqr(&pedal_high, SR).expect("compile");
    high.set_control("Level", 0.8);
    let peak_high = measure_peak(&mut high, 0.1);

    let ratio = peak_high / peak_low.max(0.0001);
    eprintln!("Screamer Level: low={peak_low:.4}V high={peak_high:.4}V ratio={ratio:.2}");
    assert!(ratio > 1.5, "Screamer Level should change output: ratio={ratio:.2}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. Check if pot leaf exists in the WDF tree
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_level_pot_leaf_in_tree() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/screamer.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Find the tone+output stage and check if Level pot is in its tree
    for stage in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = stage {
            #[cfg(debug_assertions)]
            if w.debug_label.contains("Level") {
                let has_pot = w.tree.get_pot_position("Level").is_some()
                    || w.tree.get_pot_position("Level__aw").is_some()
                    || w.tree.get_pot_position("Level__wb").is_some();
                eprintln!("Tone stage [{}]: Level pot in tree? {has_pot}", w.debug_label);
                eprintln!("  get_pot_position('Level'): {:?}", w.tree.get_pot_position("Level"));
                eprintln!("  get_pot_position('Level__aw'): {:?}", w.tree.get_pot_position("Level__aw"));
                eprintln!("  get_pot_position('Level__wb'): {:?}", w.tree.get_pot_position("Level__wb"));
                assert!(has_pot, "Level pot leaf must exist in the WDF tree");
            }
        }
    }
}
