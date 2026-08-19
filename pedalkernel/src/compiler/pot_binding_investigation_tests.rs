// ═══════════════════════════════════════════════════════════════════════════
// Pot binding investigation tests
//
// The Screamer/SD-1 Level pot doesn't respond to sweeps. The pot is in
// the tone+output WDF stage but find_pot_binding can't find it.
// These tests isolate where the binding breaks.
// ═══════════════════════════════════════════════════════════════════════════

use super::spqr_build::compile_via_spqr;
use super::test_support::load_legend_source;
use crate::PedalProcessor;

const SR: f64 = 48000.0;
const FREQ: f64 = 440.0;

fn measure_peak(compiled: &mut impl PedalProcessor, amp: f64) -> f64 {
    for s in 0..4000 {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }
    let mut peak = 0.0f64;
    for s in 0..500 {
        let out =
            compiled.process(amp * (std::f64::consts::TAU * FREQ * (4000 + s) as f64 / SR).sin());
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
    // The pot changes output — ratio may be <1 (inverted) or >1. Either way, it's NOT 1.0.
    assert!(
        (ratio - 1.0).abs() > 0.3,
        "Standalone pot should change output: ratio={ratio:.2}"
    );
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
    assert!(
        ratio > 1.5,
        "Pot in passive group should change output: ratio={ratio:.2}"
    );
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
    eprintln!(
        "Level binding: {:?}",
        level_binding.map(|b| (&b.label, &b.component_id))
    );
    assert!(level_binding.is_some(), "Level pot should be bound");

    let mut low = compile_via_spqr(&pedal, SR).expect("compile");
    low.set_control("Level", 0.2);
    let peak_low = measure_peak(&mut low, 0.1);

    let mut high = compile_via_spqr(&pedal, SR).expect("compile");
    high.set_control("Level", 0.8);
    let peak_high = measure_peak(&mut high, 0.1);

    let ratio = peak_high / peak_low.max(0.0001);
    eprintln!("Level after gain: low={peak_low:.4}V high={peak_high:.4}V ratio={ratio:.2}");
    assert!(
        (ratio - 1.0).abs() > 0.3,
        "Level pot after gain should change output: ratio={ratio:.2}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Screamer Level pot — the actual failing case
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_level_pot_is_bound() {
    let source = load_legend_source("screamer").expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // List all bindings
    for ctrl in &compiled.controls {
        eprintln!("  binding: '{}' → comp={}", ctrl.label, ctrl.component_id);
    }

    let level_binding = compiled.controls.iter().find(|c| c.label == "Level");
    assert!(
        level_binding.is_some(),
        "Screamer Level pot must be bound. Controls: {:?}",
        compiled
            .controls
            .iter()
            .map(|c| &c.label)
            .collect::<Vec<_>>()
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. Pot sweep monotonicity — output should increase with position
// ═══════════════════════════════════════════════════════════════════════════
#[test]
fn pot_sweep_monotonic() {
    // Sweep a pot from 0.1 to 0.9 in steps. Output should increase monotonically.
    // With double taper, the response is compressed and may not be monotonic.
    let source = r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                Vol: pot(100k, b)
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

    let positions = [0.1, 0.3, 0.5, 0.7, 0.9];
    let mut peaks: Vec<(f64, f64)> = Vec::new();

    for &pos in &positions {
        let pedal = crate::dsl::parse_pedal_file(source).expect("parse");
        let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");
        compiled.set_control("Volume", pos);
        let peak = measure_peak(&mut compiled, 0.1);
        peaks.push((pos, peak));
    }

    eprintln!("Pot sweep:");
    for (pos, peak) in &peaks {
        eprintln!("  pos={pos:.1}: peak={peak:.4}V");
    }

    // Check monotonicity: each step should increase
    for i in 1..peaks.len() {
        assert!(
            peaks[i].1 > peaks[i - 1].1 * 0.9,
            "Pot sweep should be monotonic: pos={:.1}→{:.4}V < pos={:.1}→{:.4}V",
            peaks[i].0,
            peaks[i].1,
            peaks[i - 1].0,
            peaks[i - 1].1
        );
    }

    // Check range: high position should be > 3x low position
    let ratio = peaks.last().unwrap().1 / peaks.first().unwrap().1.max(0.0001);
    eprintln!("  Range ratio: {ratio:.2}x");
    assert!(
        ratio > 3.0,
        "Pot should have >3x range from 0.1 to 0.9: ratio={ratio:.2}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 7. Screamer Level: 0 quiet, 1 loud
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_level_0_quiet_1_loud() {
    let source = load_legend_source("screamer").expect("read");

    let pedal_quiet = crate::dsl::parse_pedal_file(&source).expect("parse");
    let mut quiet = compile_via_spqr(&pedal_quiet, SR).expect("compile");
    quiet.set_control("Level", 0.1);
    let peak_quiet = measure_peak(&mut quiet, 0.1);

    let pedal_loud = crate::dsl::parse_pedal_file(&source).expect("parse");
    let mut loud = compile_via_spqr(&pedal_loud, SR).expect("compile");
    loud.set_control("Level", 0.9);
    let peak_loud = measure_peak(&mut loud, 0.1);

    eprintln!("Screamer Level: quiet(0.1)={peak_quiet:.4}V loud(0.9)={peak_loud:.4}V");
    let ratio = peak_loud / peak_quiet.max(0.0001);
    eprintln!("  Ratio: {ratio:.2}x");

    assert!(
        peak_loud > peak_quiet * 2.0,
        "Level 0.9 should be louder than 0.1: ratio={ratio:.2}x"
    );
}

#[test]
fn sd1_level_0_quiet_1_loud() {
    let source = load_legend_source("sd1").expect("read");

    let pedal_quiet = crate::dsl::parse_pedal_file(&source).expect("parse");
    let mut quiet = compile_via_spqr(&pedal_quiet, SR).expect("compile");
    quiet.set_control("Level", 0.1);
    let peak_quiet = measure_peak(&mut quiet, 0.1);

    let pedal_loud = crate::dsl::parse_pedal_file(&source).expect("parse");
    let mut loud = compile_via_spqr(&pedal_loud, SR).expect("compile");
    loud.set_control("Level", 0.9);
    let peak_loud = measure_peak(&mut loud, 0.1);

    eprintln!("SD-1 Level: quiet(0.1)={peak_quiet:.4}V loud(0.9)={peak_loud:.4}V");
    let ratio = peak_loud / peak_quiet.max(0.0001);
    eprintln!("  Ratio: {ratio:.2}x");

    assert!(
        peak_loud > peak_quiet * 2.0,
        "Level 0.9 should be louder than 0.1: ratio={ratio:.2}x"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// (trace helper — kept for debugging)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_level_pot_changes_output_trace() {
    // Trace: check pot rp before and after set_control
    let source = load_legend_source("screamer").expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Check pot position before
    for stage in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = stage {
            if let Some(pos) = w.tree.get_pot_position("Level") {
                eprintln!("Before set_control: Level pot position={pos:.4}");
            }
        }
    }

    compiled.set_control("Level", 0.1);

    // Check pot position after
    for stage in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = stage {
            if let Some(pos) = w.tree.get_pot_position("Level") {
                eprintln!("After set_control(0.1): Level pot position={pos:.4}");
            }
        }
    }

    compiled.set_control("Level", 0.9);

    for stage in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = stage {
            if let Some(pos) = w.tree.get_pot_position("Level") {
                eprintln!("After set_control(0.9): Level pot position={pos:.4}");
            }
        }
    }
}

#[test]
fn screamer_level_pot_changes_output() {
    let source = load_legend_source("screamer").expect("read");

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
    assert!(
        (ratio - 1.0).abs() > 0.3,
        "Screamer Level should change output: ratio={ratio:.2}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. Check if pot leaf exists in the WDF tree
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_level_pot_leaf_in_tree() {
    let source = load_legend_source("screamer").expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Find the tone+output stage and check if Level pot is in its tree
    for stage in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = stage {
            #[cfg(debug_assertions)]
            if w.debug_label.contains("Level") {
                let has_pot = w.has_pot("Level");
                eprintln!(
                    "Tone stage [{}]: Level pot owned by stage? {has_pot}",
                    w.debug_label
                );
                eprintln!(
                    "  get_pot_position('Level'): {:?}",
                    w.tree.get_pot_position("Level")
                );
                eprintln!(
                    "  get_pot_position('Level__aw'): {:?}",
                    w.tree.get_pot_position("Level__aw")
                );
                eprintln!(
                    "  get_pot_position('Level__wb'): {:?}",
                    w.tree.get_pot_position("Level__wb")
                );
                assert!(has_pot, "Level pot must be controllable by the WDF stage");
            }
        }
    }
}
