// ═══════════════════════════════════════════════════════════════════════════
// Pot wiper voltage extraction tests
//
// The Level pot in Screamer/SD-1 acts as a voltage divider:
//   signal → Vol.a → [aw] → Vol.w (wiper) → [wb] → Vol.b → GND
// The output should be taken at the wiper: V_wiper = V_in × R_wb/(R_aw+R_wb)
//
// At position 0.0: R_aw=max, R_wb=0 → V_wiper=0 (quiet)
// At position 1.0: R_aw=0, R_wb=max → V_wiper=V_in (loud)
// At position 0.5: V_wiper = 0.5×V_in
//
// The response should be monotonic, NOT a V-shape.
// ═══════════════════════════════════════════════════════════════════════════

use super::spqr_build::compile_via_spqr;
use super::test_support::load_legend_source;
use crate::PedalProcessor;

const SR: f64 = 48000.0;
const FREQ: f64 = 440.0;

fn measure_peak_at_position(source: &str, control: &str, pos: f64) -> f64 {
    let pedal = crate::dsl::parse_pedal_file(source).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");
    compiled.set_control(control, pos);
    let amp = 0.1;
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
// 1. Standalone pot divider: full sweep should be monotonic
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn pot_divider_full_sweep() {
    let source = r#"
        pedal "test" { supply 9V
            components {
                Vol: pot(100k, b)
            }
            nets {
                in -> Vol.a
                Vol.b -> gnd
                Vol.w -> out
            }
            controls {
                Vol.position -> "Vol" [0.0, 1.0] = 0.5
            }
        }"#;

    let positions = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
    let mut prev_peak = 0.0f64;
    let mut monotonic = true;

    eprintln!("Pot divider sweep:");
    for &pos in &positions {
        let peak = measure_peak_at_position(source, "Vol", pos);
        let arrow = if peak > prev_peak + 0.001 {
            "↑"
        } else if peak < prev_peak - 0.001 {
            "↓ V-SHAPE!"
        } else {
            "→"
        };
        eprintln!("  pos={pos:.1}: peak={peak:.4}V {arrow}");
        if pos > 0.0 && peak < prev_peak - 0.001 {
            monotonic = false;
        }
        prev_peak = peak;
    }

    assert!(monotonic, "Pot sweep must be monotonic (no V-shape)");
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Pot after gain stage (Screamer Level topology)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn level_pot_after_gain_full_sweep() {
    let source = r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D: diode_pair(silicon)
                Level: pot(100k, a)
                C_out: cap(10u)
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
                Level.w -> C_out.a
                C_out.b -> R_out.a
                R_out.b -> out
            }
            controls {
                Level.position -> "Level" [0.0, 1.0] = 0.5
            }
        }"#;

    let positions = [0.1, 0.3, 0.5, 0.7, 0.9];
    let mut prev_peak = 0.0f64;
    let mut monotonic = true;

    eprintln!("Level after gain sweep:");
    for &pos in &positions {
        let peak = measure_peak_at_position(source, "Level", pos);
        let arrow = if peak > prev_peak + 0.001 {
            "↑"
        } else if peak < prev_peak - 0.001 {
            "↓ V-SHAPE!"
        } else {
            "→"
        };
        eprintln!("  pos={pos:.1}: peak={peak:.4}V {arrow}");
        if pos > 0.1 && peak < prev_peak - 0.001 {
            monotonic = false;
        }
        prev_peak = peak;
    }

    assert!(monotonic, "Level pot sweep must be monotonic (no V-shape)");
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Screamer Level: full sweep monotonic
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_level_full_sweep() {
    let source = load_legend_source("screamer").expect("read");
    // Remove calibrate to prevent output_gain normalization
    let source = source.replace("calibrate", "# calibrate");
    eprintln!(
        "Screamer controls: {:?}",
        crate::dsl::parse_pedal_file(&source)
            .unwrap()
            .controls
            .iter()
            .map(|c| format!("{} [{},{}] = {}", c.label, c.range.0, c.range.1, c.default))
            .collect::<Vec<_>>()
    );

    let positions = [0.1, 0.3, 0.5, 0.7, 0.9];
    let mut prev_peak = 0.0f64;
    let mut monotonic = true;

    eprintln!("Screamer Level sweep:");
    for &pos in &positions {
        let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
        let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");
        compiled.set_control("Level", pos);
        let amp = 0.1;
        for s in 0..4000 {
            compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
        }
        let mut peak = 0.0f64;
        for s in 0..500 {
            let out = compiled
                .process(amp * (std::f64::consts::TAU * FREQ * (4000 + s) as f64 / SR).sin());
            peak = peak.max(out.abs());
        }
        let arrow = if peak > prev_peak + 0.001 {
            "↑"
        } else if peak < prev_peak - 0.001 {
            "↓ V-SHAPE!"
        } else {
            "→"
        };
        eprintln!("  pos={pos:.1}: peak={peak:.4}V {arrow}");
        if pos > 0.1 && peak < prev_peak - 0.001 {
            monotonic = false;
        }
        prev_peak = peak;
    }

    assert!(
        monotonic,
        "Screamer Level sweep must be monotonic (no V-shape)"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Direct set_pot bypass — does the WDF tree respond at all?
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn direct_set_pot_changes_tree() {
    let source = r#"
        pedal "test" { supply 9V
            components {
                Vol: pot(100k, b)
            }
            nets {
                in -> Vol.a
                Vol.b -> gnd
                Vol.w -> out
            }
            controls {
                Vol.position -> "Vol" [0.0, 1.0] = 0.5
            }
        }"#;

    let pedal = crate::dsl::parse_pedal_file(source).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Read initial pot position
    for stage in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = stage {
            let pos = w.tree.get_pot_position("Vol");
            let res = w.tree.get_pot_resistance("Vol");
            eprintln!("Initial: pos={pos:?} res={res:?}");
        }
    }

    // Directly call set_pot on the WDF stage
    for stage in &mut compiled.stages {
        if let super::compiled::Stage::Wdf(w) = stage {
            let changed = w.set_pot("Vol", 0.1);
            eprintln!("set_pot('Vol', 0.1) → changed={changed}");
            w.flush_recompute();
            let pos = w.tree.get_pot_position("Vol");
            let res = w.tree.get_pot_resistance("Vol");
            eprintln!("After set_pot(0.1): pos={pos:?} res={res:?}");
        }
    }

    for stage in &mut compiled.stages {
        if let super::compiled::Stage::Wdf(w) = stage {
            let changed = w.set_pot("Vol", 0.9);
            w.flush_recompute();
            let pos = w.tree.get_pot_position("Vol");
            let res = w.tree.get_pot_resistance("Vol");
            eprintln!("After set_pot(0.9): pos={pos:?} res={res:?}");
        }
    }
}
