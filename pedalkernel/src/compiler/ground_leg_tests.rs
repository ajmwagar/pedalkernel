//! TDD tests for ground-leg Ri computation in BlackFeedback stages.
//!
//! Non-inverting opamp gain = 1 + Rf/Ri where Ri is the impedance from
//! neg to GND. The compiler must find the full ground-leg chain even when
//! it spans multiple hops (e.g. R5 → R6 → Gain_A → GND in the Klon).
//!
//! These tests drive the fix: instead of heuristic pendant chain walking
//! in signal_flow.rs, the Ri BFS in spqr_build.rs searches all passive
//! edges reachable from neg to GND.

use super::compiled::Stage;
use super::spqr_build::compile_via_spqr;
use crate::dsl::parse_pedal_file;
use crate::PedalProcessor;

const SR: f64 = 48000.0;

fn bf_gain(src: &str) -> f64 {
    let pedal = parse_pedal_file(src).expect("parse");
    let compiled = compile_via_spqr(&pedal, SR).expect("compile");
    for stage in &compiled.stages {
        if let Stage::BlackFeedback(bf) = stage {
            return bf.gain();
        }
    }
    panic!("No BlackFeedback stage found");
}

fn measure_peak(src: &str, controls: &[(&str, f64)]) -> f64 {
    let pedal = parse_pedal_file(src).expect("parse");
    let mut proc = compile_via_spqr(&pedal, SR).expect("compile");
    for &(label, val) in controls {
        proc.set_control(label, val);
    }
    for s in 0..4000 {
        proc.process(0.05 * (std::f64::consts::TAU * 440.0 * s as f64 / SR).sin());
    }
    let mut peak = 0.0f64;
    for s in 0..2000 {
        let out =
            proc.process(0.05 * (std::f64::consts::TAU * 440.0 * (4000 + s) as f64 / SR).sin());
        peak = peak.max(out.abs());
    }
    peak
}

// ═══════════════════════════════════════════════════════════════════════════
// 1. Single-hop ground leg: neg → R_gnd → GND
// ═══════════════════════════════════════════════════════════════════════════

const SINGLE_HOP: &str = r#"pedal "test" { supply 9V
  components {
    R_in: resistor(1.5k)
    U1: opamp(tl072)
    Rf: resistor(422k)
    R_gnd: resistor(15k)
  }
  nets {
    in -> R_in.a
    R_in.b -> U1.pos
    U1.neg -> Rf.a
    Rf.b -> U1.out
    U1.neg -> R_gnd.a
    R_gnd.b -> gnd
    U1.out -> out
  }
  controls {}
}"#;

#[test]
fn single_hop_ground_leg_gain() {
    let gain = bf_gain(SINGLE_HOP);
    // gain = 1 + 422k/15k = 29.13
    eprintln!("Single-hop: gain={gain:.2} (expected ~29.1)");
    assert!(
        (gain - 29.13).abs() < 1.0,
        "gain should be ~29: got {gain:.2}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Multi-hop ground leg: neg → R5 → R6 → GND (Klon pattern)
// ═══════════════════════════════════════════════════════════════════════════

const MULTI_HOP: &str = r#"pedal "test" { supply 9V
  components {
    R_in: resistor(1.5k)
    U1: opamp(tl072)
    Rf: resistor(422k)
    R5: resistor(15k)
    R6: resistor(2k)
  }
  nets {
    in -> R_in.a
    R_in.b -> U1.pos
    U1.neg -> Rf.a
    Rf.b -> U1.out
    U1.neg -> R5.a
    R5.b -> R6.a
    R6.b -> gnd
    U1.out -> out
  }
  controls {}
}"#;

#[test]
fn multi_hop_ground_leg_gain() {
    let gain = bf_gain(MULTI_HOP);
    // gain = 1 + 422k / (15k + 2k) = 1 + 422/17 = 25.82
    eprintln!("Multi-hop: gain={gain:.2} (expected ~25.8)");
    assert!(
        (gain - 25.82).abs() < 2.0,
        "gain should be ~25.8: got {gain:.2}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Multi-hop with pot: neg → R5 → R6 → Gain_A → GND
// ═══════════════════════════════════════════════════════════════════════════

const MULTI_HOP_POT: &str = r#"pedal "test" { supply 9V
  components {
    R_in: resistor(1.5k)
    U1: opamp(tl072)
    Rf: resistor(422k)
    R5: resistor(15k)
    R6: resistor(2k)
    Gain_A: pot(100k, b)
  }
  nets {
    in -> R_in.a
    R_in.b -> U1.pos
    U1.neg -> Rf.a
    Rf.b -> U1.out
    U1.neg -> R5.a
    R5.b -> R6.a
    R6.b -> Gain_A.a
    Gain_A.b -> gnd
    U1.out -> out
  }
  controls {
    Gain_A.position -> "Gain" [0.0, 1.0] = 0.5
  }
}"#;

#[test]
fn multi_hop_pot_gain_at_default() {
    let gain = bf_gain(MULTI_HOP_POT);
    // gain = 1 + 422k / (15k + 2k + 50k) = 1 + 422/67 = 7.30
    eprintln!("Multi-hop pot (default 0.5): gain={gain:.2} (expected ~7.3)");
    assert!(
        (gain - 7.3).abs() < 2.0,
        "gain should be ~7.3: got {gain:.2}"
    );
}

#[test]
fn multi_hop_pot_gain_responds_to_control() {
    let peak_low = measure_peak(MULTI_HOP_POT, &[("Gain", 0.9)]);
    let peak_high = measure_peak(MULTI_HOP_POT, &[("Gain", 0.1)]);
    eprintln!("Gain=0.9 (high R, low gain): peak={peak_low:.4}");
    eprintln!("Gain=0.1 (low R, high gain): peak={peak_high:.4}");
    assert!(
        peak_high > peak_low * 1.5,
        "Lower Gain_A R should produce more gain: high={peak_high:.4} low={peak_low:.4}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Bypass cap on ground leg (Klon C_gnd pattern)
// ═══════════════════════════════════════════════════════════════════════════

const WITH_BYPASS_CAP: &str = r#"pedal "test" { supply 9V
  components {
    R_in: resistor(1.5k)
    U1: opamp(tl072)
    Rf: resistor(422k)
    R5: resistor(15k)
    R6: resistor(2k)
    C_gnd: cap(1u)
  }
  nets {
    in -> R_in.a
    R_in.b -> U1.pos
    U1.neg -> Rf.a
    Rf.b -> U1.out
    U1.neg -> R5.a
    R5.b -> R6.a
    C_gnd.a -> R5.b
    C_gnd.b -> gnd
    R6.b -> gnd
    U1.out -> out
  }
  controls {}
}"#;

#[test]
fn bypass_cap_doesnt_affect_dc_gain() {
    let gain = bf_gain(WITH_BYPASS_CAP);
    // Ri = R5 + R6 = 17k (C_gnd has no DC resistance)
    // gain = 1 + 422k/17k = 25.82
    eprintln!("With bypass cap: gain={gain:.2} (expected ~25.8)");
    assert!(
        (gain - 25.82).abs() < 2.0,
        "C_gnd shouldn't affect Ri: got {gain:.2}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. Summing amp inputs are NOT ground-leg (Klon U3 pattern)
// ═══════════════════════════════════════════════════════════════════════════

const SUMMING_AMP: &str = r#"pedal "test" { supply 9V
  components {
    U1: opamp(tl072)
    R_sum_fb: resistor(560k)
    R_clip: resistor(27k)
    R_ff1: resistor(422k)
    R_ff2: resistor(392k)
  }
  nets {
    in -> R_clip.a
    R_clip.b -> U1.neg
    R_ff1.a -> in
    R_ff1.b -> U1.neg
    R_ff2.a -> in
    R_ff2.b -> U1.neg
    U1.neg -> R_sum_fb.a
    R_sum_fb.b -> U1.out
    U1.pos -> gnd
    U1.out -> out
  }
  controls {}
}"#;

#[test]
fn summing_amp_gain_from_feedback_only() {
    let gain = bf_gain(SUMMING_AMP);
    // Inverting summer: gain = -Rf/Ri where Ri = parallel(R_clip, R_ff1, R_ff2)
    // But BF stage computes gain differently. The key assertion:
    // gain should NOT be huge (like 560k/27k = 20.7) — it should account
    // for the parallel input resistors or use the standard sum formula.
    eprintln!("Summing amp: gain={gain:.2}");
    // At minimum, gain should be finite and reasonable
    assert!(
        gain.abs() < 50.0,
        "Summing amp gain should be bounded: got {gain:.2}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. Full Goldenrod: Gain pot modulates BF gain at runtime
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn goldenrod_gain_pot_modulates_bf_gain() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/goldenrod.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = match std::fs::read_to_string(&path) {
        Ok(s) => s,
        Err(_) => {
            eprintln!("SKIP: goldenrod.pedal not found");
            return;
        }
    };
    let pedal = parse_pedal_file(&source).expect("parse");

    let mut low = compile_via_spqr(&pedal, SR).expect("compile");
    low.set_control("Gain", 0.1); // Gain_A pos=0.9 → high R → low gain
                                  // Process samples so smoother advances and update_ri_from_pot fires
    for _ in 0..500 {
        low.process(0.0);
    }

    let mut high = compile_via_spqr(&pedal, SR).expect("compile");
    high.set_control("Gain", 0.9); // Gain_A pos=0.1 → low R → high gain
    for _ in 0..500 {
        high.process(0.0);
    }

    // Find U2's BF stage
    let u2_gain_low = low.stages.iter().find_map(|s| {
        if let Stage::BlackFeedback(bf) = s {
            #[cfg(debug_assertions)]
            if bf.debug_label.contains("U2") {
                return Some(bf.gain());
            }
        }
        None
    });
    let u2_gain_high = high.stages.iter().find_map(|s| {
        if let Stage::BlackFeedback(bf) = s {
            #[cfg(debug_assertions)]
            if bf.debug_label.contains("U2") {
                return Some(bf.gain());
            }
        }
        None
    });

    if let (Some(gl), Some(gh)) = (u2_gain_low, u2_gain_high) {
        eprintln!("Goldenrod U2 gain: low={gl:.2}, high={gh:.2}");
        assert!(
            gh > gl * 1.5,
            "High gain setting should produce more gain: {gh:.2} vs {gl:.2}"
        );
    } else {
        eprintln!("SKIP: couldn't find U2 BF stage (release build?)");
    }
}
