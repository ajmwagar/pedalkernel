//! Tests for port semantics + coupling barriers + optimization legality.
//!
//! Validates:
//! 1. Optimization legality: v1 kick still works (no barrier), v2 kick compiles
//! 2. Negative regression: simple opamp → BlackFeedback, diode → WDF path
//! 3. Audio: no NaN, finite peak, correct frequency range

use super::spqr_build::compile_via_spqr;
use crate::dsl::parse_pedal_file;
use crate::PedalProcessor;

const SR: f64 = 48_000.0;

// ═══════════════════════════════════════════════════════════════════════════
// 1. Coupling barriers — v1 vs v2 808 kick
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn v1_808_kick_compiles_and_oscillates() {
    let src = r#"pedal "808 v1" {
  supply 9V
  components {
    U1: opamp(tl072)
    Rb1: resistor(100k)
    Rb2: resistor(100k)
    R1: resistor(150k)
    R2: resistor(150k)
    C1: cap(8.2n)
    C2: cap(8.2n)
    R_fb: resistor(470k)
    R_trig: resistor(100k)
    R_out: resistor(10k)
  }
  nets {
    vcc -> Rb1.a
    Rb1.b -> Rb2.a, U1.pos
    Rb2.b -> gnd
    U1.neg -> R1.a, C1.a
    R1.b -> R2.a, C2.a
    R2.b -> U1.out
    C1.b -> gnd
    C2.b -> gnd
    U1.neg -> R_fb.a
    R_fb.b -> U1.out
    in -> R_trig.a
    R_trig.b -> R1.b
    U1.out -> R_out.a
    R_out.b -> out
  }
  controls {}
}"#;
    let pedal = parse_pedal_file(src).unwrap();
    let mut proc = compile_via_spqr(&pedal, SR).expect("v1 kick should compile");

    proc.process(1.0);
    let mut peak = 0.0f64;
    let mut samples = Vec::new();
    for _ in 0..4800 {
        let out = proc.process(0.0);
        peak = peak.max(out.abs());
        samples.push(out);
    }

    // Should oscillate near 130Hz
    let crossings = samples.windows(2).filter(|w| w[0] * w[1] < 0.0).count();
    let freq = crossings as f64 / 2.0 / (4800.0 / SR);

    eprintln!("v1 kick: peak={peak:.4}, freq={freq:.1}Hz");
    assert!(peak > 0.01, "v1 kick should produce output: peak={peak}");
    assert!(
        freq > 90.0 && freq < 180.0,
        "v1 kick ~130Hz: got {freq:.1}Hz"
    );
}

#[test]
fn v2_808_kick_bjt_compiles_and_produces_output() {
    let src = r#"pedal "808 v2" {
  supply 9V
  components {
    U1: opamp(tl072)
    Rb1: resistor(100k)
    Rb2: resistor(100k)
    R1: resistor(150k)
    R2: resistor(150k)
    C1: cap(8.2n)
    C2: cap(8.2n)
    R_fb: resistor(470k)
    R_trig: resistor(1k)
    Q_sweep: npn(2n3904)
    R_base: resistor(47k)
    C_env: cap(100n)
    R_env: resistor(100k)
    R_out: resistor(10k)
  }
  nets {
    vcc -> Rb1.a
    Rb1.b -> Rb2.a, U1.pos
    Rb2.b -> gnd
    U1.neg -> R1.a, C1.a
    R1.b -> R2.a, C2.a
    R2.b -> U1.out
    C1.b -> gnd
    C2.b -> gnd
    U1.neg -> R_fb.a
    R_fb.b -> U1.out
    U1.neg -> Q_sweep.collector
    Q_sweep.emitter -> R1.b
    in -> C_env.a
    C_env.b -> R_base.a
    R_base.b -> Q_sweep.base
    Q_sweep.base -> R_env.a
    R_env.b -> gnd
    in -> R_trig.a
    R_trig.b -> R1.b
    U1.out -> R_out.a
    R_out.b -> out
  }
  controls {}
}"#;
    let pedal = parse_pedal_file(src).unwrap();
    let mut proc = compile_via_spqr(&pedal, SR).expect("v2 kick should compile");

    proc.process(1.0);
    let mut peak = 0.0f64;
    let mut any_bad = false;
    for _ in 0..48000 {
        let out = proc.process(0.0);
        if !out.is_finite() {
            any_bad = true;
        }
        peak = peak.max(out.abs());
    }

    eprintln!("v2 kick: peak={peak:.4}, bad={any_bad}");
    assert!(!any_bad, "v2 kick must not produce NaN/Inf");
    assert!(peak > 0.001, "v2 kick should produce output: peak={peak}");
    assert!(peak < 10.0, "v2 kick should stay bounded: peak={peak}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Negative regression — existing topologies still work
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn simple_opamp_gain_still_amplifies() {
    let src = r#"pedal "gain" {
  supply 9V
  components {
    U1: opamp(tl072)
    Ri: resistor(10k)
    Rf: resistor(100k)
  }
  nets {
    in -> Ri.a
    Ri.b -> U1.neg
    U1.neg -> Rf.a
    Rf.b -> U1.out
    U1.pos -> gnd
    U1.out -> out
  }
  controls {}
}"#;
    let pedal = parse_pedal_file(src).unwrap();
    let mut proc = compile_via_spqr(&pedal, SR).expect("opamp gain should compile");

    let mut peak = 0.0f64;
    for i in 0..4800 {
        let input = 0.01 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SR).sin();
        peak = peak.max(proc.process(input).abs());
    }

    eprintln!("opamp gain: peak={peak:.4} (expected ~0.1 for gain=10)");
    assert!(peak > 0.03, "opamp should amplify: peak={peak}");
}

#[test]
fn diode_feedback_clipper_still_clips() {
    let src = r#"pedal "clip" {
  supply 9V
  components {
    U1: opamp(jrc4558)
    Ri: resistor(4.7k)
    Rf: resistor(51k)
    D1: diode(silicon)
    D2: diode(silicon)
  }
  nets {
    in -> Ri.a
    Ri.b -> U1.neg
    U1.neg -> Rf.a
    Rf.b -> U1.out
    U1.neg -> D1.a
    D1.b -> U1.out
    U1.neg -> D2.b
    D2.a -> U1.out
    U1.pos -> gnd
    U1.out -> out
  }
  controls {}
}"#;
    let pedal = parse_pedal_file(src).unwrap();
    let mut proc = compile_via_spqr(&pedal, SR).expect("diode clipper should compile");

    let mut peak = 0.0f64;
    for i in 0..4800 {
        let input = 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SR).sin();
        peak = peak.max(proc.process(input).abs());
    }

    eprintln!("diode clipper: peak={peak:.4}");
    assert!(peak > 0.01, "clipper should produce output: peak={peak}");
    assert!(peak < 3.0, "clipper should limit output: peak={peak}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Pot + reactive does NOT trigger coupling barrier (regression guard)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tone_pot_with_cap_still_compiles_as_iir() {
    // Simple RC tone filter with pot — must NOT be blocked by coupling check.
    // ControlledConductance (pot) is not a coupling barrier.
    let src = r#"pedal "tone" {
  supply 9V
  components {
    U1: opamp(tl072)
    Rb1: resistor(100k)
    Rb2: resistor(100k)
    R_in: resistor(10k)
    Tone: pot(100k, b)
    C_tone: cap(22n)
    R_fb: resistor(100k)
  }
  nets {
    vcc -> Rb1.a
    Rb1.b -> Rb2.a, U1.pos
    Rb2.b -> gnd
    in -> R_in.a
    R_in.b -> U1.neg
    U1.neg -> R_fb.a
    R_fb.b -> U1.out
    U1.out -> Tone.a
    Tone.b -> C_tone.a
    C_tone.b -> gnd
    Tone.b -> out
  }
  controls {
    Tone.position -> "Tone" [0.0, 1.0] = 0.5
  }
}"#;
    let pedal = parse_pedal_file(src).unwrap();
    let mut proc = compile_via_spqr(&pedal, SR).expect("tone circuit should compile");

    let mut peak = 0.0f64;
    for i in 0..4800 {
        let input = 0.1 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SR).sin();
        peak = peak.max(proc.process(input).abs());
    }
    assert!(peak > 0.01, "tone circuit should pass signal: peak={peak}");
}
