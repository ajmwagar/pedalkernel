//! TDD tests for Level/Volume pot behavior.
//!
//! The Level pot is a standard output divider: signal → pot.a, pot.b → gnd,
//! pot.w → output. Position 0 = silence, position 1 = full output.
//!
//! Known bug: SPQR-decomposed groups with 11+ edges don't set the complement
//! flag, causing the pot to have no effect or V-shape behavior.

use super::spqr_build::compile_via_spqr;
use crate::dsl::parse_pedal_file;
use crate::PedalProcessor;

const SR: f64 = 48_000.0;

fn measure_output(src: &str, control: &str, position: f64) -> f64 {
    let pedal = parse_pedal_file(src).unwrap();
    let mut proc = compile_via_spqr(&pedal, SR).expect("should compile");
    proc.set_control(control, position);
    // Warmup
    for _ in 0..2000 {
        proc.process(0.0);
    }
    let mut peak = 0.0f64;
    for s in 0..4800 {
        let input = 0.1 * (2.0 * std::f64::consts::PI * 440.0 * s as f64 / SR).sin();
        peak = peak.max(proc.process(input).abs());
    }
    peak
}

/// Minimal output divider: just a pot with wiper to output.
/// This is the simplest possible Level pot circuit.
const MINIMAL_DIVIDER: &str = r#"pedal "test" {
  supply 9V
  components {
    Level: pot(100k, b)
  }
  nets {
    in -> Level.a
    Level.b -> gnd
    Level.w -> out
  }
  controls {
    Level.position -> "Level" [0.0, 1.0] = 0.5
  }
}"#;

#[test]
fn minimal_divider_position_0_is_silent() {
    let peak = measure_output(MINIMAL_DIVIDER, "Level", 0.0);
    eprintln!("minimal divider pos=0.0: peak={peak:.6}");
    assert!(peak < 0.01, "pos=0 should be near-silent: peak={peak}");
}

#[test]
fn minimal_divider_position_1_is_full() {
    let peak = measure_output(MINIMAL_DIVIDER, "Level", 1.0);
    eprintln!("minimal divider pos=1.0: peak={peak:.6}");
    assert!(peak > 0.05, "pos=1 should pass signal: peak={peak}");
}

#[test]
fn minimal_divider_monotonic_increase() {
    let p0 = measure_output(MINIMAL_DIVIDER, "Level", 0.1);
    let p5 = measure_output(MINIMAL_DIVIDER, "Level", 0.5);
    let p9 = measure_output(MINIMAL_DIVIDER, "Level", 0.9);
    eprintln!("minimal divider: p0.1={p0:.6}, p0.5={p5:.6}, p0.9={p9:.6}");
    assert!(p9 > p5, "0.9 > 0.5: {p9} vs {p5}");
    assert!(p5 > p0, "0.5 > 0.1: {p5} vs {p0}");
}

/// Screamer-like output stage: gain stage → Level pot → coupling cap → out
const SCREAMER_OUTPUT: &str = r#"pedal "test" {
  supply 9V
  components {
    U1: opamp(tl072)
    Ri: resistor(4.7k)
    Rf: resistor(51k)
    D1: diode(silicon)
    D2: diode(silicon)
    Level: pot(100k, a)
    C_out: cap(10u)
    R_out: resistor(10k)
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
    U1.out -> Level.a
    Level.b -> gnd
    Level.w -> C_out.a
    C_out.b -> R_out.a
    R_out.b -> out
    R_out.b -> gnd
  }
  controls {
    Level.position -> "Level" [0.0, 1.0] = 0.7
  }
}"#;

#[test]
fn screamer_output_level_0_quiet() {
    let peak = measure_output(SCREAMER_OUTPUT, "Level", 0.0);
    eprintln!("screamer output Level=0.0: peak={peak:.6}");
    assert!(peak < 0.05, "Level=0 should be quiet: peak={peak}");
}

#[test]
fn screamer_output_level_1_loud() {
    let peak = measure_output(SCREAMER_OUTPUT, "Level", 1.0);
    eprintln!("screamer output Level=1.0: peak={peak:.6}");
    assert!(peak > 0.05, "Level=1 should be loud: peak={peak}");
}

#[test]
fn screamer_output_level_sweep_monotonic() {
    let peaks: Vec<f64> = (0..=10)
        .map(|i| {
            let pos = i as f64 / 10.0;
            let p = measure_output(SCREAMER_OUTPUT, "Level", pos);
            eprintln!("  Level={pos:.1}: peak={p:.6}");
            p
        })
        .collect();

    // Should be monotonically increasing (with some tolerance for nonlinearity)
    let increasing_pairs = peaks.windows(2).filter(|w| w[1] >= w[0] * 0.95).count();
    eprintln!("  increasing pairs: {increasing_pairs}/10");
    assert!(
        increasing_pairs >= 7,
        "Level pot should be mostly monotonic: only {increasing_pairs}/10 increasing"
    );
}
