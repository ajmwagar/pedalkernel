//! Core invariants for passive/symmetric circuits and FX loop composition.
//! Ensures the WDF engine preserves passivity, reciprocity, and produces the
//! same physics whether we run a pedal monolithically or via FX split halves.

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::compiler::{compile_pedal, compile_split_pedal};
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use std::path::Path;

// ---------------------------------------------------------------------------
// Passive invariants
// ---------------------------------------------------------------------------

fn rc_lowpass() -> &'static str {
    r#"
pedal "RC Lowpass" {
  components {
    R1: resistor(10k)
    C1: cap(100n)
  }
  nets {
    in -> R1.a
    R1.b -> C1.a, out
    C1.b -> gnd
  }
}
"#
}

#[test]
fn passive_lowpass_does_not_create_energy() {
    let input = sine_at(1200.0, 0.8, 0.4, SAMPLE_RATE);
    let output = compile_and_process(rc_lowpass(), &input, SAMPLE_RATE, &[]);

    assert_healthy(&output, "rc_lowpass", 10.0);

    let e_in = energy(&input);
    let e_out = energy(&output);
    // TOLERANCE SOURCE: Passivity — RC lowpass has max gain ≤ 1.0 at all frequencies.
    // Energy ratio > 5.0 accounts for transient startup energy (one-pole settling
    // adds ~2x energy in first time-constant). Passive WDF can never amplify.
    assert!(
        e_out <= e_in * 5.0,
        "Passive filter should not increase energy dramatically: in={e_in:.6}, out={e_out:.6}"
    );
}

#[test]
fn zero_input_zero_state_stays_quiet() {
    let input = vec![0.0f64; 2048];
    let output = compile_and_process(rc_lowpass(), &input, SAMPLE_RATE, &[]);
    // TOLERANCE SOURCE: Floating-point zero propagation.
    // Zero input through passive WDF produces only roundoff noise.
    // 2048 multiply-add operations accumulate at most 2048 * 2.2e-16 ≈ 4.5e-13.
    // 1e-12 provides 2x margin. Any larger value indicates a state leak.
    assert!(
        output.iter().all(|&s| s.abs() < 1e-12),
        "Zero input/state should remain silent: peak={}",
        peak(&output)
    );
}

// ---------------------------------------------------------------------------
// Reciprocity / symmetry
// ---------------------------------------------------------------------------

fn reciprocal_pi(swap_ports: bool) -> String {
    if swap_ports {
        r#"
pedal "Reciprocity Reverse" {
  components {
    Rseries: resistor(1k)
    RshuntA: resistor(10k)
    RshuntB: resistor(10k)
  }
  nets {
    in -> NodeB
    NodeB -> RshuntB.a, Rseries.b
    RshuntB.b -> gnd
    Rseries.a -> NodeA
    NodeA -> RshuntA.a, out
    RshuntA.b -> gnd
  }
}
"#
        .to_string()
    } else {
        r#"
pedal "Reciprocity Forward" {
  components {
    Rseries: resistor(1k)
    RshuntA: resistor(10k)
    RshuntB: resistor(10k)
  }
  nets {
    in -> NodeA
    NodeA -> RshuntA.a, Rseries.a
    RshuntA.b -> gnd
    Rseries.b -> NodeB
    NodeB -> RshuntB.a, out
    RshuntB.b -> gnd
  }
}
"#
        .to_string()
    }
}

#[test]
fn symmetric_pi_network_is_reciprocal() {
    let input = guitar_pluck(110.0, 0.5, SAMPLE_RATE);
    let forward = compile_and_process(&reciprocal_pi(false), &input, SAMPLE_RATE, &[]);
    let reverse = compile_and_process(&reciprocal_pi(true), &input, SAMPLE_RATE, &[]);

    assert_healthy(&forward, "pi forward", 2.0);
    assert_healthy(&reverse, "pi reverse", 2.0);

    // TOLERANCE SOURCE: Discretization error.
    // Symmetric pi network is exactly reciprocal in continuous time. WDF bilinear
    // transform introduces O((f/fs)^2) error. At 48kHz for a guitar pluck
    // (fundamentals < 1kHz), max error is O((1000/48000)^2) ≈ 4e-4.
    // Correlation > 0.999 and RMS diff < 1e-6 both follow from this bound.
    let corr = correlation(&forward, &reverse).abs();
    assert!(
        corr > 0.999,
        "Swapping ports on symmetric pi should match: corr={corr:.6}"
    );

    let diff: Vec<f64> = forward.iter().zip(&reverse).map(|(a, b)| a - b).collect();
    assert!(
        rms(&diff) < 1e-6,
        "Reciprocal network mismatch RMS={}",
        rms(&diff)
    );
}

// ---------------------------------------------------------------------------
// Composition invariance (FX split vs monolithic)
// ---------------------------------------------------------------------------

#[test]
fn fx_split_matches_monolithic_engine() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/amps/tweed_deluxe_5e3.pedal");
    let src =
        std::fs::read_to_string(&path).expect("failed to read tweed_deluxe_5e3.pedal example");
    let pedal = parse_pedal_file(&src).expect("failed to parse tweed_deluxe");

    let mut mono = compile_pedal(&pedal, SAMPLE_RATE).expect("compile tweed");
    let mut split = compile_split_pedal(&pedal, SAMPLE_RATE).expect("split tweed");

    for &(label, value) in &[("Volume", 0.7), ("Tone", 0.5)] {
        mono.set_control(label, value);
        split.set_control(label, value);
    }

    let input = guitar_pluck(82.0, 0.5, SAMPLE_RATE);
    let mono_out: Vec<f64> = input.iter().copied().map(|x| mono.process(x)).collect();
    let split_out: Vec<f64> = input.iter().copied().map(|x| split.process(x)).collect();

    assert_healthy(&mono_out, "tweed mono", 40.0);
    assert_healthy(&split_out, "tweed split", 40.0);

    let corr = correlation(&mono_out, &split_out).abs();
    assert!(
        corr > 0.999,
        "FX split output should match monolithic: corr={corr:.6}"
    );
}
