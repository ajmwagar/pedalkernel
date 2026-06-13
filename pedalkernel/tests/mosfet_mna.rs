//! MOSFET support in multi-NL / general-MNA groups.
//!
//! Regression tests for `create_nl_device()` returning `None` for
//! `NonlinearKind::Mosfet`, which surfaced as the compile error
//! "Unsupported NL device kind in general MNA" (rigid/general.rs) for any
//! circuit where two or more MOSFETs land in one MNA group.
//!
//! The canonical victim is the Fulltone OCD: two 2N7000 clippers whose
//! drains share one node (`C3.b`), which collapses them into a single
//! multi-NL stage.

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::compiler::CompileOptions;

/// Minimal two-MOSFET fixture: an inverting op-amp stage whose feedback
/// resistor closes a loop around the output node, with two parallel 2N7000
/// clippers (shared drain node) hanging off that node. The feedback loop
/// makes the component non-series-parallel (rigid), so both MOSFETs land in
/// one general-MNA group — the same shape as the Fulltone OCD clipping
/// stage, isolated from its drive/tone/volume pots.
///
/// Notes on what reaches rigid/general.rs:
/// - Two parallel clippers WITHOUT the feedback loop stay series-parallel
///   and never hit the general MNA — they compile fine even without the
///   create_nl_device() Mosfet arm.
/// - With the feedback loop, blockwise decomposition may still split the
///   group into K-table blocks and bypass the rigid path, so the fixture
///   test compiles with `skip_blockwise` to force the monolithic R-type /
///   general-MNA fallback (the same path fulltone_ocd falls into when
///   blockwise finds only one K-table block).
const DUAL_MOSFET_CLIPPER: &str = r#"
pedal "Dual MOSFET Clipper" {
  components {
    C1: cap(100n)
    R1: resistor(1M)
    R2: resistor(10k)
    U1: opamp(tl072)
    R3: resistor(470)
    Drive: pot(500k, a)
    R4: resistor(1k)
    C2: cap(100p)
    M1: nmos(2n7000)
    M2: nmos(2n7000)
    R5: resistor(4.7k)
    C3: cap(47n)
  }
  nets {
    in -> C1.a
    C1.b -> R1.a, R2.a
    R1.b -> gnd
    R2.b -> U1.neg
    U1.pos -> gnd
    U1.out -> R3.a
    U1.neg -> R4.a
    R4.b -> Drive.a
    Drive.b -> R3.b
    U1.neg -> C2.a
    C2.b -> R3.b
    R3.b -> C3.a
    C3.b -> R5.a, M1.drain, M2.drain
    R5.b -> gnd
    M1.source -> gnd
    M2.source -> gnd
    C3.b -> out
  }
  controls {
    Drive.position -> "Drive" [0.0, 1.0] = 0.5
  }
}
"#;

fn assert_nonsilent_finite(output: &[f64], name: &str) {
    let pk = peak(output);
    assert!(
        output.iter().all(|s| s.is_finite()),
        "{name}: output contains non-finite samples"
    );
    assert!(pk > 1e-4, "{name}: output is silent (peak={pk:.3e})");
    assert!(pk < 50.0, "{name}: output blew up (peak={pk:.3e})");
}

/// Two parallel MOSFETs sharing a drain node must compile (one MNA group
/// with two single-port NL devices) and pass signal.
#[test]
fn dual_mosfet_clipper_compiles_and_runs() {
    let input = sine_at(220.0, 0.1, 0.5, SAMPLE_RATE);
    let options = CompileOptions {
        skip_blockwise: true,
        ..Default::default()
    };
    let output = compile_and_process_with_options(
        DUAL_MOSFET_CLIPPER,
        &input,
        SAMPLE_RATE,
        &[("Drive", 0.5)],
        options,
    );
    assert_nonsilent_finite(&output, "dual MOSFET clipper");
}

/// The Fulltone OCD (2x nmos 2n7000) must compile and produce non-silent,
/// finite output for a 0.1-amplitude 220 Hz sine.
#[test]
fn fulltone_ocd_compiles_and_produces_output() {
    let input = sine_at(220.0, 0.1, 0.5, SAMPLE_RATE);
    let output = compile_example_and_process(
        "fulltone_ocd.pedal",
        &input,
        SAMPLE_RATE,
        &[("Drive", 0.5), ("Tone", 0.5), ("Volume", 0.7)],
    );
    assert_nonsilent_finite(&output, "Fulltone OCD");
}
