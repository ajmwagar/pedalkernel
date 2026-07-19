//! Real-world before/after fixture for pedalkernel-pmv1.
//!
//! State-variable (KHN) filter loop: three op-amps (U_hp, U_bp, U_lp) form
//! one signal-flow group. LPF and HPF variants of this circuit are
//! IDENTICAL except for which op-amp's `.out` is wired to the pedal's `out`
//! net (U_lp.out for LPF, U_hp.out for HPF — see BiValve
//! `bivalve-core/build.rs` SVF_LPF_SOURCE / SVF_HPF_SOURCE, pre-workaround).
//!
//! `find_output_extract_node`'s final fallback (general.rs:929) used to pick
//! the FIRST nullor in component-declaration order regardless of which one
//! is actually wired to `out`, so both variants compiled to the same tap and
//! produced bit-identical output. This test compiles both variants directly
//! (not the WDF-safe passive workaround currently in build.rs) and asserts
//! their outputs differ.

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;

const SAMPLE_RATE: f64 = 48_000.0;

/// Real 3-opamp KHN state-variable loop, LPF tap (U_lp.out -> out).
/// U_hp declared first, U_lp declared last: this is the case the buggy
/// fallback got wrong (it always returned U_hp's node).
const SVF_LPF_SOURCE: &str = r#"pedal "SVF LPF" {
  supply 9V
  components {
    U_hp: opamp(tl072)
    U_bp: opamp(tl072)
    U_lp: opamp(tl072)
    R_in: resistor(47k)
    R_hp_fb: resistor(47k)
    R_q: resistor(150k)
    CutoffA: pot(100k, b)
    CutoffB: pot(100k, b)
    C_bp: cap(4.7n)
    C_lp: cap(4.7n)
    Resonance: pot(100k, b)
    R_out: resistor(10k)
  }
  nets {
    in -> R_in.a
    R_in.b -> U_hp.neg
    U_hp.neg -> R_hp_fb.a
    R_hp_fb.b -> U_hp.out
    U_bp.out -> R_q.a
    R_q.b -> U_hp.neg
    U_bp.out -> Resonance.a
    Resonance.b -> U_hp.neg
    U_hp.pos -> gnd

    U_hp.out -> CutoffA.a
    CutoffA.b -> U_bp.neg
    U_bp.neg -> C_bp.a
    C_bp.b -> U_bp.out
    U_bp.pos -> gnd

    U_bp.out -> CutoffB.a
    CutoffB.b -> U_lp.neg
    U_lp.neg -> C_lp.a
    C_lp.b -> U_lp.out
    U_lp.pos -> gnd

    U_lp.out -> R_out.a
    R_out.b -> out
  }
  controls {
    CutoffA.position -> "Cutoff" [1.0, 0.08] = 0.5
    CutoffB.position -> "Cutoff" [1.0, 0.08] = 0.5
    Resonance.position -> "Resonance" [1.0, 0.35] = 0.0
  }
}"#;

/// Same loop, HPF tap (U_hp.out -> out instead of U_lp.out).
const SVF_HPF_SOURCE: &str = r#"pedal "SVF HPF" {
  supply 9V
  components {
    U_hp: opamp(tl072)
    U_bp: opamp(tl072)
    U_lp: opamp(tl072)
    R_in: resistor(47k)
    R_hp_fb: resistor(47k)
    R_q: resistor(150k)
    CutoffA: pot(100k, b)
    CutoffB: pot(100k, b)
    C_bp: cap(4.7n)
    C_lp: cap(4.7n)
    Resonance: pot(100k, b)
    R_out: resistor(10k)
  }
  nets {
    in -> R_in.a
    R_in.b -> U_hp.neg
    U_hp.neg -> R_hp_fb.a
    R_hp_fb.b -> U_hp.out
    U_bp.out -> R_q.a
    R_q.b -> U_hp.neg
    U_bp.out -> Resonance.a
    Resonance.b -> U_hp.neg
    U_hp.pos -> gnd

    U_hp.out -> CutoffA.a
    CutoffA.b -> U_bp.neg
    U_bp.neg -> C_bp.a
    C_bp.b -> U_bp.out
    U_bp.pos -> gnd

    U_bp.out -> CutoffB.a
    CutoffB.b -> U_lp.neg
    U_lp.neg -> C_lp.a
    C_lp.b -> U_lp.out
    U_lp.pos -> gnd

    U_hp.out -> R_out.a
    R_out.b -> out
  }
  controls {
    CutoffA.position -> "Cutoff" [1.0, 0.08] = 0.5
    CutoffB.position -> "Cutoff" [1.0, 0.08] = 0.5
    Resonance.position -> "Resonance" [1.0, 0.35] = 0.0
  }
}"#;

fn run(source: &str, samples: usize) -> Vec<f64> {
    let def = parse_pedal_file(source).expect("parse");
    let mut compiled = compile_pedal(&def, SAMPLE_RATE).expect("compile");

    // Settle DC operating point.
    for _ in 0..2000 {
        compiled.process(0.0);
    }

    let mut out = Vec::with_capacity(samples);
    for i in 0..samples {
        let input = 0.1 * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / SAMPLE_RATE).sin();
        out.push(compiled.process(input));
    }
    out
}

/// pedalkernel-pmv1 before-state: on unmodified source, LPF and HPF taps of
/// the same 3-opamp SVF loop compile to the same extracted output node
/// (first nullor in declaration order = U_hp), so their outputs are
/// bit-identical even though the .pedal sources wire `out` to different
/// op-amps (U_lp vs U_hp).
#[test]
fn svf_lpf_and_hpf_differ() {
    let lpf = run(SVF_LPF_SOURCE, 480);
    let hpf = run(SVF_HPF_SOURCE, 480);

    let max_abs_diff = lpf
        .iter()
        .zip(hpf.iter())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0_f64, f64::max);

    eprintln!("[svf_output_tap] max|LPF-HPF| = {max_abs_diff:.6e}");
    assert!(
        max_abs_diff > 1e-6,
        "SVF LPF and HPF taps should differ (LPF taps U_lp.out, HPF taps \
         U_hp.out) but were bit-identical (max diff={max_abs_diff:.3e}). \
         This is pedalkernel-pmv1: find_output_extract_node's fallback \
         picked the first nullor (U_hp) regardless of which one 'out' is \
         wired to."
    );
}
