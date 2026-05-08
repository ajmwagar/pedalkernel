//! Debug test: trace MFB LPF compilation flow groups and stage classification.

use super::spqr_build::compile_via_spqr;
use crate::dsl::parse_pedal_file;

const SR: f64 = 48_000.0;

/// MFB with pots — the broken version.
const MFB_POT: &str = r#"pedal "MFB Pot LPF" {
  supply 9V
  components {
    U1: opamp(tl072)
    Cutoff_R1: pot(100k, b)
    Cutoff_R3: pot(100k, b)
    C1: cap(10n)
    C2: cap(10n)
    Resonance: pot(100k, b)
    R_out: resistor(10k)
  }
  nets {
    in -> Cutoff_R1.a
    Cutoff_R1.b -> U1.neg
    U1.neg -> Cutoff_R3.a
    Cutoff_R3.b -> U1.out
    U1.neg -> C2.a
    C2.b -> U1.out
    Cutoff_R1.b -> C1.a
    C1.b -> gnd
    Cutoff_R1.b -> Resonance.a
    Resonance.b -> gnd
    U1.pos -> gnd
    U1.out -> R_out.a
    R_out.b -> out
  }
  controls {
    Cutoff_R1.position -> "Cutoff" [1.0, 0.1] = 0.5
    Cutoff_R3.position -> "Cutoff" [1.0, 0.1] = 0.5
    Resonance.position -> "Resonance" [1.0, 0.0] = 0.8
  }
}"#;

/// MFB with fixed R — the working version.
const MFB_FIXED: &str = r#"pedal "MFB Fixed LPF" {
  supply 9V
  components {
    U1: opamp(tl072)
    R1: resistor(47k)
    R3: resistor(47k)
    C1: cap(10n)
    C2: cap(10n)
    R2: resistor(100k)
    R_out: resistor(10k)
  }
  nets {
    in -> R1.a
    R1.b -> U1.neg
    U1.neg -> R3.a
    R3.b -> U1.out
    U1.neg -> C2.a
    C2.b -> U1.out
    R1.b -> C1.a
    C1.b -> gnd
    R1.b -> R2.a
    R2.b -> gnd
    U1.pos -> gnd
    U1.out -> R_out.a
    R_out.b -> out
  }
}"#;

#[test]
fn trace_mfb_pot_compilation() {
    eprintln!("\n=== MFB with Pots ===");
    let pedal = parse_pedal_file(MFB_POT).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();
    eprintln!("  Stages: {}", compiled.stages.len());
    eprintln!("  Controls: {}", compiled.controls.len());
    for (i, s) in compiled.stages.iter().enumerate() {
        let kind = match s {
            super::compiled::Stage::Wdf(w) => {
                let fb = if w.feedback_opamp.is_some() { "+OpAmp" } else { "" };
                format!("WDF{fb}")
            }
            super::compiled::Stage::Iir(iir) => format!("IIR label={:?}", iir.debug_label),
            super::compiled::Stage::BlackFeedback(bf) => format!("BF gain={:.2}", bf.gain()),
            super::compiled::Stage::MultiNl(_) => "MultiNL".to_string(),
            super::compiled::Stage::StateSpace(ss) => format!("SS label={:?}", ss.debug_label),
        };
        eprintln!("  [{i}] {kind}");
    }
}

#[test]
fn trace_mfb_fixed_compilation() {
    eprintln!("\n=== MFB with Fixed R ===");
    let pedal = parse_pedal_file(MFB_FIXED).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();
    eprintln!("  Stages: {}", compiled.stages.len());
    for (i, s) in compiled.stages.iter().enumerate() {
        let kind = match s {
            super::compiled::Stage::Wdf(w) => {
                let fb = if w.feedback_opamp.is_some() { "+OpAmp" } else { "" };
                format!("WDF{fb}")
            }
            super::compiled::Stage::Iir(iir) => format!("IIR label={:?}", iir.debug_label),
            super::compiled::Stage::BlackFeedback(bf) => format!("BF gain={:.2}", bf.gain()),
            super::compiled::Stage::MultiNl(_) => "MultiNL".to_string(),
            super::compiled::Stage::StateSpace(ss) => format!("SS label={:?}", ss.debug_label),
        };
        eprintln!("  [{i}] {kind}");
    }
}
