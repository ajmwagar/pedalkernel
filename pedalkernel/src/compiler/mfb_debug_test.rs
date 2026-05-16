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
                let fb = if w.feedback_opamp.is_some() {
                    "+OpAmp"
                } else {
                    ""
                };
                format!("WDF{fb}")
            }
            super::compiled::Stage::Iir(iir) => format!("IIR label={:?}", iir.debug_label),
            super::compiled::Stage::BlackFeedback(bf) => format!("BF gain={:.2}", bf.gain()),
            super::compiled::Stage::MultiNl(_) => "MultiNL".to_string(),
            super::compiled::Stage::StateSpace(ss) => format!("SS label={:?}", ss.debug_label),
            super::compiled::Stage::BlockwiseKMethod(bk) => {
                format!("BKM dist={}", bk.signal_flow_distance)
            }
        };
        eprintln!("  [{i}] {kind}");
    }
}

/// MFB HPF with pots — matches the current build.rs circuit.
const MFB_HPF_POT: &str = r#"pedal "MFB HPF Pot" {
  supply 9V
  components {
    U1: opamp(tl072)
    C1: cap(10n)
    C2: cap(10n)
    Cutoff_R1: pot(100k, b)
    Cutoff_R2: pot(100k, b)
    Resonance: pot(100k, b)
    R_out: resistor(10k)
  }
  nets {
    in -> C1.a
    C1.b -> C2.a
    C2.b -> U1.out
    C1.b -> Cutoff_R1.a
    Cutoff_R1.b -> gnd
    C1.b -> U1.neg
    U1.neg -> Cutoff_R2.a
    Cutoff_R2.b -> U1.out
    C1.b -> Resonance.a
    Resonance.b -> gnd
    U1.pos -> gnd
    U1.out -> R_out.a
    R_out.b -> out
  }
  controls {
    Cutoff_R1.position -> "Cutoff" [0.0, 1.0] = 0.5
    Cutoff_R2.position -> "Cutoff" [0.0, 1.0] = 0.5
    Resonance.position -> "Resonance" [1.0, 0.0] = 0.8
  }
}"#;

#[test]
fn trace_mfb_hpf_compilation() {
    eprintln!("\n=== MFB HPF with Pots ===");
    let pedal = parse_pedal_file(MFB_HPF_POT).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();
    eprintln!("  Stages: {}", compiled.stages.len());
    eprintln!("  Controls: {}", compiled.controls.len());
    for (i, s) in compiled.stages.iter().enumerate() {
        let kind = match s {
            super::compiled::Stage::Wdf(w) => {
                let fb = if w.feedback_opamp.is_some() {
                    "+OpAmp"
                } else {
                    ""
                };
                format!("WDF{fb}")
            }
            super::compiled::Stage::Iir(iir) => format!("IIR label={:?}", iir.debug_label),
            super::compiled::Stage::BlackFeedback(bf) => format!("BF gain={:.2}", bf.gain()),
            super::compiled::Stage::MultiNl(_) => "MultiNL".to_string(),
            super::compiled::Stage::StateSpace(ss) => format!("SS label={:?}", ss.debug_label),
            super::compiled::Stage::BlockwiseKMethod(bk) => {
                format!("BKM dist={}", bk.signal_flow_distance)
            }
        };
        eprintln!("  [{i}] {kind}");
    }

    // Frequency test
    use crate::PedalProcessor;
    let mut proc: Box<dyn PedalProcessor> = Box::new(compiled);
    for &freq in &[100.0, 1000.0, 5000.0, 10000.0] {
        proc.reset();
        let mut peak = 0.0f64;
        for i in 0..9600 {
            let input = (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let out = proc.process(input);
            if i >= 4800 {
                peak = peak.max(out.abs());
            }
        }
        let db = 20.0 * peak.max(1e-12).log10();
        eprintln!("  {freq:.0}Hz: {db:+.1}dB");
    }
}

/// MFB HPF: C1 in series with R_dc (10M) in parallel for DC bias path.
/// R_dc >> |Z_C1| at audio (C1=10nF at 20Hz: 796k << 10M).
const MFB_HPF_FIXED: &str = r#"pedal "MFB HPF Fixed" {
  supply 9V
  components {
    U1: opamp(tl072)
    C1: cap(10n)
    C2: cap(10n)
    R_dc: resistor(10M)
    R1: resistor(47k)
    R2: resistor(47k)
    R_q: resistor(100k)
    R_out: resistor(10k)
  }
  nets {
    in -> C1.a
    in -> R_dc.a
    R_dc.b -> C1.b
    C1.b -> C2.a
    C2.b -> U1.out
    C1.b -> R1.a
    R1.b -> gnd
    C1.b -> U1.neg
    U1.neg -> R2.a
    R2.b -> U1.out
    C1.b -> R_q.a
    R_q.b -> gnd
    U1.pos -> gnd
    U1.out -> R_out.a
    R_out.b -> out
  }
}"#;

#[test]
fn trace_mfb_hpf_fixed_compilation() {
    eprintln!("\n=== MFB HPF with Fixed R ===");
    let pedal = parse_pedal_file(MFB_HPF_FIXED).unwrap();
    let compiled = compile_via_spqr(&pedal, SR).unwrap();
    eprintln!("  Stages: {}", compiled.stages.len());
    for (i, s) in compiled.stages.iter().enumerate() {
        let kind = match s {
            super::compiled::Stage::Wdf(w) => {
                let fb = if w.feedback_opamp.is_some() {
                    "+OpAmp"
                } else {
                    ""
                };
                format!("WDF{fb}")
            }
            super::compiled::Stage::Iir(iir) => format!("IIR label={:?}", iir.debug_label),
            super::compiled::Stage::BlackFeedback(bf) => format!("BF gain={:.2}", bf.gain()),
            super::compiled::Stage::MultiNl(_) => "MultiNL".to_string(),
            super::compiled::Stage::StateSpace(ss) => format!("SS label={:?}", ss.debug_label),
            super::compiled::Stage::BlockwiseKMethod(bk) => {
                format!("BKM dist={}", bk.signal_flow_distance)
            }
        };
        eprintln!("  [{i}] {kind}");
    }
    use crate::PedalProcessor;
    let mut proc: Box<dyn PedalProcessor> = Box::new(compiled);
    for &freq in &[100.0, 1000.0, 5000.0, 10000.0] {
        proc.reset();
        let mut peak = 0.0f64;
        for i in 0..9600 {
            let input = (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let out = proc.process(input);
            if i >= 4800 {
                peak = peak.max(out.abs());
            }
        }
        let db = 20.0 * peak.max(1e-12).log10();
        eprintln!("  {freq:.0}Hz: {db:+.1}dB");
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
                let fb = if w.feedback_opamp.is_some() {
                    "+OpAmp"
                } else {
                    ""
                };
                format!("WDF{fb}")
            }
            super::compiled::Stage::Iir(iir) => format!("IIR label={:?}", iir.debug_label),
            super::compiled::Stage::BlackFeedback(bf) => format!("BF gain={:.2}", bf.gain()),
            super::compiled::Stage::MultiNl(_) => "MultiNL".to_string(),
            super::compiled::Stage::StateSpace(ss) => format!("SS label={:?}", ss.debug_label),
            super::compiled::Stage::BlockwiseKMethod(bk) => {
                format!("BKM dist={}", bk.signal_flow_distance)
            }
        };
        eprintln!("  [{i}] {kind}");
    }
}
