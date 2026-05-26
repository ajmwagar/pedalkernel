//! Debug test: trace MFB LPF compilation flow groups and stage classification.

use super::spqr_build::compile_via_spqr;
use crate::dsl::parse_pedal_file;
use crate::PedalProcessor;
use std::f64::consts::PI;

const SR: f64 = 48_000.0;

const MFB_ACTIVE_VREF: &str = r#"pedal "MFB Active Vref LPF" {
  supply 9V
  components {
    U1: opamp(tl072)
    Rb1: resistor(100k)
    Rb2: resistor(100k)
    C_vref: cap(47u, electrolytic)
    R_in: resistor(10k)
    Cutoff_R1: pot(100k, b)
    Cutoff_R3: pot(100k, b)
    C1: cap(10n)
    C2: cap(10n)
    R_q_floor: resistor(10k)
    Resonance: pot(100k, b)
    R_out: resistor(10k)
  }
  nets {
    vcc -> Rb1.a
    Rb1.b -> Rb2.a, C_vref.a, vref
    Rb2.b -> gnd
    C_vref.b -> gnd
    vref -> U1.pos
    in -> R_in.a
    R_in.b -> Cutoff_R1.a
    Cutoff_R1.b -> C1.a
    C1.b -> gnd
    Cutoff_R1.b -> R_q_floor.a
    R_q_floor.b -> Resonance.a
    Resonance.b -> U1.neg
    U1.neg -> Cutoff_R3.a
    Cutoff_R3.b -> U1.out
    U1.neg -> C2.a
    C2.b -> U1.out
    U1.out -> R_out.a
    R_out.b -> out
  }
  controls {
    Cutoff_R1.position -> "Cutoff" [1.0, 0.05] = 0.5
    Cutoff_R3.position -> "Cutoff" [1.0, 0.05] = 0.5
    Resonance.position -> "Resonance" [1.0, 0.05] = 0.0
  }
}"#;

fn iir_stage_is_stable(stage: &super::compiled::Stage) -> bool {
    let super::compiled::Stage::Iir(iir) = stage else {
        return true;
    };
    let b = &iir.iir.b_coeffs;
    let a = &iir.iir.a_coeffs;
    if b.is_empty()
        || a.len() < 2
        || !b.iter().all(|v| v.is_finite())
        || !a.iter().all(|v| v.is_finite())
    {
        return false;
    }
    if a.len() >= 3 {
        let a1 = a[1];
        let a2 = a[2];
        a2.abs() < 1.0 && 1.0 + a1 + a2 > 0.0 && 1.0 - a1 + a2 > 0.0
    } else {
        a[1].abs() < 1.0
    }
}

#[test]
fn active_mfb_with_decoupled_vref_emits_finite_controlled_linear_stage() {
    let pedal = parse_pedal_file(MFB_ACTIVE_VREF).unwrap();
    let mut compiled = compile_via_spqr(&pedal, SR).unwrap();

    assert!(
        compiled.stages.iter().any(|stage| matches!(
            stage,
            super::compiled::Stage::Iir(_) | super::compiled::Stage::StateSpace(_)
        )),
        "active MFB should compile to a controlled active-linear stage, got {:?}",
        compiled.stages
    );

    for cutoff in [0.05, 0.5, 1.0] {
        for resonance in [0.0, 0.5, 1.0] {
            compiled.set_control("Cutoff", cutoff);
            compiled.set_control("Resonance", resonance);

            for (idx, stage) in compiled.stages.iter().enumerate() {
                assert!(
                    iir_stage_is_stable(stage),
                    "stage {idx} should keep finite stable IIR coefficients after cutoff={cutoff}, resonance={resonance}: {stage:?}"
                );
            }

            for i in 0..4096 {
                let input = if i == 0 {
                    0.5
                } else {
                    (2.0 * PI * 1_000.0 * i as f64 / SR).sin() * 0.5
                };
                let out = compiled.process(input);
                assert!(
                    out.is_finite(),
                    "active MFB produced non-finite output at cutoff={cutoff}, resonance={resonance}"
                );
            }
        }
    }
}

fn gain_db_at(
    compiled: &mut impl PedalProcessor,
    freq_hz: f64,
    cutoff: f64,
    resonance: f64,
) -> f64 {
    compiled.set_control("Cutoff", cutoff);
    compiled.set_control("Resonance", resonance);
    for _ in 0..2048 {
        compiled.process(0.0);
    }

    let mut peak = 0.0f64;
    for i in 0..16_384 {
        let input = (2.0 * PI * freq_hz * i as f64 / SR).sin();
        let out = compiled.process(input);
        if i >= 8192 {
            peak = peak.max(out.abs());
        }
    }
    assert!(peak.is_finite(), "active MFB response became non-finite");
    20.0 * peak.max(1e-12).log10()
}

#[test]
fn active_mfb_with_decoupled_vref_cutoff_moves_response() {
    let pedal = parse_pedal_file(MFB_ACTIVE_VREF).unwrap();
    let low_cutoff = gain_db_at(
        &mut compile_via_spqr(&pedal, SR).unwrap(),
        5_000.0,
        0.1,
        0.0,
    );
    let high_cutoff = gain_db_at(
        &mut compile_via_spqr(&pedal, SR).unwrap(),
        5_000.0,
        0.9,
        0.0,
    );

    assert!(
        high_cutoff > low_cutoff + 12.0,
        "active MFB cutoff should open at 5 kHz: low={low_cutoff:.2} dB high={high_cutoff:.2} dB"
    );
}

#[test]
fn active_mfb_with_decoupled_vref_resonance_moves_response() {
    let pedal = parse_pedal_file(MFB_ACTIVE_VREF).unwrap();
    let low_res = gain_db_at(
        &mut compile_via_spqr(&pedal, SR).unwrap(),
        1_000.0,
        0.5,
        0.0,
    );
    let high_res = gain_db_at(
        &mut compile_via_spqr(&pedal, SR).unwrap(),
        1_000.0,
        0.5,
        0.9,
    );

    assert!(
        (high_res - low_res).abs() > 3.0,
        "active MFB resonance should alter response near cutoff: res=0 {low_res:.2} dB, res=0.9 {high_res:.2} dB"
    );
}

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
            super::compiled::Stage::Blockwise(bk) => {
                format!("BKM dist={}", bk.signal_flow_distance)
            }
            super::compiled::Stage::SerialDelayedFeedback(s) => {
                format!(
                    "SerialFB rungs={} dist={}",
                    s.stages.len(),
                    s.signal_flow_distance
                )
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
            super::compiled::Stage::Blockwise(bk) => {
                format!("BKM dist={}", bk.signal_flow_distance)
            }
            super::compiled::Stage::SerialDelayedFeedback(s) => {
                format!(
                    "SerialFB rungs={} dist={}",
                    s.stages.len(),
                    s.signal_flow_distance
                )
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
            super::compiled::Stage::Blockwise(bk) => {
                format!("BKM dist={}", bk.signal_flow_distance)
            }
            super::compiled::Stage::SerialDelayedFeedback(s) => {
                format!(
                    "SerialFB rungs={} dist={}",
                    s.stages.len(),
                    s.signal_flow_distance
                )
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
            super::compiled::Stage::Blockwise(bk) => {
                format!("BKM dist={}", bk.signal_flow_distance)
            }
            super::compiled::Stage::SerialDelayedFeedback(s) => {
                format!(
                    "SerialFB rungs={} dist={}",
                    s.stages.len(),
                    s.signal_flow_distance
                )
            }
        };
        eprintln!("  [{i}] {kind}");
    }
}
