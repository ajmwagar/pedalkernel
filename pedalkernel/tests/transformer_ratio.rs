//! Transformer turns-ratio fidelity tests (GAP F).
//!
//! An ideal transformer with turns ratio n = N_primary : N_secondary
//! transfers voltage as Vsec = Vpri / n. A 10:1 step-down (n = 10) must
//! attenuate by ~20 dB; a 1:4 step-up (n = 0.25) must boost by ~12 dB.
//!
//! These fixtures mirror the SPICE validation circuits
//! (pedalkernel-validate/circuits/reactive/transformer_step{down,up}.pedal):
//! input drives the primary (T1.a), output is taken from the secondary
//! (T1.c), loaded by a resistor. Well above the magnetizing corner the
//! measured Vsec/Vpri converges on the ideal turns ratio.
//!
//! Regression guard: step-down (was ~+19 dB off vs SPICE) AND step-up
//! (the reference that already passed) must both land on the ideal ratio.

mod audio_analysis;

use audio_analysis::*;

/// 10:1 step-down, 2H primary, 1k secondary load. Ideal Vsec/Vpri = 0.1.
const STEP_DOWN: &str = r#"
pedal "Transformer Step-Down" {
  components {
    T1: transformer(10:1, 2H)
    R_load: resistor(1k)
  }
  nets {
    in -> T1.a
    T1.b -> gnd
    T1.c -> out, R_load.a
    T1.d -> gnd
    R_load.b -> gnd
  }
}
"#;

/// 1:4 step-up, 500mH primary, 10k secondary load. Ideal Vsec/Vpri = 4.0.
const STEP_UP: &str = r#"
pedal "Transformer Step-Up" {
  components {
    T1: transformer(1:4, 500m)
    R_load: resistor(10k)
  }
  nets {
    in -> T1.a
    T1.b -> gnd
    T1.c -> out, R_load.a
    T1.d -> gnd
    R_load.b -> gnd
  }
}
"#;

/// Measure the steady-state Vsec/Vpri ratio at `freq_hz`.
///
/// Drives a unit-amplitude sine, discards the leading transient, and returns
/// the ratio of output RMS to input RMS over the settled tail.
fn measured_ratio(src: &str, freq_hz: f64) -> f64 {
    let dur = 0.5;
    let input = sine_amp(freq_hz, 1.0, dur, SAMPLE_RATE);
    let output = compile_and_process(src, &input, SAMPLE_RATE, &[]);
    // Discard first 60% as settling transient; measure the steady tail.
    let start = (input.len() as f64 * 0.6) as usize;
    let in_rms = rms(&input[start..]);
    let out_rms = rms(&output[start..]);
    out_rms / in_rms
}

/// Unit-amplitude sine (audio_analysis::sine is fixed at amplitude 0.5).
fn sine_amp(freq_hz: f64, amp: f64, dur: f64, sr: f64) -> Vec<f64> {
    let n = (dur * sr) as usize;
    (0..n)
        .map(|i| amp * (2.0 * std::f64::consts::PI * freq_hz * i as f64 / sr).sin())
        .collect()
}

#[test]
fn step_down_matches_ideal_turns_ratio() {
    // n = 10 → ideal Vsec/Vpri = 0.1 (−20 dB). The bug produced ~0.9
    // (+19 dB vs SPICE), i.e. a near step-UP instead of step-down.
    let ideal = 0.1;
    for &f in &[300.0, 1_000.0, 3_000.0] {
        let ratio = measured_ratio(STEP_DOWN, f);
        let db_err = 20.0 * (ratio / ideal).log10();
        eprintln!("step-down @ {f} Hz: ratio {ratio:.4} (ideal {ideal}), err {db_err:.2} dB");
        assert!(
            db_err.abs() < 1.5,
            "step-down @ {f} Hz: ratio {ratio:.4}, ideal {ideal}, error {db_err:.2} dB exceeds 1.5 dB"
        );
    }
}

/// LA-2A output transformer in isolation: 4:1 step-down, 10H primary,
/// 600 Ohm line load. Ideal Vsec/Vpri = 0.25 (−12 dB). Proves the engine
/// transformer scaling is correct for the exact element the LA-2A uses,
/// independent of the rest of that (separately-broken) chain.
const LA2A_OUTPUT_XFMR: &str = r#"
pedal "LA2A Output Transformer" {
  components {
    T1: transformer(4:1, 10H)
    R_load: resistor(600)
  }
  nets {
    in -> T1.a
    T1.b -> gnd
    T1.c -> out, R_load.a
    T1.d -> gnd
    R_load.b -> gnd
  }
}
"#;

#[test]
fn la2a_output_transformer_steps_down_by_four() {
    // 4:1 → ideal Vsec/Vpri = 0.25. If this matched the old +19 dB bug it
    // would read ~2.2 (a step-UP); the engine must produce ~0.25.
    let ideal = 0.25;
    for &f in &[300.0, 1_000.0, 3_000.0] {
        let ratio = measured_ratio(LA2A_OUTPUT_XFMR, f);
        let db_err = 20.0 * (ratio / ideal).log10();
        eprintln!("la2a OT @ {f} Hz: ratio {ratio:.4} (ideal {ideal}), err {db_err:.2} dB");
        assert!(
            db_err.abs() < 1.5,
            "la2a OT @ {f} Hz: ratio {ratio:.4}, ideal {ideal}, error {db_err:.2} dB exceeds 1.5 dB"
        );
    }
}

#[test]
fn step_up_matches_ideal_turns_ratio() {
    // n = 0.25 → ideal Vsec/Vpri = 4.0 (+12 dB). Must-not-regress guard:
    // this path already passed SPICE validation.
    let ideal = 4.0;
    for &f in &[300.0, 1_000.0, 3_000.0] {
        let ratio = measured_ratio(STEP_UP, f);
        let db_err = 20.0 * (ratio / ideal).log10();
        eprintln!("step-up @ {f} Hz: ratio {ratio:.4} (ideal {ideal}), err {db_err:.2} dB");
        assert!(
            db_err.abs() < 1.5,
            "step-up @ {f} Hz: ratio {ratio:.4}, ideal {ideal}, error {db_err:.2} dB exceeds 1.5 dB"
        );
    }
}
