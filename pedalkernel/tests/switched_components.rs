//! F7 (audit gaps B4/G7): switched passives must build WDF leaves.
//!
//! `cap_switched` / `inductor_switched` / `resistor_switched` historically did
//! not override `Component::make_leaf`, so any all-passive SPQR stage that
//! contained one collapsed to zero stages — a unity passthrough with dead
//! controls. These tests pin the fixed-position behavior (position 0 = first
//! listed value) against the equivalent fixed component.

mod audio_analysis;

use audio_analysis::*;

// ===========================================================================
// Test circuits
// ===========================================================================

/// RC lowpass with a switched shunt cap. Position 0 selects 33n:
/// fc = 1/(2π·(10k‖100k)·33n) ≈ 530 Hz, so gain(5 kHz) − gain(100 Hz)
/// ≈ −19.7 dB. A collapsed (passthrough) build measures ≈ 0 dB.
const SWITCHED_CAP_LOWPASS: &str = r#"
pedal "Switched Cap Lowpass" {
  components {
    R1: resistor(10k)
    C1: cap_switched(33n, 330n)
    R_load: resistor(100k)
  }
  nets {
    in -> R1.a
    R1.b -> C1.a, R_load.a, out
    C1.b -> gnd
    R_load.b -> gnd
  }
  controls {
    C1.position -> "Freq" [0.0, 1.0] = 0.0
  }
}
"#;

/// Same network with a fixed 33n cap — the control reference.
const FIXED_CAP_LOWPASS: &str = r#"
pedal "Fixed Cap Lowpass" {
  components {
    R1: resistor(10k)
    C1: cap(33n)
    R_load: resistor(100k)
  }
  nets {
    in -> R1.a
    R1.b -> C1.a, R_load.a, out
    C1.b -> gnd
    R_load.b -> gnd
  }
}
"#;

/// RL highpass with a switched shunt inductor. Position 0 selects 100m:
/// fc = R/(2πL) ≈ 1.59 kHz, so gain(100 Hz) − gain(5 kHz) ≈ −23.6 dB.
const SWITCHED_INDUCTOR_HIGHPASS: &str = r#"
pedal "Switched Inductor Highpass" {
  components {
    R1: resistor(1k)
    L1: inductor_switched(100m, 10m)
  }
  nets {
    in -> R1.a
    R1.b -> L1.a, out
    L1.b -> gnd
  }
  controls {
    L1.position -> "Freq" [0.0, 1.0] = 0.0
  }
}
"#;

/// Resistive divider with a switched series resistor. Position 0 selects
/// 10k, so the 10k/10k divider attenuates by exactly −6.02 dB.
const SWITCHED_RESISTOR_DIVIDER: &str = r#"
pedal "Switched Resistor Divider" {
  components {
    R_sel: resistor_switched(10k, 100k)
    R2: resistor(10k)
  }
  nets {
    in -> R_sel.a
    R_sel.b -> R2.a, out
    R2.b -> gnd
  }
  controls {
    R_sel.position -> "Range" [0.0, 1.0] = 0.0
  }
}
"#;

// ===========================================================================
// Phase 1: switched components contribute their first value to the WDF tree
// ===========================================================================

#[test]
fn switched_cap_network_is_not_passthrough() {
    let resp = frequency_response_db(
        || pedal_processor(SWITCHED_CAP_LOWPASS, SAMPLE_RATE, &[]),
        &[100.0, 5_000.0],
        0.1,
        SAMPLE_RATE,
    );
    let g_lo = resp[0].1;
    let g_hi = resp[1].1;
    eprintln!("switched cap LP: 100 Hz -> {g_lo:.3} dB, 5 kHz -> {g_hi:.3} dB");
    assert!(
        g_hi - g_lo <= -15.0,
        "switched-cap lowpass shows no rolloff (5 kHz − 100 Hz = {:.3} dB, \
         expected <= -15 dB) — stage collapsed to passthrough",
        g_hi - g_lo
    );
}

#[test]
fn switched_cap_matches_fixed_cap() {
    let freqs = [100.0, 5_000.0];
    let switched = frequency_response_db(
        || pedal_processor(SWITCHED_CAP_LOWPASS, SAMPLE_RATE, &[]),
        &freqs,
        0.1,
        SAMPLE_RATE,
    );
    let fixed = frequency_response_db(
        || pedal_processor(FIXED_CAP_LOWPASS, SAMPLE_RATE, &[]),
        &freqs,
        0.1,
        SAMPLE_RATE,
    );
    for (&(f, g_sw), &(_, g_fx)) in switched.iter().zip(fixed.iter()) {
        eprintln!("{f:7.1} Hz: switched {g_sw:8.3} dB, fixed {g_fx:8.3} dB");
        assert!(
            (g_sw - g_fx).abs() < 0.5,
            "switched cap deviates from fixed 33n cap at {f} Hz: \
             {g_sw:.3} dB vs {g_fx:.3} dB"
        );
    }
}

#[test]
fn switched_inductor_network_is_not_passthrough() {
    let resp = frequency_response_db(
        || pedal_processor(SWITCHED_INDUCTOR_HIGHPASS, SAMPLE_RATE, &[]),
        &[100.0, 5_000.0],
        0.1,
        SAMPLE_RATE,
    );
    let g_lo = resp[0].1;
    let g_hi = resp[1].1;
    eprintln!("switched inductor HP: 100 Hz -> {g_lo:.3} dB, 5 kHz -> {g_hi:.3} dB");
    assert!(
        g_lo - g_hi <= -15.0,
        "switched-inductor highpass shows no rolloff (100 Hz − 5 kHz = {:.3} dB, \
         expected <= -15 dB) — stage collapsed to passthrough",
        g_lo - g_hi
    );
}

#[test]
fn switched_resistor_divider_attenuates() {
    let resp = frequency_response_db(
        || pedal_processor(SWITCHED_RESISTOR_DIVIDER, SAMPLE_RATE, &[]),
        &[1_000.0],
        0.1,
        SAMPLE_RATE,
    );
    let g = resp[0].1;
    eprintln!("switched resistor divider: 1 kHz -> {g:.3} dB");
    assert!(
        (g + 6.02).abs() < 0.5,
        "10k/10k divider with switched series resistor measures {g:.3} dB, \
         expected -6.02 ± 0.5 dB"
    );
}

// ===========================================================================
// Phase 2 (live switching) — intentionally red until implemented
// ===========================================================================

#[test]
#[ignore = "red until F7 phase 2 (live switching)"]
fn switched_cap_position_changes_response() {
    // Position 0 → 33n (fc ≈ 530 Hz), position 1 → 330n (fc ≈ 53 Hz).
    // At 1 kHz the analytic gap is ~18 dB; require at least 6 dB.
    let resp_0 = frequency_response_db(
        || pedal_processor(SWITCHED_CAP_LOWPASS, SAMPLE_RATE, &[("Freq", 0.0)]),
        &[1_000.0],
        0.1,
        SAMPLE_RATE,
    );
    let resp_1 = frequency_response_db(
        || pedal_processor(SWITCHED_CAP_LOWPASS, SAMPLE_RATE, &[("Freq", 1.0)]),
        &[1_000.0],
        0.1,
        SAMPLE_RATE,
    );
    let (g0, g1) = (resp_0[0].1, resp_1[0].1);
    eprintln!("switched cap @1 kHz: position 0 -> {g0:.3} dB, position 1 -> {g1:.3} dB");
    assert!(
        (g0 - g1).abs() >= 6.0,
        "Freq control is dead: position 0 = {g0:.3} dB, position 1 = {g1:.3} dB \
         at 1 kHz (expected >= 6 dB apart)"
    );
}
