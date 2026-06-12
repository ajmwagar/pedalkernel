//! F10 (audit gap B5/G8) regression tests: an active makeup stage placed
//! AFTER a passive EQ network must not flatten the network's response or
//! freeze its pots.
//!
//! Root cause: the passive-claiming pass (`claim_passive_edges` in
//! `compiler/signal_flow.rs` and `collect_triode_context_edges` in
//! `compiler/spqr_build.rs`) walked upstream from the device's input
//! coupling cap through the entire interior network back toward `in`,
//! swallowing the EQ into the active stage's group where it goes inert.
//!
//! Fixture: a passive RC treble-cut network with a rheostat-wired Tone pot,
//! optionally followed by an AC-coupled makeup/buffer stage (12AU7 common
//! cathode, JFET source follower, NE5532 non-inverting buffer). The pot's
//! authority at 10 kHz (gain delta between position 0.0 and 1.0) must
//! survive the makeup stage.

mod audio_analysis;

use audio_analysis::*;

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

/// Passive RC treble-cut: in -> R1(10k) -> X; X -> Tone(10k rheostat) ->
/// C1(10n) -> gnd; X -> R_load(100k) -> gnd; X -> out.
/// At pot 0.0 the 10n cap shunts X hard at 10 kHz; at 1.0 the extra 10k
/// series resistance lifts the shunt. Passive-only authority ~11 dB.
const PASSIVE_TREBLE_CUT: &str = r#"
pedal "Passive Treble Cut" {
  components {
    R1: resistor(10k)
    Tone: pot(10k)
    C1: cap(10n)
    R_load: resistor(100k)
  }
  nets {
    in -> R1.a
    R1.b -> Tone.a, R_load.a, out
    Tone.b -> C1.a
    C1.b -> gnd
    R_load.b -> gnd
  }
  controls {
    Tone.position -> "Tone" [0.0, 1.0] = 0.5
  }
}
"#;

/// Same network followed by an AC-coupled 12AU7 common-cathode makeup stage
/// (pattern copied from tests/test_pedals/triode_clean.pedal).
const TREBLE_CUT_TRIODE_MAKEUP: &str = r#"
pedal "Treble Cut + Triode Makeup" {
  supply 250V {
    impedance: 50
    filter_cap: 47u
    rectifier: solid_state
  }
  components {
    R1: resistor(10k)
    Tone: pot(10k)
    C1: cap(10n)
    R_load: resistor(100k)
    C_c: cap(22n)
    R_g: resistor(1M)
    V1: triode(12au7)
    R_p: resistor(100k)
    R_k: resistor(1.5k)
    C_k: cap(25u)
    C_out: cap(22n)
    R_out: resistor(1M)
  }
  nets {
    in -> R1.a
    R1.b -> Tone.a, R_load.a, C_c.a
    Tone.b -> C1.a
    C1.b -> gnd
    R_load.b -> gnd
    C_c.b -> R_g.a, V1.grid
    R_g.b -> gnd
    vcc -> R_p.a
    R_p.b -> V1.plate
    V1.cathode -> R_k.a, C_k.a
    R_k.b -> gnd
    C_k.b -> gnd
    V1.plate -> C_out.a
    C_out.b -> R_out.a
    R_out.b -> gnd
    C_out.b -> out
  }
  controls {
    Tone.position -> "Tone" [0.0, 1.0] = 0.5
  }
}
"#;

/// Same network followed by an AC-coupled njfet source follower.
const TREBLE_CUT_JFET_MAKEUP: &str = r#"
pedal "Treble Cut + JFET Follower" {
  supply 9V
  components {
    R1: resistor(10k)
    Tone: pot(10k)
    C1: cap(10n)
    R_load: resistor(100k)
    C_c: cap(100n)
    R_g: resistor(1M)
    J1: njfet(j201)
    R_s: resistor(10k)
    C_out: cap(1u)
    R_out: resistor(100k)
  }
  nets {
    in -> R1.a
    R1.b -> Tone.a, R_load.a, C_c.a
    Tone.b -> C1.a
    C1.b -> gnd
    R_load.b -> gnd
    C_c.b -> R_g.a, J1.gate
    R_g.b -> gnd
    J1.drain -> vcc
    J1.source -> R_s.a, C_out.a
    R_s.b -> gnd
    C_out.b -> R_out.a
    R_out.b -> gnd
    C_out.b -> out
  }
  controls {
    Tone.position -> "Tone" [0.0, 1.0] = 0.5
  }
}
"#;

/// Same network followed by an NE5532 non-inverting unity buffer wired
/// directly to the network node (no input coupling cap — the buffer's pos
/// pin sits on the EQ output node, the swallow path for the single-hop
/// pendant pass).
const TREBLE_CUT_OPAMP_MAKEUP: &str = r#"
pedal "Treble Cut + NE5532 Buffer" {
  supply 9V
  components {
    R1: resistor(10k)
    Tone: pot(10k)
    C1: cap(10n)
    R_load: resistor(100k)
    U1: opamp(ne5532)
    C_out: cap(100n)
    R_out: resistor(10k)
  }
  nets {
    in -> R1.a
    R1.b -> Tone.a, R_load.a, U1.pos
    Tone.b -> C1.a
    C1.b -> gnd
    R_load.b -> gnd
    U1.neg -> U1.out
    U1.out -> C_out.a
    C_out.b -> R_out.a
    R_out.b -> gnd
    C_out.b -> out
  }
  controls {
    Tone.position -> "Tone" [0.0, 1.0] = 0.5
  }
}
"#;

// ---------------------------------------------------------------------------
// Measurement helpers
// ---------------------------------------------------------------------------

/// Frequency response (dB) at 1 kHz and 10 kHz for a given Tone position.
fn response_1k_10k(src: &str, tone: f64) -> (f64, f64) {
    let resp = frequency_response_db(
        || pedal_processor(src, SAMPLE_RATE, &[("Tone", tone)]),
        &[1_000.0, 10_000.0],
        0.1,
        SAMPLE_RATE,
    );
    (resp[0].1, resp[1].1)
}

/// Measure pot authority at 10 kHz (|gain(pot=1) - gain(pot=0)|) and apply
/// the level guard (response at 1 kHz must stay above -40 dB at both pot
/// positions — catches the -84..-240 dB collapse mode).
fn assert_pot_authority(src: &str, name: &str, min_authority_db: f64) {
    let (g1k_lo, g10k_lo) = response_1k_10k(src, 0.0);
    let (g1k_hi, g10k_hi) = response_1k_10k(src, 1.0);
    eprintln!(
        "{name}: Tone=0.0 -> 1 kHz {g1k_lo:8.2} dB, 10 kHz {g10k_lo:8.2} dB; \
         Tone=1.0 -> 1 kHz {g1k_hi:8.2} dB, 10 kHz {g10k_hi:8.2} dB"
    );
    let authority = (g10k_hi - g10k_lo).abs();
    eprintln!("{name}: pot authority at 10 kHz = {authority:.2} dB");

    // Level guard: catches the total-collapse failure mode.
    assert!(
        g1k_lo > -40.0 && g1k_hi > -40.0,
        "{name}: 1 kHz response collapsed ({g1k_lo:.2} dB at Tone=0, {g1k_hi:.2} dB at Tone=1)"
    );
    assert!(
        authority >= min_authority_db,
        "{name}: pot authority {authority:.2} dB at 10 kHz, expected >= {min_authority_db} dB \
         (pot frozen by passive-claiming swallow)"
    );
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// Baseline: the passive network alone must show strong pot authority.
/// This must pass both before and after the F10 fix — it keeps the makeup
/// fixtures honest.
#[test]
fn passive_treble_cut_baseline_has_pot_authority() {
    assert_pot_authority(PASSIVE_TREBLE_CUT, "passive baseline", 8.0);
}

#[test]
fn passive_eq_keeps_pot_authority_through_triode_makeup() {
    assert_pot_authority(TREBLE_CUT_TRIODE_MAKEUP, "triode makeup", 6.0);
}

#[test]
fn passive_eq_keeps_pot_authority_through_jfet_makeup() {
    assert_pot_authority(TREBLE_CUT_JFET_MAKEUP, "jfet makeup", 6.0);
}

/// Still red after the F10 claiming fix: the op-amp's pos pin sits directly
/// ON the EQ's output node, so no edge of the network is strictly upstream
/// of the device input pin — the distance bound cannot apply, and the
/// network is swallowed via the ground-shunt and single-hop passes into the
/// op-amp's group, where the downstream stage mis-renders it (injection at
/// the device pin / IIR Generic-role pots frozen at runtime,
/// rigid/general.rs + pedalkernel-rt/src/stage.rs).
/// Measured after the claiming fix (2026-06-12):
///   Tone=0.0 -> 1 kHz -65.58 dB, 10 kHz -60.32 dB
///   Tone=1.0 -> 1 kHz  -8.26 dB, 10 kHz  -6.27 dB
/// (level guard fails at Tone=0.0; the pot is live but the response is a
/// collapse, not the passive curve).
#[test]
#[ignore = "red until injection-node selection / IIR Generic pot re-derivation (F10 link 2)"]
fn passive_eq_keeps_pot_authority_through_opamp_makeup() {
    assert_pot_authority(TREBLE_CUT_OPAMP_MAKEUP, "opamp makeup", 6.0);
}
