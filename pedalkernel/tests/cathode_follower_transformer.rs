//! Mask-7 regression: a cathode follower driving an output transformer.
//!
//! RCA (`reports/signal-routing-formation-layer-2026-06-13.md` §1): a
//! low-output-impedance source (a 12BH7 cathode follower) feeding a downstream
//! output transformer collapses to numerical zero, while a high-impedance
//! (plate-driven) source into the SAME transformer is healthy. The difference
//! is ONLY the source impedance.
//!
//! Root cause is in group FORMATION (`compiler/signal_flow.rs`): for a cathode
//! follower the active device's OUTPUT node IS its cathode bias node, so
//! `claim_passive_edges` swallows the FORWARD-SIGNAL branch leaving that node
//! (coupling cap -> transformer -> load) into the device's nonlinear group.
//! The MultiNL builder then discards the transformer -> the realized stage's
//! `output_port` reads a dead node -> silence. The plate-driven topology
//! escapes only because the plate output node != the cathode bias node, so the
//! forward branch splits cleanly into its own `PassiveRType` transformer stage.
//!
//! This file isolates mask 7 from the over-fusion mask (mask 8): there is NO
//! feedback / detector tap here, just a forward chain.
//!
//! Run (matches the RCA reproduction conditions):
//!   cargo test -p pedalkernel --no-default-features \
//!     --test cathode_follower_transformer -- --nocapture

mod audio_analysis;

use audio_analysis::*;

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;

const PROBE_HZ: f64 = 1_000.0;

/// 12BH7 cathode follower -> coupling cap -> 4:1 transformer -> 600 Ω load.
/// The cathode node is BOTH the device operating point (R_k3 to gnd) and the
/// forward-signal source (C_c3 -> T_out -> R_load -> out). This is the mask-7
/// trigger: shared output/bias node.
const CF_INTO_TRANSFORMER: &str = r#"
pedal "CF into transformer" {
  supply 250V {
    impedance: 50
    filter_cap: 47u
    rectifier: solid_state
  }
  components {
    C1: cap(100n)
    R_g3: resistor(1M)
    V3: triode(12bh7)
    R_k3: resistor(10k)
    C_c3: cap(10u, electrolytic)
    T_out: transformer(4:1, 10H)
    R_load: resistor(600)
  }
  nets {
    in -> C1.a
    C1.b -> V3.grid, R_g3.a
    R_g3.b -> gnd
    vcc -> V3.plate
    V3.cathode -> R_k3.a, C_c3.a
    R_k3.b -> gnd
    C_c3.b -> T_out.a
    T_out.b -> R_load.a, out
    R_load.b -> gnd
  }
}
"#;

/// Same 12BH7 cathode follower driving the SAME 600 Ω load directly (no
/// transformer). This is healthy regardless of the bug — it is the level
/// reference that proves the CF stage itself works.
const CF_INTO_LOAD: &str = r#"
pedal "CF into load" {
  supply 250V {
    impedance: 50
    filter_cap: 47u
    rectifier: solid_state
  }
  components {
    C1: cap(100n)
    R_g3: resistor(1M)
    V3: triode(12bh7)
    R_k3: resistor(10k)
    C_c3: cap(10u, electrolytic)
    R_load: resistor(600)
  }
  nets {
    in -> C1.a
    C1.b -> V3.grid, R_g3.a
    R_g3.b -> gnd
    vcc -> V3.plate
    V3.cathode -> R_k3.a, C_c3.a
    R_k3.b -> gnd
    C_c3.b -> R_load.a, out
    R_load.b -> gnd
  }
}
"#;

/// Count `[Stage N` entries in the debug dump.
fn stage_count(dump: &str) -> usize {
    dump.matches("[Stage ").count()
}

/// Forward gain at PROBE_HZ in dB.
fn forward_gain_db(src: &str) -> f64 {
    let resp = frequency_response_db(
        || pedal_processor(src, SAMPLE_RATE, &[]),
        &[PROBE_HZ],
        0.05,
        SAMPLE_RATE,
    );
    resp[0].1
}

/// MASK 7 — STRUCTURAL: the cathode follower into a transformer must NOT
/// compile to a single MultiNL stage that drops the transformer. It must
/// realize the transformer/load as its own R-type (PassiveRType) stage.
///
/// RED before the fix: 1 stage, no PassiveRType (transformer swallowed).
#[test]
fn cf_into_transformer_keeps_transformer_as_its_own_stage() {
    let def = parse_pedal_file(CF_INTO_TRANSFORMER).expect("parse CF->xfmr");
    let compiled = compile_pedal(&def, SAMPLE_RATE).expect("compile CF->xfmr");
    let dump = compiled.debug_dump();
    let stages = stage_count(&dump);
    eprintln!("CF->transformer: {stages} stage(s)\n{dump}");

    assert!(
        stages >= 2,
        "CF->transformer must compile to >= 2 stages (NL stage + transformer \
         R-type), got {stages} — the transformer was swallowed into the NL group"
    );
    assert!(
        dump.contains("PassiveRType"),
        "CF->transformer must realize the transformer as a PassiveRType stage; \
         dump had none (transformer absorbed/discarded by the MultiNL builder)"
    );
}

/// MASK 7 — LEVEL: forward gain through the cathode follower into the
/// transformer must be healthy (> -40 dB), not collapsed to the silence floor.
///
/// RED before the fix: ~-220 dB (truly zero). The CF-into-load reference is
/// printed for context (proves the CF stage itself is alive either way).
#[test]
fn cf_into_transformer_passes_signal() {
    let load_gain = forward_gain_db(CF_INTO_LOAD);
    let xfmr_gain = forward_gain_db(CF_INTO_TRANSFORMER);
    eprintln!(
        "mask-7 probe @ {PROBE_HZ} Hz: CF->load {load_gain:+.1} dB (reference), \
         CF->transformer {xfmr_gain:+.1} dB"
    );

    assert!(
        load_gain > -40.0,
        "reference CF->load should be healthy, got {load_gain:+.1} dB — \
         fixture broken, not the bug under test"
    );
    assert!(
        xfmr_gain > -40.0,
        "MASK 7: CF->transformer forward gain {xfmr_gain:+.1} dB collapsed \
         (expected > -40 dB) — transformer swallowed into the CF's NL group"
    );
}
