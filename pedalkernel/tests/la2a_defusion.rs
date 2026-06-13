//! Mask-8 (detector-loop over-fusion) gates for the FAITHFUL LA-2A rebuild.
//!
//! RCA (`reports/signal-routing-formation-layer-2026-06-13.md` §2, confirmed by
//! reproduction 2026-06-13): the faithful `la2a.pedal` wires the detector as a
//! real feedback tap from the output (`out -> C_sc.a`, la2a.pedal:198) bridged
//! back to the `in -> fork -> R_ff -> R37a` node. In the passive group-formation
//! pass `signal_flow.rs` does NOT treat the global `in`/`out` nodes as barriers,
//! so the detector tap bridges the input transformer (`T_in`, via `in` and the
//! Limit feed-forward arm `in -> fork -> R_ff -> R37a`) and the output
//! transformer (`T_out`, via the `out -> C_sc.a` tap) into ONE connected passive
//! PassiveRType group whose `debug_label` contains BOTH "T_in" and "T_out".
//!
//! This tap is a DETECTOR (program-dependent gain reduction) whose physical
//! realization is DELAYED (the photocoupler integrates over ms-to-s) — it is NOT
//! instantaneous electrical feedback and must not fuse the audio network.
//!
//! STATUS (2026-06-13): both gates are `#[ignore]`d RED.
//!   * The STRUCTURAL gate is the de-fusion proof. Cutting the detector tap
//!     `out -> C_sc.a` from the formation pass de-fuses the network (verified by
//!     reproduction: T_in and T_out then land in separate stages). It is the
//!     target of the detector-tap-cut fix.
//!   * The LEVEL gate is BLOCKED BY A THIRD CAUSE that de-fusion alone does NOT
//!     clear. Measured during this pass: a forward-only LA-2A (the SAME forward
//!     chain with the entire sidechain removed, so there is NOTHING to fuse and
//!     T_in/T_out are already in separate stages — see `forward_only_*` below)
//!     still reads only ~-78.5 dB at 1 kHz, well below the -40 dB floor. The
//!     collapse is therefore in the forward chain itself (cascaded GR divider
//!     with the dark T4B cell + the two-transformer chain, GAP F step-down
//!     ~19 dB, and the multi-stage cascade routing — the same family as mask 7's
//!     marginal -38.6 dB, compounded), NOT the detector-loop fusion. Fixing the
//!     level is OUT OF SCOPE for the de-fusion task and is left RED until the
//!     third cause is addressed (the larger CrossNetworkCoupler / forward-cascade
//!     work).
//!
//! Run the red suite:
//!   cargo test -p pedalkernel --no-default-features --test la2a_defusion -- --ignored --nocapture

mod audio_analysis;

use audio_analysis::*;

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use pedalkernel_rt::processor::Stage;

const LA2A: &str = "la2a.pedal";
const PROBE_HZ: f64 = 1_000.0;

/// The comma-joined component-id `debug_label` for a stage (empty for the
/// Blockwise / KMethod variants that do not carry one). Populated in debug
/// builds (the test profile); empty in release.
fn stage_label(stage: &Stage) -> String {
    match stage {
        Stage::Wdf(w) => w.debug_label.clone(),
        Stage::MultiNl(m) => m.debug_label.clone(),
        Stage::Iir(i) => i.debug_label.clone(),
        Stage::StateSpace(s) => s.debug_label.clone(),
        Stage::BlackFeedback(b) => b.debug_label.clone(),
        Stage::SerialDelayedFeedback(s) => s.debug_label.clone(),
        Stage::Blockwise(_) | Stage::KMethod { .. } => String::new(),
    }
}

/// MASK 8 — STRUCTURAL (the de-fusion proof): NO single compiled stage may
/// contain BOTH "T_in" and "T_out" in its `debug_label`. The detector tap
/// over-fuses the input transformer, the sidechain front-end, and the output
/// transformer into one connected PassiveRType group whose label holds both.
/// After the detector-tap cut they split into separate forward stages.
///
/// RED today: one stage's label
/// `C37,C_c3,C_in,C_sc,PR,R37a,R_ff,R_gr,R_load,T_in,T_out,__fork_0_path_0,__fork_0_path_1`
/// contains both "T_in" and "T_out".
#[test]
#[ignore = "MASK 8 de-fusion proof: detector tap over-fuses T_in+T_out into one passive group (signal_flow.rs passive grouping does not barrier in/out). RED until the detector-tap-cut fix lands."]
fn la2a_output_network_is_not_one_fused_group() {
    let src = example_pedal_source(LA2A);
    let def = parse_pedal_file(&src).expect("parse la2a");
    let compiled = compile_pedal(&def, SAMPLE_RATE).expect("compile la2a");

    let mut both = None;
    for (i, stage) in compiled.stages.iter().enumerate() {
        let label = stage_label(stage);
        eprintln!("[Stage {i}] {stage:?} label={label:?}");
        if label.contains("T_in") && label.contains("T_out") {
            both = Some((i, label));
        }
    }

    assert!(
        both.is_none(),
        "MASK 8: stage {:?} fuses BOTH T_in and T_out into one group — the \
         detector feedback tap over-fused the output audio network",
        both
    );
}

/// MASK 8 — LEVEL: full LA-2A forward gain at 1 kHz must clear the silence
/// floor (> -40 dB). BLOCKED BY A THIRD CAUSE independent of the detector-loop
/// fusion (see `forward_only_chain_is_collapsed_even_without_sidechain`).
///
/// RED today: ~-99 dB with the sidechain present (fused).
#[test]
#[ignore = "BLOCKED BY THIRD CAUSE (forward-cascade collapse, NOT mask-8 fusion): forward-only LA-2A with the sidechain removed still reads ~-78.5 dB. De-fusion alone cannot clear -40 dB. Out of scope for the de-fusion task; left RED."]
fn la2a_forward_path_passes_signal() {
    let src = example_pedal_source(LA2A);
    let controls: &[(&str, f64)] = &[
        ("Gain", 0.6),
        ("Peak Reduction", 0.5),
        ("Limit/Compress", 0.0),
    ];
    let resp = frequency_response_db(
        || pedal_processor(&src, SAMPLE_RATE, controls),
        &[PROBE_HZ],
        0.1,
        SAMPLE_RATE,
    );
    let gain = resp[0].1;
    eprintln!("LA-2A forward gain @ {PROBE_HZ} Hz: {gain:+.1} dB");

    assert!(
        gain > -40.0,
        "MASK 8/level: LA-2A forward gain {gain:+.1} dB collapsed (expected > -40 dB)"
    );
}

/// The LA-2A FORWARD CHAIN ONLY — the SAME components as `la2a.pedal` but with
/// the ENTIRE sidechain/detector removed. There is nothing to fuse, T_in and
/// T_out are already in separate stages, and yet the forward gain still
/// collapses. This isolates the THIRD CAUSE (forward-cascade collapse) from the
/// mask-8 detector-loop fusion, and is the evidence that de-fusion alone cannot
/// raise the LA-2A above the -40 dB floor.
const LA2A_FWD_ONLY: &str = r#"
equipment "la2a forward only" {
  supplies { B+: 275V }
  components {
    T_in: transformer(1:1, 4H)
    C_in: cap(100n)
    R_gr: resistor(100k)
    PC1: photocoupler(t4b)
    V1: triode(12ax7)
    R_g1: resistor(1M)
    R_p1: resistor(220k)
    R_k1: resistor(1.5k)
    C_k1: cap(25u, electrolytic)
    C_c1: cap(22n)
    Gain: pot(1M, a)
    V2: triode(12ax7)
    R_g2: resistor(1M)
    R_p2: resistor(220k)
    R_k2: resistor(1.5k)
    C_k2: cap(25u, electrolytic)
    V3: triode(12bh7)
    C_c2: cap(100n)
    R_g3: resistor(1M)
    R_k3: resistor(10k)
    C_c3: cap(10u, electrolytic)
    T_out: transformer(4:1, 10H)
    R_load: resistor(600)
  }
  nets {
    in -> T_in.a
    T_in.b -> C_in.a
    C_in.b -> R_gr.a
    R_gr.b -> PC1.a, V1.grid, R_g1.a
    PC1.b -> gnd
    R_g1.b -> gnd
    B+ -> R_p1.a
    R_p1.b -> V1.plate
    V1.cathode -> R_k1.a, C_k1.a
    R_k1.b -> gnd
    C_k1.b -> gnd
    V1.plate -> C_c1.a
    C_c1.b -> Gain.a
    Gain.b -> gnd
    Gain.w -> V2.grid, R_g2.a
    R_g2.b -> gnd
    B+ -> R_p2.a
    R_p2.b -> V2.plate
    V2.cathode -> R_k2.a, C_k2.a
    R_k2.b -> gnd
    C_k2.b -> gnd
    V2.plate -> C_c2.a
    C_c2.b -> V3.grid, R_g3.a
    R_g3.b -> gnd
    B+ -> V3.plate
    V3.cathode -> R_k3.a, C_c3.a
    R_k3.b -> gnd
    C_c3.b -> T_out.a
    T_out.b -> R_load.a, out
    R_load.b -> gnd
  }
  controls { Gain.position -> "Gain" [0.0, 1.0] = 0.6 }
}
"#;

/// Forward-only de-fusion control: T_in and T_out are in SEPARATE stages (no
/// sidechain to bridge them), proving the structural de-fusion is achievable —
/// the over-fusion is purely the detector tap.
#[test]
fn forward_only_is_already_defused() {
    let def = parse_pedal_file(LA2A_FWD_ONLY).expect("parse fwd-only");
    let compiled = compile_pedal(&def, SAMPLE_RATE).expect("compile fwd-only");
    let mut both = false;
    for (i, stage) in compiled.stages.iter().enumerate() {
        let label = stage_label(stage);
        eprintln!("[Stage {i}] {stage:?} label={label:?}");
        if label.contains("T_in") && label.contains("T_out") {
            both = true;
        }
    }
    assert!(
        !both,
        "forward-only chain should NOT fuse T_in+T_out (no sidechain to bridge)"
    );
}

/// THIRD-CAUSE evidence: the forward chain alone (no sidechain, already de-fused)
/// still collapses far below -40 dB. This documents that the LA-2A level gate is
/// blocked by a forward-cascade cause independent of mask-8 fusion.
#[test]
#[ignore = "THIRD-CAUSE evidence (forward-cascade collapse): forward-only LA-2A reads ~-78.5 dB at 1 kHz despite being fully de-fused. Documents that de-fusion alone cannot clear -40 dB."]
fn forward_only_chain_is_collapsed_even_without_sidechain() {
    let resp = frequency_response_db(
        || pedal_processor(LA2A_FWD_ONLY, SAMPLE_RATE, &[("Gain", 0.6)]),
        &[PROBE_HZ],
        0.1,
        SAMPLE_RATE,
    );
    let gain = resp[0].1;
    eprintln!("LA-2A forward-only (no sidechain) gain @ {PROBE_HZ} Hz: {gain:+.1} dB");
    assert!(
        gain > -40.0,
        "forward-only LA-2A gain {gain:+.1} dB collapsed (expected > -40 dB) — \
         third cause in the forward cascade, NOT the detector-loop fusion"
    );
}
