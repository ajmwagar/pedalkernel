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
/// contain BOTH "T_in" and "T_out" in its `debug_label`.
///
/// GREEN since the 4-terminal transformer rewire (`la2a.pedal`): wiring T_in
/// and T_out as proper two-ports (primary a/b, secondary c/d) gives the output
/// transformer galvanic isolation, so the detector tap no longer bridges T_in
/// and T_out into one passive group. T_out now lands in its own stage with the
/// V3 cathode follower; T_in is separate. The faithful wiring resolves the
/// mask-8 over-fusion without a detector-tap-cut compiler change.
#[test]
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

/// PHASE 2a (broker-driven de-fusion) — STRUCTURAL: `T_in` must NOT share a
/// stage with the detector front-end (`PR`/`R37a`/`R_ff`/`C37`). Before 2a, the
/// detector feedback tap fused `C_sc,R37a,PR,R_ff,...,T_in,R_load` into ONE
/// passive group, stranding `T_in` at a large signal-flow distance and starving
/// the forward V1 group. The broker `delayed_cut_edges` cuts the tap-mouth edges
/// (`C_sc` out-side, the `R_ff`/fork arm in-side) so the detector de-fuses into
/// its own stage. GREEN since Phase 2a.
#[test]
fn t_in_is_not_in_the_detector_group() {
    let src = example_pedal_source(LA2A);
    let def = parse_pedal_file(&src).expect("parse la2a");
    let compiled = compile_pedal(&def, SAMPLE_RATE).expect("compile la2a");

    // The stage holding the detector front-end (the Peak-Reduction tap network).
    let mut detector_stage = None;
    let mut t_in_stage = None;
    for (i, stage) in compiled.stages.iter().enumerate() {
        let label = stage_label(stage);
        let comps: Vec<&str> = label.split(',').collect();
        eprintln!("[Stage {i}] label={label:?}");
        // Detector front-end = the Peak-Reduction pot tap network (PR/R37a).
        if comps.iter().any(|c| *c == "PR") && comps.iter().any(|c| *c == "R37a") {
            detector_stage = Some(i);
        }
        if comps.iter().any(|c| *c == "T_in") {
            t_in_stage = Some(i);
        }
    }

    let detector_stage = detector_stage.expect("detector front-end (PR/R37a) must compile");
    let t_in_stage = t_in_stage.expect("T_in must compile into a stage");
    assert_ne!(
        detector_stage, t_in_stage,
        "PHASE 2a: T_in (stage {t_in_stage}) is still fused with the detector \
         front-end (stage {detector_stage}) — the tap-mouth cut did not de-fuse"
    );

    // And the detector front-end must NOT drag T_in OR the output load in.
    let det_label = stage_label(&compiled.stages[detector_stage]);
    let det_comps: Vec<&str> = det_label.split(',').collect();
    assert!(
        !det_comps.contains(&"T_in"),
        "detector stage must not contain T_in: {det_label:?}"
    );
    assert!(
        !det_comps.contains(&"R_load") && !det_comps.contains(&"T_out"),
        "detector stage must not contain the output network: {det_label:?}"
    );
}

/// MASK 8 — LEVEL: full LA-2A forward gain at 1 kHz must clear the silence
/// floor (> -40 dB). BLOCKED BY A THIRD CAUSE independent of the detector-loop
/// fusion (see `forward_only_chain_is_collapsed_even_without_sidechain`).
///
/// RED: ~-121.8 dB post 4-terminal rewire. The output network is now DE-FUSED
/// (structural gate green), so this is NOT transformer wiring or mask-8 fusion.
#[test]
#[ignore = "THIRD CAUSE (transformer Tight-coupling) is FIXED: T_in primary is no longer bypass_serial and the output group sorts after V1 (broker `is_tight_coupled_link` honored by bias_analysis reachability + the flow-distance BFS). The forward path now PASSES — a 4-terminal forward-only LA-2A reads +17 dB, and removing only the `in -> fork(LC, [gnd, R_ff.a])` arm from the full netlist gives +23 dB. The remaining full-LA-2A collapse (-99.5 dB) is a SEPARATE, fourth cause: that Limit/Compress fork's `in -> gnd` arm shares the `in` node with T_in.a and shorts the live input (it is the detector feed-forward mouth at the `in` seed, skipped by `delayed_cut_edges` because its interior is the gnd rail). Un-bypassing T_in (correctly) exposed that pre-existing short. Fixing it is detector/fork de-fusion work, out of scope for the transformer-coupling task. Left ignored pending the fork de-fuse."]
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

/// PHASE 1 (cross-network-coupler de-fuse) — LED-side isolation barrier.
///
/// The faithful la2a.pedal now wires the T4B LED as a real galvanically-isolated
/// electrical port-pair: `EL_drive.b -> PC1.led.a` / `gnd -> PC1.led.b`. The LED
/// edge is declared `EdgeKind::Behavioral`, so it is NEVER claimed into a
/// FlowGroup and CANNOT union the sidechain (LED-side) network into the audio
/// (LDR-side) network. This test asserts the structural barrier holds: the LDR
/// leaf (PC1) and the LED-driving sidechain element (EL_drive, the 6AQ5 panel
/// winding) land in SEPARATE stages — they were never electrically fused
/// through the LED. (Phase 1 keeps the LED optically dark — no contribute/read
/// yet — so this is purely a non-merge / boundary-port structural check.)
#[test]
fn la2a_photocoupler_led_does_not_fuse_sidechain_into_audio() {
    let src = example_pedal_source(LA2A);
    let def = parse_pedal_file(&src).expect("parse la2a");
    let compiled = compile_pedal(&def, SAMPLE_RATE).expect("compile la2a");

    let mut pc1_stage = None;
    let mut el_drive_stage = None;
    for (i, stage) in compiled.stages.iter().enumerate() {
        let label = stage_label(stage);
        eprintln!("[Stage {i}] label={label:?}");
        // Match the LDR leaf id exactly (`PC1`), not the EL-drive substring.
        if label.split(',').any(|c| c == "PC1") {
            pc1_stage = Some(i);
        }
        if label.split(',').any(|c| c == "EL_drive") {
            el_drive_stage = Some(i);
        }
    }

    let pc1_stage = pc1_stage.expect("PC1 (LDR) must compile into a stage");
    let el_drive_stage =
        el_drive_stage.expect("EL_drive (LED-side winding) must compile into a stage");
    assert_ne!(
        pc1_stage, el_drive_stage,
        "PHASE 1: the photocoupler LDR side (PC1, stage {pc1_stage}) and the \
         LED-driving sidechain winding (EL_drive, stage {el_drive_stage}) fused \
         into ONE stage — the Behavioral LED edge failed to isolate the two \
         galvanically-isolated networks"
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

/// FORWARD-PATH PASSES (was the "third cause" evidence, now GREEN): the forward
/// chain alone (no sidechain, already de-fused) used to collapse far below -40 dB.
/// With the transformer Tight-coupling fix (broker `is_tight_coupled_link` honored
/// by the reachability + flow-distance analyses) the forward cascade now passes
/// signal: this de-fused forward-only LA-2A reads ~+9.6 dB at 1 kHz (was ~-30.8).
/// The historical name is kept for traceability; the assertion (> -40 dB) is the
/// same — it simply holds now. (The FULL la2a is still blocked by the unrelated
/// Limit/Compress fork short — see `la2a_forward_path_passes_signal`.)
#[test]
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
