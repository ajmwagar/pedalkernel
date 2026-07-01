// ═══════════════════════════════════════════════════════════════════════════
// Bias analysis tests
//
// TDD tests for compile-time DC bias extraction from circuit graphs.
//
// The bias analysis module answers two questions for each flow group:
//   1. Is this group a static bias network? (supply-only inputs → fixed DC)
//   2. If so, what DC voltage does it produce at each junction node?
//
// Static bias groups (e.g. VCC voltage dividers) don't carry audio signal.
// Their DC operating point is extracted at compile time and applied to the
// connected active element's model parameters (op-amp rail limits, BJT
// quiescent point, tube bias). The group is then bypassed in the serial
// audio processing chain.
//
// Dynamic modulation groups (e.g. Fairchild sidechain) have audio-rate
// inputs and must be processed at runtime — but through modulation routing,
// not the serial audio chain.
//
// The detection is graph-level: "does this branch have an audio-rate input
// or only DC supply?" No component-type special-casing.
// ═══════════════════════════════════════════════════════════════════════════

use super::bias::{classify_group_bias, GroupBiasKind};
use super::graph::CircuitGraph;
use super::signal_flow::find_flow_groups;
use super::spqr_build::compile_via_spqr;
use crate::PedalProcessor;

const SR: f64 = 48000.0;

/// Helper: parse a pedal, build graph, find flow groups, return the graph + groups.
fn parse_and_group(source: &str) -> (CircuitGraph, Vec<super::signal_flow::FlowGroup>) {
    let pedal = crate::dsl::parse_pedal_file(source).expect("parse");
    let graph = super::graph::CircuitGraph::from_pedal(&pedal);
    let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
    let groups = find_flow_groups(&all_edges, &graph);
    (graph, groups)
}

// ═══════════════════════════════════════════════════════════════════════════
// 1. A simple VCC voltage divider is detected as static bias
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn vcc_divider_is_static_bias() {
    // R_v1/R_v2/C_v from VCC to GND creates a 4.5V reference.
    // This group's only inputs are VCC and GND — pure DC, no audio.
    // Need an active element so find_flow_groups doesn't merge everything.
    let (graph, groups) = parse_and_group(
        r#"
        pedal "test" { supply 9V
            components {
                R_v1: resistor(10k)
                R_v2: resistor(10k)
                C_v: cap(47u, electrolytic)
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
            }
            nets {
                vcc -> R_v1.a
                R_v1.b -> R_v2.a, C_v.a
                R_v2.b -> gnd
                C_v.b -> gnd
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    );

    // Find the group containing R_v1
    let bias_group_idx = groups
        .iter()
        .position(|g| {
            g.all_edges()
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "R_v1")
        })
        .expect("should find bias group");

    let kind = classify_group_bias(&groups[bias_group_idx], &graph);
    assert!(
        matches!(kind, GroupBiasKind::StaticBias { .. }),
        "VCC divider should be StaticBias, got: {kind:?}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Static bias computes correct DC voltage from resistor divider
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn static_bias_voltage_is_correct() {
    // R_v1=10k from 9V, R_v2=10k to GND → vb = 4.5V
    let (graph, groups) = parse_and_group(
        r#"
        pedal "test" { supply 9V
            components {
                R_v1: resistor(10k)
                R_v2: resistor(10k)
                C_v: cap(47u, electrolytic)
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
            }
            nets {
                vcc -> R_v1.a
                R_v1.b -> R_v2.a, C_v.a
                R_v2.b -> gnd
                C_v.b -> gnd
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    );

    let bias_group_idx = groups
        .iter()
        .position(|g| {
            g.all_edges()
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "R_v1")
        })
        .expect("should find bias group");

    let kind = classify_group_bias(&groups[bias_group_idx], &graph);
    match kind {
        GroupBiasKind::StaticBias { dc_voltages } => {
            // The junction node (R_v1.b = R_v2.a = C_v.a) should be at 4.5V
            assert!(
                !dc_voltages.is_empty(),
                "Should compute at least one DC voltage"
            );
            let vb = dc_voltages.values().next().unwrap();
            assert!(
                (vb - 4.5).abs() < 0.1,
                "Bias voltage should be ~4.5V, got {vb:.2}V"
            );
        }
        other => panic!("Expected StaticBias, got {other:?}"),
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Asymmetric divider computes correct voltage
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn asymmetric_divider_voltage() {
    // R_top=20k from 9V, R_bot=10k to GND → vb = 9 * 10/(20+10) = 3.0V
    let (graph, groups) = parse_and_group(
        r#"
        pedal "test" { supply 9V
            components {
                R_top: resistor(20k)
                R_bot: resistor(10k)
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
            }
            nets {
                vcc -> R_top.a
                R_top.b -> R_bot.a
                R_bot.b -> gnd
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    );

    let bias_group_idx = groups
        .iter()
        .position(|g| {
            g.all_edges()
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "R_top")
        })
        .expect("should find bias group");

    let kind = classify_group_bias(&groups[bias_group_idx], &graph);
    match kind {
        GroupBiasKind::StaticBias { dc_voltages } => {
            let vb = dc_voltages.values().next().unwrap();
            assert!(
                (vb - 3.0).abs() < 0.1,
                "Bias voltage should be ~3.0V, got {vb:.2}V"
            );
        }
        other => panic!("Expected StaticBias, got {other:?}"),
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Input coupling is NOT static bias (it's on the audio path)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn input_coupling_is_signal_path() {
    let (graph, groups) = parse_and_group(
        r#"
        pedal "test" { supply 9V
            components {
                R_v1: resistor(10k)
                R_v2: resistor(10k)
                Cin: cap(22n)
                R_in: resistor(510k)
                U1: opamp(tl072)
                Rf: resistor(100k)
            }
            nets {
                vcc -> R_v1.a
                R_v1.b -> R_v2.a
                R_v2.b -> gnd
                in -> Cin.a
                Cin.b -> R_in.a, U1.neg
                R_in.b -> gnd
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    );

    // Find the group containing Cin
    let input_group_idx = groups
        .iter()
        .position(|g| {
            g.all_edges()
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "Cin")
        })
        .expect("should find input group");

    let kind = classify_group_bias(&groups[input_group_idx], &graph);
    assert!(
        matches!(kind, GroupBiasKind::SignalPath),
        "Input coupling should be SignalPath, got: {kind:?}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. Op-amp clipping stage is NOT static bias
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn opamp_clipping_is_signal_path() {
    let (graph, groups) = parse_and_group(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D_clip: diode_pair(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.out -> D_clip.a
                D_clip.b -> U1.neg
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    );

    // Find the group containing U1
    let clipping_group_idx = groups
        .iter()
        .position(|g| {
            g.all_edges()
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "U1")
        })
        .expect("should find clipping group");

    let kind = classify_group_bias(&groups[clipping_group_idx], &graph);
    assert!(
        matches!(kind, GroupBiasKind::SignalPath),
        "Clipping stage should be SignalPath, got: {kind:?}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. Bias network in full screamer-like circuit detected correctly
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_bias_group_is_static() {
    let (graph, groups) = parse_and_group(
        r#"
        pedal "test" { supply 9V
            components {
                R_v1: resistor(10k)
                R_v2: resistor(10k)
                C_v: cap(47u, electrolytic)
                Cin: cap(22n)
                R_in: resistor(510k)
                C_c1: cap(1u, electrolytic)
                R_b1: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D_clip: diode_pair(silicon)
                R_out: resistor(10k)
            }
            nets {
                vcc -> R_v1.a
                R_v1.b -> R_v2.a, C_v.a
                R_v2.b -> gnd
                C_v.b -> gnd
                R_v1.b -> vb
                in -> Cin.a
                Cin.b -> R_in.a, C_c1.a
                R_in.b -> gnd
                C_c1.b -> U1.pos, R_b1.a
                R_b1.b -> vb
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.out -> D_clip.a
                D_clip.b -> U1.neg
                U1.out -> R_out.a
                R_out.b -> out
            }
            controls {}
        }"#,
    );

    // Find the group containing R_v1 (bias network)
    let bias_group_idx = groups
        .iter()
        .position(|g| {
            g.all_edges()
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "R_v1")
        })
        .expect("should find bias group");

    let kind = classify_group_bias(&groups[bias_group_idx], &graph);
    assert!(
        matches!(kind, GroupBiasKind::StaticBias { .. }),
        "Screamer bias network should be StaticBias, got: {kind:?}"
    );

    // The clipping group should be SignalPath
    let clipping_group_idx = groups
        .iter()
        .position(|g| {
            g.all_edges()
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "U1")
        })
        .expect("should find clipping group");

    let clipping_kind = classify_group_bias(&groups[clipping_group_idx], &graph);
    assert!(
        matches!(clipping_kind, GroupBiasKind::SignalPath),
        "Clipping stage should be SignalPath, got: {clipping_kind:?}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 7. Bypassing bias stage produces audio output in serial chain
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bypassing_bias_stage_produces_audio() {
    // Full screamer-like circuit with bias network.
    // When the bias stage is bypassed in the serial chain,
    // the clipping stage's output should reach the circuit output.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_v1: resistor(10k)
                R_v2: resistor(10k)
                C_v: cap(47u, electrolytic)
                Cin: cap(22n)
                R_in: resistor(510k)
                C_c1: cap(1u, electrolytic)
                R_b1: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D_clip: diode_pair(silicon)
                R_out: resistor(10k)
            }
            nets {
                vcc -> R_v1.a
                R_v1.b -> R_v2.a, C_v.a
                R_v2.b -> gnd
                C_v.b -> gnd
                R_v1.b -> vb
                in -> Cin.a
                Cin.b -> R_in.a, C_c1.a
                R_in.b -> gnd
                C_c1.b -> U1.pos, R_b1.a
                R_b1.b -> vb
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.out -> D_clip.a
                D_clip.b -> U1.neg
                U1.out -> R_out.a
                R_out.b -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    let amp = 0.1;
    let freq = 440.0;
    // Warmup
    for s in 0..2000 {
        compiled.process(amp * (std::f64::consts::TAU * freq * s as f64 / SR).sin());
    }
    // Measure
    let mut peak = 0.0f64;
    for s in 0..500 {
        let out =
            compiled.process(amp * (std::f64::consts::TAU * freq * (2000 + s) as f64 / SR).sin());
        peak = peak.max(out.abs());
    }

    eprintln!("Bypass bias test: peak={peak:.4}V");
    // The bias network must not kill the signal. Any non-trivial output proves
    // the bypass works. Gain accuracy is tested separately.
    assert!(
        peak > 0.001,
        "Signal should survive past bias network: {peak:.6}V"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 8. SD-1 legend produces audio when bias network is bypassed
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn sd1_legend_audio_with_bias_bypass() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/sd1.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read sd1.pedal");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    let amp = 0.1;
    let freq = 440.0;
    for s in 0..4000 {
        compiled.process(amp * (std::f64::consts::TAU * freq * s as f64 / SR).sin());
    }
    let mut peak = 0.0f64;
    for s in 0..500 {
        let out =
            compiled.process(amp * (std::f64::consts::TAU * freq * (4000 + s) as f64 / SR).sin());
        peak = peak.max(out.abs());
    }

    eprintln!("SD-1 with bias bypass: peak={peak:.4}V");
    assert!(peak > 0.001, "SD-1 should produce audio: {peak:.4}V");
}
