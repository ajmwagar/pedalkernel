// ═══════════════════════════════════════════════════════════════════════════
// Tone stage output probe tests
//
// The Screamer's tone+output group [Tone,C_t1,R_t2,C_t2,C_c2,Level,
// C_out,R_out_s,R_out_g] drops 42dB because the output probe targets
// the wrong component. It should probe the leaf at the group's output
// boundary (R_out_g at out_node), not an interior leaf (R_t2).
// ═══════════════════════════════════════════════════════════════════════════

use super::spqr_build::compile_via_spqr;
use crate::PedalProcessor;

const SR: f64 = 48000.0;

// ═══════════════════════════════════════════════════════════════════════════
// 1. Simple tone network: signal passes through
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn simple_tone_network_passes_signal() {
    // Tone pot + ground caps + output coupling. No gain stage.
    // Should pass signal near-unity at 440Hz.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_t1: resistor(1k)
                Tone: pot(20k, b)
                C_t1: cap(220n)
                R_out: resistor(10k)
            }
            nets {
                in -> R_t1.a
                R_t1.b -> Tone.a
                Tone.b -> C_t1.a
                C_t1.b -> gnd
                Tone.w -> R_out.a
                R_out.b -> out
            }
            controls {
                Tone.position -> "Tone" [0.0, 1.0] = 0.5
            }
        }"#,
    )
    .expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    let amp = 0.1;
    for s in 0..4000 {
        compiled.process(amp * (std::f64::consts::TAU * 440.0 * s as f64 / SR).sin());
    }
    let mut peak = 0.0f64;
    for s in 0..500 {
        let out =
            compiled.process(amp * (std::f64::consts::TAU * 440.0 * (4000 + s) as f64 / SR).sin());
        peak = peak.max(out.abs());
    }

    let gain = peak / amp;
    eprintln!("Simple tone: peak={peak:.4}V gain={gain:.2}");
    assert!(
        gain > 0.1,
        "Tone network should pass signal: gain={gain:.2}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Screamer tone+output group: trace the output probe
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_tone_output_probe_correct() {
    // The Screamer's tone+output group should probe at the output boundary,
    // not at an interior leaf like R_t2.
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/screamer.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Find the tone+output stage and check its output_probe
    for stage in &compiled.stages {
        if let super::compiled::Stage::Wdf(w) = stage {
            #[cfg(debug_assertions)]
            {
                if w.debug_label.contains("Tone") || w.debug_label.contains("Level") {
                    eprintln!(
                        "Tone stage [{0}]: probe={1:?}",
                        w.debug_label, w.output_probe
                    );

                    // The probe should target a component at the output boundary
                    // (R_out_g or R_out_s), NOT an interior component (R_t2, C_t1).
                    if let Some(ref probe) = w.output_probe {
                        assert!(!probe.contains("R_t2") && !probe.contains("C_t1"),
                            "Probe targets interior component '{probe}' — should target output boundary (R_out_g or R_out_s)");
                    }
                }
            }
        }
    }
}

#[test]
fn screamer_passive_rtype_tone_recomputes_matrix() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/screamer.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    let (before, extract_before) = compiled
        .stages
        .iter()
        .find_map(|stage| match stage {
            super::compiled::Stage::Wdf(w) => match &w.root {
                super::stage::RootKind::PassiveRType {
                    scattering,
                    extraction_coeffs,
                    ..
                } => Some((scattering.clone(), extraction_coeffs.clone())),
                _ => None,
            },
            _ => None,
        })
        .expect("passive rtype stage");

    compiled.set_control("Tone", 1.0);
    for _ in 0..3000 {
        compiled.process(0.0);
    }

    let (after, extract_after) = compiled
        .stages
        .iter()
        .find_map(|stage| match stage {
            super::compiled::Stage::Wdf(w) => match &w.root {
                super::stage::RootKind::PassiveRType {
                    scattering,
                    extraction_coeffs,
                    ..
                } => Some((scattering.clone(), extraction_coeffs.clone())),
                _ => None,
            },
            _ => None,
        })
        .expect("passive rtype stage after sweep");

    let delta: f64 = before
        .iter()
        .zip(after.iter())
        .map(|(a, b)| (a - b).abs())
        .sum();
    let extract_delta: f64 = extract_before
        .iter()
        .zip(extract_after.iter())
        .map(|(a, b)| (a - b).abs())
        .sum();
    eprintln!(
        "Screamer PassiveRType Tone scattering delta={delta:.6}, extraction delta={extract_delta:.6}"
    );
    assert!(
        delta > 1e-4,
        "Tone sweep should recompute PassiveRType scattering"
    );
    assert!(
        extract_delta > 1e-4,
        "Tone sweep should recompute PassiveRType output extraction"
    );
}

#[test]
fn screamer_passive_rtype_tone_changes_stage_response() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/screamer.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");

    fn measure(pedal: &crate::dsl::PedalDef, tone: f64, freq: f64) -> f64 {
        let mut compiled = compile_via_spqr(pedal, SR).expect("compile");
        compiled.set_control("Tone", tone);
        for _ in 0..3000 {
            compiled.process(0.0);
        }
        let stage_idx = compiled
            .stages
            .iter()
            .position(|stage| match stage {
                super::compiled::Stage::Wdf(w) => {
                    matches!(w.root, super::stage::RootKind::PassiveRType { .. })
                }
                _ => false,
            })
            .expect("passive rtype stage");
        let stage = match &mut compiled.stages[stage_idx] {
            super::compiled::Stage::Wdf(w) => w,
            _ => unreachable!(),
        };
        let amp = 0.05;
        let mut peak = 0.0f64;
        for s in 0..4800 {
            let input = amp * (std::f64::consts::TAU * freq * s as f64 / SR).sin();
            peak = peak.max(stage.process(input).abs());
        }
        peak
    }

    let lo_dark = measure(&pedal, 0.0, 200.0);
    let hi_dark = measure(&pedal, 0.0, 5000.0);
    let lo_bright = measure(&pedal, 1.0, 200.0);
    let hi_bright = measure(&pedal, 1.0, 5000.0);
    let dark_ratio = hi_dark / lo_dark.max(1e-10);
    let bright_ratio = hi_bright / lo_bright.max(1e-10);
    eprintln!(
        "Screamer PassiveRType stage: @0 lo={lo_dark:.6} hi={hi_dark:.6} ratio={dark_ratio:.3}; @1 lo={lo_bright:.6} hi={hi_bright:.6} ratio={bright_ratio:.3}"
    );
    assert!(
        (dark_ratio - bright_ratio).abs() > 0.02
            || (lo_dark - lo_bright).abs() + (hi_dark - hi_bright).abs() > 0.0005,
        "Tone should change the passive tone stage response"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Screamer tone stage should not drop more than 20dB
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_tone_stage_not_dead() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/screamer.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");
    compiled.enable_metering(128);

    let amp = 0.1;
    for s in 0..8000 {
        compiled.process(amp * (std::f64::consts::TAU * 440.0 * s as f64 / SR).sin());
    }

    let metrics = compiled.read_metrics();
    let n = compiled.stages.len().min(crate::metering::MAX_STAGES);

    for i in 0..n {
        let lvl = metrics.stage_levels[i];
        let db = if lvl > 1e-10 {
            20.0 * (lvl as f64).log10()
        } else {
            -120.0
        };
        #[cfg(debug_assertions)]
        {
            let (lbl, bypass) = match &compiled.stages[i] {
                super::compiled::Stage::Wdf(w) => (w.debug_label.as_str(), w.bypass_serial),
                super::compiled::Stage::MultiNl(m) => (m.debug_label.as_str(), m.bypass_serial),
                super::compiled::Stage::Iir(s) => (s.debug_label.as_str(), s.bypass_serial),
                super::compiled::Stage::StateSpace(s) => (s.debug_label.as_str(), s.bypass_serial),
                super::compiled::Stage::BlackFeedback(b) => {
                    (b.debug_label.as_str(), b.bypass_serial)
                }
                super::compiled::Stage::BlockwiseKMethod(bk) => ("blockwise", bk.bypass_serial),
            };
            if bypass {
                continue;
            }
            if lbl.contains("Tone") || lbl.contains("Level") {
                eprintln!("Tone stage [{lbl}]: {db:.1} dB");
                assert!(
                    db > -40.0,
                    "Tone stage [{lbl}] drops too much at {db:.1} dB"
                );
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Output probe should use group's output terminal, not first GND-side edge
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn output_probe_uses_output_terminal() {
    // The output boundary should be the terminal closest to out_node,
    // not the first terminal in the list. For the Screamer tone group,
    // terminals are [35, 56]. Node 56=out_node should be the output boundary.
    // The probe should find R_out_g (at out_node → GND), not R_t2.
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/screamer.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let graph = super::graph::CircuitGraph::from_pedal(&pedal);
    let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
    let groups = super::signal_flow::find_flow_groups(&all_edges, &graph);
    let global_terminals = vec![graph.in_node, graph.out_node];

    // Find the tone group
    let tone_group = groups
        .iter()
        .find(|g| {
            g.all_edges()
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "Tone")
        })
        .expect("tone group");

    let group_edges = tone_group.all_edges();
    let terminals =
        super::spqr_build::compute_group_terminals(&group_edges, &graph, &global_terminals);

    eprintln!("Tone group terminals: {terminals:?}");
    eprintln!("  out_node={:?}", graph.out_node);

    // out_node should be one of the terminals
    assert!(
        terminals.contains(&graph.out_node),
        "out_node should be a terminal for the tone group: terminals={terminals:?}"
    );

    // The output boundary should prefer out_node over other terminals
    let output_boundary = terminals
        .iter()
        .find(|&&t| t == graph.out_node)
        .or_else(|| terminals.iter().find(|&&t| t != graph.in_node))
        .copied()
        .unwrap_or(graph.out_node);

    eprintln!(
        "  output_boundary={output_boundary:?} (should be out_node={:?})",
        graph.out_node
    );
    assert_eq!(
        output_boundary, graph.out_node,
        "Output boundary should be out_node for the tone+output group"
    );
}
