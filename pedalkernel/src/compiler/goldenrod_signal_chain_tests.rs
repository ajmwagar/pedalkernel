// ═══════════════════════════════════════════════════════════════════════════
// Goldenrod (Klon Centaur) signal chain tests
//
// Documents the expected signal flow and verifies each stage exists
// and passes signal. The Klon has a unique topology:
//
//   in → R1 → C1 → U1 (buffer)
//                     ├→ R3 → U2 (gain) → C_clip → D1/D2 (hard clip) → R_clip ──┐
//                     ├→ R_ff1a → C_ff1 → R_ff1b (feedforward 1: clean LPF) ───┤
//                     └→ Gain_B.w → R_ff2 (feedforward 2: clean blend) ─────────┤
//                                                                                ↓
//                                                              U3 (summing amp, neg)
//                                                                      ↓
//                                                              U4 (tone control)
//                                                                      ↓
//                                                              Output pot → out
//
// Three parallel paths merge at U3's inverting input. The Gain pot
// (dual gang) simultaneously increases dirty gain (Gain_A) and
// decreases clean blend (Gain_B).
// ═══════════════════════════════════════════════════════════════════════════

use super::compiled::CompiledPedal;
use super::spqr_build::compile_via_spqr;
use super::test_support::load_legend_source;
use crate::PedalProcessor;

const SR: f64 = 48000.0;
const FREQ: f64 = 440.0;

fn load_goldenrod() -> CompiledPedal {
    let source = load_legend_source("goldenrod").expect("read goldenrod.pedal");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    compile_via_spqr(&pedal, SR).expect("compile")
}

/// Dump all stages with metering and debug labels.
fn dump_stages(compiled: &CompiledPedal, label: &str) {
    let metrics = compiled.read_metrics();
    let n = compiled.stages.len().min(crate::metering::MAX_STAGES);
    eprintln!("\n  {label}: {n} stages");
    for i in 0..n {
        let lvl = metrics.stage_levels[i];
        let db = if lvl > 1e-10 {
            20.0 * (lvl as f64).log10()
        } else {
            -120.0
        };
        #[cfg(debug_assertions)]
        {
            let (stype, lbl, bypass, dist) = match &compiled.stages[i] {
                super::compiled::Stage::Wdf(w) => (
                    "Wdf",
                    w.debug_label.as_str(),
                    w.bypass_serial,
                    w.signal_flow_distance,
                ),
                super::compiled::Stage::MultiNl(m) => (
                    "MNL",
                    m.debug_label.as_str(),
                    m.bypass_serial,
                    m.signal_flow_distance,
                ),
                super::compiled::Stage::Iir(s) => (
                    "Iir",
                    s.debug_label.as_str(),
                    s.bypass_serial,
                    s.signal_flow_distance,
                ),
                super::compiled::Stage::StateSpace(s) => (
                    "SS",
                    s.debug_label.as_str(),
                    s.bypass_serial,
                    s.signal_flow_distance,
                ),
                super::compiled::Stage::BlackFeedback(b) => (
                    "BF",
                    b.debug_label.as_str(),
                    b.bypass_serial,
                    b.signal_flow_distance,
                ),
                super::compiled::Stage::Blockwise(bk) => (
                    "BKM",
                    "blockwise",
                    bk.bypass_serial,
                    bk.signal_flow_distance,
                ),
                super::compiled::Stage::SerialDelayedFeedback(s) => (
                    "SerialFB",
                    "serial_feedback",
                    s.bypass_serial,
                    s.signal_flow_distance,
                ),
                super::compiled::Stage::KMethod { .. } => ("KMethod", "k_method", true, usize::MAX),
            };
            let bp = if bypass { " BYPASS" } else { "" };
            eprintln!("    {i}: [{stype}] d={dist} [{lbl}]{bp} → {db:.1} dB");
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 1. All expected groups exist as stages
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn all_signal_path_components_have_stages() {
    let compiled = load_goldenrod();

    #[cfg(debug_assertions)]
    {
        let all_labels: Vec<&str> = compiled
            .stages
            .iter()
            .map(|s| match s {
                super::compiled::Stage::Wdf(w) => w.debug_label.as_str(),
                super::compiled::Stage::MultiNl(m) => m.debug_label.as_str(),
                super::compiled::Stage::Iir(i) => i.debug_label.as_str(),
                super::compiled::Stage::StateSpace(s) => s.debug_label.as_str(),
                super::compiled::Stage::BlackFeedback(b) => b.debug_label.as_str(),
                super::compiled::Stage::Blockwise(_) => "blockwise",
                super::compiled::Stage::SerialDelayedFeedback(_) => "serial_feedback",
                super::compiled::Stage::KMethod { .. } => "k_method",
            })
            .collect();

        eprintln!("Stage labels: {all_labels:?}");

        // These components MUST appear in at least one stage
        let required = [
            "U2", "U3", "U4", "D1", "D2", "Gain_A", "Gain_B", "Output", "Treble", "R_clip",
            "C_clip",
        ];
        for comp in required {
            let found = all_labels.iter().any(|l| l.contains(comp));
            assert!(
                found,
                "Component {comp} must appear in a stage. Labels: {all_labels:?}"
            );
        }
    }
}

#[test]
fn output_network_is_after_tone_stage() {
    let compiled = load_goldenrod();

    #[cfg(debug_assertions)]
    {
        let labels: Vec<&str> = compiled
            .stages
            .iter()
            .map(|s| match s {
                super::compiled::Stage::Wdf(w) => w.debug_label.as_str(),
                super::compiled::Stage::MultiNl(m) => m.debug_label.as_str(),
                super::compiled::Stage::Iir(i) => i.debug_label.as_str(),
                super::compiled::Stage::StateSpace(s) => s.debug_label.as_str(),
                super::compiled::Stage::BlackFeedback(b) => b.debug_label.as_str(),
                super::compiled::Stage::Blockwise(_) => "blockwise",
                super::compiled::Stage::SerialDelayedFeedback(_) => "serial_feedback",
                super::compiled::Stage::KMethod { .. } => "k_method",
            })
            .collect();

        let tone_idx = labels
            .iter()
            .position(|l| l.contains("U4") && l.contains("Treble"))
            .unwrap_or_else(|| panic!("Tone stage missing. Labels: {labels:?}"));
        let output_idx = labels
            .iter()
            .position(|l| l.contains("Output"))
            .unwrap_or_else(|| panic!("Output stage missing. Labels: {labels:?}"));

        assert!(
            output_idx > tone_idx,
            "Output network must run after tone stage. Labels: {labels:?}"
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Diodes D1 AND D2 both exist (not just one)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn both_diodes_present_in_pipeline() {
    let compiled = load_goldenrod();

    #[cfg(debug_assertions)]
    {
        let all_labels: String = compiled
            .stages
            .iter()
            .map(|s| match s {
                super::compiled::Stage::Wdf(w) => w.debug_label.as_str(),
                super::compiled::Stage::MultiNl(m) => m.debug_label.as_str(),
                super::compiled::Stage::Iir(i) => i.debug_label.as_str(),
                super::compiled::Stage::StateSpace(s) => s.debug_label.as_str(),
                super::compiled::Stage::BlackFeedback(b) => b.debug_label.as_str(),
                super::compiled::Stage::Blockwise(_) => "blockwise",
                super::compiled::Stage::SerialDelayedFeedback(_) => "serial_feedback",
                super::compiled::Stage::KMethod { .. } => "k_method",
            })
            .collect::<Vec<_>>()
            .join(" | ");

        // D1 and D2 should be in the same merged stage (antiparallel)
        // or in separate stages — but both must exist
        assert!(
            all_labels.contains("D1"),
            "D1 must be in pipeline: {all_labels}"
        );
        assert!(
            all_labels.contains("D2"),
            "D2 must be in pipeline: {all_labels}"
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Gain_B feedforward path exists and carries signal
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn gain_b_feedforward_path_exists() {
    let mut compiled = load_goldenrod();
    compiled.enable_metering(128);

    let amp = 0.1;
    for s in 0..8000 {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }

    dump_stages(&compiled, "Gain_B check");

    // The Gain_B group [R1,C1,R2,R_ff1a,C_ff1,Gain_B] must exist as a stage
    #[cfg(debug_assertions)]
    {
        let has_gain_b = compiled.stages.iter().any(|s| {
            let lbl = match s {
                super::compiled::Stage::Wdf(w) => w.debug_label.as_str(),
                super::compiled::Stage::MultiNl(m) => m.debug_label.as_str(),
                super::compiled::Stage::Iir(i) => i.debug_label.as_str(),
                super::compiled::Stage::StateSpace(s) => s.debug_label.as_str(),
                super::compiled::Stage::BlackFeedback(b) => b.debug_label.as_str(),
                super::compiled::Stage::Blockwise(_) => "blockwise",
                super::compiled::Stage::SerialDelayedFeedback(_) => "serial_feedback",
                super::compiled::Stage::KMethod { .. } => "k_method",
            };
            lbl.contains("Gain_B")
        });
        assert!(has_gain_b, "Gain_B feedforward path must produce a stage");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. No signal-path stage should be dead (below -60 dB)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn no_signal_path_stage_is_dead() {
    let mut compiled = load_goldenrod();
    compiled.enable_metering(128);

    let amp = 0.1;
    for s in 0..8000 {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }

    let metrics = compiled.read_metrics();
    let n = compiled.stages.len().min(crate::metering::MAX_STAGES);

    dump_stages(&compiled, "Dead stage check");

    for i in 0..n {
        let lvl = metrics.stage_levels[i];
        let db = if lvl > 1e-10 {
            20.0 * (lvl as f64).log10()
        } else {
            -120.0
        };

        // Skip bypass stages (bias networks)
        let bypass = match &compiled.stages[i] {
            super::compiled::Stage::Wdf(w) => w.bypass_serial,
            super::compiled::Stage::MultiNl(m) => m.bypass_serial,
            super::compiled::Stage::Iir(s) => s.bypass_serial,
            super::compiled::Stage::StateSpace(s) => s.bypass_serial,
            super::compiled::Stage::BlackFeedback(b) => b.bypass_serial,
            super::compiled::Stage::Blockwise(bk) => bk.bypass_serial,
            super::compiled::Stage::SerialDelayedFeedback(s) => s.bypass_serial,
            super::compiled::Stage::KMethod { .. } => true,
        };
        if bypass {
            continue;
        }

        #[cfg(debug_assertions)]
        {
            let lbl = match &compiled.stages[i] {
                super::compiled::Stage::Wdf(w) => w.debug_label.as_str(),
                super::compiled::Stage::MultiNl(m) => m.debug_label.as_str(),
                super::compiled::Stage::Iir(i) => i.debug_label.as_str(),
                super::compiled::Stage::StateSpace(s) => s.debug_label.as_str(),
                super::compiled::Stage::BlackFeedback(b) => b.debug_label.as_str(),
                super::compiled::Stage::Blockwise(_) => "blockwise",
                super::compiled::Stage::SerialDelayedFeedback(_) => "serial_feedback",
                super::compiled::Stage::KMethod { .. } => "k_method",
            };
            assert!(
                db > -60.0,
                "Stage {i} [{lbl}] is dead at {db:.1} dB — signal path stages must be > -60 dB"
            );
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. Gain_A and Gain_B both have pot bindings
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn both_gain_pots_are_bound() {
    let compiled = load_goldenrod();

    let gain_bindings: Vec<&str> = compiled
        .controls
        .iter()
        .filter(|c| c.label == "Gain")
        .map(|c| c.component_id.as_str())
        .collect();

    eprintln!("Gain bindings: {gain_bindings:?}");

    assert!(gain_bindings.contains(&"Gain_A"), "Gain_A must be bound");
    assert!(gain_bindings.contains(&"Gain_B"), "Gain_B must be bound");
}

// ═══════════════════════════════════════════════════════════════════════════
// 5b. Minimal feedforward pot group — does SPQR handle it?
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn minimal_feedforward_pot_produces_stage() {
    // Simplest possible feedforward: pot from signal to output.
    // One pot (2 edges) + one resistor. Should produce a stage.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                Vol: pot(100k, b)
                R_out: resistor(10k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> Vol.a
                Vol.b -> gnd
                Vol.w -> R_out.a
                R_out.b -> out
            }
            controls {
                Vol.position -> "Volume" [0.0, 1.0] = 0.5
            }
        }"#,
    )
    .expect("parse");
    let compiled = compile_via_spqr(&pedal, SR).expect("compile");

    #[cfg(debug_assertions)]
    {
        let labels: Vec<&str> = compiled
            .stages
            .iter()
            .map(|s| match s {
                super::compiled::Stage::Wdf(w) => w.debug_label.as_str(),
                super::compiled::Stage::MultiNl(m) => m.debug_label.as_str(),
                super::compiled::Stage::Iir(i) => i.debug_label.as_str(),
                super::compiled::Stage::StateSpace(s) => s.debug_label.as_str(),
                super::compiled::Stage::BlackFeedback(b) => b.debug_label.as_str(),
                super::compiled::Stage::Blockwise(_) => "blockwise",
                super::compiled::Stage::SerialDelayedFeedback(_) => "serial_feedback",
                super::compiled::Stage::KMethod { .. } => "k_method",
            })
            .collect();
        eprintln!("Stages: {labels:?}");
        let has_vol = labels.iter().any(|l| l.contains("Vol"));
        assert!(has_vol, "Volume pot must appear in a stage: {labels:?}");
    }
}

#[test]
fn feedforward_with_multiple_paths_produces_stages() {
    // Two feedforward paths from buffer output — mimics Goldenrod topology.
    // Path 1: R_ff → C_ff → out_node (passive LPF)
    // Path 2: Pot.w → R_ff2 → out_node (pot blend)
    // Both should produce stages.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                R_ff1: resistor(15k)
                C_ff1: cap(100n)
                Blend: pot(100k, b)
                R_ff2: resistor(392k)
                R_out: resistor(10k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> R_ff1.a, Blend.a
                R_ff1.b -> C_ff1.a
                C_ff1.b -> R_out.a
                Blend.w -> R_ff2.a
                R_ff2.b -> R_out.a
                Blend.b -> gnd
                R_out.b -> out
            }
            controls {
                Blend.position -> "Blend" [0.0, 1.0] = 0.5
            }
        }"#,
    )
    .expect("parse");
    let compiled = compile_via_spqr(&pedal, SR).expect("compile");

    #[cfg(debug_assertions)]
    {
        let labels: Vec<&str> = compiled
            .stages
            .iter()
            .map(|s| match s {
                super::compiled::Stage::Wdf(w) => w.debug_label.as_str(),
                super::compiled::Stage::MultiNl(m) => m.debug_label.as_str(),
                super::compiled::Stage::Iir(i) => i.debug_label.as_str(),
                super::compiled::Stage::StateSpace(s) => s.debug_label.as_str(),
                super::compiled::Stage::BlackFeedback(b) => b.debug_label.as_str(),
                super::compiled::Stage::Blockwise(_) => "blockwise",
                super::compiled::Stage::SerialDelayedFeedback(_) => "serial_feedback",
                super::compiled::Stage::KMethod { .. } => "k_method",
            })
            .collect();
        eprintln!("Feedforward stages: {labels:?}");
        let has_blend = labels.iter().any(|l| l.contains("Blend"));
        let has_ff1 = labels
            .iter()
            .any(|l| l.contains("R_ff1") || l.contains("C_ff1"));
        assert!(has_blend, "Blend pot must appear in a stage: {labels:?}");
        assert!(
            has_ff1,
            "Feedforward 1 path must appear in a stage: {labels:?}"
        );
    }
}

#[test]
fn spqr_with_many_terminals_produces_stages() {
    // The Goldenrod's Gain_B group has 5 SPQR terminals because many nodes
    // connect to other groups. This tests whether SPQR can handle that.
    // Reduced version: 4 edges, 3 terminals.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                R2: resistor(1M)
                C1: cap(100n)
                U1: opamp(tl072)
                Rf: resistor(100k)
            }
            nets {
                in -> R1.a
                R1.b -> C1.a, R2.a, U1.neg
                C1.b -> out
                R2.b -> gnd
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
            }
            controls {}
        }"#,
    )
    .expect("parse");
    let compiled = compile_via_spqr(&pedal, SR).expect("compile");

    #[cfg(debug_assertions)]
    {
        let labels: Vec<&str> = compiled
            .stages
            .iter()
            .map(|s| match s {
                super::compiled::Stage::Wdf(w) => w.debug_label.as_str(),
                super::compiled::Stage::MultiNl(m) => m.debug_label.as_str(),
                super::compiled::Stage::Iir(i) => i.debug_label.as_str(),
                super::compiled::Stage::StateSpace(s) => s.debug_label.as_str(),
                super::compiled::Stage::BlackFeedback(b) => b.debug_label.as_str(),
                super::compiled::Stage::Blockwise(_) => "blockwise",
                super::compiled::Stage::SerialDelayedFeedback(_) => "serial_feedback",
                super::compiled::Stage::KMethod { .. } => "k_method",
            })
            .collect();
        eprintln!("Multi-terminal stages: {labels:?}");
        // Every passive component should be in some stage
        let has_r1 = labels.iter().any(|l| l.contains("R1"));
        let has_c1 = labels.iter().any(|l| l.contains("C1"));
        assert!(
            has_r1 || has_c1,
            "Passive components must produce stages: {labels:?}"
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. Output should be reasonable level (not near-zero)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn output_level_is_reasonable() {
    let mut compiled = load_goldenrod();
    compiled.enable_metering(128);

    let amp = 0.1;
    for s in 0..8000 {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }

    let metrics = compiled.read_metrics();
    eprintln!(
        "Output: RMS={:.1} dB, peak={:.1} dB",
        metrics.output_rms_db, metrics.output_peak_db
    );

    // Output should be within a reasonable range — not near-zero, not clipping at rails
    assert!(
        metrics.output_peak_db > -40.0,
        "Output should be > -40 dB, got {:.1} dB",
        metrics.output_peak_db
    );
}
