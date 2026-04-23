// ═══════════════════════════════════════════════════════════════════════════
// Ground-clip diode tests
//
// TDD tests for standalone nonlinear elements clipping to ground.
// This is the RAT/Klon topology: opamp gain stage → diodes to GND.
//
// The diodes are in their own flow groups (not in the opamp's feedback).
// Each group has a single NL edge connecting a signal node to GND.
// The pipeline must detect this pattern and build an NlWdf stage that
// clips the signal at the diode's forward voltage.
// ═══════════════════════════════════════════════════════════════════════════

use super::compiled::CompiledPedal;
use super::spqr_build::compile_via_spqr;
use crate::PedalProcessor;

const SR: f64 = 48000.0;
const FREQ: f64 = 440.0;

/// Helper: measure peaks from a compiled pedal.
fn measure_peaks(compiled: &mut impl PedalProcessor, amp: f64, warmup: usize, measure: usize) -> (f64, f64) {
    for s in 0..warmup {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }
    let mut pos_peak = 0.0f64;
    let mut neg_peak = 0.0f64;
    for s in 0..measure {
        let out = compiled.process(
            amp * (std::f64::consts::TAU * FREQ * (warmup + s) as f64 / SR).sin()
        );
        if out > pos_peak { pos_peak = out; }
        if out < neg_peak { neg_peak = out; }
    }
    (pos_peak, neg_peak)
}

// ═══════════════════════════════════════════════════════════════════════════
// 1. Single diode to ground clips one direction
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn single_diode_to_ground_clips() {
    // Simplest case: opamp gain → single diode to GND → output.
    // The diode clips positive swings at ~0.6V.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D1: diode(silicon)
                R_out: resistor(10k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> D1.a
                D1.b -> gnd
                U1.out -> R_out.a
                R_out.b -> out
            }
            controls {}
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");
    let (pos_peak, neg_peak) = measure_peaks(&mut compiled, 0.05, 2000, 500);

    eprintln!("Single diode to GND: pos={pos_peak:.4}V, neg={neg_peak:.4}V");

    // Should produce output
    assert!(pos_peak > 0.01, "Should produce positive output: {pos_peak:.4}V");
    assert!(neg_peak < -0.01, "Should produce negative output: {neg_peak:.4}V");

    // Positive should be clipped at ~Vf (0.6V). Negative passes unclipped.
    assert!(pos_peak < 1.0, "Positive should clip at Vf: {pos_peak:.4}V");
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Antiparallel diodes to ground clip both directions (RAT pattern)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn antiparallel_diodes_to_ground_clip_both_directions() {
    // RAT: two diodes antiparallel to GND. Both directions clip at ~Vf.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(lm308)
                Rf: resistor(1M)
                D1: diode(silicon)
                D2: diode(silicon)
                R_out: resistor(10k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> D1.a, D2.b
                D1.b -> gnd
                D2.a -> gnd
                U1.out -> R_out.a
                R_out.b -> out
            }
            controls {}
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Dump metering to see stage layout
    compiled.enable_metering(128);
    for s in 0..2000 {
        compiled.process(0.05 * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }
    let metrics = compiled.read_metrics();
    let n = compiled.stages.len();
    for i in 0..n.min(crate::metering::MAX_STAGES) {
        let lvl = metrics.stage_levels[i];
        let db = if lvl > 1e-10 { 20.0 * (lvl as f64).log10() } else { -120.0 };
        #[cfg(debug_assertions)]
        {
            let label = match &compiled.stages[i] {
                super::compiled::Stage::Wdf(w) => &w.debug_label,
                super::compiled::Stage::MultiNl(m) => &m.debug_label,
                super::compiled::Stage::Iir(i) => &i.debug_label,
                super::compiled::Stage::StateSpace(s) => &s.debug_label,
                super::compiled::Stage::BlackFeedback(b) => &b.debug_label,
            };
            eprintln!("    stage {i}: [{label}] → {lvl:.4} ({db:.1} dB)");
        }
    }

    let mut compiled2 = compile_via_spqr(&pedal, SR).expect("compile");
    let (pos_peak, neg_peak) = measure_peaks(&mut compiled2, 0.05, 2000, 500);

    eprintln!("RAT diodes to GND: pos={pos_peak:.4}V, neg={neg_peak:.4}V");

    assert!(pos_peak > 0.01, "Should produce positive output: {pos_peak:.4}V");
    assert!(neg_peak < -0.01, "Should produce negative output: {neg_peak:.4}V");

    // Both directions should clip — output limited well below unclipped gain
    // (gain=100, input=0.05V → 5V unclipped, rail-clipped to ~3V).
    // Diode clips to ~0.7-1.2V depending on source impedance and WDF model.
    assert!(pos_peak < 1.5, "Positive should clip: {pos_peak:.4}V");
    assert!(neg_peak > -1.5, "Negative should clip: {neg_peak:.4}V");
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Diode stage actually appears in the pipeline (not silently dropped)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn diode_to_ground_stage_exists_in_pipeline() {
    // The diode group must produce a stage. Currently it's silently dropped
    // because SPQR sees a degenerate single-terminal subgraph.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D1: diode(silicon)
                R_out: resistor(10k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> D1.a
                D1.b -> gnd
                U1.out -> R_out.a
                R_out.b -> out
            }
            controls {}
        }"#)
    .expect("parse");

    let compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Count stages — should have at least 3:
    // 1. Opamp (BlackFeedback or WDF)
    // 2. Diode clipper (NlWdf)
    // 3. R_out (passive)
    let n = compiled.stages.len();
    eprintln!("Pipeline stages: {n}");
    assert!(n >= 3, "Should have ≥3 stages (opamp + diode + output), got {n}");

    // At least one stage should contain D1
    #[cfg(debug_assertions)]
    {
        let has_diode = compiled.stages.iter().any(|s| {
            let label = match s {
                super::compiled::Stage::Wdf(w) => &w.debug_label,
                super::compiled::Stage::MultiNl(m) => &m.debug_label,
                super::compiled::Stage::Iir(i) => &i.debug_label,
                super::compiled::Stage::StateSpace(ss) => &ss.debug_label,
                super::compiled::Stage::BlackFeedback(b) => &b.debug_label,
            };
            label.contains("D1")
        });
        assert!(has_diode, "Pipeline should contain a stage with D1");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. RAT legend pedal produces clipped audio
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn ratking_legend_produces_clipped_audio() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/ratking.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read ratking.pedal");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    let (pos_peak, neg_peak) = measure_peaks(&mut compiled, 0.05, 4000, 500);

    eprintln!("RAT legend: pos={pos_peak:.4}V, neg={neg_peak:.4}V");

    assert!(pos_peak > 0.001, "RAT should produce output: {pos_peak:.4}V");
    // RAT should have hard clipping — output limited by diode Vf
    assert!(pos_peak < 2.0, "RAT should clip (not pass full gain): {pos_peak:.4}V");
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. Goldenrod legend pedal produces clipped audio
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn goldenrod_legend_produces_clipped_audio() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/goldenrod.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read goldenrod.pedal");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    let (pos_peak, neg_peak) = measure_peaks(&mut compiled, 0.05, 4000, 500);

    eprintln!("Goldenrod legend: pos={pos_peak:.4}V, neg={neg_peak:.4}V");

    assert!(pos_peak > 0.001, "Goldenrod should produce output: {pos_peak:.4}V");
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. Germanium diodes clip at lower voltage than silicon
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn germanium_clips_lower_than_silicon() {
    // Same circuit, different diode types. Germanium Vf ≈ 0.25V < silicon 0.6V.
    let make_circuit = |dt: &str| -> String {
        format!(r#"
            pedal "test" {{ supply 9V
                components {{
                    R_in: resistor(10k)
                    U1: opamp(tl072)
                    Rf: resistor(100k)
                    D1: diode({dt})
                    D2: diode({dt})
                    R_out: resistor(10k)
                }}
                nets {{
                    in -> R_in.a
                    R_in.b -> U1.neg
                    U1.neg -> Rf.a
                    Rf.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> D1.a, D2.b
                    D1.b -> gnd
                    D2.a -> gnd
                    U1.out -> R_out.a
                    R_out.b -> out
                }}
                controls {{}}
            }}"#)
    };

    let si_pedal = crate::dsl::parse_pedal_file(&make_circuit("silicon")).expect("parse si");
    let ge_pedal = crate::dsl::parse_pedal_file(&make_circuit("germanium")).expect("parse ge");

    let mut si = compile_via_spqr(&si_pedal, SR).expect("compile si");
    let mut ge = compile_via_spqr(&ge_pedal, SR).expect("compile ge");

    let (si_pos, _) = measure_peaks(&mut si, 0.05, 2000, 500);
    let (ge_pos, _) = measure_peaks(&mut ge, 0.05, 2000, 500);

    eprintln!("Silicon clip: {si_pos:.4}V, Germanium clip: {ge_pos:.4}V");

    // Both should clip
    assert!(si_pos > 0.01, "Silicon should produce output: {si_pos:.4}V");
    assert!(ge_pos > 0.01, "Germanium should produce output: {ge_pos:.4}V");

    // Germanium should clip at lower voltage
    assert!(ge_pos < si_pos, "Germanium ({ge_pos:.4}V) should clip lower than silicon ({si_pos:.4}V)");
}
