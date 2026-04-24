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
use crate::PedalProcessor;

const SR: f64 = 48000.0;
const FREQ: f64 = 440.0;

fn load_goldenrod() -> CompiledPedal {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/goldenrod.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read goldenrod.pedal");
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
        let db = if lvl > 1e-10 { 20.0 * (lvl as f64).log10() } else { -120.0 };
        #[cfg(debug_assertions)]
        {
            let (stype, lbl, bypass, dist) = match &compiled.stages[i] {
                super::compiled::Stage::Wdf(w) => ("Wdf", &w.debug_label, w.bypass_serial, w.signal_flow_distance),
                super::compiled::Stage::MultiNl(m) => ("MNL", &m.debug_label, m.bypass_serial, m.signal_flow_distance),
                super::compiled::Stage::Iir(s) => ("Iir", &s.debug_label, s.bypass_serial, s.signal_flow_distance),
                super::compiled::Stage::StateSpace(s) => ("SS", &s.debug_label, s.bypass_serial, s.signal_flow_distance),
                super::compiled::Stage::BlackFeedback(b) => ("BF", &b.debug_label, b.bypass_serial, b.signal_flow_distance),
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
        let all_labels: Vec<&str> = compiled.stages.iter().map(|s| match s {
            super::compiled::Stage::Wdf(w) => w.debug_label.as_str(),
            super::compiled::Stage::MultiNl(m) => m.debug_label.as_str(),
            super::compiled::Stage::Iir(i) => i.debug_label.as_str(),
            super::compiled::Stage::StateSpace(s) => s.debug_label.as_str(),
            super::compiled::Stage::BlackFeedback(b) => b.debug_label.as_str(),
        }).collect();

        eprintln!("Stage labels: {all_labels:?}");

        // These components MUST appear in at least one stage
        let required = ["U2", "U3", "U4", "D1", "D2", "Gain_A", "Gain_B",
                        "Output", "Treble", "R_clip", "C_clip"];
        for comp in required {
            let found = all_labels.iter().any(|l| l.contains(comp));
            assert!(found, "Component {comp} must appear in a stage. Labels: {all_labels:?}");
        }
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
        let all_labels: String = compiled.stages.iter().map(|s| match s {
            super::compiled::Stage::Wdf(w) => w.debug_label.as_str(),
            super::compiled::Stage::MultiNl(m) => m.debug_label.as_str(),
            super::compiled::Stage::Iir(i) => i.debug_label.as_str(),
            super::compiled::Stage::StateSpace(s) => s.debug_label.as_str(),
            super::compiled::Stage::BlackFeedback(b) => b.debug_label.as_str(),
        }).collect::<Vec<_>>().join(" | ");

        // D1 and D2 should be in the same merged stage (antiparallel)
        // or in separate stages — but both must exist
        assert!(all_labels.contains("D1"), "D1 must be in pipeline: {all_labels}");
        assert!(all_labels.contains("D2"), "D2 must be in pipeline: {all_labels}");
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
                super::compiled::Stage::Wdf(w) => &w.debug_label,
                super::compiled::Stage::MultiNl(m) => &m.debug_label,
                super::compiled::Stage::Iir(i) => &i.debug_label,
                super::compiled::Stage::StateSpace(s) => &s.debug_label,
                super::compiled::Stage::BlackFeedback(b) => &b.debug_label,
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
        let db = if lvl > 1e-10 { 20.0 * (lvl as f64).log10() } else { -120.0 };

        // Skip bypass stages (bias networks)
        let bypass = match &compiled.stages[i] {
            super::compiled::Stage::Wdf(w) => w.bypass_serial,
            super::compiled::Stage::MultiNl(m) => m.bypass_serial,
            super::compiled::Stage::Iir(s) => s.bypass_serial,
            super::compiled::Stage::StateSpace(s) => s.bypass_serial,
            super::compiled::Stage::BlackFeedback(b) => b.bypass_serial,
        };
        if bypass { continue; }

        #[cfg(debug_assertions)]
        {
            let lbl = match &compiled.stages[i] {
                super::compiled::Stage::Wdf(w) => &w.debug_label,
                super::compiled::Stage::MultiNl(m) => &m.debug_label,
                super::compiled::Stage::Iir(i) => &i.debug_label,
                super::compiled::Stage::StateSpace(s) => &s.debug_label,
                super::compiled::Stage::BlackFeedback(b) => &b.debug_label,
            };
            assert!(db > -60.0,
                "Stage {i} [{lbl}] is dead at {db:.1} dB — signal path stages must be > -60 dB");
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. Gain_A and Gain_B both have pot bindings
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn both_gain_pots_are_bound() {
    let compiled = load_goldenrod();

    let gain_bindings: Vec<&str> = compiled.controls.iter()
        .filter(|c| c.label == "Gain")
        .map(|c| c.component_id.as_str())
        .collect();

    eprintln!("Gain bindings: {gain_bindings:?}");

    assert!(gain_bindings.contains(&"Gain_A"), "Gain_A must be bound");
    assert!(gain_bindings.contains(&"Gain_B"), "Gain_B must be bound");
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
    eprintln!("Output: RMS={:.1} dB, peak={:.1} dB", metrics.output_rms_db, metrics.output_peak_db);

    // Output should be within a reasonable range — not near-zero, not clipping at rails
    assert!(metrics.output_peak_db > -40.0,
        "Output should be > -40 dB, got {:.1} dB", metrics.output_peak_db);
}
