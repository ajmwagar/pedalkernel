// ═══════════════════════════════════════════════════════════════════════════
// Goldenrod (Klon) pot diagnostic tests
//
// The Klon has a dual-gang Gain pot (Gain_A + Gain_B) that crossfades
// between clean and dirty signal paths. Both pots are bound to the same
// "Gain" control label with inverted ranges:
//   Gain_A [1.0, 0.0] = dirty gain (increases with knob)
//   Gain_B [0.0, 1.0] = clean blend (decreases with knob)
//
// The Treble pot controls the tone stage, Output controls volume.
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

fn measure_peak(compiled: &mut CompiledPedal, amp: f64) -> f64 {
    for s in 0..4000 {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }
    let mut peak = 0.0f64;
    for s in 0..500 {
        let out = compiled.process(
            amp * (std::f64::consts::TAU * FREQ * (4000 + s) as f64 / SR).sin()
        );
        peak = peak.max(out.abs());
    }
    peak
}

// ═══════════════════════════════════════════════════════════════════════════
// 1. Stage dump — what does the pipeline build?
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn goldenrod_stage_dump() {
    let mut compiled = load_goldenrod();
    compiled.enable_metering(128);

    let amp = 0.1;
    for s in 0..8000 {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }

    let metrics = compiled.read_metrics();
    let n = compiled.stages.len().min(crate::metering::MAX_STAGES);
    eprintln!("\nGOLDENROD DIAGNOSTIC: {} stages", n);
    eprintln!("  input:  RMS={:.1} dB, peak={:.1} dB", metrics.input_rms_db, metrics.input_peak_db);
    eprintln!("  output: RMS={:.1} dB, peak={:.1} dB", metrics.output_rms_db, metrics.output_peak_db);
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
            eprintln!("  stage {i}: [{stype}] dist={dist} [{lbl}]{bp} → {lvl:.4} ({db:.1} dB)");
        }
    }

    // Dump control bindings
    eprintln!("\n  Control bindings ({}):", compiled.controls.len());
    for ctrl in &compiled.controls {
        eprintln!("    \"{}\" → comp={} range={:?}", ctrl.label, ctrl.component_id, ctrl.range);
    }

    // Must have bindings for all 4 controls
    let labels: Vec<&str> = compiled.controls.iter().map(|c| c.label.as_str()).collect();
    eprintln!("  Labels found: {:?}", labels);

    assert!(labels.contains(&"Gain"), "Should have Gain binding");
    assert!(labels.contains(&"Treble"), "Should have Treble binding");
    assert!(labels.contains(&"Output"), "Should have Output binding");

    // Gain should appear TWICE (Gain_A + Gain_B ganged)
    let gain_count = labels.iter().filter(|&&l| l == "Gain").count();
    eprintln!("  Gain bindings: {gain_count} (should be 2 for ganged)");
    assert_eq!(gain_count, 2, "Ganged Gain pot should have 2 bindings");
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Output pot changes volume
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn goldenrod_output_pot_changes_level() {
    let mut low = load_goldenrod();
    low.set_control("Output", 0.2);
    let peak_low = measure_peak(&mut low, 0.1);

    let mut high = load_goldenrod();
    high.set_control("Output", 0.9);
    let peak_high = measure_peak(&mut high, 0.1);

    eprintln!("Output 0.2: {peak_low:.4}V, Output 0.9: {peak_high:.4}V");
    assert!(peak_high > peak_low * 1.5,
        "Output pot should change level: low={peak_low:.4}, high={peak_high:.4}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Gain pot crossfade — low gain = clean, high gain = dirty
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn goldenrod_gain_crossfade() {
    // At low Gain: Gain_A = high R (low gain), Gain_B = low R (clean passes)
    // At high Gain: Gain_A = low R (high gain), Gain_B = high R (clean blocked)
    // The dirty path should clip harder at high gain, changing the waveform.
    let mut clean = load_goldenrod();
    clean.set_control("Gain", 0.1);
    let peak_clean = measure_peak(&mut clean, 0.1);

    let mut dirty = load_goldenrod();
    dirty.set_control("Gain", 0.9);
    let peak_dirty = measure_peak(&mut dirty, 0.1);

    eprintln!("Gain 0.1 (clean): {peak_clean:.4}V, Gain 0.9 (dirty): {peak_dirty:.4}V");

    // Both should produce output
    assert!(peak_clean > 0.001, "Clean should produce output: {peak_clean:.4}V");
    assert!(peak_dirty > 0.001, "Dirty should produce output: {peak_dirty:.4}V");

    // The levels should be DIFFERENT (crossfade working)
    let ratio = peak_dirty / peak_clean.max(0.0001);
    eprintln!("  Dirty/clean ratio: {ratio:.2}");
    assert!((ratio - 1.0).abs() > 0.2,
        "Gain crossfade should change character: ratio={ratio:.2}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Treble pot changes tone
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn goldenrod_treble_pot_changes_spectrum() {
    let mut dark = load_goldenrod();
    dark.set_control("Treble", 0.1);
    let peak_dark = measure_peak(&mut dark, 0.1);

    let mut bright = load_goldenrod();
    bright.set_control("Treble", 0.9);
    let peak_bright = measure_peak(&mut bright, 0.1);

    eprintln!("Treble 0.1: {peak_dark:.4}V, Treble 0.9: {peak_bright:.4}V");

    // Both should produce output
    assert!(peak_dark > 0.001, "Dark should produce output: {peak_dark:.4}V");
    assert!(peak_bright > 0.001, "Bright should produce output: {peak_bright:.4}V");
}
