//! Comprehensive control binding regression tests.
//!
//! Covers the 6-step binding pipeline:
//!   1. Switch/RotarySwitch
//!   2. Op-amp feedback pot
//!   3. Pot in WDF/coupled-BJT/multi-NL stage tree
//!   4. Sidechain pot
//!   5. LFO / delay / BBD net connection
//!   6. Label-based heuristic fallback
//!
//! Three test categories:
//!   - Structural: compile + check debug_dump() for correct ControlTarget
//!   - Behavioral: compile + process audio, verify controls change output
//!   - Regression: real example pedals, end-to-end

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::compiler::{compile_pedal, CompiledPedal};
use pedalkernel::dsl::parse_pedal_file;
use std::path::{Path, PathBuf};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn compile_src(src: &str) -> CompiledPedal {
    let pedal = parse_pedal_file(src).expect("failed to parse pedal source");
    compile_pedal(&pedal, SAMPLE_RATE).expect("failed to compile pedal")
}

fn dump(compiled: &CompiledPedal) -> String {
    compiled.debug_dump()
}

fn load_example(filename: &str) -> PathBuf {
    let examples_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
    find_file_recursive(&examples_dir, filename)
        .unwrap_or_else(|| panic!("example file not found: {filename}"))
}

fn find_file_recursive(dir: &Path, filename: &str) -> Option<PathBuf> {
    if let Ok(entries) = std::fs::read_dir(dir) {
        for entry in entries.flatten() {
            let p = entry.path();
            if p.is_dir() {
                if let Some(found) = find_file_recursive(&p, filename) {
                    return Some(found);
                }
            } else if p.file_name().map_or(false, |n| n == filename) {
                return Some(p);
            }
        }
    }
    None
}

fn compile_example(filename: &str) -> CompiledPedal {
    let path = load_example(filename);
    let src = std::fs::read_to_string(&path).unwrap();
    let pedal = parse_pedal_file(&src).expect("parse example pedal");
    compile_pedal(&pedal, SAMPLE_RATE).expect("compile example pedal")
}

// ═══════════════════════════════════════════════════════════════════════════
// Section 1: Binding Resolution (Structural)
// ═══════════════════════════════════════════════════════════════════════════

// ---------------------------------------------------------------------------
// Step 5: LFO / BBD / DelayLine net connections
// ---------------------------------------------------------------------------

#[test]
fn pot_wiper_to_lfo_rate() {
    let src = r#"
        pedal "LFO Rate Wiper" {
            components {
                R1: resistor(10k)
                LFO1: lfo(sine, 100k, 220n)
                Speed: pot(500k)
            }
            nets {
                in -> R1.a
                R1.b -> out
                Speed.wiper -> LFO1.rate
            }
            controls {
                Speed.position -> "Speed" [0.0, 1.0] = 0.5
            }
        }
    "#;
    let compiled = compile_src(src);
    let d = dump(&compiled);
    assert!(
        d.contains("Speed -> LfoRate(0)"),
        "Pot.wiper -> LFO.rate should bind to LfoRate(0); controls:\n{d}"
    );
}

#[test]
fn pot_b_to_lfo_rate() {
    let src = r#"
        pedal "LFO Rate B" {
            components {
                R1: resistor(10k)
                LFO1: lfo(sine, 100k, 220n)
                Speed: pot(500k)
            }
            nets {
                in -> R1.a
                R1.b -> out
                Speed.b -> LFO1.rate
            }
            controls {
                Speed.position -> "Speed" [0.0, 1.0] = 0.5
            }
        }
    "#;
    let compiled = compile_src(src);
    let d = dump(&compiled);
    assert!(
        d.contains("Speed -> LfoRate(0)"),
        "Pot.b -> LFO.rate should bind to LfoRate(0); controls:\n{d}"
    );
}

#[test]
fn pot_a_to_lfo_rate() {
    let src = r#"
        pedal "LFO Rate A" {
            components {
                R1: resistor(10k)
                LFO1: lfo(sine, 100k, 220n)
                Speed: pot(500k)
            }
            nets {
                in -> R1.a
                R1.b -> out
                Speed.a -> LFO1.rate
            }
            controls {
                Speed.position -> "Speed" [0.0, 1.0] = 0.5
            }
        }
    "#;
    let compiled = compile_src(src);
    let d = dump(&compiled);
    assert!(
        d.contains("Speed -> LfoRate(0)"),
        "Pot.a -> LFO.rate should bind to LfoRate(0); controls:\n{d}"
    );
}

#[test]
fn pot_wiper_to_bbd_clock() {
    let src = r#"
        pedal "BBD Clock Wiper" {
            components {
                R1: resistor(10k)
                BBD1: bbd(mn3005)
                Time: pot(500k)
                C1: cap(100n)
            }
            nets {
                in -> R1.a
                R1.b -> BBD1.in
                Time.wiper -> BBD1.clock
                BBD1.out -> C1.a
                C1.b -> out
            }
            controls {
                Time.position -> "Time" [0.0, 1.0] = 0.5
            }
        }
    "#;
    let compiled = compile_src(src);
    let d = dump(&compiled);
    assert!(
        d.contains("Time -> BbdClockRate(0)"),
        "Pot.wiper -> BBD.clock should bind to BbdClockRate(0); controls:\n{d}"
    );
}

#[test]
fn pot_b_to_bbd_clock() {
    let src = r#"
        pedal "BBD Clock B" {
            components {
                R1: resistor(10k)
                BBD1: bbd(mn3005)
                Time: pot(500k)
                C1: cap(100n)
            }
            nets {
                in -> R1.a
                R1.b -> BBD1.in
                Time.b -> BBD1.clock
                BBD1.out -> C1.a
                C1.b -> out
            }
            controls {
                Time.position -> "Time" [0.0, 1.0] = 0.5
            }
        }
    "#;
    let compiled = compile_src(src);
    let d = dump(&compiled);
    assert!(
        d.contains("Time -> BbdClockRate(0)"),
        "Pot.b -> BBD.clock should bind to BbdClockRate(0); controls:\n{d}"
    );
}

#[test]
fn bbd_out_to_pot_detects_feedback() {
    // When a pot is wired from BBD.out, it may end up in a WDF stage tree
    // (step 3 priority) or detected as BBD feedback (step 5). Either is
    // valid — the important thing is it binds to a meaningful target.
    let src = r#"
        pedal "BBD Feedback" {
            components {
                R1: resistor(10k)
                BBD1: bbd(mn3005)
                Repeats: pot(100k)
                R2: resistor(47k)
                C1: cap(100n)
                C2: cap(100n)
            }
            nets {
                in -> R1.a
                R1.b -> BBD1.in
                BBD1.out -> Repeats.a
                Repeats.b -> R2.a
                R2.b -> C1.a
                C1.b -> BBD1.in
                BBD1.out -> C2.a
                C2.b -> out
            }
            controls {
                Repeats.position -> "Repeats" [0.0, 1.0] = 0.3
            }
        }
    "#;
    let compiled = compile_src(src);
    let d = dump(&compiled);
    assert!(
        d.contains("Repeats -> BbdFeedback(0)") || d.contains("Repeats -> PotInStage("),
        "BBD.out -> Pot.a should bind to BbdFeedback or PotInStage, not PreGain; controls:\n{d}"
    );
    // Should bind to a specific target, not a generic fallback
    assert!(
        d.contains("Repeats -> BbdFeedback(0)") || d.contains("Repeats -> PotInStage("),
        "BBD feedback pot should bind to BbdFeedback or PotInStage; controls:\n{d}"
    );
}

#[test]
fn pot_to_delay_time() {
    let src = r#"
        pedal "Delay Time" {
            components {
                R1: resistor(10k)
                DL1: delay_line(1ms, 1200ms)
                Time: pot(500k)
                C1: cap(100n)
            }
            nets {
                in -> DL1.input
                Time.wiper -> DL1.delay_time
                DL1.output -> C1.a
                C1.b -> R1.a
                R1.b -> out
            }
            controls {
                Time.position -> "Time" [0.0, 1.0] = 0.5
            }
        }
    "#;
    let compiled = compile_src(src);
    let d = dump(&compiled);
    assert!(
        d.contains("Time -> DelayTime(0)"),
        "Pot.wiper -> DelayLine.delay_time should bind to DelayTime(0); controls:\n{d}"
    );
}

#[test]
fn pot_to_delay_feedback() {
    let src = r#"
        pedal "Delay Feedback" {
            components {
                R1: resistor(10k)
                DL1: delay_line(1ms, 1200ms)
                Fbk: pot(100k)
                C1: cap(100n)
            }
            nets {
                in -> DL1.input
                Fbk.wiper -> DL1.feedback
                DL1.output -> C1.a
                C1.b -> R1.a
                R1.b -> out
            }
            controls {
                Fbk.position -> "Feedback" [0.0, 1.0] = 0.3
            }
        }
    "#;
    let compiled = compile_src(src);
    let d = dump(&compiled);
    assert!(
        d.contains("Feedback -> DelayFeedback(0)"),
        "Pot.wiper -> DelayLine.feedback should bind to DelayFeedback(0); controls:\n{d}"
    );
}

// ---------------------------------------------------------------------------
// Step 6: Label-based heuristic fallback
// ---------------------------------------------------------------------------

#[test]
fn label_gain_pot_in_circuit() {
    // Pot physically in signal path → PotInStage (no label heuristic)
    let src = r#"
        pedal "Gain In Circuit" {
            components {
                R1: resistor(10k)
                Drive: pot(500k)
            }
            nets {
                in -> R1.a
                R1.b -> Drive.a
                Drive.b -> out
            }
            controls {
                Drive.position -> "Drive" [0.0, 1.0] = 0.5
            }
        }
    "#;
    let compiled = compile_src(src);
    let d = dump(&compiled);
    assert!(
        d.contains("Drive -> PotInStage(0)"),
        "Pot physically in circuit should bind to PotInStage; controls:\n{d}"
    );
}

#[test]
fn label_volume_pot_in_circuit() {
    // Pot physically in signal path → PotInStage (not OutputGain)
    let src = r#"
        pedal "Volume In Circuit" {
            components {
                R1: resistor(10k)
                Volume: pot(500k)
            }
            nets {
                in -> R1.a
                R1.b -> Volume.a
                Volume.b -> out
            }
            controls {
                Volume.position -> "Volume" [0.0, 1.0] = 0.5
            }
        }
    "#;
    let compiled = compile_src(src);
    let d = dump(&compiled);
    assert!(
        d.contains("Volume -> PotInStage(0)"),
        "Pot physically in circuit should bind to PotInStage; controls:\n{d}"
    );
}

#[test]
fn disconnected_pot_falls_to_pot_in_stage() {
    // Pot NOT in signal path and no net connection → PotInStage(0) fallback with warning
    let src = r#"
        pedal "Disconnected Pot" {
            components {
                R1: resistor(10k)
                BBD1: bbd(mn3005)
                Delay: pot(500k)
                C1: cap(100n)
            }
            nets {
                in -> R1.a
                R1.b -> BBD1.in
                BBD1.out -> C1.a
                C1.b -> out
            }
            controls {
                Delay.position -> "Delay" [0.0, 1.0] = 0.5
            }
        }
    "#;
    let compiled = compile_src(src);
    let d = dump(&compiled);
    // Pot not wired to BBD.clock via nets → no binding → PotInStage(0) harmless fallback
    assert!(
        d.contains("Delay -> PotInStage(0)"),
        "Disconnected pot should fall through to PotInStage(0); controls:\n{d}"
    );
}

#[test]
fn bbd_clock_net_binding() {
    // Pot wired to BBD.clock via net → BbdClockRate
    let src = r#"
        pedal "BBD Clock Net" {
            components {
                R1: resistor(10k)
                BBD1: bbd(mn3005)
                Time: pot(500k)
                C1: cap(100n)
            }
            nets {
                in -> R1.a
                R1.b -> BBD1.in
                BBD1.out -> C1.a
                C1.b -> out
                Time.wiper -> BBD1.clock
            }
            controls {
                Time.position -> "Time" [0.0, 1.0] = 0.5
            }
        }
    "#;
    let compiled = compile_src(src);
    let d = dump(&compiled);
    assert!(
        d.contains("Time -> BbdClockRate(0)"),
        "Pot wired to BBD.clock should bind to BbdClockRate(0); controls:\n{d}"
    );
}

#[test]
fn delay_line_time_net_binding() {
    // Pot wired to DelayLine.delay_time via net → DelayTime
    let src = r#"
        pedal "DelayLine Time Net" {
            components {
                R1: resistor(10k)
                DL1: delay_line(1ms, 1200ms)
                Time: pot(500k)
                C1: cap(100n)
            }
            nets {
                in -> DL1.input
                DL1.output -> C1.a
                C1.b -> R1.a
                R1.b -> out
                Time.wiper -> DL1.delay_time
            }
            controls {
                Time.position -> "Time" [0.0, 1.0] = 0.5
            }
        }
    "#;
    let compiled = compile_src(src);
    let d = dump(&compiled);
    assert!(
        d.contains("Time -> DelayTime(0)"),
        "Pot wired to DL.delay_time should bind to DelayTime(0); controls:\n{d}"
    );
}

#[test]
fn delay_line_feedback_net_binding() {
    // Pot wired to DelayLine.feedback via net → DelayFeedback
    let src = r#"
        pedal "DelayLine Feedback Net" {
            components {
                R1: resistor(10k)
                DL1: delay_line(1ms, 1200ms)
                Fbk: pot(100k)
                C1: cap(100n)
            }
            nets {
                in -> DL1.input
                DL1.output -> C1.a
                C1.b -> R1.a
                R1.b -> out
                Fbk.wiper -> DL1.feedback
            }
            controls {
                Fbk.position -> "Feedback" [0.0, 1.0] = 0.3
            }
        }
    "#;
    let compiled = compile_src(src);
    let d = dump(&compiled);
    assert!(
        d.contains("Feedback -> DelayFeedback(0)"),
        "Pot wired to DL.feedback should bind to DelayFeedback(0); controls:\n{d}"
    );
}

// ---------------------------------------------------------------------------
// Step 3: Pot in WDF stage tree
// ---------------------------------------------------------------------------

#[test]
fn pot_in_wdf_stage() {
    // Pot in a diode clipping stage should bind to PotInStage or PotInMultiNlStage.
    // Use a topology modeled on the RAT's clipping section where the pot is
    // in the same connected subgraph as the diodes.
    let src = r#"
        pedal "Pot In Stage" {
            components {
                R1: resistor(1k)
                U1: opamp(tl072)
                R2: resistor(560)
                Gain: pot(100k)
                D1: diode(silicon)
                D2: diode(silicon)
                R3: resistor(1k)
                Tone: pot(100k)
                C1: cap(3.3n)
                R4: resistor(1k)
            }
            nets {
                in -> R1.a
                R1.b -> U1.neg
                U1.pos -> gnd
                U1.neg -> R2.a
                R2.b -> Gain.a
                Gain.b -> U1.out
                U1.out -> R3.a
                R3.b -> D1.a, D2.b, Tone.a
                D1.b -> gnd
                D2.a -> gnd
                Tone.b -> C1.a
                C1.b -> gnd
                Tone.b -> R4.a
                R4.b -> out
            }
            controls {
                Tone.position -> "Tone" [0.0, 1.0] = 0.5
            }
        }
    "#;
    let compiled = compile_src(src);
    let d = dump(&compiled);
    assert!(
        d.contains("Tone -> PotInStage(") || d.contains("Tone -> PotInMultiNlStage("),
        "Pot in diode clipping stage should bind to PotInStage or PotInMultiNlStage; controls:\n{d}"
    );
}

// ---------------------------------------------------------------------------
// Step 2: Op-amp feedback pot
// ---------------------------------------------------------------------------

#[test]
fn pot_in_opamp_feedback() {
    // Pot in op-amp Rf path should bind to PotInStage (with OpAmpGain side-effect)
    let src = r#"
        pedal "OpAmp Gain" {
            components {
                R1: resistor(1k)
                U1: opamp(tl072)
                Gain: pot(100k)
                R2: resistor(560)
                C1: cap(100n)
            }
            nets {
                in -> C1.a
                C1.b -> R1.a
                R1.b -> U1.neg
                U1.pos -> gnd
                U1.neg -> R2.a
                R2.b -> Gain.a
                Gain.b -> U1.out
                U1.out -> out
            }
            controls {
                Gain.position -> "Gain" [0.0, 1.0] = 0.5
            }
        }
    "#;
    let compiled = compile_src(src);
    let d = dump(&compiled);
    assert!(
        d.contains("Gain -> PotInStage("),
        "Pot in opamp Rf path should bind to PotInStage (with OpAmpGain side-effect); controls:\n{d}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Section 2: BBD Runtime (Behavioral)
// ═══════════════════════════════════════════════════════════════════════════

/// Minimal BBD delay pedal DSL for behavioral tests.
const BBD_DELAY_PEDAL: &str = r#"
    pedal "BBD Test Delay" {
        components {
            C1: cap(100n)
            R1: resistor(10k)
            C2: cap(6.8n)
            R2: resistor(10k)
            BBD1: bbd(mn3005)
            Time: pot(500k)
            Repeats: pot(100k)
            R3: resistor(47k)
            C3: cap(100n)
            C4: cap(6.8n)
            R4: resistor(10k)
            Mix: pot(100k)
            R5: resistor(10k)
            C5: cap(100n)
        }
        nets {
            in -> C1.a
            C1.b -> R1.a
            R1.b -> C2.a, R2.a
            C2.b -> gnd
            R2.b -> BBD1.in
            Time.wiper -> BBD1.clock
            BBD1.out -> Repeats.a
            Repeats.b -> R3.a
            R3.b -> C3.a
            C3.b -> BBD1.in
            BBD1.out -> C4.a, R4.a
            C4.b -> gnd
            R4.b -> Mix.a
            C1.b -> R5.a
            R5.b -> Mix.a
            Mix.b -> C5.a
            C5.b -> out
        }
        controls {
            Time.position -> "Time" [0.0, 1.0] = 0.4
            Repeats.position -> "Repeats" [0.0, 1.0] = 0.35
            Mix.position -> "Mix" [0.0, 1.0] = 0.5
        }
    }
"#;

#[test]
fn bbd_clock_rate_changes_delay_time() {
    let input = sine(440.0, 0.5, SAMPLE_RATE);
    let out_short = compile_and_process(BBD_DELAY_PEDAL, &input, SAMPLE_RATE, &[("Time", 0.2)]);
    let out_long = compile_and_process(BBD_DELAY_PEDAL, &input, SAMPLE_RATE, &[("Time", 0.8)]);

    assert_healthy(&out_short, "bbd_short_delay", 10.0);
    assert_healthy(&out_long, "bbd_long_delay", 10.0);

    let corr = correlation(&out_short, &out_long);
    assert!(
        corr < 0.99,
        "Different Time values should produce different output (corr={corr:.4})"
    );
}

#[test]
fn bbd_feedback_changes_repeats() {
    let input = guitar_pluck(330.0, 1.0, SAMPLE_RATE);
    let out_dry = compile_and_process(
        BBD_DELAY_PEDAL,
        &input,
        SAMPLE_RATE,
        &[("Repeats", 0.0), ("Mix", 0.8)],
    );
    let out_wet = compile_and_process(
        BBD_DELAY_PEDAL,
        &input,
        SAMPLE_RATE,
        &[("Repeats", 0.8), ("Mix", 0.8)],
    );

    assert_healthy(&out_dry, "bbd_no_feedback", 10.0);
    assert_healthy(&out_wet, "bbd_high_feedback", 10.0);

    // Different feedback amounts should produce different output
    let corr = correlation(&out_dry, &out_wet);
    assert!(
        corr < 0.99,
        "Different Feedback values should produce different output (corr={corr:.4})"
    );
}

#[test]
fn bbd_mix_controls_wetdry() {
    let input = sine(440.0, 0.5, SAMPLE_RATE);
    let out_dry = compile_and_process(BBD_DELAY_PEDAL, &input, SAMPLE_RATE, &[("Mix", 0.0)]);
    let out_wet = compile_and_process(BBD_DELAY_PEDAL, &input, SAMPLE_RATE, &[("Mix", 1.0)]);

    assert_healthy(&out_dry, "bbd_dry_mix", 10.0);
    assert_healthy(&out_wet, "bbd_wet_mix", 10.0);

    let corr = correlation(&out_dry, &out_wet);
    assert!(
        corr < 0.99,
        "Mix=0 and Mix=1 should produce different output (corr={corr:.4})"
    );
}

#[test]
fn bbd_mix_zero_is_dry() {
    let input = sine(440.0, 0.3, SAMPLE_RATE);
    let out = compile_and_process(BBD_DELAY_PEDAL, &input, SAMPLE_RATE, &[("Mix", 0.0)]);

    assert_healthy(&out, "bbd_mix_zero", 10.0);

    // With mix at 0, output should closely track input (high correlation)
    let corr = correlation(&input, &out);
    assert!(
        corr > 0.5,
        "Mix=0 should produce mostly dry output (corr={corr:.4})"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Section 3: Pot-in-Stage Runtime (Behavioral) — RAT controls
// ═══════════════════════════════════════════════════════════════════════════

fn load_rat_src() -> String {
    let path = load_example("proco_rat.pedal");
    std::fs::read_to_string(&path).unwrap()
}

#[test]
fn rat_distortion_pot_changes_clipping() {
    let rat_src = load_rat_src();
    let input = sine(440.0, 0.3, SAMPLE_RATE);
    let out_low = compile_and_process(&rat_src, &input, SAMPLE_RATE, &[("Distortion", 0.1)]);
    let out_high = compile_and_process(&rat_src, &input, SAMPLE_RATE, &[("Distortion", 0.9)]);

    assert_healthy(&out_low, "rat_low_dist", 10.0);
    assert_healthy(&out_high, "rat_high_dist", 10.0);

    let thd_low = thd(&out_low, SAMPLE_RATE, 440.0);
    let thd_high = thd(&out_high, SAMPLE_RATE, 440.0);

    // High distortion should produce more THD than low
    // (or at minimum, they should differ)
    let corr = correlation(&out_low, &out_high);
    assert!(
        corr < 0.99,
        "Different Distortion settings should produce different output \
         (corr={corr:.4}, thd_low={thd_low:.4}, thd_high={thd_high:.4})"
    );
}

#[test]
fn rat_filter_pot_changes_brightness() {
    let rat_src = load_rat_src();
    // Use a harmonically rich signal so the filter has content to shape
    let input = guitar_pluck(220.0, 1.0, SAMPLE_RATE);
    let out_dark = compile_and_process(
        &rat_src,
        &input,
        SAMPLE_RATE,
        &[("Distortion", 0.5), ("Filter", 0.1)],
    );
    let out_bright = compile_and_process(
        &rat_src,
        &input,
        SAMPLE_RATE,
        &[("Distortion", 0.5), ("Filter", 0.9)],
    );

    assert_healthy(&out_dark, "rat_dark", 10.0);
    assert_healthy(&out_bright, "rat_bright", 10.0);

    // With harmonic-rich input, different filter settings should produce
    // different spectral content or at least different output
    let corr = correlation(&out_dark, &out_bright);
    assert!(
        corr < 0.999,
        "Different Filter settings should produce different output (corr={corr:.6})"
    );
}

#[test]
fn rat_volume_pot_changes_level() {
    let rat_src = load_rat_src();
    let input = sine(440.0, 0.3, SAMPLE_RATE);
    let out_quiet = compile_and_process(&rat_src, &input, SAMPLE_RATE, &[("Volume", 0.2)]);
    let out_loud = compile_and_process(&rat_src, &input, SAMPLE_RATE, &[("Volume", 0.8)]);

    assert_healthy(&out_quiet, "rat_quiet", 10.0);
    assert_healthy(&out_loud, "rat_loud", 10.0);

    let rms_quiet = rms(&out_quiet);
    let rms_loud = rms(&out_loud);
    assert!(
        rms_loud > rms_quiet,
        "Higher Volume should produce louder output \
         (rms_quiet={rms_quiet:.6}, rms_loud={rms_loud:.6})"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Section 4: Real Pedal Regressions (End-to-End)
// ═══════════════════════════════════════════════════════════════════════════

// ---------------------------------------------------------------------------
// Structural: verify all controls bound correctly
// ---------------------------------------------------------------------------

#[test]
fn memory_man_all_controls_bound() {
    let compiled = compile_example("memory_man.pedal");
    let d = dump(&compiled);

    // Delay pot -> LFO rate (LFO modulates BBD clock)
    assert!(
        d.contains("Delay -> LfoRate(0)"),
        "Memory Man: Delay should bind to LfoRate(0); controls:\n{d}"
    );
    // Feedback pot -> BBD feedback
    assert!(
        d.contains("Feedback -> BbdFeedback(0)"),
        "Memory Man: Feedback should bind to BbdFeedback(0); controls:\n{d}"
    );
    // Blend pot reaches BBD output through passives → PotInStage (with BbdMix side-effect)
    assert!(
        d.contains("Blend -> PotInStage("),
        "Memory Man: Blend should bind to PotInStage (with BbdMix side-effect); controls:\n{d}"
    );
}

#[test]
fn dm2_all_controls_bound() {
    let compiled = compile_example("boss_dm2.pedal");
    let d = dump(&compiled);

    // Time pot directly drives BBD clock
    assert!(
        d.contains("Time -> BbdClockRate(0)"),
        "DM-2: Time should bind to BbdClockRate(0); controls:\n{d}"
    );
    // Repeats pot -> BBD feedback
    assert!(
        d.contains("Repeats -> BbdFeedback(0)"),
        "DM-2: Repeats should bind to BbdFeedback(0); controls:\n{d}"
    );
    // Mix pot reaches BBD output through passives → PotInStage (with BbdMix side-effect)
    assert!(
        d.contains("Mix -> PotInStage("),
        "DM-2: Mix should bind to PotInStage (with BbdMix side-effect); controls:\n{d}"
    );
}

#[test]
fn ce2_controls_bound() {
    let compiled = compile_example("boss_ce2.pedal");
    let d = dump(&compiled);

    // Rate pot -> LFO rate
    assert!(
        d.contains("Rate -> LfoRate(0)"),
        "CE-2: Rate should bind to LfoRate(0); controls:\n{d}"
    );
    // Depth pot -> LFO depth
    assert!(
        d.contains("Depth -> LfoDepth(0)"),
        "CE-2: Depth should bind to LfoDepth(0); controls:\n{d}"
    );
}

#[test]
fn rat_controls_bound() {
    let compiled = compile_example("proco_rat.pedal");
    let d = dump(&compiled);

    // Distortion: should be PotInStage (with OpAmpGain side-effect)
    assert!(
        d.contains("Distortion -> PotInStage("),
        "RAT: Distortion should bind to PotInStage; controls:\n{d}"
    );
    // Filter: should be PotInStage (tone control in passive filter stage)
    assert!(
        d.contains("Filter -> PotInStage(")
            || d.contains("Filter -> PotInMultiNlStage(")
            || d.contains("Filter -> OpAmpGain {"),
        "RAT: Filter should bind to a stage target; controls:\n{d}"
    );
    // Volume: should be PotInStage or PotInMultiNlStage
    assert!(
        d.contains("Volume -> PotInStage(")
            || d.contains("Volume -> PotInMultiNlStage("),
        "RAT: Volume should bind to a stage target; controls:\n{d}"
    );
}

#[test]
fn klon_controls_bound() {
    let compiled = compile_example("klon_centaur.pedal");
    let d = dump(&compiled);

    // Gain: should be PotInStage (with OpAmpGain side-effect if in Rf path)
    assert!(
        d.contains("Gain -> PotInStage(")
            || d.contains("Gain -> PotInMultiNlStage("),
        "Klon: Gain should bind to a stage target; controls:\n{d}"
    );
    // Treble: should be PotInStage
    assert!(
        d.contains("Treble -> PotInStage(")
            || d.contains("Treble -> PotInMultiNlStage("),
        "Klon: Treble should bind to a stage target; controls:\n{d}"
    );
    // Output: should be PotInStage
    assert!(
        d.contains("Output -> PotInStage("),
        "Klon: Output should bind to PotInStage; controls:\n{d}"
    );
}

// ---------------------------------------------------------------------------
// Behavioral: verify controls actually affect output
// ---------------------------------------------------------------------------

fn assert_control_affects_output(
    pedal_file: &str,
    control_name: &str,
    low_val: f64,
    high_val: f64,
) {
    let path = load_example(pedal_file);
    let src = std::fs::read_to_string(&path).unwrap();
    let input = sine(440.0, 0.5, SAMPLE_RATE);

    let out_low = compile_and_process(&src, &input, SAMPLE_RATE, &[(control_name, low_val)]);
    let out_high = compile_and_process(&src, &input, SAMPLE_RATE, &[(control_name, high_val)]);

    assert_healthy(&out_low, &format!("{pedal_file}_{control_name}_low"), 10.0);
    assert_healthy(
        &out_high,
        &format!("{pedal_file}_{control_name}_high"),
        10.0,
    );

    let corr = correlation(&out_low, &out_high);
    assert!(
        corr < 0.999,
        "{pedal_file}: {control_name} at {low_val} vs {high_val} should differ (corr={corr:.6})"
    );
}

#[test]
fn memory_man_controls_affect_output() {
    assert_control_affects_output("memory_man.pedal", "Delay", 0.2, 0.8);
    assert_control_affects_output("memory_man.pedal", "Feedback", 0.2, 0.8);
    assert_control_affects_output("memory_man.pedal", "Blend", 0.2, 0.8);
}

#[test]
fn dm2_controls_affect_output() {
    assert_control_affects_output("boss_dm2.pedal", "Time", 0.2, 0.8);
    assert_control_affects_output("boss_dm2.pedal", "Repeats", 0.2, 0.8);
    assert_control_affects_output("boss_dm2.pedal", "Mix", 0.2, 0.8);
}

#[test]
fn ce2_controls_affect_output() {
    assert_control_affects_output("boss_ce2.pedal", "Rate", 0.2, 0.8);
    assert_control_affects_output("boss_ce2.pedal", "Depth", 0.2, 0.8);
}
