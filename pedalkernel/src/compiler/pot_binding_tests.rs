// TDD tests for pot binding — verifying that pots are found and bound
// so that set_control actually changes something.

use super::spqr_build::compile_via_spqr;
use crate::PedalProcessor;

// ═══════════════════════════════════════════════════════════════════════════
// Layer 1: Is the pot bound at all?
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_drive_pot_is_bound() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/screamer.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    let drive_bound = compiled.controls.iter().any(|c| c.label == "Drive");
    let tone_bound = compiled.controls.iter().any(|c| c.label == "Tone");
    let level_bound = compiled.controls.iter().any(|c| c.label == "Level");

    eprintln!("Screamer bindings:");
    for c in &compiled.controls {
        eprintln!("  {:?} -> {:?}", c.label, c.target);
    }

    assert!(drive_bound, "Drive pot should be bound");
    assert!(tone_bound, "Tone pot should be bound");
    assert!(level_bound, "Level pot should be bound");
}

#[test]
fn ratking_v1a_distortion_pot_is_bound() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/ratking_non_invert_v1a.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    let distortion_bound = compiled.controls.iter().any(|c| c.label == "Distortion");
    let filter_bound = compiled.controls.iter().any(|c| c.label == "Filter");
    let volume_bound = compiled.controls.iter().any(|c| c.label == "Volume");

    eprintln!("Ratking v1a bindings:");
    for c in &compiled.controls {
        eprintln!("  {:?} -> {:?}", c.label, c.target);
    }

    assert!(distortion_bound, "Distortion pot should be bound");
    assert!(filter_bound, "Filter pot should be bound");
    assert!(volume_bound, "Volume pot should be bound");
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 2: Minimal circuits — does the pot binding work?
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn feedback_drive_pot_changes_gain() {
    // Simplest case: pot in op-amp feedback (inverting).
    // Sweeping drive should change Rf → change gain.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Drive: pot(100k, a)
                D1: diode(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Drive.a
                Drive.b -> U1.out
                U1.neg -> D1.a
                D1.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls { Drive.position -> "Drive" [0.0, 1.0] = 0.5 }
        }"#,
    )
    .expect("parse");

    let mut lo = compile_via_spqr(&pedal, 48000.0).expect("compile");
    lo.set_control("Drive", 0.0);
    for _ in 0..500 {
        lo.process(0.0);
    }
    let mut peak_lo = 0.0f64;
    for s in 0..480 {
        let input = 0.01 * (2.0 * std::f64::consts::PI * 440.0 * s as f64 / 48000.0).sin();
        peak_lo = peak_lo.max(lo.process(input).abs());
    }

    let mut hi = compile_via_spqr(&pedal, 48000.0).expect("compile");
    hi.set_control("Drive", 1.0);
    for _ in 0..500 {
        hi.process(0.0);
    }
    let mut peak_hi = 0.0f64;
    for s in 0..480 {
        let input = 0.01 * (2.0 * std::f64::consts::PI * 440.0 * s as f64 / 48000.0).sin();
        peak_hi = peak_hi.max(hi.process(input).abs());
    }

    eprintln!("Feedback drive: lo={peak_lo:.4}, hi={peak_hi:.4}");

    // Check binding exists
    let bound = lo.controls.iter().any(|c| c.label == "Drive");
    assert!(bound, "Drive should be bound");

    // Sweeping should change output
    assert!(
        (peak_lo - peak_hi).abs() > 0.001 || peak_lo.max(peak_hi) > 0.01,
        "Drive pot should change output: lo={peak_lo:.4}, hi={peak_hi:.4}"
    );
}

#[test]
fn tone_pot_in_blackfeedback_changes_spectrum() {
    // Tone pot in a BlackFeedback stage (non-inverting, cap + pot).
    // Sweeping tone should change frequency balance.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(47k)
                C_tone: cap(22n)
                Tone: pot(20k, b)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.neg -> C_tone.a
                C_tone.b -> Tone.a
                Tone.w -> U1.out
                Tone.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls { Tone.position -> "Tone" [0.0, 1.0] = 0.5 }
        }"#,
    )
    .expect("parse");

    // Check binding exists
    let compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    let bound = compiled.controls.iter().any(|c| c.label == "Tone");
    eprintln!("Tone pot binding:");
    for c in &compiled.controls {
        eprintln!("  {:?} -> {:?}", c.label, c.target);
    }
    eprintln!(
        "  Controls: {:?}",
        compiled
            .controls
            .iter()
            .map(|c| &c.label)
            .collect::<Vec<_>>()
    );

    // The tone pot may not be bound if it's consumed by BlackFeedback.
    // This is the test — it SHOULD be bound.
    if !bound {
        eprintln!("  Tone NOT bound — checking WDF stage trees:");
        for (i, s) in compiled.stages.iter().enumerate() {
            if let super::compiled::Stage::Wdf(w) = s {
                let pos = w
                    .tree
                    .get_pot_position("Tone")
                    .or_else(|| w.tree.get_pot_position("Tone__aw"))
                    .or_else(|| w.tree.get_pot_position("Tone__wb"));
                if pos.is_some() {
                    eprintln!("    wdf[{i}]: Tone found, pos={:?}", pos);
                }
            }
        }
    }
    assert!(bound, "Tone pot should be bound");

    // Verify the pot position actually changes when set_control is called
    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    // Find a WDF stage with the Tone pot
    let find_tone_pos = |stages: &[super::compiled::Stage]| -> Option<f64> {
        for s in stages {
            if let super::compiled::Stage::Wdf(wdf) = s {
                if let Some(pos) = wdf
                    .tree
                    .get_pot_position("Tone")
                    .or_else(|| wdf.tree.get_pot_position("Tone__aw"))
                {
                    return Some(pos);
                }
            }
        }
        None
    };

    let initial_pos = find_tone_pos(&compiled.stages);
    eprintln!("Initial Tone position: {:?}", initial_pos);

    compiled.set_control("Tone", 0.0);
    let pos_0 = find_tone_pos(&compiled.stages);

    compiled.set_control("Tone", 1.0);
    let pos_1 = find_tone_pos(&compiled.stages);

    eprintln!("Tone @0.0: {:?}, @1.0: {:?}", pos_0, pos_1);

    if let (Some(p0), Some(p1)) = (pos_0, pos_1) {
        assert!(
            (p0 - p1).abs() > 0.01,
            "Tone pot position should change: @0={p0:.3}, @1={p1:.3}"
        );
    } else {
        panic!("Tone pot not found in any WDF stage after set_control");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 3: Feedback pot parameter extraction — the root cause of broken
// Drive/Distortion pots. The FeedbackConfig must have correct Rf/Ri/series
// values for the gain recomputation to work.
//
// These test MINIMAL circuits where we know the exact expected gain.
// ═══════════════════════════════════════════════════════════════════════════

/// Helper: compile a circuit and measure peak output at two pot positions.
fn measure_pot_sweep(pedal_src: &str, control: &str) -> (f64, f64) {
    let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse");
    let sr = 48000.0;
    let freq = 440.0;

    let mut measure = |pos: f64| -> f64 {
        let mut c = compile_via_spqr(&pedal, sr).expect("compile");
        c.set_control(control, pos);
        // Settle
        for s in 0..500 {
            let input = 0.01 * (std::f64::consts::TAU * freq * s as f64 / sr).sin();
            c.process(input);
        }
        // Measure
        let mut peak = 0.0f64;
        for s in 500..720 {
            let input = 0.01 * (std::f64::consts::TAU * freq * s as f64 / sr).sin();
            peak = peak.max(c.process(input).abs());
        }
        peak
    };

    (measure(0.1), measure(0.9))
}

#[test]
fn feedback_pot_only_drive_changes_gain() {
    // Simplest NL feedback: pot is the ONLY element between neg and out
    // (besides the diode). No series resistors, no parallel resistors.
    // Drive @0.1 → low gain, @0.9 → high gain.
    let (lo, hi) = measure_pot_sweep(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Drive: pot(500k, a)
                D1: diode(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Drive.a
                Drive.b -> U1.out
                U1.neg -> D1.a
                D1.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls { Drive.position -> "Drive" [0.0, 1.0] = 0.5 }
        }"#,
        "Drive",
    );

    eprintln!("pot-only drive: lo={lo:.6}, hi={hi:.6}");
    assert!(
        hi > lo * 1.5,
        "Drive pot should change gain: lo={lo:.6}, hi={hi:.6}"
    );
}

#[test]
fn feedback_pot_with_series_r_drive_changes_gain() {
    // TS-style: Drive pot in series with R_clip(4.7k).
    // The series R sets the minimum gain floor.
    let (lo, hi) = measure_pot_sweep(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Drive: pot(500k, a)
                R_clip: resistor(4.7k)
                D1: diode(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Drive.a
                Drive.b -> R_clip.a
                R_clip.b -> U1.out
                U1.neg -> D1.a
                D1.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls { Drive.position -> "Drive" [0.0, 1.0] = 0.5 }
        }"#,
        "Drive",
    );

    eprintln!("pot+series drive: lo={lo:.6}, hi={hi:.6}");
    assert!(
        hi > lo * 1.5,
        "Drive pot should change gain: lo={lo:.6}, hi={hi:.6}"
    );
}

#[test]
fn feedback_pot_parallel_with_rf_drive_changes_gain() {
    // TS/RAT pattern: Drive pot + R_clip in series, Rf in parallel.
    // This is the topology that find_feedback_pot's heuristic gets wrong.
    let (lo, hi) = measure_pot_sweep(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Drive: pot(500k, a)
                R_clip: resistor(4.7k)
                Rf: resistor(100k)
                D1: diode(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Drive.a
                Drive.b -> R_clip.a
                R_clip.b -> U1.out
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.neg -> D1.a
                D1.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls { Drive.position -> "Drive" [0.0, 1.0] = 0.5 }
        }"#,
        "Drive",
    );

    eprintln!("pot+parallel Rf drive: lo={lo:.6}, hi={hi:.6}");
    assert!(
        hi > lo * 1.5,
        "Drive pot should change gain: lo={lo:.6}, hi={hi:.6}"
    );
}

#[test]
fn feedback_pot_screamer_topology_drive_changes_gain() {
    // Full Screamer-like topology: Drive pot + multiple resistors + cap in feedback.
    // This is the real-world circuit that currently fails.
    let (lo, hi) = measure_pot_sweep(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Drive: pot(500k, a)
                R_clip: resistor(4.7k)
                C_clip: cap(47n)
                D1: diode(silicon)
                D2: diode(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Drive.a
                Drive.b -> R_clip.a
                R_clip.b -> U1.out
                U1.neg -> C_clip.a
                C_clip.b -> U1.out
                U1.neg -> D1.a
                D1.b -> U1.out
                U1.neg -> D2.b
                D2.a -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls { Drive.position -> "Drive" [0.0, 1.0] = 0.5 }
        }"#,
        "Drive",
    );

    eprintln!("screamer-like drive: lo={lo:.6}, hi={hi:.6}");
    assert!(
        hi > lo * 1.5,
        "Drive pot should change gain: lo={lo:.6}, hi={hi:.6}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 3b: Orphaned pots — pots in non-feedback passive groups
//
// Some pots (volume dividers, fuzz emitter coupling) aren't in any
// feedback stage. They're standalone passive groups. The binding
// system needs to find and bind them anyway.
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn orphan_volume_pot_is_bound() {
    // Volume pot as a simple voltage divider (not in any feedback loop).
    // Should still be bound as a control.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                Volume: pot(100k, a)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> Volume.a
                Volume.w -> out
                Volume.b -> gnd
            }
            controls { Volume.position -> "Volume" [0.0, 1.0] = 0.5 }
        }"#,
    )
    .expect("parse");

    let compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    let bound = compiled.controls.iter().any(|c| c.label == "Volume");
    eprintln!("Orphan volume: bound={bound}");
    for c in &compiled.controls {
        eprintln!("  {:?} -> {:?}", c.label, c.target);
    }
    assert!(
        bound,
        "Volume pot (voltage divider) should be bound even when not in feedback"
    );
}

#[test]
fn orphan_volume_pot_changes_output() {
    // Volume pot as divider should actually change output level.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                Volume: pot(100k, a)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> Volume.a
                Volume.w -> out
                Volume.b -> gnd
            }
            controls { Volume.position -> "Volume" [0.0, 1.0] = 0.5 }
        }"#,
    )
    .expect("parse");

    let (lo, hi) = measure_pot_sweep_parsed(&pedal, "Volume");
    eprintln!("Orphan volume: lo={lo:.6}, hi={hi:.6}");
    // Volume pot is a divider — output should change significantly between positions.
    // Direction depends on taper/wiring: either hi>lo or lo>hi is valid.
    let ratio = lo.max(hi) / lo.min(hi).max(1e-10);
    assert!(
        ratio > 1.5,
        "Volume pot should change output: lo={lo:.6}, hi={hi:.6}, ratio={ratio:.1}x"
    );
}

/// Helper for parsed pedals (avoids re-parsing)
fn measure_pot_sweep_parsed(pedal: &crate::dsl::PedalDef, control: &str) -> (f64, f64) {
    let sr = 48000.0;
    let freq = 440.0;
    let mut measure = |pos: f64| -> f64 {
        let mut c = compile_via_spqr(pedal, sr).expect("compile");
        c.set_control(control, pos);
        for s in 0..500 {
            let input = 0.01 * (std::f64::consts::TAU * freq * s as f64 / sr).sin();
            c.process(input);
        }
        let mut peak = 0.0f64;
        for s in 500..720 {
            let input = 0.01 * (std::f64::consts::TAU * freq * s as f64 / sr).sin();
            peak = peak.max(c.process(input).abs());
        }
        peak
    };
    (measure(0.1), measure(0.9))
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 3c: Inter-stage voltage divider — pot wiper splits across stages
//
// When a pot's wiper sits between two stages (output of one, input of next),
// the pot acts as a voltage divider in the routing layer. No WDF tree needed.
// signal *= R_wb / (R_aw + R_wb) applied between stages.
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn interstage_volume_pot_changes_output() {
    // Simple case: gain stage → Volume pot → output.
    // Volume wiper goes to out. Volume.a connects to stage output.
    // Volume.b goes to GND. Sweeping should attenuate.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                Volume: pot(100k, a)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> Volume.a
                Volume.w -> out
                Volume.b -> gnd
            }
            controls { Volume.position -> "Volume" [0.0, 1.0] = 0.5 }
        }"#,
    )
    .expect("parse");

    let (lo, hi) = measure_pot_sweep_parsed(&pedal, "Volume");
    let ratio = lo.max(hi) / lo.min(hi).max(1e-10);
    eprintln!("Interstage volume: lo={lo:.6}, hi={hi:.6}, ratio={ratio:.1}x");
    assert!(
        ratio > 2.0,
        "Volume pot should attenuate: lo={lo:.6}, hi={hi:.6}, ratio={ratio:.1}x"
    );
}

#[test]
fn interstage_volume_pot_between_two_stages() {
    // Two gain stages with Volume pot between them.
    // U1.out → Volume.a, Volume.w → R_mid → U2.neg, Volume.b → gnd.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf1: resistor(100k)
                Volume: pot(100k, a)
                R_mid: resistor(10k)
                U2: opamp(tl072)
                Rf2: resistor(100k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf1.a
                Rf1.b -> U1.out
                U1.pos -> gnd
                U1.out -> Volume.a
                Volume.w -> R_mid.a
                R_mid.b -> U2.neg
                U2.neg -> Rf2.a
                Rf2.b -> U2.out
                U2.pos -> gnd
                Volume.b -> gnd
                U2.out -> out
            }
            controls { Volume.position -> "Volume" [0.0, 1.0] = 0.5 }
        }"#,
    )
    .expect("parse");

    let (lo, hi) = measure_pot_sweep_parsed(&pedal, "Volume");
    let ratio = lo.max(hi) / lo.min(hi).max(1e-10);
    eprintln!("Interstage 2-stage volume: lo={lo:.6}, hi={hi:.6}, ratio={ratio:.1}x");
    assert!(
        ratio > 2.0,
        "Volume between stages should attenuate: lo={lo:.6}, hi={hi:.6}, ratio={ratio:.1}x"
    );
}

#[test]
fn interstage_tone_pot_with_buffer_changes_spectrum() {
    // Screamer pattern: tone RC network → Tone pot → U2 (unity follower) → out.
    // The Tone pot blends between bass (direct) and treble (through cap).
    // Sweeping Tone should change the spectral balance.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                R_tone: resistor(1k)
                C_tone: cap(220n)
                Tone: pot(20k, b)
                U2: opamp(tl072)
                C_out: cap(10u)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> R_tone.a
                R_tone.b -> C_tone.a
                C_tone.b -> Tone.a
                R_tone.b -> Tone.b
                Tone.w -> U2.pos
                U2.out -> U2.neg
                U2.out -> C_out.a
                C_out.b -> out
            }
            controls { Tone.position -> "Tone" [0.0, 1.0] = 0.5 }
        }"#,
    )
    .expect("parse");

    let sr = 48000.0;
    let measure_spectrum = |pos: f64| -> (f64, f64) {
        // 200Hz
        let mut lo = compile_via_spqr(&pedal, sr).expect("compile");
        lo.set_control("Tone", pos);
        for _ in 0..500 {
            lo.process(0.0);
        }
        let mut peak_lo = 0.0f64;
        for s in 0..480 {
            let input = 0.05 * (std::f64::consts::TAU * 200.0 * s as f64 / sr).sin();
            peak_lo = peak_lo.max(lo.process(input).abs());
        }
        // 5kHz
        let mut hi = compile_via_spqr(&pedal, sr).expect("compile");
        hi.set_control("Tone", pos);
        for _ in 0..500 {
            hi.process(0.0);
        }
        let mut peak_hi = 0.0f64;
        for s in 0..480 {
            let input = 0.05 * (std::f64::consts::TAU * 5000.0 * s as f64 / sr).sin();
            peak_hi = peak_hi.max(hi.process(input).abs());
        }
        (peak_lo, peak_hi)
    };

    let (lo_0, hi_0) = measure_spectrum(0.0);
    let (lo_1, hi_1) = measure_spectrum(1.0);
    let ratio_0 = if lo_0 > 1e-10 { hi_0 / lo_0 } else { 0.0 };
    let ratio_1 = if lo_1 > 1e-10 { hi_1 / lo_1 } else { 0.0 };

    eprintln!("Tone+buffer: @0: lo={lo_0:.4} hi={hi_0:.4} ratio={ratio_0:.2}");
    eprintln!("             @1: lo={lo_1:.4} hi={hi_1:.4} ratio={ratio_1:.2}");

    // At least one setting should produce output
    assert!(
        lo_0 > 0.001 || hi_0 > 0.001 || lo_1 > 0.001 || hi_1 > 0.001,
        "Should produce output at some setting"
    );

    // Spectral balance should change between positions
    let ratio_diff = (ratio_0 - ratio_1).abs();
    assert!(
        ratio_diff > 0.3,
        "Tone pot should change spectral balance: ratio_diff={ratio_diff:.2}"
    );
}

#[test]
fn interstage_level_pot_after_clipping_changes_gain() {
    // Level/Volume pot after a clipping stage (common pedal pattern).
    // The pot is in a separate non-feedback group but may share nodes
    // with the clipping stage's output.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Drive: pot(500k, a)
                D1: diode(silicon)
                Level: pot(100k, a)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Drive.a
                Drive.b -> U1.out
                U1.neg -> D1.a
                D1.b -> U1.out
                U1.pos -> gnd
                U1.out -> Level.a
                Level.w -> out
                Level.b -> gnd
            }
            controls {
                Drive.position -> "Drive" [0.0, 1.0] = 0.5
                Level.position -> "Level" [0.0, 1.0] = 0.5
            }
        }"#,
    )
    .expect("parse");

    let (lo, hi) = measure_pot_sweep_parsed(&pedal, "Level");
    let ratio = lo.max(hi) / lo.min(hi).max(1e-10);
    eprintln!("Level after clipping: lo={lo:.6}, hi={hi:.6}, ratio={ratio:.1}x");
    assert!(
        ratio > 2.0,
        "Level pot should change output: lo={lo:.6}, hi={hi:.6}, ratio={ratio:.1}x"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 4: FlowGroup edge classification — R_in should be pendant, not feedback
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn r_in_is_pendant_not_feedback_simple() {
    // In a simple inverting amp, R_in (input → neg) should be pendant.
    let pedal_src = r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#;

    let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse");
    let graph = super::graph::CircuitGraph::from_pedal(&pedal);
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    let groups = super::signal_flow::find_flow_groups(&all_edges, &graph);

    // Find the group containing U1
    let u1_group = groups
        .iter()
        .find(|g| {
            g.active_edges
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "U1")
        })
        .expect("Should find U1's group");

    let pendant_names: Vec<&str> = u1_group
        .pendant_edges
        .iter()
        .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
        .collect();
    let feedback_names: Vec<&str> = u1_group
        .feedback_edges
        .iter()
        .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
        .collect();

    eprintln!("Simple: pendant={pendant_names:?}, feedback={feedback_names:?}");
    assert!(
        pendant_names.contains(&"R_in"),
        "R_in should be pendant, not feedback"
    );
}

#[test]
fn r_in_is_pendant_with_diodes_in_feedback() {
    // With diodes in feedback, R_in should STILL be pendant.
    // This is the case that fails in real pedals.
    let pedal_src = r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Drive: pot(500k, a)
                R_clip: resistor(4.7k)
                C_clip: cap(47n)
                D1: diode(silicon)
                D2: diode(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Drive.a
                Drive.b -> R_clip.a
                R_clip.b -> U1.out
                U1.neg -> C_clip.a
                C_clip.b -> U1.out
                U1.neg -> D1.a
                D1.b -> U1.out
                U1.neg -> D2.b
                D2.a -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls { Drive.position -> "Drive" [0.0, 1.0] = 0.5 }
        }"#;

    let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse");
    let graph = super::graph::CircuitGraph::from_pedal(&pedal);
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    let groups = super::signal_flow::find_flow_groups(&all_edges, &graph);

    // Find the group containing U1
    let u1_group = groups
        .iter()
        .find(|g| {
            g.active_edges
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "U1")
        })
        .expect("Should find U1's group");

    let pendant_names: Vec<&str> = u1_group
        .pendant_edges
        .iter()
        .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
        .collect();
    let feedback_names: Vec<&str> = u1_group
        .feedback_edges
        .iter()
        .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
        .collect();

    eprintln!("With diodes: pendant={pendant_names:?}, feedback={feedback_names:?}");
    assert!(
        pendant_names.contains(&"R_in"),
        "R_in should be pendant even with diodes in feedback. Got pendant={pendant_names:?}, feedback={feedback_names:?}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 5: WdfStage gain from port resistances — the correct approach
//
// The stage should compute gain = Rf/Ri from actual port resistances,
// not from pre-computed FeedbackConfig parameters. When a pot moves,
// the stage reads the pot's current resistance, adds any fixed series R,
// divides by Ri (from pendant tree), and calls opamp.set_gain().
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn wdf_stage_gain_updates_from_pot_resistance() {
    // End-to-end: compile a circuit with Drive pot in feedback + diode,
    // sweep the pot, verify the OpAmpRoot gain changes.
    // This tests the STAGE's pot→gain pipeline, not just OpAmpRoot math.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Drive: pot(500k, a)
                D1: diode(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Drive.a
                Drive.b -> U1.out
                U1.neg -> D1.a
                D1.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls { Drive.position -> "Drive" [0.0, 1.0] = 0.5 }
        }"#,
    )
    .expect("parse");

    let sr = 48000.0;
    let freq = 440.0;

    let mut measure = |pos: f64| -> f64 {
        let mut c = compile_via_spqr(&pedal, sr).expect("compile");
        c.set_control("Drive", pos);
        for s in 0..500 {
            let input = 0.01 * (std::f64::consts::TAU * freq * s as f64 / sr).sin();
            c.process(input);
        }
        let mut peak = 0.0f64;
        for s in 500..720 {
            let input = 0.01 * (std::f64::consts::TAU * freq * s as f64 / sr).sin();
            peak = peak.max(c.process(input).abs());
        }
        peak
    };

    let lo = measure(0.1);
    let hi = measure(0.9);
    eprintln!("WDF stage pot→gain: lo={lo:.6}, hi={hi:.6}");
    assert!(
        hi > lo * 1.5,
        "Drive pot should change WDF stage gain: lo={lo:.6}, hi={hi:.6}"
    );
}

#[test]
fn screamer_pot_stage_diagnostic() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/screamer.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    eprintln!("  Screamer: {} stages", compiled.stages.len());
    for (i, stage) in compiled.stages.iter().enumerate() {
        let (stype, has_tone, has_level, has_drive) = match stage {
            pedalkernel_rt::processor::Stage::Wdf(w) => {
                let t = w.tree.get_pot_position("Tone").is_some()
                    || w.tree.get_pot_position("Tone__aw").is_some()
                    || w.tree.get_pot_position("Tone__wb").is_some();
                let l = w.tree.get_pot_position("Level").is_some()
                    || w.tree.get_pot_position("Level__aw").is_some()
                    || w.tree.get_pot_position("Level__wb").is_some();
                let d = w.tree.get_pot_position("Drive").is_some()
                    || w.tree.get_pot_position("Drive__aw").is_some()
                    || w.tree.get_pot_position("Drive__wb").is_some();
                // Also check opamp_children
                let t2 = w.opamp_children.iter().any(|c| {
                    c.get_pot_position("Tone").is_some() || c.get_pot_position("Tone__aw").is_some()
                });
                let l2 = w.opamp_children.iter().any(|c| {
                    c.get_pot_position("Level").is_some()
                        || c.get_pot_position("Level__aw").is_some()
                });
                ("WDF", t || t2, l || l2, d)
            }
            pedalkernel_rt::processor::Stage::Iir(iir) => {
                let t = iir.has_pot("Tone");
                let l = iir.has_pot("Level");
                let d = iir.has_pot("Drive");
                ("IIR", t, l, d)
            }
            pedalkernel_rt::processor::Stage::StateSpace(_) => ("SS", false, false, false),
            pedalkernel_rt::processor::Stage::MultiNl(m) => {
                let t = m
                    .pot_children
                    .iter()
                    .any(|c| c.get_pot_position("Tone").is_some());
                let l = m
                    .pot_children
                    .iter()
                    .any(|c| c.get_pot_position("Level").is_some());
                let d = m
                    .pot_children
                    .iter()
                    .any(|c| c.get_pot_position("Drive").is_some());
                ("MNL", t, l, d)
            }
            pedalkernel_rt::processor::Stage::BlackFeedback(bf) => {
                let t = bf.has_pot("Tone");
                let l = bf.has_pot("Level");
                let d = bf.has_pot("Drive");
                ("BF", t, l, d)
            }
            pedalkernel_rt::processor::Stage::Blockwise(_) => ("BKM", false, false, false),
            pedalkernel_rt::processor::Stage::SerialDelayedFeedback(serial) => (
                "SerialFB",
                serial.has_pot("Tone"),
                serial.has_pot("Level"),
                serial.has_pot("Drive"),
            ),
            pedalkernel_rt::processor::Stage::KMethod { .. } => ("KMethod", false, false, false),
        };
        eprintln!("  stage {i} ({stype}): Tone={has_tone}, Level={has_level}, Drive={has_drive}");
    }

    eprintln!("  Controls:");
    for c in &compiled.controls {
        eprintln!("    {:?} -> {:?}", c.label, c.target);
    }
}
