//! Regression tests: 3-terminal tone pots must measurably affect the output.
//!
//! A 3-terminal pot (uses `.w` / `.wiper` pin in nets) is expanded into two
//! synthetic halves (`__aw`, `__wb`) during graph construction.  Before the
//! wiper-promotion fix, sp_decompose classified the pot halves as bridging
//! edges, sending them to the MNA residual and letting junction promotion
//! handle them — but the wiper node was never promoted to junction status,
//! so the pot position had zero effect on output.
//!
//! Each test below:
//!   1. Compiles the pedal at 48 kHz.
//!   2. Generates a 1 kHz sine test signal with 8 192 warm-up + 4 096 capture.
//!   3. Processes with the tone/treble control at 0.0 vs 1.0 (other pots at 0.5).
//!   4. Asserts the RMS output differs by at least 0.5 dB.

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use std::f64::consts::PI;
use std::path::Path;

const SAMPLE_RATE: f64 = 48_000.0;
const WARMUP_SAMPLES: usize = 8_192;
const CAPTURE_SAMPLES: usize = 4_096;
const FREQ_HZ: f64 = 1_000.0;
const AMPLITUDE: f64 = 0.05;

// ---------------------------------------------------------------------------
// Helper
// ---------------------------------------------------------------------------

/// Compile a pedal source string, warm up, then capture output samples.
fn capture_with_tone(pedal_src: &str, controls: &[(&str, f64)]) -> Vec<f64> {
    let pedal = parse_pedal_file(pedal_src).expect("parse failed");
    let mut proc = compile_pedal(&pedal, SAMPLE_RATE).expect("compile failed");
    for &(label, val) in controls {
        proc.set_control(label, val);
    }

    // Warm-up.
    for i in 0..WARMUP_SAMPLES {
        let t = i as f64 / SAMPLE_RATE;
        proc.process(AMPLITUDE * (2.0 * PI * FREQ_HZ * t).sin());
    }

    // Capture.
    (0..CAPTURE_SAMPLES)
        .map(|i| {
            let t = (WARMUP_SAMPLES + i) as f64 / SAMPLE_RATE;
            proc.process(AMPLITUDE * (2.0 * PI * FREQ_HZ * t).sin())
        })
        .collect()
}

fn rms_db(buf: &[f64]) -> f64 {
    let r = rms(buf);
    if r < 1e-20 {
        return -200.0;
    }
    20.0 * r.log10()
}

// ---------------------------------------------------------------------------
// SD-1 Tone pot test
//
// The SD-1 Tone uses Tone.w -> U2.pos — a classic 3-terminal wiper-blend
// topology.  Tone=0.0 routes signal through the capacitive bass path (low
// impedance to gnd), while Tone=1.0 routes through the treble RC path.
// At 1 kHz the two settings should differ by several dB.
// ---------------------------------------------------------------------------

const SD1_SRC: &str = include_str!("../examples/pedals/overdrive/sd1.pedal");

#[test]
fn sd1_tone_pot_affects_output() {
    let out_lo = capture_with_tone(SD1_SRC, &[("Tone", 0.0), ("Drive", 0.3), ("Level", 0.5)]);
    let out_hi = capture_with_tone(SD1_SRC, &[("Tone", 1.0), ("Drive", 0.3), ("Level", 0.5)]);

    let db_lo = rms_db(&out_lo);
    let db_hi = rms_db(&out_hi);
    let diff_db = (db_hi - db_lo).abs();

    eprintln!("[sd1_tone] Tone=0→{db_lo:.2} dB, Tone=1→{db_hi:.2} dB, diff={diff_db:.2} dB");

    assert!(
        diff_db >= 0.5,
        "SD-1 Tone pot should produce ≥0.5 dB difference at 1 kHz. \
         Tone=0→{db_lo:.2} dB Tone=1→{db_hi:.2} dB diff={diff_db:.2} dB. \
         Likely wiper node not promoted to junction in sp_decompose."
    );
}

// ---------------------------------------------------------------------------
// Tube Screamer Tone pot test
//
// The Screamer Tone uses Tone.a and Tone.b — it is a 2-terminal pot (variable
// series resistance blending two RC paths). Tone=0 (reversed range) puts
// maximum resistance (20 kΩ) in the bass path. Tone=1 puts 0 Ω there.
// At 1 kHz with both C4 (220n) and C5 (100n) in the audio band, the level
// difference should be clearly measurable.
// ---------------------------------------------------------------------------

const TS_SRC: &str = include_str!("../examples/pedals/overdrive/tube_screamer.pedal");

#[test]
fn screamer_tone_pot_affects_output() {
    // Tone range is [1.0, 0.0] in the DSL (reversed), so control 0.0 = fully
    // bright (0 Ω series), control 1.0 = fully dark (20 kΩ series).
    let out_bright = capture_with_tone(TS_SRC, &[("Tone", 0.0), ("Drive", 0.3), ("Level", 0.5)]);
    let out_dark = capture_with_tone(TS_SRC, &[("Tone", 1.0), ("Drive", 0.3), ("Level", 0.5)]);

    let db_bright = rms_db(&out_bright);
    let db_dark = rms_db(&out_dark);
    let diff_db = (db_bright - db_dark).abs();

    eprintln!(
        "[ts_tone] Tone=0.0(bright)→{db_bright:.2} dB, Tone=1.0(dark)→{db_dark:.2} dB, \
         diff={diff_db:.2} dB"
    );

    assert!(
        diff_db >= 0.5,
        "Screamer Tone pot should produce ≥0.5 dB difference at 1 kHz. \
         bright={db_bright:.2} dB dark={db_dark:.2} dB diff={diff_db:.2} dB."
    );
}

// ---------------------------------------------------------------------------
// Goldenrod Treble pot test (requires pedalkernel-pro bundle)
// ---------------------------------------------------------------------------

#[test]
fn goldenrod_treble_pot_affects_output_if_present() {
    let pedal_path = format!(
        "{}/src/pedalkernel/pedalkernel-pro/pedals/legends/goldenrod.pedal",
        std::env::var("HOME").unwrap_or_default()
    );
    let path = Path::new(&pedal_path);
    if !path.exists() {
        eprintln!("[goldenrod_treble] pedal file not found at {pedal_path}, skipping.");
        return;
    }

    let src = std::fs::read_to_string(path).expect("read goldenrod.pedal");

    let out_lo = capture_with_tone(&src, &[("Treble", 0.0), ("Gain", 0.5), ("Output", 0.5)]);
    let out_hi = capture_with_tone(&src, &[("Treble", 1.0), ("Gain", 0.5), ("Output", 0.5)]);

    let db_lo = rms_db(&out_lo);
    let db_hi = rms_db(&out_hi);
    let diff_db = (db_hi - db_lo).abs();

    eprintln!(
        "[goldenrod_treble] Treble=0→{db_lo:.2} dB, Treble=1→{db_hi:.2} dB, diff={diff_db:.2} dB"
    );

    assert!(
        diff_db >= 0.5,
        "Goldenrod Treble pot should produce ≥0.5 dB difference at 1 kHz. \
         Treble=0→{db_lo:.2} dB Treble=1→{db_hi:.2} dB diff={diff_db:.2} dB."
    );
}
