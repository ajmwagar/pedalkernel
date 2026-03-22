//! Integration tests for bridged-T opamp resonator circuits.
//!
//! Validates that the WDF compiler correctly handles bridged-T feedback
//! topologies — opamp circuits where the feedback network includes
//! ground-shunted reactive elements (capacitors) that create frequency-
//! selective behavior.
//!
//! Known issue: The compiler currently classifies bridged-T opamps as
//! simple Inverting stages, losing the C1/C2 shunt caps that create
//! resonance. This test suite documents the expected behavior.

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;

const SAMPLE_RATE: f64 = 48_000.0;

// ═══════════════════════════════════════════════════════════════════════════
// Inline pedal sources
// ═══════════════════════════════════════════════════════════════════════════

/// Single bridged-T resonator at C3 (130.81 Hz).
/// f0 = 1/(2π·150k·8.2n) ≈ 129.5 Hz
const BRIDGED_T_C3: &str = include_str!("test_pedals/bridged_t_resonator.pedal");

/// Minimal bridged-T — stripped to bare essentials for debugging.
/// No bias network (pos wired to gnd for simplicity), no output resistor.
const BRIDGED_T_MINIMAL: &str = r#"
pedal "Bridged-T Minimal" {
  supply 9V
  components {
    T1: trigger_input()
    U1: opamp(tl072)
    R1: resistor(150k)
    R2: resistor(150k)
    C1: cap(8.2n)
    C2: cap(8.2n)
    R_fb: resistor(470k)
    R_trig: resistor(100k)
  }
  nets {
    U1.pos -> gnd
    U1.neg -> R1.a, C1.a
    R1.b -> R2.a, C2.a
    R2.b -> U1.out
    C1.b -> gnd
    C2.b -> gnd
    U1.neg -> R_fb.a
    R_fb.b -> U1.out
    T1.out -> R_trig.a
    R_trig.b -> R1.b
    U1.out -> out
  }
  controls {
    T1.trigger -> midi(48)
  }
}
"#;

// ═══════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════

fn trigger_and_collect(source: &str, midi_note: u8, samples: usize) -> Vec<f64> {
    let pedal = parse_pedal_file(source).expect("parse failed");
    let mut proc = compile_pedal(&pedal, SAMPLE_RATE).expect("compile failed");
    proc.note_on(midi_note, 127);
    (0..samples).map(|_| proc.process(0.0)).collect()
}

fn zero_crossings(samples: &[f64]) -> usize {
    samples
        .windows(2)
        .filter(|w| w[0].signum() != w[1].signum())
        .count()
}

fn estimate_freq(samples: &[f64], sample_rate: f64) -> f64 {
    zero_crossings(samples) as f64 / 2.0 / (samples.len() as f64 / sample_rate)
}

// ═══════════════════════════════════════════════════════════════════════════
// Compilation tests
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bridged_t_c3_compiles() {
    let pedal = parse_pedal_file(BRIDGED_T_C3).expect("parse failed");
    compile_pedal(&pedal, SAMPLE_RATE).expect("compile failed");
}

#[test]
fn bridged_t_minimal_compiles() {
    let pedal = parse_pedal_file(BRIDGED_T_MINIMAL).expect("parse failed");
    compile_pedal(&pedal, SAMPLE_RATE).expect("compile failed");
}

// ═══════════════════════════════════════════════════════════════════════════
// Trigger produces output
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bridged_t_trigger_produces_output() {
    let output = trigger_and_collect(BRIDGED_T_C3, 48, 48000);
    let r = rms(&output);
    eprintln!("  bridged_t C3 — RMS = {:.6e}", r);
    assert!(r > 1e-6, "bridged_t: output too quiet (RMS={r:.2e})");
}

#[test]
fn bridged_t_minimal_trigger_produces_output() {
    let output = trigger_and_collect(BRIDGED_T_MINIMAL, 48, 48000);
    let r = rms(&output);
    eprintln!("  bridged_t minimal — RMS = {:.6e}", r);
    assert!(r > 1e-6, "bridged_t minimal: output too quiet (RMS={r:.2e})");
}

// ═══════════════════════════════════════════════════════════════════════════
// Output is finite (no NaN/Inf)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bridged_t_output_is_finite() {
    let output = trigger_and_collect(BRIDGED_T_C3, 48, 48000);
    assert!(
        output.iter().all(|v| v.is_finite()),
        "bridged_t: NaN/Inf in output"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Frequency validation — the core diagnostic
// ═══════════════════════════════════════════════════════════════════════════

/// The bridged-T resonator should oscillate near f0 = 1/(2π·R·C).
/// With R=150k, C=8.2nF: f0 ≈ 129.5 Hz (C3 = 130.81 Hz).
///
/// DIAGNOSTIC: If this fails with freq ≈ 0 or very low, the compiler
/// is not modeling the bridged-T resonance (treating it as flat gain).
#[test]
fn bridged_t_oscillates_at_expected_frequency() {
    let output = trigger_and_collect(BRIDGED_T_C3, 48, 48000);

    // Analyze multiple time windows
    let windows = [
        ("0-10ms", 0usize, 480usize),
        ("0-50ms", 0, 2400),
        ("10-100ms", 480, 4320),
        ("50-250ms", 2400, 9600),
        ("250-500ms", 12000, 12000),
        ("500-1000ms", 24000, 24000),
    ];

    for (label, start, len) in windows {
        let end = (start + len).min(output.len());
        let window = &output[start..end];
        let freq = estimate_freq(window, SAMPLE_RATE);
        let pk = window.iter().map(|v| v.abs()).fold(0.0f64, f64::max);
        let r = rms(window);
        eprintln!(
            "  {label}: freq={freq:.1}Hz peak={pk:.3e} rms={r:.3e}"
        );
    }

    // Use Goertzel to check spectral energy at expected frequency
    // Skip initial transient (first 10ms), use 200ms window
    let analysis_start = 480; // 10ms
    let analysis_len = 9600; // 200ms
    let analysis_end = (analysis_start + analysis_len).min(output.len());
    let window = &output[analysis_start..analysis_end];

    let target_freq = 129.5; // f0 for R=150k, C=8.2n
    let power_at_f0 = goertzel_power(window, SAMPLE_RATE, target_freq);
    let power_at_dc = goertzel_power(window, SAMPLE_RATE, 5.0);
    let power_at_1k = goertzel_power(window, SAMPLE_RATE, 1000.0);

    eprintln!("  Goertzel: f0={target_freq:.1}Hz power={power_at_f0:.3e}");
    eprintln!("  Goertzel: DC (5Hz) power={power_at_dc:.3e}");
    eprintln!("  Goertzel: 1kHz power={power_at_1k:.3e}");

    // The resonator should have dominant energy at f0
    assert!(
        power_at_f0 > power_at_1k * 10.0,
        "bridged_t: f0 should dominate over 1kHz (f0={power_at_f0:.3e}, 1k={power_at_1k:.3e})"
    );

    // Zero-crossing frequency should be near 130 Hz (within octave)
    let freq = estimate_freq(window, SAMPLE_RATE);
    assert!(
        freq > 65.0 && freq < 260.0,
        "bridged_t: frequency {freq:.1}Hz outside expected range (65-260 Hz for C3)"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Decay behavior — resonator should ring and decay, not click
// ═══════════════════════════════════════════════════════════════════════════

/// A properly resonating bridged-T should have sustained output that
/// decays over hundreds of milliseconds (controlled by R_fb).
/// If it just "clicks" (impulse dies in <10ms), the feedback loop isn't working.
#[test]
fn bridged_t_sustains_beyond_impulse() {
    let output = trigger_and_collect(BRIDGED_T_C3, 48, 48000);

    // First 10ms captures the trigger impulse
    let impulse_rms = rms(&output[..480]);
    // 50-200ms should still have significant energy if resonating
    let sustained_rms = rms(&output[2400..9600]);
    // 200-500ms — should still be audible
    let late_rms = rms(&output[9600..24000]);

    eprintln!("  impulse (0-10ms) RMS = {impulse_rms:.3e}");
    eprintln!("  sustained (50-200ms) RMS = {sustained_rms:.3e}");
    eprintln!("  late (200-500ms) RMS = {late_rms:.3e}");

    // Sustained output should be at least 1% of impulse peak
    // (a flat-gain stage would have sustained ≈ 0)
    assert!(
        sustained_rms > impulse_rms * 0.01,
        "bridged_t: output dies too fast — no resonance \
         (impulse={impulse_rms:.3e}, sustained={sustained_rms:.3e})"
    );
}

/// The resonator should eventually decay (not diverge/oscillate forever).
#[test]
fn bridged_t_decays_over_time() {
    let output = trigger_and_collect(BRIDGED_T_C3, 48, 96000); // 2 seconds

    let first_quarter = rms(&output[..24000]);
    let last_quarter = rms(&output[72000..]);

    eprintln!("  first 0.5s RMS = {first_quarter:.3e}");
    eprintln!("  last 0.5s RMS = {last_quarter:.3e}");

    assert!(
        last_quarter < first_quarter,
        "bridged_t: should decay over time"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Diagnostic: signal shape analysis
// ═══════════════════════════════════════════════════════════════════════════

/// Print first samples for visual inspection of trigger response shape.
#[test]
fn bridged_t_impulse_response_shape() {
    let output = trigger_and_collect(BRIDGED_T_C3, 48, 4800); // 100ms

    eprintln!("  First 30 samples:");
    for (i, &v) in output.iter().take(30).enumerate() {
        eprintln!("    [{i:3}] {v:+.6e}");
    }

    // Check if output is a decaying sinusoid (alternating sign)
    // or a monotonic decay (impulse response of flat gain stage)
    let sign_changes_0_5ms = zero_crossings(&output[..240]); // first 5ms
    let sign_changes_5_50ms = zero_crossings(&output[240..2400]); // 5-50ms

    eprintln!("  zero crossings 0-5ms: {sign_changes_0_5ms}");
    eprintln!("  zero crossings 5-50ms: {sign_changes_5_50ms}");

    // A resonator at 130 Hz should have ~6 zero crossings per 5ms
    // and ~60 per 45ms. A flat impulse would have 0-2 crossings.
    assert!(
        sign_changes_5_50ms > 10,
        "bridged_t: expected oscillating output, got {sign_changes_5_50ms} \
         zero crossings in 5-50ms (resonator at 130Hz should have ~60)"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Multi-voice sine_drum tests (from pedalkernel-pro)
// ═══════════════════════════════════════════════════════════════════════════

/// Full sine_drum compilation (12 voices) — verifies the compiler handles
/// the large circuit without crashing or producing NaN.
#[test]
fn sine_drum_full_compiles_and_produces_output() {
    let src = include_str!(
        "../../../pedalkernel-pro/pedals/new/synth/sine_drum.pedal"
    );
    let pedal = parse_pedal_file(src).expect("parse sine_drum");
    let mut proc = compile_pedal(&pedal, SAMPLE_RATE).expect("compile sine_drum");

    // Trigger C3
    proc.note_on(48, 127);

    let output: Vec<f64> = (0..48000).map(|_| proc.process(0.0)).collect();
    let r = rms(&output);
    eprintln!("  sine_drum C3 full — RMS = {r:.6e}");

    assert!(output.iter().all(|v| v.is_finite()), "NaN/Inf in sine_drum");
    assert!(r > 1e-6, "sine_drum: no output (RMS={r:.2e})");
}

/// Diagnostic: compare spectral content of sine_drum C3 vs expected 130 Hz.
#[test]
fn sine_drum_spectral_analysis() {
    let src = include_str!(
        "../../../pedalkernel-pro/pedals/new/synth/sine_drum.pedal"
    );
    let pedal = parse_pedal_file(src).expect("parse sine_drum");
    let mut proc = compile_pedal(&pedal, SAMPLE_RATE).expect("compile sine_drum");

    proc.note_on(48, 127);
    let output: Vec<f64> = (0..48000).map(|_| proc.process(0.0)).collect();

    // Analyze 10-200ms window
    let window = &output[480..9600];

    let freqs = [65.0, 130.0, 196.0, 261.0, 440.0, 1000.0, 5000.0];
    for &f in &freqs {
        let p = goertzel_power(window, SAMPLE_RATE, f);
        eprintln!("  sine_drum Goertzel at {f:.0}Hz: {p:.3e}");
    }

    let freq = estimate_freq(window, SAMPLE_RATE);
    eprintln!("  sine_drum zero-crossing freq: {freq:.1} Hz");
}
