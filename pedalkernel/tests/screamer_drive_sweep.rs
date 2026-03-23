//! Regression test: SCREAMER pedal NaN/Inf crash at high Drive settings.
//!
//! Reproduces a known bug where the output diverges to NaN or Inf when
//! Drive exceeds approximately 60% on a sustained 440 Hz sine tone.
//!
//! The pedal file lives in pedalkernel-pro/pedals/legends/screamer.pedal,
//! resolved at runtime via $HOME/src/pedalkernel/pedalkernel-pro/...

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use std::path::Path;

// ---------------------------------------------------------------------------
// Helper: locate screamer.pedal from the sibling pedalkernel-pro repo.
// ---------------------------------------------------------------------------

fn screamer_pedal_path() -> std::path::PathBuf {
    // The pedal lives in ~/src/pedalkernel/pedalkernel-pro/pedals/legends/screamer.pedal.
    // We resolve via HOME so the path is stable across worktrees and CI checkouts.
    let home = std::env::var("HOME").expect("HOME env var not set");
    Path::new(&home).join("src/pedalkernel/pedalkernel-pro/pedals/legends/screamer.pedal")
}

// ---------------------------------------------------------------------------
// Drive sweep: 0.0 → 1.0 in 20 steps, 1 s of 440 Hz sine per step.
// ---------------------------------------------------------------------------

/// Full Drive sweep regression test.
///
/// For each Drive position in [0.0, 0.05, 0.10, …, 1.0]:
/// 1. Compile the SCREAMER pedal at 48 kHz.
/// 2. Set Drive to the test value (Tone = 0.5, Level = 0.7).
/// 3. Process 48 000 samples of a 440 Hz sine at amplitude 0.5.
/// 4. Inspect every output sample for NaN or Inf.
/// 5. Record peak output level and pass/fail status.
///
/// The test asserts that **no** output sample is non-finite at any Drive value.
/// It currently fails because high Drive settings produce NaN/Inf (known bug).
#[test]
fn screamer_no_nan_inf_across_drive_sweep() {
    let pedal_path = screamer_pedal_path();
    assert!(
        pedal_path.exists(),
        "screamer.pedal not found at {}: is pedalkernel-pro checked out?",
        pedal_path.display()
    );

    let src = std::fs::read_to_string(&pedal_path)
        .unwrap_or_else(|e| panic!("failed to read screamer.pedal: {e}"));

    // Drive sweep: 0.0, 0.05, 0.10, …, 1.0  (21 steps)
    let drive_steps: Vec<f64> = (0..=20).map(|i| i as f64 * 0.05).collect();

    // 1 second of 440 Hz sine at amplitude 0.5
    let input = sine(440.0, 1.0, SAMPLE_RATE);

    let mut failures: Vec<(f64, usize, f64)> = Vec::new(); // (drive, first_bad_idx, peak)

    for &drive in &drive_steps {
        // Re-compile for each Drive setting so state is fresh.
        let pedal = parse_pedal_file(&src)
            .unwrap_or_else(|e| panic!("parse screamer.pedal at drive={drive:.2}: {e}"));
        let mut proc = compile_pedal(&pedal, SAMPLE_RATE)
            .unwrap_or_else(|e| panic!("compile screamer.pedal at drive={drive:.2}: {e}"));

        proc.set_control("Drive", drive);
        proc.set_control("Tone", 0.5);
        proc.set_control("Level", 0.7);

        let mut peak_out: f64 = 0.0;
        let mut first_bad: Option<usize> = None;

        for (idx, &sample) in input.iter().enumerate() {
            let out = proc.process(sample);
            if !out.is_finite() {
                if first_bad.is_none() {
                    first_bad = Some(idx);
                }
            } else {
                peak_out = peak_out.max(out.abs());
            }
        }

        let passed = first_bad.is_none();
        eprintln!(
            "  [screamer sweep] drive={drive:.2}  peak={peak_out:.5}  {}",
            if passed { "PASS" } else { "FAIL (NaN/Inf)" }
        );

        if let Some(idx) = first_bad {
            failures.push((drive, idx, peak_out));
        }
    }

    assert!(
        failures.is_empty(),
        "SCREAMER produced NaN/Inf output at {} Drive setting(s):\n{}",
        failures.len(),
        failures
            .iter()
            .map(|(d, idx, pk)| format!(
                "  drive={d:.2}  first_bad_sample={idx}  peak_before_diverge={pk:.5}"
            ))
            .collect::<Vec<_>>()
            .join("\n")
    );
}

// ---------------------------------------------------------------------------
// Focused repro: single Drive value above the known crash threshold (~0.65).
// ---------------------------------------------------------------------------

/// Targeted repro at Drive=0.7 — the setting that most reliably triggers the crash.
///
/// Separated from the sweep so it can be run in isolation for quick bisection:
///   cargo test -p pedalkernel screamer_high_drive_no_nan -- --nocapture
#[test]
fn screamer_high_drive_no_nan() {
    let pedal_path = screamer_pedal_path();
    assert!(
        pedal_path.exists(),
        "screamer.pedal not found at {}: is pedalkernel-pro checked out?",
        pedal_path.display()
    );

    let src = std::fs::read_to_string(&pedal_path)
        .unwrap_or_else(|e| panic!("failed to read screamer.pedal: {e}"));

    let pedal = parse_pedal_file(&src).expect("parse screamer.pedal");
    let mut proc = compile_pedal(&pedal, SAMPLE_RATE).expect("compile screamer.pedal");

    proc.set_control("Drive", 0.7);
    proc.set_control("Tone", 0.5);
    proc.set_control("Level", 0.7);

    let input = sine(440.0, 1.0, SAMPLE_RATE);

    let mut peak_out: f64 = 0.0;
    for (idx, &sample) in input.iter().enumerate() {
        let out = proc.process(sample);
        assert!(
            out.is_finite(),
            "SCREAMER output is non-finite at sample {idx} (Drive=0.7): out={out}"
        );
        peak_out = peak_out.max(out.abs());
    }

    eprintln!("  [screamer high drive] Drive=0.7  peak={peak_out:.5}  PASS");
}
