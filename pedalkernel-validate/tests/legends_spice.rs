//! SPICE validation for 'Legends' drive pedals (public spice / private .pedal).
//!
//! # Skip behaviour (CI)
//!
//! When the pro repo is absent, [`load_pro_pedal_sub`] returns `None` and
//! [`skip_if_missing!`] causes the test function to return `()` — Rust's test
//! harness records that as a pass, never a failure.  This is the canonical
//! mechanism for tests that require proprietary assets.
//!
//! # Golden bootstrap
//!
//! Run with `PEDALKERNEL_BOOTSTRAP_LEGENDS=1` to regenerate the ngspice
//! golden files committed in `golden/legends/screamer/`.  The golden must be
//! derived from the SPICE deck, NOT from WDF output, so that the two are
//! independent references.
//!
//! # Test: screamer_wdf_vs_spice
//!
//! Loads `screamer.pedal` at runtime from the pro repo, compiles it via the
//! WDF engine, drives the same stimulus as the SPICE deck, then measures the
//! error against the committed ngspice golden.  Reports actual numbers without
//! a hard pass threshold — the goal is visibility, not gating.

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use pedalkernel_validate::npy;
use pedalkernel_validate::pro_pedal::load_pro_pedal_sub;
use pedalkernel_validate::spice::{SpiceConfig, SpiceRunner};
use pedalkernel_validate::{metrics, skip_if_missing};
use std::f64::consts::PI;
use std::path::PathBuf;

/// Output sample rate used for both SPICE golden and WDF comparison.
const SR: f64 = 48_000.0;

/// SPICE oversample factor (internal simulation rate = SR * OVERSAMPLE).
const OVERSAMPLE: u32 = 4;

/// Duration of the 1 kHz validation sine (seconds).
const SINE_DURATION: f64 = 0.1;

/// Amplitude of the validation sine at the pedal input (volts, ~-20 dBu).
const SINE_AMP: f64 = 0.1;

/// Test frequency for the sine validation signal.
const TEST_FREQ: f64 = 1_000.0;

fn spice_circuit_path() -> PathBuf {
    let manifest = env!("CARGO_MANIFEST_DIR");
    PathBuf::from(format!("{manifest}/spice-circuits/legends/screamer.spice"))
}

fn golden_dir() -> PathBuf {
    let manifest = env!("CARGO_MANIFEST_DIR");
    PathBuf::from(format!("{manifest}/golden/legends/screamer"))
}

// ---------------------------------------------------------------------------
// Bootstrap: generate the ngspice golden (gated on env var)
// ---------------------------------------------------------------------------

/// Generate (or regenerate) the ngspice golden for the Screamer.
///
/// Run with:
/// ```
/// PEDALKERNEL_BOOTSTRAP_LEGENDS=1 cargo test -p pedalkernel-validate \
///   --test legends_spice bootstrap_screamer_golden -- --nocapture
/// ```
///
/// This runs ngspice on `spice-circuits/legends/screamer.spice` and writes
/// the output to `golden/legends/screamer/sine_1k.npy`.  It is the ONLY
/// authorised way to update that golden — never bootstrap from WDF output.
#[test]
fn bootstrap_screamer_golden() {
    if std::env::var("PEDALKERNEL_BOOTSTRAP_LEGENDS").as_deref() != Ok("1") {
        eprintln!(
            "  SKIP bootstrap_screamer_golden: set PEDALKERNEL_BOOTSTRAP_LEGENDS=1 to run"
        );
        return;
    }

    let circuit = spice_circuit_path();
    assert!(
        circuit.exists(),
        "screamer.spice not found at {circuit:?}"
    );

    let config = SpiceConfig {
        sample_rate: SR as u32,
        oversample: OVERSAMPLE,
    };
    let runner = SpiceRunner::new(config.clone());

    // 1 kHz sine at the internal rate.
    let internal_rate = config.internal_rate();
    let n_internal = (SINE_DURATION * internal_rate) as usize;
    let input_internal: Vec<f64> = (0..n_internal)
        .map(|i| SINE_AMP * (2.0 * PI * TEST_FREQ * i as f64 / internal_rate).sin())
        .collect();

    let golden_output = runner
        .simulate(&circuit, &input_internal, "v_out")
        .expect("ngspice simulation failed for screamer golden bootstrap");

    let dir = golden_dir();
    std::fs::create_dir_all(&dir).expect("create golden/legends/screamer");
    let golden_path = dir.join("sine_1k.npy");
    npy::write_f64(&golden_path, &golden_output)
        .expect("write screamer golden sine_1k.npy");

    eprintln!(
        "  bootstrapped golden: {} samples → {golden_path:?}",
        golden_output.len()
    );
}

// ---------------------------------------------------------------------------
// Main validation test: WDF vs ngspice golden
// ---------------------------------------------------------------------------

/// Validate the Screamer WDF model against the ngspice golden.
///
/// Requires:
///   - pro repo present at `../../pedalkernel-pro/` (or equivalent depth)
///   - `golden/legends/screamer/sine_1k.npy` committed in this repo
///
/// When the pro repo is absent the test **skips** (returns without failure)
/// so that public CI remains green.  This is the canonical CI-skip mechanism.
#[test]
fn screamer_wdf_vs_spice() {
    // --- Load .pedal from pro repo (skip if absent) ---
    let source = skip_if_missing!(
        load_pro_pedal_sub("pedals/legends/screamer.pedal"),
        "pedals/legends/screamer.pedal"
    );

    // --- Load the ngspice golden ---
    let golden_path = golden_dir().join("sine_1k.npy");
    if !golden_path.exists() {
        eprintln!(
            "  SKIP screamer_wdf_vs_spice: golden not found at {golden_path:?}; \
             run with PEDALKERNEL_BOOTSTRAP_LEGENDS=1 first"
        );
        return;
    }
    let golden = npy::read_f64(&golden_path).expect("read screamer golden");

    // --- Compile the pedal ---
    let def = parse_pedal_file(&source).expect("parse screamer.pedal");
    let mut proc = compile_pedal(&def, SR).expect("compile screamer.pedal");

    // Set controls to match SPICE deck (Drive=0.5, Tone=0.5, Level=0.7).
    proc.set_control("Drive", 0.5);
    proc.set_control("Tone", 0.5);
    proc.set_control("Level", 0.7);

    // --- Generate the WDF output ---
    let n = golden.len();
    let input: Vec<f64> = (0..n)
        .map(|i| SINE_AMP * (2.0 * PI * TEST_FREQ * i as f64 / SR).sin())
        .collect();

    // Warm up for half the buffer before recording.
    let warmup = n / 2;
    let mut all_output = Vec::with_capacity(n + warmup);
    let warmup_input: Vec<f64> = (0..warmup)
        .map(|i| SINE_AMP * (2.0 * PI * TEST_FREQ * i as f64 / SR).sin())
        .collect();
    for &s in &warmup_input {
        all_output.push(proc.process(s));
    }
    all_output.clear();
    for &s in &input {
        all_output.push(proc.process(s));
    }
    let wdf_output = all_output;

    // --- Compute metrics ---
    let result = metrics::compare(&wdf_output, &golden, SR, Some(TEST_FREQ));

    // Trim transient: skip first 10 ms for DC-blocking / coupling-cap settling.
    let trim = (0.01 * SR) as usize;
    let wdf_trim = &wdf_output[trim.min(wdf_output.len())..];
    let golden_trim = &golden[trim.min(golden.len())..];
    let result_trim = metrics::compare(wdf_trim, golden_trim, SR, Some(TEST_FREQ));

    // --- Report (no hard threshold — surface the gap) ---
    eprintln!("  Screamer WDF vs ngspice (full signal):");
    eprintln!(
        "    normalized RMS error : {:.1} dB",
        result.normalized_rms_error_db
    );
    eprintln!(
        "    peak error           : {:.1} dB",
        result.peak_error_db
    );
    eprintln!("  After {trim}-sample transient trim:");
    eprintln!(
        "    normalized RMS error : {:.1} dB",
        result_trim.normalized_rms_error_db
    );
    eprintln!(
        "    peak error           : {:.1} dB",
        result_trim.peak_error_db
    );
    eprintln!(
        "    WDF RMS amplitude    : {:.4} V",
        rms(&wdf_trim)
    );
    eprintln!(
        "    ngspice RMS amplitude: {:.4} V",
        rms(&golden_trim)
    );

    // Sanity: WDF output must not be silence — compilation/processing worked.
    assert!(
        rms(&wdf_output) > 1e-6,
        "WDF output is silence (RMS < 1e-6): compilation or processing failed"
    );
}

// ---------------------------------------------------------------------------
// Prove the skip: test skips cleanly when the pro .pedal is absent
// ---------------------------------------------------------------------------

/// Prove that the skip mechanism returns cleanly (not a failure) when the
/// pro .pedal path does not exist.
///
/// This test always runs.  It passes unconditionally because `skip_if_missing!`
/// returns `()` — the same result as a successful test — when `None` is given.
#[test]
fn screamer_skip_when_pro_absent() {
    // Deliberately pass a path that cannot exist.
    let source = skip_if_missing!(
        load_pro_pedal_sub("pedals/legends/__nonexistent_probe__.pedal"),
        "pedals/legends/__nonexistent_probe__.pedal"
    );
    // If we reach here the file somehow exists — still not a failure.
    let _ = source;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn rms(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    (signal.iter().map(|&x| x * x).sum::<f64>() / signal.len() as f64).sqrt()
}
