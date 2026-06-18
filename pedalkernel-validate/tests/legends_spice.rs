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
//! golden files committed in `golden/legends/<pedal>/`.  The golden must be
//! derived from the SPICE deck, NOT from WDF output, so that the two are
//! independent references.
//!
//! # Tests
//!
//! Each pedal has three test functions:
//!   - `bootstrap_<pedal>_golden`      — writes ngspice golden (gated on env var)
//!   - `<pedal>_wdf_vs_spice`          — WDF vs golden comparison
//!   - `<pedal>_skip_when_pro_absent`  — proves skip mechanism with a bogus path

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

// ============================================================================
// Helpers
// ============================================================================

fn spice_path(name: &str) -> PathBuf {
    let manifest = env!("CARGO_MANIFEST_DIR");
    PathBuf::from(format!("{manifest}/spice-circuits/legends/{name}.spice"))
}

fn golden_dir(name: &str) -> PathBuf {
    let manifest = env!("CARGO_MANIFEST_DIR");
    PathBuf::from(format!("{manifest}/golden/legends/{name}"))
}

fn rms(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    (signal.iter().map(|&x| x * x).sum::<f64>() / signal.len() as f64).sqrt()
}

/// Run a bootstrap: simulate the named spice deck and write `golden/<name>/sine_1k.npy`.
fn run_bootstrap(name: &str, output_node: &str) {
    let circuit = spice_path(name);
    assert!(circuit.exists(), "{name}.spice not found at {circuit:?}");

    let config = SpiceConfig {
        sample_rate: SR as u32,
        oversample: OVERSAMPLE,
    };
    let runner = SpiceRunner::new(config.clone());

    let internal_rate = config.internal_rate();
    let n_internal = (SINE_DURATION * internal_rate) as usize;
    let input_internal: Vec<f64> = (0..n_internal)
        .map(|i| SINE_AMP * (2.0 * PI * TEST_FREQ * i as f64 / internal_rate).sin())
        .collect();

    let golden_output = runner
        .simulate(&circuit, &input_internal, output_node)
        .unwrap_or_else(|e| panic!("ngspice simulation failed for {name} golden bootstrap: {e}"));

    let dir = golden_dir(name);
    std::fs::create_dir_all(&dir).unwrap_or_else(|e| panic!("create golden/legends/{name}: {e}"));
    let golden_path = dir.join("sine_1k.npy");
    npy::write_f64(&golden_path, &golden_output)
        .unwrap_or_else(|e| panic!("write {name} golden sine_1k.npy: {e}"));

    eprintln!(
        "  bootstrapped {name} golden: {} samples → {golden_path:?}",
        golden_output.len()
    );
}

/// Run the WDF-vs-golden comparison for a pedal.
///
/// `controls` is a slice of `(name, value)` pairs to set on the compiled pedal.
fn run_wdf_vs_spice(name: &str, pro_path: &str, controls: &[(&str, f64)]) {
    // Load .pedal from pro repo (skip if absent)
    let source = skip_if_missing!(load_pro_pedal_sub(pro_path), pro_path);

    // Load the ngspice golden
    let golden_path = golden_dir(name).join("sine_1k.npy");
    if !golden_path.exists() {
        eprintln!(
            "  SKIP {name}_wdf_vs_spice: golden not found at {golden_path:?}; \
             run with PEDALKERNEL_BOOTSTRAP_LEGENDS=1 first"
        );
        return;
    }
    let golden = npy::read_f64(&golden_path).expect("read golden");

    // Compile the pedal
    let def = parse_pedal_file(&source).unwrap_or_else(|e| panic!("parse {pro_path}: {e}"));
    let mut proc = compile_pedal(&def, SR).unwrap_or_else(|e| panic!("compile {pro_path}: {e}"));

    // Set controls to match SPICE deck positions
    for &(ctrl, val) in controls {
        proc.set_control(ctrl, val);
    }

    // Generate the WDF output
    let n = golden.len();
    let input: Vec<f64> = (0..n)
        .map(|i| SINE_AMP * (2.0 * PI * TEST_FREQ * i as f64 / SR).sin())
        .collect();

    // Warm up for half the buffer before recording
    let warmup = n / 2;
    let warmup_input: Vec<f64> = (0..warmup)
        .map(|i| SINE_AMP * (2.0 * PI * TEST_FREQ * i as f64 / SR).sin())
        .collect();
    for &s in &warmup_input {
        proc.process(s);
    }
    let wdf_output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

    // Compute metrics
    let result = metrics::compare(&wdf_output, &golden, SR, Some(TEST_FREQ));

    // Trim transient: skip first 10ms for DC-blocking / coupling-cap settling
    let trim = (0.01 * SR) as usize;
    let wdf_trim = &wdf_output[trim.min(wdf_output.len())..];
    let golden_trim = &golden[trim.min(golden.len())..];
    let result_trim = metrics::compare(wdf_trim, golden_trim, SR, Some(TEST_FREQ));

    // Report (no hard threshold — surface the gap)
    eprintln!("  {name} WDF vs ngspice (full signal):");
    eprintln!(
        "    normalized RMS error : {:.1} dB",
        result.normalized_rms_error_db
    );
    eprintln!("    peak error           : {:.1} dB", result.peak_error_db);
    eprintln!("  After {trim}-sample transient trim:");
    eprintln!(
        "    normalized RMS error : {:.1} dB",
        result_trim.normalized_rms_error_db
    );
    eprintln!(
        "    peak error           : {:.1} dB",
        result_trim.peak_error_db
    );
    eprintln!("    WDF RMS amplitude    : {:.4} V", rms(wdf_trim));
    eprintln!("    ngspice RMS amplitude: {:.4} V", rms(golden_trim));
    eprintln!(
        "    WDF-vs-ngspice gap   : {:.1} dB (positive = WDF hotter)",
        if rms(golden_trim) > 1e-9 {
            20.0 * (rms(wdf_trim) / rms(golden_trim)).log10()
        } else {
            f64::NAN
        }
    );

    // Sanity: WDF output must not be silence
    assert!(
        rms(&wdf_output) > 1e-6,
        "{name} WDF output is silence (RMS < 1e-6): compilation or processing failed"
    );
}

// ============================================================================
// SCREAMER (TS-808)
// ============================================================================

fn screamer_spice_circuit_path() -> PathBuf {
    spice_path("screamer")
}

fn screamer_golden_dir() -> PathBuf {
    golden_dir("screamer")
}

/// Generate (or regenerate) the ngspice golden for the Screamer.
///
/// Run with:
/// ```
/// PEDALKERNEL_BOOTSTRAP_LEGENDS=1 cargo test -p pedalkernel-validate \
///   --test legends_spice bootstrap_screamer_golden -- --nocapture
/// ```
#[test]
fn bootstrap_screamer_golden() {
    if std::env::var("PEDALKERNEL_BOOTSTRAP_LEGENDS").as_deref() != Ok("1") {
        eprintln!("  SKIP bootstrap_screamer_golden: set PEDALKERNEL_BOOTSTRAP_LEGENDS=1 to run");
        return;
    }

    let circuit = screamer_spice_circuit_path();
    assert!(circuit.exists(), "screamer.spice not found at {circuit:?}");

    let config = SpiceConfig {
        sample_rate: SR as u32,
        oversample: OVERSAMPLE,
    };
    let runner = SpiceRunner::new(config.clone());

    let internal_rate = config.internal_rate();
    let n_internal = (SINE_DURATION * internal_rate) as usize;
    let input_internal: Vec<f64> = (0..n_internal)
        .map(|i| SINE_AMP * (2.0 * PI * TEST_FREQ * i as f64 / internal_rate).sin())
        .collect();

    let golden_output = runner
        .simulate(&circuit, &input_internal, "v_out")
        .expect("ngspice simulation failed for screamer golden bootstrap");

    let dir = screamer_golden_dir();
    std::fs::create_dir_all(&dir).expect("create golden/legends/screamer");
    let golden_path = dir.join("sine_1k.npy");
    npy::write_f64(&golden_path, &golden_output).expect("write screamer golden sine_1k.npy");

    eprintln!(
        "  bootstrapped golden: {} samples → {golden_path:?}",
        golden_output.len()
    );
}

/// Validate the Screamer WDF model against the ngspice golden.
#[test]
fn screamer_wdf_vs_spice() {
    // --- Load .pedal from pro repo (skip if absent) ---
    let source = skip_if_missing!(
        load_pro_pedal_sub("pedals/legends/screamer.pedal"),
        "pedals/legends/screamer.pedal"
    );

    // --- Load the ngspice golden ---
    let golden_path = screamer_golden_dir().join("sine_1k.npy");
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
    eprintln!("    peak error           : {:.1} dB", result.peak_error_db);
    eprintln!("  After {trim}-sample transient trim:");
    eprintln!(
        "    normalized RMS error : {:.1} dB",
        result_trim.normalized_rms_error_db
    );
    eprintln!(
        "    peak error           : {:.1} dB",
        result_trim.peak_error_db
    );
    eprintln!("    WDF RMS amplitude    : {:.4} V", rms(&wdf_trim));
    eprintln!("    ngspice RMS amplitude: {:.4} V", rms(&golden_trim));

    // Sanity: WDF output must not be silence — compilation/processing worked.
    assert!(
        rms(&wdf_output) > 1e-6,
        "WDF output is silence (RMS < 1e-6): compilation or processing failed"
    );
}

/// Prove that the skip mechanism returns cleanly when the pro .pedal is absent.
#[test]
fn screamer_skip_when_pro_absent() {
    let source = skip_if_missing!(
        load_pro_pedal_sub("pedals/legends/__nonexistent_probe__.pedal"),
        "pedals/legends/__nonexistent_probe__.pedal"
    );
    let _ = source;
}

// ============================================================================
// RATKING (ProCo RAT v1a)
// ============================================================================

/// Generate (or regenerate) the ngspice golden for the Ratking.
///
/// Run with:
/// ```
/// PEDALKERNEL_BOOTSTRAP_LEGENDS=1 cargo test -p pedalkernel-validate \
///   --test legends_spice bootstrap_ratking_golden -- --nocapture
/// ```
///
/// The golden is derived from ngspice (`spice-circuits/legends/ratking.spice`),
/// NOT from WDF output, so they remain independent references.
#[test]
fn bootstrap_ratking_golden() {
    if std::env::var("PEDALKERNEL_BOOTSTRAP_LEGENDS").as_deref() != Ok("1") {
        eprintln!("  SKIP bootstrap_ratking_golden: set PEDALKERNEL_BOOTSTRAP_LEGENDS=1 to run");
        return;
    }
    run_bootstrap("ratking", "v_out");
}

/// Validate the Ratking (ProCo RAT v1a) WDF model against the ngspice golden.
///
/// Requires:
///   - pro repo present at `../../pedalkernel-pro/`
///   - `golden/legends/ratking/sine_1k.npy` committed in this repo
///
/// Controls match ratking.spice positions:
///   Distortion = 0.5 (50k in feedback path)
///   Filter     = 0.3 (fc ≈ 1.5kHz)
///   Volume     = 0.5 (wiper at midpoint)
#[test]
fn ratking_wdf_vs_spice() {
    run_wdf_vs_spice(
        "ratking",
        "pedals/legends/ratking_non_invert_v1a.pedal",
        &[("Distortion", 0.5), ("Filter", 0.3), ("Volume", 0.5)],
    );
}

/// Prove the skip mechanism for Ratking.
#[test]
fn ratking_skip_when_pro_absent() {
    let source = skip_if_missing!(
        load_pro_pedal_sub("pedals/legends/__nonexistent_ratking__.pedal"),
        "pedals/legends/__nonexistent_ratking__.pedal"
    );
    let _ = source;
}

// ============================================================================
// GOLDENROD (Klon Centaur)
// ============================================================================

/// Generate (or regenerate) the ngspice golden for the Goldenrod.
///
/// Run with:
/// ```
/// PEDALKERNEL_BOOTSTRAP_LEGENDS=1 cargo test -p pedalkernel-validate \
///   --test legends_spice bootstrap_goldenrod_golden -- --nocapture
/// ```
#[test]
fn bootstrap_goldenrod_golden() {
    if std::env::var("PEDALKERNEL_BOOTSTRAP_LEGENDS").as_deref() != Ok("1") {
        eprintln!("  SKIP bootstrap_goldenrod_golden: set PEDALKERNEL_BOOTSTRAP_LEGENDS=1 to run");
        return;
    }
    run_bootstrap("goldenrod", "v_out");
}

/// Validate the Goldenrod (Klon Centaur) WDF model against the ngspice golden.
///
/// Requires:
///   - pro repo present at `../../pedalkernel-pro/`
///   - `golden/legends/goldenrod/sine_1k.npy` committed in this repo
///
/// Controls match goldenrod.spice positions:
///   Gain   = 0.5 (dual-gang: Gain_A top=50k, Gain_B wiper at midpoint)
///   Treble = 0.5 (5k each half)
///   Output = 0.7 (audio taper: top=3k, bot=7k)
#[test]
fn goldenrod_wdf_vs_spice() {
    run_wdf_vs_spice(
        "goldenrod",
        "pedals/legends/goldenrod.pedal",
        &[("Gain", 0.5), ("Treble", 0.5), ("Output", 0.7)],
    );
}

/// Prove the skip mechanism for Goldenrod.
#[test]
fn goldenrod_skip_when_pro_absent() {
    let source = skip_if_missing!(
        load_pro_pedal_sub("pedals/legends/__nonexistent_goldenrod__.pedal"),
        "pedals/legends/__nonexistent_goldenrod__.pedal"
    );
    let _ = source;
}

// ============================================================================
// SD-1 (Boss SD-1)
// ============================================================================

/// Generate (or regenerate) the ngspice golden for the SD-1.
///
/// Run with:
/// ```
/// PEDALKERNEL_BOOTSTRAP_LEGENDS=1 cargo test -p pedalkernel-validate \
///   --test legends_spice bootstrap_sd1_golden -- --nocapture
/// ```
#[test]
fn bootstrap_sd1_golden() {
    if std::env::var("PEDALKERNEL_BOOTSTRAP_LEGENDS").as_deref() != Ok("1") {
        eprintln!("  SKIP bootstrap_sd1_golden: set PEDALKERNEL_BOOTSTRAP_LEGENDS=1 to run");
        return;
    }
    run_bootstrap("sd1", "v_out");
}

/// Validate the SD-1 (Boss SD-1) WDF model against the ngspice golden.
///
/// Requires:
///   - pro repo present at `../../pedalkernel-pro/`
///   - `golden/legends/sd1/sine_1k.npy` committed in this repo
///
/// Controls match sd1.spice positions:
///   Drive = 0.5 (250k each half of 500k pot)
///   Tone  = 0.5 (10k each half of 20k pot)
///   Level = 0.7 (audio taper: top=30k, bot=70k)
#[test]
fn sd1_wdf_vs_spice() {
    run_wdf_vs_spice(
        "sd1",
        "pedals/legends/sd1.pedal",
        &[("Drive", 0.5), ("Tone", 0.5), ("Level", 0.7)],
    );
}

/// Prove the skip mechanism for SD-1.
#[test]
fn sd1_skip_when_pro_absent() {
    let source = skip_if_missing!(
        load_pro_pedal_sub("pedals/legends/__nonexistent_sd1__.pedal"),
        "pedals/legends/__nonexistent_sd1__.pedal"
    );
    let _ = source;
}
