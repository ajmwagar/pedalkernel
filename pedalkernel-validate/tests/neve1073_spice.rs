//! SPICE validation for the Neve 1073 — 500-series target (public spice / private .pedal).
//!
//! First milestone: the **BA283 discrete Class-A gain block** in isolation
//! (`pedals/500/1073/sub/ba283.pedal`), validated 1:1 against an ngspice golden.
//! The full mic preamp (with input/output transformers) and the program EQ are
//! layered on once this clean-signal block matches — see the 1073 README in the
//! pro repo.
//!
//! # Architecture (house pattern, mirrors `legends_spice.rs`)
//!
//! The `.pedal` lives in the **private** `pedalkernel-pro` repo and is loaded at
//! runtime via [`load_pro_pedal_sub`]; the test SKIPS (counts as pass) on CI when
//! the pro repo is absent. The ngspice deck is **public**, here in this crate.
//!
//! # Golden bootstrap
//!
//! ```
//! PEDALKERNEL_BOOTSTRAP_1073=1 cargo test -p pedalkernel-validate \
//!   --test neve1073_spice bootstrap_ba283_golden -- --nocapture
//! ```
//!
//! The golden is derived from the SPICE deck, NOT from WDF output, so the two
//! remain independent references.

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use pedalkernel_validate::npy;
use pedalkernel_validate::pro_pedal::load_pro_pedal_sub;
use pedalkernel_validate::spice::{SpiceConfig, SpiceRunner};
use pedalkernel_validate::{metrics, skip_if_missing};
use std::f64::consts::PI;
use std::path::PathBuf;

const SR: f64 = 48_000.0;
const OVERSAMPLE: u32 = 4;
const SINE_DURATION: f64 = 0.1;
const SINE_AMP: f64 = 0.1;
const TEST_FREQ: f64 = 1_000.0;

const PRO_PATH: &str = "pedals/500/1073/sub/ba283.pedal";
const DECK: &str = "neve1073_ba283";
const GOLDEN: &str = "neve1073_ba283";

fn spice_path(name: &str) -> PathBuf {
    let manifest = env!("CARGO_MANIFEST_DIR");
    PathBuf::from(format!("{manifest}/spice-circuits/active/{name}.spice"))
}

fn golden_dir(name: &str) -> PathBuf {
    let manifest = env!("CARGO_MANIFEST_DIR");
    PathBuf::from(format!("{manifest}/golden/active/{name}"))
}

fn rms(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    (signal.iter().map(|&x| x * x).sum::<f64>() / signal.len() as f64).sqrt()
}

/// Simulate the ngspice deck and write `golden/active/<GOLDEN>/sine_1k.npy`.
#[test]
fn bootstrap_ba283_golden() {
    if std::env::var("PEDALKERNEL_BOOTSTRAP_1073").as_deref() != Ok("1") {
        eprintln!("  SKIP bootstrap_ba283_golden: set PEDALKERNEL_BOOTSTRAP_1073=1 to run");
        return;
    }

    let circuit = spice_path(DECK);
    assert!(circuit.exists(), "{DECK}.spice not found at {circuit:?}");

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
        .unwrap_or_else(|e| panic!("ngspice simulation failed for {GOLDEN} bootstrap: {e}"));

    let dir = golden_dir(GOLDEN);
    std::fs::create_dir_all(&dir).unwrap_or_else(|e| panic!("create golden/active/{GOLDEN}: {e}"));
    let golden_path = dir.join("sine_1k.npy");
    npy::write_f64(&golden_path, &golden_output)
        .unwrap_or_else(|e| panic!("write {GOLDEN} golden: {e}"));

    eprintln!(
        "  bootstrapped {GOLDEN} golden: {} samples → {golden_path:?}",
        golden_output.len()
    );
}

/// Compare the WDF BA283 block against the ngspice golden (GAIN = 0.5).
#[test]
fn ba283_wdf_vs_spice() {
    let source = skip_if_missing!(load_pro_pedal_sub(PRO_PATH), PRO_PATH);

    let golden_path = golden_dir(GOLDEN).join("sine_1k.npy");
    if !golden_path.exists() {
        eprintln!(
            "  SKIP ba283_wdf_vs_spice: golden not found at {golden_path:?}; \
             run with PEDALKERNEL_BOOTSTRAP_1073=1 first"
        );
        return;
    }
    let golden = npy::read_f64(&golden_path).expect("read golden");

    let def = parse_pedal_file(&source).unwrap_or_else(|e| panic!("parse {PRO_PATH}: {e}"));
    let mut proc = compile_pedal(&def, SR).unwrap_or_else(|e| panic!("compile {PRO_PATH}: {e}"));
    proc.set_control("Gain", 0.5);

    let n = golden.len();
    let input: Vec<f64> = (0..n)
        .map(|i| SINE_AMP * (2.0 * PI * TEST_FREQ * i as f64 / SR).sin())
        .collect();

    // Warm up half a buffer before recording (DC-block / coupling-cap settling).
    let warmup = n / 2;
    for i in 0..warmup {
        proc.process(SINE_AMP * (2.0 * PI * TEST_FREQ * i as f64 / SR).sin());
    }
    let wdf_output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

    let trim = (0.01 * SR) as usize;
    let wdf_trim = &wdf_output[trim.min(wdf_output.len())..];
    let golden_trim = &golden[trim.min(golden.len())..];
    let result = metrics::compare(wdf_trim, golden_trim, SR, Some(TEST_FREQ));

    eprintln!("  {GOLDEN} WDF vs ngspice (after {trim}-sample trim):");
    eprintln!("    normalized RMS error : {:.1} dB", result.normalized_rms_error_db);
    eprintln!("    peak error           : {:.1} dB", result.peak_error_db);
    eprintln!("    WDF RMS amplitude    : {:.4} V", rms(wdf_trim));
    eprintln!("    ngspice RMS amplitude: {:.4} V", rms(golden_trim));
    if rms(golden_trim) > 1e-9 {
        eprintln!(
            "    WDF-vs-ngspice gap   : {:.1} dB (positive = WDF hotter)",
            20.0 * (rms(wdf_trim) / rms(golden_trim)).log10()
        );
    }
    if let Some(thd_err) = result.thd_error_db {
        eprintln!("    THD error            : {:.1} dB", thd_err);
    }

    assert!(
        rms(&wdf_output) > 1e-6,
        "{GOLDEN} WDF output is silence (RMS < 1e-6): compilation or processing failed"
    );

    // No hard gate yet — thresholds are set after the first measured run
    // (house style: measure, then gate at measured + ~3 dB margin).
}

/// Prove the skip mechanism works when the pro repo is absent.
#[test]
fn ba283_skip_when_pro_absent() {
    let source = skip_if_missing!(
        load_pro_pedal_sub("pedals/500/1073/__nonexistent_ba283__.pedal"),
        "pedals/500/1073/__nonexistent_ba283__.pedal"
    );
    let _ = source;
}
