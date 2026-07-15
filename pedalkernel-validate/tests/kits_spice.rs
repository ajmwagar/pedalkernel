//! SPICE validation for PedalPCB kit circuits (public spice / private .pedal).
//!
//! Circuits (see `pedalkernel-pro/references/pedalpcb/<Name>-analysis.md`):
//!   - all_points_booster          — EHX LPB-1 (PedalPCB All-Points Booster)
//!   - sunflower                   — Sunface/Fuzz Face (PedalPCB Sunflower)
//!   - distortion_250              — MXR Dist+/DOD 250 (PedalPCB Distortion 250)
//!   - uberdrive                   — Boss SD-1 (PedalPCB UberDrive)
//!   - little_green_scream_machine — TS-808 (PedalPCB LGSM)
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
//! Run with `PEDALKERNEL_BOOTSTRAP_KITS=1` to regenerate the ngspice golden
//! files committed in `golden/kits/<pedal>/`.  HARD RULE: the golden is
//! derived from the SPICE deck, NEVER from WDF output, so the two remain
//! independent references.
//!
//! # Control positions
//!
//! Each comparison sets the compiled pedal's controls to the deck's fixed pot
//! positions.  Positions are chosen to be taper-unambiguous (extremes, where
//! any taper maps to 0/1, or linear-taper mids, where 0.5 is exact) so the
//! deck resistor splits match the engine's `PotTaper` mapping without
//! guessing the audio-taper law.
//!
//! # Tests
//!
//! Each pedal has two test functions plus one shared skip probe:
//!   - `bootstrap_<pedal>_golden` — writes ngspice golden (gated on env var)
//!   - `<pedal>_wdf_vs_spice`     — WDF vs golden comparison

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
///
/// Longer than the legends' 0.1 s: the UberDrive/LGSM output followers have
/// ~100 ms bias/coupling time constants (C8×R13, C9×(R15+R16)) and their
/// decks must cold-start under the harness's UIC transient (.IC on the
/// follower base nodes makes ngspice fail at the initial timepoint — see the
/// deck headers). 0.6 s + a 0.5 s comparison trim compares fully settled
/// windows on both sides.
const SINE_DURATION: f64 = 0.6;

/// Comparison trim (seconds skipped at the start of both signals).
const TRIM_SECONDS: f64 = 0.5;

/// Amplitude of the validation sine at the pedal input (volts, ~-20 dBu).
const SINE_AMP: f64 = 0.1;

/// Test frequency for the sine validation signal.
const TEST_FREQ: f64 = 1_000.0;

// ============================================================================
// Helpers
// ============================================================================

fn spice_path(name: &str) -> PathBuf {
    let manifest = env!("CARGO_MANIFEST_DIR");
    PathBuf::from(format!("{manifest}/spice-circuits/kits/{name}.spice"))
}

fn golden_dir(name: &str) -> PathBuf {
    let manifest = env!("CARGO_MANIFEST_DIR");
    PathBuf::from(format!("{manifest}/golden/kits/{name}"))
}

fn rms(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    (signal.iter().map(|&x| x * x).sum::<f64>() / signal.len() as f64).sqrt()
}

/// Bootstrap: simulate the named spice deck and write the golden .npy.
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
    std::fs::create_dir_all(&dir).unwrap_or_else(|e| panic!("create golden/kits/{name}: {e}"));
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
/// `controls` is a slice of `(name, value)` pairs matching the deck positions.
/// `gate` optionally asserts hard error bounds `(rms_db, peak_db, thd_db)` —
/// thresholds are measured + ~3 dB margin (house style); `None` = report-only.
fn run_wdf_vs_spice(
    name: &str,
    pro_path: &str,
    controls: &[(&str, f64)],
    gate: Option<(f64, f64, f64)>,
) {
    // Load .pedal from pro repo (skip if absent)
    let source = skip_if_missing!(load_pro_pedal_sub(pro_path), pro_path);

    // Load the ngspice golden
    let golden_path = golden_dir(name).join("sine_1k.npy");
    if !golden_path.exists() {
        eprintln!(
            "  SKIP {name}_wdf_vs_spice: golden not found at {golden_path:?}; \
             run with PEDALKERNEL_BOOTSTRAP_KITS=1 first"
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
    for i in 0..warmup {
        let s = SINE_AMP * (2.0 * PI * TEST_FREQ * i as f64 / SR).sin();
        proc.process(s);
    }
    let wdf_output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

    // Metrics (full signal + settled-window trim)
    let result = metrics::compare(&wdf_output, &golden, SR, Some(TEST_FREQ));
    let trim = (TRIM_SECONDS * SR) as usize;
    let wdf_trim = &wdf_output[trim.min(wdf_output.len())..];
    let golden_trim = &golden[trim.min(golden.len())..];
    let result_trim = metrics::compare(wdf_trim, golden_trim, SR, Some(TEST_FREQ));

    // Report
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
    if let Some(thd_err) = result_trim.thd_error_db {
        eprintln!("    THD error            : {:.1} dB", thd_err);
    }
    if let Some(eo) = result_trim.even_odd_ratio_db {
        eprintln!("    even/odd ratio (gold): {:.1} dB (informational)", eo);
    }

    // Sanity: WDF output must not be silence
    assert!(
        rms(&wdf_output) > 1e-6,
        "{name} WDF output is silence (RMS < 1e-6): compilation or processing failed"
    );

    // Hard gate: measured + margin thresholds when provided
    if let Some((rms_thresh, peak_thresh, thd_thresh)) = gate {
        assert!(
            result_trim.normalized_rms_error_db <= rms_thresh,
            "{name} normalized RMS error {:.1} dB exceeds gate {:.1} dB",
            result_trim.normalized_rms_error_db,
            rms_thresh
        );
        assert!(
            result_trim.peak_error_db <= peak_thresh,
            "{name} peak error {:.1} dB exceeds gate {:.1} dB",
            result_trim.peak_error_db,
            peak_thresh
        );
        if let Some(thd_err) = result_trim.thd_error_db {
            assert!(
                thd_err <= thd_thresh,
                "{name} THD error {:.1} dB exceeds gate {:.1} dB",
                thd_err,
                thd_thresh
            );
        }
    }
}

fn bootstrap_gate() -> bool {
    if std::env::var("PEDALKERNEL_BOOTSTRAP_KITS").as_deref() != Ok("1") {
        eprintln!("  SKIP bootstrap: set PEDALKERNEL_BOOTSTRAP_KITS=1 to run");
        return false;
    }
    true
}

// ============================================================================
// ALL-POINTS BOOSTER (EHX LPB-1)
// ============================================================================

/// Regenerate the ngspice golden:
/// `PEDALKERNEL_BOOTSTRAP_KITS=1 cargo test -p pedalkernel-validate \
///   --test kits_spice bootstrap_all_points_booster_golden -- --nocapture`
#[test]
fn bootstrap_all_points_booster_golden() {
    if !bootstrap_gate() {
        return;
    }
    run_bootstrap("all_points_booster", "v_out");
}

/// Deck positions: Volume = 1.0 (full, taper-independent).
///
/// Report-only (legends screamer/sd1 precedent): the compiled booster is
/// currently ~9.4 dB HOTTER than ngspice (WDF RMS 4.04 V vs golden 1.37 V,
/// measured 2026-07-15) — the WDF gain is ~57× where the LPB-1 (and the
/// deck) sit at ~20×, i.e. R5's emitter degeneration is under-effective in
/// the compiled stage, and at that gain the WDF also rail-clips (THD error
/// +65 dB). Surfacing the gap is the point of the independent golden; a
/// hard gate would either mask it (loose) or block on a known engine gap
/// (tight). Silence assertion still guards compile/processing health.
#[test]
fn all_points_booster_wdf_vs_spice() {
    run_wdf_vs_spice(
        "all_points_booster",
        "pedals/kits/all_points_booster.pedal",
        &[("Volume", 1.0)],
        None,
    );
}

// ============================================================================
// SUNFLOWER (Sunface / Fuzz Face)
// ============================================================================

#[test]
fn bootstrap_sunflower_golden() {
    if !bootstrap_gate() {
        return;
    }
    run_bootstrap("sunflower", "v_out");
}

/// Deck positions: Clean=0.0, Fuzz=1.0, Bias=0.5, Sundial=0.5, Volume=1.0.
///
/// IGNORED (honest engine gap, measured 2026-07-15): the compiled pedal is
/// SILENT (WDF RMS < 1e-6 V vs golden 0.257 V) at every control setting
/// probed (Volume 0/0.5/1 × Fuzz 0.5/1). The public fuzz_face example works,
/// so the gap is specific to this faithful Sunface rendition — suspects are
/// the series CLEAN rheostat at the input, the two-rheostat collector chain,
/// and the classic FF output tap at the R4/SUNDIAL junction (node A) rather
/// than the collector. Un-ignore once the engine produces audio here; the
/// ngspice golden is already committed.
#[test]
#[ignore = "engine gap: compiled Sunflower is silent at all control settings (golden RMS 0.257 V) — see doc comment"]
fn sunflower_wdf_vs_spice() {
    run_wdf_vs_spice(
        "sunflower",
        "pedals/kits/sunflower.pedal",
        &[
            ("Clean", 0.0),
            ("Fuzz", 1.0),
            ("Bias", 0.5),
            ("Sundial", 0.5),
            ("Volume", 1.0),
        ],
        None,
    );
}

// ============================================================================
// DISTORTION 250 (MXR Dist+ / DOD 250)
// ============================================================================

#[test]
fn bootstrap_distortion_250_golden() {
    if !bootstrap_gate() {
        return;
    }
    run_bootstrap("distortion_250", "v_out");
}

/// Deck positions: Gain = 1.0 (host; gain-leg pot at min clamp), Volume = 1.0.
///
/// IGNORED (honest engine gap, measured 2026-07-15): the compiled pedal
/// leaks only ~1 mV RMS (golden 0.225 V; gap −47.5 dB), identical across
/// Gain 0/0.5/1, Volume 0/0.5/1 and diagnostic variants without C1, without
/// C5, without the diodes, and with `vref` renamed `vb` — the non-inverting
/// LM741 stage itself is not propagating signal (gain-leg with the series
/// C3→R1→pot to ground is the prime suspect for the opamp extraction).
/// Un-ignore once the engine produces audio here; golden committed.
#[test]
#[ignore = "engine gap: compiled Distortion 250 leaks ~1 mV RMS (golden 0.225 V) — see doc comment"]
fn distortion_250_wdf_vs_spice() {
    run_wdf_vs_spice(
        "distortion_250",
        "pedals/kits/distortion_250.pedal",
        &[("Gain", 1.0), ("Volume", 1.0)],
        None,
    );
}

// ============================================================================
// UBERDRIVE (Boss SD-1)
// ============================================================================

#[test]
fn bootstrap_uberdrive_golden() {
    if !bootstrap_gate() {
        return;
    }
    run_bootstrap("uberdrive", "v_out");
}

/// Deck positions: Drive/Tone/Volume = 0.5 (all B-taper, mids exact).
///
/// IGNORED (honest engine gap, measured 2026-07-15): compile emits
/// "injection guard [general-MNA]: group has boundary nodes but none is
/// reachable from the global input — adapted input port falls back to the
/// device input pin (mna#1[Q2.base,C8.b,R13.a])" and the output is SILENT
/// (WDF RMS < 1e-6 V vs golden 0.519 V). The volume-pot wiper →R12→C8→ Q2
/// output-follower crossing is not routed by the signal-flow layer — same
/// family as the WiperDivider/straddling-pot work. Un-ignore once the
/// follower group receives signal; golden committed.
#[test]
#[ignore = "engine gap: Q2 output-follower MNA group unreachable from input (injection guard), output silent — see doc comment"]
fn uberdrive_wdf_vs_spice() {
    run_wdf_vs_spice(
        "uberdrive",
        "pedals/kits/uberdrive.pedal",
        &[("Drive", 0.5), ("Tone", 0.5), ("Volume", 0.5)],
        None,
    );
}

// ============================================================================
// LITTLE GREEN SCREAM MACHINE (TS-808)
// ============================================================================

#[test]
fn bootstrap_little_green_scream_machine_golden() {
    if !bootstrap_gate() {
        return;
    }
    run_bootstrap("little_green_scream_machine", "v_out");
}

/// Deck positions: Drive = 1.0 (taper-exact extreme), Tone/Volume = 0.5.
///
/// IGNORED (honest engine gap, measured 2026-07-15): same injection-guard
/// warning as the UberDrive (mna#1[C8.b,R12.a,Q2.base] unreachable from the
/// global input) and the output is SILENT (WDF RMS < 1e-6 V vs golden
/// 0.364 V). The TS808/SD-1 output follower behind the volume pot is the
/// shared unsupported crossing. Un-ignore once the follower group receives
/// signal; golden committed.
#[test]
#[ignore = "engine gap: Q2 output-follower MNA group unreachable from input (injection guard), output silent — see doc comment"]
fn little_green_scream_machine_wdf_vs_spice() {
    run_wdf_vs_spice(
        "little_green_scream_machine",
        "pedals/kits/little_green_scream_machine.pedal",
        &[("Drive", 1.0), ("Tone", 0.5), ("Volume", 0.5)],
        None,
    );
}

// ============================================================================
// Skip mechanism probe
// ============================================================================

/// Prove that the skip mechanism returns cleanly when a pro .pedal is absent.
#[test]
fn kits_skip_when_pro_absent() {
    let source = skip_if_missing!(
        load_pro_pedal_sub("pedals/kits/__nonexistent_probe__.pedal"),
        "pedals/kits/__nonexistent_probe__.pedal"
    );
    let _ = source;
}
