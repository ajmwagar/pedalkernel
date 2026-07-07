//! AC-signal accuracy dashboard — the audio twin of `bias_accuracy.rs`.
//!
//! `bias_accuracy.rs` grades the DC operating point; THIS grades the AC SIGNAL:
//! WDF vs the ngspice-derived golden, decomposed so every AC-fidelity dimension is
//! actually measured. It exists because `normalized_rms_error_db` DEGENERATES for
//! a pure-gain-offset circuit — when the WDF and golden differ only by a scalar
//! level `k`, that difference metric reads |k−1| and can neither see the true
//! level gap nor the shape. The BA283 is exactly such a circuit (a clean Class-A
//! gain block sitting at a fixed +2.42 dB level offset with the DC servo on), so a
//! single-bin gain scalar was all the old `ba283_wdf_vs_spice` test could report.
//!
//! The fix (see `metrics::ac_accuracy`) SEPARATES the dimensions:
//!   1. LEVEL    — `ac_gain_db` at the 1 kHz tone (the scalar offset).
//!   2. SHAPE    — gain-normalize the WDF (level factored OUT), then the RMS
//!                 (phase-sensitive) and spectral (phase-immune) residual.
//!   3. HARMONICS— THD (WDF vs ngspice) + per-harmonic (2nd–5th) dBc error.
//!   4. RESPONSE — `ac_gain_db` across a 50 Hz → 10 kHz sweep → tilt (flat scalar
//!                 vs frequency-shaped).
//!
//! # Goldens
//!
//! ngspice-DERIVED, never WDF-bootstrapped. Each sweep frequency gets a settled
//! measurement-window golden under `golden/ac/<circuit>/f<freq>.npy`, generated
//! from the PUBLIC `.spice` deck (so goldens exist even when the private BA283
//! `.pedal` is absent). Regenerate all:
//!
//! ```text
//! PK_AC_BOOTSTRAP=1 cargo test -p pedalkernel-validate --test ac_accuracy \
//!   ac_accuracy_dashboard -- --nocapture
//! ```
//!
//! # How to add a circuit
//!
//! Append a [`Circuit`] to `CIRCUITS`:
//!   * `name`  — golden dir stem (`golden/ac/<name>/`).
//!   * `deck`  — ngspice deck under `spice-circuits/` (the golden source, public).
//!   * `pedal` — `Pedal::Public("circuits/…")` or `Pedal::Pro("pedals/…")` (the
//!               Pro circuit SKIPS the WDF analysis when the pro repo is absent;
//!               its goldens are still generated from the public deck).
//!   * `gain`  — optional control set before settling.
//!
//! The deck must expose `v_in` / `v_out` and the pedal must compile to a runnable
//! processor. The WDF window is settled for `SETTLE_S` and measured over
//! `MEASURE_S`, phase-aligned to the decimated ngspice golden.

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use pedalkernel_validate::metrics::ac_accuracy::{self, AcThresholds, FreqPoint};
use pedalkernel_validate::pro_pedal::load_pro_pedal_sub;
use pedalkernel_validate::report::{render_ac_accuracy, render_boundary_loads};
use pedalkernel_validate::spice::{SpiceConfig, SpiceRunner};
use pedalkernel_validate::{metrics, npy};
use std::f64::consts::PI;
use std::path::PathBuf;

const SR: f64 = 48_000.0;
const OVERSAMPLE: u32 = 4;
const SINE_AMP: f64 = 0.1;
/// Decomposition tone (level / shape / harmonics are read here).
const TEST_FREQ: f64 = 1_000.0;
/// Coupling caps (10 µF into 10 k ⇒ RC ≈ 100 ms) need ~10 τ to settle.
const SETTLE_S: f64 = 1.0;
/// Measurement window. 0.2 s ⇒ ≥10 cycles even at the 50 Hz sweep endpoint.
const MEASURE_S: f64 = 0.2;
/// Frequency-response sweep (Hz).
const SWEEP: [f64; 6] = [50.0, 100.0, 300.0, 1_000.0, 3_000.0, 10_000.0];

enum Pedal {
    Public(&'static str),
    Pro(&'static str),
}

struct Circuit {
    name: &'static str,
    deck: &'static str,
    pedal: Pedal,
    gain: Option<f64>,
}

/// BA283 first (its real AC error — the known answer), then the public silicon
/// two-stage feedback amp `si_fb_amp` (the discrete-active BA283 proxy, always
/// runs on CI). Both expose `v_in`/`v_out` and compile to a runnable processor.
const CIRCUITS: &[Circuit] = &[
    Circuit {
        name: "neve1073_ba283",
        deck: "active/neve1073_ba283",
        pedal: Pedal::Pro("pedals/500/1073/sub/ba283.pedal"),
        gain: Some(0.5),
    },
    Circuit {
        name: "si_fb_amp",
        deck: "active/si_fb_amp",
        pedal: Pedal::Public("circuits/active/si_fb_amp.pedal"),
        gain: None,
    },
];

fn manifest() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn deck_path(deck: &str) -> PathBuf {
    manifest()
        .join("spice-circuits")
        .join(format!("{deck}.spice"))
}

fn golden_path(name: &str, freq: f64) -> PathBuf {
    manifest()
        .join("golden")
        .join("ac")
        .join(name)
        .join(format!("f{}.npy", freq as u32))
}

/// The internal-rate stimulus covering `settle + measure` for one frequency.
fn internal_input(freq: f64) -> (SpiceRunner, Vec<f64>) {
    let config = SpiceConfig {
        sample_rate: SR as u32,
        oversample: OVERSAMPLE,
    };
    let internal_rate = config.internal_rate();
    let n = ((SETTLE_S + MEASURE_S) * internal_rate) as usize;
    let input: Vec<f64> = (0..n)
        .map(|i| SINE_AMP * (2.0 * PI * freq * i as f64 / internal_rate).sin())
        .collect();
    (SpiceRunner::new(config), input)
}

/// Load the ngspice-derived settled-window golden for one frequency, generating
/// it from the PUBLIC deck if missing (or when `PK_AC_BOOTSTRAP=1`). The golden is
/// NEVER WDF-bootstrapped — it is derived straight from the `.spice` deck.
fn ngspice_window(circuit: &Circuit, freq: f64) -> Option<Vec<f64>> {
    let gp = golden_path(circuit.name, freq);
    let force = std::env::var("PK_AC_BOOTSTRAP")
        .map(|v| v == "1")
        .unwrap_or(false);
    if gp.exists() && !force {
        return npy::read_f64(&gp).ok();
    }
    let dp = deck_path(circuit.deck);
    if !dp.exists() {
        eprintln!("  deck missing: {dp:?}");
        return None;
    }
    let (runner, input) = internal_input(freq);
    match runner.simulate_settled_window(&dp, &input, "v_out", SETTLE_S) {
        Ok(golden) => {
            std::fs::create_dir_all(gp.parent().unwrap()).ok();
            if let Err(e) = npy::write_f64(&gp, &golden) {
                eprintln!("  write golden {gp:?}: {e}");
            } else {
                eprintln!("  wrote golden {gp:?} ({} samples)", golden.len());
            }
            Some(golden)
        }
        Err(e) => {
            eprintln!("  ngspice failed for {} @ {freq} Hz: {e}", circuit.name);
            None
        }
    }
}

/// Load a circuit's `.pedal` source. `None` for an absent Pro pedal (WDF analysis
/// is then skipped for that circuit; its goldens are still generated).
fn load_source(circuit: &Circuit) -> Option<String> {
    match &circuit.pedal {
        Pedal::Public(rel) => {
            let p = manifest().join(rel);
            Some(std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("read {p:?}: {e}")))
        }
        Pedal::Pro(rel) => match load_pro_pedal_sub(rel) {
            Some(s) => Some(s),
            None => {
                eprintln!("  SKIP WDF for {} — pro pedal absent ({rel})", circuit.name);
                None
            }
        },
    }
}

/// Compile a fresh processor, set the gain, warm up `SETTLE_S`, then record a
/// `MEASURE_S` window of the tone at `freq` — same rate/length/phase as the
/// decimated ngspice golden.
fn wdf_window(circuit: &Circuit, source: &str, freq: f64) -> Vec<f64> {
    let def = parse_pedal_file(source).unwrap_or_else(|e| panic!("parse {}: {e}", circuit.name));
    let mut proc =
        compile_pedal(&def, SR).unwrap_or_else(|e| panic!("compile {}: {e}", circuit.name));
    if let Some(g) = circuit.gain {
        proc.set_control("Gain", g);
    }
    let settle_n = (SETTLE_S * SR) as usize;
    let measure_n = (MEASURE_S * SR) as usize;
    for i in 0..settle_n {
        proc.process(SINE_AMP * (2.0 * PI * freq * i as f64 / SR).sin());
    }
    (0..measure_n)
        .map(|j| {
            let i = settle_n + j;
            proc.process(SINE_AMP * (2.0 * PI * freq * i as f64 / SR).sin())
        })
        .collect()
}

/// Compile once (no settling) and read the compile-time boundary-load decision
/// table off `CompiledPedal::boundary_loads` — what hangs downstream of each
/// stage's output boundary and whether the compiler fused it or left it open.
fn boundary_loads_of(
    circuit: &Circuit,
    source: &str,
) -> Vec<pedalkernel_rt::processor::BoundaryLoadBinding> {
    let def = parse_pedal_file(source).unwrap_or_else(|e| panic!("parse {}: {e}", circuit.name));
    let proc = compile_pedal(&def, SR).unwrap_or_else(|e| panic!("compile {}: {e}", circuit.name));
    proc.boundary_loads.clone()
}

/// Evaluate one circuit: returns `None` when its golden or pedal is unavailable.
/// The second tuple element is the circuit's compile-time boundary-load table.
fn evaluate_circuit(
    circuit: &Circuit,
    th: &AcThresholds,
) -> Option<(
    ac_accuracy::AcResult,
    Vec<pedalkernel_rt::processor::BoundaryLoadBinding>,
)> {
    // Generate/load ALL sweep goldens first (from the public deck — this succeeds
    // even for the Pro BA283 with no `.pedal` present, so the goldens get written
    // and committed regardless).
    let mut goldens: Vec<(f64, Vec<f64>)> = Vec::new();
    for &f in &SWEEP {
        match ngspice_window(circuit, f) {
            Some(g) => goldens.push((f, g)),
            None => {
                eprintln!("  {} @ {f} Hz: no golden — cannot grade", circuit.name);
                return None;
            }
        }
    }

    // WDF side (skipped for an absent Pro pedal).
    let source = load_source(circuit)?;

    // Compile-time boundary-load decision table (printed with the dashboard).
    let boundary = boundary_loads_of(circuit, &source);

    // Frequency-response sweep: per-frequency WDF-vs-golden AC gain.
    let mut response = Vec::new();
    let mut main: Option<(Vec<f64>, Vec<f64>)> = None; // (wdf, golden) at TEST_FREQ
    for (f, golden) in &goldens {
        let wdf = wdf_window(circuit, &source, *f);
        let n = wdf.len().min(golden.len());
        let gain_db = metrics::ac_gain_db(&wdf[..n], &golden[..n], *f, SR);
        response.push(FreqPoint {
            freq_hz: *f,
            gain_db,
        });
        if (*f - TEST_FREQ).abs() < 1e-6 {
            main = Some((wdf, golden.clone()));
        }
    }

    let (wdf, golden) = main.expect("TEST_FREQ (1 kHz) must be in the sweep");
    Some((
        ac_accuracy::evaluate(circuit.name, &wdf, &golden, SR, TEST_FREQ, response, th),
        boundary,
    ))
}

#[test]
fn ac_accuracy_dashboard() {
    let th = AcThresholds::default();
    let mut results = Vec::new();
    let mut ba283: Option<ac_accuracy::AcResult> = None;
    let mut boundary: Vec<(String, Vec<pedalkernel_rt::processor::BoundaryLoadBinding>)> =
        Vec::new();

    for circuit in CIRCUITS {
        let Some((res, loads)) = evaluate_circuit(circuit, &th) else {
            eprintln!("  skip {} (golden/pedal unavailable)", circuit.name);
            continue;
        };
        if circuit.name == "neve1073_ba283" {
            ba283 = Some(res.clone());
        }
        results.push(res);
        boundary.push((circuit.name.to_string(), loads));
    }

    if results.is_empty() {
        eprintln!("  no circuits evaluated (ngspice/pro absent) — skipping");
        return;
    }

    let table = render_ac_accuracy(&results, &th);
    println!("{table}");

    // Boundary-load decision table: what hangs off each circuit's output
    // boundary and what the compiler did about it (fused vs silently open).
    println!("{}", render_boundary_loads(&boundary));

    // ── si_fb_amp: the public, always-on CI coverage ─────────────────────────
    // (Discrete two-BJT feedback amp — the public BA283 proxy.) It must produce a
    // decisive, structurally-complete AC result on every machine.
    if let Some(r) = results.iter().find(|r| r.circuit == "si_fb_amp") {
        assert!(r.gain_db.is_finite(), "si_fb_amp AC gain must be finite");
        assert_eq!(r.response.len(), SWEEP.len(), "si_fb_amp full sweep");
        assert_ne!(
            r.verdict,
            ac_accuracy::AcVerdict::NoData,
            "si_fb_amp must reach a decisive AC verdict"
        );
    }

    // ── BA283 decomposition — THE deliverable (the real AC error) ─────────────
    if let Some(r) = ba283 {
        println!("\n──────────────── BA283 AC error, decomposed ────────────────");
        println!(
            "  LEVEL      : {:+.2} dB  (WDF {:.5} V vs ngspice {:.5} V @ {:.0} Hz)",
            r.gain_db, r.wdf_amp, r.golden_amp, r.test_freq_hz
        );
        println!(
            "  SHAPE      : rms {:+.1} dB (gain-normalized, phase-sensitive) | \
             spectral {:.1} dB (phase-immune)",
            r.shape_rms_db, r.shape_spectral_db
        );
        println!(
            "  HARMONICS  : THD wdf {:.1} dB vs ng {:.1} dB (Δ {:.1} dB); max per-harmonic {:.1} dB",
            r.thd_wdf_db,
            r.thd_golden_db,
            r.thd_error_db(),
            r.max_harmonic_error_db()
        );
        print!("               per-harmonic dBc (wdf/ng/Δ):");
        for h in &r.harmonics {
            print!(
                " h{}={:.1}/{:.1}/{:.1}{}",
                h.order,
                h.wdf_dbc,
                h.golden_dbc,
                h.error_db,
                if h.scored { "" } else { "·" }
            );
        }
        println!();
        print!("  RESPONSE   : ");
        for p in &r.response {
            let g = if p.gain_db.is_finite() {
                format!("{:+.2}", p.gain_db)
            } else {
                "n/a".to_string()
            };
            print!("{:.0}Hz={}dB  ", p.freq_hz, g);
        }
        println!("→ tilt {:.2} dB", r.response_tilt_db);
        println!("  VERDICT    : {}", r.verdict.label());

        // The honest, plain-language conclusion.
        let flat = r.response_tilt_db <= th.response_tilt_fail_db;
        let shape_clean = r.shape_spectral_db <= th.shape_spectral_fail_db
            && r.shape_rms_db <= th.shape_rms_fail_db;
        let harm_clean = r.thd_error_db() <= th.thd_error_fail_db
            && r.max_harmonic_error_db() <= th.harmonic_fail_db;
        if matches!(r.verdict, ac_accuracy::AcVerdict::LevelOnly) {
            println!(
                "\n  ⇒ The BA283 AC error is JUST the {:+.2} dB LEVEL offset: the gain-normalized\n     \
                 shape ({:.1} dB spectral), THD (Δ{:.1} dB) and frequency response (tilt {:.2} dB)\n     \
                 all agree with ngspice ⇒ the residual is a FLAT SCALAR (a pure-gain bug), the\n     \
                 waveform SHAPE is clean.",
                r.gain_db, r.shape_spectral_db, r.thd_error_db(), r.response_tilt_db
            );
        } else {
            println!(
                "\n  ⇒ The BA283 AC error is MORE than the {:+.2} dB level: {}{}{}(⇒ additional bug(s) \
                 beyond a flat scalar).",
                r.gain_db,
                if !shape_clean { "SHAPE differs; " } else { "" },
                if !harm_clean { "HARMONICS differ; " } else { "" },
                if !flat { "RESPONSE is frequency-shaped; " } else { "" },
            );
        }
        println!("────────────────────────────────────────────────────────────\n");

        // Structural gates (the metric must have COMPUTED, not a predetermined
        // verdict): a decisive grade, a full sweep, and per-harmonic data.
        assert_ne!(
            r.verdict,
            ac_accuracy::AcVerdict::NoData,
            "BA283 must grade"
        );
        assert_eq!(r.response.len(), SWEEP.len(), "BA283 full sweep computed");
        assert_eq!(r.harmonics.len(), 4, "BA283 2nd–5th harmonics computed");
        assert!(r.gain_db.is_finite(), "BA283 level must be a real number");
    } else {
        eprintln!("  (BA283 pro pedal absent — decomposition printed only for public circuits)");
    }
}
