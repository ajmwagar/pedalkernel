//! Regression tests for RATKING (ProCo RAT) pedal behaviour.
//!
//! Verifies two fixes:
//!  1. `graph.rs` reactive feedback path includes fixed_series_r (R5=560Ω)
//!     so minimum-distortion gain is correctly ~0.56x instead of effectively 0.
//!  2. `wdf_leaf.rs` set_control() applies the `.max(1.0)` clamp that build.rs
//!     uses at initialisation, preventing conductance blow-up at pot position=0.

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use std::path::Path;

const SAMPLE_RATE: f64 = 48_000.0;

/// Inline RATKING topology — mirrors ratking.pedal exactly but embedded so
/// the test does not depend on the external pro bundle path at CI time.
const RATKING_SRC: &str = r#"
pedal "RATKING" {
  components {
    C1: cap(22n)
    R1: resistor(1M)
    R2: resistor(1k)
    U1: opamp(lm308)
    C_hpf: cap(100p)
    R5: resistor(560)
    Distortion: pot(100k, a)
    D1: diode(silicon)
    D2: diode(silicon)
    C_tone: cap(3.3n)
    Filter: pot(100k, a)
    R_tone: resistor(1.5k)
    C_out: cap(4.7u)
    Volume: pot(100k, a)
  }
  nets {
    in -> C1.a
    C1.b -> R1.a, R2.a
    R1.b -> gnd
    R2.b -> U1.neg
    U1.pos -> gnd
    U1.neg -> R5.a
    R5.b -> Distortion.a
    Distortion.b -> U1.out
    U1.neg -> C_hpf.a
    C_hpf.b -> U1.out
    U1.out -> D1.a, D2.b
    D1.b -> gnd
    D2.a -> gnd
    U1.out -> C_tone.a
    C_tone.b -> gnd
    U1.out -> Filter.a
    Filter.b -> R_tone.a
    R_tone.b -> C_out.a
    C_out.b -> Volume.a
    Volume.w -> out
    Volume.b -> gnd
  }
  controls {
    Distortion.position -> "Distortion" [0.0, 1.0] = 0.5
    Filter.position     -> "Filter"     [0.0, 1.0] = 0.7
    Volume.position     -> "Volume"     [1.0, 0.0] = 0.5
  }
  calibrate
}
"#;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Process RATKING with the given control settings; returns output samples.
fn process_ratking(controls: &[(&str, f64)]) -> Vec<f64> {
    let input = sine_at(440.0, 0.1, 0.2, SAMPLE_RATE);
    compile_and_process(RATKING_SRC, &input, SAMPLE_RATE, controls)
}

// ---------------------------------------------------------------------------
// Test 1: Compilation and basic output health
// ---------------------------------------------------------------------------

/// Verify the pedal compiles and produces non-silent, non-clipping output.
#[test]
fn ratking_compiles_and_produces_output() {
    let output = process_ratking(&[("Distortion", 0.5), ("Filter", 0.5), ("Volume", 0.5)]);
    assert_healthy(&output, "RATKING mid-settings", 50.0);
}

// ---------------------------------------------------------------------------
// Test 2: Minimum distortion is near unity / clean (not silent, not clipped)
//
// RATKING gain formula: (R5 + Distortion_r) / R2
//   Min: (560 + 0) / 1000 ≈ 0.56x  → output peak ≈ 0.56 * input_amp
//
// Bug 1 regression: without the fixed_series_r fix the compiler set rf=0 at
// min distortion, collapsing gain to near-zero and producing an essentially
// silent output.  After the fix, peak should be a significant fraction of the
// 0.1 input amplitude.
// ---------------------------------------------------------------------------

#[test]
fn ratking_min_distortion_not_silent() {
    // Distortion=0 means pot wiper at minimum → pot segment = 0Ω.
    // Only R5=560Ω remains in the feedback path.
    // At max Volume (position=0 with reversed taper), output should carry
    // a clear fraction of the input level.
    let output = process_ratking(&[
        ("Distortion", 0.0),
        ("Filter", 0.0), // bright — keeps highs
        ("Volume", 0.0), // max volume (control range is [1.0, 0.0])
    ]);

    let output_peak = peak(&output);
    let input_amp = 0.1;

    // Allow settling transient and diode loading to push us below the
    // theoretical 0.56x, but output must be at least 5% of input peak.
    // Before the fix this was < 1e-4 (silent).
    assert!(
        output_peak > input_amp * 0.05,
        "RATKING min Distortion should not be silent: output_peak={output_peak:.6}, \
         input_amp={input_amp:.4}. Likely fixed_series_r=0 regression."
    );
}

// ---------------------------------------------------------------------------
// Test 3: Distortion sweep produces monotonically increasing distortion
//
// As Distortion increases, gain rises and clipping (THD) should increase.
// We allow a few non-monotone steps (measurement noise) but the overall
// trend must be strictly upward.
// ---------------------------------------------------------------------------

#[test]
fn ratking_distortion_sweep_increases_thd() {
    let steps: Vec<f64> = (0..=5).map(|i| i as f64 * 0.2).collect();
    let mut thd_values: Vec<f64> = Vec::new();

    for &d in &steps {
        let output = process_ratking(&[
            ("Distortion", d),
            ("Filter", 0.0), // bright to keep harmonics measurable
            ("Volume", 0.5),
        ]);
        let t = thd(&output, SAMPLE_RATE, 440.0);
        eprintln!("[ratking sweep] Distortion={d:.2} THD={t:.4}");
        thd_values.push(t);
    }

    // Overall: last THD must be substantially higher than first THD.
    // At max distortion (massive gain + diode clipping) THD is very high.
    // At min distortion it should be low (clean amp at 0.56x gain).
    let first = thd_values.first().copied().unwrap_or(0.0);
    let last = thd_values.last().copied().unwrap_or(0.0);
    assert!(
        last > first * 2.0,
        "RATKING Distortion sweep: max-distortion THD should be notably higher than \
         min-distortion THD. first={first:.4}, last={last:.4}"
    );
}

// ---------------------------------------------------------------------------
// Test 4: Filter pot reduces high-frequency content
//
// The RAT Filter is a passive lowpass. As Filter increases (more resistance),
// the cutoff frequency drops, reducing high-frequency harmonic energy in the
// output.  At high distortion there are many harmonics, so the spectral
// difference is pronounced.
//
// Method: compare harmonic energy above 2kHz at Filter=0 vs Filter=1.
// Filter=1 (max resistance ≈ 100k) → fc ≈ 475 Hz → most harmonics roll off.
// Filter=0 (min resistance ≈ 1.5k)  → fc ≈ 32 kHz → harmonics pass through.
// ---------------------------------------------------------------------------

#[test]
fn ratking_filter_pot_reduces_high_freq() {
    // Use high distortion so there are plenty of harmonics to filter.
    let bright = process_ratking(&[
        ("Distortion", 0.7),
        ("Filter", 0.0), // bright
        ("Volume", 0.5),
    ]);
    let dark = process_ratking(&[
        ("Distortion", 0.7),
        ("Filter", 1.0), // dark / maximum filtering
        ("Volume", 0.5),
    ]);

    // Sum harmonic energy at 880, 1320, 1760, 2200, 2640, 3080, 3520 Hz.
    let harmonics: &[f64] = &[880.0, 1320.0, 1760.0, 2200.0, 2640.0, 3080.0, 3520.0];
    let bright_hi: f64 = harmonics
        .iter()
        .map(|&f| goertzel_power(&bright, SAMPLE_RATE, f))
        .sum();
    let dark_hi: f64 = harmonics
        .iter()
        .map(|&f| goertzel_power(&dark, SAMPLE_RATE, f))
        .sum();

    eprintln!(
        "[ratking filter] bright_hi={bright_hi:.6e}, dark_hi={dark_hi:.6e}, \
         ratio={:.4}",
        dark_hi / bright_hi.max(1e-30)
    );

    // Dark output should have meaningfully less high-frequency harmonic energy.
    // We require it to be at most 80% of the bright level (>20% reduction).
    assert!(
        dark_hi < bright_hi * 0.80,
        "RATKING Filter=1 (dark) should have less harmonic content than Filter=0 \
         (bright). bright_hi={bright_hi:.6e}, dark_hi={dark_hi:.6e}. \
         Filter pot may not be taking effect."
    );
}

// ---------------------------------------------------------------------------
// Test 5 (optional): External pedal file — only runs if the pro bundle exists.
//
// Same checks but against the real ratking.pedal file.  Skipped in CI if the
// pro bundle is not present (RATKING_PEDAL_PATH not set or file absent).
// ---------------------------------------------------------------------------

#[test]
fn ratking_external_pedal_file_if_present() {
    let pedal_path = format!(
        "{}/src/pedalkernel/pedalkernel-pro/pedals/legends/ratking.pedal",
        env!("HOME")
    );
    let path = Path::new(&pedal_path);
    if !path.exists() {
        eprintln!("[ratking_external] pedal file not found at {pedal_path}, skipping.");
        return;
    }

    let src = std::fs::read_to_string(path).expect("failed to read ratking.pedal");
    let pedal = parse_pedal_file(&src).expect("parse ratking.pedal");
    let mut proc = compile_pedal(&pedal, SAMPLE_RATE).expect("compile ratking.pedal");

    let input = sine_at(440.0, 0.1, 0.2, SAMPLE_RATE);

    // Min distortion should not be silent.
    proc.set_control("Distortion", 0.0);
    proc.set_control("Filter", 0.0);
    proc.set_control("Volume", 0.0);
    let out_min: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    let out_min_peak = peak(&out_min);
    assert!(
        out_min_peak > 0.1 * 0.05,
        "External RATKING min Distortion silent: peak={out_min_peak:.6}"
    );

    // Max distortion should be louder/more distorted.
    proc.set_control("Distortion", 1.0);
    let out_max: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    let thd_min = thd(&out_min, SAMPLE_RATE, 440.0);
    let thd_max = thd(&out_max, SAMPLE_RATE, 440.0);
    eprintln!(
        "[ratking_external] thd_min={thd_min:.4}, thd_max={thd_max:.4}, \
         peak_min={out_min_peak:.6}, peak_max={:.6}",
        peak(&out_max)
    );
    assert!(
        thd_max > thd_min,
        "External RATKING: max distortion THD should exceed min distortion THD"
    );
}
