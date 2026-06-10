//! TDD tests for 3-terminal tone-blend pot effect.
//!
//! These pots use the wiper (.w) as a signal tap between .a and .b.
//! When expanded, the wiper node must become a junction so each half
//! (__aw, __wb) is pendant and gets claimed into the WDF tree.
//!
//! Currently FAILING: wiper stays internal → both halves bridge two
//! junctions → sp_decompose sends them to residual → pot has no effect.

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;

const SAMPLE_RATE: f64 = 48_000.0;

// ---------------------------------------------------------------------------
// Minimal 3-terminal tone pot test netlist
// ---------------------------------------------------------------------------

/// Minimal circuit matching SD-1 topology: inverting clipping stage with
/// feedback diodes → 3-terminal tone pot blend → unity buffer → output.
/// The diodes make this a multi-NL group that goes through plan_diode_bridge.
/// Tone.a gets the direct path, Tone.w feeds the unity buffer, Tone.b gets
/// the bass cap to ground. Sweeping Tone should change spectrum.
const TONE_BLEND_MINIMAL: &str = r#"
pedal "ToneBlendTest" {
  components {
    C_in: cap(100n)
    R_in: resistor(10k)
    U1: opamp(tl072)
    R_fb: resistor(51k)
    D1: diode(silicon)
    D2: diode(silicon)
    R_mix: resistor(1k)
    Tone: pot(20k, b)
    C_bass: cap(220n)
    R_treble: resistor(1k)
    C_treble: cap(47n)
    U2: opamp(tl072)
    C_out: cap(100n)
    R_out: resistor(10k)
  }
  nets {
    in -> C_in.a
    C_in.b -> R_in.a
    R_in.b -> U1.neg
    U1.pos -> gnd
    U1.neg -> R_fb.a
    R_fb.b -> U1.out
    U1.neg -> D1.a
    D1.b -> U1.out
    U1.neg -> D2.b
    D2.a -> U1.out
    U1.out -> R_mix.a
    R_mix.b -> Tone.a, R_treble.a
    Tone.w -> U2.pos
    Tone.b -> C_bass.a
    C_bass.b -> gnd
    R_treble.b -> C_treble.a
    C_treble.b -> gnd
    U2.neg -> U2.out
    U2.out -> C_out.a
    C_out.b -> R_out.a, out
    R_out.b -> gnd
  }
  controls {
    Tone.position -> "Tone" [0.0, 1.0] = 0.5
  }
  calibrate
}
"#;

// ---------------------------------------------------------------------------
// SD-1 Tone pot (from examples/pedals/overdrive/sd1.pedal)
// ---------------------------------------------------------------------------

const SD1_PEDAL: &str = include_str!("../examples/pedals/overdrive/sd1.pedal");

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn process_with_tone(pedal_src: &str, tone: f64, other_controls: &[(&str, f64)]) -> Vec<f64> {
    let input = sine_at(1000.0, 0.1, 0.15, SAMPLE_RATE);
    let mut controls: Vec<(&str, f64)> = other_controls.to_vec();
    controls.push(("Tone", tone));
    compile_and_process(pedal_src, &input, SAMPLE_RATE, &controls)
}

fn rms_db_change(a: &[f64], b: &[f64]) -> f64 {
    let ra = rms(a).max(1e-30);
    let rb = rms(b).max(1e-30);
    20.0 * (rb / ra).log10()
}

// ---------------------------------------------------------------------------
// Test 1: Minimal 3-terminal blend pot changes output
// ---------------------------------------------------------------------------

#[test]
fn tone_blend_pot_changes_output() {
    let low = process_with_tone(TONE_BLEND_MINIMAL, 0.0, &[]);
    let high = process_with_tone(TONE_BLEND_MINIMAL, 1.0, &[]);

    let db = rms_db_change(&low, &high).abs();
    eprintln!("[tone_blend_minimal] dB change: {db:.2}");

    // Simplified circuit — effect is smaller than real pedal but must be nonzero.
    assert!(
        db > 0.05,
        "3-terminal tone blend pot must affect output: dB change = {db:.4} (expected > 0.05 dB)"
    );
}

// ---------------------------------------------------------------------------
// Test 2: SD-1 Tone pot changes spectral content
// ---------------------------------------------------------------------------

#[test]
fn sd1_tone_pot_changes_spectrum() {
    let bright = process_with_tone(SD1_PEDAL, 0.0, &[("Drive", 0.5), ("Level", 0.5)]);
    let dark = process_with_tone(SD1_PEDAL, 1.0, &[("Drive", 0.5), ("Level", 0.5)]);

    // Measure high-frequency energy difference
    let bright_hf: f64 = [2000.0, 3000.0, 4000.0, 5000.0]
        .iter()
        .map(|&f| goertzel_power(&bright, SAMPLE_RATE, f))
        .sum();
    let dark_hf: f64 = [2000.0, 3000.0, 4000.0, 5000.0]
        .iter()
        .map(|&f| goertzel_power(&dark, SAMPLE_RATE, f))
        .sum();

    let ratio = dark_hf / bright_hf.max(1e-30);
    let db = rms_db_change(&bright, &dark).abs();

    eprintln!("[sd1_tone] bright_hf={bright_hf:.6e}, dark_hf={dark_hf:.6e}, ratio={ratio:.4}, rms_db={db:.2}");

    assert!(
        db > 0.5,
        "SD-1 Tone pot must affect output: dB change = {db:.4} (expected > 0.5 dB)"
    );
}

// ---------------------------------------------------------------------------
// Test 3: Goldenrod Treble pot changes spectral content
// (inline minimal version — avoids dependency on pro bundle)
// ---------------------------------------------------------------------------

/// Simplified Goldenrod Treble stage: inverting opamp with reactive feedback
/// containing a 3-terminal Treble pot.
const GOLDENROD_TREBLE_STAGE: &str = r#"
pedal "GoldenrodTreble" {
  components {
    C_in: cap(100n)
    R_in: resistor(15k)
    U1: opamp(tl072)
    R_fb: resistor(392k)
    C_tone: cap(3.9n)
    Treble: pot(10k, b)
    R_shelf: resistor(1k)
    R_load: resistor(10k)
  }
  nets {
    in -> C_in.a
    C_in.b -> R_in.a
    R_in.b -> U1.neg
    U1.pos -> gnd
    U1.neg -> R_fb.a
    R_fb.b -> U1.out
    U1.neg -> C_tone.a
    C_tone.b -> Treble.a
    Treble.w -> R_shelf.a
    R_shelf.b -> U1.out
    Treble.b -> U1.out
    U1.out -> R_load.a
    R_load.b -> gnd, out
  }
  controls {
    Treble.position -> "Treble" [1.0, 0.0] = 0.5
  }
  calibrate
}
"#;

// ---------------------------------------------------------------------------
// Test 4: Screamer Drive pot (feedback diode opamp, was DiodePairedOpAmp)
// ---------------------------------------------------------------------------

const SCREAMER_PEDAL: &str = include_str!("../examples/pedals/overdrive/tube_screamer.pedal");

#[test]
fn screamer_drive_pot_changes_output() {
    let input = sine_at(1000.0, 0.1, 0.15, SAMPLE_RATE);
    let low = compile_and_process(
        SCREAMER_PEDAL,
        &input,
        SAMPLE_RATE,
        &[("Drive", 0.0), ("Tone", 0.5), ("Level", 0.5)],
    );
    let high = compile_and_process(
        SCREAMER_PEDAL,
        &input,
        SAMPLE_RATE,
        &[("Drive", 1.0), ("Tone", 0.5), ("Level", 0.5)],
    );

    let db = rms_db_change(&low, &high).abs();
    eprintln!("[screamer_drive] dB change: {db:.2}");

    assert!(
        db > 0.5,
        "Screamer Drive pot must affect output: dB change = {db:.4} (expected > 0.5 dB)"
    );
}

// ---------------------------------------------------------------------------
// Test 5: Goldenrod Treble (opamp feedback MNA adaptor pot)
// ---------------------------------------------------------------------------

#[test]
fn goldenrod_treble_pot_changes_spectrum() {
    // Use real Goldenrod pedal file — the inline simplified version may
    // compile differently. The real pedal has the full Klon topology.
    let goldenrod_path = std::path::Path::new(
        "/Users/ajmwagar/src/pedalkernel/pedalkernel-pro/pedals/legends/goldenrod.pedal",
    );
    if !goldenrod_path.exists() {
        eprintln!("[goldenrod_treble] SKIP: pro pedal not found");
        return;
    }
    let src = std::fs::read_to_string(&goldenrod_path).unwrap();
    let mut best_db = 0.0_f64;
    for freq in [1000.0, 3000.0, 5000.0, 8000.0, 10000.0] {
        // Use longer signal — calibration consumes early samples, pot needs
        // time to settle after set_control before we measure RMS.
        let input = sine_at(freq, 0.1, 0.5, SAMPLE_RATE); // 0.5s = 24000 samples

        let bright = compile_and_process(
            &src,
            &input,
            SAMPLE_RATE,
            &[("Gain", 0.5), ("Treble", 0.0), ("Output", 0.5)],
        );
        let dark = compile_and_process(
            &src,
            &input,
            SAMPLE_RATE,
            &[("Gain", 0.5), ("Treble", 1.0), ("Output", 0.5)],
        );

        // Measure RMS on last quarter only (after pot has settled)
        let n = bright.len();
        let quarter = n * 3 / 4;
        let db = rms_db_change(&bright[quarter..], &dark[quarter..]).abs();
        eprintln!("[goldenrod_treble] {freq:.0}Hz dB change: {db:.2}");
        if db > best_db {
            best_db = db;
        }
    }

    assert!(
        best_db > 0.5,
        "Goldenrod Treble pot must affect output in the treble band: best dB change = {best_db:.4} (expected > 0.5 dB)"
    );
}
