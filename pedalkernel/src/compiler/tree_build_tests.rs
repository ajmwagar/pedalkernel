// ═══════════════════════════════════════════════════════════════════════════
// WDF Tree Construction Tests
//
// Tests for SPQR-based WDF tree construction from circuit edges.
// Pendant trees were removed — input coupling is now handled by separate
// SPQR passive stages. These tests verify the remaining tree builders.
// ═══════════════════════════════════════════════════════════════════════════

use super::spqr_build::compile_via_spqr;
use crate::PedalProcessor;

const SR: f64 = 48000.0;

// ═══════════════════════════════════════════════════════════════════════════
// 1. Adaptor gain from Zf/Zi — single stage (R_in in same group)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn adaptor_gain_single_stage_correct() {
    // R_in(10k) → U1.neg → Rf(100k) → U1.out → D1. Single group.
    // Gain should be ~Rf/Ri = 10 at small signal (below diode clip).
    let gain = measure_small_signal_gain(r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D1: diode(silicon)
            }
            nets {
                in -> R_in.a  R_in.b -> U1.neg
                U1.neg -> Rf.a  Rf.b -> U1.out
                U1.neg -> D1.a  D1.b -> U1.out
                U1.pos -> gnd  U1.out -> out
            }
            controls {}
        }"#);

    eprintln!("Single stage gain: {gain:.1} (expected ~10)");
    assert!(gain > 5.0 && gain < 20.0, "Gain should be ~10: {gain:.1}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Adaptor gain with coupling cap (R_in in separate group)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn adaptor_gain_with_coupling_cap() {
    // Cin(47n) → R_in(10k) → U1.neg → Rf(100k) → U1.out → D1.
    // Cin+R_in in separate group. Gain should still be ~10.
    let gain = measure_small_signal_gain(r#"
        pedal "test" { supply 9V
            components {
                Cin: cap(47n)
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D1: diode(silicon)
            }
            nets {
                in -> Cin.a  Cin.b -> R_in.a  R_in.b -> U1.neg
                U1.neg -> Rf.a  Rf.b -> U1.out
                U1.neg -> D1.a  D1.b -> U1.out
                U1.pos -> gnd  U1.out -> out
            }
            controls {}
        }"#);

    eprintln!("Coupled gain: {gain:.1} (expected ~10)");
    assert!(gain > 5.0, "Should amplify even with coupling cap: {gain:.1}");
}

#[test]
fn adaptor_gain_consistent_with_and_without_cap() {
    let gain_no_cap = measure_small_signal_gain(r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D1: diode(silicon)
            }
            nets {
                in -> R_in.a  R_in.b -> U1.neg
                U1.neg -> Rf.a  Rf.b -> U1.out
                U1.neg -> D1.a  D1.b -> U1.out
                U1.pos -> gnd  U1.out -> out
            }
            controls {}
        }"#);

    let gain_with_cap = measure_small_signal_gain(r#"
        pedal "test" { supply 9V
            components {
                Cin: cap(47n)
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D1: diode(silicon)
            }
            nets {
                in -> Cin.a  Cin.b -> R_in.a  R_in.b -> U1.neg
                U1.neg -> Rf.a  Rf.b -> U1.out
                U1.neg -> D1.a  D1.b -> U1.out
                U1.pos -> gnd  U1.out -> out
            }
            controls {}
        }"#);

    eprintln!("No cap: {gain_no_cap:.1}, With cap: {gain_with_cap:.1}");
    assert!(gain_no_cap > 5.0, "No cap should amplify: {gain_no_cap:.1}");
    assert!(gain_with_cap > 3.0, "With cap should amplify: {gain_with_cap:.1}");
    let ratio = gain_no_cap.max(gain_with_cap) / gain_no_cap.min(gain_with_cap).max(0.1);
    assert!(ratio < 5.0, "Should be within 5x: {gain_no_cap:.1} vs {gain_with_cap:.1}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Feedback with pot + series R (screamer pattern)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn adaptor_gain_with_drive_pot() {
    // Drive(500k) + R_clip(4.7k) in feedback. At 50%, Rf ≈ 255k.
    // Gain ≈ 255k / 10k ≈ 25. Diode clips, so actual < 25.
    let gain = measure_small_signal_gain(r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Drive: pot(500k, a)
                R_clip: resistor(4.7k)
                D1: diode(silicon)
            }
            nets {
                in -> R_in.a  R_in.b -> U1.neg
                U1.neg -> Drive.a  Drive.b -> R_clip.a  R_clip.b -> U1.out
                U1.neg -> D1.a  D1.b -> U1.out
                U1.pos -> gnd  U1.out -> out
            }
            controls { Drive.position -> "Drive" [0.0, 1.0] = 0.5 }
        }"#);

    eprintln!("Drive pot gain: {gain:.1}");
    assert!(gain > 3.0, "Should amplify with pot: {gain:.1}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Legend pedal gains through adaptor
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn adaptor_screamer_gain_reasonable() {
    let pedal = load_legend("screamer");
    let gain = legend_small_signal_gain(&pedal);
    eprintln!("Screamer small-signal gain: {gain:.1}");
    assert!(gain > 1.0, "Screamer should amplify: {gain:.1}");
}

#[test]
fn adaptor_sd1_gain_reasonable() {
    let pedal = load_legend("sd1");
    let gain = legend_small_signal_gain(&pedal);
    eprintln!("SD-1 small-signal gain: {gain:.1}");
    assert!(gain > 1.0, "SD-1 should amplify: {gain:.1}");
}

// ═══════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════

fn measure_small_signal_gain(src: &str) -> f64 {
    let pedal = crate::dsl::parse_pedal_file(src).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");
    let amp = 0.001; // 1mV — well below diode clip
    for s in 0..1000 {
        compiled.process(amp * (std::f64::consts::TAU * 440.0 * s as f64 / SR).sin());
    }
    let mut peak = 0.0f64;
    for s in 1000..1110 {
        peak = peak.max(compiled.process(amp * (std::f64::consts::TAU * 440.0 * (1000 + s) as f64 / SR).sin()).abs());
    }
    peak / amp
}

fn load_legend(name: &str) -> crate::dsl::PedalDef {
    let path = format!("{}/../../pedalkernel-pro/pedals/legends/{name}.pedal", env!("CARGO_MANIFEST_DIR"));
    let src = std::fs::read_to_string(&path).unwrap_or_else(|_| panic!("Can't read {path}"));
    crate::dsl::parse_pedal_file(&src).unwrap_or_else(|e| panic!("{name}: {e}"))
}

fn legend_small_signal_gain(pedal: &crate::dsl::PedalDef) -> f64 {
    let mut compiled = compile_via_spqr(pedal, SR).expect("compile");
    let amp = 0.001;
    for s in 0..2000 {
        compiled.process(amp * (std::f64::consts::TAU * 440.0 * s as f64 / SR).sin());
    }
    let mut peak = 0.0f64;
    for s in 2000..2110 {
        peak = peak.max(compiled.process(amp * (std::f64::consts::TAU * 440.0 * (2000 + s) as f64 / SR).sin()).abs());
    }
    peak / amp
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. End-to-end: adaptor gain from zi/zf trees
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn adaptor_gain_from_zi_zf_port_resistance() {
    // If zi has rp=10k and zf has rp=100k, the adaptor should produce
    // gain ≈ Zf/Zi = 10. Test this through the full pipeline.
    use super::spqr_build::compile_via_spqr;
    use crate::PedalProcessor;

    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D1: diode(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.neg -> D1.a
                D1.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    // Small signal: 1mV at 440Hz (well below diode clip)
    for s in 0..1000 {
        let input = 0.001 * (std::f64::consts::TAU * 440.0 * s as f64 / 48000.0).sin();
        compiled.process(input);
    }
    let mut peak = 0.0f64;
    for s in 1000..1110 {
        let input = 0.001 * (std::f64::consts::TAU * 440.0 * s as f64 / 48000.0).sin();
        peak = peak.max(compiled.process(input).abs());
    }

    let gain = peak / 0.001;
    eprintln!("Adaptor Zf/Zi gain: {gain:.1} (expected ~10)");
    // With R_in=10k and Rf=100k, gain should be ~10
    assert!(gain > 5.0, "Adaptor gain should be >5: {gain:.1}");
    assert!(gain < 20.0, "Adaptor gain should be <20: {gain:.1}");
}

#[test]
fn adaptor_gain_with_coupling_cap_matches_without() {
    // Adding a coupling cap shouldn't drastically change gain.
    // Without cap: gain ≈ 10 (Rf/Ri).
    // With cap: gain ≈ 10 at 440Hz (cap impedance << Ri at audio freq).
    use super::spqr_build::compile_via_spqr;
    use crate::PedalProcessor;

    let measure = |src: &str| -> f64 {
        let pedal = crate::dsl::parse_pedal_file(src).expect("parse");
        let mut c = compile_via_spqr(&pedal, 48000.0).expect("compile");
        for s in 0..1000 {
            let input = 0.001 * (std::f64::consts::TAU * 440.0 * s as f64 / 48000.0).sin();
            c.process(input);
        }
        let mut peak = 0.0f64;
        for s in 1000..1110 {
            let input = 0.001 * (std::f64::consts::TAU * 440.0 * s as f64 / 48000.0).sin();
            peak = peak.max(c.process(input).abs());
        }
        peak / 0.001
    };

    let gain_no_cap = measure(r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D1: diode(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a  Rf.b -> U1.out
                U1.neg -> D1.a  D1.b -> U1.out
                U1.pos -> gnd   U1.out -> out
            }
            controls {}
        }"#);

    let gain_with_cap = measure(r#"
        pedal "test" { supply 9V
            components {
                Cin: cap(47n)
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D1: diode(silicon)
            }
            nets {
                in -> Cin.a
                Cin.b -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a  Rf.b -> U1.out
                U1.neg -> D1.a  D1.b -> U1.out
                U1.pos -> gnd   U1.out -> out
            }
            controls {}
        }"#);

    eprintln!("No cap: {gain_no_cap:.1}, With cap: {gain_with_cap:.1}");
    assert!(gain_no_cap > 5.0, "No cap should amplify: {gain_no_cap:.1}");
    assert!(gain_with_cap > 3.0, "With cap should amplify: {gain_with_cap:.1}");

    let ratio = gain_no_cap.max(gain_with_cap) / gain_no_cap.min(gain_with_cap).max(0.1);
    assert!(ratio < 5.0, "Gains should be within 5x: no_cap={gain_no_cap:.1}, with_cap={gain_with_cap:.1}");
}
