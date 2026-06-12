// ═══════════════════════════════════════════════════════════════════════════
// Voltage extraction & DSP accuracy tests
//
// These tests validate that the compiler + runtime produce CORRECT output
// voltages for known circuit transfer functions. Not just "produces audio"
// but "produces the RIGHT audio."
//
// Each test defines a circuit with a known theoretical gain/attenuation,
// feeds a 440Hz sine, and checks the output amplitude against the expected
// transfer function.
//
// Organized by stage type and complexity:
// 1. Passive networks (resistive dividers, RC filters)
// 2. BlackFeedback stages (op-amp gain)
// 3. WDF stages (op-amp + NL feedback)
// 4. IIR stages (linear reactive feedback)
// 5. Pot dividers (voltage division)
// 6. Multi-stage cascades
// ═══════════════════════════════════════════════════════════════════════════

use super::spqr_build::compile_via_spqr;
use crate::PedalProcessor;

const SR: f64 = 48000.0;
const FREQ: f64 = 440.0;
const AMPLITUDE: f64 = 0.01;

/// Measure steady-state peak output for a 440Hz sine at given amplitude.
/// Runs 1000 warmup samples, then 2 full cycles.
fn measure_gain(pedal_src: &str) -> f64 {
    let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Warmup
    for s in 0..1000 {
        let input = AMPLITUDE * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin();
        compiled.process(input);
    }

    // Measure
    let n_samples = (2.0 * SR / FREQ).ceil() as usize;
    let mut peak = 0.0f64;
    for s in 0..n_samples {
        let input = AMPLITUDE * (std::f64::consts::TAU * FREQ * (1000 + s) as f64 / SR).sin();
        peak = peak.max(compiled.process(input).abs());
    }

    peak / AMPLITUDE
}

/// Measure gain with a specific control setting.
fn measure_gain_with_control(pedal_src: &str, control: &str, value: f64) -> f64 {
    let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");
    compiled.set_control_immediate(control, value);

    for s in 0..1000 {
        let input = AMPLITUDE * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin();
        compiled.process(input);
    }

    let n_samples = (2.0 * SR / FREQ).ceil() as usize;
    let mut peak = 0.0f64;
    for s in 0..n_samples {
        let input = AMPLITUDE * (std::f64::consts::TAU * FREQ * (1000 + s) as f64 / SR).sin();
        peak = peak.max(compiled.process(input).abs());
    }

    peak / AMPLITUDE
}

/// Measure spectral ratio (5kHz peak / 200Hz peak) for frequency response tests.
fn measure_spectral_ratio(pedal_src: &str) -> f64 {
    let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse");

    let measure_freq = |freq: f64| -> f64 {
        let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");
        for s in 0..1000 {
            let input = AMPLITUDE * (std::f64::consts::TAU * freq * s as f64 / SR).sin();
            compiled.process(input);
        }
        let n = (2.0 * SR / freq).ceil() as usize;
        let mut peak = 0.0f64;
        for s in 0..n {
            let input = AMPLITUDE * (std::f64::consts::TAU * freq * (1000 + s) as f64 / SR).sin();
            peak = peak.max(compiled.process(input).abs());
        }
        peak
    };

    let lo = measure_freq(200.0);
    let hi = measure_freq(5000.0);
    if lo > 1e-10 {
        hi / lo
    } else {
        0.0
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 1. Passive networks — known transfer functions
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn passive_wire_unity_gain() {
    // Direct wire: in → out. Gain should be exactly 1.0.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(1k) }
            nets { in -> R1.a  R1.b -> out }
            controls {}
        }"#,
    );
    eprintln!("Wire: gain={gain:.4}");
    assert!(
        gain > 0.8 && gain < 1.2,
        "Wire should be unity gain: {gain:.4}"
    );
}

#[test]
fn passive_resistive_divider_half() {
    // Voltage divider: R1=10k, R2=10k. Gain = R2/(R1+R2) = 0.5.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                R2: resistor(10k)
            }
            nets {
                in -> R1.a
                R1.b -> R2.a
                R2.b -> gnd
                R1.b -> out
            }
            controls {}
        }"#,
    );
    eprintln!("Divider 50%: gain={gain:.4}");
    assert!(
        gain > 0.35 && gain < 0.65,
        "50% divider should give ~0.5: {gain:.4}"
    );
}

#[test]
fn passive_resistive_divider_tenth() {
    // Voltage divider: R1=9k, R2=1k. Gain = 1k/(9k+1k) = 0.1.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(9k)
                R2: resistor(1k)
            }
            nets {
                in -> R1.a
                R1.b -> R2.a
                R2.b -> gnd
                R1.b -> out
            }
            controls {}
        }"#,
    );
    eprintln!("Divider 10%: gain={gain:.4}");
    assert!(
        gain > 0.05 && gain < 0.2,
        "10% divider should give ~0.1: {gain:.4}"
    );
}

#[test]
fn passive_rc_lowpass_attenuates_high_freq() {
    // RC lowpass: R=10k, C=10nF. fc = 1/(2π·R·C) ≈ 1592Hz.
    // 200Hz should pass (~unity). 5kHz should be attenuated (~0.3).
    let ratio = measure_spectral_ratio(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                C1: cap(10n)
            }
            nets {
                in -> R1.a
                R1.b -> C1.a
                C1.b -> gnd
                R1.b -> out
            }
            controls {}
        }"#,
    );
    eprintln!("RC LPF: 5kHz/200Hz ratio={ratio:.4}");
    assert!(
        ratio < 0.7,
        "RC LPF should attenuate 5kHz relative to 200Hz: ratio={ratio:.4}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. BlackFeedback stages — op-amp gain accuracy
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bf_inverting_gain_10() {
    // Inverting amp: Rf=100k, Ri=10k. Gain = Rf/Ri = 10.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    );
    eprintln!("BF inverting gain=10: measured={gain:.2}");
    assert!(
        gain > 7.0 && gain < 13.0,
        "Inverting gain should be ~10: {gain:.2}"
    );
}

#[test]
fn bf_inverting_gain_2() {
    // Gain = 20k/10k = 2.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(20k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    );
    eprintln!("BF inverting gain=2: measured={gain:.2}");
    assert!(
        gain > 1.5 && gain < 3.0,
        "Inverting gain should be ~2: {gain:.2}"
    );
}

#[test]
fn bf_noninverting_gain_11() {
    // Non-inverting: gain = 1 + Rf/Ri = 1 + 100k/10k = 11.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                U1: opamp(tl072)
                Rf: resistor(100k)
                Ri: resistor(10k)
            }
            nets {
                in -> U1.pos
                U1.neg -> Ri.a
                Ri.b -> gnd
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.out -> out
            }
            controls {}
        }"#,
    );
    eprintln!("BF non-inverting gain=11: measured={gain:.2}");
    assert!(
        gain > 8.0 && gain < 14.0,
        "Non-inverting gain should be ~11: {gain:.2}"
    );
}

#[test]
fn bf_unity_follower() {
    // Unity follower: out→neg, gain = 1.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                U1: opamp(tl072)
            }
            nets {
                in -> U1.pos
                U1.out -> U1.neg
                U1.out -> out
            }
            controls {}
        }"#,
    );
    eprintln!("BF unity follower: gain={gain:.4}");
    assert!(
        gain > 0.8 && gain < 1.2,
        "Unity follower should be ~1.0: {gain:.4}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. WDF stages — op-amp + diode clipping
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn wdf_opamp_diode_clips_at_diode_voltage() {
    // Op-amp with diode pair in feedback. At high gain, output should
    // clip at ~0.6V (silicon diode forward voltage).
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(500k)
                D1: diode(silicon)
                D2: diode(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.neg -> D1.a
                D1.b -> U1.out
                U1.neg -> D2.b
                D2.a -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Drive with large signal to ensure clipping
    for s in 0..2000 {
        let input = 0.1 * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin();
        compiled.process(input);
    }

    let mut peak = 0.0f64;
    for s in 0..220 {
        let input = 0.1 * (std::f64::consts::TAU * FREQ * (2000 + s) as f64 / SR).sin();
        peak = peak.max(compiled.process(input).abs());
    }

    eprintln!("Diode clip: peak={peak:.4}V");
    // Should clip around 0.5-0.7V (silicon diode Vf)
    // Allow wider range for WDF approximation
    assert!(peak > 0.1, "Should produce output: {peak:.4}V");
    assert!(peak < 2.0, "Should clip below 2V: {peak:.4}V");
}

#[test]
fn wdf_opamp_diode_gain_increases_with_drive() {
    // At low drive, diodes don't clip — gain ≈ Rf/Ri.
    // At high drive, diodes clip — output limited.
    // Low drive output should be proportionally larger.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
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
        }"#,
    )
    .expect("parse");

    let measure = |amplitude: f64| -> f64 {
        let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");
        for s in 0..1000 {
            let input = amplitude * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin();
            compiled.process(input);
        }
        let mut peak = 0.0f64;
        for s in 0..220 {
            let input = amplitude * (std::f64::consts::TAU * FREQ * (1000 + s) as f64 / SR).sin();
            peak = peak.max(compiled.process(input).abs());
        }
        peak
    };

    let out_lo = measure(0.001); // Well below clipping
    let out_hi = measure(0.1); // Should clip

    let gain_lo = out_lo / 0.001;
    let gain_hi = out_hi / 0.1;

    eprintln!("Diode gain: lo_gain={gain_lo:.2} (amp=0.001), hi_gain={gain_hi:.2} (amp=0.1)");
    // At low amplitude, gain should be close to Rf/Ri = 10
    assert!(
        gain_lo > 3.0,
        "Low-amplitude gain should be significant: {gain_lo:.2}"
    );
    // At high amplitude, gain should be compressed (diode clips)
    assert!(
        gain_hi < gain_lo,
        "High-amplitude gain should be compressed: hi={gain_hi:.2} < lo={gain_lo:.2}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 3a. Asymmetric diode clipping — both diodes must be modeled
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn wdf_asymmetric_diodes_both_modeled() {
    // Two separate diodes (D_ge: germanium, D_si: silicon) in feedback.
    // Asymmetric clipping: positive clips at ~0.3V (Ge), negative at ~0.6V (Si).
    // The output waveform should be asymmetric.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D_ge: diode(germanium)
                D_si: diode(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.neg -> D_ge.a
                D_ge.b -> U1.out
                U1.neg -> D_si.b
                D_si.a -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Drive hard to ensure clipping
    for s in 0..2000 {
        let input = 0.05 * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin();
        compiled.process(input);
    }

    // Measure positive and negative peaks separately
    let mut pos_peak = 0.0f64;
    let mut neg_peak = 0.0f64;
    for s in 0..220 {
        let input = 0.05 * (std::f64::consts::TAU * FREQ * (2000 + s) as f64 / SR).sin();
        let out = compiled.process(input);
        pos_peak = pos_peak.max(out);
        neg_peak = neg_peak.min(out);
    }

    eprintln!("Asymmetric diodes: pos={pos_peak:.4}, neg={neg_peak:.4}");

    // Both polarities should produce output
    assert!(
        pos_peak > 0.05,
        "Positive peak should be significant: {pos_peak:.4}"
    );
    assert!(
        neg_peak < -0.05,
        "Negative peak should be significant: {neg_peak:.4}"
    );

    // Asymmetry: germanium clips lower than silicon
    // So positive and negative peaks should differ
    let asymmetry = (pos_peak.abs() - neg_peak.abs()).abs();
    eprintln!("Asymmetry: {asymmetry:.4}");
    assert!(asymmetry > 0.01,
        "Ge vs Si should create asymmetric clipping: pos={pos_peak:.4}, neg={neg_peak:.4}, diff={asymmetry:.4}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 3b. WDF clipping with input coupling — the screamer pattern
//
// When the input coupling (R_in + Cin) is in a separate group from the
// clipping stage, the OpAmpRoot needs to get Ri from the pipeline.
// Without it, gain=1 and the clipping stage barely amplifies.
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn wdf_clipping_with_separate_input_coupling_amplifies() {
    // Screamer-like: Cin + R_in → U1.neg → Drive(feedback) + D1(clipping).
    // The input coupling (Cin + R_in) ends up in a separate FlowGroup.
    // The clipping stage should still amplify with gain ≈ Drive/R_in.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                Cin: cap(47n)
                R_in: resistor(10k)
                U1: opamp(tl072)
                Drive: pot(500k, a)
                D1: diode(silicon)
            }
            nets {
                in -> Cin.a
                Cin.b -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Drive.a
                Drive.b -> U1.out
                U1.neg -> D1.a
                D1.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls { Drive.position -> "Drive" [0.0, 1.0] = 0.5 }
        }"#,
    );
    eprintln!("Clipping+coupling: gain={gain:.2}");
    // At default Drive=50% → Rf≈250k, Ri=10k → gain≈25.
    // Diode clips, so actual gain depends on signal level.
    // At 0.01V input, 25x = 0.25V (below diode Vf) → should amplify.
    assert!(gain > 3.0, "Should amplify with input coupling: {gain:.2}");
}

#[test]
fn wdf_clipping_gain_matches_single_stage() {
    // Compare: same Rf/Ri ratio, one with coupling cap (multi-group)
    // and one without (single group). Gains should be similar.
    let gain_single = measure_gain(
        r#"
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
        }"#,
    );

    let gain_coupled = measure_gain(
        r#"
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
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.neg -> D1.a
                D1.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    );

    eprintln!("Single={gain_single:.2}, Coupled={gain_coupled:.2}");
    // Both have Rf/Ri=10. Gains should be in the same ballpark.
    assert!(
        gain_single > 3.0,
        "Single stage should amplify: {gain_single:.2}"
    );
    assert!(
        gain_coupled > 3.0,
        "Coupled stage should amplify: {gain_coupled:.2}"
    );
    let ratio = gain_single.max(gain_coupled) / gain_single.min(gain_coupled).max(0.01);
    assert!(
        ratio < 5.0,
        "Gains should be within 5x: single={gain_single:.2}, coupled={gain_coupled:.2}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Pot dividers — voltage division accuracy
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn pot_divider_at_50_percent() {
    // Pot at 50%: gain ≈ 0.5. Not exact due to taper and loading.
    let gain = measure_gain_with_control(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(1k)
                Volume: pot(100k, b)
            }
            nets {
                in -> R1.a
                R1.b -> Volume.a
                Volume.w -> out
                Volume.b -> gnd
            }
            controls { Volume.position -> "Volume" [0.0, 1.0] = 0.5 }
        }"#,
        "Volume",
        0.5,
    );
    eprintln!("Pot @50%: gain={gain:.4}");
    assert!(
        gain > 0.2 && gain < 0.8,
        "Pot at 50% should give ~0.5: {gain:.4}"
    );
}

#[test]
fn pot_divider_at_100_percent() {
    // Pot at 100%: wiper at top → nearly unity gain.
    let gain = measure_gain_with_control(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(1k)
                Volume: pot(100k, b)
            }
            nets {
                in -> R1.a
                R1.b -> Volume.a
                Volume.w -> out
                Volume.b -> gnd
            }
            controls { Volume.position -> "Volume" [0.0, 1.0] = 0.5 }
        }"#,
        "Volume",
        1.0,
    );
    eprintln!("Pot @100%: gain={gain:.4}");
    assert!(gain > 0.7, "Pot at 100% should be near unity: {gain:.4}");
}

#[test]
fn pot_divider_at_10_percent() {
    // Pot at 10%: heavy attenuation.
    let gain = measure_gain_with_control(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(1k)
                Volume: pot(100k, b)
            }
            nets {
                in -> R1.a
                R1.b -> Volume.a
                Volume.w -> out
                Volume.b -> gnd
            }
            controls { Volume.position -> "Volume" [0.0, 1.0] = 0.5 }
        }"#,
        "Volume",
        0.1,
    );
    eprintln!("Pot @10%: gain={gain:.4}");
    assert!(
        gain < 0.3,
        "Pot at 10% should be heavily attenuated: {gain:.4}"
    );
}

#[test]
fn pot_divider_monotonic() {
    // Gain should monotonically increase from 0% to 100%.
    let src = r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(1k)
                Volume: pot(100k, b)
            }
            nets {
                in -> R1.a
                R1.b -> Volume.a
                Volume.w -> out
                Volume.b -> gnd
            }
            controls { Volume.position -> "Volume" [0.0, 1.0] = 0.5 }
        }"#;

    let g1 = measure_gain_with_control(src, "Volume", 0.1);
    let g2 = measure_gain_with_control(src, "Volume", 0.3);
    let g3 = measure_gain_with_control(src, "Volume", 0.5);
    let g4 = measure_gain_with_control(src, "Volume", 0.7);
    let g5 = measure_gain_with_control(src, "Volume", 0.9);

    eprintln!("Pot monotonic: {g1:.4} → {g2:.4} → {g3:.4} → {g4:.4} → {g5:.4}");
    assert!(g2 > g1, "0.3 > 0.1: {g2:.4} > {g1:.4}");
    assert!(g3 > g2, "0.5 > 0.3: {g3:.4} > {g2:.4}");
    assert!(g4 > g3, "0.7 > 0.5: {g4:.4} > {g3:.4}");
    assert!(g5 > g4, "0.9 > 0.7: {g5:.4} > {g4:.4}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. Multi-stage cascades — gain stacking
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn cascade_two_inverting_amps() {
    // Two inverting amps in series: gain1=10, gain2=5 → total=50.
    // Phase inverts twice → net positive.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                U1: opamp(tl072)
                Rf1: resistor(100k)
                R2: resistor(10k)
                U2: opamp(tl072)
                Rf2: resistor(50k)
            }
            nets {
                in -> R1.a
                R1.b -> U1.neg
                U1.neg -> Rf1.a
                Rf1.b -> U1.out
                U1.pos -> gnd
                U1.out -> R2.a
                R2.b -> U2.neg
                U2.neg -> Rf2.a
                Rf2.b -> U2.out
                U2.pos -> gnd
                U2.out -> out
            }
            controls {}
        }"#,
    );
    eprintln!("Cascade 10×5=50: gain={gain:.1}");
    // Allow wide tolerance for GBW rolloff and oversampler effects
    assert!(gain > 20.0, "Cascade gain should be > 20: {gain:.1}");
    assert!(gain < 80.0, "Cascade gain should be < 80: {gain:.1}");
}

#[test]
fn cascade_gain_then_volume_divider() {
    // Gain=10 → Volume pot at 50% → expected ~5.
    let gain = measure_gain_with_control(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                Volume: pot(100k, b)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> Volume.a
                Volume.w -> out
                Volume.b -> gnd
            }
            controls { Volume.position -> "Volume" [0.0, 1.0] = 0.5 }
        }"#,
        "Volume",
        0.5,
    );
    eprintln!("Gain×Divider: 10 × 0.5 = {gain:.2}");
    assert!(
        gain > 2.0 && gain < 8.0,
        "Gain 10 × 0.5 divider should be ~5: {gain:.2}"
    );
}

#[test]
fn cascade_gain_then_unity_follower() {
    // Gain=10 → U2 unity follower → expected ~10 (follower is transparent).
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                U2: opamp(tl072)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> U2.pos
                U2.out -> U2.neg
                U2.out -> out
            }
            controls {}
        }"#,
    );
    eprintln!("Gain + follower: gain={gain:.2}");
    assert!(
        gain > 7.0 && gain < 13.0,
        "Gain 10 + follower should be ~10: {gain:.2}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. Output extraction specifics — where voltage is read
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn output_at_opamp_out_pin() {
    // Output directly from U1.out. Should match the op-amp's computed output.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(50k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    );
    eprintln!("Output @U1.out: gain={gain:.2}");
    assert!(
        gain > 3.0 && gain < 7.0,
        "Gain 50k/10k should be ~5: {gain:.2}"
    );
}

#[test]
fn output_through_coupling_cap() {
    // Output through a coupling cap (blocks DC, passes AC).
    // At 440Hz with C=10µF, impedance ≈ 36Ω — should pass most signal.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(50k)
                C_out: cap(10u)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> C_out.a
                C_out.b -> out
            }
            controls {}
        }"#,
    );
    eprintln!("Output through 10µF cap: gain={gain:.2}");
    // Should be close to the op-amp gain of 5, slight attenuation from cap
    assert!(gain > 2.0, "Signal should pass through 10µF cap: {gain:.2}");
}

#[test]
fn output_after_pot_wiper() {
    // Pot wiper connected to out. The extraction should read voltage at the wiper.
    let gain = measure_gain_with_control(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(1k)
                Volume: pot(10k, b)
            }
            nets {
                in -> R1.a
                R1.b -> Volume.a
                Volume.w -> out
                Volume.b -> gnd
            }
            controls { Volume.position -> "Volume" [0.0, 1.0] = 0.5 }
        }"#,
        "Volume",
        0.75,
    );
    eprintln!("Output @wiper 75%: gain={gain:.4}");
    assert!(
        gain > 0.5,
        "Wiper at 75% should give >50% signal: {gain:.4}"
    );
}

#[test]
fn output_after_multinl_named_pad() {
    // Regression: a general MultiNL stage must read the named `out` node,
    // not the first nonlinear port. At small signal the antiparallel diodes
    // are effectively off, so this is just a resistive pad:
    //   1k source + 9k top + 1k bottom => gain ≈ 1/11 = 0.091.
    // If the stage falls back to the diode node, gain is roughly 10x higher.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_src: resistor(1k)
                D_pos: diode(silicon)
                D_neg: diode(silicon)
                R_pad_top: resistor(9k)
                R_pad_bottom: resistor(1k)
            }
            nets {
                in -> R_src.a
                R_src.b -> clip
                clip -> D_pos.a
                D_pos.k -> gnd
                gnd -> D_neg.a
                D_neg.k -> clip
                clip -> R_pad_top.a
                R_pad_top.b -> out
                out -> R_pad_bottom.a
                R_pad_bottom.b -> gnd
            }
            controls {}
        }"#,
    )
    .expect("parse");
    let graph = super::graph::CircuitGraph::from_pedal(&pedal);
    let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
    let mut stage = super::rigid::build_general_mna_from_edges(&all_edges, &graph, SR)
        .expect("general MultiNL compile");

    for s in 0..1000 {
        let input = AMPLITUDE * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin();
        stage.process(input);
    }

    let n_samples = (2.0 * SR / FREQ).ceil() as usize;
    let mut peak = 0.0f64;
    for s in 0..n_samples {
        let input = AMPLITUDE * (std::f64::consts::TAU * FREQ * (1000 + s) as f64 / SR).sin();
        peak = peak.max(stage.process(input).abs());
    }

    let gain = peak / AMPLITUDE;
    eprintln!("MultiNL named output pad: gain={gain:.4}");
    assert!(
        gain > 0.04 && gain < 0.20,
        "MultiNL should extract named out after pad, expected ~0.09, got {gain:.4}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 7. Legend pedal gain sanity checks
// ═══════════════════════════════════════════════════════════════════════════

fn load_legend(name: &str) -> crate::dsl::PedalDef {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/{}.pedal",
        env!("CARGO_MANIFEST_DIR"),
        name
    );
    let source = std::fs::read_to_string(&path).unwrap_or_else(|_| panic!("Can't read {path}"));
    crate::dsl::parse_pedal_file(&source).unwrap_or_else(|e| panic!("{name}: parse error: {e}"))
}

fn legend_gain(name: &str) -> f64 {
    let pedal = load_legend(name);
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    for s in 0..2000 {
        let input = 0.01 * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin();
        compiled.process(input);
    }

    let mut peak = 0.0f64;
    for s in 0..220 {
        let input = 0.01 * (std::f64::consts::TAU * FREQ * (2000 + s) as f64 / SR).sin();
        peak = peak.max(compiled.process(input).abs());
    }

    peak / 0.01
}

#[test]
fn legend_screamer_gain_reasonable() {
    let gain = legend_gain("screamer");
    eprintln!("Screamer gain: {gain:.2}");
    // Screamer at default Drive (50%) should have moderate gain
    assert!(gain > 1.0, "Screamer should amplify: {gain:.2}");
    assert!(
        gain < 100.0,
        "Screamer shouldn't be insanely loud: {gain:.2}"
    );
}

#[test]
fn legend_sd1_gain_reasonable() {
    let gain = legend_gain("sd1");
    eprintln!("SD-1 gain: {gain:.2}");
    assert!(gain > 1.0, "SD-1 should amplify: {gain:.2}");
    assert!(gain < 100.0, "SD-1 shouldn't be insanely loud: {gain:.2}");
}

#[test]
fn legend_ratking_gain_reasonable() {
    let gain = legend_gain("ratking_non_invert_v1a");
    eprintln!("Ratking gain: {gain:.2}");
    assert!(gain > 1.0, "Ratking should amplify: {gain:.2}");
    assert!(
        gain < 200.0,
        "Ratking shouldn't be insanely loud: {gain:.2}"
    );
}

#[test]
fn legend_goldenrod_gain_reasonable() {
    let gain = legend_gain("goldenrod");
    eprintln!("Goldenrod gain: {gain:.2}");
    assert!(gain > 0.5, "Goldenrod should produce output: {gain:.2}");
    assert!(
        gain < 100.0,
        "Goldenrod shouldn't be insanely loud: {gain:.2}"
    );
}

#[test]
fn legend_muff_gain_reasonable() {
    let gain = legend_gain("muff");
    eprintln!("Muff gain: {gain:.2}");
    assert!(gain > 1.0, "Muff should amplify: {gain:.2}");
    assert!(gain < 200.0, "Muff shouldn't be insanely loud: {gain:.2}");
}

#[test]
fn legend_rangemaster_gain_reasonable() {
    let gain = legend_gain("rangemaster");
    eprintln!("Rangemaster gain: {gain:.2}");
    assert!(gain > 1.0, "Rangemaster should boost: {gain:.2}");
    assert!(
        gain < 100.0,
        "Rangemaster shouldn't be insanely loud: {gain:.2}"
    );
}

#[test]
fn legend_blues_gain_reasonable() {
    let gain = legend_gain("blues");
    eprintln!("Blues gain: {gain:.2}");
    assert!(gain > 0.5, "Blues should produce output: {gain:.2}");
    assert!(gain < 100.0, "Blues shouldn't be insanely loud: {gain:.2}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 8. Sallen-Key active filter extraction (pedalkernel-ht0o)
//
// Unity-gain Sallen-Key uses U1.neg wired directly to U1.out (unity follower).
// The bug: uf.union(id_pos, id_out) in graph.rs collapsed the reactive network
// by shorting C2/R2 junction (pos) to the output, yielding −240 dB silence.
// After the fix (union removed), the passive WDF stages see the correct SK
// topology and the filter response is verified here.
//
// Component values: R1=R2=10k, C1=C2=10n → f0 ≈ 1592 Hz
// LPF: passes 440 Hz (below fc), rejects 10 kHz (above fc)
// HPF: rejects 440 Hz (below fc), passes 10 kHz (above fc)
// ═══════════════════════════════════════════════════════════════════════════

/// Measure peak output gain for a fixed frequency on an inline pedal string.
fn sallen_key_gain_at_freq(pedal_src: &str, freq: f64) -> f64 {
    let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");
    // warmup
    for s in 0..2000 {
        let x = AMPLITUDE * (std::f64::consts::TAU * freq * s as f64 / SR).sin();
        compiled.process(x);
    }
    let n = (4.0 * SR / freq).ceil() as usize;
    let mut peak = 0.0f64;
    for s in 0..n {
        let x = AMPLITUDE * (std::f64::consts::TAU * freq * (2000 + s) as f64 / SR).sin();
        peak = peak.max(compiled.process(x).abs());
    }
    peak / AMPLITUDE
}

const SK_LPF: &str = r#"
    pedal "Extraction Sallen Key Lowpass" {
      components {
        U1: opamp(tl072)
        R1: resistor(10k)
        R2: resistor(10k)
        C1: cap(10n)
        C2: cap(10n)
        R_out: resistor(1k)
      }
      nets {
        in -> R1.a
        R1.b -> R2.a
        R2.b -> U1.pos
        R1.b -> C1.a
        C1.b -> U1.out
        U1.pos -> C2.a
        C2.b -> gnd
        U1.neg -> U1.out
        U1.out -> R_out.a
        R_out.b -> out
      }
    }"#;

const SK_HPF: &str = r#"
    pedal "Extraction Sallen Key Highpass" {
      components {
        U1: opamp(tl072)
        C1: cap(10n)
        C2: cap(10n)
        R1: resistor(10k)
        R2: resistor(10k)
        R_out: resistor(1k)
      }
      nets {
        in -> C1.a
        C1.b -> C2.a
        C2.b -> U1.pos
        C1.b -> R1.a
        R1.b -> U1.out
        U1.pos -> R2.a
        R2.b -> gnd
        U1.neg -> U1.out
        U1.out -> R_out.a
        R_out.b -> out
      }
    }"#;

#[test]
fn sallen_key_lpf_passes_low_freq() {
    // 440 Hz is well below f0≈1592 Hz — passband, expect gain close to 1.0.
    // Any non-silent output (> 0.1) passes the "silence → alive" acceptance bar.
    let gain = sallen_key_gain_at_freq(SK_LPF, 440.0);
    let gain_db = 20.0 * gain.log10();
    eprintln!("SK LPF at 440 Hz: gain={gain:.4} ({gain_db:.1} dB)");
    assert!(
        gain > 0.1,
        "SK LPF must pass 440 Hz (was −240 dB silence before fix): gain={gain:.4}"
    );
}

#[test]
fn sallen_key_lpf_attenuates_high_freq() {
    // 10 kHz is well above f0≈1592 Hz — stopband, expect significant attenuation.
    // A 2nd-order LPF at 10x fc should attenuate by ~40 dB (factor ~0.01).
    let gain_440 = sallen_key_gain_at_freq(SK_LPF, 440.0);
    let gain_10k = sallen_key_gain_at_freq(SK_LPF, 10_000.0);
    let ratio_db = 20.0 * (gain_10k / gain_440.max(1e-12)).log10();
    eprintln!(
        "SK LPF: 440 Hz gain={gain_440:.4}, 10 kHz gain={gain_10k:.4}, roll-off={ratio_db:.1} dB"
    );
    assert!(
        gain_10k < gain_440 * 0.5,
        "SK LPF must roll off at 10 kHz vs 440 Hz: ratio={ratio_db:.1} dB"
    );
}

#[test]
fn sallen_key_hpf_passes_high_freq() {
    // 10 kHz is well above f0≈1592 Hz — passband for HPF, expect gain close to 1.0.
    let gain = sallen_key_gain_at_freq(SK_HPF, 10_000.0);
    let gain_db = 20.0 * gain.log10();
    eprintln!("SK HPF at 10 kHz: gain={gain:.4} ({gain_db:.1} dB)");
    assert!(
        gain > 0.1,
        "SK HPF must pass 10 kHz (was −240 dB silence before fix): gain={gain:.4}"
    );
}

#[test]
fn sallen_key_hpf_attenuates_low_freq() {
    // 200 Hz is well below f0≈1592 Hz — stopband for HPF.
    let gain_200 = sallen_key_gain_at_freq(SK_HPF, 200.0);
    let gain_10k = sallen_key_gain_at_freq(SK_HPF, 10_000.0);
    let ratio_db = 20.0 * (gain_10k / gain_200.max(1e-12)).log10();
    eprintln!(
        "SK HPF: 200 Hz gain={gain_200:.4}, 10 kHz gain={gain_10k:.4}, ratio={ratio_db:.1} dB"
    );
    assert!(
        gain_10k > gain_200 * 2.0,
        "SK HPF must pass 10 kHz more than 200 Hz: ratio={ratio_db:.1} dB"
    );
}
