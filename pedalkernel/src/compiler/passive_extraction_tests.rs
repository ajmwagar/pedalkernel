// ═══════════════════════════════════════════════════════════════════════════
// Passive WDF Stage Extraction Tests
//
// These test the output voltage extraction from passive WDF stages:
// Passthrough root, ShortCircuit root, and combinations.
//
// The core question: when signal passes through a passive WDF stage
// (resistor, cap, pot divider, diode to ground), what voltage comes out?
//
// Known issues:
// - Wire (single R): gain=1.5 (should be 1.0)
// - RC LPF: 5kHz louder than 200Hz (should be opposite)
// - Coupling cap: attenuates too much
// - Diode to GND stages: may zero the signal
// ═══════════════════════════════════════════════════════════════════════════

use super::spqr_build::compile_via_spqr;
use crate::PedalProcessor;

const SR: f64 = 48000.0;
const FREQ: f64 = 440.0;

fn measure_gain(src: &str) -> f64 {
    let pedal = crate::dsl::parse_pedal_file(src).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");
    let amp = 0.01;
    for s in 0..1000 {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }
    let mut peak = 0.0f64;
    for s in 1000..1220 {
        peak = peak.max(
            compiled
                .process(amp * (std::f64::consts::TAU * FREQ * (1000 + s) as f64 / SR).sin())
                .abs(),
        );
    }
    peak / amp
}

// ═══════════════════════════════════════════════════════════════════════════
// 1. Single passive element — baseline
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn passive_wire_dc_step() {
    // Simplest possible test: one DC sample through a wire.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(1k) }
            nets { in -> R1.a  R1.b -> out }
            controls {}
        }"#,
    )
    .expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Process DC=1.0 for 100 samples to let filters settle
    let mut out = 0.0;
    for _ in 0..100 {
        out = compiled.process(1.0);
    }
    eprintln!("DC steady-state: in=1.0, out={out:.6}");
    assert!(
        (out.abs() - 1.0).abs() < 0.6,
        "DC steady-state should be ~1.0: {out:.6}"
    );
}

#[test]
fn passive_wire_is_unity() {
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(1k) }
            nets { in -> R1.a  R1.b -> out }
            controls {}
        }"#,
    );
    eprintln!("Wire: {gain:.3}");
    assert!(gain > 0.8 && gain < 1.2, "Wire should be unity: {gain:.3}");
}

#[test]
fn passive_coupling_cap_passes_audio() {
    // 10µF cap at 440Hz: Xc = 1/(2π·440·10e-6) ≈ 36Ω.
    // With 1k series R, attenuation = R/(R+Xc) ≈ 0.97. Nearly unity.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(1k)
                C1: cap(10u)
            }
            nets {
                in -> R1.a
                R1.b -> C1.a
                C1.b -> out
            }
            controls {}
        }"#,
    );
    eprintln!("Coupling cap: {gain:.3}");
    assert!(gain > 0.5, "10µF cap should pass 440Hz: {gain:.3}");
}

#[test]
fn passive_small_cap_blocks_low_freq() {
    // 1nF cap at 440Hz: Xc = 1/(2π·440·1e-9) ≈ 362kΩ.
    // With 10k series R, attenuation = R/(R+Xc) ≈ 0.027. Heavy.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                C1: cap(1n)
            }
            nets {
                in -> R1.a
                R1.b -> C1.a
                C1.b -> out
            }
            controls {}
        }"#,
    );
    eprintln!("1nF cap: {gain:.4}");
    assert!(gain < 0.2, "1nF cap should block 440Hz: {gain:.4}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Voltage dividers — correct ratio
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn passive_divider_50_percent() {
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
    eprintln!("50% divider: {gain:.3}");
    assert!(gain > 0.35 && gain < 0.65, "Should be ~0.5: {gain:.3}");
}

#[test]
fn passive_divider_10_percent() {
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(90k)
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
    eprintln!("10% divider: {gain:.3}");
    assert!(gain > 0.05 && gain < 0.2, "Should be ~0.1: {gain:.3}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. RC filters — correct frequency response
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn rc_lowpass_attenuates_hf() {
    // R=10k, C=10nF → fc = 1/(2π·R·C) ≈ 1592Hz.
    // 200Hz should pass. 5kHz should be attenuated.
    let pedal = crate::dsl::parse_pedal_file(
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
    )
    .expect("parse");

    let measure_freq = |freq: f64| -> f64 {
        let mut c = compile_via_spqr(&pedal, SR).expect("compile");
        let amp = 0.01;
        for s in 0..1000 {
            c.process(amp * (std::f64::consts::TAU * freq * s as f64 / SR).sin());
        }
        let mut peak = 0.0f64;
        for s in 1000..1220 {
            peak = peak.max(
                c.process(amp * (std::f64::consts::TAU * freq * (1000 + s) as f64 / SR).sin())
                    .abs(),
            );
        }
        peak
    };

    let lo = measure_freq(200.0);
    let hi = measure_freq(5000.0);
    let ratio = hi / lo.max(1e-10);
    eprintln!("RC LPF: 200Hz={lo:.4}, 5kHz={hi:.4}, ratio={ratio:.3}");
    assert!(
        ratio < 0.7,
        "5kHz should be attenuated vs 200Hz: ratio={ratio:.3}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Diode to ground — clipping at Vf
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn diode_to_ground_clips_signal() {
    // Diode from signal to GND clips positive peaks at ~0.6V.
    // Small signal (0.01V) should pass through unclipped.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(1k)
                D1: diode(silicon)
            }
            nets {
                in -> R1.a
                R1.b -> D1.a
                D1.b -> gnd
                R1.b -> out
            }
            controls {}
        }"#,
    );
    eprintln!("Diode to GND (small signal): {gain:.3}");
    // At 0.01V, diode doesn't conduct → signal passes through
    assert!(gain > 0.3, "Small signal should mostly pass: {gain:.3}");
}

#[test]
fn diode_to_ground_limits_large_signal() {
    // Drive 0.5V through 1k into diode to GND.
    // Diode clips at ~0.6V → output limited.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(1k)
                D1: diode(silicon)
            }
            nets {
                in -> R1.a
                R1.b -> D1.a
                D1.b -> gnd
                R1.b -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");
    let amp = 0.5;
    for s in 0..1000 {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }
    let mut peak = 0.0f64;
    for s in 1000..1220 {
        peak = peak.max(
            compiled
                .process(amp * (std::f64::consts::TAU * FREQ * (1000 + s) as f64 / SR).sin())
                .abs(),
        );
    }

    eprintln!("Diode to GND (large signal): peak={peak:.3}V");
    assert!(peak < 1.0, "Diode should limit output: {peak:.3}V");
    assert!(peak > 0.1, "Should still produce output: {peak:.3}V");
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. Multi-stage passive cascades — signal should survive
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn cascade_coupling_caps_preserve_signal() {
    // Three coupling caps in series. Each should pass 440Hz.
    // Total attenuation should be minimal.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                C1: cap(10u)
                R1: resistor(10k)
                C2: cap(10u)
                R2: resistor(10k)
                C3: cap(10u)
            }
            nets {
                in -> C1.a
                C1.b -> R1.a
                R1.b -> C2.a
                C2.b -> R2.a
                R2.b -> C3.a
                C3.b -> out
            }
            controls {}
        }"#,
    );
    eprintln!("3x coupling caps: {gain:.3}");
    assert!(gain > 0.3, "Coupling caps should pass 440Hz: {gain:.3}");
}

#[test]
fn cascade_gain_then_passive_preserves_level() {
    // Gain stage (10x) → coupling cap → volume divider (50%).
    // Expected: ~5x overall.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                C_out: cap(10u)
                R_div1: resistor(10k)
                R_div2: resistor(10k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> C_out.a
                C_out.b -> R_div1.a
                R_div1.b -> R_div2.a
                R_div2.b -> gnd
                R_div1.b -> out
            }
            controls {}
        }"#,
    );
    eprintln!("Gain(10) + cap + divider(50%): {gain:.1}");
    assert!(gain > 2.0, "Should preserve most of the gain: {gain:.1}");
    assert!(gain < 8.0, "Divider should attenuate: {gain:.1}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. Level/Volume pot — 3-terminal pot in passive group
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn level_pot_standalone_changes_output() {
    // Simple: signal → Level pot → out, Level.b → gnd.
    // Sweeping should change output level.
    let src = r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(1k)
                Level: pot(100k, a)
            }
            nets {
                in -> R1.a
                R1.b -> Level.a
                Level.w -> out
                Level.b -> gnd
            }
            controls { Level.position -> "Level" [0.0, 1.0] = 0.5 }
        }"#;

    let pedal = crate::dsl::parse_pedal_file(src).expect("parse");

    let measure = |pos: f64| -> f64 {
        let mut c = compile_via_spqr(&pedal, SR).expect("compile");
        c.set_control_immediate("Level", pos);
        let amp = 0.01;
        for s in 0..500 {
            c.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
        }
        let mut peak = 0.0f64;
        for s in 500..720 {
            peak = peak.max(
                c.process(amp * (std::f64::consts::TAU * FREQ * (500 + s) as f64 / SR).sin())
                    .abs(),
            );
        }
        peak
    };

    let lo = measure(0.1);
    let hi = measure(0.9);
    eprintln!("Level pot: @0.1={lo:.4}, @0.9={hi:.4}");
    assert!(
        hi > lo * 1.5,
        "Level pot should change output: lo={lo:.4}, hi={hi:.4}"
    );
}

#[test]
fn level_pot_after_gain_stage_changes_output() {
    // Gain(10x) → Level pot → out. The screamer/SD-1 pattern.
    let src = r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                Level: pot(100k, a)
                C_out: cap(10u)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> Level.a
                Level.w -> C_out.a
                C_out.b -> out
                Level.b -> gnd
            }
            controls { Level.position -> "Level" [0.0, 1.0] = 0.5 }
        }"#;

    let pedal = crate::dsl::parse_pedal_file(src).expect("parse");

    let measure = |pos: f64| -> f64 {
        let mut c = compile_via_spqr(&pedal, SR).expect("compile");
        c.set_control_immediate("Level", pos);
        let amp = 0.01;
        for s in 0..500 {
            c.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
        }
        let mut peak = 0.0f64;
        for s in 500..720 {
            peak = peak.max(
                c.process(amp * (std::f64::consts::TAU * FREQ * (500 + s) as f64 / SR).sin())
                    .abs(),
            );
        }
        peak
    };

    let lo = measure(0.1);
    let hi = measure(0.9);
    eprintln!("Level after gain: @0.1={lo:.4}, @0.9={hi:.4}");
    assert!(
        hi > lo * 1.5,
        "Level pot should change output: lo={lo:.4}, hi={hi:.4}"
    );
}

#[test]
fn level_pot_complement_correct_polarity() {
    // At position=1.0, Level should give maximum output (near unity).
    // At position=0.0, Level should give minimum output (near zero).
    let src = r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(1k)
                Level: pot(100k, b)
            }
            nets {
                in -> R1.a
                R1.b -> Level.a
                Level.w -> out
                Level.b -> gnd
            }
            controls { Level.position -> "Level" [0.0, 1.0] = 0.5 }
        }"#;

    let pedal = crate::dsl::parse_pedal_file(src).expect("parse");

    let measure = |pos: f64| -> f64 {
        let mut c = compile_via_spqr(&pedal, SR).expect("compile");
        c.set_control_immediate("Level", pos);
        let amp = 0.01;
        for s in 0..500 {
            c.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
        }
        let mut peak = 0.0f64;
        for s in 500..720 {
            peak = peak.max(
                c.process(amp * (std::f64::consts::TAU * FREQ * (500 + s) as f64 / SR).sin())
                    .abs(),
            );
        }
        peak
    };

    let at_zero = measure(0.0);
    let at_full = measure(1.0);
    eprintln!("Level polarity: @0.0={at_zero:.4}, @1.0={at_full:.4}");
    assert!(
        at_full > at_zero * 3.0,
        "Full level should be much louder than zero: full={at_full:.4}, zero={at_zero:.4}"
    );
}

#[test]
fn level_pot_in_tone_network_changes_output() {
    // Screamer pattern: Tone pot + Level pot + coupling caps in one group.
    let src = r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                R_tone: resistor(1k)
                C_tone: cap(220n)
                Tone: pot(20k, b)
                Level: pot(100k, a)
                C_out: cap(10u)
                R_out: resistor(10k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> R_tone.a
                R_tone.b -> C_tone.a
                C_tone.b -> Tone.a
                R_tone.b -> Tone.b
                Tone.w -> Level.a
                Level.w -> C_out.a
                Level.b -> gnd
                C_out.b -> R_out.a
                R_out.b -> out
            }
            controls {
                Tone.position -> "Tone" [0.0, 1.0] = 0.5
                Level.position -> "Level" [0.0, 1.0] = 0.5
            }
        }"#;

    let pedal = crate::dsl::parse_pedal_file(src).expect("parse");

    let measure = |pos: f64| -> f64 {
        let mut c = compile_via_spqr(&pedal, SR).expect("compile");
        c.set_control_immediate("Level", pos);
        let amp = 0.01;
        for s in 0..500 {
            c.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
        }
        let mut peak = 0.0f64;
        for s in 500..720 {
            peak = peak.max(
                c.process(amp * (std::f64::consts::TAU * FREQ * (500 + s) as f64 / SR).sin())
                    .abs(),
            );
        }
        peak
    };

    let lo = measure(0.1);
    let hi = measure(0.9);
    eprintln!("Level in tone network: @0.1={lo:.4}, @0.9={hi:.4}");
    assert!(
        hi > lo * 1.5,
        "Level pot should change output in tone network: lo={lo:.4}, hi={hi:.4}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 7. RAT-style diode-to-ground after gain stage
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn gain_then_diode_to_ground_produces_clipped_output() {
    // Gain stage → diode to GND. The gain stage amplifies, the diode clips.
    // At small signal (0.001V), gain=10 → 0.01V (below clip) → passes.
    // At large signal (0.1V), gain=10 → 1.0V → clips at ~0.6V.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D1: diode(silicon)
                D2: diode(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> D1.a
                D1.b -> gnd
                U1.out -> D2.b
                D2.a -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    // Small signal
    let mut c1 = compile_via_spqr(&pedal, SR).expect("compile");
    for s in 0..1000 {
        c1.process(0.001 * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }
    let mut small_peak = 0.0f64;
    for s in 1000..1220 {
        small_peak = small_peak.max(
            c1.process(0.001 * (std::f64::consts::TAU * FREQ * (1000 + s) as f64 / SR).sin())
                .abs(),
        );
    }

    // Large signal
    let mut c2 = compile_via_spqr(&pedal, SR).expect("compile");
    for s in 0..1000 {
        c2.process(0.1 * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }
    let mut large_peak = 0.0f64;
    for s in 1000..1220 {
        large_peak = large_peak.max(
            c2.process(0.1 * (std::f64::consts::TAU * FREQ * (1000 + s) as f64 / SR).sin())
                .abs(),
        );
    }

    let small_gain = small_peak / 0.001;
    let large_gain = large_peak / 0.1;

    eprintln!("Gain+diode: small_gain={small_gain:.1}, large_gain={large_gain:.1}");
    eprintln!("  small_peak={small_peak:.4}V, large_peak={large_peak:.4}V");

    assert!(
        small_peak > 0.002,
        "Small signal should amplify: {small_peak:.4}V"
    );
    assert!(
        large_peak > 0.05,
        "Large signal should produce output: {large_peak:.4}V"
    );
    assert!(large_peak < 1.5, "Diodes should limit: {large_peak:.4}V");
    assert!(small_gain > large_gain * 1.5,
        "Small-signal gain should exceed large-signal gain (compression): {small_gain:.1} vs {large_gain:.1}");
}
