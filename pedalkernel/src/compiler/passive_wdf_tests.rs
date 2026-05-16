// ═══════════════════════════════════════════════════════════════════════════
// Passive WDF tree tests
//
// TDD: build up from the simplest passive network to the Screamer's tone
// stage. Find exactly where the WDF tree breaks for multi-component
// passive groups.
//
// No active components. If any of these fail, the WDF itself is broken.
// ═══════════════════════════════════════════════════════════════════════════

use super::compiled::CompiledPedal;
use super::spqr_build::compile_via_spqr;
use crate::PedalProcessor;

const SR: f64 = 48000.0;
const FREQ: f64 = 440.0;

/// Measure gain: peak output / peak input over a few cycles.
fn measure_gain(source: &str) -> f64 {
    let pedal = crate::dsl::parse_pedal_file(source).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");
    let amp = 0.1;
    // Warmup (let caps charge)
    for s in 0..4000 {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }
    let mut peak = 0.0f64;
    for s in 0..500 {
        let out =
            compiled.process(amp * (std::f64::consts::TAU * FREQ * (4000 + s) as f64 / SR).sin());
        peak = peak.max(out.abs());
    }
    peak / amp
}

/// Measure gain with metering dump for debugging.
fn measure_gain_metered(source: &str, label: &str) -> f64 {
    let pedal = crate::dsl::parse_pedal_file(source).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");
    compiled.enable_metering(128);
    let amp = 0.1;
    for s in 0..4000 {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }
    let mut peak = 0.0f64;
    for s in 0..500 {
        let out =
            compiled.process(amp * (std::f64::consts::TAU * FREQ * (4000 + s) as f64 / SR).sin());
        peak = peak.max(out.abs());
    }
    let gain = peak / amp;

    // Dump per-stage metering
    let metrics = compiled.read_metrics();
    let n = compiled.stages.len().min(crate::metering::MAX_STAGES);
    eprintln!("  {label}: gain={gain:.3} ({n} stages)");
    for i in 0..n {
        let lvl = metrics.stage_levels[i];
        let db = if lvl > 1e-10 {
            20.0 * (lvl as f64).log10()
        } else {
            -120.0
        };
        #[cfg(debug_assertions)]
        {
            let (stype, lbl, bypass) = match &compiled.stages[i] {
                super::compiled::Stage::Wdf(w) => ("Wdf", w.debug_label.as_str(), w.bypass_serial),
                super::compiled::Stage::MultiNl(m) => {
                    ("MNL", m.debug_label.as_str(), m.bypass_serial)
                }
                super::compiled::Stage::Iir(s) => ("Iir", s.debug_label.as_str(), s.bypass_serial),
                super::compiled::Stage::StateSpace(s) => {
                    ("SS", s.debug_label.as_str(), s.bypass_serial)
                }
                super::compiled::Stage::BlackFeedback(b) => {
                    ("BF", b.debug_label.as_str(), b.bypass_serial)
                }
                super::compiled::Stage::BlockwiseKMethod(bk) => {
                    ("BKM", "blockwise", bk.bypass_serial)
                }
            };
            let bp = if bypass { " BYPASS" } else { "" };
            eprintln!("    stage {i}: [{stype}] [{lbl}]{bp} → {lvl:.4} ({db:.1} dB)");
        }
    }
    gain
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 1: single passive components
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn single_resistor_passes_signal() {
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k) }
            nets { in -> R1.a  R1.b -> out }
            controls {}
        }"#,
    );
    eprintln!("Single resistor: gain={gain:.3}");
    // Should be ~1.0 (unity, maybe with 1.5× oversampler artifact)
    assert!(gain > 0.5, "Resistor should pass signal: gain={gain:.3}");
}

#[test]
fn coupling_cap_passes_440hz() {
    // 1µF at 440Hz: Xc ≈ 362Ω. Near unity for audio.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components { C1: cap(1u) }
            nets { in -> C1.a  C1.b -> out }
            controls {}
        }"#,
    );
    eprintln!("Coupling cap 1µF: gain={gain:.3}");
    assert!(gain > 0.5, "1µF cap should pass 440Hz: gain={gain:.3}");
}

#[test]
fn tb303_output_coupling_cap_load_passes_440hz() {
    // AcidAttack/TB303 output coupling: 100n into 100k load has fc ~= 15.9Hz.
    // At 440Hz it should be close to unity, not -30dB.
    let gain = measure_gain_metered(
        r#"
        pedal "test" { supply 9V
            components {
                C_out: cap(100n)
                R_out: resistor(100k)
            }
            nets {
                in -> C_out.a
                C_out.b -> R_out.a, out
                R_out.b -> gnd
            }
            controls {}
        }"#,
        "TB303 output coupling",
    );
    eprintln!("TB303 output coupling: gain={gain:.3}");
    assert!(
        gain > 0.5,
        "100nF into 100k should pass 440Hz near unity: gain={gain:.3}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 1b: single cap — deeper investigation
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn cap_10u_passes_440hz() {
    // 10µF at 440Hz: Xc = 1/(2π·440·10e-6) ≈ 36Ω. Near unity.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components { C1: cap(10u) }
            nets { in -> C1.a  C1.b -> out }
            controls {}
        }"#,
    );
    eprintln!("Cap 10µF: gain={gain:.3}");
    assert!(gain > 0.5, "10µF cap should pass 440Hz: gain={gain:.3}");
}

#[test]
fn cap_100u_passes_440hz() {
    // 100µF at 440Hz: Xc ≈ 3.6Ω. Essentially a wire.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components { C1: cap(100u) }
            nets { in -> C1.a  C1.b -> out }
            controls {}
        }"#,
    );
    eprintln!("Cap 100µF: gain={gain:.3}");
    assert!(
        gain > 0.8,
        "100µF cap should be near unity at 440Hz: gain={gain:.3}"
    );
}

#[test]
fn cap_1n_with_load_blocks_440hz() {
    // 1nF at 440Hz: Xc ≈ 362kΩ. With 10k load to GND:
    // gain = R_load / (R_load + Xc) ≈ 10k/372k ≈ 0.027. Should block.
    // (A floating cap with open-circuit output doesn't filter — no current path.)
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                C1: cap(1n)
                R_load: resistor(10k)
            }
            nets {
                in -> C1.a
                C1.b -> R_load.a, out
                R_load.b -> gnd
            }
            controls {}
        }"#,
    );
    eprintln!("Cap 1nF + load: gain={gain:.3}");
    assert!(
        gain < 0.1,
        "1nF cap + load should block 440Hz: gain={gain:.3}"
    );
}

#[test]
fn cap_with_load_resistor_passes() {
    // Cap + load R to ground. This gives the WDF tree a proper load.
    // C=1µF, R_load=10k. At 440Hz: Xc=362Ω.
    // Voltage divider: R_load/(R_load + Xc) ≈ 10000/10362 ≈ 0.97
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                C1: cap(1u)
                R_load: resistor(10k)
            }
            nets {
                in -> C1.a
                C1.b -> R_load.a, out
                R_load.b -> gnd
            }
            controls {}
        }"#,
    );
    eprintln!("Cap + load R: gain={gain:.3}");
    assert!(gain > 0.5, "Cap + load should pass 440Hz: gain={gain:.3}");
}

#[test]
fn inductor_passes_440hz() {
    // 100mH at 440Hz: XL = 2π·440·0.1 ≈ 276Ω. Should pass some signal.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components { L1: inductor(100m) }
            nets { in -> L1.a  L1.b -> out }
            controls {}
        }"#,
    );
    eprintln!("Inductor 100mH: gain={gain:.3}");
    assert!(
        gain > 0.3,
        "100mH inductor should pass 440Hz: gain={gain:.3}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 2: two-component networks
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn rc_series_passes_signal() {
    // R + C in series: high-pass filter. At 440Hz with R=1k, C=1µF:
    // Xc = 362Ω, gain ≈ R/(R+Xc) ≈ 0.73
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(1k)
                C1: cap(1u)
            }
            nets {
                in -> C1.a
                C1.b -> R1.a
                R1.b -> out
            }
            controls {}
        }"#,
    );
    eprintln!("RC series (1k + 1µF): gain={gain:.3}");
    assert!(gain > 0.3, "RC series should pass 440Hz: gain={gain:.3}");
}

#[test]
fn resistor_divider_halves_signal() {
    // Two equal resistors: output = input/2
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
    eprintln!("Resistor divider: gain={gain:.3}");
    assert!(
        gain > 0.3 && gain < 0.9,
        "Divider should be ~0.5: gain={gain:.3}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 3: pot as voltage divider
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn pot_divider_passes_signal() {
    // Pot from signal to ground, wiper to output. At 0.5: gain ≈ 0.5.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components { Vol: pot(100k, b) }
            nets {
                in -> Vol.a
                Vol.b -> gnd
                Vol.w -> out
            }
            controls {
                Vol.position -> "Volume" [0.0, 1.0] = 0.5
            }
        }"#,
    );
    eprintln!("Pot at 0.5: gain={gain:.3}");
    assert!(gain > 0.2, "Pot at 0.5 should pass signal: gain={gain:.3}");
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 4: coupling cap + pot (output stage pattern)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn coupling_cap_then_pot_passes_signal() {
    // C_c2 → Level pot → output. This is the Screamer output topology.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                C_c2: cap(1u, electrolytic)
                Level: pot(100k, a)
                C_out: cap(10u, electrolytic)
                R_out: resistor(10k)
            }
            nets {
                in -> C_c2.a
                C_c2.b -> Level.a
                Level.b -> gnd
                Level.w -> C_out.a
                C_out.b -> R_out.a
                R_out.b -> out
            }
            controls {
                Level.position -> "Level" [0.0, 1.0] = 0.7
            }
        }"#,
    );
    eprintln!("Coupling cap + Level pot: gain={gain:.3}");
    assert!(
        gain > 0.1,
        "Coupling + Level should pass signal: gain={gain:.3}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 5: tone control (Screamer-style gyrator)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tone_control_passes_signal() {
    // Screamer tone: R_t1 → Tone pot blends bass (C_t1) and treble (R_t2+C_t2)
    let gain = measure_gain_metered(
        r#"
        pedal "test" { supply 9V
            components {
                R_t1: resistor(1k)
                Tone: pot(20k, b)
                C_t1: cap(220n)
                R_t2: resistor(220)
                C_t2: cap(100n)
            }
            nets {
                in -> R_t1.a
                R_t1.b -> Tone.a, R_t2.a
                Tone.w -> out
                Tone.b -> C_t1.a
                C_t1.b -> gnd
                R_t2.b -> C_t2.a
                C_t2.b -> gnd
            }
            controls {
                Tone.position -> "Tone" [0.0, 1.0] = 0.5
            }
        }"#,
        "Tone control",
    );
    eprintln!("Tone control: gain={gain:.3}");
    assert!(
        gain > 0.05,
        "Tone control should pass signal: gain={gain:.3}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 5b: output probe at non-pot components
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn output_at_resistor_divider_with_cap_shunt() {
    // out is at a resistor junction (R_out_s.b), not a pot wiper.
    // R_out_s in series, R_out_g shunt to GND. Output at their junction.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                C_in: cap(1u)
                R_out_s: resistor(100)
                R_out_g: resistor(10k)
            }
            nets {
                in -> C_in.a
                C_in.b -> R_out_s.a
                R_out_s.b -> R_out_g.a, out
                R_out_g.b -> gnd
            }
            controls {}
        }"#,
    );
    eprintln!("Output at resistor junction: gain={gain:.3}");
    // R_out_g/(R_out_s + R_out_g) ≈ 10k/10.1k ≈ 0.99. Near unity.
    assert!(
        gain > 0.3,
        "Output at R junction should pass signal: gain={gain:.3}"
    );
}

#[test]
fn output_at_coupling_cap_with_ground_shunt() {
    // out is after a coupling cap, with a shunt resistor to GND elsewhere.
    // The ground shunt makes it ShortCircuit root, but out is interior.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                R_series: resistor(1k)
                C_shunt: cap(100n)
                R_out: resistor(10k)
            }
            nets {
                in -> R_series.a
                R_series.b -> C_shunt.a, R_out.a
                C_shunt.b -> gnd
                R_out.b -> out
            }
            controls {}
        }"#,
    );
    eprintln!("Output after R with cap shunt: gain={gain:.3}");
    assert!(
        gain > 0.3,
        "Should pass signal to interior out node: gain={gain:.3}"
    );
}

#[test]
fn output_deep_in_rc_chain() {
    // Three-stage RC chain with out at the end, ground shunts along the way.
    // out is 3 components away from in, with caps to GND at each junction.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(1k)
                C1: cap(100n)
                R2: resistor(1k)
                C2: cap(100n)
                R3: resistor(10k)
            }
            nets {
                in -> R1.a
                R1.b -> C1.a, R2.a
                C1.b -> gnd
                R2.b -> C2.a, R3.a
                C2.b -> gnd
                R3.b -> out
            }
            controls {}
        }"#,
    );
    eprintln!("Output deep in RC chain: gain={gain:.3}");
    // Two-pole RC lowpass at 440Hz. R=1k, C=100n → fc ≈ 1.6kHz.
    // At 440Hz: each pole ≈ -1.3dB → total ≈ -2.6dB ≈ 0.74
    assert!(
        gain > 0.1,
        "Should pass 440Hz through 2-pole RC: gain={gain:.3}"
    );
}

#[test]
fn pot_wiper_output_with_ground_shunt_caps() {
    // Pot wiper as output, but also has caps to GND (tone-like).
    // The pot probe should still work here.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(1k)
                Vol: pot(100k, b)
                C_shunt: cap(220n)
            }
            nets {
                in -> R_in.a
                R_in.b -> Vol.a
                Vol.b -> C_shunt.a
                C_shunt.b -> gnd
                Vol.w -> out
            }
            controls {
                Vol.position -> "Volume" [0.0, 1.0] = 0.5
            }
        }"#,
    );
    eprintln!("Pot wiper + cap shunt: gain={gain:.3}");
    assert!(
        gain > 0.1,
        "Pot wiper output should work with ground caps: gain={gain:.3}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 5c: Screamer input coupling in isolation
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_input_coupling_alone() {
    // Screamer input: Cin(22nF) → R_in(510k) to GND, output at junction.
    // At 440Hz: Xc = 16.4kΩ. R_in = 510kΩ.
    // Gain = R_in/(R_in+Xc) = 510k/526k ≈ 0.97. Near unity.
    let gain = measure_gain_metered(
        r#"
        pedal "test" { supply 9V
            components {
                Cin: cap(22n)
                R_in: resistor(510k)
            }
            nets {
                in -> Cin.a
                Cin.b -> R_in.a, out
                R_in.b -> gnd
            }
            controls {}
        }"#,
        "Screamer input coupling",
    );
    eprintln!("Screamer input coupling: gain={gain:.3}");
    assert!(
        gain > 0.5,
        "22nF + 510k should be near unity at 440Hz: gain={gain:.3}"
    );
}

#[test]
fn two_passive_stages_chain() {
    // Two passive stages in series: input coupling → output coupling.
    // Each is near-unity. Combined should be near-unity.
    // This tests whether the serial chain preserves signal between stages.
    let gain = measure_gain_metered(
        r#"
        pedal "test" { supply 9V
            components {
                Cin: cap(22n)
                R_in: resistor(510k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                R_out: resistor(10k)
            }
            nets {
                in -> Cin.a
                Cin.b -> R_in.a, U1.neg
                R_in.b -> gnd
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> R_out.a
                R_out.b -> out
            }
            controls {}
        }"#,
        "Two stages chained",
    );
    eprintln!("Two stages chained: gain={gain:.3}");
    // Input coupling ≈ 0.97, opamp gain = 10, R_out ≈ 1.0
    // Combined ≈ 9.7
    assert!(gain > 1.0, "Chained stages should amplify: gain={gain:.3}");
}

#[test]
fn sd1_input_coupling_alone() {
    // SD-1 input: Cin(47nF) → R_in(470k) to GND.
    // At 440Hz: Xc = 7.7kΩ. R_in = 470kΩ.
    // Gain ≈ 0.98.
    let gain = measure_gain_metered(
        r#"
        pedal "test" { supply 9V
            components {
                Cin: cap(47n)
                R_in: resistor(470k)
            }
            nets {
                in -> Cin.a
                Cin.b -> R_in.a, out
                R_in.b -> gnd
            }
            controls {}
        }"#,
        "SD-1 input coupling",
    );
    eprintln!("SD-1 input coupling: gain={gain:.3}");
    assert!(
        gain > 0.5,
        "47nF + 470k should be near unity at 440Hz: gain={gain:.3}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 5d: opamp + diode clipping without pendant tree
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn inverting_opamp_diode_gain_correct() {
    // Inverting: R_in=10k, Rf=100k → gain=10. Input=0.1V → 1.0V → diode clips at ~0.5V.
    // This is the simple 4-edge test that already works.
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D_clip: diode_pair(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.out -> D_clip.a
                D_clip.b -> U1.neg
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    );
    eprintln!("Inverting opamp+diode: gain={gain:.3}");
    // Gain=10, clipped at ~0.5V → output ~0.5V from 0.1V input → gain ~5
    assert!(gain > 2.0, "Should amplify + clip: gain={gain:.3}");
}

#[test]
fn inverting_opamp_diode_with_cap_input() {
    // Same but with coupling cap input (the bug case).
    // Cin(1µF) → R_in(10k) → neg → Rf(100k) → out → D_clip → neg
    // Gain should still be ~10 (Ri=R_in=10k, not Cin's impedance).
    let gain = measure_gain(
        r#"
        pedal "test" { supply 9V
            components {
                Cin: cap(1u)
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D_clip: diode_pair(silicon)
            }
            nets {
                in -> Cin.a
                Cin.b -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.out -> D_clip.a
                D_clip.b -> U1.neg
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    );
    eprintln!("Inverting opamp+diode+cap: gain={gain:.3}");
    assert!(gain > 1.5, "Cap input shouldn't kill gain: gain={gain:.3}");
}

#[test]
fn inverting_opamp_with_ground_termination() {
    // R_in to ground + cap input — the Screamer-like inverting topology.
    // Cin → R_in_gnd(510k, to GND) → junction → Rf → out
    // Ri should be the impedance from input to neg.
    // With cap coupling: Ri = R_series if present, or just Xc(Cin).
    let gain = measure_gain_metered(
        r#"
        pedal "test" { supply 9V
            components {
                Cin: cap(1u)
                R_in_gnd: resistor(510k)
                R_series: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D_clip: diode_pair(silicon)
            }
            nets {
                in -> Cin.a
                Cin.b -> R_in_gnd.a, R_series.a
                R_in_gnd.b -> gnd
                R_series.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.out -> D_clip.a
                D_clip.b -> U1.neg
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
        "Inverting with ground termination",
    );
    eprintln!("Inverting + ground term: gain={gain:.3}");
    // Ri = R_series = 10k (not R_in_gnd=510k!)
    // Gain = Rf/Ri = 100k/10k = 10, clipped at ~0.5V → gain ~5
    assert!(
        gain > 1.5,
        "R_series should be Ri, not R_in_gnd: gain={gain:.3}"
    );
}

#[test]
fn noninverting_screamer_clipping_gain() {
    // Non-inverting: Screamer-like topology.
    // Input → C_c1 → pos, R_b1 → vbias
    // neg → R_hp → C_hp → gnd (Ri = R_hp = 4.7k)
    // Feedback: Drive + R_clip ≈ 255k
    // Gain = 1 + 255k/4.7k ≈ 55
    // With diode: clips at ~0.5V → output 0.5V from 0.1V input → gain ~5
    let gain = measure_gain_metered(
        r#"
        pedal "test" { supply 9V
            components {
                C_c1: cap(1u, electrolytic)
                U1: opamp(jrc4558)
                R_hp: resistor(4.7k)
                C_hp: cap(47n)
                Drive: pot(500k, a)
                R_clip: resistor(4.7k)
                D_clip: diode_pair(silicon)
            }
            nets {
                in -> C_c1.a
                C_c1.b -> U1.pos
                U1.pos -> gnd
                U1.neg -> R_hp.a
                R_hp.b -> C_hp.a
                C_hp.b -> gnd
                U1.out -> Drive.a, D_clip.a
                Drive.b -> R_clip.a
                R_clip.b -> U1.neg
                D_clip.b -> U1.neg
                U1.out -> out
            }
            controls {
                Drive.position -> "Drive" [0.0, 1.0] = 0.5
            }
        }"#,
        "Screamer clipping stage",
    );
    eprintln!("Screamer clipping: gain={gain:.3}");
    assert!(gain > 1.5, "Screamer should amplify + clip: gain={gain:.3}");
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 6: full Screamer output chain (tone + level + output coupling)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_output_chain_passes_signal() {
    // The exact topology from the Screamer's tone + output stages,
    // without the op-amp. Pure passive network.
    let gain = measure_gain_metered(
        r#"
        pedal "test" { supply 9V
            components {
                R_t1: resistor(1k)
                Tone: pot(20k, b)
                C_t1: cap(220n)
                R_t2: resistor(220)
                C_t2: cap(100n)
                C_c2: cap(1u, electrolytic)
                Level: pot(100k, a)
                C_out: cap(10u, electrolytic)
                R_out_s: resistor(100)
                R_out_g: resistor(10k)
            }
            nets {
                in -> R_t1.a
                R_t1.b -> Tone.a, R_t2.a
                Tone.w -> C_c2.a
                Tone.b -> C_t1.a
                C_t1.b -> gnd
                R_t2.b -> C_t2.a
                C_t2.b -> gnd
                C_c2.b -> Level.a
                Level.b -> gnd
                Level.w -> C_out.a
                C_out.b -> R_out_s.a
                R_out_s.b -> R_out_g.a, out
                R_out_g.b -> gnd
            }
            controls {
                Tone.position -> "Tone" [0.0, 1.0] = 0.5
                Level.position -> "Level" [0.0, 1.0] = 0.7
            }
        }"#,
        "Screamer output chain",
    );
    assert!(
        gain > 0.01,
        "Screamer output chain should pass signal: gain={gain:.3}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 7: mid-chain passive stages (not at global out_node)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn mid_chain_passive_stage_passes_signal() {
    // Three groups: input → opamp → passive_filter → output_resistor.
    // The passive_filter is mid-chain: it doesn't touch graph.out_node.
    // Its output boundary is the node shared with the next group.
    let gain = measure_gain_metered(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                C_tone: cap(3.3n)
                R_filter: resistor(10k)
                C_out: cap(1u)
                R_out: resistor(10k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> C_tone.a, R_filter.a
                C_tone.b -> gnd
                R_filter.b -> C_out.a
                C_out.b -> R_out.a
                R_out.b -> out
            }
            controls {}
        }"#,
        "Mid-chain passive",
    );
    eprintln!("Mid-chain passive: gain={gain:.3}");
    assert!(
        gain > 0.1,
        "Mid-chain passive should pass signal: gain={gain:.3}"
    );
}

#[test]
fn ratking_tone_volume_stage_not_dead() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/ratking.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let mut compiled = super::spqr_build::compile_via_spqr(&pedal, SR).expect("compile");
    compiled.enable_metering(128);

    let amp = 0.1;
    for s in 0..8000 {
        compiled.process(amp * (std::f64::consts::TAU * 440.0 * s as f64 / SR).sin());
    }

    let metrics = compiled.read_metrics();
    let n = compiled.stages.len().min(crate::metering::MAX_STAGES);
    for i in 0..n {
        let lvl = metrics.stage_levels[i];
        let db = if lvl > 1e-10 {
            20.0 * (lvl as f64).log10()
        } else {
            -120.0
        };
        #[cfg(debug_assertions)]
        {
            let (lbl, bypass) = match &compiled.stages[i] {
                super::compiled::Stage::Wdf(w) => (w.debug_label.as_str(), w.bypass_serial),
                super::compiled::Stage::MultiNl(m) => (m.debug_label.as_str(), m.bypass_serial),
                super::compiled::Stage::Iir(s) => (s.debug_label.as_str(), s.bypass_serial),
                super::compiled::Stage::StateSpace(s) => (s.debug_label.as_str(), s.bypass_serial),
                super::compiled::Stage::BlackFeedback(b) => {
                    (b.debug_label.as_str(), b.bypass_serial)
                }
                super::compiled::Stage::BlockwiseKMethod(bk) => ("blockwise", bk.bypass_serial),
            };
            if bypass {
                continue;
            }
            // The tone+volume stage should not be dead
            if lbl.contains("C_tone") || lbl.contains("Filter") || lbl.contains("Volume") {
                eprintln!("Ratking tone stage [{lbl}]: {db:.1} dB");
                assert!(
                    db > -60.0,
                    "Ratking tone+volume stage [{lbl}] is dead at {db:.1} dB"
                );
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 8: WDF output extraction for complex multi-edge passive groups
//
// Bug 3: short_circuit_junction_voltage returns None for trees with 5+
// edges and multiple SPQR terminals. The signal enters the tree via VS
// but the output extraction can't find the correct junction voltage.
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn five_edge_passive_group_passes_signal() {
    // Simplified Goldenrod Gain_B topology: 5 passive edges, ground shunt.
    // R1 → C1 → junction → R2 → out, with C_shunt → gnd at junction.
    // This is the simplest multi-edge group that should work but might not.
    let gain = measure_gain_metered(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                C1: cap(100n)
                C_shunt: cap(10n)
                R2: resistor(10k)
                R_out: resistor(10k)
            }
            nets {
                in -> R1.a
                R1.b -> C1.a
                C1.b -> C_shunt.a, R2.a
                C_shunt.b -> gnd
                R2.b -> R_out.a
                R_out.b -> out
            }
            controls {}
        }"#,
        "5-edge passive",
    );
    eprintln!("5-edge passive: gain={gain:.3}");
    assert!(
        gain > 0.1,
        "5-edge passive group should pass signal: gain={gain:.3}"
    );
}

#[test]
fn six_edge_with_pot_passes_signal() {
    // Closer to Goldenrod: R + C + pot(2 edges) + R + ground shunt.
    let gain = measure_gain_metered(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                C1: cap(100n)
                Vol: pot(100k, b)
                C_shunt: cap(10n)
                R_out: resistor(10k)
            }
            nets {
                in -> R1.a
                R1.b -> C1.a
                C1.b -> Vol.a, C_shunt.a
                Vol.b -> gnd
                Vol.w -> R_out.a
                C_shunt.b -> gnd
                R_out.b -> out
            }
            controls {
                Vol.position -> "Volume" [0.0, 1.0] = 0.5
            }
        }"#,
        "6-edge with pot",
    );
    eprintln!("6-edge with pot: gain={gain:.3}");
    assert!(
        gain > 0.05,
        "6-edge passive with pot should pass signal: gain={gain:.3}"
    );
}

#[test]
fn seven_edge_feedforward_group_passes_signal() {
    // Exact Goldenrod Gain_B topology: R1, C1, R_bias, R_ff, C_ff, Pot(2).
    // All passive, connected between two active stages.
    let gain = measure_gain_metered(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                C1: cap(100n)
                R_bias: resistor(1M)
                R_ff: resistor(15k)
                C_ff: cap(100n)
                Blend: pot(100k, b)
                R_out: resistor(10k)
            }
            nets {
                in -> R1.a
                R1.b -> C1.a
                C1.b -> R_bias.a, R_ff.a, Blend.a
                R_bias.b -> gnd
                R_ff.b -> C_ff.a
                C_ff.b -> R_out.a
                Blend.b -> gnd
                Blend.w -> R_out.a
                R_out.b -> out
            }
            controls {
                Blend.position -> "Blend" [0.0, 1.0] = 0.5
            }
        }"#,
        "7-edge feedforward",
    );
    eprintln!("7-edge feedforward: gain={gain:.3}");
    assert!(
        gain > 0.01,
        "7-edge feedforward group should pass signal: gain={gain:.3}"
    );
}

#[test]
fn goldenrod_gain_b_stage_not_dead() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/goldenrod.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let mut compiled = super::spqr_build::compile_via_spqr(&pedal, SR).expect("compile");
    compiled.enable_metering(128);

    let amp = 0.1;
    for s in 0..8000 {
        compiled.process(amp * (std::f64::consts::TAU * 440.0 * s as f64 / SR).sin());
    }

    let metrics = compiled.read_metrics();
    let n = compiled.stages.len().min(crate::metering::MAX_STAGES);
    for i in 0..n {
        let lvl = metrics.stage_levels[i];
        let db = if lvl > 1e-10 {
            20.0 * (lvl as f64).log10()
        } else {
            -120.0
        };
        #[cfg(debug_assertions)]
        {
            let (lbl, bypass) = match &compiled.stages[i] {
                super::compiled::Stage::Wdf(w) => (w.debug_label.as_str(), w.bypass_serial),
                super::compiled::Stage::MultiNl(m) => (m.debug_label.as_str(), m.bypass_serial),
                super::compiled::Stage::Iir(s) => (s.debug_label.as_str(), s.bypass_serial),
                super::compiled::Stage::StateSpace(s) => (s.debug_label.as_str(), s.bypass_serial),
                super::compiled::Stage::BlackFeedback(b) => {
                    (b.debug_label.as_str(), b.bypass_serial)
                }
                super::compiled::Stage::BlockwiseKMethod(bk) => ("blockwise", bk.bypass_serial),
            };
            if bypass {
                continue;
            }
            if lbl.contains("Gain_B") {
                eprintln!("Goldenrod Gain_B stage [{lbl}]: {db:.1} dB");
                assert!(
                    db > -60.0,
                    "Goldenrod Gain_B stage [{lbl}] is dead at {db:.1} dB"
                );
            }
        }
    }
}
