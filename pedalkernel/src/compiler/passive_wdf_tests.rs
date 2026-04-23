// ═══════════════════════════════════════════════════════════════════════════
// Passive WDF tree tests
//
// TDD: build up from the simplest passive network to the Screamer's tone
// stage. Find exactly where the WDF tree breaks for multi-component
// passive groups.
//
// No active components. If any of these fail, the WDF itself is broken.
// ═══════════════════════════════════════════════════════════════════════════

use super::spqr_build::compile_via_spqr;
use super::compiled::CompiledPedal;
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
        let out = compiled.process(
            amp * (std::f64::consts::TAU * FREQ * (4000 + s) as f64 / SR).sin()
        );
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
        let out = compiled.process(
            amp * (std::f64::consts::TAU * FREQ * (4000 + s) as f64 / SR).sin()
        );
        peak = peak.max(out.abs());
    }
    let gain = peak / amp;

    // Dump per-stage metering
    let metrics = compiled.read_metrics();
    let n = compiled.stages.len().min(crate::metering::MAX_STAGES);
    eprintln!("  {label}: gain={gain:.3} ({n} stages)");
    for i in 0..n {
        let lvl = metrics.stage_levels[i];
        let db = if lvl > 1e-10 { 20.0 * (lvl as f64).log10() } else { -120.0 };
        #[cfg(debug_assertions)]
        {
            let (stype, lbl, bypass) = match &compiled.stages[i] {
                super::compiled::Stage::Wdf(w) => ("Wdf", &w.debug_label, w.bypass_serial),
                super::compiled::Stage::MultiNl(m) => ("MNL", &m.debug_label, m.bypass_serial),
                super::compiled::Stage::Iir(s) => ("Iir", &s.debug_label, s.bypass_serial),
                super::compiled::Stage::StateSpace(s) => ("SS", &s.debug_label, s.bypass_serial),
                super::compiled::Stage::BlackFeedback(b) => ("BF", &b.debug_label, b.bypass_serial),
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
    let gain = measure_gain(r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k) }
            nets { in -> R1.a  R1.b -> out }
            controls {}
        }"#);
    eprintln!("Single resistor: gain={gain:.3}");
    // Should be ~1.0 (unity, maybe with 1.5× oversampler artifact)
    assert!(gain > 0.5, "Resistor should pass signal: gain={gain:.3}");
}

#[test]
fn coupling_cap_passes_440hz() {
    // 1µF at 440Hz: Xc ≈ 362Ω. Near unity for audio.
    let gain = measure_gain(r#"
        pedal "test" { supply 9V
            components { C1: cap(1u) }
            nets { in -> C1.a  C1.b -> out }
            controls {}
        }"#);
    eprintln!("Coupling cap 1µF: gain={gain:.3}");
    assert!(gain > 0.5, "1µF cap should pass 440Hz: gain={gain:.3}");
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 2: two-component networks
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn rc_series_passes_signal() {
    // R + C in series: high-pass filter. At 440Hz with R=1k, C=1µF:
    // Xc = 362Ω, gain ≈ R/(R+Xc) ≈ 0.73
    let gain = measure_gain(r#"
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
        }"#);
    eprintln!("RC series (1k + 1µF): gain={gain:.3}");
    assert!(gain > 0.3, "RC series should pass 440Hz: gain={gain:.3}");
}

#[test]
fn resistor_divider_halves_signal() {
    // Two equal resistors: output = input/2
    let gain = measure_gain(r#"
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
        }"#);
    eprintln!("Resistor divider: gain={gain:.3}");
    assert!(gain > 0.3 && gain < 0.9, "Divider should be ~0.5: gain={gain:.3}");
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 3: pot as voltage divider
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn pot_divider_passes_signal() {
    // Pot from signal to ground, wiper to output. At 0.5: gain ≈ 0.5.
    let gain = measure_gain(r#"
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
        }"#);
    eprintln!("Pot at 0.5: gain={gain:.3}");
    assert!(gain > 0.2, "Pot at 0.5 should pass signal: gain={gain:.3}");
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 4: coupling cap + pot (output stage pattern)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn coupling_cap_then_pot_passes_signal() {
    // C_c2 → Level pot → output. This is the Screamer output topology.
    let gain = measure_gain(r#"
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
        }"#);
    eprintln!("Coupling cap + Level pot: gain={gain:.3}");
    assert!(gain > 0.1, "Coupling + Level should pass signal: gain={gain:.3}");
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 5: tone control (Screamer-style gyrator)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tone_control_passes_signal() {
    // Screamer tone: R_t1 → Tone pot blends bass (C_t1) and treble (R_t2+C_t2)
    let gain = measure_gain(r#"
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
        }"#);
    eprintln!("Tone control: gain={gain:.3}");
    assert!(gain > 0.05, "Tone control should pass signal: gain={gain:.3}");
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 6: full Screamer output chain (tone + level + output coupling)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_output_chain_passes_signal() {
    // The exact topology from the Screamer's tone + output stages,
    // without the op-amp. Pure passive network.
    let gain = measure_gain_metered(r#"
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
        }"#, "Screamer output chain");
    assert!(gain > 0.01, "Screamer output chain should pass signal: gain={gain:.3}");
}
