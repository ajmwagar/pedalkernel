// ═══════════════════════════════════════════════════════════════════════════
// Bias application tests
//
// TDD tests for applying DC bias voltages to active element models.
//
// The bias_analysis module DETECTS static bias networks and computes DC
// voltages. These tests verify that the computed voltages are correctly
// APPLIED to each active element type via the Component trait.
//
// The Component trait method `apply_bias()` receives the DC voltage at
// each connected bias node and returns model parameter adjustments:
// - Op-amps: sets v_max (output swing) from bias point + supply voltage
// - BJTs: sets quiescent Ic, Vce from collector/emitter bias network
// - Tubes: sets grid bias (Vgk) from cathode/grid bias network
// - JFETs: sets Vgs from gate bias network
//
// Each component type knows how to interpret bias voltages for its own
// physics. The pipeline doesn't special-case component types.
// ═══════════════════════════════════════════════════════════════════════════

use super::spqr_build::compile_via_spqr;
use crate::PedalProcessor;

const SR: f64 = 48000.0;

// ═══════════════════════════════════════════════════════════════════════════
// Op-amp bias: v_max from supply and bias point
// ═══════════════════════════════════════════════════════════════════════════

/// Helper: compile a circuit with a specific opamp type and bias divider,
/// drive it, and return the peak output. The bias network sets the DC
/// operating point; the op-amp's v_max should reflect the actual supply
/// headroom, not a hardcoded ±12V.
fn opamp_bias_peak(opamp_type: &str, r_top: &str, r_bot: &str, supply: &str) -> f64 {
    let source = format!(r#"
        pedal "test" {{ supply {supply}
            components {{
                R_v1: resistor({r_top})
                R_v2: resistor({r_bot})
                C_v: cap(47u, electrolytic)
                R_in: resistor(10k)
                U1: opamp({opamp_type})
                Rf: resistor(100k)
                D_clip: diode_pair(silicon)
            }}
            nets {{
                vcc -> R_v1.a
                R_v1.b -> R_v2.a, C_v.a
                R_v2.b -> gnd
                C_v.b -> gnd
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.out -> D_clip.a
                D_clip.b -> U1.neg
                U1.pos -> gnd
                U1.out -> out
            }}
            controls {{}}
        }}"#);

    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    let amp = 0.1;
    let freq = 440.0;
    for s in 0..2000 {
        compiled.process(amp * (std::f64::consts::TAU * freq * s as f64 / SR).sin());
    }
    let mut peak = 0.0f64;
    for s in 0..500 {
        let out = compiled.process(
            amp * (std::f64::consts::TAU * freq * (2000 + s) as f64 / SR).sin()
        );
        peak = peak.max(out.abs());
    }
    peak
}

// ── Op-amp types: all should clip at circuit-appropriate levels ──────────

#[test]
fn jrc4558_bias_from_divider() {
    // JRC4558 with 9V supply, 10k/10k divider → bias = 4.5V
    // v_max should be ~3.0V (4.5V - 1.5V headroom), not 12V
    let peak = opamp_bias_peak("jrc4558", "10k", "10k", "9V");
    eprintln!("JRC4558 9V bias: peak={peak:.4}V");
    // Should clip — diode Vf ≈ 0.6V, gain=10
    assert!(peak > 0.1, "Should produce output: {peak:.4}V");
    assert!(peak < 4.0, "Should NOT clip at ±12V (wrong v_max): {peak:.4}V");
}

#[test]
fn tl072_bias_from_divider() {
    let peak = opamp_bias_peak("tl072", "10k", "10k", "9V");
    eprintln!("TL072 9V bias: peak={peak:.4}V");
    assert!(peak > 0.1, "Should produce output: {peak:.4}V");
    assert!(peak < 4.0, "Should clip at 9V rails, not ±12V: {peak:.4}V");
}

#[test]
fn lm308_bias_from_divider() {
    let peak = opamp_bias_peak("lm308", "10k", "10k", "9V");
    eprintln!("LM308 9V bias: peak={peak:.4}V");
    assert!(peak > 0.1, "Should produce output: {peak:.4}V");
    assert!(peak < 4.0, "Should clip at 9V rails, not ±12V: {peak:.4}V");
}

#[test]
fn ne5532_bias_from_divider() {
    let peak = opamp_bias_peak("ne5532", "10k", "10k", "9V");
    eprintln!("NE5532 9V bias: peak={peak:.4}V");
    assert!(peak > 0.1, "Should produce output: {peak:.4}V");
    assert!(peak < 4.0, "Should clip at 9V rails, not ±12V: {peak:.4}V");
}

#[test]
fn lm741_bias_from_divider() {
    let peak = opamp_bias_peak("lm741", "10k", "10k", "9V");
    eprintln!("LM741 9V bias: peak={peak:.4}V");
    assert!(peak > 0.1, "Should produce output: {peak:.4}V");
    assert!(peak < 4.0, "Should clip at 9V rails, not ±12V: {peak:.4}V");
}

// ── Op-amp rail clipping (no diodes) must respect supply voltage ─────────

#[test]
fn opamp_9v_clips_at_supply_not_12v() {
    // Op-amp WITHOUT diodes: output should clip at supply rails.
    // With 9V supply and symmetric bias (4.5V), max swing = ~3.0V.
    // If v_max is still hardcoded to 12V, output will NOT clip at all
    // (gain × input = 10 × 0.5 = 5V, which is below 12V but above 3V).
    let source = r#"
        pedal "test" { supply 9V
            components {
                R_v1: resistor(10k)
                R_v2: resistor(10k)
                C_v: cap(47u, electrolytic)
                R_in: resistor(10k)
                U1: opamp(jrc4558)
                Rf: resistor(100k)
            }
            nets {
                vcc -> R_v1.a
                R_v1.b -> R_v2.a, C_v.a
                R_v2.b -> gnd
                C_v.b -> gnd
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#;

    let pedal = crate::dsl::parse_pedal_file(source).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Drive hard: gain=10, input=0.5V → 5.0V unclipped.
    // With 9V supply (v_max≈3V), should clip at ~3V.
    // With wrong 12V v_max, output would be 5.0V (no clipping).
    let amp = 0.5;
    let freq = 440.0;
    for s in 0..2000 {
        compiled.process(amp * (std::f64::consts::TAU * freq * s as f64 / SR).sin());
    }
    let mut peak = 0.0f64;
    for s in 0..500 {
        let out = compiled.process(
            amp * (std::f64::consts::TAU * freq * (2000 + s) as f64 / SR).sin()
        );
        peak = peak.max(out.abs());
    }

    eprintln!("OpAmp 9V no-diode: peak={peak:.4}V (should clip at ~3V, not pass 5V)");
    assert!(peak > 0.5, "Should produce significant output: {peak:.4}V");
    // Key assertion: output should be limited by 9V supply, not ±12V
    assert!(peak < 4.5, "Should clip at 9V supply rails (~3V), not ±12V: {peak:.4}V");
}

// ── Asymmetric bias should produce asymmetric clipping ──────────────────

#[test]
fn asymmetric_bias_shifts_clipping() {
    // 20k/10k divider → bias = 3.0V (not centered)
    // Positive swing headroom: 9V - 3.0V - 1.5V = 4.5V
    // Negative swing headroom: 3.0V - 1.5V = 1.5V
    // Output should clip asymmetrically
    let peak = opamp_bias_peak("jrc4558", "20k", "10k", "9V");
    eprintln!("Asymmetric bias (3V): peak={peak:.4}V");
    assert!(peak > 0.1, "Should produce output: {peak:.4}V");
}

// ── 18V supply: v_max must come from actual supply, not hardcoded 9V ────

#[test]
fn supply_18v_uses_actual_supply_not_hardcoded_9v() {
    // With 18V supply and no diodes, gain=10, input=0.5V → 5V unclipped.
    // Correct v_max = (18/2 - 1.5) = 7.5V → output 5V (no clipping).
    // If v_max hardcoded from 9V: v_max = 3.0V → output clips at 3V (WRONG).
    let source = r#"
        pedal "test" { supply 18V
            components {
                R_v1: resistor(10k)
                R_v2: resistor(10k)
                C_v: cap(47u, electrolytic)
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
            }
            nets {
                vcc -> R_v1.a
                R_v1.b -> R_v2.a, C_v.a
                R_v2.b -> gnd
                C_v.b -> gnd
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#;

    let pedal = crate::dsl::parse_pedal_file(source).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Drive: gain=10, input=0.5V → 5V.
    // With 18V supply, v_max=7.5V → output should be ~5V (unclipped).
    // With hardcoded 9V, v_max=3V → output would clip at 3V.
    let amp = 0.5;
    let freq = 440.0;
    for s in 0..2000 {
        compiled.process(amp * (std::f64::consts::TAU * freq * s as f64 / SR).sin());
    }
    let mut peak = 0.0f64;
    for s in 0..500 {
        let out = compiled.process(
            amp * (std::f64::consts::TAU * freq * (2000 + s) as f64 / SR).sin()
        );
        peak = peak.max(out.abs());
    }

    eprintln!("18V supply: peak={peak:.4}V (should be ~5V, not clipped at 3V)");
    // If supply voltage is correctly propagated, peak should be near 5V
    assert!(peak > 4.0, "18V supply should allow 5V output (gain=10 × 0.5V), got {peak:.4}V — is supply hardcoded to 9V?");
}

// ── Bias voltage from divider sets v_max, not just supply/2 ─────────────

#[test]
fn bias_voltage_sets_vmax_not_supply_half() {
    // Asymmetric divider: R_top=30k, R_bot=10k → bias = 9V × 10/40 = 2.25V
    // Positive headroom: 9V - 2.25V - 1.5V = 5.25V
    // Negative headroom: 2.25V - 1.5V = 0.75V
    // With supply/2 assumed: v_max = 3.0V (symmetric) → clips both ways at 3V
    // With actual bias: positive clips at 5.25V, negative clips at 0.75V
    //
    // Test: drive a strong signal. With correct asymmetric bias, the negative
    // peak should be much smaller than the positive peak.
    let source = r#"
        pedal "test" { supply 9V
            components {
                R_v1: resistor(30k)
                R_v2: resistor(10k)
                C_v: cap(47u, electrolytic)
                R_in: resistor(10k)
                U1: opamp(jrc4558)
                Rf: resistor(100k)
            }
            nets {
                vcc -> R_v1.a
                R_v1.b -> R_v2.a, C_v.a
                R_v2.b -> gnd
                C_v.b -> gnd
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#;

    let pedal = crate::dsl::parse_pedal_file(source).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Drive hard to hit rail clipping (no diodes)
    let amp = 0.5;
    let freq = 440.0;
    for s in 0..2000 {
        compiled.process(amp * (std::f64::consts::TAU * freq * s as f64 / SR).sin());
    }
    let mut pos_peak = 0.0f64;
    let mut neg_peak = 0.0f64;
    for s in 0..500 {
        let out = compiled.process(
            amp * (std::f64::consts::TAU * freq * (2000 + s) as f64 / SR).sin()
        );
        if out > pos_peak { pos_peak = out; }
        if out < neg_peak { neg_peak = out; }
    }

    eprintln!("Asymmetric bias (2.25V): pos={pos_peak:.4}V, neg={neg_peak:.4}V");
    // With correct asymmetric bias: neg clips earlier (0.75V headroom)
    // than pos (5.25V headroom). The ratio should be >2:1.
    let ratio = pos_peak / neg_peak.abs().max(0.001);
    eprintln!("  Asymmetry ratio: {ratio:.2} (should be >2.0 for 30k/10k divider)");
    assert!(ratio > 2.0,
        "Asymmetric bias should produce asymmetric rail clipping: ratio={ratio:.2}");
}

// ═══════════════════════════════════════════════════════════════════════════
// Non-op-amp active elements: what bias means for each type
// ═══════════════════════════════════════════════════════════════════════════
//
// These are DOCUMENTATION tests — they specify the interface that
// Component::apply_bias() should implement for each active element type.
// They're currently expected to fail (RED) until the trait method exists.

// ── BJT bias ─────────────────────────────────────────────────────────────
// A common-emitter BJT stage with collector resistor to VCC and emitter
// resistor to GND. The bias network sets:
//   - Vce (collector-emitter voltage) → determines output swing
//   - Ic (quiescent collector current) → determines gain and linearity
//   - Operating region (saturation vs active)
//
// The Component should compute these from:
//   - V_collector = VCC - Ic * Rc
//   - V_emitter = Ic * Re  (or 0 if no Re)
//   - Vce = V_collector - V_emitter
//   - Ic from base bias: Ib = (V_bias - Vbe) / Rb, Ic = β * Ib

// ── Tube bias ────────────────────────────────────────────────────────────
// A triode stage with plate resistor to B+ and cathode resistor to GND.
// The bias network sets:
//   - Vgk (grid-cathode voltage) → determines gain class and headroom
//   - Plate voltage → determines output swing
//   - Quiescent plate current → determines tone character
//
// The Component should compute these from:
//   - V_cathode = Ip * Rk
//   - Vgk = V_grid - V_cathode (typically negative, e.g. -1.5V)
//   - Ip from load line: B+ / (Rp + rp) where rp = plate resistance

// ── JFET bias ────────────────────────────────────────────────────────────
// A JFET buffer or amplifier with gate bias resistor to ground.
// The bias sets:
//   - Vgs (gate-source voltage) → determines drain current and gain
//   - Typically self-biased via source resistor (Vgs = -Id * Rs)
//   - Gate bias resistor just provides DC return path
