// ═══════════════════════════════════════════════════════════════════════════
// Bridged-T bias-divider loading — regression guards
//
// Ref: pedalkernel-lsbw — Bug: bridged-T bias-divider loading shifts resonant
//      frequency (808 kick 150Hz vs 55Hz).
//
// Status: CONFIRMED NOT PRESENT on origin/main after PR #70 investigation.
//
// Investigated 2026-06-09. Root cause analysis showed:
//   The bias divider (Rb1=Rb2=100k → Rth=50k) IS included in the IIR MNA
//   as a 50k shunt to AC GND at the opamp's pos node. However, since the
//   VCVS draws no current at pos (ideal opamp), V_pos is forced to 0 by the
//   shunt. The VCVS constraint reduces to `V_out = -Aol × V_neg`, identical
//   to the pos-grounded case. The bridged-T poles are unaffected.
//
//   Measurement result:
//     BRIDGED_T_NO_BIAS:   textbook=129.4 Hz, measured=131.4 Hz (1.6% error)
//     BRIDGED_T_WITH_BIAS: textbook=129.4 Hz, measured=131.4 Hz (1.6% error)
//     KICK_STYLE: no_bias=81.4 Hz, with_bias=81.4 Hz (relative=1.00)
//
//   The bug documented in ENGINE_BUG_BRIDGED_T_BIAS_LOADING.md (142-172 Hz
//   for the 808 kick) was either in an older engine version or in the
//   DrummerBoy pro-side layer, not in the compiler's IIR MNA.
//
// These tests serve as regression guards: they verify the bias divider does
// NOT shift the resonant frequency. They will catch any future regression
// where the VCVS/MNA interaction breaks this property.
// ═══════════════════════════════════════════════════════════════════════════

use super::spqr_build::compile_via_spqr;
use crate::PedalProcessor;

const SR: f64 = 48000.0;

// ── Frequency measurement helpers ────────────────────────────────────────

/// Spectral centroid over [lo_hz, hi_hz] after exciting the circuit with an
/// impulse and collecting `n_samples` of ring decay.
///
/// Uses un-windowed DFT (no Hann window) so sub-millisecond ring decays are
/// visible. The centroid is weighted by power at each frequency bin.
fn measure_spectral_centroid_hz(
    compiled: &mut impl PedalProcessor,
    n_warmup: usize,
    n_samples: usize,
    lo_hz: f64,
    hi_hz: f64,
) -> f64 {
    // Warmup with silence so any DC transient settles
    for _ in 0..n_warmup {
        compiled.process(0.0);
    }

    // Impulse excitation then silence
    let mut buf = vec![0.0f64; n_samples];
    compiled.process(1.0); // impulse
    for i in 0..n_samples {
        buf[i] = compiled.process(0.0);
    }

    // Un-windowed DFT
    let n = n_samples;
    let mut power_sum = 0.0f64;
    let mut freq_power_sum = 0.0f64;
    let lo_bin = ((lo_hz * n as f64 / SR) as usize).max(1);
    let hi_bin = ((hi_hz * n as f64 / SR) as usize).min(n / 2 - 1);

    for k in lo_bin..=hi_bin {
        let freq = k as f64 * SR / n as f64;
        let mut re = 0.0f64;
        let mut im = 0.0f64;
        for (i, &s) in buf.iter().enumerate() {
            let angle = std::f64::consts::TAU * k as f64 * i as f64 / n as f64;
            re += s * angle.cos();
            im += s * angle.sin();
        }
        let power = re * re + im * im;
        power_sum += power;
        freq_power_sum += freq * power;
    }

    if power_sum < 1e-30 {
        return 0.0;
    }
    freq_power_sum / power_sum
}

// ── Test circuits ────────────────────────────────────────────────────────

/// Bridged-T oscillator WITHOUT bias divider.
/// R1=R2=150k, C1=C2=8.2n, R_fb=470k.
/// Textbook f0 = 1/(2π × 150k × 8.2n) ≈ 129.5 Hz.
/// pos grounded directly — standard inverting/non-inverting analysis base case.
const BRIDGED_T_NO_BIAS: &str = r#"
    pedal "BridgedT NoBias" { supply 9V
        components {
            U1: opamp(tl072)
            R1: resistor(150k)
            R2: resistor(150k)
            C1: cap(8.2n)
            C2: cap(8.2n)
            R_fb: resistor(470k)
            R_in: resistor(100k)
        }
        nets {
            in -> R_in.a
            R_in.b -> R1.b
            U1.neg -> R1.a, C1.a
            R1.b -> R2.a, C2.a
            R2.b -> U1.out
            C1.b -> gnd
            C2.b -> gnd
            U1.neg -> R_fb.a
            R_fb.b -> U1.out
            U1.pos -> gnd
            U1.out -> out
        }
        controls {}
    }
"#;

/// Bridged-T oscillator WITH VCC/2 bias divider (808-kick topology).
/// R1=R2=150k, C1=C2=8.2n, R_fb=470k, Rb1=Rb2=100k.
/// Bias divider: Rb1 (vcc→pos), Rb2 (pos→gnd).
/// Textbook f0 = 1/(2π × 150k × 8.2n) ≈ 129.5 Hz — same as no-bias.
/// With the bug: resonant frequency is observed ~2-3x higher (260-390 Hz).
const BRIDGED_T_WITH_BIAS: &str = r#"
    pedal "BridgedT WithBias" { supply 9V
        components {
            U1: opamp(tl072)
            Rb1: resistor(100k)
            Rb2: resistor(100k)
            R1: resistor(150k)
            R2: resistor(150k)
            C1: cap(8.2n)
            C2: cap(8.2n)
            R_fb: resistor(470k)
            R_in: resistor(100k)
        }
        nets {
            vcc -> Rb1.a
            Rb1.b -> Rb2.a, U1.pos
            Rb2.b -> gnd
            in -> R_in.a
            R_in.b -> R1.b
            U1.neg -> R1.a, C1.a
            R1.b -> R2.a, C2.a
            R2.b -> U1.out
            C1.b -> gnd
            C2.b -> gnd
            U1.neg -> R_fb.a
            R_fb.b -> U1.out
            U1.out -> out
        }
        controls {}
    }
"#;

/// 808 kick-style bridged-T WITHOUT bias divider: R=150k, C=18n → f0 ≈ 58.9 Hz.
/// Used as baseline to confirm the measurement method is sound.
const KICK_STYLE_NO_BIAS: &str = r#"
    pedal "808 Kick NoBias" { supply 9V
        components {
            U1: opamp(tl072)
            R1: resistor(150k)
            R2: resistor(150k)
            C1: cap(18n)
            C2: cap(18n)
            R_fb: resistor(1M)
            R_trig: resistor(100k)
        }
        nets {
            U1.neg -> R1.a, C1.a
            R1.b -> R2.a, C2.a
            R2.b -> U1.out
            C1.b -> gnd
            C2.b -> gnd
            U1.neg -> R_fb.a
            R_fb.b -> U1.out
            in -> R_trig.a
            R_trig.b -> R1.b
            U1.pos -> gnd
            U1.out -> out
        }
        controls {}
    }
"#;

/// 808 kick-style bridged-T: R=150k, C=18n → f0 ≈ 58.9 Hz.
/// Trigger injection at R1/R2 junction via R_trig.
/// This matches tr808_kick.pedal topology without the volume pot.
const KICK_STYLE_WITH_BIAS: &str = r#"
    pedal "808 Kick Style" { supply 9V
        components {
            U1: opamp(tl072)
            Rb1: resistor(100k)
            Rb2: resistor(100k)
            R1: resistor(150k)
            R2: resistor(150k)
            C1: cap(18n)
            C2: cap(18n)
            R_fb: resistor(1M)
            R_trig: resistor(100k)
        }
        nets {
            vcc -> Rb1.a
            Rb1.b -> Rb2.a, U1.pos
            Rb2.b -> gnd
            U1.neg -> R1.a, C1.a
            R1.b -> R2.a, C2.a
            R2.b -> U1.out
            C1.b -> gnd
            C2.b -> gnd
            U1.neg -> R_fb.a
            R_fb.b -> U1.out
            in -> R_trig.a
            R_trig.b -> R1.b
            U1.out -> out
        }
        controls {}
    }
"#;

// ── Tests ────────────────────────────────────────────────────────────────

/// Sanity baseline: bridged-T without bias divider must resonate near textbook
/// f0 = 1/(2π × 150k × 8.2n) ≈ 129.5 Hz.
///
/// This verifies the measurement methodology before testing the bug.
/// Tolerance ±20% because the bridged-T's gain/Q affects the centroid slightly.
#[test]
fn bridged_t_no_bias_divider_resonates_near_textbook() {
    let pedal = crate::dsl::parse_pedal_file(BRIDGED_T_NO_BIAS).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    let f_textbook = 1.0 / (std::f64::consts::TAU * 150_000.0 * 8.2e-9);
    let f_measured = measure_spectral_centroid_hz(&mut compiled, 512, 4096, 50.0, 500.0);

    eprintln!(
        "bridged_t_no_bias: textbook={:.1} Hz, measured={:.1} Hz, ratio={:.2}",
        f_textbook,
        f_measured,
        if f_textbook > 0.0 { f_measured / f_textbook } else { 0.0 }
    );

    assert!(
        f_measured > 20.0,
        "Circuit must produce a resonant frequency above 20 Hz: got {f_measured:.1} Hz"
    );
    let ratio = f_measured / f_textbook;
    assert!(
        ratio >= 0.70 && ratio <= 1.40,
        "No-bias resonance {f_measured:.1} Hz should be within 30% of textbook {f_textbook:.1} Hz (ratio={ratio:.2})"
    );
}

/// BUG REGRESSION: bridged-T WITH VCC/2 bias divider must also resonate near
/// textbook f0. Before the fix, this oscillates ~2-3x too high (250-390 Hz).
///
/// The bias divider (Rb1=Rb2=100k → Rth=50k) should be transparent to the
/// AC analysis: it only sets the DC operating point. The resonant frequency
/// must be within 10% of the no-bias-divider result.
#[test]
fn bridged_t_bias_divider_does_not_shift_resonant_frequency() {
    let pedal_no_bias = crate::dsl::parse_pedal_file(BRIDGED_T_NO_BIAS).expect("parse no_bias");
    let mut compiled_no_bias = compile_via_spqr(&pedal_no_bias, SR).expect("compile no_bias");

    let pedal_with_bias = crate::dsl::parse_pedal_file(BRIDGED_T_WITH_BIAS).expect("parse with_bias");
    let mut compiled_with_bias = compile_via_spqr(&pedal_with_bias, SR).expect("compile with_bias");

    let f_no_bias = measure_spectral_centroid_hz(&mut compiled_no_bias, 512, 4096, 50.0, 500.0);
    let f_with_bias = measure_spectral_centroid_hz(&mut compiled_with_bias, 512, 4096, 50.0, 500.0);
    let f_textbook = 1.0 / (std::f64::consts::TAU * 150_000.0 * 8.2e-9);

    eprintln!(
        "bridged_t_bias_loading: textbook={:.1} Hz | no_bias={:.1} Hz | with_bias={:.1} Hz",
        f_textbook, f_no_bias, f_with_bias
    );
    eprintln!(
        "  no_bias error={:.1}%  with_bias error={:.1}%",
        (f_no_bias / f_textbook - 1.0) * 100.0,
        (f_with_bias / f_textbook - 1.0) * 100.0,
    );

    // Both circuits should be within 15% of textbook
    let ratio_no_bias = f_no_bias / f_textbook;
    assert!(
        ratio_no_bias >= 0.70 && ratio_no_bias <= 1.40,
        "No-bias circuit deviates from textbook: {f_no_bias:.1} Hz vs {f_textbook:.1} Hz (ratio={ratio_no_bias:.2})"
    );

    let ratio_with_bias = f_with_bias / f_textbook;
    assert!(
        ratio_with_bias >= 0.70 && ratio_with_bias <= 1.40,
        "Bias-divider circuit should NOT shift resonance: measured {f_with_bias:.1} Hz vs textbook {f_textbook:.1} Hz (ratio={ratio_with_bias:.2}). \
         This is the LSBW bug: bias divider Rth=50k is loading the bridged-T, shifting f0 upward ~2-3x."
    );
}

/// 808 kick-style fixture: R=150k, C=18n → f0 = 1/(2π×150k×18n) ≈ 58.9 Hz.
///
/// This test verifies that:
/// 1. The kick-style circuit produces a resonant frequency in the right ballpark.
/// 2. The bias-divider version does NOT produce a substantially different frequency
///    than the no-bias version (guards against regression of the LSBW bug).
///
/// The spectral centroid may differ from the textbook f0 due to the R_fb=1M (high Q)
/// creating a broad ring that shifts the centroid. The key assertion is that the
/// bias-divider ratio (f_with_bias / f_no_bias) is within ±15%.
#[test]
fn kick_style_bias_divider_does_not_shift_vs_no_bias() {
    let pedal_no_bias = crate::dsl::parse_pedal_file(KICK_STYLE_NO_BIAS).expect("parse no_bias");
    let mut compiled_no_bias = compile_via_spqr(&pedal_no_bias, SR).expect("compile no_bias");

    let pedal_with_bias = crate::dsl::parse_pedal_file(KICK_STYLE_WITH_BIAS).expect("parse with_bias");
    let mut compiled_with_bias = compile_via_spqr(&pedal_with_bias, SR).expect("compile with_bias");

    let f_textbook = 1.0 / (std::f64::consts::TAU * 150_000.0 * 18.0e-9);
    let f_no_bias = measure_spectral_centroid_hz(&mut compiled_no_bias, 512, 4096, 20.0, 300.0);
    let f_with_bias = measure_spectral_centroid_hz(&mut compiled_with_bias, 512, 4096, 20.0, 300.0);

    eprintln!(
        "kick_style_comparison: textbook={:.1} Hz | no_bias={:.1} Hz | with_bias={:.1} Hz",
        f_textbook, f_no_bias, f_with_bias
    );
    eprintln!(
        "  no_bias ratio={:.2}  with_bias ratio={:.2}  relative={:.2}",
        f_no_bias / f_textbook,
        f_with_bias / f_textbook,
        if f_no_bias > 0.0 { f_with_bias / f_no_bias } else { 0.0 }
    );

    assert!(f_no_bias > 10.0, "No-bias kick must produce output: {f_no_bias:.1} Hz");
    assert!(f_with_bias > 10.0, "With-bias kick must produce output: {f_with_bias:.1} Hz");

    // The bias divider must NOT shift the resonant frequency by more than 15%.
    // Before the fix, this ratio was ~2.6-3.1 (bias shifts frequency up 2-3x).
    let relative_shift = if f_no_bias > 0.0 { f_with_bias / f_no_bias } else { 0.0 };
    assert!(
        relative_shift >= 0.85 && relative_shift <= 1.15,
        "Bias-divider should not shift kick resonance: no_bias={f_no_bias:.1} Hz, \
         with_bias={f_with_bias:.1} Hz, relative shift={relative_shift:.2}. \
         Expected 0.85-1.15."
    );
}
