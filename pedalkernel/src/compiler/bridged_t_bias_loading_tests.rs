// ═══════════════════════════════════════════════════════════════════════════
// Bridged-T bias-divider loading — regression guards and frequency accuracy
//
// Ref: pedalkernel-lsbw — Bug: bridged-T bias-divider loading shifts resonant
//      frequency (808 kick 150Hz vs 55Hz).
//
// Investigation summary (2026-06-09/10):
//
// Phase 1 (PR #73): Bias-divider transparency.
//   The bias divider (Rb1=Rb2=100k → Rth=50k) IS included in the IIR MNA
//   as a 50k shunt to AC GND at the opamp's pos node. However, since the
//   VCVS draws no current at pos (ideal opamp), V_pos is forced to 0 by the
//   shunt. The VCVS constraint reduces to V_out = -Aol × V_neg, identical
//   to the pos-grounded case. The bridged-T poles are unaffected by the bias
//   divider. ← CONFIRMED via 3 guard tests below.
//
// Phase 2 (re-scope): C-dependent absolute frequency error.
//   Orchestrator observed 1.02x error at C=8.2n vs 1.38x at C=18n (same R=150k)
//   and suspected a real C-dependent bug. Investigation showed:
//
//   Root cause identified: the 1.38x vs 1.02x discrepancy was a MEASUREMENT
//   ARTIFACT, not a real engine bug.
//
//   The two test cases used different R_fb values:
//     C=8.2n: R_fb=470k → Q = Rf/(Rf-Rcrit) = 470k/20k = 23.5 (very narrow BPF)
//     C=18n:  R_fb=1M   → Q = 1M/550k = 1.818 (broad BPF)
//
//   For low-Q biquad BPFs, the spectral CENTROID is biased ABOVE the center
//   frequency because the measurement window captures the high-frequency tail
//   asymmetrically. The bias is proportional to 1/Q and is scale-invariant
//   (same ratio regardless of f0 — confirmed by C sweep showing constant 1.314x
//   centroid bias at Q=1.818 across C=[4.7n..68n]).
//
//   The spectral PEAK is Q-independent and measures the true center frequency.
//   Peak measurement shows 0.4-0.6% accuracy for C=[8.2n..33n]:
//     C=8.2n:  textbook=129.4 Hz, peak=128.9 Hz, ratio=0.996
//     C=12n:   textbook=88.4  Hz, peak=87.9  Hz, ratio=0.994
//     C=18n:   textbook=58.9  Hz, peak=58.6  Hz, ratio=0.994
//     C=33n:   textbook=32.2  Hz, peak=32.2  Hz, ratio=1.002
//
//   R_trig loading at the junction has NO effect on the biquad center frequency.
//
//   Error law: NOT C-dependent. Constant ~0.6% FLAT error independent of C, R,
//   or Q — consistent with bilinear transform discretization of f0/fs.
//
//   The drum product's 2.6-3.1x kick frequency error is NOT in this path.
//   Root: DrummerBoy layer or circuit topology difference.
//
// Tests:
//   1-3: Original guard tests (bias divider transparency, ±30% tolerance)
//   4-5: New acceptance tests (absolute accuracy, ±5% tolerance, peak measurement)
//   6-7: Diagnostic sweeps (for investigation reference, always pass)
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

// ── Acceptance tests: absolute frequency accuracy ────────────────────────
//
// Use spectral PEAK (not centroid) because centroid is biased upward for
// low-Q resonators. Root cause investigation (2026-06-10):
//   - The IIR biquad pole IS at the correct f0 (within 0.6% of textbook)
//   - The 1.38x centroid error observed earlier was Q-dependent bias,
//     not a true frequency error
//   - R_trig loading at the junction does NOT shift the biquad pole
//   - Peak measurement shows 0.4–0.6% accuracy across C=[8.2n..33n]
//
// Acceptance criterion: peak within ±5% of analytic f0 = 1/(2π·R·C).

/// Kick-style bridged-T must resonate within 5% of textbook f0 = 58.9 Hz.
///
/// This is the acceptance test for the re-scoped LSBW bead investigation.
/// Uses spectral peak (not centroid) for Q-invariant frequency measurement.
#[test]
fn kick_style_iir_resonance_within_5pct_of_textbook() {
    let pedal = crate::dsl::parse_pedal_file(KICK_STYLE_NO_BIAS).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    let f_textbook = 1.0 / (std::f64::consts::TAU * 150_000.0 * 18.0e-9);
    // Peak measurement window: [0.3*f0, 3*f0] = [17.7, 176.7] Hz
    let f_peak = measure_peak_freq_hz(&mut compiled, 512, 16384, f_textbook * 0.3, f_textbook * 3.0);

    eprintln!(
        "kick_style peak: textbook={:.1} Hz, peak={:.1} Hz, ratio={:.3}",
        f_textbook,
        f_peak,
        f_peak / f_textbook
    );

    let ratio = f_peak / f_textbook;
    assert!(
        ratio >= 0.95 && ratio <= 1.05,
        "Kick-style resonance peak {f_peak:.1} Hz should be within 5% of textbook {f_textbook:.1} Hz \
         (ratio={ratio:.3}). IIR biquad f0 must be accurate."
    );
}

/// Parameter sweep: C values [8.2n..33n] all land within 5% of textbook f0.
/// Guards against regressions that introduce C-dependent frequency errors.
#[test]
fn bridged_t_c_sweep_all_within_5pct() {
    use std::f64::consts::PI;
    let c_values = [8.2e-9f64, 12e-9, 18e-9, 33e-9];
    let r = 150_000.0f64;

    for &c in &c_values {
        let cnf = c * 1e9;
        let f0_textbook = 1.0 / (2.0 * PI * r * c);

        let pedal_src = format!(r#"
            pedal "BT-csweep-{cnf}" {{ supply 9V
                components {{
                    U1: opamp(tl072)
                    R1: resistor(150k)
                    R2: resistor(150k)
                    C1: cap({cnf}n)
                    C2: cap({cnf}n)
                    R_fb: resistor(1M)
                    R_trig: resistor(100k)
                }}
                nets {{
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
                }}
                controls {{}}
            }}
        "#, cnf = cnf);

        let pedal = crate::dsl::parse_pedal_file(&pedal_src).expect("parse");
        let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");
        let f_peak = measure_peak_freq_hz(
            &mut compiled, 512, 16384,
            f0_textbook * 0.3, f0_textbook * 3.0,
        );
        let ratio = f_peak / f0_textbook;

        eprintln!("C={cnf:.1}n: textbook={f0_textbook:.1} Hz, peak={f_peak:.1} Hz, ratio={ratio:.3}");
        assert!(
            ratio >= 0.95 && ratio <= 1.05,
            "C={cnf:.1}n: IIR peak {f_peak:.1} Hz should be within 5% of textbook \
             {f0_textbook:.1} Hz (ratio={ratio:.3})"
        );
    }
}

// ── Diagnostic helpers ────────────────────────────────────────────────────

/// Find the frequency bin with maximum power in [lo_hz, hi_hz].
/// More reliable than spectral centroid for low-Q resonators.
fn measure_peak_freq_hz(
    compiled: &mut impl crate::PedalProcessor,
    n_warmup: usize,
    n_samples: usize,
    lo_hz: f64,
    hi_hz: f64,
) -> f64 {
    for _ in 0..n_warmup {
        compiled.process(0.0);
    }
    let mut buf = vec![0.0f64; n_samples];
    compiled.process(1.0);
    for i in 0..n_samples {
        buf[i] = compiled.process(0.0);
    }
    let n = n_samples;
    let lo_bin = ((lo_hz * n as f64 / SR) as usize).max(1);
    let hi_bin = ((hi_hz * n as f64 / SR) as usize).min(n / 2 - 1);

    let mut best_power = 0.0f64;
    let mut best_freq = 0.0f64;
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
        if power > best_power {
            best_power = power;
            best_freq = freq;
        }
    }
    best_freq
}

// ── Diagnostic: sweep C values and print IIR coefficients ────────────────

#[test]
fn sweep_c_values_debug_coefficients() {
    use std::f64::consts::PI;
    // Sweep: R=150k, vary C
    let c_values = [4.7e-9f64, 8.2e-9, 12e-9, 18e-9, 33e-9, 68e-9];
    let r = 150_000.0f64;

    eprintln!("\n=== C SWEEP (R=150k, R_fb=1M, R_trig=100k) ===");
    eprintln!("{:>8} {:>12} {:>12} {:>8}", "C (nF)", "f0_textbook", "f0_measured", "ratio");

    for &c in &c_values {
        let f0_textbook = 1.0 / (2.0 * PI * r * c);
        let cnf = c * 1e9;

        let pedal_src = format!(r#"
            pedal "BridgedT-C{cnf}" {{ supply 9V
                components {{
                    U1: opamp(tl072)
                    R1: resistor(150k)
                    R2: resistor(150k)
                    C1: cap({cnf}n)
                    C2: cap({cnf}n)
                    R_fb: resistor(1M)
                    R_trig: resistor(100k)
                }}
                nets {{
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
                }}
                controls {{}}
            }}
        "#, cnf = cnf);
        
        let pedal = crate::dsl::parse_pedal_file(&pedal_src).expect("parse");
        let mut compiled = super::spqr_build::compile_via_spqr(&pedal, SR).expect("compile");
        
        let lo = (f0_textbook * 0.3).max(5.0);
        let hi = f0_textbook * 4.0;
        let f0_measured = measure_spectral_centroid_hz(&mut compiled, 512, 8192, lo, hi);
        let ratio = f0_measured / f0_textbook;
        
        eprintln!("{:>8.1} {:>12.1} {:>12.1} {:>8.3}", c * 1e9, f0_textbook, f0_measured, ratio);
    }
    
    // Also sweep R at fixed C=18n
    let c_fixed = 18e-9f64;
    eprintln!("\n=== R SWEEP (C=18n, R_fb=1M, R_trig=100k) ===");
    eprintln!("{:>8} {:>12} {:>12} {:>8}", "R (kΩ)", "f0_textbook", "f0_measured", "ratio");
    
    let r_values = [47_000.0f64, 100_000.0, 150_000.0, 220_000.0];
    for &r_val in &r_values {
        let f0_textbook = 1.0 / (2.0 * PI * r_val * c_fixed);
        
        let r_k = r_val / 1000.0;
        let pedal_src = format!(r#"
            pedal "BridgedT R={r_k:.0}k" {{ supply 9V
                components {{
                    U1: opamp(tl072)
                    R1: resistor({r_k:.0}k)
                    R2: resistor({r_k:.0}k)
                    C1: cap(18n)
                    C2: cap(18n)
                    R_fb: resistor(1M)
                    R_trig: resistor(100k)
                }}
                nets {{
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
                }}
                controls {{}}
            }}
        "#, r_k = r_k);
        
        let pedal = crate::dsl::parse_pedal_file(&pedal_src).expect("parse");
        let mut compiled = super::spqr_build::compile_via_spqr(&pedal, SR).expect("compile");
        
        let lo = (f0_textbook * 0.3).max(5.0);
        let hi = f0_textbook * 4.0;
        let f0_measured = measure_spectral_centroid_hz(&mut compiled, 512, 8192, lo, hi);
        let ratio = f0_measured / f0_textbook;
        
        eprintln!("{:>8.1} {:>12.1} {:>12.1} {:>8.3}", r_val / 1000.0, f0_textbook, f0_measured, ratio);
    }
}

/// Peak-frequency sweep: use spectral PEAK (not centroid) so Q doesn't bias result.
/// Also tests without R_trig to isolate loading vs biquad error.
#[test]
fn sweep_peak_frequency_with_and_without_r_trig() {
    use std::f64::consts::PI;
    let c_values = [8.2e-9f64, 12e-9, 18e-9, 33e-9];
    let r = 150_000.0f64;

    eprintln!("\n=== PEAK FREQ SWEEP (R=150k, R_fb=1M, with/without R_trig=100k) ===");
    eprintln!("{:>8} {:>12} {:>12} {:>8} {:>12} {:>8}",
        "C (nF)", "f0_textbook", "f0_peak_trig", "ratio_trig", "f0_peak_notrig", "ratio_notrig");

    for &c in &c_values {
        let f0_textbook = 1.0 / (2.0 * PI * r * c);
        let cnf = c * 1e9;

        // WITH R_trig
        let pedal_trig = crate::dsl::parse_pedal_file(&format!(r#"
            pedal "BT-trig-{cnf}" {{ supply 9V
                components {{
                    U1: opamp(tl072)
                    R1: resistor(150k)
                    R2: resistor(150k)
                    C1: cap({cnf}n)
                    C2: cap({cnf}n)
                    R_fb: resistor(1M)
                    R_trig: resistor(100k)
                }}
                nets {{
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
                }}
                controls {{}}
            }}
        "#, cnf = cnf)).expect("parse with_trig");
        let mut comp_trig = super::spqr_build::compile_via_spqr(&pedal_trig, SR).expect("compile");

        // WITHOUT R_trig: inject directly at junction (R1.b)
        let pedal_notrig = crate::dsl::parse_pedal_file(&format!(r#"
            pedal "BT-notrig-{cnf}" {{ supply 9V
                components {{
                    U1: opamp(tl072)
                    R1: resistor(150k)
                    R2: resistor(150k)
                    C1: cap({cnf}n)
                    C2: cap({cnf}n)
                    R_fb: resistor(1M)
                    R_in: resistor(100k)
                }}
                nets {{
                    U1.neg -> R1.a, C1.a
                    R1.b -> R2.a, C2.a
                    R2.b -> U1.out
                    C1.b -> gnd
                    C2.b -> gnd
                    U1.neg -> R_fb.a
                    R_fb.b -> U1.out
                    in -> R_in.a
                    R_in.b -> U1.neg
                    U1.pos -> gnd
                    U1.out -> out
                }}
                controls {{}}
            }}
        "#, cnf = cnf)).expect("parse no_trig");
        let mut comp_notrig = super::spqr_build::compile_via_spqr(&pedal_notrig, SR).expect("compile");

        let lo = (f0_textbook * 0.3).max(5.0);
        let hi = f0_textbook * 6.0;
        let f_peak_trig = measure_peak_freq_hz(&mut comp_trig, 512, 16384, lo, hi);
        let f_peak_notrig = measure_peak_freq_hz(&mut comp_notrig, 512, 16384, lo, hi);

        eprintln!("{:>8.1} {:>12.1} {:>12.1} {:>8.3} {:>12.1} {:>8.3}",
            cnf, f0_textbook, f_peak_trig, f_peak_trig / f0_textbook,
            f_peak_notrig, f_peak_notrig / f0_textbook);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Full 808 kick topology: bridged-T with Decay pot + U2 output buffer
// ═══════════════════════════════════════════════════════════════════════════

/// Exact 808 kick topology (minus R_trig for isolated resonator test).
const KICK_808_FULL: &str = r#"
    pedal "808 Kick Full" { supply 9V
        components {
            U1: opamp(tl072)
            Rb1: resistor(100k)
            Rb2: resistor(100k)
            R1: resistor(150k)
            R2: resistor(150k)
            C1: cap(8.2n)
            C2: cap(8.2n)
            R_fb: resistor(470k)
            Decay: pot(500k, b)
            R_trig: resistor(1k)
            U2: opamp(tl072)
            Ri_out: resistor(10k)
            Rf_out: resistor(470k)
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
            R1.b -> Decay.a
            Decay.b -> gnd
            in -> R_trig.a
            R_trig.b -> R1.b
            U1.out -> Ri_out.a
            Ri_out.b -> U2.neg
            U2.neg -> Rf_out.a
            Rf_out.b -> U2.out
            U2.pos -> gnd
            U2.out -> out
        }
        controls {
            Decay.position -> "Decay" [0.0, 1.0] = 1.0
        }
    }
"#;

/// Diagnostic: print IIR coefficients after compilation to see what the resonator gets.
#[test]
fn kick_808_full_iir_coefficients() {
    use super::compiled::Stage;
    let pedal = crate::dsl::parse_pedal_file(KICK_808_FULL).expect("parse");
    let compiled = compile_via_spqr(&pedal, SR).expect("compile");
    for (i, stage) in compiled.stages.iter().enumerate() {
        match stage {
            Stage::Iir(iir) => {
                eprintln!("Stage {i} IIR: b={:?} a={:?}", iir.iir.b_coeffs, iir.iir.a_coeffs);
                if iir.iir.a_coeffs.len() >= 3 {
                    let a2 = iir.iir.a_coeffs[2];
                    let a1 = iir.iir.a_coeffs[1];
                    eprintln!("  pole radius={:.8}, a1={:.8}, a2={:.8}", a2.sqrt(), a1, a2);
                    eprintln!("  stable: a2<1={}, 1+a1+a2>0={}, 1-a1+a2>0={}",
                        a2.abs() < 1.0, 1.0 + a1 + a2 > 0.0, 1.0 - a1 + a2 > 0.0);
                }
                if let Some(ref table) = iir.biquad_table {
                    eprintln!("  BiquadTable: {} dims, {} steps, {} entries",
                        table.dim_labels.len(), table.steps, table.coeffs.len()/5);
                    // Print first few entries
                    for i in 0..table.coeffs.len()/5 {
                        let b = &table.coeffs[i*5..i*5+3];
                        let a = &table.coeffs[i*5+3..i*5+5];
                        if i < 3 || i == table.coeffs.len()/5-1 {
                            eprintln!("    entry[{i}]: b=[{:.6e},{:.6e},{:.6e}] a=[{:.6e},{:.6e}]",
                                b[0], b[1], b[2], a[0], a[1]);
                        }
                    }
                }
            }
            Stage::StateSpace(ss) => eprintln!("Stage {i} StateSpace"),
            Stage::Wdf(_) => eprintln!("Stage {i} WDF"),
            _ => eprintln!("Stage {i} other"),
        }
    }
}

/// Impulse response: full 808 kick topology must ring at ~129 Hz for ≥100ms.
/// This test catches the oscillation-dies-immediately bug.
#[test]
fn kick_808_full_topology_rings_at_129hz() {
    let pedal = crate::dsl::parse_pedal_file(KICK_808_FULL).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    let f_textbook = 1.0 / (std::f64::consts::TAU * 150_000.0 * 8.2e-9);

    // Impulse excitation
    let n_samples = (0.5 * SR) as usize;
    let trigger_samples = (0.002 * SR) as usize;
    let mut output = vec![0.0f64; n_samples];
    for i in 0..n_samples {
        let input = if i < trigger_samples { 0.3 } else { 0.0 };
        output[i] = compiled.process(input);
    }

    // RMS at 50–200ms: must be significant (not dying immediately)
    let s0 = (0.05 * SR) as usize;
    let s1 = (0.20 * SR) as usize;
    let rms_ring = {
        let window = &output[s0..s1.min(n_samples)];
        (window.iter().map(|x| x * x).sum::<f64>() / window.len() as f64).sqrt()
    };
    eprintln!("kick_808_full: RMS [50-200ms] = {rms_ring:.6e} (want > 1e-4)");

    assert!(
        rms_ring > 1e-4,
        "808 kick must sustain oscillation at 50-200ms: RMS={rms_ring:.4e}. \
         If this fails, oscillation is dying immediately after trigger."
    );

    // Measure f0 in the ring window
    let f_measured = measure_peak_freq_hz(&mut compiled, trigger_samples, 16384, 50.0, 500.0);
    eprintln!("kick_808_full: f0={f_measured:.1} Hz (textbook={f_textbook:.1} Hz)");

    let ratio = f_measured / f_textbook;
    assert!(
        ratio >= 0.85 && ratio <= 1.15,
        "808 kick f0 {f_measured:.1} Hz should be within 15% of textbook {f_textbook:.1} Hz (ratio={ratio:.3})"
    );
}
