//! Validation test suite with analytical references.
//!
//! Levels 0–5: from primitive WDF elements to full product pedals.
//! Every level pairs pure-math sanity checks with engine assertions.
//!
//! Run without pro-pedals:  cargo test -p pedalkernel --test validation
//! Run with pro-pedals:     cargo test -p pedalkernel --features pro-pedals --test validation

mod audio_analysis;

use std::f64::consts::PI;
use std::path::Path;

use audio_analysis::*;

// ===========================================================================
// Inline circuit definitions
// ===========================================================================

const RESISTOR_DIVIDER: &str = r#"pedal "Resistor Divider" {
  components { R1: resistor(10k)  R2: resistor(10k) }
  nets { in -> R1.a  R1.b -> R2.a, out  R2.b -> gnd }
}"#;

const RC_HIGHPASS: &str = r#"pedal "RC Highpass" {
  components { C1: cap(22n)  R1: resistor(33k) }
  nets { in -> C1.a  C1.b -> R1.a, out  R1.b -> gnd }
}"#;

const RC_LOWPASS: &str = r#"pedal "RC Lowpass" {
  components { R1: resistor(10k)  C1: cap(4.7n) }
  nets { in -> R1.a  R1.b -> C1.a, out  C1.b -> gnd }
}"#;

const RL_HIGHPASS: &str = r#"pedal "RL Highpass" {
  components { R1: resistor(1k)  L1: inductor(100m) }
  nets { in -> R1.a  R1.b -> L1.a, out  L1.b -> gnd }
}"#;

const CASCADED_LPF: &str = r#"pedal "Cascaded LPF" {
  components { R1: resistor(10k)  C1: cap(22n)  R2: resistor(10k)  C2: cap(22n) }
  nets { in -> R1.a  R1.b -> C1.a, R2.a  C1.b -> gnd  R2.b -> C2.a, out  C2.b -> gnd }
}"#;

const SERIES_RLC: &str = r#"pedal "Series RLC Bandpass" {
  components { R1: resistor(100)  L1: inductor(10m)  C1: cap(100n)  RL: resistor(10k) }
  nets { in -> R1.a  R1.b -> L1.a  L1.b -> C1.a  C1.b -> RL.a, out  RL.b -> gnd }
}"#;

const TWIN_T_NOTCH: &str = r#"pedal "Twin-T Notch" {
  components {
    R1: resistor(10k)  R2: resistor(10k)  R3: resistor(5k)
    C1: cap(10n)  C2: cap(10n)  C3: cap(20n)
  }
  nets {
    in -> R1.a, C1.a
    R1.b -> R2.a, C3.a
    R2.b -> C2.a, out
    C1.b -> C2.b, R3.a
    C3.b -> gnd
    R3.b -> gnd
  }
}"#;

const SINGLE_DIODE: &str = r#"pedal "Single Diode Rectifier" {
  components { R1: resistor(4.7k)  D1: diode(silicon)  RL: resistor(47k) }
  nets { in -> R1.a  R1.b -> D1.a  D1.b -> RL.a, out  RL.b -> gnd }
}"#;

const OPAMP_CLIPPER: &str = r#"pedal "Op-Amp Diode Clipper" {
  supply 9V
  components {
    R_in: resistor(10k)  R_fb: resistor(100k)
    D1: diode_pair(silicon)  U1: opamp(tl072)
    R_bias1: resistor(100k)  R_bias2: resistor(100k)
    RL: resistor(10k)
  }
  nets {
    vcc -> R_bias1.a
    R_bias1.b -> R_bias2.a, U1.pos
    R_bias2.b -> gnd
    in -> R_in.a
    R_in.b -> U1.neg
    R_fb.a -> U1.neg
    R_fb.b -> U1.out
    D1.a -> U1.neg
    D1.b -> U1.out
    U1.out -> RL.a, out
    RL.b -> gnd
  }
}"#;

const BJT_DIFF_PAIR: &str = r#"pedal "BJT Differential Pair" {
  supply 12V
  components {
    C1: cap(100n)  R_B1: resistor(100k)  R_B2: resistor(100k)
    Q1: npn(2n3904)  Q2: npn(2n3904)
    RC1: resistor(10k)  RC2: resistor(10k)
    R_tail: resistor(10k)
    C_out: cap(1u)  RL: resistor(10k)
  }
  nets {
    in -> C1.a
    C1.b -> R_B1.a, Q1.base
    R_B1.b -> vcc
    R_B2.a -> Q2.base
    R_B2.b -> vcc
    Q1.collector -> RC1.a
    Q2.collector -> RC2.a
    RC1.b -> vcc
    RC2.b -> vcc
    Q1.emitter -> Q2.emitter, R_tail.a
    R_tail.b -> gnd
    Q1.collector -> C_out.a
    C_out.b -> RL.a, out
    RL.b -> gnd
  }
}"#;

const POT_LPF: &str = r#"pedal "Pot LPF" {
  components { Freq: pot(100k)  C1: cap(4.7n) }
  nets { in -> Freq.a  Freq.b -> C1.a, out  C1.b -> gnd }
  controls { Freq.position -> "Freq" [0.0, 1.0] = 0.5 }
}"#;

// ===========================================================================
// Analytical formulas
// ===========================================================================

/// Thermal voltage at room temperature (25 C).
const VT: f64 = 25.85e-3;

/// RC highpass: |H(f)| = (f/fc) / sqrt(1 + (f/fc)^2)
fn rc_highpass_gain(f: f64, fc: f64) -> f64 {
    let x = f / fc;
    x / (1.0 + x * x).sqrt()
}

/// RC lowpass: |H(f)| = 1 / sqrt(1 + (f/fc)^2)
fn rc_lowpass_gain(f: f64, fc: f64) -> f64 {
    let x = f / fc;
    1.0 / (1.0 + x * x).sqrt()
}

/// Cascaded (2-pole) lowpass: square of single-pole response.
fn cascaded_lowpass_gain(f: f64, fc: f64) -> f64 {
    let g = rc_lowpass_gain(f, fc);
    g * g
}

/// RL highpass: |H(f)| = (f/fc) / sqrt(1 + (f/fc)^2), fc = R/(2piL)
#[allow(dead_code)]
fn rl_highpass_gain(f: f64, fc: f64) -> f64 {
    let x = f / fc;
    x / (1.0 + x * x).sqrt()
}

/// Series RLC with load RL — voltage across RL.
/// At resonance XL=XC cancel, gain = RL/(R+RL).
fn rlc_series_loaded_gain(f: f64, r_series: f64, r_load: f64, l: f64, c: f64) -> f64 {
    let w = 2.0 * PI * f;
    let xl = w * l;
    let xc = 1.0 / (w * c);
    let x_net = xl - xc;
    r_load / ((r_series + r_load).powi(2) + x_net * x_net).sqrt()
}

/// Shockley diode equation: I = Is * (exp(V/Vt) - 1)
fn shockley_diode(v: f64, is: f64, vt: f64) -> f64 {
    is * ((v / vt).exp() - 1.0)
}

/// Derivative of Shockley equation: dI/dV = Is/Vt * exp(V/Vt)
fn shockley_diode_deriv(v: f64, is: f64, vt: f64) -> f64 {
    is / vt * (v / vt).exp()
}

/// JFET channel resistance: Rds = 1 / (2 * K * (Vgs - Vp))
fn jfet_rds(vgs: f64, vp: f64, idss: f64) -> f64 {
    let k = idss / (vp * vp);
    1.0 / (2.0 * k * (vgs - vp))
}

// ===========================================================================
// Test helpers
// ===========================================================================

/// Number of warmup samples to skip before analysis.
const WARMUP: usize = 2048;
/// Default test buffer length.
const TEST_LEN: usize = 8192;

/// Generate a sine buffer with given sample count.
fn sine_buf(freq: f64, fs: f64, n: usize, amplitude: f64) -> Vec<f64> {
    (0..n)
        .map(|i| amplitude * (2.0 * PI * freq * i as f64 / fs).sin())
        .collect()
}

/// Measure gain at a specific frequency by comparing Goertzel magnitudes.
/// Skips WARMUP samples to avoid transient.
fn gain_at_freq(input: &[f64], output: &[f64], fs: f64, freq: f64) -> f64 {
    let in_mag = goertzel_mag(&input[WARMUP..], fs, freq);
    let out_mag = goertzel_mag(&output[WARMUP..], fs, freq);
    if in_mag < 1e-20 {
        return 0.0;
    }
    out_mag / in_mag
}

/// Assert gain is within tolerance_db of expected (in dB).
fn assert_gain_db(measured: f64, expected: f64, tolerance_db: f64, label: &str) {
    if expected < 1e-20 && measured < 1e-20 {
        return;
    }
    let err_db = if expected < 1e-20 {
        20.0 * measured.log10()
    } else {
        (20.0 * (measured / expected).log10()).abs()
    };
    assert!(
        err_db < tolerance_db,
        "{label}: gain={measured:.4}, expected={expected:.4}, err={err_db:.1}dB (tol={tolerance_db}dB)"
    );
}

// ===========================================================================
// Level 0: Primitive WDF Validation
// ===========================================================================

#[test]
fn wdf_resistor_reflection() {
    // Math: Gamma = (R - Rp) / (R + Rp). When matched, Gamma = 0.
    let r = 10_000.0;
    let rp = r;
    let gamma: f64 = (r - rp) / (r + rp);
    assert!(gamma.abs() < 1e-12, "Matched R: Gamma should be 0, got {gamma}");

    // Mismatched: R > Rp → positive reflection
    let gamma2 = (r - 1000.0) / (r + 1000.0);
    assert!((gamma2 - 9000.0_f64 / 11000.0).abs() < 1e-10);

    // Engine: resistor divider R1=R2=10k → gain = 0.5
    let fs = 48000.0;
    let input = sine_buf(1000.0, fs, TEST_LEN, 0.5);
    let output = compile_and_process(RESISTOR_DIVIDER, &input, fs, &[]);
    let g = gain_at_freq(&input, &output, fs, 1000.0);
    assert_gain_db(g, 0.5, 1.5, "Resistor divider at 1kHz");
}

#[test]
fn wdf_capacitor_port_resistance() {
    // Math: Rp = 1/(2*C*fs)
    let c = 4.7e-9;
    let fs = 48000.0;
    let rp: f64 = 1.0 / (2.0 * c * fs);
    let expected_rp: f64 = 1.0 / (2.0 * 4.7e-9 * 48000.0);
    assert!((rp - expected_rp).abs() < 1.0, "Cap Rp: {rp} vs {expected_rp}");

    // Engine: RC lowpass R=10k, C=4.7n → fc ≈ 3386 Hz, gain at fc = -3dB
    let r = 10_000.0;
    let fc = 1.0 / (2.0 * PI * r * c);
    let input = sine_buf(fc, fs, TEST_LEN, 0.5);
    let output = compile_and_process(RC_LOWPASS, &input, fs, &[]);
    let g = gain_at_freq(&input, &output, fs, fc);
    assert_gain_db(g, 0.707, 2.0, "RC LPF gain at fc");
}

#[test]
fn wdf_inductor_port_resistance() {
    // Math: Rp = 2*L*fs
    let l = 0.1;
    let fs = 48000.0;
    let rp: f64 = 2.0 * l * fs;
    assert!((rp - 9600.0).abs() < 1.0, "Inductor Rp = {rp}, expected 9600");

    // Engine: RL highpass R=1k, L=100mH → fc = R/(2piL) ≈ 1592 Hz
    let r = 1000.0;
    let fc = r / (2.0 * PI * l);
    let input = sine_buf(fc, fs, TEST_LEN, 0.5);
    let output = compile_and_process(RL_HIGHPASS, &input, fs, &[]);
    let g = gain_at_freq(&input, &output, fs, fc);
    assert_gain_db(g, 0.707, 2.0, "RL HPF gain at fc");
}

// ===========================================================================
// Level 1: Linear WDF Circuits
// ===========================================================================

#[test]
fn linear_rc_highpass() {
    let r = 33_000.0;
    let c = 22e-9;
    let fs = 48000.0;
    let fc = 1.0 / (2.0 * PI * r * c);

    // === Math sanity ===
    let g_at_fc = rc_highpass_gain(fc, fc);
    assert!((g_at_fc - 0.7071).abs() < 0.001, "Gain at fc = {g_at_fc}");
    // 10/sqrt(101) = 0.9950
    let g_at_10fc = rc_highpass_gain(10.0 * fc, fc);
    assert!((g_at_10fc - 0.9950).abs() < 0.001, "Gain at 10fc = {g_at_10fc}");

    // === Engine ===
    for &freq in &[50.0, 100.0, fc * 0.5, fc, fc * 2.0, 5000.0] {
        if freq > fs / 2.0 { continue; }
        let input = sine_buf(freq, fs, TEST_LEN, 0.1);
        let output = compile_and_process(RC_HIGHPASS, &input, fs, &[]);
        let measured = gain_at_freq(&input, &output, fs, freq);
        let expected = rc_highpass_gain(freq, fc);
        assert_gain_db(measured, expected, 1.5, &format!("RC HPF at {freq:.0}Hz"));
    }
}

#[test]
fn linear_rc_lowpass() {
    let r = 10_000.0;
    let c = 4.7e-9;
    let fs = 48000.0;
    let fc = 1.0 / (2.0 * PI * r * c);

    // === Math sanity ===
    assert!((rc_lowpass_gain(fc, fc) - 0.7071).abs() < 0.001);
    // 1/sqrt(1.01) = 0.9950
    assert!((rc_lowpass_gain(fc / 10.0, fc) - 0.9950).abs() < 0.001);

    // === Engine ===
    for &freq in &[100.0, 500.0, fc * 0.5, fc, fc * 2.0, 10000.0] {
        if freq > fs / 2.0 { continue; }
        let input = sine_buf(freq, fs, TEST_LEN, 0.1);
        let output = compile_and_process(RC_LOWPASS, &input, fs, &[]);
        let measured = gain_at_freq(&input, &output, fs, freq);
        let expected = rc_lowpass_gain(freq, fc);
        assert_gain_db(measured, expected, 1.5, &format!("RC LPF at {freq:.0}Hz"));
    }
}

#[test]
fn linear_cascaded_lowpass() {
    let r = 10_000.0;
    let c = 22e-9;
    let fs = 48000.0;
    let fc = 1.0 / (2.0 * PI * r * c);

    // === Math: at fc, gain = 0.707^2 = 0.5 (-6dB) ===
    assert!((cascaded_lowpass_gain(fc, fc) - 0.5).abs() < 0.01);

    // === Engine ===
    // BUG: Engine gives 0.33 instead of 0.50 (-3.5dB error).
    // The second RC stage loads the first through the WDF series adaptor,
    // causing excess attenuation beyond what the analytical formula predicts.
    let input = sine_buf(fc, fs, TEST_LEN, 0.1);
    let output = compile_and_process(CASCADED_LPF, &input, fs, &[]);
    let g = gain_at_freq(&input, &output, fs, fc);
    eprintln!("  [diag] Cascaded LPF at fc: engine={g:.4}, expected=0.5000");
    assert_gain_db(g, 0.5, 2.0, "Cascaded LPF at fc");

    // At 10*fc: expect < -35dB
    let freq_10fc = (10.0 * fc).min(fs / 2.0 - 100.0);
    let input_hi = sine_buf(freq_10fc, fs, TEST_LEN, 0.1);
    let output_hi = compile_and_process(CASCADED_LPF, &input_hi, fs, &[]);
    let g_hi = gain_at_freq(&input_hi, &output_hi, fs, freq_10fc);
    assert!(g_hi < 0.05, "Cascaded LPF at 10fc: gain={g_hi:.4}, expected <0.05");
}

#[test]
fn linear_rlc_bandpass() {
    let r_series = 100.0;
    let l = 10e-3;
    let c = 100e-9;
    let r_load = 10_000.0;
    let fs = 48000.0;
    let f0 = 1.0 / (2.0 * PI * (l * c as f64).sqrt());

    // === Math: at resonance, gain = RL/(R+RL) ≈ 0.99 ===
    let expected_peak = r_load / (r_series + r_load);
    let g_at_f0 = rlc_series_loaded_gain(f0, r_series, r_load, l, c);
    assert!((g_at_f0 - expected_peak).abs() < 0.01,
        "RLC at f0: {g_at_f0:.4} vs expected {expected_peak:.4}");

    // === Engine ===
    let input_res = sine_buf(f0, fs, TEST_LEN, 0.1);
    let output_res = compile_and_process(SERIES_RLC, &input_res, fs, &[]);
    let g_res = gain_at_freq(&input_res, &output_res, fs, f0);
    assert_gain_db(g_res, expected_peak, 3.0, "RLC bandpass at f0");

    // Off-resonance should be lower (note: high RL limits selectivity)
    let f_low = f0 / 10.0;
    let input_lo = sine_buf(f_low, fs, TEST_LEN, 0.1);
    let output_lo = compile_and_process(SERIES_RLC, &input_lo, fs, &[]);
    let g_lo = gain_at_freq(&input_lo, &output_lo, fs, f_low);
    eprintln!("  [diag] RLC: g_res={g_res:.4}, g_lo={g_lo:.4}");
    // BUG: Engine shows poor selectivity — g_lo is not significantly below g_res.
    // With RL=10k and R=100, analytical Q≈3.16, so off-resonance at f0/10 should
    // be well below resonance peak. Assert the expected selectivity.
    assert!(g_lo < g_res * 0.5,
        "RLC: off-resonance {g_lo:.4} should be < 50% of resonance {g_res:.4}");
}

#[test]
fn linear_twin_t_notch() {
    let r = 10_000.0;
    let c = 10e-9;
    let f_notch = 1.0 / (2.0 * PI * r * c);
    let fs = 48000.0;

    // Gain at notch
    let input_notch = sine_buf(f_notch, fs, TEST_LEN, 0.1);
    let output_notch = compile_and_process(TWIN_T_NOTCH, &input_notch, fs, &[]);
    let g_notch = gain_at_freq(&input_notch, &output_notch, fs, f_notch);

    // Gain at passband (well away from notch)
    let f_pass = f_notch / 5.0;
    let input_pass = sine_buf(f_pass, fs, TEST_LEN, 0.1);
    let output_pass = compile_and_process(TWIN_T_NOTCH, &input_pass, fs, &[]);
    let g_pass = gain_at_freq(&input_pass, &output_pass, fs, f_pass);

    eprintln!("  [diag] Twin-T: g_notch={g_notch:.4}, g_pass={g_pass:.4}");

    // Notch should be at least 10dB deeper than passband
    let depth_db = if g_notch > 1e-20 && g_pass > 1e-20 {
        20.0 * (g_pass / g_notch).log10()
    } else {
        f64::INFINITY
    };
    assert!(depth_db > 10.0, "Twin-T notch depth = {depth_db:.1}dB, expected >10dB");
}

// ===========================================================================
// Level 2: Single Nonlinear Element
// ===========================================================================

#[test]
fn diode_iv_curve() {
    let is = 1e-12;

    // V=0: I=0
    assert!(shockley_diode(0.0, is, VT).abs() < 1e-20);

    // V=0.6V: I ≈ 0.012A  (Is * exp(0.6/0.02585) ≈ 1e-12 * 1.2e10)
    let i06 = shockley_diode(0.6, is, VT);
    assert!((i06 - 0.012).abs() < 0.002, "Diode at 0.6V: I={i06:.4}, expected ~0.012");

    // V=0.3V: much smaller
    let i03 = shockley_diode(0.3, is, VT);
    assert!(i03 < i06 * 0.01, "I(0.3V)={i03:.6e} should be << I(0.6V)={i06:.6e}");

    // Derivative FD check
    let dv = 1e-6;
    let di_analytic = shockley_diode_deriv(0.6, is, VT);
    let di_fd = (shockley_diode(0.6 + dv, is, VT) - shockley_diode(0.6 - dv, is, VT)) / (2.0 * dv);
    assert!(((di_analytic - di_fd) / di_fd).abs() < 1e-4,
        "Derivative FD: analytic={di_analytic:.4e}, FD={di_fd:.4e}");
}

#[test]
fn single_diode_halfwave() {
    let fs = 48000.0;
    let v_f = 0.6;
    let r1 = 4700.0;
    let r_load = 47000.0;
    let divider = r_load / (r1 + r_load); // 0.909

    // Math: peak output = (Vpeak - Vf) * divider ≈ 0.364V
    let expected_peak: f64 = (1.0 - v_f) * divider;
    assert!((expected_peak - 0.364).abs() < 0.01);

    // Engine
    let n = TEST_LEN * 2;
    let input = sine_buf(440.0, fs, n, 1.0);
    let output = compile_and_process(SINGLE_DIODE, &input, fs, &[]);

    // Diode should create asymmetry: positive peaks larger than negative
    let pos_peak = output[WARMUP..].iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let neg_peak = output[WARMUP..].iter().cloned().fold(f64::INFINITY, f64::min).abs();
    let dc = dc_offset(&output[WARMUP..]);
    eprintln!("  [diag] Half-wave: pos_peak={pos_peak:.4}, neg_peak={neg_peak:.4}, DC={dc:.4}");

    // BUG: Engine does not produce proper half-wave rectification.
    // Expected: positive-only output with neg_ratio < 0.1 and positive DC offset.
    // Actual: near-symmetric output suggesting the diode is not clipping correctly.
    assert!(output[WARMUP..].iter().all(|x| x.is_finite()), "Half-wave: NaN/inf");
    let p = peak(&output[WARMUP..]);
    assert!(p > 1e-4, "Half-wave: output is silent (peak={p:.8})");

    // Half-wave rectification: negative peaks should be heavily attenuated
    let neg_ratio = neg_peak / pos_peak;
    assert!(neg_ratio < 0.1,
        "Half-wave: neg_ratio={neg_ratio:.4} (pos={pos_peak:.4}, neg={neg_peak:.4}), expected < 0.1");

    // Rectified signal should have positive DC offset
    assert!(dc > 0.01,
        "Half-wave: DC={dc:.6}, expected positive offset > 0.01");
}

#[test]
fn diode_pair_symmetric_clip() {
    let fs = 48000.0;
    let freq = 440.0;
    let levels = [0.01, 0.05, 0.1, 0.2, 0.5];
    let mut thd_values = Vec::new();

    for &amp in &levels {
        let input = sine_buf(freq, fs, TEST_LEN, amp);
        let output = compile_and_process(OPAMP_CLIPPER, &input, fs, &[]);
        let t = thd(&output[WARMUP..], fs, freq);
        eprintln!("  [diag] Clipper amp={amp:.2}: THD={t:.4}");
        thd_values.push(t);
    }

    // THD should generally increase with level
    let increasing = thd_values.windows(2).filter(|w| w[1] >= w[0] * 0.9).count();
    assert!(increasing >= thd_values.len() - 2,
        "Clipper THD should be mostly increasing: {thd_values:?}");
}

// ===========================================================================
// Level 3: Multi-NL (Newton-Raphson solver)
// ===========================================================================

#[test]
fn jacobian_finite_difference() {
    let is = 1e-12;
    let dv = 1e-7;

    for &v in &[0.0, 0.2, 0.4, 0.6, 0.7] {
        let analytic = shockley_diode_deriv(v, is, VT);
        let fd = (shockley_diode(v + dv, is, VT) - shockley_diode(v - dv, is, VT)) / (2.0 * dv);
        let rel_err = if fd.abs() > 1e-30 { ((analytic - fd) / fd).abs() } else { (analytic - fd).abs() };
        assert!(rel_err < 1e-3,
            "Jacobian at V={v}: analytic={analytic:.6e}, FD={fd:.6e}, err={rel_err:.6e}");
    }
}

#[test]
fn small_signal_linearization() {
    let is = 1e-12;
    let v0 = 0.5;
    let i0 = shockley_diode(v0, is, VT);
    let g0 = shockley_diode_deriv(v0, is, VT);

    for &delta in &[1e-6, 1e-5, 1e-4, 1e-3] {
        let i_exact = shockley_diode(v0 + delta, is, VT);
        let i_linear = i0 + g0 * delta;
        let rel_err = ((i_exact - i_linear) / i_exact).abs();
        if delta <= 1e-4 {
            assert!(rel_err < 0.01,
                "Linearization at delta={delta}: err={rel_err:.6e}");
        }
    }
}

#[test]
fn bjt_diff_pair_balance() {
    let fs = 48000.0;
    let input = sine_buf(440.0, fs, TEST_LEN * 2, 0.01);
    let output = compile_and_process(BJT_DIFF_PAIR, &input, fs, &[]);

    assert!(output[WARMUP..].iter().all(|x| x.is_finite()),
        "BJT diff pair: NaN/inf in output");
    let p = peak(&output[WARMUP..]);
    assert!(p < 20.0, "BJT diff pair: output railed (peak={p:.4})");
}

#[test]
#[ignore] // Needs debug-mode S-matrix instrumentation
fn r_type_embedding_constraint() {}

#[test]
#[ignore] // Needs debug-mode residual instrumentation
fn nr_residual_after_convergence() {}

// ===========================================================================
// Level 4: Hybrid Elements (JFET, OTA, Photocoupler)
// ===========================================================================

#[test]
fn jfet_variable_resistor_reflection() {
    // Math: Rds at Vgs=-0.5V with Vp=-2, Idss=5mA
    let rds = jfet_rds(-0.5, -2.0, 5e-3);
    assert!(rds > 0.0 && rds < 1e6, "JFET Rds should be finite positive: {rds}");

    let rp = 1000.0;
    let gamma = (rds - rp) / (rds + rp);
    assert!(gamma.abs() <= 1.0, "Gamma out of range: {gamma}");

    // Engine: JFET allpass produces non-silent output
    let fs = 48000.0;
    let input = sine_buf(440.0, fs, TEST_LEN, 0.1);
    let jfet_src = std::fs::read_to_string(
        Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/test_pedals/jfet_allpass.pedal"),
    ).expect("jfet_allpass.pedal not found");
    let output = compile_and_process(&jfet_src, &input, fs, &[]);
    let p = peak(&output[WARMUP..]);
    assert!(p > 1e-6, "JFET allpass silent (peak={p:.8})");
    assert!(output[WARMUP..].iter().all(|x| x.is_finite()), "JFET allpass: NaN/inf");
}

#[test]
fn jfet_rds_vs_vgs() {
    let vp = -2.0;
    let idss = 5e-3;

    // Vgs=0 (fully on): Rds = 1/(2*K*|Vp|) = 1/(2*1.25e-3*2) = 200
    let rds_on = jfet_rds(0.0, vp, idss);
    assert!((rds_on - 200.0).abs() < 1.0, "Rds(Vgs=0)={rds_on:.1}, expected 200");

    // Vgs=-1 (half on): Rds = 400
    let rds_half = jfet_rds(-1.0, vp, idss);
    assert!((rds_half - 400.0).abs() < 1.0, "Rds(Vgs=-1)={rds_half:.1}, expected 400");

    assert!(rds_half > rds_on, "Rds should increase as Vgs approaches Vp");
}

#[test]
fn ota_linear_regime() {
    // tanh(x) ≈ x, error ≈ x^2/3. At 8mV: x=0.155, err=0.80% (<1%).
    let v_small = 0.008;
    let x = v_small / (2.0 * VT);
    let rel_error = ((x.tanh() - x) / x.tanh()).abs();
    assert!(rel_error < 0.01,
        "OTA linear error at 8mV = {rel_error:.4}, expected <1%");

    // At 20mV: should be nonlinear (>1%)
    let x2 = 0.020 / (2.0 * VT);
    let rel_err2 = ((x2.tanh() - x2) / x2.tanh()).abs();
    assert!(rel_err2 > 0.01,
        "OTA should be nonlinear at 20mV (err={rel_err2:.4})");
}

#[test]
#[ignore] // Needs dyna_comp pedal
fn ota_compression_ratio() {}

#[test]
fn photocoupler_variable_resistance() {
    // VTL5C3 model: R_ldr = R_dark * exp(-k * I_led)
    // Qualitative check: bright < dim < dark
    let r_dark = 1e6;
    let k = 1000.0; // Sensitivity constant (steeper than real, for testability)
    let r_bright = r_dark * (-k * 10e-3_f64).exp();
    let r_dim = r_dark * (-k * 0.1e-3_f64).exp();

    eprintln!("  [diag] Photocoupler: bright={r_bright:.1}, dim={r_dim:.0}, dark={r_dark:.0}");

    // Monotonic: brighter LED → lower LDR resistance
    assert!(r_bright < r_dim, "Bright should be < dim: {r_bright:.0} vs {r_dim:.0}");
    assert!(r_dim < r_dark, "Dim should be < dark: {r_dim:.0} vs {r_dark:.0}");

    // Bright should be orders of magnitude less than dark
    assert!(r_bright < r_dark * 1e-3,
        "Bright should be << dark: {r_bright:.1} vs {r_dark:.0}");
}

// ===========================================================================
// Level 5: Integration Tests
// ===========================================================================

#[test]
fn pot_2terminal_filter_sweep() {
    let fs = 48000.0;
    let freq = 1000.0;
    let input = sine_buf(freq, fs, TEST_LEN, 0.1);

    // Low pot → low R → high fc → more signal at 1kHz
    let out_low = compile_and_process(POT_LPF, &input, fs, &[("Freq", 0.1)]);
    let g_low = gain_at_freq(&input, &out_low, fs, freq);

    // High pot → high R → low fc → less signal at 1kHz
    let out_high = compile_and_process(POT_LPF, &input, fs, &[("Freq", 0.9)]);
    let g_high = gain_at_freq(&input, &out_high, fs, freq);

    eprintln!("  [diag] Pot LPF: g@0.1={g_low:.4}, g@0.9={g_high:.4}");
    // BUG: Engine may have reversed pot direction or insufficient pot effect.
    // Low pot position → low R → high cutoff → MORE signal passes at 1kHz.
    // High pot position → high R → low cutoff → LESS signal passes at 1kHz.
    assert!(g_low > g_high,
        "Pot LPF: low pot should pass more signal: g@0.1={g_low:.4} should > g@0.9={g_high:.4}");
}

#[test]
#[ignore] // May need special pot topology
fn pot_3terminal_crossfade() {}

#[test]
fn deterministic_output() {
    let fs = 48000.0;
    let input = sine_buf(440.0, fs, TEST_LEN, 0.5);
    let out1 = compile_and_process(RC_LOWPASS, &input, fs, &[]);
    let out2 = compile_and_process(RC_LOWPASS, &input, fs, &[]);

    assert_eq!(out1.len(), out2.len());
    for (i, (&a, &b)) in out1.iter().zip(out2.iter()).enumerate() {
        assert!(a.to_bits() == b.to_bits(), "Non-deterministic at sample {i}: {a} != {b}");
    }
}

#[test]
fn output_bounded() {
    let fs = 48000.0;
    let bound = 10.0;

    for (name, circuit) in [("RC LPF", RC_LOWPASS), ("RC HPF", RC_HIGHPASS), ("Divider", RESISTOR_DIVIDER)] {
        let input = sine_buf(440.0, fs, TEST_LEN, 1.0);
        let output = compile_and_process(circuit, &input, fs, &[]);
        assert!(output.iter().all(|x| x.is_finite()), "{name}: NaN/inf");
        let p = peak(&output);
        assert!(p < bound, "{name}: peak={p:.4} > {bound}");
    }
}

#[test]
#[ignore] // Needs iteration count telemetry
fn solver_convergence_stats() {}

// ===========================================================================
// Level 5: Product Pedal Tests (feature-gated)
// ===========================================================================

#[cfg(feature = "pro-pedals")]
mod pro_pedals {
    use super::*;
    use pedalkernel::dsl::parse_pedal_file;
    use pedalkernel::PedalProcessor;

    fn pro_pedal_root() -> std::path::PathBuf {
        let crate_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
        // pedalkernel crate → pedalkernel repo → workspace root → pedalkernel-pro
        crate_dir.parent().unwrap().parent().unwrap().join("pedalkernel-pro")
    }

    fn find_all_pro_pedals() -> Vec<(String, String, std::path::PathBuf)> {
        let pro_root = pro_pedal_root();
        let search_dirs = [
            "pedals/patina", "pedals/legends",
            "pedals/real/chorus", "pedals/real/delay",
            "pedals/real/phaser", "pedals/real/vibrato",
            "pedals/real/overdrive", "pedals/real/distortion",
            "amps", "equipment",
        ];

        let mut pedals = Vec::new();
        for dir in &search_dirs {
            let dir_path = pro_root.join(dir);
            if let Ok(entries) = std::fs::read_dir(&dir_path) {
                for entry in entries.flatten() {
                    let path = entry.path();
                    if path.extension().map_or(false, |e| e == "pedal") {
                        if let Ok(src) = std::fs::read_to_string(&path) {
                            let name = path.file_name().unwrap().to_string_lossy().to_string();
                            pedals.push((name, src, path));
                        }
                    }
                }
            }
        }
        pedals.sort_by(|a, b| a.0.cmp(&b.0));
        pedals
    }

    #[test]
    fn all_pedals_pass_signal() {
        let fs = 48000.0;
        let input = sine_buf(440.0, fs, TEST_LEN * 2, 0.1);
        let pedals = find_all_pro_pedals();
        assert!(!pedals.is_empty(), "No pro pedals found at {}", pro_pedal_root().display());

        let mut passed = 0;
        let mut failed = Vec::new();

        for (name, src, _) in &pedals {
            let def = match parse_pedal_file(src) {
                Ok(d) => d,
                Err(e) => { eprintln!("  [skip] {name}: parse: {e}"); continue; }
            };
            let mut proc = match pedalkernel::compiler::compile_pedal(&def, fs) {
                Ok(p) => p,
                Err(e) => { eprintln!("  [skip] {name}: compile: {e}"); continue; }
            };
            for ctrl in &def.controls {
                proc.set_control(&ctrl.label, ctrl.default);
            }

            // Catch panics from engine bugs (e.g., overflow in delay element)
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
                let p = peak(&output[WARMUP..]);
                let all_finite = output[WARMUP..].iter().all(|x| x.is_finite());
                (all_finite, p)
            }));

            match result {
                Ok((all_finite, p)) if all_finite && p > 1e-6 => {
                    passed += 1;
                    eprintln!("  [pass] {name}: peak={p:.6}");
                }
                Ok((all_finite, p)) => {
                    failed.push(format!("{name}: finite={all_finite}, peak={p:.8}"));
                    eprintln!("  [FAIL] {name}: finite={all_finite}, peak={p:.8}");
                }
                Err(_) => {
                    failed.push(format!("{name}: panicked"));
                    eprintln!("  [FAIL] {name}: panicked during processing");
                }
            }
        }

        eprintln!("\n  === {passed}/{} pro pedals pass signal ===", passed + failed.len());
        let total = passed + failed.len();
        assert!(passed as f64 / total as f64 > 0.8,
            "Only {passed}/{total} pro pedals pass signal");
    }

    #[test]
    fn all_pots_affect_output() {
        let fs = 48000.0;
        let input = sine_buf(440.0, fs, TEST_LEN, 0.1);
        let pedals = find_all_pro_pedals();

        let mut pots_changed = 0;
        let mut pots_total = 0;

        for (name, src, _) in &pedals {
            let def = match parse_pedal_file(src) {
                Ok(d) => d,
                Err(_) => continue,
            };
            if def.controls.is_empty() { continue; }

            for ctrl in &def.controls {
                pots_total += 1;

                let (lo, hi) = if ctrl.range.0 < ctrl.range.1 {
                    (ctrl.range.0 + 0.05, ctrl.range.1 - 0.05)
                } else {
                    (ctrl.range.1 + 0.05, ctrl.range.0 - 0.05)
                };

                let controls_lo: Vec<(&str, f64)> = def.controls.iter()
                    .map(|c| (c.label.as_str(), if c.label == ctrl.label { lo } else { c.default }))
                    .collect();
                let controls_hi: Vec<(&str, f64)> = def.controls.iter()
                    .map(|c| (c.label.as_str(), if c.label == ctrl.label { hi } else { c.default }))
                    .collect();

                // Catch panics from engine bugs
                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    let out_lo = compile_and_process(src, &input, fs, &controls_lo);
                    let out_hi = compile_and_process(src, &input, fs, &controls_hi);
                    let corr = correlation(&out_lo[WARMUP..], &out_hi[WARMUP..]).abs();
                    let dist = spectral_distance(&out_lo[WARMUP..], &out_hi[WARMUP..], fs, 100.0);
                    (corr, dist)
                }));

                match result {
                    Ok((corr, dist)) => {
                        if corr < 0.999 || dist > 0.01 {
                            pots_changed += 1;
                        }
                        eprintln!("  [diag] {name}/{}: corr={corr:.4}, dist={dist:.4}", ctrl.label);
                    }
                    Err(_) => {
                        eprintln!("  [skip] {name}/{}: panicked", ctrl.label);
                    }
                }
            }
        }

        eprintln!("\n  === {pots_changed}/{pots_total} pots affect output ===");
        if pots_total > 0 {
            let ratio = pots_changed as f64 / pots_total as f64;
            assert!(ratio > 0.99,
                "Only {pots_changed}/{pots_total} pots affect output ({:.0}%)", ratio * 100.0);
        }
    }
}
