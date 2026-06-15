//! Voltage-driven Jiles-Atherton tape head (`TapeHeadVoltageRoot`) tests.
//!
//! Unlike the current-driven transformer core, the tape head drives the
//! magnetic field H from PORT VOLTAGE, so saturation tracks signal level at
//! line drive. These tests verify the shipped port law is:
//!
//!   * correct + saturating — THD rises with input amplitude (clean at ~0.1 V,
//!     saturated at ~1 V),
//!   * stable — Newton converges (< MAX_ITERS) across amplitude + frequency
//!     sweeps, output stays finite, no oscillation on sustained drive,
//!   * resettable — reset() clears the magnetic state.

use pedalkernel_rt::elements::{JaCoreModel, TapeHeadVoltageRoot};

const FS: f64 = 48_000.0;
const PI: f64 = std::f64::consts::PI;

/// Record/playback head model (see `tape_head` DSL component). Same J-A
/// hysteresis physics as the transformer core, but a small-geometry head:
/// the field is driven from voltage (kv) so the knee lands at line level.
fn head_model() -> JaCoreModel {
    JaCoreModel {
        ms: 3.5e5,
        a: 500.0,
        alpha: 1.6e-3,
        k: 120.0,
        c: 0.3,
        // n_turns/area/path_len/gap are unused by the voltage law's H = kv*V,
        // but is_complete() requires sane positives.
        n_turns: 100.0,
        area: 1.0e-6,
        path_len: 1.0e-3,
        gap: 0.0,
    }
}

// Field-per-volt and current scales chosen so the knee sits near ~1 V.
const KV: f64 = 1000.0; // A/m per V → H ≈ 1000 at 1 V ≈ a few * a
const ISAT: f64 = 5.0e-3; // saturating magnetic current scale (A)
const GP: f64 = 1.0e-3; // linear leakage (S)
const RP: f64 = 1_000.0; // port resistance (Ω)

// h_bias = 0 here keeps the saturator symmetric so reset() returns M exactly to
// 0 (the DSL studio head uses a nonzero bias for even-harmonic tape colour;
// that path is exercised by the circuit-level tape_saturation tests).
const H_BIAS: f64 = 0.0;

fn new_head() -> TapeHeadVoltageRoot {
    let mut root = TapeHeadVoltageRoot::new(head_model(), KV, ISAT, GP, H_BIAS);
    root.prepare(FS, RP);
    root
}

/// Drive a sine of wave-amplitude `amp` (volts), return port voltages V=(a+b)/2
/// over the last `keep` samples plus the peak Newton iteration count.
fn drive(root: &mut TapeHeadVoltageRoot, amp: f64, f0: f64, periods: usize) -> (Vec<f64>, usize) {
    let per = (FS / f0).round() as usize;
    let n = per * periods;
    let mut vs = Vec::with_capacity(n);
    let mut max_iters = 0usize;
    for k in 0..n {
        let a = amp * (2.0 * PI * f0 * k as f64 / FS).sin();
        let b = root.process(a);
        max_iters = max_iters.max(root.last_iters);
        vs.push(0.5 * (a + b));
    }
    // Keep the last 4 periods (steady state).
    let keep = per * 4;
    let start = vs.len().saturating_sub(keep);
    (vs[start..].to_vec(), max_iters)
}

/// Goertzel magnitude at frequency `f` of signal `w`.
fn goertzel(w: &[f64], f: f64) -> f64 {
    let n = w.len() as f64;
    let k = (0.5 + n * f / FS).floor();
    let omega = 2.0 * PI * k / n;
    let coeff = 2.0 * omega.cos();
    let (mut s0, mut s1, mut s2) = (0.0, 0.0, 0.0);
    for &x in w {
        s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    (s1 * s1 + s2 * s2 - coeff * s1 * s2).max(0.0).sqrt()
}

/// THD = sqrt(sum h2..h8^2) / h1 from the steady-state port voltage.
fn thd(w: &[f64], f0: f64) -> f64 {
    let fund = goertzel(w, f0);
    if fund <= 1.0e-18 {
        return 0.0;
    }
    let mut h = 0.0;
    for k in 2..=8 {
        let m = goertzel(w, k as f64 * f0);
        h += m * m;
    }
    h.sqrt() / fund
}

#[test]
fn tape_head_thd_rises_with_amplitude() {
    let f0 = 120.0;
    let mut curve = Vec::new();
    for &amp in &[0.05, 0.1, 0.25, 0.5, 1.0, 2.0] {
        let mut root = new_head();
        let (w, _) = drive(&mut root, amp, f0, 12);
        let t = thd(&w, f0);
        println!("[tape_head] amp={amp:>4} V : THD={t:.5}");
        curve.push((amp, t));
    }

    // Clean at low level, dirty when driven hard.
    let low = curve[0].1; // 0.05 V
    let high = curve.last().unwrap().1; // 2.0 V
    assert!(
        low < 0.01,
        "low-level THD should be clean (<1%), got {low:.5}"
    );
    assert!(
        high > 0.05,
        "hard-driven THD should saturate (>5%), got {high:.5}"
    );
    // Monotone-ish rise overall (allow tiny non-monotone wiggle from Goertzel
    // leakage but require a clear net climb).
    assert!(
        high > low * 5.0,
        "THD must climb with drive: {low} -> {high}"
    );
}

#[test]
fn tape_head_newton_converges_across_sweeps() {
    for &f0 in &[40.0, 120.0, 500.0, 2000.0, 8000.0] {
        for &amp in &[0.05, 0.5, 1.0, 3.0] {
            let mut root = new_head();
            let (w, iters) = drive(&mut root, amp, f0, 8);
            assert!(
                iters < 24,
                "Newton hit MAX_ITERS at f={f0} amp={amp} (iters={iters})"
            );
            assert!(
                w.iter().all(|x| x.is_finite()),
                "non-finite output at f={f0} amp={amp}"
            );
        }
    }
}

#[test]
fn tape_head_reset_clears_state() {
    let mut root = new_head();
    let _ = drive(&mut root, 2.0, 120.0, 12);
    assert!(root.magnetization().abs() > 0.0, "should have magnetized");
    root.reset();
    assert_eq!(root.magnetization(), 0.0, "reset must clear M");
    // First sample after reset must be finite and well-behaved.
    let b = root.process(0.1);
    assert!(b.is_finite());
}

#[test]
fn tape_head_no_oscillation_on_sustained_dc_drive() {
    // Sustained large DC-ish drive must settle, not ring.
    let mut root = new_head();
    let mut last = 0.0;
    for k in 0..4000 {
        let b = root.process(1.5);
        assert!(b.is_finite());
        if k > 3990 {
            // Steady state: successive outputs nearly identical.
            assert!(
                (b - last).abs() < 1.0e-6,
                "output oscillating at steady DC: {b} vs {last}"
            );
        }
        last = b;
    }
}

#[test]
fn tape_head_degenerate_config_is_passive() {
    // isat = 0 (no magnetic term) → behaves as the reflection-free fallback.
    let mut root = TapeHeadVoltageRoot::new(head_model(), KV, 0.0, GP, H_BIAS);
    root.prepare(FS, RP);
    let b = root.process(0.5);
    assert_eq!(b, -0.5);
}
