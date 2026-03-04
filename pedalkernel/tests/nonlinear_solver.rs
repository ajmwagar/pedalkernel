//! Nonlinear solver validation: diode roots respond smoothly and with the
//! expected symmetry/monotonic behavior as input excitation changes.

use pedalkernel::elements::{DiodeModel, DiodePairRoot, DiodeRoot, WdfRoot};

#[test]
fn diode_pair_voltage_is_monotonic() {
    let mut root = DiodePairRoot::new(DiodeModel::silicon());
    let rp = 1000.0;
    let mut last_v = None;
    for a in (0..=200).map(|n| n as f64 * 0.05) {
        let b = root.process(a, rp);
        let v = 0.5 * (a + b);
        if let Some(prev) = last_v {
            // TOLERANCE SOURCE: NR solver convergence tolerance is 1e-10 (voltage).
            // Monotonicity violation > 1e-3 would be audible as ~-60dBFS crackle.
            // 1e-3 is the perceptual threshold for single-sample discontinuity
            // at guitar signal levels (0.1–1V peak). Physical diode I-V is strictly
            // monotonic, so any violation indicates a solver regression.
            assert!(v >= prev - 1e-3, "Diode pair should increase with input");
        }
        last_v = Some(v);
    }
}

#[test]
fn diode_pair_is_odd_symmetric() {
    let mut root = DiodePairRoot::new(DiodeModel::silicon());
    let rp = 2200.0;
    for a in (-100..=100).map(|n| n as f64 * 0.1) {
        let b_pos = root.process(a, rp);
        let b_neg = root.process(-a, rp);
        let v_pos = 0.5 * (a + b_pos);
        let v_neg = 0.5 * (-a + b_neg);
        // TOLERANCE SOURCE: Anti-parallel diode pair has exact odd symmetry.
        // Residual asymmetry is bounded by NR tolerance (1e-10) * Jacobian
        // condition number (~1e3 for Rp=2200Ω silicon). Expected: < 1e-7.
        // 1e-6 provides 10x margin.
        assert!(
            (v_pos + v_neg).abs() < 1e-6,
            "Odd symmetry violated at a={a}: v+={v_pos}, v-={v_neg}"
        );
    }
}

#[test]
fn single_diode_current_follows_input_polarity() {
    let mut root = DiodeRoot::new(DiodeModel::silicon());
    for rp in [220.0, 1000.0, 8200.0] {
        for a in (-20..=20).map(|n| n as f64 * 0.25) {
            let b = root.process(a, rp);
            let i = (a - b) / (2.0 * rp);
            if a > 1e-6 {
                assert!(
                    i >= -1e-9,
                    "Forward bias should yield positive current: a={a}, i={i}"
                );
            } else if a < -1e-6 {
                assert!(
                    i <= 1e-9,
                    "Reverse bias should yield negative current: a={a}, i={i}"
                );
            }
        }
    }
}

#[test]
fn diode_pair_response_is_continuous() {
    let mut root = DiodePairRoot::new(DiodeModel::silicon());
    let rp = 1000.0;
    let mut last_v: Option<f64> = None;
    for a in (-200..=200).map(|n| n as f64 * 0.01) {
        let b = root.process(a, rp);
        let v = 0.5 * (a + b);
        if let Some(prev) = last_v {
            let delta = (v - prev).abs();
            // TOLERANCE SOURCE: Diode I-V is C∞ smooth. For step Δa = 0.01V
            // through Rp=1000Ω, max Δv ≈ Δa * Rp/(Rp + rd) where rd ≈ 25mV/Is.
            // At forward bias rd → 25Ω, so max Δv ≈ 0.01 * 1000/1025 ≈ 0.01V.
            // 0.05V tolerance provides 5x margin for all bias points.
            assert!(
                delta < 0.05,
                "Diode pair output jumped: Δv={delta} at a={a}"
            );
        }
        last_v = Some(v);
    }
}
