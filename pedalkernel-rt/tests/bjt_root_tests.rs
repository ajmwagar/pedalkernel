//! TDD tests for BjtRoot — the single-port WDF BJT root.
//!
//! BjtRoot and BjtTwoPort must produce identical results for the same
//! model and operating point. BjtRoot is the single-port view (Vbe external,
//! Vce is the WDF port), BjtTwoPort is the 2-port view (both Vbe and Vce
//! are NR-solved simultaneously).
//!
//! For a given (Vbe, Vce), the collector current Ic must match.

use pedalkernel_rt::elements::*;
use pedalkernel_rt::elements::nonlinear::solver::{NlDeviceGroupIv, NlDeviceIv};

fn model_2n3904() -> GummelPoonModel {
    // Simplified 2N3904 NPN parameters
    GummelPoonModel {
        is: 6.734e-15,
        bf: 416.4,
        br: 0.7374,
        nf: 1.0,
        nr: 1.0,
        vt: 0.02585, // 25°C
        vaf: 74.03,
        var: 28.0,
        ikf: 0.06678,
        ikr: f64::INFINITY,
        ise: 6.734e-15,
        ne: 1.259,
        isc: 0.0,
        nc: 2.0,
        cje: 0.0,
        vje: 0.75,
        mje: 0.33,
        cjc: 0.0,
        vjc: 0.75,
        mjc: 0.33,
        rb: 0.0,
        re: 0.0,
        rc: 0.0,
        tf: 0.0,
        tr: 0.0,
        is_pnp: false,
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 1. BjtRoot I-V matches BjtTwoPort at identical operating points
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bjt_root_ic_matches_two_port_at_active_bias() {
    // Forward active: Vbe = 0.6V, Vce = 5V
    let model = model_2n3904();

    // BjtRoot: set Vbe, query Ic(Vce)
    let root = BjtRoot::new(model, false);
    let mut root_with_vbe = root.clone();
    root_with_vbe.set_vbe(0.6);
    let ic_root = root_with_vbe.collector_current(5.0);

    // BjtTwoPort: eval([Vbe, Vce]) → currents[1] = Ic
    let two_port = BjtTwoPort::new(model);
    let mut currents = [0.0; 2];
    let mut jacobian = [0.0; 4];
    two_port.eval(&[0.6, 5.0], &mut currents, &mut jacobian);
    let ic_two_port = currents[1];

    eprintln!("  Ic root={ic_root:.6e}, two_port={ic_two_port:.6e}");
    let diff = (ic_root - ic_two_port).abs();
    let rel = diff / ic_two_port.abs().max(1e-15);
    assert!(
        rel < 0.05,
        "BjtRoot Ic should match BjtTwoPort within 5%: root={ic_root:.6e}, two_port={ic_two_port:.6e}, rel={rel:.3}"
    );
}

#[test]
fn bjt_root_ic_matches_two_port_at_cutoff() {
    // Cutoff: Vbe = 0V, Vce = 5V — both should give ~0 Ic
    let model = model_2n3904();

    let root = BjtRoot::new(model, false);
    let mut root_with_vbe = root.clone();
    root_with_vbe.set_vbe(0.0);
    let ic_root = root_with_vbe.collector_current(5.0);

    let two_port = BjtTwoPort::new(model);
    let mut currents = [0.0; 2];
    let mut jacobian = [0.0; 4];
    two_port.eval(&[0.0, 5.0], &mut currents, &mut jacobian);
    let ic_two_port = currents[1];

    eprintln!("  Cutoff: root={ic_root:.6e}, two_port={ic_two_port:.6e}");
    assert!(ic_root.abs() < 1e-6, "Root Ic at cutoff should be ~0: {ic_root:.6e}");
    assert!(ic_two_port.abs() < 1e-6, "TwoPort Ic at cutoff should be ~0: {ic_two_port:.6e}");
}

#[test]
fn bjt_root_ic_matches_two_port_sweep() {
    // Sweep Vbe from 0 to 0.7V, check Ic matches at each point
    let model = model_2n3904();
    let two_port = BjtTwoPort::new(model);
    let vce = 5.0;

    let mut max_rel_error = 0.0f64;
    for i in 0..20 {
        let vbe = i as f64 * 0.035; // 0 to 0.665V

        let mut root = BjtRoot::new(model, false);
        root.set_vbe(vbe);
        let ic_root = root.collector_current(vce);

        let mut currents = [0.0; 2];
        let mut jacobian = [0.0; 4];
        two_port.eval(&[vbe, vce], &mut currents, &mut jacobian);
        let ic_two = currents[1];

        let denom = ic_two.abs().max(1e-12);
        let rel = (ic_root - ic_two).abs() / denom;
        max_rel_error = max_rel_error.max(rel);

        if ic_two.abs() > 1e-9 {
            assert!(
                rel < 0.1,
                "Vbe={vbe:.3}: root={ic_root:.6e} vs two_port={ic_two:.6e}, rel_err={rel:.3}"
            );
        }
    }
    eprintln!("  Sweep max relative error: {max_rel_error:.4}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. BjtRoot NR solve produces valid WDF reflected waves
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bjt_root_process_at_forward_active() {
    let model = model_2n3904();
    let mut root = BjtRoot::new(model, false);
    root.set_vbe(0.6);

    // Typical WDF port resistance for a common-emitter stage
    let rp = 1000.0;
    let a = 5.0; // incident wave (≈ B+ supply in wave domain)
    let b = root.process(a, rp);

    eprintln!("  Forward active: a={a:.3}, b={b:.3}, Vce=(a+b)/2={:.3}", (a + b) / 2.0);
    assert!(b.is_finite(), "Reflected wave should be finite");
    assert!(((a + b) as f64).abs() / 2.0 < 50.0, "Vce should be reasonable");
}

#[test]
fn bjt_root_process_produces_nonzero_output() {
    let model = model_2n3904();
    let mut root = BjtRoot::new(model, false);
    root.set_vbe(0.65); // Well into forward active

    let rp = 500.0;
    let a = 3.0;
    let b = root.process(a, rp);

    // With forward bias, collector current flows → reflected wave differs from incident
    let vce = (a + b) / 2.0;
    eprintln!("  Nonzero: a={a}, b={b:.4}, Vce={vce:.4}");
    assert!(
        (a - b).abs() > 0.001f64,
        "Reflected wave should differ from incident when BJT conducts: a={a}, b={b}"
    );
}

#[test]
fn bjt_root_process_no_nan_across_range() {
    let model = model_2n3904();
    let mut root = BjtRoot::new(model, false);
    let rp = 1000.0;

    for vbe_i in 0..10 {
        let vbe = vbe_i as f64 * 0.1 - 0.2; // -0.2 to 0.7V
        root.set_vbe(vbe);
        for a_i in 0..20 {
            let a = a_i as f64 * 2.0 - 10.0; // -10 to 28V
            let b = root.process(a, rp);
            assert!(
                b.is_finite(),
                "NaN/Inf at Vbe={vbe:.2}, a={a:.1}: b={b}"
            );
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. PNP polarity
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bjt_root_pnp_inverts_correctly() {
    let model = model_2n3904(); // Same model, PNP flag handled by BjtRoot
    let mut npn = BjtRoot::new(model, false);
    let mut pnp = BjtRoot::new(model, true);

    npn.set_vbe(0.6);
    pnp.set_vbe(-0.6); // PNP: negative Vbe

    let ic_npn = npn.collector_current(5.0);
    let ic_pnp = pnp.collector_current(-5.0);

    eprintln!("  NPN Ic={ic_npn:.6e}, PNP Ic={ic_pnp:.6e}");
    // PNP current should be opposite sign of NPN
    assert!(
        (ic_npn + ic_pnp).abs() < ic_npn.abs() * 0.1,
        "PNP Ic should be ~-NPN Ic: npn={ic_npn:.6e}, pnp={ic_pnp:.6e}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Diode-connected BJT (303 topology): Vbe ≈ Vce
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bjt_root_diode_connected_mode() {
    // In the 303 filter, base and collector are shorted.
    // This means Vbe ≈ voltage at emitter, and Vce ≈ Vbe (since Vbc ≈ 0).
    // The BJT acts as a diode: Ic ≈ Is * exp(Vbe/Vt)
    let model = model_2n3904();
    let mut root = BjtRoot::new(model, false);

    // Simulate diode-connected: set Vbe = Vce (base=collector)
    let vbe = 0.6;
    root.set_vbe(vbe);
    let ic = root.collector_current(vbe); // Vce = Vbe

    eprintln!("  Diode-connected: Vbe=Vce={vbe}, Ic={ic:.6e}");
    assert!(ic > 0.0, "Should conduct in diode mode: Ic={ic:.6e}");
    assert!(ic < 1.0, "Current should be reasonable (not blown up): Ic={ic:.6e}");
}

#[test]
fn bjt_root_diode_connected_monotonic() {
    // Ic should increase monotonically with Vbe (= Vce) in diode mode
    let model = model_2n3904();
    let mut root = BjtRoot::new(model, false);

    let mut prev_ic = -1.0;
    for i in 0..10 {
        let v = i as f64 * 0.08; // 0 to 0.72V
        root.set_vbe(v);
        let ic = root.collector_current(v);
        assert!(
            ic >= prev_ic - 1e-12,
            "Ic should be monotonically increasing: v={v:.2}, ic={ic:.6e}, prev={prev_ic:.6e}"
        );
        prev_ic = ic;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. NlDeviceIv trait implementation
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bjt_root_iv_trait_matches_direct() {
    let model = model_2n3904();
    let mut root = BjtRoot::new(model, false);
    root.set_vbe(0.6);

    let vce = 5.0;
    let (i_trait, di_trait) = root.iv(vce);
    let i_direct = root.collector_current(vce);
    let di_direct = root.collector_current_derivative(vce);

    assert!((i_trait - i_direct).abs() < 1e-15, "iv() current should match direct");
    assert!((di_trait - di_direct).abs() < 1e-15, "iv() derivative should match direct");
}
