// ═══════════════════════════════════════════════════════════════════════════
// MNA + NR Solver Accuracy Tests
//
// Comprehensive TDD for the MNA solver pipeline:
// 1. Wright Omega function accuracy
// 2. Diode I-V curve correctness (Shockley equation)
// 3. Diode clipping voltage accuracy (WO vs NR)
// 4. MNA signal injection and extraction
// 5. MNA scattering matrix correctness
// 6. Full circuit transfer functions
//
// References:
// - Fettweis (1986): Wave Digital Filters — scattering formulation
// - Werner (2016): Improved NR for WDF nonlinear elements
// - D'Angelo & Välimäki (2013): Wright Omega for diode circuits
// ═══════════════════════════════════════════════════════════════════════════

use super::spqr_build::compile_via_spqr;
use crate::PedalProcessor;

const SR: f64 = 48000.0;

// ═══════════════════════════════════════════════════════════════════════════
// 1. Wright Omega accuracy
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn wright_omega_satisfies_defining_equation() {
    use crate::elements::nonlinear::diode::wright_omega;

    let test_points = [-30.0, -10.0, -5.0, -1.0, 0.0, 1.0, 5.0, 10.0, 50.0, 100.0];
    for x in test_points {
        let w = wright_omega(x);
        if w > 1e-15 {
            let residual = (w + w.ln() - x).abs();
            assert!(residual < 1e-8,
                "Wright Omega residual at x={x}: w={w:.10e}, residual={residual:.2e}");
        }
    }
}

#[test]
fn wright_omega_known_values() {
    use crate::elements::nonlinear::diode::wright_omega;

    // ω(0) ≈ 0.5671 (the Omega constant)
    let w0 = wright_omega(0.0);
    assert!((w0 - 0.5671).abs() < 0.001, "ω(0) ≈ 0.5671: got {w0:.6}");

    // ω(1) = 1.0 (1 + ln(1) = 1)
    let w1 = wright_omega(1.0);
    assert!((w1 - 1.0).abs() < 1e-8, "ω(1) = 1.0: got {w1:.10}");

    // ω(e+1) = e (e + ln(e) = e + 1)
    let we1 = wright_omega(std::f64::consts::E + 1.0);
    assert!((we1 - std::f64::consts::E).abs() < 1e-6, "ω(e+1) = e: got {we1:.6}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Explicit diode solver accuracy (Wright Omega path)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn explicit_diode_pair_clips_at_correct_voltage() {
    use crate::elements::nonlinear::diode::explicit_diode_pair;

    let is: f64 = 2.52e-9; // Silicon
    let n_vt: f64 = 1.752 * 25.85e-3;
    let rp: f64 = 10000.0;

    // Drive with 2.0V incident wave (would give 1.0V without clipping)
    let v = explicit_diode_pair(2.0, rp, is, n_vt);
    eprintln!("Explicit diode pair: a=2.0, v={v:.4}");
    assert!(v.abs() < 0.8, "Should clip below 0.8V: {v:.4}");
    assert!(v.abs() > 0.3, "Should produce some output: {v:.4}");
}

#[test]
fn explicit_single_diode_clips_asymmetrically() {
    use crate::elements::nonlinear::diode::explicit_single_diode;

    let is: f64 = 2.52e-9;
    let n_vt: f64 = 1.752 * 25.85e-3;
    let rp: f64 = 10000.0;

    let v_fwd = explicit_single_diode(2.0, rp, is, n_vt);
    let v_rev = explicit_single_diode(-2.0, rp, is, n_vt);

    eprintln!("Single diode: fwd={v_fwd:.4}, rev={v_rev:.4}");
    assert!(v_fwd.abs() < v_rev.abs(),
        "Forward should clip more than reverse: fwd={v_fwd:.4}, rev={v_rev:.4}");
}

#[test]
fn explicit_germanium_clips_lower_than_silicon() {
    use crate::elements::nonlinear::diode::explicit_diode_pair;

    let si_is: f64 = 2.52e-9;
    let si_nvt: f64 = 1.752 * 25.85e-3;
    let ge_is: f64 = 2.0e-6;
    let ge_nvt: f64 = 1.25 * 25.85e-3;
    let rp: f64 = 10000.0;

    let si_clip = explicit_diode_pair(2.0, rp, si_is, si_nvt).abs();
    let ge_clip = explicit_diode_pair(2.0, rp, ge_is, ge_nvt).abs();

    eprintln!("Clip levels: Si={si_clip:.4}V, Ge={ge_clip:.4}V");
    assert!(ge_clip < si_clip, "Ge should clip lower than Si: Ge={ge_clip:.4} < Si={si_clip:.4}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Diode I-V curve correctness (pure math)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn silicon_diode_forward_voltage_correct() {
    // Silicon 1N4148: Vf ≈ 0.6V at 1mA.
    // Shockley: I = Is * (exp(V/nVt) - 1)
    // At I=1mA: V = nVt * ln(I/Is + 1)
    let is: f64 = 2.52e-9;
    let n_vt: f64 = 1.752 * 25.85e-3;
    let i_target: f64 = 1e-3;
    let v_expected = n_vt * (i_target / is + 1.0_f64).ln();

    eprintln!("Silicon Vf at 1mA: {v_expected:.4}V");
    assert!(v_expected > 0.5 && v_expected < 0.8,
        "Silicon Vf should be 0.5-0.8V at 1mA: got {v_expected:.4}V");
}

#[test]
fn germanium_diode_lower_vf_than_silicon() {
    let si_is: f64 = 2.52e-9;
    let si_nvt: f64 = 1.752 * 25.85e-3;
    let ge_is: f64 = 2.0e-6;
    let ge_nvt: f64 = 1.25 * 25.85e-3;

    let i: f64 = 1e-3;
    let si_vf = si_nvt * (i / si_is + 1.0_f64).ln();
    let ge_vf = ge_nvt * (i / ge_is + 1.0_f64).ln();

    eprintln!("Si Vf={si_vf:.4}V, Ge Vf={ge_vf:.4}V");
    assert!(ge_vf < si_vf, "Germanium Vf should be lower than silicon: Ge={ge_vf:.4} < Si={si_vf:.4}");
    assert!(ge_vf > 0.1 && ge_vf < 0.4, "Germanium Vf should be 0.1-0.4V: {ge_vf:.4}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Diode clipping through the compiled pipeline
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
// 4. MNA signal injection and extraction
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn mna_single_stage_injects_at_circuit_input() {
    // Single-stage circuit: MNA should inject at graph.in_node.
    // R_in → U1.neg → Rf → U1.out → D1 → U1.neg.
    // Output at U1.out. Signal enters at in_node (graph.in_node).
    let gain = measure_gain(r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D1: diode(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.neg -> D1.a
                D1.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#);
    eprintln!("MNA single stage: gain={gain:.2}");
    assert!(gain > 3.0, "Single stage should amplify: {gain:.2}");
}

#[test]
fn mna_multi_stage_injects_at_vcvs_input() {
    // Multi-stage: Cin + R_in → clipping stage.
    // MNA should inject at the VCVS input pin (neg node), not graph.in_node.
    let gain = measure_gain(r#"
        pedal "test" { supply 9V
            components {
                Cin: cap(47n)
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D1: diode(silicon)
            }
            nets {
                in -> Cin.a
                Cin.b -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.neg -> D1.a
                D1.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#);
    eprintln!("MNA multi-stage injection: gain={gain:.2}");
    assert!(gain > 1.0, "Multi-stage should amplify: {gain:.2}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. MNA output extraction correctness
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn mna_output_at_opamp_output_not_virtual_ground() {
    // The output should be at the op-amp output pin, which swings
    // with the signal. NOT at the virtual ground (neg node ≈ 0V).
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D1: diode(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.neg -> D1.a
                D1.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");
    for s in 0..500 {
        compiled.process(0.01 * (std::f64::consts::TAU * 440.0 * s as f64 / SR).sin());
    }

    // Measure over one full cycle
    let mut peak = 0.0f64;
    for s in 500..610 {
        let input = 0.01 * (std::f64::consts::TAU * 440.0 * s as f64 / SR).sin();
        peak = peak.max(compiled.process(input).abs());
    }

    eprintln!("MNA output extraction: peak={peak:.4}V");
    // With Rf/Ri=10 and 0.01V input, output should be ~0.1V (below clip)
    // Virtual ground would give ~0V. Op-amp output gives ~0.1V.
    assert!(peak > 0.02, "Output should be at op-amp output, not virtual ground: {peak:.4}V");
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. Full circuit transfer functions
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn mna_inverting_amp_with_diode_gain_correct() {
    // Inverting amp with diode: below clipping, gain ≈ Rf/Ri = 10.
    // Use small signal to stay below diode Vf.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D1: diode(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.neg -> D1.a
                D1.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Very small signal: 1mV → should be 10mV (below 0.6V clip)
    for s in 0..1000 {
        let input = 0.001 * (std::f64::consts::TAU * 440.0 * s as f64 / SR).sin();
        compiled.process(input);
    }
    let mut peak = 0.0f64;
    for s in 1000..1110 {
        let input = 0.001 * (std::f64::consts::TAU * 440.0 * s as f64 / SR).sin();
        peak = peak.max(compiled.process(input).abs());
    }

    let gain = peak / 0.001;
    eprintln!("MNA small-signal gain: {gain:.2} (expected ~10)");
    assert!(gain > 5.0, "Small-signal gain should be ~Rf/Ri=10: {gain:.2}");
    assert!(gain < 20.0, "Gain shouldn't be excessive: {gain:.2}");
}

#[test]
fn mna_diode_pair_clips_symmetrically() {
    // DiodePair in feedback: positive and negative peaks should be
    // roughly equal (symmetric clipping).
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(500k)
                D1: diode(silicon)
                D2: diode(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.neg -> D1.a
                D1.b -> U1.out
                U1.neg -> D2.b
                D2.a -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Drive hard to clip
    for s in 0..1000 {
        let input = 0.05 * (std::f64::consts::TAU * 440.0 * s as f64 / SR).sin();
        compiled.process(input);
    }
    let mut pos_peak = 0.0f64;
    let mut neg_peak = 0.0f64;
    for s in 1000..1110 {
        let input = 0.05 * (std::f64::consts::TAU * 440.0 * s as f64 / SR).sin();
        let out = compiled.process(input);
        pos_peak = pos_peak.max(out);
        neg_peak = neg_peak.min(out);
    }

    eprintln!("Symmetric clip: pos={pos_peak:.4}, neg={neg_peak:.4}");
    assert!(pos_peak > 0.1, "Should produce positive output: {pos_peak:.4}");
    assert!(neg_peak < -0.1, "Should produce negative output: {neg_peak:.4}");

    let asymmetry = (pos_peak - neg_peak.abs()).abs() / pos_peak.max(0.001);
    assert!(asymmetry < 0.3, "Should be roughly symmetric: asymmetry={asymmetry:.2}");
}

#[test]
fn mna_asymmetric_diodes_different_clip_levels() {
    // Ge forward + Si reverse: should clip asymmetrically.
    // Ge clips at ~0.3V, Si clips at ~0.6V.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D_ge: diode(germanium)
                D_si: diode(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.neg -> D_ge.a
                D_ge.b -> U1.out
                U1.neg -> D_si.b
                D_si.a -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    for s in 0..1000 {
        let input = 0.05 * (std::f64::consts::TAU * 440.0 * s as f64 / SR).sin();
        compiled.process(input);
    }
    let mut pos_peak = 0.0f64;
    let mut neg_peak = 0.0f64;
    for s in 1000..1110 {
        let input = 0.05 * (std::f64::consts::TAU * 440.0 * s as f64 / SR).sin();
        let out = compiled.process(input);
        pos_peak = pos_peak.max(out);
        neg_peak = neg_peak.min(out);
    }

    eprintln!("Asymmetric clip: pos={pos_peak:.4} (Ge), neg={neg_peak:.4} (Si)");
    assert!(pos_peak > 0.05, "Should produce positive output: {pos_peak:.4}");
    assert!(neg_peak < -0.05, "Should produce negative output: {neg_peak:.4}");

    // Asymmetry: Ge clips lower than Si
    let diff = (pos_peak.abs() - neg_peak.abs()).abs();
    eprintln!("Clip level difference: {diff:.4}V");
    assert!(diff > 0.05, "Ge and Si should clip at different levels: diff={diff:.4}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 7. Helpers
// ═══════════════════════════════════════════════════════════════════════════

fn measure_gain(pedal_src: &str) -> f64 {
    let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    let amplitude = 0.01;
    let freq = 440.0;

    for s in 0..1000 {
        let input = amplitude * (std::f64::consts::TAU * freq * s as f64 / SR).sin();
        compiled.process(input);
    }
    let n = (2.0 * SR / freq).ceil() as usize;
    let mut peak = 0.0f64;
    for s in 0..n {
        let input = amplitude * (std::f64::consts::TAU * freq * (1000 + s) as f64 / SR).sin();
        peak = peak.max(compiled.process(input).abs());
    }
    peak / amplitude
}
