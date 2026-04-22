// TDD tests for OutputImpedance-driven stage splitting.
//
// The Component trait's output_impedance() method declares whether a
// component's output is a voltage source (safe split point) or finite
// impedance (splitting would change the circuit).
//
// These tests verify:
// 1. Components declare the correct output impedance
// 2. Split points are correctly identified in circuit graphs
// 3. Passive components between split points are grouped together
// 4. The HPF regression is fixed (standalone caps no longer act as high-pass)

use super::component::{Component, OutputImpedance};
use super::spqr_build::compile_via_spqr;
use crate::PedalProcessor;

// ═══════════════════════════════════════════════════════════════════════════
// Layer 1: Component trait — output_impedance() declarations
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn opamp_output_is_voltage_source() {
    use super::components::OpAmp;
    use crate::dsl::OpAmpType;

    let tl072 = OpAmp { op_type: OpAmpType::Tl072 };
    assert_eq!(tl072.output_impedance(), OutputImpedance::VoltageSource,
        "Op-amp output is a voltage source (zero impedance via feedback)");

    let lm308 = OpAmp { op_type: OpAmpType::Lm308 };
    assert_eq!(lm308.output_impedance(), OutputImpedance::VoltageSource);

    let jrc4558 = OpAmp { op_type: OpAmpType::Jrc4558 };
    assert_eq!(jrc4558.output_impedance(), OutputImpedance::VoltageSource);
}

#[test]
fn ota_output_is_finite() {
    use super::components::OpAmp;
    use crate::dsl::OpAmpType;

    let ca3080 = OpAmp { op_type: OpAmpType::Ca3080 };
    assert_eq!(ca3080.output_impedance(), OutputImpedance::Finite,
        "OTA output is a current source (high impedance)");
}

#[test]
fn passive_components_are_finite() {
    use super::components::{Resistor, Capacitor, Inductor};
    use crate::dsl::CapConfig;

    let r = Resistor { value: 10_000.0 };
    assert_eq!(r.output_impedance(), OutputImpedance::Finite);

    let c = Capacitor { config: CapConfig::new(100e-9) };
    assert_eq!(c.output_impedance(), OutputImpedance::Finite);

    let l = Inductor { value: 0.1 };
    assert_eq!(l.output_impedance(), OutputImpedance::Finite);
}

#[test]
fn diode_is_finite() {
    use super::components::Diode;
    use crate::dsl::DiodeType;

    let d = Diode { diode_type: DiodeType::Silicon };
    assert_eq!(d.output_impedance(), OutputImpedance::Finite);
}

#[test]
fn bjt_is_finite() {
    use super::components::Npn;

    let q = Npn { model: "2n3904".to_string() };
    assert_eq!(q.output_impedance(), OutputImpedance::Finite,
        "BJT collector is a current source (high impedance)");
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 2: Stage grouping — passives between split points stay together
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn rat_has_few_stages_not_many() {
    // The RAT should compile to ~3-5 stages, not 16+.
    // Without proper passive grouping, each cap/resistor becomes its own stage.
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/ratking_non_invert_v1a.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read ratking");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    let total_stages = compiled.stage_order.len();
    eprintln!("RAT v1a: {} total stages (wdf={}, iir={}, ss={}, mnl={})",
        total_stages, compiled.stages.len(), compiled.iir_stages.len(),
        compiled.state_space_stages.len(), compiled.multi_nl_stages.len());

    // With proper splitting at op-amp outputs, should be ~3-5 stages:
    // 1. Input network (C1, R1, R2) as passive WDF or IIR
    // 2. Op-amp gain stage (BlackFeedback with Rf, caps)
    // 3. Diode clippers (D1, D2 to ground)
    // 4. Tone/volume network
    assert!(
        total_stages <= 8,
        "RAT should have ≤8 stages, got {total_stages} (passive components over-split)"
    );
}

#[test]
fn screamer_has_few_stages() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/screamer.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read screamer");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    let total_stages = compiled.stage_order.len();
    eprintln!("Screamer: {} total stages", total_stages);

    assert!(
        total_stages <= 8,
        "Screamer should have ≤8 stages, got {total_stages}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 3: No HPF — low frequencies must pass through
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn rat_no_hpf_regression() {
    // A distortion pedal should not have massively more output at 5kHz vs 200Hz.
    // The HPF regression occurs when standalone caps act as high-pass filters.
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/ratking_non_invert_v1a.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read ratking");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");

    // 200Hz
    let mut lo = compile_via_spqr(&pedal, 48000.0).expect("compile");
    for _ in 0..2000 { lo.process(0.0); }
    let mut peak_lo = 0.0f64;
    for s in 0..4800 {
        let input = 0.05 * (2.0 * std::f64::consts::PI * 200.0 * s as f64 / 48000.0).sin();
        peak_lo = peak_lo.max(lo.process(input).abs());
    }

    // 5kHz
    let mut hi = compile_via_spqr(&pedal, 48000.0).expect("compile");
    for _ in 0..2000 { hi.process(0.0); }
    let mut peak_hi = 0.0f64;
    for s in 0..4800 {
        let input = 0.05 * (2.0 * std::f64::consts::PI * 5000.0 * s as f64 / 48000.0).sin();
        peak_hi = peak_hi.max(hi.process(input).abs());
    }

    let ratio = if peak_lo > 1e-10 { peak_hi / peak_lo } else { f64::INFINITY };
    eprintln!("RAT HPF check: 200Hz={peak_lo:.4}, 5kHz={peak_hi:.4}, ratio={ratio:.1}x");

    assert!(peak_lo > 0.001, "200Hz should produce audible output: {peak_lo:.6}");
    assert!(ratio < 10.0, "HPF regression: 5kHz/200Hz={ratio:.1}x (should be <10x)");
}

#[test]
fn screamer_no_hpf_regression() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/screamer.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read screamer");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");

    let mut lo = compile_via_spqr(&pedal, 48000.0).expect("compile");
    for _ in 0..2000 { lo.process(0.0); }
    let mut peak_lo = 0.0f64;
    for s in 0..4800 {
        let input = 0.05 * (2.0 * std::f64::consts::PI * 200.0 * s as f64 / 48000.0).sin();
        peak_lo = peak_lo.max(lo.process(input).abs());
    }

    let mut hi = compile_via_spqr(&pedal, 48000.0).expect("compile");
    for _ in 0..2000 { hi.process(0.0); }
    let mut peak_hi = 0.0f64;
    for s in 0..4800 {
        let input = 0.05 * (2.0 * std::f64::consts::PI * 5000.0 * s as f64 / 48000.0).sin();
        peak_hi = peak_hi.max(hi.process(input).abs());
    }

    let ratio = if peak_lo > 1e-10 { peak_hi / peak_lo } else { f64::INFINITY };
    eprintln!("Screamer HPF check: 200Hz={peak_lo:.4}, 5kHz={peak_hi:.4}, ratio={ratio:.1}x");

    assert!(peak_lo > 0.001, "200Hz should produce audible output: {peak_lo:.6}");
    assert!(ratio < 10.0, "HPF regression: 5kHz/200Hz={ratio:.1}x (should be <10x)");
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 4: Simple circuit — verify correct grouping
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn simple_rc_opamp_groups_input_with_stage() {
    // C1 → R1 → U1.neg [feedback Rf] → U1.out → R_out → out
    // C1 and R1 should be in the same stage as U1 (or at least grouped
    // together as a single passive stage), not two separate stages.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                C1: cap(100n)
                R1: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                R_out: resistor(10k)
            }
            nets {
                in -> C1.a
                C1.b -> R1.a
                R1.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> R_out.a
                R_out.b -> out
            }
            controls {}
        }"#)
    .expect("parse");

    let compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    let total = compiled.stage_order.len();
    eprintln!("Simple RC+opamp: {total} stages");

    // Should be ≤3 stages: input network, opamp, output
    // NOT 5 stages (C1, R1, opamp, Rf, R_out each separate)
    assert!(total <= 3, "Should be ≤3 stages, got {total}");
}
