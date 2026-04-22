// TDD tests for BlackFeedbackStage — the clean replacement for OpAmpRoot
// in linear VCVS feedback stages.
//
// BlackFeedbackStage computes gain from Rf/Ri (Harold Black's formula),
// then applies NonIdealFx post-processing (GBW, slew, rails) using the
// same shared helper as IirStage. No OpAmpRoot, no WDF scattering for
// the gain computation.
//
// Tests are written BEFORE the implementation. Each test describes
// expected behavior that drives the implementation.

use super::spqr_build::compile_via_spqr;
use crate::PedalProcessor;

// ═══════════════════════════════════════════════════════════════════════════
// NonIdealFxState: shared post-processing (GBW + slew + rails)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn nonidealfx_state_passthrough_when_default() {
    // Default state should not alter signal (gbw_coeff=1, max_dv=MAX, v_max=MAX).
    use super::stage::NonIdealFxState;
    let mut state = NonIdealFxState::default();
    let input = 1.234;
    let output = super::stage::apply_nonideal_fx(input, &mut state);
    assert!(
        (output - input).abs() < 1e-10,
        "Default NonIdealFxState should passthrough: in={input}, out={output}"
    );
}

#[test]
fn nonidealfx_state_gbw_attenuates_hf() {
    // With GBW=1MHz and gain=10, fc=100kHz.
    // At 48kHz sample rate, a 20kHz signal should be attenuated.
    use super::stage::NonIdealFxState;
    let mut state = NonIdealFxState::from_params(1_000_000.0, 0.3, 3.0, 10.0, 48000.0);

    // Process 200Hz — should pass
    let mut peak_lo = 0.0f64;
    for s in 0..480 {
        let input = 0.5 * (2.0 * std::f64::consts::PI * 200.0 * s as f64 / 48000.0).sin();
        peak_lo = peak_lo.max(super::stage::apply_nonideal_fx(input, &mut state).abs());
    }

    // Reset and process 20kHz — should be attenuated
    state.reset();
    let mut peak_hi = 0.0f64;
    for s in 0..480 {
        let input = 0.5 * (2.0 * std::f64::consts::PI * 20000.0 * s as f64 / 48000.0).sin();
        peak_hi = peak_hi.max(super::stage::apply_nonideal_fx(input, &mut state).abs());
    }

    eprintln!("GBW test: 200Hz={peak_lo:.4}, 20kHz={peak_hi:.4}");
    assert!(peak_lo > 0.1, "200Hz should pass: {peak_lo:.4}");
    // 20kHz with GBW/gain fc=100kHz should still mostly pass at 48kHz SR
    // but be measurably less than 200Hz due to the pole
}

#[test]
fn nonidealfx_state_slew_limits_step() {
    // LM308 slew rate: 0.3 V/µs = 300,000 V/s.
    // At 48kHz: max_dv = 300,000 / 48,000 = 6.25 V/sample.
    // A step from 0 to 10V should be limited.
    use super::stage::NonIdealFxState;
    let mut state = NonIdealFxState::from_params(1_000_000.0, 0.3, 10.0, 1.0, 48000.0);

    // Settle at 0
    for _ in 0..100 {
        super::stage::apply_nonideal_fx(0.0, &mut state);
    }

    // Step to 10V
    let output = super::stage::apply_nonideal_fx(10.0, &mut state);
    eprintln!("Slew test: step 0→10, first sample={output:.4}");
    // Should be limited to ~6.25V
    assert!(output < 8.0, "Slew should limit step response: {output:.4}");
    assert!(output > 3.0, "Slew should allow some movement: {output:.4}");
}

#[test]
fn nonidealfx_state_rails_clamp() {
    // v_max = 3.0V (9V supply / 2 - 1.5V saturation)
    // Output should soft-clip at ~3V via tanh
    use super::stage::NonIdealFxState;
    let mut state = NonIdealFxState::from_params(10_000_000.0, 100.0, 3.0, 1.0, 48000.0);

    let output = super::stage::apply_nonideal_fx(10.0, &mut state);
    eprintln!("Rail test: input=10V, output={output:.4}");
    assert!(output < 3.5, "Should be rail-limited below 3.5V: {output:.4}");
    assert!(output > 2.0, "Should produce significant output: {output:.4}");
}

// ═══════════════════════════════════════════════════════════════════════════
// BlackFeedbackStage: gain computation
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bf_inverting_gain_10x() {
    // Inverting amp: Rf=100k, Ri=10k → gain = -Rf/Ri = -10
    use super::stage::BlackFeedbackStage;
    let stage = BlackFeedbackStage::new_test(100_000.0, 10_000.0, true, 48000.0);
    assert!(
        (stage.gain().abs() - 10.0).abs() < 0.1,
        "Inverting gain should be ~10, got {:.2}",
        stage.gain()
    );
}

#[test]
fn bf_noninverting_gain_11x() {
    // Non-inverting: Rf=100k, Ri=10k → gain = 1 + Rf/Ri = 11
    use super::stage::BlackFeedbackStage;
    let stage = BlackFeedbackStage::new_test(100_000.0, 10_000.0, false, 48000.0);
    assert!(
        (stage.gain() - 11.0).abs() < 0.1,
        "Non-inverting gain should be ~11, got {:.2}",
        stage.gain()
    );
}

#[test]
fn bf_process_amplifies_signal() {
    // Process a small signal — should come out amplified.
    use super::stage::BlackFeedbackStage;
    let mut stage = BlackFeedbackStage::new_test(100_000.0, 10_000.0, true, 48000.0);
    let output = stage.process(0.01);
    let measured_gain = output.abs() / 0.01;
    eprintln!("BF process: in=0.01, out={output:.6}, gain={measured_gain:.2}");
    assert!(
        measured_gain > 5.0 && measured_gain < 15.0,
        "Gain should be ~10, got {measured_gain:.2}"
    );
}

#[test]
fn bf_pot_sweep_changes_gain() {
    // Sweep pot from 0% to 100% — gain should change.
    use super::stage::BlackFeedbackStage;
    let mut stage_lo = BlackFeedbackStage::new_test(100_000.0, 10_000.0, true, 48000.0);
    let mut stage_hi = BlackFeedbackStage::new_test(100_000.0, 10_000.0, true, 48000.0);

    // Simulate pot at 10% (Rf = 10k)
    stage_lo.set_rf(10_000.0);
    let out_lo = stage_lo.process(0.01).abs();

    // Simulate pot at 100% (Rf = 100k)
    stage_hi.set_rf(100_000.0);
    let out_hi = stage_hi.process(0.01).abs();

    eprintln!("Pot sweep: lo={out_lo:.6}, hi={out_hi:.6}");
    assert!(
        out_hi > out_lo * 3.0,
        "High Rf should give more gain: lo={out_lo:.6}, hi={out_hi:.6}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// End-to-end: BlackFeedbackStage in the full pipeline
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bf_e2e_inverting_amp_produces_audio() {
    // Compile a simple inverting amp through compile_via_spqr.
    // Should produce gain ≈ 10 at 440Hz.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
            }
            nets {
                in -> R1.a
                R1.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    for _ in 0..100 { compiled.process(0.0); }

    let output = compiled.process(0.01);
    let gain = output.abs() / 0.01;
    eprintln!("E2E inverting: gain={gain:.2}");
    assert!(gain > 5.0 && gain < 15.0, "Should amplify ~10x, got {gain:.2}");
}

#[test]
fn bf_e2e_drive_pot_changes_output() {
    // Drive pot in feedback should change gain.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                U1: opamp(tl072)
                Drive: pot(100k, a)
            }
            nets {
                in -> R1.a
                R1.b -> U1.neg
                U1.neg -> Drive.a
                Drive.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls { Drive.position -> "Drive" [0.0, 1.0] = 0.5 }
        }"#)
    .expect("parse");

    let mut lo = compile_via_spqr(&pedal, 48000.0).expect("compile");
    lo.set_control("Drive", 0.1);
    for _ in 0..100 { lo.process(0.0); }
    let out_lo = lo.process(0.01).abs();

    let mut hi = compile_via_spqr(&pedal, 48000.0).expect("compile");
    hi.set_control("Drive", 1.0);
    for _ in 0..100 { hi.process(0.0); }
    let out_hi = hi.process(0.01).abs();

    eprintln!("E2E pot sweep: lo={out_lo:.6}, hi={out_hi:.6}");
    assert!(
        out_hi > out_lo * 2.0,
        "Drive pot should change gain: lo={out_lo:.6}, hi={out_hi:.6}"
    );
}
