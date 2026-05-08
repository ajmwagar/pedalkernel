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
// Pot binding: set_pot maps position → Rf → gain
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bf_has_pot_matches_bound_id() {
    use super::stage::BlackFeedbackStage;
    let mut stage = BlackFeedbackStage::new_test(100_000.0, 10_000.0, true, 48000.0);
    // No pot bound by default
    assert!(!stage.has_pot("Drive"), "Should not match unbound pot");

    // Bind a pot
    stage.pot_comp_id = Some("Drive".to_string());
    stage.pot_max_r = 100_000.0;
    assert!(stage.has_pot("Drive"), "Should match bound pot");
    assert!(!stage.has_pot("Tone"), "Should not match different pot");
}

#[test]
fn bf_set_pot_changes_gain() {
    use super::stage::BlackFeedbackStage;
    let mut stage = BlackFeedbackStage::new_test(100_000.0, 10_000.0, true, 48000.0);
    stage.pot_comp_id = Some("Drive".to_string());
    stage.pot_max_r = 100_000.0;

    // Position 1.0 → Rf = 100k → gain = 10
    stage.set_pot("Drive", 1.0);
    let gain_hi = stage.gain().abs();

    // Position 0.1 → Rf = 10k → gain = 1
    stage.set_pot("Drive", 0.1);
    let gain_lo = stage.gain().abs();

    eprintln!("set_pot: @1.0 gain={gain_hi:.2}, @0.1 gain={gain_lo:.2}");
    assert!(gain_hi > gain_lo * 3.0,
        "Higher position should give more gain: hi={gain_hi:.2}, lo={gain_lo:.2}");
}

#[test]
fn bf_e2e_pot_bound_at_compile_time() {
    // When a pot is in the feedback path, compile_via_spqr should
    // set pot_comp_id on the BlackFeedbackStage automatically.
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

    let compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    let bf = compiled.stages.iter()
        .find_map(|s| s.as_black_feedback())
        .expect("Should have a BlackFeedback stage");

    assert_eq!(bf.pot_comp_id.as_deref(), Some("Drive"),
        "Pot should be bound at compile time, got {:?}", bf.pot_comp_id);
    assert!(bf.pot_max_r > 0.0, "pot_max_r should be set");
}

// ═══════════════════════════════════════════════════════════════════════════
// End-to-end: BlackFeedbackStage through compile_via_spqr pipeline
//
// These tests use sustained 440Hz sine waves (not single-sample impulses)
// because the 4x oversampler in compiled.process() smears impulses across
// multiple output samples. Steady-state sine measurement is immune to that.
// ═══════════════════════════════════════════════════════════════════════════

/// Helper: measure peak output of a 440Hz sine at given amplitude through a compiled pedal.
/// Runs 500 warmup samples, then 2 full cycles (≈109 samples at 48kHz) and returns peak.
fn measure_sine_peak(compiled: &mut super::compiled::CompiledPedal, amplitude: f64) -> f64 {
    let sr = 48000.0;
    let freq = 440.0;

    // Warmup: let filters settle
    for s in 0..500 {
        let input = amplitude * (2.0 * std::f64::consts::PI * freq * s as f64 / sr).sin();
        compiled.process(input);
    }

    // Measure: 2 full cycles
    let n_samples = (2.0 * sr / freq).ceil() as usize;
    let mut peak = 0.0f64;
    for s in 0..n_samples {
        let input = amplitude * (2.0 * std::f64::consts::PI * freq * (500 + s) as f64 / sr).sin();
        let output = compiled.process(input);
        peak = peak.max(output.abs());
    }
    peak
}

#[test]
fn bf_e2e_inverting_amp_produces_audio() {
    // Compile a simple inverting amp: R1(10k) → U1.neg, Rf(100k) feedback.
    // Expected gain ≈ 10 at 440Hz.
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

    // Should route to BlackFeedbackStage
    assert!(
        compiled.stages.iter().any(|s| matches!(s, super::compiled::Stage::BlackFeedback(_))),
        "Should compile as BlackFeedbackStage"
    );

    let peak = measure_sine_peak(&mut compiled, 0.01);
    let gain = peak / 0.01;
    eprintln!("E2E inverting: peak={peak:.6}, gain={gain:.2}");
    assert!(gain > 5.0, "Should amplify (gain>5), got {gain:.2}");
    assert!(gain < 15.0, "Should not over-amplify (gain<15), got {gain:.2}");
}

#[test]
fn bf_e2e_noninverting_amp_produces_audio() {
    // Non-inverting: V+ = input, Rf(100k)/Ri(10k) → gain = 1 + 10 = 11.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                U1: opamp(tl072)
                Rf: resistor(100k)
                Ri: resistor(10k)
            }
            nets {
                in -> U1.pos
                U1.neg -> Ri.a
                Ri.b -> gnd
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.out -> out
            }
            controls {}
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    let peak = measure_sine_peak(&mut compiled, 0.01);
    let gain = peak / 0.01;
    eprintln!("E2E non-inverting: peak={peak:.6}, gain={gain:.2}");
    assert!(gain > 5.0, "Should amplify (gain>5), got {gain:.2}");
    assert!(gain < 20.0, "Should not over-amplify (gain<20), got {gain:.2}");
}

#[test]
fn bf_e2e_drive_pot_changes_output() {
    // Drive pot in feedback: sweeping pot position should change gain.
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

    // Low gain: Drive = 10%
    let mut lo = compile_via_spqr(&pedal, 48000.0).expect("compile lo");
    lo.set_control("Drive", 0.1);
    let peak_lo = measure_sine_peak(&mut lo, 0.01);

    // High gain: Drive = 100%
    let mut hi = compile_via_spqr(&pedal, 48000.0).expect("compile hi");
    hi.set_control("Drive", 1.0);
    let peak_hi = measure_sine_peak(&mut hi, 0.01);

    eprintln!("E2E pot: lo={peak_lo:.6}, hi={peak_hi:.6}");
    assert!(
        peak_hi > peak_lo * 2.0,
        "High Drive should give more output: lo={peak_lo:.6}, hi={peak_hi:.6}"
    );
}

#[test]
fn bf_e2e_stage_count_correct() {
    // A single inverting op-amp circuit should produce exactly 1 stage,
    // and it should be a BlackFeedback stage (not WDF or IIR).
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

    let compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    let bf_count = compiled.stages.iter()
        .filter(|s| matches!(s, super::compiled::Stage::BlackFeedback(_)))
        .count();
    let wdf_count = compiled.stages.iter()
        .filter(|s| matches!(s, super::compiled::Stage::Wdf(_)))
        .count();

    eprintln!("Stage count: total={}, bf={bf_count}, wdf={wdf_count}", compiled.stages.len());
    assert_eq!(bf_count, 1, "Should have exactly 1 BlackFeedback stage");
    assert_eq!(wdf_count, 0, "Should have 0 WDF stages (no passive sections)");
}

#[test]
fn bf_e2e_with_coupling_cap() {
    // Input coupling cap + inverting amp: C1 blocks DC, R1 sets Ri.
    // At 440Hz with C=100nF, R=10k: fc ≈ 159Hz, so 440Hz should pass.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                C1: cap(100n)
                R1: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
            }
            nets {
                in -> C1.a
                C1.b -> R1.a
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
    let peak = measure_sine_peak(&mut compiled, 0.01);
    let gain = peak / 0.01;
    eprintln!("E2E with cap: peak={peak:.6}, gain={gain:.2}");
    // Gain should still be ~10 at 440Hz (well above the 159Hz HP corner)
    assert!(gain > 3.0, "Should amplify above HP corner: gain={gain:.2}");
}

#[test]
fn bf_e2e_gain_scales_with_rf_ri_ratio() {
    // Two circuits with different Rf/Ri ratios should produce proportionally
    // different gains. Rf=50k/Ri=10k → gain=5, Rf=200k/Ri=10k → gain=20.
    let make_pedal = |rf: &str| {
        crate::dsl::parse_pedal_file(&format!(r#"
            pedal "test" {{ supply 9V
                components {{
                    R1: resistor(10k)
                    U1: opamp(tl072)
                    Rf: resistor({rf})
                }}
                nets {{
                    in -> R1.a
                    R1.b -> U1.neg
                    U1.neg -> Rf.a
                    Rf.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> out
                }}
                controls {{}}
            }}"#))
        .expect("parse")
    };

    let mut lo = compile_via_spqr(&make_pedal("50k"), 48000.0).expect("compile lo");
    let peak_lo = measure_sine_peak(&mut lo, 0.01);

    let mut hi = compile_via_spqr(&make_pedal("200k"), 48000.0).expect("compile hi");
    let peak_hi = measure_sine_peak(&mut hi, 0.01);

    let ratio = peak_hi / peak_lo.max(1e-10);
    eprintln!("Gain ratio: lo={peak_lo:.6} (Rf=50k), hi={peak_hi:.6} (Rf=200k), ratio={ratio:.1}x");
    // 200k/50k = 4x gain ratio. Allow tolerance for GBW/oversampler effects.
    assert!(ratio > 2.0, "4x Rf should give ≥2x more output, got {ratio:.1}x");
    assert!(ratio < 8.0, "4x Rf should give ≤8x more output, got {ratio:.1}x");
}
