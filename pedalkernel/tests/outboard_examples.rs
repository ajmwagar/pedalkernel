//! Integration tests for the outboard studio gear examples in
//! `examples/outboard/` — classic compressor and EQ circuits.
//!
//! Each test compiles the example, runs ~1 s of a 220 Hz sine through it and
//! asserts the output is healthy (finite, non-silent, bounded). Compressor
//! tests additionally print loud/quiet gain so gain-reduction depth can be
//! audited; depth is NOT hard-asserted because the engine's achievable GR is
//! a known limitation (see integration_compressor.rs dyna_comp notes).

mod audio_analysis;

use audio_analysis::*;

/// Local sample rate constant (do not depend on audio_analysis::SAMPLE_RATE,
/// which may change under concurrent edits).
const SR: f64 = 48_000.0;

/// Local sine generator with explicit amplitude.
fn osc(freq_hz: f64, amplitude: f64, duration_secs: f64) -> Vec<f64> {
    let n = (duration_secs * SR) as usize;
    (0..n)
        .map(|i| {
            let t = i as f64 / SR;
            amplitude * (2.0 * std::f64::consts::PI * freq_hz * t).sin()
        })
        .collect()
}

/// Linear ratio -> dB, floored.
fn db(lin: f64) -> f64 {
    20.0 * lin.max(1e-12).log10()
}

/// Print output/input RMS gain for a loud and a quiet sine through a
/// compressor example. Asserts health (finite + non-silent) only — GR depth
/// is reported, not asserted (known engine limitation).
fn report_compressor_gain(file: &str, name: &str, controls: &[(&str, f64)]) {
    let loud_in = osc(220.0, 0.5, 1.0);
    let quiet_in = osc(220.0, 0.05, 1.0);

    let loud_out = compile_example_and_process(file, &loud_in, SR, controls);
    let quiet_out = compile_example_and_process(file, &quiet_in, SR, controls);

    assert_healthy(&loud_out, &format!("{name} loud"), 50.0);
    assert_healthy(&quiet_out, &format!("{name} quiet"), 50.0);

    // Skip the first half second so envelope state has settled.
    let settle = (0.5 * SR) as usize;
    let loud_gain = rms(&loud_out[settle..]) / rms(&loud_in[settle..]);
    let quiet_gain = rms(&quiet_out[settle..]) / rms(&quiet_in[settle..]);

    assert!(loud_gain.is_finite(), "{name}: loud gain not finite");
    assert!(quiet_gain.is_finite(), "{name}: quiet gain not finite");

    eprintln!(
        "  [diag] {name}: loud(0.5) gain={loud_gain:.4} ({:+.2} dB), \
         quiet(0.05) gain={quiet_gain:.4} ({:+.2} dB), GR delta={:+.2} dB",
        db(loud_gain),
        db(quiet_gain),
        db(quiet_gain) - db(loud_gain)
    );
}

// ===========================================================================
// FET Leveler (1176-inspired FET feedback limiter)
// ===========================================================================
//
// AUDIT NOTE: the EF -> J1.vgs modulation binding compiles but produces no
// measurable gain reduction in the current engine (loud and quiet probes
// measure identical gain), so the gain report below documents GR depth
// rather than asserting it. See the .pedal header, simplification item 7.

#[test]
fn fet_leveler_produces_output() {
    let input = osc(220.0, 0.2, 1.0);
    let output = compile_example_and_process(
        "fet_leveler.pedal",
        &input,
        SR,
        &[("Input", 0.7), ("Output", 0.7)],
    );
    assert_healthy(&output, "FET Leveler", 50.0);
}

#[test]
fn fet_leveler_gain_report() {
    report_compressor_gain(
        "fet_leveler.pedal",
        "FET Leveler",
        &[("Input", 0.7), ("Output", 0.7)],
    );
}

// ===========================================================================
// Opto Leveler (LA-2A-inspired optical leveling amplifier)
// ===========================================================================
//
// AUDIT NOTE: the engine modulates the photocoupler as a series transmission
// element (more LED drive -> more signal), so the envelope follower currently
// produces upward EXPANSION instead of LA-2A compression (~+35 dB loud-vs-
// quiet delta). Documented in the .pedal header, simplification item 6.

#[test]
fn opto_leveler_produces_output() {
    let input = osc(220.0, 0.2, 1.0);
    let output = compile_example_and_process("opto_leveler.pedal", &input, SR, &[("Gain", 0.6)]);
    assert_healthy(&output, "Opto Leveler", 50.0);
}

#[test]
fn opto_leveler_gain_report() {
    report_compressor_gain("opto_leveler.pedal", "Opto Leveler", &[("Gain", 0.6)]);
}

// ===========================================================================
// VCA Bus Comp (SSL G-bus / dbx-inspired VCA compressor)
// ===========================================================================
//
// AUDIT NOTE (updated 2026-06-12, G4 fixed): vca(ssm2164) is now lowered by
// the compiler's vca_lowering pass into a runtime SSM2164-class dB-linear
// gain element, and `cv` is a live envelope modulation sink (VcaCv). The
// example's hard-wired bypass is removed — audio runs through the VCA and
// the gain report below shows real gain reduction (loud(0.5) ≈ -4 dB vs
// quiet(0.05) ≈ -0.5 dB, GR delta ≈ +3.7 dB). Hard GR/ratio assertions live
// in outboard_acceptance::vca_bus_comp_compresses (GR > 3 dB, ratio > 1.3).

#[test]
fn vca_bus_comp_produces_output() {
    let input = osc(220.0, 0.2, 1.0);
    let output = compile_example_and_process("vca_bus_comp.pedal", &input, SR, &[("Makeup", 0.7)]);
    assert_healthy(&output, "VCA Bus Comp", 50.0);
}

#[test]
fn vca_bus_comp_gain_report() {
    report_compressor_gain("vca_bus_comp.pedal", "VCA Bus Comp", &[("Makeup", 0.7)]);
}

// ===========================================================================
// Passive LC EQ (Pultec EQP-1A-inspired passive EQ)
// ===========================================================================
//
// AUDIT NOTE: the EQ ships as the passive section only. Every attempted
// makeup stage (12AX7 after, 12AX7 before, JFET buffer, NE5532) either
// silenced the circuit or froze/flattened the EQ controls in the current
// engine — details with measurements in the .pedal header. Switched
// components (cap_switched / inductor_switched) also collapse the passive
// network to a unity passthrough, so fixed C/L values are used.

/// Measure steady-state gain at `freq` through the EQ with given controls.
/// Uses an integer number of cycles after a settle period so the Goertzel
/// bin lands on the probe frequency.
fn eq_gain_at(freq: f64, controls: &[(&str, f64)]) -> f64 {
    let settle_secs = 0.5;
    let cycles = (0.25 * freq).ceil().max(8.0);
    let window_secs = cycles / freq;
    let input = osc(freq, 0.1, settle_secs + window_secs);
    let output = compile_example_and_process("passive_lc_eq.pedal", &input, SR, controls);
    assert!(
        output.iter().all(|x| x.is_finite()),
        "EQ output at {freq} Hz contains NaN/inf"
    );
    let settle = (settle_secs * SR) as usize;
    let in_mag = goertzel_mag(&input[settle..], SR, freq);
    let out_mag = goertzel_mag(&output[settle..], SR, freq);
    out_mag / in_mag.max(1e-12)
}

#[test]
fn passive_lc_eq_produces_output() {
    let input = osc(220.0, 0.2, 1.0);
    let output = compile_example_and_process("passive_lc_eq.pedal", &input, SR, &[]);
    assert_healthy(&output, "Passive LC EQ", 50.0);
}

#[test]
fn passive_lc_eq_response_report() {
    let flat: &[(&str, f64)] = &[("LF Boost", 0.0), ("HF Boost", 0.0)];
    let boosted: &[(&str, f64)] = &[("LF Boost", 1.0), ("HF Boost", 1.0)];

    for &freq in &[100.0, 1000.0, 10_000.0] {
        let g_flat = eq_gain_at(freq, flat);
        let g_boost = eq_gain_at(freq, boosted);
        assert!(g_flat.is_finite(), "flat gain at {freq} Hz not finite");
        assert!(g_boost.is_finite(), "boosted gain at {freq} Hz not finite");
        eprintln!(
            "  [diag] Passive LC EQ @ {freq:>7.0} Hz: boost pots min={:+.2} dB, \
             max={:+.2} dB, delta={:+.2} dB",
            db(g_flat),
            db(g_boost),
            db(g_boost) - db(g_flat)
        );
    }

    // HF boost control effect at 10 kHz. Asserted only because measurement
    // confirmed the control moves the 10 kHz gain; if this regresses, the
    // assertion failure itself is the audit finding.
    let hf_min = eq_gain_at(10_000.0, &[("HF Boost", 0.0)]);
    let hf_max = eq_gain_at(10_000.0, &[("HF Boost", 1.0)]);
    let hf_delta_db = (db(hf_max) - db(hf_min)).abs();
    eprintln!("  [diag] Passive LC EQ HF Boost effect at 10 kHz: {hf_delta_db:.2} dB");
    assert!(
        hf_delta_db > 0.5,
        "HF Boost control should change 10 kHz gain by >0.5 dB (got {hf_delta_db:.3} dB)"
    );
}
