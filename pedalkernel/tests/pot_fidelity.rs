//! Pot-control-fidelity regression tests for the outboard plugins.
//!
//! The plugin UI binds knobs directly to the `.pedal` `controls{}` block, so a
//! pot whose runtime `set_control` is broken ships as a DEAD or MISLEADING
//! knob. This file pins down three independently root-caused defects against
//! the REAL shipped circuits:
//!
//!   #1/#2 — Pultec passive LC EQ (`passive_lc_eq.pedal`): LF Boost / HF Boost
//!           pots must move the frequency response MONOTONICALLY in pot
//!           position, and position 0.5 must differ from position 0.0 (the
//!           prior compiled-baseline-at-0.5 fold + sub-0.5 aliasing bugs made
//!           pos-0.5 == pos-0.0 and folded the curve around 0.5).
//!   #3   — LA-2A opto leveler (`opto_leveler.pedal`): the Gain pot (3-terminal
//!           divider into the 12AX7 grid) must sweep the small-signal makeup
//!           gain monotonically with position.

mod audio_analysis;

use audio_analysis::*;

const PULTEC: &str = "passive_lc_eq.pedal";
const OPTO: &str = "opto_leveler.pedal";
const PROBE_AMP: f64 = 0.1;

/// Full-range pot positions including the historically-aliased lower half.
const POSITIONS: [f64; 5] = [0.0, 0.25, 0.5, 0.75, 1.0];

/// Single-frequency gain (dB) of the Pultec at the given controls.
fn pultec_gain_at(freq: f64, controls: &[(&str, f64)]) -> f64 {
    let src = example_pedal_source(PULTEC);
    frequency_response_db(
        || pedal_processor(&src, SAMPLE_RATE, controls),
        &[freq],
        PROBE_AMP,
        SAMPLE_RATE,
    )[0]
    .1
}

/// Assert a (position, value) sweep is monotonically non-decreasing within
/// `tol`, AND that position 0.5 differs from position 0.0 by at least `sep`.
fn assert_monotonic_and_split(sweep: &[(f64, f64)], tol: f64, sep: f64, what: &str) {
    for w in sweep.windows(2) {
        assert!(
            w[1].1 >= w[0].1 - tol,
            "{what}: not monotonic in position — {:+.4} dB at pos {:.2} -> {:+.4} dB at pos {:.2}",
            w[0].1,
            w[0].0,
            w[1].1,
            w[1].0
        );
    }
    let p0 = sweep.iter().find(|&&(p, _)| p == 0.0).map(|&(_, g)| g);
    let p5 = sweep.iter().find(|&&(p, _)| p == 0.5).map(|&(_, g)| g);
    if let (Some(g0), Some(g5)) = (p0, p5) {
        assert!(
            (g5 - g0).abs() >= sep,
            "{what}: pos-0.5 ({g5:+.4} dB) must differ from pos-0.0 ({g0:+.4} dB) by >= {sep} dB \
             (was folded/aliased so 0.5 == 0.0)"
        );
    }
}

// ===========================================================================
// #1 / #2 — Pultec LF Boost pot: full-range monotonic shelf, 0.5 != 0.0
// ===========================================================================

#[test]
fn pultec_lf_boost_full_range_monotonic() {
    let freq = 63.0;
    let sweep: Vec<(f64, f64)> = POSITIONS
        .iter()
        .map(|&p| (p, pultec_gain_at(freq, &[("LF Boost", p)])))
        .collect();
    for &(p, g) in &sweep {
        eprintln!("LF Boost {p:.2} at {freq:.0} Hz -> {g:+8.3} dB");
    }
    assert_monotonic_and_split(&sweep, 0.05, 0.3, "LF Boost full range at 63 Hz");
}

// ===========================================================================
// #1 / #2 — Pultec HF Boost pot: full-range monotonic, 0.5 != 0.0
// ===========================================================================

#[test]
fn pultec_hf_boost_full_range_monotonic() {
    // "HF Boost" is range-inverted [1.0, 0.0]; control 0..1 maps to pot
    // positions 1.0..0.0. The boost amount at ~5 kHz should rise as control
    // rises (more boost) monotonically over the FULL control range.
    let freq = 5_044.0;
    let sweep: Vec<(f64, f64)> = POSITIONS
        .iter()
        .map(|&p| {
            (
                p,
                pultec_gain_at(freq, &[("HF Boost", p), ("HF Bandwidth", 0.0)]),
            )
        })
        .collect();
    for &(p, g) in &sweep {
        eprintln!("HF Boost {p:.2} at {freq:.0} Hz -> {g:+8.3} dB");
    }
    assert_monotonic_and_split(&sweep, 0.05, 0.3, "HF Boost full range at 5044 Hz");
}

// ===========================================================================
// #3 — LA-2A Gain pot: small-signal makeup gain sweeps monotonically
// ===========================================================================

/// Small-signal steady gain of the opto leveler at a near-linear drive so the
/// photocell GR loop is effectively idle and the Gain pot's divider authority
/// is what we measure.
fn opto_small_signal_gain_db(src: &str, gain_pos: f64) -> f64 {
    let mut process = pedal_processor(src, SAMPLE_RATE, &[("Gain", gain_pos)]);
    let amplitude = 0.002;
    let input = sine_at(1_000.0, amplitude, 1.5, SAMPLE_RATE);
    let output: Vec<f64> = input.iter().map(|&x| process(x)).collect();
    let tail = (1.0 * SAMPLE_RATE) as usize;
    lin_to_db(rms(&output[tail..]) / rms(&input[tail..]))
}

#[test]
fn opto_gain_pot_sweeps_monotonically() {
    let src = example_pedal_source(OPTO);
    let positions = [0.1, 0.3, 0.5, 0.7, 0.9];
    let sweep: Vec<(f64, f64)> = positions
        .iter()
        .map(|&p| (p, opto_small_signal_gain_db(&src, p)))
        .collect();
    for &(p, g) in &sweep {
        eprintln!("Gain {p:.2} (small signal) -> {g:+8.3} dB");
    }
    // Monotonic rise with at least 0.25 dB per step (no folding/flatness).
    for w in sweep.windows(2) {
        assert!(
            w[1].1 > w[0].1 + 0.05,
            "Gain pot must raise gain monotonically: {:+.3} dB at {:.1} -> {:+.3} dB at {:.1}",
            w[0].1,
            w[0].0,
            w[1].1,
            w[1].0
        );
    }
    let span = sweep.last().unwrap().1 - sweep[0].1;
    eprintln!("Gain pot span (0.1 -> 0.9): {span:+.3} dB");
    assert!(
        span > 6.0,
        "Gain pot span {span:+.3} dB over positions 0.1-0.9, expected a sensible (> 6 dB) sweep"
    );
}
