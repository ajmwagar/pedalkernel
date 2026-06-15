//! Failing-first (TDD red) [V] acceptance tests for the FAITHFUL LA-2A
//! rebuild (`examples/outboard/compressor/la2a.pedal`).
//!
//! These encode the tightened **[V]** acceptance criteria from
//! `reports/la2a-1to1-rebuild-plan-2026-06-13.md` §3. They are RED by
//! construction: the faithful netlist wires the detector, transformers and
//! makeup pot as the hardware INTENDS, and the engine cannot honor that yet.
//! Each test is `#[ignore]`d (CI stays green) with the BLOCKING GAP named and
//! the measured numbers from this phase-0 pass in the ignore reason. As each
//! engine gap is fixed, remove the matching `#[ignore]` to promote the test.
//!
//! Blocking engine gaps (plan §2, §4):
//!   GAP G (CRUX) — detector-from-circuit-sidechain. The `EL_drive.b ->
//!     PC1.led` net drives the LED from a real circuit node, but the engine
//!     only binds PhotocouplerLed from an `envelope_follower` element's `.out`
//!     net (compiler/bind.rs:716-762). No drive -> LED dark -> ZERO GR.
//!   GAP H — T4B GR-history memory. t4b() uses fixed taus + constant
//!     slow_weight (pedalkernel-rt/src/elements/controlled.rs:85-94, 166-205);
//!     release does not slow after heavier/longer GR.
//!   GAP F — output transformer step-down (~19 dB debt, transformer.rs:86-113).
//!   GAP E — Gain-pot placement / inert makeup pot.
//!
//! Run the red suite:
//!   cargo test -p pedalkernel --no-default-features --test la2a_acceptance -- --ignored --nocapture

mod audio_analysis;

use audio_analysis::*;

const LA2A: &str = "la2a.pedal";

/// T4B-aware long-settle steady gain: opto cells release over seconds, so a
/// 0.5 s settle measures mid-transition. Run 8 s of program at one amplitude
/// and measure the final second. (Same convention as
/// tests/outboard_acceptance.rs::opto_steady_gain_db.)
fn opto_steady_gain_db(src: &str, controls: &[(&str, f64)], amplitude: f64, freq_hz: f64) -> f64 {
    let mut process = pedal_processor(src, SAMPLE_RATE, controls);
    let input = sine_at(freq_hz, amplitude, 8.0, SAMPLE_RATE);
    let output: Vec<f64> = input.iter().map(|&x| process(x)).collect();
    let tail = (7.0 * SAMPLE_RATE) as usize;
    lin_to_db(rms(&output[tail..]) / rms(&input[tail..]))
}

// ===========================================================================
// Compiles + healthy
// ===========================================================================

/// The faithful LA-2A must at least PARSE, COMPILE, and produce a finite,
/// non-silent, bounded output for a moderate sine. This may pass or fail in
/// phase 0 — the report records which. The most likely blocker is GAP F
/// (output transformer step-down leaves the chain very quiet -> `assert_healthy`
/// silence floor) or a tube-stage bias issue; not a dynamics gap.
#[test]
#[ignore = "phase-0 [V] target: report compile/health result; blockers GAP F (xfmr step-down -> level) / tube bias"]
fn la2a_compiles_and_is_healthy() {
    let src = example_pedal_source(LA2A);
    let controls: &[(&str, f64)] = &[("Gain", 0.6), ("Peak Reduction", 0.5)];
    let input = sine_at(1_000.0, 0.1, 1.0, SAMPLE_RATE);
    let output = compile_and_process(&src, &input, SAMPLE_RATE, controls);
    assert_healthy(&output, "LA-2A faithful", 100.0);
}

// ===========================================================================
// GR direction + depth (BLOCKED BY GAP G)
// ===========================================================================

/// Louder program must mean LESS gain (downward compression via the shunt
/// T4B cell). Blocked by GAP G: the LED is wired from the real sidechain
/// node (`EL_drive.b -> PC1.led`), but only an `envelope_follower` can bind
/// PhotocouplerLed (bind.rs:716-762), so the cell never illuminates and GR is
/// ~0 dB — the circuit behaves as a static (non-dynamic) amplifier.
// PROMOTED (Phase 3, 2026-06-15): GAP G CLOSED. The detector's solved EL-drive
// value is carried one sample late (2b z⁻¹ carry) and now drives the T4B
// photocoupler LED (`DetectorLedCoupling` in spqr_build.rs / processor.rs); the
// shunt CdS cell — compiled as an MNA variable resistor (rigid/general.rs) in
// the V1 makeup-tube group — darkens with program, producing real downward GR.
// Measured: quiet(0.05) gain ~+86 dB, loud(0.5) gain ~+67 dB, GR ~19 dB.
#[test]
fn la2a_reduces_gain_as_level_rises() {
    let src = example_pedal_source(LA2A);
    let controls: &[(&str, f64)] = &[("Gain", 0.6), ("Peak Reduction", 0.5)];

    let quiet_gain = opto_steady_gain_db(&src, controls, 0.05, 1_000.0);
    let loud_gain = opto_steady_gain_db(&src, controls, 0.5, 1_000.0);
    let gr = quiet_gain - loud_gain;
    eprintln!(
        "LA-2A: quiet(0.05) gain {quiet_gain:+.3} dB, loud(0.5) gain {loud_gain:+.3} dB, GR {gr:+.3} dB"
    );

    assert!(quiet_gain.is_finite() && loud_gain.is_finite());
    assert!(
        loud_gain < quiet_gain,
        "LA-2A must compress: loud gain {loud_gain:+.3} dB not below quiet gain {quiet_gain:+.3} dB"
    );
    assert!(
        gr > 3.0,
        "LA-2A GR {gr:.3} dB between 0.05 and 0.5 amplitude, expected > 3 dB"
    );
}

// ===========================================================================
// Attack ~10 ms (BLOCKED BY GAP G; then validated against T4B ~10 ms)
// ===========================================================================

/// The real T4B attack is ~10 ms. Assert the measured step-attack lands in
/// 1-50 ms. Blocked by GAP G first (no GR -> nothing to time); once GR exists,
/// the attack is set by the t4b() tau_fast_rise (controlled.rs:90).
#[test]
#[ignore = "BLOCKED BY GAP G then validates T4B ~10 ms attack (controlled.rs:90). Report fills in measured attack."]
fn la2a_attack_near_10ms() {
    let src = example_pedal_source(LA2A);
    let controls: &[(&str, f64)] = &[("Gain", 0.6), ("Peak Reduction", 0.5)];

    let lo_secs = 0.5;
    let hi_secs = 1.5;
    let step_up = (lo_secs * SAMPLE_RATE) as usize;
    let step_down = step_up + (hi_secs * SAMPLE_RATE) as usize;
    let burst = tone_burst(1_000.0, 0.05, 0.5, lo_secs, hi_secs, 3.0, SAMPLE_RATE);
    let output = compile_and_process(&src, &burst, SAMPLE_RATE, controls);
    assert_healthy(&output, "LA-2A attack burst", 100.0);

    let attack = measure_attack_seconds(&output[..step_down], step_up, SAMPLE_RATE);
    eprintln!("LA-2A attack: {attack:.4} s (target ~10 ms)");
    assert!(
        (0.001..=0.050).contains(&attack),
        "LA-2A attack {attack:.4} s outside the real T4B range 1-50 ms"
    );
}

// ===========================================================================
// PROGRAM-DEPENDENT RELEASE (BLOCKED BY GAP H) — the defining LA-2A behavior
// ===========================================================================

/// The defining LA-2A behavior: release after heavy/long GR must be SLOWER
/// than after a light/short hit (CdS memory). Blocked by GAP H: t4b() has
/// fixed taus + constant slow_weight (controlled.rs:85-94) with no GR-history
/// state, so release is program-INDEPENDENT (and the inspired model even
/// measures it BACKWARDS). Also blocked upstream by GAP G (no GR at all yet).
/// The report records BOTH release times (heavy vs light) measured this pass.
#[test]
#[ignore = "BLOCKED BY GAP H (no T4B GR-history memory, controlled.rs:85-94) and upstream GAP G. Defining LA-2A behavior. Report fills in measured heavy-vs-light release."]
fn la2a_release_slower_after_heavy_gr() {
    let src = example_pedal_source(LA2A);
    let controls: &[(&str, f64)] = &[("Gain", 0.6), ("Peak Reduction", 0.5)];

    let mut releases = Vec::new();
    for (name, amp_hi, hi_secs) in [
        ("heavy (0.5 amp, 3.0 s)", 0.5, 3.0),
        ("light (0.15 amp, 0.5 s)", 0.15, 0.5),
    ] {
        let lo_secs = 1.0;
        let burst = tone_burst(1_000.0, 0.05, amp_hi, lo_secs, hi_secs, 8.0, SAMPLE_RATE);
        let output = compile_and_process(&src, &burst, SAMPLE_RATE, controls);
        assert_healthy(&output, name, 100.0);
        let step_down = ((lo_secs + hi_secs) * SAMPLE_RATE) as usize;
        let release = measure_release_seconds(&output, step_down, SAMPLE_RATE);
        eprintln!("LA-2A release after {name}: {release:.4} s");
        releases.push(release);
    }
    let (heavy, light) = (releases[0], releases[1]);
    eprintln!(
        "LA-2A program dependence: heavy {heavy:.3} s vs light {light:.3} s — \
         T4B memory predicts heavy > light; measured {}",
        if heavy > light { "TRUE" } else { "FALSE (no/backwards memory)" }
    );
    assert!(
        heavy > light + 0.1,
        "LA-2A release must SLOW after heavy GR: heavy {heavy:.3} s not > light {light:.3} s"
    );
}

// ===========================================================================
// GR static curve vs published LA-2A shape (BLOCKED BY GAP G)
// ===========================================================================

/// The static gain curve must fall monotonically with level (smooth soft-knee
/// leveling, limiter-grade at the top — published LA-2A character). Blocked by
/// GAP G: with the cell dark the curve is flat (linear amp), not a leveling
/// curve. The report records the measured curve + total GR.
// PROMOTED (Phase 3, 2026-06-15): GAP G CLOSED — the static curve now falls
// monotonically with level (the shunt T4B cell darkens as program rises) and
// the total GR (-40 vs 0 dB input) is deep (~50 dB), well past the 20 dB floor.
#[test]
fn la2a_gr_curve_matches_published_shape() {
    let src = example_pedal_source(LA2A);
    let controls: &[(&str, f64)] = &[("Gain", 0.6), ("Peak Reduction", 0.5)];

    let levels: Vec<f64> = (0..=8).map(|i| -40.0 + 5.0 * i as f64).collect();
    let curve: Vec<(f64, f64)> = levels
        .iter()
        .map(|&in_db| {
            let gain = opto_steady_gain_db(&src, controls, db_to_lin(in_db), 1_000.0);
            (in_db, in_db + gain)
        })
        .collect();
    for &(in_db, out_db) in &curve {
        eprintln!(
            "LA-2A GR curve: in {in_db:6.1} dB -> out {out_db:+8.3} dB (gain {:+7.3} dB)",
            out_db - in_db
        );
        assert!(out_db.is_finite(), "non-finite at {in_db} dB input");
    }

    // Gain falls monotonically with level (0.1 dB tolerance).
    for w in curve.windows(2) {
        let g0 = w[0].1 - w[0].0;
        let g1 = w[1].1 - w[1].0;
        assert!(
            g1 < g0 + 0.1,
            "gain must fall with level: {g0:+.3} dB at {:.0} -> {g1:+.3} dB at {:.0}",
            w[0].0,
            w[1].0
        );
    }

    let total_gr = (curve[0].1 - curve[0].0) - (curve[8].1 - curve[8].0);
    eprintln!("LA-2A total GR (-40 vs 0 dB input): {total_gr:.3} dB");
    assert!(
        total_gr > 20.0,
        "LA-2A GR {total_gr:.3} dB over 40 dB of input, expected > 20 dB (published deep leveling)"
    );
}
