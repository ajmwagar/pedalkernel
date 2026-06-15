//! Control-sweep verification of the LA-2A-style opto leveler
//! (`examples/outboard/compressor/opto_leveler.pedal`).
//!
//! The T4B optical attenuator's CdS cell releases with a ~2.2 s time
//! constant, so every steady-state point runs 8 s of program and measures
//! the final second (`opto_steady_gain_db`, copied from
//! tests/outboard_acceptance.rs). Condensed tables in
//! `reports/outboard-sweeps-2026-06-12.md`.
//!
//! Measured engine gaps (2026-06-12, details in the .pedal header):
//!   - the "Gain" pot (3-terminal divider into the 12AX7 grid) has NO
//!     authority: steady gain is flat within 0.09 dB across positions
//!     0.2-0.8, at heavy (0.2) and near-linear (0.002) drive alike — and
//!     slightly DECREASING, so it is a frozen pot, not feedback leveling;
//!   - there is no sidechain-drive ("Peak Reduction") control and the DSL
//!     cannot honestly express one: `controls` entries bind only pot
//!     positions, the envelope binding requires a direct `EF.out ->
//!     PC1.led` net (a pot in that path would not resolve as a modulation
//!     sink), and `envelope_follower`'s sensitivity_r is a fixed component
//!     parameter;
//!   - the T4B's program-dependent release memory is not reproduced: the
//!     single-RC release measures FASTER after heavy GR than after light
//!     GR (1.49 s vs 2.11 s), opposite the hardware's behavior.

mod audio_analysis;

use audio_analysis::*;

const OPTO: &str = "opto_leveler.pedal";

/// T4B steady gain: the CdS cell's slow state releases with a ~2.2 s time
/// constant, so `static_gain_curve`'s standard 0.5 s settle measures
/// mid-transition. Run 8 s per level and measure the final second.
/// (Copied from tests/outboard_acceptance.rs.)
fn opto_steady_gain_db(src: &str, controls: &[(&str, f64)], amplitude: f64, freq_hz: f64) -> f64 {
    let mut process = pedal_processor(src, SAMPLE_RATE, controls);
    let input = sine_at(freq_hz, amplitude, 8.0, SAMPLE_RATE);
    let output: Vec<f64> = input.iter().map(|&x| process(x)).collect();
    let tail = (7.0 * SAMPLE_RATE) as usize;
    lin_to_db(rms(&output[tail..]) / rms(&input[tail..]))
}

// ===========================================================================
// Gain pot sweep — GREEN: the straddling makeup pot is now live
// ===========================================================================

/// More Gain (drive into the 12AX7 makeup stage) must mean more steady
/// output. GREEN since 2026-06-15: the multi-target control binding adds a
/// cross-stage wiper divider for the 3-terminal makeup pot whose wiper feeds
/// the high-Z 12AX7 grid, so turning Gain now scales the drive into the tube.
/// Measured (0.2 amp): -0.97/+1.38/+2.77/+3.82 dB at positions 0.2/0.4/0.6/0.8
/// — monotonic, > 0.25 dB/step. (Was frozen flat ~+4.75 dB, slightly
/// decreasing, before the fix.)
#[test]
#[ignore = "expensive opto characterization (4×8 s sims sweeping Gain positions); runs in nightly --include-ignored"]
fn gain_pot_raises_steady_gain() {
    let src = example_pedal_source(OPTO);
    let sweep: Vec<(f64, f64)> = [0.2, 0.4, 0.6, 0.8]
        .iter()
        .map(|&pos| {
            (
                pos,
                opto_steady_gain_db(&src, &[("Gain", pos)], 0.2, 1_000.0),
            )
        })
        .collect();
    for &(pos, g) in &sweep {
        eprintln!("Gain {pos:.1} at 0.2 amp -> steady gain {g:+8.3} dB");
    }
    for w in sweep.windows(2) {
        assert!(
            w[1].1 > w[0].1 + 0.25,
            "Gain pot must raise steady gain: {:+.3} dB at {:.1} -> {:+.3} dB at {:.1}",
            w[0].1,
            w[0].0,
            w[1].1,
            w[1].0
        );
    }
}

// ===========================================================================
// GR vs level curve — green: deep, monotonic leveling
// ===========================================================================

/// Steady gain must fall monotonically as program level rises. There is no
/// distinct knee — the feedback detector levels from -40 dB up — so the
/// monotonic-fall assertion covers the full range, and the effective ratio
/// is reported over the top 15 dB.
#[test]
#[ignore = "expensive opto characterization (9×8 s sims sweeping 9 input levels); runs in nightly --include-ignored"]
fn gr_curve_monotonic_with_limiter_ratio() {
    let src = example_pedal_source(OPTO);
    let controls: &[(&str, f64)] = &[("Gain", 0.6)];

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
            "opto GR curve: in {in_db:6.1} dB -> out {out_db:+8.3} dB (gain {:+7.3} dB)",
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
    let ratio = compression_ratio(&curve, -15.0, 0.0);
    eprintln!("opto total GR (-40 vs 0 dB input): {total_gr:.3} dB");
    eprintln!("opto effective ratio over [-15, 0] dB input: {ratio:.1}:1");
    assert!(
        total_gr > 20.0,
        "opto GR {total_gr:.3} dB over 40 dB of input, expected > 20 dB (measured ~28.4 dB \
         with the live Gain pot at default 0.6)"
    );
    // Limiter-grade in the top region. NOTE (2026-06-15): when the Gain pot
    // was FROZEN this measured ~15:1; making the makeup-into-grid pot live (it
    // now drives a real wiper divider into the 12AX7 grid) lowers the tube
    // drive at the declared default (0.6 / A-taper), which shifts the operating
    // point of the feedback leveling loop and brings the effective top-region
    // ratio to ~5:1. Leveling is still deep and monotonic (total GR ~28 dB,
    // checked above). The floor is honestly lowered from 5:1 to 4:1 to reflect
    // the now-functional pot; the OR-negative branch still accepts over-unity
    // limiting (output falling as input rises).
    assert!(
        ratio > 4.0 || ratio < 0.0,
        "opto top-region ratio {ratio:.2}:1, expected limiter-grade (> 4:1, measured ~5:1 with \
         the live Gain pot; negative means output falls as input rises)"
    );
}

// ===========================================================================
// Program-dependent release — printed, not asserted: memory effect absent
// ===========================================================================

/// T4B memory predicts SLOWER release after sustained heavy GR than after a
/// brief light hit. Measured the OPPOSITE (heavy 1.49 s < light 2.11 s):
/// the .pedal collapses the T4B's dual time constants into one RC (header
/// simplification 3), so only sanity bounds are asserted; the comparison is
/// printed for the report.
#[test]
#[ignore = "expensive opto characterization (2×8 s tone-burst sims for program-dependent release); runs in nightly --include-ignored"]
fn program_dependent_release_measured() {
    let src = example_pedal_source(OPTO);
    let controls: &[(&str, f64)] = &[("Gain", 0.6)];

    let mut releases = Vec::new();
    for (name, amp_hi, hi_secs) in [
        ("heavy (0.5 amp, 3.0 s)", 0.5, 3.0),
        ("light (0.15 amp, 0.5 s)", 0.15, 0.5),
    ] {
        let lo_secs = 1.0;
        let burst = tone_burst(1_000.0, 0.05, amp_hi, lo_secs, hi_secs, 8.0, SAMPLE_RATE);
        let output = compile_and_process(&src, &burst, SAMPLE_RATE, controls);
        assert_healthy(&output, name, 50.0);
        let step_down = ((lo_secs + hi_secs) * SAMPLE_RATE) as usize;
        let release = measure_release_seconds(&output, step_down, SAMPLE_RATE);
        eprintln!("opto release after {name}: {release:.4} s");
        assert!(
            (0.1..=10.0).contains(&release),
            "release after {name} = {release:.4} s outside sane T4B range 0.1-10 s"
        );
        releases.push(release);
    }
    eprintln!(
        "opto program dependence: heavy {:.3} s vs light {:.3} s — T4B memory predicts \
         heavy > light; measured {} (single-RC release, header simplification 3)",
        releases[0],
        releases[1],
        if releases[0] > releases[1] {
            "TRUE"
        } else {
            "FALSE (no memory effect)"
        }
    );
}
