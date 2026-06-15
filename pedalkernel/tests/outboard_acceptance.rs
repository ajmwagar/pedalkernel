//! Failing-first (TDD red) acceptance tests for the outboard-gear gaps
//! documented in `reports/outboard-gear-audit-2026-06-12.md`.
//!
//! Every test here defines what "fixed" means for one audit gap and is
//! `#[ignore]`d so CI stays green while the gaps are open. Each test FAILS
//! today when run with `--ignored` — that failure is the point. As fixes
//! land, remove the corresponding `#[ignore]` to promote the test into the
//! regular suite.
//!
//! Gap IDs (see audit §6):
//!   G1  — envelope follower detector taps read the global pedal input
//!   G2  — `EF.out -> J.vgs` envelope routing is inert on JFET shunt/series
//!   G3  — photocoupler LED drive acts as series gain (inverted GR direction)
//!   G4  — `vca()` is a virtual stub; `cv` is not a modulation sink
//!   B1  — dyna_comp compiles nondeterministically across processes (audit §4)
//!   B2  — inverting op-amp stage with input coupling cap compiles to silence
//!   B3a — resistor+cap coupling between op-amp stages compiles to silence
//!   B3b — DC/resistor coupling between op-amp stages loses a stage's gain
//!
//! Run the red suite:
//!   cargo test -p pedalkernel --no-default-features --test outboard_acceptance -- --ignored

mod audio_analysis;

use audio_analysis::*;

/// Settle time (samples) discarded before steady-state measurements.
fn settle_samples() -> usize {
    (0.5 * SAMPLE_RATE) as usize
}

/// Steady-state gain (dB) of a pedal at one input amplitude: single-point
/// static gain curve at `freq_hz` (fresh processor, 0.5 s settle).
fn steady_gain_db(src: &str, controls: &[(&str, f64)], amplitude: f64, freq_hz: f64) -> f64 {
    let level_db = lin_to_db(amplitude);
    let curve = static_gain_curve(
        || pedal_processor(src, SAMPLE_RATE, controls),
        &[level_db],
        freq_hz,
        SAMPLE_RATE,
    );
    curve[0].1 - curve[0].0
}

// ===========================================================================
// G1+G2 — FET Leveler (1176-style JFET shunt divider) must compress
// ===========================================================================

/// [G1+G2] examples/outboard/compressor/fet_leveler.pedal: the EF1 -> J1.vgs
/// sidechain must produce real gain reduction. Audit measured loud and quiet
/// program at the IDENTICAL gain (GR delta 0.00 dB) because vgs envelope
/// routing is inert (G2) and detector taps are fake (G1).
///
/// 2026-06-12 (F4+F5): G2 and G1 are FIXED — the EF->J1.vgs binding is live
/// and the feedback tap (`EF1.in -> Output.w`) is honored (rewiring the
/// detector to the input measures >15 dB GR on this circuit; see
/// tests/envelope_taps.rs for the tap acceptance tests). The remaining
/// blocker is the chain's absolute gain: the BJT line amp produces almost
/// no makeup gain (G5 family) and the output transformer costs ~19 dB vs
/// SPICE, so the whole chain sits at ~-58 dB and the feedback detector sees
/// only mV-level drive (GR 0.01 dB, ratio 1.00).
/// See reports/outboard-gear-audit-2026-06-12.md §3, §6.
///
/// NGSPICE REFERENCE BASELINE (bead pedalkernel-mbte.1, 2026-06-15):
///   Golden: pedalkernel-validate/golden/compressor/fet_leveler_*/
///   ngspice GR (0 vs -40 dBVU):  ~4.2 dB measured in SPICE at circuit level
///   ngspice attack:               ~5.5 ms (25k × 220n RC)
///   ngspice release:              ~1.1 s  (5M × 220n RC)
///   WDF GR (current):             ~0.02 dB (detector starved — G5 family)
///   Full WDF-vs-ngspice gap report: pedalkernel-validate --test compressor_dynamics_gap
///
/// Pass criteria (GR > 3 dB, ratio > 1.3:1) are derived from the ngspice
/// reference and are intentionally tight.  This test stays red until the
/// BJT makeup-gain and transformer-loss bugs (G5 family) are fixed.
#[test]
#[ignore = "red until makeup-gain fix (G5 family): tap+vgs routing now live, but the BJT line amp + transformer leave the chain at ~-58 dB, starving the feedback detector (GR ~0.02 dB vs ngspice ~4.2 dB — see pedalkernel-validate compressor suite)"]
fn fet_leveler_compresses() {
    let src = example_pedal_source("fet_leveler.pedal");
    let controls: &[(&str, f64)] = &[("Input", 0.7), ("Output", 0.7)];

    let levels: Vec<f64> = (0..=8).map(|i| -40.0 + 5.0 * i as f64).collect();
    let curve = static_gain_curve(
        || pedal_processor(&src, SAMPLE_RATE, controls),
        &levels,
        1_000.0,
        SAMPLE_RATE,
    );
    for &(in_db, out_db) in &curve {
        eprintln!(
            "fet leveler curve: in {in_db:6.1} dB -> out {out_db:8.3} dB (gain {:7.3} dB)",
            out_db - in_db
        );
        assert!(
            out_db.is_finite(),
            "fet leveler: non-finite output at {in_db} dB input"
        );
    }

    let gr = gain_reduction_db(&curve, -40.0, 0.0);
    let ratio = compression_ratio(&curve, -12.0, 0.0);
    eprintln!("fet leveler gain reduction (-40 vs 0 dB): {gr:.3} dB");
    eprintln!("fet leveler ratio (-12..0 dB): {ratio:.4}:1");

    assert!(
        gr > 3.0,
        "fet leveler GR {gr:.3} dB at 0 dB input, expected > 3 dB below the -40 dB gain"
    );
    assert!(
        ratio > 1.3,
        "fet leveler ratio {ratio:.4}:1 over [-12, 0] dB, expected > 1.3:1"
    );
}

// ===========================================================================
// G2 (minimal) — envelope follower must modulate JFET gain at all
// ===========================================================================

/// [G2] tests/test_pedals/envelope_follower.pedal: a gain-relevant JFET
/// divider (series Rds against R2 = 10k) whose gate is driven by an envelope
/// follower. Loud (0.5) and quiet (0.05) program must settle to gains that
/// differ by > 1 dB. Audit measured 0.00 dB delta on this exact fixture —
/// the envelope -> JFET modulation binding has no effect on the compiled
/// audio path. See reports/outboard-gear-audit-2026-06-12.md §3 (fet_leveler
/// notes) and §6 G2.
#[test]
fn envelope_to_jfet_vgs_modulates_gain() {
    let src = test_pedal_source("envelope_follower.pedal");

    let quiet_gain = steady_gain_db(&src, &[], 0.05, 1_000.0);
    let loud_gain = steady_gain_db(&src, &[], 0.5, 1_000.0);
    let delta = (quiet_gain - loud_gain).abs();
    eprintln!(
        "envelope/jfet divider: quiet(0.05) gain {quiet_gain:+.3} dB, \
         loud(0.5) gain {loud_gain:+.3} dB, delta {delta:.3} dB"
    );

    assert!(quiet_gain.is_finite() && loud_gain.is_finite());
    assert!(
        delta > 1.0,
        "envelope follower must modulate JFET gain: loud/quiet delta {delta:.3} dB, expected > 1 dB"
    );
}

// ===========================================================================
// G3 — Opto Leveler (LA-2A-style T4B shunt cell) GR direction + depth
// ===========================================================================

/// T4B steady gain: the CdS cell's slow state releases with a ~2.2 s time
/// constant, so `static_gain_curve`'s standard 0.5 s settle measures
/// mid-transition. Run 8 s per level and measure the final second.
fn opto_steady_gain_db(src: &str, controls: &[(&str, f64)], amplitude: f64, freq_hz: f64) -> f64 {
    let mut process = pedal_processor(src, SAMPLE_RATE, controls);
    let input = sine_at(freq_hz, amplitude, 8.0, SAMPLE_RATE);
    let output: Vec<f64> = input.iter().map(|&x| process(x)).collect();
    let tail = (7.0 * SAMPLE_RATE) as usize;
    lin_to_db(rms(&output[tail..]) / rms(&input[tail..]))
}

/// [G3] examples/outboard/compressor/opto_leveler.pedal: more program level
/// must mean LESS gain (downward compression via the shunt CdS cell). Audit
/// measured the OPPOSITE — loud +14.8 dB, quiet -20.7 dB, i.e. +35.5 dB of
/// upward expansion — because the engine modulates the photocoupler as a
/// series transmission gain regardless of netlist position.
/// See reports/outboard-gear-audit-2026-06-12.md §3, §6 G3.
/// 2026-06-12 (F6): fixed — LDR compiles as a controlled-resistance MNA
/// child, LED bound to the output-tap envelope; promoted with the T4B-aware
/// settle above.
#[test]
#[ignore = "expensive opto characterization (2×8 s sims waiting for T4B CdS cell to settle); runs in nightly --include-ignored"]
fn opto_leveler_reduces_gain_as_level_rises() {
    let src = example_pedal_source("opto_leveler.pedal");
    let controls: &[(&str, f64)] = &[("Gain", 0.6)];

    let quiet_gain = opto_steady_gain_db(&src, controls, 0.05, 1_000.0);
    let loud_gain = opto_steady_gain_db(&src, controls, 0.5, 1_000.0);
    let gr = quiet_gain - loud_gain;
    eprintln!(
        "opto leveler: quiet(0.05) gain {quiet_gain:+.3} dB, \
         loud(0.5) gain {loud_gain:+.3} dB, GR {gr:+.3} dB"
    );

    assert!(quiet_gain.is_finite() && loud_gain.is_finite());
    assert!(
        loud_gain < quiet_gain,
        "opto leveler must compress: loud gain {loud_gain:+.3} dB not below quiet gain {quiet_gain:+.3} dB (expansion!)"
    );
    assert!(
        gr > 3.0,
        "opto leveler GR {gr:.3} dB between 0.05 and 0.5 amplitude, expected > 3 dB"
    );
}

/// [G3] Opto Leveler attack/release on a tone burst must land in the T4B's
/// published ballpark once the GR direction is fixed: a fast onset and a
/// multi-stage release of 40 ms up to several seconds (assert 100 ms - 10 s).
/// See reports/outboard-gear-audit-2026-06-12.md §3, §6 G3.
///
/// ATTACK BAND WIDENED 2026-06-15 (live makeup pot). When the Gain pot was
/// FROZEN it sat OUTSIDE the gain-reduction feedback loop, so the envelope
/// snapped to its new steady GR within ~2 ms (`measure_attack_seconds`
/// reports the LAST exit from the ±1 dB band = full settle time). Making the
/// makeup-into-grid pot live puts it INSIDE the loop (Gain sets the 12AX7
/// drive, whose output feeds the feedback detector), so the envelope now
/// approaches its steady GR along the CdS cell's slow program-dependent ramp
/// — full settle ~1.1 s at the declared default (0.6 / A-taper ≈ 0.33 makeup).
/// That is the T4B's documented multi-second program dependence, not a defect:
/// compression depth + direction stay correct (see
/// opto_leveler_reduces_gain_as_level_rises, GR +15 dB) and the release is
/// unchanged. The band is honestly widened from 1-50 ms to 1 ms - 2 s to
/// cover the now-in-loop settle; it still rejects a stuck/never-settling
/// envelope.
#[test]
fn opto_leveler_attack_release_in_t4b_range() {
    let src = example_pedal_source("opto_leveler.pedal");
    let controls: &[(&str, f64)] = &[("Gain", 0.6)];

    let step_up = (0.5 * SAMPLE_RATE) as usize;
    let step_down = step_up + (1.5 * SAMPLE_RATE) as usize;
    let burst = tone_burst(1_000.0, 0.05, 0.5, 0.5, 1.5, 3.0, SAMPLE_RATE);
    let output = compile_and_process(&src, &burst, SAMPLE_RATE, controls);
    assert_healthy(&output, "Opto Leveler tone burst", 50.0);

    let attack = measure_attack_seconds(&output[..step_down], step_up, SAMPLE_RATE);
    let release = measure_release_seconds(&output, step_down, SAMPLE_RATE);
    eprintln!("opto leveler attack: {attack:.4} s, release: {release:.4} s");

    assert!(
        (0.001..=2.0).contains(&attack),
        "opto leveler attack {attack:.4} s outside in-loop settle range 1 ms - 2 s"
    );
    assert!(
        (0.100..=10.0).contains(&release),
        "opto leveler release {release:.4} s outside T4B range 100 ms - 10 s"
    );
}

// ===========================================================================
// G4 — VCA Bus Comp (SSL/dbx-style SSM2164) must compress
// ===========================================================================

/// [G4] examples/outboard/compressor/vca_bus_comp.pedal: the EF1 -> VCA1.cv
/// sidechain must produce real gain reduction. The audit measured a GR delta
/// of 0.11 dB because `vca(ssm2164)` was GraphRole::Virtual (no MNA stamp,
/// the compiled `vcas` binding list never populated) and `cv` was not a
/// valid envelope modulation sink. Fixed 2026-06-12 by the compiler's
/// vca_lowering pass (see tests/vca_lowering.rs); the example's hard-wire
/// bypass is removed. See reports/outboard-gear-audit-2026-06-12.md §3,
/// §6 G4.
#[test]
fn vca_bus_comp_compresses() {
    let src = example_pedal_source("vca_bus_comp.pedal");
    let controls: &[(&str, f64)] = &[("Makeup", 0.7)];

    let levels: Vec<f64> = (0..=8).map(|i| -40.0 + 5.0 * i as f64).collect();
    let curve = static_gain_curve(
        || pedal_processor(&src, SAMPLE_RATE, controls),
        &levels,
        1_000.0,
        SAMPLE_RATE,
    );
    for &(in_db, out_db) in &curve {
        eprintln!(
            "vca bus comp curve: in {in_db:6.1} dB -> out {out_db:8.3} dB (gain {:7.3} dB)",
            out_db - in_db
        );
        assert!(
            out_db.is_finite(),
            "vca bus comp: non-finite output at {in_db} dB input"
        );
    }

    let gr = gain_reduction_db(&curve, -40.0, 0.0);
    let ratio = compression_ratio(&curve, -12.0, 0.0);
    eprintln!("vca bus comp gain reduction (-40 vs 0 dB): {gr:.3} dB");
    eprintln!("vca bus comp ratio (-12..0 dB): {ratio:.4}:1");

    assert!(
        gr > 3.0,
        "vca bus comp GR {gr:.3} dB between -40 and 0 dB input, expected > 3 dB"
    );
    assert!(
        ratio > 1.3,
        "vca bus comp ratio {ratio:.4}:1, expected > 1.3:1"
    );
}

// ===========================================================================
// B2 — inverting op-amp stage with input coupling cap must pass audio
// ===========================================================================

/// Minimal inverting op-amp stage with an input coupling cap, mirroring the
/// fixture in pedalkernel/src/compiler/zero_output_tests.rs:262
/// (`minimal_opamp_with_input_network_produces_audio`). Ideal gain
/// -Rf/R_in = x10 (20 dB); the 1/(2*pi*10k*47n) = 339 Hz input HPF corner
/// costs ~1.4 dB at 440 Hz, so roughly x8-10 is expected.
const INVERTING_OPAMP_INPUT_CAP: &str = r#"
pedal "Inverting Opamp Input Cap" {
  supply 9V
  components {
    Cin: cap(47n)
    R_in: resistor(10k)
    U1: opamp(tl072)
    Rf: resistor(100k)
  }
  nets {
    in -> Cin.a
    Cin.b -> R_in.a
    R_in.b -> U1.neg
    U1.neg -> Rf.a
    Rf.b -> U1.out
    U1.pos -> gnd
    U1.out -> out
  }
  controls {}
}
"#;

/// [B2] A standalone inverting op-amp stage with an input coupling cap
/// (in -> 47n -> 10k -> U1.neg, 100k feedback) compiles to SILENCE today
/// (audit: peak 0.000000 on this exact topology — the same bug that forced
/// vca_bus_comp.pedal to use non-inverting stages). Fixed means: audio
/// passes (peak > 0.01) at the expected ~x10 gain minus the 339 Hz HPF
/// corner loss at 440 Hz. See reports/outboard-gear-audit-2026-06-12.md §3,
/// §6 G5.
#[test]
fn inverting_opamp_with_input_cap_passes_audio() {
    let input = sine_at(440.0, 0.05, 1.0, SAMPLE_RATE);
    let output = compile_and_process(INVERTING_OPAMP_INPUT_CAP, &input, SAMPLE_RATE, &[]);
    assert!(
        output.iter().all(|x| x.is_finite()),
        "inverting op-amp output contains NaN/inf"
    );

    let steady_peak = peak(&output[settle_samples()..]);
    eprintln!("inverting op-amp steady-state peak: {steady_peak:.6}");
    assert!(
        steady_peak > 0.01,
        "inverting op-amp stage is silent: steady-state peak {steady_peak:.6}, expected > 0.01"
    );

    let resp = frequency_response_db(
        || pedal_processor(INVERTING_OPAMP_INPUT_CAP, SAMPLE_RATE, &[]),
        &[440.0],
        0.05,
        SAMPLE_RATE,
    );
    let gain_db = resp[0].1;
    eprintln!("inverting op-amp gain at 440 Hz: {gain_db:+.3} dB (ideal x10 = +20 dB, HPF corner costs ~1.4 dB)");
    assert!(
        (gain_db - 20.0).abs() <= 3.0,
        "inverting op-amp gain {gain_db:+.3} dB at 440 Hz not within 3 dB of x10 (20 dB)"
    );
}

// ===========================================================================
// B3b — DC/resistor-coupled non-inverting op-amp cascade keeps both gains
// ===========================================================================

/// Two non-inverting gain-2 op-amp stages, DC-coupled through a 10k series
/// resistor. Ideal gain 2 x 2 = 4, non-inverting (output in phase with the
/// input).
const DC_COUPLED_NONINVERTING_CASCADE: &str = r#"
pedal "DC Coupled Noninverting Cascade" {
  supply 9V
  components {
    U1: opamp(tl072)
    R_g1: resistor(10k)
    R_f1: resistor(10k)
    R_link: resistor(10k)
    U2: opamp(tl072)
    R_g2: resistor(10k)
    R_f2: resistor(10k)
  }
  nets {
    in -> U1.pos
    U1.neg -> R_g1.a
    R_g1.b -> gnd
    R_f1.a -> U1.neg
    R_f1.b -> U1.out
    U1.out -> R_link.a
    R_link.b -> U2.pos
    U2.neg -> R_g2.a
    R_g2.b -> gnd
    R_f2.a -> U2.neg
    R_f2.b -> U2.out
    U2.out -> out
  }
  controls {}
}
"#;

/// [B3b] Two DC-coupled (10k series resistor) non-inverting gain-2 stages
/// must yield gain ~4 with positive polarity. Today the cascade loses a
/// stage's worth of gain (0.1 amp in -> peak 0.199 out, i.e. gain ~2 instead
/// of 4) — part of the op-amp coupling family of compile bugs that forced
/// vca_bus_comp.pedal to cap-couple its stages.
/// See reports/outboard-gear-audit-2026-06-12.md §3, §6 G5.
#[test]
fn dc_coupled_noninverting_cascade_has_positive_gain() {
    let input = sine_at(1_000.0, 0.1, 1.0, SAMPLE_RATE);
    let output = compile_and_process(DC_COUPLED_NONINVERTING_CASCADE, &input, SAMPLE_RATE, &[]);
    assert!(
        output.iter().all(|x| x.is_finite()),
        "DC-coupled cascade output contains NaN/inf"
    );

    let settle = settle_samples();
    let steady_peak = peak(&output[settle..]);
    let corr = correlation(&input[settle..], &output[settle..]);
    eprintln!("DC-coupled cascade steady-state peak: {steady_peak:.6} (expected ~0.4)");
    eprintln!("DC-coupled cascade input/output correlation: {corr:.4}");

    assert!(
        (0.3..=0.5).contains(&steady_peak),
        "DC-coupled cascade peak {steady_peak:.6} outside [0.3, 0.5] (gain-2 x gain-2 on 0.1 amp)"
    );
    assert!(
        corr > 0.5,
        "DC-coupled cascade polarity wrong: input/output correlation {corr:.4}, expected > 0.5"
    );
}

// ===========================================================================
// B3a — resistor + cap coupling between op-amp stages must pass audio
// ===========================================================================

/// Same non-inverting cascade, but coupled U1.out -> R_thru(30k) ->
/// C_mid(1u) -> U2.pos with the working bias pattern copied from
/// examples/outboard/compressor/vca_bus_comp.pedal (NE5532 stages, 30 V
/// supply, 1M bias resistor from the coupled node to vcc). This is the
/// canonical SSM2164 hookup's inter-stage network.
const RC_COUPLED_NONINVERTING_CASCADE: &str = r#"
pedal "RC Coupled Noninverting Cascade" {
  supply 30V
  components {
    U1: opamp(ne5532)
    R_g1: resistor(10k)
    R_f1: resistor(10k)
    R_thru: resistor(30k)
    C_mid: cap(1u)
    R_b2: resistor(1M)
    U2: opamp(ne5532)
    R_g2: resistor(30k)
    R_f2: resistor(30k)
  }
  nets {
    in -> U1.pos
    U1.neg -> R_g1.a
    R_g1.b -> gnd
    R_f1.a -> U1.neg
    R_f1.b -> U1.out
    U1.out -> R_thru.a
    R_thru.b -> C_mid.a
    C_mid.b -> R_b2.a, U2.pos
    R_b2.b -> vcc
    U2.neg -> R_g2.a
    R_g2.b -> gnd
    R_f2.a -> U2.neg
    R_f2.b -> U2.out
    U2.out -> out
  }
  controls {}
}
"#;

/// [B3a] Coupling two op-amp stages through a series resistor + cap
/// (U1.out -> 30k -> 1u -> U2.pos, biased) compiles to SILENCE today (audit:
/// peak 0.000000 — vca_bus_comp.pedal had to replace its intended R_thru
/// bridge with a plain wire). Fixed means audio passes: steady-state peak
/// > 0.1 for a 1 kHz 0.1-amp sine (ideal gain 4 -> ~0.4 peak).
/// See reports/outboard-gear-audit-2026-06-12.md §3, §6 G5.
#[test]
fn resistor_cap_coupled_opamp_cascade_passes_audio() {
    let input = sine_at(1_000.0, 0.1, 1.0, SAMPLE_RATE);
    let output = compile_and_process(RC_COUPLED_NONINVERTING_CASCADE, &input, SAMPLE_RATE, &[]);
    assert!(
        output.iter().all(|x| x.is_finite()),
        "RC-coupled cascade output contains NaN/inf"
    );

    let steady_peak = peak(&output[settle_samples()..]);
    eprintln!("RC-coupled cascade steady-state peak: {steady_peak:.6} (expected ~0.4)");
    assert!(
        steady_peak > 0.1,
        "RC-coupled op-amp cascade is silent: steady-state peak {steady_peak:.6}, expected > 0.1"
    );
}

// ===========================================================================
// B1 — dyna_comp must compile deterministically across processes
// ===========================================================================

/// [B1] examples/pedals/compressor/dyna_comp.pedal must compile to the same
/// processor in every process. Audit §4 measured run-to-run differences of
/// several dB (flat-linear ratio 1.00 / GR 0 dB on some runs, ratio 1.34 /
/// GR 4.2 dB on others), pointing at hash-ordering nondeterminism in
/// compilation; engine_determinism.rs only covers in-process determinism.
///
/// Self-spawning subprocess test: with PK_DETERMINISM_CHILD set, this test
/// becomes the child — it compiles dyna_comp, measures the static gain at
/// 1 kHz / 0.25 amplitude, prints it to stdout, and exits. The parent spawns
/// itself 4 times (escalating to 8 if the first 4 agree, since the flakiness
/// is intermittent), parses the children's gains, and asserts the max-min
/// spread is < 0.1 dB. See reports/outboard-gear-audit-2026-06-12.md §4,
/// §6 G9.
#[test]
fn dyna_comp_compiles_deterministically_across_processes() {
    const CHILD_ENV: &str = "PK_DETERMINISM_CHILD";
    const MARKER: &str = "PK_DET_GAIN_DB=";

    if std::env::var(CHILD_ENV).is_ok() {
        // --- Child: compile, measure static gain, report, exit. ---
        let src = example_pedal_source("dyna_comp.pedal");
        let controls: &[(&str, f64)] = &[("Sensitivity", 0.8), ("Output", 0.7)];
        let level_db = lin_to_db(0.25);
        let curve = static_gain_curve(
            || pedal_processor(&src, SAMPLE_RATE, controls),
            &[level_db],
            1_000.0,
            SAMPLE_RATE,
        );
        let gain_db = curve[0].1 - curve[0].0;
        println!("{MARKER}{gain_db:.9}");
        // Exit before the libtest harness prints its own summary.
        std::process::exit(0);
    }

    // --- Parent: spawn children, parse gains, assert tight spread. ---
    let exe = std::env::current_exe().expect("current_exe");
    let spawn_children = |n: usize, offset: usize| -> Vec<f64> {
        (0..n)
            .map(|i| {
                let idx = offset + i;
                let out = std::process::Command::new(&exe)
                    .args([
                        "--exact",
                        "dyna_comp_compiles_deterministically_across_processes",
                        "--nocapture",
                    ])
                    .env(CHILD_ENV, "1")
                    .output()
                    .expect("failed to spawn determinism child");
                let stdout = String::from_utf8_lossy(&out.stdout);
                let gain = stdout
                    .lines()
                    .find_map(|l| l.trim().strip_prefix(MARKER))
                    .and_then(|v| v.parse::<f64>().ok())
                    .unwrap_or_else(|| {
                        panic!(
                            "child {idx}: no gain marker in stdout:\n{stdout}\nstderr:\n{}",
                            String::from_utf8_lossy(&out.stderr)
                        )
                    });
                eprintln!("dyna comp determinism child {idx}: static gain {gain:.6} dB");
                gain
            })
            .collect()
    };
    let spread_of = |gains: &[f64]| -> f64 {
        let max = gains.iter().cloned().fold(f64::MIN, f64::max);
        let min = gains.iter().cloned().fold(f64::MAX, f64::min);
        max - min
    };

    let mut gains = spawn_children(4, 0);
    if spread_of(&gains) < 0.1 {
        // Intermittent flakiness: escalate to 8 children before concluding.
        eprintln!("dyna comp determinism: first 4 children agree, escalating to 8");
        gains.extend(spawn_children(4, 4));
    }

    let spread = spread_of(&gains);
    eprintln!("dyna comp determinism: gains {gains:?}, spread {spread:.6} dB");
    assert!(
        spread < 0.1,
        "dyna_comp compiles nondeterministically across processes: \
         gain spread {spread:.4} dB over {} children (gains {gains:?}), expected < 0.1 dB",
        gains.len()
    );
}

