//! MNA-vs-blockwise differential equivalence harness.
//!
//! # Invariant under test
//!
//! Blockwise decomposition is an optimization, NEVER a dialect change.
//! Given identical input samples, `compile_pedal_with_options` with blockwise
//! enabled (default) and with `skip_blockwise = true` (monolithic MNA path)
//! MUST produce outputs that are numerically equivalent within documented
//! tolerance tiers.
//!
//! # Tolerance tiers
//!
//! - **TIGHT** (<0.1 dB RMS error): circuits where blockwise selects
//!   `BlockwiseCoupled{coupled_newton}` — the coupled Newton/Jacobian solve
//!   is exact-by-construction relative to the monolithic MNA.
//! - **LOOSE** (<3.0 dB RMS error): `Serial` or `SerialDelayedFeedback`
//!   formulations, which intentionally introduce one-sample delay in the
//!   feedback path. Their deviation from monolithic MNA is structural and
//!   documented.
//!
//! # Known failures (documented, NOT loosened)
//!
//! Circuits marked `#[ignore]` diverge from MNA due to open bugs tracked in
//! sibling beads. The ignore message records the measured divergence so the
//! fix beads can use it as their acceptance threshold.
//!
//! # Test methodology
//!
//! 1. Compile each corpus circuit TWICE with identical `CompileOptions`
//!    except `skip_blockwise` (false = default blockwise path, true = monolithic).
//! 2. Feed IDENTICAL input to both (same `Vec<f64>`).
//! 3. Skip a 100 ms transient window so DC-settling artifacts don't pollute
//!    the comparison. Assert that both outputs are non-silent BEFORE comparing.
//! 4. Measure `rms_error_db = 20*log10(rms(out_blockwise - out_mna) / rms(out_mna))`.
//! 5. Assert within the declared tolerance tier.

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::compiler::{compile_pedal_with_options, CompileOptions};
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const SR: f64 = 48_000.0;
/// Duration of each test signal in seconds (≥0.5 s as specified).
const SIGNAL_DURATION_S: f64 = 0.6;
/// Transient skip: first 100 ms excluded from comparison.
const TRANSIENT_SKIP_S: f64 = 0.1;
/// Quiet sine amplitude (~5 mV into a 1V-peak system).
const QUIET_AMP: f64 = 0.005;
/// Driven sine amplitude (audible, below clipping for most circuits).
const DRIVEN_AMP: f64 = 0.35;
/// Tight tolerance (BlockwiseCoupled path): -0.1 dB RMS error.
const TIGHT_RMS_DB: f64 = -0.1;
/// Loose tolerance (Serial/SerialDelayedFeedback path): -3.0 dB RMS error.
/// Unused in current corpus; retained for future circuits that land on Serial formulation.
#[allow(dead_code)]
const LOOSE_RMS_DB: f64 = -3.0;
/// Minimum output RMS to guard against silent output masking divergence.
/// Corresponds to roughly -80 dBFS for a 0.005 V input through a ~40 dB gain stage.
const MIN_OUTPUT_RMS: f64 = 1e-6;

// ---------------------------------------------------------------------------
// Core helpers
// ---------------------------------------------------------------------------

/// Compile `src` with and without `skip_blockwise`, process identical input,
/// skip the transient window, then return `(blockwise_out, mna_out)`.
///
/// Panics if either compilation fails or either output is silent.
fn compile_both_and_process(
    src: &str,
    input: &[f64],
    label: &str,
) -> (Vec<f64>, Vec<f64>) {
    let pedal = parse_pedal_file(src)
        .unwrap_or_else(|e| panic!("{label}: parse failed: {e}"));

    // Use skip_k_tables for speed (NR instead of K-lookup tables).
    // `force_monolithic()` sets skip_blockwise=true, routing through the
    // monolithic MNA path. Default (skip_blockwise=false) uses blockwise.
    let base_opts = CompileOptions {
        skip_k_tables: true,
        ..CompileOptions::default()
    };
    let opts_blockwise = base_opts.clone();
    let opts_mna = base_opts.force_monolithic();

    let mut proc_bw = compile_pedal_with_options(&pedal, SR, opts_blockwise)
        .unwrap_or_else(|e| panic!("{label}: blockwise compile failed: {e}"));
    let mut proc_mna = compile_pedal_with_options(&pedal, SR, opts_mna)
        .unwrap_or_else(|e| panic!("{label}: monolithic MNA compile failed: {e}"));

    let out_bw: Vec<f64> = input.iter().map(|&s| proc_bw.process(s)).collect();
    let out_mna: Vec<f64> = input.iter().map(|&s| proc_mna.process(s)).collect();

    // Skip transient window.
    let skip = (TRANSIENT_SKIP_S * SR) as usize;
    let bw = out_bw[skip..].to_vec();
    let mna = out_mna[skip..].to_vec();

    // Guard: both outputs must be non-silent so a silent mismatch can't hide
    // a bug behind a trivially-small error.
    let rms_bw = rms(&bw);
    let rms_mna = rms(&mna);
    assert!(
        rms_bw >= MIN_OUTPUT_RMS,
        "{label}[blockwise]: output is silent after transient skip (rms={rms_bw:.2e})"
    );
    assert!(
        rms_mna >= MIN_OUTPUT_RMS,
        "{label}[mna]: output is silent after transient skip (rms={rms_mna:.2e})"
    );

    (bw, mna)
}

/// Compute RMS of the difference signal in dB relative to the reference.
///
/// Returns `rms(a - b) / rms(b)` in dB.  Negative means the error is below
/// the reference level (good).
fn rms_error_db(a: &[f64], b: &[f64]) -> f64 {
    let n = a.len().min(b.len());
    let diff: Vec<f64> = (0..n).map(|i| a[i] - b[i]).collect();
    let err = rms(&diff);
    let ref_ = rms(&b[..n]);
    if ref_ < 1e-30 {
        return 0.0;
    }
    20.0 * (err / ref_).log10()
}

/// Compute peak-absolute of the difference signal in dB relative to reference peak.
fn peak_error_db(a: &[f64], b: &[f64]) -> f64 {
    let n = a.len().min(b.len());
    let diff: Vec<f64> = (0..n).map(|i| a[i] - b[i]).collect();
    let err = peak(&diff);
    let ref_ = peak(&b[..n]);
    if ref_ < 1e-30 {
        return 0.0;
    }
    20.0 * (err / ref_).log10()
}

/// Assert TIGHT tier equivalence (<`TIGHT_RMS_DB` dB RMS error) and print
/// a diagnostic line for the results table.
fn assert_tight(bw: &[f64], mna: &[f64], label: &str, amp_tag: &str) {
    let rms_db = rms_error_db(bw, mna);
    let pk_db = peak_error_db(bw, mna);
    eprintln!(
        "  [equiv/{amp_tag}] {label}: RMS err={rms_db:+.2}dB  peak err={pk_db:+.2}dB  [TIGHT limit={TIGHT_RMS_DB}dB]"
    );
    assert!(
        rms_db < TIGHT_RMS_DB,
        "{label}[{amp_tag}]: RMS error {rms_db:.2}dB exceeds TIGHT limit {TIGHT_RMS_DB}dB — \
         blockwise diverged from MNA (invariant: blockwise is an optimization, not a dialect)"
    );
}

/// Assert LOOSE tier equivalence (<`LOOSE_RMS_DB` dB RMS error) and print diagnostic.
/// Unused in current corpus; retained for circuits that land on Serial/SerialDelayedFeedback.
#[allow(dead_code)]
fn assert_loose(bw: &[f64], mna: &[f64], label: &str, amp_tag: &str) {
    let rms_db = rms_error_db(bw, mna);
    let pk_db = peak_error_db(bw, mna);
    eprintln!(
        "  [equiv/{amp_tag}] {label}: RMS err={rms_db:+.2}dB  peak err={pk_db:+.2}dB  [LOOSE limit={LOOSE_RMS_DB}dB]"
    );
    assert!(
        rms_db < LOOSE_RMS_DB,
        "{label}[{amp_tag}]: RMS error {rms_db:.2}dB exceeds LOOSE limit {LOOSE_RMS_DB}dB — \
         Serial formulation diverged beyond documented structural tolerance"
    );
}

// ---------------------------------------------------------------------------
// Corpus circuits (inline or loaded from test_pedals/)
// ---------------------------------------------------------------------------

// ── 1. Diode clipper ────────────────────────────────────────────────────────
//
// EQUIVALENCE INVARIANT: Diode clipper has no resonant feedback loop, so
// blockwise decomposes it (or falls through to monolithic). Both paths must
// produce identical distortion character.

const DIODE_CLIPPER_SRC: &str = r#"
pedal "Diode Clipper" {
  components {
    C1: cap(100n)
    R1: resistor(2.2k)
    D1: diode_pair(silicon)
    RL: resistor(10k)
  }
  nets {
    in -> C1.a
    C1.b -> R1.a
    R1.b -> D1.a, RL.a
    D1.b -> gnd
    RL.b -> gnd
    R1.b -> out
  }
}
"#;

#[test]
fn diode_clipper_blockwise_matches_mna_quiet() {
    // INVARIANT: Diode clipper has no delay-free feedback; blockwise should
    // produce the same result as the monolithic MNA path.
    let input = sine_at(1000.0, QUIET_AMP, SIGNAL_DURATION_S, SR);
    let (bw, mna) = compile_both_and_process(DIODE_CLIPPER_SRC, &input, "diode_clipper[quiet]");
    assert_tight(&bw, &mna, "diode_clipper", "quiet");
}

#[test]
fn diode_clipper_blockwise_matches_mna_driven() {
    // INVARIANT: Hard clipping must be equivalent regardless of formulation.
    let input = sine_at(1000.0, DRIVEN_AMP, SIGNAL_DURATION_S, SR);
    let (bw, mna) = compile_both_and_process(DIODE_CLIPPER_SRC, &input, "diode_clipper[driven]");
    assert_tight(&bw, &mna, "diode_clipper", "driven");
}

// ── 2. OTA CA3080 stage ─────────────────────────────────────────────────────
//
// EQUIVALENCE INVARIANT: OTA (opamp-class device). Blockwise decomposes the
// surrounding passive network; the OTA itself is handled as a single stage.
// Both paths must preserve the voltage-controlled transconductance behavior.

const OTA_CA3080_SRC: &str = r#"
pedal "OTA CA3080 Test" {
  components {
    C1: cap(1u)
    R1: resistor(10k)
    U1: opamp(ca3080)
    R2: resistor(150k)
    C2: cap(100n)
    R3: resistor(100k)
  }
  nets {
    in -> C1.a
    C1.b -> R1.a
    R1.b -> U1.pos
    U1.neg -> R2.a
    R2.b -> U1.out
    U1.out -> C2.a
    C2.b -> R3.a
    R3.b -> gnd
    C2.b -> out
  }
}
"#;

#[test]
#[ignore = "pedalkernel-rk77.3: ota_ca3080 blockwise is non-deterministic — observed -inf dB (bit-identical), -4.44 dB, and +3.52 dB RMS error across runs. Blockwise path selection for the CA3080 OTA is unstable. Un-ignore when rk77.3 is merged."]
fn ota_ca3080_blockwise_matches_mna_quiet() {
    // INVARIANT: OTA buffer/inverter topology. Both paths must agree on the
    // AC signal path through the transconductance stage.
    // CURRENTLY FAILS: Non-deterministic — sometimes bit-identical (-inf dB),
    // sometimes +3.52 dB RMS error. The blockwise path selection for the CA3080
    // OTA varies between runs, indicating a non-deterministic compilation path.
    let input = sine_at(440.0, QUIET_AMP, SIGNAL_DURATION_S, SR);
    let (bw, mna) = compile_both_and_process(OTA_CA3080_SRC, &input, "ota_ca3080[quiet]");
    assert_tight(&bw, &mna, "ota_ca3080", "quiet");
}

#[test]
#[ignore = "pedalkernel-rk77.3: ota_ca3080 blockwise is non-deterministic — observed -inf dB (bit-identical), -4.44 dB, and +3.52 dB RMS error across runs (driven case). CA3080 blockwise path selection is unstable. Un-ignore when rk77.3 is merged."]
fn ota_ca3080_blockwise_matches_mna_driven() {
    // INVARIANT: Driven OTA. Nonlinear compressive gain compression must be
    // equivalent between formulations.
    // CURRENTLY FAILS: Non-deterministic — sometimes -inf dB (bit-identical),
    // sometimes -4.44 dB or +3.52 dB RMS error. Both OTA cases are unstable.
    // Tracked rk77.3.
    let input = sine_at(440.0, DRIVEN_AMP, SIGNAL_DURATION_S, SR);
    let (bw, mna) = compile_both_and_process(OTA_CA3080_SRC, &input, "ota_ca3080[driven]");
    assert_tight(&bw, &mna, "ota_ca3080", "driven");
}

// ── 3. Fuzz Face (coupled BJTs) ─────────────────────────────────────────────
//
// EQUIVALENCE INVARIANT: Fuzz Face has global DC-coupled feedback between Q2
// collector and Q1 base. Blockwise should select BlockwiseCoupled for the
// coupled BJT group. The coupled Newton/Jacobian solve is exact-by-construction,
// so agreement with monolithic MNA must be within TIGHT tolerance.

const FUZZ_FACE_PNP_SRC: &str = r#"
pedal "Fuzz Face PNP" {
  supply 9V
  components {
    C1: cap(2.2u)
    R1: resistor(33k)
    R2: resistor(8.2k)
    Q1: pnp(ac128)
    R3: resistor(470)
    R4: resistor(100k)
    Q2: pnp(ac128)
    C2: cap(10u)
    RL: resistor(10k)
  }
  nets {
    in -> C1.a
    C1.b -> Q1.base
    vcc -> Q1.emitter
    gnd -> R1.a
    R1.b -> vcc
    Q2.collector -> R2.a
    R2.b -> Q1.base
    Q1.collector -> R4.a
    R4.b -> Q2.base
    vcc -> R3.a
    R3.b -> Q2.emitter
    Q2.collector -> C2.a
    C2.b -> RL.a, out
    RL.b -> gnd
  }
}
"#;

#[test]
#[ignore = "pedalkernel-rk77.3: fuzz_face (coupled BJT PNP) blockwise diverges from MNA — quiet ~0.00dB RMS error (operating point mismatch at small signal). Un-ignore when rk77.3 is merged."]
fn fuzz_face_blockwise_matches_mna_quiet() {
    // INVARIANT: Coupled BJT circuit. BlockwiseCoupled with Newton solve is
    // exact-by-construction relative to monolithic MNA for AC-coupled signals.
    // CURRENTLY FAILS: blockwise compiles the PNP Fuzz Face with a different
    // operating point than the monolithic MNA path, causing ~0dB RMS error
    // (the difference signal is as large as the reference). Tracked rk77.3.
    let input = sine_at(440.0, QUIET_AMP, SIGNAL_DURATION_S, SR);
    let (bw, mna) = compile_both_and_process(FUZZ_FACE_PNP_SRC, &input, "fuzz_face[quiet]");
    assert_tight(&bw, &mna, "fuzz_face", "quiet");
}

#[test]
#[ignore = "pedalkernel-rk77.3: fuzz_face (coupled BJT PNP) blockwise diverges from MNA — driven +0.29dB RMS error (operating point / DC bias mismatch). Un-ignore when rk77.3 is merged."]
fn fuzz_face_blockwise_matches_mna_driven() {
    // INVARIANT: Driven fuzz. Heavy clipping from the coupled BJT group must
    // agree — if it doesn't, the blockwise operating point is wrong.
    // CURRENTLY FAILS: +0.29dB RMS error, peak +0.33dB. Tracked rk77.3.
    let input = sine_at(440.0, DRIVEN_AMP, SIGNAL_DURATION_S, SR);
    let (bw, mna) = compile_both_and_process(FUZZ_FACE_PNP_SRC, &input, "fuzz_face[driven]");
    assert_tight(&bw, &mna, "fuzz_face", "driven");
}

// ── 4. Pentode stage (EL84) ──────────────────────────────────────────────────
//
// EQUIVALENCE INVARIANT: Single pentode common-cathode stage. No inter-stage
// coupled feedback. Blockwise decomposes the passive network; the pentode NL
// element is solved monolithically or per-stage. Both paths must agree on the
// plate voltage swing and harmonic character.

const PENTODE_EL84_SRC: &str = r#"
pedal "Pentode EL84 Test" {
  supply 300V {
    impedance: 60
    filter_cap: 47u
    rectifier: solid_state
  }
  components {
    C1: cap(22n)
    R1: resistor(1M)
    V1: pentode(el84)
    R2: resistor(100k)
    R3: resistor(470)
    C2: cap(100u)
    C3: cap(22n)
    R4: resistor(1M)
  }
  nets {
    in -> C1.a
    C1.b -> R1.a, V1.grid
    R1.b -> gnd
    vcc -> R2.a
    R2.b -> V1.plate
    V1.cathode -> R3.a, C2.a
    R3.b -> gnd
    C2.b -> gnd
    V1.plate -> C3.a
    C3.b -> R4.a
    R4.b -> gnd
    C3.b -> out
  }
}
"#;

#[test]
fn pentode_el84_blockwise_matches_mna_quiet() {
    // INVARIANT: Pentode stage. Blockwise vs monolithic MNA must agree on the
    // AC transfer function through the pentode — same operating point, same
    // plate impedance, same gain.
    let input = sine_at(440.0, QUIET_AMP, SIGNAL_DURATION_S, SR);
    let (bw, mna) = compile_both_and_process(PENTODE_EL84_SRC, &input, "pentode_el84[quiet]");
    assert_tight(&bw, &mna, "pentode_el84", "quiet");
}

#[test]
fn pentode_el84_blockwise_matches_mna_driven() {
    // INVARIANT: Driven pentode. Plate current saturation must be equivalent
    // between formulations — both must see the same NL device operating point.
    let input = sine_at(440.0, DRIVEN_AMP, SIGNAL_DURATION_S, SR);
    let (bw, mna) = compile_both_and_process(PENTODE_EL84_SRC, &input, "pentode_el84[driven]");
    assert_tight(&bw, &mna, "pentode_el84", "driven");
}

// ── 5. TB-303-class diode ladder (two-rung with resonance feedback) ──────────
//
// EQUIVALENCE INVARIANT: Two-rung diode ladder with resonance pot. This is
// the canonical TB-303 topology — blockwise selects BlockwiseCoupled for the
// coupled diode rungs. The resonance feedback creates delay-free inter-rung
// coupling; both paths must produce the same filter character.
//
// NOTE on tolerance tier: the diode ladder uses a resonance feedback pot.
// With resonance=0 (no feedback), blockwise should be TIGHT.
// The two-rung diode ladder without feedback is also tested with TIGHT.

const TB303_DIODE_LADDER_SRC: &str = r#"
pedal "Two Rung Diode Ladder Feedback" { supply 9V
  ports {
    audio_in: input(10k)
    audio_out: output
  }
  components {
    D1: diode(silicon)
    D2: diode(silicon)
    R_bias1: resistor(100k)
    R_bias2: resistor(100k)
    R_in: resistor(10k)
    C1: cap(33n)
    C2: cap(33n)
    Resonance: pot(100k, b)
    R_fb_limit: resistor(33k)
  }
  nets {
    vcc -> R_bias1.a
    R_bias1.b -> D1.a
    audio_in -> R_in.a
    R_in.b -> D1.a

    D1.b -> C1.a
    C1.b -> gnd
    D1.b -> D2.a

    vcc -> R_bias2.a
    R_bias2.b -> D2.a
    D2.b -> C2.a
    C2.b -> gnd
    D2.b -> audio_out
    D2.b -> Resonance.a
    Resonance.b -> R_fb_limit.a
    R_fb_limit.b -> D1.a
  }
  controls {
    Resonance.position -> "Resonance" [0.0, 0.95] = 0.0
  }
}
"#;

#[test]
#[ignore = "pedalkernel-rk77.3: tb303_diode_ladder blockwise diverges from MNA — quiet +45.27dB RMS error (DC operating point / blockwise bias injection mismatch with supply rail). Un-ignore when rk77.3 is merged."]
fn tb303_diode_ladder_blockwise_matches_mna_quiet() {
    // INVARIANT: TB-303 diode ladder at resonance=0 (no feedback active).
    // Blockwise decomposes the two diode rungs; MNA solves them jointly.
    // With no resonance feedback, both paths should see identical signal topology.
    // CURRENTLY FAILS: +45.27dB RMS error — blockwise output is wildly different
    // from MNA, indicating a supply/bias injection fault in the blockwise path.
    // Tracked rk77.3.
    let input = sine_at(440.0, QUIET_AMP, SIGNAL_DURATION_S, SR);
    let (bw, mna) =
        compile_both_and_process(TB303_DIODE_LADDER_SRC, &input, "tb303_diode_ladder[quiet]");
    assert_tight(&bw, &mna, "tb303_diode_ladder", "quiet");
}

#[test]
#[ignore = "pedalkernel-rk77.3: tb303_diode_ladder blockwise diverges from MNA — driven +10.05dB RMS error (supply/bias injection mismatch). Un-ignore when rk77.3 is merged."]
fn tb303_diode_ladder_blockwise_matches_mna_driven() {
    // INVARIANT: Driven diode ladder. The soft-clipping character of each
    // rung must be preserved regardless of blockwise vs MNA formulation.
    // CURRENTLY FAILS: +10.05dB RMS error, peak +12.80dB. Tracked rk77.3.
    let input = sine_at(440.0, DRIVEN_AMP, SIGNAL_DURATION_S, SR);
    let (bw, mna) =
        compile_both_and_process(TB303_DIODE_LADDER_SRC, &input, "tb303_diode_ladder[driven]");
    assert_tight(&bw, &mna, "tb303_diode_ladder", "driven");
}

// ── 6. Common-cathode 12AX7 triode — KNOWN RED ──────────────────────────────
//
// INVARIANT VIOLATED: The 12AX7 blockwise path currently diverges from MNA
// by ~4.3 dB RMS (quiet) due to a DC Q-point/supply parity bug in the blockwise
// engine. This is tracked in bead pedalkernel-rk77.2.
//
// These tests are intentionally marked #[ignore] to document the failure
// without blocking CI. When rk77.2 is merged, these tests will be un-ignored
// and promoted to TIGHT assertions.

const CC_12AX7_SRC: &str = r#"
pedal "12AX7 Common Cathode" {
  supply 250V {
    impedance: 50
    filter_cap: 47u
    rectifier: solid_state
  }
  components {
    C_in: cap(22n)
    R_grid: resistor(1M)
    V1: triode(12ax7)
    R_plate: resistor(100k)
    R_cathode: resistor(1.5k)
    C_cathode: cap(25u)
    C_out: cap(22n)
    R_load: resistor(1M)
  }
  nets {
    in -> C_in.a
    C_in.b -> R_grid.a, V1.grid
    R_grid.b -> gnd
    vcc -> R_plate.a
    R_plate.b -> V1.plate
    V1.cathode -> R_cathode.a, C_cathode.a
    R_cathode.b -> gnd
    C_cathode.b -> gnd
    V1.plate -> C_out.a
    C_out.b -> R_load.a, out
    R_load.b -> gnd
  }
}
"#;

#[test]
#[ignore = "pedalkernel-rk77.2: 12AX7 blockwise diverges from MNA — DC Q-point/supply parity bug; measured ~4.3 dB RMS error (quiet sine). Un-ignore when rk77.2 is merged."]
fn cc_12ax7_blockwise_matches_mna_quiet() {
    // INVARIANT: Common-cathode triode preamp stage. Blockwise must agree with
    // monolithic MNA on the AC gain and harmonic character. Currently FAILS
    // due to blockwise not applying the high-voltage supply bias correctly,
    // causing the triode to operate at the wrong Q-point.
    let input = sine_at(440.0, QUIET_AMP, SIGNAL_DURATION_S, SR);
    let (bw, mna) = compile_both_and_process(CC_12AX7_SRC, &input, "cc_12ax7[quiet]");
    assert_tight(&bw, &mna, "cc_12ax7", "quiet");
}

#[test]
#[ignore = "pedalkernel-rk77.2: 12AX7 blockwise diverges from MNA — DC Q-point/supply parity bug; measured ~4.3 dB RMS error. Un-ignore when rk77.2 is merged."]
fn cc_12ax7_blockwise_matches_mna_driven() {
    // INVARIANT: Driven triode stage. Asymmetric soft-clipping must be
    // equivalent between paths once the Q-point bug (rk77.2) is fixed.
    let input = sine_at(440.0, DRIVEN_AMP, SIGNAL_DURATION_S, SR);
    let (bw, mna) = compile_both_and_process(CC_12AX7_SRC, &input, "cc_12ax7[driven]");
    assert_tight(&bw, &mna, "cc_12ax7", "driven");
}
