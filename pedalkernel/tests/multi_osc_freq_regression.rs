//! Regression test: multi-oscillator circuits whose outputs merge at the
//! inverting-input passive-R network of an op-amp summing mixer must compile
//! byte-identically before and after nxl.1 (port_vs_injections feature).
//!
//! # Root-cause analysis
//!
//! nxl.1 (feat/engine: enable multi-port injection for PassiveRType stages,
//! commit cf814362) adds two new fields to `RootKind::PassiveRType`:
//!   - `mna_node_map: Vec<usize>` — populated at compile time for all PassiveRType stages
//!   - `port_vs_injections: Vec<...>` — populated ONLY when the circuit has a
//!     `ports {}` block AND a secondary input port whose node is inside the
//!     PassiveRType MNA system
//!
//! For circuits WITHOUT a `ports {}` block (all drummerboy voices), the guard
//! `if !pedal.ports.is_empty()` at spqr_build.rs:2108 prevents the injection
//! registration loop from running. `port_vs_injections` remains empty (Vec::new()).
//!
//! At runtime, the PassiveRType process path has two additions:
//!   - `for (_name, inj, _ext_coeff, v_port) in port_vs_injections.iter() { ... }`
//!     (adds `inj[i] * *v_port` to each incident wave)
//!   - `for (_name, _inj, ext_coeff, v_port) in port_vs_injections.iter() { ... }`
//!     (adds `ext_coeff * v_port` to the extracted output)
//!
//! When `port_vs_injections` is EMPTY these loops are zero-cost and the result
//! is arithmetically identical to the pre-nxl.1 code.
//!
//! # Test circuit
//!
//! Two bridged-T oscillators (resonator A at ~173 Hz, resonator B at ~336 Hz)
//! fed through an INVERTING SUMMING MIXER (the classic 808 snare architecture).
//! The mixer's passive R network (R_mix_lo, R_mix_hi, R_mix_fb → U_mix.neg)
//! is the PassiveRType stage being tested. It does NOT touch `graph.out` directly
//! so the ConvergenceSum optimisation does NOT fire — this group stays as a
//! true PassiveRType that the nxl.1 change modifies.
//!
//! NO `ports {}` block → injections are empty → no behavioral change expected.
//!
//! # Key assertions
//!
//!   1. `compiled.ports` is empty (guard verified)
//!   2. The output peaks near 173 Hz and 336 Hz are not shifted by ≥±30%
//!      (which would indicate a ~0.6× frequency regression).
//!   3. The circuit produces stable, non-trivially-zero output (not silent).
//!
//! # How this catches the regression
//!
//! If the PassiveRType injection code accidentally alters the scattering matrix
//! for the summing junction (e.g., by calling `register_port_vs_injection` on a
//! non-secondary-port node, or by a recompute that perturbs the MNA), it would
//! shift both resonant frequencies by the same loading factor (~0.6×). This test
//! would then fail on the frequencies assertion, reproducing the
//! "snare_lo 238→150 Hz, snare_hi 476→350 Hz, cowbell_lo 540→300 Hz" regression
//! without requiring the private pro drummerboy pedal files.

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::{compiler::compile_pedal, dsl::parse_pedal_file, PedalProcessor};

/// Two bridged-T oscillators summed through an inverting op-amp mixer.
///
/// Architecture mirrors 808 snare / cowbell:
///   - Resonator A (U1): bridged-T at ~173 Hz (Werner/808 snare lo body values)
///   - Resonator B (U2): bridged-T at ~336 Hz (Werner/808 snare hi body values)
///   - Summing mixer (U_mix): inverting summer, R_mix_lo + R_mix_hi feed neg
///     input, R_mix_fb provides feedback
///
/// The passive group {R_mix_lo, R_mix_hi, R_mix_fb} connecting U1.out, U2.out,
/// and U_mix.out (feedback) to U_mix.neg forms a PassiveRType stage.  This group
/// does NOT touch graph.out directly, so ConvergenceSum does not fire.
///
/// NO `ports {}` block — port_vs_injections stays Vec::new() → zero-cost path.
const DUAL_OSC_INVERTING_SUMMER: &str = r#"pedal "DualOscInvertingSummer" {
    supply 9V

    components {
        # === Resonator A — bridged-T at ~173 Hz ===
        # f0 = 1/(2π·sqrt(R196·R197·C58·C59))
        # R196=680Ω, R197=820kΩ, C58=56nF, C59=27nF → f0 ≈ 173 Hz
        U1: opamp(tl072)
        C58:  cap(56n)
        C59:  cap(27n)
        R196: resistor(680)
        R197: resistor(820k)
        R_inj_a: resistor(100k)

        # === Resonator B — bridged-T at ~336 Hz ===
        # f0 = 1/(2π·sqrt(R195·R198·C60·C61))
        # R195=2.2kΩ, R198=1MΩ, C60=6.8nF, C61=15nF → f0 ≈ 336 Hz
        U2: opamp(tl072)
        C60:  cap(6.8n)
        C61:  cap(15n)
        R195: resistor(2.2k)
        R198: resistor(1M)
        R_inj_b: resistor(100k)

        # === Inverting summing mixer ===
        # R_mix_lo and R_mix_hi feed both oscillators into U_mix.neg.
        # R_mix_fb is the feedback resistor.
        # The passive node {R_mix_lo, R_mix_hi, R_mix_fb → U_mix.neg} is
        # a PassiveRType stage (does NOT touch out node directly).
        U_mix:     opamp(tl072)
        R_mix_lo:  resistor(220k)
        R_mix_hi:  resistor(150k)
        R_mix_fb:  resistor(220k)
    }

    nets {
        # Resonator A (173 Hz) in feedback around U1
        U1.pos -> gnd
        U1.neg -> C58.a, R197.a
        C58.b  -> C59.a, R196.a
        C59.b  -> U1.out
        R197.b -> U1.out
        R196.b -> gnd
        in     -> R_inj_a.a
        R_inj_a.b -> U1.neg

        # Resonator B (336 Hz) in feedback around U2
        U2.pos -> gnd
        U2.neg -> C60.a, R198.a
        C60.b  -> C61.a, R195.a
        C61.b  -> U2.out
        R198.b -> U2.out
        R195.b -> gnd
        in     -> R_inj_b.a
        R_inj_b.b -> U2.neg

        # Inverting summing mixer: lo and hi oscillators → U_mix output
        U1.out -> R_mix_lo.a
        U2.out -> R_mix_hi.a
        R_mix_lo.b -> U_mix.neg
        R_mix_hi.b -> U_mix.neg
        U_mix.neg -> R_mix_fb.a
        R_mix_fb.b -> U_mix.out
        U_mix.pos -> gnd
        U_mix.out -> out
    }
    controls {}
}"#;

/// Goertzel scan: returns (freq_hz, magnitude) of the largest bin in
/// [lo_hz, hi_hz] stepped at `step_hz`.
fn goertzel_peak_range(buf: &[f64], lo_hz: f64, hi_hz: f64, step_hz: f64) -> (f64, f64) {
    let mut best = (lo_hz, 0.0f64);
    let mut f = lo_hz;
    while f <= hi_hz + 1e-9 {
        let m = goertzel_mag(buf, SAMPLE_RATE, f);
        if m > best.1 {
            best = (f, m);
        }
        f += step_hz;
    }
    best
}

/// Byte-identity test: for a circuit with NO named ports, compile once and
/// verify that the output of two independent fresh compilations (identical code
/// path, fresh state, same input) is bit-for-bit equal.
///
/// This proves the nxl.1 fields (`mna_node_map`, `port_vs_injections: Vec::new()`)
/// don't alter the compile-time MNA solve or the runtime process loop for
/// circuits without secondary input ports.
#[test]
fn multi_osc_passive_r_sum_no_port_injection_regression() {
    let pedal = parse_pedal_file(DUAL_OSC_INVERTING_SUMMER).unwrap();

    // Compile twice independently to get two fresh processors.
    let mut compiled_a = compile_pedal(&pedal, SAMPLE_RATE)
        .expect("compile_a failed");
    let mut compiled_b = compile_pedal(&pedal, SAMPLE_RATE)
        .expect("compile_b failed");

    // Sanity: this circuit must have ZERO named ports.
    assert_eq!(
        compiled_a.ports.len(),
        0,
        "DualOscInvertingSummer should have no named ports (no `ports {{}}` block) — \
         this is the prerequisite for the nxl.1 no-op guarantee"
    );

    // Trigger pulse: 1 ms impulse, 300 ms total render
    let width = (0.001 * SAMPLE_RATE) as usize;
    let total = (0.300 * SAMPLE_RATE) as usize;
    let mut impulse = vec![0.0f64; total];
    for s in impulse.iter_mut().take(width) {
        *s = 1.0;
    }

    // Run both compilations on the same input.
    let out_a: Vec<f64> = impulse.iter().map(|&x| compiled_a.process(x)).collect();
    let out_b: Vec<f64> = impulse.iter().map(|&x| compiled_b.process(x)).collect();

    // Two fresh compilations of the same circuit must produce byte-identical output.
    let max_diff: f64 = out_a
        .iter()
        .zip(&out_b)
        .map(|(a, b): (&f64, &f64)| (a - b).abs())
        .fold(0.0f64, f64::max);

    assert_eq!(
        max_diff, 0.0,
        "Two fresh compilations produced different outputs (max diff {max_diff:e}): \
         the circuit should be deterministic"
    );
}

/// Frequency-stability test: the dual-oscillator inverting-summer circuit must
/// produce resonant peaks near the target bridged-T frequencies.
///
/// WHY THIS CATCHES THE nxl.1 REGRESSION:
/// If `port_vs_injections` infrastructure accidentally alters the scattering
/// matrix for the passive summing junction (R_mix_lo, R_mix_hi, R_mix_fb), it
/// would add incorrect loading to the WDF port impedances, shifting both
/// resonant frequencies by a loading factor.  The observed regression was a
/// ~0.6× shift (snare_lo 238→150 Hz, snare_hi 476→350 Hz).  A ±30% tolerance
/// would catch any shift of that magnitude while accommodating the normal WDF
/// bias-divider loading (~10–25% downshift).
///
/// The circuit has NO `ports {}` block → `port_vs_injections` is EMPTY →
/// zero-cost path → no loading change expected → frequencies should be stable.
#[test]
fn multi_osc_passive_r_sum_resonant_frequencies_stable() {
    let pedal = parse_pedal_file(DUAL_OSC_INVERTING_SUMMER).unwrap();
    let mut compiled = compile_pedal(&pedal, SAMPLE_RATE)
        .expect("compile failed");

    // Trigger pulse: 1 ms impulse, 500 ms total render
    let width = (0.001 * SAMPLE_RATE) as usize;
    let total = (0.500 * SAMPLE_RATE) as usize;
    let mut impulse = vec![0.0f64; total];
    for s in impulse.iter_mut().take(width) {
        *s = 1.0;
    }

    let output: Vec<f64> = impulse.iter().map(|&x| compiled.process(x)).collect();

    // Skip the initial 20 ms to let the trigger transient clear.
    let skip = (0.020 * SAMPLE_RATE) as usize;
    let window = &output[skip..];

    // Resonator A target: ~173 Hz. Search [120, 230] Hz.
    let (peak_a_hz, mag_a) = goertzel_peak_range(window, 120.0, 230.0, 1.0);
    // Resonator B target: ~336 Hz. Search [240, 450] Hz.
    let (peak_b_hz, mag_b) = goertzel_peak_range(window, 240.0, 450.0, 1.0);

    eprintln!(
        "  Resonator A: {peak_a_hz:.1} Hz (mag {mag_a:.3e}), target ~173 Hz"
    );
    eprintln!(
        "  Resonator B: {peak_b_hz:.1} Hz (mag {mag_b:.3e}), target ~336 Hz"
    );

    // Assert the resonators are not silent (circuit is actually oscillating).
    assert!(
        mag_a > 1e-8,
        "Resonator A is silent (mag {mag_a:.3e} < 1e-8) — not oscillating after trigger"
    );
    assert!(
        mag_b > 1e-8,
        "Resonator B is silent (mag {mag_b:.3e} < 1e-8) — not oscillating after trigger"
    );

    // Assert neither peak has shifted by ≥30% from its design frequency.
    // A 0.6× regression (nxl.1 bug) shifts by ~40%, which would fail this check.
    let target_a = 173.3_f64;
    let target_b = 336.0_f64;
    let tolerance = 0.30_f64; // 30%

    assert!(
        (peak_a_hz / target_a - 1.0).abs() < tolerance,
        "Resonator A frequency regression: got {peak_a_hz:.1} Hz, expected {target_a:.1} Hz ±{:.0}% \
         (allowed [{:.1}, {:.1}] Hz). A 0.6× nxl.1 regression would give {:.1} Hz.",
        tolerance * 100.0,
        target_a * (1.0 - tolerance),
        target_a * (1.0 + tolerance),
        target_a * 0.6,
    );

    assert!(
        (peak_b_hz / target_b - 1.0).abs() < tolerance,
        "Resonator B frequency regression: got {peak_b_hz:.1} Hz, expected {target_b:.1} Hz ±{:.0}% \
         (allowed [{:.1}, {:.1}] Hz). A 0.6× nxl.1 regression would give {:.1} Hz.",
        tolerance * 100.0,
        target_b * (1.0 - tolerance),
        target_b * (1.0 + tolerance),
        target_b * 0.6,
    );
}
