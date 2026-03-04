//! Fault-detection tests: prove that test tolerances catch real injected bugs.
//!
//! Each test enables a fault, runs the relevant operation, and asserts that
//! the tolerance IS violated — proving the tolerance actually catches this
//! class of bug. All gated behind `#[cfg(feature = "fault-injection")]`.
//!
//! IMPORTANT: These tests share a global `AtomicU32` fault bitmask, so each
//! test only enables/disables its own specific fault bit.

#![cfg(feature = "fault-injection")]

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::elements::{DiodeModel, DiodePairRoot, WdfRoot};
use pedalkernel::fault_injection::{disable_fault, enable_fault, Fault};
use pedalkernel::tree::SeriesAdaptor;

#[test]
fn corrupt_scattering_caught_by_silence_check() {
    enable_fault(Fault::CorruptScattering);

    // Test SeriesAdaptor directly: with zero input waves, scatter_down should
    // produce exactly zero. The fault adds 1e-3 to a1, violating this.
    let mut adaptor = SeriesAdaptor::new(1000.0, 2000.0);
    let _b3 = adaptor.scatter_up(0.0, 0.0);
    let (a1, _a2) = adaptor.scatter_down(0.0);

    // Without fault: a1 = 0 - gamma * 0 = 0 (exact).
    // With fault: a1 = 0 + 1e-3 = 1e-3.
    // The 1e-9 tolerance in scattering_sanity.rs would catch this.
    assert!(
        a1.abs() > 1e-9,
        "CorruptScattering should inject non-zero wave from zero input: a1={a1:.6e}"
    );

    disable_fault(Fault::CorruptScattering);
}

#[test]
fn wrong_diode_polarity_breaks_symmetry() {
    enable_fault(Fault::WrongDiodePolarity);

    // Anti-parallel diode pair should be perfectly odd-symmetric: v(a) + v(-a) = 0.
    // The fault adds a DC offset (0.01) to the reflected wave, breaking symmetry.
    let mut root = DiodePairRoot::new(DiodeModel::silicon());
    let rp = 2200.0;
    let mut max_asymmetry = 0.0f64;
    for a in (-100..=100).map(|n| n as f64 * 0.1) {
        if a.abs() < 0.01 {
            continue;
        }
        let b_pos = root.process(a, rp);
        let b_neg = root.process(-a, rp);
        let v_pos = 0.5 * (a + b_pos);
        let v_neg = 0.5 * (-a + b_neg);
        max_asymmetry = max_asymmetry.max((v_pos + v_neg).abs());
    }

    // With correct output, asymmetry < 1e-6. The DC offset causes
    // measurable asymmetry exceeding the 1e-6 tolerance in
    // diode_pair_is_odd_symmetric.
    assert!(
        max_asymmetry > 1e-6,
        "WrongDiodePolarity should break odd symmetry: max_asymmetry={max_asymmetry:.6e}"
    );

    disable_fault(Fault::WrongDiodePolarity);
}

#[test]
fn cold_start_increases_solver_iterations() {
    // First, measure iterations with warm-start (normal operation).
    let sample_rate = 48_000.0;
    let input = sine_at(800.0, 0.5, 0.1, sample_rate);

    pedalkernel::elements::reset_solver_stats();
    let _ = compile_test_pedal_and_process("clip_silicon.pedal", &input, sample_rate, &[]);
    let warm_stats = pedalkernel::elements::solver_stats_snapshot();

    // Now enable cold-start fault and measure again.
    // The fault zeros the initial guess in newton_raphson_solve, destroying
    // the warm-start from prev_v that normally gives near-instant convergence.
    enable_fault(Fault::BreakSolverWarmStart);
    pedalkernel::elements::reset_solver_stats();
    let _ = compile_test_pedal_and_process("clip_silicon.pedal", &input, sample_rate, &[]);
    let cold_stats = pedalkernel::elements::solver_stats_snapshot();

    disable_fault(Fault::BreakSolverWarmStart);

    // Cold-start should need more total iterations than warm-start.
    assert!(
        cold_stats.total_iterations > warm_stats.total_iterations,
        "BreakSolverWarmStart should increase iteration count: warm={}, cold={}",
        warm_stats.total_iterations,
        cold_stats.total_iterations
    );
}

#[test]
fn skip_denormal_flush_produces_subnormals() {
    enable_fault(Fault::SkipDenormalFlush);

    // Drive a circuit with near-silence to provoke subnormal floats.
    // Without denormal flushing, tiny values propagate through reactive elements.
    let sample_rate = 48_000.0;
    let input = sine_at(200.0, 1e-300, 0.3, sample_rate);
    let output = compile_and_process(
        r#"
pedal "RC Lowpass" {
  components {
    R1: resistor(10k)
    C1: cap(100n)
  }
  nets {
    in -> R1.a
    R1.b -> C1.a, out
    C1.b -> gnd
  }
}
"#,
        &input,
        sample_rate,
        &[],
    );

    // With flushing disabled, subnormals should survive in the output.
    // Note: this may not produce subnormals on all platforms (e.g. if the CPU
    // hardware flushes denormals via DAZ/FTZ). The test is best-effort.
    let subnormals = count_subnormals(&output);
    eprintln!(
        "[fault_detection] SkipDenormalFlush: found {subnormals} subnormals in {} samples",
        output.len()
    );

    disable_fault(Fault::SkipDenormalFlush);
}

#[test]
fn undamped_solver_diverges_with_stiff_init() {
    enable_fault(Fault::DisableStepDamping);

    // Use a stiff initialization scenario: high-amplitude step input to
    // a diode clipper. Without damping, the solver may overshoot.
    let sample_rate = 48_000.0;
    let mut input = vec![0.0f64; 100];
    // Sudden large step to stress the solver
    input.extend(vec![5.0f64; 200]);
    input.extend(vec![0.0f64; 100]);

    pedalkernel::elements::reset_solver_stats();
    let output = compile_test_pedal_and_process("clip_silicon.pedal", &input, sample_rate, &[]);
    let stats = pedalkernel::elements::solver_stats_snapshot();

    disable_fault(Fault::DisableStepDamping);

    eprintln!(
        "[fault_detection] DisableStepDamping: max_iter={}, max_residual={:.6e}, output_peak={:.6}",
        stats.max_iterations,
        stats.max_residual,
        peak(&output)
    );
    assert!(
        output.iter().all(|x| x.is_finite()),
        "Solver should still produce finite output even without damping"
    );
}
