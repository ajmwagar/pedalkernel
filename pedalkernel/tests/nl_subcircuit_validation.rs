//! Nonlinear subcircuit validation tests.
//!
//! Compiles minimal .pedal files that isolate each nonlinear topology
//! (RAT diode clipper, Klon germanium clipper) and validates:
//! - Non-silence, finite output, no denormals
//! - Monotonic voltage response (louder input → louder output)
//! - Solver convergence health via solver stats

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::elements::{
    disable_solver_trace, enable_solver_trace, reset_solver_stats, solver_stats_snapshot,
};

// ---------------------------------------------------------------------------
// RAT NL Block
// ---------------------------------------------------------------------------

#[test]
fn nl_subcircuit_rat_produces_output() {
    let input = sine(440.0, 0.5, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("rat_nl_block.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "RAT NL block", 50.0);
}

#[test]
fn nl_subcircuit_rat_monotonic_response() {
    let soft = sine_at(440.0, 0.05, 0.25, SAMPLE_RATE);
    let loud = sine_at(440.0, 0.5, 0.25, SAMPLE_RATE);

    let out_soft = compile_test_pedal_and_process("rat_nl_block.pedal", &soft, SAMPLE_RATE, &[]);
    let out_loud = compile_test_pedal_and_process("rat_nl_block.pedal", &loud, SAMPLE_RATE, &[]);

    let rms_soft = rms(&out_soft);
    let rms_loud = rms(&out_loud);

    assert!(
        rms_loud > rms_soft,
        "RAT NL block: louder input should produce louder output: \
         rms_soft={rms_soft:.6e}, rms_loud={rms_loud:.6e}"
    );
}

#[test]
fn nl_subcircuit_rat_solver_health() {
    reset_solver_stats();
    enable_solver_trace(50000);

    let input = sine(440.0, 0.5, SAMPLE_RATE);
    let _output = compile_test_pedal_and_process("rat_nl_block.pedal", &input, SAMPLE_RATE, &[]);

    let stats = solver_stats_snapshot();
    let trace = disable_solver_trace();

    // Solver should have been called many times (at least once per sample)
    assert!(stats.solves > 0, "No solver calls recorded");

    // Average iterations should be reasonable for a simple diode circuit
    if stats.solves > 0 {
        let avg_iter = stats.total_iterations as f64 / stats.solves as f64;
        assert!(
            avg_iter < 10.0,
            "RAT NL block: avg iterations too high: {avg_iter:.2}"
        );
    }

    // Check trace: no sample should need excessive iterations
    for (idx, entry) in trace.iter().enumerate() {
        assert!(
            entry.iterations <= 20,
            "RAT NL block: sample {idx} needed {} iterations (max 20 expected)",
            entry.iterations
        );
        assert!(
            entry.v_solution.iter().all(|v| v.is_finite()),
            "RAT NL block: sample {idx} has non-finite voltage solution"
        );
    }
}

// ---------------------------------------------------------------------------
// Klon NL Block
// ---------------------------------------------------------------------------

#[test]
fn nl_subcircuit_klon_produces_output() {
    let input = sine(440.0, 0.5, SAMPLE_RATE);
    let output = compile_test_pedal_and_process("klon_nl_block.pedal", &input, SAMPLE_RATE, &[]);
    assert_healthy(&output, "Klon NL block", 50.0);
}

#[test]
fn nl_subcircuit_klon_monotonic_response() {
    let soft = sine_at(440.0, 0.05, 0.25, SAMPLE_RATE);
    let loud = sine_at(440.0, 0.5, 0.25, SAMPLE_RATE);

    let out_soft = compile_test_pedal_and_process("klon_nl_block.pedal", &soft, SAMPLE_RATE, &[]);
    let out_loud = compile_test_pedal_and_process("klon_nl_block.pedal", &loud, SAMPLE_RATE, &[]);

    let rms_soft = rms(&out_soft);
    let rms_loud = rms(&out_loud);

    assert!(
        rms_loud > rms_soft,
        "Klon NL block: louder input should produce louder output: \
         rms_soft={rms_soft:.6e}, rms_loud={rms_loud:.6e}"
    );
}

#[test]
fn nl_subcircuit_klon_solver_health() {
    reset_solver_stats();
    enable_solver_trace(50000);

    let input = sine(440.0, 0.5, SAMPLE_RATE);
    let _output = compile_test_pedal_and_process("klon_nl_block.pedal", &input, SAMPLE_RATE, &[]);

    let stats = solver_stats_snapshot();
    let trace = disable_solver_trace();

    assert!(stats.solves > 0, "No solver calls recorded");

    if stats.solves > 0 {
        let avg_iter = stats.total_iterations as f64 / stats.solves as f64;
        assert!(
            avg_iter < 10.0,
            "Klon NL block: avg iterations too high: {avg_iter:.2}"
        );
    }

    for (idx, entry) in trace.iter().enumerate() {
        assert!(
            entry.iterations <= 20,
            "Klon NL block: sample {idx} needed {} iterations (max 20 expected)",
            entry.iterations
        );
        assert!(
            entry.v_solution.iter().all(|v| v.is_finite()),
            "Klon NL block: sample {idx} has non-finite voltage solution"
        );
    }
}
