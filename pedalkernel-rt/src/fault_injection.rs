//! Fault injection stubs for testing abnormal runtime conditions.
//!
//! All faults are disabled by default; tests enable them via environment variable
//! or compile-time toggle. This module is only compiled with `feature = "fault-injection"`.

/// A named fault that can be activated to force specific runtime misbehavior.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fault {
    /// Zero all warm-start voltage guesses, forcing cold NR convergence.
    BreakSolverWarmStart,
    /// Corrupt the scattering matrix entries with NaN.
    CorruptScattering,
    /// Disable the step-damping clamp in the NR solver.
    DisableStepDamping,
    /// Skip the denormal-flush pass on stage output.
    SkipDenormalFlush,
    /// Reverse diode polarity in WDF reflection.
    WrongDiodePolarity,
}

/// Returns `true` if the given fault is currently active.
///
/// Always returns `false` in production; tests set faults via the `PEDALKERNEL_FAULT` env var
/// or by calling [`activate`].
pub fn is_active(_fault: Fault) -> bool {
    // Stub: fault injection not yet wired to a registry.
    false
}
