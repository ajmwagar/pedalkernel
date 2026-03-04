//! Deliberate fault injection for validating test tolerances.
//!
//! Each fault is a single-bit flag in an `AtomicU32` bitmask. When enabled,
//! the corresponding hook in production code injects a known error. Tests
//! enable a fault, run the operation, and assert the tolerance catches it.
//!
//! All hooks are behind `#[cfg(feature = "fault-injection")]` — zero cost
//! when the feature is disabled.

use std::sync::atomic::{AtomicU32, Ordering};

static ACTIVE_FAULTS: AtomicU32 = AtomicU32::new(0);

/// Faults that can be injected into the WDF engine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum Fault {
    /// Add 1e-3 to `a1` in `SeriesAdaptor::scatter_down`.
    CorruptScattering = 1 << 0,
    /// Skip denormal flushing — return `x` as-is.
    SkipDenormalFlush = 1 << 1,
    /// Zero `v_guess` on every NR call — destroy warm-start.
    BreakSolverWarmStart = 1 << 2,
    /// Negate diode pair current — break odd symmetry.
    WrongDiodePolarity = 1 << 3,
    /// Skip the `dv *= 0.5` step damping in NR solver.
    DisableStepDamping = 1 << 4,
}

/// Enable a specific fault.
pub fn enable_fault(f: Fault) {
    ACTIVE_FAULTS.fetch_or(f as u32, Ordering::SeqCst);
}

/// Disable a specific fault.
pub fn disable_fault(f: Fault) {
    ACTIVE_FAULTS.fetch_and(!(f as u32), Ordering::SeqCst);
}

/// Clear all active faults.
pub fn clear_faults() {
    ACTIVE_FAULTS.store(0, Ordering::SeqCst);
}

/// Check if a fault is active. Always returns `false` without the feature.
#[inline(always)]
pub fn is_active(f: Fault) -> bool {
    ACTIVE_FAULTS.load(Ordering::Relaxed) & (f as u32) != 0
}
