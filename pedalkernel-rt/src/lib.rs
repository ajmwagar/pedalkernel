//! PedalKernel Runtime — `no_std` WDF audio engine.
//!
//! This crate contains everything needed to **process** audio at runtime:
//! WDF tree scattering, nonlinear NR solvers, stage processing, pot smoothing,
//! and oversampling. It is `no_std + alloc` so it can run on embedded targets
//! (Cortex-M7, etc.) as well as desktop.
//!
//! The **compiler** (DSL parsing, SPQR decomposition, circuit graph analysis)
//! lives in the `pedalkernel` crate, which depends on this crate to construct
//! runtime types.
//!
//! # Features
//!
//! - `std` (default): Enables debug output (`eprintln!`), solver statistics
//!   (`thread_local!`), and native math intrinsics. Disable for embedded.
//! - `debug-trace`: Verbose per-sample tracing. Zero-cost when disabled.

#![no_std]
extern crate alloc;

#[cfg(feature = "std")]
extern crate std;

// ── Runtime math ─────────────────────────────────────────────────────────
// On std targets, f64 inherent methods (sin, cos, exp, etc.) are available.
// On no_std targets, we use libm. This module provides a uniform interface.
pub mod math;

// ── Fast math (LUT-based exp/tanh for embedded) ─────────────────────────
pub mod fast_math;

// ── Shared types (no dependencies) ───────────────────────────────────────
pub mod pot_taper;
pub mod nonideal_fx;

// ── Oversampling (antialiasing for nonlinear stages) ────────────────────
pub mod oversampling;

// ── Impedance loading (interstage, cable models) ──��─────────────────────
pub mod loading;

// ── Metering (lock-free audio->UI metrics) ──────────────────────────────
pub mod metering;

// ── Thermal drift (temperature-dependent component behavior) ────────────
pub mod thermal;

// ── WDF circuit elements (leaf, root, modulator, controlled) ────────────
pub mod elements;

// ── WDF tree adaptors and MNA system ────────────────────────────────────
pub mod tree;

// ── Dynamic WDF leaf nodes and tree ────────────────────────────────────
pub mod wdf_leaf;
pub mod dyn_node;

// ── Runtime helpers (balance_parallel_vs, has_pot) ───────────────────
pub mod helpers;

// ── Subcircuit routing types ─────────────────────────────────────────
pub mod subcircuit;

// ── WDF stage processing ─────────────────────────────────────────────
pub mod stage;

// ── Compiled pedal processor (audio pipeline) ────────────────────────
pub mod processor;

// ── Audio processor trait ────────────────────────────────────────────

use alloc::{string::String, vec::Vec};

/// Audio processor trait for pedals.
pub trait PedalProcessor {
    /// Process a single sample.
    fn process(&mut self, input: f64) -> f64;

    /// Set sample rate (call before processing).
    fn set_sample_rate(&mut self, rate: f64);

    /// Reset all internal state.
    fn reset(&mut self);

    /// Set a named control parameter (0.0-1.0). Default: no-op.
    fn set_control(&mut self, _label: &str, _value: f64) {}

    /// Set supply voltage in volts (default 9.0).
    fn set_supply_voltage(&mut self, _voltage: f64) {}

    /// List all editable passive components (R, C, L with comp_ids).
    /// Returns (comp_id, kind_str, current_value) for each editable leaf.
    fn list_editable_components(&self) -> Vec<(String, &'static str, f64)> {
        Vec::new()
    }

    /// Set a passive component's value by comp_id. Returns true if found.
    fn set_passive(&mut self, _comp_id: &str, _value: f64) -> bool {
        false
    }

    /// Reset a passive component to its original value. Returns true if found.
    fn reset_passive(&mut self, _comp_id: &str) -> bool {
        false
    }

    /// Debug: return current control state as (label, target_value, smoothed_value).
    fn control_debug_info(&self) -> Vec<(String, f64, f64)> {
        Vec::new()
    }
}
