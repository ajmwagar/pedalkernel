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

/// Runtime audio/wave scalar.
///
/// Desktop builds use `f64` to preserve regression-test precision and analysis
/// output. Embedded Cortex-M builds use `f32`, matching the M7 FPU.
// `f32` on embedded ARM, or anywhere the `wave-f32` feature is set — the latter
// lets an f64 host compile a blob whose serialized scalar width matches an f32
// device (postcard is not self-describing, so the widths must agree). Compiler
// math still runs in f64; only the stored/serialized runtime values are f32.
#[cfg(any(all(target_arch = "arm", target_os = "none"), feature = "wave-f32"))]
pub type Wave = f32;
#[cfg(not(any(all(target_arch = "arm", target_os = "none"), feature = "wave-f32")))]
pub type Wave = f64;

/// Thermal voltage kT/q at the SPICE default analysis temperature
/// TEMP = TNOM = 27 °C (300.15 K), using the CODATA-2018 constants modern
/// ngspice uses (k = 1.380649e-23 J/K, q = 1.602176634e-19 C) ⇒ ≈ 25.8649 mV.
///
/// This is the single source of truth for every junction-device thermal
/// voltage (BJT, diode, JFET gate, OTA input pair). It must match ngspice
/// exactly: junction currents scale as exp(v/(N·Vt)), so the historical
/// 25 °C value (0.02585) left a systematic +0.9 % Ib / +1.3 % Ic error at
/// fixed junction voltages — which a shunt-feedback bias network (e.g. the
/// BA283's 56k/68k) amplifies into a ~40 mV DC-root offset. Validated to
/// <0.001 % per-device against ngspice `.op` in
/// `pedalkernel-validate/tests/ba283_ib_diagnostic.rs`.
///
/// Any code that *extracts* an ideality factor from a stored `n_vt` product
/// (e.g. thermal drift in `stage.rs::apply_thermal`) must divide by this
/// same constant, or the extracted N silently shifts.
pub const SPICE_VT_27C: Wave = 1.380649e-23 * 300.15 / 1.602176634e-19;

// ── Runtime math ─────────────────────────────────────────────────────────
// On std targets, f64 inherent methods (sin, cos, exp, etc.) are available.
// On no_std targets, we use libm. This module provides a uniform interface.
pub mod math;

// ── Fast math (LUT-based exp/tanh for embedded) ─────────────────────────
pub mod fast_math;

// ── Fault injection (testing only — always-false stubs) ─────────────────
#[cfg(feature = "fault-injection")]
pub mod fault_injection;

// ── Shared types (no dependencies) ───────────────────────────────────────
pub mod nonideal_fx;
pub mod pot_taper;

// ── Oversampling (antialiasing for nonlinear stages) ────────────────────
pub mod oversampling;

// ── Impedance loading (interstage, cable models) ──��─────────────────────
pub mod loading;

// ── Metering (lock-free audio->UI metrics) ──────────────────────────────
pub mod metering;

// ── Stage graph routing (compiled topology → runtime route plan) ─────────
pub mod route;
pub mod routing;

// ── Typed WDF boundary-drive math ──────────────────────────────────────
pub mod boundary_math;

// ── Thermal drift (temperature-dependent component behavior) ────────────
pub mod thermal;

// ── WDF circuit elements (leaf, root, modulator, controlled) ────────────
pub mod elements;

// ── WDF tree adaptors and MNA system ────────────────────────────────────
pub mod tree;

// ── Dynamic WDF leaf nodes and tree ────────────────────────────────────
pub mod dyn_node;
pub mod wdf_leaf;

// ── Runtime helpers (balance_parallel_vs, has_pot) ───────────────────
pub mod helpers;

// ── Subcircuit routing types ─────────────────────────────────────────
pub mod subcircuit;

// ── WDF stage processing ─────────────────────────────────────────────
pub mod stage;

// ── Parallel-branch convergence summation (F13b) ─────────────────────
pub mod convergence;

// ── Band-limited VCO with analog imperfections ──────────────────────
pub mod oscillator;

// ── One-pole exponential decay envelope (MEG/VEG) ──────────────────
pub mod envelope;

// ── One-pole pitch glide (portamento) ───────────────────────────────
pub mod glide;

// ── Compiled pedal processor (audio pipeline) ────────────────────────
pub mod processor;

// ── Diagnostics: compile-time MNA/scattering snapshot (IPC Phase A) ───
pub mod diag;

// ── Diagnostics: live runtime ring (IPC Phase B, `diag` feature) ──────
#[cfg(feature = "diag")]
pub mod diag_ring;

// ── Audio processor trait ────────────────────────────────────────────

use alloc::{string::String, vec::Vec};

// ── Named ports (audio-rate voltage I/O) ────────────────────────────

/// Direction of a named voltage port.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PortDirection {
    Input,
    Output,
}

/// Audio processor trait for pedals.
pub trait PedalProcessor {
    /// Process a single sample.
    fn process(&mut self, input: Wave) -> Wave;

    /// Set sample rate (call before processing).
    fn set_sample_rate(&mut self, rate: Wave);

    /// Reset all internal state.
    fn reset(&mut self);

    /// Set a named control parameter (0.0-1.0). Default: no-op.
    fn set_control(&mut self, _label: &str, _value: Wave) {}

    /// Set supply voltage in volts (default 9.0).
    fn set_supply_voltage(&mut self, _voltage: Wave) {}

    // ── Named port API (audio-rate voltage I/O) ─────────────────────

    /// Resolve a port name to its index in the ports slice.
    /// Call once at init to cache indices.
    fn resolve_port(&self, _name: &str) -> Option<usize> {
        None
    }

    /// Number of declared ports (for allocating the ports slice).
    fn port_count(&self) -> usize {
        0
    }

    /// Process one sample with named port I/O.
    ///
    /// `ports` is a mutable slice indexed by port index (from `resolve_port`).
    /// Write input port values before calling. Read output port values after.
    /// All ports are voltages. Audio rate, no impedance recompute.
    ///
    /// ```ignore
    /// let mut ports = vec![0.0; filter.port_count()];
    /// ports[audio_in] = input_sample;
    /// ports[cv_cutoff] = envelope_value;
    /// filter.process_ports(&mut ports);
    /// let output = ports[audio_out];
    /// ```
    fn process_ports(&mut self, _ports: &mut [Wave]) {}

    // ── Component introspection ─────────────────────────────────────

    /// List all editable passive components (R, C, L with comp_ids).
    /// Returns (comp_id, kind_str, current_value) for each editable leaf.
    fn list_editable_components(&self) -> Vec<(String, &'static str, Wave)> {
        Vec::new()
    }

    /// Set a passive component's value by comp_id. Returns true if found.
    fn set_passive(&mut self, _comp_id: &str, _value: Wave) -> bool {
        false
    }

    /// Reset a passive component to its original value. Returns true if found.
    fn reset_passive(&mut self, _comp_id: &str) -> bool {
        false
    }

    /// Debug: return current control state as (label, target_value, smoothed_value).
    fn control_debug_info(&self) -> Vec<(String, Wave, Wave)> {
        Vec::new()
    }
}
