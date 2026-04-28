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
