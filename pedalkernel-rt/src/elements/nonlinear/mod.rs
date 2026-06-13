//! Nonlinear WDF root elements: Diodes, JFETs, MOSFETs, Zener diodes, OTAs.
//!
//! These elements sit at the tree root and use Newton-Raphson iteration
//! to solve the implicit WDF constraint equation for the reflected wave.
//!
//! Also includes the slew rate limiter (models op-amp bandwidth limiting),
//! OTA (operational transconductance amplifier) for CA3080-based circuits,
//! and delay line implementations (BBD and generic).

pub mod solver;

mod bbd;
mod bjt;
mod delay;
pub mod diode;
mod jfet;
mod jiles_atherton;
mod mosfet;
mod opamp;
mod ota;
mod pentode;
mod slew;
mod spring;
mod triode;
mod vari_mu;

pub use bbd::*;
pub use bjt::*;
pub use delay::*;
pub use diode::*;
pub use jfet::*;
pub use jiles_atherton::*;
pub use mosfet::*;
pub use opamp::*;
pub use ota::*;
pub use pentode::*;
pub use slew::*;
pub use spring::*;
pub use triode::*;
pub use vari_mu::*;

pub use solver::{
    disable_solver_trace, enable_solver_trace, reset_solver_stats, solver_stats_snapshot,
    SolverMethod as DiodeSolverMethod, SolverStatsSnapshot, SolverTraceEntry,
};
