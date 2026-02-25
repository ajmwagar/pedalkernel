//! Nonlinear WDF root elements: Diodes, JFETs, MOSFETs, Zener diodes, OTAs.
//!
//! These elements sit at the tree root and use Newton-Raphson iteration
//! to solve the implicit WDF constraint equation for the reflected wave.
//!
//! Also includes the slew rate limiter (models op-amp bandwidth limiting),
//! OTA (operational transconductance amplifier) for CA3080-based circuits,
//! and delay line implementations (BBD and generic).

mod solver;

mod bjt;
mod bbd;
mod delay;
mod diode;
mod jfet;
mod mosfet;
mod ota;
mod opamp;
mod pentode;
mod slew;
mod triode;

pub use bjt::*;
pub use bbd::*;
pub use delay::*;
pub use diode::*;
pub use jfet::*;
pub use mosfet::*;
pub use ota::*;
pub use opamp::*;
pub use pentode::*;
pub use slew::*;
pub use triode::*;
