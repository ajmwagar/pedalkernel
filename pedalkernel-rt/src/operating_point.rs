//! DC operating-point report types.
//!
//! A [`OperatingPoint`] is an optional, compile-time-computed snapshot of the
//! circuit's settled DC state: the voltage at every named net plus a per-device
//! Q-point (terminal voltages + collector/drain current) for every transistor.
//!
//! These types live in the runtime crate so [`crate::processor::CompiledPedal`]
//! can hold the report, but they are *populated* by the compiler (which is the
//! only place the circuit graph — net names and transistor pins — is available).
//! The numbers come from settling a freshly-built processor at DC (input 0) and
//! reading back node voltages; values that genuinely cannot be recovered from
//! the runtime representation are stored as `None` and rendered "n/a".

use alloc::string::String;
use alloc::vec::Vec;

/// A settled DC operating-point report for a compiled circuit.
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OperatingPoint {
    /// DC voltage at every named net, in declaration/graph order.
    /// `None` = the net's voltage could not be recovered from the runtime.
    pub nets: Vec<NetOp>,
    /// Per-device Q-point for every active device (BJT/JFET/MOSFET).
    pub devices: Vec<DeviceOp>,
    /// Settle duration actually used, in samples (for provenance).
    pub settle_samples: usize,
}

/// DC voltage at one named net.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct NetOp {
    pub name: String,
    /// Settled DC voltage (volts), or `None` if unrecoverable.
    pub voltage: Option<crate::Wave>,
    /// How the voltage was obtained (for honesty in the report).
    pub source: VoltageSource,
}

/// Provenance of a recovered node voltage.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum VoltageSource {
    /// A supply/ground rail with a known fixed voltage.
    Rail,
    /// Derived from a passive component voltage referenced to a rail.
    PassiveProbe,
    /// Could not be recovered.
    None,
}

impl VoltageSource {
    pub fn label(self) -> &'static str {
        match self {
            VoltageSource::Rail => "rail",
            VoltageSource::PassiveProbe => "probe",
            VoltageSource::None => "n/a",
        }
    }
}

/// Q-point for one active device.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DeviceOp {
    /// Component id (e.g. "Q1").
    pub id: String,
    /// Device kind label (e.g. "npn", "pnp", "njfet").
    pub kind: String,
    /// Terminal voltages: (pin name, voltage or None).
    pub terminals: Vec<(String, Option<crate::Wave>)>,
    /// Collector (BJT) / drain (FET) current, amps. `None` if unrecoverable.
    pub ic: Option<crate::Wave>,
}
