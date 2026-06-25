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
    /// Nonlinear-ROOT Q-point, when this device compiled to a WDF NL root
    /// (`RootKind::Bjt`, ...). `None` when the device folded to a passive
    /// approximation (e.g. PNP → PassiveRType dummies) with no NL root, in which
    /// case the report says so explicitly rather than rendering bare "n/a".
    #[cfg_attr(feature = "serde", serde(default))]
    pub root: Option<RootOp>,
}

/// Operating point read directly from a nonlinear WDF root after a DC settle.
///
/// This is the REAL device Q-point for an NL-root stage — distinct from the
/// passive per-net `terminals`/`ic` above, which are propagated through caps and
/// do NOT see the root's seeded bias or runtime-solved state.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RootOp {
    /// Root kind label (e.g. "Bjt").
    pub kind: String,
    /// Whether the compile-time DC Q-point solve resolved (`Some`) a seed.
    /// `false` ⇒ root left at vbe_bias = 0 (cutoff). The key bias diagnostic.
    pub seed_resolved: bool,
    /// Seeded base-emitter bias from compile-time DC analysis, if resolved.
    pub seeded_vbe: Option<crate::Wave>,
    /// Seeded collector-emitter warm-start from compile-time DC analysis.
    pub seeded_vce: Option<crate::Wave>,
    /// Current bias the root is actually holding (`BjtRoot::vbe_bias`).
    pub vbe_bias: Option<crate::Wave>,
    /// Runtime-solved Vce after the DC settle (root port voltage).
    pub solved_vce: Option<crate::Wave>,
    /// Runtime-solved collector current after the DC settle, amps.
    pub solved_ic: Option<crate::Wave>,
}
