//! Diagnostics snapshot types for the MNA/scattering IPC channel (Phase A).
//!
//! These are **read-only, compile-time** snapshots captured from a built
//! [`crate::stage::MultiNlStage`] (the grouped-Newton coupled-NL stage). They
//! expose the scattering matrix, per-NL-port operating-point linearization
//! (`gm`), adapted port resistances, node-voltage extraction coefficients, and
//! the voltage-source injection vector — everything needed to reconstruct the
//! realized small-signal transfer (open-loop `A`, feedback `β`) by hand.
//!
//! ## no_std / std split
//!
//! This module lives in the `no_std + alloc` runtime crate and only produces
//! plain `Vec`/`String` data — **no** file or `mmap`. The std host (the
//! `pedalkernel` CLI) is responsible for serializing these structs into the
//! memory-mapped IPC file (see `pedalkernel`'s `diag_ipc` module). Keeping the
//! capture here lets embedded builds inspect the same structures without a
//! filesystem.
//!
//! The snapshot is intentionally **derived, not stored**: it is built on demand
//! from the stage's existing fields, so it adds no per-sample cost and does not
//! change runtime behavior.

use alloc::string::String;
use alloc::vec::Vec;

/// Per-NL-port operating-point linearization (the companion model `gm`).
///
/// Captured by evaluating each device's I-V at its warm-start voltage
/// `v_prev[port]`. For a grouped 2-port BJT the two ports are the
/// base-emitter (`Vbe`) and collector-emitter (`Vce`) ports; the relevant
/// transconductance for gain analysis is `dIc/dVbe`, reported as
/// `cross_gm` on the CE port.
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct NlPortLinearization {
    /// Human-readable port label, e.g. `"Q3.be"` / `"Q3.ce"`.
    pub label: String,
    /// Device family name (`Triode`, `BjtNpn2P`, `Diode`, ...).
    pub device: String,
    /// Operating-point voltage at this port (the NR warm-start `v_prev`).
    pub v_op: crate::Wave,
    /// Self conductance `di/dv` at the operating point (the companion `g`).
    /// For a port `k` this is the diagonal Jacobian term `∂i_k/∂v_k`.
    pub gm: crate::Wave,
    /// Companion small-signal resistance `1/gm` (∞ reported as `f64::INFINITY`).
    pub r_companion: crate::Wave,
    /// Cross transconductance for grouped multi-port devices.
    ///
    /// For a BJT CE port this is `∂Ic/∂Vbe` (the textbook `gm`); for a
    /// BE port it is `∂Ib/∂Vce`. `None` for single-port devices.
    pub cross_gm: Option<crate::Wave>,
    /// Adapted WDF port resistance for this NL port (`nl_port_resistances[k]`).
    pub port_resistance: crate::Wave,
}

/// A compile-time MNA / scattering snapshot of one `MultiNlStage`.
///
/// All matrices are row-major `n×n` (or `n×m`) flat `Vec`s; `*_labels` give the
/// interpretation. `s_full` is the **standard** (non-power-normalized) R-type
/// scattering matrix reconstructed from the adaptor.
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MnaStageSnapshot {
    /// Index of this stage in `CompiledPedal::stages`.
    pub stage_index: usize,
    /// Debug label for the stage (component ids), when available.
    pub label: String,
    /// Number of nonlinear ports.
    pub n_nl: usize,
    /// Number of reactive/passive one-ports folded into the adaptor.
    pub n_passive: usize,
    /// Total adaptor port count (`n_nl + n_passive + adapted/vcc extras`).
    pub n_ports_total: usize,

    /// Port labels for every adaptor port, in adaptor order
    /// (`[NL_0..NL_{n-1}, passive_0..passive_{m-1}, (vcc?), adapted]`).
    pub port_labels: Vec<String>,
    /// Per-port adaptor resistances (full adaptor ordering).
    pub port_resistances: Vec<crate::Wave>,

    /// Full **standard** scattering matrix `S` (`n_total × n_total`, row-major).
    ///
    /// Reconstructed from the adaptor's power-normalized matrix `S̄` via
    /// `S[i][j] = S̄[i][j] · √(R_i / R_j)`.
    pub s_full: Vec<crate::Wave>,
    /// Side length of `s_full` (== `n_ports_total`).
    pub s_dim: usize,

    /// Per-NL-port companion linearization (length `n_nl`).
    pub nl_linearization: Vec<NlPortLinearization>,

    /// Adapted NL port resistances (`nl_port_resistances`, length `n_nl`).
    pub nl_port_resistances: Vec<crate::Wave>,

    /// Node-voltage output extraction coefficients (`extract_coeffs`), if the
    /// stage reads its output via direct node-voltage extraction.
    pub extraction_coeffs: Option<Vec<crate::Wave>>,
    /// Extraction VS coefficient (`extract_vs`).
    pub extraction_vs: crate::Wave,

    /// Voltage-source injection vector (`vs_injection`), if the input is
    /// injected as an ideal VS rather than an adapted WDF port.
    pub vs_injection: Option<Vec<crate::Wave>>,

    /// Output NL port index (`output_port`).
    pub output_port: usize,
    /// Passive attenuation compensation factor (`compensation`).
    pub compensation: crate::Wave,
    /// Supply voltage used for DC bias (`supply_voltage`).
    pub supply_voltage: crate::Wave,
    /// Whether the stage is off the serial audio path (`bypass_serial`).
    pub bypass_serial: bool,
}

/// A complete diagnostics snapshot of a compiled pedal: every `MultiNlStage`'s
/// MNA snapshot plus a coarse description of the other stages.
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DiagSnapshot {
    /// Pedal name (set by the std host).
    pub pedal_name: String,
    /// Source file path (set by the std host).
    pub source_path: String,
    /// Sample rate the pedal was compiled at.
    pub sample_rate: crate::Wave,
    /// Total number of stages in `CompiledPedal::stages`.
    pub stage_count: usize,
    /// Per-stage kind labels (e.g. `Wdf`, `MultiNl`, `Iir`), in stage order.
    pub stage_kinds: Vec<String>,
    /// MNA snapshots for every `MultiNl` stage.
    pub multinl: Vec<MnaStageSnapshot>,
}
