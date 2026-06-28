//! WDF clipping/processing stage combining a tree with a nonlinear root.

extern crate alloc;
use alloc::{format, string::String, vec, vec::Vec};

use crate::elements::*;
use crate::oversampling::Oversampler;
use crate::tree::{MnaSystem, RTypeAdaptor, ScatteringInterpolationTable, WdfPort};
use crate::{PedalProcessor, Wave};

use crate::boundary_math::{
    scatter_matrix_into, sum_incident_offsets, BoundaryIncidentDrive, CircuitMappedPort,
    ExtractionProbe, GraphNodeId, LinearMultiportNetwork, LtiStateMap, MnaNodeId, MnaOnePort,
    MnaPortTerminals, MnaVariableResistorBinding, OnePortKind, OnePortState, RolePortBinding,
    RuntimeOnePort, RuntimeState, ScatteringPortId, WdfPortTerminals, WdfWaveCache,
};
use crate::dyn_node::DynNode;
use crate::helpers::balance_parallel_vs;
use crate::route::{BindingId, PortBinding};

// ═══════════════════════════════════════════════════════════════════════════
// VsPtr: Send/Sync-safe raw pointer to a VS leaf's voltage field
// ═══════════════════════════════════════════════════════════════════════════

/// A raw pointer to a `WdfVoltageSource::voltage` field inside the WDF tree.
///
/// Resolved once at construction time via `DynNode::resolve_main_vs_ptr()` /
/// `resolve_port_vs_ptr()`. At runtime, `set()` writes directly — no tree
/// walk, no string comparison, no recursion.
///
/// # Safety
/// The pointer targets a heap-allocated `Box<DynNode>` child, so it remains
/// stable as long as the owning `WdfStage` (and its tree) is not dropped.
/// The tree structure is never modified after construction.
#[derive(Clone, Copy)]
pub struct VsPtr(*mut crate::Wave);

// SAFETY: VsPtr points into a Box<DynNode> owned by the same WdfStage.
// The stage is only accessed from one thread (audio thread). No shared
// mutation — the pointer is written only by the owning stage's process().
unsafe impl Send for VsPtr {}
unsafe impl Sync for VsPtr {}

impl VsPtr {
    /// Create a new VsPtr from a raw pointer.
    #[inline(always)]
    pub fn new(ptr: *mut crate::Wave) -> Self {
        Self(ptr)
    }

    /// Write a voltage value directly to the VS leaf.
    /// # Safety
    /// Caller must ensure the pointer is still valid (tree not dropped).
    #[inline(always)]
    pub unsafe fn set(&self, v: crate::Wave) {
        *self.0 = v;
    }

    /// Read the current voltage value from the VS leaf.
    /// # Safety
    /// Caller must ensure the pointer is still valid.
    #[inline(always)]
    pub unsafe fn get(&self) -> crate::Wave {
        *self.0
    }
}

impl core::fmt::Debug for VsPtr {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "VsPtr({:p})", self.0)
    }
}

/// Flush denormals to zero. Subnormal floats are 100x slower to process
/// on x86 and serve no useful purpose in audio signals.
#[inline(always)]
pub fn flush_denormal(x: crate::Wave) -> crate::Wave {
    #[cfg(feature = "fault-injection")]
    if crate::fault_injection::is_active(crate::fault_injection::Fault::SkipDenormalFlush) {
        return x;
    }
    if x.is_subnormal() {
        0.0
    } else {
        x
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// NonIdealFxState: shared GBW + slew + rail post-processing
// ═══════════════════════════════════════════════════════════════════════════

/// Pre-computed runtime state for component-declared non-idealities.
///
/// Constructed from [`NonIdealFx`] values at compile time, applied per-sample
/// at runtime. All stage types that contain an op-amp use this — one
/// implementation, called at the physically correct point in each stage:
///
/// - [`BlackFeedbackStage`]: after gain multiplication
/// - [`IirStage`]: after biquad filtering
/// - `OpAmpRoot` (NL path): inside WDF scatter, before diode NR solver
///
/// No heap allocations. All fields are scalars.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct NonIdealFxState {
    /// GBW lowpass coefficient: α = 2π·fc / (2π·fc + fs).
    pub gbw_coeff: crate::Wave,
    /// Single-pole IIR state for GBW rolloff.
    pub gbw_state: crate::Wave,
    /// Maximum voltage change per sample (slew_rate_V_per_us * 1e6 / sample_rate).
    pub max_dv: crate::Wave,
    /// Previous output sample for slew rate limiting.
    pub prev_out: crate::Wave,
    /// Positive rail saturation voltage (tanh soft clip ceiling).
    pub v_rail_pos: crate::Wave,
    /// Negative rail saturation voltage.
    pub v_rail_neg: crate::Wave,
}

impl Default for NonIdealFxState {
    /// Default: passthrough (no GBW limiting, no slew, no rails).
    fn default() -> Self {
        Self {
            gbw_coeff: 1.0,
            gbw_state: 0.0,
            max_dv: crate::Wave::MAX,
            prev_out: 0.0,
            v_rail_pos: crate::Wave::MAX,
            v_rail_neg: crate::Wave::MAX,
        }
    }
}

impl NonIdealFxState {
    /// Construct from SPICE model parameters.
    ///
    /// - `gbw`: gain-bandwidth product in Hz
    /// - `slew_rate`: maximum dV/dt in V/µs
    /// - `v_max`: symmetric rail saturation voltage in V
    /// - `gain`: closed-loop gain (for fc = GBW/gain)
    /// - `sample_rate`: in Hz
    pub fn from_params(
        gbw: crate::Wave,
        slew_rate: crate::Wave,
        v_max: crate::Wave,
        gain: crate::Wave,
        sample_rate: crate::Wave,
    ) -> Self {
        let gain_abs = gain.abs().max(1.0);
        let fc = gbw / gain_abs;
        let w = 2.0 * crate::math::PI * fc;
        Self {
            gbw_coeff: w / (w + sample_rate),
            gbw_state: 0.0,
            max_dv: slew_rate * 1e6 / sample_rate,
            prev_out: 0.0,
            v_rail_pos: v_max,
            v_rail_neg: v_max,
        }
    }

    /// Construct from Component trait's NonIdealFx declarations.
    pub fn from_nonideal_fx(
        fx: &[crate::nonideal_fx::NonIdealFx],
        gain: crate::Wave,
        sample_rate: crate::Wave,
    ) -> Self {
        let mut state = Self::default();
        for effect in fx {
            match effect {
                crate::nonideal_fx::NonIdealFx::OpAmpBandwidth { gbw, slew_rate } => {
                    let gain_abs = gain.abs().max(1.0);
                    let fc = gbw / gain_abs;
                    let w = 2.0 * crate::math::PI * fc;
                    state.gbw_coeff = w / (w + sample_rate);
                    state.max_dv = slew_rate * 1e6 / sample_rate;
                }
                crate::nonideal_fx::NonIdealFx::RailSaturation { v_max } => {
                    state.v_rail_pos = *v_max;
                    state.v_rail_neg = *v_max;
                }
            }
        }
        state
    }

    /// Reset runtime state (for recomputation after gain change).
    pub fn reset(&mut self) {
        self.gbw_state = 0.0;
        self.prev_out = 0.0;
    }

    /// Update GBW coefficient for a new gain value (pot changed Rf).
    pub fn update_gain(
        &mut self,
        gbw: crate::Wave,
        new_gain: crate::Wave,
        sample_rate: crate::Wave,
    ) {
        let gain_abs = new_gain.abs().max(1.0);
        let fc = gbw / gain_abs;
        let w = 2.0 * crate::math::PI * fc;
        self.gbw_coeff = w / (w + sample_rate);
    }

    // ── Single source of truth: GBW / slew / rail primitives ──────────────
    //
    // These three `*_step` methods are the ONLY implementations of the
    // op-amp single-pole GBW lowpass, the dV/dt slew clamp, and the tanh
    // rail soft-clip in the entire crate. Every op-amp path — the post-FX
    // path (`apply_nonideal_fx` / `process_post`), the in-loop linear gain
    // stage (`BlackFeedbackStage`), the IIR stage, the WDF `OpAmpRoot` NL
    // root, and the legacy `SlewRateLimiter` — routes through these.

    /// Single-pole GBW lowpass step. Advances `gbw_state`.
    #[inline]
    pub fn gbw_step(&mut self, sample: crate::Wave) -> crate::Wave {
        let out = self.gbw_coeff * sample + (1.0 - self.gbw_coeff) * self.gbw_state;
        self.gbw_state = out;
        out
    }

    /// dV/dt slew clamp toward `target` from the previous (real) output.
    ///
    /// This is THE dV/dt clamp. `prev_out` carries the actual limited output,
    /// so when used in-loop the feedback senses the genuinely slew-limited
    /// node. Advances `prev_out`.
    #[inline]
    pub fn slew_step(&mut self, target: crate::Wave) -> crate::Wave {
        let dv = target - self.prev_out;
        let out = if dv > self.max_dv {
            self.prev_out + self.max_dv
        } else if dv < -self.max_dv {
            self.prev_out - self.max_dv
        } else {
            target
        };
        self.prev_out = out;
        out
    }

    /// Asymmetric tanh rail soft-clip. Stateless. This is THE tanh rail.
    #[inline]
    pub fn rail_step(&self, sample: crate::Wave) -> crate::Wave {
        if sample > 0.0 && self.v_rail_pos < crate::Wave::MAX {
            self.v_rail_pos * crate::fast_math::fast_tanh(sample / self.v_rail_pos)
        } else if sample < 0.0 && self.v_rail_neg < crate::Wave::MAX {
            -self.v_rail_neg * crate::fast_math::fast_tanh(-sample / self.v_rail_neg)
        } else {
            sample
        }
    }
}

/// Apply NonIdealFx post-processing to a sample.
///
/// GBW rolloff → slew rate limiting → rail saturation (tanh soft clip).
/// Called at the correct point by each stage type.
/// No heap allocations, no branching on stage type.
#[inline]
pub fn apply_nonideal_fx(sample: crate::Wave, state: &mut NonIdealFxState) -> crate::Wave {
    // GBW rolloff → slew rate limiting → rail saturation, all via the shared
    // single-source-of-truth primitives on NonIdealFxState.
    let out = state.gbw_step(sample);
    let out = state.slew_step(out);
    let out = state.rail_step(out);
    flush_denormal(out)
}

// ═══════════════════════════════════════════════════════════════════════════

/// Compute adaptive NR tolerance based on input signal change rate.
///
/// During transients (large delta), loosen tolerance to reduce iterations.
/// During steady-state (small delta), use tight tolerance for accuracy.
/// The linear interpolation avoids a discontinuity at the threshold boundary.
#[inline]
fn adaptive_nr_tolerance(input_delta: crate::Wave) -> crate::Wave {
    const TIGHT: crate::Wave = 1e-6;
    const LOOSE: crate::Wave = 1e-4;
    const TRANSIENT_THRESHOLD: crate::Wave = 0.1; // ~-20dBFS delta

    if input_delta.abs() > TRANSIENT_THRESHOLD {
        LOOSE
    } else {
        // Smooth interpolation between tight and loose
        let t = (input_delta.abs() / TRANSIENT_THRESHOLD).min(1.0);
        TIGHT + t * (LOOSE - TIGHT)
    }
}

/// Stamp (or unstamp) a conductance value into an MNA G matrix.
///
/// Adds `g` to the diagonal entries and subtracts from off-diagonal.
/// Use negative `g` to unstamp (remove) a previous stamp.
#[inline]
fn stamp_g(
    g_matrix: &mut [crate::Wave],
    n: usize,
    n1: Option<usize>,
    n2: Option<usize>,
    g: crate::Wave,
) {
    if let Some(i) = n1 {
        g_matrix[i * n + i] += g;
        if let Some(j) = n2 {
            g_matrix[i * n + j] -= g;
        }
    }
    if let Some(j) = n2 {
        g_matrix[j * n + j] += g;
        if let Some(i) = n1 {
            g_matrix[j * n + i] -= g;
        }
    }
}

#[cfg(feature = "debug-trace")]
use core::sync::atomic::{AtomicU64, Ordering};

/// Count of traced push-pull samples. Only active with `debug-trace` feature.
#[cfg(feature = "debug-trace")]
static TRACE_COUNT_PP: AtomicU64 = AtomicU64::new(0);

/// Max push-pull samples to trace (only non-zero signals are counted).
#[cfg(feature = "debug-trace")]
const MAX_TRACE_PP: u64 = 10;

/// Count of traced multi-NL stage samples. Only active with `debug-trace` feature.
#[cfg(feature = "debug-trace")]
static TRACE_COUNT_MNL: AtomicU64 = AtomicU64::new(0);

/// Max multi-NL stage samples to trace.
#[cfg(feature = "debug-trace")]
const MAX_TRACE_MNL: u64 = 20;

// ═══════════════════════════════════════════════════════════════════════════
// K-method lookup table for NL roots
// ═══════════════════════════════════════════════════════════════════════════

/// Precomputed lookup table replacing Newton-Raphson for a WDF NL root.
///
/// Maps incident wave (b_tree) → reflected wave (a_root) for a fixed port
/// resistance. Built at compile time by evaluating the NL device's I-V curve
/// across the operating domain.
///
/// 1D: single junction (diode pair, single diode) — indexed by b_tree only.
/// 2D: two-port device (BJT, triode) — indexed by (b_tree, control_voltage).
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct KTable {
    /// Number of input dimensions (1 for diode, 2 for BJT/triode).
    pub dims: usize,
    /// Min/max of the b_tree input domain.
    pub b_min: crate::Wave,
    pub b_max: crate::Wave,
    /// Min/max of the second dimension (control voltage). Unused for 1D.
    pub ctrl_min: crate::Wave,
    pub ctrl_max: crate::Wave,
    /// Number of steps per dimension.
    pub steps: usize,
    /// Flat table entries: a_root values.
    /// 1D: [steps] entries.
    /// 2D: [steps × steps] entries, row-major (b varies fastest).
    pub entries: alloc::vec::Vec<Wave>,
    /// Precomputed (steps-1) / (b_max - b_min). Eliminates per-sample division.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub inv_b_scale: crate::Wave,
    /// Precomputed (steps-1) / (ctrl_max - ctrl_min). Eliminates per-sample division.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub inv_c_scale: crate::Wave,
}

impl KTable {
    /// Precompute cached inverse scales after construction or deserialization.
    /// Must be called once before first lookup.
    pub fn precompute_scales(&mut self) {
        let b_range = self.b_max - self.b_min;
        let c_range = self.ctrl_max - self.ctrl_min;
        let n = (self.steps.max(2) - 1) as crate::Wave;
        self.inv_b_scale = if b_range.abs() > 1e-15 {
            n / b_range
        } else {
            0.0
        };
        self.inv_c_scale = if c_range.abs() > 1e-15 {
            n / c_range
        } else {
            0.0
        };
    }

    /// 1D lookup: interpolate a_root from b_tree.
    #[inline(always)]
    pub fn lookup_1d(&self, b_tree: crate::Wave) -> crate::Wave {
        if self.steps < 2 || self.entries.is_empty() {
            return 0.0;
        }
        // inv_b_scale = (steps-1) / (b_max - b_min), precomputed.
        // p = (b_tree - b_min) * inv_b_scale, clamped to [0, steps-1].
        let n_max = (self.steps - 1) as Wave;
        let p = (((b_tree as Wave) - self.b_min as Wave) * self.inv_b_scale as Wave)
            .clamp(0.0 as Wave, n_max);
        let i = (p as usize).min(self.steps - 2);
        let frac = p - i as Wave;
        let y0 = self.entries[i];
        let y1 = self.entries[i + 1];
        (y0 + frac * (y1 - y0)) as crate::Wave
    }

    /// 1D lookup plus local derivative da_root/db_tree.
    #[inline(always)]
    pub fn lookup_1d_with_derivative(&self, b_tree: crate::Wave) -> (crate::Wave, crate::Wave) {
        if self.steps < 2 || self.entries.is_empty() {
            return (0.0, 0.0);
        }
        let n_max = (self.steps - 1) as Wave;
        let p = (((b_tree as Wave) - self.b_min as Wave) * self.inv_b_scale as Wave)
            .clamp(0.0 as Wave, n_max);
        let i = (p as usize).min(self.steps - 2);
        let frac = p - i as Wave;
        let y0 = self.entries[i];
        let y1 = self.entries[i + 1];
        let slope = (y1 - y0) as crate::Wave * self.inv_b_scale;
        ((y0 + frac * (y1 - y0)) as crate::Wave, slope)
    }

    /// 2D lookup: interpolate a_root from (b_tree, control_voltage).
    #[inline(always)]
    pub fn lookup_2d(&self, b_tree: crate::Wave, ctrl: crate::Wave) -> crate::Wave {
        if self.steps < 2 || self.entries.is_empty() {
            return 0.0;
        }
        // Guard: if this is a 1D table, fall back to 1D lookup
        if self.dims == 1 {
            return self.lookup_1d(b_tree);
        }
        // Precomputed: inv_b_scale = (steps-1)/(b_max-b_min), same for ctrl.
        // Eliminates 2 divisions per sample (div ~10 cycles vs mul ~3 cycles).
        let n_max = (self.steps - 1) as Wave;
        let pb = (((b_tree as Wave) - self.b_min as Wave) * self.inv_b_scale as Wave)
            .clamp(0.0 as Wave, n_max);
        let pc = (((ctrl as Wave) - self.ctrl_min as Wave) * self.inv_c_scale as Wave)
            .clamp(0.0 as Wave, n_max);
        let ib = (pb as usize).min(self.steps - 2);
        let ic = (pc as usize).min(self.steps - 2);
        let fb = pb - ib as Wave;
        let fc = pc - ic as Wave;
        let s = self.steps;
        let v00 = self.entries[ib + ic * s];
        let v10 = self.entries[ib + 1 + ic * s];
        let v01 = self.entries[ib + (ic + 1) * s];
        let v11 = self.entries[ib + 1 + (ic + 1) * s];
        // Bilinear interpolation (4 muls + 3 adds)
        let t0 = v00 + fb * (v10 - v00);
        let t1 = v01 + fb * (v11 - v01);
        (t0 + fc * (t1 - t0)) as crate::Wave
    }

    /// 2D lookup plus local derivatives `(value, d/db_tree, d/dctrl)`.
    #[inline(always)]
    pub fn lookup_2d_with_derivatives(
        &self,
        b_tree: crate::Wave,
        ctrl: crate::Wave,
    ) -> (crate::Wave, crate::Wave, crate::Wave) {
        if self.steps < 2 || self.entries.is_empty() {
            return (0.0, 0.0, 0.0);
        }
        if self.dims == 1 {
            let (value, db) = self.lookup_1d_with_derivative(b_tree);
            return (value, db, 0.0);
        }
        let n_max = (self.steps - 1) as Wave;
        let pb = (((b_tree as Wave) - self.b_min as Wave) * self.inv_b_scale as Wave)
            .clamp(0.0 as Wave, n_max);
        let pc = (((ctrl as Wave) - self.ctrl_min as Wave) * self.inv_c_scale as Wave)
            .clamp(0.0 as Wave, n_max);
        let ib = (pb as usize).min(self.steps - 2);
        let ic = (pc as usize).min(self.steps - 2);
        let fb = pb - ib as Wave;
        let fc = pc - ic as Wave;
        let s = self.steps;
        let v00 = self.entries[ib + ic * s];
        let v10 = self.entries[ib + 1 + ic * s];
        let v01 = self.entries[ib + (ic + 1) * s];
        let v11 = self.entries[ib + 1 + (ic + 1) * s];
        let t0 = v00 + fb * (v10 - v00);
        let t1 = v01 + fb * (v11 - v01);
        let value = t0 + fc * (t1 - t0);
        let d_db =
            ((1.0 as Wave - fc) * (v10 - v00) + fc * (v11 - v01)) as crate::Wave * self.inv_b_scale;
        let d_dc =
            ((1.0 as Wave - fb) * (v01 - v00) + fb * (v11 - v10)) as crate::Wave * self.inv_c_scale;
        (value as crate::Wave, d_db, d_dc)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NonlinearSolverKind {
    Newton,
    KTable,
}

/// Contract for audio-rate nonlinear root solvers.
///
/// Implementations must be allocation-free and callable from realtime code.
/// The runtime uses a concrete enum implementation (`NonlinearSolver`) rather
/// than `dyn` dispatch; the trait documents the solver contract shared by WDF
/// stages and blockwise stages.
pub trait NonlinearSolverContract {
    fn kind(&self) -> NonlinearSolverKind;
    fn solve_root_incident(&self, b_tree: crate::Wave, control: crate::Wave)
        -> Option<crate::Wave>;
    fn solve_root_incident_with_derivatives(
        &self,
        b_tree: crate::Wave,
        control: crate::Wave,
    ) -> Option<(crate::Wave, crate::Wave, crate::Wave)>;
}

#[derive(Debug, Clone, Copy)]
pub enum NonlinearSolver<'a> {
    Newton,
    KTable(&'a KTable),
}

impl<'a> NonlinearSolver<'a> {
    #[inline(always)]
    pub fn from_k_table(table: Option<&'a KTable>) -> Self {
        match table {
            Some(table) => Self::KTable(table),
            None => Self::Newton,
        }
    }
}

impl NonlinearSolverContract for NonlinearSolver<'_> {
    #[inline(always)]
    fn kind(&self) -> NonlinearSolverKind {
        match self {
            Self::Newton => NonlinearSolverKind::Newton,
            Self::KTable(_) => NonlinearSolverKind::KTable,
        }
    }

    #[inline(always)]
    fn solve_root_incident(
        &self,
        b_tree: crate::Wave,
        control: crate::Wave,
    ) -> Option<crate::Wave> {
        match self {
            Self::Newton => None,
            Self::KTable(table) => Some(if table.dims == 1 {
                table.lookup_1d(b_tree)
            } else {
                table.lookup_2d(b_tree, control)
            }),
        }
    }

    #[inline(always)]
    fn solve_root_incident_with_derivatives(
        &self,
        b_tree: crate::Wave,
        control: crate::Wave,
    ) -> Option<(crate::Wave, crate::Wave, crate::Wave)> {
        match self {
            Self::Newton => None,
            Self::KTable(table) => Some(if table.dims == 1 {
                let (value, d_b) = table.lookup_1d_with_derivative(b_tree);
                (value, d_b, 0.0)
            } else {
                table.lookup_2d_with_derivatives(b_tree, control)
            }),
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// ADAA (Antiderivative Antialiasing) for K-tables
// ═══════════════════════════════════════════════════════════════════════════

impl KTable {
    /// Compute the antiderivative table from the existing entries.
    ///
    /// The antiderivative A(b) = ∫₀ᵇ f(x) dx is approximated via
    /// trapezoidal integration over the table entries. Stored as a
    /// companion vector with the same grid spacing.
    ///
    /// For 1D tables: `antideriv[i] = ∫_{b_min}^{b_i} f(x) dx`
    /// For 2D tables: integrates along the b axis for each ctrl slice.
    pub fn compute_antiderivative(&self) -> Vec<crate::Wave> {
        let s = self.steps;
        if s < 2 || self.entries.is_empty() {
            return Vec::new();
        }
        let db = (self.b_max - self.b_min) / (s - 1) as crate::Wave;

        if self.dims == 1 {
            let mut ad = vec![0.0; s];
            for i in 1..s {
                // Trapezoidal: A[i] = A[i-1] + (f[i-1] + f[i]) / 2 * db
                ad[i] = ad[i - 1]
                    + (self.entries[i - 1] as crate::Wave + self.entries[i] as crate::Wave)
                        * 0.5
                        * db;
            }
            ad
        } else {
            // 2D: integrate along b for each ctrl row
            let mut ad = vec![0.0; s * s];
            for ic in 0..s {
                for ib in 1..s {
                    let idx = ib + ic * s;
                    let prev = ib - 1 + ic * s;
                    ad[idx] = ad[prev]
                        + (self.entries[prev] as crate::Wave + self.entries[idx] as crate::Wave)
                            * 0.5
                            * db;
                }
            }
            ad
        }
    }

    /// 1D ADAA lookup: bandwidth-limited evaluation.
    ///
    /// Uses first-order antiderivative antialiasing:
    /// `y = (A(b_new) - A(b_old)) / (b_new - b_old)` when inputs differ,
    /// falls back to direct `f(b_new)` when inputs are equal (avoids 0/0).
    ///
    /// `antideriv` must be the output of `compute_antiderivative()`.
    /// `b_prev` is the previous sample's b_tree value.
    #[inline(always)]
    pub fn lookup_1d_adaa(
        &self,
        b_tree: crate::Wave,
        b_prev: crate::Wave,
        antideriv: &[crate::Wave],
    ) -> crate::Wave {
        let delta = b_tree - b_prev;
        if delta.abs() < 1e-7 {
            // Inputs nearly equal — use direct evaluation (L'Hôpital)
            return self.lookup_1d(b_tree);
        }
        let a_new = self.interp_table(b_tree, antideriv);
        let a_old = self.interp_table(b_prev, antideriv);
        (a_new - a_old) / delta
    }

    /// 2D ADAA lookup: bandwidth-limited evaluation with control voltage.
    ///
    /// ADAA along the b axis; ctrl axis uses standard interpolation.
    /// `b_prev` is the previous sample's b_tree value.
    #[inline(always)]
    pub fn lookup_2d_adaa(
        &self,
        b_tree: crate::Wave,
        b_prev: crate::Wave,
        ctrl: crate::Wave,
        antideriv: &[crate::Wave],
    ) -> crate::Wave {
        let delta = b_tree - b_prev;
        if delta.abs() < 1e-7 {
            return self.lookup_2d(b_tree, ctrl);
        }
        let a_new = self.interp_2d_table(b_tree, ctrl, antideriv);
        let a_old = self.interp_2d_table(b_prev, ctrl, antideriv);
        (a_new - a_old) / delta
    }

    /// Interpolate from an arbitrary table (antiderivative or other) using
    /// the same grid as self.entries. 1D version.
    #[inline(always)]
    fn interp_table(&self, b: crate::Wave, table: &[crate::Wave]) -> crate::Wave {
        if self.steps < 2 || table.len() < self.steps {
            return 0.0;
        }
        let p = ((b - self.b_min) * self.inv_b_scale).clamp(0.0, (self.steps - 1) as crate::Wave);
        let i = (p as usize).min(self.steps - 2);
        let frac = p - i as crate::Wave;
        table[i] + frac * (table[i + 1] - table[i])
    }

    /// Interpolate from an arbitrary 2D table using the same grid.
    #[inline(always)]
    fn interp_2d_table(
        &self,
        b: crate::Wave,
        ctrl: crate::Wave,
        table: &[crate::Wave],
    ) -> crate::Wave {
        if self.steps < 2 || table.len() < self.steps * self.steps {
            return 0.0;
        }
        let n_max = (self.steps - 1) as crate::Wave;
        let pb = ((b - self.b_min) * self.inv_b_scale).clamp(0.0, n_max);
        let pc = ((ctrl - self.ctrl_min) * self.inv_c_scale).clamp(0.0, n_max);
        let ib = (pb as usize).min(self.steps - 2);
        let ic = (pc as usize).min(self.steps - 2);
        let fb = pb - ib as crate::Wave;
        let fc = pc - ic as crate::Wave;
        let s = self.steps;
        let v00 = table[ib + ic * s];
        let v10 = table[ib + 1 + ic * s];
        let v01 = table[ib + (ic + 1) * s];
        let v11 = table[ib + 1 + (ic + 1) * s];
        let t0 = v00 + fb * (v10 - v00);
        let t1 = v01 + fb * (v11 - v01);
        t0 + fc * (t1 - t0)
    }
}

#[cfg(test)]
mod adaa_tests {
    use super::*;

    fn make_tanh_table() -> KTable {
        // 1D K-table: f(b) = tanh(b)
        let steps = 256;
        let b_min = -5.0;
        let b_max = 5.0;
        let entries: Vec<crate::Wave> = (0..steps)
            .map(|i| {
                let b = b_min + (b_max - b_min) * i as crate::Wave / (steps - 1) as crate::Wave;
                b.tanh()
            })
            .collect();
        let mut kt = KTable {
            dims: 1,
            b_min,
            b_max,
            ctrl_min: 0.0,
            ctrl_max: 1.0,
            steps,
            entries,
            inv_b_scale: 0.0,
            inv_c_scale: 0.0,
        };
        kt.precompute_scales();
        kt
    }

    #[test]
    fn adaa_antiderivative_correct_shape() {
        let kt = make_tanh_table();
        let ad = kt.compute_antiderivative();
        assert_eq!(ad.len(), kt.steps);
        // A(b) = ∫ tanh(x) dx = ln(cosh(x)). This is a U-shape (minimum at x=0).
        // The trapezoidal integral starts at b_min, so ad[0]=0.
        // In the right half (b > 0), the integral should increase.
        let mid = kt.steps / 2;
        assert!(ad[0].abs() < 1e-10, "ad[0] should be 0, got {}", ad[0]);
        assert!(
            ad[kt.steps - 1] > ad[mid],
            "Right half should increase: ad[last]={}, ad[mid]={}",
            ad[kt.steps - 1],
            ad[mid]
        );
        // ad should not be all zeros
        let max_ad = ad
            .iter()
            .map(|v| v.abs())
            .fold(0.0 as crate::Wave, crate::Wave::max);
        assert!(
            max_ad > 0.1,
            "Antiderivative should be nonzero, max={max_ad}"
        );
    }

    #[test]
    fn adaa_matches_direct_for_slow_signals() {
        let kt = make_tanh_table();
        let ad = kt.compute_antiderivative();
        // For slowly varying input (small delta b), ADAA should ≈ direct lookup
        let b_prev = 0.5;
        let b_new = 0.500001; // tiny step
        let adaa = kt.lookup_1d_adaa(b_new, b_prev, &ad);
        let direct = kt.lookup_1d(b_new);
        assert!(
            (adaa - direct).abs() < 0.01,
            "ADAA should match direct for slow signals: adaa={adaa:.6}, direct={direct:.6}"
        );
    }

    #[test]
    fn adaa_attenuates_fast_signals() {
        let kt = make_tanh_table();
        let ad = kt.compute_antiderivative();
        // For fast input (large delta b), ADAA should be LESS than the peak
        // of direct evaluation — it's averaging over the transition.
        let b_prev = -3.0;
        let b_new = 3.0; // big jump across the nonlinearity
        let adaa = kt.lookup_1d_adaa(b_new, b_prev, &ad);
        let direct_peak = kt.lookup_1d(b_new); // tanh(3) ≈ 0.995
        assert!(
            adaa.abs() < direct_peak.abs(),
            "ADAA should attenuate fast transitions: adaa={adaa:.6}, direct_peak={direct_peak:.6}"
        );
    }

    #[test]
    fn adaa_fallback_at_zero_delta() {
        let kt = make_tanh_table();
        let ad = kt.compute_antiderivative();
        // When b_prev == b_new, ADAA falls back to direct evaluation
        let b = 1.0;
        let adaa = kt.lookup_1d_adaa(b, b, &ad);
        let direct = kt.lookup_1d(b);
        assert!(
            (adaa - direct).abs() < 1e-10,
            "ADAA at zero delta should equal direct: adaa={adaa}, direct={direct}"
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// WDF clipping stage
// ═══════════════════════════════════════════════════════════════════════════

#[allow(dead_code)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum RootKind {
    DiodePair(DiodePairRoot),
    SingleDiode(DiodeRoot),
    /// Wright Omega–based explicit anti-parallel diode pair root.
    ExplicitDiodePair(ExplicitDiodePairRoot),
    /// Wright Omega–based explicit single diode root.
    ExplicitSingleDiode(ExplicitDiodeRoot),
    Zener(ZenerRoot),
    ZenerPair(ZenerPairRoot),
    Jfet(JfetRoot),
    /// JFET operating as a voltage-controlled variable resistor.
    /// Used for LFO-modulated JFETs in phaser circuits (e.g., Phase 90).
    /// Simple resistor reflection instead of Newton-Raphson solving.
    JfetVr(JfetVariableResistor),
    Triode(TriodeRoot),
    VariMu(VariMuTriodeRoot),
    Pentode(PentodeRoot),
    Mosfet(MosfetRoot),
    /// BJT as single-port WDF root (common-emitter topology).
    /// Vbe is set externally (like triode Vgk), C-E is the WDF port.
    /// Enables K-method tabulation (2D: b_tree × Vbe).
    Bjt(BjtRoot),
    /// Diff-pair macromodel for ladder filter stages.
    /// I = α·I_tail·tanh(V/(2·n·Vt)). Cutoff CV modulates I_tail.
    /// K-method: 2D (b_tree × I_tail).
    DiffPair(DiffPairRoot),
    Ota(OtaRoot),
    /// Op-amp gain stage (TL072, LM308, JRC4558, etc.).
    /// Topology (inverting vs non-inverting) is stored in the root itself.
    /// - Inverting: Vout = -(Rf/Ri) * Vin
    /// - Non-inverting: Vout = (1 + Rf/Ri) * Vin
    OpAmp(OpAmpRoot),
    /// Passthrough (transparent) root for passive-only circuits.
    /// Models an open-circuit termination: b = a (reflected = incident).
    /// Used when the circuit has reactive elements (caps/inductors) but no
    /// nonlinear elements, allowing the WDF tree to process the filtering.
    Passthrough,
    /// Short-circuit root for passive filters terminated to ground.
    /// Models a short-circuit: a = -b (total reflection, inverted).
    ///
    /// In physical terms, ground has zero impedance, so it reflects
    /// incident waves with inverted sign. This allows current to flow
    /// through series elements (L, R) to ground, enabling voltage divider
    /// behavior in series filters like RL lowpass.
    ///
    /// The output is taken at an internal junction (e.g., between L and R),
    /// not at the grounded root port.
    ShortCircuit,
    /// Voltage source driver for truly passive-only filters.
    ///
    /// The input signal is injected as the incident wave at the root port,
    /// and the tree (containing L, C, R without embedded VS) reflects.
    /// Output voltage = (a_root + b_tree) / 2.
    ///
    /// This is the proper WDF architecture for passive filters:
    /// - No voltage source embedded in tree
    /// - Signal injected at root as wave: a = 2 * V_in
    /// - Load resistor is adapted as part of tree
    ///
    /// Only used when circuit contains NO active elements (op-amps, transistors, etc.)
    /// since active element breaks require the fallback Passthrough approach.
    VoltageSourceDriver,
    /// Capacitor as WDF root for RC lowpass and similar filters.
    /// The capacitor reflects its stored state: b = state.
    /// Output voltage = (a + state) / 2 gives correct transfer function.
    CapacitorRoot {
        /// Capacitance in Farads
        capacitance: crate::Wave,
        /// Port resistance: 1 / (2 * fs * C)
        rp: crate::Wave,
        /// State (previous incident wave)
        state: crate::Wave,
    },
    /// Inductor as WDF root for RL highpass and similar filters.
    /// The inductor reflects negated state: b = -state.
    /// Output voltage = (a - state) / 2 gives correct transfer function.
    InductorRoot {
        /// Inductance in Henrys
        inductance: crate::Wave,
        /// Port resistance: 2 * fs * L
        rp: crate::Wave,
        /// State (previous incident wave)
        state: crate::Wave,
    },
    /// Resistive termination: load resistor at the root port.
    /// The resistor absorbs all incident energy: b = 0, so a_root = 0.
    /// Output voltage = (0 + b_tree) / 2 = b_tree / 2.
    /// Used for RC highpass and RL lowpass where R is the grounded load.
    ResistiveTermination,
    /// Passive R-type adaptor for non-series-parallel topologies.
    ///
    /// When `sp_reduce` fails (e.g., Twin-T notch), this MNA-derived
    /// R-type adaptor handles arbitrary passive topologies. Resistors are
    /// stamped into the MNA conductance matrix; reactive elements (caps,
    /// inductors) are WDF children with state. The VS is stamped directly
    /// into the MNA B/C/D matrices (zero internal impedance), giving an
    /// ideal voltage source. A high-impedance output probe port extracts
    /// voltage at the output node.
    ///
    /// Processing:
    /// ```text
    /// a[i] = Σ_j S[i][j] · b[j] + k[i] · V_in
    /// output = (a_out + b_out) / 2
    /// ```
    PassiveRType {
        /// Scattering matrix S (n_ports × n_ports, row-major).
        scattering: Vec<crate::Wave>,
        /// VS injection vector k (n_ports elements).
        vs_injection: Vec<crate::Wave>,
        /// Number of ports (reactive + output probe).
        n_ports: usize,
        /// Passive child nodes (capacitors, inductors, output probe, and pots).
        /// Pots are stored here for control binding but are NOT WDF ports —
        /// they live in the MNA G matrix as conductance entries.
        children: Vec<DynNode>,
        /// Runtime state for each child DynNode.
        ///
        /// Reactive WDF port children own capacitor/inductor state here. Pot
        /// children are MNA conductance controls and normally have empty state.
        #[cfg_attr(feature = "serde", serde(default))]
        child_runtime_states: Vec<RuntimeState>,
        /// Index into `children` for the output probe port.
        output_port: usize,
        /// Direct node-voltage extraction coefficients for output.
        ///
        /// When present, output is read from the original MNA output node:
        /// `Vout = sum(extraction_coeffs[i] * b[i]) + extraction_vs * Vin`.
        /// This avoids using a high-Z probe port as an approximation of a
        /// circuit node voltage.
        extraction_coeffs: Vec<crate::Wave>,
        extraction_vs: crate::Wave,
        extraction_output_pos: Option<usize>,
        extraction_output_neg: Option<usize>,
        /// Stored MNA system for pot recomputation (None if no pots).
        recompute_mna: Option<MnaSystem>,
        /// WDF port definitions (reactive ports only, not pots).
        recompute_ports: Option<Vec<WdfPort>>,
        /// Runtime variable-resistor bindings for MNA G-matrix updates.
        variable_resistors: Vec<MnaVariableResistorBinding>,
        /// Dirty flag: set when a pot changes, cleared after S re-derivation.
        needs_recompute: bool,
        /// Precomputed interpolation table for single-pot stages.
        /// When Some, pot changes use table lookup instead of MNA re-inversion.
        interp_table: Option<ScatteringInterpolationTable>,
    },
}

// Shared bias constants for NL device control voltage setting.
// Used by both RootKind (WdfStage) and NlDeviceKind (MultiNlStage).
// Triode grid bias is now per-instance (TriodeRoot::vgk_bias), set from
// circuit analysis. Default -2.0V (typical 12AX7) is set in TriodeRoot::new().
// Pentode grid bias is now per-instance (PentodeRoot::vg1k_bias).
// BJT base bias is set per-instance from circuit analysis (BjtRoot::set_bias).

/// Maximum total NR iterations per base sample across all sub-samples.
///
/// With X4 oversampling and NR_MAX_ITER=24, uncapped cost is 4×24 = 96 iterations
/// per base sample per stage. On transients, all sub-samples may need full NR,
/// causing CPU spikes. This budget caps the total across all sub-samples so that
/// later sub-samples reuse the previous b_nl when the budget is exhausted.
/// 48 is generous enough for normal signals (most sub-samples converge in 2–5 iters)
/// while protecting against extreme transient spikes.
pub const NR_ITERATION_BUDGET: usize = 48;

impl RootKind {
    /// Returns `true` for roots that clip the signal (diodes, zeners).
    pub fn is_clipping_stage(&self) -> bool {
        matches!(
            self,
            RootKind::DiodePair(_)
                | RootKind::SingleDiode(_)
                | RootKind::ExplicitDiodePair(_)
                | RootKind::ExplicitSingleDiode(_)
                | RootKind::Zener(_)
                | RootKind::ZenerPair(_)
        )
    }

    /// Short stable name for the root kind (diagnostic / stage-graph use).
    pub fn kind_name(&self) -> &'static str {
        match self {
            RootKind::DiodePair(_) => "DiodePair",
            RootKind::SingleDiode(_) => "SingleDiode",
            RootKind::ExplicitDiodePair(_) => "ExplicitDiodePair",
            RootKind::ExplicitSingleDiode(_) => "ExplicitSingleDiode",
            RootKind::Zener(_) => "Zener",
            RootKind::ZenerPair(_) => "ZenerPair",
            RootKind::Jfet(_) => "Jfet",
            RootKind::JfetVr(_) => "JfetVr",
            RootKind::Triode(_) => "Triode",
            RootKind::VariMu(_) => "VariMu",
            RootKind::Pentode(_) => "Pentode",
            RootKind::Mosfet(_) => "Mosfet",
            RootKind::Bjt(_) => "Bjt",
            RootKind::DiffPair(_) => "DiffPair",
            RootKind::Ota(_) => "Ota",
            RootKind::OpAmp(_) => "OpAmp",
            RootKind::Passthrough => "Passthrough",
            RootKind::ShortCircuit => "ShortCircuit",
            RootKind::VoltageSourceDriver => "VoltageSourceDriver",
            RootKind::CapacitorRoot { .. } => "CapacitorRoot",
            RootKind::InductorRoot { .. } => "InductorRoot",
            RootKind::ResistiveTermination => "ResistiveTermination",
            RootKind::PassiveRType { .. } => "PassiveRType",
        }
    }

    /// Whether this root requires a per-sample nonlinear (Newton-Raphson /
    /// Wright-Omega) solve. Passive/linear terminations return `false`.
    pub fn is_nonlinear(&self) -> bool {
        matches!(
            self,
            RootKind::DiodePair(_)
                | RootKind::SingleDiode(_)
                | RootKind::ExplicitDiodePair(_)
                | RootKind::ExplicitSingleDiode(_)
                | RootKind::Zener(_)
                | RootKind::ZenerPair(_)
                | RootKind::Jfet(_)
                | RootKind::Triode(_)
                | RootKind::VariMu(_)
                | RootKind::Pentode(_)
                | RootKind::Mosfet(_)
                | RootKind::Bjt(_)
                | RootKind::DiffPair(_)
                | RootKind::Ota(_)
                | RootKind::OpAmp(_)
        )
    }

    /// K-method eligibility: (is_eligible, port_dimensions).
    /// Mirrors Component::k_method_candidacy() for the runtime root type.
    pub fn k_method_candidacy(&self) -> (bool, usize) {
        match self {
            RootKind::DiodePair(_)
            | RootKind::SingleDiode(_)
            | RootKind::ExplicitDiodePair(_)
            | RootKind::ExplicitSingleDiode(_)
            | RootKind::Zener(_)
            | RootKind::ZenerPair(_) => (true, 1),
            RootKind::Jfet(_)
            | RootKind::Triode(_)
            | RootKind::Mosfet(_)
            | RootKind::Bjt(_)
            | RootKind::DiffPair(_) => (true, 2),
            RootKind::Pentode(_) => (true, 3),
            // Not eligible: linear, variable, or hysteretic roots
            _ => (false, 0),
        }
    }

    /// Reset NR warm-start state for clean cold-start solve.
    /// Used by K-table generation to ensure each entry is independent.
    pub fn reset_nr_state(&mut self) {
        // Process with b=0 to reset internal prev_v to a neutral state.
        // This is a no-op functionally (zero input → near-zero output)
        // but resets the warm-start cache.
        let _ = self.process(0.0, 1000.0);
    }

    /// Process the NL root: incident wave → reflected wave.
    /// Dispatches to the concrete root type's NR solver.
    pub fn process(&mut self, b_tree: crate::Wave, rp: crate::Wave) -> crate::Wave {
        match self {
            RootKind::DiodePair(dp) => dp.process(b_tree, rp),
            RootKind::SingleDiode(d) => d.process(b_tree, rp),
            RootKind::ExplicitDiodePair(dp) => dp.process(b_tree, rp),
            RootKind::ExplicitSingleDiode(d) => d.process(b_tree, rp),
            RootKind::Zener(z) => z.process(b_tree, rp),
            RootKind::ZenerPair(z) => z.process(b_tree, rp),
            RootKind::Jfet(j) => j.process(b_tree, rp),
            RootKind::JfetVr(j) => j.process_root(b_tree, rp),
            RootKind::Triode(t) => t.process(b_tree, rp),
            RootKind::VariMu(t) => t.process(b_tree, rp),
            RootKind::Pentode(p) => p.process(b_tree, rp),
            RootKind::Mosfet(m) => m.process(b_tree, rp),
            RootKind::Bjt(b) => b.process(b_tree, rp),
            RootKind::DiffPair(dp) => dp.process(b_tree, rp),
            RootKind::Ota(o) => o.process(b_tree, rp),
            RootKind::OpAmp(op) => op.process(b_tree, rp),
            RootKind::Passthrough => b_tree,
            RootKind::ShortCircuit => -b_tree,
            // Passive/linear roots: reflect without NL processing
            _ => -b_tree,
        }
    }

    /// Set the control voltage on the nonlinear root from the input signal.
    ///
    /// Maps the audio input to the device's control terminal:
    /// - BJTs: Vbe/Veb from input with bias offset
    /// - Triodes/VariMu: Vgk from input
    /// - Pentodes: Vg1k from input
    /// - Diodes/JFETs/other: no control voltage
    pub fn set_control_voltage(
        &mut self,
        input: crate::Wave,
        compensation: crate::Wave,
        _bias_offset: crate::Wave,
    ) {
        match self {
            RootKind::Triode(t) => {
                t.set_vgk(t.vgk_bias() + input * compensation);
            }
            RootKind::VariMu(t) => {
                t.set_vgk(t.vgk_bias() + input * compensation);
            }
            RootKind::Bjt(b) => {
                // BJT Vbe = DC bias (from circuit analysis) + AC input signal.
                // The bias is set once at compile time via set_bias().
                b.set_vbe(b.vbe_bias() + input * compensation);
            }
            RootKind::Jfet(j) => {
                j.set_vgs(j.vgs_bias() + input * compensation);
            }
            RootKind::Mosfet(m) => {
                m.set_vgs(m.vgs_bias() + input * compensation);
            }
            RootKind::DiffPair(dp) => {
                // Cutoff CV modulates tail current I_tail.
                // Input maps to I_tail range: bias * (1 + input * compensation)
                let i_tail = dp.i_tail_bias * (1.0 + input * compensation).max(0.01);
                dp.set_i_tail(i_tail);
            }
            RootKind::Pentode(p) => {
                p.set_vg1k(p.vg1k_bias() + input * compensation);
            }
            _ => {}
        }
    }
}

/// Feedback impedance IIR for JFET→opamp-neg all-pass stages (Phase 90 topology).
///
/// Models the inverting all-pass where JFET drain connects to opamp neg
/// with Rf||Cf feedback to output. Transfer function: V_out = -(Z_fb/Z_in) * V_in
/// where Z_fb(s) = Rf/(1 + s·Rf·Cf) via bilinear transform.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AllpassFeedback {
    /// Input resistance (R_ap) in the signal path before the JFET.
    pub r_ap: crate::Wave,
    /// IIR numerator coefficient: Rf / (1 + K) where K = 2·fs·Rf·Cf.
    pub b0: crate::Wave,
    /// IIR denominator coefficient: (K - 1) / (K + 1).
    pub a1: crate::Wave,
    /// Previous input current sample for IIR.
    pub x_prev: crate::Wave,
    /// Previous output voltage for IIR.
    pub y_prev: crate::Wave,
}

/// Non-inverting all-pass IIR for Phase 90-style JFET + opamp topology.
///
/// Transfer function: H(s) = (1 - s·Rds·C) / (1 + s·Rds·C)
/// Bilinear-transformed: y[n] = a1·(x[n] - y[n-1]) + x[n-1]
/// where a1 = (1 - K) / (1 + K), K = 2·fs·Rds·C.
///
/// The WDF tree still processes each sample to maintain the JFET's NR state,
/// but the output comes from this IIR rather than the tree's root port voltage.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AllpassDirect {
    /// Phase-shifting capacitor value (e.g., C_ap = 47nF).
    pub cap: crate::Wave,
    /// Sample rate for coefficient computation.
    pub sample_rate: crate::Wave,
    /// Previous input sample.
    pub x_prev: crate::Wave,
    /// Previous output sample.
    pub y_prev: crate::Wave,
}

/// Models a bridged-T opamp resonator as a 2nd-order IIR bandpass.
///
/// The analog transfer function from trigger injection to output is
/// a 2nd-order bandpass centered at f0 = 1/(2π·√(R1·R2·C1·C2)).
/// The Q factor is controlled by Rf/√(R1·R2) — higher Rf = longer ring.
///
/// Bilinear-transformed to a 2nd-order IIR:
///   y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] - a1·y[n-1] - a2·y[n-2]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ResonatorFeedback {
    /// IIR coefficients (normalized by a0).
    pub b0: crate::Wave,
    pub b1: crate::Wave,
    pub b2: crate::Wave,
    pub a1: crate::Wave,
    pub a2: crate::Wave,
    /// State: previous input samples.
    pub x1: crate::Wave,
    pub x2: crate::Wave,
    /// State: previous output samples.
    pub y1: crate::Wave,
    pub y2: crate::Wave,
}

impl ResonatorFeedback {
    /// Create a new bridged-T resonator IIR from circuit parameters.
    ///
    /// Uses the Audio EQ Cookbook bandpass formula with bilinear pre-warping.
    pub fn new(
        r1: crate::Wave,
        r2: crate::Wave,
        c1: crate::Wave,
        c2: crate::Wave,
        rf: crate::Wave,
        sample_rate: crate::Wave,
    ) -> Self {
        // Resonant angular frequency
        let omega_0 = 1.0 / crate::math::sqrt((r1 * r2 * c1 * c2) as crate::Wave) as crate::Wave;

        // Q factor for a bridged-T oscillator.
        // The T-network has a notch attenuation of ~1/3 for equal R,C.
        // For oscillation, Rf must exceed the critical value R_crit.
        // Q = Rf / (Rf - R_crit) where R_crit ≈ R1 + R2 + R1*C1/C2.
        // For equal R,C: R_crit = 3*R, so Q = Rf/(Rf - 3*R).
        let r_crit = r1 + r2 + r1 * c1 / c2;
        // Clamp to avoid division by zero / negative Q when Rf is too small
        let q = if rf > r_crit * 1.01 {
            rf / (rf - r_crit)
        } else {
            // At or below oscillation threshold — use high Q for marginal oscillation
            100.0
        };

        // Bilinear pre-warping: map analog frequency to digital
        let w0 =
            2.0 * crate::math::atan((omega_0 / (2.0 * sample_rate)) as crate::Wave) as crate::Wave;

        // Audio EQ Cookbook: BPF (constant 0 dB peak gain)
        let sin_w0 = crate::math::sin(w0 as crate::Wave) as crate::Wave;
        let cos_w0 = crate::math::cos(w0 as crate::Wave) as crate::Wave;
        let alpha = sin_w0 / (2.0 * q);

        let b0 = alpha;
        let b1 = 0.0;
        let b2 = -alpha;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cos_w0;
        let a2 = 1.0 - alpha;

        // Normalize by a0 and scale output by loop gain (Rf/R1)
        let gain = rf / r1;

        ResonatorFeedback {
            b0: gain * b0 / a0,
            b1: gain * b1 / a0,
            b2: gain * b2 / a0,
            a1: a1 / a0,
            a2: a2 / a0,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    /// Process one sample through the 2nd-order bandpass.
    #[inline]
    pub fn process(&mut self, input: crate::Wave) -> crate::Wave {
        let y = self.b0 * input + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = input;
        self.y2 = self.y1;
        self.y1 = flush_denormal(y);
        self.y1
    }
}

/// Custom 2-port WDF adaptor for ideal op-amp circuits.
///
/// Enforces V_neg = V+ at the internal junction of two subtrees (Zi and Zf)
/// using the closed-form scattering equations derived from KCL + op-amp constraint.
/// The gain emerges from the impedance ratio Zf/Zi — no precomputed scalar.
///
/// # Scattering equations (inverting)
///
/// ```text
/// i₁ = (V_in − b₁ − V+) / R₁
/// a₁ = 2·V_in − 2·V+ − b₁
/// a₂ = b₂ − 2·R₂·(V_in − b₁ − V+) / R₁
/// V_out = V+ + R₂·(V_in − b₁ − V+) / R₁ − b₂
/// ```
///
/// # Scattering equations (non-inverting)
///
/// ```text
/// i₁ = (V+ − b₁) / R₁
/// a₁ = 2·V+ − b₁
/// a₂ = b₂ − 2·R₂·(V+ − b₁) / R₁
/// V_out = V+ + R₂·(V+ − b₁) / R₁ − b₂
/// ```
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OpAmpWdfAdaptor {
    /// Zi subtree (input network for inverting, ground-leg for non-inverting).
    pub zi: DynNode,
    /// Runtime state for reactive one-ports owned by `zi`.
    #[cfg_attr(feature = "serde", serde(default))]
    pub zi_runtime_state: RuntimeState,
    /// Zf subtree (feedback network: caps, pots, resistors between neg and output).
    pub zf: DynNode,
    /// Runtime state for reactive one-ports owned by `zf`.
    #[cfg_attr(feature = "serde", serde(default))]
    pub zf_runtime_state: RuntimeState,
    /// True for inverting topology, false for non-inverting.
    pub is_inverting: bool,
    /// Op-amp model for GBW rolloff and slew rate limiting.
    pub opamp: OpAmpRoot,
    /// GBW rolloff filter state (single-pole LPF on output).
    pub gbw_state: crate::Wave,
    /// Previous output for slew rate limiting.
    pub prev_out: crate::Wave,
    /// Feedback pot ID (if any pot in Zf subtree responds to control changes).
    pub feedback_pot_id: Option<String>,
}

impl OpAmpWdfAdaptor {
    pub fn new(
        mut zi: DynNode,
        mut zf: DynNode,
        is_inverting: bool,
        opamp: OpAmpRoot,
        feedback_pot_id: Option<String>,
    ) -> Self {
        let zi_runtime_state = zi.bind_runtime_state();
        let zf_runtime_state = zf.bind_runtime_state();
        Self {
            zi,
            zi_runtime_state,
            zf,
            zf_runtime_state,
            is_inverting,
            opamp,
            gbw_state: 0.0,
            prev_out: 0.0,
            feedback_pot_id,
        }
    }

    /// Process one sample through the op-amp adaptor.
    ///
    /// `v_plus`: non-inverting input voltage (bias for inverting, signal for non-inv).
    /// `v_in`: far-end voltage of Zi (signal for inverting, 0 for non-inverting).
    #[inline]
    pub fn process(&mut self, v_plus: crate::Wave, v_in: crate::Wave) -> crate::Wave {
        // Up-sweep: get reflected waves from both subtrees
        let b1 = self.zi.reflected_with_state(&mut self.zi_runtime_state);
        let b2 = self.zf.reflected_with_state(&mut self.zf_runtime_state);
        let r1 = self.zi.port_resistance();
        let r2 = self.zf.port_resistance();

        // Scattering from V_neg = V+ constraint + KCL at neg
        let (a1, a2, v_out) = if self.is_inverting {
            // Inverting: Port 1 + at V_in, - at V_neg
            // V_neg = V_in - v1 = V_in - b1 - R1·i1
            let drive = v_in - b1 - v_plus;
            let a1 = 2.0 * v_in - 2.0 * v_plus - b1;
            let a2 = b2 - 2.0 * r2 * drive / r1;
            let v_out = v_plus + r2 * drive / r1 - b2;
            (a1, a2, v_out)
        } else {
            // Non-inverting: Port 1 + at V_neg, - at ground
            // V_neg = b1 + R1·i1 = V+
            let drive = v_plus - b1;
            let a1 = 2.0 * v_plus - b1;
            let a2 = b2 - 2.0 * r2 * drive / r1;
            let v_out = v_plus + r2 * drive / r1 - b2;
            (a1, a2, v_out)
        };

        // Down-sweep: send incident waves back into subtrees (state update)
        self.zi
            .set_incident_with_state(a1, &mut self.zi_runtime_state);
        self.zf
            .set_incident_with_state(a2, &mut self.zf_runtime_state);

        // Apply op-amp non-idealities: GBW rolloff → hard rail clip → slew rate.
        // GBW and the dV/dt slew use the shared NonIdealFxState primitives (no
        // copy of that math); the supply-rail clip stays a hard clamp here (not
        // the tanh soft-clip of the post-FX path), matching the original.
        let v_max = self.opamp.v_max();
        let mut fx = NonIdealFxState {
            gbw_coeff: self.opamp.gbw_coeff(),
            gbw_state: self.gbw_state,
            max_dv: self.opamp.model.slew_rate * 1e6 / self.opamp.sample_rate(),
            prev_out: self.prev_out,
            ..NonIdealFxState::default()
        };

        let mut out = fx.gbw_step(v_out);
        out = out.clamp(-v_max, v_max);
        out = fx.slew_step(out);

        self.gbw_state = fx.gbw_state;
        self.prev_out = fx.prev_out;

        flush_denormal(out)
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.gbw_state = 0.0;
        self.prev_out = 0.0;
        self.zi.reset_with_state(&mut self.zi_runtime_state);
        self.zf.reset_with_state(&mut self.zf_runtime_state);
        self.opamp.reset();
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WdfStage {
    pub tree: DynNode,
    /// Shared physical one-port state owned by this WDF stage's main tree.
    ///
    /// The tree keeps topology and state-slot bindings; physical state and WDF
    /// wave sidecars live here so WDF uses the same owner pattern as BKM
    /// coupling passives.
    #[cfg_attr(feature = "serde", serde(default))]
    pub runtime_state: RuntimeState,
    pub root: RootKind,
    /// Compensates for passive attenuation in the tree topology.
    /// Computed automatically from the tree's impedance structure.
    pub compensation: crate::Wave,
    /// Oversampler for antialiasing at nonlinear stages.
    pub oversampler: Oversampler,
    /// Base diode model (before thermal modulation). Stored so thermal
    /// drift can be applied as a multiplier without accumulation.
    pub base_diode_model: Option<DiodeModel>,
    /// Base BJT model (before thermal modulation). Stored so thermal
    /// drift can be applied as a multiplier without accumulation.
    /// Only set when `options.thermal` is true at compile time.
    pub base_bjt_model: Option<crate::elements::nonlinear::GummelPoonModel>,
    /// Op-amp buffer paired with this WDF stage (for all-pass circuits).
    ///
    /// When a unity-gain op-amp feedback loop (neg=out) is detected at this
    /// stage's output, the op-amp is processed inline after the WDF cycle.
    /// The op-amp receives the stage INPUT as its Vp (non-inverting input),
    /// modeling the all-pass behavior where the op-amp buffers the signal
    /// at unity gain while the R/C/JFET network shifts phase.
    pub paired_opamp: Option<OpAmpRoot>,
    /// Inverting all-pass feedback (Phase 90 topology).
    /// When present, bypasses the WDF output and computes V_out = -(Z_fb/Z_in) * V_in
    /// where Z_in includes the JFET variable resistance.
    pub allpass_feedback: Option<AllpassFeedback>,
    /// Non-inverting all-pass IIR (Phase 90 gain-of-2 style).
    /// Computes H(s) = (1 - s·Rds·C) / (1 + s·Rds·C) directly,
    /// using the JFET's approximate Rds from its operating point.
    pub allpass_direct: Option<AllpassDirect>,
    /// DC-blocking highpass filter for triode stages.
    /// Models the output coupling capacitor's DC blocking behavior.
    /// Format: (a1, b0, y_prev, x_prev) for IIR highpass.
    pub dc_block: Option<(crate::Wave, crate::Wave, crate::Wave, crate::Wave)>,
    /// Inter-stage grid DC blocker: removes plate DC from the previous stage
    /// before setting the tube's grid voltage. Without this, multi-stage chains
    /// feed plate voltage (~80-200V) directly into Vgk, saturating the tube.
    /// The WDF tree's coupling cap blocks DC for wave propagation but the grid
    /// voltage is set externally via set_control_voltage(). This HPF mimics
    /// the coupling cap's DC-blocking effect on the grid bias.
    /// Format: (x_prev, y_prev). α = 0.9995, fc ≈ 3.5 Hz at 48 kHz.
    pub grid_dc_blocker: Option<(crate::Wave, crate::Wave)>,
    /// Source follower mode for JFETs.
    /// When true, Vgs is computed as Vgate (input) - Vsource (output).
    /// This enables proper source follower behavior where the source follows the gate.
    pub is_source_follower: bool,
    /// Previous source voltage for source follower Vgs calculation.
    /// Vgs[n] = input[n] - Vsource[n-1]
    pub prev_source_voltage: crate::Wave,
    /// Absolute DC voltage added to the WDF root's incremental output.
    ///
    /// Most WDF roots solve absolute port voltage, so this remains zero. FET
    /// amplifier roots solve AC deviation around a DC Q-point; the following
    /// coupling network still needs the absolute device node voltage so its
    /// uncharged-cap startup transient matches the SPICE UIC harness.
    pub output_bias: crate::Wave,
    /// Source-leg component used to estimate FET source AC voltage.
    ///
    /// FET gate drive is externally controlled, but physical Vgs is
    /// Vgate - Vsource. When this is set, `prev_source_voltage` is updated from
    /// the probed source-leg leaf after the WDF down-sweep and subtracted from
    /// the next sample's gate drive.
    pub fet_source_probe: Option<String>,
    /// BFS distance from input of the injection node (for topological ordering).
    pub signal_flow_distance: usize,
    /// Component names in this stage (e.g. "R_in,Cin"). Debug builds only.
    #[cfg(debug_assertions)]
    pub debug_label: String,
    /// When true, this stage is not on the audio signal path (e.g. bias
    /// network). It still processes and meters, but its output does NOT
    /// overwrite the serial audio chain signal.
    pub bypass_serial: bool,
    /// Inter-stage voltage gain from a transformer boundary.
    /// When the stage's injection node is on a transformer secondary,
    /// this is 1/turns_ratio (e.g., 17.0 for a 1:17 step-up).
    pub transformer_gain: crate::Wave,
    /// Circuit graph node ID where this stage's voltage source injects signal.
    /// Used for node-based routing in parallel-path topologies.
    pub injection_node_id: usize,
    /// Circuit graph node ID where this stage's output is extracted.
    /// Typically the plate (triode), collector (BJT), or drain (JFET) node.
    /// Used for node-based routing in parallel-path topologies.
    pub output_node_id: usize,
    /// When true, this stage is a per-trigger voice stage that reads
    /// exclusively from `node_signals` (trigger impulses) rather than
    /// the serial chain signal. Unfired voices receive 0.0 input.
    pub is_trigger_voice: bool,
    /// Activation gate for trigger voice stages. When false, the stage
    /// skips processing entirely (outputs 0.0). Set to true on the first
    /// trigger impulse. Prevents unfired voices from contributing VCC
    /// bias to the output sum.
    pub voice_active: bool,
    /// When true, this stage is a feedforward (parallel) path that reads
    /// from `node_signals` at `injection_node_id` and additively blends
    /// its output into the serial chain signal.
    pub is_feedforward: bool,
    /// Component ID for the root device (used for NaN diagnostics and debug output).
    pub root_comp_id: String,
    /// When set, identifies a pot in the WDF tree whose resistance drives
    /// OpAmpRoot gain recalculation. After pot update + recompute, the stage
    /// reads the pot's resistance and calls `OpAmpRoot::set_feedback_pot_r()`.
    pub feedback_pot_id: Option<String>,
    /// Fixed resistance in series with the feedback pot (e.g., R_clip).
    /// Used to compute effective Rf = pot_r + series_r when pot changes.
    pub feedback_series_r: crate::Wave,
    /// Input resistance (Ri) for gain computation. Read from pendant tree
    /// port resistance or input-touching resistor at compile time.
    pub feedback_ri: crate::Wave,
    /// Pot in the op-amp ground/input leg whose resistance controls Ri.
    ///
    /// Some compiler splits consume this pot into scalar gain setup instead of
    /// keeping it as a WDF leaf in the same stage. These fields preserve the
    /// runtime mapping so the control can still update op-amp gain.
    pub feedback_ri_pot_id: Option<String>,
    pub feedback_ri_fixed_r: crate::Wave,
    pub feedback_ri_pot_max_r: crate::Wave,
    pub feedback_ri_pot_taper: crate::pot_taper::PotTaper,
    /// When set, extract output voltage at this component (pot leaf) in the
    /// WDF tree instead of at the root junction. After the down-sweep,
    /// V_out = a_leaf / 2 (for a resistive leaf where b=0).
    /// This models voltage extraction at the circuit's output node when
    /// a pot sits between the NL junction and the output.
    pub output_probe: Option<String>,
    /// Series-diode rectifier load-divider ratio `RL / (R1 + RL)`.
    ///
    /// A diode WDF root normally reports the diode *junction* voltage. That is
    /// correct for shunt clippers and op-amp feedback clippers, but a *series*
    /// rectifier `in -> R1 -> D1 -> RL -> gnd` taps the load voltage at the
    /// D1/RL junction. By KVL around the loop the load voltage is
    /// `V_RL = (Vin - V_diode) * RL/(R1+RL)`, which goes to ~0 when the diode
    /// blocks (true half-wave rectification) instead of passing the signal.
    /// When `Some(ratio)`, the diode output path computes the load voltage from
    /// this ratio. `None` keeps the junction-voltage behaviour.
    pub series_rectifier_divider: Option<crate::Wave>,
    /// Op-amp gain stage paired with a DiodePair/SingleDiode root.
    ///
    /// When an inverting op-amp has diodes in its feedback path (e.g., Tube Screamer,
    /// Klon Centaur), the op-amp gain + GBW + slew limiting are applied to the VS
    /// voltage before the diode NR solver clips. This replaces the standalone OpAmp
    /// stage + soft-clip tanh approximation with proper gain-into-diode-clipping.
    ///
    /// NOT the same as `paired_opamp` (which is for Bridged-T all-pass circuits).
    pub feedback_opamp: Option<OpAmpRoot>,
    /// K-method lookup table: precomputed NL root response.
    /// When present, process() uses table lookup instead of NR iteration.
    pub k_table: Option<KTable>,
    /// VCC injection coefficient (per-unit, wave domain).
    /// Multiply by supply voltage to get the DC bias added to the reflected wave.
    /// Computed from a small resistive MNA at build time. Zero when no VCC edge.
    pub vcc_injection_coeff: crate::Wave,
    /// Gradual DC ramp counter (0..256) to prevent NR solver divergence on startup.
    pub vcc_dc_ramp: u32,
    /// Component ID of the coupling capacitor connected to the injection node.
    /// When set for triode/pentode stages, the input signal flows through the WDF
    /// tree (through the coupling cap) so DC is naturally blocked. The cap's WDF
    /// state is used to extract Vgk instead of a software HPF.
    pub coupling_cap_id: Option<String>,
    /// Bridged-T resonator IIR for opamps with series R1→R2 path and
    /// C1/C2 shunt caps to ground. When present, the stage output is
    /// computed from this 2nd-order bandpass IIR instead of the WDF tree.
    pub resonator_feedback: Option<ResonatorFeedback>,
    /// Negate the voltage source value for tree topologies where the Series
    /// adaptor sign convention produces a negative reflected wave. Detected
    /// during construction: if `tree.reflected()` with positive VS gives a
    /// negative b_tree, the VS must be negated so the NR solver can find
    /// a valid operating point (plate/collector voltage must be positive).
    pub negate_vs: bool,
    /// Input-path photocouplers that modulate the input impedance (Ri) of
    /// an inverting opamp. When the envelope follower drives these, the
    /// opamp gain is updated: gain = Rf / (fixed_r + photocoupler_r).
    /// Used by Mu-Tron style envelope-controlled integrators.
    pub input_photocouplers: Vec<InputPhotocoupler>,
    /// Feedback (Zf) subtree for the 3-port linear opamp adaptor (NonInverting).
    /// Used with `zg_child` + `opamp_adaptor` when Zf/Zg split cleanly (e.g. RAT).
    pub zf_child: Option<DynNode>,
    #[cfg_attr(feature = "serde", serde(default))]
    pub zf_child_runtime_state: RuntimeState,
    /// Ground-leg (Zg) subtree for the 3-port linear opamp adaptor (NonInverting).
    pub zg_child: Option<DynNode>,
    #[cfg_attr(feature = "serde", serde(default))]
    pub zg_child_runtime_state: RuntimeState,
    /// R-type adaptor scattering matrix. Used by BOTH the 3-port path (zf/zg)
    /// and the MNA path (opamp_children). When Some, process() uses the adaptor.
    pub opamp_adaptor: Option<crate::tree::RTypeAdaptor>,
    /// MNA-based child ports for opamps with reactive feedback (Inverting).
    /// Each child is a DynNode leaf (cap, inductor, or pot). Fixed resistors
    /// are stamped into the MNA. Used when build_feedback_tree + scalar gain
    /// can't model frequency-dependent feedback (e.g. Goldenrod Treble).
    pub opamp_children: Vec<DynNode>,
    /// Stored MNA + port pairs for scattering recompute on pot change.
    pub opamp_recompute: Option<OpAmpRecomputeData>,
    /// Index of the VoltageSource child in `opamp_children` that models R_in.
    /// When `Some(i)`, the stage drives `opamp_children[i]` with the input signal
    /// each sample before scatter_up. This is the R_in port of the MNA adaptor
    /// for Inverting opamps — the scattering matrix encodes the full Rf/Ri gain
    /// so the OpAmpRoot gain is set to 1.0 and gain comes from impedance ratios.
    pub opamp_input_child_idx: Option<usize>,
    #[cfg_attr(feature = "serde", serde(default))]
    pub opamp_child_runtime_states: Vec<RuntimeState>,
    /// Custom WDF adaptor for op-amp circuits with reactive feedback.
    /// When `Some`, the stage uses the V_neg = V+ constraint-based scattering
    /// instead of the MNA/RType adaptor or precomputed scalar gain.
    /// The gain emerges from Zf/Zi impedance ratios in the wave propagation.
    pub opamp_wdf_adaptor: Option<OpAmpWdfAdaptor>,

    // ── Cached VS pointers (resolved once, used every sample) ────────────
    //
    // These raw pointers point directly into the WDF tree's heap-allocated
    // VoltageSource leaves. They eliminate per-sample tree walking for
    // set_voltage() and set_voltage_by_port(). Valid as long as the tree
    // is not dropped or structurally modified (which never happens after
    // construction).
    /// Pointer to the main (non-port) voltage source's `voltage` field.
    /// Resolved once via `cache_vs_pointers()`. Used by process() instead
    /// of `tree.set_voltage()` to avoid pattern-match overhead.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub main_vs_ptr: Option<VsPtr>,

    /// Pointers to named port voltage sources' `voltage` fields.
    /// Each entry is (port_name, pointer). Resolved once via
    /// `cache_vs_pointers()`. Used instead of `tree.set_voltage_by_port()`
    /// to eliminate per-sample tree walking + string comparison.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub port_vs_ptrs: Vec<(alloc::string::String, VsPtr)>,

    /// Boundary bindings expose multiple physical terminals from one WDF
    /// block to a surrounding adaptor/coupling solve. Ordinary WDF stages
    /// leave this empty and continue to behave as one-input/one-output stages.
    #[cfg_attr(feature = "serde", serde(default))]
    pub boundary_bindings: Vec<WdfBoundaryBinding>,

    /// Last-APPLIED controlled resistance per component id, for threshold
    /// gating of photocoupler / `jfet_vr` scattering re-derivations.
    ///
    /// Physics: a CdS photocell's resistance trajectory is band-limited by
    /// its own carrier dynamics (ms-to-s time constants — e.g. T4B attack
    /// ~10 ms, release 0.5–5 s), and a sidechain-driven JFET Vgs is likewise
    /// smoothed by the detector's RC network. Control content therefore
    /// lives below a few hundred Hz, so re-deriving the MNA scattering
    /// matrix (or WDF adaptor gammas) at audio rate is wasted work: between
    /// recomputes the applied resistance lags the true value by less than
    /// the gate epsilon. The element's internal state still updates every
    /// sample, so attack/release timing physics are unaffected — only the
    /// expensive re-derivation is skipped. Comparing against the
    /// last-APPLIED value (not the last-seen one) lets sub-epsilon drift
    /// accumulate and eventually trigger, bounding steady-state error by
    /// the epsilon.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub ctrl_r_last_applied: Vec<CtrlRecomputeGate>,

    /// Parallel-branch convergence summation (F13b).
    ///
    /// When `Some`, this stage is NOT processed as a normal WDF/serial stage.
    /// Instead it sums several upstream op-amp-output branches read from
    /// `node_signals`: `V_out = Σ_i g_i · node_signals[source_i]`. The transfer
    /// gains `g_i` are solved by DC superposition at compile time and re-solved
    /// only when a blend pot moves — the per-sample cost is `N` multiply-adds.
    #[cfg_attr(feature = "serde", serde(default))]
    pub convergence: Option<crate::convergence::ConvergenceSum>,
}

/// Per-component gating state for controlled-resistor recomputes.
/// See [`WdfStage::ctrl_resistance_drifted`].
pub struct CtrlRecomputeGate {
    pub comp_id: String,
    /// Resistance encoded in the currently-applied scattering matrix /
    /// adaptor gammas (the last value that actually triggered a recompute).
    pub last_applied_r: crate::Wave,
}

/// Relative resistance change below which a controlled-resistor
/// (photocoupler LDR / JFET VR) scattering re-derivation is skipped.
///
/// 1e-3 relative resistance corresponds to well under 0.01 dB of gain
/// error in a divider, far below measurement tolerances, while skipping
/// the vast majority of per-sample recomputes once the envelope settles.
///
/// MEASURED (2026-06-13, opto_leveler/fet_leveler sweeps): eps = 1e-3
/// shifts the opto GR curve by at most 0.006 dB and release timing by
/// 0.01% — far inside the 0.1 dB / 5% acceptance band. Do NOT add a
/// control-rate decimation on top of this check: a /16 decimation was
/// measured to buy only ~14% throughput while aliasing the loud-input
/// envelope ripple into a 0.154 dB GR shift (effective ratio 15.3 -> 17.9)
/// — the check must stay at audio rate so every >eps resistance move is
/// applied sample-accurately.
pub const CTRL_R_RECOMPUTE_EPS: crate::Wave = 1e-3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum WdfBoundaryDirection {
    Input,
    Output,
    Control,
}

#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WdfBoundaryBinding {
    pub label: alloc::string::String,
    pub node_id: usize,
    pub direction: WdfBoundaryDirection,
}

impl WdfStage {
    /// Create a new WdfStage with the given tree, root, and oversampling.
    /// All optional fields default to None/zero/false.
    ///
    /// Use struct update syntax to override specific fields:
    /// ```ignore
    /// WdfStage { feedback_pot_id: Some("Gain".into()), ..WdfStage::new(tree, root, os) }
    /// ```
    pub fn new(
        tree: DynNode,
        root: RootKind,
        oversampler: crate::oversampling::Oversampler,
    ) -> Self {
        let mut tree = tree;
        let runtime_state = tree.bind_runtime_state();
        tree.compute_dynamic_flags();
        Self {
            tree,
            runtime_state,
            root,
            compensation: 1.0,
            oversampler,
            base_diode_model: None,
            base_bjt_model: None,
            paired_opamp: None,
            allpass_feedback: None,
            allpass_direct: None,
            dc_block: None,
            grid_dc_blocker: None,
            is_source_follower: false,
            prev_source_voltage: 0.0,
            output_bias: 0.0,
            fet_source_probe: None,
            signal_flow_distance: 0,
            #[cfg(debug_assertions)]
            debug_label: String::new(),
            bypass_serial: false,
            transformer_gain: 1.0,
            injection_node_id: usize::MAX,
            output_node_id: usize::MAX,
            is_trigger_voice: false,
            voice_active: false,
            is_feedforward: false,
            root_comp_id: String::new(),
            feedback_pot_id: None,
            feedback_series_r: 0.0,
            feedback_ri: crate::Wave::INFINITY,
            feedback_ri_pot_id: None,
            feedback_ri_fixed_r: 0.0,
            feedback_ri_pot_max_r: 0.0,
            feedback_ri_pot_taper: crate::pot_taper::PotTaper::B,
            output_probe: None,
            series_rectifier_divider: None,
            feedback_opamp: None,
            k_table: None,
            vcc_injection_coeff: 0.0,
            vcc_dc_ramp: 0,
            coupling_cap_id: None,
            resonator_feedback: None,
            negate_vs: false,
            input_photocouplers: Vec::new(),
            zf_child: None,
            zf_child_runtime_state: RuntimeState::new(),
            zg_child: None,
            zg_child_runtime_state: RuntimeState::new(),
            opamp_adaptor: None,
            opamp_children: Vec::new(),
            opamp_recompute: None,
            opamp_input_child_idx: None,
            opamp_child_runtime_states: Vec::new(),
            opamp_wdf_adaptor: None,
            main_vs_ptr: None,
            port_vs_ptrs: Vec::new(),
            boundary_bindings: Vec::new(),
            ctrl_r_last_applied: Vec::new(),
            convergence: None,
        }
    }

    /// Resolve and cache raw pointers to all VS leaves in the tree.
    ///
    /// Call once after tree construction (and after any `wrap_leaf_with_vs`).
    /// The cached pointers eliminate per-sample tree walking in `process()`.
    ///
    /// `port_names`: names of ports assigned to this stage (from NamedPortBinding).
    pub fn cache_vs_pointers(&mut self, port_names: &[&str]) {
        // Resolve main (non-port) VS
        self.main_vs_ptr = self.tree.resolve_main_vs_ptr().map(VsPtr::new);

        // Resolve each named port VS
        self.port_vs_ptrs.clear();
        for &name in port_names {
            if let Some(ptr) = self.tree.resolve_port_vs_ptr(name) {
                self.port_vs_ptrs
                    .push((alloc::string::String::from(name), VsPtr::new(ptr)));
            }
        }
    }

    /// Set the main voltage source value via cached pointer.
    /// Falls back to tree walk if pointer not cached.
    #[inline(always)]
    pub fn set_vs_voltage(&mut self, v: crate::Wave) {
        if let Some(ptr) = self.main_vs_ptr {
            // SAFETY: pointer was resolved from our own tree and tree
            // structure is immutable after construction.
            unsafe {
                ptr.set(v);
            }
        } else {
            self.tree.set_voltage(v);
        }
    }

    /// Set a named port's voltage source via cached pointer.
    /// Falls back to tree walk if pointer not cached.
    #[inline(always)]
    pub fn set_port_voltage(&mut self, port_name: &str, v: crate::Wave) {
        for (name, ptr) in &self.port_vs_ptrs {
            if name == port_name {
                // SAFETY: same as set_vs_voltage
                unsafe {
                    ptr.set(v);
                }
                return;
            }
        }
        // Fallback: tree walk (shouldn't happen if cache_vs_pointers was called)
        self.tree.set_voltage_by_port(port_name, v);
    }
}

/// Stored data for recomputing opamp adaptor scattering when pots change.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OpAmpRecomputeData {
    pub mna: crate::tree::MnaSystem,
    pub port_pairs: Vec<WdfPortTerminals>,
    pub port_resistances: Vec<crate::Wave>,
}

/// A photocoupler in the input path of an inverting opamp stage.
/// Stores the element itself (for asymmetric time constant modeling)
/// plus the fixed series resistance and DC feedback resistance for gain updates.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct InputPhotocoupler {
    pub comp_id: String,
    pub element: Photocoupler,
    /// Fixed resistance in series with this photocoupler (e.g., R_min).
    pub fixed_series_r: crate::Wave,
    /// DC feedback resistance (Rf) for gain recalculation.
    pub dc_rf: crate::Wave,
}

impl WdfStage {
    fn ensure_passive_rtype_child_runtime_states(&mut self) {
        let RootKind::PassiveRType {
            children,
            child_runtime_states,
            ..
        } = &mut self.root
        else {
            return;
        };

        if child_runtime_states.len() == children.len()
            && child_runtime_states
                .iter()
                .all(|state| state.states.len() == state.wave_cache.len())
        {
            return;
        }

        child_runtime_states.clear();
        child_runtime_states.reserve(children.len());
        for child in children {
            child_runtime_states.push(child.bind_runtime_state());
        }
    }

    fn ensure_opamp_adaptor_runtime_states(&mut self) {
        if self.opamp_child_runtime_states.len() != self.opamp_children.len() {
            self.opamp_child_runtime_states.clear();
            self.opamp_child_runtime_states
                .reserve(self.opamp_children.len());
            for child in &mut self.opamp_children {
                self.opamp_child_runtime_states
                    .push(child.bind_runtime_state());
            }
        }

        if let Some(ref mut zf) = self.zf_child {
            if self.zf_child_runtime_state.is_empty()
                || self.zf_child_runtime_state.states.len()
                    != self.zf_child_runtime_state.wave_cache.len()
            {
                self.zf_child_runtime_state = zf.bind_runtime_state();
            }
        }

        if let Some(ref mut zg) = self.zg_child {
            if self.zg_child_runtime_state.is_empty()
                || self.zg_child_runtime_state.states.len()
                    != self.zg_child_runtime_state.wave_cache.len()
            {
                self.zg_child_runtime_state = zg.bind_runtime_state();
            }
        }
    }

    /// Check if any DynNode tree in this stage contains a pot with the given ID.
    pub fn has_pot(&self, pot_id: &str) -> bool {
        use crate::helpers::has_pot;
        if has_pot(&self.tree, pot_id) {
            return true;
        }
        if let Some(ref zf) = self.zf_child {
            if has_pot(zf, pot_id) {
                return true;
            }
        }
        if let Some(ref zg) = self.zg_child {
            if has_pot(zg, pot_id) {
                return true;
            }
        }
        for child in &self.opamp_children {
            if has_pot(child, pot_id) {
                return true;
            }
        }
        if let RootKind::PassiveRType { children, .. } = &self.root {
            if children.iter().any(|c| has_pot(c, pot_id)) {
                return true;
            }
        }
        // WDF constraint adaptor subtrees
        if let Some(ref adaptor) = self.opamp_wdf_adaptor {
            if has_pot(&adaptor.zi, pot_id) || has_pot(&adaptor.zf, pot_id) {
                return true;
            }
        }
        false
    }

    /// Process one sample through the WDF tree with oversampling.
    ///
    /// The oversampler wraps the entire WDF scatter-up → root solve →
    /// scatter-down cycle, ensuring that harmonics generated by the
    /// nonlinear root are properly bandlimited before decimation.
    ///
    /// When a paired op-amp is present (all-pass circuits), the WDF tree
    /// processes normally to update capacitor states (encoding phase shift),
    /// but the output is taken from the op-amp VCVS which buffers the
    /// stage input signal.  This models the real circuit behavior where
    /// the op-amp maintains unity gain while the R/C/JFET network shifts
    /// the signal's phase.
    #[inline]
    pub fn process(&mut self, input: crate::Wave) -> crate::Wave {
        // Apply inter-stage transformer voltage gain (1.0 when no transformer).
        let input = input * self.transformer_gain;
        self.ensure_passive_rtype_child_runtime_states();

        // ── OpAmp WDF adaptor path (V_neg = V+ constraint scattering) ──
        // Uses closed-form scattering from the op-amp constraint. Gain emerges
        // from Zf/Zi impedance ratios — no precomputed scalar, reactive feedback
        // handled naturally through cap/inductor state updates in the subtrees.
        if let Some(ref mut adaptor) = self.opamp_wdf_adaptor {
            let compensation = self.compensation;
            let (v_plus, v_in) = if adaptor.is_inverting {
                // Inverting: V+ = 0 (or bias), V_in = stage input
                (0.0, input * compensation)
            } else {
                // Non-inverting: V+ = stage input, V_in = 0 (ground through Rg)
                (input * compensation, 0.0)
            };

            let output = adaptor.process(v_plus, v_in);

            // DC block
            if let Some((a1, b0, ref mut y_prev, ref mut x_prev)) = self.dc_block {
                let y = b0 * (output - *x_prev) + a1 * *y_prev;
                *x_prev = output;
                *y_prev = flush_denormal(y);
                return *y_prev;
            }

            return flush_denormal(output);
        }

        // ── Opamp adaptor path (3-port subtrees OR MNA children) ──
        // Both paths use RTypeAdaptor scattering. The 3-port path has
        // graph_reduce subtrees (zf/zg); the MNA path has individual
        // component leaves (caps, pots) with resistors in the MNA.
        if self.opamp_adaptor.is_some() {
            self.ensure_opamp_adaptor_runtime_states();
            let adaptor = self.opamp_adaptor.as_mut().unwrap();

            // Collect reflected waves from children (3-port or MNA)
            let (b_children, use_mna) = if !self.opamp_children.is_empty() {
                // Drive the input VoltageSource child with the stage input signal
                // before collecting reflected waves. This is the R_in port for the
                // Inverting MNA adaptor — the VS injects signal into the neg node
                // via the scattering matrix, which encodes the full Rf/Ri topology.
                if let Some(idx) = self.opamp_input_child_idx {
                    self.opamp_children[idx].set_voltage(input * self.compensation);
                }
                let b: Vec<crate::Wave> = self
                    .opamp_children
                    .iter_mut()
                    .zip(self.opamp_child_runtime_states.iter_mut())
                    .map(|(child, state)| child.reflected_with_state(state))
                    .collect();
                (b, true)
            } else if let (Some(zf), Some(zg)) = (self.zf_child.as_mut(), self.zg_child.as_mut()) {
                (
                    vec![
                        zf.reflected_with_state(&mut self.zf_child_runtime_state),
                        zg.reflected_with_state(&mut self.zg_child_runtime_state),
                    ],
                    false,
                )
            } else {
                unreachable!("opamp adaptor requires children");
            };

            // Scatter to adapted port
            let b_adapted = adaptor.scatter_up(&b_children);

            // OpAmp root processes the adapted wave
            let RootKind::OpAmp(ref mut op) = &mut self.root else {
                unreachable!("opamp adaptor requires OpAmp root");
            };
            op.set_vp(input * self.compensation);
            let rp = adaptor.port_resistance;
            let a_adapted = op.process(b_adapted, rp);

            // Down-sweep: distribute to children
            let a_children = adaptor.scatter_down(a_adapted);
            if use_mna {
                for (i, (child, state)) in self
                    .opamp_children
                    .iter_mut()
                    .zip(self.opamp_child_runtime_states.iter_mut())
                    .enumerate()
                {
                    child.set_incident_with_state(a_children[i], state);
                }
            } else {
                self.zf_child
                    .as_mut()
                    .unwrap()
                    .set_incident_with_state(a_children[0], &mut self.zf_child_runtime_state);
                self.zg_child
                    .as_mut()
                    .unwrap()
                    .set_incident_with_state(a_children[1], &mut self.zg_child_runtime_state);
            }

            // Output = voltage at adapted port (out→gnd = V_out)
            let output = (a_adapted + b_adapted) / 2.0;

            // DC block (IIR highpass: y[n] = b0*(x[n]-x[n-1]) + a1*y[n-1])
            if let Some((a1, b0, ref mut y_prev, ref mut x_prev)) = self.dc_block {
                let y = b0 * (output - *x_prev) + a1 * *y_prev;
                *x_prev = output;
                *y_prev = flush_denormal(y);
                return *y_prev;
            }

            return flush_denormal(output);
        }

        // Borrow fields individually to satisfy the borrow checker
        let tree = &mut self.tree;
        let runtime_state = &mut self.runtime_state;
        let root = &mut self.root;
        let k_table = &self.k_table;
        let compensation = self.compensation;
        let output_probe = &self.output_probe;
        let source_probe = &self.fet_source_probe;
        let series_rectifier_divider = self.series_rectifier_divider;
        let feedback_opamp = &mut self.feedback_opamp;
        let vcc_injection_coeff = self.vcc_injection_coeff;
        let vcc_dc_ramp = &mut self.vcc_dc_ramp;
        let coupling_cap_id = &self.coupling_cap_id;
        let prev_source_voltage = &mut self.prev_source_voltage;

        // For stages with an inter-stage coupling cap, the input signal is routed
        // through the WDF tree (VS = input) so the coupling cap naturally blocks
        // DC. In that case we skip the software grid_dc_blocker and also skip the
        // up-front set_control_voltage() — Vgk is computed inside the oversampler
        // closure from the cap's WDF state after tree.reflected().
        //
        // For all other stages (first stage, non-tube, JFET, etc.) we keep the
        // original software HPF + set_control_voltage() flow.
        let has_coupling_cap = coupling_cap_id.is_some();

        // Determine the grid voltage for the control input of tube stages.
        // For inter-stage triode/pentode stages the incoming signal carries the
        // previous stage's full plate voltage (DC + tiny AC). The coupling
        // capacitor between stages blocks DC in the real circuit; here we mimic
        // that with a software HPF (grid_dc_blocker) so the tube grid only sees
        // the AC component.
        //
        // α = 0.9 → τ ≈ 0.21ms at 48kHz. This fast convergence removes the large
        // DC offset from the previous stage's plate voltage within < 1ms, preventing
        // the tube from being driven with large positive Vgk during startup (which
        // would cause cathode bypass cap latch).
        //
        // The output is NOT clamped to ≤ 0: clamping causes half-wave rectification
        // of the inter-stage signal, as only negative Vgk excursions would pass
        // through to the grid. The full AC signal must be preserved for correct gain.
        //
        // Skip for coupling-cap stages: Vgk is computed from cap state inside closure.
        let grid_input = if !has_coupling_cap {
            if let Some((ref mut x_prev, ref mut y_prev)) = self.grid_dc_blocker {
                let x = input;
                let y = x - *x_prev + 0.9 * *y_prev;
                *x_prev = x;
                *y_prev = if y.is_finite() { y } else { 0.0 };
                *y_prev
            } else {
                input
            }
        } else {
            input
        };

        let negate_vs = self.negate_vs;
        // Set control voltage for active devices (triodes, pentodes).
        // No-op for diodes/JFETs/other passive roots.
        // Skip for coupling-cap stages: Vgk is set after tree.reflected() inside
        // the oversampler closure using the cap's WDF state.
        if !has_coupling_cap {
            root.set_control_voltage(grid_input, compensation, 0.0);
        }

        if source_probe.is_some() {
            match root {
                RootKind::Jfet(j) => {
                    j.set_vgs(j.vgs_bias() + input - *prev_source_voltage);
                }
                RootKind::Mosfet(m) => {
                    m.set_vgs(m.vgs_bias() + input - *prev_source_voltage);
                }
                _ => {}
            }
        }

        // For JFET source followers, compute Vgs from input (gate) and previous output (source).
        // Vgs = Vgate - Vsource, where Vgate ≈ input and Vsource is the WDF output.
        // We use the previous sample's source voltage for stability.
        if self.is_source_follower {
            if let RootKind::Jfet(ref mut j) = root {
                // Bias point: Vgs typically -0.5 to -2V for N-channel JFET
                // The input modulates around this bias point
                let vgs = j.vgs_bias() + input - *prev_source_voltage;
                j.set_vgs(vgs);
            }
        }

        // For source followers, the voltage source in the tree should NOT be
        // driven by input. The input signal modulates Vgs (gate voltage), and
        // the source follows via the JFET current. The voltage source is just
        // a WDF artifact that should be 0.
        let is_sf = self.is_source_follower;

        let wdf_out = self.oversampler.process(input, |sample| {
            // Determine voltage source value based on stage type:
            // - Triode with coupling cap: VS = input signal (cap blocks DC naturally)
            // - Triode without coupling cap: VS = B+ supply voltage (DC operating point)
            // - Pentode: VS = input signal (legacy; pentode's B+ comes from plate load)
            // - VariMu: VS = B+ supply (3-port MultiNlStage handles grid separately)
            // - Source follower: VS = 0 (input goes to gate, not VS)
            // - Feedback opamp + diode: opamp gain drives diode clipping
            // - Other: VS = input * compensation
            let vs_voltage = if let RootKind::Triode(t) = root {
                if has_coupling_cap {
                    // Route input through tree so coupling cap naturally blocks DC.
                    // B+ is injected separately via vcc_injection_coeff after reflected().
                    // negate_vs sign correction is also applied below.
                    sample * compensation
                } else {
                    // Use B+ as VS for triodes. negate_vs (applied below) handles
                    // Series adaptor sign correction when needed.
                    t.v_max()
                }
            } else if let RootKind::Pentode(_) = root {
                // Pentodes use VS = input signal in both cases (coupling-cap or not).
                // When has_coupling_cap: cap blocks DC naturally, B+ injected via
                // vcc_injection_coeff after reflected().
                // When no coupling cap: original behavior (signal is first-stage input).
                sample * compensation
            } else if let RootKind::VariMu(t) = root {
                t.v_max()
            } else if is_sf {
                0.0 // Source follower: input modulates Vgs, not VS
            } else if let Some(ref mut opamp) = feedback_opamp {
                // OpAmp feedback diode stage: apply gain + GBW + slew.
                // compute_vs_voltage returns |gain| * input (positive magnitude).
                // The DiodePair negation below handles WDF sign convention.
                opamp.compute_vs_voltage(sample * compensation)
            } else {
                sample * compensation
            };
            // Negate for non-inverting root kinds (series adaptor convention:
            // b = -(b1+b2) negates the VS contribution, which phase-inverting
            // devices like triodes/BJTs cancel naturally, but diodes/zeners don't)
            let vs_voltage = match root {
                RootKind::DiodePair(_)
                | RootKind::SingleDiode(_)
                | RootKind::ExplicitDiodePair(_)
                | RootKind::ExplicitSingleDiode(_)
                | RootKind::Zener(_)
                | RootKind::ZenerPair(_) => -vs_voltage,
                _ => vs_voltage,
            };
            // Apply tree-topology sign correction (detected during construction).
            // Some trees produce negative b_tree with positive VS due to Series
            // adaptor nesting — negate VS to make b_tree positive for the NR solver.
            let vs_voltage = if negate_vs { -vs_voltage } else { vs_voltage };
            // DynNode::set_voltage already has a fast path (root→left child
            // pattern match, ~2 ops). Using the cached raw pointer here would
            // alias with the live &mut tree borrow — technically UB.
            tree.set_voltage(vs_voltage);
            let b1 = tree.reflected_with_state(runtime_state);

            // For coupling-cap stages: after tree.reflected(), the capacitor's WDF
            // state encodes its DC charge (the DC component of previous input samples).
            // Extract it to compute Vgk = GRID_BIAS + (input_ac) where
            //   input_ac = sample * compensation - cap_voltage  (DC blocked by cap).
            // This must happen after reflected() so the cap state is current.
            if has_coupling_cap {
                if let Some(ref cap_id) = coupling_cap_id {
                    // cap_voltage is the voltage across the coupling cap from the
                    // previous down-sweep (one-sample delay, negligible at audio rates).
                    let cap_v = tree
                        .leaf_voltage_with_state(cap_id, runtime_state)
                        .unwrap_or(0.0);
                    // AC grid voltage = total input minus cap's stored DC
                    let vgk_ac = sample * compensation - cap_v;
                    root.set_control_voltage(vgk_ac, 1.0, 0.0);
                }
            }

            // VCC bias injection: add DC operating point from supply voltage.
            // Ramped over 256 samples to prevent NR solver divergence on startup.
            // For coupling-cap stages, vcc_injection_coeff = k_vs * B+ (non-zero).
            // For other triode stages, vcc_injection_coeff = 0.0 (B+ already in VS).
            let b1 = if vcc_injection_coeff != 0.0 {
                const DC_RAMP_SAMPLES: u32 = 256;
                let dc_scale = if *vcc_dc_ramp >= DC_RAMP_SAMPLES {
                    1.0
                } else {
                    *vcc_dc_ramp += 1;
                    *vcc_dc_ramp as crate::Wave / DC_RAMP_SAMPLES as crate::Wave
                };
                b1 + vcc_injection_coeff * dc_scale
            } else {
                b1
            };

            let b_tree = b1;

            let solver = NonlinearSolver::from_k_table(k_table.as_ref());
            let solver_control = match root {
                RootKind::Bjt(b) => b.vbe() - b.vbe_bias(),
                RootKind::Triode(t) => t.vgk() - t.vgk_bias(),
                RootKind::Jfet(j) => j.vgs() - j.vgs_bias(),
                RootKind::Mosfet(m) => m.vgs() - m.vgs_bias(),
                RootKind::DiffPair(dp) => {
                    // Ctrl axis = I_tail modulation relative to bias
                    (dp.i_tail / dp.i_tail_bias()) - 1.0
                }
                _ => 0.0,
            };

            let a_root = if let Some(a_root) = solver.solve_root_incident(b_tree, solver_control) {
                a_root
            } else {
                // NR fallback — rp only needed here, not for K-table path
                let rp = tree.port_resistance();
                match root {
                    RootKind::DiodePair(dp) => dp.process(b_tree, rp),
                    RootKind::SingleDiode(d) => d.process(b_tree, rp),
                    RootKind::ExplicitDiodePair(dp) => dp.process(b_tree, rp),
                    RootKind::ExplicitSingleDiode(d) => d.process(b_tree, rp),
                    RootKind::Zener(z) => z.process(b_tree, rp),
                    RootKind::ZenerPair(z) => z.process(b_tree, rp),
                    RootKind::Jfet(j) => {
                        if is_sf {
                            j.process_source_follower(b_tree, rp, sample * compensation)
                        } else {
                            j.process(b_tree, rp)
                        }
                    }
                    RootKind::JfetVr(j) => j.process_root(b_tree, rp),
                    RootKind::Triode(t) => t.process(b_tree, rp),
                    RootKind::VariMu(t) => t.process(b_tree, rp),
                    RootKind::Pentode(p) => p.process(b_tree, rp),
                    RootKind::Mosfet(m) => m.process(b_tree, rp),
                    RootKind::Bjt(b) => b.process(b_tree, rp),
                    RootKind::DiffPair(dp) => dp.process(b_tree, rp),
                    RootKind::Ota(o) => o.process(b_tree, rp),
                    RootKind::OpAmp(op) => {
                        if op.is_non_inverting() {
                            op.set_vp(sample * compensation);
                        }
                        op.process(b_tree, rp)
                    }
                    // Passthrough: open-circuit termination (b = a)
                    // The tree processes normally but the root just reflects.
                    // For passive filters with voltage source, the output voltage
                    // should be extracted at the load resistor, not the root port.
                    RootKind::Passthrough => {
                        // Open-circuit termination: a_root = b_tree (total reflection).
                        // The unnamed voltage source leaf emits b = 2V, so when there
                        // is no explicit output probe the root fallback must report
                        // the physical source-side voltage, not the wave magnitude.
                        //
                        // MERGE NOTE (#85 double down-sweep fix): the down-sweep is
                        // done HERE via set_incident_with_state(b_tree). We MUST
                        // early-return so the shared down-sweep at the end of this
                        // function (`tree.set_incident_with_state(a_root, ...)`) is
                        // never reached from the Passthrough path — re-scattering
                        // with a_root = b_tree/2.0 corrupts capacitor state and
                        // produces 0.39x gain instead of 1.0x. Returning the value
                        // (instead of falling through) preserves single-down-sweep
                        // semantics.
                        tree.set_incident_with_state(b_tree, runtime_state);
                        // Check output_probe BEFORE fallback
                        if let Some(ref probe_id) = output_probe {
                            if let Some(v) = tree.leaf_voltage_with_state(probe_id, runtime_state) {
                                return v;
                            }
                        }
                        return b_tree / 2.0;
                    }
                    // ShortCircuit: ground termination (a = -b)
                    // Ground has zero impedance, so it reflects with inverted sign.
                    // This allows current to flow through series elements to ground.
                    RootKind::ShortCircuit => {
                        // Short-circuit reflection: a = -b
                        let a_root = -b_tree;
                        tree.set_incident_with_state(a_root, runtime_state);

                        // Check output_probe first (for SP-reduced orphan stages).
                        // MERGE NOTE (#85): leaf_voltage() returns a/2 for resistors;
                        // for a VS-driven tree the WDF b=2V convention inflates a by
                        // 2× relative to physical. The short_circuit_junction_voltage()
                        // path already divides by 2 when a VS is detected, so divide
                        // here too for consistency.
                        if let Some(ref probe_id) = output_probe {
                            if let Some(v) = tree.leaf_voltage_with_state(probe_id, runtime_state) {
                                return v / 2.0;
                            }
                        }
                        // Extract output at junction (voltage across load resistor)
                        // For Series(VS, Series(L, R)) with short at gnd:
                        // - VS emits b_vs = 2 * Vin (already done in reflected())
                        // - We need V_R = voltage across R (between L.b/R.a and gnd)
                        if let Some(v_junction) = tree.short_circuit_junction_voltage(a_root) {
                            return v_junction;
                        }
                        // Fallback: V at grounded port is 0 (short circuit)
                        0.0
                    }
                    // VoltageSourceDriver: ideal voltage source at root port.
                    //
                    // Correct WDF scattering for ideal VS: a_root = 2*V - b_tree.
                    // This ensures V_port = (a+b)/2 = V regardless of b_tree,
                    // giving the exact bilinear-transform pole (k-1)/(k+1).
                    //
                    // Output at series junction is negated (WDF sign convention:
                    // V_root = -(V_left + V_right) in Fettweis series adaptor).
                    RootKind::VoltageSourceDriver => {
                        let v_in = sample * compensation;
                        let a_root = 2.0 * v_in - b_tree;
                        tree.set_incident_with_state(a_root, runtime_state);

                        // Check output_probe first (for SP-reduced feedforward stages)
                        if let Some(ref probe_id) = output_probe {
                            if let Some(v) = tree.leaf_voltage_with_state(probe_id, runtime_state) {
                                return v;
                            }
                        }
                        if let Some(v_junction) = tree.series_junction_voltage(a_root) {
                            return -v_junction;
                        }
                        return (a_root + b_tree) / 2.0;
                    }
                    // Capacitor root: b = state (reflects stored incident)
                    // This gives correct RC lowpass transfer function.
                    RootKind::CapacitorRoot { state, .. } => {
                        let b_root = *state;
                        *state = b_tree; // Update state with new incident
                        b_root
                    }
                    // Inductor root: b = -state (reflects negated stored incident)
                    // This gives correct RL highpass transfer function.
                    RootKind::InductorRoot { state, .. } => {
                        let b_root = -*state;
                        *state = b_tree; // Update state with new incident
                        b_root
                    }
                    // Resistive termination: resistor absorbs all energy (b = 0).
                    // Output at root port = (0 + b_tree) / 2 = b_tree / 2.
                    RootKind::ResistiveTermination => 0.0,
                    // Passive R-type: self-contained processing bypassing the main tree.
                    // VS is an ideal voltage source (zero impedance) stamped into
                    // the MNA, producing an injection vector separate from the
                    // scattering matrix.
                    RootKind::PassiveRType {
                        scattering,
                        vs_injection,
                        n_ports,
                        children,
                        child_runtime_states,
                        output_port,
                        extraction_coeffs,
                        extraction_vs,
                        ..
                    } => {
                        let vs_voltage = sample * compensation;
                        let n = *n_ports;
                        // 1. Collect reflected waves from children
                        let b_children: Vec<crate::Wave> = children
                            .iter_mut()
                            .zip(child_runtime_states.iter_mut())
                            .map(|(child, state)| child.reflected_with_state(state))
                            .collect();
                        // 2. Compute incident waves: a[i] = Σ_j S[i][j]·b[j] + k[i]·V_in
                        let mut a_children = vec![0.0; n];
                        for i in 0..n {
                            let mut a_i = vs_injection[i] * vs_voltage;
                            for j in 0..n {
                                a_i += scattering[i * n + j] * b_children[j];
                            }
                            a_children[i] = a_i;
                        }
                        // 3. Set incident waves on children
                        for ((child, state), &a_i) in children
                            .iter_mut()
                            .zip(child_runtime_states.iter_mut())
                            .zip(a_children.iter())
                        {
                            child.set_incident_with_state(a_i, state);
                        }
                        // 4. Output voltage at probe port
                        if !extraction_coeffs.is_empty() {
                            let mut out = *extraction_vs * vs_voltage;
                            for i in 0..n {
                                out += extraction_coeffs[i] * b_children[i];
                            }
                            return out;
                        } else if n == 0 || *extraction_vs != 0.0 {
                            return *extraction_vs * vs_voltage;
                        } else {
                            let a_out = a_children[*output_port];
                            let b_out = b_children[*output_port];
                            return (a_out + b_out) / 2.0;
                        }
                    }
                } // end NR fallback else
            }; // end K-table if/else
               // ── Down-sweep ────────────────────────────────────────────
            tree.set_incident_with_state(a_root, runtime_state);

            // If an output probe is set, extract voltage at that leaf
            // after the down-sweep instead of the root junction.
            if let Some(ref probe_id) = output_probe {
                if let Some(v) = tree.leaf_voltage_with_state(probe_id, runtime_state) {
                    return v;
                }
            }
            if let Some(ref probe_id) = source_probe {
                if let Some(v) = tree.leaf_voltage_with_state(probe_id, runtime_state) {
                    *prev_source_voltage = v;
                }
            }

            // For feedback_opamp stages (op-amp + diode clipping):
            // The VS drives gain × input into the tree. The diode root
            // clips the wave nonlinearly (Wright Omega). The clipped output
            // voltage is vs_voltage shaped by the diode's I-V curve.
            //
            // We use vs_voltage as the base (correct gain) and apply the
            // diode's clipping from the WDF wave variables. The diode
            // voltage v_d = (a_root + b_tree) / 2 tells us how much
            // the diode conducted. The output is vs_voltage minus the
            // voltage absorbed by the diode current through Rf.
            // The junction voltage v_d = (a_root + b_tree) / 2 is the voltage
            // at the root port (= diode junction voltage in WDF theory).
            //
            // For feedback_opamp stages: V_out = V_diode because the diode
            // is across the feedback path (V_out - V_neg) and V_neg ≈ 0
            // (virtual ground). The diode clamps V_out to ±Vf.
            match root {
                RootKind::DiodePair(_)
                | RootKind::SingleDiode(_)
                | RootKind::ExplicitDiodePair(_)
                | RootKind::ExplicitSingleDiode(_)
                | RootKind::Zener(_)
                | RootKind::ZenerPair(_) => {
                    // Diode junction voltage (anode-cathode) in WDF wave terms.
                    let v_diode = (a_root + b_tree) / 2.0;
                    if let Some(divider) = series_rectifier_divider {
                        // Series rectifier: output is the load voltage at the
                        // D1/RL junction. By KVL around the series loop
                        //   V_RL = (Vin - V_diode) * RL/(R1+RL)
                        // which collapses to ~0 when the diode blocks (correct
                        // half-wave rectification) and rises when it conducts.
                        let v_in = sample * compensation;
                        (v_in - v_diode) * divider
                    } else {
                        // Shunt clipper / op-amp feedback clipper: the junction
                        // voltage IS the output.
                        -v_diode
                    }
                }
                _ => (a_root + b_tree) / 2.0,
            }
        });

        // Inverting all-pass feedback (Phase 90 topology).
        // Bypasses WDF output: V_out = -(Z_fb/Z_in) * V_in where Z_fb = Rf||Cf (IIR)
        // and Z_in = R_ap + R_jfet (variable from LFO modulation).
        if let Some(ref mut fb) = self.allpass_feedback {
            let r_jfet = match &self.root {
                RootKind::JfetVr(jvr) => jvr.rds(),
                _ => fb.r_ap,
            };
            let z_in = fb.r_ap + r_jfet;
            let i_in = (input * self.compensation) / z_in;
            let v_fb = fb.b0 * (i_in + fb.x_prev) + fb.a1 * fb.y_prev;
            fb.x_prev = i_in;
            fb.y_prev = flush_denormal(v_fb);
            return -fb.y_prev;
        }

        // Bridged-T resonator IIR (opamp with series R1-R2 and shunt C1/C2).
        // Bypasses WDF output with a 2nd-order bandpass filter whose
        // resonant frequency matches 1/(2π·√(R1·R2·C1·C2)).
        if let Some(ref mut rf) = self.resonator_feedback {
            return flush_denormal(rf.process(input * self.compensation));
        }

        // Non-inverting all-pass IIR (Phase 90 gain-of-2 style).
        // The WDF tree runs the JFET NR solver to maintain its Vgs/Vds state,
        // but the output comes from a direct IIR: H(s) = (1-sRC)/(1+sRC)
        // where R = JFET Rds (variable) and C = phase-shift cap.
        // Bilinear: y[n] = a1*(x[n] - y[n-1]) + x[n-1], a1 = (1-K)/(1+K), K = 2fs*R*C.
        if let Some(ref mut ap) = self.allpass_direct {
            let rds = match &self.root {
                RootKind::Jfet(j) => j.rds_approx(),
                RootKind::JfetVr(jvr) => jvr.rds(),
                _ => 10_000.0,
            };
            let k = 2.0 * ap.sample_rate * rds * ap.cap;
            let a1 = (1.0 - k) / (1.0 + k);
            let x = input * self.compensation;
            let y = a1 * (x - ap.y_prev) + ap.x_prev;
            ap.x_prev = x;
            ap.y_prev = flush_denormal(y);
            return ap.y_prev;
        }

        // All-pass with op-amp buffer (bridged-T topology):
        // V_allpass = 2 * V_junction - V_in.
        // The WDF tree models the RC network phase shift; the op-amp
        // reconstructs the all-pass output at unity magnitude.
        if self.paired_opamp.is_some() {
            return flush_denormal(2.0 * wdf_out - input);
        }

        // Update source follower state (for next sample's Vgs calculation)
        if self.is_source_follower {
            self.prev_source_voltage = wdf_out;
        }

        let wdf_out = wdf_out + self.output_bias;

        // Apply DC-blocking filter for triode stages.
        // This models the output coupling capacitor (C_out) which blocks DC
        // and passes AC. The filter is a single-pole IIR highpass.
        if let Some((a1, b0, ref mut y_prev, ref mut x_prev)) = self.dc_block {
            // IIR highpass: y[n] = b0 * (x[n] - x[n-1]) + a1 * y[n-1]
            let x = wdf_out;
            let y = b0 * (x - *x_prev) + a1 * *y_prev;
            *x_prev = x;
            *y_prev = flush_denormal(y);

            return *y_prev;
        }

        flush_denormal(wdf_out)
    }

    /// Adjust reactive element port resistances for oversampling.
    ///
    /// When oversampling is active, the WDF cycle runs at `base_rate * ratio`.
    /// Reactive elements (C/L) must use this effective rate for correct
    /// bilinear-transform discretization. Must be called after construction
    /// when the oversampling factor > 1.
    pub fn apply_oversampling_rate(&mut self, base_rate: crate::Wave) {
        let ratio = self.oversampler.ratio();
        if ratio <= 1 {
            return;
        }
        let effective_rate = base_rate * ratio as crate::Wave;
        self.tree.set_sample_rate(effective_rate);
        self.tree.recompute();

        // Also update CapacitorRoot / InductorRoot port resistances
        match &mut self.root {
            RootKind::CapacitorRoot {
                capacitance, rp, ..
            } => {
                *rp = 1.0 / (2.0 * effective_rate * *capacitance);
            }
            RootKind::InductorRoot { inductance, rp, .. } => {
                *rp = 2.0 * effective_rate * *inductance;
            }
            RootKind::PassiveRType { .. } => {
                // Children and scattering matrix were derived at the effective
                // (oversampled) rate during construction. No update needed.
            }
            RootKind::OpAmp(op) => {
                // OpAmpRoot GBW filter and slew rate limiter depend on sample rate.
                // Without this, both are 4x too gentle at X4 oversampling.
                op.set_sample_rate(effective_rate);
            }
            _ => {}
        }

        // Update OpAmpRoots used as feedback/paired opamps in diode stages.
        // These also have GBW and slew rate filters that need the oversampled rate.
        if let Some(ref mut opamp) = self.feedback_opamp {
            opamp.set_sample_rate(effective_rate);
        }
        if let Some(ref mut opamp) = self.paired_opamp {
            opamp.set_sample_rate(effective_rate);
        }
    }

    pub fn reset(&mut self) {
        self.tree.reset_with_state(&mut self.runtime_state);
        self.oversampler.reset();
        // Reset returns controlled-resistor leaves to their dark/zero-bias
        // resistance without re-deriving scattering; drop the gating
        // baseline so the first post-reset control write recomputes.
        self.ctrl_r_last_applied.clear();
        self.ensure_passive_rtype_child_runtime_states();
        if let Some(ref mut opamp) = self.paired_opamp {
            opamp.reset();
        }
        if let Some(ref mut opamp) = self.feedback_opamp {
            opamp.reset();
        }
        if let Some((_, _, ref mut y_prev, ref mut x_prev)) = self.dc_block {
            *y_prev = 0.0;
            *x_prev = 0.0;
        }
        if let Some((ref mut x_prev, ref mut y_prev)) = self.grid_dc_blocker {
            *x_prev = 0.0;
            *y_prev = 0.0;
        }
        self.prev_source_voltage = 0.0;
        if let RootKind::Bjt(ref mut bjt) = self.root {
            bjt.reset();
        }
        if let RootKind::PassiveRType {
            children,
            child_runtime_states,
            ..
        } = &mut self.root
        {
            for (child, state) in children.iter_mut().zip(child_runtime_states.iter_mut()) {
                child.reset_with_state(state);
            }
        }
        self.ensure_opamp_adaptor_runtime_states();
        for (child, state) in self
            .opamp_children
            .iter_mut()
            .zip(self.opamp_child_runtime_states.iter_mut())
        {
            child.reset_with_state(state);
        }
        if let Some(ref mut zf) = self.zf_child {
            zf.reset_with_state(&mut self.zf_child_runtime_state);
        }
        if let Some(ref mut zg) = self.zg_child {
            zg.reset_with_state(&mut self.zg_child_runtime_state);
        }
    }

    /// Set a pot value in the PassiveRType children and mark for recompute.
    ///
    /// Returns `true` if the pot was found and updated.
    pub fn set_passive_rtype_pot(&mut self, comp_id: &str, value: crate::Wave) -> bool {
        if let RootKind::PassiveRType {
            children,
            needs_recompute,
            ..
        } = &mut self.root
        {
            let mut found = false;
            for child in children.iter_mut() {
                if child.set_pot(comp_id, value) {
                    found = true;
                }
            }
            if found {
                *needs_recompute = true;
            }
            return found;
        }
        false
    }

    /// Re-derive scattering matrix from stored MNA after pot changes.
    ///
    /// Removes old variable-resistor conductances from the G matrix, adds new ones based
    /// on current DynNode::Pot resistance, then re-derives S and k vectors.
    pub fn flush_passive_rtype_recompute(&mut self) {
        if let RootKind::PassiveRType {
            scattering,
            vs_injection,
            needs_recompute,
            recompute_mna,
            recompute_ports,
            variable_resistors,
            children,
            interp_table,
            extraction_coeffs,
            extraction_vs,
            extraction_output_pos,
            extraction_output_neg,
            ..
        } = &mut self.root
        {
            if !*needs_recompute {
                return;
            }

            // Fast path: interpolation table lookup for single-pot stages
            if let Some(table) = interp_table.as_ref() {
                if variable_resistors.len() == 1 {
                    let pot_r = children[variable_resistors[0].child_idx].port_resistance();
                    let (new_s, new_k) = table.lookup(pot_r);
                    *scattering = new_s;
                    *vs_injection = new_k;
                    *needs_recompute = false;
                    return;
                }
            }

            // Slow path: full MNA re-derivation
            if let (Some(mna), Some(ports)) = (recompute_mna.as_mut(), recompute_ports.as_ref()) {
                let n = mna.num_nodes;
                // Update G matrix: remove old conductance, add new.
                for binding in variable_resistors.iter_mut() {
                    let new_r = children[binding.child_idx].port_resistance();
                    let new_g = 1.0 / new_r;
                    let (n1, n2) = binding.terminals.raw().as_tuple();
                    stamp_g(&mut mna.g_matrix, n, n1, n2, -binding.conductance);
                    stamp_g(&mut mna.g_matrix, n, n1, n2, new_g);
                    binding.conductance = new_g;
                }
                // Re-derive scattering matrix and VS injection vector
                let (new_s, new_k) = mna.derive_scattering_and_vs_injection(ports, 0);
                let (new_extract, new_extract_vs) = mna.derive_extraction_coeffs(
                    ports,
                    0,
                    *extraction_output_pos,
                    *extraction_output_neg,
                );
                *extraction_coeffs = new_extract;
                *extraction_vs = new_extract_vs;
                *scattering = new_s;
                *vs_injection = new_k;
                *needs_recompute = false;
            }
        }
    }

    /// Returns true if this stage has an interpolation table for fast pot recompute.
    pub fn has_interp_table(&self) -> bool {
        if let RootKind::PassiveRType { interp_table, .. } = &self.root {
            interp_table.is_some()
        } else {
            false
        }
    }

    /// Apply thermal drift to temperature-sensitive root elements.
    ///
    /// Modulates diode Is/n_vt and BJT vt/Is/bf based on the current thermal
    /// state. Uses stored base models to prevent multiplier accumulation.
    ///
    /// Thermal time constants are 10–100s, so this is called at the existing
    /// `ThermalModel::update_interval` cadence (~1000 samples), not per-sample.
    pub fn apply_thermal(&mut self, state: &crate::thermal::ThermalState) {
        // Diode roots: modulate Is and n_vt.
        if let Some(base) = &self.base_diode_model {
            let ideality_ratio = base.n_vt / 0.02585; // n factor (ideality * Vt_ref)
            match &mut self.root {
                RootKind::DiodePair(dp) => {
                    dp.model.is = base.is * state.is_multiplier;
                    dp.model.n_vt = ideality_ratio * state.vt;
                }
                RootKind::SingleDiode(d) => {
                    d.model.is = base.is * state.is_multiplier;
                    d.model.n_vt = ideality_ratio * state.vt;
                }
                RootKind::ExplicitDiodePair(dp) => {
                    dp.model.is = base.is * state.is_multiplier;
                    dp.model.n_vt = ideality_ratio * state.vt;
                }
                RootKind::ExplicitSingleDiode(d) => {
                    d.model.is = base.is * state.is_multiplier;
                    d.model.n_vt = ideality_ratio * state.vt;
                }
                _ => {}
            }
        }

        // BJT WDF root: modulate vt, Is, and bf.
        // vt scales with absolute temperature (Boltzmann).
        // Is doubles every ~10°C (silicon) / ~8°C (germanium).
        // bf drifts linearly with beta_tempco.
        if let Some(base) = &self.base_bjt_model {
            if let RootKind::Bjt(bjt) = &mut self.root {
                bjt.model.vt = state.vt;
                bjt.model.is = base.is * state.is_multiplier;
                bjt.model.bf = (base.bf * state.beta_multiplier).max(1.0);
            }
        }
    }

    /// Balance the voltage source impedance to match the network.
    ///
    /// When the Vs branch is inside a Parallel adaptor whose sibling has much
    /// higher impedance, the signal is heavily attenuated.  This adjusts the
    /// Vs port resistance so the branches are balanced (gamma ≈ 0.5).
    pub fn balance_vs_impedance(&mut self) {
        balance_parallel_vs(&mut self.tree);
        self.tree.recompute();
        self.tree.compute_dynamic_flags();
    }

    /// Set the gate-source voltage for JFET root elements.
    ///
    /// This is used for external modulation (LFO, envelope, etc.).
    /// Has no effect if the root is not a JFET.
    #[inline]
    pub fn set_jfet_vgs(&mut self, vgs: crate::Wave) {
        match &mut self.root {
            RootKind::Jfet(j) => j.set_vgs(vgs),
            RootKind::JfetVr(j) => j.set_vgs(vgs),
            _ => {}
        }
    }

    /// Get the current gate-source voltage if this is a JFET stage.
    #[allow(dead_code)]
    pub fn jfet_vgs(&self) -> Option<crate::Wave> {
        match &self.root {
            RootKind::Jfet(j) => Some(j.vgs()),
            RootKind::JfetVr(j) => Some(j.vgs()),
            _ => None,
        }
    }

    /// Current effective resistance of the controlled-resistor leaf with
    /// the given component id (WDF tree or PassiveRType MNA children).
    fn controlled_leaf_resistance(&self, comp_id: &str) -> Option<crate::Wave> {
        let probe = |leaf: &dyn crate::wdf_leaf::WdfLeaf| -> Option<crate::Wave> {
            (leaf.comp_id() == Some(comp_id)).then(|| leaf.port_resistance())
        };
        if let Some(r) = self.tree.find_leaf(&probe) {
            return Some(r);
        }
        if let RootKind::PassiveRType { children, .. } = &self.root {
            for child in children {
                if let Some(r) = child.find_leaf(&probe) {
                    return Some(r);
                }
            }
        }
        None
    }

    /// Threshold gate for controlled-resistor (photocoupler / `jfet_vr`)
    /// scattering re-derivations. Returns `true` when the re-derivation
    /// should run, recording the new resistance as the last-applied value.
    ///
    /// Physics justification: the CdS cell's resistance trajectory is
    /// band-limited by its own ms-to-s time constants, so its control
    /// content lives below a few hundred Hz — re-deriving the scattering
    /// matrix at audio rate is wasted work. The element's internal state
    /// (carrier dynamics / Vgs) still updates every sample upstream of this
    /// gate, so timing physics are unaffected. The comparison is against
    /// the last-APPLIED resistance, so sub-epsilon per-sample drift
    /// accumulates and eventually triggers a recompute: steady-state gain
    /// error is bounded by [`CTRL_R_RECOMPUTE_EPS`].
    fn ctrl_resistance_drifted(&mut self, comp_id: &str) -> bool {
        let Some(new_r) = self.controlled_leaf_resistance(comp_id) else {
            // Can't read the leaf back — recompute unconditionally (safe).
            return true;
        };
        for entry in self.ctrl_r_last_applied.iter_mut() {
            if entry.comp_id == comp_id {
                if (new_r - entry.last_applied_r).abs()
                    <= CTRL_R_RECOMPUTE_EPS * entry.last_applied_r.abs()
                {
                    return false;
                }
                entry.last_applied_r = new_r;
                return true;
            }
        }
        self.ctrl_r_last_applied.push(CtrlRecomputeGate {
            comp_id: String::from(comp_id),
            last_applied_r: new_r,
        });
        true
    }

    /// Set Vgs on a `jfet_vr` LEAF anywhere in this stage — in the WDF tree
    /// or among PassiveRType MNA children — and trigger the matching
    /// impedance recompute.
    ///
    /// Gate-modulated JFETs compile to `jfet_vr` leaves (variable
    /// resistors), not stage roots, so external modulation (LFO, envelope)
    /// must reach the leaf; the root setter [`Self::set_jfet_vgs`] silently
    /// no-ops for them. Returns `true` if the component was found.
    ///
    /// The leaf's Vgs/Rds state updates every sample; the expensive
    /// adaptor/scattering re-derivation is threshold-gated — see
    /// [`Self::ctrl_resistance_drifted`].
    pub fn set_jfet_vr_vgs(&mut self, comp_id: &str, vgs: crate::Wave) -> bool {
        if self.tree.set_jfet_vr_vgs(comp_id, vgs) {
            if self.ctrl_resistance_drifted(comp_id) {
                self.tree.recompute();
            }
            return true;
        }
        let mut found = false;
        if let RootKind::PassiveRType { children, .. } = &mut self.root {
            for child in children.iter_mut() {
                if child.set_jfet_vr_vgs(comp_id, vgs) {
                    found = true;
                }
            }
        }
        if found && self.ctrl_resistance_drifted(comp_id) {
            if let RootKind::PassiveRType {
                needs_recompute, ..
            } = &mut self.root
            {
                *needs_recompute = true;
            }
            // Re-derive the scattering matrix with the JFET's new Rds.
            self.flush_passive_rtype_recompute();
        }
        found
    }

    /// Whether this stage contains ANY leaf belonging to the given component
    /// id (in the WDF tree or among PassiveRType MNA children), or owns it
    /// as a pot. Used by envelope-tap resolution to map a detector's tap
    /// node onto the stage that produces its signal.
    pub fn contains_component(&self, comp_id: &str) -> bool {
        let probe = |leaf: &dyn crate::wdf_leaf::WdfLeaf| -> Option<()> {
            (leaf.comp_id() == Some(comp_id)).then_some(())
        };
        if self.tree.find_leaf(&probe).is_some() {
            return true;
        }
        if let RootKind::PassiveRType { children, .. } = &self.root {
            if children.iter().any(|c| c.find_leaf(&probe).is_some()) {
                return true;
            }
        }
        self.has_pot(comp_id) || self.root_comp_id == comp_id
    }

    /// Whether this stage contains a `jfet_vr` leaf with the given component
    /// id (in the WDF tree or among PassiveRType MNA children).
    pub fn contains_jfet_vr(&self, comp_id: &str) -> bool {
        let probe = |leaf: &dyn crate::wdf_leaf::WdfLeaf| -> Option<()> {
            (leaf.type_tag() == "jfet_vr" && leaf.comp_id() == Some(comp_id)).then_some(())
        };
        if self.tree.find_leaf(&probe).is_some() {
            return true;
        }
        if let RootKind::PassiveRType { children, .. } = &self.root {
            return children.iter().any(|c| c.find_leaf(&probe).is_some());
        }
        false
    }

    /// Whether this stage contains a photocoupler LEAF with the given
    /// component id (in the WDF tree or among PassiveRType MNA children).
    ///
    /// A leaf is the photocoupler at its netlist position — a dynamic
    /// resistance in the audio path. This deliberately does NOT look at
    /// `input_photocouplers` (the op-amp input-path gain model): the two
    /// representations are exclusive (audit gap G3) and binding resolution
    /// must prefer the leaf.
    pub fn contains_photocoupler(&self, comp_id: &str) -> bool {
        let probe = |leaf: &dyn crate::wdf_leaf::WdfLeaf| -> Option<()> {
            (leaf.type_tag() == "photocoupler" && leaf.comp_id() == Some(comp_id)).then_some(())
        };
        if self.tree.find_leaf(&probe).is_some() {
            return true;
        }
        if let RootKind::PassiveRType { children, .. } = &self.root {
            return children.iter().any(|c| c.find_leaf(&probe).is_some());
        }
        false
    }

    /// Set LED drive on a photocoupler LEAF anywhere in this stage — in the
    /// WDF tree or among PassiveRType MNA children — and trigger the
    /// matching impedance recompute. Returns `true` if the component was
    /// found.
    ///
    /// Mirrors [`Self::set_jfet_vr_vgs`]: photocouplers inside a
    /// PassiveRType stage are MNA variable-resistor children, so after the
    /// LED drive updates the CdS resistance the scattering matrix must be
    /// re-derived for the gain change to reach the audio path.
    ///
    /// The CdS carrier state updates every sample (the photocoupler's
    /// attack/release physics live in the leaf); the expensive scattering
    /// re-derivation is threshold-gated — see
    /// [`Self::ctrl_resistance_drifted`].
    pub fn set_photocoupler_led(&mut self, comp_id: &str, led_drive: crate::Wave) -> bool {
        if self.tree.set_photocoupler_led(comp_id, led_drive) {
            if self.ctrl_resistance_drifted(comp_id) {
                self.tree.recompute();
            }
            return true;
        }
        let mut found = false;
        if let RootKind::PassiveRType { children, .. } = &mut self.root {
            for child in children.iter_mut() {
                if child.set_photocoupler_led(comp_id, led_drive) {
                    found = true;
                }
            }
        }
        if found && self.ctrl_resistance_drifted(comp_id) {
            if let RootKind::PassiveRType {
                needs_recompute, ..
            } = &mut self.root
            {
                *needs_recompute = true;
            }
            // Re-derive the scattering matrix with the CdS cell's new R.
            self.flush_passive_rtype_recompute();
        }
        found
    }

    /// Set the grid-cathode voltage for triode root elements.
    ///
    /// This is used for external modulation (bias, LFO, signal input).
    /// Has no effect if the root is not a triode.
    #[inline]
    pub fn set_triode_vgk(&mut self, vgk: crate::Wave) {
        if let RootKind::Triode(t) = &mut self.root {
            t.set_vgk(vgk);
        }
    }

    /// Get the current grid-cathode voltage if this is a triode stage.
    #[allow(dead_code)]
    pub fn triode_vgk(&self) -> Option<crate::Wave> {
        match &self.root {
            RootKind::Triode(t) => Some(t.vgk()),
            _ => None,
        }
    }

    /// Set the grid-cathode voltage for variable-mu triode root elements.
    #[inline]
    pub fn set_vari_mu_vgk(&mut self, vgk: crate::Wave) {
        if let RootKind::VariMu(t) = &mut self.root {
            t.set_vgk(vgk);
        }
    }

    /// Get the current grid-cathode voltage if this is a variable-mu triode stage.
    #[allow(dead_code)]
    pub fn vari_mu_vgk(&self) -> Option<crate::Wave> {
        match &self.root {
            RootKind::VariMu(t) => Some(t.vgk()),
            _ => None,
        }
    }

    /// Set the control grid voltage (g1-cathode) for pentode root elements.
    #[inline]
    pub fn set_pentode_vg1k(&mut self, vg1k: crate::Wave) {
        if let RootKind::Pentode(p) = &mut self.root {
            p.set_vg1k(vg1k);
        }
    }

    /// Set the screen grid voltage (g2-cathode) for pentode root elements.
    #[allow(dead_code)]
    #[inline]
    pub fn set_pentode_vg2k(&mut self, vg2k: crate::Wave) {
        if let RootKind::Pentode(p) = &mut self.root {
            p.set_vg2k(vg2k);
        }
    }

    /// Get the current control grid voltage if this is a pentode stage.
    #[allow(dead_code)]
    pub fn pentode_vg1k(&self) -> Option<crate::Wave> {
        match &self.root {
            RootKind::Pentode(p) => Some(p.vg1k()),
            _ => None,
        }
    }

    /// Set the gate-source voltage for MOSFET root elements.
    #[inline]
    pub fn set_mosfet_vgs(&mut self, vgs: crate::Wave) {
        if let RootKind::Mosfet(m) = &mut self.root {
            m.set_vgs(vgs);
        }
    }

    /// Get the current gate-source voltage if this is a MOSFET stage.
    #[allow(dead_code)]
    pub fn mosfet_vgs(&self) -> Option<crate::Wave> {
        match &self.root {
            RootKind::Mosfet(m) => Some(m.vgs()),
            _ => None,
        }
    }

    /// Set the OTA bias current (for envelope-controlled gain).
    #[allow(dead_code)]
    #[inline]
    pub fn set_ota_iabc(&mut self, iabc: crate::Wave) {
        if let RootKind::Ota(o) = &mut self.root {
            o.set_iabc(iabc);
        }
    }

    /// Set OTA gain as normalized value (0.0–1.0).
    #[inline]
    pub fn set_ota_gain(&mut self, gain: crate::Wave) {
        if let RootKind::Ota(o) = &mut self.root {
            o.set_gain_normalized(gain);
        }
    }

    /// Set the non-inverting input voltage (Vp) for op-amp root elements.
    ///
    /// For unity-gain buffers, the op-amp output will follow this voltage.
    /// Has no effect if the root is not an op-amp.
    #[allow(dead_code)]
    #[inline]
    pub fn set_opamp_vp(&mut self, vp: crate::Wave) {
        if let RootKind::OpAmp(op) = &mut self.root {
            op.set_vp(vp);
        }
    }

    /// Get the current non-inverting input voltage if this is an op-amp stage.
    #[allow(dead_code)]
    pub fn opamp_vp(&self) -> Option<crate::Wave> {
        match &self.root {
            RootKind::OpAmp(op) => Some(op.vp()),
            _ => None,
        }
    }

    /// Configure op-amp feedback topology.
    ///
    /// - `ratio = 1.0`: Unity-gain buffer (Vm = Vout)
    /// - `ratio < 1.0`: Gain stage with feedback network
    #[allow(dead_code)]
    #[inline]
    pub fn set_opamp_feedback(&mut self, ratio: crate::Wave, vm_external: crate::Wave) {
        if let RootKind::OpAmp(op) = &mut self.root {
            op.set_feedback(ratio, vm_external);
        }
    }

    /// Set the non-inverting input voltage for the paired op-amp buffer.
    ///
    /// Called before `process()` to set the stage input signal as the
    /// op-amp's Vp reference.  In all-pass circuits, this is the signal
    /// before the R/C/JFET network attenuates it.
    #[inline]
    pub fn set_paired_opamp_vp(&mut self, vp: crate::Wave) {
        if let Some(ref mut opamp) = self.paired_opamp {
            opamp.set_vp(vp);
        }
    }

    /// Notify that a pot in this stage changed.
    ///
    /// If this stage has a `feedback_pot_id`, reads the pot's current resistance
    /// and calls `OpAmpRoot::set_feedback_pot_r()` to recompute gain.
    /// Checks both the OpAmp root (standalone) and feedback_opamp (DiodePair paired).
    pub fn notify_pot_changed(&mut self) {
        if let Some(ref pot_id) = self.feedback_pot_id {
            // Check main tree, then 3-port children, then MNA children
            let pot_r = self
                .tree
                .get_pot_resistance(pot_id)
                .or_else(|| {
                    self.zf_child
                        .as_ref()
                        .and_then(|c| c.get_pot_resistance(pot_id))
                })
                .or_else(|| {
                    self.zg_child
                        .as_ref()
                        .and_then(|c| c.get_pot_resistance(pot_id))
                })
                .or_else(|| {
                    self.opamp_children
                        .iter()
                        .find_map(|c| c.get_pot_resistance(pot_id))
                });
            if let Some(pot_r) = pot_r {
                // Compute gain from port resistances:
                // Rf = pot_r + fixed series resistance (e.g., R_clip)
                // Gain = Rf / Ri (inverting) — set_gain takes absolute value
                let rf = pot_r + self.feedback_series_r;
                let ri = self.feedback_ri;
                if ri > 0.0 && ri < crate::Wave::MAX {
                    let gain = rf / ri;
                    if let RootKind::OpAmp(ref mut oa) = self.root {
                        oa.set_gain(gain);
                    }
                    if let Some(ref mut oa) = self.feedback_opamp {
                        oa.set_gain(gain);
                    }
                }
            }
        }
    }

    pub fn update_feedback_ri_from_pot(&mut self, comp_id: &str, position: crate::Wave) -> bool {
        if self.feedback_ri_pot_id.as_deref() != Some(comp_id) {
            return false;
        }

        let tapered = self.feedback_ri_pot_taper.apply(position);
        let pot_r = (tapered * self.feedback_ri_pot_max_r).max(1.0);
        let ri = self.feedback_ri_fixed_r + pot_r;
        if ri > 0.0 && ri.is_finite() {
            let previous_ri = self.feedback_ri;
            self.feedback_ri = ri;
            let rf = if let Some(ref pot_id) = self.feedback_pot_id {
                let rf = self
                    .tree
                    .get_pot_resistance(pot_id)
                    .or_else(|| {
                        self.zf_child
                            .as_ref()
                            .and_then(|c| c.get_pot_resistance(pot_id))
                    })
                    .or_else(|| {
                        self.zg_child
                            .as_ref()
                            .and_then(|c| c.get_pot_resistance(pot_id))
                    })
                    .or_else(|| {
                        self.opamp_children
                            .iter()
                            .find_map(|c| c.get_pot_resistance(pot_id))
                    })
                    .map(|r| r + self.feedback_series_r)
                    .unwrap_or(self.feedback_series_r);
                rf
            } else if previous_ri > 0.0 && previous_ri.is_finite() {
                match &self.root {
                    RootKind::OpAmp(oa) => oa.gain().abs() * previous_ri,
                    _ => self
                        .feedback_opamp
                        .as_ref()
                        .map(|oa| oa.gain().abs() * previous_ri)
                        .unwrap_or(0.0),
                }
            } else {
                0.0
            };
            if rf > 0.0 {
                let gain = rf / ri;
                if let RootKind::OpAmp(ref mut oa) = self.root {
                    oa.set_gain(gain);
                }
                if let Some(ref mut oa) = self.feedback_opamp {
                    oa.set_gain(gain);
                }
            }
        }

        true
    }

    /// Set a pot position in this stage, checking tree + all opamp children.
    ///
    /// Uses accumulator pattern (not early return) so split pots (__aw/__wb)
    /// that appear in multiple locations all get updated. Triggers
    /// recompute_all + notify_pot_changed when any pot is found.
    pub fn set_pot(&mut self, comp_id: &str, value: crate::Wave) -> bool {
        let mut found = false;
        let aw_id = alloc::format!("{comp_id}__aw");
        let wb_id = alloc::format!("{comp_id}__wb");
        // Main tree: use set_pot_dirty for incremental recompute.
        // Marks only the leaf-to-root path dirty instead of full recompute.
        if self.tree.set_pot_dirty(comp_id, value) {
            found = true;
        }
        if self.tree.set_pot_dirty(&aw_id, value) {
            found = true;
        }
        if self.tree.set_pot_dirty(&wb_id, 1.0 - value) {
            found = true;
        }
        if let Some(ref mut zf) = self.zf_child {
            if zf.set_pot_dirty(comp_id, value) {
                found = true;
            }
            if zf.set_pot_dirty(&aw_id, value) {
                found = true;
            }
            if zf.set_pot_dirty(&wb_id, 1.0 - value) {
                found = true;
            }
        }
        if let Some(ref mut zg) = self.zg_child {
            if zg.set_pot_dirty(comp_id, value) {
                found = true;
            }
            if zg.set_pot_dirty(&aw_id, value) {
                found = true;
            }
            if zg.set_pot_dirty(&wb_id, 1.0 - value) {
                found = true;
            }
        }
        for child in self.opamp_children.iter_mut() {
            if child.set_pot_dirty(comp_id, value) {
                found = true;
            }
            if child.set_pot_dirty(&aw_id, value) {
                found = true;
            }
            if child.set_pot_dirty(&wb_id, 1.0 - value) {
                found = true;
            }
        }
        // WDF constraint adaptor subtrees
        if let Some(ref mut adaptor) = self.opamp_wdf_adaptor {
            if adaptor.zi.set_pot_dirty(comp_id, value) {
                adaptor.zi.recompute_incremental();
                found = true;
            }
            if adaptor.zi.set_pot_dirty(&aw_id, value) {
                adaptor.zi.recompute_incremental();
                found = true;
            }
            if adaptor.zi.set_pot_dirty(&wb_id, 1.0 - value) {
                adaptor.zi.recompute_incremental();
                found = true;
            }
            if adaptor.zf.set_pot_dirty(comp_id, value) {
                adaptor.zf.recompute_incremental();
                found = true;
            }
            if adaptor.zf.set_pot_dirty(&aw_id, value) {
                adaptor.zf.recompute_incremental();
                found = true;
            }
            if adaptor.zf.set_pot_dirty(&wb_id, 1.0 - value) {
                adaptor.zf.recompute_incremental();
                found = true;
            }
        }
        // Passive RType children
        if self.set_passive_rtype_pot(comp_id, value) {
            found = true;
        }
        if found {
            self.recompute_all();
            self.notify_pot_changed();
        }
        if self.update_feedback_ri_from_pot(comp_id, value) {
            found = true;
        }
        // Convergence-summation blend pot: stored outside the WDF tree, in the
        // ConvergenceSum network. Re-solve the superposition gains (control-rate
        // only — never per sample).
        if let Some(ref mut cs) = self.convergence {
            if cs.set_pot(comp_id, value) {
                cs.recompute_gains();
                found = true;
            }
        }
        found
    }

    /// Recompute all trees including opamp children.
    /// Uses incremental recompute for all sub-trees (only dirty subtrees
    /// are recomputed, static branches are skipped).
    pub fn recompute_all(&mut self) {
        self.tree.recompute_incremental();
        if let Some(ref mut zf) = self.zf_child {
            zf.recompute_incremental();
        }
        if let Some(ref mut zg) = self.zg_child {
            zg.recompute_incremental();
        }
        for child in &mut self.opamp_children {
            child.recompute_incremental();
        }
    }

    /// Recompute the MNA opamp adaptor scattering matrix after pot changes.
    /// Pots are NOT in the MNA G matrix — they're WDF ports only.
    /// Re-derives scattering with current port resistances.
    pub fn flush_opamp_adaptor_recompute(&mut self) {
        let recompute = match &self.opamp_recompute {
            Some(r) => r,
            None => return,
        };

        // Determine whether this is the MNA path (opamp_children) or the 3-port path (zf/zg).
        let port_resistances: Vec<crate::Wave> = if !self.opamp_children.is_empty() {
            // MNA path: rebuild port resistances from children (all but last) + stored adapted R.
            let n_ports = recompute.port_pairs.len();
            let mut pr = Vec::with_capacity(n_ports);
            for (i, child) in self.opamp_children.iter().enumerate() {
                if i < n_ports - 1 {
                    pr.push(child.port_resistance());
                }
            }
            // Adapted port resistance (last port) — fixed (not a WDF child).
            pr.push(recompute.port_resistances.last().copied().unwrap_or(1000.0));
            pr
        } else if self.zf_child.is_some() && self.zg_child.is_some() {
            // 3-port path: zf and zg resistances change when reactive children update.
            let r_f = self.zf_child.as_ref().unwrap().port_resistance();
            let r_g = self.zg_child.as_ref().unwrap().port_resistance();
            // Adapted port: parallel combination of Zf and Zg.
            let r_adapted = if r_f + r_g > 0.0 {
                (r_f * r_g) / (r_f + r_g)
            } else {
                1000.0
            };
            vec![r_f, r_g, r_adapted]
        } else {
            return; // No children to rebuild from.
        };

        // Build WdfPorts and re-derive scattering
        let ports: Vec<crate::tree::WdfPort> = recompute
            .port_pairs
            .iter()
            .zip(port_resistances.iter())
            .map(|(&terminals, &r)| terminals.to_wdf_port(r))
            .collect();
        let scattering = recompute.mna.derive_scattering_matrix_general(&ports);
        if scattering.iter().all(|s| s.is_finite()) {
            self.opamp_adaptor = Some(crate::tree::RTypeAdaptor::new(
                scattering,
                &port_resistances,
            ));
        }
    }

    /// Rebuild scattering/state matrices after pot changes.
    /// Flushes both passive RType and opamp adaptor recomputes.
    pub fn flush_recompute(&mut self) {
        // Recompute binary adaptor gamma from updated children port resistances.
        // This is needed after set_pot changes a leaf's rp — the parent
        // adaptor's gamma depends on the children's impedance ratio.
        self.tree.recompute_incremental();
        self.flush_passive_rtype_recompute();
        self.flush_opamp_adaptor_recompute();
    }

    /// Update an input-path photocoupler's LED drive and recompute opamp gain.
    ///
    /// Called by the modulation system when an envelope follower drives a
    /// photocoupler that's in the input path of an inverting opamp (e.g.,
    /// Mu-Tron integrator). The photocoupler's asymmetric time constants
    /// are modeled internally, and the resulting resistance modulates the
    /// opamp's gain: gain = dc_rf / (fixed_r + photocoupler_r).
    pub fn set_input_photocoupler_led(&mut self, comp_id: &str, led_drive: crate::Wave) -> bool {
        let mut found = false;
        for pc in &mut self.input_photocouplers {
            if pc.comp_id == comp_id {
                pc.element.set_led_drive(led_drive);
                let pc_r = pc.element.port_resistance();
                let total_ri = pc.fixed_series_r + pc_r;
                let gain = pc.dc_rf / total_ri;
                if let RootKind::OpAmp(ref mut oa) = self.root {
                    oa.set_gain(gain);
                }
                found = true;
            }
        }
        found
    }

    /// Set the closed-loop gain for inverting or non-inverting op-amp stages.
    ///
    /// For runtime modulation when a potentiometer is in the feedback path
    /// (like RAT Distortion pot, TS Drive pot).
    ///
    /// - Inverting: gain = Rf/Ri (the absolute value)
    /// - Non-inverting: gain = 1 + Rf/Ri
    ///
    /// Has no effect if the root is not an op-amp gain stage.
    #[inline]
    pub fn set_opamp_gain(&mut self, gain: crate::Wave) {
        if let RootKind::OpAmp(op) = &mut self.root {
            op.set_gain(gain);
        }
    }

    /// Get the current gain if this is an op-amp gain stage.
    #[allow(dead_code)]
    pub fn opamp_gain(&self) -> Option<crate::Wave> {
        match &self.root {
            RootKind::OpAmp(op) => Some(op.gain()),
            _ => None,
        }
    }

    /// Get the feedback pot ID for this stage (if any).
    #[allow(dead_code)]
    pub fn feedback_pot_id(&self) -> Option<&str> {
        self.feedback_pot_id.as_deref()
    }

    /// Get OpAmpRoot sample rate (for verifying oversampling propagation).
    pub fn opamp_sample_rate(&self) -> Option<crate::Wave> {
        match &self.root {
            RootKind::OpAmp(op) => Some(op.sample_rate()),
            _ => None,
        }
    }

    /// Get OpAmpRoot GBW coefficient (for verifying oversampling propagation).
    pub fn opamp_gbw_coeff(&self) -> Option<crate::Wave> {
        match &self.root {
            RootKind::OpAmp(op) => Some(op.gbw_coeff()),
            _ => None,
        }
    }

    /// Returns `true` if this stage has a paired op-amp buffer.
    #[inline]
    pub fn has_paired_opamp(&self) -> bool {
        self.paired_opamp.is_some()
    }

    /// Debug dump: print stage structure with tree and root details.
    pub fn debug_dump(&self) -> String {
        let root_name = match &self.root {
            RootKind::DiodePair(_) => "DiodePair",
            RootKind::SingleDiode(_) => "SingleDiode",
            RootKind::ExplicitDiodePair(_) => "ExplicitDiodePair",
            RootKind::ExplicitSingleDiode(_) => "ExplicitSingleDiode",
            RootKind::Zener(_) => "Zener",
            RootKind::ZenerPair(_) => "ZenerPair",
            RootKind::Jfet(_) => "Jfet",
            RootKind::JfetVr(_) => "JfetVr",
            RootKind::Triode(_) => "Triode",
            RootKind::VariMu(_) => "VariMu",
            RootKind::Pentode(_) => "Pentode",
            RootKind::Mosfet(_) => "Mosfet",
            RootKind::Bjt(_) => "Bjt",
            RootKind::DiffPair(_) => "DiffPair",
            RootKind::Ota(_) => "Ota",
            RootKind::OpAmp(_) => "OpAmp",
            RootKind::Passthrough => "Passthrough",
            RootKind::ShortCircuit => "ShortCircuit",
            RootKind::VoltageSourceDriver => "VoltageSourceDriver",
            RootKind::CapacitorRoot { .. } => "CapacitorRoot",
            RootKind::InductorRoot { .. } => "InductorRoot",
            RootKind::ResistiveTermination => "ResistiveTermination",
            RootKind::PassiveRType { .. } => "PassiveRType",
        };

        let mut s = format!(
            "WdfStage(root={}, compensation={:.6}, tree_rp={:.1}Ω, nodes={}",
            root_name,
            self.compensation,
            self.tree.port_resistance(),
            self.tree.node_count()
        );
        if self.is_trigger_voice {
            s.push_str(", trigger_voice");
        }
        if self.is_feedforward {
            s.push_str(", feedforward");
        }
        if self.injection_node_id != usize::MAX {
            s.push_str(&format!(", inj={}", self.injection_node_id));
        }
        if self.output_node_id != usize::MAX {
            s.push_str(&format!(", out={}", self.output_node_id));
        }
        if self.output_bias != 0.0 {
            s.push_str(&format!(", output_bias={:.4}", self.output_bias));
        }
        if let Some(ref probe) = self.fet_source_probe {
            s.push_str(&format!(", fet_source_probe={probe}"));
        }
        s.push_str(")\n");
        s.push_str(&self.tree.debug_dump(1));

        // Print PassiveRType scattering matrix and VS injection vector.
        if let RootKind::PassiveRType {
            scattering,
            vs_injection,
            n_ports,
            children,
            output_port,
            extraction_coeffs,
            extraction_vs,
            ..
        } = &self.root
        {
            s.push_str(&format!(
                "  PassiveRType: {} ports, output_port={}\n",
                n_ports, output_port
            ));
            // Print children port resistances.
            for (i, child) in children.iter().enumerate() {
                let marker = if i == *output_port { " (probe)" } else { "" };
                s.push_str(&format!(
                    "    port[{}]: Rp={:.1}Ω{}\n",
                    i,
                    child.port_resistance(),
                    marker
                ));
            }
            // Print scattering matrix (compact).
            s.push_str(&format!("  S[{}x{}]:\n", n_ports, n_ports));
            for i in 0..*n_ports {
                s.push_str("    [");
                for j in 0..*n_ports {
                    if j > 0 {
                        s.push_str(", ");
                    }
                    s.push_str(&format!("{:+.4}", scattering[i * n_ports + j]));
                }
                s.push_str("]\n");
            }
            // Print VS injection vector.
            s.push_str("  k: [");
            for (i, k) in vs_injection.iter().enumerate() {
                if i > 0 {
                    s.push_str(", ");
                }
                s.push_str(&format!("{:+.4}", k));
            }
            s.push_str("]\n");
            s.push_str("  extract: [");
            for (i, coeff) in extraction_coeffs.iter().enumerate() {
                if i > 0 {
                    s.push_str(", ");
                }
                s.push_str(&format!("{:+.4}", coeff));
            }
            s.push_str(&format!("], vs={:+.4}\n", extraction_vs));
        }
        s
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Push-pull stage for differential tube amplifiers (e.g., Fairchild 670)
// ═══════════════════════════════════════════════════════════════════════════

/// Tube root that dispatches to either a standard Koren triode or a
/// Raffensperger variable-mu triode. Used in PushPullStage where
/// both types may appear.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum TubeRoot {
    Koren(TriodeRoot),
    VariMu(VariMuTriodeRoot),
    Pentode(PentodeRoot),
}

impl TubeRoot {
    #[inline]
    pub fn set_vgk(&mut self, vgk: crate::Wave) {
        match self {
            TubeRoot::Koren(t) => t.set_vgk(vgk),
            TubeRoot::VariMu(t) => t.set_vgk(vgk),
            TubeRoot::Pentode(p) => p.set_vg1k(vgk),
        }
    }

    #[inline]
    pub fn v_max(&self) -> crate::Wave {
        match self {
            TubeRoot::Koren(t) => t.v_max(),
            TubeRoot::VariMu(t) => t.v_max(),
            TubeRoot::Pentode(p) => p.v_max(),
        }
    }

    #[inline]
    pub fn set_v_max(&mut self, v_max: crate::Wave) {
        match self {
            TubeRoot::Koren(t) => t.set_v_max(v_max),
            TubeRoot::VariMu(t) => t.set_v_max(v_max),
            TubeRoot::Pentode(p) => p.set_v_max(v_max),
        }
    }

    #[inline]
    pub fn process(&mut self, b_tree: crate::Wave, rp: crate::Wave) -> crate::Wave {
        match self {
            TubeRoot::Koren(t) => t.process(b_tree, rp),
            TubeRoot::VariMu(t) => t.process(b_tree, rp),
            TubeRoot::Pentode(p) => p.process(b_tree, rp),
        }
    }

    #[inline]
    pub fn plate_current(&self, vpk: crate::Wave) -> crate::Wave {
        match self {
            TubeRoot::Koren(t) => t.plate_current(vpk),
            TubeRoot::VariMu(t) => t.plate_current(vpk),
            TubeRoot::Pentode(_) => 0.0, // Pentode doesn't expose plate_current the same way
        }
    }

    pub fn parallel_count(&self) -> usize {
        match self {
            TubeRoot::Koren(t) => t.parallel_count(),
            TubeRoot::VariMu(t) => t.parallel_count(),
            TubeRoot::Pentode(_) => 1,
        }
    }
}

/// A push-pull stage processes two triode halves simultaneously.
/// Push gets +Vin, pull gets -Vin. Output = push_v - pull_v.
/// Used for circuits like the Fairchild 670 where push and pull triodes
/// connect to opposite ends of a center-tapped output transformer.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PushPullStage {
    /// WDF tree for push half (plate load + cathode passives).
    pub push_tree: DynNode,
    #[cfg_attr(feature = "serde", serde(default))]
    pub push_runtime_state: RuntimeState,
    /// WDF tree for pull half (plate load + cathode passives).
    pub pull_tree: DynNode,
    #[cfg_attr(feature = "serde", serde(default))]
    pub pull_runtime_state: RuntimeState,
    /// Push tube model (Koren triode or Raffensperger variable-mu).
    pub push_root: TubeRoot,
    /// Pull tube model (Koren triode or Raffensperger variable-mu).
    pub pull_root: TubeRoot,
    /// Oversampler for push half.
    pub push_oversampler: Oversampler,
    /// Oversampler for pull half.
    pub pull_oversampler: Oversampler,
    /// Compensation factor (mu/ref_mu).
    pub compensation: crate::Wave,
    /// Output transformer turns ratio (primary:secondary).
    /// Output is scaled by 1/ratio (step-down).
    pub turns_ratio: crate::Wave,
    /// Grid bias voltage (class AB operating point).
    pub grid_bias: crate::Wave,
    /// DC blocker state: previous input sample (1-pole HPF, ~3.5Hz).
    pub dc_blocker_x1: crate::Wave,
    /// DC blocker state: previous output sample.
    pub dc_blocker_y1: crate::Wave,
    /// R-type adaptor for push half (3-port mode, when grid passives present).
    pub push_adaptor: Option<PushPullHalfAdaptor>,
    /// R-type adaptor for pull half (3-port mode).
    pub pull_adaptor: Option<PushPullHalfAdaptor>,
}

/// R-type adaptor data for one push-pull half (3-port mode).
///
/// When grid passives are present, the push-pull half uses an R-type adaptor
/// with the grid as a WDF port instead of a simple WDF tree. This allows
/// coupling caps and grid stoppers to naturally AC-couple the signal.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PushPullHalfAdaptor {
    pub adaptor: RTypeAdaptor,
    pub device: NlDeviceGroupKind,
    pub scattering: MultiNlScattering,
    pub passive_children: Vec<DynNode>,
    #[cfg_attr(feature = "serde", serde(default))]
    pub passive_child_runtime_states: Vec<RuntimeState>,
    pub nl_port_resistances: Vec<crate::Wave>,
    pub v_prev: Vec<crate::Wave>,
    pub dc_bias: Vec<crate::Wave>,
    pub output_port: usize,
    pub n_nl: usize,
    pub vs_injection: Option<Vec<crate::Wave>>,
    pub vcc_bias_all: Vec<crate::Wave>,
    pub dc_ramp: u32,
}

impl PushPullStage {
    fn ensure_tree_runtime_states(&mut self) {
        if self.push_runtime_state.is_empty()
            || self.push_runtime_state.states.len() != self.push_runtime_state.wave_cache.len()
        {
            self.push_runtime_state = self.push_tree.bind_runtime_state();
        }
        if self.pull_runtime_state.is_empty()
            || self.pull_runtime_state.states.len() != self.pull_runtime_state.wave_cache.len()
        {
            self.pull_runtime_state = self.pull_tree.bind_runtime_state();
        }
    }

    /// Process one sample through the push-pull stage.
    /// Input is applied with opposite polarity to push and pull halves.
    /// Output is the differential plate voltage divided by turns ratio.
    #[inline]
    pub fn process(&mut self, input: crate::Wave) -> crate::Wave {
        self.ensure_tree_runtime_states();
        // Check for 3-port R-type adaptor path
        if self.push_adaptor.is_some() {
            return self.process_three_port(input);
        }
        // Existing WDF path (backward compatibility)
        let comp = self.compensation;
        let bias = self.grid_bias;

        // Push: positive phase, Pull: negative phase
        let vgk_push = bias + input * comp;
        let vgk_pull = bias - input * comp;
        self.push_root.set_vgk(vgk_push);
        self.pull_root.set_vgk(vgk_pull);

        #[cfg(feature = "debug-trace")]
        let push_b = core::cell::Cell::new(0.0 as crate::Wave);
        #[cfg(feature = "debug-trace")]
        let push_a = core::cell::Cell::new(0.0 as crate::Wave);

        let push_out = self.push_oversampler.process(input, |_| {
            let vs = self.push_root.v_max();
            self.push_tree.set_voltage(vs);
            let b = self
                .push_tree
                .reflected_with_state(&mut self.push_runtime_state);
            let rp = self.push_tree.port_resistance();
            let a = self.push_root.process(b, rp);
            self.push_tree
                .set_incident_with_state(a, &mut self.push_runtime_state);
            #[cfg(feature = "debug-trace")]
            {
                push_b.set(b);
                push_a.set(a);
            }
            (a + b) / 2.0
        });

        let pull_out = self.pull_oversampler.process(-input, |_| {
            let vs = self.pull_root.v_max();
            self.pull_tree.set_voltage(vs);
            let b = self
                .pull_tree
                .reflected_with_state(&mut self.pull_runtime_state);
            let rp = self.pull_tree.port_resistance();
            let a = self.pull_root.process(b, rp);
            self.pull_tree
                .set_incident_with_state(a, &mut self.pull_runtime_state);
            (a + b) / 2.0
        });

        // Cross-coupled cathode delay: exchange unit-delay states between
        // push and pull halves. This models the bidirectional cathode bypass
        // capacitor interaction with 1-sample latency (wavechild670's E_VcathodeBias).
        let push_cath = self.push_tree.get_unit_delay_state();
        let pull_cath = self.pull_tree.get_unit_delay_state();
        self.push_tree.set_unit_delay_partner(pull_cath);
        self.pull_tree.set_unit_delay_partner(push_cath);

        // Differential output: push - pull, scaled by transformer turns ratio.
        // The CT transformer load wrapping (in build.rs) already accounts for
        // center-tap halving, so we use turns_ratio directly here.
        let diff = push_out - pull_out;
        let raw_output = diff / self.turns_ratio;

        // DC blocker: 1-pole HPF (α=0.9995, fc≈3.5Hz at 44.1kHz).
        // The push-pull stage generates DC from plate bias voltages;
        // the real output transformer provides AC coupling, so we model
        // that with a DC blocking filter.
        let x0 = if raw_output.is_finite() {
            raw_output
        } else {
            0.0
        };
        let y0 = x0 - self.dc_blocker_x1 + 0.9995 * self.dc_blocker_y1;
        self.dc_blocker_x1 = x0;
        self.dc_blocker_y1 = if y0.is_finite() { y0 } else { 0.0 };
        let output = self.dc_blocker_y1;

        #[cfg(feature = "debug-trace")]
        if input.abs() > 1e-10 {
            let n = TRACE_COUNT_PP.fetch_add(1, Ordering::Relaxed);
            if n < MAX_TRACE_PP {
                let push_rp = self.push_tree.port_resistance();
                let push_vs = self.push_root.v_max();
                let pb = push_b.get();
                let pa = push_a.get();
                let vpk = (pa + pb) / 2.0;
                std::eprintln!(
                    "[PP n={n}] in={input:.6e} comp={comp:.4} bias={bias:.2} \
                     vgk_push={vgk_push:.4} vgk_pull={vgk_pull:.4} \
                     vs={push_vs:.1} rp={push_rp:.1}"
                );
                std::eprintln!(
                    "  b={pb:.6e} a={pa:.6e} Vpk={vpk:.4} \
                     push_out={push_out:.6e} pull_out={pull_out:.6e} \
                     diff={diff:.6e} ratio={:.2} out={output:.6e}",
                    self.turns_ratio
                );
            }
        }

        flush_denormal(output)
    }

    /// Process one sample through the push-pull stage using 3-port R-type adaptors.
    ///
    /// The grid voltage is NO LONGER set externally -- it emerges from the NR solver
    /// at port 0. The coupling cap naturally blocks DC.
    fn process_three_port(&mut self, input: crate::Wave) -> crate::Wave {
        let push_out = self.push_oversampler.process(input, |sample| {
            Self::process_adaptor_half(self.push_adaptor.as_mut().unwrap(), sample)
        });

        let pull_out = self.pull_oversampler.process(-input, |sample| {
            Self::process_adaptor_half(self.pull_adaptor.as_mut().unwrap(), sample)
        });

        // Differential output scaled by transformer turns ratio
        let diff = push_out - pull_out;
        let raw_output = diff / self.turns_ratio;

        // DC blocker
        let x0 = if raw_output.is_finite() {
            raw_output
        } else {
            0.0
        };
        let y0 = x0 - self.dc_blocker_x1 + 0.9995 * self.dc_blocker_y1;
        self.dc_blocker_x1 = x0;
        self.dc_blocker_y1 = if y0.is_finite() { y0 } else { 0.0 };

        flush_denormal(self.dc_blocker_y1)
    }

    /// Process one oversampled sub-sample through a single adaptor half.
    fn process_adaptor_half(adaptor: &mut PushPullHalfAdaptor, sample: crate::Wave) -> crate::Wave {
        let n_nl = adaptor.n_nl;
        let n_passive = adaptor.passive_children.len();
        if adaptor.passive_child_runtime_states.len() != n_passive {
            adaptor.passive_child_runtime_states.clear();
            adaptor
                .passive_child_runtime_states
                .reserve(adaptor.passive_children.len());
            for child in &mut adaptor.passive_children {
                adaptor
                    .passive_child_runtime_states
                    .push(child.bind_runtime_state());
            }
        }

        // DC ramp: gradually increase VCC bias over DC_RAMP_SAMPLES to let
        // the NR solver converge to the correct operating point without diverging.
        const DC_RAMP_SAMPLES: u32 = 256;
        let dc_scale = if adaptor.dc_ramp >= DC_RAMP_SAMPLES {
            1.0
        } else {
            adaptor.dc_ramp += 1;
            adaptor.dc_ramp as crate::Wave / DC_RAMP_SAMPLES as crate::Wave
        };

        // 1. Scatter-up passive children
        let mut b_passive = Vec::with_capacity(n_passive);
        for (child, state) in adaptor
            .passive_children
            .iter_mut()
            .zip(adaptor.passive_child_runtime_states.iter_mut())
        {
            b_passive.push(child.reflected_with_state(state));
        }

        // 2. Compute known_a for each NL port
        let b_adapted = sample;
        let mut known_a = vec![0.0; n_nl];
        for (i, a_i_out) in known_a.iter_mut().enumerate() {
            let mut a_i = if let Some(ref vs_inj) = adaptor.vs_injection {
                vs_inj[i] * b_adapted
            } else {
                adaptor.scattering.s_nl_adapted[i] * b_adapted
            };
            for (&bp, &s) in b_passive[..n_passive]
                .iter()
                .zip(&adaptor.scattering.s_nl_passive[i * n_passive..i * n_passive + n_passive])
            {
                a_i += s * bp;
            }
            a_i += adaptor.dc_bias[i] * dc_scale;
            *a_i_out = a_i;
        }

        // 3. NR solve
        let group_ref: &dyn NlDeviceGroupIv = adaptor.device.as_group_iv();
        let groups: Vec<&dyn NlDeviceGroupIv> = vec![group_ref];
        let offsets = vec![0usize];

        let b_nl = multi_port_nr_solve_grouped(
            n_nl,
            &adaptor.scattering.s_nl,
            &known_a,
            &adaptor.nl_port_resistances,
            &groups,
            &offsets,
            &mut adaptor.v_prev,
            crate::elements::nonlinear::solver::NR_MAX_ITER,
            1e-6,
        );

        // 4. Build full b-vector and scatter back
        let use_vs = adaptor.vs_injection.is_some();
        let n_total = n_nl + n_passive + if use_vs { 0 } else { 1 };
        let mut b_all = Vec::with_capacity(n_total);
        b_all.extend_from_slice(&b_nl);
        b_all.extend_from_slice(&b_passive);
        if !use_vs {
            b_all.push(b_adapted);
        }

        let mut a_all = adaptor.adaptor.scatter_all(&b_all);

        // VS injection
        if let Some(ref k) = adaptor.vs_injection {
            for i in 0..a_all.len().min(k.len()) {
                a_all[i] += k[i] * b_adapted;
            }
        }

        // VCC bias
        if !adaptor.vcc_bias_all.is_empty() {
            for i in 0..a_all.len().min(adaptor.vcc_bias_all.len()) {
                a_all[i] += adaptor.vcc_bias_all[i] * dc_scale;
            }
        }

        // 5. Set incident waves on passive children
        for (k, (child, state)) in adaptor
            .passive_children
            .iter_mut()
            .zip(adaptor.passive_child_runtime_states.iter_mut())
            .enumerate()
        {
            child.set_incident_with_state(a_all[n_nl + k], state);
        }

        // 6. Output: (a + b) / 2 at plate port (port 1)
        let a_out = a_all[adaptor.output_port];
        let b_out = b_nl[adaptor.output_port];
        (a_out + b_out) / 2.0
    }

    pub fn debug_dump(&self) -> String {
        let mode = if self.push_adaptor.is_some() {
            "3-port"
        } else {
            "WDF"
        };
        format!(
            "PushPullStage(mode={}, ratio={:.1}:1, bias={:.1}V, push_par={}, pull_par={}, comp={:.4})\n  Push: rp={:.1}Ω, nodes={}\n  Pull: rp={:.1}Ω, nodes={}",
            mode,
            self.turns_ratio,
            self.grid_bias,
            self.push_root.parallel_count(),
            self.pull_root.parallel_count(),
            self.compensation,
            self.push_tree.port_resistance(),
            self.push_tree.node_count(),
            self.pull_tree.port_resistance(),
            self.pull_tree.node_count(),
        )
    }
}

impl PushPullStage {
    /// Adjust reactive element port resistances for oversampling.
    pub fn apply_oversampling_rate(&mut self, base_rate: crate::Wave) {
        let push_ratio = self.push_oversampler.ratio();
        if push_ratio > 1 {
            let effective_rate = base_rate * push_ratio as crate::Wave;
            self.push_tree.set_sample_rate(effective_rate);
            self.push_tree.recompute();
        }
        let pull_ratio = self.pull_oversampler.ratio();
        if pull_ratio > 1 {
            let effective_rate = base_rate * pull_ratio as crate::Wave;
            self.pull_tree.set_sample_rate(effective_rate);
            self.pull_tree.recompute();
        }
        // Also adjust adaptor passive children if present.
        if let Some(ref mut adaptor) = self.push_adaptor {
            let ratio = self.push_oversampler.ratio();
            if ratio > 1 {
                let effective_rate = base_rate * ratio as crate::Wave;
                for child in &mut adaptor.passive_children {
                    child.set_sample_rate(effective_rate);
                    child.recompute();
                }
            }
        }
        if let Some(ref mut adaptor) = self.pull_adaptor {
            let ratio = self.pull_oversampler.ratio();
            if ratio > 1 {
                let effective_rate = base_rate * ratio as crate::Wave;
                for child in &mut adaptor.passive_children {
                    child.set_sample_rate(effective_rate);
                    child.recompute();
                }
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Multi-NL coupled stage (R-type adaptor + multi-port NR solver)
// ═══════════════════════════════════════════════════════════════════════════

use crate::elements::nonlinear::solver::{
    multi_port_nr_solve_grouped, multi_port_nr_solve_grouped_into, multi_port_nr_solve_into,
    NlDeviceGroupIv, NlDeviceIv,
};
use crate::elements::nonlinear::{PentodeThreePort, VariMuThreePort};

/// Nonlinear device kind for the multi-NL solver.
///
/// Each variant wraps a concrete nonlinear root type that implements
/// `NlDeviceIv`, providing the I-V characteristic and its derivative.
#[allow(dead_code)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum NlDeviceKind {
    Triode(TriodeRoot),
    VariMu(VariMuTriodeRoot),
    Pentode(PentodeRoot),
    Diode(DiodeRoot),
    DiodePair(DiodePairRoot),
    /// Wright Omega explicit single diode (multi-NL context).
    ExplicitDiode(ExplicitDiodeRoot),
    /// Wright Omega explicit anti-parallel diode pair (multi-NL context).
    ExplicitDiodePair(ExplicitDiodePairRoot),
    /// JFET drain-source as a 1-port NL device (Vgs set externally).
    Jfet(JfetRoot),
    /// CA3080 OTA transconductance amplifier.
    ///
    /// Current-output device: `Iout = Iabc * tanh(Vdiff / (2*Vt))`.
    /// Gain is controlled by `Iabc` (set via modulation or fixed bias).
    Ota(OtaRoot),
}

impl NlDeviceKind {
    /// Set the control voltage for this NL device from the input signal.
    ///
    /// `bias_offset` is an additional bias voltage from external controls
    /// (e.g., BJT bias pots). Zero for non-BJT devices.
    pub fn set_control_voltage(
        &mut self,
        input: crate::Wave,
        compensation: crate::Wave,
        _bias_offset: crate::Wave,
    ) {
        match self {
            NlDeviceKind::Triode(t) => {
                t.set_vgk(t.vgk_bias() + input * compensation);
            }
            NlDeviceKind::VariMu(t) => {
                t.set_vgk(t.vgk_bias() + input * compensation);
            }
            NlDeviceKind::Pentode(p) => {
                p.set_vg1k(p.vg1k_bias() + input * compensation);
            }
            NlDeviceKind::Diode(_)
            | NlDeviceKind::DiodePair(_)
            | NlDeviceKind::ExplicitDiode(_)
            | NlDeviceKind::ExplicitDiodePair(_)
            | NlDeviceKind::Ota(_) => {}
            NlDeviceKind::Jfet(j) => {
                // Vgs driven by input signal (for pitch sweep etc.)
                j.set_vgs(input * compensation);
            }
        }
    }

    /// Get a reference to this device as an `NlDeviceIv` trait object.
    pub fn as_nl_device_iv(&self) -> &dyn NlDeviceIv {
        match self {
            NlDeviceKind::Triode(t) => t,
            NlDeviceKind::VariMu(t) => t,
            NlDeviceKind::Pentode(p) => p,
            NlDeviceKind::Diode(d) => d,
            NlDeviceKind::DiodePair(d) => d,
            NlDeviceKind::ExplicitDiode(d) => d,
            NlDeviceKind::ExplicitDiodePair(d) => d,
            NlDeviceKind::Jfet(j) => j,
            NlDeviceKind::Ota(o) => o,
        }
    }

    pub fn debug_name(&self) -> &'static str {
        match self {
            NlDeviceKind::Triode(_) => "Triode",
            NlDeviceKind::VariMu(_) => "VariMu",
            NlDeviceKind::Pentode(_) => "Pentode",
            NlDeviceKind::Diode(_) => "Diode",
            NlDeviceKind::DiodePair(_) => "DiodePair",
            NlDeviceKind::ExplicitDiode(_) => "ExplicitDiode",
            NlDeviceKind::ExplicitDiodePair(_) => "ExplicitDiodePair",
            NlDeviceKind::Jfet(_) => "Jfet",
            NlDeviceKind::Ota(_) => "Ota",
        }
    }
}

/// Implement NlDeviceGroupIv for NlDeviceKind, treating each as a 1-port group.
/// This allows SinglePort(NlDeviceKind) in NlDeviceGroupKind to delegate
/// through as_group_iv() in mixed-device collapsed stages.
impl NlDeviceGroupIv for NlDeviceKind {
    fn n_ports(&self) -> usize {
        1
    }

    fn eval(&self, v: &[crate::Wave], currents: &mut [crate::Wave], jacobian: &mut [crate::Wave]) {
        let (i, di) = self.as_nl_device_iv().iv(v[0]);
        currents[0] = i;
        jacobian[0] = di;
    }

    fn v_clamp_port(&self, _port: usize) -> (crate::Wave, crate::Wave) {
        self.as_nl_device_iv().v_clamp()
    }
}

/// Device group kind for multi-port NL devices with cross-coupled I-V.
///
/// Each variant wraps a concrete device with multiple coupled ports
/// (e.g., a 3-port triode with grid and plate ports), or a single-port
/// device adapted to the grouped interface for mixed-device solves.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum NlDeviceGroupKind {
    /// 3-port variable-mu triode (grid-cathode + plate-cathode).
    VariMuThreePort(VariMuThreePort),
    /// 3-port Koren triode (grid-cathode + plate-cathode) for MNA fallback.
    TriodeThreePort(TriodeThreePort),
    /// 3-port Koren pentode (grid-cathode + plate-cathode) for push-pull.
    PentodeThreePort(PentodeThreePort),
    /// 2-port BJT (base-emitter + collector-emitter) using Gummel-Poon.
    BjtTwoPort(BjtTwoPort),
    /// 2-port BJT using simplified Ebers-Moll model (faster alternative).
    ///
    /// Enabled when the `ebers-moll` feature is active. Omits Early effect,
    /// high-injection, leakage, and parasitic resistances for reduced per-iteration
    /// cost (~3× fewer exp() calls vs Gummel-Poon).
    EbersMollTwoPort(EbersMollTwoPort),
    /// Single-port NL device adapted as a 1-port device group.
    /// Used for mixed-device collapsed stages (e.g., sidechain with
    /// triodes + pentodes + diodes in one MultiNlStage).
    SinglePort(NlDeviceKind),
}

impl NlDeviceGroupKind {
    pub fn as_group_iv(&self) -> &dyn NlDeviceGroupIv {
        match self {
            NlDeviceGroupKind::VariMuThreePort(t) => t,
            NlDeviceGroupKind::TriodeThreePort(t) => t,
            NlDeviceGroupKind::PentodeThreePort(p) => p,
            NlDeviceGroupKind::BjtTwoPort(b) => b,
            NlDeviceGroupKind::EbersMollTwoPort(e) => e,
            NlDeviceGroupKind::SinglePort(d) => d,
        }
    }

    pub fn debug_name(&self) -> &'static str {
        match self {
            NlDeviceGroupKind::VariMuThreePort(_) => "VariMuThreePort",
            NlDeviceGroupKind::TriodeThreePort(_) => "TriodeThreePort",
            NlDeviceGroupKind::PentodeThreePort(_) => "PentodeThreePort",
            NlDeviceGroupKind::BjtTwoPort(b) => {
                if b.is_pnp {
                    "BjtPnp2P"
                } else {
                    "BjtNpn2P"
                }
            }
            NlDeviceGroupKind::EbersMollTwoPort(e) => {
                if e.is_pnp() {
                    "EmPnp2P"
                } else {
                    "EmNpn2P"
                }
            }
            NlDeviceGroupKind::SinglePort(d) => d.debug_name(),
        }
    }

    pub fn n_ports(&self) -> usize {
        match self {
            NlDeviceGroupKind::VariMuThreePort(_) => 2,
            NlDeviceGroupKind::TriodeThreePort(_) => 2,
            NlDeviceGroupKind::PentodeThreePort(_) => 2,
            NlDeviceGroupKind::BjtTwoPort(_) => 2,
            NlDeviceGroupKind::EbersMollTwoPort(_) => 2,
            NlDeviceGroupKind::SinglePort(_) => 1,
        }
    }
}

/// Grouped device configuration for MultiNlStage.
///
/// When present, the multi-port NR solver uses cross-coupled device Jacobians
/// instead of treating each port independently.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MultiNlDeviceGroups {
    pub groups: Vec<NlDeviceGroupKind>,
    pub offsets: Vec<usize>,
}

/// Data needed to recompute the scattering matrix when pot values change.
///
/// The MNA conductance matrix stores only fixed resistor stamps (immutable).
/// Port node pairs are fixed; only port resistances change (from pots).
/// On pot change, we rebuild `WdfPort`s with current resistances, re-derive
/// the scattering matrix, and update all sub-blocks.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ScatteringRecomputeData {
    /// MNA system with fixed resistors stamped (no pots).
    pub mna: MnaSystem,
    /// Port node pairs: (pos_mna_idx, neg_mna_idx) for each port.
    /// Ordering: [NL_0..NL_{n-1}, passive_0..passive_{m-1}] (VS injection mode)
    /// Or: [NL_0..NL_{n-1}, passive_0..passive_{m-1}, adapted] (standard mode).
    pub port_node_pairs: Vec<WdfPortTerminals>,
    /// Resistance of the adapted (voltage source) port.
    pub adapted_resistance: crate::Wave,
    /// When Some, the input VS is stamped as an ideal voltage source in MNA B/C
    /// matrices. The value is the MNA VS branch index. Recompute uses
    /// `derive_scattering_and_vs_injection()` instead of `derive_scattering_matrix_general()`.
    pub vs_source_index: Option<usize>,
    /// VCC voltage source index in MNA. When Some, VCC is an ideal VS and
    /// recompute re-extracts dc_bias from the VCC injection vector.
    pub vcc_vs_index: Option<usize>,
    /// Output MNA node pair for direct node-voltage extraction.
    /// When Some, recompute also updates the extraction coefficients.
    pub extract_output_nodes: Option<WdfPortTerminals>,
}

/// Scattering matrix sub-blocks for multi-NL solving.
///
/// These sub-blocks are extracted from the full R-type adaptor scattering
/// matrix and are the only parts needed during per-sample NR solving.
/// They are recomputed together when pot values change.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MultiNlScattering {
    /// NL-to-NL sub-block (n_nl × n_nl, row-major).
    pub s_nl: Vec<crate::Wave>,
    /// NL-to-passive sub-block (n_nl × n_passive, row-major).
    pub s_nl_passive: Vec<crate::Wave>,
    /// NL-to-adapted column (n_nl).
    pub s_nl_adapted: Vec<crate::Wave>,
}

impl MultiNlScattering {
    /// Extract sub-blocks from a full scattering matrix.
    ///
    /// The full matrix is `n_total × n_total` (row-major) with port ordering:
    /// `[NL_0..NL_{n-1}, passive_0..passive_{m-1}, (vcc?), adapted]`.
    /// `n_total` is inferred from the scattering matrix size, so extra ports
    /// (like VCC) are handled automatically — only the first `n_nl` rows and
    /// the NL, passive, and last (adapted) columns are extracted.
    pub fn from_full_matrix(scattering: &[crate::Wave], n_nl: usize, n_passive: usize) -> Self {
        let n_total = if scattering.is_empty() {
            n_nl + n_passive + 1
        } else {
            let len = scattering.len();
            let nt = crate::math::sqrt(len as crate::Wave) as usize;
            debug_assert_eq!(nt * nt, len, "scattering matrix must be square");
            nt
        };
        let mut s_nl = vec![0.0; n_nl * n_nl];
        for i in 0..n_nl {
            for j in 0..n_nl {
                s_nl[i * n_nl + j] = scattering[i * n_total + j];
            }
        }
        let mut s_nl_passive = vec![0.0; n_nl * n_passive];
        for i in 0..n_nl {
            for k in 0..n_passive {
                s_nl_passive[i * n_passive + k] = scattering[i * n_total + (n_nl + k)];
            }
        }
        let mut s_nl_adapted = vec![0.0; n_nl];
        for i in 0..n_nl {
            s_nl_adapted[i] = scattering[i * n_total + (n_total - 1)];
        }
        Self {
            s_nl,
            s_nl_passive,
            s_nl_adapted,
        }
    }
}

/// Coupled nonlinear stage using an R-type adaptor and multi-port NR solver.
///
/// This uses a physically correct formulation:
/// the full passive network between coupled NL elements is captured as an
/// R-type adaptor scattering matrix, and all NL ports are solved simultaneously
/// via a multi-port Newton-Raphson solver.
///
/// This correctly models circuits like:
/// - Fuzz Face (2 PNP BJTs with collector-base feedback)
/// - Darlington pairs
/// - Long-tailed pairs
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MultiNlStage {
    /// R-type adaptor containing the full scattering matrix.
    pub adaptor: RTypeAdaptor,
    /// Nonlinear devices at the NL ports.
    pub nl_devices: Vec<NlDeviceKind>,
    /// Port resistances for the NL ports.
    pub nl_port_resistances: Vec<crate::Wave>,
    /// Reactive passive one-ports needing WDF state updates.
    pub passive_one_ports: Vec<RuntimeOnePort<MnaNodeId>>,
    /// Shared runtime state for `passive_one_ports`.
    #[cfg_attr(feature = "serde", serde(default))]
    pub passive_runtime_state: RuntimeState,
    /// Effective sample rate used to derive reactive port resistances.
    pub passive_sample_rate: crate::Wave,
    /// Pot DynNodes stored separately — pots are G-matrix conductances, not WDF ports.
    pub pot_children: Vec<DynNode>,
    /// Runtime variable-resistor bindings for MNA G-matrix updates.
    pub variable_resistors: Vec<MnaVariableResistorBinding>,
    /// Number of nonlinear ports.
    pub n_nl: usize,
    /// Warm-start voltages for NR solver.
    pub v_prev: Vec<crate::Wave>,
    /// Scattering matrix sub-blocks for NR solving.
    pub scattering: MultiNlScattering,
    /// Oversampler for antialiasing.
    pub oversampler: Oversampler,
    /// Passive attenuation compensation factor.
    pub compensation: crate::Wave,
    /// Which NL port to tap for output.
    pub output_port: usize,
    /// Optional device groups for cross-coupled NR solve (3-port triodes).
    /// When Some, uses `multi_port_nr_solve_grouped()` instead of `multi_port_nr_solve()`.
    pub device_groups: Option<MultiNlDeviceGroups>,
    /// Data for recomputing scattering matrix when pots change.
    /// None if the stage has no pots (no recomputation needed).
    pub recompute_data: Option<ScatteringRecomputeData>,
    /// BFS distance from input of the injection node (for topological ordering).
    pub signal_flow_distance: usize,
    /// Component names in this stage (e.g. "R_in,Cin"). Debug builds only.
    #[cfg(debug_assertions)]
    pub debug_label: String,
    /// When true, this stage is not on the audio signal path (e.g. bias
    /// network). It still processes and meters, but its output does NOT
    /// overwrite the serial audio chain signal.
    pub bypass_serial: bool,
    /// Inter-stage voltage gain from a transformer boundary.
    pub transformer_gain: crate::Wave,
    /// Circuit graph node ID (for debug routing).
    pub injection_node_id: usize,
    /// Circuit graph node ID (for debug routing).
    pub output_node_id: usize,
    /// Flag: pot changed since last scattering recompute.
    pub recompute_pending: bool,
    /// VEB bias offset from a feedback pot.
    pub veb_bias_offset: crate::Wave,
    /// Feedback scale for coupled BJT stages.
    pub feedback_scale: crate::Wave,
    /// Feedback opamp root for diode-paired stages (Bluesbreaker, Tube Screamer).
    /// Applies opamp gain + GBW + slew to the input before MNA/NR solve.
    pub feedback_opamp: Option<OpAmpRoot>,
    /// Pot ID that controls feedback opamp gain (if any).
    pub feedback_pot_id: Option<String>,
    /// Linearized OTA data for gm-based scattering recompute.
    /// When Some, the OTA's transconductance is stamped into the MNA as a linear
    /// conductance. When the envelope changes gain, we delta-update the MNA and
    /// recompute the scattering matrix. No NR iteration needed.
    pub linearized_ota: Option<LinearizedOtaData>,
    /// VS injection vector for ideal voltage source input.
    /// When Some, signal is injected via `a[i] += k[i] * V_in` instead of
    /// an adapted WDF port. Used for linearized OTA stages where the adapted
    /// port would share an MNA node with a reactive port.
    pub vs_injection: Option<Vec<crate::Wave>>,
    /// Node-voltage extraction coefficients for direct output reading.
    /// When Some, the output is computed as:
    ///   V_out = Σ_k extract_coeffs[k] * b[k] + extract_vs * V_in
    /// This bypasses WDF port impedance mismatch by reading the MNA node
    /// voltage directly from X⁻¹ coefficients.
    pub extract_coeffs: Option<Vec<crate::Wave>>,
    pub extract_vs: crate::Wave,
    /// When set, identifies a pot in pot_children whose resistance drives
    /// BJT bias recalculation (feedback_scale + veb_bias_offset).
    pub bias_pot_id: Option<String>,
    /// Emitter resistance for bias pot computation (default 470Ω).
    pub bias_emitter_r: crate::Wave,
    /// State-space model for direct discrete-time simulation.
    /// When Some, process() uses state-space update (A·x + b·u) instead of
    /// WDF scattering. Used for linearized OTA stages where cap port
    /// conductances overwhelm circuit conductances.
    pub state_space: Option<StateSpaceData>,
    /// IIR filter compiled from linear R-node MNA. Takes priority over
    /// state_space when present — simpler, faster, correct Q for oscillators.
    pub iir: Option<IirData>,
    /// Precomputed interpolation table for single-pot stages.
    /// When Some, pot changes use table lookup instead of MNA re-inversion.
    pub interp_table: Option<ScatteringInterpolationTable>,
    /// Precomputed DC bias from VCC supply injection vector (NL ports only).
    /// dc_bias[i] = vcc_injection[i] * supply_voltage, for i in 0..n_nl.
    /// Added to known_a[i] in the NR solver to establish transistor operating points.
    pub dc_bias: Vec<crate::Wave>,
    /// Full VCC injection vector × supply_voltage for ALL ports.
    /// Added to a_all after scatter_all to provide DC bias to passive children
    /// and correct output extraction. Length = n_nl + n_passive + adapted(0 or 1).
    pub vcc_bias_all: Vec<crate::Wave>,
    /// VCC voltage source index in the MNA (for recomputing dc_bias on pot changes).
    /// When Some, VCC is stamped as an ideal VS in the MNA with zero impedance.
    pub vcc_vs_index: Option<usize>,
    /// Nominal supply voltage (volts). Used for dc_bias computation.
    pub supply_voltage: crate::Wave,
    /// DC ramp counter for gradual bias injection.
    /// Counts from 0 to DC_RAMP_SAMPLES, scaling dc_bias by ramp/N to let the
    /// NR solver track the operating point as supply voltage gradually increases.
    pub dc_ramp: u32,
    /// Value to restore dc_ramp to on reset().
    ///
    /// Default is 0 (full ramp from zero on each reset). When init hints are
    /// present for a stage (e.g. `init { Q1: saturated, Q2: cutoff }` in the
    /// .pedal DSL), this is set to `DC_RAMP_SAMPLES` (256) so that reset()
    /// restores dc_scale = 1.0 immediately. This preserves the hint-seeded
    /// v_prev as a meaningful NR warm-start: with a near-zero excitation from
    /// dc_scale ≈ 0, the NR solver converges to ≈0 regardless of the warm-start,
    /// erasing the asymmetric seed. Skipping the ramp ensures the first sample
    /// sees the full DC bias and the hinted Vce difference matters.
    #[cfg_attr(feature = "serde", serde(default))]
    pub initial_dc_ramp: u32,
    /// Physics-based initial v_prev values. Restored on reset() instead of zeroing,
    /// so the NR solver starts near the correct operating point after a DAW reset.
    pub initial_v_prev: Vec<crate::Wave>,
    /// Previous-previous sample's NR solution (v[n-2]).
    /// Used with v_prev (v[n-1]) to extrapolate a warm-start guess for v[n]:
    ///   v_guess = 2·v[n-1] − v[n-2]
    /// Reduces NR iterations on transients compared to a plain v_prev warm-start.
    pub v_prev_2: Vec<crate::Wave>,
    /// DC blocker state: previous input sample (x[n-1]).
    pub dc_blocker_x1: crate::Wave,
    /// DC blocker state: previous output sample (y[n-1]).
    pub dc_blocker_y1: crate::Wave,
    /// Pre-allocated workspace for NR solver (eliminates per-sample heap allocations).
    pub nr_workspace: crate::elements::nonlinear::solver::NrWorkspace,
    /// Pre-allocated process buffers.
    pub work_b_passive: Vec<crate::Wave>,
    pub work_known_a: Vec<crate::Wave>,
    pub work_b_all: Vec<crate::Wave>,
    pub work_a_all: Vec<crate::Wave>,
    /// Adaptive oversampling: when true, skip NR on odd sub-samples (X2 NR rate).
    /// Set based on previous base sample's frozen Newton success rate.
    pub adaptive_x2: bool,
    /// Sub-sample counter within oversampler loop.
    pub subsample_counter: u8,
    /// Remaining NR iteration budget for the current base sample.
    ///
    /// Reset to `NR_ITERATION_BUDGET` at the start of each base sample's
    /// oversampler loop. Each sub-sample's solve decrements it by `iters_used`.
    /// When it reaches zero, subsequent sub-samples skip the NR solve and reuse
    /// the most recent `b_nl` values from the workspace — still doing scattering
    /// and WDF port updates correctly.
    pub iteration_budget_remaining: usize,
    /// Previous base-sample input value for transient detection.
    ///
    /// Used to compute `input_delta = input - prev_input` each sample.
    /// A large delta indicates a transient, allowing the NR tolerance to be
    /// loosened via `adaptive_nr_tolerance()` to reduce iteration count.
    pub prev_input: crate::Wave,
    /// Optional **nonlinear DC operating-point** per NL port (`v* = V(pos) − V(neg)`),
    /// solved by the compiler's grouped-BJT DC Q-point pass (ko5g g725.2).
    ///
    /// When present, the runtime derives `dc_bias[i]` from this target via the
    /// WDF residual inversion
    /// `dc_bias[i] = v_i + R_i·i_i(v) − Σ_j S[i][j]·(v_j − R_j·i_j(v))`
    /// instead of the linear VCC-injection superposition — both at build time and
    /// (critically) after every `recompute_scattering` (e.g. a feedback-pot move),
    /// which would otherwise re-extract the linear `dc_bias` and erase the seed.
    /// `None` (the default) leaves the legacy VCC-injection path untouched, so
    /// triode/non-BJT/feedforward stages are byte-identical.
    #[cfg_attr(feature = "serde", serde(default))]
    pub dc_qpoint_v: Option<Vec<crate::Wave>>,
    /// DC steady-state reflected wave of each passive port (`b_passive[k]`), used
    /// alongside `dc_qpoint_v` to subtract the passive coupling contribution from
    /// the target incident wave: at DC the caps settle to their node voltages, so
    /// `known_a_i` includes `Σ_k s_nl_passive[i][k]·b_passive_DC[k]` which must be
    /// removed from `dc_bias[i]`.  Same length as the passive-port count; empty
    /// when no seed is present.
    #[cfg_attr(feature = "serde", serde(default))]
    pub dc_qpoint_passive_b: Vec<crate::Wave>,
    /// When true (bead b2ps generic joint path), `dc_bias` has ALREADY been
    /// inverted to a consistent joint NL+reactive DC fixed point by
    /// `solve_joint_dc_qpoint`; `reset()` must re-charge the caps and restore the
    /// warm-start but must NOT call `apply_dc_qpoint_seed` (which would re-derive a
    /// DIFFERENT `dc_bias` and clobber the consistent one + corrupt `vcc_bias_all`).
    #[cfg_attr(feature = "serde", serde(default))]
    pub dc_qpoint_joint: bool,
}

/// State-space data for direct discrete-time simulation.
///
/// Replaces WDF scattering for linearized OTA stages where cap port
/// conductances dominate circuit conductances, causing identity S-matrix rows.
/// Uses bilinear transform on the continuous-time MNA to get node-voltage
/// dynamics directly, avoiding WDF port-impedance scaling issues.
/// IIR filter compiled from a linear R-node MNA.
/// For oscillators: biquad with f0/Q derived from component values.
/// Recomputation on pot change is O(1) — just recalculate f0, Q, gain
/// from stored component values and update the 5 biquad coefficients.
/// No matrix inversion needed. Cortex-M7 safe.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct IirData {
    /// Numerator coefficients [b0, b1, b2].
    pub b_coeffs: Vec<crate::Wave>,
    /// Denominator coefficients [1.0, a1, a2].
    pub a_coeffs: Vec<crate::Wave>,
    /// Input history [x[n-1], x[n-2]].
    pub x_hist: Vec<crate::Wave>,
    /// Output history [y[n-1], y[n-2]].
    pub y_hist: Vec<crate::Wave>,
    /// Sample rate for coefficient recomputation.
    pub sample_rate: crate::Wave,
    /// Stored component values for O(1) recomputation.
    /// R_series: product of series resistors (R1×R2).
    pub r_series_product: crate::Wave,
    /// C_shunt: product of shunt caps (C1×C2).
    pub c_shunt_product: crate::Wave,
    /// Feedback resistor value (current, changes with pot).
    pub r_fb: crate::Wave,
    /// Critical resistance: R1 + R2 + R1*C1/C2.
    /// Recomputed when series R changes (tuning pot).
    pub r_crit: crate::Wave,
    /// Pot-to-component mapping for recomputation.
    /// Each entry: (pot_child_index, affects_r_series, affects_r_fb)
    pub pot_map: Vec<(usize, bool, bool)>,
    /// Base R values for series resistors (before pot contribution).
    pub r_series_base: [crate::Wave; 2],
    /// Base C values for shunt caps.
    pub c_shunt_base: [crate::Wave; 2],
}

impl IirData {
    pub fn new(
        b_coeffs: Vec<crate::Wave>,
        a_coeffs: Vec<crate::Wave>,
        sample_rate: crate::Wave,
    ) -> Self {
        let order = a_coeffs.len() - 1;
        Self {
            b_coeffs,
            a_coeffs,
            x_hist: vec![0.0; order],
            y_hist: vec![0.0; order],
            sample_rate,
            r_series_product: 0.0,
            c_shunt_product: 0.0,
            r_fb: 0.0,
            r_crit: 0.0,
            pot_map: Vec::new(),
            r_series_base: [0.0; 2],
            c_shunt_base: [0.0; 2],
        }
    }

    /// Recompute biquad coefficients from current R/C values.
    /// O(1): sqrt + sin + cos + a few multiplies. Cortex-M7 safe.
    pub fn recompute(&mut self) {
        use crate::math::PI;
        if self.r_series_product <= 0.0 || self.c_shunt_product <= 0.0 || self.r_fb <= 0.0 {
            return;
        }

        let f0 = 1.0
            / (2.0
                * PI
                * crate::math::sqrt((self.r_series_product * self.c_shunt_product) as crate::Wave)
                    as crate::Wave);
        let q = if self.r_fb > self.r_crit * 1.01 {
            self.r_fb / (self.r_fb - self.r_crit)
        } else {
            100.0
        };
        let gain =
            self.r_fb / (crate::math::sqrt(self.r_series_product as crate::Wave) as crate::Wave); // Rf / sqrt(R1*R2)

        let w0 = 2.0 * PI * f0 / self.sample_rate;
        let sin_w0 = crate::math::sin(w0 as crate::Wave) as crate::Wave;
        let cos_w0 = crate::math::cos(w0 as crate::Wave) as crate::Wave;
        let alpha = sin_w0 / (2.0 * q);

        let a0 = 1.0 + alpha;
        self.b_coeffs[0] = alpha * gain / a0;
        self.b_coeffs[1] = 0.0;
        self.b_coeffs[2] = -alpha * gain / a0;
        self.a_coeffs[1] = -2.0 * cos_w0 / a0;
        self.a_coeffs[2] = (1.0 - alpha) / a0;
    }

    /// DC gain: H(z=1) = sum(b) / sum(a).
    pub fn dc_gain(&self) -> crate::Wave {
        let num: crate::Wave = self.b_coeffs.iter().sum();
        let den: crate::Wave = self.a_coeffs.iter().sum();
        if den.abs() < 1e-30 {
            1.0
        } else {
            num / den
        }
    }

    /// Process one sample through the IIR (Direct Form I).
    #[inline]
    pub fn process(&mut self, input: crate::Wave) -> crate::Wave {
        let order = self.x_hist.len();
        let mut y = self.b_coeffs[0] * input;
        for k in 0..order {
            y += self.b_coeffs.get(k + 1).copied().unwrap_or(0.0) * self.x_hist[k];
            y -= self.a_coeffs[k + 1] * self.y_hist[k];
        }
        for k in (1..order).rev() {
            self.x_hist[k] = self.x_hist[k - 1];
            self.y_hist[k] = self.y_hist[k - 1];
        }
        if order > 0 {
            self.x_hist[0] = input;
            self.y_hist[0] = y;
        }
        y
    }

    /// Process one sample with the op-amp output non-ideality folded **inside**
    /// the feedback recurrence (Direct Form I).
    ///
    /// For a single-op-amp active filter (MFB / Sallen-Key) the op-amp output
    /// node *is* the biquad output, and the reactive feedback elements charge
    /// from that physical node. So the slew-rate clamp and the supply-rail
    /// soft-clip act on the value that is then fed back through `a_coeffs`,
    /// not on a separate post-filter copy. This routine:
    ///
    /// 1. computes the *demanded* (ideal) biquad output `y` from the recurrence,
    /// 2. applies the op-amp output non-ideality via the shared
    ///    [`NonIdealFxState`] (dV/dt slew clamp + tanh rail), and
    /// 3. commits the **limited** value into `y_hist[0]` — so next sample's
    ///    denominator feedback senses the genuinely slew/rail-limited node.
    ///
    /// GBW is deliberately **not** folded into the loop here (the op-amp's own
    /// finite-bandwidth pole is left to the post path) — see `IirStage::process`
    /// for the rationale. When the signal is small/slow (slew never engages, no
    /// rail), the limited value equals `y` exactly, so `y_hist[0]` receives the
    /// ideal value and the linear filter response is bit-identical to `process`.
    #[inline]
    pub fn process_inloop(&mut self, input: crate::Wave, fx: &mut NonIdealFxState) -> crate::Wave {
        let order = self.x_hist.len();
        let mut y = self.b_coeffs[0] * input;
        for k in 0..order {
            y += self.b_coeffs.get(k + 1).copied().unwrap_or(0.0) * self.x_hist[k];
            y -= self.a_coeffs[k + 1] * self.y_hist[k];
        }
        // Op-amp output node non-ideality, in-loop: slew clamp then rail clip.
        // `prev_out` carries the real limited node, so the dV/dt clamp tracks the
        // physical output, and the limited value is what gets fed back.
        let slewed = fx.slew_step(y);
        let limited = flush_denormal(fx.rail_step(slewed));
        for k in (1..order).rev() {
            self.x_hist[k] = self.x_hist[k - 1];
            self.y_hist[k] = self.y_hist[k - 1];
        }
        if order > 0 {
            self.x_hist[0] = input;
            self.y_hist[0] = limited;
        }
        limited
    }
}

/// Standalone IIR biquad stage compiled from a linear rigid MNA.
///
/// Clean, minimal: just coefficients + state + process(). O(1)/sample.
/// No WDF tree, no scattering matrix, no NR solver.
///
/// When the stage contains an op-amp, component-declared `NonIdealFx`
/// are applied as post-processing: GBW rolloff → slew limiting → rail clamp.
/// Pot binding info stored in IirStage for runtime coefficient recomputation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum IirPotRole {
    Generic,
    Feedback,
    GroundLeg,
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct IirPotBinding {
    /// Component ID of the pot (matches ControlBinding::component_id).
    pub comp_id: String,
    /// Maximum resistance of the pot.
    pub max_r: crate::Wave,
    /// Fixed resistance in series with the pot (e.g., R_min before Drive pot).
    pub fixed_series_r: crate::Wave,
    /// Input resistance Ri (for gain = Rf/Ri calculation).
    pub ri: crate::Wave,
    /// Current pot position (0.0–1.0).
    pub position: crate::Wave,
    /// Circuit role of this pot inside the IIR group.
    pub role: IirPotRole,
    /// Feedback resistance used by ground-leg gain recomputation.
    pub feedback_r: crate::Wave,
    /// True when the op-amp topology is non-inverting.
    pub non_inverting: bool,
}

/// How an IIR stage's direct-form state relates to physical reactive one-ports.
pub type IirStateMap = LtiStateMap<MnaNodeId>;

/// Precomputed biquad coefficient lookup table for pot-controlled IIR stages.
///
/// Built at compile time by sweeping pot positions over an N-dimensional grid.
/// At runtime, `set_pot` quantizes the position and bilinearly interpolates
/// the 5 biquad coefficients from the nearest grid neighbors. O(1) lookup,
/// no matrix math, no alloc. Cortex-M7 safe.
///
/// Dimensions correspond to independent control labels (ganged pots = 1 dim).
/// Table size: `steps^n_dims × 5` crate::Wave entries.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BiquadTable {
    /// Number of uniformly-spaced steps per dimension (e.g. 64).
    pub steps: usize,
    /// Dimension labels — each is a control label (e.g. "Cutoff", "Resonance").
    /// Order matches the index arithmetic: dim 0 is the innermost.
    pub dim_labels: alloc::vec::Vec<alloc::string::String>,
    /// Flattened [b0, b1, b2, a1, a2] × (steps^n_dims) entries.
    /// Entry index for (d0, d1, ...): d0 + d1*steps + d2*steps² + ...
    pub coeffs: alloc::vec::Vec<crate::Wave>,
}

impl BiquadTable {
    /// Number of coefficients per entry (b0, b1, b2, a1, a2).
    const COEFF_COUNT: usize = 5;

    /// Look up and interpolate biquad coefficients for given positions.
    /// `positions` maps dim index → normalized position [0.0, 1.0].
    /// Writes 5 values into `out`: [b0, b1, b2, a1, a2].
    pub fn lookup(&self, positions: &[crate::Wave], out: &mut [crate::Wave; 5]) {
        let n_dims = self.dim_labels.len();
        if n_dims == 0 || self.steps < 2 {
            return;
        }
        let max_idx = self.steps - 1;

        match n_dims {
            1 => {
                // Linear interpolation
                let p = positions[0].clamp(0.0, 1.0) * max_idx as crate::Wave;
                let i0 = (p as usize).min(max_idx - 1);
                let frac = p - i0 as crate::Wave;
                let base0 = i0 * Self::COEFF_COUNT;
                let base1 = (i0 + 1) * Self::COEFF_COUNT;
                for (o, (&c0, &c1)) in out.iter_mut().zip(
                    self.coeffs[base0..base0 + 5]
                        .iter()
                        .zip(&self.coeffs[base1..base1 + 5]),
                ) {
                    *o = c0 * (1.0 - frac) + c1 * frac;
                }
            }
            2 => {
                // Bilinear interpolation
                let p0 = positions[0].clamp(0.0, 1.0) * max_idx as crate::Wave;
                let p1 = positions[1].clamp(0.0, 1.0) * max_idx as crate::Wave;
                let i0 = (p0 as usize).min(max_idx - 1);
                let i1 = (p1 as usize).min(max_idx - 1);
                let f0 = p0 - i0 as crate::Wave;
                let f1 = p1 - i1 as crate::Wave;
                let s = self.steps;
                let idx00 = (i0 + i1 * s) * Self::COEFF_COUNT;
                let idx10 = (i0 + 1 + i1 * s) * Self::COEFF_COUNT;
                let idx01 = (i0 + (i1 + 1) * s) * Self::COEFF_COUNT;
                let idx11 = (i0 + 1 + (i1 + 1) * s) * Self::COEFF_COUNT;
                let w00 = (1.0 - f0) * (1.0 - f1);
                let w10 = f0 * (1.0 - f1);
                let w01 = (1.0 - f0) * f1;
                let w11 = f0 * f1;
                for (o, ((&c00, &c10), (&c01, &c11))) in out.iter_mut().zip(
                    self.coeffs[idx00..idx00 + 5]
                        .iter()
                        .zip(&self.coeffs[idx10..idx10 + 5])
                        .zip(
                            self.coeffs[idx01..idx01 + 5]
                                .iter()
                                .zip(&self.coeffs[idx11..idx11 + 5]),
                        ),
                ) {
                    *o = c00 * w00 + c10 * w10 + c01 * w01 + c11 * w11;
                }
            }
            _ => {
                // Fallback: nearest-neighbor for 3+ dims
                let mut flat_idx = 0usize;
                let mut stride = 1usize;
                for &pos in &positions[..n_dims] {
                    let p = pos.clamp(0.0, 1.0) * max_idx as crate::Wave;
                    let i = (p as usize).min(max_idx);
                    flat_idx += i * stride;
                    stride *= self.steps;
                }
                let base = flat_idx * Self::COEFF_COUNT;
                out.copy_from_slice(&self.coeffs[base..base + 5]);
            }
        }
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct IirStage {
    /// The biquad filter data (coefficients + history).
    pub iir: IirData,
    /// Passive attenuation compensation factor.
    pub compensation: crate::Wave,
    /// BFS distance from input (for topological ordering).
    pub signal_flow_distance: usize,
    /// Component names in this stage (e.g. "R_in,Cin"). Debug builds only.
    #[cfg(debug_assertions)]
    pub debug_label: String,
    /// When true, this stage is not on the audio signal path (e.g. bias
    /// network). It still processes and meters, but its output does NOT
    /// overwrite the serial audio chain signal.
    pub bypass_serial: bool,
    /// Component-declared non-idealities applied after IIR computation.
    pub nonideal_fx: Vec<crate::nonideal_fx::NonIdealFx>,
    /// Pot bindings for runtime coefficient recomputation.
    pub pot_bindings: Vec<IirPotBinding>,
    /// Canonical physical reactive state slots for this IIR's source network.
    pub runtime_state: RuntimeState,
    /// Explicit mapping from physical one-port state to transformed IIR state.
    pub state_map: IirStateMap,
    /// Declarative graph input binding for shared stage routing.
    #[cfg_attr(feature = "serde", serde(default))]
    pub input_binding: Option<PortBinding>,
    /// Declarative graph output binding for shared stage routing.
    #[cfg_attr(feature = "serde", serde(default))]
    pub output_binding: Option<PortBinding>,
    /// Precomputed biquad lookup table (compile-time sweeps over pot positions).
    /// When present, `set_pot` interpolates from this instead of the DC gain formula.
    pub biquad_table: Option<BiquadTable>,
    /// Sample rate (needed for GBW recomputation on gain change).
    pub sample_rate: crate::Wave,
    // ── NonIdealFx runtime state ──
    /// Shared GBW/slew/rail post-processing state — same single-source-of-truth
    /// struct used by `BlackFeedbackStage` and the post-FX path.
    fx_state: NonIdealFxState,
    /// Stored GBW from OpAmpBandwidth (for recomputation when gain changes).
    stored_gbw: crate::Wave,
}

impl IirStage {
    pub fn new(iir: IirData) -> Self {
        let sample_rate = iir.sample_rate;
        let transformed_state_count = iir.a_coeffs.len().saturating_sub(1);
        Self {
            iir,
            compensation: 1.0,
            signal_flow_distance: 0,
            #[cfg(debug_assertions)]
            debug_label: String::new(),
            bypass_serial: false,
            nonideal_fx: Vec::new(),
            pot_bindings: Vec::new(),
            runtime_state: RuntimeState::new(),
            state_map: IirStateMap::empty(transformed_state_count),
            input_binding: None,
            output_binding: None,
            biquad_table: None,
            sample_rate,
            fx_state: NonIdealFxState::default(),
            stored_gbw: 0.0,
        }
    }

    pub fn bind_physical_one_ports(&mut self, reactive_one_ports: &[MnaOnePort]) {
        let mut runtime_state = RuntimeState::new();
        let mut physical_one_ports = Vec::new();

        for one_port in reactive_one_ports {
            let state_slot = runtime_state.allocate_one_port(one_port.kind);
            physical_one_ports.push(RuntimeOnePort::new(*one_port, state_slot));
        }

        let physical_state_count = runtime_state.len();
        let transformed_state_count = self.iir.a_coeffs.len().saturating_sub(1);

        self.runtime_state = runtime_state;
        self.state_map = IirStateMap::new(
            physical_one_ports,
            physical_state_count,
            transformed_state_count,
        );
    }

    pub fn one_port_states(&self) -> &[OnePortState] {
        &self.runtime_state.states
    }

    pub fn one_port_states_mut(&mut self) -> &mut [OnePortState] {
        &mut self.runtime_state.states
    }

    pub fn bind_ports(&mut self, input_node_id: Option<usize>, output_node_id: Option<usize>) {
        self.input_binding = input_node_id.map(|node| PortBinding::new(BindingId::new(node), 0));
        self.output_binding = output_node_id.map(|node| PortBinding::new(BindingId::new(node), 1));
    }

    pub fn ins(&self) -> Vec<PortBinding> {
        self.input_binding.into_iter().collect()
    }

    pub fn outs(&self) -> Vec<PortBinding> {
        self.output_binding.into_iter().collect()
    }

    /// Configure NonIdealFx post-processing from component declarations.
    ///
    /// Pre-computes runtime constants (gbw_coeff, max_dv_per_sample, v_max)
    /// so process() stays O(1) with no branching on enum variants.
    pub fn set_nonideal_fx(
        &mut self,
        fx: Vec<crate::nonideal_fx::NonIdealFx>,
        sample_rate: crate::Wave,
    ) {
        use crate::nonideal_fx::NonIdealFx;
        for effect in &fx {
            match effect {
                NonIdealFx::OpAmpBandwidth { gbw, slew_rate } => {
                    self.stored_gbw = *gbw;
                    // Estimate closed-loop gain from IIR DC response (b[0]+b[1]+b[2]) / (a[0]+a[1]+a[2])
                    let gain = self.iir.dc_gain().abs().max(1.0);
                    let fc = gbw / gain;
                    let w = 2.0 * crate::math::PI * fc;
                    self.fx_state.gbw_coeff = w / (w + sample_rate);
                    // slew_rate from SPICE model is in V/µs — convert to V/s
                    self.fx_state.max_dv = slew_rate * 1e6 / sample_rate;
                }
                NonIdealFx::RailSaturation { v_max } => {
                    self.fx_state.v_rail_pos = *v_max;
                    self.fx_state.v_rail_neg = *v_max;
                }
            }
        }
        self.nonideal_fx = fx;
    }

    #[inline]
    pub fn process(&mut self, input: crate::Wave) -> crate::Wave {
        // For a single-op-amp active filter (MFB / Sallen-Key) the op-amp output
        // node IS the biquad output: the feedback caps charge from that physical
        // node, so the slew/rail non-ideality acts INSIDE the recurrence. The
        // slew-limited / rail-clipped value is committed to `y_hist[0]` so the
        // denominator feedback senses the real op-amp output node next sample
        // (out-of-loop post-FX would let the filter state evolve from the IDEAL,
        // un-slewed output — the bug this fixes).
        let opamp_out = self
            .iir
            .process_inloop(input * self.compensation, &mut self.fx_state);
        // GBW stays OUT of the loop. The op-amp's own finite-bandwidth pole is a
        // unity-DC single-pole LPF; folding it into the denominator would add an
        // extra in-loop pole that shifts the designed filter's cutoff/Q and
        // colours the passband (the biquad already carries the intended rolloff).
        // Applied here as a post step it models the residual output-bandwidth
        // limit without altering the linear filter response. (Slew + rails are
        // genuine large-signal output-node effects and stay in-loop above.)
        flush_denormal(self.fx_state.gbw_step(opamp_out))
    }

    /// Check if this stage contains a pot with the given component ID.
    pub fn has_pot(&self, comp_id: &str) -> bool {
        self.pot_bindings.iter().any(|b| b.comp_id == comp_id)
    }

    /// Update pot position and recompute IIR coefficients.
    ///
    /// When a lookup table is present, interpolates the full biquad from
    /// precomputed MNA sweeps. Handles cutoff, resonance, and other generic
    /// frequency-shaping pots without corrupting a single numerator term.
    ///
    /// Structured feedback stages can use direct gain/biquad recomputation.
    /// No heap allocations — all state is pre-allocated.
    pub fn set_pot(&mut self, comp_id: &str, position: crate::Wave) {
        let binding_idx = match self.pot_bindings.iter().position(|b| b.comp_id == comp_id) {
            Some(idx) => idx,
            None => return,
        };
        let old_position = self.pot_bindings[binding_idx].position;
        self.pot_bindings[binding_idx].position = position;
        let binding = self.pot_bindings[binding_idx].clone();

        // ── Table lookup path (full biquad interpolation) ──
        if let Some(ref table) = self.biquad_table {
            // Build position vector from all pot bindings, matched by comp_id
            let mut positions = alloc::vec![0.0 as crate::Wave; table.dim_labels.len()];
            for binding in &self.pot_bindings {
                for (di, comp_id) in table.dim_labels.iter().enumerate() {
                    if comp_id.as_str() == binding.comp_id.as_str() {
                        positions[di] = binding.position;
                    }
                }
            }
            let mut coeffs = [0.0 as crate::Wave; 5];
            table.lookup(&positions, &mut coeffs);
            // Update biquad coefficients
            if self.iir.b_coeffs.len() >= 3 && self.iir.a_coeffs.len() >= 3 {
                self.iir.b_coeffs[0] = coeffs[0];
                self.iir.b_coeffs[1] = coeffs[1];
                self.iir.b_coeffs[2] = coeffs[2];
                self.iir.a_coeffs[1] = coeffs[3];
                self.iir.a_coeffs[2] = coeffs[4];
            }
            // Recompute GBW for new DC gain
            if self.stored_gbw > 0.0 {
                let dc_gain = self.iir.dc_gain().abs().max(1.0);
                let fc = self.stored_gbw / dc_gain;
                let w = 2.0 * crate::math::PI * fc;
                self.fx_state.gbw_coeff = w / (w + self.sample_rate);
            }
            return;
        }

        // ── Structured feedback IIR path ──
        // Active feedback filters keep the small set of R/C products needed
        // to recompute the biquad directly. This is the intended fast path
        // for RAT-style gain stages with reactive feedback/ground legs.
        if self.iir.r_series_product > 0.0
            && self.iir.c_shunt_product > 0.0
            && self.iir.r_crit > 0.0
            && binding.role != IirPotRole::GroundLeg
        {
            self.iir.r_fb = (binding.fixed_series_r + position * binding.max_r).max(1.0);
            self.iir.recompute();

            if self.stored_gbw > 0.0 {
                let gain_abs = self.iir.dc_gain().abs().max(1.0);
                let fc = self.stored_gbw / gain_abs;
                let w = 2.0 * crate::math::PI * fc;
                self.fx_state.gbw_coeff = w / (w + self.sample_rate);
            }
            return;
        }

        if binding.role == IirPotRole::GroundLeg && binding.feedback_r > 0.0 {
            let gain_for = |pos: crate::Wave| {
                let rg = (binding.fixed_series_r + pos * binding.max_r).max(1.0);
                if binding.non_inverting {
                    1.0 + binding.feedback_r / rg
                } else {
                    -(binding.feedback_r / rg)
                }
            };
            let old_gain = gain_for(old_position);
            let new_gain = gain_for(position);
            let scale = if old_gain.abs() > 1e-12 {
                new_gain / old_gain
            } else {
                new_gain
            };
            for b in &mut self.iir.b_coeffs {
                *b *= scale;
            }

            if self.stored_gbw > 0.0 {
                let gain_abs = new_gain.abs().max(1.0);
                let fc = self.stored_gbw / gain_abs;
                let w = 2.0 * crate::math::PI * fc;
                self.fx_state.gbw_coeff = w / (w + self.sample_rate);
            }
            return;
        }

        // ── Fallback: DC gain recalculation ──
        if binding.role == IirPotRole::Generic {
            return;
        }

        let rf = binding.fixed_series_r + position * binding.max_r;
        let ri = binding.ri;
        let dc_gain = if ri > 0.0 { -(rf / ri) } else { -1.0 };

        self.iir.b_coeffs[0] = dc_gain;

        if self.stored_gbw > 0.0 {
            let gain_abs = dc_gain.abs().max(1.0);
            let fc = self.stored_gbw / gain_abs;
            let w = 2.0 * crate::math::PI * fc;
            self.fx_state.gbw_coeff = w / (w + self.sample_rate);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// BlackFeedbackStage: clean Rf/Ri gain + NonIdealFx
// ═══════════════════════════════════════════════════════════════════════════

/// Linear VCVS feedback stage using Harold Black's negative feedback formula.
///
/// Gain = Rf/Ri (inverting) or 1 + Rf/Ri (non-inverting). GBW rolloff,
/// slew limiting, and rail saturation come from the shared [`NonIdealFxState`]
/// post-processing — same code path as [`IirStage`].
///
/// No OpAmpRoot, no WDF scattering for the gain computation. The pendant
/// tree (input coupling network) is processed as a passive WDF stage to
/// get the input signal, then gain is applied as scalar multiplication.
///
/// Pot binding: when Rf is a pot, `set_rf()` recomputes gain and updates
/// the GBW coefficient for the new closed-loop bandwidth.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BlackFeedbackStage {
    /// Feedback resistance (Ohms). Changes at runtime when pot sweeps.
    rf: crate::Wave,
    /// Input resistance (Ohms). Fixed at compile time.
    ri: crate::Wave,
    /// True = inverting (gain = -Rf/Ri), false = non-inverting (1 + Rf/Ri).
    inverting: bool,
    /// NonIdealFx post-processing state (GBW/slew/rails).
    fx_state: NonIdealFxState,
    /// Stored GBW for recomputation when gain changes.
    stored_gbw: crate::Wave,
    /// Sample rate for GBW recomputation.
    sample_rate: crate::Wave,
    /// BFS distance from input (for topological ordering).
    pub signal_flow_distance: usize,
    /// Component names in this stage (e.g. "R_in,Cin"). Debug builds only.
    #[cfg(debug_assertions)]
    pub debug_label: String,
    /// When true, this stage is not on the audio signal path (e.g. bias
    /// network). It still processes and meters, but its output does NOT
    /// overwrite the serial audio chain signal.
    pub bypass_serial: bool,
    /// Circuit graph node ID for writing to node_signals (feedforward routing).
    pub output_node_id: usize,
    /// Pot component ID bound to Rf (if any). Set at compile time.
    pub pot_comp_id: Option<String>,
    /// Fixed resistance in series with the feedback pot.
    pub pot_fixed_r: crate::Wave,
    /// Maximum pot resistance (Ohms). Position 1.0 adds this value.
    pub pot_max_r: crate::Wave,
    /// Taper for the feedback pot.
    pub pot_taper: crate::pot_taper::PotTaper,
    /// Pot component ID in the ground leg (Ri). When this pot changes,
    /// Ri = ri_fixed_r + taper(position) × ri_pot_max_r.
    pub ri_pot_comp_id: Option<String>,
    /// Fixed resistance in the ground leg (sum of non-pot resistors: R5 + R6).
    pub ri_fixed_r: crate::Wave,
    /// Max resistance of the Ri pot (e.g. Gain_A max_r = 100k).
    pub ri_pot_max_r: crate::Wave,
    /// Taper for the ground-leg pot.
    pub ri_pot_taper: crate::pot_taper::PotTaper,
    /// Shared one-port runtime state owned by this stage.
    ///
    /// Pure resistive BlackFeedback stages have no physical one-port state; reactive
    /// feedback/network variants can attach explicit state here using the same
    /// physical-state plus WDF-cache sidecar shape as WDF and BKM.
    #[cfg_attr(feature = "serde", serde(default))]
    pub runtime_state: RuntimeState,
}

impl BlackFeedbackStage {
    /// Construct from circuit parameters.
    pub fn new(
        rf: crate::Wave,
        ri: crate::Wave,
        inverting: bool,
        fx: &[crate::nonideal_fx::NonIdealFx],
        sample_rate: crate::Wave,
    ) -> Self {
        let gain = if inverting {
            rf / ri.max(1.0)
        } else {
            1.0 + rf / ri.max(1.0)
        };
        let fx_state = NonIdealFxState::from_nonideal_fx(fx, gain, sample_rate);
        let stored_gbw = fx
            .iter()
            .find_map(|f| match f {
                crate::nonideal_fx::NonIdealFx::OpAmpBandwidth { gbw, .. } => Some(*gbw),
                _ => None,
            })
            .unwrap_or(0.0);

        Self {
            rf,
            ri,
            inverting,
            fx_state,
            stored_gbw,
            sample_rate,
            signal_flow_distance: 0,
            #[cfg(debug_assertions)]
            debug_label: String::new(),
            bypass_serial: false,
            output_node_id: usize::MAX,
            pot_comp_id: None,
            pot_fixed_r: 0.0,
            pot_max_r: 0.0,
            pot_taper: crate::pot_taper::PotTaper::B,
            ri_pot_comp_id: None,
            ri_fixed_r: 0.0,
            ri_pot_max_r: 0.0,
            ri_pot_taper: crate::pot_taper::PotTaper::B,
            runtime_state: RuntimeState::new(),
        }
    }

    /// Test helper: construct with default NonIdealFx.
    pub fn new_test(
        rf: crate::Wave,
        ri: crate::Wave,
        inverting: bool,
        sample_rate: crate::Wave,
    ) -> Self {
        Self::new(rf, ri, inverting, &[], sample_rate)
    }

    /// Current closed-loop gain.
    pub fn gain(&self) -> crate::Wave {
        // When Rf is 0 (all-reactive feedback, e.g., coupling cap only),
        // the DC gain is undefined. Use unity passthrough — the reactive
        // elements handle frequency shaping in the WDF tree, not here.
        if self.rf <= 0.0 && self.ri <= 0.0 {
            return 1.0;
        }
        if self.inverting {
            let g = self.rf / self.ri.max(1.0);
            if g < 1e-6 {
                return -1.0;
            } // Cap-only feedback → unity
            -g
        } else {
            1.0 + self.rf / self.ri.max(1.0)
        }
    }

    /// Current feedback resistance.
    pub fn rf(&self) -> crate::Wave {
        self.rf
    }

    pub fn one_port_states(&self) -> &[OnePortState] {
        &self.runtime_state.states
    }

    pub fn one_port_states_mut(&mut self) -> &mut [OnePortState] {
        &mut self.runtime_state.states
    }

    pub fn runtime_state(&self) -> &RuntimeState {
        &self.runtime_state
    }

    pub fn runtime_state_mut(&mut self) -> &mut RuntimeState {
        &mut self.runtime_state
    }

    /// Check if this stage owns a pot with the given component ID.
    pub fn has_pot(&self, comp_id: &str) -> bool {
        self.pot_comp_id.as_deref() == Some(comp_id)
            || self.ri_pot_comp_id.as_deref() == Some(comp_id)
    }

    /// Set feedback pot position (0.0–1.0).
    ///
    /// Converts to `Rf = fixed_series + taper(position) * max_r`.
    pub fn set_pot(&mut self, comp_id: &str, position: crate::Wave) {
        if self.pot_comp_id.as_deref() == Some(comp_id) && self.pot_max_r > 0.0 {
            let tapered = self.pot_taper.apply(position);
            self.set_rf((self.pot_fixed_r + tapered * self.pot_max_r).max(1.0));
        }
    }

    /// Update Ri from ground-leg pot position. Called when a pot in the
    /// ground leg (e.g. Gain_A in Goldenrod) changes position.
    /// The position is range-mapped but NOT tapered — apply taper here.
    pub fn update_ri_from_pot(&mut self, comp_id: &str, position: crate::Wave) {
        if self.ri_pot_comp_id.as_deref() == Some(comp_id) {
            let tapered = self.ri_pot_taper.apply(position);
            let pot_r = tapered * self.ri_pot_max_r;
            let new_ri = (self.ri_fixed_r + pot_r).max(1.0);
            self.set_ri(new_ri);
        }
    }

    /// Set input resistance (for pipeline Ri fix).
    pub fn set_ri(&mut self, ri: crate::Wave) {
        self.ri = ri;
    }

    /// Set asymmetric rail limits from bias analysis.
    pub fn set_v_rails(&mut self, v_rail_pos: crate::Wave, v_rail_neg: crate::Wave) {
        self.fx_state.v_rail_pos = v_rail_pos;
        self.fx_state.v_rail_neg = v_rail_neg;
    }

    /// Update Rf (pot sweep). Recomputes gain and GBW coefficient.
    pub fn set_rf(&mut self, rf: crate::Wave) {
        self.rf = rf;
        if self.stored_gbw > 0.0 {
            self.fx_state
                .update_gain(self.stored_gbw, self.gain(), self.sample_rate);
        }
    }

    /// Process one sample: in-loop GBW → slew → rails around the closed-form gain.
    ///
    /// `demand = gain * input` is the closed-form linear target — this preserves
    /// the closed-loop DC/low-frequency gain EXACTLY: when `|demand - prev_out|`
    /// is within the slew limit and the GBW pole is above Nyquist, the chain is
    /// transparent and `out == demand`.
    ///
    /// The in-loop coupling lives in `slew_step`: it clamps the change of the
    /// *real* (limited) output node and the same `prev_out` is the feedback the
    /// next sample sees. During a slew event the stage runs effectively
    /// open-loop, ramping at the slew limit regardless of input, then closed-loop
    /// gain resumes once the output catches up. Same shared primitives as the
    /// post-FX `IirStage` path — there is no second copy of this math.
    #[inline]
    pub fn process(&mut self, input: crate::Wave) -> crate::Wave {
        let demand = input * self.gain();
        let out = self.fx_state.gbw_step(demand);
        let out = self.fx_state.slew_step(out);
        let out = self.fx_state.rail_step(out);
        flush_denormal(out)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SerialDelayedFeedbackStage: diagnostic serial WDF cascade + z^-1 feedback
// ═══════════════════════════════════════════════════════════════════════════

/// Diagnostic wrapper for forced-serial blockwise ladders.
///
/// This keeps the individual WDF rung stages intact, but adds a one-sample
/// output feedback path around the serial cascade. It is not a replacement for
/// delay-free BKM coupling; it exists to compare the serial approximation
/// against the fully coupled ladder while preserving normal compiled-stage
/// execution and control binding.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SerialDelayedFeedbackStage {
    pub stages: Vec<WdfStage>,
    pub feedback_gain: crate::Wave,
    #[cfg_attr(feature = "serde", serde(skip))]
    pub delayed_output: crate::Wave,
    pub signal_flow_distance: usize,
    pub bypass_serial: bool,
    #[cfg(debug_assertions)]
    pub debug_label: String,
}

impl SerialDelayedFeedbackStage {
    pub fn new(stages: Vec<WdfStage>, feedback_gain: crate::Wave) -> Self {
        Self {
            stages,
            feedback_gain,
            delayed_output: 0.0,
            signal_flow_distance: 0,
            bypass_serial: false,
            #[cfg(debug_assertions)]
            debug_label: String::new(),
        }
    }

    pub fn has_pot(&self, comp_id: &str) -> bool {
        let aw_id = format!("{comp_id}__aw");
        let wb_id = format!("{comp_id}__wb");
        self.stages
            .iter()
            .any(|stage| stage.has_pot(comp_id) || stage.has_pot(&aw_id) || stage.has_pot(&wb_id))
    }

    pub fn set_pot(&mut self, comp_id: &str, value: crate::Wave) -> bool {
        let aw_id = format!("{comp_id}__aw");
        let wb_id = format!("{comp_id}__wb");
        let mut changed = false;
        for stage in &mut self.stages {
            if stage.set_pot(comp_id, value) {
                stage.flush_recompute();
                changed = true;
            }
            if stage.set_pot(&aw_id, value) {
                stage.flush_recompute();
                changed = true;
            }
            if stage.set_pot(&wb_id, 1.0 - value) {
                stage.flush_recompute();
                changed = true;
            }
        }
        changed
    }

    pub fn reset(&mut self) {
        self.delayed_output = 0.0;
        for stage in &mut self.stages {
            stage.reset();
        }
    }

    #[inline]
    pub fn process(&mut self, input: crate::Wave) -> crate::Wave {
        let mut signal = input + self.feedback_gain * self.delayed_output;
        for stage in &mut self.stages {
            signal = stage.process(signal);
        }
        self.delayed_output = signal;
        signal
    }
}

/// Standalone state-space stage for linear circuits with 3+ reactive elements.
///
/// Implements discrete-time state-space simulation:
///   x[n] = A·x[n-1] + b·u[n]
///   y = c·x + d·u
///
/// O(N²)/sample where N = number of states. Covers complex active filters
/// (Klon stages, BB Preamp) that can't reduce to a biquad.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StateSpaceStage {
    pub ss: StateSpaceData,
    /// Pre-allocated work buffer for state update (avoids per-sample allocation).
    work: Vec<crate::Wave>,
    /// Passive attenuation compensation factor.
    pub compensation: crate::Wave,
    /// BFS distance from input (for topological ordering).
    pub signal_flow_distance: usize,
    /// Component names in this stage (e.g. "R_in,Cin"). Debug builds only.
    #[cfg(debug_assertions)]
    pub debug_label: String,
    /// When true, this stage is not on the audio signal path (e.g. bias
    /// network). It still processes and meters, but its output does NOT
    /// overwrite the serial audio chain signal.
    pub bypass_serial: bool,
    /// Supply voltage for rail saturation.
    pub supply_voltage: crate::Wave,
    /// Pot bindings used to restamp the MNA conductance matrix and rebuild the
    /// discrete state-space matrices when a control changes.
    pub pot_bindings: Vec<StateSpacePotBinding>,
    /// Baseline MNA system used for controlled restamping.
    pub recompute_mna: Option<crate::tree::MnaSystem>,
    /// Precomputed v_rail = (supply/2 - 1.5).max(0.5) and 1/v_rail.
    /// Eliminates per-sample division in tanh saturation loop.
    #[cfg_attr(feature = "serde", serde(skip))]
    v_rail: crate::Wave,
    #[cfg_attr(feature = "serde", serde(skip))]
    inv_v_rail: crate::Wave,
    /// Previous input sample for bilinear state-space forms that carry both
    /// u[n+1] and u[n] input vectors.
    prev_input: crate::Wave,
    /// Canonical physical reactive state slots for this StateSpace source network.
    #[cfg_attr(feature = "serde", serde(default))]
    pub runtime_state: RuntimeState,
    /// Explicit mapping from physical one-port state to the MNA state vector.
    #[cfg_attr(feature = "serde", serde(default))]
    pub state_map: StateSpaceStateMap,
    /// Declarative graph input binding for shared stage routing.
    #[cfg_attr(feature = "serde", serde(default))]
    pub input_binding: Option<PortBinding>,
    /// Declarative graph output binding for shared stage routing.
    #[cfg_attr(feature = "serde", serde(default))]
    pub output_binding: Option<PortBinding>,
    /// F13b: when this is a parallel branch feeding a convergence mixer, read the
    /// per-sample input from `node_signals[input_node_id]` (the shared drive node)
    /// instead of the serial chain, so sibling branches all see the same source
    /// rather than cascading. `usize::MAX` = use the serial chain (default).
    #[cfg_attr(feature = "serde", serde(default = "crate::stage::usize_max"))]
    pub input_node_id: usize,
    /// F13b: when set, publish this stage's per-sample output into
    /// `node_signals[output_node_id]` so the convergence mixer can read it.
    /// `usize::MAX` = do not publish (default).
    #[cfg_attr(feature = "serde", serde(default = "crate::stage::usize_max"))]
    pub output_node_id: usize,
}

/// serde default helper: `usize::MAX` (sentinel for "unbound node").
pub fn usize_max() -> usize {
    usize::MAX
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StateSpacePotBinding {
    pub comp_id: String,
    pub max_r: crate::Wave,
    pub taper: crate::pot_taper::PotTaper,
    pub position: crate::Wave,
    pub terminals: MnaPortTerminals,
    pub conductance: crate::Wave,
    /// True for the `w→b` half of a 3-terminal wiper divider: its resistance is
    /// `(1 − taper.apply(position)) · max_r` (the complement of the `a→w` half),
    /// matching the compile-time `stamp_mna_multi` baseline so deltas stay exact.
    /// Defaults to false (2-terminal rheostats and the `a→w` half).
    #[cfg_attr(feature = "serde", serde(default))]
    pub complement: bool,
}

/// How a StateSpace stage's vector relates to physical reactive one-ports.
pub type StateSpaceStateMap = LtiStateMap<MnaNodeId>;

impl StateSpaceStage {
    pub fn new(ss: StateSpaceData, supply_voltage: crate::Wave) -> Self {
        let n = ss.n_states;
        let v_rail = (supply_voltage * 0.5 - 1.5).max(0.5);
        let mut stage = Self {
            ss,
            work: vec![0.0; n],
            compensation: 1.0,
            signal_flow_distance: 0,
            #[cfg(debug_assertions)]
            debug_label: String::new(),
            bypass_serial: false,
            supply_voltage,
            pot_bindings: Vec::new(),
            recompute_mna: None,
            v_rail,
            inv_v_rail: 1.0 / v_rail,
            prev_input: 0.0,
            runtime_state: RuntimeState::new(),
            state_map: StateSpaceStateMap::empty(n),
            input_binding: None,
            output_binding: None,
            input_node_id: usize::MAX,
            output_node_id: usize::MAX,
        };
        stage.bind_physical_one_ports();
        stage
    }

    pub fn bind_physical_one_ports(&mut self) {
        let mut runtime_state = RuntimeState::new();
        let mut physical_one_ports = Vec::new();

        for one_port in &self.ss.reactive_one_ports {
            let state_slot = runtime_state.allocate_one_port(one_port.kind);
            physical_one_ports.push(RuntimeOnePort::new(*one_port, state_slot));
        }

        let physical_state_count = runtime_state.len();
        let transformed_state_count = self.ss.n_states;

        self.runtime_state = runtime_state;
        self.state_map = StateSpaceStateMap::new(
            physical_one_ports,
            physical_state_count,
            transformed_state_count,
        );
    }

    pub fn one_port_states(&self) -> &[OnePortState] {
        &self.runtime_state.states
    }

    pub fn one_port_states_mut(&mut self) -> &mut [OnePortState] {
        &mut self.runtime_state.states
    }

    pub fn bind_ports(&mut self, input_node_id: Option<usize>, output_node_id: Option<usize>) {
        self.input_binding = input_node_id.map(|node| PortBinding::new(BindingId::new(node), 0));
        self.output_binding = output_node_id.map(|node| PortBinding::new(BindingId::new(node), 1));
    }

    pub fn ins(&self) -> Vec<PortBinding> {
        self.input_binding.into_iter().collect()
    }

    pub fn outs(&self) -> Vec<PortBinding> {
        self.output_binding.into_iter().collect()
    }

    /// Recompute cached v_rail after supply voltage change or deserialization.
    pub fn recompute_v_rail(&mut self) {
        self.v_rail = (self.supply_voltage * 0.5 - 1.5).max(0.5);
        self.inv_v_rail = 1.0 / self.v_rail;
    }

    pub fn has_pot(&self, comp_id: &str) -> bool {
        let aw_id = format!("{comp_id}__aw");
        let wb_id = format!("{comp_id}__wb");
        self.pot_bindings.iter().any(|binding| {
            binding.comp_id == comp_id || binding.comp_id == aw_id || binding.comp_id == wb_id
        })
    }

    /// Delta-update the stored MNA G-matrix for one pot binding at `idx` to the
    /// new wiper `position`. Returns true if the conductance changed (matrix was
    /// touched). Does NOT rebuild the state-space matrices — callers batch that.
    fn restamp_pot_binding(&mut self, idx: usize, position: crate::Wave) -> bool {
        let Some(ref mut mna) = self.recompute_mna else {
            return false;
        };
        let binding = &mut self.pot_bindings[idx];
        binding.position = position.clamp(0.0, 1.0);
        let tapered = binding.taper.apply(binding.position);
        // w→b half: resistance is the complement of the tapered a→w half, so the
        // two segments always sum to max_r and the baseline matches the
        // compile-time `stamp_mna_multi` stamp ((1−taper.apply(0.5))·max_r).
        let frac = if binding.complement {
            1.0 - tapered
        } else {
            tapered
        };
        let new_r = (frac * binding.max_r).max(1.0);
        let new_g = 1.0 / new_r;
        let delta = new_g - binding.conductance;
        if delta.abs() <= 1e-15 {
            return false;
        }

        let n_mna = mna.num_nodes;
        let (node_pos, node_neg) = binding.terminals.raw().as_tuple();
        if let Some(p) = node_pos {
            mna.g_matrix[p * n_mna + p] += delta;
            if let Some(n) = node_neg {
                mna.g_matrix[p * n_mna + n] -= delta;
            }
        }
        if let Some(n) = node_neg {
            mna.g_matrix[n * n_mna + n] += delta;
            if let Some(p) = node_pos {
                mna.g_matrix[n * n_mna + p] -= delta;
            }
        }
        binding.conductance = new_g;
        true
    }

    pub fn set_pot(&mut self, comp_id: &str, position: crate::Wave) {
        // GAP (a): a 3-terminal wiper pot is bound as two segment leaves with
        // synthetic ids `{comp_id}__aw` (tracks `position`) and `{comp_id}__wb`
        // (tracks `1 − position`). Drive every matching binding — the exact id
        // (2-terminal rheostat) and both split halves — then rebuild once.
        let aw_id = format!("{comp_id}__aw");
        let wb_id = format!("{comp_id}__wb");
        let mut touched = false;
        for idx in 0..self.pot_bindings.len() {
            let id = &self.pot_bindings[idx].comp_id;
            // All matching bindings receive the SAME wiper position. The `__wb`
            // binding carries `complement: true`, so `restamp_pot_binding`
            // derives its (1 − taper.apply(position)) resistance internally.
            if !(id == comp_id || id == &aw_id || id == &wb_id) {
                continue;
            }
            touched |= self.restamp_pot_binding(idx, position);
        }
        if !touched {
            return;
        }

        let Some(ref mut mna) = self.recompute_mna else {
            return;
        };

        let (a_d, b_d, c_out, n_states, d_feedthrough) = mna.build_state_space_matrices(
            &self.ss.reactive_one_ports,
            self.ss.vs_idx,
            self.ss.output_pos,
            self.ss.output_neg,
            self.ss.sample_rate,
        );
        if n_states == self.ss.n_states
            && a_d.iter().all(|v| v.is_finite())
            && b_d.iter().all(|v| v.is_finite())
            && c_out.iter().all(|v| v.is_finite())
            && d_feedthrough.is_finite()
        {
            self.ss.a_matrix = a_d;
            self.ss.b_vector = b_d;
            self.ss.c_vector = c_out;
            self.ss.d_feedthrough = d_feedthrough;
        }
    }

    #[inline]
    pub fn process(&mut self, input: crate::Wave) -> crate::Wave {
        let n = self.ss.n_states;
        let sample = input * self.compensation;

        // x[n] = A · x[n-1] + b · u[n]  (into pre-allocated work buffer)
        let has_b_minus = self.ss.b_vector.len() >= n * 2;
        for i in 0..n {
            let mut v = self.ss.b_vector[i] * sample;
            if has_b_minus {
                v += self.ss.b_vector[n + i] * self.prev_input;
            }
            let row_start = i * n;
            for j in 0..n {
                v += self.ss.a_matrix[row_start + j] * self.ss.x[j];
            }
            self.work[i] = flush_denormal(v);
        }
        self.prev_input = sample;

        // Op-amp rail saturation: tanh soft-clip state variables.
        // v_rail and inv_v_rail are precomputed (supply voltage is constant).
        let v_rail = self.v_rail;
        let inv_v_rail = self.inv_v_rail;
        for x in &mut self.work[..n] {
            *x = v_rail * crate::fast_math::fast_tanh(*x * inv_v_rail);
        }
        self.ss.x[..n].copy_from_slice(&self.work[..n]);

        // Output extraction: y[n] = c · x[n] + d · u[n]
        let mut y_raw = self.ss.d_feedthrough * sample;
        for i in 0..n {
            y_raw += self.ss.c_vector[i] * self.work[i];
        }

        // 2-sample moving average: kills Nyquist (-1 eigenvalue) parasitics.
        let y = (y_raw + self.ss.prev_output) * 0.5;
        self.ss.prev_output = y_raw;
        flush_denormal(y)
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StateSpaceData {
    /// State vector [V_nodes..., I_vs...] (n_states elements).
    pub x: Vec<crate::Wave>,
    /// State transition matrix A_d = M⁻¹·N (n_states × n_states, row-major).
    pub a_matrix: Vec<crate::Wave>,
    /// Input vector b_d = M⁻¹·F (n_states × 1).
    pub b_vector: Vec<crate::Wave>,
    /// Output extraction vector c_out (1 × n_states).
    pub c_vector: Vec<crate::Wave>,
    /// Number of state variables (num_nodes + num_vsources).
    pub n_states: usize,
    /// Reactive one-ports for rebuilding state-space when G changes.
    pub reactive_one_ports: Vec<MnaOnePort>,
    /// VS branch index for input.
    pub vs_idx: usize,
    /// Output extraction nodes.
    pub output_pos: Option<usize>,
    pub output_neg: Option<usize>,
    /// Sample rate for bilinear transform (2·f_s·C scaling).
    pub sample_rate: crate::Wave,
    /// Direct feedthrough: y = c·x + d·u. From Schur complement elimination.
    pub d_feedthrough: crate::Wave,
    /// Previous output for 2-sample Nyquist filter.
    /// Removes parasitic -1 eigenvalue oscillation from unreduced systems.
    pub prev_output: crate::Wave,
    /// Runtime variable-resistor bindings for delta-updating G when controls change.
    pub variable_resistors: Vec<MnaVariableResistorBinding>,
}

/// Data for a linearized OTA whose gm is stamped into the R-type MNA.
///
/// The OTA operates as a VCCS: I_out = gm * V_in. Its transconductance gm
/// is an off-diagonal conductance in the MNA matrix. When the envelope
/// follower changes the OTA's bias current (Iabc), we recompute gm,
/// delta-update the stored MNA, and re-derive the scattering matrix.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LinearizedOtaData {
    /// OTA model parameters (iabc_max, vt, r_load).
    pub model: crate::elements::OtaModel,
    /// Current normalized gain (0.0 = off, 1.0 = max). Set by envelope.
    pub gain: crate::Wave,
    /// MNA cell indices for the VCCS stamp: (row, col, sign).
    /// These are the cells in g_matrix that encode the transconductance.
    pub stamp_cells: Vec<(usize, usize, crate::Wave)>,
    /// Number of MNA nodes (for indexing into g_matrix).
    pub num_mna_nodes: usize,
}

impl MultiNlStage {
    /// Process one sample through the multi-NL stage.
    ///
    /// 1. Set control voltages (Vbe/Vgk) on each NL device
    /// 2. For each oversampled sub-sample:
    ///    a. Scatter-up passive children → b_passive[k]
    ///    b. Compute known_a[i] = Σ_k S_nl_passive[i][k]·b_passive[k] + S_nl_adapted[i]·b_vs
    ///    c. Multi-port NR solve → b_nl[i] for all NL ports
    ///    d. Set all b values on adaptor → scatter_down → set_incident on passive children
    ///    e. Output = (a_out + b_nl[output_port]) / 2 (voltage at output NL port)
    /// 3. Return output sample
    pub fn process(&mut self, input: crate::Wave) -> crate::Wave {
        // Apply inter-stage transformer voltage gain (1.0 when no transformer).
        let input = input * self.transformer_gain;

        // ── IIR path: compiled from linear R-node MNA ────────────────────
        // Takes priority over state-space. Correct Q for oscillators,
        // simpler and faster per-sample than matrix state-space.
        if let Some(ref mut iir) = self.iir {
            let output = self.oversampler.process(input, |sample| {
                flush_denormal(iir.process(sample * self.compensation))
            });
            return flush_denormal(output);
        }

        // ── State-space path: direct discrete-time simulation ────────────
        // Bypasses WDF scattering entirely. Used for linearized OTA stages
        // where cap port conductances overwhelm circuit conductances.
        if let Some(ref mut ss) = self.state_space {
            let work = &mut self.work_b_passive;
            let v_rail = (self.supply_voltage * 0.5 - 1.5).max(0.5);
            let output = self.oversampler.process(input, |sample| {
                let n = ss.n_states;
                // x[n] = A · x[n-1] + b · u[n]
                work.resize(n, 0.0);
                for (i, (w, &b_i)) in work[..n].iter_mut().zip(&ss.b_vector[..n]).enumerate() {
                    let row_start = i * n;
                    let mut v = b_i * sample;
                    for (&a_ij, &x_j) in
                        ss.a_matrix[row_start..row_start + n].iter().zip(&ss.x[..n])
                    {
                        v += a_ij * x_j;
                    }
                    *w = flush_denormal(v);
                }

                // Op-amp rail saturation: apply tanh soft-clip to every state.
                // In oscillator circuits (bridged-T), the op-amp output exceeds
                // supply rails → tanh limits the amplitude → creates a stable
                // limit cycle whose frequency is set by the RC network.
                // Cap voltages are also clipped since they can't exceed Vcc.
                if v_rail > 0.0 {
                    for w in &mut work[..n] {
                        *w = v_rail * crate::fast_math::fast_tanh(*w / v_rail);
                    }
                }

                ss.x[..n].copy_from_slice(&work[..n]);

                // Output extraction: y[n] = c · x[n] + d · u[n]
                let mut y_raw = ss.d_feedthrough * sample;
                for (&c_i, &w) in ss.c_vector[..n].iter().zip(&work[..n]) {
                    y_raw += c_i * w;
                }
                // 2-sample moving average: kills Nyquist (-1 eigenvalue)
                // parasitics from unreduced VCVS algebraic constraints.
                // At 130 Hz this averages consecutive samples that are
                // nearly identical → negligible signal loss. At Nyquist
                // (alternating +/-) the average is zero → perfect cancellation.
                let y = (y_raw + ss.prev_output) * 0.5;
                ss.prev_output = y_raw;
                y
            });
            return flush_denormal(output);
        }

        let compensation = self.compensation;
        let n_nl = self.n_nl;
        let n_passive = self.passive_one_ports.len();
        let output_port = self.output_port;

        // Set control voltages on each NL device.
        // For independent devices, set directly. For grouped devices,
        // multi-port groups (TriodeThreePort, VariMuThreePort) get grid
        // voltage from the WDF port, but SinglePort wrappers (e.g., BJT
        // in a mixed BJT+Diode stage) still need explicit control voltage.
        if let Some(ref mut dg) = self.device_groups {
            let bias_offset = self.veb_bias_offset;
            for group in &mut dg.groups {
                if let NlDeviceGroupKind::SinglePort(ref mut device) = group {
                    device.set_control_voltage(input, compensation, bias_offset);
                }
            }
        } else {
            let bias_offset = self.veb_bias_offset;
            for device in &mut self.nl_devices {
                device.set_control_voltage(input, compensation, bias_offset);
            }
        }

        // DC ramp: gradually increase VCC bias over DC_RAMP_SAMPLES to let
        // the NR solver track the operating point as supply voltage ramps up.
        const DC_RAMP_SAMPLES: u32 = 256;
        let still_ramping = self.dc_ramp < DC_RAMP_SAMPLES;
        let dc_scale = if !still_ramping {
            1.0
        } else {
            self.dc_ramp += 1;
            self.dc_ramp as crate::Wave / DC_RAMP_SAMPLES as crate::Wave
        };

        // Adaptive NR tolerance: loosen on transients to reduce iterations.
        let input_delta = input - self.prev_input;
        self.prev_input = input;
        let nr_tolerance = adaptive_nr_tolerance(input_delta);

        // Reset frozen failure counter for this base sample's sub-samples
        self.nr_workspace.frozen_failures = 0;
        self.subsample_counter = 0;
        // Reset per-base-sample NR iteration budget. This caps total NR work
        // across all sub-samples to prevent CPU spikes on transients.
        self.iteration_budget_remaining = NR_ITERATION_BUDGET;
        let adaptive_x2 = self.adaptive_x2;

        // Linear extrapolation warm-start for NR solver (first sub-sample only).
        // v_prev holds v[n-1]; v_prev_2 holds v[n-2].
        // Extrapolated guess: v[n] ~= 2*v[n-1] - v[n-2].
        // We shift history and write the guess into v_prev in one pass
        // (no allocation). Subsequent sub-samples naturally warm-start from
        // the previous sub-sample's converged solution.
        if !self.v_prev.is_empty() {
            for i in 0..self.v_prev.len() {
                let old = self.v_prev[i];
                let extrap = 2.0 * old - self.v_prev_2[i];
                self.v_prev_2[i] = old;
                self.v_prev[i] = if extrap.is_finite() { extrap } else { old };
            }
        }
        let output = self.oversampler.process(input, |sample| {
            let subsample_idx = self.subsample_counter;
            self.subsample_counter += 1;

            // 1. Scatter-up passive children (always — caps need state updates)
            let b_passive = &mut self.work_b_passive[..n_passive];
            for (k, one_port) in self.passive_one_ports.iter().enumerate() {
                b_passive[k] = one_port.wdf_reflected(&self.passive_runtime_state);
            }

            // Adaptive X2: on odd sub-samples, skip known_a + NR solve.
            // Reuse b_nl from previous sub-sample (still in nr_workspace).
            let skip_nr_adaptive = adaptive_x2 && (subsample_idx % 2 == 1) && n_nl > 0;
            // Budget cap: skip NR when this base sample's iteration budget is exhausted.
            // Reuse last b_nl from workspace — still valid from the previous sub-sample.
            let skip_nr_budget = n_nl > 0 && self.iteration_budget_remaining == 0;
            let skip_nr = skip_nr_adaptive || skip_nr_budget;

            // The adapted port's b-wave is the input signal (voltage source).
            // When a feedback opamp is paired (e.g. Bluesbreaker, Tube Screamer fallback),
            // apply opamp gain + GBW + slew limiting before the MNA/NR solve.
            let b_adapted = if let Some(ref mut opamp) = self.feedback_opamp {
                opamp.compute_vs_voltage(sample * compensation)
            } else {
                sample * compensation
            };

            if !skip_nr {
            // 2. Compute known_a[i] for each NL port:
            // known_a[i] = Σ_k S_nl_passive[i][k] * b_passive[k]
            //             + S_nl_adapted[i] * b_adapted
            //             + dc_bias[i]  (VCC supply contribution, precomputed constant)
            let known_a = &mut self.work_known_a[..n_nl];
            for (i, (ka, (&s_adapt, &dc))) in known_a
                .iter_mut()
                .zip(
                    self.scattering.s_nl_adapted[..n_nl]
                        .iter()
                        .zip(&self.dc_bias[..n_nl]),
                )
                .enumerate()
            {
                let mut a_i = s_adapt * b_adapted;
                for (&s, &bp) in self.scattering.s_nl_passive
                    [i * n_passive..i * n_passive + n_passive]
                    .iter()
                    .zip(&b_passive[..n_passive])
                {
                    a_i += s * bp;
                }
                a_i += dc * dc_scale;
                *ka = a_i;
            }

            #[cfg(feature = "debug-trace")]
            {
                static NR_TRACE: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);
                let n = NR_TRACE.fetch_add(1, Ordering::Relaxed);
                if n < 5 {
                    std::eprintln!("[NR-input] n_nl={} sample={:.6e} comp={} known_a={:?} b_passive={:?} b_adapted={:.6e} s_nl_adapted={:?}", n_nl, sample, compensation, known_a, b_passive, b_adapted, &self.scattering.s_nl_adapted);
                }
            }
            // 3. Multi-port NR solve (skipped when n_nl=0, e.g. linearized OTA)
            if n_nl > 0 {
                // Clamp per-solve max_iter to whatever budget remains for this
                // base sample. This prevents unlimited iteration consumption on
                // transients when all sub-samples need full NR convergence.
                let max_iter = crate::elements::nonlinear::solver::NR_MAX_ITER
                    .min(self.iteration_budget_remaining);
                if let Some(ref dg) = self.device_groups {
                    // Grouped solver: cross-coupled device Jacobians
                    let groups: Vec<&dyn NlDeviceGroupIv> =
                        dg.groups.iter().map(|g| g.as_group_iv()).collect();

multi_port_nr_solve_grouped_into(
                        n_nl,
                        &self.scattering.s_nl,
                        known_a,
                        &self.nl_port_resistances,
                        &groups,
                        &dg.offsets,
                        &mut self.v_prev,
                        max_iter,
                        nr_tolerance,
                        &mut self.nr_workspace,
                    );
                } else {
                    // Independent solver: each device has its own I-V
                    let devices: Vec<&dyn NlDeviceIv> = self
                        .nl_devices
                        .iter()
                        .map(|d| d.as_nl_device_iv())
                        .collect();
                    multi_port_nr_solve_into(
                        n_nl,
                        &self.scattering.s_nl,
                        known_a,
                        &self.nl_port_resistances,
                        &devices,
                        &mut self.v_prev,
                        max_iter,
                        nr_tolerance,
                        &mut self.nr_workspace,
                    );
                }
                // Deduct actual iterations from the remaining budget.
                // When budget reaches zero, subsequent sub-samples reuse b_nl.
                self.iteration_budget_remaining = self
                    .iteration_budget_remaining
                    .saturating_sub(self.nr_workspace.iters_used);
            }
            } // end if !skip_nr — b_nl in workspace is either fresh or reused

            let b_nl = &self.nr_workspace.b_nl[..n_nl];

            #[cfg(feature = "debug-trace")]
            {
                static NR_TRACE2: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);
                let n = NR_TRACE2.fetch_add(1, Ordering::Relaxed);
                if n < 5 {
                    std::eprintln!("[NR-output] b_nl={:?} v_prev={:?}", b_nl, self.v_prev);
                }
            }
            // 4. Build full b-vector for scatter_down (into pre-allocated buffer):
            //    [b_nl..., b_passive..., b_adapted]
            //    With VS injection: no adapted port at end.
            let use_vs_injection = self.vs_injection.is_some();
            let n_total = n_nl + n_passive + if use_vs_injection { 0 } else { 1 };
            let b_all = &mut self.work_b_all[..n_total];
            b_all[..n_nl].copy_from_slice(b_nl);
            b_all[n_nl..n_nl + n_passive].copy_from_slice(b_passive);
            if !use_vs_injection {
                b_all[n_nl + n_passive] = b_adapted;
            }

            // Use scatter_all_into to avoid allocation
            let a_all = &mut self.work_a_all[..n_total];
            self.adaptor.scatter_all_into(b_all, a_all);

            // Add VS injection: a[i] += k[i] * V_in
            if let Some(ref k) = self.vs_injection {
                for i in 0..a_all.len() {
                    if i < k.len() {
                        a_all[i] += k[i] * b_adapted;
                    }
                }
            }

            // Add VCC supply injection to all ports (with DC ramp).
            if !self.vcc_bias_all.is_empty() {
                for i in 0..a_all.len().min(self.vcc_bias_all.len()) {
                    a_all[i] += self.vcc_bias_all[i] * dc_scale;
                }
            }

            // 5. Set incident waves on passive children
            for (k, one_port) in self.passive_one_ports.iter().enumerate() {
                one_port.wdf_set_incident(a_all[n_nl + k], &mut self.passive_runtime_state);
            }

            // 6. Output extraction
            let raw_out = if let Some(ref coeffs) = self.extract_coeffs {
                // Direct node-voltage extraction in the same port order used
                // to derive the coefficients: [NL..., passive..., adapted?].
                let mut v_out = self.extract_vs * b_adapted;
                for k in 0..n_total.min(coeffs.len()) {
                    v_out += coeffs[k] * b_all[k];
                }
                v_out
            } else {
                // Standard WDF output: (a + b) / 2 at the output port
                let a_out = a_all[output_port];
                let b_out = if output_port < n_nl {
                    b_nl[output_port]
                } else {
                    b_passive[output_port - n_nl]
                };
                (a_out + b_out) / 2.0
            };

            // 7. DC blocker for stages with VCC supply injection.
            // With VCC as an ideal VS, NL port voltages include the DC operating
            // point. The real circuit's coupling caps block this DC, so we apply
            // a 1-pole high-pass: y[n] = x[n] - x[n-1] + α·y[n-1]
            // with α ≈ 0.9995 (fc ≈ 3.5Hz at 44.1kHz, below audible range).
            if !self.vcc_bias_all.is_empty() {
                let x0 = if raw_out.is_finite() { raw_out } else { 0.0 };
                if still_ramping {
                    // During DC ramp, seed the blocker state with the current
                    // DC operating point but output zero — the ramp transient
                    // is not audio, just NR solver initialization.
                    self.dc_blocker_x1 = x0;
                    self.dc_blocker_y1 = 0.0;
                    0.0
                } else {
                    let y0 = x0 - self.dc_blocker_x1 + 0.9995 * self.dc_blocker_y1;
                    let y0 = if y0.is_finite() { y0 } else { 0.0 };
                    self.dc_blocker_x1 = x0;
                    self.dc_blocker_y1 = y0;
                    y0
                }
            } else {
                raw_out
            }
        });

        // Update adaptive oversampling for next base sample:
        // If all sub-samples converged via frozen Newton (zero failures),
        // the signal is slowly varying → X2 NR rate suffices next sample.
        self.adaptive_x2 =
            self.nr_workspace.frozen_failures == 0 && self.nr_workspace.has_cached_jac && n_nl > 0;

        #[cfg(feature = "debug-trace")]
        if input.abs() > 1e-10 {
            let n = TRACE_COUNT_MNL.fetch_add(1, Ordering::Relaxed);
            if n < MAX_TRACE_MNL {
                let stage_id = if let Some(ref dg) = self.device_groups {
                    dg.groups
                        .iter()
                        .map(|g| g.debug_name())
                        .collect::<Vec<_>>()
                        .join(", ")
                } else {
                    self.nl_devices
                        .iter()
                        .map(|d| d.debug_name())
                        .collect::<Vec<_>>()
                        .join(", ")
                };
                let gain_db = if output.abs() > 1e-30 && input.abs() > 1e-30 {
                    20.0 * (output / input).abs().log10()
                } else {
                    -999.0
                };
                std::eprintln!(
                    "[MNL n={n}] [{stage_id}] n_nl={} out_port={} comp={:.4} xfmr={:.2}",
                    self.n_nl,
                    self.output_port,
                    self.compensation,
                    self.transformer_gain
                );
                std::eprintln!("  in={input:.6e} out={output:.6e} gain={gain_db:.1}dB");
                std::eprintln!("  s_nl_adapted={:.6?}", &self.scattering.s_nl_adapted);
                // s_nl diagonal (self-coupling) and off-diagonal
                let mut s_diag = Vec::with_capacity(self.n_nl);
                for i in 0..self.n_nl {
                    s_diag.push(self.scattering.s_nl[i * self.n_nl + i]);
                }
                std::eprintln!("  s_nl_diag={:.6?}", s_diag);
                std::eprintln!(
                    "  v_prev={:.4?} Rp={:.1?}",
                    &self.v_prev,
                    &self.nl_port_resistances
                );
            }
        }

        flush_denormal(output)
    }

    /// Adjust reactive element port resistances for oversampling.
    ///
    /// MultiNlStage passive children (caps/inductors) need their port
    /// resistances updated to the oversampled rate for correct frequency
    /// Adjust reactive element port resistances for oversampling.
    ///
    /// Since the scattering matrix is now built at the effective (oversampled)
    /// rate, passive children are already at the correct rate. This is a no-op
    /// for MultiNlStage — the builder handles oversampled rate directly.
    pub fn apply_oversampling_rate(&mut self, base_rate: crate::Wave) {
        // DynNode trees are already at the correct rate (built with effective_rate).
        // But feedback_opamp GBW/slew filters need the oversampled rate.
        if let Some(ref mut opamp) = self.feedback_opamp {
            let effective_rate = base_rate * self.oversampler.ratio() as crate::Wave;
            if effective_rate > base_rate {
                opamp.set_sample_rate(effective_rate);
            }
        }
    }

    /// Update the supply voltage for power supply sag.
    ///
    /// Scales dc_bias and vcc_bias_all linearly with the new voltage relative
    /// to the build-time supply voltage. This shifts the transistor DC operating
    /// point in response to supply droop, modeling real power supply sag.
    pub fn update_supply_voltage(&mut self, new_voltage: crate::Wave) {
        if self.supply_voltage == 0.0 || self.vcc_bias_all.is_empty() {
            return;
        }
        let scale = new_voltage / self.supply_voltage;
        for bias in &mut self.dc_bias {
            *bias *= scale;
        }
        for bias in &mut self.vcc_bias_all {
            *bias *= scale;
        }
        self.supply_voltage = new_voltage;
    }

    /// Reset all internal state.
    pub fn reset(&mut self) {
        for one_port in &self.passive_one_ports {
            one_port.wdf_reset(&mut self.passive_runtime_state);
        }
        // Restore physics-based initial guesses instead of zeroing.
        // Zeroing v_prev causes the NR solver to start far from the operating
        // point, leading to divergence and 100% CPU in real-time contexts.
        self.v_prev.copy_from_slice(&self.initial_v_prev);
        // Reset 2-sample history for extrapolation warm-start.
        // Seed from initial_v_prev so extrapolation starts near the operating point.
        self.v_prev_2.copy_from_slice(&self.initial_v_prev);
        // When init hints are present (initial_dc_ramp = DC_RAMP_SAMPLES), restore
        // dc_ramp to fully-ramped so the first sample sees dc_scale = 1.0.
        // This preserves the hint-seeded v_prev: without it, dc_scale ≈ 0 on
        // sample 0 drives known_a ≈ 0 and the NR converges to v ≈ 0 regardless
        // of the asymmetric warm-start, erasing the init hint's effect.
        self.dc_ramp = self.initial_dc_ramp;
        self.dc_blocker_x1 = 0.0;
        self.dc_blocker_y1 = 0.0;
        if let Some(ref mut ss) = self.state_space {
            for v in &mut ss.x {
                *v = 0.0;
            }
        }
        self.oversampler.reset();
        self.adaptive_x2 = false;
        self.nr_workspace.has_cached_jac = false;
        self.nr_workspace.frozen_failures = 0;
        self.prev_input = 0.0;
        self.iteration_budget_remaining = NR_ITERATION_BUDGET;
        if let Some(ref mut opamp) = self.feedback_opamp {
            opamp.reset();
        }

        // ko5g g725.2: a nonlinear DC Q-point seed pre-charges the coupling caps
        // to their DC voltages so the grouped-BJT NR starts in the active basin
        // (not the cutoff fixed point).  `wdf_reset` above just zeroed those caps,
        // so re-apply the pre-charge (and re-derive dc_bias) here.  No-op when no
        // seed is present, so non-BJT stages are unaffected.
        if self.dc_qpoint_v.is_some() && !self.dc_qpoint_passive_b.is_empty() {
            let n_pre = self
                .passive_one_ports
                .len()
                .min(self.dc_qpoint_passive_b.len());
            for k in 0..n_pre {
                let bk = self.dc_qpoint_passive_b[k];
                let one_port = self.passive_one_ports[k];
                // Seed the reflected wave directly (see compiler apply_bjt_dc_qpoint).
                one_port.wdf_set_incident(bk, &mut self.passive_runtime_state);
            }
            // Generic joint path (b2ps): dc_bias is already the consistent joint
            // inversion — do NOT re-derive it (would clobber vcc_bias_all).  The
            // cap pre-charge + the v_prev restore above suffice.
            if !self.dc_qpoint_joint {
                self.apply_dc_qpoint_seed();
            }
        }
    }

    /// Build a read-only compile-time MNA/scattering diagnostics snapshot.
    ///
    /// Captures the full **standard** scattering matrix `S` (reconstructed from
    /// the adaptor's power-normalized form), every NL port's operating-point
    /// linearization (`gm`/companion), the adapted `nl_port_resistances`, the
    /// node-voltage `extraction_coeffs`, and the VS injection vector — plus port
    /// labels so the matrix is interpretable. Derived on demand; no per-sample
    /// cost and no behavior change.
    ///
    /// `stage_index` is the caller-supplied position of this stage in
    /// `CompiledPedal::stages` (used only to label the snapshot).
    pub fn mna_snapshot(&self, stage_index: usize) -> crate::diag::MnaStageSnapshot {
        use alloc::format;
        use alloc::string::String;
        use alloc::vec::Vec;

        let n_nl = self.n_nl;
        let n_passive = self.passive_one_ports.len();

        // ── Full standard scattering matrix from the adaptor ─────────────────
        // The adaptor stores S̄ (power-normalized); reconstruct standard S via
        // S[i][j] = S̄[i][j] · √(R_i / R_j).
        let n_total = self.adaptor.num_ports();
        let s_bar = self.adaptor.power_scattering();
        let port_r = self.adaptor.port_resistances();
        let mut s_full = Vec::with_capacity(n_total * n_total);
        if s_bar.len() == n_total * n_total && port_r.len() == n_total {
            for i in 0..n_total {
                for j in 0..n_total {
                    let scale = crate::math::sqrt(port_r[i] / port_r[j]);
                    s_full.push(s_bar[i * n_total + j] * scale);
                }
            }
        }

        // ── Port labels (adaptor order: NL, passive, extras, adapted) ────────
        let mut port_labels: Vec<String> = Vec::with_capacity(n_total);
        let nl_labels = self.nl_port_labels();
        for lbl in &nl_labels {
            port_labels.push(lbl.clone());
        }
        for k in 0..n_passive {
            port_labels.push(format!("passive[{k}]:{}", self.passive_one_ports[k].type_tag()));
        }
        // Remaining ports (vcc / adapted parent) — label by position.
        while port_labels.len() < n_total {
            let idx = port_labels.len();
            if idx + 1 == n_total {
                port_labels.push(String::from("adapted"));
            } else {
                port_labels.push(format!("extra[{idx}]"));
            }
        }

        // ── Per-NL-port operating-point linearization (gm / companion) ───────
        let nl_linearization = self.nl_linearizations(&nl_labels);

        // ── Extraction + VS injection ────────────────────────────────────────
        let extraction_coeffs = self.extract_coeffs.as_ref().map(|c| c.to_vec());
        let vs_injection = self.vs_injection.as_ref().map(|c| c.to_vec());

        let label = {
            #[cfg(debug_assertions)]
            {
                self.debug_label.clone()
            }
            #[cfg(not(debug_assertions))]
            {
                String::new()
            }
        };

        crate::diag::MnaStageSnapshot {
            stage_index,
            label,
            n_nl,
            n_passive,
            n_ports_total: n_total,
            port_labels,
            port_resistances: port_r,
            s_full,
            s_dim: n_total,
            nl_linearization,
            nl_port_resistances: self.nl_port_resistances.clone(),
            extraction_coeffs,
            extraction_vs: self.extract_vs,
            vs_injection,
            output_port: self.output_port,
            compensation: self.compensation,
            supply_voltage: self.supply_voltage,
            bypass_serial: self.bypass_serial,
        }
    }

    /// Build human-readable labels for each NL port (length `n_nl`).
    ///
    /// Grouped devices (BJT/triode) emit two ports — base-emitter (`.be`) and
    /// collector-emitter (`.ce`) for BJTs, grid (`.g`) and plate (`.p`) for
    /// triodes/pentodes. Independent devices emit one port labelled by family.
    fn nl_port_labels(&self) -> alloc::vec::Vec<alloc::string::String> {
        use alloc::format;
        use alloc::string::String;
        use alloc::vec::Vec;
        let mut labels: Vec<String> = Vec::with_capacity(self.n_nl);
        if let Some(ref dg) = self.device_groups {
            for (g, group) in dg.groups.iter().enumerate() {
                let name = group.debug_name();
                let np = group.n_ports();
                let suffixes: &[&str] = match group {
                    NlDeviceGroupKind::BjtTwoPort(_) | NlDeviceGroupKind::EbersMollTwoPort(_) => {
                        &["be", "ce"]
                    }
                    NlDeviceGroupKind::VariMuThreePort(_)
                    | NlDeviceGroupKind::TriodeThreePort(_)
                    | NlDeviceGroupKind::PentodeThreePort(_) => &["gk", "pk"],
                    NlDeviceGroupKind::SinglePort(_) => &["p0"],
                };
                for p in 0..np {
                    let suf = suffixes.get(p).copied().unwrap_or("p");
                    labels.push(format!("G{g}:{name}.{suf}"));
                }
            }
        } else {
            for (i, device) in self.nl_devices.iter().enumerate() {
                labels.push(format!("NL{i}:{}", device.debug_name()));
            }
        }
        // Pad/truncate defensively to n_nl.
        while labels.len() < self.n_nl {
            let i = labels.len();
            labels.push(format!("NL{i}"));
        }
        labels.truncate(self.n_nl);
        labels
    }

    /// Evaluate each NL port's operating-point companion linearization.
    ///
    /// For grouped multi-port devices, `eval` is called once per group at the
    /// group's `v_prev` slice; the diagonal Jacobian term is the port's `gm`
    /// and the off-diagonal `∂i/∂(other v)` is reported as `cross_gm`. For
    /// independent devices, `iv(v_prev)` gives `(i, di)` where `di == gm`.
    fn nl_linearizations(
        &self,
        nl_labels: &[alloc::string::String],
    ) -> alloc::vec::Vec<crate::diag::NlPortLinearization> {
        use alloc::vec::Vec;
        let mut out: Vec<crate::diag::NlPortLinearization> = Vec::with_capacity(self.n_nl);
        let v_prev = &self.v_prev;
        let recip = |g: crate::Wave| -> f64 {
            if g.abs() > 1e-30 {
                1.0 / g
            } else {
                f64::INFINITY
            }
        };

        if let Some(ref dg) = self.device_groups {
            // Pre-allocate the largest group's workspace.
            let max_ports = dg.groups.iter().map(|g| g.n_ports()).max().unwrap_or(1);
            let mut v_buf = alloc::vec![0.0 as crate::Wave; max_ports];
            let mut i_buf = alloc::vec![0.0 as crate::Wave; max_ports];
            let mut jac = alloc::vec![0.0 as crate::Wave; max_ports * max_ports];
            for (g, group) in dg.groups.iter().enumerate() {
                let np = group.n_ports();
                let off = dg.offsets[g];
                for (p, slot) in v_buf.iter_mut().enumerate().take(np) {
                    *slot = v_prev.get(off + p).copied().unwrap_or(0.0);
                }
                group
                    .as_group_iv()
                    .eval(&v_buf[..np], &mut i_buf[..np], &mut jac[..np * np]);
                for p in 0..np {
                    let self_g = jac[p * np + p];
                    // Cross term: the strongest off-diagonal Jacobian entry in
                    // this row (∂i_p/∂v_other). For a BJT CE row this is gm.
                    let cross = (0..np)
                        .filter(|&q| q != p)
                        .map(|q| jac[p * np + q])
                        .max_by(|a, b| a.abs().partial_cmp(&b.abs()).unwrap())
                        .filter(|c| c.abs() > 0.0);
                    let idx = off + p;
                    out.push(crate::diag::NlPortLinearization {
                        label: nl_labels.get(idx).cloned().unwrap_or_default(),
                        device: alloc::string::String::from(group.debug_name()),
                        v_op: v_buf[p],
                        gm: self_g,
                        r_companion: recip(self_g),
                        cross_gm: cross,
                        port_resistance: self.nl_port_resistances.get(idx).copied().unwrap_or(0.0),
                    });
                }
            }
        } else {
            for (i, device) in self.nl_devices.iter().enumerate() {
                let v = v_prev.get(i).copied().unwrap_or(0.0);
                let (_i_op, di) = device.as_nl_device_iv().iv(v);
                out.push(crate::diag::NlPortLinearization {
                    label: nl_labels.get(i).cloned().unwrap_or_default(),
                    device: alloc::string::String::from(device.debug_name()),
                    v_op: v,
                    gm: di,
                    r_companion: recip(di),
                    cross_gm: None,
                    port_resistance: self.nl_port_resistances.get(i).copied().unwrap_or(0.0),
                });
            }
        }
        out
    }

    /// Live runtime operating-point capture for the diagnostics ring (Phase B).
    ///
    /// Evaluates each grouped 2-port device at its **current** NR operating point
    /// (`v_prev` after this sample's solve) to read out, per device:
    /// `Vbe = v_prev[be]`, `Vce = v_prev[ce]`, the device currents
    /// `Ib = currents[0]`, `Ic = currents[1]`, the transconductance
    /// `gm = ∂Ic/∂Vbe = jac[2]`, and the output conductance `∂Ic/∂Vce = jac[3]`.
    ///
    /// This is the per-sample analogue of [`Self::nl_linearizations`] — it is the
    /// only place that reads the **dynamic** Vbe→Ic relationship the static MNA
    /// snapshot cannot show (the whole BA283 −49.5 dB question). Triode/pentode
    /// groups map `gk`→`v_be`/`i_b` slot, `pk`→`v_ce`/`i_c` slot so the same
    /// record carries grid/plate state. `records.len()` bounds the output.
    #[cfg(feature = "diag")]
    pub fn runtime_op_points(
        &self,
        stage_index: usize,
        records: &mut [crate::diag_ring::OpPointRecord],
    ) -> usize {
        use crate::diag_ring::OpPointRecord;
        let dg = match self.device_groups {
            Some(ref dg) => dg,
            None => return 0,
        };
        let mut v_buf = [0.0 as crate::Wave; 3];
        let mut i_buf = [0.0 as crate::Wave; 3];
        let mut jac = [0.0 as crate::Wave; 9];
        let mut n = 0usize;
        for (g, group) in dg.groups.iter().enumerate() {
            if n >= records.len() {
                break;
            }
            let np = group.n_ports().min(3);
            let off = dg.offsets.get(g).copied().unwrap_or(0);
            for (p, slot) in v_buf.iter_mut().enumerate().take(np) {
                *slot = self.v_prev.get(off + p).copied().unwrap_or(0.0);
            }
            group
                .as_group_iv()
                .eval(&v_buf[..np], &mut i_buf[..np], &mut jac[..np * np]);
            // 2-port devices: row 1 = CE/plate; jac[1*np+0] = ∂Ic/∂Vbe (gm),
            // jac[1*np+1] = ∂Ic/∂Vce. 1-port devices fold into the BE slot.
            let (v_be, v_ce, i_b, i_c, gm, g_ce) = if np >= 2 {
                (
                    v_buf[0] as f32,
                    v_buf[1] as f32,
                    i_buf[0] as f32,
                    i_buf[1] as f32,
                    jac[np] as f32,         // jac[1*np + 0] = ∂i_ce/∂v_be
                    jac[np + 1] as f32,     // jac[1*np + 1] = ∂i_ce/∂v_ce
                )
            } else {
                (v_buf[0] as f32, 0.0, i_buf[0] as f32, 0.0, jac[0] as f32, 0.0)
            };
            records[n] = OpPointRecord {
                stage_index: stage_index as u32,
                group_index: g as u32,
                v_be,
                v_ce,
                i_c,
                i_b,
                gm,
                g_ce,
            };
            n += 1;
        }
        n
    }

    /// Debug dump: print multi-NL stage structure.
    pub fn debug_dump(&self) -> String {
        let n_passive = self.passive_one_ports.len();
        let n_total = self.n_nl + n_passive + 1; // +1 for adapted port
        let solver_type = if self.device_groups.is_some() {
            "grouped"
        } else {
            "independent"
        };
        let mut s = format!(
            "MultiNlStage(n_nl={}, n_passive={}, n_total={}, output_port={}, comp={:.6}, solver={})\n",
            self.n_nl, n_passive, n_total, self.output_port, self.compensation, solver_type
        );
        if let Some(ref dg) = self.device_groups {
            for (g, group) in dg.groups.iter().enumerate() {
                s.push_str(&format!(
                    "  Group[{}]: {} ({} ports, offset={})\n",
                    g,
                    group.debug_name(),
                    group.n_ports(),
                    dg.offsets[g]
                ));
            }
        } else {
            for (i, device) in self.nl_devices.iter().enumerate() {
                s.push_str(&format!(
                    "  NL[{}]: {}, Rp={:.1}Ω\n",
                    i,
                    device.debug_name(),
                    self.nl_port_resistances[i]
                ));
            }
        }
        for (k, one_port) in self.passive_one_ports.iter().enumerate() {
            s.push_str(&format!(
                "  Passive[{}]: {}, Rp={:.1}Ω, terminals={:?}\n",
                k,
                one_port.type_tag(),
                one_port.rp(self.passive_sample_rate),
                one_port.spec.terminals
            ));
        }
        for (k, child) in self.pot_children.iter().enumerate() {
            s.push_str(&format!(
                "  Pot[{}]: Rp={:.1}Ω (in G matrix)\n",
                k,
                child.port_resistance(),
            ));
        }
        if let Some(ref opamp) = self.feedback_opamp {
            s.push_str(&format!("  feedback_opamp: gain={:.2}\n", opamp.gain()));
        }
        // Scattering sub-blocks (compact).
        let n_nl = self.n_nl;
        let n_passive = self.passive_one_ports.len();
        s.push_str(&format!("  S_nl[{}x{}]: [", n_nl, n_nl));
        for (i, v) in self.scattering.s_nl.iter().enumerate() {
            if i > 0 {
                s.push_str(", ");
            }
            s.push_str(&format!("{:+.4}", v));
        }
        s.push_str("]\n");
        if n_passive > 0 {
            s.push_str(&format!("  S_nl_passive[{}x{}]: [", n_nl, n_passive));
            for (i, v) in self.scattering.s_nl_passive.iter().enumerate() {
                if i > 0 {
                    s.push_str(", ");
                }
                s.push_str(&format!("{:+.4}", v));
            }
            s.push_str("]\n");
        }
        s.push_str("  S_nl_adapted: [");
        for (i, v) in self.scattering.s_nl_adapted.iter().enumerate() {
            if i > 0 {
                s.push_str(", ");
            }
            s.push_str(&format!("{:+.4}", v));
        }
        s.push_str("]\n");
        s
    }

    /// Set a pot value, searching through passive children.
    ///
    /// Handles Baxandall-decomposed pots: a 3-terminal pot is split into
    /// `{id}__aw` (wiper-to-a, tracks `value`) and `{id}__wb` (wiper-to-b,
    /// tracks `1 - value`).  Both halves are updated so the scattering
    /// matrix sees the correct resistance on each side of the wiper.
    ///
    /// When a pot is found and updated, recomputes the scattering matrix
    /// from the stored MNA data with updated port resistances.
    pub fn set_pot(&mut self, target_id: &str, value: crate::Wave) -> bool {
        let mut found = false;

        // Try base name. Native 3-terminal pots can contribute multiple
        // children with the same component id, so update every match.
        for child in &mut self.pot_children {
            if child.set_pot(target_id, value) {
                found = true;
            }
        }

        // Try explicitly decomposed halves.
        let aw = format!("{target_id}__aw");
        for child in &mut self.pot_children {
            if child.set_pot(&aw, value) {
                found = true;
            }
        }
        let wb = format!("{target_id}__wb");
        for child in &mut self.pot_children {
            if child.set_pot(&wb, 1.0 - value) {
                found = true;
            }
        }

        if found {
            self.recompute_pending = true;
        }
        found
    }

    /// Flush a pending scattering recompute.
    ///
    /// Called by the throttle in `advance_smoothers()` every N samples
    /// (typically 32) instead of on every pot change.  This reduces the
    /// O(n³) matrix inversion cost by ~32× during knob sweeps while
    /// keeping the pot resistance itself updated per-sample in the DynNode.
    #[inline]
    pub fn flush_recompute(&mut self) {
        if self.recompute_pending {
            self.recompute_pending = false;
            self.recompute_scattering();
        }
    }

    /// Set OTA gain for a linearized OTA stage.
    ///
    /// Delta-updates the stored MNA's conductance matrix to reflect the new gm,
    /// then marks the scattering matrix for recompute. The throttled recompute
    /// in `flush_recompute()` will re-derive the scattering matrix.
    ///
    /// gain: 0.0 = off, 1.0 = max transconductance.
    pub fn set_ota_gain_linear(&mut self, gain: crate::Wave) {
        let gain = gain.clamp(0.0, 1.0);
        if let Some(ref mut ota) = self.linearized_ota {
            let old_gm = ota.gain * ota.model.iabc_max / (2.0 * ota.model.vt);
            ota.gain = gain;
            let new_gm = gain * ota.model.iabc_max / (2.0 * ota.model.vt);
            let delta = new_gm - old_gm;

            if delta.abs() > 1e-15 {
                // Delta-update the stored MNA conductance matrix.
                if let Some(ref mut recompute) = self.recompute_data {
                    let n = ota.num_mna_nodes;
                    for &(row, col, sign) in &ota.stamp_cells {
                        recompute.mna.g_matrix[row * n + col] += sign * delta;
                    }
                }
                self.recompute_pending = true;
            }
        }
    }

    /// Check if this stage has a linearized OTA (for modulation target binding).
    pub fn has_linearized_ota(&self) -> bool {
        self.linearized_ota.is_some()
    }

    /// Set feedback coupling from a bias pot position.
    ///
    /// Updates `feedback_scale` and `veb_bias_offset` which are applied during
    /// `set_control_voltage()` on BJT NL devices.
    pub fn set_feedback_from_pot(&mut self, position: crate::Wave, max_pot_r: crate::Wave) {
        let pot_r = position * max_pot_r;
        let emitter_r = self.bias_emitter_r;
        self.feedback_scale = emitter_r / (emitter_r + pot_r);
        self.veb_bias_offset = self.feedback_scale * 0.1;
    }

    /// Update bias from pot by reading pot resistance from pot_children.
    ///
    /// Called after a pot change + recompute when `bias_pot_id` is set.
    /// Reads the pot's current rp from pot_children and updates
    /// feedback_scale + veb_bias_offset.
    pub fn update_bias_from_pot(&mut self) {
        if let Some(ref pot_id) = self.bias_pot_id {
            if let Some(pot_r) = self.get_pot_child_resistance(pot_id) {
                let emitter_r = self.bias_emitter_r;
                self.feedback_scale = emitter_r / (emitter_r + pot_r);
                self.veb_bias_offset = self.feedback_scale * 0.1;
            }
        }
    }

    /// Get the current resistance of a pot in pot_children by component ID.
    fn get_pot_child_resistance(&self, pot_id: &str) -> Option<crate::Wave> {
        for child in &self.pot_children {
            if let Some(r) = child.get_pot_resistance(pot_id) {
                return Some(r);
            }
        }
        None
    }

    /// Set the LED drive of a photocoupler LDR leaf held among this stage's
    /// MNA `pot_children` (a controlled G-matrix conductance, not a WDF port)
    /// and delta-update the scattering matrix so the new cell resistance takes
    /// effect on the FORWARD divider this sample. Returns `true` if a matching
    /// photocoupler was found.
    ///
    /// This is the MultiNlStage counterpart of `WdfStage::set_photocoupler_led`.
    /// Used by the LA-2A Phase-3 detector → LED gain-reduction coupling, where
    /// the T4B shunt cell compiles into the same MNA stage as the makeup tube
    /// V1 (stage label `C_in,...,PC1,...,V1`). The recompute is threshold-gated
    /// (the cell's resistance trajectory is band-limited by its own ms-to-s time
    /// constants) so audio-rate re-inversion is avoided.
    pub fn set_photocoupler_led(&mut self, comp_id: &str, led_drive: crate::Wave) -> bool {
        // Locate the photocoupler among pot_children and the variable-resistor
        // binding that maps it into the MNA G matrix (so we can read its
        // post-set resistance and threshold-gate the expensive recompute).
        let mut found = false;
        let mut child_hit: Option<usize> = None;
        for (ci, child) in self.pot_children.iter_mut().enumerate() {
            if child.set_photocoupler_led(comp_id, led_drive) {
                found = true;
                child_hit = Some(ci);
            }
        }
        if !found {
            return false;
        }
        // Threshold-gate the (matrix-re-inverting) recompute on a meaningful
        // relative resistance change vs the conductance last folded into the G
        // matrix. The cell's resistance trajectory is band-limited by its own
        // ms-to-s time constants, so its control content lives well below audio
        // rate — re-inverting the scattering matrix every sample is wasted work.
        let should_recompute = match child_hit {
            Some(ci) => {
                let new_r = self.pot_children[ci].port_resistance();
                if new_r.is_finite() && new_r > 0.0 {
                    let new_g = 1.0 / new_r;
                    // Compare against the matching binding's last-applied g.
                    match self.variable_resistors.iter().find(|ps| ps.child_idx == ci) {
                        Some(ps) => {
                            let last_g = ps.conductance;
                            (new_g - last_g).abs() > CTRL_R_RECOMPUTE_EPS * last_g.abs()
                        }
                        // No binding match — recompute to be safe.
                        None => true,
                    }
                } else {
                    false
                }
            }
            None => true,
        };
        if should_recompute {
            self.recompute_scattering();
        }
        found
    }

    /// Recompute the scattering matrix from stored MNA data after a pot change.
    ///
    /// Rebuilds WdfPort vec with current port resistances from nl_port_resistances
    /// and passive_one_ports, re-derives the scattering matrix, and updates all
    /// sub-blocks (s_nl, s_nl_passive, s_nl_adapted) and the RTypeAdaptor.
    fn recompute_scattering(&mut self) {
        // ── IIR recompute ────────────────────────────────────────────────
        // When IIR is active, read current pot resistances and recompute
        // the biquad coefficients. O(1) — no matrix inversion.
        if let Some(ref mut iir) = self.iir {
            // Update R_fb and R_series from pot children
            for &(child_idx, is_series, is_fb) in &iir.pot_map.clone() {
                if child_idx < self.pot_children.len() {
                    let pot_r = self.pot_children[child_idx].port_resistance();
                    if is_fb {
                        iir.r_fb = pot_r;
                        // R_crit depends on R_series, not R_fb — no update needed
                    }
                    if is_series {
                        // Tuning pot adds to R1 — recompute R products and R_crit
                        let r1 = iir.r_series_base[0] + pot_r;
                        let r2 = iir.r_series_base[1];
                        iir.r_series_product = r1 * r2;
                        iir.r_crit = r1 + r2 + r1 * iir.c_shunt_base[0] / iir.c_shunt_base[1];
                    }
                }
            }
            iir.recompute();
            return;
        }

        // ── State-space recompute ────────────────────────────────────────
        // When state-space is active, rebuild A_d, b_d from the updated MNA
        // (G matrix has been delta-updated by set_ota_gain_linear or set_pot).
        if let Some(ref mut ss) = self.state_space {
            if let Some(ref mut recompute) = self.recompute_data {
                // Delta-update pot conductances in the MNA G matrix.
                let n_mna = recompute.mna.num_nodes;
                for ps in &mut ss.variable_resistors {
                    let (pos, neg) = ps.terminals.raw().as_tuple();
                    let new_r = self.pot_children[ps.child_idx].port_resistance();
                    let new_g = 1.0 / new_r;
                    let delta = new_g - ps.conductance;
                    if delta.abs() > 1e-15 {
                        // Delta-update G: remove old conductance, add new.
                        if let Some(p) = pos {
                            recompute.mna.g_matrix[p * n_mna + p] += delta;
                            if let Some(n) = neg {
                                recompute.mna.g_matrix[p * n_mna + n] -= delta;
                            }
                        }
                        if let Some(n) = neg {
                            recompute.mna.g_matrix[n * n_mna + n] += delta;
                            if let Some(p) = pos {
                                recompute.mna.g_matrix[n * n_mna + p] -= delta;
                            }
                        }
                        ps.conductance = new_g;
                    }
                }
                let (a_d, b_d, c_out, _n, d_ft) = recompute.mna.build_state_space_matrices(
                    &ss.reactive_one_ports,
                    ss.vs_idx,
                    ss.output_pos,
                    ss.output_neg,
                    ss.sample_rate,
                );
                if a_d.iter().all(|v| v.is_finite()) && b_d.iter().all(|v| v.is_finite()) {
                    ss.a_matrix = a_d;
                    ss.b_vector = b_d;
                    ss.c_vector = c_out;
                    ss.d_feedthrough = d_ft;
                }
            }
            return;
        }

        // Fast path: interpolation table lookup for single-pot stages
        if let Some(ref table) = self.interp_table {
            if self.variable_resistors.len() == 1 {
                let pot_r =
                    self.pot_children[self.variable_resistors[0].child_idx].port_resistance();
                let (new_scat, new_vs) = table.lookup(pot_r);

                if new_scat.iter().all(|&s| s.is_finite()) {
                    let n_nl = self.n_nl;
                    let n_passive = self.passive_one_ports.len();
                    self.scattering =
                        MultiNlScattering::from_full_matrix(&new_scat, n_nl, n_passive);

                    // Re-extract dc_bias and vcc_bias_all from VCC injection vector
                    if self.vcc_vs_index.is_some() {
                        // The interp table's vs_injection is the VCC injection vector
                        for (dc, &vs) in self.dc_bias[..n_nl]
                            .iter_mut()
                            .zip(new_vs.iter())
                            .take(n_nl.min(new_vs.len()))
                        {
                            *dc = vs * self.supply_voltage;
                        }
                        self.vcc_bias_all =
                            new_vs.iter().map(|&k| k * self.supply_voltage).collect();
                    }

                    // Rebuild port_resistances for RTypeAdaptor
                    let mut port_resistances = self.nl_port_resistances.clone();
                    for one_port in &self.passive_one_ports {
                        port_resistances.push(one_port.rp(self.passive_sample_rate));
                    }
                    // Add adapted port resistance if not VS mode
                    if self.vs_injection.is_none() {
                        if let Some(ref recompute) = self.recompute_data {
                            port_resistances.push(recompute.adapted_resistance);
                        }
                    }
                    self.adaptor = RTypeAdaptor::new(new_scat.clone(), &port_resistances);

                    if self.vs_injection.is_some() {
                        self.vs_injection = Some(new_vs);
                    }

                    // Recompute extraction coefficients from table if needed
                    if let Some(ref recompute) = self.recompute_data {
                        if let Some(out) = recompute.extract_output_nodes {
                            let (out_pos, out_neg) = out.as_tuple();
                            let _ = (out_pos, out_neg);
                        }
                    }
                }
                return;
            }
        }

        let recompute = match &mut self.recompute_data {
            Some(r) => r,
            None => return,
        };

        // Delta-update pot conductances in the MNA G matrix.
        let n_mna = recompute.mna.num_nodes;
        for ps in &mut self.variable_resistors {
            let child_idx = ps.child_idx;
            let (pos, neg) = ps.terminals.raw().as_tuple();
            let new_r = self.pot_children[child_idx].port_resistance();
            let new_g = 1.0 / new_r;
            let delta = new_g - ps.conductance;
            if delta.abs() > 1e-15 {
                if let Some(p) = pos {
                    recompute.mna.g_matrix[p * n_mna + p] += delta;
                    if let Some(n) = neg {
                        recompute.mna.g_matrix[p * n_mna + n] -= delta;
                    }
                }
                if let Some(n) = neg {
                    recompute.mna.g_matrix[n * n_mna + n] += delta;
                    if let Some(p) = pos {
                        recompute.mna.g_matrix[n * n_mna + p] -= delta;
                    }
                }
                ps.conductance = new_g;
            }
        }

        let n_nl = self.n_nl;
        let n_passive = self.passive_one_ports.len();
        let use_vs = recompute.vs_source_index.is_some();
        let _has_vcc_vs = recompute.vcc_vs_index.is_some();
        let n_total = if use_vs {
            n_nl + n_passive
        } else {
            n_nl + n_passive + 1
        };

        // Rebuild ports with current resistances (reactive elements only, no pots).
        let mut ports: Vec<WdfPort> = Vec::with_capacity(n_total);

        // NL ports (resistances don't change)
        for i in 0..n_nl {
            ports.push(recompute.port_node_pairs[i].to_wdf_port(self.nl_port_resistances[i]));
        }

        // Passive ports (caps/inductors — resistances are fixed for a given sample rate)
        for k in 0..n_passive {
            let rp = self.passive_one_ports[k].rp(self.passive_sample_rate);
            ports.push(recompute.port_node_pairs[n_nl + k].to_wdf_port(rp));
        }

        if use_vs {
            // VS injection mode (OTA): derive scattering + injection vector (no adapted port).
            let vs_idx = recompute.vs_source_index.unwrap();
            let (scattering, vs_inj) = recompute
                .mna
                .derive_scattering_and_vs_injection(&ports, vs_idx);

            if scattering.iter().any(|&s| !s.is_finite()) {
                return;
            }

            self.scattering = MultiNlScattering::from_full_matrix(&scattering, n_nl, n_passive);
            let port_resistances: Vec<crate::Wave> = ports.iter().map(|p| p.resistance).collect();
            self.adaptor = RTypeAdaptor::new(scattering, &port_resistances);
            self.vs_injection = Some(vs_inj);

            // Recompute extraction coefficients if used.
            if let Some(out) = recompute.extract_output_nodes {
                let (out_pos, out_neg) = out.as_tuple();
                let (coeffs, vs_coeff) = recompute
                    .mna
                    .derive_extraction_coeffs(&ports, vs_idx, out_pos, out_neg);
                self.extract_coeffs = Some(coeffs);
                self.extract_vs = vs_coeff;
            }
        } else {
            // Standard mode: adapted port included (always last).
            let adapted_pair_idx = n_nl + n_passive;
            ports.push(
                recompute.port_node_pairs[adapted_pair_idx]
                    .to_wdf_port(recompute.adapted_resistance),
            );

            if let Some(vcc_idx) = recompute.vcc_vs_index {
                // VCC as ideal VS: derive scattering + VCC injection vector
                let (scattering, vcc_inj) = recompute
                    .mna
                    .derive_scattering_and_vs_injection(&ports, vcc_idx);

                if scattering.iter().any(|&s| !s.is_finite()) {
                    return;
                }

                // Re-extract dc_bias and vcc_bias_all from VCC injection vector
                for (dc, &inj) in self.dc_bias[..n_nl]
                    .iter_mut()
                    .zip(vcc_inj.iter())
                    .take(n_nl.min(vcc_inj.len()))
                {
                    *dc = inj * self.supply_voltage;
                }
                self.vcc_bias_all = vcc_inj.iter().map(|&k| k * self.supply_voltage).collect();

                self.scattering = MultiNlScattering::from_full_matrix(&scattering, n_nl, n_passive);
                let port_resistances: Vec<crate::Wave> =
                    ports.iter().map(|p| p.resistance).collect();
                self.adaptor = RTypeAdaptor::new(scattering, &port_resistances);

                if let Some(out) = recompute.extract_output_nodes {
                    let (out_pos, out_neg) = out.as_tuple();
                    self.extract_coeffs = Some(
                        recompute
                            .mna
                            .derive_node_extraction_coeffs(&ports, out_pos, out_neg),
                    );
                    self.extract_vs = 0.0;
                }
            } else {
                // No VS at all: standard scattering derivation
                let scattering = recompute.mna.derive_scattering_matrix_general(&ports);

                if scattering.iter().any(|&s| !s.is_finite()) {
                    return;
                }

                self.scattering = MultiNlScattering::from_full_matrix(&scattering, n_nl, n_passive);
                let port_resistances: Vec<crate::Wave> =
                    ports.iter().map(|p| p.resistance).collect();
                self.adaptor = RTypeAdaptor::new(scattering, &port_resistances);

                if let Some(out) = recompute.extract_output_nodes {
                    let (out_pos, out_neg) = out.as_tuple();
                    self.extract_coeffs = Some(
                        recompute
                            .mna
                            .derive_node_extraction_coeffs(&ports, out_pos, out_neg),
                    );
                    self.extract_vs = 0.0;
                }
            }
        }

        // A recompute just re-extracted the LINEAR vcc-injection `dc_bias` AND
        // `vcc_bias_all` against the freshly-recomputed scattering.  Re-establish
        // the nonlinear DC Q-point seed so the feedback-servo operating point is
        // preserved across pot moves.
        if self.dc_qpoint_joint {
            // Generic joint path (bead b2ps): the scattering changed, so the cap
            // fixed point AND the inverted dc_bias must be recomputed against the
            // NEW scattering.  Re-run the full joint solve from the (scattering-
            // invariant) nodal operating point v*.
            if let Some(v_seed) = self.dc_qpoint_v.clone() {
                // Reset the joint flag so the re-solve starts from the freshly
                // rebuilt linear dc_bias/vcc_bias_all, then re-derives consistently.
                self.dc_qpoint_joint = false;
                let _ = self.solve_joint_dc_qpoint(&v_seed);
            }
        } else {
            self.apply_dc_qpoint_seed();
        }
    }

    /// Re-derive `dc_bias` (and `vcc_bias_all`) from the nonlinear DC Q-point seed
    /// `dc_qpoint_v` via the WDF residual inversion, using the CURRENT scattering
    /// and port resistances.  No-op when no seed is present.
    ///
    /// The runtime grouped-NR residual at a port is
    /// `F_i = known_a_i − v_i − R_i·i_i(v) + Σ_j S[i][j]·(v_j − R_j·i_j(v))`, and
    /// at DC steady state `known_a_i = dc_bias[i]`.  Solving for the incident-wave
    /// DC term that makes the NR settle at the seeded operating point `v*` gives
    /// `dc_bias[i] = v_i* + R_i·i_i(v*) − Σ_j S[i][j]·(v_j* − R_j·i_j(v*))`.
    pub fn apply_dc_qpoint_seed(&mut self) {
        let v_star = match self.dc_qpoint_v.clone() {
            Some(v) => v,
            None => return,
        };
        let n_nl = self.dc_bias.len();
        if v_star.len() != n_nl || n_nl == 0 {
            return;
        }
        let dg = match self.device_groups {
            Some(ref dg) => dg,
            None => return,
        };

        // Device port currents i_i(v*) using the same grouped models the NR uses.
        let mut i_op = vec![0.0 as crate::Wave; n_nl];
        let mut cur = [0.0 as crate::Wave; 3];
        let mut jac = [0.0 as crate::Wave; 9];
        for (g, group) in dg.groups.iter().enumerate() {
            let np = group.n_ports().min(3);
            let off = dg.offsets.get(g).copied().unwrap_or(0);
            if off + np > n_nl {
                continue;
            }
            group
                .as_group_iv()
                .eval(&v_star[off..off + np], &mut cur[..np], &mut jac[..np * np]);
            for p in 0..np {
                i_op[off + p] = cur[p];
            }
        }

        let r = &self.nl_port_resistances;
        let s = &self.scattering.s_nl;
        let has_s = s.len() == n_nl * n_nl;
        // Passive-port DC contribution: at DC steady state `b_passive[k]` settles
        // to the cap node voltage, contributing `Σ_k s_nl_passive[i][k]·b[k]` to
        // `known_a_i`.  Subtract it so `dc_bias[i]` is the residual DC source term.
        let n_passive = self.dc_qpoint_passive_b.len();
        let s_pass = &self.scattering.s_nl_passive;
        let has_pass = n_passive > 0 && s_pass.len() >= n_nl * n_passive;
        for i in 0..n_nl {
            let ri = r.get(i).copied().unwrap_or(1.0);
            let mut bias = v_star[i] + ri * i_op[i];
            if has_s {
                for j in 0..n_nl {
                    let rj = r.get(j).copied().unwrap_or(1.0);
                    bias -= s[i * n_nl + j] * (v_star[j] - rj * i_op[j]);
                }
            }
            if has_pass {
                for k in 0..n_passive {
                    bias -= s_pass[i * n_passive + k] * self.dc_qpoint_passive_b[k];
                }
            }
            if !bias.is_finite() {
                continue;
            }
            self.dc_bias[i] = bias;
            if i < self.vcc_bias_all.len() {
                self.vcc_bias_all[i] = bias;
            }
        }
    }

    /// Generic joint NL-port + reactive-port DC equilibrium solver (bead b2ps).
    ///
    /// Runs the **stage's own runtime DC step** (`input = 0`, `dc_scale = 1`) to a
    /// joint fixed point of (a) the grouped Newton on the NL ports and (b) the
    /// reactive (capacitor/inductor) ports' steady state, using the exact same
    /// scattering / port-resistances / `dc_bias` / `vcc_bias_all` / `scatter_all`
    /// the audio loop uses.  This is device-agnostic: it iterates over the group's
    /// `&dyn NlDeviceGroupIv` (triode, BJT, FET, …) through the runtime solver, so
    /// the seed it produces is a fixed point of the *exact* system the runtime
    /// iterates — stable by construction (no separate Newton that the runtime then
    /// drifts away from).
    ///
    /// `v_seed` is an initial warm-start at the active operating point (e.g. from
    /// the nonlinear nodal DC solve).  The method:
    ///   1. iterates the full DC step until both `v` and the reactive reflected
    ///      waves `b_passive` stop changing (the joint fixed point), and
    ///   2. on success, writes the converged `v` into the NR warm-start
    ///      (`v_prev` / `v_prev_2` / `initial_v_prev`) and pre-charges every
    ///      reactive one-port to its fixed-point incident wave so the very first
    ///      audio sample starts *at* the equilibrium and stays there.
    ///
    /// Returns `Some(realized_dc_gain_proxy)` if a stable joint fixed point was
    /// found, else `None` (caller leaves the linear baseline untouched).  The
    /// reactive fixed point uses each cap's unit-delay relation `b[n] = a[n-1]`:
    /// at DC steady state `b_passive[k] = a_passive[k]`, which the relaxation
    /// drives to consistency under the stage's *own* scattering — the deep-WDF
    /// piece, solved once here for any device family.
    pub fn solve_joint_dc_qpoint(&mut self, v_seed: &[crate::Wave]) -> Option<crate::Wave> {
        let n_nl = self.dc_bias.len();
        if n_nl == 0 || v_seed.len() != n_nl {
            return None;
        }
        let dg_present = self.device_groups.is_some();
        if !dg_present {
            return None;
        }
        let n_passive = self.passive_one_ports.len();
        let use_vs_injection = self.vs_injection.is_some();
        let n_total = n_nl + n_passive + if use_vs_injection { 0 } else { 1 };
        let s_nl = &self.scattering.s_nl;
        if s_nl.len() != n_nl * n_nl {
            return None;
        }
        let has_pass = n_passive > 0 && self.scattering.s_nl_passive.len() >= n_nl * n_passive;

        let debug = std::env::var("PK_JOINTDC_DEBUG").is_ok();

        // The operating point v* (from the nonlinear nodal DC solve) is HELD
        // FIXED — it is the physically correct Q-point.  We solve for the two DC
        // source terms that make v* a self-consistent fixed point of the runtime
        // step *together with* the reactive ports:
        //   • the reactive reflected waves `b_passive[k]` (the cap DC charge), via
        //     the cap steady-state fixed point `b_passive = a_passive` under the
        //     stage's OWN scattering (with b_nl held at v*), and
        //   • `dc_bias[i]`, re-inverted each iteration so the grouped NR keeps v*
        //     as its root given the *current* `b_passive` coupling.
        // Because both the NR source and the cap charge are derived from the SAME
        // scattering the audio loop uses, the seed is a true joint fixed point of
        // the runtime step → no drift.  Holding v* fixed (rather than letting the
        // wave-domain NR relax freely) is essential: the linear `dc_bias`/`vcc`
        // superposition admits a *spurious* wave-domain fixed point away from the
        // true nodal Q-point, which is the decaying mode the free relaxation found.
        let v = v_seed.to_vec();

        // Device port currents i_i(v*) (v-only; independent of the DC sources).
        let mut i_op = vec![0.0 as crate::Wave; n_nl];
        {
            let dg = self.device_groups.as_ref().unwrap();
            let mut cur = [0.0 as crate::Wave; 3];
            let mut jac = [0.0 as crate::Wave; 9];
            for (g, group) in dg.groups.iter().enumerate() {
                let np = group.n_ports().min(3);
                let off = dg.offsets.get(g).copied().unwrap_or(0);
                if off + np > n_nl {
                    continue;
                }
                group
                    .as_group_iv()
                    .eval(&v[off..off + np], &mut cur[..np], &mut jac[..np * np]);
                for p in 0..np {
                    i_op[off + p] = cur[p];
                }
            }
        }
        // Device b-wave at v*: b_nl[i] = v[i] - R_i·i_i(v*)  …  PLUS the part of
        // the WDF reflection that comes from the incident a.  In a WDF port,
        // b = 2v - a and i = (a - b)/(2R) ⇒ b = a - 2R·i and v = (a+b)/2.  With v
        // and i fixed, b = v - R·i + (v - R·i) - … — eliminate a: b = v - R·i is
        // the *device-determined* half; the incident-dependent half is folded into
        // the scatter below since a_nl = scatter(...)[i] depends on b_passive.  For
        // the cap fixed point we need b_nl as a function of v* only, which is the
        // companion form b_nl[i] = v[i] - R_i·i_i(v*) (the reflected wave a port
        // with a fixed v,i presents to the adaptor).
        let mut b_nl = vec![0.0 as crate::Wave; n_nl];
        for i in 0..n_nl {
            let ri = self.nl_port_resistances.get(i).copied().unwrap_or(1.0);
            b_nl[i] = v[i] - ri * i_op[i];
        }

        let mut b_passive = vec![0.0 as crate::Wave; n_passive];
        let mut b_all = vec![0.0 as crate::Wave; n_total];
        let mut a_all = vec![0.0 as crate::Wave; n_total];

        // Cap fixed-point relaxation: b_passive = a_passive (cap unit-delay at DC).
        // a_passive = scatter([b_nl, b_passive, 0])[n_nl+k] + vcc_bias_all[n_nl+k].
        // The passive↔passive scattering sub-block is a contraction (passive net),
        // so the simple iteration converges; under-relax for robustness.
        let relax: crate::Wave = 0.7;
        let tol: crate::Wave = 1e-10;
        let max_outer = 20000usize;
        let mut converged = n_passive == 0;
        let mut prev_metric = crate::Wave::INFINITY;
        for outer in 0..max_outer {
            if n_passive == 0 {
                converged = true;
                break;
            }
            b_all[..n_nl].copy_from_slice(&b_nl);
            b_all[n_nl..n_nl + n_passive].copy_from_slice(&b_passive);
            if !use_vs_injection {
                b_all[n_nl + n_passive] = 0.0;
            }
            self.adaptor.scatter_all_into(&b_all, &mut a_all);
            if !self.vcc_bias_all.is_empty() {
                for i in 0..a_all.len().min(self.vcc_bias_all.len()) {
                    a_all[i] += self.vcc_bias_all[i];
                }
            }
            let mut max_db = 0.0 as crate::Wave;
            for k in 0..n_passive {
                let target = a_all[n_nl + k];
                let nb = b_passive[k] + relax * (target - b_passive[k]);
                max_db = max_db.max((nb - b_passive[k]).abs());
                b_passive[k] = nb;
            }
            if debug && (outer < 4 || outer % 1024 == 0) {
                std::eprintln!(
                    "[PK_JOINTDC] cap-relax outer={outer} max_db={max_db:.3e} b_passive={:?}",
                    &b_passive
                );
            }
            if max_db < tol {
                converged = true;
                break;
            }
            if !max_db.is_finite() || (outer > 64 && max_db > prev_metric * 4.0 + 1.0) {
                break;
            }
            prev_metric = max_db;
        }

        if !converged || !b_passive.iter().all(|x| x.is_finite()) {
            if debug {
                std::eprintln!("[PK_JOINTDC] cap fixed point NOT converged");
            }
            return None;
        }

        // Now invert dc_bias so the grouped NR holds v* given this cap charge:
        //   dc_bias[i] = v*[i] + R_i·i_i(v*) − Σ_j S[i][j]·(v*[j] − R_j·i_j(v*))
        //                − Σ_k S_passive[i][k]·b_passive[k]
        // (the runtime NR residual is zero at v* with known_a = this dc_bias plus
        //  the passive coupling Σ_k S_passive[i][k]·b_passive[k]).
        let mut new_dc_bias = vec![0.0 as crate::Wave; n_nl];
        for i in 0..n_nl {
            let ri = self.nl_port_resistances.get(i).copied().unwrap_or(1.0);
            let mut bias = v[i] + ri * i_op[i];
            for j in 0..n_nl {
                let rj = self.nl_port_resistances.get(j).copied().unwrap_or(1.0);
                bias -= s_nl[i * n_nl + j] * (v[j] - rj * i_op[j]);
            }
            if has_pass {
                let row = &self.scattering.s_nl_passive[i * n_passive..i * n_passive + n_passive];
                for (k, &s) in row.iter().enumerate() {
                    bias -= s * b_passive[k];
                }
            }
            if !bias.is_finite() {
                return None;
            }
            new_dc_bias[i] = bias;
        }
        // Commit the inverted NR source.  Leave `vcc_bias_all` for the NL ports
        // untouched (those entries only feed output extraction, not the loop); the
        // passive-port `vcc_bias_all` entries are the DC rail injection the cap
        // fixed point already accounts for, so they stay as built.
        for i in 0..n_nl {
            self.dc_bias[i] = new_dc_bias[i];
        }

        if debug {
            std::eprintln!(
                "[PK_JOINTDC] CONVERGED (v* held) v={:?} b_passive={:?} dc_bias={:?}",
                &v, &b_passive, &new_dc_bias
            );
        }

        // ── Commit the joint fixed point as the runtime seed. ────────────────
        // 1. NR warm-start at the converged operating point.
        for i in 0..n_nl {
            let w = v[i];
            if i < self.v_prev.len() {
                self.v_prev[i] = w;
            }
            if i < self.initial_v_prev.len() {
                self.initial_v_prev[i] = w;
            }
            if i < self.v_prev_2.len() {
                self.v_prev_2[i] = w;
            }
        }
        // 2. Pre-charge each reactive one-port to its fixed-point incident wave so
        //    the first sample's `wdf_reflected` returns exactly b_passive[k] and the
        //    cap does not drift.  `wdf_set_incident` stores wave_state = incident and
        //    `wdf_reflected` returns wave_state ⇒ first sample sees b_passive[k].
        for (k, &one_port) in self.passive_one_ports.iter().enumerate() {
            if let Some(&bk) = b_passive.get(k) {
                one_port.wdf_set_incident(bk, &mut self.passive_runtime_state);
            }
        }
        // 3. Record the seed so reset() can re-charge the caps + restore the
        //    warm-start.  `dc_qpoint_joint` tells reset() that `dc_bias` is ALREADY
        //    the consistent joint inversion — it must NOT call apply_dc_qpoint_seed
        //    (which would re-derive a different dc_bias and clobber vcc_bias_all).
        self.dc_qpoint_v = Some(v.clone());
        self.dc_qpoint_passive_b = b_passive.clone();
        self.dc_qpoint_joint = true;

        // Skip the DC ramp — the operating point is already at equilibrium.
        self.dc_ramp = 256;
        self.initial_dc_ramp = 256;

        // Self-check: run ONE runtime-style grouped NR from v* with the inverted
        // dc_bias + the cap charge, exactly as sample 0 will, and report the
        // residual + where it lands.  If it stays at v*, the seed is a true fixed
        // point of the per-sample solver; if it walks off, the per-sample NR (with
        // its iteration budget + step clamp) cannot hold v* → the loop is unstable.
        if debug {
            let mut v_chk = v.clone();
            let mut known_a = vec![0.0 as crate::Wave; n_nl];
            for i in 0..n_nl {
                let mut a_i = self.dc_bias[i];
                if has_pass {
                    let row =
                        &self.scattering.s_nl_passive[i * n_passive..i * n_passive + n_passive];
                    for (k, &s) in row.iter().enumerate() {
                        a_i += s * b_passive[k];
                    }
                }
                known_a[i] = a_i;
            }
            let dg = self.device_groups.as_ref().unwrap();
            let mgp = dg.groups.iter().map(|g| g.n_ports()).max().unwrap_or(1);
            let mut ws_chk =
                crate::elements::nonlinear::solver::NrWorkspace::new_grouped(n_nl, mgp);
            let groups: alloc::vec::Vec<&dyn NlDeviceGroupIv> =
                dg.groups.iter().map(|g| g.as_group_iv()).collect();
            crate::elements::nonlinear::solver::multi_port_nr_solve_grouped_into(
                n_nl,
                s_nl,
                &known_a,
                &self.nl_port_resistances,
                &groups,
                &dg.offsets,
                &mut v_chk,
                64,
                1e-12,
                &mut ws_chk,
            );
            let mut walk = 0.0 as crate::Wave;
            for i in 0..n_nl {
                walk = walk.max((v_chk[i] - v[i]).abs());
            }
            std::eprintln!(
                "[PK_JOINTDC] self-check: per-sample NR from v* landed at {:?} (max walk-off={walk:.3e})",
                &v_chk
            );
            // Cap-consistency check: build b_all from the RUNTIME b_nl (ws_chk.b_nl
            // at v*) and verify scatter+vcc gives back b_passive at the cap ports.
            let mut bchk = vec![0.0 as crate::Wave; n_total];
            let mut achk = vec![0.0 as crate::Wave; n_total];
            bchk[..n_nl].copy_from_slice(&ws_chk.b_nl[..n_nl]);
            bchk[n_nl..n_nl + n_passive].copy_from_slice(&b_passive);
            if !use_vs_injection {
                bchk[n_nl + n_passive] = 0.0;
            }
            self.adaptor.scatter_all_into(&bchk, &mut achk);
            if let Some(ref kvec) = self.vs_injection {
                for i in 0..achk.len().min(kvec.len()) {
                    achk[i] += kvec[i] * 0.0;
                }
            }
            if !self.vcc_bias_all.is_empty() {
                for i in 0..achk.len().min(self.vcc_bias_all.len()) {
                    achk[i] += self.vcc_bias_all[i];
                }
            }
            let mut cap_err = 0.0 as crate::Wave;
            for k in 0..n_passive {
                cap_err = cap_err.max((achk[n_nl + k] - b_passive[k]).abs());
            }
            std::eprintln!(
                "[PK_JOINTDC] cap-consistency (runtime b_nl): a_passive={:?} vs b_passive={:?} max_err={cap_err:.3e}; my_b_nl={:?} runtime_b_nl={:?}",
                &achk[n_nl..n_nl + n_passive], &b_passive, &b_nl, &ws_chk.b_nl[..n_nl]
            );
        }

        // Proxy for "did it land active": sum |b_nl| as a coarse activity metric.
        let activity: crate::Wave = b_nl.iter().map(|x| x.abs()).sum();
        Some(activity)
    }

    // ── Precompute accessors ─────────────────────────────────────────────
    // Minimal read-only accessors for `precompute::extract_precomputed`.

    /// The R-type adaptor containing the scattering matrix and port data.
    pub fn adaptor(&self) -> &RTypeAdaptor {
        &self.adaptor
    }

    /// VS injection vector (present when driven by an ideal voltage source).
    pub fn vs_injection(&self) -> Option<&Vec<crate::Wave>> {
        self.vs_injection.as_ref()
    }

    /// Node-voltage extraction coefficients (present when output is read from
    /// MNA node voltages directly rather than WDF port waves).
    pub fn extract_coeffs(&self) -> Option<&Vec<crate::Wave>> {
        self.extract_coeffs.as_ref()
    }

    /// VS component of the extraction formula.
    pub fn extract_vs(&self) -> crate::Wave {
        self.extract_vs
    }

    /// Pot interpolation table (present for single-pot stages).
    pub fn interp_table(&self) -> Option<&crate::tree::ScatteringInterpolationTable> {
        self.interp_table.as_ref()
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Sidechain processor — feedback compression loop
// ═══════════════════════════════════════════════════════════════════════════

/// Sidechain processor — a sub-circuit compiled through the same WDF
/// pipeline as the main audio path.
///
/// The sidechain is extracted from the parent PedalDef as a separate
/// sub-PedalDef containing its own components, nets, controls, and supplies.
/// It is compiled via `compile_pedal()` into a full `CompiledPedal`, giving
/// it proper WDF trees, transformers, nonlinear elements, and passives —
/// exactly like the main circuit.
///
/// At runtime, the sidechain processes the tapped audio and produces an
/// output (CV) that modulates the main circuit's push-pull grid bias
/// with a 1-sample feedback delay.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SidechainProcessor {
    /// The compiled sidechain sub-circuit.
    pub circuit: crate::processor::CompiledPedal,
    /// 1-sample delay state for the feedback loop CV.
    pub cv_delayed: crate::Wave,
}

impl SidechainProcessor {
    /// Process one sample through the sidechain sub-circuit.
    #[inline]
    pub fn process(&mut self, tapped_signal: crate::Wave) -> crate::Wave {
        self.circuit.process(tapped_signal as Wave) as crate::Wave
    }

    /// Forward a control change to the sidechain sub-circuit.
    pub fn set_control(&mut self, label: &str, value: crate::Wave) {
        self.circuit.set_control(label, value);
    }

    /// Reset the sidechain state.
    pub fn reset(&mut self) {
        self.cv_delayed = 0.0;
        self.circuit.reset();
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SubcircuitProcessor
// ═══════════════════════════════════════════════════════════════════════════

/// A compiled subcircuit with its own [`CompiledPedal`] and optional rate domain.
///
/// Subcircuits partition an equipment definition into named sub-networks that
/// are each compiled via the full WDF pipeline.  Rate-reduced subcircuits
/// (e.g. `rate: 1/64` sidechain detectors) process one sample every
/// `rate_divisor` input samples and use linear interpolation to fill gaps.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SubcircuitProcessor {
    /// The compiled sub-circuit.
    pub circuit: crate::processor::CompiledPedal,
    /// Subcircuit name (for debug and control routing).
    #[allow(dead_code)]
    pub name: String,
    /// Decimation factor (1 = full rate, 64 = process every 64th sample).
    pub rate_divisor: u32,
    /// Countdown to next processing tick (decrements each sample, resets to `rate_divisor`).
    pub rate_counter: u32,
    /// Last computed output value (held between decimated samples).
    pub held_output: crate::Wave,
    /// Output from one `rate_divisor` period ago (for linear interpolation).
    pub prev_output: crate::Wave,
}

impl SubcircuitProcessor {
    /// Process one input sample.
    ///
    /// For full-rate subcircuits (`rate_divisor == 1`) this is a direct call to
    /// the inner [`CompiledPedal::process`].  For rate-reduced subcircuits the
    /// inner circuit is only called every `rate_divisor` samples; between calls
    /// the output is linearly interpolated from the previous to the current value.
    #[inline]
    pub fn process(&mut self, input: crate::Wave) -> crate::Wave {
        if self.rate_divisor <= 1 {
            self.held_output = self.circuit.process(input as Wave) as crate::Wave;
            return self.held_output;
        }

        self.rate_counter -= 1;
        if self.rate_counter == 0 {
            self.rate_counter = self.rate_divisor;
            self.prev_output = self.held_output;
            self.held_output = self.circuit.process(input as Wave) as crate::Wave;
        }

        // Linear interpolation between prev_output and held_output.
        // t = 1.0 when we just computed a new value, 0.0 when counter is back to rate_divisor.
        let t = 1.0 - (self.rate_counter as crate::Wave / self.rate_divisor as crate::Wave);
        self.prev_output + t * (self.held_output - self.prev_output)
    }

    /// Forward a control change to the subcircuit's inner processor.
    pub fn set_control(&mut self, label: &str, value: crate::Wave) {
        self.circuit.set_control(label, value);
    }

    /// Reset subcircuit state (e.g., between songs / on silence detection).
    #[allow(dead_code)]
    pub fn reset(&mut self) {
        self.held_output = 0.0;
        self.prev_output = 0.0;
        self.rate_counter = self.rate_divisor;
        self.circuit.reset();
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Blockwise K-method stage: wave-domain Newton for coupled NL blocks
// ═══════════════════════════════════════════════════════════════════════════

/// A single block in the blockwise K-method stage.
///
/// Wraps a WDF tree + K-table as a "composite NL device." The tree contains
/// the block's local passives (emitter resistor, bypass cap, cutoff pot) and
/// a VS leaf for signal injection. The K-table encodes the nonlinear
/// element's (BJT) response, precomputed at compile time.
#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct KMethodBlock {
    /// WDF tree: VS + passives (R_e_floor, C, Cutoff pot).
    pub tree: DynNode,
    /// Shared physical one-port state for this block's WDF tree.
    #[cfg_attr(feature = "serde", serde(default))]
    pub runtime_state: RuntimeState,
    /// Precomputed K-table: b_tree × ctrl → a_root.
    pub k_table: KTable,
    /// Direct explicit diode root for one-port diode ladders.
    ///
    /// Diode ladder cutoff can move by changing the block source impedance.
    /// K-tables are generated for one fixed tree Rp, so diode BKM blocks keep
    /// the explicit root and bypass the table when Rp is modulated.
    pub explicit_diode_root: Option<ExplicitDiodeRoot>,
    /// Nominal voltage-source resistance used when the block was compiled.
    pub nominal_vs_rp: crate::Wave,
    /// Circuit-derived cutoff calibration for explicit diode ladders.
    pub diode_cutoff: Option<DiodeCutoffCalibration>,
    /// Port resistance of this block's WDF tree root.
    pub rp: crate::Wave,
    /// DC bias voltage for the BJT (Vbe operating point).
    pub vbe_bias: crate::Wave,
    /// DC operating point tracked at runtime.
    pub dc_offset: crate::Wave,
    /// If true, cascade output is from the passive subtree (right child
    /// of Series adaptor = cap voltage). If false, from root port.
    /// Set by compiler from circuit graph topology.
    pub cascade_from_passive: bool,
    /// Component ID to probe for cascade output, usually the rung shunt cap.
    /// This is more robust than reconstructing voltage from adaptor internals.
    pub cascade_probe_id: Option<alloc::string::String>,
    /// Voltage-source polarity needed to drive this block's WDF tree.
    ///
    /// This mirrors `WdfStage::process()` root/topology sign handling. BKM
    /// computes physical block drive voltages in the coupling/cascade domain,
    /// then maps them into the WDF tree's voltage-source convention here.
    pub source_polarity: crate::Wave,
    /// Polarity used for K-table control axes such as bias voltage.
    ///
    /// This is intentionally separate from `source_polarity`: the WDF tree may
    /// need a sign flip for its voltage-source convention, while a K-table
    /// bias axis is defined in the physical device voltage convention.
    #[cfg_attr(feature = "serde", serde(default = "default_k_table_control_polarity"))]
    pub k_table_control_polarity: crate::Wave,
    /// Optional shared diode bias voltage for current-controlled diode ladders.
    ///
    /// TB-303-style ladders use one cutoff current to set all rung operating
    /// points coherently. When present, this voltage drives the K-table control
    /// axis instead of the block's local coupling voltage.
    #[cfg_attr(feature = "serde", serde(default))]
    pub shared_diode_bias_voltage: Option<crate::Wave>,
    /// Whether the K-table control axis should be driven from this block's
    /// physical input voltage.
    ///
    /// Ordinary 2D roots such as BJT/triode use the driven terminal voltage as
    /// their device-control coordinate. Differential ladder rungs do not: their
    /// second table axis is shared tail-current modulation, which comes from
    /// the cutoff network rather than the audio/bias drive at the rung port.
    #[cfg_attr(feature = "serde", serde(default = "default_control_from_drive"))]
    pub control_from_drive: bool,
}

#[cfg(feature = "serde")]
fn default_k_table_control_polarity() -> crate::Wave {
    1.0
}

#[cfg(feature = "serde")]
fn default_control_from_drive() -> bool {
    true
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum BlockwiseSolveMode {
    #[default]
    Cascade,
    CoupledFixedPoint,
    CoupledNewton,
    DiodeLadderCore,
}

/// Circuit data needed to map cutoff CV to diode small-signal resistance.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DiodeCutoffCalibration {
    pub bias_voltage: crate::Wave,
    pub bias_resistance: crate::Wave,
    pub cv_resistance: Option<crate::Wave>,
    pub min_rp: crate::Wave,
    pub max_rp: crate::Wave,
}

impl DiodeCutoffCalibration {
    pub fn source_resistance(&self, model: DiodeModel, cutoff_cv: crate::Wave) -> crate::Wave {
        if let Some(r_cv) = self.cv_resistance {
            model
                .dynamic_resistance_from_sources(&[
                    (self.bias_voltage, self.bias_resistance),
                    (cutoff_cv, r_cv),
                ])
                .clamp(self.min_rp, self.max_rp)
        } else {
            model
                .dynamic_resistance_from_sources(&[(self.bias_voltage, self.bias_resistance)])
                .clamp(self.min_rp, self.max_rp)
        }
    }
}

fn diode_bias_voltage_from_current(model: DiodeModel, current: crate::Wave) -> crate::Wave {
    let i = current.max(1.0e-12);
    let junction = model.n_vt * crate::math::ln((i / model.is + 1.0) as crate::Wave) as crate::Wave;
    junction + i * model.rs
}

/// Table-backed differential diode ladder core.
///
/// The TB-303 ladder is not a cascade of independent one-port nonlinear
/// blocks. One tail current sets the operating point for every rung, while
/// the four capacitors provide the audio-rate state. This primitive keeps the
/// runtime cheap by using a single 1D table for the nonlinear differential
/// `tanh(v_dm / 2Vt)` shape and owning the four ladder states locally.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DiodeLadderCore {
    pub tanh_table: KTable,
    pub cap_values: Vec<crate::Wave>,
    pub states: Vec<Wave>,
    pub rung_alphas: Vec<Wave>,
    pub sample_rate: crate::Wave,
    pub alpha_bjt: crate::Wave,
    pub n_vt: crate::Wave,
    pub tail_current_table: KTable,
    pub i_tail_bias: crate::Wave,
    pub i_tail_min: crate::Wave,
    pub i_tail_max: crate::Wave,
    pub cutoff_bias_resistance: crate::Wave,
    pub last_alpha_i_tail: crate::Wave,
}

impl DiodeLadderCore {
    #[allow(clippy::too_many_arguments)] // ladder constructor: each arg is a distinct circuit parameter
    pub fn new(
        tanh_table: KTable,
        cap_values: Vec<crate::Wave>,
        sample_rate: crate::Wave,
        alpha_bjt: crate::Wave,
        n_vt: crate::Wave,
        tail_current_table: KTable,
        i_tail_bias: crate::Wave,
        i_tail_max: crate::Wave,
        cutoff_bias_resistance: crate::Wave,
    ) -> Self {
        let mut core = Self {
            tanh_table,
            states: vec![0.0 as Wave; cap_values.len()],
            rung_alphas: vec![0.0 as Wave; cap_values.len()],
            cap_values,
            sample_rate,
            alpha_bjt: alpha_bjt.clamp(0.0, 1.0),
            n_vt,
            tail_current_table,
            i_tail_bias,
            i_tail_min: (i_tail_bias * 0.01).max(1.0e-9),
            i_tail_max: i_tail_max.max(i_tail_bias * 1.01).max(1.0e-9),
            cutoff_bias_resistance: cutoff_bias_resistance.max(1.0),
            last_alpha_i_tail: crate::Wave::NAN,
        };
        core.tanh_table.precompute_scales();
        core.tail_current_table.precompute_scales();
        core
    }

    pub fn init(&mut self) {
        self.tanh_table.precompute_scales();
        self.tail_current_table.precompute_scales();
        if self.states.len() != self.cap_values.len() {
            self.states = vec![0.0 as Wave; self.cap_values.len()];
        }
        if self.rung_alphas.len() != self.cap_values.len() {
            self.rung_alphas = vec![0.0 as Wave; self.cap_values.len()];
            self.last_alpha_i_tail = crate::Wave::NAN;
        }
    }

    pub fn tail_current(
        &self,
        supply_voltage: crate::Wave,
        cutoff_cv_voltage: crate::Wave,
        cutoff_series_resistance: Option<crate::Wave>,
    ) -> crate::Wave {
        if let Some(r_cutoff) = cutoff_series_resistance {
            let total_r = (self.cutoff_bias_resistance + r_cutoff).max(1.0);
            return self
                .tail_current_table
                .lookup_2d(total_r, supply_voltage + cutoff_cv_voltage)
                .clamp(self.i_tail_min, self.i_tail_max);
        }

        let octave_span = 5.0;
        let norm = (0.5 + cutoff_cv_voltage / 5.0).clamp(0.0, 1.0);
        let ratio =
            crate::math::exp(((norm - 0.5) * octave_span * crate::math::LN_2) as crate::Wave)
                as crate::Wave;
        (self.i_tail_bias * ratio).clamp(self.i_tail_min, self.i_tail_max)
    }

    fn update_rung_alphas(&mut self, i_tail: crate::Wave) {
        if (i_tail - self.last_alpha_i_tail).abs()
            <= self.last_alpha_i_tail.abs().max(1.0e-12) * 1.0e-6
        {
            return;
        }
        let sample_rate = self.sample_rate.max(1.0);
        let n_vt = self.n_vt.max(1.0e-6);
        let side_current = 0.5 * i_tail;
        for (idx, alpha) in self.rung_alphas.iter_mut().enumerate() {
            let c = self
                .cap_values
                .get(idx)
                .copied()
                .unwrap_or(33.0e-9)
                .max(1.0e-12);
            // Local small-signal differential-pair physics:
            // the shared tail current splits through the two diode chains, so
            // each rung side is biased by I_tail/2. Stinchcombe fc belongs to
            // validation of the composed ladder, not this per-rung state update.
            let fc = (self.alpha_bjt * side_current / (4.0 * crate::math::PI * c * n_vt))
                .clamp(1.0, sample_rate * 0.45);
            *alpha = (1.0
                - crate::math::exp((-2.0 * crate::math::PI * fc / sample_rate) as crate::Wave)
                    as crate::Wave)
                .clamp(0.0, 1.0) as Wave;
        }
        self.last_alpha_i_tail = i_tail;
    }

    pub fn process(
        &mut self,
        input: Wave,
        supply_voltage: crate::Wave,
        cutoff_cv_voltage: crate::Wave,
        cutoff_series_resistance: Option<crate::Wave>,
    ) -> Wave {
        self.init();
        let i_tail = self.tail_current(supply_voltage, cutoff_cv_voltage, cutoff_series_resistance);
        self.update_rung_alphas(i_tail);
        let mut x = input;
        let n_vt = self.n_vt.max(1.0e-6);

        for (idx, state) in self.states.iter_mut().enumerate() {
            let drive = (x as crate::Wave).clamp(self.tanh_table.b_min, self.tanh_table.b_max);
            let tanh_v = self.tanh_table.lookup_1d(drive);
            let target =
                (2.0 * n_vt * self.alpha_bjt * tanh_v).clamp(-2.0 * n_vt, 2.0 * n_vt) as Wave;
            let alpha = self.rung_alphas.get(idx).copied().unwrap_or(0.0 as Wave);
            *state += alpha * (target - *state);
            if !state.is_finite() {
                *state = 0.0 as Wave;
            }
            x = *state;
        }

        if x.is_finite() {
            x * 100.0 as Wave
        } else {
            0.0 as Wave
        }
    }
}

#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CouplingElement {
    pub comp_id: String,
    /// MNA-local node endpoints used for scattering recomputation.
    pub node_a: Option<usize>,
    pub node_b: Option<usize>,
    /// Original circuit graph node endpoints used for route/debug derivation.
    #[cfg_attr(feature = "serde", serde(default))]
    pub graph_node_a: Option<usize>,
    #[cfg_attr(feature = "serde", serde(default))]
    pub graph_node_b: Option<usize>,
    pub resistance: crate::Wave,
    pub pot_max_resistance: Option<crate::Wave>,
    pub taper: crate::pot_taper::PotTaper,
    /// Feedback amount controls are wired as series resistances in the
    /// coupling graph, but the exposed control convention is "more = more
    /// feedback." Invert position before converting to resistance.
    pub invert_control: bool,
}

#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
/// Metadata binding for a reactive one-port attached to the BKM coupling adaptor.
///
/// Runtime equations and state live in `coupling_one_ports` and
/// `coupling_runtime_state`; this only preserves component identity and the
/// scattering-port attachment.
pub struct CouplingPassive {
    pub comp_id: String,
    pub port_idx: usize,
    pub one_port_idx: usize,
}

#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CouplingVcvs {
    pub pos: Option<usize>,
    pub neg: Option<usize>,
    pub out_pos: Option<usize>,
    pub out_neg: Option<usize>,
    pub gain: crate::Wave,
    pub output_resistance: crate::Wave,
    pub vsource_index: usize,
}

/// Reusable scratch space for coupled blockwise solves.
///
/// This keeps the realtime Newton path allocation-free without spreading a
/// set of parallel temporary vectors across `BlockwiseStage` itself.
#[derive(Clone, Default)]
pub struct CoupledSolveScratch {
    pub a: Vec<crate::Wave>,
    pub b: Vec<crate::Wave>,
    pub f: Vec<crate::Wave>,
    pub g: Vec<crate::Wave>,
    pub j: Vec<crate::Wave>,
    pub rhs: Vec<crate::Wave>,
    pub db_da: Vec<crate::Wave>,
    pub last_iterations: u32,
    pub last_residual: crate::Wave,
    pub last_converged: bool,
    pub last_linear_solve_failed: bool,
}

impl CoupledSolveScratch {
    pub fn new(n_ports: usize) -> Self {
        let mut scratch = Self::default();
        scratch.resize(n_ports);
        scratch
    }

    pub fn resize(&mut self, n_ports: usize) {
        self.a.resize(n_ports, 0.0);
        self.b.resize(n_ports, 0.0);
        self.f.resize(n_ports, 0.0);
        self.g.resize(n_ports, 0.0);
        self.rhs.resize(n_ports, 0.0);
        self.j.resize(n_ports * n_ports, 0.0);
        self.db_da.resize(n_ports * n_ports, 0.0);
    }

    fn load_a_from(&mut self, source: &[crate::Wave], n_ports: usize) {
        self.a.resize(n_ports, 0.0);
        if source.len() == n_ports {
            self.a.copy_from_slice(source);
        } else {
            self.a.fill(0.0);
        }
    }

    fn load_b_from(&mut self, source: &[crate::Wave], n_ports: usize) {
        self.b.resize(n_ports, 0.0);
        if source.len() == n_ports {
            self.b.copy_from_slice(source);
        } else {
            self.b.fill(0.0);
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SolveDiagnostics {
    pub iterations: u32,
    pub residual: crate::Wave,
    pub converged: bool,
    pub linear_solve_failed: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum OwnedPortRole {
    /// A conventional one-port block boundary.
    Primary,
    /// Positive side of a differential block input, used as `v_pos - v_neg`.
    DifferentialPositive,
    /// Negative/reference side of a differential block input.
    DifferentialNegative,
    /// Coupling-network output port that reflects the solved block voltage.
    DifferentialOutput,
}

pub type OwnedPortBinding = RolePortBinding<OwnedPortRole>;

impl OwnedPortBinding {
    pub const fn primary(port_idx: usize) -> Self {
        Self::new(port_idx, OwnedPortRole::Primary)
    }
}

/// Blockwise K-method stage: N coupled NL blocks with linear coupling.
///
/// Each block is a WDF tree + K-table (composite NL device).
/// The coupling network is a linear scattering matrix derived from
/// inter-block resistors (R_in, R_cv, R_fb_limit, Resonance, R_bias).
///
/// Wave-domain Newton iteration solves the algebraic feedback loop
/// each sample — no unit delay in the feedback path. This preserves
/// correct resonance tracking and fs-independent stability.
///
/// **Reference:** Werner, Smith, Abel (2015) "Wave Digital Filter Adaptors
/// for Arbitrary Topologies and Multiport Linear Elements"
#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BlockwiseStage {
    /// Coupling scattering matrix (n_ports × n_ports, row-major).
    /// Port ordering: [block_0, block_1, ..., block_{N-1}, VS_input].
    pub coupling_s: Vec<crate::Wave>,
    /// MNA node count used to derive the coupling matrix.
    pub coupling_n_mna: usize,
    /// Ports used to derive the coupling scattering matrix, mapped from
    /// original circuit graph terminals to MNA-local terminals.
    pub coupling_ports: Vec<CircuitMappedPort>,
    /// Recursive child evaluators inside this coupled stage.
    ///
    /// Simple one-port evaluators use one `Primary` port. Differential
    /// evaluators list the signed input ports and any reflected output ports
    /// they own. The stage-level `coupling_ports` still owns the graph/MNA
    /// terminal mapping; this field only says which evaluator produces each
    /// reflected wave.
    #[cfg_attr(
        feature = "serde",
        serde(default, alias = "owned_ports", alias = "block_ports")
    )]
    pub sub_stages: Vec<crate::processor::Stage>,
    /// Linear elements stamped into the coupling MNA. Pot entries keep enough
    /// metadata to recompute `coupling_s` when a coupling control moves.
    pub coupling_elements: Vec<CouplingElement>,
    /// Reactive elements represented as passive WDF ports in the coupling
    /// adaptor. These carry coupling-cap state such as TB303 C_out.
    #[cfg_attr(feature = "serde", serde(default))]
    pub coupling_passives: Vec<CouplingPassive>,
    /// Physical one-port runtime bindings for coupling passives.
    ///
    /// BKM-specific metadata lives in `coupling_passives`; the capacitor/
    /// inductor state equations stay in the shared `RuntimeOnePort` methods.
    #[cfg_attr(feature = "serde", serde(default))]
    pub coupling_one_ports: Vec<RuntimeOnePort<ScatteringPortId>>,
    /// Shared physical state and WDF wave sidecars for `coupling_one_ports`.
    #[cfg_attr(feature = "serde", serde(default))]
    pub coupling_runtime_state: RuntimeState,
    /// Dense scattering-port lookup into `coupling_passives`.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub coupling_passive_by_port: Vec<Option<usize>>,
    /// Active linear controlled sources stamped into the coupling MNA.
    ///
    /// These are static for a compiled circuit, but must be retained so pot
    /// updates can rebuild `coupling_s` without dropping op-amp/VCVS polarity
    /// stages from the adaptor.
    #[cfg_attr(feature = "serde", serde(default))]
    pub coupling_vcvss: Vec<CouplingVcvs>,
    /// Number of coupling ports (blocks.len() + 1 for VS input).
    pub n_ports: usize,
    /// Which block index to tap for output.
    pub output_block: usize,
    /// Optional high-impedance coupling port used to read the circuit output
    /// node after post-ladder coupling networks.
    #[cfg_attr(feature = "serde", serde(default))]
    pub output_port_index: Option<usize>,
    /// Read-only MNA extraction probe for the circuit output node.
    ///
    /// Unlike `output_port_index`, these do not add an observation port to the
    /// coupling adaptor, so reading `audio_out` cannot perturb the solve.
    #[cfg_attr(feature = "serde", serde(default))]
    pub output_extraction: ExtractionProbe<MnaNodeId>,
    /// Supply voltage (V) for supply VS ports in the coupling.
    pub supply_voltage: crate::Wave,
    /// VS port mapping: (port_name, scattering_port_index).
    /// Input ports from the .pedal that connect through coupling edges.
    /// At runtime, port values are written to work_b[scattering_port_idx]
    /// before the coupling scatter. The scattering distributes them.
    pub vs_port_map: Vec<(String, usize)>,
    /// Optional input port that modulates diode-ladder cutoff.
    ///
    /// Set by the compiler for blockwise explicit-diode ladders with a cutoff
    /// CV/input port in the coupling network.
    pub cutoff_cv_port: Option<String>,
    /// Coupling pot that controls one shared diode-ladder cutoff current.
    ///
    /// The pot is discovered by the compiler when a multi-rung explicit diode
    /// BKM ladder has a supply-side coupling pot on the first rung bias path.
    #[cfg_attr(feature = "serde", serde(default))]
    pub shared_diode_cutoff_pot: Option<String>,
    /// Internal feedback drives: (block_index, scattering_port_index).
    /// These inject the latest cascade output into coupling nodes such as the
    /// TB-303 resonance path from the final emitter back to the first base.
    pub feedback_port_map: Vec<(usize, usize)>,
    /// Passive attenuation compensation factor.
    pub compensation: crate::Wave,
    /// Oversampler for antialiasing.
    pub oversampler: Oversampler,
    /// BFS distance from input (for pipeline ordering).
    pub signal_flow_distance: usize,
    /// When true, output does not overwrite the serial chain.
    pub bypass_serial: bool,
    /// Composition mode for owned ports. `Cascade` preserves the legacy
    /// serial blockwise path for true chains; `CoupledFixedPoint` solves all
    /// owned ports simultaneously against the coupling adaptor.
    #[cfg_attr(feature = "serde", serde(default))]
    pub solve_mode: BlockwiseSolveMode,
    /// Optional whole-ladder primitive for differential diode ladders. This
    /// replaces the one-port-per-rung BKM solve when the compiler recognizes a
    /// shared-current four-rung ladder.
    #[cfg_attr(feature = "serde", serde(default))]
    pub diode_ladder_core: Option<DiodeLadderCore>,

    // ── Work buffers (pre-allocated, not serialized) ────────────────────
    /// Previous sample's reflected waves (warm-start for Newton).
    #[cfg_attr(feature = "serde", serde(skip))]
    pub b_warm: Vec<crate::Wave>,
    /// Work buffer: reflected waves from blocks (b_coupling).
    #[cfg_attr(feature = "serde", serde(skip))]
    pub work_b: Vec<crate::Wave>,
    /// Work buffer: incident waves from coupling scatter (a_coupling).
    #[cfg_attr(feature = "serde", serde(skip))]
    pub work_a: Vec<crate::Wave>,
    /// Coupled solver scratch buffers.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub coupled_scratch: CoupledSolveScratch,
    /// Cached port indices: maps vs_port_map entries to port_values indices.
    /// Resolved once at init (cache_all_vs_pointers), no per-sample string compare.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub port_index_cache: Vec<usize>,
}

impl core::fmt::Debug for KMethodBlock {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "KMethodBlock(rp={:.1}, bias={:.2})",
            self.rp, self.vbe_bias
        )
    }
}

#[deprecated(note = "Use BlockwiseStage; K-method is a per-block solver, not the topology")]
pub type BlockwiseKMethodStage = BlockwiseStage;

impl core::fmt::Debug for BlockwiseStage {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "BlockwiseStage({}blocks, {}ports)",
            self.block_count(),
            self.n_ports
        )
    }
}

impl BlockwiseStage {
    /// Maximum Newton iterations per sample.
    const MAX_ITER: usize = 8;
    /// Convergence tolerance on wave variables.
    const TOL: crate::Wave = 1e-6;
    /// Coupled delayed mode is a realtime approximation. Treat values outside
    /// normal audio/circuit rails as solver failure so they cannot poison the
    /// next sample's warm start.
    const MAX_STABLE_OUTPUT: crate::Wave = 1.0;
    /// Physical voltages entering one local K-method block must stay inside
    /// plausible circuit rails. The coupled adaptor may transiently propose a
    /// larger wave during Newton/fixed-point iteration; letting that value
    /// charge the local WDF caps creates artificial energy and audible blowups.
    const MAX_BLOCK_INCIDENT_VOLTAGE: crate::Wave = 4.0;
    /// One-sample coupled mode trades delay-free accuracy for real-time cost.
    /// Relaxing the adaptor wave update prevents high-bias diode ladders from
    /// turning the explicit delay into an artificial energy source.
    const DELAYED_COUPLING_RELAXATION: crate::Wave = 0.25;

    pub fn block_count(&self) -> usize {
        self.sub_stages
            .iter()
            .filter(|stage| matches!(stage, crate::processor::Stage::KMethod { .. }))
            .count()
    }

    pub fn k_method_block(&self, block_idx: usize) -> Option<&KMethodBlock> {
        match self.sub_stages.get(block_idx)? {
            crate::processor::Stage::KMethod { block, .. } => Some(block),
            _ => None,
        }
    }

    pub fn k_method_block_mut(&mut self, block_idx: usize) -> Option<&mut KMethodBlock> {
        match self.sub_stages.get_mut(block_idx)? {
            crate::processor::Stage::KMethod { block, .. } => Some(block),
            _ => None,
        }
    }

    pub fn k_method_blocks(&self) -> impl Iterator<Item = &KMethodBlock> {
        self.sub_stages.iter().filter_map(|stage| match stage {
            crate::processor::Stage::KMethod { block, .. } => Some(block),
            _ => None,
        })
    }

    pub fn k_method_blocks_mut(&mut self) -> impl Iterator<Item = &mut KMethodBlock> {
        self.sub_stages.iter_mut().filter_map(|stage| match stage {
            crate::processor::Stage::KMethod { block, .. } => Some(block),
            _ => None,
        })
    }

    pub fn k_method_ports(&self, block_idx: usize) -> Option<&[(OwnedPortRole, PortBinding)]> {
        match self.sub_stages.get(block_idx)? {
            crate::processor::Stage::KMethod { ports, .. } => Some(ports.as_slice()),
            _ => None,
        }
    }

    pub fn k_method_ports_mut(
        &mut self,
        block_idx: usize,
    ) -> Option<&mut Vec<(OwnedPortRole, PortBinding)>> {
        match self.sub_stages.get_mut(block_idx)? {
            crate::processor::Stage::KMethod { ports, .. } => Some(ports),
            _ => None,
        }
    }

    pub fn owned_port_for_role(&self, block_idx: usize, role: OwnedPortRole) -> Option<usize> {
        let ports = self.k_method_ports(block_idx)?;
        ports
            .iter()
            .find(|(owned_role, _)| *owned_role == role)
            .map(|(_, binding)| binding.local_port)
            .or_else(|| {
                (role == OwnedPortRole::Primary)
                    .then(|| ports.first().map(|(_, binding)| binding.local_port))
                    .flatten()
            })
    }

    pub fn owned_port_ids(&self, block_idx: usize) -> Vec<usize> {
        let mut ports = Vec::new();
        if let Some(bindings) = self.k_method_ports(block_idx) {
            for (_, binding) in bindings {
                if !ports.contains(&binding.local_port) {
                    ports.push(binding.local_port);
                }
            }
        }
        ports
    }

    fn bindings_for_coupling_port(&self, port_idx: usize) -> Vec<PortBinding> {
        let Some(terminals) = self.coupling_port_graph_terminals(port_idx) else {
            return Vec::new();
        };
        let (pos, neg) = terminals.as_tuple();
        pos.into_iter()
            .chain(neg)
            .map(|node| PortBinding::new(BindingId::new(node), port_idx))
            .collect()
    }

    /// Declarative input bindings exposed by the blockwise stage boundary.
    pub fn ins(&self) -> Vec<PortBinding> {
        self.coupling_ports
            .iter()
            .enumerate()
            .flat_map(|(idx, _)| self.bindings_for_coupling_port(idx))
            .collect()
    }

    /// Declarative output bindings exposed by the blockwise stage boundary.
    ///
    /// Coupling ports are bidirectional at the stage boundary. The
    /// realtime blockwise solver owns internal coupling; cross-stage routing
    /// should use this boundary surface through `Stage::ins()`/`Stage::outs()`.
    pub fn outs(&self) -> Vec<PortBinding> {
        self.ins()
    }

    pub fn differential_owned_ports(&self, block_idx: usize) -> Option<(usize, usize, usize)> {
        Some((
            self.owned_port_for_role(block_idx, OwnedPortRole::DifferentialPositive)?,
            self.owned_port_for_role(block_idx, OwnedPortRole::DifferentialNegative)?,
            self.owned_port_for_role(block_idx, OwnedPortRole::DifferentialOutput)?,
        ))
    }

    pub fn coupling_port_graph_terminals(&self, port_idx: usize) -> Option<WdfPortTerminals> {
        self.coupling_ports
            .get(port_idx)
            .map(|port| port.graph.raw())
    }

    pub fn coupling_wdf_ports(&self) -> Vec<crate::tree::WdfPort> {
        self.coupling_ports
            .iter()
            .copied()
            .map(CircuitMappedPort::to_wdf_port)
            .collect()
    }

    pub fn coupling_network_model(&self) -> LinearMultiportNetwork<GraphNodeId, MnaNodeId> {
        let mut network =
            LinearMultiportNetwork::new(self.coupling_s.clone(), self.coupling_ports.clone());
        network.variable_resistors = self
            .coupling_elements
            .iter()
            .enumerate()
            .filter_map(|(element_idx, element)| {
                element
                    .pot_max_resistance
                    .map(|_| crate::boundary_math::VariableResistorBinding {
                        child_idx: element_idx,
                        terminals: MnaPortTerminals::maybe_differential(
                            element.node_a.map(MnaNodeId::new),
                            element.node_b.map(MnaNodeId::new),
                        ),
                        conductance: 1.0 / element.resistance.max(1.0e-12),
                    })
            })
            .collect();
        network.extraction = Some(self.output_extraction.clone());
        network
    }

    fn block_drive_voltage(block: &KMethodBlock, physical_voltage: crate::Wave) -> crate::Wave {
        block.source_polarity
            * physical_voltage.clamp(
                -Self::MAX_BLOCK_INCIDENT_VOLTAGE,
                Self::MAX_BLOCK_INCIDENT_VOLTAGE,
            )
    }

    fn block_control_voltage(block: &KMethodBlock, physical_voltage: crate::Wave) -> crate::Wave {
        if let Some(v_bias) = block.shared_diode_bias_voltage {
            return v_bias;
        }
        if !block.control_from_drive {
            return 0.0;
        }
        block.k_table_control_polarity * physical_voltage
    }

    fn ensure_sub_stages(&mut self) {}

    fn ensure_coupling_runtime_state(&mut self) {
        let Some(required_len) = self
            .coupling_one_ports
            .iter()
            .filter_map(|one_port| one_port.state_slot())
            .map(|slot| slot.0)
            .max()
            .map(|slot| slot + 1)
        else {
            return;
        };

        if self.coupling_runtime_state.states.len() == required_len
            && self.coupling_runtime_state.wave_cache.len() == required_len
        {
            return;
        }

        let mut slot_kinds = vec![None; required_len];
        for one_port in &self.coupling_one_ports {
            if let Some(slot) = one_port.state_slot() {
                if slot.0 < required_len {
                    slot_kinds[slot.0] = Some(one_port.spec.kind);
                }
            }
        }

        self.coupling_runtime_state
            .states
            .resize_with(required_len, || OnePortState::CapacitorVoltage(0.0));
        for (idx, kind) in slot_kinds.into_iter().enumerate() {
            match kind {
                Some(OnePortKind::Capacitor(_)) => {
                    if !matches!(
                        self.coupling_runtime_state.states[idx],
                        OnePortState::CapacitorVoltage(_)
                    ) {
                        self.coupling_runtime_state.states[idx] =
                            OnePortState::CapacitorVoltage(0.0);
                    }
                }
                Some(OnePortKind::Inductor(_)) => {
                    if !matches!(
                        self.coupling_runtime_state.states[idx],
                        OnePortState::InductorScaledCurrent(_)
                    ) {
                        self.coupling_runtime_state.states[idx] =
                            OnePortState::InductorScaledCurrent(0.0);
                    }
                }
                Some(OnePortKind::Resistor(_)) | None => {}
            }
        }
        self.coupling_runtime_state
            .wave_cache
            .resize(required_len, WdfWaveCache::default());
    }

    fn ensure_block_runtime_states(&mut self) {
        for block in self.k_method_blocks_mut() {
            if block.runtime_state.states.len() != block.runtime_state.wave_cache.len()
                || block.runtime_state.is_empty()
            {
                block.runtime_state = block.tree.bind_runtime_state();
            }
        }
    }

    fn primary_port_for_block(&self, block_idx: usize) -> Option<usize> {
        self.owned_port_for_role(block_idx, OwnedPortRole::Primary)
            .or_else(|| (block_idx < self.n_ports).then_some(block_idx))
    }

    fn port_is_block_owned(&self, port_idx: usize) -> bool {
        (0..self.block_count()).any(|block_idx| self.owned_port_ids(block_idx).contains(&port_idx))
            || port_idx < self.block_count()
    }

    fn scatter_owned_ports(
        &mut self,
        block_idx: usize,
        a: &[crate::Wave],
        serial_input: crate::Wave,
        boundary_drives: &[BoundaryIncidentDrive],
        update_state: bool,
        b_out: &mut [crate::Wave],
    ) -> crate::Wave {
        let Some(primary_port) = self.primary_port_for_block(block_idx) else {
            return 0.0;
        };
        let incident = (a.get(primary_port).copied().unwrap_or(0.0)
            + if block_idx == 0 { serial_input } else { 0.0 })
        .clamp(
            -Self::MAX_BLOCK_INCIDENT_VOLTAGE,
            Self::MAX_BLOCK_INCIDENT_VOLTAGE,
        );

        if self.k_method_block(block_idx).is_some() {
            if let Some((positive_port, negative_port, output_port)) =
                self.differential_owned_ports(block_idx)
            {
                let a_positive = a.get(positive_port).copied().unwrap_or(0.0)
                    + sum_incident_offsets(positive_port, boundary_drives);
                let a_negative = a.get(negative_port).copied().unwrap_or(0.0)
                    + sum_incident_offsets(negative_port, boundary_drives);
                let differential_incident = (a_positive - a_negative
                    + if block_idx == 0 { serial_input } else { 0.0 })
                .clamp(
                    -Self::MAX_BLOCK_INCIDENT_VOLTAGE,
                    Self::MAX_BLOCK_INCIDENT_VOLTAGE,
                );
                let Some(block) = self.k_method_block_mut(block_idx) else {
                    return 0.0;
                };
                let (raw_voltage, ac_voltage, _) =
                    Self::solve_block_port(block, differential_incident, update_state);

                // Reflect the solved physical voltage on every evaluator-owned
                // output port so passive coupling observes the live block state,
                // not only the imposed incident wave.
                if positive_port < b_out.len() {
                    b_out[positive_port] =
                        2.0 * raw_voltage - a.get(positive_port).copied().unwrap_or(0.0);
                }
                if negative_port < b_out.len() {
                    b_out[negative_port] = a.get(negative_port).copied().unwrap_or(0.0);
                }
                if output_port < b_out.len() {
                    b_out[output_port] =
                        2.0 * raw_voltage - a.get(output_port).copied().unwrap_or(0.0);
                }
                return ac_voltage;
            }

            let is_multiport_block = self
                .k_method_ports(block_idx)
                .map(|ports| ports.len() > 1)
                .unwrap_or(false);
            let Some(block) = self.k_method_block_mut(block_idx) else {
                return 0.0;
            };
            let (raw_voltage, ac_voltage, primary_reflected) =
                Self::solve_block_port(block, incident, update_state);
            if primary_port < b_out.len() {
                b_out[primary_port] = if is_multiport_block {
                    2.0 * incident - a.get(primary_port).copied().unwrap_or(0.0)
                } else {
                    primary_reflected
                };
            }

            for port_idx in self.owned_port_ids(block_idx) {
                if port_idx == primary_port {
                    continue;
                }
                if port_idx < b_out.len() {
                    b_out[port_idx] = 2.0 * raw_voltage - a.get(port_idx).copied().unwrap_or(0.0);
                }
            }
            ac_voltage
        } else if primary_port < b_out.len() {
            let Some(block) = self.k_method_block_mut(block_idx) else {
                return 0.0;
            };
            let (_, ac_voltage, primary_reflected) =
                Self::solve_block_port(block, incident, update_state);
            b_out[primary_port] = primary_reflected;
            ac_voltage
        } else {
            let Some(block) = self.k_method_block_mut(block_idx) else {
                return 0.0;
            };
            let (_, ac_voltage, _) = Self::solve_block_port(block, incident, update_state);
            ac_voltage
        }
    }

    fn block_root_incident(
        block: &mut KMethodBlock,
        b_tree: crate::Wave,
        ctrl: crate::Wave,
    ) -> crate::Wave {
        NonlinearSolver::KTable(&block.k_table)
            .solve_root_incident(b_tree, ctrl)
            .unwrap_or(0.0)
    }

    fn block_root_incident_with_derivatives(
        block: &KMethodBlock,
        b_tree: crate::Wave,
        ctrl: crate::Wave,
    ) -> (crate::Wave, crate::Wave, crate::Wave) {
        NonlinearSolver::KTable(&block.k_table)
            .solve_root_incident_with_derivatives(b_tree, ctrl)
            .unwrap_or((0.0, 0.0, 0.0))
    }

    fn solve_block_without_state_update(
        block: &mut KMethodBlock,
        physical_voltage: crate::Wave,
    ) -> (crate::Wave, crate::Wave, crate::Wave, crate::Wave) {
        let drive_voltage = Self::block_drive_voltage(block, physical_voltage);
        let control_voltage = Self::block_control_voltage(block, physical_voltage);
        block.tree.set_voltage(drive_voltage);
        let b_tree = block.tree.reflected_with_state(&mut block.runtime_state);
        let a_root = Self::block_root_incident(block, b_tree, control_voltage);
        let raw_cascade = Self::block_output_voltage(block, a_root, b_tree);
        (a_root, b_tree, raw_cascade, raw_cascade - block.dc_offset)
    }

    fn solve_block_without_state_update_with_derivative(
        block: &mut KMethodBlock,
        physical_voltage: crate::Wave,
    ) -> (crate::Wave, crate::Wave, crate::Wave) {
        let drive_voltage = Self::block_drive_voltage(block, physical_voltage);
        let control_voltage = Self::block_control_voltage(block, physical_voltage);
        block.tree.set_voltage(drive_voltage);
        let b_tree = block.tree.reflected_with_state(&mut block.runtime_state);
        let (a_root, da_db_tree, da_dctrl) =
            Self::block_root_incident_with_derivatives(block, b_tree, control_voltage);
        let raw_cascade = Self::block_output_voltage(block, a_root, b_tree);

        let db_tree_dv = block.source_polarity * block.tree.reflected_voltage_gain();
        let dctrl_dv = if block.shared_diode_bias_voltage.is_some() || !block.control_from_drive {
            0.0
        } else {
            block.k_table_control_polarity
        };
        let da_root_dv = da_db_tree * db_tree_dv + da_dctrl * dctrl_dv;
        let output_gain = Self::block_output_voltage_gain(block, da_root_dv, db_tree_dv);

        (
            raw_cascade - block.dc_offset,
            2.0 * output_gain - 1.0,
            output_gain,
        )
    }

    fn solve_block_and_update_state(
        block: &mut KMethodBlock,
        physical_voltage: crate::Wave,
    ) -> (crate::Wave, crate::Wave, crate::Wave, crate::Wave) {
        let (a_root, b_tree, raw_cascade, ac_cascade) =
            Self::solve_block_without_state_update(block, physical_voltage);
        block
            .tree
            .set_incident_with_state(a_root, &mut block.runtime_state);
        (a_root, b_tree, raw_cascade, ac_cascade)
    }

    fn write_vs_ports(&mut self, vs_signals: &[crate::Wave]) {
        let n = self.n_ports;
        for (i, &(ref name, vs_idx)) in self.vs_port_map.iter().enumerate() {
            if vs_idx < n {
                let v = if name.starts_with("_supply_") {
                    self.supply_voltage
                } else {
                    vs_signals.get(i).copied().unwrap_or(0.0)
                };
                self.work_b[vs_idx] = 2.0 * v - self.work_a[vs_idx];
            }
        }
    }

    fn write_feedback_ports(&mut self, output: crate::Wave) {
        let n = self.n_ports;
        for &(block_idx, port_idx) in &self.feedback_port_map {
            if port_idx < n {
                if !self.feedback_port_is_active(port_idx) {
                    self.work_b[port_idx] = 0.0;
                    continue;
                }
                let v = if block_idx == self.output_block {
                    output
                } else {
                    self.b_warm.get(block_idx).copied().unwrap_or(0.0)
                };
                self.work_b[port_idx] = 2.0 * v - self.work_a[port_idx];
            }
        }
    }

    fn feedback_port_is_active(&self, port_idx: usize) -> bool {
        let Some(port) = self.coupling_ports.get(port_idx) else {
            return true;
        };
        let feedback_node = port.mna.terminals.pos.map(MnaNodeId::get);

        let mut saw_gated_feedback = false;
        for element in &self.coupling_elements {
            if element.node_a != feedback_node && element.node_b != feedback_node {
                continue;
            }
            let Some(max_r) = element.pot_max_resistance else {
                continue;
            };
            if !element.invert_control {
                continue;
            }

            saw_gated_feedback = true;
            if element.resistance < max_r * 0.9 {
                return true;
            }
        }

        !saw_gated_feedback
    }

    fn coupling_passive_index(&self, port_idx: usize) -> Option<usize> {
        self.coupling_passive_by_port
            .get(port_idx)
            .and_then(|idx| *idx)
    }

    fn rebuild_coupling_passive_by_port(&mut self) {
        self.coupling_passive_by_port.clear();
        self.coupling_passive_by_port.resize(self.n_ports, None);
        for (passive_idx, passive) in self.coupling_passives.iter().enumerate() {
            if passive.port_idx < self.n_ports {
                self.coupling_passive_by_port[passive.port_idx] = Some(passive_idx);
            }
        }
    }

    pub fn coupling_passive_one_port(
        &self,
        passive_idx: usize,
    ) -> Option<&RuntimeOnePort<ScatteringPortId>> {
        let one_port_idx = self.coupling_passives.get(passive_idx)?.one_port_idx;
        self.coupling_one_ports.get(one_port_idx)
    }

    pub fn coupling_passive_one_port_state(&self, passive_idx: usize) -> Option<OnePortState> {
        self.coupling_passive_one_port(passive_idx)?
            .wdf_one_port_state(&self.coupling_runtime_state)
    }

    pub fn set_coupling_passive_one_port_state(
        &mut self,
        passive_idx: usize,
        state: OnePortState,
    ) -> bool {
        let Some(one_port_idx) = self
            .coupling_passives
            .get(passive_idx)
            .map(|passive| passive.one_port_idx)
        else {
            return false;
        };
        let Some(one_port) = self.coupling_one_ports.get(one_port_idx).copied() else {
            return false;
        };
        one_port.wdf_set_one_port_state(state, &mut self.coupling_runtime_state)
    }

    fn output_probe_voltage(&self, fallback: crate::Wave) -> crate::Wave {
        if let Some(v) = self.output_extraction.read_reflected(&self.work_a) {
            return v;
        }

        let Some(port_idx) = self.output_port_index else {
            return fallback;
        };
        if port_idx >= self.work_a.len() || port_idx >= self.work_b.len() {
            return fallback;
        }
        let v = (self.work_a[port_idx] + self.work_b[port_idx]) * 0.5;
        if v.is_finite() {
            v
        } else {
            fallback
        }
    }

    fn scatter_coupling(&mut self) {
        if !scatter_matrix_into(
            &self.coupling_s,
            self.n_ports,
            &self.work_b,
            &mut self.work_a,
        ) {
            self.work_a.fill(0.0);
        }
    }

    fn run_block_cascade(
        &mut self,
        include_cascade: bool,
        update_state: bool,
        serial_input: crate::Wave,
        mut block_outputs: Option<&mut alloc::vec::Vec<crate::Wave>>,
    ) -> crate::Wave {
        let mut cascade_drive = 0.0;
        let mut cascade_out = 0.0;
        for i in 0..self.block_count() {
            let v_coupling = (self.work_a[i] + self.work_b[i]) / 2.0;
            let v_block = if include_cascade {
                v_coupling + if i == 0 { serial_input } else { cascade_drive }
            } else {
                v_coupling
            };
            let Some(block) = self.k_method_block_mut(i) else {
                continue;
            };
            let (a_root, _, _raw_cascade, ac_cascade) = if update_state {
                Self::solve_block_and_update_state(block, v_block)
            } else {
                Self::solve_block_without_state_update(block, v_block)
            };
            cascade_drive = ac_cascade;
            cascade_out = ac_cascade;
            let _ = a_root;
            self.work_b[i] = 2.0 * v_coupling - self.work_a[i];
            if let Some(outputs) = block_outputs.as_deref_mut() {
                outputs.push(ac_cascade);
            }
        }
        cascade_out
    }

    fn solve_block_port(
        block: &mut KMethodBlock,
        incident: crate::Wave,
        update_state: bool,
    ) -> (crate::Wave, crate::Wave, crate::Wave) {
        if !incident.is_finite() {
            return (0.0, 0.0, 0.0);
        }
        let (_, _, raw_voltage, ac_voltage) = if update_state {
            Self::solve_block_and_update_state(block, incident)
        } else {
            Self::solve_block_without_state_update(block, incident)
        };
        if !raw_voltage.is_finite() || !ac_voltage.is_finite() {
            return (0.0, 0.0, -incident);
        }
        let reflected = 2.0 * raw_voltage - incident;
        if reflected.is_finite() {
            (raw_voltage, ac_voltage, reflected)
        } else {
            (0.0, 0.0, -incident)
        }
    }

    fn vs_voltage_for_port(
        &self,
        port_idx: usize,
        vs_signals: &[crate::Wave],
    ) -> Option<crate::Wave> {
        self.vs_port_map
            .iter()
            .enumerate()
            .find_map(|(signal_idx, (name, idx))| {
                if *idx != port_idx {
                    return None;
                }
                Some(if name.starts_with("_supply_") {
                    self.supply_voltage
                } else {
                    vs_signals.get(signal_idx).copied().unwrap_or(0.0)
                })
            })
    }

    fn port_incident_offset(
        port_idx: usize,
        boundary_drives: &[BoundaryIncidentDrive],
    ) -> crate::Wave {
        sum_incident_offsets(port_idx, boundary_drives)
    }

    fn coupled_eval_b_for_a(
        &mut self,
        a: &[crate::Wave],
        vs_signals: &[crate::Wave],
        serial_input: crate::Wave,
        boundary_drives: &[BoundaryIncidentDrive],
        update_state: bool,
        b_out: &mut [crate::Wave],
    ) -> crate::Wave {
        let mut output = 0.0;
        for v in b_out.iter_mut() {
            *v = 0.0;
        }
        for block_idx in 0..self.block_count() {
            let block_output = self.scatter_owned_ports(
                block_idx,
                a,
                serial_input,
                boundary_drives,
                update_state,
                b_out,
            );
            if block_idx == self.output_block {
                output = block_output;
            }
        }
        for (i, b_i) in b_out.iter_mut().enumerate().take(self.n_ports) {
            if self.port_is_block_owned(i) {
                continue;
            } else if let Some(passive_idx) = self.coupling_passive_index(i) {
                let one_port_idx = self.coupling_passives[passive_idx].one_port_idx;
                let one_port = self.coupling_one_ports[one_port_idx];
                *b_i = one_port.wdf_reflected(&self.coupling_runtime_state);
                if update_state {
                    one_port.wdf_set_incident(
                        a.get(i).copied().unwrap_or(0.0),
                        &mut self.coupling_runtime_state,
                    );
                }
            } else if self.output_port_index == Some(i) {
                *b_i = a.get(i).copied().unwrap_or(0.0);
            } else if let Some(&(block_idx, _)) = self
                .feedback_port_map
                .iter()
                .find(|(_, port_idx)| *port_idx == i)
            {
                if !self.feedback_port_is_active(i) {
                    *b_i = 0.0;
                    continue;
                }
                let v = if block_idx == self.output_block {
                    output
                } else {
                    self.b_warm.get(block_idx).copied().unwrap_or(0.0)
                };
                *b_i = 2.0 * v - a.get(i).copied().unwrap_or(0.0);
            } else if let Some(v) = self.vs_voltage_for_port(i, vs_signals) {
                *b_i = 2.0 * v - a.get(i).copied().unwrap_or(0.0);
            } else {
                *b_i = -a.get(i).copied().unwrap_or(0.0);
            }

            if !b_i.is_finite() {
                *b_i = -a.get(i).copied().unwrap_or(0.0);
            }
        }
        output
    }

    fn coupled_scatter_from_b(&self, b: &[crate::Wave], a_out: &mut [crate::Wave]) {
        if !scatter_matrix_into(&self.coupling_s, self.n_ports, b, a_out) {
            a_out.fill(0.0);
            return;
        }
        for incident in a_out {
            if !incident.is_finite() {
                *incident = 0.0;
            }
        }
    }

    fn coupled_fill_sparse_db_da(
        &mut self,
        a: &[crate::Wave],
        serial_input: crate::Wave,
        boundary_drives: &[BoundaryIncidentDrive],
        db_da: &mut [crate::Wave],
    ) {
        let n = self.n_ports;
        db_da.fill(0.0);

        // Voltage-source, open, and delayed-feedback ports all reflect their
        // own incident wave as b = const - a unless a live block-output
        // feedback dependency below overwrites/adds a cross derivative.
        for port_idx in 0..n {
            if !self.port_is_block_owned(port_idx) {
                db_da[port_idx * n + port_idx] = if self.coupling_passive_index(port_idx).is_some()
                {
                    0.0
                } else if self.output_port_index == Some(port_idx) {
                    1.0
                } else {
                    -1.0
                };
            }
        }

        let mut output_derivative_by_block = alloc::vec![0.0; self.block_count()];
        for (block_idx, d_slot) in output_derivative_by_block.iter_mut().enumerate() {
            let Some(primary_port) = self.primary_port_for_block(block_idx) else {
                continue;
            };
            if primary_port >= n {
                continue;
            }

            let incident = (a.get(primary_port).copied().unwrap_or(0.0)
                + if block_idx == 0 { serial_input } else { 0.0 })
            .clamp(
                -Self::MAX_BLOCK_INCIDENT_VOLTAGE,
                Self::MAX_BLOCK_INCIDENT_VOLTAGE,
            );
            let Some(block) = self.k_method_block_mut(block_idx) else {
                continue;
            };
            let (_, d_refl, d_out) =
                Self::solve_block_without_state_update_with_derivative(block, incident);
            let d_out = d_out.clamp(-1.0e6, 1.0e6);
            let d_refl = d_refl.clamp(-1.0e6, 1.0e6);
            let d_out = if d_out.is_finite() { d_out } else { 0.0 };
            let d_refl = if d_refl.is_finite() { d_refl } else { 0.0 };
            *d_slot = d_out;

            if self.k_method_block(block_idx).is_some() {
                if let Some((positive_port, negative_port, output_port)) =
                    self.differential_owned_ports(block_idx)
                {
                    let a_positive = a.get(positive_port).copied().unwrap_or(0.0)
                        + Self::port_incident_offset(positive_port, boundary_drives);
                    let a_negative = a.get(negative_port).copied().unwrap_or(0.0)
                        + Self::port_incident_offset(negative_port, boundary_drives);
                    let differential_incident = (a_positive - a_negative
                        + if block_idx == 0 { serial_input } else { 0.0 })
                    .clamp(
                        -Self::MAX_BLOCK_INCIDENT_VOLTAGE,
                        Self::MAX_BLOCK_INCIDENT_VOLTAGE,
                    );
                    let Some(block) = self.k_method_block_mut(block_idx) else {
                        continue;
                    };
                    let (_, _, d_out) = Self::solve_block_without_state_update_with_derivative(
                        block,
                        differential_incident,
                    );
                    let d_out = d_out.clamp(-1.0e6, 1.0e6);
                    let d_out = if d_out.is_finite() { d_out } else { 0.0 };
                    *d_slot = d_out;

                    if positive_port < n {
                        db_da[positive_port * n + positive_port] = 2.0 * d_out - 1.0;
                        db_da[positive_port * n + negative_port] = -2.0 * d_out;
                    }
                    if negative_port < n {
                        db_da[negative_port * n + negative_port] = 1.0;
                    }
                    if output_port < n {
                        db_da[output_port * n + positive_port] = 2.0 * d_out;
                        db_da[output_port * n + negative_port] = -2.0 * d_out;
                        db_da[output_port * n + output_port] = -1.0;
                    }
                    continue;
                }

                let is_multiport_block = self
                    .k_method_ports(block_idx)
                    .map(|ports| ports.len() > 1)
                    .unwrap_or(false);
                db_da[primary_port * n + primary_port] = if is_multiport_block {
                    // Multiport block primary ports currently reflect the
                    // imposed physical incident voltage: b = 2*incident - a.
                    1.0
                } else {
                    d_refl
                };
                for port_idx in self.owned_port_ids(block_idx) {
                    if port_idx == primary_port {
                        continue;
                    }
                    if port_idx < n {
                        db_da[port_idx * n + primary_port] = 2.0 * d_out;
                        db_da[port_idx * n + port_idx] = -1.0;
                    }
                }
            } else {
                db_da[primary_port * n + primary_port] = d_refl;
            }
        }

        for &(block_idx, port_idx) in &self.feedback_port_map {
            if port_idx >= n || block_idx >= self.block_count() {
                continue;
            }
            if !self.feedback_port_is_active(port_idx) {
                db_da[port_idx * n + port_idx] = 0.0;
                continue;
            }
            db_da[port_idx * n + port_idx] = -1.0;
            if block_idx == self.output_block {
                if let Some(primary_port) = self.primary_port_for_block(block_idx) {
                    if primary_port < n {
                        db_da[port_idx * n + primary_port] +=
                            2.0 * output_derivative_by_block[block_idx];
                    }
                }
            }
        }
    }

    fn coupled_solve_newton(
        &mut self,
        vs_signals: &[crate::Wave],
        serial_input: crate::Wave,
        boundary_drives: &[BoundaryIncidentDrive],
    ) -> (crate::Wave, bool) {
        let n = self.n_ports;
        if n == 0 {
            return (0.0, true);
        }

        let mut scratch = core::mem::take(&mut self.coupled_scratch);
        scratch.resize(n);
        scratch.load_a_from(&self.work_a, n);
        let mut output = 0.0;
        let mut converged = false;
        let mut iterations = 0u32;
        let mut last_residual = crate::Wave::INFINITY;
        let mut linear_solve_failed = false;

        for _ in 0..Self::MAX_ITER {
            iterations = iterations.saturating_add(1);
            output = self.coupled_eval_b_for_a(
                &scratch.a,
                vs_signals,
                serial_input,
                boundary_drives,
                false,
                &mut scratch.b,
            );
            self.coupled_scatter_from_b(&scratch.b, &mut scratch.f);

            let mut max_err: crate::Wave = 0.0;
            for i in 0..n {
                scratch.g[i] = scratch.a[i] - scratch.f[i];
                max_err = max_err.max(scratch.g[i].abs());
            }
            last_residual = max_err;
            if max_err < Self::TOL {
                converged = true;
                break;
            }

            self.coupled_fill_sparse_db_da(
                &scratch.a,
                serial_input,
                boundary_drives,
                &mut scratch.db_da,
            );

            for row in 0..n {
                for col in 0..n {
                    let mut coupling_derivative = 0.0;
                    for k in 0..n {
                        coupling_derivative +=
                            self.coupling_s[row * n + k] * scratch.db_da[k * n + col];
                    }
                    scratch.j[row * n + col] =
                        if row == col { 1.0 } else { 0.0 } - coupling_derivative;
                }
                scratch.rhs[row] = -scratch.g[row];
            }

            if !crate::elements::nonlinear::solver::solve_small_linear(
                n,
                &mut scratch.j,
                &mut scratch.rhs,
            ) {
                linear_solve_failed = true;
                break;
            }

            let mut max_step: crate::Wave = 0.0;
            for i in 0..n {
                let step = scratch.rhs[i].clamp(-1.0, 1.0);
                scratch.a[i] += step;
                if !scratch.a[i].is_finite() {
                    scratch.a[i] = 0.0;
                }
                max_step = max_step.max(step.abs());
            }
            if max_step < Self::TOL {
                converged = true;
                break;
            }
        }

        if !converged {
            output = self.coupled_eval_b_for_a(
                &scratch.a,
                vs_signals,
                serial_input,
                boundary_drives,
                false,
                &mut scratch.b,
            );
        }
        self.work_a.copy_from_slice(&scratch.a);
        self.work_b.copy_from_slice(&scratch.b);
        scratch.last_iterations = iterations;
        scratch.last_residual = last_residual;
        scratch.last_converged = converged;
        scratch.last_linear_solve_failed = linear_solve_failed;

        self.coupled_scratch = scratch;

        (output, converged)
    }

    fn run_coupled_blocks(
        &mut self,
        update_state: bool,
        serial_input: crate::Wave,
        mut block_outputs: Option<&mut alloc::vec::Vec<crate::Wave>>,
    ) -> crate::Wave {
        let n_blocks = self.block_count();
        let n_ports = self.n_ports;
        let mut scratch = core::mem::take(&mut self.coupled_scratch);
        scratch.load_a_from(&self.work_a, n_ports);
        scratch.load_b_from(&self.work_b, n_ports);
        let mut output = 0.0;
        for i in 0..n_blocks {
            let port_voltage = self.scatter_owned_ports(
                i,
                &scratch.a,
                serial_input,
                &[],
                update_state,
                &mut scratch.b,
            );
            if i == self.output_block {
                output = port_voltage;
            }
            if let Some(outputs) = block_outputs.as_deref_mut() {
                outputs.push(port_voltage);
            }
        }
        for block_idx in 0..self.block_count() {
            for port_idx in self.owned_port_ids(block_idx) {
                if port_idx < self.work_b.len() && port_idx < scratch.b.len() {
                    self.work_b[port_idx] = if update_state {
                        scratch.b[port_idx]
                    } else {
                        0.5 * self.work_b[port_idx] + 0.5 * scratch.b[port_idx]
                    };
                }
            }
        }
        self.coupled_scratch = scratch;
        output
    }

    pub fn debug_process_with_block_outputs(
        &mut self,
        vs_signals: &[crate::Wave],
    ) -> (crate::Wave, alloc::vec::Vec<crate::Wave>) {
        self.debug_process_with_block_outputs_with_serial_input(0.0, vs_signals)
    }

    pub fn debug_process_with_block_outputs_with_serial_input(
        &mut self,
        serial_input: crate::Wave,
        vs_signals: &[crate::Wave],
    ) -> (crate::Wave, alloc::vec::Vec<crate::Wave>) {
        if self.work_b.len() != self.n_ports
            || self
                .coupling_one_ports
                .iter()
                .filter_map(|one_port| one_port.state_slot())
                .any(|slot| slot.0 >= self.coupling_runtime_state.len())
            || self.k_method_blocks().any(|block| {
                block.runtime_state.states.len() != block.runtime_state.wave_cache.len()
            })
        {
            self.init_buffers();
        }

        let mut outputs = alloc::vec::Vec::with_capacity(self.block_count());
        let output = if matches!(
            self.solve_mode,
            BlockwiseSolveMode::CoupledFixedPoint | BlockwiseSolveMode::CoupledNewton
        ) {
            if self.solve_mode == BlockwiseSolveMode::CoupledNewton {
                let _ = self.coupled_solve_newton(vs_signals, serial_input, &[]);
            } else {
                self.write_vs_ports(vs_signals);
                self.scatter_coupling();
            }
            self.run_coupled_blocks(true, serial_input, Some(&mut outputs))
        } else {
            self.write_vs_ports(vs_signals);
            self.scatter_coupling();
            self.run_block_cascade(true, true, serial_input, Some(&mut outputs))
        };
        (output, outputs)
    }

    pub fn debug_current_differential_incidents(&self) -> alloc::vec::Vec<crate::Wave> {
        (0..self.block_count())
            .map(|block_idx| {
                self.differential_owned_ports(block_idx)
                    .map(|(positive_port, negative_port, _)| {
                        self.work_a.get(positive_port).copied().unwrap_or(0.0)
                            - self.work_a.get(negative_port).copied().unwrap_or(0.0)
                    })
                    .unwrap_or(0.0)
            })
            .collect()
    }

    fn block_output_voltage(
        block: &KMethodBlock,
        a_root: crate::Wave,
        b_tree: crate::Wave,
    ) -> crate::Wave {
        if let Some(ref probe_id) = block.cascade_probe_id {
            if let Some(v) = block.tree.leaf_voltage_for_incident_with_state(
                probe_id,
                a_root,
                &block.runtime_state,
            ) {
                return v;
            }
        }

        if block.cascade_from_passive {
            if let DynNode::Binary {
                kind: crate::dyn_node::BinaryKind::Series,
                gamma,
                b1,
                b2,
                ..
            } = &block.tree
            {
                let sum = *b1 + *b2 + a_root;
                let a_right = *b2 - (1.0 - *gamma) * sum;
                (a_right + *b2) / 2.0
            } else {
                (a_root + b_tree) / 2.0
            }
        } else {
            (a_root + b_tree) / 2.0
        }
    }

    fn block_output_voltage_gain(
        block: &KMethodBlock,
        da_root_dv: crate::Wave,
        db_tree_dv: crate::Wave,
    ) -> crate::Wave {
        if let Some(ref probe_id) = block.cascade_probe_id {
            if let Some(gain) = block.tree.leaf_voltage_for_incident_gain(probe_id) {
                return gain * da_root_dv;
            }
        }

        if block.cascade_from_passive {
            if let DynNode::Binary {
                kind: crate::dyn_node::BinaryKind::Series,
                gamma,
                left,
                right,
                ..
            } = &block.tree
            {
                let g1 = left.reflected_voltage_gain() * block.source_polarity;
                let g2 = right.reflected_voltage_gain() * block.source_polarity;
                return g2 - 0.5 * (1.0 - *gamma) * (g1 + g2 + da_root_dv);
            }
        }

        0.5 * (da_root_dv + db_tree_dv)
    }

    /// Set a pot position in any block-local passive tree.
    ///
    /// Blockwise K-method stages are compiled from normal WDF subtrees, so pot
    /// updates use the same dirty-path recompute as WDF stages. Split pot
    /// halves are intentionally handled here because BKM controls bind through
    /// the generic PotInStage target.
    pub fn set_pot(&mut self, comp_id: &str, value: crate::Wave) -> bool {
        let aw_id = alloc::format!("{comp_id}__aw");
        let wb_id = alloc::format!("{comp_id}__wb");
        let mut found = false;

        for block in self.k_method_blocks_mut() {
            let mut block_found = false;
            block_found |= block.tree.set_pot_dirty(comp_id, value);
            block_found |= block.tree.set_pot_dirty(&aw_id, value);
            block_found |= block.tree.set_pot_dirty(&wb_id, 1.0 - value);

            if block_found {
                block.tree.recompute_incremental();
                block.rp = block.tree.port_resistance();
                found = true;
            }
        }

        let mut coupling_found = false;
        for element in &mut self.coupling_elements {
            if element.comp_id == comp_id {
                if let Some(max_r) = element.pot_max_resistance {
                    let position = if element.invert_control {
                        1.0 - value
                    } else {
                        value
                    };
                    let tapered = element.taper.apply(position.clamp(0.0, 1.0));
                    element.resistance = (tapered * max_r).max(1.0e-3);
                    coupling_found = true;
                }
            }
        }

        if coupling_found {
            self.recompute_coupling_scattering();
            if self.shared_diode_cutoff_pot.as_deref() == Some(comp_id) {
                self.update_shared_diode_cutoff_bias(0.0);
            }
            found = true;
        }

        found
    }

    fn update_shared_diode_cutoff_bias_from_ports(&mut self, vs_signals: &[crate::Wave]) {
        let cutoff_cv = self
            .cutoff_cv_port
            .as_ref()
            .and_then(|cutoff_name| {
                self.vs_port_map
                    .iter()
                    .position(|(name, _)| name == cutoff_name)
                    .and_then(|idx| vs_signals.get(idx).copied())
            })
            .unwrap_or(0.0);
        self.update_shared_diode_cutoff_bias(cutoff_cv);
    }

    fn update_shared_diode_cutoff_bias(&mut self, cutoff_cv_voltage: crate::Wave) {
        let Some(comp_id) = self.shared_diode_cutoff_pot.as_deref() else {
            return;
        };
        let Some(first_block) = self.k_method_block(0) else {
            return;
        };
        let Some(calibration) = first_block.diode_cutoff.as_ref() else {
            return;
        };
        let Some(cutoff_resistance) = self
            .coupling_elements
            .iter()
            .find(|element| element.comp_id == comp_id)
            .map(|element| element.resistance)
        else {
            return;
        };

        let total_r = (calibration.bias_resistance + cutoff_resistance).max(1.0);
        let cv_resistance = calibration.cv_resistance.or_else(|| {
            self.coupling_elements
                .iter()
                .find(|element| {
                    let id = element.comp_id.to_ascii_lowercase();
                    id.contains("cv") && !id.contains("res")
                })
                .map(|element| element.resistance.max(1.0))
        });
        let bias_current = (self.supply_voltage + cutoff_cv_voltage - 0.6).max(0.0) / total_r;
        let cv_current = cv_resistance
            .map(|r_cv| cutoff_cv_voltage / r_cv)
            .unwrap_or(cutoff_cv_voltage / total_r);
        let i_f = (bias_current + cv_current).max(0.0);

        if let Some(model) = first_block.explicit_diode_root.map(|root| root.model) {
            let v_bias = diode_bias_voltage_from_current(model, i_f);
            for block in self.k_method_blocks_mut() {
                if block.explicit_diode_root.is_some() {
                    block.shared_diode_bias_voltage = Some(v_bias);
                }
            }
        } else {
            for block in self.k_method_blocks_mut() {
                if block.diode_cutoff.is_some() {
                    let i_bias = block.vbe_bias.max(1.0e-9);
                    let ctrl = (i_f / i_bias - 1.0).clamp(-0.9, 4.0);
                    block.shared_diode_bias_voltage = Some(ctrl);
                }
            }
        }
    }

    pub fn has_coupling_pot(&self, comp_id: &str) -> bool {
        self.coupling_elements
            .iter()
            .any(|e| e.comp_id == comp_id && e.pot_max_resistance.is_some())
    }

    fn recompute_coupling_scattering(&mut self) {
        if self.coupling_n_mna == 0 || self.coupling_ports.is_empty() {
            return;
        }

        let n_vsources = self
            .coupling_vcvss
            .iter()
            .map(|vcvs| vcvs.vsource_index + 1)
            .max()
            .unwrap_or(0);
        let mut mna = crate::tree::MnaSystem::new(self.coupling_n_mna, n_vsources);
        for vcvs in &self.coupling_vcvss {
            mna.stamp_vcvs(
                vcvs.pos,
                vcvs.neg,
                vcvs.out_pos,
                vcvs.out_neg,
                vcvs.gain,
                vcvs.output_resistance,
                vcvs.vsource_index,
            );
        }
        for element in &self.coupling_elements {
            mna.stamp_resistor(
                element.node_a,
                element.node_b,
                element.resistance.max(1.0e-3),
            );
        }
        for i in 0..self.coupling_n_mna {
            mna.stamp_resistor(Some(i), None, 1e9);
        }

        let ports = self.coupling_wdf_ports();
        let scattering = mna.derive_scattering_matrix_general(&ports);
        if scattering.len() == self.n_ports * self.n_ports
            && scattering.iter().all(|v| v.is_finite())
        {
            self.coupling_s = scattering;
            if let Some(terminals) = self.output_extraction.terminals {
                let (out_pos, out_neg) = terminals.raw().as_tuple();
                self.output_extraction.coeffs =
                    mna.derive_node_extraction_coeffs(&ports, out_pos, out_neg);
            }
        }
    }

    /// Solve the DC operating point by running zero-input samples until
    /// cap states converge. This is the equivalent of SPICE `.op` analysis.
    /// Must be called once at init (after deserialization) before audio.
    pub fn solve_dc_operating_point(&mut self) {
        self.init_buffers();
        if matches!(
            self.solve_mode,
            BlockwiseSolveMode::CoupledFixedPoint | BlockwiseSolveMode::CoupledNewton
        ) && !self.coupling_passives.is_empty()
        {
            // Coupling passives are AC storage ports inside the R-type adaptor.
            // A DC operating-point solve should open those capacitors; iterating
            // the audio WDF with floating AC-coupled boundary nodes can converge
            // to arbitrary huge offsets. Keep the small-signal BKM state centered
            // until we have a true static open-cap MNA operating-point pass.
            for block in self.k_method_blocks_mut() {
                block.dc_offset = 0.0;
            }
            for passive in &self.coupling_passives {
                if let Some(one_port) = self.coupling_one_ports.get(passive.one_port_idx) {
                    one_port.wdf_reset(&mut self.coupling_runtime_state);
                }
            }
            for v in &mut self.work_a {
                *v = 0.0;
            }
            for v in &mut self.work_b {
                *v = 0.0;
            }
            for v in &mut self.b_warm {
                *v = 0.0;
            }
            return;
        }
        // Run 2000 samples of silence — enough for caps to reach DC equilibrium
        let zeros = vec![0.0; self.vs_port_map.len()];
        for _ in 0..2000 {
            self.process_inner(&zeros, false, 0.0, &[]);
        }
        // Record the converged DC state — use the same extraction method
        // as the cascade (cap voltage for cascade_from_passive, root port otherwise).
        // This ensures the DC offset matches what process() subtracts.
        //
        // Run one more sample at DC to get the final voltages.
        self.process_inner(&zeros, false, 0.0, &[]);
        let n_blocks = self.block_count();
        for i in 0..n_blocks {
            // Re-extract at the current converged state
            let Some(block) = self.k_method_block_mut(i) else {
                continue;
            };
            let b_tree = block.tree.reflected_with_state(&mut block.runtime_state);
            let a_root = Self::block_root_incident(block, b_tree, 0.0);
            block.dc_offset = if block.cascade_from_passive {
                Self::block_output_voltage(block, a_root, b_tree)
            } else {
                (a_root + b_tree) / 2.0
            };
        }

        // Runtime BKM processing carries AC between rungs and subtracts each
        // block's DC offset from the probed cascade voltage. Keep both the
        // measured offsets and the converged reactive state: the cap state is
        // part of the operating point, not scratch state.
        for v in &mut self.work_a {
            *v = 0.0;
        }
        for v in &mut self.work_b {
            *v = 0.0;
        }
        for v in &mut self.b_warm {
            *v = 0.0;
        }

        #[cfg(feature = "std")]
        {
            for (i, block) in self.k_method_blocks().enumerate() {
                let mut t = block.tree.clone();
                let mut runtime_state = block.runtime_state.clone();
                t.set_voltage(0.0);
                let b = t.reflected_with_state(&mut runtime_state);
                std::eprintln!(
                    "  DC op: block {i} b_tree={b:.4}, dc_offset={:.4}",
                    block.dc_offset
                );
            }
        }
    }

    /// Allocate work buffers after deserialization.
    pub fn init_buffers(&mut self) {
        self.ensure_sub_stages();
        self.ensure_coupling_runtime_state();
        self.ensure_block_runtime_states();
        let n = self.n_ports;
        self.rebuild_coupling_passive_by_port();
        if self.work_b.len() != n {
            self.work_b = vec![0.0; n];
            self.work_a = vec![0.0; n];
            self.b_warm = vec![0.0; n];
        }
        self.coupled_scratch.resize(n);
        // Ensure K-table scales are precomputed
        for block in self.k_method_blocks_mut() {
            block.k_table.precompute_scales();
        }
        if let Some(core) = &mut self.diode_ladder_core {
            core.init();
        }
    }

    /// Process one sample through the blockwise K-method stage.
    ///
    /// The blocks are a serial cascade: each block's emitter output feeds
    /// the next block's VS input. The coupling scattering matrix handles
    /// bias (R_bias pull-ups) and the resonance feedback path (Q4.emitter
    /// → Resonance → R_fb_limit → Q1.base).
    ///
    /// Algorithm:
    /// 1. Compute coupling feedback from previous sample's state
    /// 2. Set block[0].VS = input + feedback
    /// 3. Serial cascade: each block processes, output feeds next block
    /// 4. Newton iteration: re-run cascade with updated feedback until convergence
    /// 5. Backward sweep to update reactive state (cap voltages)
    ///
    /// Process one sample. `vs_signals` contains one voltage per VS port
    /// in scattering order (matching `vs_port_map`). These are the input
    /// signals (audio, VCO, CV) that drive the coupling network.
    pub fn process(&mut self, vs_signals: &[crate::Wave]) -> crate::Wave {
        self.process_with_serial_input(0.0, vs_signals)
    }

    /// Process one sample with a serial audio drive plus named control/source
    /// voltages.
    ///
    /// Some compiler splits put the input resistor in an earlier stage, leaving
    /// BKM with only CV and supply VS ports. In that case the processor feeds
    /// the current serial audio sample here instead of requiring a synthetic
    /// `audio_in` coupling port.
    pub fn process_with_serial_input(
        &mut self,
        serial_input: crate::Wave,
        vs_signals: &[crate::Wave],
    ) -> crate::Wave {
        self.process_inner(vs_signals, true, serial_input, &[])
    }

    pub fn process_with_boundary_drives(
        &mut self,
        boundary_drives: &[BoundaryIncidentDrive],
        vs_signals: &[crate::Wave],
    ) -> crate::Wave {
        self.process_inner(vs_signals, true, 0.0, boundary_drives)
    }

    pub fn debug_process_without_feedback_ports(
        &mut self,
        vs_signals: &[crate::Wave],
    ) -> crate::Wave {
        if self.work_b.len() != self.n_ports {
            self.init_buffers();
        }

        self.write_vs_ports(vs_signals);
        for &(_, port_idx) in &self.feedback_port_map {
            if port_idx < self.n_ports {
                self.work_b[port_idx] = 0.0;
            }
        }
        self.scatter_coupling();
        if matches!(
            self.solve_mode,
            BlockwiseSolveMode::CoupledFixedPoint | BlockwiseSolveMode::CoupledNewton
        ) {
            self.run_coupled_blocks(true, 0.0, None)
        } else {
            self.run_block_cascade(true, true, 0.0, None)
        }
    }

    fn process_inner(
        &mut self,
        vs_signals: &[crate::Wave],
        include_cascade: bool,
        serial_input: crate::Wave,
        boundary_drives: &[BoundaryIncidentDrive],
    ) -> crate::Wave {
        if self.work_b.len() != self.n_ports
            || self
                .coupling_one_ports
                .iter()
                .filter_map(|one_port| one_port.state_slot())
                .any(|slot| slot.0 >= self.coupling_runtime_state.len())
            || self.k_method_blocks().any(|block| {
                block.runtime_state.states.len() != block.runtime_state.wave_cache.len()
            })
        {
            self.init_buffers();
        }

        if matches!(self.solve_mode, BlockwiseSolveMode::DiodeLadderCore) {
            return self.process_diode_ladder_core(vs_signals, serial_input);
        }

        if matches!(
            self.solve_mode,
            BlockwiseSolveMode::CoupledFixedPoint | BlockwiseSolveMode::CoupledNewton
        ) {
            return self.process_coupled_fixed_point(vs_signals, serial_input, boundary_drives);
        }

        self.update_shared_diode_cutoff_bias_from_ports(vs_signals);

        // Write VS signals into the coupling scattering ports.
        // b = 2·V (WDF voltage source reflected wave).
        // External ports (audio, CV) come from vs_signals.
        // Supply ports (_supply_*) get the fixed supply voltage.
        self.write_vs_ports(vs_signals);

        let mut last_output = self.b_warm[0];

        self.write_feedback_ports(last_output);

        for _iter in 0..Self::MAX_ITER {
            // 1. Coupling scatter: compute feedback + distribute VS signals.
            //    The coupling handles bias (R_bias), feedback (Resonance),
            //    and input injection (audio_in, vco_in, cv_cutoff).
            self.scatter_coupling();

            // 2. Serial cascade through the rungs.
            //    Each block's VS = previous rung's output + this block's
            //    coupling contribution (bias from supply, feedback, etc.).
            //    Block 0: VS from coupling only (input + feedback + bias).
            //    Blocks 1+: VS = previous output + coupling bias.
            let output = self.run_block_cascade(include_cascade, false, serial_input, None);

            // 3. Feed current cascade output back into coupling ports for the
            // next Newton iteration.
            self.write_feedback_ports(output);

            // 4. Convergence: Newton iterates until feedback stabilizes.
            if (output - last_output).abs() < Self::TOL {
                break;
            }
            last_output = output;
        }

        // Backward sweep: update cap states with converged cascade. The
        // observable output must come from this pass because reactive WDF
        // state is committed here.
        let output = self.run_block_cascade(include_cascade, true, serial_input, None);

        // Save warm-start for next sample
        self.b_warm[0] = output;

        // Guard against NaN
        if output.is_finite() {
            output
        } else {
            0.0
        }
    }

    fn process_diode_ladder_core(
        &mut self,
        vs_signals: &[crate::Wave],
        serial_input: crate::Wave,
    ) -> crate::Wave {
        let cutoff_cv = self
            .cutoff_cv_port
            .as_ref()
            .and_then(|cutoff_name| self.signal_voltage_by_name(cutoff_name, vs_signals))
            .unwrap_or(0.0);
        let cutoff_series_resistance = self.shared_diode_cutoff_pot.as_ref().and_then(|comp_id| {
            self.coupling_elements
                .iter()
                .find(|element| &element.comp_id == comp_id)
                .map(|element| element.resistance)
        });

        let mut input = serial_input;
        for name in ["audio_in", "vco_in"] {
            if let Some(v) = self.signal_voltage_by_name(name, vs_signals) {
                input += v;
            }
        }

        let Some(core) = &mut self.diode_ladder_core else {
            return 0.0;
        };
        let output = core.process(
            input as Wave,
            self.supply_voltage,
            cutoff_cv,
            cutoff_series_resistance,
        ) as crate::Wave;
        self.b_warm[0] = output;
        output
    }

    fn process_coupled_one_step_delay(
        &mut self,
        vs_signals: &[crate::Wave],
        serial_input: crate::Wave,
        boundary_drives: &[BoundaryIncidentDrive],
    ) -> crate::Wave {
        let previous_output = self.b_warm[0];
        let mut scratch = core::mem::take(&mut self.coupled_scratch);
        scratch.resize(self.n_ports);
        scratch.load_a_from(&self.work_a, self.n_ports);
        scratch.b.resize(self.n_ports, 0.0);

        let block_output = self.coupled_eval_b_for_a(
            &scratch.a,
            vs_signals,
            serial_input,
            boundary_drives,
            true,
            &mut scratch.b,
        );
        self.work_b.copy_from_slice(&scratch.b);
        self.coupled_scatter_from_b(&scratch.b, &mut scratch.f);
        let relaxation = Self::DELAYED_COUPLING_RELAXATION.clamp(0.0, 1.0);
        for (dst, next) in self.work_a.iter_mut().zip(scratch.f.iter()) {
            *dst += relaxation * (*next - *dst);
            if !dst.is_finite() {
                *dst = 0.0;
            }
        }
        let raw_output = self.output_probe_voltage(block_output);
        let output_is_stable =
            raw_output.is_finite() && raw_output.abs() <= Self::MAX_STABLE_OUTPUT;
        let output = if output_is_stable { raw_output } else { 0.0 };
        self.b_warm[0] = output;
        if !output_is_stable {
            for v in &mut self.work_a {
                *v = 0.0;
            }
            for v in &mut self.work_b {
                *v = 0.0;
            }
            for v in &mut scratch.f {
                *v = 0.0;
            }
            for v in &mut scratch.b {
                *v = 0.0;
            }
        }
        scratch.last_iterations = 1;
        scratch.last_residual = if output_is_stable {
            (output - previous_output).abs()
        } else {
            Self::MAX_STABLE_OUTPUT
        };
        scratch.last_converged = output_is_stable;
        scratch.last_linear_solve_failed = false;
        self.coupled_scratch = scratch;

        output
    }

    fn signal_voltage_by_name(
        &self,
        needle: &str,
        vs_signals: &[crate::Wave],
    ) -> Option<crate::Wave> {
        self.vs_port_map
            .iter()
            .enumerate()
            .find_map(|(idx, (name, _))| {
                if name == needle {
                    Some(vs_signals.get(idx).copied().unwrap_or(0.0))
                } else {
                    None
                }
            })
    }

    fn process_coupled_fixed_point(
        &mut self,
        vs_signals: &[crate::Wave],
        serial_input: crate::Wave,
        boundary_drives: &[BoundaryIncidentDrive],
    ) -> crate::Wave {
        self.update_shared_diode_cutoff_bias_from_ports(vs_signals);

        if self.solve_mode == BlockwiseSolveMode::CoupledNewton {
            let (_output, _converged) =
                self.coupled_solve_newton(vs_signals, serial_input, boundary_drives);
            let mut scratch = core::mem::take(&mut self.coupled_scratch);
            scratch.load_a_from(&self.work_a, self.n_ports);
            scratch.b.resize(self.n_ports, 0.0);
            let block_output = self.coupled_eval_b_for_a(
                &scratch.a,
                vs_signals,
                serial_input,
                boundary_drives,
                true,
                &mut scratch.b,
            );
            self.work_b.copy_from_slice(&scratch.b);
            self.coupled_scratch = scratch;
            let raw_output = self.output_probe_voltage(block_output);
            let output_is_stable =
                raw_output.is_finite() && raw_output.abs() <= Self::MAX_STABLE_OUTPUT;
            let output = if output_is_stable { raw_output } else { 0.0 };
            self.b_warm[0] = output;
            if !output_is_stable {
                for v in &mut self.work_a {
                    *v = 0.0;
                }
                for v in &mut self.work_b {
                    *v = 0.0;
                }
            }
            return output;
        }

        self.process_coupled_one_step_delay(vs_signals, serial_input, boundary_drives)
    }

    pub fn solve_diagnostics(&self) -> SolveDiagnostics {
        SolveDiagnostics {
            iterations: self.coupled_scratch.last_iterations,
            residual: self.coupled_scratch.last_residual,
            converged: self.coupled_scratch.last_converged,
            linear_solve_failed: self.coupled_scratch.last_linear_solve_failed,
        }
    }

    /// Debug label for tracing.
    pub fn debug_label(&self) -> String {
        let max_output_coeff = self
            .output_extraction
            .coeffs
            .iter()
            .map(|c| c.abs())
            .fold(0.0 as crate::Wave, crate::Wave::max);
        format!(
            "Blockwise({}blocks, {}ports, mode={:?}, vs_ports={}, feedback_ports={}, output_coeff_max={:.3e})",
            self.block_count(),
            self.n_ports,
            self.solve_mode,
            self.vs_port_map.len(),
            self.feedback_port_map.len(),
            max_output_coeff,
        )
    }
}

#[cfg(test)]
mod iir_inloop_slew_tests {
    use super::*;

    /// Build a normalized RBJ bandpass biquad (a0-normalized Direct Form I).
    /// High Q so the `a_coeffs` feedback is significant — this is precisely
    /// what makes the in-loop path differ from slewing the ideal output.
    fn resonant_bandpass(fc: crate::Wave, q: crate::Wave, fs: crate::Wave) -> IirData {
        let w0 = 2.0 * crate::math::PI * fc / fs;
        let cos_w0 = crate::math::cos(w0);
        let sin_w0 = crate::math::sin(w0);
        let alpha = sin_w0 / (2.0 * q);
        let a0 = 1.0 + alpha;
        // RBJ bandpass (constant-skirt, peak gain = Q): b = [alpha, 0, -alpha].
        let b = vec![alpha / a0, 0.0, -alpha / a0];
        let a = vec![1.0, (-2.0 * cos_w0) / a0, (1.0 - alpha) / a0];
        IirData::new(b, a, fs)
    }

    /// Slow-slew, rail-limited op-amp: slew and rails active, GBW transparent.
    fn slow_slew_fx() -> NonIdealFxState {
        NonIdealFxState {
            max_dv: 0.02,    // dV/dt clamp engages on a hard transient
            v_rail_pos: 0.7, // supply-rail soft-clip ceiling
            v_rail_neg: 0.7,
            // gbw_coeff stays 1.0 (default) → GBW post-path transparent.
            ..NonIdealFxState::default()
        }
    }

    const FS: crate::Wave = 48_000.0;
    const FC: crate::Wave = 1_000.0;
    const Q: crate::Wave = 4.0;

    /// Assertion 1: a hard transient drives the in-loop feedback away from the
    /// post-FX reference. If `process_inloop` were ever reverted to feed the
    /// *ideal* `y` back through `a_coeffs`, this divergence would collapse and
    /// the test would fail — that is the regression being locked in.
    #[test]
    fn iir_inloop_slew_diverges_from_postfx_under_hard_transient() {
        let mut sub = resonant_bandpass(FC, Q, FS);
        let mut reference = resonant_bandpass(FC, Q, FS);
        let mut fx_sub = slow_slew_fx();
        let mut fx_ref = slow_slew_fx();

        let mut max_div: crate::Wave = 0.0;
        // 0.8-amplitude step held — a hard transient that exercises slew + rail.
        for _ in 0..400 {
            let x = 0.8;
            let out_inloop = sub.process_inloop(x, &mut fx_sub);
            // Post-FX reference: slew/rail the *ideal* biquad output independently.
            let y = reference.process(x);
            let slewed = fx_ref.slew_step(y);
            let out_post = fx_ref.rail_step(slewed);
            max_div = max_div.max((out_inloop - out_post).abs());
        }

        assert!(
            max_div > 1e-3,
            "in-loop output never diverged from post-FX reference (max |Δ| = {max_div:e}); \
             process_inloop appears to feed the ideal output back instead of the limited node"
        );
    }

    /// Assertion 2: under a tiny, slow signal the slew clamp never engages and
    /// no rail clips, so the limited node equals the ideal `y` exactly and the
    /// linear filter response is preserved bit-for-bit vs the post-FX path.
    #[test]
    fn iir_inloop_bit_identical_under_small_slow_signal() {
        let mut sub = resonant_bandpass(FC, Q, FS);
        let mut reference = resonant_bandpass(FC, Q, FS);
        let mut fx_sub = slow_slew_fx();
        let mut fx_ref = slow_slew_fx();

        let mut max_diff: crate::Wave = 0.0;
        // 1e-4 amplitude, slow (50 Hz) — never approaches max_dv (0.02) or rails.
        for n in 0..2000 {
            let phase = 2.0 * crate::math::PI * 50.0 * (n as crate::Wave) / FS;
            let x = 1e-4 * crate::math::sin(phase);
            let out_inloop = sub.process_inloop(x, &mut fx_sub);
            let y = reference.process(x);
            let slewed = fx_ref.slew_step(y);
            let out_post = fx_ref.rail_step(slewed);
            max_diff = max_diff.max((out_inloop - out_post).abs());
        }

        assert!(
            max_diff < 1e-12,
            "linear (small/slow) response not preserved in-loop: max |Δ| = {max_diff:e}"
        );
    }

    /// Assertion 3: over the hard burst every in-loop output is finite and the
    /// per-sample step obeys the dV/dt clamp (rail_step is a contraction, so it
    /// cannot widen the step). Proves the slew constraint governs the fed-back
    /// node.
    #[test]
    fn iir_inloop_obeys_slew_bound_and_stays_finite() {
        let mut sub = resonant_bandpass(FC, Q, FS);
        let mut fx_sub = slow_slew_fx();
        let max_dv = fx_sub.max_dv;

        let mut prev = 0.0;
        for n in 0..400 {
            // 1 kHz sine at 0.8 amplitude: a continually-demanding transient.
            let phase = 2.0 * crate::math::PI * FC * (n as crate::Wave) / FS;
            let out = sub.process_inloop(0.8 * crate::math::sin(phase), &mut fx_sub);
            assert!(out.is_finite(), "in-loop output non-finite at sample {n}");
            assert!(
                (out - prev).abs() <= max_dv + 1e-9,
                "dV/dt clamp violated at sample {n}: |Δ| = {} > max_dv = {max_dv}",
                (out - prev).abs()
            );
            prev = out;
        }
    }
}

#[cfg(test)]
mod blockwise_stage_tests {
    use super::*;
    use crate::boundary_math::{GraphNodeId, PortSpec};
    use crate::dyn_node::DynNode;

    fn mapped_port(terminals: WdfPortTerminals, resistance: crate::Wave) -> CircuitMappedPort {
        CircuitMappedPort::new(
            terminals.map(GraphNodeId::new),
            PortSpec::new(terminals.map(MnaNodeId::new), resistance),
        )
    }

    fn control_sensitive_table() -> KTable {
        let steps = 3;
        let mut entries = Vec::with_capacity(steps * steps);
        for ic in 0..steps {
            let ctrl = ic as crate::Wave;
            for ib in 0..steps {
                let b = ib as crate::Wave;
                entries.push(b + 10.0 * ctrl);
            }
        }

        let mut table = KTable {
            dims: 2,
            b_min: 0.0,
            b_max: 2.0,
            ctrl_min: 0.0,
            ctrl_max: 2.0,
            steps,
            entries,
            inv_b_scale: 0.0,
            inv_c_scale: 0.0,
        };
        table.precompute_scales();
        table
    }

    fn one_dimensional_table() -> KTable {
        let mut table = KTable {
            dims: 1,
            b_min: 0.0,
            b_max: 2.0,
            ctrl_min: 0.0,
            ctrl_max: 0.0,
            steps: 3,
            entries: vec![0.0, 1.0, 2.0],
            inv_b_scale: 0.0,
            inv_c_scale: 0.0,
        };
        table.precompute_scales();
        table
    }

    fn test_block(vbe_bias: crate::Wave) -> KMethodBlock {
        KMethodBlock {
            tree: DynNode::VoltageSource(0.0, 1.0),
            runtime_state: RuntimeState::new(),
            k_table: control_sensitive_table(),
            explicit_diode_root: None,
            nominal_vs_rp: 1.0,
            diode_cutoff: None,
            rp: 1.0,
            vbe_bias,
            dc_offset: 0.0,
            cascade_from_passive: false,
            cascade_probe_id: None,
            source_polarity: 1.0,
            k_table_control_polarity: 1.0,
            shared_diode_bias_voltage: None,
            control_from_drive: true,
        }
    }

    fn blockwise_stage_fixture(
        blocks: Vec<KMethodBlock>,
        coupling_ports: Vec<CircuitMappedPort>,
        owned_ports: Vec<Vec<OwnedPortBinding>>,
    ) -> BlockwiseStage {
        let n_ports = coupling_ports.len();
        let sub_stages = blocks
            .into_iter()
            .zip(owned_ports)
            .map(|(block, owned_ports)| crate::processor::Stage::KMethod {
                block,
                ports: owned_ports
                    .into_iter()
                    .flat_map(|owned| {
                        let terminals = coupling_ports
                            .get(owned.port)
                            .map(|port| port.graph.raw())
                            .unwrap_or_else(WdfPortTerminals::grounded);
                        let (pos, neg) = terminals.as_tuple();
                        pos.into_iter().chain(neg).map(move |node| {
                            (
                                owned.role,
                                PortBinding::new(BindingId::new(node), owned.port),
                            )
                        })
                    })
                    .collect(),
            })
            .collect();
        BlockwiseStage {
            coupling_s: vec![0.0; n_ports * n_ports],
            coupling_n_mna: 0,
            coupling_ports,
            sub_stages,
            coupling_elements: vec![],
            coupling_passives: vec![],
            coupling_one_ports: vec![],
            coupling_runtime_state: RuntimeState::new(),
            coupling_passive_by_port: vec![],
            coupling_vcvss: vec![],
            n_ports,
            output_block: 0,
            output_port_index: None,
            supply_voltage: 9.0,
            vs_port_map: vec![],
            cutoff_cv_port: None,
            shared_diode_cutoff_pot: None,
            feedback_port_map: vec![],
            output_extraction: ExtractionProbe::default(),
            compensation: 1.0,
            oversampler: crate::oversampling::Oversampler::new(
                crate::oversampling::OversamplingFactor::X1,
            ),
            signal_flow_distance: 0,
            bypass_serial: false,
            solve_mode: BlockwiseSolveMode::Cascade,
            diode_ladder_core: None,
            b_warm: vec![0.0],
            work_b: vec![0.0],
            work_a: vec![0.0],
            coupled_scratch: CoupledSolveScratch::default(),
            port_index_cache: vec![],
        }
    }

    fn single_block_stage() -> BlockwiseStage {
        let mut block = test_block(1.0);
        block.k_table = one_dimensional_table();
        blockwise_stage_fixture(
            vec![block],
            vec![mapped_port(WdfPortTerminals::grounded(), 1.0)],
            vec![vec![OwnedPortBinding::primary(0)]],
        )
    }

    #[test]
    fn nonlinear_solver_enum_uses_k_table_without_dyn_dispatch() {
        let table = control_sensitive_table();
        let solver = NonlinearSolver::from_k_table(Some(&table));

        assert_eq!(solver.kind(), NonlinearSolverKind::KTable);
        assert_eq!(
            core::mem::size_of_val(&solver),
            core::mem::size_of::<Option<&KTable>>(),
            "solver dispatch should stay a concrete pointer-sized enum, not a boxed dyn trait"
        );

        let value = solver
            .solve_root_incident(1.0, 2.0)
            .expect("K-table solver should produce a root incident wave");
        assert!((value - 21.0).abs() < 1e-9, "value={value}");
    }

    #[test]
    fn nonlinear_solver_enum_reports_newton_fallback() {
        let solver = NonlinearSolver::from_k_table(None);

        assert_eq!(solver.kind(), NonlinearSolverKind::Newton);
        assert!(
            solver.solve_root_incident(1.0, 2.0).is_none(),
            "Newton is the concrete fallback strategy handled by the root device"
        );
    }

    #[test]
    fn blockwise_stage_debug_names_topology_not_solver() {
        let stage = single_block_stage();

        let debug = format!("{stage:?}");
        let label = stage.debug_label();

        assert!(debug.starts_with("BlockwiseStage("), "debug={debug}");
        assert!(
            !debug.contains("KMethod") && !label.contains("KMethod"),
            "K-method is a per-block solver detail, not the topology name: debug={debug}, label={label}"
        );
    }

    #[test]
    fn blockwise_sub_stage_exposes_ports_through_child_bindings() {
        let stage = blockwise_stage_fixture(
            vec![test_block(1.0)],
            vec![
                mapped_port(WdfPortTerminals::single_ended(10), 1.0),
                mapped_port(WdfPortTerminals::single_ended(20), 1.0),
                mapped_port(WdfPortTerminals::differential(30, 31), 1.0),
            ],
            vec![vec![
                OwnedPortBinding::new(0, OwnedPortRole::DifferentialPositive),
                OwnedPortBinding::new(1, OwnedPortRole::DifferentialNegative),
                OwnedPortBinding::new(2, OwnedPortRole::DifferentialOutput),
            ]],
        );

        let boundary = stage.ins();
        assert_eq!(stage.outs(), boundary);
        assert_eq!(
            boundary
                .iter()
                .map(|binding| (binding.binding_id.get(), binding.local_port))
                .collect::<Vec<_>>(),
            vec![(10, 0), (20, 1), (30, 2), (31, 2)]
        );
        assert_eq!(stage.sub_stages.len(), 1);
        assert_eq!(stage.sub_stages[0].ins(), boundary);
        assert_eq!(stage.sub_stages[0].outs(), boundary);
    }

    #[test]
    fn blockwise_primary_ports_are_stage_boundary_bindings() {
        let stage = blockwise_stage_fixture(
            vec![test_block(1.0)],
            vec![mapped_port(WdfPortTerminals::single_ended(44), 1.0)],
            vec![vec![OwnedPortBinding::primary(0)]],
        );

        let expected = vec![PortBinding::new(BindingId::new(44), 0)];

        assert_eq!(stage.ins(), expected);
        assert_eq!(stage.outs(), expected);
    }

    #[test]
    fn blockwise_stage_boundary_ports_are_bidirectional_bindings() {
        let stage = blockwise_stage_fixture(
            vec![test_block(1.0)],
            vec![mapped_port(WdfPortTerminals::differential(50, 51), 1.0)],
            vec![vec![OwnedPortBinding::primary(0)]],
        );

        let expected = vec![
            PortBinding::new(BindingId::new(50), 0),
            PortBinding::new(BindingId::new(51), 0),
        ];

        assert_eq!(stage.ins(), expected);
        assert_eq!(stage.outs(), expected);
    }

    #[test]
    fn blockwise_stage_boundary_bindings_preserve_shared_graph_nodes() {
        let stage = blockwise_stage_fixture(
            vec![test_block(1.0), test_block(1.0)],
            vec![
                mapped_port(WdfPortTerminals::single_ended(10), 1.0),
                mapped_port(WdfPortTerminals::single_ended(20), 1.0),
                mapped_port(WdfPortTerminals::single_ended(20), 1.0),
                mapped_port(WdfPortTerminals::single_ended(30), 1.0),
            ],
            vec![
                vec![
                    OwnedPortBinding::new(0, OwnedPortRole::DifferentialPositive),
                    OwnedPortBinding::new(1, OwnedPortRole::DifferentialOutput),
                ],
                vec![
                    OwnedPortBinding::new(2, OwnedPortRole::DifferentialPositive),
                    OwnedPortBinding::new(3, OwnedPortRole::DifferentialOutput),
                ],
            ],
        );

        assert_eq!(
            stage
                .ins()
                .iter()
                .filter(|binding| binding.binding_id == BindingId::new(20))
                .map(|binding| binding.local_port)
                .collect::<Vec<_>>(),
            vec![1, 2]
        );
    }

    #[test]
    fn bkm_2d_k_table_uses_runtime_drive_control_coordinate() {
        let mut block = test_block(1.0);

        let a_root = BlockwiseStage::block_root_incident(&mut block, 0.5, 0.25);

        assert!(
            (a_root - 3.0).abs() < 1e-9,
            "2D BKM K-table lookup must use the runtime drive as the control coordinate"
        );
    }

    #[test]
    fn bkm_control_coordinate_is_independent_from_tree_source_polarity() {
        let mut block = test_block(1.0);
        block.source_polarity = -1.0;
        block.k_table_control_polarity = 1.0;

        let drive = BlockwiseStage::block_drive_voltage(&block, 0.25);
        let control = BlockwiseStage::block_control_voltage(&block, 0.25);

        assert!(
            (drive + 0.25).abs() < 1e-9 && (control - 0.25).abs() < 1e-9,
            "BKM must allow WDF source orientation to differ from K-table bias/control orientation"
        );
    }

    #[test]
    fn bkm_can_decouple_k_table_control_from_drive_voltage() {
        let mut block = test_block(1.0);
        block.control_from_drive = false;

        let drive = BlockwiseStage::block_drive_voltage(&block, 0.75);
        let control = BlockwiseStage::block_control_voltage(&block, 0.75);

        assert!((drive - 0.75).abs() < 1e-12);
        assert!(
            control.abs() < 1e-12,
            "differential ladder rung K-table control is shared tail current, not local drive voltage"
        );
    }

    #[test]
    fn bkm_serial_input_drives_first_block_when_audio_port_is_split_out() {
        let mut stage = single_block_stage();

        let silent = stage.process(&[]);
        let driven = stage.process_with_serial_input(0.5, &[]);

        assert!(silent.abs() < 1e-12);
        assert!(
            driven.abs() > 1e-6,
            "BKM must accept serial audio input when the compiler split the named audio port into an earlier stage"
        );
    }

    #[test]
    fn bkm_diode_cutoff_uses_voltage_wave_not_port_resistance_axis() {
        let mut block = test_block(1.0);
        block.k_table = one_dimensional_table();
        block.explicit_diode_root = Some(ExplicitDiodeRoot::new(DiodeModel::silicon()));
        block.diode_cutoff = Some(DiodeCutoffCalibration {
            bias_voltage: 9.0,
            bias_resistance: 100_000.0,
            cv_resistance: Some(47_000.0),
            min_rp: 1.0,
            max_rp: 2.0,
        });
        block.tree = DynNode::VoltageSource(0.0, 1.5);

        let a_root = BlockwiseStage::block_root_incident(&mut block, 0.5, 0.25);

        assert!(
            (a_root - 0.5).abs() < 1e-9,
            "cv_cutoff is a voltage in the coupling network; diode BKM lookup \
             should use the incident wave table, not reinterpret CV as Rp"
        );
    }

    #[test]
    fn bkm_cutoff_cv_write_does_not_recompute_block_port_resistance() {
        let mut block = test_block(1.0);
        block.explicit_diode_root = Some(ExplicitDiodeRoot::new(DiodeModel::silicon()));
        block.diode_cutoff = Some(DiodeCutoffCalibration {
            bias_voltage: 9.0,
            bias_resistance: 100_000.0,
            cv_resistance: Some(47_000.0),
            min_rp: 1.0,
            max_rp: 100_000.0,
        });
        let mut stage = blockwise_stage_fixture(
            vec![block],
            vec![mapped_port(WdfPortTerminals::grounded(), 1.0)],
            vec![vec![OwnedPortBinding::primary(0)]],
        );
        stage.coupling_s = vec![1.0];
        stage.vs_port_map = vec![(alloc::string::String::from("cv_cutoff"), 0)];
        stage.cutoff_cv_port = Some(alloc::string::String::from("cv_cutoff"));
        let rp_before = stage.k_method_block(0).unwrap().tree.port_resistance();

        stage.write_vs_ports(&[3.0]);

        assert_eq!(stage.work_b[0], 6.0);
        assert!(
            (stage.k_method_block(0).unwrap().tree.port_resistance() - rp_before).abs() < 1e-12,
            "writing cv_cutoff volts should update the coupling VS wave, not mutate block Rp"
        );
    }

    #[test]
    fn bkm_voltage_sources_reflect_incident_coupling_wave() {
        let mut stage = blockwise_stage_fixture(
            vec![test_block(1.0)],
            vec![mapped_port(WdfPortTerminals::grounded(), 1.0)],
            vec![vec![OwnedPortBinding::primary(0)]],
        );
        stage.coupling_s = vec![1.0];
        stage.vs_port_map = vec![(alloc::string::String::from("cv_cutoff"), 0)];
        stage.cutoff_cv_port = Some(alloc::string::String::from("cv_cutoff"));
        stage.work_a = vec![1.25];

        stage.write_vs_ports(&[3.0]);

        assert!(
            (stage.work_b[0] - 4.75).abs() < 1e-12,
            "BKM voltage-source ports must use b=2V-a so source impedance participates in coupling"
        );
    }

    #[test]
    fn blockwise_stage_exposes_neutral_coupling_network_model() {
        let mut stage = blockwise_stage_fixture(
            vec![test_block(1.0)],
            vec![
                mapped_port(WdfPortTerminals::single_ended(0), 100.0),
                mapped_port(WdfPortTerminals::single_ended(1), 200.0),
            ],
            vec![vec![OwnedPortBinding::primary(0)]],
        );
        stage.coupling_s = vec![0.0, 1.0, -1.0, 0.0];
        stage.vs_port_map = vec![(alloc::string::String::from("cv_cutoff"), 1)];
        stage.feedback_port_map = vec![(0, 0)];
        stage.coupling_elements = vec![CouplingElement {
            comp_id: alloc::string::String::from("Cutoff"),
            node_a: Some(0),
            node_b: Some(1),
            graph_node_a: Some(0),
            graph_node_b: Some(1),
            resistance: 50_000.0,
            pot_max_resistance: Some(100_000.0),
            taper: crate::pot_taper::PotTaper::B,
            invert_control: false,
        }];
        stage.output_extraction = ExtractionProbe::new(
            vec![0.25, -0.5],
            Some(MnaPortTerminals::single_ended(MnaNodeId::new(1))),
        );

        let network = stage.coupling_network_model();
        let mut incident = vec![0.0; 2];

        assert_eq!(network.n_ports(), 2);
        assert!(network.has_square_scattering());
        assert!(network.scatter_into(&[2.0, 3.0], &mut incident));
        assert_eq!(incident, vec![3.0, -2.0]);
        assert_eq!(network.variable_resistors[0].child_idx, 0);
        assert_eq!(
            network.variable_resistors[0].terminals,
            MnaPortTerminals::differential(MnaNodeId::new(0), MnaNodeId::new(1))
        );
        assert_eq!(
            network
                .extraction
                .as_ref()
                .and_then(|probe| probe.read_reflected(&[2.0, 3.0])),
            Some(-1.0)
        );
        assert_eq!(
            network
                .extraction
                .as_ref()
                .and_then(|probe| probe.terminals)
                .map(|terminals| terminals.raw()),
            Some(WdfPortTerminals::single_ended(1))
        );
    }

    #[test]
    fn bkm_inactive_feedback_port_clears_stale_wave() {
        let mut stage = blockwise_stage_fixture(
            vec![test_block(1.0)],
            vec![
                mapped_port(WdfPortTerminals::grounded(), 1.0),
                mapped_port(WdfPortTerminals::single_ended(0), 1.0),
            ],
            vec![vec![OwnedPortBinding::primary(0)]],
        );
        stage.coupling_s = vec![1.0, 0.0, 0.0, 1.0];
        stage.coupling_n_mna = 1;
        stage.coupling_elements = vec![CouplingElement {
            comp_id: alloc::string::String::from("Resonance"),
            node_a: Some(0),
            node_b: None,
            graph_node_a: Some(0),
            graph_node_b: None,
            resistance: 100_000.0,
            pot_max_resistance: Some(100_000.0),
            taper: crate::pot_taper::PotTaper::B,
            invert_control: true,
        }];
        stage.feedback_port_map = vec![(0, 1)];
        stage.b_warm = vec![0.0, 0.0];
        stage.work_b = vec![0.0, 123.0];
        stage.work_a = vec![0.0, 5.0];

        stage.write_feedback_ports(0.75);

        assert_eq!(
            stage.work_b[1], 0.0,
            "an open resonance leg must remove the synthetic feedback source; \
             leaving stale b waves lets the coupling matrix bypass the lowpass cascade"
        );
    }

    #[test]
    fn bkm_active_feedback_port_reflects_incident_coupling_wave() {
        let mut stage = blockwise_stage_fixture(
            vec![test_block(1.0)],
            vec![mapped_port(WdfPortTerminals::single_ended(0), 1.0)],
            vec![vec![OwnedPortBinding::primary(0)]],
        );
        stage.coupling_s = vec![1.0];
        stage.coupling_n_mna = 1;
        stage.coupling_elements = vec![CouplingElement {
            comp_id: alloc::string::String::from("Resonance"),
            node_a: Some(0),
            node_b: None,
            graph_node_a: Some(0),
            graph_node_b: None,
            resistance: 10_000.0,
            pot_max_resistance: Some(100_000.0),
            taper: crate::pot_taper::PotTaper::B,
            invert_control: true,
        }];
        stage.feedback_port_map = vec![(0, 0)];
        stage.work_a = vec![0.25];

        stage.write_feedback_ports(0.75);

        assert!(
            (stage.work_b[0] - 1.25).abs() < 1e-12,
            "active BKM feedback ports are ideal voltage sources and must write b=2V-a"
        );
    }

    #[test]
    fn bkm_coupling_cap_uses_wdf_passive_port_state() {
        let mut coupling_runtime_state = RuntimeState::new();
        let cap_kind = crate::boundary_math::OnePortKind::Capacitor(100e-9);
        let cap_slot = coupling_runtime_state.allocate_one_port(cap_kind);
        let coupling_one_ports = vec![RuntimeOnePort::new(
            crate::boundary_math::OnePort::new(
                crate::boundary_math::PortTerminals::single_ended(ScatteringPortId::new(0)),
                cap_kind,
            ),
            cap_slot,
        )];
        let mut stage = blockwise_stage_fixture(
            vec![],
            vec![mapped_port(
                WdfPortTerminals::single_ended(0),
                104.1666666667,
            )],
            vec![],
        );
        stage.coupling_s = vec![1.0];
        stage.coupling_n_mna = 1;
        stage.coupling_passives = vec![CouplingPassive {
            comp_id: alloc::string::String::from("C_out"),
            port_idx: 0,
            one_port_idx: 0,
        }];
        stage.coupling_one_ports = coupling_one_ports;
        stage.coupling_runtime_state = coupling_runtime_state;
        stage.coupling_passive_by_port = vec![Some(0)];
        stage.solve_mode = BlockwiseSolveMode::CoupledNewton;

        let mut b = vec![0.0];
        let output = stage.coupled_eval_b_for_a(&[1.25], &[], 0.0, &[], true, &mut b);
        assert_eq!(output, 0.0);
        assert!(
            b[0].abs() < 1.0e-12,
            "a WDF capacitor reflects its previous state before accepting the new incident wave"
        );
        assert_eq!(stage.coupling_runtime_state.len(), 1);
        assert_eq!(
            stage.coupling_passive_one_port_state(0),
            Some(OnePortState::CapacitorVoltage(0.625)),
            "BKM coupling cap state should be inspectable as shared OnePortState"
        );

        stage.coupled_eval_b_for_a(&[0.0], &[], 0.0, &[], false, &mut b);
        assert!(
            (b[0] - 1.25).abs() < 1.0e-12,
            "BKM coupling caps must behave as passive WDF ports: b_C[n+1] = a_C[n]"
        );
        assert!(
            stage.set_coupling_passive_one_port_state(0, OnePortState::CapacitorVoltage(-0.25)),
            "BKM coupling cap should update through shared OnePortState"
        );
        stage.coupled_eval_b_for_a(&[0.5], &[], 0.0, &[], false, &mut b);
        assert!(
            (b[0] + 0.5).abs() < 1.0e-12,
            "setting shared capacitor voltage should update the WDF wave cache"
        );
    }

    #[test]
    fn bkm_coupling_recompute_preserves_static_vcvs_stamps() {
        let mut base = blockwise_stage_fixture(
            vec![],
            vec![
                mapped_port(WdfPortTerminals::single_ended(0), 1.0),
                mapped_port(WdfPortTerminals::single_ended(1), 1.0),
            ],
            vec![],
        );
        base.coupling_s = vec![0.0; 4];
        base.coupling_n_mna = 2;
        base.coupling_elements = vec![CouplingElement {
            comp_id: alloc::string::String::from("Rload"),
            node_a: Some(1),
            node_b: None,
            graph_node_a: Some(1),
            graph_node_b: None,
            resistance: 10_000.0,
            pot_max_resistance: None,
            taper: crate::pot_taper::PotTaper::B,
            invert_control: false,
        }];
        base.solve_mode = BlockwiseSolveMode::CoupledNewton;
        base.b_warm = vec![0.0; 2];
        base.work_b = vec![0.0; 2];
        base.work_a = vec![0.0; 2];

        let mut passive = base.clone();
        passive.recompute_coupling_scattering();

        let mut active = base;
        active.coupling_vcvss.push(CouplingVcvs {
            pos: Some(0),
            neg: None,
            out_pos: Some(1),
            out_neg: None,
            gain: -10.0,
            output_resistance: 75.0,
            vsource_index: 0,
        });
        active.recompute_coupling_scattering();

        let feedthrough_without_vcvs = passive.coupling_s[2];
        let feedthrough_with_vcvs = active.coupling_s[2];
        assert!(
            (feedthrough_with_vcvs - feedthrough_without_vcvs).abs() > 0.1,
            "BKM coupling recompute must retain active VCVS stamps; passive={feedthrough_without_vcvs}, active={feedthrough_with_vcvs}"
        );
    }

    #[test]
    fn bkm_sparse_coupled_jacobian_matches_dense_finite_difference() {
        let mut stage = blockwise_stage_fixture(
            vec![{
                let mut block = test_block(1.0);
                block.k_table = one_dimensional_table();
                block
            }],
            vec![
                mapped_port(WdfPortTerminals::single_ended(0), 1.0),
                mapped_port(WdfPortTerminals::single_ended(0), 1.0),
                mapped_port(WdfPortTerminals::single_ended(0), 1.0),
            ],
            vec![vec![
                OwnedPortBinding::primary(0),
                OwnedPortBinding::new(1, OwnedPortRole::DifferentialOutput),
            ]],
        );
        stage.coupling_s = vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        stage.coupling_n_mna = 1;
        stage.coupling_elements = vec![CouplingElement {
            comp_id: alloc::string::String::from("Resonance"),
            node_a: Some(0),
            node_b: None,
            graph_node_a: Some(0),
            graph_node_b: None,
            resistance: 10_000.0,
            pot_max_resistance: Some(100_000.0),
            taper: crate::pot_taper::PotTaper::B,
            invert_control: true,
        }];
        stage.vs_port_map = vec![(alloc::string::String::from("audio_in"), 2)];
        stage.feedback_port_map = vec![(0, 2)];
        stage.solve_mode = BlockwiseSolveMode::CoupledNewton;
        stage.b_warm = vec![0.0; 3];
        stage.work_b = vec![0.0; 3];
        stage.work_a = vec![0.0; 3];
        let a = vec![0.25, -0.125, 0.5];
        let vs = vec![0.75];
        let n = stage.n_ports;
        let mut sparse = vec![0.0; n * n];
        stage.coupled_fill_sparse_db_da(&a, 0.0, &[], &mut sparse);

        let mut dense = vec![0.0; n * n];
        for col in 0..n {
            let h = 1.0e-5 * a[col].abs().max(1.0);
            let mut a_hi = a.clone();
            let mut a_lo = a.clone();
            a_hi[col] += h;
            a_lo[col] -= h;

            let mut hi_stage = stage.clone();
            let mut lo_stage = stage.clone();
            let mut b_hi = vec![0.0; n];
            let mut b_lo = vec![0.0; n];
            hi_stage.coupled_eval_b_for_a(&a_hi, &vs, 0.0, &[], false, &mut b_hi);
            lo_stage.coupled_eval_b_for_a(&a_lo, &vs, 0.0, &[], false, &mut b_lo);

            for row in 0..n {
                dense[row * n + col] = (b_hi[row] - b_lo[row]) / (2.0 * h);
            }
        }

        for idx in 0..n * n {
            assert!(
                (sparse[idx] - dense[idx]).abs() < 1.0e-6,
                "sparse db/da[{idx}] = {}, dense = {}",
                sparse[idx],
                dense[idx]
            );
        }
    }

    #[test]
    fn bkm_block_solve_returns_raw_and_ac_cascade_voltages() {
        let mut block = test_block(1.0);
        block.k_table = one_dimensional_table();
        block.dc_offset = 0.75;

        let (_, _, raw, ac) = BlockwiseStage::solve_block_without_state_update(&mut block, 0.25);

        assert!(
            (raw - ac - 0.75).abs() < 1e-12,
            "BKM should keep raw cascade voltage available for driving downstream nonlinear blocks \
             while exposing AC output separately"
        );
    }

    #[test]
    fn bkm_block_port_reflects_raw_voltage_but_reports_ac_output() {
        let mut block = test_block(1.0);
        block.k_table = one_dimensional_table();
        block.dc_offset = 0.75;

        let (raw, ac, reflected) = BlockwiseStage::solve_block_port(&mut block, 0.25, false);

        assert!((raw - ac - 0.75).abs() < 1e-12);
        assert!(
            (reflected - (2.0 * raw - 0.25)).abs() < 1e-12,
            "coupled BKM ports must reflect physical voltage; DC subtraction is only for exposed audio/debug output"
        );
    }

    #[test]
    fn bkm_shared_diode_cutoff_current_updates_all_rungs_from_pot_and_cv() {
        let diode_root = ExplicitDiodeRoot::new(DiodeModel::silicon());
        let calibration = DiodeCutoffCalibration {
            bias_voltage: 9.0,
            bias_resistance: 10_000.0,
            cv_resistance: Some(47_000.0),
            min_rp: 10.0,
            max_rp: 100_000.0,
        };
        let mut stage = blockwise_stage_fixture(
            vec![
                {
                    let mut block = test_block(1.0);
                    block.explicit_diode_root = Some(diode_root);
                    block.diode_cutoff = Some(calibration.clone());
                    block
                },
                {
                    let mut block = test_block(1.0);
                    block.explicit_diode_root = Some(diode_root);
                    block.diode_cutoff = Some(calibration);
                    block
                },
            ],
            vec![mapped_port(WdfPortTerminals::grounded(), 1.0)],
            vec![
                vec![OwnedPortBinding::primary(0)],
                vec![OwnedPortBinding::primary(0)],
            ],
        );
        stage.coupling_s = vec![1.0];
        stage.coupling_n_mna = 0;
        stage.coupling_elements = vec![CouplingElement {
            comp_id: alloc::string::String::from("Cutoff"),
            node_a: Some(0),
            node_b: None,
            graph_node_a: Some(0),
            graph_node_b: None,
            resistance: 50_000.0,
            pot_max_resistance: Some(100_000.0),
            taper: crate::pot_taper::PotTaper::B,
            invert_control: false,
        }];
        stage.output_block = 1;
        stage.vs_port_map = vec![(alloc::string::String::from("cv_cutoff"), 0)];
        stage.cutoff_cv_port = Some(alloc::string::String::from("cv_cutoff"));
        stage.shared_diode_cutoff_pot = Some(alloc::string::String::from("Cutoff"));

        stage.update_shared_diode_cutoff_bias_from_ports(&[0.0]);
        let base_bias = stage
            .k_method_block(0)
            .unwrap()
            .shared_diode_bias_voltage
            .unwrap();
        assert_eq!(
            stage.k_method_block(0).unwrap().shared_diode_bias_voltage,
            stage.k_method_block(1).unwrap().shared_diode_bias_voltage,
            "one cutoff current must set every explicit diode rung coherently"
        );

        stage.update_shared_diode_cutoff_bias_from_ports(&[2.0]);
        let cv_bias = stage
            .k_method_block(0)
            .unwrap()
            .shared_diode_bias_voltage
            .unwrap();
        assert!(
            cv_bias > base_bias,
            "positive cutoff CV must raise the shared diode operating bias"
        );
    }

    #[test]
    fn diode_cutoff_calibration_uses_circuit_sources_not_octave_scaling() {
        let calibration = DiodeCutoffCalibration {
            bias_voltage: 9.0,
            bias_resistance: 100_000.0,
            cv_resistance: Some(47_000.0),
            min_rp: 10.0,
            max_rp: 100_000.0,
        };
        let model = DiodeModel::silicon();

        let r_0v = calibration.source_resistance(model, 0.0);
        let r_3v = calibration.source_resistance(model, 3.0);
        let ratio = r_3v / r_0v;

        assert!(
            ratio > 0.35 && ratio < 0.85,
            "3V cutoff CV should follow diode dynamic resistance from R_bias/R_cv, \
             not the old 2^-3 octave heuristic: r0={r_0v:.3}, r3={r_3v:.3}, ratio={ratio:.3}"
        );
    }

    #[test]
    fn diode_ladder_core_adds_four_lowpass_poles() {
        let steps = 64;
        let mut entries = Vec::with_capacity(steps);
        for ib in 0..steps {
            let v = -1.0 + (ib as crate::Wave / (steps - 1) as crate::Wave) * 2.0;
            entries.push((v / (2.0 * 0.026)).tanh() as Wave);
        }
        let mut table = KTable {
            dims: 1,
            b_min: -1.0,
            b_max: 1.0,
            ctrl_min: 0.0,
            ctrl_max: 0.0,
            steps,
            entries,
            inv_b_scale: 0.0,
            inv_c_scale: 0.0,
        };
        table.precompute_scales();
        let mut current_table = KTable {
            dims: 2,
            b_min: 100_000.0,
            b_max: 1_100_000.0,
            ctrl_min: 0.0,
            ctrl_max: 14.0,
            steps: 2,
            entries: vec![
                5.0e-6 as Wave,
                5.0e-6 as Wave,
                5.0e-6 as Wave,
                5.0e-6 as Wave,
            ],
            inv_b_scale: 0.0,
            inv_c_scale: 0.0,
        };
        current_table.precompute_scales();
        let mut low = DiodeLadderCore::new(
            table.clone(),
            vec![33.0e-9; 4],
            48_000.0,
            1.0,
            0.026,
            current_table,
            5.0e-6,
            10.0e-6,
            100_000.0,
        );
        let mut high = low.clone();
        let measure = |core: &mut DiodeLadderCore, freq: crate::Wave| {
            for i in 0..9600 {
                let x = 0.01 * (2.0 * crate::math::PI * freq * i as crate::Wave / 48_000.0).sin();
                let _ = core.process(x as Wave, 9.0, 0.0, Some(1_000_000.0));
            }
            let mut sum = 0.0 as Wave;
            for i in 0..9600 {
                let x = 0.01 * (2.0 * crate::math::PI * freq * i as crate::Wave / 48_000.0).sin();
                let y = core.process(x as Wave, 9.0, 0.0, Some(1_000_000.0));
                sum += y * y;
            }
            crate::math::sqrt(sum / 9600.0) as crate::Wave
        };

        let y_low = measure(&mut low, 100.0);
        let y_high = measure(&mut high, 10_000.0);
        assert!(
            y_low > y_high * 20.0,
            "ladder core should be a strong four-pole lowpass: low={y_low}, high={y_high}"
        );
    }
}
