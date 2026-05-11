//! WDF clipping/processing stage combining a tree with a nonlinear root.

extern crate alloc;
use alloc::{format, string::String, vec, vec::Vec};

use crate::elements::*;
use crate::oversampling::Oversampler;
use crate::tree::{MnaSystem, RTypeAdaptor, ScatteringInterpolationTable, WdfPort};
use crate::PedalProcessor;

use crate::dyn_node::DynNode;
use crate::helpers::{balance_parallel_vs, has_pot};

/// Flush denormals to zero. Subnormal floats are 100x slower to process
/// on x86 and serve no useful purpose in audio signals.
#[inline(always)]
pub fn flush_denormal(x: f64) -> f64 {
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
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct NonIdealFxState {
    /// GBW lowpass coefficient: α = 2π·fc / (2π·fc + fs).
    pub gbw_coeff: f64,
    /// Single-pole IIR state for GBW rolloff.
    pub gbw_state: f64,
    /// Maximum voltage change per sample (slew_rate_V_per_us * 1e6 / sample_rate).
    pub max_dv: f64,
    /// Previous output sample for slew rate limiting.
    pub prev_out: f64,
    /// Positive rail saturation voltage (tanh soft clip ceiling).
    pub v_rail_pos: f64,
    /// Negative rail saturation voltage.
    pub v_rail_neg: f64,
}

impl Default for NonIdealFxState {
    /// Default: passthrough (no GBW limiting, no slew, no rails).
    fn default() -> Self {
        Self {
            gbw_coeff: 1.0,
            gbw_state: 0.0,
            max_dv: f64::MAX,
            prev_out: 0.0,
            v_rail_pos: f64::MAX,
            v_rail_neg: f64::MAX,
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
    pub fn from_params(gbw: f64, slew_rate: f64, v_max: f64, gain: f64, sample_rate: f64) -> Self {
        let gain_abs = gain.abs().max(1.0);
        let fc = gbw / gain_abs;
        let w = 2.0 * core::f64::consts::PI * fc;
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
        gain: f64,
        sample_rate: f64,
    ) -> Self {
        let mut state = Self::default();
        for effect in fx {
            match effect {
                crate::nonideal_fx::NonIdealFx::OpAmpBandwidth { gbw, slew_rate } => {
                    let gain_abs = gain.abs().max(1.0);
                    let fc = gbw / gain_abs;
                    let w = 2.0 * core::f64::consts::PI * fc;
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
    pub fn update_gain(&mut self, gbw: f64, new_gain: f64, sample_rate: f64) {
        let gain_abs = new_gain.abs().max(1.0);
        let fc = gbw / gain_abs;
        let w = 2.0 * core::f64::consts::PI * fc;
        self.gbw_coeff = w / (w + sample_rate);
    }
}

/// Apply NonIdealFx post-processing to a sample.
///
/// GBW rolloff → slew rate limiting → rail saturation (tanh soft clip).
/// Called at the correct point by each stage type.
/// No heap allocations, no branching on stage type.
#[inline]
pub fn apply_nonideal_fx(sample: f64, state: &mut NonIdealFxState) -> f64 {
    // GBW rolloff: single-pole lowpass
    let mut out = state.gbw_coeff * sample + (1.0 - state.gbw_coeff) * state.gbw_state;
    state.gbw_state = out;

    // Slew rate limiting
    let dv = out - state.prev_out;
    if dv > state.max_dv {
        out = state.prev_out + state.max_dv;
    } else if dv < -state.max_dv {
        out = state.prev_out - state.max_dv;
    }
    state.prev_out = out;

    // Rail saturation: asymmetric tanh soft clip
    if out > 0.0 && state.v_rail_pos < f64::MAX {
        out = state.v_rail_pos * crate::fast_math::fast_tanh(out / state.v_rail_pos);
    } else if out < 0.0 && state.v_rail_neg < f64::MAX {
        out = -state.v_rail_neg * crate::fast_math::fast_tanh(-out / state.v_rail_neg);
    }

    flush_denormal(out)
}

// ═══════════════════════════════════════════════════════════════════════════

/// Compute adaptive NR tolerance based on input signal change rate.
///
/// During transients (large delta), loosen tolerance to reduce iterations.
/// During steady-state (small delta), use tight tolerance for accuracy.
/// The linear interpolation avoids a discontinuity at the threshold boundary.
#[inline]
fn adaptive_nr_tolerance(input_delta: f64) -> f64 {
    const TIGHT: f64 = 1e-6;
    const LOOSE: f64 = 1e-4;
    const TRANSIENT_THRESHOLD: f64 = 0.1; // ~-20dBFS delta

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
fn stamp_g(g_matrix: &mut [f64], n: usize, n1: Option<usize>, n2: Option<usize>, g: f64) {
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
    pub b_min: f64,
    pub b_max: f64,
    /// Min/max of the second dimension (control voltage). Unused for 1D.
    pub ctrl_min: f64,
    pub ctrl_max: f64,
    /// Number of steps per dimension.
    pub steps: usize,
    /// Flat table entries: a_root values.
    /// 1D: [steps] entries.
    /// 2D: [steps × steps] entries, row-major (b varies fastest).
    pub entries: alloc::vec::Vec<f64>,
}

impl KTable {
    /// 1D lookup: interpolate a_root from b_tree.
    pub fn lookup_1d(&self, b_tree: f64) -> f64 {
        if self.steps < 2 || self.entries.is_empty() {
            return 0.0;
        }
        let range = self.b_max - self.b_min;
        if range.abs() < 1e-15 {
            return self.entries[0];
        }
        let t = ((b_tree - self.b_min) / range).clamp(0.0, 1.0);
        let p = t * (self.steps - 1) as f64;
        let i = (p as usize).min(self.steps - 2);
        let frac = p - i as f64;
        self.entries[i] * (1.0 - frac) + self.entries[i + 1] * frac
    }

    /// 2D lookup: interpolate a_root from (b_tree, control_voltage).
    pub fn lookup_2d(&self, b_tree: f64, ctrl: f64) -> f64 {
        if self.steps < 2 || self.entries.is_empty() {
            return 0.0;
        }
        let b_range = self.b_max - self.b_min;
        let c_range = self.ctrl_max - self.ctrl_min;
        if b_range.abs() < 1e-15 || c_range.abs() < 1e-15 {
            return self.entries[0];
        }
        let tb = ((b_tree - self.b_min) / b_range).clamp(0.0, 1.0);
        let tc = ((ctrl - self.ctrl_min) / c_range).clamp(0.0, 1.0);
        let pb = tb * (self.steps - 1) as f64;
        let pc = tc * (self.steps - 1) as f64;
        let ib = (pb as usize).min(self.steps - 2);
        let ic = (pc as usize).min(self.steps - 2);
        let fb = pb - ib as f64;
        let fc = pc - ic as f64;
        let s = self.steps;
        let v00 = self.entries[ib + ic * s];
        let v10 = self.entries[ib + 1 + ic * s];
        let v01 = self.entries[ib + (ic + 1) * s];
        let v11 = self.entries[ib + 1 + (ic + 1) * s];
        v00 * (1.0 - fb) * (1.0 - fc)
            + v10 * fb * (1.0 - fc)
            + v01 * (1.0 - fb) * fc
            + v11 * fb * fc
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
        capacitance: f64,
        /// Port resistance: 1 / (2 * fs * C)
        rp: f64,
        /// State (previous incident wave)
        state: f64,
    },
    /// Inductor as WDF root for RL highpass and similar filters.
    /// The inductor reflects negated state: b = -state.
    /// Output voltage = (a - state) / 2 gives correct transfer function.
    InductorRoot {
        /// Inductance in Henrys
        inductance: f64,
        /// Port resistance: 2 * fs * L
        rp: f64,
        /// State (previous incident wave)
        state: f64,
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
        scattering: Vec<f64>,
        /// VS injection vector k (n_ports elements).
        vs_injection: Vec<f64>,
        /// Number of ports (reactive + output probe).
        n_ports: usize,
        /// Passive child nodes (capacitors, inductors, output probe, and pots).
        /// Pots are stored here for control binding but are NOT WDF ports —
        /// they live in the MNA G matrix as conductance entries.
        children: Vec<DynNode>,
        /// Index into `children` for the output probe port.
        output_port: usize,
        /// Stored MNA system for pot recomputation (None if no pots).
        recompute_mna: Option<MnaSystem>,
        /// WDF port definitions (reactive ports only, not pots).
        recompute_ports: Option<Vec<WdfPort>>,
        /// Pot conductance stamps in the MNA G matrix: (child_index, node_pos, node_neg, current_g).
        /// Used to unstamp old conductance and re-stamp new conductance on pot change.
        pot_stamps: Vec<(usize, Option<usize>, Option<usize>, f64)>,
        /// Dirty flag: set when a pot changes, cleared after S re-derivation.
        needs_recompute: bool,
        /// Precomputed interpolation table for single-pot stages.
        /// When Some, pot changes use table lookup instead of MNA re-inversion.
        interp_table: Option<ScatteringInterpolationTable>,
    },
}

// Shared bias constants for NL device control voltage setting.
// Used by both RootKind (WdfStage) and NlDeviceKind (MultiNlStage).
const TRIODE_GRID_BIAS: f64 = -2.0;
const PENTODE_GRID_BIAS: f64 = -8.0;
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
            | RootKind::Zener(_) => (true, 1),
            RootKind::Jfet(_) | RootKind::Triode(_)
            | RootKind::Mosfet(_) | RootKind::Bjt(_) => (true, 2),
            RootKind::Pentode(_) => (true, 3),
            // Not eligible: linear, variable, or hysteretic roots
            _ => (false, 0),
        }
    }

    /// Process the NL root: incident wave → reflected wave.
    /// Dispatches to the concrete root type's NR solver.
    pub fn process(&mut self, b_tree: f64, rp: f64) -> f64 {
        match self {
            RootKind::DiodePair(dp) => dp.process(b_tree, rp),
            RootKind::SingleDiode(d) => d.process(b_tree, rp),
            RootKind::ExplicitDiodePair(dp) => dp.process(b_tree, rp),
            RootKind::ExplicitSingleDiode(d) => d.process(b_tree, rp),
            RootKind::Zener(z) => z.process(b_tree, rp),
            RootKind::Jfet(j) => j.process(b_tree, rp),
            RootKind::JfetVr(j) => j.process_root(b_tree, rp),
            RootKind::Triode(t) => t.process(b_tree, rp),
            RootKind::VariMu(t) => t.process(b_tree, rp),
            RootKind::Pentode(p) => p.process(b_tree, rp),
            RootKind::Mosfet(m) => m.process(b_tree, rp),
            RootKind::Bjt(b) => b.process(b_tree, rp),
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
    pub fn set_control_voltage(&mut self, input: f64, compensation: f64, _bias_offset: f64) {
        match self {
            RootKind::Triode(t) => {
                t.set_vgk(TRIODE_GRID_BIAS + input * compensation);
            }
            RootKind::VariMu(t) => {
                t.set_vgk(TRIODE_GRID_BIAS + input * compensation);
            }
            RootKind::Bjt(b) => {
                // BJT Vbe = DC bias (from circuit analysis) + AC input signal.
                // The bias is set once at compile time via set_bias().
                b.set_vbe(b.vbe_bias() + input * compensation);
            }
            RootKind::Pentode(p) => {
                p.set_vg1k(PENTODE_GRID_BIAS + input * compensation);
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
    pub r_ap: f64,
    /// IIR numerator coefficient: Rf / (1 + K) where K = 2·fs·Rf·Cf.
    pub b0: f64,
    /// IIR denominator coefficient: (K - 1) / (K + 1).
    pub a1: f64,
    /// Previous input current sample for IIR.
    pub x_prev: f64,
    /// Previous output voltage for IIR.
    pub y_prev: f64,
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
    pub cap: f64,
    /// Sample rate for coefficient computation.
    pub sample_rate: f64,
    /// Previous input sample.
    pub x_prev: f64,
    /// Previous output sample.
    pub y_prev: f64,
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
    pub b0: f64,
    pub b1: f64,
    pub b2: f64,
    pub a1: f64,
    pub a2: f64,
    /// State: previous input samples.
    pub x1: f64,
    pub x2: f64,
    /// State: previous output samples.
    pub y1: f64,
    pub y2: f64,
}

impl ResonatorFeedback {
    /// Create a new bridged-T resonator IIR from circuit parameters.
    ///
    /// Uses the Audio EQ Cookbook bandpass formula with bilinear pre-warping.
    pub fn new(r1: f64, r2: f64, c1: f64, c2: f64, rf: f64, sample_rate: f64) -> Self {
        // Resonant angular frequency
        let omega_0 = 1.0 / crate::math::sqrt(r1 * r2 * c1 * c2);

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
        let w0 = 2.0 * crate::math::atan(omega_0 / (2.0 * sample_rate));

        // Audio EQ Cookbook: BPF (constant 0 dB peak gain)
        let sin_w0 = crate::math::sin(w0);
        let cos_w0 = crate::math::cos(w0);
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
    pub fn process(&mut self, input: f64) -> f64 {
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
    /// Zf subtree (feedback network: caps, pots, resistors between neg and output).
    pub zf: DynNode,
    /// True for inverting topology, false for non-inverting.
    pub is_inverting: bool,
    /// Op-amp model for GBW rolloff and slew rate limiting.
    pub opamp: OpAmpRoot,
    /// GBW rolloff filter state (single-pole LPF on output).
    pub gbw_state: f64,
    /// Previous output for slew rate limiting.
    pub prev_out: f64,
    /// Feedback pot ID (if any pot in Zf subtree responds to control changes).
    pub feedback_pot_id: Option<String>,
}

impl OpAmpWdfAdaptor {
    /// Process one sample through the op-amp adaptor.
    ///
    /// `v_plus`: non-inverting input voltage (bias for inverting, signal for non-inv).
    /// `v_in`: far-end voltage of Zi (signal for inverting, 0 for non-inverting).
    #[inline]
    pub fn process(&mut self, v_plus: f64, v_in: f64) -> f64 {
        // Up-sweep: get reflected waves from both subtrees
        let b1 = self.zi.reflected();
        let b2 = self.zf.reflected();
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
        self.zi.set_incident(a1);
        self.zf.set_incident(a2);

        // Apply op-amp non-idealities: GBW rolloff + slew rate + rail clipping
        let v_max = self.opamp.v_max();
        let gbw_coeff = self.opamp.gbw_coeff();

        // GBW rolloff: single-pole LPF at f_cl = GBW / gain
        let mut out = gbw_coeff * v_out + (1.0 - gbw_coeff) * self.gbw_state;
        self.gbw_state = out;

        // Hard clip at supply rails
        out = out.clamp(-v_max, v_max);

        // Slew rate limiting
        let sr = self.opamp.model.slew_rate;
        let fs = self.opamp.sample_rate();
        let max_dv = sr * 1e6 / fs;
        let dv = out - self.prev_out;
        if dv > max_dv {
            out = self.prev_out + max_dv;
        } else if dv < -max_dv {
            out = self.prev_out - max_dv;
        }
        self.prev_out = out;

        flush_denormal(out)
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.gbw_state = 0.0;
        self.prev_out = 0.0;
        self.zi.reset();
        self.zf.reset();
        self.opamp.reset();
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WdfStage {
    pub tree: DynNode,
    pub root: RootKind,
    /// Compensates for passive attenuation in the tree topology.
    /// Computed automatically from the tree's impedance structure.
    pub compensation: f64,
    /// Oversampler for antialiasing at nonlinear stages.
    pub oversampler: Oversampler,
    /// Base diode model (before thermal modulation). Stored so thermal
    /// drift can be applied as a multiplier without accumulation.
    pub base_diode_model: Option<DiodeModel>,
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
    pub dc_block: Option<(f64, f64, f64, f64)>,
    /// Inter-stage grid DC blocker: removes plate DC from the previous stage
    /// before setting the tube's grid voltage. Without this, multi-stage chains
    /// feed plate voltage (~80-200V) directly into Vgk, saturating the tube.
    /// The WDF tree's coupling cap blocks DC for wave propagation but the grid
    /// voltage is set externally via set_control_voltage(). This HPF mimics
    /// the coupling cap's DC-blocking effect on the grid bias.
    /// Format: (x_prev, y_prev). α = 0.9995, fc ≈ 3.5 Hz at 48 kHz.
    pub grid_dc_blocker: Option<(f64, f64)>,
    /// Source follower mode for JFETs.
    /// When true, Vgs is computed as Vgate (input) - Vsource (output).
    /// This enables proper source follower behavior where the source follows the gate.
    pub is_source_follower: bool,
    /// Previous source voltage for source follower Vgs calculation.
    /// Vgs[n] = input[n] - Vsource[n-1]
    pub prev_source_voltage: f64,
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
    pub transformer_gain: f64,
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
    /// Sample counter for runtime warnings rate limiting.
    /// Only meaningful when `runtime-warnings` feature is enabled.
    #[allow(dead_code)]
    pub sample_counter: u64,
    /// Component ID for the root device (for runtime warning attribution).
    /// Only meaningful when `runtime-warnings` feature is enabled.
    #[allow(dead_code)]
    pub root_comp_id: String,
    /// When set, identifies a pot in the WDF tree whose resistance drives
    /// OpAmpRoot gain recalculation. After pot update + recompute, the stage
    /// reads the pot's resistance and calls `OpAmpRoot::set_feedback_pot_r()`.
    pub feedback_pot_id: Option<String>,
    /// Fixed resistance in series with the feedback pot (e.g., R_clip).
    /// Used to compute effective Rf = pot_r + series_r when pot changes.
    pub feedback_series_r: f64,
    /// Input resistance (Ri) for gain computation. Read from pendant tree
    /// port resistance or input-touching resistor at compile time.
    pub feedback_ri: f64,
    /// When set, extract output voltage at this component (pot leaf) in the
    /// WDF tree instead of at the root junction. After the down-sweep,
    /// V_out = a_leaf / 2 (for a resistive leaf where b=0).
    /// This models voltage extraction at the circuit's output node when
    /// a pot sits between the NL junction and the output.
    pub output_probe: Option<String>,
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
    pub vcc_injection_coeff: f64,
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
    /// Ground-leg (Zg) subtree for the 3-port linear opamp adaptor (NonInverting).
    pub zg_child: Option<DynNode>,
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
    /// Custom WDF adaptor for op-amp circuits with reactive feedback.
    /// When `Some`, the stage uses the V_neg = V+ constraint-based scattering
    /// instead of the MNA/RType adaptor or precomputed scalar gain.
    /// The gain emerges from Zf/Zi impedance ratios in the wave propagation.
    pub opamp_wdf_adaptor: Option<OpAmpWdfAdaptor>,
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
        mut tree: DynNode,
        root: RootKind,
        oversampler: crate::oversampling::Oversampler,
    ) -> Self {
        tree.compute_dynamic_flags();
        Self {
            tree,
            root,
            compensation: 1.0,
            oversampler,
            base_diode_model: None,
            paired_opamp: None,
            allpass_feedback: None,
            allpass_direct: None,
            dc_block: None,
            grid_dc_blocker: None,
            is_source_follower: false,
            prev_source_voltage: 0.0,
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
            sample_counter: 0,
            root_comp_id: String::new(),
            feedback_pot_id: None,
            feedback_series_r: 0.0,
            feedback_ri: f64::INFINITY,
            output_probe: None,
            feedback_opamp: None,
            k_table: None,
            vcc_injection_coeff: 0.0,
            vcc_dc_ramp: 0,
            coupling_cap_id: None,
            resonator_feedback: None,
            negate_vs: false,
            input_photocouplers: Vec::new(),
            zf_child: None,
            zg_child: None,
            opamp_adaptor: None,
            opamp_children: Vec::new(),
            opamp_recompute: None,
            opamp_input_child_idx: None,
            opamp_wdf_adaptor: None,
        }
    }
}

/// Stored data for recomputing opamp adaptor scattering when pots change.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OpAmpRecomputeData {
    pub mna: crate::tree::MnaSystem,
    pub port_pairs: Vec<(Option<usize>, Option<usize>)>,
    pub port_resistances: Vec<f64>,
}

/// A photocoupler in the input path of an inverting opamp stage.
/// Stores the element itself (for asymmetric time constant modeling)
/// plus the fixed series resistance and DC feedback resistance for gain updates.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct InputPhotocoupler {
    pub comp_id: String,
    pub element: Photocoupler,
    /// Fixed resistance in series with this photocoupler (e.g., R_min).
    pub fixed_series_r: f64,
    /// DC feedback resistance (Rf) for gain recalculation.
    pub dc_rf: f64,
}

impl WdfStage {
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
    pub fn process(&mut self, input: f64) -> f64 {
        // Apply inter-stage transformer voltage gain (1.0 when no transformer).
        let input = input * self.transformer_gain;

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

            let mut output = adaptor.process(v_plus, v_in);

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
                let b: Vec<f64> = self
                    .opamp_children
                    .iter_mut()
                    .map(|c| c.reflected())
                    .collect();
                (b, true)
            } else if let (Some(zf), Some(zg)) = (self.zf_child.as_mut(), self.zg_child.as_mut()) {
                (vec![zf.reflected(), zg.reflected()], false)
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
                for (i, child) in self.opamp_children.iter_mut().enumerate() {
                    child.set_incident(a_children[i]);
                }
            } else {
                self.zf_child.as_mut().unwrap().set_incident(a_children[0]);
                self.zg_child.as_mut().unwrap().set_incident(a_children[1]);
            }

            // Output = voltage at adapted port (out→gnd = V_out)
            let mut output = (a_adapted + b_adapted) / 2.0;

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
        let root = &mut self.root;
        let k_table = &self.k_table;
        let compensation = self.compensation;
        let output_probe = &self.output_probe;
        let feedback_opamp = &mut self.feedback_opamp;
        let vcc_injection_coeff = self.vcc_injection_coeff;
        let vcc_dc_ramp = &mut self.vcc_dc_ramp;
        let coupling_cap_id = &self.coupling_cap_id;

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

        // For JFET source followers, compute Vgs from input (gate) and previous output (source).
        // Vgs = Vgate - Vsource, where Vgate ≈ input and Vsource is the WDF output.
        // We use the previous sample's source voltage for stability.
        if self.is_source_follower {
            if let RootKind::Jfet(ref mut j) = root {
                // Bias point: Vgs typically -0.5 to -2V for N-channel JFET
                // The input modulates around this bias point
                let vgs = input - self.prev_source_voltage;
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
                | RootKind::Zener(_) => -vs_voltage,
                _ => vs_voltage,
            };
            // Apply tree-topology sign correction (detected during construction).
            // Some trees produce negative b_tree with positive VS due to Series
            // adaptor nesting — negate VS to make b_tree positive for the NR solver.
            let vs_voltage = if negate_vs { -vs_voltage } else { vs_voltage };
            tree.set_voltage(vs_voltage);
            let b1 = tree.reflected();

            // For coupling-cap stages: after tree.reflected(), the capacitor's WDF
            // state encodes its DC charge (the DC component of previous input samples).
            // Extract it to compute Vgk = GRID_BIAS + (input_ac) where
            //   input_ac = sample * compensation - cap_voltage  (DC blocked by cap).
            // This must happen after reflected() so the cap state is current.
            if has_coupling_cap {
                if let Some(ref cap_id) = coupling_cap_id {
                    // cap_voltage is the voltage across the coupling cap from the
                    // previous down-sweep (one-sample delay, negligible at audio rates).
                    let cap_v = tree.leaf_voltage(cap_id).unwrap_or(0.0);
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
                    *vcc_dc_ramp as f64 / DC_RAMP_SAMPLES as f64
                };
                b1 + vcc_injection_coeff * dc_scale
            } else {
                b1
            };

            let rp = tree.port_resistance();
            let b_tree = b1;

            // K-method fast path: use precomputed table lookup instead of NR
            let a_root = if let Some(ref table) = k_table {
                if table.dims == 1 {
                    table.lookup_1d(b_tree)
                } else {
                    // 2D: use current control voltage from root
                    let ctrl = match root {
                        RootKind::Bjt(b) => b.vbe(),
                        RootKind::Triode(t) => t.vgk(),
                        RootKind::Jfet(j) => j.vgs(),
                        RootKind::Mosfet(m) => m.vgs(),
                        _ => 0.0,
                    };
                    table.lookup_2d(b_tree, ctrl)
                }
            } else {
                // NR fallback
                match root {
                RootKind::DiodePair(dp) => dp.process(b_tree, rp),
                RootKind::SingleDiode(d) => d.process(b_tree, rp),
                RootKind::ExplicitDiodePair(dp) => dp.process(b_tree, rp),
                RootKind::ExplicitSingleDiode(d) => d.process(b_tree, rp),
                RootKind::Zener(z) => z.process(b_tree, rp),
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
                    // Port voltage V = (a + b) / 2 = (b_tree + b_tree) / 2 = b_tree.
                    tree.set_incident(b_tree);
                    // Check output_probe BEFORE fallback
                    if let Some(ref probe_id) = output_probe {
                        if let Some(v) = tree.leaf_voltage(probe_id) {
                            return v;
                        }
                    }
                    // Open circuit: full voltage appears at the port.
                    (b_tree + b_tree) / 2.0
                }
                // ShortCircuit: ground termination (a = -b)
                // Ground has zero impedance, so it reflects with inverted sign.
                // This allows current to flow through series elements to ground.
                RootKind::ShortCircuit => {
                    // Short-circuit reflection: a = -b
                    let a_root = -b_tree;
                    tree.set_incident(a_root);

                    // Check output_probe first (for SP-reduced orphan stages)
                    if let Some(ref probe_id) = output_probe {
                        if let Some(v) = tree.leaf_voltage(probe_id) {
                            return v;
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
                    tree.set_incident(a_root);

                    // Check output_probe first (for SP-reduced feedforward stages)
                    if let Some(ref probe_id) = output_probe {
                        if let Some(v) = tree.leaf_voltage(probe_id) {
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
                    output_port,
                    ..
                } => {
                    let vs_voltage = sample * compensation;
                    let n = *n_ports;
                    // 1. Collect reflected waves from children
                    let b_children: Vec<f64> = children.iter_mut().map(|c| c.reflected()).collect();
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
                    for (child, &a_i) in children.iter_mut().zip(a_children.iter()) {
                        child.set_incident(a_i);
                    }
                    // 4. Output voltage at probe port
                    let a_out = a_children[*output_port];
                    let b_out = b_children[*output_port];
                    return (a_out + b_out) / 2.0;
                }
            } // end NR fallback else
            }; // end K-table if/else
            // ── Down-sweep ────────────────────────────────────────────
            tree.set_incident(a_root);

            // If an output probe is set, extract voltage at that leaf
            // after the down-sweep instead of the root junction.
            if let Some(ref probe_id) = output_probe {
                if let Some(v) = tree.leaf_voltage(probe_id) {
                    return v;
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
            let out = (a_root + b_tree) / 2.0;
            out
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

        // Runtime warning checks for hybrid linear/nonlinear devices.
        // Placed after all tree borrows are dropped to avoid borrow conflicts.
        #[cfg(feature = "runtime-warnings")]
        {
            self.sample_counter += 1;
            let sc = self.sample_counter;
            let comp_id = &self.root_comp_id;
            match &self.root {
                RootKind::JfetVr(j) => {
                    j.check_operating_region(wdf_out, None, comp_id, sc);
                }
                RootKind::Ota(o) => {
                    o.check_operating_region(wdf_out, comp_id, sc);
                }
                _ => {}
            }
            // Check hybrid devices in the tree (JfetVr, Photocoupler leaves).
            self.tree.check_hybrid_warnings(wdf_out, sc);
        }

        flush_denormal(wdf_out)
    }

    /// Adjust reactive element port resistances for oversampling.
    ///
    /// When oversampling is active, the WDF cycle runs at `base_rate * ratio`.
    /// Reactive elements (C/L) must use this effective rate for correct
    /// bilinear-transform discretization. Must be called after construction
    /// when the oversampling factor > 1.
    pub fn apply_oversampling_rate(&mut self, base_rate: f64) {
        let ratio = self.oversampler.ratio();
        if ratio <= 1 {
            return;
        }
        let effective_rate = base_rate * ratio as f64;
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
        self.tree.reset();
        self.oversampler.reset();
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
        if let RootKind::PassiveRType { children, .. } = &mut self.root {
            for child in children.iter_mut() {
                child.reset();
            }
        }
    }

    /// Set a pot value in the PassiveRType children and mark for recompute.
    ///
    /// Returns `true` if the pot was found and updated.
    pub fn set_passive_rtype_pot(&mut self, comp_id: &str, value: f64) -> bool {
        if let RootKind::PassiveRType {
            children,
            needs_recompute,
            ..
        } = &mut self.root
        {
            for child in children.iter_mut() {
                if child.set_pot(comp_id, value) {
                    *needs_recompute = true;
                    return true;
                }
            }
        }
        false
    }

    /// Re-derive scattering matrix from stored MNA after pot changes.
    ///
    /// Unstamps old pot conductances from the G matrix, stamps new ones based
    /// on current DynNode::Pot resistance, then re-derives S and k vectors.
    pub fn flush_passive_rtype_recompute(&mut self) {
        if let RootKind::PassiveRType {
            scattering,
            vs_injection,
            needs_recompute,
            recompute_mna,
            recompute_ports,
            pot_stamps,
            children,
            interp_table,
            ..
        } = &mut self.root
        {
            if !*needs_recompute {
                return;
            }

            // Fast path: interpolation table lookup for single-pot stages
            if let Some(table) = interp_table.as_ref() {
                if pot_stamps.len() == 1 {
                    let pot_r = children[pot_stamps[0].0].port_resistance();
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
                // Update G matrix: unstamp old conductance, stamp new
                for (child_idx, n1, n2, old_g) in pot_stamps.iter_mut() {
                    let new_r = children[*child_idx].port_resistance();
                    let new_g = 1.0 / new_r;
                    // Unstamp old conductance
                    stamp_g(&mut mna.g_matrix, n, *n1, *n2, -*old_g);
                    // Stamp new conductance
                    stamp_g(&mut mna.g_matrix, n, *n1, *n2, new_g);
                    *old_g = new_g;
                }
                // Re-derive scattering matrix and VS injection vector
                let (new_s, new_k) = mna.derive_scattering_and_vs_injection(ports, 0);
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
    /// Modulates diode Is and n_vt based on the current thermal state.
    /// Uses stored base model to prevent multiplier accumulation.
    pub fn apply_thermal(&mut self, state: &crate::thermal::ThermalState) {
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
    pub fn set_jfet_vgs(&mut self, vgs: f64) {
        match &mut self.root {
            RootKind::Jfet(j) => j.set_vgs(vgs),
            RootKind::JfetVr(j) => j.set_vgs(vgs),
            _ => {}
        }
    }

    /// Get the current gate-source voltage if this is a JFET stage.
    #[allow(dead_code)]
    pub fn jfet_vgs(&self) -> Option<f64> {
        match &self.root {
            RootKind::Jfet(j) => Some(j.vgs()),
            RootKind::JfetVr(j) => Some(j.vgs()),
            _ => None,
        }
    }

    /// Set the grid-cathode voltage for triode root elements.
    ///
    /// This is used for external modulation (bias, LFO, signal input).
    /// Has no effect if the root is not a triode.
    #[inline]
    pub fn set_triode_vgk(&mut self, vgk: f64) {
        if let RootKind::Triode(t) = &mut self.root {
            t.set_vgk(vgk);
        }
    }

    /// Get the current grid-cathode voltage if this is a triode stage.
    #[allow(dead_code)]
    pub fn triode_vgk(&self) -> Option<f64> {
        match &self.root {
            RootKind::Triode(t) => Some(t.vgk()),
            _ => None,
        }
    }

    /// Set the grid-cathode voltage for variable-mu triode root elements.
    #[inline]
    pub fn set_vari_mu_vgk(&mut self, vgk: f64) {
        if let RootKind::VariMu(t) = &mut self.root {
            t.set_vgk(vgk);
        }
    }

    /// Get the current grid-cathode voltage if this is a variable-mu triode stage.
    #[allow(dead_code)]
    pub fn vari_mu_vgk(&self) -> Option<f64> {
        match &self.root {
            RootKind::VariMu(t) => Some(t.vgk()),
            _ => None,
        }
    }

    /// Set the control grid voltage (g1-cathode) for pentode root elements.
    #[inline]
    pub fn set_pentode_vg1k(&mut self, vg1k: f64) {
        if let RootKind::Pentode(p) = &mut self.root {
            p.set_vg1k(vg1k);
        }
    }

    /// Set the screen grid voltage (g2-cathode) for pentode root elements.
    #[allow(dead_code)]
    #[inline]
    pub fn set_pentode_vg2k(&mut self, vg2k: f64) {
        if let RootKind::Pentode(p) = &mut self.root {
            p.set_vg2k(vg2k);
        }
    }

    /// Get the current control grid voltage if this is a pentode stage.
    #[allow(dead_code)]
    pub fn pentode_vg1k(&self) -> Option<f64> {
        match &self.root {
            RootKind::Pentode(p) => Some(p.vg1k()),
            _ => None,
        }
    }

    /// Set the gate-source voltage for MOSFET root elements.
    #[inline]
    pub fn set_mosfet_vgs(&mut self, vgs: f64) {
        if let RootKind::Mosfet(m) = &mut self.root {
            m.set_vgs(vgs);
        }
    }

    /// Get the current gate-source voltage if this is a MOSFET stage.
    #[allow(dead_code)]
    pub fn mosfet_vgs(&self) -> Option<f64> {
        match &self.root {
            RootKind::Mosfet(m) => Some(m.vgs()),
            _ => None,
        }
    }

    /// Set the OTA bias current (for envelope-controlled gain).
    #[allow(dead_code)]
    #[inline]
    pub fn set_ota_iabc(&mut self, iabc: f64) {
        if let RootKind::Ota(o) = &mut self.root {
            o.set_iabc(iabc);
        }
    }

    /// Set OTA gain as normalized value (0.0–1.0).
    #[inline]
    pub fn set_ota_gain(&mut self, gain: f64) {
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
    pub fn set_opamp_vp(&mut self, vp: f64) {
        if let RootKind::OpAmp(op) = &mut self.root {
            op.set_vp(vp);
        }
    }

    /// Get the current non-inverting input voltage if this is an op-amp stage.
    #[allow(dead_code)]
    pub fn opamp_vp(&self) -> Option<f64> {
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
    pub fn set_opamp_feedback(&mut self, ratio: f64, vm_external: f64) {
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
    pub fn set_paired_opamp_vp(&mut self, vp: f64) {
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
                if ri > 0.0 && ri < f64::MAX {
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

    /// Set a pot position in this stage, checking tree + all opamp children.
    ///
    /// Uses accumulator pattern (not early return) so split pots (__aw/__wb)
    /// that appear in multiple locations all get updated. Triggers
    /// recompute_all + notify_pot_changed when any pot is found.
    pub fn set_pot(&mut self, comp_id: &str, value: f64) -> bool {
        let mut found = false;
        // Main tree: use set_pot_dirty for incremental recompute.
        // Marks only the leaf-to-root path dirty instead of full recompute.
        if self.tree.set_pot_dirty(comp_id, value) {
            found = true;
        }
        if let Some(ref mut zf) = self.zf_child {
            if zf.set_pot_dirty(comp_id, value) {
                found = true;
            }
        }
        if let Some(ref mut zg) = self.zg_child {
            if zg.set_pot_dirty(comp_id, value) {
                found = true;
            }
        }
        for child in self.opamp_children.iter_mut() {
            if child.set_pot_dirty(comp_id, value) {
                found = true;
            }
        }
        // WDF constraint adaptor subtrees
        if let Some(ref mut adaptor) = self.opamp_wdf_adaptor {
            if adaptor.zi.set_pot_dirty(comp_id, value) {
                adaptor.zi.recompute_incremental();
                found = true;
            }
            if adaptor.zf.set_pot_dirty(comp_id, value) {
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
        let port_resistances: Vec<f64> = if !self.opamp_children.is_empty() {
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
            .map(|(&(pos, neg), &r)| crate::tree::WdfPort {
                node_pos: pos,
                node_neg: neg,
                resistance: r,
            })
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
    pub fn set_input_photocoupler_led(&mut self, comp_id: &str, led_drive: f64) -> bool {
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
    pub fn set_opamp_gain(&mut self, gain: f64) {
        if let RootKind::OpAmp(op) = &mut self.root {
            op.set_gain(gain);
        }
    }

    /// Get the current gain if this is an op-amp gain stage.
    #[allow(dead_code)]
    pub fn opamp_gain(&self) -> Option<f64> {
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
    pub fn opamp_sample_rate(&self) -> Option<f64> {
        match &self.root {
            RootKind::OpAmp(op) => Some(op.sample_rate()),
            _ => None,
        }
    }

    /// Get OpAmpRoot GBW coefficient (for verifying oversampling propagation).
    pub fn opamp_gbw_coeff(&self) -> Option<f64> {
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
            RootKind::Jfet(_) => "Jfet",
            RootKind::JfetVr(_) => "JfetVr",
            RootKind::Triode(_) => "Triode",
            RootKind::VariMu(_) => "VariMu",
            RootKind::Pentode(_) => "Pentode",
            RootKind::Mosfet(_) => "Mosfet",
            RootKind::Bjt(_) => "Bjt",
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
        s.push_str(")\n");
        s.push_str(&self.tree.debug_dump(1));

        // Print PassiveRType scattering matrix and VS injection vector.
        if let RootKind::PassiveRType {
            scattering,
            vs_injection,
            n_ports,
            children,
            output_port,
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
    pub fn set_vgk(&mut self, vgk: f64) {
        match self {
            TubeRoot::Koren(t) => t.set_vgk(vgk),
            TubeRoot::VariMu(t) => t.set_vgk(vgk),
            TubeRoot::Pentode(p) => p.set_vg1k(vgk),
        }
    }

    #[inline]
    pub fn v_max(&self) -> f64 {
        match self {
            TubeRoot::Koren(t) => t.v_max(),
            TubeRoot::VariMu(t) => t.v_max(),
            TubeRoot::Pentode(p) => p.v_max(),
        }
    }

    #[inline]
    pub fn set_v_max(&mut self, v_max: f64) {
        match self {
            TubeRoot::Koren(t) => t.set_v_max(v_max),
            TubeRoot::VariMu(t) => t.set_v_max(v_max),
            TubeRoot::Pentode(p) => p.set_v_max(v_max),
        }
    }

    #[inline]
    pub fn process(&mut self, b_tree: f64, rp: f64) -> f64 {
        match self {
            TubeRoot::Koren(t) => t.process(b_tree, rp),
            TubeRoot::VariMu(t) => t.process(b_tree, rp),
            TubeRoot::Pentode(p) => p.process(b_tree, rp),
        }
    }

    #[inline]
    pub fn plate_current(&self, vpk: f64) -> f64 {
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
    /// WDF tree for pull half (plate load + cathode passives).
    pub pull_tree: DynNode,
    /// Push tube model (Koren triode or Raffensperger variable-mu).
    pub push_root: TubeRoot,
    /// Pull tube model (Koren triode or Raffensperger variable-mu).
    pub pull_root: TubeRoot,
    /// Oversampler for push half.
    pub push_oversampler: Oversampler,
    /// Oversampler for pull half.
    pub pull_oversampler: Oversampler,
    /// Compensation factor (mu/ref_mu).
    pub compensation: f64,
    /// Output transformer turns ratio (primary:secondary).
    /// Output is scaled by 1/ratio (step-down).
    pub turns_ratio: f64,
    /// Grid bias voltage (class AB operating point).
    pub grid_bias: f64,
    /// DC blocker state: previous input sample (1-pole HPF, ~3.5Hz).
    pub dc_blocker_x1: f64,
    /// DC blocker state: previous output sample.
    pub dc_blocker_y1: f64,
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
    pub nl_port_resistances: Vec<f64>,
    pub v_prev: Vec<f64>,
    pub dc_bias: Vec<f64>,
    pub output_port: usize,
    pub n_nl: usize,
    pub vs_injection: Option<Vec<f64>>,
    pub vcc_bias_all: Vec<f64>,
    pub dc_ramp: u32,
}

impl PushPullStage {
    /// Process one sample through the push-pull stage.
    /// Input is applied with opposite polarity to push and pull halves.
    /// Output is the differential plate voltage divided by turns ratio.
    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
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
        let push_b = core::cell::Cell::new(0.0f64);
        #[cfg(feature = "debug-trace")]
        let push_a = core::cell::Cell::new(0.0f64);

        let push_out = self.push_oversampler.process(input, |_| {
            let vs = self.push_root.v_max();
            self.push_tree.set_voltage(vs);
            let b = self.push_tree.reflected();
            let rp = self.push_tree.port_resistance();
            let a = self.push_root.process(b, rp);
            self.push_tree.set_incident(a);
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
            let b = self.pull_tree.reflected();
            let rp = self.pull_tree.port_resistance();
            let a = self.pull_root.process(b, rp);
            self.pull_tree.set_incident(a);
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
    fn process_three_port(&mut self, input: f64) -> f64 {
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
    fn process_adaptor_half(adaptor: &mut PushPullHalfAdaptor, sample: f64) -> f64 {
        let n_nl = adaptor.n_nl;
        let n_passive = adaptor.passive_children.len();

        // DC ramp: gradually increase VCC bias over DC_RAMP_SAMPLES to let
        // the NR solver converge to the correct operating point without diverging.
        const DC_RAMP_SAMPLES: u32 = 256;
        let dc_scale = if adaptor.dc_ramp >= DC_RAMP_SAMPLES {
            1.0
        } else {
            adaptor.dc_ramp += 1;
            adaptor.dc_ramp as f64 / DC_RAMP_SAMPLES as f64
        };

        // 1. Scatter-up passive children
        let mut b_passive = Vec::with_capacity(n_passive);
        for child in &mut adaptor.passive_children {
            b_passive.push(child.reflected());
        }

        // 2. Compute known_a for each NL port
        let b_adapted = sample;
        let mut known_a = vec![0.0; n_nl];
        for i in 0..n_nl {
            let mut a_i = if let Some(ref k) = adaptor.vs_injection {
                k[i] * b_adapted
            } else {
                adaptor.scattering.s_nl_adapted[i] * b_adapted
            };
            for k in 0..n_passive {
                a_i += adaptor.scattering.s_nl_passive[i * n_passive + k] * b_passive[k];
            }
            a_i += adaptor.dc_bias[i] * dc_scale;
            known_a[i] = a_i;
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
        for (k, child) in adaptor.passive_children.iter_mut().enumerate() {
            child.set_incident(a_all[n_nl + k]);
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
    pub fn apply_oversampling_rate(&mut self, base_rate: f64) {
        let push_ratio = self.push_oversampler.ratio();
        if push_ratio > 1 {
            let effective_rate = base_rate * push_ratio as f64;
            self.push_tree.set_sample_rate(effective_rate);
            self.push_tree.recompute();
        }
        let pull_ratio = self.pull_oversampler.ratio();
        if pull_ratio > 1 {
            let effective_rate = base_rate * pull_ratio as f64;
            self.pull_tree.set_sample_rate(effective_rate);
            self.pull_tree.recompute();
        }
        // Also adjust adaptor passive children if present.
        if let Some(ref mut adaptor) = self.push_adaptor {
            let ratio = self.push_oversampler.ratio();
            if ratio > 1 {
                let effective_rate = base_rate * ratio as f64;
                for child in &mut adaptor.passive_children {
                    child.set_sample_rate(effective_rate);
                    child.recompute();
                }
            }
        }
        if let Some(ref mut adaptor) = self.pull_adaptor {
            let ratio = self.pull_oversampler.ratio();
            if ratio > 1 {
                let effective_rate = base_rate * ratio as f64;
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
    multi_port_nr_solve, multi_port_nr_solve_grouped, multi_port_nr_solve_grouped_into,
    multi_port_nr_solve_into, NlDeviceGroupIv, NlDeviceIv,
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
}

impl NlDeviceKind {
    /// Set the control voltage for this NL device from the input signal.
    ///
    /// `bias_offset` is an additional bias voltage from external controls
    /// (e.g., BJT bias pots). Zero for non-BJT devices.
    pub fn set_control_voltage(&mut self, input: f64, compensation: f64, bias_offset: f64) {
        match self {
            NlDeviceKind::Triode(t) => {
                t.set_vgk(TRIODE_GRID_BIAS + input * compensation);
            }
            NlDeviceKind::VariMu(t) => {
                t.set_vgk(TRIODE_GRID_BIAS + input * compensation);
            }
            NlDeviceKind::Pentode(p) => {
                p.set_vg1k(PENTODE_GRID_BIAS + input * compensation);
            }
            NlDeviceKind::Diode(_)
            | NlDeviceKind::DiodePair(_)
            | NlDeviceKind::ExplicitDiode(_)
            | NlDeviceKind::ExplicitDiodePair(_) => {}
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

    fn eval(&self, v: &[f64], currents: &mut [f64], jacobian: &mut [f64]) {
        let (i, di) = self.as_nl_device_iv().iv(v[0]);
        currents[0] = i;
        jacobian[0] = di;
    }

    fn v_clamp_port(&self, _port: usize) -> (f64, f64) {
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
    pub port_node_pairs: Vec<(Option<usize>, Option<usize>)>,
    /// Resistance of the adapted (voltage source) port.
    pub adapted_resistance: f64,
    /// When Some, the input VS is stamped as an ideal voltage source in MNA B/C
    /// matrices. The value is the MNA VS branch index. Recompute uses
    /// `derive_scattering_and_vs_injection()` instead of `derive_scattering_matrix_general()`.
    pub vs_source_index: Option<usize>,
    /// VCC voltage source index in MNA. When Some, VCC is an ideal VS and
    /// recompute re-extracts dc_bias from the VCC injection vector.
    pub vcc_vs_index: Option<usize>,
    /// Output MNA node pair for direct node-voltage extraction.
    /// When Some, recompute also updates the extraction coefficients.
    pub extract_output_nodes: Option<(Option<usize>, Option<usize>)>,
}

/// Scattering matrix sub-blocks for multi-NL solving.
///
/// These sub-blocks are extracted from the full R-type adaptor scattering
/// matrix and are the only parts needed during per-sample NR solving.
/// They are recomputed together when pot values change.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MultiNlScattering {
    /// NL-to-NL sub-block (n_nl × n_nl, row-major).
    pub s_nl: Vec<f64>,
    /// NL-to-passive sub-block (n_nl × n_passive, row-major).
    pub s_nl_passive: Vec<f64>,
    /// NL-to-adapted column (n_nl).
    pub s_nl_adapted: Vec<f64>,
}

impl MultiNlScattering {
    /// Extract sub-blocks from a full scattering matrix.
    ///
    /// The full matrix is `n_total × n_total` (row-major) with port ordering:
    /// `[NL_0..NL_{n-1}, passive_0..passive_{m-1}, (vcc?), adapted]`.
    /// `n_total` is inferred from the scattering matrix size, so extra ports
    /// (like VCC) are handled automatically — only the first `n_nl` rows and
    /// the NL, passive, and last (adapted) columns are extracted.
    pub fn from_full_matrix(scattering: &[f64], n_nl: usize, n_passive: usize) -> Self {
        let n_total = if scattering.is_empty() {
            n_nl + n_passive + 1
        } else {
            let len = scattering.len();
            let nt = crate::math::sqrt(len as f64) as usize;
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
    pub nl_port_resistances: Vec<f64>,
    /// Passive child nodes (capacitors, inductors) needing WDF state updates.
    pub passive_children: Vec<DynNode>,
    /// Pot DynNodes stored separately — pots are G-matrix conductances, not WDF ports.
    pub pot_children: Vec<DynNode>,
    /// MNA stamp tracking for pots: (pot_child_idx, node_pos, node_neg, last_conductance).
    /// Used for delta-updating the G matrix when pot values change.
    pub pot_mna_stamps: Vec<(usize, Option<usize>, Option<usize>, f64)>,
    /// Number of nonlinear ports.
    pub n_nl: usize,
    /// Warm-start voltages for NR solver.
    pub v_prev: Vec<f64>,
    /// Scattering matrix sub-blocks for NR solving.
    pub scattering: MultiNlScattering,
    /// Oversampler for antialiasing.
    pub oversampler: Oversampler,
    /// Passive attenuation compensation factor.
    pub compensation: f64,
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
    pub transformer_gain: f64,
    /// Circuit graph node ID (for debug routing).
    pub injection_node_id: usize,
    /// Circuit graph node ID (for debug routing).
    pub output_node_id: usize,
    /// Flag: pot changed since last scattering recompute.
    pub recompute_pending: bool,
    /// VEB bias offset from a feedback pot.
    pub veb_bias_offset: f64,
    /// Feedback scale for coupled BJT stages.
    pub feedback_scale: f64,
    /// Feedback opamp root for diode-paired stages (Bluesbreaker, Tube Screamer).
    /// Applies opamp gain + GBW + slew to the input before MNA/NR solve.
    pub feedback_opamp: Option<OpAmpRoot>,
    /// Pot ID that controls feedback opamp gain (if any).
    pub feedback_pot_id: Option<String>,
    /// Post-scattering non-ideality filter for op-amps absorbed via nullor
    /// stamps. Applies supply rail clamping and slew rate limiting to the
    /// extracted output node voltage. `None` for stages whose output is
    /// not dominated by an op-amp, or stages where we cannot identify a
    /// specific op-amp as the audio output source.
    pub opamp_post_fx: Option<crate::elements::OpAmpPostFx>,
    /// Linearized OTA data for gm-based scattering recompute.
    /// When Some, the OTA's transconductance is stamped into the MNA as a linear
    /// conductance. When the envelope changes gain, we delta-update the MNA and
    /// recompute the scattering matrix. No NR iteration needed.
    pub linearized_ota: Option<LinearizedOtaData>,
    /// VS injection vector for ideal voltage source input.
    /// When Some, signal is injected via `a[i] += k[i] * V_in` instead of
    /// an adapted WDF port. Used for linearized OTA stages where the adapted
    /// port would share an MNA node with a reactive port.
    pub vs_injection: Option<Vec<f64>>,
    /// Node-voltage extraction coefficients for direct output reading.
    /// When Some, the output is computed as:
    ///   V_out = Σ_k extract_coeffs[k] * b[k] + extract_vs * V_in
    /// This bypasses WDF port impedance mismatch by reading the MNA node
    /// voltage directly from X⁻¹ coefficients.
    pub extract_coeffs: Option<Vec<f64>>,
    pub extract_vs: f64,
    /// When set, identifies a pot in pot_children whose resistance drives
    /// BJT bias recalculation (feedback_scale + veb_bias_offset).
    pub bias_pot_id: Option<String>,
    /// Emitter resistance for bias pot computation (default 470Ω).
    pub bias_emitter_r: f64,
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
    pub dc_bias: Vec<f64>,
    /// Full VCC injection vector × supply_voltage for ALL ports.
    /// Added to a_all after scatter_all to provide DC bias to passive children
    /// and correct output extraction. Length = n_nl + n_passive + adapted(0 or 1).
    pub vcc_bias_all: Vec<f64>,
    /// VCC voltage source index in the MNA (for recomputing dc_bias on pot changes).
    /// When Some, VCC is stamped as an ideal VS in the MNA with zero impedance.
    pub vcc_vs_index: Option<usize>,
    /// Nominal supply voltage (volts). Used for dc_bias computation.
    pub supply_voltage: f64,
    /// DC ramp counter for gradual bias injection.
    /// Counts from 0 to DC_RAMP_SAMPLES, scaling dc_bias by ramp/N to let the
    /// NR solver track the operating point as supply voltage gradually increases.
    pub dc_ramp: u32,
    /// Physics-based initial v_prev values. Restored on reset() instead of zeroing,
    /// so the NR solver starts near the correct operating point after a DAW reset.
    pub initial_v_prev: Vec<f64>,
    /// Previous-previous sample's NR solution (v[n-2]).
    /// Used with v_prev (v[n-1]) to extrapolate a warm-start guess for v[n]:
    ///   v_guess = 2·v[n-1] − v[n-2]
    /// Reduces NR iterations on transients compared to a plain v_prev warm-start.
    pub v_prev_2: Vec<f64>,
    /// DC blocker state: previous input sample (x[n-1]).
    pub dc_blocker_x1: f64,
    /// DC blocker state: previous output sample (y[n-1]).
    pub dc_blocker_y1: f64,
    /// Pre-allocated workspace for NR solver (eliminates per-sample heap allocations).
    pub nr_workspace: crate::elements::nonlinear::solver::NrWorkspace,
    /// Pre-allocated process buffers.
    pub work_b_passive: Vec<f64>,
    pub work_known_a: Vec<f64>,
    pub work_b_all: Vec<f64>,
    pub work_a_all: Vec<f64>,
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
    pub prev_input: f64,
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
    pub b_coeffs: Vec<f64>,
    /// Denominator coefficients [1.0, a1, a2].
    pub a_coeffs: Vec<f64>,
    /// Input history [x[n-1], x[n-2]].
    pub x_hist: Vec<f64>,
    /// Output history [y[n-1], y[n-2]].
    pub y_hist: Vec<f64>,
    /// Sample rate for coefficient recomputation.
    pub sample_rate: f64,
    /// Stored component values for O(1) recomputation.
    /// R_series: product of series resistors (R1×R2).
    pub r_series_product: f64,
    /// C_shunt: product of shunt caps (C1×C2).
    pub c_shunt_product: f64,
    /// Feedback resistor value (current, changes with pot).
    pub r_fb: f64,
    /// Critical resistance: R1 + R2 + R1*C1/C2.
    /// Recomputed when series R changes (tuning pot).
    pub r_crit: f64,
    /// Pot-to-component mapping for recomputation.
    /// Each entry: (pot_child_index, affects_r_series, affects_r_fb)
    pub pot_map: Vec<(usize, bool, bool)>,
    /// Base R values for series resistors (before pot contribution).
    pub r_series_base: [f64; 2],
    /// Base C values for shunt caps.
    pub c_shunt_base: [f64; 2],
}

impl IirData {
    pub fn new(b_coeffs: Vec<f64>, a_coeffs: Vec<f64>, sample_rate: f64) -> Self {
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
        use core::f64::consts::PI;
        if self.r_series_product <= 0.0 || self.c_shunt_product <= 0.0 || self.r_fb <= 0.0 {
            return;
        }

        let f0 = 1.0 / (2.0 * PI * crate::math::sqrt(self.r_series_product * self.c_shunt_product));
        let q = if self.r_fb > self.r_crit * 1.01 {
            self.r_fb / (self.r_fb - self.r_crit)
        } else {
            100.0
        };
        let gain = self.r_fb / (crate::math::sqrt(self.r_series_product)); // Rf / sqrt(R1*R2)

        let w0 = 2.0 * PI * f0 / self.sample_rate;
        let sin_w0 = crate::math::sin(w0);
        let cos_w0 = crate::math::cos(w0);
        let alpha = sin_w0 / (2.0 * q);

        let a0 = 1.0 + alpha;
        self.b_coeffs[0] = alpha * gain / a0;
        self.b_coeffs[1] = 0.0;
        self.b_coeffs[2] = -alpha * gain / a0;
        self.a_coeffs[1] = -2.0 * cos_w0 / a0;
        self.a_coeffs[2] = (1.0 - alpha) / a0;
    }

    /// DC gain: H(z=1) = sum(b) / sum(a).
    pub fn dc_gain(&self) -> f64 {
        let num: f64 = self.b_coeffs.iter().sum();
        let den: f64 = self.a_coeffs.iter().sum();
        if den.abs() < 1e-30 {
            1.0
        } else {
            num / den
        }
    }

    /// Process one sample through the IIR (Direct Form I).
    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
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
}

/// Standalone IIR biquad stage compiled from a linear rigid MNA.
///
/// Clean, minimal: just coefficients + state + process(). O(1)/sample.
/// No WDF tree, no scattering matrix, no NR solver.
///
/// When the stage contains an op-amp, component-declared `NonIdealFx`
/// are applied as post-processing: GBW rolloff → slew limiting → rail clamp.
/// Pot binding info stored in IirStage for runtime coefficient recomputation.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct IirPotBinding {
    /// Component ID of the pot (matches ControlBinding::component_id).
    pub comp_id: String,
    /// Maximum resistance of the pot.
    pub max_r: f64,
    /// Fixed resistance in series with the pot (e.g., R_min before Drive pot).
    pub fixed_series_r: f64,
    /// Input resistance Ri (for gain = Rf/Ri calculation).
    pub ri: f64,
    /// Current pot position (0.0–1.0).
    pub position: f64,
}

/// Precomputed biquad coefficient lookup table for pot-controlled IIR stages.
///
/// Built at compile time by sweeping pot positions over an N-dimensional grid.
/// At runtime, `set_pot` quantizes the position and bilinearly interpolates
/// the 5 biquad coefficients from the nearest grid neighbors. O(1) lookup,
/// no matrix math, no alloc. Cortex-M7 safe.
///
/// Dimensions correspond to independent control labels (ganged pots = 1 dim).
/// Table size: `steps^n_dims × 5` f64 entries.
#[cfg(feature = "biquad-table")]
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
    pub coeffs: alloc::vec::Vec<f64>,
}

#[cfg(feature = "biquad-table")]
impl BiquadTable {
    /// Number of coefficients per entry (b0, b1, b2, a1, a2).
    const COEFF_COUNT: usize = 5;

    /// Look up and interpolate biquad coefficients for given positions.
    /// `positions` maps dim index → normalized position [0.0, 1.0].
    /// Writes 5 values into `out`: [b0, b1, b2, a1, a2].
    pub fn lookup(&self, positions: &[f64], out: &mut [f64; 5]) {
        let n_dims = self.dim_labels.len();
        if n_dims == 0 || self.steps < 2 {
            return;
        }
        let max_idx = self.steps - 1;

        match n_dims {
            1 => {
                // Linear interpolation
                let p = positions[0].clamp(0.0, 1.0) * max_idx as f64;
                let i0 = (p as usize).min(max_idx - 1);
                let frac = p - i0 as f64;
                let base0 = i0 * Self::COEFF_COUNT;
                let base1 = (i0 + 1) * Self::COEFF_COUNT;
                for c in 0..5 {
                    out[c] = self.coeffs[base0 + c] * (1.0 - frac)
                        + self.coeffs[base1 + c] * frac;
                }
            }
            2 => {
                // Bilinear interpolation
                let p0 = positions[0].clamp(0.0, 1.0) * max_idx as f64;
                let p1 = positions[1].clamp(0.0, 1.0) * max_idx as f64;
                let i0 = (p0 as usize).min(max_idx - 1);
                let i1 = (p1 as usize).min(max_idx - 1);
                let f0 = p0 - i0 as f64;
                let f1 = p1 - i1 as f64;
                let s = self.steps;
                let idx00 = (i0 + i1 * s) * Self::COEFF_COUNT;
                let idx10 = (i0 + 1 + i1 * s) * Self::COEFF_COUNT;
                let idx01 = (i0 + (i1 + 1) * s) * Self::COEFF_COUNT;
                let idx11 = (i0 + 1 + (i1 + 1) * s) * Self::COEFF_COUNT;
                let w00 = (1.0 - f0) * (1.0 - f1);
                let w10 = f0 * (1.0 - f1);
                let w01 = (1.0 - f0) * f1;
                let w11 = f0 * f1;
                for c in 0..5 {
                    out[c] = self.coeffs[idx00 + c] * w00
                        + self.coeffs[idx10 + c] * w10
                        + self.coeffs[idx01 + c] * w01
                        + self.coeffs[idx11 + c] * w11;
                }
            }
            _ => {
                // Fallback: nearest-neighbor for 3+ dims
                let mut flat_idx = 0usize;
                let mut stride = 1usize;
                for d in 0..n_dims {
                    let p = positions[d].clamp(0.0, 1.0) * max_idx as f64;
                    let i = (p as usize).min(max_idx);
                    flat_idx += i * stride;
                    stride *= self.steps;
                }
                let base = flat_idx * Self::COEFF_COUNT;
                for c in 0..5 {
                    out[c] = self.coeffs[base + c];
                }
            }
        }
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct IirStage {
    /// The biquad filter data (coefficients + history).
    pub iir: IirData,
    /// Passive attenuation compensation factor.
    pub compensation: f64,
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
    /// Precomputed biquad lookup table (compile-time sweeps over pot positions).
    /// When present, `set_pot` interpolates from this instead of the DC gain formula.
    #[cfg(feature = "biquad-table")]
    pub biquad_table: Option<BiquadTable>,
    /// Sample rate (needed for GBW recomputation on gain change).
    pub sample_rate: f64,
    // ── NonIdealFx runtime state ──
    /// GBW single-pole IIR state (for OpAmpBandwidth).
    gbw_state: f64,
    /// GBW lowpass coefficient: α = 2π·fc / (2π·fc + fs) where fc = GBW/gain.
    gbw_coeff: f64,
    /// Previous output sample (for slew rate limiting).
    prev_out: f64,
    /// Maximum dV per sample from slew rate (slew_rate / sample_rate).
    max_dv_per_sample: f64,
    /// Rail saturation voltage (from RailSaturation).
    v_max: f64,
    /// Stored GBW from OpAmpBandwidth (for recomputation when gain changes).
    stored_gbw: f64,
}

impl IirStage {
    pub fn new(iir: IirData) -> Self {
        let sample_rate = iir.sample_rate;
        Self {
            iir,
            compensation: 1.0,
            signal_flow_distance: 0,
            #[cfg(debug_assertions)]
            debug_label: String::new(),
            bypass_serial: false,
            nonideal_fx: Vec::new(),
            pot_bindings: Vec::new(),
            #[cfg(feature = "biquad-table")]
            biquad_table: None,
            sample_rate,
            gbw_state: 0.0,
            gbw_coeff: 1.0, // passthrough (no GBW limiting)
            prev_out: 0.0,
            max_dv_per_sample: f64::MAX,
            v_max: f64::MAX,
            stored_gbw: 0.0,
        }
    }

    /// Configure NonIdealFx post-processing from component declarations.
    ///
    /// Pre-computes runtime constants (gbw_coeff, max_dv_per_sample, v_max)
    /// so process() stays O(1) with no branching on enum variants.
    pub fn set_nonideal_fx(&mut self, fx: Vec<crate::nonideal_fx::NonIdealFx>, sample_rate: f64) {
        use crate::nonideal_fx::NonIdealFx;
        for effect in &fx {
            match effect {
                NonIdealFx::OpAmpBandwidth { gbw, slew_rate } => {
                    self.stored_gbw = *gbw;
                    // Estimate closed-loop gain from IIR DC response (b[0]+b[1]+b[2]) / (a[0]+a[1]+a[2])
                    let gain = self.iir.dc_gain().abs().max(1.0);
                    let fc = gbw / gain;
                    let w = 2.0 * core::f64::consts::PI * fc;
                    self.gbw_coeff = w / (w + sample_rate);
                    // slew_rate from SPICE model is in V/µs — convert to V/s
                    self.max_dv_per_sample = slew_rate * 1e6 / sample_rate;
                }
                NonIdealFx::RailSaturation { v_max } => {
                    self.v_max = *v_max;
                }
            }
        }
        self.nonideal_fx = fx;
    }

    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
        let mut out = self.iir.process(input * self.compensation);

        // GBW rolloff: single-pole lowpass
        out = self.gbw_coeff * out + (1.0 - self.gbw_coeff) * self.gbw_state;
        self.gbw_state = out;

        // Slew rate limiting
        let dv = out - self.prev_out;
        if dv > self.max_dv_per_sample {
            out = self.prev_out + self.max_dv_per_sample;
        } else if dv < -self.max_dv_per_sample {
            out = self.prev_out - self.max_dv_per_sample;
        }
        self.prev_out = out;

        // Rail saturation: tanh soft clip
        if self.v_max < f64::MAX {
            out = self.v_max * crate::fast_math::fast_tanh(out / self.v_max);
        }

        flush_denormal(out)
    }

    /// Check if this stage contains a pot with the given component ID.
    pub fn has_pot(&self, comp_id: &str) -> bool {
        self.pot_bindings.iter().any(|b| b.comp_id == comp_id)
    }

    /// Update pot position and recompute IIR coefficients.
    ///
    /// With `biquad-table` feature: interpolates full biquad from precomputed
    /// table. Handles cutoff, resonance, and any other pot — all coefficients
    /// update correctly for frequency-dependent changes.
    ///
    /// Without table: falls back to DC gain recalculation (Rf/Ri).
    /// No heap allocations — all state is pre-allocated.
    pub fn set_pot(&mut self, comp_id: &str, position: f64) {
        let binding = match self.pot_bindings.iter_mut().find(|b| b.comp_id == comp_id) {
            Some(b) => b,
            None => return,
        };
        binding.position = position;

        // ── Table lookup path (full biquad interpolation) ──
        #[cfg(feature = "biquad-table")]
        if let Some(ref table) = self.biquad_table {
            // Build position vector from all pot bindings, matched by comp_id
            let mut positions = alloc::vec![0.0f64; table.dim_labels.len()];
            for binding in &self.pot_bindings {
                for (di, comp_id) in table.dim_labels.iter().enumerate() {
                    if comp_id.as_str() == binding.comp_id.as_str() {
                        positions[di] = binding.position;
                    }
                }
            }
            let mut coeffs = [0.0f64; 5];
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
                let w = 2.0 * core::f64::consts::PI * fc;
                self.gbw_coeff = w / (w + self.sample_rate);
            }
            return;
        }

        // ── Fallback: DC gain recalculation ──
        let rf = binding.fixed_series_r + position * binding.max_r;
        let ri = binding.ri;
        let dc_gain = if ri > 0.0 { -(rf / ri) } else { -1.0 };

        self.iir.b_coeffs[0] = dc_gain;

        if self.stored_gbw > 0.0 {
            let gain_abs = dc_gain.abs().max(1.0);
            let fc = self.stored_gbw / gain_abs;
            let w = 2.0 * core::f64::consts::PI * fc;
            self.gbw_coeff = w / (w + self.sample_rate);
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
    rf: f64,
    /// Input resistance (Ohms). Fixed at compile time.
    ri: f64,
    /// True = inverting (gain = -Rf/Ri), false = non-inverting (1 + Rf/Ri).
    inverting: bool,
    /// NonIdealFx post-processing state (GBW/slew/rails).
    fx_state: NonIdealFxState,
    /// Stored GBW for recomputation when gain changes.
    stored_gbw: f64,
    /// Sample rate for GBW recomputation.
    sample_rate: f64,
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
    /// Maximum pot resistance (Ohms). Position 1.0 = this value.
    pub pot_max_r: f64,
    /// Pot component ID in the ground leg (Ri). When this pot changes,
    /// Ri = ri_fixed_r + pot_position × ri_pot_max_r.
    pub ri_pot_comp_id: Option<String>,
    /// Fixed resistance in the ground leg (sum of non-pot resistors: R5 + R6).
    pub ri_fixed_r: f64,
    /// Max resistance of the Ri pot (e.g. Gain_A max_r = 100k).
    pub ri_pot_max_r: f64,
}

impl BlackFeedbackStage {
    /// Construct from circuit parameters.
    pub fn new(
        rf: f64,
        ri: f64,
        inverting: bool,
        fx: &[crate::nonideal_fx::NonIdealFx],
        sample_rate: f64,
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
            pot_max_r: 0.0,
            ri_pot_comp_id: None,
            ri_fixed_r: 0.0,
            ri_pot_max_r: 0.0,
        }
    }

    /// Test helper: construct with default NonIdealFx.
    pub fn new_test(rf: f64, ri: f64, inverting: bool, sample_rate: f64) -> Self {
        Self::new(rf, ri, inverting, &[], sample_rate)
    }

    /// Current closed-loop gain.
    pub fn gain(&self) -> f64 {
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

    /// Check if this stage owns a pot with the given component ID.
    pub fn has_pot(&self, comp_id: &str) -> bool {
        self.pot_comp_id.as_deref() == Some(comp_id)
    }

    /// Set pot position (0.0–1.0). Converts to Rf = position * max_r.
    pub fn set_pot(&mut self, _comp_id: &str, position: f64) {
        if self.pot_max_r > 0.0 {
            self.set_rf(position * self.pot_max_r);
        }
    }

    /// Update Ri from ground-leg pot position. Called when a pot in the
    /// ground leg (e.g. Gain_A in Goldenrod) changes position.
    /// The position is range-mapped but NOT tapered — apply taper here.
    pub fn update_ri_from_pot(&mut self, comp_id: &str, position: f64) {
        if self.ri_pot_comp_id.as_deref() == Some(comp_id) {
            // Apply taper to get actual resistance fraction.
            // Gain_A is linear (b) taper, so taper(pos) ≈ pos.
            // For audio (a) taper, taper(pos) gives the log curve.
            let tapered = crate::pot_taper::PotTaper::B.apply(position); // TODO: store actual taper
            let pot_r = tapered * self.ri_pot_max_r;
            let new_ri = (self.ri_fixed_r + pot_r).max(1.0);
            self.set_ri(new_ri);
        }
    }

    /// Set input resistance (for pipeline Ri fix).
    pub fn set_ri(&mut self, ri: f64) {
        self.ri = ri;
    }

    /// Set asymmetric rail limits from bias analysis.
    pub fn set_v_rails(&mut self, v_rail_pos: f64, v_rail_neg: f64) {
        self.fx_state.v_rail_pos = v_rail_pos;
        self.fx_state.v_rail_neg = v_rail_neg;
    }

    /// Update Rf (pot sweep). Recomputes gain and GBW coefficient.
    pub fn set_rf(&mut self, rf: f64) {
        self.rf = rf;
        if self.stored_gbw > 0.0 {
            self.fx_state
                .update_gain(self.stored_gbw, self.gain(), self.sample_rate);
        }
    }

    /// Process one sample: gain × input → NonIdealFx → output.
    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
        let gained = input * self.gain();
        flush_denormal(apply_nonideal_fx(gained, &mut self.fx_state))
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
    work: Vec<f64>,
    /// Passive attenuation compensation factor.
    pub compensation: f64,
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
    pub supply_voltage: f64,
}

impl StateSpaceStage {
    pub fn new(ss: StateSpaceData, supply_voltage: f64) -> Self {
        let n = ss.n_states;
        Self {
            ss,
            work: vec![0.0; n],
            compensation: 1.0,
            signal_flow_distance: 0,
            #[cfg(debug_assertions)]
            debug_label: String::new(),
            bypass_serial: false,
            supply_voltage,
        }
    }

    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
        let n = self.ss.n_states;
        let sample = input * self.compensation;

        // x[n] = A · x[n-1] + b · u[n]  (into pre-allocated work buffer)
        for i in 0..n {
            let mut v = self.ss.b_vector[i] * sample;
            let row_start = i * n;
            for j in 0..n {
                v += self.ss.a_matrix[row_start + j] * self.ss.x[j];
            }
            self.work[i] = flush_denormal(v);
        }

        // Op-amp rail saturation: tanh soft-clip state variables.
        let v_rail = (self.supply_voltage * 0.5 - 1.5).max(0.5);
        for x in &mut self.work[..n] {
            *x = v_rail * crate::fast_math::fast_tanh(*x / v_rail);
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
    pub x: Vec<f64>,
    /// State transition matrix A_d = M⁻¹·N (n_states × n_states, row-major).
    pub a_matrix: Vec<f64>,
    /// Input vector b_d = M⁻¹·F (n_states × 1).
    pub b_vector: Vec<f64>,
    /// Output extraction vector c_out (1 × n_states).
    pub c_vector: Vec<f64>,
    /// Number of state variables (num_nodes + num_vsources).
    pub n_states: usize,
    /// Capacitance stamps for rebuilding state-space when G changes.
    /// Each entry: (node_pos, node_neg, capacitance_farads).
    pub cap_stamps: Vec<(Option<usize>, Option<usize>, f64)>,
    /// VS branch index for input.
    pub vs_idx: usize,
    /// Output extraction nodes.
    pub output_pos: Option<usize>,
    pub output_neg: Option<usize>,
    /// Sample rate for bilinear transform (2·f_s·C scaling).
    pub sample_rate: f64,
    /// Direct feedthrough: y = c·x + d·u. From Schur complement elimination.
    pub d_feedthrough: f64,
    /// Previous output for 2-sample Nyquist filter.
    /// Removes parasitic -1 eigenvalue oscillation from unreduced systems.
    pub prev_output: f64,
    /// Pot stamps for delta-updating G when pots change.
    /// Each entry: (passive_child_index, node_pos, node_neg, last_conductance).
    pub pot_stamps: Vec<(usize, Option<usize>, Option<usize>, f64)>,
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
    pub gain: f64,
    /// MNA cell indices for the VCCS stamp: (row, col, sign).
    /// These are the cells in g_matrix that encode the transconductance.
    pub stamp_cells: Vec<(usize, usize, f64)>,
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
    pub fn process(&mut self, input: f64) -> f64 {
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
                for i in 0..n {
                    let mut v = ss.b_vector[i] * sample;
                    let row_start = i * n;
                    for j in 0..n {
                        v += ss.a_matrix[row_start + j] * ss.x[j];
                    }
                    work[i] = flush_denormal(v);
                }

                // Op-amp rail saturation: apply tanh soft-clip to every state.
                // In oscillator circuits (bridged-T), the op-amp output exceeds
                // supply rails → tanh limits the amplitude → creates a stable
                // limit cycle whose frequency is set by the RC network.
                // Cap voltages are also clipped since they can't exceed Vcc.
                if v_rail > 0.0 {
                    for i in 0..n {
                        work[i] = v_rail * crate::fast_math::fast_tanh(work[i] / v_rail);
                    }
                }

                ss.x[..n].copy_from_slice(&work[..n]);

                // Output extraction: y[n] = c · x[n] + d · u[n]
                let mut y_raw = ss.d_feedthrough * sample;
                for i in 0..n {
                    y_raw += ss.c_vector[i] * work[i];
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
        let n_passive = self.passive_children.len();
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
            self.dc_ramp as f64 / DC_RAMP_SAMPLES as f64
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
            for (k, child) in self.passive_children.iter_mut().enumerate() {
                b_passive[k] = child.reflected();
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
            for i in 0..n_nl {
                let mut a_i = self.scattering.s_nl_adapted[i] * b_adapted;
                for k in 0..n_passive {
                    a_i += self.scattering.s_nl_passive[i * n_passive + k] * b_passive[k];
                }
                a_i += self.dc_bias[i] * dc_scale;
                known_a[i] = a_i;
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
            for (k, child) in self.passive_children.iter_mut().enumerate() {
                child.set_incident(a_all[n_nl + k]);
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

            // 6b. Apply op-amp post-FX (supply rail clamp + slew rate).
            // The VCVS stamp in MNA already captured Aol and Ro; rails and
            // slew are the non-LTI behaviours that must live outside the
            // scattering matrix.
            let raw_out = if let Some(ref mut post_fx) = self.opamp_post_fx {
                post_fx.process(raw_out)
            } else {
                raw_out
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
    pub fn apply_oversampling_rate(&mut self, base_rate: f64) {
        // DynNode trees are already at the correct rate (built with effective_rate).
        // But feedback_opamp GBW/slew filters need the oversampled rate.
        if let Some(ref mut opamp) = self.feedback_opamp {
            let effective_rate = base_rate * self.oversampler.ratio() as f64;
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
    pub fn update_supply_voltage(&mut self, new_voltage: f64) {
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
        for child in &mut self.passive_children {
            child.reset();
        }
        // Restore physics-based initial guesses instead of zeroing.
        // Zeroing v_prev causes the NR solver to start far from the operating
        // point, leading to divergence and 100% CPU in real-time contexts.
        self.v_prev.copy_from_slice(&self.initial_v_prev);
        // Reset 2-sample history for extrapolation warm-start.
        // Seed from initial_v_prev so extrapolation starts near the operating point.
        self.v_prev_2.copy_from_slice(&self.initial_v_prev);
        self.dc_ramp = 0;
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
    }

    /// Debug dump: print multi-NL stage structure.
    pub fn debug_dump(&self) -> String {
        let n_passive = self.passive_children.len();
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
        for (k, child) in self.passive_children.iter().enumerate() {
            s.push_str(&format!(
                "  Passive[{}]: Rp={:.1}Ω, nodes={}\n",
                k,
                child.port_resistance(),
                child.node_count()
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
        let n_passive = self.passive_children.len();
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
    pub fn set_pot(&mut self, target_id: &str, value: f64) -> bool {
        let mut found = false;

        // Try base name (2-terminal pot)
        for child in &mut self.pot_children {
            if child.set_pot(target_id, value) {
                found = true;
                break;
            }
        }

        // Try Baxandall-decomposed halves (3-terminal pot)
        let aw = format!("{target_id}__aw");
        for child in &mut self.pot_children {
            if child.set_pot(&aw, value) {
                found = true;
                break;
            }
        }
        let wb = format!("{target_id}__wb");
        for child in &mut self.pot_children {
            if child.set_pot(&wb, 1.0 - value) {
                found = true;
                break;
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
    pub fn set_ota_gain_linear(&mut self, gain: f64) {
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
    pub fn set_feedback_from_pot(&mut self, position: f64, max_pot_r: f64) {
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
    fn get_pot_child_resistance(&self, pot_id: &str) -> Option<f64> {
        for child in &self.pot_children {
            if let Some(r) = child.get_pot_resistance(pot_id) {
                return Some(r);
            }
        }
        None
    }

    /// Recompute the scattering matrix from stored MNA data after a pot change.
    ///
    /// Rebuilds WdfPort vec with current port resistances from nl_port_resistances
    /// and passive_children, re-derives the scattering matrix, and updates all
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
                for ps in &mut ss.pot_stamps {
                    let child_idx = ps.0;
                    let pos = ps.1;
                    let neg = ps.2;
                    let new_r = self.pot_children[child_idx].port_resistance();
                    let new_g = 1.0 / new_r;
                    let delta = new_g - ps.3;
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
                        ps.3 = new_g;
                    }
                }
                let (a_d, b_d, c_out, _n, d_ft) = recompute.mna.build_state_space_matrices(
                    &ss.cap_stamps,
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
            if self.pot_mna_stamps.len() == 1 {
                let pot_r = self.pot_children[self.pot_mna_stamps[0].0].port_resistance();
                let (new_scat, new_vs) = table.lookup(pot_r);

                if new_scat.iter().all(|&s| s.is_finite()) {
                    let n_nl = self.n_nl;
                    let n_passive = self.passive_children.len();
                    self.scattering =
                        MultiNlScattering::from_full_matrix(&new_scat, n_nl, n_passive);

                    // Re-extract dc_bias and vcc_bias_all from VCC injection vector
                    if self.vcc_vs_index.is_some() {
                        // The interp table's vs_injection is the VCC injection vector
                        for i in 0..n_nl.min(new_vs.len()) {
                            self.dc_bias[i] = new_vs[i] * self.supply_voltage;
                        }
                        self.vcc_bias_all =
                            new_vs.iter().map(|&k| k * self.supply_voltage).collect();
                    }

                    // Rebuild port_resistances for RTypeAdaptor
                    let mut port_resistances = self.nl_port_resistances.clone();
                    for child in &self.passive_children {
                        port_resistances.push(child.port_resistance());
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
                        if let Some((out_pos, out_neg)) = recompute.extract_output_nodes {
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
        for ps in &mut self.pot_mna_stamps {
            let child_idx = ps.0;
            let pos = ps.1;
            let neg = ps.2;
            let new_r = self.pot_children[child_idx].port_resistance();
            let new_g = 1.0 / new_r;
            let delta = new_g - ps.3;
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
                ps.3 = new_g;
            }
        }

        let n_nl = self.n_nl;
        let n_passive = self.passive_children.len();
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
            let (pos, neg) = recompute.port_node_pairs[i];
            ports.push(WdfPort {
                node_pos: pos,
                node_neg: neg,
                resistance: self.nl_port_resistances[i],
            });
        }

        // Passive ports (caps/inductors — resistances are fixed for a given sample rate)
        for k in 0..n_passive {
            let (pos, neg) = recompute.port_node_pairs[n_nl + k];
            let rp = self.passive_children[k].port_resistance();
            ports.push(WdfPort {
                node_pos: pos,
                node_neg: neg,
                resistance: rp,
            });
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
            let port_resistances: Vec<f64> = ports.iter().map(|p| p.resistance).collect();
            self.adaptor = RTypeAdaptor::new(scattering, &port_resistances);
            self.vs_injection = Some(vs_inj);

            // Recompute extraction coefficients if used.
            if let Some((out_pos, out_neg)) = recompute.extract_output_nodes {
                let (coeffs, vs_coeff) = recompute
                    .mna
                    .derive_extraction_coeffs(&ports, vs_idx, out_pos, out_neg);
                self.extract_coeffs = Some(coeffs);
                self.extract_vs = vs_coeff;
            }
        } else {
            // Standard mode: adapted port included (always last).
            let adapted_pair_idx = n_nl + n_passive;
            let (pos, neg) = recompute.port_node_pairs[adapted_pair_idx];
            ports.push(WdfPort {
                node_pos: pos,
                node_neg: neg,
                resistance: recompute.adapted_resistance,
            });

            if let Some(vcc_idx) = recompute.vcc_vs_index {
                // VCC as ideal VS: derive scattering + VCC injection vector
                let (scattering, vcc_inj) = recompute
                    .mna
                    .derive_scattering_and_vs_injection(&ports, vcc_idx);

                if scattering.iter().any(|&s| !s.is_finite()) {
                    return;
                }

                // Re-extract dc_bias and vcc_bias_all from VCC injection vector
                for i in 0..n_nl.min(vcc_inj.len()) {
                    self.dc_bias[i] = vcc_inj[i] * self.supply_voltage;
                }
                self.vcc_bias_all = vcc_inj.iter().map(|&k| k * self.supply_voltage).collect();

                self.scattering = MultiNlScattering::from_full_matrix(&scattering, n_nl, n_passive);
                let port_resistances: Vec<f64> = ports.iter().map(|p| p.resistance).collect();
                self.adaptor = RTypeAdaptor::new(scattering, &port_resistances);

                if let Some((out_pos, out_neg)) = recompute.extract_output_nodes {
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
                let port_resistances: Vec<f64> = ports.iter().map(|p| p.resistance).collect();
                self.adaptor = RTypeAdaptor::new(scattering, &port_resistances);

                if let Some((out_pos, out_neg)) = recompute.extract_output_nodes {
                    self.extract_coeffs = Some(
                        recompute
                            .mna
                            .derive_node_extraction_coeffs(&ports, out_pos, out_neg),
                    );
                    self.extract_vs = 0.0;
                }
            }
        }
    }

    // ── Precompute accessors ─────────────────────────────────────────────
    // Minimal read-only accessors for `precompute::extract_precomputed`.

    /// The R-type adaptor containing the scattering matrix and port data.
    pub fn adaptor(&self) -> &RTypeAdaptor {
        &self.adaptor
    }

    /// VS injection vector (present when driven by an ideal voltage source).
    pub fn vs_injection(&self) -> Option<&Vec<f64>> {
        self.vs_injection.as_ref()
    }

    /// Node-voltage extraction coefficients (present when output is read from
    /// MNA node voltages directly rather than WDF port waves).
    pub fn extract_coeffs(&self) -> Option<&Vec<f64>> {
        self.extract_coeffs.as_ref()
    }

    /// VS component of the extraction formula.
    pub fn extract_vs(&self) -> f64 {
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
    pub cv_delayed: f64,
}

impl SidechainProcessor {
    /// Process one sample through the sidechain sub-circuit.
    #[inline]
    pub fn process(&mut self, tapped_signal: f64) -> f64 {
        self.circuit.process(tapped_signal)
    }

    /// Forward a control change to the sidechain sub-circuit.
    pub fn set_control(&mut self, label: &str, value: f64) {
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
    pub held_output: f64,
    /// Output from one `rate_divisor` period ago (for linear interpolation).
    pub prev_output: f64,
}

impl SubcircuitProcessor {
    /// Process one input sample.
    ///
    /// For full-rate subcircuits (`rate_divisor == 1`) this is a direct call to
    /// the inner [`CompiledPedal::process`].  For rate-reduced subcircuits the
    /// inner circuit is only called every `rate_divisor` samples; between calls
    /// the output is linearly interpolated from the previous to the current value.
    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
        if self.rate_divisor <= 1 {
            self.held_output = self.circuit.process(input);
            return self.held_output;
        }

        self.rate_counter -= 1;
        if self.rate_counter == 0 {
            self.rate_counter = self.rate_divisor;
            self.prev_output = self.held_output;
            self.held_output = self.circuit.process(input);
        }

        // Linear interpolation between prev_output and held_output.
        // t = 1.0 when we just computed a new value, 0.0 when counter is back to rate_divisor.
        let t = 1.0 - (self.rate_counter as f64 / self.rate_divisor as f64);
        self.prev_output + t * (self.held_output - self.prev_output)
    }

    /// Forward a control change to the subcircuit's inner processor.
    pub fn set_control(&mut self, label: &str, value: f64) {
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
