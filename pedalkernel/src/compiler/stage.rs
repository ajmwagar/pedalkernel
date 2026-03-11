//! WDF clipping/processing stage combining a tree with a nonlinear root.

use crate::elements::*;
use crate::oversampling::Oversampler;
use crate::tree::{MnaSystem, RTypeAdaptor, ScatteringInterpolationTable, WdfPort};
use crate::PedalProcessor;

use super::dyn_node::DynNode;
use super::helpers::balance_parallel_vs;

/// Flush denormals to zero. Subnormal floats are 100x slower to process
/// on x86 and serve no useful purpose in audio signals.
#[inline(always)]
fn flush_denormal(x: f64) -> f64 {
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
use std::sync::atomic::{AtomicU64, Ordering};

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
// WDF clipping stage
// ═══════════════════════════════════════════════════════════════════════════

#[allow(dead_code)]
pub(super) enum RootKind {
    DiodePair(DiodePairRoot),
    SingleDiode(DiodeRoot),
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
    Ota(OtaRoot),
    /// Op-amp gain stage (TL072, LM308, JRC4558, etc.).
    /// Topology (inverting vs non-inverting) is stored in the root itself.
    /// - Inverting: Vout = -(Rf/Ri) * Vin
    /// - Non-inverting: Vout = (1 + Rf/Ri) * Vin
    OpAmp(OpAmpRoot),
    /// NPN BJT transistor (2N3904, BC109, 2N5089, etc.).
    /// Modeled with Ebers-Moll equations.
    BjtNpn(BjtNpnRoot),
    /// PNP BJT transistor (2N3906, AC128, NKT275, etc.).
    /// Modeled with Ebers-Moll equations.
    BjtPnp(BjtPnpRoot),
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
const BJT_VBE_BIAS: f64 = 0.6;
const BJT_VBE_SCALE: f64 = 0.15;
const PNP_VEB_BIAS: f64 = 0.15;
const PNP_VEB_SCALE: f64 = 0.3;
const TRIODE_GRID_BIAS: f64 = -2.0;
const PENTODE_GRID_BIAS: f64 = -8.0;

impl RootKind {
    /// Returns `true` for roots that clip the signal (diodes, zeners).
    pub(super) fn is_clipping_stage(&self) -> bool {
        matches!(
            self,
            RootKind::DiodePair(_) | RootKind::SingleDiode(_) | RootKind::Zener(_)
        )
    }

    /// Set the control voltage on the nonlinear root from the input signal.
    ///
    /// Maps the audio input to the device's control terminal:
    /// - BJTs: Vbe/Veb from input with bias offset
    /// - Triodes/VariMu: Vgk from input
    /// - Pentodes: Vg1k from input
    /// - Diodes/JFETs/other: no control voltage
    pub(super) fn set_control_voltage(&mut self, input: f64, compensation: f64, bias_offset: f64) {
        match self {
            RootKind::BjtNpn(bjt) => {
                bjt.set_vbe(BJT_VBE_BIAS + bias_offset + input * compensation * BJT_VBE_SCALE);
            }
            RootKind::BjtPnp(bjt) => {
                bjt.set_veb(PNP_VEB_BIAS + bias_offset + input * compensation * PNP_VEB_SCALE);
            }
            RootKind::Triode(t) => {
                t.set_vgk(TRIODE_GRID_BIAS + input * compensation);
            }
            RootKind::VariMu(t) => {
                t.set_vgk(TRIODE_GRID_BIAS + input * compensation);
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
pub(super) struct AllpassFeedback {
    /// Input resistance (R_ap) in the signal path before the JFET.
    pub(super) r_ap: f64,
    /// IIR numerator coefficient: Rf / (1 + K) where K = 2·fs·Rf·Cf.
    pub(super) b0: f64,
    /// IIR denominator coefficient: (K - 1) / (K + 1).
    pub(super) a1: f64,
    /// Previous input current sample for IIR.
    pub(super) x_prev: f64,
    /// Previous output voltage for IIR.
    pub(super) y_prev: f64,
}

/// Models a frequency-dependent opamp feedback tone control where a resistive
/// DC path (R_fb) is in parallel with a cap-series AC path (C + R_pot + R_shelf).
///
/// Transfer function: H(s) = -(R_fb/R_in) * (1 + s*C*R_ac) / (1 + s*C*(R_fb + R_ac))
/// where R_ac = R_pot_current + R_shelf.
///
/// This is a first-order shelving filter whose transition frequency and HF gain
/// depend on the pot position. DC gain = -R_fb/R_in (constant).
/// HF gain = -(R_fb/R_in) * R_ac/(R_fb + R_ac) (varies with pot).
///
/// Bilinear-transformed IIR coefficients are recomputed when the pot changes.
pub(super) struct ToneFeedback {
    /// DC feedback resistance (R_fb), constant.
    pub(super) r_fb: f64,
    /// Input resistance (R_in), constant.
    pub(super) r_in: f64,
    /// Tone capacitance (C_tone), constant.
    pub(super) c_tone: f64,
    /// Fixed shelf resistance after the pot (R_shelf), constant.
    pub(super) r_shelf: f64,
    /// Maximum pot resistance (for computing current R_pot from position).
    pub(super) max_pot_r: f64,
    /// Pot component ID for tracking changes.
    pub(super) pot_id: String,
    /// Sample rate for bilinear transform.
    pub(super) sample_rate: f64,
    /// IIR numerator coefficient b0 (normalized).
    pub(super) b0: f64,
    /// IIR numerator coefficient b1 (normalized).
    pub(super) b1: f64,
    /// IIR denominator coefficient a1 (normalized).
    pub(super) a1: f64,
    /// DC gain = R_fb / R_in.
    pub(super) dc_gain: f64,
    /// Previous input sample.
    pub(super) x_prev: f64,
    /// Previous output sample.
    pub(super) y_prev: f64,
}

impl ToneFeedback {
    /// Create a new ToneFeedback with the given circuit parameters.
    pub(super) fn new(
        r_fb: f64,
        r_in: f64,
        c_tone: f64,
        r_shelf: f64,
        max_pot_r: f64,
        pot_id: String,
        sample_rate: f64,
        initial_pot_position: f64,
    ) -> Self {
        let mut fb = ToneFeedback {
            r_fb,
            r_in,
            c_tone,
            r_shelf,
            max_pot_r,
            pot_id,
            sample_rate,
            b0: 0.0,
            b1: 0.0,
            a1: 0.0,
            dc_gain: r_fb / r_in,
            x_prev: 0.0,
            y_prev: 0.0,
        };
        fb.update_coefficients(initial_pot_position);
        fb
    }

    /// Recompute IIR coefficients for a new pot position (0.0..1.0).
    pub(super) fn update_coefficients(&mut self, pot_position: f64) {
        let r_pot = (pot_position * self.max_pot_r).max(1.0);
        let r_ac = r_pot + self.r_shelf;
        let k = 2.0 * self.sample_rate * self.c_tone;

        // Numerator: (1 + k*R_ac) + (k*R_ac - 1)*z^{-1}
        let a_num = k * r_ac;
        let num0 = 1.0 + a_num;
        let num1 = a_num - 1.0;

        // Denominator: (1 + k*(R_fb + R_ac)) + (k*(R_fb + R_ac) - 1)*z^{-1}
        let b_den = k * (self.r_fb + r_ac);
        let den0 = 1.0 + b_den;
        let den1 = b_den - 1.0;

        // Normalize by den0
        self.b0 = num0 / den0;
        self.b1 = num1 / den0;
        self.a1 = den1 / den0;
    }

    /// Process one sample through the shelving IIR.
    /// Returns the output voltage (already negated for inverting topology).
    #[inline]
    pub(super) fn process(&mut self, input: f64) -> f64 {
        // y[n] = -dc_gain * (b0*x[n] + b1*x[n-1]) - a1*y[n-1]
        let y = -self.dc_gain * (self.b0 * input + self.b1 * self.x_prev) - self.a1 * self.y_prev;
        self.x_prev = input;
        self.y_prev = flush_denormal(y);
        self.y_prev
    }
}

pub(super) struct WdfStage {
    pub(super) tree: DynNode,
    pub(super) root: RootKind,
    /// Compensates for passive attenuation in the tree topology.
    /// Computed automatically from the tree's impedance structure.
    pub(super) compensation: f64,
    /// Oversampler for antialiasing at nonlinear stages.
    pub(super) oversampler: Oversampler,
    /// Base diode model (before thermal modulation). Stored so thermal
    /// drift can be applied as a multiplier without accumulation.
    pub(super) base_diode_model: Option<DiodeModel>,
    /// Op-amp buffer paired with this WDF stage (for all-pass circuits).
    ///
    /// When a unity-gain op-amp feedback loop (neg=out) is detected at this
    /// stage's output, the op-amp is processed inline after the WDF cycle.
    /// The op-amp receives the stage INPUT as its Vp (non-inverting input),
    /// modeling the all-pass behavior where the op-amp buffers the signal
    /// at unity gain while the R/C/JFET network shifts phase.
    pub(super) paired_opamp: Option<OpAmpRoot>,
    /// Inverting all-pass feedback (Phase 90 topology).
    /// When present, bypasses the WDF output and computes V_out = -(Z_fb/Z_in) * V_in
    /// where Z_in includes the JFET variable resistance.
    pub(super) allpass_feedback: Option<AllpassFeedback>,
    /// DC-blocking highpass filter for triode stages.
    /// Models the output coupling capacitor's DC blocking behavior.
    /// Format: (a1, b0, y_prev, x_prev) for IIR highpass.
    pub(super) dc_block: Option<(f64, f64, f64, f64)>,
    /// Source follower mode for JFETs.
    /// When true, Vgs is computed as Vgate (input) - Vsource (output).
    /// This enables proper source follower behavior where the source follows the gate.
    pub(super) is_source_follower: bool,
    /// Previous source voltage for source follower Vgs calculation.
    /// Vgs[n] = input[n] - Vsource[n-1]
    pub(super) prev_source_voltage: f64,
    /// BFS distance from input of the injection node (for topological ordering).
    pub(super) signal_flow_distance: usize,
    /// Inter-stage voltage gain from a transformer boundary.
    /// When the stage's injection node is on a transformer secondary,
    /// this is 1/turns_ratio (e.g., 17.0 for a 1:17 step-up).
    pub(super) transformer_gain: f64,
    /// Circuit graph node ID where this stage's voltage source injects signal.
    /// Used for node-based routing in parallel-path topologies.
    pub(super) injection_node_id: usize,
    /// Circuit graph node ID where this stage's output is extracted.
    /// Typically the plate (triode), collector (BJT), or drain (JFET) node.
    /// Used for node-based routing in parallel-path topologies.
    pub(super) output_node_id: usize,
    /// When true, this stage is a per-trigger voice stage that reads
    /// exclusively from `node_signals` (trigger impulses) rather than
    /// the serial chain signal. Unfired voices receive 0.0 input.
    pub(super) is_trigger_voice: bool,
    /// When true, this stage is a feedforward (parallel) path that reads
    /// from `node_signals` at `injection_node_id` and additively blends
    /// its output into the serial chain signal.
    pub(super) is_feedforward: bool,
    /// Sample counter for runtime warnings rate limiting.
    /// Only meaningful when `runtime-warnings` feature is enabled.
    #[allow(dead_code)]
    pub(super) sample_counter: u64,
    /// Component ID for the root device (for runtime warning attribution).
    /// Only meaningful when `runtime-warnings` feature is enabled.
    #[allow(dead_code)]
    pub(super) root_comp_id: String,
    /// When set, identifies a pot in the WDF tree whose resistance drives
    /// OpAmpRoot gain recalculation. After pot update + recompute, the stage
    /// reads the pot's resistance and calls `OpAmpRoot::set_feedback_pot_r()`.
    pub(super) feedback_pot_id: Option<String>,
    /// When set, extract output voltage at this component (pot leaf) in the
    /// WDF tree instead of at the root junction. After the down-sweep,
    /// V_out = a_leaf / 2 (for a resistive leaf where b=0).
    /// This models voltage extraction at the circuit's output node when
    /// a pot sits between the NL junction and the output.
    pub(super) output_probe: Option<String>,
    /// Op-amp gain stage paired with a DiodePair/SingleDiode root.
    ///
    /// When an inverting op-amp has diodes in its feedback path (e.g., Tube Screamer,
    /// Klon Centaur), the op-amp gain + GBW + slew limiting are applied to the VS
    /// voltage before the diode NR solver clips. This replaces the standalone OpAmp
    /// stage + soft-clip tanh approximation with proper gain-into-diode-clipping.
    ///
    /// NOT the same as `paired_opamp` (which is for Bridged-T all-pass circuits).
    pub(super) feedback_opamp: Option<OpAmpRoot>,
    /// IIR-based tone feedback for inverting opamps with a cap+pot in the
    /// feedback path (e.g., Klon Centaur Treble).  When present, the stage
    /// output is computed from this IIR instead of the WDF tree, giving the
    /// correct frequency-dependent shelving behaviour.
    pub(super) tone_feedback: Option<ToneFeedback>,
    /// Auxiliary load tree for BJT collector-emitter impedance.
    ///
    /// When present, this tree is built between (collector, emitter) and
    /// contains the collector load + emitter bypass + virtual Rce.  The NL
    /// solver uses `load_tree.port_resistance()` as rp instead of the signal
    /// tree's rp, giving the correct load-line slope.  The signal tree
    /// (base network) provides the incident wave b as usual.
    ///
    /// This is purely an impedance calculator — it doesn't participate in
    /// wave scattering.  Pot changes update its resistance via `set_pot()`
    /// and `recompute()`.
    pub(super) load_tree: Option<DynNode>,
}

impl WdfStage {
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

        // Borrow fields individually to satisfy the borrow checker
        let tree = &mut self.tree;
        let root = &mut self.root;
        let compensation = self.compensation;
        let output_probe = &self.output_probe;
        let feedback_opamp = &mut self.feedback_opamp;
        let load_tree = &mut self.load_tree;

        // Set control voltage for active devices (triodes, BJTs, pentodes).
        // Maps the input signal to the device's control terminal with appropriate
        // bias point and scaling. No-op for diodes/JFETs/other passive roots.
        root.set_control_voltage(input, compensation, 0.0);

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
            // - Triode: VS = B+ supply (plate bias)
            // - Source follower: VS = 0 (input goes to gate, not VS)
            // - Feedback opamp + diode: opamp gain drives diode clipping
            // - Other: VS = input * compensation
            let vs_voltage = if let RootKind::Triode(t) = root {
                t.v_max()
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
                RootKind::DiodePair(_) | RootKind::SingleDiode(_) | RootKind::Zener(_) => {
                    -vs_voltage
                }
                _ => vs_voltage,
            };
            tree.set_voltage(vs_voltage);
            let b1 = tree.reflected();

            // ── Load tree Thevenin voltage + rp for BJT stages ───────
            // For BJT stages, the load tree provides both the Thevenin
            // voltage (b_tree, which encodes the VCC supply through the
            // collector load resistor) and the correct port_resistance for
            // the NL solver's load line.
            //
            // Using b1 (signal tree) instead of lt.reflected() would give
            // zero Thevenin voltage (VCC collapsed to GND) and zero gain.
            //
            // For non-BJT stages, b1 carries the incident wave as usual and
            // only rp is overridden.
            let (b_tree, rp) = if let Some(ref mut lt) = load_tree {
                match root {
                    RootKind::BjtNpn(_) | RootKind::BjtPnp(_) => {
                        // BJT two-domain: load tree provides collector Thevenin voltage + impedance
                        (lt.reflected(), lt.port_resistance())
                    }
                    _ => (b1, lt.port_resistance()),
                }
            } else {
                (b1, tree.port_resistance())
            };

            let a_root = match root {
                RootKind::DiodePair(dp) => dp.process(b_tree, rp),
                RootKind::SingleDiode(d) => d.process(b_tree, rp),
                RootKind::Zener(z) => z.process(b_tree, rp),
                RootKind::Jfet(j) => {
                    if is_sf {
                        j.process_source_follower(b_tree, rp, sample * compensation)
                    } else {
                        j.process(b_tree, rp)
                    }
                }
                RootKind::JfetVr(j) => {
                    j.process_root(b_tree, rp)
                }
                RootKind::Triode(t) => t.process(b_tree, rp),
                RootKind::VariMu(t) => t.process(b_tree, rp),
                RootKind::Pentode(p) => p.process(b_tree, rp),
                RootKind::Mosfet(m) => m.process(b_tree, rp),
                RootKind::Ota(o) => o.process(b_tree, rp),
                RootKind::OpAmp(op) => {
                    if op.is_non_inverting() {
                        op.set_vp(sample * compensation);
                    }
                    op.process(b_tree, rp)
                }
                RootKind::BjtNpn(bjt) => bjt.process(b_tree, rp),
                RootKind::BjtPnp(bjt) => bjt.process(b_tree, rp),
                // Passthrough: open-circuit termination (b = a)
                // The tree processes normally but the root just reflects.
                // For passive filters with voltage source, the output voltage
                // should be extracted at the load resistor, not the root port.
                RootKind::Passthrough => {
                    // Standard open-circuit behavior for state updates
                    tree.set_incident(b_tree);
                    // Check output_probe BEFORE resistive_termination fallback
                    if let Some(ref probe_id) = output_probe {
                        if let Some(v) = tree.leaf_voltage(probe_id) {
                            return v;
                        }
                    }
                    // For passive filters with embedded voltage source, the output
                    // voltage at the load is half the root wave (resistive extraction)
                    if tree.resistive_termination_voltage(b_tree).is_some() {
                        // V_out = b_tree / 2 (but keep open-circuit for state updates)
                        return b_tree / 2.0;
                    }
                    // Fallback: standard open-circuit extraction
                    b_tree
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
                    let b_children: Vec<f64> =
                        children.iter_mut().map(|c| c.reflected()).collect();
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
                    for (child, &a_i) in children.iter_mut().zip(a_children.iter())
                    {
                        child.set_incident(a_i);
                    }
                    // 4. Output voltage at probe port
                    let a_out = a_children[*output_port];
                    let b_out = b_children[*output_port];
                    return (a_out + b_out) / 2.0;
                }
            };
            // ── Down-sweep: propagate reflected wave back through signal tree
            tree.set_incident(a_root);
            // Also down-sweep the load tree so its capacitor states (e.g.,
            // emitter bypass capacitor) advance correctly each sample.
            if let Some(ref mut lt) = load_tree {
                lt.set_incident(a_root);
            }
            // If an output probe is set, extract voltage at that leaf
            // after the down-sweep instead of the root junction.
            if let Some(ref probe_id) = output_probe {
                if let Some(v) = tree.leaf_voltage(probe_id) {
                    #[cfg(test)]
                    {
                        static PROBE_DBG: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
                        let n = PROBE_DBG.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                        if n < 20 || (n % 1000 == 0 && n < 5000) {
                            eprintln!("  [probe] n={n} b_tree={b_tree:.6} a_root={a_root:.6} junction={:.6} probe({probe_id})={v:.6}",
                                (a_root + b_tree) / 2.0);
                        }
                    }
                    return v;
                }
            }
            (a_root + b_tree) / 2.0
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

        // Tone feedback IIR (Klon-style cap+pot in inverting opamp feedback).
        // Bypasses WDF output with a first-order shelving filter whose
        // coefficients track the pot position.
        if let Some(ref mut tf) = self.tone_feedback {
            return flush_denormal(tf.process(input * self.compensation));
        }

        // Bridged-T all-pass with unity-gain op-amp buffer:
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
            _ => {}
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
    pub(super) fn set_passive_rtype_pot(&mut self, comp_id: &str, value: f64) -> bool {
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
    pub(super) fn flush_passive_rtype_recompute(&mut self) {
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
    pub(super) fn has_interp_table(&self) -> bool {
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
    pub(super) fn apply_thermal(&mut self, state: &crate::thermal::ThermalState) {
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
                _ => {}
            }
        }
    }

    /// Balance the voltage source impedance to match the network.
    ///
    /// When the Vs branch is inside a Parallel adaptor whose sibling has much
    /// higher impedance, the signal is heavily attenuated.  This adjusts the
    /// Vs port resistance so the branches are balanced (gamma ≈ 0.5).
    pub(super) fn balance_vs_impedance(&mut self) {
        balance_parallel_vs(&mut self.tree);
        self.tree.recompute();
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
    ///
    /// If `tone_feedback` is set (cap+pot in feedback path), updates the IIR
    /// coefficients from the pot's current position.
    pub(super) fn notify_pot_changed(&mut self) {
        if let Some(ref pot_id) = self.feedback_pot_id {
            if let Some(pot_r) = self.tree.get_pot_resistance(pot_id) {
                if let RootKind::OpAmp(ref mut oa) = self.root {
                    oa.set_feedback_pot_r(pot_r);
                }
                if let Some(ref mut oa) = self.feedback_opamp {
                    oa.set_feedback_pot_r(pot_r);
                }
            }
        }
        // Update IIR coefficients for tone feedback (cap+pot in feedback path).
        if let Some(ref mut tf) = self.tone_feedback {
            if let Some(pot_pos) = self.tree.get_pot_position(&tf.pot_id) {
                tf.update_coefficients(pot_pos);
            }
        }
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
            RootKind::Zener(_) => "Zener",
            RootKind::Jfet(_) => "Jfet",
            RootKind::JfetVr(_) => "JfetVr",
            RootKind::Triode(_) => "Triode",
            RootKind::VariMu(_) => "VariMu",
            RootKind::Pentode(_) => "Pentode",
            RootKind::Mosfet(_) => "Mosfet",
            RootKind::Ota(_) => "Ota",
            RootKind::OpAmp(_) => "OpAmp",
            RootKind::BjtNpn(_) => "BjtNpn",
            RootKind::BjtPnp(_) => "BjtPnp",
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
pub(super) enum TubeRoot {
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
pub(super) struct PushPullStage {
    /// WDF tree for push half (plate load + cathode passives).
    pub(super) push_tree: DynNode,
    /// WDF tree for pull half (plate load + cathode passives).
    pub(super) pull_tree: DynNode,
    /// Push tube model (Koren triode or Raffensperger variable-mu).
    pub(super) push_root: TubeRoot,
    /// Pull tube model (Koren triode or Raffensperger variable-mu).
    pub(super) pull_root: TubeRoot,
    /// Oversampler for push half.
    pub(super) push_oversampler: Oversampler,
    /// Oversampler for pull half.
    pub(super) pull_oversampler: Oversampler,
    /// Compensation factor (mu/ref_mu).
    pub(super) compensation: f64,
    /// Output transformer turns ratio (primary:secondary).
    /// Output is scaled by 1/ratio (step-down).
    pub(super) turns_ratio: f64,
    /// Grid bias voltage (class AB operating point).
    pub(super) grid_bias: f64,
    /// DC blocker state: previous input sample (1-pole HPF, ~3.5Hz).
    pub(super) dc_blocker_x1: f64,
    /// DC blocker state: previous output sample.
    pub(super) dc_blocker_y1: f64,
    /// R-type adaptor for push half (3-port mode, when grid passives present).
    pub(super) push_adaptor: Option<PushPullHalfAdaptor>,
    /// R-type adaptor for pull half (3-port mode).
    pub(super) pull_adaptor: Option<PushPullHalfAdaptor>,
}

/// R-type adaptor data for one push-pull half (3-port mode).
///
/// When grid passives are present, the push-pull half uses an R-type adaptor
/// with the grid as a WDF port instead of a simple WDF tree. This allows
/// coupling caps and grid stoppers to naturally AC-couple the signal.
pub(super) struct PushPullHalfAdaptor {
    pub(super) adaptor: RTypeAdaptor,
    pub(super) device: NlDeviceGroupKind,
    pub(super) scattering: MultiNlScattering,
    pub(super) passive_children: Vec<DynNode>,
    pub(super) nl_port_resistances: Vec<f64>,
    pub(super) v_prev: Vec<f64>,
    pub(super) dc_bias: Vec<f64>,
    pub(super) output_port: usize,
    pub(super) n_nl: usize,
    pub(super) vs_injection: Option<Vec<f64>>,
    pub(super) vcc_bias_all: Vec<f64>,
    pub(super) dc_ramp: u32,
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
        let push_b = std::cell::Cell::new(0.0f64);
        #[cfg(feature = "debug-trace")]
        let push_a = std::cell::Cell::new(0.0f64);

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
                eprintln!(
                    "[PP n={n}] in={input:.6e} comp={comp:.4} bias={bias:.2} \
                     vgk_push={vgk_push:.4} vgk_pull={vgk_pull:.4} \
                     vs={push_vs:.1} rp={push_rp:.1}"
                );
                eprintln!(
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

        // DC ramp
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
    multi_port_nr_solve, multi_port_nr_solve_grouped, NlDeviceGroupIv, NlDeviceIv,
};
use crate::elements::nonlinear::{PentodeThreePort, VariMuThreePort};

/// Nonlinear device kind for the multi-NL solver.
///
/// Each variant wraps a concrete nonlinear root type that implements
/// `NlDeviceIv`, providing the I-V characteristic and its derivative.
#[allow(dead_code)]
pub(super) enum NlDeviceKind {
    BjtNpn(BjtNpnRoot),
    BjtPnp(BjtPnpRoot),
    Triode(TriodeRoot),
    VariMu(VariMuTriodeRoot),
    Pentode(PentodeRoot),
    Diode(DiodeRoot),
    DiodePair(DiodePairRoot),
}

impl NlDeviceKind {
    /// Set the control voltage for this NL device from the input signal.
    ///
    /// `bias_offset` is an additional bias voltage from external controls
    /// (e.g., BJT bias pots). Zero for non-BJT devices.
    pub(super) fn set_control_voltage(&mut self, input: f64, compensation: f64, bias_offset: f64) {
        match self {
            NlDeviceKind::BjtNpn(bjt) => {
                bjt.set_vbe(BJT_VBE_BIAS + bias_offset + input * compensation * BJT_VBE_SCALE);
            }
            NlDeviceKind::BjtPnp(bjt) => {
                bjt.set_veb(PNP_VEB_BIAS + bias_offset + input * compensation * PNP_VEB_SCALE);
            }
            NlDeviceKind::Triode(t) => {
                t.set_vgk(TRIODE_GRID_BIAS + input * compensation);
            }
            NlDeviceKind::VariMu(t) => {
                t.set_vgk(TRIODE_GRID_BIAS + input * compensation);
            }
            NlDeviceKind::Pentode(p) => {
                p.set_vg1k(PENTODE_GRID_BIAS + input * compensation);
            }
            NlDeviceKind::Diode(_) | NlDeviceKind::DiodePair(_) => {}
        }
    }

    /// Get a reference to this device as an `NlDeviceIv` trait object.
    pub(super) fn as_nl_device_iv(&self) -> &dyn NlDeviceIv {
        match self {
            NlDeviceKind::BjtNpn(bjt) => bjt,
            NlDeviceKind::BjtPnp(bjt) => bjt,
            NlDeviceKind::Triode(t) => t,
            NlDeviceKind::VariMu(t) => t,
            NlDeviceKind::Pentode(p) => p,
            NlDeviceKind::Diode(d) => d,
            NlDeviceKind::DiodePair(d) => d,
        }
    }

    pub(super) fn debug_name(&self) -> &'static str {
        match self {
            NlDeviceKind::BjtNpn(_) => "BjtNpn",
            NlDeviceKind::BjtPnp(_) => "BjtPnp",
            NlDeviceKind::Triode(_) => "Triode",
            NlDeviceKind::VariMu(_) => "VariMu",
            NlDeviceKind::Pentode(_) => "Pentode",
            NlDeviceKind::Diode(_) => "Diode",
            NlDeviceKind::DiodePair(_) => "DiodePair",
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
pub(super) enum NlDeviceGroupKind {
    /// 3-port variable-mu triode (grid-cathode + plate-cathode).
    VariMuThreePort(VariMuThreePort),
    /// 3-port Koren triode (grid-cathode + plate-cathode) for MNA fallback.
    TriodeThreePort(TriodeThreePort),
    /// 3-port Koren pentode (grid-cathode + plate-cathode) for push-pull.
    PentodeThreePort(PentodeThreePort),
    /// 2-port BJT (base-emitter + collector-emitter) using Gummel-Poon.
    BjtTwoPort(BjtTwoPort),
    /// Single-port NL device adapted as a 1-port device group.
    /// Used for mixed-device collapsed stages (e.g., sidechain with
    /// triodes + pentodes + diodes in one MultiNlStage).
    SinglePort(NlDeviceKind),
}

impl NlDeviceGroupKind {
    pub(super) fn as_group_iv(&self) -> &dyn NlDeviceGroupIv {
        match self {
            NlDeviceGroupKind::VariMuThreePort(t) => t,
            NlDeviceGroupKind::TriodeThreePort(t) => t,
            NlDeviceGroupKind::PentodeThreePort(p) => p,
            NlDeviceGroupKind::BjtTwoPort(b) => b,
            NlDeviceGroupKind::SinglePort(d) => d,
        }
    }

    pub(super) fn debug_name(&self) -> &'static str {
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
            NlDeviceGroupKind::SinglePort(d) => d.debug_name(),
        }
    }

    pub(super) fn n_ports(&self) -> usize {
        match self {
            NlDeviceGroupKind::VariMuThreePort(_) => 2,
            NlDeviceGroupKind::TriodeThreePort(_) => 2,
            NlDeviceGroupKind::PentodeThreePort(_) => 2,
            NlDeviceGroupKind::BjtTwoPort(_) => 2,
            NlDeviceGroupKind::SinglePort(_) => 1,
        }
    }
}

/// Grouped device configuration for MultiNlStage.
///
/// When present, the multi-port NR solver uses cross-coupled device Jacobians
/// instead of treating each port independently.
pub(super) struct MultiNlDeviceGroups {
    pub(super) groups: Vec<NlDeviceGroupKind>,
    pub(super) offsets: Vec<usize>,
}

/// Data needed to recompute the scattering matrix when pot values change.
///
/// The MNA conductance matrix stores only fixed resistor stamps (immutable).
/// Port node pairs are fixed; only port resistances change (from pots).
/// On pot change, we rebuild `WdfPort`s with current resistances, re-derive
/// the scattering matrix, and update all sub-blocks.
pub(super) struct ScatteringRecomputeData {
    /// MNA system with fixed resistors stamped (no pots).
    pub(super) mna: MnaSystem,
    /// Port node pairs: (pos_mna_idx, neg_mna_idx) for each port.
    /// Ordering: [NL_0..NL_{n-1}, passive_0..passive_{m-1}] (VS injection mode)
    /// Or: [NL_0..NL_{n-1}, passive_0..passive_{m-1}, adapted] (standard mode).
    pub(super) port_node_pairs: Vec<(Option<usize>, Option<usize>)>,
    /// Resistance of the adapted (voltage source) port.
    pub(super) adapted_resistance: f64,
    /// When Some, the input VS is stamped as an ideal voltage source in MNA B/C
    /// matrices. The value is the MNA VS branch index. Recompute uses
    /// `derive_scattering_and_vs_injection()` instead of `derive_scattering_matrix_general()`.
    pub(super) vs_source_index: Option<usize>,
    /// VCC voltage source index in MNA. When Some, VCC is an ideal VS and
    /// recompute re-extracts dc_bias from the VCC injection vector.
    pub(super) vcc_vs_index: Option<usize>,
    /// Output MNA node pair for direct node-voltage extraction.
    /// When Some, recompute also updates the extraction coefficients.
    pub(super) extract_output_nodes: Option<(Option<usize>, Option<usize>)>,
}

/// Scattering matrix sub-blocks for multi-NL solving.
///
/// These sub-blocks are extracted from the full R-type adaptor scattering
/// matrix and are the only parts needed during per-sample NR solving.
/// They are recomputed together when pot values change.
pub(super) struct MultiNlScattering {
    /// NL-to-NL sub-block (n_nl × n_nl, row-major).
    pub(super) s_nl: Vec<f64>,
    /// NL-to-passive sub-block (n_nl × n_passive, row-major).
    pub(super) s_nl_passive: Vec<f64>,
    /// NL-to-adapted column (n_nl).
    pub(super) s_nl_adapted: Vec<f64>,
}

impl MultiNlScattering {
    /// Extract sub-blocks from a full scattering matrix.
    ///
    /// The full matrix is `n_total × n_total` (row-major) with port ordering:
    /// `[NL_0..NL_{n-1}, passive_0..passive_{m-1}, (vcc?), adapted]`.
    /// `n_total` is inferred from the scattering matrix size, so extra ports
    /// (like VCC) are handled automatically — only the first `n_nl` rows and
    /// the NL, passive, and last (adapted) columns are extracted.
    pub(super) fn from_full_matrix(scattering: &[f64], n_nl: usize, n_passive: usize) -> Self {
        let n_total = if scattering.is_empty() {
            n_nl + n_passive + 1
        } else {
            let len = scattering.len();
            let nt = (len as f64).sqrt() as usize;
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
pub(super) struct MultiNlStage {
    /// R-type adaptor containing the full scattering matrix.
    pub(super) adaptor: RTypeAdaptor,
    /// Nonlinear devices at the NL ports.
    pub(super) nl_devices: Vec<NlDeviceKind>,
    /// Port resistances for the NL ports.
    pub(super) nl_port_resistances: Vec<f64>,
    /// Passive child nodes (capacitors, inductors) needing WDF state updates.
    pub(super) passive_children: Vec<DynNode>,
    /// Pot DynNodes stored separately — pots are G-matrix conductances, not WDF ports.
    pub(super) pot_children: Vec<DynNode>,
    /// MNA stamp tracking for pots: (pot_child_idx, node_pos, node_neg, last_conductance).
    /// Used for delta-updating the G matrix when pot values change.
    pub(super) pot_mna_stamps: Vec<(usize, Option<usize>, Option<usize>, f64)>,
    /// Number of nonlinear ports.
    pub(super) n_nl: usize,
    /// Warm-start voltages for NR solver.
    pub(super) v_prev: Vec<f64>,
    /// Scattering matrix sub-blocks for NR solving.
    pub(super) scattering: MultiNlScattering,
    /// Oversampler for antialiasing.
    pub(super) oversampler: Oversampler,
    /// Passive attenuation compensation factor.
    pub(super) compensation: f64,
    /// Which NL port to tap for output.
    pub(super) output_port: usize,
    /// Optional device groups for cross-coupled NR solve (3-port triodes).
    /// When Some, uses `multi_port_nr_solve_grouped()` instead of `multi_port_nr_solve()`.
    pub(super) device_groups: Option<MultiNlDeviceGroups>,
    /// Data for recomputing scattering matrix when pots change.
    /// None if the stage has no pots (no recomputation needed).
    pub(super) recompute_data: Option<ScatteringRecomputeData>,
    /// BFS distance from input of the injection node (for topological ordering).
    pub(super) signal_flow_distance: usize,
    /// Inter-stage voltage gain from a transformer boundary.
    pub(super) transformer_gain: f64,
    /// Circuit graph node ID (for debug routing).
    pub(super) injection_node_id: usize,
    /// Circuit graph node ID (for debug routing).
    pub(super) output_node_id: usize,
    /// Flag: pot changed since last scattering recompute.
    pub(super) recompute_pending: bool,
    /// VEB bias offset from a feedback pot.
    pub(super) veb_bias_offset: f64,
    /// Feedback scale for coupled BJT stages.
    pub(super) feedback_scale: f64,
    /// Linearized OTA data for gm-based scattering recompute.
    /// When Some, the OTA's transconductance is stamped into the MNA as a linear
    /// conductance. When the envelope changes gain, we delta-update the MNA and
    /// recompute the scattering matrix. No NR iteration needed.
    pub(super) linearized_ota: Option<LinearizedOtaData>,
    /// VS injection vector for ideal voltage source input.
    /// When Some, signal is injected via `a[i] += k[i] * V_in` instead of
    /// an adapted WDF port. Used for linearized OTA stages where the adapted
    /// port would share an MNA node with a reactive port.
    pub(super) vs_injection: Option<Vec<f64>>,
    /// Node-voltage extraction coefficients for direct output reading.
    /// When Some, the output is computed as:
    ///   V_out = Σ_k extract_coeffs[k] * b[k] + extract_vs * V_in
    /// This bypasses WDF port impedance mismatch by reading the MNA node
    /// voltage directly from X⁻¹ coefficients.
    pub(super) extract_coeffs: Option<Vec<f64>>,
    pub(super) extract_vs: f64,
    /// When set, identifies a pot in pot_children whose resistance drives
    /// BJT bias recalculation (feedback_scale + veb_bias_offset).
    pub(super) bias_pot_id: Option<String>,
    /// Emitter resistance for bias pot computation (default 470Ω).
    pub(super) bias_emitter_r: f64,
    /// State-space model for direct discrete-time simulation.
    /// When Some, process() uses state-space update (A·x + b·u) instead of
    /// WDF scattering. Used for linearized OTA stages where cap port
    /// conductances overwhelm circuit conductances.
    pub(super) state_space: Option<StateSpaceData>,
    /// Precomputed interpolation table for single-pot stages.
    /// When Some, pot changes use table lookup instead of MNA re-inversion.
    pub(super) interp_table: Option<ScatteringInterpolationTable>,
    /// Precomputed DC bias from VCC supply injection vector (NL ports only).
    /// dc_bias[i] = vcc_injection[i] * supply_voltage, for i in 0..n_nl.
    /// Added to known_a[i] in the NR solver to establish transistor operating points.
    pub(super) dc_bias: Vec<f64>,
    /// Full VCC injection vector × supply_voltage for ALL ports.
    /// Added to a_all after scatter_all to provide DC bias to passive children
    /// and correct output extraction. Length = n_nl + n_passive + adapted(0 or 1).
    pub(super) vcc_bias_all: Vec<f64>,
    /// VCC voltage source index in the MNA (for recomputing dc_bias on pot changes).
    /// When Some, VCC is stamped as an ideal VS in the MNA with zero impedance.
    pub(super) vcc_vs_index: Option<usize>,
    /// Nominal supply voltage (volts). Used for dc_bias computation.
    pub(super) supply_voltage: f64,
    /// DC ramp counter for gradual bias injection.
    /// Counts from 0 to DC_RAMP_SAMPLES, scaling dc_bias by ramp/N to let the
    /// NR solver track the operating point as supply voltage gradually increases.
    pub(super) dc_ramp: u32,
    /// Physics-based initial v_prev values. Restored on reset() instead of zeroing,
    /// so the NR solver starts near the correct operating point after a DAW reset.
    pub(super) initial_v_prev: Vec<f64>,
    /// DC blocker state: previous input sample (x[n-1]).
    pub(super) dc_blocker_x1: f64,
    /// DC blocker state: previous output sample (y[n-1]).
    pub(super) dc_blocker_y1: f64,
}

/// State-space data for direct discrete-time simulation.
///
/// Replaces WDF scattering for linearized OTA stages where cap port
/// conductances dominate circuit conductances, causing identity S-matrix rows.
/// Uses bilinear transform on the continuous-time MNA to get node-voltage
/// dynamics directly, avoiding WDF port-impedance scaling issues.
pub(super) struct StateSpaceData {
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
pub(super) struct LinearizedOtaData {
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

        // ── State-space path: direct discrete-time simulation ────────────
        // Bypasses WDF scattering entirely. Used for linearized OTA stages
        // where cap port conductances overwhelm circuit conductances.
        if let Some(ref mut ss) = self.state_space {
            let output = self.oversampler.process(input, |sample| {
                let n = ss.n_states;
                // x[n] = A · x[n-1] + b · u[n]
                let mut x_new = vec![0.0; n];
                for i in 0..n {
                    let mut v = ss.b_vector[i] * sample;
                    let row_start = i * n;
                    for j in 0..n {
                        v += ss.a_matrix[row_start + j] * ss.x[j];
                    }
                    x_new[i] = flush_denormal(v);
                }
                // y[n] = c · x[n]
                let mut y = 0.0;
                for i in 0..n {
                    y += ss.c_vector[i] * x_new[i];
                }
                ss.x = x_new;
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
        let dc_scale = if self.dc_ramp >= DC_RAMP_SAMPLES {
            1.0
        } else {
            self.dc_ramp += 1;
            self.dc_ramp as f64 / DC_RAMP_SAMPLES as f64
        };

        let output = self.oversampler.process(input, |sample| {
            // 1. Scatter-up passive children
            let mut b_passive = Vec::with_capacity(n_passive);
            for child in &mut self.passive_children {
                b_passive.push(child.reflected());
            }

            // 2. Compute known_a[i] for each NL port:
            // known_a[i] = Σ_k S_nl_passive[i][k] * b_passive[k]
            //             + S_nl_adapted[i] * b_adapted
            //             + dc_bias[i]  (VCC supply contribution, precomputed constant)
            // The adapted port's b-wave is the input signal (voltage source)
            let b_adapted = sample * compensation;
            let mut known_a = vec![0.0; n_nl];
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
                static NR_TRACE: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
                let n = NR_TRACE.fetch_add(1, Ordering::Relaxed);
                if n < 5 {
                    eprintln!("[NR-input] n_nl={} sample={:.6e} comp={} known_a={:?} b_passive={:?} b_adapted={:.6e} s_nl_adapted={:?}", n_nl, sample, compensation, known_a, b_passive, b_adapted, &self.scattering.s_nl_adapted);
                }
            }
            // 3. Multi-port NR solve (skipped when n_nl=0, e.g. linearized OTA)
            let b_nl = if n_nl == 0 {
                // No NR ports — signal propagation is fully encoded in S matrix.
                Vec::new()
            } else if let Some(ref dg) = self.device_groups {
                // Grouped solver: cross-coupled device Jacobians
                let groups: Vec<&dyn NlDeviceGroupIv> =
                    dg.groups.iter().map(|g| g.as_group_iv()).collect();
                multi_port_nr_solve_grouped(
                    n_nl,
                    &self.scattering.s_nl,
                    &known_a,
                    &self.nl_port_resistances,
                    &groups,
                    &dg.offsets,
                    &mut self.v_prev,
                    crate::elements::nonlinear::solver::NR_MAX_ITER,
                    1e-6,
                )
            } else {
                // Independent solver: each device has its own I-V
                let devices: Vec<&dyn NlDeviceIv> = self
                    .nl_devices
                    .iter()
                    .map(|d| d.as_nl_device_iv())
                    .collect();
                multi_port_nr_solve(
                    n_nl,
                    &self.scattering.s_nl,
                    &known_a,
                    &self.nl_port_resistances,
                    &devices,
                    &mut self.v_prev,
                    crate::elements::nonlinear::solver::NR_MAX_ITER,
                    1e-6,
                )
            };

            #[cfg(feature = "debug-trace")]
            {
                static NR_TRACE2: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
                let n = NR_TRACE2.fetch_add(1, Ordering::Relaxed);
                if n < 5 {
                    eprintln!("[NR-output] b_nl={:?} v_prev={:?}", b_nl, self.v_prev);
                }
            }
            // 4. Build full b-vector for scatter_down:
            //    [b_nl..., b_passive..., b_adapted]
            //    With VS injection: no adapted port at end.
            //    VCC is an MNA voltage source (not a WDF port), so no b_vcc entry.
            let use_vs_injection = self.vs_injection.is_some();
            let n_total = n_nl + n_passive + if use_vs_injection { 0 } else { 1 };
            let mut b_all = Vec::with_capacity(n_total);
            b_all.extend_from_slice(&b_nl);
            b_all.extend_from_slice(&b_passive);
            if !use_vs_injection {
                b_all.push(b_adapted);
            }

            // Use scatter_all to get incident waves for all ports
            let mut a_all = self.adaptor.scatter_all(&b_all);

            // Add VS injection: a[i] += k[i] * V_in
            if let Some(ref k) = self.vs_injection {
                for i in 0..a_all.len() {
                    if i < k.len() {
                        a_all[i] += k[i] * b_adapted;
                    }
                }
            }

            // Add VCC supply injection to all ports (with DC ramp).
            // The scattering matrix was derived with VCC as a VS, so scatter_all
            // only gives port-to-port interactions. The VCC contribution must be
            // added separately: a[i] += vcc_inj[i] * V_supply.
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
                // Direct node-voltage extraction: V = Σ coeffs[k]*b[k] + vs*V_in
                let mut v_out = self.extract_vs * b_adapted;
                for k in 0..n_passive {
                    v_out += coeffs[k] * b_passive[k];
                }
                for k in 0..n_nl {
                    v_out += coeffs[n_passive + k] * b_nl[k];
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
                let y0 = x0 - self.dc_blocker_x1 + 0.9995 * self.dc_blocker_y1;
                let y0 = if y0.is_finite() { y0 } else { 0.0 };
                self.dc_blocker_x1 = x0;
                self.dc_blocker_y1 = y0;
                y0
            } else {
                raw_out
            }
        });

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
                eprintln!(
                    "[MNL n={n}] [{stage_id}] n_nl={} out_port={} comp={:.4} xfmr={:.2}",
                    self.n_nl, self.output_port, self.compensation, self.transformer_gain
                );
                eprintln!("  in={input:.6e} out={output:.6e} gain={gain_db:.1}dB");
                eprintln!("  s_nl_adapted={:.6?}", &self.scattering.s_nl_adapted);
                // s_nl diagonal (self-coupling) and off-diagonal
                let mut s_diag = Vec::with_capacity(self.n_nl);
                for i in 0..self.n_nl {
                    s_diag.push(self.scattering.s_nl[i * self.n_nl + i]);
                }
                eprintln!("  s_nl_diag={:.6?}", s_diag);
                eprintln!(
                    "  v_prev={:.4?} Rp={:.1?}",
                    &self.v_prev, &self.nl_port_resistances
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
    pub fn apply_oversampling_rate(&mut self, _base_rate: f64) {
        // No-op: build_rtype_stage() now uses effective_rate = sample_rate * oversampling
        // for all DynNode creation and scattering derivation, so children are already
        // at the correct rate and the scattering matrix is consistent.
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
        self.dc_ramp = 0;
        self.dc_blocker_x1 = 0.0;
        self.dc_blocker_y1 = 0.0;
        if let Some(ref mut ss) = self.state_space {
            for v in &mut ss.x {
                *v = 0.0;
            }
        }
        self.oversampler.reset();
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
                let (a_d, b_d, c_out, _n) = recompute.mna.build_state_space_matrices(
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
            } else {
                // No VS at all: standard scattering derivation
                let scattering = recompute.mna.derive_scattering_matrix_general(&ports);

                if scattering.iter().any(|&s| !s.is_finite()) {
                    return;
                }

                self.scattering = MultiNlScattering::from_full_matrix(&scattering, n_nl, n_passive);
                let port_resistances: Vec<f64> = ports.iter().map(|p| p.resistance).collect();
                self.adaptor = RTypeAdaptor::new(scattering, &port_resistances);
            }
        }
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
pub(super) struct SidechainProcessor {
    /// The compiled sidechain sub-circuit.
    pub(super) circuit: super::compiled::CompiledPedal,
    /// 1-sample delay state for the feedback loop CV.
    pub(super) cv_delayed: f64,
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
