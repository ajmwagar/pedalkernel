//! WDF clipping/processing stage combining a tree with a nonlinear root.

use crate::elements::*;
use crate::oversampling::Oversampler;
use crate::tree::{MnaSystem, RTypeAdaptor, WdfPort};
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
    /// Sample counter for runtime warnings rate limiting.
    /// Only meaningful when `runtime-warnings` feature is enabled.
    #[allow(dead_code)]
    pub(super) sample_counter: u64,
    /// Component ID for the root device (for runtime warning attribution).
    /// Only meaningful when `runtime-warnings` feature is enabled.
    #[allow(dead_code)]
    pub(super) root_comp_id: String,
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
            // - Other: VS = input * compensation
            let vs_voltage = if let RootKind::Triode(t) = root {
                t.v_max()
            } else if let RootKind::VariMu(t) = root {
                t.v_max()
            } else if is_sf {
                0.0 // Source follower: input modulates Vgs, not VS
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
            let b_tree = tree.reflected();
            let rp = tree.port_resistance();

            let a_root = match root {
                RootKind::DiodePair(dp) => dp.process(b_tree, rp),
                RootKind::SingleDiode(d) => d.process(b_tree, rp),
                RootKind::Zener(z) => z.process(b_tree, rp),
                RootKind::Jfet(j) => {
                    if is_sf {
                        // Source follower: solve Vs where Ids(Vgate - Vs) = Vs/Rs
                        // Vgate = input signal (sample), Vs is what we're solving for
                        j.process_source_follower(b_tree, rp, sample * compensation)
                    } else {
                        // Normal JFET (phaser, common-source): Vgs set externally
                        j.process(b_tree, rp)
                    }
                }
                RootKind::JfetVr(j) => {
                    // Variable resistance mode: simple resistor reflection.
                    // No NR iterations — Rds is pre-computed from Vgs.
                    j.process_root(b_tree, rp)
                }
                RootKind::Triode(t) => t.process(b_tree, rp),
                RootKind::VariMu(t) => t.process(b_tree, rp),
                RootKind::Pentode(p) => p.process(b_tree, rp),
                RootKind::Mosfet(m) => m.process(b_tree, rp),
                RootKind::Ota(o) => o.process(b_tree, rp),
                RootKind::OpAmp(op) => {
                    // For non-inverting op-amps, the input signal must be set via set_vp().
                    // Inverting op-amps derive input from the WDF wave variable.
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
                    // For passive filters with embedded voltage source, the output
                    // voltage at the load is half the root wave (resistive extraction)
                    if let Some(_) = tree.resistive_termination_voltage(b_tree) {
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
            tree.set_incident(a_root);
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
            RootKind::InductorRoot {
                inductance, rp, ..
            } => {
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
            ..
        } = &mut self.root
        {
            if !*needs_recompute {
                return;
            }
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
            "WdfStage(root={}, compensation={:.6}, tree_rp={:.1}Ω, nodes={})\n",
            root_name,
            self.compensation,
            self.tree.port_resistance(),
            self.tree.node_count()
        );
        s.push_str(&self.tree.debug_dump(1));
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
}

impl TubeRoot {
    #[inline]
    pub fn set_vgk(&mut self, vgk: f64) {
        match self {
            TubeRoot::Koren(t) => t.set_vgk(vgk),
            TubeRoot::VariMu(t) => t.set_vgk(vgk),
        }
    }

    #[inline]
    pub fn v_max(&self) -> f64 {
        match self {
            TubeRoot::Koren(t) => t.v_max(),
            TubeRoot::VariMu(t) => t.v_max(),
        }
    }

    #[inline]
    pub fn set_v_max(&mut self, v_max: f64) {
        match self {
            TubeRoot::Koren(t) => t.set_v_max(v_max),
            TubeRoot::VariMu(t) => t.set_v_max(v_max),
        }
    }

    #[inline]
    pub fn process(&mut self, b_tree: f64, rp: f64) -> f64 {
        match self {
            TubeRoot::Koren(t) => t.process(b_tree, rp),
            TubeRoot::VariMu(t) => t.process(b_tree, rp),
        }
    }

    #[inline]
    pub fn plate_current(&self, vpk: f64) -> f64 {
        match self {
            TubeRoot::Koren(t) => t.plate_current(vpk),
            TubeRoot::VariMu(t) => t.plate_current(vpk),
        }
    }

    pub fn parallel_count(&self) -> usize {
        match self {
            TubeRoot::Koren(t) => t.parallel_count(),
            TubeRoot::VariMu(t) => t.parallel_count(),
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
}

impl PushPullStage {
    /// Process one sample through the push-pull stage.
    /// Input is applied with opposite polarity to push and pull halves.
    /// Output is the differential plate voltage divided by turns ratio.
    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
        let comp = self.compensation;
        let bias = self.grid_bias;

        // Push: positive phase, Pull: negative phase
        let vgk_push = bias + input * comp;
        let vgk_pull = bias - input * comp;
        self.push_root.set_vgk(vgk_push);
        self.pull_root.set_vgk(vgk_pull);

        let push_out = self.push_oversampler.process(input, |_| {
            let vs = self.push_root.v_max();
            self.push_tree.set_voltage(vs);
            let b = self.push_tree.reflected();
            let rp = self.push_tree.port_resistance();
            let a = self.push_root.process(b, rp);
            self.push_tree.set_incident(a);
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
        let output = diff / self.turns_ratio;

        #[cfg(feature = "debug-trace")]
        if input.abs() > 1e-10 {
            let n = TRACE_COUNT_PP.fetch_add(1, Ordering::Relaxed);
            if n < MAX_TRACE_PP {
                let push_rp = self.push_tree.port_resistance();
                let push_vs = self.push_root.v_max();
                eprintln!(
                    "[PP n={n}] in={input:.6e} comp={comp:.4} bias={bias:.2} \
                     vgk_push={vgk_push:.4} vgk_pull={vgk_pull:.4} \
                     vs={push_vs:.1} rp={push_rp:.1}"
                );
                eprintln!(
                    "  push_out={push_out:.6e} pull_out={pull_out:.6e} \
                     diff={diff:.6e} ratio={:.2} out={output:.6e}",
                    self.turns_ratio
                );
            }
        }

        flush_denormal(output)
    }

    pub fn debug_dump(&self) -> String {
        format!(
            "PushPullStage(ratio={:.1}:1, bias={:.1}V, push_par={}, pull_par={}, comp={:.4})\n  Push: rp={:.1}Ω, nodes={}\n  Pull: rp={:.1}Ω, nodes={}",
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
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Multi-NL coupled stage (R-type adaptor + multi-port NR solver)
// ═══════════════════════════════════════════════════════════════════════════

use crate::elements::nonlinear::solver::{
    multi_port_nr_solve, multi_port_nr_solve_grouped, NlDeviceGroupIv, NlDeviceIv,
};
use crate::elements::nonlinear::VariMuThreePort;

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
    fn as_nl_device_iv(&self) -> &dyn NlDeviceIv {
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
    /// Single-port NL device adapted as a 1-port device group.
    /// Used for mixed-device collapsed stages (e.g., sidechain with
    /// triodes + pentodes + diodes in one MultiNlStage).
    SinglePort(NlDeviceKind),
}

impl NlDeviceGroupKind {
    fn as_group_iv(&self) -> &dyn NlDeviceGroupIv {
        match self {
            NlDeviceGroupKind::VariMuThreePort(t) => t,
            NlDeviceGroupKind::TriodeThreePort(t) => t,
            NlDeviceGroupKind::SinglePort(d) => d,
        }
    }

    pub(super) fn debug_name(&self) -> &'static str {
        match self {
            NlDeviceGroupKind::VariMuThreePort(_) => "VariMuThreePort",
            NlDeviceGroupKind::TriodeThreePort(_) => "TriodeThreePort",
            NlDeviceGroupKind::SinglePort(d) => d.debug_name(),
        }
    }

    pub(super) fn n_ports(&self) -> usize {
        match self {
            NlDeviceGroupKind::VariMuThreePort(_) => 2,
            NlDeviceGroupKind::TriodeThreePort(_) => 2,
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
    /// `[NL_0..NL_{n-1}, passive_0..passive_{m-1}, adapted]`.
    pub(super) fn from_full_matrix(
        scattering: &[f64],
        n_nl: usize,
        n_passive: usize,
    ) -> Self {
        let n_total = n_nl + n_passive + 1;
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
    /// State-space model for direct discrete-time simulation.
    /// When Some, process() uses state-space update (A·x + b·u) instead of
    /// WDF scattering. Used for linearized OTA stages where cap port
    /// conductances overwhelm circuit conductances.
    pub(super) state_space: Option<StateSpaceData>,
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

        // Set control voltages on each NL device (only for independent devices;
        // grouped devices get their grid voltage from the WDF port).
        if self.device_groups.is_none() {
            let bias_offset = self.veb_bias_offset;
            for device in &mut self.nl_devices {
                device.set_control_voltage(input, compensation, bias_offset);
            }
        }

        let output = self.oversampler.process(input, |sample| {
            // 1. Scatter-up passive children
            let mut b_passive = Vec::with_capacity(n_passive);
            for child in &mut self.passive_children {
                b_passive.push(child.reflected());
            }

            // 2. Compute known_a[i] for each NL port:
            // known_a[i] = Σ_k S_nl_passive[i][k] * b_passive[k]
            //             + S_nl_adapted[i] * b_adapted
            // The adapted port's b-wave is the input signal (voltage source)
            let b_adapted = sample * compensation;
            let mut known_a = vec![0.0; n_nl];
            for i in 0..n_nl {
                let mut a_i = self.scattering.s_nl_adapted[i] * b_adapted;
                for k in 0..n_passive {
                    a_i += self.scattering.s_nl_passive[i * n_passive + k] * b_passive[k];
                }
                known_a[i] = a_i;
            }

            // 3. Multi-port NR solve (skipped when n_nl=0, e.g. linearized OTA)
            let b_nl = if n_nl == 0 {
                // No NR ports — signal propagation is fully encoded in S matrix.
                Vec::new()
            } else if let Some(ref dg) = self.device_groups {
                // Grouped solver: cross-coupled device Jacobians
                let groups: Vec<&dyn NlDeviceGroupIv> =
                    dg.groups.iter().map(|g| g.as_group_iv()).collect();
                let result = multi_port_nr_solve_grouped(
                    n_nl,
                    &self.scattering.s_nl,
                    &known_a,
                    &self.nl_port_resistances,
                    &groups,
                    &dg.offsets,
                    &mut self.v_prev,
                    20,
                    1e-6,
                );
                result
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
                    20,
                    1e-6,
                )
            };

            // 4. Build full b-vector for scatter_down:
            //    With VS injection: [b_nl_0..., b_passive_0...] (no adapted port)
            //    Without:           [b_nl_0..., b_passive_0..., b_adapted]
            let use_vs_injection = self.vs_injection.is_some();
            let n_total = if use_vs_injection {
                n_nl + n_passive
            } else {
                n_nl + n_passive + 1
            };
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

            // 5. Set incident waves on passive children
            for (k, child) in self.passive_children.iter_mut().enumerate() {
                child.set_incident(a_all[n_nl + k]);
            }

            // 6. Output extraction
            if let Some(ref coeffs) = self.extract_coeffs {
                // Direct node-voltage extraction: V = Σ coeffs[k]*b[k] + vs*V_in
                // This reads the MNA node voltage directly, bypassing port
                // impedance mismatch that makes WDF port waves tiny.
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
    /// response. Note: the scattering matrix is NOT recomputed here because
    /// the MNA-derived matrix uses port resistance ratios that are scale-
    /// invariant — the relative impedances stay correct.
    pub fn apply_oversampling_rate(&mut self, base_rate: f64) {
        let ratio = self.oversampler.ratio();
        if ratio <= 1 {
            return;
        }
        let effective_rate = base_rate * ratio as f64;
        for child in &mut self.passive_children {
            child.set_sample_rate(effective_rate);
        }
    }

    /// Reset all internal state.
    pub fn reset(&mut self) {
        for child in &mut self.passive_children {
            child.reset();
        }
        for v in &mut self.v_prev {
            *v = 0.0;
        }
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
        let n_total = self.n_nl + n_passive + 1;
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
        let emitter_r = 470.0;
        self.feedback_scale = emitter_r / (emitter_r + pot_r);
        self.veb_bias_offset = self.feedback_scale * 0.1;
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

        let recompute = match &mut self.recompute_data {
            Some(r) => r,
            None => return,
        };

        // Delta-update pot conductances in the MNA G matrix.
        // Pots are stamped into G (not as WDF ports), so when pot values change
        // we update G before re-deriving the scattering matrix.
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
        let n_total = if use_vs { n_nl + n_passive } else { n_nl + n_passive + 1 };

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
            // VS injection mode: derive scattering + injection vector (no adapted port).
            let vs_idx = recompute.vs_source_index.unwrap();
            let (scattering, vs_inj) =
                recompute.mna.derive_scattering_and_vs_injection(&ports, vs_idx);

            if scattering.iter().any(|&s| !s.is_finite()) {
                return;
            }

            self.scattering = MultiNlScattering::from_full_matrix(&scattering, n_nl, n_passive);
            let port_resistances: Vec<f64> = ports.iter().map(|p| p.resistance).collect();
            self.adaptor = RTypeAdaptor::new(scattering, &port_resistances);
            self.vs_injection = Some(vs_inj);

            // Recompute extraction coefficients if used.
            if let Some((out_pos, out_neg)) = recompute.extract_output_nodes {
                let (coeffs, vs_coeff) =
                    recompute.mna.derive_extraction_coeffs(&ports, vs_idx, out_pos, out_neg);
                self.extract_coeffs = Some(coeffs);
                self.extract_vs = vs_coeff;
            }
        } else {
            // Standard mode: adapted port included.
            let (pos, neg) = recompute.port_node_pairs[n_nl + n_passive];
            ports.push(WdfPort {
                node_pos: pos,
                node_neg: neg,
                resistance: recompute.adapted_resistance,
            });

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
