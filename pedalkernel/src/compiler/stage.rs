//! WDF clipping/processing stage combining a tree with a nonlinear root.

use crate::elements::*;
use crate::oversampling::Oversampler;

use super::dyn_node::DynNode;
use super::helpers::balance_parallel_vs;

// ═══════════════════════════════════════════════════════════════════════════
// WDF clipping stage
// ═══════════════════════════════════════════════════════════════════════════

pub(super) enum RootKind {
    DiodePair(DiodePairRoot),
    SingleDiode(DiodeRoot),
    Zener(ZenerRoot),
    Jfet(JfetRoot),
    Triode(TriodeRoot),
    Pentode(PentodeRoot),
    Mosfet(MosfetRoot),
    Ota(OtaRoot),
    /// Voltage-mode op-amp (TL072, LM308, JRC4558, etc.).
    /// Modeled as a VCVS: Vout = Aol * (Vp - Vm).
    OpAmp(OpAmpRoot),
    /// Inverting op-amp amplifier with closed-loop gain.
    /// Vout = -(Rf/Ri) * Vin. The input comes through the WDF tree.
    OpAmpInverting(OpAmpInvertingRoot),
    /// Non-inverting op-amp amplifier with closed-loop gain.
    /// Vout = (1 + Rf/Ri) * Vp. The input is set via set_vp().
    OpAmpNonInverting(OpAmpNonInvertingRoot),
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
    /// First-order IIR lowpass filter (analytical, not WDF-based).
    /// Used for simple RC lowpass circuits where WDF topology issues cause
    /// incorrect frequency response.
    /// H(z) = b0*(1 + z^-1) / (1 - a1*z^-1)
    IirLowpass {
        /// Filter coefficient a1 = (1 - ωRC)/(1 + ωRC) where ω = 2*fs
        a1: f64,
        /// Filter coefficient b0 = 1/(1 + ωRC)
        b0: f64,
        /// Previous output y[n-1]
        y_prev: f64,
        /// Previous input x[n-1]
        x_prev: f64,
    },
    /// First-order IIR highpass filter (analytical, not WDF-based).
    /// Used for simple RC highpass circuits.
    /// H(z) = b0*(1 - z^-1) / (1 - a1*z^-1)
    IirHighpass {
        /// Filter coefficient a1 = (1 - ωRC)/(1 + ωRC) where ω = 2*fs
        a1: f64,
        /// Filter coefficient b0 = ωRC/(1 + ωRC)
        b0: f64,
        /// Previous output y[n-1]
        y_prev: f64,
        /// Previous input x[n-1]
        x_prev: f64,
    },
}

impl RootKind {
    /// Returns `true` for roots that clip the signal (diodes, zeners).
    ///
    /// Clipping stages reduce the signal to roughly the diode forward voltage
    /// (~0.3–0.7 V), so cascaded clipping stages need inter-stage gain to
    /// re-amplify before the next clipper.  Non-clipping roots (JFETs acting
    /// as variable resistors, op-amp buffers, transistor gain stages) pass
    /// the signal through at roughly the same amplitude; applying extra gain
    /// before each one causes exponential level growth.
    pub(super) fn is_clipping_stage(&self) -> bool {
        matches!(
            self,
            RootKind::DiodePair(_) | RootKind::SingleDiode(_) | RootKind::Zener(_)
        )
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
    /// DC-blocking highpass filter for triode stages.
    /// Models the output coupling capacitor's DC blocking behavior.
    /// Format: (a1, b0, y_prev, x_prev) for IIR highpass.
    pub(super) dc_block: Option<(f64, f64, f64, f64)>,
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
        // Borrow fields individually to satisfy the borrow checker
        let tree = &mut self.tree;
        let root = &mut self.root;
        let compensation = self.compensation;

        // For non-inverting op-amp stages, the input signal goes directly to Vp.
        // Set this before the oversampler so each oversampled cycle uses the correct input.
        if let RootKind::OpAmpNonInverting(ref mut noninv) = root {
            noninv.set_vp(input * compensation);
        }

        // For triode stages, the input signal modulates Vgk (grid-cathode voltage).
        // This is the key to triode amplification: the input controls the grid,
        // which modulates plate current and creates voltage drop across R_plate.
        // Vgk = Vbias + Vin (Vbias ~-2V for 12AX7 with typical biasing)
        if let RootKind::Triode(ref mut t) = root {
            // Apply input signal to grid. The bias point is typically around -2V
            // for class A operation with a 12AX7. The input signal swings around this.
            const TRIODE_GRID_BIAS: f64 = -2.0;
            t.set_vgk(TRIODE_GRID_BIAS + input * compensation);
        }

        let wdf_out = self.oversampler.process(input, |sample| {
            // For triodes, the voltage source is the B+ supply (v_max).
            // The input already modulated Vgk; the supply biases the plate circuit.
            let vs_voltage = if let RootKind::Triode(t) = root {
                t.v_max() // B+ supply voltage for triode bias
            } else {
                sample * compensation
            };
            tree.set_voltage(vs_voltage);
            let b_tree = tree.reflected();
            let rp = tree.port_resistance();

            let a_root = match root {
                RootKind::DiodePair(dp) => dp.process(b_tree, rp),
                RootKind::SingleDiode(d) => d.process(b_tree, rp),
                RootKind::Zener(z) => z.process(b_tree, rp),
                RootKind::Jfet(j) => j.process(b_tree, rp),
                RootKind::Triode(t) => t.process(b_tree, rp),
                RootKind::Pentode(p) => p.process(b_tree, rp),
                RootKind::Mosfet(m) => m.process(b_tree, rp),
                RootKind::Ota(o) => o.process(b_tree, rp),
                RootKind::OpAmp(op) => op.process(b_tree, rp),
                RootKind::OpAmpInverting(inv) => inv.process(b_tree, rp),
                RootKind::OpAmpNonInverting(noninv) => noninv.process(b_tree, rp),
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
                // IIR lowpass: bypass WDF tree, use direct filter
                RootKind::IirLowpass { a1, b0, y_prev, x_prev } => {
                    // First-order IIR: y[n] = b0*(x[n] + x[n-1]) + a1*y[n-1]
                    let x = sample * compensation;
                    let y = *b0 * (x + *x_prev) + *a1 * *y_prev;
                    *x_prev = x;
                    *y_prev = y;
                    return y; // Skip normal WDF processing
                }
                // IIR highpass: bypass WDF tree, use direct filter
                RootKind::IirHighpass { a1, b0, y_prev, x_prev } => {
                    // First-order IIR: y[n] = b0*(x[n] - x[n-1]) + a1*y[n-1]
                    let x = sample * compensation;
                    let y = *b0 * (x - *x_prev) + *a1 * *y_prev;
                    *x_prev = x;
                    *y_prev = y;
                    return y; // Skip normal WDF processing
                }
            };
            tree.set_incident(a_root);
            (a_root + b_tree) / 2.0
        });

        // If a paired op-amp buffer is present, it models the all-pass
        // output reconstruction.  The op-amp receives the stage input as
        // Vp (non-inverting input) and uses the WDF junction voltage to
        // compute: V_allpass = 2 * V_junction - V_in.
        //
        // This is the standard all-pass output formula derived from the
        // bridged-T topology: the op-amp with gain 2 applied to the
        // voltage divider output produces H(s) = (Z-R)/(Z+R), which
        // has |H| = 1 (unity magnitude, phase shift only).
        if let Some(ref mut opamp) = self.paired_opamp {
            // Feed the WDF junction voltage through the op-amp.
            // The op-amp's Vp was set to the stage input externally
            // (by the process loop in compiled.rs).
            let a = 2.0 * wdf_out;
            let b = opamp.process(a, tree.port_resistance());
            return (a + b) / 2.0;
        }

        // Apply DC-blocking filter for triode stages.
        // This models the output coupling capacitor (C_out) which blocks DC
        // and passes AC. The filter is a single-pole IIR highpass.
        if let Some((a1, b0, ref mut y_prev, ref mut x_prev)) = self.dc_block {
            // IIR highpass: y[n] = b0 * (x[n] - x[n-1]) + a1 * y[n-1]
            let x = wdf_out;
            let y = b0 * (x - *x_prev) + a1 * *y_prev;
            *x_prev = x;
            *y_prev = y;
            return y;
        }

        wdf_out
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
        if let RootKind::Jfet(j) = &mut self.root {
            j.set_vgs(vgs);
        }
    }

    /// Get the current gate-source voltage if this is a JFET stage.
    pub fn jfet_vgs(&self) -> Option<f64> {
        match &self.root {
            RootKind::Jfet(j) => Some(j.vgs()),
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
    pub fn triode_vgk(&self) -> Option<f64> {
        match &self.root {
            RootKind::Triode(t) => Some(t.vgk()),
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
    #[inline]
    pub fn set_pentode_vg2k(&mut self, vg2k: f64) {
        if let RootKind::Pentode(p) = &mut self.root {
            p.set_vg2k(vg2k);
        }
    }

    /// Get the current control grid voltage if this is a pentode stage.
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
    pub fn mosfet_vgs(&self) -> Option<f64> {
        match &self.root {
            RootKind::Mosfet(m) => Some(m.vgs()),
            _ => None,
        }
    }

    /// Set the OTA bias current (for envelope-controlled gain).
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
    #[inline]
    pub fn set_opamp_vp(&mut self, vp: f64) {
        if let RootKind::OpAmp(op) = &mut self.root {
            op.set_vp(vp);
        }
    }

    /// Get the current non-inverting input voltage if this is an op-amp stage.
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
        match &mut self.root {
            RootKind::OpAmpInverting(inv) => inv.set_gain(gain),
            RootKind::OpAmpNonInverting(noninv) => noninv.set_gain(gain),
            _ => {}
        }
    }

    /// Get the current gain if this is an op-amp gain stage.
    pub fn opamp_gain(&self) -> Option<f64> {
        match &self.root {
            RootKind::OpAmpInverting(inv) => Some(inv.gain()),
            RootKind::OpAmpNonInverting(noninv) => Some(noninv.gain()),
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
            RootKind::Triode(_) => "Triode",
            RootKind::Pentode(_) => "Pentode",
            RootKind::Mosfet(_) => "Mosfet",
            RootKind::Ota(_) => "Ota",
            RootKind::OpAmp(_) => "OpAmp",
            RootKind::OpAmpInverting(_) => "OpAmpInverting",
            RootKind::OpAmpNonInverting(_) => "OpAmpNonInverting",
            RootKind::BjtNpn(_) => "BjtNpn",
            RootKind::BjtPnp(_) => "BjtPnp",
            RootKind::Passthrough => "Passthrough",
            RootKind::CapacitorRoot { .. } => "CapacitorRoot",
            RootKind::InductorRoot { .. } => "InductorRoot",
            RootKind::IirLowpass { .. } => "IirLowpass",
            RootKind::IirHighpass { .. } => "IirHighpass",
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

/// A push-pull stage processes two triode halves simultaneously.
/// Push gets +Vin, pull gets -Vin. Output = push_v - pull_v.
/// Used for circuits like the Fairchild 670 where push and pull triodes
/// connect to opposite ends of a center-tapped output transformer.
pub(super) struct PushPullStage {
    /// WDF tree for push half (plate load + cathode passives).
    pub(super) push_tree: DynNode,
    /// WDF tree for pull half (plate load + cathode passives).
    pub(super) pull_tree: DynNode,
    /// Push triode model (with parallel_count for 4× parallel tubes).
    pub(super) push_root: TriodeRoot,
    /// Pull triode model (with parallel_count for 4× parallel tubes).
    pub(super) pull_root: TriodeRoot,
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
    /// Cathode coupling state (1-sample delay between push and pull cathodes).
    /// Stores the previous sample's cathode voltage difference.
    pub(super) cathode_delay_state: f64,
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
        self.push_root.set_vgk(bias + input * comp);
        self.pull_root.set_vgk(bias - input * comp);

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

        // Cathode coupling: 1-sample delay exchanges energy between halves.
        // This models the bidirectional cathode bypass capacitor interaction.
        let cathode_diff = push_out - pull_out;
        let prev = self.cathode_delay_state;
        self.cathode_delay_state = cathode_diff;

        // Differential output: push - pull, scaled by transformer turns ratio.
        // The CT transformer combines push and pull; step-down reduces voltage.
        let diff = push_out - pull_out;
        let coupled = diff * 0.5 + prev * 0.5; // Low-pass blend with delay
        coupled / self.turns_ratio
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
