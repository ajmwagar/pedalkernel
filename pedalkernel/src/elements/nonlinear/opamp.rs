//! Op-amp (voltage-controlled voltage source) WDF root elements.
//!
//! Models common op-amps (TL072, LM308, JRC4558) as VCVS with feedback.

use crate::elements::WdfRoot;

// ---------------------------------------------------------------------------
// Op-Amp (Voltage-Controlled Voltage Source) Models
// ---------------------------------------------------------------------------

/// Op-amp model parameters for WDF integration.
///
/// Models the op-amp as a voltage-controlled voltage source:
/// `Vout = Aol * (Vp - Vm)`
///
/// Where:
/// - `Aol` is the open-loop DC gain (typically 100k-1M)
/// - `Vp` is the non-inverting input voltage (set externally)
/// - `Vm` is the inverting input voltage (often feedback-dependent)
///
/// The feedback network stays in the WDF tree as passive components.
/// The op-amp root enforces the gain relationship between its differential
/// input and output via Newton-Raphson iteration.
///
/// Non-idealities captured:
/// - Finite open-loop gain
/// - Output voltage saturation
/// - Slew rate limiting (optional, applied post-convergence)
/// - Gain-bandwidth product (closed-loop bandwidth rolloff)
#[derive(Debug, Clone, Copy)]
pub struct OpAmpModel {
    /// Open-loop DC gain. TL072 ≈ 200k (106dB), LM308 ≈ 300k.
    pub open_loop_gain: f64,
    /// Gain-bandwidth product (Hz). TL072 ≈ 3MHz, LM308 ≈ 1MHz.
    /// Determines closed-loop bandwidth: f_cl = GBW / closed_loop_gain.
    pub gbw: f64,
    /// Slew rate (V/µs). TL072 ≈ 13, LM308 ≈ 0.3.
    pub slew_rate: f64,
    /// Maximum output voltage swing (V, single-sided from Vcc/2).
    /// Typically Vcc/2 - 1.5V for rail-to-rail, less for others.
    pub v_max: f64,
    /// Output impedance (Ω). Typically 50-200Ω for voltage-feedback op-amps.
    /// Used by the VCVS stamp in the MNA nullor path.
    pub output_impedance: f64,
    /// Output capacitance (F). Physical capacitance of the output stage.
    /// Keeps the output node as a dynamic state in the state-space reduction,
    /// preserving the feedback loop's energy recirculation. Typical: 10-50pF.
    pub output_capacitance: f64,
}

impl OpAmpModel {
    /// TL072 — JFET-input dual op-amp.
    ///
    /// The modern standard for guitar pedals. Fast slew rate (13 V/µs),
    /// low noise, high input impedance. Used in Klon Centaur, Phase 90,
    /// and countless modern designs.
    pub fn tl072() -> Self {
        Self {
            open_loop_gain: 200_000.0, // 106 dB
            gbw: 3e6,                  // 3 MHz
            slew_rate: 13.0,           // 13 V/µs
            v_max: 12.0,               // ±12V swing at ±15V supply
            output_impedance: 75.0,  // Ω
            output_capacitance: 20e-12, // 20pF
        }
    }

    /// TL082 — JFET-input dual op-amp (TL072 variant).
    ///
    /// Similar to TL072 but slightly different specs. Used interchangeably
    /// in many designs.
    pub fn tl082() -> Self {
        Self {
            open_loop_gain: 200_000.0,
            gbw: 4e6,
            slew_rate: 13.0,
            v_max: 12.0,
            output_impedance: 75.0,
            output_capacitance: 20e-12,
        }
    }

    pub fn lm308() -> Self {
        Self {
            open_loop_gain: 300_000.0,
            gbw: 1e6,
            slew_rate: 0.3,
            v_max: 12.0,
            output_impedance: 200.0, // Higher Ro than JFET types
            output_capacitance: 30e-12, // 30pF — slower output stage
        }
    }

    pub fn lm741() -> Self {
        Self {
            open_loop_gain: 200_000.0,
            gbw: 1e6,
            slew_rate: 0.5,
            v_max: 12.0,
            output_impedance: 75.0,
            output_capacitance: 20e-12,
        }
    }

    pub fn jrc4558() -> Self {
        Self {
            open_loop_gain: 100_000.0,
            gbw: 3e6,
            slew_rate: 1.7,
            v_max: 12.0,
            output_impedance: 75.0,
            output_capacitance: 20e-12,
        }
    }

    pub fn rc4558() -> Self {
        Self {
            open_loop_gain: 100_000.0,
            gbw: 3e6,
            slew_rate: 1.5,
            v_max: 12.0,
            output_impedance: 75.0,
            output_capacitance: 20e-12,
        }
    }

    pub fn ne5532() -> Self {
        Self {
            open_loop_gain: 100_000.0,
            gbw: 10e6,
            slew_rate: 9.0,
            v_max: 12.0,
            output_impedance: 50.0, // Lower Ro — studio grade
            output_capacitance: 15e-12, // 15pF — fast output
        }
    }

    pub fn op07() -> Self {
        Self {
            open_loop_gain: 400_000.0,
            gbw: 0.6e6,
            slew_rate: 0.3,
            v_max: 12.0,
            output_impedance: 60.0,
            output_capacitance: 25e-12,
        }
    }

    pub fn generic() -> Self {
        Self {
            open_loop_gain: 100_000.0,
            gbw: 1e6,
            slew_rate: 1.0,
            v_max: 12.0,
            output_impedance: 75.0,
            output_capacitance: 20e-12,
        }
    }

    /// Convert from DSL OpAmpType to runtime OpAmpModel.
    pub fn from_opamp_type(ot: &crate::dsl::OpAmpType) -> Self {
        use crate::dsl::OpAmpType;
        match ot {
            OpAmpType::Generic => Self::generic(),
            OpAmpType::Tl072 => Self::tl072(),
            OpAmpType::Tl082 => Self::tl082(),
            OpAmpType::Jrc4558 => Self::jrc4558(),
            OpAmpType::Rc4558 => Self::rc4558(),
            OpAmpType::Lm308 => Self::lm308(),
            OpAmpType::Lm741 => Self::lm741(),
            OpAmpType::Ne5532 => Self::ne5532(),
            OpAmpType::Op07 => Self::op07(),
            // OTAs (CA3080) are handled separately as OtaRoot, not OpAmpRoot
            OpAmpType::Ca3080 => Self::generic(),
        }
    }
}

// ---------------------------------------------------------------------------
// Op-Amp Root (VCVS WDF Element)
// ---------------------------------------------------------------------------

/// Configuration for a feedback pot in an op-amp gain stage.
///
/// Stores topology info so the op-amp can recompute its gain from the
/// pot's current resistance, without relying on external side-effect systems.
#[derive(Debug, Clone)]
pub struct FeedbackConfig {
    /// Component ID of the pot controlling gain.
    pub pot_comp_id: String,
    /// Resistance of the other leg (Ri when pot is Rf, Rf when pot is Ri).
    pub other_leg_r: f64,
    /// Fixed resistance in series with the pot.
    pub fixed_series_r: f64,
    /// Fixed resistance in parallel across (pot + series R). None if no parallel R.
    pub parallel_r: Option<f64>,
    /// True = pot is in the feedback (Rf) leg; false = pot is in the input (Ri) leg.
    pub pot_is_feedback: bool,
    /// True = inverting topology; false = non-inverting.
    pub is_inverting: bool,
}

/// Op-amp nonlinear root for WDF trees.
///
/// Models the op-amp as a voltage-controlled voltage source:
/// `Vout = Aol * (Vp - Vm)`
///
/// For a unity-gain buffer (voltage follower), Vm = Vout (direct feedback),
/// which creates the familiar virtual short between inputs. The Newton-Raphson
/// solver finds the output voltage that satisfies both the op-amp equation
/// and the WDF constraint.
///
/// **Usage in Phase 90 all-pass stages:**
/// Each op-amp is configured as a unity-gain buffer (neg tied to out).
/// Op-amp topology mode.
///
/// Both topologies use the same op-amp model (nullor/VCVS). The difference is
/// where the input signal enters relative to the feedback path:
///
/// - **Inverting**: Input to V- through Ri, V+ grounded. Gain = -Rf/Ri.
///   Input comes from WDF wave variable (the virtual ground at V-).
///
/// - **NonInverting**: Input to V+, feedback to V-. Gain = 1 + Rf/Ri.
///   Input set via `set_vp()`. Unity-gain buffer is NonInverting with gain=1.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OpAmpMode {
    /// Inverting amplifier: Vout = -gain * Vin
    /// Input comes from WDF wave variable (virtual ground at V-).
    Inverting { gain: f64 },
    /// Non-inverting amplifier: Vout = gain * Vp
    /// Input set via `set_vp()`. Unity buffer = gain of 1.0.
    NonInverting { gain: f64 },
}

/// Unified op-amp root for WDF trees.
///
/// Post-scattering non-ideality filter for op-amps absorbed into R-type
/// adaptors via `MnaSystem::stamp_vcvs`.
///
/// The VCVS stamp inside the MNA already captures the frequency-independent
/// gain (Aol) and output impedance (Ro). The genuinely non-linear-time-
/// invariant behaviours — supply rail clamping and slew rate limiting —
/// cannot live in the linear scattering matrix, so they run as a tiny
/// per-sample filter on the extracted output node voltage.
///
/// This replaces the `feedback_opamp: Option<OpAmpRoot>` field that diode-
/// paired stages used in the legacy topology-detection path. With the
/// unified nullor pipeline, any op-amp whose output is the stage's
/// audio output gets an `OpAmpPostFx` attached.
#[derive(Debug, Clone)]
pub struct OpAmpPostFx {
    /// Datasheet parameters (slew_rate, v_max).
    pub model: OpAmpModel,
    /// Sample rate for slew-per-sample conversion.
    pub sample_rate: f64,
    /// Previous output voltage for slew-rate limiting.
    prev_out: f64,
}

impl OpAmpPostFx {
    /// Create with model parameters and sample rate.
    pub fn new(model: OpAmpModel, sample_rate: f64) -> Self {
        Self {
            model,
            sample_rate,
            prev_out: 0.0,
        }
    }

    /// Apply rail clamping then slew-rate limiting in that order.
    ///
    /// Rail clamping first so the slew limiter is bounded by the clamped
    /// target. This matches SPICE behaviour where the op-amp can't exceed
    /// the supply rails regardless of input rate of change.
    pub fn process(&mut self, v_raw: f64) -> f64 {
        let v_max = self.model.v_max;
        let v_clip = v_raw.clamp(-v_max, v_max);
        // slew_rate is V/µs in the datasheet. Convert to V/sample.
        let slew_max = self.model.slew_rate * 1e6 / self.sample_rate;
        let delta = (v_clip - self.prev_out).clamp(-slew_max, slew_max);
        let v_out = self.prev_out + delta;
        self.prev_out = v_out;
        v_out
    }

    /// Reset state (typically on stage reset / sample rate change).
    pub fn reset(&mut self) {
        self.prev_out = 0.0;
    }
}

/// Models op-amp behavior in two topologies:
/// - **Inverting**: Vout = -(Rf/Ri) * Vin, input from WDF wave (virtual ground)
/// - **Non-inverting**: Vout = (1 + Rf/Ri) * Vp, input via `set_vp()`
///
/// Unity-gain buffer (voltage follower) is just NonInverting with gain=1.
/// The "third mode" doesn't exist - it's the same circuit with Rf=0, Ri=∞.
///
/// Non-idealities:
/// - GBW-limited closed-loop bandwidth (single-pole rolloff at `f_cl = GBW / gain`)
/// - Slew rate limiting (LM308's slow slew = RAT's character)
/// - Output saturation at supply rails
/// - Soft clipping via feedback diodes (optional)
#[derive(Debug, Clone)]
pub struct OpAmpRoot {
    pub model: OpAmpModel,
    /// Operating mode (inverting/non-inverting).
    mode: OpAmpMode,
    /// Non-inverting input voltage (for NonInverting mode).
    vp: f64,
    /// Previous output voltage (for slew rate limiting).
    prev_out: f64,
    /// Sample rate (needed for slew rate limiting and GBW filter).
    sample_rate: f64,
    /// Soft clipping limit from feedback diodes.
    /// If set, uses tanh-based soft clipping instead of hard clipping.
    soft_clip_v: Option<f64>,
    /// GBW rolloff filter state (single-pole LPF on closed-loop output).
    /// Models the dominant-pole frequency response: f_cl = GBW / closed_loop_gain.
    gbw_state: f64,
    /// GBW rolloff filter coefficient: g = 1 - exp(-2π·f_cl/fs).
    /// Recomputed when gain or sample rate changes.
    gbw_coeff: f64,
    /// Actual closed-loop gain for GBW bandwidth calculation ONLY.
    ///
    /// In the 3-port adaptor path, the VCVS mode gain is set to 1.0 because
    /// the scattering matrix handles the output impedance — but the GBW filter
    /// still needs the true closed-loop gain to compute f_cl = GBW / gbw_gain.
    /// For all other paths, gbw_gain == self.gain().
    gbw_gain: f64,
    /// Feedback pot configuration. When set, the op-amp recomputes its own
    /// gain from the pot's resistance via `set_feedback_pot_r()`.
    feedback_config: Option<FeedbackConfig>,
}

impl OpAmpRoot {
    /// Compute GBW rolloff coefficient for the single-pole closed-loop filter.
    ///
    /// The closed-loop bandwidth of an opamp with gain G is approximately:
    ///   f_cl = GBW / G
    ///
    /// We model this as a single-pole LPF: g = 1 - exp(-2π·f_cl / fs).
    /// When f_cl >= fs/2 (Nyquist), the filter is transparent (g = 1.0).
    #[inline]
    fn compute_gbw_coeff(gbw: f64, gain: f64, sample_rate: f64) -> f64 {
        let g = gain.max(1.0);
        let f_cl = gbw / g;
        if f_cl >= sample_rate * 0.5 {
            1.0 // Bandwidth exceeds Nyquist — no filtering needed
        } else {
            1.0 - (-std::f64::consts::TAU * f_cl / sample_rate).exp()
        }
    }

    /// Create a unity-gain buffer (voltage follower).
    ///
    /// This is a non-inverting amplifier with gain=1 (Rf=0, Ri=∞).
    /// Input via `set_vp()`, output tracks Vp exactly.
    pub fn new(model: OpAmpModel) -> Self {
        let gbw_coeff = Self::compute_gbw_coeff(model.gbw, 1.0, 48000.0);
        Self {
            model,
            mode: OpAmpMode::NonInverting { gain: 1.0 },
            vp: 0.0,
            prev_out: 0.0,
            sample_rate: 48000.0,
            soft_clip_v: None,
            gbw_state: 0.0,
            gbw_coeff,
            gbw_gain: 1.0,
            feedback_config: None,
        }
    }

    /// Create a unity-gain buffer (alias for `new`).
    pub fn unity_gain(model: OpAmpModel) -> Self {
        Self::new(model)
    }

    /// Create an inverting op-amp: Vout = -gain * Vin.
    pub fn new_inverting(model: OpAmpModel, gain: f64) -> Self {
        let g = gain.abs();
        let gbw_coeff = Self::compute_gbw_coeff(model.gbw, g, 48000.0);
        Self {
            model,
            mode: OpAmpMode::Inverting { gain: g },
            vp: 0.0,
            prev_out: 0.0,
            sample_rate: 48000.0,
            soft_clip_v: None,
            gbw_state: 0.0,
            gbw_coeff,
            gbw_gain: g,
            feedback_config: None,
        }
    }

    /// Create a non-inverting op-amp: Vout = gain * Vin.
    pub fn new_non_inverting(model: OpAmpModel, gain: f64) -> Self {
        let g = gain.abs();
        let gbw_coeff = Self::compute_gbw_coeff(model.gbw, g, 48000.0);
        Self {
            model,
            mode: OpAmpMode::NonInverting { gain: g },
            vp: 0.0,
            prev_out: 0.0,
            sample_rate: 48000.0,
            soft_clip_v: None,
            gbw_state: 0.0,
            gbw_coeff,
            gbw_gain: g,
            feedback_config: None,
        }
    }

    /// Set the non-inverting input voltage (V+).
    ///
    /// For non-inverting mode (including unity buffer), this is the input signal.
    /// For inverting mode, this is typically ground or a bias voltage.
    #[inline]
    pub fn set_vp(&mut self, vp: f64) {
        self.vp = vp;
    }

    /// Get the current non-inverting input voltage.
    #[inline]
    pub fn vp(&self) -> f64 {
        self.vp
    }

    /// Set the closed-loop gain. Recomputes GBW rolloff coefficient.
    #[inline]
    pub fn set_gain(&mut self, gain: f64) {
        let g = gain.abs();
        match &mut self.mode {
            OpAmpMode::Inverting { gain: gv } => *gv = g,
            OpAmpMode::NonInverting { gain: gv } => *gv = g,
        }
        self.gbw_gain = g;
        self.gbw_coeff = Self::compute_gbw_coeff(self.model.gbw, g, self.sample_rate);
    }

    /// Set only the GBW gain used for bandwidth calculation, without changing the VCVS gain.
    ///
    /// Used by the 3-port adaptor path where the VCVS mode gain is 1.0 (the scattering
    /// matrix handles port impedance), but the GBW rolloff filter must use the true
    /// closed-loop gain to compute f_cl = GBW / gbw_gain correctly.
    #[inline]
    pub fn set_gbw_gain(&mut self, gbw_gain: f64) {
        self.gbw_gain = gbw_gain.abs().max(1.0);
        self.gbw_coeff = Self::compute_gbw_coeff(self.model.gbw, self.gbw_gain, self.sample_rate);
    }

    /// Get the current gain.
    #[inline]
    pub fn gain(&self) -> f64 {
        match &self.mode {
            OpAmpMode::Inverting { gain } => *gain,
            OpAmpMode::NonInverting { gain } => *gain,
        }
    }

    /// Check if this op-amp is in non-inverting mode.
    ///
    /// Non-inverting op-amps require input via `set_vp()`.
    /// Inverting op-amps get input from the WDF wave variable.
    #[inline]
    pub fn is_non_inverting(&self) -> bool {
        matches!(self.mode, OpAmpMode::NonInverting { .. })
    }

    /// Enable soft clipping mode for feedback diodes.
    #[inline]
    pub fn set_soft_clip(&mut self, diode_vf: f64) {
        self.soft_clip_v = Some(diode_vf.max(0.1));
    }

    /// Disable soft clipping.
    #[inline]
    pub fn clear_soft_clip(&mut self) {
        self.soft_clip_v = None;
    }

    /// Set the sample rate (for slew rate limiting and GBW filter).
    #[inline]
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
        self.gbw_coeff = Self::compute_gbw_coeff(self.model.gbw, self.gbw_gain, sample_rate);
    }

    /// Get the current sample rate.
    #[inline]
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Get the current GBW filter coefficient (for testing).
    #[inline]
    pub fn gbw_coeff(&self) -> f64 {
        self.gbw_coeff
    }

    /// Set the maximum output voltage (supply rails).
    #[inline]
    pub fn set_v_max(&mut self, v_max: f64) {
        self.model.v_max = v_max.max(0.5);
    }

    /// Get the current v_max setting.
    #[inline]
    pub fn v_max(&self) -> f64 {
        self.model.v_max
    }

    /// Set feedback configuration for pot-driven gain control.
    ///
    /// After this is set, `set_feedback_pot_r()` can be called to recompute
    /// the gain from the pot's current resistance.
    pub fn set_feedback_config(&mut self, cfg: FeedbackConfig) {
        self.feedback_config = Some(cfg);
    }

    /// Get the feedback pot component ID, if configured.
    pub fn feedback_pot_id(&self) -> Option<&str> {
        self.feedback_config
            .as_ref()
            .map(|c| c.pot_comp_id.as_str())
    }

    /// Recompute gain from the feedback pot's current resistance.
    ///
    /// Called by the stage when it detects the feedback pot changed.
    /// The gain formula depends on the topology:
    /// - Pot in Rf, inverting:      gain = effective_rf / ri
    /// - Pot in Rf, non-inverting:  gain = 1 + effective_rf / ri
    /// - Pot in Ri, non-inverting:  gain = 1 + rf / effective_ri
    pub fn set_feedback_pot_r(&mut self, pot_r: f64) {
        let cfg = match &self.feedback_config {
            Some(c) => c,
            None => return,
        };
        let gain = if cfg.pot_is_feedback {
            // Pot is in Rf leg
            let eff_r = (cfg.fixed_series_r + pot_r).max(500.0);
            let rf = match cfg.parallel_r {
                Some(pr) => (pr * eff_r) / (pr + eff_r),
                None => eff_r,
            };
            if cfg.is_inverting {
                rf / cfg.other_leg_r
            } else {
                1.0 + rf / cfg.other_leg_r
            }
        } else {
            // Pot is in Ri leg
            let ri = (cfg.fixed_series_r + pot_r).max(100.0);
            if cfg.is_inverting {
                cfg.other_leg_r / ri
            } else {
                1.0 + cfg.other_leg_r / ri
            }
        };
        #[cfg(test)]
        eprintln!("[OPAMP_DEBUG] set_feedback_pot_r: pot_r={pot_r:.1} pot_is_fb={} is_inv={} fixed_series={:.1} parallel_r={:?} other_leg={:.1} → gain={gain:.4}",
            cfg.pot_is_feedback, cfg.is_inverting, cfg.fixed_series_r, cfg.parallel_r, cfg.other_leg_r);
        // set_gain updates both the VCVS mode gain and gbw_gain, then recomputes gbw_coeff.
        self.set_gain(gain);
    }

    /// Configure feedback topology (for advanced use).
    /// This is a no-op for gain stages; use set_gain() instead.
    #[inline]
    pub fn set_feedback(&mut self, _ratio: f64, _vm_external: f64) {
        // Kept for API compatibility with existing code
    }

    /// Apply slew rate limiting.
    #[inline]
    fn apply_slew_limit(&mut self, v: f64) -> f64 {
        let max_dv = self.model.slew_rate * 1e6 / self.sample_rate;
        let dv = v - self.prev_out;
        let limited = if dv > max_dv {
            self.prev_out + max_dv
        } else if dv < -max_dv {
            self.prev_out - max_dv
        } else {
            v
        };
        self.prev_out = limited;
        limited
    }

    /// Compute the voltage source value for a DiodePair stage with opamp feedback.
    ///
    /// Applies inverting gain + GBW rolloff + slew limiting, but NOT soft clipping
    /// (the DiodePair NR solver handles the actual clipping).
    ///
    /// For an inverting opamp with feedback diodes, the signal path is:
    ///   input → Ri → virtual ground (neg) → diodes in feedback → output
    /// The opamp gain sets the drive level into the diode clipping stage.
    ///
    /// Returns the magnitude (positive) of the gained voltage, since WDF
    /// DiodePair convention already negates the VS voltage.
    #[inline]
    pub fn compute_vs_voltage(&mut self, input: f64) -> f64 {
        let gain = self.gain();
        let v_max = self.model.v_max;

        // Apply gain (magnitude only — WDF handles sign convention)
        let mut v_out = gain * input;

        // GBW rolloff: single-pole LPF at f_cl = GBW / gain
        v_out = self.gbw_coeff * v_out + (1.0 - self.gbw_coeff) * self.gbw_state;
        self.gbw_state = v_out;

        // Hard clip at supply rails
        v_out = v_out.clamp(-v_max, v_max);

        // Slew rate limiting
        v_out = self.apply_slew_limit(v_out);

        v_out
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.prev_out = 0.0;
        self.vp = 0.0;
        self.gbw_state = 0.0;
    }
}

impl WdfRoot for OpAmpRoot {
    /// Op-amp processing based on mode.
    #[inline]
    fn process(&mut self, a: f64, _rp: f64) -> f64 {
        let v_max = self.model.v_max;

        #[cfg(test)]
        {
            static OPAMP_PROC_TRACE: std::sync::atomic::AtomicU64 =
                std::sync::atomic::AtomicU64::new(0);
            let n = OPAMP_PROC_TRACE.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            // X4 oversampling: 4800 warmup × 4 = 19200 sub-samples before signal
            if n >= 19200 && n < 19220 {
                eprintln!("[OPAMP_PROC] n={n} a={a:.6e} vp={:.6e} gain={:.4} gbw_coeff={:.6} gbw_state={:.6e} v_max={v_max:.2} mode={:?}",
                    self.vp, self.gain(), self.gbw_coeff, self.gbw_state,
                    match self.mode { OpAmpMode::Inverting{..} => "inv", OpAmpMode::NonInverting{..} => "ni" });
            }
        }

        // Compute output voltage based on topology
        let mut v_out = match self.mode {
            OpAmpMode::Inverting { gain } => {
                // Inverting: Vout = -gain * Vin, input from WDF wave (virtual ground)
                let v_in = a / 2.0;
                -gain * v_in
            }
            OpAmpMode::NonInverting { gain } => {
                // Non-inverting: Vout = gain * Vp, input set externally
                // Unity buffer is just gain=1.0
                gain * self.vp
            }
        };

        // Apply GBW-limited bandwidth: single-pole LPF at f_cl = GBW / gain.
        // This models the dominant-pole rolloff that makes LM308 (1MHz) darker
        // than TL072 (3MHz) at high gain settings.
        v_out = self.gbw_coeff * v_out + (1.0 - self.gbw_coeff) * self.gbw_state;
        self.gbw_state = v_out;

        // Apply soft clipping if feedback diodes present
        if let Some(vd) = self.soft_clip_v {
            v_out = vd * (v_out / vd).tanh();
        }

        // Hard clip at supply rails
        v_out = v_out.clamp(-v_max, v_max);

        // Apply slew rate limiting
        v_out = self.apply_slew_limit(v_out);

        // Return reflected wave: b = 2*Vout - a
        2.0 * v_out - a
    }
}
