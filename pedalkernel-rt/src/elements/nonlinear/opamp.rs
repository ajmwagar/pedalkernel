//! Op-amp (voltage-controlled voltage source) WDF root elements.
//!
//! Models common op-amps (TL072, LM308, JRC4558) as VCVS with feedback.

use alloc::string::String;
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
    /// Maximum positive output swing (V above bias point).
    /// For symmetric bias: same as v_rail_neg. For asymmetric: differs.
    pub v_rail_pos: f64,
    /// Maximum negative output swing (V below bias point).
    pub v_rail_neg: f64,
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
            v_rail_pos: 12.0,
            v_rail_neg: 12.0,               // ±12V swing at ±15V supply
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
            v_rail_pos: 12.0,
            v_rail_neg: 12.0,
            output_impedance: 75.0,
            output_capacitance: 20e-12,
        }
    }

    pub fn lm308() -> Self {
        Self {
            open_loop_gain: 300_000.0,
            gbw: 1e6,
            slew_rate: 0.3,
            v_rail_pos: 12.0,
            v_rail_neg: 12.0,
            output_impedance: 200.0, // Higher Ro than JFET types
            output_capacitance: 30e-12, // 30pF — slower output stage
        }
    }

    pub fn lm741() -> Self {
        Self {
            open_loop_gain: 200_000.0,
            gbw: 1e6,
            slew_rate: 0.5,
            v_rail_pos: 12.0,
            v_rail_neg: 12.0,
            output_impedance: 75.0,
            output_capacitance: 20e-12,
        }
    }

    pub fn jrc4558() -> Self {
        Self {
            open_loop_gain: 100_000.0,
            gbw: 3e6,
            slew_rate: 1.7,
            v_rail_pos: 12.0,
            v_rail_neg: 12.0,
            output_impedance: 75.0,
            output_capacitance: 20e-12,
        }
    }

    pub fn rc4558() -> Self {
        Self {
            open_loop_gain: 100_000.0,
            gbw: 3e6,
            slew_rate: 1.5,
            v_rail_pos: 12.0,
            v_rail_neg: 12.0,
            output_impedance: 75.0,
            output_capacitance: 20e-12,
        }
    }

    pub fn ne5532() -> Self {
        Self {
            open_loop_gain: 100_000.0,
            gbw: 10e6,
            slew_rate: 9.0,
            v_rail_pos: 12.0,
            v_rail_neg: 12.0,
            output_impedance: 50.0, // Lower Ro — studio grade
            output_capacitance: 15e-12, // 15pF — fast output
        }
    }

    pub fn op07() -> Self {
        Self {
            open_loop_gain: 400_000.0,
            gbw: 0.6e6,
            slew_rate: 0.3,
            v_rail_pos: 12.0,
            v_rail_neg: 12.0,
            output_impedance: 60.0,
            output_capacitance: 25e-12,
        }
    }

    pub fn generic() -> Self {
        Self {
            open_loop_gain: 100_000.0,
            gbw: 1e6,
            slew_rate: 1.0,
            v_rail_pos: 12.0,
            v_rail_neg: 12.0,
            output_impedance: 75.0,
            output_capacitance: 20e-12,
        }
    }

    // TODO: move to pedalkernel as extension impl
    // pub fn from_opamp_type(ot: &OpAmpType) -> Self { ... }
}

// ---------------------------------------------------------------------------
// Op-Amp Root (VCVS WDF Element)
// ---------------------------------------------------------------------------

/// Configuration for a feedback pot in an op-amp gain stage.
///
/// Stores topology info so the op-amp can recompute its gain from the
/// pot's current resistance, without relying on external side-effect systems.
///
/// DEPRECATED: Only used in test code. The compiler pipeline uses
/// `WdfStage::notify_pot_changed()` + incremental recompute instead.
/// Will be removed once test helpers are updated.
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
        let v_clip = v_raw.clamp(-self.model.v_rail_neg, self.model.v_rail_pos);
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
    /// Set symmetric rail limits (both positive and negative).
    pub fn set_v_max(&mut self, v_max: f64) {
        let v = v_max.max(0.5);
        self.model.v_rail_pos = v;
        self.model.v_rail_neg = v;
    }

    /// Set asymmetric rail limits (from bias analysis).
    pub fn set_v_rails(&mut self, v_rail_pos: f64, v_rail_neg: f64) {
        self.model.v_rail_pos = v_rail_pos.max(0.5);
        self.model.v_rail_neg = v_rail_neg.max(0.5);
    }

    /// Get the symmetric v_max (minimum of both rails).
    #[inline]
    pub fn v_max(&self) -> f64 {
        self.model.v_rail_pos.min(self.model.v_rail_neg)
    }

    /// Set feedback configuration for pot-driven gain control.
    ///
    /// After this is set, `set_feedback_pot_r()` can be called to recompute
    /// the gain from the pot's current resistance.
    ///
    /// DEPRECATED: Only used in test code. The compiler pipeline uses
    /// `WdfStage::notify_pot_changed()` + incremental recompute instead.
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
        #[cfg(all(test, feature = "std"))]
        std::eprintln!("[OPAMP_DEBUG] set_feedback_pot_r: pot_r={pot_r:.1} pot_is_fb={} is_inv={} fixed_series={:.1} parallel_r={:?} other_leg={:.1} → gain={gain:.4}",
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

        // Apply gain (magnitude only — WDF handles sign convention)
        let mut v_out = gain * input;

        // GBW rolloff: single-pole LPF at f_cl = GBW / gain
        v_out = self.gbw_coeff * v_out + (1.0 - self.gbw_coeff) * self.gbw_state;
        self.gbw_state = v_out;

        // Hard clip at supply rails (asymmetric)
        v_out = v_out.clamp(-self.model.v_rail_neg, self.model.v_rail_pos);

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

        // Hard clip at supply rails (asymmetric)
        v_out = v_out.clamp(-self.model.v_rail_neg, self.model.v_rail_pos);

        // Apply slew rate limiting
        v_out = self.apply_slew_limit(v_out);

        // Return reflected wave: b = 2*Vout - a
        2.0 * v_out - a
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests: capture OpAmpRoot nonideal behavior before refactoring to
// NonIdealFxState. These tests define the contract that must be preserved.
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(all(test, DISABLED_tests_in_pedalkernel))]
mod tests {
    use super::*;
    use crate::elements::WdfRoot;

    fn lm308() -> OpAmpModel {
        OpAmpModel::lm308()
    }

    fn tl072() -> OpAmpModel {
        OpAmpModel::tl072()
    }

    /// Helper: drive OpAmpRoot with a WDF incident wave `a` and extract
    /// the output voltage. In WDF: v_out = (a + b) / 2, where b = process(a).
    fn drive(root: &mut OpAmpRoot, a: f64) -> f64 {
        let b = root.process(a, 10_000.0);
        (a + b) / 2.0
    }

    // ── GBW rolloff ──────────────────────────────────────────────────────

    #[test]
    fn opamp_gbw_lm308_attenuates_hf_more_than_tl072() {
        // LM308: GBW=1MHz. TL072: GBW=3MHz.
        // At gain=100, LM308 fc=10kHz, TL072 fc=30kHz.
        // A 15kHz signal should be attenuated more by LM308 (fc below signal)
        // than TL072 (fc above signal).
        let sr = 48000.0;
        let gain = 100.0;
        let freq = 15000.0;

        let mut lm_model = lm308();
        lm_model.slew_rate = 100.0; // fast slew so it doesn't interfere
        lm_model.v_rail_pos = 1000.0; lm_model.v_rail_neg = 1000.0;
        let mut tl_model = tl072();
        tl_model.slew_rate = 100.0;
        tl_model.v_rail_pos = 1000.0; tl_model.v_rail_neg = 1000.0;

        let mut lm = OpAmpRoot::new_inverting(lm_model, gain);
        lm.set_sample_rate(sr);
        let mut tl = OpAmpRoot::new_inverting(tl_model, gain);
        tl.set_sample_rate(sr);

        let measure = |root: &mut OpAmpRoot| {
            for s in 0..500 {
                let v = 0.001 * (std::f64::consts::TAU * freq * s as f64 / sr).sin();
                drive(root, 2.0 * v);
            }
            let mut peak = 0.0f64;
            for s in 500..600 {
                let v = 0.001 * (std::f64::consts::TAU * freq * (500 + s) as f64 / sr).sin();
                peak = peak.max(drive(root, 2.0 * v).abs());
            }
            peak
        };

        let peak_lm = measure(&mut lm);
        let peak_tl = measure(&mut tl);
        eprintln!("GBW: LM308 peak={peak_lm:.6}, TL072 peak={peak_tl:.6}");
        assert!(peak_tl > peak_lm * 1.1, "TL072 (higher GBW) should pass more HF: tl={peak_tl:.6} > lm={peak_lm:.6}");
    }

    // ── Slew rate limiting ───────────────────────────────────────────────

    #[test]
    fn opamp_slew_lm308_limits_step_response() {
        // LM308 slew = 0.3 V/µs = 300kV/s.
        // At 48kHz: max_dv = 300000/48000 ≈ 6.25V/sample.
        // A step from 0 to 10V should be limited on the first sample.
        let sr = 48000.0;
        let mut root = OpAmpRoot::new_non_inverting(lm308(), 1.0);
        root.set_sample_rate(sr);

        // Settle at 0
        for _ in 0..100 {
            root.set_vp(0.0);
            drive(&mut root, 0.0);
        }

        // Step to 10V
        root.set_vp(10.0);
        let out = drive(&mut root, 0.0);
        eprintln!("Slew LM308: step 0→10, first sample={out:.4}");
        assert!(out < 8.0, "Slew should limit: got {out:.4}");
        assert!(out > 3.0, "Should allow some movement: got {out:.4}");
    }

    #[test]
    fn opamp_slew_tl072_faster_than_lm308() {
        // TL072 slew = 13 V/µs vs LM308 = 0.3 V/µs.
        // Same step, TL072 should get closer to target on first sample.
        let sr = 48000.0;
        let mut lm = OpAmpRoot::new_non_inverting(lm308(), 1.0);
        lm.set_sample_rate(sr);
        let mut tl = OpAmpRoot::new_non_inverting(tl072(), 1.0);
        tl.set_sample_rate(sr);

        for root in [&mut lm, &mut tl] {
            for _ in 0..100 {
                root.set_vp(0.0);
                drive(root, 0.0);
            }
        }

        lm.set_vp(10.0);
        let out_lm = drive(&mut lm, 0.0);
        tl.set_vp(10.0);
        let out_tl = drive(&mut tl, 0.0);

        eprintln!("Slew comparison: LM308={out_lm:.4}, TL072={out_tl:.4}");
        assert!(out_tl > out_lm, "TL072 should slew faster: tl={out_tl:.4} > lm={out_lm:.4}");
    }

    // ── Rail saturation ──────────────────────────────────────────────────

    #[test]
    fn opamp_rails_clamp_output() {
        // v_max = 3.0V (9V supply). Output should hard clip at ±3V.
        let mut model = tl072();
        model.v_rail_pos =3.0; model.v_rail_neg =3.0;
        // High slew rate so it doesn't interfere
        model.slew_rate = 100.0;
        let mut root = OpAmpRoot::new_non_inverting(model, 1.0);
        root.set_sample_rate(48000.0);

        // Warmup
        for _ in 0..100 {
            root.set_vp(0.0);
            drive(&mut root, 0.0);
        }

        // Drive way past rails
        root.set_vp(10.0);
        let out = drive(&mut root, 0.0);
        eprintln!("Rail test: input=10V, output={out:.4}");
        // OpAmpRoot does hard clamp, so should be exactly v_max
        assert!(out <= 3.01, "Should clamp at v_max: got {out:.4}");
        assert!(out > 2.5, "Should produce output near rails: got {out:.4}");
    }

    // ── Order of operations ──────────────────────────────────────────────

    #[test]
    fn opamp_gbw_before_slew() {
        // GBW rolloff smooths the signal BEFORE slew limiting.
        // This means a fast step gets GBW-filtered first (reducing dv),
        // so the slew limiter sees a smaller step.
        // With very low GBW and slow slew, the GBW filter dominates.
        let mut model = lm308();
        model.gbw = 100_000.0; // Very low GBW
        model.slew_rate = 0.1; // Very slow slew
        model.v_rail_pos =20.0; model.v_rail_neg =20.0; // High rails so they don't interfere
        let mut root = OpAmpRoot::new_non_inverting(model, 10.0);
        root.set_sample_rate(48000.0);

        for _ in 0..200 {
            root.set_vp(0.0);
            drive(&mut root, 0.0);
        }

        // Step to 5V — both GBW and slew should limit this
        root.set_vp(5.0);
        let out = drive(&mut root, 0.0);
        eprintln!("GBW+slew: step to 5V, out={out:.6}");
        // Output should be very small — GBW smooths first, then slew limits what's left
        assert!(out < 3.0, "Combined GBW+slew should heavily limit: got {out:.6}");
    }

    // ── Inverting gain ───────────────────────────────────────────────────

    #[test]
    fn opamp_inverting_gain_correct() {
        // Inverting with gain=10: v_out = -10 * v_in.
        // In WDF: v_in = a/2, so a=0.02 → v_in=0.01 → v_out=-0.1.
        let mut model = tl072();
        model.slew_rate = 100.0; // Fast slew
        model.v_rail_pos =10.0; model.v_rail_neg =10.0;
        let mut root = OpAmpRoot::new_inverting(model, 10.0);
        root.set_sample_rate(48000.0);

        // Warmup
        for _ in 0..200 {
            drive(&mut root, 0.0);
        }

        // Small signal: a = 2 * v_in = 0.02
        let out = drive(&mut root, 0.02);
        let gain = out / 0.01;
        eprintln!("Inverting: out={out:.6}, gain={gain:.2}");
        assert!(gain < -5.0, "Should invert with gain ~10: got {gain:.2}");
    }

    // ── Gain from port resistances (new approach) ────────────────────────
    //
    // Instead of FeedbackConfig with pre-computed Rf/Ri/series/parallel,
    // gain should be computed from the actual WDF port resistances at
    // runtime. These tests verify that set_gain() with Rf/Ri ratios
    // produces correct compute_vs_voltage() output.

    #[test]
    fn opamp_vs_voltage_tracks_gain_change() {
        // When gain changes via set_gain(), compute_vs_voltage() should
        // produce proportionally different output.
        let mut model = tl072();
        model.slew_rate = 100.0; // fast slew
        model.v_rail_pos =100.0; model.v_rail_neg =100.0; // high rails
        let mut root = OpAmpRoot::new_inverting(model, 10.0);
        root.set_sample_rate(48000.0);

        // Warmup at gain=10
        for _ in 0..100 { root.compute_vs_voltage(0.0); }
        let out_g10 = root.compute_vs_voltage(0.01).abs();

        // Change gain to 2
        root.set_gain(2.0);
        for _ in 0..100 { root.compute_vs_voltage(0.0); }
        let out_g2 = root.compute_vs_voltage(0.01).abs();

        eprintln!("VS voltage: g10={out_g10:.6}, g2={out_g2:.6}");
        assert!(out_g10 > out_g2 * 3.0,
            "Higher gain should give more VS voltage: g10={out_g10:.6}, g2={out_g2:.6}");
    }

    #[test]
    fn opamp_gain_from_rf_ri_inverting() {
        // set_gain(Rf/Ri) should produce correct gain for inverting topology.
        // Rf=100k, Ri=10k → gain=10.
        let mut model = tl072();
        model.slew_rate = 100.0;
        model.v_rail_pos =100.0; model.v_rail_neg =100.0;
        let rf = 100_000.0;
        let ri = 10_000.0;
        let gain = rf / ri; // = 10.0

        let mut root = OpAmpRoot::new_inverting(model, gain);
        root.set_sample_rate(48000.0);
        for _ in 0..100 { root.compute_vs_voltage(0.0); }

        let out = root.compute_vs_voltage(0.01).abs();
        let measured_gain = out / 0.01;
        eprintln!("Rf/Ri gain: expected=10, measured={measured_gain:.2}");
        assert!(measured_gain > 7.0 && measured_gain < 13.0,
            "Gain should be ~10: got {measured_gain:.2}");
    }

    #[test]
    fn opamp_gain_updates_when_pot_changes() {
        // Simulates what should happen when a pot moves:
        // 1. Compute new Rf from pot resistance
        // 2. Call set_gain(new_rf / ri)
        // 3. Output should change
        let mut model = tl072();
        model.slew_rate = 100.0;
        model.v_rail_pos =100.0; model.v_rail_neg =100.0;

        let ri = 10_000.0;
        let rf_lo = 10_000.0; // pot at 10% of 100k
        let rf_hi = 100_000.0; // pot at 100% of 100k

        let mut root = OpAmpRoot::new_inverting(model, rf_lo / ri);
        root.set_sample_rate(48000.0);

        // Measure at low gain
        for _ in 0..100 { root.compute_vs_voltage(0.0); }
        let out_lo = root.compute_vs_voltage(0.01).abs();

        // Update gain to high
        root.set_gain(rf_hi / ri);
        for _ in 0..100 { root.compute_vs_voltage(0.0); }
        let out_hi = root.compute_vs_voltage(0.01).abs();

        eprintln!("Pot change: lo={out_lo:.6} (g={:.1}), hi={out_hi:.6} (g={:.1})",
            rf_lo/ri, rf_hi/ri);
        assert!(out_hi > out_lo * 3.0,
            "Higher Rf should give more output: lo={out_lo:.6}, hi={out_hi:.6}");
    }

    #[test]
    fn opamp_gain_from_pot_plus_series_r() {
        // Pot + series R_clip: effective Rf = pot_r + R_clip.
        // This is what the stage should compute from port resistances.
        let mut model = tl072();
        model.slew_rate = 100.0;
        model.v_rail_pos =100.0; model.v_rail_neg =100.0;

        let ri = 10_000.0;
        let r_clip = 4_700.0;
        let pot_lo = 10_000.0;
        let pot_hi = 500_000.0;

        let rf_lo = pot_lo + r_clip; // 14.7k
        let rf_hi = pot_hi + r_clip; // 504.7k

        let mut root = OpAmpRoot::new_inverting(model, rf_lo / ri);
        root.set_sample_rate(48000.0);

        for _ in 0..100 { root.compute_vs_voltage(0.0); }
        let out_lo = root.compute_vs_voltage(0.01).abs();

        root.set_gain(rf_hi / ri);
        for _ in 0..100 { root.compute_vs_voltage(0.0); }
        let out_hi = root.compute_vs_voltage(0.01).abs();

        eprintln!("Pot+series: lo={out_lo:.6} (Rf={rf_lo:.0}), hi={out_hi:.6} (Rf={rf_hi:.0})");
        assert!(out_hi > out_lo * 3.0,
            "Higher pot should give more output: lo={out_lo:.6}, hi={out_hi:.6}");
    }
}
