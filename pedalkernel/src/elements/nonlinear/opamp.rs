//! Op-amp (voltage-controlled voltage source) WDF root elements.
//!
//! Models common op-amps (TL072, LM308, JRC4558) as VCVS with feedback.

use super::solver::newton_raphson_solve;
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
/// - Gain-bandwidth product (for future frequency-dependent modeling)
#[derive(Debug, Clone, Copy)]
pub struct OpAmpModel {
    /// Open-loop DC gain. TL072 ≈ 200k (106dB), LM308 ≈ 300k.
    pub open_loop_gain: f64,
    /// Gain-bandwidth product (Hz). TL072 ≈ 3MHz, LM308 ≈ 1MHz.
    /// Used for frequency-dependent modeling (future).
    pub gbw: f64,
    /// Slew rate (V/µs). TL072 ≈ 13, LM308 ≈ 0.3.
    pub slew_rate: f64,
    /// Maximum output voltage swing (V, single-sided from Vcc/2).
    /// Typically Vcc/2 - 1.5V for rail-to-rail, less for others.
    pub v_max: f64,
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
            gbw: 3e6,                   // 3 MHz
            slew_rate: 13.0,            // 13 V/µs
            v_max: 12.0,                // ±12V swing at ±15V supply
        }
    }

    /// TL082 — JFET-input dual op-amp (TL072 variant).
    ///
    /// Similar to TL072 but slightly different specs. Used interchangeably
    /// in many designs.
    pub fn tl082() -> Self {
        Self {
            open_loop_gain: 200_000.0,
            gbw: 4e6,                   // 4 MHz (slightly faster)
            slew_rate: 13.0,
            v_max: 12.0,
        }
    }

    /// LM308 — The RAT's secret weapon.
    ///
    /// Very slow slew rate (0.3 V/µs) creates the distinctive compression
    /// and "sag" character of the ProCo RAT. The slow slew rate rounds off
    /// transients, creating a smoother, more compressed distortion.
    pub fn lm308() -> Self {
        Self {
            open_loop_gain: 300_000.0, // 110 dB
            gbw: 1e6,                   // 1 MHz
            slew_rate: 0.3,             // 0.3 V/µs — THE key to RAT tone
            v_max: 12.0,
        }
    }

    /// LM741 — The classic vintage op-amp.
    ///
    /// Slow and noisy by modern standards, but has a characteristic sound.
    /// Used in early Big Muff Pi and some vintage designs.
    pub fn lm741() -> Self {
        Self {
            open_loop_gain: 200_000.0,
            gbw: 1e6,
            slew_rate: 0.5,             // Slow
            v_max: 12.0,
        }
    }

    /// JRC4558 — The Tube Screamer's heart.
    ///
    /// Medium slew rate (1.7 V/µs) contributes to the Tube Screamer's
    /// warm, slightly compressed midrange. The JRC (Japan Radio Company)
    /// 4558D is the legendary variant.
    pub fn jrc4558() -> Self {
        Self {
            open_loop_gain: 100_000.0, // 100 dB
            gbw: 3e6,                   // 3 MHz
            slew_rate: 1.7,             // Moderate
            v_max: 12.0,
        }
    }

    /// RC4558 — Common 4558 variant.
    ///
    /// Texas Instruments version of the 4558, used in many TS clones.
    /// Very similar to JRC4558 but some players hear subtle differences.
    pub fn rc4558() -> Self {
        Self {
            open_loop_gain: 100_000.0,
            gbw: 3e6,
            slew_rate: 1.5,
            v_max: 12.0,
        }
    }

    /// NE5532 — Studio-grade low-noise op-amp.
    ///
    /// Very fast, very clean. Used in high-end effects and studio gear.
    /// The "transparent" choice when you don't want op-amp coloration.
    pub fn ne5532() -> Self {
        Self {
            open_loop_gain: 100_000.0,
            gbw: 10e6,                  // 10 MHz
            slew_rate: 9.0,             // Fast
            v_max: 12.0,
        }
    }

    /// OP07 — Precision op-amp.
    ///
    /// Very low offset voltage and drift. Used in some boutique designs
    /// where DC accuracy matters.
    pub fn op07() -> Self {
        Self {
            open_loop_gain: 400_000.0, // 112 dB
            gbw: 0.6e6,                 // 600 kHz
            slew_rate: 0.3,             // Slow
            v_max: 12.0,
        }
    }

    /// Generic op-amp with typical values.
    ///
    /// Use when the specific op-amp type doesn't matter or isn't specified.
    pub fn generic() -> Self {
        Self {
            open_loop_gain: 100_000.0,
            gbw: 1e6,
            slew_rate: 1.0,
            v_max: 12.0,
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
/// Set `vp` to the signal at the positive input each sample, and the
/// root will produce the buffered output.
///
/// **Feedback configurations:**
/// - Unity-gain (voltage follower): `feedback_ratio = 1.0`, output follows Vp
/// - Inverting amp: `feedback_ratio = Rf/(Rf+Rin)`, Vm depends on output
/// - Non-inverting amp: similar, but Vp receives the signal
///
/// The feedback network components (resistors, capacitors) remain in the
/// WDF tree — this root just enforces the op-amp gain relationship.
#[derive(Debug, Clone, Copy)]
pub struct OpAmpRoot {
    pub model: OpAmpModel,
    /// Non-inverting input voltage (set externally each sample).
    vp: f64,
    /// Feedback ratio: 0.0 = no feedback, 1.0 = unity-gain buffer.
    /// Vm = vm_external * (1 - fb_ratio) + v_out * fb_ratio
    feedback_ratio: f64,
    /// External voltage at inverting input (for non-feedback path).
    vm_external: f64,
    /// Previous output voltage (for slew rate limiting).
    prev_out: f64,
    /// Sample rate (needed for slew rate limiting).
    sample_rate: f64,
    /// Maximum Newton-Raphson iterations (bounded for RT safety).
    max_iter: usize,
    /// Convergence tolerance.
    tolerance: f64,
}

impl OpAmpRoot {
    /// Create a new op-amp root with the given model.
    ///
    /// Default configuration: unity-gain buffer (feedback_ratio = 1.0).
    pub fn new(model: OpAmpModel) -> Self {
        Self {
            model,
            vp: 0.0,
            feedback_ratio: 1.0, // Unity-gain buffer by default
            vm_external: 0.0,
            prev_out: 0.0,
            sample_rate: 48000.0,
            max_iter: 16,
            tolerance: 1e-9,
        }
    }

    /// Create a unity-gain buffer (voltage follower).
    ///
    /// This is the most common op-amp configuration in guitar pedals.
    /// The output directly follows the non-inverting input.
    pub fn unity_gain(model: OpAmpModel) -> Self {
        let mut root = Self::new(model);
        root.feedback_ratio = 1.0;
        root
    }

    /// Set the non-inverting input voltage.
    ///
    /// Call this each sample before `process()` to set the signal
    /// that the op-amp is buffering/amplifying.
    #[inline]
    pub fn set_vp(&mut self, vp: f64) {
        self.vp = vp;
    }

    /// Get the current non-inverting input voltage.
    #[inline]
    pub fn vp(&self) -> f64 {
        self.vp
    }

    /// Configure the feedback topology.
    ///
    /// - `ratio = 1.0`: Unity-gain buffer (Vm = Vout)
    /// - `ratio = 0.5`: 2x non-inverting gain
    /// - `ratio = R2/(R1+R2)`: General non-inverting amp
    ///
    /// For inverting configurations, set `vm_external` to the input signal
    /// and `ratio` to the feedback divider ratio.
    #[inline]
    pub fn set_feedback(&mut self, ratio: f64, vm_external: f64) {
        self.feedback_ratio = ratio.clamp(0.0, 1.0);
        self.vm_external = vm_external;
    }

    /// Set the sample rate (for slew rate limiting).
    #[inline]
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
    }

    /// Apply slew rate limiting to the output.
    ///
    /// Real op-amps have a maximum rate of voltage change (dV/dt).
    /// When the output tries to change faster than this, it "slews"
    /// at the maximum rate, creating the compression and warmth
    /// characteristic of slow op-amps like the LM308.
    #[inline]
    fn apply_slew_limit(&mut self, v: f64) -> f64 {
        // Maximum voltage change per sample: slew_rate * (1e6 / sample_rate)
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

    /// Reset internal state (call on audio restart).
    pub fn reset(&mut self) {
        self.prev_out = 0.0;
        self.vp = 0.0;
        self.vm_external = 0.0;
    }

    /// Set the maximum output voltage (determined by supply rails).
    ///
    /// For single-supply pedals at Vsupply biased at Vsupply/2:
    /// v_max ≈ (Vsupply/2) - saturation_margin
    ///
    /// For ±15V supplies (lab standard): v_max ≈ ±12V
    /// For 9V single-supply biased at 4.5V: v_max ≈ ±3.5V
    #[inline]
    pub fn set_v_max(&mut self, v_max: f64) {
        self.model.v_max = v_max.max(0.5); // Minimum 0.5V to avoid degeneracy
    }

    /// Get the current v_max setting.
    #[inline]
    pub fn v_max(&self) -> f64 {
        self.model.v_max
    }
}

impl WdfRoot for OpAmpRoot {
    /// Op-amp VCVS: `Vout = Aol * (Vp - Vm)`, with soft saturation and slew limiting.
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
        let aol = self.model.open_loop_gain;
        let v_max = self.model.v_max;
        let vp = self.vp;
        let feedback_ratio = self.feedback_ratio;
        let vm_external = self.vm_external;

        let v0 = if feedback_ratio > 0.5 {
            vp.clamp(-v_max, v_max)
        } else {
            (0.5 * a).clamp(-v_max, v_max)
        };

        let b = newton_raphson_solve(
            a, rp, v0, self.max_iter, self.tolerance,
            Some((-v_max * 1.5, v_max * 1.5)), Some(v_max),
            |v| {
                // Compute Vm as a function of output voltage
                let vm = vm_external * (1.0 - feedback_ratio) + v * feedback_ratio;
                let delta = vp - vm;
                let ddelta_dv = -feedback_ratio;

                // Soft saturation: tanh-based limiting preserves gradient
                let raw_ideal = aol * delta;
                let v_ideal = v_max * (raw_ideal / v_max).tanh();
                let t = (raw_ideal / v_max).tanh();
                let dv_ideal_ddelta = aol * (1.0 - t * t);

                // Effective output resistance
                let r_out_eff = (rp / (aol * feedback_ratio.max(0.001))).max(0.01);

                let i_out = (v_ideal - v) / r_out_eff;
                let di_dv = (dv_ideal_ddelta * ddelta_dv - 1.0) / r_out_eff;
                (i_out, di_dv)
            },
        );

        // Recover voltage from reflected wave, apply post-NR processing
        let mut v = (a + b) / 2.0;

        // Final hard clip at rails (after convergence)
        v = v.clamp(-v_max, v_max);

        // Apply slew rate limiting
        v = self.apply_slew_limit(v);

        2.0 * v - a
    }
}

// ---------------------------------------------------------------------------
// Inverting Op-Amp Gain Stage
// ---------------------------------------------------------------------------

/// Inverting op-amp amplifier for WDF trees.
///
/// Models the closed-loop behavior of an inverting amplifier:
/// `Vout = -(Rf/Ri) * Vin`
///
/// The input signal enters through the WDF tree (representing Ri),
/// and the output is the amplified, inverted signal.
///
/// Key characteristics:
/// - Virtual ground at the inverting input (Vm ≈ 0V)
/// - Input impedance = Ri (the resistance seen from the WDF tree)
/// - Closed-loop gain = Rf/Ri
/// - Phase inversion (180°)
///
/// Non-idealities:
/// - Slew rate limiting (LM308's slow slew = RAT's character)
/// - Output saturation at supply rails
#[derive(Debug, Clone, Copy)]
pub struct OpAmpInvertingRoot {
    pub model: OpAmpModel,
    /// Closed-loop gain magnitude |Rf/Ri|.
    gain: f64,
    /// Previous output voltage (for slew rate limiting).
    prev_out: f64,
    /// Sample rate (needed for slew rate limiting).
    sample_rate: f64,
    /// Soft clipping limit from feedback diodes.
    /// If set, uses tanh-based soft clipping instead of hard clipping.
    /// Value is the diode forward voltage (~0.6V silicon, ~0.3V germanium).
    soft_clip_v: Option<f64>,
}

impl OpAmpInvertingRoot {
    /// Create an inverting op-amp with the given gain (Rf/Ri).
    pub fn new(model: OpAmpModel, gain: f64) -> Self {
        Self {
            model,
            gain: gain.abs(), // Store magnitude, sign handled in process
            prev_out: 0.0,
            sample_rate: 48000.0,
            soft_clip_v: None,
        }
    }

    /// Enable soft clipping mode for feedback diodes.
    ///
    /// When diodes are present in the op-amp feedback loop (like Tube Screamer),
    /// they create soft clipping by limiting the feedback voltage. This uses
    /// tanh-based soft limiting instead of hard clipping at the supply rails.
    ///
    /// `diode_vf` is the diode forward voltage:
    /// - Silicon (1N4148, 1N914): ~0.6V
    /// - Germanium (1N34A): ~0.3V
    #[inline]
    pub fn set_soft_clip(&mut self, diode_vf: f64) {
        self.soft_clip_v = Some(diode_vf.max(0.1));
    }

    /// Disable soft clipping (revert to hard rail clipping).
    #[inline]
    pub fn clear_soft_clip(&mut self) {
        self.soft_clip_v = None;
    }

    /// Set the closed-loop gain (Rf/Ri).
    #[inline]
    pub fn set_gain(&mut self, gain: f64) {
        self.gain = gain.abs();
    }

    /// Get the current gain.
    #[inline]
    pub fn gain(&self) -> f64 {
        self.gain
    }

    /// Set the sample rate (for slew rate limiting).
    #[inline]
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
    }

    /// Set the maximum output voltage (determined by supply rails).
    #[inline]
    pub fn set_v_max(&mut self, v_max: f64) {
        self.model.v_max = v_max.max(0.5);
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

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.prev_out = 0.0;
    }
}

impl WdfRoot for OpAmpInvertingRoot {
    /// Inverting amplifier: Vout = -gain * Vin
    ///
    /// For an inverting amplifier, the WDF incident wave `a` comes from
    /// a voltage source representing the input signal. The op-amp root
    /// acts as a voltage-controlled voltage source, driving its output
    /// back to the tree as the reflected wave.
    ///
    /// Wave variable convention:
    /// - Input voltage: V_in = (a + b) / 2, but for source-terminated: V_in ≈ a/2
    /// - For voltage source output: b = 2*V_out - a
    #[inline]
    fn process(&mut self, a: f64, _rp: f64) -> f64 {
        // For a tree with voltage source, the incident wave `a` encodes
        // the source voltage: a ≈ 2*V_source at the root.
        // Extract input voltage from the wave.
        let v_in = a / 2.0;

        // Apply closed-loop gain (with inversion)
        let mut v_out = -self.gain * v_in;

        // Apply clipping:
        // 1. First apply soft clipping if feedback diodes present
        // 2. Then hard clip at supply rails (always applies)
        if let Some(vd) = self.soft_clip_v {
            // Soft clipping via feedback diodes (Tube Screamer style)
            // Uses tanh to create smooth knee at diode forward voltage.
            // Formula: Vout = Vd * tanh(Vout_ideal / Vd)
            // This models the diode shunting current when voltage exceeds Vf.
            v_out = vd * (v_out / vd).tanh();
        }
        // Always hard clip at supply rails - op-amps can't exceed supply
        let v_max = self.model.v_max;
        v_out = v_out.clamp(-v_max, v_max);

        // Apply slew rate limiting - this is the key non-ideality
        // that gives character to slow op-amps like LM308.
        v_out = self.apply_slew_limit(v_out);

        // Reflect as voltage source: b = 2*V_out - a
        2.0 * v_out - a
    }
}

// ---------------------------------------------------------------------------
// Non-Inverting Op-Amp Gain Stage
// ---------------------------------------------------------------------------

/// Non-inverting op-amp amplifier for WDF trees.
///
/// Models the closed-loop behavior of a non-inverting amplifier:
/// `Vout = (1 + Rf/Ri) * Vin`
///
/// The input signal is set externally via `set_vp()`, representing the
/// signal at the non-inverting input. The output is the amplified signal
/// with no phase inversion.
///
/// Key characteristics:
/// - High input impedance (signal goes directly to Vp)
/// - Closed-loop gain = 1 + Rf/Ri
/// - No phase inversion
///
/// Non-idealities:
/// - Slew rate limiting
/// - Output saturation at supply rails
#[derive(Debug, Clone, Copy)]
pub struct OpAmpNonInvertingRoot {
    pub model: OpAmpModel,
    /// Closed-loop gain (1 + Rf/Ri).
    gain: f64,
    /// Non-inverting input voltage (set externally each sample).
    vp: f64,
    /// Previous output voltage (for slew rate limiting).
    prev_out: f64,
    /// Sample rate (needed for slew rate limiting).
    sample_rate: f64,
}

impl OpAmpNonInvertingRoot {
    /// Create a non-inverting op-amp with the given gain (1 + Rf/Ri).
    pub fn new(model: OpAmpModel, gain: f64) -> Self {
        Self {
            model,
            gain: gain.max(1.0), // Minimum gain is 1 (unity buffer)
            vp: 0.0,
            prev_out: 0.0,
            sample_rate: 48000.0,
        }
    }

    /// Set the closed-loop gain (1 + Rf/Ri).
    #[inline]
    pub fn set_gain(&mut self, gain: f64) {
        self.gain = gain.max(1.0);
    }

    /// Get the current gain.
    #[inline]
    pub fn gain(&self) -> f64 {
        self.gain
    }

    /// Set the non-inverting input voltage (the signal to amplify).
    #[inline]
    pub fn set_vp(&mut self, vp: f64) {
        self.vp = vp;
    }

    /// Get the current non-inverting input voltage.
    #[inline]
    pub fn vp(&self) -> f64 {
        self.vp
    }

    /// Set the sample rate (for slew rate limiting).
    #[inline]
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
    }

    /// Set the maximum output voltage (determined by supply rails).
    #[inline]
    pub fn set_v_max(&mut self, v_max: f64) {
        self.model.v_max = v_max.max(0.5);
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

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.prev_out = 0.0;
        self.vp = 0.0;
    }
}

impl WdfRoot for OpAmpNonInvertingRoot {
    /// Non-inverting amplifier: Vout = gain * Vp
    ///
    /// The input signal is set via `set_vp()` before calling this.
    /// The WDF tree represents the load, and we drive it with the
    /// amplified signal.
    #[inline]
    fn process(&mut self, a: f64, _rp: f64) -> f64 {
        // Apply closed-loop gain to the input
        let mut v_out = self.gain * self.vp;

        // Hard clip at rails - this is how real op-amps behave.
        let v_max = self.model.v_max;
        v_out = v_out.clamp(-v_max, v_max);

        // Apply slew rate limiting
        v_out = self.apply_slew_limit(v_out);

        // Convert to wave domain: for voltage source, b = 2*V - a
        2.0 * v_out - a
    }
}
