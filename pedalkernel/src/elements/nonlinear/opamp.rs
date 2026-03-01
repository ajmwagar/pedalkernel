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
/// Models op-amp behavior in two topologies:
/// - **Inverting**: Vout = -(Rf/Ri) * Vin, input from WDF wave (virtual ground)
/// - **Non-inverting**: Vout = (1 + Rf/Ri) * Vp, input via `set_vp()`
///
/// Unity-gain buffer (voltage follower) is just NonInverting with gain=1.
/// The "third mode" doesn't exist - it's the same circuit with Rf=0, Ri=∞.
///
/// Non-idealities:
/// - Slew rate limiting (LM308's slow slew = RAT's character)
/// - Output saturation at supply rails
/// - Soft clipping via feedback diodes (optional)
#[derive(Debug, Clone, Copy)]
pub struct OpAmpRoot {
    pub model: OpAmpModel,
    /// Operating mode (inverting/non-inverting).
    mode: OpAmpMode,
    /// Non-inverting input voltage (for NonInverting mode).
    vp: f64,
    /// Previous output voltage (for slew rate limiting).
    prev_out: f64,
    /// Sample rate (needed for slew rate limiting).
    sample_rate: f64,
    /// Soft clipping limit from feedback diodes.
    /// If set, uses tanh-based soft clipping instead of hard clipping.
    soft_clip_v: Option<f64>,
}

impl OpAmpRoot {
    /// Create a unity-gain buffer (voltage follower).
    ///
    /// This is a non-inverting amplifier with gain=1 (Rf=0, Ri=∞).
    /// Input via `set_vp()`, output tracks Vp exactly.
    pub fn new(model: OpAmpModel) -> Self {
        Self {
            model,
            mode: OpAmpMode::NonInverting { gain: 1.0 },
            vp: 0.0,
            prev_out: 0.0,
            sample_rate: 48000.0,
            soft_clip_v: None,
        }
    }

    /// Create a unity-gain buffer (alias for `new`).
    pub fn unity_gain(model: OpAmpModel) -> Self {
        Self::new(model)
    }

    /// Create an inverting op-amp: Vout = -gain * Vin.
    pub fn new_inverting(model: OpAmpModel, gain: f64) -> Self {
        Self {
            model,
            mode: OpAmpMode::Inverting { gain: gain.abs() },
            vp: 0.0,
            prev_out: 0.0,
            sample_rate: 48000.0,
            soft_clip_v: None,
        }
    }

    /// Create a non-inverting op-amp: Vout = gain * Vin.
    pub fn new_non_inverting(model: OpAmpModel, gain: f64) -> Self {
        Self {
            model,
            mode: OpAmpMode::NonInverting { gain: gain.abs() },
            vp: 0.0,
            prev_out: 0.0,
            sample_rate: 48000.0,
            soft_clip_v: None,
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

    /// Set the closed-loop gain.
    #[inline]
    pub fn set_gain(&mut self, gain: f64) {
        match &mut self.mode {
            OpAmpMode::Inverting { gain: g } => *g = gain.abs(),
            OpAmpMode::NonInverting { gain: g } => *g = gain.abs(),
        }
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

    /// Set the sample rate (for slew rate limiting).
    #[inline]
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
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

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.prev_out = 0.0;
        self.vp = 0.0;
    }
}

impl WdfRoot for OpAmpRoot {
    /// Op-amp processing based on mode.
    #[inline]
    fn process(&mut self, a: f64, _rp: f64) -> f64 {
        let v_max = self.model.v_max;

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

