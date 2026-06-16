//! OTA (Operational Transconductance Amplifier) WDF root elements.
//!
//! Models the CA3080 and similar OTAs with tanh transfer characteristic.

use super::solver::{newton_raphson_solve, NlDeviceGroupIv, LEAKAGE_CONDUCTANCE};
use crate::elements::WdfRoot;

// ---------------------------------------------------------------------------
// OTA (Operational Transconductance Amplifier) Model
// ---------------------------------------------------------------------------

/// CA3080 OTA model parameters.
///
/// Unlike voltage-mode op-amps, an OTA's output is a current proportional
/// to the differential input voltage, controlled by a bias current (Iabc).
///
/// The key equation: `Iout = gm * (V+ - V-)` where `gm = Iabc / (2 * Vt)`
/// and Vt is the thermal voltage (~26mV at room temperature).
///
/// The CA3080 has a tanh-like transfer characteristic:
/// `Iout = Iabc * tanh(Vdiff / (2 * Vt))`
///
/// This means:
/// - For small signals: linear gain proportional to Iabc
/// - For large signals: soft clipping at ±Iabc
/// - The gain is controlled by current, not voltage — perfect for VCA/compressor
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OtaModel {
    /// Maximum bias current (A). CA3080 typical: 0.5mA.
    pub iabc_max: crate::Wave,
    /// Thermal voltage (V). ~25.85mV at 20°C.
    pub vt: crate::Wave,
    /// Output load resistance (Ω). Determines voltage gain from current output.
    pub r_load: crate::Wave,
}

impl OtaModel {
    /// CA3080 with typical bias point.
    ///
    /// The CA3080 is the classic OTA used in:
    /// - MXR Dyna Comp (compressor)
    /// - Ross Compressor
    /// - EHX Doctor Q (envelope filter)
    /// - Boss CE-1 (chorus VCA section)
    pub fn ca3080() -> Self {
        Self {
            iabc_max: 0.5e-3, // 500µA max amplifier bias current
            vt: 25.85e-3,     // Thermal voltage at 20°C
            r_load: 10_000.0, // Typical 10k load resistor
        }
    }
}

/// OTA nonlinear root for WDF trees.
///
/// Models the transconductance amplifier as a current-output device.
/// The output current follows a tanh transfer characteristic:
/// `Iout = Iabc * tanh(Vin / (2*Vt))`
///
/// The bias current Iabc controls the gain — this is the compression
/// mechanism in OTA compressors. An envelope follower generates a
/// control signal that reduces Iabc as the input gets louder, creating
/// automatic gain reduction.
///
/// For the WDF constraint, the OTA output current flows through the
/// load resistor, creating the voltage that the rest of the tree sees.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OtaRoot {
    pub model: OtaModel,
    /// Current amplifier bias current (A). Controls gain.
    /// In a compressor, this is modulated by the envelope follower.
    iabc: crate::Wave,
    /// Maximum Newton-Raphson iterations (bounded for RT safety).
    max_iter: usize,
}

impl OtaRoot {
    pub fn new(model: OtaModel) -> Self {
        Self {
            iabc: model.iabc_max,
            model,
            max_iter: super::solver::NR_MAX_ITER,
        }
    }

    /// Set the amplifier bias current (external control from envelope, LFO, etc.).
    ///
    /// In a compressor circuit, higher envelope = lower Iabc = less gain.
    /// Range: 0 to iabc_max.
    #[inline]
    pub fn set_iabc(&mut self, iabc: crate::Wave) {
        self.iabc = iabc.clamp(0.0, self.model.iabc_max);
    }

    /// Get current amplifier bias current.
    #[inline]
    pub fn iabc(&self) -> crate::Wave {
        self.iabc
    }

    /// Set gain as a normalized value (0.0 = off, 1.0 = max gain).
    ///
    /// This is a convenience wrapper for compressor-style control:
    /// the envelope follower output (0-1) directly maps to gain reduction.
    #[inline]
    pub fn set_gain_normalized(&mut self, gain: crate::Wave) {
        self.iabc = gain.clamp(0.0, 1.0) * self.model.iabc_max;
    }

    /// Compute the OTA output current for a given input voltage.
    ///
    /// Uses the tanh transfer characteristic of the differential pair:
    /// `Iout = Iabc * tanh(Vin / (2*Vt))`
    #[inline]
    pub fn output_current(&self, v_in: crate::Wave) -> crate::Wave {
        let x = v_in / (2.0 * self.model.vt);
        self.iabc * crate::math::tanh(x)
    }

    /// Derivative of output current w.r.t. input voltage.
    ///
    /// `dIout/dVin = Iabc / (2*Vt) * sech²(Vin / (2*Vt))`
    ///            = gm * sech²(Vin / (2*Vt))
    #[inline]
    fn output_current_derivative(&self, v_in: crate::Wave) -> crate::Wave {
        let x = v_in / (2.0 * self.model.vt);
        let sech = 1.0 / crate::math::cosh(x);
        self.iabc / (2.0 * self.model.vt) * sech * sech
    }

    /// Compute the transconductance (gm) at the current bias point.
    ///
    /// gm = Iabc / (2 * Vt) — this is the small-signal gain.
    /// For CA3080 at Iabc=500µA: gm ≈ 9.7 mA/V (≈ 9.7 mS)
    pub fn transconductance(&self) -> crate::Wave {
        self.iabc / (2.0 * self.model.vt)
    }
}

impl OtaRoot {
    /// Check if the OTA is operating outside its valid region.
    ///
    /// Checks:
    /// - Entering compression: |Vdiff| > 20mV
    /// - Hard saturation: |Vdiff| > 40mV
    /// - Effectively off: Iabc < 1µA
    /// - Overbiased: Iabc > 1mA
    #[cfg(feature = "runtime-warnings")]
    pub fn check_operating_region(&self, v_port: crate::Wave, comp_id: &str, sample_index: u64) {
        use crate::runtime_warnings::{emit_warning, Severity, WarningKind};

        let vdiff = v_port.abs();

        // |Vdiff| > 40mV → hard saturation (check first, more severe)
        const HARD_SAT_THRESHOLD: crate::Wave = 40e-3;
        if vdiff > HARD_SAT_THRESHOLD {
            emit_warning(
                sample_index,
                comp_id,
                WarningKind::OtaHardSaturation,
                Severity::Danger,
                vdiff,
                HARD_SAT_THRESHOLD,
                "OTA |Vdiff| exceeds 40mV (hard saturation)",
            );
        }
        // |Vdiff| > 20mV → entering compression
        else {
            const COMPRESSION_THRESHOLD: crate::Wave = 20e-3;
            if vdiff > COMPRESSION_THRESHOLD {
                emit_warning(
                    sample_index,
                    comp_id,
                    WarningKind::OtaEnteringCompression,
                    Severity::Caution,
                    vdiff,
                    COMPRESSION_THRESHOLD,
                    "OTA |Vdiff| exceeds 20mV (entering compression)",
                );
            }
        }

        // Iabc < 1µA → effectively off
        const OFF_THRESHOLD: crate::Wave = 1e-6;
        if self.iabc < OFF_THRESHOLD {
            emit_warning(
                sample_index,
                comp_id,
                WarningKind::OtaEffectivelyOff,
                Severity::Caution,
                self.iabc,
                OFF_THRESHOLD,
                "OTA Iabc below 1µA (effectively off)",
            );
        }

        // Iabc > 1mA → overbiased
        const OVERBIAS_THRESHOLD: crate::Wave = 1e-3;
        if self.iabc > OVERBIAS_THRESHOLD {
            emit_warning(
                sample_index,
                comp_id,
                WarningKind::OtaOverbiased,
                Severity::Caution,
                self.iabc,
                OVERBIAS_THRESHOLD,
                "OTA Iabc exceeds 1mA (overbiased)",
            );
        }
    }
}

impl WdfRoot for OtaRoot {
    /// OTA transconductance: `i = Iabc * tanh(v / (2·Vt))`
    #[inline]
    fn process(&mut self, a: crate::Wave, rp: crate::Wave) -> crate::Wave {
        let root = *self;
        let gm = self.transconductance();
        let v0 = if gm > LEAKAGE_CONDUCTANCE {
            a / (2.0 + 2.0 * rp * gm)
        } else {
            a * 0.5
        };
        newton_raphson_solve(a, rp, v0, self.max_iter, 1e-6, None, None, |v| {
            (root.output_current(v), root.output_current_derivative(v))
        })
    }
}

impl super::solver::NlDeviceIv for OtaRoot {
    /// OTA I-V characteristic for multi-NL MNA solver.
    ///
    /// Returns `(Iout, dIout/dVdiff)` where:
    /// - `Iout = Iabc * tanh(v / (2*Vt))`
    /// - `dIout/dVdiff = Iabc / (2*Vt) * sech²(v / (2*Vt))`
    #[inline]
    fn iv(&self, v: f64) -> (f64, f64) {
        (self.output_current(v), self.output_current_derivative(v))
    }

    /// OTA differential input clamp: ±200mV (well beyond hard saturation at ~40mV).
    #[inline]
    fn v_clamp(&self) -> (f64, f64) {
        (-0.2, 0.2)
    }
}

// ---------------------------------------------------------------------------
// OTA Two-Port grouped solver (for multi-port NR / MultiNlStage)
// ---------------------------------------------------------------------------

/// Two-port OTA model for the grouped Newton-Raphson solver.
///
/// Models the OTA as a transconductance amplifier with two WDF ports:
/// - Port 0: (pos, neg) — differential input — high impedance, near-zero input current
/// - Port 1: (out, gnd) — current output — Iout = Iabc·tanh(V_pn / (2·Vt))
///
/// The 2×2 Jacobian captures the cross-transconductance:
/// ```text
///   ∂I₀/∂V₀ = ε  (near-zero input conductance, leakage for NR stability)
///   ∂I₀/∂V₁ = 0
///   ∂I₁/∂V₀ = gm·sech²(V₀ / (2·Vt))   (transconductance)
///   ∂I₁/∂V₁ = ε  (near-zero output conductance, leakage for NR stability)
/// ```
/// where V₀ = V_pos − V_neg, V₁ = V_out − V_gnd.
///
/// The leakage conductance ε prevents a zero-diagonal Jacobian which would
/// make the linear system singular.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OtaTwoPort {
    /// Underlying OTA model parameters.
    pub model: OtaModel,
    /// Current amplifier bias current (A). Controls gain.
    pub iabc: crate::Wave,
}

impl OtaTwoPort {
    /// Create an OtaTwoPort from a model, initialising iabc to iabc_max.
    pub fn new(model: OtaModel) -> Self {
        Self {
            iabc: model.iabc_max,
            model,
        }
    }

    /// Set the amplifier bias current (external control).
    #[inline]
    pub fn set_iabc(&mut self, iabc: crate::Wave) {
        self.iabc = iabc.clamp(0.0, self.model.iabc_max);
    }

    /// Compute the small-signal transconductance: gm = Iabc / (2·Vt).
    #[inline]
    pub fn transconductance(&self) -> crate::Wave {
        self.iabc / (2.0 * self.model.vt)
    }

    /// OTA output current for a given differential input voltage.
    #[inline]
    fn iout(&self, v_diff: crate::Wave) -> crate::Wave {
        let x = v_diff / (2.0 * self.model.vt);
        self.iabc * crate::math::tanh(x)
    }

    /// Derivative of output current w.r.t. differential input voltage.
    ///
    /// `dIout/dVdiff = Iabc / (2·Vt) · sech²(Vdiff / (2·Vt))`
    #[inline]
    fn diout_dvdiff(&self, v_diff: crate::Wave) -> crate::Wave {
        let x = v_diff / (2.0 * self.model.vt);
        let sech = 1.0 / crate::math::cosh(x);
        self.iabc / (2.0 * self.model.vt) * sech * sech
    }
}

impl NlDeviceGroupIv for OtaTwoPort {
    fn n_ports(&self) -> usize {
        2
    }

    /// Evaluate OTA two-port currents and 2×2 Jacobian.
    ///
    /// Port layout (matches nl_terminals in classify_nl_devices):
    /// - v[0] = V_pos − V_neg   (differential input)
    /// - v[1] = V_out − V_gnd   (output voltage, used only for Rload effects)
    ///
    /// Currents:
    /// - I[0] ≈ 0  (near-ideal high-impedance input)
    /// - I[1] = Iabc·tanh(v[0] / (2·Vt))  (transconductance output)
    ///
    /// Jacobian (row-major 2×2):
    /// ```text
    ///   [ε,    0  ]   (∂I₀/∂V₀,  ∂I₀/∂V₁)
    ///   [gm·s², ε ]   (∂I₁/∂V₀,  ∂I₁/∂V₁)
    /// ```
    fn eval(&self, v: &[crate::Wave], currents: &mut [crate::Wave], jacobian: &mut [crate::Wave]) {
        let v_diff = v[0]; // V_pos - V_neg

        // Input port: near-zero current (ideal high-Z input)
        currents[0] = LEAKAGE_CONDUCTANCE * v_diff;

        // Output port: tanh transconductance
        currents[1] = self.iout(v_diff);

        // 2×2 Jacobian (row-major: jac[row*2 + col])
        // Row 0 (input port): ∂I₀/∂V₀ = ε, ∂I₀/∂V₁ = 0
        jacobian[0] = LEAKAGE_CONDUCTANCE; // ∂I₀/∂V₀
        jacobian[1] = 0.0; // ∂I₀/∂V₁

        // Row 1 (output port): ∂I₁/∂V₀ = gm·sech², ∂I₁/∂V₁ = ε
        jacobian[2] = self.diout_dvdiff(v_diff); // ∂I₁/∂V₀ (transconductance)
        jacobian[3] = LEAKAGE_CONDUCTANCE; // ∂I₁/∂V₁
    }

    /// Voltage clamps per port.
    fn v_clamp_port(&self, port: usize) -> (crate::Wave, crate::Wave) {
        match port {
            0 => (-0.2, 0.2), // Differential input: ±200mV
            _ => (-15.0, 15.0), // Output: ±15V (supply rail)
        }
    }
}
