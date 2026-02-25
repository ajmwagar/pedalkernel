//! OTA (Operational Transconductance Amplifier) WDF root elements.
//!
//! Models the CA3080 and similar OTAs with tanh transfer characteristic.

use super::solver::{newton_raphson_solve, LEAKAGE_CONDUCTANCE};
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
pub struct OtaModel {
    /// Maximum bias current (A). CA3080 typical: 0.5mA.
    pub iabc_max: f64,
    /// Thermal voltage (V). ~25.85mV at 20°C.
    pub vt: f64,
    /// Output load resistance (Ω). Determines voltage gain from current output.
    pub r_load: f64,
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
pub struct OtaRoot {
    pub model: OtaModel,
    /// Current amplifier bias current (A). Controls gain.
    /// In a compressor, this is modulated by the envelope follower.
    iabc: f64,
    /// Maximum Newton-Raphson iterations (bounded for RT safety).
    max_iter: usize,
}

impl OtaRoot {
    pub fn new(model: OtaModel) -> Self {
        Self {
            iabc: model.iabc_max,
            model,
            max_iter: 16,
        }
    }

    /// Set the amplifier bias current (external control from envelope, LFO, etc.).
    ///
    /// In a compressor circuit, higher envelope = lower Iabc = less gain.
    /// Range: 0 to iabc_max.
    #[inline]
    pub fn set_iabc(&mut self, iabc: f64) {
        self.iabc = iabc.clamp(0.0, self.model.iabc_max);
    }

    /// Get current amplifier bias current.
    #[inline]
    pub fn iabc(&self) -> f64 {
        self.iabc
    }

    /// Set gain as a normalized value (0.0 = off, 1.0 = max gain).
    ///
    /// This is a convenience wrapper for compressor-style control:
    /// the envelope follower output (0-1) directly maps to gain reduction.
    #[inline]
    pub fn set_gain_normalized(&mut self, gain: f64) {
        self.iabc = gain.clamp(0.0, 1.0) * self.model.iabc_max;
    }

    /// Compute the OTA output current for a given input voltage.
    ///
    /// Uses the tanh transfer characteristic of the differential pair:
    /// `Iout = Iabc * tanh(Vin / (2*Vt))`
    #[inline]
    pub fn output_current(&self, v_in: f64) -> f64 {
        let x = v_in / (2.0 * self.model.vt);
        self.iabc * x.tanh()
    }

    /// Derivative of output current w.r.t. input voltage.
    ///
    /// `dIout/dVin = Iabc / (2*Vt) * sech²(Vin / (2*Vt))`
    ///            = gm * sech²(Vin / (2*Vt))
    #[inline]
    fn output_current_derivative(&self, v_in: f64) -> f64 {
        let x = v_in / (2.0 * self.model.vt);
        let sech = 1.0 / x.cosh();
        self.iabc / (2.0 * self.model.vt) * sech * sech
    }

    /// Compute the transconductance (gm) at the current bias point.
    ///
    /// gm = Iabc / (2 * Vt) — this is the small-signal gain.
    /// For CA3080 at Iabc=500µA: gm ≈ 9.7 mA/V (≈ 9.7 mS)
    pub fn transconductance(&self) -> f64 {
        self.iabc / (2.0 * self.model.vt)
    }
}

impl WdfRoot for OtaRoot {
    /// OTA transconductance: `i = Iabc * tanh(v / (2·Vt))`
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
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
