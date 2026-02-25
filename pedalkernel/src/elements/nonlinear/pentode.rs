//! Pentode vacuum tube WDF root elements.
//!
//! Models screen-referenced pentodes (EF86, EL84) using the Koren equation.

use super::solver::{softplus, newton_raphson_solve, LEAKAGE_CONDUCTANCE};
use crate::elements::WdfRoot;

// ---------------------------------------------------------------------------
// Pentode (Vacuum Tube) Models
// ---------------------------------------------------------------------------

/// Pentode model parameters using a screen-referenced Koren equation.
///
/// Pentodes have five electrodes: cathode, control grid (g1), screen grid (g2),
/// suppressor grid (g3, usually tied to cathode), and plate. The screen grid
/// acts as the "effective plate" for controlling current — the plate merely
/// collects it, giving pentodes their characteristic flat plate curves and
/// high output impedance.
///
/// The screen-referenced Koren equation:
/// ```text
/// Ip_base = (Vg2k/Kp * ln(1 + exp(Kp * (1/mu + Vg1k/sqrt(Kvb + Vg2k^2)))))^Ex
/// Ip = Ip_base * (2/π) * atan(Vpk / Kvb2)
/// ```
/// The atan term models the pentode's plate saturation characteristic —
/// plate current is nearly independent of Vpk in the normal operating region.
#[derive(Debug, Clone, Copy)]
pub struct PentodeModel {
    /// Amplification factor (mu), screen-referenced. EF86 ≈ 38, EL84 ≈ 19.
    pub mu: f64,
    /// Plate resistance factor. Affects output impedance.
    pub kp: f64,
    /// Knee voltage constant for the screen-referenced Koren equation.
    pub kvb: f64,
    /// Exponent (typically 1.3-1.5). Affects transfer curve shape.
    pub ex: f64,
    /// Knee voltage for pentode plate saturation (V).
    /// Controls how quickly plate current saturates with Vpk.
    /// Smaller = sharper knee. EF86 ≈ 12, EL84 ≈ 20.
    pub kvb2: f64,
    /// Default screen grid voltage (V) for typical operating point.
    pub vg2_default: f64,
}

impl PentodeModel {
    /// EF86 / 6267 — Small-signal pentode.
    ///
    /// The preamp tube in the Vox AC15 and AC30 (V1 position).
    /// Very high gain, low noise. Used for its chimey, glassy character.
    /// Typical operating point: Va=170V, Vg2=140V, Vg1=-2V, Ia=3mA.
    pub fn p_ef86() -> Self {
        Self {
            mu: 38.0,
            kp: 1460.0,
            kvb: 300.0,
            ex: 1.35,
            kvb2: 12.0,
            vg2_default: 140.0,
        }
    }

    /// EL84 / 6BQ5 — Power pentode.
    ///
    /// The output tube in the Vox AC15, AC30, and many boutique amps.
    /// Known for musical, chimey distortion and relatively low headroom
    /// (12-17W per push-pull pair). Class A operation in the AC15.
    /// Typical operating point: Va=250V, Vg2=250V, Vg1=-7.3V, Ia=48mA.
    pub fn p_el84() -> Self {
        Self {
            mu: 19.0,
            kp: 600.0,
            kvb: 300.0,
            ex: 1.35,
            kvb2: 20.0,
            vg2_default: 250.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Pentode Root
// ---------------------------------------------------------------------------

/// Pentode nonlinear root for WDF trees.
///
/// Models the plate-cathode path as a nonlinear element controlled by
/// an external control grid voltage (Vg1k) and screen grid voltage (Vg2k).
/// Uses Newton-Raphson to solve the implicit WDF constraint equation.
///
/// The plate current follows a screen-referenced Koren model with an
/// atan-based plate saturation term, accurately capturing the pentode's
/// nearly flat plate curves in the saturation region.
///
/// Pins: `.grid` (control grid g1), `.screen` (screen grid g2),
///       `.plate`, `.cathode`
#[derive(Debug, Clone, Copy)]
pub struct PentodeRoot {
    pub model: PentodeModel,
    /// Current control grid voltage (g1-cathode). Set externally.
    vg1k: f64,
    /// Current screen grid voltage (g2-cathode). Typically fixed at operating point.
    vg2k: f64,
    /// Maximum plate voltage (determined by supply rail B+).
    /// Pentode plate can swing from 0V (saturated) to B+ (cutoff).
    v_max: f64,
    /// Maximum Newton-Raphson iterations (bounded for RT safety).
    max_iter: usize,
}

impl PentodeRoot {
    pub fn new(model: PentodeModel) -> Self {
        let vg2k = model.vg2_default;
        Self {
            model,
            vg1k: 0.0,
            vg2k,
            v_max: 500.0, // Default to high-voltage tube amp (will be set by supply)
            max_iter: 16,
        }
    }

    /// Create a pentode root with a specific supply voltage (B+).
    ///
    /// Use this when the supply voltage is known at construction time.
    pub fn new_with_v_max(model: PentodeModel, v_max: f64) -> Self {
        let vg2k = model.vg2_default;
        Self {
            model,
            vg1k: 0.0,
            vg2k,
            v_max: v_max.max(1.0),
            max_iter: 16,
        }
    }

    /// Set the maximum plate voltage (B+ supply rail).
    ///
    /// For tube circuits, the plate voltage can swing from 0V (tube saturated)
    /// to B+ (tube in cutoff). This sets the upper bound for Newton-Raphson.
    #[inline]
    pub fn set_v_max(&mut self, v_max: f64) {
        self.v_max = v_max.max(1.0); // Minimum 1V to avoid degeneracy
    }

    /// Get the current v_max setting.
    #[inline]
    pub fn v_max(&self) -> f64 {
        self.v_max
    }

    /// Set the control grid voltage (g1-cathode). External modulation.
    #[inline]
    pub fn set_vg1k(&mut self, vg1k: f64) {
        self.vg1k = vg1k;
    }

    /// Get current control grid voltage.
    #[inline]
    pub fn vg1k(&self) -> f64 {
        self.vg1k
    }

    /// Set the screen grid voltage (g2-cathode).
    /// Usually fixed at the operating point, but can be modulated for sag effects.
    #[inline]
    pub fn set_vg2k(&mut self, vg2k: f64) {
        self.vg2k = vg2k;
    }

    /// Get current screen grid voltage.
    #[inline]
    pub fn vg2k(&self) -> f64 {
        self.vg2k
    }

    /// Compute plate current for given Vpk at current Vg1k and Vg2k.
    ///
    /// Screen-referenced Koren model:
    /// ```text
    /// E1 = Kp * (1/mu + Vg1k / sqrt(Kvb + Vg2k^2))
    /// Ip_base = (Vg2k/Kp * ln(1 + exp(E1)))^Ex
    /// Ip = Ip_base * (2/π) * atan(Vpk / Kvb2)
    /// ```
    #[inline]
    pub fn plate_current(&self, vpk: f64) -> f64 {
        let mu = self.model.mu;
        let kp = self.model.kp;
        let kvb = self.model.kvb;
        let ex = self.model.ex;
        let kvb2 = self.model.kvb2;
        let vg1k = self.vg1k;
        let vg2k = self.vg2k;

        // No current for negative plate voltage
        if vpk <= 0.0 {
            return 0.0;
        }

        // Screen-referenced Koren: E1 uses Vg2k instead of Vpk
        let e1 = kp * (1.0 / mu + vg1k / (kvb + vg2k * vg2k).sqrt());

        let ln_term = softplus(e1);
        let base = (vg2k / kp) * ln_term;
        if base <= 0.0 {
            return 0.0;
        }

        let ip_base = base.powf(ex);

        // Pentode plate saturation: atan-based transition
        // Normalized to approach 1.0 for large Vpk
        let plate_factor = (2.0 / std::f64::consts::PI) * (vpk / kvb2).atan();

        ip_base * plate_factor.max(0.0)
    }

    /// Compute derivative of plate current w.r.t. Vpk for Newton-Raphson.
    ///
    /// Since Ip_base is independent of Vpk in the pentode model,
    /// only the plate saturation factor contributes:
    /// `dIp/dVpk = Ip_base * (2/π) * (1 / (1 + (Vpk/Kvb2)^2)) * (1/Kvb2)`
    #[inline]
    fn plate_current_derivative(&self, vpk: f64) -> f64 {
        let mu = self.model.mu;
        let kp = self.model.kp;
        let kvb = self.model.kvb;
        let ex = self.model.ex;
        let kvb2 = self.model.kvb2;
        let vg1k = self.vg1k;
        let vg2k = self.vg2k;

        if vpk <= 0.0 {
            return LEAKAGE_CONDUCTANCE;
        }

        let e1 = kp * (1.0 / mu + vg1k / (kvb + vg2k * vg2k).sqrt());

        let ln_term = softplus(e1);
        let base = (vg2k / kp) * ln_term;
        if base <= 0.0 {
            return LEAKAGE_CONDUCTANCE;
        }

        let ip_base = base.powf(ex);

        // d/dVpk of (2/π) * atan(Vpk/Kvb2) = (2/π) * 1/(1 + (Vpk/Kvb2)^2) * 1/Kvb2
        let vpk_ratio = vpk / kvb2;
        let d_plate_factor = (2.0 / std::f64::consts::PI) / (1.0 + vpk_ratio * vpk_ratio) / kvb2;

        ip_base * d_plate_factor
    }
}

impl WdfRoot for PentodeRoot {
    /// Pentode plate-cathode path: `i = Ip(Vpk, Vg1k, Vg2k)`
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
        let root = *self;
        let v_max = self.v_max;
        newton_raphson_solve(a, rp, a * 0.5, self.max_iter, 1e-6,
            Some((-50.0, v_max)), None,
            |v| (root.plate_current(v), root.plate_current_derivative(v)),
        )
    }
}
