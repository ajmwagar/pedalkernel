//! Triode vacuum tube WDF root elements.
//!
//! Models preamp tubes (12AX7, 12AT7, 12AU7) using the Koren equation.
//! Parameters are loaded from the embedded `triodes.model` file.

use super::solver::{softplus, newton_raphson_solve, NlDeviceIv, LEAKAGE_CONDUCTANCE};
use crate::elements::WdfRoot;
use crate::models::{SpiceTriodeModel, triode_by_name};

// ---------------------------------------------------------------------------
// Triode (Vacuum Tube) Models
// ---------------------------------------------------------------------------

/// Triode model parameters using the Koren equation.
///
/// Models common preamp tubes (12AX7, 12AT7, 12AU7) with the industry-standard
/// Koren model for accurate tube amplifier simulation.
///
/// The Koren equation:
/// `Ip = (Vpk/Kp * ln(1 + exp(Kp * (1/mu + Vgk/sqrt(Kvb + Vpk^2)))))^Ex / KG1`
#[derive(Debug, Clone, Copy)]
pub struct TriodeModel {
    /// Amplification factor (mu). Higher = more gain. 12AX7 ≈ 100, 12AU7 ≈ 20.
    pub mu: f64,
    /// Plate resistance factor. Affects output impedance.
    pub kp: f64,
    /// Knee voltage constant. Affects saturation behavior.
    pub kvb: f64,
    /// Exponent (typically 1.3-1.5). Affects transfer curve shape.
    pub ex: f64,
    /// Plate current scaling factor (KG1). Scales the absolute plate current
    /// magnitude. 12AX7 ≈ 1060, 12AU7 ≈ 1180.
    pub kg1: f64,
    /// Plate resistance at typical operating point (Ω).
    /// Used as virtual resistance in WDF tree construction.
    /// 12AX7 ≈ 62.5kΩ, 12AU7 ≈ 7.7kΩ, 12AT7 ≈ 10.9kΩ.
    pub rp: f64,
}

impl TriodeModel {
    /// Look up a triode model by name from the model registry.
    /// Panics if the name is not found.
    pub fn by_name(name: &str) -> Self {
        Self::try_by_name(name).unwrap_or_else(|| {
            panic!("Unknown triode model: '{}'. Use triode_model_names() to list available models.", name)
        })
    }

    /// Look up a triode model by name, returning None if not found.
    pub fn try_by_name(name: &str) -> Option<Self> {
        triode_by_name(name).map(Self::from)
    }
}

impl From<&SpiceTriodeModel> for TriodeModel {
    fn from(spice: &SpiceTriodeModel) -> Self {
        Self {
            mu: spice.mu,
            kp: spice.kp,
            kvb: spice.kvb,
            ex: spice.ex,
            kg1: spice.kg1,
            rp: spice.rp,
        }
    }
}

// ---------------------------------------------------------------------------
// Triode Root
// ---------------------------------------------------------------------------

/// Triode nonlinear root for WDF trees.
///
/// Models the plate-cathode path as a nonlinear element controlled by
/// an external grid-cathode voltage Vgk. Uses Newton-Raphson to solve
/// the implicit WDF constraint equation.
///
/// The plate current follows the Koren model, which accurately captures
/// the tube's behavior in cutoff, active, and saturation regions.
#[derive(Debug, Clone, Copy)]
pub struct TriodeRoot {
    pub model: TriodeModel,
    /// Current grid-cathode voltage (external control parameter).
    vgk: f64,
    /// Maximum plate voltage (determined by supply rail B+).
    /// Triode plate can swing from 0V (saturated) to B+ (cutoff).
    v_max: f64,
    /// Maximum Newton-Raphson iterations (bounded for RT safety).
    max_iter: usize,
    /// Number of parallel tubes (default 1). Plate current is scaled by N.
    parallel_count: usize,
}

impl TriodeRoot {
    pub fn new(model: TriodeModel) -> Self {
        Self {
            model,
            vgk: 0.0,
            v_max: 500.0, // Default to high-voltage tube amp (will be set by supply)
            max_iter: 16,
            parallel_count: 1,
        }
    }

    /// Create a triode root with a specific supply voltage (B+).
    pub fn new_with_v_max(model: TriodeModel, v_max: f64) -> Self {
        Self {
            model,
            vgk: 0.0,
            v_max: v_max.max(1.0),
            max_iter: 16,
            parallel_count: 1,
        }
    }

    /// Set the number of parallel tubes for current scaling.
    pub fn with_parallel_count(mut self, count: usize) -> Self {
        self.parallel_count = count.max(1);
        self
    }

    /// Set the maximum plate voltage (B+ supply rail).
    ///
    /// For tube circuits, the plate voltage can swing from 0V (tube saturated)
    /// to B+ (tube in cutoff). This sets the upper bound for Newton-Raphson.
    ///
    /// Examples:
    /// - Pultec EQP-1A: 250V
    /// - Fender Deluxe: 350V
    /// - Starved plate design: 9-48V
    #[inline]
    pub fn set_v_max(&mut self, v_max: f64) {
        self.v_max = v_max.max(1.0); // Minimum 1V to avoid degeneracy
    }

    /// Get the current v_max setting.
    #[inline]
    pub fn v_max(&self) -> f64 {
        self.v_max
    }

    pub fn parallel_count(&self) -> usize {
        self.parallel_count
    }

    /// Set the grid-cathode voltage (external control from bias, signal, LFO).
    #[inline]
    pub fn set_vgk(&mut self, vgk: f64) {
        self.vgk = vgk;
    }

    /// Get current grid-cathode voltage.
    #[inline]
    pub fn vgk(&self) -> f64 {
        self.vgk
    }

    /// Compute plate current for given Vpk at current Vgk using Koren model.
    ///
    /// The Koren equation:
    /// `Ip = (Vpk/Kp * ln(1 + exp(Kp * (1/mu + Vgk/sqrt(Kvb + Vpk^2)))))^Ex / KG1`
    #[inline]
    pub fn plate_current(&self, vpk: f64) -> f64 {
        let mu = self.model.mu;
        let kp = self.model.kp;
        let kvb = self.model.kvb;
        let ex = self.model.ex;
        let vgk = self.vgk;

        // Handle negative plate voltage (reverse bias) - no current
        if vpk <= 0.0 {
            return 0.0;
        }

        // Koren model: E1 = Kp * (1/mu + Vgk / sqrt(Kvb + Vpk^2))
        let e1 = kp * (1.0 / mu + vgk / (kvb + vpk * vpk).sqrt());

        // Softplus: ln(1 + exp(E1)) — handles cutoff (E1 << 0) and
        // saturation (E1 >> 0) numerically stably.
        let ln_term = softplus(e1);
        if ln_term <= 0.0 {
            return 0.0;
        }

        // Ip = (Vpk/Kp * ln_term)^Ex
        let base = (vpk / kp) * ln_term;
        if base <= 0.0 {
            return 0.0;
        }

        // Ip = base^Ex / KG1, scaled by parallel_count for N tubes in parallel.
        (base.powf(ex) / self.model.kg1) * self.parallel_count as f64
    }

    /// Compute derivative of plate current w.r.t. Vpk for Newton-Raphson.
    #[inline]
    fn plate_current_derivative(&self, vpk: f64) -> f64 {
        let mu = self.model.mu;
        let kp = self.model.kp;
        let kvb = self.model.kvb;
        let ex = self.model.ex;
        let vgk = self.vgk;

        if vpk <= 0.0 {
            return LEAKAGE_CONDUCTANCE; // Small conductance to avoid division issues
        }

        let vpk_sq = vpk * vpk;
        let sqrt_term = (kvb + vpk_sq).sqrt();
        let e1 = kp * (1.0 / mu + vgk / sqrt_term);

        let ln_term = softplus(e1);
        if ln_term <= 0.0 {
            return LEAKAGE_CONDUCTANCE;
        }

        // Compute exp(E1)/(1+exp(E1)) = sigmoid(E1) for the derivative.
        // Numerically stable: for large E1, sigmoid → 1.0.
        let sigmoid_e1 = if e1 > 50.0 {
            1.0
        } else if e1 < -50.0 {
            0.0
        } else {
            let exp_e1 = e1.exp();
            exp_e1 / (1.0 + exp_e1)
        };

        let base = (vpk / kp) * ln_term;
        if base <= 0.0 {
            return LEAKAGE_CONDUCTANCE;
        }

        // dE1/dVpk = Kp * Vgk * (-Vpk) / (Kvb + Vpk^2)^(3/2)
        let de1_dvpk = -kp * vgk * vpk / (sqrt_term * sqrt_term * sqrt_term);

        // d(ln(1+exp(E1)))/dVpk = sigmoid(E1) * dE1/dVpk
        let dln_dvpk = sigmoid_e1 * de1_dvpk;

        // d(Vpk/Kp * ln_term)/dVpk = ln_term/Kp + (Vpk/Kp) * dln_dvpk
        let dbase_dvpk = ln_term / kp + (vpk / kp) * dln_dvpk;

        // d(base^Ex / KG1)/dVpk = Ex * base^(Ex-1) * dbase_dvpk / KG1
        // Scale by parallel_count to match plate_current() scaling.
        (ex * base.powf(ex - 1.0) * dbase_dvpk / self.model.kg1) * self.parallel_count as f64
    }
}

impl WdfRoot for TriodeRoot {
    /// Triode plate-cathode path: `i = Ip(Vpk, Vgk)`
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

impl NlDeviceIv for TriodeRoot {
    #[inline]
    fn iv(&self, v: f64) -> (f64, f64) {
        (self.plate_current(v), self.plate_current_derivative(v))
    }

    #[inline]
    fn v_clamp(&self) -> (f64, f64) {
        (-50.0, self.v_max)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// SPICE-reference validation: 12AX7 at Vgk=-1.5V, Vpk=200V.
    ///
    /// Hand-computed from Koren's equation with 12AX7 parameters
    /// (MU=100, EX=1.4, KG1=1060, KP=600, KVB=300):
    ///
    /// ```text
    /// E1_arg = 600 * (1/100 + (-1.5)/sqrt(300 + 200²))
    ///        = 600 * (0.01 - 1.5/200.749) = 600 * 0.002528 = 1.517
    /// ln_term = ln(1 + exp(1.517)) = ln(5.559) = 1.715
    /// base = (200/600) * 1.715 = 0.5717
    /// Ip = 0.5717^1.4 / 1060 = 0.457 / 1060 ≈ 4.31e-4 A
    /// ```
    #[test]
    fn triode_12ax7_spice_reference() {
        let mut triode = TriodeRoot::new(TriodeModel::by_name("12AX7"));
        triode.set_vgk(-1.5);
        let ip = triode.plate_current(200.0);

        // Hand-computed: ~4.31e-4 A (0.431 mA)
        let expected = 4.31e-4;
        let error = (ip - expected).abs() / expected;
        assert!(
            error < 0.01,
            "12AX7 Ip should match SPICE reference within 1%: got {ip:.6e}, expected {expected:.6e}, error={error:.4}"
        );
    }
}
