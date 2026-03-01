//! Triode vacuum tube WDF root elements.
//!
//! Models preamp tubes (12AX7, 12AT7, 12AU7) using the Koren equation.
//! Parameters are loaded from the embedded `triodes.model` file.

use super::solver::{softplus, newton_raphson_solve, LEAKAGE_CONDUCTANCE};
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
/// `Ip = (Vpk/Kp * ln(1 + exp(Kp * (1/mu + Vgk/sqrt(Kvb + Vpk^2)))))^Ex`
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
    /// Plate current scaling factor. Stored for reference but not yet used
    /// in the plate_current() equation (future KG1 incorporation).
    pub kg1: f64,
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
    /// `Ip = (Vpk/Kp * ln(1 + exp(Kp * (1/mu + Vgk/sqrt(Kvb + Vpk^2)))))^Ex`
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

        // Scale by parallel_count: N tubes in parallel = N × single tube current.
        base.powf(ex) * self.parallel_count as f64
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

        // d(base^Ex)/dVpk = Ex * base^(Ex-1) * dbase_dvpk
        // Scale by parallel_count to match plate_current() scaling.
        ex * base.powf(ex - 1.0) * dbase_dvpk * self.parallel_count as f64
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
