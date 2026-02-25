//! Triode vacuum tube WDF root elements.
//!
//! Models preamp tubes (12AX7, 12AT7, 12AU7) using the Koren equation.

use super::solver::{softplus, newton_raphson_solve, LEAKAGE_CONDUCTANCE};
use crate::elements::WdfRoot;

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
    /// Exponent (typically 1.4-1.5). Affects transfer curve shape.
    pub ex: f64,
}

impl TriodeModel {
    /// 12AX7 / ECC83 - High gain preamp tube.
    ///
    /// The most common guitar amp tube. High mu (100) gives strong gain,
    /// making it ideal for preamp stages. Produces warm, musical distortion.
    pub fn t_12ax7() -> Self {
        Self {
            mu: 100.0,
            kp: 600.0,
            kvb: 300.0,
            ex: 1.4,
        }
    }

    /// 12AT7 / ECC81 - Medium gain preamp tube.
    ///
    /// Lower gain than 12AX7 but higher transconductance. Often used in
    /// reverb drivers, phase inverters, and where cleaner gain is needed.
    pub fn t_12at7() -> Self {
        Self {
            mu: 60.0,
            kp: 300.0,
            kvb: 300.0,
            ex: 1.5,
        }
    }

    /// 12AU7 / ECC82 - Low gain preamp tube.
    ///
    /// Much lower mu (20) gives clean, linear amplification. Used in
    /// phase inverters, cathode followers, and hi-fi applications.
    pub fn t_12au7() -> Self {
        Self {
            mu: 20.0,
            kp: 84.0,
            kvb: 300.0,
            ex: 1.5,
        }
    }

    /// ECC83 - European designation for 12AX7.
    pub fn t_ecc83() -> Self {
        Self::t_12ax7()
    }

    /// ECC81 - European designation for 12AT7.
    pub fn t_ecc81() -> Self {
        Self::t_12at7()
    }

    /// ECC82 - European designation for 12AU7.
    pub fn t_ecc82() -> Self {
        Self::t_12au7()
    }

    /// 12AY7 / 6072 - Low-medium gain preamp tube.
    ///
    /// The original Fender tweed tube. Lower mu (44) than 12AX7 gives a
    /// cleaner, more dynamic response — the amp cleans up beautifully with
    /// guitar volume. Used in the Fender 5E3 Tweed Deluxe V1 position.
    /// Produces warm, touch-sensitive breakup that's highly responsive to
    /// picking dynamics.
    pub fn t_12ay7() -> Self {
        Self {
            mu: 44.0,
            kp: 420.0,
            kvb: 300.0,
            ex: 1.4,
        }
    }

    /// 6072 - Military/industrial designation for 12AY7.
    pub fn t_6072() -> Self {
        Self::t_12ay7()
    }

    /// 12BH7A - Medium-mu dual triode with high current capability.
    /// Used in cathode follower stages requiring higher current than 12AU7.
    /// Popular in LA-2A totem-pole output stage, hi-fi amplifiers.
    /// Mu ≈ 17 (similar to 12AU7), max Ip = 15mA per section.
    pub fn t_12bh7() -> Self {
        Self {
            mu: 17.0,       // Same as 12AU7
            kp: 60.0,       // Lower Kp for higher current handling
            kvb: 400.0,     // Similar to 12AU7
            ex: 1.3,
        }
    }

    /// 6386 - Remote-cutoff (variable-mu) dual triode.
    ///
    /// THE tube that makes the Fairchild 670 compressor possible.
    /// Unlike regular triodes, mu varies with grid bias:
    ///   - At low negative bias (~0V): mu ≈ 50
    ///   - At high negative bias (-5V): mu ≈ 5
    ///
    /// This variable-mu characteristic provides smooth, musical gain reduction.
    /// The compression ratio is set by how much the side-chain drives the grid.
    ///
    /// Note: For full variable-mu modeling, use RemoteCutoffTriodeRoot which
    /// dynamically calculates mu from Vgk. This basic model uses nominal mu=40
    /// which represents mid-range operation.
    ///
    /// Developed by General Electric for audio AGC applications. The Fairchild
    /// 670 uses two matched sections per channel in push-pull configuration.
    pub fn t_6386() -> Self {
        Self {
            mu: 40.0,       // Nominal mu (varies 5-50 with bias in real tube)
            kp: 180.0,      // Medium Kp for variable-mu characteristic
            kvb: 300.0,     // Standard knee voltage
            ex: 1.4,        // Standard exponent
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
}

impl TriodeRoot {
    pub fn new(model: TriodeModel) -> Self {
        Self {
            model,
            vgk: 0.0,
            v_max: 500.0, // Default to high-voltage tube amp (will be set by supply)
            max_iter: 16,
        }
    }

    /// Create a triode root with a specific supply voltage (B+).
    ///
    /// Use this when the supply voltage is known at construction time.
    pub fn new_with_v_max(model: TriodeModel, v_max: f64) -> Self {
        Self {
            model,
            vgk: 0.0,
            v_max: v_max.max(1.0),
            max_iter: 16,
        }
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

        base.powf(ex)
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
        ex * base.powf(ex - 1.0) * dbase_dvpk
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
