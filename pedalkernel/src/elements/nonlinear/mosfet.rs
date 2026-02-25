//! Enhancement-mode MOSFET WDF root elements.
//!
//! Models N-channel and P-channel MOSFETs with square-law drain current.

use super::solver::{newton_raphson_solve, LEAKAGE_CONDUCTANCE};
use crate::elements::WdfRoot;

// ---------------------------------------------------------------------------
// MOSFET Models (Enhancement-mode)
// ---------------------------------------------------------------------------

/// Enhancement-mode MOSFET model parameters.
///
/// Uses the standard square-law model for drain current:
/// - Cutoff: `Ids = 0` when `|Vgs| < |Vth|`
/// - Triode: `Ids = Kp * [2*(Vgs-Vth)*Vds - Vds^2]`
/// - Saturation: `Ids = Kp * (Vgs-Vth)^2 * (1 + lambda*|Vds|)`
#[derive(Debug, Clone, Copy)]
pub struct MosfetModel {
    /// Threshold voltage (V). N-channel: positive, P-channel: negative.
    pub vth: f64,
    /// Transconductance parameter (A/V²). Kp = µn·Cox·W/(2L).
    pub kp: f64,
    /// Channel-length modulation parameter (1/V). Typical: 0.01-0.1.
    pub lambda: f64,
    /// True for N-channel, false for P-channel.
    pub is_n_channel: bool,
}

impl MosfetModel {
    /// 2N7000 N-channel enhancement MOSFET.
    ///
    /// Very common in guitar pedal clipping (Fulltone OCD, modern drives).
    /// Low threshold voltage, small package, fast switching.
    pub fn n_2n7000() -> Self {
        Self {
            vth: 2.1,
            kp: 0.1,
            lambda: 0.04,
            is_n_channel: true,
        }
    }

    /// IRF520 N-channel power MOSFET.
    ///
    /// Higher current capability, used in some high-headroom drive circuits.
    pub fn n_irf520() -> Self {
        Self {
            vth: 4.0,
            kp: 1.0,
            lambda: 0.02,
            is_n_channel: true,
        }
    }

    /// BS250 P-channel enhancement MOSFET.
    ///
    /// Used in some boost circuits and complementary configurations.
    pub fn p_bs250() -> Self {
        Self {
            vth: -1.5,
            kp: 0.07,
            lambda: 0.04,
            is_n_channel: false,
        }
    }

    /// IRF9520 P-channel power MOSFET.
    pub fn p_irf9520() -> Self {
        Self {
            vth: -4.0,
            kp: 0.5,
            lambda: 0.02,
            is_n_channel: false,
        }
    }
}

// ---------------------------------------------------------------------------
// MOSFET Root
// ---------------------------------------------------------------------------

/// Enhancement-mode MOSFET nonlinear root for WDF trees.
///
/// Models the drain-source path as a nonlinear element controlled by
/// an external gate-source voltage Vgs. Uses Newton-Raphson to solve
/// the implicit WDF constraint equation.
///
/// Unlike JFETs (depletion-mode, normally on), enhancement MOSFETs are
/// normally off and require Vgs > Vth to conduct.
#[derive(Debug, Clone, Copy)]
pub struct MosfetRoot {
    pub model: MosfetModel,
    /// Current gate-source voltage (external control parameter).
    vgs: f64,
    /// Maximum Newton-Raphson iterations (bounded for RT safety).
    max_iter: usize,
}

impl MosfetRoot {
    pub fn new(model: MosfetModel) -> Self {
        Self {
            model,
            vgs: 0.0,
            max_iter: 16,
        }
    }

    /// Set the gate-source voltage (external control from bias, signal, etc.)
    #[inline]
    pub fn set_vgs(&mut self, vgs: f64) {
        self.vgs = vgs;
    }

    /// Get current gate-source voltage.
    #[inline]
    pub fn vgs(&self) -> f64 {
        self.vgs
    }

    /// Compute drain current for given Vds at current Vgs.
    ///
    /// Enhancement-mode square-law model:
    /// - Cutoff: Ids = 0 when |Vgs| < |Vth|
    /// - Triode: Ids = Kp * [2*(Vgs-Vth)*Vds - Vds²]
    /// - Saturation: Ids = Kp * (Vgs-Vth)² * (1 + lambda*|Vds|)
    #[inline]
    pub fn drain_current(&self, vds: f64) -> f64 {
        let vth = self.model.vth;
        let kp = self.model.kp;
        let lambda = self.model.lambda;
        let vgs = self.vgs;

        // Cutoff check
        if self.model.is_n_channel {
            if vgs <= vth {
                return 0.0;
            }
        } else {
            // P-channel: conducts when Vgs < Vth (Vth is negative)
            if vgs >= vth {
                return 0.0;
            }
        }

        // Overdrive voltage
        let vov = if self.model.is_n_channel {
            vgs - vth
        } else {
            vth - vgs // positive magnitude for P-channel
        };

        let vds_abs = vds.abs();
        let vds_sign = vds.signum();

        let ids_magnitude = if vds_abs < vov {
            // Triode (linear) region
            kp * (2.0 * vov * vds_abs - vds_abs * vds_abs)
        } else {
            // Saturation region with channel-length modulation
            kp * vov * vov * (1.0 + lambda * vds_abs)
        };

        ids_magnitude * vds_sign
    }

    /// Compute derivative of drain current w.r.t. Vds.
    #[inline]
    fn drain_current_derivative(&self, vds: f64) -> f64 {
        let vth = self.model.vth;
        let kp = self.model.kp;
        let lambda = self.model.lambda;
        let vgs = self.vgs;

        // Cutoff: small conductance
        if self.model.is_n_channel {
            if vgs <= vth {
                return LEAKAGE_CONDUCTANCE;
            }
        } else if vgs >= vth {
            return LEAKAGE_CONDUCTANCE;
        }

        let vov = if self.model.is_n_channel {
            vgs - vth
        } else {
            vth - vgs
        };

        let vds_abs = vds.abs();

        if vds_abs < vov {
            // Triode: dIds/dVds = Kp * 2*(Vov - Vds)
            kp * 2.0 * (vov - vds_abs)
        } else {
            // Saturation: dIds/dVds = Kp * Vov^2 * lambda
            kp * vov * vov * lambda
        }
    }
}

impl WdfRoot for MosfetRoot {
    /// MOSFET drain-source path: `i = Ids(Vds, Vgs)`
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
        let root = *self;
        let vov = if self.model.is_n_channel {
            (self.vgs - self.model.vth).max(0.0)
        } else {
            (self.model.vth - self.vgs).max(0.0)
        };
        let gds_approx = 2.0 * self.model.kp * vov;
        let v0 = if gds_approx > LEAKAGE_CONDUCTANCE {
            a / (2.0 + 2.0 * rp * gds_approx)
        } else {
            a * 0.5
        };
        newton_raphson_solve(a, rp, v0, self.max_iter, 1e-6, None, None, |v| {
            (root.drain_current(v), root.drain_current_derivative(v))
        })
    }
}
