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
/// Uses the SPICE Level 1 square-law model for drain current:
/// - Cutoff: `Ids = 0` when `|Vgs| < |Vth|`
/// - Triode: `Ids = Kp * [(Vgs-Vth)*Vds - 0.5*Vds^2]`
/// - Saturation: `Ids = 0.5*Kp * (Vgs-Vth)^2 * (1 + lambda*|Vds|)`
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MosfetModel {
    /// Threshold voltage (V). N-channel: positive, P-channel: negative.
    pub vth: crate::Wave,
    /// SPICE Level 1 transconductance parameter (A/V²), before the 1/2
    /// square-law saturation factor.
    pub kp: crate::Wave,
    /// Channel-length modulation parameter (1/V). Typical: 0.01-0.1.
    pub lambda: crate::Wave,
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

    /// Drain current `Ids(Vgs, Vds)` — enhancement-mode square-law model.
    ///
    /// - Cutoff: `Ids = 0` when the channel is off (N: `Vgs <= Vth`,
    ///   P: `Vgs >= Vth`)
    /// - Triode: `Ids = Kp * [Vov*|Vds| - 0.5*Vds²] * sign(Vds)`
    /// - Saturation: `Ids = 0.5*Kp * Vov² * (1 + lambda*|Vds|) * sign(Vds)`
    ///
    /// The channel conducts symmetrically in both Vds polarities (drain and
    /// source are interchangeable in the square-law model).
    #[inline]
    pub fn ids(&self, vgs: crate::Wave, vds: crate::Wave) -> crate::Wave {
        let vov = match self.overdrive(vgs) {
            Some(vov) => vov,
            None => return 0.0,
        };

        let vds_abs = vds.abs();
        let vds_sign = vds.signum();

        let ids_magnitude = if vds_abs < vov {
            // Triode (linear) region
            self.kp * (vov * vds_abs - 0.5 * vds_abs * vds_abs)
        } else {
            // Saturation region with channel-length modulation
            0.5 * self.kp * vov * vov * (1.0 + self.lambda * vds_abs)
        };

        ids_magnitude * vds_sign
    }

    /// Derivative `dIds/dVds` at fixed Vgs.
    ///
    /// Returns `LEAKAGE_CONDUCTANCE` in cutoff so Newton-Raphson always sees
    /// a non-zero slope.
    #[inline]
    pub fn ids_vds_derivative(&self, vgs: crate::Wave, vds: crate::Wave) -> crate::Wave {
        let vov = match self.overdrive(vgs) {
            Some(vov) => vov,
            None => return LEAKAGE_CONDUCTANCE,
        };

        let vds_abs = vds.abs();

        if vds_abs < vov {
            // Triode: dIds/dVds = Kp * (Vov - |Vds|)
            self.kp * (vov - vds_abs)
        } else {
            // Saturation: dIds/dVds = 0.5 * Kp * Vov² * lambda
            0.5 * self.kp * vov * vov * self.lambda
        }
    }

    /// Overdrive voltage magnitude `Vov`, or `None` when in cutoff.
    ///
    /// N-channel: `Vov = Vgs - Vth` (conducts when `Vgs > Vth`).
    /// P-channel: `Vov = Vth - Vgs` (conducts when `Vgs < Vth`, Vth < 0).
    #[inline]
    fn overdrive(&self, vgs: crate::Wave) -> Option<crate::Wave> {
        if self.is_n_channel {
            if vgs <= self.vth {
                return None;
            }
            Some(vgs - self.vth)
        } else {
            if vgs >= self.vth {
                return None;
            }
            Some(self.vth - vgs)
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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MosfetRoot {
    pub model: MosfetModel,
    /// Current gate-source voltage (external control parameter).
    vgs: crate::Wave,
    /// DC gate-source operating point. Runtime signal control is added on top.
    #[cfg_attr(feature = "serde", serde(default))]
    vgs_bias: crate::Wave,
    /// DC drain-source operating point. The WDF port solves AC deviation
    /// around this voltage.
    #[cfg_attr(feature = "serde", serde(default))]
    vds_bias: crate::Wave,
    /// Drain current at the DC operating point.
    #[cfg_attr(feature = "serde", serde(default))]
    ids_bias: crate::Wave,
    /// Maximum Newton-Raphson iterations (bounded for RT safety).
    max_iter: usize,
}

impl MosfetRoot {
    pub fn new(model: MosfetModel) -> Self {
        Self {
            model,
            vgs: 0.0,
            vgs_bias: 0.0,
            vds_bias: 0.0,
            ids_bias: 0.0,
            max_iter: super::solver::NR_MAX_ITER,
        }
    }

    /// Set the gate-source voltage (external control from bias, signal, etc.)
    #[inline]
    pub fn set_vgs(&mut self, vgs: crate::Wave) {
        self.vgs = vgs;
    }

    /// Set the DC gate-source operating point and current Vgs.
    #[inline]
    pub fn set_bias(&mut self, vgs_bias: crate::Wave) {
        self.vgs_bias = vgs_bias;
        self.vgs = vgs_bias;
    }

    /// Set the DC operating point used by the incremental WDF one-port.
    #[inline]
    pub fn set_operating_point(&mut self, vgs_bias: crate::Wave, vds_bias: crate::Wave) {
        self.vgs_bias = vgs_bias;
        self.vgs = vgs_bias;
        self.vds_bias = vds_bias;
        self.ids_bias = self.drain_current(vds_bias);
    }

    /// DC gate-source operating point.
    #[inline]
    pub fn vgs_bias(&self) -> crate::Wave {
        self.vgs_bias
    }

    /// Get current gate-source voltage.
    #[inline]
    pub fn vgs(&self) -> crate::Wave {
        self.vgs
    }

    /// Compute drain current for given Vds at current Vgs.
    ///
    /// See [`MosfetModel::ids`] for the enhancement-mode square-law model.
    #[inline]
    pub fn drain_current(&self, vds: crate::Wave) -> crate::Wave {
        self.model.ids(self.vgs, vds)
    }

    /// Compute derivative of drain current w.r.t. Vds.
    #[inline]
    fn drain_current_derivative(&self, vds: crate::Wave) -> crate::Wave {
        self.model.ids_vds_derivative(self.vgs, vds)
    }

    #[inline]
    fn port_current(&self, vds_ac: crate::Wave) -> crate::Wave {
        self.drain_current(self.vds_bias + vds_ac) - self.ids_bias
    }

    #[inline]
    fn port_current_derivative(&self, vds_ac: crate::Wave) -> crate::Wave {
        self.drain_current_derivative(self.vds_bias + vds_ac)
    }
}

impl super::solver::NlDeviceIv for MosfetRoot {
    /// I(Vds) at current Vgs — drain-source as a 1-port nonlinear element.
    ///
    /// Mirrors the JFET/OTA pattern so the MOSFET can participate in
    /// multi-NL MNA solves once `NlDeviceKind` grows a `Mosfet` variant.
    #[inline]
    fn iv(&self, v: crate::Wave) -> (crate::Wave, crate::Wave) {
        (self.drain_current(v), self.drain_current_derivative(v))
    }

    #[inline]
    fn v_clamp(&self) -> (crate::Wave, crate::Wave) {
        (-20.0, 20.0)
    }
}

impl WdfRoot for MosfetRoot {
    /// MOSFET drain-source path: `i = Ids(Vds, Vgs)`
    #[inline]
    fn process(&mut self, a: crate::Wave, rp: crate::Wave) -> crate::Wave {
        let root = *self;
        let vov = if self.model.is_n_channel {
            (self.vgs - self.model.vth).max(0.0)
        } else {
            (self.model.vth - self.vgs).max(0.0)
        };
        let gds_approx = self
            .port_current_derivative(0.0)
            .abs()
            .max(self.model.kp * vov);
        let v0 = if gds_approx > LEAKAGE_CONDUCTANCE {
            a / (2.0 + 2.0 * rp * gds_approx)
        } else {
            a * 0.5
        };
        newton_raphson_solve(a, rp, v0, self.max_iter, 1e-6, None, None, |v| {
            (root.port_current(v), root.port_current_derivative(v))
        })
    }
}
