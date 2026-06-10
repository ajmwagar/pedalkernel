//! JFET (Junction Field-Effect Transistor) WDF root elements.
//!
//! Models N-channel and P-channel JFETs using standard SPICE JFET Level 1
//! equations with triode, saturation, and cutoff regions, plus gate junction
//! diode currents.
//!
//! Parameters are loaded from the embedded SPICE model file via `by_name()`.

use super::solver::{newton_raphson_solve, LEAKAGE_CONDUCTANCE};
use crate::elements::WdfRoot;

/// Thermal voltage at room temperature (26.85°C / 300K).
const VT: crate::Wave = 0.02585;

// ---------------------------------------------------------------------------
// JFET Models
// ---------------------------------------------------------------------------

/// JFET model parameters in standard SPICE Level 1 parameterization.
///
/// Uses VTO (threshold voltage) and BETA (transconductance coefficient)
/// directly from SPICE `.MODEL` statements rather than the derived Idss.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct JfetModel {
    /// Threshold (pinch-off) voltage (V). Negative for N-channel depletion mode.
    pub vto: crate::Wave,
    /// Transconductance coefficient (A/V²). Ids_sat = BETA × (Vgs - VTO)².
    pub beta: crate::Wave,
    /// Channel-length modulation parameter (1/V).
    pub lambda: crate::Wave,
    /// Gate junction saturation current (A).
    pub gate_is: crate::Wave,
    /// Gate junction ideality factor.
    pub n: crate::Wave,
    /// Drain ohmic resistance (Ω).
    pub rd: crate::Wave,
    /// Source ohmic resistance (Ω).
    pub rs: crate::Wave,
    /// Gate-source zero-bias junction capacitance (F).
    pub cgs: crate::Wave,
    /// Gate-drain zero-bias junction capacitance (F).
    pub cgd: crate::Wave,
    /// True for N-channel, false for P-channel.
    pub is_n_channel: bool,
}

impl JfetModel {
    // TODO: move to pedalkernel as extension impl
    // pub fn by_name(name: &str) -> Self { ... }
    // pub fn try_by_name(name: &str) -> Option<Self> { ... }

    /// Zero-gate-voltage drain saturation current: Idss = BETA × VTO².
    #[inline]
    pub fn idss(&self) -> crate::Wave {
        self.beta * self.vto * self.vto
    }

    /// Compute gate-source junction diode current.
    ///
    /// `Igs = IS × (exp(Vgs / (N × Vt)) - 1)`
    #[inline]
    pub fn gate_source_current(&self, vgs: crate::Wave) -> crate::Wave {
        let sign = if self.is_n_channel { 1.0 } else { -1.0 };
        let vgs_int = sign * vgs;
        // Limit exponent to avoid overflow
        let arg = (vgs_int / (self.n * VT)).min(40.0);
        sign * self.gate_is * (crate::math::exp(arg) - 1.0)
    }

    /// Compute gate-drain junction diode current.
    ///
    /// `Igd = IS × (exp(Vgd / (N × Vt)) - 1)`
    #[inline]
    pub fn gate_drain_current(&self, vgd: crate::Wave) -> crate::Wave {
        let sign = if self.is_n_channel { 1.0 } else { -1.0 };
        let vgd_int = sign * vgd;
        let arg = (vgd_int / (self.n * VT)).min(40.0);
        sign * self.gate_is * (crate::math::exp(arg) - 1.0)
    }

    /// Compute d(Igs)/d(Vgs): conductance of the gate-source junction diode.
    ///
    /// `dIgs/dVgs = IS / (N × Vt) × exp(Vgs / (N × Vt))`
    ///
    /// Returns the external-convention derivative (same sign convention as
    /// `gate_source_current`).
    #[inline]
    pub fn gate_source_conductance(&self, vgs: f64) -> f64 {
        let sign = if self.is_n_channel { 1.0 } else { -1.0 };
        let vgs_int = sign * vgs;
        let arg = (vgs_int / (self.n * VT)).min(40.0);
        // sign² = 1, so the external derivative equals the internal derivative
        self.gate_is / (self.n * VT) * crate::math::exp(arg)
    }
}

// TODO: move to pedalkernel as extension impl
// impl From<&SpiceJfetModel> for JfetModel { ... }

// ---------------------------------------------------------------------------
// JFET Root
// ---------------------------------------------------------------------------

/// JFET nonlinear root for WDF trees.
///
/// Models the drain-source path as a nonlinear element controlled by
/// an external gate-source voltage Vgs. Uses Newton-Raphson to solve
/// the implicit WDF constraint equation.
///
/// Drain-source current follows the SPICE Level 1 model:
///
/// **Triode** (`|Vds| < |Vgs - VTO|`):
///   `Ids = Beta × (2×(Vgs-VTO)×Vds - Vds²) × (1 + LAMBDA×|Vds|)`
///
/// **Saturation** (`|Vds| ≥ |Vgs - VTO|`):
///   `Ids = Beta × (Vgs-VTO)² × (1 + LAMBDA×|Vds|)`
///
/// **Cutoff** (`Vgs - VTO ≤ 0`):
///   `Ids = 0`
///
/// Uses SPICE sign convention: internal voltages are `sign × external`,
/// where `sign = +1` for N-channel and `-1` for P-channel.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct JfetRoot {
    pub model: JfetModel,
    /// Current gate-source voltage (external control parameter).
    vgs: crate::Wave,
    /// Maximum Newton-Raphson iterations (bounded for RT safety).
    max_iter: usize,
    /// Previous sample's drain voltage for warm-starting Newton-Raphson.
    prev_v: crate::Wave,
}

impl JfetRoot {
    pub fn new(model: JfetModel) -> Self {
        Self {
            model,
            vgs: 0.0,
            max_iter: super::solver::NR_MAX_ITER,
            prev_v: 0.0,
        }
    }

    /// Set the gate-source voltage (external control from LFO, bias, etc.)
    #[inline]
    pub fn set_vgs(&mut self, vgs: crate::Wave) {
        self.vgs = vgs;
    }

    /// Get current gate-source voltage.
    #[inline]
    pub fn vgs(&self) -> crate::Wave {
        self.vgs
    }

    /// Approximate drain-source resistance at Vds ≈ 0 (triode region onset).
    /// Rds = 1 / (2 × Beta × Vov), where Vov = Vgs - Vto.
    /// Returns a large resistance (1MΩ) if the JFET is in cutoff.
    #[inline]
    pub fn rds_approx(&self) -> crate::Wave {
        let sign = if self.model.is_n_channel { 1.0 } else { -1.0 };
        let vov = sign * self.vgs - self.model.vto;
        if vov > 0.0 {
            1.0 / (2.0 * self.model.beta * vov)
        } else {
            1e6
        }
    }

    /// Compute drain current for given Vds at current Vgs.
    ///
    /// Handles triode, saturation, and cutoff regions using SPICE sign
    /// convention for both N-channel and P-channel devices.
    #[inline]
    pub fn drain_current(&self, vds: crate::Wave) -> crate::Wave {
        let sign = if self.model.is_n_channel { 1.0 } else { -1.0 };
        let vgs_int = sign * self.vgs;
        let vds_int = sign * vds;
        let vto = self.model.vto;

        // Overdrive voltage
        let vov = vgs_int - vto;

        // Cutoff: no channel
        if vov <= 0.0 {
            return 0.0;
        }

        let ids_int = if vds_int.abs() < vov {
            // Triode region:
            // Ids = Beta × (2×vov×Vds - Vds²) × (1 + lambda×|Vds|)
            self.model.beta
                * (2.0 * vov * vds_int - vds_int * vds_int)
                * (1.0 + self.model.lambda * vds_int.abs())
        } else {
            // Saturation region:
            // Ids = Beta × vov² × (1 + lambda×|Vds|)
            self.model.beta * vov * vov * (1.0 + self.model.lambda * vds_int.abs())
        };

        // Convert back to external sign convention
        sign * ids_int
    }

    /// Compute derivative of drain current w.r.t. Vds.
    ///
    /// The external derivative dIds/dVds equals the internal derivative
    /// dIds_int/dVds_int because the two sign factors cancel.
    #[inline]
    fn drain_current_derivative(&self, vds: crate::Wave) -> crate::Wave {
        let sign = if self.model.is_n_channel { 1.0 } else { -1.0 };
        let vgs_int = sign * self.vgs;
        let vds_int = sign * vds;
        let vto = self.model.vto;

        let vov = vgs_int - vto;

        if vov <= 0.0 {
            return LEAKAGE_CONDUCTANCE;
        }

        if vds_int.abs() < vov {
            // Triode: product rule on f(v) = (2*vov*v - v²) * (1 + lambda*|v|)
            let f = 2.0 * vov * vds_int - vds_int * vds_int;
            let g = 1.0 + self.model.lambda * vds_int.abs();
            let df = 2.0 * vov - 2.0 * vds_int;
            let dg = self.model.lambda * if vds_int >= 0.0 { 1.0 } else { -1.0 };
            self.model.beta * (df * g + f * dg)
        } else {
            // Saturation: d/dv [vov² * (1 + lambda*|v|)] = vov² * lambda * sign(v)
            let dg = self.model.lambda * if vds_int >= 0.0 { 1.0 } else { -1.0 };
            self.model.beta * vov * vov * dg
        }
    }
}

impl super::solver::NlDeviceIv for JfetRoot {
    /// I(Vds) at current Vgs — drain-source as a 1-port nonlinear element.
    fn iv(&self, v: crate::Wave) -> (crate::Wave, crate::Wave) {
        (self.drain_current(v), self.drain_current_derivative(v))
    }

    fn v_clamp(&self) -> (crate::Wave, crate::Wave) {
        (-20.0, 20.0)
    }
}

impl JfetRoot {
    /// Source follower processing: solve for Vs where Ids(Vgate - Vs) = Vs / Rs.
    ///
    /// In a source follower:
    /// - The gate voltage (Vgate) is the input signal
    /// - The source voltage (Vs) follows the gate with ~unity gain
    /// - Vgs = Vgate - Vs (computed during Newton-Raphson solve)
    /// - The JFET operates in saturation: Ids ≈ Beta × (Vgs - VTO)²
    /// - The WDF constraint: Ids = Vs / Rp (source current into load)
    ///
    /// Returns the reflected wave b = 2*Vs - a.
    #[inline]
    pub fn process_source_follower(
        &mut self,
        a: crate::Wave,
        rp: crate::Wave,
        vgate: crate::Wave,
    ) -> crate::Wave {
        let model = self.model;
        let max_iter = self.max_iter;
        let sign = if model.is_n_channel { 1.0 } else { -1.0 };

        let source_follower_current = |vs: crate::Wave| -> (crate::Wave, crate::Wave) {
            let vgs = vgate - vs;
            let vgs_int = sign * vgs;
            let vov = vgs_int - model.vto;

            // Cutoff check — gate diode leakage still applies even in cutoff,
            // but it is physically negligible (< 1 nA) so we use LEAKAGE_CONDUCTANCE
            // as the slope rather than computing the full exponential.
            if vov <= 0.0 {
                return (0.0, LEAKAGE_CONDUCTANCE);
            }

            // Saturation current (assuming Vds >> Vdsat, typical for source follower)
            let ids_int = model.beta * vov * vov;
            let ids = sign * ids_int;

            // Gate-source junction diode current.
            //
            // In a source follower the gate is driven from a low-impedance source,
            // so forward gate conduction draws current that flows through the source
            // node. Total current into the WDF source port:
            //   I_source = Ids + Igs
            // The NR constraint becomes:
            //   f(Vs) = a - 2*Vs - 2*Rp*(Ids + Igs) = 0
            let igs = model.gate_source_current(vgs);

            // Derivatives w.r.t. Vs:
            //
            // dIds/dVs: Vgs = Vgate - Vs  →  dVgs/dVs = -1
            //   dIds_int/dVgs_int = 2 × Beta × vov,  dVgs_int/dVs = sign × (-1)
            //   dIds/dVs = sign × 2β·vov × sign × (-1) = -2β·vov
            //
            // dIgs/dVs: dIgs/dVgs × dVgs/dVs = gS × (-1)  where gS = dIgs/dVgs
            //   gS = gate_source_conductance(vgs) ≥ 0
            //   dIgs/dVs = -gS
            let gs = model.gate_source_conductance(vgs);
            let di_total_dvs = -2.0 * model.beta * vov - gs;

            // Current INTO port = -(Ids + Igs),  d/dVs = -di_total_dvs
            (-ids - igs, -di_total_dvs)
        };

        // Initial guess: source follows gate minus typical Vgs bias
        let v0 = (vgate + model.vto.abs() * 0.5).max(0.0);

        newton_raphson_solve(
            a,
            rp,
            v0,
            max_iter,
            1e-6,
            None,
            None,
            source_follower_current,
        )
    }
}

impl WdfRoot for JfetRoot {
    /// JFET drain-source path with warm-starting.
    #[inline]
    fn process(&mut self, a: crate::Wave, rp: crate::Wave) -> crate::Wave {
        let root = *self;

        let sign = if self.model.is_n_channel { 1.0 } else { -1.0 };
        let vgs_int = sign * self.vgs;
        let vov = vgs_int - self.model.vto;
        let gds_approx = if vov > 0.0 {
            2.0 * self.model.beta * vov
        } else {
            LEAKAGE_CONDUCTANCE
        };
        let cold = if gds_approx > LEAKAGE_CONDUCTANCE {
            a / (2.0 + 2.0 * rp * gds_approx)
        } else {
            a * 0.5
        };
        let v0 =
            if self.prev_v != 0.0 && (self.prev_v - cold).abs() < 50.0 && self.prev_v.abs() < 100.0
            {
                self.prev_v
            } else {
                cold
            };

        let b = newton_raphson_solve(a, rp, v0, self.max_iter, 1e-6, None, None, |v| {
            (root.drain_current(v), root.drain_current_derivative(v))
        });
        self.prev_v = (a + b) * 0.5;
        b
    }
}
