//! JFET (Junction Field-Effect Transistor) WDF root elements.
//!
//! Models N-channel and P-channel JFETs using standard SPICE JFET Level 1
//! equations with triode, saturation, and cutoff regions, plus gate junction
//! diode currents.
//!
//! Parameters are loaded from the embedded SPICE model file via `by_name()`.

use super::solver::{newton_raphson_solve, LEAKAGE_CONDUCTANCE};
use crate::elements::WdfRoot;
use crate::models::{jfet_by_name, SpiceJfetModel};

/// Thermal voltage at room temperature (26.85°C / 300K).
const VT: f64 = 0.02585;

// ---------------------------------------------------------------------------
// JFET Models
// ---------------------------------------------------------------------------

/// JFET model parameters in standard SPICE Level 1 parameterization.
///
/// Uses VTO (threshold voltage) and BETA (transconductance coefficient)
/// directly from SPICE `.MODEL` statements rather than the derived Idss.
#[derive(Debug, Clone, Copy)]
pub struct JfetModel {
    /// Threshold (pinch-off) voltage (V). Negative for N-channel depletion mode.
    pub vto: f64,
    /// Transconductance coefficient (A/V²). Ids_sat = BETA × (Vgs - VTO)².
    pub beta: f64,
    /// Channel-length modulation parameter (1/V).
    pub lambda: f64,
    /// Gate junction saturation current (A).
    pub gate_is: f64,
    /// Gate junction ideality factor.
    pub n: f64,
    /// Drain ohmic resistance (Ω).
    pub rd: f64,
    /// Source ohmic resistance (Ω).
    pub rs: f64,
    /// Gate-source zero-bias junction capacitance (F).
    pub cgs: f64,
    /// Gate-drain zero-bias junction capacitance (F).
    pub cgd: f64,
    /// True for N-channel, false for P-channel.
    pub is_n_channel: bool,
}

impl JfetModel {
    /// Look up a JFET model by name from the embedded SPICE model file.
    ///
    /// Panics if the model name is not found.
    pub fn by_name(name: &str) -> Self {
        Self::try_by_name(name)
            .unwrap_or_else(|| panic!("Unknown JFET model: '{name}'"))
    }

    /// Try to look up a JFET model by name. Returns `None` if not found.
    pub fn try_by_name(name: &str) -> Option<Self> {
        jfet_by_name(name).map(Self::from)
    }

    /// Zero-gate-voltage drain saturation current: Idss = BETA × VTO².
    #[inline]
    pub fn idss(&self) -> f64 {
        self.beta * self.vto * self.vto
    }

    /// Compute gate-source junction diode current.
    ///
    /// `Igs = IS × (exp(Vgs / (N × Vt)) - 1)`
    #[inline]
    pub fn gate_source_current(&self, vgs: f64) -> f64 {
        let sign = if self.is_n_channel { 1.0 } else { -1.0 };
        let vgs_int = sign * vgs;
        // Limit exponent to avoid overflow
        let arg = (vgs_int / (self.n * VT)).min(40.0);
        sign * self.gate_is * (arg.exp() - 1.0)
    }

    /// Compute gate-drain junction diode current.
    ///
    /// `Igd = IS × (exp(Vgd / (N × Vt)) - 1)`
    #[inline]
    pub fn gate_drain_current(&self, vgd: f64) -> f64 {
        let sign = if self.is_n_channel { 1.0 } else { -1.0 };
        let vgd_int = sign * vgd;
        let arg = (vgd_int / (self.n * VT)).min(40.0);
        sign * self.gate_is * (arg.exp() - 1.0)
    }
}

impl From<&SpiceJfetModel> for JfetModel {
    fn from(s: &SpiceJfetModel) -> Self {
        Self {
            vto: s.vto,
            beta: s.beta,
            lambda: s.lambda,
            gate_is: s.is,
            n: s.n,
            rd: s.rd,
            rs: s.rs,
            cgs: s.cgs,
            cgd: s.cgd,
            is_n_channel: s.is_n_channel,
        }
    }
}

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
pub struct JfetRoot {
    pub model: JfetModel,
    /// Current gate-source voltage (external control parameter).
    vgs: f64,
    /// Maximum Newton-Raphson iterations (bounded for RT safety).
    max_iter: usize,
}

impl JfetRoot {
    pub fn new(model: JfetModel) -> Self {
        Self {
            model,
            vgs: 0.0,
            max_iter: 16,
        }
    }

    /// Set the gate-source voltage (external control from LFO, bias, etc.)
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
    /// Handles triode, saturation, and cutoff regions using SPICE sign
    /// convention for both N-channel and P-channel devices.
    #[inline]
    pub fn drain_current(&self, vds: f64) -> f64 {
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
            self.model.beta
                * vov
                * vov
                * (1.0 + self.model.lambda * vds_int.abs())
        };

        // Convert back to external sign convention
        sign * ids_int
    }

    /// Compute derivative of drain current w.r.t. Vds.
    ///
    /// The external derivative dIds/dVds equals the internal derivative
    /// dIds_int/dVds_int because the two sign factors cancel.
    #[inline]
    fn drain_current_derivative(&self, vds: f64) -> f64 {
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
    pub fn process_source_follower(&mut self, a: f64, rp: f64, vgate: f64) -> f64 {
        let model = self.model;
        let max_iter = self.max_iter;
        let sign = if model.is_n_channel { 1.0 } else { -1.0 };

        let source_follower_current = |vs: f64| -> (f64, f64) {
            let vgs = vgate - vs;
            let vgs_int = sign * vgs;
            let vov = vgs_int - model.vto;

            // Cutoff check
            if vov <= 0.0 {
                return (0.0, LEAKAGE_CONDUCTANCE);
            }

            // Saturation current (assuming Vds >> Vdsat, typical for source follower)
            let ids_int = model.beta * vov * vov;
            let ids = sign * ids_int;

            // Derivative of Ids w.r.t. Vs:
            // dIds/dVs = dIds/dVgs × dVgs/dVs
            // dIds_int/dVgs_int = 2 × Beta × vov
            // dVgs_int/dVgs = sign, dVgs/dVs = -1
            // dIds/dVs = sign × 2 × Beta × vov × sign × (-1)
            //          = -2 × Beta × vov
            //
            // Current INTO port = -Ids (sign convention for WDF root)
            // d(-Ids)/dVs = -dIds/dVs = 2 × Beta × vov
            let di_dvs = 2.0 * model.beta * vov;

            (-ids, di_dvs)
        };

        // Initial guess: source follows gate minus typical Vgs bias
        let v0 = (vgate + model.vto.abs() * 0.5).max(0.0);

        newton_raphson_solve(a, rp, v0, max_iter, 1e-6, None, None, source_follower_current)
    }
}

impl WdfRoot for JfetRoot {
    /// JFET drain-source path: `i = Ids(v, Vgs)`
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
        let root = *self;
        let sign = if self.model.is_n_channel { 1.0 } else { -1.0 };
        let vgs_int = sign * self.vgs;
        let vov = vgs_int - self.model.vto;

        // Approximate conductance for initial guess
        let gds_approx = if vov > 0.0 {
            2.0 * self.model.beta * vov
        } else {
            LEAKAGE_CONDUCTANCE
        };

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
