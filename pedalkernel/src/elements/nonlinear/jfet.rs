//! JFET (Junction Field-Effect Transistor) WDF root elements.
//!
//! Models N-channel and P-channel JFETs with Shockley square-law drain current.

use super::solver::{newton_raphson_solve, LEAKAGE_CONDUCTANCE};
use crate::elements::WdfRoot;

// ---------------------------------------------------------------------------
// JFET Models
// ---------------------------------------------------------------------------

/// JFET model parameters for Shockley square-law drain current.
///
/// Models both N-channel and P-channel JFETs with triode and saturation
/// region behavior.
#[derive(Debug, Clone, Copy)]
pub struct JfetModel {
    /// Zero-gate-voltage drain current (A). Typical: 1-20 mA.
    pub idss: f64,
    /// Pinch-off voltage (V). N-channel: -0.5 to -6V, P-channel: +0.5 to +6V.
    pub vp: f64,
    /// Channel-length modulation parameter (1/V). Typical: 0.01-0.1.
    pub lambda: f64,
    /// True for N-channel, false for P-channel.
    pub is_n_channel: bool,
}

impl JfetModel {
    /// 2N5457 N-channel JFET (common, medium Idss).
    ///
    /// Parameters matched to SPICE model:
    /// - VTO = -2.0V (pinch-off voltage)
    /// - BETA = 1.5 mA/V² → Idss = BETA × Vp² = 6 mA
    /// - LAMBDA = 0.01 (channel-length modulation)
    pub fn n_2n5457() -> Self {
        Self {
            idss: 6e-3,     // SPICE: BETA=1.5m, Idss = 1.5e-3 * 2.0² = 6 mA
            vp: -2.0,       // SPICE: VTO=-2.0
            lambda: 0.01,   // SPICE: LAMBDA=0.01
            is_n_channel: true,
        }
    }

    /// J201 N-channel JFET (low Idss, low pinch-off).
    pub fn n_j201() -> Self {
        Self {
            idss: 1e-3,
            vp: -1.0,
            lambda: 0.01,
            is_n_channel: true,
        }
    }

    /// 2N5952 N-channel JFET (US, original MXR Phase 90).
    ///
    /// The 2N5952 is the JFET used in original 1974 MXR Phase 90 pedals.
    /// Similar characteristics to 2SK30A but slightly different specs.
    ///
    /// Datasheet specs:
    /// - Idss: 1.0 - 5.0 mA (modeled at 2.4 mA typical)
    /// - Vgs(off): -0.5V to -4.0V (modeled at -2.0V typical)
    pub fn n_2n5952() -> Self {
        Self {
            idss: 2.4e-3,
            vp: -2.0,
            lambda: 0.02,
            is_n_channel: true,
        }
    }

    /// 2SK30A N-channel JFET (Toshiba, classic phaser/modulation JFET).
    ///
    /// The 2SK30A is the classic choice for MXR Phase 90, Small Stone,
    /// and other vintage phasers. It has moderate Idss and a wider
    /// pinch-off range that makes it ideal for voltage-controlled
    /// resistance applications.
    ///
    /// Datasheet specs:
    /// - Idss: 0.6 - 6.5 mA (modeled at 2.6 mA typical)
    /// - Vgs(off): -0.4V to -5V (modeled at -1.8V typical)
    /// - Available in GR (low), Y (mid), BL (high) Idss grades
    pub fn n_2sk30a() -> Self {
        Self {
            idss: 2.6e-3,  // Typical mid-grade
            vp: -1.8,      // Typical pinch-off
            lambda: 0.015, // Moderate channel-length modulation
            is_n_channel: true,
        }
    }

    /// 2SK30A-GR (Green) - Low Idss grade (0.6-1.4 mA).
    pub fn n_2sk30a_gr() -> Self {
        Self {
            idss: 1.0e-3,
            vp: -1.2,
            lambda: 0.015,
            is_n_channel: true,
        }
    }

    /// 2SK30A-Y (Yellow) - Medium Idss grade (1.2-3.0 mA).
    pub fn n_2sk30a_y() -> Self {
        Self {
            idss: 2.0e-3,
            vp: -1.6,
            lambda: 0.015,
            is_n_channel: true,
        }
    }

    /// 2SK30A-BL (Blue) - High Idss grade (2.6-6.5 mA).
    pub fn n_2sk30a_bl() -> Self {
        Self {
            idss: 4.0e-3,
            vp: -2.2,
            lambda: 0.015,
            is_n_channel: true,
        }
    }

    /// 2N5460 P-channel JFET.
    pub fn p_2n5460() -> Self {
        Self {
            idss: 5e-3,
            vp: 2.5,
            lambda: 0.02,
            is_n_channel: false,
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
/// The drain-source current follows the Shockley square-law model:
/// - Triode: `Ids = Idss * [2*(1-Vgs/Vp)*(Vds/|Vp|) - (Vds/Vp)^2]`
/// - Saturation: `Ids = Idss * (1 - Vgs/Vp)^2 * (1 + lambda*|Vds|)`
///
/// Max iterations capped for real-time safety.
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
    /// Handles triode, saturation, and cutoff regions.
    #[inline]
    pub fn drain_current(&self, vds: f64) -> f64 {
        let vp = self.model.vp;
        let idss = self.model.idss;
        let lambda = self.model.lambda;
        let vgs = self.vgs;

        // Cutoff check: N-channel cuts off when Vgs < Vp (Vp is negative)
        //               P-channel cuts off when Vgs > Vp (Vp is positive)
        if self.model.is_n_channel {
            if vgs <= vp {
                return 0.0;
            }
        } else if vgs >= vp {
            return 0.0;
        }

        // Normalized gate factor: 0 at pinch-off, 1 at Vgs=0
        let vgs_factor = (1.0 - vgs / vp).clamp(0.0, 1.0);

        // Saturation voltage magnitude
        let vdsat = vp.abs() * vgs_factor;

        // Handle symmetric operation for both polarities
        let vds_abs = vds.abs();
        let vds_sign = vds.signum();

        let ids_magnitude = if vds_abs < vdsat {
            // Triode region: Ids = Idss * [2*(1-Vgs/Vp)*(Vds/|Vp|) - (Vds/Vp)^2]
            let vds_norm = vds_abs / vp.abs();
            idss * (2.0 * vgs_factor * vds_norm - vds_norm * vds_norm)
        } else {
            // Saturation region with channel-length modulation
            idss * vgs_factor * vgs_factor * (1.0 + lambda * vds_abs)
        };

        // Current direction follows Vds polarity
        ids_magnitude * vds_sign
    }

    /// Compute derivative of drain current w.r.t. Vds.
    #[inline]
    fn drain_current_derivative(&self, vds: f64) -> f64 {
        let vp = self.model.vp;
        let idss = self.model.idss;
        let lambda = self.model.lambda;
        let vgs = self.vgs;

        // Cutoff: very small conductance to avoid division issues
        if self.model.is_n_channel {
            if vgs <= vp {
                return LEAKAGE_CONDUCTANCE;
            }
        } else if vgs >= vp {
            return LEAKAGE_CONDUCTANCE;
        }

        let vgs_factor = (1.0 - vgs / vp).clamp(0.0, 1.0);
        let vdsat = vp.abs() * vgs_factor;
        let vds_abs = vds.abs();

        if vds_abs < vdsat {
            // Triode region: dIds/dVds = (2*Idss/|Vp|) * [(1-Vgs/Vp) - Vds/|Vp|]
            let vds_norm = vds_abs / vp.abs();
            (2.0 * idss / vp.abs()) * (vgs_factor - vds_norm)
        } else {
            // Saturation region: dIds/dVds = Idss * (1-Vgs/Vp)^2 * lambda
            idss * vgs_factor * vgs_factor * lambda
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
    /// - The JFET operates in saturation: Ids ≈ Idss * (1 - Vgs/Vp)^2
    /// - The WDF constraint: Ids = Vs / Rp (source current into load)
    ///
    /// Returns the reflected wave b = 2*Vs - a.
    #[inline]
    pub fn process_source_follower(&mut self, a: f64, rp: f64, vgate: f64) -> f64 {
        let model = self.model;
        let max_iter = self.max_iter;

        // Source follower current equation:
        // Ids = f(Vgs) = f(Vgate - Vs)
        // WDF constraint: Ids = (a - b) / (2*Rp) = (a - v) / Rp where v = (a+b)/2 = Vs
        // So: Ids(Vgate - Vs) = (a - Vs) / Rp

        // Source follower current equation:
        // WDF constraint: I_port = (a - b) / (2*Rp) where I_port is current INTO the port
        // For source follower: current flows OUT of source (into Rs), so I_port = -Ids
        //
        // Ids = Idss * (1 - Vgs/Vp)^2 in saturation
        // Vgs = Vgate - Vs (computed during solve)
        let source_follower_current = |vs: f64| -> (f64, f64) {
            let vgs = vgate - vs;

            // Cutoff check
            if model.is_n_channel {
                if vgs <= model.vp {
                    // Cutoff: no current, but small leakage for numerical stability
                    return (0.0, LEAKAGE_CONDUCTANCE);
                }
            } else if vgs >= model.vp {
                return (0.0, LEAKAGE_CONDUCTANCE);
            }

            let vgs_factor = (1.0 - vgs / model.vp).clamp(0.0, 1.0);

            // Saturation current (assuming Vds >> Vdsat, typical for source follower)
            let ids = model.idss * vgs_factor * vgs_factor;

            // Derivative of Ids w.r.t. Vs:
            // dIds/dVs = dIds/dVgs * dVgs/dVs
            //
            // dIds/dVgs: For Ids = Idss * (1 - Vgs/Vp)^2
            //   = 2 * Idss * (1 - Vgs/Vp) * (-1/Vp)
            //   = -2 * Idss / Vp * vgs_factor
            //   For N-channel (Vp < 0): = -2 * Idss / (-|Vp|) * vgs_factor = 2*Idss/|Vp| * vgs_factor > 0
            //
            // dVgs/dVs = -1 (Vgs = Vgate - Vs)
            //
            // dIds/dVs = dIds/dVgs * dVgs/dVs = (2*Idss/|Vp| * vgs_factor) * (-1) < 0
            //
            // Current INTO port = -Ids
            // d(I_port)/dVs = -dIds/dVs = -(negative) = positive
            // Derivative chain:
            // dIds/dVgs = 2 * Idss * vgs_factor / |Vp| (positive for N-channel)
            // dVgs/dVs = -1 (Vgs = Vgate - Vs)
            // dIds/dVs = dIds/dVgs * (-1) = -2 * Idss * vgs_factor / |Vp| (negative)
            //
            // i = -Ids, so di/dVs = -dIds/dVs (positive)
            let d_ids_dvgs = 2.0 * model.idss / model.vp.abs() * vgs_factor;
            let d_ids_dvs = -d_ids_dvgs;  // negative
            let di_dvs = -d_ids_dvs;      // positive (i = -Ids)

            // Return (i, di/dVs) where i = -Ids
            (-ids, di_dvs)
        };

        // Initial guess: source follows gate minus typical Vgs
        // For N-channel at moderate bias, Vgs ≈ -Vp/2
        let v0 = (vgate + model.vp.abs() * 0.5).max(0.0);

        newton_raphson_solve(a, rp, v0, max_iter, 1e-6, None, None, source_follower_current)
    }
}

impl WdfRoot for JfetRoot {
    /// JFET drain-source path: `i = Ids(v, Vgs)`
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
        let root = *self;
        let vgs_factor = (1.0 - self.vgs / self.model.vp).clamp(0.0, 1.0);
        let gds_approx = (2.0 * self.model.idss / self.model.vp.abs()) * vgs_factor;
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
