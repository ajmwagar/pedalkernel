//! Nonlinear WDF root elements: Diodes, JFETs, MOSFETs, Zener diodes.
//!
//! These elements sit at the tree root and use Newton-Raphson iteration
//! to solve the implicit WDF constraint equation for the reflected wave.

use super::WdfRoot;

// ---------------------------------------------------------------------------
// Diode Models
// ---------------------------------------------------------------------------

/// Diode model parameters derived from the Shockley equation.
#[derive(Debug, Clone, Copy)]
pub struct DiodeModel {
    /// Saturation current (A). Silicon ≈ 1e-15, Germanium ≈ 1e-6.
    pub is: f64,
    /// Thermal voltage * ideality factor (V). Vt ≈ 25.85 mV at 20 °C.
    pub n_vt: f64,
}

impl DiodeModel {
    pub fn silicon() -> Self {
        Self {
            is: 2.52e-9,
            n_vt: 1.752 * 25.85e-3,
        }
    }

    pub fn germanium() -> Self {
        Self {
            is: 1e-6,
            n_vt: 1.3 * 25.85e-3,
        }
    }

    pub fn led() -> Self {
        Self {
            is: 2.96e-12,
            n_vt: 1.9 * 25.85e-3,
        }
    }
}

// ---------------------------------------------------------------------------
// Diode Pair Root
// ---------------------------------------------------------------------------

/// Anti-parallel diode pair at the tree root.
///
/// Solves for the reflected wave `b` given the incident wave `a` and
/// port resistance `Rp` using Newton-Raphson on the implicit equation
/// derived from the Shockley model of two anti-parallel diodes.
///
/// Max iterations capped for real-time safety.
#[derive(Debug, Clone, Copy)]
pub struct DiodePairRoot {
    pub model: DiodeModel,
    max_iter: usize,
}

impl DiodePairRoot {
    pub fn new(model: DiodeModel) -> Self {
        Self {
            model,
            max_iter: 16,
        }
    }
}

impl WdfRoot for DiodePairRoot {
    /// Compute reflected wave from incident wave `a` and port resistance `rp`.
    ///
    /// The voltage across the diode pair is `v = (a + b) / 2`.
    /// Current through the pair: `i_d(v) = Is*(exp(v/nVt) - exp(-v/nVt))`.
    /// The WDF constraint: `b = a - 2*Rp*i_d(v)`.
    ///
    /// We solve for `v` using Newton-Raphson on:
    ///   `f(v) = a - 2*v - 2*Rp * Is * (exp(v/nVt) - exp(-v/nVt))`
    ///   `f'(v) = -2 - 2*Rp * Is / nVt * (exp(v/nVt) + exp(-v/nVt))`
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
        let is = self.model.is;
        let nvt = self.model.n_vt;

        // Initial guess: for large |a| the diode is saturated and we can
        // estimate v from the Shockley equation in the forward-bias regime
        // (2*Rp*Is*exp(|v|/nVt) ≈ |a|). For small |a| the linear
        // approximation v ≈ a/2 is accurate.
        let mut v = if a.abs() > 10.0 * nvt {
            let log_arg = (a.abs() / (2.0 * rp * is)).max(1.0);
            nvt * log_arg.ln() * a.signum()
        } else {
            a * 0.5
        };

        for _ in 0..self.max_iter {
            let x = (v / nvt).clamp(-500.0, 500.0);
            let ev_pos = x.exp();
            let ev_neg = (-x).exp();
            let sinh_term = is * (ev_pos - ev_neg);
            let cosh_term = is * (ev_pos + ev_neg) / nvt;

            let f = a - 2.0 * v - 2.0 * rp * sinh_term;
            let fp = -2.0 - 2.0 * rp * cosh_term;

            let dv = f / fp;
            v -= dv;

            if dv.abs() < 1e-6 {
                break;
            }
        }

        2.0 * v - a
    }
}

// ---------------------------------------------------------------------------
// Single Diode Root
// ---------------------------------------------------------------------------

/// Single diode at the tree root (asymmetric clipping).
///
/// Solves `i_d(v) = Is*(exp(v/nVt) - 1)` via Newton-Raphson.
#[derive(Debug, Clone, Copy)]
pub struct DiodeRoot {
    pub model: DiodeModel,
    max_iter: usize,
}

impl DiodeRoot {
    pub fn new(model: DiodeModel) -> Self {
        Self {
            model,
            max_iter: 16,
        }
    }
}

impl WdfRoot for DiodeRoot {
    /// `f(v) = a - 2v - 2*Rp*Is*(exp(v/nVt) - 1)`
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
        let is = self.model.is;
        let nvt = self.model.n_vt;

        // Same analytical initial guess as DiodePairRoot.
        let mut v = if a.abs() > 10.0 * nvt {
            let log_arg = (a.abs() / (2.0 * rp * is)).max(1.0);
            nvt * log_arg.ln() * a.signum()
        } else {
            a * 0.5
        };

        for _ in 0..self.max_iter {
            let x = (v / nvt).clamp(-500.0, 500.0);
            let ev = x.exp();
            let i_d = is * (ev - 1.0);
            let di_d = is * ev / nvt;

            let f = a - 2.0 * v - 2.0 * rp * i_d;
            let fp = -2.0 - 2.0 * rp * di_d;

            let dv = f / fp;
            v -= dv;

            if dv.abs() < 1e-6 {
                break;
            }
        }

        2.0 * v - a
    }
}

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
    pub fn n_2n5457() -> Self {
        Self {
            idss: 5e-3,
            vp: -2.5,
            lambda: 0.02,
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
                return 1e-12;
            }
        } else if vgs >= vp {
            return 1e-12;
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

impl WdfRoot for JfetRoot {
    /// Compute reflected wave from incident wave `a` and port resistance `rp`.
    ///
    /// WDF constraint: `v = (a + b) / 2`, `i = (a - b) / (2 * Rp)`
    /// JFET: `i = Ids(v, Vgs)`
    ///
    /// Solve: `f(v) = a - 2*v - 2*Rp*Ids(v, Vgs) = 0`
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
        // Initial guess: linear approximation using small-signal conductance
        let vgs_factor = (1.0 - self.vgs / self.model.vp).clamp(0.0, 1.0);
        let gds_approx = (2.0 * self.model.idss / self.model.vp.abs()) * vgs_factor;
        let mut v = if gds_approx > 1e-12 {
            a / (2.0 + 2.0 * rp * gds_approx)
        } else {
            a * 0.5
        };

        for _ in 0..self.max_iter {
            let ids = self.drain_current(v);
            let dids = self.drain_current_derivative(v);

            let f = a - 2.0 * v - 2.0 * rp * ids;
            let fp = -2.0 - 2.0 * rp * dids;

            // Avoid division by zero
            if fp.abs() < 1e-15 {
                break;
            }

            let dv = f / fp;
            v -= dv;

            if dv.abs() < 1e-6 {
                break;
            }
        }

        2.0 * v - a
    }
}

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
    /// Maximum Newton-Raphson iterations (bounded for RT safety).
    max_iter: usize,
}

impl TriodeRoot {
    pub fn new(model: TriodeModel) -> Self {
        Self {
            model,
            vgk: 0.0,
            max_iter: 16,
        }
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

        // Cutoff: if E1 is very negative, exp(E1) ≈ 0, so ln(1+exp(E1)) ≈ 0
        // Use soft cutoff to avoid numerical issues
        if e1 < -50.0 {
            return 0.0;
        }

        // ln(1 + exp(x)) with numerical stability
        let ln_term = if e1 > 50.0 {
            e1 // For large x, ln(1 + exp(x)) ≈ x
        } else {
            (1.0 + e1.exp()).ln()
        };

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
            return 1e-12; // Small conductance to avoid division issues
        }

        let vpk_sq = vpk * vpk;
        let sqrt_term = (kvb + vpk_sq).sqrt();
        let e1 = kp * (1.0 / mu + vgk / sqrt_term);

        if e1 < -50.0 {
            return 1e-12;
        }

        // Compute ln(1 + exp(E1)) and its derivative
        let exp_e1 = if e1 > 50.0 { f64::MAX / 2.0 } else { e1.exp() };
        let one_plus_exp = 1.0 + exp_e1;
        let ln_term = if e1 > 50.0 { e1 } else { one_plus_exp.ln() };

        let base = (vpk / kp) * ln_term;
        if base <= 0.0 {
            return 1e-12;
        }

        // dE1/dVpk = Kp * Vgk * (-Vpk) / (Kvb + Vpk^2)^(3/2)
        let de1_dvpk = -kp * vgk * vpk / (sqrt_term * sqrt_term * sqrt_term);

        // d(ln(1+exp(E1)))/dVpk = exp(E1)/(1+exp(E1)) * dE1/dVpk
        let dln_dvpk = (exp_e1 / one_plus_exp) * de1_dvpk;

        // d(Vpk/Kp * ln_term)/dVpk = ln_term/Kp + (Vpk/Kp) * dln_dvpk
        let dbase_dvpk = ln_term / kp + (vpk / kp) * dln_dvpk;

        // d(base^Ex)/dVpk = Ex * base^(Ex-1) * dbase_dvpk
        ex * base.powf(ex - 1.0) * dbase_dvpk
    }
}

impl WdfRoot for TriodeRoot {
    /// Compute reflected wave from incident wave `a` and port resistance `rp`.
    ///
    /// WDF constraint: `v = (a + b) / 2`, `i = (a - b) / (2 * Rp)`
    /// Triode: `i = Ip(v, Vgk)`
    ///
    /// Solve: `f(v) = a - 2*v - 2*Rp*Ip(v, Vgk) = 0`
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
        // Initial guess: assume small signal, linear approximation
        // For a tube, typical plate resistance is on order of model.kp * 1000
        let mut v = a * 0.5;

        for _ in 0..self.max_iter {
            let ip = self.plate_current(v);
            let dip = self.plate_current_derivative(v);

            let f = a - 2.0 * v - 2.0 * rp * ip;
            let fp = -2.0 - 2.0 * rp * dip;

            // Avoid division by zero
            if fp.abs() < 1e-15 {
                break;
            }

            let dv = f / fp;
            v -= dv;

            // Clamp to reasonable range to prevent runaway
            v = v.clamp(-1000.0, 1000.0);

            if dv.abs() < 1e-6 {
                break;
            }
        }

        2.0 * v - a
    }
}

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
                return 1e-12;
            }
        } else if vgs >= vth {
            return 1e-12;
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
    /// Compute reflected wave from incident wave `a` and port resistance `rp`.
    ///
    /// WDF constraint: `v = (a + b) / 2`, `i = (a - b) / (2 * Rp)`
    /// MOSFET: `i = Ids(v, Vgs)`
    ///
    /// Solve: `f(v) = a - 2*v - 2*Rp*Ids(v, Vgs) = 0`
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
        // Initial guess: linear approximation
        let vov = if self.model.is_n_channel {
            (self.vgs - self.model.vth).max(0.0)
        } else {
            (self.model.vth - self.vgs).max(0.0)
        };
        let gds_approx = 2.0 * self.model.kp * vov;
        let mut v = if gds_approx > 1e-12 {
            a / (2.0 + 2.0 * rp * gds_approx)
        } else {
            a * 0.5
        };

        for _ in 0..self.max_iter {
            let ids = self.drain_current(v);
            let dids = self.drain_current_derivative(v);

            let f = a - 2.0 * v - 2.0 * rp * ids;
            let fp = -2.0 - 2.0 * rp * dids;

            if fp.abs() < 1e-15 {
                break;
            }

            let dv = f / fp;
            v -= dv;

            if dv.abs() < 1e-6 {
                break;
            }
        }

        2.0 * v - a
    }
}

// ---------------------------------------------------------------------------
// Zener Diode Model
// ---------------------------------------------------------------------------

/// Zener diode model with forward and reverse breakdown behavior.
///
/// Forward bias: standard Shockley equation (same as regular diode).
/// Reverse bias: sharp exponential breakdown at the Zener voltage Vz.
///
/// Used in voltage regulation circuits and some clipping configurations
/// in guitar pedals.
#[derive(Debug, Clone, Copy)]
pub struct ZenerModel {
    /// Zener breakdown voltage (V). Always positive.
    pub vz: f64,
    /// Forward saturation current (A).
    pub is_fwd: f64,
    /// Forward thermal voltage * ideality factor (V).
    pub n_vt_fwd: f64,
    /// Reverse breakdown sharpness (A). Controls how sharp the knee is.
    pub is_rev: f64,
    /// Reverse thermal voltage * ideality factor (V). Smaller = sharper knee.
    pub n_vt_rev: f64,
}

impl ZenerModel {
    /// Create a Zener diode model for a given breakdown voltage.
    ///
    /// The forward characteristics are similar to a standard silicon diode.
    /// The reverse characteristics model the Zener/avalanche breakdown.
    pub fn new(vz: f64) -> Self {
        Self {
            vz,
            is_fwd: 2.52e-9,           // Same as standard silicon
            n_vt_fwd: 1.752 * 25.85e-3, // ~45mV
            is_rev: 1e-12,              // Very small reverse leakage
            n_vt_rev: 0.5 * 25.85e-3,   // Sharp knee (~13mV)
        }
    }
}

// ---------------------------------------------------------------------------
// Zener Diode Root
// ---------------------------------------------------------------------------

/// Zener diode at the WDF tree root.
///
/// Models both forward conduction (standard diode behavior) and
/// reverse breakdown at the Zener voltage Vz.
///
/// Current model:
/// - Forward: `i = Is_fwd * (exp(v / nVt_fwd) - 1)`
/// - Reverse: `i = -Is_rev * (exp((-v - Vz) / nVt_rev) - 1)`
///
/// Total: `i(v) = i_fwd(v) + i_rev(v)`
#[derive(Debug, Clone, Copy)]
pub struct ZenerRoot {
    pub model: ZenerModel,
    max_iter: usize,
}

impl ZenerRoot {
    pub fn new(model: ZenerModel) -> Self {
        Self {
            model,
            max_iter: 16,
        }
    }

    /// Total current through the Zener diode.
    #[inline]
    pub fn current(&self, v: f64) -> f64 {
        let m = &self.model;

        // Forward bias current (standard Shockley)
        let x_fwd = (v / m.n_vt_fwd).clamp(-500.0, 500.0);
        let i_fwd = m.is_fwd * (x_fwd.exp() - 1.0);

        // Reverse breakdown current
        let x_rev = ((-v - m.vz) / m.n_vt_rev).clamp(-500.0, 500.0);
        let i_rev = -m.is_rev * (x_rev.exp() - 1.0);

        i_fwd + i_rev
    }

    /// Derivative of total current w.r.t. voltage.
    #[inline]
    fn current_derivative(&self, v: f64) -> f64 {
        let m = &self.model;

        // Forward: di_fwd/dv = Is_fwd / nVt_fwd * exp(v / nVt_fwd)
        let x_fwd = (v / m.n_vt_fwd).clamp(-500.0, 500.0);
        let di_fwd = m.is_fwd * x_fwd.exp() / m.n_vt_fwd;

        // Reverse: di_rev/dv = Is_rev / nVt_rev * exp((-v - Vz) / nVt_rev)
        let x_rev = ((-v - m.vz) / m.n_vt_rev).clamp(-500.0, 500.0);
        let di_rev = m.is_rev * x_rev.exp() / m.n_vt_rev;

        di_fwd + di_rev
    }
}

impl WdfRoot for ZenerRoot {
    /// Solve the WDF constraint for a Zener diode.
    ///
    /// `f(v) = a - 2v - 2*Rp*i(v) = 0`
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
        let nvt = self.model.n_vt_fwd;

        let mut v = if a.abs() > 10.0 * nvt {
            let log_arg = (a.abs() / (2.0 * rp * self.model.is_fwd)).max(1.0);
            nvt * log_arg.ln() * a.signum()
        } else {
            a * 0.5
        };

        for _ in 0..self.max_iter {
            let i = self.current(v);
            let di = self.current_derivative(v);

            let f = a - 2.0 * v - 2.0 * rp * i;
            let fp = -2.0 - 2.0 * rp * di;

            if fp.abs() < 1e-15 {
                break;
            }

            let dv = f / fp;
            v -= dv;

            if dv.abs() < 1e-6 {
                break;
            }
        }

        2.0 * v - a
    }
}
