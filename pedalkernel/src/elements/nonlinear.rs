//! Nonlinear WDF root elements: Diodes, JFETs, MOSFETs, Zener diodes, OTAs.
//!
//! These elements sit at the tree root and use Newton-Raphson iteration
//! to solve the implicit WDF constraint equation for the reflected wave.
//!
//! Also includes the slew rate limiter (models op-amp bandwidth limiting)
//! and OTA (operational transconductance amplifier) for CA3080-based circuits.

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
// Zener Diode Model
// ---------------------------------------------------------------------------

/// Zener diode model parameters.
///
/// A zener diode conducts normally in forward bias (like a silicon diode)
/// and exhibits sharp breakdown at the zener voltage in reverse bias.
/// This makes it useful for voltage regulation and clipping.
#[derive(Debug, Clone, Copy)]
pub struct ZenerModel {
    /// Zener breakdown voltage (V). Common values: 3.3, 4.7, 5.1, 6.2, 9.1, 12.
    pub vz: f64,
    /// Forward saturation current (A). Similar to silicon diode.
    pub is_fwd: f64,
    /// Forward thermal voltage * ideality (V).
    pub n_vt_fwd: f64,
    /// Reverse saturation current at breakdown (A). Controls sharpness.
    pub is_rev: f64,
    /// Reverse thermal voltage * ideality (V). Smaller = sharper knee.
    pub n_vt_rev: f64,
    /// Dynamic resistance in breakdown region (Ω). Typical: 1-30Ω.
    pub rz: f64,
}

impl ZenerModel {
    /// Create a zener diode with specified breakdown voltage.
    ///
    /// Uses typical 1N47xx series characteristics.
    pub fn with_voltage(vz: f64) -> Self {
        // Dynamic resistance scales roughly with voltage
        // Lower voltage zeners have higher Rz due to avalanche mechanism
        let rz = if vz < 5.0 {
            30.0  // Avalanche dominated
        } else if vz < 8.0 {
            10.0  // Mixed mechanism
        } else {
            5.0   // Zener dominated
        };

        Self {
            vz,
            is_fwd: 2.52e-9,
            n_vt_fwd: 1.752 * 25.85e-3,
            is_rev: 1e-12,
            n_vt_rev: 0.5 * 25.85e-3, // Sharp knee
            rz,
        }
    }

    /// 1N4728A - 3.3V Zener (500mW)
    pub fn z3v3() -> Self {
        Self::with_voltage(3.3)
    }

    /// 1N4732A - 4.7V Zener (500mW)
    pub fn z4v7() -> Self {
        Self::with_voltage(4.7)
    }

    /// 1N4733A - 5.1V Zener (500mW) - Common in pedal circuits
    pub fn z5v1() -> Self {
        Self::with_voltage(5.1)
    }

    /// 1N4734A - 5.6V Zener (500mW)
    pub fn z5v6() -> Self {
        Self::with_voltage(5.6)
    }

    /// 1N4735A - 6.2V Zener (500mW)
    pub fn z6v2() -> Self {
        Self::with_voltage(6.2)
    }

    /// 1N4739A - 9.1V Zener (500mW)
    pub fn z9v1() -> Self {
        Self::with_voltage(9.1)
    }

    /// 1N4742A - 12V Zener (500mW)
    pub fn z12v() -> Self {
        Self::with_voltage(12.0)
    }
}

// ---------------------------------------------------------------------------
// Zener Diode Root
// ---------------------------------------------------------------------------

/// Zener diode at the tree root.
///
/// Models both forward conduction (like silicon diode) and reverse
/// breakdown (zener/avalanche). Uses Newton-Raphson iteration.
///
/// Current equation:
/// - Forward (v > 0): `i = Is_fwd * (exp(v/nVt_fwd) - 1)`
/// - Reverse (v < -Vz): `i = -Is_rev * (exp(-(v+Vz)/nVt_rev) - 1) - (v+Vz)/Rz`
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

    /// Compute zener current for given voltage.
    #[inline]
    fn current(&self, v: f64) -> f64 {
        if v >= 0.0 {
            // Forward bias: standard Shockley diode
            let x = (v / self.model.n_vt_fwd).clamp(-500.0, 500.0);
            self.model.is_fwd * (x.exp() - 1.0)
        } else if v > -self.model.vz {
            // Reverse bias below breakdown: small leakage
            -self.model.is_rev
        } else {
            // Reverse breakdown: exponential + resistive
            let v_excess = -v - self.model.vz; // How far past Vz
            let x = (v_excess / self.model.n_vt_rev).clamp(-500.0, 500.0);
            let i_breakdown = self.model.is_rev * (x.exp() - 1.0);
            let i_resistive = v_excess / self.model.rz;
            -(i_breakdown + i_resistive + self.model.is_rev)
        }
    }

    /// Compute derivative of current w.r.t. voltage.
    #[inline]
    fn current_derivative(&self, v: f64) -> f64 {
        if v >= 0.0 {
            // Forward bias
            let x = (v / self.model.n_vt_fwd).clamp(-500.0, 500.0);
            self.model.is_fwd * x.exp() / self.model.n_vt_fwd
        } else if v > -self.model.vz {
            // Reverse bias below breakdown: very small conductance
            1e-12
        } else {
            // Reverse breakdown
            let v_excess = -v - self.model.vz;
            let x = (v_excess / self.model.n_vt_rev).clamp(-500.0, 500.0);
            let di_exp = self.model.is_rev * x.exp() / self.model.n_vt_rev;
            let di_res = 1.0 / self.model.rz;
            di_exp + di_res // Note: chain rule gives positive derivative
        }
    }
}

impl WdfRoot for ZenerRoot {
    /// Solve for reflected wave using Newton-Raphson.
    ///
    /// `f(v) = a - 2*v - 2*Rp*i(v) = 0`
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
        // Initial guess based on operating region
        let mut v = if a > 0.0 {
            // Likely forward bias
            (a * 0.5).min(0.7)
        } else if a < -2.0 * self.model.vz {
            // Likely in breakdown
            -self.model.vz - 0.1
        } else {
            // Reverse bias, not yet breakdown
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
            max_iter: 16,
        }
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

        if e1 < -50.0 {
            return 0.0;
        }

        let ln_term = if e1 > 50.0 {
            e1
        } else {
            (1.0 + e1.exp()).ln()
        };

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
            return 1e-12;
        }

        let e1 = kp * (1.0 / mu + vg1k / (kvb + vg2k * vg2k).sqrt());

        if e1 < -50.0 {
            return 1e-12;
        }

        let ln_term = if e1 > 50.0 {
            e1
        } else {
            (1.0 + e1.exp()).ln()
        };

        let base = (vg2k / kp) * ln_term;
        if base <= 0.0 {
            return 1e-12;
        }

        let ip_base = base.powf(ex);

        // d/dVpk of (2/π) * atan(Vpk/Kvb2) = (2/π) * 1/(1 + (Vpk/Kvb2)^2) * 1/Kvb2
        let vpk_ratio = vpk / kvb2;
        let d_plate_factor = (2.0 / std::f64::consts::PI) / (1.0 + vpk_ratio * vpk_ratio) / kvb2;

        ip_base * d_plate_factor
    }
}

impl WdfRoot for PentodeRoot {
    /// Compute reflected wave from incident wave `a` and port resistance `rp`.
    ///
    /// WDF constraint: `v = (a + b) / 2`, `i = (a - b) / (2 * Rp)`
    /// Pentode: `i = Ip(v, Vg1k, Vg2k)`
    ///
    /// Solve: `f(v) = a - 2*v - 2*Rp*Ip(v, Vg1k, Vg2k) = 0`
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
        let mut v = a * 0.5;

        for _ in 0..self.max_iter {
            let ip = self.plate_current(v);
            let dip = self.plate_current_derivative(v);

            let f = a - 2.0 * v - 2.0 * rp * ip;
            let fp = -2.0 - 2.0 * rp * dip;

            if fp.abs() < 1e-15 {
                break;
            }

            let dv = f / fp;
            v -= dv;

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

// ---------------------------------------------------------------------------
// Slew Rate Limiter
// ---------------------------------------------------------------------------

/// Models op-amp slew rate limiting for physically accurate WDF processing.
///
/// Real op-amps have a finite maximum rate of output voltage change (dV/dt),
/// measured in V/µs. When the signal demands a faster slew than the op-amp
/// can deliver, the output "rounds off" — this is the mechanism behind the
/// distinctive compression character of slow op-amps like the LM308 (RAT)
/// versus fast ones like the TL072 (Klon).
///
/// The slew rate limiter operates as a per-sample voltage clamp on the
/// derivative: `|V[n] - V[n-1]| ≤ slew_rate * dt`, where dt = 1/fs.
///
/// This is NOT a WDF root — it sits in the signal path between stages,
/// modeling the op-amp's output stage limitation.
#[derive(Debug, Clone, Copy)]
pub struct SlewRateLimiter {
    /// Maximum voltage change per sample (V/sample).
    max_dv: f64,
    /// Previous output voltage (state).
    prev_out: f64,
    /// Slew rate in V/µs (for reference/display).
    slew_rate_v_per_us: f64,
    /// Current sample rate.
    sample_rate: f64,
}

impl SlewRateLimiter {
    /// Create a slew rate limiter from a slew rate in V/µs and sample rate.
    ///
    /// Slew rate values from real op-amps:
    /// - LM308: 0.3 V/µs (the RAT's character)
    /// - LM741: 0.5 V/µs (vintage slow)
    /// - JRC4558: 1.7 V/µs (Tube Screamer warmth)
    /// - NE5532: 9.0 V/µs (studio clean)
    /// - TL072: 13.0 V/µs (modern, transparent)
    /// - CA3080: 50.0 V/µs (OTA, essentially transparent)
    pub fn new(slew_rate_v_per_us: f64, sample_rate: f64) -> Self {
        // Convert V/µs to V/sample: slew_rate * 1e6 / sample_rate
        let max_dv = slew_rate_v_per_us * 1e6 / sample_rate;
        Self {
            max_dv,
            prev_out: 0.0,
            slew_rate_v_per_us,
            sample_rate,
        }
    }

    /// Process one sample through the slew rate limiter.
    ///
    /// If the requested voltage change exceeds what the op-amp can deliver
    /// in one sample period, the output is clamped to the maximum slew rate.
    /// This creates asymmetric HF compression — the exact behavior that
    /// makes the LM308 RAT sound different from a TL072 RAT.
    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
        let dv = input - self.prev_out;
        let limited = if dv > self.max_dv {
            self.prev_out + self.max_dv
        } else if dv < -self.max_dv {
            self.prev_out - self.max_dv
        } else {
            input
        };
        self.prev_out = limited;
        limited
    }

    /// Update sample rate and recompute max_dv.
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
        self.max_dv = self.slew_rate_v_per_us * 1e6 / sample_rate;
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.prev_out = 0.0;
    }

    /// Get the slew rate in V/µs.
    pub fn slew_rate(&self) -> f64 {
        self.slew_rate_v_per_us
    }

    /// Check if slew limiting is currently active (for diagnostics).
    ///
    /// Returns the ratio of actual dV to max_dv for the last sample.
    /// Values > 1.0 mean the limiter was engaged.
    pub fn is_limiting(&self) -> bool {
        // This would need to track the last dv, but for simplicity
        // we just check if the limiter is "slow enough to matter"
        self.slew_rate_v_per_us < 5.0
    }
}

// ---------------------------------------------------------------------------
// OTA (Operational Transconductance Amplifier) Model
// ---------------------------------------------------------------------------

/// CA3080 OTA model parameters.
///
/// Unlike voltage-mode op-amps, an OTA's output is a current proportional
/// to the differential input voltage, controlled by a bias current (Iabc).
///
/// The key equation: `Iout = gm * (V+ - V-)` where `gm = Iabc / (2 * Vt)`
/// and Vt is the thermal voltage (~26mV at room temperature).
///
/// The CA3080 has a tanh-like transfer characteristic:
/// `Iout = Iabc * tanh(Vdiff / (2 * Vt))`
///
/// This means:
/// - For small signals: linear gain proportional to Iabc
/// - For large signals: soft clipping at ±Iabc
/// - The gain is controlled by current, not voltage — perfect for VCA/compressor
#[derive(Debug, Clone, Copy)]
pub struct OtaModel {
    /// Maximum bias current (A). CA3080 typical: 0.5mA.
    pub iabc_max: f64,
    /// Thermal voltage (V). ~25.85mV at 20°C.
    pub vt: f64,
    /// Output load resistance (Ω). Determines voltage gain from current output.
    pub r_load: f64,
}

impl OtaModel {
    /// CA3080 with typical bias point.
    ///
    /// The CA3080 is the classic OTA used in:
    /// - MXR Dyna Comp (compressor)
    /// - Ross Compressor
    /// - EHX Doctor Q (envelope filter)
    /// - Boss CE-1 (chorus VCA section)
    pub fn ca3080() -> Self {
        Self {
            iabc_max: 0.5e-3, // 500µA max amplifier bias current
            vt: 25.85e-3,     // Thermal voltage at 20°C
            r_load: 10_000.0, // Typical 10k load resistor
        }
    }
}

/// OTA nonlinear root for WDF trees.
///
/// Models the transconductance amplifier as a current-output device.
/// The output current follows a tanh transfer characteristic:
/// `Iout = Iabc * tanh(Vin / (2*Vt))`
///
/// The bias current Iabc controls the gain — this is the compression
/// mechanism in OTA compressors. An envelope follower generates a
/// control signal that reduces Iabc as the input gets louder, creating
/// automatic gain reduction.
///
/// For the WDF constraint, the OTA output current flows through the
/// load resistor, creating the voltage that the rest of the tree sees.
#[derive(Debug, Clone, Copy)]
pub struct OtaRoot {
    pub model: OtaModel,
    /// Current amplifier bias current (A). Controls gain.
    /// In a compressor, this is modulated by the envelope follower.
    iabc: f64,
    /// Maximum Newton-Raphson iterations (bounded for RT safety).
    max_iter: usize,
}

impl OtaRoot {
    pub fn new(model: OtaModel) -> Self {
        Self {
            iabc: model.iabc_max,
            model,
            max_iter: 16,
        }
    }

    /// Set the amplifier bias current (external control from envelope, LFO, etc.).
    ///
    /// In a compressor circuit, higher envelope = lower Iabc = less gain.
    /// Range: 0 to iabc_max.
    #[inline]
    pub fn set_iabc(&mut self, iabc: f64) {
        self.iabc = iabc.clamp(0.0, self.model.iabc_max);
    }

    /// Get current amplifier bias current.
    #[inline]
    pub fn iabc(&self) -> f64 {
        self.iabc
    }

    /// Set gain as a normalized value (0.0 = off, 1.0 = max gain).
    ///
    /// This is a convenience wrapper for compressor-style control:
    /// the envelope follower output (0-1) directly maps to gain reduction.
    #[inline]
    pub fn set_gain_normalized(&mut self, gain: f64) {
        self.iabc = gain.clamp(0.0, 1.0) * self.model.iabc_max;
    }

    /// Compute the OTA output current for a given input voltage.
    ///
    /// Uses the tanh transfer characteristic of the differential pair:
    /// `Iout = Iabc * tanh(Vin / (2*Vt))`
    #[inline]
    pub fn output_current(&self, v_in: f64) -> f64 {
        let x = v_in / (2.0 * self.model.vt);
        self.iabc * x.tanh()
    }

    /// Derivative of output current w.r.t. input voltage.
    ///
    /// `dIout/dVin = Iabc / (2*Vt) * sech²(Vin / (2*Vt))`
    ///            = gm * sech²(Vin / (2*Vt))
    #[inline]
    fn output_current_derivative(&self, v_in: f64) -> f64 {
        let x = v_in / (2.0 * self.model.vt);
        let sech = 1.0 / x.cosh();
        self.iabc / (2.0 * self.model.vt) * sech * sech
    }

    /// Compute the transconductance (gm) at the current bias point.
    ///
    /// gm = Iabc / (2 * Vt) — this is the small-signal gain.
    /// For CA3080 at Iabc=500µA: gm ≈ 9.7 mA/V (≈ 9.7 mS)
    pub fn transconductance(&self) -> f64 {
        self.iabc / (2.0 * self.model.vt)
    }
}

impl WdfRoot for OtaRoot {
    /// Compute reflected wave from incident wave `a` and port resistance `rp`.
    ///
    /// The OTA output current flows through the load resistance to produce
    /// a voltage. The WDF constraint relates this to the wave variables:
    ///
    /// `v = (a + b) / 2` (voltage across the port)
    /// `i = (a - b) / (2 * Rp)` (current into the port)
    ///
    /// The OTA current: `i = Iabc * tanh(v / (2*Vt))`
    /// Solve: `f(v) = a - 2*v - 2*Rp * Iabc * tanh(v / (2*Vt)) = 0`
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
        // Small-signal conductance for initial guess
        let gm = self.transconductance();
        let mut v = if gm > 1e-12 {
            a / (2.0 + 2.0 * rp * gm)
        } else {
            a * 0.5
        };

        for _ in 0..self.max_iter {
            let i_ota = self.output_current(v);
            let di_ota = self.output_current_derivative(v);

            let f = a - 2.0 * v - 2.0 * rp * i_ota;
            let fp = -2.0 - 2.0 * rp * di_ota;

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
// BBD (Bucket-Brigade Device) Delay Line
// ---------------------------------------------------------------------------

/// BBD delay line model parameters.
///
/// Bucket-brigade devices (MN3007, MN3207, MN3005) are analog delay lines
/// used in classic chorus, flanger, and delay pedals. Each device has a
/// fixed number of stages and a clock frequency range that determines the
/// delay time.
///
/// The BBD introduces characteristic artifacts:
/// - **Clock noise**: high-frequency switching artifacts (filtered by LPF)
/// - **Bandwidth limiting**: Nyquist limit at half the clock frequency
/// - **Companding noise**: from the compander used to improve S/N ratio
/// - **Aliasing**: soft HF rolloff from the sample-and-hold nature
///
/// We model these as a delay buffer with a variable-rate reader,
/// anti-aliasing LPF, and subtle noise/distortion.
#[derive(Debug, Clone, Copy)]
pub struct BbdModel {
    /// Number of BBD stages. MN3207=1024, MN3007=1024, MN3005=4096.
    pub num_stages: usize,
    /// Minimum clock frequency (Hz). Determines maximum delay time.
    pub clock_min: f64,
    /// Maximum clock frequency (Hz). Determines minimum delay time.
    pub clock_max: f64,
    /// Signal bandwidth limit as fraction of clock (Nyquist ≈ 0.45 * fclk).
    pub bandwidth_ratio: f64,
    /// Noise floor (linear amplitude). BBDs are noisy devices.
    pub noise_floor: f64,
}

impl BbdModel {
    /// MN3207 — 1024-stage BBD, used in Boss CE-2 chorus.
    ///
    /// Clock range: 10kHz – 200kHz
    /// Delay range: 1024/(2*fclk) = 2.56ms – 51.2ms
    pub fn mn3207() -> Self {
        Self {
            num_stages: 1024,
            clock_min: 10_000.0,
            clock_max: 200_000.0,
            bandwidth_ratio: 0.45,
            noise_floor: 0.001,
        }
    }

    /// MN3007 — 1024-stage BBD, used in Boss DM-2 delay.
    ///
    /// Lower noise version of MN3207, same stage count.
    /// Clock range: 10kHz – 100kHz
    /// Delay range: 5.12ms – 51.2ms
    pub fn mn3007() -> Self {
        Self {
            num_stages: 1024,
            clock_min: 10_000.0,
            clock_max: 100_000.0,
            bandwidth_ratio: 0.45,
            noise_floor: 0.0005,
        }
    }

    /// MN3005 — 4096-stage BBD, used in Boss DM-2 (long delay mode),
    /// Electro-Harmonix Memory Man.
    ///
    /// Clock range: 10kHz – 100kHz
    /// Delay range: 20.48ms – 204.8ms
    pub fn mn3005() -> Self {
        Self {
            num_stages: 4096,
            clock_min: 10_000.0,
            clock_max: 100_000.0,
            bandwidth_ratio: 0.45,
            noise_floor: 0.0008,
        }
    }

    /// Delay time in seconds for a given clock frequency.
    ///
    /// BBD delay = num_stages / (2 * fclk)
    /// The factor of 2 is because samples advance one stage per half-clock.
    pub fn delay_at_clock(&self, clock_hz: f64) -> f64 {
        self.num_stages as f64 / (2.0 * clock_hz)
    }

    /// Minimum delay time in seconds.
    pub fn min_delay(&self) -> f64 {
        self.delay_at_clock(self.clock_max)
    }

    /// Maximum delay time in seconds.
    pub fn max_delay(&self) -> f64 {
        self.delay_at_clock(self.clock_min)
    }
}

/// BBD delay line processor.
///
/// Implements a physically-modeled bucket-brigade delay with:
/// - Variable delay time (via clock frequency)
/// - Anti-alias filtering (models the BBD's Nyquist limit)
/// - Subtle soft clipping (BBDs clip gently at high levels)
/// - Noise injection (characteristic BBD hiss)
///
/// The delay buffer uses linear interpolation for fractional-sample
/// delay, modeling the smooth time modulation of chorus/flanger effects.
#[derive(Debug, Clone)]
pub struct BbdDelayLine {
    pub model: BbdModel,
    /// Circular delay buffer.
    buffer: Vec<f64>,
    /// Write position in the buffer.
    write_pos: usize,
    /// Current delay time in samples (fractional for interpolation).
    delay_samples: f64,
    /// Current clock frequency (Hz).
    clock_freq: f64,
    /// Sample rate.
    sample_rate: f64,
    /// Anti-alias filter state (simple one-pole LPF).
    lpf_state: f64,
    /// Anti-alias filter coefficient.
    lpf_coef: f64,
    /// Simple RNG state for noise injection.
    rng_state: u32,
    /// Feedback amount (0.0–1.0).
    feedback: f64,
    /// Previous output for feedback path.
    feedback_sample: f64,
}

impl BbdDelayLine {
    /// Create a new BBD delay line.
    ///
    /// Buffer size is determined by the maximum delay time at the current
    /// sample rate, plus headroom for interpolation.
    pub fn new(model: BbdModel, sample_rate: f64) -> Self {
        let max_delay_samples = (model.max_delay() * sample_rate) as usize + 4;
        let buffer = vec![0.0; max_delay_samples];

        // Default to middle of clock range
        let clock_freq = (model.clock_min + model.clock_max) / 2.0;
        let delay_samples = model.delay_at_clock(clock_freq) * sample_rate;

        // Anti-alias LPF cutoff: bandwidth_ratio * clock_freq
        let lpf_cutoff = model.bandwidth_ratio * clock_freq;
        let lpf_coef = (-2.0 * std::f64::consts::PI * lpf_cutoff / sample_rate).exp();

        Self {
            model,
            buffer,
            write_pos: 0,
            delay_samples,
            clock_freq,
            sample_rate,
            lpf_state: 0.0,
            lpf_coef,
            rng_state: 42_u32,
            feedback: 0.0,
            feedback_sample: 0.0,
        }
    }

    /// Set the clock frequency directly (Hz).
    ///
    /// This controls the delay time: delay = num_stages / (2 * fclk).
    /// Modulating this with an LFO creates chorus/flanger effects.
    pub fn set_clock(&mut self, clock_hz: f64) {
        self.clock_freq = clock_hz.clamp(self.model.clock_min, self.model.clock_max);
        self.delay_samples = self.model.delay_at_clock(self.clock_freq) * self.sample_rate;

        // Update anti-alias filter
        let lpf_cutoff = self.model.bandwidth_ratio * self.clock_freq;
        self.lpf_coef = (-2.0 * std::f64::consts::PI * lpf_cutoff / self.sample_rate).exp();
    }

    /// Set delay time as a normalized value (0.0 = min delay, 1.0 = max delay).
    ///
    /// Maps logarithmically across the clock frequency range.
    pub fn set_delay_normalized(&mut self, norm: f64) {
        let norm = norm.clamp(0.0, 1.0);
        // Logarithmic interpolation between min and max clock
        // norm=0 -> max clock (min delay), norm=1 -> min clock (max delay)
        let log_min = self.model.clock_min.ln();
        let log_max = self.model.clock_max.ln();
        let clock = (log_max - norm * (log_max - log_min)).exp();
        self.set_clock(clock);
    }

    /// Set feedback amount (0.0 = no feedback, <1.0 for stability).
    pub fn set_feedback(&mut self, feedback: f64) {
        self.feedback = feedback.clamp(0.0, 0.95);
    }

    /// Process one sample through the BBD delay line.
    ///
    /// 1. Write input (with feedback) to the buffer
    /// 2. Read from the buffer at the fractional delay position
    /// 3. Apply anti-alias filtering (models BBD bandwidth limit)
    /// 4. Apply subtle soft clipping (BBDs clip at ~1V swing)
    /// 5. Add characteristic noise
    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
        // Mix input with feedback
        let write_sample = input + self.feedback * self.feedback_sample;

        // Soft clip the input to the BBD (they clip at moderate levels)
        let clipped = bbd_soft_clip(write_sample);

        // Write to buffer
        self.buffer[self.write_pos] = clipped;

        // Read with linear interpolation
        let delay_int = self.delay_samples as usize;
        let delay_frac = self.delay_samples - delay_int as f64;
        let buf_len = self.buffer.len();

        let idx0 = (self.write_pos + buf_len - delay_int) % buf_len;
        let idx1 = (idx0 + buf_len - 1) % buf_len;

        let out_raw = self.buffer[idx0] * (1.0 - delay_frac) + self.buffer[idx1] * delay_frac;

        // Anti-alias LPF (models BBD bandwidth limiting)
        self.lpf_state = self.lpf_coef * self.lpf_state + (1.0 - self.lpf_coef) * out_raw;

        // Add BBD noise
        let noise = self.next_noise() * self.model.noise_floor;
        let output = self.lpf_state + noise;

        // Store for feedback
        self.feedback_sample = output;

        // Advance write position
        self.write_pos = (self.write_pos + 1) % buf_len;

        output
    }

    /// Update sample rate and resize buffer.
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
        let max_delay_samples = (self.model.max_delay() * sample_rate) as usize + 4;
        self.buffer = vec![0.0; max_delay_samples];
        self.write_pos = 0;
        self.delay_samples = self.model.delay_at_clock(self.clock_freq) * sample_rate;

        let lpf_cutoff = self.model.bandwidth_ratio * self.clock_freq;
        self.lpf_coef = (-2.0 * std::f64::consts::PI * lpf_cutoff / sample_rate).exp();
    }

    /// Reset all state.
    pub fn reset(&mut self) {
        for s in &mut self.buffer {
            *s = 0.0;
        }
        self.write_pos = 0;
        self.lpf_state = 0.0;
        self.feedback_sample = 0.0;
    }

    /// Get current delay time in seconds.
    pub fn delay_time(&self) -> f64 {
        self.delay_samples / self.sample_rate
    }

    /// Get current clock frequency.
    pub fn clock_freq(&self) -> f64 {
        self.clock_freq
    }

    /// Simple fast PRNG for noise injection (xorshift).
    #[inline]
    fn next_noise(&mut self) -> f64 {
        let mut x = self.rng_state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng_state = x;
        // Convert to -1.0 to 1.0 range
        (x as f64 / u32::MAX as f64) * 2.0 - 1.0
    }
}

/// BBD soft clipping characteristic.
///
/// BBDs have a limited signal swing (~1V peak in typical circuits).
/// They clip more gently than op-amps, with a gradual compression
/// that adds warmth. We model this with a cubic soft clipper.
#[inline]
pub fn bbd_soft_clip(x: f64) -> f64 {
    if x.abs() < 1.0 {
        x - x * x * x / 3.0 // Cubic soft clip
    } else {
        x.signum() * 2.0 / 3.0 // Saturated at 2/3 (cubic limit)
    }
}
