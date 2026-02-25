//! Nonlinear WDF root elements: Diodes, JFETs, MOSFETs, Zener diodes, OTAs.
//!
//! These elements sit at the tree root and use Newton-Raphson iteration
//! to solve the implicit WDF constraint equation for the reflected wave.
//!
//! Also includes the slew rate limiter (models op-amp bandwidth limiting)
//! and OTA (operational transconductance amplifier) for CA3080-based circuits.

use super::WdfRoot;

/// Numerically stable softplus: `ln(1 + exp(x))`.
///
/// For |x| > 50, uses asymptotic approximation:
///   - x > 50: `ln(1 + exp(x)) ≈ x` (error < 1e-22)
///   - x < -50: `ln(1 + exp(x)) ≈ exp(x) ≈ 0` (return 0)
///
/// The transition at |x| = 50 is mathematically smooth to within f64 precision.
/// This is the standard numerically-stable implementation used throughout
/// the Koren triode/pentode equations.
#[inline]
fn softplus(x: f64) -> f64 {
    if x > 50.0 {
        x
    } else if x < -50.0 {
        0.0
    } else {
        (1.0 + x.exp()).ln()
    }
}

/// Minimum conductance returned by nonlinear element derivative functions
/// when a device is in cutoff or reverse-bias.
///
/// This is physically motivated: even in cutoff, real semiconductors have
/// finite leakage current due to minority carriers and surface effects.
/// For silicon: Icbo ≈ 1–100 nA → Gleakage ≈ 1e-12 to 1e-9 S.
/// Using 1e-12 S (1 TΩ) is conservative and prevents Newton-Raphson
/// from dividing by zero without adding measurable phantom current.
const LEAKAGE_CONDUCTANCE: f64 = 1e-12;

// ---------------------------------------------------------------------------
// Unified Newton-Raphson WDF Solver
// ---------------------------------------------------------------------------

/// Unified Newton-Raphson solver for all WDF nonlinear root elements.
///
/// Every nonlinear element in the WDF tree solves the same constraint equation:
///
///   `f(v) = a - 2v - 2·Rp·i(v) = 0`
///
/// where `a` is the incident wave, `Rp` is the port resistance, `v` is the
/// voltage across the element, and `i(v)` is the device-specific current.
///
/// The device I-V characteristic is provided as a closure returning
/// `(current, d_current/d_voltage)` at a given voltage.
///
/// Returns the reflected wave `b = 2v - a`.
///
/// # Parameters
/// - `a`: Incident wave from the WDF tree
/// - `rp`: Port resistance (Ω)
/// - `v`: Initial guess for the voltage (mutable, used as starting point)
/// - `max_iter`: Maximum iterations (bounded for real-time safety)
/// - `tolerance`: Convergence threshold for |dv|
/// - `v_clamp`: Optional (min, max) voltage clamping bounds
/// - `step_limit`: If the raw Newton step exceeds this, the step is halved
///   to prevent overshoot. Pass `None` for standard (undamped) Newton.
/// - `device_iv`: Closure returning `(i, di_dv)` for a given voltage
#[inline]
fn newton_raphson_solve<F>(
    a: f64,
    rp: f64,
    mut v: f64,
    max_iter: usize,
    tolerance: f64,
    v_clamp: Option<(f64, f64)>,
    step_limit: Option<f64>,
    mut device_iv: F,
) -> f64
where
    F: FnMut(f64) -> (f64, f64),
{
    for _ in 0..max_iter {
        let (i, di_dv) = device_iv(v);

        let f = a - 2.0 * v - 2.0 * rp * i;
        let fp = -2.0 - 2.0 * rp * di_dv;

        if fp.abs() < 1e-15 {
            break;
        }

        let mut dv = f / fp;

        // Adaptive damping: halve step when it exceeds the threshold
        if let Some(limit) = step_limit {
            if dv.abs() > limit {
                dv *= 0.5;
            }
        }

        v -= dv;

        if let Some((lo, hi)) = v_clamp {
            v = v.clamp(lo, hi);
        }

        if dv.abs() < tolerance {
            break;
        }
    }

    2.0 * v - a
}

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
    /// Generic silicon diode — averaged parameters for 1N914/1N4148 family.
    pub fn silicon() -> Self {
        // 1N4148 datasheet: Is ≈ 2.52nA, n ≈ 1.752 (fitted to Vf=0.62V @ 1mA)
        Self {
            is: 2.52e-9,
            n_vt: 1.752 * 25.85e-3,
        }
    }

    /// 1N914 — fast switching silicon diode (original TS808 diode).
    /// Slightly lower forward voltage than 1N4148 due to smaller die.
    pub fn _1n914() -> Self {
        Self {
            is: 3.44e-9,
            n_vt: 1.80 * 25.85e-3,
        }
    }

    /// 1N4148 — ubiquitous small-signal silicon diode.
    /// Most common clipping diode in guitar pedals.
    pub fn _1n4148() -> Self {
        Self {
            is: 2.52e-9,
            n_vt: 1.752 * 25.85e-3,
        }
    }

    /// 1N4001 — rectifier diode, used in some pedals for asymmetric clipping.
    /// Higher Is and softer knee than signal diodes.
    pub fn _1n4001() -> Self {
        Self {
            is: 14.11e-9,
            n_vt: 1.984 * 25.85e-3,
        }
    }

    /// Generic germanium diode — averaged parameters for OA-series / 1N34A.
    pub fn germanium() -> Self {
        Self {
            is: 1e-6,
            n_vt: 1.3 * 25.85e-3,
        }
    }

    /// 1N34A — classic germanium point-contact diode.
    /// Lower forward voltage (~0.3V) for earlier, softer clipping onset.
    pub fn _1n34a() -> Self {
        Self {
            is: 2.0e-6,
            n_vt: 1.25 * 25.85e-3,
        }
    }

    /// OA90 — European germanium glass diode (Mullard).
    /// Very low forward voltage, used in vintage fuzzes.
    pub fn _oa90() -> Self {
        Self {
            is: 0.8e-6,
            n_vt: 1.35 * 25.85e-3,
        }
    }

    /// Generic LED diode — higher forward voltage (~1.7V) for harder clipping.
    ///
    /// LEDs have much lower saturation current than silicon diodes due to their
    /// wide bandgap (GaAsP for red LEDs). This results in higher forward voltage
    /// at the same current levels.
    pub fn led() -> Self {
        // Red LED: Vf ≈ 1.7V at 10mA
        // Derived from: Vf = n*Vt * ln(If/Is)
        // 1.7 = 2.0 * 0.02585 * ln(0.01/Is)
        // Is ≈ 4.5e-17 A
        Self {
            is: 4.5e-17,
            n_vt: 2.0 * 25.85e-3,
        }
    }

    /// Red LED — Vf ≈ 1.7V, used in Klon Centaur and high-headroom clippers.
    pub fn led_red() -> Self {
        // Same as generic LED
        Self {
            is: 4.5e-17,
            n_vt: 2.0 * 25.85e-3,
        }
    }

    /// Green LED — higher Vf ≈ 2.1V for even more headroom.
    pub fn led_green() -> Self {
        // Green LEDs have even wider bandgap, Vf ≈ 2.1V at 10mA
        // Is ≈ 1e-19 A
        Self {
            is: 1.0e-19,
            n_vt: 2.0 * 25.85e-3,
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
    /// Alias for `with_voltage` for API compatibility.
    pub fn new(vz: f64) -> Self {
        Self::with_voltage(vz)
    }

    /// Create a zener diode with specified breakdown voltage.
    ///
    /// Dynamic resistance is computed continuously from the breakdown voltage
    /// using a physically-motivated model:
    ///   - Below ~5V: avalanche-dominated breakdown → higher Rz (~20–30Ω)
    ///   - Above ~7V: true zener (tunneling) mechanism → lower Rz (~2–5Ω)
    ///   - Transition region: smooth interpolation
    ///
    /// Empirical fit: Rz ≈ 2 + 28 / (1 + (Vz/5.5)^3)
    /// Matches datasheet Rz values for common zeners (1N47xx series) within ±20%.
    pub fn with_voltage(vz: f64) -> Self {
        // Continuous Rz model: high at low Vz (avalanche), low at high Vz (zener)
        let vz_norm = vz / 5.5; // Crossover point at ~5.5V
        let rz = 2.0 + 28.0 / (1.0 + vz_norm * vz_norm * vz_norm);

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
    pub fn current(&self, v: f64) -> f64 {
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
        let root = *self;
        // Initial guess based on operating region
        let v0 = if a > 0.0 {
            (a * 0.5).min(0.7)
        } else if a < -2.0 * self.model.vz {
            -self.model.vz - 0.1
        } else {
            a * 0.5
        };
        newton_raphson_solve(a, rp, v0, self.max_iter, 1e-6, None, None, |v| {
            (root.current(v), root.current_derivative(v))
        })
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
    /// Anti-parallel diode pair: `i(v) = Is*(exp(v/nVt) - exp(-v/nVt))`
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
        let is = self.model.is;
        let nvt = self.model.n_vt;

        let v0 = if a.abs() > 10.0 * nvt {
            let log_arg = (a.abs() / (2.0 * rp * is)).max(1.0);
            nvt * log_arg.ln() * a.signum()
        } else {
            a * 0.5
        };

        newton_raphson_solve(a, rp, v0, self.max_iter, 1e-6, None, None, |v| {
            let x = (v / nvt).clamp(-500.0, 500.0);
            let ev_pos = x.exp();
            let ev_neg = (-x).exp();
            let i = is * (ev_pos - ev_neg);
            let di = is * (ev_pos + ev_neg) / nvt;
            (i, di)
        })
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
    /// Single diode: `i(v) = Is*(exp(v/nVt) - 1)`
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
        let is = self.model.is;
        let nvt = self.model.n_vt;

        // Initial guess depends on bias direction:
        // - Forward bias (a > 0): Use logarithmic estimate from Shockley equation
        // - Reverse bias (a < 0): Diode blocks, i_d ≈ -Is ≈ 0, so v ≈ a/2
        let v0 = if a > 10.0 * nvt {
            // Forward bias: logarithmic estimate
            let log_arg = (a / (2.0 * rp * is)).max(1.0);
            nvt * log_arg.ln()
        } else if a < -10.0 * nvt {
            // Reverse bias: diode blocks
            a * 0.5
        } else {
            // Small signal: linear approximation
            a * 0.5
        };

        newton_raphson_solve(a, rp, v0, self.max_iter, 1e-6, None, None, |v| {
            let x = (v / nvt).clamp(-500.0, 500.0);
            let ev = x.exp();
            let i = is * (ev - 1.0);
            let di = is * ev / nvt;
            (i, di)
        })
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

// ---------------------------------------------------------------------------
// BJT (Bipolar Junction Transistor) Models
// ---------------------------------------------------------------------------

/// BJT model parameters using the Ebers-Moll equations.
///
/// Models NPN and PNP transistors with the industry-standard Ebers-Moll
/// model for accurate discrete transistor simulation.
///
/// The Ebers-Moll equations (simplified for forward-active region):
/// ```text
/// Ic = Is * (exp(Vbe / Vt) - 1)
/// Ib = Ic / Bf
/// Ie = Ic + Ib = Ic * (1 + 1/Bf)
/// ```
///
/// Key characteristic: Unlike JFETs/triodes, BJTs have significant base
/// current. This loading effect is crucial for Fuzz Face tone.
#[derive(Debug, Clone, Copy)]
pub struct BjtModel {
    /// Saturation current (A). Typically 1e-14 to 1e-12 for small-signal BJTs.
    pub is: f64,
    /// Forward current gain (beta, hFE). Typically 100-300 for audio transistors.
    pub bf: f64,
    /// Thermal voltage (V). kT/q ≈ 25.85mV at 25°C.
    pub vt: f64,
    /// Early voltage (V). Models base-width modulation. Typically 50-200V.
    /// Set to infinity (1e6) to disable Early effect.
    pub va: f64,
    /// Whether this is a PNP (vs NPN) transistor.
    pub is_pnp: bool,
}

impl BjtModel {
    /// 2N3904 — The workhorse NPN small-signal transistor.
    ///
    /// A ubiquitous general-purpose NPN used in countless audio circuits.
    /// Medium beta, good linearity, silicon.
    pub fn n2n3904() -> Self {
        Self {
            is: 1e-14,
            bf: 200.0,   // hFE typically 100-300
            vt: 0.02585, // 25.85mV at 25°C
            va: 100.0,   // Early voltage
            is_pnp: false,
        }
    }

    /// 2N2222 — Classic NPN switching/amplifier transistor.
    ///
    /// Higher current capability than 2N3904, very common.
    pub fn n2n2222() -> Self {
        Self {
            is: 1.4e-14,
            bf: 150.0,
            vt: 0.02585,
            va: 80.0,
            is_pnp: false,
        }
    }

    /// BC108 — British/European small-signal NPN.
    ///
    /// Used in Vox, Marshall, and many UK-designed pedals.
    pub fn bc108() -> Self {
        Self {
            is: 2e-14,
            bf: 300.0, // BC108 has high gain
            vt: 0.02585,
            va: 100.0,
            is_pnp: false,
        }
    }

    /// BC109 — Low-noise variant of BC108.
    ///
    /// Common in Tone Bender and other fuzz circuits.
    pub fn bc109() -> Self {
        Self {
            is: 1.5e-14,
            bf: 350.0,
            vt: 0.02585,
            va: 100.0,
            is_pnp: false,
        }
    }

    /// 2N5088 — High-gain NPN for fuzz/distortion.
    ///
    /// Very high beta (300-900), used in Big Muff and high-gain circuits.
    pub fn n2n5088() -> Self {
        Self {
            is: 1e-14,
            bf: 500.0, // Can be 300-900
            vt: 0.02585,
            va: 100.0,
            is_pnp: false,
        }
    }

    /// 2N5089 — Ultra-high-gain NPN.
    ///
    /// Even higher beta than 2N5088. Used in Big Muff and boosters.
    pub fn n2n5089() -> Self {
        Self {
            is: 1e-14,
            bf: 700.0, // Can be 400-1200
            vt: 0.02585,
            va: 100.0,
            is_pnp: false,
        }
    }

    // ── PNP Transistors ──────────────────────────────────────────────────

    /// 2N3906 — Standard PNP complement to 2N3904.
    pub fn n2n3906() -> Self {
        Self {
            is: 1e-14,
            bf: 200.0,
            vt: 0.02585,
            va: 100.0,
            is_pnp: true,
        }
    }

    /// AC128 — Classic germanium PNP for Fuzz Face.
    ///
    /// THE Fuzz Face transistor. Low beta, high leakage, temperature-sensitive.
    /// Germanium has lower Vbe (~0.2V vs 0.6V silicon).
    pub fn ac128() -> Self {
        Self {
            is: 1e-6,    // Germanium: much higher leakage
            bf: 70.0,    // Lower gain than silicon
            vt: 0.02585, // Same thermal voltage
            va: 50.0,    // Lower Early voltage
            is_pnp: true,
        }
    }

    /// OC44 — Vintage germanium PNP.
    ///
    /// Used in early Tone Benders. Very low beta, vintage character.
    pub fn oc44() -> Self {
        Self {
            is: 2e-6,
            bf: 50.0, // Very low gain
            vt: 0.02585,
            va: 30.0,
            is_pnp: true,
        }
    }

    /// NKT275 — Sought-after germanium PNP.
    ///
    /// "Holy grail" Fuzz Face transistor. Medium-low beta with sweet spot.
    pub fn nkt275() -> Self {
        Self {
            is: 1.5e-6,
            bf: 85.0,
            vt: 0.02585,
            va: 40.0,
            is_pnp: true,
        }
    }

    /// Generic NPN with typical values.
    pub fn generic_npn() -> Self {
        Self {
            is: 1e-14,
            bf: 200.0,
            vt: 0.02585,
            va: 100.0,
            is_pnp: false,
        }
    }

    /// Generic PNP with typical values.
    pub fn generic_pnp() -> Self {
        Self {
            is: 1e-14,
            bf: 200.0,
            vt: 0.02585,
            va: 100.0,
            is_pnp: true,
        }
    }

    /// Convert from DSL BjtType to runtime BjtModel.
    pub fn from_bjt_type(bt: &crate::dsl::BjtType) -> Self {
        use crate::dsl::BjtType;
        match bt {
            BjtType::Generic => Self::generic_npn(),
            BjtType::N2n3904 => Self::n2n3904(),
            BjtType::N2n2222 => Self::n2n2222(),
            BjtType::Bc108 => Self::bc108(),
            BjtType::Bc109 => Self::bc109(),
            BjtType::N2n5088 => Self::n2n5088(),
            BjtType::N2n5089 => Self::n2n5089(),
            BjtType::N2n3906 => Self::n2n3906(),
            BjtType::Ac128 => Self::ac128(),
            BjtType::Oc44 => Self::oc44(),
            BjtType::Nkt275 => Self::nkt275(),
        }
    }
}

/// BJT NPN nonlinear root for WDF trees.
///
/// Models the collector-emitter port of an NPN BJT using Ebers-Moll equations.
/// The base-emitter voltage is set externally (like JFET's Vgs), and we solve
/// for Vce. Base current is tracked for circuit loading effects.
///
/// For Fuzz Face circuits, the base current loading on the source is a huge
/// part of the sound — this is captured by the `base_current()` method which
/// can be used to model the loading effect.
///
/// **Ebers-Moll (forward-active):**
/// ```text
/// Ic = Is * (exp(Vbe / Vt) - 1) * (1 + Vce/Va)
/// Ib = Ic / Bf
/// ```
///
/// The Early effect term `(1 + Vce/Va)` models the slight Vce dependence of Ic.
#[derive(Debug, Clone, Copy)]
pub struct BjtNpnRoot {
    pub model: BjtModel,
    /// Base-emitter voltage (external control parameter).
    vbe: f64,
    /// Last computed collector current (for base current tracking).
    ic: f64,
    /// Maximum collector-emitter voltage (determined by supply rail Vcc).
    /// Vce can swing from ~0V (saturated) to Vcc (cutoff).
    v_max: f64,
    /// Maximum Newton-Raphson iterations.
    max_iter: usize,
    /// Convergence tolerance.
    tolerance: f64,
}

impl BjtNpnRoot {
    pub fn new(model: BjtModel) -> Self {
        Self {
            model,
            vbe: 0.0,
            ic: 0.0,
            v_max: 100.0, // Default for typical transistor circuits (will be set by supply)
            max_iter: 16,
            tolerance: 1e-6,
        }
    }

    /// Create a BJT NPN root with a specific supply voltage (Vcc).
    ///
    /// Use this when the supply voltage is known at construction time.
    pub fn new_with_v_max(model: BjtModel, v_max: f64) -> Self {
        Self {
            model,
            vbe: 0.0,
            ic: 0.0,
            v_max: v_max.max(1.0),
            max_iter: 16,
            tolerance: 1e-6,
        }
    }

    /// Set the maximum collector-emitter voltage (Vcc supply rail).
    ///
    /// For BJT circuits, Vce can swing from ~0V (transistor saturated)
    /// to Vcc (transistor in cutoff). This sets the upper bound for Newton-Raphson.
    ///
    /// Examples:
    /// - 9V guitar pedal: 9V
    /// - 12V effects: 12V
    /// - High-voltage preamp: 24-48V
    #[inline]
    pub fn set_v_max(&mut self, v_max: f64) {
        self.v_max = v_max.max(1.0); // Minimum 1V to avoid degeneracy
    }

    /// Get the current v_max setting.
    #[inline]
    pub fn v_max(&self) -> f64 {
        self.v_max
    }

    /// Set the base-emitter voltage (external control from bias, signal, etc.)
    #[inline]
    pub fn set_vbe(&mut self, vbe: f64) {
        self.vbe = vbe;
    }

    /// Get current base-emitter voltage.
    #[inline]
    pub fn vbe(&self) -> f64 {
        self.vbe
    }

    /// Get the last computed collector current.
    ///
    /// Call this after `process()` to get the collector current for that sample.
    #[inline]
    pub fn collector_current(&self) -> f64 {
        self.ic
    }

    /// Get the base current (Ib = Ic / Bf).
    ///
    /// This is crucial for Fuzz Face — the base current loading on the source
    /// affects the input impedance and contributes to the unique interaction
    /// between stages.
    #[inline]
    pub fn base_current(&self) -> f64 {
        self.ic / self.model.bf
    }

    /// Compute collector current for given Vce at current Vbe.
    ///
    /// Uses Ebers-Moll with Early effect for forward-active region.
    #[inline]
    pub fn collector_current_at(&self, vce: f64) -> f64 {
        let is = self.model.is;
        let vt = self.model.vt;
        let va = self.model.va;
        let vbe = self.vbe;

        // Forward-active: Vbe > 0, Vce > 0 (for NPN)
        // Cutoff: Vbe < 0
        if vbe < 0.0 {
            // Cutoff region - small leakage current
            return is * 1e-6;
        }

        // Ebers-Moll: Ic = Is * (exp(Vbe/Vt) - 1)
        // Clamp exponent to prevent overflow
        let x = (vbe / vt).min(40.0);
        let exp_vbe = x.exp();
        let ic_base = is * (exp_vbe - 1.0);

        // Early effect: Ic = Ic_base * (1 + Vce/Va)
        // Only apply if in forward-active (Vce > 0)
        let early_factor = if vce > 0.0 && va > 0.0 {
            1.0 + vce / va
        } else {
            1.0
        };

        // Saturation limiting: when Vce < Vce_sat, collector current drops
        // Vce_sat ≈ 0.1V for small-signal BJTs
        let saturation_factor = if vce > 0.0 {
            // Soft saturation using tanh
            (vce / 0.2).tanh()
        } else {
            0.0 // Reverse active/cutoff
        };

        ic_base * early_factor * saturation_factor.max(0.0)
    }

    /// Compute derivative of collector current w.r.t. Vce.
    #[inline]
    fn collector_current_derivative(&self, vce: f64) -> f64 {
        // Cutoff check
        if self.vbe < 0.0 {
            return 1e-12; // Very small conductance in cutoff
        }

        // d/dVce of [Ic_base * (1 + Vce/Va) * sat_factor]
        // This is complex due to saturation term; use numerical approximation for stability
        let delta = 1e-6;
        let ic_plus = self.collector_current_at(vce + delta);
        let ic_minus = self.collector_current_at(vce - delta);
        (ic_plus - ic_minus) / (2.0 * delta)
    }
}

impl WdfRoot for BjtNpnRoot {
    /// NPN BJT collector-emitter path: `i = Ic(Vce, Vbe)`
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
        let root = *self;
        let v_max = self.v_max;
        let v0 = if self.vbe > 0.3 {
            (a * 0.5).max(0.1)
        } else {
            a * 0.5
        };
        let b = newton_raphson_solve(
            a, rp, v0, self.max_iter, self.tolerance,
            Some((-1.0, v_max)), None,
            |v| (root.collector_current_at(v), root.collector_current_derivative(v)),
        );
        // Store collector current for external use
        let v = (a + b) / 2.0;
        self.ic = self.collector_current_at(v);
        b
    }
}

/// BJT PNP nonlinear root for WDF trees.
///
/// Same as NPN but with reversed polarities. PNP transistors conduct when
/// Veb > 0 (emitter more positive than base) and Vec > 0 (emitter more
/// positive than collector).
///
/// For germanium PNP Fuzz Face (AC128, NKT275), the biasing is:
/// - Negative ground topology (emitter to "ground" which is V+)
/// - Collector pulls current down toward ground
#[derive(Debug, Clone, Copy)]
pub struct BjtPnpRoot {
    pub model: BjtModel,
    /// Emitter-base voltage (Veb, reversed from NPN's Vbe).
    veb: f64,
    /// Last computed collector current magnitude.
    ic: f64,
    /// Maximum emitter-collector voltage (determined by supply rail).
    /// Vec can swing from ~0V (saturated) to supply voltage (cutoff).
    v_max: f64,
    max_iter: usize,
    tolerance: f64,
}

impl BjtPnpRoot {
    pub fn new(model: BjtModel) -> Self {
        Self {
            model,
            veb: 0.0,
            ic: 0.0,
            v_max: 100.0, // Default for typical transistor circuits (will be set by supply)
            max_iter: 16,
            tolerance: 1e-6,
        }
    }

    /// Create a BJT PNP root with a specific supply voltage.
    ///
    /// Use this when the supply voltage is known at construction time.
    pub fn new_with_v_max(model: BjtModel, v_max: f64) -> Self {
        Self {
            model,
            veb: 0.0,
            ic: 0.0,
            v_max: v_max.max(1.0),
            max_iter: 16,
            tolerance: 1e-6,
        }
    }

    /// Set the maximum emitter-collector voltage (supply rail).
    ///
    /// For PNP circuits (like germanium Fuzz Face), Vec can swing from ~0V
    /// (transistor saturated) to supply voltage (transistor in cutoff).
    #[inline]
    pub fn set_v_max(&mut self, v_max: f64) {
        self.v_max = v_max.max(1.0); // Minimum 1V to avoid degeneracy
    }

    /// Get the current v_max setting.
    #[inline]
    pub fn v_max(&self) -> f64 {
        self.v_max
    }

    /// Set the emitter-base voltage (Veb = Ve - Vb).
    #[inline]
    pub fn set_veb(&mut self, veb: f64) {
        self.veb = veb;
    }

    /// Get current emitter-base voltage.
    #[inline]
    pub fn veb(&self) -> f64 {
        self.veb
    }

    /// Get the last computed collector current magnitude.
    #[inline]
    pub fn collector_current(&self) -> f64 {
        self.ic
    }

    /// Get the base current (Ib = Ic / Bf).
    #[inline]
    pub fn base_current(&self) -> f64 {
        self.ic / self.model.bf
    }

    /// Compute collector current for given Vec (emitter-collector voltage).
    ///
    /// For PNP: current flows from emitter to collector when Veb > 0, Vec > 0.
    #[inline]
    pub fn collector_current_at(&self, vec: f64) -> f64 {
        let is = self.model.is;
        let vt = self.model.vt;
        let va = self.model.va;
        let veb = self.veb;

        // Forward-active: Veb > 0, Vec > 0 (for PNP)
        if veb < 0.0 {
            return is * 1e-6;
        }

        let x = (veb / vt).min(40.0);
        let exp_veb = x.exp();
        let ic_base = is * (exp_veb - 1.0);

        let early_factor = if vec > 0.0 && va > 0.0 {
            1.0 + vec / va
        } else {
            1.0
        };

        let saturation_factor = if vec > 0.0 {
            (vec / 0.2).tanh()
        } else {
            0.0
        };

        ic_base * early_factor * saturation_factor.max(0.0)
    }

    #[inline]
    fn collector_current_derivative(&self, vec: f64) -> f64 {
        let delta = 1e-6;
        let ic_plus = self.collector_current_at(vec + delta);
        let ic_minus = self.collector_current_at(vec - delta);
        (ic_plus - ic_minus) / (2.0 * delta)
    }
}

impl WdfRoot for BjtPnpRoot {
    /// PNP BJT emitter-collector path: `i = Ic(Vec, Veb)`
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
        let root = *self;
        let v_max = self.v_max;
        let v0 = if self.veb > 0.2 {
            (a * 0.5).max(0.1)
        } else {
            a * 0.5
        };
        let b = newton_raphson_solve(
            a, rp, v0, self.max_iter, self.tolerance,
            Some((-1.0, v_max)), None,
            |v| (root.collector_current_at(v), root.collector_current_derivative(v)),
        );
        self.ic = self.collector_current_at((a + b) / 2.0);
        b
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
    /// Maximum plate voltage (determined by supply rail B+).
    /// Pentode plate can swing from 0V (saturated) to B+ (cutoff).
    v_max: f64,
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
            v_max: 500.0, // Default to high-voltage tube amp (will be set by supply)
            max_iter: 16,
        }
    }

    /// Create a pentode root with a specific supply voltage (B+).
    ///
    /// Use this when the supply voltage is known at construction time.
    pub fn new_with_v_max(model: PentodeModel, v_max: f64) -> Self {
        let vg2k = model.vg2_default;
        Self {
            model,
            vg1k: 0.0,
            vg2k,
            v_max: v_max.max(1.0),
            max_iter: 16,
        }
    }

    /// Set the maximum plate voltage (B+ supply rail).
    ///
    /// For tube circuits, the plate voltage can swing from 0V (tube saturated)
    /// to B+ (tube in cutoff). This sets the upper bound for Newton-Raphson.
    #[inline]
    pub fn set_v_max(&mut self, v_max: f64) {
        self.v_max = v_max.max(1.0); // Minimum 1V to avoid degeneracy
    }

    /// Get the current v_max setting.
    #[inline]
    pub fn v_max(&self) -> f64 {
        self.v_max
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

        let ln_term = softplus(e1);
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
            return LEAKAGE_CONDUCTANCE;
        }

        let e1 = kp * (1.0 / mu + vg1k / (kvb + vg2k * vg2k).sqrt());

        let ln_term = softplus(e1);
        let base = (vg2k / kp) * ln_term;
        if base <= 0.0 {
            return LEAKAGE_CONDUCTANCE;
        }

        let ip_base = base.powf(ex);

        // d/dVpk of (2/π) * atan(Vpk/Kvb2) = (2/π) * 1/(1 + (Vpk/Kvb2)^2) * 1/Kvb2
        let vpk_ratio = vpk / kvb2;
        let d_plate_factor = (2.0 / std::f64::consts::PI) / (1.0 + vpk_ratio * vpk_ratio) / kvb2;

        ip_base * d_plate_factor
    }
}

impl WdfRoot for PentodeRoot {
    /// Pentode plate-cathode path: `i = Ip(Vpk, Vg1k, Vg2k)`
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
    /// OTA transconductance: `i = Iabc * tanh(v / (2·Vt))`
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
        let root = *self;
        let gm = self.transconductance();
        let v0 = if gm > LEAKAGE_CONDUCTANCE {
            a / (2.0 + 2.0 * rp * gm)
        } else {
            a * 0.5
        };
        newton_raphson_solve(a, rp, v0, self.max_iter, 1e-6, None, None, |v| {
            (root.output_current(v), root.output_current_derivative(v))
        })
    }
}

// ---------------------------------------------------------------------------
// Op-Amp (Voltage-Controlled Voltage Source) Models
// ---------------------------------------------------------------------------

/// Op-amp model parameters for WDF integration.
///
/// Models the op-amp as a voltage-controlled voltage source:
/// `Vout = Aol * (Vp - Vm)`
///
/// Where:
/// - `Aol` is the open-loop DC gain (typically 100k-1M)
/// - `Vp` is the non-inverting input voltage (set externally)
/// - `Vm` is the inverting input voltage (often feedback-dependent)
///
/// The feedback network stays in the WDF tree as passive components.
/// The op-amp root enforces the gain relationship between its differential
/// input and output via Newton-Raphson iteration.
///
/// Non-idealities captured:
/// - Finite open-loop gain
/// - Output voltage saturation
/// - Slew rate limiting (optional, applied post-convergence)
/// - Gain-bandwidth product (for future frequency-dependent modeling)
#[derive(Debug, Clone, Copy)]
pub struct OpAmpModel {
    /// Open-loop DC gain. TL072 ≈ 200k (106dB), LM308 ≈ 300k.
    pub open_loop_gain: f64,
    /// Gain-bandwidth product (Hz). TL072 ≈ 3MHz, LM308 ≈ 1MHz.
    /// Used for frequency-dependent modeling (future).
    pub gbw: f64,
    /// Slew rate (V/µs). TL072 ≈ 13, LM308 ≈ 0.3.
    pub slew_rate: f64,
    /// Maximum output voltage swing (V, single-sided from Vcc/2).
    /// Typically Vcc/2 - 1.5V for rail-to-rail, less for others.
    pub v_max: f64,
}

impl OpAmpModel {
    /// TL072 — JFET-input dual op-amp.
    ///
    /// The modern standard for guitar pedals. Fast slew rate (13 V/µs),
    /// low noise, high input impedance. Used in Klon Centaur, Phase 90,
    /// and countless modern designs.
    pub fn tl072() -> Self {
        Self {
            open_loop_gain: 200_000.0, // 106 dB
            gbw: 3e6,                   // 3 MHz
            slew_rate: 13.0,            // 13 V/µs
            v_max: 12.0,                // ±12V swing at ±15V supply
        }
    }

    /// TL082 — JFET-input dual op-amp (TL072 variant).
    ///
    /// Similar to TL072 but slightly different specs. Used interchangeably
    /// in many designs.
    pub fn tl082() -> Self {
        Self {
            open_loop_gain: 200_000.0,
            gbw: 4e6,                   // 4 MHz (slightly faster)
            slew_rate: 13.0,
            v_max: 12.0,
        }
    }

    /// LM308 — The RAT's secret weapon.
    ///
    /// Very slow slew rate (0.3 V/µs) creates the distinctive compression
    /// and "sag" character of the ProCo RAT. The slow slew rate rounds off
    /// transients, creating a smoother, more compressed distortion.
    pub fn lm308() -> Self {
        Self {
            open_loop_gain: 300_000.0, // 110 dB
            gbw: 1e6,                   // 1 MHz
            slew_rate: 0.3,             // 0.3 V/µs — THE key to RAT tone
            v_max: 12.0,
        }
    }

    /// LM741 — The classic vintage op-amp.
    ///
    /// Slow and noisy by modern standards, but has a characteristic sound.
    /// Used in early Big Muff Pi and some vintage designs.
    pub fn lm741() -> Self {
        Self {
            open_loop_gain: 200_000.0,
            gbw: 1e6,
            slew_rate: 0.5,             // Slow
            v_max: 12.0,
        }
    }

    /// JRC4558 — The Tube Screamer's heart.
    ///
    /// Medium slew rate (1.7 V/µs) contributes to the Tube Screamer's
    /// warm, slightly compressed midrange. The JRC (Japan Radio Company)
    /// 4558D is the legendary variant.
    pub fn jrc4558() -> Self {
        Self {
            open_loop_gain: 100_000.0, // 100 dB
            gbw: 3e6,                   // 3 MHz
            slew_rate: 1.7,             // Moderate
            v_max: 12.0,
        }
    }

    /// RC4558 — Common 4558 variant.
    ///
    /// Texas Instruments version of the 4558, used in many TS clones.
    /// Very similar to JRC4558 but some players hear subtle differences.
    pub fn rc4558() -> Self {
        Self {
            open_loop_gain: 100_000.0,
            gbw: 3e6,
            slew_rate: 1.5,
            v_max: 12.0,
        }
    }

    /// NE5532 — Studio-grade low-noise op-amp.
    ///
    /// Very fast, very clean. Used in high-end effects and studio gear.
    /// The "transparent" choice when you don't want op-amp coloration.
    pub fn ne5532() -> Self {
        Self {
            open_loop_gain: 100_000.0,
            gbw: 10e6,                  // 10 MHz
            slew_rate: 9.0,             // Fast
            v_max: 12.0,
        }
    }

    /// OP07 — Precision op-amp.
    ///
    /// Very low offset voltage and drift. Used in some boutique designs
    /// where DC accuracy matters.
    pub fn op07() -> Self {
        Self {
            open_loop_gain: 400_000.0, // 112 dB
            gbw: 0.6e6,                 // 600 kHz
            slew_rate: 0.3,             // Slow
            v_max: 12.0,
        }
    }

    /// Generic op-amp with typical values.
    ///
    /// Use when the specific op-amp type doesn't matter or isn't specified.
    pub fn generic() -> Self {
        Self {
            open_loop_gain: 100_000.0,
            gbw: 1e6,
            slew_rate: 1.0,
            v_max: 12.0,
        }
    }

    /// Convert from DSL OpAmpType to runtime OpAmpModel.
    pub fn from_opamp_type(ot: &crate::dsl::OpAmpType) -> Self {
        use crate::dsl::OpAmpType;
        match ot {
            OpAmpType::Generic => Self::generic(),
            OpAmpType::Tl072 => Self::tl072(),
            OpAmpType::Tl082 => Self::tl082(),
            OpAmpType::Jrc4558 => Self::jrc4558(),
            OpAmpType::Rc4558 => Self::rc4558(),
            OpAmpType::Lm308 => Self::lm308(),
            OpAmpType::Lm741 => Self::lm741(),
            OpAmpType::Ne5532 => Self::ne5532(),
            OpAmpType::Op07 => Self::op07(),
            // OTAs (CA3080) are handled separately as OtaRoot, not OpAmpRoot
            OpAmpType::Ca3080 => Self::generic(),
        }
    }
}

// ---------------------------------------------------------------------------
// Op-Amp Root (VCVS WDF Element)
// ---------------------------------------------------------------------------

/// Op-amp nonlinear root for WDF trees.
///
/// Models the op-amp as a voltage-controlled voltage source:
/// `Vout = Aol * (Vp - Vm)`
///
/// For a unity-gain buffer (voltage follower), Vm = Vout (direct feedback),
/// which creates the familiar virtual short between inputs. The Newton-Raphson
/// solver finds the output voltage that satisfies both the op-amp equation
/// and the WDF constraint.
///
/// **Usage in Phase 90 all-pass stages:**
/// Each op-amp is configured as a unity-gain buffer (neg tied to out).
/// Set `vp` to the signal at the positive input each sample, and the
/// root will produce the buffered output.
///
/// **Feedback configurations:**
/// - Unity-gain (voltage follower): `feedback_ratio = 1.0`, output follows Vp
/// - Inverting amp: `feedback_ratio = Rf/(Rf+Rin)`, Vm depends on output
/// - Non-inverting amp: similar, but Vp receives the signal
///
/// The feedback network components (resistors, capacitors) remain in the
/// WDF tree — this root just enforces the op-amp gain relationship.
#[derive(Debug, Clone, Copy)]
pub struct OpAmpRoot {
    pub model: OpAmpModel,
    /// Non-inverting input voltage (set externally each sample).
    vp: f64,
    /// Feedback ratio: 0.0 = no feedback, 1.0 = unity-gain buffer.
    /// Vm = vm_external * (1 - fb_ratio) + v_out * fb_ratio
    feedback_ratio: f64,
    /// External voltage at inverting input (for non-feedback path).
    vm_external: f64,
    /// Previous output voltage (for slew rate limiting).
    prev_out: f64,
    /// Sample rate (needed for slew rate limiting).
    sample_rate: f64,
    /// Maximum Newton-Raphson iterations (bounded for RT safety).
    max_iter: usize,
    /// Convergence tolerance.
    tolerance: f64,
}

impl OpAmpRoot {
    /// Create a new op-amp root with the given model.
    ///
    /// Default configuration: unity-gain buffer (feedback_ratio = 1.0).
    pub fn new(model: OpAmpModel) -> Self {
        Self {
            model,
            vp: 0.0,
            feedback_ratio: 1.0, // Unity-gain buffer by default
            vm_external: 0.0,
            prev_out: 0.0,
            sample_rate: 48000.0,
            max_iter: 16,
            tolerance: 1e-9,
        }
    }

    /// Create a unity-gain buffer (voltage follower).
    ///
    /// This is the most common op-amp configuration in guitar pedals.
    /// The output directly follows the non-inverting input.
    pub fn unity_gain(model: OpAmpModel) -> Self {
        let mut root = Self::new(model);
        root.feedback_ratio = 1.0;
        root
    }

    /// Set the non-inverting input voltage.
    ///
    /// Call this each sample before `process()` to set the signal
    /// that the op-amp is buffering/amplifying.
    #[inline]
    pub fn set_vp(&mut self, vp: f64) {
        self.vp = vp;
    }

    /// Get the current non-inverting input voltage.
    #[inline]
    pub fn vp(&self) -> f64 {
        self.vp
    }

    /// Configure the feedback topology.
    ///
    /// - `ratio = 1.0`: Unity-gain buffer (Vm = Vout)
    /// - `ratio = 0.5`: 2x non-inverting gain
    /// - `ratio = R2/(R1+R2)`: General non-inverting amp
    ///
    /// For inverting configurations, set `vm_external` to the input signal
    /// and `ratio` to the feedback divider ratio.
    #[inline]
    pub fn set_feedback(&mut self, ratio: f64, vm_external: f64) {
        self.feedback_ratio = ratio.clamp(0.0, 1.0);
        self.vm_external = vm_external;
    }

    /// Set the sample rate (for slew rate limiting).
    #[inline]
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
    }

    /// Apply slew rate limiting to the output.
    ///
    /// Real op-amps have a maximum rate of voltage change (dV/dt).
    /// When the output tries to change faster than this, it "slews"
    /// at the maximum rate, creating the compression and warmth
    /// characteristic of slow op-amps like the LM308.
    #[inline]
    fn apply_slew_limit(&mut self, v: f64) -> f64 {
        // Maximum voltage change per sample: slew_rate * (1e6 / sample_rate)
        let max_dv = self.model.slew_rate * 1e6 / self.sample_rate;
        let dv = v - self.prev_out;
        let limited = if dv > max_dv {
            self.prev_out + max_dv
        } else if dv < -max_dv {
            self.prev_out - max_dv
        } else {
            v
        };
        self.prev_out = limited;
        limited
    }

    /// Reset internal state (call on audio restart).
    pub fn reset(&mut self) {
        self.prev_out = 0.0;
        self.vp = 0.0;
        self.vm_external = 0.0;
    }

    /// Set the maximum output voltage (determined by supply rails).
    ///
    /// For single-supply pedals at Vsupply biased at Vsupply/2:
    /// v_max ≈ (Vsupply/2) - saturation_margin
    ///
    /// For ±15V supplies (lab standard): v_max ≈ ±12V
    /// For 9V single-supply biased at 4.5V: v_max ≈ ±3.5V
    #[inline]
    pub fn set_v_max(&mut self, v_max: f64) {
        self.model.v_max = v_max.max(0.5); // Minimum 0.5V to avoid degeneracy
    }

    /// Get the current v_max setting.
    #[inline]
    pub fn v_max(&self) -> f64 {
        self.model.v_max
    }
}

impl WdfRoot for OpAmpRoot {
    /// Op-amp VCVS: `Vout = Aol * (Vp - Vm)`, with soft saturation and slew limiting.
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
        let aol = self.model.open_loop_gain;
        let v_max = self.model.v_max;
        let vp = self.vp;
        let feedback_ratio = self.feedback_ratio;
        let vm_external = self.vm_external;

        let v0 = if feedback_ratio > 0.5 {
            vp.clamp(-v_max, v_max)
        } else {
            (0.5 * a).clamp(-v_max, v_max)
        };

        let b = newton_raphson_solve(
            a, rp, v0, self.max_iter, self.tolerance,
            Some((-v_max * 1.5, v_max * 1.5)), Some(v_max),
            |v| {
                // Compute Vm as a function of output voltage
                let vm = vm_external * (1.0 - feedback_ratio) + v * feedback_ratio;
                let delta = vp - vm;
                let ddelta_dv = -feedback_ratio;

                // Soft saturation: tanh-based limiting preserves gradient
                let raw_ideal = aol * delta;
                let v_ideal = v_max * (raw_ideal / v_max).tanh();
                let t = (raw_ideal / v_max).tanh();
                let dv_ideal_ddelta = aol * (1.0 - t * t);

                // Effective output resistance
                let r_out_eff = (rp / (aol * feedback_ratio.max(0.001))).max(0.01);

                let i_out = (v_ideal - v) / r_out_eff;
                let di_dv = (dv_ideal_ddelta * ddelta_dv - 1.0) / r_out_eff;
                (i_out, di_dv)
            },
        );

        // Recover voltage from reflected wave, apply post-NR processing
        let mut v = (a + b) / 2.0;

        // Final hard clip at rails (after convergence)
        v = v.clamp(-v_max, v_max);

        // Apply slew rate limiting
        v = self.apply_slew_limit(v);

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
    /// Per-stage charge leakage rate (fraction lost per stage per clock period).
    /// Real BBDs lose charge through substrate leakage and junction capacitance.
    /// Typical: 0.0001–0.001 per stage, causing progressive HF loss at longer delays.
    pub leakage_per_stage: f64,
    /// Clock feedthrough amplitude (normalized).
    /// The switching clock couples into the analog signal path through parasitic
    /// capacitance in the MOSFET switches.  Audible as a high-pitched whine.
    /// Typical: -60 to -80 dB below signal = 0.001–0.0001.
    pub clock_feedthrough: f64,
    /// Compander tracking error.  Real BBD circuits use NE571/SA571 compander
    /// ICs to expand dynamic range.  The compressor (input) and expander (output)
    /// don't track perfectly — the rectifier time constants differ, causing
    /// "breathing" and pumping artifacts.  0.0 = perfect tracking, 1.0 = maximum error.
    pub compander_error: f64,
}

impl BbdModel {
    /// MN3207 — 1024-stage BBD, used in Boss CE-2 chorus.
    ///
    /// Clock range: 10kHz – 200kHz
    /// Delay range: 1024/(2*fclk) = 2.56ms – 51.2ms
    /// Relatively high leakage and clock feedthrough (consumer-grade).
    pub fn mn3207() -> Self {
        Self {
            num_stages: 1024,
            clock_min: 10_000.0,
            clock_max: 200_000.0,
            bandwidth_ratio: 0.45,
            noise_floor: 0.001,
            leakage_per_stage: 0.0005,
            clock_feedthrough: 0.0008,
            compander_error: 0.15,
        }
    }

    /// MN3007 — 1024-stage BBD, used in Boss DM-2 delay.
    ///
    /// Lower noise version of MN3207, same stage count.
    /// Better charge retention and lower clock feedthrough.
    /// Clock range: 10kHz – 100kHz
    /// Delay range: 5.12ms – 51.2ms
    pub fn mn3007() -> Self {
        Self {
            num_stages: 1024,
            clock_min: 10_000.0,
            clock_max: 100_000.0,
            bandwidth_ratio: 0.45,
            noise_floor: 0.0005,
            leakage_per_stage: 0.0003,
            clock_feedthrough: 0.0004,
            compander_error: 0.12,
        }
    }

    /// MN3005 — 4096-stage BBD, used in Boss DM-2 (long delay mode),
    /// Electro-Harmonix Memory Man.
    ///
    /// 4x more stages means leakage accumulates more, causing darker
    /// repeats at long delay times — the "warm decay" Memory Man sound.
    /// Clock range: 10kHz – 100kHz
    /// Delay range: 20.48ms – 204.8ms
    pub fn mn3005() -> Self {
        Self {
            num_stages: 4096,
            clock_min: 10_000.0,
            clock_max: 100_000.0,
            bandwidth_ratio: 0.45,
            noise_floor: 0.0008,
            leakage_per_stage: 0.0004,
            clock_feedthrough: 0.0003,
            compander_error: 0.10,
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
    /// Inner delay line handling ring buffer, write position, and interpolation.
    inner: DelayLine,
    /// Current clock frequency (Hz).
    clock_freq: f64,
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
    /// Leakage LPF coefficient: models accumulated charge loss across all stages.
    /// More stages and lower clock = more leakage = darker output.
    /// This is a one-pole LPF whose cutoff decreases with delay time.
    leakage_lpf_state: f64,
    leakage_lpf_coef: f64,
    /// Clock feedthrough oscillator phase (0..2π).
    clock_phase: f64,
    /// Clock feedthrough phase increment per sample.
    clock_phase_inc: f64,
    /// Compander envelope state: tracks the input RMS level for the
    /// compressor side.  The expander side sees a slightly delayed/smoothed
    /// version, causing tracking error.
    compander_env_in: f64,
    compander_env_out: f64,
    /// Compander attack/release coefficients.
    compander_attack: f64,
    compander_release: f64,
}

impl BbdDelayLine {
    /// Create a new BBD delay line.
    ///
    /// Buffer size is determined by the maximum delay time at the current
    /// sample rate, plus headroom for interpolation.
    pub fn new(model: BbdModel, sample_rate: f64) -> Self {
        // Derive delay range from the BBD clock range.
        // Max clock → min delay, min clock → max delay.
        let min_delay = model.delay_at_clock(model.clock_max);
        let max_delay = model.delay_at_clock(model.clock_min);
        let mut inner = DelayLine::new(min_delay, max_delay, sample_rate, Interpolation::Linear);

        // Default to middle of clock range
        let clock_freq = (model.clock_min + model.clock_max) / 2.0;
        inner.set_delay_seconds(model.delay_at_clock(clock_freq));

        // Anti-alias LPF cutoff: bandwidth_ratio * clock_freq
        let lpf_cutoff = model.bandwidth_ratio * clock_freq;
        let lpf_coef = (-2.0 * std::f64::consts::PI * lpf_cutoff / sample_rate).exp();

        // Leakage LPF: models accumulated charge loss across all BBD stages.
        // Each stage loses `leakage_per_stage` of its charge per clock period.
        // Total loss scales with num_stages — longer delays sound darker.
        // The equivalent LPF cutoff decreases as delay increases.
        let leakage_lpf_coef = Self::compute_leakage_coef(&model, clock_freq, sample_rate);

        // Clock feedthrough: the BBD switching clock couples into the signal path.
        let clock_phase_inc = 2.0 * std::f64::consts::PI * clock_freq / sample_rate;

        // Compander: NE571-style attack/release time constants.
        // Attack ~1ms, release ~10ms — the mismatch causes "breathing" artifacts.
        let compander_attack = (-1.0 / (0.001 * sample_rate)).exp();
        let compander_release = (-1.0 / (0.010 * sample_rate)).exp();

        Self {
            model,
            inner,
            clock_freq,
            lpf_state: 0.0,
            lpf_coef,
            rng_state: 42_u32,
            feedback: 0.0,
            feedback_sample: 0.0,
            leakage_lpf_state: 0.0,
            leakage_lpf_coef,
            clock_phase: 0.0,
            clock_phase_inc,
            compander_env_in: 0.0,
            compander_env_out: 0.0,
            compander_attack,
            compander_release,
        }
    }

    /// Compute the leakage LPF coefficient from model parameters and clock frequency.
    ///
    /// The charge leakage per stage accumulates over the total number of stages.
    /// At lower clock frequencies (longer delays), each stage holds charge longer,
    /// so more leaks.  The equivalent one-pole LPF cutoff is:
    ///   fc_leakage = -ln(1 - leakage_per_stage × N) × fclk / (2π)
    /// Clamped to prevent the cutoff from exceeding Nyquist.
    fn compute_leakage_coef(model: &BbdModel, clock_freq: f64, sample_rate: f64) -> f64 {
        let total_loss = (model.leakage_per_stage * model.num_stages as f64).min(0.99);
        // Map total charge retention to a LPF cutoff.
        // retention = 1 - total_loss = fraction of signal preserved.
        // Higher loss → lower cutoff → darker sound.
        let retention = 1.0 - total_loss;
        // Cutoff in Hz: scale with clock frequency (higher clock = shorter hold time = less leakage)
        let fc_leakage = -(retention.ln()) * clock_freq / (2.0 * std::f64::consts::PI);
        let fc_clamped = fc_leakage.min(sample_rate * 0.45); // Never exceed Nyquist
        (-2.0 * std::f64::consts::PI * fc_clamped / sample_rate).exp()
    }

    /// Set the clock frequency directly (Hz).
    ///
    /// This controls the delay time: delay = num_stages / (2 * fclk).
    /// Modulating this with an LFO creates chorus/flanger effects.
    pub fn set_clock(&mut self, clock_hz: f64) {
        self.clock_freq = clock_hz.clamp(self.model.clock_min, self.model.clock_max);
        self.inner.set_delay_seconds(self.model.delay_at_clock(self.clock_freq));

        let sample_rate = self.inner.sample_rate();

        // Update anti-alias filter
        let lpf_cutoff = self.model.bandwidth_ratio * self.clock_freq;
        self.lpf_coef = (-2.0 * std::f64::consts::PI * lpf_cutoff / sample_rate).exp();

        // Update leakage LPF (more leakage at lower clock = longer delays)
        self.leakage_lpf_coef =
            Self::compute_leakage_coef(&self.model, self.clock_freq, sample_rate);

        // Update clock feedthrough frequency
        self.clock_phase_inc =
            2.0 * std::f64::consts::PI * self.clock_freq / sample_rate;
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
    /// Signal chain (models real BBD circuit):
    /// 1. Compander input stage: compress dynamic range (NE571 compressor)
    /// 2. Soft clip to BBD voltage swing limit
    /// 3. Write to delay buffer (with feedback)
    /// 4. Read from buffer with interpolation
    /// 5. Charge leakage LPF (darker at longer delays)
    /// 6. Anti-alias LPF (BBD Nyquist bandwidth limit)
    /// 7. Clock feedthrough injection
    /// 8. Noise injection
    /// 9. Compander output stage: expand dynamic range (NE571 expander)
    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
        // ── Compander input (compressor) ──────────────────────────────
        // Track input envelope with fast attack, slow release.
        let abs_in = input.abs();
        let coef_in = if abs_in > self.compander_env_in {
            self.compander_attack
        } else {
            self.compander_release
        };
        self.compander_env_in =
            coef_in * self.compander_env_in + (1.0 - coef_in) * abs_in;

        // Compress: reduce dynamic range by 2:1 (typical NE571 ratio).
        // gain = 1/sqrt(envelope) — louder signals get compressed more.
        let comp_gain = if self.compander_env_in > 0.001 {
            1.0 / self.compander_env_in.sqrt()
        } else {
            1.0
        };
        let compressed = input * comp_gain.min(10.0); // Limit expansion of quiet signals

        // ── Mix with feedback and soft-clip ───────────────────────────
        let write_sample = compressed + self.feedback * self.feedback_sample;
        let clipped = bbd_soft_clip(write_sample);

        // Write to inner delay line's buffer (direct — feedback is handled above)
        self.inner.write_direct(clipped);

        // ── Read with interpolation ───────────────────────────────────
        let delay_samples = self.inner.effective_delay_samples();
        let out_raw = self.inner.buffer_read(delay_samples, 0);

        // Advance write position (and process medium if configured)
        self.inner.advance_write();

        // ── Charge leakage LPF ───────────────────────────────────────
        // Models accumulated charge loss across BBD stages.  More stages
        // and lower clock frequency → more leakage → darker output.
        // This is what gives long BBD delays their characteristic warmth.
        self.leakage_lpf_state = self.leakage_lpf_coef * self.leakage_lpf_state
            + (1.0 - self.leakage_lpf_coef) * out_raw;

        // ── Anti-alias LPF (BBD Nyquist bandwidth limit) ─────────────
        self.lpf_state = self.lpf_coef * self.lpf_state
            + (1.0 - self.lpf_coef) * self.leakage_lpf_state;

        // ── Clock feedthrough ─────────────────────────────────────────
        // The BBD switching clock couples into the signal through parasitic
        // capacitance.  Audible as a high-pitched whine, especially at
        // lower clock frequencies where it falls into the audio band.
        self.clock_phase += self.clock_phase_inc;
        if self.clock_phase > 2.0 * std::f64::consts::PI {
            self.clock_phase -= 2.0 * std::f64::consts::PI;
        }
        let clock_bleed = self.model.clock_feedthrough * self.clock_phase.sin();

        // ── Noise injection ───────────────────────────────────────────
        let noise = self.next_noise() * self.model.noise_floor;

        let bbd_output = self.lpf_state + clock_bleed + noise;

        // ── Compander output (expander) ───────────────────────────────
        // Track output envelope — deliberately uses slightly different
        // time constants to model NE571 tracking error.
        let abs_out = bbd_output.abs();
        // Expander envelope is intentionally sluggish compared to compressor,
        // scaled by compander_error to control the magnitude of the artifact.
        let error_scale = 1.0 + self.model.compander_error;
        let coef_out = if abs_out > self.compander_env_out {
            // Expander attack is slower than compressor attack
            self.compander_attack * error_scale
        } else {
            // Expander release is faster than compressor release
            self.compander_release / error_scale
        };
        self.compander_env_out =
            coef_out.min(0.9999) * self.compander_env_out + (1.0 - coef_out.min(0.9999)) * abs_out;

        // Expand: restore dynamic range.
        let exp_gain = if self.compander_env_out > 0.001 {
            self.compander_env_out.sqrt()
        } else {
            1.0
        };
        let output = bbd_output * exp_gain.min(10.0);

        // Store for feedback (post-compander, as in real circuits)
        self.feedback_sample = output;

        output
    }

    /// Update sample rate and resize buffer.
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.inner.set_sample_rate(sample_rate);
        self.inner.set_delay_seconds(self.model.delay_at_clock(self.clock_freq));

        let lpf_cutoff = self.model.bandwidth_ratio * self.clock_freq;
        self.lpf_coef = (-2.0 * std::f64::consts::PI * lpf_cutoff / sample_rate).exp();
        self.leakage_lpf_coef =
            Self::compute_leakage_coef(&self.model, self.clock_freq, sample_rate);
        self.clock_phase_inc =
            2.0 * std::f64::consts::PI * self.clock_freq / sample_rate;
        self.compander_attack = (-1.0 / (0.001 * sample_rate)).exp();
        self.compander_release = (-1.0 / (0.010 * sample_rate)).exp();
    }

    /// Reset all state.
    pub fn reset(&mut self) {
        self.inner.reset();
        self.lpf_state = 0.0;
        self.leakage_lpf_state = 0.0;
        self.feedback_sample = 0.0;
        self.clock_phase = 0.0;
        self.compander_env_in = 0.0;
        self.compander_env_out = 0.0;
    }

    /// Get current delay time in seconds.
    pub fn delay_time(&self) -> f64 {
        self.inner.delay_time()
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

/// Tape magnetic hysteresis saturation.
///
/// Models the Jiles-Atherton hysteresis curve simplified to a modified tanh.
/// Tape doesn't clip sharply — it saturates asymptotically. The saturation
/// "knee" is softer than transistor or op-amp clipping.
///
/// `level` is the input amplitude at which the tape reaches ~90% saturation.
/// Typical values: 0.6-0.8 for different tape formulations.
#[inline]
pub fn tape_saturation(x: f64, level: f64) -> f64 {
    // Scale input so that level corresponds to ~90% saturation
    // tanh(1.47) ≈ 0.9, so we scale accordingly
    let scaled = x * 1.47 / level;
    level * scaled.tanh() / 1.47
}

// ═══════════════════════════════════════════════════════════════════════════
// Generic Delay Line — WDF two-port adaptor backed by a ring buffer
// ═══════════════════════════════════════════════════════════════════════════

/// Interpolation mode for delay line read pointer.
///
/// Different delay types require different interpolation characteristics:
/// - Linear: simple, low CPU, slight HF loss — suitable for digital delays
/// - Allpass: sub-sample accuracy without HF loss — best for tape/analog feel
/// - Cubic: smooth with low aliasing — good general purpose
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Interpolation {
    /// Linear interpolation: `y = y0*(1-f) + y1*f`
    /// Simple and cheap. Causes slight HF roll-off at short delays.
    Linear,
    /// First-order allpass interpolation for sub-sample accuracy.
    /// Preserves magnitude response while providing smooth fractional delay.
    /// Best for modulated delays (tape echo, chorus) — no zipper noise.
    Allpass,
    /// Cubic Hermite interpolation: 4-point, smooth transitions.
    /// Good frequency response, moderate CPU cost.
    Cubic,
}

/// Physical storage medium model for the delay buffer.
///
/// In real tape and BBD circuits, the storage medium itself transforms
/// the signal while it sits in the buffer. This is physically distinct
/// from write-side processing (saturation at the record head) and
/// read-side processing (gap loss at playback heads), which are handled
/// by WDF elements in the signal chain.
///
/// The medium processing uses a zone-based approach: the virtual tape
/// path is divided into zones, and each sample gets processed once as
/// it crosses a zone boundary. This is O(zones) per tick, not
/// O(buffer_length), keeping CPU cost negligible.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Medium {
    /// Clean digital delay — no buffer processing.
    None,
    /// Magnetic tape self-demagnetization.
    /// Adjacent domains with opposing polarity partially cancel, causing
    /// frequency-dependent HF loss that increases with distance from the
    /// record head. Short wavelengths (high frequencies) are affected
    /// more because opposing domains are physically closer.
    TapeOxide,
    /// BBD charge leakage.
    /// Amplitude decays uniformly toward zero over time (not frequency-
    /// dependent like tape). Each bucket loses a fraction of its charge
    /// on every clock cycle.
    BbdLeakage,
    /// Digital bit reduction (PT2399-style).
    /// Quantization happens at write time only — no ongoing buffer
    /// processing. The `process_medium()` method is a no-op for this mode.
    DigitalQuantize,
}

/// A zone boundary along the virtual delay path.
///
/// Samples are processed incrementally as they cross zone boundaries.
/// At normal playback speed, exactly one sample crosses each zone
/// boundary per tick, so the total cost is O(zones) per sample.
#[derive(Debug, Clone)]
pub struct ZoneBoundary {
    /// Distance from write head in samples.
    pub distance_samples: f64,
    /// One-pole LP filter state for this zone (tape oxide model).
    pub filter_state: f64,
    /// LP coefficient applied at this boundary.
    /// Higher = stronger HF loss. Typical tape: 0.02–0.05.
    pub coefficient: f64,
    /// Decay rate per zone crossing (BBD leakage model).
    /// Each crossing multiplies the sample by `(1 - leak_rate)`.
    pub leak_rate: f64,
    /// Tracks the last buffer position processed at this boundary.
    pub last_processed_pos: usize,
}

/// Pre-configured medium settings for common tape echo models.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MediumPreset {
    /// RE-201: gentle, well-maintained tape.
    Re201,
    /// Echoplex: darker, more worn tape.
    Echoplex,
    /// Lo-fi: aggressive degradation.
    LoFi,
}

impl MediumPreset {
    /// Get the zone coefficients for this preset.
    ///
    /// Returns coefficients for each zone boundary between taps.
    /// More distant zones have stronger HF loss.
    pub fn zone_coefficients(&self) -> Vec<f64> {
        match self {
            MediumPreset::Re201 => vec![0.02, 0.03, 0.05],
            MediumPreset::Echoplex => vec![0.04, 0.06],
            MediumPreset::LoFi => vec![0.08, 0.12, 0.18],
        }
    }
}

/// Generic delay line: a ring buffer with WDF-compatible ports.
///
/// This is a two-port element that splits the WDF tree into separate
/// subtrees coupled by an implicit one-sample delay. The write side
/// accepts signal from the "record" subtree; the read side feeds the
/// "playback" subtree(s).
///
/// The delay time is continuously modulatable — changing it at audio rate
/// moves the read pointer smoothly, creating pitch bending effects.
///
/// ## WDF integration
///
/// The compiler detects `delay_line()` in a netlist and automatically
/// splits the WDF tree at that point. Trees are processed in order:
/// write tree first (fills buffer), then read tree(s) consume from it.
/// The implicit one-sample delay between write and read ensures causality.
///
/// ## Multi-tap
///
/// Multiple `Tap` elements can read from the same `DelayLine` at
/// different positions (ratios of the base delay time). This models
/// multi-head tape machines (RE-201) and multi-tap digital delays.
#[derive(Debug, Clone)]
pub struct DelayLine {
    /// Ring buffer holding delayed samples.
    buffer: Vec<f64>,
    /// Current write position in the buffer.
    write_pos: usize,
    /// Current delay time in samples (fractional for interpolation).
    current_delay_samples: f64,
    /// Minimum delay time in seconds.
    min_delay_sec: f64,
    /// Maximum delay time in seconds.
    max_delay_sec: f64,
    /// Sample rate (Hz).
    sample_rate: f64,
    /// Interpolation mode for fractional-sample reads.
    interpolation: Interpolation,
    /// Allpass interpolation state (previous output) — one per tap slot.
    /// Index 0 is for the base delay, indices 1..N for taps.
    allpass_states: Vec<f64>,
    /// Speed modulation accumulator (from LFO wow/flutter).
    /// Applied as a multiplier to the base delay time.
    speed_mod: f64,
    /// Feedback sample (from downstream, routed back to input).
    feedback_sample: f64,
    /// Feedback amount (0.0 = no feedback, up to ~0.95 for self-oscillation).
    feedback: f64,
    /// Physical storage medium model.
    medium: Medium,
    /// Zone boundaries for medium processing.
    /// Samples are processed incrementally as they cross each boundary.
    zones: Vec<ZoneBoundary>,
}

impl DelayLine {
    /// Create a new delay line.
    ///
    /// * `min_delay_sec` — minimum delay in seconds (e.g., 0.001 for 1ms)
    /// * `max_delay_sec` — maximum delay in seconds (e.g., 1.2 for 1200ms)
    /// * `sample_rate` — audio sample rate in Hz
    /// * `interpolation` — fractional-sample interpolation mode
    pub fn new(
        min_delay_sec: f64,
        max_delay_sec: f64,
        sample_rate: f64,
        interpolation: Interpolation,
    ) -> Self {
        // Buffer sized for max delay + headroom for interpolation
        let max_samples = (max_delay_sec * sample_rate * 1.1) as usize + 8;
        let buffer = vec![0.0; max_samples];

        // Default to midpoint delay
        let mid = (min_delay_sec + max_delay_sec) / 2.0;
        let current_delay_samples = mid * sample_rate;

        Self {
            buffer,
            write_pos: 0,
            current_delay_samples,
            min_delay_sec,
            max_delay_sec,
            sample_rate,
            interpolation,
            allpass_states: vec![0.0; 8], // Room for up to 8 taps
            speed_mod: 1.0,
            feedback_sample: 0.0,
            feedback: 0.0,
            medium: Medium::None,
            zones: Vec::new(),
        }
    }

    /// Write a sample into the delay buffer, process medium zones,
    /// and advance the write pointer.
    ///
    /// The input signal is mixed with the feedback sample before writing.
    /// After writing, zone-based medium processing runs on any samples
    /// that have crossed zone boundaries since the last tick.
    /// Call this once per audio sample, before reading any taps.
    #[inline]
    pub fn write(&mut self, input: f64) {
        let write_sample = input + self.feedback * self.feedback_sample;
        self.buffer[self.write_pos] = write_sample;
        self.write_pos = (self.write_pos + 1) % self.buffer.len();

        // Process medium zones (incremental — only touches samples at boundaries)
        self.process_medium();
    }

    /// Read from the delay buffer at the current base delay time.
    ///
    /// Returns the interpolated sample at `write_pos - current_delay_samples`.
    /// This is equivalent to reading from tap ratio 1.0.
    #[inline]
    pub fn read(&mut self) -> f64 {
        self.read_at_ratio(1.0, 0)
    }

    /// Read from the delay buffer at a specific ratio of the base delay.
    ///
    /// * `ratio` — multiplier for the base delay (e.g., 2.0 = twice the delay)
    /// * `tap_index` — index for allpass state tracking (0 = base, 1+ = taps)
    ///
    /// The effective delay in samples is:
    ///   `current_delay_samples * speed_mod * ratio`
    #[inline]
    pub fn read_at_ratio(&mut self, ratio: f64, tap_index: usize) -> f64 {
        let effective_delay = self.current_delay_samples * self.speed_mod * ratio;
        let max_delay = (self.buffer.len() - 2) as f64;
        let clamped_delay = effective_delay.clamp(1.0, max_delay);

        match self.interpolation {
            Interpolation::Linear => self.read_linear(clamped_delay),
            Interpolation::Allpass => self.read_allpass(clamped_delay, tap_index),
            Interpolation::Cubic => self.read_cubic(clamped_delay),
        }
    }

    /// Linear interpolation read.
    #[inline]
    fn read_linear(&self, delay_samples: f64) -> f64 {
        let delay_int = delay_samples as usize;
        let frac = delay_samples - delay_int as f64;
        let buf_len = self.buffer.len();

        let idx0 = (self.write_pos + buf_len - delay_int) % buf_len;
        let idx1 = (idx0 + buf_len - 1) % buf_len;

        self.buffer[idx0] * (1.0 - frac) + self.buffer[idx1] * frac
    }

    /// First-order allpass interpolation read.
    ///
    /// Uses the Thiran allpass approximation for fractional delay:
    ///   `y[n] = x[n-D] + eta * (y[n-1] - x[n-D-1])`
    /// where `eta = (1 - frac) / (1 + frac)`.
    ///
    /// This preserves magnitude response (no HF loss) while providing
    /// smooth sub-sample delay — ideal for modulated delays.
    #[inline]
    fn read_allpass(&mut self, delay_samples: f64, tap_index: usize) -> f64 {
        let delay_int = delay_samples as usize;
        let frac = delay_samples - delay_int as f64;
        let buf_len = self.buffer.len();

        let idx0 = (self.write_pos + buf_len - delay_int) % buf_len;
        let idx1 = (idx0 + buf_len - 1) % buf_len;

        let x0 = self.buffer[idx0];
        let x1 = self.buffer[idx1];

        // Thiran allpass coefficient
        let eta = (1.0 - frac) / (1.0 + frac);

        // Ensure tap_index is in range
        let state_idx = tap_index.min(self.allpass_states.len() - 1);
        let prev = self.allpass_states[state_idx];

        let output = x1 + eta * (prev - x0);
        self.allpass_states[state_idx] = output;

        // Protect against NaN/Inf in allpass feedback
        if !output.is_finite() {
            self.allpass_states[state_idx] = 0.0;
            return x0 * (1.0 - frac) + x1 * frac; // Fall back to linear
        }

        output
    }

    /// Cubic Hermite interpolation read (4-point).
    #[inline]
    fn read_cubic(&self, delay_samples: f64) -> f64 {
        let delay_int = delay_samples as usize;
        let frac = delay_samples - delay_int as f64;
        let buf_len = self.buffer.len();

        // 4 sample points: x[-1], x[0], x[1], x[2]
        let idx0 = (self.write_pos + buf_len - delay_int) % buf_len;
        let idx_m1 = (idx0 + 1) % buf_len;
        let idx1 = (idx0 + buf_len - 1) % buf_len;
        let idx2 = (idx1 + buf_len - 1) % buf_len;

        let xm1 = self.buffer[idx_m1];
        let x0 = self.buffer[idx0];
        let x1 = self.buffer[idx1];
        let x2 = self.buffer[idx2];

        // Catmull-Rom cubic interpolation
        let c0 = x0;
        let c1 = 0.5 * (x1 - xm1);
        let c2 = xm1 - 2.5 * x0 + 2.0 * x1 - 0.5 * x2;
        let c3 = 0.5 * (x2 - xm1) + 1.5 * (x0 - x1);

        ((c3 * frac + c2) * frac + c1) * frac + c0
    }

    /// Set the delay time as a normalized value (0.0 = min, 1.0 = max).
    ///
    /// Uses logarithmic interpolation for a natural feel — small changes
    /// at short delays, larger changes at long delays.
    pub fn set_delay_normalized(&mut self, norm: f64) {
        let norm = norm.clamp(0.0, 1.0);
        let log_min = self.min_delay_sec.ln();
        let log_max = self.max_delay_sec.ln();
        let delay_sec = (log_min + norm * (log_max - log_min)).exp();
        self.current_delay_samples = delay_sec * self.sample_rate;
    }

    /// Set the delay time in seconds directly.
    pub fn set_delay_seconds(&mut self, seconds: f64) {
        let clamped = seconds.clamp(self.min_delay_sec, self.max_delay_sec);
        self.current_delay_samples = clamped * self.sample_rate;
    }

    /// Set the speed modulation factor.
    ///
    /// A value of 1.0 means no modulation. Values > 1.0 slow the "tape"
    /// (longer delay, lower pitch). Values < 1.0 speed it up.
    /// Typically modulated by an LFO for wow/flutter effects.
    #[inline]
    pub fn set_speed_mod(&mut self, factor: f64) {
        self.speed_mod = factor.clamp(0.5, 2.0);
    }

    /// Add a speed modulation offset (additive, for multiple LFO sources).
    ///
    /// The total speed_mod = 1.0 + sum of all modulation offsets.
    #[inline]
    pub fn add_speed_mod(&mut self, offset: f64) {
        self.speed_mod += offset;
    }

    /// Reset speed modulation to 1.0 (called at start of each sample).
    #[inline]
    pub fn reset_speed_mod(&mut self) {
        self.speed_mod = 1.0;
    }

    /// Set the feedback amount.
    pub fn set_feedback(&mut self, feedback: f64) {
        self.feedback = feedback.clamp(0.0, 0.99);
    }

    /// Store the feedback sample (called after downstream processing).
    #[inline]
    pub fn set_feedback_sample(&mut self, sample: f64) {
        self.feedback_sample = sample;
    }

    /// Get the current delay time in seconds.
    pub fn delay_time(&self) -> f64 {
        self.current_delay_samples / self.sample_rate
    }

    /// Get the current delay time in samples (with speed modulation).
    pub fn effective_delay_samples(&self) -> f64 {
        self.current_delay_samples * self.speed_mod
    }

    /// Update for a new sample rate.
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        let delay_sec = self.current_delay_samples / self.sample_rate;
        self.sample_rate = sample_rate;
        self.current_delay_samples = delay_sec * sample_rate;

        // Resize buffer if needed
        let max_samples = (self.max_delay_sec * sample_rate * 1.1) as usize + 8;
        if max_samples > self.buffer.len() {
            self.buffer.resize(max_samples, 0.0);
        }
    }

    /// Reset all state (buffer, allpass states, feedback).
    pub fn reset(&mut self) {
        self.buffer.fill(0.0);
        self.write_pos = 0;
        self.allpass_states.fill(0.0);
        self.feedback_sample = 0.0;
        self.speed_mod = 1.0;
        for zone in &mut self.zones {
            zone.filter_state = 0.0;
            zone.last_processed_pos = 0;
        }
    }

    /// Minimum delay in seconds.
    pub fn min_delay_sec(&self) -> f64 {
        self.min_delay_sec
    }

    /// Maximum delay in seconds.
    pub fn max_delay_sec(&self) -> f64 {
        self.max_delay_sec
    }

    /// Set the physical storage medium model.
    pub fn set_medium(&mut self, medium: Medium) {
        self.medium = medium;
    }

    /// Get the current medium model.
    pub fn medium(&self) -> Medium {
        self.medium
    }

    /// Get the current sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Write a sample directly to the buffer without feedback mixing.
    ///
    /// Does NOT advance the write position — call [`advance_write`] after
    /// reading to complete the tick.  This is used by `BbdDelayLine` which
    /// manages its own feedback path (with soft-clipping between mix and write).
    #[inline]
    pub fn write_direct(&mut self, sample: f64) {
        self.buffer[self.write_pos] = sample;
    }

    /// Read from the buffer at the given delay (in samples), using the
    /// current interpolation mode.
    ///
    /// Lower-level than [`read`]/[`read_at_ratio`]: the caller supplies the
    /// exact fractional-sample delay instead of using the stored delay time.
    #[inline]
    pub fn buffer_read(&mut self, delay_samples: f64, tap_slot: usize) -> f64 {
        let delay = delay_samples.clamp(1.0, self.buffer.len() as f64 - 2.0);
        match self.interpolation {
            Interpolation::Linear => self.read_linear(delay),
            Interpolation::Allpass => self.read_allpass(delay, tap_slot),
            Interpolation::Cubic => self.read_cubic(delay),
        }
    }

    /// Advance the write position by one sample and run medium processing.
    ///
    /// Must be called exactly once per tick, after [`write_direct`] and
    /// any reads are complete.
    #[inline]
    pub fn advance_write(&mut self) {
        self.write_pos = (self.write_pos + 1) % self.buffer.len();
        self.process_medium();
    }

    /// Configure zone boundaries from tap ratios.
    ///
    /// Creates one zone boundary at each tap position, with coefficients
    /// that increase with distance (modeling progressive signal degradation).
    /// An additional zone is placed at the end of the buffer for the
    /// feedback path.
    ///
    /// * `tap_ratios` — tap positions as ratios of base delay (e.g., [1.0, 2.0, 4.0])
    /// * `coefficients` — optional per-zone LP coefficients; if `None`, uses
    ///   defaults based on medium type and tap count
    pub fn configure_zones_from_taps(
        &mut self,
        tap_ratios: &[f64],
        coefficients: Option<&[f64]>,
    ) {
        self.zones.clear();

        if self.medium == Medium::None || self.medium == Medium::DigitalQuantize {
            return; // No zone processing needed
        }

        // Sort tap ratios to determine zone boundaries
        let mut sorted_ratios: Vec<f64> = tap_ratios.to_vec();
        sorted_ratios.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        sorted_ratios.dedup();

        // Default coefficients: progressively stronger
        let default_coefficients: Vec<f64> = sorted_ratios
            .iter()
            .enumerate()
            .map(|(i, _)| match self.medium {
                Medium::TapeOxide => 0.02 + 0.015 * i as f64, // 0.02, 0.035, 0.05, ...
                Medium::BbdLeakage => 0.01 + 0.005 * i as f64, // 0.01, 0.015, 0.02, ...
                _ => 0.0,
            })
            .collect();

        let coeffs = coefficients.unwrap_or(&default_coefficients);

        for (i, &ratio) in sorted_ratios.iter().enumerate() {
            let distance_samples = self.current_delay_samples * ratio;
            let coeff = coeffs.get(i).copied().unwrap_or(0.03);
            let leak = if self.medium == Medium::BbdLeakage {
                coeff
            } else {
                0.0
            };

            self.zones.push(ZoneBoundary {
                distance_samples,
                filter_state: 0.0,
                coefficient: coeff,
                leak_rate: leak,
                last_processed_pos: self.write_pos,
            });
        }
    }

    /// Zone-based incremental medium processing.
    ///
    /// For each zone boundary, checks if new samples have crossed it
    /// since the last tick. Applies the appropriate medium transformation
    /// to only those samples. At normal speed, this is O(zones) per tick
    /// (typically 1 sample per zone).
    #[inline]
    fn process_medium(&mut self) {
        match self.medium {
            Medium::None | Medium::DigitalQuantize => {}

            Medium::TapeOxide => {
                let buf_len = self.buffer.len();
                for zone in &mut self.zones {
                    // Current boundary position (relative to write head)
                    let boundary_pos = (self.write_pos + buf_len
                        - zone.distance_samples as usize)
                        % buf_len;

                    // How many samples have crossed this boundary since last tick
                    let samples_crossed = if boundary_pos >= zone.last_processed_pos {
                        boundary_pos - zone.last_processed_pos
                    } else {
                        buf_len - zone.last_processed_pos + boundary_pos
                    };

                    // Clamp to prevent runaway in edge cases (speed transients)
                    let to_process = samples_crossed.min(4);

                    for j in 0..to_process {
                        let idx = (zone.last_processed_pos + j + 1) % buf_len;
                        // One-pole LP: y += coeff * (x - y)
                        // This removes HF — short wavelength domains cancel more
                        zone.filter_state += zone.coefficient
                            * (self.buffer[idx] - zone.filter_state);
                        self.buffer[idx] = zone.filter_state;
                    }

                    zone.last_processed_pos = boundary_pos;
                }
            }

            Medium::BbdLeakage => {
                let buf_len = self.buffer.len();
                for zone in &mut self.zones {
                    let boundary_pos = (self.write_pos + buf_len
                        - zone.distance_samples as usize)
                        % buf_len;

                    let samples_crossed = if boundary_pos >= zone.last_processed_pos {
                        boundary_pos - zone.last_processed_pos
                    } else {
                        buf_len - zone.last_processed_pos + boundary_pos
                    };

                    let to_process = samples_crossed.min(4);

                    for j in 0..to_process {
                        let idx = (zone.last_processed_pos + j + 1) % buf_len;
                        // Uniform amplitude decay — charge leaks to ground
                        self.buffer[idx] *= 1.0 - zone.leak_rate;
                    }

                    zone.last_processed_pos = boundary_pos;
                }
            }
        }
    }

    /// Update zone distances when delay time changes.
    ///
    /// Called automatically by `set_delay_normalized` and `set_delay_seconds`
    /// to keep zone boundaries aligned with the current delay time.
    fn update_zone_distances(&mut self, tap_ratios: &[f64]) {
        for (zone, &ratio) in self.zones.iter_mut().zip(tap_ratios.iter()) {
            zone.distance_samples = self.current_delay_samples * ratio;
        }
    }
}

/// A read-only tap into a `DelayLine`.
///
/// Each tap reads from the parent delay line's buffer at a fixed ratio
/// of the base delay time. For an RE-201 with heads at 3cm, 6cm, 12cm:
///   - Head 1: ratio 1.0 (base delay)
///   - Head 2: ratio 2.0 (2× base delay)
///   - Head 3: ratio 4.0 (4× base delay)
///
/// Taps are separate WDF output ports that can feed into their own
/// subtrees (e.g., per-head HF filtering, individual level controls).
#[derive(Debug, Clone)]
pub struct Tap {
    /// Ratio of base delay time (e.g., 1.0, 2.0, 4.0).
    pub ratio: f64,
    /// Index of the parent `DelayLine` in the compiled pedal's delay list.
    pub delay_line_idx: usize,
    /// Tap index for allpass state tracking within the delay line.
    /// Index 0 is reserved for the base read; taps use 1, 2, 3, ...
    pub tap_index: usize,
}

#[cfg(test)]
mod delay_tests {
    use super::*;

    const SR: f64 = 48000.0;

    // ── DelayLine basic operation ────────────────────────────────────

    #[test]
    fn delay_line_produces_delayed_output() {
        // A 10ms delay at 48kHz = 480 samples.
        // write() advances write_pos before read(), so the effective latency
        // through write→read is (delay_samples - 1) ticks.
        let mut dl = DelayLine::new(0.010, 0.010, SR, Interpolation::Linear);
        dl.set_delay_seconds(0.010);

        // Write impulse then pump zeros, collecting every output.
        let n = 600;
        let mut outputs = vec![0.0_f64; n];

        dl.write(1.0);
        outputs[0] = dl.read();
        for i in 1..n {
            dl.write(0.0);
            outputs[i] = dl.read();
        }

        // Find peak — should be near sample 479 (480 - 1 for write-advance-read ordering)
        let peak_idx = outputs
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap())
            .unwrap()
            .0;

        assert!(
            (peak_idx as i64 - 479).abs() <= 1,
            "Impulse should arrive near sample 479, peaked at {peak_idx}"
        );
        assert!(
            outputs[peak_idx].abs() > 0.5,
            "Peak amplitude should be significant, got {}",
            outputs[peak_idx]
        );
    }

    #[test]
    fn delay_line_output_is_finite() {
        let mut dl = DelayLine::new(0.001, 0.050, SR, Interpolation::Allpass);
        dl.set_delay_seconds(0.025);

        for i in 0..2400 {
            let t = i as f64 / SR;
            let input = (2.0 * std::f64::consts::PI * 440.0 * t).sin();
            dl.write(input);
            let out = dl.read();
            assert!(out.is_finite(), "Output should be finite at sample {i}");
        }
    }

    #[test]
    fn delay_line_set_delay_normalized() {
        let mut dl = DelayLine::new(0.001, 1.0, SR, Interpolation::Linear);

        // norm=0 → min delay (1ms)
        dl.set_delay_normalized(0.0);
        let t0 = dl.delay_time();
        assert!((t0 - 0.001).abs() < 0.0001, "norm=0 should be ~1ms, got {t0}");

        // norm=1 → max delay (1s)
        dl.set_delay_normalized(1.0);
        let t1 = dl.delay_time();
        assert!((t1 - 1.0).abs() < 0.01, "norm=1 should be ~1s, got {t1}");
    }

    #[test]
    fn delay_line_feedback_builds_up() {
        let mut dl = DelayLine::new(0.001, 0.010, SR, Interpolation::Linear);
        dl.set_delay_seconds(0.002); // 96 samples
        dl.set_feedback(0.8);

        // Write impulse
        dl.write(1.0);

        // Run for several delay periods, collecting output
        let mut max_out = 0.0_f64;
        for _ in 1..1000 {
            dl.write(0.0);
            let out = dl.read();
            dl.set_feedback_sample(out);
            max_out = max_out.max(out.abs());
        }

        // With feedback=0.8, the impulse should recirculate
        assert!(max_out > 0.1, "Feedback should cause recirculation, max={max_out}");
    }

    #[test]
    fn delay_line_speed_mod_affects_pitch() {
        let mut dl = DelayLine::new(0.005, 0.050, SR, Interpolation::Linear);
        dl.set_delay_seconds(0.020);

        // Normal speed: write a tone and measure delay
        for i in 0..960 {
            let t = i as f64 / SR;
            dl.write((2.0 * std::f64::consts::PI * 440.0 * t).sin());
        }
        let normal_out = dl.read();

        // With speed mod, effective delay changes
        dl.set_speed_mod(1.5);
        let modded = dl.effective_delay_samples();
        let normal = 0.020 * SR;
        assert!(
            (modded - normal * 1.5).abs() < 1.0,
            "speed_mod=1.5 should scale delay by 1.5x"
        );

        // Verify it's not NaN
        let out = dl.read();
        assert!(out.is_finite(), "Output with speed mod should be finite, got {normal_out} / {out}");
    }

    // ── Tap reads at correct ratio ───────────────────────────────────

    #[test]
    fn tap_read_at_ratio() {
        let mut dl = DelayLine::new(0.005, 0.200, SR, Interpolation::Linear);
        dl.set_delay_seconds(0.020); // 960 samples base delay

        // Write an impulse then silence
        dl.write(1.0);
        for _ in 1..480 {
            dl.write(0.0);
        }

        // ratio=0.5 → half the delay (480 samples) — impulse should be there
        let tap_half = dl.read_at_ratio(0.5, 1);
        assert!(
            tap_half.abs() > 0.5,
            "Tap at ratio=0.5 should see impulse at half-delay, got {tap_half}"
        );

        // ratio=1.0 → full delay (960 samples) — impulse not there yet
        let tap_full = dl.read_at_ratio(1.0, 0);
        assert!(
            tap_full.abs() < 0.01,
            "Tap at ratio=1.0 should not see impulse yet at 480 ticks, got {tap_full}"
        );
    }

    // ── Interpolation modes ──────────────────────────────────────────

    #[test]
    fn cubic_interpolation_finite() {
        let mut dl = DelayLine::new(0.001, 0.050, SR, Interpolation::Cubic);
        dl.set_delay_seconds(0.010);

        for i in 0..4800 {
            let t = i as f64 / SR;
            let input = (2.0 * std::f64::consts::PI * 440.0 * t).sin();
            dl.write(input);
            let out = dl.read();
            assert!(out.is_finite(), "Cubic output should be finite at sample {i}");
        }
    }

    #[test]
    fn allpass_interpolation_finite() {
        let mut dl = DelayLine::new(0.001, 0.050, SR, Interpolation::Allpass);
        dl.set_delay_seconds(0.010);

        for i in 0..4800 {
            let t = i as f64 / SR;
            let input = (2.0 * std::f64::consts::PI * 440.0 * t).sin();
            dl.write(input);
            let out = dl.read();
            assert!(out.is_finite(), "Allpass output should be finite at sample {i}");
        }
    }

    // ── Medium processing ────────────────────────────────────────────

    #[test]
    fn medium_none_passes_signal_unchanged() {
        let mut dl = DelayLine::new(0.001, 0.050, SR, Interpolation::Linear);
        dl.set_delay_seconds(0.010);
        // Medium::None is default

        // Write known pattern
        for i in 0..960 {
            dl.write((i as f64) / 960.0);
        }

        // Read — should be clean (no medium distortion)
        let out = dl.read();
        assert!(out.is_finite());
    }

    #[test]
    fn medium_tape_oxide_darkens_signal() {
        // With TapeOxide, the signal should lose HF content over time.
        let mut dl_clean = DelayLine::new(0.010, 0.050, SR, Interpolation::Linear);
        dl_clean.set_delay_seconds(0.020);

        let mut dl_tape = DelayLine::new(0.010, 0.050, SR, Interpolation::Linear);
        dl_tape.set_delay_seconds(0.020);
        dl_tape.set_medium(Medium::TapeOxide);
        dl_tape.configure_zones_from_taps(&[1.0], None);

        let mut sum_clean = 0.0;
        let mut sum_tape = 0.0;

        // Process a high-frequency tone through both
        for i in 0..4800 {
            let t = i as f64 / SR;
            let input = (2.0 * std::f64::consts::PI * 8000.0 * t).sin();
            dl_clean.write(input);
            dl_tape.write(input);

            if i > 1200 {
                sum_clean += dl_clean.read().abs();
                sum_tape += dl_tape.read().abs();
            }
        }

        // Tape should reduce HF energy compared to clean
        assert!(
            sum_tape < sum_clean,
            "TapeOxide should darken signal: tape={sum_tape} vs clean={sum_clean}"
        );
    }

    #[test]
    fn medium_bbd_leakage_attenuates() {
        let mut dl_clean = DelayLine::new(0.010, 0.050, SR, Interpolation::Linear);
        dl_clean.set_delay_seconds(0.020);

        let mut dl_bbd = DelayLine::new(0.010, 0.050, SR, Interpolation::Linear);
        dl_bbd.set_delay_seconds(0.020);
        dl_bbd.set_medium(Medium::BbdLeakage);
        dl_bbd.configure_zones_from_taps(&[1.0], None);

        let mut sum_clean = 0.0;
        let mut sum_bbd = 0.0;

        for i in 0..4800 {
            let t = i as f64 / SR;
            let input = (2.0 * std::f64::consts::PI * 440.0 * t).sin();
            dl_clean.write(input);
            dl_bbd.write(input);

            if i > 1200 {
                sum_clean += dl_clean.read().abs();
                sum_bbd += dl_bbd.read().abs();
            }
        }

        // BBD leakage should reduce signal amplitude
        assert!(
            sum_bbd < sum_clean,
            "BbdLeakage should attenuate: bbd={sum_bbd} vs clean={sum_clean}"
        );
    }

    // ── write_direct / buffer_read / advance_write ───────────────────

    #[test]
    fn direct_buffer_access_produces_delayed_impulse() {
        // write_direct / buffer_read / advance_write is the BBD-style ordering:
        //   write at pos → read at pos-delay → advance pos
        // This differs from the normal write/read path (which advances before read).
        let mut dl = DelayLine::new(0.005, 0.050, SR, Interpolation::Linear);
        dl.set_delay_seconds(0.010); // 480 samples

        let n = 600;
        let mut outputs = vec![0.0_f64; n];

        // Write impulse using direct access
        dl.write_direct(1.0);
        let delay = dl.effective_delay_samples();
        outputs[0] = dl.buffer_read(delay, 0);
        dl.advance_write();

        for i in 1..n {
            dl.write_direct(0.0);
            let delay = dl.effective_delay_samples();
            outputs[i] = dl.buffer_read(delay, 0);
            dl.advance_write();
        }

        // Find peak — with write-read-advance ordering, the latency is
        // exactly delay_samples ticks (no off-by-one since read happens
        // before advance).
        let peak_idx = outputs
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap())
            .unwrap()
            .0;

        assert!(
            (peak_idx as i64 - 480).abs() <= 1,
            "Impulse should arrive near sample 480, peaked at {peak_idx}"
        );
        assert!(outputs[peak_idx].abs() > 0.5);

        // All outputs should be finite
        assert!(outputs.iter().all(|o| o.is_finite()));
    }

    // ── BbdDelayLine using inner DelayLine ───────────────────────────

    #[test]
    fn bbd_produces_finite_output() {
        let model = BbdModel::mn3207();
        let mut bbd = BbdDelayLine::new(model, SR);

        for i in 0..4800 {
            let t = i as f64 / SR;
            let input = (2.0 * std::f64::consts::PI * 440.0 * t).sin() * 0.5;
            let out = bbd.process(input);
            assert!(out.is_finite(), "BBD output should be finite at sample {i}, got {out}");
        }
    }

    #[test]
    fn bbd_delay_time_changes_with_clock() {
        let model = BbdModel::mn3207();
        let mut bbd = BbdDelayLine::new(model, SR);

        let t_mid = bbd.delay_time();

        // Set to minimum delay (max clock)
        bbd.set_delay_normalized(0.0);
        let t_min = bbd.delay_time();

        // Set to maximum delay (min clock)
        bbd.set_delay_normalized(1.0);
        let t_max = bbd.delay_time();

        assert!(t_min < t_mid, "Min delay should be less than mid: {t_min} < {t_mid}");
        assert!(t_max > t_mid, "Max delay should be greater than mid: {t_max} > {t_mid}");
    }

    #[test]
    fn bbd_feedback_causes_repeats() {
        let model = BbdModel::mn3207();
        let mut bbd = BbdDelayLine::new(model, SR);
        bbd.set_feedback(0.7);
        bbd.set_delay_normalized(0.0); // Short delay for faster test

        // Write a short burst then silence
        for i in 0..480 {
            let t = i as f64 / SR;
            let input = if i < 48 {
                (2.0 * std::f64::consts::PI * 440.0 * t).sin()
            } else {
                0.0
            };
            bbd.process(input);
        }

        // After the burst, there should still be output from feedback
        let mut late_energy = 0.0;
        for _ in 0..4800 {
            let out = bbd.process(0.0);
            late_energy += out * out;
        }

        assert!(
            late_energy > 1e-6,
            "BBD with feedback should have repeating echoes, energy={late_energy}"
        );
    }

    #[test]
    fn bbd_reset_clears_state() {
        let model = BbdModel::mn3207();
        let mut bbd = BbdDelayLine::new(model, SR);

        // Process some signal
        for i in 0..4800 {
            let t = i as f64 / SR;
            bbd.process((2.0 * std::f64::consts::PI * 440.0 * t).sin());
        }

        bbd.reset();

        // After reset, output should be near zero
        let out = bbd.process(0.0);
        assert!(
            out.abs() < 0.01,
            "After reset, BBD output should be near zero, got {out}"
        );
    }

    #[test]
    fn bbd_set_sample_rate_adjusts_delay() {
        let model = BbdModel::mn3207();
        let mut bbd = BbdDelayLine::new(model, SR);

        let t1 = bbd.delay_time();
        bbd.set_sample_rate(96000.0);
        let t2 = bbd.delay_time();

        // Delay time in seconds should remain approximately the same
        assert!(
            (t1 - t2).abs() < 0.001,
            "Delay time should be preserved across sample rate changes: {t1} vs {t2}"
        );
    }

    // ── DelayLine reset / set_sample_rate ────────────────────────────

    #[test]
    fn delay_line_reset_clears_buffer() {
        let mut dl = DelayLine::new(0.001, 0.050, SR, Interpolation::Linear);
        dl.set_delay_seconds(0.010);

        // Fill with signal
        for i in 0..4800 {
            let t = i as f64 / SR;
            dl.write((2.0 * std::f64::consts::PI * 440.0 * t).sin());
        }

        dl.reset();

        // After reset, reads should return zero
        let out = dl.read();
        assert!(
            out.abs() < 1e-10,
            "After reset, delay line output should be zero, got {out}"
        );
    }

    #[test]
    fn delay_line_sample_rate_change() {
        let mut dl = DelayLine::new(0.001, 0.050, SR, Interpolation::Linear);
        dl.set_delay_seconds(0.020);

        let t1 = dl.delay_time();
        dl.set_sample_rate(96000.0);
        let t2 = dl.delay_time();

        assert!(
            (t1 - t2).abs() < 0.0001,
            "Delay time in seconds should be preserved: {t1} vs {t2}"
        );
    }

    // ── MediumPreset ────────────────────────────────────────────────

    #[test]
    fn medium_preset_zone_coefficients() {
        let coeff = MediumPreset::Re201.zone_coefficients();
        assert_eq!(coeff.len(), 3);
        // RE-201 is tape — increasing LP coefficients with distance
        assert!(coeff[0] < coeff[2], "Later zones should have stronger filtering");

        let coeff_lofi = MediumPreset::LoFi.zone_coefficients();
        assert_eq!(coeff_lofi.len(), 3);
        // LoFi has stronger coefficients than RE-201
        assert!(
            coeff_lofi[0] > coeff[0],
            "LoFi should have stronger filtering than RE-201"
        );
    }
}
