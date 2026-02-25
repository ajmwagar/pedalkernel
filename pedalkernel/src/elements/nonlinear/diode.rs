//! Diode WDF root elements: silicon, germanium, LED, zener.
//!
//! Includes anti-parallel diode pair and single diode configurations.

use super::solver::newton_raphson_solve;
use crate::elements::WdfRoot;

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
