//! WDF circuit elements — one-port leaves and nonlinear roots.
//!
//! One-port (leaf) elements absorb an incident wave `a` and produce a
//! reflected wave `b`.  The port resistance `Rp` is used by parent
//! adaptors to compute scattering coefficients.
//!
//! Nonlinear elements sit at the tree root and use Newton-Raphson
//! iteration bounded to a fixed number of steps for RT safety.

// ---------------------------------------------------------------------------
// One-port elements (leaves)
// ---------------------------------------------------------------------------

/// Ideal resistor — absorbs everything, reflects nothing.
///
/// `b = 0`  (matched termination when Rp == R)
#[derive(Debug, Clone, Copy)]
pub struct Resistor {
    pub port_resistance: f64,
}

impl Resistor {
    pub fn new(resistance: f64) -> Self {
        Self {
            port_resistance: resistance,
        }
    }

    #[inline]
    pub fn reflected(&self) -> f64 {
        0.0
    }

    #[inline]
    pub fn set_incident(&mut self, _a: f64) {
        // resistor has no state to update
    }
}

/// Capacitor — energy-storage element.
///
/// `b[n] = z^{-1} a[n]`  (previous incident becomes current reflected)
/// `Rp = 1 / (2 * fs * C)`
#[derive(Debug, Clone, Copy)]
pub struct Capacitor {
    capacitance: f64,
    pub port_resistance: f64,
    state: f64, // z^{-1} of incident wave
}

impl Capacitor {
    pub fn new(capacitance: f64, sample_rate: f64) -> Self {
        Self {
            capacitance,
            port_resistance: 1.0 / (2.0 * sample_rate * capacitance),
            state: 0.0,
        }
    }

    #[inline]
    pub fn reflected(&self) -> f64 {
        self.state
    }

    /// Call *after* scatter_down delivers the incident wave.
    #[inline]
    pub fn set_incident(&mut self, a: f64) {
        self.state = a;
    }

    pub fn update_sample_rate(&mut self, fs: f64) {
        self.port_resistance = 1.0 / (2.0 * fs * self.capacitance);
    }

    pub fn reset(&mut self) {
        self.state = 0.0;
    }
}

/// Inductor — energy-storage element.
///
/// `b[n] = -z^{-1} a[n]`
/// `Rp = 2 * fs * L`
#[derive(Debug, Clone, Copy)]
pub struct Inductor {
    inductance: f64,
    pub port_resistance: f64,
    state: f64,
}

impl Inductor {
    pub fn new(inductance: f64, sample_rate: f64) -> Self {
        Self {
            inductance,
            port_resistance: 2.0 * sample_rate * inductance,
            state: 0.0,
        }
    }

    #[inline]
    pub fn reflected(&self) -> f64 {
        -self.state
    }

    #[inline]
    pub fn set_incident(&mut self, a: f64) {
        self.state = a;
    }

    pub fn update_sample_rate(&mut self, fs: f64) {
        self.port_resistance = 2.0 * fs * self.inductance;
    }

    pub fn reset(&mut self) {
        self.state = 0.0;
    }
}

/// Ideal voltage source (e.g. input signal injector).
///
/// `b = 2 * Vs - a`  where Vs is the source voltage.
/// Port resistance set to a small value (near-ideal source).
#[derive(Debug, Clone, Copy)]
pub struct VoltageSource {
    voltage: f64,
    pub port_resistance: f64,
}

impl VoltageSource {
    pub fn new(port_resistance: f64) -> Self {
        Self {
            voltage: 0.0,
            port_resistance,
        }
    }

    pub fn set_voltage(&mut self, v: f64) {
        self.voltage = v;
    }

    #[inline]
    pub fn reflected(&self) -> f64 {
        2.0 * self.voltage
    }

    #[inline]
    pub fn set_incident(&mut self, _a: f64) {}
}

// ---------------------------------------------------------------------------
// Nonlinear root elements
// ---------------------------------------------------------------------------

/// Diode model parameters derived from the Shockley equation.
#[derive(Debug, Clone, Copy)]
pub struct DiodeModel {
    /// Saturation current (A). Silicon ≈ 1e-15, Germanium ≈ 1e-6.
    pub is: f64,
    /// Thermal voltage * ideality factor (V).  Vt ≈ 25.85 mV at 20 °C.
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
    pub fn process(&self, a: f64, rp: f64) -> f64 {
        let is = self.model.is;
        let nvt = self.model.n_vt;

        // Initial guess: for large |a| the diode is saturated and we can
        // estimate v from the Shockley equation in the forward-bias regime
        // (2*Rp*Is*exp(|v|/nVt) ≈ |a|).  For small |a| the linear
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

    /// `f(v) = a - 2v - 2*Rp*Is*(exp(v/nVt) - 1)`
    #[inline]
    pub fn process(&self, a: f64, rp: f64) -> f64 {
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
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resistor_reflects_zero() {
        let r = Resistor::new(1000.0);
        assert_eq!(r.reflected(), 0.0);
    }

    #[test]
    fn capacitor_port_resistance() {
        let c = Capacitor::new(220e-9, 48000.0);
        let expected = 1.0 / (2.0 * 48000.0 * 220e-9);
        assert!((c.port_resistance - expected).abs() < 1e-3);
    }

    #[test]
    fn capacitor_reflects_previous_incident() {
        let mut c = Capacitor::new(220e-9, 48000.0);
        assert_eq!(c.reflected(), 0.0);
        c.set_incident(0.5);
        assert_eq!(c.reflected(), 0.5);
    }

    #[test]
    fn inductor_port_resistance() {
        let l = Inductor::new(0.1, 48000.0);
        let expected = 2.0 * 48000.0 * 0.1;
        assert!((l.port_resistance - expected).abs() < 1e-3);
    }

    #[test]
    fn inductor_reflects_negated_previous() {
        let mut l = Inductor::new(0.1, 48000.0);
        l.set_incident(0.5);
        assert_eq!(l.reflected(), -0.5);
    }

    #[test]
    fn voltage_source_reflection() {
        let mut vs = VoltageSource::new(1.0);
        vs.set_voltage(1.0);
        assert_eq!(vs.reflected(), 2.0);
    }

    #[test]
    fn diode_pair_zero_input() {
        let dp = DiodePairRoot::new(DiodeModel::silicon());
        let b = dp.process(0.0, 1000.0);
        assert!(
            b.abs() < 1e-6,
            "symmetric diode pair should reflect ~0 for zero input"
        );
    }

    #[test]
    fn diode_pair_clips() {
        let dp = DiodePairRoot::new(DiodeModel::silicon());
        let b_small = dp.process(0.01, 1000.0);
        let b_large = dp.process(10.0, 1000.0);
        // The diode voltage v = (a + b) / 2 is what gets clipped.
        let v_small = (0.01 + b_small) / 2.0;
        let v_large = (10.0 + b_large) / 2.0;
        // Diode voltage should be bounded (silicon: < 2V regardless of input).
        assert!(
            v_large.abs() < 2.0,
            "diode voltage should be bounded: {v_large}"
        );
        // Relative clipping: v/a ratio should shrink for larger inputs.
        assert!(
            v_large.abs() / 10.0 < v_small.abs() / 0.01,
            "large input should be clipped harder: \
             v_small/a_small={:.4}, v_large/a_large={:.4}",
            v_small.abs() / 0.01,
            v_large.abs() / 10.0,
        );
    }

    #[test]
    fn single_diode_asymmetric() {
        let d = DiodeRoot::new(DiodeModel::silicon());
        let b_pos = d.process(1.0, 1000.0);
        let b_neg = d.process(-1.0, 1000.0);
        // Single diode should produce different reflections for + vs -
        assert!(
            (b_pos - b_neg).abs() > 1e-10,
            "should be asymmetric: b+={b_pos}, b-={b_neg}"
        );
    }
}
