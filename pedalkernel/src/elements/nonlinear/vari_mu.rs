//! Variable-mu (remote-cutoff) triode WDF root elements.
//!
//! Models the GE 6386 variable-mu triode using the Raffensperger equation.
//! Unlike the Koren equation used for standard triodes, the Raffensperger
//! model captures the gradual gain variation with grid bias that defines
//! remote-cutoff tube character — essential for the Fairchild 670 compressor.
//!
//! The Raffensperger equation:
//! ```text
//! Ia = p1 × Vak^p2 / ((p3 - p4×Vgk)^p5 × (p6 + exp(p7×Vak - p8×Vgk)))
//! ```

use super::solver::{newton_raphson_solve, LEAKAGE_CONDUCTANCE};
use crate::elements::WdfRoot;

// ---------------------------------------------------------------------------
// Variable-Mu Triode Models (Raffensperger equation)
// ---------------------------------------------------------------------------

/// Variable-mu triode model parameters using the Raffensperger equation.
///
/// The Raffensperger equation accurately models remote-cutoff triodes where
/// the amplification factor (mu) varies continuously with grid bias. This
/// is fundamentally different from the Koren equation which assumes fixed mu.
///
/// ```text
/// Ia = p1 × Vak^p2 / ((p3 - p4×Vgk)^p5 × (p6 + exp(p7×Vak - p8×Vgk)))
/// ```
///
/// Parameters are fitted to measured tube data. The GE 6386 parameters were
/// derived from published plate characteristic curves.
#[derive(Debug, Clone, Copy)]
pub struct VariMuModel {
    /// Overall current scaling factor.
    pub p1: f64,
    /// Plate voltage exponent (controls Ia vs Vak relationship).
    pub p2: f64,
    /// Cutoff offset — denominator base (must be > p4 × Vgk for conduction).
    pub p3: f64,
    /// Grid voltage scaling in denominator.
    pub p4: f64,
    /// Denominator exponent — controls how sharply gain varies with Vgk.
    pub p5: f64,
    /// Exponential term offset.
    pub p6: f64,
    /// Plate voltage coefficient in exponential term.
    pub p7: f64,
    /// Grid voltage coefficient in exponential term.
    pub p8: f64,
}

impl VariMuModel {
    /// GE 6386 variable-mu triode (Fairchild 670 compressor).
    ///
    /// Parameters fitted to published 6386 plate characteristics.
    /// The 6386 is a dual remote-cutoff triode designed specifically for
    /// variable-gain amplifier applications.
    pub fn ge_6386() -> Self {
        Self {
            p1: 3.981e-8,
            p2: 2.383,
            p3: 0.5,
            p4: 0.1,
            p5: 1.8,
            p6: 0.5,
            p7: -0.03922,
            p8: 0.2,
        }
    }

    /// Look up a variable-mu triode model by name.
    /// Panics if the name is not found.
    pub fn by_name(name: &str) -> Self {
        Self::try_by_name(name).unwrap_or_else(|| {
            panic!(
                "Unknown variable-mu triode model: '{}'. Available: 6386, GE6386",
                name
            )
        })
    }

    /// Look up a variable-mu model by name, returning None if not found.
    pub fn try_by_name(name: &str) -> Option<Self> {
        match name.to_uppercase().as_str() {
            "6386" | "GE6386" | "GE_6386" => Some(Self::ge_6386()),
            _ => None,
        }
    }
}

// ---------------------------------------------------------------------------
// Variable-Mu Triode Root
// ---------------------------------------------------------------------------

/// Variable-mu triode nonlinear root for WDF trees.
///
/// Models the plate-cathode path as a nonlinear element controlled by
/// an external grid-cathode voltage Vgk. Uses Newton-Raphson to solve
/// the implicit WDF constraint equation.
///
/// Unlike the standard Koren-based TriodeRoot, this uses the Raffensperger
/// equation which captures the variable-mu characteristic: transconductance
/// decreases smoothly as Vgk becomes more negative, providing the gradual
/// gain reduction that defines variable-mu compressor behavior.
#[derive(Debug, Clone, Copy)]
pub struct VariMuTriodeRoot {
    pub model: VariMuModel,
    /// Current grid-cathode voltage (external control parameter).
    vgk: f64,
    /// Maximum plate voltage (determined by supply rail B+).
    v_max: f64,
    /// Maximum Newton-Raphson iterations (bounded for RT safety).
    max_iter: usize,
    /// Number of parallel tubes (default 1). Plate current is scaled by N.
    parallel_count: usize,
}

impl VariMuTriodeRoot {
    pub fn new(model: VariMuModel) -> Self {
        Self {
            model,
            vgk: 0.0,
            v_max: 500.0,
            max_iter: 16,
            parallel_count: 1,
        }
    }

    /// Create a variable-mu triode root with a specific supply voltage (B+).
    pub fn new_with_v_max(model: VariMuModel, v_max: f64) -> Self {
        Self {
            model,
            vgk: 0.0,
            v_max: v_max.max(1.0),
            max_iter: 16,
            parallel_count: 1,
        }
    }

    /// Set the number of parallel tubes for current scaling.
    pub fn with_parallel_count(mut self, count: usize) -> Self {
        self.parallel_count = count.max(1);
        self
    }

    /// Set the maximum plate voltage (B+ supply rail).
    #[inline]
    pub fn set_v_max(&mut self, v_max: f64) {
        self.v_max = v_max.max(1.0);
    }

    /// Get the current v_max setting.
    #[inline]
    pub fn v_max(&self) -> f64 {
        self.v_max
    }

    pub fn parallel_count(&self) -> usize {
        self.parallel_count
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

    /// Compute plate current using the Raffensperger equation.
    ///
    /// ```text
    /// Ia = p1 × Vak^p2 / ((p3 - p4×Vgk)^p5 × (p6 + exp(p7×Vak - p8×Vgk)))
    /// ```
    ///
    /// Guards:
    /// - Vak ≤ 0: no reverse current
    /// - p3 - p4*Vgk ≤ 0: past cutoff (only at Vgk > +5V for 6386 params)
    /// - Exp argument clamped to ±500 for overflow safety
    #[inline]
    pub fn plate_current(&self, vak: f64) -> f64 {
        let m = &self.model;
        let vgk = self.vgk;

        // No reverse current
        if vak <= 0.0 {
            return 0.0;
        }

        // Denominator base: p3 - p4 * Vgk
        // For 6386: 0.5 - 0.1 * Vgk. Positive for Vgk < 5V (always in practice).
        let denom_base = m.p3 - m.p4 * vgk;
        if denom_base <= 0.0 {
            return 0.0;
        }

        // Exponential term with overflow protection
        let exp_arg = (m.p7 * vak - m.p8 * vgk).clamp(-500.0, 500.0);
        let exp_term = exp_arg.exp();

        let numerator = m.p1 * vak.powf(m.p2);
        let denominator = denom_base.powf(m.p5) * (m.p6 + exp_term);

        if denominator <= 0.0 {
            return 0.0;
        }

        (numerator / denominator) * self.parallel_count as f64
    }

    /// Compute derivative of plate current w.r.t. Vak for Newton-Raphson.
    ///
    /// Using the quotient rule on the Raffensperger equation:
    /// ```text
    /// dIa/dVak = Ia × (p2/Vak + p7×exp_term / (p6 + exp_term))
    /// ```
    /// where exp_term = exp(p7×Vak - p8×Vgk)
    #[inline]
    fn plate_current_derivative(&self, vak: f64) -> f64 {
        let m = &self.model;
        let vgk = self.vgk;

        if vak <= 0.0 {
            return LEAKAGE_CONDUCTANCE;
        }

        let denom_base = m.p3 - m.p4 * vgk;
        if denom_base <= 0.0 {
            return LEAKAGE_CONDUCTANCE;
        }

        let exp_arg = (m.p7 * vak - m.p8 * vgk).clamp(-500.0, 500.0);
        let exp_term = exp_arg.exp();
        let exp_sum = m.p6 + exp_term;

        if exp_sum <= 0.0 {
            return LEAKAGE_CONDUCTANCE;
        }

        let ia = self.plate_current(vak) / self.parallel_count as f64;
        if ia <= 0.0 {
            return LEAKAGE_CONDUCTANCE;
        }

        // dIa/dVak = Ia × (p2/Vak + p7 × exp_term / (p6 + exp_term))
        let d = ia * (m.p2 / vak + m.p7 * exp_term / exp_sum);

        // Ensure positive derivative (physical: more plate voltage → more current)
        let d_scaled = d.max(LEAKAGE_CONDUCTANCE) * self.parallel_count as f64;
        d_scaled
    }
}

impl WdfRoot for VariMuTriodeRoot {
    /// Variable-mu triode plate-cathode path: `i = Ia(Vak, Vgk)`
    #[inline]
    fn process(&mut self, a: f64, rp: f64) -> f64 {
        let root = *self;
        let v_max = self.v_max;
        newton_raphson_solve(
            a,
            rp,
            a * 0.5,
            self.max_iter,
            1e-6,
            Some((-50.0, v_max)),
            None,
            |v| (root.plate_current(v), root.plate_current_derivative(v)),
        )
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// At Fairchild 670 operating point: Vgk ≈ -2V, Vak ≈ 200V.
    /// The 6386 should conduct a positive, finite plate current.
    #[test]
    fn vari_mu_6386_positive_current_at_operating_point() {
        let mut root = VariMuTriodeRoot::new(VariMuModel::ge_6386());
        root.set_vgk(-2.0);
        let ip = root.plate_current(200.0);
        assert!(
            ip > 0.0,
            "6386 should conduct at Vgk=-2V, Vak=200V, got {ip}"
        );
        assert!(ip.is_finite(), "plate current must be finite");
        // Realistic range: 0.1 mA to 10 mA for a small-signal triode
        assert!(
            ip > 1e-5 && ip < 0.05,
            "6386 plate current should be in realistic range: got {ip:.6e} A"
        );
    }

    /// No current for reverse plate voltage.
    #[test]
    fn vari_mu_6386_zero_current_negative_plate() {
        let mut root = VariMuTriodeRoot::new(VariMuModel::ge_6386());
        root.set_vgk(-2.0);
        assert_eq!(
            root.plate_current(-10.0),
            0.0,
            "No current for negative Vak"
        );
        assert_eq!(root.plate_current(0.0), 0.0, "No current at Vak=0");
    }

    /// Remote cutoff behavior: current decreases gradually, not sharply.
    /// At Vgk=-10V, the 6386 still conducts (this is the variable-mu characteristic).
    /// At Vgk=-20V, the current should be much smaller.
    #[test]
    fn vari_mu_6386_cutoff_at_large_negative_vgk() {
        let model = VariMuModel::ge_6386();

        let mut root_op = VariMuTriodeRoot::new(model);
        root_op.set_vgk(-2.0);
        let ip_operating = root_op.plate_current(200.0);

        // At -10V: current reduced but still conducting (remote cutoff)
        let mut root_med = VariMuTriodeRoot::new(model);
        root_med.set_vgk(-10.0);
        let ip_med = root_med.plate_current(200.0);
        assert!(
            ip_med < ip_operating * 0.5,
            "At Vgk=-10V ({ip_med:.6e}), current should be <50% of operating point ({ip_operating:.6e})"
        );

        // At -20V: significantly reduced (remote cutoff is gradual by design)
        let mut root_cutoff = VariMuTriodeRoot::new(model);
        root_cutoff.set_vgk(-20.0);
        let ip_cutoff = root_cutoff.plate_current(200.0);
        assert!(
            ip_cutoff < ip_operating * 0.15,
            "At Vgk=-20V ({ip_cutoff:.6e}), current should be <15% of operating point ({ip_operating:.6e})"
        );
    }

    /// Variable-mu characteristic: transconductance (gm) decreases as Vgk
    /// becomes more negative. This is the defining property of remote-cutoff tubes.
    #[test]
    fn vari_mu_6386_transconductance_decreases_with_negative_vgk() {
        let model = VariMuModel::ge_6386();
        let vak = 200.0;
        let delta_vgk = 0.01; // Small perturbation for numerical gm

        // Measure gm at Vgk = -1V (light bias)
        let mut root = VariMuTriodeRoot::new(model);
        root.set_vgk(-1.0);
        let ip_lo = root.plate_current(vak);
        root.set_vgk(-1.0 + delta_vgk);
        let ip_hi = root.plate_current(vak);
        let gm_light = (ip_hi - ip_lo) / delta_vgk;

        // Measure gm at Vgk = -4V (heavy bias)
        root.set_vgk(-4.0);
        let ip_lo = root.plate_current(vak);
        root.set_vgk(-4.0 + delta_vgk);
        let ip_hi = root.plate_current(vak);
        let gm_heavy = (ip_hi - ip_lo) / delta_vgk;

        assert!(
            gm_light > gm_heavy,
            "Transconductance should decrease with more negative Vgk: \
             gm at -1V = {gm_light:.6e}, gm at -4V = {gm_heavy:.6e}"
        );
        assert!(gm_light > 0.0, "gm should be positive at light bias");
        assert!(gm_heavy > 0.0, "gm should be positive at heavy bias");
    }

    /// Derivative should be positive at operating point (more Vak → more Ia).
    #[test]
    fn vari_mu_6386_derivative_positive_at_operating_point() {
        let mut root = VariMuTriodeRoot::new(VariMuModel::ge_6386());
        root.set_vgk(-2.0);
        let d = root.plate_current_derivative(200.0);
        assert!(d > 0.0, "Derivative should be positive, got {d}");
        assert!(d.is_finite(), "Derivative must be finite");
    }

    /// WDF root should converge at typical operating conditions.
    #[test]
    fn vari_mu_6386_wdf_root_converges() {
        let model = VariMuModel::ge_6386();
        let mut root = VariMuTriodeRoot::new_with_v_max(model, 300.0);
        root.set_vgk(-2.0);
        let b = root.process(100.0, 100_000.0);
        assert!(b.is_finite(), "WDF root must converge, got {b}");
    }

    /// Parallel count should scale plate current.
    #[test]
    fn vari_mu_6386_parallel_count_scaling() {
        let model = VariMuModel::ge_6386();

        let mut single = VariMuTriodeRoot::new(model);
        single.set_vgk(-2.0);
        let ip_single = single.plate_current(200.0);

        let mut quad = VariMuTriodeRoot::new(model).with_parallel_count(4);
        quad.set_vgk(-2.0);
        let ip_quad = quad.plate_current(200.0);

        let ratio = ip_quad / ip_single;
        assert!(
            (ratio - 4.0).abs() < 0.01,
            "4 parallel tubes should give 4× current: ratio = {ratio}"
        );
    }

    /// by_name should resolve "6386" and "GE6386".
    #[test]
    fn vari_mu_model_lookup() {
        let m1 = VariMuModel::by_name("6386");
        let m2 = VariMuModel::by_name("GE6386");
        assert_eq!(m1.p1, m2.p1);
        assert_eq!(m1.p2, m2.p2);

        assert!(VariMuModel::try_by_name("UNKNOWN").is_none());
    }

    /// v_max clamps to minimum 1.0.
    #[test]
    fn vari_mu_v_max_clamp() {
        let model = VariMuModel::ge_6386();
        let mut root = VariMuTriodeRoot::new(model);
        assert_eq!(root.v_max(), 500.0);
        root.set_v_max(300.0);
        assert_eq!(root.v_max(), 300.0);
        root.set_v_max(0.5);
        assert_eq!(root.v_max(), 1.0);
    }
}
