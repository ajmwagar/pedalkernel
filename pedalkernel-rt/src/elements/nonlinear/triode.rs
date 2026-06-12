//! Triode vacuum tube WDF root elements.
//!
//! Models preamp tubes (12AX7, 12AT7, 12AU7) using the Koren equation.
//! Parameters are loaded from the embedded `triodes.model` file.

use super::solver::{
    newton_raphson_solve, softplus, NlDeviceGroupIv, NlDeviceIv, LEAKAGE_CONDUCTANCE,
};
use crate::elements::WdfRoot;
// ---------------------------------------------------------------------------
// Triode (Vacuum Tube) Models
// ---------------------------------------------------------------------------

/// Triode model parameters using the Koren equation.
///
/// Models common preamp tubes (12AX7, 12AT7, 12AU7) with the industry-standard
/// Koren model for accurate tube amplifier simulation.
///
/// The Koren equation:
/// `Ip = (Vpk/Kp * ln(1 + exp(Kp * (1/mu + Vgk/sqrt(Kvb + Vpk^2)))))^Ex / KG1`
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TriodeModel {
    /// Amplification factor (mu). Higher = more gain. 12AX7 ≈ 100, 12AU7 ≈ 20.
    pub mu: crate::Wave,
    /// Plate resistance factor. Affects output impedance.
    pub kp: crate::Wave,
    /// Knee voltage constant. Affects saturation behavior.
    pub kvb: crate::Wave,
    /// Exponent (typically 1.3-1.5). Affects transfer curve shape.
    pub ex: crate::Wave,
    /// Plate current scaling factor (KG1). Scales the absolute plate current
    /// magnitude. 12AX7 ≈ 1060, 12AU7 ≈ 1180.
    pub kg1: crate::Wave,
    /// Plate resistance at typical operating point (Ω).
    /// Used as virtual resistance in WDF tree construction.
    /// 12AX7 ≈ 62.5kΩ, 12AU7 ≈ 7.7kΩ, 12AT7 ≈ 10.9kΩ.
    pub rp: crate::Wave,
}

impl TriodeModel {
    // TODO: move to pedalkernel as extension impl
    // pub fn by_name(name: &str) -> Self { ... }
    // pub fn try_by_name(name: &str) -> Option<Self> { ... }
}

// TODO: move to pedalkernel as extension impl
// impl From<&SpiceTriodeModel> for TriodeModel { ... }

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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TriodeRoot {
    pub model: TriodeModel,
    /// DC grid-cathode bias voltage, set at compile time from circuit analysis.
    /// The runtime input signal modulates around this operating point.
    vgk_bias: crate::Wave,
    /// Current grid-cathode voltage (bias + AC signal).
    vgk: crate::Wave,
    /// Maximum plate voltage (determined by supply rail B+).
    /// Triode plate can swing from 0V (saturated) to B+ (cutoff).
    v_max: crate::Wave,
    /// Maximum Newton-Raphson iterations (bounded for RT safety).
    max_iter: usize,
    /// Number of parallel tubes (default 1). Plate current is scaled by N.
    parallel_count: usize,
    /// Previous sample's plate voltage for warm-starting Newton-Raphson.
    prev_v: crate::Wave,
}

impl TriodeRoot {
    pub fn new(model: TriodeModel) -> Self {
        Self {
            model,
            vgk_bias: -2.0, // Default: typical 12AX7 bias
            vgk: -2.0,
            v_max: 500.0,
            max_iter: super::solver::NR_MAX_ITER,
            parallel_count: 1,
            prev_v: 0.0,
        }
    }

    /// Create a triode root with a specific supply voltage (B+).
    pub fn new_with_v_max(model: TriodeModel, v_max: crate::Wave) -> Self {
        Self {
            model,
            vgk_bias: -2.0,
            vgk: -2.0,
            v_max: v_max.max(1.0),
            max_iter: super::solver::NR_MAX_ITER,
            parallel_count: 1,
            prev_v: 0.0,
        }
    }

    /// Set the number of parallel tubes for current scaling.
    pub fn with_parallel_count(mut self, count: usize) -> Self {
        self.parallel_count = count.max(1);
        self
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
    pub fn set_v_max(&mut self, v_max: crate::Wave) {
        self.v_max = v_max.max(1.0); // Minimum 1V to avoid degeneracy
    }

    /// Get the current v_max setting.
    #[inline]
    pub fn v_max(&self) -> crate::Wave {
        self.v_max
    }

    pub fn parallel_count(&self) -> usize {
        self.parallel_count
    }

    /// Set the DC bias operating point from circuit analysis.
    pub fn set_bias(&mut self, vgk_bias: crate::Wave) {
        self.vgk_bias = vgk_bias;
        self.vgk = vgk_bias;
    }

    /// Get the DC bias operating point.
    pub fn vgk_bias(&self) -> crate::Wave {
        self.vgk_bias
    }

    /// Set the grid-cathode voltage (external control from bias, signal, LFO).
    #[inline]
    pub fn set_vgk(&mut self, vgk: crate::Wave) {
        self.vgk = vgk;
    }

    /// Get current grid-cathode voltage.
    #[inline]
    pub fn vgk(&self) -> crate::Wave {
        self.vgk
    }

    /// Compute plate current for given Vpk at current Vgk using Koren model.
    ///
    /// The Koren equation:
    /// `Ip = (Vpk/Kp * ln(1 + exp(Kp * (1/mu + Vgk/sqrt(Kvb + Vpk^2)))))^Ex / KG1`
    #[inline]
    pub fn plate_current(&self, vpk: crate::Wave) -> crate::Wave {
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
        let e1 = kp * (1.0 / mu + vgk / crate::math::sqrt(kvb + vpk * vpk));

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

        // Ip = base^Ex / KG1, scaled by parallel_count for N tubes in parallel.
        (crate::math::powf(base, ex) / self.model.kg1) * self.parallel_count as crate::Wave
    }

    /// Compute derivative of plate current w.r.t. Vpk for Newton-Raphson.
    #[inline]
    fn plate_current_derivative(&self, vpk: crate::Wave) -> crate::Wave {
        let mu = self.model.mu;
        let kp = self.model.kp;
        let kvb = self.model.kvb;
        let ex = self.model.ex;
        let vgk = self.vgk;

        if vpk <= 0.0 {
            return LEAKAGE_CONDUCTANCE; // Small conductance to avoid division issues
        }

        let vpk_sq = vpk * vpk;
        let sqrt_term = crate::math::sqrt(kvb + vpk_sq);
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
            let exp_e1 = crate::math::exp(e1);
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

        // d(base^Ex / KG1)/dVpk = Ex * base^(Ex-1) * dbase_dvpk / KG1
        // Scale by parallel_count to match plate_current() scaling.
        (ex * crate::math::powf(base, ex - 1.0) * dbase_dvpk / self.model.kg1)
            * self.parallel_count as crate::Wave
    }
}

impl WdfRoot for TriodeRoot {
    /// Triode plate-cathode path with warm-starting.
    #[inline]
    fn process(&mut self, a: crate::Wave, rp: crate::Wave) -> crate::Wave {
        let root = *self;
        let v_max = self.v_max;
        let cold = a * 0.5;
        let v0 = if self.prev_v != 0.0
            && (self.prev_v - cold).abs() < v_max
            && self.prev_v.abs() < v_max
        {
            self.prev_v
        } else {
            cold
        };
        let b = newton_raphson_solve(
            a,
            rp,
            v0,
            self.max_iter,
            1e-6,
            Some((-50.0, v_max)),
            None,
            |v| (root.plate_current(v), root.plate_current_derivative(v)),
        );
        self.prev_v = (a + b) * 0.5;
        b
    }
}

impl NlDeviceIv for TriodeRoot {
    #[inline]
    fn iv(&self, v: crate::Wave) -> (crate::Wave, crate::Wave) {
        (self.plate_current(v), self.plate_current_derivative(v))
    }

    #[inline]
    fn v_clamp(&self) -> (crate::Wave, crate::Wave) {
        (-50.0, self.v_max)
    }
}

// ---------------------------------------------------------------------------
// TriodeThreePort — 3-port (grid + plate) for R-type adaptor MNA fallback
// ---------------------------------------------------------------------------

/// 3-port Koren triode for use with the grouped multi-port NR solver.
///
/// Presents 2 WDF ports to the R-type adaptor:
/// - Port 0: grid-to-cathode (grid conduction diode)
/// - Port 1: plate-to-cathode (Koren plate current, depends on Vgk)
///
/// Unlike `TriodeRoot` where the grid voltage is an external parameter set
/// via `set_vgk()`, here the grid voltage comes from the grid port of the
/// R-type adaptor. This allows the scattering matrix to couple the input
/// signal to the grid, and the NR solver handles the grid-plate interaction
/// through the transconductance cross-derivative ∂Ia/∂Vgk.
///
/// This is necessary for MNA fallback triode stages where the grid-side
/// passive network (bias resistors, threshold pots) is disconnected from
/// the plate-side network in the MNA. Without a grid NL port, the adapted
/// port (input) has `s_nl_adapted = 0` for the plate port, and the triode
/// produces zero output.
///
/// Grid current model: `i_g = I_gs × (exp(Vgk / Vt_g) - 1)`
/// Plate current: Koren equation `Ip = (Vpk/Kp × ln(1 + exp(E1)))^Ex / KG1`
/// Transconductance: `∂Ia/∂Vgk = Ex × base^(Ex-1) / KG1 × Vpk × σ(E1) / √(Kvb + Vpk²)`
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TriodeThreePort {
    pub model: TriodeModel,
    /// Maximum plate voltage (B+ supply rail).
    v_max: crate::Wave,
    /// Voltage offset added to port-1 wave before computing Vpk.
    ///
    /// Two MNA conventions exist:
    /// - **VCC-referenced** (original): MNA ground = VCC, so port-1 wave =
    ///   `V_plate - VCC` (negative). `v_offset = v_max = VCC` recovers actual Vpk.
    ///   Clamp: `(-v_max, 10)`.
    /// - **GND-referenced** (general MNA path): MNA ground = GND, so port-1 wave =
    ///   `V_plate` (positive). Set `v_offset = 0` and clamp `(0, v_max)`.
    v_offset: crate::Wave,
    /// Number of parallel tubes.
    parallel_count: usize,
    /// Grid emission current (saturation current for grid diode).
    grid_is: crate::Wave,
    /// Grid thermal voltage.
    grid_vt: crate::Wave,
}

impl TriodeThreePort {
    pub fn new(model: TriodeModel) -> Self {
        Self {
            model,
            v_max: 500.0,
            v_offset: 500.0,
            parallel_count: 1,
            grid_is: 1e-9,
            grid_vt: 0.025,
        }
    }

    pub fn new_with_v_max(model: TriodeModel, v_max: crate::Wave) -> Self {
        let v_max = v_max.max(1.0);
        Self {
            v_max,
            v_offset: v_max, // VCC-referenced: offset = supply voltage
            ..Self::new(model)
        }
    }

    /// Create a TriodeThreePort for use in a GND-referenced MNA context.
    ///
    /// In the general MNA path, VCC is an explicit voltage source rather than
    /// the MNA reference node. Port-1 waves represent actual plate voltage above
    /// GND (positive, 0..supply_voltage). No offset is needed; the clamp is
    /// adjusted accordingly.
    pub fn new_gnd_referenced(model: TriodeModel, supply_voltage: f64) -> Self {
        let v_max = supply_voltage.max(1.0);
        Self {
            v_max,
            v_offset: 0.0, // GND-referenced: port-1 is already the physical plate voltage
            ..Self::new(model)
        }
    }

    pub fn with_parallel_count(mut self, count: usize) -> Self {
        self.parallel_count = count.max(1);
        self
    }

    pub fn set_v_max(&mut self, v_max: crate::Wave) {
        let old_offset_was_vmax = (self.v_offset - self.v_max).abs() < 1.0;
        self.v_max = v_max.max(1.0);
        // Keep v_offset in sync for VCC-referenced convention
        if old_offset_was_vmax {
            self.v_offset = self.v_max;
        }
    }

    pub fn v_max(&self) -> crate::Wave {
        self.v_max
    }

    pub fn parallel_count(&self) -> usize {
        self.parallel_count
    }

    /// Grid current (diode model): i_g = I_gs × (exp(Vgk/Vt) - 1).
    /// Returns (current, di_g/dv_gk).
    #[inline]
    fn grid_iv(&self, vgk: crate::Wave) -> (crate::Wave, crate::Wave) {
        let x = (vgk / self.grid_vt).clamp(-500.0, 500.0);
        let ev = crate::math::exp(x);
        let ig = self.grid_is * (ev - 1.0) * self.parallel_count as crate::Wave;
        let dig = self.grid_is * ev / self.grid_vt * self.parallel_count as crate::Wave;
        (ig, dig)
    }

    /// Plate current using the Koren equation, with explicit Vgk and Vpk.
    /// Returns (Ia, ∂Ia/∂Vpk, ∂Ia/∂Vgk).
    #[inline]
    fn plate_iv(
        &self,
        vgk: crate::Wave,
        vpk: crate::Wave,
    ) -> (crate::Wave, crate::Wave, crate::Wave) {
        let m = &self.model;
        let pc = self.parallel_count as crate::Wave;

        if vpk <= 0.0 {
            return (0.0, LEAKAGE_CONDUCTANCE * pc, 0.0);
        }

        let vpk_sq = vpk * vpk;
        let sqrt_term = crate::math::sqrt(m.kvb + vpk_sq);

        // E1 = Kp * (1/mu + Vgk / sqrt(Kvb + Vpk^2))
        let e1 = m.kp * (1.0 / m.mu + vgk / sqrt_term);

        let ln_term = softplus(e1);
        if ln_term <= 0.0 {
            return (0.0, LEAKAGE_CONDUCTANCE * pc, 0.0);
        }

        let base = (vpk / m.kp) * ln_term;
        if base <= 0.0 {
            return (0.0, LEAKAGE_CONDUCTANCE * pc, 0.0);
        }

        // Ip = base^Ex / KG1
        let ia = (crate::math::powf(base, m.ex) / m.kg1) * pc;
        if ia <= 0.0 {
            return (0.0, LEAKAGE_CONDUCTANCE * pc, 0.0);
        }

        // Sigmoid(E1) = exp(E1) / (1 + exp(E1)), numerically stable
        let sigmoid_e1 = if e1 > 50.0 {
            1.0
        } else if e1 < -50.0 {
            0.0
        } else {
            let exp_e1 = crate::math::exp(e1);
            exp_e1 / (1.0 + exp_e1)
        };

        // Common factor: Ex * base^(Ex-1) / KG1
        let ex_base_factor = m.ex * crate::math::powf(base, m.ex - 1.0) / m.kg1 * pc;

        // ∂Ia/∂Vpk:
        // dE1/dVpk = -Kp * Vgk * Vpk / (Kvb + Vpk^2)^(3/2)
        let de1_dvpk = -m.kp * vgk * vpk / (sqrt_term * sqrt_term * sqrt_term);
        let dln_dvpk = sigmoid_e1 * de1_dvpk;
        let dbase_dvpk = ln_term / m.kp + (vpk / m.kp) * dln_dvpk;
        let dia_dvpk = (ex_base_factor * dbase_dvpk).max(LEAKAGE_CONDUCTANCE * pc);

        // ∂Ia/∂Vgk (transconductance):
        // dE1/dVgk = Kp / sqrt(Kvb + Vpk^2)
        // dbase/dVgk = (Vpk/Kp) * sigmoid(E1) * Kp / sqrt_term
        //            = Vpk * sigmoid(E1) / sqrt_term
        let dbase_dvgk = vpk * sigmoid_e1 / sqrt_term;
        let dia_dvgk = ex_base_factor * dbase_dvgk;

        (ia, dia_dvpk, dia_dvgk)
    }
}

impl NlDeviceGroupIv for TriodeThreePort {
    fn n_ports(&self) -> usize {
        2
    }

    fn eval(&self, v: &[crate::Wave], currents: &mut [crate::Wave], jacobian: &mut [crate::Wave]) {
        let vgk = v[0]; // Port 0: grid-cathode (actual voltage, no shift needed)

        // Port 1: plate-cathode. The wave v[1] represents the port voltage in
        // whatever MNA convention was used at compile time:
        //
        // - VCC-referenced (v_offset = v_max = VCC): v[1] = V_plate - VCC
        //   (negative). v_offset shifts it back to physical Vpk.
        // - GND-referenced (v_offset = 0): v[1] = V_plate (positive, already physical).
        //
        // The Jacobian is unaffected by the constant shift.
        let vpk = v[1] + self.v_offset;

        // Grid current (diode model)
        let (ig, dig_dvgk) = self.grid_iv(vgk);
        currents[0] = ig;
        jacobian[0] = dig_dvgk; // ∂ig/∂vgk
        jacobian[1] = 0.0; // ∂ig/∂vpk (grid current independent of plate voltage)

        // Plate current (Koren model with cross-coupling)
        let (ip, dip_dvpk, dip_dvgk) = self.plate_iv(vgk, vpk);
        currents[1] = ip;
        jacobian[2] = dip_dvgk; // ∂ip/∂vgk (transconductance — cross-coupling)
        jacobian[3] = dip_dvpk; // ∂ip/∂vpk
    }

    fn v_clamp_port(&self, port: usize) -> (crate::Wave, crate::Wave) {
        match port {
            0 => (-50.0, 10.0), // Grid: well below cutoff to slight forward bias
            _ => {
                if self.v_offset < 1.0 {
                    // GND-referenced: plate voltage is physical (0..v_max)
                    (0.0, self.v_max)
                } else {
                    // VCC-referenced: plate wave is V_plate - VCC, range (-v_max, ~0)
                    (-self.v_max, 10.0)
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(all(test, DISABLED_needs_model_db))]
mod tests {
    use super::*;

    /// SPICE-reference validation: 12AX7 at Vgk=-1.5V, Vpk=200V.
    ///
    /// Hand-computed from Koren's equation with 12AX7 parameters
    /// (MU=100, EX=1.4, KG1=1060, KP=600, KVB=300):
    ///
    /// ```text
    /// E1_arg = 600 * (1/100 + (-1.5)/sqrt(300 + 200²))
    ///        = 600 * (0.01 - 1.5/200.749) = 600 * 0.002528 = 1.517
    /// ln_term = ln(1 + exp(1.517)) = ln(5.559) = 1.715
    /// base = (200/600) * 1.715 = 0.5717
    /// Ip = 0.5717^1.4 / 1060 = 0.457 / 1060 ≈ 4.31e-4 A
    /// ```
    #[test]
    fn triode_12ax7_spice_reference() {
        let mut triode = TriodeRoot::new(TriodeModel::by_name("12AX7"));
        triode.set_vgk(-1.5);
        let ip = triode.plate_current(200.0);

        // Hand-computed: ~4.31e-4 A (0.431 mA)
        let expected = 4.31e-4;
        let error = (ip - expected).abs() / expected;
        assert!(
            error < 0.01,
            "12AX7 Ip should match SPICE reference within 1%: got {ip:.6e}, expected {expected:.6e}, error={error:.4}"
        );
    }

    /// TriodeThreePort plate current must match TriodeRoot plate current.
    #[test]
    fn three_port_matches_single_port() {
        let model = TriodeModel::by_name("12AX7");
        let mut root = TriodeRoot::new(model);
        let tp = TriodeThreePort::new(model);

        for &vgk in &[-3.0, -2.0, -1.0, 0.0] {
            for &vpk in &[50.0, 100.0, 200.0, 300.0] {
                root.set_vgk(vgk);
                let ip_root = root.plate_current(vpk);
                let (ip_tp, _, _) = tp.plate_iv(vgk, vpk);

                let diff = (ip_root - ip_tp).abs();
                assert!(
                    diff < 1e-12,
                    "Plate current mismatch at Vgk={vgk}, Vpk={vpk}: root={ip_root:.6e}, tp={ip_tp:.6e}"
                );
            }
        }
    }

    /// TriodeThreePort derivative w.r.t Vpk must match TriodeRoot derivative
    /// in the conducting region (both above leakage floor).
    #[test]
    fn three_port_derivative_matches_single_port() {
        let model = TriodeModel::by_name("12AX7");
        let mut root = TriodeRoot::new(model);
        let tp = TriodeThreePort::new(model);

        for &vgk in &[-1.5, -1.0, 0.0] {
            for &vpk in &[100.0, 200.0, 300.0] {
                root.set_vgk(vgk);
                let dip_root = root.plate_current_derivative(vpk);
                let (_, dip_tp, _) = tp.plate_iv(vgk, vpk);

                // Skip near-zero regime where leakage floor differs
                if dip_root < 1e-10 && dip_tp < 1e-10 {
                    continue;
                }
                let rel_err = (dip_root - dip_tp).abs() / dip_root.abs().max(1e-15);
                assert!(
                    rel_err < 0.01,
                    "dIp/dVpk mismatch at Vgk={vgk}, Vpk={vpk}: root={dip_root:.6e}, tp={dip_tp:.6e}"
                );
            }
        }
    }

    /// Transconductance (dIp/dVgk) is positive and finite.
    #[test]
    fn three_port_transconductance_positive() {
        let tp = TriodeThreePort::new(TriodeModel::by_name("12AX7"));

        for &vpk in &[100.0, 200.0] {
            for &vgk in &[-2.0, -1.0, 0.0] {
                let (ia, _, gm) = tp.plate_iv(vgk, vpk);
                assert!(
                    gm >= 0.0,
                    "gm should be non-negative at Vgk={vgk}, Vpk={vpk}: got {gm:.6e}"
                );
                assert!(
                    gm.is_finite(),
                    "gm should be finite at Vgk={vgk}, Vpk={vpk}"
                );
                if ia > 1e-10 {
                    assert!(gm > 1e-10, "gm should be significantly positive when conducting: ia={ia:.6e}, gm={gm:.6e}");
                }
            }
        }
    }

    /// TriodeThreePort in grouped solver produces finite, physical results.
    /// With VCC grounding shift: v[1] in WDF is V_plate - V_CC, so the
    /// operating point has negative v[1] (actual plate below V_CC).
    #[test]
    fn three_port_in_grouped_solver() {
        use crate::elements::nonlinear::solver::multi_port_nr_solve_grouped;

        let v_cc = 300.0;
        let tp = TriodeThreePort::new_with_v_max(TriodeModel::by_name("12AX7"), v_cc);
        let groups: [&dyn NlDeviceGroupIv; 1] = [&tp];
        let offsets = [0];

        // Scattering: grid self-coupling small, plate self-coupling moderate negative.
        // s_nl[1][0] = 0: no passive coupling between grid and plate (grounded cathode).
        let s_nl = [-0.1, 0.0, 0.0, -0.5];
        // Grid biased negative; plate has no external excitation (typical for grounded VCC).
        let known_a = [-2.0, 0.0];
        let port_resistances = [500_000.0, 50_000.0];
        // Start at v[1]=0, which with the shift means vpk = V_CC (tube at full supply).
        let mut v_guess = [-2.0, 0.0];

        let b = multi_port_nr_solve_grouped(
            2,
            &s_nl,
            &known_a,
            &port_resistances,
            &groups,
            &offsets,
            &mut v_guess,
            50,
            1e-8,
        );

        assert!(b[0].is_finite(), "Grid reflected wave should be finite");
        assert!(b[1].is_finite(), "Plate reflected wave should be finite");
        // WDF plate voltage should be negative (V_plate < V_CC)
        assert!(
            v_guess[1] < 0.0,
            "Plate WDF voltage should be negative (below V_CC): got {}",
            v_guess[1]
        );
        // Actual plate voltage should be positive (tube conducting, plate above 0)
        let v_plate_actual = v_guess[1] + v_cc;
        assert!(
            v_plate_actual > 0.0,
            "Actual plate voltage should be positive: got {}",
            v_plate_actual
        );
    }
}
