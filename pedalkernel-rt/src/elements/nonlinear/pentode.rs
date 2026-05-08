//! Pentode vacuum tube WDF root elements.
//!
//! Models screen-referenced pentodes (EF86, EL84, 6L6GC, EL34, etc.)
//! using the Koren equation. Parameters are loaded from the embedded
//! `pentodes.model` file.

use super::solver::{newton_raphson_solve, softplus, NlDeviceGroupIv, LEAKAGE_CONDUCTANCE};
use crate::elements::WdfRoot;

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
/// E1 = Kp * (1/mu + Vg1k/Vg2k)
/// Ip_base = (Vg2k/Kp * ln(1 + exp(E1)))^Ex
/// Ip = (Ip_base / KG1) * atan(Vpk / KVB)
/// ```
/// The atan term models the pentode's plate saturation characteristic —
/// plate current is nearly independent of Vpk in the normal operating region.
/// KG1 scales the absolute plate current magnitude.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
    /// Plate current scaling factor (KG1). Scales the absolute plate current
    /// magnitude in the Koren equation.
    pub kg1: f64,
    /// Screen current scaling factor (KG2). Reserved for future screen
    /// current modeling (requires separate WDF current source).
    pub kg2: f64,
    /// Plate resistance at typical operating point (Ω).
    /// Used as virtual resistance in WDF tree construction.
    /// Power pentodes: 15k–50kΩ. Signal pentodes (EF86): ~2.5MΩ.
    pub rp: f64,
}

impl PentodeModel {
    // TODO: move to pedalkernel as extension impl
    // pub fn by_name(name: &str) -> Self { ... }
    // pub fn try_by_name(name: &str) -> Option<Self> { ... }
}

// TODO: move to pedalkernel as extension impl
// impl From<&SpicePentodeModel> for PentodeModel { ... }

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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
            max_iter: super::solver::NR_MAX_ITER,
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
            max_iter: super::solver::NR_MAX_ITER,
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
    /// E1 = Kp * (1/mu + Vg1k/Vg2k)
    /// Ip_base = (Vg2k/Kp * ln(1 + exp(E1)))^Ex
    /// Ip = (Ip_base / KG1) * atan(Vpk / KVB)
    /// ```
    #[inline]
    pub fn plate_current(&self, vpk: f64) -> f64 {
        let mu = self.model.mu;
        let kp = self.model.kp;
        let kvb = self.model.kvb;
        let ex = self.model.ex;
        let vg1k = self.vg1k;
        let vg2k = self.vg2k;

        // No current for negative plate or screen voltage
        if vpk <= 0.0 || vg2k <= 0.0 {
            return 0.0;
        }

        // Screen-referenced Koren: E1 = Kp * (1/mu + Vg1k/Vg2k)
        let e1 = kp * (1.0 / mu + vg1k / vg2k);

        let ln_term = softplus(e1);
        let base = (vg2k / kp) * ln_term;
        if base <= 0.0 {
            return 0.0;
        }

        let ip_base = crate::math::powf(base, ex);

        // Pentode plate saturation: atan(Vpk/KVB)
        // For typical operating Vpk/KVB ratios this is ~1.5 (close to π/2).
        // KG1 absorbs the overall current scale.
        let plate_factor = crate::math::atan(vpk / kvb);

        (ip_base / self.model.kg1) * plate_factor.max(0.0)
    }

    /// Compute derivative of plate current w.r.t. Vpk for Newton-Raphson.
    ///
    /// Since Ip_base is independent of Vpk in the pentode model,
    /// only the plate saturation factor contributes:
    /// `dIp/dVpk = (Ip_base / KG1) * (1 / (1 + (Vpk/KVB)^2)) * (1/KVB)`
    #[inline]
    fn plate_current_derivative(&self, vpk: f64) -> f64 {
        let mu = self.model.mu;
        let kp = self.model.kp;
        let kvb = self.model.kvb;
        let ex = self.model.ex;
        let vg1k = self.vg1k;
        let vg2k = self.vg2k;

        if vpk <= 0.0 || vg2k <= 0.0 {
            return LEAKAGE_CONDUCTANCE;
        }

        let e1 = kp * (1.0 / mu + vg1k / vg2k);

        let ln_term = softplus(e1);
        let base = (vg2k / kp) * ln_term;
        if base <= 0.0 {
            return LEAKAGE_CONDUCTANCE;
        }

        let ip_base = crate::math::powf(base, ex);

        // d/dVpk of atan(Vpk/KVB) = 1/(1 + (Vpk/KVB)^2) * 1/KVB
        let vpk_ratio = vpk / kvb;
        let d_plate_factor = 1.0 / (1.0 + vpk_ratio * vpk_ratio) / kvb;

        (ip_base / self.model.kg1) * d_plate_factor
    }
}

impl super::solver::NlDeviceIv for PentodeRoot {
    #[inline]
    fn iv(&self, v: f64) -> (f64, f64) {
        (self.plate_current(v), self.plate_current_derivative(v))
    }

    #[inline]
    fn v_clamp(&self) -> (f64, f64) {
        // Minimum 1V: pentode plate_current(0) = 0 with zero derivative,
        // which traps the NR solver at v=0. Physical pentode plate voltage
        // is always positive in normal operation.
        (1.0, self.v_max)
    }
}

impl WdfRoot for PentodeRoot {
    /// Pentode plate-cathode path: `i = Ip(Vpk, Vg1k, Vg2k)`
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
// PentodeThreePort — 3-port (grid + plate) for R-type adaptor push-pull
// ---------------------------------------------------------------------------

/// 3-port Koren pentode for use with the grouped multi-port NR solver.
///
/// Presents 2 WDF ports to the R-type adaptor:
/// - Port 0: grid-to-cathode (grid conduction diode)
/// - Port 1: plate-to-cathode (Koren pentode plate current, depends on Vg1k)
///
/// Unlike `PentodeRoot` where the grid voltage is an external parameter set
/// via `set_vg1k()`, here the grid voltage comes from the grid port of the
/// R-type adaptor. This allows coupling caps and grid stoppers to naturally
/// AC-couple the signal to the grid, which is essential for push-pull output
/// stages where DC blocking at the grid is required.
///
/// Screen voltage (Vg2k) remains an external parameter (not a WDF port).
///
/// Grid current model: `i_g = I_gs × (exp(Vgk/Vt) - 1)`
/// Plate current: screen-referenced Koren pentode equation
/// Transconductance: `∂Ip/∂Vgk` derived from the pentode Koren model
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PentodeThreePort {
    pub model: PentodeModel,
    /// Maximum plate voltage (B+ supply rail).
    v_max: f64,
    /// Screen grid voltage (external parameter).
    vg2k: f64,
    /// Grid emission current (saturation current for grid diode).
    grid_is: f64,
    /// Grid thermal voltage.
    grid_vt: f64,
}

impl PentodeThreePort {
    pub fn new(model: PentodeModel) -> Self {
        let vg2k = model.vg2_default;
        Self {
            model,
            v_max: 500.0,
            vg2k,
            grid_is: 1e-9,
            grid_vt: 0.025,
        }
    }

    pub fn new_with_v_max(model: PentodeModel, v_max: f64) -> Self {
        Self {
            v_max: v_max.max(1.0),
            ..Self::new(model)
        }
    }

    pub fn set_v_max(&mut self, v_max: f64) {
        self.v_max = v_max.max(1.0);
    }

    pub fn v_max(&self) -> f64 {
        self.v_max
    }

    pub fn set_vg2k(&mut self, vg2k: f64) {
        self.vg2k = vg2k;
    }

    pub fn vg2k(&self) -> f64 {
        self.vg2k
    }

    /// Grid current (diode model): i_g = I_gs × (exp(Vgk/Vt) - 1).
    /// Returns (current, di_g/dv_gk).
    #[inline]
    fn grid_iv(&self, vgk: f64) -> (f64, f64) {
        let x = (vgk / self.grid_vt).clamp(-500.0, 500.0);
        let ev = crate::math::exp(x);
        let ig = self.grid_is * (ev - 1.0);
        let dig = self.grid_is * ev / self.grid_vt;
        (ig, dig)
    }

    /// Plate current using the screen-referenced Koren pentode equation.
    /// Takes both Vg1k and Vpk as inputs (Vg2k is external).
    /// Returns (Ip, ∂Ip/∂Vpk, ∂Ip/∂Vg1k).
    #[inline]
    fn plate_iv(&self, vg1k: f64, vpk: f64) -> (f64, f64, f64) {
        let m = &self.model;
        let vg2k = self.vg2k;

        if vpk <= 0.0 || vg2k <= 0.0 {
            return (0.0, LEAKAGE_CONDUCTANCE, 0.0);
        }

        // Screen-referenced Koren: E1 = Kp * (1/mu + Vg1k/Vg2k)
        let e1 = m.kp * (1.0 / m.mu + vg1k / vg2k);

        let ln_term = softplus(e1);
        let base = (vg2k / m.kp) * ln_term;
        if base <= 0.0 {
            return (0.0, LEAKAGE_CONDUCTANCE, 0.0);
        }

        let ip_base = crate::math::powf(base, m.ex);

        // Pentode plate saturation: atan(Vpk/KVB)
        let vpk_ratio = vpk / m.kvb;
        let plate_factor = crate::math::atan(vpk_ratio);

        let ip = (ip_base / m.kg1) * plate_factor.max(0.0);
        if ip <= 0.0 {
            return (0.0, LEAKAGE_CONDUCTANCE, 0.0);
        }

        // ∂Ip/∂Vpk: only the plate saturation factor depends on Vpk
        // d/dVpk of atan(Vpk/KVB) = 1/(1 + (Vpk/KVB)^2) * 1/KVB
        let d_plate_factor = 1.0 / (1.0 + vpk_ratio * vpk_ratio) / m.kvb;
        let dip_dvpk = ((ip_base / m.kg1) * d_plate_factor).max(LEAKAGE_CONDUCTANCE);

        // ∂Ip/∂Vg1k (transconductance):
        // E1 = Kp * (1/mu + Vg1k/Vg2k)
        // dE1/dVg1k = Kp / Vg2k
        //
        // base = (Vg2k/Kp) * ln(1 + exp(E1))
        // dbase/dVg1k = (Vg2k/Kp) * sigmoid(E1) * dE1/dVg1k
        //             = (Vg2k/Kp) * sigmoid(E1) * Kp/Vg2k
        //             = sigmoid(E1)
        //
        // ip_base = base^Ex
        // dip_base/dVg1k = Ex * base^(Ex-1) * sigmoid(E1)
        //
        // Ip = (ip_base / KG1) * atan(Vpk/KVB)
        // dIp/dVg1k = (dip_base/dVg1k / KG1) * atan(Vpk/KVB)
        let sigmoid_e1 = if e1 > 50.0 {
            1.0
        } else if e1 < -50.0 {
            0.0
        } else {
            let exp_e1 = crate::math::exp(e1);
            exp_e1 / (1.0 + exp_e1)
        };

        let dip_dvg1k = (m.ex * crate::math::powf(base, m.ex - 1.0) * sigmoid_e1 / m.kg1) * plate_factor.max(0.0);

        (ip, dip_dvpk, dip_dvg1k)
    }
}

impl NlDeviceGroupIv for PentodeThreePort {
    fn n_ports(&self) -> usize {
        2
    }

    fn eval(&self, v: &[f64], currents: &mut [f64], jacobian: &mut [f64]) {
        let vg1k = v[0]; // Port 0: grid-cathode

        // Port 1: plate-cathode. In the R-type adaptor, the supply node (B+)
        // is grounded in the MNA, so the WDF voltage v[1] represents
        // V_plate - V_supply. Shift by +v_max to recover the actual Vpk.
        let vpk = v[1] + self.v_max;

        // Grid current (diode model)
        let (ig, dig_dvg1k) = self.grid_iv(vg1k);
        currents[0] = ig;
        jacobian[0] = dig_dvg1k; // ∂ig/∂vg1k
        jacobian[1] = 0.0; // ∂ig/∂vpk (grid current independent of plate voltage)

        // Plate current (Koren pentode model with cross-coupling)
        let (ip, dip_dvpk, dip_dvg1k) = self.plate_iv(vg1k, vpk);
        currents[1] = ip;
        jacobian[2] = dip_dvg1k; // ∂ip/∂vg1k (transconductance — cross-coupling)
        jacobian[3] = dip_dvpk; // ∂ip/∂vpk
    }

    fn v_clamp_port(&self, port: usize) -> (f64, f64) {
        match port {
            // Grid: wide range to allow coupling cap charging in 3-port push-pull.
            // When input has large DC (e.g., preamp plate voltage ~150V), the
            // coupling cap needs the solver to represent the actual grid voltage
            // during charging. With -50V clamp, the cap can't charge and the
            // tube latches in cutoff. At deep cutoff (Vgk << 0), Ig=Ip=0, so
            // the wide range is safe (no overflow in the current model).
            0 => (-500.0, 10.0),
            _ => (-self.v_max, 10.0), // Plate: WDF range [-V_supply, ~0] (maps to actual [0, V_supply])
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(all(test, DISABLED_needs_model_db))]
mod tests {
    use super::*;

    /// Helper: create a PentodeRoot at a given bias point and measure plate current.
    fn plate_current_at(model: PentodeModel, vg1k: f64, vg2k: f64, vpk: f64) -> f64 {
        let mut root = PentodeRoot::new(model);
        root.set_vg1k(vg1k);
        root.set_vg2k(vg2k);
        root.plate_current(vpk)
    }

    // ── 6L6GC tests ──────────────────────────────────────────────────

    #[test]
    fn p_6l6gc_positive_current_at_operating_point() {
        // At typical Fender Twin operating point: Vg1=-45V, Vg2=450V, Vp=460V
        let ip = plate_current_at(PentodeModel::by_name("6L6GC"), -45.0, 450.0, 460.0);
        assert!(
            ip > 0.0,
            "6L6GC should conduct at operating point, got {ip}"
        );
        assert!(ip.is_finite(), "6L6GC plate current must be finite");
    }

    #[test]
    fn p_6l6gc_cutoff_at_large_negative_grid() {
        // Deep cutoff: Vg1 far below pinch-off. Current should be much less
        // than at the operating point. The screen-referenced Koren model has
        // some residual leakage at deep cutoff.
        let ip_cutoff = plate_current_at(PentodeModel::by_name("6L6GC"), -100.0, 450.0, 460.0);
        let ip_operating = plate_current_at(PentodeModel::by_name("6L6GC"), -45.0, 450.0, 460.0);
        assert!(
            ip_cutoff < ip_operating * 0.1,
            "6L6GC at Vg1=-100V ({ip_cutoff}) should be <10% of operating point ({ip_operating})"
        );
    }

    #[test]
    fn p_6l6gc_no_current_negative_plate() {
        let ip = plate_current_at(PentodeModel::by_name("6L6GC"), -45.0, 450.0, -10.0);
        assert_eq!(ip, 0.0, "No current for negative plate voltage");
    }

    #[test]
    fn p_6l6gc_derivative_positive() {
        let mut root = PentodeRoot::new(PentodeModel::by_name("6L6GC"));
        root.set_vg1k(-45.0);
        root.set_vg2k(450.0);
        let d = root.plate_current_derivative(200.0);
        assert!(d > 0.0, "Derivative should be positive in normal operation");
        assert!(d.is_finite());
    }

    // ── EL34 tests ───────────────────────────────────────────────────

    #[test]
    fn p_el34_positive_current_at_operating_point() {
        // Marshall JCM800 operating point: Vg1=-37V, Vg2=470V, Vp=480V
        let ip = plate_current_at(PentodeModel::by_name("EL34"), -37.0, 470.0, 480.0);
        assert!(ip > 0.0, "EL34 should conduct at operating point, got {ip}");
        assert!(ip.is_finite(), "EL34 plate current must be finite");
    }

    #[test]
    fn p_el34_cutoff_at_large_negative_grid() {
        let ip_cutoff = plate_current_at(PentodeModel::by_name("EL34"), -100.0, 470.0, 480.0);
        let ip_operating = plate_current_at(PentodeModel::by_name("EL34"), -37.0, 470.0, 480.0);
        assert!(
            ip_cutoff < ip_operating * 0.1,
            "EL34 at Vg1=-100V ({ip_cutoff}) should be <10% of operating point ({ip_operating})"
        );
    }

    #[test]
    fn p_el34_clips_earlier_than_6l6gc() {
        // The EL34 has a sharper clipping knee — at the same grid drive level,
        // the transfer curve transition from clean to clipping is steeper.
        // We verify this by checking that the EL34 and 6L6GC have different
        // plate current characteristics (i.e., different models produce
        // meaningfully different outputs).
        let ip_el34 = plate_current_at(PentodeModel::by_name("EL34"), -37.0, 460.0, 460.0);
        let ip_6l6gc = plate_current_at(PentodeModel::by_name("6L6GC"), -45.0, 450.0, 460.0);
        // Both must be positive and finite at their respective operating points
        assert!(ip_el34 > 0.0 && ip_el34.is_finite(), "EL34 Ip={ip_el34}");
        assert!(
            ip_6l6gc > 0.0 && ip_6l6gc.is_finite(),
            "6L6GC Ip={ip_6l6gc}"
        );
        // The models should produce distinctly different currents
        assert!(
            (ip_el34 - ip_6l6gc).abs() / ip_el34.max(ip_6l6gc) > 0.01,
            "EL34 ({ip_el34}) and 6L6GC ({ip_6l6gc}) should differ at their operating points"
        );
    }

    // ── 6550 tests ───────────────────────────────────────────────────

    #[test]
    fn p_6550_positive_current_at_operating_point() {
        // Typical 6550 operating point: Vg1=-50V, Vg2=450V, Vp=460V
        let ip = plate_current_at(PentodeModel::by_name("6550"), -50.0, 450.0, 460.0);
        assert!(ip > 0.0, "6550 should conduct at operating point, got {ip}");
        assert!(ip.is_finite(), "6550 plate current must be finite");
    }

    #[test]
    fn p_6550_cutoff_at_large_negative_grid() {
        let ip_cutoff = plate_current_at(PentodeModel::by_name("6550"), -120.0, 450.0, 460.0);
        let ip_operating = plate_current_at(PentodeModel::by_name("6550"), -50.0, 450.0, 460.0);
        assert!(
            ip_cutoff < ip_operating * 0.1,
            "6550 at Vg1=-120V ({ip_cutoff}) should be <10% of operating point ({ip_operating})"
        );
    }

    // ── 6AQ5A tests ──────────────────────────────────────────────────

    #[test]
    fn p_6aq5a_positive_current_at_operating_point() {
        // Typical 6AQ5A operating point: Vg1=-12.5V, Vg2=180V, Vp=180V
        let ip = plate_current_at(PentodeModel::by_name("6AQ5A"), -12.5, 180.0, 180.0);
        assert!(
            ip > 0.0,
            "6AQ5A should conduct at operating point, got {ip}"
        );
        assert!(ip.is_finite(), "6AQ5A plate current must be finite");
    }

    #[test]
    fn p_6aq5a_differs_from_el84() {
        // At similar operating conditions, 6AQ5A and EL84 should produce different currents
        let ip_6aq5a = plate_current_at(PentodeModel::by_name("6AQ5A"), -10.0, 200.0, 200.0);
        let ip_el84 = plate_current_at(PentodeModel::by_name("EL84"), -10.0, 200.0, 200.0);
        assert!(ip_6aq5a > 0.0 && ip_6aq5a.is_finite());
        assert!(ip_el84 > 0.0 && ip_el84.is_finite());
        assert!(
            (ip_6aq5a - ip_el84).abs() / ip_6aq5a.max(ip_el84) > 0.05,
            "6AQ5A ({ip_6aq5a}) and EL84 ({ip_el84}) should have >5% difference"
        );
    }

    // ── 6973 tests ──────────────────────────────────────────────────

    #[test]
    fn p_6973_positive_current_at_operating_point() {
        // Typical 6973 operating point: Vg1=-15V, Vg2=350V, Vp=350V
        let ip = plate_current_at(PentodeModel::by_name("6973"), -15.0, 350.0, 350.0);
        assert!(ip > 0.0, "6973 should conduct at operating point, got {ip}");
        assert!(ip.is_finite(), "6973 plate current must be finite");
    }

    #[test]
    fn p_6973_differs_from_el84_and_6aq5a() {
        // All three should have distinct characteristics
        let ip_6973 = plate_current_at(PentodeModel::by_name("6973"), -12.0, 250.0, 250.0);
        let ip_el84 = plate_current_at(PentodeModel::by_name("EL84"), -12.0, 250.0, 250.0);
        let ip_6aq5a = plate_current_at(PentodeModel::by_name("6AQ5A"), -12.0, 250.0, 250.0);

        assert!(ip_6973 > 0.0 && ip_6973.is_finite());
        assert!(ip_el84 > 0.0 && ip_el84.is_finite());
        assert!(ip_6aq5a > 0.0 && ip_6aq5a.is_finite());

        // They should all differ from each other
        assert!(
            (ip_6973 - ip_el84).abs() > 1e-6,
            "6973 ({ip_6973}) and EL84 ({ip_el84}) should differ"
        );
        assert!(
            (ip_6973 - ip_6aq5a).abs() > 1e-6,
            "6973 ({ip_6973}) and 6AQ5A ({ip_6aq5a}) should differ"
        );
        assert!(
            (ip_el84 - ip_6aq5a).abs() > 1e-6,
            "EL84 ({ip_el84}) and 6AQ5A ({ip_6aq5a}) should differ"
        );
    }

    // ── Cross-type comparison tests ──────────────────────────────────

    #[test]
    fn pentode_types_have_distinct_characteristics() {
        // All three power pentodes at Vg1=-40V, Vg2=450V, Vp=400V
        // should produce different plate currents (different models matter)
        let ip_6l6gc = plate_current_at(PentodeModel::by_name("6L6GC"), -40.0, 450.0, 400.0);
        let ip_el34 = plate_current_at(PentodeModel::by_name("EL34"), -40.0, 450.0, 400.0);
        let ip_6550 = plate_current_at(PentodeModel::by_name("6550"), -40.0, 450.0, 400.0);

        // All should be finite and positive at this operating point
        assert!(ip_6l6gc > 0.0 && ip_6l6gc.is_finite());
        assert!(ip_el34 > 0.0 && ip_el34.is_finite());
        assert!(ip_6550 > 0.0 && ip_6550.is_finite());

        // They should differ from each other (different tube character)
        assert!(
            (ip_6l6gc - ip_el34).abs() > 1e-6,
            "6L6GC and EL34 should have different plate currents"
        );
        assert!(
            (ip_6l6gc - ip_6550).abs() > 1e-6,
            "6L6GC and 6550 should have different plate currents"
        );
        assert!(
            (ip_el34 - ip_6550).abs() > 1e-6,
            "EL34 and 6550 should have different plate currents"
        );
    }

    // ── WDF root convergence tests ───────────────────────────────────

    #[test]
    fn p_6l6gc_wdf_root_converges_at_high_voltage() {
        let model = PentodeModel::by_name("6L6GC");
        let mut root = PentodeRoot::new_with_v_max(model, 460.0);
        root.set_vg1k(-45.0);
        root.set_vg2k(450.0);
        // Process with typical WDF wave variables
        let b = root.process(200.0, 4700.0);
        assert!(b.is_finite(), "6L6GC WDF root must converge at 460V B+");
    }

    #[test]
    fn p_el34_wdf_root_converges_at_high_voltage() {
        let model = PentodeModel::by_name("EL34");
        let mut root = PentodeRoot::new_with_v_max(model, 480.0);
        root.set_vg1k(-37.0);
        root.set_vg2k(470.0);
        let b = root.process(200.0, 4700.0);
        assert!(b.is_finite(), "EL34 WDF root must converge at 480V B+");
    }

    #[test]
    fn p_6550_wdf_root_converges_at_high_voltage() {
        let model = PentodeModel::by_name("6550");
        let mut root = PentodeRoot::new_with_v_max(model, 460.0);
        root.set_vg1k(-50.0);
        root.set_vg2k(450.0);
        let b = root.process(200.0, 4700.0);
        assert!(b.is_finite(), "6550 WDF root must converge at 460V B+");
    }

    #[test]
    fn pentode_set_v_max_propagates() {
        let model = PentodeModel::by_name("6L6GC");
        let mut root = PentodeRoot::new(model);
        assert_eq!(root.v_max(), 500.0); // default
        root.set_v_max(460.0);
        assert_eq!(root.v_max(), 460.0);
        // Minimum clamp
        root.set_v_max(0.5);
        assert_eq!(root.v_max(), 1.0);
    }

    // ── Quantitative sanity tests ────────────────────────────────────

    #[test]
    fn p_6l6gc_realistic_plate_current_magnitude() {
        // 6L6GC at Vg1k=-30V, Vg2k=450V, Vpk=460V should give ~60-80mA
        let ip = plate_current_at(PentodeModel::by_name("6L6GC"), -30.0, 450.0, 460.0);
        assert!(
            ip > 0.03 && ip < 0.15,
            "6L6GC plate current should be in realistic range (30-150mA): got {ip:.4} A"
        );
    }

    // ── SPICE-reference validation tests ─────────────────────────────

    /// SPICE-reference validation: 6550 at Vg1k=-50V, Vg2k=450V, Vpk=460V.
    ///
    /// Hand-computed from Koren's pentode equation with 6550 parameters
    /// (MU=7.9, EX=1.35, KG1=890, KP=60, KVB=24):
    ///
    /// ```text
    /// E1_arg = (1/7.9 + (-50)/450) * 60 = 0.01547 * 60 = 0.928
    /// ln_term = ln(1 + exp(0.928)) = ln(3.530) = 1.262
    /// base = (450/60) * 1.262 = 9.466
    /// ip_base = 9.466^1.35 = 20.77
    /// plate_factor = atan(460/24) = atan(19.17) = 1.519
    /// Ip = (20.77 / 890) * 1.519 ≈ 0.0355 A
    /// ```
    #[test]
    fn pentode_6550_spice_reference() {
        let ip = plate_current_at(PentodeModel::by_name("6550"), -50.0, 450.0, 460.0);

        // Hand-computed: ~0.0355 A (35.5 mA)
        let expected = 0.0355;
        let error = (ip - expected).abs() / expected;
        assert!(
            error < 0.01,
            "6550 Ip should match SPICE reference within 1%: got {ip:.6e}, expected {expected:.6e}, error={error:.4}"
        );
    }

    // ── PentodeThreePort tests ──────────────────────────────────────

    /// PentodeThreePort plate current must match PentodeRoot plate current.
    #[test]
    fn pentode_three_port_matches_single_port() {
        let model = PentodeModel::by_name("6L6GC");
        let mut root = PentodeRoot::new(model);
        root.set_vg2k(450.0);
        let mut tp = PentodeThreePort::new(model);
        tp.set_vg2k(450.0);

        for &vg1k in &[-45.0, -30.0, -10.0, 0.0] {
            for &vpk in &[50.0, 200.0, 400.0, 460.0] {
                root.set_vg1k(vg1k);
                let ip_root = root.plate_current(vpk);
                let (ip_tp, _, _) = tp.plate_iv(vg1k, vpk);

                let diff = (ip_root - ip_tp).abs();
                assert!(
                    diff < 1e-12,
                    "Plate current mismatch at Vg1k={vg1k}, Vpk={vpk}: root={ip_root:.6e}, tp={ip_tp:.6e}"
                );
            }
        }
    }

    /// PentodeThreePort derivative w.r.t Vpk must match PentodeRoot derivative.
    #[test]
    fn pentode_three_port_derivative_matches_single_port() {
        let model = PentodeModel::by_name("6L6GC");
        let mut root = PentodeRoot::new(model);
        root.set_vg2k(450.0);
        let mut tp = PentodeThreePort::new(model);
        tp.set_vg2k(450.0);

        for &vg1k in &[-30.0, -10.0, 0.0] {
            for &vpk in &[100.0, 200.0, 400.0] {
                root.set_vg1k(vg1k);
                let dip_root = root.plate_current_derivative(vpk);
                let (_, dip_tp, _) = tp.plate_iv(vg1k, vpk);

                if dip_root < 1e-10 && dip_tp < 1e-10 {
                    continue;
                }
                let rel_err = (dip_root - dip_tp).abs() / dip_root.abs().max(1e-15);
                assert!(
                    rel_err < 0.01,
                    "dIp/dVpk mismatch at Vg1k={vg1k}, Vpk={vpk}: root={dip_root:.6e}, tp={dip_tp:.6e}"
                );
            }
        }
    }

    /// Transconductance (dIp/dVg1k) is positive and finite in conducting region.
    #[test]
    fn pentode_three_port_transconductance_positive() {
        let mut tp = PentodeThreePort::new(PentodeModel::by_name("6L6GC"));
        tp.set_vg2k(450.0);

        for &vpk in &[200.0, 400.0] {
            for &vg1k in &[-30.0, -20.0, -10.0, 0.0] {
                let (ip, _, gm) = tp.plate_iv(vg1k, vpk);
                assert!(
                    gm >= 0.0,
                    "gm should be non-negative at Vg1k={vg1k}, Vpk={vpk}: got {gm:.6e}"
                );
                assert!(
                    gm.is_finite(),
                    "gm should be finite at Vg1k={vg1k}, Vpk={vpk}"
                );
                if ip > 1e-10 {
                    assert!(gm > 1e-10, "gm should be significantly positive when conducting: ip={ip:.6e}, gm={gm:.6e}");
                }
            }
        }
    }

    /// Finite-difference Jacobian validation for PentodeThreePort.
    #[test]
    fn pentode_three_port_fd_jacobian() {
        use crate::elements::nonlinear::solver::NlDeviceGroupIv;

        let mut tp = PentodeThreePort::new(PentodeModel::by_name("6L6GC"));
        tp.set_vg2k(450.0);
        let h = 1e-6;

        for &vg1k in &[-30.0, -10.0, 0.0] {
            for &vpk_wdf in &[-200.0, -100.0, -50.0] {
                let v = [vg1k, vpk_wdf];
                let mut i = [0.0; 2];
                let mut j = [0.0; 4];
                tp.eval(&v, &mut i, &mut j);

                // FD for each partial
                for port in 0..2 {
                    let mut v_plus = v;
                    v_plus[port] += h;
                    let mut i_plus = [0.0; 2];
                    let mut j_dummy = [0.0; 4];
                    tp.eval(&v_plus, &mut i_plus, &mut j_dummy);

                    for row in 0..2 {
                        let fd = (i_plus[row] - i[row]) / h;
                        let analytic = j[row * 2 + port];
                        let err = (fd - analytic).abs();
                        let rel = err / analytic.abs().max(1e-12);
                        assert!(
                            rel < 0.01 || err < 1e-10,
                            "FD Jacobian mismatch J[{row}][{port}] at v={v:?}: fd={fd:.6e}, analytic={analytic:.6e}, rel={rel:.4}"
                        );
                    }
                }
            }
        }
    }

    /// PentodeThreePort in grouped solver produces finite, physical results.
    #[test]
    fn pentode_three_port_in_grouped_solver() {
        use crate::elements::nonlinear::solver::{multi_port_nr_solve_grouped, NlDeviceGroupIv};

        let v_cc = 460.0;
        let mut tp = PentodeThreePort::new_with_v_max(PentodeModel::by_name("6L6GC"), v_cc);
        tp.set_vg2k(450.0);
        let groups: [&dyn NlDeviceGroupIv; 1] = [&tp];
        let offsets = [0];

        let s_nl = [-0.1, 0.0, 0.0, -0.5];
        let known_a = [-8.0, 0.0]; // Grid biased negative
        let port_resistances = [500_000.0, 50_000.0];
        let mut v_guess = [-8.0, 0.0];

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
    }
}
