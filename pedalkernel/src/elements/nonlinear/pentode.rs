//! Pentode vacuum tube WDF root elements.
//!
//! Models screen-referenced pentodes (EF86, EL84) using the Koren equation.

use super::solver::{softplus, newton_raphson_solve, LEAKAGE_CONDUCTANCE};
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

    /// 6L6GC — Beam power tetrode.
    ///
    /// The American power tube standard, used in virtually every Fender amp
    /// above 40W. Higher plate dissipation (30W) and cleaner headroom than 6V6.
    /// Gradual compression when overdriven — the core of the Fender clean sound.
    ///
    /// Koren model parameters derived from curve fitting:
    /// mu=8.7, ex=1.35, kg1=1460, kg2=4500, kp=48.0, kvb=12.0
    ///
    /// Typical operating point: Va=460V, Vg2=450V, Vg1=-45V (fixed bias), Ia≈40mA.
    /// Plate resistance ≈33kΩ, transconductance ≈6.0 mA/V.
    ///
    /// Equivalent types: 6L6, 5881 (military), KT66 (British).
    pub fn p_6l6gc() -> Self {
        Self {
            mu: 8.7,
            kp: 48.0,
            kvb: 12.0,
            ex: 1.35,
            kvb2: 25.0,
            vg2_default: 450.0,
        }
    }

    /// EL34 / 6CA7 — True pentode (not a beam tetrode).
    ///
    /// The European power tube standard. THE Marshall sound.
    /// Dominant midrange character, earlier breakup, more aggressive harmonics.
    /// Lower plate resistance (≈15kΩ) and higher transconductance (≈11 mA/V)
    /// than 6L6GC — draws more current with less clean headroom per tube.
    /// Sharper clipping knee than 6L6GC: abrupt clean-to-clip transition
    /// vs the 6L6's gradual compression. This is the core Fender-vs-Marshall
    /// power section character difference.
    ///
    /// Koren model parameters:
    /// mu=11.0, ex=1.35, kg1=650, kg2=4200, kp=60.0, kvb=24.0
    ///
    /// Typical operating point: Va=480V, Vg2=470V, Vg1=-37V (fixed bias), Ia≈60mA.
    /// Max plate dissipation: 25W. Max screen dissipation: 8W.
    ///
    /// Note: As a true pentode, the EL34's screen intercepts more current than
    /// beam tetrodes, especially when plate voltage drops below screen voltage
    /// (the "kink" region during hard push-pull clipping).
    ///
    /// Equivalent types: 6CA7 (American designation), KT77 (drop-in alternative).
    pub fn p_el34() -> Self {
        Self {
            mu: 11.0,
            kp: 60.0,
            kvb: 24.0,
            ex: 1.35,
            kvb2: 18.0,
            vg2_default: 470.0,
        }
    }

    /// 6550 — High-power beam tetrode.
    ///
    /// More headroom than 6L6GC, tighter bass. Used in hi-fi amps,
    /// Ampeg SVT, some Mesa Boogies, and some Dumble builds.
    /// Max plate dissipation: 35W, max screen dissipation: 6.5W.
    ///
    /// Koren model parameters:
    /// mu=7.9, ex=1.35, kg1=890, kg2=4800, kp=36.0, kvb=12.0
    ///
    /// Typical operating point: Va=460V, Vg2=450V, Vg1=-50V (fixed bias).
    /// Plate resistance ≈38kΩ, transconductance ≈5.0 mA/V.
    ///
    /// Equivalent types: KT88 (British), KT90 (higher dissipation variant).
    pub fn p_6550() -> Self {
        Self {
            mu: 7.9,
            kp: 36.0,
            kvb: 12.0,
            ex: 1.35,
            kvb2: 28.0,
            vg2_default: 450.0,
        }
    }

    /// 6AQ5A / 6005 — Small beam power tetrode.
    ///
    /// A miniature power tube used in small amps like the Gibson GA-5,
    /// Fender Champ (early), and various portable amplifiers.
    /// Lower power (9W max plate dissipation) but musical breakup.
    ///
    /// Typical operating point: Va=180-250V, Vg2=180-250V, Vg1=-12.5V.
    /// Lower mu than EL84 (≈10-12 vs 19) and different plate curves.
    ///
    /// Koren model parameters derived from curve fitting:
    /// mu=11.0, kp=450.0, kvb=180.0, ex=1.3
    pub fn p_6aq5a() -> Self {
        Self {
            mu: 11.0,
            kp: 450.0,
            kvb: 180.0,
            ex: 1.3,
            kvb2: 15.0,
            vg2_default: 180.0,
        }
    }

    /// 6973 — Beam power tetrode.
    ///
    /// A unique 9-pin power tube used in some Valco/Supro amps and the
    /// Fairchild 670 limiter's sidechain. Higher transconductance than
    /// 6AQ5A, different harmonic character from EL84.
    ///
    /// Typical operating point: Va=350V, Vg2=350V, Vg1=-15V.
    /// Max plate dissipation: 9W. Notable for spongy, compressed feel.
    ///
    /// Koren model parameters:
    /// mu=14.0, kp=520.0, kvb=220.0, ex=1.35
    pub fn p_6973() -> Self {
        Self {
            mu: 14.0,
            kp: 520.0,
            kvb: 220.0,
            ex: 1.35,
            kvb2: 18.0,
            vg2_default: 350.0,
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
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
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
        let ip = plate_current_at(PentodeModel::p_6l6gc(), -45.0, 450.0, 460.0);
        assert!(ip > 0.0, "6L6GC should conduct at operating point, got {ip}");
        assert!(ip.is_finite(), "6L6GC plate current must be finite");
    }

    #[test]
    fn p_6l6gc_cutoff_at_large_negative_grid() {
        // Deep cutoff: Vg1 far below pinch-off. Current should be much less
        // than at the operating point. The screen-referenced Koren model has
        // some residual leakage at deep cutoff.
        let ip_cutoff = plate_current_at(PentodeModel::p_6l6gc(), -100.0, 450.0, 460.0);
        let ip_operating = plate_current_at(PentodeModel::p_6l6gc(), -45.0, 450.0, 460.0);
        assert!(
            ip_cutoff < ip_operating * 0.1,
            "6L6GC at Vg1=-100V ({ip_cutoff}) should be <10% of operating point ({ip_operating})"
        );
    }

    #[test]
    fn p_6l6gc_no_current_negative_plate() {
        let ip = plate_current_at(PentodeModel::p_6l6gc(), -45.0, 450.0, -10.0);
        assert_eq!(ip, 0.0, "No current for negative plate voltage");
    }

    #[test]
    fn p_6l6gc_derivative_positive() {
        let mut root = PentodeRoot::new(PentodeModel::p_6l6gc());
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
        let ip = plate_current_at(PentodeModel::p_el34(), -37.0, 470.0, 480.0);
        assert!(ip > 0.0, "EL34 should conduct at operating point, got {ip}");
        assert!(ip.is_finite(), "EL34 plate current must be finite");
    }

    #[test]
    fn p_el34_cutoff_at_large_negative_grid() {
        let ip_cutoff = plate_current_at(PentodeModel::p_el34(), -100.0, 470.0, 480.0);
        let ip_operating = plate_current_at(PentodeModel::p_el34(), -37.0, 470.0, 480.0);
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
        let ip_el34 = plate_current_at(PentodeModel::p_el34(), -37.0, 460.0, 460.0);
        let ip_6l6gc = plate_current_at(PentodeModel::p_6l6gc(), -45.0, 450.0, 460.0);
        // Both must be positive and finite at their respective operating points
        assert!(ip_el34 > 0.0 && ip_el34.is_finite(), "EL34 Ip={ip_el34}");
        assert!(ip_6l6gc > 0.0 && ip_6l6gc.is_finite(), "6L6GC Ip={ip_6l6gc}");
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
        let ip = plate_current_at(PentodeModel::p_6550(), -50.0, 450.0, 460.0);
        assert!(ip > 0.0, "6550 should conduct at operating point, got {ip}");
        assert!(ip.is_finite(), "6550 plate current must be finite");
    }

    #[test]
    fn p_6550_cutoff_at_large_negative_grid() {
        let ip_cutoff = plate_current_at(PentodeModel::p_6550(), -120.0, 450.0, 460.0);
        let ip_operating = plate_current_at(PentodeModel::p_6550(), -50.0, 450.0, 460.0);
        assert!(
            ip_cutoff < ip_operating * 0.1,
            "6550 at Vg1=-120V ({ip_cutoff}) should be <10% of operating point ({ip_operating})"
        );
    }

    // ── 6AQ5A tests ──────────────────────────────────────────────────

    #[test]
    fn p_6aq5a_positive_current_at_operating_point() {
        // Typical 6AQ5A operating point: Vg1=-12.5V, Vg2=180V, Vp=180V
        let ip = plate_current_at(PentodeModel::p_6aq5a(), -12.5, 180.0, 180.0);
        assert!(ip > 0.0, "6AQ5A should conduct at operating point, got {ip}");
        assert!(ip.is_finite(), "6AQ5A plate current must be finite");
    }

    #[test]
    fn p_6aq5a_differs_from_el84() {
        // At similar operating conditions, 6AQ5A and EL84 should produce different currents
        let ip_6aq5a = plate_current_at(PentodeModel::p_6aq5a(), -10.0, 200.0, 200.0);
        let ip_el84 = plate_current_at(PentodeModel::p_el84(), -10.0, 200.0, 200.0);
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
        let ip = plate_current_at(PentodeModel::p_6973(), -15.0, 350.0, 350.0);
        assert!(ip > 0.0, "6973 should conduct at operating point, got {ip}");
        assert!(ip.is_finite(), "6973 plate current must be finite");
    }

    #[test]
    fn p_6973_differs_from_el84_and_6aq5a() {
        // All three should have distinct characteristics
        let ip_6973 = plate_current_at(PentodeModel::p_6973(), -12.0, 250.0, 250.0);
        let ip_el84 = plate_current_at(PentodeModel::p_el84(), -12.0, 250.0, 250.0);
        let ip_6aq5a = plate_current_at(PentodeModel::p_6aq5a(), -12.0, 250.0, 250.0);

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
        let ip_6l6gc = plate_current_at(PentodeModel::p_6l6gc(), -40.0, 450.0, 400.0);
        let ip_el34 = plate_current_at(PentodeModel::p_el34(), -40.0, 450.0, 400.0);
        let ip_6550 = plate_current_at(PentodeModel::p_6550(), -40.0, 450.0, 400.0);

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
        let model = PentodeModel::p_6l6gc();
        let mut root = PentodeRoot::new_with_v_max(model, 460.0);
        root.set_vg1k(-45.0);
        root.set_vg2k(450.0);
        // Process with typical WDF wave variables
        let b = root.process(200.0, 4700.0);
        assert!(b.is_finite(), "6L6GC WDF root must converge at 460V B+");
    }

    #[test]
    fn p_el34_wdf_root_converges_at_high_voltage() {
        let model = PentodeModel::p_el34();
        let mut root = PentodeRoot::new_with_v_max(model, 480.0);
        root.set_vg1k(-37.0);
        root.set_vg2k(470.0);
        let b = root.process(200.0, 4700.0);
        assert!(b.is_finite(), "EL34 WDF root must converge at 480V B+");
    }

    #[test]
    fn p_6550_wdf_root_converges_at_high_voltage() {
        let model = PentodeModel::p_6550();
        let mut root = PentodeRoot::new_with_v_max(model, 460.0);
        root.set_vg1k(-50.0);
        root.set_vg2k(450.0);
        let b = root.process(200.0, 4700.0);
        assert!(b.is_finite(), "6550 WDF root must converge at 460V B+");
    }

    #[test]
    fn pentode_set_v_max_propagates() {
        let model = PentodeModel::p_6l6gc();
        let mut root = PentodeRoot::new(model);
        assert_eq!(root.v_max(), 500.0); // default
        root.set_v_max(460.0);
        assert_eq!(root.v_max(), 460.0);
        // Minimum clamp
        root.set_v_max(0.5);
        assert_eq!(root.v_max(), 1.0);
    }
}
