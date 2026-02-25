//! BJT (Bipolar Junction Transistor) WDF root elements.
//!
//! Models NPN and PNP transistors using Ebers-Moll equations.

use super::solver::newton_raphson_solve;
use crate::elements::WdfRoot;

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
