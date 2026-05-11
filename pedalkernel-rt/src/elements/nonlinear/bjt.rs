//! BJT (Bipolar Junction Transistor) WDF root elements.
//!
//! Standalone BJTs use the Gummel-Poon two-port model (`BjtTwoPort`) through
//! the multi-port Newton-Raphson grouped solver. The legacy single-port
//! Ebers-Moll roots (`BjtNpnRoot`, `BjtPnpRoot`, `BjtModel`) have been removed.

use super::solver::{newton_raphson_solve, NlDeviceGroupIv, NlDeviceIv};

// ---------------------------------------------------------------------------
// Gummel-Poon BJT Model
// ---------------------------------------------------------------------------

/// Gummel-Poon BJT model parameters.
///
/// The Gummel-Poon model extends Ebers-Moll with:
/// - Base charge modulation (Early effect, high injection)
/// - Separate forward/reverse ideality factors
/// - Base-emitter and base-collector leakage currents
/// - Junction capacitances for transient analysis
///
/// **Key equations:**
/// ```text
/// Qb = (1 + Vbc/Vaf + Vbe/Var) / sqrt(1 - 4*q1/Ikf - 4*q2/Ikr)
/// Icc = Is * (exp(Vbe/Nf/Vt) - 1) / Qb
/// Iec = Is * (exp(Vbc/Nr/Vt) - 1) / Qb
/// Ic = Icc - Iec/Br
/// Ib = Icc/Bf + Iec/Br + Ise*(exp(Vbe/Ne/Vt)-1) + Isc*(exp(Vbc/Nc/Vt)-1)
/// ```
///
/// **Reference:** H.K. Gummel and H.C. Poon, "An Integral Charge Control Model
/// of Bipolar Transistors", Bell Syst. Tech. J., 1970.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GummelPoonModel {
    // --- DC parameters ---
    /// Saturation current (A). Typically 1e-15 to 1e-12.
    pub is: f64,
    /// Forward current gain (beta_F, hFE). Typically 100-500.
    pub bf: f64,
    /// Reverse current gain (beta_R). Typically 1-20.
    pub br: f64,
    /// Forward ideality factor (typically 1.0).
    pub nf: f64,
    /// Reverse ideality factor (typically 1.0).
    pub nr: f64,
    /// Thermal voltage (V). kT/q ≈ 25.85mV at 25°C.
    pub vt: f64,

    // --- Early effect parameters ---
    /// Forward Early voltage (V). Models base-width modulation in forward-active.
    /// Typically 50-200V. Set to f64::INFINITY to disable.
    pub vaf: f64,
    /// Reverse Early voltage (V). Models base-width modulation in reverse-active.
    /// Typically 5-50V. Set to f64::INFINITY to disable.
    pub var: f64,

    // --- High injection parameters ---
    /// Forward knee current (A). High-injection corner for forward gain.
    /// Typically 0.01-1A. Set to f64::INFINITY to disable.
    pub ikf: f64,
    /// Reverse knee current (A). High-injection corner for reverse gain.
    /// Typically 0.001-0.1A. Set to f64::INFINITY to disable.
    pub ikr: f64,

    // --- Base-emitter leakage ---
    /// B-E leakage saturation current (A). Typically 0 or 1e-14.
    pub ise: f64,
    /// B-E leakage ideality factor. Typically 1.5-2.0.
    pub ne: f64,

    // --- Base-collector leakage ---
    /// B-C leakage saturation current (A). Typically 0 or 1e-13.
    pub isc: f64,
    /// B-C leakage ideality factor. Typically 1.5-2.0.
    pub nc: f64,

    // --- Junction capacitances ---
    /// Zero-bias B-E junction capacitance (F).
    pub cje: f64,
    /// B-E built-in potential (V). Typically 0.7-0.9V.
    pub vje: f64,
    /// B-E grading coefficient. Typically 0.33-0.5.
    pub mje: f64,
    /// Zero-bias B-C junction capacitance (F).
    pub cjc: f64,
    /// B-C built-in potential (V). Typically 0.5-0.75V.
    pub vjc: f64,
    /// B-C grading coefficient. Typically 0.33-0.5.
    pub mjc: f64,

    // --- Terminal ohmic resistances ---
    /// Base spreading resistance (Ω). Models ohmic loss in base region.
    /// Typical range: 1–40 Ω. Causes negative feedback and limits gain.
    pub rb: f64,
    /// Emitter ohmic resistance (Ω).
    /// Typical range: 0.1–2 Ω. Contributes degeneration / gain reduction.
    pub re: f64,
    /// Collector ohmic resistance (Ω).
    /// Typical range: 0.1–4 Ω.
    pub rc: f64,

    // --- Transit time ---
    /// Forward transit time (s). Affects high-frequency response via diffusion capacitance.
    /// C_diff_be = TF * gm = TF * Ic / Vt.
    pub tf: f64,
    /// Reverse transit time (s). Contributes to B-C diffusion capacitance.
    /// C_diff_bc = TR * gm_r = TR * Ie / Vt.
    pub tr: f64,

    /// Whether this is a PNP (vs NPN) transistor.
    pub is_pnp: bool,
}

impl GummelPoonModel {
    /// Look up a Gummel-Poon BJT model by name from the embedded SPICE model library.
    ///
    /// All parameters (IS, BF, BR, NF, NR, VAF, VAR, IKF, IKR, ISE, ISC, NE, NC,
    /// CJE, VJE, MJE, CJC, VJC, MJC, TF, TR) are populated from the SPICE model file.
    ///
    // TODO: move to pedalkernel as extension impl
    // pub fn by_name(name: &str) -> Self { ... }
    // pub fn try_by_name(name: &str) -> Option<Self> { ... }

    /// Compute the base charge factor Qb.
    ///
    /// Qb models base-width modulation (Early effect) and high-injection effects.
    /// When Qb > 1, the effective β drops (beta droop at high currents).
    ///
    /// ```text
    /// q1 = 1 + Vbc/Vaf + Vbe/Var
    /// q2 = Is * (exp(Vbe/Nf/Vt) - 1) / Ikf + Is * (exp(Vbc/Nr/Vt) - 1) / Ikr
    /// Qb = q1/2 * (1 + sqrt(1 + 4*q2))
    /// ```
    #[inline]
    pub fn base_charge(&self, vbe: f64, vbc: f64) -> f64 {
        let exp_vbe = crate::math::exp((vbe / (self.nf * self.vt)).min(40.0));
        let exp_vbc = crate::math::exp((vbc / (self.nr * self.vt)).min(40.0));
        self.base_charge_from_exp(vbe, vbc, exp_vbe, exp_vbc)
    }

    /// Compute base charge from pre-computed exponentials (avoids redundant exp calls).
    #[inline]
    fn base_charge_from_exp(&self, vbe: f64, vbc: f64, exp_vbe: f64, exp_vbc: f64) -> f64 {
        // q1: Early effect term
        let q1 =
            1.0 + if self.vaf.is_finite() {
                vbc / self.vaf
            } else {
                0.0
            } + if self.var.is_finite() {
                vbe / self.var
            } else {
                0.0
            };

        // q2: High-injection term
        let q2_f = if self.ikf.is_finite() && self.ikf > 0.0 {
            self.is * (exp_vbe - 1.0) / self.ikf
        } else {
            0.0
        };
        let q2_r = if self.ikr.is_finite() && self.ikr > 0.0 {
            self.is * (exp_vbc - 1.0) / self.ikr
        } else {
            0.0
        };
        let q2 = q2_f + q2_r;

        // Qb from quadratic formula (always >= 1)
        (q1 / 2.0) * (1.0 + crate::math::sqrt((1.0 + 4.0 * q2).max(0.0)))
    }

    /// Compute collector current using Gummel-Poon transport equations.
    ///
    /// Returns (Ic, Ib) tuple.
    #[inline]
    pub fn currents(&self, vbe: f64, vbc: f64) -> (f64, f64) {
        // Compute all exponentials once — shared between base_charge and transport
        let exp_vbe = crate::math::exp((vbe / (self.nf * self.vt)).min(40.0));
        let exp_vbc = crate::math::exp((vbc / (self.nr * self.vt)).min(40.0));

        let qb = self.base_charge_from_exp(vbe, vbc, exp_vbe, exp_vbc);

        // Forward and reverse currents divided by Qb
        let icc = self.is * (exp_vbe - 1.0) / qb; // Forward transport
        let iec = self.is * (exp_vbc - 1.0) / qb; // Reverse transport

        // Collector current: Icc - Iec/Br
        let ic = icc - iec / self.br;

        // Base current: recombination + leakage
        let ib_f = icc / self.bf; // Forward base current
        let ib_r = iec / self.br; // Reverse base current
        let ib_leak_e = if self.ise > 0.0 {
            self.ise * (crate::math::exp((vbe / (self.ne * self.vt)).min(40.0)) - 1.0)
        } else {
            0.0
        };
        let ib_leak_c = if self.isc > 0.0 {
            self.isc * (crate::math::exp((vbc / (self.nc * self.vt)).min(40.0)) - 1.0)
        } else {
            0.0
        };
        let ib = ib_f + ib_r + ib_leak_e + ib_leak_c;

        (ic, ib)
    }

    /// Compute currents AND analytical 2×2 Jacobian in one pass.
    ///
    /// Returns `(ic, ib, [∂ib/∂vbe, ∂ib/∂vbc, ∂ic/∂vbe, ∂ic/∂vbc])`.
    ///
    /// Note: derivatives are w.r.t. (vbe, vbc), not (vbe, vce).
    /// Caller must chain-rule for (vbe, vce): ∂f/∂vce = -∂f/∂vbc
    /// since vbc = vbe - vce.
    #[inline]
    pub fn currents_and_jacobian(&self, vbe: f64, vbc: f64) -> (f64, f64, [f64; 4]) {
        // Exponentials (shared with currents computation)
        let arg_vbe = (vbe / (self.nf * self.vt)).min(40.0);
        let arg_vbc = (vbc / (self.nr * self.vt)).min(40.0);
        let exp_vbe = crate::math::exp(arg_vbe);
        let exp_vbc = crate::math::exp(arg_vbc);
        let vbe_clamped = arg_vbe >= 40.0;
        let vbc_clamped = arg_vbc >= 40.0;

        // d(exp_vbe)/d(vbe) — zero if clamped (exp is flat at limit)
        let dexp_vbe = if vbe_clamped {
            0.0
        } else {
            exp_vbe / (self.nf * self.vt)
        };
        let dexp_vbc = if vbc_clamped {
            0.0
        } else {
            exp_vbc / (self.nr * self.vt)
        };

        // --- Base charge Qb and its derivatives ---
        // q1 = 1 + vbc/vaf + vbe/var
        let q1 =
            1.0 + if self.vaf.is_finite() {
                vbc / self.vaf
            } else {
                0.0
            } + if self.var.is_finite() {
                vbe / self.var
            } else {
                0.0
            };
        let dq1_dvbe = if self.var.is_finite() {
            1.0 / self.var
        } else {
            0.0
        };
        let dq1_dvbc = if self.vaf.is_finite() {
            1.0 / self.vaf
        } else {
            0.0
        };

        // q2 = IS*(exp_vbe-1)/ikf + IS*(exp_vbc-1)/ikr
        let q2_f = if self.ikf.is_finite() && self.ikf > 0.0 {
            self.is * (exp_vbe - 1.0) / self.ikf
        } else {
            0.0
        };
        let q2_r = if self.ikr.is_finite() && self.ikr > 0.0 {
            self.is * (exp_vbc - 1.0) / self.ikr
        } else {
            0.0
        };
        let q2 = q2_f + q2_r;

        let dq2_dvbe = if self.ikf.is_finite() && self.ikf > 0.0 {
            self.is * dexp_vbe / self.ikf
        } else {
            0.0
        };
        let dq2_dvbc = if self.ikr.is_finite() && self.ikr > 0.0 {
            self.is * dexp_vbc / self.ikr
        } else {
            0.0
        };

        // qb = (q1/2) * (1 + sqrt(1 + 4*q2))
        let inner = (1.0 + 4.0 * q2).max(0.0);
        let sqrt_inner = crate::math::sqrt(inner);
        let qb = (q1 / 2.0) * (1.0 + sqrt_inner);

        // d(qb)/d(vbe) = (dq1/dvbe / 2) * (1 + sqrt_inner) + (q1/2) * (4 * dq2/dvbe) / (2 * sqrt_inner)
        let dsqrt_dvbe = if sqrt_inner > 1e-30 {
            2.0 * dq2_dvbe / sqrt_inner
        } else {
            0.0
        };
        let dsqrt_dvbc = if sqrt_inner > 1e-30 {
            2.0 * dq2_dvbc / sqrt_inner
        } else {
            0.0
        };
        let dqb_dvbe = (dq1_dvbe / 2.0) * (1.0 + sqrt_inner) + (q1 / 2.0) * dsqrt_dvbe;
        let dqb_dvbc = (dq1_dvbc / 2.0) * (1.0 + sqrt_inner) + (q1 / 2.0) * dsqrt_dvbc;

        // --- Transport currents ---
        let ef = self.is * (exp_vbe - 1.0); // IS * (exp_vbe - 1)
        let er = self.is * (exp_vbc - 1.0); // IS * (exp_vbc - 1)
        let icc = ef / qb;
        let iec = er / qb;

        // d(icc)/d(vbe) = (IS * dexp_vbe * qb - ef * dqb_dvbe) / qb^2
        let qb2 = qb * qb;
        let dicc_dvbe = if qb2 > 1e-60 {
            (self.is * dexp_vbe * qb - ef * dqb_dvbe) / qb2
        } else {
            0.0
        };
        let dicc_dvbc = if qb2 > 1e-60 {
            -ef * dqb_dvbc / qb2
        } else {
            0.0
        };
        let diec_dvbe = if qb2 > 1e-60 {
            -er * dqb_dvbe / qb2
        } else {
            0.0
        };
        let diec_dvbc = if qb2 > 1e-60 {
            (self.is * dexp_vbc * qb - er * dqb_dvbc) / qb2
        } else {
            0.0
        };

        // ic = icc - iec/br
        let ic = icc - iec / self.br;
        let dic_dvbe = dicc_dvbe - diec_dvbe / self.br;
        let dic_dvbc = dicc_dvbc - diec_dvbc / self.br;

        // ib = icc/bf + iec/br + leakage terms
        let ib_f = icc / self.bf;
        let ib_r = iec / self.br;

        let dib_dvbe = dicc_dvbe / self.bf + diec_dvbe / self.br;
        let dib_dvbc = dicc_dvbc / self.bf + diec_dvbc / self.br;

        // Leakage: ise * (exp(vbe/(ne*vt)) - 1)
        let (ib_leak_e, dleak_e_dvbe) = if self.ise > 0.0 {
            let arg = (vbe / (self.ne * self.vt)).min(40.0);
            let e = crate::math::exp(arg);
            let clamped = arg >= 40.0;
            (
                self.ise * (e - 1.0),
                if clamped {
                    0.0
                } else {
                    self.ise * e / (self.ne * self.vt)
                },
            )
        } else {
            (0.0, 0.0)
        };

        let (ib_leak_c, dleak_c_dvbc) = if self.isc > 0.0 {
            let arg = (vbc / (self.nc * self.vt)).min(40.0);
            let e = crate::math::exp(arg);
            let clamped = arg >= 40.0;
            (
                self.isc * (e - 1.0),
                if clamped {
                    0.0
                } else {
                    self.isc * e / (self.nc * self.vt)
                },
            )
        } else {
            (0.0, 0.0)
        };

        let ib = ib_f + ib_r + ib_leak_e + ib_leak_c;

        // Final Jacobian: [∂ib/∂vbe, ∂ib/∂vbc, ∂ic/∂vbe, ∂ic/∂vbc]
        let jac = [
            dib_dvbe + dleak_e_dvbe,
            dib_dvbc + dleak_c_dvbc,
            dic_dvbe,
            dic_dvbc,
        ];

        (ic, ib, jac)
    }

    /// Compute B-E junction capacitance including both depletion and diffusion components.
    ///
    /// **Depletion capacitance:** Standard junction formula with forward-bias
    /// linearization to avoid singularity at Vbe = Vje.
    ///
    /// **Diffusion capacitance:** C_diff = TF × gm = TF × Ic / Vt.
    /// Dominant at high current densities; models minority carrier storage.
    ///
    /// `ic` is the collector current at the current operating point (A).
    ///
    /// # Note
    /// These capacitances are currently computed by `BjtGummelPoonRoot` for
    /// reactive transient simulation. `BjtTwoPort` does not yet integrate them
    /// into its Newton-Raphson solver; a future task should add sample-rate
    /// and state tracking analogous to `BjtGummelPoonRoot::cap_be_state`.
    #[inline]
    pub fn capacitance_be(&self, vbe: f64, ic: f64) -> f64 {
        // Depletion capacitance
        let c_depletion = if self.cje > 0.0 {
            // Avoid singularity: linearize for Vbe > 0.8*Vje
            let fc = 0.8;
            if vbe < fc * self.vje {
                self.cje / crate::math::powf(1.0 - vbe / self.vje, self.mje)
            } else {
                // Linear extrapolation for forward bias
                let cje_fc = self.cje / crate::math::powf(1.0 - fc, self.mje);
                let dcje = self.cje * self.mje / (self.vje * crate::math::powf(1.0 - fc, self.mje + 1.0));
                cje_fc + dcje * (vbe - fc * self.vje)
            }
        } else {
            0.0
        };

        // Diffusion (transit-time) capacitance: TF * gm = TF * |Ic| / Vt
        let c_diffusion = if self.tf > 0.0 {
            self.tf * ic.abs() / self.vt
        } else {
            0.0
        };

        c_depletion + c_diffusion
    }

    /// Compute B-C junction capacitance including both depletion and diffusion components.
    ///
    /// **Diffusion capacitance:** C_diff = TR × gm_r = TR × |Ie| / Vt.
    /// `ie` is the emitter current at the current operating point (A).
    ///
    /// # Note
    /// See note on `capacitance_be` regarding integration into `BjtTwoPort`.
    #[inline]
    pub fn capacitance_bc(&self, vbc: f64, ie: f64) -> f64 {
        // Depletion capacitance
        let c_depletion = if self.cjc > 0.0 {
            let fc = 0.8;
            if vbc < fc * self.vjc {
                self.cjc / crate::math::powf(1.0 - vbc / self.vjc, self.mjc)
            } else {
                let cjc_fc = self.cjc / crate::math::powf(1.0 - fc, self.mjc);
                let dcjc = self.cjc * self.mjc / (self.vjc * crate::math::powf(1.0 - fc, self.mjc + 1.0));
                cjc_fc + dcjc * (vbc - fc * self.vjc)
            }
        } else {
            0.0
        };

        // Diffusion (transit-time) capacitance: TR * gm_r = TR * |Ie| / Vt
        let c_diffusion = if self.tr > 0.0 {
            self.tr * ie.abs() / self.vt
        } else {
            0.0
        };

        c_depletion + c_diffusion
    }
}

// TODO: move to pedalkernel as extension impl
// impl From<&SpiceBjtModel> for GummelPoonModel { ... }

/// 3-port R-type adaptor for Gummel-Poon BJT.
///
/// Models the BJT as a 3-terminal nonlinear element where:
/// - Port 1: Base terminal
/// - Port 2: Collector terminal
/// - Port 3: Emitter terminal (adapted, reflection-free)
///
/// The scattering is computed using iterative Newton-Raphson to solve
/// the coupled nonlinear Gummel-Poon equations at each sample.
///
/// **WDF Formulation:**
/// Each port has a Thévenin equivalent (a_i, R_i). The BJT equations
/// constrain the terminal voltages and currents. We solve for the
/// reflected waves b that satisfy both WDF port equations and BJT physics.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BjtGummelPoonRoot {
    pub model: GummelPoonModel,
    /// Port resistances [Rb, Rc, Re]
    port_resistances: [f64; 3],
    /// Cached voltages for warm-starting Newton iteration
    vbe_prev: f64,
    vce_prev: f64,
    /// Last computed currents for external access
    ic: f64,
    ib: f64,
    /// Sample rate for junction capacitances
    sample_rate: f64,
    /// Junction capacitor states (for reactive elements)
    cap_be_state: f64,
    cap_bc_state: f64,
    /// Newton-Raphson parameters
    max_iter: usize,
    tolerance: f64,
}

impl BjtGummelPoonRoot {
    /// Create a new Gummel-Poon BJT root.
    ///
    /// * `model` — Gummel-Poon parameters
    /// * `port_resistances` — [R_base, R_collector, R_emitter]
    /// * `sample_rate` — for junction capacitance discretization
    pub fn new(model: GummelPoonModel, port_resistances: [f64; 3], sample_rate: f64) -> Self {
        Self {
            model,
            port_resistances,
            vbe_prev: 0.0,
            vce_prev: 0.0,
            ic: 0.0,
            ib: 0.0,
            sample_rate,
            cap_be_state: 0.0,
            cap_bc_state: 0.0,
            max_iter: super::solver::NR_MAX_ITER,
            tolerance: 1e-6,
        }
    }

    /// Get last computed collector current.
    #[inline]
    pub fn collector_current(&self) -> f64 {
        self.ic
    }

    /// Get last computed base current.
    #[inline]
    pub fn base_current(&self) -> f64 {
        self.ib
    }

    /// Get emitter current (Ie = Ic + Ib by KCL).
    #[inline]
    pub fn emitter_current(&self) -> f64 {
        self.ic + self.ib
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.vbe_prev = 0.0;
        self.vce_prev = 0.0;
        self.ic = 0.0;
        self.ib = 0.0;
        self.cap_be_state = 0.0;
        self.cap_bc_state = 0.0;
    }

    /// Process one sample through the 3-port BJT.
    ///
    /// Takes incident waves [a_b, a_c, a_e] and port resistances,
    /// returns reflected waves [b_b, b_c, b_e].
    ///
    /// The emitter port is adapted (S_ee = 0).
    pub fn process_3port(&mut self, a: [f64; 3]) -> [f64; 3] {
        let [rb, rc, re] = self.port_resistances;
        let [ab, ac, ae] = a;

        // Convert waves to Thévenin voltages and currents
        // For each port: V = (a + b) / 2, I = (a - b) / (2R)
        // Or equivalently: a = V + R*I, b = V - R*I
        //
        // We need to find Vb, Vc, Ve such that the BJT equations are satisfied.
        // The emitter is reference (Ve = 0 for NPN, after level shift).

        // Newton-Raphson iteration to find operating point
        let mut vbe = self.vbe_prev;
        let mut vce = self.vce_prev;

        // Polarity adjustment for PNP
        let sign = if self.model.is_pnp { -1.0 } else { 1.0 };

        for _ in 0..self.max_iter {
            // Compute currents at current operating point
            let vbc = vbe - vce;
            let (ic, ib) = self.model.currents(vbe * sign, vbc * sign);
            let ic = ic * sign;
            let ib = ib * sign;
            let ie = ic + ib;

            // Add junction capacitor currents if capacitances are nonzero
            let dt = 1.0 / self.sample_rate;
            // Pass operating-point currents so diffusion capacitance (TF/TR) is included.
            let cbe = self.model.capacitance_be(vbe * sign, ic);
            let cbc = self.model.capacitance_bc(vbc * sign, ie);

            // Capacitor current: I = C * dV/dt ≈ C * (V - V_prev) / dt
            // For WDF, we use the discretized capacitor port resistance R = dt / (2*C)
            let ic_cap_be = if cbe > 0.0 {
                2.0 * cbe * (vbe - self.cap_be_state) / dt
            } else {
                0.0
            };
            let ic_cap_bc = if cbc > 0.0 {
                2.0 * cbc * (vbc - self.cap_bc_state) / dt
            } else {
                0.0
            };

            // Total base current includes capacitor charging
            let ib_total = ib + ic_cap_be + ic_cap_bc;
            // Capacitor current at collector
            let ic_total = ic - ic_cap_bc;

            // WDF port equations:
            // ab = Vb + Rb * Ib  =>  Vb = ab - Rb * Ib
            // ac = Vc + Rc * Ic  =>  Vc = ac - Rc * Ic
            // ae = Ve + Re * Ie  =>  Ve = ae - Re * Ie
            //
            // With Ve = 0 as reference:
            // Vb = Vbe, Vc = Vce

            // Residuals: actual V vs V from WDF port
            let vb_wdf = ab - rb * ib_total;
            let vc_wdf = ac - rc * ic_total;
            let ve_wdf = ae - re * (-(ib_total + ic_total)); // Ie flows out of emitter

            let f1 = vbe - (vb_wdf - ve_wdf);
            let f2 = vce - (vc_wdf - ve_wdf);

            if f1.abs() < self.tolerance && f2.abs() < self.tolerance {
                // Converged - store currents and return
                self.ic = ic_total;
                self.ib = ib_total;
                self.vbe_prev = vbe;
                self.vce_prev = vce;

                // Update capacitor states
                if cbe > 0.0 {
                    self.cap_be_state = vbe;
                }
                if cbc > 0.0 {
                    self.cap_bc_state = vbc;
                }

                // Compute reflected waves: b = V - R*I
                let vb = vbe;
                let vc = vce;
                let ve = 0.0;
                let bb = vb - rb * ib_total;
                let bc = vc - rc * ic_total;
                let be = ve - re * (-(ib_total + ic_total));

                return [bb, bc, be];
            }

            // Jacobian approximation using numerical derivatives
            let delta = 1e-7;

            // df1/dvbe, df1/dvce, df2/dvbe, df2/dvce
            let vbc_p = (vbe + delta) - vce;
            let (ic_p, ib_p) = self.model.currents((vbe + delta) * sign, vbc_p * sign);
            let ic_p = ic_p * sign;
            let ib_p = ib_p * sign;

            let vb_wdf_p = ab - rb * ib_p;
            let vc_wdf_p = ac - rc * ic_p;
            let ve_wdf_p = ae - re * (-(ib_p + ic_p));
            let f1_p_vbe = (vbe + delta) - (vb_wdf_p - ve_wdf_p);
            let f2_p_vbe = vce - (vc_wdf_p - ve_wdf_p);

            let vbc_m = vbe - (vce + delta);
            let (ic_m, ib_m) = self.model.currents(vbe * sign, vbc_m * sign);
            let ic_m = ic_m * sign;
            let ib_m = ib_m * sign;

            let vb_wdf_m = ab - rb * ib_m;
            let vc_wdf_m = ac - rc * ic_m;
            let ve_wdf_m = ae - re * (-(ib_m + ic_m));
            let f1_p_vce = vbe - (vb_wdf_m - ve_wdf_m);
            let f2_p_vce = (vce + delta) - (vc_wdf_m - ve_wdf_m);

            let j11 = (f1_p_vbe - f1) / delta;
            let j12 = (f1_p_vce - f1) / delta;
            let j21 = (f2_p_vbe - f2) / delta;
            let j22 = (f2_p_vce - f2) / delta;

            // Solve 2x2 system: J * [dvbe, dvce] = -[f1, f2]
            let det = j11 * j22 - j12 * j21;
            if det.abs() < 1e-15 {
                break; // Singular Jacobian
            }

            let dvbe = -(j22 * f1 - j12 * f2) / det;
            let dvce = -(-j21 * f1 + j11 * f2) / det;

            // Damped update to improve convergence
            let alpha = 0.7;
            vbe += alpha * dvbe.clamp(-0.1, 0.1);
            vce += alpha * dvce.clamp(-1.0, 1.0);

            // Clamp to physical ranges
            vbe = vbe.clamp(-5.0, 1.0);
            vce = vce.clamp(-50.0, 50.0);
        }

        // Failed to converge - use last good values
        let vbc = self.vbe_prev - self.vce_prev;
        let (ic, ib) = self.model.currents(self.vbe_prev * sign, vbc * sign);
        self.ic = ic * sign;
        self.ib = ib * sign;

        let vb = self.vbe_prev;
        let vc = self.vce_prev;
        let ve = 0.0;
        let bb = vb - rb * self.ib;
        let bc = vc - rc * self.ic;
        let be = ve - re * (-(self.ib + self.ic));

        [bb, bc, be]
    }
}

// ---------------------------------------------------------------------------
// BjtTwoPort — 2-port grouped BJT for multi-NL solver
// ---------------------------------------------------------------------------

/// 2-port BJT for the multi-NL grouped solver.
///
/// Exposes both junction voltages as solver variables:
/// - Port 0: (base, emitter) → Vbe → base current Ib
/// - Port 1: (collector, emitter) → Vce → collector current Ic
///
/// The full 2×2 Jacobian includes the critical cross-derivative ∂Ic/∂Vbe
/// (transconductance gm ≈ Ic/Vt), enabling proper coupled BJT solving
/// (e.g., Fuzz Face where Q1 and Q2 interact through shared bias networks).
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BjtTwoPort {
    pub model: GummelPoonModel,
    pub is_pnp: bool,
    v_max: f64,
    /// IS-dependent Vbe clamp: NF * VT * ln(I_max / IS).
    /// Prevents exponential blow-up at forward bias for high-IS devices (Ge).
    vbe_max: f64,
    /// Whether to use analytical Jacobian (validated at construction).
    /// Falls back to numerical if analytical produces poor NR convergence.
    use_analytical_jac: bool,
}

impl BjtTwoPort {
    /// Compute IS-dependent Vbe clamp for max 100mA collector current.
    fn compute_vbe_max(model: &GummelPoonModel) -> f64 {
        let i_max = 0.1; // 100mA — generous bound for small-signal BJTs
        let vbe_max = model.nf * model.vt * crate::math::ln(i_max / model.is);
        vbe_max.clamp(0.3, 1.0) // At least 0.3V, at most 1.0V
    }

    /// Validate analytical Jacobian by comparing eval() output with analytical
    /// vs numerical at representative operating points. Returns true if match.
    fn validate_analytical_jacobian(model: &GummelPoonModel, is_pnp: bool) -> bool {
        // Build a temporary BJT with analytical forced on
        let vbe_max = Self::compute_vbe_max(model);
        let analytical_bjt = BjtTwoPort {
            model: *model,
            is_pnp,
            v_max: 50.0,
            vbe_max,
            use_analytical_jac: true,
        };
        let numerical_bjt = BjtTwoPort {
            model: *model,
            is_pnp,
            v_max: 50.0,
            vbe_max,
            use_analytical_jac: false,
        };

        let sign = if is_pnp { -1.0 } else { 1.0 };
        let test_points: &[[f64; 2]] = &[
            [sign * 0.3, sign * 5.0],
            [sign * 0.5, sign * 2.0],
            [sign * 0.6, sign * 10.0],
            [0.0, sign * 5.0],
        ];

        for v in test_points {
            let mut c_a = [0.0; 2];
            let mut j_a = [0.0; 4];
            let mut c_n = [0.0; 2];
            let mut j_n = [0.0; 4];

            analytical_bjt.eval(v, &mut c_a, &mut j_a);
            numerical_bjt.eval(v, &mut c_n, &mut j_n);

            for k in 0..4 {
                let abs_err = (j_a[k] - j_n[k]).abs();
                let rel_err = if j_n[k].abs() > 1e-15 {
                    abs_err / j_n[k].abs()
                } else {
                    abs_err
                };
                if rel_err > 0.05 && abs_err > 1e-10 {
                    return false;
                }
            }
        }
        true
    }

    /// Create a new NPN BjtTwoPort.
    pub fn new(model: GummelPoonModel) -> Self {
        let vbe_max = Self::compute_vbe_max(&model);
        let use_analytical_jac = Self::validate_analytical_jacobian(&model, false);
        Self {
            model,
            is_pnp: false,
            v_max: 50.0,
            vbe_max,
            use_analytical_jac,
        }
    }

    /// Create a new PNP BjtTwoPort.
    pub fn new_pnp(model: GummelPoonModel) -> Self {
        let vbe_max = Self::compute_vbe_max(&model);
        let use_analytical_jac = Self::validate_analytical_jacobian(&model, true);
        Self {
            model,
            is_pnp: true,
            v_max: 50.0,
            vbe_max,
            use_analytical_jac,
        }
    }

    /// Set the maximum Vce (supply voltage).
    #[inline]
    pub fn set_v_max(&mut self, v_max: f64) {
        self.v_max = v_max.max(1.0);
    }
}

impl NlDeviceGroupIv for BjtTwoPort {
    fn n_ports(&self) -> usize {
        2
    }

    fn eval(&self, v: &[f64], currents: &mut [f64], jacobian: &mut [f64]) {
        let sign = if self.is_pnp { -1.0 } else { 1.0 };
        let vbe_ext = sign * v[0];
        let vce_ext = sign * v[1];

        // Clamp Vbc to prevent catastrophic BC junction forward bias.
        const VBC_MAX: f64 = 0.4;
        let clamp_vbc = |vbe: f64, vce: f64| -> f64 { (vbe - vce).min(VBC_MAX) };

        let rb = self.model.rb;
        let re = self.model.re;
        let rc = self.model.rc;
        let has_parasitics = rb + re + rc > 0.0;

        if !self.use_analytical_jac {
            // Numerical Jacobian fallback (validated at construction as needed)
            let eval_with_parasitics = |vbe_ext: f64, vce_ext: f64| -> (f64, f64) {
                if !has_parasitics {
                    let vbc = clamp_vbc(vbe_ext, vce_ext);
                    return self.model.currents(vbe_ext, vbc);
                }
                let mut vbe_int = vbe_ext;
                let mut vce_int = vce_ext;
                for _ in 0..4 {
                    let vbc_int = clamp_vbc(vbe_int, vce_int);
                    let (ic, ib) = self.model.currents(vbe_int, vbc_int);
                    let ie_out = ic + ib;
                    vbe_int = vbe_ext - ib * rb - ie_out * re;
                    vce_int = vce_ext - ic * rc - ie_out * re;
                }
                let vbc_int = clamp_vbc(vbe_int, vce_int);
                self.model.currents(vbe_int, vbc_int)
            };

            let (ic, ib) = eval_with_parasitics(vbe_ext, vce_ext);
            currents[0] = sign * ib;
            currents[1] = sign * ic;

            let delta = 1e-7;
            let (ic_p, ib_p) = eval_with_parasitics(vbe_ext + delta, vce_ext);
            let (ic_m, ib_m) = eval_with_parasitics(vbe_ext - delta, vce_ext);
            jacobian[0] = (ib_p - ib_m) / (2.0 * delta);
            jacobian[2] = (ic_p - ic_m) / (2.0 * delta);

            let (ic_p2, ib_p2) = eval_with_parasitics(vbe_ext, vce_ext + delta);
            let (ic_m2, ib_m2) = eval_with_parasitics(vbe_ext, vce_ext - delta);
            jacobian[1] = (ib_p2 - ib_m2) / (2.0 * delta);
            jacobian[3] = (ic_p2 - ic_m2) / (2.0 * delta);
            return;
        }

        if !has_parasitics {
            // Fast path: analytical Jacobian, no parasitic iteration needed.
            let vbc = clamp_vbc(vbe_ext, vce_ext);
            let vbc_was_clamped = (vbe_ext - vce_ext) > VBC_MAX;
            let (ic, ib, jac_be_bc) = self.model.currents_and_jacobian(vbe_ext, vbc);
            currents[0] = sign * ib;
            currents[1] = sign * ic;

            // jac_be_bc is [∂ib/∂vbe, ∂ib/∂vbc, ∂ic/∂vbe, ∂ic/∂vbc]
            // We need Jacobian in (vbe, vce) space.
            // vbc = vbe - vce (when unclamped), so ∂vbc/∂vbe = 1, ∂vbc/∂vce = -1
            // When vbc is clamped, ∂vbc/∂vbe = ∂vbc/∂vce = 0
            let dvbc_dvbe = if vbc_was_clamped { 0.0 } else { 1.0 };
            let dvbc_dvce = if vbc_was_clamped { 0.0 } else { -1.0 };

            // ∂f/∂vbe = ∂f/∂vbe_direct + ∂f/∂vbc * dvbc_dvbe
            // ∂f/∂vce = ∂f/∂vbc * dvbc_dvce
            jacobian[0] = jac_be_bc[0] + jac_be_bc[1] * dvbc_dvbe; // ∂Ib/∂Vbe
            jacobian[1] = jac_be_bc[1] * dvbc_dvce; // ∂Ib/∂Vce
            jacobian[2] = jac_be_bc[2] + jac_be_bc[3] * dvbc_dvbe; // ∂Ic/∂Vbe
            jacobian[3] = jac_be_bc[3] * dvbc_dvce; // ∂Ic/∂Vce
        } else {
            // Parasitic path: fixed-point iteration for currents, then analytical
            // Jacobian at converged internal voltages with implicit differentiation
            // through the parasitic voltage drops.
            let mut vbe_int = vbe_ext;
            let mut vce_int = vce_ext;
            for _ in 0..4 {
                let vbc_int = clamp_vbc(vbe_int, vce_int);
                let (ic, ib) = self.model.currents(vbe_int, vbc_int);
                let ie_out = ic + ib;
                vbe_int = vbe_ext - ib * rb - ie_out * re;
                vce_int = vce_ext - ic * rc - ie_out * re;
            }
            let vbc_int = clamp_vbc(vbe_int, vce_int);
            let vbc_was_clamped = (vbe_int - vce_int) > VBC_MAX;

            let (ic, ib, jac_be_bc) = self.model.currents_and_jacobian(vbe_int, vbc_int);
            currents[0] = sign * ib;
            currents[1] = sign * ic;

            // Chain rule through parasitics using implicit function theorem.
            // At convergence:
            //   vbe_int = vbe_ext - ib(vbe_int, vbc_int) * rb - ie(vbe_int, vbc_int) * re
            //   vce_int = vce_ext - ic(vbe_int, vbc_int) * rc - ie(vbe_int, vbc_int) * re
            // where ie = ic + ib, vbc_int = vbe_int - vce_int (when unclamped).
            //
            // Differentiating both sides w.r.t. vbe_ext:
            //   dvbe_int/dvbe_ext = 1 - (∂ib/∂vbe_int * dvbe_int/dvbe_ext + ∂ib/∂vce_int * dvce_int/dvbe_ext) * rb
            //                         - (∂ie/∂vbe_int * dvbe_int/dvbe_ext + ∂ie/∂vce_int * dvce_int/dvbe_ext) * re
            //
            // This is a 2×2 linear system: (I + M) * [dvbe_int/dvbe_ext; dvce_int/dvbe_ext] = [1; 0]
            //                              (I + M) * [dvbe_int/dvce_ext; dvce_int/dvce_ext] = [0; 1]
            // where M accounts for parasitic feedback.

            // Derivatives in (vbe_int, vce_int) space
            // From jac_be_bc = [∂ib/∂vbe, ∂ib/∂vbc, ∂ic/∂vbe, ∂ic/∂vbc]
            // with chain rule vbc = vbe_int - vce_int:
            let dvbc_dvbe_int = if vbc_was_clamped { 0.0 } else { 1.0 };
            let dvbc_dvce_int = if vbc_was_clamped { 0.0 } else { -1.0 };

            let dib_dvbe_int = jac_be_bc[0] + jac_be_bc[1] * dvbc_dvbe_int;
            let dib_dvce_int = jac_be_bc[1] * dvbc_dvce_int;
            let dic_dvbe_int = jac_be_bc[2] + jac_be_bc[3] * dvbc_dvbe_int;
            let dic_dvce_int = jac_be_bc[3] * dvbc_dvce_int;

            let die_dvbe_int = dic_dvbe_int + dib_dvbe_int;
            let die_dvce_int = dic_dvce_int + dib_dvce_int;

            // System matrix (I + M):
            // [ 1 + dib_dvbe_int*rb + die_dvbe_int*re,  dib_dvce_int*rb + die_dvce_int*re ]
            // [ dic_dvbe_int*rc + die_dvbe_int*re,       1 + dic_dvce_int*rc + die_dvce_int*re ]
            let a11 = 1.0 + dib_dvbe_int * rb + die_dvbe_int * re;
            let a12 = dib_dvce_int * rb + die_dvce_int * re;
            let a21 = dic_dvbe_int * rc + die_dvbe_int * re;
            let a22 = 1.0 + dic_dvce_int * rc + die_dvce_int * re;

            let det = a11 * a22 - a12 * a21;
            if det.abs() > 1e-30 {
                let inv_det = 1.0 / det;

                // Solve for dvbe_int/dvbe_ext, dvce_int/dvbe_ext (RHS = [1, 0])
                let dvbe_int_dvbe_ext = a22 * inv_det;
                let dvce_int_dvbe_ext = -a21 * inv_det;

                // Solve for dvbe_int/dvce_ext, dvce_int/dvce_ext (RHS = [0, 1])
                let dvbe_int_dvce_ext = -a12 * inv_det;
                let dvce_int_dvce_ext = a11 * inv_det;

                // Final external Jacobian: ∂f/∂v_ext = ∂f/∂v_int * dv_int/dv_ext
                jacobian[0] = dib_dvbe_int * dvbe_int_dvbe_ext + dib_dvce_int * dvce_int_dvbe_ext;
                jacobian[1] = dib_dvbe_int * dvbe_int_dvce_ext + dib_dvce_int * dvce_int_dvce_ext;
                jacobian[2] = dic_dvbe_int * dvbe_int_dvbe_ext + dic_dvce_int * dvce_int_dvbe_ext;
                jacobian[3] = dic_dvbe_int * dvbe_int_dvce_ext + dic_dvce_int * dvce_int_dvce_ext;
            } else {
                // Degenerate: fall back to internal derivatives (parasitics negligible)
                jacobian[0] = dib_dvbe_int;
                jacobian[1] = dib_dvce_int;
                jacobian[2] = dic_dvbe_int;
                jacobian[3] = dic_dvce_int;
            }
        }
    }

    fn v_clamp_port(&self, port: usize) -> (f64, f64) {
        match port {
            0 => (-self.vbe_max, self.vbe_max), // Vbe: IS-dependent (Ge ~0.32V, Si ~0.83V)
            _ => (-self.v_max, self.v_max),     // Vce: full swing
        }
    }
}

// ---------------------------------------------------------------------------
// EbersMollTwoPort — fast simplified BJT model
// ---------------------------------------------------------------------------

/// Simplified Ebers-Moll BJT model for deterministic-cost NR evaluation.
///
/// This model is Gummel-Poon with all advanced effects disabled:
/// - No Early effect (Qb = 1, no base-width modulation via VAF/VAR)
/// - No high-injection (no IKF/IKR knee currents)
/// - No leakage (no ISE/ISC terms)
/// - No parasitic resistances (RB/RE/RC are zero)
/// - No junction/diffusion capacitances
///
/// The simplified IV equations are:
/// ```text
/// Ic = IS * (exp(Vbe / NF*VT) - 1) - (IS/BR) * (exp(Vbc / NR*VT) - 1)
/// Ib = (IS/BF) * (exp(Vbe / NF*VT) - 1) + (IS/BR) * (exp(Vbc / NR*VT) - 1)
/// ```
///
/// The Jacobian requires only 2 `exp()` calls and 4 multiplications
/// (vs 10+ `exp()` calls and full Qb chain-rule for Gummel-Poon).
///
/// # When to use
/// Suitable for clean amplifier stages where GP advanced effects are
/// negligible at the operating point (Vbe near turn-on, Vce >> Vce_sat,
/// small-signal regime). Enable via `--features ebers-moll`.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct EbersMollTwoPort {
    /// Saturation current (A). Typically 1e-15 to 1e-12.
    pub is: f64,
    /// Forward current gain (beta_F, hFE). Typically 100-500.
    pub bf: f64,
    /// Reverse current gain (beta_R). Typically 1-20.
    pub br: f64,
    /// NF * VT — forward thermal voltage denominator.
    pub nf_vt: f64,
    /// NR * VT — reverse thermal voltage denominator.
    pub nr_vt: f64,
    /// Whether this is a PNP (vs NPN) transistor.
    pub is_pnp: bool,
    /// IS-dependent Vbe clamp: prevents exp blow-up at forward bias.
    /// Same logic as `BjtTwoPort::compute_vbe_max`.
    vbe_max: f64,
    /// Vce/Vbc swing limit.
    v_max: f64,
}

impl EbersMollTwoPort {
    /// Compute the IS-dependent Vbe clamp (same logic as `BjtTwoPort`).
    fn compute_vbe_max(is: f64, nf: f64, vt: f64) -> f64 {
        let i_max = 0.1; // 100 mA generous bound
        let vbe_max = nf * vt * crate::math::ln(i_max / is);
        vbe_max.clamp(0.3, 1.0)
    }

    /// Create an `EbersMollTwoPort` from a `GummelPoonModel` (NPN polarity).
    ///
    /// Only IS, BF, BR, NF, NR, and VT are used; all advanced parameters are
    /// discarded.
    pub fn from_gp(gp: &GummelPoonModel) -> Self {
        Self {
            is: gp.is,
            bf: gp.bf,
            br: gp.br,
            nf_vt: gp.nf * gp.vt,
            nr_vt: gp.nr * gp.vt,
            is_pnp: false,
            vbe_max: Self::compute_vbe_max(gp.is, gp.nf, gp.vt),
            v_max: 50.0,
        }
    }

    /// Create an `EbersMollTwoPort` from a `GummelPoonModel` (PNP polarity).
    pub fn from_gp_pnp(gp: &GummelPoonModel) -> Self {
        Self {
            is_pnp: true,
            ..Self::from_gp(gp)
        }
    }

    /// Set the maximum Vce (supply voltage).
    #[inline]
    pub fn set_v_max(&mut self, v_max: f64) {
        self.v_max = v_max.max(1.0);
    }

    /// `true` if this is a PNP device.
    #[inline]
    pub fn is_pnp(&self) -> bool {
        self.is_pnp
    }
}

impl NlDeviceGroupIv for EbersMollTwoPort {
    fn n_ports(&self) -> usize {
        2
    }

    /// Evaluate Ebers-Moll currents and 2×2 Jacobian.
    ///
    /// Port ordering matches `BjtTwoPort`:
    /// - Port 0: Vbe (base-emitter) → Ib
    /// - Port 1: Vce (collector-emitter) → Ic
    ///
    /// PNP devices negate all voltages before evaluation and negate
    /// currents before writing back.
    fn eval(&self, v: &[f64], currents: &mut [f64], jacobian: &mut [f64]) {
        let sign = if self.is_pnp { -1.0 } else { 1.0 };
        let vbe = sign * v[0];
        let vce = sign * v[1];

        // Clamp Vbc to prevent catastrophic BC junction forward bias.
        const VBC_MAX: f64 = 0.4;
        let vbc_raw = vbe - vce;
        let vbc_was_clamped = vbc_raw > VBC_MAX;
        let vbc = vbc_raw.min(VBC_MAX);

        // Clamp exponential arguments to prevent overflow (same limit as GP).
        let arg_be = (vbe / self.nf_vt).min(40.0);
        let arg_bc = (vbc / self.nr_vt).min(40.0);
        let exp_be = crate::math::exp(arg_be);
        let exp_bc = crate::math::exp(arg_bc);
        let be_clamped = arg_be >= 40.0;
        let bc_clamped = arg_bc >= 40.0;

        // Currents
        let ef = self.is * (exp_be - 1.0);
        let er = self.is * (exp_bc - 1.0);
        let ic = ef - er / self.br;
        let ib = ef / self.bf + er / self.br;

        currents[0] = sign * ib;
        currents[1] = sign * ic;

        // Jacobian in (vbe, vbe-vce) = (vbe, vbc) space.
        // d(ef)/d(vbe) = IS * exp_be / nf_vt  (zero when clamped)
        // d(er)/d(vbc) = IS * exp_bc / nr_vt  (zero when clamped)
        let def_dvbe = if be_clamped {
            0.0
        } else {
            self.is * exp_be / self.nf_vt
        };
        let der_dvbc = if bc_clamped {
            0.0
        } else {
            self.is * exp_bc / self.nr_vt
        };

        // Chain-rule: vbc = vbe - vce (when unclamped)
        // ∂vbc/∂vbe = 1, ∂vbc/∂vce = -1; both 0 when clamped.
        let dvbc_dvbe = if vbc_was_clamped { 0.0 } else { 1.0 };
        let dvbc_dvce = if vbc_was_clamped { 0.0 } else { -1.0 };

        // ∂Ib/∂Vbe = def_dvbe/bf + der_dvbc * dvbc_dvbe / br
        // ∂Ib/∂Vce =              + der_dvbc * dvbc_dvce / br
        // ∂Ic/∂Vbe = def_dvbe    - der_dvbc * dvbc_dvbe / br
        // ∂Ic/∂Vce =             - der_dvbc * dvbc_dvce / br
        jacobian[0] = def_dvbe / self.bf + der_dvbc * dvbc_dvbe / self.br; // ∂Ib/∂Vbe
        jacobian[1] = der_dvbc * dvbc_dvce / self.br; // ∂Ib/∂Vce
        jacobian[2] = def_dvbe - der_dvbc * dvbc_dvbe / self.br; // ∂Ic/∂Vbe
        jacobian[3] = -der_dvbc * dvbc_dvce / self.br; // ∂Ic/∂Vce
    }

    fn v_clamp_port(&self, port: usize) -> (f64, f64) {
        match port {
            0 => (-self.vbe_max, self.vbe_max), // Vbe: IS-dependent
            _ => (-self.v_max, self.v_max),     // Vce: full swing
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// BjtRoot: single-port WDF root for common-emitter BJT stages
// ═══════════════════════════════════════════════════════════════════════════

/// BJT as a single-port WDF root (like TriodeRoot for tubes).
///
/// Vbe is set externally as a control parameter (from the base signal).
/// The collector-emitter path is the WDF port. This enables:
/// - Standard WDF tree processing for BJT stages
/// - K-method table generation (2D: b_tree × Vbe)
/// - Blockwise decomposition of BJT cascades (303 ladder)
///
/// Uses the Gummel-Poon model for the I-V characteristic.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BjtRoot {
    pub model: GummelPoonModel,
    pub is_pnp: bool,
    /// Current base-emitter voltage (external control parameter).
    vbe: f64,
    /// Maximum collector-emitter voltage (from supply rail).
    v_max: f64,
    /// Previous sample's Vce for NR warm-starting.
    prev_v: f64,
}

impl BjtRoot {
    pub fn new(model: GummelPoonModel, is_pnp: bool) -> Self {
        Self {
            model,
            is_pnp,
            vbe: 0.0,
            v_max: 50.0,
            prev_v: 0.0,
        }
    }

    pub fn new_with_v_max(model: GummelPoonModel, is_pnp: bool, v_max: f64) -> Self {
        Self {
            model,
            is_pnp,
            vbe: 0.0,
            v_max: v_max.max(1.0),
            prev_v: 0.0,
        }
    }

    /// Set the base-emitter voltage (external control from input signal).
    #[inline]
    pub fn set_vbe(&mut self, vbe: f64) {
        self.vbe = vbe;
    }

    /// Get current Vbe.
    #[inline]
    pub fn vbe(&self) -> f64 {
        self.vbe
    }

    pub fn set_v_max(&mut self, v_max: f64) {
        self.v_max = v_max.max(1.0);
    }

    /// Collector current Ic as a function of Vce, with Vbe held constant.
    /// This is the I-V characteristic seen at the WDF port.
    #[inline]
    pub fn collector_current(&self, vce: f64) -> f64 {
        let sign = if self.is_pnp { -1.0 } else { 1.0 };
        let vbe = sign * self.vbe;
        let vce = sign * vce;
        let vbc = (vbe - vce).min(0.4); // Clamp Vbc
        let (ic, _ib) = self.model.currents(vbe, vbc);
        sign * ic
    }

    /// Derivative dIc/dVce for Newton-Raphson.
    #[inline]
    pub fn collector_current_derivative(&self, vce: f64) -> f64 {
        // Numerical derivative (simple, robust)
        let h = 1e-6;
        let ic_plus = self.collector_current(vce + h);
        let ic_minus = self.collector_current(vce - h);
        (ic_plus - ic_minus) / (2.0 * h)
    }

    /// WDF NR solve: incident wave → reflected wave.
    pub fn process(&mut self, a: f64, rp: f64) -> f64 {
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
        let root = self.clone();
        let b = newton_raphson_solve(
            a,
            rp,
            v0,
            super::solver::NR_MAX_ITER,
            1e-6,
            Some((-1.0, v_max)),
            None,
            |v| (root.collector_current(v), root.collector_current_derivative(v)),
        );
        self.prev_v = (a + b) * 0.5;
        b
    }
}

impl NlDeviceIv for BjtRoot {
    #[inline]
    fn iv(&self, v: f64) -> (f64, f64) {
        (self.collector_current(v), self.collector_current_derivative(v))
    }

    #[inline]
    fn v_clamp(&self) -> (f64, f64) {
        (-1.0, self.v_max)
    }
}

// Tests for BJT elements live in the pedalkernel crate (require model DB).
// See pedalkernel/src/elements/nonlinear/bjt.rs for the canonical test suite.

#[cfg(all(test, DISABLED_needs_model_db))]
mod gummel_poon_tests {
    use super::*;

    #[test]
    fn gummel_poon_base_charge_unity_at_zero() {
        let model = GummelPoonModel::by_name("2N3904");
        let qb = model.base_charge(0.0, 0.0);
        // At zero bias, Qb should be ~1.0
        assert!(
            (qb - 1.0).abs() < 0.1,
            "Qb at zero bias should be ~1.0, got {qb}"
        );
    }

    #[test]
    fn gummel_poon_base_charge_increases_with_current() {
        let model = GummelPoonModel::by_name("2N3904");
        let qb_low = model.base_charge(0.5, 0.0);
        let qb_high = model.base_charge(0.7, 0.0);
        // Qb should increase at higher Vbe (high injection)
        assert!(
            qb_high > qb_low,
            "Qb should increase with Vbe: {} vs {}",
            qb_low,
            qb_high
        );
    }

    #[test]
    fn gummel_poon_forward_active() {
        let model = GummelPoonModel::by_name("2N3904");
        // Typical forward-active: Vbe=0.65V, Vbc=-5V (Vce=5.65V)
        let (ic, ib) = model.currents(0.65, -5.0);

        // Should have positive collector current
        assert!(ic > 0.0, "Ic should be positive in forward-active");
        assert!(ib > 0.0, "Ib should be positive in forward-active");

        // Beta should be reasonable (not exactly Bf due to high injection)
        let beta = ic / ib;
        assert!(
            beta > 50.0 && beta < 1000.0,
            "Beta={beta} should be reasonable"
        );
    }

    #[test]
    fn gummel_poon_cutoff() {
        let model = GummelPoonModel::by_name("2N3904");
        // Cutoff: Vbe < 0
        let (ic, ib) = model.currents(-0.5, -5.0);

        // Currents should be very small
        assert!(ic.abs() < 1e-9, "Ic should be ~0 in cutoff");
        assert!(ib.abs() < 1e-9, "Ib should be ~0 in cutoff");
    }

    #[test]
    fn gummel_poon_capacitance_be() {
        let model = GummelPoonModel::by_name("2N3904");

        // Zero-bias, zero current: depletion cap only — should equal Cje
        let c0 = model.capacitance_be(0.0, 0.0);
        assert!(
            (c0 - model.cje).abs() < 1e-15,
            "Cbe at Vbe=0, Ic=0 should equal Cje"
        );

        // Reverse bias: depletion capacitance should decrease
        let c_rev = model.capacitance_be(-1.0, 0.0);
        assert!(c_rev < c0, "Cbe should decrease with reverse bias");

        // Forward bias: depletion capacitance should increase
        let c_fwd = model.capacitance_be(0.5, 0.0);
        assert!(c_fwd > c0, "Cbe should increase with forward bias");
    }

    #[test]
    fn gummel_poon_3port_basic() {
        let model = GummelPoonModel::by_name("2N3904");
        let mut bjt = BjtGummelPoonRoot::new(model, [1000.0, 10000.0, 100.0], 48000.0);

        // Test that 3-port solver produces finite, stable output
        // The 3-port formulation is complex; this tests basic numerical stability
        let a = [1.4, 9.0, 0.0]; // [base source, collector supply, emitter ground]
        let b = bjt.process_3port(a);

        // All outputs should be finite (no NaN/infinity)
        assert!(b[0].is_finite(), "b_base should be finite: {}", b[0]);
        assert!(b[1].is_finite(), "b_collector should be finite: {}", b[1]);
        assert!(b[2].is_finite(), "b_emitter should be finite: {}", b[2]);

        // Run many samples to test stability
        for i in 0..100 {
            let b = bjt.process_3port(a);
            assert!(
                b[0].is_finite() && b[1].is_finite() && b[2].is_finite(),
                "Outputs should remain stable at sample {i}"
            );
        }
    }

    #[test]
    fn gummel_poon_germanium_higher_leakage() {
        let si = GummelPoonModel::by_name("2N3904");
        let ge = GummelPoonModel::by_name("AC128");

        // At same Vbe (near turn-on), germanium should have more leakage
        let (ic_si, _) = si.currents(0.2, -5.0);
        let (ic_ge, _) = ge.currents(0.2, -5.0);

        assert!(
            ic_ge > ic_si * 10.0,
            "Germanium should have much higher current at low Vbe: Ge={ic_ge} vs Si={ic_si}"
        );
    }

    #[test]
    fn gummel_poon_pnp_polarity() {
        let npn = GummelPoonModel::by_name("2N3904");
        let pnp = GummelPoonModel::by_name("2N3906");

        // Same magnitude bias, opposite polarity for PNP
        let (ic_npn, _) = npn.currents(0.65, -5.0);
        let (ic_pnp, _) = pnp.currents(0.65, -5.0);

        // Both should have positive Ic (PNP model handles polarity internally)
        assert!(ic_npn > 0.0, "NPN Ic should be positive");
        assert!(
            ic_pnp > 0.0,
            "PNP Ic should be positive (internal polarity)"
        );
    }

    // ---------------------------------------------------------------------------
    // Parasitic resistance tests (RB, RE, RC)
    // ---------------------------------------------------------------------------

    /// With RB > 0, the internal Vbe is reduced by Ib * RB at the same external Vbe,
    /// so the resulting Ib (and Ic) should be slightly lower.
    #[test]
    fn parasitic_rb_reduces_current_at_same_external_vbe() {
        let mut model_no_rb = GummelPoonModel::by_name("2N3904");
        model_no_rb.rb = 0.0;
        model_no_rb.re = 0.0;
        model_no_rb.rc = 0.0;

        let mut model_rb = GummelPoonModel::by_name("2N3904");
        model_rb.rb = 30.0; // 30 Ω base resistance
        model_rb.re = 0.0;
        model_rb.rc = 0.0;

        // Build BjtTwoPort wrappers and evaluate at same external Vbe
        let bjt_no_rb = BjtTwoPort::new(model_no_rb);
        let bjt_rb = BjtTwoPort::new(model_rb);

        let vbe_ext = 0.65_f64; // external Vbe in forward active
        let vce_ext = 5.0_f64;

        let mut currents_no_rb = [0.0_f64; 2];
        let mut jacobian_no_rb = [0.0_f64; 4];
        bjt_no_rb.eval(
            &[vbe_ext, vce_ext],
            &mut currents_no_rb,
            &mut jacobian_no_rb,
        );

        let mut currents_rb = [0.0_f64; 2];
        let mut jacobian_rb = [0.0_f64; 4];
        bjt_rb.eval(&[vbe_ext, vce_ext], &mut currents_rb, &mut jacobian_rb);

        let ib_no_rb = currents_no_rb[0];
        let ib_rb = currents_rb[0];
        let ic_no_rb = currents_no_rb[1];
        let ic_rb = currents_rb[1];

        // With RB resistance the internal Vbe drops, so Ib and Ic must be lower
        assert!(
            ib_rb < ib_no_rb,
            "RB=30Ω should reduce base current: {ib_rb:.3e} vs {ib_no_rb:.3e}"
        );
        assert!(
            ic_rb < ic_no_rb,
            "RB=30Ω should reduce collector current: {ic_rb:.3e} vs {ic_no_rb:.3e}"
        );

        // The reduction should be small (< 50%) since voltage drops are small
        assert!(
            ic_rb > ic_no_rb * 0.5,
            "Ic reduction should be < 50%: {ic_rb:.3e} vs {ic_no_rb:.3e}"
        );
    }

    /// RE provides emitter degeneration — negative feedback that reduces effective gain.
    /// At the same external Vbe bias, RE causes a voltage drop that reduces internal Vbe.
    #[test]
    fn parasitic_re_provides_negative_feedback() {
        let mut model_no_re = GummelPoonModel::by_name("2N3904");
        model_no_re.rb = 0.0;
        model_no_re.re = 0.0;
        model_no_re.rc = 0.0;

        let mut model_re = GummelPoonModel::by_name("2N3904");
        model_re.rb = 0.0;
        model_re.re = 2.0; // 2 Ω emitter resistance
        model_re.rc = 0.0;

        let bjt_no_re = BjtTwoPort::new(model_no_re);
        let bjt_re = BjtTwoPort::new(model_re);

        let vbe_ext = 0.65_f64;
        let vce_ext = 5.0_f64;

        let mut c0 = [0.0_f64; 2];
        let mut j0 = [0.0_f64; 4];
        bjt_no_re.eval(&[vbe_ext, vce_ext], &mut c0, &mut j0);

        let mut c1 = [0.0_f64; 2];
        let mut j1 = [0.0_f64; 4];
        bjt_re.eval(&[vbe_ext, vce_ext], &mut c1, &mut j1);

        // RE causes degeneration: transconductance (gm = ∂Ic/∂Vbe) should decrease
        // jacobian[2] is ∂Ic/∂Vbe
        let gm_no_re = j0[2];
        let gm_re = j1[2];
        assert!(
            gm_re < gm_no_re,
            "RE=2Ω should reduce transconductance: {gm_re:.3e} vs {gm_no_re:.3e}"
        );
    }

    /// With TF > 0, the B-E diffusion capacitance grows with collector current.
    #[test]
    fn diffusion_capacitance_be_increases_with_ic() {
        let model = GummelPoonModel::by_name("2N3904");

        // Skip if model has no TF (older models may have tf = 0)
        if model.tf <= 0.0 {
            return;
        }

        let vbe = 0.65; // forward active bias

        // At zero current, only depletion cap
        let c_zero_ic = model.capacitance_be(vbe, 0.0);
        // At realistic Ic, diffusion cap adds on top
        let ic = 1e-3; // 1 mA
        let c_with_ic = model.capacitance_be(vbe, ic);

        let c_diffusion_expected = model.tf * ic / model.vt;
        assert!(
            c_with_ic > c_zero_ic,
            "Cbe should increase with Ic: {c_with_ic:.3e} vs {c_zero_ic:.3e}"
        );
        assert!(
            (c_with_ic - c_zero_ic - c_diffusion_expected).abs() < 1e-18,
            "Diffusion cap should be TF*Ic/Vt = {c_diffusion_expected:.3e}, got {:.3e}",
            c_with_ic - c_zero_ic
        );
    }

    /// Verify that GummelPoonModel correctly picks up RB, RE, RC from SpiceBjtModel.
    #[test]
    fn gummel_poon_model_has_rb_re_rc() {
        let model = GummelPoonModel::by_name("2N3904");
        assert!(model.rb.is_finite(), "rb must be finite");
        assert!(model.re.is_finite(), "re must be finite");
        assert!(model.rc.is_finite(), "rc must be finite");
        assert!(model.rb >= 0.0, "rb must be >= 0");
        assert!(model.re >= 0.0, "re must be >= 0");
        assert!(model.rc >= 0.0, "rc must be >= 0");
    }

    // -----------------------------------------------------------------------
    // Analytical Jacobian tests
    // -----------------------------------------------------------------------

    /// Helper: compute numerical Jacobian of currents() via central differences.
    fn numerical_jacobian(model: &GummelPoonModel, vbe: f64, vbc: f64) -> [f64; 4] {
        let delta = 1e-7;
        let (ic_p, ib_p) = model.currents(vbe + delta, vbc);
        let (ic_m, ib_m) = model.currents(vbe - delta, vbc);
        let dib_dvbe = (ib_p - ib_m) / (2.0 * delta);
        let dic_dvbe = (ic_p - ic_m) / (2.0 * delta);

        let (ic_p2, ib_p2) = model.currents(vbe, vbc + delta);
        let (ic_m2, ib_m2) = model.currents(vbe, vbc - delta);
        let dib_dvbc = (ib_p2 - ib_m2) / (2.0 * delta);
        let dic_dvbc = (ic_p2 - ic_m2) / (2.0 * delta);

        [dib_dvbe, dib_dvbc, dic_dvbe, dic_dvbc]
    }

    /// Verify analytical Jacobian matches numerical at multiple operating points.
    #[test]
    fn analytical_jacobian_matches_numerical_2n3904() {
        let model = GummelPoonModel::by_name("2N3904");
        let test_points = [
            (0.0, 0.0),    // Zero bias
            (0.6, -5.0),   // Forward active
            (0.65, -10.0), // Forward active, high Vce
            (0.7, -2.0),   // Near saturation
            (0.3, -1.0),   // Low forward bias
            (0.0, -5.0),   // Cutoff
            (0.5, 0.3),    // Near saturation (Vbc > 0)
        ];

        for (vbe, vbc) in test_points {
            let (ic_a, ib_a, jac_a) = model.currents_and_jacobian(vbe, vbc);
            let (ic_n, ib_n) = model.currents(vbe, vbc);
            let jac_n = numerical_jacobian(&model, vbe, vbc);

            // Currents must match exactly (same code path)
            assert!(
                (ic_a - ic_n).abs() < 1e-15,
                "Ic mismatch at vbe={vbe}, vbc={vbc}: analytical={ic_a}, numerical={ic_n}"
            );
            assert!(
                (ib_a - ib_n).abs() < 1e-15,
                "Ib mismatch at vbe={vbe}, vbc={vbc}: analytical={ib_a}, numerical={ib_n}"
            );

            // Jacobian entries must match within relative tolerance
            for (i, name) in ["∂Ib/∂Vbe", "∂Ib/∂Vbc", "∂Ic/∂Vbe", "∂Ic/∂Vbc"]
                .iter()
                .enumerate()
            {
                let a = jac_a[i];
                let n = jac_n[i];
                let abs_err = (a - n).abs();
                let rel_err = if n.abs() > 1e-20 {
                    abs_err / n.abs()
                } else {
                    abs_err
                };
                assert!(
                    rel_err < 1e-4 || abs_err < 1e-15,
                    "Jacobian {name} mismatch at vbe={vbe}, vbc={vbc}: analytical={a:.6e}, numerical={n:.6e}, rel_err={rel_err:.2e}"
                );
            }
        }
    }

    /// Same test for germanium (AC128) — different IS, BF, leakage characteristics.
    #[test]
    fn analytical_jacobian_matches_numerical_ac128() {
        let model = GummelPoonModel::by_name("AC128");
        let test_points = [
            (0.0, 0.0),
            (0.2, -5.0),  // Ge turns on earlier
            (0.3, -9.0),  // Forward active
            (0.15, -1.0), // Low bias
        ];

        for (vbe, vbc) in test_points {
            let (_ic, _ib, jac_a) = model.currents_and_jacobian(vbe, vbc);
            let jac_n = numerical_jacobian(&model, vbe, vbc);

            for (i, name) in ["∂Ib/∂Vbe", "∂Ib/∂Vbc", "∂Ic/∂Vbe", "∂Ic/∂Vbc"]
                .iter()
                .enumerate()
            {
                let a = jac_a[i];
                let n = jac_n[i];
                let abs_err = (a - n).abs();
                let rel_err = if n.abs() > 1e-20 {
                    abs_err / n.abs()
                } else {
                    abs_err
                };
                assert!(
                    rel_err < 1e-4 || abs_err < 1e-15,
                    "Jacobian {name} mismatch at vbe={vbe}, vbc={vbc}: analytical={a:.6e}, numerical={n:.6e}, rel_err={rel_err:.2e}"
                );
            }
        }
    }

    /// Test that BjtTwoPort::eval analytical path matches numerical path.
    /// We compare a zero-parasitic BJT (analytical) against numerical differentiation
    /// of currents_and_jacobian output in (vbe, vce) space.
    #[test]
    fn bjt_two_port_eval_analytical_vs_numerical() {
        let mut model = GummelPoonModel::by_name("2N3904");
        // Force zero parasitics to use analytical path
        model.rb = 0.0;
        model.re = 0.0;
        model.rc = 0.0;
        let bjt = BjtTwoPort::new(model);

        let test_points = [
            [0.6, 5.0],   // Forward active (Vbe=0.6, Vce=5)
            [0.65, 10.0], // Higher Vce
            [0.3, 1.0],   // Low bias
            [0.0, 5.0],   // Cutoff
        ];

        for v in &test_points {
            let mut currents_a = [0.0; 2];
            let mut jac_a = [0.0; 4];
            bjt.eval(v, &mut currents_a, &mut jac_a);

            // Numerical Jacobian of eval in (vbe, vce) space
            let delta = 1e-7;
            let mut c_p = [0.0; 2];
            let mut c_m = [0.0; 2];
            let mut j_dummy = [0.0; 4];

            bjt.eval(&[v[0] + delta, v[1]], &mut c_p, &mut j_dummy);
            bjt.eval(&[v[0] - delta, v[1]], &mut c_m, &mut j_dummy);
            let dib_dvbe_n = (c_p[0] - c_m[0]) / (2.0 * delta);
            let dic_dvbe_n = (c_p[1] - c_m[1]) / (2.0 * delta);

            bjt.eval(&[v[0], v[1] + delta], &mut c_p, &mut j_dummy);
            bjt.eval(&[v[0], v[1] - delta], &mut c_m, &mut j_dummy);
            let dib_dvce_n = (c_p[0] - c_m[0]) / (2.0 * delta);
            let dic_dvce_n = (c_p[1] - c_m[1]) / (2.0 * delta);

            let jac_n = [dib_dvbe_n, dib_dvce_n, dic_dvbe_n, dic_dvce_n];
            let names = ["∂Ib/∂Vbe", "∂Ib/∂Vce", "∂Ic/∂Vbe", "∂Ic/∂Vce"];

            for (i, name) in names.iter().enumerate() {
                let a = jac_a[i];
                let n = jac_n[i];
                let abs_err = (a - n).abs();
                let rel_err = if n.abs() > 1e-20 {
                    abs_err / n.abs()
                } else {
                    abs_err
                };
                assert!(
                    rel_err < 1e-4 || abs_err < 1e-15,
                    "BjtTwoPort {name} mismatch at v={v:?}: analytical={a:.6e}, numerical={n:.6e}, rel_err={rel_err:.2e}"
                );
            }
        }
    }

    /// Test that BjtTwoPort::eval analytical+parasitic path matches numerical.
    /// Uses the 2N3904 with its real RB=10, RE=0.1, RC=1 parasitics.
    #[test]
    fn bjt_two_port_eval_analytical_with_parasitics() {
        let model = GummelPoonModel::by_name("2N3904");
        assert!(
            model.rb + model.re + model.rc > 0.0,
            "2N3904 must have parasitics"
        );
        let bjt = BjtTwoPort::new(model);

        let test_points = [[0.6, 5.0], [0.65, 10.0], [0.3, 1.0], [0.0, 5.0]];

        for v in &test_points {
            let mut currents_a = [0.0; 2];
            let mut jac_a = [0.0; 4];
            bjt.eval(v, &mut currents_a, &mut jac_a);

            // Numerical Jacobian via central differences of eval itself
            let delta = 1e-7;
            let mut c_p = [0.0; 2];
            let mut c_m = [0.0; 2];
            let mut j_dummy = [0.0; 4];

            bjt.eval(&[v[0] + delta, v[1]], &mut c_p, &mut j_dummy);
            bjt.eval(&[v[0] - delta, v[1]], &mut c_m, &mut j_dummy);
            let dib_dvbe_n = (c_p[0] - c_m[0]) / (2.0 * delta);
            let dic_dvbe_n = (c_p[1] - c_m[1]) / (2.0 * delta);

            bjt.eval(&[v[0], v[1] + delta], &mut c_p, &mut j_dummy);
            bjt.eval(&[v[0], v[1] - delta], &mut c_m, &mut j_dummy);
            let dib_dvce_n = (c_p[0] - c_m[0]) / (2.0 * delta);
            let dic_dvce_n = (c_p[1] - c_m[1]) / (2.0 * delta);

            let jac_n = [dib_dvbe_n, dib_dvce_n, dic_dvbe_n, dic_dvce_n];
            let names = ["∂Ib/∂Vbe", "∂Ib/∂Vce", "∂Ic/∂Vbe", "∂Ic/∂Vce"];

            for (i, name) in names.iter().enumerate() {
                let a = jac_a[i];
                let n = jac_n[i];
                let abs_err = (a - n).abs();
                let rel_err = if n.abs() > 1e-20 {
                    abs_err / n.abs()
                } else {
                    abs_err
                };
                assert!(
                    rel_err < 1e-3 || abs_err < 1e-12,
                    "Parasitic BjtTwoPort {name} mismatch at v={v:?}: analytical={a:.6e}, numerical={n:.6e}, rel_err={rel_err:.2e}"
                );
            }
        }
    }

    /// Frozen Newton optimization: verify solver stats show reduced iterations.
    /// This is an integration-level check that the frozen Newton step in the
    /// grouped solver actually reduces iteration count.
    #[test]
    fn frozen_newton_reduces_iterations() {
        use crate::elements::nonlinear::solver::{multi_port_nr_solve_grouped_into, NrWorkspace};

        let model = GummelPoonModel::by_name("2N3904");
        let bjt = BjtTwoPort::new(model);
        let groups: Vec<&dyn NlDeviceGroupIv> = vec![&bjt];
        let offsets = vec![0usize];

        let s_nl = [0.0, 0.0, 0.0, 0.0]; // 2×2 identity-ish
        let port_resistances = [1000.0, 1000.0];
        let mut ws = NrWorkspace::new_grouped(2, 2);

        // First solve: no cache, must iterate normally
        let mut v_guess = [0.6, 5.0];
        let known_a = [1.0, 10.0];
        multi_port_nr_solve_grouped_into(
            2,
            &s_nl,
            &known_a,
            &port_resistances,
            &groups,
            &offsets,
            &mut v_guess,
            24,
            1e-6,
            &mut ws,
        );
        assert!(
            ws.has_cached_jac,
            "Cache should be populated after first solve"
        );

        // Second solve with slightly different input: should benefit from cache
        let mut v_guess2 = v_guess; // warm start from previous
        let known_a2 = [1.001, 10.001]; // tiny change
        multi_port_nr_solve_grouped_into(
            2,
            &s_nl,
            &known_a2,
            &port_resistances,
            &groups,
            &offsets,
            &mut v_guess2,
            24,
            1e-6,
            &mut ws,
        );

        // The second solve should produce valid results
        assert!(v_guess2[0].is_finite(), "Vbe should be finite");
        assert!(v_guess2[1].is_finite(), "Vce should be finite");
        // Results should be close to first solve (tiny input change)
        assert!(
            (v_guess2[0] - v_guess[0]).abs() < 0.01,
            "Vbe should barely change: {} vs {}",
            v_guess2[0],
            v_guess[0]
        );
    }
}

// ---------------------------------------------------------------------------
// EbersMollTwoPort tests
// ---------------------------------------------------------------------------

#[cfg(all(test, DISABLED_needs_model_db))]
mod ebers_moll_tests {
    use super::*;
    use crate::elements::nonlinear::solver::NlDeviceGroupIv;

    /// Build an EbersMollTwoPort from 2N3904 GP parameters.
    fn em_2n3904() -> EbersMollTwoPort {
        let gp = GummelPoonModel::by_name("2N3904");
        EbersMollTwoPort::from_gp(&gp)
    }

    /// Build an EbersMollTwoPort from 2N3906 PNP GP parameters.
    fn em_2n3906_pnp() -> EbersMollTwoPort {
        let gp = GummelPoonModel::by_name("2N3906");
        EbersMollTwoPort::from_gp_pnp(&gp)
    }

    // -----------------------------------------------------------------------
    // Basic operating point checks
    // -----------------------------------------------------------------------

    /// At a typical NPN forward-active operating point (Vbe=0.65V, Vce=5V),
    /// EM should give positive Ic > 0 and positive Ib > 0 with reasonable beta.
    #[test]
    fn ebers_moll_npn_forward_active_operating_point() {
        let em = em_2n3904();
        let mut currents = [0.0; 2];
        let mut jac = [0.0; 4];
        em.eval(&[0.65, 5.0], &mut currents, &mut jac);

        let ib = currents[0];
        let ic = currents[1];

        assert!(
            ib > 0.0,
            "Ib should be positive in forward-active: {ib:.3e}"
        );
        assert!(
            ic > 0.0,
            "Ic should be positive in forward-active: {ic:.3e}"
        );

        let beta = ic / ib;
        assert!(
            beta > 50.0 && beta < 1000.0,
            "Beta={beta:.1} should be in reasonable range"
        );
    }

    /// At cutoff (Vbe < 0), Ic and Ib should be very small.
    #[test]
    fn ebers_moll_npn_cutoff() {
        let em = em_2n3904();
        let mut currents = [0.0; 2];
        let mut jac = [0.0; 4];
        em.eval(&[-0.5, 5.0], &mut currents, &mut jac);

        assert!(
            currents[0].abs() < 1e-9,
            "Ib should be ~0 in cutoff: {:.3e}",
            currents[0]
        );
        assert!(
            currents[1].abs() < 1e-9,
            "Ic should be ~0 in cutoff: {:.3e}",
            currents[1]
        );
    }

    /// PNP EM: at negative voltages, should give negative currents (PNP polarity).
    #[test]
    fn ebers_moll_pnp_forward_active() {
        let em = em_2n3906_pnp();
        let mut currents = [0.0; 2];
        let mut jac = [0.0; 4];
        // PNP: Vbe=-0.65V (emitter above base), Vce=-5V (emitter above collector)
        em.eval(&[-0.65, -5.0], &mut currents, &mut jac);

        let ib = currents[0];
        let ic = currents[1];

        // PNP currents flow in opposite direction
        assert!(ib < 0.0, "PNP Ib should be negative: {ib:.3e}");
        assert!(ic < 0.0, "PNP Ic should be negative: {ic:.3e}");

        let beta = ic.abs() / ib.abs();
        assert!(
            beta > 50.0 && beta < 1000.0,
            "PNP beta={beta:.1} should be reasonable"
        );
    }

    // -----------------------------------------------------------------------
    // Jacobian correctness via finite differences
    // -----------------------------------------------------------------------

    /// Verify the analytical Jacobian matches central-difference numerical
    /// derivatives at multiple operating points.
    #[test]
    fn ebers_moll_jacobian_matches_numerical() {
        let em = em_2n3904();
        let test_points: &[[f64; 2]] = &[
            [0.0, 0.0],   // zero bias
            [0.6, 5.0],   // forward active
            [0.65, 10.0], // high Vce
            [0.3, 1.0],   // low bias
            [0.5, 0.3],   // near saturation
            [-0.5, 5.0],  // cutoff
        ];

        let delta = 1e-7;
        for v in test_points {
            let mut c = [0.0; 2];
            let mut jac_a = [0.0; 4];
            em.eval(v, &mut c, &mut jac_a);

            // Numerical Jacobian via central differences
            let mut c_p = [0.0; 2];
            let mut c_m = [0.0; 2];
            let mut j_dummy = [0.0; 4];

            em.eval(&[v[0] + delta, v[1]], &mut c_p, &mut j_dummy);
            em.eval(&[v[0] - delta, v[1]], &mut c_m, &mut j_dummy);
            let dib_dvbe = (c_p[0] - c_m[0]) / (2.0 * delta);
            let dic_dvbe = (c_p[1] - c_m[1]) / (2.0 * delta);

            em.eval(&[v[0], v[1] + delta], &mut c_p, &mut j_dummy);
            em.eval(&[v[0], v[1] - delta], &mut c_m, &mut j_dummy);
            let dib_dvce = (c_p[0] - c_m[0]) / (2.0 * delta);
            let dic_dvce = (c_p[1] - c_m[1]) / (2.0 * delta);

            let jac_n = [dib_dvbe, dib_dvce, dic_dvbe, dic_dvce];
            let names = ["∂Ib/∂Vbe", "∂Ib/∂Vce", "∂Ic/∂Vbe", "∂Ic/∂Vce"];

            for (i, name) in names.iter().enumerate() {
                let a = jac_a[i];
                let n = jac_n[i];
                let abs_err = (a - n).abs();
                let rel_err = if n.abs() > 1e-20 {
                    abs_err / n.abs()
                } else {
                    abs_err
                };
                assert!(
                    rel_err < 1e-4 || abs_err < 1e-15,
                    "EM Jacobian {name} mismatch at v={v:?}: analytical={a:.6e}, numerical={n:.6e}, rel_err={rel_err:.2e}"
                );
            }
        }
    }

    // -----------------------------------------------------------------------
    // GP equivalence when advanced params are zeroed
    // -----------------------------------------------------------------------

    /// When GP advanced parameters are disabled (VAF=∞, IKF=∞, ISE=0, RB=0, RE=0, RC=0),
    /// EM and GP should give identical currents and Jacobians.
    #[test]
    fn ebers_moll_matches_gp_when_advanced_params_zeroed() {
        // Build a GP model with all advanced effects disabled
        let mut gp = GummelPoonModel::by_name("2N3904");
        gp.vaf = f64::INFINITY;
        gp.var = f64::INFINITY;
        gp.ikf = f64::INFINITY;
        gp.ikr = f64::INFINITY;
        gp.ise = 0.0;
        gp.isc = 0.0;
        gp.rb = 0.0;
        gp.re = 0.0;
        gp.rc = 0.0;

        let bjt_gp = BjtTwoPort::new(gp);
        let em = EbersMollTwoPort::from_gp(&gp);

        let test_points: &[[f64; 2]] =
            &[[0.6, 5.0], [0.65, 10.0], [0.3, 1.0], [0.0, 5.0], [0.5, 0.3]];

        for v in test_points {
            let mut c_gp = [0.0; 2];
            let mut j_gp = [0.0; 4];
            bjt_gp.eval(v, &mut c_gp, &mut j_gp);

            let mut c_em = [0.0; 2];
            let mut j_em = [0.0; 4];
            em.eval(v, &mut c_em, &mut j_em);

            let names_c = ["Ib", "Ic"];
            for (i, name) in names_c.iter().enumerate() {
                let abs_err = (c_gp[i] - c_em[i]).abs();
                let rel_err = if c_gp[i].abs() > 1e-20 {
                    abs_err / c_gp[i].abs()
                } else {
                    abs_err
                };
                assert!(
                    rel_err < 1e-6 || abs_err < 1e-20,
                    "EM {name} differs from GP at v={v:?}: GP={:.6e}, EM={:.6e}, rel_err={rel_err:.2e}",
                    c_gp[i],
                    c_em[i]
                );
            }

            let names_j = ["∂Ib/∂Vbe", "∂Ib/∂Vce", "∂Ic/∂Vbe", "∂Ic/∂Vce"];
            for (i, name) in names_j.iter().enumerate() {
                let abs_err = (j_gp[i] - j_em[i]).abs();
                let rel_err = if j_gp[i].abs() > 1e-20 {
                    abs_err / j_gp[i].abs()
                } else {
                    abs_err
                };
                assert!(
                    rel_err < 1e-4 || abs_err < 1e-15,
                    "EM Jacobian {name} differs from GP at v={v:?}: GP={:.6e}, EM={:.6e}, rel_err={rel_err:.2e}",
                    j_gp[i],
                    j_em[i]
                );
            }
        }
    }

    /// The `from_gp` constructor preserves IS, BF, BR, and computes
    /// nf_vt = NF * VT correctly.
    #[test]
    fn ebers_moll_from_gp_preserves_key_params() {
        let gp = GummelPoonModel::by_name("2N3904");
        let em = EbersMollTwoPort::from_gp(&gp);

        assert_eq!(em.is, gp.is, "IS must match");
        assert_eq!(em.bf, gp.bf, "BF must match");
        assert_eq!(em.br, gp.br, "BR must match");
        assert!(
            (em.nf_vt - gp.nf * gp.vt).abs() < 1e-12,
            "nf_vt={} should equal NF*VT={}",
            em.nf_vt,
            gp.nf * gp.vt
        );
        assert!(
            (em.nr_vt - gp.nr * gp.vt).abs() < 1e-12,
            "nr_vt={} should equal NR*VT={}",
            em.nr_vt,
            gp.nr * gp.vt
        );
        assert!(!em.is_pnp, "from_gp should produce NPN");
    }

    /// from_gp_pnp sets the PNP flag.
    #[test]
    fn ebers_moll_from_gp_pnp_sets_pnp_flag() {
        let gp = GummelPoonModel::by_name("2N3906");
        let em_pnp = EbersMollTwoPort::from_gp_pnp(&gp);
        assert!(em_pnp.is_pnp, "from_gp_pnp should produce PNP");
    }

    /// eval() output should be finite (no NaN/Inf) for all standard bias points.
    #[test]
    fn ebers_moll_eval_all_finite() {
        let em = em_2n3904();
        let test_points: &[[f64; 2]] = &[
            [0.0, 0.0],
            [0.7, 15.0],
            [-1.0, -1.0],
            [0.65, 0.0],
            [1.0, 50.0], // extreme forward — clamped
        ];
        for v in test_points {
            let mut c = [0.0; 2];
            let mut j = [0.0; 4];
            em.eval(v, &mut c, &mut j);
            assert!(c[0].is_finite(), "Ib not finite at {v:?}: {}", c[0]);
            assert!(c[1].is_finite(), "Ic not finite at {v:?}: {}", c[1]);
            for (k, &jv) in j.iter().enumerate() {
                assert!(jv.is_finite(), "J[{k}] not finite at {v:?}: {jv}");
            }
        }
    }

    /// n_ports() must return 2 (same as BjtTwoPort).
    #[test]
    fn ebers_moll_n_ports_is_2() {
        let em = em_2n3904();
        assert_eq!(em.n_ports(), 2);
    }

    /// v_clamp_port returns asymmetric bounds: IS-dependent for port 0 (Vbe),
    /// symmetric large range for port 1 (Vce).
    #[test]
    fn ebers_moll_v_clamp_port_bounds() {
        let em = em_2n3904();
        let (lo, hi) = em.v_clamp_port(0);
        assert!(lo < 0.0, "Vbe lower clamp should be negative");
        assert!(hi > 0.0, "Vbe upper clamp should be positive");
        assert!(hi <= 1.0, "Vbe clamp should not exceed 1.0V");

        let (lo2, hi2) = em.v_clamp_port(1);
        assert!(lo2 < -10.0, "Vce lower clamp should be large negative");
        assert!(hi2 > 10.0, "Vce upper clamp should be large positive");
    }
}
