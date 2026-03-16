//! BJT (Bipolar Junction Transistor) WDF root elements.
//!
//! Standalone BJTs use the Gummel-Poon two-port model (`BjtTwoPort`) through
//! the multi-port Newton-Raphson grouped solver. The legacy single-port
//! Ebers-Moll roots (`BjtNpnRoot`, `BjtPnpRoot`, `BjtModel`) have been removed.

use super::solver::NlDeviceGroupIv;
use crate::models::{bjt_by_name, SpiceBjtModel};

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
    /// # Panics
    /// Panics if the model name is not found. Use `try_by_name` for fallible lookup.
    pub fn by_name(name: &str) -> Self {
        Self::try_by_name(name).unwrap_or_else(|| panic!("Unknown BJT model: '{name}'"))
    }

    /// Try to look up a Gummel-Poon model by name. Returns `None` if not found.
    pub fn try_by_name(name: &str) -> Option<Self> {
        bjt_by_name(name).map(Self::from)
    }

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
        let exp_vbe = ((vbe / (self.nf * self.vt)).min(40.0)).exp();
        let exp_vbc = ((vbc / (self.nr * self.vt)).min(40.0)).exp();
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
        (q1 / 2.0) * (1.0 + (1.0 + 4.0 * q2).max(0.0).sqrt())
    }

    /// Compute collector current using Gummel-Poon transport equations.
    ///
    /// Returns (Ic, Ib) tuple.
    #[inline]
    pub fn currents(&self, vbe: f64, vbc: f64) -> (f64, f64) {
        // Compute all exponentials once — shared between base_charge and transport
        let exp_vbe = ((vbe / (self.nf * self.vt)).min(40.0)).exp();
        let exp_vbc = ((vbc / (self.nr * self.vt)).min(40.0)).exp();

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
            self.ise * (((vbe / (self.ne * self.vt)).min(40.0)).exp() - 1.0)
        } else {
            0.0
        };
        let ib_leak_c = if self.isc > 0.0 {
            self.isc * (((vbc / (self.nc * self.vt)).min(40.0)).exp() - 1.0)
        } else {
            0.0
        };
        let ib = ib_f + ib_r + ib_leak_e + ib_leak_c;

        (ic, ib)
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
                self.cje / (1.0 - vbe / self.vje).powf(self.mje)
            } else {
                // Linear extrapolation for forward bias
                let cje_fc = self.cje / (1.0 - fc).powf(self.mje);
                let dcje = self.cje * self.mje / (self.vje * (1.0 - fc).powf(self.mje + 1.0));
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
                self.cjc / (1.0 - vbc / self.vjc).powf(self.mjc)
            } else {
                let cjc_fc = self.cjc / (1.0 - fc).powf(self.mjc);
                let dcjc = self.cjc * self.mjc / (self.vjc * (1.0 - fc).powf(self.mjc + 1.0));
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

impl From<&SpiceBjtModel> for GummelPoonModel {
    fn from(spice: &SpiceBjtModel) -> Self {
        Self {
            is: spice.is,
            bf: spice.bf,
            br: spice.br,
            nf: spice.nf,
            nr: spice.nr,
            vt: 0.02585, // kT/q at 25°C
            vaf: spice.vaf,
            var: spice.var,
            ikf: spice.ikf,
            ikr: spice.ikr,
            ise: spice.ise,
            ne: spice.ne,
            isc: spice.isc,
            nc: spice.nc,
            cje: spice.cje,
            vje: spice.vje,
            mje: spice.mje,
            cjc: spice.cjc,
            vjc: spice.vjc,
            mjc: spice.mjc,
            rb: spice.rb,
            re: spice.re,
            rc: spice.rc,
            tf: spice.tf,
            tr: spice.tr,
            is_pnp: spice.is_pnp,
        }
    }
}

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
pub struct BjtTwoPort {
    pub model: GummelPoonModel,
    pub(crate) is_pnp: bool,
    v_max: f64,
    /// IS-dependent Vbe clamp: NF * VT * ln(I_max / IS).
    /// Prevents exponential blow-up at forward bias for high-IS devices (Ge).
    vbe_max: f64,
}

impl BjtTwoPort {
    /// Compute IS-dependent Vbe clamp for max 100mA collector current.
    fn compute_vbe_max(model: &GummelPoonModel) -> f64 {
        let i_max = 0.1; // 100mA — generous bound for small-signal BJTs
        let vbe_max = model.nf * model.vt * (i_max / model.is).ln();
        vbe_max.clamp(0.3, 1.0) // At least 0.3V, at most 1.0V
    }

    /// Create a new NPN BjtTwoPort.
    pub fn new(model: GummelPoonModel) -> Self {
        let vbe_max = Self::compute_vbe_max(&model);
        Self {
            model,
            is_pnp: false,
            v_max: 50.0,
            vbe_max,
        }
    }

    /// Create a new PNP BjtTwoPort.
    pub fn new_pnp(model: GummelPoonModel) -> Self {
        let vbe_max = Self::compute_vbe_max(&model);
        Self {
            model,
            is_pnp: true,
            v_max: 50.0,
            vbe_max,
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
        // When Vbe and Vce are clamped independently, their combination can
        // produce Vbc >> 0 (e.g., PNP Vbe=0.5, Vce=-9 → Vbc=9.5V → ISC*exp(365)).
        // Limit to 0.4V forward to allow saturation while preventing overflow.
        const VBC_MAX: f64 = 0.4;
        let clamp_vbc = |vbe: f64, vce: f64| -> f64 { (vbe - vce).min(VBC_MAX) };

        // Helper: given external (Vbe_ext, Vce_ext), return (Ic, Ib) accounting for
        // terminal resistances RB, RE, RC via fixed-point iteration.
        //
        // For NPN with terminal resistances in each lead:
        //   Ve = Ie_out * RE  where Ie_out = Ic + Ib  (KCL: current out of emitter)
        //   Vb = Vbe_ext - Ib * RB  (drop across base resistance, external → base node)
        //   Vc = Vce_ext - Ic * RC  (drop across collector resistance)
        //
        //   V_be_int = Vb - Ve = Vbe_ext - Ib * RB - (Ic + Ib) * RE
        //   V_ce_int = Vc - Ve = Vce_ext - Ic * RC - (Ic + Ib) * RE
        //
        // Since RB/RE/RC ≤ 40 Ω and currents ≤ ~100 mA, drops are ≤ a few mV.
        // 4 iterations are sufficient for convergence (verified analytically).
        let eval_with_parasitics = |vbe_ext: f64, vce_ext: f64| -> (f64, f64) {
            let rb = self.model.rb;
            let re = self.model.re;
            let rc = self.model.rc;
            let has_parasitics = rb + re + rc > 0.0;

            if !has_parasitics {
                let vbc = clamp_vbc(vbe_ext, vce_ext);
                return self.model.currents(vbe_ext, vbc);
            }

            // Fixed-point iteration: voltage updates, then final eval at converged voltages.
            // Each iteration computes currents → voltage drops → updated internal voltages.
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

        // 2×2 Jacobian via numerical differentiation.
        // sign² = 1, so Jacobian is polarity-invariant.
        // The parasitic-resistance iteration runs inside the perturbed evaluations
        // so the Jacobian naturally captures terminal-R effects.
        let delta = 1e-7;

        // ∂/∂Vbe (port 0): vbe varies, vce fixed
        let (ic_p, ib_p) = eval_with_parasitics(vbe_ext + delta, vce_ext);
        let (ic_m, ib_m) = eval_with_parasitics(vbe_ext - delta, vce_ext);
        jacobian[0] = (ib_p - ib_m) / (2.0 * delta); // ∂Ib/∂Vbe
        jacobian[2] = (ic_p - ic_m) / (2.0 * delta); // ∂Ic/∂Vbe (gm!)

        // ∂/∂Vce (port 1): vce varies, vbe fixed
        let (ic_p2, ib_p2) = eval_with_parasitics(vbe_ext, vce_ext + delta);
        let (ic_m2, ib_m2) = eval_with_parasitics(vbe_ext, vce_ext - delta);
        jacobian[1] = (ib_p2 - ib_m2) / (2.0 * delta); // ∂Ib/∂Vce
        jacobian[3] = (ic_p2 - ic_m2) / (2.0 * delta); // ∂Ic/∂Vce (Early)
    }

    fn v_clamp_port(&self, port: usize) -> (f64, f64) {
        match port {
            0 => (-self.vbe_max, self.vbe_max), // Vbe: IS-dependent (Ge ~0.32V, Si ~0.83V)
            _ => (-self.v_max, self.v_max),     // Vce: full swing
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
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
        // Fields must exist and be finite; exact values depend on SPICE model data
        assert!(model.rb.is_finite(), "rb must be finite");
        assert!(model.re.is_finite(), "re must be finite");
        assert!(model.rc.is_finite(), "rc must be finite");
        // Should be non-negative (physical resistance)
        assert!(model.rb >= 0.0, "rb must be >= 0");
        assert!(model.re >= 0.0, "re must be >= 0");
        assert!(model.rc >= 0.0, "rc must be >= 0");
    }
}
