//! Jiles-Atherton nonlinear magnetizing branch for transformer cores.
//!
//! This is a WDF one-port for the magnetizing shunt branch in the transformer
//! T-equivalent. The electrical topology around it remains linear; the core
//! history lives in `(H, Hdot, M, Mdot)` and is advanced by an implicit
//! trapezoidal solve per sample.
//!
//! The susceptibility form is the Chowdhury/Holters-Zolzer bulk-susceptibility
//! arrangement used by the Python oracle in `pedalkernel-validate/oracles`.
//! The reversal indicators `delta` and `delta_m` are intentionally frozen
//! within a Newton iteration, making the solve piecewise-smooth at field
//! reversals.

use crate::{math, Wave};

const MU0: Wave = 1.256_637_061_435_917_3e-6;
const MAX_ITERS: usize = 12;
const STEP_LIMIT_A: Wave = 50.0;

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct JaCoreModel {
    pub ms: Wave,
    pub a: Wave,
    pub alpha: Wave,
    pub k: Wave,
    pub c: Wave,
    pub n_turns: Wave,
    pub area: Wave,
    pub path_len: Wave,
    pub gap: Wave,
}

impl JaCoreModel {
    pub fn si_steel_demo() -> Self {
        Self {
            ms: 1.6e6,
            a: 1100.0,
            alpha: 1.6e-3,
            k: 400.0,
            c: 0.2,
            n_turns: 2000.0,
            area: 2.0e-4,
            path_len: 0.10,
            gap: 0.0,
        }
    }

    pub fn is_complete(self) -> bool {
        self.ms > 0.0
            && self.a > 0.0
            && self.k > 0.0
            && self.n_turns > 0.0
            && self.area > 0.0
            && self.path_len > 0.0
            && (0.0..=1.0).contains(&self.c)
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct JaState {
    pub h: Wave,
    pub h_dot: Wave,
    pub m: Wave,
    pub m_dot: Wave,
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct JaMagnetizingRoot {
    model: JaCoreModel,
    state: JaState,
    k_faraday: Wave,
    le_eff: Wave,
    fs: Wave,
    t: Wave,
    rp: Wave,
    pub last_iters: usize,
}

impl JaMagnetizingRoot {
    pub fn new(model: JaCoreModel) -> Self {
        Self {
            k_faraday: model.n_turns * model.area * MU0,
            le_eff: model.path_len + model.gap,
            model,
            state: JaState::default(),
            fs: 0.0,
            t: 0.0,
            rp: 0.0,
            last_iters: 0,
        }
    }

    pub fn prepare(&mut self, sample_rate: Wave, port_resistance: Wave) {
        self.fs = sample_rate;
        self.t = 1.0 / sample_rate;
        self.rp = port_resistance;
        self.reset();
    }

    pub fn reset(&mut self) {
        self.state = JaState::default();
        self.last_iters = 0;
    }

    /// Seed the core from a standing primary DC current.
    ///
    /// Single-ended output transformers deliberately carry DC through the
    /// primary. Starting a gapped core at `(H, M) = (0, 0)` and asking the
    /// audio-rate transient solve to find that operating point creates an
    /// artificial startup hit. This initializer solves the static Ampere law
    /// plus the anhysteretic magnetization curve and leaves derivative state
    /// at zero.
    pub fn set_dc_bias_current(&mut self, current: Wave) {
        if !current.is_finite() || !self.model.is_complete() {
            return;
        }

        let p = self.model;
        let ampere_turns = p.n_turns * current;
        let mut m: Wave = 0.0;
        if p.gap <= 0.0 {
            let h = ampere_turns / self.le_eff.max(1.0e-12);
            m = solve_anhysteretic_m(&p, h, 0.0);
            self.state = JaState {
                h,
                h_dot: 0.0,
                m,
                m_dot: 0.0,
            };
            return;
        }

        // Gap couples M back into Ampere's law. Iterate H(M) and M_an(H, M).
        for _ in 0..32 {
            let h = (ampere_turns - m * p.gap) / self.le_eff.max(1.0e-12);
            let next_m = solve_anhysteretic_m(&p, h, m);
            if math::abs(next_m - m) <= 1.0e-9 * p.ms.max(1.0) {
                m = next_m;
                break;
            }
            m = 0.5 * m + 0.5 * next_m;
        }

        let h = (ampere_turns - m * p.gap) / self.le_eff.max(1.0e-12);
        self.state = JaState {
            h,
            h_dot: 0.0,
            m,
            m_dot: 0.0,
        };
    }

    #[inline]
    fn winding_current(&self, h: Wave, m: Wave) -> Wave {
        (h * self.le_eff + m * self.model.gap) / self.model.n_turns
    }

    pub fn process(&mut self, a: Wave) -> Wave {
        if self.fs <= 0.0 || self.rp <= 0.0 || !self.model.is_complete() {
            return -a;
        }

        let p = self.model;
        let st = self.state;
        let rp_over_n = self.rp / p.n_turns;
        let mut h = st.h;
        let mut m = st.m;
        let mut iters = MAX_ITERS;

        for it in 0..MAX_ITERS {
            let h_dot = 2.0 * self.fs * (h - st.h) - st.h_dot;
            let e = mdot_eval(&p, h, m, h_dot);
            let f1 =
                a - rp_over_n * (h * self.le_eff + m * p.gap) - self.k_faraday * (h_dot + e.value);
            let f2 = m - st.m - 0.5 * self.t * (e.value + st.m_dot);

            if math::abs(f1) < 1.0e-9 * (1.0 + math::abs(a)) && math::abs(f2) < 1.0e-9 * p.ms {
                iters = it;
                break;
            }

            let dmdot_dh_tot = e.d_dh + e.d_dhdot * 2.0 * self.fs;
            let j11 = -rp_over_n * self.le_eff - self.k_faraday * (2.0 * self.fs + dmdot_dh_tot);
            let j12 = -rp_over_n * p.gap - self.k_faraday * e.d_dm;
            let j21 = -0.5 * self.t * dmdot_dh_tot;
            let j22 = 1.0 - 0.5 * self.t * e.d_dm;
            let det = j11 * j22 - j12 * j21;
            if math::abs(det) < 1.0e-30 {
                iters = it;
                break;
            }

            let mut dh = (j22 * f1 - j12 * f2) / det;
            let mut dm = (-j21 * f1 + j11 * f2) / det;
            let step_max = STEP_LIMIT_A * p.a;
            if math::abs(dh) > step_max {
                let s = step_max / math::abs(dh);
                dh *= s;
                dm *= s;
            }

            h -= dh;
            m -= dm;
            // Hard physical bound: |M| cannot exceed Ms (Langevin saturation).
            // The H-space step limiter (STEP_LIMIT_A * a) is not tight enough
            // at small `a` — clamp M here as the authoritative guard.
            m = m.clamp(-p.ms, p.ms);
        }
        self.last_iters = iters;

        let h_dot = 2.0 * self.fs * (h - st.h) - st.h_dot;
        let e = mdot_eval(&p, h, m, h_dot);
        let i = self.winding_current(h, m);
        let b = a - 2.0 * self.rp * i;
        self.state = JaState {
            h,
            h_dot,
            m,
            m_dot: e.value,
        };
        b
    }

    pub fn flux_density(&self) -> Wave {
        MU0 * (self.state.h + self.state.m)
    }

    pub fn state(&self) -> JaState {
        self.state
    }

    pub fn model(&self) -> JaCoreModel {
        self.model
    }

    pub fn port_resistance(&self) -> Wave {
        self.rp
    }
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WdfJaMagnetizing {
    pub comp_id: Option<alloc::string::String>,
    pub root: JaMagnetizingRoot,
    pub last_b: Wave,
}

impl WdfJaMagnetizing {
    pub fn new(
        comp_id: Option<alloc::string::String>,
        model: JaCoreModel,
        sample_rate: Wave,
        port_resistance: Wave,
    ) -> Self {
        Self::new_with_dc_bias(comp_id, model, sample_rate, port_resistance, 0.0)
    }

    pub fn new_with_dc_bias(
        comp_id: Option<alloc::string::String>,
        model: JaCoreModel,
        sample_rate: Wave,
        port_resistance: Wave,
        dc_bias_current: Wave,
    ) -> Self {
        let mut root = JaMagnetizingRoot::new(model);
        root.prepare(sample_rate, port_resistance);
        root.set_dc_bias_current(dc_bias_current);
        Self {
            comp_id,
            root,
            last_b: 0.0,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Voltage-driven tape head (TapeHeadVoltageRoot)
// ═══════════════════════════════════════════════════════════════════════════
//
// The transformer J-A core above is CURRENT-driven via Ampere's law
// (`I = (H*le + M*gap)/N`). With studio turns counts the knee needs ~hundreds
// of mA — a line op-amp cannot source that, so the core never saturates with
// level. A record/playback tape head instead drives the magnetic field H from
// the head GAP VOLTAGE: the head is a tiny coil/gap whose H scales with the
// applied voltage, so saturation tracks signal level at ~1 V line drive.
//
// CONSTITUTIVE RELATION (the shipped port law)
// --------------------------------------------
// One-port WDF with port resistance Rp. Port voltage and current:
//   V = (a + b) / 2          i = (a - b) / (2*Rp)
// so with b = 2V - a we have the convenient identity  i = (a - V) / Rp.
//
// The head drives the field linearly from the gap voltage, with a small fixed
// record-bias offset that makes the transfer ASYMMETRIC (even-harmonic, tape-
// like — a symmetric drive would be odd-only):
//                          H = kv * V + h_bias                          [A/m]
// Magnetization M is advanced along the Jiles-Atherton hysteresis trajectory
// RATE-INDEPENDENTLY in the FIELD (H) domain (no time derivative → no audio-
// rate stiffness): `dM/dH = ja_slope(H, M, sign(dH))`, integrated with one
// sub-stepped field step from H_prev to H (see `ja_field_step`). The port
// current is a small linear leakage plus a saturating magnetic term:
//                       i(V) = Gp*V + Isat * (M / Ms)
// M is bounded by the Langevin saturation built into the J-A curve, so as |V|
// grows past the knee the magnetic current compresses -> harmonics.
//
// Each sample we solve the scalar port KCL for V by damped Newton:
//   f(V) = (a - V)/Rp - Gp*V - Isat*M(H(V))/Ms = 0
// with M(H) and dM/dH supplied by the field-domain J-A step (the field kink at
// reversals is handled by a descent-guarded step). Then b = 2V - a. `kv` is
// chosen so H reaches a few * `a` (A/m) near V ~ 1 V, placing the knee at line
// level; Rp sets a well-conditioned scattering port (gamma neither ~0 nor ~1).

#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TapeHeadVoltageRoot {
    model: JaCoreModel,
    /// Field-per-volt coupling (A/m per V). Places the J-A knee at line level.
    kv: Wave,
    /// Saturating magnetic current scale (A) — sets harmonic drive strength.
    isat: Wave,
    /// Small linear leakage conductance (S) keeping the port well-posed.
    gp: Wave,
    /// Fixed record-bias field offset (A/m): asymmetric transfer -> even
    /// harmonics (tape/transformer colour). 0 = symmetric (odd-only).
    h_bias: Wave,
    /// Magnetization, field and their bilinear derivatives (per-sample memory).
    m: Wave,
    m_dot: Wave,
    h: Wave,
    h_dot: Wave,
    v_prev: Wave,
    fs: Wave,
    t: Wave,
    rp: Wave,
    pub last_iters: usize,
}

impl TapeHeadVoltageRoot {
    pub fn new(model: JaCoreModel, kv: Wave, isat: Wave, gp: Wave, h_bias: Wave) -> Self {
        Self {
            model,
            kv,
            isat,
            gp,
            h_bias,
            m: 0.0,
            m_dot: 0.0,
            h: 0.0,
            h_dot: 0.0,
            v_prev: 0.0,
            fs: 0.0,
            t: 0.0,
            rp: 0.0,
            last_iters: 0,
        }
    }

    pub fn prepare(&mut self, sample_rate: Wave, port_resistance: Wave) {
        self.fs = sample_rate;
        self.t = 1.0 / sample_rate;
        self.rp = port_resistance;
        self.reset();
    }

    pub fn reset(&mut self) {
        // Seed the field at the record-bias operating point so the first sample
        // does not see a spurious dH jump from 0 to h_bias.
        self.h = self.h_bias;
        self.h_dot = 0.0;
        self.v_prev = 0.0;
        self.last_iters = 0;
        // Seat M on the J-A trajectory at the bias field (anhysteretic seed).
        self.m = if self.model.is_complete() {
            solve_anhysteretic_m(&self.model, self.h_bias, 0.0)
        } else {
            0.0
        };
        self.m_dot = 0.0;
    }

    pub fn port_resistance(&self) -> Wave {
        self.rp
    }

    pub fn model(&self) -> JaCoreModel {
        self.model
    }

    pub fn magnetization(&self) -> Wave {
        self.m
    }

    pub fn process(&mut self, a: Wave) -> Wave {
        if self.fs <= 0.0 || self.rp <= 0.0 || !self.model.is_complete() || self.isat <= 0.0 {
            // Degenerate config: behave as a plain Rp resistor (b = a - 2*Rp*i,
            // with i = a/(2*Rp) into a short -> b = 0). Returning -a matches the
            // reflection-free fallback used by the J-A magnetizing root.
            return -a;
        }

        // The port law is solved RATE-INDEPENDENTLY in the field (H) domain:
        // there is no bilinear time derivative scaled by kv, so the system is
        // well-conditioned and never stiff. Magnetization advances along the
        // Jiles-Atherton hysteresis trajectory as a function of H, integrated
        // with one implicit field step from H_prev to H = kv*V.
        const TAPE_MAX_ITERS: usize = 24;

        let p = self.model;
        let inv_ms = 1.0 / p.ms;
        let (h_prev, m_prev) = (self.h, self.m);

        // Newton initial guess: linearized port with no magnetic load.
        // i = (a - V)/Rp = Gp*V  ->  V0 = a / (1 + Gp*Rp).
        let mut v = a / (1.0 + self.gp * self.rp);
        let i_scale = 1.0 + math::abs(a) / self.rp;
        let mut iters = TAPE_MAX_ITERS;

        let mut f_prev = Wave::MAX;
        for it in 0..TAPE_MAX_ITERS {
            let h = self.kv * v + self.h_bias;
            // Magnetization on the J-A trajectory at this H (field domain),
            // plus its sensitivity dM/dH — both rate-independent.
            let (m, dm_dh) = ja_field_step(&p, h, h_prev, m_prev);
            let dm_dv = dm_dh * self.kv;

            // Port KCL residual: i = (a-V)/Rp must equal Gp*V + Isat*M/Ms.
            let f = (a - v) / self.rp - self.gp * v - self.isat * m * inv_ms;
            if math::abs(f) < 1.0e-10 * i_scale {
                iters = it;
                break;
            }
            // df/dV = -1/Rp - Gp - Isat/Ms * dM/dV.
            let df = -1.0 / self.rp - self.gp - self.isat * inv_ms * dm_dv;
            if math::abs(df) < 1.0e-30 {
                iters = it;
                break;
            }
            // Damped Newton with a kink guard: the field-domain slope has a
            // branch kink at field reversals (delta = sign(dH) flips). If the
            // raw step does not reduce |f|, halve it. This guarantees descent
            // and termination even across the reversal corner.
            let full = f / df;
            let mut step = full;
            if math::abs(f) >= math::abs(f_prev) {
                step *= 0.5;
            }
            v -= step;
            f_prev = math::abs(f);
            // Step-size convergence: at a field-reversal kink the residual can
            // sit just above the f-tolerance while V has effectively settled.
            if math::abs(step) < 1.0e-12 * (1.0 + math::abs(v)) {
                iters = it;
                break;
            }
        }
        self.last_iters = iters;

        // Commit the converged operating point (H includes the record bias).
        let h = self.kv * v + self.h_bias;
        let (m_final, _) = ja_field_step(&p, h, h_prev, m_prev);
        self.h = h;
        self.h_dot = 0.0;
        self.m = m_final;
        self.m_dot = 0.0;
        self.v_prev = v;

        // b = 2V - a.
        2.0 * v - a
    }
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WdfTapeHeadVoltage {
    pub comp_id: Option<alloc::string::String>,
    pub root: TapeHeadVoltageRoot,
    pub last_b: Wave,
}

impl WdfTapeHeadVoltage {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        comp_id: Option<alloc::string::String>,
        model: JaCoreModel,
        kv: Wave,
        isat: Wave,
        gp: Wave,
        h_bias: Wave,
        sample_rate: Wave,
        port_resistance: Wave,
    ) -> Self {
        let mut root = TapeHeadVoltageRoot::new(model, kv, isat, gp, h_bias);
        root.prepare(sample_rate, port_resistance);
        Self {
            comp_id,
            root,
            last_b: 0.0,
        }
    }
}

struct MdotEval {
    value: Wave,
    d_dm: Wave,
    d_dh: Wave,
    d_dhdot: Wave,
}

#[inline]
fn langevin(x: Wave) -> Wave {
    if math::abs(x) < 1.0e-4 {
        x / 3.0 - x * x * x / 45.0
    } else {
        1.0 / math::tanh(x) - 1.0 / x
    }
}

#[inline]
fn langevin_d(x: Wave) -> Wave {
    if math::abs(x) < 1.0e-4 {
        1.0 / 3.0 - x * x / 15.0
    } else {
        let s = math::sinh(x);
        1.0 / (x * x) - 1.0 / (s * s)
    }
}

#[inline]
fn langevin_d2(x: Wave) -> Wave {
    if math::abs(x) < 1.0e-3 {
        -2.0 * x / 15.0
    } else {
        let s = math::sinh(x);
        2.0 * math::cosh(x) / (s * s * s) - 2.0 / (x * x * x)
    }
}

/// Anhysteretic magnetization and its field slope at field `h`, with `m` the
/// current magnetization used for the (small) inter-domain coupling term.
#[inline]
fn anhysteretic(p: &JaCoreModel, h: Wave, m: Wave) -> (Wave, Wave) {
    let q = (h + p.alpha * m) / p.a;
    let m_an = p.ms * langevin(q);
    // dM_an/dH = Ms*L'(q)/a  (coupling alpha folded in by the caller's loop).
    let dman_dh = p.ms * langevin_d(q) / p.a;
    (m_an, dman_dh)
}

/// Instantaneous rate-independent Jiles-Atherton field slope dM/dH.
///
///   dM/dH = (1-c)*deltaM*(M_an - M) / (delta*k*(1-c) - alpha*(M_an - M))
///           + c * dM_an/dH
/// `delta = sign(dH)` selects the ascending/descending branch (hysteresis);
/// `deltaM` gates irreversible wall motion to the field direction. Clamped to
/// be non-negative (a soft-magnetic head cannot have dM/dH < 0).
#[inline]
fn ja_slope(p: &JaCoreModel, h: Wave, m: Wave, delta: Wave) -> Wave {
    let one_c = 1.0 - p.c;
    let kk = p.k.max(1.0e-9);
    let (m_an, dman_dh) = anhysteretic(p, h, m);
    let diff = m_an - m;
    let mut den = delta * kk * one_c - p.alpha * diff;
    let den_eps = 1.0e-9 * (1.0 + math::abs(diff));
    if math::abs(den) < den_eps {
        den = if den >= 0.0 { den_eps } else { -den_eps };
    }
    let delta_m = if diff * delta >= 0.0 { 1.0 } else { 0.0 };
    let slope = one_c * delta_m * diff / den + p.c * dman_dh;
    slope.max(p.c * dman_dh).max(0.0)
}

/// One rate-independent Jiles-Atherton step in the FIELD (H) domain.
///
/// Advances magnetization from `(h_prev, m_prev)` to field `h` along the J-A
/// hysteresis trajectory and returns `(M, dM/dH at h)`. Rate-independent (no
/// time derivative → no audio-rate stiffness). The field-domain ODE `dM/dH =
/// ja_slope(...)` is integrated with explicit sub-steps in H; the field
/// interval is subdivided so each sub-step moves M by a bounded fraction of
/// Ms, which keeps the integrator unconditionally well-behaved even under
/// extreme overdrive. The returned slope is the trajectory slope AT `h`, used
/// as the outer port-solver Jacobian.
#[inline]
fn ja_field_step(p: &JaCoreModel, h: Wave, h_prev: Wave, m_prev: Wave) -> (Wave, Wave) {
    let dh_total = h - h_prev;
    let delta: Wave = if dh_total >= 0.0 { 1.0 } else { -1.0 };

    // Sub-step count: bound |dH| per step so M can't jump more than ~Ms/64 at
    // the steepest slope (~Ms/a). nsub = ceil(|dH| * (Ms/a) / (Ms/64) ... )
    // simplified to |dH| / a scaled — cheap and conservative.
    let n = (math::abs(dh_total) / (0.25 * p.a)).ceil();
    let nsub = (n as usize).clamp(1, 256);
    let dh = dh_total / nsub as Wave;

    let mut m = m_prev.clamp(-p.ms, p.ms);
    let mut hh = h_prev;
    let mut slope = 0.0;
    for _ in 0..nsub {
        // Midpoint (RK2) in field for accuracy at modest cost.
        let s0 = ja_slope(p, hh, m, delta);
        let m_mid = (m + 0.5 * dh * s0).clamp(-p.ms, p.ms);
        let s_mid = ja_slope(p, hh + 0.5 * dh, m_mid, delta);
        m = (m + dh * s_mid).clamp(-p.ms, p.ms);
        hh += dh;
        slope = s_mid;
    }
    // Final slope evaluated at the endpoint for the outer Jacobian.
    slope = if nsub == 1 {
        ja_slope(p, h, m, delta)
    } else {
        slope
    };
    (m, slope)
}

fn solve_anhysteretic_m(p: &JaCoreModel, h: Wave, initial_m: Wave) -> Wave {
    let mut m = initial_m.clamp(-p.ms, p.ms);
    for _ in 0..24 {
        let q = (h + p.alpha * m) / p.a;
        let f = m - p.ms * langevin(q);
        let df = 1.0 - p.ms * langevin_d(q) * p.alpha / p.a;
        if math::abs(df) < 1.0e-12 {
            break;
        }
        let dm = (f / df).clamp(-0.25 * p.ms, 0.25 * p.ms);
        m = (m - dm).clamp(-p.ms, p.ms);
        if math::abs(dm) <= 1.0e-9 * p.ms.max(1.0) {
            break;
        }
    }
    m
}

#[inline]
fn mdot_eval(p: &JaCoreModel, h: Wave, m: Wave, h_dot: Wave) -> MdotEval {
    let q = (h + p.alpha * m) / p.a;
    let lp = langevin_d(q);
    let lpp = langevin_d2(q);
    let m_an = p.ms * langevin(q);
    let dm = m_an - m;
    let delta = if h_dot >= 0.0 { 1.0 } else { -1.0 };
    let delta_m = if dm * h_dot >= 0.0 { 1.0 } else { 0.0 };
    let one_c = 1.0 - p.c;

    let mut d_irr = one_c * delta * p.k - p.alpha * dm;
    let eps = 1.0e-9 * p.k.max(1.0);
    if math::abs(d_irr) < eps {
        d_irr = if d_irr >= 0.0 { eps } else { -eps };
    }

    let g = one_c * delta_m * dm / d_irr;
    let dg_du = one_c * one_c * delta_m * delta * p.k / (d_irr * d_irr);
    let r_coef = p.c * p.ms / p.a;
    let r = r_coef * lp;
    let dr_dq = r_coef * lpp;
    let mut den = 1.0 - p.alpha * r;
    // Sign-preserving epsilon clamp: den approaching zero makes chi diverge.
    // Guard with the same idiom used for d_irr above (eps scaled to chi's
    // numerator magnitude so the clamp only bites at near-singular corners).
    let den_eps = 1.0e-9 * (1.0 + math::abs(g + r));
    if math::abs(den) < den_eps {
        den = if den >= 0.0 { den_eps } else { -den_eps };
    }
    let dden_dq = -p.alpha * dr_dq;
    let chi = (g + r) / den;
    let value = chi * h_dot;

    let du_dh = p.ms * lp / p.a;
    let du_dm = p.alpha * p.ms * lp / p.a - 1.0;
    let dq_dh = 1.0 / p.a;
    let dq_dm = p.alpha / p.a;
    let num = g + r;
    let dnum_dh = dg_du * du_dh + dr_dq * dq_dh;
    let dnum_dm = dg_du * du_dm + dr_dq * dq_dm;
    let dchi_dh = (dnum_dh * den - num * dden_dq * dq_dh) / (den * den);
    let dchi_dm = (dnum_dm * den - num * dden_dq * dq_dm) / (den * den);

    MdotEval {
        value,
        d_dm: dchi_dm * h_dot,
        d_dh: dchi_dh * h_dot,
        d_dhdot: chi,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;

    const PI: Wave = core::f64::consts::PI;

    fn drive_sine(
        root: &mut JaMagnetizingRoot,
        amp: Wave,
        f0: Wave,
        fs: Wave,
        periods: usize,
    ) -> (Vec<Wave>, Vec<Wave>, Vec<Wave>) {
        let per = (fs / f0) as usize;
        let n = per * periods;
        let mut hs = Vec::with_capacity(n);
        let mut ms = Vec::with_capacity(n);
        let mut bs = Vec::with_capacity(n);
        for k in 0..n {
            let a = amp * math::sin(2.0 * PI * f0 * k as Wave / fs);
            let b = root.process(a);
            hs.push(root.state().h);
            ms.push(root.state().m);
            bs.push(b);
        }
        (hs, ms, bs)
    }

    #[test]
    fn ja_jacobian_matches_finite_differences() {
        let p = JaCoreModel::si_steel_demo();
        let (h0, m0, hd0) = (300.0, 2.0e5, 1.0e6);
        let e = mdot_eval(&p, h0, m0, hd0);
        let eps = 1.0e-3;
        let fd_dm = (mdot_eval(&p, h0, m0 + eps, hd0).value
            - mdot_eval(&p, h0, m0 - eps, hd0).value)
            / (2.0 * eps);
        let fd_dh = (mdot_eval(&p, h0 + eps, m0, hd0).value
            - mdot_eval(&p, h0 - eps, m0, hd0).value)
            / (2.0 * eps);
        let fd_dhd =
            (mdot_eval(&p, h0, m0, hd0 + 1.0).value - mdot_eval(&p, h0, m0, hd0 - 1.0).value) / 2.0;
        let tol = |a: Wave, b: Wave| math::abs(a - b) <= 1.0e-4 * a.abs().max(b.abs()).max(1.0e-12);
        assert!(tol(e.d_dm, fd_dm), "dMdot/dM: {} vs {}", e.d_dm, fd_dm);
        assert!(tol(e.d_dh, fd_dh), "dMdot/dH: {} vs {}", e.d_dh, fd_dh);
        assert!(
            tol(e.d_dhdot, fd_dhd),
            "dMdot/dHdot: {} vs {}",
            e.d_dhdot,
            fd_dhd
        );
    }

    #[test]
    fn ja_sine_drive_is_finite_and_hysteretic() {
        let mut root = JaMagnetizingRoot::new(JaCoreModel::si_steel_demo());
        let fs = 48_000.0;
        root.prepare(fs, 1000.0);
        let (hs, ms, bs) = drive_sine(&mut root, 40.0, 50.0, fs, 6);
        assert!(bs.iter().all(|x| x.is_finite()));
        let per = (fs / 50.0) as usize;
        let mut min_rem = Wave::MAX;
        for k in 2 * per..hs.len() - 1 {
            if hs[k].signum() != hs[k + 1].signum() {
                min_rem = min_rem.min(ms[k].abs());
            }
        }
        assert!(min_rem > 1.0e-3 * JaCoreModel::si_steel_demo().ms);
    }

    #[test]
    fn ja_deep_saturation_dissipates() {
        let mut root = JaMagnetizingRoot::new(JaCoreModel::si_steel_demo());
        let (fs, rp, f0) = (48_000.0, 1000.0, 50.0);
        root.prepare(fs, rp);
        let per = (fs / f0) as usize;
        let n = per * 10;
        let mut e_cycle = 0.0;
        let mut peak_m: Wave = 0.0;
        for k in 0..n {
            let a = 400.0 * math::sin(2.0 * PI * f0 * k as Wave / fs);
            let b = root.process(a);
            assert!(b.is_finite());
            peak_m = peak_m.max(root.state().m.abs());
            if k >= n - per {
                let v = 0.5 * (a + b);
                let i = (a - b) / (2.0 * rp);
                e_cycle += v * i / fs;
            }
        }
        assert!(peak_m / root.model().ms > 0.5, "peak_m={peak_m}");
        assert!(e_cycle > 0.0, "core generated energy: {e_cycle}");
    }

    #[test]
    fn ja_dc_bias_initializes_gapped_core_state() {
        let mut model = JaCoreModel::si_steel_demo();
        model.gap = 1.0e-3;
        let mut root = JaMagnetizingRoot::new(model);
        root.prepare(48_000.0, 1000.0);
        root.set_dc_bias_current(45.0e-3);

        let st = root.state();
        assert!(st.h.is_finite());
        assert!(st.m.is_finite());
        assert!(st.h.abs() > 1.0, "DC bias should produce standing H");
        assert!(st.m.abs() > 1.0, "DC bias should produce standing M");
        assert_eq!(st.h_dot, 0.0);
        assert_eq!(st.m_dot, 0.0);

        let ampere_turns = model.n_turns * 45.0e-3;
        let reconstructed = st.h * (model.path_len + model.gap) + st.m * model.gap;
        assert!(
            (reconstructed - ampere_turns).abs() < 1.0e-6 * ampere_turns.abs().max(1.0),
            "Ampere mismatch: {reconstructed} vs {ampere_turns}"
        );
    }
}
