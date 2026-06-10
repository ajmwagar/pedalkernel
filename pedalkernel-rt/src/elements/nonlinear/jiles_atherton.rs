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
        let mut root = JaMagnetizingRoot::new(model);
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
    let den = 1.0 - p.alpha * r;
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
}
