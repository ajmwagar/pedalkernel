//! Globalized damped-Newton tests for the grouped NR solver (pedalkernel-577s).
//!
//! These tests encode the multi-BJT divergence bug fixed by the Armijo line
//! search + SPICE-style junction limiting (`pnjlim`) in
//! `multi_port_nr_solve_grouped_into`.
//!
//! Coverage:
//!   * `stiff_two_junction_converges` — a synthetic 2-port exponential-junction
//!     system whose full Newton step overshoots wildly. THIS is the pure-solver
//!     fail-old / pass-new encoder: it DIVERGES (residual ~1e34) on the old
//!     crude per-component damping and CONVERGES on the new globalized step.
//!     Fast CI unit test.
//!   * `stiff_two_junction_monotone_residual` — the line search produces a
//!     monotone non-increasing residual across accepted iterations (this
//!     property is violated — residual blows up — on the old damping).
//!   * `coupled_two_bjt_converges` — TWO cross-coupled 2N3904 `BjtTwoPort`
//!     groups (n_nl = 4), the coupled-junction reproducer at the solver level.
//!     Converges to a small residual well under the cap with no rail-pinning.
//!     NOTE: the `BjtTwoPort` model carries internal Vbe/Vbc clamps that mask
//!     pure-solver divergence, so the coupled-BJT *circuit*-level fail-old /
//!     pass-new signature lives in the `xref_twostage_nofb` / `xref_feedback_amp`
//!     validate reproducers (which exercise the full WDF scattering embedding).
//!     This test is a convergence + no-rail-pin GUARD for the multi-BJT path.
//!   * `single_bjt_still_converges` — single-BJT path converges in a few iters
//!     (no slowdown / behavior change).

use pedalkernel_rt::elements::nonlinear::solver::{
    multi_port_nr_solve_grouped_into, NlDeviceGroupIv, NrWorkspace,
};
use pedalkernel_rt::elements::nonlinear::{
    disable_grouped_residual_log, enable_grouped_residual_log, BjtTwoPort, GummelPoonModel,
};

type W = f64;

/// A 2N3904 Gummel-Poon model (standard Fairchild SPICE params), built inline
/// because the model registry / `by_name` lookup lives in the `pedalkernel`
/// crate which `pedalkernel-rt` cannot depend on. Matches `bjt_from_spice` with
/// the canonical 2N3904 `.MODEL` card.
fn model_2n3904() -> GummelPoonModel {
    GummelPoonModel {
        is: 6.734e-15,
        bf: 416.4,
        br: 0.7371,
        nf: 1.0,
        nr: 1.0,
        vt: 0.02585,
        vaf: 74.03,
        var: W::INFINITY,
        ikf: 66.78e-3,
        ikr: W::INFINITY,
        ise: 6.734e-15,
        ne: 1.259,
        isc: 0.0,
        nc: 2.0,
        cje: 4.493e-12,
        vje: 0.75,
        mje: 0.2593,
        cjc: 3.638e-12,
        vjc: 0.75,
        mjc: 0.3085,
        rb: 10.0,
        re: 0.0,
        rc: 1.0,
        tf: 301.2e-12,
        tr: 239.5e-9,
        is_pnp: false,
    }
}

const NR_CAP: usize = 24;
const TOL: W = 1e-9;

/// A single exponential pn-junction port: `i = Is*(exp(v/Vt) - 1)`.
/// Two of these are placed in one group with NO internal coupling, but the
/// solver couples them through the scattering matrix `s_nl`. With a tiny `Is`
/// the conductance grows by `exp` so an unguarded Newton step overshoots by
/// many volts — the exact stiffness that broke the old damping.
struct TwoJunctionGroup {
    is: W,
    vt: W,
}

impl NlDeviceGroupIv for TwoJunctionGroup {
    fn n_ports(&self) -> usize {
        2
    }

    fn eval(&self, v: &[W], currents: &mut [W], jacobian: &mut [W]) {
        for p in 0..2 {
            let x = (v[p] / self.vt).clamp(-500.0, 500.0);
            let ev = x.exp();
            currents[p] = self.is * (ev - 1.0);
        }
        // Diagonal Jacobian (ports independent inside the group).
        let g = |vp: W| {
            let x = (vp / self.vt).clamp(-500.0, 500.0);
            self.is * x.exp() / self.vt
        };
        jacobian[0] = g(v[0]);
        jacobian[1] = 0.0;
        jacobian[2] = 0.0;
        jacobian[3] = g(v[1]);
    }

    fn v_clamp_port(&self, _port: usize) -> (W, W) {
        // Generous bounds: convergence must come from the line search, NOT from
        // the clamp pinning the iterate at a rail.
        (-5.0, 5.0)
    }

    fn limit_port_voltage(&self, _port: usize, v_new: W, v_old: W) -> W {
        // Classic SPICE pnjlim so the synthetic test exercises the same path.
        let vt = self.vt;
        let is = self.is.max(1e-30);
        let vcrit = vt * (vt / (std::f64::consts::SQRT_2 * is)).ln();
        if v_new > vcrit && (v_new - v_old).abs() > 2.0 * vt {
            if v_old > 0.0 {
                let arg = 1.0 + (v_new - v_old) / vt;
                if arg > 0.0 {
                    return v_old + vt * arg.ln();
                }
                return vcrit;
            }
            return vt * (v_new / vt).max(1e-30).ln();
        }
        v_new
    }
}

/// Inf-norm residual `F(v)` of the embedded WDF root system for verification:
/// `F_i = known_a_i - v_i - R_i*i_i + sum_j s_ij*(v_j - R_j*i_j)`.
fn residual_norm(
    group: &dyn NlDeviceGroupIv,
    n: usize,
    s_nl: &[W],
    known_a: &[W],
    rp: &[W],
    v: &[W],
) -> W {
    let mut currents = vec![0.0; n];
    let mut jac = vec![0.0; n * n];
    group.eval(v, &mut currents, &mut jac);
    let mut maxf = 0.0f64;
    for i in 0..n {
        let mut fi = known_a[i] - v[i] - rp[i] * currents[i];
        for j in 0..n {
            fi += s_nl[i * n + j] * (v[j] - rp[j] * currents[j]);
        }
        maxf = maxf.max(fi.abs());
    }
    maxf
}

#[test]
fn stiff_two_junction_converges() {
    let group = TwoJunctionGroup {
        is: 1e-14,
        vt: 0.02585,
    };
    let groups: [&dyn NlDeviceGroupIv; 1] = [&group];
    let offsets = [0usize];

    // Strong cross-coupling + drive that forces both junctions well into
    // forward conduction. The naive full Newton step from a cold seed jumps
    // many volts past turn-on (exp overshoot) — the old damping ping-ponged.
    let s_nl = [0.0, 0.6, 0.6, 0.0];
    let known_a = [1.2, 1.4];
    let rp = [220.0, 330.0];

    // Cold seed (reverse-biased) — the hard case.
    let mut v = [-0.5, -0.5];
    let mut ws = NrWorkspace::new_grouped(2, 2);
    multi_port_nr_solve_grouped_into(
        2, &s_nl, &known_a, &rp, &groups, &offsets, &mut v, NR_CAP, TOL, &mut ws,
    );

    let r = residual_norm(&group, 2, &s_nl, &known_a, &rp, &v);
    assert!(
        r < 1e-6,
        "stiff 2-junction did not converge: residual={r:.3e}, v={v:?}, iters={}",
        ws.iters_used
    );
    assert!(
        ws.iters_used < NR_CAP,
        "converged only by hitting the iteration cap ({} iters)",
        ws.iters_used
    );
    // Not pinned at a rail.
    for &vi in &v {
        assert!(
            vi.abs() < 4.99,
            "iterate pinned at the clamp rail (v={v:?}) — line search failed"
        );
    }
}

#[test]
fn stiff_two_junction_monotone_residual() {
    let group = TwoJunctionGroup {
        is: 1e-14,
        vt: 0.02585,
    };
    let groups: [&dyn NlDeviceGroupIv; 1] = [&group];
    let offsets = [0usize];
    let s_nl = [0.0, 0.6, 0.6, 0.0];
    let known_a = [1.2, 1.4];
    let rp = [220.0, 330.0];
    let mut v = [-0.5, -0.5];
    let mut ws = NrWorkspace::new_grouped(2, 2);

    enable_grouped_residual_log(64);
    multi_port_nr_solve_grouped_into(
        2, &s_nl, &known_a, &rp, &groups, &offsets, &mut v, NR_CAP, TOL, &mut ws,
    );
    let residuals = disable_grouped_residual_log();

    assert!(
        residuals.len() >= 2,
        "expected multiple iterations to check monotonicity, got {residuals:?}"
    );
    // Armijo guarantees a strictly-decreasing residual across accepted steps
    // (allow a tiny epsilon for fp noise).
    for w in residuals.windows(2) {
        assert!(
            w[1] <= w[0] + 1e-12,
            "residual increased across an accepted Newton step: {:?}",
            residuals
        );
    }
}

/// Build two cross-coupled 2N3904 BJTs as one grouped-NR system (n_nl = 4:
/// [Vbe1, Vce1, Vbe2, Vce2]). This mirrors the stiff coupled-junction
/// reproducer (`xref_twostage_nofb` / `xref_feedback_amp`) at the solver level.
#[test]
fn coupled_two_bjt_converges() {
    let model = model_2n3904();
    let q1 = BjtTwoPort::new(model);
    let q2 = BjtTwoPort::new(model);
    let groups: [&dyn NlDeviceGroupIv; 2] = [&q1, &q2];
    let offsets = [0usize, 2usize];
    let n = 4;

    // A stiff coupled operating point: strong inter-stage + feedback coupling
    // through the scattering matrix (collector of stage 1 drives base of stage
    // 2, and a feedback term back to stage 1's base). The large drive pushes the
    // junctions hard into conduction so the UNDAMPED Newton step from the cold
    // seed overshoots by many volts — exactly the coupled-exponential-junction
    // stiffness that made the old per-component damping diverge/rail.
    #[rustfmt::skip]
    let s_nl = [
        0.00, 0.20, 0.30, 0.00,
        0.20, 0.00, 0.00, 0.30,
        0.40, 0.00, 0.00, 0.20,
        0.00, 0.20, 0.30, 0.00,
    ];
    // known_a drives the Vbe ports toward turn-on and the Vce ports to mid-rail.
    let known_a = [1.2, 8.0, 1.2, 8.0];
    let rp = [1_000.0, 4_700.0, 1_000.0, 4_700.0];

    // Cold seed — every junction reverse/zero biased. This is precisely the
    // condition under which the old per-component 5 V cap + hard clamp diverged
    // (residual ~4, 24-iter cap, railed Vbe/Vce).
    let mut v = [0.0, 0.0, 0.0, 0.0];
    let mut ws = NrWorkspace::new_grouped(n, 2);

    multi_port_nr_solve_grouped_into(
        n, &s_nl, &known_a, &rp, &groups, &offsets, &mut v, NR_CAP, TOL, &mut ws,
    );

    let r = bjt_residual_norm(&groups, &offsets, n, &s_nl, &known_a, &rp, &v);
    assert!(
        r < 1e-5,
        "coupled 2-BJT system did not converge (the multi-BJT divergence bug): \
         residual={r:.3e}, iters={}, v={v:?}",
        ws.iters_used
    );
    assert!(
        ws.iters_used < NR_CAP,
        "coupled 2-BJT converged only at the iteration cap ({} iters) — \
         divergence not fixed",
        ws.iters_used
    );
    // No rail-pinning: the old bug left every iterate hard against a clamp rail
    // with a large residual (ping-pong). A converged solution must sit strictly
    // inside the clamp bounds on every port.
    for p in 0..n {
        let (g, lp) = (if p < 2 { 0 } else { 1 }, p % 2);
        let dev = if g == 0 { &q1 } else { &q2 };
        let (lo, hi) = dev.v_clamp_port(lp);
        assert!(
            v[p] > lo + 1e-3 && v[p] < hi - 1e-3,
            "port {p} pinned at clamp rail ({}), bounds=({lo},{hi})",
            v[p]
        );
    }
}

/// Single-BJT regression: the grouped solver with ONE 2N3904 group (n_nl = 2)
/// converges quickly (the mild case that always worked) — assert no slowdown.
#[test]
fn single_bjt_still_converges() {
    let model = model_2n3904();
    let q = BjtTwoPort::new(model);
    let groups: [&dyn NlDeviceGroupIv; 1] = [&q];
    let offsets = [0usize];
    let n = 2;

    let s_nl = [0.0, 0.1, 0.2, 0.0];
    let known_a = [0.7, 6.0];
    let rp = [2_200.0, 4_700.0];
    let mut ws = NrWorkspace::new_grouped(n, 2);

    // Cold-seed solve (DC startup): must converge well under the cap.
    let mut v = [0.0, 0.0];
    multi_port_nr_solve_grouped_into(
        n, &s_nl, &known_a, &rp, &groups, &offsets, &mut v, NR_CAP, TOL, &mut ws,
    );
    let r = bjt_residual_norm(&groups, &offsets, n, &s_nl, &known_a, &rp, &v);
    assert!(
        r < 1e-5,
        "single-BJT cold-seed did not converge: residual={r:.3e}, iters={}",
        ws.iters_used
    );
    assert!(
        ws.iters_used < NR_CAP,
        "single-BJT cold-seed hit the iteration cap ({} iters)",
        ws.iters_used
    );
    let v_solution = v;

    // Warm-seed solve (the realistic per-sample reseed: the previous sample's
    // solution seeds the next). This is the path that must stay fast (1-3 iters)
    // — assert the line search adds no slowdown to the mild single-BJT case.
    // Use a fresh workspace so the frozen-Newton cache does not mask iters.
    let mut ws_warm = NrWorkspace::new_grouped(n, 2);
    let mut v_warm = v_solution;
    multi_port_nr_solve_grouped_into(
        n,
        &s_nl,
        &known_a,
        &rp,
        &groups,
        &offsets,
        &mut v_warm,
        NR_CAP,
        TOL,
        &mut ws_warm,
    );
    let r_warm = bjt_residual_norm(&groups, &offsets, n, &s_nl, &known_a, &rp, &v_warm);
    assert!(r_warm < 1e-5, "single-BJT warm-seed residual={r_warm:.3e}");
    assert!(
        ws_warm.iters_used <= 3,
        "single-BJT warm-seed now takes {} iters (expected 1-3) — \
         line search slowed the mild path",
        ws_warm.iters_used
    );
}

/// Residual norm for a multi-group BJT system.
fn bjt_residual_norm(
    groups: &[&dyn NlDeviceGroupIv],
    offsets: &[usize],
    n: usize,
    s_nl: &[W],
    known_a: &[W],
    rp: &[W],
    v: &[W],
) -> W {
    let mut currents = vec![0.0; n];
    for (g, group) in groups.iter().enumerate() {
        let np = group.n_ports();
        let off = offsets[g];
        let mut c = vec![0.0; np];
        let mut j = vec![0.0; np * np];
        group.eval(&v[off..off + np], &mut c, &mut j);
        currents[off..off + np].copy_from_slice(&c);
    }
    let mut maxf = 0.0f64;
    for i in 0..n {
        let mut fi = known_a[i] - v[i] - rp[i] * currents[i];
        for j in 0..n {
            fi += s_nl[i * n + j] * (v[j] - rp[j] * currents[j]);
        }
        maxf = maxf.max(fi.abs());
    }
    maxf
}
