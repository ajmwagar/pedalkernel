"""Numerical validation of the Jiles-Atherton WDF magnetizing-branch root.

Same equations and solver structure as the Rust mockup:

  State per sample: H, Hdot, M, Mdot
  Unknowns per sample: (H_n, M_n), solved by 2x2 Newton-Raphson.

  Field eq (gap folded in):   N*i = H*(le+g) + M*g
  Faraday:                    v = N*Ae*mu0*(Hdot + Mdot)
  Wave port:                  v = a - Rp*i,  b = a - 2*Rp*i
  Hdot via trapezoid diff:    Hdot = 2*fs*(H - H_prev) - Hdot_prev
  M update via trapezoid:     M = M_prev + T/2*(Mdot + Mdot_prev)
"""
import numpy as np

MU0 = 4e-7 * np.pi

# J-A params: ballpark silicon-steel audio iron
MS, A, ALPHA, K, C = 1.6e6, 1100.0, 1.6e-3, 400.0, 0.2
# geometry
N, AE, LE, GAP = 2000.0, 2.0e-4, 0.10, 0.0


def langevin(x):
    if abs(x) < 1e-4:
        return x / 3.0 - x**3 / 45.0
    return 1.0 / np.tanh(x) - 1.0 / x


def langevin_d(x):
    if abs(x) < 1e-4:
        return 1.0 / 3.0 - x * x / 15.0
    s = np.sinh(x)
    return 1.0 / (x * x) - 1.0 / (s * s)


def langevin_d2(x):
    if abs(x) < 1e-3:
        return -2.0 * x / 15.0
    s = np.sinh(x)
    return 2.0 * np.cosh(x) / (s ** 3) - 2.0 / (x ** 3)


def mdot_eval(h, m, h_dot):
    """Returns (Mdot, dMdot/dM, dMdot/dH (fixed Hdot), dMdot/dHdot)."""
    q = (h + ALPHA * m) / A
    lq, lp, lpp = langevin(q), langevin_d(q), langevin_d2(q)
    m_an = MS * lq
    dm = m_an - m
    delta = 1.0 if h_dot >= 0.0 else -1.0
    delta_m = 1.0 if dm * h_dot >= 0.0 else 0.0
    one_c = 1.0 - C

    d_irr = one_c * delta * K - ALPHA * dm
    eps = 1e-9 * max(K, 1.0)
    if abs(d_irr) < eps:
        d_irr = eps if d_irr >= 0.0 else -eps

    g = one_c * delta_m * dm / d_irr
    dg_du = one_c * one_c * delta_m * delta * K / (d_irr * d_irr)

    r_coef = C * MS / A
    r = r_coef * lp
    dr_dq = r_coef * lpp

    den = 1.0 - ALPHA * r
    dden_dq = -ALPHA * dr_dq

    chi = (g + r) / den
    value = chi * h_dot

    du_dh = MS * lp / A
    du_dm = ALPHA * MS * lp / A - 1.0
    dq_dh = 1.0 / A
    dq_dm = ALPHA / A
    num = g + r
    dnum_dh = dg_du * du_dh + dr_dq * dq_dh
    dnum_dm = dg_du * du_dm + dr_dq * dq_dm
    dchi_dh = (dnum_dh * den - num * dden_dq * dq_dh) / (den * den)
    dchi_dm = (dnum_dm * den - num * dden_dq * dq_dm) / (den * den)

    return value, dchi_dm * h_dot, dchi_dh * h_dot, chi


class JaRoot:
    def __init__(self, fs, rp):
        self.fs, self.t, self.rp = fs, 1.0 / fs, rp
        self.kf = N * AE * MU0
        self.le_eff = LE + GAP
        self.h = self.h_dot = self.m = self.m_dot = 0.0
        self.iters = []

    def process(self, a):
        fs, t, rp = self.fs, self.t, self.rp
        h, m = self.h, self.m
        it = 0
        for it in range(12):
            h_dot = 2.0 * fs * (h - self.h) - self.h_dot
            e = mdot_eval(h, m, h_dot)
            f1 = a - (rp / N) * (h * self.le_eff + m * GAP) - self.kf * (h_dot + e[0])
            f2 = m - self.m - 0.5 * t * (e[0] + self.m_dot)
            if abs(f1) < 1e-9 * (1.0 + abs(a)) and abs(f2) < 1e-9 * MS:
                break
            dmdot_dh_tot = e[2] + e[3] * 2.0 * fs
            j11 = -(rp / N) * self.le_eff - self.kf * (2.0 * fs + dmdot_dh_tot)
            j12 = -(rp / N) * GAP - self.kf * e[1]
            j21 = -0.5 * t * dmdot_dh_tot
            j22 = 1.0 - 0.5 * t * e[1]
            det = j11 * j22 - j12 * j21
            if abs(det) < 1e-30:
                break
            dh = (j22 * f1 - j12 * f2) / det
            dm = (-j21 * f1 + j11 * f2) / det
            step_max = 50.0 * A
            if abs(dh) > step_max:
                s = step_max / abs(dh)
                dh *= s
                dm *= s
            h -= dh
            m -= dm
        self.iters.append(it + 1)
        h_dot = 2.0 * fs * (h - self.h) - self.h_dot
        e = mdot_eval(h, m, h_dot)
        i = (h * self.le_eff + m * GAP) / N
        b = a - 2.0 * rp * i
        self.h, self.h_dot, self.m, self.m_dot = h, h_dot, m, e[0]
        return b


def run():
    fs, rp = 48000.0, 1000.0
    root = JaRoot(fs, rp)
    f0 = 50.0
    per = int(fs / f0)
    n_samp = per * 6
    drive = 40.0
    tax = np.arange(n_samp) / fs
    a_sig = drive * np.sin(2 * np.pi * f0 * tax)

    hs, ms, bs = [], [], []
    for a in a_sig:
        b = root.process(a)
        hs.append(root.h)
        ms.append(root.m)
        bs.append(b)
    hs, ms, bs = map(np.array, (hs, ms, bs))

    print(f"finite: {np.all(np.isfinite(bs))}")
    print(f"mean NR iters: {np.mean(root.iters):.2f}  max: {max(root.iters)}")
    print(f"H range: [{hs.min():.1f}, {hs.max():.1f}] A/m")
    print(f"M range: [{ms.min():.3e}, {ms.max():.3e}] A/m  (Ms={MS:.1e})")
    print(f"M/Ms peak: {max(abs(ms.min()), ms.max())/MS:.3f}")

    h_late, m_late = hs[2 * per:], ms[2 * per:]
    crossings = np.where(np.diff(np.sign(h_late)) != 0)[0]
    rem = np.abs(m_late[crossings])
    print(f"remanence |M| at H=0 crossings: {rem.min():.3e} .. {rem.max():.3e}")
    assert rem.min() > 1e-3 * MS, "no hysteresis - loop is degenerate"

    for _ in range(4000):
        root.process(0.0)
    print(f"M after drive removed: {root.m:.3e} (remanent)")
    assert abs(root.m) > 1e-4 * MS
    assert np.isfinite(root.m)

    p3 = ms[3 * per:4 * per].max()
    p5 = ms[5 * per:6 * per].max()
    print(f"peak M period 3 vs 5: {p3:.4e} vs {p5:.4e}  (rel diff {abs(p3-p5)/max(p3,1e-12):.2e})")

    # Jacobian spot check vs finite differences
    h0, m0, hd0 = 300.0, 2.0e5, 1.0e6
    v0, ddm, ddh, ddhd = mdot_eval(h0, m0, hd0)
    epsd = 1e-3
    fd_dm = (mdot_eval(h0, m0 + epsd, hd0)[0] - mdot_eval(h0, m0 - epsd, hd0)[0]) / (2 * epsd)
    fd_dh = (mdot_eval(h0 + epsd, m0, hd0)[0] - mdot_eval(h0 - epsd, m0, hd0)[0]) / (2 * epsd)
    fd_dhd = (mdot_eval(h0, m0, hd0 + 1.0)[0] - mdot_eval(h0, m0, hd0 - 1.0)[0]) / 2.0
    print(f"dMdot/dM   analytic {ddm:.6e}  fd {fd_dm:.6e}")
    print(f"dMdot/dH   analytic {ddh:.6e}  fd {fd_dh:.6e}")
    print(f"dMdot/dHd  analytic {ddhd:.6e}  fd {fd_dhd:.6e}")
    for an, fd in ((ddm, fd_dm), (ddh, fd_dh), (ddhd, fd_dhd)):
        assert abs(an - fd) <= 1e-4 * max(abs(an), abs(fd), 1e-12), "Jacobian mismatch"

    print("ALL CHECKS PASSED")


if __name__ == "__main__":
    run()
