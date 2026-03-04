//! Shared Newton-Raphson solver and numerical utilities for nonlinear WDF roots.

use std::cell::RefCell;

const SOLVER_STATS_ENABLED: bool = true;

thread_local! {
    static SOLVER_STATS: RefCell<SolverStatsAggregate> = RefCell::new(SolverStatsAggregate::default());
}

#[derive(Default, Clone, Debug)]
struct SolverStatsAggregate {
    solves: u64,
    total_iterations: u64,
    max_iterations: u32,
    max_residual: f64,
    clamp_hits: u64,
    step_limited: u64,
    iter_cap_hits: u64,
}

impl SolverStatsAggregate {
    fn record(&mut self, entry: SolverStatsEntry) {
        self.solves += 1;
        self.total_iterations += entry.iterations as u64;
        self.max_iterations = self.max_iterations.max(entry.iterations);
        self.max_residual = self.max_residual.max(entry.residual);
        if entry.clamp_hit {
            self.clamp_hits += 1;
        }
        if entry.step_limited {
            self.step_limited += 1;
        }
        if entry.iter_cap_hit {
            self.iter_cap_hits += 1;
        }
    }
}

#[derive(Default, Clone, Debug, PartialEq)]
pub struct SolverStatsSnapshot {
    pub solves: u64,
    pub total_iterations: u64,
    pub max_iterations: u32,
    pub max_residual: f64,
    pub clamp_hits: u64,
    pub step_limited: u64,
    pub iter_cap_hits: u64,
}

#[derive(Default, Clone, Copy)]
struct SolverStatsEntry {
    iterations: u32,
    residual: f64,
    clamp_hit: bool,
    step_limited: bool,
    iter_cap_hit: bool,
}

fn record_solver_stats(entry: SolverStatsEntry) {
    if SOLVER_STATS_ENABLED {
        SOLVER_STATS.with(|stats| stats.borrow_mut().record(entry));
    }
}

pub fn reset_solver_stats() {
    if SOLVER_STATS_ENABLED {
        SOLVER_STATS.with(|stats| *stats.borrow_mut() = SolverStatsAggregate::default());
    }
}

pub fn solver_stats_snapshot() -> SolverStatsSnapshot {
    if SOLVER_STATS_ENABLED {
        SOLVER_STATS.with(|stats| {
            let agg = stats.borrow();
            SolverStatsSnapshot {
                solves: agg.solves,
                total_iterations: agg.total_iterations,
                max_iterations: agg.max_iterations,
                max_residual: agg.max_residual,
                clamp_hits: agg.clamp_hits,
                step_limited: agg.step_limited,
                iter_cap_hits: agg.iter_cap_hits,
            }
        })
    } else {
        SolverStatsSnapshot::default()
    }
}

/// Numerically stable softplus: `ln(1 + exp(x))`.
///
/// For |x| > 50, uses asymptotic approximation:
///   - x > 50: `ln(1 + exp(x)) ≈ x` (error < 1e-22)
///   - x < -50: `ln(1 + exp(x)) ≈ exp(x) ≈ 0` (return 0)
///
/// The transition at |x| = 50 is mathematically smooth to within f64 precision.
/// This is the standard numerically-stable implementation used throughout
/// the Koren triode/pentode equations.
#[inline]
pub(crate) fn softplus(x: f64) -> f64 {
    if x > 50.0 {
        x
    } else if x < -50.0 {
        0.0
    } else {
        (1.0 + x.exp()).ln()
    }
}

/// Minimum conductance returned by nonlinear element derivative functions
/// when a device is in cutoff or reverse-bias.
///
/// This is physically motivated: even in cutoff, real semiconductors have
/// finite leakage current due to minority carriers and surface effects.
/// For silicon: Icbo ≈ 1–100 nA → Gleakage ≈ 1e-12 to 1e-9 S.
/// Using 1e-12 S (1 TΩ) is conservative and prevents Newton-Raphson
/// from dividing by zero without adding measurable phantom current.
pub(crate) const LEAKAGE_CONDUCTANCE: f64 = 1e-12;

// ---------------------------------------------------------------------------
// NlDeviceIv trait for multi-port NR solver
// ---------------------------------------------------------------------------

/// Trait for nonlinear WDF devices that provide their I-V characteristic.
///
/// Used by the multi-port Newton-Raphson solver to evaluate the device
/// current and its derivative at a given port voltage.
pub(crate) trait NlDeviceIv {
    /// Return `(current, d_current/d_voltage)` at the given port voltage.
    fn iv(&self, v: f64) -> (f64, f64);

    /// Return `(v_min, v_max)` voltage clamping bounds for this device.
    fn v_clamp(&self) -> (f64, f64);
}

// ---------------------------------------------------------------------------
// NlDeviceGroupIv trait for multi-port NL devices with cross-coupled I-V
// ---------------------------------------------------------------------------

/// Trait for multi-port nonlinear WDF devices with internally coupled I-V.
///
/// Used for 3-terminal devices (triodes, FETs) where the current at one port
/// depends on the voltage at another port of the same device. For example,
/// a triode's plate current depends on both plate-cathode and grid-cathode
/// voltages.
///
/// Each device group owns N ports. The grouped NR solver uses the full
/// N×N device Jacobian (including cross-derivatives) when building the
/// system Jacobian.
pub(crate) trait NlDeviceGroupIv {
    /// Number of ports in this device group.
    fn n_ports(&self) -> usize;

    /// Evaluate currents and full Jacobian for all ports.
    ///
    /// Given port voltages `v[0..n]`, writes:
    /// - `currents[i]` = current at port i
    /// - `jacobian[i * n + j]` = ∂i_i/∂v_j  (row-major, n×n)
    ///
    /// Buffers are pre-allocated by the caller (length `n` and `n*n`).
    fn eval(&self, v: &[f64], currents: &mut [f64], jacobian: &mut [f64]);

    /// Voltage clamp bounds for a specific port.
    fn v_clamp_port(&self, port: usize) -> (f64, f64);
}

// ---------------------------------------------------------------------------
// Single-port group adapter (wraps NlDeviceIv as 1-port NlDeviceGroupIv)
// ---------------------------------------------------------------------------

/// Adapts a single-port `NlDeviceIv` into a 1-port `NlDeviceGroupIv`.
#[allow(dead_code)]
///
/// This allows mixing independent NL devices (diodes, pentodes) with
/// cross-coupled device groups (triodes) in the same grouped NR solver.
/// The adapter wraps the single-port I-V into the grouped interface with
/// a trivial 1×1 Jacobian.
pub(crate) struct SinglePortGroupAdapter<'a>(pub &'a dyn NlDeviceIv);

impl NlDeviceGroupIv for SinglePortGroupAdapter<'_> {
    fn n_ports(&self) -> usize {
        1
    }

    fn eval(&self, v: &[f64], currents: &mut [f64], jacobian: &mut [f64]) {
        let (i, di) = self.0.iv(v[0]);
        currents[0] = i;
        jacobian[0] = di;
    }

    fn v_clamp_port(&self, _port: usize) -> (f64, f64) {
        self.0.v_clamp()
    }
}

// ---------------------------------------------------------------------------
// Small linear system solver
// ---------------------------------------------------------------------------

/// Solve a small N×N linear system `A·x = b` in-place.
///
/// For N=1: trivial division.
/// For N=2: Cramer's rule (no pivoting needed for well-conditioned 2×2).
/// For N≥3: Gaussian elimination with partial pivoting.
///
/// Returns `false` if the system is singular (near-zero determinant/pivot).
pub(crate) fn solve_small_linear(n: usize, a: &mut [f64], b: &mut [f64]) -> bool {
    debug_assert_eq!(a.len(), n * n);
    debug_assert_eq!(b.len(), n);

    if n == 1 {
        if a[0].abs() < 1e-30 {
            return false;
        }
        b[0] /= a[0];
        return true;
    }

    if n == 2 {
        // Cramer's rule for 2×2
        let det = a[0] * a[3] - a[1] * a[2];
        if det.abs() < 1e-30 {
            return false;
        }
        let inv_det = 1.0 / det;
        let x0 = (a[3] * b[0] - a[1] * b[1]) * inv_det;
        let x1 = (a[0] * b[1] - a[2] * b[0]) * inv_det;
        b[0] = x0;
        b[1] = x1;
        return true;
    }

    // Gaussian elimination with partial pivoting for N≥3
    for col in 0..n {
        // Find pivot
        let mut max_row = col;
        let mut max_val = a[col * n + col].abs();
        for row in (col + 1)..n {
            let val = a[row * n + col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }

        if max_val < 1e-30 {
            return false; // Singular
        }

        // Swap rows
        if max_row != col {
            for j in 0..n {
                a.swap(col * n + j, max_row * n + j);
            }
            b.swap(col, max_row);
        }

        // Eliminate below
        let pivot = a[col * n + col];
        for row in (col + 1)..n {
            let factor = a[row * n + col] / pivot;
            for j in (col + 1)..n {
                a[row * n + j] -= factor * a[col * n + j];
            }
            a[row * n + col] = 0.0;
            b[row] -= factor * b[col];
        }
    }

    // Back-substitution
    for i in (0..n).rev() {
        let mut sum = b[i];
        for j in (i + 1)..n {
            sum -= a[i * n + j] * b[j];
        }
        if a[i * n + i].abs() < 1e-30 {
            return false;
        }
        b[i] = sum / a[i * n + i];
    }

    true
}

// ---------------------------------------------------------------------------
// Multi-port Newton-Raphson WDF Solver
// ---------------------------------------------------------------------------

/// Multi-port Newton-Raphson solver for coupled nonlinear WDF elements.
///
/// For N nonlinear ports on an R-type adaptor, the system of equations is:
///
/// ```text
/// F_i(v) = known_a_i + Σ_j S_nl[i][j]·(-2·R_j·i_j(v_j)) - 2·v_i = 0
/// ```
///
/// where `known_a[i]` includes contributions from all passive/adapted ports,
/// and `S_nl` is the NL-to-NL sub-block of the scattering matrix.
///
/// The Jacobian is:
/// ```text
/// J[i][j] = S_nl[i][j]·(-2·R_j)·(di_j/dv_j)           for j ≠ i
/// J[i][i] = S_nl[i][i]·(-2·R_i)·(di_i/dv_i) - 2        for j = i
/// ```
///
/// # Parameters
/// - `n_nl`: Number of coupled nonlinear ports
/// - `s_nl`: NL-to-NL scattering sub-block (n_nl × n_nl, row-major)
/// - `known_a`: Pre-computed incident wave contributions from passive ports (n_nl)
/// - `port_resistances`: Port resistance for each NL port (n_nl)
/// - `devices`: NL device I-V evaluators (n_nl)
/// - `v_guess`: Warm-start voltages, mutated to final solution (n_nl)
/// - `max_iter`: Maximum Newton-Raphson iterations
/// - `tolerance`: Convergence threshold for |dv|
///
/// # Returns
/// Reflected wave `b[i]` for each NL port.
pub(crate) fn multi_port_nr_solve(
    n_nl: usize,
    s_nl: &[f64],
    known_a: &[f64],
    port_resistances: &[f64],
    devices: &[&dyn NlDeviceIv],
    v_guess: &mut [f64],
    max_iter: usize,
    tolerance: f64,
) -> Vec<f64> {
    debug_assert_eq!(s_nl.len(), n_nl * n_nl);
    debug_assert_eq!(known_a.len(), n_nl);
    debug_assert_eq!(port_resistances.len(), n_nl);
    debug_assert_eq!(devices.len(), n_nl);
    debug_assert_eq!(v_guess.len(), n_nl);
    debug_assert!(
        port_resistances.iter().all(|&r| r > 0.0),
        "All port resistances must be positive, got: {:?}",
        port_resistances
    );

    #[cfg(feature = "fault-injection")]
    if crate::fault_injection::is_active(crate::fault_injection::Fault::BreakSolverWarmStart) {
        v_guess.iter_mut().for_each(|v| *v = 0.0);
    }

    // Working arrays
    let mut f_vec = vec![0.0; n_nl];
    let mut jacobian = vec![0.0; n_nl * n_nl];
    let mut currents = vec![0.0; n_nl];
    let mut derivatives = vec![0.0; n_nl];
    let mut stats_entry = if SOLVER_STATS_ENABLED {
        Some(SolverStatsEntry::default())
    } else {
        None
    };
    let mut clamp_hit = false;
    let mut step_limited = false;
    let mut last_residual = 0.0;

    for iter in 0..max_iter {
        if let Some(entry) = stats_entry.as_mut() {
            entry.iterations = iter as u32 + 1;
        }
        // Evaluate I-V for each device
        for i in 0..n_nl {
            let (clamp_lo, clamp_hi) = devices[i].v_clamp();
            let clamped = v_guess[i].clamp(clamp_lo, clamp_hi);
            if clamped != v_guess[i] {
                clamp_hit = true;
            }
            v_guess[i] = clamped;
            let (current, deriv) = devices[i].iv(v_guess[i]);
            currents[i] = current;
            derivatives[i] = deriv;
        }

        // Compute residual F_i and Jacobian J
        //
        // The reflected wave from NL port j is: b_j = 2·v_j - a_j
        //   where a_j is the port's incident wave
        // But the NL device also constrains: a_j - b_j = 2·R_j·i_j(v_j)
        //   => a_j = v_j + R_j·i_j(v_j) [Thévenin], b_j = v_j - R_j·i_j(v_j)
        //
        // The scattering relation for NL port i is:
        //   a_i = known_a_i + Σ_j S_nl[i][j] · b_j
        //       = known_a_i + Σ_j S_nl[i][j] · (2·v_j - a_j)
        //
        // Wait — in the scattering formulation, a = S·b, where b[j] is the
        // reflected wave from port j. For an NL device, b_j is what the device
        // "sends" and a_j is what it "receives". The constraint is:
        //   b_j = 2·v_j - a_j (voltage definition)
        //   a_j - 2·v_j = -b_j = -(2·v_j - a_j) ... circular
        //
        // Actually the NL device constraint is: b_j = a_j - 2·R_j·i_j(v_j)
        // And scattering: a_i = Σ_j S[i][j]·b_j (for NL ports only, with known_a
        //   absorbing the passive port contributions)
        //
        // Substituting b_j = a_j - 2·R_j·i_j:
        //   a_i = known_a_i + Σ_j S_nl[i][j]·(a_j - 2·R_j·i_j)
        //
        // And voltage: v_j = (a_j + b_j)/2 = (a_j + a_j - 2·R_j·i_j)/2 = a_j - R_j·i_j
        //   => a_j = v_j + R_j·i_j
        //   => b_j = v_j - R_j·i_j
        //
        // So: a_i = known_a_i + Σ_j S_nl[i][j]·(v_j - R_j·i_j)
        // And: a_i = v_i + R_i·i_i
        //
        // Therefore:
        //   v_i + R_i·i_i = known_a_i + Σ_j S_nl[i][j]·(v_j - R_j·i_j)
        //
        // F_i = known_a_i + Σ_j S_nl[i][j]·(v_j - R_j·i_j(v_j)) - v_i - R_i·i_i(v_i) = 0

        let mut max_f = 0.0_f64;
        for i in 0..n_nl {
            let mut fi = known_a[i] - v_guess[i] - port_resistances[i] * currents[i];
            for j in 0..n_nl {
                fi += s_nl[i * n_nl + j] * (v_guess[j] - port_resistances[j] * currents[j]);
            }
            f_vec[i] = fi;
            max_f = max_f.max(fi.abs());
        }
        last_residual = max_f;

        // Check convergence on residual
        if max_f < tolerance {
            break;
        }

        // Build Jacobian
        // dF_i/dv_j:
        //   j ≠ i: S_nl[i][j] · (1 - R_j·di_j/dv_j)
        //   j = i: S_nl[i][i] · (1 - R_i·di_i/dv_i) - 1 - R_i·di_i/dv_i
        //        = (S_nl[i][i] - 1) · (1 - R_i·di_i/dv_i) - (1 - 1) ...
        //   Let me redo: dF_i/dv_j = S_nl[i][j]·(1 - R_j·di_j/dv_j) + δ_ij·(-1 - R_i·di_i/dv_i)
        //   Wait: dF_i/dv_j for the non-scattering terms:
        //     d(-v_i - R_i·i_i)/dv_j = δ_ij · (-1 - R_i · di_i/dv_i)
        //   For the scattering terms:
        //     d(Σ_k S[i][k]·(v_k - R_k·i_k))/dv_j = S[i][j] · (1 - R_j·di_j/dv_j)
        //
        // So: J[i][j] = S_nl[i][j]·(1 - R_j·derivatives[j]) + δ_ij·(-1 - R_i·derivatives[i])

        for i in 0..n_nl {
            for j in 0..n_nl {
                let sij = s_nl[i * n_nl + j];
                let term = sij * (1.0 - port_resistances[j] * derivatives[j]);
                if i == j {
                    jacobian[i * n_nl + j] = term - 1.0 - port_resistances[i] * derivatives[i];
                } else {
                    jacobian[i * n_nl + j] = term;
                }
            }
        }

        // Solve J·dv = -F
        // (we negate f_vec in-place to get the RHS)
        let mut rhs = f_vec.clone();
        for x in rhs.iter_mut() {
            *x = -*x;
        }
        // Note: negate because we solve J·dv = -F, so dv = -J⁻¹·F
        // Actually the standard NR step is: J·Δv = -F, so Δv = -J⁻¹·F
        // But solve_small_linear solves A·x = b, so we pass -F as b.
        // Wait, we already negated rhs above. So solve J·dv = rhs where rhs = -F.
        // Actually let's just pass F as-is and negate: solve J·(-dv) = F
        // Hmm, let me be precise: NR update is v_new = v - J⁻¹·F
        // So we solve J·delta = F, then v -= delta.

        let mut rhs = f_vec.clone();
        let mut jac_copy = jacobian.clone();

        if !solve_small_linear(n_nl, &mut jac_copy, &mut rhs) {
            break;
        }

        // Bail out if the linear solve produced NaN (near-singular Jacobian)
        if rhs.iter().any(|v| !v.is_finite()) {
            break;
        }

        // Apply damped Newton step: v -= alpha * delta
        let mut max_dv = 0.0_f64;
        for i in 0..n_nl {
            let mut dv = rhs[i];
            #[cfg(feature = "fault-injection")]
            let skip_damp = crate::fault_injection::is_active(crate::fault_injection::Fault::DisableStepDamping);
            #[cfg(not(feature = "fault-injection"))]
            let skip_damp = false;
            if dv.abs() > 5.0 && !skip_damp {
                dv *= 0.5;
                step_limited = true;
            }
            v_guess[i] -= dv;
            let (lo, hi) = devices[i].v_clamp();
            let clamped = v_guess[i].clamp(lo, hi);
            if clamped != v_guess[i] {
                clamp_hit = true;
            }
            v_guess[i] = clamped;
            max_dv = max_dv.max(dv.abs());
        }

        if max_dv < tolerance {
            break;
        }
    }

    // Track iteration cap hits (solver ran all iterations without converging)
    let iter_cap_hit = last_residual >= tolerance;

    // Ensure v_guess is clean after solver exits
    for i in 0..n_nl {
        if !v_guess[i].is_finite() {
            let (lo, hi) = devices[i].v_clamp();
            v_guess[i] = (lo + hi) * 0.5;
        }
    }

    // Compute final reflected waves: b_i = v_i - R_i·i_i(v_i)
    // (This is the WDF reflected wave from each NL port)
    let mut b_nl = vec![0.0; n_nl];
    for i in 0..n_nl {
        let (current, _) = devices[i].iv(v_guess[i]);
        let b = v_guess[i] - port_resistances[i] * current;
        // TODO(perf): Same as grouped solver — remove once all circuits
        // compile with adapted NL port resistances from build.rs.
        b_nl[i] = b.clamp(-1000.0, 1000.0);
    }

    #[cfg(feature = "debug-trace")]
    {
        let r_max = port_resistances.iter().copied().fold(0.0_f64, f64::max);
        let r_min = port_resistances
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min);
        let cond = r_max / r_min;
        for i in 0..n_nl {
            let (current, _) = devices[i].iv(v_guess[i]);
            let a_i = v_guess[i] + port_resistances[i] * current;
            let a_scatter: f64 = known_a[i]
                + (0..n_nl)
                    .map(|j| s_nl[i * n_nl + j] * b_nl[j])
                    .sum::<f64>();
            let embed_err = (a_i - a_scatter).abs();
            if embed_err > 1e-4 || cond > 1e6 {
                eprintln!(
                    "[NR] port R cond={cond:.1e} embed_err[{i}]={embed_err:.2e} residual={last_residual:.2e}"
                );
            }
        }
    }

    if let Some(mut entry) = stats_entry {
        entry.residual = last_residual;
        entry.clamp_hit = clamp_hit;
        entry.step_limited = step_limited;
        entry.iter_cap_hit = iter_cap_hit;
        record_solver_stats(entry);
    }
    b_nl
}

// ---------------------------------------------------------------------------
// Multi-port NR solver with device groups (cross-coupled I-V)
// ---------------------------------------------------------------------------

/// Multi-port Newton-Raphson solver for devices with cross-coupled I-V.
///
/// Generalizes `multi_port_nr_solve()` for devices where port currents depend
/// on voltages at other ports of the same device. For example, a triode's
/// plate current depends on both Vpk and Vgk.
///
/// # Device groups
///
/// NL ports are organized into device groups. Each group is a multi-port NL
/// device (e.g., a 3-port triode with grid and plate ports). The device
/// provides a full Jacobian including cross-derivatives (∂i_plate/∂v_grid).
///
/// The global NL ports are laid out contiguously by group:
/// `[group0_port0, group0_port1, ..., group1_port0, ...]`
///
/// # Residual equation
///
/// Same as `multi_port_nr_solve()`:
/// ```text
/// F_i = known_a_i - v_i - R_i·i_i(v) + Σ_j S[i][j]·(v_j - R_j·i_j(v)) = 0
/// ```
///
/// The difference is that `i_j(v)` now depends on voltages of ALL ports in
/// device(j), not just v_j. The Jacobian includes cross-terms:
///
/// ```text
/// ∂F_i/∂v_k = S[i][k] - δ_{ik}
///           - R_i · ∂i_i/∂v_k                              (if k ∈ device(i))
///           - Σ_j S[i][j] · R_j · ∂i_j/∂v_k               (j ∈ device(k))
/// ```
pub(crate) fn multi_port_nr_solve_grouped(
    n_nl: usize,
    s_nl: &[f64],             // n_nl × n_nl scattering sub-block
    known_a: &[f64],          // n_nl
    port_resistances: &[f64], // n_nl
    device_groups: &[&dyn NlDeviceGroupIv],
    group_port_offsets: &[usize], // start index of each group's ports
    v_guess: &mut [f64],          // n_nl (warm-start, mutated)
    max_iter: usize,
    tolerance: f64,
) -> Vec<f64> {
    debug_assert_eq!(s_nl.len(), n_nl * n_nl);
    debug_assert_eq!(known_a.len(), n_nl);
    debug_assert_eq!(port_resistances.len(), n_nl);
    debug_assert_eq!(v_guess.len(), n_nl);
    debug_assert!(
        port_resistances.iter().all(|&r| r > 0.0),
        "All port resistances must be positive, got: {:?}",
        port_resistances
    );

    // Build port-to-group mapping
    let n_groups = device_groups.len();
    // port_group[i] = (group_index, local_port_index)
    let mut port_group = vec![(0usize, 0usize); n_nl];
    for g in 0..n_groups {
        let np = device_groups[g].n_ports();
        let offset = group_port_offsets[g];
        for lp in 0..np {
            port_group[offset + lp] = (g, lp);
        }
    }

    // Working arrays
    let mut f_vec = vec![0.0; n_nl];
    let mut sys_jacobian = vec![0.0; n_nl * n_nl];
    let mut currents = vec![0.0; n_nl];

    // Per-group temporary buffers for device evaluation
    let max_group_ports = device_groups.iter().map(|d| d.n_ports()).max().unwrap_or(1);
    let mut dev_currents = vec![0.0; max_group_ports];
    let mut dev_jacobian = vec![0.0; max_group_ports * max_group_ports];
    // Full n_nl × n_nl device Jacobian (sparse: only same-group entries non-zero)
    let mut full_dev_jac = vec![0.0; n_nl * n_nl];
    let mut stats_entry = if SOLVER_STATS_ENABLED {
        Some(SolverStatsEntry::default())
    } else {
        None
    };
    let mut clamp_hit = false;
    let mut step_limited = false;
    let mut last_residual = 0.0;

    for iter in 0..max_iter {
        if let Some(entry) = stats_entry.as_mut() {
            entry.iterations = iter as u32 + 1;
        }
        // Clamp voltages
        for i in 0..n_nl {
            let (g, lp) = port_group[i];
            let (lo, hi) = device_groups[g].v_clamp_port(lp);
            let clamped = v_guess[i].clamp(lo, hi);
            if clamped != v_guess[i] {
                clamp_hit = true;
            }
            v_guess[i] = clamped;
        }

        // Evaluate all device groups
        full_dev_jac.iter_mut().for_each(|x| *x = 0.0);
        for g in 0..n_groups {
            let np = device_groups[g].n_ports();
            let offset = group_port_offsets[g];

            // Extract voltages for this group
            let v_group = &v_guess[offset..offset + np];
            device_groups[g].eval(
                v_group,
                &mut dev_currents[..np],
                &mut dev_jacobian[..np * np],
            );

            // Copy to global arrays
            for lp in 0..np {
                currents[offset + lp] = dev_currents[lp];
                for lq in 0..np {
                    full_dev_jac[(offset + lp) * n_nl + (offset + lq)] = dev_jacobian[lp * np + lq];
                }
            }
        }

        // Compute residual F_i
        let mut max_f = 0.0_f64;
        for i in 0..n_nl {
            let mut fi = known_a[i] - v_guess[i] - port_resistances[i] * currents[i];
            for j in 0..n_nl {
                fi += s_nl[i * n_nl + j] * (v_guess[j] - port_resistances[j] * currents[j]);
            }
            f_vec[i] = fi;
            max_f = max_f.max(fi.abs());
        }
        last_residual = max_f;

        if max_f < tolerance {
            break;
        }

        // Build system Jacobian
        //
        // ∂F_i/∂v_k = S[i][k] - δ_{ik}
        //            - R_i · (∂i_i/∂v_k)                        [from -R_i·i_i term]
        //            - Σ_j S[i][j] · R_j · (∂i_j/∂v_k)         [from scattering term]
        //
        // Note: ∂i_j/∂v_k = full_dev_jac[j][k], non-zero only when j,k in same group.
        for i in 0..n_nl {
            for k in 0..n_nl {
                let mut val = s_nl[i * n_nl + k];
                if i == k {
                    val -= 1.0;
                }

                // -R_i · ∂i_i/∂v_k (non-zero if i,k in same device group)
                let dii_dvk = full_dev_jac[i * n_nl + k];
                val -= port_resistances[i] * dii_dvk;

                // -Σ_j S[i][j] · R_j · ∂i_j/∂v_k
                // Only iterate over j in the same group as k (where ∂i_j/∂v_k ≠ 0)
                let (gk, _) = port_group[k];
                let gk_offset = group_port_offsets[gk];
                let gk_np = device_groups[gk].n_ports();
                for lj in 0..gk_np {
                    let j = gk_offset + lj;
                    let dij_dvk = full_dev_jac[j * n_nl + k];
                    if dij_dvk != 0.0 {
                        val -= s_nl[i * n_nl + j] * port_resistances[j] * dij_dvk;
                    }
                }

                sys_jacobian[i * n_nl + k] = val;
            }
        }

        // Solve J·delta = F, then v -= delta
        let mut rhs = f_vec.clone();
        let mut jac_copy = sys_jacobian.clone();

        if !solve_small_linear(n_nl, &mut jac_copy, &mut rhs) {
            break; // Singular
        }

        // Bail out if the linear solve produced NaN (near-singular Jacobian)
        if rhs.iter().any(|v| !v.is_finite()) {
            break;
        }

        // Apply damped Newton step
        let mut max_dv = 0.0_f64;
        for i in 0..n_nl {
            let mut dv = rhs[i];
            if dv.abs() > 5.0 {
                dv *= 0.5;
                step_limited = true;
            }
            v_guess[i] -= dv;
            let (g, lp) = port_group[i];
            let (lo, hi) = device_groups[g].v_clamp_port(lp);
            let clamped = v_guess[i].clamp(lo, hi);
            if clamped != v_guess[i] {
                clamp_hit = true;
            }
            v_guess[i] = clamped;
            max_dv = max_dv.max(dv.abs());
        }

        if max_dv < tolerance {
            break;
        }
    }

    // Track iteration cap hits
    let iter_cap_hit = last_residual >= tolerance;

    // Ensure v_guess is clean: if NaN leaked through, reset to midpoint of clamp range
    for i in 0..n_nl {
        if !v_guess[i].is_finite() {
            let (g, lp) = port_group[i];
            let (lo, hi) = device_groups[g].v_clamp_port(lp);
            v_guess[i] = (lo + hi) * 0.5;
        }
    }

    // Compute final reflected waves: b_i = v_i - R_i·i_i(v_i)
    // Re-evaluate all devices at final voltages
    for g in 0..n_groups {
        let np = device_groups[g].n_ports();
        let offset = group_port_offsets[g];
        let v_group = &v_guess[offset..offset + np];
        device_groups[g].eval(
            v_group,
            &mut dev_currents[..np],
            &mut dev_jacobian[..np * np],
        );
        for lp in 0..np {
            currents[offset + lp] = dev_currents[lp];
        }
    }

    let mut b_nl = vec![0.0; n_nl];
    for i in 0..n_nl {
        let b = v_guess[i] - port_resistances[i] * currents[i];
        // TODO(perf): The proper fix is to set each R_port = Z_thévenin at
        // compile time (already done in build.rs adaptive port resistance).
        // Once all port resistances are properly adapted, |b| should never
        // exceed a few hundred volts and this clamp becomes a no-op.
        // Remove this safety clamp after verifying all circuits compile
        // with adapted NL port resistances.
        b_nl[i] = b.clamp(-1000.0, 1000.0);
    }

    #[cfg(feature = "debug-trace")]
    {
        let r_max = port_resistances.iter().copied().fold(0.0_f64, f64::max);
        let r_min = port_resistances
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min);
        let cond = r_max / r_min;
        for i in 0..n_nl {
            let a_i = v_guess[i] + port_resistances[i] * currents[i];
            let a_scatter: f64 = known_a[i]
                + (0..n_nl)
                    .map(|j| s_nl[i * n_nl + j] * b_nl[j])
                    .sum::<f64>();
            let embed_err = (a_i - a_scatter).abs();
            if embed_err > 1e-4 || cond > 1e6 {
                eprintln!(
                    "[NR-grouped] port R cond={cond:.1e} embed_err[{i}]={embed_err:.2e} residual={last_residual:.2e}"
                );
            }
        }
    }

    if let Some(mut entry) = stats_entry {
        entry.residual = last_residual;
        entry.clamp_hit = clamp_hit;
        entry.step_limited = step_limited;
        entry.iter_cap_hit = iter_cap_hit;
        record_solver_stats(entry);
    }
    b_nl
}

// ---------------------------------------------------------------------------
// Unified Newton-Raphson WDF Solver (single-port)
// ---------------------------------------------------------------------------

/// Unified Newton-Raphson solver for all WDF nonlinear root elements.
///
/// Every nonlinear element in the WDF tree solves the same constraint equation:
///
///   `f(v) = a - 2v - 2·Rp·i(v) = 0`
///
/// where `a` is the incident wave, `Rp` is the port resistance, `v` is the
/// voltage across the element, and `i(v)` is the device-specific current.
///
/// The device I-V characteristic is provided as a closure returning
/// `(current, d_current/d_voltage)` at a given voltage.
///
/// Returns the reflected wave `b = 2v - a`.
///
/// # Parameters
/// - `a`: Incident wave from the WDF tree
/// - `rp`: Port resistance (Ω)
/// - `v`: Initial guess for the voltage (mutable, used as starting point)
/// - `max_iter`: Maximum iterations (bounded for real-time safety)
/// - `tolerance`: Convergence threshold for |dv|
/// - `v_clamp`: Optional (min, max) voltage clamping bounds
/// - `step_limit`: If the raw Newton step exceeds this, the step is halved
///   to prevent overshoot. Pass `None` for standard (undamped) Newton.
/// - `device_iv`: Closure returning `(i, di_dv)` for a given voltage
#[inline]
pub(crate) fn newton_raphson_solve<F>(
    a: f64,
    rp: f64,
    mut v: f64,
    max_iter: usize,
    tolerance: f64,
    v_clamp: Option<(f64, f64)>,
    step_limit: Option<f64>,
    mut device_iv: F,
) -> f64
where
    F: FnMut(f64) -> (f64, f64),
{
    #[cfg(feature = "fault-injection")]
    if crate::fault_injection::is_active(crate::fault_injection::Fault::BreakSolverWarmStart) {
        v = 0.0;
    }

    let mut stats_entry = if SOLVER_STATS_ENABLED {
        Some(SolverStatsEntry::default())
    } else {
        None
    };

    for _ in 0..max_iter {
        if let Some(entry) = stats_entry.as_mut() {
            entry.iterations += 1;
        }
        let (i, di_dv) = device_iv(v);

        let f = a - 2.0 * v - 2.0 * rp * i;
        let fp = -2.0 - 2.0 * rp * di_dv;

        if fp.abs() < 1e-15 {
            break;
        }

        let mut dv = f / fp;

        // Adaptive damping: halve step when it exceeds the threshold
        if let Some(limit) = step_limit {
            if dv.abs() > limit {
                dv *= 0.5;
                if let Some(entry) = stats_entry.as_mut() {
                    entry.step_limited = true;
                }
            }
        }

        v -= dv;

        if let Some((lo, hi)) = v_clamp {
            let clamped = v.clamp(lo, hi);
            if clamped != v {
                if let Some(entry) = stats_entry.as_mut() {
                    entry.clamp_hit = true;
                }
            }
            v = clamped;
        }

        if let Some(entry) = stats_entry.as_mut() {
            entry.residual = f.abs();
        }

        if dv.abs() < tolerance {
            break;
        }
    }

    let result = 2.0 * v - a;

    if let Some(mut entry) = stats_entry {
        // Check if we hit the iteration cap without converging
        if entry.iterations as usize == max_iter && entry.residual >= tolerance {
            entry.iter_cap_hit = true;
        }
        record_solver_stats(entry);
    }

    result
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// A simple linear device: i = v / R_device, di/dv = 1/R_device.
    struct LinearDevice {
        conductance: f64,
        v_max: f64,
    }

    impl NlDeviceIv for LinearDevice {
        fn iv(&self, v: f64) -> (f64, f64) {
            (v * self.conductance, self.conductance)
        }
        fn v_clamp(&self) -> (f64, f64) {
            (-self.v_max, self.v_max)
        }
    }

    /// A simple exponential diode: i = Is*(exp(v/Vt) - 1).
    struct SimpleExponentialDiode {
        is: f64,
        vt: f64,
    }

    impl NlDeviceIv for SimpleExponentialDiode {
        fn iv(&self, v: f64) -> (f64, f64) {
            let x = (v / self.vt).clamp(-500.0, 500.0);
            let ev = x.exp();
            let i = self.is * (ev - 1.0);
            let di = self.is * ev / self.vt;
            (i, di)
        }
        fn v_clamp(&self) -> (f64, f64) {
            (-5.0, 5.0)
        }
    }

    #[test]
    fn test_solve_small_linear_1x1() {
        let mut a = [3.0];
        let mut b = [6.0];
        assert!(solve_small_linear(1, &mut a, &mut b));
        assert!((b[0] - 2.0).abs() < 1e-12);
    }

    #[test]
    fn test_solve_small_linear_2x2_cramer() {
        // [2 1; 5 3] · [x; y] = [4; 7]
        // det = 6-5 = 1, x = (3*4-1*7)/1 = 5, y = (2*7-5*4)/1 = -6
        let mut a = [2.0, 1.0, 5.0, 3.0];
        let mut b = [4.0, 7.0];
        assert!(solve_small_linear(2, &mut a, &mut b));
        assert!((b[0] - 5.0).abs() < 1e-10, "x={}", b[0]);
        assert!((b[1] - (-6.0)).abs() < 1e-10, "y={}", b[1]);
    }

    #[test]
    fn test_solve_small_linear_3x3_gaussian() {
        // [1 2 3; 4 5 6; 7 8 10] · x = [14 32 53]
        // Solution: x = [1, 2, 3]
        let mut a = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0];
        let mut b = [14.0, 32.0, 53.0];
        assert!(solve_small_linear(3, &mut a, &mut b));
        assert!((b[0] - 1.0).abs() < 1e-8, "x0={}", b[0]);
        assert!((b[1] - 2.0).abs() < 1e-8, "x1={}", b[1]);
        assert!((b[2] - 3.0).abs() < 1e-8, "x2={}", b[2]);
    }

    #[test]
    fn test_solve_small_linear_singular() {
        let mut a = [1.0, 2.0, 2.0, 4.0]; // Singular (rows proportional)
        let mut b = [3.0, 6.0];
        assert!(!solve_small_linear(2, &mut a, &mut b));
    }

    #[test]
    fn test_multi_nr_two_linear_devices() {
        // Two linear devices (i = v/R_device) with known scattering.
        // For linear devices, the system should converge in very few iterations.
        let r_dev0 = 1000.0;
        let r_dev1 = 2000.0;
        let rp0 = 500.0;
        let rp1 = 800.0;

        let dev0 = LinearDevice {
            conductance: 1.0 / r_dev0,
            v_max: 100.0,
        };
        let dev1 = LinearDevice {
            conductance: 1.0 / r_dev1,
            v_max: 100.0,
        };

        // Simple 2×2 scattering (identity-like, with coupling)
        let s_nl = [0.3, 0.2, 0.2, 0.3];
        let known_a = [1.0, 0.5];
        let port_resistances = [rp0, rp1];
        let devices: [&dyn NlDeviceIv; 2] = [&dev0, &dev1];
        let mut v_guess = [0.0, 0.0];

        let b = multi_port_nr_solve(
            2,
            &s_nl,
            &known_a,
            &port_resistances,
            &devices,
            &mut v_guess,
            50,
            1e-10,
        );

        // Verify: b should be finite and consistent
        assert!(b[0].is_finite(), "b[0] is not finite: {}", b[0]);
        assert!(b[1].is_finite(), "b[1] is not finite: {}", b[1]);

        // For linear devices, b = v - R_port * (v / R_device)
        // = v * (1 - R_port/R_device)
        // Verify consistency: a_i = v_i + R_i * i_i(v_i)
        for i in 0..2 {
            let v = v_guess[i];
            let (current, _) = devices[i].iv(v);
            let a_i = v + port_resistances[i] * current;
            let b_i = v - port_resistances[i] * current;
            assert!(
                (b_i - b[i]).abs() < 1e-8,
                "b[{i}] inconsistent: computed {b_i}, returned {}",
                b[i]
            );

            // Verify scattering: a_i = known_a_i + Σ_j S[i][j] * b_j
            let a_from_scatter = known_a[i] + s_nl[i * 2] * b[0] + s_nl[i * 2 + 1] * b[1];
            assert!(
                (a_i - a_from_scatter).abs() < 1e-8,
                "Scattering inconsistency at port {i}: a={a_i}, scatter={a_from_scatter}",
            );
        }
    }

    #[test]
    fn test_multi_nr_two_identical_diodes() {
        // Two identical diodes with symmetric excitation → symmetric solution.
        let diode0 = SimpleExponentialDiode {
            is: 1e-12,
            vt: 0.02585,
        };
        let diode1 = SimpleExponentialDiode {
            is: 1e-12,
            vt: 0.02585,
        };

        // Symmetric scattering and equal known_a → v0 should equal v1
        let s_nl = [0.2, 0.3, 0.3, 0.2];
        let known_a = [0.5, 0.5];
        let port_resistances = [1000.0, 1000.0];
        let devices: [&dyn NlDeviceIv; 2] = [&diode0, &diode1];
        let mut v_guess = [0.25, 0.25];

        let b = multi_port_nr_solve(
            2,
            &s_nl,
            &known_a,
            &port_resistances,
            &devices,
            &mut v_guess,
            50,
            1e-10,
        );

        assert!(
            (v_guess[0] - v_guess[1]).abs() < 1e-8,
            "Symmetric excitation should give symmetric voltages: {} vs {}",
            v_guess[0],
            v_guess[1]
        );
        assert!(
            (b[0] - b[1]).abs() < 1e-8,
            "Symmetric excitation should give symmetric b: {} vs {}",
            b[0],
            b[1]
        );
    }

    #[test]
    fn test_multi_nr_convergence_various_init() {
        // Same problem with different initial guesses should converge to the same solution.
        let diode0 = SimpleExponentialDiode {
            is: 1e-12,
            vt: 0.02585,
        };
        let diode1 = SimpleExponentialDiode {
            is: 1e-12,
            vt: 0.02585,
        };

        let s_nl = [0.1, 0.4, 0.4, 0.1];
        let known_a = [1.0, -0.5];
        let port_resistances = [500.0, 800.0];
        let devices: [&dyn NlDeviceIv; 2] = [&diode0, &diode1];

        // Cold start (v=0)
        let mut v_cold = [0.0, 0.0];
        let b_cold = multi_port_nr_solve(
            2,
            &s_nl,
            &known_a,
            &port_resistances,
            &devices,
            &mut v_cold,
            50,
            1e-10,
        );

        // Warm start (v near solution)
        let mut v_warm = [0.3, -0.2];
        let b_warm = multi_port_nr_solve(
            2,
            &s_nl,
            &known_a,
            &port_resistances,
            &devices,
            &mut v_warm,
            50,
            1e-10,
        );

        // Both should converge to the same solution
        for i in 0..2 {
            assert!(
                (b_cold[i] - b_warm[i]).abs() < 1e-6,
                "b[{i}] differs between cold and warm start: {} vs {}",
                b_cold[i],
                b_warm[i]
            );
        }
    }

    #[test]
    fn test_multi_nr_singular_jacobian() {
        // Deep cutoff: di/dv ≈ 0. The solver should not produce NaN or inf.
        let diode0 = SimpleExponentialDiode {
            is: 1e-15,
            vt: 0.02585,
        };
        let diode1 = SimpleExponentialDiode {
            is: 1e-15,
            vt: 0.02585,
        };

        let s_nl = [0.3, 0.2, 0.2, 0.3];
        // Very negative known_a → both diodes deep in reverse bias
        let known_a = [-10.0, -10.0];
        let port_resistances = [1000.0, 1000.0];
        let devices: [&dyn NlDeviceIv; 2] = [&diode0, &diode1];
        let mut v_guess = [-5.0, -5.0];

        let b = multi_port_nr_solve(
            2,
            &s_nl,
            &known_a,
            &port_resistances,
            &devices,
            &mut v_guess,
            50,
            1e-10,
        );

        // Should be finite, no NaN/inf
        for i in 0..2 {
            assert!(
                b[i].is_finite(),
                "b[{i}] should be finite in deep cutoff, got {}",
                b[i]
            );
            assert!(
                v_guess[i].is_finite(),
                "v[{i}] should be finite in deep cutoff, got {}",
                v_guess[i]
            );
        }
    }

    #[test]
    fn test_cramer_matches_gaussian() {
        // For N=2, verify Cramer's and Gaussian give the same answer.
        // Use the same 2×2 system for both.
        let a_orig = [3.0, 1.0, 2.0, 5.0];
        let b_orig = [7.0, 11.0];

        // Cramer (N=2 path)
        let mut a2 = a_orig;
        let mut b2 = b_orig;
        assert!(solve_small_linear(2, &mut a2, &mut b2));

        // Gaussian (N=3 path, but we embed the 2×2 in a 3×3 with identity row)
        // Actually, let's just test that the 2×2 result is correct analytically.
        // det = 15 - 2 = 13
        // x = (5*7 - 1*11)/13 = 24/13
        // y = (3*11 - 2*7)/13 = 19/13
        let x_expected = 24.0 / 13.0;
        let y_expected = 19.0 / 13.0;

        assert!(
            (b2[0] - x_expected).abs() < 1e-12,
            "x: {} vs {}",
            b2[0],
            x_expected
        );
        assert!(
            (b2[1] - y_expected).abs() < 1e-12,
            "y: {} vs {}",
            b2[1],
            y_expected
        );
    }

    #[test]
    fn test_multi_nr_damping() {
        // Verify that a large initial step gets damped but still converges.
        // Use very stiff diodes with a far-off initial guess.
        let diode0 = SimpleExponentialDiode {
            is: 1e-12,
            vt: 0.02585,
        };
        let diode1 = SimpleExponentialDiode {
            is: 1e-12,
            vt: 0.02585,
        };

        let s_nl = [0.1, 0.3, 0.3, 0.1];
        let known_a = [2.0, 1.0];
        let port_resistances = [100.0, 100.0];
        let devices: [&dyn NlDeviceIv; 2] = [&diode0, &diode1];

        // Moderately far-off initial guess for diodes (beyond forward voltage
        // but not so extreme that exp(v/Vt) overflows)
        let mut v_guess = [1.5, 1.5];

        let b = multi_port_nr_solve(
            2,
            &s_nl,
            &known_a,
            &port_resistances,
            &devices,
            &mut v_guess,
            100,
            1e-10,
        );

        // Should still converge to finite values
        for i in 0..2 {
            assert!(
                b[i].is_finite(),
                "b[{i}] should be finite after damped convergence, got {}",
                b[i]
            );
        }

        // Verify solution satisfies the system
        for i in 0..2 {
            let v = v_guess[i];
            let (current, _) = devices[i].iv(v);
            let a_i = v + port_resistances[i] * current;
            let b_i = v - port_resistances[i] * current;
            let a_from_scatter = known_a[i] + s_nl[i * 2] * b[0] + s_nl[i * 2 + 1] * b[1];
            assert!(
                (a_i - a_from_scatter).abs() < 1e-4,
                "Scattering residual at port {i}: a={a_i}, scatter={a_from_scatter}",
            );
            assert!(
                (b_i - b[i]).abs() < 1e-8,
                "b[{i}] mismatch: computed {b_i}, returned {}",
                b[i]
            );
        }
    }

    // -----------------------------------------------------------------------
    // Tests for grouped NR solver
    // -----------------------------------------------------------------------

    /// A 2-port device simulating a simplified triode:
    /// Port 0 = grid (diode), Port 1 = plate (controlled by grid voltage).
    struct MockTriodeGroup {
        /// Grid: i_g = Is*(exp(v_g/Vt) - 1)
        grid_is: f64,
        grid_vt: f64,
        /// Plate: i_p = gm * max(v_g, 0) + v_p / rp
        gm: f64,
        rp: f64,
        v_max: f64,
    }

    impl NlDeviceGroupIv for MockTriodeGroup {
        fn n_ports(&self) -> usize {
            2
        }

        fn eval(&self, v: &[f64], currents: &mut [f64], jacobian: &mut [f64]) {
            let vg = v[0];
            let vp = v[1];

            // Grid current (diode)
            let x = (vg / self.grid_vt).clamp(-500.0, 500.0);
            let ev = x.exp();
            let ig = self.grid_is * (ev - 1.0);
            let dig_dvg = self.grid_is * ev / self.grid_vt;
            let dig_dvp = 0.0;

            // Plate current (transconductance + plate resistance)
            let vg_eff = vg.max(0.0);
            let dvg_eff = if vg > 0.0 { 1.0 } else { 0.0 };
            let ip = self.gm * vg_eff + vp / self.rp;
            let dip_dvg = self.gm * dvg_eff;
            let dip_dvp = 1.0 / self.rp;

            currents[0] = ig;
            currents[1] = ip;
            jacobian[0] = dig_dvg; // ∂ig/∂vg
            jacobian[1] = dig_dvp; // ∂ig/∂vp
            jacobian[2] = dip_dvg; // ∂ip/∂vg  (cross-coupling!)
            jacobian[3] = dip_dvp; // ∂ip/∂vp
        }

        fn v_clamp_port(&self, port: usize) -> (f64, f64) {
            match port {
                0 => (-50.0, 10.0),       // grid
                _ => (-50.0, self.v_max), // plate
            }
        }
    }

    /// Wrapper: single-port NlDeviceIv as a 1-port NlDeviceGroupIv.
    struct SinglePortGroup<'a>(&'a dyn NlDeviceIv);

    impl<'a> NlDeviceGroupIv for SinglePortGroup<'a> {
        fn n_ports(&self) -> usize {
            1
        }
        fn eval(&self, v: &[f64], currents: &mut [f64], jacobian: &mut [f64]) {
            let (i, di) = self.0.iv(v[0]);
            currents[0] = i;
            jacobian[0] = di;
        }
        fn v_clamp_port(&self, _port: usize) -> (f64, f64) {
            self.0.v_clamp()
        }
    }

    #[test]
    fn test_grouped_matches_ungrouped_for_independent_devices() {
        // Two independent diodes: grouped solver should give same results
        // as the existing ungrouped solver.
        let diode0 = SimpleExponentialDiode {
            is: 1e-12,
            vt: 0.02585,
        };
        let diode1 = SimpleExponentialDiode {
            is: 1e-12,
            vt: 0.02585,
        };

        let s_nl = [0.2, 0.3, 0.3, 0.2];
        let known_a = [0.8, -0.3];
        let port_resistances = [500.0, 800.0];

        // Ungrouped solver
        let devices: [&dyn NlDeviceIv; 2] = [&diode0, &diode1];
        let mut v_ug = [0.0, 0.0];
        let b_ug = multi_port_nr_solve(
            2,
            &s_nl,
            &known_a,
            &port_resistances,
            &devices,
            &mut v_ug,
            50,
            1e-10,
        );

        // Grouped solver (each diode wrapped as 1-port group)
        let g0 = SinglePortGroup(&diode0 as &dyn NlDeviceIv);
        let g1 = SinglePortGroup(&diode1 as &dyn NlDeviceIv);
        let groups: [&dyn NlDeviceGroupIv; 2] = [&g0, &g1];
        let offsets = [0, 1];
        let mut v_g = [0.0, 0.0];
        let b_g = multi_port_nr_solve_grouped(
            2,
            &s_nl,
            &known_a,
            &port_resistances,
            &groups,
            &offsets,
            &mut v_g,
            50,
            1e-10,
        );

        for i in 0..2 {
            assert!(
                (b_ug[i] - b_g[i]).abs() < 1e-8,
                "Grouped vs ungrouped b[{i}]: {} vs {}",
                b_ug[i],
                b_g[i]
            );
            assert!(
                (v_ug[i] - v_g[i]).abs() < 1e-8,
                "Grouped vs ungrouped v[{i}]: {} vs {}",
                v_ug[i],
                v_g[i]
            );
        }
    }

    #[test]
    fn test_grouped_mock_triode_converges() {
        // Single 2-port triode group with cross-coupled I-V.
        let triode = MockTriodeGroup {
            grid_is: 1e-9,
            grid_vt: 0.025,
            gm: 0.002, // 2 mA/V
            rp: 50_000.0,
            v_max: 300.0,
        };

        let groups: [&dyn NlDeviceGroupIv; 1] = [&triode];
        let offsets = [0];

        // 2×2 scattering (triode is 2 NL ports)
        let s_nl = [0.1, 0.3, 0.3, 0.1];
        let known_a = [0.5, 50.0]; // Grid: small signal, Plate: ~200V supply
        let port_resistances = [100_000.0, 50_000.0]; // Grid: high Z, Plate: rp
        let mut v_guess = [-2.0, 100.0]; // Start near operating point

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

        // Should converge to finite values
        for i in 0..2 {
            assert!(b[i].is_finite(), "b[{i}] not finite: {}", b[i]);
            assert!(v_guess[i].is_finite(), "v[{i}] not finite: {}", v_guess[i]);
        }

        // Verify scattering consistency
        for i in 0..2 {
            let a_i = v_guess[i]
                + port_resistances[i] * {
                    let mut c = [0.0; 2];
                    let mut j = [0.0; 4];
                    triode.eval(&v_guess, &mut c, &mut j);
                    c[i]
                };
            let a_from_scatter = known_a[i] + s_nl[i * 2] * b[0] + s_nl[i * 2 + 1] * b[1];
            assert!(
                (a_i - a_from_scatter).abs() < 1e-4,
                "Port {i}: a={a_i:.6}, scatter={a_from_scatter:.6}"
            );
        }
    }

    #[test]
    fn test_grouped_triode_cross_coupling_matters() {
        // Verify that the cross-coupling (∂ip/∂vg) actually affects the solution.
        // Compare a triode with gm=0 (no cross-coupling) vs gm=0.002.

        let triode_no_cross = MockTriodeGroup {
            grid_is: 1e-9,
            grid_vt: 0.025,
            gm: 0.0, // No cross-coupling
            rp: 50_000.0,
            v_max: 300.0,
        };
        let triode_with_cross = MockTriodeGroup {
            grid_is: 1e-9,
            grid_vt: 0.025,
            gm: 0.002, // 2 mA/V cross-coupling
            rp: 50_000.0,
            v_max: 300.0,
        };

        let s_nl = [0.15, 0.25, 0.25, 0.15];
        let known_a = [1.0, 80.0];
        let port_resistances = [100_000.0, 50_000.0];

        let groups_no: [&dyn NlDeviceGroupIv; 1] = [&triode_no_cross];
        let groups_yes: [&dyn NlDeviceGroupIv; 1] = [&triode_with_cross];
        let offsets = [0];

        let mut v_no = [-1.0, 100.0];
        let b_no = multi_port_nr_solve_grouped(
            2,
            &s_nl,
            &known_a,
            &port_resistances,
            &groups_no,
            &offsets,
            &mut v_no,
            50,
            1e-10,
        );

        let mut v_yes = [-1.0, 100.0];
        let b_yes = multi_port_nr_solve_grouped(
            2,
            &s_nl,
            &known_a,
            &port_resistances,
            &groups_yes,
            &offsets,
            &mut v_yes,
            50,
            1e-10,
        );

        // Plate voltage should differ due to transconductance
        assert!(
            (v_no[1] - v_yes[1]).abs() > 0.01,
            "Cross-coupling should affect plate voltage: v_no={:.4}, v_yes={:.4}",
            v_no[1],
            v_yes[1]
        );

        // Both should be finite
        for i in 0..2 {
            assert!(b_no[i].is_finite());
            assert!(b_yes[i].is_finite());
        }
    }

    #[test]
    fn test_grouped_mixed_single_and_multi_port() {
        // Mix of a 1-port diode and a 2-port triode in the same system.
        let diode = SimpleExponentialDiode {
            is: 1e-12,
            vt: 0.02585,
        };
        let triode = MockTriodeGroup {
            grid_is: 1e-9,
            grid_vt: 0.025,
            gm: 0.001,
            rp: 80_000.0,
            v_max: 300.0,
        };

        let g0 = SinglePortGroup(&diode as &dyn NlDeviceIv);
        let groups: [&dyn NlDeviceGroupIv; 2] = [&g0, &triode];
        let offsets = [0, 1]; // diode at port 0, triode at ports 1,2

        // 3×3 scattering
        let s_nl = [0.1, 0.2, 0.1, 0.2, 0.15, 0.15, 0.1, 0.15, 0.2];
        let known_a = [0.3, 0.5, 60.0];
        let port_resistances = [1000.0, 100_000.0, 80_000.0];
        let mut v_guess = [0.0, -2.0, 100.0];

        let b = multi_port_nr_solve_grouped(
            3,
            &s_nl,
            &known_a,
            &port_resistances,
            &groups,
            &offsets,
            &mut v_guess,
            50,
            1e-8,
        );

        for i in 0..3 {
            assert!(b[i].is_finite(), "b[{i}] not finite: {}", b[i]);
        }
    }

    // -----------------------------------------------------------------------
    // Verification helpers
    // -----------------------------------------------------------------------

    /// Verify the WDF embedding constraint: a_i = v_i + R_i * i_i(v_i)
    /// must equal the scattering relation: a_i = known_a_i + Σ_j S[i][j] * b_j.
    fn verify_embedding_constraint(
        n_nl: usize,
        s_nl: &[f64],
        known_a: &[f64],
        port_resistances: &[f64],
        v: &[f64],
        b: &[f64],
        devices: &[&dyn NlDeviceIv],
        tolerance: f64,
    ) {
        for i in 0..n_nl {
            let (current, _) = devices[i].iv(v[i]);
            let a_device = v[i] + port_resistances[i] * current;
            let a_scatter: f64 = known_a[i]
                + (0..n_nl)
                    .map(|j| s_nl[i * n_nl + j] * b[j])
                    .sum::<f64>();
            let err = (a_device - a_scatter).abs();
            assert!(
                err < tolerance,
                "Embedding constraint violated at port {i}: a_device={a_device:.6e}, \
                 a_scatter={a_scatter:.6e}, err={err:.2e}, tol={tolerance:.2e}"
            );
        }
    }

    /// Verify the system residual F_i is near zero at the converged solution.
    /// F_i = known_a_i + Σ_j S[i][j]*(v_j - R_j*i_j) - v_i - R_i*i_i
    fn verify_device_residuals(
        n_nl: usize,
        s_nl: &[f64],
        known_a: &[f64],
        port_resistances: &[f64],
        v: &[f64],
        devices: &[&dyn NlDeviceIv],
        tolerance: f64,
    ) {
        for i in 0..n_nl {
            let (i_i, _) = devices[i].iv(v[i]);
            let mut fi = known_a[i] - v[i] - port_resistances[i] * i_i;
            for j in 0..n_nl {
                let (i_j, _) = devices[j].iv(v[j]);
                fi += s_nl[i * n_nl + j] * (v[j] - port_resistances[j] * i_j);
            }
            assert!(
                fi.abs() < tolerance,
                "Device residual F[{i}]={fi:.2e} exceeds tolerance {tolerance:.2e}"
            );
        }
    }

    // -----------------------------------------------------------------------
    // Retrofit verification into existing tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_embedding_constraint_two_linear() {
        let dev0 = LinearDevice {
            conductance: 1.0 / 1000.0,
            v_max: 100.0,
        };
        let dev1 = LinearDevice {
            conductance: 1.0 / 2000.0,
            v_max: 100.0,
        };
        let s_nl = [0.3, 0.2, 0.2, 0.3];
        let known_a = [1.0, 0.5];
        let port_resistances = [500.0, 800.0];
        let devices: [&dyn NlDeviceIv; 2] = [&dev0, &dev1];
        let mut v_guess = [0.0, 0.0];

        let b = multi_port_nr_solve(
            2,
            &s_nl,
            &known_a,
            &port_resistances,
            &devices,
            &mut v_guess,
            50,
            1e-10,
        );

        verify_embedding_constraint(2, &s_nl, &known_a, &port_resistances, &v_guess, &b, &devices, 1e-8);
        verify_device_residuals(2, &s_nl, &known_a, &port_resistances, &v_guess, &devices, 1e-8);
    }

    #[test]
    fn test_embedding_constraint_two_diodes() {
        let diode0 = SimpleExponentialDiode {
            is: 1e-12,
            vt: 0.02585,
        };
        let diode1 = SimpleExponentialDiode {
            is: 1e-12,
            vt: 0.02585,
        };
        let s_nl = [0.2, 0.3, 0.3, 0.2];
        let known_a = [0.5, 0.5];
        let port_resistances = [1000.0, 1000.0];
        let devices: [&dyn NlDeviceIv; 2] = [&diode0, &diode1];
        let mut v_guess = [0.25, 0.25];

        let b = multi_port_nr_solve(
            2,
            &s_nl,
            &known_a,
            &port_resistances,
            &devices,
            &mut v_guess,
            50,
            1e-10,
        );

        verify_embedding_constraint(2, &s_nl, &known_a, &port_resistances, &v_guess, &b, &devices, 1e-8);
        verify_device_residuals(2, &s_nl, &known_a, &port_resistances, &v_guess, &devices, 1e-8);
    }

    #[test]
    fn test_embedding_constraint_damped() {
        let diode0 = SimpleExponentialDiode {
            is: 1e-12,
            vt: 0.02585,
        };
        let diode1 = SimpleExponentialDiode {
            is: 1e-12,
            vt: 0.02585,
        };
        let s_nl = [0.1, 0.3, 0.3, 0.1];
        let known_a = [2.0, 1.0];
        let port_resistances = [100.0, 100.0];
        let devices: [&dyn NlDeviceIv; 2] = [&diode0, &diode1];
        let mut v_guess = [1.5, 1.5];

        let b = multi_port_nr_solve(
            2,
            &s_nl,
            &known_a,
            &port_resistances,
            &devices,
            &mut v_guess,
            100,
            1e-10,
        );

        verify_embedding_constraint(2, &s_nl, &known_a, &port_resistances, &v_guess, &b, &devices, 1e-4);
        verify_device_residuals(2, &s_nl, &known_a, &port_resistances, &v_guess, &devices, 1e-4);
    }

    #[test]
    fn test_embedding_constraint_grouped_triode() {
        let triode = MockTriodeGroup {
            grid_is: 1e-9,
            grid_vt: 0.025,
            gm: 0.002,
            rp: 50_000.0,
            v_max: 300.0,
        };
        let groups: [&dyn NlDeviceGroupIv; 1] = [&triode];
        let offsets = [0];
        let s_nl = [0.1, 0.3, 0.3, 0.1];
        let known_a = [0.5, 50.0];
        let port_resistances = [100_000.0, 50_000.0];
        let mut v_guess = [-2.0, 100.0];

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

        // Verify embedding using grouped eval
        for i in 0..2 {
            let mut c = [0.0; 2];
            let mut j = [0.0; 4];
            triode.eval(&v_guess, &mut c, &mut j);
            let a_device = v_guess[i] + port_resistances[i] * c[i];
            let a_scatter = known_a[i] + s_nl[i * 2] * b[0] + s_nl[i * 2 + 1] * b[1];
            assert!(
                (a_device - a_scatter).abs() < 1e-4,
                "Grouped embedding port {i}: a_dev={a_device:.6}, a_scat={a_scatter:.6}"
            );
        }
    }

    // -----------------------------------------------------------------------
    // Jacobian finite-difference tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_jacobian_finite_difference_ungrouped() {
        // Test the Jacobian formula for the ungrouped NR solver:
        // J[i][j] = S[i][j]*(1 - R_j*di_j/dv_j) + δ_ij*(-1 - R_i*di_i/dv_i)
        let diode0 = SimpleExponentialDiode {
            is: 1e-12,
            vt: 0.02585,
        };
        let diode1 = SimpleExponentialDiode {
            is: 1e-12,
            vt: 0.02585,
        };
        let devices: [&dyn NlDeviceIv; 2] = [&diode0, &diode1];

        let s_nl = [0.2, 0.3, 0.3, 0.2];
        let known_a = [0.8, 0.4];
        let port_resistances = [500.0, 800.0];

        // Realistic operating point (forward-biased diodes)
        let v = [0.3, 0.25];
        let n_nl = 2;

        // Compute analytic Jacobian
        let mut jac_analytic = [0.0_f64; 4];
        let mut currents = [0.0; 2];
        let mut derivatives = [0.0; 2];
        for i in 0..n_nl {
            let (c, d) = devices[i].iv(v[i]);
            currents[i] = c;
            derivatives[i] = d;
        }
        for i in 0..n_nl {
            for j in 0..n_nl {
                let sij = s_nl[i * n_nl + j];
                let term = sij * (1.0 - port_resistances[j] * derivatives[j]);
                if i == j {
                    jac_analytic[i * n_nl + j] = term - 1.0 - port_resistances[i] * derivatives[i];
                } else {
                    jac_analytic[i * n_nl + j] = term;
                }
            }
        }

        // Compute F_i at operating point
        let compute_f = |v_test: &[f64]| -> [f64; 2] {
            let mut f = [0.0; 2];
            for i in 0..n_nl {
                let (i_i, _) = devices[i].iv(v_test[i]);
                let mut fi = known_a[i] - v_test[i] - port_resistances[i] * i_i;
                for j in 0..n_nl {
                    let (i_j, _) = devices[j].iv(v_test[j]);
                    fi += s_nl[i * n_nl + j] * (v_test[j] - port_resistances[j] * i_j);
                }
                f[i] = fi;
            }
            f
        };

        // Finite-difference Jacobian
        let eps = 1e-7;
        let f0 = compute_f(&v);
        for j in 0..n_nl {
            let mut v_plus = v;
            let mut v_minus = v;
            v_plus[j] += eps;
            v_minus[j] -= eps;
            let f_plus = compute_f(&v_plus);
            let f_minus = compute_f(&v_minus);
            for i in 0..n_nl {
                let jac_fd = (f_plus[i] - f_minus[i]) / (2.0 * eps);
                let jac_a = jac_analytic[i * n_nl + j];
                let rel_err = (jac_a - jac_fd).abs() / jac_a.abs().max(1e-10);
                // TOLERANCE SOURCE: Central FD with eps=1e-7 has truncation error O(eps²) = 1e-14.
                // Roundoff is O(machine_eps/eps) = 1e-9. For exponential diode with
                // Jacobian entries spanning 1e-12 to 1e3, relative error at small values
                // can reach 1e-3. 1e-4 excludes only gross formula errors.
                assert!(
                    rel_err < 1e-4,
                    "Jacobian FD mismatch J[{i}][{j}]: analytic={jac_a:.6e}, fd={jac_fd:.6e}, \
                     rel_err={rel_err:.2e}, f0={:?}",
                    f0
                );
            }
        }
    }

    #[test]
    fn test_jacobian_finite_difference_grouped() {
        // Test the grouped solver Jacobian with cross-coupled MockTriode
        let triode = MockTriodeGroup {
            grid_is: 1e-9,
            grid_vt: 0.025,
            gm: 0.002,
            rp: 50_000.0,
            v_max: 300.0,
        };
        let groups: [&dyn NlDeviceGroupIv; 1] = [&triode];
        let offsets = [0usize];
        let n_nl = 2;

        let s_nl = [0.1, 0.3, 0.3, 0.1];
        let known_a = [0.5, 50.0];
        let port_resistances = [100_000.0, 50_000.0];

        // Operating point (grid slightly negative, plate at ~100V)
        let v = [0.5, 100.0];

        // Compute the system residual F_i for given voltages
        let compute_f = |v_test: &[f64]| -> [f64; 2] {
            let mut c = [0.0; 2];
            let mut j = [0.0; 4];
            triode.eval(v_test, &mut c, &mut j);
            let mut f = [0.0; 2];
            for i in 0..n_nl {
                let mut fi = known_a[i] - v_test[i] - port_resistances[i] * c[i];
                for jj in 0..n_nl {
                    fi += s_nl[i * n_nl + jj] * (v_test[jj] - port_resistances[jj] * c[jj]);
                }
                f[i] = fi;
            }
            f
        };

        // Compute analytic system Jacobian using the grouped formula:
        // ∂F_i/∂v_k = S[i][k] - δ_{ik}
        //            - R_i · ∂i_i/∂v_k
        //            - Σ_j S[i][j] · R_j · ∂i_j/∂v_k
        let mut dev_c = [0.0; 2];
        let mut dev_j = [0.0; 4]; // 2x2 device Jacobian
        triode.eval(&v, &mut dev_c, &mut dev_j);

        let mut jac_analytic = [0.0_f64; 4];
        for i in 0..n_nl {
            for k in 0..n_nl {
                let mut val = s_nl[i * n_nl + k];
                if i == k {
                    val -= 1.0;
                }
                // -R_i * ∂i_i/∂v_k
                val -= port_resistances[i] * dev_j[i * n_nl + k];
                // -Σ_j S[i][j] * R_j * ∂i_j/∂v_k
                for j in 0..n_nl {
                    val -= s_nl[i * n_nl + j] * port_resistances[j] * dev_j[j * n_nl + k];
                }
                jac_analytic[i * n_nl + k] = val;
            }
        }

        // Finite-difference Jacobian
        let eps = 1e-7;
        for k in 0..n_nl {
            let mut v_plus = v;
            let mut v_minus = v;
            v_plus[k] += eps;
            v_minus[k] -= eps;
            let f_plus = compute_f(&v_plus);
            let f_minus = compute_f(&v_minus);
            for i in 0..n_nl {
                let jac_fd = (f_plus[i] - f_minus[i]) / (2.0 * eps);
                let jac_a = jac_analytic[i * n_nl + k];
                let rel_err = (jac_a - jac_fd).abs() / jac_a.abs().max(1e-10);
                assert!(
                    rel_err < 1e-4,
                    "Grouped Jacobian FD mismatch J[{i}][{k}]: analytic={jac_a:.6e}, \
                     fd={jac_fd:.6e}, rel_err={rel_err:.2e}"
                );
            }
        }
    }

    #[test]
    fn test_device_iv_derivative_accuracy() {
        use super::super::{
            BjtModel, BjtNpnRoot, BjtPnpRoot, DiodeModel, DiodePairRoot, DiodeRoot, PentodeModel,
            PentodeRoot, TriodeModel, TriodeRoot,
        };

        struct TestCase {
            name: &'static str,
            device: Box<dyn NlDeviceIv>,
            test_voltages: Vec<f64>,
        }

        let cases = vec![
            TestCase {
                name: "DiodeRoot(silicon)",
                device: Box::new(DiodeRoot::new(DiodeModel::silicon())),
                test_voltages: vec![-1.0, -0.5, 0.0, 0.2, 0.4, 0.6, 0.7],
            },
            TestCase {
                name: "DiodePairRoot(silicon)",
                device: Box::new(DiodePairRoot::new(DiodeModel::silicon())),
                test_voltages: vec![-0.7, -0.4, -0.2, 0.0, 0.2, 0.4, 0.7],
            },
            TestCase {
                name: "BjtNpnRoot(2N2222)",
                device: Box::new(BjtNpnRoot::new(BjtModel::by_name("2N2222"))),
                test_voltages: vec![-0.5, 0.0, 0.5, 1.0, 5.0, 20.0, 50.0],
            },
            TestCase {
                name: "BjtPnpRoot(2N3906)",
                device: Box::new(BjtPnpRoot::new(BjtModel::by_name("2N3906"))),
                test_voltages: vec![-0.5, 0.0, 0.5, 1.0, 5.0, 20.0, 50.0],
            },
            TestCase {
                name: "TriodeRoot(12AX7)",
                device: Box::new(TriodeRoot::new(TriodeModel::by_name("12AX7"))),
                test_voltages: vec![1.0, 10.0, 50.0, 100.0, 200.0, 350.0],
            },
            TestCase {
                name: "PentodeRoot(EL34)",
                device: Box::new(PentodeRoot::new(PentodeModel::by_name("EL34"))),
                test_voltages: vec![1.0, 10.0, 50.0, 100.0, 200.0, 400.0],
            },
        ];

        let eps = 1e-7;
        for case in &cases {
            let (v_lo, v_hi) = case.device.v_clamp();
            for &v in &case.test_voltages {
                // Skip if outside clamp range
                if v < v_lo || v > v_hi {
                    continue;
                }
                let (_, di_analytic) = case.device.iv(v);
                let (i_plus, _) = case.device.iv(v + eps);
                let (i_minus, _) = case.device.iv(v - eps);
                let di_fd = (i_plus - i_minus) / (2.0 * eps);

                let abs_err = (di_analytic - di_fd).abs();
                // Use relative error for significant derivatives, absolute
                // tolerance for near-zero values (deep cutoff / reverse bias)
                let ok = if di_analytic.abs() > 1e-10 {
                    abs_err / di_analytic.abs() < 1e-3
                } else {
                    abs_err < 1e-12
                };
                assert!(
                    ok,
                    "{}: iv derivative mismatch at v={v:.3}: analytic={di_analytic:.6e}, \
                     fd={di_fd:.6e}, abs_err={abs_err:.2e}",
                    case.name
                );
            }
        }
    }

    // -----------------------------------------------------------------------
    // Linear-limit test
    // -----------------------------------------------------------------------

    #[test]
    fn test_linear_limit_small_signal() {
        // At the system equilibrium, for small perturbations the diode
        // behaves linearly. The NR solver with a real diode and a
        // LinearDevice (with matching conductance) should agree closely.

        struct LinearizedDiode {
            g: f64,
            i_offset: f64,
            v_max: f64,
        }
        impl NlDeviceIv for LinearizedDiode {
            fn iv(&self, v: f64) -> (f64, f64) {
                (self.g * v + self.i_offset, self.g)
            }
            fn v_clamp(&self) -> (f64, f64) {
                (-self.v_max, self.v_max)
            }
        }

        let s_nl = [0.2, 0.3, 0.3, 0.2];
        let known_a_base = [0.5, 0.3];
        let port_resistances = [500.0, 800.0];

        // Step 1: Find the actual equilibrium of the NL system
        let diode_eq0 = SimpleExponentialDiode {
            is: 1e-12,
            vt: 0.02585,
        };
        let diode_eq1 = SimpleExponentialDiode {
            is: 1e-12,
            vt: 0.02585,
        };
        let devices_eq: [&dyn NlDeviceIv; 2] = [&diode_eq0, &diode_eq1];
        let mut v_eq = [0.0, 0.0];
        let _b_eq = multi_port_nr_solve(
            2,
            &s_nl,
            &known_a_base,
            &port_resistances,
            &devices_eq,
            &mut v_eq,
            50,
            1e-12,
        );

        // Step 2: Linearize at the equilibrium voltages
        let (i0_0, g0_0) = diode_eq0.iv(v_eq[0]);
        let (i0_1, g0_1) = diode_eq1.iv(v_eq[1]);

        // Small-signal perturbation: ±1mV around the equilibrium
        for delta in &[0.001, -0.001] {
            let known_a = [known_a_base[0] + delta, known_a_base[1]];

            // Solve with real diode
            let diode0 = SimpleExponentialDiode {
                is: 1e-12,
                vt: 0.02585,
            };
            let diode1 = SimpleExponentialDiode {
                is: 1e-12,
                vt: 0.02585,
            };
            let devices_nl: [&dyn NlDeviceIv; 2] = [&diode0, &diode1];
            let mut v_nl = v_eq;
            let b_nl = multi_port_nr_solve(
                2,
                &s_nl,
                &known_a,
                &port_resistances,
                &devices_nl,
                &mut v_nl,
                50,
                1e-12,
            );

            // Solve with linearized device
            let lin0 = LinearizedDiode {
                g: g0_0,
                i_offset: i0_0 - g0_0 * v_eq[0],
                v_max: 5.0,
            };
            let lin1 = LinearizedDiode {
                g: g0_1,
                i_offset: i0_1 - g0_1 * v_eq[1],
                v_max: 5.0,
            };
            let devices_lin: [&dyn NlDeviceIv; 2] = [&lin0, &lin1];
            let mut v_lin = v_eq;
            let b_lin = multi_port_nr_solve(
                2,
                &s_nl,
                &known_a,
                &port_resistances,
                &devices_lin,
                &mut v_lin,
                50,
                1e-12,
            );

            // Small-signal: should match within 1%
            for i in 0..2 {
                let rel_err = (b_nl[i] - b_lin[i]).abs() / b_nl[i].abs().max(1e-10);
                assert!(
                    rel_err < 0.01,
                    "Small-signal mismatch (delta={delta}) port {i}: \
                     nl={:.6e}, lin={:.6e}, rel_err={rel_err:.4}",
                    b_nl[i],
                    b_lin[i]
                );
            }
        }

        // Large drive: should diverge (sanity check that NL actually matters)
        let known_a_large = [5.0, 3.0];
        let diode0 = SimpleExponentialDiode {
            is: 1e-12,
            vt: 0.02585,
        };
        let diode1 = SimpleExponentialDiode {
            is: 1e-12,
            vt: 0.02585,
        };
        let devices_nl: [&dyn NlDeviceIv; 2] = [&diode0, &diode1];
        let mut v_nl = v_eq;
        let b_nl = multi_port_nr_solve(
            2,
            &s_nl,
            &known_a_large,
            &port_resistances,
            &devices_nl,
            &mut v_nl,
            50,
            1e-12,
        );

        let lin0 = LinearizedDiode {
            g: g0_0,
            i_offset: i0_0 - g0_0 * v_eq[0],
            v_max: 5.0,
        };
        let lin1 = LinearizedDiode {
            g: g0_1,
            i_offset: i0_1 - g0_1 * v_eq[1],
            v_max: 5.0,
        };
        let devices_lin: [&dyn NlDeviceIv; 2] = [&lin0, &lin1];
        let mut v_lin = v_eq;
        let b_lin = multi_port_nr_solve(
            2,
            &s_nl,
            &known_a_large,
            &port_resistances,
            &devices_lin,
            &mut v_lin,
            50,
            1e-12,
        );

        // Large drive should produce measurably different results
        let max_diff = (0..2)
            .map(|i| (b_nl[i] - b_lin[i]).abs())
            .fold(0.0_f64, f64::max);
        assert!(
            max_diff > 0.01,
            "Large drive should show NL divergence, but max_diff={max_diff:.6e}"
        );
    }

    // -----------------------------------------------------------------------
    // Determinism test
    // -----------------------------------------------------------------------

    #[test]
    fn test_solve_deterministic() {
        let diode0 = SimpleExponentialDiode {
            is: 1e-12,
            vt: 0.02585,
        };
        let diode1 = SimpleExponentialDiode {
            is: 1e-12,
            vt: 0.02585,
        };

        let s_nl = [0.2, 0.3, 0.3, 0.2];
        let known_a = [0.8, -0.3];
        let port_resistances = [500.0, 800.0];
        let devices: [&dyn NlDeviceIv; 2] = [&diode0, &diode1];

        // Reference run
        let mut v_ref = [0.0, 0.0];
        let b_ref = multi_port_nr_solve(
            2,
            &s_nl,
            &known_a,
            &port_resistances,
            &devices,
            &mut v_ref,
            50,
            1e-10,
        );

        // Repeat 100 times — must produce bitwise-identical results
        for run in 0..100 {
            let mut v_test = [0.0, 0.0];
            let b_test = multi_port_nr_solve(
                2,
                &s_nl,
                &known_a,
                &port_resistances,
                &devices,
                &mut v_test,
                50,
                1e-10,
            );
            for i in 0..2 {
                assert_eq!(
                    b_ref[i].to_bits(),
                    b_test[i].to_bits(),
                    "Run {run}: b[{i}] differs from reference: {:.17e} vs {:.17e}",
                    b_ref[i],
                    b_test[i]
                );
                assert_eq!(
                    v_ref[i].to_bits(),
                    v_test[i].to_bits(),
                    "Run {run}: v[{i}] differs from reference: {:.17e} vs {:.17e}",
                    v_ref[i],
                    v_test[i]
                );
            }
        }
    }
}
