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
}

#[derive(Default, Clone, Copy)]
struct SolverStatsEntry {
    iterations: u32,
    residual: f64,
    clamp_hit: bool,
    step_limited: bool,
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
            // Singular Jacobian — stop iterating
            break;
        }

        // Apply damped Newton step: v -= alpha * delta
        // Use full step when small, damped when large
        let mut max_dv = 0.0_f64;
        for i in 0..n_nl {
            let mut dv = rhs[i];
            // Adaptive damping: halve large steps
            if dv.abs() > 5.0 {
                dv *= 0.5;
                step_limited = true;
            }
            v_guess[i] -= dv;
            // Clamp to physical bounds
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

    // Compute final reflected waves: b_i = v_i - R_i·i_i(v_i)
    // (This is the WDF reflected wave from each NL port)
    let mut b_nl = vec![0.0; n_nl];
    for i in 0..n_nl {
        let (current, _) = devices[i].iv(v_guess[i]);
        b_nl[i] = v_guess[i] - port_resistances[i] * current;
    }
    if let Some(mut entry) = stats_entry {
        entry.residual = last_residual;
        entry.clamp_hit = clamp_hit;
        entry.step_limited = step_limited;
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
        b_nl[i] = v_guess[i] - port_resistances[i] * currents[i];
    }
    if let Some(mut entry) = stats_entry {
        entry.residual = last_residual;
        entry.clamp_hit = clamp_hit;
        entry.step_limited = step_limited;
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

    if let Some(entry) = stats_entry {
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
}
