//! Shared Newton-Raphson solver and numerical utilities for nonlinear WDF roots.

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
// Unified Newton-Raphson WDF Solver
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
    for _ in 0..max_iter {
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
            }
        }

        v -= dv;

        if let Some((lo, hi)) = v_clamp {
            v = v.clamp(lo, hi);
        }

        if dv.abs() < tolerance {
            break;
        }
    }

    2.0 * v - a
}
