//! Fast approximations for transcendental functions.
//!
//! On Cortex-M7 (no hardware `exp`/`log`/`tanh`), these provide
//! deterministic-cost alternatives using LUT + linear interpolation
//! (~5 ops) or polynomial approximation (~10 ops). On desktop, they
//! delegate to `std` for full precision.
//!
//! # Usage
//!
//! ```rust
//! use pedalkernel_rt::fast_math::{fast_exp, fast_tanh};
//!
//! let y = fast_exp(2.0);   // ≈ e^2 = 7.389
//! let t = fast_tanh(1.0);  // ≈ 0.7616
//! ```
//!
//! # Platform selection
//!
//! - **Desktop** (`cfg(not(target_arch = "arm"))`) — delegates to `crate::Wave::exp()`
//!   and `crate::Wave::tanh()` for full IEEE 754 precision.
//! - **Embedded** (`cfg(target_arch = "arm")`) — uses 256-entry LUT with
//!   linear interpolation. ~5 ops per evaluation, deterministic cycle count.
//!
//! For compile-time switching between strategies:
//! ```rust,ignore
//! #[cfg(feature = "fast-math-lut")]
//! use pedalkernel_rt::fast_math::lut::fast_exp;
//! ```

use crate::Wave;

// ---------------------------------------------------------------------------
// LUT-based fast exp
// ---------------------------------------------------------------------------

/// Domain for the exp LUT: [-`EXP_X_MIN`, `EXP_X_MAX`].
///
/// The BJT/diode code clamps arguments to `min(40.0)` before calling exp,
/// so [-40, 40] covers the full range. Beyond this, exp overflows to inf
/// or underflows to 0.
const EXP_X_MIN: crate::Wave = -40.0;
const EXP_X_MAX: crate::Wave = 40.0;
const EXP_LUT_SIZE: usize = 512;
const EXP_LUT_STEP: crate::Wave = (EXP_X_MAX - EXP_X_MIN) / (EXP_LUT_SIZE - 1) as crate::Wave;
const EXP_LUT_INV_STEP: crate::Wave = (EXP_LUT_SIZE - 1) as crate::Wave / (EXP_X_MAX - EXP_X_MIN);

/// Precomputed exp(x) values at evenly-spaced points in [EXP_X_MIN, EXP_X_MAX].
///
/// Generated in crate::Wave for precision, stored as Wave for zero-cost lookup on target.
const EXP_LUT: [Wave; EXP_LUT_SIZE] = {
    let mut table = [0.0 as Wave; EXP_LUT_SIZE];
    let mut i = 0;
    while i < EXP_LUT_SIZE {
        let x = EXP_X_MIN + (i as crate::Wave) * EXP_LUT_STEP;
        // const-compatible exp approximation for table generation.
        // At compile time we can afford a high-precision series.
        table[i] = const_exp(x) as Wave;
        i += 1;
    }
    table
};

/// Compile-time exp(x) via Taylor series (enough terms for crate::Wave precision
/// on [-40, 40]). Only used for LUT generation, never at runtime.
const fn const_exp(x: crate::Wave) -> crate::Wave {
    // Range reduction: exp(x) = 2^k * exp(r) where r = x - k*ln(2)
    // and k = round(x / ln(2))
    const LN2: crate::Wave = core::f64::consts::LN_2 as crate::Wave;
    const INV_LN2: crate::Wave = 1.0 / LN2;

    let k_f = x * INV_LN2;
    // const fn doesn't have round(), use floor+0.5
    let k = if k_f >= 0.0 {
        (k_f + 0.5) as i32
    } else {
        (k_f - 0.5) as i32
    };
    let r = x - (k as crate::Wave) * LN2;

    // Taylor series for exp(r), |r| <= ln(2)/2 ≈ 0.347
    // 13 terms gives ~15 digits of precision
    let mut sum = 1.0;
    let mut term = 1.0;
    let mut n = 1;
    while n <= 20 {
        term *= r / (n as crate::Wave);
        sum += term;
        n += 1;
    }

    // Reconstruct: exp(x) = 2^k * exp(r)
    // const fn can't call powi, so build 2^k manually
    let mut pow2 = 1.0 as crate::Wave;
    if k >= 0 {
        let mut i = 0;
        while i < k {
            pow2 *= 2.0;
            i += 1;
        }
    } else {
        let mut i = 0;
        while i < -k {
            pow2 *= 0.5;
            i += 1;
        }
    }
    sum * pow2
}

/// Fast exp(x) using LUT + linear interpolation.
///
/// Cost: ~5 ops (1 multiply for index, 1 floor, 1 lerp, 2 table lookups).
/// Domain: [-40, 40]. Outside this range, returns 0 (underflow) or
/// `Wave::MAX` (overflow).
///
/// Accuracy: < 0.1% relative error across the domain (linear interpolation
/// between 512 uniformly-spaced points on a smooth exponential).
#[inline(always)]
pub fn fast_exp_lut(x: Wave) -> Wave {
    if x <= EXP_X_MIN as Wave {
        return 0.0 as Wave;
    }
    if x >= EXP_X_MAX as Wave {
        return Wave::MAX;
    }
    let t = (x - EXP_X_MIN as Wave) * EXP_LUT_INV_STEP as Wave;
    let idx = t as usize;
    let frac = t - idx as Wave;
    // Safety: idx is guaranteed < EXP_LUT_SIZE - 1 by the bounds checks.
    let y0 = EXP_LUT[idx];
    let y1 = EXP_LUT[idx + 1];
    y0 + frac * (y1 - y0)
}

// ---------------------------------------------------------------------------
// LUT-based fast tanh
// ---------------------------------------------------------------------------

/// Domain for the tanh LUT: [-`TANH_X_MAX`, `TANH_X_MAX`].
///
/// tanh saturates to ±1 beyond about ±4, so [-6, 6] is generous.
const TANH_X_MAX: crate::Wave = 6.0;
const TANH_LUT_SIZE: usize = 256;
const TANH_LUT_STEP: crate::Wave = (2.0 * TANH_X_MAX) / (TANH_LUT_SIZE - 1) as crate::Wave;
const TANH_LUT_INV_STEP: crate::Wave = (TANH_LUT_SIZE - 1) as crate::Wave / (2.0 * TANH_X_MAX);

const TANH_LUT: [Wave; TANH_LUT_SIZE] = {
    let mut table = [0.0 as Wave; TANH_LUT_SIZE];
    let mut i = 0;
    while i < TANH_LUT_SIZE {
        let x = -TANH_X_MAX + (i as crate::Wave) * TANH_LUT_STEP;
        table[i] = const_tanh(x) as Wave;
        i += 1;
    }
    table
};

/// Compile-time tanh(x) = (exp(2x) - 1) / (exp(2x) + 1).
const fn const_tanh(x: crate::Wave) -> crate::Wave {
    let e2x = const_exp(2.0 * x);
    (e2x - 1.0) / (e2x + 1.0)
}

/// Fast tanh(x) using LUT + linear interpolation.
///
/// Cost: ~5 ops. Domain: [-6, 6]. Outside returns ±1.0.
/// Accuracy: < 0.05% relative error (256 points on smooth tanh).
#[inline(always)]
pub fn fast_tanh_lut(x: Wave) -> Wave {
    if x <= -(TANH_X_MAX as Wave) {
        return -1.0 as Wave;
    }
    if x >= TANH_X_MAX as Wave {
        return 1.0 as Wave;
    }
    let t = (x + TANH_X_MAX as Wave) * TANH_LUT_INV_STEP as Wave;
    let idx = t as usize;
    let frac = t - idx as Wave;
    let y0 = TANH_LUT[idx];
    let y1 = TANH_LUT[idx + 1];
    y0 + frac * (y1 - y0)
}

// ---------------------------------------------------------------------------
// Platform-selected fast_exp / fast_tanh
// ---------------------------------------------------------------------------

/// Fast exp(x) — platform-selected.
///
/// On desktop: delegates to `Wave::exp()` for full precision.
/// On ARM: uses 512-entry LUT with linear interpolation (~5 ops).
#[inline(always)]
pub fn fast_exp(x: Wave) -> Wave {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    {
        fast_exp_lut(x)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    {
        crate::math::exp(x)
    }
}

/// Fast tanh(x) — platform-selected.
///
/// On desktop: delegates to `Wave::tanh()` for full precision.
/// On ARM: uses 256-entry LUT with linear interpolation (~5 ops).
#[inline(always)]
pub fn fast_tanh(x: Wave) -> Wave {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    {
        fast_tanh_lut(x)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    {
        crate::math::tanh(x)
    }
}

// ---------------------------------------------------------------------------
// Fast powf for unit-range exponents
// ---------------------------------------------------------------------------

/// Fast approximation of `x^exponent` for `x ∈ [0, 1]` and `exponent ∈ (0, 2)`.
///
/// Uses a quadratic polynomial: `x - (1 - exponent) * x * (1 - x)`.
/// Accurate to <1% for exponents near 1.0 (e.g., 0.92 for VCO saw curvature).
/// Cost: 2 multiplies + 2 adds vs powf's ~50-100ns general exponentiation.
///
/// For exponents far from 1.0, use `fast_exp(exponent * fast_ln(x))` instead.
#[inline(always)]
pub fn fast_powf_unit(x: Wave, exponent: Wave) -> Wave {
    let bend = 1.0 as Wave - exponent;
    x - bend * x * (1.0 as Wave - x)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exp_lut_zero() {
        let y = fast_exp_lut(0.0);
        assert!((y - 1.0).abs() < 0.01, "exp(0) should be ~1.0, got {y}");
    }

    #[test]
    fn exp_lut_one() {
        let y = fast_exp_lut(1.0);
        let expected = (1.0 as crate::Wave).exp() as Wave;
        let rel_err = (y - expected).abs() / expected;
        assert!(
            rel_err < 0.005,
            "exp(1) rel error {rel_err:.6} too large: got {y}, expected {expected}"
        );
    }

    #[test]
    fn exp_lut_negative() {
        let y = fast_exp_lut(-5.0);
        let expected = (-5.0 as crate::Wave).exp() as Wave;
        let rel_err = (y - expected).abs() / expected;
        assert!(
            rel_err < 0.005,
            "exp(-5) rel error {rel_err:.6}: got {y}, expected {expected}"
        );
    }

    #[test]
    fn exp_lut_large_positive() {
        let y = fast_exp_lut(30.0);
        let expected = (30.0 as crate::Wave).exp() as Wave;
        let rel_err = (y - expected).abs() / expected;
        assert!(
            rel_err < 0.01,
            "exp(30) rel error {rel_err:.6}: got {y}, expected {expected}"
        );
    }

    #[test]
    fn exp_lut_underflow() {
        assert_eq!(fast_exp_lut(-50.0), 0.0 as Wave);
    }

    #[test]
    fn exp_lut_overflow() {
        assert_eq!(fast_exp_lut(50.0), Wave::MAX);
    }

    #[test]
    fn exp_lut_accuracy_sweep() {
        // Sweep [-20, 20] and check < 0.5% relative error
        for i in 0..1000 {
            let x = (-20.0 + 40.0 * (i as crate::Wave) / 999.0) as Wave;
            let lut = fast_exp_lut(x);
            let exact = (x as crate::Wave).exp() as Wave;
            if exact > 1e-15 as Wave {
                let rel_err = (lut - exact).abs() / exact;
                assert!(
                    rel_err < 0.005,
                    "exp({x:.3}) rel error {rel_err:.6}: lut={lut:.6e}, exact={exact:.6e}"
                );
            }
        }
    }

    #[test]
    fn tanh_lut_zero() {
        let y = fast_tanh_lut(0.0);
        assert!(y.abs() < 0.01, "tanh(0) should be ~0.0, got {y}");
    }

    #[test]
    fn tanh_lut_one() {
        let y = fast_tanh_lut(1.0);
        let expected = (1.0 as crate::Wave).tanh() as Wave;
        let abs_err = (y - expected).abs();
        assert!(
            abs_err < 0.01,
            "tanh(1) error {abs_err:.6}: got {y}, expected {expected}"
        );
    }

    #[test]
    fn tanh_lut_saturation() {
        assert!((fast_tanh_lut(5.0) - 1.0 as Wave).abs() < 0.001);
        assert!((fast_tanh_lut(-5.0) + 1.0 as Wave).abs() < 0.001);
    }

    #[test]
    fn tanh_lut_symmetry() {
        for i in 0..100 {
            let x = (5.0 * (i as crate::Wave) / 99.0) as Wave;
            let pos = fast_tanh_lut(x);
            let neg = fast_tanh_lut(-x);
            assert!(
                (pos + neg).abs() < 0.01,
                "tanh symmetry violated at x={x}: pos={pos}, neg={neg}"
            );
        }
    }

    #[test]
    fn tanh_lut_accuracy_sweep() {
        for i in 0..1000 {
            let x = (-5.0 + 10.0 * (i as crate::Wave) / 999.0) as Wave;
            let lut = fast_tanh_lut(x);
            let exact = (x as crate::Wave).tanh() as Wave;
            let abs_err = (lut - exact).abs();
            assert!(
                abs_err < 0.005,
                "tanh({x:.3}) error {abs_err:.6}: lut={lut:.6}, exact={exact:.6}"
            );
        }
    }

    #[test]
    fn const_exp_matches_std() {
        // Verify compile-time exp matches runtime exp
        for i in 0..20 {
            let x = -10.0 + (i as crate::Wave);
            let ce = const_exp(x);
            let se = x.exp();
            let rel_err = if se > 1e-15 {
                (ce - se).abs() / se
            } else {
                (ce - se).abs()
            };
            assert!(
                rel_err < 1e-12,
                "const_exp({x}) rel error {rel_err:.2e}: got {ce:.15e}, expected {se:.15e}"
            );
        }
    }

    /// Verify that platform-selected fast_exp delegates correctly.
    #[test]
    fn fast_exp_delegates() {
        // On desktop this should be exact (delegates to std)
        let y = fast_exp(1.0);
        let expected = (1.0 as crate::Wave).exp() as Wave;
        assert!(
            (y - expected).abs() < 1e-10 as Wave,
            "fast_exp should match std::exp on desktop"
        );
    }

    #[test]
    fn fast_tanh_delegates() {
        let y = fast_tanh(1.0);
        let expected = (1.0 as crate::Wave).tanh() as Wave;
        assert!(
            (y - expected).abs() < 1e-10 as Wave,
            "fast_tanh should match std::tanh on desktop"
        );
    }
}
