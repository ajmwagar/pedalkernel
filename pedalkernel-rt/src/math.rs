//! Portable math functions for `no_std` + `std` targets.
//!
//! On `std` builds, these are thin wrappers around native f64 intrinsics.
//! On `no_std` builds, they delegate to `libm`.

#[cfg(feature = "std")]
mod inner {
    #[inline(always)]
    pub fn sin(x: f64) -> f64 { x.sin() }
    #[inline(always)]
    pub fn cos(x: f64) -> f64 { x.cos() }
    #[inline(always)]
    pub fn exp(x: f64) -> f64 { x.exp() }
    #[inline(always)]
    pub fn ln(x: f64) -> f64 { x.ln() }
    #[inline(always)]
    pub fn log10(x: f64) -> f64 { x.log10() }
    #[inline(always)]
    pub fn sqrt(x: f64) -> f64 { x.sqrt() }
    #[inline(always)]
    pub fn powf(x: f64, y: f64) -> f64 { x.powf(y) }
    #[inline(always)]
    pub fn tan(x: f64) -> f64 { x.tan() }
    #[inline(always)]
    pub fn tanh(x: f64) -> f64 { x.tanh() }
    #[inline(always)]
    pub fn abs(x: f64) -> f64 { x.abs() }
    #[inline(always)]
    pub fn atan2(y: f64, x: f64) -> f64 { y.atan2(x) }
    #[inline(always)]
    pub fn floor(x: f64) -> f64 { x.floor() }
    #[inline(always)]
    pub fn ceil(x: f64) -> f64 { x.ceil() }
    #[inline(always)]
    pub fn round(x: f64) -> f64 { x.round() }
    #[inline(always)]
    pub fn atan(x: f64) -> f64 { x.atan() }
    #[inline(always)]
    pub fn log2(x: f64) -> f64 { x.log2() }
    #[inline(always)]
    pub fn sinh(x: f64) -> f64 { x.sinh() }
    #[inline(always)]
    pub fn cosh(x: f64) -> f64 { x.cosh() }
    #[inline(always)]
    pub fn asin(x: f64) -> f64 { x.asin() }
    #[inline(always)]
    pub fn cbrt(x: f64) -> f64 { x.cbrt() }
    #[inline(always)]
    pub fn copysign(x: f64, y: f64) -> f64 { x.copysign(y) }
    #[inline(always)]
    pub fn fma(x: f64, y: f64, z: f64) -> f64 { x.mul_add(y, z) }
}

#[cfg(not(feature = "std"))]
mod inner {
    #[inline(always)]
    pub fn sin(x: f64) -> f64 { libm::sin(x) }
    #[inline(always)]
    pub fn cos(x: f64) -> f64 { libm::cos(x) }
    #[inline(always)]
    pub fn exp(x: f64) -> f64 { libm::exp(x) }
    #[inline(always)]
    pub fn ln(x: f64) -> f64 { libm::log(x) }
    #[inline(always)]
    pub fn log10(x: f64) -> f64 { libm::log10(x) }
    #[inline(always)]
    pub fn sqrt(x: f64) -> f64 { libm::sqrt(x) }
    #[inline(always)]
    pub fn powf(x: f64, y: f64) -> f64 { libm::pow(x, y) }
    #[inline(always)]
    pub fn tan(x: f64) -> f64 { libm::tan(x) }
    #[inline(always)]
    pub fn tanh(x: f64) -> f64 { libm::tanh(x) }
    #[inline(always)]
    pub fn abs(x: f64) -> f64 { libm::fabs(x) }
    #[inline(always)]
    pub fn atan2(y: f64, x: f64) -> f64 { libm::atan2(y, x) }
    #[inline(always)]
    pub fn floor(x: f64) -> f64 { libm::floor(x) }
    #[inline(always)]
    pub fn ceil(x: f64) -> f64 { libm::ceil(x) }
    #[inline(always)]
    pub fn round(x: f64) -> f64 { libm::round(x) }
    #[inline(always)]
    pub fn atan(x: f64) -> f64 { libm::atan(x) }
    #[inline(always)]
    pub fn log2(x: f64) -> f64 { libm::log2(x) }
    #[inline(always)]
    pub fn sinh(x: f64) -> f64 { libm::sinh(x) }
    #[inline(always)]
    pub fn cosh(x: f64) -> f64 { libm::cosh(x) }
    #[inline(always)]
    pub fn asin(x: f64) -> f64 { libm::asin(x) }
    #[inline(always)]
    pub fn cbrt(x: f64) -> f64 { libm::cbrt(x) }
    #[inline(always)]
    pub fn copysign(x: f64, y: f64) -> f64 { libm::copysign(x, y) }
    #[inline(always)]
    pub fn fma(x: f64, y: f64, z: f64) -> f64 { libm::fma(x, y, z) }
}

pub use inner::*;

/// π constant — available in both std and no_std.
pub const PI: f64 = core::f64::consts::PI;
/// τ = 2π
pub const TAU: f64 = core::f64::consts::TAU;
/// e
pub const E: f64 = core::f64::consts::E;
/// ln(2)
pub const LN_2: f64 = core::f64::consts::LN_2;
/// ln(10)
pub const LN_10: f64 = core::f64::consts::LN_10;
/// 1/sqrt(2)
pub const FRAC_1_SQRT_2: f64 = core::f64::consts::FRAC_1_SQRT_2;
