//! Portable math functions for `no_std` + `std` targets.
//!
//! On `std` builds, these are thin wrappers around native Wave intrinsics.
//! On `no_std` builds, they delegate to `libm`.

#[cfg(feature = "std")]
mod inner {
    use crate::Wave;
    #[inline(always)]
    pub fn sin(x: Wave) -> Wave {
        x.sin()
    }
    #[inline(always)]
    pub fn cos(x: Wave) -> Wave {
        x.cos()
    }
    #[inline(always)]
    pub fn exp(x: Wave) -> Wave {
        x.exp()
    }
    #[inline(always)]
    pub fn ln(x: Wave) -> Wave {
        x.ln()
    }
    #[inline(always)]
    pub fn log10(x: Wave) -> Wave {
        x.log10()
    }
    #[inline(always)]
    pub fn sqrt(x: Wave) -> Wave {
        x.sqrt()
    }
    #[inline(always)]
    pub fn powf(x: Wave, y: Wave) -> Wave {
        x.powf(y)
    }
    #[inline(always)]
    pub fn tan(x: Wave) -> Wave {
        x.tan()
    }
    #[inline(always)]
    pub fn tanh(x: Wave) -> Wave {
        x.tanh()
    }
    #[inline(always)]
    pub fn abs(x: Wave) -> Wave {
        x.abs()
    }
    #[inline(always)]
    pub fn atan2(y: Wave, x: Wave) -> Wave {
        y.atan2(x)
    }
    #[inline(always)]
    pub fn floor(x: Wave) -> Wave {
        x.floor()
    }
    #[inline(always)]
    pub fn ceil(x: Wave) -> Wave {
        x.ceil()
    }
    #[inline(always)]
    pub fn round(x: Wave) -> Wave {
        x.round()
    }
    #[inline(always)]
    pub fn atan(x: Wave) -> Wave {
        x.atan()
    }
    #[inline(always)]
    pub fn log2(x: Wave) -> Wave {
        x.log2()
    }
    #[inline(always)]
    pub fn sinh(x: Wave) -> Wave {
        x.sinh()
    }
    #[inline(always)]
    pub fn cosh(x: Wave) -> Wave {
        x.cosh()
    }
    #[inline(always)]
    pub fn asin(x: Wave) -> Wave {
        x.asin()
    }
    #[inline(always)]
    pub fn cbrt(x: Wave) -> Wave {
        x.cbrt()
    }
    #[inline(always)]
    pub fn copysign(x: Wave, y: Wave) -> Wave {
        x.copysign(y)
    }
    #[inline(always)]
    pub fn fma(x: Wave, y: Wave, z: Wave) -> Wave {
        x.mul_add(y, z)
    }
    #[inline(always)]
    pub fn rem_euclid(x: Wave, y: Wave) -> Wave {
        x.rem_euclid(y)
    }
}

#[cfg(not(feature = "std"))]
mod inner {
    use crate::Wave;

    // ARM (Wave=f32): use libm f32 variants to avoid software-emulated crate::Wave.
    // Desktop (Wave=crate::Wave): use libm crate::Wave variants.

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn sin(x: Wave) -> Wave {
        libm::sinf(x)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn sin(x: Wave) -> Wave {
        libm::sin(x)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn cos(x: Wave) -> Wave {
        libm::cosf(x)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn cos(x: Wave) -> Wave {
        libm::cos(x)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn exp(x: Wave) -> Wave {
        libm::expf(x)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn exp(x: Wave) -> Wave {
        libm::exp(x)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn ln(x: Wave) -> Wave {
        libm::logf(x)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn ln(x: Wave) -> Wave {
        libm::log(x)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn log10(x: Wave) -> Wave {
        libm::log10f(x)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn log10(x: Wave) -> Wave {
        libm::log10(x)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn sqrt(x: Wave) -> Wave {
        libm::sqrtf(x)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn sqrt(x: Wave) -> Wave {
        libm::sqrt(x)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn powf(x: Wave, y: Wave) -> Wave {
        libm::powf(x, y)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn powf(x: Wave, y: Wave) -> Wave {
        libm::pow(x, y)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn tan(x: Wave) -> Wave {
        libm::tanf(x)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn tan(x: Wave) -> Wave {
        libm::tan(x)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn tanh(x: Wave) -> Wave {
        libm::tanhf(x)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn tanh(x: Wave) -> Wave {
        libm::tanh(x)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn abs(x: Wave) -> Wave {
        libm::fabsf(x)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn abs(x: Wave) -> Wave {
        libm::fabs(x)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn atan2(y: Wave, x: Wave) -> Wave {
        libm::atan2f(y, x)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn atan2(y: Wave, x: Wave) -> Wave {
        libm::atan2(y, x)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn floor(x: Wave) -> Wave {
        libm::floorf(x)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn floor(x: Wave) -> Wave {
        libm::floor(x)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn ceil(x: Wave) -> Wave {
        libm::ceilf(x)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn ceil(x: Wave) -> Wave {
        libm::ceil(x)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn round(x: Wave) -> Wave {
        libm::roundf(x)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn round(x: Wave) -> Wave {
        libm::round(x)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn atan(x: Wave) -> Wave {
        libm::atanf(x)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn atan(x: Wave) -> Wave {
        libm::atan(x)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn log2(x: Wave) -> Wave {
        libm::log2f(x)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn log2(x: Wave) -> Wave {
        libm::log2(x)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn sinh(x: Wave) -> Wave {
        libm::sinhf(x)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn sinh(x: Wave) -> Wave {
        libm::sinh(x)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn cosh(x: Wave) -> Wave {
        libm::coshf(x)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn cosh(x: Wave) -> Wave {
        libm::cosh(x)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn asin(x: Wave) -> Wave {
        libm::asinf(x)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn asin(x: Wave) -> Wave {
        libm::asin(x)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn cbrt(x: Wave) -> Wave {
        libm::cbrtf(x)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn cbrt(x: Wave) -> Wave {
        libm::cbrt(x)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn copysign(x: Wave, y: Wave) -> Wave {
        libm::copysignf(x, y)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn copysign(x: Wave, y: Wave) -> Wave {
        libm::copysign(x, y)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn fma(x: Wave, y: Wave, z: Wave) -> Wave {
        libm::fmaf(x, y, z)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn fma(x: Wave, y: Wave, z: Wave) -> Wave {
        libm::fma(x, y, z)
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    #[inline(always)]
    pub fn rem_euclid(x: Wave, y: Wave) -> Wave {
        let r = libm::fmodf(x, y);
        if r < 0.0 {
            r + libm::fabsf(y)
        } else {
            r
        }
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    #[inline(always)]
    pub fn rem_euclid(x: Wave, y: Wave) -> Wave {
        let r = libm::fmod(x, y);
        if r < 0.0 {
            r + libm::fabs(y)
        } else {
            r
        }
    }
}

pub use inner::*;

use crate::Wave;

/// π constant — Wave-typed so it can be used directly in Wave expressions.
pub const PI: Wave = core::f64::consts::PI as Wave;
/// τ = 2π
pub const TAU: Wave = core::f64::consts::TAU as Wave;
/// e
pub const E: Wave = core::f64::consts::E as Wave;
/// ln(2)
pub const LN_2: Wave = core::f64::consts::LN_2 as Wave;
/// ln(10)
pub const LN_10: Wave = core::f64::consts::LN_10 as Wave;
/// 1/sqrt(2)
pub const FRAC_1_SQRT_2: Wave = core::f64::consts::FRAC_1_SQRT_2 as Wave;
