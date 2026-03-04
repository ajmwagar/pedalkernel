//! Controlled resistance elements: Photocoupler (Vactrol), JFET variable resistor.
//!
//! These elements have a resistance that varies based on an external
//! control signal (LED drive current, gate-source voltage, etc.).

use super::{ControlledResistance, WdfLeaf};
use super::nonlinear::JfetModel;

// ---------------------------------------------------------------------------
// Photocoupler Model
// ---------------------------------------------------------------------------

/// Photocoupler (Vactrol) model parameters.
///
/// Models the LED + CdS photoresistor combination found in optical
/// tremolos and Uni-Vibe circuits. Uses a Persyn-style two-rate model
/// for accurate CdS carrier dynamics.
#[derive(Debug, Clone, Copy)]
pub struct PhotocouplerModel {
    /// Resistance when fully dark (Ω). Typical: 500kΩ - 5MΩ.
    pub r_dark: f64,
    /// Minimum resistance at full illumination (Ω). Typical: 100Ω - 5kΩ.
    pub r_light: f64,
    /// Nonlinearity exponent. Typical: 0.7 - 0.9.
    pub gamma: f64,
    /// Fast time constant - rise (s). Typical: 1-3ms.
    pub tau_fast_rise: f64,
    /// Fast time constant - fall (s). Typical: 3-10ms.
    pub tau_fast_fall: f64,
    /// Slow time constant - rise (s). Typical: 10-30ms.
    pub tau_slow_rise: f64,
    /// Slow time constant - fall (s). Typical: 30-100ms.
    pub tau_slow_fall: f64,
    /// Weight of slow component (0-1). Typical: 0.2 - 0.4.
    pub slow_weight: f64,
}

impl PhotocouplerModel {
    /// VTL5C3 - Classic Vactrol used in Uni-Vibe and optical tremolos.
    pub fn vtl5c3() -> Self {
        Self {
            r_dark: 1_000_000.0, // 1MΩ
            r_light: 1_500.0,    // 1.5kΩ
            gamma: 0.75,
            tau_fast_rise: 0.002, // 2ms
            tau_fast_fall: 0.008, // 8ms
            tau_slow_rise: 0.020, // 20ms
            tau_slow_fall: 0.080, // 80ms
            slow_weight: 0.3,
        }
    }

    /// VTL5C1 - Faster response, used in some compressors.
    pub fn vtl5c1() -> Self {
        Self {
            r_dark: 600_000.0, // 600kΩ
            r_light: 600.0,    // 600Ω
            gamma: 0.8,
            tau_fast_rise: 0.001, // 1ms
            tau_fast_fall: 0.005, // 5ms
            tau_slow_rise: 0.010, // 10ms
            tau_slow_fall: 0.050, // 50ms
            slow_weight: 0.25,
        }
    }

    /// NSL-32 - Common optocoupler, moderate response.
    pub fn nsl32() -> Self {
        Self {
            r_dark: 2_000_000.0, // 2MΩ
            r_light: 2_000.0,    // 2kΩ
            gamma: 0.7,
            tau_fast_rise: 0.003, // 3ms
            tau_fast_fall: 0.010, // 10ms
            tau_slow_rise: 0.025, // 25ms
            tau_slow_fall: 0.100, // 100ms
            slow_weight: 0.35,
        }
    }

    /// T4B - Electroluminescent panel + CdS LDR used in LA-2A.
    /// Very slow response gives the characteristic "program-dependent"
    /// compression feel. The EL panel has inherent lag compared to LEDs.
    pub fn t4b() -> Self {
        Self {
            r_dark: 3_000_000.0,  // 3MΩ (very high when no signal)
            r_light: 500.0,       // 500Ω at full drive
            gamma: 0.65,          // More gradual curve
            tau_fast_rise: 0.010, // 10ms - much slower than LED vactrols
            tau_fast_fall: 0.040, // 40ms
            tau_slow_rise: 0.100, // 100ms
            tau_slow_fall: 0.500, // 500ms - very slow release
            slow_weight: 0.50,    // Strong slow component for that LA-2A feel
        }
    }
}

// ---------------------------------------------------------------------------
// Photocoupler
// ---------------------------------------------------------------------------

/// Photocoupler element with Persyn-style two-rate CdS dynamics.
///
/// The LDR is modeled as a resistor whose value depends on LED illumination.
/// The CdS material has two carrier recombination pathways (fast and slow),
/// creating the characteristic "breathing" response of optical tremolos.
///
/// In WDF terms, this is a one-port resistor with time-varying resistance.
/// The LED current is an external control parameter (0.0 = off, 1.0 = full).
#[derive(Debug, Clone, Copy)]
pub struct Photocoupler {
    pub model: PhotocouplerModel,
    /// Fast carrier state (normalized 0-1).
    state_fast: f64,
    /// Slow carrier state (normalized 0-1).
    state_slow: f64,
    /// Current port resistance (Ω).
    resistance: f64,
    /// Sample rate for time constant conversion.
    sample_rate: f64,
    /// Precomputed coefficients for exponential decay.
    alpha_fast_rise: f64,
    alpha_fast_fall: f64,
    alpha_slow_rise: f64,
    alpha_slow_fall: f64,
}

impl Photocoupler {
    pub fn new(model: PhotocouplerModel, sample_rate: f64) -> Self {
        let mut pc = Self {
            model,
            state_fast: 0.0,
            state_slow: 0.0,
            resistance: model.r_dark,
            sample_rate,
            alpha_fast_rise: 0.0,
            alpha_fast_fall: 0.0,
            alpha_slow_rise: 0.0,
            alpha_slow_fall: 0.0,
        };
        pc.update_coefficients();
        pc
    }

    /// Precompute exponential decay coefficients from time constants.
    fn update_coefficients(&mut self) {
        let dt = 1.0 / self.sample_rate;
        // alpha = exp(-dt/tau), so (1 - alpha) is the step response per sample
        self.alpha_fast_rise = (-dt / self.model.tau_fast_rise).exp();
        self.alpha_fast_fall = (-dt / self.model.tau_fast_fall).exp();
        self.alpha_slow_rise = (-dt / self.model.tau_slow_rise).exp();
        self.alpha_slow_fall = (-dt / self.model.tau_slow_fall).exp();
    }

    /// Set LED drive level and update LDR resistance.
    ///
    /// `led_drive` is normalized 0.0 (off) to 1.0 (full brightness).
    /// Call this once per sample before using the element in WDF.
    #[inline]
    pub fn set_led_drive(&mut self, led_drive: f64) {
        let target = led_drive.clamp(0.0, 1.0);

        // Update fast state with asymmetric time constants
        let alpha_fast = if target > self.state_fast {
            self.alpha_fast_rise
        } else {
            self.alpha_fast_fall
        };
        self.state_fast = alpha_fast * self.state_fast + (1.0 - alpha_fast) * target;

        // Update slow state with asymmetric time constants
        let alpha_slow = if target > self.state_slow {
            self.alpha_slow_rise
        } else {
            self.alpha_slow_fall
        };
        self.state_slow = alpha_slow * self.state_slow + (1.0 - alpha_slow) * target;

        // Combine fast and slow components
        let w = self.model.slow_weight;
        let effective_light = (1.0 - w) * self.state_fast + w * self.state_slow;

        // Compute resistance using log-interpolation with gamma nonlinearity
        // x = effective_light^gamma, then R = R_dark^(1-x) * R_light^x
        // This gives R_dark when light=0 and approaches R_light as light→1
        if effective_light > 1e-9 {
            let x = effective_light.powf(self.model.gamma);
            let log_r_dark = self.model.r_dark.ln();
            let log_r_light = self.model.r_light.ln();
            let log_r = log_r_dark + x * (log_r_light - log_r_dark);
            self.resistance = log_r.exp().clamp(self.model.r_light, self.model.r_dark);
        } else {
            self.resistance = self.model.r_dark;
        }
    }

    /// Get current LED drive level (for monitoring).
    pub fn effective_light_level(&self) -> f64 {
        let w = self.model.slow_weight;
        (1.0 - w) * self.state_fast + w * self.state_slow
    }
}

impl WdfLeaf for Photocoupler {
    #[inline]
    fn port_resistance(&self) -> f64 {
        self.resistance
    }

    /// Reflected wave (same as ideal resistor: b = 0 when matched).
    #[inline]
    fn reflected(&self) -> f64 {
        0.0
    }

    /// Set incident wave (no state update needed for resistor).
    #[inline]
    fn set_incident(&mut self, _a: f64) {
        // Resistor has no reactive state
    }

    fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
        self.update_coefficients();
    }

    fn reset(&mut self) {
        self.state_fast = 0.0;
        self.state_slow = 0.0;
        self.resistance = self.model.r_dark;
    }
}

impl ControlledResistance for Photocoupler {
    #[inline]
    fn set_control(&mut self, value: f64) {
        self.set_led_drive(value);
    }

    fn effective_control(&self) -> f64 {
        self.effective_light_level()
    }
}

// ---------------------------------------------------------------------------
// JFET Variable Resistor
// ---------------------------------------------------------------------------

/// JFET operating as a voltage-controlled variable resistor.
///
/// In phaser circuits, JFETs operate in the triode region with small Vds,
/// where they act as voltage-controlled resistors. The drain-source resistance
/// Rds varies with the gate-source voltage Vgs:
///
///   `Rds = Rds_on / (1 - Vgs/Vp)²`
///
/// where:
/// - `Rds_on = 1 / (2 × β × |Vp|)` — resistance at Vgs = 0
/// - `Vp = VTO` — pinch-off voltage (negative for N-channel)
///
/// This replaces the full Newton-Raphson JFET solver for LFO-modulated
/// JFETs that serve as variable resistors (e.g., Phase 90 all-pass stages).
#[derive(Debug, Clone, Copy)]
pub struct JfetVariableResistor {
    pub model: JfetModel,
    /// Current gate-source voltage (V).
    vgs: f64,
    /// Current drain-source resistance (Ω).
    resistance: f64,
    /// Resistance at Vgs = 0: 1 / (2 × β × |Vp|).
    rds_on: f64,
}

impl JfetVariableResistor {
    pub fn new(model: JfetModel) -> Self {
        let rds_on = 1.0 / (2.0 * model.beta * model.vto.abs());
        Self {
            model,
            vgs: 0.0,
            resistance: rds_on,
            rds_on,
        }
    }

    /// Set the gate-source voltage and update Rds.
    #[inline]
    pub fn set_vgs(&mut self, vgs: f64) {
        self.vgs = vgs;
        self.resistance = self.compute_rds();
    }

    /// Current gate-source voltage.
    #[inline]
    pub fn vgs(&self) -> f64 {
        self.vgs
    }

    /// Current drain-source resistance.
    #[inline]
    pub fn rds(&self) -> f64 {
        self.resistance
    }

    /// Resistance at Vgs = 0.
    #[inline]
    pub fn rds_on(&self) -> f64 {
        self.rds_on
    }

    /// Compute Rds from current Vgs using the triode-region formula.
    ///
    /// `Rds = Rds_on / (1 - Vgs/Vp)²`
    ///
    /// The ratio Vgs/Vp is clamped to [-0.95, 0.95] to avoid singularity
    /// at pinch-off and unrealistically low resistance beyond zero bias.
    #[inline]
    fn compute_rds(&self) -> f64 {
        let vp = self.model.vto; // Negative for N-channel
        // Clamp ratio to avoid div-by-zero near pinch-off
        let ratio = (self.vgs / vp).clamp(-0.95, 0.95);
        let denom = 1.0 - ratio;
        (self.rds_on / (denom * denom)).clamp(self.rds_on * 0.5, 10_000_000.0)
    }

    /// Process as a WDF root: simple resistor reflection.
    ///
    /// Returns the reflected wave for a resistive termination with
    /// resistance Rds at the root port with tree port resistance Rp:
    ///
    ///   `b = a × (Rds - Rp) / (Rds + Rp)`
    #[inline]
    pub fn process_root(&self, a: f64, rp: f64) -> f64 {
        let rds = self.resistance;
        let sum = rds + rp;
        if sum.abs() < 1e-15 {
            return 0.0;
        }
        a * (rds - rp) / sum
    }
}

impl WdfLeaf for JfetVariableResistor {
    #[inline]
    fn port_resistance(&self) -> f64 {
        self.resistance
    }

    #[inline]
    fn reflected(&self) -> f64 {
        0.0 // Pure resistor: no stored energy
    }

    #[inline]
    fn set_incident(&mut self, _a: f64) {
        // Resistor has no reactive state
    }

    fn set_sample_rate(&mut self, _sample_rate: f64) {
        // No sample-rate-dependent state
    }

    fn reset(&mut self) {
        self.vgs = 0.0;
        self.resistance = self.rds_on;
    }
}

impl ControlledResistance for JfetVariableResistor {
    #[inline]
    fn set_control(&mut self, value: f64) {
        self.set_vgs(value);
    }

    fn effective_control(&self) -> f64 {
        self.vgs
    }
}
