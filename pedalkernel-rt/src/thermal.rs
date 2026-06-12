//! Thermal drift model for temperature-dependent component behavior.
//!
//! Transistor and diode characteristics shift with temperature. A germanium
//! Fuzz Face played cold sounds different than one that's warmed up for 20
//! minutes. Key temperature dependencies:
//!
//! - **BJT current gain (beta/hFE)**: Increases ~0.5-1%/deg C. A cold germanium
//!   transistor has lower gain -> cleaner, more gated sound.
//! - **Diode saturation current (Is)**: Doubles every ~10 deg C. This shifts the
//!   clipping knee -- warmer = earlier onset, softer clipping.
//! - **Diode thermal voltage (Vt)**: Increases linearly with temperature
//!   (Vt = kT/q ~ 25.85 mV at 20 deg C).
//!
//! The thermal model simulates a simple first-order system: the pedal starts
//! at ambient temperature and warms toward a steady-state operating temperature
//! over time, driven by power dissipation.

use crate::math;

/// Device-specific thermal coefficients.
///
/// Different semiconductor technologies have different temperature sensitivities.
/// These coefficients parameterize the thermal model for the dominant device type.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ThermalCoefficients {
    /// Beta (hFE) temperature coefficient (%/deg C from 25 deg C reference).
    pub beta_tempco: crate::Wave,
    /// Is doubling temperature (deg C).  Is(T) = Is(Tref) * exp(dT / t0)
    pub is_doubling_temp: crate::Wave,
}

impl ThermalCoefficients {
    /// Germanium transistor (AC128, OC44, etc.) -- very temperature-sensitive.
    pub fn germanium_bjt() -> Self {
        Self {
            beta_tempco: 0.015,
            is_doubling_temp: 8.0,
        }
    }

    /// Silicon BJT (2N3904, BC108, etc.) -- moderate sensitivity.
    pub fn silicon_bjt() -> Self {
        Self {
            beta_tempco: 0.007,
            is_doubling_temp: 10.0,
        }
    }

    /// JFET (2N5457, J201, etc.) -- Idss drifts but no beta concept.
    pub fn jfet() -> Self {
        Self {
            beta_tempco: 0.003,
            is_doubling_temp: 12.0,
        }
    }

    /// Vacuum tube -- cathode emission changes with filament temperature.
    pub fn vacuum_tube() -> Self {
        Self {
            beta_tempco: 0.002,
            is_doubling_temp: 15.0,
        }
    }
}

impl Default for ThermalCoefficients {
    /// Default (silicon) for circuits without specific device info.
    fn default() -> Self {
        Self::silicon_bjt()
    }
}

/// Temperature-dependent parameters that change as the circuit warms up.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ThermalState {
    /// Current junction temperature in deg C.
    pub temperature: crate::Wave,
    /// BJT current gain multiplier (relative to nominal at 25 deg C).
    /// > 1.0 when warm, < 1.0 when cold.
    pub beta_multiplier: crate::Wave,
    /// Diode saturation current multiplier (relative to nominal at 25 deg C).
    /// > 1.0 when warm (Is doubles every ~10 deg C).
    pub is_multiplier: crate::Wave,
    /// Thermal voltage at current temperature (V).
    /// Vt = k*T/q where T is in Kelvin. ~25.85 mV at 20 deg C.
    pub vt: crate::Wave,
}

impl ThermalState {
    /// Compute thermal state using device-specific coefficients.
    fn at_temperature_with_coeffs(temp_c: crate::Wave, coeffs: &ThermalCoefficients) -> Self {
        let temp_k = temp_c + 273.15;
        let ref_temp_c = 25.0;

        // Boltzmann constant * T / electron charge
        let vt = 8.617e-5 * temp_k; // k/q * T in eV, = Vt in volts

        // BJT beta: device-specific tempco from reference
        let beta_multiplier = 1.0 + coeffs.beta_tempco * (temp_c - ref_temp_c);

        // Diode Is: exponential temperature dependence with device-specific doubling temp
        let is_multiplier = math::exp((temp_c - ref_temp_c) / coeffs.is_doubling_temp);

        Self {
            temperature: temp_c,
            beta_multiplier: if beta_multiplier > 0.1 {
                beta_multiplier
            } else {
                0.1
            },
            is_multiplier,
            vt,
        }
    }
}

/// Thermal model for a circuit's temperature evolution.
///
/// Models a simple first-order thermal system:
/// - Ambient temperature (starting point)
/// - Steady-state operating temperature (target after warm-up)
/// - Thermal time constant (how fast it warms up)
///
/// The temperature follows: `T(t) = T_ambient + (T_steady - T_ambient) * (1 - exp(-t/tau))`
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ThermalModel {
    /// Ambient (starting) temperature in deg C.
    ambient_temp: crate::Wave,
    /// Steady-state operating temperature in deg C.
    steady_state_temp: crate::Wave,
    /// Thermal time constant in seconds.
    thermal_tau: crate::Wave,
    /// Device-specific thermal coefficients.
    coefficients: ThermalCoefficients,
    /// Current thermal state.
    state: ThermalState,
    /// Elapsed time in seconds.
    elapsed: crate::Wave,
    /// Sample rate (for converting samples to time).
    sample_rate: crate::Wave,
    /// Samples since last thermal update (we don't need to update every sample).
    samples_since_update: usize,
    /// How often to update thermal state (in samples).
    update_interval: usize,
}

impl ThermalModel {
    /// Create a new thermal model with device-specific coefficients.
    pub fn with_coefficients(
        ambient_temp: crate::Wave,
        steady_state_temp: crate::Wave,
        thermal_tau: crate::Wave,
        coefficients: ThermalCoefficients,
        sample_rate: crate::Wave,
    ) -> Self {
        let state = ThermalState::at_temperature_with_coeffs(ambient_temp, &coefficients);
        Self {
            ambient_temp,
            steady_state_temp,
            thermal_tau,
            coefficients,
            state,
            elapsed: 0.0,
            sample_rate,
            samples_since_update: 0,
            update_interval: 1000,
        }
    }

    /// Create a new thermal model with default (silicon) coefficients.
    pub fn new(
        ambient_temp: crate::Wave,
        steady_state_temp: crate::Wave,
        thermal_tau: crate::Wave,
        sample_rate: crate::Wave,
    ) -> Self {
        Self::with_coefficients(
            ambient_temp,
            steady_state_temp,
            thermal_tau,
            ThermalCoefficients::default(),
            sample_rate,
        )
    }

    /// Germanium Fuzz Face thermal model.
    pub fn germanium_fuzz(sample_rate: crate::Wave) -> Self {
        Self::with_coefficients(
            15.0,
            40.0,
            600.0,
            ThermalCoefficients::germanium_bjt(),
            sample_rate,
        )
    }

    /// Silicon circuit thermal model.
    pub fn silicon_standard(sample_rate: crate::Wave) -> Self {
        Self::with_coefficients(
            25.0,
            35.0,
            300.0,
            ThermalCoefficients::silicon_bjt(),
            sample_rate,
        )
    }

    /// Tube amp thermal model.
    pub fn tube_amp(sample_rate: crate::Wave) -> Self {
        Self::with_coefficients(
            25.0,
            60.0,
            900.0,
            ThermalCoefficients::vacuum_tube(),
            sample_rate,
        )
    }

    /// Advance the thermal model by one audio sample.
    #[inline]
    pub fn tick(&mut self) -> &ThermalState {
        self.samples_since_update += 1;
        if self.samples_since_update >= self.update_interval {
            self.samples_since_update = 0;
            self.elapsed += self.update_interval as crate::Wave / self.sample_rate;

            // First-order thermal response
            let alpha = 1.0 - math::exp(-self.elapsed / self.thermal_tau);
            let temp = self.ambient_temp + (self.steady_state_temp - self.ambient_temp) * alpha;

            self.state = ThermalState::at_temperature_with_coeffs(temp, &self.coefficients);
        }
        &self.state
    }

    /// Get current thermal state without advancing time.
    pub fn state(&self) -> &ThermalState {
        &self.state
    }

    /// Get current temperature in deg C.
    pub fn temperature(&self) -> crate::Wave {
        self.state.temperature
    }

    /// Reset to cold start (ambient temperature, elapsed = 0).
    pub fn reset(&mut self) {
        self.elapsed = 0.0;
        self.samples_since_update = 0;
        self.state =
            ThermalState::at_temperature_with_coeffs(self.ambient_temp, &self.coefficients);
    }

    /// Jump to fully warmed up state.
    pub fn warm_up(&mut self) {
        self.elapsed = self.thermal_tau * 10.0;
        self.state =
            ThermalState::at_temperature_with_coeffs(self.steady_state_temp, &self.coefficients);
    }

    /// Set sample rate.
    pub fn set_sample_rate(&mut self, sample_rate: crate::Wave) {
        self.sample_rate = sample_rate;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Test helper: compute thermal state with default (silicon) coefficients.
    fn state_at(temp_c: crate::Wave) -> ThermalState {
        ThermalState::at_temperature_with_coeffs(temp_c, &ThermalCoefficients::default())
    }

    #[test]
    fn thermal_state_at_reference() {
        let state = state_at(25.0);
        assert!(
            (state.beta_multiplier - 1.0).abs() < 0.01,
            "Beta at 25C should be ~1.0: {}",
            state.beta_multiplier
        );
        assert!(
            (state.is_multiplier - 1.0).abs() < 0.01,
            "Is at 25C should be ~1.0: {}",
            state.is_multiplier
        );
        assert!(
            (state.vt - 0.02569).abs() < 0.001,
            "Vt at 25C should be ~25.7mV: {}",
            state.vt
        );
    }

    #[test]
    fn thermal_state_warm() {
        let state = state_at(40.0);
        assert!(
            state.beta_multiplier > 1.05,
            "Beta at 40C should be > 1.05: {}",
            state.beta_multiplier
        );
        assert!(
            state.is_multiplier > 2.0,
            "Is at 40C should be > 2x: {}",
            state.is_multiplier
        );
    }

    #[test]
    fn thermal_state_cold() {
        let state = state_at(5.0);
        assert!(
            state.beta_multiplier < 0.9,
            "Beta at 5C should be < 0.9: {}",
            state.beta_multiplier
        );
        assert!(
            state.is_multiplier < 0.3,
            "Is at 5C should be < 0.3: {}",
            state.is_multiplier
        );
    }

    #[test]
    fn thermal_model_starts_cold() {
        let model = ThermalModel::germanium_fuzz(48000.0);
        assert!(
            (model.temperature() - 15.0).abs() < 0.01,
            "Should start at ambient: {}",
            model.temperature()
        );
    }

    #[test]
    fn thermal_model_warms_up_over_time() {
        let mut model = ThermalModel::germanium_fuzz(48000.0);

        let samples_per_minute = 48000 * 60;

        let t0 = model.temperature();
        assert!((t0 - 15.0).abs() < 0.1, "Start temp: {t0}");

        for _ in 0..5 * samples_per_minute {
            model.tick();
        }
        let t5 = model.temperature();
        assert!(t5 > t0, "Should be warmer after 5 min: {t5} > {t0}");
        assert!(t5 < 40.0, "Should not be at steady state yet: {t5}");

        for _ in 0..55 * samples_per_minute {
            model.tick();
        }
        let t60 = model.temperature();
        assert!(
            (t60 - 40.0).abs() < 2.0,
            "Should be near steady state after 60 min: {t60}"
        );
    }

    #[test]
    fn thermal_model_reset() {
        let mut model = ThermalModel::germanium_fuzz(48000.0);

        model.warm_up();
        assert!(
            (model.temperature() - 40.0).abs() < 0.1,
            "Should be at steady state: {}",
            model.temperature()
        );

        model.reset();
        assert!(
            (model.temperature() - 15.0).abs() < 0.1,
            "Should be back to ambient: {}",
            model.temperature()
        );
    }

    #[test]
    fn thermal_model_warm_up_shortcut() {
        let mut model = ThermalModel::germanium_fuzz(48000.0);
        model.warm_up();

        let state = model.state();
        assert!(
            state.beta_multiplier > 1.05,
            "Warm beta should be elevated: {}",
            state.beta_multiplier
        );
        assert!(
            state.is_multiplier > 2.0,
            "Warm Is should be elevated: {}",
            state.is_multiplier
        );
    }

    #[test]
    fn thermal_model_silicon_less_dramatic() {
        let ge = state_at(40.0);
        let si = state_at(35.0);

        assert!(ge.is_multiplier > si.is_multiplier);
    }

    #[test]
    fn thermal_model_vt_temperature_dependence() {
        let cold = state_at(0.0);
        let hot = state_at(50.0);

        assert!(
            hot.vt > cold.vt,
            "Vt should increase with temperature: cold={}, hot={}",
            cold.vt,
            hot.vt
        );

        let dvt = hot.vt - cold.vt;
        let expected = 50.0 * 8.617e-5;
        assert!(
            (dvt - expected).abs() < 0.001,
            "Vt change: actual={dvt:.4}, expected={expected:.4}"
        );
    }
}
