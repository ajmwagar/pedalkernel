//! Thermal drift model for temperature-dependent component behavior.
//!
//! Transistor and diode characteristics shift with temperature. A germanium
//! Fuzz Face played cold sounds different than one that's warmed up for 20
//! minutes. Key temperature dependencies:
//!
//! - **BJT current gain (β/hFE)**: Increases ~0.5–1%/°C. A cold germanium
//!   transistor has lower gain → cleaner, more gated sound.
//! - **Diode saturation current (Is)**: Doubles every ~10°C. This shifts the
//!   clipping knee — warmer = earlier onset, softer clipping.
//! - **Diode thermal voltage (Vt)**: Increases linearly with temperature
//!   (Vt = kT/q ≈ 25.85 mV at 20°C).
//!
//! The thermal model simulates a simple first-order system: the pedal starts
//! at ambient temperature and warms toward a steady-state operating temperature
//! over time, driven by power dissipation.

/// Temperature-dependent parameters that change as the circuit warms up.
#[derive(Debug, Clone, Copy)]
pub struct ThermalState {
    /// Current junction temperature in °C.
    pub temperature: f64,
    /// BJT current gain multiplier (relative to nominal at 25°C).
    /// > 1.0 when warm, < 1.0 when cold.
    pub beta_multiplier: f64,
    /// Diode saturation current multiplier (relative to nominal at 25°C).
    /// > 1.0 when warm (Is doubles every ~10°C).
    pub is_multiplier: f64,
    /// Thermal voltage at current temperature (V).
    /// Vt = k*T/q where T is in Kelvin. ~25.85 mV at 20°C.
    pub vt: f64,
}

impl ThermalState {
    /// Compute thermal state for a given temperature.
    fn at_temperature(temp_c: f64) -> Self {
        let temp_k = temp_c + 273.15;
        let ref_temp_c = 25.0;

        // Boltzmann constant * T / electron charge
        let vt = 8.617e-5 * temp_k; // k/q * T in eV, = Vt in volts

        // BJT beta: approximately +0.7%/°C from reference
        let beta_multiplier = 1.0 + 0.007 * (temp_c - ref_temp_c);

        // Diode Is: doubles every ~10°C (exponential temperature dependence)
        // Is(T) = Is(Tref) * exp((T - Tref) / T0) where T0 ≈ 14.5°C
        // This gives approximately 2x per 10°C
        let is_multiplier = ((temp_c - ref_temp_c) / 14.5).exp();

        Self {
            temperature: temp_c,
            beta_multiplier: beta_multiplier.max(0.1), // Clamp to prevent negative
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
pub struct ThermalModel {
    /// Ambient (starting) temperature in °C.
    ambient_temp: f64,
    /// Steady-state operating temperature in °C.
    steady_state_temp: f64,
    /// Thermal time constant in seconds.
    /// Typical: 300–600s for a small pedal enclosure.
    thermal_tau: f64,
    /// Current thermal state.
    state: ThermalState,
    /// Elapsed time in seconds.
    elapsed: f64,
    /// Sample rate (for converting samples to time).
    sample_rate: f64,
    /// Samples since last thermal update (we don't need to update every sample).
    samples_since_update: usize,
    /// How often to update thermal state (in samples). Thermal changes are slow
    /// enough that updating every 1000 samples is more than sufficient.
    update_interval: usize,
}

impl ThermalModel {
    /// Create a new thermal model.
    ///
    /// - `ambient_temp`: Starting temperature in °C (room temp: ~20–25°C)
    /// - `steady_state_temp`: Temperature the circuit stabilizes at (30–50°C typical)
    /// - `thermal_tau`: Time constant in seconds (300 = 5 min warm-up)
    /// - `sample_rate`: Audio sample rate
    pub fn new(
        ambient_temp: f64,
        steady_state_temp: f64,
        thermal_tau: f64,
        sample_rate: f64,
    ) -> Self {
        let state = ThermalState::at_temperature(ambient_temp);
        Self {
            ambient_temp,
            steady_state_temp,
            thermal_tau,
            state,
            elapsed: 0.0,
            sample_rate,
            samples_since_update: 0,
            update_interval: 1000,
        }
    }

    /// Germanium Fuzz Face thermal model.
    ///
    /// Germanium transistors are notoriously temperature-sensitive.
    /// A cold germanium Fuzz Face (just powered on in a cold room) sounds
    /// gated and sputtery. After 15–20 minutes it warms up and becomes
    /// smooth and fat.
    ///
    /// Starting at 15°C (cold stage/rehearsal room), warming to 40°C
    /// (enclosure heat from biasing current).
    pub fn germanium_fuzz(sample_rate: f64) -> Self {
        Self::new(15.0, 40.0, 600.0, sample_rate)
    }

    /// Silicon circuit thermal model.
    ///
    /// Silicon devices are less temperature-sensitive but still drift.
    /// Starts at room temp, warms slightly from power dissipation.
    pub fn silicon_standard(sample_rate: f64) -> Self {
        Self::new(25.0, 35.0, 300.0, sample_rate)
    }

    /// Tube amp thermal model.
    ///
    /// Tubes run hot and take longer to reach thermal equilibrium.
    /// The characteristic "tube warm-up" period.
    pub fn tube_amp(sample_rate: f64) -> Self {
        Self::new(25.0, 60.0, 900.0, sample_rate)
    }

    /// Advance the thermal model by one audio sample.
    ///
    /// Returns the current thermal state. The actual temperature update
    /// happens periodically (every `update_interval` samples) since thermal
    /// changes are extremely slow compared to the audio rate.
    #[inline]
    pub fn tick(&mut self) -> &ThermalState {
        self.samples_since_update += 1;
        if self.samples_since_update >= self.update_interval {
            self.samples_since_update = 0;
            self.elapsed += self.update_interval as f64 / self.sample_rate;

            // First-order thermal response
            let alpha = 1.0 - (-self.elapsed / self.thermal_tau).exp();
            let temp =
                self.ambient_temp + (self.steady_state_temp - self.ambient_temp) * alpha;

            self.state = ThermalState::at_temperature(temp);
        }
        &self.state
    }

    /// Get current thermal state without advancing time.
    pub fn state(&self) -> &ThermalState {
        &self.state
    }

    /// Get current temperature in °C.
    pub fn temperature(&self) -> f64 {
        self.state.temperature
    }

    /// Reset to cold start (ambient temperature, elapsed = 0).
    pub fn reset(&mut self) {
        self.elapsed = 0.0;
        self.samples_since_update = 0;
        self.state = ThermalState::at_temperature(self.ambient_temp);
    }

    /// Jump to fully warmed up state.
    pub fn warm_up(&mut self) {
        self.elapsed = self.thermal_tau * 10.0; // Well past steady state
        self.state = ThermalState::at_temperature(self.steady_state_temp);
    }

    /// Set sample rate.
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thermal_state_at_reference() {
        let state = ThermalState::at_temperature(25.0);
        // At 25°C (reference), multipliers should be ~1.0
        assert!(
            (state.beta_multiplier - 1.0).abs() < 0.01,
            "Beta at 25°C should be ~1.0: {}",
            state.beta_multiplier
        );
        assert!(
            (state.is_multiplier - 1.0).abs() < 0.01,
            "Is at 25°C should be ~1.0: {}",
            state.is_multiplier
        );
        assert!(
            (state.vt - 0.02569).abs() < 0.001,
            "Vt at 25°C should be ~25.7mV: {}",
            state.vt
        );
    }

    #[test]
    fn thermal_state_warm() {
        let state = ThermalState::at_temperature(40.0);
        // At 40°C: beta should be higher, Is should be higher
        assert!(
            state.beta_multiplier > 1.05,
            "Beta at 40°C should be > 1.05: {}",
            state.beta_multiplier
        );
        assert!(
            state.is_multiplier > 2.0,
            "Is at 40°C should be > 2x (doubles per ~10°C): {}",
            state.is_multiplier
        );
    }

    #[test]
    fn thermal_state_cold() {
        let state = ThermalState::at_temperature(5.0);
        // At 5°C: beta should be lower, Is should be lower
        assert!(
            state.beta_multiplier < 0.9,
            "Beta at 5°C should be < 0.9: {}",
            state.beta_multiplier
        );
        assert!(
            state.is_multiplier < 0.3,
            "Is at 5°C should be < 0.3 (cold = less leakage): {}",
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

        // Advance 10 minutes worth of samples (10 * 60 * 48000 = 28.8M)
        // That's a lot, so let's check at intervals
        let samples_per_minute = 48000 * 60;

        // After 0 minutes
        let t0 = model.temperature();
        assert!((t0 - 15.0).abs() < 0.1, "Start temp: {t0}");

        // After 5 minutes (half a time constant for 600s tau)
        for _ in 0..5 * samples_per_minute {
            model.tick();
        }
        let t5 = model.temperature();
        assert!(t5 > t0, "Should be warmer after 5 min: {t5} > {t0}");
        assert!(t5 < 40.0, "Should not be at steady state yet: {t5}");

        // After 60 minutes total (well past tau=600s, ~6 time constants)
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

        // Warm up
        model.warm_up();
        assert!(
            (model.temperature() - 40.0).abs() < 0.1,
            "Should be at steady state: {}",
            model.temperature()
        );

        // Reset
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
        let ge = ThermalState::at_temperature(40.0);
        let si = ThermalState::at_temperature(35.0); // Silicon runs cooler

        // Both should see elevated Is, but germanium more so (higher temp)
        assert!(ge.is_multiplier > si.is_multiplier);
    }

    #[test]
    fn thermal_model_vt_temperature_dependence() {
        let cold = ThermalState::at_temperature(0.0);
        let hot = ThermalState::at_temperature(50.0);

        assert!(
            hot.vt > cold.vt,
            "Vt should increase with temperature: cold={}, hot={}",
            cold.vt,
            hot.vt
        );

        // Vt should change by about 1 mV per 12°C
        let dvt = hot.vt - cold.vt;
        let expected = 50.0 * 8.617e-5; // 50 degrees * k/q
        assert!(
            (dvt - expected).abs() < 0.001,
            "Vt change: actual={dvt:.4}, expected={expected:.4}"
        );
    }
}
