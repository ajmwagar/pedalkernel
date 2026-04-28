/// Non-ideal behavior declared by a component (from its datasheet/SPICE model).
///
/// Each variant is a distinct physical effect. A component returns a `Vec` of
/// these — the stage builder applies them as post-processing. No pattern
/// matching on component type anywhere in the pipeline.
#[derive(Debug, Clone)]
pub enum NonIdealFx {
    /// Gain-bandwidth product limiting + slew rate.
    /// Applied as a first-order IIR lowpass (fc = GBW/gain) followed by
    /// sample-rate-limited dV/dt clamping.
    OpAmpBandwidth {
        /// GBW product in Hz. Determines the -3dB frequency at unity gain.
        gbw: f64,
        /// Maximum output rate of change in V/s.
        slew_rate: f64,
    },
    /// Output rail saturation (tanh soft clip at supply limits).
    /// Separate from power supply sag — this is instantaneous clamping,
    /// not the slow voltage droop under load.
    RailSaturation {
        /// Maximum output swing in V (half-supply minus saturation voltage).
        v_max: f64,
    },
    // Future variants:
    // BjtThermal { thermal_voltage: f64 },
    // TubeGridCurrent { onset_voltage: f64 },
    // PowerSupplySag { esr: f64, filter_cap: f64 },
}
