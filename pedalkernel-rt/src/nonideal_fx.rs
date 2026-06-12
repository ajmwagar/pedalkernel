/// Non-ideal behavior declared by a component (from its datasheet/SPICE model).
///
/// Each variant is a distinct physical effect. A component returns a `Vec` of
/// these — the stage builder applies them as post-processing. No pattern
/// matching on component type anywhere in the pipeline.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum NonIdealFx {
    /// Gain-bandwidth product limiting + slew rate.
    /// Applied as a first-order IIR lowpass (fc = GBW/gain) followed by
    /// sample-rate-limited dV/dt clamping.
    OpAmpBandwidth {
        /// GBW product in Hz. Determines the -3dB frequency at unity gain.
        gbw: crate::Wave,
        /// Maximum output rate of change in V/s.
        slew_rate: crate::Wave,
    },
    /// Output rail saturation (tanh soft clip at supply limits).
    /// Separate from power supply sag — this is instantaneous clamping,
    /// not the slow voltage droop under load.
    RailSaturation {
        /// Maximum output swing in V (half-supply minus saturation voltage).
        v_max: crate::Wave,
    },
    // Future variants:
    // BjtThermal { thermal_voltage: crate::Wave },
    // TubeGridCurrent { onset_voltage: crate::Wave },
    // PowerSupplySag { esr: crate::Wave, filter_cap: crate::Wave },
}
