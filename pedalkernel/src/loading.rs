//! Electrical loading and impedance interaction between pedal stages.
//!
//! Real pedals load each other. A fuzz going into a wah behaves differently
//! than a fuzz in isolation because the input impedance of the wah affects the
//! fuzz's output stage. WDF handles this at the component level within one
//! circuit, but between pedals we need to model the source/load impedance at
//! each junction.
//!
//! This module provides:
//! - [`ImpedanceModel`] — characterizes a pedal's input and output impedance
//! - [`InterstageLoading`] — models the voltage divider and frequency-dependent
//!   loading between two cascaded pedals
//! - [`CableModel`] — guitar cable capacitance (RC low-pass with pickup L)

/// Impedance characteristics for a pedal's input or output.
///
/// Real pedals have frequency-dependent impedance. We model this with a
/// simplified parallel RC equivalent: a resistance in parallel with a
/// capacitance. This captures both the DC impedance and the HF rolloff
/// that loading causes.
#[derive(Debug, Clone, Copy)]
pub struct ImpedanceModel {
    /// DC resistance (Ω). Typical input: 500kΩ–1MΩ. Output: 1kΩ–10kΩ.
    pub resistance: f64,
    /// Parallel capacitance (F). Models HF rolloff. Typical: 10pF–100pF.
    pub capacitance: f64,
}

impl ImpedanceModel {
    /// High impedance input (buffer, most modern pedals).
    /// ~1MΩ input, negligible capacitance.
    pub fn high_z_input() -> Self {
        Self {
            resistance: 1_000_000.0,
            capacitance: 20e-12,
        }
    }

    /// Low impedance input (Fuzz Face, vintage wahs).
    /// ~10kΩ input — significantly loads the source.
    pub fn low_z_input() -> Self {
        Self {
            resistance: 10_000.0,
            capacitance: 50e-12,
        }
    }

    /// Medium impedance input (some fuzzes, trembuckers).
    pub fn medium_z_input() -> Self {
        Self {
            resistance: 100_000.0,
            capacitance: 30e-12,
        }
    }

    /// Low impedance output (buffered output, op-amp based pedals).
    /// ~1kΩ output — drives cables and next pedal easily.
    pub fn low_z_output() -> Self {
        Self {
            resistance: 1_000.0,
            capacitance: 10e-12,
        }
    }

    /// High impedance output (unbuffered, passive tone circuits).
    /// ~50kΩ–100kΩ — susceptible to cable loading.
    pub fn high_z_output() -> Self {
        Self {
            resistance: 50_000.0,
            capacitance: 30e-12,
        }
    }

    /// Guitar pickup output (high impedance, inductive).
    /// Models a typical single-coil pickup: ~7kΩ DC, ~100pF cable loading
    /// creates a resonant peak around 3–5kHz.
    pub fn guitar_pickup() -> Self {
        Self {
            resistance: 7_000.0,
            capacitance: 0.0, // Handled by cable model
        }
    }
}

/// Models the interaction between a source (output) and load (input) impedance.
///
/// The source and load form a frequency-dependent voltage divider:
///
/// ```text
///   V_source ──[R_src + 1/sC_src]──┬──[R_load || 1/sC_load]── V_load
///                                  │
///                                 GND
/// ```
///
/// At DC: `V_load = V_source * R_load / (R_src + R_load)`
/// At high frequencies, the capacitances create additional rolloff.
///
/// For most pedal combinations (buffered output → high-Z input), the loading
/// is negligible. But for impedance-sensitive circuits (guitar → Fuzz Face,
/// wah → fuzz), this interaction is crucial to the sound.
#[derive(Debug, Clone)]
pub struct InterstageLoading {
    /// Source (output) impedance model.
    source: ImpedanceModel,
    /// Load (input) impedance model.
    load: ImpedanceModel,
    /// One-pole low-pass filter state for the combined RC loading.
    lpf_state: f64,
    /// Filter coefficient (computed from combined impedance).
    lpf_coef: f64,
    /// DC attenuation factor: R_load / (R_src + R_load).
    dc_gain: f64,
    /// Sample rate.
    sample_rate: f64,
}

impl InterstageLoading {
    /// Create a new interstage loading model.
    pub fn new(source: ImpedanceModel, load: ImpedanceModel, sample_rate: f64) -> Self {
        let mut s = Self {
            source,
            load,
            lpf_state: 0.0,
            lpf_coef: 0.0,
            dc_gain: 1.0,
            sample_rate,
        };
        s.recompute();
        s
    }

    /// Create a loading model that's effectively transparent (buffer → buffer).
    pub fn transparent(sample_rate: f64) -> Self {
        Self::new(
            ImpedanceModel::low_z_output(),
            ImpedanceModel::high_z_input(),
            sample_rate,
        )
    }

    /// Recompute filter parameters from current impedance models.
    fn recompute(&mut self) {
        let r_src = self.source.resistance;
        let r_load = self.load.resistance;

        // DC voltage divider
        self.dc_gain = r_load / (r_src + r_load);

        // Combined capacitance for frequency-dependent rolloff.
        // The effective RC time constant is:
        //   tau = (R_src || R_load) * (C_src + C_load)
        // where R_src || R_load = R_src * R_load / (R_src + R_load)
        let r_parallel = r_src * r_load / (r_src + r_load);
        let c_total = self.source.capacitance + self.load.capacitance;

        if c_total > 0.0 {
            let fc = 1.0 / (2.0 * std::f64::consts::PI * r_parallel * c_total);
            self.lpf_coef = (-2.0 * std::f64::consts::PI * fc / self.sample_rate).exp();
        } else {
            // No capacitance → no HF rolloff (coef = 0 means no filtering)
            self.lpf_coef = 0.0;
        }
    }

    /// Process one sample through the loading model.
    ///
    /// Applies DC attenuation and frequency-dependent rolloff.
    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
        let attenuated = input * self.dc_gain;

        if self.lpf_coef > 0.0 {
            // One-pole LPF for RC loading
            self.lpf_state =
                self.lpf_coef * self.lpf_state + (1.0 - self.lpf_coef) * attenuated;
            self.lpf_state
        } else {
            attenuated
        }
    }

    /// Get the DC gain (attenuation) of this loading stage.
    pub fn dc_gain(&self) -> f64 {
        self.dc_gain
    }

    /// Update sample rate.
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
        self.recompute();
    }

    /// Reset filter state.
    pub fn reset(&mut self) {
        self.lpf_state = 0.0;
    }
}

/// Guitar cable model — RC low-pass filter.
///
/// Guitar cables act as a low-pass filter due to their distributed
/// capacitance. A typical cable has 30–40 pF/ft (100–130 pF/m).
/// Combined with the guitar pickup's inductance and resistance, this
/// creates a resonant peak followed by rolloff.
///
/// This is why true-bypass pedals can sound darker than buffered ones
/// with long cable runs — the cable capacitance loads the pickup directly.
///
/// Simplified model: first-order RC low-pass where:
/// - R = source impedance (pickup or buffer output)
/// - C = cable capacitance (length × pF/ft)
#[derive(Debug, Clone)]
pub struct CableModel {
    /// Cable capacitance in Farads.
    capacitance: f64,
    /// Source resistance (pickup or buffer output impedance).
    source_resistance: f64,
    /// One-pole filter state.
    lpf_state: f64,
    /// Filter coefficient.
    lpf_coef: f64,
    /// Sample rate.
    sample_rate: f64,
}

impl CableModel {
    /// Create a cable model.
    ///
    /// - `length_feet`: Cable length in feet.
    /// - `pf_per_foot`: Cable capacitance in pF/foot (typical: 30–40).
    /// - `source_resistance`: Output impedance of the driving source.
    /// - `sample_rate`: Audio sample rate.
    pub fn new(
        length_feet: f64,
        pf_per_foot: f64,
        source_resistance: f64,
        sample_rate: f64,
    ) -> Self {
        let capacitance = length_feet * pf_per_foot * 1e-12;
        let mut s = Self {
            capacitance,
            source_resistance,
            lpf_state: 0.0,
            lpf_coef: 0.0,
            sample_rate,
        };
        s.recompute();
        s
    }

    /// Standard 10-foot guitar cable (~30 pF/ft).
    pub fn standard_10ft(source_resistance: f64, sample_rate: f64) -> Self {
        Self::new(10.0, 30.0, source_resistance, sample_rate)
    }

    /// Long 20-foot guitar cable (~30 pF/ft). Noticeable treble rolloff.
    pub fn long_20ft(source_resistance: f64, sample_rate: f64) -> Self {
        Self::new(20.0, 30.0, source_resistance, sample_rate)
    }

    /// Cheap cable with higher capacitance (~40 pF/ft).
    pub fn cheap_20ft(source_resistance: f64, sample_rate: f64) -> Self {
        Self::new(20.0, 40.0, source_resistance, sample_rate)
    }

    fn recompute(&mut self) {
        let tau = self.source_resistance * self.capacitance;
        if tau > 0.0 {
            let fc = 1.0 / (2.0 * std::f64::consts::PI * tau);
            self.lpf_coef = (-2.0 * std::f64::consts::PI * fc / self.sample_rate).exp();
        } else {
            self.lpf_coef = 0.0;
        }
    }

    /// Process one sample.
    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
        if self.lpf_coef > 0.0 {
            self.lpf_state =
                self.lpf_coef * self.lpf_state + (1.0 - self.lpf_coef) * input;
            self.lpf_state
        } else {
            input
        }
    }

    /// Get the -3dB cutoff frequency of this cable.
    pub fn cutoff_hz(&self) -> f64 {
        let tau = self.source_resistance * self.capacitance;
        if tau > 0.0 {
            1.0 / (2.0 * std::f64::consts::PI * tau)
        } else {
            f64::INFINITY
        }
    }

    /// Update sample rate.
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
        self.recompute();
    }

    /// Reset filter state.
    pub fn reset(&mut self) {
        self.lpf_state = 0.0;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interstage_dc_attenuation() {
        // 10kΩ source into 10kΩ load → 50% attenuation
        let source = ImpedanceModel {
            resistance: 10_000.0,
            capacitance: 0.0,
        };
        let load = ImpedanceModel {
            resistance: 10_000.0,
            capacitance: 0.0,
        };
        let loading = InterstageLoading::new(source, load, 48000.0);
        assert!(
            (loading.dc_gain() - 0.5).abs() < 0.001,
            "Equal impedances should give 50% gain: {}",
            loading.dc_gain()
        );
    }

    #[test]
    fn interstage_high_z_load_transparent() {
        // 1kΩ source into 1MΩ load → ~99.9% (effectively transparent)
        let source = ImpedanceModel::low_z_output();
        let load = ImpedanceModel::high_z_input();
        let loading = InterstageLoading::new(source, load, 48000.0);
        assert!(
            loading.dc_gain() > 0.999,
            "Low-Z into high-Z should be transparent: {}",
            loading.dc_gain()
        );
    }

    #[test]
    fn interstage_fuzz_face_loading() {
        // Guitar pickup (7kΩ) into Fuzz Face (10kΩ input) → significant loading
        let source = ImpedanceModel::guitar_pickup();
        let load = ImpedanceModel::low_z_input(); // 10kΩ Fuzz Face
        let loading = InterstageLoading::new(source, load, 48000.0);
        let gain = loading.dc_gain();
        // 10k / (7k + 10k) = 0.588
        assert!(
            (gain - 0.588).abs() < 0.01,
            "Guitar → Fuzz Face should show significant loading: {gain}"
        );
    }

    #[test]
    fn interstage_processes_signal() {
        let source = ImpedanceModel::low_z_output();
        let load = ImpedanceModel::high_z_input();
        let mut loading = InterstageLoading::new(source, load, 48000.0);

        // Process DC signal through transparent loading
        let mut out = 0.0;
        for _ in 0..1000 {
            out = loading.process(1.0);
        }
        assert!(
            (out - 1.0).abs() < 0.01,
            "Transparent loading should pass DC: {out}"
        );
    }

    #[test]
    fn interstage_low_z_loading_attenuates() {
        let source = ImpedanceModel {
            resistance: 10_000.0,
            capacitance: 0.0,
        };
        let load = ImpedanceModel {
            resistance: 10_000.0,
            capacitance: 0.0,
        };
        let mut loading = InterstageLoading::new(source, load, 48000.0);

        let mut out = 0.0;
        for _ in 0..1000 {
            out = loading.process(1.0);
        }
        assert!(
            (out - 0.5).abs() < 0.01,
            "Equal-Z loading should halve signal: {out}"
        );
    }

    #[test]
    fn cable_model_cutoff_frequency() {
        // 20ft cable, 30 pF/ft, 7kΩ pickup
        // C = 20 * 30e-12 = 600e-12 = 600pF
        // fc = 1 / (2π * 7000 * 600e-12) ≈ 37.9 kHz
        let cable = CableModel::long_20ft(7000.0, 48000.0);
        let fc = cable.cutoff_hz();
        assert!(
            (fc - 37_894.0).abs() < 100.0,
            "20ft cable with 7kΩ source: fc={fc:.0} Hz"
        );
    }

    #[test]
    fn cable_model_longer_cable_lower_cutoff() {
        let short = CableModel::standard_10ft(7000.0, 48000.0);
        let long = CableModel::long_20ft(7000.0, 48000.0);

        assert!(
            long.cutoff_hz() < short.cutoff_hz(),
            "Longer cable should have lower cutoff: short={:.0}, long={:.0}",
            short.cutoff_hz(),
            long.cutoff_hz()
        );
    }

    #[test]
    fn cable_model_higher_source_z_lower_cutoff() {
        // High-Z source (pickup) should be more affected than low-Z (buffer)
        let high_z = CableModel::long_20ft(7000.0, 48000.0);
        let low_z = CableModel::long_20ft(1000.0, 48000.0);

        assert!(
            high_z.cutoff_hz() < low_z.cutoff_hz(),
            "Higher source Z should lower cutoff: hi={:.0}, lo={:.0}",
            high_z.cutoff_hz(),
            low_z.cutoff_hz()
        );
    }

    #[test]
    fn cable_model_processes_signal() {
        let mut cable = CableModel::standard_10ft(7000.0, 48000.0);

        // DC should pass through (cutoff is well above 0 Hz)
        let mut out = 0.0;
        for _ in 0..5000 {
            out = cable.process(1.0);
        }
        assert!(
            (out - 1.0).abs() < 0.01,
            "Cable should pass DC: {out}"
        );
    }

    #[test]
    fn cable_model_reset() {
        let mut cable = CableModel::long_20ft(7000.0, 48000.0);

        for _ in 0..1000 {
            cable.process(1.0);
        }
        cable.reset();

        let out = cable.process(0.0);
        assert!(
            out.abs() < 1e-10,
            "After reset, output should be zero: {out}"
        );
    }
}
