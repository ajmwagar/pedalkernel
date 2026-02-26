//! Linear one-port WDF elements: Resistor, Capacitor, Inductor, VoltageSource,
//! PowerSupply.

use super::WdfLeaf;
use crate::dsl::RectifierType;

// ---------------------------------------------------------------------------
// Resistor
// ---------------------------------------------------------------------------

/// Ideal resistor — absorbs everything, reflects nothing.
///
/// `b = 0` (matched termination when Rp == R)
#[derive(Debug, Clone, Copy)]
pub struct Resistor {
    resistance: f64,
}

impl Resistor {
    pub fn new(resistance: f64) -> Self {
        Self { resistance }
    }

    /// Change the resistance value in-place (no state reset).
    pub fn set_resistance(&mut self, resistance: f64) {
        self.resistance = resistance;
    }
}

impl WdfLeaf for Resistor {
    #[inline]
    fn port_resistance(&self) -> f64 {
        self.resistance
    }

    #[inline]
    fn reflected(&self) -> f64 {
        0.0
    }

    #[inline]
    fn set_incident(&mut self, _a: f64) {
        // Resistor has no state to update
    }
}

// ---------------------------------------------------------------------------
// Capacitor
// ---------------------------------------------------------------------------

/// Capacitor — energy-storage element.
///
/// `b[n] = z^{-1} a[n]` (previous incident becomes current reflected)
/// `Rp = 1 / (2 * fs * C)`
#[derive(Debug, Clone, Copy)]
pub struct Capacitor {
    capacitance: f64,
    resistance: f64,
    state: f64, // z^{-1} of incident wave
}

impl Capacitor {
    pub fn new(capacitance: f64, sample_rate: f64) -> Self {
        Self {
            capacitance,
            resistance: 1.0 / (2.0 * sample_rate * capacitance),
            state: 0.0,
        }
    }
}

impl WdfLeaf for Capacitor {
    #[inline]
    fn port_resistance(&self) -> f64 {
        self.resistance
    }

    #[inline]
    fn reflected(&self) -> f64 {
        self.state
    }

    #[inline]
    fn set_incident(&mut self, a: f64) {
        self.state = a;
    }

    fn set_sample_rate(&mut self, sample_rate: f64) {
        self.resistance = 1.0 / (2.0 * sample_rate * self.capacitance);
    }

    fn reset(&mut self) {
        self.state = 0.0;
    }
}

// ---------------------------------------------------------------------------
// Inductor
// ---------------------------------------------------------------------------

/// Inductor — energy-storage element.
///
/// `b[n] = -z^{-1} a[n]`
/// `Rp = 2 * fs * L`
#[derive(Debug, Clone, Copy)]
pub struct Inductor {
    inductance: f64,
    resistance: f64,
    state: f64,
}

impl Inductor {
    pub fn new(inductance: f64, sample_rate: f64) -> Self {
        Self {
            inductance,
            resistance: 2.0 * sample_rate * inductance,
            state: 0.0,
        }
    }
}

impl WdfLeaf for Inductor {
    #[inline]
    fn port_resistance(&self) -> f64 {
        self.resistance
    }

    #[inline]
    fn reflected(&self) -> f64 {
        -self.state
    }

    #[inline]
    fn set_incident(&mut self, a: f64) {
        self.state = a;
    }

    fn set_sample_rate(&mut self, sample_rate: f64) {
        self.resistance = 2.0 * sample_rate * self.inductance;
    }

    fn reset(&mut self) {
        self.state = 0.0;
    }
}

// ---------------------------------------------------------------------------
// VoltageSource
// ---------------------------------------------------------------------------

/// Ideal voltage source (e.g. input signal injector).
///
/// `b = 2 * Vs - a` where Vs is the source voltage.
/// Port resistance set to a small value (near-ideal source).
#[derive(Debug, Clone, Copy)]
pub struct VoltageSource {
    voltage: f64,
    resistance: f64,
}

impl VoltageSource {
    pub fn new(port_resistance: f64) -> Self {
        Self {
            voltage: 0.0,
            resistance: port_resistance,
        }
    }

    pub fn set_voltage(&mut self, v: f64) {
        self.voltage = v;
    }

    pub fn voltage(&self) -> f64 {
        self.voltage
    }
}

impl WdfLeaf for VoltageSource {
    #[inline]
    fn port_resistance(&self) -> f64 {
        self.resistance
    }

    #[inline]
    fn reflected(&self) -> f64 {
        2.0 * self.voltage
    }

    #[inline]
    fn set_incident(&mut self, _a: f64) {
        // No state update needed
    }
}

// ---------------------------------------------------------------------------
// PowerSupply — B+ rail sag model
// ---------------------------------------------------------------------------

/// Power supply sag model for tube amp B+ rail behavior.
///
/// Models the real electrical phenomenon where the B+ supply voltage drops
/// ("sags") when the output stage draws more current than the supply can
/// deliver instantly. The power supply has output impedance from:
/// - Rectifier forward voltage drop (tube: ~40-80V, solid-state: ~1-2V)
/// - Transformer winding resistance
/// - Filter capacitor ESR (equivalent series resistance)
///
/// The instantaneous sagged voltage is:
///   `V_sagged = V_nominal - I_draw * Z_output`
///
/// The filter capacitor provides energy storage that smooths the sag,
/// creating the characteristic "spongy" compression feel. The RC time
/// constant (Z_output * C_filter) determines how quickly the voltage
/// recovers after a transient.
///
/// This is what distinguishes a Marshall Plexi (tube rectifier, high sag,
/// spongy feel) from a Mesa Boogie (solid-state rectifier, stiff response).
#[derive(Debug, Clone)]
pub struct PowerSupply {
    /// Nominal (no-load) supply voltage (V).
    nominal_voltage: f64,
    /// Total supply output impedance (Ω): rectifier + transformer + ESR.
    impedance: f64,
    /// Filter capacitance (F) — determines sag recovery time constant.
    filter_cap: f64,
    /// Current cap voltage — tracks the RC charge/discharge state.
    cap_voltage: f64,
    /// Reciprocal of sample rate × capacitance (precomputed for efficiency).
    /// This is dt/C where dt = 1/fs.
    dt_over_c: f64,
    /// Sample rate (Hz).
    sample_rate: f64,
    /// Rectifier type (affects static voltage drop model).
    rectifier: RectifierType,
}

impl PowerSupply {
    /// Create a new power supply sag model.
    ///
    /// * `nominal_voltage` — No-load B+ voltage (e.g., 480V)
    /// * `impedance` — Supply output impedance in ohms (tube rect: 50–200Ω, SS: 1–10Ω)
    /// * `filter_cap` — Main filter capacitance in farads (e.g., 40µF)
    /// * `rectifier` — Tube or solid-state rectifier
    /// * `sample_rate` — Audio sample rate (Hz)
    pub fn new(
        nominal_voltage: f64,
        impedance: f64,
        filter_cap: f64,
        rectifier: RectifierType,
        sample_rate: f64,
    ) -> Self {
        Self {
            nominal_voltage,
            impedance,
            filter_cap,
            cap_voltage: nominal_voltage,
            dt_over_c: 1.0 / (sample_rate * filter_cap),
            sample_rate,
            rectifier,
        }
    }

    /// Process one sample: given the instantaneous current draw (A),
    /// returns the sagged supply voltage.
    ///
    /// The model works as follows:
    /// 1. The rectifier charges the filter cap toward V_nominal through Z_output.
    /// 2. The load draws current from the cap, discharging it.
    /// 3. The cap voltage is the actual supply voltage seen by the circuit.
    ///
    /// The differential equation is:
    ///   `C * dVcap/dt = (V_nominal - Vcap) / Z_output - I_load`
    ///
    /// Discretized (forward Euler):
    ///   `Vcap[n+1] = Vcap[n] + dt/C * ((V_nominal - Vcap[n]) / Z_output - I_load)`
    #[inline]
    pub fn tick(&mut self, current_draw: f64) -> f64 {
        // Rectifier charging current: (V_nominal - V_cap) / Z_output
        // This is the current flowing from the rectifier into the filter cap.
        // When V_cap < V_nominal, current flows in to recharge.
        let charge_current = (self.nominal_voltage - self.cap_voltage) / self.impedance;

        // Net current into cap = charge current - load current
        let net_current = charge_current - current_draw;

        // Update cap voltage: dV = I * dt / C
        self.cap_voltage += net_current * self.dt_over_c;

        // Clamp: cap voltage can't exceed nominal (rectifier doesn't reverse)
        // and shouldn't go below zero (physically impossible).
        self.cap_voltage = self.cap_voltage.clamp(0.0, self.nominal_voltage);

        // For tube rectifiers, add a small static voltage drop that increases
        // with current draw (tube rectifier forward drop is current-dependent).
        let static_drop = match self.rectifier {
            RectifierType::Tube => {
                // GZ34-style: ~15V drop at low current, ~60V at high current.
                // Modeled as a linear resistance of ~100Ω in the rectifier itself.
                // This is already captured in the impedance parameter, but we
                // add a minimum standing drop of ~10V for the tube's forward voltage.
                10.0
            }
            RectifierType::SolidState => {
                // Silicon diode bridge: ~1.4V drop (2 × 0.7V forward)
                1.4
            }
        };

        (self.cap_voltage - static_drop).max(0.0)
    }

    /// Update the sample rate (recomputes dt/C).
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
        self.dt_over_c = 1.0 / (sample_rate * self.filter_cap);
    }

    /// Reset to nominal (no-load) state.
    pub fn reset(&mut self) {
        self.cap_voltage = self.nominal_voltage;
    }

    /// Get the current (sagged) voltage.
    pub fn voltage(&self) -> f64 {
        self.cap_voltage
    }

    /// Get the nominal (no-load) voltage.
    pub fn nominal_voltage(&self) -> f64 {
        self.nominal_voltage
    }

    /// Get the steady-state output voltage at no load.
    ///
    /// This accounts for the rectifier static voltage drop that is always
    /// present, even with zero current draw. The cap charges to nominal
    /// voltage, but the output seen by the circuit is always reduced by the
    /// rectifier forward voltage drop (tube: ~10V, solid-state: ~1.4V).
    ///
    /// Use this to initialize `set_supply_voltage()` so the first audio
    /// sample sees the same voltage the PSU will produce at idle.
    pub fn steady_state_voltage(&self) -> f64 {
        let static_drop = match self.rectifier {
            RectifierType::Tube => 10.0,
            RectifierType::SolidState => 1.4,
        };
        (self.nominal_voltage - static_drop).max(0.0)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn power_supply_no_load_returns_nominal() {
        // With zero current draw, voltage should be near nominal minus rectifier drop.
        let mut psu = PowerSupply::new(480.0, 150.0, 40e-6, RectifierType::Tube, 48000.0);
        // After reset, cap is at nominal voltage.
        // With zero load, voltage should stabilize at nominal - static_drop.
        for _ in 0..1000 {
            psu.tick(0.0);
        }
        let v = psu.tick(0.0);
        // Tube rectifier has 10V static drop
        assert!(
            (v - (480.0 - 10.0)).abs() < 1.0,
            "no-load voltage should be near nominal - 10V, got {v}"
        );
    }

    #[test]
    fn power_supply_sags_under_load() {
        // Drawing current should reduce the output voltage.
        let mut psu = PowerSupply::new(480.0, 150.0, 40e-6, RectifierType::Tube, 48000.0);

        // Stabilize at no-load
        for _ in 0..1000 {
            psu.tick(0.0);
        }
        let v_no_load = psu.tick(0.0);

        // Now draw 50mA (typical tube stage current)
        for _ in 0..1000 {
            psu.tick(0.050);
        }
        let v_loaded = psu.tick(0.050);

        assert!(
            v_loaded < v_no_load,
            "loaded voltage ({v_loaded}) should be less than no-load ({v_no_load})"
        );
        // With 150Ω impedance and 50mA, steady-state drop should be ~7.5V
        // (I * Z = 0.050 * 150 = 7.5V)
        let expected_drop = 0.050 * 150.0;
        let actual_drop = v_no_load - v_loaded;
        assert!(
            (actual_drop - expected_drop).abs() < 2.0,
            "sag should be approximately {expected_drop}V, got {actual_drop}V"
        );
    }

    #[test]
    fn power_supply_tube_more_sag_than_solid_state() {
        let mut psu_tube = PowerSupply::new(480.0, 150.0, 40e-6, RectifierType::Tube, 48000.0);
        let mut psu_ss =
            PowerSupply::new(480.0, 5.0, 220e-6, RectifierType::SolidState, 48000.0);

        // Apply a step load of 100mA
        for _ in 0..5000 {
            psu_tube.tick(0.100);
            psu_ss.tick(0.100);
        }
        let v_tube = psu_tube.tick(0.100);
        let v_ss = psu_ss.tick(0.100);

        assert!(
            v_tube < v_ss,
            "tube rectifier should sag more: tube={v_tube}V, SS={v_ss}V"
        );
    }

    #[test]
    fn power_supply_recovers_after_transient() {
        let mut psu = PowerSupply::new(480.0, 150.0, 40e-6, RectifierType::Tube, 48000.0);

        // Stabilize at no-load
        for _ in 0..5000 {
            psu.tick(0.0);
        }
        let v_stable = psu.tick(0.0);

        // Hit it with a big transient
        for _ in 0..1000 {
            psu.tick(0.200); // 200mA — heavy load
        }
        let v_sagged = psu.tick(0.200);
        assert!(v_sagged < v_stable - 10.0, "should sag significantly");

        // Remove load, let it recover
        for _ in 0..50000 {
            psu.tick(0.0);
        }
        let v_recovered = psu.tick(0.0);

        assert!(
            (v_recovered - v_stable).abs() < 1.0,
            "should recover to near no-load: stable={v_stable}, recovered={v_recovered}"
        );
    }

    #[test]
    fn power_supply_voltage_stays_positive() {
        // Even with extreme current draw, voltage should never go negative.
        let mut psu = PowerSupply::new(480.0, 150.0, 40e-6, RectifierType::Tube, 48000.0);
        for _ in 0..100000 {
            let v = psu.tick(10.0); // Unrealistically high current
            assert!(v >= 0.0, "voltage should never go negative, got {v}");
        }
    }

    #[test]
    fn power_supply_solid_state_minimal_drop() {
        // Solid-state rectifier with low impedance should barely sag.
        let mut psu =
            PowerSupply::new(480.0, 5.0, 220e-6, RectifierType::SolidState, 48000.0);

        // Stabilize
        for _ in 0..5000 {
            psu.tick(0.0);
        }
        let v_no_load = psu.tick(0.0);

        // Apply moderate load
        for _ in 0..5000 {
            psu.tick(0.050);
        }
        let v_loaded = psu.tick(0.050);

        // With 5Ω and 50mA, drop should be only ~0.25V
        let drop = v_no_load - v_loaded;
        assert!(
            drop < 1.0,
            "solid-state sag should be minimal: {drop}V"
        );
    }

    #[test]
    fn power_supply_reset_restores_nominal() {
        let mut psu = PowerSupply::new(480.0, 150.0, 40e-6, RectifierType::Tube, 48000.0);

        // Sag heavily
        for _ in 0..10000 {
            psu.tick(0.200);
        }
        assert!(psu.voltage() < 470.0);

        // Reset
        psu.reset();
        assert_eq!(psu.voltage(), 480.0, "reset should restore nominal voltage");
    }

    #[test]
    fn power_supply_sample_rate_affects_recovery() {
        // Higher sample rate means more update ticks per second,
        // but the physical recovery time should be similar.
        let mut psu_48k = PowerSupply::new(480.0, 150.0, 40e-6, RectifierType::Tube, 48000.0);
        let mut psu_96k = PowerSupply::new(480.0, 150.0, 40e-6, RectifierType::Tube, 96000.0);

        // Both sag equally
        for _ in 0..4800 {
            psu_48k.tick(0.100);
        }
        for _ in 0..9600 {
            psu_96k.tick(0.100);
        }
        // After same real-time duration (0.1s), both should have similar sag
        let v_48k = psu_48k.tick(0.100);
        let v_96k = psu_96k.tick(0.100);
        assert!(
            (v_48k - v_96k).abs() < 2.0,
            "sample rate shouldn't affect sag amount: 48k={v_48k}, 96k={v_96k}"
        );
    }

    #[test]
    fn power_supply_steady_state_matches_first_tick() {
        // steady_state_voltage() should match the output of the first tick(0.0)
        // call so that set_supply_voltage() initializes consistently with what
        // the PSU will actually produce.
        let mut psu = PowerSupply::new(350.0, 180.0, 16e-6, RectifierType::Tube, 48000.0);
        let steady = psu.steady_state_voltage();
        let first_tick = psu.tick(0.0);
        assert!(
            (steady - first_tick).abs() < 0.01,
            "steady_state_voltage ({steady}) should match first tick ({first_tick})"
        );
    }

    #[test]
    fn power_supply_steady_state_solid_state() {
        let psu = PowerSupply::new(480.0, 5.0, 220e-6, RectifierType::SolidState, 48000.0);
        let steady = psu.steady_state_voltage();
        assert!(
            (steady - 478.6).abs() < 0.1,
            "SS steady state should be ~478.6V, got {steady}"
        );
    }

    #[test]
    fn power_supply_steady_state_low_voltage_clamped() {
        // A supply with very low nominal voltage should not go negative
        // even with the static drop subtracted.
        let psu = PowerSupply::new(5.0, 10.0, 100e-6, RectifierType::Tube, 48000.0);
        let steady = psu.steady_state_voltage();
        assert!(
            steady >= 0.0,
            "steady state should not be negative, got {steady}"
        );
    }
}
