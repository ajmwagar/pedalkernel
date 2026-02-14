//! Linear one-port WDF elements: Resistor, Capacitor, Inductor, VoltageSource.

use super::WdfLeaf;

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
