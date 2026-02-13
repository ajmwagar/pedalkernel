//! WDF circuit elements - resistors, capacitors, inductors, diodes, etc.

use crate::WdfNode;

/// Ideal resistor
#[derive(Debug, Clone, Copy)]
pub struct Resistor {
    resistance: f64,
    port_resistance: f64,
}

impl Resistor {
    pub fn new(resistance: f64) -> Self {
        Self {
            resistance,
            port_resistance: resistance,
        }
    }
    
    pub fn resistance(&self) -> f64 {
        self.resistance
    }
    
    pub fn set_resistance(&mut self, r: f64) {
        self.resistance = r;
        self.port_resistance = r;
    }
}

impl WdfNode for Resistor {
    fn reflection(&self, incident_wave: f64) -> f64 {
        0.0 // Ideal resistor reflects nothing
    }
    
    fn update(&mut self, _sample_rate: f64) {
        // Resistors have no state to update
    }
    
    fn reset(&mut self) {
        // No state to reset
    }
}

/// Capacitor with WDF implementation
#[derive(Debug, Clone)]
pub struct Capacitor {
    capacitance: f64,
    port_resistance: f64,
    state: f64,
}

impl Capacitor {
    pub fn new(capacitance: f64) -> Self {
        Self {
            capacitance,
            port_resistance: 0.0, // Will be computed based on sample rate
            state: 0.0,
        }
    }
    
    pub fn capacitance(&self) -> f64 {
        self.capacitance
    }
}

impl WdfNode for Capacitor {
    fn reflection(&self, incident_wave: f64) -> f64 {
        self.state
    }
    
    fn update(&mut self, sample_rate: f64) {
        self.port_resistance = 1.0 / (2.0 * self.capacitance * sample_rate);
    }
    
    fn reset(&mut self) {
        self.state = 0.0;
    }
}

/// Inductor with WDF implementation  
#[derive(Debug, Clone)]
pub struct Inductor {
    inductance: f64,
    port_resistance: f64,
    state: f64,
}

impl Inductor {
    pub fn new(inductance: f64) -> Self {
        Self {
            inductance,
            port_resistance: 0.0,
            state: 0.0,
        }
    }
    
    pub fn inductance(&self) -> f64 {
        self.inductance
    }
}

impl WdfNode for Inductor {
    fn reflection(&self, incident_wave: f64) -> f64 {
        -self.state
    }
    
    fn update(&mut self, sample_rate: f64) {
        self.port_resistance = 2.0 * self.inductance * sample_rate;
    }
    
    fn reset(&mut self) {
        self.state = 0.0;
    }
}

/// Ideal voltage source
#[derive(Debug, Clone)]
pub struct VoltageSource {
    voltage: f64,
    port_resistance: f64,
}

impl VoltageSource {
    pub fn new(voltage: f64, port_resistance: f64) -> Self {
        Self {
            voltage,
            port_resistance,
        }
    }
    
    pub fn set_voltage(&mut self, v: f64) {
        self.voltage = v;
    }
}

impl WdfNode for VoltageSource {
    fn reflection(&self, _incident_wave: f64) -> f64 {
        self.voltage
    }
    
    fn update(&mut self, _sample_rate: f64) {
        // No update needed
    }
    
    fn reset(&mut self) {
        // No state to reset
    }
}
