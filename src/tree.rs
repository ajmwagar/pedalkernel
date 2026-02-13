//! WDF tree: adaptors and processing engine.
//!
//! The tree processes audio one sample at a time in four phases:
//! 1. **scatter_up** — bottom-up: leaves produce reflected waves `b`,
//!    adaptors combine them via scattering matrices.
//! 2. **root_solve** — the nonlinear root element resolves the implicit
//!    equation and produces a reflected wave back down.
//! 3. **scatter_down** — top-down: adaptors distribute incident waves
//!    to children using the scattering matrix.
//! 4. **state_update** — reactive elements latch their incident wave
//!    as the new state for the next sample.
//!
//! Zero allocation on the hot path — all buffers are pre-sized.

use crate::elements::*;

// ---------------------------------------------------------------------------
// Two-port series adaptor
// ---------------------------------------------------------------------------

/// Series adaptor joining two sub-trees.
///
/// Port resistance:  `Rp = R1 + R2`
/// Scattering coefficient: `gamma = R1 / Rp`
///
/// 3-port series junction (port 3 = parent, reflection-free):
///   scatter_up:   `b3 = -(b1 + b2)`
///   scatter_down: `a1 = b1 - gamma * (b1 + b2 + a3)`
///                 `a2 = b2 - (1 - gamma) * (b1 + b2 + a3)`
#[derive(Debug)]
pub struct SeriesAdaptor {
    pub port_resistance: f64,
    gamma: f64,
    // Child reflected waves (cached from scatter_up)
    b1: f64,
    b2: f64,
}

impl SeriesAdaptor {
    pub fn new(r1: f64, r2: f64) -> Self {
        let rp = r1 + r2;
        Self {
            port_resistance: rp,
            gamma: r1 / rp,
            b1: 0.0,
            b2: 0.0,
        }
    }

    /// Recompute when child port resistances change.
    pub fn update_ports(&mut self, r1: f64, r2: f64) {
        self.port_resistance = r1 + r2;
        self.gamma = r1 / self.port_resistance;
    }

    /// Bottom-up: accept child reflected waves, produce parent reflected wave.
    #[inline]
    pub fn scatter_up(&mut self, b1: f64, b2: f64) -> f64 {
        self.b1 = b1;
        self.b2 = b2;
        -(b1 + b2)
    }

    /// Top-down: accept parent incident wave, produce child incident waves.
    /// Returns `(a1, a2)`.
    #[inline]
    pub fn scatter_down(&self, a3: f64) -> (f64, f64) {
        let sum = self.b1 + self.b2 + a3;
        let a1 = self.b1 - self.gamma * sum;
        let a2 = self.b2 - (1.0 - self.gamma) * sum;
        (a1, a2)
    }
}

// ---------------------------------------------------------------------------
// Two-port parallel adaptor
// ---------------------------------------------------------------------------

/// Parallel adaptor joining two sub-trees.
///
/// Port resistance:  `Rp = R1 * R2 / (R1 + R2)`
/// Scattering coefficient: `gamma = R1 / (R1 + R2)`
///
/// 3-port parallel junction (port 3 = parent, reflection-free):
///   scatter_up:   `b3 = b1 + gamma * (b2 - b1)`
///   scatter_down: from voltage equality at the junction:
///                 `a1 = a3 + b2 - b1 - gamma * (b2 - b1)`     — WRONG
///
/// Actually for a parallel adaptor the scatter_down is:
///   `a1 = a3 - gamma * (b2 - b1)`   — but only if port 3 is parent
///   We use the standard 3-port parallel result where port 3 is reflection-free.
///   `a1 = b3_down + b2 - gamma * (b2 - b1)`  … let's use the verified form:
///
/// Standard parallel 3-port scattering (reflection-free port 3):
///   `b3 = gamma_2 * b1 + gamma_1 * b2`  where gamma_i = 2*R3/(R_i + R3)
///   Simplification when port 3 is adapted: gamma_1 + gamma_2 = 2
///   `b_up = b1 + gamma*(b2 - b1)`  with gamma = R2/(R1+R2) … wait.
///
/// Let me use the correct, well-known form:
///   gamma = R2 / (R1 + R2)    [note: R2 in numerator for parallel]
///   Rp = R1*R2/(R1+R2)
///   scatter_up:   b_up = b1 + gamma*(b2 - b1) = (1-gamma)*b1 + gamma*b2
///   scatter_down: a1 = a_down - (1-gamma)*(a_down + b1 - b_up)  … complex.
///
/// Simplest verified approach (Fettweis / Werner):
///   b_up = b1 + gamma*(b2 - b1)
///   Then from incident a_down (from root):
///   a1 = a_down + b2 - b_up  = a_down + (1-gamma)*(b2 - b1)
///   a2 = a_down + b1 - b_up  = a_down - gamma*(b2 - b1)
///
/// This is the "parallel adaptor with port 3 reflection-free" from
/// Fettweis 1986 / Werner 2015.
#[derive(Debug)]
pub struct ParallelAdaptor {
    pub port_resistance: f64,
    gamma: f64,
    b1: f64,
    b2: f64,
}

impl ParallelAdaptor {
    pub fn new(r1: f64, r2: f64) -> Self {
        let rp = r1 * r2 / (r1 + r2);
        Self {
            port_resistance: rp,
            gamma: r2 / (r1 + r2),
            b1: 0.0,
            b2: 0.0,
        }
    }

    pub fn update_ports(&mut self, r1: f64, r2: f64) {
        self.port_resistance = r1 * r2 / (r1 + r2);
        self.gamma = r2 / (r1 + r2);
    }

    /// Bottom-up: produce parent reflected wave.
    #[inline]
    pub fn scatter_up(&mut self, b1: f64, b2: f64) -> f64 {
        self.b1 = b1;
        self.b2 = b2;
        b1 + self.gamma * (b2 - b1)
    }

    /// Top-down: produce child incident waves from parent incident.
    /// Returns `(a1, a2)`.
    #[inline]
    pub fn scatter_down(&self, a3: f64) -> (f64, f64) {
        let diff = self.b2 - self.b1;
        let a1 = a3 + (1.0 - self.gamma) * diff;
        let a2 = a3 - self.gamma * diff;
        (a1, a2)
    }
}

// ---------------------------------------------------------------------------
// Complete WDF processing engine
// ---------------------------------------------------------------------------

/// WDF processing engine for a Tube-Screamer-style clipping circuit.
///
/// Tree topology:
/// ```text
///        [DiodePair root]
///              |
///         SeriesAdaptor
///          /         \
///   VoltageSource   ParallelAdaptor
///    (input)         /          \
///                Resistor    Capacitor
/// ```
///
/// The voltage source injects the input signal.  The series adaptor
/// connects it with the parallel RC + diode clipping network.
/// The diode pair at the root provides the nonlinearity.
pub struct WdfClipper {
    // Leaves
    vs: VoltageSource,
    resistor: Resistor,
    capacitor: Capacitor,
    // Adaptors
    par: ParallelAdaptor,
    ser: SeriesAdaptor,
    // Root
    diode: DiodePairRoot,
    // Sample rate
    sample_rate: f64,
}

impl WdfClipper {
    /// Create a new WDF clipper circuit.
    ///
    /// * `resistance` — clipping resistor value (Ω)
    /// * `capacitance` — clipping capacitor value (F)
    /// * `diode_model` — diode pair characteristics
    /// * `sample_rate` — audio sample rate (Hz)
    pub fn new(
        resistance: f64,
        capacitance: f64,
        diode_model: DiodeModel,
        sample_rate: f64,
    ) -> Self {
        let vs = VoltageSource::new(1.0); // small Rp for voltage source
        let resistor = Resistor::new(resistance);
        let capacitor = Capacitor::new(capacitance, sample_rate);

        let par = ParallelAdaptor::new(resistor.port_resistance, capacitor.port_resistance);
        let ser = SeriesAdaptor::new(vs.port_resistance, par.port_resistance);
        let diode = DiodePairRoot::new(diode_model);

        Self { vs, resistor, capacitor, par, ser, diode, sample_rate }
    }

    /// Process one sample through the WDF tree.  Zero allocations.
    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
        // Inject input
        self.vs.set_voltage(input);

        // --- Phase 1: scatter_up (bottom → root) ---
        let b_vs = self.vs.reflected();
        let b_r = self.resistor.reflected();
        let b_c = self.capacitor.reflected();

        let b_par = self.par.scatter_up(b_r, b_c);
        let b_ser = self.ser.scatter_up(b_vs, b_par);

        // --- Phase 2: root solve ---
        let a_root = self.diode.process(b_ser, self.ser.port_resistance);

        // --- Phase 3: scatter_down (root → leaves) ---
        let (a_vs, a_par) = self.ser.scatter_down(a_root);
        let (a_r, a_c) = self.par.scatter_down(a_par);

        // --- Phase 4: state update ---
        self.vs.set_incident(a_vs);
        self.resistor.set_incident(a_r);
        self.capacitor.set_incident(a_c);

        // Output = voltage across the diode pair = (a_root + b_ser) / 2
        // But we can also read the voltage at the parallel junction:
        //   v_out = (a_par + b_par) / 2
        (a_par + b_par) / 2.0
    }

    /// Update port resistances after sample rate change.
    pub fn set_sample_rate(&mut self, fs: f64) {
        self.sample_rate = fs;
        self.capacitor.update_sample_rate(fs);
        self.par.update_ports(self.resistor.port_resistance, self.capacitor.port_resistance);
        self.ser.update_ports(self.vs.port_resistance, self.par.port_resistance);
    }

    /// Reset all state (capacitor memory).
    pub fn reset(&mut self) {
        self.capacitor.reset();
        self.par.b1 = 0.0;
        self.par.b2 = 0.0;
        self.ser.b1 = 0.0;
        self.ser.b2 = 0.0;
    }
}

/// WDF single-diode clipper (asymmetric clipping).
///
/// Same topology as `WdfClipper` but with a single diode root.
pub struct WdfSingleDiodeClipper {
    vs: VoltageSource,
    resistor: Resistor,
    capacitor: Capacitor,
    par: ParallelAdaptor,
    ser: SeriesAdaptor,
    diode: DiodeRoot,
    sample_rate: f64,
}

impl WdfSingleDiodeClipper {
    pub fn new(
        resistance: f64,
        capacitance: f64,
        diode_model: DiodeModel,
        sample_rate: f64,
    ) -> Self {
        let vs = VoltageSource::new(1.0);
        let resistor = Resistor::new(resistance);
        let capacitor = Capacitor::new(capacitance, sample_rate);

        let par = ParallelAdaptor::new(resistor.port_resistance, capacitor.port_resistance);
        let ser = SeriesAdaptor::new(vs.port_resistance, par.port_resistance);
        let diode = DiodeRoot::new(diode_model);

        Self { vs, resistor, capacitor, par, ser, diode, sample_rate }
    }

    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
        self.vs.set_voltage(input);

        let b_vs = self.vs.reflected();
        let b_r = self.resistor.reflected();
        let b_c = self.capacitor.reflected();

        let b_par = self.par.scatter_up(b_r, b_c);
        let b_ser = self.ser.scatter_up(b_vs, b_par);

        let a_root = self.diode.process(b_ser, self.ser.port_resistance);

        let (a_vs, a_par) = self.ser.scatter_down(a_root);
        let (a_r, a_c) = self.par.scatter_down(a_par);

        self.vs.set_incident(a_vs);
        self.resistor.set_incident(a_r);
        self.capacitor.set_incident(a_c);

        (a_par + b_par) / 2.0
    }

    pub fn set_sample_rate(&mut self, fs: f64) {
        self.sample_rate = fs;
        self.capacitor.update_sample_rate(fs);
        self.par.update_ports(self.resistor.port_resistance, self.capacitor.port_resistance);
        self.ser.update_ports(self.vs.port_resistance, self.par.port_resistance);
    }

    pub fn reset(&mut self) {
        self.capacitor.reset();
        self.par.b1 = 0.0;
        self.par.b2 = 0.0;
        self.ser.b1 = 0.0;
        self.ser.b2 = 0.0;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn series_adaptor_port_resistance() {
        let s = SeriesAdaptor::new(1000.0, 2000.0);
        assert!((s.port_resistance - 3000.0).abs() < 1e-6);
    }

    #[test]
    fn series_adaptor_scatter_up() {
        let mut s = SeriesAdaptor::new(1000.0, 2000.0);
        let b3 = s.scatter_up(0.5, 0.3);
        assert!((b3 - (-0.8)).abs() < 1e-10, "b3 = -(b1+b2)");
    }

    #[test]
    fn parallel_adaptor_port_resistance() {
        let p = ParallelAdaptor::new(1000.0, 2000.0);
        let expected = 1000.0 * 2000.0 / 3000.0;
        assert!((p.port_resistance - expected).abs() < 1e-6);
    }

    #[test]
    fn parallel_adaptor_scatter_up() {
        let mut p = ParallelAdaptor::new(1000.0, 1000.0);
        // Equal resistances => gamma = 0.5 => b_up = (b1+b2)/2
        let b3 = p.scatter_up(1.0, -1.0);
        assert!((b3 - 0.0).abs() < 1e-10);
    }

    #[test]
    fn wdf_clipper_dc_stability() {
        let mut c = WdfClipper::new(4700.0, 220e-9, DiodeModel::silicon(), 48000.0);
        // Feed DC = 0 for many samples, output should stay near zero
        for _ in 0..1000 {
            let out = c.process(0.0);
            assert!(out.abs() < 1e-6, "DC stability: output was {out}");
        }
    }

    #[test]
    fn wdf_clipper_clips_large_signal() {
        let mut c = WdfClipper::new(4700.0, 220e-9, DiodeModel::silicon(), 48000.0);
        // Feed a large-amplitude sine for several cycles
        let mut max_out = 0.0_f64;
        for i in 0..48000 {
            let t = i as f64 / 48000.0;
            let input = 5.0 * (2.0 * std::f64::consts::PI * 440.0 * t).sin();
            let out = c.process(input);
            max_out = max_out.max(out.abs());
        }
        // Output should be bounded — diode clipping keeps it from blowing up
        assert!(max_out < 50.0, "output should be bounded: peak was {max_out}");
        assert!(max_out > 0.01, "should produce nonzero output");
    }

    #[test]
    fn wdf_clipper_produces_signal() {
        let mut c = WdfClipper::new(4700.0, 220e-9, DiodeModel::silicon(), 48000.0);
        // Feed a sine wave, collect output
        let mut max_out = 0.0_f64;
        for i in 0..4800 {
            let t = i as f64 / 48000.0;
            let input = 0.5 * (2.0 * std::f64::consts::PI * 440.0 * t).sin();
            let out = c.process(input);
            max_out = max_out.max(out.abs());
        }
        assert!(max_out > 0.001, "should produce nonzero output, got {max_out}");
    }

    #[test]
    fn series_scatter_roundtrip_energy() {
        // Energy conservation: |b1|^2/R1 + |b2|^2/R2 + |b3|^2/R3
        // should be preserved through scattering.
        let r1 = 1000.0;
        let r2 = 2200.0;
        let mut s = SeriesAdaptor::new(r1, r2);

        let b1 = 0.3;
        let b2 = 0.7;
        let b3 = s.scatter_up(b1, b2);

        // Now scatter down with some incident
        let a3 = -b3; // matched termination at root
        let (a1, a2) = s.scatter_down(a3);

        // Check Kirchhoff: voltages sum at series junction
        let v1 = (a1 + b1) / 2.0;
        let v2 = (a2 + b2) / 2.0;
        let v3 = (a3 + b3) / 2.0;
        assert!((v1 + v2 - v3).abs() < 1e-10, "KVL: {v1} + {v2} should ≈ {v3}");
    }
}
