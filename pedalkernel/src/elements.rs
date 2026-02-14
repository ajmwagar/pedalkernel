//! WDF circuit elements — one-port leaves and nonlinear roots.
//!
//! One-port (leaf) elements absorb an incident wave `a` and produce a
//! reflected wave `b`.  The port resistance `Rp` is used by parent
//! adaptors to compute scattering coefficients.
//!
//! Nonlinear elements sit at the tree root and use Newton-Raphson
//! iteration bounded to a fixed number of steps for RT safety.

// ---------------------------------------------------------------------------
// One-port elements (leaves)
// ---------------------------------------------------------------------------

/// Ideal resistor — absorbs everything, reflects nothing.
///
/// `b = 0`  (matched termination when Rp == R)
#[derive(Debug, Clone, Copy)]
pub struct Resistor {
    pub port_resistance: f64,
}

impl Resistor {
    pub fn new(resistance: f64) -> Self {
        Self {
            port_resistance: resistance,
        }
    }

    #[inline]
    pub fn reflected(&self) -> f64 {
        0.0
    }

    #[inline]
    pub fn set_incident(&mut self, _a: f64) {
        // resistor has no state to update
    }
}

/// Capacitor — energy-storage element.
///
/// `b[n] = z^{-1} a[n]`  (previous incident becomes current reflected)
/// `Rp = 1 / (2 * fs * C)`
#[derive(Debug, Clone, Copy)]
pub struct Capacitor {
    capacitance: f64,
    pub port_resistance: f64,
    state: f64, // z^{-1} of incident wave
}

impl Capacitor {
    pub fn new(capacitance: f64, sample_rate: f64) -> Self {
        Self {
            capacitance,
            port_resistance: 1.0 / (2.0 * sample_rate * capacitance),
            state: 0.0,
        }
    }

    #[inline]
    pub fn reflected(&self) -> f64 {
        self.state
    }

    /// Call *after* scatter_down delivers the incident wave.
    #[inline]
    pub fn set_incident(&mut self, a: f64) {
        self.state = a;
    }

    pub fn update_sample_rate(&mut self, fs: f64) {
        self.port_resistance = 1.0 / (2.0 * fs * self.capacitance);
    }

    pub fn reset(&mut self) {
        self.state = 0.0;
    }
}

/// Inductor — energy-storage element.
///
/// `b[n] = -z^{-1} a[n]`
/// `Rp = 2 * fs * L`
#[derive(Debug, Clone, Copy)]
pub struct Inductor {
    inductance: f64,
    pub port_resistance: f64,
    state: f64,
}

impl Inductor {
    pub fn new(inductance: f64, sample_rate: f64) -> Self {
        Self {
            inductance,
            port_resistance: 2.0 * sample_rate * inductance,
            state: 0.0,
        }
    }

    #[inline]
    pub fn reflected(&self) -> f64 {
        -self.state
    }

    #[inline]
    pub fn set_incident(&mut self, a: f64) {
        self.state = a;
    }

    pub fn update_sample_rate(&mut self, fs: f64) {
        self.port_resistance = 2.0 * fs * self.inductance;
    }

    pub fn reset(&mut self) {
        self.state = 0.0;
    }
}

/// Ideal voltage source (e.g. input signal injector).
///
/// `b = 2 * Vs - a`  where Vs is the source voltage.
/// Port resistance set to a small value (near-ideal source).
#[derive(Debug, Clone, Copy)]
pub struct VoltageSource {
    voltage: f64,
    pub port_resistance: f64,
}

impl VoltageSource {
    pub fn new(port_resistance: f64) -> Self {
        Self {
            voltage: 0.0,
            port_resistance,
        }
    }

    pub fn set_voltage(&mut self, v: f64) {
        self.voltage = v;
    }

    #[inline]
    pub fn reflected(&self) -> f64 {
        2.0 * self.voltage
    }

    #[inline]
    pub fn set_incident(&mut self, _a: f64) {}
}

// ---------------------------------------------------------------------------
// Nonlinear root elements
// ---------------------------------------------------------------------------

/// Diode model parameters derived from the Shockley equation.
#[derive(Debug, Clone, Copy)]
pub struct DiodeModel {
    /// Saturation current (A). Silicon ≈ 1e-15, Germanium ≈ 1e-6.
    pub is: f64,
    /// Thermal voltage * ideality factor (V).  Vt ≈ 25.85 mV at 20 °C.
    pub n_vt: f64,
}

impl DiodeModel {
    pub fn silicon() -> Self {
        Self {
            is: 2.52e-9,
            n_vt: 1.752 * 25.85e-3,
        }
    }

    pub fn germanium() -> Self {
        Self {
            is: 1e-6,
            n_vt: 1.3 * 25.85e-3,
        }
    }

    pub fn led() -> Self {
        Self {
            is: 2.96e-12,
            n_vt: 1.9 * 25.85e-3,
        }
    }
}

/// Anti-parallel diode pair at the tree root.
///
/// Solves for the reflected wave `b` given the incident wave `a` and
/// port resistance `Rp` using Newton-Raphson on the implicit equation
/// derived from the Shockley model of two anti-parallel diodes.
///
/// Max iterations capped for real-time safety.
#[derive(Debug, Clone, Copy)]
pub struct DiodePairRoot {
    pub model: DiodeModel,
    max_iter: usize,
}

impl DiodePairRoot {
    pub fn new(model: DiodeModel) -> Self {
        Self {
            model,
            max_iter: 16,
        }
    }

    /// Compute reflected wave from incident wave `a` and port resistance `rp`.
    ///
    /// The voltage across the diode pair is `v = (a + b) / 2`.
    /// Current through the pair: `i_d(v) = Is*(exp(v/nVt) - exp(-v/nVt))`.
    /// The WDF constraint: `b = a - 2*Rp*i_d(v)`.
    ///
    /// We solve for `v` using Newton-Raphson on:
    ///   `f(v) = a - 2*v - 2*Rp * Is * (exp(v/nVt) - exp(-v/nVt))`
    ///   `f'(v) = -2 - 2*Rp * Is / nVt * (exp(v/nVt) + exp(-v/nVt))`
    #[inline]
    pub fn process(&self, a: f64, rp: f64) -> f64 {
        let is = self.model.is;
        let nvt = self.model.n_vt;

        // Initial guess: for large |a| the diode is saturated and we can
        // estimate v from the Shockley equation in the forward-bias regime
        // (2*Rp*Is*exp(|v|/nVt) ≈ |a|).  For small |a| the linear
        // approximation v ≈ a/2 is accurate.
        let mut v = if a.abs() > 10.0 * nvt {
            let log_arg = (a.abs() / (2.0 * rp * is)).max(1.0);
            nvt * log_arg.ln() * a.signum()
        } else {
            a * 0.5
        };

        for _ in 0..self.max_iter {
            let x = (v / nvt).clamp(-500.0, 500.0);
            let ev_pos = x.exp();
            let ev_neg = (-x).exp();
            let sinh_term = is * (ev_pos - ev_neg);
            let cosh_term = is * (ev_pos + ev_neg) / nvt;

            let f = a - 2.0 * v - 2.0 * rp * sinh_term;
            let fp = -2.0 - 2.0 * rp * cosh_term;

            let dv = f / fp;
            v -= dv;

            if dv.abs() < 1e-6 {
                break;
            }
        }

        2.0 * v - a
    }
}

/// Single diode at the tree root (asymmetric clipping).
///
/// Solves `i_d(v) = Is*(exp(v/nVt) - 1)` via Newton-Raphson.
#[derive(Debug, Clone, Copy)]
pub struct DiodeRoot {
    pub model: DiodeModel,
    max_iter: usize,
}

impl DiodeRoot {
    pub fn new(model: DiodeModel) -> Self {
        Self {
            model,
            max_iter: 16,
        }
    }

    /// `f(v) = a - 2v - 2*Rp*Is*(exp(v/nVt) - 1)`
    #[inline]
    pub fn process(&self, a: f64, rp: f64) -> f64 {
        let is = self.model.is;
        let nvt = self.model.n_vt;

        // Same analytical initial guess as DiodePairRoot.
        let mut v = if a.abs() > 10.0 * nvt {
            let log_arg = (a.abs() / (2.0 * rp * is)).max(1.0);
            nvt * log_arg.ln() * a.signum()
        } else {
            a * 0.5
        };

        for _ in 0..self.max_iter {
            let x = (v / nvt).clamp(-500.0, 500.0);
            let ev = x.exp();
            let i_d = is * (ev - 1.0);
            let di_d = is * ev / nvt;

            let f = a - 2.0 * v - 2.0 * rp * i_d;
            let fp = -2.0 - 2.0 * rp * di_d;

            let dv = f / fp;
            v -= dv;

            if dv.abs() < 1e-6 {
                break;
            }
        }

        2.0 * v - a
    }
}

/// JFET model parameters for Shockley square-law drain current.
///
/// Models both N-channel and P-channel JFETs with triode and saturation
/// region behavior.
#[derive(Debug, Clone, Copy)]
pub struct JfetModel {
    /// Zero-gate-voltage drain current (A). Typical: 1-20 mA.
    pub idss: f64,
    /// Pinch-off voltage (V). N-channel: -0.5 to -6V, P-channel: +0.5 to +6V.
    pub vp: f64,
    /// Channel-length modulation parameter (1/V). Typical: 0.01-0.1.
    pub lambda: f64,
    /// True for N-channel, false for P-channel.
    pub is_n_channel: bool,
}

impl JfetModel {
    /// 2N5457 N-channel JFET (common, medium Idss).
    pub fn n_2n5457() -> Self {
        Self {
            idss: 5e-3,
            vp: -2.5,
            lambda: 0.02,
            is_n_channel: true,
        }
    }

    /// J201 N-channel JFET (low Idss, low pinch-off).
    pub fn n_j201() -> Self {
        Self {
            idss: 1e-3,
            vp: -1.0,
            lambda: 0.01,
            is_n_channel: true,
        }
    }

    /// 2N5460 P-channel JFET.
    pub fn p_2n5460() -> Self {
        Self {
            idss: 5e-3,
            vp: 2.5,
            lambda: 0.02,
            is_n_channel: false,
        }
    }
}

/// JFET nonlinear root for WDF trees.
///
/// Models the drain-source path as a nonlinear element controlled by
/// an external gate-source voltage Vgs. Uses Newton-Raphson to solve
/// the implicit WDF constraint equation.
///
/// The drain-source current follows the Shockley square-law model:
/// - Triode: `Ids = Idss * [2*(1-Vgs/Vp)*(Vds/|Vp|) - (Vds/Vp)^2]`
/// - Saturation: `Ids = Idss * (1 - Vgs/Vp)^2 * (1 + lambda*|Vds|)`
///
/// Max iterations capped for real-time safety.
#[derive(Debug, Clone, Copy)]
pub struct JfetRoot {
    pub model: JfetModel,
    /// Current gate-source voltage (external control parameter).
    vgs: f64,
    /// Maximum Newton-Raphson iterations (bounded for RT safety).
    max_iter: usize,
}

impl JfetRoot {
    pub fn new(model: JfetModel) -> Self {
        Self {
            model,
            vgs: 0.0,
            max_iter: 16,
        }
    }

    /// Set the gate-source voltage (external control from LFO, bias, etc.)
    #[inline]
    pub fn set_vgs(&mut self, vgs: f64) {
        self.vgs = vgs;
    }

    /// Get current gate-source voltage.
    #[inline]
    pub fn vgs(&self) -> f64 {
        self.vgs
    }

    /// Compute drain current for given Vds at current Vgs.
    ///
    /// Handles triode, saturation, and cutoff regions.
    #[inline]
    pub fn drain_current(&self, vds: f64) -> f64 {
        let vp = self.model.vp;
        let idss = self.model.idss;
        let lambda = self.model.lambda;
        let vgs = self.vgs;

        // Cutoff check: N-channel cuts off when Vgs < Vp (Vp is negative)
        //               P-channel cuts off when Vgs > Vp (Vp is positive)
        if self.model.is_n_channel {
            if vgs <= vp {
                return 0.0;
            }
        } else if vgs >= vp {
            return 0.0;
        }

        // Normalized gate factor: 0 at pinch-off, 1 at Vgs=0
        let vgs_factor = (1.0 - vgs / vp).clamp(0.0, 1.0);

        // Saturation voltage magnitude
        let vdsat = vp.abs() * vgs_factor;

        // Handle symmetric operation for both polarities
        let vds_abs = vds.abs();
        let vds_sign = vds.signum();

        let ids_magnitude = if vds_abs < vdsat {
            // Triode region: Ids = Idss * [2*(1-Vgs/Vp)*(Vds/|Vp|) - (Vds/Vp)^2]
            let vds_norm = vds_abs / vp.abs();
            idss * (2.0 * vgs_factor * vds_norm - vds_norm * vds_norm)
        } else {
            // Saturation region with channel-length modulation
            idss * vgs_factor * vgs_factor * (1.0 + lambda * vds_abs)
        };

        // Current direction follows Vds polarity
        ids_magnitude * vds_sign
    }

    /// Compute derivative of drain current w.r.t. Vds.
    #[inline]
    fn drain_current_derivative(&self, vds: f64) -> f64 {
        let vp = self.model.vp;
        let idss = self.model.idss;
        let lambda = self.model.lambda;
        let vgs = self.vgs;

        // Cutoff: very small conductance to avoid division issues
        if self.model.is_n_channel {
            if vgs <= vp {
                return 1e-12;
            }
        } else if vgs >= vp {
            return 1e-12;
        }

        let vgs_factor = (1.0 - vgs / vp).clamp(0.0, 1.0);
        let vdsat = vp.abs() * vgs_factor;
        let vds_abs = vds.abs();

        if vds_abs < vdsat {
            // Triode region: dIds/dVds = (2*Idss/|Vp|) * [(1-Vgs/Vp) - Vds/|Vp|]
            let vds_norm = vds_abs / vp.abs();
            (2.0 * idss / vp.abs()) * (vgs_factor - vds_norm)
        } else {
            // Saturation region: dIds/dVds = Idss * (1-Vgs/Vp)^2 * lambda
            idss * vgs_factor * vgs_factor * lambda
        }
    }

    /// Compute reflected wave from incident wave `a` and port resistance `rp`.
    ///
    /// WDF constraint: `v = (a + b) / 2`, `i = (a - b) / (2 * Rp)`
    /// JFET: `i = Ids(v, Vgs)`
    ///
    /// Solve: `f(v) = a - 2*v - 2*Rp*Ids(v, Vgs) = 0`
    #[inline]
    pub fn process(&self, a: f64, rp: f64) -> f64 {
        // Initial guess: linear approximation using small-signal conductance
        let vgs_factor = (1.0 - self.vgs / self.model.vp).clamp(0.0, 1.0);
        let gds_approx = (2.0 * self.model.idss / self.model.vp.abs()) * vgs_factor;
        let mut v = if gds_approx > 1e-12 {
            a / (2.0 + 2.0 * rp * gds_approx)
        } else {
            a * 0.5
        };

        for _ in 0..self.max_iter {
            let ids = self.drain_current(v);
            let dids = self.drain_current_derivative(v);

            let f = a - 2.0 * v - 2.0 * rp * ids;
            let fp = -2.0 - 2.0 * rp * dids;

            // Avoid division by zero
            if fp.abs() < 1e-15 {
                break;
            }

            let dv = f / fp;
            v -= dv;

            if dv.abs() < 1e-6 {
                break;
            }
        }

        2.0 * v - a
    }
}

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
            r_dark: 1_000_000.0,    // 1MΩ
            r_light: 1_500.0,        // 1.5kΩ
            gamma: 0.75,
            tau_fast_rise: 0.002,    // 2ms
            tau_fast_fall: 0.008,    // 8ms
            tau_slow_rise: 0.020,    // 20ms
            tau_slow_fall: 0.080,    // 80ms
            slow_weight: 0.3,
        }
    }

    /// VTL5C1 - Faster response, used in some compressors.
    pub fn vtl5c1() -> Self {
        Self {
            r_dark: 600_000.0,       // 600kΩ
            r_light: 600.0,          // 600Ω
            gamma: 0.8,
            tau_fast_rise: 0.001,    // 1ms
            tau_fast_fall: 0.005,    // 5ms
            tau_slow_rise: 0.010,    // 10ms
            tau_slow_fall: 0.050,    // 50ms
            slow_weight: 0.25,
        }
    }

    /// NSL-32 - Common optocoupler, moderate response.
    pub fn nsl32() -> Self {
        Self {
            r_dark: 2_000_000.0,     // 2MΩ
            r_light: 2_000.0,        // 2kΩ
            gamma: 0.7,
            tau_fast_rise: 0.003,    // 3ms
            tau_fast_fall: 0.010,    // 10ms
            tau_slow_rise: 0.025,    // 25ms
            tau_slow_fall: 0.100,    // 100ms
            slow_weight: 0.35,
        }
    }
}

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
    pub port_resistance: f64,
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
            port_resistance: model.r_dark,
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

    /// Update sample rate and recompute coefficients.
    pub fn update_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
        self.update_coefficients();
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
            self.port_resistance = log_r.exp().clamp(self.model.r_light, self.model.r_dark);
        } else {
            self.port_resistance = self.model.r_dark;
        }
    }

    /// Get current LED drive level (for monitoring).
    pub fn effective_light_level(&self) -> f64 {
        let w = self.model.slow_weight;
        (1.0 - w) * self.state_fast + w * self.state_slow
    }

    /// Reflected wave (same as ideal resistor: b = 0 when matched).
    #[inline]
    pub fn reflected(&self) -> f64 {
        0.0
    }

    /// Set incident wave (no state update needed for resistor).
    #[inline]
    pub fn set_incident(&mut self, _a: f64) {
        // Resistor has no reactive state
    }

    /// Reset to fully dark state.
    pub fn reset(&mut self) {
        self.state_fast = 0.0;
        self.state_slow = 0.0;
        self.port_resistance = self.model.r_dark;
    }
}

// ---------------------------------------------------------------------------
// LFO (Low Frequency Oscillator)
// ---------------------------------------------------------------------------

/// LFO waveform shapes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LfoWaveform {
    /// Smooth sine wave - classic tremolo sound.
    Sine,
    /// Linear triangle wave - similar to sine but brighter.
    Triangle,
    /// Hard square wave - choppy, staccato effect.
    Square,
    /// Rising sawtooth - asymmetric, "ramping" feel.
    SawUp,
    /// Falling sawtooth - reverse ramp.
    SawDown,
    /// Random levels held for each cycle - sample-and-hold.
    SampleAndHold,
}

/// Low Frequency Oscillator for modulation effects.
///
/// Generates periodic control signals for tremolo, vibrato, phaser, etc.
/// Uses a phase accumulator for efficient, RT-safe operation.
#[derive(Debug, Clone)]
pub struct Lfo {
    /// Current phase (0.0 - 1.0).
    phase: f64,
    /// Frequency in Hz.
    frequency: f64,
    /// Modulation depth (0.0 - 1.0).
    depth: f64,
    /// Waveform shape.
    waveform: LfoWaveform,
    /// Sample rate for phase increment calculation.
    sample_rate: f64,
    /// Phase increment per sample.
    phase_inc: f64,
    /// Held value for sample-and-hold mode.
    sh_value: f64,
    /// Simple RNG state for sample-and-hold.
    rng_state: u32,
}

impl Lfo {
    /// Create a new LFO with the given waveform and sample rate.
    pub fn new(waveform: LfoWaveform, sample_rate: f64) -> Self {
        let frequency = 5.0; // Default 5 Hz
        let phase_inc = frequency / sample_rate;
        Self {
            phase: 0.0,
            frequency,
            depth: 1.0,
            waveform,
            sample_rate,
            phase_inc,
            sh_value: 0.0,
            rng_state: 12345,
        }
    }

    /// Set the LFO frequency in Hz (typically 0.1 - 20 Hz).
    #[inline]
    pub fn set_rate(&mut self, hz: f64) {
        self.frequency = hz.max(0.01);
        self.phase_inc = self.frequency / self.sample_rate;
    }

    /// Get the current frequency in Hz.
    pub fn rate(&self) -> f64 {
        self.frequency
    }

    /// Set modulation depth (0.0 = no modulation, 1.0 = full depth).
    #[inline]
    pub fn set_depth(&mut self, depth: f64) {
        self.depth = depth.clamp(0.0, 1.0);
    }

    /// Get the current depth.
    pub fn depth(&self) -> f64 {
        self.depth
    }

    /// Set the waveform shape.
    pub fn set_waveform(&mut self, waveform: LfoWaveform) {
        self.waveform = waveform;
    }

    /// Get the current waveform.
    pub fn waveform(&self) -> LfoWaveform {
        self.waveform
    }

    /// Update sample rate (call when audio engine sample rate changes).
    pub fn update_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
        self.phase_inc = self.frequency / sample_rate;
    }

    /// Simple fast PRNG for sample-and-hold (xorshift).
    #[inline]
    fn next_random(&mut self) -> f64 {
        let mut x = self.rng_state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng_state = x;
        // Convert to 0.0 - 1.0 range
        (x as f64) / (u32::MAX as f64)
    }

    /// Generate raw waveform value from phase (returns -1.0 to 1.0).
    #[inline]
    fn raw_waveform(&mut self, phase: f64) -> f64 {
        match self.waveform {
            LfoWaveform::Sine => {
                (phase * std::f64::consts::TAU).sin()
            }
            LfoWaveform::Triangle => {
                // Triangle: rises from -1 to 1 in first half, falls in second
                if phase < 0.5 {
                    4.0 * phase - 1.0
                } else {
                    3.0 - 4.0 * phase
                }
            }
            LfoWaveform::Square => {
                if phase < 0.5 {
                    1.0
                } else {
                    -1.0
                }
            }
            LfoWaveform::SawUp => {
                // Rising saw: -1 at phase 0, +1 at phase 1
                2.0 * phase - 1.0
            }
            LfoWaveform::SawDown => {
                // Falling saw: +1 at phase 0, -1 at phase 1
                1.0 - 2.0 * phase
            }
            LfoWaveform::SampleAndHold => {
                // Value is held in sh_value, updated when phase wraps
                self.sh_value * 2.0 - 1.0 // Convert 0-1 to -1 to 1
            }
        }
    }

    /// Advance the LFO by one sample and return the output value.
    ///
    /// Returns a value in the range [-depth, +depth].
    /// For unipolar output (0 to depth), use `tick_unipolar()`.
    #[inline]
    pub fn tick(&mut self) -> f64 {
        let value = self.raw_waveform(self.phase);

        // Advance phase
        self.phase += self.phase_inc;

        // Handle phase wrap
        if self.phase >= 1.0 {
            self.phase -= 1.0;
            // Update sample-and-hold on wrap
            if self.waveform == LfoWaveform::SampleAndHold {
                self.sh_value = self.next_random();
            }
        }

        value * self.depth
    }

    /// Advance the LFO and return unipolar output (0.0 to depth).
    ///
    /// Useful for tremolo depth where 0 = no cut, depth = full cut.
    #[inline]
    pub fn tick_unipolar(&mut self) -> f64 {
        (self.tick() + self.depth) * 0.5
    }

    /// Get current phase (0.0 - 1.0), useful for phase displays.
    pub fn phase(&self) -> f64 {
        self.phase
    }

    /// Set phase directly (0.0 - 1.0), useful for sync.
    pub fn set_phase(&mut self, phase: f64) {
        self.phase = phase.rem_euclid(1.0);
    }

    /// Reset LFO to initial state.
    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.sh_value = 0.0;
        self.rng_state = 12345;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resistor_reflects_zero() {
        let r = Resistor::new(1000.0);
        assert_eq!(r.reflected(), 0.0);
    }

    #[test]
    fn capacitor_port_resistance() {
        let c = Capacitor::new(220e-9, 48000.0);
        let expected = 1.0 / (2.0 * 48000.0 * 220e-9);
        assert!((c.port_resistance - expected).abs() < 1e-3);
    }

    #[test]
    fn capacitor_reflects_previous_incident() {
        let mut c = Capacitor::new(220e-9, 48000.0);
        assert_eq!(c.reflected(), 0.0);
        c.set_incident(0.5);
        assert_eq!(c.reflected(), 0.5);
    }

    #[test]
    fn inductor_port_resistance() {
        let l = Inductor::new(0.1, 48000.0);
        let expected = 2.0 * 48000.0 * 0.1;
        assert!((l.port_resistance - expected).abs() < 1e-3);
    }

    #[test]
    fn inductor_reflects_negated_previous() {
        let mut l = Inductor::new(0.1, 48000.0);
        l.set_incident(0.5);
        assert_eq!(l.reflected(), -0.5);
    }

    #[test]
    fn voltage_source_reflection() {
        let mut vs = VoltageSource::new(1.0);
        vs.set_voltage(1.0);
        assert_eq!(vs.reflected(), 2.0);
    }

    #[test]
    fn diode_pair_zero_input() {
        let dp = DiodePairRoot::new(DiodeModel::silicon());
        let b = dp.process(0.0, 1000.0);
        assert!(
            b.abs() < 1e-6,
            "symmetric diode pair should reflect ~0 for zero input"
        );
    }

    #[test]
    fn diode_pair_clips() {
        let dp = DiodePairRoot::new(DiodeModel::silicon());
        let b_small = dp.process(0.01, 1000.0);
        let b_large = dp.process(10.0, 1000.0);
        // The diode voltage v = (a + b) / 2 is what gets clipped.
        let v_small = (0.01 + b_small) / 2.0;
        let v_large = (10.0 + b_large) / 2.0;
        // Diode voltage should be bounded (silicon: < 2V regardless of input).
        assert!(
            v_large.abs() < 2.0,
            "diode voltage should be bounded: {v_large}"
        );
        // Relative clipping: v/a ratio should shrink for larger inputs.
        assert!(
            v_large.abs() / 10.0 < v_small.abs() / 0.01,
            "large input should be clipped harder: \
             v_small/a_small={:.4}, v_large/a_large={:.4}",
            v_small.abs() / 0.01,
            v_large.abs() / 10.0,
        );
    }

    #[test]
    fn single_diode_asymmetric() {
        let d = DiodeRoot::new(DiodeModel::silicon());
        let b_pos = d.process(1.0, 1000.0);
        let b_neg = d.process(-1.0, 1000.0);
        // Single diode should produce different reflections for + vs -
        assert!(
            (b_pos - b_neg).abs() > 1e-10,
            "should be asymmetric: b+={b_pos}, b-={b_neg}"
        );
    }

    #[test]
    fn jfet_cutoff_at_pinchoff() {
        let mut jfet = JfetRoot::new(JfetModel::n_2n5457());
        // Vp = -2.5V, so Vgs = -3.0V should be in cutoff
        jfet.set_vgs(-3.0);
        let ids = jfet.drain_current(5.0);
        assert!(ids.abs() < 1e-9, "should be cutoff: ids={ids}");
    }

    #[test]
    fn jfet_triode_region() {
        let jfet = JfetRoot::new(JfetModel::n_2n5457());
        // Vgs=0, small Vds -> triode region, current increases with Vds
        let ids_small = jfet.drain_current(0.5);
        let ids_larger = jfet.drain_current(1.0);
        assert!(
            ids_larger > ids_small,
            "triode: current should increase with Vds: {ids_small} -> {ids_larger}"
        );
    }

    #[test]
    fn jfet_saturation_region() {
        let jfet = JfetRoot::new(JfetModel::n_2n5457());
        // Vgs=0, Vds large -> saturation, current nearly constant
        let ids_5v = jfet.drain_current(5.0);
        let ids_10v = jfet.drain_current(10.0);
        // In saturation, current changes only due to lambda (channel-length mod)
        let ratio = ids_10v / ids_5v;
        assert!(
            (0.9..1.2).contains(&ratio),
            "saturation: current should be nearly constant, ratio={ratio}"
        );
    }

    #[test]
    fn jfet_vgs_modulates_current() {
        let mut jfet = JfetRoot::new(JfetModel::n_2n5457());
        jfet.set_vgs(0.0);
        let ids_0 = jfet.drain_current(5.0);

        jfet.set_vgs(-1.0);
        let ids_1 = jfet.drain_current(5.0);

        jfet.set_vgs(-2.0);
        let ids_2 = jfet.drain_current(5.0);

        assert!(
            ids_0 > ids_1 && ids_1 > ids_2,
            "more negative Vgs should reduce current: {ids_0} > {ids_1} > {ids_2}"
        );
    }

    #[test]
    fn jfet_newton_converges() {
        let jfet = JfetRoot::new(JfetModel::n_2n5457());
        // Process various inputs, should not produce NaN or Inf
        for a in [-5.0, -1.0, 0.0, 1.0, 5.0] {
            let b = jfet.process(a, 1000.0);
            assert!(b.is_finite(), "should converge for a={a}, got b={b}");
        }
    }

    #[test]
    fn jfet_wdf_constraint_satisfied() {
        let jfet = JfetRoot::new(JfetModel::n_2n5457());
        let rp = 1000.0;
        let a = 2.0;
        let b = jfet.process(a, rp);

        // Verify WDF equation: a - 2*v - 2*Rp*Ids(v) should be ~0 at solution
        let v = (a + b) / 2.0;
        let ids = jfet.drain_current(v);
        let residual = a - 2.0 * v - 2.0 * rp * ids;

        assert!(
            residual.abs() < 1e-4,
            "WDF equation not satisfied: residual={residual}, v={v}, ids={ids}"
        );
    }

    #[test]
    fn jfet_p_channel_polarity() {
        let mut jfet = JfetRoot::new(JfetModel::p_2n5460());
        // P-channel: Vgs > Vp (positive) causes cutoff
        jfet.set_vgs(3.0); // Vp = 2.5V, so this is cutoff
        let ids_cutoff = jfet.drain_current(-5.0);
        assert!(ids_cutoff.abs() < 1e-9, "P-channel should be cutoff");

        jfet.set_vgs(0.0); // Conducting
        let ids_on = jfet.drain_current(-5.0);
        assert!(ids_on.abs() > 1e-6, "P-channel should conduct at Vgs=0");
    }

    // -------------------------------------------------------------------------
    // Photocoupler tests
    // -------------------------------------------------------------------------

    #[test]
    fn photocoupler_dark_state() {
        let pc = Photocoupler::new(PhotocouplerModel::vtl5c3(), 48000.0);
        assert!(
            (pc.port_resistance - 1_000_000.0).abs() < 1.0,
            "dark state should be R_dark: {}",
            pc.port_resistance
        );
    }

    #[test]
    fn photocoupler_full_illumination() {
        let mut pc = Photocoupler::new(PhotocouplerModel::vtl5c3(), 48000.0);
        // Run for many samples to let state settle
        for _ in 0..48000 {
            // 1 second at 48kHz
            pc.set_led_drive(1.0);
        }
        // Should be near R_light (1.5kΩ)
        assert!(
            pc.port_resistance < 2000.0,
            "full illumination should approach R_light: {}",
            pc.port_resistance
        );
    }

    #[test]
    fn photocoupler_asymmetric_response() {
        let mut pc = Photocoupler::new(PhotocouplerModel::vtl5c3(), 48000.0);

        // Rise: drive to full for 100ms
        let samples_100ms = (48000.0 * 0.1) as usize;
        for _ in 0..samples_100ms {
            pc.set_led_drive(1.0);
        }
        let r_after_rise = pc.port_resistance;
        let level_after_rise = pc.effective_light_level();

        // Fall: drive to zero for same duration
        for _ in 0..samples_100ms {
            pc.set_led_drive(0.0);
        }
        let level_after_fall = pc.effective_light_level();

        // Rise should reach higher level than fall decays (asymmetric)
        // After 100ms rise from 0, then 100ms fall, should not be back to 0
        assert!(
            level_after_fall > 0.01,
            "fall should be slower than rise: level_after_rise={level_after_rise}, level_after_fall={level_after_fall}"
        );
        assert!(
            r_after_rise < 100_000.0,
            "should have decreased from dark during rise: {}",
            r_after_rise
        );
    }

    #[test]
    fn photocoupler_reset() {
        let mut pc = Photocoupler::new(PhotocouplerModel::vtl5c3(), 48000.0);

        // Drive to some state
        for _ in 0..10000 {
            pc.set_led_drive(0.8);
        }
        assert!(pc.port_resistance < 500_000.0);

        // Reset should return to dark state
        pc.reset();
        assert!(
            (pc.port_resistance - 1_000_000.0).abs() < 1.0,
            "reset should return to R_dark: {}",
            pc.port_resistance
        );
        assert!(
            pc.effective_light_level() < 1e-9,
            "reset should zero state: {}",
            pc.effective_light_level()
        );
    }

    #[test]
    fn photocoupler_clamps_drive() {
        let mut pc = Photocoupler::new(PhotocouplerModel::vtl5c3(), 48000.0);
        // Overdrive should be clamped
        pc.set_led_drive(2.0);
        let level = pc.effective_light_level();
        assert!(level <= 1.0, "should clamp overdrive: {level}");

        pc.reset();
        pc.set_led_drive(-0.5);
        let level = pc.effective_light_level();
        assert!(level >= 0.0, "should clamp negative: {level}");
    }

    #[test]
    fn photocoupler_model_presets() {
        // Verify preset models have reasonable values
        let vtl5c3 = PhotocouplerModel::vtl5c3();
        assert!(vtl5c3.r_dark > vtl5c3.r_light);
        assert!(vtl5c3.tau_fast_fall > vtl5c3.tau_fast_rise);
        assert!(vtl5c3.tau_slow_fall > vtl5c3.tau_slow_rise);

        let vtl5c1 = PhotocouplerModel::vtl5c1();
        assert!(vtl5c1.r_dark > vtl5c1.r_light);
        assert!(vtl5c1.tau_fast_rise < vtl5c3.tau_fast_rise); // VTL5C1 is faster

        let nsl32 = PhotocouplerModel::nsl32();
        assert!(nsl32.r_dark > nsl32.r_light);
    }

    #[test]
    fn photocoupler_sample_rate_update() {
        let mut pc = Photocoupler::new(PhotocouplerModel::vtl5c3(), 48000.0);
        let alpha_before = pc.alpha_fast_rise;

        pc.update_sample_rate(96000.0);
        let alpha_after = pc.alpha_fast_rise;

        // Higher sample rate = smaller dt = alpha closer to 1
        assert!(
            alpha_after > alpha_before,
            "higher sample rate should increase alpha: {alpha_before} -> {alpha_after}"
        );
    }

    #[test]
    fn photocoupler_wdf_interface() {
        let mut pc = Photocoupler::new(PhotocouplerModel::vtl5c3(), 48000.0);
        // Resistor-like element: reflected wave is 0 when matched
        assert_eq!(pc.reflected(), 0.0);
        // set_incident should not panic
        pc.set_incident(1.0);
        assert_eq!(pc.reflected(), 0.0);
    }

    // -------------------------------------------------------------------------
    // LFO tests
    // -------------------------------------------------------------------------

    #[test]
    fn lfo_sine_range() {
        let mut lfo = Lfo::new(LfoWaveform::Sine, 48000.0);
        lfo.set_rate(10.0);
        lfo.set_depth(1.0);

        // Run for one full cycle
        let samples = (48000.0 / 10.0) as usize;
        let mut min_val = f64::MAX;
        let mut max_val = f64::MIN;

        for _ in 0..samples {
            let v = lfo.tick();
            min_val = min_val.min(v);
            max_val = max_val.max(v);
        }

        assert!(min_val >= -1.01, "sine min should be >= -1: {min_val}");
        assert!(max_val <= 1.01, "sine max should be <= 1: {max_val}");
        assert!(min_val < -0.9, "sine should reach near -1: {min_val}");
        assert!(max_val > 0.9, "sine should reach near +1: {max_val}");
    }

    #[test]
    fn lfo_triangle_symmetry() {
        let mut lfo = Lfo::new(LfoWaveform::Triangle, 48000.0);
        lfo.set_rate(10.0);

        let samples = (48000.0 / 10.0) as usize;
        let mut values = Vec::with_capacity(samples);

        for _ in 0..samples {
            values.push(lfo.tick());
        }

        // Triangle should be symmetric: mean should be ~0
        let mean: f64 = values.iter().sum::<f64>() / values.len() as f64;
        assert!(mean.abs() < 0.01, "triangle mean should be ~0: {mean}");
    }

    #[test]
    fn lfo_square_values() {
        let mut lfo = Lfo::new(LfoWaveform::Square, 48000.0);
        lfo.set_rate(10.0);

        let samples = (48000.0 / 10.0) as usize;
        let mut pos_count = 0;

        for _ in 0..samples {
            let v = lfo.tick();
            if v > 0.5 {
                pos_count += 1;
            }
        }

        // Square wave: ~50% positive, ~50% negative
        let ratio = pos_count as f64 / samples as f64;
        assert!(
            (0.45..0.55).contains(&ratio),
            "square should be ~50% positive: {ratio}"
        );
    }

    #[test]
    fn lfo_depth_scaling() {
        let mut lfo = Lfo::new(LfoWaveform::Sine, 48000.0);
        lfo.set_rate(10.0);
        lfo.set_depth(0.5);

        let samples = (48000.0 / 10.0) as usize;
        let mut max_val = 0.0f64;

        for _ in 0..samples {
            max_val = max_val.max(lfo.tick().abs());
        }

        assert!(
            max_val <= 0.51,
            "depth 0.5 should limit output to 0.5: {max_val}"
        );
        assert!(
            max_val > 0.45,
            "depth 0.5 should reach near 0.5: {max_val}"
        );
    }

    #[test]
    fn lfo_rate_affects_frequency() {
        let mut lfo_slow = Lfo::new(LfoWaveform::Square, 48000.0);
        let mut lfo_fast = Lfo::new(LfoWaveform::Square, 48000.0);
        lfo_slow.set_rate(1.0);
        lfo_fast.set_rate(10.0);

        // Count zero crossings over 1 second
        let samples = 48000;
        let mut slow_crossings = 0;
        let mut fast_crossings = 0;
        let mut prev_slow = lfo_slow.tick();
        let mut prev_fast = lfo_fast.tick();

        for _ in 1..samples {
            let s = lfo_slow.tick();
            let f = lfo_fast.tick();
            if s.signum() != prev_slow.signum() {
                slow_crossings += 1;
            }
            if f.signum() != prev_fast.signum() {
                fast_crossings += 1;
            }
            prev_slow = s;
            prev_fast = f;
        }

        // Fast LFO should have ~10x more crossings
        assert!(
            fast_crossings > slow_crossings * 5,
            "10Hz should have more crossings than 1Hz: {fast_crossings} vs {slow_crossings}"
        );
    }

    #[test]
    fn lfo_unipolar_range() {
        let mut lfo = Lfo::new(LfoWaveform::Sine, 48000.0);
        lfo.set_rate(10.0);
        lfo.set_depth(1.0);

        let samples = (48000.0 / 10.0) as usize;
        let mut min_val = f64::MAX;
        let mut max_val = f64::MIN;

        for _ in 0..samples {
            let v = lfo.tick_unipolar();
            min_val = min_val.min(v);
            max_val = max_val.max(v);
        }

        assert!(min_val >= -0.01, "unipolar min should be >= 0: {min_val}");
        assert!(max_val <= 1.01, "unipolar max should be <= 1: {max_val}");
    }

    #[test]
    fn lfo_reset() {
        let mut lfo = Lfo::new(LfoWaveform::Sine, 48000.0);
        lfo.set_rate(10.0);

        // Advance some samples
        for _ in 0..1000 {
            lfo.tick();
        }
        assert!(lfo.phase() > 0.1);

        // Reset should return to start
        lfo.reset();
        assert!(lfo.phase() < 0.001, "reset should zero phase: {}", lfo.phase());
    }

    #[test]
    fn lfo_sample_and_hold() {
        let mut lfo = Lfo::new(LfoWaveform::SampleAndHold, 48000.0);
        lfo.set_rate(10.0);

        // Run through one cycle - value should stay constant
        let samples_per_cycle = (48000.0 / 10.0) as usize;

        // Skip first sample which triggers initial value
        lfo.tick();
        let held_value = lfo.tick();

        // Values should be constant until phase wraps
        for _ in 2..(samples_per_cycle - 10) {
            let v = lfo.tick();
            assert!(
                (v - held_value).abs() < 1e-10,
                "S&H should hold value: {v} vs {held_value}"
            );
        }
    }

    #[test]
    fn lfo_saw_waveforms() {
        let mut lfo_up = Lfo::new(LfoWaveform::SawUp, 48000.0);
        let mut lfo_down = Lfo::new(LfoWaveform::SawDown, 48000.0);
        lfo_up.set_rate(10.0);
        lfo_down.set_rate(10.0);

        // First sample of saw up should be near -1 (start of ramp)
        let up_start = lfo_up.tick();
        let down_start = lfo_down.tick();

        assert!(up_start < -0.9, "saw up should start near -1: {up_start}");
        assert!(down_start > 0.9, "saw down should start near +1: {down_start}");
    }
}
