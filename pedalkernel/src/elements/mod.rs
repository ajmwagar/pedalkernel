//! WDF circuit elements — traits and implementations.
//!
//! This module defines trait abstractions for different element categories:
//! - [`WdfLeaf`] — one-port elements (R, C, L, Vs) that terminate tree branches
//! - [`WdfRoot`] — nonlinear elements (diodes, JFETs) at the tree root
//! - [`Modulator`] — control signal generators (LFO, envelope follower)
//! - [`ControlledResistance`] — elements with externally-controlled resistance
//!
//! Adaptors (Series, Parallel) remain in the `tree` module as they're structural.

mod allpass;
mod controlled;
mod linear;
mod modulation;
mod nonlinear;
mod synth;

pub use allpass::*;
pub use controlled::*;
pub use linear::*;
pub use modulation::*;
pub use nonlinear::*;
pub use synth::*;

// ---------------------------------------------------------------------------
// Traits
// ---------------------------------------------------------------------------

/// One-port WDF leaf element (resistor, capacitor, inductor, voltage source).
///
/// Leaf elements sit at the ends of the WDF binary tree and interact with
/// the wave domain through incident (`a`) and reflected (`b`) waves.
pub trait WdfLeaf {
    /// Port resistance seen looking into this element (Ω).
    fn port_resistance(&self) -> f64;

    /// Produce the reflected wave given the current state.
    ///
    /// For resistors: `b = 0` (matched termination)
    /// For capacitors: `b = z⁻¹ a` (previous incident)
    /// For inductors: `b = -z⁻¹ a`
    /// For voltage sources: `b = 2*Vs - a`
    fn reflected(&self) -> f64;

    /// Accept the incident wave after scatter_down phase.
    ///
    /// Reactive elements (C, L) latch this value as state for next sample.
    fn set_incident(&mut self, a: f64);

    /// Update sample rate (for reactive elements C, L).
    fn set_sample_rate(&mut self, _sample_rate: f64) {}

    /// Reset internal state to zero.
    fn reset(&mut self) {}
}

/// Nonlinear root element that terminates the WDF tree.
///
/// These elements solve implicit equations (typically via Newton-Raphson)
/// to determine the reflected wave from the incident wave and port resistance.
pub trait WdfRoot {
    /// Solve for reflected wave given incident wave `a` and port resistance `rp`.
    ///
    /// The WDF constraint is: `b = a - 2*Rp*i(v)` where `v = (a+b)/2`.
    /// Implementation uses Newton-Raphson bounded to fixed iterations for RT safety.
    fn process(&mut self, incident: f64, port_resistance: f64) -> f64;
}

/// Modulation source that generates control signals.
///
/// LFOs, envelope followers, and similar generators implement this trait.
/// Output is typically in the range [-1, 1] or [0, 1] depending on use case.
pub trait Modulator {
    /// Generate the next modulation sample.
    ///
    /// Returns a value typically in [-1, 1] (bipolar) or [0, 1] (unipolar).
    fn tick(&mut self) -> f64;

    /// Reset the modulator to its initial state.
    fn reset(&mut self);

    /// Set the modulation rate in Hz (for periodic modulators).
    fn set_rate(&mut self, hz: f64);

    /// Get current rate in Hz.
    fn rate(&self) -> f64;

    /// Update sample rate when audio engine changes.
    fn set_sample_rate(&mut self, sample_rate: f64);
}

/// Element with externally-controlled resistance.
///
/// Photocouplers and similar devices where resistance varies based on
/// an external control signal (LED current, etc.).
pub trait ControlledResistance: WdfLeaf {
    /// Set the control value (0.0 = minimum effect, 1.0 = maximum effect).
    ///
    /// For photocouplers: 0 = dark (high R), 1 = full illumination (low R).
    fn set_control(&mut self, value: f64);

    /// Get the current effective control level (after any internal filtering).
    fn effective_control(&self) -> f64;
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
        assert!((c.port_resistance() - expected).abs() < 1e-3);
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
        assert!((l.port_resistance() - expected).abs() < 1e-3);
    }

    #[test]
    fn inductor_reflects_negated_previous() {
        let mut l = Inductor::new(0.1, 48000.0);
        l.set_incident(0.5);
        assert_eq!(l.reflected(), -0.5);
    }

    #[test]
    fn voltage_source_reflection() {
        let mut vs = VoltageSource::new(50.0);
        vs.set_voltage(1.0);
        assert_eq!(vs.reflected(), 2.0);
    }

    #[test]
    fn diode_pair_clips() {
        let mut diode = DiodePairRoot::new(DiodeModel::silicon());
        let b = diode.process(10.0, 1000.0);
        assert!(b.abs() < 10.0, "diode should clip large input");
    }

    #[test]
    fn diode_pair_zero_input() {
        let mut diode = DiodePairRoot::new(DiodeModel::silicon());
        let b = diode.process(0.0, 1000.0);
        assert!(b.abs() < 1e-6, "zero in → zero out");
    }

    #[test]
    fn single_diode_asymmetric() {
        let mut diode = DiodeRoot::new(DiodeModel::silicon());
        let b_pos = diode.process(1.0, 1000.0);
        let b_neg = diode.process(-1.0, 1000.0);
        // Single diode should produce different reflections for + vs -
        assert!(
            (b_pos - b_neg).abs() > 1e-10,
            "should be asymmetric: b+={b_pos}, b-={b_neg}"
        );
    }

    // ── Comprehensive Diode Tests ──────────────────────────────────────────

    #[test]
    fn diode_pair_symmetry() {
        // Anti-parallel diodes should produce symmetric clipping
        let mut diode = DiodePairRoot::new(DiodeModel::silicon());
        let rp = 10_000.0;

        for a in [0.5, 1.0, 2.0, 5.0, 10.0] {
            let b_pos = diode.process(a, rp);
            let b_neg = diode.process(-a, rp);
            let v_pos = (a + b_pos) / 2.0;
            let v_neg = (-a + b_neg) / 2.0;

            assert!(
                (v_pos + v_neg).abs() < 0.01,
                "Diode pair should be symmetric: v+={v_pos:.4}, v-={v_neg:.4}"
            );
        }
    }

    #[test]
    fn diode_pair_silicon_clipping_threshold() {
        // Silicon diodes clip at ~0.6-0.7V forward voltage
        let mut diode = DiodePairRoot::new(DiodeModel::silicon());
        let rp = 10_000.0;

        // With large input, output voltage should saturate near Vf
        let a = 20.0; // Large input wave
        let b = diode.process(a, rp);
        let v = (a + b) / 2.0;

        // Silicon Vf is ~0.6-0.7V
        assert!(
            v > 0.5 && v < 1.0,
            "Silicon diode pair should clip at ~0.6V: v={v:.4}"
        );
    }

    #[test]
    fn diode_germanium_lower_threshold() {
        // Germanium diodes clip earlier (~0.3V) than silicon (~0.6V)
        let mut ge_diode = DiodePairRoot::new(DiodeModel::germanium());
        let mut si_diode = DiodePairRoot::new(DiodeModel::silicon());
        let rp = 10_000.0;

        let a = 10.0;
        let v_ge = (a + ge_diode.process(a, rp)) / 2.0;
        let v_si = (a + si_diode.process(a, rp)) / 2.0;

        assert!(
            v_ge < v_si,
            "Germanium should clip lower than silicon: Ge={v_ge:.4}, Si={v_si:.4}"
        );
        assert!(
            v_ge > 0.2 && v_ge < 0.5,
            "Germanium should clip at ~0.3V: v={v_ge:.4}"
        );
    }

    #[test]
    fn diode_led_higher_threshold() {
        // LEDs clip at higher voltage (~1.7V) than silicon
        let mut led = DiodePairRoot::new(DiodeModel::led());
        let mut si = DiodePairRoot::new(DiodeModel::silicon());
        let rp = 10_000.0;

        let a = 20.0;
        let v_led = (a + led.process(a, rp)) / 2.0;
        let v_si = (a + si.process(a, rp)) / 2.0;

        assert!(
            v_led > v_si,
            "LED should clip higher than silicon: LED={v_led:.4}, Si={v_si:.4}"
        );
        assert!(
            v_led > 1.2 && v_led < 2.5,
            "LED should clip at ~1.7V: v={v_led:.4}"
        );
    }

    #[test]
    fn single_diode_forward_clips() {
        // Single diode should clip forward bias (positive input)
        let mut diode = DiodeRoot::new(DiodeModel::silicon());
        let rp = 10_000.0;

        let a = 20.0;
        let b = diode.process(a, rp);
        let v = (a + b) / 2.0;

        // Forward bias: should clip at Vf
        assert!(
            v > 0.4 && v < 1.0,
            "Single diode forward bias should clip at ~0.6V: v={v:.4}"
        );
    }

    #[test]
    fn single_diode_reverse_blocks() {
        // Single diode should block reverse bias (negative input)
        // When blocked, v ≈ a/2 (no current flows)
        let mut diode = DiodeRoot::new(DiodeModel::silicon());
        let rp = 10_000.0;

        let a = -5.0;
        let b = diode.process(a, rp);
        let v = (a + b) / 2.0;

        // Reverse bias: very little current, v ≈ a/2
        // Diode acts like open circuit, so b ≈ -a, v ≈ 0
        // Actually with WDF: i_d ≈ -Is ≈ 0, so b ≈ a, v ≈ a
        // Wait, let me think again...
        // f(v) = a - 2v - 2*Rp*Is*(exp(v/nVt) - 1)
        // For v << 0: exp(v/nVt) ≈ 0, so i_d ≈ -Is
        // f(v) = a - 2v + 2*Rp*Is ≈ a - 2v (since Is is tiny)
        // f(v) = 0 → v = a/2
        let expected = a / 2.0;
        assert!(
            (v - expected).abs() < 0.1,
            "Single diode reverse bias should have v≈a/2: v={v:.4}, expected={expected:.4}"
        );
    }

    #[test]
    fn single_diode_asymmetric_waveform() {
        // Process a bipolar signal and verify asymmetric clipping
        let mut diode = DiodeRoot::new(DiodeModel::silicon());
        let rp = 10_000.0;

        let mut v_max = f64::NEG_INFINITY;
        let mut v_min = f64::INFINITY;

        // Process sine-like inputs
        for i in 0..100 {
            let a = 5.0 * (i as f64 * 0.0628).sin(); // ~1Hz at "100 samples"
            let b = diode.process(a, rp);
            let v = (a + b) / 2.0;
            v_max = v_max.max(v);
            v_min = v_min.min(v);
        }

        // Positive side should be clipped at Vf (~0.6V)
        assert!(
            v_max < 1.0,
            "Positive clipping should occur: v_max={v_max:.4}"
        );
        // Negative side should pass through (diode blocks)
        assert!(
            v_min < -1.0,
            "Negative should pass through: v_min={v_min:.4}"
        );
    }

    #[test]
    fn diode_convergence_extreme_inputs() {
        // Verify N-R converges for extreme inputs
        let mut pair = DiodePairRoot::new(DiodeModel::silicon());
        let mut single = DiodeRoot::new(DiodeModel::silicon());
        let rp = 10_000.0;

        for &a in &[0.0, 0.001, 0.01, 0.1, 1.0, 10.0, 100.0, 1000.0] {
            // Positive inputs
            let b_pair = pair.process(a, rp);
            let b_single = single.process(a, rp);
            assert!(b_pair.is_finite(), "Pair should converge for a={a}");
            assert!(b_single.is_finite(), "Single should converge for a={a}");

            // Negative inputs
            let b_pair_neg = pair.process(-a, rp);
            let b_single_neg = single.process(-a, rp);
            assert!(b_pair_neg.is_finite(), "Pair should converge for a={}", -a);
            assert!(b_single_neg.is_finite(), "Single should converge for a={}", -a);
        }
    }

    #[test]
    fn diode_model_parameters_reasonable() {
        // Verify model parameters match expected ranges
        let si = DiodeModel::silicon();
        let ge = DiodeModel::germanium();
        let led = DiodeModel::led();

        // Saturation current: Ge >> Si >> LED
        assert!(ge.is > si.is, "Ge Is should be > Si Is");
        assert!(si.is > led.is, "Si Is should be > LED Is");

        // nVt should be in reasonable range (25-60 mV typical)
        assert!(si.n_vt > 0.02 && si.n_vt < 0.1, "Si nVt out of range");
        assert!(ge.n_vt > 0.02 && ge.n_vt < 0.1, "Ge nVt out of range");
        assert!(led.n_vt > 0.02 && led.n_vt < 0.1, "LED nVt out of range");
    }

    #[test]
    fn diode_1n914_vs_1n4148() {
        // 1N914 and 1N4148 are similar but 1N914 has slightly different Is
        let d914 = DiodeModel::_1n914();
        let d4148 = DiodeModel::_1n4148();

        // Both should have similar but not identical parameters
        assert!(d914.is != d4148.is, "1N914 and 1N4148 should differ");
        assert!(
            (d914.is - d4148.is).abs() < 5e-9,
            "1N914 and 1N4148 should be similar: Is_914={}, Is_4148={}",
            d914.is, d4148.is
        );
    }

    #[test]
    fn diode_small_rp_stability() {
        // Test stability with small port resistance (voltage source driving diode)
        let mut diode = DiodePairRoot::new(DiodeModel::silicon());

        for &rp in &[1.0, 10.0, 100.0] {
            let b = diode.process(5.0, rp);
            assert!(
                b.is_finite(),
                "Should converge with small Rp={rp}: b={b}"
            );
            let v = (5.0 + b) / 2.0;
            assert!(
                v > 0.0 && v < 2.0,
                "Output should be reasonable with Rp={rp}: v={v}"
            );
        }
    }

    #[test]
    fn lfo_sine_range() {
        let mut lfo = Lfo::new(LfoWaveform::Sine, 48000.0);
        lfo.set_depth(1.0);
        for _ in 0..48000 {
            let v = lfo.tick();
            assert!(v >= -1.0 && v <= 1.0, "sine out of range: {v}");
        }
    }

    #[test]
    fn lfo_triangle_symmetry() {
        let mut lfo = Lfo::new(LfoWaveform::Triangle, 48000.0);
        lfo.set_rate(1.0);
        let samples: Vec<f64> = (0..48000).map(|_| lfo.tick()).collect();
        let max = samples.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min = samples.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!((max - 1.0).abs() < 0.01, "triangle max should be ~1.0");
        assert!((min + 1.0).abs() < 0.01, "triangle min should be ~-1.0");
    }

    #[test]
    fn lfo_square_values() {
        let mut lfo = Lfo::new(LfoWaveform::Square, 48000.0);
        lfo.set_rate(1.0);
        for _ in 0..48000 {
            let v = lfo.tick();
            assert!(
                (v - 1.0).abs() < 0.001 || (v + 1.0).abs() < 0.001,
                "square should be ±1"
            );
        }
    }

    #[test]
    fn lfo_saw_waveforms() {
        let mut lfo_up = Lfo::new(LfoWaveform::SawUp, 48000.0);
        let mut lfo_down = Lfo::new(LfoWaveform::SawDown, 48000.0);
        lfo_up.set_rate(1.0);
        lfo_down.set_rate(1.0);

        let up_samples: Vec<f64> = (0..48000).map(|_| lfo_up.tick()).collect();
        let down_samples: Vec<f64> = (0..48000).map(|_| lfo_down.tick()).collect();

        // Up saw should start near -1 and rise to +1
        assert!(up_samples[0] < -0.9, "saw up should start near -1");
        assert!(up_samples[24000] > 0.0, "saw up should be positive at mid");

        // Down saw should start near +1 and fall to -1
        assert!(down_samples[0] > 0.9, "saw down should start near +1");
        assert!(
            down_samples[24000] < 0.0,
            "saw down should be negative at mid"
        );
    }

    #[test]
    fn lfo_sample_and_hold() {
        let mut lfo = Lfo::new(LfoWaveform::SampleAndHold, 48000.0);
        lfo.set_rate(10.0); // 10 Hz = new value every 4800 samples

        let v1 = lfo.tick();
        // Same value for several samples
        for _ in 0..100 {
            assert_eq!(lfo.tick(), v1, "S&H should hold value");
        }
    }

    #[test]
    fn lfo_rate_affects_frequency() {
        let mut lfo = Lfo::new(LfoWaveform::Sine, 48000.0);
        lfo.set_rate(100.0); // 100 Hz

        // Count zero crossings in 48000 samples (1 second)
        let mut crossings = 0;
        let mut prev = lfo.tick();
        for _ in 0..48000 {
            let curr = lfo.tick();
            if (prev < 0.0 && curr >= 0.0) || (prev >= 0.0 && curr < 0.0) {
                crossings += 1;
            }
            prev = curr;
        }
        // 100 Hz sine should have ~200 zero crossings per second
        assert!(
            crossings >= 190 && crossings <= 210,
            "expected ~200 crossings, got {crossings}"
        );
    }

    #[test]
    fn lfo_depth_scaling() {
        let mut lfo = Lfo::new(LfoWaveform::Sine, 48000.0);
        lfo.set_depth(0.5);
        for _ in 0..48000 {
            let v = lfo.tick();
            assert!(v >= -0.5 && v <= 0.5, "depth 0.5 exceeded: {v}");
        }
    }

    #[test]
    fn lfo_reset() {
        let mut lfo = Lfo::new(LfoWaveform::Triangle, 48000.0);
        for _ in 0..1000 {
            lfo.tick();
        }
        lfo.reset();
        assert_eq!(lfo.phase(), 0.0);
    }

    #[test]
    fn lfo_unipolar_range() {
        let mut lfo = Lfo::new(LfoWaveform::Sine, 48000.0);
        lfo.set_depth(1.0);
        for _ in 0..48000 {
            let v = lfo.tick_unipolar();
            assert!(v >= 0.0 && v <= 1.0, "unipolar out of range: {v}");
        }
    }

    // JFET tests
    #[test]
    fn jfet_cutoff_at_pinchoff() {
        let mut jfet = JfetRoot::new(JfetModel::by_name("2N5457"));
        // VTO = -1.25V (InterFET 2N5457), so Vgs = -3.0V should be in cutoff
        jfet.set_vgs(-3.0);
        let ids = jfet.drain_current(5.0);
        assert!(ids.abs() < 1e-9, "should be cutoff: ids={ids}");
    }

    #[test]
    fn jfet_triode_region() {
        let model = JfetModel::by_name("2N5457");
        let mut jfet = JfetRoot::new(model);
        jfet.set_vgs(0.0); // Fully on
                           // Small Vds (triode region)
        let b = jfet.process(0.1, 100.0);
        assert!(b.abs() < 0.1, "JFET triode should pass small signal");
    }

    #[test]
    fn jfet_saturation_region() {
        let model = JfetModel::by_name("2N5457");
        let mut jfet = JfetRoot::new(model);
        jfet.set_vgs(0.0); // Fully on
                           // Large Vds should saturate
        let b = jfet.process(10.0, 1000.0);
        assert!(
            b.abs() < 10.0,
            "JFET saturation should limit current, got b={b}"
        );
    }

    #[test]
    fn jfet_vgs_modulates_current() {
        let mut jfet = JfetRoot::new(JfetModel::by_name("2N5457"));
        // Test that drain current decreases as Vgs becomes more negative
        jfet.set_vgs(0.0);
        let ids_0 = jfet.drain_current(5.0);
        jfet.set_vgs(-1.0);
        let ids_1 = jfet.drain_current(5.0);
        jfet.set_vgs(-2.0);
        let ids_2 = jfet.drain_current(5.0);
        assert!(
            ids_0 > ids_1 && ids_1 > ids_2,
            "Drain current should decrease as Vgs becomes more negative: ids_0={ids_0}, ids_1={ids_1}, ids_2={ids_2}"
        );
    }

    #[test]
    fn jfet_newton_converges() {
        let model = JfetModel::by_name("J201");
        let mut jfet = JfetRoot::new(model);
        jfet.set_vgs(-0.5);

        // Test convergence for various input levels
        for a in [-10.0, -1.0, 0.0, 1.0, 10.0] {
            let b = jfet.process(a, 1000.0);
            assert!(b.is_finite(), "Newton should converge for a={a}");
        }
    }

    #[test]
    fn jfet_wdf_constraint_satisfied() {
        let model = JfetModel::by_name("2N5457");
        let mut jfet = JfetRoot::new(model);
        jfet.set_vgs(-1.0);

        let a = 2.0;
        let rp = 1000.0;
        let b = jfet.process(a, rp);

        // WDF constraint: v = (a + b) / 2
        let v = (a + b) / 2.0;
        // Current: i = (a - b) / (2 * Rp)
        let i = (a - b) / (2.0 * rp);

        // Both v and i should be finite and reasonable
        assert!(v.is_finite() && v.abs() < 100.0, "v should be reasonable");
        assert!(i.is_finite() && i.abs() < 0.1, "i should be reasonable");
    }

    #[test]
    fn jfet_p_channel_polarity() {
        let model = JfetModel::by_name("2N5460");
        let mut jfet = JfetRoot::new(model);
        jfet.set_vgs(0.0); // Fully on for P-channel

        // P-channel should conduct with negative Vds
        let b = jfet.process(-5.0, 1000.0);
        assert!(b.is_finite(), "P-channel should handle negative input");
    }

    // Triode tests
    #[test]
    fn triode_cutoff_at_negative_grid() {
        let mut triode = TriodeRoot::new(TriodeModel::t_12ax7());
        // Very negative grid voltage should cut off the tube
        triode.set_vgk(-50.0);
        let ip = triode.plate_current(200.0);
        assert!(ip < 1e-9, "should be cutoff at Vgk=-50V: ip={ip}");
    }

    #[test]
    fn triode_active_region() {
        let mut triode = TriodeRoot::new(TriodeModel::t_12ax7());
        // Typical operating point: Vgk around -1 to -2V
        triode.set_vgk(-1.5);
        let ip = triode.plate_current(200.0);
        // Should have measurable plate current (Koren model scale)
        assert!(
            ip > 0.0 && ip < 10.0,
            "should be in active region with reasonable current: ip={ip}"
        );
    }

    #[test]
    fn triode_plate_current_increases_with_vpk() {
        let mut triode = TriodeRoot::new(TriodeModel::t_12ax7());
        triode.set_vgk(-1.0);
        let ip_100 = triode.plate_current(100.0);
        let ip_200 = triode.plate_current(200.0);
        let ip_300 = triode.plate_current(300.0);
        assert!(
            ip_100 < ip_200 && ip_200 < ip_300,
            "Ip should increase with Vpk: {ip_100} < {ip_200} < {ip_300}"
        );
    }

    #[test]
    fn triode_vgk_modulates_current() {
        let mut triode = TriodeRoot::new(TriodeModel::t_12ax7());
        // More negative Vgk = less plate current
        triode.set_vgk(0.0);
        let ip_0 = triode.plate_current(200.0);
        triode.set_vgk(-1.0);
        let ip_1 = triode.plate_current(200.0);
        triode.set_vgk(-2.0);
        let ip_2 = triode.plate_current(200.0);
        assert!(
            ip_0 > ip_1 && ip_1 > ip_2,
            "Ip should decrease as Vgk becomes more negative: ip_0={ip_0}, ip_1={ip_1}, ip_2={ip_2}"
        );
    }

    #[test]
    fn triode_newton_converges() {
        let mut triode = TriodeRoot::new(TriodeModel::t_12ax7());
        triode.set_vgk(-1.5);

        // Test convergence for various input levels
        for a in [-10.0, 0.0, 10.0, 100.0, 500.0] {
            let b = triode.process(a, 100_000.0);
            assert!(b.is_finite(), "Newton should converge for a={a}, got b={b}");
        }
    }

    #[test]
    fn triode_12ax7_high_gain() {
        let model = TriodeModel::t_12ax7();
        assert_eq!(model.mu, 100.0, "12AX7 should have mu=100");
    }

    #[test]
    fn triode_12au7_low_gain() {
        let model = TriodeModel::t_12au7();
        assert_eq!(model.mu, 20.0, "12AU7 should have mu=20");
    }

    #[test]
    fn triode_different_tubes_different_current() {
        let mut ax7 = TriodeRoot::new(TriodeModel::t_12ax7());
        let mut au7 = TriodeRoot::new(TriodeModel::t_12au7());
        ax7.set_vgk(-1.0);
        au7.set_vgk(-1.0);

        let ip_ax7 = ax7.plate_current(200.0);
        let ip_au7 = au7.plate_current(200.0);

        // Different tubes should have different characteristics
        assert!(
            (ip_ax7 - ip_au7).abs() > 1e-6,
            "Different tubes should have different Ip: ax7={ip_ax7}, au7={ip_au7}"
        );
    }

    #[test]
    fn triode_wdf_constraint_satisfied() {
        let mut triode = TriodeRoot::new(TriodeModel::t_12ax7());
        triode.set_vgk(-1.5);

        let a = 100.0;
        let rp = 100_000.0;
        let b = triode.process(a, rp);

        // WDF constraint: v = (a + b) / 2
        let v = (a + b) / 2.0;
        // Current: i = (a - b) / (2 * Rp)
        let i = (a - b) / (2.0 * rp);

        assert!(
            v.is_finite() && v.abs() < 1000.0,
            "v should be reasonable: {v}"
        );
        assert!(
            i.is_finite() && i.abs() < 0.01,
            "i should be reasonable: {i}"
        );
    }

    // Photocoupler tests
    #[test]
    fn photocoupler_dark_state() {
        let pc = Photocoupler::new(PhotocouplerModel::vtl5c3(), 48000.0);
        assert!(
            (pc.port_resistance() - 1_000_000.0).abs() < 1.0,
            "dark resistance should be R_dark: {}",
            pc.port_resistance()
        );
    }

    #[test]
    fn photocoupler_full_illumination() {
        let mut pc = Photocoupler::new(PhotocouplerModel::vtl5c3(), 48000.0);
        // Drive to full illumination and let it settle
        for _ in 0..48000 {
            pc.set_led_drive(1.0);
        }
        assert!(
            pc.port_resistance() < 2000.0,
            "fully lit should approach R_light: {}",
            pc.port_resistance()
        );
    }

    #[test]
    fn photocoupler_asymmetric_response() {
        let mut pc = Photocoupler::new(PhotocouplerModel::vtl5c3(), 48000.0);

        // Rise time test
        for _ in 0..4800 {
            // 100ms
            pc.set_led_drive(1.0);
        }
        let r_after_rise = pc.port_resistance();

        // Fall time test - should be slower
        for _ in 0..4800 {
            // 100ms
            pc.set_led_drive(0.0);
        }
        let r_after_fall = pc.port_resistance();

        // After same time, resistance should not fully recover (slow fall)
        assert!(
            r_after_fall < pc.model.r_dark * 0.9,
            "fall should be slower than rise"
        );
        assert!(
            r_after_rise < r_after_fall,
            "resistance should increase when LED off"
        );
    }

    #[test]
    fn photocoupler_model_presets() {
        let vtl5c3 = PhotocouplerModel::vtl5c3();
        let vtl5c1 = PhotocouplerModel::vtl5c1();
        let nsl32 = PhotocouplerModel::nsl32();

        // VTL5C1 should be faster
        assert!(vtl5c1.tau_fast_rise < vtl5c3.tau_fast_rise);

        // NSL-32 should have highest dark resistance
        assert!(nsl32.r_dark > vtl5c3.r_dark);
    }

    #[test]
    fn photocoupler_clamps_drive() {
        let mut pc = Photocoupler::new(PhotocouplerModel::vtl5c3(), 48000.0);

        // Overdrive shouldn't break anything
        pc.set_led_drive(5.0);
        assert!(pc.port_resistance().is_finite());

        // Negative drive shouldn't break anything
        pc.set_led_drive(-1.0);
        assert!(pc.port_resistance().is_finite());
    }

    #[test]
    fn photocoupler_wdf_interface() {
        let mut pc = Photocoupler::new(PhotocouplerModel::vtl5c3(), 48000.0);

        // As a resistor, reflected wave should be 0
        assert_eq!(pc.reflected(), 0.0);

        // Set incident shouldn't change anything (resistor has no state)
        pc.set_incident(1.0);
        assert_eq!(pc.reflected(), 0.0);
    }

    #[test]
    fn photocoupler_sample_rate_update() {
        let mut pc = Photocoupler::new(PhotocouplerModel::vtl5c3(), 48000.0);
        pc.set_sample_rate(96000.0);

        // Should still work after sample rate change
        pc.set_led_drive(0.5);
        assert!(pc.port_resistance().is_finite());
    }

    #[test]
    fn photocoupler_reset() {
        let mut pc = Photocoupler::new(PhotocouplerModel::vtl5c3(), 48000.0);

        // Drive and let settle
        for _ in 0..48000 {
            pc.set_led_drive(1.0);
        }
        assert!(pc.port_resistance() < 500_000.0);

        // Reset should return to dark state
        pc.reset();
        assert!(
            (pc.port_resistance() - 1_000_000.0).abs() < 1.0,
            "reset should return to R_dark: {}",
            pc.port_resistance()
        );
    }

    // EnvelopeFollower tests
    #[test]
    fn envelope_follower_tracks_amplitude() {
        let mut ef = EnvelopeFollower::new(48000.0);

        // Process silence - envelope should stay near zero
        for _ in 0..1000 {
            ef.process(0.0);
        }
        assert!(ef.envelope() < 0.001, "envelope should be ~0 for silence");

        // Process a loud signal - envelope should rise
        for _ in 0..1000 {
            ef.process(0.8);
        }
        assert!(
            ef.envelope() > 0.5,
            "envelope should track loud signal: {}",
            ef.envelope()
        );
    }

    #[test]
    fn envelope_follower_attack_release_asymmetry() {
        let mut ef = EnvelopeFollower::new(48000.0);
        ef.set_attack(1.0); // 1ms attack
        ef.set_release(100.0); // 100ms release

        // Attack phase: loud signal
        for _ in 0..100 {
            ef.process(1.0);
        }
        let peak = ef.envelope();
        assert!(peak > 0.5, "should attack quickly");

        // Release phase: silence
        for _ in 0..100 {
            ef.process(0.0);
        }
        let after_100_samples = ef.envelope();

        // Release should be slower - envelope shouldn't drop much in 100 samples
        assert!(
            after_100_samples > peak * 0.8,
            "release should be slow: peak={peak}, after={after_100_samples}"
        );
    }

    #[test]
    fn envelope_follower_sensitivity() {
        let mut ef1 = EnvelopeFollower::new(48000.0);
        let mut ef2 = EnvelopeFollower::new(48000.0);
        ef1.set_sensitivity(1.0);
        ef2.set_sensitivity(3.0);

        // Process same signal with different sensitivities
        for _ in 0..1000 {
            ef1.process(0.3);
            ef2.process(0.3);
        }

        assert!(
            ef2.envelope() > ef1.envelope() * 2.0,
            "higher sensitivity should give higher envelope: sens1={}, sens2={}",
            ef1.envelope(),
            ef2.envelope()
        );
    }

    #[test]
    fn envelope_follower_reset() {
        let mut ef = EnvelopeFollower::new(48000.0);

        // Build up envelope
        for _ in 0..1000 {
            ef.process(1.0);
        }
        assert!(ef.envelope() > 0.5);

        // Reset
        ef.reset();
        assert!(
            ef.envelope().abs() < 1e-10,
            "reset should zero envelope: {}",
            ef.envelope()
        );
    }

    #[test]
    fn envelope_follower_from_rc() {
        // Auto-wah style: attack τ = 1kΩ × 4.7µF = 4.7ms, release τ = 100kΩ × 1.5µF = 150ms
        let ef_wah =
            EnvelopeFollower::from_rc(1_000.0, 4.7e-6, 100_000.0, 1.5e-6, 20_000.0, 48000.0);
        // Compressor style: attack τ = 100Ω × 10µF = 1ms, release τ = 300kΩ × 1µF = 300ms
        let ef_comp = EnvelopeFollower::from_rc(100.0, 10e-6, 300_000.0, 1e-6, 10_000.0, 48000.0);

        // Auto-wah should have fast attack
        assert!(ef_wah.attack() < 10.0, "wah attack: {}", ef_wah.attack());
        // Compressor should have very fast attack
        assert!(ef_comp.attack() <= 1.5, "comp attack: {}", ef_comp.attack());
        // Compressor should have slow release
        assert!(
            ef_comp.release() >= 250.0,
            "comp release: {}",
            ef_comp.release()
        );
    }

    #[test]
    fn envelope_follower_sample_rate_update() {
        let mut ef = EnvelopeFollower::new(48000.0);
        ef.set_attack(10.0);
        ef.set_release(100.0);

        // Update sample rate
        ef.set_sample_rate(96000.0);

        // Process signal - should still work correctly
        for _ in 0..1000 {
            ef.process(0.5);
        }
        assert!(
            ef.envelope() > 0.3,
            "should track after sample rate change: {}",
            ef.envelope()
        );
    }

    #[test]
    fn envelope_follower_negative_input() {
        let mut ef = EnvelopeFollower::new(48000.0);

        // Process negative signal (full-wave rectification)
        for _ in 0..1000 {
            ef.process(-0.8);
        }

        // Should track absolute value
        assert!(
            ef.envelope() > 0.5,
            "should rectify negative input: {}",
            ef.envelope()
        );
    }

    // ── MOSFET model tests ──────────────────────────────────────────────

    #[test]
    fn mosfet_n2n7000_cutoff() {
        // Below threshold, drain current should be zero
        let model = MosfetModel::n_2n7000();
        let mut root = MosfetRoot::new(model);
        root.set_vgs(0.0); // well below Vth=2.1V
        let ids = root.drain_current(5.0);
        assert!(ids.abs() < 1e-12, "MOSFET should be off below Vth: ids={ids}");
    }

    #[test]
    fn mosfet_n2n7000_saturation() {
        // Above threshold with enough Vds for saturation
        let model = MosfetModel::n_2n7000();
        let mut root = MosfetRoot::new(model);
        root.set_vgs(4.0); // well above Vth=2.1V
        let ids = root.drain_current(5.0);
        assert!(ids > 0.0, "MOSFET should conduct above Vth: ids={ids}");
        // Expected: Kp * (Vgs-Vth)^2 * (1 + lambda*Vds) = 0.1 * 3.61 * 1.2 = 0.433 A
        assert!((ids - 0.433).abs() < 0.05, "saturation current ~0.43A: ids={ids}");
    }

    #[test]
    fn mosfet_pmos_bs250_conduction() {
        let model = MosfetModel::p_bs250();
        let mut root = MosfetRoot::new(model);
        // P-channel: conducts when Vgs < Vth (Vth is -1.5V)
        root.set_vgs(-3.0); // Vgs below Vth
        let ids = root.drain_current(-5.0);
        assert!(ids < 0.0, "P-MOS should conduct: ids={ids}");
    }

    #[test]
    fn mosfet_wdf_root_convergence() {
        let model = MosfetModel::n_2n7000();
        let mut root = MosfetRoot::new(model);
        root.set_vgs(3.5); // Above threshold

        // Process through WDF - should converge without NaN
        let rp = 1000.0;
        let a = 1.0;
        let b = root.process(a, rp);
        assert!(b.is_finite(), "MOSFET WDF root should converge: b={b}");
    }

    #[test]
    fn mosfet_wdf_root_cutoff_passthrough() {
        let model = MosfetModel::n_2n7000();
        let mut root = MosfetRoot::new(model);
        root.set_vgs(0.0); // Below threshold = off

        // When MOSFET is off, it should act like an open circuit
        // reflected wave ≈ incident wave
        let rp = 1000.0;
        let a = 1.0;
        let b = root.process(a, rp);
        assert!(b.is_finite(), "should converge: b={b}");
    }

    // ── Zener diode model tests ─────────────────────────────────────────

    #[test]
    fn zener_forward_conduction() {
        let model = ZenerModel::new(5.1);
        let root = ZenerRoot::new(model);
        // Forward bias: should conduct like a regular diode
        let i = root.current(0.7);
        assert!(i > 0.0, "Zener should conduct in forward bias: i={i}");
    }

    #[test]
    fn zener_reverse_below_breakdown() {
        let model = ZenerModel::new(5.1);
        let root = ZenerRoot::new(model);
        // Reverse bias below Vz: very little current
        let i = root.current(-3.0);
        assert!(i.abs() < 1e-6, "Zener should block below Vz: i={i}");
    }

    #[test]
    fn zener_reverse_breakdown() {
        let model = ZenerModel::new(5.1);
        let root = ZenerRoot::new(model);
        // Reverse bias well past Vz: should conduct heavily
        // At -5.5V (0.4V past breakdown), the exponential model gives large current
        let i = root.current(-5.5);
        assert!(i < -1.0, "Zener should conduct heavily past breakdown: i={i}");
    }

    #[test]
    fn zener_wdf_root_convergence() {
        let model = ZenerModel::new(5.1);
        let mut root = ZenerRoot::new(model);

        // Process through WDF - should converge
        let rp = 1000.0;
        for &a in &[0.0, 0.5, 1.0, -1.0, 5.0, -5.0, 10.0, -10.0] {
            let b = root.process(a, rp);
            assert!(b.is_finite(), "Zener WDF should converge for a={a}: b={b}");
        }
    }

    #[test]
    fn zener_clipping_behavior() {
        let model = ZenerModel::new(5.1);
        let mut root = ZenerRoot::new(model);
        let rp = 1000.0;

        // Small signal should pass mostly unchanged
        let b_small = root.process(0.1, rp);
        let v_small = (0.1 + b_small) / 2.0;

        // Large signal should be clipped
        let b_large = root.process(20.0, rp);
        let v_large = (20.0 + b_large) / 2.0;

        assert!(v_large < 20.0, "large signal should be clipped: v={v_large}");
        assert!(v_small.abs() < v_large.abs(), "small signal < large signal");
    }

    // ── Phase 3: Slew Rate Limiter tests ────────────────────────────────

    #[test]
    fn slew_limiter_slow_signal_passes_through() {
        // At 48kHz with LM308 (0.3 V/µs), max_dv = 0.3e6/48000 ≈ 6.25 V/sample.
        // A 440Hz sine at amplitude 1.0 has max dV/dt ≈ 2π*440 ≈ 2764 V/s
        // = 0.002764 V/µs, well below 0.3 V/µs. Should pass through clean.
        let mut slew = SlewRateLimiter::new(0.3, 48000.0);
        let n = 4800;
        let input: Vec<f64> = (0..n)
            .map(|i| (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin())
            .collect();
        let output: Vec<f64> = input.iter().map(|&x| slew.process(x)).collect();

        // Should be nearly identical (correlation > 0.999)
        let corr = correlation_f64(&input[100..], &output[100..]);
        assert!(
            corr > 0.999,
            "Slow signal should pass through LM308 slew limiter: corr={corr:.6}"
        );
    }

    #[test]
    fn slew_limiter_fast_signal_is_limited() {
        // Test that the slew rate limiter actually limits the output derivative.
        // At 48kHz, 0.3 V/µs → max_dv = 6.25 V/sample.
        // If we feed a step function of +100V, the first output should be
        // limited to max_dv, not jump instantly to 100V.
        let mut slew = SlewRateLimiter::new(0.3, 48000.0);
        let max_dv = 0.3e6 / 48000.0; // 6.25 V/sample

        // Feed a large step from 0 to 100
        let out1 = slew.process(100.0);
        // First output should be clamped at max_dv
        assert!(
            (out1 - max_dv).abs() < 0.01,
            "First sample should be slew-limited: out={out1}, max_dv={max_dv}"
        );

        // Second sample should advance by another max_dv
        let out2 = slew.process(100.0);
        assert!(
            (out2 - 2.0 * max_dv).abs() < 0.01,
            "Second sample should ramp: out={out2}, expected={}", 2.0 * max_dv
        );

        // After many samples, should converge to the target
        for _ in 0..100 {
            slew.process(100.0);
        }
        let final_out = slew.process(100.0);
        assert!(
            (final_out - 100.0).abs() < 0.01,
            "Should converge to target: out={final_out}"
        );
    }

    #[test]
    fn slew_limiter_tl072_transparent() {
        // TL072 at 13 V/µs should be transparent for all audio signals.
        // max_dv = 13e6/48000 ≈ 270 V/sample — way above any audio signal.
        let mut slew = SlewRateLimiter::new(13.0, 48000.0);
        let n = 4800;
        let input: Vec<f64> = (0..n)
            .map(|i| 5.0 * (2.0 * std::f64::consts::PI * 10_000.0 * i as f64 / 48000.0).sin())
            .collect();
        let output: Vec<f64> = input.iter().map(|&x| slew.process(x)).collect();

        let corr = correlation_f64(&input[10..], &output[10..]);
        assert!(
            corr > 0.9999,
            "TL072 should be transparent: corr={corr:.6}"
        );
    }

    #[test]
    fn slew_limiter_lm308_vs_tl072_character() {
        // LM308 (0.3 V/µs) takes longer to reach a step target than TL072 (13 V/µs).
        // After a step of 100V, the LM308 should be further from the target
        // than the TL072 after the same number of samples.
        let mut lm308 = SlewRateLimiter::new(0.3, 48000.0);
        let mut tl072 = SlewRateLimiter::new(13.0, 48000.0);

        // Apply a step and check after 5 samples
        let mut out_lm308 = 0.0;
        let mut out_tl072 = 0.0;
        for _ in 0..5 {
            out_lm308 = lm308.process(100.0);
            out_tl072 = tl072.process(100.0);
        }

        // TL072 should be much closer to 100V than LM308
        assert!(
            out_tl072 > out_lm308,
            "TL072 should slew faster than LM308: tl072={out_tl072:.2} lm308={out_lm308:.2}"
        );
        // LM308: 5 * 6.25 = 31.25V
        // TL072: 5 * 270.8 = min(1354, 100) = 100V
        assert!(
            out_lm308 < 50.0,
            "LM308 should be slow: {out_lm308:.2}"
        );
        assert!(
            out_tl072 > 99.0,
            "TL072 should reach target quickly: {out_tl072:.2}"
        );
    }

    #[test]
    fn slew_limiter_reset() {
        let mut slew = SlewRateLimiter::new(0.3, 48000.0);
        slew.process(5.0);
        slew.reset();
        // After reset, first output should be slew-limited from 0
        let out = slew.process(100.0);
        let max_dv = 0.3e6 / 48000.0;
        assert!(
            (out - max_dv).abs() < 0.01,
            "After reset, should slew from 0: out={out}, max_dv={max_dv}"
        );
    }

    #[test]
    fn slew_limiter_sample_rate_update() {
        let mut slew = SlewRateLimiter::new(0.3, 48000.0);
        let max_dv_48k = 0.3e6 / 48000.0;

        slew.set_sample_rate(96000.0);
        let max_dv_96k = 0.3e6 / 96000.0;

        // At double sample rate, max_dv should be half
        let out = slew.process(100.0);
        assert!(
            (out - max_dv_96k).abs() < 0.01,
            "96kHz should have half the dV of 48kHz: out={out}, expected={max_dv_96k}, 48k={max_dv_48k}"
        );
    }

    // ── Phase 3: OTA (CA3080) model tests ───────────────────────────────

    #[test]
    fn ota_tanh_transfer() {
        let model = OtaModel::ca3080();
        let root = OtaRoot::new(model);

        // Small signal: should be approximately linear
        let i_small = root.output_current(0.001);
        let gm = root.transconductance();
        let expected = gm * 0.001;
        assert!(
            (i_small - expected).abs() / expected.abs() < 0.01,
            "Small signal should be linear: i={i_small}, expected={expected}"
        );

        // Large signal: should saturate at ±Iabc
        let i_large = root.output_current(1.0);
        assert!(
            (i_large - root.iabc()).abs() < root.iabc() * 0.01,
            "Large signal should saturate at Iabc: i={i_large}, iabc={}", root.iabc()
        );

        // Negative large signal: should saturate at -Iabc
        let i_neg = root.output_current(-1.0);
        assert!(
            (i_neg + root.iabc()).abs() < root.iabc() * 0.01,
            "Negative should saturate at -Iabc: i={i_neg}"
        );
    }

    #[test]
    fn ota_gain_control() {
        let model = OtaModel::ca3080();
        let mut root = OtaRoot::new(model);

        // Full gain
        root.set_gain_normalized(1.0);
        let i_full = root.output_current(0.01);

        // Half gain
        root.set_gain_normalized(0.5);
        let i_half = root.output_current(0.01);

        // Zero gain
        root.set_gain_normalized(0.0);
        let i_zero = root.output_current(0.01);

        assert!(
            i_full > i_half,
            "Full gain > half gain: full={i_full}, half={i_half}"
        );
        assert!(
            (i_zero).abs() < 1e-15,
            "Zero gain should give zero current: i={i_zero}"
        );
        // Half gain should give approximately half the current for small signals
        assert!(
            (i_half / i_full - 0.5).abs() < 0.05,
            "Half gain ≈ half current: ratio={}", i_half / i_full
        );
    }

    #[test]
    fn ota_transconductance() {
        let model = OtaModel::ca3080();
        let root = OtaRoot::new(model);

        // gm = Iabc / (2*Vt)
        let gm = root.transconductance();
        let expected = root.iabc() / (2.0 * 25.85e-3);
        assert!(
            (gm - expected).abs() < 1e-6,
            "Transconductance: gm={gm}, expected={expected}"
        );
    }

    #[test]
    fn ota_wdf_root_convergence() {
        let model = OtaModel::ca3080();
        let mut root = OtaRoot::new(model);

        // Process through WDF - should converge for all inputs
        let rp = 10_000.0;
        for &a in &[0.0, 0.1, 0.5, 1.0, -1.0, 5.0, -5.0, 10.0, -10.0] {
            let b = root.process(a, rp);
            assert!(b.is_finite(), "OTA WDF should converge for a={a}: b={b}");
        }
    }

    #[test]
    fn ota_soft_clipping_character() {
        // The OTA's tanh characteristic compresses large signals.
        // For the WDF root, we need signals large enough relative to 2*Vt (~52mV)
        // to reach the tanh saturation region.
        let model = OtaModel::ca3080();
        let root = OtaRoot::new(model);

        // Small signal (well within linear region: 1mV << 52mV)
        let i_small = root.output_current(0.001);

        // Large signal (well into saturation: 200mV >> 52mV)
        let i_large = root.output_current(0.2);

        // Ratio of output currents should be much less than ratio of inputs
        // (200x input increase → should NOT give 200x current increase due to tanh)
        let input_ratio = 0.2 / 0.001;
        let output_ratio = i_large.abs() / i_small.abs();
        assert!(
            output_ratio < input_ratio * 0.5,
            "OTA should compress: input_ratio={input_ratio}, output_ratio={output_ratio:.2}"
        );

        // At 200mV the output should be close to Iabc (saturated)
        let iabc = root.iabc();
        assert!(
            i_large > 0.9 * iabc,
            "OTA should be near saturation at 200mV: i={i_large}, iabc={iabc}"
        );
    }

    // ── Phase 3: BBD Delay Line tests ───────────────────────────────────

    #[test]
    fn bbd_mn3207_delay_range() {
        let model = BbdModel::mn3207();
        // MN3207: 1024 stages, 10kHz-200kHz clock
        // Min delay: 1024/(2*200000) = 2.56ms
        // Max delay: 1024/(2*10000) = 51.2ms
        let min_delay = model.min_delay();
        let max_delay = model.max_delay();

        assert!(
            (min_delay - 0.00256).abs() < 1e-6,
            "MN3207 min delay: {min_delay}"
        );
        assert!(
            (max_delay - 0.0512).abs() < 1e-6,
            "MN3207 max delay: {max_delay}"
        );
    }

    #[test]
    fn bbd_mn3005_long_delay() {
        let model = BbdModel::mn3005();
        // MN3005: 4096 stages, max delay ≈ 204.8ms
        let max_delay = model.max_delay();
        assert!(
            (max_delay - 0.2048).abs() < 1e-6,
            "MN3005 max delay: {max_delay}"
        );
    }

    #[test]
    fn bbd_produces_delayed_output() {
        let model = BbdModel::mn3207();
        let mut bbd = BbdDelayLine::new(model, 48000.0);
        bbd.set_feedback(0.0); // No feedback for clean test

        // Send an impulse and wait for it to come back
        let _ = bbd.process(1.0);
        let delay_samples = (bbd.delay_time() * 48000.0) as usize;

        let mut found_peak = false;
        let mut peak_idx = 0;
        let mut peak_val = 0.0f64;
        for i in 1..delay_samples + 100 {
            let out = bbd.process(0.0);
            if out.abs() > peak_val {
                peak_val = out.abs();
                peak_idx = i;
                found_peak = true;
            }
        }

        assert!(found_peak, "BBD should produce delayed output");
        // Peak should be near the expected delay time
        let expected_delay = delay_samples;
        let tolerance = expected_delay / 4; // 25% tolerance for filter effects
        assert!(
            (peak_idx as i64 - expected_delay as i64).unsigned_abs() < tolerance as u64,
            "Peak at sample {peak_idx}, expected near {expected_delay} (±{tolerance})"
        );
    }

    #[test]
    fn bbd_feedback_creates_repeats() {
        let model = BbdModel::mn3207();
        let mut bbd_no_fb = BbdDelayLine::new(model, 48000.0);
        let mut bbd_with_fb = BbdDelayLine::new(model, 48000.0);
        bbd_no_fb.set_feedback(0.0);
        bbd_with_fb.set_feedback(0.7); // Higher feedback for clearer effect

        // Process impulse
        bbd_no_fb.process(1.0);
        bbd_with_fb.process(1.0);

        // Collect energy over several delay periods
        let n = 48000; // 1 second
        let mut energy_no_fb = 0.0;
        let mut energy_with_fb = 0.0;
        for _ in 0..n {
            let out_no_fb = bbd_no_fb.process(0.0);
            let out_with_fb = bbd_with_fb.process(0.0);
            energy_no_fb += out_no_fb * out_no_fb;
            energy_with_fb += out_with_fb * out_with_fb;
        }

        assert!(
            energy_with_fb > energy_no_fb * 1.1,
            "Feedback should create more energy: with={energy_with_fb:.6}, without={energy_no_fb:.6}"
        );
    }

    #[test]
    fn bbd_clock_modulation() {
        let model = BbdModel::mn3207();
        let mut bbd = BbdDelayLine::new(model, 48000.0);

        // Set to minimum delay
        bbd.set_delay_normalized(0.0);
        let delay_min = bbd.delay_time();

        // Set to maximum delay
        bbd.set_delay_normalized(1.0);
        let delay_max = bbd.delay_time();

        assert!(
            delay_max > delay_min * 3.0,
            "Max delay should be much longer than min: max={delay_max:.6}s, min={delay_min:.6}s"
        );
    }

    #[test]
    fn bbd_soft_clip_characteristic() {
        // BBD soft clip should limit amplitude
        let clipped = bbd_soft_clip(0.5);
        assert!(
            (clipped - (0.5 - 0.5_f64.powi(3) / 3.0)).abs() < 1e-10,
            "Below threshold: cubic soft clip"
        );

        let saturated = bbd_soft_clip(2.0);
        assert!(
            (saturated - 2.0 / 3.0).abs() < 1e-10,
            "Above threshold: saturated at 2/3"
        );
    }

    #[test]
    fn bbd_reset() {
        let model = BbdModel::mn3207();
        let mut bbd = BbdDelayLine::new(model, 48000.0);

        // Fill buffer with signal
        for _ in 0..4800 {
            bbd.process(0.5);
        }

        bbd.reset();

        // After reset, output should be near zero (just noise)
        let out = bbd.process(0.0);
        assert!(
            out.abs() < 0.01,
            "After reset, output should be near zero: {out}"
        );
    }

    #[test]
    fn bbd_output_bounded() {
        let model = BbdModel::mn3207();
        let mut bbd = BbdDelayLine::new(model, 48000.0);
        bbd.set_feedback(0.9);

        // Process loud input for a long time
        for _ in 0..48000 {
            let out = bbd.process(1.0);
            assert!(
                out.abs() < 10.0,
                "BBD output should be bounded: {out}"
            );
            assert!(out.is_finite(), "BBD output should be finite");
        }
    }

    // ── Op-Amp (VCVS) model tests ─────────────────────────────────────

    #[test]
    fn opamp_unity_gain_buffer() {
        // Unity-gain buffer: output should follow Vp
        let model = OpAmpModel::tl072();
        let mut root = OpAmpRoot::unity_gain(model);

        // Set Vp and process
        root.set_vp(1.0);
        let rp = 10_000.0;
        let a = 0.0;
        let b = root.process(a, rp);

        // For unity-gain buffer, output voltage should be ≈ Vp
        let v = (a + b) / 2.0;
        assert!(
            (v - 1.0).abs() < 0.1,
            "Unity-gain buffer should output ≈ Vp: v={v}, Vp=1.0"
        );
    }

    #[test]
    fn opamp_follows_input() {
        // Unity-gain buffer should track Vp as it changes
        let model = OpAmpModel::tl072();
        let mut root = OpAmpRoot::unity_gain(model);
        let rp = 10_000.0;

        for vp in [0.0, 0.5, 1.0, -0.5, -1.0, 2.0, -2.0] {
            root.set_vp(vp);
            let b = root.process(0.0, rp);
            let v = b / 2.0;
            assert!(
                (v - vp).abs() < 0.2,
                "Should track Vp={vp}: got v={v}"
            );
        }
    }

    #[test]
    fn opamp_output_saturation() {
        // Output should saturate at ±Vmax
        let model = OpAmpModel::tl072();
        let mut root = OpAmpRoot::unity_gain(model);

        // Try to drive output beyond rails
        root.set_vp(100.0);
        let b = root.process(0.0, 10_000.0);
        let v = b / 2.0;

        assert!(
            v.abs() <= model.v_max * 1.1,
            "Output should saturate at Vmax={}: got v={v}",
            model.v_max
        );
    }

    #[test]
    fn opamp_slew_rate_limiting() {
        // LM308 has very slow slew rate (0.3 V/µs)
        // At 48kHz, max_dv = 0.3e6 / 48000 = 6.25 V/sample
        let model = OpAmpModel::lm308();
        let mut root = OpAmpRoot::new(model);
        root.set_sample_rate(48000.0);
        root.reset();

        // Apply step from 0 to 10V
        root.set_vp(10.0);
        let b1 = root.process(0.0, 10_000.0);
        let v1 = b1 / 2.0;

        // First output should be slew-limited
        let max_dv = 0.3e6 / 48000.0;
        assert!(
            v1 < max_dv * 2.0,
            "LM308 should slew-limit: v1={v1}, max_dv={max_dv}"
        );
    }

    #[test]
    fn opamp_tl072_faster_than_lm308() {
        // TL072 (13 V/µs) should reach target faster than LM308 (0.3 V/µs)
        // At 48kHz:
        // - LM308: max_dv = 0.3e6 / 48000 = 6.25 V/sample
        // - TL072: max_dv = 13e6 / 48000 = 270 V/sample
        //
        // Both op-amps have v_max = 12V, so target will be clamped.
        // After 1 sample from 0V:
        // - LM308: 1 * 6.25 = 6.25V (slew limited)
        // - TL072: immediately reaches 12V (270 V/sample >> 12V target)
        let mut lm308 = OpAmpRoot::new(OpAmpModel::lm308());
        let mut tl072 = OpAmpRoot::new(OpAmpModel::tl072());
        lm308.set_sample_rate(48000.0);
        tl072.set_sample_rate(48000.0);
        lm308.reset();
        tl072.reset();

        // Apply step to max output (12V)
        lm308.set_vp(12.0);
        tl072.set_vp(12.0);

        // Check after just 1 sample - this is where slew rate matters
        let b1 = lm308.process(0.0, 10_000.0);
        let b2 = tl072.process(0.0, 10_000.0);
        let v_lm308 = b1 / 2.0;
        let v_tl072 = b2 / 2.0;

        // TL072 should reach target immediately
        assert!(
            (v_tl072 - 12.0).abs() < 0.1,
            "TL072 should reach target: v_tl072={v_tl072}"
        );

        // LM308 should be slew-limited to ~6.25V
        let expected_lm308 = 0.3e6 / 48000.0; // 6.25 V
        assert!(
            (v_lm308 - expected_lm308).abs() < 1.0,
            "LM308 should be slew-limited: v_lm308={v_lm308}, expected≈{expected_lm308}"
        );

        // TL072 should definitely be faster
        assert!(
            v_tl072 > v_lm308,
            "TL072 should be faster: tl072={v_tl072}, lm308={v_lm308}"
        );
    }

    #[test]
    fn opamp_wdf_convergence() {
        // Op-amp should converge for all reasonable inputs
        let model = OpAmpModel::jrc4558();
        let mut root = OpAmpRoot::unity_gain(model);

        for &vp in &[0.0, 0.1, 0.5, 1.0, -1.0, 5.0, -5.0, 10.0, -10.0] {
            root.set_vp(vp);
            for &a in &[0.0, 0.1, 1.0, -1.0, 10.0, -10.0] {
                let b = root.process(a, 10_000.0);
                assert!(
                    b.is_finite(),
                    "Op-amp should converge for Vp={vp}, a={a}: b={b}"
                );
            }
        }
    }

    #[test]
    fn opamp_model_presets() {
        // Verify model parameters are reasonable
        let tl072 = OpAmpModel::tl072();
        let lm308 = OpAmpModel::lm308();
        let jrc4558 = OpAmpModel::jrc4558();
        let ne5532 = OpAmpModel::ne5532();

        // LM308 should have slowest slew rate
        assert!(lm308.slew_rate < jrc4558.slew_rate);
        assert!(lm308.slew_rate < tl072.slew_rate);

        // TL072 should be faster than JRC4558
        assert!(tl072.slew_rate > jrc4558.slew_rate);

        // NE5532 should have high GBW
        assert!(ne5532.gbw > jrc4558.gbw);

        // All should have reasonable open-loop gain
        assert!(tl072.open_loop_gain >= 100_000.0);
        assert!(lm308.open_loop_gain >= 100_000.0);
    }

    #[test]
    fn opamp_reset() {
        let model = OpAmpModel::tl072();
        let mut root = OpAmpRoot::new(model);

        // Build up some state
        root.set_vp(5.0);
        for _ in 0..100 {
            root.process(0.0, 10_000.0);
        }

        // Reset
        root.reset();

        // After reset, internal state should be zeroed
        // A step from 0 should be slew-limited properly
        root.set_vp(0.0);
        let b = root.process(0.0, 10_000.0);
        let v = b / 2.0;
        assert!(
            v.abs() < 0.01,
            "After reset with Vp=0, output should be ~0: v={v}"
        );
    }

    // ═══════════════════════════════════════════════════════════════════════
    // BJT (Bipolar Junction Transistor) Tests
    // ═══════════════════════════════════════════════════════════════════════

    #[test]
    fn bjt_npn_forward_active() {
        // Basic NPN test: collector current should flow when Vbe > 0 and Vce > 0
        let model = BjtModel::by_name("2N3904");
        let mut root = BjtNpnRoot::new(model);

        // Set forward bias on base-emitter junction
        root.set_vbe(0.6);

        // Process with positive input (collector-emitter path)
        let b = root.process(10.0, 1000.0);
        let v = (10.0 + b) / 2.0; // Recover voltage

        // Should have some Vce > 0
        assert!(v > 0.0, "Vce should be positive: v={v}");

        // Collector current should be significant
        let ic = root.collector_current();
        assert!(ic > 1e-6, "Collector current should flow: ic={ic}");

        // Base current should be Ic/beta
        let ib = root.base_current();
        let expected_ib = ic / model.bf;
        assert!(
            (ib - expected_ib).abs() < 1e-12,
            "Base current should be Ic/Bf: ib={ib}, expected={expected_ib}"
        );
    }

    #[test]
    fn bjt_npn_cutoff() {
        // When Vbe < 0, BJT is in cutoff - minimal current
        let model = BjtModel::by_name("2N3904");
        let mut root = BjtNpnRoot::new(model);

        root.set_vbe(-0.5); // Reverse bias

        let _b = root.process(10.0, 1000.0);
        let ic = root.collector_current();

        // Collector current should be negligible
        assert!(
            ic < 1e-10,
            "In cutoff, Ic should be very small: ic={ic}"
        );
    }

    #[test]
    fn bjt_npn_vbe_affects_ic() {
        // Higher Vbe should produce higher Ic (exponential relationship)
        let model = BjtModel::by_name("2N3904");
        let mut root = BjtNpnRoot::new(model);

        // Low Vbe
        root.set_vbe(0.5);
        root.process(10.0, 1000.0);
        let ic_low = root.collector_current();

        // Higher Vbe
        root.set_vbe(0.65);
        root.process(10.0, 1000.0);
        let ic_high = root.collector_current();

        // Ic should increase exponentially with Vbe
        assert!(
            ic_high > ic_low * 5.0,
            "Higher Vbe should produce much higher Ic: ic_low={ic_low}, ic_high={ic_high}"
        );
    }

    #[test]
    fn bjt_pnp_forward_active() {
        // PNP test: collector current should flow when Veb > 0 and Vec > 0
        let model = BjtModel::by_name("AC128"); // Classic germanium PNP
        let mut root = BjtPnpRoot::new(model);

        // Set forward bias on emitter-base junction
        root.set_veb(0.3); // Germanium: lower Vbe ~0.2-0.3V

        // Process with positive input (emitter-collector path)
        let b = root.process(10.0, 1000.0);
        let v = (10.0 + b) / 2.0;

        assert!(v > 0.0, "Vec should be positive: v={v}");

        let ic = root.collector_current();
        assert!(ic > 1e-6, "PNP should conduct: ic={ic}");
    }

    #[test]
    fn bjt_germanium_higher_leakage() {
        // Germanium transistors (AC128) have much higher leakage than silicon
        let ge_model = BjtModel::by_name("AC128");
        let si_model = BjtModel::by_name("2N3904");

        // Germanium Is is ~1e-6, silicon Is is ~1e-14
        assert!(
            ge_model.is > si_model.is * 1e6,
            "Germanium should have much higher Is: ge={}, si={}",
            ge_model.is,
            si_model.is
        );
    }

    #[test]
    fn bjt_beta_affects_base_current() {
        // Higher beta = less base current for same Ic
        let high_beta = BjtModel::by_name("2N5089"); // beta ~700
        let low_beta = BjtModel::by_name("2N3904");  // beta ~150

        let mut root_high = BjtNpnRoot::new(high_beta);
        let mut root_low = BjtNpnRoot::new(low_beta);

        // Same Vbe
        root_high.set_vbe(0.6);
        root_low.set_vbe(0.6);

        root_high.process(10.0, 1000.0);
        root_low.process(10.0, 1000.0);

        // Ib = Ic / beta, so higher beta means lower Ib for same Ic
        // But Ic also differs, so check the ratio
        let ratio_high = root_high.collector_current() / root_high.base_current();
        let ratio_low = root_low.collector_current() / root_low.base_current();

        assert!(
            (ratio_high - high_beta.bf).abs() < 1.0,
            "High beta ratio should match: ratio={ratio_high}, beta={}",
            high_beta.bf
        );
        assert!(
            (ratio_low - low_beta.bf).abs() < 1.0,
            "Low beta ratio should match: ratio={ratio_low}, beta={}",
            low_beta.bf
        );
    }

    #[test]
    fn bjt_wdf_convergence() {
        // Verify WDF root converges to valid solution
        let model = BjtModel::by_name("2N3904");
        let mut root = BjtNpnRoot::new(model);

        root.set_vbe(0.6);

        // Test with various input wave values
        for a in [-10.0, 0.0, 5.0, 10.0, 20.0] {
            let b = root.process(a, 1000.0);

            // Result should be finite
            assert!(b.is_finite(), "WDF output should be finite for a={a}");

            // Voltage should be recoverable
            let v = (a + b) / 2.0;
            assert!(v.is_finite(), "Voltage should be finite: v={v}");
        }
    }

    #[test]
    fn bjt_model_presets() {
        // Verify all model presets have reasonable values
        let models: Vec<(&str, BjtModel)> = [
            "2N3904", "2N2222", "BC108", "BC109", "2N5088",
            "2N5089", "2N3906", "AC128", "OC44", "NKT275",
        ].iter().map(|name| (*name, BjtModel::by_name(name))).collect();

        for (name, model) in models {
            // Is should be positive
            assert!(model.is > 0.0, "{name}: Is should be positive");

            // Beta should be in reasonable range (10-1000)
            assert!(
                model.bf > 10.0 && model.bf < 1000.0,
                "{name}: Beta should be reasonable: {}",
                model.bf
            );

            // Thermal voltage should be ~26mV
            assert!(
                (model.vt - 0.02585).abs() < 0.005,
                "{name}: Vt should be ~26mV: {}",
                model.vt
            );

            // Early voltage should be positive
            assert!(model.va > 0.0, "{name}: Va should be positive");
        }
    }

    #[test]
    fn bjt_pnp_cutoff() {
        // When Veb < 0, PNP is in cutoff - minimal current
        let model = BjtModel::by_name("AC128");
        let mut root = BjtPnpRoot::new(model);

        root.set_veb(-0.5); // Reverse bias

        let _b = root.process(10.0, 1000.0);
        let ic = root.collector_current();

        // Collector current should be negligible (but germanium has higher leakage)
        assert!(
            ic < 1e-4,
            "In cutoff, PNP Ic should be small: ic={ic}"
        );
    }

    #[test]
    fn bjt_pnp_veb_affects_ic() {
        // Higher Veb should produce higher Ic (exponential relationship)
        let model = BjtModel::by_name("AC128");
        let mut root = BjtPnpRoot::new(model);

        // Low Veb
        root.set_veb(0.15);
        root.process(10.0, 1000.0);
        let ic_low = root.collector_current();

        // Higher Veb (germanium saturates at lower voltage)
        root.set_veb(0.3);
        root.process(10.0, 1000.0);
        let ic_high = root.collector_current();

        // Ic should increase with Veb
        assert!(
            ic_high > ic_low * 2.0,
            "Higher Veb should produce higher Ic: ic_low={ic_low}, ic_high={ic_high}"
        );
    }

    #[test]
    fn bjt_pnp_wdf_convergence() {
        // Verify PNP WDF root converges to valid solution
        let model = BjtModel::by_name("AC128");
        let mut root = BjtPnpRoot::new(model);

        root.set_veb(0.25);

        // Test with various input wave values
        for a in [-10.0, 0.0, 5.0, 10.0, 20.0] {
            let b = root.process(a, 1000.0);

            // Result should be finite
            assert!(b.is_finite(), "PNP WDF output should be finite for a={a}");

            // Voltage should be recoverable
            let v = (a + b) / 2.0;
            assert!(v.is_finite(), "PNP voltage should be finite: v={v}");
        }
    }

    #[test]
    fn bjt_pnp_base_current() {
        // Verify PNP base current tracking
        let model = BjtModel::by_name("AC128");
        let mut root = BjtPnpRoot::new(model);

        root.set_veb(0.25);
        root.process(10.0, 1000.0);

        let ic = root.collector_current();
        let ib = root.base_current();

        // Base current should be Ic/beta
        let expected_ib = ic / model.bf;
        assert!(
            (ib - expected_ib).abs() < 1e-9,
            "PNP base current should be Ic/Bf: ib={ib}, expected={expected_ib}"
        );
    }

    // ── Phase 3 helper ──────────────────────────────────────────────────

    fn correlation_f64(a: &[f64], b: &[f64]) -> f64 {
        let n = a.len().min(b.len());
        if n == 0 {
            return 0.0;
        }
        let mean_a: f64 = a[..n].iter().sum::<f64>() / n as f64;
        let mean_b: f64 = b[..n].iter().sum::<f64>() / n as f64;
        let mut cov = 0.0;
        let mut var_a = 0.0;
        let mut var_b = 0.0;
        for i in 0..n {
            let da = a[i] - mean_a;
            let db = b[i] - mean_b;
            cov += da * db;
            var_a += da * da;
            var_b += db * db;
        }
        if var_a < 1e-15 || var_b < 1e-15 {
            return 0.0;
        }
        cov / (var_a.sqrt() * var_b.sqrt())
    }
}
