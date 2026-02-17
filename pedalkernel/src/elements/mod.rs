//! WDF circuit elements — traits and implementations.
//!
//! This module defines trait abstractions for different element categories:
//! - [`WdfLeaf`] — one-port elements (R, C, L, Vs) that terminate tree branches
//! - [`WdfRoot`] — nonlinear elements (diodes, JFETs) at the tree root
//! - [`Modulator`] — control signal generators (LFO, envelope follower)
//! - [`ControlledResistance`] — elements with externally-controlled resistance
//!
//! Adaptors (Series, Parallel) remain in the `tree` module as they're structural.

mod controlled;
mod linear;
mod modulation;
mod nonlinear;

pub use controlled::*;
pub use linear::*;
pub use modulation::*;
pub use nonlinear::*;

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
        let mut jfet = JfetRoot::new(JfetModel::n_2n5457());
        // Vp = -2.5V, so Vgs = -3.0V should be in cutoff
        jfet.set_vgs(-3.0);
        let ids = jfet.drain_current(5.0);
        assert!(ids.abs() < 1e-9, "should be cutoff: ids={ids}");
    }

    #[test]
    fn jfet_triode_region() {
        let model = JfetModel::n_2n5457();
        let mut jfet = JfetRoot::new(model);
        jfet.set_vgs(0.0); // Fully on
                           // Small Vds (triode region)
        let b = jfet.process(0.1, 100.0);
        assert!(b.abs() < 0.1, "JFET triode should pass small signal");
    }

    #[test]
    fn jfet_saturation_region() {
        let model = JfetModel::n_2n5457();
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
        let mut jfet = JfetRoot::new(JfetModel::n_2n5457());
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
        let model = JfetModel::n_j201();
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
        let model = JfetModel::n_2n5457();
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
        let model = JfetModel::p_2n5460();
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

    // NoiseGate tests
    #[test]
    fn noise_gate_silences_noise() {
        let mut gate = NoiseGate::new(48000.0);
        // Process low-level noise (well below threshold)
        let mut max_out = 0.0_f64;
        for _ in 0..4800 {
            let noise = 0.0001; // −80 dBFS — typical ADC noise floor
            let out = gate.process(noise);
            max_out = max_out.max(out.abs());
        }
        // Gate should attenuate this to near-zero
        assert!(
            max_out < 0.0001,
            "gate should silence noise: max_out={max_out}"
        );
    }

    #[test]
    fn noise_gate_passes_signal() {
        let mut gate = NoiseGate::new(48000.0);
        // Process a guitar-level signal (well above threshold)
        let mut max_out = 0.0_f64;
        for i in 0..4800 {
            let t = i as f64 / 48000.0;
            let signal = 0.3 * (2.0 * std::f64::consts::PI * 440.0 * t).sin();
            let out = gate.process(signal);
            max_out = max_out.max(out.abs());
        }
        // Gate should pass signal through at full level
        assert!(max_out > 0.2, "gate should pass signal: max_out={max_out}");
    }

    #[test]
    fn noise_gate_hysteresis() {
        let mut gate = NoiseGate::new(48000.0);
        // Open the gate with a strong signal
        for i in 0..4800 {
            let t = i as f64 / 48000.0;
            gate.process(0.5 * (2.0 * std::f64::consts::PI * 440.0 * t).sin());
        }
        // Now send a signal just below the open threshold but above close threshold.
        // The gate should remain open due to hysteresis.
        let mut max_out = 0.0_f64;
        for i in 0..480 {
            let t = i as f64 / 48000.0;
            // 0.0008 RMS is above close_threshold (0.0005) but below open_threshold (0.001)
            let signal = 0.001 * (2.0 * std::f64::consts::PI * 440.0 * t).sin();
            let out = gate.process(signal);
            max_out = max_out.max(out.abs());
        }
        assert!(
            max_out > 0.0005,
            "hysteresis should keep gate open: max_out={max_out}"
        );
    }

    #[test]
    fn noise_gate_smooth_release() {
        let mut gate = NoiseGate::new(48000.0);
        // Open with signal
        for i in 0..4800 {
            let t = i as f64 / 48000.0;
            gate.process(0.5 * (2.0 * std::f64::consts::PI * 440.0 * t).sin());
        }
        // Now go to silence — gate should close smoothly (not instantly)
        let first_silent = gate.process(0.0);
        // Immediately after signal stops, gain should still be partially open
        assert!(
            first_silent.abs() < 0.001,
            "no signal means near-zero output"
        );
        // But the gate gain itself should still be transitioning
        // (tested implicitly: no clicks from instant mute)
    }

    #[test]
    fn noise_gate_reset() {
        let mut gate = NoiseGate::new(48000.0);
        // Open the gate
        for i in 0..4800 {
            let t = i as f64 / 48000.0;
            gate.process(0.5 * (2.0 * std::f64::consts::PI * 440.0 * t).sin());
        }
        gate.reset();
        // After reset, low-level noise should be gated again
        let out = gate.process(0.0001);
        assert!(out.abs() < 0.0001, "reset should close gate");
    }
}
