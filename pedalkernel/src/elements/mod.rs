//! WDF circuit elements — re-exported from pedalkernel-rt.
//!
//! This module re-exports all element types, traits, and implementations from
//! the `pedalkernel-rt` runtime crate. The canonical implementations live there
//! (no_std compatible); this module adds model-DB-dependent lookup functions
//! via `crate::model_lookup`.

pub use pedalkernel_rt::elements::*;
pub use pedalkernel_rt::elements::nonlinear;

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
#[allow(deprecated)]
mod tests {
    use super::*;
    use crate::model_lookup::*;

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
        assert!(
            (b_pos - b_neg).abs() > 1e-10,
            "should be asymmetric: b+={b_pos}, b-={b_neg}"
        );
    }

    #[test]
    fn diode_pair_symmetry() {
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
        let mut diode = DiodePairRoot::new(DiodeModel::silicon());
        let rp = 10_000.0;
        let a = 20.0;
        let b = diode.process(a, rp);
        let v = (a + b) / 2.0;
        assert!(
            v > 0.5 && v < 1.0,
            "Silicon diode pair should clip at ~0.6V: v={v:.4}"
        );
    }

    #[test]
    fn diode_germanium_lower_threshold() {
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
    fn diode_model_parameters_reasonable() {
        let si = DiodeModel::silicon();
        let ge = DiodeModel::germanium();
        let led = DiodeModel::led();

        assert!(ge.is > si.is, "Ge Is should be > Si Is");
        assert!(si.is > led.is, "Si Is should be > LED Is");
        assert!(si.n_vt > 0.02 && si.n_vt < 0.1, "Si nVt out of range");
        assert!(ge.n_vt > 0.02 && ge.n_vt < 0.1, "Ge nVt out of range");
        assert!(led.n_vt > 0.02 && led.n_vt < 0.1, "LED nVt out of range");
    }

    #[test]
    fn diode_1n914_vs_1n4148() {
        let d914 = DiodeModel::_1n914();
        let d4148 = DiodeModel::_1n4148();
        assert!(d914.is != d4148.is, "1N914 and 1N4148 should differ");
        assert!(
            (d914.is - d4148.is).abs() < 5e-9,
            "1N914 and 1N4148 should be similar: Is_914={}, Is_4148={}",
            d914.is,
            d4148.is
        );
    }

    #[test]
    fn diode_small_rp_stability() {
        let mut diode = DiodePairRoot::new(DiodeModel::silicon());
        for &rp in &[1.0, 10.0, 100.0] {
            let b = diode.process(5.0, rp);
            assert!(b.is_finite(), "Should converge with small Rp={rp}: b={b}");
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

    // JFET tests
    #[test]
    fn jfet_cutoff_at_pinchoff() {
        let mut jfet = JfetRoot::new(jfet_model_by_name("2N5457"));
        jfet.set_vgs(-3.0);
        let ids = jfet.drain_current(5.0);
        assert!(ids.abs() < 1e-9, "should be cutoff: ids={ids}");
    }

    #[test]
    fn jfet_triode_region() {
        let model = jfet_model_by_name("2N5457");
        let mut jfet = JfetRoot::new(model);
        jfet.set_vgs(0.0);
        let b = jfet.process(0.1, 100.0);
        assert!(b.abs() < 0.1, "JFET triode should pass small signal");
    }

    #[test]
    fn jfet_saturation_region() {
        let model = jfet_model_by_name("2N5457");
        let mut jfet = JfetRoot::new(model);
        jfet.set_vgs(0.0);
        let b = jfet.process(10.0, 1000.0);
        assert!(
            b.abs() < 10.0,
            "JFET saturation should limit current, got b={b}"
        );
    }

    #[test]
    fn jfet_vgs_modulates_current() {
        let mut jfet = JfetRoot::new(jfet_model_by_name("2N5457"));
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
        let model = jfet_model_by_name("J201");
        let mut jfet = JfetRoot::new(model);
        jfet.set_vgs(-0.5);
        for a in [-10.0, -1.0, 0.0, 1.0, 10.0] {
            let b = jfet.process(a, 1000.0);
            assert!(b.is_finite(), "Newton should converge for a={a}");
        }
    }

    #[test]
    fn jfet_wdf_constraint_satisfied() {
        let model = jfet_model_by_name("2N5457");
        let mut jfet = JfetRoot::new(model);
        jfet.set_vgs(-1.0);
        let a = 2.0;
        let rp = 1000.0;
        let b = jfet.process(a, rp);
        let v = (a + b) / 2.0;
        let i = (a - b) / (2.0 * rp);
        assert!(v.is_finite() && v.abs() < 100.0, "v should be reasonable");
        assert!(i.is_finite() && i.abs() < 0.1, "i should be reasonable");
    }

    #[test]
    fn jfet_p_channel_polarity() {
        let model = jfet_model_by_name("2N5460");
        let mut jfet = JfetRoot::new(model);
        jfet.set_vgs(0.0);
        let b = jfet.process(-5.0, 1000.0);
        assert!(b.is_finite(), "P-channel should handle negative input");
    }

    // Triode tests
    #[test]
    fn triode_cutoff_at_negative_grid() {
        let mut triode = TriodeRoot::new(triode_model_by_name("12AX7"));
        triode.set_vgk(-50.0);
        let ip = triode.plate_current(200.0);
        assert!(ip < 1e-9, "should be cutoff at Vgk=-50V: ip={ip}");
    }

    #[test]
    fn triode_active_region() {
        let mut triode = TriodeRoot::new(triode_model_by_name("12AX7"));
        triode.set_vgk(-1.5);
        let ip = triode.plate_current(200.0);
        assert!(
            ip > 1e-4 && ip < 5e-3,
            "should be in active region with realistic 12AX7 current (0.1-5mA): ip={ip} A"
        );
    }

    #[test]
    fn triode_plate_current_increases_with_vpk() {
        let mut triode = TriodeRoot::new(triode_model_by_name("12AX7"));
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
        let mut triode = TriodeRoot::new(triode_model_by_name("12AX7"));
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
        let mut triode = TriodeRoot::new(triode_model_by_name("12AX7"));
        triode.set_vgk(-1.5);
        for a in [-10.0, 0.0, 10.0, 100.0, 500.0] {
            let b = triode.process(a, 100_000.0);
            assert!(b.is_finite(), "Newton should converge for a={a}, got b={b}");
        }
    }

    #[test]
    fn triode_12ax7_high_gain() {
        let model = triode_model_by_name("12AX7");
        assert_eq!(model.mu, 100.0, "12AX7 should have mu=100");
    }

    #[test]
    fn triode_12au7_low_gain() {
        let model = triode_model_by_name("12AU7");
        assert!(
            (model.mu - 21.5).abs() < 0.01,
            "12AU7 should have mu=21.5 (Koren)"
        );
    }

    #[test]
    fn triode_different_tubes_different_current() {
        let mut ax7 = TriodeRoot::new(triode_model_by_name("12AX7"));
        let mut au7 = TriodeRoot::new(triode_model_by_name("12AU7"));
        ax7.set_vgk(-1.0);
        au7.set_vgk(-1.0);

        let ip_ax7 = ax7.plate_current(200.0);
        let ip_au7 = au7.plate_current(200.0);

        assert!(
            (ip_ax7 - ip_au7).abs() > 1e-6,
            "Different tubes should have different Ip: ax7={ip_ax7}, au7={ip_au7}"
        );
    }

    #[test]
    fn triode_wdf_constraint_satisfied() {
        let mut triode = TriodeRoot::new(triode_model_by_name("12AX7"));
        triode.set_vgk(-1.5);
        let a = 100.0;
        let rp = 100_000.0;
        let b = triode.process(a, rp);
        let v = (a + b) / 2.0;
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
    fn photocoupler_model_presets() {
        let vtl5c3 = PhotocouplerModel::vtl5c3();
        let vtl5c1 = PhotocouplerModel::vtl5c1();
        let nsl32 = PhotocouplerModel::nsl32();
        assert!(vtl5c1.tau_fast_rise < vtl5c3.tau_fast_rise);
        assert!(nsl32.r_dark > vtl5c3.r_dark);
    }

    // Op-amp tests
    #[test]
    fn opamp_unity_gain_buffer() {
        let model = OpAmpModel::tl072();
        let mut root = OpAmpRoot::unity_gain(model);
        root.set_vp(1.0);
        let rp = 10_000.0;
        let a = 0.0;
        let b = root.process(a, rp);
        let v = (a + b) / 2.0;
        assert!(
            (v - 1.0).abs() < 0.1,
            "Unity-gain buffer should output ≈ Vp: v={v}, Vp=1.0"
        );
    }

    #[test]
    fn opamp_model_presets() {
        let tl072 = OpAmpModel::tl072();
        let lm308 = OpAmpModel::lm308();
        let jrc4558 = OpAmpModel::jrc4558();
        let ne5532 = OpAmpModel::ne5532();
        assert!(lm308.slew_rate < jrc4558.slew_rate);
        assert!(lm308.slew_rate < tl072.slew_rate);
        assert!(tl072.slew_rate > jrc4558.slew_rate);
        assert!(ne5532.gbw > jrc4558.gbw);
        assert!(tl072.open_loop_gain >= 100_000.0);
        assert!(lm308.open_loop_gain >= 100_000.0);
    }

    #[test]
    fn opamp_wdf_convergence() {
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
}
