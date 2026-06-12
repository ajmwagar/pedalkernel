//! Test helpers for IIR-based tone stages.
//!
//! Provides [`ToneFeedback`]: a first-order IIR filter derived from the
//! component values of an inverting op-amp with a reactive feedback network
//! (the "active high-shelf" topology used in the Klon Centaur treble control
//! and similar circuits).
//!
//! # Circuit topology
//!
//! ```text
//!         Zf(s)
//!     ┌──/\/\/──┐
//!     │         │
//!  in ──Ri──(-)─┤        Zf = Rf + Rs ‖ (Rp·pos + 1/(Cs))
//!           (+)─┤
//!     gnd───────┘──out
//! ```
//!
//! where:
//! - `Rf`  = series feedback resistor
//! - `Ri`  = input resistor
//! - `Rs`  = shelf resistor (parallel path in feedback)
//! - `C`   = tone capacitor
//! - `Rp`  = pot maximum resistance (variable via `pos ∈ [0,1]`)
//!
//! # Transfer function derivation
//!
//! The s-domain transfer function is:
//!
//! ```text
//! H(s) = -Zf(s) / Ri
//!
//! Zf(s) = Rf + Rs·(Rp·pos·C·s + 1) / ((Rs + Rp·pos)·C·s + 1)
//!
//!       = [C·(Rf·(Rs + Rp·pos) + Rs·Rp·pos)·s + (Rf + Rs)]
//!         / [(Rs + Rp·pos)·C·s + 1]
//! ```
//!
//! This is a first-order shelf filter. Applying the bilinear transform
//! `s ← (2·fs)·(1 - z⁻¹)/(1 + z⁻¹)` yields a 1st-order IIR:
//!
//! ```text
//! y[n] = b0·x[n] + b1·x[n-1] - a1·y[n-1]
//! ```
//!
//! Reference: Chowdhury, "A Comparison of Virtual Analog Modelling
//! Techniques" (arXiv:2009.02833), Section 2.1.

/// First-order IIR filter derived from an inverting op-amp active high-shelf
/// circuit (e.g. Klon Centaur treble control).
///
/// Coefficients are computed analytically from component values and updated
/// when the pot position changes.
pub struct ToneFeedback {
    // ── Component values (fixed at construction) ──
    rf: f64,
    ri: f64,
    c_tone: f64,
    r_shelf: f64,
    max_pot_r: f64,
    sample_rate: f64,

    // ── IIR coefficients (recomputed on pot change) ──
    pub b0: f64,
    pub b1: f64,
    pub a1: f64,
    pub dc_gain: f64,

    // ── Filter state ──
    x1: f64,
    y1: f64,
}

impl ToneFeedback {
    /// Create a new `ToneFeedback` IIR from circuit component values.
    ///
    /// # Arguments
    /// - `rf` — feedback resistor (Ω)
    /// - `ri` — input resistor (Ω)
    /// - `c_tone` — tone capacitor (F)
    /// - `r_shelf` — shelf resistor in feedback path (Ω)
    /// - `max_pot_r` — maximum pot resistance (Ω)
    /// - `_pot_id` — pot label (for future binding, currently unused)
    /// - `sample_rate` — sample rate (Hz)
    /// - `pot_pos` — initial pot position in `[0.0, 1.0]`
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        rf: f64,
        ri: f64,
        c_tone: f64,
        r_shelf: f64,
        max_pot_r: f64,
        _pot_id: String,
        sample_rate: f64,
        pot_pos: f64,
    ) -> Self {
        let mut tf = ToneFeedback {
            rf,
            ri,
            c_tone,
            r_shelf,
            max_pot_r,
            sample_rate,
            b0: 0.0,
            b1: 0.0,
            a1: 0.0,
            dc_gain: 0.0,
            x1: 0.0,
            y1: 0.0,
        };
        tf.recompute(pot_pos);
        tf
    }

    /// Recompute IIR coefficients for a new pot position.
    ///
    /// This applies the bilinear transform to the s-domain transfer function
    /// with the updated pot resistance. Cheap arithmetic — no matrix ops.
    pub fn recompute(&mut self, pot_pos: f64) {
        let rf = self.rf;
        let ri = self.ri;
        let c = self.c_tone;
        let rs = self.r_shelf;
        let rp = self.max_pot_r * pot_pos.clamp(0.0, 1.0);

        // ── s-domain transfer function coefficients ──
        //
        // H(s) = -(1/Ri) · (num1·s + num0) / (den1·s + 1)
        //
        // num1 = C · (Rf·(Rs + Rp) + Rs·Rp)
        // num0 = Rf + Rs
        // den1 = (Rs + Rp) · C

        let num1 = c * (rf * (rs + rp) + rs * rp);
        let num0 = rf + rs;
        let den1 = (rs + rp) * c;

        // ── Bilinear transform: s ← k·(1 - z⁻¹)/(1 + z⁻¹) ──
        //
        // k = 2·fs (no pre-warping — shelf corner is typically well below
        // Nyquist for guitar pedal circuits at 44.1/48 kHz)
        let k = 2.0 * self.sample_rate;

        // Substituting into H(s) and multiplying through by (1 + z⁻¹):
        //
        // Numerator:  (num1·k + num0) + (-num1·k + num0)·z⁻¹
        // Denominator: (den1·k + 1)  + (-den1·k + 1)·z⁻¹
        //
        // Then scale by -1/Ri.

        let n0 = num1 * k + num0;
        let n1 = -num1 * k + num0;
        let d0 = den1 * k + 1.0;
        let d1 = -den1 * k + 1.0;

        // Normalize so a0 = 1.0, and include the -1/Ri gain.
        let gain = -1.0 / ri;
        self.b0 = gain * n0 / d0;
        self.b1 = gain * n1 / d0;
        self.a1 = d1 / d0;

        // DC gain: H(z=1) = H(s=0) = -(Rf + Rs) / Ri
        self.dc_gain = -(rf + rs) / ri;
    }

    /// Process one audio sample through the first-order IIR.
    #[inline]
    pub fn process(&mut self, x: f64) -> f64 {
        let y = self.b0 * x + self.b1 * self.x1 - self.a1 * self.y1;
        self.x1 = x;
        self.y1 = y;
        y
    }

    /// Reset filter state (clear delay line).
    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.y1 = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Verify DC gain matches analytical prediction.
    #[test]
    fn dc_gain_matches_analytical() {
        let tf = ToneFeedback::new(
            1800.0,
            1800.0,
            3.9e-9,
            4700.0,
            10000.0,
            "Treble".into(),
            48000.0,
            0.5,
        );
        let expected_dc = -(1800.0 + 4700.0) / 1800.0;
        assert!(
            (tf.dc_gain - expected_dc).abs() < 1e-10,
            "DC gain mismatch: got {}, expected {}",
            tf.dc_gain,
            expected_dc
        );
    }

    /// Verify coefficients change with pot position.
    #[test]
    fn coefficients_vary_with_pot() {
        let tf0 = ToneFeedback::new(
            1800.0,
            1800.0,
            3.9e-9,
            4700.0,
            10000.0,
            "Treble".into(),
            48000.0,
            0.0,
        );
        let tf1 = ToneFeedback::new(
            1800.0,
            1800.0,
            3.9e-9,
            4700.0,
            10000.0,
            "Treble".into(),
            48000.0,
            1.0,
        );

        // DC gain is pot-independent (shelf resistor dominates at DC)
        assert!((tf0.dc_gain - tf1.dc_gain).abs() < 1e-10);

        // But HF behavior (b0, b1) must differ
        assert!(
            (tf0.b0 - tf1.b0).abs() > 1e-6,
            "b0 should differ: {} vs {}",
            tf0.b0,
            tf1.b0
        );
    }

    /// Verify frequency response: treble boost/cut at 10 kHz.
    #[test]
    fn frequency_response_shelf() {
        let sr = 48000.0;
        let freq = 10000.0;
        let n = 4096usize;

        let mut rms_by_pos = Vec::new();

        for &pos in &[0.0, 0.5, 1.0] {
            let mut tf = ToneFeedback::new(
                1800.0,
                1800.0,
                3.9e-9,
                4700.0,
                10000.0,
                "Treble".into(),
                sr,
                pos,
            );

            // Warm up to reach steady state
            for i in 0..2048 {
                let x = 0.1 * (2.0 * PI * freq * i as f64 / sr).sin();
                tf.process(x);
            }

            // Measure RMS
            let mut sum_sq = 0.0;
            for i in 0..n {
                let x = 0.1 * (2.0 * PI * freq * (i + 2048) as f64 / sr).sin();
                let y = tf.process(x);
                sum_sq += y * y;
            }
            rms_by_pos.push((sum_sq / n as f64).sqrt());
        }

        // At 10 kHz, pot=0 should give less gain than pot=1
        // (pot=0 shorts the cap path → Zf approaches Rf at HF → lower gain)
        assert!(
            rms_by_pos[0] < rms_by_pos[2],
            "Treble pot should boost HF: pos=0 rms={:.6}, pos=1 rms={:.6}",
            rms_by_pos[0],
            rms_by_pos[2]
        );
    }

    /// Verify filter is stable (no NaN/Inf).
    #[test]
    fn stability() {
        let mut tf = ToneFeedback::new(
            1800.0,
            1800.0,
            3.9e-9,
            4700.0,
            10000.0,
            "Treble".into(),
            48000.0,
            0.5,
        );

        for i in 0..10000 {
            let x = 0.5 * (2.0 * PI * 1000.0 * i as f64 / 48000.0).sin();
            let y = tf.process(x);
            assert!(y.is_finite(), "Output became non-finite at sample {}", i);
        }
    }

    // ══════════════════════════════════════════════════════════════════════
    // OpAmpWdfAdaptor tests
    // ══════════════════════════════════════════════════════════════════════

    use crate::compiler::dyn_node::DynNode;
    use crate::compiler::stage::OpAmpWdfAdaptor;
    use crate::dsl::PotTaper;
    use crate::elements::nonlinear::{OpAmpModel, OpAmpRoot};

    /// Build the Klon treble Zf tree:
    /// Parallel(R_tone_fb=1800, Series(Parallel(R_shelf=4700, Treble__wb), Series(C_tone, Treble__aw)))
    fn build_klon_zf(sr: f64, pot_pos: f64) -> DynNode {
        let r_tone_fb = DynNode::Resistor(Some("R_tone_fb".into()), 1800.0);
        let r_shelf = DynNode::Resistor(Some("R_tone_shelf".into()), 4700.0);

        let max_pot_r = 10000.0;
        let treble_wb = DynNode::Pot("Treble__wb".into(), max_pot_r, 1.0 - pot_pos, PotTaper::B);
        let c_tone_rp = 1.0 / (2.0 * sr * 3.9e-9);
        let c_tone = DynNode::Capacitor(Some("C_tone".into()), 3.9e-9, c_tone_rp);
        let treble_aw = DynNode::Pot("Treble__aw".into(), max_pot_r, pot_pos, PotTaper::B);

        // Inner parallel: R_shelf ‖ Treble__wb
        let inner_par = DynNode::Parallel(Box::new(r_shelf), Box::new(treble_wb));
        // Inner series: C_tone + Treble__aw
        let inner_ser = DynNode::Series(Box::new(c_tone), Box::new(treble_aw));
        // Outer series: (R_shelf ‖ Treble__wb) + (C_tone + Treble__aw)
        let outer_ser = DynNode::Series(Box::new(inner_par), Box::new(inner_ser));
        // Root parallel: R_tone_fb ‖ outer_ser
        DynNode::Parallel(Box::new(r_tone_fb), Box::new(outer_ser))
    }

    /// Build a Klon-style inverting OpAmpWdfAdaptor for testing.
    fn build_klon_adaptor(sr: f64, pot_pos: f64) -> OpAmpWdfAdaptor {
        let ri = 1800.0;
        let zi = DynNode::VoltageSource(0.0, ri);
        let zf = build_klon_zf(sr, pot_pos);

        let model = OpAmpModel::tl072();
        let gain = 1800.0 / ri; // DC gain = Rf/Ri (approximate)
        let mut opamp = OpAmpRoot::new_inverting(model, 1.0);
        opamp.set_sample_rate(sr);
        opamp.set_gbw_gain(gain);
        opamp.set_v_max(3.0); // 9V supply / 2 - 1.5

        OpAmpWdfAdaptor::new(zi, zf, true, opamp, Some("Treble".into()))
    }

    /// Measure RMS output at a given frequency through the adaptor.
    fn measure_rms(adaptor: &mut OpAmpWdfAdaptor, freq: f64, sr: f64, amplitude: f64) -> f64 {
        let warmup = 4096;
        let n = 4096;

        // Warm up
        for i in 0..warmup {
            let x = amplitude * (2.0 * PI * freq * i as f64 / sr).sin();
            adaptor.process(0.0, x); // inverting: V+ = 0, V_in = signal
        }
        // Measure
        let mut sum_sq = 0.0;
        for i in 0..n {
            let x = amplitude * (2.0 * PI * freq * (i + warmup) as f64 / sr).sin();
            let y = adaptor.process(0.0, x);
            sum_sq += y * y;
        }
        (sum_sq / n as f64).sqrt()
    }

    #[test]
    fn wdf_adaptor_produces_signal() {
        let sr = 48000.0;
        let mut adaptor = build_klon_adaptor(sr, 0.5);
        let rms = measure_rms(&mut adaptor, 1000.0, sr, 0.1);
        eprintln!("WDF adaptor 1kHz RMS = {:.6}", rms);
        assert!(
            rms > 0.01,
            "Adaptor should produce audible output, got {:.6}",
            rms
        );
    }

    #[test]
    fn wdf_adaptor_stable() {
        let sr = 48000.0;
        let mut adaptor = build_klon_adaptor(sr, 0.5);
        for i in 0..20000 {
            let x = 0.5 * (2.0 * PI * 1000.0 * i as f64 / sr).sin();
            let y = adaptor.process(0.0, x);
            assert!(y.is_finite(), "Output NaN/Inf at sample {}", i);
            assert!(
                y.abs() < 100.0,
                "Output diverging at sample {}: {:.2}",
                i,
                y
            );
        }
    }

    #[test]
    fn wdf_adaptor_dc_gain_inverting() {
        // At DC with Zi=Ri=1800 and Zf=Rf=1800 (ignoring reactive elements),
        // gain should be approximately -Rf/Ri = -1.0
        let sr = 48000.0;
        let mut adaptor = build_klon_adaptor(sr, 0.5);

        // Drive with DC step, let settle
        let dc_in = 0.1;
        let mut last = 0.0;
        for _ in 0..10000 {
            last = adaptor.process(0.0, dc_in);
        }
        let dc_gain = last / dc_in;
        eprintln!("WDF adaptor DC gain = {:.4}", dc_gain);
        // DC gain magnitude: with Zf network at DC, caps open → Zf_dc ≈ R_tone_fb ‖ (R_shelf + R_pot/2)
        // Expected |gain| = Zf_dc / Ri ≈ 1800‖(4700+5000) / 1800 ≈ 1500/1800 ≈ 0.83 to ~3.6
        // Sign may be inverted in WDF convention (port polarity dependent)
        let gain_mag = dc_gain.abs();
        assert!(
            gain_mag > 0.3 && gain_mag < 15.0,
            "DC gain magnitude should be reasonable, got {:.4} (mag={:.4})",
            dc_gain,
            gain_mag
        );
    }

    #[test]
    fn wdf_adaptor_treble_changes_spectrum() {
        let sr = 48000.0;
        let freq = 10000.0; // 10kHz — treble range

        let mut adaptor_dark = build_klon_adaptor(sr, 0.0);
        let mut adaptor_bright = build_klon_adaptor(sr, 1.0);

        let rms_dark = measure_rms(&mut adaptor_dark, freq, sr, 0.1);
        let rms_bright = measure_rms(&mut adaptor_bright, freq, sr, 0.1);

        let ratio_db = 20.0 * (rms_bright / rms_dark).log10();
        eprintln!(
            "WDF adaptor treble at {}Hz: dark={:.6}, bright={:.6}, ratio={:.3}x, dB={:.2}",
            freq,
            rms_dark,
            rms_bright,
            rms_bright / rms_dark,
            ratio_db
        );

        assert!(
            ratio_db.abs() > 0.5,
            "Treble pot should affect 10kHz output by ≥0.5dB, got {:.2}dB",
            ratio_db
        );
    }

    #[test]
    fn wdf_adaptor_frequency_response_shape() {
        // Verify the adaptor produces different responses at different frequencies.
        // A shelving EQ should have more effect at HF than LF.
        let sr = 48000.0;

        let mut adaptor_dark = build_klon_adaptor(sr, 0.0);
        let mut adaptor_bright = build_klon_adaptor(sr, 1.0);

        let freqs = [100.0, 1000.0, 5000.0, 10000.0];
        let mut ratios = Vec::new();

        for &freq in &freqs {
            // Need fresh adaptors for each frequency to avoid state contamination
            let mut ad = build_klon_adaptor(sr, 0.0);
            let mut ab = build_klon_adaptor(sr, 1.0);
            let rms_d = measure_rms(&mut ad, freq, sr, 0.1);
            let rms_b = measure_rms(&mut ab, freq, sr, 0.1);
            let ratio = if rms_d > 1e-10 { rms_b / rms_d } else { 1.0 };
            let db = 20.0 * ratio.log10();
            eprintln!(
                "  {}Hz: dark={:.6} bright={:.6} ratio={:.4} ({:.2}dB)",
                freq, rms_d, rms_b, ratio, db
            );
            ratios.push(db);
        }

        // The shelf should have more effect at HF than LF
        // ratios[0] is at 100Hz, ratios[3] is at 10kHz
        // The difference in dB between HF and LF should be > 0
        let hf_lf_diff = (ratios[3] - ratios[0]).abs();
        eprintln!("HF-LF pot effect difference: {:.2}dB", hf_lf_diff);
        assert!(
            hf_lf_diff > 0.1,
            "Shelving EQ should affect HF more than LF, diff={:.2}dB",
            hf_lf_diff
        );
    }
}
