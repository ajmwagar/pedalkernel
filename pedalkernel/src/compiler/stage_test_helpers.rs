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
            1800.0, 1800.0, 3.9e-9, 4700.0, 10000.0,
            "Treble".into(), 48000.0, 0.5,
        );
        let expected_dc = -(1800.0 + 4700.0) / 1800.0;
        assert!(
            (tf.dc_gain - expected_dc).abs() < 1e-10,
            "DC gain mismatch: got {}, expected {}",
            tf.dc_gain, expected_dc
        );
    }

    /// Verify coefficients change with pot position.
    #[test]
    fn coefficients_vary_with_pot() {
        let tf0 = ToneFeedback::new(
            1800.0, 1800.0, 3.9e-9, 4700.0, 10000.0,
            "Treble".into(), 48000.0, 0.0,
        );
        let tf1 = ToneFeedback::new(
            1800.0, 1800.0, 3.9e-9, 4700.0, 10000.0,
            "Treble".into(), 48000.0, 1.0,
        );

        // DC gain is pot-independent (shelf resistor dominates at DC)
        assert!((tf0.dc_gain - tf1.dc_gain).abs() < 1e-10);

        // But HF behavior (b0, b1) must differ
        assert!(
            (tf0.b0 - tf1.b0).abs() > 1e-6,
            "b0 should differ: {} vs {}",
            tf0.b0, tf1.b0
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
                1800.0, 1800.0, 3.9e-9, 4700.0, 10000.0,
                "Treble".into(), sr, pos,
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
            rms_by_pos[0], rms_by_pos[2]
        );
    }

    /// Verify filter is stable (no NaN/Inf).
    #[test]
    fn stability() {
        let mut tf = ToneFeedback::new(
            1800.0, 1800.0, 3.9e-9, 4700.0, 10000.0,
            "Treble".into(), 48000.0, 0.5,
        );

        for i in 0..10000 {
            let x = 0.5 * (2.0 * PI * 1000.0 * i as f64 / 48000.0).sin();
            let y = tf.process(x);
            assert!(y.is_finite(), "Output became non-finite at sample {}", i);
        }
    }
}
