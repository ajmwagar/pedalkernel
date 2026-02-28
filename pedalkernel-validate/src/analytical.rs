//! Analytical reference generators for linear circuits.
//!
//! These provide mathematically exact responses for validating WDF
//! against theory, without needing SPICE.

use std::f64::consts::PI;

/// First-order RC lowpass filter impulse response.
///
/// The continuous-time impulse response is:
///   h(t) = (1/τ) * exp(-t/τ)  for t ≥ 0
/// where τ = RC.
///
/// For a digital system, we sample this and normalize to get
/// the discrete impulse response that, when convolved with input,
/// gives the lowpass-filtered output.
pub fn rc_lowpass_impulse_response(
    r_ohms: f64,
    c_farads: f64,
    sample_rate: f64,
    n_samples: usize,
) -> Vec<f64> {
    let tau = r_ohms * c_farads;
    let dt = 1.0 / sample_rate;

    // The bilinear transform gives us the exact discrete-time
    // equivalent. For a first-order lowpass:
    //   H(s) = 1 / (1 + s*τ)
    // Bilinear: s = (2/T) * (1 - z^-1) / (1 + z^-1)
    //
    // This gives: H(z) = (b0 + b1*z^-1) / (1 + a1*z^-1)
    // where:
    //   K = 2 / T
    //   b0 = b1 = 1 / (1 + K*τ)
    //   a1 = (1 - K*τ) / (1 + K*τ)

    let k = 2.0 / dt;
    let denom = 1.0 + k * tau;
    let b0 = 1.0 / denom;
    let b1 = b0;
    let a1 = (1.0 - k * tau) / denom;

    // Generate impulse response by feeding impulse through the filter
    let mut output = Vec::with_capacity(n_samples);
    let mut x_prev = 0.0;
    let mut y_prev = 0.0;

    for i in 0..n_samples {
        let x = if i == 0 { 1.0 } else { 0.0 };
        let y = b0 * x + b1 * x_prev - a1 * y_prev;
        output.push(y);
        x_prev = x;
        y_prev = y;
    }

    output
}

/// First-order RC lowpass frequency response magnitude at given frequency.
///
/// |H(f)| = 1 / sqrt(1 + (f/fc)^2)
/// where fc = 1 / (2*pi*R*C)
pub fn rc_lowpass_magnitude(r_ohms: f64, c_farads: f64, freq_hz: f64) -> f64 {
    let fc = 1.0 / (2.0 * PI * r_ohms * c_farads);
    1.0 / (1.0 + (freq_hz / fc).powi(2)).sqrt()
}

/// First-order RC lowpass phase response at given frequency.
///
/// phase(f) = -atan(f/fc)
pub fn rc_lowpass_phase(r_ohms: f64, c_farads: f64, freq_hz: f64) -> f64 {
    let fc = 1.0 / (2.0 * PI * r_ohms * c_farads);
    -(freq_hz / fc).atan()
}

/// Process a signal through an ideal first-order RC lowpass.
///
/// Uses bilinear transform for exact discrete-time response.
pub fn rc_lowpass_filter(
    signal: &[f64],
    r_ohms: f64,
    c_farads: f64,
    sample_rate: f64,
) -> Vec<f64> {
    let tau = r_ohms * c_farads;
    let dt = 1.0 / sample_rate;

    let k = 2.0 / dt;
    let denom = 1.0 + k * tau;
    let b0 = 1.0 / denom;
    let b1 = b0;
    let a1 = (1.0 - k * tau) / denom;

    let mut output = Vec::with_capacity(signal.len());
    let mut x_prev = 0.0;
    let mut y_prev = 0.0;

    for &x in signal {
        let y = b0 * x + b1 * x_prev - a1 * y_prev;
        output.push(y);
        x_prev = x;
        y_prev = y;
    }

    output
}

/// First-order RL lowpass frequency response magnitude at given frequency.
///
/// For RL lowpass: L in series with input, R to ground (output across R)
/// |H(f)| = 1 / sqrt(1 + (f/fc)^2)
/// where fc = R / (2*pi*L)
pub fn rl_lowpass_magnitude(l_henrys: f64, r_ohms: f64, freq_hz: f64) -> f64 {
    let fc = r_ohms / (2.0 * PI * l_henrys);
    1.0 / (1.0 + (freq_hz / fc).powi(2)).sqrt()
}

/// Process a signal through an ideal first-order RL lowpass.
///
/// Uses the proper WDF series adaptor algorithm:
/// - Source sets incident wave: a_s = 2 * v_in
/// - Inductor reflects: b_L = a_L[n-1]
/// - Resistor reflects: b_R = 0 (matched)
/// - Series adaptor scatters
/// - Output: v_out = a_R / 2
///
/// This gives H(s) = R / (R + sL) with fc = R / (2*pi*L)
pub fn rl_lowpass_filter(
    signal: &[f64],
    l_henrys: f64,
    r_ohms: f64,
    sample_rate: f64,
) -> Vec<f64> {
    // WDF port resistances
    let r_l = 2.0 * l_henrys * sample_rate;  // Inductor: R = 2*L*fs
    let r_r = r_ohms;                         // Resistor: R = R
    let r_total = r_l + r_r;

    // Scattering coefficients for series adaptor
    let gamma_l = r_l / r_total;
    let gamma_r = r_r / r_total;

    let mut output = Vec::with_capacity(signal.len());
    let mut a_l_prev = 0.0; // Inductor state

    for &v_in in signal {
        // 1. Source sets incident wave (wave = 2 * voltage)
        let a_s = 2.0 * v_in;

        // 2. Inductor reflects previous incident wave
        let b_l = a_l_prev;

        // 3. Resistor reflects zero (matched load)
        // let b_r = 0.0;

        // 4. Series adaptor scattering
        let sum = a_s + b_l; // + b_r, but b_r = 0
        let a_l = a_s - gamma_l * sum;
        let a_r = a_s - gamma_r * sum;

        // 5. Update inductor state
        a_l_prev = a_l;

        // 6. Output voltage across resistor: v_out = (a_R + b_R) / 2 = a_R / 2
        let v_out = a_r / 2.0;
        output.push(v_out);
    }

    output
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rc_lowpass_impulse_decays() {
        let r = 10_000.0; // 10k
        let c = 10e-9;    // 10nF
        let sr = 96000.0;
        let ir = rc_lowpass_impulse_response(r, c, sr, 1000);

        // First sample should be positive
        assert!(ir[0] > 0.0);

        // Find peak (bilinear transform may have peak after first sample)
        let peak_idx = ir.iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .map(|(i, _)| i)
            .unwrap();

        // Peak should be early (within first 10 samples)
        assert!(peak_idx < 10, "Peak should be near the start, got idx {}", peak_idx);

        // After peak, should decay monotonically
        for i in (peak_idx + 1)..ir.len() {
            assert!(
                ir[i] <= ir[i - 1] + 1e-10, // Allow tiny numerical tolerance
                "Impulse response should decay after peak at sample {}", i
            );
        }

        // Should approach zero by end
        let peak_val = ir[peak_idx];
        assert!(ir[999] < peak_val * 0.01, "Should decay to <1% of peak");
    }

    #[test]
    fn rc_lowpass_cutoff_is_3db() {
        let r = 10_000.0;
        let c = 10e-9;
        let fc = 1.0 / (2.0 * PI * r * c); // ~1591 Hz

        let mag_at_fc = rc_lowpass_magnitude(r, c, fc);
        let db_at_fc = 20.0 * mag_at_fc.log10();

        // Should be -3dB at cutoff
        assert!(
            (db_at_fc + 3.0).abs() < 0.1,
            "Magnitude at fc should be -3dB, got {db_at_fc}"
        );
    }

    #[test]
    fn rc_lowpass_passband_is_unity() {
        let r = 10_000.0;
        let c = 10e-9;

        let mag_dc = rc_lowpass_magnitude(r, c, 0.0);
        assert!((mag_dc - 1.0).abs() < 1e-10, "DC gain should be unity");
    }
}
