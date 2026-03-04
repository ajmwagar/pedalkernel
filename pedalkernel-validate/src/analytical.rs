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

/// First-order RC highpass filter impulse response.
///
/// The continuous-time transfer function is:
///   H(s) = sRC / (1 + sRC)
/// where τ = RC.
///
/// Bilinear transform: s = (2/T) * (1 - z^-1) / (1 + z^-1)
/// This gives: H(z) = (b0 + b1*z^-1) / (1 + a1*z^-1)
/// where:
///   K = 2/T, Kτ = K*τ
///   b0 = Kτ / (1 + Kτ)
///   b1 = -Kτ / (1 + Kτ) = -b0
///   a1 = (1 - Kτ) / (1 + Kτ)
pub fn rc_highpass_impulse_response(
    r_ohms: f64,
    c_farads: f64,
    sample_rate: f64,
    n_samples: usize,
) -> Vec<f64> {
    let tau = r_ohms * c_farads;
    let dt = 1.0 / sample_rate;

    let k = 2.0 / dt;
    let kt = k * tau;
    let denom = 1.0 + kt;
    let b0 = kt / denom;
    let b1 = -b0;
    let a1 = (1.0 - kt) / denom;

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

/// Process a signal through an ideal first-order RC highpass.
///
/// Uses bilinear transform for exact discrete-time response.
/// H(s) = sRC / (1 + sRC)
pub fn rc_highpass_filter(signal: &[f64], r_ohms: f64, c_farads: f64, sample_rate: f64) -> Vec<f64> {
    let tau = r_ohms * c_farads;
    let dt = 1.0 / sample_rate;

    let k = 2.0 / dt;
    let kt = k * tau;
    let denom = 1.0 + kt;
    let b0 = kt / denom;
    let b1 = -b0;
    let a1 = (1.0 - kt) / denom;

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

/// Series RLC bandpass filter.
///
/// For a series RLC circuit with output across the load resistor RL
/// in the topology: Vin -> R_series -> L -> C -> RL -> GND
///
/// The transfer function considering the load resistor is:
///   H(s) = RL / (R + sL + 1/(sC) + RL)
///        = s * RL * C / (s^2 * L * C + s * (R + RL) * C + 1)
///
/// This is a bandpass with:
///   f0 = 1 / (2*pi*sqrt(L*C))
///   Q = sqrt(L/C) / (R + RL)
///
/// Uses bilinear transform to convert to discrete-time biquad.
pub fn rlc_bandpass_filter(
    signal: &[f64],
    r_ohms: f64,
    l_henrys: f64,
    c_farads: f64,
    r_load: f64,
    sample_rate: f64,
) -> Vec<f64> {
    let dt = 1.0 / sample_rate;

    // Continuous-time coefficients for H(s) = (RL*C*s) / (L*C*s^2 + (R+RL)*C*s + 1)
    // In standard biquad form: H(s) = (b_s * s) / (a2 * s^2 + a1 * s + a0)
    let b_s = r_load * c_farads;
    let a2 = l_henrys * c_farads;
    let a1 = (r_ohms + r_load) * c_farads;
    let a0 = 1.0;

    // Bilinear transform: s = (2/T)(1 - z^-1)/(1 + z^-1)
    let k = 2.0 / dt;
    let k2 = k * k;

    // Substitute into H(s) and collect z^-1, z^-2 terms
    // Numerator: b_s * k * (1 - z^-1) / (1 + z^-1)
    // Denominator: a2 * k^2 * (1-z^-1)^2/(1+z^-1)^2 + a1 * k * (1-z^-1)/(1+z^-1) + a0
    //
    // Multiply through by (1+z^-1)^2:
    // Num: b_s * k * (1 - z^-1)(1 + z^-1) = b_s * k * (1 - z^-2)
    // Den: a2 * k^2 * (1 - z^-1)^2 + a1 * k * (1 - z^-1)(1 + z^-1) + a0 * (1 + z^-1)^2
    //    = a2*k2*(1 - 2z^-1 + z^-2) + a1*k*(1 - z^-2) + a0*(1 + 2z^-1 + z^-2)

    let d0 = a2 * k2 + a1 * k + a0;
    let d1 = -2.0 * a2 * k2 + 2.0 * a0;
    let d2 = a2 * k2 - a1 * k + a0;

    let n0 = b_s * k;
    // n1 = 0 (the z^-1 coefficient of (1 - z^-2) is 0)
    let n2 = -b_s * k;

    // Normalize by d0
    let b0 = n0 / d0;
    let b1_coeff = 0.0; // n1/d0
    let b2 = n2 / d0;
    let a1_coeff = d1 / d0;
    let a2_coeff = d2 / d0;

    // Apply biquad: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    let mut output = Vec::with_capacity(signal.len());
    let mut x1 = 0.0;
    let mut x2 = 0.0;
    let mut y1 = 0.0;
    let mut y2 = 0.0;

    for &x in signal {
        let y = b0 * x + b1_coeff * x1 + b2 * x2 - a1_coeff * y1 - a2_coeff * y2;
        output.push(y);
        x2 = x1;
        x1 = x;
        y2 = y1;
        y1 = y;
    }

    output
}

/// Twin-T notch filter.
///
/// Classic Twin-T topology with R, R, R/2, C, C, 2C.
/// The ideal transfer function (with infinite impedance output) is:
///   H(s) = (s^2*R^2*C^2 + 1) / (s^2*R^2*C^2 + 4*s*R*C + 1)
///
/// This gives a notch at f_notch = 1/(2*pi*R*C) with notch Q determined
/// by the topology. The load resistor RL modifies the depth of the notch.
///
/// For the ideal case (no load), uses bilinear transform to biquad IIR.
pub fn twin_t_notch_filter(
    signal: &[f64],
    r_ohms: f64,
    c_farads: f64,
    sample_rate: f64,
) -> Vec<f64> {
    let dt = 1.0 / sample_rate;
    let rc = r_ohms * c_farads;
    let rc2 = rc * rc;

    // H(s) = (rc2 * s^2 + 1) / (rc2 * s^2 + 4*rc*s + 1)
    // Numerator: a2n*s^2 + a0n = rc2*s^2 + 1
    // Denominator: a2d*s^2 + a1d*s + a0d = rc2*s^2 + 4*rc*s + 1

    let k = 2.0 / dt;
    let k2 = k * k;

    // Bilinear substitution and multiply by (1+z^-1)^2:
    // s^2 -> k^2 * (1 - z^-1)^2 / (1 + z^-1)^2
    // s -> k * (1 - z^-1) / (1 + z^-1)
    //
    // Numerator * (1+z^-1)^2:
    //   rc2 * k2 * (1-z^-1)^2 + 1 * (1+z^-1)^2
    //   = (rc2*k2 + 1) + (-2*rc2*k2 + 2)*z^-1 + (rc2*k2 + 1)*z^-2
    //
    // Denominator * (1+z^-1)^2:
    //   rc2 * k2 * (1-z^-1)^2 + 4*rc*k * (1-z^-1)(1+z^-1) + 1*(1+z^-1)^2
    //   = (rc2*k2 + 4*rc*k + 1) + (-2*rc2*k2 + 2)*z^-1 + (rc2*k2 - 4*rc*k + 1)*z^-2

    let n0 = rc2 * k2 + 1.0;
    let n1 = -2.0 * rc2 * k2 + 2.0;
    let n2 = rc2 * k2 + 1.0;

    let d0 = rc2 * k2 + 4.0 * rc * k + 1.0;
    let d1 = -2.0 * rc2 * k2 + 2.0;
    let d2 = rc2 * k2 - 4.0 * rc * k + 1.0;

    // Normalize
    let b0 = n0 / d0;
    let b1_coeff = n1 / d0;
    let b2 = n2 / d0;
    let a1_coeff = d1 / d0;
    let a2_coeff = d2 / d0;

    let mut output = Vec::with_capacity(signal.len());
    let mut x1 = 0.0;
    let mut x2 = 0.0;
    let mut y1 = 0.0;
    let mut y2 = 0.0;

    for &x in signal {
        let y = b0 * x + b1_coeff * x1 + b2 * x2 - a1_coeff * y1 - a2_coeff * y2;
        output.push(y);
        x2 = x1;
        x1 = x;
        y2 = y1;
        y1 = y;
    }

    output
}

/// Process a signal through an ideal first-order RC lowpass.
///
/// Uses bilinear transform for exact discrete-time response.
pub fn rc_lowpass_filter(signal: &[f64], r_ohms: f64, c_farads: f64, sample_rate: f64) -> Vec<f64> {
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
/// Uses bilinear transform for exact discrete-time response.
/// H(s) = R / (R + sL) = 1 / (1 + sL/R)
/// Time constant τ = L/R (same form as RC lowpass with τ = RC)
///
/// This gives fc = R / (2*pi*L)
pub fn rl_lowpass_filter(signal: &[f64], l_henrys: f64, r_ohms: f64, sample_rate: f64) -> Vec<f64> {
    // Time constant τ = L/R
    let tau = l_henrys / r_ohms;
    let dt = 1.0 / sample_rate;

    // Bilinear transform coefficients (same form as RC lowpass)
    // α = 2*fs*τ
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rc_lowpass_impulse_decays() {
        let r = 10_000.0; // 10k
        let c = 10e-9; // 10nF
        let sr = 96000.0;
        let ir = rc_lowpass_impulse_response(r, c, sr, 1000);

        // First sample should be positive
        assert!(ir[0] > 0.0);

        // Find peak (bilinear transform may have peak after first sample)
        let peak_idx = ir
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .map(|(i, _)| i)
            .unwrap();

        // Peak should be early (within first 10 samples)
        assert!(
            peak_idx < 10,
            "Peak should be near the start, got idx {}",
            peak_idx
        );

        // After peak, should decay monotonically
        for i in (peak_idx + 1)..ir.len() {
            assert!(
                ir[i] <= ir[i - 1] + 1e-10, // Allow tiny numerical tolerance
                "Impulse response should decay after peak at sample {}",
                i
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

    #[test]
    fn rc_highpass_blocks_dc() {
        let r = 33_000.0;
        let c = 22e-9;
        let sr = 96000.0;

        // Feed DC signal through highpass — should output zero after transient
        let dc_input = vec![1.0; 10000];
        let output = rc_highpass_filter(&dc_input, r, c, sr);

        // Last sample should be very close to zero (DC blocked)
        assert!(
            output.last().unwrap().abs() < 1e-3,
            "Highpass should block DC, got {}",
            output.last().unwrap()
        );
    }

    #[test]
    fn rc_highpass_passes_high_freq() {
        let r = 33_000.0;
        let c = 22e-9;
        let sr = 96000.0;
        let fc = 1.0 / (2.0 * PI * r * c); // ~219 Hz

        // Test with sine well above cutoff (10x fc)
        let f_test = fc * 10.0;
        let duration = 0.1;
        let n = (sr * duration) as usize;
        let input: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f_test * i as f64 / sr).sin())
            .collect();

        let output = rc_highpass_filter(&input, r, c, sr);

        // RMS of output should be close to RMS of input (near unity gain)
        let in_rms: f64 = (input.iter().map(|x| x * x).sum::<f64>() / n as f64).sqrt();
        let out_rms: f64 = (output.iter().map(|x| x * x).sum::<f64>() / n as f64).sqrt();
        let ratio = out_rms / in_rms;

        assert!(
            ratio > 0.95,
            "Highpass should pass high frequencies, ratio = {}",
            ratio
        );
    }

    #[test]
    fn rlc_bandpass_peaks_at_resonance() {
        // Use smaller R_load to get higher Q for a clearer bandpass peak
        let r = 100.0_f64;
        let l = 10e-3_f64;
        let c = 100e-9_f64;
        let r_load = 100.0_f64; // Small load for higher Q
        let sr = 96000.0 * 4.0; // High sample rate for accuracy
        let f0 = 1.0 / (2.0 * PI * (l * c).sqrt()); // ~5033 Hz
        let duration = 0.1;
        let n = (sr * duration) as usize;

        // Test at resonance
        let input_res: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f0 * i as f64 / sr).sin())
            .collect();
        let output_res = rlc_bandpass_filter(&input_res, r, l, c, r_load, sr);

        // Test well below resonance (f0/20)
        let f_low = f0 / 20.0;
        let input_low: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f_low * i as f64 / sr).sin())
            .collect();
        let output_low = rlc_bandpass_filter(&input_low, r, l, c, r_load, sr);

        // Use second half to avoid transients
        let half = n / 2;
        let rms_res: f64 = (output_res[half..]
            .iter()
            .map(|x| x * x)
            .sum::<f64>()
            / (n - half) as f64)
            .sqrt();
        let rms_low: f64 = (output_low[half..]
            .iter()
            .map(|x| x * x)
            .sum::<f64>()
            / (n - half) as f64)
            .sqrt();

        assert!(
            rms_res > rms_low * 1.5,
            "Response at resonance ({:.3}) should be greater than at {:.0}Hz ({:.3})",
            rms_res,
            f_low,
            rms_low
        );
    }

    #[test]
    fn twin_t_notch_at_design_frequency() {
        let r = 10_000.0;
        let c = 10e-9;
        let sr = 96000.0 * 4.0;
        let f_notch = 1.0 / (2.0 * PI * r * c); // ~1591 Hz
        let duration = 0.1;
        let n = (sr * duration) as usize;

        // Test at notch frequency
        let input_notch: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f_notch * i as f64 / sr).sin())
            .collect();
        let output_notch = twin_t_notch_filter(&input_notch, r, c, sr);

        // Test away from notch (5x)
        let f_pass = f_notch * 5.0;
        let input_pass: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f_pass * i as f64 / sr).sin())
            .collect();
        let output_pass = twin_t_notch_filter(&input_pass, r, c, sr);

        // Use second half
        let half = n / 2;
        let rms_notch: f64 = (output_notch[half..]
            .iter()
            .map(|x| x * x)
            .sum::<f64>()
            / (n - half) as f64)
            .sqrt();
        let rms_pass: f64 = (output_pass[half..]
            .iter()
            .map(|x| x * x)
            .sum::<f64>()
            / (n - half) as f64)
            .sqrt();

        // At the notch, output should be much smaller
        assert!(
            rms_notch < rms_pass * 0.1,
            "Notch response ({:.6}) should be much less than passband ({:.6})",
            rms_notch,
            rms_pass
        );
    }
}
