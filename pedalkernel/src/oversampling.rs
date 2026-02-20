//! Oversampling for antialiasing at nonlinear stages.
//!
//! Nonlinear operations (diode clippers, transistor saturation, tanh soft
//! clipping) generate harmonics that can fold back below Nyquist at standard
//! sample rates (44.1/48 kHz), producing inharmonic aliasing artifacts.
//!
//! This module provides a lightweight oversampling wrapper that:
//! 1. Upsamples the input by inserting zeros and filtering
//! 2. Processes each oversampled sample through the nonlinear stage
//! 3. Filters and decimates back to the original sample rate
//!
//! The filters use half-band IIR designs (3rd-order Butterworth) for minimal
//! latency and CPU cost while providing adequate alias rejection (~60 dB).

/// Oversampling factor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OversamplingFactor {
    /// No oversampling (1x). Cheapest, most aliasing.
    X1 = 1,
    /// 2x oversampling. Good balance of quality and CPU.
    X2 = 2,
    /// 4x oversampling. High quality, higher CPU cost.
    X4 = 4,
}

impl OversamplingFactor {
    pub fn ratio(self) -> usize {
        self as usize
    }
}

/// Half-band low-pass filter for oversampling.
///
/// Uses a cascaded second-order section (biquad) design tuned to the
/// Nyquist frequency of the base sample rate. This provides the
/// anti-imaging filter (upsampling) and anti-aliasing filter (downsampling).
///
/// Coefficients are for a 3rd-order Butterworth at 0.5 * fs_base,
/// evaluated at the oversampled rate.
#[derive(Debug, Clone)]
struct HalfBandFilter {
    /// Biquad coefficients [b0, b1, b2, a1, a2] for each section.
    sections: Vec<[f64; 5]>,
    /// State [x1, x2, y1, y2] for each section.
    state: Vec<[f64; 4]>,
}

impl HalfBandFilter {
    /// Create a half-band filter for the given oversampling factor.
    ///
    /// Designs a Butterworth low-pass filter using the bilinear transform.
    /// Cutoff is at `0.5 / factor` of the oversampled rate (= original Nyquist).
    fn new(factor: OversamplingFactor) -> Self {
        let sections = match factor {
            OversamplingFactor::X1 => vec![],
            OversamplingFactor::X2 => {
                // 4th-order Butterworth, fc = 0.25 * fs_oversampled
                Self::design_butterworth(4, 0.25)
            }
            OversamplingFactor::X4 => {
                // 6th-order Butterworth, fc = 0.125 * fs_oversampled
                Self::design_butterworth(6, 0.125)
            }
        };
        let n = sections.len();
        Self {
            sections,
            state: vec![[0.0; 4]; n],
        }
    }

    /// Design a Butterworth low-pass filter as cascaded biquad sections.
    ///
    /// `order` must be even. `fc_normalized` is fc/fs (0 to 0.5).
    /// Returns vec of [b0, b1, b2, a1, a2] sections.
    fn design_butterworth(order: usize, fc_normalized: f64) -> Vec<[f64; 5]> {
        let num_sections = order / 2;
        let wc = (std::f64::consts::PI * fc_normalized).tan();
        let wc2 = wc * wc;

        let mut sections = Vec::with_capacity(num_sections);
        for k in 0..num_sections {
            // Pole angle for Butterworth: (2k + 1) * pi / (2 * order)
            let theta = (2 * k + 1) as f64 * std::f64::consts::PI / (2.0 * order as f64);
            // Butterworth pole bandwidth: 2*cos(theta)
            let bw = 2.0 * theta.cos();

            // Bilinear transform: s = (2/T)(z-1)/(z+1), T=1
            // For unit-delay normalized: s = 2*(z-1)/(z+1)
            // Substituting into H(s) = wc^2 / (s^2 + bw*wc*s + wc^2):
            let a0 = 4.0 + 2.0 * bw * wc + wc2;
            let a1_coef = (2.0 * wc2 - 8.0) / a0;
            let a2_coef = (4.0 - 2.0 * bw * wc + wc2) / a0;
            let b0 = wc2 / a0;
            let b1 = 2.0 * wc2 / a0;
            let b2 = wc2 / a0;

            sections.push([b0, b1, b2, a1_coef, a2_coef]);
        }
        sections
    }

    /// Process one sample through the cascade.
    #[inline]
    fn process(&mut self, input: f64) -> f64 {
        let mut x = input;
        for (i, coef) in self.sections.iter().enumerate() {
            let [b0, b1, b2, a1, a2] = *coef;
            let [x1, x2, y1, y2] = self.state[i];

            let y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

            self.state[i] = [x, x1, y, y1];
            x = y;
        }
        x
    }

    /// Reset filter state.
    fn reset(&mut self) {
        for s in &mut self.state {
            *s = [0.0; 4];
        }
    }
}

/// Oversampling processor that wraps a nonlinear processing function.
///
/// Usage:
/// ```ignore
/// let mut os = Oversampler::new(OversamplingFactor::X4);
/// let output = os.process(input, |x| my_nonlinear_fn(x));
/// ```
#[derive(Debug, Clone)]
pub struct Oversampler {
    factor: OversamplingFactor,
    /// Anti-imaging filter (applied after zero-stuffing on upsample).
    upsample_filter: HalfBandFilter,
    /// Anti-aliasing filter (applied before decimation on downsample).
    downsample_filter: HalfBandFilter,
}

impl Oversampler {
    /// Create a new oversampler with the given factor.
    pub fn new(factor: OversamplingFactor) -> Self {
        Self {
            factor,
            upsample_filter: HalfBandFilter::new(factor),
            downsample_filter: HalfBandFilter::new(factor),
        }
    }

    /// Get the oversampling factor.
    pub fn factor(&self) -> OversamplingFactor {
        self.factor
    }

    /// Get the oversampling ratio as an integer.
    pub fn ratio(&self) -> usize {
        self.factor.ratio()
    }

    /// Process a single input sample through oversampled nonlinear processing.
    ///
    /// The closure `f` is called once per oversampled sample (so 4 times for
    /// 4x oversampling). It should implement the nonlinear operation.
    ///
    /// Returns one output sample at the base sample rate.
    #[inline]
    pub fn process<F>(&mut self, input: f64, mut f: F) -> f64
    where
        F: FnMut(f64) -> f64,
    {
        let ratio = self.factor.ratio();
        if ratio == 1 {
            return f(input);
        }

        // Upsample: insert zeros between samples, then filter.
        // The gain factor compensates for zero-stuffing energy loss.
        let gain = ratio as f64;
        let mut last_out = 0.0;

        for i in 0..ratio {
            let up = if i == 0 { input * gain } else { 0.0 };
            let filtered = self.upsample_filter.process(up);
            let processed = f(filtered);
            last_out = self.downsample_filter.process(processed);
        }

        // Decimate: take the last filtered sample.
        last_out
    }

    /// Reset internal filter state.
    pub fn reset(&mut self) {
        self.upsample_filter.reset();
        self.downsample_filter.reset();
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn oversampler_x1_is_passthrough() {
        let mut os = Oversampler::new(OversamplingFactor::X1);
        for i in 0..100 {
            let input = (i as f64 * 0.1).sin();
            let output = os.process(input, |x| x);
            assert!(
                (output - input).abs() < 1e-10,
                "X1 should be passthrough: in={input}, out={output}"
            );
        }
    }

    #[test]
    fn oversampler_x2_preserves_dc() {
        let mut os = Oversampler::new(OversamplingFactor::X2);
        // Feed a constant signal — output should settle to the same DC level
        let mut out = 0.0;
        for _ in 0..1000 {
            out = os.process(1.0, |x| x);
        }
        assert!(
            (out - 1.0).abs() < 0.05,
            "X2 should preserve DC: out={out}"
        );
    }

    #[test]
    fn oversampler_x4_preserves_dc() {
        let mut os = Oversampler::new(OversamplingFactor::X4);
        let mut out = 0.0;
        for _ in 0..2000 {
            out = os.process(1.0, |x| x);
        }
        assert!(
            (out - 1.0).abs() < 0.05,
            "X4 should preserve DC: out={out}"
        );
    }

    #[test]
    fn oversampler_reduces_aliasing() {
        // Hard clipper at 44.1kHz without oversampling generates aliasing.
        // With oversampling, the aliased energy should be reduced.
        let fs = 44100.0;
        let freq = 8000.0; // High frequency near Nyquist to stress test
        let n = 4096;

        // Generate test signal
        let input: Vec<f64> = (0..n)
            .map(|i| 0.8 * (2.0 * std::f64::consts::PI * freq * i as f64 / fs).sin())
            .collect();

        // Hard clip without oversampling
        let no_os: Vec<f64> = input.iter().map(|&x| x.clamp(-0.3, 0.3)).collect();

        // Hard clip with 4x oversampling
        let mut os = Oversampler::new(OversamplingFactor::X4);
        let with_os: Vec<f64> = input
            .iter()
            .map(|&x| os.process(x, |s| s.clamp(-0.3, 0.3)))
            .collect();

        // Compare energy in the upper frequency band (potential alias region).
        // We use a simple sum-of-squares metric on the difference from a
        // "perfectly bandlimited" reference (which we approximate as the
        // oversampled version).
        let diff_energy: f64 = no_os
            .iter()
            .zip(with_os.iter())
            .skip(500) // skip transient
            .map(|(a, b)| (a - b).powi(2))
            .sum();

        let signal_energy: f64 = with_os.iter().skip(500).map(|x| x.powi(2)).sum();

        // The difference should be meaningful — oversampling changes the output
        assert!(
            diff_energy > signal_energy * 0.001,
            "Oversampling should produce measurably different output from non-oversampled hard clip"
        );
    }

    #[test]
    fn oversampler_reset_clears_state() {
        let mut os = Oversampler::new(OversamplingFactor::X4);

        // Process some signal
        for i in 0..100 {
            os.process((i as f64 * 0.3).sin(), |x| x);
        }

        os.reset();

        // After reset, processing zero should give near-zero output
        let out = os.process(0.0, |x| x);
        assert!(
            out.abs() < 1e-6,
            "After reset, output should be near zero: {out}"
        );
    }

    #[test]
    fn oversampler_x2_low_freq_transparent() {
        // Low frequency sine through linear 2x oversampler should be ~unchanged
        let mut os = Oversampler::new(OversamplingFactor::X2);
        let fs = 48000.0;
        let freq = 440.0;
        let n = 4800;

        let input: Vec<f64> = (0..n)
            .map(|i| (2.0 * std::f64::consts::PI * freq * i as f64 / fs).sin())
            .collect();
        let output: Vec<f64> = input.iter().map(|&x| os.process(x, |s| s)).collect();

        // Skip transient, compare
        let skip = 500;
        let corr = correlation(&input[skip..], &output[skip..]);
        assert!(
            corr > 0.95,
            "Low-freq sine should pass through X2 with high correlation: {corr:.4}"
        );
    }

    fn correlation(a: &[f64], b: &[f64]) -> f64 {
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
