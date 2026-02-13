//! Pedal implementations backed by the WDF engine.
//!
//! Each pedal maps knob positions to WDF tree parameters and exposes
//! the `PedalProcessor` trait for integration with the audio engine.

use crate::elements::*;
use crate::tree::*;

/// Tube Screamer-style overdrive using a WDF diode clipper.
///
/// Models the TS808 topology: an op-amp gain stage driving anti-parallel
/// silicon clipping diodes in the feedback loop.  The WDF engine handles
/// the nonlinear diode clipping; `pre_gain` models the op-amp.
///
/// - **Drive** knob: controls both op-amp gain (2×–100×) and feedback
///   resistance (tone/clipping character).
/// - **Level** knob: output attenuation.
pub struct Overdrive {
    clipper: WdfClipper,
    gain: f64,
    level: f64,
    pre_gain: f64,
    base_resistance: f64,
    sample_rate: f64,
}

impl Overdrive {
    pub fn new(sample_rate: f64) -> Self {
        let base_r = 4700.0; // 4.7 kΩ
        let mut od = Self {
            clipper: WdfClipper::new(
                base_r,
                47e-9,  // 47 nF
                DiodeModel::silicon(),
                sample_rate,
            ),
            gain: 0.5,
            level: 0.8,
            pre_gain: 1.0,
            base_resistance: base_r,
            sample_rate,
        };
        od.set_gain(0.5);
        od
    }

    pub fn set_gain(&mut self, gain: f64) {
        self.gain = gain.clamp(0.0, 1.0);
        // Pre-gain models the TS808 op-amp gain stage.
        // gain=0 → 2× (barely clips), gain=1 → 100× (heavy saturation).
        self.pre_gain = 2.0 * 50.0_f64.powf(self.gain);
        // Resistance varies with gain (affects clipping tone character).
        // Higher gain → lower R → sharper knee.
        let r = self.base_resistance * (1.0 - 0.9 * self.gain) + 51.0;
        self.clipper = WdfClipper::new(r, 47e-9, DiodeModel::silicon(), self.sample_rate);
    }

    pub fn set_level(&mut self, level: f64) {
        self.level = level.clamp(0.0, 1.0);
    }
}

impl crate::PedalProcessor for Overdrive {
    fn process(&mut self, input: f64) -> f64 {
        // The diode voltage output is naturally bounded by silicon
        // forward voltage (~±0.7–1.0V), so no post-normalization needed.
        self.clipper.process(input * self.pre_gain) * self.level
    }

    fn set_sample_rate(&mut self, rate: f64) {
        self.sample_rate = rate;
        self.clipper.set_sample_rate(rate);
    }

    fn reset(&mut self) {
        self.clipper.reset();
    }
}

/// Fuzz Face-style distortion using a WDF single-diode clipper.
///
/// Models a two-transistor germanium fuzz with asymmetric clipping.
/// `pre_gain` models the cascaded transistor gain stages.
///
/// - **Fuzz** knob: controls transistor drive (5×–250×) and bias resistance.
/// - **Volume** knob: output attenuation with makeup gain for germanium's
///   lower clipping voltage.
pub struct FuzzFace {
    clipper: WdfSingleDiodeClipper,
    fuzz: f64,
    volume: f64,
    pre_gain: f64,
    sample_rate: f64,
}

impl FuzzFace {
    pub fn new(sample_rate: f64) -> Self {
        let mut ff = Self {
            clipper: WdfSingleDiodeClipper::new(
                1000.0,
                100e-9,
                DiodeModel::germanium(),
                sample_rate,
            ),
            fuzz: 0.5,
            volume: 0.8,
            pre_gain: 1.0,
            sample_rate,
        };
        ff.set_fuzz(0.5);
        ff
    }

    pub fn set_fuzz(&mut self, fuzz: f64) {
        self.fuzz = fuzz.clamp(0.0, 1.0);
        // Pre-gain models two cascaded transistor gain stages.
        // fuzz=0 → 5× (mild grit), fuzz=1 → 250× (full splat).
        self.pre_gain = 5.0 * 50.0_f64.powf(self.fuzz);
        let r = 10000.0 * (1.0 - 0.95 * self.fuzz) + 100.0;
        self.clipper = WdfSingleDiodeClipper::new(r, 100e-9, DiodeModel::germanium(), self.sample_rate);
    }

    pub fn set_volume(&mut self, volume: f64) {
        self.volume = volume.clamp(0.0, 1.0);
    }
}

/// Germanium diodes clip at ~0.3V vs silicon's ~0.7V.
/// Scale factor to bring the germanium clip voltage into useful range
/// before the tanh soft-limiter.
const GERMANIUM_SCALE: f64 = 3.0;

impl crate::PedalProcessor for FuzzFace {
    fn process(&mut self, input: f64) -> f64 {
        let raw = self.clipper.process(input * self.pre_gain);
        // The single-diode clipper only clips one polarity; the reverse-bias
        // half passes through linearly and can be very large.  tanh() provides
        // a musically useful soft limit on both halves while preserving the
        // asymmetric germanium character on the clipped side.
        (raw * GERMANIUM_SCALE).tanh() * self.volume
    }

    fn set_sample_rate(&mut self, rate: f64) {
        self.sample_rate = rate;
        self.clipper.set_sample_rate(rate);
    }

    fn reset(&mut self) {
        self.clipper.reset();
    }
}

/// Simple delay pedal (non-WDF, digital delay line).
pub struct Delay {
    delay_time_ms: f64,
    feedback: f64,
    mix: f64,
    buffer: Vec<f64>,
    write_pos: usize,
    sample_rate: f64,
}

impl Delay {
    pub fn new(sample_rate: f64) -> Self {
        let max_delay_ms = 2000.0;
        let buffer_size = (max_delay_ms / 1000.0 * sample_rate) as usize;
        Self {
            delay_time_ms: 300.0,
            feedback: 0.4,
            mix: 0.5,
            buffer: vec![0.0; buffer_size],
            write_pos: 0,
            sample_rate,
        }
    }

    pub fn set_delay_time(&mut self, ms: f64) {
        self.delay_time_ms = ms.clamp(10.0, 2000.0);
    }

    pub fn set_feedback(&mut self, feedback: f64) {
        self.feedback = feedback.clamp(0.0, 0.95);
    }

    pub fn set_mix(&mut self, mix: f64) {
        self.mix = mix.clamp(0.0, 1.0);
    }
}

impl crate::PedalProcessor for Delay {
    fn process(&mut self, input: f64) -> f64 {
        let delay_samples = (self.delay_time_ms / 1000.0 * self.sample_rate) as usize;
        let read_pos = if self.write_pos >= delay_samples {
            self.write_pos - delay_samples
        } else {
            self.buffer.len() - (delay_samples - self.write_pos)
        };

        let delayed = self.buffer[read_pos];
        let output = input + delayed * self.feedback;

        self.buffer[self.write_pos] = output;
        self.write_pos = (self.write_pos + 1) % self.buffer.len();

        input * (1.0 - self.mix) + delayed * self.mix
    }

    fn set_sample_rate(&mut self, rate: f64) {
        self.sample_rate = rate;
        let max_delay_ms = 2000.0;
        let buffer_size = (max_delay_ms / 1000.0 * rate) as usize;
        self.buffer = vec![0.0; buffer_size];
        self.write_pos = 0;
    }

    fn reset(&mut self) {
        self.buffer.fill(0.0);
        self.write_pos = 0;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PedalProcessor;

    const SR: f64 = 48000.0;
    const N: usize = 48000; // 1 second at 48 kHz

    // ── Signal analysis helpers ────────────────────────────────────

    /// Pearson correlation between two equal-length slices.
    fn correlation(a: &[f64], b: &[f64]) -> f64 {
        let n = a.len() as f64;
        let ma = a.iter().sum::<f64>() / n;
        let mb = b.iter().sum::<f64>() / n;
        let (mut cov, mut va, mut vb) = (0.0, 0.0, 0.0);
        for i in 0..a.len() {
            let (da, db) = (a[i] - ma, b[i] - mb);
            cov += da * db;
            va += da * da;
            vb += db * db;
        }
        if va == 0.0 || vb == 0.0 { return 0.0; }
        cov / (va.sqrt() * vb.sqrt())
    }

    fn rms(buf: &[f64]) -> f64 {
        (buf.iter().map(|x| x * x).sum::<f64>() / buf.len() as f64).sqrt()
    }

    fn peak(buf: &[f64]) -> f64 {
        buf.iter().fold(0.0_f64, |m, x| m.max(x.abs()))
    }

    fn sine(freq: f64, amplitude: f64, n_samples: usize) -> Vec<f64> {
        sine_at(freq, amplitude, n_samples, SR)
    }

    fn sine_at(freq: f64, amplitude: f64, n_samples: usize, sample_rate: f64) -> Vec<f64> {
        (0..n_samples)
            .map(|i| {
                let t = i as f64 / sample_rate;
                amplitude * (2.0 * std::f64::consts::PI * freq * t).sin()
            })
            .collect()
    }

    fn process_buf(proc: &mut dyn PedalProcessor, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&s| proc.process(s)).collect()
    }

    /// Estimate THD by comparing power in the fundamental vs total power.
    /// Returns a value in [0, 1]: 0 = pure sine, 1 = all harmonics.
    fn estimate_thd(buf: &[f64], fundamental_hz: f64, sample_rate: f64) -> f64 {
        let n = buf.len();
        // Simple DFT bin for fundamental
        let bin_f = (fundamental_hz * n as f64 / sample_rate).round() as usize;
        let mut re = 0.0_f64;
        let mut im = 0.0_f64;
        for (i, &s) in buf.iter().enumerate() {
            let angle = 2.0 * std::f64::consts::PI * bin_f as f64 * i as f64 / n as f64;
            re += s * angle.cos();
            im += s * angle.sin();
        }
        let fundamental_power = (re * re + im * im) / (n as f64 * n as f64);
        let total_power: f64 = buf.iter().map(|x| x * x).sum::<f64>() / n as f64;
        if total_power < 1e-20 { return 0.0; }
        let harmonic_power = (total_power - fundamental_power).max(0.0);
        (harmonic_power / total_power).sqrt()
    }

    // ══════════════════════════════════════════════════════════════════
    //  Overdrive tests
    // ══════════════════════════════════════════════════════════════════

    #[test]
    fn overdrive_produces_output() {
        let input = sine(440.0, 0.5, 4800);
        let mut od = Overdrive::new(SR);
        let output = process_buf(&mut od, &input);
        assert!(peak(&output) > 0.01, "overdrive should produce output");
    }

    #[test]
    fn overdrive_high_gain_is_not_a_wire() {
        let input = sine(440.0, 0.5, N);
        let mut od = Overdrive::new(SR);
        od.set_gain(0.9);
        od.set_level(1.0);
        let output = process_buf(&mut od, &input);
        let corr = correlation(&input, &output).abs();
        // A clipped sine still shares the fundamental frequency so |corr| ≈ 0.9.
        // Pre-fix this was 0.9999 (no distortion at all). We require < 0.98.
        assert!(
            corr < 0.98,
            "Overdrive gain=0.9 should distort: |corr|={corr:.4}"
        );
    }

    #[test]
    fn overdrive_low_gain_closer_to_clean() {
        let input = sine(440.0, 0.5, N);
        let mut od = Overdrive::new(SR);
        od.set_gain(0.0);
        od.set_level(1.0);
        let output = process_buf(&mut od, &input);
        let corr = correlation(&input, &output).abs();
        // Low gain should still be somewhat close to clean
        assert!(
            corr > 0.5,
            "Overdrive gain=0.0 should be mostly clean: |corr|={corr:.4}"
        );
    }

    #[test]
    fn overdrive_more_gain_means_more_distortion() {
        let input = sine(440.0, 0.5, N);

        let mut od_lo = Overdrive::new(SR);
        od_lo.set_gain(0.1);
        od_lo.set_level(1.0);
        let out_lo = process_buf(&mut od_lo, &input);

        let mut od_hi = Overdrive::new(SR);
        od_hi.set_gain(0.9);
        od_hi.set_level(1.0);
        let out_hi = process_buf(&mut od_hi, &input);

        let corr_lo = correlation(&input, &out_lo).abs();
        let corr_hi = correlation(&input, &out_hi).abs();
        assert!(
            corr_hi < corr_lo,
            "Higher gain → lower correlation: lo={corr_lo:.4}, hi={corr_hi:.4}"
        );
    }

    #[test]
    fn overdrive_gain_monotonic_thd() {
        // THD should increase monotonically across several gain points.
        let input = sine(440.0, 0.5, N);
        let gains = [0.0, 0.25, 0.5, 0.75, 1.0];
        let mut prev_thd = 0.0;
        for &g in &gains {
            let mut od = Overdrive::new(SR);
            od.set_gain(g);
            od.set_level(1.0);
            let output = process_buf(&mut od, &input);
            let thd = estimate_thd(&output, 440.0, SR);
            assert!(
                thd >= prev_thd - 0.01,
                "THD should increase with gain: gain={g}, thd={thd:.4}, prev={prev_thd:.4}"
            );
            prev_thd = thd;
        }
        // At max gain, THD should be substantial
        assert!(
            prev_thd > 0.3,
            "Max gain THD should be > 30%: got {prev_thd:.4}"
        );
    }

    #[test]
    fn overdrive_output_bounded() {
        // Even at max gain, output must not exceed ±2.0 (leave headroom).
        let input = sine(440.0, 1.0, N);
        let mut od = Overdrive::new(SR);
        od.set_gain(1.0);
        od.set_level(1.0);
        let output = process_buf(&mut od, &input);
        let p = peak(&output);
        assert!(p < 2.0, "output should be bounded: peak={p:.4}");
        assert!(p > 0.1, "output should not be silent: peak={p:.4}");
    }

    #[test]
    fn overdrive_level_controls_volume() {
        let input = sine(440.0, 0.5, N);

        let mut od_full = Overdrive::new(SR);
        od_full.set_gain(0.5);
        od_full.set_level(1.0);
        let out_full = process_buf(&mut od_full, &input);

        let mut od_half = Overdrive::new(SR);
        od_half.set_gain(0.5);
        od_half.set_level(0.5);
        let out_half = process_buf(&mut od_half, &input);

        let ratio = rms(&out_half) / rms(&out_full);
        assert!(
            (ratio - 0.5).abs() < 0.05,
            "level=0.5 should halve the output: ratio={ratio:.4}"
        );
    }

    #[test]
    fn overdrive_dc_stable() {
        // Constant zero input should produce zero output.
        let mut od = Overdrive::new(SR);
        od.set_gain(1.0);
        for _ in 0..4800 {
            let out = od.process(0.0);
            assert!(out.abs() < 1e-6, "DC stability: got {out}");
        }
    }

    #[test]
    fn overdrive_sample_rate_independence() {
        // The character should be similar at 44100 vs 48000.
        let input_48 = sine_at(440.0, 0.5, 48000, 48000.0);
        let input_44 = sine_at(440.0, 0.5, 44100, 44100.0);

        let mut od_48 = Overdrive::new(48000.0);
        od_48.set_gain(0.7);
        od_48.set_level(1.0);
        let out_48 = process_buf(&mut od_48, &input_48);

        let mut od_44 = Overdrive::new(44100.0);
        od_44.set_gain(0.7);
        od_44.set_level(1.0);
        let out_44 = process_buf(&mut od_44, &input_44);

        let thd_48 = estimate_thd(&out_48, 440.0, 48000.0);
        let thd_44 = estimate_thd(&out_44, 440.0, 44100.0);
        // THD should be within 30% of each other
        let ratio = thd_48 / thd_44;
        assert!(
            (0.7..=1.3).contains(&ratio),
            "THD should be similar across sample rates: 48k={thd_48:.4}, 44k={thd_44:.4}"
        );
    }

    #[test]
    fn overdrive_reset_clears_state() {
        let input = sine(440.0, 0.5, 4800);
        let mut od = Overdrive::new(SR);
        od.set_gain(0.7);

        let out1 = process_buf(&mut od, &input);
        od.reset();
        let out2 = process_buf(&mut od, &input);

        // After reset, processing the same input should give the same result.
        let diff_rms = rms(&out1.iter().zip(&out2).map(|(a, b)| a - b).collect::<Vec<_>>());
        assert!(
            diff_rms < 1e-10,
            "reset should restore identical behavior: diff_rms={diff_rms}"
        );
    }

    // ══════════════════════════════════════════════════════════════════
    //  FuzzFace tests
    // ══════════════════════════════════════════════════════════════════

    #[test]
    fn fuzzface_produces_output() {
        let input = sine(440.0, 0.3, 4800);
        let mut ff = FuzzFace::new(SR);
        let output = process_buf(&mut ff, &input);
        assert!(peak(&output) > 0.01, "fuzz should produce output");
    }

    #[test]
    fn fuzzface_high_fuzz_is_not_a_wire() {
        let input = sine(440.0, 0.5, N);
        let mut ff = FuzzFace::new(SR);
        ff.set_fuzz(0.9);
        ff.set_volume(1.0);
        let output = process_buf(&mut ff, &input);
        let corr = correlation(&input, &output).abs();
        // Pre-fix this was 0.995. Require < 0.98 to catch regression.
        assert!(
            corr < 0.98,
            "FuzzFace fuzz=0.9 should distort: |corr|={corr:.4}"
        );
    }

    #[test]
    fn fuzzface_more_fuzz_means_more_distortion() {
        let input = sine(440.0, 0.5, N);

        let mut ff_lo = FuzzFace::new(SR);
        ff_lo.set_fuzz(0.1);
        ff_lo.set_volume(1.0);
        let out_lo = process_buf(&mut ff_lo, &input);

        let mut ff_hi = FuzzFace::new(SR);
        ff_hi.set_fuzz(0.9);
        ff_hi.set_volume(1.0);
        let out_hi = process_buf(&mut ff_hi, &input);

        let corr_lo = correlation(&input, &out_lo).abs();
        let corr_hi = correlation(&input, &out_hi).abs();
        assert!(
            corr_hi < corr_lo,
            "Higher fuzz → lower correlation: lo={corr_lo:.4}, hi={corr_hi:.4}"
        );
    }

    #[test]
    fn fuzzface_asymmetric_clipping() {
        // Single-diode clipper should produce asymmetric output:
        // positive and negative peaks should differ in magnitude.
        let input = sine(440.0, 0.5, N);
        let mut ff = FuzzFace::new(SR);
        ff.set_fuzz(0.8);
        ff.set_volume(1.0);
        let output = process_buf(&mut ff, &input);
        let pos_peak = output.iter().copied().fold(0.0_f64, |m, x| m.max(x));
        let neg_peak = output.iter().copied().fold(0.0_f64, |m, x| m.min(x)).abs();
        // The ratio should differ from 1.0 (asymmetric).
        let ratio = pos_peak / neg_peak;
        assert!(
            !(0.95..=1.05).contains(&ratio),
            "Single-diode should clip asymmetrically: +peak={pos_peak:.4}, -peak={neg_peak:.4}, ratio={ratio:.4}"
        );
    }

    #[test]
    fn fuzzface_output_bounded() {
        let input = sine(440.0, 1.0, N);
        let mut ff = FuzzFace::new(SR);
        ff.set_fuzz(1.0);
        ff.set_volume(1.0);
        let output = process_buf(&mut ff, &input);
        let p = peak(&output);
        assert!(p < 3.0, "fuzz output should be bounded: peak={p:.4}");
        assert!(p > 0.1, "fuzz output should not be silent: peak={p:.4}");
    }

    #[test]
    fn fuzzface_volume_controls_output() {
        let input = sine(440.0, 0.5, N);

        let mut ff_full = FuzzFace::new(SR);
        ff_full.set_fuzz(0.5);
        ff_full.set_volume(1.0);
        let out_full = process_buf(&mut ff_full, &input);

        let mut ff_half = FuzzFace::new(SR);
        ff_half.set_fuzz(0.5);
        ff_half.set_volume(0.5);
        let out_half = process_buf(&mut ff_half, &input);

        let ratio = rms(&out_half) / rms(&out_full);
        assert!(
            (ratio - 0.5).abs() < 0.05,
            "volume=0.5 should halve: ratio={ratio:.4}"
        );
    }

    #[test]
    fn fuzzface_dc_stable() {
        let mut ff = FuzzFace::new(SR);
        ff.set_fuzz(1.0);
        for _ in 0..4800 {
            let out = ff.process(0.0);
            assert!(out.abs() < 1e-6, "DC stability: got {out}");
        }
    }

    #[test]
    fn fuzzface_thd_at_max_fuzz() {
        let input = sine(440.0, 0.5, N);
        let mut ff = FuzzFace::new(SR);
        ff.set_fuzz(1.0);
        ff.set_volume(1.0);
        let output = process_buf(&mut ff, &input);
        let thd = estimate_thd(&output, 440.0, SR);
        assert!(
            thd > 0.3,
            "Max fuzz THD should be > 30%: got {thd:.4}"
        );
    }

    // ══════════════════════════════════════════════════════════════════
    //  Overdrive vs FuzzFace — the two should sound different
    // ══════════════════════════════════════════════════════════════════

    #[test]
    fn overdrive_symmetric_fuzz_asymmetric() {
        // Overdrive uses a diode pair (symmetric clipping).
        // FuzzFace uses a single diode (asymmetric clipping).
        // The asymmetry ratio should differ.
        let input = sine(440.0, 0.5, N);

        let mut od = Overdrive::new(SR);
        od.set_gain(0.7);
        od.set_level(1.0);
        let out_od = process_buf(&mut od, &input);

        let mut ff = FuzzFace::new(SR);
        ff.set_fuzz(0.7);
        ff.set_volume(1.0);
        let out_ff = process_buf(&mut ff, &input);

        // Measure positive-to-negative peak ratio for each
        let od_pos = out_od.iter().copied().fold(0.0_f64, |m, x| m.max(x));
        let od_neg = out_od.iter().copied().fold(0.0_f64, |m, x| m.min(x)).abs();
        let od_ratio = od_pos / od_neg;

        let ff_pos = out_ff.iter().copied().fold(0.0_f64, |m, x| m.max(x));
        let ff_neg = out_ff.iter().copied().fold(0.0_f64, |m, x| m.min(x)).abs();
        let ff_ratio = ff_pos / ff_neg;

        // Overdrive should be roughly symmetric (ratio ≈ 1.0)
        assert!(
            (0.85..=1.15).contains(&od_ratio),
            "Overdrive should be symmetric: +/-ratio={od_ratio:.4}"
        );
        // FuzzFace should be noticeably asymmetric (ratio far from 1.0)
        assert!(
            !(0.90..=1.10).contains(&ff_ratio),
            "FuzzFace should be asymmetric: +/-ratio={ff_ratio:.4}"
        );
    }

    // ══════════════════════════════════════════════════════════════════
    //  Multi-frequency / dynamics tests
    // ══════════════════════════════════════════════════════════════════

    #[test]
    fn overdrive_clips_at_multiple_frequencies() {
        // Clipping should work for bass and treble, not just 440Hz.
        for &freq in &[82.0, 440.0, 2000.0] {
            let input = sine(freq, 0.5, N);
            let mut od = Overdrive::new(SR);
            od.set_gain(0.8);
            od.set_level(1.0);
            let output = process_buf(&mut od, &input);
            let thd = estimate_thd(&output, freq, SR);
            assert!(
                thd > 0.2,
                "Overdrive should clip at {freq}Hz: THD={thd:.4}"
            );
        }
    }

    #[test]
    fn overdrive_quiet_signal_less_clipped() {
        // Quiet signals should pass through with less distortion
        // than loud signals — the hallmark of soft clipping.
        let loud = sine(440.0, 0.5, N);
        let quiet = sine(440.0, 0.05, N);

        let mut od_loud = Overdrive::new(SR);
        od_loud.set_gain(0.7);
        od_loud.set_level(1.0);
        let out_loud = process_buf(&mut od_loud, &loud);

        let mut od_quiet = Overdrive::new(SR);
        od_quiet.set_gain(0.7);
        od_quiet.set_level(1.0);
        let out_quiet = process_buf(&mut od_quiet, &quiet);

        let thd_loud = estimate_thd(&out_loud, 440.0, SR);
        let thd_quiet = estimate_thd(&out_quiet, 440.0, SR);
        assert!(
            thd_loud > thd_quiet,
            "Louder signal should distort more: loud_thd={thd_loud:.4}, quiet_thd={thd_quiet:.4}"
        );
    }

    #[test]
    fn overdrive_sustain_compression() {
        // Clipping compresses dynamics: the ratio of peak/RMS should
        // decrease with more gain (output waveform is more "square").
        let input = sine(440.0, 0.5, N);

        let mut od_lo = Overdrive::new(SR);
        od_lo.set_gain(0.1);
        od_lo.set_level(1.0);
        let out_lo = process_buf(&mut od_lo, &input);
        let crest_lo = peak(&out_lo) / rms(&out_lo);

        let mut od_hi = Overdrive::new(SR);
        od_hi.set_gain(1.0);
        od_hi.set_level(1.0);
        let out_hi = process_buf(&mut od_hi, &input);
        let crest_hi = peak(&out_hi) / rms(&out_hi);

        // A sine has crest factor √2 ≈ 1.414.
        // Heavy clipping → squarewave → crest ≈ 1.0.
        assert!(
            crest_hi < crest_lo,
            "Heavy clipping should reduce crest factor: lo={crest_lo:.4}, hi={crest_hi:.4}"
        );
    }

    // ══════════════════════════════════════════════════════════════════
    //  Delay tests
    // ══════════════════════════════════════════════════════════════════

    #[test]
    fn delay_echoes() {
        let mut d = Delay::new(SR);
        d.set_delay_time(100.0);
        d.set_mix(1.0);
        d.set_feedback(0.0);
        let _ = d.process(1.0);
        let delay_samples = (100.0 / 1000.0 * SR) as usize;
        for _ in 1..delay_samples {
            let _ = d.process(0.0);
        }
        let echo = d.process(0.0);
        assert!(echo.abs() > 0.5, "should hear the echo");
    }

    #[test]
    fn delay_feedback_repeats() {
        let mut d = Delay::new(SR);
        d.set_delay_time(100.0);
        d.set_mix(1.0);
        d.set_feedback(0.5);
        // Send impulse, then collect all output
        let delay_samples = (100.0 / 1000.0 * SR) as usize;
        let total = delay_samples * 3;
        let mut output = Vec::with_capacity(total);
        output.push(d.process(1.0));
        for _ in 1..total {
            output.push(d.process(0.0));
        }
        // Find peaks near expected echo positions
        let echo1 = peak(&output[delay_samples - 2..delay_samples + 2]);
        let echo2 = peak(&output[2 * delay_samples - 2..2 * delay_samples + 2]);
        assert!(echo1 > 0.3, "first echo should be audible: {echo1}");
        assert!(echo2 > 0.05, "second echo should exist: {echo2}");
        assert!(echo2 < echo1, "echoes should decay");
    }

    // ══════════════════════════════════════════════════════════════════
    //  Edge cases / robustness
    // ══════════════════════════════════════════════════════════════════

    #[test]
    fn overdrive_handles_silence() {
        let silence = vec![0.0; 4800];
        let mut od = Overdrive::new(SR);
        od.set_gain(1.0);
        let output = process_buf(&mut od, &silence);
        assert!(peak(&output) < 1e-6, "silence in → silence out");
    }

    #[test]
    fn overdrive_handles_impulse() {
        let mut impulse = vec![0.0; 4800];
        impulse[0] = 1.0;
        let mut od = Overdrive::new(SR);
        od.set_gain(0.5);
        od.set_level(1.0);
        let output = process_buf(&mut od, &impulse);
        assert!(peak(&output) > 0.01, "impulse should produce output");
        assert!(output.iter().all(|x| x.is_finite()), "no NaN/inf");
    }

    #[test]
    fn fuzzface_handles_impulse() {
        let mut impulse = vec![0.0; 4800];
        impulse[0] = 1.0;
        let mut ff = FuzzFace::new(SR);
        ff.set_fuzz(0.9);
        ff.set_volume(1.0);
        let output = process_buf(&mut ff, &impulse);
        assert!(peak(&output) > 0.01, "impulse should produce output");
        assert!(output.iter().all(|x| x.is_finite()), "no NaN/inf");
    }

    #[test]
    fn overdrive_no_nan_at_extreme_gain() {
        // Full gain + hot signal — stress test for numerical stability.
        let input = sine(440.0, 1.0, N);
        let mut od = Overdrive::new(SR);
        od.set_gain(1.0);
        od.set_level(1.0);
        let output = process_buf(&mut od, &input);
        assert!(
            output.iter().all(|x| x.is_finite()),
            "no NaN/inf at extreme settings"
        );
    }

    #[test]
    fn fuzzface_no_nan_at_extreme_fuzz() {
        let input = sine(440.0, 1.0, N);
        let mut ff = FuzzFace::new(SR);
        ff.set_fuzz(1.0);
        ff.set_volume(1.0);
        let output = process_buf(&mut ff, &input);
        assert!(
            output.iter().all(|x| x.is_finite()),
            "no NaN/inf at extreme settings"
        );
    }

    #[test]
    fn overdrive_knob_sweep_stable() {
        // Changing gain while processing should not produce glitches.
        let input = sine(440.0, 0.5, N);
        let mut od = Overdrive::new(SR);
        od.set_level(1.0);
        for (i, &s) in input.iter().enumerate() {
            // Sweep gain from 0 to 1 over 1 second
            let g = i as f64 / N as f64;
            od.set_gain(g);
            let out = od.process(s);
            assert!(out.is_finite(), "NaN during knob sweep at sample {i}");
            assert!(out.abs() < 5.0, "output spike at sample {i}: {out}");
        }
    }
}
