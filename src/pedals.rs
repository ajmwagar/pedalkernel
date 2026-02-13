//! Pedal implementations backed by the WDF engine.
//!
//! Each pedal maps knob positions to WDF tree parameters and exposes
//! the `PedalProcessor` trait for integration with the audio engine.

use crate::elements::*;
use crate::tree::*;

/// Tube Screamer-style overdrive using a WDF diode clipper.
///
/// Topology: input → series(Vs, parallel(R_gain, C)) → diode_pair_root
///
/// The "Drive" knob scales the resistor in the feedback network,
/// controlling how much the diode pair clips.  "Level" is a simple
/// output gain.
pub struct Overdrive {
    clipper: WdfClipper,
    gain: f64,
    level: f64,
    base_resistance: f64,
    sample_rate: f64,
}

impl Overdrive {
    pub fn new(sample_rate: f64) -> Self {
        let base_r = 4700.0; // 4.7 kΩ
        Self {
            clipper: WdfClipper::new(
                base_r,
                47e-9,  // 47 nF
                DiodeModel::silicon(),
                sample_rate,
            ),
            gain: 0.5,
            level: 0.8,
            base_resistance: base_r,
            sample_rate,
        }
    }

    pub fn set_gain(&mut self, gain: f64) {
        self.gain = gain.clamp(0.0, 1.0);
        // Map gain [0,1] -> resistance [51k, 551] (higher gain = lower R = more clipping)
        let r = self.base_resistance * (1.0 - 0.9 * self.gain) + 51.0;
        self.clipper = WdfClipper::new(r, 47e-9, DiodeModel::silicon(), self.sample_rate);
    }

    pub fn set_level(&mut self, level: f64) {
        self.level = level.clamp(0.0, 1.0);
    }
}

impl crate::PedalProcessor for Overdrive {
    fn process(&mut self, input: f64) -> f64 {
        self.clipper.process(input) * self.level
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
/// Uses a germanium diode for asymmetric, splatty clipping.
pub struct FuzzFace {
    clipper: WdfSingleDiodeClipper,
    fuzz: f64,
    volume: f64,
    sample_rate: f64,
}

impl FuzzFace {
    pub fn new(sample_rate: f64) -> Self {
        Self {
            clipper: WdfSingleDiodeClipper::new(
                1000.0,
                100e-9,
                DiodeModel::germanium(),
                sample_rate,
            ),
            fuzz: 0.5,
            volume: 0.8,
            sample_rate,
        }
    }

    pub fn set_fuzz(&mut self, fuzz: f64) {
        self.fuzz = fuzz.clamp(0.0, 1.0);
        let r = 10000.0 * (1.0 - 0.95 * self.fuzz) + 100.0;
        self.clipper = WdfSingleDiodeClipper::new(r, 100e-9, DiodeModel::germanium(), self.sample_rate);
    }

    pub fn set_volume(&mut self, volume: f64) {
        self.volume = volume.clamp(0.0, 1.0);
    }
}

impl crate::PedalProcessor for FuzzFace {
    fn process(&mut self, input: f64) -> f64 {
        self.clipper.process(input) * self.volume
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

    #[test]
    fn overdrive_produces_output() {
        let mut od = Overdrive::new(48000.0);
        let mut max_out = 0.0_f64;
        for i in 0..4800 {
            let t = i as f64 / 48000.0;
            let input = 0.5 * (2.0 * std::f64::consts::PI * 440.0 * t).sin();
            let out = od.process(input);
            max_out = max_out.max(out.abs());
        }
        assert!(max_out > 0.001, "overdrive should produce output");
    }

    #[test]
    fn overdrive_gain_increases_distortion() {
        let mut od_low = Overdrive::new(48000.0);
        od_low.set_gain(0.1);
        let mut od_high = Overdrive::new(48000.0);
        od_high.set_gain(0.9);

        // Feed identical sine, measure RMS difference from clean
        let n = 4800;
        let mut rms_low = 0.0;
        let mut rms_high = 0.0;
        for i in 0..n {
            let t = i as f64 / 48000.0;
            let input = 0.5 * (2.0 * std::f64::consts::PI * 440.0 * t).sin();
            let out_low = od_low.process(input);
            let out_high = od_high.process(input);
            rms_low += (out_low - input).powi(2);
            rms_high += (out_high - input).powi(2);
        }
        rms_low = (rms_low / n as f64).sqrt();
        rms_high = (rms_high / n as f64).sqrt();
        assert!(rms_high > rms_low, "higher gain should produce more distortion");
    }

    #[test]
    fn fuzzface_produces_output() {
        let mut ff = FuzzFace::new(48000.0);
        let mut max_out = 0.0_f64;
        for i in 0..4800 {
            let t = i as f64 / 48000.0;
            let input = 0.3 * (2.0 * std::f64::consts::PI * 440.0 * t).sin();
            let out = ff.process(input);
            max_out = max_out.max(out.abs());
        }
        assert!(max_out > 0.001, "fuzz should produce output");
    }

    #[test]
    fn delay_echoes() {
        let mut d = Delay::new(48000.0);
        d.set_delay_time(100.0);
        d.set_mix(1.0);
        d.set_feedback(0.0);

        // Send impulse
        let _ = d.process(1.0);
        // Wait until delay time
        let delay_samples = (100.0 / 1000.0 * 48000.0) as usize;
        for _ in 1..delay_samples {
            let _ = d.process(0.0);
        }
        let echo = d.process(0.0);
        assert!(echo.abs() > 0.5, "should hear the echo at delay_samples");
    }
}
