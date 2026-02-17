//! Pedal implementations backed by the WDF engine.
//!
//! Each pedal maps knob positions to WDF tree parameters and exposes
//! the `PedalProcessor` trait for integration with the audio engine.

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
        let peak = |s: &[f64]| s.iter().fold(0.0_f64, |m, x| m.max(x.abs()));
        let echo1 = peak(&output[delay_samples - 2..delay_samples + 2]);
        let echo2 = peak(&output[2 * delay_samples - 2..2 * delay_samples + 2]);
        assert!(echo1 > 0.3, "first echo should be audible: {echo1}");
        assert!(echo2 > 0.05, "second echo should exist: {echo2}");
        assert!(echo2 < echo1, "echoes should decay");
    }
}
