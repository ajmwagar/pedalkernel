//! Pedal implementations using WDF

use crate::{PedalProcessor, elements::*, tree::WdfTree};

/// Tube Screamer-style overdrive
pub struct Overdrive {
    gain: f64,
    tone: f64,
    level: f64,
    sample_rate: f64,
}

impl Overdrive {
    pub fn new() -> Self {
        Self {
            gain: 0.5,
            tone: 0.5,
            level: 0.8,
            sample_rate: 48000.0,
        }
    }
    
    pub fn set_gain(&mut self, gain: f64) {
        self.gain = gain.clamp(0.0, 1.0);
    }
    
    pub fn set_tone(&mut self, tone: f64) {
        self.tone = tone.clamp(0.0, 1.0);
    }
    
    pub fn set_level(&mut self, level: f64) {
        self.level = level.clamp(0.0, 1.0);
    }
}

impl PedalProcessor for Overdrive {
    fn process(&mut self, input: f64) -> f64 {
        // Simplified overdrive - clip the signal
        let boosted = input * (1.0 + self.gain * 10.0);
        let clipped = boosted.tanh();
        clipped * self.level
    }
    
    fn set_sample_rate(&mut self, rate: f64) {
        self.sample_rate = rate;
    }
    
    fn reset(&mut self) {
        // No state to reset in this simplified version
    }
}

/// Fuzz Face style fuzz pedal
pub struct FuzzFace {
    fuzz: f64,
    volume: f64,
    sample_rate: f64,
}

impl FuzzFace {
    pub fn new() -> Self {
        Self {
            fuzz: 0.5,
            volume: 0.8,
            sample_rate: 48000.0,
        }
    }
    
    pub fn set_fuzz(&mut self, fuzz: f64) {
        self.fuzz = fuzz.clamp(0.0, 1.0);
    }
    
    pub fn set_volume(&mut self, volume: f64) {
        self.volume = volume.clamp(0.0, 1.0);
    }
}

impl PedalProcessor for FuzzFace {
    fn process(&mut self, input: f64) -> f64 {
        // Harsh fuzz clipping
        let boosted = input * (1.0 + self.fuzz * 50.0);
        let clipped = if boosted > 0.0 {
            boosted.powf(0.3) // Asymmetric clipping
        } else {
            boosted
        };
        clipped.tanh() * self.volume
    }
    
    fn set_sample_rate(&mut self, rate: f64) {
        self.sample_rate = rate;
    }
    
    fn reset(&mut self) {
        // No state
    }
}

/// Simple delay pedal
pub struct Delay {
    delay_time_ms: f64,
    feedback: f64,
    mix: f64,
    buffer: Vec<f64>,
    write_pos: usize,
    sample_rate: f64,
}

impl Delay {
    pub fn new() -> Self {
        let sample_rate = 48000.0;
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

impl PedalProcessor for Delay {
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
