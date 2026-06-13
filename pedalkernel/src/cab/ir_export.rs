//! Impulse response capture and WAV export.
//!
//! Generic module — works with any [`PedalProcessor`]. Captures the IR by
//! feeding a unit impulse and recording the output, then writes the result
//! as a WAV file.

use std::path::Path;

use crate::PedalProcessor;

/// Configuration for IR export.
#[derive(Debug, Clone)]
pub struct IrExportConfig {
    /// Sample rate in Hz.
    pub sample_rate: u32,
    /// IR length in samples.
    pub length: usize,
    /// Bit depth for WAV output (16, 24, or 32).
    pub bit_depth: u16,
    /// Normalize the IR to peak = 1.0.
    pub normalize: bool,
    /// Fade-out length in samples (cosine taper at end). 0 = no fade.
    pub fade_out: usize,
}

impl Default for IrExportConfig {
    fn default() -> Self {
        Self {
            sample_rate: 48_000,
            length: 48_000, // 1 second
            bit_depth: 32,
            normalize: true,
            fade_out: 2048,
        }
    }
}

/// Capture an impulse response from any PedalProcessor.
///
/// Feeds a unit impulse (1.0 at sample 0, then silence) and records the output.
/// Optionally normalizes and applies a fade-out taper.
pub fn capture_ir(proc: &mut dyn PedalProcessor, config: &IrExportConfig) -> Vec<f64> {
    proc.set_sample_rate(config.sample_rate as f64);
    proc.reset();

    let mut ir = Vec::with_capacity(config.length);

    // Feed impulse
    ir.push(proc.process(1.0));

    // Feed silence for the rest
    for _ in 1..config.length {
        ir.push(proc.process(0.0));
    }

    // Fade-out (cosine taper)
    if config.fade_out > 0 && config.fade_out < config.length {
        let fade_start = config.length - config.fade_out;
        for i in fade_start..config.length {
            let t = (i - fade_start) as f64 / config.fade_out as f64;
            let gain = 0.5 * (1.0 + (std::f64::consts::PI * t).cos());
            ir[i] *= gain;
        }
    }

    // Normalize
    if config.normalize {
        let peak = ir.iter().map(|s| s.abs()).fold(0.0f64, f64::max);
        if peak > 1e-12 {
            let scale = 1.0 / peak;
            for s in &mut ir {
                *s *= scale;
            }
        }
    }

    ir
}

/// Export a mono IR to a WAV file.
pub fn export_ir_wav(
    ir: &[f64],
    path: &Path,
    config: &IrExportConfig,
) -> Result<(), Box<dyn std::error::Error>> {
    use hound::{SampleFormat, WavSpec, WavWriter};

    let spec = WavSpec {
        channels: 1,
        sample_rate: config.sample_rate,
        bits_per_sample: config.bit_depth,
        sample_format: if config.bit_depth == 32 {
            SampleFormat::Float
        } else {
            SampleFormat::Int
        },
    };

    let mut writer = WavWriter::create(path, spec)?;

    match config.bit_depth {
        32 => {
            for &s in ir {
                writer.write_sample(s as f32)?;
            }
        }
        24 => {
            let max = (1_i32 << 23) - 1;
            for &s in ir {
                let sample = (s * max as f64).round().clamp(-(max as f64), max as f64) as i32;
                writer.write_sample(sample)?;
            }
        }
        16 => {
            for &s in ir {
                let sample = (s * 32767.0).round().clamp(-32768.0, 32767.0) as i16;
                writer.write_sample(sample)?;
            }
        }
        _ => return Err(format!("Unsupported bit depth: {}", config.bit_depth).into()),
    }

    writer.finalize()?;
    Ok(())
}

/// Capture and export a stereo IR from two processors (e.g., two mic positions).
pub fn export_stereo_ir(
    proc_l: &mut dyn PedalProcessor,
    proc_r: &mut dyn PedalProcessor,
    path: &Path,
    config: &IrExportConfig,
) -> Result<(), Box<dyn std::error::Error>> {
    let ir_l = capture_ir(proc_l, config);
    let ir_r = capture_ir(proc_r, config);

    use hound::{SampleFormat, WavSpec, WavWriter};

    let spec = WavSpec {
        channels: 2,
        sample_rate: config.sample_rate,
        bits_per_sample: 32,
        sample_format: SampleFormat::Float,
    };

    let mut writer = WavWriter::create(path, spec)?;
    let n = ir_l.len().min(ir_r.len());
    for i in 0..n {
        writer.write_sample(ir_l[i] as f32)?;
        writer.write_sample(ir_r[i] as f32)?;
    }
    writer.finalize()?;
    Ok(())
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    /// Simple pass-through processor for testing IR capture.
    struct PassThrough;

    impl PedalProcessor for PassThrough {
        fn process(&mut self, input: f64) -> f64 {
            input
        }
        fn set_sample_rate(&mut self, _rate: f64) {}
        fn reset(&mut self) {}
    }

    #[test]
    fn capture_ir_passthrough() {
        let mut proc = PassThrough;
        let config = IrExportConfig {
            length: 100,
            normalize: false,
            fade_out: 0,
            ..Default::default()
        };
        let ir = capture_ir(&mut proc, &config);
        assert_eq!(ir.len(), 100);
        assert!((ir[0] - 1.0).abs() < 1e-10, "First sample should be 1.0");
        assert!(ir[1].abs() < 1e-10, "Rest should be 0.0");
    }

    #[test]
    fn capture_ir_normalized() {
        let mut proc = PassThrough;
        let config = IrExportConfig {
            length: 100,
            normalize: true,
            fade_out: 0,
            ..Default::default()
        };
        let ir = capture_ir(&mut proc, &config);
        let peak = ir.iter().map(|s| s.abs()).fold(0.0f64, f64::max);
        assert!((peak - 1.0).abs() < 1e-10, "Peak should be 1.0 after normalization");
    }

    #[test]
    fn capture_ir_fadeout() {
        let mut proc = PassThrough;
        let config = IrExportConfig {
            length: 100,
            normalize: false,
            fade_out: 10,
            ..Default::default()
        };
        let ir = capture_ir(&mut proc, &config);
        // Last sample should be faded to near zero
        assert!(ir[99].abs() < 0.01, "Last sample should be near zero after fade");
    }

    #[test]
    fn export_wav_roundtrip() {
        let mut proc = PassThrough;
        let config = IrExportConfig {
            length: 100,
            normalize: false,
            fade_out: 0,
            ..Default::default()
        };
        let ir = capture_ir(&mut proc, &config);

        let tmp = std::env::temp_dir().join("pedalkernel_ir_test.wav");
        export_ir_wav(&ir, &tmp, &config).unwrap();

        // Verify file exists and has correct sample count
        let reader = hound::WavReader::open(&tmp).unwrap();
        assert_eq!(reader.spec().sample_rate, 48_000);
        assert_eq!(reader.len(), 100);
        let _ = std::fs::remove_file(&tmp);
    }
}
