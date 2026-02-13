//! WAV file I/O for testing and offline rendering.
//!
//! Uses `hound` to write processed audio to WAV files so circuits
//! can be auditioned without a running JACK server.

use hound::{SampleFormat, WavSpec, WavWriter};
use std::path::Path;

use crate::PedalProcessor;

/// WAV output spec: 48 kHz, 32-bit float, mono.
pub const DEFAULT_SAMPLE_RATE: u32 = 48_000;

fn wav_spec(sample_rate: u32) -> WavSpec {
    WavSpec {
        channels: 1,
        sample_rate,
        bits_per_sample: 32,
        sample_format: SampleFormat::Float,
    }
}

/// Generate a sine wave test signal.
pub fn sine_wave(freq_hz: f64, duration_secs: f64, sample_rate: u32) -> Vec<f64> {
    let n = (duration_secs * sample_rate as f64) as usize;
    let mut buf = Vec::with_capacity(n);
    for i in 0..n {
        let t = i as f64 / sample_rate as f64;
        buf.push(0.5 * (2.0 * std::f64::consts::PI * freq_hz * t).sin());
    }
    buf
}

/// Generate a guitar-like test signal (sum of harmonics with decay).
pub fn guitar_pluck(freq_hz: f64, duration_secs: f64, sample_rate: u32) -> Vec<f64> {
    let n = (duration_secs * sample_rate as f64) as usize;
    let mut buf = Vec::with_capacity(n);
    for i in 0..n {
        let t = i as f64 / sample_rate as f64;
        let envelope = (-3.0 * t).exp(); // exponential decay
        let fundamental = (2.0 * std::f64::consts::PI * freq_hz * t).sin();
        let h2 = 0.5 * (2.0 * std::f64::consts::PI * 2.0 * freq_hz * t).sin();
        let h3 = 0.25 * (2.0 * std::f64::consts::PI * 3.0 * freq_hz * t).sin();
        let h4 = 0.125 * (2.0 * std::f64::consts::PI * 4.0 * freq_hz * t).sin();
        buf.push(0.4 * envelope * (fundamental + h2 + h3 + h4));
    }
    buf
}

/// Process a buffer of samples through a pedal and write the result to a WAV file.
pub fn render_to_wav<P: PedalProcessor>(
    pedal: &mut P,
    input: &[f64],
    path: &Path,
    sample_rate: u32,
) -> Result<(), Box<dyn std::error::Error>> {
    pedal.set_sample_rate(sample_rate as f64);
    pedal.reset();

    let mut writer = WavWriter::create(path, wav_spec(sample_rate))?;
    for &sample in input {
        let out = pedal.process(sample);
        writer.write_sample(out as f32)?;
    }
    writer.finalize()?;
    Ok(())
}

/// Write raw f64 samples to a WAV file.
pub fn write_wav(
    samples: &[f64],
    path: &Path,
    sample_rate: u32,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut writer = WavWriter::create(path, wav_spec(sample_rate))?;
    for &s in samples {
        writer.write_sample(s as f32)?;
    }
    writer.finalize()?;
    Ok(())
}

/// Write both input and output (stereo) to a WAV for A/B comparison.
pub fn write_stereo_wav(
    left: &[f64],
    right: &[f64],
    path: &Path,
    sample_rate: u32,
) -> Result<(), Box<dyn std::error::Error>> {
    let spec = WavSpec {
        channels: 2,
        sample_rate,
        bits_per_sample: 32,
        sample_format: SampleFormat::Float,
    };
    let mut writer = WavWriter::create(path, spec)?;
    let n = left.len().min(right.len());
    for i in 0..n {
        writer.write_sample(left[i] as f32)?;
        writer.write_sample(right[i] as f32)?;
    }
    writer.finalize()?;
    Ok(())
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sine_wave_length() {
        let buf = sine_wave(440.0, 1.0, 48000);
        assert_eq!(buf.len(), 48000);
    }

    #[test]
    fn sine_wave_amplitude() {
        let buf = sine_wave(440.0, 1.0, 48000);
        let max = buf.iter().copied().fold(0.0_f64, |a, b| a.max(b.abs()));
        assert!((max - 0.5).abs() < 0.01, "expected peak ≈ 0.5, got {max}");
    }

    #[test]
    fn guitar_pluck_decays() {
        let buf = guitar_pluck(82.41, 2.0, 48000);
        // First 1000 samples should be louder than last 1000
        let rms_start: f64 = buf[..1000].iter().map(|x| x * x).sum::<f64>() / 1000.0;
        let rms_end: f64 = buf[buf.len() - 1000..].iter().map(|x| x * x).sum::<f64>() / 1000.0;
        assert!(rms_start > rms_end * 10.0, "signal should decay");
    }

    #[test]
    fn render_wav_roundtrip() {
        use crate::pedals::Overdrive;
        let tmp = std::env::temp_dir().join("pedalkernel_test_render.wav");
        let input = sine_wave(440.0, 0.1, 48000);
        let mut od = Overdrive::new(48000.0);
        render_to_wav(&mut od, &input, &tmp, 48000).unwrap();

        // Verify the file was created and has correct length
        let reader = hound::WavReader::open(&tmp).unwrap();
        assert_eq!(reader.spec().sample_rate, 48000);
        assert_eq!(reader.len(), input.len() as u32);
        let _ = std::fs::remove_file(&tmp);
    }
}
