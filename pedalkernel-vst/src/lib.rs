//! PedalKernel VST3/CLAP plugin.
//!
//! Wraps the PedalKernel WDF engine as a standard plugin for use in any DAW.
//! Provides three built-in pedal models — Overdrive (Tube Screamer), Fuzz Face,
//! and Delay — selectable at runtime.

use nih_plug::prelude::*;
use pedalkernel::pedals::{Delay, FuzzFace, Overdrive};
use pedalkernel::PedalProcessor;
use std::sync::Arc;

// ═══════════════════════════════════════════════════════════════════════════
// Pedal type selector
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Enum, Debug, PartialEq, Eq, Clone, Copy)]
enum PedalType {
    Overdrive,
    #[name = "Fuzz Face"]
    FuzzFace,
    Delay,
}

// ═══════════════════════════════════════════════════════════════════════════
// Parameters
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Params)]
struct PedalKernelParams {
    /// Which pedal model to use.
    #[id = "type"]
    pedal_type: EnumParam<PedalType>,

    // ── Overdrive ──────────────────────────────────────────────────────
    /// Drive amount (Overdrive mode). Controls op-amp gain and clipping.
    #[id = "drive"]
    drive: FloatParam,

    /// Output level (Overdrive mode).
    #[id = "level"]
    level: FloatParam,

    // ── Fuzz Face ──────────────────────────────────────────────────────
    /// Fuzz amount (Fuzz Face mode). Controls transistor gain.
    #[id = "fuzz"]
    fuzz: FloatParam,

    /// Output volume (Fuzz Face mode).
    #[id = "vol"]
    volume: FloatParam,

    // ── Delay ──────────────────────────────────────────────────────────
    /// Delay time in milliseconds.
    #[id = "time"]
    delay_time: FloatParam,

    /// Feedback amount (echo repeats).
    #[id = "fback"]
    feedback: FloatParam,

    /// Wet/dry mix.
    #[id = "mix"]
    mix: FloatParam,
}

impl Default for PedalKernelParams {
    fn default() -> Self {
        Self {
            pedal_type: EnumParam::new("Pedal Type", PedalType::Overdrive),

            drive: FloatParam::new("Drive", 0.5, FloatRange::Linear { min: 0.0, max: 1.0 })
                .with_unit("")
                .with_smoother(SmoothingStyle::Linear(20.0))
                .with_value_to_string(formatters::v2s_f32_rounded(2)),

            level: FloatParam::new("Level", 0.8, FloatRange::Linear { min: 0.0, max: 1.0 })
                .with_unit("")
                .with_smoother(SmoothingStyle::Linear(20.0))
                .with_value_to_string(formatters::v2s_f32_rounded(2)),

            fuzz: FloatParam::new("Fuzz", 0.5, FloatRange::Linear { min: 0.0, max: 1.0 })
                .with_unit("")
                .with_smoother(SmoothingStyle::Linear(20.0))
                .with_value_to_string(formatters::v2s_f32_rounded(2)),

            volume: FloatParam::new("Volume", 0.8, FloatRange::Linear { min: 0.0, max: 1.0 })
                .with_unit("")
                .with_smoother(SmoothingStyle::Linear(20.0))
                .with_value_to_string(formatters::v2s_f32_rounded(2)),

            delay_time: FloatParam::new(
                "Delay Time",
                300.0,
                FloatRange::Skewed {
                    min: 10.0,
                    max: 2000.0,
                    factor: FloatRange::skew_factor(-1.5),
                },
            )
            .with_unit(" ms")
            .with_smoother(SmoothingStyle::Linear(50.0))
            .with_value_to_string(formatters::v2s_f32_rounded(1)),

            feedback: FloatParam::new(
                "Feedback",
                0.4,
                FloatRange::Linear { min: 0.0, max: 0.95 },
            )
            .with_unit("")
            .with_smoother(SmoothingStyle::Linear(20.0))
            .with_value_to_string(formatters::v2s_f32_rounded(2)),

            mix: FloatParam::new("Mix", 0.5, FloatRange::Linear { min: 0.0, max: 1.0 })
                .with_unit("")
                .with_smoother(SmoothingStyle::Linear(20.0))
                .with_value_to_string(formatters::v2s_f32_rounded(2)),
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Plugin struct
// ═══════════════════════════════════════════════════════════════════════════

struct PedalKernelPlugin {
    params: Arc<PedalKernelParams>,
    overdrive: Overdrive,
    fuzzface: FuzzFace,
    delay: Delay,
}

impl Default for PedalKernelPlugin {
    fn default() -> Self {
        let sample_rate = 48000.0;
        Self {
            params: Arc::new(PedalKernelParams::default()),
            overdrive: Overdrive::new(sample_rate),
            fuzzface: FuzzFace::new(sample_rate),
            delay: Delay::new(sample_rate),
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Plugin trait implementation
// ═══════════════════════════════════════════════════════════════════════════

impl Plugin for PedalKernelPlugin {
    const NAME: &'static str = "PedalKernel";
    const VENDOR: &'static str = "Avery Wagar";
    const URL: &'static str = "https://github.com/ajmwagar/pedalkernel";
    const EMAIL: &'static str = "ajmwagar@gmail.com";
    const VERSION: &'static str = env!("CARGO_PKG_VERSION");

    const AUDIO_IO_LAYOUTS: &'static [AudioIOLayout] = &[
        // Mono (guitar input → processed output)
        AudioIOLayout {
            main_input_channels: NonZeroU32::new(1),
            main_output_channels: NonZeroU32::new(1),
            ..AudioIOLayout::const_default()
        },
        // Stereo (process both channels identically)
        AudioIOLayout {
            main_input_channels: NonZeroU32::new(2),
            main_output_channels: NonZeroU32::new(2),
            ..AudioIOLayout::const_default()
        },
    ];

    type SysExMessage = ();
    type BackgroundTask = ();

    fn params(&self) -> Arc<dyn Params> {
        self.params.clone()
    }

    fn initialize(
        &mut self,
        _audio_io_layout: &AudioIOLayout,
        buffer_config: &BufferConfig,
        _context: &mut impl InitContext<Self>,
    ) -> bool {
        let sr = buffer_config.sample_rate as f64;
        self.overdrive.set_sample_rate(sr);
        self.fuzzface.set_sample_rate(sr);
        self.delay.set_sample_rate(sr);
        true
    }

    fn reset(&mut self) {
        self.overdrive.reset();
        self.fuzzface.reset();
        self.delay.reset();
    }

    fn process(
        &mut self,
        buffer: &mut Buffer,
        _aux: &mut AuxiliaryBuffers,
        _context: &mut impl ProcessContext<Self>,
    ) -> ProcessStatus {
        let pedal_type = self.params.pedal_type.value();

        for channel_samples in buffer.iter_samples() {
            // Read smoothed parameter values (per-sample for zipper-free automation)
            let drive = self.params.drive.smoothed.next() as f64;
            let level = self.params.level.smoothed.next() as f64;
            let fuzz = self.params.fuzz.smoothed.next() as f64;
            let vol = self.params.volume.smoothed.next() as f64;
            let delay_time = self.params.delay_time.smoothed.next() as f64;
            let feedback = self.params.feedback.smoothed.next() as f64;
            let mix = self.params.mix.smoothed.next() as f64;

            // Apply params to the active pedal
            match pedal_type {
                PedalType::Overdrive => {
                    self.overdrive.set_gain(drive);
                    self.overdrive.set_level(level);
                }
                PedalType::FuzzFace => {
                    self.fuzzface.set_fuzz(fuzz);
                    self.fuzzface.set_volume(vol);
                }
                PedalType::Delay => {
                    self.delay.set_delay_time(delay_time);
                    self.delay.set_feedback(feedback);
                    self.delay.set_mix(mix);
                }
            }

            // Process each channel through the same pedal instance.
            // For mono this is one channel; for stereo, both get the same
            // processing (matching how a real guitar pedal works).
            for sample in channel_samples {
                let input = *sample as f64;
                let output = match pedal_type {
                    PedalType::Overdrive => self.overdrive.process(input),
                    PedalType::FuzzFace => self.fuzzface.process(input),
                    PedalType::Delay => self.delay.process(input),
                };
                *sample = output as f32;
            }
        }

        ProcessStatus::Normal
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// VST3 metadata
// ═══════════════════════════════════════════════════════════════════════════

impl Vst3Plugin for PedalKernelPlugin {
    const VST3_CLASS_ID: [u8; 16] = *b"PdlKrnlWdfV0_01\0";
    const VST3_SUBCATEGORIES: &'static [Vst3SubCategory] = &[
        Vst3SubCategory::Fx,
        Vst3SubCategory::Distortion,
    ];
}

impl ClapPlugin for PedalKernelPlugin {
    const CLAP_ID: &'static str = "com.ajmwagar.pedalkernel";
    const CLAP_DESCRIPTION: Option<&'static str> =
        Some("WDF-based guitar pedal emulations");
    const CLAP_MANUAL_URL: Option<&'static str> = None;
    const CLAP_SUPPORT_URL: Option<&'static str> = None;
    const CLAP_FEATURES: &'static [ClapFeature] = &[
        ClapFeature::AudioEffect,
        ClapFeature::Distortion,
        ClapFeature::Mono,
        ClapFeature::Stereo,
    ];
}

nih_export_clap!(PedalKernelPlugin);
nih_export_vst3!(PedalKernelPlugin);
