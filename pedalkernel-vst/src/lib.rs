//! PedalKernel VST3/CLAP plugin.
//!
//! Single-plugin architecture with a pedal selector dropdown and three generic
//! knob parameters.  Each pedal type remaps the knobs to its own controls
//! (Drive, Tone, Sustain, etc.) via the compiled `.pedal` DSL pipeline.
//!
//! Seven WDF-based pedal models are compiled from embedded `.pedal` source
//! files at plugin init; one digital delay is provided as a built-in.

use nih_plug::prelude::*;
use pedalkernel::compiler::{compile_pedal, CompiledPedal};
use pedalkernel::dsl;
use pedalkernel::pedals::Delay;
use pedalkernel::PedalProcessor;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Arc;

// ═══════════════════════════════════════════════════════════════════════════
// Embedded .pedal sources (compiled at init)
// ═══════════════════════════════════════════════════════════════════════════

const PEDAL_SOURCES: &[&str] = &[
    include_str!("../../pedalkernel/examples/tube_screamer.pedal"),
    include_str!("../../pedalkernel/examples/fuzz_face.pedal"),
    include_str!("../../pedalkernel/examples/big_muff.pedal"),
    include_str!("../../pedalkernel/examples/dyna_comp.pedal"),
    include_str!("../../pedalkernel/examples/proco_rat.pedal"),
    include_str!("../../pedalkernel/examples/blues_driver.pedal"),
    include_str!("../../pedalkernel/examples/klon_centaur.pedal"),
];

// ═══════════════════════════════════════════════════════════════════════════
// Pedal type selector
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Enum, Debug, PartialEq, Eq, Clone, Copy)]
enum PedalType {
    #[name = "Tube Screamer"]
    TubeScreamer,
    #[name = "Fuzz Face"]
    FuzzFace,
    #[name = "Big Muff"]
    BigMuff,
    #[name = "MXR Dyna Comp"]
    DynaComp,
    #[name = "ProCo RAT"]
    ProCoRat,
    #[name = "Blues Driver"]
    BluesDriver,
    #[name = "Klon Centaur"]
    KlonCentaur,
    Delay,
}

impl PedalType {
    /// Per-pedal control labels for each knob slot: [drive, tone, level].
    /// `None` means the knob is inactive for this pedal.
    fn control_labels(self) -> [Option<&'static str>; 3] {
        match self {
            Self::TubeScreamer => [Some("Drive"), None, Some("Level")],
            Self::FuzzFace => [Some("Fuzz"), None, Some("Volume")],
            Self::BigMuff => [Some("Sustain"), Some("Tone"), Some("Volume")],
            Self::DynaComp => [Some("Sensitivity"), None, Some("Output")],
            Self::ProCoRat => [Some("Distortion"), Some("Filter"), Some("Volume")],
            Self::BluesDriver => [Some("Gain"), Some("Tone"), Some("Level")],
            Self::KlonCentaur => [Some("Gain"), Some("Treble"), Some("Output")],
            Self::Delay => [Some("Time"), Some("Feedback"), Some("Mix")],
        }
    }

    /// Index into the compiled pedals vec, or `None` for the built-in Delay.
    fn compiled_index(self) -> Option<usize> {
        match self {
            Self::TubeScreamer => Some(0),
            Self::FuzzFace => Some(1),
            Self::BigMuff => Some(2),
            Self::DynaComp => Some(3),
            Self::ProCoRat => Some(4),
            Self::BluesDriver => Some(5),
            Self::KlonCentaur => Some(6),
            Self::Delay => None,
        }
    }
}

/// Recover `PedalType` from the atomic indicator byte.
fn pedal_from_indicator(val: u8) -> PedalType {
    match val {
        0 => PedalType::TubeScreamer,
        1 => PedalType::FuzzFace,
        2 => PedalType::BigMuff,
        3 => PedalType::DynaComp,
        4 => PedalType::ProCoRat,
        5 => PedalType::BluesDriver,
        6 => PedalType::KlonCentaur,
        _ => PedalType::Delay,
    }
}

/// Indicator byte for a `PedalType` (matches enum declaration order).
fn pedal_to_indicator(pt: PedalType) -> u8 {
    match pt {
        PedalType::TubeScreamer => 0,
        PedalType::FuzzFace => 1,
        PedalType::BigMuff => 2,
        PedalType::DynaComp => 3,
        PedalType::ProCoRat => 4,
        PedalType::BluesDriver => 5,
        PedalType::KlonCentaur => 6,
        PedalType::Delay => 7,
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Parameters
// ═══════════════════════════════════════════════════════════════════════════

/// Shared atomic byte used by value-to-string formatters to read the current
/// pedal type without requiring mutable access to the plugin struct.
type PedalIndicator = Arc<AtomicU8>;

#[derive(Params)]
struct PedalKernelParams {
    /// Which pedal model to use.
    #[id = "type"]
    pedal_type: EnumParam<PedalType>,

    /// Primary control (Drive / Fuzz / Sustain / Sensitivity / Gain / Time).
    #[id = "k1"]
    knob1: FloatParam,

    /// Secondary control (Tone / Filter / Treble / Feedback).
    /// Inactive for two-knob pedals.
    #[id = "k2"]
    knob2: FloatParam,

    /// Output control (Level / Volume / Output / Mix).
    #[id = "k3"]
    knob3: FloatParam,
}

impl PedalKernelParams {
    fn new(indicator: PedalIndicator) -> Self {
        let pi1 = indicator.clone();
        let pi2 = indicator.clone();
        let pi3 = indicator.clone();

        Self {
            pedal_type: EnumParam::new("Pedal", PedalType::TubeScreamer),

            knob1: FloatParam::new("Drive", 0.5, FloatRange::Linear { min: 0.0, max: 1.0 })
                .with_smoother(SmoothingStyle::Linear(20.0))
                .with_value_to_string(Arc::new(move |v: f32| {
                    let pt = pedal_from_indicator(pi1.load(Ordering::Relaxed));
                    let label = pt.control_labels()[0].unwrap_or("—");
                    format!("{label}: {v:.2}")
                })),

            knob2: FloatParam::new("Tone", 0.5, FloatRange::Linear { min: 0.0, max: 1.0 })
                .with_smoother(SmoothingStyle::Linear(20.0))
                .with_value_to_string(Arc::new(move |v: f32| {
                    let pt = pedal_from_indicator(pi2.load(Ordering::Relaxed));
                    match pt.control_labels()[1] {
                        Some(label) => format!("{label}: {v:.2}"),
                        None => "—".to_string(),
                    }
                })),

            knob3: FloatParam::new("Level", 0.8, FloatRange::Linear { min: 0.0, max: 1.0 })
                .with_smoother(SmoothingStyle::Linear(20.0))
                .with_value_to_string(Arc::new(move |v: f32| {
                    let pt = pedal_from_indicator(pi3.load(Ordering::Relaxed));
                    let label = pt.control_labels()[2].unwrap_or("—");
                    format!("{label}: {v:.2}")
                })),
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Plugin struct
// ═══════════════════════════════════════════════════════════════════════════

struct PedalKernelPlugin {
    params: Arc<PedalKernelParams>,
    /// Seven compiled WDF pedal processors, indexed by `PedalType::compiled_index()`.
    pedals: Vec<CompiledPedal>,
    /// Built-in digital delay (non-WDF).
    delay: Delay,
    /// Shared indicator so value formatters know the active pedal.
    pedal_indicator: PedalIndicator,
}

fn compile_embedded(src: &str, sample_rate: f64) -> CompiledPedal {
    let def = dsl::parse_pedal_file(src).expect("embedded .pedal file should parse");
    compile_pedal(&def, sample_rate).expect("embedded .pedal file should compile")
}

impl Default for PedalKernelPlugin {
    fn default() -> Self {
        let sr = 48000.0;
        let pedal_indicator: PedalIndicator = Arc::new(AtomicU8::new(0));
        let params = Arc::new(PedalKernelParams::new(pedal_indicator.clone()));
        let pedals: Vec<CompiledPedal> = PEDAL_SOURCES
            .iter()
            .map(|src| compile_embedded(src, sr))
            .collect();

        Self {
            params,
            pedals,
            delay: Delay::new(sr),
            pedal_indicator,
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
        for pedal in &mut self.pedals {
            pedal.set_sample_rate(sr);
        }
        self.delay.set_sample_rate(sr);
        true
    }

    fn reset(&mut self) {
        for pedal in &mut self.pedals {
            pedal.reset();
        }
        self.delay.reset();
    }

    fn process(
        &mut self,
        buffer: &mut Buffer,
        _aux: &mut AuxiliaryBuffers,
        _context: &mut impl ProcessContext<Self>,
    ) -> ProcessStatus {
        let pedal_type = self.params.pedal_type.value();
        self.pedal_indicator
            .store(pedal_to_indicator(pedal_type), Ordering::Relaxed);
        let labels = pedal_type.control_labels();

        for channel_samples in buffer.iter_samples() {
            // Read smoothed knob values (per-sample for zipper-free automation)
            let k1 = self.params.knob1.smoothed.next() as f64;
            let k2 = self.params.knob2.smoothed.next() as f64;
            let k3 = self.params.knob3.smoothed.next() as f64;

            // Route knobs to the active pedal's controls
            match pedal_type.compiled_index() {
                Some(idx) => {
                    let pedal = &mut self.pedals[idx];
                    if let Some(label) = labels[0] {
                        pedal.set_control(label, k1);
                    }
                    if let Some(label) = labels[1] {
                        pedal.set_control(label, k2);
                    }
                    if let Some(label) = labels[2] {
                        pedal.set_control(label, k3);
                    }
                }
                None => {
                    // Delay: map 0–1 knobs to delay-specific ranges
                    // Quadratic curve for natural-feeling time control
                    self.delay.set_delay_time(10.0 + k1 * k1 * 1990.0);
                    self.delay.set_feedback(k2 * 0.95);
                    self.delay.set_mix(k3);
                }
            }

            // Process each channel through the active pedal
            for sample in channel_samples {
                let input = *sample as f64;
                let output = match pedal_type.compiled_index() {
                    Some(idx) => self.pedals[idx].process(input),
                    None => self.delay.process(input),
                };
                *sample = output as f32;
            }
        }

        ProcessStatus::Normal
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// VST3 / CLAP metadata
// ═══════════════════════════════════════════════════════════════════════════

impl Vst3Plugin for PedalKernelPlugin {
    const VST3_CLASS_ID: [u8; 16] = *b"PdlKrnlWdfV0_02\0";
    const VST3_SUBCATEGORIES: &'static [Vst3SubCategory] =
        &[Vst3SubCategory::Fx, Vst3SubCategory::Distortion];
}

impl ClapPlugin for PedalKernelPlugin {
    const CLAP_ID: &'static str = "com.ajmwagar.pedalkernel";
    const CLAP_DESCRIPTION: Option<&'static str> = Some("WDF-based guitar pedal emulations");
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
