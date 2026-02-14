//! PedalKernel VST3/CLAP plugin.
//!
//! Single-plugin architecture with a pedal selector and three generic
//! knob parameters.  Built-in pedals are always available; user-supplied
//! `.pedal` files are loaded from `~/.pedalkernel/pedals/` at plugin init.
//!
//! The pedal selector is an `IntParam` whose range adapts to the number of
//! loaded pedals.  Built-in pedals occupy indices 0–7 (seven WDF-based
//! models plus a digital delay); user pedals follow at index 8+.

use nih_plug::prelude::*;
use pedalkernel::compiler::{compile_pedal, CompiledPedal};
use pedalkernel::dsl;
use pedalkernel::pedals::Delay;
use pedalkernel::PedalProcessor;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Arc;

// ═══════════════════════════════════════════════════════════════════════════
// Embedded .pedal sources (always available)
// ═══════════════════════════════════════════════════════════════════════════

const EMBEDDED_SOURCES: &[&str] = &[
    include_str!("../../pedalkernel/examples/tube_screamer.pedal"),
    include_str!("../../pedalkernel/examples/fuzz_face.pedal"),
    include_str!("../../pedalkernel/examples/big_muff.pedal"),
    include_str!("../../pedalkernel/examples/dyna_comp.pedal"),
    include_str!("../../pedalkernel/examples/proco_rat.pedal"),
    include_str!("../../pedalkernel/examples/blues_driver.pedal"),
    include_str!("../../pedalkernel/examples/klon_centaur.pedal"),
];

// ═══════════════════════════════════════════════════════════════════════════
// Pedal metadata & loading
// ═══════════════════════════════════════════════════════════════════════════

/// Immutable metadata for a loaded pedal slot, shared with parameter formatters.
#[derive(Clone)]
struct PedalMeta {
    name: String,
    /// [primary, secondary, output] — `None` means the knob is inactive.
    labels: [Option<String>; 3],
}

/// The actual processor for a pedal slot.
enum PedalEngine {
    Compiled(CompiledPedal),
    Delay(Delay),
}

impl PedalEngine {
    fn process(&mut self, input: f64) -> f64 {
        match self {
            Self::Compiled(p) => p.process(input),
            Self::Delay(d) => d.process(input),
        }
    }

    fn set_sample_rate(&mut self, rate: f64) {
        match self {
            Self::Compiled(p) => p.set_sample_rate(rate),
            Self::Delay(d) => d.set_sample_rate(rate),
        }
    }

    fn reset(&mut self) {
        match self {
            Self::Compiled(p) => p.reset(),
            Self::Delay(d) => d.reset(),
        }
    }
}

/// Classify a control label into one of the three knob slots:
///   0 = primary (drive/gain/fuzz/etc.)
///   1 = secondary (tone/filter/treble/etc.)
///   2 = output (level/volume/output/mix)
fn knob_slot(label: &str) -> usize {
    match label.to_ascii_lowercase().as_str() {
        "drive" | "gain" | "fuzz" | "sustain" | "distortion" | "sensitivity" | "time" => 0,
        "level" | "volume" | "output" | "mix" => 2,
        _ => 1,
    }
}

/// Parse and compile a `.pedal` source string into metadata + engine.
fn load_pedal_source(src: &str, sample_rate: f64) -> Option<(PedalMeta, PedalEngine)> {
    let def = dsl::parse_pedal_file(src).ok()?;
    let compiled = compile_pedal(&def, sample_rate).ok()?;

    let mut labels: [Option<String>; 3] = [None, None, None];
    for ctrl in &def.controls {
        let slot = knob_slot(&ctrl.label);
        // First control that maps to a slot wins.
        if labels[slot].is_none() {
            labels[slot] = Some(ctrl.label.clone());
        }
    }

    let meta = PedalMeta {
        name: def.name.clone(),
        labels,
    };
    Some((meta, PedalEngine::Compiled(compiled)))
}

/// Resolve the directory to scan for user `.pedal` files.
///
/// Checks `PEDALKERNEL_PEDALS_DIR` first, then falls back to
/// `~/.pedalkernel/pedals/`.
fn user_pedals_dir() -> Option<std::path::PathBuf> {
    if let Ok(dir) = std::env::var("PEDALKERNEL_PEDALS_DIR") {
        return Some(std::path::PathBuf::from(dir));
    }
    std::env::var("HOME")
        .ok()
        .map(|h| std::path::PathBuf::from(h).join(".pedalkernel").join("pedals"))
}

/// Scan the user pedals directory for `.pedal` files and compile them.
fn scan_user_pedals(sample_rate: f64) -> Vec<(PedalMeta, PedalEngine)> {
    let dir = match user_pedals_dir() {
        Some(d) => d,
        None => return Vec::new(),
    };
    let entries = match std::fs::read_dir(&dir) {
        Ok(e) => e,
        Err(_) => return Vec::new(),
    };

    let mut paths: Vec<_> = entries
        .filter_map(|e| e.ok())
        .filter(|e| e.path().extension().is_some_and(|ext| ext == "pedal"))
        .map(|e| e.path())
        .collect();
    paths.sort(); // deterministic ordering

    let mut pedals = Vec::new();
    for path in paths {
        if let Ok(src) = std::fs::read_to_string(&path) {
            if let Some(loaded) = load_pedal_source(&src, sample_rate) {
                pedals.push(loaded);
            }
        }
    }
    pedals
}

// ═══════════════════════════════════════════════════════════════════════════
// Parameters
// ═══════════════════════════════════════════════════════════════════════════

/// Shared atomic byte used by value-to-string formatters to read the current
/// pedal index without requiring mutable access to the plugin struct.
type PedalIndicator = Arc<AtomicU8>;

#[derive(Params)]
struct PedalKernelParams {
    /// Which pedal model to use (index into the loaded pedals list).
    #[id = "type"]
    pedal_type: IntParam,

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
    fn new(indicator: PedalIndicator, meta: Arc<Vec<PedalMeta>>) -> Self {
        let max_idx = (meta.len() as i32 - 1).max(0);

        let meta_sel = meta.clone();
        let meta_k1 = meta.clone();
        let meta_k2 = meta.clone();
        let meta_k3 = meta;
        let pi1 = indicator.clone();
        let pi2 = indicator.clone();
        let pi3 = indicator;

        Self {
            pedal_type: IntParam::new("Pedal", 0, IntRange::Linear { min: 0, max: max_idx })
                .with_value_to_string(Arc::new(move |v| {
                    meta_sel
                        .get(v as usize)
                        .map(|m| m.name.clone())
                        .unwrap_or_else(|| format!("#{v}"))
                })),

            knob1: FloatParam::new("Drive", 0.5, FloatRange::Linear { min: 0.0, max: 1.0 })
                .with_smoother(SmoothingStyle::Linear(20.0))
                .with_value_to_string(Arc::new(move |v: f32| {
                    let idx = pi1.load(Ordering::Relaxed) as usize;
                    let label = meta_k1
                        .get(idx)
                        .and_then(|m| m.labels[0].as_deref())
                        .unwrap_or("—");
                    format!("{label}: {v:.2}")
                })),

            knob2: FloatParam::new("Tone", 0.5, FloatRange::Linear { min: 0.0, max: 1.0 })
                .with_smoother(SmoothingStyle::Linear(20.0))
                .with_value_to_string(Arc::new(move |v: f32| {
                    let idx = pi2.load(Ordering::Relaxed) as usize;
                    match meta_k2
                        .get(idx)
                        .and_then(|m| m.labels[1].as_deref())
                    {
                        Some(label) => format!("{label}: {v:.2}"),
                        None => "—".to_string(),
                    }
                })),

            knob3: FloatParam::new("Level", 0.8, FloatRange::Linear { min: 0.0, max: 1.0 })
                .with_smoother(SmoothingStyle::Linear(20.0))
                .with_value_to_string(Arc::new(move |v: f32| {
                    let idx = pi3.load(Ordering::Relaxed) as usize;
                    let label = meta_k3
                        .get(idx)
                        .and_then(|m| m.labels[2].as_deref())
                        .unwrap_or("—");
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
    /// Metadata for every loaded pedal (shared with parameter formatters).
    meta: Arc<Vec<PedalMeta>>,
    /// One engine per loaded pedal, indexed in lockstep with `meta`.
    engines: Vec<PedalEngine>,
    /// Shared indicator so value formatters know the active pedal index.
    pedal_indicator: PedalIndicator,
}

impl Default for PedalKernelPlugin {
    fn default() -> Self {
        let sr = 48000.0;

        let mut meta_list = Vec::new();
        let mut engines = Vec::new();

        // 1. Embedded WDF pedals (indices 0–6).
        for src in EMBEDDED_SOURCES {
            if let Some((m, e)) = load_pedal_source(src, sr) {
                meta_list.push(m);
                engines.push(e);
            }
        }

        // 2. Built-in digital delay (index 7).
        meta_list.push(PedalMeta {
            name: "Delay".to_string(),
            labels: [
                Some("Time".to_string()),
                Some("Feedback".to_string()),
                Some("Mix".to_string()),
            ],
        });
        engines.push(PedalEngine::Delay(Delay::new(sr)));

        // 3. User-supplied .pedal files (indices 8+).
        for (m, e) in scan_user_pedals(sr) {
            meta_list.push(m);
            engines.push(e);
        }

        let meta = Arc::new(meta_list);
        let pedal_indicator: PedalIndicator = Arc::new(AtomicU8::new(0));
        let params = Arc::new(PedalKernelParams::new(
            pedal_indicator.clone(),
            meta.clone(),
        ));

        Self {
            params,
            meta,
            engines,
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
        for engine in &mut self.engines {
            engine.set_sample_rate(sr);
        }
        true
    }

    fn reset(&mut self) {
        for engine in &mut self.engines {
            engine.reset();
        }
    }

    fn process(
        &mut self,
        buffer: &mut Buffer,
        _aux: &mut AuxiliaryBuffers,
        _context: &mut impl ProcessContext<Self>,
    ) -> ProcessStatus {
        let idx = self.params.pedal_type.value() as usize;
        self.pedal_indicator.store(idx as u8, Ordering::Relaxed);

        let labels = self.meta.get(idx).map(|m| &m.labels);

        for channel_samples in buffer.iter_samples() {
            // Read smoothed knob values (per-sample for zipper-free automation)
            let k1 = self.params.knob1.smoothed.next() as f64;
            let k2 = self.params.knob2.smoothed.next() as f64;
            let k3 = self.params.knob3.smoothed.next() as f64;

            if let Some(engine) = self.engines.get_mut(idx) {
                // Route knobs to the active pedal's controls
                match engine {
                    PedalEngine::Compiled(pedal) => {
                        if let Some(labels) = labels {
                            if let Some(ref label) = labels[0] {
                                pedal.set_control(label, k1);
                            }
                            if let Some(ref label) = labels[1] {
                                pedal.set_control(label, k2);
                            }
                            if let Some(ref label) = labels[2] {
                                pedal.set_control(label, k3);
                            }
                        }
                    }
                    PedalEngine::Delay(delay) => {
                        // Quadratic curve for natural-feeling time control
                        delay.set_delay_time(10.0 + k1 * k1 * 1990.0);
                        delay.set_feedback(k2 * 0.95);
                        delay.set_mix(k3);
                    }
                }

                // Process each channel through the active pedal
                for sample in channel_samples {
                    let input = *sample as f64;
                    let output = engine.process(input);
                    *sample = output as f32;
                }
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
