//! PedalKernel VST3/CLAP plugin.
//!
//! Single-plugin architecture with a pedal selector and six generic
//! knob parameters.  Built-in pedals are always available; user-supplied
//! `.pedal` and `.board` files are loaded from `~/.pedalkernel/pedals/`
//! at plugin init.
//!
//! The pedal selector is an `IntParam` whose range adapts to the number of
//! loaded entries.  Built-in pedals occupy indices 0–7 (seven WDF-based
//! models plus a digital delay); user pedals follow, then user boards.
//!
//! Controls are mapped sequentially: the first control declared in the
//! `.pedal` file maps to knob 1, the second to knob 2, etc. (up to 6).
//! Unused knobs display "—".  Board entries use their baked-in overrides
//! and show no active knobs.

use nih_plug::prelude::*;
use pedalkernel::board::parse_board_file;
use pedalkernel::compiler::{compile_pedal, CompiledPedal};
use pedalkernel::dsl;
use pedalkernel::pedalboard::PedalboardProcessor;
use pedalkernel::pedals::Delay;
use pedalkernel::PedalProcessor;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Arc;

// ═══════════════════════════════════════════════════════════════════════════
// Embedded .pedal sources (always available)
// ═══════════════════════════════════════════════════════════════════════════

const EMBEDDED_SOURCES: &[&str] = &[
    include_str!("../../pedalkernel/examples/pedals/overdrive/tube_screamer.pedal"),
    include_str!("../../pedalkernel/examples/pedals/fuzz/fuzz_face.pedal"),
    include_str!("../../pedalkernel/examples/pedals/fuzz/big_muff.pedal"),
    include_str!("../../pedalkernel/examples/pedals/compressor/dyna_comp.pedal"),
    include_str!("../../pedalkernel/examples/pedals/distortion/proco_rat.pedal"),
    include_str!("../../pedalkernel/examples/pedals/overdrive/blues_driver.pedal"),
    include_str!("../../pedalkernel/examples/pedals/overdrive/klon_centaur.pedal"),
];

// ═══════════════════════════════════════════════════════════════════════════
// Pro pedals (private repo, feature-gated by variant)
//
// Guitar: overdrives, fuzz, wah — voiced for guitar frequencies.
// Shared: compressors, delays, modulation, reverb — instrument-agnostic.
// Bass:   bass-specific circuits (populated in pedalkernel-pro/pedals/bass/).
//
// `pro-pedals` = guitar + bass + shared (the complete Pro bundle).
// `pro-guitar` = guitar + shared.
// `pro-bass`   = bass + shared.
// ═══════════════════════════════════════════════════════════════════════════

/// Guitar-voiced pedals (overdrives, fuzz, wah).
#[cfg(feature = "pro-guitar")]
const PRO_GUITAR_SOURCES: &[&str] = &[
    include_str!("../../../pedalkernel-pro/pedals/overdrive/fulltone_ocd.pedal"),
    include_str!("../../../pedalkernel-pro/pedals/overdrive/klon_centaur.pedal"),
    include_str!("../../../pedalkernel-pro/pedals/overdrive/morning_glory.pedal"),
    include_str!("../../../pedalkernel-pro/pedals/overdrive/tumnus.pedal"),
    include_str!("../../../pedalkernel-pro/pedals/overdrive/ts808_tubescreamer.pedal"),
    include_str!("../../../pedalkernel-pro/pedals/overdrive/eqd_plumes.pedal"),
    include_str!("../../../pedalkernel-pro/pedals/wah/crybaby_wah.pedal"),
];

/// Instrument-agnostic pedals (compression, delay, modulation, reverb).
/// Included in both guitar and bass variants.
#[cfg(any(feature = "pro-guitar", feature = "pro-bass"))]
const PRO_SHARED_SOURCES: &[&str] = &[
    include_str!("../../../pedalkernel-pro/pedals/compressor/keeley_compressor_plus.pedal"),
    include_str!("../../../pedalkernel-pro/pedals/chorus/ce2_chorus.pedal"),
    include_str!("../../../pedalkernel-pro/pedals/phaser/phase90.pedal"),
    include_str!("../../../pedalkernel-pro/pedals/tremolo/fender_tremolo.pedal"),
    include_str!("../../../pedalkernel-pro/pedals/tremolo/optical_tremolo.pedal"),
    include_str!("../../../pedalkernel-pro/pedals/tremolo/harmonic_tremolo.pedal"),
    include_str!("../../../pedalkernel-pro/pedals/phaser/univibe.pedal"),
    include_str!("../../../pedalkernel-pro/pedals/delay/boss_dm2_delay.pedal"),
    include_str!("../../../pedalkernel-pro/pedals/reverb/walrus_slo_reverb.pedal"),
];

/// Bass-specific pedals (bass overdrive, bass compressor, etc.).
/// Add new bass .pedal files to pedalkernel-pro/pedals/bass/ and list them here.
#[cfg(feature = "pro-bass")]
const PRO_BASS_SOURCES: &[&str] = &[
    // TODO: Add bass-specific pedals as they're created, e.g.:
    // include_str!("../../../pedalkernel-pro/pedals/bass/darkglass_b7k.pedal"),
];

// ═══════════════════════════════════════════════════════════════════════════
// Pedal metadata & loading
// ═══════════════════════════════════════════════════════════════════════════

/// Number of generic knob parameters exposed by the plugin.
const NUM_KNOBS: usize = 6;

/// Immutable metadata for a loaded pedal slot, shared with parameter formatters.
#[derive(Clone)]
struct PedalMeta {
    name: String,
    /// Up to 6 control labels, assigned in declaration order from the `.pedal` file.
    /// `None` means the knob is inactive for this pedal.
    labels: [Option<String>; NUM_KNOBS],
}

/// The actual processor for a pedal slot.
enum PedalEngine {
    Compiled(CompiledPedal),
    Delay(Delay),
    /// A pedalboard (signal chain of multiple pedals from a `.board` file).
    Board(PedalboardProcessor),
}

impl PedalEngine {
    fn process(&mut self, input: f64) -> f64 {
        match self {
            Self::Compiled(p) => p.process(input),
            Self::Delay(d) => d.process(input),
            Self::Board(b) => b.process(input),
        }
    }

    fn set_sample_rate(&mut self, rate: f64) {
        match self {
            Self::Compiled(p) => p.set_sample_rate(rate),
            Self::Delay(d) => d.set_sample_rate(rate),
            Self::Board(b) => b.set_sample_rate(rate),
        }
    }

    fn reset(&mut self) {
        match self {
            Self::Compiled(p) => p.reset(),
            Self::Delay(d) => d.reset(),
            Self::Board(b) => b.reset(),
        }
    }
}

/// Parse and compile a `.pedal` source string into metadata + engine.
fn load_pedal_source(src: &str, sample_rate: f64) -> Option<(PedalMeta, PedalEngine)> {
    let def = dsl::parse_pedal_file(src).ok()?;
    let compiled = compile_pedal(&def, sample_rate).ok()?;

    // Map controls sequentially to knob slots (up to NUM_KNOBS).
    let mut labels: [Option<String>; NUM_KNOBS] = Default::default();
    for (i, ctrl) in def.controls.iter().take(NUM_KNOBS).enumerate() {
        labels[i] = Some(ctrl.label.clone());
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

/// Scan the user pedals directory for `.board` files and compile them.
///
/// Board files define a signal chain of multiple pedals with optional knob
/// overrides.  Each board appears as a single selectable entry in the plugin.
/// The six DAW knobs are inactive for boards — the overrides in the `.board`
/// file set the knob positions.
fn scan_user_boards(sample_rate: f64) -> Vec<(PedalMeta, PedalEngine)> {
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
        .filter(|e| e.path().extension().is_some_and(|ext| ext == "board"))
        .map(|e| e.path())
        .collect();
    paths.sort(); // deterministic ordering

    let mut boards = Vec::new();
    for path in paths {
        if let Ok(src) = std::fs::read_to_string(&path) {
            if let Ok(board_def) = parse_board_file(&src) {
                let board_dir = path.parent().unwrap_or(&dir);
                let (processor, _warnings) =
                    PedalboardProcessor::from_board_with_warnings(&board_def, board_dir, sample_rate);

                let meta = PedalMeta {
                    name: format!("{} [Board]", board_def.name),
                    labels: Default::default(), // no knobs — overrides baked in
                };
                boards.push((meta, PedalEngine::Board(processor)));
            }
        }
    }
    boards
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

    #[id = "k1"]
    knob1: FloatParam,
    #[id = "k2"]
    knob2: FloatParam,
    #[id = "k3"]
    knob3: FloatParam,
    #[id = "k4"]
    knob4: FloatParam,
    #[id = "k5"]
    knob5: FloatParam,
    #[id = "k6"]
    knob6: FloatParam,
}

/// Default display names for each knob slot (used as the parameter name in the DAW).
const KNOB_NAMES: [&str; NUM_KNOBS] = ["Knob 1", "Knob 2", "Knob 3", "Knob 4", "Knob 5", "Knob 6"];

/// Build a single knob `FloatParam` with a formatter that shows the active pedal's label.
fn make_knob(
    name: &str,
    default: f32,
    slot: usize,
    indicator: &PedalIndicator,
    meta: &Arc<Vec<PedalMeta>>,
) -> FloatParam {
    let pi = indicator.clone();
    let m = meta.clone();
    FloatParam::new(name, default, FloatRange::Linear { min: 0.0, max: 1.0 })
        .with_smoother(SmoothingStyle::Linear(20.0))
        .with_value_to_string(Arc::new(move |v: f32| {
            let idx = pi.load(Ordering::Relaxed) as usize;
            match m.get(idx).and_then(|meta| meta.labels[slot].as_deref()) {
                Some(label) => format!("{label}: {v:.2}"),
                None => "\u{2014}".to_string(),
            }
        }))
}

impl PedalKernelParams {
    fn new(indicator: PedalIndicator, meta: Arc<Vec<PedalMeta>>) -> Self {
        let max_idx = (meta.len() as i32 - 1).max(0);

        let meta_sel = meta.clone();

        Self {
            pedal_type: IntParam::new("Pedal", 0, IntRange::Linear { min: 0, max: max_idx })
                .with_value_to_string(Arc::new(move |v| {
                    meta_sel
                        .get(v as usize)
                        .map(|m| m.name.clone())
                        .unwrap_or_else(|| format!("#{v}"))
                })),

            knob1: make_knob(KNOB_NAMES[0], 0.5, 0, &indicator, &meta),
            knob2: make_knob(KNOB_NAMES[1], 0.5, 1, &indicator, &meta),
            knob3: make_knob(KNOB_NAMES[2], 0.5, 2, &indicator, &meta),
            knob4: make_knob(KNOB_NAMES[3], 0.5, 3, &indicator, &meta),
            knob5: make_knob(KNOB_NAMES[4], 0.5, 4, &indicator, &meta),
            knob6: make_knob(KNOB_NAMES[5], 0.5, 5, &indicator, &meta),
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

        // 1b. Pro guitar pedals.
        #[cfg(feature = "pro-guitar")]
        for src in PRO_GUITAR_SOURCES {
            if let Some((m, e)) = load_pedal_source(src, sr) {
                meta_list.push(m);
                engines.push(e);
            }
        }

        // 1c. Pro bass pedals.
        #[cfg(feature = "pro-bass")]
        for src in PRO_BASS_SOURCES {
            if let Some((m, e)) = load_pedal_source(src, sr) {
                meta_list.push(m);
                engines.push(e);
            }
        }

        // 1d. Pro shared pedals (included in both guitar and bass variants).
        #[cfg(any(feature = "pro-guitar", feature = "pro-bass"))]
        for src in PRO_SHARED_SOURCES {
            if let Some((m, e)) = load_pedal_source(src, sr) {
                meta_list.push(m);
                engines.push(e);
            }
        }

        // 2. Built-in digital delay.
        meta_list.push(PedalMeta {
            name: "Delay".to_string(),
            labels: [
                Some("Time".to_string()),
                Some("Feedback".to_string()),
                Some("Mix".to_string()),
                None,
                None,
                None,
            ],
        });
        engines.push(PedalEngine::Delay(Delay::new(sr)));

        // 3. User-supplied .pedal files (indices 8+).
        for (m, e) in scan_user_pedals(sr) {
            meta_list.push(m);
            engines.push(e);
        }

        // 4. User-supplied .board files (pedalboard signal chains).
        for (m, e) in scan_user_boards(sr) {
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
    #[cfg(not(any(feature = "pro-guitar", feature = "pro-bass")))]
    const NAME: &'static str = "PedalKernel";
    #[cfg(all(feature = "pro-guitar", feature = "pro-bass"))]
    const NAME: &'static str = "PedalKernel Pro";
    #[cfg(all(feature = "pro-guitar", not(feature = "pro-bass")))]
    const NAME: &'static str = "PedalKernel Pro Guitar";
    #[cfg(all(feature = "pro-bass", not(feature = "pro-guitar")))]
    const NAME: &'static str = "PedalKernel Pro Bass";

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
            let knobs: [f64; NUM_KNOBS] = [
                self.params.knob1.smoothed.next() as f64,
                self.params.knob2.smoothed.next() as f64,
                self.params.knob3.smoothed.next() as f64,
                self.params.knob4.smoothed.next() as f64,
                self.params.knob5.smoothed.next() as f64,
                self.params.knob6.smoothed.next() as f64,
            ];

            if let Some(engine) = self.engines.get_mut(idx) {
                // Route knobs to the active pedal's controls
                match engine {
                    PedalEngine::Compiled(pedal) => {
                        if let Some(labels) = labels {
                            for (slot, value) in knobs.iter().enumerate() {
                                if let Some(ref label) = labels[slot] {
                                    pedal.set_control(label, *value);
                                }
                            }
                        }
                    }
                    PedalEngine::Delay(delay) => {
                        // Quadratic curve for natural-feeling time control
                        delay.set_delay_time(10.0 + knobs[0] * knobs[0] * 1990.0);
                        delay.set_feedback(knobs[1] * 0.95);
                        delay.set_mix(knobs[2]);
                    }
                    PedalEngine::Board(_) => {
                        // Board knob overrides are baked in at load time;
                        // DAW knobs are inactive for board entries.
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
    #[cfg(not(any(feature = "pro-guitar", feature = "pro-bass")))]
    const VST3_CLASS_ID: [u8; 16] = *b"PdlKrnlWdfV0_03\0";
    #[cfg(all(feature = "pro-guitar", feature = "pro-bass"))]
    const VST3_CLASS_ID: [u8; 16] = *b"PdlKrnlProV0_03\0";
    #[cfg(all(feature = "pro-guitar", not(feature = "pro-bass")))]
    const VST3_CLASS_ID: [u8; 16] = *b"PdlKrnlGtrV0_03\0";
    #[cfg(all(feature = "pro-bass", not(feature = "pro-guitar")))]
    const VST3_CLASS_ID: [u8; 16] = *b"PdlKrnlBssV0_03\0";

    const VST3_SUBCATEGORIES: &'static [Vst3SubCategory] =
        &[Vst3SubCategory::Fx, Vst3SubCategory::Distortion];
}

impl ClapPlugin for PedalKernelPlugin {
    #[cfg(not(any(feature = "pro-guitar", feature = "pro-bass")))]
    const CLAP_ID: &'static str = "com.ajmwagar.pedalkernel";
    #[cfg(all(feature = "pro-guitar", feature = "pro-bass"))]
    const CLAP_ID: &'static str = "com.ajmwagar.pedalkernel-pro";
    #[cfg(all(feature = "pro-guitar", not(feature = "pro-bass")))]
    const CLAP_ID: &'static str = "com.ajmwagar.pedalkernel-pro-guitar";
    #[cfg(all(feature = "pro-bass", not(feature = "pro-guitar")))]
    const CLAP_ID: &'static str = "com.ajmwagar.pedalkernel-pro-bass";

    #[cfg(not(any(feature = "pro-guitar", feature = "pro-bass")))]
    const CLAP_DESCRIPTION: Option<&'static str> = Some("WDF-based guitar pedal emulations");
    #[cfg(all(feature = "pro-guitar", feature = "pro-bass"))]
    const CLAP_DESCRIPTION: Option<&'static str> = Some("WDF-based pedal emulations — Pro edition");
    #[cfg(all(feature = "pro-guitar", not(feature = "pro-bass")))]
    const CLAP_DESCRIPTION: Option<&'static str> = Some("WDF-based guitar pedal emulations — Pro Guitar edition");
    #[cfg(all(feature = "pro-bass", not(feature = "pro-guitar")))]
    const CLAP_DESCRIPTION: Option<&'static str> = Some("WDF-based bass pedal emulations — Pro Bass edition");
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
