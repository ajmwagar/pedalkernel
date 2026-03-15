//! Compiled pedal processor: the runtime audio processing chain.

use crate::elements::*;
use crate::loading::{ImpedanceModel, InterstageLoading};
use crate::metering::{MetricsAccumulator, MetricsRingBuffer, UiMetrics};
use crate::oversampling::OversamplingFactor;
use crate::thermal::ThermalModel;
use crate::PedalProcessor;
use std::collections::HashMap;
use std::sync::Arc;

use super::stage::{
    MultiNlStage, NlDeviceGroupKind, NlDeviceKind, PushPullStage, RootKind, SidechainProcessor,
    WdfStage,
};

/// Reference to a stage by type and index, for topological ordering.
#[derive(Clone, Copy, Debug)]
pub(super) enum StageRef {
    Wdf(usize),
    MultiNl(usize),
}

#[cfg(feature = "debug-trace")]
use std::sync::atomic::{AtomicU64, Ordering};

/// Count of traced pipeline samples (non-zero only).
#[cfg(feature = "debug-trace")]
static TRACE_COUNT_PIPELINE: AtomicU64 = AtomicU64::new(0);

/// Max pipeline samples to trace (non-zero only).
#[cfg(feature = "debug-trace")]
const MAX_TRACE_PIPELINE: u64 = 5;

// ═══════════════════════════════════════════════════════════════════════════
// Compiled pedal
// ═══════════════════════════════════════════════════════════════════════════

/// Pre-computed mirror pot IDs (avoids format!() allocation on RT thread).
pub(super) struct MirrorPot {
    pub(super) id: String,
    pub(super) id_aw: String,
    pub(super) id_wb: String,
}

/// Control binding: maps a knob label to a parameter in the processing chain.
pub(super) struct ControlBinding {
    pub(super) label: String,
    pub(super) target: ControlTarget,
    pub(super) component_id: String,
    /// Pre-computed "{component_id}__aw" for 3-terminal pot halves (avoids RT allocation).
    pub(super) component_id_aw: String,
    /// Pre-computed "{component_id}__wb" for 3-terminal pot halves (avoids RT allocation).
    pub(super) component_id_wb: String,
    #[allow(dead_code)]
    pub(super) max_resistance: f64,
    /// Range mapping: (min, max).  Input 0→min, 1→max.
    /// Default (0.0, 1.0) is identity; (1.0, 0.0) inverts.
    pub(super) range: (f64, f64),
}

#[derive(Debug)]
pub(super) enum ControlTarget {
    /// Modify a pot in a specific WDF stage.
    PotInStage(usize),
    /// Modify a pot inside a multi-NL stage (R-type adaptor approach).
    /// (multi_nl_stage_idx, passive_child_idx)
    PotInMultiNlStage(usize, usize),
    /// Forward a control change to a sidechain sub-circuit.
    /// The sidechain's own CompiledPedal handles the pot lookup and
    /// scattering matrix recomputation internally.
    SidechainControl(usize),
    /// Modify an LFO's rate (index into lfos vector).
    LfoRate(usize),
    /// Modify an LFO's depth/amplitude (index into lfos vector).
    LfoDepth(usize),
    /// Modify a delay line's delay time (normalized 0–1).
    DelayTime(usize),
    /// Modify a delay line's feedback amount (0–1).
    DelayFeedback(usize),
    /// Modify a BBD delay line's clock rate (normalized 0–1 → delay time).
    BbdClockRate(usize),
    /// Modify a BBD delay line's feedback amount (0–1).
    BbdFeedback(usize),
    /// Modify a switch position for fork() routing.
    /// Contains: (switch_id, num_positions)
    SwitchPosition {
        switch_id: String,
        num_positions: usize,
    },
    /// Fire a trigger impulse (index into triggers vec).
    Trigger(usize),
}

/// Runtime state for a single-sample impulse source (drum trigger).
#[derive(Debug, Clone)]
pub(super) struct TriggerState {
    pub(super) amplitude: f64,
    pub(super) countdown: u32,
    /// True on the sample when this trigger fired (before tick() clears countdown).
    /// Used by VCA envelope gating which runs after tick().
    pub(super) active_this_sample: bool,
    /// Index of the WDF stage this trigger injects into (None = global input).
    pub(super) target_stage: Option<usize>,
    /// Circuit graph node ID where this trigger injects its impulse.
    /// `usize::MAX` means use global input (backward compat for single-trigger circuits).
    pub(super) injection_node: usize,
}

impl TriggerState {
    pub(super) fn new(amplitude: f64) -> Self {
        Self {
            amplitude,
            countdown: 0,
            active_this_sample: false,
            target_stage: None,
            injection_node: usize::MAX,
        }
    }
    pub(super) fn fire(&mut self) {
        self.countdown = 1;
    }
    /// Returns the impulse value for this sample (amplitude or 0.0).
    /// Also sets `active_this_sample` so downstream consumers (VCA envelopes)
    /// can detect the trigger even after countdown is decremented.
    pub(super) fn tick(&mut self) -> f64 {
        if self.countdown > 0 {
            self.countdown -= 1;
            self.active_this_sample = true;
            self.amplitude
        } else {
            self.active_this_sample = false;
            0.0
        }
    }
}

// PotEffect enum removed — all pot effects are now handled by component-driven
// updates: OpAmpRoot::set_feedback_pot_r(), MultiNlStage::update_bias_from_pot(),
// and bbd_mix_pot_id on CompiledPedal.

/// Modulation target for LFOs and envelope followers.
#[derive(Debug, Clone)]
pub(super) enum ModulationTarget {
    /// Modulate a JFET's Vgs.
    JfetVgs { stage_idx: usize },
    /// Modulate ALL JFET stages' Vgs together (for phasers).
    AllJfetVgs,
    /// Modulate a Photocoupler's LED drive.
    PhotocouplerLed { stage_idx: usize, comp_id: String },
    /// Modulate a Triode's Vgk (grid-cathode bias).
    TriodeVgk { stage_idx: usize },
    /// Modulate a Pentode's Vg1k (control grid bias).
    PentodeVg1k { stage_idx: usize },
    /// Modulate a Variable-mu triode's Vgk (grid-cathode bias).
    VariMuVgk { stage_idx: usize },
    /// Modulate a MOSFET's Vgs.
    MosfetVgs { stage_idx: usize },
    /// Modulate an OTA's bias current (for VCA/compressor) — WDF tree root.
    OtaIabc { stage_idx: usize },
    /// Modulate a linearized OTA's gm in the R-type MNA (for VCA/compressor).
    /// The OTA is stamped as a VCCS in the MNA; gm changes trigger S-matrix recompute.
    OtaIabcLinear { multi_nl_idx: usize },
    /// Modulate a BBD's clock frequency (for chorus/flanger).
    BbdClock { bbd_idx: usize },
    /// Modulate an op-amp's non-inverting input voltage.
    #[allow(dead_code)]
    OpAmpVp { opamp_idx: usize },
    /// Modulate a delay line's speed (for wow/flutter).
    /// The modulation value is added to the speed factor (1.0 = no change).
    DelaySpeed { delay_idx: usize },
    /// Modulate a delay line's delay time (normalized 0–1).
    DelayTime { delay_idx: usize },
}

/// Delay line binding in a compiled pedal.
///
/// Holds the generic delay line and metadata about its taps.
/// The delay line is processed separately from the WDF tree:
/// write once per sample, then read from each tap.
pub(super) struct DelayLineBinding {
    /// The ring-buffer delay line.
    pub(super) delay_line: crate::elements::DelayLine,
    /// Tap ratios for multi-tap reading (e.g., [1.0, 2.0, 4.0] for RE-201).
    /// Each tap reads from the buffer at `base_delay * ratio`.
    pub(super) taps: Vec<f64>,
    /// Component ID for debugging and control binding.
    #[allow(dead_code)]
    pub(super) comp_id: String,
}

/// VCO runtime binding — generates audio-rate waveforms at a circuit node.
pub(super) struct VcoBinding {
    pub(super) vco: crate::elements::Vco,
    /// Which waveform output to use (saw/tri/pulse).
    pub(super) waveform: crate::elements::VcoWaveform,
    /// Circuit node where VCO output is injected.
    pub(super) output_node_id: usize,
    /// Component ID for debugging.
    #[allow(dead_code)]
    pub(super) comp_id: String,
}

/// VCA runtime binding — amplitude modulation controlled by trigger envelope.
pub(super) struct VcaBinding {
    pub(super) vca: crate::elements::Vca,
    /// ADSR envelope that gates the VCA.
    pub(super) envelope: crate::elements::AdsrEnvelope,
    /// Index into triggers vec (which trigger gates this VCA).
    pub(super) trigger_idx: Option<usize>,
    /// Circuit node to read audio input from.
    pub(super) input_node_id: usize,
    /// Circuit node to write amplitude-modulated output to.
    pub(super) output_node_id: usize,
    /// Component ID for debugging.
    #[allow(dead_code)]
    pub(super) comp_id: String,
}

/// Op-amp stage for unity-gain buffers and gain stages.
///
/// Models the op-amp as a voltage-controlled voltage source (VCVS).
/// The output voltage follows: Vout = Aol * (Vp - Vm)
/// For unity-gain buffers (neg tied to out), Vout ≈ Vp.
pub(super) struct OpAmpStage {
    /// The op-amp root element with Newton-Raphson solver.
    pub(super) opamp: OpAmpRoot,
    /// Component ID for debugging.
    #[allow(dead_code)]
    pub(super) comp_id: String,
}

/// LFO binding in a compiled pedal.
pub(super) struct LfoBinding {
    pub(super) lfo: crate::elements::Lfo,
    pub(super) target: ModulationTarget,
    /// Bias offset for the modulation (e.g., Vgs center point).
    pub(super) bias: f64,
    /// Modulation range (amplitude).
    pub(super) range: f64,
    /// Base frequency from RC timing: f = 1/(2πRC).
    pub(super) base_freq: f64,
    /// LFO component ID (for debugging and future control binding).
    #[allow(dead_code)]
    pub(super) lfo_id: String,
}

/// Envelope follower binding in a compiled pedal.
pub(super) struct EnvelopeBinding {
    pub(super) envelope: crate::elements::EnvelopeFollower,
    pub(super) target: ModulationTarget,
    /// Bias offset for the modulation.
    pub(super) bias: f64,
    /// Modulation range (amplitude).
    pub(super) range: f64,
    /// Envelope follower component ID.
    #[allow(dead_code)]
    pub(super) env_id: String,
}

/// Smoothed parameter for zipper-free pot control.
///
/// Uses a one-pole low-pass filter to smoothly interpolate from current
/// to target value. This eliminates clicks/pops when knobs are turned.
///
/// Smoothing time is ~10ms (coefficient ~0.9995 at 48kHz).
#[derive(Debug, Clone)]
pub(super) struct SmoothedParam {
    /// Target value (set immediately by user input)
    pub target: f64,
    /// Current smoothed value (approaches target over time)
    pub current: f64,
    /// Smoothing coefficient (0.9995 = ~10ms at 48kHz)
    pub coef: f64,
    /// Control index in the controls vec (for updating the actual control)
    pub control_idx: usize,
}

impl SmoothedParam {
    /// Create a new smoothed parameter.
    pub fn new(initial: f64, control_idx: usize, sample_rate: f64) -> Self {
        // Smoothing time constant ~10ms = 0.010 seconds
        // coef = exp(-1 / (tau * fs)) ≈ exp(-1 / (0.010 * 48000)) ≈ 0.9979
        let tau = 0.010; // 10ms smoothing
        let coef = (-1.0 / (tau * sample_rate)).exp();
        Self {
            target: initial,
            current: initial,
            coef,
            control_idx,
        }
    }

    /// Advance the smoother by one sample. Returns true if value changed.
    #[inline]
    pub fn advance(&mut self) -> bool {
        self.current = self.coef * self.current + (1.0 - self.coef) * self.target;
        // Snap to target when asymptotically close (avoids infinite tail).
        if (self.current - self.target).abs() < 1e-6 {
            self.current = self.target;
        }
        // Always return true — caller checks is_settled() before calling us,
        // so if we're here the pot needs updating.
        true
    }

    /// Set the target value (called from set_control).
    pub fn set_target(&mut self, value: f64) {
        self.target = value;
    }

    /// Check if we're close enough to target to stop smoothing.
    pub fn is_settled(&self) -> bool {
        (self.current - self.target).abs() < 0.0001
    }

    /// Update smoothing coefficient for new sample rate.
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        let tau = 0.010;
        self.coef = (-1.0 / (tau * sample_rate)).exp();
    }
}

/// Device-aware rail saturation model.
///
/// Real rail saturation depends on the active device's output stage topology.
/// Instead of a generic `tanh` waveshaper, this models the saturation shape
/// based on the dominant active device type in the circuit.
#[derive(Debug, Clone)]
pub(super) enum RailSaturation {
    /// No active devices — passive clipping only (no rail saturation).
    None,
    /// Op-amp output stage: symmetric saturation with slight crossover
    /// distortion near zero. Output stage is push-pull emitter follower
    /// that saturates symmetrically against both rails.
    /// `output_swing_ratio` is Vout_max / Vsupply (typically 0.85–0.95).
    OpAmp { output_swing_ratio: f64 },
    /// BJT common-emitter: asymmetric saturation.  The collector saturates
    /// hard (~0.2V from rail) when driven into saturation, but cuts off
    /// softly when the base drive is removed.  NPN saturates at positive
    /// rail (hard), cuts off toward negative rail (soft).
    Bjt { vce_sat: f64 },
    /// JFET/MOSFET: soft saturation onset due to square-law I-V curve.
    /// The drain current pinches off gradually as Vds approaches Vgs-Vth.
    Fet,
    /// Triode/pentode: grid conduction (hard clip) at positive swing,
    /// plate current cutoff (soft) at negative swing.
    /// `mu` is the amplification factor (affects saturation sharpness).
    Tube { mu: f64 },
}

impl RailSaturation {
    /// Apply rail saturation to a signal given the supply headroom.
    ///
    /// `headroom` is `supply_voltage / 9.0` (normalized to 9V reference).
    #[inline]
    fn process(&self, signal: f64, headroom: f64) -> f64 {
        match self {
            RailSaturation::None => signal,
            RailSaturation::OpAmp { output_swing_ratio } => {
                // Op-amp output stage: symmetric clipping at ±(swing_ratio * headroom).
                // Uses a quintic polynomial for smoother knee than tanh, matching
                // the push-pull emitter-follower output stage characteristic.
                let ceiling = output_swing_ratio * headroom;
                let x = signal / ceiling;
                let ax = x.abs();
                if ax <= 0.75 {
                    // Linear region: no saturation
                    signal
                } else if ax >= 1.5 {
                    // Hard saturation
                    ceiling * x.signum()
                } else {
                    // Knee: quintic ease from 0.75 to ceiling
                    // Normalized to [0, 1] within knee region
                    let t = (ax - 0.75) / 0.75; // 0..1
                    let t2 = t * t;
                    let t3 = t2 * t;
                    // Hermite-like: starts at 0.75, ends at 1.0, smooth at both ends
                    let compressed = 0.75 + 0.25 * (3.0 * t2 - 2.0 * t3);
                    (ceiling * compressed).copysign(signal)
                }
            }
            RailSaturation::Bjt { vce_sat } => {
                // BJT asymmetric saturation.
                // Positive rail (collector saturation): hard knee at Vcc - Vce_sat
                // Negative rail (cutoff): soft exponential tail
                let pos_ceiling = headroom * (1.0 - vce_sat / (9.0 * headroom));
                let neg_ceiling = headroom;
                if signal > 0.0 {
                    // Hard saturation toward positive rail (collector saturation)
                    let x = signal / pos_ceiling;
                    if x <= 0.7 {
                        signal
                    } else if x >= 1.5 {
                        pos_ceiling
                    } else {
                        let t = (x - 0.7) / 0.8;
                        let t2 = t * t;
                        pos_ceiling * (0.7 + 0.3 * (2.0 * t - t2))
                    }
                } else {
                    // Soft cutoff toward negative rail (transistor cutoff)
                    let x = -signal / neg_ceiling;
                    if x <= 0.8 {
                        signal
                    } else {
                        let over = x - 0.8;
                        // Exponential compression: gradual cutoff
                        -(neg_ceiling * (0.8 + 0.2 * (1.0 - (-over * 5.0).exp())))
                    }
                }
            }
            RailSaturation::Fet => {
                // JFET/MOSFET: soft saturation from square-law characteristic.
                // The onset is gentler than BJT — current gradually pinches off.
                let ceiling = headroom;
                let x = signal / ceiling;
                let ax = x.abs();
                if ax <= 0.85 {
                    signal
                } else {
                    // Square-law-like soft compression
                    let over = ax - 0.85;
                    let compressed = 0.85 + 0.15 * (1.0 - 1.0 / (1.0 + over * 3.0));
                    (ceiling * compressed).copysign(signal)
                }
            }
            RailSaturation::Tube { mu } => {
                // Triode: asymmetric saturation.
                // Positive swing → grid conduction (hard clipping, like a diode)
                // Negative swing → plate current cutoff (soft, gradual)
                // Higher mu = sharper saturation knee
                let ceiling = headroom;
                let sharpness = (mu / 100.0).clamp(0.5, 3.0);
                if signal > 0.0 {
                    // Grid conduction — hard clip (grid draws current)
                    let x = signal / ceiling;
                    if x <= 0.6 {
                        signal
                    } else if x >= 2.0 {
                        ceiling
                    } else {
                        let t = (x - 0.6) / 1.4;
                        let curve = 1.0 - (-t * 2.5 * sharpness).exp();
                        ceiling * (0.6 + 0.4 * curve)
                    }
                } else {
                    // Plate cutoff — soft, gradual
                    let x = -signal / ceiling;
                    if x <= 0.9 {
                        signal
                    } else {
                        let over = x - 0.9;
                        let compressed = 0.9 + 0.1 * over / (0.1 + over);
                        -(ceiling * compressed)
                    }
                }
            }
        }
    }
}

/// A pedal processor compiled from a `.pedal` file's netlist.
///
/// Each `.pedal` file produces a unique processor with its own WDF tree topology,
/// component values, and diode models — no hardcoded processor selection.
pub struct CompiledPedal {
    pub(super) stages: Vec<WdfStage>,
    /// Push-pull differential stages (e.g., Fairchild 670 gain cell).
    /// These are processed after regular WDF stages.
    pub(super) push_pull_stages: Vec<PushPullStage>,
    /// Multi-NL stages using R-type adaptor + multi-port NR solver.
    pub(super) multi_nl_stages: Vec<MultiNlStage>,
    pub(super) pre_gain: f64,
    /// Auto-calibrated output gain scalar (1.0 = no calibration).
    pub(super) output_gain: f64,
    pub(super) rail_saturation: RailSaturation,
    /// Oversampler for rail saturation to anti-alias harmonics generated
    /// by the nonlinear clipping at supply rails.
    pub(super) rail_sat_oversampler: crate::oversampling::Oversampler,
    pub(super) sample_rate: f64,
    pub(super) controls: Vec<ControlBinding>,
    pub(super) gain_range: (f64, f64),
    /// Supply voltage in volts (default 9.0).
    pub(super) supply_voltage: f64,
    /// LFO modulators.
    pub(super) lfos: Vec<LfoBinding>,
    /// Envelope follower modulators.
    pub(super) envelopes: Vec<EnvelopeBinding>,
    /// Slew rate limiters for op-amp stages (one per op-amp in the circuit).
    /// Applied to the signal path to model HF compression from slow op-amps.
    pub(super) slew_limiters: Vec<SlewRateLimiter>,
    /// BBD delay lines for delay/chorus/flanger effects.
    pub(super) bbds: Vec<BbdDelayLine>,
    /// Generic delay lines (tape echo, digital delay, Karplus-Strong, etc.).
    pub(super) delay_lines: Vec<DelayLineBinding>,
    /// VCO audio-rate oscillator bindings (inject audio at circuit nodes).
    pub(super) vcos: Vec<VcoBinding>,
    /// VCA amplitude modulation bindings (scale audio by envelope).
    pub(super) vcas: Vec<VcaBinding>,
    /// Thermal model for temperature-dependent behavior.
    /// When present, modulates diode Is and BJT gain over time.
    pub(super) thermal: Option<ThermalModel>,
    /// Tolerance engine seed (stored for diagnostics).
    pub(super) tolerance_seed: u64,
    /// Oversampling factor used for this pedal's nonlinear stages.
    pub(super) oversampling: OversamplingFactor,
    /// Op-amp stages for unity-gain buffers and gain stages.
    /// Each op-amp is modeled as a VCVS with the OpAmpRoot element.
    pub(super) opamp_stages: Vec<OpAmpStage>,
    /// Power supply sag model.
    /// When present, computes instantaneous B+ droop based on signal current draw
    /// and feeds the sagged voltage into `set_supply_voltage()` each sample.
    pub(super) power_supply: Option<PowerSupply>,
    /// Debug statistics for monitoring WDF engine behavior.
    /// When set, the processor will record per-sample stats for debugging.
    #[cfg(debug_assertions)]
    pub(super) debug_stats: Option<std::sync::Arc<crate::debug::DebugStats>>,
    /// Metrics accumulator for real-time UI visualization.
    /// Accumulates per-sample data and reduces to `UiMetrics` every block.
    pub(super) metrics_accumulator: Option<MetricsAccumulator>,
    /// Ring buffer for sending metrics to the UI thread (shared via Arc).
    pub(super) metrics_buffer: Option<Arc<MetricsRingBuffer>>,
    /// Input loading model — models source impedance interaction at circuit input.
    /// Applies DC attenuation and frequency-dependent rolloff based on the
    /// source (e.g., guitar pickup, upstream pedal) driving this circuit.
    pub(super) input_loading: Option<InterstageLoading>,
    /// Output loading model — models load impedance interaction at circuit output.
    /// Applies DC attenuation and frequency-dependent rolloff based on what
    /// this circuit is driving (e.g., downstream pedal, amp input).
    pub(super) output_loading: Option<InterstageLoading>,
    /// Sidechain processors for feedback compression loops.
    /// Each sidechain taps audio, extracts an envelope, and modulates
    /// the push-pull grid bias. Multiple sidechains are supported
    /// (e.g., one per channel in a stereo compressor like the 670).
    pub(super) sidechains: Vec<SidechainProcessor>,
    /// Smoothed parameters for zipper-free pot control.
    /// One per pot in the circuit that needs smoothing.
    pub(super) pot_smoothers: Vec<SmoothedParam>,
    /// Mirrored pot mappings: source_comp_id → list of mirrored pots.
    /// When a source pot is updated, each mirror gets position = 1.0 - source.
    pub(super) pot_mirrors: HashMap<String, Vec<MirrorPot>>,
    /// Base (unmodulated) grid bias for push-pull stages.
    /// Stored so sidechain CV can be subtracted without accumulation.
    pub(super) base_grid_bias: f64,
    /// Sample counter for throttling multi-NL scattering recomputes.
    /// When a pot inside a multi-NL R-type adaptor changes, the O(n³) matrix
    /// inversion is deferred and flushed every 32 samples (~0.7 ms at 48 kHz).
    pub(super) multi_nl_recompute_counter: u32,
    /// Topological ordering of WDF, multi-NL, and coupled BJT stages.
    /// Stages are sorted by signal_flow_distance so earlier stages in the
    /// signal path process first, regardless of stage type.
    pub(super) stage_order: Vec<StageRef>,
    /// Node-based signal routing buffer for parallel-path topologies.
    /// Maps circuit graph node IDs to signal values. When non-empty, multi-NL
    /// stages read from their injection_node_id and write to their output_node_id,
    /// enabling correct parallel channel processing (e.g., 670 sidechain).
    pub(super) node_signals: Vec<(usize, f64)>,
    /// BBD wet/dry mix (0.0 = fully dry, 1.0 = fully wet).
    /// Controlled by "Blend"/"Mix" knobs on BBD delay pedals.
    pub(super) bbd_wet_mix: f64,
    /// Component ID of the pot controlling BBD wet/dry mix.
    /// When set, the pot position is read directly after pot updates.
    pub(super) bbd_mix_pot_id: Option<String>,
    /// Trigger impulse sources for drum/percussion circuits.
    pub(super) triggers: Vec<TriggerState>,
    /// MIDI note → trigger index mapping for per-note dispatch.
    /// When non-empty, `note_on()` fires only the trigger for the given note.
    /// When empty, `note_on()` fires all triggers (backward compatibility).
    pub(super) midi_trigger_map: HashMap<u8, usize>,
    /// Original passive component values for reset support.
    /// Maps comp_id -> (kind_str, original_value).
    pub(super) original_passive_values: HashMap<String, (&'static str, f64)>,
}

/// Gain-like control labels.
#[allow(dead_code)]
pub(super) fn is_gain_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "drive" | "gain" | "fuzz" | "sustain" | "distortion" | "sensitivity"
    )
}

/// Level-like control labels.
#[allow(dead_code)]
pub(super) fn is_level_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "level" | "volume" | "output"
    )
}

/// Rate-like control labels (for LFO).
#[allow(dead_code)]
pub(super) fn is_rate_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "rate" | "speed" | "tempo"
    )
}

/// Depth-like control labels (for LFO modulation amount).
#[allow(dead_code)]
pub(super) fn is_depth_label(label: &str) -> bool {
    matches!(label.to_ascii_lowercase().as_str(), "depth" | "width")
}

/// Delay time control labels.
#[allow(dead_code)]
pub(super) fn is_delay_time_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "time" | "delay" | "repeat_rate" | "echo"
    )
}

/// Delay feedback/intensity control labels.
#[allow(dead_code)]
pub(super) fn is_delay_feedback_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "feedback" | "repeats" | "intensity" | "regeneration"
    )
}

/// Wet/dry mix control labels.
pub(super) fn is_mix_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "mix" | "blend" | "wet" | "dry/wet" | "wet/dry"
    )
}

impl CompiledPedal {
    /// Set the supply voltage and update all voltage-dependent models.
    ///
    /// This affects:
    /// - Headroom for rail saturation modeling
    /// - Op-amp output swing (v_max)
    /// - Triode/pentode plate voltage limits (v_max = B+)
    /// - BJT collector-emitter voltage limits (v_max = Vcc)
    ///
    /// For single-supply pedals biased at Vsupply/2, the op-amp can swing
    /// from about 1.5V to (Vsupply - 1.5V), giving v_max ≈ (Vsupply/2) - 1.5V.
    ///
    /// For tube circuits, the plate voltage can swing from 0V to B+ (supply).
    /// For BJT circuits, Vce can swing from ~0V to Vcc (supply).
    pub fn set_supply_voltage(&mut self, voltage: f64) {
        let prev_voltage = self.supply_voltage;
        // Support negative supplies (PNP positive-ground, e.g. -9V Rangemaster)
        self.supply_voltage = if voltage >= 0.0 {
            voltage.clamp(5.0, 500.0)
        } else {
            voltage.clamp(-500.0, -5.0)
        };

        // Calculate op-amp v_max from supply voltage.
        // For single-supply biased at Vsupply/2: v_max = (Vsupply/2) - saturation_margin
        // Saturation margin is typically 1.0-1.5V for most op-amps.
        // For high voltages (tube gear), the margin stays ~1.5V.
        let saturation_margin = 1.5;
        let abs_voltage = voltage.abs();
        let opamp_v_max = (abs_voltage / 2.0 - saturation_margin).max(1.0);

        // For tubes and transistors, v_max is the magnitude of the supply voltage
        // (plate/collector can swing from 0V to |B+/Vcc|)
        let tube_v_max = abs_voltage;
        let bjt_v_max = abs_voltage;

        // Propagate v_max to standalone op-amp stages
        for stage in &mut self.opamp_stages {
            stage.opamp.set_v_max(opamp_v_max);
        }

        // Propagate v_max to all nonlinear roots in WDF stages
        for stage in &mut self.stages {
            match &mut stage.root {
                RootKind::OpAmp(op) => op.set_v_max(opamp_v_max),
                RootKind::Triode(t) => t.set_v_max(tube_v_max),
                RootKind::VariMu(t) => t.set_v_max(tube_v_max),
                RootKind::Pentode(p) => p.set_v_max(tube_v_max),
                // Diodes, JFETs, MOSFETs, OTAs, Zeners don't need supply voltage
                // (their behavior is determined by their intrinsic parameters)
                _ => {}
            }
        }

        // Propagate v_max to push-pull stages
        for pp in &mut self.push_pull_stages {
            pp.push_root.set_v_max(tube_v_max);
            pp.pull_root.set_v_max(tube_v_max);
        }

        // Propagate v_max to multi-NL stage NL devices
        for stage in &mut self.multi_nl_stages {
            for device in &mut stage.nl_devices {
                match device {
                    NlDeviceKind::Triode(t) => t.set_v_max(tube_v_max),
                    NlDeviceKind::Pentode(p) => p.set_v_max(tube_v_max),
                    NlDeviceKind::VariMu(t) => t.set_v_max(tube_v_max),
                    NlDeviceKind::Diode(_) | NlDeviceKind::DiodePair(_) => {}
                }
            }
            // Propagate to grouped devices (TriodeThreePort, VariMuThreePort)
            if let Some(ref mut dg) = stage.device_groups {
                for group in &mut dg.groups {
                    match group {
                        NlDeviceGroupKind::VariMuThreePort(t) => t.set_v_max(tube_v_max),
                        NlDeviceGroupKind::TriodeThreePort(t) => t.set_v_max(tube_v_max),
                        NlDeviceGroupKind::PentodeThreePort(p) => p.set_v_max(tube_v_max),
                        NlDeviceGroupKind::BjtTwoPort(b) => b.set_v_max(bjt_v_max),
                        NlDeviceGroupKind::SinglePort(d) => match d {
                            NlDeviceKind::Triode(t) => t.set_v_max(tube_v_max),
                            NlDeviceKind::Pentode(p) => p.set_v_max(tube_v_max),
                            NlDeviceKind::VariMu(t) => t.set_v_max(tube_v_max),
                            NlDeviceKind::Diode(_) | NlDeviceKind::DiodePair(_) => {}
                        },
                    }
                }
            }
        }

        // Update multi-NL stage DC bias for new supply voltage.
        for stage in &mut self.multi_nl_stages {
            stage.update_supply_voltage(voltage);
        }

        // Update single-NL WDF stage VCC bias.
        if prev_voltage.abs() > 0.0 {
            let scale = self.supply_voltage / prev_voltage;
            for stage in &mut self.stages {
                if stage.vcc_injection_coeff != 0.0 {
                    stage.vcc_injection_coeff *= scale;
                }
            }
        }
    }

    /// Get the tolerance seed for this pedal unit (for diagnostics/UI).
    pub fn tolerance_seed(&self) -> u64 {
        self.tolerance_seed
    }

    /// Get the oversampling factor used for this pedal.
    pub fn oversampling(&self) -> OversamplingFactor {
        self.oversampling
    }

    /// Get the number of WDF stages and op-amp stages.
    /// Returns (stage_count, opamp_count).
    pub fn wdf_element_counts(&self) -> (u32, u32) {
        let stage_count = self.stages.len() as u32 + self.push_pull_stages.len() as u32;
        (stage_count, self.opamp_stages.len() as u32)
    }

    /// Get the supply voltage.
    pub fn supply_voltage(&self) -> f64 {
        self.supply_voltage
    }

    /// Get the input impedance of this pedal (Ω).
    ///
    /// Returns the WDF port resistance seen looking into the input.
    /// This is the impedance of the first WDF stage's tree.
    /// If the pedal has no stages, returns high impedance (1MΩ).
    pub fn input_impedance(&self) -> f64 {
        self.stages
            .first()
            .map(|s| s.tree.port_resistance())
            .unwrap_or(1_000_000.0) // High-Z default (doesn't load source)
    }

    /// Get the output impedance of this pedal (Ω).
    ///
    /// Returns the WDF port resistance seen looking back into the output.
    /// This is the impedance of the last WDF stage's tree.
    /// If the pedal has no stages, returns low impedance (1kΩ).
    pub fn output_impedance(&self) -> f64 {
        self.stages
            .last()
            .map(|s| s.tree.port_resistance())
            .unwrap_or(1_000.0) // Low-Z default (can drive loads)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Inter-Plugin Impedance API
    // ═══════════════════════════════════════════════════════════════════════════

    /// Set the source impedance driving this circuit's input.
    ///
    /// This models the electrical interaction between whatever is driving this
    /// circuit (guitar pickup, upstream pedal, preamp output) and the circuit's
    /// input stage. The interaction produces:
    ///
    /// - **DC attenuation**: voltage divider formed by source and load impedance
    /// - **Frequency-dependent rolloff**: combined source/load capacitance creates
    ///   a low-pass filter that rolls off high frequencies
    ///
    /// # Arguments
    ///
    /// * `resistance` - Source resistance in Ohms (e.g., 7000 for guitar pickup)
    /// * `capacitance` - Source capacitance in Farads (e.g., 500e-12 for cable)
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Guitar pickup through 15ft cable
    /// pedal.set_source_impedance(7000.0, 500e-12);
    ///
    /// // Buffered pedal output
    /// pedal.set_source_impedance(1000.0, 50e-12);
    /// ```
    pub fn set_source_impedance(&mut self, resistance: f64, capacitance: f64) {
        let source = ImpedanceModel {
            resistance,
            capacitance,
        };

        // Use this circuit's input impedance as the load
        let load = ImpedanceModel {
            resistance: self.input_impedance(),
            capacitance: 50e-12, // Typical input capacitance
        };

        self.input_loading = Some(InterstageLoading::new(source, load, self.sample_rate));
    }

    /// Set the load impedance this circuit drives.
    ///
    /// This models the electrical interaction between this circuit's output stage
    /// and whatever it's driving (downstream pedal, amp input, mixer). The
    /// interaction produces:
    ///
    /// - **DC attenuation**: if the load impedance is low relative to output impedance
    /// - **Frequency-dependent rolloff**: if significant capacitance is present
    ///
    /// # Arguments
    ///
    /// * `resistance` - Load resistance in Ohms (e.g., 1_000_000 for tube amp)
    /// * `capacitance` - Load capacitance in Farads (e.g., 50e-12 for Miller cap)
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Driving a tube amp input
    /// pedal.set_load_impedance(1_000_000.0, 50e-12);
    ///
    /// // Driving a Fuzz Face (low impedance!)
    /// pedal.set_load_impedance(10_000.0, 50e-12);
    /// ```
    pub fn set_load_impedance(&mut self, resistance: f64, capacitance: f64) {
        // Use this circuit's output impedance as the source
        let source = ImpedanceModel {
            resistance: self.output_impedance(),
            capacitance: 30e-12, // Typical output capacitance
        };

        let load = ImpedanceModel {
            resistance,
            capacitance,
        };

        self.output_loading = Some(InterstageLoading::new(source, load, self.sample_rate));
    }

    /// Clear source impedance modeling (use ideal voltage source).
    pub fn clear_source_impedance(&mut self) {
        self.input_loading = None;
    }

    /// Clear load impedance modeling (use ideal open-circuit load).
    pub fn clear_load_impedance(&mut self) {
        self.output_loading = None;
    }

    /// Check if source impedance modeling is active.
    pub fn has_source_impedance(&self) -> bool {
        self.input_loading.is_some()
    }

    /// Check if load impedance modeling is active.
    pub fn has_load_impedance(&self) -> bool {
        self.output_loading.is_some()
    }

    /// Set debug statistics tracking.
    ///
    /// When set, the processor will record per-sample statistics for debugging
    /// issues like clipping, DC offset, NaN/Inf, and automuting.
    #[cfg(debug_assertions)]
    pub fn set_debug_stats(&mut self, stats: std::sync::Arc<crate::debug::DebugStats>) {
        // Configure the stats with engine info
        stats.set_engine_config(
            self.sample_rate as u32,
            self.stages.len() as u32,
            self.opamp_stages.len() as u32,
            self.supply_voltage,
            self.oversampling.ratio() as u32,
        );
        self.debug_stats = Some(stats);
    }

    /// Get a reference to the debug stats (if set).
    #[cfg(debug_assertions)]
    pub fn debug_stats(&self) -> Option<&std::sync::Arc<crate::debug::DebugStats>> {
        self.debug_stats.as_ref()
    }

    /// Enable or disable debug mode.
    #[cfg(debug_assertions)]
    pub fn set_debug_enabled(&self, enabled: bool) {
        if let Some(stats) = &self.debug_stats {
            stats.set_enabled(enabled);
        }
    }

    /// Enable metering for real-time UI visualization.
    ///
    /// Creates a metrics accumulator and ring buffer. Call this before starting
    /// audio processing, then retrieve the ring buffer with `metrics_buffer()`
    /// to read metrics in the UI thread.
    ///
    /// `block_size`: Number of samples between metric reductions (typically 128 or 256).
    pub fn enable_metering(&mut self, block_size: usize) {
        let buffer = Arc::new(MetricsRingBuffer::new(16)); // ~16 frames buffered
        self.metrics_accumulator = Some(MetricsAccumulator::new(block_size));
        self.metrics_buffer = Some(buffer);

        // Set nominal supply voltage for sag calculation
        if let Some(ref mut acc) = self.metrics_accumulator {
            acc.set_nominal_supply(self.supply_voltage as f32);
        }
    }

    /// Get a reference to the metrics ring buffer for UI thread reading.
    ///
    /// Returns `None` if metering is not enabled.
    pub fn metrics_buffer(&self) -> Option<Arc<MetricsRingBuffer>> {
        self.metrics_buffer.clone()
    }

    /// Read the latest metrics (convenience method for the UI thread).
    ///
    /// Returns default metrics if metering is not enabled.
    pub fn read_metrics(&self) -> UiMetrics {
        self.metrics_buffer
            .as_ref()
            .map(|b| b.read_latest())
            .unwrap_or_default()
    }

    /// List all editable passive components across all stages.
    pub fn list_editable_components(&self) -> Vec<(String, &'static str, f64)> {
        let mut result = Vec::new();
        for stage in &self.stages {
            result.extend(stage.tree.list_editable_leaves());
        }
        for stage in &self.push_pull_stages {
            result.extend(stage.push_tree.list_editable_leaves());
            result.extend(stage.pull_tree.list_editable_leaves());
        }
        for stage in &self.multi_nl_stages {
            for child in &stage.passive_children {
                result.extend(child.list_editable_leaves());
            }
        }
        result
    }

    /// Set a passive component's value by comp_id across all stages.
    /// Automatically determines component type from original_passive_values.
    /// After setting, recomputes all affected WDF tree coefficients.
    pub fn set_passive(&mut self, comp_id: &str, value: f64) -> bool {
        let kind = match self.original_passive_values.get(comp_id) {
            Some((k, _)) => *k,
            None => return false,
        };
        let sample_rate = self.sample_rate;
        let mut found = false;

        // Search WDF stages
        for stage in &mut self.stages {
            let hit = match kind {
                "resistor" => stage.tree.set_resistor(comp_id, value),
                "capacitor" => stage.tree.set_capacitor(comp_id, value, sample_rate),
                "inductor" => stage.tree.set_inductor(comp_id, value, sample_rate),
                _ => false,
            };
            if hit {
                stage.tree.recompute();
                found = true;
            }
        }

        // Search push-pull stages
        for stage in &mut self.push_pull_stages {
            let hit_push = match kind {
                "resistor" => stage.push_tree.set_resistor(comp_id, value),
                "capacitor" => stage.push_tree.set_capacitor(comp_id, value, sample_rate),
                "inductor" => stage.push_tree.set_inductor(comp_id, value, sample_rate),
                _ => false,
            };
            let hit_pull = match kind {
                "resistor" => stage.pull_tree.set_resistor(comp_id, value),
                "capacitor" => stage.pull_tree.set_capacitor(comp_id, value, sample_rate),
                "inductor" => stage.pull_tree.set_inductor(comp_id, value, sample_rate),
                _ => false,
            };
            if hit_push {
                stage.push_tree.recompute();
                found = true;
            }
            if hit_pull {
                stage.pull_tree.recompute();
                found = true;
            }
        }

        // Search multi-NL stages (passive children)
        for stage in &mut self.multi_nl_stages {
            for child in &mut stage.passive_children {
                let hit = match kind {
                    "resistor" => child.set_resistor(comp_id, value),
                    "capacitor" => child.set_capacitor(comp_id, value, sample_rate),
                    "inductor" => child.set_inductor(comp_id, value, sample_rate),
                    _ => false,
                };
                if hit {
                    child.recompute();
                    stage.recompute_pending = true;
                    found = true;
                }
            }
        }

        found
    }

    /// Reset a passive component to its original value.
    pub fn reset_passive(&mut self, comp_id: &str) -> bool {
        if let Some(&(_, original_value)) = self.original_passive_values.get(comp_id) {
            self.set_passive(comp_id, original_value)
        } else {
            false
        }
    }

    /// Populate original_passive_values from all stages' current editable leaves.
    /// Should be called once after compilation is complete.
    pub fn snapshot_original_values(&mut self) {
        for (id, kind, value) in self.list_editable_components() {
            self.original_passive_values.entry(id).or_insert((kind, value));
        }
    }

    /// Set a control by its label (e.g., "Drive", "Level", "Rate").
    pub fn set_control(&mut self, label: &str, value: f64) {
        let value = value.clamp(0.0, 1.0);
        let mut found = false;
        for i in 0..self.controls.len() {
            if !self.controls[i].label.eq_ignore_ascii_case(label) {
                continue;
            }
            found = true;
            // Map through range: (0,1)→identity, (1,0)→invert (dual-gang).
            let (r0, r1) = self.controls[i].range;
            let value = (r0 + value * (r1 - r0)).clamp(0.0, 1.0);
            match &self.controls[i].target {
                ControlTarget::PotInStage(_stage_idx) => {
                    // Use smoothing for pot changes to avoid zipper noise / clicks.
                    // Find the smoother for this control and set the target.
                    // The actual pot update happens in advance_smoothers().
                    if let Some(smoother) =
                        self.pot_smoothers.iter_mut().find(|s| s.control_idx == i)
                    {
                        smoother.set_target(value);
                    } else {
                        // Fallback: no smoother (shouldn't happen), update immediately.
                        let stage_idx = *_stage_idx;
                        let comp_id = self.controls[i].component_id.clone();
                        let comp_id_aw = self.controls[i].component_id_aw.clone();
                        let comp_id_wb = self.controls[i].component_id_wb.clone();
                        if let Some(stage) = self.stages.get_mut(stage_idx) {
                            if !stage.tree.set_pot(&comp_id, value) {
                                stage.set_passive_rtype_pot(&comp_id, value);
                            }
                            if !stage.tree.set_pot(&comp_id_aw, value) {
                                stage.set_passive_rtype_pot(&comp_id_aw, value);
                            }
                            if !stage.tree.set_pot(&comp_id_wb, 1.0 - value) {
                                stage.set_passive_rtype_pot(&comp_id_wb, 1.0 - value);
                            }
                            stage.tree.recompute();
                            stage.flush_passive_rtype_recompute();
                        }
                        self.notify_stage_pot_changed(stage_idx);
                        if self.bbd_mix_pot_id.as_deref() == Some(&*comp_id) {
                            self.bbd_wet_mix = value;
                        }
                        self.apply_pot_mirrors(&comp_id, value);
                    }
                }
                ControlTarget::PotInMultiNlStage(stage_idx, _child_idx) => {
                    if let Some(smoother) =
                        self.pot_smoothers.iter_mut().find(|s| s.control_idx == i)
                    {
                        smoother.set_target(value);
                    } else {
                        let stage_idx = *stage_idx;
                        let comp_id = &self.controls[i].component_id;
                        if let Some(stage) = self.multi_nl_stages.get_mut(stage_idx) {
                            stage.set_pot(comp_id, value);
                        }
                    }
                }
                ControlTarget::SidechainControl(sc_idx) => {
                    let sc_idx = *sc_idx;
                    if let Some(sc) = self.sidechains.get_mut(sc_idx) {
                        sc.set_control(label, value);
                    }
                }
                ControlTarget::LfoRate(lfo_idx) => {
                    let lfo_idx = *lfo_idx;
                    if let Some(binding) = self.lfos.get_mut(lfo_idx) {
                        // Scale rate around base_freq: 0.1x to 10x (100x range)
                        // pot=0 -> 0.1x, pot=0.5 -> 1x, pot=1 -> 10x
                        let scale = 0.1_f64 * 100.0_f64.powf(value);
                        let rate = binding.base_freq * scale;
                        binding.lfo.set_rate(rate);
                    }
                }
                ControlTarget::LfoDepth(lfo_idx) => {
                    let lfo_idx = *lfo_idx;
                    if let Some(binding) = self.lfos.get_mut(lfo_idx) {
                        binding.lfo.set_depth(value);
                    }
                }
                ControlTarget::DelayTime(delay_idx) => {
                    let delay_idx = *delay_idx;
                    if let Some(dl) = self.delay_lines.get_mut(delay_idx) {
                        dl.delay_line.set_delay_normalized(value);
                    }
                }
                ControlTarget::DelayFeedback(delay_idx) => {
                    let delay_idx = *delay_idx;
                    if let Some(dl) = self.delay_lines.get_mut(delay_idx) {
                        dl.delay_line.set_feedback(value * 0.95); // Scale to max 0.95
                    }
                }
                ControlTarget::BbdClockRate(bbd_idx) => {
                    let bbd_idx = *bbd_idx;
                    if let Some(bbd) = self.bbds.get_mut(bbd_idx) {
                        bbd.set_delay_normalized(value);
                    }
                }
                ControlTarget::BbdFeedback(bbd_idx) => {
                    let bbd_idx = *bbd_idx;
                    if let Some(bbd) = self.bbds.get_mut(bbd_idx) {
                        bbd.set_feedback(value * 0.95);
                    }
                }
                ControlTarget::SwitchPosition {
                    switch_id,
                    num_positions,
                } => {
                    // Convert normalized 0-1 value to discrete position index
                    // value=0 → pos=0, value=1 → pos=num_positions-1
                    let num_positions = *num_positions;
                    let position = if num_positions <= 1 {
                        0
                    } else {
                        ((value * (num_positions as f64 - 0.001)) as usize).min(num_positions - 1)
                    };
                    // Update all stages' switched resistors
                    for stage in &mut self.stages {
                        stage.tree.set_switch_position(switch_id, position);
                        stage.tree.recompute();
                    }
                }
                ControlTarget::Trigger(idx) => {
                    if value > 0.5 {
                        if let Some(trig) = self.triggers.get_mut(*idx) {
                            trig.fire();
                        }
                    }
                }
            }
        }

        if !found {
            // Forward unmatched controls to sidechain sub-circuits.
            // Sidechain controls (e.g., "Lat Time Constant") are compiled into
            // the sidechain's own CompiledPedal and handled there.
            for sc in &mut self.sidechains {
                sc.set_control(label, value);
            }
        }
    }

    /// Handle MIDI note-on by firing trigger inputs.
    /// If MIDI bindings are present, fires only the trigger for the given note.
    /// Otherwise, fires all triggers (backward compatibility).
    pub fn note_on(&mut self, note: u8, _velocity: u8) {
        if self.midi_trigger_map.is_empty() {
            #[cfg(debug_assertions)]
            eprintln!("[CompiledPedal] note_on({}) → firing ALL {} triggers", note, self.triggers.len());
            for trig in &mut self.triggers {
                trig.fire();
            }
        } else if let Some(&idx) = self.midi_trigger_map.get(&note) {
            #[cfg(debug_assertions)]
            eprintln!("[CompiledPedal] note_on({}) → trigger[{}]", note, idx);
            if let Some(trig) = self.triggers.get_mut(idx) {
                trig.fire();
            }
        } else {
            #[cfg(debug_assertions)]
            eprintln!("[CompiledPedal] note_on({}) → no trigger mapped", note);
        }
    }

    /// Notify a WDF stage that a pot changed and it should update
    /// component-owned behaviors (e.g., OpAmpRoot gain from feedback pot).
    fn notify_stage_pot_changed(&mut self, stage_idx: usize) {
        if let Some(stage) = self.stages.get_mut(stage_idx) {
            stage.notify_pot_changed();
        }
    }

    /// Notify multi-NL stages that a pot changed and they should update
    /// bias from their bias_pot_id if it matches.
    fn notify_multi_nl_pot_changed(&mut self, comp_id: &str) {
        for stage in &mut self.multi_nl_stages {
            if stage.bias_pot_id.as_deref() == Some(comp_id) {
                stage.update_bias_from_pot();
            }
        }
    }

    /// Update mirrored pots (position = 1.0 - source) across all stages.
    fn apply_pot_mirrors(&mut self, comp_id: &str, value: f64) {
        // Collect mirror info to avoid borrow conflict with self.stages.
        let mirror_info: Option<Vec<(String, String, String)>> =
            self.pot_mirrors.get(comp_id).map(|mirrors| {
                mirrors
                    .iter()
                    .map(|mp| (mp.id.clone(), mp.id_aw.clone(), mp.id_wb.clone()))
                    .collect()
            });
        if let Some(mirrors) = mirror_info {
            let inv = 1.0 - value;
            for (id, id_aw, id_wb) in &mirrors {
                for stage in &mut self.stages {
                    if !stage.tree.set_pot(id, inv) {
                        stage.set_passive_rtype_pot(id, inv);
                    }
                    if !stage.tree.set_pot(id_aw, inv) {
                        stage.set_passive_rtype_pot(id_aw, inv);
                    }
                    if !stage.tree.set_pot(id_wb, 1.0 - inv) {
                        stage.set_passive_rtype_pot(id_wb, 1.0 - inv);
                    }
                    stage.tree.recompute();
                    stage.flush_passive_rtype_recompute();
                }
            }
        }
    }

    /// Advance all pot smoothers and update WDF trees where values have changed.
    ///
    /// Call this once per sample (or per block for efficiency) to smoothly
    /// interpolate pot values toward their targets. This eliminates zipper
    /// noise and clicks when knobs are turned.
    ///
    /// Multi-NL scattering recomputes are throttled to every 32 samples
    /// (~0.7 ms at 48 kHz) to avoid the O(n³) matrix inversion cost on
    /// every sample.  The pot resistance in the DynNode is still updated
    /// per-sample so the passive port impedance tracks smoothly.
    #[inline]
    fn advance_smoothers(&mut self) {
        for si in 0..self.pot_smoothers.len() {
            if self.pot_smoothers[si].is_settled() {
                continue;
            }
            if self.pot_smoothers[si].advance() {
                // Value changed, update the actual pot.
                // Clone control info to avoid borrow conflict with &mut self methods.
                let ctrl_idx = self.pot_smoothers[si].control_idx;
                let value = self.pot_smoothers[si].current;
                let comp_id = self.controls[ctrl_idx].component_id.clone();
                let comp_id_aw = self.controls[ctrl_idx].component_id_aw.clone();
                let comp_id_wb = self.controls[ctrl_idx].component_id_wb.clone();
                let max_r = self.controls[ctrl_idx].max_resistance;
                match &self.controls[ctrl_idx].target {
                    ControlTarget::PotInStage(stage_idx) => {
                        let stage_idx = *stage_idx;
                        if let Some(stage) = self.stages.get_mut(stage_idx) {
                            if !stage.tree.set_pot(&comp_id, value) {
                                stage.set_passive_rtype_pot(&comp_id, value);
                            }
                            if !stage.tree.set_pot(&comp_id_aw, value) {
                                stage.set_passive_rtype_pot(&comp_id_aw, value);
                            }
                            if !stage.tree.set_pot(&comp_id_wb, 1.0 - value) {
                                stage.set_passive_rtype_pot(&comp_id_wb, 1.0 - value);
                            }
                            stage.tree.recompute();
                            stage.flush_passive_rtype_recompute();
                        }
                        // Also update the same pot in other WDF stages where it
                        // may appear (e.g., opamp feedback pot also in diode stage).
                        for (oi, stage) in self.stages.iter_mut().enumerate() {
                            if oi == stage_idx {
                                continue;
                            }
                            let mut changed = false;
                            changed |= stage.tree.set_pot(&comp_id, value);
                            changed |= stage.tree.set_pot(&comp_id_aw, value);
                            changed |= stage.tree.set_pot(&comp_id_wb, 1.0 - value);
                            if !changed {
                                // Try PassiveRType children in this stage too.
                                changed |= stage.set_passive_rtype_pot(&comp_id, value);
                                changed |= stage.set_passive_rtype_pot(&comp_id_aw, value);
                                changed |= stage.set_passive_rtype_pot(&comp_id_wb, 1.0 - value);
                            }
                            if changed {
                                stage.tree.recompute();
                                stage.flush_passive_rtype_recompute();
                            }
                        }
                        // Component-driven updates: OpAmpRoot gain, BBD mix
                        self.notify_stage_pot_changed(stage_idx);
                        if self.bbd_mix_pot_id.as_deref() == Some(&comp_id) {
                            self.bbd_wet_mix = value;
                        }
                        self.apply_pot_mirrors(&comp_id, value);
                    }
                    ControlTarget::PotInMultiNlStage(stage_idx, _child_idx) => {
                        let stage_idx = *stage_idx;
                        if let Some(stage) = self.multi_nl_stages.get_mut(stage_idx) {
                            stage.set_pot(&comp_id, value);
                            stage.update_bias_from_pot();
                            // Table lookup is cheap enough for per-sample recompute
                            if max_r < 5_000.0 || stage.interp_table.is_some() {
                                stage.flush_recompute();
                            }
                        }
                    }
                    _ => {}
                }
            }
        }

        // Immediately flush table-based PassiveRType stages (lookup is cheap).
        for stage in &mut self.stages {
            if stage.has_interp_table() {
                stage.flush_passive_rtype_recompute();
            }
        }

        // Throttle scattering recomputes to every 32 samples.
        // The pot resistance in the DynNode is updated per-sample above,
        // but the expensive matrix re-derivation is batched here.
        self.multi_nl_recompute_counter += 1;
        if self.multi_nl_recompute_counter >= 32 {
            self.multi_nl_recompute_counter = 0;
            for stage in &mut self.multi_nl_stages {
                stage.flush_recompute();
            }
            // Also flush PassiveRType stages with dirty pot changes (non-table stages)
            for stage in &mut self.stages {
                stage.flush_passive_rtype_recompute();
            }
        }
    }

    /// Debug dump: print complete pedal structure for debugging.
    ///
    /// Number of WDF stages (for debug reporting).
    pub fn debug_stage_count(&self) -> usize {
        self.stages.len()
    }
    /// Number of push-pull stages (for debug reporting).
    pub fn debug_push_pull_count(&self) -> usize {
        self.push_pull_stages.len()
    }
    /// Number of multi-NL stages (for debug reporting).
    pub fn debug_multi_nl_count(&self) -> usize {
        self.multi_nl_stages.len()
    }

    /// Shows gain structure, all WDF stages with their trees, and control bindings.
    pub fn debug_dump(&self) -> String {
        let mut s = String::new();
        s.push_str("═══════════════════════════════════════════════════════════════════════════\n");
        s.push_str(&"CompiledPedal Debug Dump\n".to_string());
        s.push_str(
            "═══════════════════════════════════════════════════════════════════════════\n\n",
        );

        s.push_str(&format!("Sample Rate: {} Hz\n", self.sample_rate));
        s.push_str(&format!("Supply Voltage: {:.1}V\n", self.supply_voltage));
        s.push_str(&format!(
            "Pre-Gain: {:.6} ({:.2} dB)\n",
            self.pre_gain,
            20.0 * self.pre_gain.log10()
        ));
        s.push_str(&format!(
            "Gain Range: ({:.6}, {:.6})\n",
            self.gain_range.0, self.gain_range.1
        ));
        s.push_str(&format!("Oversampling: {:?}\n\n", self.oversampling));

        s.push_str(&format!("WDF Stages: {}\n", self.stages.len()));
        s.push_str("───────────────────────────────────────────────────────────────────────────\n");
        for (i, stage) in self.stages.iter().enumerate() {
            s.push_str(&format!("\n[Stage {}]\n", i));
            s.push_str(&stage.debug_dump());
            s.push('\n');
        }

        if !self.multi_nl_stages.is_empty() {
            s.push_str(&format!(
                "\nMulti-NL Stages: {}\n",
                self.multi_nl_stages.len()
            ));
            s.push_str(
                "───────────────────────────────────────────────────────────────────────────\n",
            );
            for (i, mnl) in self.multi_nl_stages.iter().enumerate() {
                s.push_str(&format!("  [{}] {}\n", i, mnl.debug_dump()));
            }
        }

        if !self.push_pull_stages.is_empty() {
            s.push_str(&format!(
                "\nPush-Pull Stages: {}\n",
                self.push_pull_stages.len()
            ));
            s.push_str(
                "───────────────────────────────────────────────────────────────────────────\n",
            );
            for (i, pp) in self.push_pull_stages.iter().enumerate() {
                s.push_str(&format!("  [{}] {}\n", i, pp.debug_dump()));
            }
        }

        if !self.delay_lines.is_empty() {
            s.push_str(&format!("\nDelay Lines: {}\n", self.delay_lines.len()));
            s.push_str(
                "───────────────────────────────────────────────────────────────────────────\n",
            );
            for (i, dl) in self.delay_lines.iter().enumerate() {
                s.push_str(&format!(
                    "  [{}] {} - {} taps\n",
                    i,
                    dl.comp_id,
                    dl.taps.len()
                ));
            }
        }

        if !self.controls.is_empty() {
            s.push_str("\nControls:\n");
            s.push_str(
                "───────────────────────────────────────────────────────────────────────────\n",
            );
            for ctrl in &self.controls {
                s.push_str(&format!("  {} -> {:?}\n", ctrl.label, ctrl.target));
            }
        }

        if !self.vcos.is_empty() {
            s.push_str(&format!("\nVCO Bindings: {}\n", self.vcos.len()));
            s.push_str(
                "───────────────────────────────────────────────────────────────────────────\n",
            );
            for (i, vco) in self.vcos.iter().enumerate() {
                s.push_str(&format!(
                    "  [{}] {} - freq={:.1}Hz, waveform={:?}, out_node={}\n",
                    i, vco.comp_id, vco.vco.frequency(), vco.waveform, vco.output_node_id
                ));
            }
        }

        if !self.vcas.is_empty() {
            s.push_str(&format!("\nVCA Bindings: {}\n", self.vcas.len()));
            s.push_str(
                "───────────────────────────────────────────────────────────────────────────\n",
            );
            for (i, vca) in self.vcas.iter().enumerate() {
                let trig = match vca.trigger_idx {
                    Some(idx) => format!("trigger {idx}"),
                    None => "none".to_string(),
                };
                s.push_str(&format!(
                    "  [{}] {} - in_node={}, out_node={}, trigger={}\n",
                    i, vca.comp_id, vca.input_node_id, vca.output_node_id, trig
                ));
            }
        }

        if !self.triggers.is_empty() {
            s.push_str(&format!("\nTriggers: {}\n", self.triggers.len()));
            s.push_str(
                "───────────────────────────────────────────────────────────────────────────\n",
            );
            for (i, trig) in self.triggers.iter().enumerate() {
                let target = match trig.target_stage {
                    Some(idx) => format!("stage {idx}"),
                    None => "global".to_string(),
                };
                let inj = if trig.injection_node != usize::MAX {
                    format!(", inj_node={}", trig.injection_node)
                } else {
                    String::new()
                };
                s.push_str(&format!(
                    "  [{}] amp={:.1}V, target={}{}\n",
                    i, trig.amplitude, target, inj
                ));
            }
        }

        s.push_str(&format!("\nStage Order: {:?}\n", self.stage_order));

        s
    }
}

impl PedalProcessor for CompiledPedal {
    fn process(&mut self, input: f64) -> f64 {
        // Fire any pending triggers.
        // Triggers with explicit injection nodes route through node_signals
        // for per-voice WDF stage routing. Triggers without (injection_node == MAX)
        // sum into a global impulse that replaces the input signal.
        self.node_signals.clear();
        let mut global_trigger: f64 = 0.0;
        for trigger in &mut self.triggers {
            let impulse = trigger.tick();
            if impulse != 0.0 {
                if trigger.injection_node != usize::MAX {
                    self.node_signals.push((trigger.injection_node, impulse));
                } else {
                    global_trigger += impulse;
                }
            }
        }
        let input = if global_trigger != 0.0 { global_trigger } else { input };

        // Advance pot smoothers — smoothly interpolate pot values toward targets.
        // This eliminates zipper noise and clicks when knobs are turned.
        self.advance_smoothers();

        // Tick thermal model — temperature changes are very slow (updated
        // every ~1000 samples internally), so the overhead is negligible.
        // The thermal state modulates diode Is and n_vt for each stage.
        if let Some(ref mut thermal) = self.thermal {
            let state = *thermal.tick();
            for stage in &mut self.stages {
                stage.apply_thermal(&state);
            }
        }

        // Tick all LFOs and route their outputs to targets.
        for binding in &mut self.lfos {
            let lfo_out = binding.lfo.tick();
            let modulation = binding.bias + lfo_out * binding.range;

            match &binding.target {
                ModulationTarget::JfetVgs { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage.set_jfet_vgs(modulation);
                    }
                }
                ModulationTarget::AllJfetVgs => {
                    // Modulate ALL JFET stages with the same value (for phasers)
                    for stage in &mut self.stages {
                        if matches!(&stage.root, RootKind::Jfet(_) | RootKind::JfetVr(_)) {
                            stage.set_jfet_vgs(modulation);
                        }
                    }
                }
                ModulationTarget::PhotocouplerLed { stage_idx, comp_id } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage
                            .tree
                            .set_photocoupler_led(comp_id, modulation.clamp(0.0, 1.0));
                        stage.tree.recompute();
                    }
                }
                ModulationTarget::TriodeVgk { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage.set_triode_vgk(modulation);
                    }
                }
                ModulationTarget::PentodeVg1k { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage.set_pentode_vg1k(modulation);
                    }
                }
                ModulationTarget::VariMuVgk { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage.set_vari_mu_vgk(modulation);
                    }
                }
                ModulationTarget::MosfetVgs { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage.set_mosfet_vgs(modulation);
                    }
                }
                ModulationTarget::OtaIabc { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        // LFO modulation of OTA: modulation maps to gain (0-1)
                        stage.set_ota_gain(modulation.clamp(0.0, 1.0));
                    }
                }
                ModulationTarget::OtaIabcLinear { multi_nl_idx } => {
                    if let Some(stage) = self.multi_nl_stages.get_mut(*multi_nl_idx) {
                        stage.set_ota_gain_linear(modulation.clamp(0.0, 1.0));
                    }
                }
                ModulationTarget::BbdClock { bbd_idx } => {
                    if let Some(bbd) = self.bbds.get_mut(*bbd_idx) {
                        // LFO modulates delay time: modulation = normalized 0-1
                        bbd.set_delay_normalized(modulation.clamp(0.0, 1.0));
                    }
                }
                ModulationTarget::OpAmpVp { opamp_idx } => {
                    if let Some(opamp_stage) = self.opamp_stages.get_mut(*opamp_idx) {
                        // LFO modulates op-amp's non-inverting input
                        opamp_stage.opamp.set_vp(modulation);
                    }
                }
                ModulationTarget::DelaySpeed { delay_idx } => {
                    if let Some(dl) = self.delay_lines.get_mut(*delay_idx) {
                        // LFO modulates tape speed: modulation is additive offset
                        // from 1.0 (center). Typical wow: ±0.02, flutter: ±0.005.
                        dl.delay_line.add_speed_mod(lfo_out * binding.range);
                    }
                }
                ModulationTarget::DelayTime { delay_idx } => {
                    if let Some(dl) = self.delay_lines.get_mut(*delay_idx) {
                        // LFO modulates delay time (normalized 0–1).
                        dl.delay_line
                            .set_delay_normalized(modulation.clamp(0.0, 1.0));
                    }
                }
            }
        }

        // Tick all envelope followers and route their outputs to targets.
        for binding in &mut self.envelopes {
            let env_out = binding.envelope.process(input);
            let modulation = binding.bias + env_out * binding.range;

            match &binding.target {
                ModulationTarget::JfetVgs { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage.set_jfet_vgs(modulation);
                    }
                }
                ModulationTarget::AllJfetVgs => {
                    // Modulate ALL JFET stages with the same value (for phasers)
                    for stage in &mut self.stages {
                        if matches!(&stage.root, RootKind::Jfet(_) | RootKind::JfetVr(_)) {
                            stage.set_jfet_vgs(modulation);
                        }
                    }
                }
                ModulationTarget::PhotocouplerLed { stage_idx, comp_id } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage
                            .tree
                            .set_photocoupler_led(comp_id, modulation.clamp(0.0, 1.0));
                        stage.tree.recompute();
                    }
                }
                ModulationTarget::TriodeVgk { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage.set_triode_vgk(modulation);
                    }
                }
                ModulationTarget::PentodeVg1k { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage.set_pentode_vg1k(modulation);
                    }
                }
                ModulationTarget::VariMuVgk { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage.set_vari_mu_vgk(modulation);
                    }
                }
                ModulationTarget::MosfetVgs { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage.set_mosfet_vgs(modulation);
                    }
                }
                ModulationTarget::OtaIabc { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        // Envelope controls OTA gain: louder input → lower gain
                        // Invert: high envelope (loud) → low gain (compression)
                        let gain = (1.0 - modulation).clamp(0.0, 1.0);
                        stage.set_ota_gain(gain);
                    }
                }
                ModulationTarget::OtaIabcLinear { multi_nl_idx } => {
                    if let Some(stage) = self.multi_nl_stages.get_mut(*multi_nl_idx) {
                        // Envelope controls linearized OTA: louder → lower gm
                        let gain = (1.0 - modulation).clamp(0.0, 1.0);
                        stage.set_ota_gain_linear(gain);
                    }
                }
                ModulationTarget::BbdClock { bbd_idx } => {
                    if let Some(bbd) = self.bbds.get_mut(*bbd_idx) {
                        bbd.set_delay_normalized(modulation.clamp(0.0, 1.0));
                    }
                }
                ModulationTarget::OpAmpVp { opamp_idx } => {
                    if let Some(opamp_stage) = self.opamp_stages.get_mut(*opamp_idx) {
                        opamp_stage.opamp.set_vp(modulation);
                    }
                }
                ModulationTarget::DelaySpeed { delay_idx } => {
                    if let Some(dl) = self.delay_lines.get_mut(*delay_idx) {
                        dl.delay_line.add_speed_mod(env_out * binding.range);
                    }
                }
                ModulationTarget::DelayTime { delay_idx } => {
                    if let Some(dl) = self.delay_lines.get_mut(*delay_idx) {
                        dl.delay_line
                            .set_delay_normalized(modulation.clamp(0.0, 1.0));
                    }
                }
            }
        }

        // Tick VCOs — generate audio-rate waveforms and inject into node_signals.
        // This happens before WDF stages so the VCO audio is available as input.
        for vco_binding in &mut self.vcos {
            let (saw, tri, pulse) = vco_binding.vco.tick();
            let sample = match vco_binding.waveform {
                crate::elements::VcoWaveform::Saw => saw,
                crate::elements::VcoWaveform::Triangle => tri,
                crate::elements::VcoWaveform::Pulse => pulse,
            };
            self.node_signals.push((vco_binding.output_node_id, sample));
        }

        // Record input for debug stats
        #[cfg(debug_assertions)]
        if let Some(ref stats) = self.debug_stats {
            stats.record_input(input);
        }

        // Power supply sag: compute instantaneous current draw from signal
        // level and update the supply voltage.  The power supply model tracks
        // the filter cap charge/discharge dynamics, producing B+ droop under
        // load that recovers at a rate set by the filter capacitance.
        if let Some(ref mut psu) = self.power_supply {
            // Estimate current draw from the signal amplitude.
            // In a tube amp, plate current ≈ signal voltage / plate load resistance.
            // We use a nominal 100kΩ load impedance for the normalized estimate.
            let current_draw = input.abs() * self.pre_gain / 100_000.0;
            let sagged_voltage = psu.tick(current_draw);
            // Propagate the sagged voltage to all voltage-dependent elements.
            // This is the same mechanism as set_supply_voltage() but avoids
            // the clamp so we can track the dynamic sag precisely.
            let prev_supply_voltage = self.supply_voltage;
            self.supply_voltage = sagged_voltage;
            // Update v_max for all root elements
            let saturation_margin = 1.5;
            let abs_sagged = sagged_voltage.abs();
            let opamp_v_max = (abs_sagged / 2.0 - saturation_margin).max(1.0);
            let tube_v_max = abs_sagged;
            let bjt_v_max = abs_sagged;
            for stage in &mut self.opamp_stages {
                stage.opamp.set_v_max(opamp_v_max);
            }
            for stage in &mut self.stages {
                match &mut stage.root {
                    RootKind::OpAmp(op) => op.set_v_max(opamp_v_max),
                    RootKind::Triode(t) => t.set_v_max(tube_v_max),
                    RootKind::Pentode(p) => p.set_v_max(tube_v_max),
                    _ => {}
                }
            }
            for pp in &mut self.push_pull_stages {
                pp.push_root.set_v_max(tube_v_max);
                pp.pull_root.set_v_max(tube_v_max);
            }
            // Update multi-NL stage DC bias for supply sag.
            // dc_bias and vcc_bias_all scale linearly with supply voltage.
            for stage in &mut self.multi_nl_stages {
                stage.update_supply_voltage(sagged_voltage);
            }
            // Update single-NL WDF stage VCC bias for supply sag.
            if prev_supply_voltage.abs() > 0.0 {
                let scale = sagged_voltage / prev_supply_voltage;
                for stage in &mut self.stages {
                    if stage.vcc_injection_coeff != 0.0 {
                        stage.vcc_injection_coeff *= scale;
                    }
                }
            }
        }

        // Headroom models the supply rail ceiling.  In real circuits, gain
        // is set by resistor ratios (feedback/input) and is independent of
        // supply voltage.  What changes with voltage is how far the active
        // element can swing before hitting the rail — i.e. the clipping
        // ceiling.  So headroom only scales the soft limiter, not pre_gain
        // or output_gain.
        let headroom = self.supply_voltage.abs() / 9.0;

        // Apply pre-gain ONCE at the input.  This models the initial active
        // gain element (op-amp or transistor) that drives signal into the
        // first processing stage.
        let mut signal = input * self.pre_gain;

        #[cfg(feature = "debug-trace")]
        let trace_on = if input.abs() > 1e-10 {
            let n = TRACE_COUNT_PIPELINE.fetch_add(1, Ordering::Relaxed);
            if n < MAX_TRACE_PIPELINE {
                eprintln!(
                    "[PIPE n={n}] input={input:.6e} pre_gain={:.6} signal={signal:.6e} supply={:.1}V stages={} pp={}",
                    self.pre_gain, self.supply_voltage, self.stages.len(), self.push_pull_stages.len()
                );
                true
            } else {
                false
            }
        } else {
            false
        };

        // Apply input loading from upstream source impedance.
        // Models the voltage divider and frequency-dependent rolloff created by
        // the source impedance (guitar pickup, upstream pedal) driving this
        // circuit's input impedance.
        if let Some(ref mut loading) = self.input_loading {
            signal = loading.process(signal);
        }

        // Process all stages in topological (signal-flow) order.
        // WDF, multi-NL, and coupled BJT stages are interleaved by their
        // signal_flow_distance so earlier stages process first regardless of type.
        // WDF clipping stages get inter-stage re-amplification.
        //
        // Multi-NL stages use node-based routing: each stage reads from its
        // injection_node_id and writes to its output_node_id. This correctly
        // handles parallel-path topologies (e.g., 670 dual-channel sidechain)
        // where two stages at the same level should receive the same input,
        // not chain serially.
        let mut prev_was_clipping = false;
        let num_stages = self.stages.len();
        let mut stage_levels = [0.0f64; crate::metering::MAX_STAGES];
        let mut wdf_stage_counter = 0usize;

        // Node routing: trigger impulses were already pushed to node_signals above.

        for sr in &self.stage_order {
            match sr {
                StageRef::Wdf(i) => {
                    let stage = &mut self.stages[*i];

                    // Node-based routing for per-voice trigger stages:
                    // Only trigger voice stages read exclusively from node_signals.
                    // Regular stages (triodes, BJTs, etc.) use serial chain even
                    // if they have injection_node_id set for other purposes.
                    let stage_input = if stage.is_trigger_voice {
                        self.node_signals
                            .iter()
                            .rev()
                            .filter(|(nid, _)| *nid == stage.injection_node_id)
                            .map(|(_, v)| *v)
                            .sum::<f64>()
                    } else if stage.is_feedforward {
                        // Feedforward stages read from upstream node_signals
                        self.node_signals
                            .iter()
                            .rev()
                            .find(|(nid, _)| *nid == stage.injection_node_id)
                            .map(|(_, v)| *v)
                            .unwrap_or(0.0)
                    } else {
                        // Re-amplify only after the *previous* stage clipped.
                        if prev_was_clipping {
                            signal *= self.pre_gain;
                        }
                        signal
                    };
                    prev_was_clipping = !stage.is_feedforward && stage.root.is_clipping_stage();

                    if stage.has_paired_opamp() {
                        stage.set_paired_opamp_vp(stage_input);
                    }

                    #[cfg(feature = "debug-trace")]
                    let pre_stage = stage_input;
                    let stage_output = stage.process(stage_input);
                    // Guard against NaN from NR solver divergence.
                    let stage_output = if stage_output.is_finite() { stage_output } else { 0.0 };

                    // Write output to stage's output node for junction summing
                    // (only for trigger voice stages).
                    if stage.is_feedforward {
                        // Feedforward: additive blend into serial chain
                        signal += stage_output;
                    } else if stage.is_trigger_voice && stage.output_node_id != usize::MAX {
                        if let Some(entry) = self.node_signals.iter_mut()
                            .find(|(nid, _)| *nid == stage.output_node_id)
                        {
                            entry.1 += stage_output;
                        } else {
                            self.node_signals.push((stage.output_node_id, stage_output));
                        }
                        // Use accumulated value at output node as the serial chain signal.
                        signal = self.node_signals
                            .iter()
                            .rev()
                            .find(|(nid, _)| *nid == stage.output_node_id)
                            .map(|(_, v)| *v)
                            .unwrap_or(stage_output);
                    } else {
                        signal = stage_output;
                        // Write to node_signals for downstream feedforward stages
                        if stage.output_node_id != usize::MAX {
                            self.node_signals.push((stage.output_node_id, stage_output));
                        }
                    }

                    #[cfg(feature = "debug-trace")]
                    if trace_on {
                        eprintln!(
                            "  [WDF {wdf_stage_counter}] in={pre_stage:.6e} out={stage_output:.6e} rp={:.1}",
                            stage.tree.port_resistance()
                        );
                    }

                    if wdf_stage_counter < crate::metering::MAX_STAGES {
                        stage_levels[wdf_stage_counter] = stage_output;
                    }

                    #[cfg(debug_assertions)]
                    if let Some(ref stats) = self.debug_stats {
                        stats.record_stage_level(wdf_stage_counter, stage_output);
                    }
                    wdf_stage_counter += 1;
                }
                StageRef::MultiNl(i) => {
                    prev_was_clipping = false;

                    // Main-path multi-NL stages use the serial chain signal.
                    // Sidechain multi-NL stages live in self.sidechains (separate
                    // SidechainProcessor structs) and are NOT in stage_order,
                    // so all MultiNl refs here are main-path.
                    let mnl = &mut self.multi_nl_stages[*i];
                    let mnl_input = signal;

                    #[cfg(feature = "debug-trace")]
                    let pre = mnl_input;
                    let mnl_output = mnl.process(mnl_input);
                    // Guard against NaN from multi-NL NR solver divergence.
                    let mnl_output = if mnl_output.is_finite() { mnl_output } else { 0.0 };

                    // Store output at the stage's output node for downstream routing.
                    self.node_signals.push((mnl.output_node_id, mnl_output));
                    signal = mnl_output;

                    #[cfg(feature = "debug-trace")]
                    if trace_on {
                        let mnl = &self.multi_nl_stages[*i];
                        let devices: String = if let Some(ref dg) = mnl.device_groups {
                            dg.groups
                                .iter()
                                .map(|g| g.debug_name())
                                .collect::<Vec<_>>()
                                .join(",")
                        } else {
                            mnl.nl_devices
                                .iter()
                                .map(|d| d.debug_name())
                                .collect::<Vec<_>>()
                                .join(",")
                        };
                        eprintln!(
                            "  [MNL {i}] [{devices}] in={pre:.6e} out={signal:.6e} n_nl={} out_port={} xfmr={:.2} inj={} out={}",
                            mnl.n_nl, mnl.output_port, mnl.transformer_gain,
                            mnl.injection_node_id, mnl.output_node_id,
                        );
                        eprintln!(
                            "    s_nl_adapted={:.6?} v_prev={:.4?}",
                            &mnl.scattering.s_nl_adapted, &mnl.v_prev,
                        );
                    }
                }
            }
        }

        #[cfg(feature = "debug-trace")]
        if trace_on {
            eprintln!(
                "  [PRE-PP] signal={signal:.6e} n_pp={} n_sc={} stage_order={}(wdf={}, mnl={})",
                self.push_pull_stages.len(), self.sidechains.len(),
                self.stage_order.len(), self.stages.len(),
                self.multi_nl_stages.len()
            );
        }

        // Tick VCAs — apply envelope-gated amplitude modulation after WDF stages.
        for vca_binding in &mut self.vcas {
            // Gate envelope from trigger: only call gate_on() when the trigger
            // fires. Do NOT call gate_off() on subsequent samples — let the ADSR
            // run its full cycle (attack→decay→sustain). For percussive use
            // (sustain=0), the envelope naturally decays to zero.
            if let Some(trig_idx) = vca_binding.trigger_idx {
                if let Some(trig) = self.triggers.get(trig_idx) {
                    if trig.active_this_sample {
                        vca_binding.envelope.gate_on();
                    }
                }
            }
            let env_val = vca_binding.envelope.tick();
            vca_binding.vca.set_gain(env_val);

            // Read audio from input node (sum all signals at that node)
            let vca_input = self.node_signals.iter().rev()
                .filter(|(nid, _)| *nid == vca_binding.input_node_id)
                .map(|(_, v)| *v)
                .sum::<f64>();

            // If no node signal found, use the serial chain signal
            let vca_input = if vca_input == 0.0 && vca_binding.input_node_id == usize::MAX {
                signal
            } else {
                vca_input
            };

            let output = vca_binding.vca.process(vca_input);
            self.node_signals.push((vca_binding.output_node_id, output));

            // Feed VCA output into the serial signal chain
            signal = output;
        }

        // If VCOs exist but no VCA consumed their output, sum VCO contributions
        // directly into the signal chain. This handles VCO→out circuits without a VCA.
        if !self.vcos.is_empty() && self.vcas.is_empty() {
            for vco_binding in &self.vcos {
                if let Some((_, val)) = self.node_signals.iter().rev()
                    .find(|(nid, _)| *nid == vco_binding.output_node_id)
                {
                    signal += *val;
                }
            }
        }

        // Process through push-pull stages (differential tube amplifiers).
        // These model circuits like the Fairchild 670 where push and pull
        // triode halves process the signal simultaneously with opposite phase,
        // and the output is the differential plate voltage through a CT transformer.
        //
        // When sidechains are present, the CV from the previous sample modulates
        // the grid bias: Vgrid = base_bias - cv_delayed. This implements
        // variable-mu compression — higher CV = more negative bias = less gain.
        //
        // For stereo equipment (multiple identical push-pull stages), the stages
        // represent parallel channels (e.g., Fairchild 670 Lat/Vert). Only
        // process the first stage to avoid incorrect cascading.
        let pp_active = if self.push_pull_stages.len() > 1
            && self.push_pull_stages.windows(2).all(|w| {
                w[0].turns_ratio == w[1].turns_ratio && w[0].compensation == w[1].compensation
            }) {
            1
        } else {
            self.push_pull_stages.len()
        };

        if !self.sidechains.is_empty() {
            let total_cv: f64 = self.sidechains.iter().map(|sc| sc.cv_delayed).sum();
            for pp_stage in self.push_pull_stages[..pp_active].iter_mut() {
                pp_stage.grid_bias = self.base_grid_bias - total_cv;
                signal = pp_stage.process(signal);
            }
        } else {
            for pp_stage in self.push_pull_stages[..pp_active].iter_mut() {
                signal = pp_stage.process(signal);
            }
        }

        // Process sidechains: tap the audio output and compute CV for next sample.
        // The 1-sample delay is inherent and correct for discrete-time feedback.
        for (i, sc) in self.sidechains.iter_mut().enumerate() {
            let cv = sc.process(signal);
            #[cfg(feature = "debug-trace")]
            if trace_on {
                eprintln!(
                    "  [SC {i}] tap={signal:.6e} cv_out={cv:.6e} cv_prev={:.6e}",
                    sc.cv_delayed
                );
            }
            sc.cv_delayed = cv;
        }

        // Process through op-amp stages.
        // Each op-amp is modeled as a VCVS (voltage-controlled voltage source).
        // For unity-gain buffers (most common), the op-amp passes the signal
        // through with high input impedance and low output impedance.
        // The slew rate limiting is built into OpAmpRoot.
        for opamp_stage in &mut self.opamp_stages {
            // Set the input signal as Vp (non-inverting input)
            opamp_stage.opamp.set_vp(signal);

            // WDF formulation: incident wave a comes from the network.
            // For a unity-gain buffer where we want v_out ≈ vp:
            //   a = 2 * vp (the voltage wave from the input)
            //   b = 2 * v_out - a (reflected wave)
            //   v_out = (a + b) / 2
            //
            // By passing a = 2*signal, the solver has the correct reference
            // for the desired output and can properly compute the current
            // needed to drive any load (represented by Rp).
            let a = 2.0 * signal;
            let b = opamp_stage.opamp.process(a, 10_000.0);
            signal = (a + b) / 2.0; // Convert wave pair to voltage
        }

        // Apply slew rate limiting from op-amps.
        // Each op-amp in the circuit contributes its own slew rate limit.
        // This is the mechanism that makes the LM308 RAT sound different
        // from a TL072 RAT — the slow slew rate rounds off HF transients.
        for slew in &mut self.slew_limiters {
            signal = slew.process(signal);
        }

        // Guard against NaN/Inf from NL solver divergence — prevents
        // permanent state corruption in the oversampler's IIR filters.
        if !signal.is_finite() {
            signal = 0.0;
        }
        signal = self.rail_sat_oversampler.process(signal, |s| {
            self.rail_saturation.process(s, headroom)
        });

        // Process through BBD delay lines (wet signal mixed with dry).
        for bbd in &mut self.bbds {
            let wet = bbd.process(signal);
            let mix = self.bbd_wet_mix;
            signal = signal * (1.0 - mix) + wet * mix;
        }

        // Process through generic delay lines.
        // Each delay line writes the current signal, then reads from all taps.
        // Tap outputs are mixed equally and blended with the dry signal.
        for dl_binding in &mut self.delay_lines {
            // Reset speed modulation for this sample (LFOs will have already added offsets)
            dl_binding.delay_line.write(signal);

            // Read all taps and sum
            let num_taps = dl_binding.taps.len();
            if num_taps > 0 {
                let mut wet = 0.0;
                for (tap_idx, &ratio) in dl_binding.taps.iter().enumerate() {
                    wet += dl_binding.delay_line.read_at_ratio(ratio, tap_idx);
                }
                wet /= num_taps as f64;

                // Mix wet/dry equally (same convention as BBD)
                signal = signal * 0.5 + wet * 0.5;

                // Route the wet signal back as feedback
                dl_binding.delay_line.set_feedback_sample(wet);
            }

            // Reset speed mod accumulator for next sample
            dl_binding.delay_line.reset_speed_mod();
        }

        // Apply output loading from downstream load impedance.
        // Models the voltage divider and frequency-dependent rolloff created by
        // this circuit's output impedance driving the load (downstream pedal,
        // amp input).
        if let Some(ref mut loading) = self.output_loading {
            signal = loading.process(signal);
        }

        // Final NaN guard — prevent corrupted audio from reaching the output.
        if !signal.is_finite() {
            signal = 0.0;
        }
        let output = signal * self.output_gain;

        #[cfg(feature = "debug-trace")]
        if trace_on {
            eprintln!("  [OUT] output={output:.6e}");
        }

        // Record output for debug stats
        #[cfg(debug_assertions)]
        if let Some(ref stats) = self.debug_stats {
            stats.record_output(output);
        }

        // ═══════════════════════════════════════════════════════════════════════
        // Metering: accumulate per-sample, reduce per-block, write to ring buffer
        // ═══════════════════════════════════════════════════════════════════════
        if let Some(ref mut acc) = self.metrics_accumulator {
            // Accumulate input/output levels
            acc.accumulate_levels(input, output);

            // Accumulate per-stage signal levels for circuit view visualization
            for i in 0..num_stages.min(crate::metering::MAX_STAGES) {
                acc.accumulate_stage(i, stage_levels[i]);
            }

            // Record tube state from WDF stages (plate current for glow shaders)
            let mut tube_idx = 0;
            for stage in &self.stages {
                match &stage.root {
                    RootKind::Triode(t) => {
                        // Get plate voltage from last WDF wave (approximate)
                        // For more accurate values, we'd need to track v_pk from process()
                        let vpk = (self.supply_voltage * 0.6) as f32; // Typical idle point
                        let ip_ma = (t.plate_current(vpk as f64) * 1000.0) as f32;
                        acc.record_tube(tube_idx, ip_ma, vpk);
                        tube_idx += 1;
                    }
                    RootKind::Pentode(p) => {
                        let vpk = (self.supply_voltage * 0.6) as f32;
                        let ip_ma = (p.plate_current(vpk as f64) * 1000.0) as f32;
                        acc.record_tube(tube_idx, ip_ma, vpk);
                        tube_idx += 1;
                    }
                    _ => {}
                }
            }
            // Record push-pull triode tubes
            for pp in &self.push_pull_stages {
                let vpk = (self.supply_voltage * 0.6) as f32;
                let ip_push = (pp.push_root.plate_current(vpk as f64) * 1000.0) as f32;
                acc.record_tube(tube_idx, ip_push, vpk);
                tube_idx += 1;
                let ip_pull = (pp.pull_root.plate_current(vpk as f64) * 1000.0) as f32;
                acc.record_tube(tube_idx, ip_pull, vpk);
                tube_idx += 1;
            }

            // Record LFO phase (first LFO for tremolo indicator)
            if let Some(binding) = self.lfos.first() {
                acc.record_lfo_phase(binding.lfo.phase() as f32);
            }

            // Record supply voltage (for sag visualization when we add dynamic sag)
            acc.record_supply(self.supply_voltage as f32);

            // Check if block is complete — reduce and write to ring buffer
            if acc.is_block_complete() {
                let metrics = acc.reduce();
                if let Some(ref buffer) = self.metrics_buffer {
                    buffer.write(metrics);
                }
            }
        }

        output
    }

    fn set_sample_rate(&mut self, rate: f64) {
        self.sample_rate = rate;
        for stage in &mut self.stages {
            stage.tree.update_sample_rate(rate);
            stage.tree.recompute();
        }
        for binding in &mut self.lfos {
            binding.lfo.set_sample_rate(rate);
        }
        for binding in &mut self.envelopes {
            binding.envelope.set_sample_rate(rate);
        }
        for slew in &mut self.slew_limiters {
            slew.set_sample_rate(rate);
        }
        for bbd in &mut self.bbds {
            bbd.set_sample_rate(rate);
        }
        for dl_binding in &mut self.delay_lines {
            dl_binding.delay_line.set_sample_rate(rate);
        }
        if let Some(ref mut thermal) = self.thermal {
            thermal.set_sample_rate(rate);
        }
        for opamp_stage in &mut self.opamp_stages {
            opamp_stage.opamp.set_sample_rate(rate);
        }
        if let Some(ref mut psu) = self.power_supply {
            psu.set_sample_rate(rate);
        }
        for smoother in &mut self.pot_smoothers {
            smoother.set_sample_rate(rate);
        }
    }

    fn reset(&mut self) {
        for stage in &mut self.stages {
            stage.reset();
        }
        for mnl in &mut self.multi_nl_stages {
            mnl.reset();
        }
        for binding in &mut self.lfos {
            binding.lfo.reset();
        }
        for binding in &mut self.envelopes {
            binding.envelope.reset();
        }
        for slew in &mut self.slew_limiters {
            slew.reset();
        }
        for bbd in &mut self.bbds {
            bbd.reset();
        }
        for dl_binding in &mut self.delay_lines {
            dl_binding.delay_line.reset();
        }
        for opamp_stage in &mut self.opamp_stages {
            opamp_stage.opamp.reset();
        }
        if let Some(ref mut thermal) = self.thermal {
            thermal.reset();
        }
        if let Some(ref mut psu) = self.power_supply {
            psu.reset();
        }
        for sc in &mut self.sidechains {
            sc.reset();
        }
        self.rail_sat_oversampler.reset();
    }

    fn set_control(&mut self, label: &str, value: f64) {
        self.set_control(label, value);
    }

    fn set_supply_voltage(&mut self, voltage: f64) {
        self.set_supply_voltage(voltage);
    }

    fn list_editable_components(&self) -> Vec<(String, &'static str, f64)> {
        self.list_editable_components()
    }

    fn set_passive(&mut self, comp_id: &str, value: f64) -> bool {
        self.set_passive(comp_id, value)
    }

    fn reset_passive(&mut self, comp_id: &str) -> bool {
        self.reset_passive(comp_id)
    }
}
