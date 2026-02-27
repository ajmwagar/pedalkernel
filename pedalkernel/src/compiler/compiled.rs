//! Compiled pedal processor: the runtime audio processing chain.

use crate::elements::*;
use crate::metering::{MetricsAccumulator, MetricsRingBuffer, UiMetrics};
use crate::oversampling::OversamplingFactor;
use crate::thermal::ThermalModel;
use crate::PedalProcessor;
use std::sync::Arc;

use super::stage::{RootKind, WdfStage};

// ═══════════════════════════════════════════════════════════════════════════
// Compiled pedal
// ═══════════════════════════════════════════════════════════════════════════

/// Control binding: maps a knob label to a parameter in the processing chain.
pub(super) struct ControlBinding {
    pub(super) label: String,
    pub(super) target: ControlTarget,
    pub(super) component_id: String,
    #[allow(dead_code)]
    pub(super) max_resistance: f64,
}

#[derive(Debug)]
pub(super) enum ControlTarget {
    /// Modify the pre-gain multiplier (gain/drive/fuzz/distortion/sustain).
    PreGain,
    /// Modify output attenuation (level/volume/output).
    OutputGain,
    /// Modify a pot in a specific WDF stage.
    PotInStage(usize),
    /// Modify an LFO's rate (index into lfos vector).
    LfoRate(usize),
    /// Modify an LFO's depth/amplitude (index into lfos vector).
    LfoDepth(usize),
    /// Modify a delay line's delay time (normalized 0–1).
    DelayTime(usize),
    /// Modify a delay line's feedback amount (0–1).
    DelayFeedback(usize),
    /// Modify a switch position for fork() routing.
    /// Contains: (switch_id, num_positions)
    SwitchPosition { switch_id: String, num_positions: usize },
    /// Modify op-amp feedback gain (pot in Rf path).
    /// For series topology: Rf = fixed_series_r + pot_r
    /// For parallel topology: Rf = parallel_fixed_r || (fixed_series_r + pot_r)
    OpAmpGain {
        stage_idx: usize,
        ri: f64,
        fixed_series_r: f64,
        max_pot_r: f64,
        /// If Some, pot is in parallel with a fixed resistance path
        parallel_fixed_r: Option<f64>,
        is_inverting: bool,
    },
}

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
    /// Modulate a MOSFET's Vgs.
    MosfetVgs { stage_idx: usize },
    /// Modulate an OTA's bias current (for VCA/compressor).
    OtaIabc { stage_idx: usize },
    /// Modulate a BBD's clock frequency (for chorus/flanger).
    BbdClock { bbd_idx: usize },
    /// Modulate an op-amp's non-inverting input voltage.
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
    pub(super) pre_gain: f64,
    pub(super) output_gain: f64,
    pub(super) rail_saturation: RailSaturation,
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
}

/// Gain-like control labels.
pub(super) fn is_gain_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "drive" | "gain" | "fuzz" | "sustain" | "distortion" | "sensitivity"
    )
}

/// Level-like control labels.
pub(super) fn is_level_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "level" | "volume" | "output"
    )
}

/// Rate-like control labels (for LFO).
pub(super) fn is_rate_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "rate" | "speed" | "tempo"
    )
}

/// Depth-like control labels (for LFO modulation amount).
pub(super) fn is_depth_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "depth" | "width"
    )
}

/// Delay time control labels.
pub(super) fn is_delay_time_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "time" | "delay" | "repeat_rate" | "echo"
    )
}

/// Delay feedback/intensity control labels.
pub(super) fn is_delay_feedback_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "feedback" | "repeats" | "intensity" | "regeneration"
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
        self.supply_voltage = voltage.clamp(5.0, 500.0); // Allow up to 500V for tube gear

        // Calculate op-amp v_max from supply voltage.
        // For single-supply biased at Vsupply/2: v_max = (Vsupply/2) - saturation_margin
        // Saturation margin is typically 1.0-1.5V for most op-amps.
        // For high voltages (tube gear), the margin stays ~1.5V.
        let saturation_margin = 1.5;
        let opamp_v_max = (voltage / 2.0 - saturation_margin).max(1.0);

        // For tubes and transistors, v_max is the full supply voltage
        // (plate/collector can swing from 0V to B+/Vcc)
        let tube_v_max = voltage;
        let bjt_v_max = voltage;

        // Propagate v_max to standalone op-amp stages
        for stage in &mut self.opamp_stages {
            stage.opamp.set_v_max(opamp_v_max);
        }

        // Propagate v_max to all nonlinear roots in WDF stages
        for stage in &mut self.stages {
            match &mut stage.root {
                RootKind::OpAmp(op) => op.set_v_max(opamp_v_max),
                RootKind::Triode(t) => t.set_v_max(tube_v_max),
                RootKind::Pentode(p) => p.set_v_max(tube_v_max),
                RootKind::BjtNpn(bjt) => bjt.set_v_max(bjt_v_max),
                RootKind::BjtPnp(bjt) => bjt.set_v_max(bjt_v_max),
                // Diodes, JFETs, MOSFETs, OTAs, Zeners don't need supply voltage
                // (their behavior is determined by their intrinsic parameters)
                _ => {}
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
        (self.stages.len() as u32, self.opamp_stages.len() as u32)
    }

    /// Get the supply voltage.
    pub fn supply_voltage(&self) -> f64 {
        self.supply_voltage
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

    /// Set a control by its label (e.g., "Drive", "Level", "Rate").
    pub fn set_control(&mut self, label: &str, value: f64) {
        let value = value.clamp(0.0, 1.0);
        for i in 0..self.controls.len() {
            if !self.controls[i].label.eq_ignore_ascii_case(label) {
                continue;
            }
            match &self.controls[i].target {
                ControlTarget::PreGain => {
                    let (lo, hi) = self.gain_range;
                    self.pre_gain = lo * (hi / lo).powf(value);
                }
                ControlTarget::OutputGain => {
                    self.output_gain = value;
                }
                ControlTarget::PotInStage(stage_idx) => {
                    let stage_idx = *stage_idx;
                    let comp_id = self.controls[i].component_id.clone();
                    if let Some(stage) = self.stages.get_mut(stage_idx) {
                        stage.tree.set_pot(&comp_id, value);
                        // For 3-terminal pots: update synthetic halves with
                        // inverse coupling. No-ops for 2-terminal pots.
                        stage.tree.set_pot(&format!("{comp_id}__aw"), value);
                        stage.tree.set_pot(&format!("{comp_id}__wb"), 1.0 - value);
                        stage.tree.recompute();
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
                ControlTarget::SwitchPosition { switch_id, num_positions } => {
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
                ControlTarget::OpAmpGain {
                    stage_idx,
                    ri,
                    fixed_series_r,
                    max_pot_r,
                    parallel_fixed_r,
                    is_inverting,
                } => {
                    // Calculate pot resistance from position
                    let pot_r = value * *max_pot_r;
                    // Calculate pot path resistance (series with any fixed resistors in that path)
                    let pot_path_r = *fixed_series_r + pot_r;

                    // Real pots have minimum wiper-to-end resistance of ~100-1000 ohms
                    // even at the "0" position, due to contact resistance and track design.
                    // Use 500 ohms as typical minimum for audio pots.
                    let min_pot_r = 500.0;
                    let effective_pot_path_r = pot_path_r.max(min_pot_r);

                    // Calculate effective Rf based on topology
                    let rf = if let Some(parallel_r) = parallel_fixed_r {
                        // Parallel topology: Rf = parallel_fixed_r || pot_path_r
                        // Formula: 1/Rf = 1/R1 + 1/R2, so Rf = (R1*R2)/(R1+R2)
                        // When pot_path_r → min, Rf → parallel_r || min
                        // When pot_path_r → ∞, Rf → parallel_r
                        (parallel_r * effective_pot_path_r) / (parallel_r + effective_pot_path_r)
                    } else {
                        // Series topology: Rf = fixed_series_r + pot_r
                        effective_pot_path_r
                    };

                    // Calculate gain based on op-amp configuration
                    // - Inverting: gain = Rf/Ri
                    // - Non-inverting: gain = 1 + Rf/Ri
                    let gain = if *is_inverting {
                        rf / *ri
                    } else {
                        1.0 + (rf / *ri)
                    };

                    let stage_idx = *stage_idx;
                    if let Some(stage) = self.stages.get_mut(stage_idx) {
                        stage.set_opamp_gain(gain);
                    }
                }
            }
            return;
        }
    }
}

impl PedalProcessor for CompiledPedal {
    fn process(&mut self, input: f64) -> f64 {
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
                        if matches!(&stage.root, RootKind::Jfet(_)) {
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
                        dl.delay_line.set_delay_normalized(modulation.clamp(0.0, 1.0));
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
                        if matches!(&stage.root, RootKind::Jfet(_)) {
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
                        dl.delay_line.set_delay_normalized(modulation.clamp(0.0, 1.0));
                    }
                }
            }
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
            self.supply_voltage = sagged_voltage;
            // Update v_max for all root elements
            let saturation_margin = 1.5;
            let opamp_v_max = (sagged_voltage / 2.0 - saturation_margin).max(1.0);
            let tube_v_max = sagged_voltage;
            let bjt_v_max = sagged_voltage;
            for stage in &mut self.opamp_stages {
                stage.opamp.set_v_max(opamp_v_max);
            }
            for stage in &mut self.stages {
                match &mut stage.root {
                    RootKind::OpAmp(op) => op.set_v_max(opamp_v_max),
                    RootKind::Triode(t) => t.set_v_max(tube_v_max),
                    RootKind::Pentode(p) => p.set_v_max(tube_v_max),
                    RootKind::BjtNpn(bjt) => bjt.set_v_max(bjt_v_max),
                    RootKind::BjtPnp(bjt) => bjt.set_v_max(bjt_v_max),
                    _ => {}
                }
            }
        }

        // Headroom models the supply rail ceiling.  In real circuits, gain
        // is set by resistor ratios (feedback/input) and is independent of
        // supply voltage.  What changes with voltage is how far the active
        // element can swing before hitting the rail — i.e. the clipping
        // ceiling.  So headroom only scales the soft limiter, not pre_gain
        // or output_gain.
        let headroom = self.supply_voltage / 9.0;

        // Apply pre-gain ONCE at the input.  This models the initial active
        // gain element (op-amp or transistor) that drives signal into the
        // first processing stage.
        let mut signal = input * self.pre_gain;

        // Process through WDF stages.  Only diode-clipping stages get
        // inter-stage re-amplification: clipping squashes the signal to the
        // diode forward voltage (~0.3–0.7 V), so each subsequent clipper
        // needs the signal re-driven.  Non-clipping stages (JFETs acting as
        // variable resistors in phasers, op-amp buffers, BJT gain stages)
        // pass the signal at roughly the same amplitude; re-applying
        // pre_gain before them causes exponential level growth
        // (pre_gain^N for N stages).
        let mut prev_was_clipping = false;
        for (stage_idx, stage) in self.stages.iter_mut().enumerate() {
            // Re-amplify only after the *previous* stage clipped.
            if prev_was_clipping {
                signal *= self.pre_gain;
            }
            prev_was_clipping = stage.root.is_clipping_stage();

            // For stages with a paired op-amp buffer (all-pass circuits),
            // set the op-amp's Vp to the stage input BEFORE processing.
            // The op-amp buffer models the unity-gain amplifier that
            // compensates for passive R/C network attenuation in the WDF tree.
            if stage.has_paired_opamp() {
                stage.set_paired_opamp_vp(signal);
            }

            signal = stage.process(signal);
            // Record per-stage level for debug
            #[cfg(debug_assertions)]
            if let Some(ref stats) = self.debug_stats {
                stats.record_stage_level(stage_idx, signal);
            }
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

        signal = self.rail_saturation.process(signal, headroom);

        // Process through BBD delay lines (wet signal mixed with dry).
        for bbd in &mut self.bbds {
            let wet = bbd.process(signal);
            signal = signal * 0.5 + wet * 0.5;
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

        let output = signal * self.output_gain;

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
    }

    fn reset(&mut self) {
        for stage in &mut self.stages {
            stage.reset();
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
    }

    fn set_control(&mut self, label: &str, value: f64) {
        self.set_control(label, value);
    }

    fn set_supply_voltage(&mut self, voltage: f64) {
        self.set_supply_voltage(voltage);
    }
}

