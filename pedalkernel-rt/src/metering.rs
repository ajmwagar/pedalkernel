//! Lock-free metering system for real-time UI visualization.
//!
//! The audio thread computes reduced metrics every N samples and writes them
//! to a lock-free ring buffer. The UI thread reads the latest complete frame
//! at 60fps and applies visual ballistics (smoothing) before uploading to GPU.

use crate::math;
use alloc::boxed::Box;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicUsize, Ordering};

/// Maximum number of tube stages that can be monitored.
pub const MAX_TUBES: usize = 12;

/// Maximum number of transformer stages that can be monitored.
pub const MAX_TRANSFORMERS: usize = 4;

/// Maximum number of WDF stages for signal flow visualization.
pub const MAX_STAGES: usize = 16;

/// Metrics exported from the audio thread for UI visualization.
///
/// This struct is written by the audio thread every `block_size` samples
/// (typically 128 or 256) and read by the UI thread at ~60fps.
#[derive(Debug, Clone, Copy, Default)]
#[repr(C)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct UiMetrics {
    // ═══════════════════════════════════════════════════════════════════════════
    // Level meters — for VU meters, LED bars, clip indicators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Input RMS level (dB, typically -60 to +6).
    pub input_rms_db: f32,
    /// Output RMS level (dB).
    pub output_rms_db: f32,
    /// Input peak level (dB) — instant attack, slow decay in accumulator.
    pub input_peak_db: f32,
    /// Output peak level (dB).
    pub output_peak_db: f32,

    // ═══════════════════════════════════════════════════════════════════════════
    // Compressor/limiter meters — for LA-2A, 1176, Fairchild GR display
    // ═══════════════════════════════════════════════════════════════════════════
    /// Gain reduction in dB (0 = no reduction, negative = compression).
    pub gain_reduction_db: f32,

    // ═══════════════════════════════════════════════════════════════════════════
    // Tube state — for glow shaders
    // ═══════════════════════════════════════════════════════════════════════════
    /// Per-tube plate current (mA). Higher current = brighter glow.
    pub tube_plate_current: [f32; MAX_TUBES],

    /// Per-tube plate dissipation (watts = Vpk * Ip).
    pub tube_dissipation: [f32; MAX_TUBES],

    /// Number of active tubes in this circuit.
    pub tube_count: u8,

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformer state — for saturation visualization
    // ═══════════════════════════════════════════════════════════════════════════
    /// Core flux as fraction of saturation (0.0 = idle, 1.0 = saturating).
    pub transformer_flux: [f32; MAX_TRANSFORMERS],

    /// Number of active transformers in this circuit.
    pub transformer_count: u8,

    // ═══════════════════════════════════════════════════════════════════════════
    // Signal envelope — for waveform displays
    // ═══════════════════════════════════════════════════════════════════════════
    /// Minimum sample value in this block (for envelope display).
    pub signal_min: f32,
    /// Maximum sample value in this block (for envelope display).
    pub signal_max: f32,

    // ═══════════════════════════════════════════════════════════════════════════
    // Control signals — for LFO/tremolo indicators
    // ═══════════════════════════════════════════════════════════════════════════
    /// LFO phase (0.0-1.0) for tremolo/vibrato pulse indicator.
    pub lfo_phase: f32,

    // ═══════════════════════════════════════════════════════════════════════════
    // Power supply — for sag visualization
    // ═══════════════════════════════════════════════════════════════════════════
    /// Current B+ voltage. Drops under load (tube sag).
    pub supply_voltage: f32,

    /// Supply sag as fraction (0.0 = full voltage, 1.0 = max sag).
    pub supply_sag: f32,

    // ═══════════════════════════════════════════════════════════════════════════
    // Per-stage metrics — for signal flow visualization
    // ═══════════════════════════════════════════════════════════════════════════
    /// RMS level at each circuit stage (normalized 0-1 for shader).
    pub stage_levels: [f32; MAX_STAGES],

    /// Number of active stages.
    pub stage_count: u8,

    // ═══════════════════════════════════════════════════════════════════════════
    // Nonlinear solver diagnostics — for compiler/runtime regression harnesses
    // ═══════════════════════════════════════════════════════════════════════════
    /// Total nonlinear/coupled solve calls observed in this metrics block.
    pub nr_solve_count: u32,
    /// Total Newton/fixed-point iterations across solve calls in this block.
    pub nr_total_iterations: u32,
    /// Maximum iterations used by any solve in this block.
    pub nr_max_iterations: u16,
    /// Number of solves that did not converge in this block.
    pub nr_nonconverged_count: u16,
    /// Maximum final residual observed in this block.
    pub nr_max_residual: f32,
    /// Per-stage maximum iterations observed in this block.
    pub stage_nr_iterations: [u16; MAX_STAGES],
    /// Per-stage maximum final residual observed in this block.
    pub stage_nr_residual: [f32; MAX_STAGES],
    /// Per-stage non-convergence counts, saturated at u8::MAX.
    pub stage_nr_nonconverged: [u8; MAX_STAGES],
    /// Non-finite inputs, intermediate values, or outputs observed this block.
    pub nonfinite_count: u32,
    /// Samples whose magnitude exceeded a conservative blowup threshold.
    pub blowup_count: u32,
    /// Largest absolute signal magnitude observed this block.
    pub max_abs_sample: f32,

    // ═══════════════════════════════════════════════════════════════════════════
    // Timing
    // ═══════════════════════════════════════════════════════════════════════════
    /// Block counter (monotonic, wraps at u32::MAX).
    pub block_counter: u32,
}

impl UiMetrics {
    /// Create zeroed metrics.
    pub fn new() -> Self {
        Self::default()
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Metrics Accumulator (Audio Thread)
// ═══════════════════════════════════════════════════════════════════════════════

/// Accumulates per-sample statistics and reduces them to `UiMetrics` every block.
///
/// Used by the audio thread to compute RMS, peak, and tap tube/transformer state.
/// Call `accumulate()` every sample, then `reduce()` every `block_size` samples.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MetricsAccumulator {
    /// Block size for metric reduction (typically 128 or 256).
    block_size: usize,
    /// Current sample count within the block.
    sample_count: usize,
    /// Block counter for timing.
    block_counter: u32,

    // Running accumulators
    input_sum_sq: crate::Wave,
    output_sum_sq: crate::Wave,
    input_peak: crate::Wave,
    output_peak: crate::Wave,
    signal_min: crate::Wave,
    signal_max: crate::Wave,

    // Per-stage level accumulators
    stage_sum_sq: [crate::Wave; MAX_STAGES],
    stage_count: usize,

    // Solver diagnostics
    nr_solve_count: u32,
    nr_total_iterations: u32,
    nr_max_iterations: u16,
    nr_nonconverged_count: u16,
    nr_max_residual: f32,
    stage_nr_iterations: [u16; MAX_STAGES],
    stage_nr_residual: [f32; MAX_STAGES],
    stage_nr_nonconverged: [u8; MAX_STAGES],
    nonfinite_count: u32,
    blowup_count: u32,
    max_abs_sample: f32,

    // Tube state (sampled, not accumulated)
    tube_plate_current: [f32; MAX_TUBES],
    tube_dissipation: [f32; MAX_TUBES],
    tube_count: usize,

    // Supply voltage (sampled)
    supply_voltage: f32,
    nominal_supply: f32,

    // LFO phase (sampled)
    lfo_phase: f32,

    // Gain reduction (sampled from compressor)
    gain_reduction_db: f32,
}

impl MetricsAccumulator {
    /// Create a new accumulator with the given block size.
    pub fn new(block_size: usize) -> Self {
        Self {
            block_size,
            sample_count: 0,
            block_counter: 0,
            input_sum_sq: 0.0,
            output_sum_sq: 0.0,
            input_peak: 0.0,
            output_peak: 0.0,
            signal_min: 0.0,
            signal_max: 0.0,
            stage_sum_sq: [0.0; MAX_STAGES],
            stage_count: 0,
            nr_solve_count: 0,
            nr_total_iterations: 0,
            nr_max_iterations: 0,
            nr_nonconverged_count: 0,
            nr_max_residual: 0.0,
            stage_nr_iterations: [0; MAX_STAGES],
            stage_nr_residual: [0.0; MAX_STAGES],
            stage_nr_nonconverged: [0; MAX_STAGES],
            nonfinite_count: 0,
            blowup_count: 0,
            max_abs_sample: 0.0,
            tube_plate_current: [0.0; MAX_TUBES],
            tube_dissipation: [0.0; MAX_TUBES],
            tube_count: 0,
            supply_voltage: 9.0,
            nominal_supply: 9.0,
            lfo_phase: 0.0,
            gain_reduction_db: 0.0,
        }
    }

    /// Set the nominal supply voltage for sag calculation.
    #[inline]
    pub fn set_nominal_supply(&mut self, voltage: f32) {
        self.nominal_supply = voltage;
    }

    /// Accumulate input/output levels for one sample.
    #[inline]
    pub fn accumulate_levels(&mut self, input: crate::Wave, output: crate::Wave) {
        if !input.is_finite() || !output.is_finite() {
            self.nonfinite_count = self.nonfinite_count.saturating_add(1);
        }
        let input = if input.is_finite() { input } else { 0.0 };
        let output = if output.is_finite() { output } else { 0.0 };
        self.input_sum_sq += input * input;
        self.output_sum_sq += output * output;

        let abs_in = input.abs();
        let abs_out = output.abs();
        let max_abs = abs_in.max(abs_out);
        self.max_abs_sample = self
            .max_abs_sample
            .max(max_abs.min(f32::MAX as crate::Wave) as f32);
        if max_abs > 32.0 {
            self.blowup_count = self.blowup_count.saturating_add(1);
        }
        if abs_in > self.input_peak {
            self.input_peak = abs_in;
        }
        if abs_out > self.output_peak {
            self.output_peak = abs_out;
        }

        if output < self.signal_min {
            self.signal_min = output;
        }
        if output > self.signal_max {
            self.signal_max = output;
        }

        self.sample_count += 1;
    }

    /// Record a stage level (call after each WDF stage processes).
    #[inline]
    pub fn accumulate_stage(&mut self, stage_idx: usize, level: crate::Wave) {
        if stage_idx < MAX_STAGES {
            self.stage_sum_sq[stage_idx] += level * level;
            if stage_idx >= self.stage_count {
                self.stage_count = stage_idx + 1;
            }
        }
    }

    /// Record one nonlinear/coupled solver call for a stage.
    #[inline]
    pub fn record_stage_solver(
        &mut self,
        stage_idx: usize,
        iterations: u32,
        residual: crate::Wave,
        converged: bool,
    ) {
        self.nr_solve_count = self.nr_solve_count.saturating_add(1);
        self.nr_total_iterations = self.nr_total_iterations.saturating_add(iterations);
        self.nr_max_iterations = self
            .nr_max_iterations
            .max(iterations.min(u16::MAX as u32) as u16);
        if !converged {
            self.nr_nonconverged_count = self.nr_nonconverged_count.saturating_add(1);
        }
        if residual.is_finite() {
            self.nr_max_residual = self.nr_max_residual.max(residual as f32);
        }

        if stage_idx < MAX_STAGES {
            self.stage_nr_iterations[stage_idx] =
                self.stage_nr_iterations[stage_idx].max(iterations.min(u16::MAX as u32) as u16);
            if residual.is_finite() {
                self.stage_nr_residual[stage_idx] =
                    self.stage_nr_residual[stage_idx].max(residual as f32);
            }
            if !converged {
                self.stage_nr_nonconverged[stage_idx] =
                    self.stage_nr_nonconverged[stage_idx].saturating_add(1);
            }
            if stage_idx >= self.stage_count {
                self.stage_count = stage_idx + 1;
            }
        }
    }

    /// Record a runtime signal fault caught outside normal level metering.
    #[inline]
    pub fn record_signal_fault(&mut self, value: crate::Wave) {
        if !value.is_finite() {
            self.nonfinite_count = self.nonfinite_count.saturating_add(1);
            return;
        }
        let abs = math::abs(value);
        self.max_abs_sample = self
            .max_abs_sample
            .max(abs.min(f32::MAX as crate::Wave) as f32);
        if abs > 32.0 {
            self.blowup_count = self.blowup_count.saturating_add(1);
        }
    }

    /// Record tube state (call once per sample or per block).
    #[inline]
    pub fn record_tube(&mut self, tube_idx: usize, plate_current_ma: f32, vpk: f32) {
        if tube_idx < MAX_TUBES {
            self.tube_plate_current[tube_idx] = plate_current_ma;
            self.tube_dissipation[tube_idx] = vpk * plate_current_ma / 1000.0;
            if tube_idx >= self.tube_count {
                self.tube_count = tube_idx + 1;
            }
        }
    }

    /// Record supply voltage (for sag visualization).
    #[inline]
    pub fn record_supply(&mut self, voltage: f32) {
        self.supply_voltage = voltage;
    }

    /// Record LFO phase (for pulse indicator).
    #[inline]
    pub fn record_lfo_phase(&mut self, phase: f32) {
        self.lfo_phase = phase;
    }

    /// Record gain reduction (for compressor meters).
    #[inline]
    pub fn record_gain_reduction(&mut self, gr_db: f32) {
        self.gain_reduction_db = gr_db;
    }

    /// Check if a block is complete and ready for reduction.
    #[inline]
    pub fn is_block_complete(&self) -> bool {
        self.sample_count >= self.block_size
    }

    /// Reduce accumulated statistics to `UiMetrics` and reset for next block.
    ///
    /// Call this every `block_size` samples. Returns the reduced metrics.
    pub fn reduce(&mut self) -> UiMetrics {
        let n = self.sample_count.max(1) as crate::Wave;

        // RMS to dB (reference: 1.0 = 0 dB)
        let input_rms = math::sqrt(self.input_sum_sq / n);
        let output_rms = math::sqrt(self.output_sum_sq / n);

        let to_db = |x: crate::Wave| -> f32 {
            if x > 1e-10 {
                (20.0 * math::log10(x)) as f32
            } else {
                -120.0
            }
        };

        // Compute per-stage RMS levels (normalized to 0-1 for shader)
        let mut stage_levels = [0.0f32; MAX_STAGES];
        for i in 0..self.stage_count {
            let rms = math::sqrt(self.stage_sum_sq[i] / n);
            // Normalize: assume +/-1.0 is full scale, RMS of 0.7 = full
            stage_levels[i] = ((rms / 0.7) as f32).min(1.0);
        }

        // Supply sag: fraction of voltage drop from nominal
        let supply_sag = if self.nominal_supply > 0.0 {
            ((self.nominal_supply - self.supply_voltage) / self.nominal_supply).clamp(0.0, 1.0)
        } else {
            0.0
        };

        let metrics = UiMetrics {
            input_rms_db: to_db(input_rms),
            output_rms_db: to_db(output_rms),
            input_peak_db: to_db(self.input_peak),
            output_peak_db: to_db(self.output_peak),
            gain_reduction_db: self.gain_reduction_db,
            tube_plate_current: self.tube_plate_current,
            tube_dissipation: self.tube_dissipation,
            tube_count: self.tube_count as u8,
            transformer_flux: [0.0; MAX_TRANSFORMERS], // TODO: implement
            transformer_count: 0,
            signal_min: self.signal_min as f32,
            signal_max: self.signal_max as f32,
            lfo_phase: self.lfo_phase,
            supply_voltage: self.supply_voltage,
            supply_sag,
            stage_levels,
            stage_count: self.stage_count as u8,
            nr_solve_count: self.nr_solve_count,
            nr_total_iterations: self.nr_total_iterations,
            nr_max_iterations: self.nr_max_iterations,
            nr_nonconverged_count: self.nr_nonconverged_count,
            nr_max_residual: self.nr_max_residual,
            stage_nr_iterations: self.stage_nr_iterations,
            stage_nr_residual: self.stage_nr_residual,
            stage_nr_nonconverged: self.stage_nr_nonconverged,
            nonfinite_count: self.nonfinite_count,
            blowup_count: self.blowup_count,
            max_abs_sample: self.max_abs_sample,
            block_counter: self.block_counter,
        };

        // Reset accumulators
        self.sample_count = 0;
        self.block_counter = self.block_counter.wrapping_add(1);
        self.input_sum_sq = 0.0;
        self.output_sum_sq = 0.0;
        self.input_peak *= 0.9995;
        self.output_peak *= 0.9995;
        self.signal_min = 0.0;
        self.signal_max = 0.0;
        for i in 0..MAX_STAGES {
            self.stage_sum_sq[i] = 0.0;
        }
        self.nr_solve_count = 0;
        self.nr_total_iterations = 0;
        self.nr_max_iterations = 0;
        self.nr_nonconverged_count = 0;
        self.nr_max_residual = 0.0;
        self.stage_nr_iterations = [0; MAX_STAGES];
        self.stage_nr_residual = [0.0; MAX_STAGES];
        self.stage_nr_nonconverged = [0; MAX_STAGES];
        self.nonfinite_count = 0;
        self.blowup_count = 0;
        self.max_abs_sample = 0.0;

        metrics
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Lock-free SPSC Ring Buffer
// ═══════════════════════════════════════════════════════════════════════════════

/// Single-producer single-consumer lock-free ring buffer for `UiMetrics`.
///
/// The audio thread writes metrics every ~128 samples (~375 writes/sec at 48kHz).
/// The UI thread reads at 60fps. The buffer holds multiple frames so the UI
/// always gets fresh data even if timing varies.
pub struct MetricsRingBuffer {
    /// Ring buffer storage.
    buffer: Box<[UiMetrics]>,
    /// Write index (audio thread only).
    write_idx: AtomicUsize,
    /// Mask for fast modulo: `idx & mask` instead of `idx % capacity`.
    mask: usize,
}

impl MetricsRingBuffer {
    /// Create a new ring buffer with the given capacity (rounded up to power of 2).
    pub fn new(min_capacity: usize) -> Self {
        let capacity = min_capacity.next_power_of_two().max(4);
        let mask = capacity - 1;

        let buffer = (0..capacity)
            .map(|_| UiMetrics::default())
            .collect::<Vec<_>>()
            .into_boxed_slice();

        Self {
            buffer,
            write_idx: AtomicUsize::new(0),
            mask,
        }
    }

    /// Write metrics to the buffer (audio thread).
    ///
    /// Lock-free, wait-free. Never blocks.
    #[inline]
    pub fn write(&self, metrics: UiMetrics) {
        let idx = self.write_idx.load(Ordering::Relaxed);
        let slot = idx & self.mask;

        // Safety: we're the only writer, and the reader only reads completed slots.
        // UiMetrics is Copy and has no pointers.
        unsafe {
            let ptr = self.buffer.as_ptr() as *mut UiMetrics;
            core::ptr::write_volatile(ptr.add(slot), metrics);
        }

        self.write_idx.store(idx.wrapping_add(1), Ordering::Release);
    }

    /// Read the latest complete metrics (UI thread).
    ///
    /// Lock-free. Returns the most recently written metrics frame.
    #[inline]
    pub fn read_latest(&self) -> UiMetrics {
        let idx = self.write_idx.load(Ordering::Acquire);
        if idx == 0 {
            return UiMetrics::default();
        }

        let slot = (idx.wrapping_sub(1)) & self.mask;

        unsafe {
            let ptr = self.buffer.as_ptr();
            core::ptr::read_volatile(ptr.add(slot))
        }
    }

    /// Get the number of frames written (for debugging).
    #[inline]
    pub fn frames_written(&self) -> usize {
        self.write_idx.load(Ordering::Relaxed)
    }
}

// Safety: MetricsRingBuffer can be shared between threads
unsafe impl Send for MetricsRingBuffer {}
unsafe impl Sync for MetricsRingBuffer {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_accumulator_basic() {
        let mut acc = MetricsAccumulator::new(4);

        acc.accumulate_levels(0.5, 0.4);
        acc.accumulate_levels(0.6, 0.5);
        acc.accumulate_levels(-0.5, -0.4);
        acc.accumulate_levels(-0.6, -0.5);

        assert!(acc.is_block_complete());

        let metrics = acc.reduce();
        assert!(metrics.input_rms_db > -10.0);
        assert!(metrics.output_rms_db > -10.0);
        assert!(metrics.input_peak_db > metrics.input_rms_db);
    }

    #[test]
    fn test_ring_buffer() {
        let buffer = MetricsRingBuffer::new(8);

        for i in 0..10 {
            let mut m = UiMetrics::default();
            m.block_counter = i;
            buffer.write(m);
        }

        let latest = buffer.read_latest();
        assert_eq!(latest.block_counter, 9);
    }

    #[test]
    fn test_solver_diagnostics_reduce() {
        let mut acc = MetricsAccumulator::new(2);
        acc.accumulate_levels(0.0, 0.0);
        acc.record_stage_solver(1, 3, 2.0e-5, true);
        acc.accumulate_levels(0.0, 0.0);
        acc.record_stage_solver(1, 8, 4.0e-4, false);

        let metrics = acc.reduce();
        assert_eq!(metrics.nr_solve_count, 2);
        assert_eq!(metrics.nr_total_iterations, 11);
        assert_eq!(metrics.nr_max_iterations, 8);
        assert_eq!(metrics.nr_nonconverged_count, 1);
        assert_eq!(metrics.stage_nr_iterations[1], 8);
        assert_eq!(metrics.stage_nr_nonconverged[1], 1);
        assert!(metrics.nr_max_residual >= 4.0e-4);
    }

    #[test]
    fn test_db_conversion() {
        let mut acc = MetricsAccumulator::new(1);

        acc.accumulate_levels(1.0, 1.0);
        let m = acc.reduce();
        assert!((m.input_rms_db - 0.0).abs() < 0.1);

        acc.accumulate_levels(0.1, 0.1);
        let m = acc.reduce();
        assert!((m.input_rms_db - (-20.0)).abs() < 0.5);
    }
}
