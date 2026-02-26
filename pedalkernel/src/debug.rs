//! Debug statistics for WDF engine diagnostics.
//!
//! This module provides lock-free statistics tracking for real-time debugging
//! of audio processing issues like clipping, DC offset, NaN/Inf, and automuting.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};

/// Lock-free debug statistics for WDF engine monitoring.
///
/// All fields use atomics for safe sharing between audio and UI threads.
/// Statistics are updated per-sample or per-buffer depending on the metric.
#[derive(Debug)]
pub struct DebugStats {
    // ── Signal Levels ──
    /// Input peak level (0.0–1.0+), updated per buffer
    input_peak_bits: AtomicU64,
    /// Output peak level (0.0–1.0+), updated per buffer
    output_peak_bits: AtomicU64,
    /// Input RMS level, updated per buffer
    input_rms_bits: AtomicU64,
    /// Output RMS level, updated per buffer
    output_rms_bits: AtomicU64,
    /// DC offset in output signal
    dc_offset_bits: AtomicU64,

    // ── Error Detection ──
    /// Count of NaN values detected in output
    nan_count: AtomicU32,
    /// Count of Inf values detected in output
    inf_count: AtomicU32,
    /// Count of samples that hit the output limiter (clipping)
    clip_count: AtomicU32,
    /// Count of silent output samples (potential automute)
    silent_count: AtomicU32,
    /// Count of denormal values detected
    denormal_count: AtomicU32,

    // ── Processing Stats ──
    /// Total samples processed
    samples_processed: AtomicU64,
    /// Maximum Newton-Raphson iterations in this buffer
    max_nr_iterations: AtomicU32,
    /// Total Newton-Raphson iterations (for averaging)
    total_nr_iterations: AtomicU64,
    /// Number of NR solves (for averaging)
    nr_solve_count: AtomicU64,

    // ── Engine State ──
    /// Sample rate in Hz (stored as u32)
    sample_rate: AtomicU32,
    /// Number of WDF stages
    stage_count: AtomicU32,
    /// Number of op-amp stages
    opamp_count: AtomicU32,
    /// Supply voltage in millivolts (for integer storage)
    supply_mv: AtomicU32,
    /// Oversampling factor (1, 2, 4, 8)
    oversampling: AtomicU32,
    /// Whether debug mode is enabled
    enabled: AtomicBool,

    // ── Per-Stage Levels (first 8 stages) ──
    stage_levels_bits: [AtomicU64; 8],
}

impl Default for DebugStats {
    fn default() -> Self {
        Self::new()
    }
}

impl DebugStats {
    /// Create new debug stats (disabled by default).
    pub fn new() -> Self {
        Self {
            input_peak_bits: AtomicU64::new(0.0_f64.to_bits()),
            output_peak_bits: AtomicU64::new(0.0_f64.to_bits()),
            input_rms_bits: AtomicU64::new(0.0_f64.to_bits()),
            output_rms_bits: AtomicU64::new(0.0_f64.to_bits()),
            dc_offset_bits: AtomicU64::new(0.0_f64.to_bits()),
            nan_count: AtomicU32::new(0),
            inf_count: AtomicU32::new(0),
            clip_count: AtomicU32::new(0),
            silent_count: AtomicU32::new(0),
            denormal_count: AtomicU32::new(0),
            samples_processed: AtomicU64::new(0),
            max_nr_iterations: AtomicU32::new(0),
            total_nr_iterations: AtomicU64::new(0),
            nr_solve_count: AtomicU64::new(0),
            sample_rate: AtomicU32::new(48000),
            stage_count: AtomicU32::new(0),
            opamp_count: AtomicU32::new(0),
            supply_mv: AtomicU32::new(9000), // 9V default
            oversampling: AtomicU32::new(1),
            enabled: AtomicBool::new(false),
            stage_levels_bits: std::array::from_fn(|_| AtomicU64::new(0.0_f64.to_bits())),
        }
    }

    /// Enable or disable debug tracking.
    pub fn set_enabled(&self, enabled: bool) {
        self.enabled.store(enabled, Ordering::Relaxed);
    }

    /// Check if debug mode is enabled.
    #[inline]
    pub fn is_enabled(&self) -> bool {
        self.enabled.load(Ordering::Relaxed)
    }

    /// Reset all counters (call when starting a new analysis session).
    pub fn reset_counters(&self) {
        self.nan_count.store(0, Ordering::Relaxed);
        self.inf_count.store(0, Ordering::Relaxed);
        self.clip_count.store(0, Ordering::Relaxed);
        self.silent_count.store(0, Ordering::Relaxed);
        self.denormal_count.store(0, Ordering::Relaxed);
        self.samples_processed.store(0, Ordering::Relaxed);
        self.max_nr_iterations.store(0, Ordering::Relaxed);
        self.total_nr_iterations.store(0, Ordering::Relaxed);
        self.nr_solve_count.store(0, Ordering::Relaxed);
    }

    // ── Audio Thread Updates (inline for RT safety) ──

    /// Record input sample for statistics.
    #[inline]
    pub fn record_input(&self, sample: f64) {
        if !self.is_enabled() {
            return;
        }
        // Update peak (relaxed ordering is fine for stats)
        let current_peak = f64::from_bits(self.input_peak_bits.load(Ordering::Relaxed));
        let abs_sample = sample.abs();
        if abs_sample > current_peak {
            self.input_peak_bits
                .store(abs_sample.to_bits(), Ordering::Relaxed);
        }
    }

    /// Record output sample for statistics.
    #[inline]
    pub fn record_output(&self, sample: f64) {
        if !self.is_enabled() {
            return;
        }

        self.samples_processed.fetch_add(1, Ordering::Relaxed);

        // Check for NaN
        if sample.is_nan() {
            self.nan_count.fetch_add(1, Ordering::Relaxed);
            return;
        }

        // Check for Inf
        if sample.is_infinite() {
            self.inf_count.fetch_add(1, Ordering::Relaxed);
            return;
        }

        let abs_sample = sample.abs();

        // Check for denormals (very small non-zero values)
        if abs_sample > 0.0 && abs_sample < 1e-38 {
            self.denormal_count.fetch_add(1, Ordering::Relaxed);
        }

        // Check for clipping (> 1.0)
        if abs_sample > 1.0 {
            self.clip_count.fetch_add(1, Ordering::Relaxed);
        }

        // Check for silence (potential automute)
        if abs_sample < 1e-10 {
            self.silent_count.fetch_add(1, Ordering::Relaxed);
        }

        // Update output peak
        let current_peak = f64::from_bits(self.output_peak_bits.load(Ordering::Relaxed));
        if abs_sample > current_peak {
            self.output_peak_bits
                .store(abs_sample.to_bits(), Ordering::Relaxed);
        }

        // Update DC offset (exponential moving average)
        let dc = f64::from_bits(self.dc_offset_bits.load(Ordering::Relaxed));
        let new_dc = dc * 0.9999 + sample * 0.0001;
        self.dc_offset_bits
            .store(new_dc.to_bits(), Ordering::Relaxed);
    }

    /// Record Newton-Raphson iteration count for a solve.
    #[inline]
    pub fn record_nr_iterations(&self, iterations: u32) {
        if !self.is_enabled() {
            return;
        }
        self.total_nr_iterations
            .fetch_add(iterations as u64, Ordering::Relaxed);
        self.nr_solve_count.fetch_add(1, Ordering::Relaxed);

        // Update max
        let current_max = self.max_nr_iterations.load(Ordering::Relaxed);
        if iterations > current_max {
            self.max_nr_iterations.store(iterations, Ordering::Relaxed);
        }
    }

    /// Record per-stage signal level (for first 8 stages).
    #[inline]
    pub fn record_stage_level(&self, stage_idx: usize, level: f64) {
        if !self.is_enabled() || stage_idx >= 8 {
            return;
        }
        self.stage_levels_bits[stage_idx].store(level.abs().to_bits(), Ordering::Relaxed);
    }

    /// Update buffer-level statistics (call once per buffer).
    pub fn update_buffer_stats(&self, input_rms: f64, output_rms: f64) {
        if !self.is_enabled() {
            return;
        }
        self.input_rms_bits
            .store(input_rms.to_bits(), Ordering::Relaxed);
        self.output_rms_bits
            .store(output_rms.to_bits(), Ordering::Relaxed);
    }

    /// Reset peak meters (call periodically from UI for decay).
    pub fn decay_peaks(&self, factor: f64) {
        let input_peak = f64::from_bits(self.input_peak_bits.load(Ordering::Relaxed));
        let output_peak = f64::from_bits(self.output_peak_bits.load(Ordering::Relaxed));
        self.input_peak_bits
            .store((input_peak * factor).to_bits(), Ordering::Relaxed);
        self.output_peak_bits
            .store((output_peak * factor).to_bits(), Ordering::Relaxed);
    }

    // ── Engine Configuration (set once at init) ──

    /// Set engine configuration.
    pub fn set_engine_config(
        &self,
        sample_rate: u32,
        stage_count: u32,
        opamp_count: u32,
        supply_voltage: f64,
        oversampling: u32,
    ) {
        self.sample_rate.store(sample_rate, Ordering::Relaxed);
        self.stage_count.store(stage_count, Ordering::Relaxed);
        self.opamp_count.store(opamp_count, Ordering::Relaxed);
        self.supply_mv
            .store((supply_voltage * 1000.0) as u32, Ordering::Relaxed);
        self.oversampling.store(oversampling, Ordering::Relaxed);
    }

    // ── UI Thread Reads ──

    /// Get input peak level.
    pub fn input_peak(&self) -> f64 {
        f64::from_bits(self.input_peak_bits.load(Ordering::Relaxed))
    }

    /// Get output peak level.
    pub fn output_peak(&self) -> f64 {
        f64::from_bits(self.output_peak_bits.load(Ordering::Relaxed))
    }

    /// Get input RMS level.
    pub fn input_rms(&self) -> f64 {
        f64::from_bits(self.input_rms_bits.load(Ordering::Relaxed))
    }

    /// Get output RMS level.
    pub fn output_rms(&self) -> f64 {
        f64::from_bits(self.output_rms_bits.load(Ordering::Relaxed))
    }

    /// Get DC offset.
    pub fn dc_offset(&self) -> f64 {
        f64::from_bits(self.dc_offset_bits.load(Ordering::Relaxed))
    }

    /// Get NaN count.
    pub fn nan_count(&self) -> u32 {
        self.nan_count.load(Ordering::Relaxed)
    }

    /// Get Inf count.
    pub fn inf_count(&self) -> u32 {
        self.inf_count.load(Ordering::Relaxed)
    }

    /// Get clip count.
    pub fn clip_count(&self) -> u32 {
        self.clip_count.load(Ordering::Relaxed)
    }

    /// Get silent sample count (potential automute detection).
    pub fn silent_count(&self) -> u32 {
        self.silent_count.load(Ordering::Relaxed)
    }

    /// Get denormal count.
    pub fn denormal_count(&self) -> u32 {
        self.denormal_count.load(Ordering::Relaxed)
    }

    /// Get total samples processed.
    pub fn samples_processed(&self) -> u64 {
        self.samples_processed.load(Ordering::Relaxed)
    }

    /// Get maximum Newton-Raphson iterations.
    pub fn max_nr_iterations(&self) -> u32 {
        self.max_nr_iterations.load(Ordering::Relaxed)
    }

    /// Get average Newton-Raphson iterations.
    pub fn avg_nr_iterations(&self) -> f64 {
        let total = self.total_nr_iterations.load(Ordering::Relaxed);
        let count = self.nr_solve_count.load(Ordering::Relaxed);
        if count > 0 {
            total as f64 / count as f64
        } else {
            0.0
        }
    }

    /// Get sample rate.
    pub fn sample_rate(&self) -> u32 {
        self.sample_rate.load(Ordering::Relaxed)
    }

    /// Get stage count.
    pub fn stage_count(&self) -> u32 {
        self.stage_count.load(Ordering::Relaxed)
    }

    /// Get op-amp count.
    pub fn opamp_count(&self) -> u32 {
        self.opamp_count.load(Ordering::Relaxed)
    }

    /// Get supply voltage in volts.
    pub fn supply_voltage(&self) -> f64 {
        self.supply_mv.load(Ordering::Relaxed) as f64 / 1000.0
    }

    /// Get oversampling factor.
    pub fn oversampling(&self) -> u32 {
        self.oversampling.load(Ordering::Relaxed)
    }

    /// Get per-stage signal level.
    pub fn stage_level(&self, stage_idx: usize) -> f64 {
        if stage_idx >= 8 {
            return 0.0;
        }
        f64::from_bits(self.stage_levels_bits[stage_idx].load(Ordering::Relaxed))
    }

    /// Format stats as a debug string for display.
    pub fn format_summary(&self) -> String {
        let samples = self.samples_processed();
        let silence_pct = if samples > 0 {
            (self.silent_count() as f64 / samples as f64) * 100.0
        } else {
            0.0
        };

        format!(
            "=== WDF Debug Stats ===\n\
             Sample Rate: {} Hz | Stages: {} | Op-Amps: {} | OS: {}x\n\
             Supply: {:.1}V | Samples: {}\n\
             \n\
             Signal Levels:\n\
             Input:  Peak {:.4} | RMS {:.4}\n\
             Output: Peak {:.4} | RMS {:.4}\n\
             DC Offset: {:.6}\n\
             \n\
             Errors:\n\
             NaN: {} | Inf: {} | Clip: {} | Denormal: {}\n\
             Silent: {} ({:.2}%)\n\
             \n\
             Newton-Raphson:\n\
             Max Iter: {} | Avg Iter: {:.2}\n\
             \n\
             Stage Levels:\n\
             {}",
            self.sample_rate(),
            self.stage_count(),
            self.opamp_count(),
            self.oversampling(),
            self.supply_voltage(),
            samples,
            self.input_peak(),
            self.input_rms(),
            self.output_peak(),
            self.output_rms(),
            self.dc_offset(),
            self.nan_count(),
            self.inf_count(),
            self.clip_count(),
            self.denormal_count(),
            self.silent_count(),
            silence_pct,
            self.max_nr_iterations(),
            self.avg_nr_iterations(),
            (0..8)
                .map(|i| format!("  S{}: {:.4}", i, self.stage_level(i)))
                .collect::<Vec<_>>()
                .join(" | ")
        )
    }

    /// Check if there are any error conditions.
    pub fn has_errors(&self) -> bool {
        self.nan_count() > 0 || self.inf_count() > 0
    }

    /// Check if there's potential automuting (>50% silent samples).
    pub fn is_automuting(&self) -> bool {
        let samples = self.samples_processed();
        if samples < 1000 {
            return false;
        }
        let silent = self.silent_count() as f64;
        silent / samples as f64 > 0.5
    }

    /// Check if DC offset is significant (>0.01).
    pub fn has_dc_offset(&self) -> bool {
        self.dc_offset().abs() > 0.01
    }
}

// SAFETY: All fields are atomic, safe to share between threads.
unsafe impl Send for DebugStats {}
unsafe impl Sync for DebugStats {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_debug_stats_basic() {
        let stats = DebugStats::new();
        stats.set_enabled(true);

        // Record some samples
        stats.record_input(0.5);
        stats.record_output(0.3);
        stats.record_output(0.8);
        stats.record_output(1.5); // clipping

        assert_eq!(stats.samples_processed(), 3);
        assert_eq!(stats.clip_count(), 1);
        assert!(stats.input_peak() >= 0.5);
        assert!(stats.output_peak() >= 0.8);
    }

    #[test]
    fn test_nan_detection() {
        let stats = DebugStats::new();
        stats.set_enabled(true);

        stats.record_output(f64::NAN);
        stats.record_output(0.5);
        stats.record_output(f64::INFINITY);

        assert_eq!(stats.nan_count(), 1);
        assert_eq!(stats.inf_count(), 1);
        assert!(stats.has_errors());
    }

    #[test]
    fn test_automute_detection() {
        let stats = DebugStats::new();
        stats.set_enabled(true);

        // Simulate many silent samples
        for _ in 0..2000 {
            stats.record_output(0.0);
        }

        assert!(stats.is_automuting());
    }
}
