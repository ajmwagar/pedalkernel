//! Runtime warnings for hybrid linear/nonlinear devices.
//!
//! Some WDF components (JFETs, OTAs, photocouplers) are nonlinear by nature but
//! are stamped into MNA matrices as linear elements when used in their linear
//! operating region. Runtime warnings detect when signal levels or control
//! voltages push these devices outside their valid region.
//!
//! Zero cost when `runtime-warnings` feature is disabled — all check methods,
//! tracking fields, and collector code are behind `#[cfg(feature = "runtime-warnings")]`.
//!
//! Output: `eprintln!` + thread-local ring buffer. Rate-limited to at most one
//! warning per device per 1024 samples (~21ms at 48kHz).

use hashbrown::HashMap;
use std::cell::RefCell;
use std::fmt;

/// Rate limit: at most one warning per device per this many samples.
const COOLDOWN_SAMPLES: u64 = 1024;

/// Maximum entries in the ring buffer before oldest are evicted.
const RING_BUFFER_CAPACITY: usize = 64;

// ---------------------------------------------------------------------------
// Warning kind
// ---------------------------------------------------------------------------

/// All possible runtime warning conditions for hybrid devices.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum WarningKind {
    // JFET variable resistor warnings
    /// |Vds| > 20% of |Vgs - Vp| — JFET leaving triode region.
    JfetLeavingTriode,
    /// Rds < Rds_on × 0.9 — JFET near fully on.
    JfetNearFullyOn,
    /// Rds > 10MΩ — JFET near pinch-off.
    JfetNearPinchOff,
    /// |ΔRds| > 50% of Rds per sample — excessive slew rate.
    JfetRdsSlewExceeded,

    // OTA warnings
    /// |Vdiff| > 20mV — OTA entering compression region.
    OtaEnteringCompression,
    /// |Vdiff| > 40mV — OTA hard saturation.
    OtaHardSaturation,
    /// Iabc < 1µA — OTA effectively off.
    OtaEffectivelyOff,
    /// Iabc > 1mA — OTA overbiased.
    OtaOverbiased,

    // Photocoupler warnings
    /// R < R_light — photocoupler overdrive.
    PhotocouplerOverdrive,
    /// R > 0.95 × R_dark — photocoupler open circuit.
    PhotocouplerOpenCircuit,
    /// |ΔR/Δt| exceeds physical spec — photocoupler slew exceeded.
    PhotocouplerSlewExceeded,
}

impl fmt::Display for WarningKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::JfetLeavingTriode => write!(f, "JFET leaving triode region"),
            Self::JfetNearFullyOn => write!(f, "JFET near fully on"),
            Self::JfetNearPinchOff => write!(f, "JFET near pinch-off"),
            Self::JfetRdsSlewExceeded => write!(f, "JFET Rds slew exceeded"),
            Self::OtaEnteringCompression => write!(f, "OTA entering compression"),
            Self::OtaHardSaturation => write!(f, "OTA hard saturation"),
            Self::OtaEffectivelyOff => write!(f, "OTA effectively off"),
            Self::OtaOverbiased => write!(f, "OTA overbiased"),
            Self::PhotocouplerOverdrive => write!(f, "Photocoupler overdrive"),
            Self::PhotocouplerOpenCircuit => write!(f, "Photocoupler open circuit"),
            Self::PhotocouplerSlewExceeded => write!(f, "Photocoupler slew exceeded"),
        }
    }
}

// ---------------------------------------------------------------------------
// Severity
// ---------------------------------------------------------------------------

/// Severity level for runtime warnings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Severity {
    /// Device is approaching limits but may still be acceptable.
    Caution,
    /// Device has left its valid operating region.
    Danger,
}

impl fmt::Display for Severity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Caution => write!(f, "CAUTION"),
            Self::Danger => write!(f, "DANGER"),
        }
    }
}

// ---------------------------------------------------------------------------
// Warning struct
// ---------------------------------------------------------------------------

/// A single runtime warning emitted by a hybrid device.
#[derive(Debug, Clone)]
pub struct RuntimeWarning {
    /// Sample index when the warning was emitted.
    pub sample_index: u64,
    /// Component ID that generated the warning.
    pub component_id: String,
    /// What kind of warning.
    pub kind: WarningKind,
    /// Severity level.
    pub severity: Severity,
    /// The measured value that triggered the warning.
    pub value: f64,
    /// The threshold that was exceeded.
    pub threshold: f64,
    /// Human-readable message.
    pub message: String,
}

impl fmt::Display for RuntimeWarning {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "[{}] {} ({}): {} (value={:.4e}, threshold={:.4e}) @ sample {}",
            self.severity,
            self.kind,
            self.component_id,
            self.message,
            self.value,
            self.threshold,
            self.sample_index,
        )
    }
}

// ---------------------------------------------------------------------------
// Warning collector (thread-local ring buffer with per-device cooldown)
// ---------------------------------------------------------------------------

/// Collects warnings with rate limiting and ring buffer eviction.
struct WarningCollector {
    /// Ring buffer of recent warnings.
    buffer: Vec<RuntimeWarning>,
    /// Write position in ring buffer (wraps at capacity).
    write_pos: usize,
    /// Total warnings recorded (including evicted).
    total_count: usize,
    /// Per-(component, kind) cooldown: last sample index that warning was emitted.
    cooldowns: HashMap<(String, WarningKind), u64>,
}

impl Default for WarningCollector {
    fn default() -> Self {
        Self {
            buffer: Vec::with_capacity(RING_BUFFER_CAPACITY),
            write_pos: 0,
            total_count: 0,
            cooldowns: HashMap::new(),
        }
    }
}

impl WarningCollector {
    /// Try to record a warning. Returns false if rate-limited.
    fn record(&mut self, warning: RuntimeWarning) -> bool {
        let key = (warning.component_id.clone(), warning.kind);
        if let Some(&last_sample) = self.cooldowns.get(&key) {
            if warning.sample_index.saturating_sub(last_sample) < COOLDOWN_SAMPLES {
                return false; // rate-limited
            }
        }
        self.cooldowns.insert(key, warning.sample_index);

        // Insert into ring buffer
        if self.buffer.len() < RING_BUFFER_CAPACITY {
            self.buffer.push(warning);
        } else {
            self.buffer[self.write_pos] = warning;
        }
        self.write_pos = (self.write_pos + 1) % RING_BUFFER_CAPACITY;
        self.total_count += 1;
        true
    }

    /// Drain all buffered warnings in insertion order, clearing the buffer.
    fn drain(&mut self) -> Vec<RuntimeWarning> {
        let result = if self.buffer.len() < RING_BUFFER_CAPACITY
            || self.total_count <= RING_BUFFER_CAPACITY
        {
            // Buffer hasn't wrapped yet — drain in order
            self.buffer.drain(..).collect()
        } else {
            // Buffer has wrapped — read from write_pos to end, then start to write_pos
            let mut out = Vec::with_capacity(self.buffer.len());
            out.extend(self.buffer[self.write_pos..].iter().cloned());
            out.extend(self.buffer[..self.write_pos].iter().cloned());
            self.buffer.clear();
            out
        };
        self.write_pos = 0;
        self.total_count = 0;
        self.cooldowns.clear();
        result
    }

    /// Reset all state (buffer, cooldowns, counters).
    fn reset(&mut self) {
        self.buffer.clear();
        self.write_pos = 0;
        self.total_count = 0;
        self.cooldowns.clear();
    }
}

thread_local! {
    static WARNING_COLLECTOR: RefCell<WarningCollector> = RefCell::new(WarningCollector::default());
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Record a runtime warning. Rate-limited per (component, kind) pair.
///
/// Also prints via `eprintln!` when the warning is not rate-limited.
pub fn record_warning(warning: RuntimeWarning) {
    WARNING_COLLECTOR.with(|collector| {
        let msg = warning.to_string();
        if collector.borrow_mut().record(warning) {
            tracing::warn!("{}", msg);
        }
    });
}

/// Drain all buffered warnings, returning them in insertion order.
/// Clears the buffer and resets cooldowns.
pub fn drain_warnings() -> Vec<RuntimeWarning> {
    WARNING_COLLECTOR.with(|collector| collector.borrow_mut().drain())
}

/// Reset the warning collector (clears buffer and cooldowns).
pub fn reset_warnings() {
    WARNING_COLLECTOR.with(|collector| collector.borrow_mut().reset());
}

/// Helper to construct and record a warning in one call.
pub fn emit_warning(
    sample_index: u64,
    component_id: &str,
    kind: WarningKind,
    severity: Severity,
    value: f64,
    threshold: f64,
    message: &str,
) {
    record_warning(RuntimeWarning {
        sample_index,
        component_id: component_id.to_string(),
        kind,
        severity,
        value,
        threshold,
        message: message.to_string(),
    });
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_warning(comp: &str, kind: WarningKind, sample: u64) -> RuntimeWarning {
        RuntimeWarning {
            sample_index: sample,
            component_id: comp.to_string(),
            kind,
            severity: Severity::Caution,
            value: 1.0,
            threshold: 0.5,
            message: "test".to_string(),
        }
    }

    #[test]
    fn rate_limiting_prevents_duplicates_within_cooldown() {
        reset_warnings();

        // First warning should be recorded
        record_warning(make_warning("J1", WarningKind::JfetNearPinchOff, 0));
        // Within cooldown — should be rate-limited
        record_warning(make_warning("J1", WarningKind::JfetNearPinchOff, 500));
        // Still within cooldown
        record_warning(make_warning("J1", WarningKind::JfetNearPinchOff, 1023));

        let warnings = drain_warnings();
        assert_eq!(warnings.len(), 1);
        assert_eq!(warnings[0].sample_index, 0);
    }

    #[test]
    fn rate_limiting_allows_after_cooldown() {
        reset_warnings();

        record_warning(make_warning("J1", WarningKind::JfetNearPinchOff, 0));
        // Exactly at cooldown boundary — should be allowed
        record_warning(make_warning("J1", WarningKind::JfetNearPinchOff, 1024));

        let warnings = drain_warnings();
        assert_eq!(warnings.len(), 2);
    }

    #[test]
    fn different_components_have_independent_cooldowns() {
        reset_warnings();

        record_warning(make_warning("J1", WarningKind::JfetNearPinchOff, 0));
        record_warning(make_warning("J2", WarningKind::JfetNearPinchOff, 0));

        let warnings = drain_warnings();
        assert_eq!(warnings.len(), 2);
    }

    #[test]
    fn different_kinds_have_independent_cooldowns() {
        reset_warnings();

        record_warning(make_warning("J1", WarningKind::JfetNearPinchOff, 0));
        record_warning(make_warning("J1", WarningKind::JfetLeavingTriode, 0));

        let warnings = drain_warnings();
        assert_eq!(warnings.len(), 2);
    }

    #[test]
    fn ring_buffer_evicts_oldest_when_full() {
        reset_warnings();

        // Fill beyond capacity (each at different sample to avoid rate limiting)
        for i in 0..(RING_BUFFER_CAPACITY + 10) as u64 {
            let sample = i * COOLDOWN_SAMPLES; // avoid rate limiting
            let comp = format!("C{}", i); // unique component per warning
            record_warning(make_warning(&comp, WarningKind::OtaHardSaturation, sample));
        }

        let warnings = drain_warnings();
        // Should have exactly RING_BUFFER_CAPACITY entries
        assert_eq!(warnings.len(), RING_BUFFER_CAPACITY);
        // Oldest should have been evicted — first entry should be #10
        assert_eq!(warnings[0].component_id, "C10");
    }

    #[test]
    fn drain_clears_buffer() {
        reset_warnings();

        record_warning(make_warning("J1", WarningKind::JfetNearPinchOff, 0));
        let w1 = drain_warnings();
        assert_eq!(w1.len(), 1);

        let w2 = drain_warnings();
        assert!(w2.is_empty());
    }

    #[test]
    fn reset_clears_everything() {
        reset_warnings();

        record_warning(make_warning("J1", WarningKind::JfetNearPinchOff, 0));
        reset_warnings();

        let warnings = drain_warnings();
        assert!(warnings.is_empty());

        // After reset, cooldowns should also be cleared — warning at sample 0 should work
        record_warning(make_warning("J1", WarningKind::JfetNearPinchOff, 0));
        let warnings = drain_warnings();
        assert_eq!(warnings.len(), 1);
    }
}
