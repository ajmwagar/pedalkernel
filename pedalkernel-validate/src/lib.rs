//! # PedalKernel Validation Library
//!
//! A toolkit for validating WDF (Wave Digital Filter) circuit simulations against
//! reference implementations (analytical, SPICE, or hardware measurements).
//!
//! ## Library Usage
//!
//! This crate can be used as a dependency for writing custom validation tests:
//!
//! ```toml
//! [dependencies]
//! pedalkernel-validate = { path = "../pedalkernel-validate" }
//! ```
//!
//! ### Signal Generation
//!
//! Generate deterministic test signals for audio processing validation:
//!
//! ```rust
//! use pedalkernel_validate::signals;
//!
//! let sample_rate = 48000.0;
//!
//! // Unit impulse (100ms duration)
//! let impulse = signals::impulse(4800, 1.0);
//!
//! // 1kHz sine wave, 100ms duration, amplitude 0.5
//! let sine = signals::sine(sample_rate, 1000.0, 0.1, 0.5);
//!
//! // Two-tone IMD test signal
//! let two_tone = signals::two_tone(sample_rate, 1000.0, 1100.0, 0.1, 0.5);
//!
//! // Exponential frequency sweep 20Hz-20kHz over 1 second
//! let sweep = signals::exp_sweep(sample_rate, 20.0, 20000.0, 1.0, 0.8);
//!
//! // Tone burst for attack/release testing
//! let burst = signals::tone_burst(sample_rate, 1000.0, 0.5, 10.0, 90.0, 5);
//!
//! // Level sweep for gain curve measurement
//! let levels = signals::level_sweep(sample_rate, 1000.0, &[-20.0, -10.0, 0.0, 10.0], 0.5);
//! ```
//!
//! ### Comparison Metrics
//!
//! Compare two audio signals with various metrics:
//!
//! ```rust
//! use pedalkernel_validate::metrics;
//!
//! let wdf_output: Vec<f64> = vec![/* your WDF output */];
//! let reference: Vec<f64> = vec![/* golden reference */];
//! let sample_rate = 48000.0;
//!
//! // Normalized RMS error in dB (lower is better, -60dB = 0.1% error)
//! let rms_err = metrics::normalized_rms_error_db(&wdf_output, &reference);
//!
//! // Peak error in dB
//! let peak_err = metrics::peak_error_db(&wdf_output, &reference);
//!
//! // THD (Total Harmonic Distortion) in dB
//! let thd = metrics::thd_db(&wdf_output, 1000.0, sample_rate, 10);
//!
//! // THD difference between WDF and reference
//! let thd_err = metrics::thd_error_db(&wdf_output, &reference, 1000.0, sample_rate);
//!
//! // Even/odd harmonic ratio (for push-pull validation)
//! let ratio = metrics::even_odd_ratio_db(&wdf_output, 1000.0, sample_rate, 10);
//!
//! // Spectral error in dB
//! let spectral_err = metrics::spectral_error_db(&wdf_output, &reference, sample_rate, None);
//!
//! // DC drift measurement
//! let dc_drift = metrics::dc_drift_mv(&wdf_output, sample_rate, 100.0);
//!
//! // Or use the combined comparison function:
//! let result = metrics::compare(&wdf_output, &reference, sample_rate, Some(1000.0));
//! println!("RMS error: {:.1} dB", result.normalized_rms_error_db);
//! println!("Peak error: {:.1} dB", result.peak_error_db);
//! ```
//!
//! ### Analytical References
//!
//! Generate mathematically exact references for linear circuits:
//!
//! ```rust
//! use pedalkernel_validate::analytical;
//!
//! let r = 10_000.0;  // 10k ohms
//! let c = 10e-9;     // 10nF
//! let sample_rate = 48000.0;
//!
//! // Impulse response of RC lowpass (bilinear transform)
//! let ir = analytical::rc_lowpass_impulse_response(r, c, sample_rate, 1000);
//!
//! // Filter a signal through ideal RC lowpass
//! let input = vec![1.0; 1000];
//! let output = analytical::rc_lowpass_filter(&input, r, c, sample_rate);
//!
//! // Frequency response calculations
//! let mag = analytical::rc_lowpass_magnitude(r, c, 1000.0);  // magnitude at 1kHz
//! let phase = analytical::rc_lowpass_phase(r, c, 1000.0);    // phase at 1kHz
//! ```
//!
//! ### NumPy File I/O
//!
//! Read/write NumPy .npy files for interoperability with Python:
//!
//! ```rust,ignore
//! use pedalkernel_validate::npy;
//!
//! // Write test data
//! let data = vec![1.0, 2.0, 3.0, 4.0];
//! npy::write_f64("output.npy", &data).unwrap();
//!
//! // Read reference data
//! let reference = npy::read_f64("golden.npy").unwrap();
//! ```
//!
//! ### Custom Test Runner
//!
//! Build custom validation pipelines:
//!
//! ```rust,ignore
//! use pedalkernel_validate::{signals, metrics, npy};
//! use pedalkernel_validate::config::PassCriteria;
//!
//! fn validate_my_circuit(
//!     circuit_path: &str,
//!     golden_path: &str,
//!     sample_rate: f64,
//! ) -> bool {
//!     // Generate test signal
//!     let input = signals::sine(sample_rate, 1000.0, 0.1, 1.0);
//!
//!     // Process through your circuit (pseudocode)
//!     // let output = my_circuit.process(&input);
//!     let output = input.clone(); // placeholder
//!
//!     // Load golden reference
//!     let golden = npy::read_f64(golden_path).unwrap();
//!
//!     // Compare
//!     let result = metrics::compare(&output, &golden, sample_rate, Some(1000.0));
//!
//!     // Check against criteria
//!     let criteria = PassCriteria {
//!         normalized_rms_error_db: Some(-60.0),
//!         peak_error_db: Some(-40.0),
//!         thd_error_db: Some(1.0),
//!         ..Default::default()
//!     };
//!
//!     result.passes(&criteria)
//! }
//! ```
//!
//! ## CLI Usage
//!
//! The crate also provides a CLI for running predefined test suites:
//!
//! ```bash
//! # List available tests
//! pedalkernel-validate list
//!
//! # Run all validation suites
//! pedalkernel-validate run --suite all
//!
//! # Run specific suite
//! pedalkernel-validate run --suite nonlinear
//!
//! # Quick validate a single circuit
//! pedalkernel-validate quick my_circuit.pedal
//!
//! # Bootstrap golden references from current WDF output
//! pedalkernel-validate bootstrap --suite all
//!
//! # Generate analytical golden references
//! pedalkernel-validate generate-linear
//! ```
//!
//! ## Module Overview
//!
//! - [`signals`] - Deterministic test signal generators
//! - [`metrics`] - Audio comparison metrics (RMS, THD, spectral, etc.)
//! - [`analytical`] - Mathematically exact references for linear circuits
//! - [`npy`] - NumPy .npy file I/O
//! - [`config`] - YAML-based test configuration
//! - [`runner`] - Test orchestration
//! - [`report`] - JSON and terminal reporting

// ============================================================================
// Public modules
// ============================================================================

pub mod analytical;
pub mod config;
pub mod metrics;
pub mod npy;
pub mod report;
pub mod runner;
pub mod signals;

// ============================================================================
// Top-level re-exports for convenience
// ============================================================================

// Config types
pub use config::{
    ConfigError,
    GlobalConfig,
    MetricConfig,
    PassCriteria,
    SignalConfig,
    TestCase,
    TestSuite,
    ValidationConfig,
};

// Metrics types
pub use metrics::ComparisonResult;

// Runner types
pub use runner::{RunnerConfig, RunnerError, ValidationRunner};

// Report types
pub use report::{
    ComparisonMetrics,
    ReportSummary,
    SignalResult,
    SuiteResult,
    TestResult,
    ValidationReport,
};

// Signal types
pub use signals::SignalSpec;

/// Prelude module - import everything commonly needed
///
/// ```rust
/// use pedalkernel_validate::prelude::*;
/// ```
pub mod prelude {
    // Signal generation
    pub use crate::signals::{
        dbvu_to_peak, exp_sweep, impulse, level_sweep, silence, sine, tone_burst, two_tone,
        SignalSpec,
    };

    // Metrics
    pub use crate::metrics::{
        compare, dc_drift_mv, even_odd_ratio_db, normalized_rms_error_db, peak_error_db,
        spectral_error_db, thd_db, thd_error_db, ComparisonResult,
    };

    // Analytical references
    pub use crate::analytical::{
        rc_lowpass_filter, rc_lowpass_impulse_response, rc_lowpass_magnitude, rc_lowpass_phase,
    };

    // Config
    pub use crate::config::{
        GlobalConfig, MetricConfig, PassCriteria, SignalConfig, TestCase, TestSuite,
        ValidationConfig,
    };

    // NPY I/O
    pub use crate::npy::{exists as npy_exists, read_f64 as npy_read, write_f64 as npy_write};

    // Runner
    pub use crate::runner::{quick_validate, RunnerConfig, ValidationRunner};

    // Report
    pub use crate::report::{SignalResult, SuiteResult, TestResult, ValidationReport};
}
