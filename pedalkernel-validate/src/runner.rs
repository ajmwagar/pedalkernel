//! Test execution and orchestration.
//!
//! This module provides the [`ValidationRunner`] for executing validation tests,
//! as well as the [`quick_validate`] function for single-circuit validation.
//!
//! # Using ValidationRunner
//!
//! ```rust,ignore
//! use pedalkernel_validate::{
//!     config::ValidationConfig,
//!     runner::{RunnerConfig, ValidationRunner},
//! };
//!
//! // Configure the runner
//! let runner_config = RunnerConfig {
//!     circuits_dir: "circuits".into(),
//!     golden_dir: "golden".into(),
//!     output_dir: "output".into(),
//!     sample_rate: 96000,
//!     oversample: 4,
//!     save_output: true,
//!     regenerate_golden: false,
//! };
//!
//! // Load validation config
//! let validation_config = ValidationConfig::default_config();
//!
//! // Create runner and execute
//! let runner = ValidationRunner::new(runner_config, validation_config);
//! let results = runner.run_all().unwrap();
//!
//! // Check results
//! for (suite_name, suite_result) in &results {
//!     println!("{}: {}/{} passed",
//!         suite_name, suite_result.passed, suite_result.passed + suite_result.failed);
//! }
//! ```
//!
//! # Quick Validation
//!
//! For validating a single circuit without full configuration:
//!
//! ```rust,ignore
//! use pedalkernel_validate::runner::quick_validate;
//! use std::path::Path;
//!
//! let result = quick_validate(
//!     Path::new("my_circuit.pedal"),
//!     Some(Path::new("golden.npy")),
//!     96000.0
//! ).unwrap();
//!
//! println!("RMS error: {:.1} dB", result.normalized_rms_error_db);
//! ```

use crate::config::{PassCriteria, SignalConfig, TestCase, TestSuite, ValidationConfig};
use crate::metrics::{self, ComparisonResult};
use crate::npy;
use crate::report::{SignalResult, SuiteResult, TestResult};
use pedalkernel::compiler::{compile_pedal_with_options, CompileOptions, CompiledPedal};
use pedalkernel::oversampling::OversamplingFactor;
use pedalkernel::PedalProcessor;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum RunnerError {
    #[error("Failed to load circuit: {0}")]
    CircuitLoadError(String),
    #[error("Failed to compile circuit: {0}")]
    CompileError(String),
    #[error("Golden reference not found: {0}")]
    GoldenNotFound(PathBuf),
    #[error("NPY error: {0}")]
    NpyError(#[from] crate::npy::NpyError),
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),
}

/// Validation runner configuration.
pub struct RunnerConfig {
    /// Base directory for circuit files.
    pub circuits_dir: PathBuf,
    /// Base directory for golden references.
    pub golden_dir: PathBuf,
    /// Directory to write WDF output for comparison.
    pub output_dir: PathBuf,
    /// Sample rate for processing.
    pub sample_rate: u32,
    /// Oversampling factor.
    pub oversample: u32,
    /// Whether to save WDF output to disk.
    pub save_output: bool,
    /// Whether to regenerate golden references (requires SPICE).
    pub regenerate_golden: bool,
    /// Skip K-method table generation (offline NR fallback). Saves build time
    /// for validation runs, which are not real-time.
    pub skip_k_tables: bool,
}

impl Default for RunnerConfig {
    fn default() -> Self {
        Self {
            circuits_dir: PathBuf::from("circuits"),
            golden_dir: PathBuf::from("golden"),
            output_dir: PathBuf::from("output"),
            sample_rate: 96000,
            oversample: 4,
            save_output: true,
            regenerate_golden: false,
            skip_k_tables: false,
        }
    }
}

/// Main validation runner.
pub struct ValidationRunner {
    config: RunnerConfig,
    validation_config: ValidationConfig,
}

impl ValidationRunner {
    /// Create a new runner with the given configuration.
    pub fn new(config: RunnerConfig, validation_config: ValidationConfig) -> Self {
        Self {
            config,
            validation_config,
        }
    }

    /// Run all test suites.
    pub fn run_all(&self) -> Result<BTreeMap<String, SuiteResult>, RunnerError> {
        let mut results = BTreeMap::new();
        for (suite_name, suite) in &self.validation_config.suites {
            let suite_result = self.run_suite(suite_name, suite)?;
            results.insert(suite_name.clone(), suite_result);
        }
        Ok(results)
    }

    /// Run a specific suite by name.
    pub fn run_suite(
        &self,
        suite_name: &str,
        suite: &TestSuite,
    ) -> Result<SuiteResult, RunnerError> {
        let mut test_results = BTreeMap::new();
        let mut passed = 0;
        let mut failed = 0;
        let mut pending = 0;

        for (test_name, test_case) in &suite.tests {
            match self.run_test(suite_name, test_name, test_case) {
                Ok(result) => {
                    if result.pending {
                        // Excluded from both passed and failed (and thus the
                        // gate total/denominator).
                        pending += 1;
                    } else if result.passed {
                        passed += 1;
                    } else {
                        failed += 1;
                    }
                    test_results.insert(test_name.clone(), result);
                }
                Err(e) => {
                    failed += 1;
                    test_results.insert(
                        test_name.clone(),
                        TestResult {
                            passed: false,
                            pending: false,
                            error: Some(e.to_string()),
                            signals: vec![],
                        },
                    );
                }
            }
        }

        Ok(SuiteResult {
            description: suite.description.clone(),
            passed,
            failed,
            pending,
            tests: test_results,
        })
    }

    /// Run a single test case.
    pub fn run_test(
        &self,
        suite_name: &str,
        test_name: &str,
        test_case: &TestCase,
    ) -> Result<TestResult, RunnerError> {
        // Resolve circuit path
        let circuit_path = self.config.circuits_dir.join(&test_case.circuit);

        // Load and compile the circuit
        let mut pedal = self.load_and_compile(&circuit_path)?;

        let effective_sr = self.config.sample_rate * self.config.oversample;
        let warmup_trim_ms = test_case.effective_warmup_trim_ms(&self.validation_config.global);
        let mut signal_results = vec![];
        let mut all_passed = true;
        let mut any_pending = false;

        for signal_config in &test_case.signals {
            let signal_result = self.run_signal(
                suite_name,
                test_name,
                signal_config,
                &mut pedal,
                effective_sr as f64,
                &test_case.pass_criteria,
                warmup_trim_ms,
                test_case.pending_reference,
            )?;

            if signal_result.pending {
                any_pending = true;
            } else if !signal_result.passed {
                all_passed = false;
            }
            signal_results.push(signal_result);
        }

        // A test is PENDING when at least one of its signals is pending (golden
        // missing + pending_reference). Pending tests are neither passed nor
        // failed; the suite excludes them from the gate counts.
        if any_pending {
            return Ok(TestResult {
                passed: false,
                pending: true,
                error: Some("golden not generated".to_string()),
                signals: signal_results,
            });
        }

        Ok(TestResult {
            passed: all_passed,
            pending: false,
            error: None,
            signals: signal_results,
        })
    }

    /// Run a single signal through the circuit and compare to golden.
    ///
    /// `warmup_trim_ms` milliseconds are discarded from the head of both the
    /// WDF output and the golden reference before any metric is computed.  The
    /// stored golden files are left untouched.
    #[allow(clippy::too_many_arguments)]
    fn run_signal(
        &self,
        suite_name: &str,
        test_name: &str,
        signal_config: &SignalConfig,
        pedal: &mut CompiledPedal,
        sample_rate: f64,
        pass_criteria: &PassCriteria,
        warmup_trim_ms: f64,
        pending_reference: bool,
    ) -> Result<SignalResult, RunnerError> {
        let signal_label = signal_config.label();

        // Generate input signal
        let spec = signal_config.to_spec();
        let input = spec.generate(sample_rate);

        // Process through WDF
        let output = self.process_signal(pedal, &input);

        // Save WDF output if configured (full signal, pre-trim)
        if self.config.save_output {
            let output_path = self
                .config
                .output_dir
                .join(suite_name)
                .join(test_name)
                .join(format!("{}.npy", signal_label));
            npy::write_f64(&output_path, &output)?;
        }

        // Load golden reference
        let golden_path = self
            .config
            .golden_dir
            .join(suite_name)
            .join(test_name)
            .join(format!("{}.npy", signal_label));

        if !golden_path.exists() {
            // No golden reference - can't compare, but circuit ran.
            if pending_reference {
                // Pending reference: the .npy has not been generated yet (e.g.
                // committed before ngspice produced it). Report as PENDING so it
                // is excluded from the gate's passed AND total counts, rather
                // than failing. Once the golden is dropped in, golden_path will
                // exist and the comparison below runs normally — no code change
                // needed (auto-activation).
                return Ok(SignalResult::pending(
                    signal_label,
                    format!("golden not generated: {:?}", golden_path),
                ));
            }
            // Not pending: a missing golden is a genuine failure (e.g. an
            // accidental deletion), so fail loudly.
            return Ok(SignalResult {
                label: signal_label,
                passed: false,
                pending: false,
                comparison: None,
                error: Some(format!("Golden reference not found: {:?}", golden_path)),
            });
        }

        let golden = npy::read_f64(&golden_path)?;

        // Apply steady-state warmup trim to both signals before metric computation.
        // Trimming happens here (metric time) so stored golden files remain valid.
        let trim_samples = ((warmup_trim_ms / 1000.0) * sample_rate).round() as usize;
        let output_trimmed = if trim_samples < output.len() {
            &output[trim_samples..]
        } else {
            &output[..]
        };
        let golden_trimmed = if trim_samples < golden.len() {
            &golden[trim_samples..]
        } else {
            &golden[..]
        };

        // Compute comparison metrics on trimmed windows
        let fundamental_hz = signal_config.fundamental_hz();
        let mut comparison =
            metrics::compare(output_trimmed, golden_trimmed, sample_rate, fundamental_hz);

        // Cap the spectral comparison at the audio Nyquist (base_rate / 2).
        // Circuits run oversampled and the engine anti-aliases, deliberately
        // rolling off content above the audio band before downsampling. The raw
        // ngspice golden, sampled at the oversampled rate, retains that
        // ultrasonic content — so comparing above the audio Nyquist scores our
        // anti-aliasing as error. `metrics::compare` only sees the oversampled
        // `sample_rate`, so override its bundled spectral value here where the
        // base rate is known.
        let audio_nyquist_hz = self.config.sample_rate as f64 / 2.0;
        comparison.spectral_error_db = metrics::spectral_error_db(
            output_trimmed,
            golden_trimmed,
            sample_rate,
            Some(audio_nyquist_hz),
        );

        // Check pass/fail
        let passed = comparison.passes(pass_criteria);

        Ok(SignalResult {
            label: signal_label,
            passed,
            pending: false,
            comparison: Some(comparison.into()),
            error: None,
        })
    }

    /// Load a .pedal file and compile it.
    fn load_and_compile(&self, path: &Path) -> Result<CompiledPedal, RunnerError> {
        let contents = std::fs::read_to_string(path)
            .map_err(|e| RunnerError::CircuitLoadError(format!("{}: {}", path.display(), e)))?;

        let pedal_def = pedalkernel::dsl::parse_pedal_file(&contents)
            .map_err(|e| RunnerError::CompileError(format!("Parse error: {}", e)))?;

        let effective_sr = (self.config.sample_rate * self.config.oversample) as f64;

        // Test circuits have no Gain/Drive controls, so they automatically get unity gain.
        // Only pedals with explicit gain controls get automatic distortion gain.
        let options = CompileOptions {
            oversampling: OversamplingFactor::X1,
            skip_k_tables: self.config.skip_k_tables,
            ..CompileOptions::default()
        };

        compile_pedal_with_options(&pedal_def, effective_sr, options)
            .map_err(|e| RunnerError::CompileError(format!("Compile error: {}", e)))
    }

    /// Process input signal through the compiled pedal.
    fn process_signal(&self, pedal: &mut CompiledPedal, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());

        // Reset pedal state
        pedal.reset();

        for &sample in input {
            let out = pedal.process(sample);
            output.push(out);
        }

        // If oversampling, we might want to decimate here
        // For now, return at full rate for comparison
        output
    }

    /// Get the sample rate used for processing.
    pub fn effective_sample_rate(&self) -> f64 {
        (self.config.sample_rate * self.config.oversample) as f64
    }
}

/// Quick validation of a single circuit with default settings.
pub fn quick_validate(
    circuit_path: &Path,
    golden_path: Option<&Path>,
    sample_rate: f64,
) -> Result<ComparisonResult, RunnerError> {
    let contents = std::fs::read_to_string(circuit_path)
        .map_err(|e| RunnerError::CircuitLoadError(e.to_string()))?;

    let pedal_def = pedalkernel::dsl::parse_pedal_file(&contents)
        .map_err(|e| RunnerError::CompileError(e.to_string()))?;

    let options = CompileOptions {
        oversampling: OversamplingFactor::X1,
        ..CompileOptions::default()
    };

    let mut pedal = compile_pedal_with_options(&pedal_def, sample_rate, options)
        .map_err(|e| RunnerError::CompileError(e.to_string()))?;

    // Generate 1kHz sine test signal
    let input = crate::signals::sine(sample_rate, 1000.0, 0.1, 1.0);

    // Process
    let mut output = Vec::with_capacity(input.len());
    for &sample in &input {
        output.push(pedal.process(sample));
    }

    // Compare to golden if provided
    if let Some(gp) = golden_path {
        let golden = npy::read_f64(gp)?;
        Ok(metrics::compare(
            &output,
            &golden,
            sample_rate,
            Some(1000.0),
        ))
    } else {
        // Self-comparison (useful for sanity check)
        Ok(metrics::compare(
            &output,
            &output,
            sample_rate,
            Some(1000.0),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn runner_config_has_sensible_defaults() {
        let config = RunnerConfig::default();
        assert_eq!(config.sample_rate, 96000);
        assert_eq!(config.oversample, 4);
        assert!(config.save_output);
    }

    #[test]
    fn warmup_trim_sample_count_is_correct() {
        // At 384 kHz internal rate, 10 ms = 3840 samples.
        let sample_rate = 384_000.0_f64;
        let warmup_trim_ms = 10.0_f64;
        let trim_samples = ((warmup_trim_ms / 1000.0) * sample_rate).round() as usize;
        assert_eq!(trim_samples, 3840);
    }

    #[test]
    fn warmup_trim_preserves_enough_window() {
        // Shortest signal: 50 ms at 384 kHz = 19200 samples.
        // After 10 ms trim: 15360 samples remain = 40 ms.
        // 1 kHz fundamental at 40 ms = 40 cycles — sufficient for THD (needs ~5).
        let sample_rate = 384_000.0_f64;
        let signal_len = (0.050 * sample_rate) as usize;
        let trim_samples = ((10.0_f64 / 1000.0) * sample_rate).round() as usize;
        let remaining = signal_len - trim_samples;
        let remaining_ms = remaining as f64 / sample_rate * 1000.0;
        let cycles_at_1khz = remaining_ms / 1.0; // 1 ms per cycle at 1 kHz
        assert!(remaining_ms >= 20.0, "at least 20 ms must remain after trim");
        assert!(
            cycles_at_1khz >= 5.0,
            "at least 5 cycles required for THD; got {cycles_at_1khz}"
        );
    }

    #[test]
    fn warmup_trim_clamps_when_signal_shorter_than_trim() {
        // If trim >= signal length, fall back to full slice (do not panic).
        let output: Vec<f64> = vec![1.0, 2.0, 3.0];
        let trim_samples = 10;
        let trimmed = if trim_samples < output.len() {
            &output[trim_samples..]
        } else {
            &output[..]
        };
        // Full slice returned, not empty.
        assert_eq!(trimmed.len(), output.len());
    }
}
