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
use crate::report::{SignalResult, TestResult, SuiteResult};
use pedalkernel::compiler::CompiledPedal;
use pedalkernel::PedalProcessor;
use std::collections::HashMap;
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
    pub fn run_all(&self) -> Result<HashMap<String, SuiteResult>, RunnerError> {
        let mut results = HashMap::new();
        for (suite_name, suite) in &self.validation_config.suites {
            let suite_result = self.run_suite(suite_name, suite)?;
            results.insert(suite_name.clone(), suite_result);
        }
        Ok(results)
    }

    /// Run a specific suite by name.
    pub fn run_suite(&self, suite_name: &str, suite: &TestSuite) -> Result<SuiteResult, RunnerError> {
        let mut test_results = HashMap::new();
        let mut passed = 0;
        let mut failed = 0;

        for (test_name, test_case) in &suite.tests {
            match self.run_test(suite_name, test_name, test_case) {
                Ok(result) => {
                    if result.passed {
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
        let mut signal_results = vec![];
        let mut all_passed = true;

        for signal_config in &test_case.signals {
            let signal_result = self.run_signal(
                suite_name,
                test_name,
                signal_config,
                &mut pedal,
                effective_sr as f64,
                &test_case.pass_criteria,
            )?;

            if !signal_result.passed {
                all_passed = false;
            }
            signal_results.push(signal_result);
        }

        Ok(TestResult {
            passed: all_passed,
            error: None,
            signals: signal_results,
        })
    }

    /// Run a single signal through the circuit and compare to golden.
    #[allow(clippy::too_many_arguments)]
    fn run_signal(
        &self,
        suite_name: &str,
        test_name: &str,
        signal_config: &SignalConfig,
        pedal: &mut CompiledPedal,
        sample_rate: f64,
        pass_criteria: &PassCriteria,
    ) -> Result<SignalResult, RunnerError> {
        let signal_label = signal_config.label();

        // Generate input signal
        let spec = signal_config.to_spec();
        let input = spec.generate(sample_rate);

        // Process through WDF
        let output = self.process_signal(pedal, &input);

        // Save WDF output if configured
        if self.config.save_output {
            let output_path = self.config.output_dir
                .join(suite_name)
                .join(test_name)
                .join(format!("{}.npy", signal_label));
            npy::write_f64(&output_path, &output)?;
        }

        // Load golden reference
        let golden_path = self.config.golden_dir
            .join(suite_name)
            .join(test_name)
            .join(format!("{}.npy", signal_label));

        if !golden_path.exists() {
            // No golden reference - can't compare, but circuit ran
            return Ok(SignalResult {
                label: signal_label,
                passed: false,
                comparison: None,
                error: Some(format!("Golden reference not found: {:?}", golden_path)),
            });
        }

        let golden = npy::read_f64(&golden_path)?;

        // Compute comparison metrics
        let fundamental_hz = signal_config.fundamental_hz();
        let comparison = metrics::compare(&output, &golden, sample_rate, fundamental_hz);

        // Check pass/fail
        let passed = comparison.passes(pass_criteria);

        Ok(SignalResult {
            label: signal_label,
            passed,
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

        pedalkernel::compiler::compile_pedal(&pedal_def, effective_sr)
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

    let mut pedal = pedalkernel::compiler::compile_pedal(&pedal_def, sample_rate)
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
        Ok(metrics::compare(&output, &golden, sample_rate, Some(1000.0)))
    } else {
        // Self-comparison (useful for sanity check)
        Ok(metrics::compare(&output, &output, sample_rate, Some(1000.0)))
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
}
