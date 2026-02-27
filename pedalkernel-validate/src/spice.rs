//! ngspice integration for golden reference generation.
//!
//! This module provides functions to run ngspice simulations and extract
//! output for comparison with WDF results.
//!
//! # Requirements
//!
//! ngspice must be installed and available in PATH:
//! - macOS: `brew install ngspice`
//! - Ubuntu/Debian: `apt install ngspice`
//! - Windows: Download from ngspice.sourceforge.io
//!
//! # Example
//!
//! ```rust,ignore
//! use pedalkernel_validate::spice::{SpiceRunner, SpiceConfig};
//! use pedalkernel_validate::signals;
//!
//! let config = SpiceConfig {
//!     sample_rate: 96000,
//!     oversample: 4,
//! };
//!
//! let runner = SpiceRunner::new(config);
//!
//! // Generate input signal
//! let input = signals::sine(config.internal_rate(), 1000.0, 0.1, 1.0);
//!
//! // Run SPICE simulation
//! let output = runner.simulate(
//!     "circuits/nonlinear/diode_clipper.spice",
//!     &input,
//!     "v_out"
//! ).unwrap();
//! ```

use std::io::Write;
use std::path::Path;
use std::process::Command;
use tempfile::TempDir;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum SpiceError {
    #[error("ngspice not found in PATH. Install with: brew install ngspice (macOS) or apt install ngspice (Linux)")]
    NgspiceNotFound,
    #[error("ngspice execution failed: {0}")]
    ExecutionFailed(String),
    #[error("Failed to parse ngspice output: {0}")]
    ParseError(String),
    #[error("Circuit file not found: {0}")]
    CircuitNotFound(String),
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),
    #[error("Simulation did not converge")]
    ConvergenceFailed,
}

/// Configuration for SPICE simulations.
#[derive(Debug, Clone)]
pub struct SpiceConfig {
    /// Output sample rate in Hz.
    pub sample_rate: u32,
    /// Oversampling factor for internal simulation.
    pub oversample: u32,
}

impl Default for SpiceConfig {
    fn default() -> Self {
        Self {
            sample_rate: 96000,
            oversample: 4,
        }
    }
}

impl SpiceConfig {
    /// Internal simulation rate (sample_rate × oversample).
    pub fn internal_rate(&self) -> f64 {
        (self.sample_rate * self.oversample) as f64
    }

    /// Time step for simulation.
    pub fn timestep(&self) -> f64 {
        1.0 / self.internal_rate()
    }
}

/// Runner for ngspice simulations.
pub struct SpiceRunner {
    config: SpiceConfig,
}

impl SpiceRunner {
    /// Create a new SPICE runner with the given configuration.
    pub fn new(config: SpiceConfig) -> Self {
        Self { config }
    }

    /// Check if ngspice is available.
    pub fn check_ngspice() -> Result<String, SpiceError> {
        let output = Command::new("ngspice")
            .arg("--version")
            .output()
            .map_err(|_| SpiceError::NgspiceNotFound)?;

        if output.status.success() {
            let version = String::from_utf8_lossy(&output.stdout);
            // Extract version number from output
            let version_line = version.lines().next().unwrap_or("unknown");
            Ok(version_line.to_string())
        } else {
            Err(SpiceError::NgspiceNotFound)
        }
    }

    /// Run a SPICE simulation with the given input signal.
    ///
    /// # Arguments
    /// * `circuit_path` - Path to the .spice circuit file
    /// * `input` - Input signal samples at internal_rate
    /// * `output_node` - Node name to measure (e.g., "v_out")
    ///
    /// # Returns
    /// Output signal resampled to `sample_rate`
    pub fn simulate(
        &self,
        circuit_path: impl AsRef<Path>,
        input: &[f64],
        output_node: &str,
    ) -> Result<Vec<f64>, SpiceError> {
        let circuit_path = circuit_path.as_ref();
        if !circuit_path.exists() {
            return Err(SpiceError::CircuitNotFound(
                circuit_path.display().to_string(),
            ));
        }

        // Create temp directory for simulation files
        let tmpdir = TempDir::new()?;
        let pwl_path = tmpdir.path().join("input.pwl");
        let netlist_path = tmpdir.path().join("circuit.spice");
        let output_path = tmpdir.path().join("output.txt");

        // Write PWL input file
        self.write_pwl_file(input, &pwl_path)?;

        // Generate complete netlist
        let duration = input.len() as f64 / self.config.internal_rate();
        let netlist = self.generate_netlist(circuit_path, &pwl_path, duration, output_node, &output_path)?;

        std::fs::write(&netlist_path, &netlist)?;

        // Run ngspice
        let result = Command::new("ngspice")
            .args(["-b", netlist_path.to_str().unwrap()])
            .output()?;

        if !result.status.success() {
            let stderr = String::from_utf8_lossy(&result.stderr);
            // Look for convergence errors
            if stderr.contains("no convergence") || stderr.contains("timestep too small") {
                return Err(SpiceError::ConvergenceFailed);
            }
            return Err(SpiceError::ExecutionFailed(stderr.to_string()));
        }

        // Check if output file was created
        if !output_path.exists() {
            return Err(SpiceError::ExecutionFailed(
                "ngspice did not produce output file".to_string(),
            ));
        }

        // Parse output and resample
        let raw_output = self.parse_wrdata_output(&output_path)?;
        let resampled = self.resample_and_decimate(&raw_output, duration);

        Ok(resampled)
    }

    /// Write input signal as PWL (piecewise-linear) file.
    fn write_pwl_file(&self, signal: &[f64], path: &Path) -> Result<(), SpiceError> {
        let mut file = std::fs::File::create(path)?;
        let dt = self.config.timestep();

        for (i, &sample) in signal.iter().enumerate() {
            let t = i as f64 * dt;
            writeln!(file, "{:.12e} {:.12e}", t, sample)?;
        }

        Ok(())
    }

    /// Generate complete ngspice netlist from circuit template.
    fn generate_netlist(
        &self,
        circuit_path: &Path,
        pwl_path: &Path,
        duration: f64,
        output_node: &str,
        output_file: &Path,
    ) -> Result<String, SpiceError> {
        let circuit_body = std::fs::read_to_string(circuit_path)?;
        let timestep = self.config.timestep();

        let netlist = format!(
            r#"* PedalKernel Golden Reference Generation
* Circuit: {circuit_name}
* Internal rate: {rate} Hz (timestep: {timestep:.12e} s)
* Duration: {duration} s

.TITLE Golden Reference — {circuit_stem}

{circuit_body}

* Input source — driven by PWL file
VIN v_in 0 PWL file="{pwl_path}"

* Simulation control
.OPTIONS RELTOL=1e-6 ABSTOL=1e-12 VNTOL=1e-9
.OPTIONS METHOD=GEAR MAXORD=2
.OPTIONS ITL1=500 ITL2=200 ITL4=50
* Disable random number generation for determinism
.OPTIONS SEED=42

.TRAN {timestep:.12e} {duration:.12e} 0 {timestep:.12e} UIC

.CONTROL
  set filetype=ascii
  run
  wrdata {output_file} {output_node}
  quit
.ENDC

.END
"#,
            circuit_name = circuit_path.file_name().unwrap().to_string_lossy(),
            circuit_stem = circuit_path.file_stem().unwrap().to_string_lossy(),
            rate = self.config.internal_rate(),
            timestep = timestep,
            duration = duration,
            circuit_body = circuit_body,
            pwl_path = pwl_path.display(),
            output_file = output_file.display(),
            output_node = output_node,
        );

        Ok(netlist)
    }

    /// Parse wrdata output file (time, value columns).
    fn parse_wrdata_output(&self, path: &Path) -> Result<Vec<(f64, f64)>, SpiceError> {
        let contents = std::fs::read_to_string(path)?;
        let mut data = Vec::new();

        for line in contents.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('*') || line.starts_with('#') {
                continue;
            }

            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() >= 2 {
                let t: f64 = parts[0].parse().map_err(|e| {
                    SpiceError::ParseError(format!("Invalid time value: {}", e))
                })?;
                let v: f64 = parts[1].parse().map_err(|e| {
                    SpiceError::ParseError(format!("Invalid voltage value: {}", e))
                })?;
                data.push((t, v));
            }
        }

        if data.is_empty() {
            return Err(SpiceError::ParseError("No data points found".to_string()));
        }

        Ok(data)
    }

    /// Resample SPICE output to uniform grid and decimate.
    fn resample_and_decimate(&self, raw_data: &[(f64, f64)], duration: f64) -> Vec<f64> {
        let internal_rate = self.config.internal_rate();
        let n_internal = (duration * internal_rate) as usize;

        // Create uniform time grid at internal rate
        let mut uniform_output = Vec::with_capacity(n_internal);

        for i in 0..n_internal {
            let t = i as f64 / internal_rate;
            let v = self.interpolate(raw_data, t);
            uniform_output.push(v);
        }

        // Decimate to output sample rate
        if self.config.oversample > 1 {
            self.decimate(&uniform_output, self.config.oversample as usize)
        } else {
            uniform_output
        }
    }

    /// Linear interpolation at time t.
    fn interpolate(&self, data: &[(f64, f64)], t: f64) -> f64 {
        if data.is_empty() {
            return 0.0;
        }
        if t <= data[0].0 {
            return data[0].1;
        }
        if t >= data.last().unwrap().0 {
            return data.last().unwrap().1;
        }

        // Binary search for bracket
        let mut lo = 0;
        let mut hi = data.len() - 1;
        while hi - lo > 1 {
            let mid = (lo + hi) / 2;
            if data[mid].0 <= t {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        // Linear interpolation
        let (t0, v0) = data[lo];
        let (t1, v1) = data[hi];
        let alpha = (t - t0) / (t1 - t0);
        v0 + alpha * (v1 - v0)
    }

    /// Simple decimation by averaging.
    fn decimate(&self, signal: &[f64], factor: usize) -> Vec<f64> {
        signal
            .chunks(factor)
            .map(|chunk| chunk.iter().sum::<f64>() / chunk.len() as f64)
            .collect()
    }

    /// Get the configuration.
    pub fn config(&self) -> &SpiceConfig {
        &self.config
    }
}

/// Generate golden references for a test circuit.
///
/// This is a convenience function that runs SPICE for multiple signals
/// and saves the outputs as .npy files.
pub fn generate_golden(
    circuit_path: impl AsRef<Path>,
    signals: &[(&str, Vec<f64>)],
    output_dir: impl AsRef<Path>,
    config: SpiceConfig,
) -> Result<(), SpiceError> {
    let runner = SpiceRunner::new(config);
    let output_dir = output_dir.as_ref();

    std::fs::create_dir_all(output_dir)?;

    for (label, input) in signals {
        let output = runner.simulate(&circuit_path, input, "v_out")?;

        let output_path = output_dir.join(format!("{}.npy", label));
        crate::npy::write_f64(&output_path, &output)
            .map_err(|e| SpiceError::IoError(std::io::Error::new(
                std::io::ErrorKind::Other,
                e.to_string(),
            )))?;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_defaults_are_sensible() {
        let config = SpiceConfig::default();
        assert_eq!(config.sample_rate, 96000);
        assert_eq!(config.oversample, 4);
        assert_eq!(config.internal_rate(), 384000.0);
    }

    #[test]
    fn interpolation_works() {
        let runner = SpiceRunner::new(SpiceConfig::default());
        let data = vec![(0.0, 0.0), (1.0, 10.0), (2.0, 20.0)];

        assert!((runner.interpolate(&data, 0.5) - 5.0).abs() < 1e-10);
        assert!((runner.interpolate(&data, 1.5) - 15.0).abs() < 1e-10);
    }

    #[test]
    fn decimation_works() {
        let runner = SpiceRunner::new(SpiceConfig::default());
        let signal = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        let decimated = runner.decimate(&signal, 4);

        assert_eq!(decimated.len(), 2);
        assert!((decimated[0] - 2.5).abs() < 1e-10); // (1+2+3+4)/4
        assert!((decimated[1] - 6.5).abs() < 1e-10); // (5+6+7+8)/4
    }
}
