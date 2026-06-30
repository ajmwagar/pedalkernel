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
        let netlist_path = tmpdir.path().join("circuit.spice");
        let output_path = tmpdir.path().join("output.txt");

        // Generate complete netlist with inline PWL
        let duration = input.len() as f64 / self.config.internal_rate();
        let netlist =
            self.generate_netlist(circuit_path, input, duration, output_node, &output_path)?;
        let sample_time_offset = if single_sample_impulse(input).is_some() {
            0.5 * self.config.timestep()
        } else {
            0.0
        };

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
        let resampled = self.resample_and_decimate(&raw_output, duration, sample_time_offset);

        Ok(resampled)
    }

    /// Strip standalone-deck elements that conflict with harness injection.
    ///
    /// New SPICE decks are authored as self-contained files (they can be run
    /// with `ngspice deck.spice` for standalone verification) but the harness
    /// composes them into a fresh netlist with its own VIN source, .TRAN, and
    /// .CONTROL block. Strip the standalone-only lines so the composition
    /// doesn't produce duplicate element definitions.
    ///
    /// Stripped:
    /// - Lines beginning with `VIN ` or `VIN\t` (the standalone AC source)
    /// - `.TRAN` lines
    /// - `.OP` lines
    /// - `.MEAS` lines
    /// - `.FOUR` lines
    /// - `.CONTROL` … `.ENDC` blocks (including contents)
    /// - Bare `.END` lines (not inside a subcircuit `.ENDS`)
    fn strip_standalone_elements(body: &str) -> String {
        let mut out = Vec::with_capacity(body.len());
        let mut in_control = false;
        for line in body.lines() {
            let upper = line.trim().to_uppercase();
            if upper == ".CONTROL" {
                in_control = true;
                continue;
            }
            if in_control {
                if upper == ".ENDC" {
                    in_control = false;
                }
                continue;
            }
            // Skip standalone-only directives. .OPTIONS is stripped so the
            // harness can set its own consistent convergence knobs; per-deck
            // convergence tweaks belong as .IC initial-condition statements
            // instead (those survive and help UIC startup).
            if upper.starts_with("VIN ")
                || upper.starts_with("VIN\t")
                || upper.starts_with(".TRAN")
                || upper.starts_with(".OP")
                || upper.starts_with(".MEAS")
                || upper.starts_with(".FOUR")
                || upper.starts_with(".OPTIONS")
                || upper == ".END"
            {
                continue;
            }
            out.push(line);
        }
        out.join("\n")
    }

    /// Generate complete ngspice netlist from circuit template.
    fn generate_netlist(
        &self,
        circuit_path: &Path,
        input_signal: &[f64],
        duration: f64,
        output_node: &str,
        output_file: &Path,
    ) -> Result<String, SpiceError> {
        let raw_body = std::fs::read_to_string(circuit_path)?;
        // Strip standalone-deck elements before embedding in the harness netlist
        let circuit_body = Self::strip_standalone_elements(&raw_body);
        let timestep = self.config.timestep();

        // Generate inline PWL data (more portable than file= syntax)
        let pwl_data = self.generate_pwl_inline(input_signal);

        let netlist = format!(
            r#"* PedalKernel Golden Reference Generation
* Circuit: {circuit_name}
* Internal rate: {rate} Hz (timestep: {timestep:.12e} s)
* Duration: {duration} s

.TITLE Golden Reference — {circuit_stem}

{circuit_body}

* Input source — inline PWL data
VIN v_in 0 PWL({pwl_data})

* Simulation control
* RELTOL=5e-3 / ABSTOL=1e-9: balances accuracy vs convergence for stiff
* nonlinear circuits (diodes, BJTs, triodes, pentodes). Tighter tolerances
* can cause non-convergence in high-current tube stages.
.OPTIONS RELTOL=5e-3 ABSTOL=1e-9 VNTOL=1e-6
.OPTIONS METHOD=GEAR MAXORD=2
.OPTIONS ITL1=500 ITL2=200 ITL4=200 DELMAX=50u

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
            pwl_data = pwl_data,
            output_file = output_file.display(),
            output_node = output_node,
        );

        Ok(netlist)
    }

    /// Generate inline PWL string (decimated to reduce netlist size).
    fn generate_pwl_inline(&self, signal: &[f64]) -> String {
        let dt = self.config.timestep();

        if let Some(amplitude) = single_sample_impulse(signal) {
            return generate_sample_hold_impulse_pwl(amplitude, signal.len(), dt);
        }

        // Keep shorter validation signals sample-exact. Very long signals are
        // decimated to keep generated netlists practical, but their tail stays
        // exact so high-frequency sweep endings do not pick up PWL interpolation
        // residuals.
        let max_pwl_points = 200_000;
        let exact_tail_samples = 65_536;
        let decimate_factor = if signal.len() <= max_pwl_points {
            1
        } else {
            signal.len().div_ceil(max_pwl_points)
        };

        let mut pwl_points = Vec::new();
        let exact_tail_start = if decimate_factor > 1 {
            signal.len().saturating_sub(exact_tail_samples)
        } else {
            signal.len()
        };

        for i in (0..exact_tail_start).step_by(decimate_factor) {
            let t = i as f64 * dt;
            pwl_points.push(format!("{:.9e} {:.9e}", t, signal[i]));
        }

        if decimate_factor > 1 {
            for (i, &sample) in signal.iter().enumerate().skip(exact_tail_start) {
                let t = i as f64 * dt;
                pwl_points.push(format!("{:.9e} {:.9e}", t, sample));
            }
        }

        pwl_points.join(" ")
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
                let t: f64 = parts[0]
                    .parse()
                    .map_err(|e| SpiceError::ParseError(format!("Invalid time value: {}", e)))?;
                let v: f64 = parts[1]
                    .parse()
                    .map_err(|e| SpiceError::ParseError(format!("Invalid voltage value: {}", e)))?;
                data.push((t, v));
            }
        }

        if data.is_empty() {
            return Err(SpiceError::ParseError("No data points found".to_string()));
        }

        Ok(data)
    }

    /// Resample SPICE output to uniform grid at internal rate.
    ///
    /// Returns samples at the full internal rate (sample_rate × oversample)
    /// to match the WDF runner's output rate for sample-by-sample comparison.
    fn resample_and_decimate(
        &self,
        raw_data: &[(f64, f64)],
        duration: f64,
        time_offset: f64,
    ) -> Vec<f64> {
        let internal_rate = self.config.internal_rate();
        let n_internal = (duration * internal_rate) as usize;

        // Create uniform time grid at internal rate
        let mut uniform_output = Vec::with_capacity(n_internal);

        for i in 0..n_internal {
            let t = i as f64 / internal_rate + time_offset;
            let v = self.interpolate(raw_data, t);
            uniform_output.push(v);
        }

        uniform_output
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

    /// Get the configuration.
    pub fn config(&self) -> &SpiceConfig {
        &self.config
    }

    /// Decimate an internal-rate signal (`sample_rate × oversample`) down to the
    /// base `sample_rate` by taking every `oversample`-th sample.
    ///
    /// [`simulate`] returns at the INTERNAL rate (to match the oversampled main
    /// runner).  The standalone `*_spice.rs` tests instead compare against a WDF
    /// processor compiled at the BASE `sample_rate`, so their golden must be at
    /// the base rate too — otherwise a 1 kHz golden tone carries `oversample×`
    /// the samples-per-period of the WDF tone and the 1:1 comparison lines up two
    /// signals at different rates (the bug this fixes).
    pub fn decimate_to_base(&self, internal: &[f64]) -> Vec<f64> {
        let k = self.config.oversample.max(1) as usize;
        internal.iter().step_by(k).copied().collect()
    }

    /// Run a SETTLED steady-state simulation and return the MEASUREMENT window at
    /// the base `sample_rate`.
    ///
    /// `input_internal` is the full stimulus at the internal rate covering
    /// `settle_s + measure_s` seconds.  The circuit is simulated over that whole
    /// span so coupling caps / bias networks reach steady state; the first
    /// `settle_s` is then discarded and the remaining tail is decimated to the
    /// base `sample_rate`.  The settle boundary is floored to a multiple of
    /// `oversample` so the decimated grid stays phase-aligned with a WDF
    /// processor warmed up by the same `settle_s`.
    ///
    /// The returned golden is at the base `sample_rate`: `len ≈ measure_s ×
    /// sample_rate`, and a tone of frequency `f` has `sample_rate / f`
    /// samples per period.
    pub fn simulate_settled_window(
        &self,
        circuit_path: impl AsRef<Path>,
        input_internal: &[f64],
        output_node: &str,
        settle_s: f64,
    ) -> Result<Vec<f64>, SpiceError> {
        let internal = self.simulate(circuit_path, input_internal, output_node)?;
        let k = self.config.oversample.max(1) as usize;
        // Floor the settle boundary to a multiple of the oversample factor so the
        // decimation phase is identical to a base-rate WDF warmup.
        let mut settle_internal = (settle_s * self.config.internal_rate()).round() as usize;
        settle_internal -= settle_internal % k;
        let tail: &[f64] = if settle_internal < internal.len() {
            &internal[settle_internal..]
        } else {
            &internal[..]
        };
        Ok(self.decimate_to_base(tail))
    }
}

fn single_sample_impulse(signal: &[f64]) -> Option<f64> {
    let (&first, rest) = signal.split_first()?;
    if first == 0.0 {
        return None;
    }

    if rest.iter().all(|&sample| sample == 0.0) {
        Some(first)
    } else {
        None
    }
}

fn generate_sample_hold_impulse_pwl(amplitude: f64, len: usize, dt: f64) -> String {
    let end_time = len.saturating_sub(1) as f64 * dt;
    let drop_time = dt;

    let mut pwl_points = vec![
        format!("{:.9e} {:.9e}", 0.0, amplitude),
        format!("{:.9e} {:.9e}", drop_time, amplitude),
        format!("{:.9e} {:.9e}", drop_time, 0.0),
    ];

    if end_time > drop_time {
        pwl_points.push(format!("{:.9e} {:.9e}", end_time, 0.0));
    }

    pwl_points.join(" ")
}

// ============================================================================
// DC operating-point (.op) extraction — the bias-accuracy golden source
// ============================================================================

/// One device's DC operating point as reported by ngspice's `.op` / `show`.
///
/// `vce`/`ic` are derived for BJTs from the `show` table:
/// `vce = vbe − vbc`, `ic` is the collector current row. All voltages in volts,
/// currents in amps. For PNP devices ngspice reports the junction-voltage
/// magnitudes (positive `vbe`) with a negative `ic`; the values are stored
/// verbatim so the caller controls sign convention.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq)]
pub struct SpiceDeviceOp {
    /// Device reference, uppercased (e.g. `Q1`, `Q3`).
    pub device: String,
    /// Model name (e.g. `QBC184C`, `QAC128`).
    pub model: String,
    /// Base–emitter voltage `vbe`, volts.
    pub vbe: f64,
    /// Collector–emitter voltage `vce = vbe − vbc`, volts.
    pub vce: f64,
    /// Collector current `ic`, amps.
    pub ic: f64,
    /// Base current `ib`, amps.
    pub ib: f64,
    /// Transconductance `gm`, siemens.
    pub gm: f64,
}

/// A full ngspice `.op` snapshot: per-device operating points plus node voltages.
///
/// This is the **ngspice-derived golden** for the bias-accuracy dimension. It is
/// serialized to `golden/op/<circuit>.json` and is never WDF-bootstrapped.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct SpiceOpSnapshot {
    /// Source deck name (file stem).
    pub circuit: String,
    /// Per-device operating points (BJTs), keyed by uppercased device ref.
    pub devices: Vec<SpiceDeviceOp>,
    /// DC node voltages (node name → volts).
    pub nodes: std::collections::BTreeMap<String, f64>,
}

impl SpiceRunner {
    /// Run ngspice `.op` on a standalone deck and capture the DC operating point.
    ///
    /// Composes the deck the same way [`Self::simulate`] does (stripping
    /// standalone control/analysis directives), injects a quiescent `VIN v_in 0
    /// DC 0` source, and runs an `.op` followed by `show` (per-device params) and
    /// `print all` (node voltages). The `show` table is parsed into
    /// [`SpiceDeviceOp`] records (`vce = vbe − vbc`, `ic` from the row).
    ///
    /// This is read-only and ngspice-derived — the resulting snapshot is the
    /// golden the WDF DC bias is graded against.
    pub fn operating_point(
        &self,
        circuit_path: impl AsRef<Path>,
    ) -> Result<SpiceOpSnapshot, SpiceError> {
        let circuit_path = circuit_path.as_ref();
        if !circuit_path.exists() {
            return Err(SpiceError::CircuitNotFound(
                circuit_path.display().to_string(),
            ));
        }
        let raw_body = std::fs::read_to_string(circuit_path)?;
        let circuit_body = Self::strip_standalone_elements(&raw_body);

        let netlist = format!(
            r#"* PedalKernel DC operating-point extraction
.TITLE OP — {stem}

{circuit_body}

* Quiescent input (DC bias point, no signal)
VIN v_in 0 DC 0

.OPTIONS GMIN=1e-12 RELTOL=1e-4 ABSTOL=1e-12 VNTOL=1e-9

.CONTROL
  op
  show
  print all
  quit
.ENDC

.END
"#,
            stem = circuit_path.file_stem().unwrap().to_string_lossy(),
            circuit_body = circuit_body,
        );

        let tmpdir = TempDir::new()?;
        let netlist_path = tmpdir.path().join("op.spice");
        std::fs::write(&netlist_path, &netlist)?;

        let result = Command::new("ngspice")
            .args(["-b", netlist_path.to_str().unwrap()])
            .output()?;
        let stdout = String::from_utf8_lossy(&result.stdout);
        let stderr = String::from_utf8_lossy(&result.stderr);
        if stderr.contains("no convergence") || stdout.contains("no convergence") {
            return Err(SpiceError::ConvergenceFailed);
        }
        if !result.status.success() && stdout.trim().is_empty() {
            return Err(SpiceError::ExecutionFailed(stderr.to_string()));
        }

        let stem = circuit_path.file_stem().unwrap().to_string_lossy().to_string();
        parse_op_output(&stdout, stem)
    }
}

/// Parse the combined `show` + `print all` ngspice `.op` output.
fn parse_op_output(out: &str, circuit: String) -> Result<SpiceOpSnapshot, SpiceError> {
    // ── `show` device table ─────────────────────────────────────────────────
    // ngspice prints one block per device type. A `device <name> <name>...`
    // header names the columns; following `param val val ...` rows carry values.
    // We accumulate per-column maps and keep only BJT-like columns (those that
    // carry both `vbe` and `vbc`).
    let mut cur_names: Vec<String> = Vec::new();
    let mut cur_models: Vec<String> = Vec::new();
    let mut cols: std::collections::HashMap<String, f64> = std::collections::HashMap::new();
    // device -> (param -> value)
    let mut dev_params: Vec<(String, std::collections::HashMap<String, f64>)> = Vec::new();

    let flush = |names: &mut Vec<String>,
                 models: &mut Vec<String>,
                 dev_params: &mut Vec<(String, std::collections::HashMap<String, f64>)>,
                 collected: &mut std::collections::HashMap<String, f64>| {
        // collected keys are "<col_idx>::<param>"
        for (i, name) in names.iter().enumerate() {
            let mut p = std::collections::HashMap::new();
            for (k, v) in collected.iter() {
                if let Some(rest) = k.strip_prefix(&format!("{i}::")) {
                    p.insert(rest.to_string(), *v);
                }
            }
            if let Some(m) = models.get(i) {
                p.insert("__model_idx".to_string(), i as f64);
            }
            dev_params.push((name.clone(), p));
        }
        names.clear();
        models.clear();
        collected.clear();
    };

    let mut models_by_dev: std::collections::HashMap<String, String> = std::collections::HashMap::new();
    let mut nodes: std::collections::BTreeMap<String, f64> = std::collections::BTreeMap::new();

    for line in out.lines() {
        let t = line.trim();
        if t.is_empty() {
            continue;
        }
        let parts: Vec<&str> = t.split_whitespace().collect();

        // Node-voltage lines from `print all`: `name = 1.234e+00`
        if parts.len() == 3 && parts[1] == "=" {
            if let Ok(v) = parts[2].parse::<f64>() {
                // skip #branch currents — keep node voltages only
                if !parts[0].contains("#branch") {
                    nodes.insert(parts[0].to_string(), v);
                }
                continue;
            }
        }

        // `device q2 q1 q3` header begins a new device block.
        if parts[0] == "device" {
            // flush previous block
            if !cur_names.is_empty() {
                flush(&mut cur_names, &mut cur_models, &mut dev_params, &mut cols);
            }
            cur_names = parts[1..].iter().map(|s| s.to_uppercase()).collect();
            continue;
        }
        if parts[0] == "model" && !cur_names.is_empty() {
            cur_models = parts[1..].iter().map(|s| s.to_string()).collect();
            for (i, n) in cur_names.iter().enumerate() {
                if let Some(m) = cur_models.get(i) {
                    models_by_dev.insert(n.clone(), m.clone());
                }
            }
            continue;
        }
        // value row: `vbe 0.40 0.53 0.60`
        if !cur_names.is_empty() && parts.len() == cur_names.len() + 1 {
            let param = parts[0].to_string();
            let mut ok = true;
            let mut vals = Vec::with_capacity(cur_names.len());
            for raw in &parts[1..] {
                match raw.parse::<f64>() {
                    Ok(v) => vals.push(v),
                    Err(_) => {
                        ok = false;
                        break;
                    }
                }
            }
            if ok {
                for (i, v) in vals.into_iter().enumerate() {
                    cols.insert(format!("{i}::{param}"), v);
                }
            }
        }
    }
    if !cur_names.is_empty() {
        flush(&mut cur_names, &mut cur_models, &mut dev_params, &mut cols);
    }

    let mut devices = Vec::new();
    for (name, p) in dev_params {
        // Keep only transistor columns (carry both vbe and vbc).
        let (vbe, vbc) = match (p.get("vbe"), p.get("vbc")) {
            (Some(a), Some(b)) => (*a, *b),
            _ => continue,
        };
        let ic = p.get("ic").copied().unwrap_or(0.0);
        let ib = p.get("ib").copied().unwrap_or(0.0);
        let gm = p.get("gm").copied().unwrap_or(0.0);
        devices.push(SpiceDeviceOp {
            model: models_by_dev.get(&name).cloned().unwrap_or_default(),
            device: name,
            vbe,
            vce: vbe - vbc,
            ic,
            ib,
            gm,
        });
    }
    devices.sort_by(|a, b| a.device.cmp(&b.device));

    if devices.is_empty() && nodes.is_empty() {
        return Err(SpiceError::ParseError(
            "no devices or nodes parsed from .op output".to_string(),
        ));
    }

    Ok(SpiceOpSnapshot {
        circuit,
        devices,
        nodes,
    })
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
        crate::npy::write_f64(&output_path, &output).map_err(|e| {
            SpiceError::IoError(std::io::Error::new(
                std::io::ErrorKind::Other,
                e.to_string(),
            ))
        })?;
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
    fn pwl_generation_keeps_short_signals_sample_exact() {
        let runner = SpiceRunner::new(SpiceConfig::default());
        let signal: Vec<f64> = (0..12).map(|i| i as f64).collect();

        let pwl = runner.generate_pwl_inline(&signal);
        let pairs = pwl.split_whitespace().collect::<Vec<_>>();

        assert_eq!(pairs.len(), signal.len() * 2);
        assert_eq!(pairs[1], "0.000000000e0");
        assert_eq!(pairs[3], "1.000000000e0");
        assert_eq!(pairs[23], "1.100000000e1");
    }

    #[test]
    fn pwl_generation_keeps_long_signal_tail_exact() {
        let runner = SpiceRunner::new(SpiceConfig::default());
        let signal: Vec<f64> = (0..800_000).map(|i| i as f64).collect();

        let pwl = runner.generate_pwl_inline(&signal);
        let pairs = pwl.split_whitespace().collect::<Vec<_>>();
        let values: Vec<f64> = pairs
            .chunks_exact(2)
            .map(|pair| pair[1].parse::<f64>().unwrap())
            .collect();

        assert!(
            values.len() < signal.len(),
            "long signal should still be decimated"
        );
        assert!(values.windows(2).any(|w| (w[1] - w[0]).abs() > 1.0));
        assert_eq!(values[values.len() - 3], 799_997.0);
        assert_eq!(values[values.len() - 2], 799_998.0);
        assert_eq!(values[values.len() - 1], 799_999.0);
    }

    #[test]
    fn pwl_generation_uses_sample_hold_for_impulses() {
        let runner = SpiceRunner::new(SpiceConfig::default());
        let mut signal = vec![0.0; 8];
        signal[0] = 2.0;

        let pwl = runner.generate_pwl_inline(&signal);
        let pairs = pwl.split_whitespace().collect::<Vec<_>>();

        assert_eq!(pairs.len(), 8);
        assert_eq!(pairs[1], "2.000000000e0");
        assert_eq!(pairs[3], "2.000000000e0");
        assert_eq!(pairs[5], "0.000000000e0");
        assert_eq!(pairs[6], "1.822916667e-5");
        assert_eq!(pairs[7], "0.000000000e0");
    }
}
