//! Validation report generation and display.
//!
//! This module provides types for structured validation results and functions
//! to generate human-readable output and JSON reports.
//!
//! # Report Structure
//!
//! - [`ValidationReport`] - Top-level report containing all results
//!   - [`SuiteResult`] - Results for a test suite (e.g., "linear", "nonlinear")
//!     - [`TestResult`] - Results for a single test case
//!       - [`SignalResult`] - Results for each test signal
//!         - [`ComparisonMetrics`] - Detailed metric values
//!
//! # Example
//!
//! ```rust,ignore
//! use pedalkernel_validate::report::ValidationReport;
//! use std::collections::HashMap;
//!
//! // After running tests, create a report
//! let report = ValidationReport::new(suite_results, 96000, 4);
//!
//! // Print to terminal
//! report.print_summary();
//!
//! // Print detailed metrics table
//! report.print_detailed();
//!
//! // Save as JSON
//! report.save_json("report.json").unwrap();
//! ```
//!
//! # JSON Format
//!
//! The JSON output includes:
//! - Timestamp and git commit (if available)
//! - Sample rate and oversampling settings
//! - Per-suite and per-test results
//! - Summary statistics (pass/fail counts, pass rate)

use crate::config::ValidationProfile;
use crate::metrics::ComparisonResult;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::path::Path;

/// Full validation report.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationReport {
    /// Timestamp of the validation run.
    pub timestamp: String,
    /// Git commit hash (if available).
    pub git_commit: Option<String>,
    /// Sample rate used.
    pub sample_rate: u32,
    /// Oversampling factor.
    pub oversample: u32,
    /// Suite results.
    pub suites: BTreeMap<String, SuiteResult>,
    /// Summary statistics.
    pub summary: ReportSummary,
}

/// Summary of all test results.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReportSummary {
    /// Number of tests counted by the gate (passed + failed). PENDING tests are
    /// EXCLUDED from this total so the pass-rate denominator is unaffected by a
    /// committed-but-not-yet-generated reference.
    pub total_tests: usize,
    pub passed: usize,
    pub failed: usize,
    pub skipped: usize,
    /// Tests in the PENDING state (golden not generated, `pending_reference`).
    /// Excluded from both `passed` and `total_tests`.
    #[serde(default)]
    pub pending: usize,
    pub pass_rate: f64,
    /// Gate summary split by validation profile.
    #[serde(default)]
    pub profiles: BTreeMap<String, ProfileSummary>,
}

/// Summary of test results for one validation profile.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ProfileSummary {
    pub total_tests: usize,
    pub passed: usize,
    pub failed: usize,
    #[serde(default)]
    pub pending: usize,
    pub pass_rate: f64,
}

/// Result for a single test suite.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SuiteResult {
    pub description: String,
    pub passed: usize,
    pub failed: usize,
    /// Pending tests in this suite (golden not generated). Excluded from the
    /// gate's passed/total counts.
    #[serde(default)]
    pub pending: usize,
    pub tests: BTreeMap<String, TestResult>,
}

/// Result for a single test case.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestResult {
    /// Validation intent/profile used to interpret pass/fail.
    #[serde(default)]
    pub profile: ValidationProfile,
    pub passed: bool,
    /// `true` when the test is a pending reference (golden missing + the test
    /// is flagged `pending_reference`). A pending test is neither passed nor
    /// failed; it is excluded from the gate denominator entirely.
    #[serde(default)]
    pub pending: bool,
    pub error: Option<String>,
    pub signals: Vec<SignalResult>,
}

/// Result for a single signal within a test.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignalResult {
    pub label: String,
    pub passed: bool,
    /// `true` when this signal's golden is missing and the test is pending.
    #[serde(default)]
    pub pending: bool,
    pub comparison: Option<ComparisonMetrics>,
    pub error: Option<String>,
}

/// Serializable version of ComparisonResult.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComparisonMetrics {
    pub normalized_rms_error_db: f64,
    pub peak_error_db: f64,
    pub thd_error_db: Option<f64>,
    /// THD+N error (includes broadband noise/intermod). None when fundamental
    /// frequency is not provided.  `#[serde(default)]` for forward compat.
    #[serde(default)]
    pub thd_plus_n_error_db: Option<f64>,
    /// Maximum per-harmonic magnitude error in dB.  None when fundamental
    /// frequency is not provided.  `#[serde(default)]` for forward compat.
    #[serde(default)]
    pub harmonic_mag_error_db: Option<f64>,
    /// Even/odd ratio error in dB.  None when fundamental frequency is not
    /// provided.  `#[serde(default)]` for forward compat.
    #[serde(default)]
    pub even_odd_ratio_error_db: Option<f64>,
    /// Audio-band spectral error (capped at the audio Nyquist) — the gating value.
    pub spectral_error_db: f64,
    /// Full-band (raw) spectral error up to the data Nyquist. Informational.
    /// `#[serde(default)]` so older reports without this field still deserialize.
    #[serde(default)]
    pub spectral_error_full_db: f64,
    pub even_odd_ratio_db: Option<f64>,
    pub dc_drift_mv: Option<f64>,
}

impl From<ComparisonResult> for ComparisonMetrics {
    fn from(cr: ComparisonResult) -> Self {
        Self {
            normalized_rms_error_db: cr.normalized_rms_error_db,
            peak_error_db: cr.peak_error_db,
            thd_error_db: cr.thd_error_db,
            thd_plus_n_error_db: cr.thd_plus_n_error_db,
            harmonic_mag_error_db: cr.harmonic_mag_error_db,
            even_odd_ratio_error_db: cr.even_odd_ratio_error_db,
            spectral_error_db: cr.spectral_error_db,
            spectral_error_full_db: cr.spectral_error_full_db,
            even_odd_ratio_db: cr.even_odd_ratio_db,
            dc_drift_mv: cr.dc_drift_mv,
        }
    }
}

impl ValidationReport {
    /// Create a new report from suite results.
    pub fn new(suites: BTreeMap<String, SuiteResult>, sample_rate: u32, oversample: u32) -> Self {
        let mut total = 0;
        let mut passed = 0;
        let mut failed = 0;
        let mut pending = 0;

        for suite in suites.values() {
            // PENDING tests are excluded from the gate total entirely, so the
            // pass-rate denominator does not move when a not-yet-generated
            // reference is committed.
            total += suite.passed + suite.failed;
            passed += suite.passed;
            failed += suite.failed;
            pending += suite.pending;
        }

        let mut profiles: BTreeMap<String, ProfileSummary> = BTreeMap::new();
        for suite in suites.values() {
            for test in suite.tests.values() {
                let bucket = profiles
                    .entry(test.profile.as_str().to_string())
                    .or_default();
                if test.pending {
                    bucket.pending += 1;
                } else {
                    bucket.total_tests += 1;
                    if test.passed {
                        bucket.passed += 1;
                    } else {
                        bucket.failed += 1;
                    }
                }
            }
        }
        for bucket in profiles.values_mut() {
            bucket.pass_rate = if bucket.total_tests > 0 {
                bucket.passed as f64 / bucket.total_tests as f64
            } else {
                0.0
            };
        }

        let pass_rate = if total > 0 {
            passed as f64 / total as f64
        } else {
            0.0
        };

        Self {
            timestamp: chrono_lite_timestamp(),
            git_commit: get_git_commit(),
            sample_rate,
            oversample,
            suites,
            summary: ReportSummary {
                total_tests: total,
                passed,
                failed,
                skipped: 0,
                pending,
                pass_rate,
                profiles,
            },
        }
    }

    /// Save report to JSON file.
    pub fn save_json(&self, path: impl AsRef<Path>) -> Result<(), std::io::Error> {
        let path = path.as_ref();
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let json = serde_json::to_string_pretty(self)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;
        std::fs::write(path, json)
    }

    /// Print human-readable summary to terminal.
    pub fn print_summary(&self) {
        use colored::Colorize;

        println!("\n{}", "═".repeat(60).bold());
        println!("{}", " PEDALKERNEL VALIDATION REPORT ".bold().on_blue());
        println!("{}", "═".repeat(60).bold());

        if let Some(ref commit) = self.git_commit {
            println!("Git commit: {}", commit.dimmed());
        }
        println!("Timestamp:  {}", self.timestamp.dimmed());
        println!(
            "Config:     {}Hz × {}x oversample",
            self.sample_rate, self.oversample
        );
        if !self.summary.profiles.is_empty() {
            let buckets: Vec<String> = self
                .summary
                .profiles
                .iter()
                .map(|(name, summary)| {
                    if summary.pending > 0 {
                        format!(
                            "{} {}/{} ({} pending)",
                            name, summary.passed, summary.total_tests, summary.pending
                        )
                    } else {
                        format!("{} {}/{}", name, summary.passed, summary.total_tests)
                    }
                })
                .collect();
            println!("Profiles:   {}", buckets.join(" | ").dimmed());
        }
        println!();

        for (suite_name, suite) in &self.suites {
            let status = if suite.failed == 0 {
                "PASS".green().bold()
            } else {
                "FAIL".red().bold()
            };

            let pend_note = if suite.pending > 0 {
                format!(" [{} pending]", suite.pending)
            } else {
                String::new()
            };
            println!(
                "[{}] {} - {} ({}/{}){}",
                status,
                suite_name.bold(),
                suite.description.dimmed(),
                suite.passed,
                suite.passed + suite.failed,
                pend_note.yellow()
            );

            for (test_name, test) in &suite.tests {
                if test.pending {
                    // PENDING: golden not generated yet. Printed clearly and
                    // excluded from the pass/fail counts above.
                    let reason = test
                        .error
                        .clone()
                        .unwrap_or_else(|| "golden not generated".to_string());
                    println!(
                        "  {} {} ({})",
                        "[PEND]".yellow().bold(),
                        test_name,
                        reason.dimmed()
                    );
                    continue;
                }

                let test_status = if test.passed {
                    "✓".green()
                } else {
                    "✗".red()
                };

                println!(
                    "  {} {} [{}]",
                    test_status,
                    test_name,
                    test.profile.as_str().dimmed()
                );

                if let Some(ref err) = test.error {
                    println!("    {} {}", "Error:".red(), err);
                }

                for signal in &test.signals {
                    if let Some(ref comp) = signal.comparison {
                        let signal_status = if signal.passed {
                            "✓".green()
                        } else {
                            "✗".red()
                        };
                        println!(
                            "    {} {} | RMS: {:.1}dB | Peak: {:.1}dB | Spectral: {:.1}dB",
                            signal_status,
                            signal.label.dimmed(),
                            comp.normalized_rms_error_db,
                            comp.peak_error_db,
                            comp.spectral_error_db
                        );
                        if let Some(thd) = comp.thd_error_db {
                            println!("      THD error: {:.2}dB", thd);
                        }
                    } else if let Some(ref err) = signal.error {
                        println!("    {} {} - {}", "⚠".yellow(), signal.label.dimmed(), err);
                    }
                }
            }
            println!();
        }

        println!("{}", "─".repeat(60));
        let overall_status = if self.summary.failed == 0 {
            "ALL TESTS PASSED".green().bold()
        } else {
            format!("{} TESTS FAILED", self.summary.failed).red().bold()
        };
        let pend_suffix = if self.summary.pending > 0 {
            format!(" | {} pending", self.summary.pending)
        } else {
            String::new()
        };
        println!(
            "{} | {}/{} passed ({:.1}%){}",
            overall_status,
            self.summary.passed,
            self.summary.total_tests,
            self.summary.pass_rate * 100.0,
            pend_suffix.yellow()
        );
        println!("{}\n", "═".repeat(60).bold());
    }

    /// Print detailed metrics table.
    pub fn print_detailed(&self) {
        use tabled::{Table, Tabled};

        #[derive(Tabled)]
        struct MetricRow {
            suite: String,
            test: String,
            signal: String,
            profile: String,
            #[tabled(rename = "RMS (dB)")]
            rms_db: String,
            #[tabled(rename = "Peak (dB)")]
            peak_db: String,
            #[tabled(rename = "THD Err")]
            thd_err: String,
            #[tabled(rename = "Spectral")]
            spectral: String,
            status: String,
        }

        let mut rows = vec![];

        for (suite_name, suite) in &self.suites {
            for (test_name, test) in &suite.tests {
                if test.pending {
                    rows.push(MetricRow {
                        suite: suite_name.clone(),
                        test: test_name.clone(),
                        signal: "-".to_string(),
                        profile: test.profile.as_str().to_string(),
                        rms_db: "-".to_string(),
                        peak_db: "-".to_string(),
                        thd_err: "-".to_string(),
                        spectral: "-".to_string(),
                        status: "PEND".to_string(),
                    });
                    continue;
                }
                for signal in &test.signals {
                    let (rms, peak, thd, spectral) = if let Some(ref c) = signal.comparison {
                        (
                            format!("{:.1}", c.normalized_rms_error_db),
                            format!("{:.1}", c.peak_error_db),
                            c.thd_error_db
                                .map(|t| format!("{:.2}", t))
                                .unwrap_or_else(|| "-".to_string()),
                            format!("{:.1}", c.spectral_error_db),
                        )
                    } else {
                        (
                            "-".to_string(),
                            "-".to_string(),
                            "-".to_string(),
                            "-".to_string(),
                        )
                    };

                    rows.push(MetricRow {
                        suite: suite_name.clone(),
                        test: test_name.clone(),
                        signal: signal.label.clone(),
                        profile: test.profile.as_str().to_string(),
                        rms_db: rms,
                        peak_db: peak,
                        thd_err: thd,
                        spectral: spectral,
                        status: if signal.passed {
                            "PASS".to_string()
                        } else {
                            "FAIL".to_string()
                        },
                    });
                }
            }
        }

        if !rows.is_empty() {
            let table = Table::new(rows);
            println!("\nDetailed Metrics:\n{}", table);
        }
    }
}

// ============================================================================
// Boundary-load decision table (compile-time output-boundary analysis)
// ============================================================================

use pedalkernel_rt::processor::{
    BoundaryLoadBinding, BoundaryLoadDisposition, BoundaryLoadSummary,
};

/// Engineering-notation formatter for component values (`10µF`, `10kΩ`).
fn si_value(value: f64, unit: &str) -> String {
    if !value.is_finite() {
        return format!("{value}{unit}");
    }
    let a = value.abs();
    let (scale, prefix) = if a == 0.0 {
        (1.0, "")
    } else if a >= 1e6 {
        (1e-6, "M")
    } else if a >= 1e3 {
        (1e-3, "k")
    } else if a >= 1.0 {
        (1.0, "")
    } else if a >= 1e-3 {
        (1e3, "m")
    } else if a >= 1e-6 {
        (1e6, "µ")
    } else if a >= 1e-9 {
        (1e9, "n")
    } else {
        (1e12, "p")
    };
    let scaled = value * scale;
    let s = if (scaled - scaled.round()).abs() < 5e-3 {
        format!("{}", scaled.round() as i64)
    } else {
        format!("{scaled:.2}")
    };
    format!("{s}{prefix}{unit}")
}

/// Compact human-readable boundary-load MODEL summary — kind + element values +
/// component ids, e.g. `SeriesCapIntoLoad{10µF → 10kΩ; Cout,RL}`.
pub fn format_boundary_model(m: &BoundaryLoadSummary) -> String {
    match m {
        BoundaryLoadSummary::Unloaded => "Unloaded".to_string(),
        BoundaryLoadSummary::GroundedResistive {
            r_total,
            component_ids,
        } => format!(
            "GroundedResistive{{{}; {}}}",
            si_value(*r_total, "Ω"),
            component_ids.join(",")
        ),
        BoundaryLoadSummary::SeriesCapIntoLoad {
            c,
            r_total,
            component_ids,
        } => format!(
            "SeriesCapIntoLoad{{{} → {}; {}}}",
            si_value(*c, "F"),
            si_value(*r_total, "Ω"),
            component_ids.join(",")
        ),
        BoundaryLoadSummary::PassiveNetwork { component_ids } => {
            format!("PassiveNetwork{{{}}}", component_ids.join(","))
        }
        BoundaryLoadSummary::SeriesCapIntoPotLoad {
            c,
            pot_id,
            r_pot,
            r_fixed,
            component_ids,
        } => {
            let load = match r_fixed {
                Some(rf) => format!(
                    "{pot_id}:{} pot ∥ {}",
                    si_value(*r_pot, "Ω"),
                    si_value(*rf, "Ω")
                ),
                None => format!("{pot_id}:{} pot", si_value(*r_pot, "Ω")),
            };
            format!(
                "SeriesCapIntoPotLoad{{{} → {}; {}}}",
                si_value(*c, "F"),
                load,
                component_ids.join(",")
            )
        }
        BoundaryLoadSummary::PotDividerAtOut {
            pot_id,
            r_pot,
            component_ids,
        } => format!(
            "PotDividerAtOut{{{pot_id}:{} pot; {}}}",
            si_value(*r_pot, "Ω"),
            component_ids.join(",")
        ),
        BoundaryLoadSummary::CollectorChainIntoOut {
            c,
            pot_id,
            r_pot,
            r_tap_fixed,
            component_ids,
        } => format!(
            "CollectorChainIntoOut{{{} → {} tap ∥ {}:{} pot; {}}}",
            si_value(*c, "F"),
            si_value(*r_tap_fixed, "Ω"),
            pot_id,
            si_value(*r_pot, "Ω"),
            component_ids.join(",")
        ),
    }
}

/// Compact human-readable boundary-load DISPOSITION — `FusedUpstream` or
/// `Unloaded{reason}` (namespaced reasons: `env:*`, `gate:*`, `analysis:*`).
pub fn format_boundary_disposition(d: &BoundaryLoadDisposition) -> String {
    match d {
        BoundaryLoadDisposition::FusedUpstream => "FusedUpstream".to_string(),
        BoundaryLoadDisposition::Unloaded { reason } => format!("Unloaded{{{reason}}}"),
        BoundaryLoadDisposition::ReflectedThroughTransformer {
            turns_ratio,
            r_reflected,
        } => format!("ReflectedThroughTransformer{{n={turns_ratio}, r_reflected={r_reflected}}}"),
    }
}

/// WSL/NaN triage flag: `Some(flag)` when a circuit's OUTPUT boundary is left
/// electrically open — either an analyzed boundary whose disposition is
/// `Unloaded{reason}`, or an EMPTY table (no feedback group ever reached the
/// general-MNA call site, e.g. the whole pedal compiled via blockwise/other
/// paths — the boundary was never even analyzed). The two absence modes are
/// surfaced DISTINCTLY because they need different fixes (widen the policy vs
/// widen the analysis coverage).
///
/// WHY this flag rides next to the bias verdict: unloaded output boundaries and
/// marginal op-points travel together — the stage solves against an open
/// circuit, so the compile-time op-point never feels the load-current demand,
/// and the resulting bias sits closer to cutoff/rail than the real circuit's.
/// Live cases: Klon Centaur (renders +16 dBFS over on macOS and NaN-flushes to
/// silence on x86/WSL), ProCo RAT, Pultec — all with silently-unloaded output
/// boundaries. This table is the triage lever for the ~76 corpus failures.
pub fn unloaded_output_flag(loads: &[BoundaryLoadBinding]) -> Option<String> {
    if loads.iter().any(|b| {
        matches!(
            b.disposition,
            BoundaryLoadDisposition::FusedUpstream
                | BoundaryLoadDisposition::ReflectedThroughTransformer { .. }
        )
    }) {
        return None; // Output boundary load is fused into the upstream solve.
    }
    if loads.is_empty() {
        return Some(
            "⚠ OUTPUT BOUNDARY UNANALYZED: empty table — no feedback group reached the \
             general-MNA call site (blockwise/other compile paths)"
                .to_string(),
        );
    }
    let mut reasons: Vec<&str> = loads
        .iter()
        .filter_map(|b| match &b.disposition {
            BoundaryLoadDisposition::Unloaded { reason } => Some(reason.as_str()),
            BoundaryLoadDisposition::FusedUpstream
            | BoundaryLoadDisposition::ReflectedThroughTransformer { .. } => None,
        })
        .collect();
    reasons.sort_unstable();
    reasons.dedup();
    Some(format!(
        "⚠ OUTPUT BOUNDARY UNLOADED: {}",
        reasons.join(", ")
    ))
}

/// Render the compact per-circuit boundary-load decision table: one row per
/// analyzed stage output boundary (model + disposition), one explicit
/// `no-analysis (empty table)` row per circuit whose table is EMPTY (that
/// absence is itself a triage signal, distinct from `Unloaded{reason}`), and a
/// trailing flag line per circuit whose output boundary is open (see
/// [`unloaded_output_flag`]).
pub fn render_boundary_loads(circuits: &[(String, Vec<BoundaryLoadBinding>)]) -> String {
    use std::fmt::Write as _;
    use tabled::settings::Style;
    use tabled::{Table, Tabled};

    #[derive(Tabled)]
    struct BoundaryRow {
        circuit: String,
        stage: String,
        #[tabled(rename = "boundary node")]
        node: String,
        #[tabled(rename = "load model")]
        model: String,
        disposition: String,
    }

    let mut rows = Vec::new();
    for (circuit, loads) in circuits {
        if loads.is_empty() {
            rows.push(BoundaryRow {
                circuit: circuit.clone(),
                stage: "-".into(),
                node: "-".into(),
                model: "-".into(),
                disposition: "no-analysis (empty table)".into(),
            });
            continue;
        }
        for (i, b) in loads.iter().enumerate() {
            rows.push(BoundaryRow {
                circuit: if i == 0 { circuit.clone() } else { String::new() },
                stage: if b.upstream_stage == usize::MAX {
                    "?".into()
                } else {
                    b.upstream_stage.to_string()
                },
                node: b.boundary_node.to_string(),
                model: format_boundary_model(&b.model),
                disposition: format_boundary_disposition(&b.disposition),
            });
        }
    }

    let mut out = String::new();
    out.push_str(
        "\n────────────────── BOUNDARY-LOAD DECISION TABLE (compile-time) ──────────────────\n  \
         What hangs downstream of each stage's OUTPUT boundary and what the compiler did.\n  \
         `Unloaded{reason}` = analyzed, declined (boundary left open — stage solves into an\n  \
         open circuit). `no-analysis (empty table)` = no feedback group reached the\n  \
         general-MNA call site at all (blockwise/other paths) — a DISTINCT triage signal.\n",
    );
    if !rows.is_empty() {
        let mut table = Table::new(rows);
        table.with(Style::rounded());
        let _ = writeln!(out, "{}", table);
    }
    for (circuit, loads) in circuits {
        if let Some(flag) = unloaded_output_flag(loads) {
            let _ = writeln!(out, "  {circuit}: {flag}");
        }
    }
    out.push_str(
        "──────────────────────────────────────────────────────────────────────────────────\n",
    );
    out
}

// ============================================================================
// DC bias-accuracy dashboard section
// ============================================================================

/// Render the bias-accuracy table for a set of per-circuit op-point results.
///
/// Produces a `circuit × device × (Vbe / Vce / Ic: ours / ngspice / Δ)` table
/// plus, per circuit, the worst DC-balance residual `F(ngspice_op)` and the
/// formulation-vs-solver verdict. Returns the rendered string (so tests can
/// print AND assert on it).
///
/// `boundary_loads` maps circuit name → its compile-time boundary-load table
/// (`CompiledPedal::boundary_loads`). Every circuit whose OUTPUT boundary is
/// left open — `Unloaded{reason}` rows or an EMPTY table — gets a triage flag
/// rendered directly under its verdict line (see [`unloaded_output_flag`] for
/// the rationale: unloaded boundaries and marginal op-points travel together).
/// Circuits absent from the map (e.g. pro pedal not compiled) are not flagged.
pub fn render_bias_accuracy(
    results: &[crate::metrics::op_point::OpPointResult],
    criteria: &crate::metrics::op_point::OpPassCriteria,
    boundary_loads: &BTreeMap<String, Vec<BoundaryLoadBinding>>,
) -> String {
    use crate::metrics::op_point::Verdict;
    use std::fmt::Write as _;
    use tabled::settings::Style;
    use tabled::{Table, Tabled};

    #[derive(Tabled)]
    struct BiasRow {
        circuit: String,
        device: String,
        model: String,
        #[tabled(rename = "Vbe ours")]
        vbe_ours: String,
        #[tabled(rename = "Vbe ng")]
        vbe_ng: String,
        #[tabled(rename = "ΔVbe")]
        d_vbe: String,
        #[tabled(rename = "Vce ours")]
        vce_ours: String,
        #[tabled(rename = "Vce ng")]
        vce_ng: String,
        #[tabled(rename = "ΔVce")]
        d_vce: String,
        #[tabled(rename = "Ic ours")]
        ic_ours: String,
        #[tabled(rename = "Ic ng")]
        ic_ng: String,
        #[tabled(rename = "ΔIc%")]
        d_ic: String,
    }

    fn fmt_i(a: f64) -> String {
        if a.abs() >= 1e-3 {
            format!("{:.3}mA", a * 1e3)
        } else {
            format!("{:.1}µA", a * 1e6)
        }
    }

    let mut rows = Vec::new();
    for r in results {
        if r.devices.is_empty() {
            rows.push(BiasRow {
                circuit: r.circuit.clone(),
                device: "(no BJT devices)".to_string(),
                model: "-".into(),
                vbe_ours: "-".into(),
                vbe_ng: "-".into(),
                d_vbe: "-".into(),
                vce_ours: "-".into(),
                vce_ng: "-".into(),
                d_vce: "-".into(),
                ic_ours: "-".into(),
                ic_ng: "-".into(),
                d_ic: "-".into(),
            });
        }
        for (i, d) in r.devices.iter().enumerate() {
            let flag = if d.d_vbe.abs() > criteria.vbe_fail_v {
                "!"
            } else if d.d_vbe.abs() > criteria.vbe_warn_v {
                "~"
            } else {
                ""
            };
            rows.push(BiasRow {
                circuit: if i == 0 { r.circuit.clone() } else { String::new() },
                device: d.reference.clone(),
                model: d.model.clone(),
                vbe_ours: format!("{:+.4}", d.ours.0),
                vbe_ng: format!("{:+.4}", d.spice.0),
                d_vbe: format!("{:+.4}{}", d.d_vbe, flag),
                vce_ours: format!("{:+.3}", d.ours.1),
                vce_ng: format!("{:+.3}", d.spice.1),
                d_vce: format!("{:+.3}", d.d_vce),
                ic_ours: fmt_i(d.ours.2),
                ic_ng: fmt_i(d.spice.2),
                d_ic: format!("{:+.1}", d.d_ic_pct),
            });
        }
    }

    let mut out = String::new();
    out.push_str("\n══════════════════════ DC BIAS ACCURACY (WDF vs ngspice .op) ══════════════════════\n");
    out.push_str(
        "  MATCH = our settled DC bias vs ngspice .op (Layer B / solver-root).\n  \
         ΔVbe flags: '~' warn >",
    );
    let _ = write!(
        out,
        "{:.0}mV, '!' fail >{:.0}mV; |ΔVce| fail >{:.2}V; |ΔIc| fail >{:.0}%.\n",
        criteria.vbe_warn_v * 1e3,
        criteria.vbe_fail_v * 1e3,
        criteria.vce_fail_v,
        criteria.ic_fail_pct,
    );
    if !rows.is_empty() {
        let mut table = Table::new(rows);
        table.with(Style::rounded());
        let _ = write!(out, "{}\n", table);
    }

    // Per-circuit residual + verdict roll-up.
    out.push_str(
        "\n  RESIDUAL = F(ngspice_op): is ngspice's .op a fixed point of OUR DC equations?\n  \
         (residual ≈ 0 ⇒ formulation ok → Layer B; residual ≫ 0 ⇒ formulation bug → Layer A)\n",
    );
    for r in results {
        let mark = match r.verdict {
            Verdict::BiasOk => "✓",
            Verdict::FormulationOkSolverOff => "→B",
            Verdict::FormulationBug => "→A",
            Verdict::NoData => "·",
        };
        let _ = write!(
            out,
            "  [{mark}] {:<22} max|resid|={:>7.4}V  (full={:.2e}V, {} ports)  \
             max ΔVbe={:.4}V ΔVce={:.3}V ΔIc={:.1}%  ⇒ {}\n",
            r.circuit,
            r.max_abs_residual,
            r.max_abs_residual_full,
            r.n_residual_ports,
            r.max_abs_d_vbe,
            r.max_abs_d_vce,
            r.max_abs_d_ic_pct,
            r.verdict.label(),
        );
        // WSL/NaN triage: an open output boundary rides RIGHT NEXT TO the bias
        // verdict — the two travel together (see `unloaded_output_flag`).
        if let Some(loads) = boundary_loads.get(&r.circuit) {
            if let Some(flag) = unloaded_output_flag(loads) {
                let _ = writeln!(out, "        {flag}");
            }
        }
    }
    out.push_str("═══════════════════════════════════════════════════════════════════════════════════\n");
    out
}

// ============================================================================
// AC-signal accuracy dashboard section (the audio twin of the bias dashboard)
// ============================================================================

/// Render the AC-accuracy table for a set of per-circuit results.
///
/// Mirrors [`render_bias_accuracy`], but for the AC SIGNAL instead of the DC
/// operating point. One summary row per circuit — LEVEL (`gain@f`), the
/// gain-normalized SHAPE residual (rms + phase-immune spectral), THD (WDF vs
/// ngspice) and its Δ, the response tilt (max−min gain across the sweep), and the
/// verdict — followed by the per-harmonic breakdown and the full frequency sweep.
/// Returns the rendered string so tests can print AND assert on it.
pub fn render_ac_accuracy(
    results: &[crate::metrics::ac_accuracy::AcResult],
    th: &crate::metrics::ac_accuracy::AcThresholds,
) -> String {
    use crate::metrics::ac_accuracy::AcVerdict;
    use std::fmt::Write as _;
    use tabled::settings::Style;
    use tabled::{Table, Tabled};

    #[derive(Tabled)]
    struct AcRow {
        circuit: String,
        #[tabled(rename = "gain@f")]
        gain: String,
        #[tabled(rename = "shape rms")]
        shape_rms: String,
        #[tabled(rename = "shape spec")]
        shape_spec: String,
        #[tabled(rename = "THD wdf")]
        thd_wdf: String,
        #[tabled(rename = "THD ng")]
        thd_ng: String,
        #[tabled(rename = "ΔTHD")]
        d_thd: String,
        #[tabled(rename = "resp tilt")]
        tilt: String,
        verdict: String,
    }

    let mut rows = Vec::new();
    for r in results {
        let mark = match r.verdict {
            AcVerdict::Clean => "✓",
            AcVerdict::LevelOnly => "≈",
            AcVerdict::ShapeError => "!S",
            AcVerdict::HarmonicError => "!H",
            AcVerdict::ResponseError => "!R",
            AcVerdict::NoData => "·",
        };
        rows.push(AcRow {
            circuit: format!("{mark} {}", r.circuit),
            gain: format!("{:+.2}dB", r.gain_db),
            shape_rms: format!("{:.1}dB", r.shape_rms_db),
            shape_spec: format!("{:.1}dB", r.shape_spectral_db),
            thd_wdf: format!("{:.1}", r.thd_wdf_db),
            thd_ng: format!("{:.1}", r.thd_golden_db),
            d_thd: format!("{:.1}", r.thd_error_db()),
            tilt: format!("{:.2}dB", r.response_tilt_db),
            verdict: r.verdict.label().to_string(),
        });
    }

    let mut out = String::new();
    out.push_str("\n══════════════════════ AC SIGNAL ACCURACY (WDF vs ngspice golden) ══════════════════════\n");
    let _ = write!(
        out,
        "  LEVEL = ac_gain@f (scalar offset).  SHAPE = gain-normalized residual (rms time-domain,\n  \
         spec = phase-immune magnitude, in-band).  THD ratio is level-independent.  \n  \
         resp tilt = max−min gain across the sweep (≈0 ⇒ flat scalar; ≫0 ⇒ frequency-shaped).\n  \
         flags: '≈' level-only (shape clean); '!S' shape; '!H' harmonic; '!R' response.  \n  \
         thresholds: |gain|>{:.1}dB=level, shape_rms>{:.0}dB, shape_spec>{:.0}dB, ΔTHD>{:.0}dB, \
         harm>{:.0}dB, tilt>{:.0}dB.\n",
        th.gain_warn_db,
        th.shape_rms_fail_db,
        th.shape_spectral_fail_db,
        th.thd_error_fail_db,
        th.harmonic_fail_db,
        th.response_tilt_fail_db,
    );
    if !rows.is_empty() {
        let mut table = Table::new(rows);
        table.with(Style::rounded());
        let _ = write!(out, "{}\n", table);
    }

    // Per-harmonic breakdown (2nd–5th, dBc) + full sweep, per circuit.
    for r in results {
        let _ = write!(
            out,
            "\n  [{}] {} @ {:.0} Hz  —  {}\n",
            match r.verdict {
                AcVerdict::Clean => "✓",
                AcVerdict::LevelOnly => "≈",
                AcVerdict::ShapeError => "!S",
                AcVerdict::HarmonicError => "!H",
                AcVerdict::ResponseError => "!R",
                AcVerdict::NoData => "·",
            },
            r.circuit,
            r.test_freq_hz,
            r.verdict.label(),
        );
        out.push_str("    harmonics (dBc, wdf / ng / Δ): ");
        if r.harmonics.is_empty() {
            out.push_str("(none)");
        }
        for h in &r.harmonics {
            let note = if h.scored { "" } else { "·" };
            let _ = write!(
                out,
                "h{}={:.1}/{:.1}/{:.1}{}  ",
                h.order, h.wdf_dbc, h.golden_dbc, h.error_db, note
            );
        }
        out.push('\n');
        if !r.response.is_empty() {
            out.push_str("    response gain vs freq: ");
            for p in &r.response {
                let g = if p.gain_db.is_finite() {
                    format!("{:+.2}", p.gain_db)
                } else {
                    "n/a".to_string()
                };
                let _ = write!(out, "{:.0}Hz={}dB  ", p.freq_hz, g);
            }
            let _ = write!(out, " (tilt {:.2} dB)\n", r.response_tilt_db);
        }
    }
    out.push_str("══════════════════════════════════════════════════════════════════════════════════════\n");
    out
}

/// Get a simple timestamp without pulling in chrono.
fn chrono_lite_timestamp() -> String {
    use std::time::{SystemTime, UNIX_EPOCH};
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    format!("{}", duration.as_secs())
}

/// Try to get the current git commit hash.
fn get_git_commit() -> Option<String> {
    std::process::Command::new("git")
        .args(["rev-parse", "--short", "HEAD"])
        .output()
        .ok()
        .and_then(|o| {
            if o.status.success() {
                String::from_utf8(o.stdout)
                    .ok()
                    .map(|s| s.trim().to_string())
            } else {
                None
            }
        })
}

/// Wrapper to make SignalResult from comparison.
impl SignalResult {
    pub fn from_comparison(label: String, comparison: ComparisonResult, passed: bool) -> Self {
        Self {
            label,
            passed,
            pending: false,
            comparison: Some(comparison.into()),
            error: None,
        }
    }

    pub fn from_error(label: String, error: String) -> Self {
        Self {
            label,
            passed: false,
            pending: false,
            comparison: None,
            error: Some(error),
        }
    }

    /// A pending signal: golden missing and the test is `pending_reference`.
    pub fn pending(label: String, reason: String) -> Self {
        Self {
            label,
            passed: false,
            pending: true,
            comparison: None,
            error: Some(reason),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn report_summary_calculation() {
        let mut suites = BTreeMap::new();
        suites.insert(
            "test_suite".to_string(),
            SuiteResult {
                description: "Test".to_string(),
                passed: 3,
                failed: 1,
                pending: 0,
                tests: BTreeMap::new(),
            },
        );

        let report = ValidationReport::new(suites, 96000, 4);
        assert_eq!(report.summary.total_tests, 4);
        assert_eq!(report.summary.passed, 3);
        assert_eq!(report.summary.failed, 1);
        assert!((report.summary.pass_rate - 0.75).abs() < 0.001);
    }
}
