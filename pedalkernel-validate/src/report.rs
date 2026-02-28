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
    pub total_tests: usize,
    pub passed: usize,
    pub failed: usize,
    pub skipped: usize,
    pub pass_rate: f64,
}

/// Result for a single test suite.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SuiteResult {
    pub description: String,
    pub passed: usize,
    pub failed: usize,
    pub tests: BTreeMap<String, TestResult>,
}

/// Result for a single test case.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestResult {
    pub passed: bool,
    pub error: Option<String>,
    pub signals: Vec<SignalResult>,
}

/// Result for a single signal within a test.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignalResult {
    pub label: String,
    pub passed: bool,
    pub comparison: Option<ComparisonMetrics>,
    pub error: Option<String>,
}

/// Serializable version of ComparisonResult.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComparisonMetrics {
    pub normalized_rms_error_db: f64,
    pub peak_error_db: f64,
    pub thd_error_db: Option<f64>,
    pub spectral_error_db: f64,
    pub even_odd_ratio_db: Option<f64>,
    pub dc_drift_mv: Option<f64>,
}

impl From<ComparisonResult> for ComparisonMetrics {
    fn from(cr: ComparisonResult) -> Self {
        Self {
            normalized_rms_error_db: cr.normalized_rms_error_db,
            peak_error_db: cr.peak_error_db,
            thd_error_db: cr.thd_error_db,
            spectral_error_db: cr.spectral_error_db,
            even_odd_ratio_db: cr.even_odd_ratio_db,
            dc_drift_mv: cr.dc_drift_mv,
        }
    }
}

impl ValidationReport {
    /// Create a new report from suite results.
    pub fn new(
        suites: BTreeMap<String, SuiteResult>,
        sample_rate: u32,
        oversample: u32,
    ) -> Self {
        let mut total = 0;
        let mut passed = 0;
        let mut failed = 0;

        for suite in suites.values() {
            total += suite.passed + suite.failed;
            passed += suite.passed;
            failed += suite.failed;
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
                pass_rate,
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
        println!();

        for (suite_name, suite) in &self.suites {
            let status = if suite.failed == 0 {
                "PASS".green().bold()
            } else {
                "FAIL".red().bold()
            };

            println!(
                "[{}] {} - {} ({}/{})",
                status,
                suite_name.bold(),
                suite.description.dimmed(),
                suite.passed,
                suite.passed + suite.failed
            );

            for (test_name, test) in &suite.tests {
                let test_status = if test.passed {
                    "✓".green()
                } else {
                    "✗".red()
                };

                println!("  {} {}", test_status, test_name);

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
        println!(
            "{} | {}/{} passed ({:.1}%)",
            overall_status,
            self.summary.passed,
            self.summary.total_tests,
            self.summary.pass_rate * 100.0
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
                        ("-".to_string(), "-".to_string(), "-".to_string(), "-".to_string())
                    };

                    rows.push(MetricRow {
                        suite: suite_name.clone(),
                        test: test_name.clone(),
                        signal: signal.label.clone(),
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
                String::from_utf8(o.stdout).ok().map(|s| s.trim().to_string())
            } else {
                None
            }
        })
}

/// Wrapper to make SignalResult from comparison.
impl SignalResult {
    pub fn from_comparison(
        label: String,
        comparison: ComparisonResult,
        passed: bool,
    ) -> Self {
        Self {
            label,
            passed,
            comparison: Some(comparison.into()),
            error: None,
        }
    }

    pub fn from_error(label: String, error: String) -> Self {
        Self {
            label,
            passed: false,
            comparison: None,
            error: Some(error),
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
