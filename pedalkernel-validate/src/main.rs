#![allow(clippy::ptr_arg)]
#![allow(clippy::needless_borrows_for_generic_args)]
//! PedalKernel SPICE Validation CLI
//!
//! Compare WDF engine output against ngspice golden references.
//!
//! # Usage
//!
//! ```bash
//! # Run all validation suites
//! pedalkernel-validate --suite all
//!
//! # Run specific suite
//! pedalkernel-validate --suite nonlinear
//!
//! # Run specific test
//! pedalkernel-validate --suite nonlinear --test diode_clipper
//!
//! # List available tests
//! pedalkernel-validate --list
//!
//! # Custom directories
//! pedalkernel-validate --circuits ./my-circuits --golden ./my-golden
//! ```

use clap::{Parser, Subcommand};
use colored::Colorize;
use pedalkernel_validate::{
    config::{SignalConfig, ValidationConfig},
    report::ValidationReport,
    runner::{RunnerConfig, ValidationRunner},
};
use std::path::PathBuf;

#[derive(Parser)]
#[command(name = "pedalkernel-validate")]
#[command(about = "Validate WDF engine against SPICE golden references")]
#[command(version)]
struct Cli {
    #[command(subcommand)]
    command: Option<Commands>,

    /// Path to validation config YAML
    #[arg(short, long, default_value = "validate.yaml")]
    config: PathBuf,

    /// Base directory for .pedal circuit files
    #[arg(long, default_value = "circuits")]
    circuits: PathBuf,

    /// Base directory for golden .npy references
    #[arg(long, default_value = "golden")]
    golden: PathBuf,

    /// Output directory for WDF results
    #[arg(long, default_value = "output")]
    output: PathBuf,

    /// Sample rate in Hz
    #[arg(long, default_value = "96000")]
    sample_rate: u32,

    /// Oversampling factor
    #[arg(long, default_value = "4")]
    oversample: u32,

    /// Save WDF output to disk
    #[arg(long, default_value = "true")]
    save_output: bool,

    /// Output JSON report path
    #[arg(long)]
    report: Option<PathBuf>,

    /// Output path for the dynamics dashboard JSON (used by `dynamics`).
    #[arg(long, default_value = "data/dynamics.json")]
    report_dynamics: PathBuf,

    /// Show detailed metrics table
    #[arg(long, short = 'd')]
    detailed: bool,
}

#[derive(Subcommand)]
enum Commands {
    /// Run validation tests
    Run {
        /// Test suite to run (or 'all')
        #[arg(short, long, default_value = "all")]
        suite: String,

        /// Specific test to run within the suite
        #[arg(short, long)]
        test: Option<String>,
    },

    /// List available test suites and tests
    List,

    /// Quick validation of a single circuit
    Quick {
        /// Path to .pedal circuit file
        circuit: PathBuf,

        /// Path to golden reference .npy (optional)
        #[arg(short, long)]
        golden: Option<PathBuf>,
    },

    /// Generate default validation config
    Init,

    /// Generate analytical golden references for linear circuits
    GenerateLinear,

    /// Bootstrap WDF golden references from current WDF output (for regression testing).
    ///
    /// Writes to golden/{suite}/{test}/wdf/{signal}.npy.
    /// Does NOT overwrite the ngspice golden at golden/{suite}/{test}/{signal}.npy.
    Bootstrap {
        /// Test suite to bootstrap
        #[arg(short, long, default_value = "all")]
        suite: String,
    },

    /// Compare committed WDF goldens vs committed ngspice goldens.
    ///
    /// Loads golden/{suite}/{test}/wdf/{signal}.npy (WDF) and
    /// golden/{suite}/{test}/{signal}.npy (ngspice reference) and runs the
    /// same metrics::compare + PassCriteria check as `run`.  Requires
    /// neither a .pedal source file nor ngspice at runtime.
    CompareGoldens {
        /// Test suite to compare (or 'all')
        #[arg(short, long, default_value = "all")]
        suite: String,

        /// Specific test to compare within the suite
        #[arg(short, long)]
        test: Option<String>,
    },

    /// Generate golden references from SPICE simulation
    GenerateSpice {
        /// Test suite to generate
        #[arg(short, long, default_value = "all")]
        suite: String,

        /// Specific test to generate within the suite
        #[arg(long)]
        test: Option<String>,

        /// Directory containing .spice circuit files
        #[arg(long, default_value = "spice-circuits")]
        spice_dir: PathBuf,
    },

    /// Check if ngspice is available
    CheckSpice,

    /// Generate the dynamics dashboard JSON from committed dynamic-stimulus
    /// goldens (LevelSweep / ToneBurst).
    ///
    /// Iterates every validation suite/test; for each LevelSweep or ToneBurst
    /// signal it loads the ngspice golden (`{golden}/{suite}/{test}/{label}.npy`)
    /// and the WDF golden (`.../wdf/{label}.npy`), measures static gain/GR curves
    /// and attack/release ballistics via the trusted `pedalkernel::analysis`
    /// dynamics tools, and writes `--report-dynamics` (default
    /// `data/dynamics.json`).  Tests sharing a circuit are merged into one entry.
    Dynamics {
        /// ISO-8601 timestamp string written into the JSON (e.g. from CI).
        #[arg(long)]
        timestamp: Option<String>,
    },

    /// Import an externally exported LTspice/CSV trace as a golden .npy file
    ImportCsvGolden {
        /// CSV file exported from LTspice or another SPICE reference
        csv: PathBuf,

        /// Validation suite name
        #[arg(long)]
        suite: String,

        /// Validation test name
        #[arg(long)]
        test: String,

        /// Signal label, e.g. sine or sweep
        #[arg(long)]
        signal: String,

        /// Output/value column name. Defaults to common LTspice V(out).
        #[arg(long, default_value = "V(out)")]
        column: String,

        /// Time column name used for resampling variable-step SPICE output.
        #[arg(long, default_value = "time")]
        time_column: String,

        /// Duration to import in seconds. Defaults to the final CSV time.
        #[arg(long)]
        duration: Option<f64>,

        /// Keep raw CSV samples instead of resampling to sample_rate * oversample.
        #[arg(long)]
        no_resample: bool,
    },

    /// Import an LTspice binary .raw trace as a golden .npy file
    ImportLtspiceRawGolden {
        /// LTspice .raw file
        raw: PathBuf,

        /// Validation suite name
        #[arg(long)]
        suite: String,

        /// Validation test name
        #[arg(long)]
        test: String,

        /// Signal label, e.g. sine or sweep
        #[arg(long)]
        signal: String,

        /// Trace name. Defaults to common LTspice V(out).
        #[arg(long, default_value = "V(out)")]
        trace: String,

        /// Duration to import in seconds. Defaults to the final raw time.
        #[arg(long)]
        duration: Option<f64>,

        /// Keep raw LTspice samples instead of resampling to sample_rate * oversample.
        #[arg(long)]
        no_resample: bool,
    },
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();

    match &cli.command {
        Some(Commands::List) => {
            list_tests(&cli)?;
        }
        Some(Commands::Init) => {
            init_config(&cli)?;
        }
        Some(Commands::Quick { circuit, golden }) => {
            quick_validate(&cli, circuit, golden.as_deref())?;
        }
        Some(Commands::Run { suite, test }) => {
            run_validation(&cli, suite, test.clone())?;
        }
        Some(Commands::GenerateLinear) => {
            generate_linear_golden(&cli)?;
        }
        Some(Commands::Bootstrap { suite }) => {
            bootstrap_golden(&cli, suite)?;
        }
        Some(Commands::CompareGoldens { suite, test }) => {
            compare_goldens(&cli, suite, test.clone())?;
        }
        Some(Commands::GenerateSpice {
            suite,
            test,
            spice_dir,
        }) => {
            generate_spice_golden(&cli, suite, test.as_deref(), spice_dir)?;
        }
        Some(Commands::CheckSpice) => {
            check_spice()?;
        }
        Some(Commands::Dynamics { timestamp }) => {
            generate_dynamics(&cli, timestamp.clone())?;
        }
        Some(Commands::ImportCsvGolden {
            csv,
            suite,
            test,
            signal,
            column,
            time_column,
            duration,
            no_resample,
        }) => {
            import_csv_golden(
                &cli,
                csv,
                suite,
                test,
                signal,
                column,
                time_column,
                *duration,
                *no_resample,
            )?;
        }
        Some(Commands::ImportLtspiceRawGolden {
            raw,
            suite,
            test,
            signal,
            trace,
            duration,
            no_resample,
        }) => {
            import_ltspice_raw_golden(
                &cli,
                raw,
                suite,
                test,
                signal,
                trace,
                *duration,
                *no_resample,
            )?;
        }
        None => {
            run_validation(&cli, "all", None)?;
        }
    }

    Ok(())
}

fn run_validation(cli: &Cli, suite: &str, test: Option<String>) -> anyhow::Result<()> {
    println!("{} Loading configuration...", "▶".blue());

    // Load or create default config
    let validation_config = if cli.config.exists() {
        ValidationConfig::load(&cli.config)?
    } else {
        println!("  {} Config not found, using defaults", "⚠".yellow());
        ValidationConfig::default_config()
    };

    let runner_config = RunnerConfig {
        circuits_dir: cli.circuits.clone(),
        golden_dir: cli.golden.clone(),
        output_dir: cli.output.clone(),
        sample_rate: cli.sample_rate,
        oversample: cli.oversample,
        save_output: cli.save_output,
        regenerate_golden: false,
    };

    let runner = ValidationRunner::new(runner_config, validation_config.clone());

    println!(
        "{} Running validation at {}Hz × {}x...\n",
        "▶".blue(),
        cli.sample_rate,
        cli.oversample
    );

    let results = if suite == "all" {
        runner.run_all()?
    } else {
        let suite_config = validation_config
            .suites
            .get(suite)
            .ok_or_else(|| anyhow::anyhow!("Suite '{}' not found", suite))?;

        // If specific test requested, filter to just that test
        let suite_to_run = if let Some(ref test_name) = test {
            let test_case = suite_config.tests.get(test_name).ok_or_else(|| {
                anyhow::anyhow!("Test '{}' not found in suite '{}'", test_name, suite)
            })?;

            let mut filtered = suite_config.clone();
            filtered.tests.clear();
            filtered.tests.insert(test_name.clone(), test_case.clone());
            filtered
        } else {
            suite_config.clone()
        };

        let result = runner.run_suite(suite, &suite_to_run)?;
        let mut results = std::collections::BTreeMap::new();
        results.insert(suite.to_string(), result);
        results
    };

    // Create and display report
    let report = ValidationReport::new(results, cli.sample_rate, cli.oversample);
    report.print_summary();

    if cli.detailed {
        report.print_detailed();
    }

    // Save JSON report if requested
    if let Some(ref path) = cli.report {
        report.save_json(path)?;
        println!("Report saved to: {}", path.display());
    }

    // Exit with non-zero code if any tests failed
    if report.summary.failed > 0 {
        std::process::exit(1);
    }

    Ok(())
}

fn list_tests(cli: &Cli) -> anyhow::Result<()> {
    let config = if cli.config.exists() {
        ValidationConfig::load(&cli.config)?
    } else {
        ValidationConfig::default_config()
    };

    println!("{}", "Available Test Suites".bold());
    println!("{}", "─".repeat(50));

    for (suite_name, suite) in &config.suites {
        println!(
            "\n{} - {}",
            suite_name.bold().blue(),
            suite.description.dimmed()
        );

        for (test_name, test) in &suite.tests {
            println!("  • {} - {}", test_name.green(), test.description.dimmed());
            println!("    Circuit: {}", test.circuit);
            println!(
                "    Signals: {}",
                test.signals
                    .iter()
                    .map(|s| s.label())
                    .collect::<Vec<_>>()
                    .join(", ")
            );
        }
    }

    Ok(())
}

fn init_config(cli: &Cli) -> anyhow::Result<()> {
    let config = ValidationConfig::default_config();
    let yaml = serde_yaml::to_string(&config)?;

    let path = &cli.config;
    std::fs::write(path, &yaml)?;

    println!(
        "{} Created default config at: {}",
        "✓".green(),
        path.display()
    );
    println!("\nEdit this file to add your own test suites and criteria.");

    // Create directory structure
    std::fs::create_dir_all(&cli.circuits)?;
    std::fs::create_dir_all(&cli.golden)?;
    std::fs::create_dir_all(&cli.output)?;

    println!("{} Created directories:", "✓".green());
    println!("  • {}", cli.circuits.display());
    println!("  • {}", cli.golden.display());
    println!("  • {}", cli.output.display());

    Ok(())
}

fn quick_validate(
    cli: &Cli,
    circuit: &PathBuf,
    golden: Option<&std::path::Path>,
) -> anyhow::Result<()> {
    use pedalkernel_validate::runner::quick_validate as qv;

    println!("{} Quick validation of: {}", "▶".blue(), circuit.display());

    let sample_rate = (cli.sample_rate * cli.oversample) as f64;
    let result = qv(circuit, golden, sample_rate)?;

    println!("\n{}", "Results".bold());
    println!("{}", "─".repeat(40));
    println!(
        "Normalized RMS Error: {:.2} dB",
        result.normalized_rms_error_db
    );
    println!("Peak Error:           {:.2} dB", result.peak_error_db);
    println!("Spectral Error:       {:.2} dB", result.spectral_error_db);

    if let Some(thd) = result.thd_error_db {
        println!("THD Error:            {:.2} dB", thd);
    }

    if let Some(dc) = result.dc_drift_mv {
        println!("DC Drift:             {:.3} mV", dc);
    }

    // Simple pass/fail heuristic
    if result.normalized_rms_error_db < -40.0 && result.peak_error_db < -30.0 {
        println!("\n{}", "PASS".green().bold());
    } else if golden.is_some() {
        println!("\n{} (RMS > -40dB or Peak > -30dB)", "FAIL".red().bold());
        std::process::exit(1);
    } else {
        println!("\n{}", "(No golden reference for comparison)".dimmed());
    }

    Ok(())
}

fn bootstrap_golden(cli: &Cli, suite: &str) -> anyhow::Result<()> {
    use pedalkernel_validate::npy;

    println!(
        "{} Bootstrapping golden references from WDF output...",
        "▶".blue()
    );

    let validation_config = if cli.config.exists() {
        ValidationConfig::load(&cli.config)?
    } else {
        ValidationConfig::default_config()
    };

    // Build list of suites to process
    let mut suites_to_process: Vec<(String, pedalkernel_validate::TestSuite)> = Vec::new();

    if suite == "all" {
        for (name, suite_config) in &validation_config.suites {
            suites_to_process.push((name.clone(), suite_config.clone()));
        }
    } else {
        let suite_config = validation_config
            .suites
            .get(suite)
            .ok_or_else(|| anyhow::anyhow!("Suite '{}' not found", suite))?;
        suites_to_process.push((suite.to_string(), suite_config.clone()));
    }

    let sample_rate = (cli.sample_rate * cli.oversample) as f64;

    for (suite_name, suite_config) in &suites_to_process {
        println!("\n{} Suite: {}", "•".green(), suite_name.bold());

        for (test_name, test_case) in &suite_config.tests {
            println!("  {} Test: {}", "→".blue(), test_name);

            let circuit_path = cli.circuits.join(&test_case.circuit);
            if !circuit_path.exists() {
                println!(
                    "    {} Circuit not found: {}",
                    "⚠".yellow(),
                    circuit_path.display()
                );
                continue;
            }

            // Load and compile circuit
            let contents = std::fs::read_to_string(&circuit_path)?;
            let pedal_def = pedalkernel::dsl::parse_pedal_file(&contents)
                .map_err(|e| anyhow::anyhow!("Parse error: {}", e))?;
            let options = pedalkernel::compiler::CompileOptions {
                oversampling: pedalkernel::oversampling::OversamplingFactor::X1,
                ..pedalkernel::compiler::CompileOptions::default()
            };
            let mut pedal =
                pedalkernel::compiler::compile_pedal_with_options(&pedal_def, sample_rate, options)
                    .map_err(|e| anyhow::anyhow!("Compile error: {}", e))?;

            for signal_config in &test_case.signals {
                let label = signal_config.label();
                let spec = signal_config.to_spec();
                let input = spec.generate(sample_rate);

                // Reset and process
                use pedalkernel::PedalProcessor;
                pedal.reset();
                let output: Vec<f64> = input.iter().map(|&s| pedal.process(s)).collect();

                // Save as WDF golden — written to the wdf/ subdirectory so the
                // ngspice golden at golden/{suite}/{test}/{signal}.npy is never
                // clobbered.  Both artifacts are then compared by compare-goldens.
                let golden_path = cli
                    .golden
                    .join(suite_name)
                    .join(test_name)
                    .join("wdf")
                    .join(format!("{}.npy", label));

                npy::write_f64(&golden_path, &output)?;
                println!("    {} {} → {}", "✓".green(), label, golden_path.display());
            }
        }
    }

    println!(
        "\n{}",
        "Done! Golden references bootstrapped from WDF."
            .green()
            .bold()
    );
    println!("These can be used for regression testing future WDF changes.");

    Ok(())
}

/// Compare committed WDF goldens against committed ngspice goldens.
///
/// For each circuit/signal pair:
///   - WDF golden:    golden/{suite}/{test}/wdf/{signal}.npy
///   - SPICE golden:  golden/{suite}/{test}/{signal}.npy
///
/// Runs metrics::compare + PassCriteria identically to `run`.
/// Missing either file → "missing" row (not a panic).
fn compare_goldens(
    cli: &Cli,
    suite: &str,
    test_filter: Option<String>,
) -> anyhow::Result<()> {
    use pedalkernel_validate::npy;
    use pedalkernel_validate::{metrics, report::ValidationReport};

    println!(
        "{} Comparing WDF goldens vs ngspice goldens...",
        "▶".blue()
    );

    let validation_config = if cli.config.exists() {
        ValidationConfig::load(&cli.config)?
    } else {
        println!("  {} Config not found, using defaults", "⚠".yellow());
        ValidationConfig::default_config()
    };

    // Build the list of suites/tests to compare.
    let suites_to_process: Vec<(String, pedalkernel_validate::TestSuite)> = if suite == "all" {
        validation_config
            .suites
            .iter()
            .map(|(n, s)| (n.clone(), s.clone()))
            .collect()
    } else {
        let mut suite_config = validation_config
            .suites
            .get(suite)
            .cloned()
            .ok_or_else(|| anyhow::anyhow!("Suite '{}' not found", suite))?;
        if let Some(ref test_name) = test_filter {
            let test_case = suite_config
                .tests
                .get(test_name)
                .cloned()
                .ok_or_else(|| {
                    anyhow::anyhow!("Test '{}' not found in suite '{}'", test_name, suite)
                })?;
            suite_config.tests.clear();
            suite_config.tests.insert(test_name.clone(), test_case);
        }
        vec![(suite.to_string(), suite_config)]
    };

    let sample_rate = (cli.sample_rate * cli.oversample) as f64;

    let mut suite_results = std::collections::BTreeMap::new();

    for (suite_name, suite_config) in &suites_to_process {
        println!("\n{} Suite: {}", "•".green(), suite_name.bold());

        let mut test_results = std::collections::BTreeMap::new();
        let mut suite_passed = 0usize;
        let mut suite_failed = 0usize;

        for (test_name, test_case) in &suite_config.tests {
            println!("  {} Test: {}", "→".blue(), test_name);

            let warmup_trim_ms =
                test_case.effective_warmup_trim_ms(&validation_config.global);
            let trim_samples =
                ((warmup_trim_ms / 1000.0) * sample_rate).round() as usize;

            let mut signal_results = vec![];
            let mut test_passed = true;

            for signal_config in &test_case.signals {
                let label = signal_config.label();

                let spice_path = cli
                    .golden
                    .join(&suite_name)
                    .join(test_name)
                    .join(format!("{}.npy", label));
                let wdf_path = cli
                    .golden
                    .join(&suite_name)
                    .join(test_name)
                    .join("wdf")
                    .join(format!("{}.npy", label));

                let spice_exists = spice_path.exists();
                let wdf_exists = wdf_path.exists();

                if !spice_exists || !wdf_exists {
                    let missing = if !spice_exists && !wdf_exists {
                        format!(
                            "missing both goldens: {} and {}",
                            spice_path.display(),
                            wdf_path.display()
                        )
                    } else if !spice_exists {
                        format!("missing ngspice golden: {}", spice_path.display())
                    } else {
                        format!("missing WDF golden: {}", wdf_path.display())
                    };
                    println!(
                        "    {} {} — {}",
                        "⚠".yellow(),
                        label.dimmed(),
                        missing
                    );
                    signal_results.push(pedalkernel_validate::report::SignalResult {
                        label: label.clone(),
                        passed: false,
                        pending: false,
                        comparison: None,
                        error: Some(missing),
                    });
                    test_passed = false;
                    continue;
                }

                let spice_golden = npy::read_f64(&spice_path)
                    .map_err(|e| anyhow::anyhow!("Failed to read {}: {}", spice_path.display(), e))?;
                let wdf_golden = npy::read_f64(&wdf_path)
                    .map_err(|e| anyhow::anyhow!("Failed to read {}: {}", wdf_path.display(), e))?;

                // Apply warmup trim identically to `run`.
                let spice_trimmed = if trim_samples < spice_golden.len() {
                    &spice_golden[trim_samples..]
                } else {
                    &spice_golden[..]
                };
                let wdf_trimmed = if trim_samples < wdf_golden.len() {
                    &wdf_golden[trim_samples..]
                } else {
                    &wdf_golden[..]
                };

                let fundamental_hz = signal_config.fundamental_hz();
                let mut comparison =
                    metrics::compare(wdf_trimmed, spice_trimmed, sample_rate, fundamental_hz);

                // Cap the spectral comparison at the audio Nyquist (base_rate / 2):
                // above it, the engine's anti-aliasing rolls off content the raw
                // ngspice golden still carries, which is not an accuracy error.
                // `sample_rate` here is oversampled, so recompute with the cap.
                let audio_nyquist_hz = cli.sample_rate as f64 / 2.0;
                comparison.spectral_error_db = metrics::spectral_error_db(
                    wdf_trimmed,
                    spice_trimmed,
                    sample_rate,
                    Some(audio_nyquist_hz),
                );
                let passed = comparison.passes(&test_case.pass_criteria);

                let status = if passed { "✓".green() } else { "✗".red() };
                println!(
                    "    {} {} | RMS: {:.1}dB | Peak: {:.1}dB | Spectral: {:.1}dB",
                    status,
                    label.dimmed(),
                    comparison.normalized_rms_error_db,
                    comparison.peak_error_db,
                    comparison.spectral_error_db,
                );

                if !passed {
                    test_passed = false;
                }

                signal_results.push(pedalkernel_validate::report::SignalResult::from_comparison(
                    label,
                    comparison,
                    passed,
                ));
            }

            if test_passed {
                suite_passed += 1;
            } else {
                suite_failed += 1;
            }

            test_results.insert(
                test_name.clone(),
                pedalkernel_validate::report::TestResult {
                    passed: test_passed,
                    pending: false,
                    error: None,
                    signals: signal_results,
                },
            );
        }

        suite_results.insert(
            suite_name.clone(),
            pedalkernel_validate::report::SuiteResult {
                description: suite_config.description.clone(),
                passed: suite_passed,
                failed: suite_failed,
                pending: 0,
                tests: test_results,
            },
        );
    }

    let report = ValidationReport::new(suite_results, cli.sample_rate, cli.oversample);
    report.print_summary();

    if cli.detailed {
        report.print_detailed();
    }

    if let Some(ref path) = cli.report {
        report.save_json(path)?;
        println!("Report saved to: {}", path.display());
    }

    if report.summary.failed > 0 {
        std::process::exit(1);
    }

    Ok(())
}

fn generate_linear_golden(cli: &Cli) -> anyhow::Result<()> {
    use pedalkernel_validate::analytical;
    use pedalkernel_validate::npy;
    use pedalkernel_validate::signals;

    let sample_rate = (cli.sample_rate * cli.oversample) as f64;

    println!("{} Generating analytical golden references...", "▶".blue());
    println!("  Sample rate: {} Hz", sample_rate);

    // RC Lowpass: R=10k, C=10n
    let r = 10_000.0;
    let c = 10e-9;
    let fc = 1.0 / (2.0 * std::f64::consts::PI * r * c);

    println!(
        "\n{} RC Lowpass (R=10k, C=10n, fc={:.1} Hz)",
        "•".green(),
        fc
    );

    // Generate impulse response (impulse -> analytical lowpass)
    let n_samples = (0.1 * sample_rate) as usize; // 100ms
    let impulse_golden = analytical::rc_lowpass_impulse_response(r, c, sample_rate, n_samples);

    let impulse_path = cli.golden.join("linear/rc_lowpass/impulse.npy");
    npy::write_f64(&impulse_path, &impulse_golden)?;
    println!(
        "  {} Wrote impulse response: {}",
        "✓".green(),
        impulse_path.display()
    );

    // Generate sine response (for THD sanity check - linear circuit should have ~0 THD)
    let sine_input = signals::sine(sample_rate, 1000.0, 0.1, 1.0);
    let sine_golden = analytical::rc_lowpass_filter(&sine_input, r, c, sample_rate);

    let sine_path = cli.golden.join("linear/rc_lowpass/sine.npy");
    npy::write_f64(&sine_path, &sine_golden)?;
    println!(
        "  {} Wrote sine response: {}",
        "✓".green(),
        sine_path.display()
    );

    // Generate sweep response
    let sweep_input = signals::exp_sweep(sample_rate, 20.0, 20000.0, 1.0, 1.0);
    let sweep_golden = analytical::rc_lowpass_filter(&sweep_input, r, c, sample_rate);

    let sweep_path = cli.golden.join("linear/rc_lowpass/sweep.npy");
    npy::write_f64(&sweep_path, &sweep_golden)?;
    println!(
        "  {} Wrote sweep response: {}",
        "✓".green(),
        sweep_path.display()
    );

    // RL Lowpass: L=100mH, R=1k
    let l = 0.1; // 100mH
    let r_rl = 1000.0; // 1k
    let fc_rl = r_rl / (2.0 * std::f64::consts::PI * l);

    println!(
        "\n{} RL Lowpass (L=100mH, R=1k, fc={:.1} Hz)",
        "•".green(),
        fc_rl
    );

    // Generate sine response
    let sine_input_rl = signals::sine(sample_rate, 1000.0, 0.1, 1.0);
    let sine_golden_rl = analytical::rl_lowpass_filter(&sine_input_rl, l, r_rl, sample_rate);

    // Create directory if needed
    std::fs::create_dir_all(cli.golden.join("linear/rl_lowpass"))?;

    let sine_path_rl = cli.golden.join("linear/rl_lowpass/sine.npy");
    npy::write_f64(&sine_path_rl, &sine_golden_rl)?;
    println!(
        "  {} Wrote sine response: {}",
        "✓".green(),
        sine_path_rl.display()
    );

    // Resistor Divider: 10k/10k = 0.5 gain (frequency-independent)
    {
        println!("\n{} Resistor Divider (10k/10k, gain=0.5)", "•".green(),);

        std::fs::create_dir_all(cli.golden.join("linear/resistor_divider"))?;

        let sine_input = signals::sine(sample_rate, 1000.0, 0.1, 1.0);
        let sine_golden: Vec<f64> = sine_input.iter().map(|&x| x * 0.5).collect();
        let sine_path = cli.golden.join("linear/resistor_divider/sine.npy");
        npy::write_f64(&sine_path, &sine_golden)?;
        println!("  {} sine → {}", "✓".green(), sine_path.display());
    }

    // RC Highpass: C=100n, R=10k, fc≈159Hz
    {
        let r_hp = 10_000.0;
        let c_hp = 100e-9;
        let fc_hp = 1.0 / (2.0 * std::f64::consts::PI * r_hp * c_hp);

        println!(
            "\n{} RC Highpass (C=100n, R=10k, fc={:.1} Hz)",
            "•".green(),
            fc_hp
        );

        std::fs::create_dir_all(cli.golden.join("linear/rc_highpass"))?;

        let sine_input = signals::sine(sample_rate, 1000.0, 0.1, 1.0);
        let sine_golden = analytical::rc_highpass_filter(&sine_input, r_hp, c_hp, sample_rate);
        let sine_path = cli.golden.join("linear/rc_highpass/sine.npy");
        npy::write_f64(&sine_path, &sine_golden)?;
        println!("  {} sine → {}", "✓".green(), sine_path.display());

        let sweep_input = signals::exp_sweep(sample_rate, 20.0, 20000.0, 1.0, 1.0);
        let sweep_golden = analytical::rc_highpass_filter(&sweep_input, r_hp, c_hp, sample_rate);
        let sweep_path = cli.golden.join("linear/rc_highpass/sweep.npy");
        npy::write_f64(&sweep_path, &sweep_golden)?;
        println!("  {} sweep → {}", "✓".green(), sweep_path.display());
    }

    // ===== Canonical suite linear circuits =====

    // Canonical RC Lowpass: R=10k, C=10n (same as linear suite)
    {
        let r_can = 10_000.0;
        let c_can = 10e-9;
        let fc_can = 1.0 / (2.0 * std::f64::consts::PI * r_can * c_can);

        println!(
            "\n{} Canonical RC Lowpass (R=10k, C=10n, fc={:.1} Hz)",
            "•".green(),
            fc_can
        );

        let n = (0.1 * sample_rate) as usize;
        let impulse_golden = analytical::rc_lowpass_impulse_response(r_can, c_can, sample_rate, n);
        let impulse_path = cli.golden.join("canonical/rc_lowpass/impulse.npy");
        npy::write_f64(&impulse_path, &impulse_golden)?;
        println!("  {} impulse → {}", "✓".green(), impulse_path.display());

        let sine_input = signals::sine(sample_rate, 1000.0, 0.1, 1.0);
        let sine_golden = analytical::rc_lowpass_filter(&sine_input, r_can, c_can, sample_rate);
        let sine_path = cli.golden.join("canonical/rc_lowpass/sine.npy");
        npy::write_f64(&sine_path, &sine_golden)?;
        println!("  {} sine → {}", "✓".green(), sine_path.display());

        let sweep_input = signals::exp_sweep(sample_rate, 20.0, 20000.0, 1.0, 1.0);
        let sweep_golden = analytical::rc_lowpass_filter(&sweep_input, r_can, c_can, sample_rate);
        let sweep_path = cli.golden.join("canonical/rc_lowpass/sweep.npy");
        npy::write_f64(&sweep_path, &sweep_golden)?;
        println!("  {} sweep → {}", "✓".green(), sweep_path.display());
    }

    // Canonical RC Highpass: C=22n, R=33k
    {
        let r_hp = 33_000.0;
        let c_hp = 22e-9;
        let fc_hp = 1.0 / (2.0 * std::f64::consts::PI * r_hp * c_hp);

        println!(
            "\n{} Canonical RC Highpass (C=22n, R=33k, fc={:.1} Hz)",
            "•".green(),
            fc_hp
        );

        let n = (0.1 * sample_rate) as usize;
        let impulse_golden = analytical::rc_highpass_impulse_response(r_hp, c_hp, sample_rate, n);
        let impulse_path = cli.golden.join("canonical/rc_highpass/impulse.npy");
        npy::write_f64(&impulse_path, &impulse_golden)?;
        println!("  {} impulse → {}", "✓".green(), impulse_path.display());

        let sine_input = signals::sine(sample_rate, 1000.0, 0.1, 1.0);
        let sine_golden = analytical::rc_highpass_filter(&sine_input, r_hp, c_hp, sample_rate);
        let sine_path = cli.golden.join("canonical/rc_highpass/sine.npy");
        npy::write_f64(&sine_path, &sine_golden)?;
        println!("  {} sine → {}", "✓".green(), sine_path.display());

        let sweep_input = signals::exp_sweep(sample_rate, 20.0, 20000.0, 1.0, 1.0);
        let sweep_golden = analytical::rc_highpass_filter(&sweep_input, r_hp, c_hp, sample_rate);
        let sweep_path = cli.golden.join("canonical/rc_highpass/sweep.npy");
        npy::write_f64(&sweep_path, &sweep_golden)?;
        println!("  {} sweep → {}", "✓".green(), sweep_path.display());
    }

    // Canonical Series RLC Bandpass: R=100, L=10mH, C=100nF, RL=10k
    {
        let r_rlc = 100.0_f64;
        let l_rlc = 10e-3_f64;
        let c_rlc = 100e-9_f64;
        let rl_rlc = 10_000.0_f64;
        let f0 = 1.0 / (2.0 * std::f64::consts::PI * (l_rlc * c_rlc).sqrt());

        println!(
            "\n{} Canonical Series RLC Bandpass (R=100, L=10mH, C=100nF, f0={:.1} Hz)",
            "•".green(),
            f0
        );

        let n = (0.1 * sample_rate) as usize;
        let impulse_input: Vec<f64> = (0..n).map(|i| if i == 0 { 1.0 } else { 0.0 }).collect();
        let impulse_golden = analytical::rlc_bandpass_filter(
            &impulse_input,
            r_rlc,
            l_rlc,
            c_rlc,
            rl_rlc,
            sample_rate,
        );
        let impulse_path = cli.golden.join("canonical/series_rlc/impulse.npy");
        npy::write_f64(&impulse_path, &impulse_golden)?;
        println!("  {} impulse → {}", "✓".green(), impulse_path.display());

        let sine_input = signals::sine(sample_rate, 5000.0, 0.1, 1.0);
        let sine_golden =
            analytical::rlc_bandpass_filter(&sine_input, r_rlc, l_rlc, c_rlc, rl_rlc, sample_rate);
        let sine_path = cli.golden.join("canonical/series_rlc/sine.npy");
        npy::write_f64(&sine_path, &sine_golden)?;
        println!("  {} sine → {}", "✓".green(), sine_path.display());

        let sweep_input = signals::exp_sweep(sample_rate, 20.0, 20000.0, 1.0, 1.0);
        let sweep_golden =
            analytical::rlc_bandpass_filter(&sweep_input, r_rlc, l_rlc, c_rlc, rl_rlc, sample_rate);
        let sweep_path = cli.golden.join("canonical/series_rlc/sweep.npy");
        npy::write_f64(&sweep_path, &sweep_golden)?;
        println!("  {} sweep → {}", "✓".green(), sweep_path.display());
    }

    // Canonical Twin-T Notch: R=10k, C=10nF
    {
        let r_tt = 10_000.0;
        let c_tt = 10e-9;
        let f_notch = 1.0 / (2.0 * std::f64::consts::PI * r_tt * c_tt);

        println!(
            "\n{} Canonical Twin-T Notch (R=10k, C=10nF, f_notch={:.1} Hz)",
            "•".green(),
            f_notch
        );

        let n = (0.1 * sample_rate) as usize;
        let impulse_input: Vec<f64> = (0..n).map(|i| if i == 0 { 1.0 } else { 0.0 }).collect();
        let impulse_golden =
            analytical::twin_t_notch_filter(&impulse_input, r_tt, c_tt, sample_rate);
        let impulse_path = cli.golden.join("canonical/twin_t_notch/impulse.npy");
        npy::write_f64(&impulse_path, &impulse_golden)?;
        println!("  {} impulse → {}", "✓".green(), impulse_path.display());

        let sine_input = signals::sine(sample_rate, 1500.0, 0.1, 1.0);
        let sine_golden = analytical::twin_t_notch_filter(&sine_input, r_tt, c_tt, sample_rate);
        let sine_path = cli.golden.join("canonical/twin_t_notch/sine.npy");
        npy::write_f64(&sine_path, &sine_golden)?;
        println!("  {} sine → {}", "✓".green(), sine_path.display());

        let sweep_input = signals::exp_sweep(sample_rate, 20.0, 20000.0, 1.0, 1.0);
        let sweep_golden = analytical::twin_t_notch_filter(&sweep_input, r_tt, c_tt, sample_rate);
        let sweep_path = cli.golden.join("canonical/twin_t_notch/sweep.npy");
        npy::write_f64(&sweep_path, &sweep_golden)?;
        println!("  {} sweep → {}", "✓".green(), sweep_path.display());
    }

    println!("\n{}", "Done! Golden references generated.".green().bold());
    println!("Run 'pedalkernel-validate run --suite linear' or '--suite canonical' to validate.");

    Ok(())
}

// ===========================================================================
// Dynamics dashboard — JSON model + generator
// ===========================================================================

/// 0 dBVU reference peak voltage (matches `signals::dbvu_to_peak`).
const DBVU_REF_PEAK: f64 = 0.7746;

/// Convert an RMS amplitude (of a steady sine) to output dBVU.
///
/// `rms * SQRT_2` is the sine peak; dividing by the 0-dBVU peak reference and
/// taking 20·log10 yields dBVU.  Mirrors the harness convention so the WDF and
/// ngspice numbers are directly comparable to the `levels_dbvu` inputs.
fn rms_to_dbvu(rms: f64) -> f64 {
    pedalkernel::analysis::lin_to_db(rms * std::f64::consts::SQRT_2 / DBVU_REF_PEAK)
}

#[derive(serde::Serialize)]
struct DynamicsJson {
    schema_version: u32,
    run_timestamp: String,
    git_commit: Option<String>,
    sample_rate: u32,
    oversample: u32,
    circuits: std::collections::BTreeMap<String, DynamicsCircuit>,
}

#[derive(serde::Serialize)]
struct DynamicsCircuit {
    suite: String,
    test: String,
    circuit: String,
    frequency_hz: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    static_curve: Option<StaticCurve>,
    #[serde(skip_serializing_if = "Option::is_none")]
    tone_burst: Option<ToneBurstDyn>,
}

#[derive(serde::Serialize)]
struct StaticCurve {
    input_levels_dbvu: Vec<f64>,
    wdf_output_dbvu: Vec<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    spice_output_dbvu: Option<Vec<f64>>,
    wdf_gr_db: Vec<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    spice_gr_db: Option<Vec<f64>>,
    wdf_ratio: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    spice_ratio: Option<f64>,
    wdf_gr_total_db: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    spice_gr_total_db: Option<f64>,
}

#[derive(serde::Serialize)]
struct ToneBurstDyn {
    on_ms: f64,
    off_ms: f64,
    time_ms: Vec<f64>,
    wdf_gr_db: Vec<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    spice_gr_db: Option<Vec<f64>>,
    wdf_attack_ms: f64,
    wdf_release_ms: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    spice_attack_ms: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    spice_release_ms: Option<f64>,
}

/// Measure a static gain curve from a level-sweep golden.
///
/// The golden is `levels.len()` segments of `duration_per_level` seconds each
/// (no warmup pre-roll for the level sweep — the trim only matters for skipping
/// the per-level transient, which we do by measuring the LAST 60% of each
/// segment).  Returns `(input_dbvu, output_dbvu)` points.
fn static_curve_from_golden(
    golden: &[f64],
    levels_dbvu: &[f64],
    duration_per_level: f64,
    effective_sr: f64,
) -> Vec<(f64, f64)> {
    let seg = (duration_per_level * effective_sr).round() as usize;
    let mut curve = Vec::with_capacity(levels_dbvu.len());
    for (k, &in_dbvu) in levels_dbvu.iter().enumerate() {
        let start = k * seg;
        let end = (start + seg).min(golden.len());
        if start >= golden.len() || end <= start {
            break;
        }
        let segment = &golden[start..end];
        // Measure RMS over the last 60% of the segment (skip per-level transient).
        let meas_start = segment.len() - (segment.len() * 6) / 10;
        let meas = &segment[meas_start..];
        let out_dbvu = rms_to_dbvu(pedalkernel::analysis::rms(meas));
        curve.push((in_dbvu, out_dbvu));
    }
    curve
}

/// Per-input gain-reduction series relative to the lowest input.
///
/// `gr_k = (out_0 - in_0) - (out_k - in_k)` — the gain applied at the lowest
/// input minus the gain at input k.  Positive = the circuit pulls level down as
/// the input rises (i.e. compresses).  `gr_0 == 0`.
fn gr_series(curve: &[(f64, f64)]) -> Vec<f64> {
    if curve.is_empty() {
        return Vec::new();
    }
    let (in0, out0) = curve[0];
    let g0 = out0 - in0;
    curve.iter().map(|&(i, o)| g0 - (o - i)).collect()
}

/// Detect the onset and end sample of a tone burst by RMS-envelope threshold.
///
/// Robust to warmup-trim shifts: the onset is the first sample where the
/// envelope crosses 10% of its peak; the end is the first sample AFTER the
/// onset where it falls back below that threshold.  Falls back to the nominal
/// `on_ms` index for the end if no clean falling edge is found.
fn detect_burst_edges(env: &[f64], on_ms: f64, effective_sr: f64) -> (usize, usize) {
    let peak = env.iter().cloned().fold(0.0f64, f64::max);
    let thresh = peak * 0.10;
    let onset = env.iter().position(|&e| e > thresh).unwrap_or(0);
    let nominal_end = onset + ((on_ms / 1000.0) * effective_sr).round() as usize;
    // Search for the falling edge after the onset (skip a few samples past it).
    let search_from = (onset + (env.len() / 1000).max(1)).min(env.len());
    let end = env[search_from..]
        .iter()
        .position(|&e| e < thresh)
        .map(|p| search_from + p)
        .unwrap_or_else(|| nominal_end.min(env.len().saturating_sub(1)));
    (onset, end)
}

/// Measure tone-burst dynamics (attack/release ms + GR-vs-time envelope).
fn tone_burst_from_golden(
    golden: &[f64],
    on_ms: f64,
    off_ms: f64,
    effective_sr: f64,
) -> (f64, f64, Vec<f64>, Vec<f64>) {
    use pedalkernel::analysis::{db_to_lin, lin_to_db, measure_attack_seconds, measure_release_seconds, rms_envelope};

    let env = rms_envelope(golden, 0.002, effective_sr);
    let (onset, end) = detect_burst_edges(&env, on_ms, effective_sr);

    // Attack measured from the burst onset; release from the burst end.
    let attack_ms = 1000.0 * measure_attack_seconds(golden, onset, effective_sr);
    let release_ms = 1000.0 * measure_release_seconds(golden, end, effective_sr);

    // GR-vs-time reference = the EARLY-burst steady level (the gain before the
    // envelope detector reacts), measured 1–4 ms after onset to skip the onset
    // transient. GR(t) = ref_db - level(t): a non-compressing path stays ~0,
    // while a compressor's level drops over the burst so GR rises (attack).
    // Using the on-region MAX instead made a flat path read a spurious constant
    // GR equal to its onset spike.
    let ref_lo = (onset + (0.001 * effective_sr) as usize).min(env.len());
    let ref_hi = (onset + (0.004 * effective_sr) as usize).min(env.len()).max(ref_lo + 1);
    let ref_slice = &env[ref_lo.min(env.len().saturating_sub(1))..ref_hi.min(env.len()).max(ref_lo.min(env.len().saturating_sub(1)) + 1)];
    let reference = (ref_slice.iter().sum::<f64>() / ref_slice.len().max(1) as f64).max(1e-12);
    let ref_db = lin_to_db(reference);

    // GR-vs-time is only meaningful over the burst ON window: outside it there
    // is no signal, so the envelope collapses to ~0 and `lin_to_db` would emit
    // a spurious ~200 dB "gain reduction". Restrict to [onset, end], floor the
    // envelope 60 dB below the reference, and clamp GR to a sane band.
    let win_start = onset.min(env.len());
    // Window = the known burst length (onset + on_ms). The detected falling edge
    // (`end`) misfires on circuits whose level dips mid-burst (e.g. opto riding
    // gain down), truncating the window to a few ms; the nominal length is the
    // robust choice since we know the stimulus.
    let _ = end;
    let nominal_end = onset + ((on_ms / 1000.0) * effective_sr).round() as usize;
    let win_end = nominal_end.min(env.len()).max(win_start + 1);
    let span = win_end.saturating_sub(win_start).max(1);
    let stride = (span / 2000).max(1);
    let floor = reference * db_to_lin(-60.0);
    let mut time_ms = Vec::new();
    let mut gr_db = Vec::new();
    let mut i = win_start;
    while i < win_end {
        let t_ms = (i as f64 / effective_sr) * 1000.0;
        let gr = (ref_db - lin_to_db(env[i].max(floor))).clamp(-3.0, 40.0);
        time_ms.push(t_ms);
        gr_db.push(gr);
        i += stride;
    }
    let _ = off_ms;
    (attack_ms, release_ms, time_ms, gr_db)
}

/// One of the live-measured compressors: a `.pedal` to compile + the control
/// settings that put it in a COMPRESSING state. Paths are repo-relative to the
/// `pedalkernel/` crate's `examples/` tree.
struct LiveCircuit {
    /// Dashboard test/circuit key stem (e.g. "la2a").
    test: &'static str,
    /// `.pedal` path relative to the `pedalkernel/` crate root.
    pedal_rel: &'static str,
    /// Control settings (label, value) — applied if the control exists.
    controls: &'static [(&'static str, f64)],
}

/// The five compressors graphed on the dynamics dashboard. `fet_leveler` is
/// listed for completeness but is measured from its committed goldens (it has an
/// ngspice golden), so the live path SKIPS it to avoid clobbering that entry.
const LIVE_CIRCUITS: &[LiveCircuit] = &[
    LiveCircuit {
        test: "la2a",
        pedal_rel: "examples/outboard/compressor/la2a.pedal",
        // Peak Reduction high = max compression; Gain mid = makeup.
        controls: &[("Peak Reduction", 0.8), ("Gain", 0.6), ("Limit/Compress", 0.0)],
    },
    LiveCircuit {
        test: "opto_leveler",
        pedal_rel: "examples/outboard/compressor/opto_leveler.pedal",
        // Only a Gain (makeup) knob; the photocell IS the time constant.
        controls: &[("Gain", 0.6)],
    },
    LiveCircuit {
        test: "vca_bus_comp",
        pedal_rel: "examples/outboard/compressor/vca_bus_comp.pedal",
        // Only a Makeup knob; compression is automatic via the sidechain.
        controls: &[("Makeup", 0.5)],
    },
    LiveCircuit {
        test: "dyna_comp",
        pedal_rel: "examples/pedals/compressor/dyna_comp.pedal",
        // Sensitivity high = max compression; Output mid = makeup.
        controls: &[("Sensitivity", 0.8), ("Output", 0.5)],
    },
];

/// Resolve a `pedalkernel/`-crate-relative example path to an absolute path,
/// trying the build-time crate sibling first, then a few cwd-relative fallbacks.
fn resolve_pedal_path(pedal_rel: &str) -> Option<std::path::PathBuf> {
    let candidates = [
        // pedalkernel-validate/../pedalkernel/<rel>
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("pedalkernel")
            .join(pedal_rel),
        // cwd = repo root: pedalkernel/<rel>
        std::path::Path::new("pedalkernel").join(pedal_rel),
        // cwd = pedalkernel crate: <rel>
        std::path::PathBuf::from(pedal_rel),
    ];
    candidates.into_iter().find(|p| p.exists())
}

/// Build a fresh compiled processor for `src` at `sample_rate`, applying the
/// controls that exist. Unknown control labels are skipped gracefully (the
/// compiled pedal's `set_control_immediate` is a no-op for absent labels).
fn build_live_processor(
    src: &str,
    sample_rate: f64,
    controls: &[(&str, f64)],
) -> anyhow::Result<impl FnMut(f64) -> f64> {
    let pedal_def = pedalkernel::dsl::parse_pedal_file(src)
        .map_err(|e| anyhow::anyhow!("Parse error: {}", e))?;
    let options = pedalkernel::compiler::CompileOptions {
        oversampling: pedalkernel::oversampling::OversamplingFactor::X1,
        // Offline one-shot measurement: skip the (expensive) K-method table
        // generation and solve NL roots with direct Newton-Raphson at runtime.
        // We are not real-time here, so the per-sample NR cost is fine and we
        // avoid rebuilding K-tables for every compile (this loop recompiles per
        // level), which dominated the dynamics run time.
        skip_k_tables: true,
        ..pedalkernel::compiler::CompileOptions::default()
    };
    let mut pedal =
        pedalkernel::compiler::compile_pedal_with_options(&pedal_def, sample_rate, options)
            .map_err(|e| anyhow::anyhow!("Compile error: {}", e))?;
    for &(label, val) in controls {
        // Immediate (not smoothed) so the compressing state is in effect from
        // sample 0 — no control-smoother ramp eating into the settle window.
        pedal.set_control_immediate(label, val);
    }
    use pedalkernel::PedalProcessor;
    Ok(move |s| pedal.process(s))
}

/// LIVE static gain curve: a 1 kHz sine at each level (-40..=0 step 5 dBVU) is
/// run through a FRESH processor (so the previous level's envelope state cannot
/// bleed in) with a generous SETTLE before measuring the steady RMS over the
/// last WINDOW. Returns `(input_dbvu, output_dbvu)` points.
fn live_static_curve(
    src: &str,
    sample_rate: f64,
    controls: &[(&str, f64)],
    freq_hz: f64,
) -> anyhow::Result<Vec<(f64, f64)>> {
    use pedalkernel_validate::signals::dbvu_to_peak;
    // Slow opto/LA-2A ballistics release over 1-2 s: a short settle would
    // measure mid-transition, so use a generous 4 s settle + 1 s window.
    const SETTLE: f64 = 4.0;
    const WINDOW: f64 = 1.0;
    let total = SETTLE + WINDOW;
    let n_total = (total * sample_rate).round() as usize;
    let n_window = (WINDOW * sample_rate).round() as usize;

    let mut curve = Vec::new();
    let mut level = -40.0_f64;
    while level <= 0.0 + 1e-9 {
        let amp = dbvu_to_peak(level);
        let mut proc = build_live_processor(src, sample_rate, controls)?;
        let mut out = Vec::with_capacity(n_total);
        for i in 0..n_total {
            let t = i as f64 / sample_rate;
            let x = amp * (2.0 * std::f64::consts::PI * freq_hz * t).sin();
            out.push(proc(x));
        }
        let meas = &out[n_total.saturating_sub(n_window)..];
        let out_dbvu = rms_to_dbvu(pedalkernel::analysis::rms(meas));
        curve.push((level, out_dbvu));
        level += 5.0;
    }
    Ok(curve)
}

/// LIVE tone burst: a 1 kHz, -6 dBVU burst (on `on_ms`, off `off_ms`) through a
/// fresh processor, then the SAME measurement logic as `tone_burst_from_golden`.
fn live_tone_burst(
    src: &str,
    sample_rate: f64,
    controls: &[(&str, f64)],
    freq_hz: f64,
    on_ms: f64,
    off_ms: f64,
) -> anyhow::Result<(f64, f64, Vec<f64>, Vec<f64>)> {
    use pedalkernel_validate::signals::dbvu_to_peak;
    let amp = dbvu_to_peak(-6.0);
    let on_n = ((on_ms / 1000.0) * sample_rate).round() as usize;
    let off_n = ((off_ms / 1000.0) * sample_rate).round() as usize;
    let mut proc = build_live_processor(src, sample_rate, controls)?;
    let mut out = Vec::with_capacity(on_n + off_n);
    // ON segment: live tone.
    for i in 0..on_n {
        let t = i as f64 / sample_rate;
        let x = amp * (2.0 * std::f64::consts::PI * freq_hz * t).sin();
        out.push(proc(x));
    }
    // OFF segment: silence (the release tail).
    for _ in 0..off_n {
        out.push(proc(0.0));
    }
    Ok(tone_burst_from_golden(&out, on_ms, off_ms, sample_rate))
}

/// Measure all `LIVE_CIRCUITS` through the engine and insert WDF-only entries.
fn add_live_dynamics(
    cli: &Cli,
    circuits: &mut std::collections::BTreeMap<String, DynamicsCircuit>,
    effective_sr: f64,
) {
    use pedalkernel::analysis::{compression_ratio, gain_reduction_db};
    const FREQ_HZ: f64 = 1000.0;
    const ON_MS: f64 = 500.0;
    const OFF_MS: f64 = 2000.0;
    // Live dynamics are measured at a low base rate, NOT the oversampled
    // accuracy rate: a 1 kHz tone's envelope / steady-state gain is sample-rate
    // independent, so 48 kHz gives the same curves at ~8x less work than the
    // 384 kHz `effective_sr`. (The fet_leveler GOLDEN path still uses its own
    // committed rate.)
    const LIVE_SR: f64 = 48_000.0;
    let _ = effective_sr;
    let suite = "compressor";

    for lc in LIVE_CIRCUITS {
        let Some(path) = resolve_pedal_path(lc.pedal_rel) else {
            println!(
                "  {} {}/{} — pedal not found ({}), skipping live",
                "⚠".yellow(),
                suite,
                lc.test,
                lc.pedal_rel
            );
            continue;
        };
        let src = match std::fs::read_to_string(&path) {
            Ok(s) => s,
            Err(e) => {
                println!(
                    "  {} {}/{} — read error: {}, skipping",
                    "⚠".yellow(),
                    suite,
                    lc.test,
                    e
                );
                continue;
            }
        };

        let circuit_key = format!("{}/{}", suite, lc.test);
        // The circuit path for display (repo-relative under pedalkernel/).
        let circuit_disp = format!("pedalkernel/{}", lc.pedal_rel);

        // --- LIVE static gain curve ---
        let static_curve = match live_static_curve(&src, LIVE_SR, lc.controls, FREQ_HZ) {
            Ok(c) if c.len() >= 2 => Some(c),
            Ok(_) => {
                println!("  {} {} — static curve too short, skipping", "⚠".yellow(), circuit_key);
                None
            }
            Err(e) => {
                println!("  {} {} — static curve error: {}", "⚠".yellow(), circuit_key, e);
                None
            }
        };

        // --- LIVE tone burst ---
        let tone_burst =
            match live_tone_burst(&src, LIVE_SR, lc.controls, FREQ_HZ, ON_MS, OFF_MS) {
                Ok((a, r, t, gr)) if !t.is_empty() => Some((a, r, t, gr)),
                Ok(_) => {
                    println!("  {} {} — tone burst empty, skipping", "⚠".yellow(), circuit_key);
                    None
                }
                Err(e) => {
                    println!("  {} {} — tone burst error: {}", "⚠".yellow(), circuit_key, e);
                    None
                }
            };

        let entry = circuits.entry(circuit_key.clone()).or_insert_with(|| DynamicsCircuit {
            suite: suite.to_string(),
            test: lc.test.to_string(),
            circuit: circuit_disp.clone(),
            frequency_hz: FREQ_HZ,
            static_curve: None,
            tone_burst: None,
        });

        if let Some(curve) = static_curve {
            let lo = curve.first().unwrap().0;
            let hi = curve.last().unwrap().0;
            let wdf_ratio = compression_ratio(&curve, lo, hi);
            let wdf_gr_total = gain_reduction_db(&curve, lo, hi);
            entry.static_curve = Some(StaticCurve {
                input_levels_dbvu: curve.iter().map(|p| p.0).collect(),
                wdf_output_dbvu: curve.iter().map(|p| p.1).collect(),
                spice_output_dbvu: None,
                wdf_gr_db: gr_series(&curve),
                spice_gr_db: None,
                wdf_ratio,
                spice_ratio: None,
                wdf_gr_total_db: wdf_gr_total,
                spice_gr_total_db: None,
            });
            println!(
                "  {} {} [live level_sweep] WDF GR {:.2} dB / ratio {:.2}:1 (WDF-only)",
                "✓".green(),
                circuit_key,
                wdf_gr_total,
                wdf_ratio,
            );
        }

        if let Some((a, r, t, gr)) = tone_burst {
            entry.tone_burst = Some(ToneBurstDyn {
                on_ms: ON_MS,
                off_ms: OFF_MS,
                time_ms: t,
                wdf_gr_db: gr,
                spice_gr_db: None,
                wdf_attack_ms: a,
                wdf_release_ms: r,
                spice_attack_ms: None,
                spice_release_ms: None,
            });
            println!(
                "  {} {} [live tone_burst] WDF atk {:.1} ms / rel {:.1} ms (WDF-only)",
                "✓".green(),
                circuit_key,
                a,
                r,
            );
        }
    }
    let _ = cli;
}

fn generate_dynamics(cli: &Cli, timestamp: Option<String>) -> anyhow::Result<()> {
    use pedalkernel_validate::npy;

    println!("{} Generating dynamics dashboard JSON...", "▶".blue());

    let validation_config = if cli.config.exists() {
        ValidationConfig::load(&cli.config)?
    } else {
        ValidationConfig::default_config()
    };

    let base_rate = cli.sample_rate;
    let oversample = cli.oversample;
    let effective_sr = (base_rate * oversample) as f64;

    // Two stimulus types feed two facets of ONE entry per circuit (.pedal path),
    // so a circuit with both a LevelSweep test and a ToneBurst test merges.
    let mut circuits: std::collections::BTreeMap<String, DynamicsCircuit> =
        std::collections::BTreeMap::new();

    for (suite_name, suite_config) in &validation_config.suites {
        for (test_name, test_case) in &suite_config.tests {
            let warmup_trim_ms = test_case.effective_warmup_trim_ms(&validation_config.global);
            let trim_samples = ((warmup_trim_ms / 1000.0) * effective_sr).round() as usize;

            for signal in &test_case.signals {
                // Only dynamic stimuli are dashboarded.
                let is_dynamic = matches!(
                    signal,
                    SignalConfig::LevelSweep { .. } | SignalConfig::ToneBurst { .. }
                );
                if !is_dynamic {
                    continue;
                }

                let label = signal.label();
                let spice_path = cli
                    .golden
                    .join(suite_name)
                    .join(test_name)
                    .join(format!("{}.npy", label));
                let wdf_path = cli
                    .golden
                    .join(suite_name)
                    .join(test_name)
                    .join("wdf")
                    .join(format!("{}.npy", label));

                if !spice_path.exists() || !wdf_path.exists() {
                    println!(
                        "  {} {}/{} [{}] — missing golden(s), skipping",
                        "⚠".yellow(),
                        suite_name,
                        test_name,
                        label
                    );
                    continue;
                }

                let spice = npy::read_f64(&spice_path)?;
                let wdf = npy::read_f64(&wdf_path)?;
                let freq_hz = signal.fundamental_hz().unwrap_or(0.0);

                // The dashboard key is the circuit path stem so tests sharing a
                // .pedal (e.g. *_level_sweep + *_tone_burst) collapse to one card.
                let circuit_key = format!(
                    "{}/{}",
                    suite_name,
                    std::path::Path::new(&test_case.circuit)
                        .file_stem()
                        .and_then(|s| s.to_str())
                        .unwrap_or(test_name)
                );

                let entry = circuits.entry(circuit_key.clone()).or_insert_with(|| {
                    DynamicsCircuit {
                        suite: suite_name.clone(),
                        test: std::path::Path::new(&test_case.circuit)
                            .file_stem()
                            .and_then(|s| s.to_str())
                            .unwrap_or(test_name)
                            .to_string(),
                        circuit: test_case.circuit.clone(),
                        frequency_hz: freq_hz,
                        static_curve: None,
                        tone_burst: None,
                    }
                });

                match signal {
                    SignalConfig::LevelSweep {
                        levels_dbvu,
                        duration_per_level,
                        ..
                    } => {
                        if levels_dbvu.len() < 2 {
                            continue;
                        }
                        let wdf_curve = static_curve_from_golden(
                            &wdf,
                            levels_dbvu,
                            *duration_per_level,
                            effective_sr,
                        );
                        let spice_curve = static_curve_from_golden(
                            &spice,
                            levels_dbvu,
                            *duration_per_level,
                            effective_sr,
                        );
                        if wdf_curve.len() < 2 || spice_curve.len() < 2 {
                            continue;
                        }
                        let lo = *levels_dbvu.first().unwrap();
                        let hi = *levels_dbvu.last().unwrap();
                        use pedalkernel::analysis::{compression_ratio, gain_reduction_db};
                        let wdf_ratio = compression_ratio(&wdf_curve, lo, hi);
                        let spice_ratio = compression_ratio(&spice_curve, lo, hi);
                        let wdf_gr_total = gain_reduction_db(&wdf_curve, lo, hi);
                        let spice_gr_total = gain_reduction_db(&spice_curve, lo, hi);

                        entry.static_curve = Some(StaticCurve {
                            input_levels_dbvu: wdf_curve.iter().map(|p| p.0).collect(),
                            wdf_output_dbvu: wdf_curve.iter().map(|p| p.1).collect(),
                            spice_output_dbvu: Some(spice_curve.iter().map(|p| p.1).collect()),
                            wdf_gr_db: gr_series(&wdf_curve),
                            spice_gr_db: Some(gr_series(&spice_curve)),
                            wdf_ratio,
                            spice_ratio: Some(spice_ratio),
                            wdf_gr_total_db: wdf_gr_total,
                            spice_gr_total_db: Some(spice_gr_total),
                        });
                        println!(
                            "  {} {} [level_sweep] WDF GR {:.2} dB / ratio {:.2}:1 | ngspice GR {:.2} dB / ratio {:.2}:1",
                            "✓".green(),
                            circuit_key,
                            wdf_gr_total,
                            wdf_ratio,
                            spice_gr_total,
                            spice_ratio,
                        );
                    }
                    SignalConfig::ToneBurst {
                        on_ms, off_ms, ..
                    } => {
                        // Apply the warmup trim to both arrays before measuring.
                        let wdf_t: &[f64] = if trim_samples < wdf.len() {
                            &wdf[trim_samples..]
                        } else {
                            &wdf[..]
                        };
                        let spice_t: &[f64] = if trim_samples < spice.len() {
                            &spice[trim_samples..]
                        } else {
                            &spice[..]
                        };
                        let (wdf_a, wdf_r, wdf_time, wdf_gr) =
                            tone_burst_from_golden(wdf_t, *on_ms, *off_ms, effective_sr);
                        let (sp_a, sp_r, _sp_time, sp_gr) =
                            tone_burst_from_golden(spice_t, *on_ms, *off_ms, effective_sr);
                        // Both downsample with the same stride/length; align by
                        // truncating to the shorter (WDF time base is the shared x).
                        let n = wdf_time.len().min(sp_gr.len());
                        entry.tone_burst = Some(ToneBurstDyn {
                            on_ms: *on_ms,
                            off_ms: *off_ms,
                            time_ms: wdf_time[..n].to_vec(),
                            wdf_gr_db: wdf_gr[..n].to_vec(),
                            spice_gr_db: Some(sp_gr[..n].to_vec()),
                            wdf_attack_ms: wdf_a,
                            wdf_release_ms: wdf_r,
                            spice_attack_ms: Some(sp_a),
                            spice_release_ms: Some(sp_r),
                        });
                        println!(
                            "  {} {} [tone_burst] WDF atk {:.1} ms / rel {:.1} ms | ngspice atk {:.1} ms / rel {:.1} ms",
                            "✓".green(),
                            circuit_key,
                            wdf_a,
                            wdf_r,
                            sp_a,
                            sp_r,
                        );
                    }
                    _ => unreachable!(),
                }
            }
        }
    }

    // Live WDF measurement for the four golden-less compressors (and any other
    // LIVE_CIRCUITS that lack a golden entry). fet_leveler is intentionally
    // skipped here — it is measured from its committed WDF + ngspice goldens.
    println!(
        "{} Measuring WDF-only compressors live through the engine...",
        "▶".blue()
    );
    add_live_dynamics(cli, &mut circuits, effective_sr);

    let git_commit = std::env::var("GIT_COMMIT").ok().filter(|s| !s.is_empty());
    let run_timestamp = timestamp.unwrap_or_else(|| {
        // No git shelling; a coarse epoch-seconds fallback keeps it deterministic-ish.
        format!(
            "unix:{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_secs())
                .unwrap_or(0)
        )
    });

    let json = DynamicsJson {
        schema_version: 1,
        run_timestamp,
        git_commit,
        sample_rate: base_rate,
        oversample,
        circuits,
    };

    if let Some(parent) = cli.report_dynamics.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let pretty = serde_json::to_string_pretty(&json)?;
    std::fs::write(&cli.report_dynamics, &pretty)?;

    println!(
        "\n{} Wrote {} circuit entr{} to {}",
        "✓".green(),
        json.circuits.len(),
        if json.circuits.len() == 1 { "y" } else { "ies" },
        cli.report_dynamics.display()
    );

    Ok(())
}

fn check_spice() -> anyhow::Result<()> {
    use pedalkernel_validate::spice::SpiceRunner;

    println!("{} Checking ngspice installation...", "▶".blue());

    match SpiceRunner::check_ngspice() {
        Ok(version) => {
            println!("{} ngspice found: {}", "✓".green(), version);
            Ok(())
        }
        Err(e) => {
            println!("{} {}", "✗".red(), e);
            println!("\nInstall ngspice:");
            println!("  macOS:  brew install ngspice");
            println!("  Ubuntu: apt install ngspice");
            println!("  Arch:   pacman -S ngspice");
            std::process::exit(1);
        }
    }
}

#[allow(clippy::too_many_arguments)] // import helper: each arg is a distinct CSV field selector
fn import_csv_golden(
    cli: &Cli,
    csv: &std::path::Path,
    suite: &str,
    test: &str,
    signal: &str,
    column: &str,
    time_column: &str,
    duration: Option<f64>,
    no_resample: bool,
) -> anyhow::Result<()> {
    let contents = std::fs::read_to_string(csv)?;
    let mut lines = contents.lines().filter(|line| !line.trim().is_empty());
    let header = lines
        .next()
        .ok_or_else(|| anyhow::anyhow!("CSV is empty: {}", csv.display()))?;
    let headers = split_csv_line(header);
    let value_idx = headers
        .iter()
        .position(|h| h == column)
        .or_else(|| headers.iter().position(|h| h.eq_ignore_ascii_case(column)))
        .ok_or_else(|| {
            anyhow::anyhow!(
                "Column '{}' not found in {}. Available columns: {}",
                column,
                csv.display(),
                headers.join(", ")
            )
        })?;
    let time_idx = headers.iter().position(|h| h == time_column).or_else(|| {
        headers
            .iter()
            .position(|h| h.eq_ignore_ascii_case(time_column))
    });

    let mut samples = Vec::new();
    for (line_idx, line) in lines.enumerate() {
        let fields = split_csv_line(line);
        let Some(raw) = fields.get(value_idx) else {
            continue;
        };
        let value = parse_spice_float(raw).map_err(|e| {
            anyhow::anyhow!(
                "Failed to parse row {} column '{}': {}",
                line_idx + 2,
                column,
                e
            )
        })?;
        let time = time_idx
            .and_then(|idx| fields.get(idx))
            .and_then(|raw| parse_spice_float(raw).ok());
        samples.push((time, value));
    }

    if samples.is_empty() {
        anyhow::bail!("No numeric values imported from {}", csv.display());
    }

    let values = if no_resample {
        samples.iter().map(|(_, value)| *value).collect()
    } else {
        let Some(_) = time_idx else {
            anyhow::bail!(
                "CSV has no '{}' column for resampling. Use --no-resample to import raw samples.",
                time_column
            );
        };
        let timed: Vec<(f64, f64)> = samples
            .into_iter()
            .filter_map(|(time, value)| time.map(|time| (time, value)))
            .collect();
        if timed.len() < 2 {
            anyhow::bail!(
                "Need at least two timed samples to resample {}",
                csv.display()
            );
        }
        let last_time = timed.last().map(|(time, _)| *time).unwrap_or(0.0);
        let duration = duration.unwrap_or(last_time);
        let sample_rate = (cli.sample_rate * cli.oversample) as f64;
        resample_linear(&timed, sample_rate, duration)
    };
    if values.is_empty() {
        anyhow::bail!(
            "Imported trace produced zero samples; check --duration and sample-rate settings"
        );
    }

    let golden_path = cli
        .golden
        .join(suite)
        .join(test)
        .join(format!("{signal}.npy"));
    pedalkernel_validate::npy::write_f64(&golden_path, &values)?;
    println!(
        "{} Imported {} samples from {} column '{}' -> {}",
        "✓".green(),
        values.len(),
        csv.display(),
        column,
        golden_path.display()
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)] // import helper: each arg is a distinct trace selector
fn import_ltspice_raw_golden(
    cli: &Cli,
    raw: &std::path::Path,
    suite: &str,
    test: &str,
    signal: &str,
    trace: &str,
    duration: Option<f64>,
    no_resample: bool,
) -> anyhow::Result<()> {
    let samples = read_ltspice_raw_trace(raw, trace)?;
    let values: Vec<f64> = if no_resample {
        samples.iter().map(|(_, value)| *value).collect()
    } else {
        let last_time = samples.last().map(|(time, _)| *time).unwrap_or(0.0);
        let duration = duration.unwrap_or(last_time);
        let sample_rate = (cli.sample_rate * cli.oversample) as f64;
        resample_linear(&samples, sample_rate, duration)
    };
    if values.is_empty() {
        anyhow::bail!(
            "Imported trace produced zero samples; check --duration and sample-rate settings"
        );
    }

    let golden_path = cli
        .golden
        .join(suite)
        .join(test)
        .join(format!("{signal}.npy"));
    pedalkernel_validate::npy::write_f64(&golden_path, &values)?;
    println!(
        "{} Imported {} samples from {} trace '{}' -> {}",
        "✓".green(),
        values.len(),
        raw.display(),
        trace,
        golden_path.display()
    );
    Ok(())
}

fn read_ltspice_raw_trace(raw: &std::path::Path, trace: &str) -> anyhow::Result<Vec<(f64, f64)>> {
    let bytes = std::fs::read(raw)?;
    let marker = utf16le_bytes("Binary:\n");
    let header_end = bytes
        .windows(marker.len())
        .position(|window| window == marker.as_slice())
        .map(|pos| pos + marker.len())
        .ok_or_else(|| anyhow::anyhow!("LTspice raw Binary marker not found: {}", raw.display()))?;
    let header = decode_utf16le(&bytes[..header_end])?;
    let n_vars = parse_ltspice_header_usize(&header, "No. Variables:")?;
    let n_points = parse_ltspice_header_usize(&header, "No. Points:")?;
    let variables = parse_ltspice_variables(&header);
    if variables.len() != n_vars {
        anyhow::bail!(
            "Expected {} variables in {}, found {}",
            n_vars,
            raw.display(),
            variables.len()
        );
    }
    let time_idx = variables
        .iter()
        .position(|name| name == "time")
        .ok_or_else(|| anyhow::anyhow!("LTspice raw has no time variable: {}", raw.display()))?;
    if time_idx != 0 {
        anyhow::bail!("Unsupported LTspice raw: time variable is not first");
    }
    let trace_idx = variables
        .iter()
        .position(|name| name == trace)
        .ok_or_else(|| {
            anyhow::anyhow!(
                "Trace '{}' not found in {}. Available traces: {}",
                trace,
                raw.display(),
                variables.join(", ")
            )
        })?;

    let row_len = 8 + 4 * (n_vars - 1);
    let data = &bytes[header_end..];
    if data.len() < row_len * n_points {
        anyhow::bail!(
            "LTspice raw data too short: {} bytes for {} points x {} bytes",
            data.len(),
            n_points,
            row_len
        );
    }

    let mut samples = Vec::with_capacity(n_points);
    for point in 0..n_points {
        let row = &data[point * row_len..(point + 1) * row_len];
        let time = f64::from_le_bytes(row[0..8].try_into().unwrap());
        let value = if trace_idx == 0 {
            time
        } else {
            let offset = 8 + 4 * (trace_idx - 1);
            f32::from_le_bytes(row[offset..offset + 4].try_into().unwrap()) as f64
        };
        samples.push((time, value));
    }

    Ok(samples)
}

fn split_csv_line(line: &str) -> Vec<String> {
    line.split(',')
        .map(|field| field.trim().trim_matches('"').to_string())
        .collect()
}

fn parse_spice_float(raw: &str) -> Result<f64, std::num::ParseFloatError> {
    raw.trim().trim_matches('"').parse::<f64>()
}

fn utf16le_bytes(text: &str) -> Vec<u8> {
    text.encode_utf16()
        .flat_map(|unit| unit.to_le_bytes())
        .collect()
}

fn decode_utf16le(bytes: &[u8]) -> anyhow::Result<String> {
    let units: Vec<u16> = bytes
        .chunks_exact(2)
        .map(|chunk| u16::from_le_bytes([chunk[0], chunk[1]]))
        .collect();
    String::from_utf16(&units).map_err(|e| anyhow::anyhow!("Invalid UTF-16LE header: {e}"))
}

fn parse_ltspice_header_usize(header: &str, key: &str) -> anyhow::Result<usize> {
    header
        .lines()
        .find_map(|line| line.strip_prefix(key))
        .ok_or_else(|| anyhow::anyhow!("LTspice raw header missing '{}'", key))?
        .trim()
        .parse::<usize>()
        .map_err(|e| anyhow::anyhow!("Invalid LTspice raw '{}' value: {e}", key))
}

fn parse_ltspice_variables(header: &str) -> Vec<String> {
    let mut variables = Vec::new();
    let mut in_vars = false;
    for line in header.lines() {
        if line == "Variables:" {
            in_vars = true;
            continue;
        }
        if line == "Binary:" {
            break;
        }
        if !in_vars {
            continue;
        }
        let fields: Vec<_> = line.split_whitespace().collect();
        if fields.len() >= 2 {
            variables.push(fields[1].to_string());
        }
    }
    variables
}

fn resample_linear(samples: &[(f64, f64)], sample_rate: f64, duration: f64) -> Vec<f64> {
    let n_samples = (duration * sample_rate) as usize;
    let mut out = Vec::with_capacity(n_samples);
    let mut j = 0usize;
    for i in 0..n_samples {
        let t = i as f64 / sample_rate;
        while j + 1 < samples.len() && samples[j + 1].0 < t {
            j += 1;
        }
        if j + 1 >= samples.len() {
            out.push(samples.last().map(|(_, v)| *v).unwrap_or(0.0));
            continue;
        }
        let (t0, y0) = samples[j];
        let (t1, y1) = samples[j + 1];
        if t1 <= t0 {
            out.push(y0);
            continue;
        }
        let frac = ((t - t0) / (t1 - t0)).clamp(0.0, 1.0);
        out.push(y0 + frac * (y1 - y0));
    }
    out
}

fn generate_spice_golden(
    cli: &Cli,
    suite: &str,
    test_filter: Option<&str>,
    spice_dir: &std::path::Path,
) -> anyhow::Result<()> {
    use pedalkernel_validate::npy;
    use pedalkernel_validate::spice::{SpiceConfig, SpiceRunner};

    println!("{} Generating SPICE golden references...", "▶".blue());

    // Check ngspice first
    match SpiceRunner::check_ngspice() {
        Ok(version) => println!("  Using: {}", version.dimmed()),
        Err(e) => {
            println!("{} {}", "✗".red(), e);
            std::process::exit(1);
        }
    }

    let config = SpiceConfig {
        sample_rate: cli.sample_rate,
        oversample: cli.oversample,
    };
    let runner = SpiceRunner::new(config.clone());

    println!(
        "  Sample rate: {} Hz × {}x = {} Hz internal",
        cli.sample_rate,
        cli.oversample,
        config.internal_rate()
    );

    // Load validation config
    let validation_config = if cli.config.exists() {
        ValidationConfig::load(&cli.config)?
    } else {
        ValidationConfig::default_config()
    };

    if suite == "all" && test_filter.is_some() {
        return Err(anyhow::anyhow!(
            "--test requires a specific --suite; use --suite <name> --test <test>"
        ));
    }

    // Build list of suites to process
    let suites_to_process: Vec<(String, pedalkernel_validate::TestSuite)> = if suite == "all" {
        validation_config
            .suites
            .iter()
            .map(|(n, s)| (n.clone(), s.clone()))
            .collect()
    } else {
        let mut suite_config = validation_config
            .suites
            .get(suite)
            .cloned()
            .ok_or_else(|| anyhow::anyhow!("Suite '{}' not found", suite))?;
        if let Some(test_name) = test_filter {
            let test_case = suite_config.tests.get(test_name).cloned().ok_or_else(|| {
                anyhow::anyhow!("Test '{}' not found in suite '{}'", test_name, suite)
            })?;
            suite_config.tests.clear();
            suite_config.tests.insert(test_name.to_string(), test_case);
        }
        vec![(suite.to_string(), suite_config)]
    };

    let mut total_generated = 0;
    let mut total_skipped = 0;
    let mut total_failed = 0;

    for (suite_name, suite_config) in &suites_to_process {
        println!("\n{} Suite: {}", "•".green(), suite_name.bold());

        for (test_name, test_case) in &suite_config.tests {
            // Look for corresponding .spice file
            let spice_path = spice_dir.join(&test_case.circuit.replace(".pedal", ".spice"));

            if !spice_path.exists() {
                println!(
                    "  {} {} - SPICE file not found: {}",
                    "⚠".yellow(),
                    test_name,
                    spice_path.display()
                );
                total_skipped += 1;
                continue;
            }

            println!("  {} Test: {}", "→".blue(), test_name);

            for signal_config in &test_case.signals {
                let label = signal_config.label();
                let spec = signal_config.to_spec();
                let input = spec.generate(config.internal_rate());

                print!("    {} {}... ", "◦".dimmed(), label);
                std::io::Write::flush(&mut std::io::stdout())?;

                match runner.simulate(&spice_path, &input, "v_out") {
                    Ok(output) => {
                        // Save as golden reference
                        let golden_path = cli
                            .golden
                            .join(&suite_name)
                            .join(test_name)
                            .join(format!("{}.npy", label));

                        npy::write_f64(&golden_path, &output)?;
                        println!("{} ({} samples)", "✓".green(), output.len());
                        total_generated += 1;
                    }
                    Err(e) => {
                        println!("{} {}", "✗".red(), e);
                        total_failed += 1;
                    }
                }
            }
        }
    }

    println!("\n{}", "─".repeat(50));
    println!(
        "Generated: {} | Skipped: {} | Failed: {}",
        total_generated.to_string().green(),
        total_skipped.to_string().yellow(),
        total_failed.to_string().red()
    );

    if total_failed > 0 {
        std::process::exit(1);
    }

    Ok(())
}
