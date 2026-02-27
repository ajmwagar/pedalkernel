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
    config::ValidationConfig,
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

    /// Bootstrap golden references from current WDF output (for regression testing)
    Bootstrap {
        /// Test suite to bootstrap
        #[arg(short, long, default_value = "all")]
        suite: String,
    },

    /// Generate golden references from SPICE simulation
    GenerateSpice {
        /// Test suite to generate
        #[arg(short, long, default_value = "all")]
        suite: String,

        /// Directory containing .spice circuit files
        #[arg(long, default_value = "spice-circuits")]
        spice_dir: PathBuf,
    },

    /// Check if ngspice is available
    CheckSpice,
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
        Some(Commands::GenerateSpice { suite, spice_dir }) => {
            generate_spice_golden(&cli, suite, spice_dir)?;
        }
        Some(Commands::CheckSpice) => {
            check_spice()?;
        }
        None => {
            run_validation(&cli, "all", None)?;
        }
    }

    Ok(())
}

fn run_validation(cli: &Cli, suite: &str, test: Option<String>) -> anyhow::Result<()> {
    println!(
        "{} Loading configuration...",
        "▶".blue()
    );

    // Load or create default config
    let validation_config = if cli.config.exists() {
        ValidationConfig::load(&cli.config)?
    } else {
        println!(
            "  {} Config not found, using defaults",
            "⚠".yellow()
        );
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
            let test_case = suite_config
                .tests
                .get(test_name)
                .ok_or_else(|| anyhow::anyhow!("Test '{}' not found in suite '{}'", test_name, suite))?;

            let mut filtered = suite_config.clone();
            filtered.tests.clear();
            filtered.tests.insert(test_name.clone(), test_case.clone());
            filtered
        } else {
            suite_config.clone()
        };

        let result = runner.run_suite(suite, &suite_to_run)?;
        let mut results = std::collections::HashMap::new();
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
        println!("\n{} - {}", suite_name.bold().blue(), suite.description.dimmed());

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

    println!("{} Created default config at: {}", "✓".green(), path.display());
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

fn quick_validate(cli: &Cli, circuit: &PathBuf, golden: Option<&std::path::Path>) -> anyhow::Result<()> {
    use pedalkernel_validate::runner::quick_validate as qv;

    println!(
        "{} Quick validation of: {}",
        "▶".blue(),
        circuit.display()
    );

    let sample_rate = (cli.sample_rate * cli.oversample) as f64;
    let result = qv(circuit, golden, sample_rate)?;

    println!("\n{}", "Results".bold());
    println!("{}", "─".repeat(40));
    println!("Normalized RMS Error: {:.2} dB", result.normalized_rms_error_db);
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

    println!("{} Bootstrapping golden references from WDF output...", "▶".blue());

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
                println!("    {} Circuit not found: {}", "⚠".yellow(), circuit_path.display());
                continue;
            }

            // Load and compile circuit
            let contents = std::fs::read_to_string(&circuit_path)?;
            let pedal_def = pedalkernel::dsl::parse_pedal_file(&contents)
                .map_err(|e| anyhow::anyhow!("Parse error: {}", e))?;
            let mut pedal = pedalkernel::compiler::compile_pedal(&pedal_def, sample_rate)
                .map_err(|e| anyhow::anyhow!("Compile error: {}", e))?;

            for signal_config in &test_case.signals {
                let label = signal_config.label();
                let spec = signal_config.to_spec();
                let input = spec.generate(sample_rate);

                // Reset and process
                use pedalkernel::PedalProcessor;
                pedal.reset();
                let output: Vec<f64> = input.iter().map(|&s| pedal.process(s)).collect();

                // Save as golden
                let golden_path = cli.golden
                    .join(suite_name)
                    .join(test_name)
                    .join(format!("{}.npy", label));

                npy::write_f64(&golden_path, &output)?;
                println!("    {} {} → {}", "✓".green(), label, golden_path.display());
            }
        }
    }

    println!("\n{}", "Done! Golden references bootstrapped from WDF.".green().bold());
    println!("These can be used for regression testing future WDF changes.");

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
    println!("  {} Wrote impulse response: {}", "✓".green(), impulse_path.display());

    // Generate sine response (for THD sanity check - linear circuit should have ~0 THD)
    let sine_input = signals::sine(sample_rate, 1000.0, 0.1, 1.0);
    let sine_golden = analytical::rc_lowpass_filter(&sine_input, r, c, sample_rate);

    let sine_path = cli.golden.join("linear/rc_lowpass/sine.npy");
    npy::write_f64(&sine_path, &sine_golden)?;
    println!("  {} Wrote sine response: {}", "✓".green(), sine_path.display());

    // Generate sweep response
    let sweep_input = signals::exp_sweep(sample_rate, 20.0, 20000.0, 1.0, 1.0);
    let sweep_golden = analytical::rc_lowpass_filter(&sweep_input, r, c, sample_rate);

    let sweep_path = cli.golden.join("linear/rc_lowpass/sweep.npy");
    npy::write_f64(&sweep_path, &sweep_golden)?;
    println!("  {} Wrote sweep response: {}", "✓".green(), sweep_path.display());

    println!("\n{}", "Done! Golden references generated.".green().bold());
    println!("Run 'pedalkernel-validate run --suite linear' to validate.");

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

fn generate_spice_golden(cli: &Cli, suite: &str, spice_dir: &std::path::Path) -> anyhow::Result<()> {
    use pedalkernel_validate::spice::{SpiceConfig, SpiceRunner};
    use pedalkernel_validate::npy;

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

    println!("  Sample rate: {} Hz × {}x = {} Hz internal",
        cli.sample_rate, cli.oversample, config.internal_rate());

    // Load validation config
    let validation_config = if cli.config.exists() {
        ValidationConfig::load(&cli.config)?
    } else {
        ValidationConfig::default_config()
    };

    // Build list of suites to process
    let suites_to_process: Vec<(String, pedalkernel_validate::TestSuite)> = if suite == "all" {
        validation_config.suites.iter()
            .map(|(n, s)| (n.clone(), s.clone()))
            .collect()
    } else {
        let suite_config = validation_config
            .suites
            .get(suite)
            .ok_or_else(|| anyhow::anyhow!("Suite '{}' not found", suite))?;
        vec![(suite.to_string(), suite_config.clone())]
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
                println!("  {} {} - SPICE file not found: {}",
                    "⚠".yellow(), test_name, spice_path.display());
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
                        let golden_path = cli.golden
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
    println!("Generated: {} | Skipped: {} | Failed: {}",
        total_generated.to_string().green(),
        total_skipped.to_string().yellow(),
        total_failed.to_string().red()
    );

    if total_failed > 0 {
        std::process::exit(1);
    }

    Ok(())
}
