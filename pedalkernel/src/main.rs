#![allow(clippy::too_many_arguments)]
#![allow(clippy::unnecessary_map_or)]
#![allow(clippy::needless_borrows_for_generic_args)]
#![allow(clippy::manual_map)]
#![allow(clippy::single_match)]
#![allow(dead_code)]
#![allow(unused_imports)]
mod cli;

use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(
    name = "pedalkernel",
    about = "Guitar pedal DSP engine — DSL → WDF → audio"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Offline WAV processor — run a .pedal or .board file on an input WAV.
    Process {
        /// Path to the .pedal or .board file.
        file: String,
        /// Input WAV file.
        input: String,
        /// Output WAV file.
        output: String,
        /// Optional companion .pedalhw file for presets and mods.
        #[arg(long)]
        pedalhw: Option<String>,
        /// Factory preset name from .pedalhw [[presets]].
        #[arg(long)]
        preset: Option<String>,
        /// Named .pedalhw [[mods]] entry to apply. Repeat to stack mods.
        #[arg(long = "mod")]
        mods: Vec<String>,
        /// Disable the .pedal file's calibrate output normalization for this render.
        #[arg(long)]
        no_calibrate: bool,
        /// Knob overrides (e.g. Drive=0.8 or ts.Drive=0.7 for boards).
        knobs: Vec<String>,
    },
    /// Debug a .pedal file — compile and print stage structure, WDF trees,
    /// scattering matrices, controls, and other internal state.
    Debug {
        /// Path to the .pedal file.
        file: String,
        /// Emit a structured JSON StageGraph instead of the prose dump.
        #[arg(long)]
        json: bool,
    },
    /// Diagnostics — compile-time MNA/scattering snapshot over a memory-mapped
    /// IPC file (Phase A of the diagnostics IPC channel).
    ///
    /// Examples:
    ///   pedalkernel diag my.pedal                       # compile + print
    ///   pedalkernel diag my.pedal --ipc out.pkdiag      # also write IPC file
    ///   pedalkernel diag --ipc out.pkdiag               # read + print IPC file
    ///   pedalkernel diag my.pedal --knob Gain=0.5       # apply a knob first
    ///   pedalkernel diag my.pedal --ipc out.pkdiag --tail   # Phase B: live ring
    ///   pedalkernel diag --ipc out.pkdiag --tail            # tail an existing file
    #[cfg(feature = "diag")]
    Diag {
        /// Path to the .pedal file to compile (omit to read --ipc only).
        file: Option<String>,
        /// IPC file: written (with a pedal) or read (without).
        #[arg(long)]
        ipc: Option<String>,
        /// Control overrides applied before capture (e.g. --knob Gain=0.5).
        #[arg(long = "knob")]
        knobs: Vec<String>,
        /// Phase B: drive the pedal and live-tail the runtime ring (per-block
        /// NR convergence + per-BJT op-points + per-stage levels). With a
        /// `.pedal` file it runs the engine; with only `--ipc` it tails an
        /// existing file written by another process.
        #[arg(long)]
        tail: bool,
        /// Tail: test-sine frequency (Hz). Default 1000.
        #[arg(long, default_value_t = 1000.0)]
        tail_freq: f64,
        /// Tail: test-sine amplitude (linear). Default 0.1.
        #[arg(long, default_value_t = 0.1)]
        tail_amp: f64,
        /// Tail: number of blocks to capture before exiting. Default 64.
        #[arg(long, default_value_t = 64)]
        tail_blocks: usize,
    },
    /// Validate .pedal files — check for common errors and warnings.
    ///
    /// Accepts files, directories (recursive), or glob patterns.
    /// Examples:
    ///   pedalkernel validate examples/
    ///   pedalkernel validate "examples/**/*.pedal"
    ///   pedalkernel validate my_pedal.pedal --fix
    Validate {
        /// Paths to .pedal files, directories, or glob patterns.
        #[arg(required = true)]
        paths: Vec<String>,
        /// Auto-fix obvious issues (e.g., pin renames) in place.
        #[arg(long)]
        fix: bool,
    },
    /// Export a KiCad netlist (S-expression) from a .pedal file.
    ///
    /// Subcircuit use(...) instances are expanded before export, so the
    /// netlist covers the full flattened circuit.
    ExportKicad {
        /// Path to the .pedal file.
        file: String,
        /// Output file (defaults to stdout).
        #[arg(short, long)]
        output: Option<String>,
    },
    /// Export a bill of materials from a .pedal file.
    ///
    /// Prints per-refdes rows (refdes, category, value, model, qty) plus an
    /// aggregated view grouped by (category, value, model). A diode_pair
    /// counts as 2 physical diodes.
    ExportBom {
        /// Path to the .pedal file.
        file: String,
        /// Output format.
        #[arg(long, value_enum, default_value_t = cli::export_bom::BomFormat::Table)]
        format: cli::export_bom::BomFormat,
        /// Output file (defaults to stdout).
        #[arg(short, long)]
        output: Option<String>,
    },
    /// List and search available component models (BJTs, JFETs, op-amps, transformers, etc.)
    ///
    /// Examples:
    ///   pedalkernel models                   # List all models
    ///   pedalkernel models --type bjt         # Only BJTs
    ///   pedalkernel models --type jfet -s 2N5 # Search JFETs matching "2N5"
    ///   pedalkernel models --type opamp      # Only op-amps and OTAs
    ///   pedalkernel models --type transformer # Only transformers
    ///   pedalkernel models --show 2N3904      # Show full details for a model
    Models {
        /// Filter by component type: bjt, npn, pnp, jfet, njf, pjf, triode, pentode, opamp, ota, transformer
        #[arg(short = 't', long = "type")]
        model_type: Option<String>,
        /// Search model names (case-insensitive substring match)
        #[arg(short, long)]
        search: Option<String>,
        /// Show full details for a specific model
        #[arg(long)]
        show: Option<String>,
    },
    /// Interactive TUI with live JACK audio — select I/O ports and tweak knobs.
    #[cfg(all(feature = "jack-rt", feature = "tui"))]
    Tui {
        /// Path to a .pedal file, .board file, or directory (default: current dir).
        file: Option<String>,
        /// Optional WAV file to loop as audio input instead of a JACK input port.
        #[arg(long)]
        input: Option<String>,
    },
}

fn main() {
    let cli = Cli::parse();

    match cli.command {
        Command::Process {
            file,
            input,
            output,
            pedalhw,
            preset,
            mods,
            no_calibrate,
            knobs,
        } => cli::process::run(
            &file,
            &input,
            &output,
            cli::process::ProcessOptions {
                pedalhw: pedalhw.as_deref(),
                preset: preset.as_deref(),
                mods: &mods,
                no_calibrate,
            },
            &knobs,
        ),
        Command::Debug { file, json } => cli::debug::run(&file, json),
        #[cfg(feature = "diag")]
        Command::Diag {
            file,
            ipc,
            knobs,
            tail,
            tail_freq,
            tail_amp,
            tail_blocks,
        } => {
            if tail {
                cli::diag::run_tail(
                    file.as_deref(),
                    ipc.as_deref(),
                    &knobs,
                    tail_freq,
                    tail_amp,
                    tail_blocks,
                );
            } else {
                cli::diag::run(file.as_deref(), ipc.as_deref(), &knobs);
            }
        }
        Command::ExportKicad { file, output } => cli::export_kicad::run(&file, output.as_deref()),
        Command::ExportBom {
            file,
            format,
            output,
        } => cli::export_bom::run(&file, format, output.as_deref()),
        Command::Validate { paths, fix } => cli::validate::run(&paths, fix),
        Command::Models {
            model_type,
            search,
            show,
        } => {
            if let Some(name) = show {
                cli::models::show(&name);
            } else {
                cli::models::run(model_type.as_deref(), search.as_deref());
            }
        }
        #[cfg(all(feature = "jack-rt", feature = "tui"))]
        Command::Tui { file, input } => {
            let wav_input = input.as_deref();
            let mut current_file = file.unwrap_or_else(|| ".".to_string());

            loop {
                let path = std::path::Path::new(&current_file);
                let result = if path.is_dir() {
                    // Check for .board files first (existing behavior)
                    let has_boards = std::fs::read_dir(&current_file)
                        .ok()
                        .map(|rd| {
                            rd.flatten().any(|e| {
                                e.path()
                                    .extension()
                                    .and_then(|x| x.to_str())
                                    .map(|x| x == "board")
                                    .unwrap_or(false)
                            })
                        })
                        .unwrap_or(false);
                    if has_boards {
                        cli::folder_tui::run(&current_file, wav_input)
                    } else {
                        cli::pedal_folder_tui::run(&current_file, wav_input)
                    }
                } else if current_file.ends_with(".board") {
                    cli::board_tui::run(&current_file, wav_input)
                } else {
                    cli::tui::run(&current_file, wav_input)
                };

                match result {
                    Ok(Some(new_file)) => {
                        current_file = new_file;
                    }
                    Ok(None) => break,
                    Err(e) => {
                        eprintln!("Error: {e}");
                        std::process::exit(1);
                    }
                }
            }
        }
    }
}
