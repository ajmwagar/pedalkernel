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
        /// Knob overrides (e.g. Drive=0.8 or ts.Drive=0.7 for boards).
        #[arg(trailing_var_arg = true)]
        knobs: Vec<String>,
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
            knobs,
        } => cli::process::run(&file, &input, &output, &knobs),
        Command::Validate { paths, fix } => cli::validate::run(&paths, fix),
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
