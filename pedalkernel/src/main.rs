mod cli;

use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(name = "pedalkernel", about = "Guitar pedal DSP engine — DSL → WDF → audio")]
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
    /// Interactive TUI with live JACK audio — select I/O ports and tweak knobs.
    #[cfg(all(feature = "jack-rt", feature = "tui"))]
    Tui {
        /// Path to the .pedal or .board file.
        file: String,
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
        #[cfg(all(feature = "jack-rt", feature = "tui"))]
        Command::Tui { file } => {
            let result = if file.ends_with(".board") {
                cli::board_tui::run(&file)
            } else {
                cli::tui::run(&file)
            };
            if let Err(e) = result {
                eprintln!("Error: {e}");
                std::process::exit(1);
            }
        }
    }
}
