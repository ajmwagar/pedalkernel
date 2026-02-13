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
    /// Offline WAV processor — run a .pedal file on an input WAV.
    Process {
        /// Path to the .pedal circuit definition file.
        pedal: String,
        /// Input WAV file.
        input: String,
        /// Output WAV file.
        output: String,
        /// Knob overrides (e.g. Drive=0.8 Volume=0.5).
        #[arg(trailing_var_arg = true)]
        knobs: Vec<String>,
    },
    /// Interactive TUI with live JACK audio — select I/O ports and tweak knobs.
    #[cfg(all(feature = "jack-rt", feature = "tui"))]
    Tui {
        /// Path to the .pedal circuit definition file.
        pedal: String,
    },
}

fn main() {
    let cli = Cli::parse();

    match cli.command {
        Command::Process {
            pedal,
            input,
            output,
            knobs,
        } => cli::process::run(&pedal, &input, &output, &knobs),
        #[cfg(all(feature = "jack-rt", feature = "tui"))]
        Command::Tui { pedal } => {
            if let Err(e) = cli::tui::run(&pedal) {
                eprintln!("Error: {e}");
                std::process::exit(1);
            }
        }
    }
}
