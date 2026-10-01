use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand};
use pedalkernel_core::HostCommand;

#[derive(Parser)]
#[command(about = "PedalKernel VST3 host control client")]
struct Args {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Validate a JSON host command. Transport is intentionally a future host concern.
    Validate { json: String },
}

fn main() -> Result<()> {
    let args = Args::parse();
    match args.command {
        Command::Validate { json } => {
            let command: HostCommand =
                serde_json::from_str(&json).context("invalid command JSON")?;
            if let Err(error) = command.validate() {
                bail!(error);
            }
            println!("valid");
        }
    }
    Ok(())
}
