use std::{
    env,
    io::{BufRead, BufReader, Write},
    net::TcpStream,
};

use anyhow::{bail, Context, Result};
use pedalkernel_host_protocol::{ControlRequest, ControlResponse, HostCommand};

const DEFAULT_ENDPOINT: &str = "127.0.0.1:9473";

fn main() -> Result<()> {
    let mut args = env::args().skip(1).collect::<Vec<_>>();
    let endpoint = if args.first().is_some_and(|arg| arg == "--endpoint") {
        if args.len() < 3 {
            bail!("--endpoint requires an address and a command");
        }
        args.remove(0);
        args.remove(0)
    } else {
        DEFAULT_ENDPOINT.to_owned()
    };
    let command = parse_command(&args)?;
    let request = ControlRequest {
        request_id: format!("ctl-{}", std::process::id()),
        command,
    };
    let mut stream = TcpStream::connect(&endpoint)
        .with_context(|| format!("connect to PedalKernel host at {endpoint}"))?;
    serde_json::to_writer(&mut stream, &request).context("encode control request")?;
    stream.write_all(b"\n").context("send control request")?;
    stream.flush().context("flush control request")?;

    let mut response = String::new();
    BufReader::new(stream)
        .read_line(&mut response)
        .context("read control response")?;
    let response: ControlResponse =
        serde_json::from_str(&response).context("decode control response")?;
    println!("{}", serde_json::to_string_pretty(&response)?);
    if matches!(
        response.result,
        pedalkernel_host_protocol::HostResult::Error { .. }
    ) {
        std::process::exit(1);
    }
    Ok(())
}

fn parse_command(args: &[String]) -> Result<HostCommand> {
    match args {
        [command] if command == "status" => Ok(HostCommand::Status),
        [command] if command == "surface-status" => Ok(HostCommand::EditorSurfaceStatus),
        [command, instance] if command == "editor-open" => Ok(HostCommand::OpenEditor {
            instance: instance.clone(),
        }),
        [command, instance] if command == "editor-close" => Ok(HostCommand::CloseEditor {
            instance: instance.clone(),
        }),
        [command, instance, title] if command == "surface-start" => {
            Ok(HostCommand::StartEditorSurface {
                instance: instance.clone(),
                title_contains: title.clone(),
                max_fps: 30,
            })
        }
        [command, instance, title, max_fps] if command == "surface-start" => {
            Ok(HostCommand::StartEditorSurface {
                instance: instance.clone(),
                title_contains: title.clone(),
                max_fps: max_fps
                    .parse()
                    .context("max_fps must be an integer in 1..=120")?,
            })
        }
        [command, instance] if command == "surface-stop" => {
            Ok(HostCommand::StopEditorSurface {
                instance: instance.clone(),
            })
        }
        [command, instance, gain] if command == "gain" => Ok(HostCommand::SetOutputGain {
            instance: instance.clone(),
            gain: gain.parse().context("gain must be a number in 0..=2")?,
        }),
        [command, instance] if command == "mute" || command == "unmute" => {
            Ok(HostCommand::SetOutputMute {
                instance: instance.clone(),
                muted: command == "mute",
            })
        }
        _ => bail!(
            "usage: pedalkernel-studio-ctl [--endpoint HOST:PORT] status | editor-open <instance> | editor-close <instance> | surface-start <instance> <title-substring> [max-fps] | surface-stop <instance> | surface-status | gain <instance> <0..2> | mute <instance> | unmute <instance>"
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_output_controls() {
        assert_eq!(
            parse_command(&["gain".into(), "surge".into(), "0.25".into()]).unwrap(),
            HostCommand::SetOutputGain {
                instance: "surge".into(),
                gain: 0.25,
            }
        );
        assert_eq!(
            parse_command(&["mute".into(), "surge".into()]).unwrap(),
            HostCommand::SetOutputMute {
                instance: "surge".into(),
                muted: true,
            }
        );
    }

    #[test]
    fn parses_editor_surface_lifecycle() {
        assert_eq!(
            parse_command(&[
                "surface-start".into(),
                "surge".into(),
                "Surge XT".into(),
                "60".into(),
            ])
            .unwrap(),
            HostCommand::StartEditorSurface {
                instance: "surge".into(),
                title_contains: "Surge XT".into(),
                max_fps: 60,
            }
        );
        assert_eq!(
            parse_command(&["surface-stop".into(), "surge".into()]).unwrap(),
            HostCommand::StopEditorSurface {
                instance: "surge".into(),
            }
        );
        assert_eq!(
            parse_command(&["surface-status".into()]).unwrap(),
            HostCommand::EditorSurfaceStatus
        );
    }
}
