use std::{
    env,
    io::{BufRead, BufReader, Write},
    net::{TcpListener, TcpStream},
};

use anyhow::{anyhow, Context, Result};
use pedalkernel_host_protocol::{
    ControlRequest, ControlResponse, HostCommand, HostResult, ParameterInfo, PluginDescriptor,
};
use vst3_host::{backends::CpalBackend, midi::MidiEvent, AudioHandle, Vst3Host};

const DEFAULT_LISTEN: &str = "127.0.0.1:9473";
struct Runtime {
    host: Option<Vst3Host>,
    audio: Option<AudioHandle>,
    instance: Option<String>,
}

impl Runtime {
    fn new() -> Self {
        Self {
            host: None,
            audio: None,
            instance: None,
        }
    }

    fn execute(&mut self, command: HostCommand) -> Result<HostResult> {
        command.validate()?;
        match command {
            HostCommand::Configure { session, .. } => {
                if self.audio.is_some() {
                    return Err(anyhow!("unload the active plugin before reconfiguring"));
                }
                self.host = Some(
                    Vst3Host::builder()
                        .sample_rate(f64::from(session.sample_rate_hz))
                        .block_size(session.block_frames as usize)
                        .scan_default_paths()
                        .build()?,
                );
                Ok(HostResult::Ok)
            }
            HostCommand::Discover => {
                let plugins = self
                    .host_mut()?
                    .discover_plugins()?
                    .into_iter()
                    .map(|p| PluginDescriptor {
                        bundle_path: p.path.to_string_lossy().into_owned(),
                        name: p.name,
                        vendor: p.vendor,
                        version: p.version,
                        category: p.category,
                        class_id: p.uid,
                        audio_inputs: p.audio_inputs,
                        audio_outputs: p.audio_outputs,
                        has_midi_input: p.has_midi_input,
                        has_midi_output: p.has_midi_output,
                        has_gui: p.has_gui,
                    })
                    .collect();
                Ok(HostResult::Plugins { plugins })
            }
            HostCommand::ListAudioDevices => Ok(HostResult::AudioDevices {
                outputs: CpalBackend::new()?.list_output_devices()?,
            }),
            HostCommand::Status => Ok(HostResult::Status {
                configured: self.host.is_some(),
                instance: self.instance.clone(),
            }),
            HostCommand::LoadVst3 {
                instance,
                class_id,
                bundle_path,
                ..
            } => {
                if self.audio.is_some() {
                    return Err(anyhow!("one plugin instance is supported; unload it first"));
                }
                let plugin = self.host_mut()?.load_plugin_class(bundle_path, &class_id)?;
                let audio = self.host_ref()?.play(plugin)?;
                self.audio = Some(audio);
                self.instance = Some(instance);
                Ok(HostResult::Ok)
            }
            HostCommand::GetParameters { instance } => {
                self.require_instance(&instance)?;
                let parameters = self
                    .audio_ref()?
                    .lock()
                    .get_parameters()?
                    .into_iter()
                    .map(|p| ParameterInfo {
                        id: p.id,
                        name: p.name,
                        normalized: p.value,
                        default_normalized: p.default,
                        unit: p.unit,
                        step_count: p.step_count,
                        can_automate: p.can_automate,
                        read_only: p.is_read_only,
                        bypass: p.is_bypass,
                    })
                    .collect();
                Ok(HostResult::Parameters {
                    instance,
                    parameters,
                })
            }
            HostCommand::SetParameter {
                instance,
                parameter_id,
                normalized,
                ..
            } => {
                self.require_instance(&instance)?;
                if !self.audio_ref()?.set_parameter(parameter_id, normalized) {
                    return Err(anyhow!("realtime command queue is full"));
                }
                Ok(HostResult::Ok)
            }
            HostCommand::Midi {
                instance,
                bytes,
                sample_offset,
            } => {
                self.require_instance(&instance)?;
                let event = MidiEvent::from_midi_bytes(&bytes)
                    .ok_or_else(|| anyhow!("unsupported or malformed MIDI message"))?;
                if !self.audio_ref()?.send_midi_at(event, sample_offset as i32) {
                    return Err(anyhow!("realtime command queue is full"));
                }
                Ok(HostResult::Ok)
            }
            HostCommand::SetBypass { instance, bypassed } => {
                self.require_instance(&instance)?;
                let bypass = self
                    .audio_ref()?
                    .lock()
                    .get_parameters()?
                    .into_iter()
                    .find(|parameter| parameter.is_bypass)
                    .ok_or_else(|| anyhow!("plugin does not expose a bypass parameter"))?;
                if !self
                    .audio_ref()?
                    .set_parameter(bypass.id, f64::from(bypassed))
                {
                    return Err(anyhow!("realtime command queue is full"));
                }
                Ok(HostResult::Ok)
            }
            HostCommand::Unload { instance } => {
                self.require_instance(&instance)?;
                self.audio.take();
                self.instance.take();
                Ok(HostResult::Ok)
            }
            HostCommand::Stop => {
                self.audio.take();
                self.instance.take();
                self.host.take();
                Ok(HostResult::Ok)
            }
        }
    }

    fn host_mut(&mut self) -> Result<&mut Vst3Host> {
        self.host
            .as_mut()
            .ok_or_else(|| anyhow!("host is not configured"))
    }

    fn host_ref(&self) -> Result<&Vst3Host> {
        self.host
            .as_ref()
            .ok_or_else(|| anyhow!("host is not configured"))
    }

    fn audio_ref(&self) -> Result<&AudioHandle> {
        self.audio
            .as_ref()
            .ok_or_else(|| anyhow!("no plugin is loaded"))
    }

    fn require_instance(&self, requested: &str) -> Result<()> {
        match self.instance.as_deref() {
            Some(active) if active == requested => Ok(()),
            Some(active) => Err(anyhow!(
                "instance {requested:?} is not active; active instance is {active:?}"
            )),
            None => Err(anyhow!("no plugin is loaded")),
        }
    }
}

fn main() -> Result<()> {
    let listen = env::args().nth(1).unwrap_or_else(|| DEFAULT_LISTEN.into());
    let listener = TcpListener::bind(&listen)
        .with_context(|| format!("failed to bind control socket {listen}"))?;
    eprintln!("pedalkernel-studio-host listening on {listen}");

    let mut runtime = Runtime::new();
    for connection in listener.incoming() {
        match connection {
            Ok(stream) => {
                if let Err(error) = serve_connection(stream, &mut runtime) {
                    eprintln!("control connection failed: {error:#}");
                }
            }
            Err(error) => eprintln!("failed to accept control connection: {error}"),
        }
    }
    Ok(())
}

fn serve_connection(mut stream: TcpStream, runtime: &mut Runtime) -> Result<()> {
    let peer = stream.peer_addr()?;
    if !peer.ip().is_loopback() {
        return Err(anyhow!(
            "refusing non-loopback controller {peer}; use an SSH tunnel"
        ));
    }
    let mut reader = BufReader::new(stream.try_clone()?);
    let mut line = String::new();
    while reader.read_line(&mut line)? != 0 {
        let response = match serde_json::from_str::<ControlRequest>(line.trim_end()) {
            Ok(request) => {
                let result =
                    runtime
                        .execute(request.command)
                        .unwrap_or_else(|error| HostResult::Error {
                            message: format!("{error:#}"),
                        });
                ControlResponse {
                    request_id: request.request_id,
                    result,
                }
            }
            Err(error) => ControlResponse {
                request_id: String::new(),
                result: HostResult::Error {
                    message: format!("invalid request: {error}"),
                },
            },
        };
        serde_json::to_writer(&mut stream, &response)?;
        stream.write_all(b"\n")?;
        stream.flush()?;
        line.clear();
    }
    Ok(())
}
