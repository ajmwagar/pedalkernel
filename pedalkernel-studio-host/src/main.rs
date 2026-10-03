use std::{
    env,
    io::{BufRead, BufReader, Write},
    net::{SocketAddr, TcpListener, TcpStream},
    path::Path,
    sync::{
        atomic::{AtomicBool, AtomicU32, Ordering},
        mpsc::{self, Receiver, SyncSender},
        Arc, Mutex,
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

use anyhow::{anyhow, Context, Result};
use pedalkernel_editor_surface::{
    EditorCapture, EditorSelector, FrameLease, NativeEditorCapture, NativeEditorFrame,
};
use pedalkernel_host_protocol::{
    ControlRequest, ControlResponse, HostCommand, HostResult, ParameterInfo, PluginDescriptor,
};
use vst3_host::{
    backends::CpalBackend,
    midi::MidiEvent,
    midi_input::{self, MidiInputConnection},
    play_with_backend, AudioBackend, AudioConfig, AudioHandle, AudioStream, PluginWindow, Vst3Host,
};

#[derive(Clone, Copy)]
struct IsochroneDevice;

struct IsochroneBackend {
    destination: SocketAddr,
    output: OutputControl,
}

#[derive(Clone)]
struct OutputControl {
    gain_bits: Arc<AtomicU32>,
    muted: Arc<AtomicBool>,
}

impl OutputControl {
    fn new(gain: f32) -> Self {
        Self {
            gain_bits: Arc::new(AtomicU32::new(gain.to_bits())),
            muted: Arc::new(AtomicBool::new(false)),
        }
    }

    fn set_gain(&self, gain: f32) {
        self.gain_bits.store(gain.to_bits(), Ordering::Release);
    }

    fn set_muted(&self, muted: bool) {
        self.muted.store(muted, Ordering::Release);
    }

    fn target_gain(&self) -> f32 {
        if self.muted.load(Ordering::Acquire) {
            0.0
        } else {
            f32::from_bits(self.gain_bits.load(Ordering::Acquire))
        }
    }

    fn configured_gain(&self) -> f32 {
        f32::from_bits(self.gain_bits.load(Ordering::Acquire))
    }

    fn is_muted(&self) -> bool {
        self.muted.load(Ordering::Acquire)
    }
}

struct IsochroneStream {
    running: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
    thread: Mutex<Option<JoinHandle<()>>>,
}

impl Drop for IsochroneStream {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(thread) = self.thread.lock().expect("stream thread lock").take() {
            let _ = thread.join();
        }
    }
}

impl AudioStream for IsochroneStream {
    fn play(&self) -> std::result::Result<(), Box<dyn std::error::Error>> {
        self.running.store(true, Ordering::Release);
        Ok(())
    }

    fn pause(&self) -> std::result::Result<(), Box<dyn std::error::Error>> {
        self.running.store(false, Ordering::Release);
        Ok(())
    }
}

impl AudioBackend for IsochroneBackend {
    type Stream = IsochroneStream;
    type Device = IsochroneDevice;
    type Error = std::io::Error;

    fn enumerate_output_devices(&self) -> std::io::Result<Vec<Self::Device>> {
        Ok(vec![IsochroneDevice])
    }

    fn enumerate_input_devices(&self) -> std::io::Result<Vec<Self::Device>> {
        Ok(Vec::new())
    }

    fn default_output_device(&self) -> Option<Self::Device> {
        Some(IsochroneDevice)
    }

    fn default_input_device(&self) -> Option<Self::Device> {
        None
    }

    fn create_output_stream(
        &self,
        _device: &Self::Device,
        config: AudioConfig,
        mut data_callback: Box<dyn FnMut(&mut [f32]) + Send>,
        mut error_callback: Box<dyn FnMut(Self::Error) + Send>,
    ) -> std::io::Result<Self::Stream> {
        if config.sample_rate as u32 != 48_000 || config.output_channels != 2 {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "Isochrone output requires 48 kHz stereo",
            ));
        }
        let format = isochrone_core::StreamFormat::aes67_48k_stereo();
        let ssrc = u32::from_be_bytes([0x50, 0x4b, 0x56, 0x31]);
        let mut sender = isochrone::UdpSender::connect(
            "0.0.0.0:0".parse().expect("constant socket address"),
            self.destination,
            format,
            ssrc,
        )?;
        let running = Arc::new(AtomicBool::new(false));
        let stop = Arc::new(AtomicBool::new(false));
        let thread_running = Arc::clone(&running);
        let thread_stop = Arc::clone(&stop);
        let output = self.output.clone();
        // VST3 call overhead dominates at a 48-frame quantum (notably in Surge).
        // Render ten RTP packets at once; the receiver's 20 ms playout buffer
        // smooths this 10 ms burst while the media timestamps remain 1 ms apart.
        let block_frames = config.block_size.max(480);
        let period = Duration::from_secs_f64(block_frames as f64 / config.sample_rate);
        let thread = thread::Builder::new()
            .name("pedalkernel-isochrone".into())
            .spawn(move || {
                let mut block = vec![0.0_f32; block_frames * 2];
                let mut packet = [0.0_f32; 96];
                let mut packet_len = 0;
                let mut current_gain = output.target_gain();
                let max_gain_step =
                    (2.0 / (config.sample_rate * 0.010 * 2.0).round().max(1.0)) as f32;
                let mut deadline = Instant::now();
                while !thread_stop.load(Ordering::Acquire) {
                    if !thread_running.load(Ordering::Acquire) {
                        thread::sleep(Duration::from_millis(1));
                        deadline = Instant::now();
                        continue;
                    }
                    data_callback(&mut block);
                    for &sample in &block {
                        let target_gain = output.target_gain();
                        let delta = target_gain - current_gain;
                        if delta.abs() > f32::EPSILON {
                            current_gain += delta.clamp(-max_gain_step, max_gain_step);
                        }
                        packet[packet_len] = sample * current_gain;
                        packet_len += 1;
                        if packet_len == packet.len() {
                            if let Err(error) = sender.send(&packet) {
                                error_callback(error);
                            }
                            packet_len = 0;
                        }
                    }
                    deadline += period;
                    if let Some(wait) = deadline.checked_duration_since(Instant::now()) {
                        thread::sleep(wait);
                    } else {
                        deadline = Instant::now();
                    }
                }
            })?;
        Ok(IsochroneStream {
            running,
            stop,
            thread: Mutex::new(Some(thread)),
        })
    }

    fn create_input_stream(
        &self,
        _device: &Self::Device,
        _config: AudioConfig,
        _data_callback: Box<dyn FnMut(&[f32]) + Send>,
        _error_callback: Box<dyn FnMut(Self::Error) + Send>,
    ) -> std::io::Result<Self::Stream> {
        Err(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "Isochrone instrument backend is output-only",
        ))
    }
}

const DEFAULT_LISTEN: &str = "127.0.0.1:9473";
struct Runtime {
    host: Option<Vst3Host>,
    audio: Option<AudioHandle>,
    instance: Option<String>,
    isochrone_destination: Option<SocketAddr>,
    output: OutputControl,
    editor: Option<PluginWindow>,
    editor_surface: Option<EditorSurfaceRuntime>,
    midi_queries: Vec<String>,
    midi_inputs: Vec<MidiInputConnection>,
}

impl Runtime {
    fn new(isochrone_destination: Option<SocketAddr>, midi_queries: Vec<String>) -> Self {
        Self {
            host: None,
            audio: None,
            instance: None,
            isochrone_destination,
            output: OutputControl::new(0.501_187_2),
            editor: None,
            editor_surface: None,
            midi_queries,
            midi_inputs: Vec::new(),
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
                    .map(plugin_descriptor)
                    .collect();
                Ok(HostResult::Plugins { plugins })
            }
            HostCommand::InspectVst3 { bundle_path } => {
                let plugin = vst3_host::get_detailed_plugin_info(Path::new(&bundle_path))?.info;
                Ok(HostResult::Plugins {
                    plugins: vec![plugin_descriptor(plugin)],
                })
            }
            HostCommand::ListAudioDevices => Ok(HostResult::AudioDevices {
                outputs: CpalBackend::new()?.list_output_devices()?,
            }),
            HostCommand::ListMidiInputs => Ok(HostResult::MidiInputs {
                inputs: midi_input::list_midi_input_ports()?
                    .iter()
                    .map(|port| port.name().to_owned())
                    .collect(),
            }),
            HostCommand::Status => Ok(HostResult::Status {
                configured: self.host.is_some(),
                instance: self.instance.clone(),
                output_gain: self.output.configured_gain(),
                output_muted: self.output.is_muted(),
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
                let audio = if let Some(destination) = self.isochrone_destination {
                    let backend = IsochroneBackend {
                        destination,
                        output: self.output.clone(),
                    };
                    let mut config = *self.host_ref()?.config();
                    config.input_channels = 0;
                    config.output_channels = 2;
                    play_with_backend(&backend, plugin, config)?
                } else {
                    self.host_ref()?.play(plugin)?
                };
                self.midi_inputs = bind_midi_inputs(&audio, &self.midi_queries)?;
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
            HostCommand::SetOutputGain { instance, gain } => {
                self.require_instance(&instance)?;
                if self.isochrone_destination.is_none() {
                    return Err(anyhow!(
                        "output gain control requires the Isochrone output backend"
                    ));
                }
                self.output.set_gain(gain);
                Ok(HostResult::Ok)
            }
            HostCommand::SetOutputMute { instance, muted } => {
                self.require_instance(&instance)?;
                if self.isochrone_destination.is_none() {
                    return Err(anyhow!(
                        "output mute control requires the Isochrone output backend"
                    ));
                }
                self.output.set_muted(muted);
                Ok(HostResult::Ok)
            }
            HostCommand::OpenEditor { instance } => {
                self.require_instance(&instance)?;
                if self.editor.is_some() {
                    return Err(anyhow!("the plugin editor is already open"));
                }
                let plugin = self.audio_ref()?.plugin();
                let (width, height) = plugin
                    .lock()
                    .unwrap_or_else(|poisoned| poisoned.into_inner())
                    .get_editor_size()
                    .unwrap_or((960, 640));
                let mut editor = PluginWindow::new(plugin);
                editor.open()?;
                activate_platform_ui();
                self.editor = Some(editor);
                Ok(HostResult::Editor {
                    instance,
                    open: true,
                    width: u32::try_from(width.max(1))?,
                    height: u32::try_from(height.max(1))?,
                })
            }
            HostCommand::CloseEditor { instance } => {
                self.require_instance(&instance)?;
                self.editor_surface = None;
                if let Some(mut editor) = self.editor.take() {
                    editor.close();
                }
                Ok(HostResult::Editor {
                    instance,
                    open: false,
                    width: 0,
                    height: 0,
                })
            }
            HostCommand::StartEditorSurface {
                instance,
                title_contains,
                max_fps,
            } => {
                self.require_instance(&instance)?;
                if self.editor.is_none() {
                    return Err(anyhow!(
                        "open the plugin editor before starting its surface"
                    ));
                }
                let mut surface = EditorSurfaceRuntime::open(title_contains, max_fps)?;
                surface.capture_now()?;
                self.editor_surface = Some(surface);
                Ok(self.editor_surface_result())
            }
            HostCommand::StopEditorSurface { instance } => {
                self.require_instance(&instance)?;
                self.editor_surface = None;
                Ok(self.editor_surface_result())
            }
            HostCommand::EditorSurfaceStatus => Ok(self.editor_surface_result()),
            HostCommand::Unload { instance } => {
                self.require_instance(&instance)?;
                self.editor_surface = None;
                if let Some(mut editor) = self.editor.take() {
                    editor.close();
                }
                self.midi_inputs.clear();
                self.audio.take();
                self.instance.take();
                Ok(HostResult::Ok)
            }
            HostCommand::Stop => {
                self.editor_surface = None;
                if let Some(mut editor) = self.editor.take() {
                    editor.close();
                }
                self.midi_inputs.clear();
                self.audio.take();
                self.instance.take();
                self.host.take();
                Ok(HostResult::Ok)
            }
        }
    }

    fn service_editor(&mut self) -> Result<()> {
        let Some(editor) = self.editor.as_mut() else {
            return Ok(());
        };
        if editor.closed_by_user() {
            self.editor_surface = None;
            editor.close();
            self.editor = None;
        } else {
            editor.service_platform_events()?;
        }
        Ok(())
    }

    fn service_editor_surface(&mut self) {
        let Some(surface) = self.editor_surface.as_mut() else {
            return;
        };
        if let Err(error) = surface.capture_if_due() {
            let message = format!("{error:#}");
            if surface.last_error.as_deref() != Some(&message) {
                eprintln!("editor surface capture failed: {message}");
            }
            surface.last_error = Some(message);
        }
    }

    fn editor_surface_result(&self) -> HostResult {
        let Some(surface) = self.editor_surface.as_ref() else {
            return HostResult::EditorSurface {
                active: false,
                backend: None,
                zero_copy: false,
                frame: None,
                error: None,
            };
        };
        let backend = surface.capture.backend();
        HostResult::EditorSurface {
            active: true,
            backend: Some(backend),
            zero_copy: backend.zero_copy(),
            frame: surface
                .current_frame
                .as_ref()
                .map(|frame| frame.descriptor().clone()),
            error: surface.last_error.clone(),
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

struct EditorSurfaceRuntime {
    capture: NativeEditorCapture,
    current_frame: Option<NativeEditorFrame>,
    frame_interval: Duration,
    next_capture: Instant,
    last_error: Option<String>,
}

impl EditorSurfaceRuntime {
    fn open(title_contains: String, max_fps: u16) -> Result<Self> {
        let capture = NativeEditorCapture::open(EditorSelector {
            process_id: std::process::id(),
            title_contains,
        })?;
        Ok(Self {
            capture,
            current_frame: None,
            frame_interval: Duration::from_secs_f64(1.0 / f64::from(max_fps)),
            next_capture: Instant::now(),
            last_error: None,
        })
    }

    fn capture_now(&mut self) -> Result<()> {
        // Capture completes before assignment, so the previously published
        // frame stays alive until its replacement is ready.
        let replacement = self.capture.capture()?;
        self.current_frame = Some(replacement);
        self.next_capture = Instant::now() + self.frame_interval;
        self.last_error = None;
        Ok(())
    }

    fn capture_if_due(&mut self) -> Result<()> {
        if Instant::now() >= self.next_capture {
            self.capture_now()?;
        }
        Ok(())
    }
}

fn plugin_descriptor(plugin: vst3_host::PluginInfo) -> PluginDescriptor {
    PluginDescriptor {
        bundle_path: plugin.path.to_string_lossy().into_owned(),
        name: plugin.name,
        vendor: plugin.vendor,
        version: plugin.version,
        category: plugin.category,
        class_id: plugin.uid,
        audio_inputs: plugin.audio_inputs,
        audio_outputs: plugin.audio_outputs,
        has_midi_input: plugin.has_midi_input,
        has_midi_output: plugin.has_midi_output,
        has_gui: plugin.has_gui,
    }
}

fn bind_midi_inputs(audio: &AudioHandle, queries: &[String]) -> Result<Vec<MidiInputConnection>> {
    if queries.is_empty() {
        return Ok(Vec::new());
    }
    let ports = midi_input::list_midi_input_ports()?;
    let mut connections = Vec::with_capacity(queries.len());
    for query in queries {
        let needle = query.to_ascii_lowercase();
        let matches = ports
            .iter()
            .filter(|port| port.name().to_ascii_lowercase().contains(&needle))
            .collect::<Vec<_>>();
        let port = match matches.as_slice() {
            [port] => *port,
            [] => return Err(anyhow!("MIDI input {query:?} was not found")),
            _ => return Err(anyhow!("MIDI input {query:?} matched more than one port")),
        };
        eprintln!("binding MIDI input {}", port.name());
        connections.push(midi_input::bind_to_handle(port, audio)?);
    }
    Ok(connections)
}

struct ControlJob {
    request: ControlRequest,
    reply: SyncSender<ControlResponse>,
}

fn main() -> Result<()> {
    let listen = env::args().nth(1).unwrap_or_else(|| DEFAULT_LISTEN.into());
    let isochrone_destination = env::args()
        .nth(2)
        .map(|value| value.parse::<SocketAddr>())
        .transpose()
        .context("invalid Isochrone destination")?;
    let listener = TcpListener::bind(&listen)
        .with_context(|| format!("failed to bind control socket {listen}"))?;
    eprintln!("pedalkernel-studio-host listening on {listen}");
    let midi_queries = env::var("PEDALKERNEL_MIDI_INPUTS")
        .ok()
        .into_iter()
        .flat_map(|value| {
            value
                .split(',')
                .map(str::trim)
                .filter(|value| !value.is_empty())
                .map(str::to_owned)
                .collect::<Vec<_>>()
        })
        .collect();
    let (jobs_tx, jobs_rx) = mpsc::sync_channel(64);
    thread::Builder::new()
        .name("pedalkernel-control".into())
        .spawn(move || serve(listener, jobs_tx))?;
    run_main_loop(Runtime::new(isochrone_destination, midi_queries), jobs_rx)
}

fn serve(listener: TcpListener, jobs: SyncSender<ControlJob>) {
    for connection in listener.incoming() {
        match connection {
            Ok(stream) => {
                if let Err(error) = serve_connection(stream, &jobs) {
                    eprintln!("control connection failed: {error:#}");
                }
            }
            Err(error) => eprintln!("failed to accept control connection: {error}"),
        }
    }
}

fn serve_connection(mut stream: TcpStream, jobs: &SyncSender<ControlJob>) -> Result<()> {
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
                let (reply_tx, reply_rx) = mpsc::sync_channel(1);
                jobs.send(ControlJob {
                    request,
                    reply: reply_tx,
                })?;
                reply_rx.recv()?
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

fn run_main_loop(mut runtime: Runtime, jobs: Receiver<ControlJob>) -> Result<()> {
    initialize_platform_ui()?;
    loop {
        match jobs.recv_timeout(Duration::from_millis(10)) {
            Ok(job) => {
                let result = runtime
                    .execute(job.request.command)
                    .unwrap_or_else(|error| HostResult::Error {
                        message: format!("{error:#}"),
                    });
                let _ = job.reply.send(ControlResponse {
                    request_id: job.request.request_id,
                    result,
                });
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                return Err(anyhow!("control listener stopped"));
            }
        }
        runtime.service_editor()?;
        runtime.service_editor_surface();
        pump_platform_ui();
    }
}

#[cfg(target_os = "macos")]
fn initialize_platform_ui() -> Result<()> {
    use objc2::MainThreadMarker;
    use objc2_app_kit::{NSApplication, NSApplicationActivationPolicy};
    let marker = MainThreadMarker::new()
        .ok_or_else(|| anyhow!("PedalKernel must start on the macOS main thread"))?;
    let application = NSApplication::sharedApplication(marker);
    application.setActivationPolicy(NSApplicationActivationPolicy::Regular);
    application.finishLaunching();
    Ok(())
}

#[cfg(target_os = "macos")]
fn activate_platform_ui() {
    use objc2::MainThreadMarker;
    use objc2_app_kit::{NSApplication, NSWindowSharingType};
    let Some(marker) = MainThreadMarker::new() else {
        return;
    };
    let application = NSApplication::sharedApplication(marker);
    application.unhide(None);
    #[allow(deprecated)]
    application.activateIgnoringOtherApps(true);
    application.arrangeInFront(None);
    for window in application.windows().iter() {
        window.setSharingType(NSWindowSharingType::ReadOnly);
        window.orderFrontRegardless();
    }
}

#[cfg(not(target_os = "macos"))]
fn activate_platform_ui() {}

#[cfg(not(target_os = "macos"))]
fn initialize_platform_ui() -> Result<()> {
    Ok(())
}

#[cfg(target_os = "macos")]
fn pump_platform_ui() {
    use objc2::MainThreadMarker;
    use objc2_app_kit::{NSApplication, NSEventMask};
    use objc2_foundation::{NSDate, NSDefaultRunLoopMode};
    let Some(marker) = MainThreadMarker::new() else {
        return;
    };
    let application = NSApplication::sharedApplication(marker);
    let until = NSDate::dateWithTimeIntervalSinceNow(0.0);
    while let Some(event) = unsafe {
        application.nextEventMatchingMask_untilDate_inMode_dequeue(
            NSEventMask::Any,
            Some(&until),
            NSDefaultRunLoopMode,
            true,
        )
    } {
        application.sendEvent(&event);
    }
}

#[cfg(not(target_os = "macos"))]
fn pump_platform_ui() {}
