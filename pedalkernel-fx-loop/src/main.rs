use std::{
    collections::VecDeque,
    f32::consts::PI,
    fs::{self, File},
    io::Write,
    path::{Path, PathBuf},
    sync::{
        atomic::{AtomicBool, AtomicUsize, Ordering},
        Arc, Mutex,
    },
    thread,
    time::{Duration, Instant},
};

use anyhow::{anyhow, bail, Context, Result};
use clap::{Parser, Subcommand, ValueEnum};
use cpal::{
    traits::{DeviceTrait, HostTrait, StreamTrait},
    Device, SampleFormat, Stream, StreamConfig,
};
use hound::{SampleFormat as WavSampleFormat, WavReader, WavSpec, WavWriter};

#[derive(Parser)]
#[command(author, version, about)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// List audio hosts and devices.
    List,
    /// Send a deterministic test signal to one output and record one input.
    Capture(CaptureArgs),
    /// Send a looping signal and print return levels until the duration expires.
    Monitor(MonitorArgs),
}

#[derive(Parser, Clone)]
struct DeviceArgs {
    /// Audio host to use. Omit to use the platform default host.
    #[arg(long)]
    host: Option<String>,
    /// Device-name substring used for both input and output devices.
    #[arg(long, default_value = "Scarlett")]
    device: String,
    /// Input device-name substring. Overrides --device for input.
    #[arg(long)]
    input_device: Option<String>,
    /// Output device-name substring. Overrides --device for output.
    #[arg(long)]
    output_device: Option<String>,
    /// Sample rate for both streams.
    #[arg(long, default_value_t = 48_000)]
    sample_rate: u32,
    /// 1-based physical output channel to feed.
    #[arg(long, default_value_t = 3)]
    send_channel: usize,
    /// 1-based physical input channel to record.
    #[arg(long, default_value_t = 3)]
    return_channel: usize,
}

#[derive(Parser)]
struct CaptureArgs {
    #[command(flatten)]
    device: DeviceArgs,
    #[command(flatten)]
    signal: SignalArgs,
    /// Directory for WAV, NPY, and metadata artifacts.
    #[arg(long, default_value = "hardware-goldens/ts9_noon")]
    out_dir: PathBuf,
    /// Label prefix for artifact names.
    #[arg(long, default_value = "sine_1k")]
    label: String,
    /// Leading silence before the stimulus, in seconds.
    #[arg(long, default_value_t = 0.25)]
    pre_roll: f32,
    /// Trailing capture time after the stimulus, in seconds.
    #[arg(long, default_value_t = 0.25)]
    tail: f32,
    /// Also mirror the return to these 1-based output channels, comma-separated.
    #[arg(long)]
    monitor_channels: Option<String>,
    /// Gain applied to mirrored monitor output.
    #[arg(long, default_value_t = 0.5)]
    monitor_gain: f32,
}

#[derive(Parser)]
struct MonitorArgs {
    #[command(flatten)]
    device: DeviceArgs,
    #[command(flatten)]
    signal: SignalArgs,
    /// Optional monitor run duration. Omit to run until Ctrl-C.
    #[arg(long)]
    run_duration: Option<f32>,
    /// Mirror returned hardware audio to these 1-based output channels.
    #[arg(long)]
    monitor_channels: Option<String>,
    /// Gain applied to mirrored monitor output.
    #[arg(long, default_value_t = 0.5)]
    monitor_gain: f32,
}

#[derive(Parser, Clone)]
struct SignalArgs {
    /// Test signal to send.
    #[arg(long, value_enum, default_value_t = SignalKind::Sine)]
    signal: SignalKind,
    /// Signal duration in seconds.
    #[arg(long, default_value_t = 4.0)]
    duration: f32,
    /// Peak send amplitude.
    #[arg(long, default_value_t = 0.15)]
    amp: f32,
    /// Sine frequency.
    #[arg(long, default_value_t = 1000.0)]
    freq: f32,
    /// Sweep start frequency.
    #[arg(long, default_value_t = 20.0)]
    start_freq: f32,
    /// Sweep end frequency.
    #[arg(long, default_value_t = 20_000.0)]
    end_freq: f32,
    /// WAV file to send when --signal wav is selected.
    #[arg(long)]
    input_wav: Option<PathBuf>,
}

#[derive(Copy, Clone, Debug, ValueEnum)]
enum SignalKind {
    Sine,
    Sweep,
    Impulse,
    Wav,
}

#[derive(Clone)]
struct AudioDevices {
    input: Device,
    output: Device,
    input_name: String,
    output_name: String,
    input_channels: usize,
    output_channels: usize,
    input_format: SampleFormat,
    output_format: SampleFormat,
}

#[derive(Default)]
struct LevelStats {
    frames: usize,
    sum_sq: f64,
    peak: f32,
}

impl LevelStats {
    fn push(&mut self, sample: f32) {
        self.frames += 1;
        self.sum_sq += (sample as f64) * (sample as f64);
        self.peak = self.peak.max(sample.abs());
    }

    fn reset(&mut self) -> (f32, f32) {
        let rms = if self.frames == 0 {
            0.0
        } else {
            (self.sum_sq / self.frames as f64).sqrt() as f32
        };
        let peak = self.peak;
        *self = Self::default();
        (rms, peak)
    }
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Command::List => list_devices(),
        Command::Capture(args) => capture(args),
        Command::Monitor(args) => monitor(args),
    }
}

fn list_devices() -> Result<()> {
    for host_id in cpal::available_hosts() {
        let host = cpal::host_from_id(host_id)?;
        println!("host: {}", host_id.name());
        for device in host.devices()? {
            let name = device.name().unwrap_or_else(|_| "<unknown>".to_string());
            let input = device
                .default_input_config()
                .map(|c| {
                    format!(
                        "in {}ch {:?} {}Hz",
                        c.channels(),
                        c.sample_format(),
                        c.sample_rate().0
                    )
                })
                .unwrap_or_else(|_| "no input".to_string());
            let output = device
                .default_output_config()
                .map(|c| {
                    format!(
                        "out {}ch {:?} {}Hz",
                        c.channels(),
                        c.sample_format(),
                        c.sample_rate().0
                    )
                })
                .unwrap_or_else(|_| "no output".to_string());
            println!("  {name} ({input}; {output})");
        }
    }
    Ok(())
}

fn capture(args: CaptureArgs) -> Result<()> {
    validate_signal_args(&args.signal)?;
    let audio = open_audio(&args.device)?;
    let monitor_channels =
        parse_optional_channels(args.monitor_channels.as_deref())?.unwrap_or_default();
    validate_channels(
        &audio,
        args.device.send_channel,
        args.device.return_channel,
        &monitor_channels,
    )?;

    fs::create_dir_all(&args.out_dir)?;
    let body = build_signal(&args.signal, args.device.sample_rate)?;
    let pre = seconds_to_frames(args.pre_roll, args.device.sample_rate)?;
    let tail = seconds_to_frames(args.tail, args.device.sample_rate)?;
    let mut stimulus = vec![0.0; pre + body.len() + tail];
    stimulus[pre..pre + body.len()].copy_from_slice(&body);

    let recorded = run_duplex(
        &audio,
        args.device.sample_rate,
        args.device.send_channel - 1,
        args.device.return_channel - 1,
        stimulus.clone(),
        Some(stimulus.len()),
        monitor_channels.iter().copied().map(|ch| ch - 1).collect(),
        args.monitor_gain,
        false,
    )?;

    let prefix = args.out_dir.join(&args.label);
    write_wav_mono(
        &stimulus,
        &prefix.with_extension("stimulus.wav"),
        args.device.sample_rate,
    )?;
    write_wav_mono(
        &recorded,
        &prefix.with_extension("return.wav"),
        args.device.sample_rate,
    )?;
    write_wav_stereo(
        &stimulus,
        &recorded,
        &prefix.with_extension("send_return.wav"),
        args.device.sample_rate,
    )?;
    write_npy_f64(
        &prefix.with_extension("stimulus.npy"),
        &stimulus.iter().map(|&s| s as f64).collect::<Vec<_>>(),
    )?;
    write_npy_f64(
        &prefix.with_extension("return.npy"),
        &recorded.iter().map(|&s| s as f64).collect::<Vec<_>>(),
    )?;
    write_metadata(
        &prefix.with_extension("json"),
        &args,
        &audio,
        recorded.len(),
    )?;

    let (rms, peak) = buffer_rms_peak(&recorded);
    println!(
        "captured {} frames from {} input {}: rms {:.1} dBFS, peak {:.1} dBFS",
        recorded.len(),
        audio.input_name,
        args.device.return_channel,
        dbfs(rms),
        dbfs(peak),
    );
    println!("wrote {}", args.out_dir.display());
    Ok(())
}

fn monitor(args: MonitorArgs) -> Result<()> {
    validate_signal_args(&args.signal)?;
    let audio = open_audio(&args.device)?;
    let monitor_channels =
        parse_optional_channels(args.monitor_channels.as_deref())?.unwrap_or_else(|| vec![1, 2]);
    validate_channels(
        &audio,
        args.device.send_channel,
        args.device.return_channel,
        &monitor_channels,
    )?;

    let mut stimulus = build_signal(&args.signal, args.device.sample_rate)?;
    if stimulus.is_empty() {
        stimulus.push(0.0);
    }

    println!(
        "sending {} output {} -> returning {} input {}; monitoring return on outputs {:?}",
        audio.output_name,
        args.device.send_channel,
        audio.input_name,
        args.device.return_channel,
        monitor_channels
    );
    let start = Instant::now();
    let duration_frames = args
        .run_duration
        .map(|duration| seconds_to_frames(duration, args.device.sample_rate))
        .transpose()?;

    let _recorded = run_duplex(
        &audio,
        args.device.sample_rate,
        args.device.send_channel - 1,
        args.device.return_channel - 1,
        stimulus,
        duration_frames,
        monitor_channels.into_iter().map(|ch| ch - 1).collect(),
        args.monitor_gain,
        true,
    )?;
    println!(
        "monitor stopped after {:.1}s",
        start.elapsed().as_secs_f32()
    );
    Ok(())
}

fn open_audio(args: &DeviceArgs) -> Result<AudioDevices> {
    let host = match &args.host {
        Some(name) => {
            let host_id = cpal::available_hosts()
                .into_iter()
                .find(|id| id.name().eq_ignore_ascii_case(name))
                .ok_or_else(|| anyhow!("audio host not found: {name}"))?;
            cpal::host_from_id(host_id)?
        }
        None => cpal::default_host(),
    };

    let input_query = args.input_device.as_deref().unwrap_or(&args.device);
    let output_query = args.output_device.as_deref().unwrap_or(&args.device);
    let input = find_device(&host, input_query, Direction::Input)?;
    let output = find_device(&host, output_query, Direction::Output)?;
    let input_name = input
        .name()
        .unwrap_or_else(|_| "<unknown input>".to_string());
    let output_name = output
        .name()
        .unwrap_or_else(|_| "<unknown output>".to_string());
    let input_config = input.default_input_config()?;
    let output_config = output.default_output_config()?;

    Ok(AudioDevices {
        input,
        output,
        input_name,
        output_name,
        input_channels: input_config.channels() as usize,
        output_channels: output_config.channels() as usize,
        input_format: input_config.sample_format(),
        output_format: output_config.sample_format(),
    })
}

#[derive(Copy, Clone)]
enum Direction {
    Input,
    Output,
}

fn find_device(host: &cpal::Host, query: &str, direction: Direction) -> Result<Device> {
    let query = query.to_ascii_lowercase();
    let mut matches = Vec::new();
    for device in host.devices()? {
        let name = device.name().unwrap_or_else(|_| "<unknown>".to_string());
        let has_direction = match direction {
            Direction::Input => device.default_input_config().is_ok(),
            Direction::Output => device.default_output_config().is_ok(),
        };
        if has_direction && name.to_ascii_lowercase().contains(&query) {
            matches.push(device);
        }
    }
    matches.into_iter().next().ok_or_else(|| {
        anyhow!(
            "no {:?} device matching {query:?}",
            direction_name(direction)
        )
    })
}

fn direction_name(direction: Direction) -> &'static str {
    match direction {
        Direction::Input => "input",
        Direction::Output => "output",
    }
}

fn validate_channels(
    audio: &AudioDevices,
    send_channel: usize,
    return_channel: usize,
    monitor_channels: &[usize],
) -> Result<()> {
    if send_channel == 0 || send_channel > audio.output_channels {
        bail!(
            "send channel {} is outside output channel count {}",
            send_channel,
            audio.output_channels
        );
    }
    if return_channel == 0 || return_channel > audio.input_channels {
        bail!(
            "return channel {} is outside input channel count {}",
            return_channel,
            audio.input_channels
        );
    }
    for &channel in monitor_channels {
        if channel == 0 || channel > audio.output_channels {
            bail!(
                "monitor channel {} is outside output channel count {}",
                channel,
                audio.output_channels
            );
        }
    }
    Ok(())
}

fn run_duplex(
    audio: &AudioDevices,
    sample_rate: u32,
    send_channel: usize,
    return_channel: usize,
    stimulus: Vec<f32>,
    stop_after_frames: Option<usize>,
    monitor_channels: Vec<usize>,
    monitor_gain: f32,
    loop_stimulus: bool,
) -> Result<Vec<f32>> {
    let input_config = StreamConfig {
        channels: audio.input_channels as u16,
        sample_rate: cpal::SampleRate(sample_rate),
        buffer_size: cpal::BufferSize::Default,
    };
    let output_config = StreamConfig {
        channels: audio.output_channels as u16,
        sample_rate: cpal::SampleRate(sample_rate),
        buffer_size: cpal::BufferSize::Default,
    };
    let captured = Arc::new(Mutex::new(Vec::<f32>::new()));
    let monitor_fifo = Arc::new(Mutex::new(VecDeque::<f32>::with_capacity(
        sample_rate as usize,
    )));
    let done = Arc::new(AtomicBool::new(false));
    let input_frames = Arc::new(AtomicUsize::new(0));
    let output_frames = Arc::new(AtomicUsize::new(0));
    let level = Arc::new(Mutex::new(LevelStats::default()));

    let input_stream = build_input_stream(
        audio,
        &input_config,
        return_channel,
        stop_after_frames,
        captured.clone(),
        monitor_fifo.clone(),
        done.clone(),
        input_frames.clone(),
        level.clone(),
    )?;
    let output_stream = build_output_stream(
        audio,
        &output_config,
        send_channel,
        stimulus,
        monitor_channels,
        monitor_gain,
        loop_stimulus,
        monitor_fifo,
        done.clone(),
        output_frames,
    )?;

    input_stream.play()?;
    output_stream.play()?;

    let mut last_print = Instant::now();
    loop {
        if done.load(Ordering::SeqCst) {
            break;
        }
        if stop_after_frames.is_none() && last_print.elapsed() >= Duration::from_millis(500) {
            let (rms, peak) = level.lock().unwrap().reset();
            println!(
                "return rms {:>6.1} dBFS peak {:>6.1} dBFS",
                dbfs(rms),
                dbfs(peak)
            );
            last_print = Instant::now();
        }
        thread::sleep(Duration::from_millis(10));
    }

    drop(output_stream);
    drop(input_stream);
    let mut recorded = captured
        .lock()
        .map_err(|_| anyhow!("capture buffer poisoned"))?
        .clone();
    if let Some(target) = stop_after_frames {
        recorded.truncate(target);
    }
    Ok(recorded)
}

fn build_input_stream(
    audio: &AudioDevices,
    config: &StreamConfig,
    return_channel: usize,
    stop_after_frames: Option<usize>,
    captured: Arc<Mutex<Vec<f32>>>,
    monitor_fifo: Arc<Mutex<VecDeque<f32>>>,
    done: Arc<AtomicBool>,
    input_frames: Arc<AtomicUsize>,
    level: Arc<Mutex<LevelStats>>,
) -> Result<Stream> {
    let channels = config.channels as usize;
    match audio.input_format {
        SampleFormat::F32 => audio.input.build_input_stream(
            config,
            move |data: &[f32], _| {
                capture_input_frames(
                    data,
                    channels,
                    return_channel,
                    stop_after_frames,
                    &captured,
                    &monitor_fifo,
                    &done,
                    &input_frames,
                    &level,
                )
            },
            stream_error,
            None,
        ),
        other => bail!("unsupported input sample format {other:?}; expected F32"),
    }
    .context("failed to build input stream")
}

fn build_output_stream(
    audio: &AudioDevices,
    config: &StreamConfig,
    send_channel: usize,
    stimulus: Vec<f32>,
    monitor_channels: Vec<usize>,
    monitor_gain: f32,
    loop_stimulus: bool,
    monitor_fifo: Arc<Mutex<VecDeque<f32>>>,
    done: Arc<AtomicBool>,
    output_frames: Arc<AtomicUsize>,
) -> Result<Stream> {
    let channels = config.channels as usize;
    match audio.output_format {
        SampleFormat::F32 => audio.output.build_output_stream(
            config,
            move |data: &mut [f32], _| {
                fill_output_frames(
                    data,
                    channels,
                    send_channel,
                    &stimulus,
                    &monitor_channels,
                    monitor_gain,
                    loop_stimulus,
                    &monitor_fifo,
                    &done,
                    &output_frames,
                )
            },
            stream_error,
            None,
        ),
        other => bail!("unsupported output sample format {other:?}; expected F32"),
    }
    .context("failed to build output stream")
}

fn capture_input_frames(
    data: &[f32],
    channels: usize,
    return_channel: usize,
    stop_after_frames: Option<usize>,
    captured: &Arc<Mutex<Vec<f32>>>,
    monitor_fifo: &Arc<Mutex<VecDeque<f32>>>,
    done: &Arc<AtomicBool>,
    input_frames: &Arc<AtomicUsize>,
    level: &Arc<Mutex<LevelStats>>,
) {
    let mut captured = captured.lock().unwrap();
    let mut monitor_fifo = monitor_fifo.lock().unwrap();
    let mut level = level.lock().unwrap();
    for frame in data.chunks(channels) {
        let sample = frame.get(return_channel).copied().unwrap_or(0.0);
        captured.push(sample);
        level.push(sample);
        monitor_fifo.push_back(sample);
        if monitor_fifo.len() > 96_000 {
            monitor_fifo.pop_front();
        }
    }
    let frames =
        input_frames.fetch_add(data.len() / channels, Ordering::SeqCst) + data.len() / channels;
    if stop_after_frames.is_some_and(|target| frames >= target) {
        done.store(true, Ordering::SeqCst);
    }
}

fn fill_output_frames(
    data: &mut [f32],
    channels: usize,
    send_channel: usize,
    stimulus: &[f32],
    monitor_channels: &[usize],
    monitor_gain: f32,
    loop_stimulus: bool,
    monitor_fifo: &Arc<Mutex<VecDeque<f32>>>,
    done: &Arc<AtomicBool>,
    output_frames: &Arc<AtomicUsize>,
) {
    let mut monitor_fifo = monitor_fifo.lock().unwrap();
    for frame in data.chunks_mut(channels) {
        frame.fill(0.0);
        let idx = output_frames.fetch_add(1, Ordering::SeqCst);
        let send = if loop_stimulus {
            stimulus[idx % stimulus.len()]
        } else {
            stimulus.get(idx).copied().unwrap_or(0.0)
        };
        if let Some(slot) = frame.get_mut(send_channel) {
            *slot = send;
        }
        let monitor = monitor_fifo.pop_front().unwrap_or(0.0) * monitor_gain;
        for &channel in monitor_channels {
            if channel != send_channel {
                if let Some(slot) = frame.get_mut(channel) {
                    *slot = monitor;
                }
            }
        }
    }
    if !loop_stimulus
        && output_frames.load(Ordering::SeqCst) >= stimulus.len()
        && done.load(Ordering::SeqCst)
    {
        for sample in data {
            *sample = 0.0;
        }
    }
}

fn stream_error(err: cpal::StreamError) {
    eprintln!("audio stream error: {err}");
}

fn build_signal(args: &SignalArgs, sample_rate: u32) -> Result<Vec<f32>> {
    let frames = seconds_to_frames(args.duration, sample_rate)?;
    let amp = args.amp.clamp(0.0, 1.0);
    match args.signal {
        SignalKind::Sine => Ok((0..frames)
            .map(|i| {
                let t = i as f32 / sample_rate as f32;
                amp * (2.0 * PI * args.freq * t).sin()
            })
            .collect()),
        SignalKind::Sweep => {
            if args.start_freq <= 0.0 || args.end_freq <= 0.0 {
                bail!("sweep frequencies must be positive");
            }
            let ratio = args.end_freq / args.start_freq;
            let ln_ratio = ratio.ln();
            Ok((0..frames)
                .map(|i| {
                    let t = i as f32 / sample_rate as f32;
                    let phase = if ln_ratio.abs() < f32::EPSILON {
                        2.0 * PI * args.start_freq * t
                    } else {
                        2.0 * PI
                            * args.start_freq
                            * args.duration
                            * ((ratio).powf(t / args.duration) - 1.0)
                            / ln_ratio
                    };
                    amp * phase.sin()
                })
                .collect())
        }
        SignalKind::Impulse => {
            let mut signal = vec![0.0; frames.max(1)];
            signal[0] = amp;
            Ok(signal)
        }
        SignalKind::Wav => {
            let path = args
                .input_wav
                .as_deref()
                .ok_or_else(|| anyhow!("--input-wav is required when --signal wav"))?;
            let (samples, wav_rate) = read_wav_mono(path)?;
            if wav_rate != sample_rate {
                bail!(
                    "{} is {}Hz, but --sample-rate is {}Hz; resample first",
                    path.display(),
                    wav_rate,
                    sample_rate
                );
            }
            if samples.len() >= frames {
                Ok(samples[..frames].iter().map(|s| s * amp).collect())
            } else {
                let mut out = Vec::with_capacity(frames);
                while out.len() < frames {
                    out.extend(samples.iter().map(|s| s * amp));
                }
                out.truncate(frames);
                Ok(out)
            }
        }
    }
}

fn validate_signal_args(args: &SignalArgs) -> Result<()> {
    if args.duration <= 0.0 {
        bail!("duration must be positive");
    }
    if args.amp < 0.0 || args.amp > 1.0 {
        bail!("amp must be in 0.0..=1.0");
    }
    Ok(())
}

fn seconds_to_frames(seconds: f32, sample_rate: u32) -> Result<usize> {
    if seconds < 0.0 {
        bail!("seconds must be non-negative");
    }
    Ok((seconds * sample_rate as f32).round() as usize)
}

fn parse_channels(value: &str) -> Result<Vec<usize>, String> {
    value
        .split(',')
        .map(str::trim)
        .filter(|part| !part.is_empty())
        .map(|part| {
            part.parse::<usize>()
                .map_err(|_| format!("invalid channel number: {part}"))
        })
        .collect()
}

fn parse_optional_channels(value: Option<&str>) -> Result<Option<Vec<usize>>> {
    value
        .map(parse_channels)
        .transpose()
        .map_err(|err| anyhow!(err))
}

fn read_wav_mono(path: &Path) -> Result<(Vec<f32>, u32)> {
    let mut reader = WavReader::open(path)?;
    let spec = reader.spec();
    let channels = spec.channels as usize;
    let samples = match spec.sample_format {
        WavSampleFormat::Float => {
            let all = reader.samples::<f32>().collect::<Result<Vec<_>, _>>()?;
            mix_to_mono(&all, channels)
        }
        WavSampleFormat::Int => {
            let max = (1_i64 << (spec.bits_per_sample - 1)) as f32;
            let all = reader
                .samples::<i32>()
                .map(|sample| sample.map(|s| s as f32 / max))
                .collect::<Result<Vec<_>, _>>()?;
            mix_to_mono(&all, channels)
        }
    };
    Ok((samples, spec.sample_rate))
}

fn mix_to_mono(interleaved: &[f32], channels: usize) -> Vec<f32> {
    if channels == 1 {
        return interleaved.to_vec();
    }
    interleaved
        .chunks(channels)
        .map(|frame| frame.iter().sum::<f32>() / channels as f32)
        .collect()
}

fn write_wav_mono(samples: &[f32], path: &Path, sample_rate: u32) -> Result<()> {
    let spec = WavSpec {
        channels: 1,
        sample_rate,
        bits_per_sample: 32,
        sample_format: WavSampleFormat::Float,
    };
    let mut writer = WavWriter::create(path, spec)?;
    for &sample in samples {
        writer.write_sample(sample)?;
    }
    writer.finalize()?;
    Ok(())
}

fn write_wav_stereo(left: &[f32], right: &[f32], path: &Path, sample_rate: u32) -> Result<()> {
    let spec = WavSpec {
        channels: 2,
        sample_rate,
        bits_per_sample: 32,
        sample_format: WavSampleFormat::Float,
    };
    let mut writer = WavWriter::create(path, spec)?;
    for i in 0..left.len().min(right.len()) {
        writer.write_sample(left[i])?;
        writer.write_sample(right[i])?;
    }
    writer.finalize()?;
    Ok(())
}

fn write_npy_f64(path: &Path, data: &[f64]) -> Result<()> {
    let mut header = format!(
        "{{'descr': '<f8', 'fortran_order': False, 'shape': ({},), }}",
        data.len()
    );
    let padding = (16 - ((10 + header.len() + 1) % 16)) % 16;
    header.push_str(&" ".repeat(padding));
    header.push('\n');

    let mut file = File::create(path)?;
    file.write_all(b"\x93NUMPY")?;
    file.write_all(&[1, 0])?;
    file.write_all(&(header.len() as u16).to_le_bytes())?;
    file.write_all(header.as_bytes())?;
    for &sample in data {
        file.write_all(&sample.to_le_bytes())?;
    }
    Ok(())
}

fn write_metadata(
    path: &Path,
    args: &CaptureArgs,
    audio: &AudioDevices,
    frames: usize,
) -> Result<()> {
    let metadata = format!(
        concat!(
            "{{\n",
            "  \"tool\": \"pedalkernel-fx-loop\",\n",
            "  \"device_input\": \"{}\",\n",
            "  \"device_output\": \"{}\",\n",
            "  \"sample_rate\": {},\n",
            "  \"send_channel\": {},\n",
            "  \"return_channel\": {},\n",
            "  \"signal\": \"{:?}\",\n",
            "  \"duration_seconds\": {},\n",
            "  \"amp\": {},\n",
            "  \"frames\": {},\n",
            "  \"note\": \"Scarlett channel numbers are 1-based; TS9 noon capture is output 3 to input 3.\"\n",
            "}}\n"
        ),
        json_escape(&audio.input_name),
        json_escape(&audio.output_name),
        args.device.sample_rate,
        args.device.send_channel,
        args.device.return_channel,
        args.signal.signal,
        args.signal.duration,
        args.signal.amp,
        frames
    );
    fs::write(path, metadata)?;
    Ok(())
}

fn json_escape(value: &str) -> String {
    value.replace('\\', "\\\\").replace('"', "\\\"")
}

fn buffer_rms_peak(buffer: &[f32]) -> (f32, f32) {
    if buffer.is_empty() {
        return (0.0, 0.0);
    }
    let sum_sq = buffer.iter().map(|&s| (s as f64) * (s as f64)).sum::<f64>();
    let rms = (sum_sq / buffer.len() as f64).sqrt() as f32;
    let peak = buffer.iter().fold(0.0_f32, |peak, &s| peak.max(s.abs()));
    (rms, peak)
}

fn dbfs(value: f32) -> f32 {
    20.0 * value.max(1.0e-9).log10()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_channel_list() {
        assert_eq!(parse_channels("1,2, 4").unwrap(), vec![1, 2, 4]);
    }

    #[test]
    fn sine_has_expected_length() {
        let args = SignalArgs {
            signal: SignalKind::Sine,
            duration: 1.0,
            amp: 0.25,
            freq: 1000.0,
            start_freq: 20.0,
            end_freq: 20_000.0,
            input_wav: None,
        };
        assert_eq!(build_signal(&args, 48_000).unwrap().len(), 48_000);
    }

    #[test]
    fn npy_header_is_aligned() {
        let tmp = std::env::temp_dir().join("pedalkernel_fx_loop_test.npy");
        write_npy_f64(&tmp, &[0.0, 1.0, -1.0]).unwrap();
        let bytes = fs::read(&tmp).unwrap();
        assert_eq!(&bytes[..6], b"\x93NUMPY");
        let header_len = u16::from_le_bytes([bytes[8], bytes[9]]) as usize;
        assert_eq!((10 + header_len) % 16, 0);
        let _ = fs::remove_file(tmp);
    }
}
