//! Pedal Folder TUI — discover all `.pedal` files in a directory (one level deep),
//! pre-compile them, and allow seamless switching between pedals during live JACK audio.

use crossterm::{
    event::{self, Event, KeyCode, KeyModifiers},
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
    ExecutableCommand,
};
use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::pedalboard::PedalSwitcherProcessor;
use pedalkernel::{AudioEngine, SharedControls, WavLoopProcessor};
use ratatui::{
    layout::{Alignment, Constraint, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, BorderType, Borders, Paragraph},
    Frame,
};
use std::io::stdout;
use std::path::Path;
use std::sync::Arc;

use super::tui_widgets::{
    help_spans, render_footswitch, render_io_bar, render_knob_row, run_file_picker,
    run_output_select, run_port_select, FilePickerResult, KnobState, OutputSelectResult,
    PortSelectResult, Term,
};

// ═════════════════════════════════════════════════════════════════════════════
// State
// ═════════════════════════════════════════════════════════════════════════════

struct PedalEntry {
    name: String,
    knobs: Vec<KnobState>,
    selected_knob: usize,
    bypassed: bool,
}

struct PedalFolderState {
    pedals: Vec<PedalEntry>,
    active_pedal: usize,
    controls: Arc<SharedControls>,
    input_port: String,
    output_port: String,
    voltage: f64,
}

impl PedalFolderState {
    fn switch_pedal(&mut self, new_idx: usize) {
        if new_idx < self.pedals.len() && new_idx != self.active_pedal {
            self.active_pedal = new_idx;
            self.controls
                .set_control("__switch_pedal__", new_idx as f64);
            // Re-apply all knob values for the new pedal
            let pedal = &self.pedals[self.active_pedal];
            for knob in &pedal.knobs {
                self.controls.set_control(&knob.label, knob.value);
            }
            self.controls
                .set_bypassed(pedal.bypassed);
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// Input handling
// ═════════════════════════════════════════════════════════════════════════════

/// Handle input for the pedal folder TUI. Returns false when the user wants to quit.
fn handle_folder_input(state: &mut PedalFolderState, code: KeyCode, modifiers: KeyModifiers) -> bool {
    // Pedal switching: { and }
    match code {
        KeyCode::Char('{') => {
            if state.pedals.len() > 1 {
                let new_idx = (state.active_pedal + state.pedals.len() - 1) % state.pedals.len();
                state.switch_pedal(new_idx);
            }
            return true;
        }
        KeyCode::Char('}') => {
            if state.pedals.len() > 1 {
                let new_idx = (state.active_pedal + 1) % state.pedals.len();
                state.switch_pedal(new_idx);
            }
            return true;
        }
        _ => {}
    }

    // Delegate to standard pedal key handling
    let pedal = &mut state.pedals[state.active_pedal];
    match code {
        KeyCode::Char('q') | KeyCode::Esc => return false,
        KeyCode::Tab if modifiers.contains(KeyModifiers::SHIFT) => {
            if !pedal.knobs.is_empty() {
                pedal.selected_knob =
                    (pedal.selected_knob + pedal.knobs.len() - 1) % pedal.knobs.len();
            }
        }
        KeyCode::BackTab => {
            if !pedal.knobs.is_empty() {
                pedal.selected_knob =
                    (pedal.selected_knob + pedal.knobs.len() - 1) % pedal.knobs.len();
            }
        }
        KeyCode::Tab => {
            if !pedal.knobs.is_empty() {
                pedal.selected_knob = (pedal.selected_knob + 1) % pedal.knobs.len();
            }
        }
        KeyCode::Left => adjust_knob(pedal, -0.05, &state.controls),
        KeyCode::Right => adjust_knob(pedal, 0.05, &state.controls),
        KeyCode::Down => adjust_knob(pedal, -0.01, &state.controls),
        KeyCode::Up => adjust_knob(pedal, 0.01, &state.controls),
        KeyCode::Char(' ') => {
            pedal.bypassed = !pedal.bypassed;
            state.controls.set_bypassed(pedal.bypassed);
        }
        KeyCode::Char('r') | KeyCode::Char('R') => {
            if let Some(k) = pedal.knobs.get_mut(pedal.selected_knob) {
                k.value = k.default;
                state.controls.set_control(&k.label, k.value);
            }
        }
        KeyCode::Char(']') | KeyCode::Char('+') | KeyCode::Char('=') => {
            state.voltage = (state.voltage + 1.0).min(18.0);
            state.controls.set_supply_voltage(state.voltage);
        }
        KeyCode::Char('[') | KeyCode::Char('-') | KeyCode::Char('_') => {
            state.voltage = (state.voltage - 1.0).max(5.0);
            state.controls.set_supply_voltage(state.voltage);
        }
        _ => {}
    }
    true
}

fn adjust_knob(pedal: &mut PedalEntry, delta: f64, controls: &SharedControls) {
    if let Some(k) = pedal.knobs.get_mut(pedal.selected_knob) {
        let span = k.range.1 - k.range.0;
        k.value = (k.value + delta * span).clamp(k.range.0, k.range.1);
        controls.set_control(&k.label, k.value);
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// Drawing
// ═════════════════════════════════════════════════════════════════════════════

fn draw_pedal_folder(frame: &mut Frame, state: &PedalFolderState) {
    let area = frame.area();

    if area.width < 30 || area.height < 16 {
        let msg = Paragraph::new("Terminal too small!\nResize to at least 30x16.")
            .alignment(Alignment::Center);
        frame.render_widget(msg, area);
        return;
    }

    // Top-level layout: pedal selector bar + pedal content
    let v_chunks = Layout::vertical([
        Constraint::Length(1), // pedal selector bar
        Constraint::Min(14),   // pedal content
    ])
    .split(area);

    // Pedal selector bar
    render_pedal_selector(frame, v_chunks[0], &state.pedals, state.active_pedal);

    // Pedal content
    let pedal = &state.pedals[state.active_pedal];

    let title_text = if pedal.bypassed {
        format!("  {} [BYPASSED]  ", pedal.name.to_uppercase())
    } else {
        format!("  {}  ", pedal.name.to_uppercase())
    };
    let border_color = if pedal.bypassed {
        Color::DarkGray
    } else {
        Color::White
    };
    let title_color = if pedal.bypassed {
        Color::DarkGray
    } else {
        Color::Yellow
    };
    let outer = Block::default()
        .title(Span::styled(
            title_text,
            Style::default()
                .fg(title_color)
                .add_modifier(Modifier::BOLD),
        ))
        .title_alignment(Alignment::Center)
        .borders(Borders::ALL)
        .border_type(BorderType::Double)
        .style(Style::default().fg(border_color));
    let inner = outer.inner(v_chunks[1]);
    frame.render_widget(outer, v_chunks[1]);

    draw_pedal_content(frame, state, pedal, inner);
}

fn draw_pedal_content(
    frame: &mut Frame,
    state: &PedalFolderState,
    pedal: &PedalEntry,
    inner: Rect,
) {
    let v_chunks = Layout::vertical([
        Constraint::Length(1), // I/O info
        Constraint::Length(1), // padding
        Constraint::Length(5), // knob art
        Constraint::Length(1), // labels
        Constraint::Length(1), // values
        Constraint::Length(1), // gap
        Constraint::Length(3), // footswitch
        Constraint::Length(1), // help
    ])
    .split(inner);

    // I/O info bar
    render_io_bar(frame, v_chunks[0], &state.input_port, &state.output_port, state.voltage);

    // Knob row
    if pedal.knobs.is_empty() {
        let msg =
            Paragraph::new("No controls defined for this pedal.").alignment(Alignment::Center);
        frame.render_widget(msg, v_chunks[2]);
    } else {
        render_knob_row(
            frame,
            &pedal.knobs,
            pedal.selected_knob,
            v_chunks[2],
            v_chunks[3],
            v_chunks[4],
        );
    }

    // Footswitch
    render_footswitch(frame, v_chunks[6], pedal.bypassed);

    // Help bar
    let help = Paragraph::new(help_spans(&[
        ("{ }", "pedal"),
        ("←→", "adjust"),
        ("↑↓", "fine"),
        ("Tab", "next"),
        ("R", "reset"),
        ("[/]", "voltage"),
        ("Space", "bypass"),
        ("P", "picker"),
        ("Q", "quit"),
    ]))
    .alignment(Alignment::Center);
    frame.render_widget(help, v_chunks[7]);
}

/// Render the pedal selector bar: `  TUBE SCREAMER  → [ BLUES DRIVER ] → ...`
fn render_pedal_selector(frame: &mut Frame, area: Rect, pedals: &[PedalEntry], active: usize) {
    let mut spans: Vec<Span> = Vec::new();
    for (i, pedal) in pedals.iter().enumerate() {
        if i > 0 {
            spans.push(Span::styled(" → ", Style::default().fg(Color::DarkGray)));
        }
        let name = pedal.name.to_uppercase();
        if i == active {
            spans.push(Span::styled(
                format!("[ {name} ]"),
                Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD),
            ));
        } else {
            spans.push(Span::styled(
                format!("  {name}  "),
                Style::default()
                    .fg(Color::DarkGray)
                    .add_modifier(Modifier::DIM),
            ));
        }
    }
    let line = Paragraph::new(Line::from(spans)).alignment(Alignment::Center);
    frame.render_widget(line, area);
}

// ═════════════════════════════════════════════════════════════════════════════
// Pedal discovery
// ═════════════════════════════════════════════════════════════════════════════

struct DiscoveredPedal {
    name: String,
    knobs: Vec<KnobState>,
    processor: pedalkernel::compiler::CompiledPedal,
}

fn discover_pedals(
    dir_path: &str,
    sample_rate: f64,
) -> Result<Vec<DiscoveredPedal>, Box<dyn std::error::Error>> {
    let dir = Path::new(dir_path);
    if !dir.is_dir() {
        return Err(format!("'{}' is not a directory", dir_path).into());
    }

    // Collect .pedal files from the directory and one level of subdirectories.
    let mut pedal_files = Vec::new();
    collect_pedal_files(dir, &mut pedal_files);

    // Also scan one level of subdirectories
    if let Ok(entries) = std::fs::read_dir(dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                collect_pedal_files(&path, &mut pedal_files);
            }
        }
    }

    pedal_files.sort();

    if pedal_files.is_empty() {
        return Err(format!("No .pedal files found in '{}'", dir_path).into());
    }

    let mut discovered = Vec::with_capacity(pedal_files.len());

    for pedal_path in &pedal_files {
        let source = match std::fs::read_to_string(pedal_path) {
            Ok(s) => s,
            Err(e) => {
                eprintln!(
                    "WARNING: Skipping {} — cannot read: {e}",
                    pedal_path.display()
                );
                continue;
            }
        };

        let pedal_def = match parse_pedal_file(&source) {
            Ok(p) => p,
            Err(e) => {
                eprintln!(
                    "WARNING: Skipping {} — parse error: {e}",
                    pedal_path.display()
                );
                continue;
            }
        };

        let mut proc = match compile_pedal(&pedal_def, sample_rate) {
            Ok(p) => p,
            Err(e) => {
                eprintln!(
                    "WARNING: Skipping {} — compile error: {e}",
                    pedal_path.display()
                );
                continue;
            }
        };

        // Apply default control values
        for c in &pedal_def.controls {
            proc.set_control(&c.label, c.default);
        }

        let knobs = pedal_def
            .controls
            .iter()
            .map(|c| KnobState {
                label: c.label.clone(),
                value: c.default,
                range: c.range,
                default: c.default,
            })
            .collect();

        discovered.push(DiscoveredPedal {
            name: pedal_def.name.clone(),
            knobs,
            processor: proc,
        });
    }

    Ok(discovered)
}

/// Collect `.pedal` files from a single directory (non-recursive).
fn collect_pedal_files(dir: &Path, out: &mut Vec<std::path::PathBuf>) {
    if let Ok(entries) = std::fs::read_dir(dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.extension().and_then(|e| e.to_str()) == Some("pedal") {
                out.push(path);
            }
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// Control loops
// ═════════════════════════════════════════════════════════════════════════════

fn run_pedal_folder_control(
    terminal: &mut Term,
    client: jack::Client,
    switcher: PedalSwitcherProcessor,
    pedals: Vec<PedalEntry>,
    connect_from: &str,
    connect_to: &str,
    base_dir: &Path,
) -> Result<Option<std::path::PathBuf>, Box<dyn std::error::Error>> {
    let controls = Arc::new(SharedControls::new());
    let (async_client, our_in, our_out) = AudioEngine::start(client, switcher, controls.clone())?;

    async_client
        .as_client()
        .connect_ports_by_name(connect_from, &our_in)?;
    async_client
        .as_client()
        .connect_ports_by_name(&our_out, connect_to)?;
    if let Some(pair) = super::stereo_pair(connect_to) {
        let _ = async_client.as_client().connect_ports_by_name(&our_out, &pair);
    }

    let mut state = PedalFolderState {
        pedals,
        active_pedal: 0,
        controls,
        input_port: connect_from.to_string(),
        output_port: super::stereo_display(connect_to),
        voltage: 9.0,
    };

    let result = loop {
        terminal.draw(|f| draw_pedal_folder(f, &state))?;

        if event::poll(std::time::Duration::from_millis(50))? {
            if let Event::Key(key) = event::read()? {
                if key.code == KeyCode::Char('p') || key.code == KeyCode::Char('P') {
                    drop(async_client);
                    break match run_file_picker(terminal, base_dir)? {
                        FilePickerResult::Selected(path) => Some(path),
                        FilePickerResult::Quit => None,
                    };
                }
                if !handle_folder_input(&mut state, key.code, key.modifiers) {
                    break None;
                }
            }
        }
    };

    Ok(result)
}

fn run_pedal_folder_control_wav(
    terminal: &mut Term,
    client: jack::Client,
    switcher: WavLoopProcessor<PedalSwitcherProcessor>,
    pedals: Vec<PedalEntry>,
    input_label: &str,
    connect_to: &str,
    base_dir: &Path,
) -> Result<Option<std::path::PathBuf>, Box<dyn std::error::Error>> {
    let controls = Arc::new(SharedControls::new());
    let (async_client, _our_in, our_out) = AudioEngine::start(client, switcher, controls.clone())?;

    // Only connect output port
    async_client
        .as_client()
        .connect_ports_by_name(&our_out, connect_to)?;
    if let Some(pair) = super::stereo_pair(connect_to) {
        let _ = async_client.as_client().connect_ports_by_name(&our_out, &pair);
    }

    let mut state = PedalFolderState {
        pedals,
        active_pedal: 0,
        controls,
        input_port: input_label.to_string(),
        output_port: super::stereo_display(connect_to),
        voltage: 9.0,
    };

    let result = loop {
        terminal.draw(|f| draw_pedal_folder(f, &state))?;

        if event::poll(std::time::Duration::from_millis(50))? {
            if let Event::Key(key) = event::read()? {
                if key.code == KeyCode::Char('p') || key.code == KeyCode::Char('P') {
                    drop(async_client);
                    break match run_file_picker(terminal, base_dir)? {
                        FilePickerResult::Selected(path) => Some(path),
                        FilePickerResult::Quit => None,
                    };
                }
                if !handle_folder_input(&mut state, key.code, key.modifiers) {
                    break None;
                }
            }
        }
    };

    Ok(result)
}

// ═════════════════════════════════════════════════════════════════════════════
// Main entry point
// ═════════════════════════════════════════════════════════════════════════════

/// Run the pedal folder TUI. Returns `Some(path)` to switch files, `None` on quit.
pub fn run(
    dir_path: &str,
    wav_input: Option<&str>,
) -> Result<Option<String>, Box<dyn std::error::Error>> {
    // Load WAV input if provided
    let wav_data = if let Some(path) = wav_input {
        let (samples, wav_sr) = pedalkernel::wav::read_wav_mono(std::path::Path::new(path))?;
        if samples.is_empty() {
            return Err("WAV file contains no samples".into());
        }
        Some((samples, wav_sr, path.to_string()))
    } else {
        None
    };

    // Create JACK client first to get sample rate for compilation
    let client = AudioEngine::create_client("pedalkernel")?;
    let sample_rate = client.sample_rate() as f64;

    // Warn if WAV sample rate differs from JACK
    if let Some((_, wav_sr, ref path)) = wav_data {
        let jack_sr = client.sample_rate() as u32;
        if wav_sr != jack_sr {
            eprintln!(
                "WARNING: WAV file '{}' sample rate ({} Hz) differs from JACK ({} Hz)",
                path, wav_sr, jack_sr
            );
        }
    }

    let base_dir = Path::new(dir_path);

    // Discover and pre-compile all pedals
    let discovered = discover_pedals(dir_path, sample_rate)?;

    if discovered.is_empty() {
        return Err(format!(
            "No valid .pedal files found in '{}' (all failed to load)",
            dir_path
        )
        .into());
    }

    eprintln!(
        "Found {} pedal(s): {}",
        discovered.len(),
        discovered
            .iter()
            .map(|p| p.name.as_str())
            .collect::<Vec<_>>()
            .join(", ")
    );

    // Split into processors and TUI state
    let mut processors = Vec::with_capacity(discovered.len());
    let mut pedal_entries = Vec::with_capacity(discovered.len());

    for dp in discovered {
        processors.push(dp.processor);
        pedal_entries.push(PedalEntry {
            name: dp.name,
            knobs: dp.knobs,
            selected_knob: 0,
            bypassed: false,
        });
    }

    let switcher = PedalSwitcherProcessor::new(processors);

    // Enumerate JACK ports
    let output_ports = AudioEngine::list_output_destinations(&client);
    let client_name = client.name().to_string();
    let output_ports: Vec<String> = output_ports
        .into_iter()
        .filter(|p| !p.starts_with(&format!("{client_name}:")))
        .collect();

    if output_ports.is_empty() {
        return Err(
            "No JACK output destinations found. Is JACK running with audio hardware?".into(),
        );
    }

    // Terminal setup
    enable_raw_mode()?;
    stdout().execute(EnterAlternateScreen)?;
    let backend = ratatui::backend::CrosstermBackend::new(stdout());
    let mut terminal = ratatui::Terminal::new(backend)?;

    let title = format!("Pedals: {} ({} found)", dir_path, pedal_entries.len());

    let result = if let Some((samples, _, ref wav_path)) = wav_data {
        // WAV input mode: output-only port selection
        let selected_out = match run_output_select(&mut terminal, &title, output_ports)? {
            OutputSelectResult::Selected(output) => output,
            OutputSelectResult::Quit => {
                disable_raw_mode()?;
                stdout().execute(LeaveAlternateScreen)?;
                return Ok(None);
            }
        };

        let wav_filename = std::path::Path::new(wav_path)
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or(wav_path);
        let input_label = format!("WAV: {} (looping)", wav_filename);

        let wav_switcher = WavLoopProcessor::new(samples, switcher);

        run_pedal_folder_control_wav(
            &mut terminal,
            client,
            wav_switcher,
            pedal_entries,
            &input_label,
            &selected_out,
            base_dir,
        )
    } else {
        // Normal mode: input + output port selection
        let input_ports = AudioEngine::list_input_sources(&client);
        let input_ports: Vec<String> = input_ports
            .into_iter()
            .filter(|p| !p.starts_with(&format!("{client_name}:")))
            .collect();

        if input_ports.is_empty() {
            disable_raw_mode()?;
            stdout().execute(LeaveAlternateScreen)?;
            return Err("No JACK input sources found. Is JACK running with audio hardware?".into());
        }

        let (selected_in, selected_out) =
            match run_port_select(&mut terminal, &title, input_ports, output_ports)? {
                PortSelectResult::Selected { input, output } => (input, output),
                PortSelectResult::Quit => {
                    disable_raw_mode()?;
                    stdout().execute(LeaveAlternateScreen)?;
                    return Ok(None);
                }
            };

        run_pedal_folder_control(
            &mut terminal,
            client,
            switcher,
            pedal_entries,
            &selected_in,
            &selected_out,
            base_dir,
        )
    };

    // Teardown
    disable_raw_mode()?;
    stdout().execute(LeaveAlternateScreen)?;

    result.map(|opt| opt.map(|p| p.to_string_lossy().into_owned()))
}
