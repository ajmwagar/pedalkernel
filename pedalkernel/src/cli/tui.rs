use crossterm::{
    event::{self, Event, KeyCode, KeyModifiers},
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
    ExecutableCommand,
};
use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::{AudioEngine, SharedControls, WavLoopProcessor};
use ratatui::{
    layout::{Alignment, Constraint, Layout},
    style::{Color, Style},
    text::{Line, Span},
    widgets::{Block, BorderType, Borders, Paragraph},
    Frame, Terminal,
};
use std::io::stdout;
use std::sync::Arc;

use super::tui_widgets::{
    render_footswitch, render_knob_row, run_output_select, run_port_select, KnobState,
    OutputSelectResult, PortSelectResult, Term,
};

// ═════════════════════════════════════════════════════════════════════════════
// Pedal control screen (live audio with knobs)
// ═════════════════════════════════════════════════════════════════════════════

struct PedalControlState {
    name: String,
    knobs: Vec<KnobState>,
    selected: usize,
    bypassed: bool,
    controls: Arc<SharedControls>,
    input_port: String,
    output_port: String,
}

enum PedalAction {
    Quit,
    NextKnob,
    PrevKnob,
    Adjust(f64),
    ResetKnob,
    ToggleBypass,
    None,
}

fn handle_pedal_key(code: KeyCode, modifiers: KeyModifiers) -> PedalAction {
    match code {
        KeyCode::Char('q') | KeyCode::Esc => PedalAction::Quit,
        KeyCode::Tab if modifiers.contains(KeyModifiers::SHIFT) => PedalAction::PrevKnob,
        KeyCode::BackTab => PedalAction::PrevKnob,
        KeyCode::Tab => PedalAction::NextKnob,
        KeyCode::Left => PedalAction::Adjust(-0.05),
        KeyCode::Right => PedalAction::Adjust(0.05),
        KeyCode::Down => PedalAction::Adjust(-0.01),
        KeyCode::Up => PedalAction::Adjust(0.01),
        KeyCode::Char(' ') => PedalAction::ToggleBypass,
        KeyCode::Char('r') | KeyCode::Char('R') => PedalAction::ResetKnob,
        _ => PedalAction::None,
    }
}

impl PedalControlState {
    /// Returns false when the user wants to quit.
    fn update(&mut self, action: PedalAction) -> bool {
        match action {
            PedalAction::Quit => return false,
            PedalAction::NextKnob => {
                if !self.knobs.is_empty() {
                    self.selected = (self.selected + 1) % self.knobs.len();
                }
            }
            PedalAction::PrevKnob => {
                if !self.knobs.is_empty() {
                    self.selected =
                        (self.selected + self.knobs.len() - 1) % self.knobs.len();
                }
            }
            PedalAction::Adjust(delta) => {
                if let Some(k) = self.knobs.get_mut(self.selected) {
                    let span = k.range.1 - k.range.0;
                    k.value = (k.value + delta * span).clamp(k.range.0, k.range.1);
                    self.controls.set_control(&k.label, k.value);
                }
            }
            PedalAction::ResetKnob => {
                if let Some(k) = self.knobs.get_mut(self.selected) {
                    k.value = k.default;
                    self.controls.set_control(&k.label, k.value);
                }
            }
            PedalAction::ToggleBypass => {
                self.bypassed = !self.bypassed;
                self.controls.set_bypassed(self.bypassed);
            }
            PedalAction::None => {}
        }
        true
    }
}

/// Run the pedal control screen with live JACK audio.
/// The JACK async client is kept alive on the stack for the duration.
fn run_pedal_control(
    terminal: &mut Term,
    client: jack::Client,
    pedal: &pedalkernel::dsl::PedalDef,
    connect_from: &str,
    connect_to: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut proc =
        compile_pedal(pedal, client.sample_rate() as f64)
            .map_err(|e| format!("Compilation error: {e}"))?;

    // Apply default control values
    for c in &pedal.controls {
        proc.set_control(&c.label, c.default);
    }

    let controls = Arc::new(SharedControls::new());
    let (async_client, our_in, our_out) =
        AudioEngine::start(client, proc, controls.clone())?;

    // Connect selected ports
    async_client
        .as_client()
        .connect_ports_by_name(connect_from, &our_in)?;
    async_client
        .as_client()
        .connect_ports_by_name(&our_out, connect_to)?;

    let knobs = pedal
        .controls
        .iter()
        .map(|c| KnobState {
            label: c.label.clone(),
            value: c.default,
            range: c.range,
            default: c.default,
        })
        .collect();

    let mut state = PedalControlState {
        name: pedal.name.clone(),
        knobs,
        selected: 0,
        bypassed: false,
        controls,
        input_port: connect_from.to_string(),
        output_port: connect_to.to_string(),
    };

    loop {
        terminal.draw(|f| draw_pedal_control(f, &state))?;

        if event::poll(std::time::Duration::from_millis(50))? {
            if let Event::Key(key) = event::read()? {
                let action = handle_pedal_key(key.code, key.modifiers);
                if !state.update(action) {
                    break;
                }
            }
        }
    }

    // async_client drops here, stopping JACK processing.
    drop(async_client);
    Ok(())
}

fn draw_pedal_control(frame: &mut Frame, state: &PedalControlState) {
    let area = frame.area();

    if area.width < 30 || area.height < 14 {
        let msg = Paragraph::new("Terminal too small!\nResize to at least 30x14.")
            .alignment(Alignment::Center);
        frame.render_widget(msg, area);
        return;
    }

    let outer = Block::default()
        .title(format!("  {}  ", state.name.to_uppercase()))
        .title_alignment(Alignment::Center)
        .borders(Borders::ALL)
        .border_type(BorderType::Double)
        .style(Style::default().fg(Color::White));
    let inner = outer.inner(area);
    frame.render_widget(outer, area);

    if state.knobs.is_empty() {
        let msg = Paragraph::new("No controls defined for this pedal.")
            .alignment(Alignment::Center);
        frame.render_widget(msg, inner);
        return;
    }

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
    let io_info = Paragraph::new(Line::from(vec![
        Span::styled(" IN: ", Style::default().fg(Color::Green)),
        Span::raw(&state.input_port),
        Span::raw("  "),
        Span::styled("OUT: ", Style::default().fg(Color::Green)),
        Span::raw(&state.output_port),
    ]))
    .alignment(Alignment::Center);
    frame.render_widget(io_info, v_chunks[0]);

    // Knob row (delegated to shared widget)
    render_knob_row(
        frame,
        &state.knobs,
        state.selected,
        v_chunks[2],
        v_chunks[3],
        v_chunks[4],
    );

    // Footswitch
    render_footswitch(frame, v_chunks[6], state.bypassed);

    // Help bar
    let help = Paragraph::new(Line::from(vec![
        Span::styled("←→", Style::default().fg(Color::Cyan)),
        Span::raw(" adjust  "),
        Span::styled("↑↓", Style::default().fg(Color::Cyan)),
        Span::raw(" fine  "),
        Span::styled("Tab", Style::default().fg(Color::Cyan)),
        Span::raw(" next  "),
        Span::styled("R", Style::default().fg(Color::Cyan)),
        Span::raw(" reset  "),
        Span::styled("Space", Style::default().fg(Color::Cyan)),
        Span::raw(" bypass  "),
        Span::styled("Q", Style::default().fg(Color::Cyan)),
        Span::raw(" quit"),
    ]))
    .alignment(Alignment::Center);
    frame.render_widget(help, v_chunks[7]);
}

// ═════════════════════════════════════════════════════════════════════════════
// Main entry point
// ═════════════════════════════════════════════════════════════════════════════

pub fn run(pedal_path: &str, wav_input: Option<&str>) -> Result<(), Box<dyn std::error::Error>> {
    let source = std::fs::read_to_string(pedal_path)?;
    let pedal = parse_pedal_file(&source).map_err(|e| format!("Parse error: {e}"))?;

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

    // Create JACK client and enumerate available ports
    let client = AudioEngine::create_client("pedalkernel").map_err(|e| {
        format!(
            "Failed to connect to JACK server: {e}\n\n\
             Make sure JACK is running. You can start it with:\n  \
             jackd -d alsa -r 48000 -p 256\n\
             or use a JACK control app like QjackCtl / Cadence."
        )
    })?;

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

    let output_ports = AudioEngine::list_output_destinations(&client);
    let client_name = client.name().to_string();
    let output_ports: Vec<String> = output_ports
        .into_iter()
        .filter(|p| !p.starts_with(&format!("{client_name}:")))
        .collect();

    if output_ports.is_empty() {
        return Err(
            "No JACK output destinations found. Is JACK running with audio hardware?"
                .into(),
        );
    }

    // Terminal setup
    enable_raw_mode()?;
    stdout().execute(EnterAlternateScreen)?;
    let backend = ratatui::backend::CrosstermBackend::new(stdout());
    let mut terminal = Terminal::new(backend)?;

    let result = if let Some((samples, _, ref wav_path)) = wav_data {
        // WAV input mode: output-only port selection
        let selected_out = match run_output_select(&mut terminal, &pedal.name, output_ports)? {
            OutputSelectResult::Selected(output) => output,
            OutputSelectResult::Quit => {
                disable_raw_mode()?;
                stdout().execute(LeaveAlternateScreen)?;
                return Ok(());
            }
        };

        let wav_filename = std::path::Path::new(wav_path)
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or(wav_path);
        let input_label = format!("WAV: {} (looping)", wav_filename);

        let mut proc =
            compile_pedal(&pedal, client.sample_rate() as f64)
                .map_err(|e| format!("Compilation error: {e}"))?;
        for c in &pedal.controls {
            proc.set_control(&c.label, c.default);
        }
        let wav_proc = WavLoopProcessor::new(samples, proc);

        let controls = Arc::new(SharedControls::new());
        let (async_client, _our_in, our_out) =
            AudioEngine::start(client, wav_proc, controls.clone())?;

        // Only connect output port
        async_client
            .as_client()
            .connect_ports_by_name(&our_out, &selected_out)?;

        let knobs = pedal
            .controls
            .iter()
            .map(|c| KnobState {
                label: c.label.clone(),
                value: c.default,
                range: c.range,
                default: c.default,
            })
            .collect();

        let mut state = PedalControlState {
            name: pedal.name.clone(),
            knobs,
            selected: 0,
            bypassed: false,
            controls,
            input_port: input_label,
            output_port: selected_out,
        };

        let r = (|| -> Result<(), Box<dyn std::error::Error>> {
            loop {
                terminal.draw(|f| draw_pedal_control(f, &state))?;
                if event::poll(std::time::Duration::from_millis(50))? {
                    if let Event::Key(key) = event::read()? {
                        let action = handle_pedal_key(key.code, key.modifiers);
                        if !state.update(action) {
                            break;
                        }
                    }
                }
            }
            drop(async_client);
            Ok(())
        })();
        r
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
            return Err(
                "No JACK input sources found. Is JACK running with audio hardware?".into(),
            );
        }

        let (selected_in, selected_out) = match run_port_select(
            &mut terminal,
            &pedal.name,
            input_ports,
            output_ports,
        )? {
            PortSelectResult::Selected { input, output } => (input, output),
            PortSelectResult::Quit => {
                disable_raw_mode()?;
                stdout().execute(LeaveAlternateScreen)?;
                return Ok(());
            }
        };

        run_pedal_control(
            &mut terminal,
            client,
            &pedal,
            &selected_in,
            &selected_out,
        )
    };

    // Teardown (always runs)
    disable_raw_mode()?;
    stdout().execute(LeaveAlternateScreen)?;

    result
}
