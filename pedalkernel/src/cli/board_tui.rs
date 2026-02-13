//! Pedalboard TUI — chain view with per-pedal knob editing and bypass.

use crossterm::{
    event::{self, Event, KeyCode, KeyModifiers},
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
    ExecutableCommand,
};
use pedalkernel::board::parse_board_file;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::pedalboard::PedalboardProcessor;
use pedalkernel::{AudioEngine, SharedControls};
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
    render_footswitch, render_knob_row, run_port_select, KnobState, PortSelectResult, Term,
};

// ═════════════════════════════════════════════════════════════════════════════
// State
// ═════════════════════════════════════════════════════════════════════════════

struct PedalPanel {
    name: String,
    knobs: Vec<KnobState>,
    bypassed: bool,
}

struct BoardControlState {
    board_name: String,
    pedals: Vec<PedalPanel>,
    focused_pedal: usize,
    focused_knob: usize,
    controls: Arc<SharedControls>,
    input_port: String,
    output_port: String,
}

// ═════════════════════════════════════════════════════════════════════════════
// Actions
// ═════════════════════════════════════════════════════════════════════════════

enum BoardAction {
    Quit,
    NextPedal,
    PrevPedal,
    NextKnob,
    PrevKnob,
    Adjust(f64),
    ResetKnob,
    ToggleBypass,
    None,
}

fn handle_board_key(code: KeyCode, modifiers: KeyModifiers) -> BoardAction {
    match code {
        KeyCode::Char('q') | KeyCode::Esc => BoardAction::Quit,
        KeyCode::Char(']') => BoardAction::NextPedal,
        KeyCode::Char('[') => BoardAction::PrevPedal,
        KeyCode::Tab if modifiers.contains(KeyModifiers::SHIFT) => BoardAction::PrevKnob,
        KeyCode::BackTab => BoardAction::PrevKnob,
        KeyCode::Tab => BoardAction::NextKnob,
        KeyCode::Left => BoardAction::Adjust(-0.05),
        KeyCode::Right => BoardAction::Adjust(0.05),
        KeyCode::Down => BoardAction::Adjust(-0.01),
        KeyCode::Up => BoardAction::Adjust(0.01),
        KeyCode::Char(' ') => BoardAction::ToggleBypass,
        KeyCode::Char('r') | KeyCode::Char('R') => BoardAction::ResetKnob,
        _ => BoardAction::None,
    }
}

impl BoardControlState {
    /// Returns false when the user wants to quit.
    fn update(&mut self, action: BoardAction) -> bool {
        match action {
            BoardAction::Quit => return false,
            BoardAction::NextPedal => {
                if !self.pedals.is_empty() {
                    self.focused_pedal = (self.focused_pedal + 1) % self.pedals.len();
                    self.focused_knob = 0;
                }
            }
            BoardAction::PrevPedal => {
                if !self.pedals.is_empty() {
                    self.focused_pedal =
                        (self.focused_pedal + self.pedals.len() - 1) % self.pedals.len();
                    self.focused_knob = 0;
                }
            }
            BoardAction::NextKnob => {
                if let Some(panel) = self.pedals.get(self.focused_pedal) {
                    if !panel.knobs.is_empty() {
                        self.focused_knob = (self.focused_knob + 1) % panel.knobs.len();
                    }
                }
            }
            BoardAction::PrevKnob => {
                if let Some(panel) = self.pedals.get(self.focused_pedal) {
                    if !panel.knobs.is_empty() {
                        self.focused_knob =
                            (self.focused_knob + panel.knobs.len() - 1) % panel.knobs.len();
                    }
                }
            }
            BoardAction::Adjust(delta) => {
                let pidx = self.focused_pedal;
                if let Some(panel) = self.pedals.get_mut(pidx) {
                    if let Some(k) = panel.knobs.get_mut(self.focused_knob) {
                        let span = k.range.1 - k.range.0;
                        k.value = (k.value + delta * span).clamp(k.range.0, k.range.1);
                        self.controls
                            .set_control(&format!("{pidx}:{}", k.label), k.value);
                    }
                }
            }
            BoardAction::ResetKnob => {
                let pidx = self.focused_pedal;
                if let Some(panel) = self.pedals.get_mut(pidx) {
                    if let Some(k) = panel.knobs.get_mut(self.focused_knob) {
                        k.value = k.default;
                        self.controls
                            .set_control(&format!("{pidx}:{}", k.label), k.value);
                    }
                }
            }
            BoardAction::ToggleBypass => {
                let pidx = self.focused_pedal;
                if let Some(panel) = self.pedals.get_mut(pidx) {
                    panel.bypassed = !panel.bypassed;
                    let val = if panel.bypassed { 1.0 } else { 0.0 };
                    self.controls
                        .set_control(&format!("{pidx}:__bypass__"), val);
                }
            }
            BoardAction::None => {}
        }
        true
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// Drawing
// ═════════════════════════════════════════════════════════════════════════════

fn draw_board_control(frame: &mut Frame, state: &BoardControlState) {
    let area = frame.area();

    if area.width < 40 || area.height < 18 {
        let msg = Paragraph::new("Terminal too small!\nResize to at least 40x18.")
            .alignment(Alignment::Center);
        frame.render_widget(msg, area);
        return;
    }

    let outer = Block::default()
        .title(format!("  {}  ", state.board_name.to_uppercase()))
        .title_alignment(Alignment::Center)
        .borders(Borders::ALL)
        .border_type(BorderType::Double)
        .style(Style::default().fg(Color::White));
    let inner = outer.inner(area);
    frame.render_widget(outer, area);

    if state.pedals.is_empty() {
        let msg = Paragraph::new("No pedals in this board.")
            .alignment(Alignment::Center);
        frame.render_widget(msg, inner);
        return;
    }

    let v_chunks = Layout::vertical([
        Constraint::Length(1), // I/O info
        Constraint::Length(1), // padding
        Constraint::Length(1), // chain bar
        Constraint::Length(1), // status row
        Constraint::Length(1), // padding
        Constraint::Length(1), // focused pedal header
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

    // Chain bar: [PEDAL1] → [PEDAL2] → [PEDAL3]
    render_chain_bar(frame, v_chunks[2], &state.pedals, state.focused_pedal);

    // Status row: ACTIVE / BYPASSED under each pedal
    render_status_row(frame, v_chunks[3], &state.pedals, state.focused_pedal);

    // Focused pedal header
    if let Some(panel) = state.pedals.get(state.focused_pedal) {
        let header_style = Style::default()
            .fg(Color::Yellow)
            .add_modifier(Modifier::BOLD);
        let header = Paragraph::new(Line::from(Span::styled(
            format!("── {} ──", panel.name.to_uppercase()),
            header_style,
        )))
        .alignment(Alignment::Center);
        frame.render_widget(header, v_chunks[5]);

        // Knob row
        if !panel.knobs.is_empty() {
            render_knob_row(
                frame,
                &panel.knobs,
                state.focused_knob,
                v_chunks[7],
                v_chunks[8],
                v_chunks[9],
            );
        } else {
            let msg = Paragraph::new("No controls for this pedal.")
                .alignment(Alignment::Center)
                .style(Style::default().fg(Color::DarkGray));
            frame.render_widget(msg, v_chunks[7]);
        }

        // Footswitch for focused pedal
        render_footswitch(frame, v_chunks[11], panel.bypassed);
    }

    // Help bar
    let help = Paragraph::new(Line::from(vec![
        Span::styled("[/]", Style::default().fg(Color::Cyan)),
        Span::raw(" pedal  "),
        Span::styled("Tab", Style::default().fg(Color::Cyan)),
        Span::raw(" knob  "),
        Span::styled("←→", Style::default().fg(Color::Cyan)),
        Span::raw(" adjust  "),
        Span::styled("↑↓", Style::default().fg(Color::Cyan)),
        Span::raw(" fine  "),
        Span::styled("Space", Style::default().fg(Color::Cyan)),
        Span::raw(" bypass  "),
        Span::styled("R", Style::default().fg(Color::Cyan)),
        Span::raw(" reset  "),
        Span::styled("Q", Style::default().fg(Color::Cyan)),
        Span::raw(" quit"),
    ]))
    .alignment(Alignment::Center);
    frame.render_widget(help, v_chunks[12]);
}

/// Render the chain bar: [TUBE SCREAMER] → [BLUES DRIVER] → ...
fn render_chain_bar(frame: &mut Frame, area: Rect, pedals: &[PedalPanel], focused: usize) {
    let mut spans: Vec<Span> = Vec::new();
    for (i, panel) in pedals.iter().enumerate() {
        if i > 0 {
            spans.push(Span::styled(
                " → ",
                Style::default().fg(Color::DarkGray),
            ));
        }
        let name = panel.name.to_uppercase();
        let style = if i == focused {
            Style::default()
                .fg(Color::Yellow)
                .add_modifier(Modifier::BOLD)
        } else if panel.bypassed {
            Style::default()
                .fg(Color::DarkGray)
                .add_modifier(Modifier::DIM)
        } else {
            Style::default().fg(Color::White)
        };
        spans.push(Span::styled(format!("[{name}]"), style));
    }
    let line = Paragraph::new(Line::from(spans)).alignment(Alignment::Center);
    frame.render_widget(line, area);
}

/// Render status indicators under each pedal name in the chain.
fn render_status_row(frame: &mut Frame, area: Rect, pedals: &[PedalPanel], _focused: usize) {
    let mut spans: Vec<Span> = Vec::new();
    for (i, panel) in pedals.iter().enumerate() {
        if i > 0 {
            spans.push(Span::raw("   ")); // matches " → " width
        }
        let name_len = panel.name.len() + 2; // account for [ ]
        let (label, style) = if panel.bypassed {
            (
                "BYPASSED",
                Style::default()
                    .fg(Color::DarkGray)
                    .add_modifier(Modifier::DIM),
            )
        } else {
            ("ACTIVE", Style::default().fg(Color::Green))
        };
        // Center the status text under the pedal name box
        let padding = name_len.saturating_sub(label.len()) / 2;
        let padded = format!("{:>width$}", label, width = padding + label.len());
        spans.push(Span::styled(padded, style));
        // Fill remaining space
        let remaining = name_len.saturating_sub(padding + label.len());
        if remaining > 0 {
            spans.push(Span::raw(" ".repeat(remaining)));
        }
    }
    let line = Paragraph::new(Line::from(spans)).alignment(Alignment::Center);
    frame.render_widget(line, area);
}

// ═════════════════════════════════════════════════════════════════════════════
// Board control loop
// ═════════════════════════════════════════════════════════════════════════════

fn run_board_control(
    terminal: &mut Term,
    client: jack::Client,
    processor: PedalboardProcessor,
    panels: Vec<PedalPanel>,
    board_name: &str,
    connect_from: &str,
    connect_to: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let controls = Arc::new(SharedControls::new());
    let (async_client, our_in, our_out) =
        AudioEngine::start(client, processor, controls.clone())?;

    // Connect selected ports
    async_client
        .as_client()
        .connect_ports_by_name(connect_from, &our_in)?;
    async_client
        .as_client()
        .connect_ports_by_name(&our_out, connect_to)?;

    let mut state = BoardControlState {
        board_name: board_name.to_string(),
        pedals: panels,
        focused_pedal: 0,
        focused_knob: 0,
        controls,
        input_port: connect_from.to_string(),
        output_port: connect_to.to_string(),
    };

    loop {
        terminal.draw(|f| draw_board_control(f, &state))?;

        if event::poll(std::time::Duration::from_millis(50))? {
            if let Event::Key(key) = event::read()? {
                let action = handle_board_key(key.code, key.modifiers);
                if !state.update(action) {
                    break;
                }
            }
        }
    }

    drop(async_client);
    Ok(())
}

// ═════════════════════════════════════════════════════════════════════════════
// Main entry point
// ═════════════════════════════════════════════════════════════════════════════

pub fn run(board_path: &str) -> Result<(), Box<dyn std::error::Error>> {
    let source = std::fs::read_to_string(board_path)?;
    let board = parse_board_file(&source).map_err(|e| format!("Parse error: {e}"))?;

    let board_dir = Path::new(board_path)
        .parent()
        .unwrap_or_else(|| Path::new("."));

    // Build pedal panels (need control info from .pedal files for the TUI)
    let mut panels = Vec::new();
    for entry in &board.pedals {
        let pedal_path = board_dir.join(&entry.path);
        let pedal_source = std::fs::read_to_string(&pedal_path).map_err(|e| {
            format!(
                "Error reading pedal '{}' at {}: {e}",
                entry.id,
                pedal_path.display()
            )
        })?;
        let pedal_def = parse_pedal_file(&pedal_source)
            .map_err(|e| format!("Parse error in '{}': {e}", entry.id))?;

        let knobs: Vec<KnobState> = pedal_def
            .controls
            .iter()
            .map(|c| {
                // Check for board-level override for the default value
                let default_val = entry
                    .overrides
                    .iter()
                    .find(|(label, _)| label.eq_ignore_ascii_case(&c.label))
                    .map(|(_, v)| *v)
                    .unwrap_or(c.default);
                KnobState {
                    label: c.label.clone(),
                    value: default_val,
                    range: c.range,
                    default: default_val,
                }
            })
            .collect();

        panels.push(PedalPanel {
            name: pedal_def.name.clone(),
            knobs,
            bypassed: false,
        });
    }

    // Create JACK client and enumerate available ports
    let client = AudioEngine::create_client("pedalkernel")?;
    let input_ports = AudioEngine::list_input_sources(&client);
    let output_ports = AudioEngine::list_output_destinations(&client);

    // Filter out our own ports
    let client_name = client.name().to_string();
    let input_ports: Vec<String> = input_ports
        .into_iter()
        .filter(|p| !p.starts_with(&format!("{client_name}:")))
        .collect();
    let output_ports: Vec<String> = output_ports
        .into_iter()
        .filter(|p| !p.starts_with(&format!("{client_name}:")))
        .collect();

    if input_ports.is_empty() {
        return Err(
            "No JACK input sources found. Is JACK running with audio hardware?".into(),
        );
    }
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
    let mut terminal = ratatui::Terminal::new(backend)?;

    // Phase 1: port selection
    let (selected_in, selected_out) = match run_port_select(
        &mut terminal,
        &board.name,
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

    // Build the pedalboard processor
    let processor =
        PedalboardProcessor::from_board(&board, board_dir, client.sample_rate() as f64)
            .map_err(|e| format!("Board compilation error: {e}"))?;

    // Phase 2: board control with live JACK audio
    let result = run_board_control(
        &mut terminal,
        client,
        processor,
        panels,
        &board.name,
        &selected_in,
        &selected_out,
    );

    // Teardown (always runs)
    disable_raw_mode()?;
    stdout().execute(LeaveAlternateScreen)?;

    result
}
