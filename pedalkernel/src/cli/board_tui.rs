//! Pedalboard TUI — overview + detail views with per-pedal knob editing and bypass.

use crossterm::{
    event::{self, Event, KeyCode, KeyModifiers},
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
    ExecutableCommand,
};
use pedalkernel::board::parse_board_file;
use pedalkernel::pedalboard::PedalboardProcessor;
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
    knob_frame_index, render_footswitch, render_knob_row, run_output_select, run_port_select,
    KnobState, OutputSelectResult, PortSelectResult, Term, KNOB_FRAMES,
};

// ═════════════════════════════════════════════════════════════════════════════
// State
// ═════════════════════════════════════════════════════════════════════════════

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum ViewMode {
    Overview,
    Detail,
}

pub(crate) struct PedalPanel {
    pub name: String,
    pub knobs: Vec<KnobState>,
    pub bypassed: bool,
}

pub(crate) struct BoardControlState {
    pub board_name: String,
    pub pedals: Vec<PedalPanel>,
    pub focused_pedal: usize,
    pub focused_knob: usize,
    pub board_bypassed: bool,
    pub view_mode: ViewMode,
    pub controls: Arc<SharedControls>,
    pub input_port: String,
    pub output_port: String,
}

// ═════════════════════════════════════════════════════════════════════════════
// Actions
// ═════════════════════════════════════════════════════════════════════════════

pub(crate) enum BoardAction {
    Quit,
    NextPedal,
    PrevPedal,
    NextKnob,
    PrevKnob,
    Adjust(f64),
    ResetKnob,
    ToggleBypass,
    ToggleBoardBypass,
    ToggleView,
    EnterDetail,
    None,
}

pub(crate) fn handle_board_key(code: KeyCode, modifiers: KeyModifiers) -> BoardAction {
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
        KeyCode::Char('b') | KeyCode::Char('B') => BoardAction::ToggleBoardBypass,
        KeyCode::Char('r') | KeyCode::Char('R') => BoardAction::ResetKnob,
        KeyCode::Char('v') | KeyCode::Char('V') => BoardAction::ToggleView,
        KeyCode::Enter => BoardAction::EnterDetail,
        _ => BoardAction::None,
    }
}

impl BoardControlState {
    /// Returns false when the user wants to quit.
    pub(crate) fn update(&mut self, action: BoardAction) -> bool {
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
                if self.view_mode == ViewMode::Detail {
                    if let Some(panel) = self.pedals.get(self.focused_pedal) {
                        if !panel.knobs.is_empty() {
                            self.focused_knob = (self.focused_knob + 1) % panel.knobs.len();
                        }
                    }
                }
            }
            BoardAction::PrevKnob => {
                if self.view_mode == ViewMode::Detail {
                    if let Some(panel) = self.pedals.get(self.focused_pedal) {
                        if !panel.knobs.is_empty() {
                            self.focused_knob =
                                (self.focused_knob + panel.knobs.len() - 1) % panel.knobs.len();
                        }
                    }
                }
            }
            BoardAction::Adjust(delta) => {
                if self.view_mode == ViewMode::Overview {
                    // In overview, left/right navigate pedals
                    if !self.pedals.is_empty() {
                        if delta < 0.0 {
                            self.focused_pedal =
                                (self.focused_pedal + self.pedals.len() - 1) % self.pedals.len();
                        } else {
                            self.focused_pedal = (self.focused_pedal + 1) % self.pedals.len();
                        }
                        self.focused_knob = 0;
                    }
                } else {
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
            }
            BoardAction::ResetKnob => {
                if self.view_mode == ViewMode::Detail {
                    let pidx = self.focused_pedal;
                    if let Some(panel) = self.pedals.get_mut(pidx) {
                        if let Some(k) = panel.knobs.get_mut(self.focused_knob) {
                            k.value = k.default;
                            self.controls
                                .set_control(&format!("{pidx}:{}", k.label), k.value);
                        }
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
            BoardAction::ToggleBoardBypass => {
                self.board_bypassed = !self.board_bypassed;
                self.controls.set_bypassed(self.board_bypassed);
            }
            BoardAction::ToggleView => {
                self.view_mode = match self.view_mode {
                    ViewMode::Overview => ViewMode::Detail,
                    ViewMode::Detail => ViewMode::Overview,
                };
            }
            BoardAction::EnterDetail => {
                if self.view_mode == ViewMode::Overview {
                    self.view_mode = ViewMode::Detail;
                }
            }
            BoardAction::None => {}
        }
        true
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// Drawing — dispatcher
// ═════════════════════════════════════════════════════════════════════════════

pub(crate) fn draw_board_control(frame: &mut Frame, state: &BoardControlState) {
    let area = frame.area();

    if area.width < 40 || area.height < 14 {
        let msg = Paragraph::new("Terminal too small!\nResize to at least 40x14.")
            .alignment(Alignment::Center);
        frame.render_widget(msg, area);
        return;
    }

    let title = if state.board_bypassed {
        format!("  {} [BYPASSED]  ", state.board_name.to_uppercase())
    } else {
        format!("  {}  ", state.board_name.to_uppercase())
    };
    let border_color = if state.board_bypassed {
        Color::DarkGray
    } else {
        Color::White
    };
    let outer = Block::default()
        .title(title)
        .title_alignment(Alignment::Center)
        .borders(Borders::ALL)
        .border_type(BorderType::Double)
        .style(Style::default().fg(border_color));
    let inner = outer.inner(area);
    frame.render_widget(outer, area);

    if state.pedals.is_empty() {
        let msg = Paragraph::new("No pedals in this board.").alignment(Alignment::Center);
        frame.render_widget(msg, inner);
        return;
    }

    match state.view_mode {
        ViewMode::Overview => draw_board_overview(frame, state, inner),
        ViewMode::Detail => draw_board_detail(frame, state, inner),
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// Drawing — Overview mode (mini pedal cards side-by-side)
// ═════════════════════════════════════════════════════════════════════════════

pub(crate) fn draw_board_overview(frame: &mut Frame, state: &BoardControlState, inner: Rect) {
    let v_chunks = Layout::vertical([
        Constraint::Length(1), // I/O info
        Constraint::Length(1), // padding
        Constraint::Min(12),   // pedal cards
        Constraint::Length(1), // help
    ])
    .split(inner);

    // I/O info bar
    render_io_bar(frame, v_chunks[0], &state.input_port, &state.output_port);

    // Pedal cards — split horizontally
    let n = state.pedals.len();
    let h_constraints: Vec<Constraint> = (0..n).map(|_| Constraint::Ratio(1, n as u32)).collect();
    let card_cols = Layout::horizontal(&h_constraints).split(v_chunks[2]);

    for (i, panel) in state.pedals.iter().enumerate() {
        let is_focused = i == state.focused_pedal;
        render_mini_pedal(frame, card_cols[i], panel, is_focused);
    }

    // Help bar
    let help = Paragraph::new(Line::from(vec![
        Span::styled("[ ] ←→", Style::default().fg(Color::Cyan)),
        Span::raw(" pedal  "),
        Span::styled("V", Style::default().fg(Color::Cyan)),
        Span::raw("/"),
        Span::styled("Enter", Style::default().fg(Color::Cyan)),
        Span::raw(" detail  "),
        Span::styled("Space", Style::default().fg(Color::Cyan)),
        Span::raw(" bypass  "),
        Span::styled("B", Style::default().fg(Color::Cyan)),
        Span::raw(" bypass all  "),
        Span::styled("Q", Style::default().fg(Color::Cyan)),
        Span::raw(" quit"),
    ]))
    .alignment(Alignment::Center);
    frame.render_widget(help, v_chunks[3]);
}

/// Render a single mini pedal card within its allocated rect.
pub(crate) fn render_mini_pedal(
    frame: &mut Frame,
    area: Rect,
    panel: &PedalPanel,
    is_focused: bool,
) {
    let (border_style, title_style) = if is_focused {
        (
            Style::default()
                .fg(Color::Yellow)
                .add_modifier(Modifier::BOLD),
            Style::default()
                .fg(Color::Yellow)
                .add_modifier(Modifier::BOLD),
        )
    } else if panel.bypassed {
        (
            Style::default()
                .fg(Color::DarkGray)
                .add_modifier(Modifier::DIM),
            Style::default()
                .fg(Color::DarkGray)
                .add_modifier(Modifier::DIM),
        )
    } else {
        (
            Style::default().fg(Color::White),
            Style::default().fg(Color::White),
        )
    };

    let block = Block::default()
        .title(Span::styled(
            format!(" {} ", panel.name.to_uppercase()),
            title_style,
        ))
        .title_alignment(Alignment::Center)
        .borders(Borders::ALL)
        .border_type(if is_focused {
            BorderType::Double
        } else {
            BorderType::Rounded
        })
        .style(border_style);
    let card_inner = block.inner(area);
    frame.render_widget(block, area);

    if card_inner.height < 3 || card_inner.width < 4 {
        return;
    }

    // Layout inside card: knobs + footswitch
    let card_chunks = Layout::vertical([
        Constraint::Length(5), // knob art
        Constraint::Length(1), // labels
        Constraint::Length(1), // values
        Constraint::Min(0),    // spacer
        Constraint::Length(3), // footswitch
    ])
    .split(card_inner);

    // Render mini knobs (no selection highlight in overview)
    if !panel.knobs.is_empty() {
        render_mini_knobs(
            frame,
            &panel.knobs,
            card_chunks[0],
            card_chunks[1],
            card_chunks[2],
            panel.bypassed,
        );
    }

    // Footswitch
    if card_chunks[4].height >= 3 {
        render_footswitch(frame, card_chunks[4], panel.bypassed);
    }
}

/// Render knobs compactly inside a mini pedal card (no selection highlight).
fn render_mini_knobs(
    frame: &mut Frame,
    knobs: &[KnobState],
    knob_area: Rect,
    label_area: Rect,
    value_area: Rect,
    dimmed: bool,
) {
    let n = knobs.len();
    if n == 0 {
        return;
    }
    let h_constraints: Vec<Constraint> = (0..n).map(|_| Constraint::Ratio(1, n as u32)).collect();
    let knob_cols = Layout::horizontal(&h_constraints).split(knob_area);
    let label_cols = Layout::horizontal(&h_constraints).split(label_area);
    let value_cols = Layout::horizontal(&h_constraints).split(value_area);

    let style = if dimmed {
        Style::default()
            .fg(Color::DarkGray)
            .add_modifier(Modifier::DIM)
    } else {
        Style::default().fg(Color::White)
    };

    for (i, knob) in knobs.iter().enumerate() {
        let fi = knob_frame_index(knob.value, knob.range);
        let art = KNOB_FRAMES[fi];

        let lines: Vec<Line> = art
            .iter()
            .map(|l| Line::from(Span::styled(*l, style)))
            .collect();
        frame.render_widget(
            Paragraph::new(lines).alignment(Alignment::Center),
            knob_cols[i],
        );

        frame.render_widget(
            Paragraph::new(Span::styled(&knob.label, style)).alignment(Alignment::Center),
            label_cols[i],
        );

        frame.render_widget(
            Paragraph::new(Span::styled(format!("{:.2}", knob.value), style))
                .alignment(Alignment::Center),
            value_cols[i],
        );
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// Drawing — Detail mode (focused pedal with full knob controls)
// ═════════════════════════════════════════════════════════════════════════════

pub(crate) fn draw_board_detail(frame: &mut Frame, state: &BoardControlState, inner: Rect) {
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
    render_io_bar(frame, v_chunks[0], &state.input_port, &state.output_port);

    // Chain bar
    render_chain_bar(frame, v_chunks[2], &state.pedals, state.focused_pedal);

    // Status row
    render_status_row(frame, v_chunks[3], &state.pedals);

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
        Span::styled("[ ]", Style::default().fg(Color::Cyan)),
        Span::raw(" pedal  "),
        Span::styled("Tab", Style::default().fg(Color::Cyan)),
        Span::raw(" knob  "),
        Span::styled("←→", Style::default().fg(Color::Cyan)),
        Span::raw(" adjust  "),
        Span::styled("↑↓", Style::default().fg(Color::Cyan)),
        Span::raw(" fine  "),
        Span::styled("Space", Style::default().fg(Color::Cyan)),
        Span::raw(" bypass  "),
        Span::styled("B", Style::default().fg(Color::Cyan)),
        Span::raw(" bypass all  "),
        Span::styled("R", Style::default().fg(Color::Cyan)),
        Span::raw(" reset  "),
        Span::styled("V", Style::default().fg(Color::Cyan)),
        Span::raw(" overview  "),
        Span::styled("Q", Style::default().fg(Color::Cyan)),
        Span::raw(" quit"),
    ]))
    .alignment(Alignment::Center);
    frame.render_widget(help, v_chunks[12]);
}

// ═════════════════════════════════════════════════════════════════════════════
// Shared drawing helpers
// ═════════════════════════════════════════════════════════════════════════════

pub(crate) fn render_io_bar(frame: &mut Frame, area: Rect, input_port: &str, output_port: &str) {
    let io_info = Paragraph::new(Line::from(vec![
        Span::styled(" IN: ", Style::default().fg(Color::Green)),
        Span::raw(input_port),
        Span::raw("  "),
        Span::styled("OUT: ", Style::default().fg(Color::Green)),
        Span::raw(output_port),
    ]))
    .alignment(Alignment::Center);
    frame.render_widget(io_info, area);
}

/// Render the chain bar: [TUBE SCREAMER] → [BLUES DRIVER] → ...
pub(crate) fn render_chain_bar(
    frame: &mut Frame,
    area: Rect,
    pedals: &[PedalPanel],
    focused: usize,
) {
    let mut spans: Vec<Span> = Vec::new();
    for (i, panel) in pedals.iter().enumerate() {
        if i > 0 {
            spans.push(Span::styled(" → ", Style::default().fg(Color::DarkGray)));
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
pub(crate) fn render_status_row(frame: &mut Frame, area: Rect, pedals: &[PedalPanel]) {
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
        let padding = name_len.saturating_sub(label.len()) / 2;
        let padded = format!("{:>width$}", label, width = padding + label.len());
        spans.push(Span::styled(padded, style));
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
    let (async_client, our_in, our_out) = AudioEngine::start(client, processor, controls.clone())?;

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
        board_bypassed: false,
        view_mode: ViewMode::Overview,
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

fn run_board_control_wav(
    terminal: &mut Term,
    client: jack::Client,
    processor: WavLoopProcessor<PedalboardProcessor>,
    panels: Vec<PedalPanel>,
    board_name: &str,
    input_label: &str,
    connect_to: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let controls = Arc::new(SharedControls::new());
    let (async_client, _our_in, our_out) = AudioEngine::start(client, processor, controls.clone())?;

    // Only connect output port
    async_client
        .as_client()
        .connect_ports_by_name(&our_out, connect_to)?;

    let mut state = BoardControlState {
        board_name: board_name.to_string(),
        pedals: panels,
        focused_pedal: 0,
        focused_knob: 0,
        board_bypassed: false,
        view_mode: ViewMode::Overview,
        controls,
        input_port: input_label.to_string(),
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

pub fn run(board_path: &str, wav_input: Option<&str>) -> Result<(), Box<dyn std::error::Error>> {
    let source = std::fs::read_to_string(board_path)?;
    let board = parse_board_file(&source).map_err(|e| format!("Parse error: {e}"))?;

    let board_dir = Path::new(board_path)
        .parent()
        .unwrap_or_else(|| Path::new("."));

    // Load global utilities from ~/.config/pedalkernel/utilities
    let global_utilities = match pedalkernel::config::load_global_utilities() {
        Ok(gu) => gu,
        Err(e) => {
            eprintln!("WARNING: {e}");
            None
        }
    };

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

    // Build pedal panels (need control info from .pedal files for the TUI).
    // If a pedal file can't be read/parsed, create a failed panel (no knobs)
    // — the processor will use dry passthrough for that slot.
    let mut panels = Vec::new();
    for entry in &board.pedals {
        let pedal_path = board_dir.join(&entry.path);

        let panel = match std::fs::read_to_string(&pedal_path)
            .map_err(|e| e.to_string())
            .and_then(|src| pedalkernel::dsl::parse_pedal_file(&src).map_err(|e| e.to_string()))
        {
            Ok(pedal_def) => {
                let knobs: Vec<KnobState> = pedal_def
                    .controls
                    .iter()
                    .map(|c| {
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
                PedalPanel {
                    name: pedal_def.name.clone(),
                    knobs,
                    bypassed: false,
                }
            }
            Err(e) => {
                eprintln!(
                    "WARNING: Pedal '{}' failed to load — dry passthrough: {e}",
                    entry.id
                );
                PedalPanel {
                    name: format!("{} [FAILED]", entry.id),
                    knobs: Vec::new(),
                    bypassed: false,
                }
            }
        };

        panels.push(panel);
    }

    // Create JACK client and enumerate available ports
    let client = AudioEngine::create_client("pedalkernel")?;

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
            "No JACK output destinations found. Is JACK running with audio hardware?".into(),
        );
    }

    // Terminal setup
    enable_raw_mode()?;
    stdout().execute(EnterAlternateScreen)?;
    let backend = ratatui::backend::CrosstermBackend::new(stdout());
    let mut terminal = ratatui::Terminal::new(backend)?;

    let result = if let Some((samples, _, ref wav_path)) = wav_data {
        // WAV input mode: output-only port selection
        let selected_out = match run_output_select(&mut terminal, &board.name, output_ports)? {
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

        let (processor, build_warnings) = PedalboardProcessor::from_board_with_options(
            &board,
            board_dir,
            client.sample_rate() as f64,
            global_utilities.as_ref(),
        );
        for w in &build_warnings {
            eprintln!("WARNING: {w}");
        }
        let wav_proc = WavLoopProcessor::new(samples, processor);

        run_board_control_wav(
            &mut terminal,
            client,
            wav_proc,
            panels,
            &board.name,
            &input_label,
            &selected_out,
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
            match run_port_select(&mut terminal, &board.name, input_ports, output_ports)? {
                PortSelectResult::Selected { input, output } => (input, output),
                PortSelectResult::Quit => {
                    disable_raw_mode()?;
                    stdout().execute(LeaveAlternateScreen)?;
                    return Ok(());
                }
            };

        let (processor, build_warnings) = PedalboardProcessor::from_board_with_options(
            &board,
            board_dir,
            client.sample_rate() as f64,
            global_utilities.as_ref(),
        );
        for w in &build_warnings {
            eprintln!("WARNING: {w}");
        }

        run_board_control(
            &mut terminal,
            client,
            processor,
            panels,
            &board.name,
            &selected_in,
            &selected_out,
        )
    };

    // Teardown (always runs)
    disable_raw_mode()?;
    stdout().execute(LeaveAlternateScreen)?;

    result
}
