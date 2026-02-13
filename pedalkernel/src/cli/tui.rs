use crossterm::{
    event::{self, Event, KeyCode, KeyModifiers},
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
    ExecutableCommand,
};
use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::{AudioEngine, SharedControls};
use ratatui::{
    layout::{Alignment, Constraint, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, BorderType, List, ListItem, Paragraph},
    Frame, Terminal,
};
use std::io::stdout;
use std::sync::Arc;

type Term = Terminal<ratatui::backend::CrosstermBackend<std::io::Stdout>>;

// ═════════════════════════════════════════════════════════════════════════════
// Port selection
// ═════════════════════════════════════════════════════════════════════════════

#[derive(Clone, Copy, PartialEq, Eq)]
enum PortPanel {
    Input,
    Output,
}

struct PortSelectState {
    input_ports: Vec<String>,
    output_ports: Vec<String>,
    input_cursor: usize,
    output_cursor: usize,
    active_panel: PortPanel,
}

impl PortSelectState {
    fn new(input_ports: Vec<String>, output_ports: Vec<String>) -> Self {
        Self {
            input_ports,
            output_ports,
            input_cursor: 0,
            output_cursor: 0,
            active_panel: PortPanel::Input,
        }
    }

    fn selected_input(&self) -> Option<&str> {
        self.input_ports.get(self.input_cursor).map(|s| s.as_str())
    }

    fn selected_output(&self) -> Option<&str> {
        self.output_ports
            .get(self.output_cursor)
            .map(|s| s.as_str())
    }

    fn cursor_up(&mut self) {
        match self.active_panel {
            PortPanel::Input => {
                if self.input_cursor > 0 {
                    self.input_cursor -= 1;
                }
            }
            PortPanel::Output => {
                if self.output_cursor > 0 {
                    self.output_cursor -= 1;
                }
            }
        }
    }

    fn cursor_down(&mut self) {
        match self.active_panel {
            PortPanel::Input => {
                if self.input_cursor + 1 < self.input_ports.len() {
                    self.input_cursor += 1;
                }
            }
            PortPanel::Output => {
                if self.output_cursor + 1 < self.output_ports.len() {
                    self.output_cursor += 1;
                }
            }
        }
    }

    fn toggle_panel(&mut self) {
        self.active_panel = match self.active_panel {
            PortPanel::Input => PortPanel::Output,
            PortPanel::Output => PortPanel::Input,
        };
    }
}

/// Result of the port selection screen.
enum PortSelectResult {
    Selected { input: String, output: String },
    Quit,
}

/// Run the port selection screen. Returns the selected ports or Quit.
fn run_port_select(
    terminal: &mut Term,
    pedal_name: &str,
    input_ports: Vec<String>,
    output_ports: Vec<String>,
) -> Result<PortSelectResult, Box<dyn std::error::Error>> {
    let mut state = PortSelectState::new(input_ports, output_ports);

    loop {
        terminal.draw(|f| draw_port_select(f, &state, pedal_name))?;

        if event::poll(std::time::Duration::from_millis(50))? {
            if let Event::Key(key) = event::read()? {
                match key.code {
                    KeyCode::Char('q') | KeyCode::Esc => return Ok(PortSelectResult::Quit),
                    KeyCode::Up => state.cursor_up(),
                    KeyCode::Down => state.cursor_down(),
                    KeyCode::Tab | KeyCode::BackTab => state.toggle_panel(),
                    KeyCode::Enter => {
                        let input = state
                            .selected_input()
                            .unwrap_or_default()
                            .to_string();
                        let output = state
                            .selected_output()
                            .unwrap_or_default()
                            .to_string();
                        if !input.is_empty() && !output.is_empty() {
                            return Ok(PortSelectResult::Selected { input, output });
                        }
                    }
                    _ => {}
                }
            }
        }
    }
}

fn draw_port_select(frame: &mut Frame, state: &PortSelectState, pedal_name: &str) {
    let area = frame.area();

    if area.width < 40 || area.height < 12 {
        let msg = Paragraph::new("Terminal too small!\nResize to at least 40x12.")
            .alignment(Alignment::Center);
        frame.render_widget(msg, area);
        return;
    }

    let outer = Block::default()
        .title(format!(
            "  {} — JACK Port Selection  ",
            pedal_name.to_uppercase()
        ))
        .title_alignment(Alignment::Center)
        .borders(Borders::ALL)
        .border_type(BorderType::Double)
        .style(Style::default().fg(Color::White));
    let inner = outer.inner(area);
    frame.render_widget(outer, area);

    let v_chunks = Layout::vertical([
        Constraint::Length(1), // top padding
        Constraint::Length(1), // column headers
        Constraint::Min(3),    // port lists
        Constraint::Length(1), // gap
        Constraint::Length(1), // help bar
    ])
    .split(inner);

    // Column headers
    let h_cols =
        Layout::horizontal([Constraint::Percentage(50), Constraint::Percentage(50)])
            .split(v_chunks[1]);

    let input_style = if state.active_panel == PortPanel::Input {
        Style::default()
            .fg(Color::Yellow)
            .add_modifier(Modifier::BOLD)
    } else {
        Style::default().fg(Color::White)
    };
    let output_style = if state.active_panel == PortPanel::Output {
        Style::default()
            .fg(Color::Yellow)
            .add_modifier(Modifier::BOLD)
    } else {
        Style::default().fg(Color::White)
    };

    frame.render_widget(
        Paragraph::new(Span::styled(" Input (Source)", input_style)),
        h_cols[0],
    );
    frame.render_widget(
        Paragraph::new(Span::styled(" Output (Destination)", output_style)),
        h_cols[1],
    );

    // Port lists
    let list_cols =
        Layout::horizontal([Constraint::Percentage(50), Constraint::Percentage(50)])
            .split(v_chunks[2]);

    draw_port_list(
        frame,
        list_cols[0],
        &state.input_ports,
        state.input_cursor,
        state.active_panel == PortPanel::Input,
    );
    draw_port_list(
        frame,
        list_cols[1],
        &state.output_ports,
        state.output_cursor,
        state.active_panel == PortPanel::Output,
    );

    // Help bar
    let help = Paragraph::new(Line::from(vec![
        Span::styled("↑↓", Style::default().fg(Color::Cyan)),
        Span::raw(" navigate  "),
        Span::styled("Tab", Style::default().fg(Color::Cyan)),
        Span::raw(" switch panel  "),
        Span::styled("Enter", Style::default().fg(Color::Cyan)),
        Span::raw(" confirm  "),
        Span::styled("Q", Style::default().fg(Color::Cyan)),
        Span::raw(" quit"),
    ]))
    .alignment(Alignment::Center);
    frame.render_widget(help, v_chunks[4]);
}

fn draw_port_list(
    frame: &mut Frame,
    area: Rect,
    ports: &[String],
    cursor: usize,
    is_active: bool,
) {
    let block = Block::default()
        .borders(Borders::ALL)
        .border_type(BorderType::Rounded);

    if ports.is_empty() {
        let msg = Paragraph::new(Span::styled(
            "  (no ports found)",
            Style::default().fg(Color::DarkGray),
        ))
        .block(block);
        frame.render_widget(msg, area);
        return;
    }

    let items: Vec<ListItem> = ports
        .iter()
        .enumerate()
        .map(|(i, name)| {
            let is_selected = i == cursor;
            let style = if is_selected && is_active {
                Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD)
            } else if is_selected {
                Style::default()
                    .fg(Color::White)
                    .add_modifier(Modifier::BOLD)
            } else {
                Style::default().fg(Color::Gray)
            };
            let prefix = if is_selected { "> " } else { "  " };
            ListItem::new(Span::styled(format!("{prefix}{name}"), style))
        })
        .collect();

    let list = List::new(items).block(block);
    frame.render_widget(list, area);
}

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

struct KnobState {
    label: String,
    value: f64,
    range: (f64, f64),
    default: f64,
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

// ── Knob rendering ──────────────────────────────────────────────────────

const KNOB_FRAMES: [&[&str; 5]; 8] = [
    &[
        " ╭───╮ ",
        "╱     ╲",
        "│      │",
        "╲/    ╱",
        " ╰───╯ ",
    ],
    &[
        " ╭───╮ ",
        "╱     ╲",
        "│-     │",
        "╲     ╱",
        " ╰───╯ ",
    ],
    &[
        " ╭───╮ ",
        "╱     ╲",
        "│─     │",
        "╲     ╱",
        " ╰───╯ ",
    ],
    &[
        " ╭───╮ ",
        "╱/    ╲",
        "│      │",
        "╲     ╱",
        " ╰───╯ ",
    ],
    &[
        " ╭───╮ ",
        "╱  |  ╲",
        "│  |   │",
        "╲     ╱",
        " ╰───╯ ",
    ],
    &[
        " ╭───╮ ",
        "╱    ╲╲",
        "│      │",
        "╲     ╱",
        " ╰───╯ ",
    ],
    &[
        " ╭───╮ ",
        "╱     ╲",
        "│     ─│",
        "╲     ╱",
        " ╰───╯ ",
    ],
    &[
        " ╭───╮ ",
        "╱     ╲",
        "│      │",
        "╲    \\╱",
        " ╰───╯ ",
    ],
];

fn knob_frame_index(value: f64, range: (f64, f64)) -> usize {
    let norm = if (range.1 - range.0).abs() < f64::EPSILON {
        0.5
    } else {
        ((value - range.0) / (range.1 - range.0)).clamp(0.0, 1.0)
    };
    let idx = (norm * 7.0).round() as usize;
    idx.min(7)
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

    // Knob row
    let n = state.knobs.len();
    let h_constraints: Vec<Constraint> =
        (0..n).map(|_| Constraint::Ratio(1, n as u32)).collect();
    let knob_cols = Layout::horizontal(&h_constraints).split(v_chunks[2]);
    let label_cols = Layout::horizontal(&h_constraints).split(v_chunks[3]);
    let value_cols = Layout::horizontal(&h_constraints).split(v_chunks[4]);

    for (i, knob) in state.knobs.iter().enumerate() {
        let selected = i == state.selected;
        let fi = knob_frame_index(knob.value, knob.range);
        let art = KNOB_FRAMES[fi];

        let style = if selected {
            Style::default()
                .fg(Color::Yellow)
                .add_modifier(Modifier::BOLD)
        } else {
            Style::default().fg(Color::White)
        };

        let lines: Vec<Line> = art
            .iter()
            .map(|l| Line::from(Span::styled(*l, style)))
            .collect();
        let knob_widget = Paragraph::new(lines).alignment(Alignment::Center);
        frame.render_widget(knob_widget, knob_cols[i]);

        let label_text = if selected {
            format!(">>{label}<<", label = knob.label)
        } else {
            knob.label.clone()
        };
        let label =
            Paragraph::new(Span::styled(label_text, style)).alignment(Alignment::Center);
        frame.render_widget(label, label_cols[i]);

        let val_text = format!("{:.2}", knob.value);
        let val =
            Paragraph::new(Span::styled(val_text, style)).alignment(Alignment::Center);
        frame.render_widget(val, value_cols[i]);
    }

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

fn render_footswitch(frame: &mut Frame, area: Rect, bypassed: bool) {
    let (label, style) = if bypassed {
        (
            " BYPASSED ",
            Style::default()
                .fg(Color::DarkGray)
                .add_modifier(Modifier::DIM),
        )
    } else {
        (
            "  ACTIVE  ",
            Style::default()
                .fg(Color::Green)
                .add_modifier(Modifier::BOLD),
        )
    };
    let switch = Paragraph::new(Line::from(Span::styled(label, style)))
        .alignment(Alignment::Center)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_type(BorderType::Rounded),
        );
    let sw_width = 14_u16;
    let x = area.x + area.width.saturating_sub(sw_width) / 2;
    let sw_area = Rect::new(x, area.y, sw_width.min(area.width), area.height);
    frame.render_widget(switch, sw_area);
}

// ═════════════════════════════════════════════════════════════════════════════
// Main entry point
// ═════════════════════════════════════════════════════════════════════════════

pub fn run(pedal_path: &str) -> Result<(), Box<dyn std::error::Error>> {
    let source = std::fs::read_to_string(pedal_path)?;
    let pedal = parse_pedal_file(&source).map_err(|e| format!("Parse error: {e}"))?;

    // Create JACK client and enumerate available ports
    let client = AudioEngine::create_client("pedalkernel").map_err(|e| {
        format!(
            "Failed to connect to JACK server: {e}\n\n\
             Make sure JACK is running. You can start it with:\n  \
             jackd -d alsa -r 48000 -p 256\n\
             or use a JACK control app like QjackCtl / Cadence."
        )
    })?;
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
    let mut terminal = Terminal::new(backend)?;

    // Phase 1: port selection
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

    // Phase 2: pedal control with live JACK audio
    let result = run_pedal_control(
        &mut terminal,
        client,
        &pedal,
        &selected_in,
        &selected_out,
    );

    // Teardown (always runs)
    disable_raw_mode()?;
    stdout().execute(LeaveAlternateScreen)?;

    result
}
