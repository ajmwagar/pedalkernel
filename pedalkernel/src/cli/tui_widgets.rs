//! Shared TUI widgets for both single-pedal and pedalboard UIs.
//!
//! Extracted from `tui.rs` — port selection, knob rendering, footswitch.

use crossterm::event::{self, Event, KeyCode};
use ratatui::{
    layout::{Alignment, Constraint, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, BorderType, Borders, List, ListItem, Paragraph},
    Frame, Terminal,
};

/// Terminal type alias used across TUI modules.
pub type Term = Terminal<ratatui::backend::CrosstermBackend<std::io::Stdout>>;

// ═════════════════════════════════════════════════════════════════════════════
// Knob state and rendering
// ═════════════════════════════════════════════════════════════════════════════

/// State for a single control knob.
pub struct KnobState {
    pub label: String,
    pub value: f64,
    pub range: (f64, f64),
    pub default: f64,
}

pub const KNOB_FRAMES: [&[&str; 5]; 8] = [
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

pub fn knob_frame_index(value: f64, range: (f64, f64)) -> usize {
    let norm = if (range.1 - range.0).abs() < f64::EPSILON {
        0.5
    } else {
        ((value - range.0) / (range.1 - range.0)).clamp(0.0, 1.0)
    };
    let idx = (norm * 7.0).round() as usize;
    idx.min(7)
}

/// Render a row of knobs into the given areas.
pub fn render_knob_row(
    frame: &mut Frame,
    knobs: &[KnobState],
    selected: usize,
    knob_area: Rect,
    label_area: Rect,
    value_area: Rect,
) {
    let n = knobs.len();
    if n == 0 {
        return;
    }
    let h_constraints: Vec<Constraint> =
        (0..n).map(|_| Constraint::Ratio(1, n as u32)).collect();
    let knob_cols = Layout::horizontal(&h_constraints).split(knob_area);
    let label_cols = Layout::horizontal(&h_constraints).split(label_area);
    let value_cols = Layout::horizontal(&h_constraints).split(value_area);

    for (i, knob) in knobs.iter().enumerate() {
        let is_selected = i == selected;
        let fi = knob_frame_index(knob.value, knob.range);
        let art = KNOB_FRAMES[fi];

        let style = if is_selected {
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

        let label_text = if is_selected {
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
}

// ═════════════════════════════════════════════════════════════════════════════
// Footswitch
// ═════════════════════════════════════════════════════════════════════════════

pub fn render_footswitch(frame: &mut Frame, area: Rect, bypassed: bool) {
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
// Port selection
// ═════════════════════════════════════════════════════════════════════════════

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum PortPanel {
    Input,
    Output,
}

pub struct PortSelectState {
    pub input_ports: Vec<String>,
    pub output_ports: Vec<String>,
    pub input_cursor: usize,
    pub output_cursor: usize,
    pub active_panel: PortPanel,
}

impl PortSelectState {
    pub fn new(input_ports: Vec<String>, output_ports: Vec<String>) -> Self {
        Self {
            input_ports,
            output_ports,
            input_cursor: 0,
            output_cursor: 0,
            active_panel: PortPanel::Input,
        }
    }

    pub fn selected_input(&self) -> Option<&str> {
        self.input_ports.get(self.input_cursor).map(|s| s.as_str())
    }

    pub fn selected_output(&self) -> Option<&str> {
        self.output_ports
            .get(self.output_cursor)
            .map(|s| s.as_str())
    }

    pub fn cursor_up(&mut self) {
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

    pub fn cursor_down(&mut self) {
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

    pub fn toggle_panel(&mut self) {
        self.active_panel = match self.active_panel {
            PortPanel::Input => PortPanel::Output,
            PortPanel::Output => PortPanel::Input,
        };
    }
}

/// Result of the port selection screen.
pub enum PortSelectResult {
    Selected { input: String, output: String },
    Quit,
}

/// Run the port selection screen. Returns the selected ports or Quit.
pub fn run_port_select(
    terminal: &mut Term,
    title: &str,
    input_ports: Vec<String>,
    output_ports: Vec<String>,
) -> Result<PortSelectResult, Box<dyn std::error::Error>> {
    let mut state = PortSelectState::new(input_ports, output_ports);

    loop {
        terminal.draw(|f| draw_port_select(f, &state, title))?;

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

pub fn draw_port_select(frame: &mut Frame, state: &PortSelectState, title: &str) {
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
            title.to_uppercase()
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

/// Result of the output-only selection screen.
pub enum OutputSelectResult {
    Selected(String),
    Quit,
}

/// Run an output-only port selection screen (for WAV input mode).
pub fn run_output_select(
    terminal: &mut Term,
    title: &str,
    output_ports: Vec<String>,
) -> Result<OutputSelectResult, Box<dyn std::error::Error>> {
    let mut cursor = 0_usize;

    loop {
        terminal.draw(|f| {
            draw_output_select(f, title, &output_ports, cursor);
        })?;

        if event::poll(std::time::Duration::from_millis(50))? {
            if let Event::Key(key) = event::read()? {
                match key.code {
                    KeyCode::Char('q') | KeyCode::Esc => return Ok(OutputSelectResult::Quit),
                    KeyCode::Up => {
                        if cursor > 0 {
                            cursor -= 1;
                        }
                    }
                    KeyCode::Down => {
                        if cursor + 1 < output_ports.len() {
                            cursor += 1;
                        }
                    }
                    KeyCode::Enter => {
                        if let Some(port) = output_ports.get(cursor) {
                            return Ok(OutputSelectResult::Selected(port.clone()));
                        }
                    }
                    _ => {}
                }
            }
        }
    }
}

fn draw_output_select(frame: &mut Frame, title: &str, ports: &[String], cursor: usize) {
    let area = frame.area();

    if area.width < 40 || area.height < 10 {
        let msg = Paragraph::new("Terminal too small!\nResize to at least 40x10.")
            .alignment(Alignment::Center);
        frame.render_widget(msg, area);
        return;
    }

    let outer = Block::default()
        .title(format!(
            "  {} — Output Port Selection  ",
            title.to_uppercase()
        ))
        .title_alignment(Alignment::Center)
        .borders(Borders::ALL)
        .border_type(BorderType::Double)
        .style(Style::default().fg(Color::White));
    let inner = outer.inner(area);
    frame.render_widget(outer, area);

    let v_chunks = Layout::vertical([
        Constraint::Length(1), // top padding
        Constraint::Length(1), // column header
        Constraint::Min(3),    // port list
        Constraint::Length(1), // gap
        Constraint::Length(1), // help bar
    ])
    .split(inner);

    let header_style = Style::default()
        .fg(Color::Yellow)
        .add_modifier(Modifier::BOLD);
    frame.render_widget(
        Paragraph::new(Span::styled(" Output (Destination)", header_style)),
        v_chunks[1],
    );

    draw_port_list(frame, v_chunks[2], ports, cursor, true);

    let help = Paragraph::new(Line::from(vec![
        Span::styled("↑↓", Style::default().fg(Color::Cyan)),
        Span::raw(" navigate  "),
        Span::styled("Enter", Style::default().fg(Color::Cyan)),
        Span::raw(" confirm  "),
        Span::styled("Q", Style::default().fg(Color::Cyan)),
        Span::raw(" quit"),
    ]))
    .alignment(Alignment::Center);
    frame.render_widget(help, v_chunks[4]);
}

pub fn draw_port_list(
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
