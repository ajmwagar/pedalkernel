//! Interactive ASCII pedal control surface.
//!
//! Run: `cargo run --example tui --features tui -- examples/tube_screamer.pedal`

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tui_app::run()
}

mod tui_app {
    use crossterm::{
        event::{self, Event, KeyCode, KeyModifiers},
        terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
        ExecutableCommand,
    };
    use pedalkernel::dsl::{parse_pedal_file, ControlDef};
    use ratatui::{
        layout::{Alignment, Constraint, Layout, Rect},
        style::{Color, Modifier, Style},
        text::{Line, Span},
        widgets::{Block, Borders, Paragraph},
        Frame, Terminal,
    };
    use std::io::stdout;

    // ── Model ──────────────────────────────────────────────────────────

    struct App {
        name: String,
        knobs: Vec<KnobState>,
        selected: usize,
        bypassed: bool,
        running: bool,
    }

    struct KnobState {
        label: String,
        value: f64,
        range: (f64, f64),
        default: f64,
    }

    impl App {
        fn from_controls(name: &str, controls: &[ControlDef]) -> Self {
            let knobs = controls
                .iter()
                .map(|c| KnobState {
                    label: c.label.clone(),
                    value: c.default,
                    range: c.range,
                    default: c.default,
                })
                .collect::<Vec<_>>();
            Self {
                name: name.to_string(),
                knobs,
                selected: 0,
                bypassed: false,
                running: true,
            }
        }

        fn update(&mut self, action: AppAction) {
            match action {
                AppAction::Quit => self.running = false,
                AppAction::NextKnob => {
                    if !self.knobs.is_empty() {
                        self.selected = (self.selected + 1) % self.knobs.len();
                    }
                }
                AppAction::PrevKnob => {
                    if !self.knobs.is_empty() {
                        self.selected = (self.selected + self.knobs.len() - 1) % self.knobs.len();
                    }
                }
                AppAction::Adjust(delta) => {
                    if let Some(k) = self.knobs.get_mut(self.selected) {
                        let span = k.range.1 - k.range.0;
                        k.value = (k.value + delta * span).clamp(k.range.0, k.range.1);
                    }
                }
                AppAction::ResetKnob => {
                    if let Some(k) = self.knobs.get_mut(self.selected) {
                        k.value = k.default;
                    }
                }
                AppAction::ToggleBypass => self.bypassed = !self.bypassed,
                AppAction::None => {}
            }
        }
    }

    // ── Update ─────────────────────────────────────────────────────────

    enum AppAction {
        Quit,
        NextKnob,
        PrevKnob,
        Adjust(f64),
        ResetKnob,
        ToggleBypass,
        None,
    }

    fn handle_key(code: KeyCode, modifiers: KeyModifiers) -> AppAction {
        match code {
            KeyCode::Char('q') | KeyCode::Esc => AppAction::Quit,
            KeyCode::Tab if modifiers.contains(KeyModifiers::SHIFT) => AppAction::PrevKnob,
            KeyCode::BackTab => AppAction::PrevKnob,
            KeyCode::Tab => AppAction::NextKnob,
            KeyCode::Left => AppAction::Adjust(-0.05),
            KeyCode::Right => AppAction::Adjust(0.05),
            KeyCode::Down => AppAction::Adjust(-0.01),
            KeyCode::Up => AppAction::Adjust(0.01),
            KeyCode::Char(' ') => AppAction::ToggleBypass,
            KeyCode::Char('r') | KeyCode::Char('R') => AppAction::ResetKnob,
            _ => AppAction::None,
        }
    }

    // ── Knob rendering ─────────────────────────────────────────────────

    /// 8 indicator positions mapping normalized 0..1 to 7-o'clock → 5-o'clock.
    ///
    /// Each position is a set of 5 lines (the knob body), 7 chars wide.
    /// The indicator mark varies per position.
    const KNOB_FRAMES: [&[&str; 5]; 8] = [
        // 0: 7-o'clock  (value ≈ 0.00)
        &[" ╭───╮ ", "╱     ╲", "│      │", "╲/    ╱", " ╰───╯ "],
        // 1: 8-o'clock  (value ≈ 0.14)
        &[" ╭───╮ ", "╱     ╲", "│-     │", "╲     ╱", " ╰───╯ "],
        // 2: 9-o'clock  (value ≈ 0.29)
        &[" ╭───╮ ", "╱     ╲", "│─     │", "╲     ╱", " ╰───╯ "],
        // 3: 10-o'clock (value ≈ 0.43)
        &[" ╭───╮ ", "╱/    ╲", "│      │", "╲     ╱", " ╰───╯ "],
        // 4: 12-o'clock (value ≈ 0.57)
        &[" ╭───╮ ", "╱  |  ╲", "│  |   │", "╲     ╱", " ╰───╯ "],
        // 5: 2-o'clock  (value ≈ 0.71)
        &[" ╭───╮ ", "╱    ╲╲", "│      │", "╲     ╱", " ╰───╯ "],
        // 6: 3-o'clock  (value ≈ 0.86)
        &[" ╭───╮ ", "╱     ╲", "│     ─│", "╲     ╱", " ╰───╯ "],
        // 7: 5-o'clock  (value ≈ 1.00)
        &[" ╭───╮ ", "╱     ╲", "│      │", "╲    \\╱", " ╰───╯ "],
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

    // ── View ───────────────────────────────────────────────────────────

    fn draw(frame: &mut Frame, app: &App) {
        let area = frame.area();

        // Terminal-too-small guard
        if area.width < 30 || area.height < 14 {
            let msg = Paragraph::new("Terminal too small!\nResize to at least 30×14.")
                .alignment(Alignment::Center);
            frame.render_widget(msg, area);
            return;
        }

        // Outer pedal border
        let outer = Block::default()
            .title(format!("  {}  ", app.name.to_uppercase()))
            .title_alignment(Alignment::Center)
            .borders(Borders::ALL)
            .border_type(ratatui::widgets::BorderType::Double)
            .style(Style::default().fg(Color::White));
        let inner = outer.inner(area);
        frame.render_widget(outer, area);

        if app.knobs.is_empty() {
            let msg =
                Paragraph::new("No controls defined for this pedal.").alignment(Alignment::Center);
            frame.render_widget(msg, inner);
            return;
        }

        // Vertical layout: padding | knobs | labels | values | gap | footswitch | help
        let v_chunks = Layout::vertical([
            Constraint::Length(1), // top padding
            Constraint::Length(5), // knob art
            Constraint::Length(1), // labels
            Constraint::Length(1), // values
            Constraint::Length(1), // gap
            Constraint::Length(3), // footswitch
            Constraint::Length(1), // help
        ])
        .split(inner);

        // ── Knob row ───────────────────────────────────────────────
        let n = app.knobs.len();
        let h_constraints: Vec<Constraint> =
            (0..n).map(|_| Constraint::Ratio(1, n as u32)).collect();
        let knob_cols = Layout::horizontal(&h_constraints).split(v_chunks[1]);
        let label_cols = Layout::horizontal(&h_constraints).split(v_chunks[2]);
        let value_cols = Layout::horizontal(&h_constraints).split(v_chunks[3]);

        for (i, knob) in app.knobs.iter().enumerate() {
            let selected = i == app.selected;
            let fi = knob_frame_index(knob.value, knob.range);
            let art = KNOB_FRAMES[fi];

            let style = if selected {
                Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD)
            } else {
                Style::default().fg(Color::White)
            };

            // Knob art
            let lines: Vec<Line> = art
                .iter()
                .map(|l| Line::from(Span::styled(*l, style)))
                .collect();
            let knob_widget = Paragraph::new(lines).alignment(Alignment::Center);
            frame.render_widget(knob_widget, knob_cols[i]);

            // Label
            let label_text = if selected {
                format!(">>{label}<<", label = knob.label)
            } else {
                knob.label.clone()
            };
            let label =
                Paragraph::new(Span::styled(label_text, style)).alignment(Alignment::Center);
            frame.render_widget(label, label_cols[i]);

            // Value
            let val_text = format!("{:.2}", knob.value);
            let val = Paragraph::new(Span::styled(val_text, style)).alignment(Alignment::Center);
            frame.render_widget(val, value_cols[i]);
        }

        // ── Footswitch ─────────────────────────────────────────────
        render_footswitch(frame, v_chunks[5], app.bypassed);

        // ── Help bar ───────────────────────────────────────────────
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
        frame.render_widget(help, v_chunks[6]);
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
                    .border_type(ratatui::widgets::BorderType::Rounded),
            );
        // Center the footswitch box (12 chars wide)
        let sw_width = 14_u16;
        let x = area.x + area.width.saturating_sub(sw_width) / 2;
        let sw_area = Rect::new(x, area.y, sw_width.min(area.width), area.height);
        frame.render_widget(switch, sw_area);
    }

    // ── Main event loop ────────────────────────────────────────────────

    pub fn run() -> Result<(), Box<dyn std::error::Error>> {
        let args: Vec<String> = std::env::args().collect();
        if args.len() < 2 {
            eprintln!("Usage: cargo run --example tui --features tui -- <file.pedal>");
            std::process::exit(1);
        }

        let source = std::fs::read_to_string(&args[1])?;
        let pedal = parse_pedal_file(&source).map_err(|e| format!("Parse error: {e}"))?;

        let mut app = App::from_controls(&pedal.name, &pedal.controls);

        // Terminal setup
        enable_raw_mode()?;
        stdout().execute(EnterAlternateScreen)?;
        let backend = ratatui::backend::CrosstermBackend::new(stdout());
        let mut terminal = Terminal::new(backend)?;

        // Event loop
        while app.running {
            terminal.draw(|f| draw(f, &app))?;

            if event::poll(std::time::Duration::from_millis(50))? {
                if let Event::Key(key) = event::read()? {
                    let action = handle_key(key.code, key.modifiers);
                    app.update(action);
                }
            }
        }

        // Teardown
        disable_raw_mode()?;
        stdout().execute(LeaveAlternateScreen)?;
        Ok(())
    }
}
