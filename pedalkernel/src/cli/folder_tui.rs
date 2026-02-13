//! Folder TUI — discover all `.board` files in a directory, pre-compile them,
//! and allow seamless switching between entire pedalboards during live JACK audio.

use crossterm::{
    event::{self, Event, KeyCode, KeyModifiers},
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
    ExecutableCommand,
};
use pedalkernel::board::parse_board_file;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::pedalboard::{BoardSwitcherProcessor, PedalboardProcessor};
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

use super::board_tui::{handle_board_key, render_io_bar, BoardAction, PedalPanel, ViewMode};
use super::tui_widgets::{
    run_output_select, run_port_select, KnobState, OutputSelectResult, PortSelectResult, Term,
};

// ═════════════════════════════════════════════════════════════════════════════
// State
// ═════════════════════════════════════════════════════════════════════════════

struct BoardState {
    name: String,
    pedals: Vec<PedalPanel>,
    focused_pedal: usize,
    focused_knob: usize,
}

struct FolderControlState {
    boards: Vec<BoardState>,
    active_board: usize,
    board_bypassed: bool,
    view_mode: ViewMode,
    controls: Arc<SharedControls>,
    input_port: String,
    output_port: String,
}

impl FolderControlState {
    fn switch_board(&mut self, new_idx: usize) {
        if new_idx < self.boards.len() && new_idx != self.active_board {
            self.active_board = new_idx;
            // Tell the audio engine to switch
            self.controls
                .set_control("__switch_board__", new_idx as f64);
            // Re-apply all knob values for the new board so the processor
            // state matches the TUI display
            let board = &self.boards[self.active_board];
            for (pidx, panel) in board.pedals.iter().enumerate() {
                for knob in &panel.knobs {
                    self.controls
                        .set_control(&format!("{pidx}:{}", knob.label), knob.value);
                }
                let bypass_val = if panel.bypassed { 1.0 } else { 0.0 };
                self.controls
                    .set_control(&format!("{pidx}:__bypass__"), bypass_val);
            }
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// Drawing
// ═════════════════════════════════════════════════════════════════════════════

fn draw_folder_control(frame: &mut Frame, state: &FolderControlState) {
    let area = frame.area();

    if area.width < 40 || area.height < 16 {
        let msg = Paragraph::new("Terminal too small!\nResize to at least 40x16.")
            .alignment(Alignment::Center);
        frame.render_widget(msg, area);
        return;
    }

    // Top-level layout: board selector bar + board content
    let v_chunks = Layout::vertical([
        Constraint::Length(1), // board selector bar
        Constraint::Min(14),   // board content (delegated to board_tui drawing)
    ])
    .split(area);

    // ── Board selector bar ──
    render_board_selector(frame, v_chunks[0], &state.boards, state.active_board);

    // ── Board content — reuse board_tui drawing ──
    let board = &state.boards[state.active_board];

    let title = if state.board_bypassed {
        format!("  {} [BYPASSED]  ", board.name.to_uppercase())
    } else {
        format!("  {}  ", board.name.to_uppercase())
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
    let inner = outer.inner(v_chunks[1]);
    frame.render_widget(outer, v_chunks[1]);

    if board.pedals.is_empty() {
        let msg = Paragraph::new("No pedals in this board.").alignment(Alignment::Center);
        frame.render_widget(msg, inner);
        return;
    }

    draw_board_content(frame, state, inner);
}

/// Draw the board content area (overview or detail) for the active board.
fn draw_board_content(frame: &mut Frame, state: &FolderControlState, inner: Rect) {
    let board = &state.boards[state.active_board];

    match state.view_mode {
        ViewMode::Overview => draw_folder_overview(frame, state, board, inner),
        ViewMode::Detail => draw_folder_detail(frame, state, board, inner),
    }
}

fn draw_folder_overview(
    frame: &mut Frame,
    state: &FolderControlState,
    board: &BoardState,
    inner: Rect,
) {
    use super::board_tui::render_mini_pedal;

    let v_chunks = Layout::vertical([
        Constraint::Length(1), // I/O info
        Constraint::Length(1), // padding
        Constraint::Min(12),   // pedal cards
        Constraint::Length(1), // help
    ])
    .split(inner);

    render_io_bar(frame, v_chunks[0], &state.input_port, &state.output_port);

    let n = board.pedals.len();
    if n > 0 {
        let h_constraints: Vec<Constraint> =
            (0..n).map(|_| Constraint::Ratio(1, n as u32)).collect();
        let card_cols = Layout::horizontal(&h_constraints).split(v_chunks[2]);

        for (i, panel) in board.pedals.iter().enumerate() {
            let is_focused = i == board.focused_pedal;
            render_mini_pedal(frame, card_cols[i], panel, is_focused);
        }
    }

    // Help bar with { } hint
    let help = Paragraph::new(Line::from(vec![
        Span::styled("{ }", Style::default().fg(Color::Cyan)),
        Span::raw(" board  "),
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

fn draw_folder_detail(
    frame: &mut Frame,
    state: &FolderControlState,
    board: &BoardState,
    inner: Rect,
) {
    use super::board_tui::{render_chain_bar, render_status_row};
    use super::tui_widgets::{render_footswitch, render_knob_row};

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

    render_io_bar(frame, v_chunks[0], &state.input_port, &state.output_port);
    render_chain_bar(frame, v_chunks[2], &board.pedals, board.focused_pedal);
    render_status_row(frame, v_chunks[3], &board.pedals);

    if let Some(panel) = board.pedals.get(board.focused_pedal) {
        let header_style = Style::default()
            .fg(Color::Yellow)
            .add_modifier(Modifier::BOLD);
        let header = Paragraph::new(Line::from(Span::styled(
            format!("── {} ──", panel.name.to_uppercase()),
            header_style,
        )))
        .alignment(Alignment::Center);
        frame.render_widget(header, v_chunks[5]);

        if !panel.knobs.is_empty() {
            render_knob_row(
                frame,
                &panel.knobs,
                board.focused_knob,
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

        render_footswitch(frame, v_chunks[11], panel.bypassed);
    }

    // Help bar with { } hint
    let help = Paragraph::new(Line::from(vec![
        Span::styled("{ }", Style::default().fg(Color::Cyan)),
        Span::raw(" board  "),
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

/// Render the board selector bar: `{ BLUES RIG } → [ GRUNGE BOARD ]`
fn render_board_selector(frame: &mut Frame, area: Rect, boards: &[BoardState], active: usize) {
    let mut spans: Vec<Span> = Vec::new();
    for (i, board) in boards.iter().enumerate() {
        if i > 0 {
            spans.push(Span::styled(" → ", Style::default().fg(Color::DarkGray)));
        }
        let name = board.name.to_uppercase();
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
// Input handling
// ═════════════════════════════════════════════════════════════════════════════

/// Handle input for the folder TUI. Returns false when the user wants to quit.
fn handle_folder_input(
    state: &mut FolderControlState,
    code: KeyCode,
    modifiers: KeyModifiers,
) -> bool {
    // Board switching: { and } (Shift+[ and Shift+])
    match code {
        KeyCode::Char('{') => {
            if state.boards.len() > 1 {
                let new_idx = (state.active_board + state.boards.len() - 1) % state.boards.len();
                state.switch_board(new_idx);
            }
            return true;
        }
        KeyCode::Char('}') => {
            if state.boards.len() > 1 {
                let new_idx = (state.active_board + 1) % state.boards.len();
                state.switch_board(new_idx);
            }
            return true;
        }
        _ => {}
    }

    // Delegate to board_tui key handler for all other keys
    let action = handle_board_key(code, modifiers);

    // Apply the action to the active board's state
    let board = &mut state.boards[state.active_board];

    match action {
        BoardAction::Quit => return false,
        BoardAction::NextPedal => {
            if !board.pedals.is_empty() {
                board.focused_pedal = (board.focused_pedal + 1) % board.pedals.len();
                board.focused_knob = 0;
            }
        }
        BoardAction::PrevPedal => {
            if !board.pedals.is_empty() {
                board.focused_pedal =
                    (board.focused_pedal + board.pedals.len() - 1) % board.pedals.len();
                board.focused_knob = 0;
            }
        }
        BoardAction::NextKnob => {
            if state.view_mode == ViewMode::Detail {
                if let Some(panel) = board.pedals.get(board.focused_pedal) {
                    if !panel.knobs.is_empty() {
                        board.focused_knob = (board.focused_knob + 1) % panel.knobs.len();
                    }
                }
            }
        }
        BoardAction::PrevKnob => {
            if state.view_mode == ViewMode::Detail {
                if let Some(panel) = board.pedals.get(board.focused_pedal) {
                    if !panel.knobs.is_empty() {
                        board.focused_knob =
                            (board.focused_knob + panel.knobs.len() - 1) % panel.knobs.len();
                    }
                }
            }
        }
        BoardAction::Adjust(delta) => {
            if state.view_mode == ViewMode::Overview {
                if !board.pedals.is_empty() {
                    if delta < 0.0 {
                        board.focused_pedal =
                            (board.focused_pedal + board.pedals.len() - 1) % board.pedals.len();
                    } else {
                        board.focused_pedal = (board.focused_pedal + 1) % board.pedals.len();
                    }
                    board.focused_knob = 0;
                }
            } else {
                let pidx = board.focused_pedal;
                if let Some(panel) = board.pedals.get_mut(pidx) {
                    if let Some(k) = panel.knobs.get_mut(board.focused_knob) {
                        let span = k.range.1 - k.range.0;
                        k.value = (k.value + delta * span).clamp(k.range.0, k.range.1);
                        state
                            .controls
                            .set_control(&format!("{pidx}:{}", k.label), k.value);
                    }
                }
            }
        }
        BoardAction::ResetKnob => {
            if state.view_mode == ViewMode::Detail {
                let pidx = board.focused_pedal;
                if let Some(panel) = board.pedals.get_mut(pidx) {
                    if let Some(k) = panel.knobs.get_mut(board.focused_knob) {
                        k.value = k.default;
                        state
                            .controls
                            .set_control(&format!("{pidx}:{}", k.label), k.value);
                    }
                }
            }
        }
        BoardAction::ToggleBypass => {
            let pidx = board.focused_pedal;
            if let Some(panel) = board.pedals.get_mut(pidx) {
                panel.bypassed = !panel.bypassed;
                let val = if panel.bypassed { 1.0 } else { 0.0 };
                state
                    .controls
                    .set_control(&format!("{pidx}:__bypass__"), val);
            }
        }
        BoardAction::ToggleBoardBypass => {
            state.board_bypassed = !state.board_bypassed;
            state.controls.set_bypassed(state.board_bypassed);
        }
        BoardAction::ToggleView => {
            state.view_mode = match state.view_mode {
                ViewMode::Overview => ViewMode::Detail,
                ViewMode::Detail => ViewMode::Overview,
            };
        }
        BoardAction::EnterDetail => {
            if state.view_mode == ViewMode::Overview {
                state.view_mode = ViewMode::Detail;
            }
        }
        BoardAction::None => {}
    }
    true
}

// ═════════════════════════════════════════════════════════════════════════════
// Board discovery and compilation
// ═════════════════════════════════════════════════════════════════════════════

struct DiscoveredBoard {
    name: String,
    panels: Vec<PedalPanel>,
    processor: PedalboardProcessor,
    warnings: Vec<String>,
}

fn discover_boards(
    dir_path: &str,
    sample_rate: f64,
) -> Result<Vec<DiscoveredBoard>, Box<dyn std::error::Error>> {
    let dir = Path::new(dir_path);
    if !dir.is_dir() {
        return Err(format!("'{}' is not a directory", dir_path).into());
    }

    let mut board_files: Vec<_> = std::fs::read_dir(dir)?
        .filter_map(|entry| {
            let entry = entry.ok()?;
            let path = entry.path();
            if path.extension().and_then(|e| e.to_str()) == Some("board") {
                Some(path)
            } else {
                None
            }
        })
        .collect();

    board_files.sort();

    if board_files.is_empty() {
        return Err(format!("No .board files found in '{}'", dir_path).into());
    }

    let mut discovered = Vec::with_capacity(board_files.len());

    for board_path in &board_files {
        // Read the board file — if we can't even read it, warn and skip
        let source = match std::fs::read_to_string(board_path) {
            Ok(s) => s,
            Err(e) => {
                eprintln!(
                    "WARNING: Skipping {} — cannot read: {e}",
                    board_path.display()
                );
                continue;
            }
        };

        // Parse the board definition — if it fails, warn and skip
        let board_def = match parse_board_file(&source) {
            Ok(b) => b,
            Err(e) => {
                eprintln!(
                    "WARNING: Skipping {} — parse error: {e}",
                    board_path.display()
                );
                continue;
            }
        };

        let board_dir = board_path.parent().unwrap_or_else(|| Path::new("."));

        // Build the processor — individual pedal failures produce warnings
        // and dry passthrough slots (never returns Err)
        let (processor, warnings) =
            PedalboardProcessor::from_board_with_warnings(&board_def, board_dir, sample_rate);

        // Build pedal panels for the TUI, matching processor slots.
        // If a pedal source can't be read/parsed, create a failed panel
        // (no knobs) to match the dry passthrough slot in the processor.
        let mut panels = Vec::new();
        for entry in &board_def.pedals {
            let pedal_path = board_dir.join(&entry.path);

            let panel = match std::fs::read_to_string(&pedal_path)
                .map_err(|e| e.to_string())
                .and_then(|src| parse_pedal_file(&src).map_err(|e| e.to_string()))
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
                Err(_) => {
                    // Failed pedal — no knobs, just a label
                    PedalPanel {
                        name: format!("{} [FAILED]", entry.id),
                        knobs: Vec::new(),
                        bypassed: false,
                    }
                }
            };

            panels.push(panel);
        }

        discovered.push(DiscoveredBoard {
            name: board_def.name,
            panels,
            processor,
            warnings,
        });
    }

    Ok(discovered)
}

// ═════════════════════════════════════════════════════════════════════════════
// Control loop
// ═════════════════════════════════════════════════════════════════════════════

fn run_folder_control(
    terminal: &mut Term,
    client: jack::Client,
    switcher: BoardSwitcherProcessor,
    boards: Vec<BoardState>,
    connect_from: &str,
    connect_to: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let controls = Arc::new(SharedControls::new());
    let (async_client, our_in, our_out) = AudioEngine::start(client, switcher, controls.clone())?;

    async_client
        .as_client()
        .connect_ports_by_name(connect_from, &our_in)?;
    async_client
        .as_client()
        .connect_ports_by_name(&our_out, connect_to)?;

    let mut state = FolderControlState {
        boards,
        active_board: 0,
        board_bypassed: false,
        view_mode: ViewMode::Overview,
        controls,
        input_port: connect_from.to_string(),
        output_port: connect_to.to_string(),
    };

    loop {
        terminal.draw(|f| draw_folder_control(f, &state))?;

        if event::poll(std::time::Duration::from_millis(50))? {
            if let Event::Key(key) = event::read()? {
                if !handle_folder_input(&mut state, key.code, key.modifiers) {
                    break;
                }
            }
        }
    }

    drop(async_client);
    Ok(())
}

fn run_folder_control_wav(
    terminal: &mut Term,
    client: jack::Client,
    switcher: WavLoopProcessor<BoardSwitcherProcessor>,
    boards: Vec<BoardState>,
    input_label: &str,
    connect_to: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let controls = Arc::new(SharedControls::new());
    let (async_client, _our_in, our_out) = AudioEngine::start(client, switcher, controls.clone())?;

    // Only connect output port
    async_client
        .as_client()
        .connect_ports_by_name(&our_out, connect_to)?;

    let mut state = FolderControlState {
        boards,
        active_board: 0,
        board_bypassed: false,
        view_mode: ViewMode::Overview,
        controls,
        input_port: input_label.to_string(),
        output_port: connect_to.to_string(),
    };

    loop {
        terminal.draw(|f| draw_folder_control(f, &state))?;

        if event::poll(std::time::Duration::from_millis(50))? {
            if let Event::Key(key) = event::read()? {
                if !handle_folder_input(&mut state, key.code, key.modifiers) {
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

pub fn run(dir_path: &str, wav_input: Option<&str>) -> Result<(), Box<dyn std::error::Error>> {
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

    // Discover and pre-compile all boards
    let discovered = discover_boards(dir_path, sample_rate)?;

    if discovered.is_empty() {
        return Err(format!(
            "No valid .board files found in '{}' (all failed to load)",
            dir_path
        )
        .into());
    }

    eprintln!(
        "Found {} board(s): {}",
        discovered.len(),
        discovered
            .iter()
            .map(|b| b.name.as_str())
            .collect::<Vec<_>>()
            .join(", ")
    );

    // Split into processors and TUI state, printing any per-board warnings
    let mut processors = Vec::with_capacity(discovered.len());
    let mut board_states = Vec::with_capacity(discovered.len());

    for db in discovered {
        for w in &db.warnings {
            eprintln!("WARNING [{}]: {w}", db.name);
        }
        processors.push(db.processor);
        board_states.push(BoardState {
            name: db.name,
            pedals: db.panels,
            focused_pedal: 0,
            focused_knob: 0,
        });
    }

    let switcher = BoardSwitcherProcessor::new(processors);

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

    let title = format!("Folder: {} ({} boards)", dir_path, board_states.len());

    let result = if let Some((samples, _, ref wav_path)) = wav_data {
        // WAV input mode: output-only port selection
        let selected_out = match run_output_select(&mut terminal, &title, output_ports)? {
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

        let wav_switcher = WavLoopProcessor::new(samples, switcher);

        run_folder_control_wav(
            &mut terminal,
            client,
            wav_switcher,
            board_states,
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
            match run_port_select(&mut terminal, &title, input_ports, output_ports)? {
                PortSelectResult::Selected { input, output } => (input, output),
                PortSelectResult::Quit => {
                    disable_raw_mode()?;
                    stdout().execute(LeaveAlternateScreen)?;
                    return Ok(());
                }
            };

        run_folder_control(
            &mut terminal,
            client,
            switcher,
            board_states,
            &selected_in,
            &selected_out,
        )
    };

    // Teardown
    disable_raw_mode()?;
    stdout().execute(LeaveAlternateScreen)?;

    result
}
