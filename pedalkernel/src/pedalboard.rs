//! Pedalboard processor — chains multiple compiled pedals in series.
//!
//! Implements [`PedalProcessor`] so it can be used directly with
//! [`AudioEngine::start()`] — zero changes to the JACK engine.
//!
//! Control routing uses a `"idx:label"` prefix scheme:
//! - `"0:Drive"` → pedal index 0, knob "Drive"
//! - `"2:__bypass__"` → toggle bypass on pedal index 2

use crate::board::{BoardDef, BoardPedalEntry};
use crate::compiler::{compile_pedal, CompiledPedal};
use crate::dsl::parse_pedal_file;
use crate::PedalProcessor;
use std::path::Path;

/// A single slot in the pedalboard chain.
///
/// When `processor` is `None`, the slot acts as dry passthrough — this
/// happens when a pedal fails to load or compile. A warning is emitted
/// at build time and the slot is marked as failed.
struct PedalSlot {
    name: String,
    processor: Option<CompiledPedal>,
    bypassed: bool,
    failed: bool,
    /// Whether this is a utility pedal (noise gate, buffer, etc.).
    /// Utility pedals are auto-inserted at the start/end of the chain
    /// and are not user-addressable by index in the main chain.
    #[allow(dead_code)]
    utility: bool,
}

/// Chains N `CompiledPedal` instances in series.
///
/// The internal chain is: `[utility_start...] [user pedals...] [utility_end...]`
///
/// User-facing indices (in `set_control("idx:Label", ...)`) address only the
/// user pedals, not the utility slots. Utility pedals can be controlled via
/// `"start:idx:Label"` or `"end:idx:Label"`.
pub struct PedalboardProcessor {
    /// Utility pedals at the start of the chain.
    utility_start: Vec<PedalSlot>,
    /// User-defined pedals in the chain.
    pedals: Vec<PedalSlot>,
    /// Utility pedals at the end of the chain.
    utility_end: Vec<PedalSlot>,
}

impl PedalboardProcessor {
    /// Build a pedalboard from a parsed `.board` definition.
    ///
    /// Each `.pedal` file is read relative to `board_dir`, compiled at the
    /// given sample rate, and any knob overrides from the board file are applied.
    ///
    /// If a pedal fails to load or compile, it is replaced with a dry
    /// passthrough slot and a warning is emitted to stderr. The board
    /// still loads — only the broken pedal is silent.
    pub fn from_board(
        board: &BoardDef,
        board_dir: &Path,
        sample_rate: f64,
    ) -> Result<Self, String> {
        let (pb, warnings) = Self::from_board_with_warnings(board, board_dir, sample_rate);
        for w in &warnings {
            eprintln!("WARNING: {w}");
        }
        Ok(pb)
    }

    /// Build a pedalboard, returning both the processor and any warnings.
    ///
    /// Pedals that fail to read, parse, or compile are replaced with dry
    /// passthrough slots. The returned warnings describe each failure.
    pub fn from_board_with_warnings(
        board: &BoardDef,
        board_dir: &Path,
        sample_rate: f64,
    ) -> (Self, Vec<String>) {
        let mut warnings = Vec::new();

        let utility_start = Self::compile_entries(
            &board.utility_start,
            board_dir,
            sample_rate,
            true,
            &mut warnings,
        );
        let pedals =
            Self::compile_entries(&board.pedals, board_dir, sample_rate, false, &mut warnings);
        let utility_end = Self::compile_entries(
            &board.utility_end,
            board_dir,
            sample_rate,
            true,
            &mut warnings,
        );

        (
            Self {
                utility_start,
                pedals,
                utility_end,
            },
            warnings,
        )
    }

    /// Compile a list of board pedal entries into slots.
    fn compile_entries(
        entries: &[BoardPedalEntry],
        board_dir: &Path,
        sample_rate: f64,
        utility: bool,
        warnings: &mut Vec<String>,
    ) -> Vec<PedalSlot> {
        let mut slots = Vec::with_capacity(entries.len());

        for entry in entries {
            let pedal_path = board_dir.join(&entry.path);

            let result: Result<(String, CompiledPedal), String> = (|| {
                let source = std::fs::read_to_string(&pedal_path).map_err(|e| {
                    format!(
                        "Error reading pedal '{}' at {}: {e}",
                        entry.id,
                        pedal_path.display()
                    )
                })?;
                let pedal_def = parse_pedal_file(&source)
                    .map_err(|e| format!("Parse error in '{}' ({}): {e}", entry.id, entry.path))?;
                let mut proc = compile_pedal(&pedal_def, sample_rate).map_err(|e| {
                    format!("Compilation error in '{}' ({}): {e}", entry.id, entry.path)
                })?;

                // Apply default control values from the .pedal file
                for ctrl in &pedal_def.controls {
                    proc.set_control(&ctrl.label, ctrl.default);
                }

                // Apply overrides from the .board file
                for (label, val) in &entry.overrides {
                    proc.set_control(label, *val);
                }

                Ok((pedal_def.name.clone(), proc))
            })();

            match result {
                Ok((name, proc)) => {
                    slots.push(PedalSlot {
                        name,
                        processor: Some(proc),
                        bypassed: false,
                        failed: false,
                        utility,
                    });
                }
                Err(msg) => {
                    warnings.push(format!(
                        "Pedal '{}' failed — using dry passthrough: {msg}",
                        entry.id
                    ));
                    slots.push(PedalSlot {
                        name: format!("{} [FAILED]", entry.id),
                        processor: None,
                        bypassed: false,
                        failed: true,
                        utility,
                    });
                }
            }
        }

        slots
    }

    /// Number of user pedals in the chain (excludes utilities).
    pub fn len(&self) -> usize {
        self.pedals.len()
    }

    /// Whether the pedalboard has no user pedals.
    pub fn is_empty(&self) -> bool {
        self.pedals.is_empty()
    }

    /// Get the name of a user pedal at the given index.
    pub fn pedal_name(&self, index: usize) -> Option<&str> {
        self.pedals.get(index).map(|s| s.name.as_str())
    }

    /// Check if a user pedal is bypassed.
    pub fn is_pedal_bypassed(&self, index: usize) -> bool {
        self.pedals.get(index).is_some_and(|s| s.bypassed)
    }

    /// Check if a pedal slot failed to compile (dry passthrough).
    pub fn is_pedal_failed(&self, index: usize) -> bool {
        self.pedals.get(index).is_some_and(|s| s.failed)
    }

    /// Number of utility pedals at the start of the chain.
    pub fn utility_start_len(&self) -> usize {
        self.utility_start.len()
    }

    /// Number of utility pedals at the end of the chain.
    pub fn utility_end_len(&self) -> usize {
        self.utility_end.len()
    }

    /// Get the name of a start-of-chain utility pedal.
    pub fn utility_start_name(&self, index: usize) -> Option<&str> {
        self.utility_start.get(index).map(|s| s.name.as_str())
    }

    /// Get the name of an end-of-chain utility pedal.
    pub fn utility_end_name(&self, index: usize) -> Option<&str> {
        self.utility_end.get(index).map(|s| s.name.as_str())
    }
}

/// Process signal through a slice of pedal slots.
fn process_slots(slots: &mut [PedalSlot], signal: &mut f64) {
    for slot in slots {
        if !slot.bypassed {
            if let Some(ref mut proc) = slot.processor {
                *signal = proc.process(*signal);
            }
        }
    }
}

/// Set sample rate on all slots.
fn set_sample_rate_slots(slots: &mut [PedalSlot], rate: f64) {
    for slot in slots {
        if let Some(ref mut proc) = slot.processor {
            proc.set_sample_rate(rate);
        }
    }
}

/// Reset all slots.
fn reset_slots(slots: &mut [PedalSlot]) {
    for slot in slots {
        if let Some(ref mut proc) = slot.processor {
            proc.reset();
        }
    }
}

/// Dispatch a control to a slot by index within a slice.
fn set_control_slot(slots: &mut [PedalSlot], idx: usize, label: &str, value: f64) {
    if let Some(slot) = slots.get_mut(idx) {
        if label == "__bypass__" {
            slot.bypassed = value > 0.5;
        } else if let Some(ref mut proc) = slot.processor {
            proc.set_control(label, value);
        }
    }
}

impl PedalProcessor for PedalboardProcessor {
    fn process(&mut self, input: f64) -> f64 {
        let mut signal = input;
        // Utility pedals at chain start (e.g. noise gate)
        process_slots(&mut self.utility_start, &mut signal);
        // User pedals
        process_slots(&mut self.pedals, &mut signal);
        // Utility pedals at chain end (e.g. limiter, buffer)
        process_slots(&mut self.utility_end, &mut signal);
        signal
    }

    fn set_sample_rate(&mut self, rate: f64) {
        set_sample_rate_slots(&mut self.utility_start, rate);
        set_sample_rate_slots(&mut self.pedals, rate);
        set_sample_rate_slots(&mut self.utility_end, rate);
    }

    fn reset(&mut self) {
        reset_slots(&mut self.utility_start);
        reset_slots(&mut self.pedals);
        reset_slots(&mut self.utility_end);
    }

    /// Control routing:
    /// - `"idx:label"` → user pedal at index `idx`
    /// - `"start:idx:label"` → utility pedal at chain start
    /// - `"end:idx:label"` → utility pedal at chain end
    /// - `"idx:__bypass__"` → toggle bypass (value > 0.5 = bypassed)
    fn set_control(&mut self, label: &str, value: f64) {
        if let Some((prefix, remainder)) = label.split_once(':') {
            match prefix {
                "start" => {
                    if let Some((idx_str, ctrl)) = remainder.split_once(':') {
                        if let Ok(idx) = idx_str.parse::<usize>() {
                            set_control_slot(&mut self.utility_start, idx, ctrl, value);
                        }
                    }
                }
                "end" => {
                    if let Some((idx_str, ctrl)) = remainder.split_once(':') {
                        if let Ok(idx) = idx_str.parse::<usize>() {
                            set_control_slot(&mut self.utility_end, idx, ctrl, value);
                        }
                    }
                }
                _ => {
                    if let Ok(idx) = prefix.parse::<usize>() {
                        set_control_slot(&mut self.pedals, idx, remainder, value);
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Board Switcher — meta-processor for switching between pre-compiled boards
// ---------------------------------------------------------------------------

/// Meta-processor wrapping multiple pre-compiled pedalboards.
///
/// Delegates `process()` to the active board. Switching is instant —
/// just an index change, no allocation, no audio gap.
pub struct BoardSwitcherProcessor {
    boards: Vec<PedalboardProcessor>,
    active: usize,
}

impl BoardSwitcherProcessor {
    /// Create a new board switcher from a vec of pre-compiled pedalboards.
    pub fn new(boards: Vec<PedalboardProcessor>) -> Self {
        Self { boards, active: 0 }
    }

    /// Number of boards.
    pub fn len(&self) -> usize {
        self.boards.len()
    }

    /// Whether there are no boards.
    pub fn is_empty(&self) -> bool {
        self.boards.is_empty()
    }

    /// Current active board index.
    pub fn active(&self) -> usize {
        self.active
    }
}

impl PedalProcessor for BoardSwitcherProcessor {
    fn process(&mut self, input: f64) -> f64 {
        if let Some(board) = self.boards.get_mut(self.active) {
            board.process(input)
        } else {
            input
        }
    }

    fn set_sample_rate(&mut self, rate: f64) {
        for board in &mut self.boards {
            board.set_sample_rate(rate);
        }
    }

    fn reset(&mut self) {
        for board in &mut self.boards {
            board.reset();
        }
    }

    fn set_control(&mut self, label: &str, value: f64) {
        if label == "__switch_board__" {
            let idx = value as usize;
            if idx < self.boards.len() {
                self.active = idx;
            }
        } else if let Some(board) = self.boards.get_mut(self.active) {
            board.set_control(label, value);
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::parse_board_file;

    #[test]
    fn parse_and_build_pedalboard() {
        let src = r#"
board "Test Board" {
  ts: "tube_screamer.pedal" { Drive = 0.6, Level = 0.7 }
}
"#;
        let board = parse_board_file(src).unwrap();
        let board_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let pb = PedalboardProcessor::from_board(&board, &board_dir, 48000.0).unwrap();
        assert_eq!(pb.len(), 1);
        assert_eq!(pb.pedal_name(0), Some("Tube Screamer"));
        assert!(!pb.is_pedal_bypassed(0));
    }

    #[test]
    fn pedalboard_processes_audio() {
        let src = r#"
board "Chain" {
  ts: "tube_screamer.pedal"
  bd: "blues_driver.pedal"
}
"#;
        let board = parse_board_file(src).unwrap();
        let board_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let mut pb = PedalboardProcessor::from_board(&board, &board_dir, 48000.0).unwrap();

        // Process some samples — should produce non-zero output
        let mut max_out = 0.0_f64;
        for i in 0..1000 {
            let input = (i as f64 * 440.0 * 2.0 * std::f64::consts::PI / 48000.0).sin() * 0.1;
            let output = pb.process(input);
            max_out = max_out.max(output.abs());
        }
        assert!(max_out > 0.0001, "pedalboard should produce output");
    }

    #[test]
    fn pedalboard_bypass_control() {
        let src = r#"
board "Bypass Test" {
  ts: "tube_screamer.pedal"
}
"#;
        let board = parse_board_file(src).unwrap();
        let board_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let mut pb = PedalboardProcessor::from_board(&board, &board_dir, 48000.0).unwrap();

        // Bypass pedal 0
        pb.set_control("0:__bypass__", 1.0);
        assert!(pb.is_pedal_bypassed(0));

        // Bypassed pedal should pass through
        let input = 0.42;
        let output = pb.process(input);
        assert!((output - input).abs() < f64::EPSILON);

        // Un-bypass
        pb.set_control("0:__bypass__", 0.0);
        assert!(!pb.is_pedal_bypassed(0));
    }

    #[test]
    fn pedalboard_knob_control_routing() {
        let src = r#"
board "Knob Test" {
  ts: "tube_screamer.pedal"
}
"#;
        let board = parse_board_file(src).unwrap();
        let board_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let mut pb = PedalboardProcessor::from_board(&board, &board_dir, 48000.0).unwrap();

        // Setting a control via the routing scheme should not panic
        pb.set_control("0:Drive", 0.9);
        pb.set_control("0:Level", 0.3);

        // Invalid index should be a no-op
        pb.set_control("99:Drive", 0.5);
    }

    #[test]
    fn pedalboard_missing_pedal_file_uses_dry_passthrough() {
        let src = r#"
board "Bad" {
  x: "nonexistent.pedal"
}
"#;
        let board = parse_board_file(src).unwrap();
        let board_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let (pb, warnings) =
            PedalboardProcessor::from_board_with_warnings(&board, &board_dir, 48000.0);

        // Board still loads — the failed pedal becomes dry passthrough
        assert_eq!(pb.len(), 1);
        assert!(pb.is_pedal_failed(0));
        assert!(!warnings.is_empty());

        // Dry passthrough: input == output
        let mut pb = pb;
        let output = pb.process(0.42);
        assert!((output - 0.42).abs() < f64::EPSILON);
    }

    // ── BoardSwitcherProcessor tests ─────────────────────────────────────

    fn make_test_board(pedal_file: &str) -> PedalboardProcessor {
        let src = format!(
            r#"board "Test" {{
  p: "{pedal_file}"
}}"#
        );
        let board = parse_board_file(&src).unwrap();
        let board_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        PedalboardProcessor::from_board(&board, &board_dir, 48000.0).unwrap()
    }

    #[test]
    fn switcher_delegates_to_active_board() {
        let board0 = make_test_board("tube_screamer.pedal");
        let board1 = make_test_board("blues_driver.pedal");
        let mut switcher = BoardSwitcherProcessor::new(vec![board0, board1]);

        assert_eq!(switcher.len(), 2);
        assert_eq!(switcher.active(), 0);

        // Process through board 0
        let out0 = switcher.process(0.1);

        // Switch to board 1
        switcher.set_control("__switch_board__", 1.0);
        assert_eq!(switcher.active(), 1);

        // Process through board 1 — output should differ from board 0
        let out1 = switcher.process(0.1);
        // Both should produce output (not necessarily different on first sample,
        // but the switcher should not crash)
        assert!(out0.is_finite());
        assert!(out1.is_finite());
    }

    #[test]
    fn switcher_bounds_check() {
        let board0 = make_test_board("tube_screamer.pedal");
        let mut switcher = BoardSwitcherProcessor::new(vec![board0]);

        // Out-of-bounds switch should be ignored
        switcher.set_control("__switch_board__", 99.0);
        assert_eq!(switcher.active(), 0);
    }

    #[test]
    fn switcher_forwards_controls_to_active() {
        let board0 = make_test_board("tube_screamer.pedal");
        let board1 = make_test_board("blues_driver.pedal");
        let mut switcher = BoardSwitcherProcessor::new(vec![board0, board1]);

        // Forward a control to board 0 (active) — should not panic
        switcher.set_control("0:Drive", 0.9);

        // Switch and forward to board 1
        switcher.set_control("__switch_board__", 1.0);
        switcher.set_control("0:Gain", 0.8);
    }

    #[test]
    fn switcher_set_sample_rate_all_boards() {
        let board0 = make_test_board("tube_screamer.pedal");
        let board1 = make_test_board("blues_driver.pedal");
        let mut switcher = BoardSwitcherProcessor::new(vec![board0, board1]);

        // Should not panic — sets rate on all boards
        switcher.set_sample_rate(44100.0);
        switcher.reset();
    }

    // ── Utility pedal tests ─────────────────────────────────────────────

    #[test]
    fn board_with_utility_start() {
        let src = r#"
board "Gated Rig" {
  ts: "tube_screamer.pedal" { Drive = 0.7 }
}

utilities {
  start { ns: "noise_suppressor.pedal" { Threshold = 0.5 } }
}
"#;
        let board = parse_board_file(src).unwrap();
        assert_eq!(board.pedals.len(), 1);
        assert_eq!(board.utility_start.len(), 1);
        assert_eq!(board.utility_start[0].id, "ns");
        assert!(board.utility_end.is_empty());

        let board_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let pb = PedalboardProcessor::from_board(&board, &board_dir, 48000.0).unwrap();
        assert_eq!(pb.len(), 1); // user pedals only
        assert_eq!(pb.utility_start_len(), 1);
        assert_eq!(pb.utility_start_name(0), Some("Noise Suppressor"));
    }

    #[test]
    fn board_with_utility_start_and_end() {
        let src = r#"
board "Full Rig" {
  ts: "tube_screamer.pedal"
}

utilities {
  start { ns: "noise_suppressor.pedal" }
  end   { buf: "noise_suppressor.pedal" }
}
"#;
        let board = parse_board_file(src).unwrap();
        assert_eq!(board.utility_start.len(), 1);
        assert_eq!(board.utility_end.len(), 1);

        let board_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let pb = PedalboardProcessor::from_board(&board, &board_dir, 48000.0).unwrap();
        assert_eq!(pb.utility_start_len(), 1);
        assert_eq!(pb.utility_end_len(), 1);
    }

    #[test]
    fn utility_pedal_processes_audio() {
        let src = r#"
board "Gated" {
  ts: "tube_screamer.pedal"
}

utilities {
  start { ns: "noise_suppressor.pedal" { Threshold = 0.3 } }
}
"#;
        let board = parse_board_file(src).unwrap();
        let board_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let mut pb = PedalboardProcessor::from_board(&board, &board_dir, 48000.0).unwrap();

        // Process a real signal — should produce output through the whole chain
        let mut max_out = 0.0_f64;
        for i in 0..4800 {
            let input = (i as f64 * 440.0 * 2.0 * std::f64::consts::PI / 48000.0).sin() * 0.3;
            let output = pb.process(input);
            max_out = max_out.max(output.abs());
        }
        assert!(
            max_out > 0.001,
            "signal should pass through gated chain: {max_out}"
        );
    }

    #[test]
    fn utility_pedal_control_routing() {
        let src = r#"
board "Control Test" {
  ts: "tube_screamer.pedal"
}

utilities {
  start { ns: "noise_suppressor.pedal" }
}
"#;
        let board = parse_board_file(src).unwrap();
        let board_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let mut pb = PedalboardProcessor::from_board(&board, &board_dir, 48000.0).unwrap();

        // Control utility pedal via "start:idx:label" routing
        pb.set_control("start:0:Threshold", 0.8);
        pb.set_control("start:0:Decay", 0.3);

        // Control user pedal via normal routing
        pb.set_control("0:Drive", 0.9);

        // Neither should panic
    }

    #[test]
    fn board_without_utilities_still_works() {
        // Existing boards without utilities block should work unchanged
        let src = r#"
board "Simple" {
  ts: "tube_screamer.pedal"
}
"#;
        let board = parse_board_file(src).unwrap();
        assert!(board.utility_start.is_empty());
        assert!(board.utility_end.is_empty());

        let board_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let mut pb = PedalboardProcessor::from_board(&board, &board_dir, 48000.0).unwrap();
        assert_eq!(pb.utility_start_len(), 0);
        assert_eq!(pb.utility_end_len(), 0);

        // Process should still work
        let output = pb.process(0.1);
        assert!(output.is_finite());
    }
}
