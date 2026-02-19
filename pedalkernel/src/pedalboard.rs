//! Pedalboard processor — chains multiple compiled pedals in series.
//!
//! Implements [`PedalProcessor`] so it can be used directly with
//! [`AudioEngine::start()`] — zero changes to the JACK engine.
//!
//! Control routing uses a `"idx:label"` prefix scheme:
//! - `"0:Drive"` → pedal index 0, knob "Drive"
//! - `"2:__bypass__"` → toggle bypass on pedal index 2

use crate::board::{find_global_board, BoardDef, BoardPedalEntry};
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
}

/// Chains N `CompiledPedal` instances in series.
pub struct PedalboardProcessor {
    pedals: Vec<PedalSlot>,
}

/// Search subdirectories of `base` for a file matching `filename`.
fn find_in_subdirs(base: &Path, filename: &str) -> Option<std::path::PathBuf> {
    let target = Path::new(filename).file_name()?.to_str()?;
    walk_for_file(base, target)
}

fn walk_for_file(dir: &Path, target: &str) -> Option<std::path::PathBuf> {
    for entry in std::fs::read_dir(dir).ok()?.flatten() {
        let path = entry.path();
        if path.is_dir() {
            if let Some(found) = walk_for_file(&path, target) {
                return Some(found);
            }
        } else if path.file_name().and_then(|n| n.to_str()) == Some(target) {
            return Some(path);
        }
    }
    None
}

/// Build `PedalSlot`s from a list of `BoardPedalEntry`, appending to `out`.
/// Failed pedals become dry passthrough slots with a warning.
fn build_slots(
    entries: &[BoardPedalEntry],
    base_dir: &Path,
    sample_rate: f64,
    out: &mut Vec<PedalSlot>,
    warnings: &mut Vec<String>,
) {
    for entry in entries {
        // Try direct path first, then search subdirectories by filename.
        let pedal_path = {
            let direct = base_dir.join(&entry.path);
            if direct.exists() {
                direct
            } else if let Some(found) = find_in_subdirs(base_dir, &entry.path) {
                found
            } else {
                direct // fall through to produce the original error message
            }
        };

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
                out.push(PedalSlot {
                    name,
                    processor: Some(proc),
                    bypassed: false,
                    failed: false,
                });
            }
            Err(msg) => {
                warnings.push(format!(
                    "Pedal '{}' failed — using dry passthrough: {msg}",
                    entry.id
                ));
                out.push(PedalSlot {
                    name: format!("{} [FAILED]", entry.id),
                    processor: None,
                    bypassed: false,
                    failed: true,
                });
            }
        }
    }
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
    ///
    /// If a `.global.board` file is found (by walking up from `board_dir`),
    /// its `start` pedals are prepended and `end` pedals are appended to
    /// the signal chain.
    pub fn from_board_with_warnings(
        board: &BoardDef,
        board_dir: &Path,
        sample_rate: f64,
    ) -> (Self, Vec<String>) {
        let mut warnings = Vec::new();

        // Detect global board
        let global = find_global_board(board_dir);
        if let Some((ref def, ref dir)) = global {
            let n_start = def.start.len();
            let n_end = def.end.len();
            eprintln!(
                "Global: {} ({} start, {} end)",
                dir.join(".global.board").display(),
                n_start,
                n_end,
            );
        }

        // Build slots: [global start] + [board] + [global end]
        let mut pedals = Vec::new();

        if let Some((ref def, ref global_dir)) = global {
            build_slots(&def.start, global_dir, sample_rate, &mut pedals, &mut warnings);
        }

        build_slots(&board.pedals, board_dir, sample_rate, &mut pedals, &mut warnings);

        if let Some((ref def, ref global_dir)) = global {
            build_slots(&def.end, global_dir, sample_rate, &mut pedals, &mut warnings);
        }

        (Self { pedals }, warnings)
    }

    /// Number of pedals in the chain.
    pub fn len(&self) -> usize {
        self.pedals.len()
    }

    /// Whether the pedalboard is empty.
    pub fn is_empty(&self) -> bool {
        self.pedals.is_empty()
    }

    /// Get the name of pedal at the given index.
    pub fn pedal_name(&self, index: usize) -> Option<&str> {
        self.pedals.get(index).map(|s| s.name.as_str())
    }

    /// Check if a pedal is bypassed.
    pub fn is_pedal_bypassed(&self, index: usize) -> bool {
        self.pedals.get(index).is_some_and(|s| s.bypassed)
    }

    /// Check if a pedal slot failed to compile (dry passthrough).
    pub fn is_pedal_failed(&self, index: usize) -> bool {
        self.pedals.get(index).is_some_and(|s| s.failed)
    }
}

impl PedalProcessor for PedalboardProcessor {
    fn process(&mut self, input: f64) -> f64 {
        let mut signal = input;
        for slot in &mut self.pedals {
            if !slot.bypassed {
                if let Some(ref mut proc) = slot.processor {
                    signal = proc.process(signal);
                }
                // None → dry passthrough (failed pedal)
            }
        }
        signal
    }

    fn set_sample_rate(&mut self, rate: f64) {
        for slot in &mut self.pedals {
            if let Some(ref mut proc) = slot.processor {
                proc.set_sample_rate(rate);
            }
        }
    }

    fn reset(&mut self) {
        for slot in &mut self.pedals {
            if let Some(ref mut proc) = slot.processor {
                proc.reset();
            }
        }
    }

    /// Control routing: `"idx:label"` dispatches to the appropriate pedal.
    ///
    /// Special control `"idx:__bypass__"` toggles bypass (value > 0.5 = bypassed).
    fn set_control(&mut self, label: &str, value: f64) {
        if let Some((prefix, remainder)) = label.split_once(':') {
            if let Ok(idx) = prefix.parse::<usize>() {
                if let Some(slot) = self.pedals.get_mut(idx) {
                    if remainder == "__bypass__" {
                        slot.bypassed = value > 0.5;
                    } else if let Some(ref mut proc) = slot.processor {
                        proc.set_control(remainder, value);
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
// PedalSwitcherProcessor — switches between individual compiled pedals
// ---------------------------------------------------------------------------

/// Meta-processor wrapping multiple pre-compiled individual pedals.
///
/// Delegates `process()` to the active pedal. Switching is instant —
/// just an index change, no allocation, no audio gap.
/// Unlike `BoardSwitcherProcessor`, forwards bare control labels (no prefix).
pub struct PedalSwitcherProcessor {
    pedals: Vec<CompiledPedal>,
    active: usize,
}

impl PedalSwitcherProcessor {
    /// Create a new pedal switcher from a vec of pre-compiled pedals.
    pub fn new(pedals: Vec<CompiledPedal>) -> Self {
        Self { pedals, active: 0 }
    }

    /// Number of pedals.
    pub fn len(&self) -> usize {
        self.pedals.len()
    }

    /// Whether there are no pedals.
    pub fn is_empty(&self) -> bool {
        self.pedals.is_empty()
    }

    /// Current active pedal index.
    pub fn active(&self) -> usize {
        self.active
    }
}

impl PedalProcessor for PedalSwitcherProcessor {
    fn process(&mut self, input: f64) -> f64 {
        if let Some(pedal) = self.pedals.get_mut(self.active) {
            pedal.process(input)
        } else {
            input
        }
    }

    fn set_sample_rate(&mut self, rate: f64) {
        for pedal in &mut self.pedals {
            pedal.set_sample_rate(rate);
        }
    }

    fn reset(&mut self) {
        for pedal in &mut self.pedals {
            pedal.reset();
        }
    }

    fn set_control(&mut self, label: &str, value: f64) {
        if label == "__switch_pedal__" {
            let idx = value as usize;
            if idx < self.pedals.len() {
                self.active = idx;
            }
        } else if let Some(pedal) = self.pedals.get_mut(self.active) {
            pedal.set_control(label, value);
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

    // ── Global board tests ──────────────────────────────────────────────

    #[test]
    fn global_board_prepends_and_appends() {
        let examples = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let ts_path = examples.join("pedals/overdrive/tube_screamer.pedal").canonicalize().unwrap();
        let comp_path = examples.join("pedals/compressor/dyna_comp.pedal").canonicalize().unwrap();
        let bd_path = examples.join("pedals/overdrive/blues_driver.pedal").canonicalize().unwrap();

        let tmp = tempfile::tempdir().unwrap();

        // Create .global.board with absolute paths
        let global_content = format!(
            "global {{\n  start {{\n    comp: \"{}\" {{ Sensitivity = 0.3 }}\n  }}\n  end {{\n    bd: \"{}\"\n  }}\n}}",
            comp_path.display(),
            bd_path.display(),
        );
        std::fs::write(tmp.path().join(".global.board"), &global_content).unwrap();

        // Create board file in subdirectory
        let sub = tmp.path().join("boards");
        std::fs::create_dir(&sub).unwrap();
        let board_content = format!(
            "board \"Test\" {{\n  ts: \"{}\" {{ Drive = 0.6 }}\n}}",
            ts_path.display(),
        );
        std::fs::write(sub.join("test.board"), &board_content).unwrap();

        let board = parse_board_file(&board_content).unwrap();
        let (pb, warnings) =
            PedalboardProcessor::from_board_with_warnings(&board, &sub, 48000.0);

        // Should have 3 pedals: comp (start) + ts (board) + bd (end)
        assert_eq!(pb.len(), 3, "warnings: {warnings:?}");
        assert_eq!(pb.pedal_name(0), Some("MXR Dyna Comp"));
        assert_eq!(pb.pedal_name(1), Some("Tube Screamer"));
        assert_eq!(pb.pedal_name(2), Some("Boss Blues Driver"));
    }

    #[test]
    fn global_board_start_only() {
        let examples = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let ts_path = examples.join("pedals/overdrive/tube_screamer.pedal").canonicalize().unwrap();
        let comp_path = examples.join("pedals/compressor/dyna_comp.pedal").canonicalize().unwrap();

        let tmp = tempfile::tempdir().unwrap();

        let global_content = format!(
            "global {{\n  start {{\n    comp: \"{}\"\n  }}\n}}",
            comp_path.display(),
        );
        std::fs::write(tmp.path().join(".global.board"), &global_content).unwrap();

        let board_content = format!(
            "board \"Test\" {{\n  ts: \"{}\"\n}}",
            ts_path.display(),
        );

        let board = parse_board_file(&board_content).unwrap();
        let (pb, _) =
            PedalboardProcessor::from_board_with_warnings(&board, tmp.path(), 48000.0);

        assert_eq!(pb.len(), 2);
        assert_eq!(pb.pedal_name(0), Some("MXR Dyna Comp"));
        assert_eq!(pb.pedal_name(1), Some("Tube Screamer"));
    }

    #[test]
    fn global_board_signal_chain_order() {
        let examples = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let ts_path = examples.join("pedals/overdrive/tube_screamer.pedal").canonicalize().unwrap();
        let comp_path = examples.join("pedals/compressor/dyna_comp.pedal").canonicalize().unwrap();
        let bd_path = examples.join("pedals/overdrive/blues_driver.pedal").canonicalize().unwrap();

        let tmp = tempfile::tempdir().unwrap();

        let global_content = format!(
            "global {{\n  start {{\n    comp: \"{}\"\n  }}\n  end {{\n    bd: \"{}\"\n  }}\n}}",
            comp_path.display(),
            bd_path.display(),
        );
        std::fs::write(tmp.path().join(".global.board"), &global_content).unwrap();

        let board_content = format!(
            "board \"Chain\" {{\n  ts: \"{}\" {{ Drive = 0.8 }}\n}}",
            ts_path.display(),
        );

        let board = parse_board_file(&board_content).unwrap();
        let (pb, _) =
            PedalboardProcessor::from_board_with_warnings(&board, tmp.path(), 48000.0);

        // Signal chain: comp → ts → bd
        assert_eq!(pb.len(), 3);

        // Process audio — should produce non-zero, finite output
        let mut pb = pb;
        let mut max_out = 0.0_f64;
        for i in 0..1000 {
            let input = (i as f64 * 440.0 * 2.0 * std::f64::consts::PI / 48000.0).sin() * 0.1;
            let output = pb.process(input);
            assert!(output.is_finite());
            max_out = max_out.max(output.abs());
        }
        assert!(max_out > 0.0001, "pedalboard with global should produce output");
    }

    #[test]
    fn no_global_board_unchanged() {
        // Board in a dir with no .global.board — should work as before
        let tmp = tempfile::tempdir().unwrap();
        let examples = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let ts_path = examples.join("pedals/overdrive/tube_screamer.pedal").canonicalize().unwrap();

        let board_content = format!(
            "board \"Solo\" {{\n  ts: \"{}\"\n}}",
            ts_path.display(),
        );

        let board = parse_board_file(&board_content).unwrap();
        let (pb, warnings) =
            PedalboardProcessor::from_board_with_warnings(&board, tmp.path(), 48000.0);

        assert_eq!(pb.len(), 1);
        assert!(warnings.is_empty());
        assert_eq!(pb.pedal_name(0), Some("Tube Screamer"));
    }

    #[test]
    fn global_board_control_routing_flat_index() {
        let examples = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let ts_path = examples.join("pedals/overdrive/tube_screamer.pedal").canonicalize().unwrap();
        let comp_path = examples.join("pedals/compressor/dyna_comp.pedal").canonicalize().unwrap();

        let tmp = tempfile::tempdir().unwrap();

        let global_content = format!(
            "global {{\n  start {{\n    comp: \"{}\"\n  }}\n}}",
            comp_path.display(),
        );
        std::fs::write(tmp.path().join(".global.board"), &global_content).unwrap();

        let board_content = format!(
            "board \"Test\" {{\n  ts: \"{}\"\n}}",
            ts_path.display(),
        );

        let board = parse_board_file(&board_content).unwrap();
        let (mut pb, _) =
            PedalboardProcessor::from_board_with_warnings(&board, tmp.path(), 48000.0);

        // Index 0 = global comp, index 1 = board ts
        // Bypass the global comp via flat index
        pb.set_control("0:__bypass__", 1.0);
        assert!(pb.is_pedal_bypassed(0));
        assert!(!pb.is_pedal_bypassed(1));

        // Route a knob control to the board pedal
        pb.set_control("1:Drive", 0.9);
    }
}
