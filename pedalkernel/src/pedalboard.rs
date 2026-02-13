//! Pedalboard processor — chains multiple compiled pedals in series.
//!
//! Implements [`PedalProcessor`] so it can be used directly with
//! [`AudioEngine::start()`] — zero changes to the JACK engine.
//!
//! Control routing uses a `"idx:label"` prefix scheme:
//! - `"0:Drive"` → pedal index 0, knob "Drive"
//! - `"2:__bypass__"` → toggle bypass on pedal index 2

use crate::board::BoardDef;
use crate::compiler::{compile_pedal, CompiledPedal};
use crate::dsl::parse_pedal_file;
use crate::PedalProcessor;
use std::path::Path;

/// A single slot in the pedalboard chain.
struct PedalSlot {
    name: String,
    processor: CompiledPedal,
    bypassed: bool,
}

/// Chains N `CompiledPedal` instances in series.
pub struct PedalboardProcessor {
    pedals: Vec<PedalSlot>,
}

impl PedalboardProcessor {
    /// Build a pedalboard from a parsed `.board` definition.
    ///
    /// Each `.pedal` file is read relative to `board_dir`, compiled at the
    /// given sample rate, and any knob overrides from the board file are applied.
    pub fn from_board(
        board: &BoardDef,
        board_dir: &Path,
        sample_rate: f64,
    ) -> Result<Self, String> {
        let mut pedals = Vec::with_capacity(board.pedals.len());

        for entry in &board.pedals {
            let pedal_path = board_dir.join(&entry.path);
            let source = std::fs::read_to_string(&pedal_path).map_err(|e| {
                format!(
                    "Error reading pedal '{}' at {}: {e}",
                    entry.id,
                    pedal_path.display()
                )
            })?;
            let pedal_def = parse_pedal_file(&source).map_err(|e| {
                format!("Parse error in '{}' ({}): {e}", entry.id, entry.path)
            })?;
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

            pedals.push(PedalSlot {
                name: pedal_def.name.clone(),
                processor: proc,
                bypassed: false,
            });
        }

        Ok(Self { pedals })
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
        self.pedals.get(index).map_or(false, |s| s.bypassed)
    }
}

impl PedalProcessor for PedalboardProcessor {
    fn process(&mut self, input: f64) -> f64 {
        let mut signal = input;
        for slot in &mut self.pedals {
            if !slot.bypassed {
                signal = slot.processor.process(signal);
            }
        }
        signal
    }

    fn set_sample_rate(&mut self, rate: f64) {
        for slot in &mut self.pedals {
            slot.processor.set_sample_rate(rate);
        }
    }

    fn reset(&mut self) {
        for slot in &mut self.pedals {
            slot.processor.reset();
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
                    } else {
                        slot.processor.set_control(remainder, value);
                    }
                }
            }
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
    fn pedalboard_missing_pedal_file() {
        let src = r#"
board "Bad" {
  x: "nonexistent.pedal"
}
"#;
        let board = parse_board_file(src).unwrap();
        let board_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
        let result = PedalboardProcessor::from_board(&board, &board_dir, 48000.0);
        assert!(result.is_err());
    }
}
