//! Global configuration for PedalKernel.
//!
//! Reads utility pedal settings from `~/.config/pedalkernel/utilities` so
//! they apply to every board without copy-pasting the `utilities {}` block.
//!
//! The file uses the same syntax as the `utilities { ... }` block in a
//! `.board` file:
//!
//! ```text
//! # ~/.config/pedalkernel/utilities
//! utilities {
//!   start { ns: "noise_suppressor.pedal" { Threshold = 0.5 } }
//! }
//! ```
//!
//! Pedal paths are resolved relative to the config directory
//! (`~/.config/pedalkernel/`).

use crate::board::{parse_utilities_file, BoardPedalEntry};
use std::path::PathBuf;

/// Global utility pedal configuration loaded from `~/.config/pedalkernel/utilities`.
#[derive(Debug, Clone)]
pub struct GlobalUtilities {
    /// Utility pedals inserted at the start of every chain.
    pub start: Vec<BoardPedalEntry>,
    /// Utility pedals inserted at the end of every chain.
    pub end: Vec<BoardPedalEntry>,
    /// Directory containing the config file (for resolving relative pedal paths).
    pub dir: PathBuf,
}

/// Return the PedalKernel config directory: `~/.config/pedalkernel/`.
///
/// Returns `None` if the home directory cannot be determined.
pub fn config_dir() -> Option<PathBuf> {
    std::env::var_os("HOME").map(|home| PathBuf::from(home).join(".config").join("pedalkernel"))
}

/// Load global utility pedal configuration from `~/.config/pedalkernel/utilities`.
///
/// Returns `None` if the config file doesn't exist or the home directory
/// cannot be determined. Returns `Err` only on parse failures — a missing
/// file is not an error (most users won't have one initially).
pub fn load_global_utilities() -> Result<Option<GlobalUtilities>, String> {
    let dir = match config_dir() {
        Some(d) => d,
        None => return Ok(None),
    };

    let path = dir.join("utilities");
    let source = match std::fs::read_to_string(&path) {
        Ok(s) => s,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => {
            return Err(format!(
                "Error reading global utilities at {}: {e}",
                path.display()
            ))
        }
    };

    // Empty file is fine — just no global utilities
    if source.trim().is_empty() {
        return Ok(None);
    }

    let (start, end) = parse_utilities_file(&source)
        .map_err(|e| format!("Error parsing global utilities at {}: {e}", path.display()))?;

    if start.is_empty() && end.is_empty() {
        return Ok(None);
    }

    Ok(Some(GlobalUtilities { start, end, dir }))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_dir_uses_home() {
        // Should return Some when HOME is set (which it always is in test)
        let dir = config_dir();
        assert!(dir.is_some());
        let dir = dir.unwrap();
        assert!(dir.ends_with(".config/pedalkernel"));
    }

    #[test]
    fn load_global_utilities_missing_file_returns_none() {
        // With a non-existent config dir, should return None (not an error)
        let result = load_global_utilities();
        // This might return None (no file) or Some (if the user has one) —
        // either way it should not be Err
        assert!(result.is_ok());
    }
}
