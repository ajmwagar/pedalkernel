//! Parser for `.board` pedalboard definition files.
//!
//! A `.board` file defines a signal chain of `.pedal` files with optional
//! per-pedal knob overrides.
//!
//! ```text
//! board "Blues Rig" {
//!   ts: "tube_screamer.pedal" { Drive = 0.6, Level = 0.7 }
//!   bd: "blues_driver.pedal"
//! }
//! ```

use nom::{
    branch::alt,
    bytes::complete::{tag, take_while, take_while1},
    character::complete::{char, multispace1, not_line_ending},
    combinator::{opt, recognize, value},
    multi::{many0, separated_list1},
    number::complete::double,
    sequence::pair,
    IResult,
};
use std::path::{Path, PathBuf};

// ---------------------------------------------------------------------------
// AST
// ---------------------------------------------------------------------------

/// Top-level pedalboard definition.
#[derive(Debug, Clone, PartialEq)]
pub struct BoardDef {
    pub name: String,
    pub pedals: Vec<BoardPedalEntry>,
}

/// A single pedal entry in the board.
#[derive(Debug, Clone, PartialEq)]
pub struct BoardPedalEntry {
    /// Identifier for this pedal slot (e.g. "ts").
    pub id: String,
    /// Path to the `.pedal` file (resolved relative to the `.board` file).
    pub path: String,
    /// Optional knob overrides (e.g. `[("Drive", 0.7)]`).
    pub overrides: Vec<(String, f64)>,
}

/// Global board definition — pedals auto-injected at the start/end of every board.
#[derive(Debug, Clone, PartialEq)]
pub struct GlobalBoardDef {
    pub start: Vec<BoardPedalEntry>,
    pub end: Vec<BoardPedalEntry>,
}

// ---------------------------------------------------------------------------
// Helpers (same patterns as dsl.rs)
// ---------------------------------------------------------------------------

/// Consume whitespace **and** `# …` comments.
fn ws_comments(input: &str) -> IResult<&str, ()> {
    let (input, _) = many0(alt((
        value((), multispace1),
        value((), pair(char('#'), not_line_ending)),
    )))(input)?;
    Ok((input, ()))
}

/// Identifier: starts with alpha/underscore, continues with alphanumeric/underscore.
fn identifier(input: &str) -> IResult<&str, &str> {
    recognize(pair(
        take_while1(|c: char| c.is_ascii_alphabetic() || c == '_'),
        take_while(|c: char| c.is_ascii_alphanumeric() || c == '_'),
    ))(input)
}

/// Quoted string: `"Foo Bar"`
fn quoted_string(input: &str) -> IResult<&str, &str> {
    nom::sequence::delimited(char('"'), take_while(|c: char| c != '"'), char('"'))(input)
}

// ---------------------------------------------------------------------------
// Knob override parsers
// ---------------------------------------------------------------------------

/// Parse a single knob override: `Drive = 0.7`
fn knob_override(input: &str) -> IResult<&str, (String, f64)> {
    let (input, _) = ws_comments(input)?;
    let (input, name) = identifier(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('=')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, val) = double(input)?;
    Ok((input, (name.to_string(), val)))
}

/// Parse knob overrides block: `{ Drive = 0.6, Level = 0.7 }`
fn knob_overrides(input: &str) -> IResult<&str, Vec<(String, f64)>> {
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('{')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, overrides) = separated_list1(
        |i| {
            let (i, _) = ws_comments(i)?;
            let (i, _) = char(',')(i)?;
            let (i, _) = ws_comments(i)?;
            Ok((i, ()))
        },
        knob_override,
    )(input)?;
    let (input, _) = ws_comments(input)?;
    // Allow optional trailing comma
    let (input, _) = opt(char(','))(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('}')(input)?;
    Ok((input, overrides))
}

// ---------------------------------------------------------------------------
// Pedal entry parser
// ---------------------------------------------------------------------------

/// Parse a single pedal entry: `ts: "tube_screamer.pedal" { Drive = 0.6 }`
fn pedal_entry(input: &str) -> IResult<&str, BoardPedalEntry> {
    let (input, _) = ws_comments(input)?;
    let (input, id) = identifier(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(':')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, path) = quoted_string(input)?;
    let (input, overrides) = opt(knob_overrides)(input)?;
    Ok((
        input,
        BoardPedalEntry {
            id: id.to_string(),
            path: path.to_string(),
            overrides: overrides.unwrap_or_default(),
        },
    ))
}

// ---------------------------------------------------------------------------
// Top-level
// ---------------------------------------------------------------------------

/// Parse a complete `.board` file.
fn parse_board(input: &str) -> IResult<&str, BoardDef> {
    let (input, _) = ws_comments(input)?;
    let (input, _) = tag("board")(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, name) = quoted_string(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('{')(input)?;
    let (input, pedals) = many0(pedal_entry)(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('}')(input)?;
    let (input, _) = ws_comments(input)?;
    Ok((
        input,
        BoardDef {
            name: name.to_string(),
            pedals,
        },
    ))
}

/// Convenience wrapper that returns `Result`.
pub fn parse_board_file(src: &str) -> Result<BoardDef, String> {
    match parse_board(src) {
        Ok(("", def)) => Ok(def),
        Ok((rest, _)) => Err(format!("Trailing input: {:?}", &rest[..rest.len().min(60)])),
        Err(e) => Err(format!("Parse error: {e}")),
    }
}

// ---------------------------------------------------------------------------
// Global board parser
// ---------------------------------------------------------------------------

/// Parse a `start { ... }` or `end { ... }` section inside a global block.
fn global_section<'a>(keyword: &'static str) -> impl FnMut(&'a str) -> IResult<&'a str, Vec<BoardPedalEntry>> {
    move |input: &'a str| {
        let (input, _) = ws_comments(input)?;
        let (input, _) = tag(keyword)(input)?;
        let (input, _) = ws_comments(input)?;
        let (input, _) = char('{')(input)?;
        let (input, entries) = many0(pedal_entry)(input)?;
        let (input, _) = ws_comments(input)?;
        let (input, _) = char('}')(input)?;
        Ok((input, entries))
    }
}

/// Parse a complete `.global.board` file.
fn parse_global_board(input: &str) -> IResult<&str, GlobalBoardDef> {
    let (input, _) = ws_comments(input)?;
    let (input, _) = tag("global")(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('{')(input)?;

    // Both sections are optional and can appear in any order
    let mut start = Vec::new();
    let mut end = Vec::new();
    let mut input = input;

    loop {
        let (rest, _) = ws_comments(input)?;
        input = rest;

        if let Ok((rest, entries)) = global_section("start")(input) {
            start = entries;
            input = rest;
        } else if let Ok((rest, entries)) = global_section("end")(input) {
            end = entries;
            input = rest;
        } else {
            break;
        }
    }

    let (input, _) = ws_comments(input)?;
    let (input, _) = char('}')(input)?;
    let (input, _) = ws_comments(input)?;

    Ok((input, GlobalBoardDef { start, end }))
}

/// Parse a `.global.board` file, returning a `Result`.
pub fn parse_global_board_file(src: &str) -> Result<GlobalBoardDef, String> {
    match parse_global_board(src) {
        Ok(("", def)) => Ok(def),
        Ok((rest, _)) => Err(format!("Trailing input: {:?}", &rest[..rest.len().min(60)])),
        Err(e) => Err(format!("Parse error: {e}")),
    }
}

/// Search for a `.global.board` file by walking up from `start_dir`.
///
/// Returns the parsed definition and the directory it was found in
/// (for resolving relative pedal paths).
pub fn find_global_board(start_dir: &Path) -> Option<(GlobalBoardDef, PathBuf)> {
    let mut dir = start_dir.canonicalize().ok()?;
    loop {
        let candidate = dir.join(".global.board");
        if candidate.is_file() {
            let source = std::fs::read_to_string(&candidate).ok()?;
            let def = parse_global_board_file(&source).ok()?;
            return Some((def, dir));
        }
        if !dir.pop() {
            return None;
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_simple_board() {
        let src = r#"
board "Blues Rig" {
  ts: "tube_screamer.pedal"
  bd: "blues_driver.pedal"
}
"#;
        let board = parse_board_file(src).unwrap();
        assert_eq!(board.name, "Blues Rig");
        assert_eq!(board.pedals.len(), 2);
        assert_eq!(board.pedals[0].id, "ts");
        assert_eq!(board.pedals[0].path, "tube_screamer.pedal");
        assert!(board.pedals[0].overrides.is_empty());
        assert_eq!(board.pedals[1].id, "bd");
        assert_eq!(board.pedals[1].path, "blues_driver.pedal");
    }

    #[test]
    fn parse_board_with_overrides() {
        let src = r#"
board "Test" {
  ts: "tube_screamer.pedal" { Drive = 0.6, Level = 0.7 }
  bd: "blues_driver.pedal" { Gain = 0.5 }
}
"#;
        let board = parse_board_file(src).unwrap();
        assert_eq!(board.pedals.len(), 2);
        assert_eq!(board.pedals[0].overrides.len(), 2);
        assert_eq!(board.pedals[0].overrides[0], ("Drive".to_string(), 0.6));
        assert_eq!(board.pedals[0].overrides[1], ("Level".to_string(), 0.7));
        assert_eq!(board.pedals[1].overrides.len(), 1);
        assert_eq!(board.pedals[1].overrides[0], ("Gain".to_string(), 0.5));
    }

    #[test]
    fn parse_board_with_comments() {
        let src = r#"
# My awesome board
board "Stage Rig" {
  # Compressor first
  comp: "dyna_comp.pedal"
  # Then overdrive
  ts: "tube_screamer.pedal" { Drive = 0.8 }
}
"#;
        let board = parse_board_file(src).unwrap();
        assert_eq!(board.name, "Stage Rig");
        assert_eq!(board.pedals.len(), 2);
        assert_eq!(board.pedals[0].id, "comp");
        assert_eq!(board.pedals[1].id, "ts");
        assert_eq!(board.pedals[1].overrides[0].1, 0.8);
    }

    #[test]
    fn parse_empty_board() {
        let src = r#"board "Empty" {}"#;
        let board = parse_board_file(src).unwrap();
        assert_eq!(board.name, "Empty");
        assert!(board.pedals.is_empty());
    }

    #[test]
    fn parse_board_trailing_input_error() {
        let src = r#"board "X" {} garbage"#;
        assert!(parse_board_file(src).is_err());
    }

    #[test]
    fn parse_single_pedal_board() {
        let src = r#"
board "Solo" {
  od: "tube_screamer.pedal" { Drive = 1.0, Level = 0.5 }
}
"#;
        let board = parse_board_file(src).unwrap();
        assert_eq!(board.pedals.len(), 1);
        assert_eq!(board.pedals[0].id, "od");
        assert_eq!(board.pedals[0].overrides.len(), 2);
    }

    // ── Global board tests ──────────────────────────────────────────────

    #[test]
    fn parse_global_board_both_sections() {
        let src = r#"
# Global board
global {
  start {
    gate: "noise_gate.pedal" { Threshold = 0.3 }
  }
  end {
    amp: "clean_fender.pedal" { Volume = 0.7 }
    cab: "cabinet.pedal"
  }
}
"#;
        let def = parse_global_board_file(src).unwrap();
        assert_eq!(def.start.len(), 1);
        assert_eq!(def.start[0].id, "gate");
        assert_eq!(def.start[0].path, "noise_gate.pedal");
        assert_eq!(def.start[0].overrides, vec![("Threshold".to_string(), 0.3)]);
        assert_eq!(def.end.len(), 2);
        assert_eq!(def.end[0].id, "amp");
        assert_eq!(def.end[1].id, "cab");
        assert!(def.end[1].overrides.is_empty());
    }

    #[test]
    fn parse_global_board_start_only() {
        let src = r#"
global {
  start {
    comp: "dyna_comp.pedal"
  }
}
"#;
        let def = parse_global_board_file(src).unwrap();
        assert_eq!(def.start.len(), 1);
        assert!(def.end.is_empty());
    }

    #[test]
    fn parse_global_board_end_only() {
        let src = r#"
global {
  end {
    amp: "clean_fender.pedal" { Volume = 0.5 }
  }
}
"#;
        let def = parse_global_board_file(src).unwrap();
        assert!(def.start.is_empty());
        assert_eq!(def.end.len(), 1);
    }

    #[test]
    fn parse_global_board_empty() {
        let src = "global {}";
        let def = parse_global_board_file(src).unwrap();
        assert!(def.start.is_empty());
        assert!(def.end.is_empty());
    }

    #[test]
    fn parse_global_board_reversed_order() {
        let src = r#"
global {
  end {
    amp: "clean_fender.pedal"
  }
  start {
    gate: "noise_gate.pedal"
  }
}
"#;
        let def = parse_global_board_file(src).unwrap();
        assert_eq!(def.start.len(), 1);
        assert_eq!(def.start[0].id, "gate");
        assert_eq!(def.end.len(), 1);
        assert_eq!(def.end[0].id, "amp");
    }

    #[test]
    fn find_global_board_in_same_dir() {
        use std::io::Write;
        let tmp = tempfile::tempdir().unwrap();
        let global_path = tmp.path().join(".global.board");
        let mut f = std::fs::File::create(&global_path).unwrap();
        writeln!(f, "global {{ start {{ g: \"gate.pedal\" }} }}").unwrap();

        let result = find_global_board(tmp.path());
        assert!(result.is_some());
        let (def, dir) = result.unwrap();
        assert_eq!(def.start.len(), 1);
        assert_eq!(def.start[0].id, "g");
        assert_eq!(dir, tmp.path().canonicalize().unwrap());
    }

    #[test]
    fn find_global_board_walks_up() {
        use std::io::Write;
        let tmp = tempfile::tempdir().unwrap();
        // Put .global.board in root
        let global_path = tmp.path().join(".global.board");
        let mut f = std::fs::File::create(&global_path).unwrap();
        writeln!(f, "global {{ end {{ a: \"amp.pedal\" }} }}").unwrap();

        // Create a subdirectory
        let sub = tmp.path().join("boards");
        std::fs::create_dir(&sub).unwrap();

        let result = find_global_board(&sub);
        assert!(result.is_some());
        let (def, dir) = result.unwrap();
        assert_eq!(def.end.len(), 1);
        assert_eq!(dir, tmp.path().canonicalize().unwrap());
    }

    #[test]
    fn find_global_board_none() {
        let tmp = tempfile::tempdir().unwrap();
        let result = find_global_board(tmp.path());
        assert!(result.is_none());
    }
}
