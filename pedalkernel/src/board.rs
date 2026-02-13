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
        Ok((rest, _)) => Err(format!(
            "Trailing input: {:?}",
            &rest[..rest.len().min(60)]
        )),
        Err(e) => Err(format!("Parse error: {e}")),
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
}
