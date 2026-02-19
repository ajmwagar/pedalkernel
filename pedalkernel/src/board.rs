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
    pub entries: Vec<BoardEntry>,
}

impl BoardDef {
    /// Flat list of all pedal entries (for backward compatibility).
    /// FX loop entries are flattened into the list.
    pub fn pedals(&self) -> Vec<&BoardPedalEntry> {
        let mut out = Vec::new();
        for entry in &self.entries {
            match entry {
                BoardEntry::Pedal(p) => out.push(p),
                BoardEntry::FxLoop(group) => {
                    for p in &group.pedals {
                        out.push(p);
                    }
                }
            }
        }
        out
    }
}

/// An entry in the board signal chain — either a single pedal or an FX loop group.
#[derive(Debug, Clone, PartialEq)]
pub enum BoardEntry {
    /// A single pedal in the chain.
    Pedal(BoardPedalEntry),
    /// An FX send/return loop containing a sub-chain of pedals.
    FxLoop(FxLoopGroup),
}

/// An FX send/return loop group.
///
/// References a pedal that has `fx_send`/`fx_return` in its circuit.
/// The pedal is split at that boundary: everything before `fx_send` runs first,
/// then the FX loop effects chain, then everything after `fx_return`.
#[derive(Debug, Clone, PartialEq)]
pub struct FxLoopGroup {
    /// ID of the pedal that provides the send/return (e.g. "amp").
    /// This pedal must have `fx_send` and `fx_return` nodes in its .pedal file.
    pub target_id: String,
    /// Pedals inside the FX loop (processed in order between send and return).
    pub pedals: Vec<BoardPedalEntry>,
    /// Wet/dry mix (0.0 = fully dry/bypass, 1.0 = fully wet). Defaults to 1.0.
    pub mix: f64,
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
// FX loop parser
// ---------------------------------------------------------------------------

/// Parse an FX loop group: `fx_loop(amp_id) { ... }`
///
/// The `amp_id` references a pedal entry earlier in the board that has
/// `fx_send`/`fx_return` nodes. Optionally: `fx_loop(amp_id, 0.5)` for wet/dry mix.
fn fx_loop_entry(input: &str) -> IResult<&str, BoardEntry> {
    let (input, _) = ws_comments(input)?;
    let (input, _) = tag("fx_loop")(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, target_id) = identifier(input)?;
    // Optional mix parameter after comma
    let (input, mix) = opt(|i| {
        let (i, _) = ws_comments(i)?;
        let (i, _) = char(',')(i)?;
        let (i, _) = ws_comments(i)?;
        let (i, val) = double(i)?;
        Ok((i, val))
    })(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(')')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('{')(input)?;
    let (input, pedals) = many0(pedal_entry)(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('}')(input)?;
    Ok((
        input,
        BoardEntry::FxLoop(FxLoopGroup {
            target_id: target_id.to_string(),
            pedals,
            mix: mix.unwrap_or(1.0),
        }),
    ))
}

/// Parse a board entry — either a pedal or an FX loop.
fn board_entry(input: &str) -> IResult<&str, BoardEntry> {
    alt((fx_loop_entry, |i| {
        let (i, entry) = pedal_entry(i)?;
        Ok((i, BoardEntry::Pedal(entry)))
    }))(input)
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
    let (input, entries) = many0(board_entry)(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('}')(input)?;
    let (input, _) = ws_comments(input)?;
    Ok((
        input,
        BoardDef {
            name: name.to_string(),
            entries,
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
        assert_eq!(board.pedals().len(), 2);
        assert_eq!(board.pedals()[0].id, "ts");
        assert_eq!(board.pedals()[0].path, "tube_screamer.pedal");
        assert!(board.pedals()[0].overrides.is_empty());
        assert_eq!(board.pedals()[1].id, "bd");
        assert_eq!(board.pedals()[1].path, "blues_driver.pedal");
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
        assert_eq!(board.pedals().len(), 2);
        assert_eq!(board.pedals()[0].overrides.len(), 2);
        assert_eq!(board.pedals()[0].overrides[0], ("Drive".to_string(), 0.6));
        assert_eq!(board.pedals()[0].overrides[1], ("Level".to_string(), 0.7));
        assert_eq!(board.pedals()[1].overrides.len(), 1);
        assert_eq!(board.pedals()[1].overrides[0], ("Gain".to_string(), 0.5));
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
        assert_eq!(board.pedals().len(), 2);
        assert_eq!(board.pedals()[0].id, "comp");
        assert_eq!(board.pedals()[1].id, "ts");
        assert_eq!(board.pedals()[1].overrides[0].1, 0.8);
    }

    #[test]
    fn parse_empty_board() {
        let src = r#"board "Empty" {}"#;
        let board = parse_board_file(src).unwrap();
        assert_eq!(board.name, "Empty");
        assert!(board.pedals().is_empty());
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
        assert_eq!(board.pedals().len(), 1);
        assert_eq!(board.pedals()[0].id, "od");
        assert_eq!(board.pedals()[0].overrides.len(), 2);
    }

    #[test]
    fn parse_board_with_fx_loop() {
        let src = r#"
board "FX Loop Test" {
  ts: "tube_screamer.pedal"
  amp: "tweed_deluxe.pedal" { Volume = 0.7 }
  fx_loop(amp) {
    delay: "delay.pedal"
    chorus: "chorus.pedal"
  }
}
"#;
        let board = parse_board_file(src).unwrap();
        assert_eq!(board.name, "FX Loop Test");
        assert_eq!(board.entries.len(), 3);
        // First entry: pedal
        assert!(matches!(&board.entries[0], BoardEntry::Pedal(p) if p.id == "ts"));
        // Second entry: pedal (amp)
        assert!(matches!(&board.entries[1], BoardEntry::Pedal(p) if p.id == "amp"));
        // Third entry: FX loop
        if let BoardEntry::FxLoop(group) = &board.entries[2] {
            assert_eq!(group.target_id, "amp");
            assert_eq!(group.pedals.len(), 2);
            assert_eq!(group.pedals[0].id, "delay");
            assert_eq!(group.pedals[1].id, "chorus");
            assert!((group.mix - 1.0).abs() < f64::EPSILON); // default mix
        } else {
            panic!("Expected FxLoop entry");
        }
        // Flat pedals should include amp + ts + fx loop contents
        assert_eq!(board.pedals().len(), 4);
    }

    #[test]
    fn parse_board_fx_loop_with_mix() {
        let src = r#"
board "Mix Test" {
  amp: "amp.pedal"
  fx_loop(amp, 0.5) {
    delay: "delay.pedal"
  }
}
"#;
        let board = parse_board_file(src).unwrap();
        if let BoardEntry::FxLoop(group) = &board.entries[1] {
            assert!((group.mix - 0.5).abs() < f64::EPSILON);
        } else {
            panic!("Expected FxLoop entry");
        }
    }
}
