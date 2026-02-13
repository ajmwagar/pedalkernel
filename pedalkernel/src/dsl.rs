//! DSL parser for `.pedal` circuit definition files.
//!
//! Parses component declarations, net connections, and control mappings
//! into an AST that can drive WDF tree construction or KiCad export.

use nom::{
    branch::alt,
    bytes::complete::{tag, take_while, take_while1},
    character::complete::{char, multispace1, not_line_ending},
    combinator::{opt, recognize, value},
    multi::{many0, separated_list1},
    number::complete::double,
    sequence::{delimited, pair, preceded, tuple},
    IResult,
};

// ---------------------------------------------------------------------------
// AST
// ---------------------------------------------------------------------------

/// Top-level pedal definition.
#[derive(Debug, Clone, PartialEq)]
pub struct PedalDef {
    pub name: String,
    pub components: Vec<ComponentDef>,
    pub nets: Vec<NetDef>,
    pub controls: Vec<ControlDef>,
}

/// A single component declaration, e.g. `R1: resistor(4.7k)`
#[derive(Debug, Clone, PartialEq)]
pub struct ComponentDef {
    pub id: String,
    pub kind: ComponentKind,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ComponentKind {
    Resistor(f64),
    Capacitor(f64),
    Inductor(f64),
    DiodePair(DiodeType),
    Diode(DiodeType),
    Potentiometer(f64),
    Npn,
    Pnp,
    OpAmp,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiodeType {
    Silicon,
    Germanium,
    Led,
}

/// A net connection: `in -> C1.a` or `C1.b -> R1.a, D1.a`
#[derive(Debug, Clone, PartialEq)]
pub struct NetDef {
    pub from: Pin,
    pub to: Vec<Pin>,
}

/// A pin reference – either a reserved node (`in`, `out`, `gnd`, `vcc`)
/// or a component pin (`C1.a`, `R1.b`).
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum Pin {
    Reserved(String),
    ComponentPin { component: String, pin: String },
}

/// Control mapping: `Gain.position -> "Drive" [0.0, 1.0] = 0.5`
#[derive(Debug, Clone, PartialEq)]
pub struct ControlDef {
    pub component: String,
    pub property: String,
    pub label: String,
    pub range: (f64, f64),
    pub default: f64,
}

// ---------------------------------------------------------------------------
// Helpers
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

/// Engineering-notation multiplier suffix.
fn eng_suffix(input: &str) -> IResult<&str, f64> {
    alt((
        value(1e-12, tag("p")),
        value(1e-9, tag("n")),
        value(1e-6, tag("u")),
        value(1e3, tag("k")),
        value(1e6, tag("M")),
        value(1e-3, tag("m")), // milli – after 'M' to disambiguate
    ))(input)
}

/// Parse a number with optional engineering suffix, e.g. `4.7k`, `220n`, `100m`.
fn eng_value(input: &str) -> IResult<&str, f64> {
    let (input, num) = double(input)?;
    let (input, mult) = opt(eng_suffix)(input)?;
    Ok((input, num * mult.unwrap_or(1.0)))
}

/// Quoted string: `"Foo Bar"`
fn quoted_string(input: &str) -> IResult<&str, &str> {
    delimited(char('"'), take_while(|c: char| c != '"'), char('"'))(input)
}

// ---------------------------------------------------------------------------
// Component parsers
// ---------------------------------------------------------------------------

fn parse_resistor(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("resistor")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, val) = eng_value(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Resistor(val)))
}

fn parse_cap(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("cap")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, val) = eng_value(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Capacitor(val)))
}

fn parse_inductor(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("inductor")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, val) = eng_value(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Inductor(val)))
}

fn diode_type(input: &str) -> IResult<&str, DiodeType> {
    alt((
        value(DiodeType::Silicon, tag("silicon")),
        value(DiodeType::Germanium, tag("germanium")),
        value(DiodeType::Led, tag("led")),
    ))(input)
}

fn parse_diode_pair(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("diode_pair")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, dt) = diode_type(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::DiodePair(dt)))
}

fn parse_diode(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("diode")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, dt) = diode_type(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Diode(dt)))
}

fn parse_pot(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("pot")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, val) = eng_value(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Potentiometer(val)))
}

fn parse_npn(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("npn")(input)?;
    let (input, _) = tag("()")(input)?;
    Ok((input, ComponentKind::Npn))
}

fn parse_pnp(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("pnp")(input)?;
    let (input, _) = tag("()")(input)?;
    Ok((input, ComponentKind::Pnp))
}

fn parse_opamp(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("opamp")(input)?;
    let (input, _) = tag("()")(input)?;
    Ok((input, ComponentKind::OpAmp))
}

fn component_kind(input: &str) -> IResult<&str, ComponentKind> {
    alt((
        parse_resistor,
        parse_cap,
        parse_inductor,
        parse_diode_pair, // must come before parse_diode
        parse_diode,
        parse_pot,
        parse_npn,
        parse_pnp,
        parse_opamp,
    ))(input)
}

/// `R1: resistor(4.7k)`
fn component_def(input: &str) -> IResult<&str, ComponentDef> {
    let (input, _) = ws_comments(input)?;
    let (input, id) = identifier(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(':')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, kind) = component_kind(input)?;
    Ok((
        input,
        ComponentDef {
            id: id.to_string(),
            kind,
        },
    ))
}

// ---------------------------------------------------------------------------
// Net parsers
// ---------------------------------------------------------------------------

const RESERVED_NODES: &[&str] = &["in", "out", "gnd", "vcc"];

fn pin(input: &str) -> IResult<&str, Pin> {
    let (input, first) = identifier(input)?;
    let (input, dot_pin) = opt(preceded(char('.'), identifier))(input)?;
    match dot_pin {
        Some(p) => Ok((
            input,
            Pin::ComponentPin {
                component: first.to_string(),
                pin: p.to_string(),
            },
        )),
        None => {
            if RESERVED_NODES.contains(&first) {
                Ok((input, Pin::Reserved(first.to_string())))
            } else {
                // Bare component name treated as reserved-style node
                Ok((input, Pin::Reserved(first.to_string())))
            }
        }
    }
}

/// `in -> C1.a`  or  `C1.b -> R1.a, D1.a`
fn net_def(input: &str) -> IResult<&str, NetDef> {
    let (input, _) = ws_comments(input)?;
    let (input, from) = pin(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = tag("->")(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, to) = separated_list1(tuple((ws_comments, char(','), ws_comments)), pin)(input)?;
    Ok((input, NetDef { from, to }))
}

// ---------------------------------------------------------------------------
// Control parsers
// ---------------------------------------------------------------------------

/// `Gain.position -> "Drive" [0.0, 1.0] = 0.5`
fn control_def(input: &str) -> IResult<&str, ControlDef> {
    let (input, _) = ws_comments(input)?;
    let (input, comp) = identifier(input)?;
    let (input, _) = char('.')(input)?;
    let (input, prop) = identifier(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = tag("->")(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, label) = quoted_string(input)?;
    let (input, _) = ws_comments(input)?;
    // [min, max]
    let (input, _) = char('[')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, lo) = double(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(',')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, hi) = double(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(']')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('=')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, def) = double(input)?;
    Ok((
        input,
        ControlDef {
            component: comp.to_string(),
            property: prop.to_string(),
            label: label.to_string(),
            range: (lo, hi),
            default: def,
        },
    ))
}

// ---------------------------------------------------------------------------
// Section parsers
// ---------------------------------------------------------------------------

fn components_section(input: &str) -> IResult<&str, Vec<ComponentDef>> {
    let (input, _) = ws_comments(input)?;
    let (input, _) = tag("components")(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('{')(input)?;
    let (input, comps) = many0(component_def)(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('}')(input)?;
    Ok((input, comps))
}

fn nets_section(input: &str) -> IResult<&str, Vec<NetDef>> {
    let (input, _) = ws_comments(input)?;
    let (input, _) = tag("nets")(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('{')(input)?;
    let (input, nets) = many0(net_def)(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('}')(input)?;
    Ok((input, nets))
}

fn controls_section(input: &str) -> IResult<&str, Vec<ControlDef>> {
    let (input, _) = ws_comments(input)?;
    let (input, _) = tag("controls")(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('{')(input)?;
    let (input, ctrls) = many0(control_def)(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('}')(input)?;
    Ok((input, ctrls))
}

// ---------------------------------------------------------------------------
// Top-level
// ---------------------------------------------------------------------------

/// Parse a complete `.pedal` file.
pub fn parse_pedal(input: &str) -> IResult<&str, PedalDef> {
    let (input, _) = ws_comments(input)?;
    let (input, _) = tag("pedal")(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, name) = quoted_string(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('{')(input)?;

    let (input, components) = components_section(input)?;
    let (input, nets) = nets_section(input)?;
    let (input, controls) = opt(controls_section)(input)?;

    let (input, _) = ws_comments(input)?;
    let (input, _) = char('}')(input)?;
    let (input, _) = ws_comments(input)?;

    Ok((
        input,
        PedalDef {
            name: name.to_string(),
            components,
            nets,
            controls: controls.unwrap_or_default(),
        },
    ))
}

/// Convenience wrapper that returns `Result`.
pub fn parse_pedal_file(src: &str) -> Result<PedalDef, String> {
    match parse_pedal(src) {
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
    fn parse_eng_value_k() {
        let (_, v) = eng_value("4.7k").unwrap();
        assert!((v - 4700.0).abs() < 1e-6);
    }

    #[test]
    fn parse_eng_value_n() {
        let (_, v) = eng_value("220n").unwrap();
        assert!((v - 220e-9).abs() < 1e-18);
    }

    #[test]
    fn parse_eng_value_m() {
        let (_, v) = eng_value("100m").unwrap();
        assert!((v - 0.1).abs() < 1e-10);
    }

    #[test]
    #[allow(non_snake_case)]
    fn parse_eng_value_M() {
        let (_, v) = eng_value("1M").unwrap();
        assert!((v - 1e6).abs() < 1e-2);
    }

    #[test]
    fn parse_component_resistor() {
        let (_, c) = component_def("R1: resistor(4.7k)").unwrap();
        assert_eq!(c.id, "R1");
        assert_eq!(c.kind, ComponentKind::Resistor(4700.0));
    }

    #[test]
    fn parse_component_cap() {
        let (_, c) = component_def("C1: cap(220n)").unwrap();
        assert_eq!(c.id, "C1");
        if let ComponentKind::Capacitor(v) = c.kind {
            assert!((v - 220e-9).abs() < 1e-18);
        } else {
            panic!("expected Capacitor");
        }
    }

    #[test]
    fn parse_component_diode_pair() {
        let (_, c) = component_def("D1: diode_pair(silicon)").unwrap();
        assert_eq!(c.kind, ComponentKind::DiodePair(DiodeType::Silicon));
    }

    #[test]
    fn parse_component_pot() {
        let (_, c) = component_def("Gain: pot(500k)").unwrap();
        assert_eq!(c.kind, ComponentKind::Potentiometer(500_000.0));
    }

    #[test]
    fn parse_net_simple() {
        let (_, n) = net_def("in -> C1.a").unwrap();
        assert_eq!(n.from, Pin::Reserved("in".to_string()));
        assert_eq!(
            n.to,
            vec![Pin::ComponentPin {
                component: "C1".to_string(),
                pin: "a".to_string()
            }]
        );
    }

    #[test]
    fn parse_net_multi() {
        let (_, n) = net_def("C1.b -> R1.a, D1.a").unwrap();
        assert_eq!(n.to.len(), 2);
    }

    #[test]
    fn parse_control() {
        let (_, c) = control_def(r#"Gain.position -> "Drive" [0.0, 1.0] = 0.5"#).unwrap();
        assert_eq!(c.component, "Gain");
        assert_eq!(c.label, "Drive");
        assert!((c.default - 0.5).abs() < 1e-10);
    }

    #[test]
    fn parse_full_pedal() {
        let src = r#"
pedal "Tube Screamer" {
  components {
    R1: resistor(4.7k)
    C1: cap(220n)
    D1: diode_pair(silicon)
    Gain: pot(500k)
  }
  nets {
    in -> C1.a
    C1.b -> R1.a, D1.a
    D1.b -> gnd
  }
  controls {
    Gain.position -> "Drive" [0.0, 1.0] = 0.5
  }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert_eq!(def.name, "Tube Screamer");
        assert_eq!(def.components.len(), 4);
        assert_eq!(def.nets.len(), 3);
        assert_eq!(def.controls.len(), 1);
    }

    #[test]
    fn parse_with_comments() {
        let src = r#"
# This is a fuzz pedal
pedal "Fuzz" {
  components {
    # Input cap
    C1: cap(100n)
    Q1: npn()
  }
  nets {
    in -> C1.a
    C1.b -> Q1.base
  }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert_eq!(def.name, "Fuzz");
        assert_eq!(def.components.len(), 2);
    }

    #[test]
    fn parse_all_component_kinds() {
        let src = r#"
pedal "All" {
  components {
    R1: resistor(10k)
    C1: cap(100n)
    L1: inductor(100m)
    D1: diode_pair(germanium)
    D2: diode(led)
    P1: pot(1M)
    Q1: npn()
    Q2: pnp()
    U1: opamp()
  }
  nets {
    in -> out
  }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert_eq!(def.components.len(), 9);
    }
}
