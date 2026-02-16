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
    NJfet(JfetType),
    PJfet(JfetType),
    Photocoupler(PhotocouplerType),
    /// LFO: waveform, timing_r (Ω), timing_c (F) -> f = 1/(2πRC)
    Lfo(LfoWaveformDsl, f64, f64),
    /// Triode vacuum tube
    Triode(TriodeType),
    /// Envelope follower: attack_r (Ω), attack_c (F), release_r (Ω), release_c (F), sensitivity_r (Ω)
    /// Attack τ = attack_r × attack_c, Release τ = release_r × release_c
    /// Sensitivity = sensitivity_r / 10kΩ
    EnvelopeFollower(f64, f64, f64, f64, f64),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiodeType {
    Silicon,
    Germanium,
    Led,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JfetType {
    J201,
    N2n5457,
    P2n5460,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhotocouplerType {
    Vtl5c3,
    Vtl5c1,
    Nsl32,
}

/// LFO waveform shapes for DSL.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LfoWaveformDsl {
    Sine,
    Triangle,
    Square,
    SawUp,
    SawDown,
    SampleAndHold,
}

/// Triode vacuum tube types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriodeType {
    /// 12AX7 / ECC83 - High gain preamp tube
    T12ax7,
    /// 12AT7 / ECC81 - Medium gain
    T12at7,
    /// 12AU7 / ECC82 - Low gain
    T12au7,
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

fn jfet_type(input: &str) -> IResult<&str, JfetType> {
    alt((
        value(JfetType::J201, tag("j201")),
        value(JfetType::N2n5457, tag("2n5457")),
        value(JfetType::P2n5460, tag("2n5460")),
    ))(input)
}

fn parse_njfet(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("njfet")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, jt) = jfet_type(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::NJfet(jt)))
}

fn parse_pjfet(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("pjfet")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, jt) = jfet_type(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::PJfet(jt)))
}

fn photocoupler_type(input: &str) -> IResult<&str, PhotocouplerType> {
    alt((
        value(PhotocouplerType::Vtl5c3, tag("vtl5c3")),
        value(PhotocouplerType::Vtl5c1, tag("vtl5c1")),
        value(PhotocouplerType::Nsl32, tag("nsl32")),
    ))(input)
}

fn parse_photocoupler(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("photocoupler")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, pt) = photocoupler_type(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Photocoupler(pt)))
}

fn triode_type(input: &str) -> IResult<&str, TriodeType> {
    alt((
        value(TriodeType::T12ax7, tag("12ax7")),
        value(TriodeType::T12at7, tag("12at7")),
        value(TriodeType::T12au7, tag("12au7")),
        // European designations
        value(TriodeType::T12ax7, tag("ecc83")),
        value(TriodeType::T12at7, tag("ecc81")),
        value(TriodeType::T12au7, tag("ecc82")),
    ))(input)
}

fn parse_triode(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("triode")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, tt) = triode_type(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Triode(tt)))
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
        parse_njfet,
        parse_pjfet,
        parse_photocoupler,
        parse_envelope_follower, // must come before parse_lfo (both are long keywords)
        parse_lfo,
        parse_triode,
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

fn lfo_waveform(input: &str) -> IResult<&str, LfoWaveformDsl> {
    alt((
        value(LfoWaveformDsl::Sine, tag("sine")),
        value(LfoWaveformDsl::Triangle, tag("triangle")),
        value(LfoWaveformDsl::Square, tag("square")),
        value(LfoWaveformDsl::SawUp, tag("saw_up")),
        value(LfoWaveformDsl::SawDown, tag("saw_down")),
        value(LfoWaveformDsl::SampleAndHold, tag("sample_hold")),
    ))(input)
}

/// `envelope_follower(1k, 4.7u, 100k, 1u, 20k)` - attack_r, attack_c, release_r, release_c, sensitivity_r
/// Attack τ = R_attack × C_attack, Release τ = R_release × C_release
/// Sensitivity gain = R_sensitivity / 10kΩ
fn parse_envelope_follower(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("envelope_follower")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, attack_r) = eng_value(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(',')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, attack_c) = eng_value(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(',')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, release_r) = eng_value(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(',')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, release_c) = eng_value(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(',')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, sensitivity_r) = eng_value(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(')')(input)?;
    Ok((
        input,
        ComponentKind::EnvelopeFollower(attack_r, attack_c, release_r, release_c, sensitivity_r),
    ))
}

/// `lfo(triangle, 100k, 220n)` - waveform, timing_r, timing_c
/// Frequency is computed as f = 1/(2πRC)
fn parse_lfo(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("lfo")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, waveform) = lfo_waveform(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(',')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, timing_r) = eng_value(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(',')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, timing_c) = eng_value(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Lfo(waveform, timing_r, timing_c)))
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

    #[test]
    fn parse_njfet_j201() {
        let (_, c) = component_def("J1: njfet(j201)").unwrap();
        assert_eq!(c.id, "J1");
        assert_eq!(c.kind, ComponentKind::NJfet(JfetType::J201));
    }

    #[test]
    fn parse_njfet_2n5457() {
        let (_, c) = component_def("J2: njfet(2n5457)").unwrap();
        assert_eq!(c.kind, ComponentKind::NJfet(JfetType::N2n5457));
    }

    #[test]
    fn parse_pjfet_2n5460() {
        let (_, c) = component_def("J3: pjfet(2n5460)").unwrap();
        assert_eq!(c.kind, ComponentKind::PJfet(JfetType::P2n5460));
    }

    #[test]
    fn parse_pedal_with_jfet() {
        let src = r#"
pedal "Tremolo" {
  components {
    C1: cap(100n)
    J1: njfet(j201)
    R1: resistor(10k)
  }
  nets {
    in -> C1.a
    C1.b -> J1.gate
    J1.drain -> R1.a
    J1.source -> gnd
  }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert_eq!(def.name, "Tremolo");
        assert_eq!(def.components.len(), 3);
        assert_eq!(def.components[1].kind, ComponentKind::NJfet(JfetType::J201));
    }

    #[test]
    fn parse_photocoupler_vtl5c3() {
        let (_, c) = component_def("OC1: photocoupler(vtl5c3)").unwrap();
        assert_eq!(c.id, "OC1");
        assert_eq!(
            c.kind,
            ComponentKind::Photocoupler(PhotocouplerType::Vtl5c3)
        );
    }

    #[test]
    fn parse_photocoupler_vtl5c1() {
        let (_, c) = component_def("OC2: photocoupler(vtl5c1)").unwrap();
        assert_eq!(
            c.kind,
            ComponentKind::Photocoupler(PhotocouplerType::Vtl5c1)
        );
    }

    #[test]
    fn parse_photocoupler_nsl32() {
        let (_, c) = component_def("OC3: photocoupler(nsl32)").unwrap();
        assert_eq!(c.kind, ComponentKind::Photocoupler(PhotocouplerType::Nsl32));
    }

    #[test]
    fn parse_pedal_with_photocoupler() {
        let src = r#"
pedal "Optical Tremolo" {
  components {
    C1: cap(100n)
    OC1: photocoupler(vtl5c3)
    R1: resistor(10k)
  }
  nets {
    in -> C1.a
    C1.b -> OC1.a
    OC1.b -> R1.a
    R1.b -> out
  }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert_eq!(def.name, "Optical Tremolo");
        assert_eq!(def.components.len(), 3);
        assert_eq!(
            def.components[1].kind,
            ComponentKind::Photocoupler(PhotocouplerType::Vtl5c3)
        );
    }

    #[test]
    fn parse_triode_12ax7() {
        let (_, c) = component_def("V1: triode(12ax7)").unwrap();
        assert_eq!(c.id, "V1");
        assert_eq!(c.kind, ComponentKind::Triode(TriodeType::T12ax7));
    }

    #[test]
    fn parse_triode_12at7() {
        let (_, c) = component_def("V2: triode(12at7)").unwrap();
        assert_eq!(c.kind, ComponentKind::Triode(TriodeType::T12at7));
    }

    #[test]
    fn parse_triode_12au7() {
        let (_, c) = component_def("V3: triode(12au7)").unwrap();
        assert_eq!(c.kind, ComponentKind::Triode(TriodeType::T12au7));
    }

    #[test]
    fn parse_triode_ecc83() {
        // ECC83 = European 12AX7
        let (_, c) = component_def("V4: triode(ecc83)").unwrap();
        assert_eq!(c.kind, ComponentKind::Triode(TriodeType::T12ax7));
    }

    #[test]
    fn parse_lfo_component() {
        // lfo(triangle, 100k, 220n) -> f = 1/(2π*100000*220e-9) ≈ 7.2 Hz
        let (_, c) = component_def("LFO1: lfo(triangle, 100k, 220n)").unwrap();
        assert_eq!(c.id, "LFO1");
        if let ComponentKind::Lfo(waveform, timing_r, timing_c) = c.kind {
            assert_eq!(waveform, LfoWaveformDsl::Triangle);
            assert!((timing_r - 100_000.0).abs() < 1.0);
            assert!((timing_c - 220e-9).abs() < 1e-12);
        } else {
            panic!("expected Lfo");
        }
    }

    #[test]
    fn parse_lfo_waveforms() {
        assert_eq!(lfo_waveform("sine").unwrap().1, LfoWaveformDsl::Sine);
        assert_eq!(
            lfo_waveform("triangle").unwrap().1,
            LfoWaveformDsl::Triangle
        );
        assert_eq!(lfo_waveform("square").unwrap().1, LfoWaveformDsl::Square);
        assert_eq!(lfo_waveform("saw_up").unwrap().1, LfoWaveformDsl::SawUp);
        assert_eq!(lfo_waveform("saw_down").unwrap().1, LfoWaveformDsl::SawDown);
        assert_eq!(
            lfo_waveform("sample_hold").unwrap().1,
            LfoWaveformDsl::SampleAndHold
        );
    }

    #[test]
    fn parse_pedal_with_lfo_component() {
        let src = r#"
pedal "Harmonic Tremolo" {
  components {
    LFO1: lfo(triangle, 100k, 220n)
    C1: cap(100n)
    J1: njfet(j201)
    R1: resistor(10k)
  }
  nets {
    in -> C1.a
    C1.b -> J1.drain
    J1.source -> gnd
    LFO1.out -> J1.vgs
  }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert_eq!(def.name, "Harmonic Tremolo");
        assert_eq!(def.components.len(), 4);
        // Check LFO component: 100k timing resistor, 220n timing cap
        if let ComponentKind::Lfo(waveform, timing_r, timing_c) = &def.components[0].kind {
            assert_eq!(*waveform, LfoWaveformDsl::Triangle);
            assert!((*timing_r - 100_000.0).abs() < 1.0);
            assert!((*timing_c - 220e-9).abs() < 1e-12);
        } else {
            panic!("expected Lfo component");
        }
        // Check LFO.out net connection
        let lfo_net = def.nets.iter().find(|n| {
            matches!(&n.from, Pin::ComponentPin { component, pin }
                     if component == "LFO1" && pin == "out")
        });
        assert!(lfo_net.is_some(), "should have LFO1.out net");
    }

    #[test]
    fn parse_envelope_follower_component() {
        // envelope_follower(1k, 4.7u, 100k, 1u, 20k)
        let (_, c) = component_def("EF1: envelope_follower(1k, 4.7u, 100k, 1u, 20k)").unwrap();
        assert_eq!(c.id, "EF1");
        if let ComponentKind::EnvelopeFollower(attack_r, attack_c, release_r, release_c, sens_r) =
            c.kind
        {
            assert!((attack_r - 1_000.0).abs() < 1.0);
            assert!((attack_c - 4.7e-6).abs() < 1e-9);
            assert!((release_r - 100_000.0).abs() < 1.0);
            assert!((release_c - 1e-6).abs() < 1e-9);
            assert!((sens_r - 20_000.0).abs() < 1.0);
        } else {
            panic!("expected EnvelopeFollower");
        }
    }

    #[test]
    fn parse_pedal_with_envelope_follower() {
        let src = r#"
pedal "Auto Wah" {
  components {
    EF1: envelope_follower(1k, 4.7u, 100k, 1.5u, 20k)
    C1: cap(100n)
    J1: njfet(j201)
    R1: resistor(10k)
  }
  nets {
    in -> C1.a
    C1.b -> J1.drain
    J1.source -> gnd
    EF1.out -> J1.vgs
    R1.a -> J1.drain
    R1.b -> out
  }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert_eq!(def.name, "Auto Wah");
        assert_eq!(def.components.len(), 4);
        // Check EnvelopeFollower component
        if let ComponentKind::EnvelopeFollower(attack_r, attack_c, release_r, release_c, sens_r) =
            &def.components[0].kind
        {
            assert!((*attack_r - 1_000.0).abs() < 1.0);
            assert!((*attack_c - 4.7e-6).abs() < 1e-9);
            assert!((*release_r - 100_000.0).abs() < 1.0);
            assert!((*release_c - 1.5e-6).abs() < 1e-9);
            assert!((*sens_r - 20_000.0).abs() < 1.0);
        } else {
            panic!("expected EnvelopeFollower component");
        }
        // Check EF1.out net connection
        let ef_net = def.nets.iter().find(|n| {
            matches!(&n.from, Pin::ComponentPin { component, pin }
                     if component == "EF1" && pin == "out")
        });
        assert!(ef_net.is_some(), "should have EF1.out net");
    }
}
