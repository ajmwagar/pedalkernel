//! Hardware limits for physical pedal builds.
//!
//! When prototyping a pedal tone digitally with the intent to build it in
//! real life, the component specs matter.  A 10 µF electrolytic rated at
//! 10 V will pop at 12 V; a germanium AC128 transistor might not survive
//! 18 V.  This module lets hardware-minded users declare those limits in a
//! companion `.pedalhw` file — separate from the `.pedal` DSL so
//! tone-focused users never see it.
//!
//! # File format
//!
//! ```text
//! # fuzz_face.pedalhw — physical component specs
//!
//! Q1: vce_max(32) part("AC128")
//! Q2: vce_max(32) part("AC128")
//! C1: voltage_rating(25)
//! C2: voltage_rating(16)
//! U1: supply_max(36)
//! D1: breakdown(50)
//! ```
//!
//! Each line maps a component ID (matching the `.pedal` file) to one or
//! more hardware properties.  Unknown IDs are silently ignored so the
//! `.pedalhw` file can be a superset.
//!
//! # Feature gate
//!
//! This module is only compiled when the `hardware` feature is enabled:
//!
//! ```toml
//! pedalkernel = { version = "0.1", features = ["hardware"] }
//! ```

use std::collections::HashMap;

use nom::{
    branch::alt,
    bytes::complete::{tag, take_while, take_while1},
    character::complete::{char, multispace1, not_line_ending},
    combinator::value,
    multi::many0,
    number::complete::double,
    sequence::pair,
    IResult,
};

use crate::compiler::{VoltageWarning, WarningSeverity};
use crate::dsl::{ComponentKind, PedalDef};

// ═══════════════════════════════════════════════════════════════════════════
// Data model
// ═══════════════════════════════════════════════════════════════════════════

/// Per-component hardware spec.  All fields are optional — only specify
/// what you know about the physical part.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct HardwareSpec {
    /// Part number / model name, e.g. "AC128", "TL072", "1N4148".
    pub part: Option<String>,
    /// Transistor max collector-emitter voltage (volts).
    pub vce_max: Option<f64>,
    /// Capacitor voltage rating (volts).
    pub voltage_rating: Option<f64>,
    /// Op-amp max total supply voltage (volts), e.g. 36.0 for TL072 (±18V).
    pub supply_max: Option<f64>,
    /// Diode reverse breakdown voltage (volts).
    pub breakdown: Option<f64>,
    /// Resistor power rating (watts).
    pub power_rating: Option<f64>,
}

/// A map from component ID to hardware spec.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct HardwareLimits {
    pub specs: HashMap<String, HardwareSpec>,
}

// ═══════════════════════════════════════════════════════════════════════════
// Parser
// ═══════════════════════════════════════════════════════════════════════════

/// Consume whitespace and `# …` comments.
fn ws_comments(input: &str) -> IResult<&str, ()> {
    let (input, _) = many0(alt((
        value((), multispace1),
        value((), pair(char('#'), not_line_ending)),
    )))(input)?;
    Ok((input, ()))
}

fn identifier(input: &str) -> IResult<&str, &str> {
    nom::combinator::recognize(pair(
        take_while1(|c: char| c.is_ascii_alphabetic() || c == '_'),
        take_while(|c: char| c.is_ascii_alphanumeric() || c == '_'),
    ))(input)
}

/// Quoted string: `"AC128"`
fn quoted_string(input: &str) -> IResult<&str, &str> {
    nom::sequence::delimited(char('"'), take_while(|c: char| c != '"'), char('"'))(input)
}

/// A single property like `vce_max(32)` or `part("AC128")`.
#[derive(Debug)]
enum HwProp {
    Part(String),
    VceMax(f64),
    VoltageRating(f64),
    SupplyMax(f64),
    Breakdown(f64),
    PowerRating(f64),
}

fn parse_part(input: &str) -> IResult<&str, HwProp> {
    let (input, _) = tag("part")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, name) = quoted_string(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, HwProp::Part(name.to_string())))
}

fn parse_numeric_prop<'a>(
    keyword: &'static str,
    ctor: fn(f64) -> HwProp,
) -> impl Fn(&'a str) -> IResult<&'a str, HwProp> {
    move |input: &'a str| {
        let (input, _) = tag(keyword)(input)?;
        let (input, _) = char('(')(input)?;
        let (input, val) = double(input)?;
        let (input, _) = char(')')(input)?;
        Ok((input, ctor(val)))
    }
}

fn hw_prop(input: &str) -> IResult<&str, HwProp> {
    alt((
        parse_part,
        parse_numeric_prop("vce_max", HwProp::VceMax),
        parse_numeric_prop("voltage_rating", HwProp::VoltageRating),
        parse_numeric_prop("supply_max", HwProp::SupplyMax),
        parse_numeric_prop("breakdown", HwProp::Breakdown),
        parse_numeric_prop("power_rating", HwProp::PowerRating),
    ))(input)
}

/// Parse one line: `Q1: vce_max(32) part("AC128")`
fn hw_line(input: &str) -> IResult<&str, (String, Vec<HwProp>)> {
    let (input, _) = ws_comments(input)?;
    let (input, id) = identifier(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(':')(input)?;
    let (input, _) = ws_comments(input)?;
    // One or more properties separated by optional whitespace.
    let (input, first) = hw_prop(input)?;
    let (input, rest) = many0(|i| {
        let (i, _) = ws_comments(i)?;
        hw_prop(i)
    })(input)?;
    let mut props = vec![first];
    props.extend(rest);
    Ok((input, (id.to_string(), props)))
}

/// Parse a complete `.pedalhw` file.
pub fn parse_pedalhw(input: &str) -> Result<HardwareLimits, String> {
    let mut remaining = input;
    let mut specs: HashMap<String, HardwareSpec> = HashMap::new();

    loop {
        // Skip whitespace and comments.
        match ws_comments(remaining) {
            Ok((r, _)) => remaining = r,
            Err(_) => break,
        }
        if remaining.is_empty() {
            break;
        }
        match hw_line(remaining) {
            Ok((r, (id, props))) => {
                remaining = r;
                let spec = specs.entry(id).or_default();
                for prop in props {
                    match prop {
                        HwProp::Part(name) => spec.part = Some(name),
                        HwProp::VceMax(v) => spec.vce_max = Some(v),
                        HwProp::VoltageRating(v) => spec.voltage_rating = Some(v),
                        HwProp::SupplyMax(v) => spec.supply_max = Some(v),
                        HwProp::Breakdown(v) => spec.breakdown = Some(v),
                        HwProp::PowerRating(v) => spec.power_rating = Some(v),
                    }
                }
            }
            Err(e) => {
                return Err(format!(
                    "Parse error in .pedalhw at: {:?} — {e}",
                    &remaining[..remaining.len().min(60)]
                ));
            }
        }
    }

    Ok(HardwareLimits { specs })
}

/// Convenience: parse from a file path.
pub fn parse_pedalhw_file(path: &str) -> Result<HardwareLimits, String> {
    let src = std::fs::read_to_string(path).map_err(|e| format!("Failed to read {path}: {e}"))?;
    parse_pedalhw(&src)
}

// ═══════════════════════════════════════════════════════════════════════════
// Voltage check with hardware specs
// ═══════════════════════════════════════════════════════════════════════════

/// Check voltage compatibility using real hardware specs from a `.pedalhw`
/// file.  When a component has an explicit spec, that takes priority over
/// the heuristic guesses in [`crate::compiler::check_voltage_compatibility`].
/// Components without specs still get the heuristic treatment.
pub fn check_voltage_with_specs(
    pedal: &PedalDef,
    voltage: f64,
    limits: &HardwareLimits,
) -> Vec<VoltageWarning> {
    let mut warnings = Vec::new();

    for comp in &pedal.components {
        if let Some(spec) = limits.specs.get(&comp.id) {
            // ── Explicit spec: use real values ────────────────────────
            check_spec_voltage(&comp.id, &comp.kind, spec, voltage, &mut warnings);
        } else {
            // ── No spec: fall back to heuristic for this component ───
            check_heuristic_voltage(pedal, &comp.id, &comp.kind, voltage, &mut warnings);
        }
    }

    warnings
}

/// Check a single component against its explicit hardware spec.
fn check_spec_voltage(
    id: &str,
    kind: &ComponentKind,
    spec: &HardwareSpec,
    voltage: f64,
    warnings: &mut Vec<VoltageWarning>,
) {
    let part_label = spec
        .part
        .as_deref()
        .map(|p| format!(" ({p})"))
        .unwrap_or_default();

    // Transistor Vce(max)
    if let Some(vce_max) = spec.vce_max {
        match kind {
            ComponentKind::Npn | ComponentKind::Pnp => {
                if voltage > vce_max {
                    warnings.push(VoltageWarning {
                        component_id: id.to_string(),
                        severity: WarningSeverity::Danger,
                        message: format!(
                            "Transistor {id}{part_label} exceeds Vce(max) {vce_max:.0}V \
                             at {voltage:.0}V supply",
                        ),
                    });
                } else if voltage > vce_max * 0.8 {
                    warnings.push(VoltageWarning {
                        component_id: id.to_string(),
                        severity: WarningSeverity::Caution,
                        message: format!(
                            "Transistor {id}{part_label} at {voltage:.0}V is within 20% \
                             of Vce(max) {vce_max:.0}V",
                        ),
                    });
                }
            }
            _ => {}
        }
    }

    // Capacitor voltage rating
    if let Some(rating) = spec.voltage_rating {
        if let ComponentKind::Capacitor(_) = kind {
            if voltage > rating {
                warnings.push(VoltageWarning {
                    component_id: id.to_string(),
                    severity: WarningSeverity::Danger,
                    message: format!(
                        "Capacitor {id}{part_label} rated {rating:.0}V — will fail at {voltage:.0}V",
                    ),
                });
            } else if voltage > rating * 0.8 {
                warnings.push(VoltageWarning {
                    component_id: id.to_string(),
                    severity: WarningSeverity::Caution,
                    message: format!(
                        "Capacitor {id}{part_label} rated {rating:.0}V — {voltage:.0}V \
                         is within 20% of rating (derate for reliability)",
                    ),
                });
            }
        }
    }

    // Op-amp supply max
    if let Some(supply_max) = spec.supply_max {
        if let ComponentKind::OpAmp = kind {
            if voltage > supply_max {
                warnings.push(VoltageWarning {
                    component_id: id.to_string(),
                    severity: WarningSeverity::Danger,
                    message: format!(
                        "Op-amp {id}{part_label} max supply {supply_max:.0}V — \
                         {voltage:.0}V exceeds absolute max",
                    ),
                });
            } else if voltage > supply_max * 0.9 {
                warnings.push(VoltageWarning {
                    component_id: id.to_string(),
                    severity: WarningSeverity::Caution,
                    message: format!(
                        "Op-amp {id}{part_label} max supply {supply_max:.0}V — \
                         {voltage:.0}V is close to limit",
                    ),
                });
            }
        }
    }

    // Diode reverse breakdown
    if let Some(breakdown) = spec.breakdown {
        match kind {
            ComponentKind::Diode(_) | ComponentKind::DiodePair(_) => {
                if voltage > breakdown {
                    warnings.push(VoltageWarning {
                        component_id: id.to_string(),
                        severity: WarningSeverity::Danger,
                        message: format!(
                            "Diode {id}{part_label} reverse breakdown {breakdown:.0}V — \
                             may fail at {voltage:.0}V",
                        ),
                    });
                }
            }
            _ => {}
        }
    }

    // Resistor power
    if let Some(power_max) = spec.power_rating {
        if let ComponentKind::Resistor(r) = kind {
            // P = V²/R (worst case: full supply across the resistor)
            let worst_case_power = voltage * voltage / r;
            if worst_case_power > power_max {
                warnings.push(VoltageWarning {
                    component_id: id.to_string(),
                    severity: WarningSeverity::Caution,
                    message: format!(
                        "Resistor {id}{part_label} rated {power_max:.2}W — worst-case \
                         dissipation at {voltage:.0}V is {worst_case_power:.2}W",
                    ),
                });
            }
        }
    }
}

/// Heuristic voltage check for a single component (no spec available).
/// This mirrors the logic in `compiler::check_voltage_compatibility` but
/// operates per-component so we can mix spec'd and unspec'd components.
fn check_heuristic_voltage(
    pedal: &PedalDef,
    id: &str,
    kind: &ComponentKind,
    voltage: f64,
    warnings: &mut Vec<VoltageWarning>,
) {
    match kind {
        ComponentKind::Pnp | ComponentKind::Npn => {
            let likely_ge = *kind == ComponentKind::Pnp
                && pedal
                    .controls
                    .iter()
                    .any(|c| c.label.eq_ignore_ascii_case("fuzz"));
            if likely_ge {
                if voltage > 18.0 {
                    warnings.push(VoltageWarning {
                        component_id: id.to_string(),
                        severity: WarningSeverity::Danger,
                        message: format!(
                            "Germanium transistor {id} likely exceeds Vce(max) at {voltage:.0}V \
                             (typical Ge PNP rated 15–32V) — add a .pedalhw file to specify exact limits",
                        ),
                    });
                } else if voltage > 12.0 {
                    warnings.push(VoltageWarning {
                        component_id: id.to_string(),
                        severity: WarningSeverity::Caution,
                        message: format!(
                            "Germanium transistor {id} may run hot at {voltage:.0}V \
                             — add a .pedalhw file to specify exact Vce(max)",
                        ),
                    });
                }
            } else if voltage > 24.0 {
                warnings.push(VoltageWarning {
                    component_id: id.to_string(),
                    severity: WarningSeverity::Caution,
                    message: format!(
                        "Transistor {id} at {voltage:.0}V — add a .pedalhw file to verify Vce(max)",
                    ),
                });
            }
        }
        ComponentKind::OpAmp => {
            if voltage > 18.0 {
                warnings.push(VoltageWarning {
                    component_id: id.to_string(),
                    severity: WarningSeverity::Caution,
                    message: format!(
                        "Op-amp {id} at {voltage:.0}V — add a .pedalhw file to specify supply_max",
                    ),
                });
            }
        }
        ComponentKind::Capacitor(farads) => {
            if *farads >= 1e-6 {
                if voltage > 16.0 {
                    warnings.push(VoltageWarning {
                        component_id: id.to_string(),
                        severity: WarningSeverity::Danger,
                        message: format!(
                            "Electrolytic cap {id} ({:.0}µF) may exceed voltage rating at {voltage:.0}V \
                             — add a .pedalhw file to specify voltage_rating",
                            farads * 1e6,
                        ),
                    });
                } else if voltage > 12.0 {
                    warnings.push(VoltageWarning {
                        component_id: id.to_string(),
                        severity: WarningSeverity::Caution,
                        message: format!(
                            "Electrolytic cap {id} ({:.0}µF) — add a .pedalhw file to confirm \
                             voltage_rating ≥ {voltage:.0}V",
                            farads * 1e6,
                        ),
                    });
                }
            }
        }
        ComponentKind::DiodePair(crate::dsl::DiodeType::Germanium)
        | ComponentKind::Diode(crate::dsl::DiodeType::Germanium) => {
            if voltage > 18.0 {
                warnings.push(VoltageWarning {
                    component_id: id.to_string(),
                    severity: WarningSeverity::Info,
                    message: format!(
                        "Germanium diode {id} — higher power dissipation at {voltage:.0}V \
                         may shift forward voltage",
                    ),
                });
            }
        }
        _ => {}
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_empty() {
        let limits = parse_pedalhw("").unwrap();
        assert!(limits.specs.is_empty());
    }

    #[test]
    fn parse_comments_only() {
        let limits = parse_pedalhw("# just a comment\n# another one\n").unwrap();
        assert!(limits.specs.is_empty());
    }

    #[test]
    fn parse_single_prop() {
        let limits = parse_pedalhw("Q1: vce_max(32)").unwrap();
        assert_eq!(limits.specs["Q1"].vce_max, Some(32.0));
    }

    #[test]
    fn parse_multiple_props() {
        let limits = parse_pedalhw(r#"Q1: vce_max(32) part("AC128")"#).unwrap();
        let spec = &limits.specs["Q1"];
        assert_eq!(spec.vce_max, Some(32.0));
        assert_eq!(spec.part.as_deref(), Some("AC128"));
    }

    #[test]
    fn parse_multiple_lines() {
        let src = r#"
# Hardware specs
Q1: vce_max(32) part("AC128")
Q2: vce_max(32) part("AC128")
C1: voltage_rating(25)
C2: voltage_rating(16)
U1: supply_max(36) part("TL072")
D1: breakdown(50)
R1: power_rating(0.25)
"#;
        let limits = parse_pedalhw(src).unwrap();
        assert_eq!(limits.specs.len(), 7);
        assert_eq!(limits.specs["Q1"].vce_max, Some(32.0));
        assert_eq!(limits.specs["C2"].voltage_rating, Some(16.0));
        assert_eq!(limits.specs["U1"].supply_max, Some(36.0));
        assert_eq!(limits.specs["U1"].part.as_deref(), Some("TL072"));
        assert_eq!(limits.specs["D1"].breakdown, Some(50.0));
        assert_eq!(limits.specs["R1"].power_rating, Some(0.25));
    }

    // ── Voltage checks with specs ────────────────────────────────────────

    fn fuzz_face_pedal() -> PedalDef {
        let src = std::fs::read_to_string("examples/fuzz_face.pedal").unwrap();
        crate::dsl::parse_pedal_file(&src).unwrap()
    }

    fn tube_screamer_pedal() -> PedalDef {
        let src = std::fs::read_to_string("examples/tube_screamer.pedal").unwrap();
        crate::dsl::parse_pedal_file(&src).unwrap()
    }

    #[test]
    fn spec_transistor_exceeds_vce_max() {
        let pedal = fuzz_face_pedal();
        let limits = parse_pedalhw("Q1: vce_max(15)\nQ2: vce_max(15)").unwrap();
        let warnings = check_voltage_with_specs(&pedal, 18.0, &limits);
        let transistor_danger: Vec<_> = warnings
            .iter()
            .filter(|w| w.component_id.starts_with('Q') && w.severity == WarningSeverity::Danger)
            .collect();
        assert_eq!(
            transistor_danger.len(),
            2,
            "Both Ge transistors should be Danger at 18V"
        );
    }

    #[test]
    fn spec_transistor_within_rating() {
        let pedal = fuzz_face_pedal();
        let limits = parse_pedalhw("Q1: vce_max(32)\nQ2: vce_max(32)").unwrap();
        let warnings = check_voltage_with_specs(&pedal, 12.0, &limits);
        let transistor_warns: Vec<_> = warnings
            .iter()
            .filter(|w| w.component_id.starts_with('Q'))
            .collect();
        assert!(
            transistor_warns.is_empty(),
            "32V-rated transistors should be fine at 12V: got {transistor_warns:?}"
        );
    }

    #[test]
    fn spec_cap_voltage_rating_exceeded() {
        let pedal = fuzz_face_pedal();
        // C2 is 10µF — set a 10V rating, check at 12V
        let limits = parse_pedalhw("C2: voltage_rating(10)").unwrap();
        let warnings = check_voltage_with_specs(&pedal, 12.0, &limits);
        let c2_danger: Vec<_> = warnings
            .iter()
            .filter(|w| w.component_id == "C2" && w.severity == WarningSeverity::Danger)
            .collect();
        assert_eq!(c2_danger.len(), 1, "10V cap at 12V should be Danger");
    }

    #[test]
    fn spec_cap_voltage_rating_ok() {
        let pedal = fuzz_face_pedal();
        let limits = parse_pedalhw("C2: voltage_rating(25)").unwrap();
        let warnings = check_voltage_with_specs(&pedal, 12.0, &limits);
        let c2_warns: Vec<_> = warnings
            .iter()
            .filter(|w| w.component_id == "C2")
            .collect();
        assert!(c2_warns.is_empty(), "25V cap at 12V should be fine");
    }

    #[test]
    fn spec_overrides_heuristic() {
        // Even though heuristic would warn about Ge PNPs >12V, a spec
        // showing 32V Vce(max) should suppress the warning at 15V.
        let pedal = fuzz_face_pedal();
        let limits = parse_pedalhw("Q1: vce_max(32)\nQ2: vce_max(32)").unwrap();
        let warnings = check_voltage_with_specs(&pedal, 15.0, &limits);
        let transistor_warns: Vec<_> = warnings
            .iter()
            .filter(|w| w.component_id.starts_with('Q'))
            .collect();
        assert!(
            transistor_warns.is_empty(),
            "Spec'd transistors should override heuristic: got {transistor_warns:?}"
        );
    }

    #[test]
    fn unspecced_components_get_heuristic() {
        // Only spec Q1 — Q2 should still get the heuristic warning.
        let pedal = fuzz_face_pedal();
        let limits = parse_pedalhw("Q1: vce_max(32)").unwrap();
        let warnings = check_voltage_with_specs(&pedal, 15.0, &limits);
        let q2_warns: Vec<_> = warnings
            .iter()
            .filter(|w| w.component_id == "Q2")
            .collect();
        assert!(
            !q2_warns.is_empty(),
            "Unspec'd Q2 should get heuristic warning at 15V"
        );
        let q1_warns: Vec<_> = warnings
            .iter()
            .filter(|w| w.component_id == "Q1")
            .collect();
        assert!(
            q1_warns.is_empty(),
            "Spec'd Q1 (32V) should be fine at 15V"
        );
    }

    #[test]
    fn tube_screamer_no_specs_12v_clean() {
        // TS at 12V with no specs should have no warnings (all passive + Si diodes).
        let pedal = tube_screamer_pedal();
        let limits = HardwareLimits::default();
        let warnings = check_voltage_with_specs(&pedal, 12.0, &limits);
        assert!(
            warnings.is_empty(),
            "TS at 12V should be clean: got {warnings:?}"
        );
    }

    #[test]
    fn opamp_supply_max_exceeded() {
        let pedal = {
            let src = std::fs::read_to_string("examples/klon_centaur.pedal").unwrap();
            crate::dsl::parse_pedal_file(&src).unwrap()
        };
        let limits = parse_pedalhw("U1: supply_max(36)\nU2: supply_max(36)").unwrap();
        let warnings = check_voltage_with_specs(&pedal, 40.0, &limits);
        let opamp_danger: Vec<_> = warnings
            .iter()
            .filter(|w| w.component_id.starts_with('U') && w.severity == WarningSeverity::Danger)
            .collect();
        assert_eq!(opamp_danger.len(), 2, "Both opamps should be Danger at 40V");
    }

    #[test]
    fn resistor_power_warning() {
        // R2 in Fuzz Face is 8.2k — P = 18²/8200 ≈ 0.04W, under 0.25W.
        // But at 18V with a tiny 100Ω resistor rated 0.01W it would warn.
        let src = r#"
pedal "Test" {
  components {
    R1: resistor(100)
  }
  nets {
    in -> R1.a
    R1.b -> out
  }
}
"#;
        let pedal = crate::dsl::parse_pedal_file(src).unwrap();
        let limits = parse_pedalhw("R1: power_rating(0.01)").unwrap();
        let warnings = check_voltage_with_specs(&pedal, 18.0, &limits);
        let r1_warns: Vec<_> = warnings
            .iter()
            .filter(|w| w.component_id == "R1")
            .collect();
        // P = 18²/100 = 3.24W >> 0.01W
        assert!(!r1_warns.is_empty(), "Should warn about power dissipation");
    }

    // ── Example .pedalhw file tests ──────────────────────────────────────

    #[test]
    fn parse_fuzz_face_pedalhw() {
        let src = std::fs::read_to_string("examples/fuzz_face.pedalhw");
        if let Ok(src) = src {
            let limits = parse_pedalhw(&src).unwrap();
            assert!(!limits.specs.is_empty());
        }
        // If file doesn't exist yet, skip — it's created by the example files step.
    }
}
