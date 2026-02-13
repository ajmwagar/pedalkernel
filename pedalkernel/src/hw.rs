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

    while let Ok((r, _)) = ws_comments(remaining) {
        remaining = r;
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
// Curated parts database
// ═══════════════════════════════════════════════════════════════════════════

/// A known part from the curated database.
#[derive(Debug, Clone)]
pub struct KnownPart {
    /// Mouser part number.
    pub mouser_pn: &'static str,
    /// Human-readable description.
    pub description: &'static str,
    /// Hardware specs we know from the datasheet.
    pub spec: HardwareSpec,
}

/// Resistors — Yageo MFR-25 1/4W metal film.
const RESISTORS: &[(f64, &str, &str)] = &[
    (100.0, "603-MFR-25FRF52-100R", "100\u{2126} 1/4W Metal Film"),
    (470.0, "603-MFR-25FRF52-470R", "470\u{2126} 1/4W Metal Film"),
    (1e3, "603-MFR-25FRF52-1K", "1k\u{2126} 1/4W Metal Film"),
    (1.5e3, "603-MFR-25FRF52-1K5", "1.5k\u{2126} 1/4W Metal Film"),
    (4.7e3, "603-MFR-25FRF52-4K7", "4.7k\u{2126} 1/4W Metal Film"),
    (8.2e3, "603-MFR-25FRF52-8K2", "8.2k\u{2126} 1/4W Metal Film"),
    (10e3, "603-MFR-25FRF52-10K", "10k\u{2126} 1/4W Metal Film"),
    (15e3, "603-MFR-25FRF52-15K", "15k\u{2126} 1/4W Metal Film"),
    (22e3, "603-MFR-25FRF52-22K", "22k\u{2126} 1/4W Metal Film"),
    (33e3, "603-MFR-25FRF52-33K", "33k\u{2126} 1/4W Metal Film"),
    (47e3, "603-MFR-25FRF52-47K", "47k\u{2126} 1/4W Metal Film"),
    (
        100e3,
        "603-MFR-25FRF52-100K",
        "100k\u{2126} 1/4W Metal Film",
    ),
    (
        150e3,
        "603-MFR-25FRF52-150K",
        "150k\u{2126} 1/4W Metal Film",
    ),
    (
        422e3,
        "603-MFR-25FRF52-422K",
        "422k\u{2126} 1/4W Metal Film",
    ),
    (
        470e3,
        "603-MFR-25FRF52-470K",
        "470k\u{2126} 1/4W Metal Film",
    ),
    (
        500e3,
        "603-MFR-25FRF52-500K",
        "500k\u{2126} 1/4W Metal Film",
    ),
    (
        560e3,
        "603-MFR-25FRF52-560K",
        "560k\u{2126} 1/4W Metal Film",
    ),
    (1e6, "603-MFR-25FRF52-1M", "1M\u{2126} 1/4W Metal Film"),
];

/// Capacitors — Kemet ceramic (pF), WIMA MKS2 film (nF), Nichicon electrolytic (µF).
const CAPACITORS: &[(f64, &str, &str, f64)] = &[
    // (farads, mouser_pn, description, voltage_rating)
    (100e-12, "80-C320C101J1G5TA", "100pF Ceramic", 50.0),
    (390e-12, "80-C320C391J1G5TA", "390pF Ceramic", 50.0),
    (1e-9, "80-C320C102K5R5TA", "1nF Ceramic", 50.0),
    (3.3e-9, "80-C320C332K5R5TA", "3.3nF Ceramic", 50.0),
    (4e-9, "80-C320C402K5R5TA", "4nF Ceramic", 50.0),
    (10e-9, "5-MKS2D031001A00JSSD", "10nF WIMA Film", 100.0),
    (22e-9, "5-MKS2D032201A00JSSD", "22nF WIMA Film", 100.0),
    (47e-9, "5-MKS2D034701A00JSSD", "47nF WIMA Film", 100.0),
    (68e-9, "5-MKS2D036801A00JSSD", "68nF WIMA Film", 100.0),
    (100e-9, "5-MKS2D041001A00JSSD", "100nF WIMA Film", 100.0),
    (220e-9, "5-MKS2D042201A00JSSD", "220nF WIMA Film", 100.0),
    (1e-6, "647-UPW1H010MDD6", "1\u{b5}F 50V Electrolytic", 50.0),
    (
        2.2e-6,
        "647-UPW1H2R2MDD6",
        "2.2\u{b5}F 50V Electrolytic",
        50.0,
    ),
    (
        10e-6,
        "647-UPW1H100MDD6",
        "10\u{b5}F 50V Electrolytic",
        50.0,
    ),
    (
        100e-6,
        "647-UPW1H101MED6",
        "100\u{b5}F 50V Electrolytic",
        50.0,
    ),
];

/// Potentiometers — Alpha 16mm single-gang.
const POTS: &[(f64, &str, &str)] = &[
    (1e3, "317-2001F-1K", "1k\u{2126} Alpha 16mm Pot"),
    (4.7e3, "317-2001F-4.7K", "4.7k\u{2126} Alpha 16mm Pot"),
    (10e3, "317-2001F-10K", "10k\u{2126} Alpha 16mm Pot"),
    (25e3, "317-2001F-25K", "25k\u{2126} Alpha 16mm Pot"),
    (50e3, "317-2001F-50K", "50k\u{2126} Alpha 16mm Pot"),
    (100e3, "317-2001F-100K", "100k\u{2126} Alpha 16mm Pot"),
    (250e3, "317-2001F-250K", "250k\u{2126} Alpha 16mm Pot"),
    (500e3, "317-2001F-500K", "500k\u{2126} Alpha 16mm Pot"),
];

/// Known diode parts with specs.
struct DiodePart {
    mouser_pn: &'static str,
    description: &'static str,
    breakdown: f64,
}

const SILICON_DIODE: DiodePart = DiodePart {
    mouser_pn: "512-1N4148",
    description: "1N4148 Silicon Diode",
    breakdown: 100.0,
};

const GERMANIUM_DIODE: DiodePart = DiodePart {
    mouser_pn: "583-1N34A",
    description: "1N34A Germanium Diode",
    breakdown: 60.0,
};

const LED_DIODE: DiodePart = DiodePart {
    mouser_pn: "604-WP7113ID",
    description: "3mm Red LED",
    breakdown: 5.0,
};

/// Known transistor parts with specs.
struct TransistorPart {
    mouser_pn: &'static str,
    description: &'static str,
    vce_max: f64,
}

const NPN_DEFAULT: TransistorPart = TransistorPart {
    mouser_pn: "512-2N3904BU",
    description: "2N3904 NPN Transistor",
    vce_max: 40.0,
};

const PNP_DEFAULT: TransistorPart = TransistorPart {
    mouser_pn: "512-2N3906BU",
    description: "2N3906 PNP Transistor",
    vce_max: 40.0,
};

const OPAMP_DEFAULT: (&str, &str, f64) = ("595-TL072CP", "TL072 Dual Op-Amp", 36.0);

/// Find the closest match in a value table within 2% tolerance.
fn find_closest<'a>(
    table: &'a [(f64, &'a str, &'a str)],
    target: f64,
) -> Option<(&'a str, &'a str)> {
    table.iter().find_map(|(val, pn, desc)| {
        if *val == 0.0 || target == 0.0 {
            return None;
        }
        if (val - target).abs() / val < 0.02 {
            Some((*pn, *desc))
        } else {
            None
        }
    })
}

/// Find the closest capacitor match (returns voltage rating too).
fn find_closest_cap(target: f64) -> Option<(&'static str, &'static str, f64)> {
    CAPACITORS.iter().find_map(|(val, pn, desc, vr)| {
        if *val == 0.0 || target == 0.0 {
            return None;
        }
        if (val - target).abs() / val < 0.02 {
            Some((*pn, *desc, *vr))
        } else {
            None
        }
    })
}

// ═══════════════════════════════════════════════════════════════════════════
// Auto-populate HardwareSpec from parts DB
// ═══════════════════════════════════════════════════════════════════════════

/// Enrich a `HardwareLimits` by filling in missing spec fields from the
/// curated parts database.  If a component has `part("TL072")` but no
/// `supply_max`, this fills it in from the known TL072 datasheet.
///
/// Also populates defaults for components with no `.pedalhw` entry at all,
/// based on the DSL component kind (e.g. all opamps default to TL072 specs).
pub fn auto_populate_specs(pedal: &PedalDef, limits: &mut HardwareLimits) {
    for comp in &pedal.components {
        let spec = limits.specs.entry(comp.id.clone()).or_default();
        match &comp.kind {
            ComponentKind::Resistor(_) => {
                if spec.power_rating.is_none() {
                    spec.power_rating = Some(0.25); // 1/4W default
                }
            }
            ComponentKind::Capacitor(farads) => {
                if spec.voltage_rating.is_none() {
                    if let Some((_, _, vr)) = find_closest_cap(*farads) {
                        spec.voltage_rating = Some(vr);
                    }
                }
            }
            ComponentKind::Npn => {
                if spec.vce_max.is_none() {
                    spec.vce_max = Some(NPN_DEFAULT.vce_max);
                }
            }
            ComponentKind::Pnp => {
                if spec.vce_max.is_none() {
                    spec.vce_max = Some(PNP_DEFAULT.vce_max);
                }
            }
            ComponentKind::OpAmp => {
                if spec.supply_max.is_none() {
                    spec.supply_max = Some(OPAMP_DEFAULT.2);
                }
            }
            ComponentKind::Diode(dt) | ComponentKind::DiodePair(dt) => {
                if spec.breakdown.is_none() {
                    let dp = match dt {
                        crate::dsl::DiodeType::Silicon => &SILICON_DIODE,
                        crate::dsl::DiodeType::Germanium => &GERMANIUM_DIODE,
                        crate::dsl::DiodeType::Led => &LED_DIODE,
                    };
                    spec.breakdown = Some(dp.breakdown);
                }
            }
            _ => {}
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// BOM generation
// ═══════════════════════════════════════════════════════════════════════════

/// A single line item in a bill of materials.
#[derive(Debug, Clone)]
pub struct BomEntry {
    /// Component reference from the .pedal file (e.g. "R1", "C2").
    pub reference: String,
    /// Component type for display (e.g. "Resistor", "Capacitor").
    pub display: String,
    /// Formatted value (e.g. "4.7k\u{2126}", "47nF").
    pub value: String,
    /// Mouser part number, or `None` if no match found.
    pub mouser_pn: Option<String>,
    /// Human-readable part description.
    pub description: String,
    /// Quantity needed per pedal (2 for diode_pair, 1 for everything else).
    pub qty_per_unit: u32,
}

/// Build a bill of materials from a parsed pedal definition.
///
/// If `limits` is provided, `.pedalhw` overrides (like `part("AC128")`) take
/// priority over the default parts DB lookup.
pub fn build_bom(pedal: &PedalDef, limits: Option<&HardwareLimits>) -> Vec<BomEntry> {
    use crate::dsl::DiodeType;

    pedal
        .components
        .iter()
        .map(|comp| {
            let hw_part = limits
                .and_then(|l| l.specs.get(&comp.id))
                .and_then(|s| s.part.as_deref());

            match &comp.kind {
                ComponentKind::Resistor(val) => {
                    let (pn, desc) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string())
                    } else if let Some((pn, desc)) = find_closest(RESISTORS, *val) {
                        (Some(pn.to_string()), desc.to_string())
                    } else {
                        (
                            None,
                            format!("{} Resistor", crate::kicad::format_eng(*val, "\u{2126}")),
                        )
                    };
                    BomEntry {
                        reference: comp.id.clone(),
                        display: "Resistor".into(),
                        value: crate::kicad::format_eng(*val, "\u{2126}"),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }
                }
                ComponentKind::Capacitor(val) => {
                    let (pn, desc) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string())
                    } else if let Some((cpn, cdesc, _)) = find_closest_cap(*val) {
                        (Some(cpn.to_string()), cdesc.to_string())
                    } else {
                        (
                            None,
                            format!("{} Capacitor", crate::kicad::format_eng(*val, "F")),
                        )
                    };
                    BomEntry {
                        reference: comp.id.clone(),
                        display: "Capacitor".into(),
                        value: crate::kicad::format_eng(*val, "F"),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }
                }
                ComponentKind::Inductor(val) => BomEntry {
                    reference: comp.id.clone(),
                    display: "Inductor".into(),
                    value: crate::kicad::format_eng(*val, "H"),
                    mouser_pn: None,
                    description: format!("{} Inductor", crate::kicad::format_eng(*val, "H")),
                    qty_per_unit: 1,
                },
                ComponentKind::Potentiometer(val) => {
                    let (pn, desc) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string())
                    } else if let Some((ppn, pdesc)) = find_closest(POTS, *val) {
                        (Some(ppn.to_string()), pdesc.to_string())
                    } else {
                        (
                            None,
                            format!(
                                "{} Potentiometer",
                                crate::kicad::format_eng(*val, "\u{2126}")
                            ),
                        )
                    };
                    BomEntry {
                        reference: comp.id.clone(),
                        display: "Potentiometer".into(),
                        value: crate::kicad::format_eng(*val, "\u{2126}"),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }
                }
                ComponentKind::Diode(dt) => {
                    let dp = match dt {
                        DiodeType::Silicon => &SILICON_DIODE,
                        DiodeType::Germanium => &GERMANIUM_DIODE,
                        DiodeType::Led => &LED_DIODE,
                    };
                    let (pn, desc) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string())
                    } else {
                        (Some(dp.mouser_pn.to_string()), dp.description.to_string())
                    };
                    BomEntry {
                        reference: comp.id.clone(),
                        display: format!("Diode ({dt:?})"),
                        value: format!("{dt:?}"),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }
                }
                ComponentKind::DiodePair(dt) => {
                    let dp = match dt {
                        DiodeType::Silicon => &SILICON_DIODE,
                        DiodeType::Germanium => &GERMANIUM_DIODE,
                        DiodeType::Led => &LED_DIODE,
                    };
                    let (pn, desc) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string())
                    } else {
                        (Some(dp.mouser_pn.to_string()), dp.description.to_string())
                    };
                    BomEntry {
                        reference: comp.id.clone(),
                        display: format!("Diode Pair ({dt:?})"),
                        value: format!("{dt:?} x2"),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 2,
                    }
                }
                ComponentKind::Npn => {
                    let (pn, desc) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string())
                    } else {
                        (
                            Some(NPN_DEFAULT.mouser_pn.to_string()),
                            NPN_DEFAULT.description.to_string(),
                        )
                    };
                    BomEntry {
                        reference: comp.id.clone(),
                        display: "NPN Transistor".into(),
                        value: "\u{2014}".into(),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }
                }
                ComponentKind::Pnp => {
                    let (pn, desc) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string())
                    } else {
                        (
                            Some(PNP_DEFAULT.mouser_pn.to_string()),
                            PNP_DEFAULT.description.to_string(),
                        )
                    };
                    BomEntry {
                        reference: comp.id.clone(),
                        display: "PNP Transistor".into(),
                        value: "\u{2014}".into(),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }
                }
                ComponentKind::OpAmp => {
                    let (pn, desc) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string())
                    } else {
                        (
                            Some(OPAMP_DEFAULT.0.to_string()),
                            OPAMP_DEFAULT.1.to_string(),
                        )
                    };
                    BomEntry {
                        reference: comp.id.clone(),
                        display: "Op-Amp".into(),
                        value: "\u{2014}".into(),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }
                }
            }
        })
        .collect()
}

/// Export a BOM as a Mouser-compatible CSV.
///
/// Format: `Mouser Part Number,Quantity,Reference,Description`
///
/// Lines without a Mouser P/N are omitted (the user needs to source those
/// manually).  `qty` multiplies the per-unit quantity for batch builds.
pub fn export_bom_csv(bom: &[BomEntry], qty: u32) -> String {
    let mut out = String::from("Mouser Part Number,Quantity,Reference,Description\n");
    for entry in bom {
        if let Some(pn) = &entry.mouser_pn {
            out.push_str(&format!(
                "{},{},{},{}\n",
                pn,
                entry.qty_per_unit * qty,
                entry.reference,
                entry.description,
            ));
        }
    }
    out
}

/// Format a BOM as a human-readable table string.
pub fn format_bom_table(pedal_name: &str, bom: &[BomEntry], qty: u32) -> String {
    let line = "\u{2550}".repeat(78);
    let thin = "\u{2500}".repeat(78);
    let mut out = String::new();

    out.push_str(&format!("\n{line}\n"));
    let label = if qty > 1 {
        format!("  BOM: {pedal_name} (x{qty})")
    } else {
        format!("  BOM: {pedal_name}")
    };
    out.push_str(&format!("{label}\n{line}\n"));

    out.push_str(&format!(
        "  {:<8} {:<20} {:<12} {:<24} {:>3}\n",
        "Ref", "Component", "Value", "Mouser P/N", "Qty"
    ));
    out.push_str(&format!(
        "  {:<8} {:<20} {:<12} {:<24} {:>3}\n",
        "\u{2500}".repeat(6),
        "\u{2500}".repeat(9),
        "\u{2500}".repeat(5),
        "\u{2500}".repeat(10),
        "\u{2500}".repeat(3),
    ));

    let mut total_parts = 0u32;
    for entry in bom {
        let pn = entry.mouser_pn.as_deref().unwrap_or("[manual]");
        let entry_qty = entry.qty_per_unit * qty;
        out.push_str(&format!(
            "  {:<8} {:<20} {:<12} {:<24} {:>3}\n",
            entry.reference, entry.display, entry.value, pn, entry_qty,
        ));
        total_parts += entry_qty;
    }

    out.push_str(&format!("{thin}\n"));
    out.push_str(&format!(
        "  Total: {} line items, {} parts\n",
        bom.len(),
        total_parts
    ));

    let unmatched: Vec<_> = bom.iter().filter(|e| e.mouser_pn.is_none()).collect();
    if !unmatched.is_empty() {
        out.push('\n');
        for entry in &unmatched {
            out.push_str(&format!(
                "  Warning: {} — no curated Mouser match for {}\n",
                entry.reference, entry.description
            ));
        }
    }

    out.push_str(&format!("{line}\n"));
    out.push_str("  Upload CSV at: https://www.mouser.com/Bom/Upload\n");
    out
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
        let c2_warns: Vec<_> = warnings.iter().filter(|w| w.component_id == "C2").collect();
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
        let q2_warns: Vec<_> = warnings.iter().filter(|w| w.component_id == "Q2").collect();
        assert!(
            !q2_warns.is_empty(),
            "Unspec'd Q2 should get heuristic warning at 15V"
        );
        let q1_warns: Vec<_> = warnings.iter().filter(|w| w.component_id == "Q1").collect();
        assert!(q1_warns.is_empty(), "Spec'd Q1 (32V) should be fine at 15V");
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
        let r1_warns: Vec<_> = warnings.iter().filter(|w| w.component_id == "R1").collect();
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

    // ── BOM generation tests ─────────────────────────────────────────────

    fn klon_pedal() -> PedalDef {
        let src = std::fs::read_to_string("examples/klon_centaur.pedal").unwrap();
        crate::dsl::parse_pedal_file(&src).unwrap()
    }

    #[test]
    fn bom_tube_screamer_all_matched() {
        let pedal = tube_screamer_pedal();
        let bom = build_bom(&pedal, None);
        assert_eq!(bom.len(), 4); // Drive pot, C1, D1, Level pot
                                  // All should have a Mouser P/N from the curated DB.
        for entry in &bom {
            assert!(
                entry.mouser_pn.is_some(),
                "TS {} should have a Mouser P/N",
                entry.reference
            );
        }
    }

    #[test]
    fn bom_fuzz_face_components() {
        let pedal = fuzz_face_pedal();
        let bom = build_bom(&pedal, None);
        assert_eq!(bom.len(), 9);
        // PNP transistors should default to 2N3906
        let q1 = bom.iter().find(|e| e.reference == "Q1").unwrap();
        assert!(q1.description.contains("2N3906"));
    }

    #[test]
    fn bom_fuzz_face_hw_override() {
        let pedal = fuzz_face_pedal();
        let limits = parse_pedalhw(r#"Q1: part("AC128")"#).unwrap();
        let bom = build_bom(&pedal, Some(&limits));
        let q1 = bom.iter().find(|e| e.reference == "Q1").unwrap();
        assert_eq!(q1.description, "AC128");
        // .pedalhw override means no curated Mouser P/N — user sources manually.
        assert!(q1.mouser_pn.is_none());
    }

    #[test]
    fn bom_diode_pair_qty_2() {
        let pedal = tube_screamer_pedal();
        let bom = build_bom(&pedal, None);
        let d1 = bom.iter().find(|e| e.reference == "D1").unwrap();
        assert_eq!(d1.qty_per_unit, 2, "Diode pair should need 2 units");
    }

    #[test]
    fn bom_klon_opamp_default() {
        let pedal = klon_pedal();
        let bom = build_bom(&pedal, None);
        let u1 = bom.iter().find(|e| e.reference == "U1").unwrap();
        assert!(u1.description.contains("TL072"));
    }

    #[test]
    fn bom_csv_export() {
        let pedal = tube_screamer_pedal();
        let bom = build_bom(&pedal, None);
        let csv = export_bom_csv(&bom, 1);
        assert!(csv.starts_with("Mouser Part Number,"));
        // Should have a line for each matched component.
        let lines: Vec<_> = csv.lines().collect();
        assert!(lines.len() >= 5, "Header + 4 components");
    }

    #[test]
    fn bom_csv_qty_multiplier() {
        let pedal = tube_screamer_pedal();
        let bom = build_bom(&pedal, None);
        let csv_x1 = export_bom_csv(&bom, 1);
        let csv_x5 = export_bom_csv(&bom, 5);
        // D1 is a diode_pair = qty 2 at x1, qty 10 at x5
        assert!(csv_x1.contains(",2,D1,"));
        assert!(csv_x5.contains(",10,D1,"));
    }

    #[test]
    fn bom_table_format() {
        let pedal = tube_screamer_pedal();
        let bom = build_bom(&pedal, None);
        let table = format_bom_table(&pedal.name, &bom, 1);
        assert!(table.contains("Tube Screamer"));
        assert!(table.contains("Drive"));
        assert!(table.contains("Mouser P/N"));
        assert!(table.contains("Total:"));
    }

    #[test]
    fn bom_all_pedals_nonzero() {
        let files = [
            "tube_screamer.pedal",
            "fuzz_face.pedal",
            "big_muff.pedal",
            "blues_driver.pedal",
            "dyna_comp.pedal",
            "klon_centaur.pedal",
            "proco_rat.pedal",
        ];
        for f in files {
            let src = std::fs::read_to_string(format!("examples/{f}")).unwrap();
            let pedal = crate::dsl::parse_pedal_file(&src).unwrap();
            let bom = build_bom(&pedal, None);
            assert!(!bom.is_empty(), "{f} should produce a non-empty BOM");
            // Every entry should have a non-empty description.
            for entry in &bom {
                assert!(
                    !entry.description.is_empty(),
                    "{f} {}: empty description",
                    entry.reference
                );
            }
        }
    }

    // ── Auto-populate tests ──────────────────────────────────────────────

    #[test]
    fn auto_populate_fills_defaults() {
        let pedal = tube_screamer_pedal();
        let mut limits = HardwareLimits::default();
        auto_populate_specs(&pedal, &mut limits);
        // D1 (silicon diode_pair) should get breakdown = 100V (1N4148)
        let d1 = &limits.specs["D1"];
        assert_eq!(d1.breakdown, Some(100.0));
        // C1 (47nF) should get voltage_rating from WIMA film = 100V
        let c1 = &limits.specs["C1"];
        assert_eq!(c1.voltage_rating, Some(100.0));
    }

    #[test]
    fn auto_populate_preserves_explicit() {
        let pedal = fuzz_face_pedal();
        let mut limits = parse_pedalhw("Q1: vce_max(15)").unwrap();
        auto_populate_specs(&pedal, &mut limits);
        // Q1 should keep the explicit 15V, not be overwritten by default 40V.
        assert_eq!(limits.specs["Q1"].vce_max, Some(15.0));
        // Q2 should get the default 40V.
        assert_eq!(limits.specs["Q2"].vce_max, Some(40.0));
    }

    #[test]
    fn auto_populate_opamp_supply() {
        let pedal = klon_pedal();
        let mut limits = HardwareLimits::default();
        auto_populate_specs(&pedal, &mut limits);
        // Both opamps should get TL072 default supply_max = 36V.
        assert_eq!(limits.specs["U1"].supply_max, Some(36.0));
        assert_eq!(limits.specs["U2"].supply_max, Some(36.0));
    }

    #[test]
    fn auto_populate_then_voltage_check_precise() {
        // Auto-populate + voltage check should use real DB values,
        // not heuristics.
        let pedal = fuzz_face_pedal();
        let mut limits = HardwareLimits::default();
        auto_populate_specs(&pedal, &mut limits);
        // PNP defaults to 2N3906 with Vce(max) = 40V.
        // At 15V this should NOT warn (heuristic would have warned).
        let warnings = check_voltage_with_specs(&pedal, 15.0, &limits);
        let q_warns: Vec<_> = warnings
            .iter()
            .filter(|w| w.component_id.starts_with('Q'))
            .collect();
        assert!(
            q_warns.is_empty(),
            "Auto-populated 40V transistors should be fine at 15V: got {q_warns:?}"
        );
    }
}
