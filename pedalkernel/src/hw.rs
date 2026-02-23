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

    // IC supply max (op-amps, synth ICs, comparators, analog switches)
    if let Some(supply_max) = spec.supply_max {
        let is_ic = matches!(kind,
            ComponentKind::OpAmp(_) | ComponentKind::Vco(_) | ComponentKind::Vcf(_) |
            ComponentKind::Vca(_) | ComponentKind::Comparator(_) | ComponentKind::AnalogSwitch(_)
        );
        if is_ic {
            if voltage > supply_max {
                warnings.push(VoltageWarning {
                    component_id: id.to_string(),
                    severity: WarningSeverity::Danger,
                    message: format!(
                        "IC {id}{part_label} max supply {supply_max:.0}V — \
                         {voltage:.0}V exceeds absolute max",
                    ),
                });
            } else if voltage > supply_max * 0.9 {
                warnings.push(VoltageWarning {
                    component_id: id.to_string(),
                    severity: WarningSeverity::Caution,
                    message: format!(
                        "IC {id}{part_label} max supply {supply_max:.0}V — \
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

    // Triode plate voltage — tubes expect much higher voltages than transistors.
    // The WDF model simulates tube behavior at any voltage, but a real build
    // needs a plate supply of 150-400V (via a step-up transformer or charge pump).
    if let ComponentKind::Triode(_) = kind {
        if voltage < 100.0 {
            warnings.push(VoltageWarning {
                component_id: id.to_string(),
                severity: WarningSeverity::Info,
                message: format!(
                    "Tube {id}{part_label} needs 150-400V plate supply; \
                     at {voltage:.0}V the WDF model runs fine but a physical build \
                     needs a B+ supply (charge pump or transformer)",
                ),
            });
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
        ComponentKind::OpAmp(ot) => {
            // Use the op-amp type's supply_max for accurate warning threshold
            let max_supply = ot.supply_max();
            if voltage > max_supply * 0.5 {
                warnings.push(VoltageWarning {
                    component_id: id.to_string(),
                    severity: WarningSeverity::Caution,
                    message: format!(
                        "Op-amp {id} ({ot:?}) at {voltage:.0}V — max supply {max_supply:.0}V",
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
        ComponentKind::Triode(_) => {
            if voltage < 100.0 {
                warnings.push(VoltageWarning {
                    component_id: id.to_string(),
                    severity: WarningSeverity::Info,
                    message: format!(
                        "Tube {id} needs 150-400V plate supply; at {voltage:.0}V the WDF \
                         model runs fine but a physical build needs a B+ supply \
                         — add a .pedalhw file to specify plate voltage",
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
            ComponentKind::OpAmp(ot) => {
                if spec.supply_max.is_none() {
                    spec.supply_max = Some(ot.supply_max());
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
            // Synth ICs: populate supply max from known datasheets
            ComponentKind::Vco(_) => {
                if spec.supply_max.is_none() {
                    spec.supply_max = Some(18.0); // CEM3340/AS3340/V3340: ±5V to ±9V (18V total)
                }
            }
            ComponentKind::Vcf(_) => {
                if spec.supply_max.is_none() {
                    spec.supply_max = Some(18.0); // CEM3320/AS3320: ±9V max
                }
            }
            ComponentKind::Vca(_) => {
                if spec.supply_max.is_none() {
                    spec.supply_max = Some(36.0); // SSM2164/V2164: ±18V max
                }
            }
            ComponentKind::Comparator(ct) => {
                if spec.supply_max.is_none() {
                    spec.supply_max = Some(match ct {
                        crate::dsl::ComparatorType::Lm311 => 36.0,
                        crate::dsl::ComparatorType::Lm393 => 36.0,
                    });
                }
            }
            ComponentKind::AnalogSwitch(_) => {
                if spec.supply_max.is_none() {
                    spec.supply_max = Some(20.0); // CD4066/DG411: 20V max
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
        .flat_map(|comp| {
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
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: "Resistor".into(),
                        value: crate::kicad::format_eng(*val, "\u{2126}"),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }]
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
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: "Capacitor".into(),
                        value: crate::kicad::format_eng(*val, "F"),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }]
                }
                ComponentKind::Inductor(val) => vec![BomEntry {
                    reference: comp.id.clone(),
                    display: "Inductor".into(),
                    value: crate::kicad::format_eng(*val, "H"),
                    mouser_pn: None,
                    description: format!("{} Inductor", crate::kicad::format_eng(*val, "H")),
                    qty_per_unit: 1,
                }],
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
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: "Potentiometer".into(),
                        value: crate::kicad::format_eng(*val, "\u{2126}"),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }]
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
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: format!("Diode ({dt:?})"),
                        value: format!("{dt:?}"),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }]
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
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: format!("Diode Pair ({dt:?})"),
                        value: format!("{dt:?} x2"),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 2,
                    }]
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
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: "NPN Transistor".into(),
                        value: "\u{2014}".into(),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }]
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
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: "PNP Transistor".into(),
                        value: "\u{2014}".into(),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }]
                }
                ComponentKind::OpAmp(ot) => {
                    let (pn, desc, label) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string(), format!("{ot:?}"))
                    } else {
                        // Use type-specific Mouser part numbers
                        match ot {
                            crate::dsl::OpAmpType::Generic | crate::dsl::OpAmpType::Tl072 => {
                                (Some("595-TL072CP".to_string()), "TL072 Dual Op-Amp".to_string(), "TL072".to_string())
                            }
                            crate::dsl::OpAmpType::Tl082 => {
                                (Some("595-TL082CP".to_string()), "TL082 Dual Op-Amp".to_string(), "TL082".to_string())
                            }
                            crate::dsl::OpAmpType::Jrc4558 => {
                                (Some("513-NJM4558DD".to_string()), "JRC4558D Dual Op-Amp".to_string(), "JRC4558D".to_string())
                            }
                            crate::dsl::OpAmpType::Rc4558 => {
                                (Some("595-RC4558P".to_string()), "RC4558 Dual Op-Amp".to_string(), "RC4558".to_string())
                            }
                            crate::dsl::OpAmpType::Lm308 => {
                                (Some("926-LM308N/NOPB".to_string()), "LM308N Op-Amp".to_string(), "LM308N".to_string())
                            }
                            crate::dsl::OpAmpType::Lm741 => {
                                (Some("595-LM741CN/NOPB".to_string()), "LM741 Op-Amp".to_string(), "LM741".to_string())
                            }
                            crate::dsl::OpAmpType::Ne5532 => {
                                (Some("595-NE5532P".to_string()), "NE5532 Dual Op-Amp".to_string(), "NE5532".to_string())
                            }
                            crate::dsl::OpAmpType::Ca3080 => {
                                (Some("595-CA3080EZ".to_string()), "CA3080 OTA".to_string(), "CA3080".to_string())
                            }
                            crate::dsl::OpAmpType::Op07 => {
                                (Some("595-OP07CP".to_string()), "OP07 Precision Op-Amp".to_string(), "OP07".to_string())
                            }
                        }
                    };
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: "Op-Amp".into(),
                        value: label,
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }]
                }
                ComponentKind::NJfet(jt) => {
                    let (pn, desc) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string())
                    } else {
                        match jt {
                            crate::dsl::JfetType::J201 => {
                                (Some("512-J201".to_string()), "J201 N-JFET".to_string())
                            }
                            crate::dsl::JfetType::N2n5457 => {
                                (Some("512-2N5457".to_string()), "2N5457 N-JFET".to_string())
                            }
                            crate::dsl::JfetType::N2n5952 => {
                                (Some("512-2N5952".to_string()), "2N5952 N-JFET".to_string())
                            }
                            crate::dsl::JfetType::P2n5460 => {
                                (None, "N-JFET (unknown model)".to_string())
                            }
                            crate::dsl::JfetType::N2sk30a
                            | crate::dsl::JfetType::N2sk30aGr
                            | crate::dsl::JfetType::N2sk30aY
                            | crate::dsl::JfetType::N2sk30aBl => {
                                (None, format!("2SK30A N-JFET ({jt:?})"))
                            }
                        }
                    };
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: format!("N-JFET ({jt:?})"),
                        value: format!("{jt:?}"),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }]
                }
                ComponentKind::PJfet(jt) => {
                    let (pn, desc) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string())
                    } else {
                        match jt {
                            crate::dsl::JfetType::P2n5460 => {
                                (Some("512-2N5460".to_string()), "2N5460 P-JFET".to_string())
                            }
                            _ => (None, "P-JFET".to_string()),
                        }
                    };
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: format!("P-JFET ({jt:?})"),
                        value: format!("{jt:?}"),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }]
                }
                ComponentKind::Photocoupler(pt) => {
                    let (pn, desc) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string())
                    } else {
                        match pt {
                            crate::dsl::PhotocouplerType::Vtl5c3 => {
                                (Some("VTL5C3".to_string()), "VTL5C3 Vactrol".to_string())
                            }
                            crate::dsl::PhotocouplerType::Vtl5c1 => {
                                (Some("VTL5C1".to_string()), "VTL5C1 Vactrol".to_string())
                            }
                            crate::dsl::PhotocouplerType::Nsl32 => {
                                (Some("NSL-32".to_string()), "NSL-32 Optocoupler".to_string())
                            }
                        }
                    };
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: format!("Vactrol ({pt:?})"),
                        value: format!("{pt:?}"),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }]
                }
                ComponentKind::Lfo(_wf, timing_r, timing_c) => {
                    let mut entries = Vec::new();

                    // Timing resistor
                    let r_val = crate::kicad::format_eng(*timing_r, "\u{2126}");
                    let (r_pn, r_desc) = find_closest(RESISTORS, *timing_r)
                        .map(|(pn, desc)| (Some(pn.to_string()), desc.to_string()))
                        .unwrap_or((None, format!("{r_val} LFO Timing Resistor")));
                    entries.push(BomEntry {
                        reference: format!("R_{}", comp.id),
                        display: "LFO Timing R".into(),
                        value: r_val,
                        mouser_pn: r_pn,
                        description: r_desc,
                        qty_per_unit: 1,
                    });

                    // Timing capacitor
                    let c_val = crate::kicad::format_eng(*timing_c, "F");
                    let (c_pn, c_desc) = find_closest_cap(*timing_c)
                        .map(|(pn, desc, _)| (Some(pn.to_string()), desc.to_string()))
                        .unwrap_or((None, format!("{c_val} LFO Timing Capacitor")));
                    entries.push(BomEntry {
                        reference: format!("C_{}", comp.id),
                        display: "LFO Timing C".into(),
                        value: c_val,
                        mouser_pn: c_pn,
                        description: c_desc,
                        qty_per_unit: 1,
                    });

                    entries
                }
                ComponentKind::Triode(tt) => {
                    let (pn, desc) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string())
                    } else {
                        match tt {
                            crate::dsl::TriodeType::T12ax7 => (
                                Some("JJ-12AX7".to_string()),
                                "12AX7 / ECC83 Preamp Tube (JJ Electronic)".to_string(),
                            ),
                            crate::dsl::TriodeType::T12at7 => (
                                Some("JJ-12AT7".to_string()),
                                "12AT7 / ECC81 Preamp Tube (JJ Electronic)".to_string(),
                            ),
                            crate::dsl::TriodeType::T12au7 => (
                                Some("JJ-12AU7".to_string()),
                                "12AU7 / ECC82 Preamp Tube (JJ Electronic)".to_string(),
                            ),
                            crate::dsl::TriodeType::T12ay7 => (
                                Some("JJ-12AY7".to_string()),
                                "12AY7 Preamp Tube (JJ Electronic)".to_string(),
                            ),
                        }
                    };
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: format!("Vacuum Tube ({tt:?})"),
                        value: format!("{tt:?}"),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }]
                }
                ComponentKind::Zener(vz) => {
                    let desc = if let Some(part_name) = hw_part {
                        part_name.to_string()
                    } else {
                        format!("{:.1}V Zener Diode", vz)
                    };
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: "Zener Diode".into(),
                        value: format!("{:.1}V", vz),
                        mouser_pn: None, // Zener voltage-specific P/N varies
                        description: desc,
                        qty_per_unit: 1,
                    }]
                }
                ComponentKind::Pentode(pt) => {
                    let (pn, desc) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string())
                    } else {
                        match pt {
                            crate::dsl::PentodeType::Ef86 => (
                                Some("JJ-EF86".to_string()),
                                "EF86 / 6267 Pentode (JJ Electronic)".to_string(),
                            ),
                            crate::dsl::PentodeType::El84 => (
                                Some("JJ-EL84".to_string()),
                                "EL84 / 6BQ5 Power Pentode (JJ Electronic)".to_string(),
                            ),
                        }
                    };
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: format!("Pentode ({pt:?})"),
                        value: format!("{pt:?}"),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }]
                }
                ComponentKind::Nmos(mt) => {
                    let (pn, desc) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string())
                    } else {
                        match mt {
                            crate::dsl::MosfetType::N2n7000 => {
                                (Some("512-2N7000".to_string()), "2N7000 N-MOSFET".to_string())
                            }
                            crate::dsl::MosfetType::Irf520 => {
                                (Some("942-IRF520NPBF".to_string()), "IRF520 N-MOSFET".to_string())
                            }
                            _ => (None, format!("N-MOSFET ({mt:?})")),
                        }
                    };
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: format!("N-MOSFET ({mt:?})"),
                        value: format!("{mt:?}"),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }]
                }
                ComponentKind::Pmos(mt) => {
                    let (pn, desc) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string())
                    } else {
                        match mt {
                            crate::dsl::MosfetType::Bs250 => {
                                (Some("512-BS250".to_string()), "BS250 P-MOSFET".to_string())
                            }
                            crate::dsl::MosfetType::Irf9520 => {
                                (Some("942-IRF9520NPBF".to_string()), "IRF9520 P-MOSFET".to_string())
                            }
                            _ => (None, format!("P-MOSFET ({mt:?})")),
                        }
                    };
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: format!("P-MOSFET ({mt:?})"),
                        value: format!("{mt:?}"),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }]
                }
                ComponentKind::Bbd(bt) => {
                    let (pn, desc) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string())
                    } else {
                        match bt {
                            crate::dsl::BbdType::Mn3207 => (
                                None, // NOS sourcing
                                "MN3207 1024-stage BBD".to_string(),
                            ),
                            crate::dsl::BbdType::Mn3007 => (
                                None,
                                "MN3007 1024-stage BBD (low-noise)".to_string(),
                            ),
                            crate::dsl::BbdType::Mn3005 => (
                                None,
                                "MN3005 4096-stage BBD (long delay)".to_string(),
                            ),
                        }
                    };
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: format!("BBD ({bt:?})"),
                        value: format!("{bt:?}"),
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }]
                }
                ComponentKind::EnvelopeFollower(
                    attack_r,
                    attack_c,
                    release_r,
                    release_c,
                    sens_r,
                ) => {
                    let mut entries = Vec::new();

                    // Attack timing resistor
                    let val = crate::kicad::format_eng(*attack_r, "\u{2126}");
                    let (pn, desc) = find_closest(RESISTORS, *attack_r)
                        .map(|(pn, desc)| (Some(pn.to_string()), desc.to_string()))
                        .unwrap_or((None, format!("{val} Attack Timing Resistor")));
                    entries.push(BomEntry {
                        reference: format!("R_{}_ATK", comp.id),
                        display: "Envelope Attack R".into(),
                        value: val,
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    });

                    // Attack timing capacitor
                    let val = crate::kicad::format_eng(*attack_c, "F");
                    let (pn, desc) = find_closest_cap(*attack_c)
                        .map(|(pn, desc, _)| (Some(pn.to_string()), desc.to_string()))
                        .unwrap_or((None, format!("{val} Attack Timing Capacitor")));
                    entries.push(BomEntry {
                        reference: format!("C_{}_ATK", comp.id),
                        display: "Envelope Attack C".into(),
                        value: val,
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    });

                    // Release timing resistor
                    let val = crate::kicad::format_eng(*release_r, "\u{2126}");
                    let (pn, desc) = find_closest(RESISTORS, *release_r)
                        .map(|(pn, desc)| (Some(pn.to_string()), desc.to_string()))
                        .unwrap_or((None, format!("{val} Release Timing Resistor")));
                    entries.push(BomEntry {
                        reference: format!("R_{}_REL", comp.id),
                        display: "Envelope Release R".into(),
                        value: val,
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    });

                    // Release timing capacitor
                    let val = crate::kicad::format_eng(*release_c, "F");
                    let (pn, desc) = find_closest_cap(*release_c)
                        .map(|(pn, desc, _)| (Some(pn.to_string()), desc.to_string()))
                        .unwrap_or((None, format!("{val} Release Timing Capacitor")));
                    entries.push(BomEntry {
                        reference: format!("C_{}_REL", comp.id),
                        display: "Envelope Release C".into(),
                        value: val,
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    });

                    // Sensitivity resistor
                    let val = crate::kicad::format_eng(*sens_r, "\u{2126}");
                    let (pn, desc) = find_closest(RESISTORS, *sens_r)
                        .map(|(pn, desc)| (Some(pn.to_string()), desc.to_string()))
                        .unwrap_or((None, format!("{val} Sensitivity Resistor")));
                    entries.push(BomEntry {
                        reference: format!("R_{}_SENS", comp.id),
                        display: "Envelope Sens R".into(),
                        value: val,
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    });

                    // Rectifier diode (1N4148)
                    entries.push(BomEntry {
                        reference: format!("D_{}", comp.id),
                        display: "Envelope Rectifier".into(),
                        value: "1N4148".into(),
                        mouser_pn: Some(SILICON_DIODE.mouser_pn.to_string()),
                        description: "1N4148 Envelope Rectifier Diode".to_string(),
                        qty_per_unit: 1,
                    });

                    entries
                }
                // ── Synth ICs ──────────────────────────────────────────
                ComponentKind::Vco(vt) => {
                    let (pn, desc, label) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string(), format!("{vt:?}"))
                    } else {
                        match vt {
                            crate::dsl::VcoType::Cem3340 => (
                                None, // NOS/specialty sourcing
                                "CEM3340 VCO (Curtis, NOS)".to_string(),
                                "CEM3340".to_string(),
                            ),
                            crate::dsl::VcoType::As3340 => (
                                Some("ALFA-AS3340".to_string()),
                                "AS3340 VCO (Alfa RPAR)".to_string(),
                                "AS3340".to_string(),
                            ),
                            crate::dsl::VcoType::V3340 => (
                                Some("COOLAUDIO-V3340".to_string()),
                                "V3340 VCO (CoolAudio)".to_string(),
                                "V3340".to_string(),
                            ),
                        }
                    };
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: "VCO".into(),
                        value: label,
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }]
                }
                ComponentKind::Vcf(vt) => {
                    let (pn, desc, label) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string(), format!("{vt:?}"))
                    } else {
                        match vt {
                            crate::dsl::VcfType::Cem3320 => (
                                None,
                                "CEM3320 VCF (Curtis, NOS)".to_string(),
                                "CEM3320".to_string(),
                            ),
                            crate::dsl::VcfType::As3320 => (
                                Some("ALFA-AS3320".to_string()),
                                "AS3320 VCF (Alfa RPAR)".to_string(),
                                "AS3320".to_string(),
                            ),
                        }
                    };
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: "VCF".into(),
                        value: label,
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }]
                }
                ComponentKind::Vca(vt) => {
                    let (pn, desc, label) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string(), format!("{vt:?}"))
                    } else {
                        match vt {
                            crate::dsl::VcaType::Ssm2164 => (
                                None,
                                "SSM2164 Quad VCA (NOS)".to_string(),
                                "SSM2164".to_string(),
                            ),
                            crate::dsl::VcaType::V2164 => (
                                Some("COOLAUDIO-V2164".to_string()),
                                "V2164 Quad VCA (CoolAudio)".to_string(),
                                "V2164".to_string(),
                            ),
                        }
                    };
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: "VCA".into(),
                        value: label,
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }]
                }
                ComponentKind::Comparator(ct) => {
                    let (pn, desc, label) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string(), format!("{ct:?}"))
                    } else {
                        match ct {
                            crate::dsl::ComparatorType::Lm311 => (
                                Some("595-LM311P".to_string()),
                                "LM311 Comparator".to_string(),
                                "LM311".to_string(),
                            ),
                            crate::dsl::ComparatorType::Lm393 => (
                                Some("595-LM393P".to_string()),
                                "LM393 Dual Comparator".to_string(),
                                "LM393".to_string(),
                            ),
                        }
                    };
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: "Comparator".into(),
                        value: label,
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }]
                }
                ComponentKind::AnalogSwitch(st) => {
                    let (pn, desc, label) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string(), format!("{st:?}"))
                    } else {
                        match st {
                            crate::dsl::AnalogSwitchType::Cd4066 => (
                                Some("595-CD4066BE".to_string()),
                                "CD4066 Quad Bilateral Switch".to_string(),
                                "CD4066".to_string(),
                            ),
                            crate::dsl::AnalogSwitchType::Dg411 => (
                                Some("700-DG411DJ-E3".to_string()),
                                "DG411 Quad SPST Switch".to_string(),
                                "DG411".to_string(),
                            ),
                        }
                    };
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: "Analog Switch".into(),
                        value: label,
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }]
                }
                ComponentKind::MatchedNpn(mt) | ComponentKind::MatchedPnp(mt) => {
                    let (pn, desc, label) = if let Some(part_name) = hw_part {
                        (None, part_name.to_string(), format!("{mt:?}"))
                    } else {
                        match mt {
                            crate::dsl::MatchedTransistorType::Ssm2210 => (
                                Some("584-SSM2210PZ".to_string()),
                                "SSM2210 Matched Dual NPN".to_string(),
                                "SSM2210".to_string(),
                            ),
                            crate::dsl::MatchedTransistorType::Ca3046 => (
                                None, // Discontinued, NOS sourcing
                                "CA3046 5-NPN Transistor Array (NOS)".to_string(),
                                "CA3046".to_string(),
                            ),
                            crate::dsl::MatchedTransistorType::Lm394 => (
                                None, // Discontinued, NOS sourcing
                                "LM394 Supermatch Pair (NOS)".to_string(),
                                "LM394".to_string(),
                            ),
                            crate::dsl::MatchedTransistorType::That340 => (
                                Some("THAT-340P14-U".to_string()),
                                "THAT340 Matched Quad NPN".to_string(),
                                "THAT340".to_string(),
                            ),
                        }
                    };
                    let is_npn = matches!(&comp.kind, ComponentKind::MatchedNpn(_));
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: if is_npn {
                            "Matched NPN".into()
                        } else {
                            "Matched PNP".into()
                        },
                        value: label,
                        mouser_pn: pn,
                        description: desc,
                        qty_per_unit: 1,
                    }]
                }
                ComponentKind::Tempco(r, ppm) => {
                    vec![BomEntry {
                        reference: comp.id.clone(),
                        display: "Tempco Resistor".into(),
                        value: format!(
                            "{} +{:.0}ppm/\u{b0}C",
                            crate::kicad::format_eng(*r, "\u{2126}"),
                            ppm
                        ),
                        mouser_pn: None, // Specialist supplier
                        description: format!(
                            "{} Tempco Resistor +{:.0}ppm/\u{b0}C",
                            crate::kicad::format_eng(*r, "\u{2126}"),
                            ppm
                        ),
                        qty_per_unit: 1,
                    }]
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

    fn find_example(filename: &str) -> String {
        fn walk(dir: &str, target: &str) -> Option<String> {
            for entry in std::fs::read_dir(dir).ok()?.flatten() {
                let path = entry.path();
                if path.is_dir() {
                    if let Some(found) = walk(path.to_str()?, target) {
                        return Some(found);
                    }
                } else if path.file_name().and_then(|n| n.to_str()) == Some(target) {
                    return Some(path.to_string_lossy().to_string());
                }
            }
            None
        }
        walk("examples", filename).unwrap_or_else(|| panic!("example file not found: {filename}"))
    }

    fn fuzz_face_pedal() -> PedalDef {
        let src = std::fs::read_to_string(find_example("fuzz_face.pedal")).unwrap();
        crate::dsl::parse_pedal_file(&src).unwrap()
    }

    fn tube_screamer_pedal() -> PedalDef {
        let src = std::fs::read_to_string(find_example("tube_screamer.pedal")).unwrap();
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
            let src = std::fs::read_to_string(find_example("klon_centaur.pedal")).unwrap();
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
        let src = std::fs::read_to_string(find_example("fuzz_face.pedalhw"));
        if let Ok(src) = src {
            let limits = parse_pedalhw(&src).unwrap();
            assert!(!limits.specs.is_empty());
        }
        // If file doesn't exist yet, skip — it's created by the example files step.
    }

    // ── BOM generation tests ─────────────────────────────────────────────

    fn klon_pedal() -> PedalDef {
        let src = std::fs::read_to_string(find_example("klon_centaur.pedal")).unwrap();
        crate::dsl::parse_pedal_file(&src).unwrap()
    }

    #[test]
    fn bom_tube_screamer_all_matched() {
        let pedal = tube_screamer_pedal();
        let bom = build_bom(&pedal, None);
        // Tube Screamer: 20 components total (R×7, C×6, U×2, D×2, Pot×3)
        assert_eq!(bom.len(), pedal.components.len());
        // All curated components should have a Mouser P/N.
        // Op-amps and well-known passives are all in the DB.
        let matched = bom.iter().filter(|e| e.mouser_pn.is_some()).count();
        assert!(matched >= 4, "At least Drive pot, C1, D1, Level pot should have Mouser P/Ns");
    }

    #[test]
    fn bom_fuzz_face_components() {
        let pedal = fuzz_face_pedal();
        let bom = build_bom(&pedal, None);
        // All components should produce BOM entries
        assert_eq!(bom.len(), pedal.components.len());
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
        // Build a pedal with a diode_pair to verify qty=2
        let src = r#"
pedal "Pair Test" {
  components {
    D1: diode_pair(silicon)
  }
  nets {
    in -> D1.a
    D1.b -> out
  }
}
"#;
        let pedal = crate::dsl::parse_pedal_file(src).unwrap();
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
        // Use a pedal with a diode_pair for qty multiplier testing
        let src = r#"
pedal "Qty Test" {
  components {
    D1: diode_pair(silicon)
    R1: resistor(10k)
  }
  nets {
    in -> D1.a, R1.a
    D1.b -> out
    R1.b -> gnd
  }
}
"#;
        let pedal = crate::dsl::parse_pedal_file(src).unwrap();
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
            let src = std::fs::read_to_string(find_example(f)).unwrap();
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
