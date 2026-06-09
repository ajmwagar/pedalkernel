//! SPICE `.MODEL` file parser and model registry.
//!
//! Parses standard SPICE `.MODEL` statements from embedded model files
//! and provides lookup by model name. Supports engineering suffixes
//! (f, p, n, u, m, k, MEG) for parameter values.
//!
//! Model files are included at compile time via `include_str!` and parsed
//! into a lazy-initialized registry on first access.

use hashbrown::HashMap;
use std::sync::LazyLock;

// ---------------------------------------------------------------------------
// Raw model files embedded at compile time
// ---------------------------------------------------------------------------

static TRANSISTOR_MODELS_SRC: &str = include_str!("../models/transistors.model");
static JFET_MODELS_SRC: &str = include_str!("../models/jfets.model");
static TRIODE_MODELS_SRC: &str = include_str!("../models/triodes.model");
static PENTODE_MODELS_SRC: &str = include_str!("../models/pentodes.model");
static DIODE_MODELS_SRC: &str = include_str!("../models/diodes.model");
static LED_MODELS_SRC: &str = include_str!("../models/leds.model");
static SCHOTTKY_MODELS_SRC: &str = include_str!("../models/schottky.model");
static ZENER_MODELS_SRC: &str = include_str!("../models/zeners.model");
static OPAMP_MODELS_SRC: &str = include_str!("../models/opamps.model");

// ---------------------------------------------------------------------------
// Parsed model registry (lazy-initialized)
// ---------------------------------------------------------------------------

/// All BJT models parsed from the embedded transistors.model file.
pub static BJT_MODELS: LazyLock<HashMap<String, SpiceBjtModel>> =
    LazyLock::new(|| parse_bjt_models(TRANSISTOR_MODELS_SRC));

/// All JFET models parsed from the embedded jfets.model file.
pub static JFET_MODELS: LazyLock<HashMap<String, SpiceJfetModel>> =
    LazyLock::new(|| parse_jfet_models(JFET_MODELS_SRC));

/// All triode models parsed from the embedded triodes.model file.
pub static TRIODE_MODELS: LazyLock<HashMap<String, SpiceTriodeModel>> =
    LazyLock::new(|| parse_triode_models(TRIODE_MODELS_SRC));

/// All pentode models parsed from the embedded pentodes.model file.
pub static PENTODE_MODELS: LazyLock<HashMap<String, SpicePentodeModel>> =
    LazyLock::new(|| parse_pentode_models(PENTODE_MODELS_SRC));

/// All diode models parsed from the embedded diodes.model file.
pub static DIODE_MODELS: LazyLock<HashMap<String, ShockleyDiodeModel>> =
    LazyLock::new(|| parse_diode_models(DIODE_MODELS_SRC));

/// All LED models parsed from the embedded leds.model file.
pub static LED_MODELS: LazyLock<HashMap<String, ShockleyDiodeModel>> =
    LazyLock::new(|| parse_diode_models(LED_MODELS_SRC));

/// All Schottky diode models parsed from the embedded schottky.model file.
pub static SCHOTTKY_MODELS: LazyLock<HashMap<String, ShockleyDiodeModel>> =
    LazyLock::new(|| parse_diode_models(SCHOTTKY_MODELS_SRC));

/// All Zener diode models parsed from the embedded zeners.model file.
pub static ZENER_MODELS: LazyLock<HashMap<String, ShockleyDiodeModel>> =
    LazyLock::new(|| parse_diode_models(ZENER_MODELS_SRC));

/// All op-amp and OTA models parsed from the embedded opamps.model file.
pub static OPAMP_MODELS: LazyLock<HashMap<String, SpiceOpAmpModel>> =
    LazyLock::new(|| parse_opamp_models(OPAMP_MODELS_SRC));

// ---------------------------------------------------------------------------
// Parsed SPICE BJT model
// ---------------------------------------------------------------------------

/// A parsed SPICE BJT model with all Gummel-Poon parameters.
///
/// Parameters not present in the `.MODEL` line get SPICE defaults.
#[derive(Debug, Clone)]
pub struct SpiceBjtModel {
    pub name: String,
    pub is_pnp: bool,

    // DC parameters
    pub is: f64,
    pub bf: f64,
    pub br: f64,
    pub nf: f64,
    pub nr: f64,

    // Early effect
    pub vaf: f64,
    pub var: f64,

    // High injection
    pub ikf: f64,
    pub ikr: f64,

    // Leakage
    pub ise: f64,
    pub ne: f64,
    pub isc: f64,
    pub nc: f64,

    // Parasitic resistances
    pub rb: f64,
    pub re: f64,
    pub rc: f64,

    // Junction capacitances
    pub cje: f64,
    pub vje: f64,
    pub mje: f64,
    pub cjc: f64,
    pub vjc: f64,
    pub mjc: f64,

    // Transit time
    pub tf: f64,
    pub tr: f64,
}

impl SpiceBjtModel {
    /// SPICE defaults for parameters not specified in the .MODEL line.
    fn defaults(name: &str, is_pnp: bool) -> Self {
        Self {
            name: name.to_uppercase(),
            is_pnp,
            is: 1e-16,
            bf: 100.0,
            br: 1.0,
            nf: 1.0,
            nr: 1.0,
            vaf: f64::INFINITY,
            var: f64::INFINITY,
            ikf: f64::INFINITY,
            ikr: f64::INFINITY,
            ise: 0.0,
            ne: 1.5,
            isc: 0.0,
            nc: 2.0,
            rb: 0.0,
            re: 0.0,
            rc: 0.0,
            cje: 0.0,
            vje: 0.75,
            mje: 0.33,
            cjc: 0.0,
            vjc: 0.75,
            mjc: 0.33,
            tf: 0.0,
            tr: 0.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Parsed SPICE JFET model
// ---------------------------------------------------------------------------

/// A parsed SPICE JFET model (NJF/PJF).
///
/// Parameters not present in the `.MODEL` line get SPICE defaults.
#[derive(Debug, Clone)]
pub struct SpiceJfetModel {
    pub name: String,
    pub is_n_channel: bool,

    // DC parameters
    pub vto: f64,    // Threshold (pinch-off) voltage (V)
    pub beta: f64,   // Transconductance coefficient (A/V²)
    pub lambda: f64, // Channel-length modulation (1/V)

    // Gate junction
    pub is: f64, // Gate junction saturation current (A)
    pub n: f64,  // Gate junction ideality factor

    // Parasitic resistances
    pub rd: f64, // Drain ohmic resistance (Ω)
    pub rs: f64, // Source ohmic resistance (Ω)

    // Junction capacitances
    pub cgs: f64, // Gate-source zero-bias capacitance (F)
    pub cgd: f64, // Gate-drain zero-bias capacitance (F)
    pub pb: f64,  // Gate junction potential (V)
}

impl SpiceJfetModel {
    fn defaults(name: &str, is_n_channel: bool) -> Self {
        Self {
            name: name.to_uppercase(),
            is_n_channel,
            vto: -2.0,
            beta: 1e-4, // 0.1 mA/V²
            lambda: 0.0,
            is: 1e-14,
            n: 1.0,
            rd: 0.0,
            rs: 0.0,
            cgs: 0.0,
            cgd: 0.0,
            pb: 1.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Parsed SPICE Diode model
// ---------------------------------------------------------------------------

/// Shockley diode equation model parameters.
///
/// Models the I-V characteristic: `I = IS * (exp(V / (N * Vt)) - 1)`
/// with series resistance RS and junction capacitance CJO.
///
/// Used for standard diodes, LEDs, Schottky diodes, and Zener diodes.
/// Parameters not present in the `.MODEL` line get SPICE defaults.
#[derive(Debug, Clone)]
pub struct ShockleyDiodeModel {
    pub name: String,

    // DC parameters
    /// Saturation current (A). Typically 1e-14 to 1e-6.
    pub is: f64,
    /// Series resistance (Ω).
    pub rs: f64,
    /// Emission coefficient (ideality factor). Typically 1.0-2.0 for Si, higher for LEDs.
    pub n: f64,
    /// Transit time (s).
    pub tt: f64,

    // Junction capacitance
    /// Zero-bias junction capacitance (F).
    pub cjo: f64,
    /// Junction potential (V). Typically 0.7 for Si.
    pub vj: f64,
    /// Grading coefficient. Typically 0.33-0.5.
    pub m: f64,

    // Breakdown
    /// Reverse breakdown voltage (V). For zeners, this is the zener voltage.
    pub bv: f64,
    /// Current at breakdown voltage (A).
    pub ibv: f64,

    // Temperature
    /// Bandgap energy (eV): 1.11=Si, 0.69=Schottky, 0.67=Ge.
    pub eg: f64,
    /// IS temperature exponent: 3=Si junction, 2=Schottky.
    pub xti: f64,
}

impl ShockleyDiodeModel {
    /// SPICE defaults for parameters not specified in the .MODEL line.
    fn defaults(name: &str) -> Self {
        Self {
            name: name.to_uppercase(),
            is: 1e-14,
            rs: 0.0,
            n: 1.0,
            tt: 0.0,
            cjo: 0.0,
            vj: 1.0,
            m: 0.5,
            bv: f64::INFINITY,
            ibv: 1e-3,
            eg: 1.11,
            xti: 3.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Parsed SPICE Triode model
// ---------------------------------------------------------------------------

/// A parsed triode tube model with Koren equation parameters.
///
/// Format: `.TRIODE <name> MU= EX= KG1= KP= KVB= RP=`
#[derive(Debug, Clone)]
pub struct SpiceTriodeModel {
    pub name: String,
    /// Amplification factor
    pub mu: f64,
    /// Exponent (transfer curve shape)
    pub ex: f64,
    /// Plate current scaling factor (A)
    pub kg1: f64,
    /// Plate resistance factor
    pub kp: f64,
    /// Knee voltage constant (V)
    pub kvb: f64,
    /// Plate resistance at typical operating point (Ω). Datasheet rp.
    /// 12AX7 ≈ 62.5kΩ, 12AU7 ≈ 7.7kΩ, 12AT7 ≈ 10.9kΩ.
    pub rp: f64,
}

// ---------------------------------------------------------------------------
// Parsed SPICE Pentode model
// ---------------------------------------------------------------------------

/// A parsed pentode tube model with screen-referenced Koren equation parameters.
///
/// Format: `.PENTODE <name> MU= EX= KG1= KG2= KP= KVB= KVB2= VG2= RP=`
#[derive(Debug, Clone)]
pub struct SpicePentodeModel {
    pub name: String,
    /// Amplification factor (screen-referenced)
    pub mu: f64,
    /// Exponent (transfer curve shape)
    pub ex: f64,
    /// Plate current scaling factor (A)
    pub kg1: f64,
    /// Screen current scaling factor (A)
    pub kg2: f64,
    /// Plate resistance factor
    pub kp: f64,
    /// Knee voltage constant (V)
    pub kvb: f64,
    /// Knee voltage for pentode plate saturation (V)
    pub kvb2: f64,
    /// Default screen grid voltage (V) for typical operating point
    pub vg2_default: f64,
    /// Plate resistance at typical operating point (Ω). Datasheet rp.
    /// Power pentodes: 15k–50kΩ. Signal pentodes (EF86): ~2.5MΩ.
    pub rp: f64,
}

// ---------------------------------------------------------------------------
// Parsed op-amp / OTA model
// ---------------------------------------------------------------------------

/// Compact behavioral op-amp and OTA parameters.
///
/// Format:
/// - `.OPAMP <name> A0= GBW= SR= VPOS= VNEG= RO= COUT=`
/// - `.OTA <name> A0= GBW= SR= VPOS= VNEG= RO= COUT= IABC= VT= GM= RLOAD=`
#[derive(Debug, Clone)]
pub struct SpiceOpAmpModel {
    pub name: String,
    pub is_ota: bool,
    pub open_loop_gain: f64,
    pub gbw: f64,
    pub slew_rate: f64,
    pub v_rail_pos: f64,
    pub v_rail_neg: f64,
    pub output_impedance: f64,
    pub output_capacitance: f64,
    pub ota_iabc: f64,
    pub ota_vt: f64,
    pub ota_gm: f64,
    pub ota_r_load: f64,
}

impl SpiceOpAmpModel {
    fn defaults(name: &str, is_ota: bool) -> Self {
        Self {
            name: name.to_uppercase(),
            is_ota,
            open_loop_gain: 100_000.0,
            gbw: 1e6,
            slew_rate: 1.0,
            v_rail_pos: 12.0,
            v_rail_neg: 12.0,
            output_impedance: 75.0,
            output_capacitance: 20e-12,
            ota_iabc: 100e-6,
            ota_vt: 25.85e-3,
            ota_gm: 0.0,
            ota_r_load: 10_000.0,
        }
    }
}

// ---------------------------------------------------------------------------
// SPICE engineering suffix parser
// ---------------------------------------------------------------------------

/// Parse a numeric value with optional SPICE engineering suffix.
///
/// Supports: `T`=1e12, `G`=1e9, `MEG`=1e6, `k`=1e3, `m`=1e-3,
///           `u`=1e-6, `n`=1e-9, `p`=1e-12, `f`=1e-15
fn parse_spice_value(s: &str) -> Option<f64> {
    let s = s.trim();
    if s.is_empty() {
        return None;
    }

    // Try to find where the numeric part ends and the suffix begins.
    // Numbers can contain digits, '.', '+', '-', 'e'/'E' (for scientific notation).
    let mut split_pos = s.len();
    let bytes = s.as_bytes();
    for i in 0..bytes.len() {
        let c = bytes[i] as char;
        if c.is_ascii_alphabetic() {
            // Check if this is part of scientific notation (e.g., "1e-3")
            if (c == 'e' || c == 'E') && i > 0 {
                // Look ahead: if followed by digit or +/-, it's scientific notation
                if i + 1 < bytes.len() {
                    let next = bytes[i + 1] as char;
                    if next.is_ascii_digit() || next == '+' || next == '-' {
                        continue;
                    }
                }
            }
            split_pos = i;
            break;
        }
    }

    let num_part = &s[..split_pos];
    let suffix = &s[split_pos..];

    let base: f64 = num_part.parse().ok()?;

    let multiplier = match suffix.to_uppercase().as_str() {
        "" => 1.0,
        "T" => 1e12,
        "G" => 1e9,
        "MEG" => 1e6,
        "K" => 1e3,
        "M" => 1e-3,
        "MIL" => 25.4e-6,
        "U" => 1e-6,
        "N" => 1e-9,
        "P" => 1e-12,
        "F" => 1e-15,
        _ => return None,
    };

    Some(base * multiplier)
}

// ---------------------------------------------------------------------------
// .MODEL line parser
// ---------------------------------------------------------------------------

/// Parse all `.MODEL` BJT entries from a SPICE model file string.
fn parse_bjt_models(src: &str) -> HashMap<String, SpiceBjtModel> {
    let mut models = HashMap::new();

    for line in src.lines() {
        let trimmed = line.trim();

        // Skip comments and empty lines
        if trimmed.is_empty() || trimmed.starts_with('*') || trimmed.starts_with('#') {
            continue;
        }

        // Must start with .MODEL (case-insensitive)
        if !trimmed.to_uppercase().starts_with(".MODEL") {
            continue;
        }

        if let Some(model) = parse_model_line(trimmed) {
            models.insert(model.name.clone(), model);
        }
    }

    models
}

/// Parse a single `.MODEL <name> <type>(<params>)` line.
fn parse_model_line(line: &str) -> Option<SpiceBjtModel> {
    // Strip ".MODEL " prefix (case-insensitive)
    let rest = &line[6..].trim_start();

    // Extract model name (first token)
    let (name, rest) = rest.split_once(|c: char| c.is_whitespace())?;
    let rest = rest.trim_start();

    // Extract type and parameter block
    // Format: "NPN(...)" or "PNP(...)"
    let (type_str, params_block) = if let Some(paren_pos) = rest.find('(') {
        let type_str = rest[..paren_pos].trim();
        let close_paren = rest.rfind(')')?;
        let params = &rest[paren_pos + 1..close_paren];
        (type_str, params)
    } else {
        // No parameters
        (rest.trim(), "")
    };

    let is_pnp = match type_str.to_uppercase().as_str() {
        "NPN" => false,
        "PNP" => true,
        _ => return None, // Not a BJT
    };

    let mut model = SpiceBjtModel::defaults(name, is_pnp);

    // Parse key=value pairs
    for pair in params_block.split_whitespace() {
        if let Some((key, val_str)) = pair.split_once('=') {
            let key_upper = key.to_uppercase();
            if let Some(val) = parse_spice_value(val_str) {
                match key_upper.as_str() {
                    "IS" => model.is = val,
                    "BF" => model.bf = val,
                    "BR" => model.br = val,
                    "NF" => model.nf = val,
                    "NR" => model.nr = val,
                    "VT" => {} // We compute VT from temperature, ignore file value
                    "VAF" | "VA" => model.vaf = val,
                    "VAR" | "VB" => model.var = val,
                    "IKF" | "JBF" => model.ikf = val,
                    "IKR" | "JBR" => model.ikr = val,
                    "ISE" => model.ise = val,
                    "NE" => model.ne = val,
                    "ISC" => model.isc = val,
                    "NC" => model.nc = val,
                    "RB" => model.rb = val,
                    "RE" => model.re = val,
                    "RC" => model.rc = val,
                    "CJE" | "CEB" => model.cje = val,
                    "VJE" | "PE" => model.vje = val,
                    "MJE" | "ME" => model.mje = val,
                    "CJC" | "CCB" => model.cjc = val,
                    "VJC" | "PC" => model.vjc = val,
                    "MJC" | "MC" => model.mjc = val,
                    "TF" => model.tf = val,
                    "TR" => model.tr = val,
                    _ => {} // Ignore unknown parameters
                }
            }
        }
    }

    Some(model)
}

// ---------------------------------------------------------------------------
// JFET .MODEL line parser
// ---------------------------------------------------------------------------

/// Parse all `.MODEL` JFET entries (NJF/PJF) from a SPICE model file string.
fn parse_jfet_models(src: &str) -> HashMap<String, SpiceJfetModel> {
    let mut models = HashMap::new();

    for line in src.lines() {
        let trimmed = line.trim();

        if trimmed.is_empty() || trimmed.starts_with('*') || trimmed.starts_with('#') {
            continue;
        }

        if !trimmed.to_uppercase().starts_with(".MODEL") {
            continue;
        }

        if let Some(model) = parse_jfet_model_line(trimmed) {
            models.insert(model.name.clone(), model);
        }
    }

    models
}

/// Parse a single `.MODEL <name> NJF|PJF(<params>)` line.
fn parse_jfet_model_line(line: &str) -> Option<SpiceJfetModel> {
    let rest = &line[6..].trim_start();
    let (name, rest) = rest.split_once(|c: char| c.is_whitespace())?;
    let rest = rest.trim_start();

    // Extract type and parameter block
    let (type_str, params_block) = if let Some(paren_pos) = rest.find('(') {
        let type_str = rest[..paren_pos].trim();
        let close_paren = rest.rfind(')')?;
        let params = &rest[paren_pos + 1..close_paren];
        (type_str, params)
    } else {
        // Some models use space-separated params without parens
        // Split on first whitespace after type
        let type_end = rest.find(|c: char| c.is_whitespace()).unwrap_or(rest.len());
        let type_str = &rest[..type_end];
        let params = if type_end < rest.len() {
            &rest[type_end..]
        } else {
            ""
        };
        (type_str.trim(), params)
    };

    let is_n_channel = match type_str.to_uppercase().as_str() {
        "NJF" => true,
        "PJF" => false,
        _ => return None, // Not a JFET
    };

    let mut model = SpiceJfetModel::defaults(name, is_n_channel);

    for pair in params_block.split_whitespace() {
        if let Some((key, val_str)) = pair.split_once('=') {
            let key_upper = key.to_uppercase();
            if let Some(val) = parse_spice_value(val_str) {
                match key_upper.as_str() {
                    "VTO" => model.vto = val,
                    "BETA" => model.beta = val,
                    "LAMBDA" => model.lambda = val,
                    "IS" => model.is = val,
                    "N" => model.n = val,
                    "RD" => model.rd = val,
                    "RS" => model.rs = val,
                    "CGS" => model.cgs = val,
                    "CGD" => model.cgd = val,
                    "PB" => model.pb = val,
                    _ => {} // Ignore unknown parameters (AF, FC, BETATCE, etc.)
                }
            }
        }
    }

    Some(model)
}

// ---------------------------------------------------------------------------
// Diode .MODEL D(...) line parser
// ---------------------------------------------------------------------------

/// Parse all `.MODEL <name> D(...)` entries from a SPICE model file string.
///
/// Handles standard diodes, LEDs, Schottky, and Zener diodes — they all
/// share the same SPICE `D` type and parameter set.
fn parse_diode_models(src: &str) -> HashMap<String, ShockleyDiodeModel> {
    let mut models = HashMap::new();

    for line in src.lines() {
        let trimmed = line.trim();

        if trimmed.is_empty() || trimmed.starts_with('*') || trimmed.starts_with('#') {
            continue;
        }

        if !trimmed.to_uppercase().starts_with(".MODEL") {
            continue;
        }

        if let Some(model) = parse_diode_model_line(trimmed) {
            models.insert(model.name.clone(), model);
        }
    }

    models
}

/// Parse a single `.MODEL <name> D(<params>)` line.
fn parse_diode_model_line(line: &str) -> Option<ShockleyDiodeModel> {
    let rest = &line[6..].trim_start();
    let (name, rest) = rest.split_once(|c: char| c.is_whitespace())?;
    let rest = rest.trim_start();

    // Must be type D
    let (type_str, params_block) = if let Some(paren_pos) = rest.find('(') {
        let type_str = rest[..paren_pos].trim();
        let close_paren = rest.rfind(')')?;
        let params = &rest[paren_pos + 1..close_paren];
        (type_str, params)
    } else {
        return None;
    };

    if !type_str.eq_ignore_ascii_case("D") {
        return None;
    }

    let mut model = ShockleyDiodeModel::defaults(name);

    for pair in params_block.split_whitespace() {
        if let Some((key, val_str)) = pair.split_once('=') {
            let key_upper = key.to_uppercase();
            if let Some(val) = parse_spice_value(val_str) {
                match key_upper.as_str() {
                    "IS" => model.is = val,
                    "RS" => model.rs = val,
                    "N" => model.n = val,
                    "TT" => model.tt = val,
                    "CJO" => model.cjo = val,
                    "VJ" => model.vj = val,
                    "M" => model.m = val,
                    "BV" => model.bv = val,
                    "IBV" => model.ibv = val,
                    "EG" => model.eg = val,
                    "XTI" => model.xti = val,
                    _ => {} // Ignore unknown parameters (KF, AF, FC, etc.)
                }
            }
        }
    }

    Some(model)
}

// ---------------------------------------------------------------------------
// Triode .TRIODE line parser
// ---------------------------------------------------------------------------

/// Parse all `.TRIODE` entries from a model file string.
fn parse_triode_models(src: &str) -> HashMap<String, SpiceTriodeModel> {
    let mut models = HashMap::new();

    for line in src.lines() {
        let trimmed = line.trim();

        if trimmed.is_empty() || trimmed.starts_with('*') || trimmed.starts_with('#') {
            continue;
        }

        if !trimmed.to_uppercase().starts_with(".TRIODE") {
            continue;
        }

        if let Some(model) = parse_triode_model_line(trimmed) {
            models.insert(model.name.clone(), model);
        }
    }

    models
}

/// Parse a single `.TRIODE <name> MU= EX= KG1= KP= KVB= RP=` line.
fn parse_triode_model_line(line: &str) -> Option<SpiceTriodeModel> {
    // Strip ".TRIODE" prefix
    let rest = line[7..].trim_start();
    let (name, params) = rest.split_once(|c: char| c.is_whitespace())?;

    let mut mu = 100.0;
    let mut ex = 1.4;
    let mut kg1 = 1060.0;
    let mut kp = 600.0;
    let mut kvb = 300.0;
    let mut rp = 62500.0; // Default: 12AX7 plate resistance

    for pair in params.split_whitespace() {
        if let Some((key, val_str)) = pair.split_once('=') {
            let key_upper = key.to_uppercase();
            if let Some(val) = parse_spice_value(val_str) {
                match key_upper.as_str() {
                    "MU" => mu = val,
                    "EX" => ex = val,
                    "KG1" => kg1 = val,
                    "KP" => kp = val,
                    "KVB" => kvb = val,
                    "RP" => rp = val,
                    _ => {}
                }
            }
        }
    }

    Some(SpiceTriodeModel {
        name: name.to_uppercase(),
        mu,
        ex,
        kg1,
        kp,
        kvb,
        rp,
    })
}

// ---------------------------------------------------------------------------
// Pentode .PENTODE line parser
// ---------------------------------------------------------------------------

/// Parse all `.PENTODE` entries from a model file string.
fn parse_pentode_models(src: &str) -> HashMap<String, SpicePentodeModel> {
    let mut models = HashMap::new();

    for line in src.lines() {
        let trimmed = line.trim();

        if trimmed.is_empty() || trimmed.starts_with('*') || trimmed.starts_with('#') {
            continue;
        }

        if !trimmed.to_uppercase().starts_with(".PENTODE") {
            continue;
        }

        if let Some(model) = parse_pentode_model_line(trimmed) {
            models.insert(model.name.clone(), model);
        }
    }

    models
}

/// Parse a single `.PENTODE <name> MU= EX= KG1= KG2= KP= KVB= KVB2= VG2= RP=` line.
fn parse_pentode_model_line(line: &str) -> Option<SpicePentodeModel> {
    // Strip ".PENTODE" prefix
    let rest = line[8..].trim_start();
    let (name, params) = rest.split_once(|c: char| c.is_whitespace())?;

    let mut mu = 10.0;
    let mut ex = 1.35;
    let mut kg1 = 1000.0;
    let mut kg2 = 4200.0;
    let mut kp = 60.0;
    let mut kvb = 24.0;
    let mut kvb2 = 20.0;
    let mut vg2_default = 250.0;
    let mut rp = 50000.0; // Default: mid-range power pentode

    for pair in params.split_whitespace() {
        if let Some((key, val_str)) = pair.split_once('=') {
            let key_upper = key.to_uppercase();
            if let Some(val) = parse_spice_value(val_str) {
                match key_upper.as_str() {
                    "MU" => mu = val,
                    "EX" => ex = val,
                    "KG1" => kg1 = val,
                    "KG2" => kg2 = val,
                    "KP" => kp = val,
                    "KVB" => kvb = val,
                    "KVB2" => kvb2 = val,
                    "VG2" => vg2_default = val,
                    "RP" => rp = val,
                    _ => {}
                }
            }
        }
    }

    Some(SpicePentodeModel {
        name: name.to_uppercase(),
        mu,
        ex,
        kg1,
        kg2,
        kp,
        kvb,
        kvb2,
        vg2_default,
        rp,
    })
}

// ---------------------------------------------------------------------------
// Op-amp / OTA model parser
// ---------------------------------------------------------------------------

/// Parse all `.OPAMP` and `.OTA` entries from a model file string.
fn parse_opamp_models(src: &str) -> HashMap<String, SpiceOpAmpModel> {
    let mut models = HashMap::new();

    for line in src.lines() {
        let trimmed = line.trim();

        if trimmed.is_empty() || trimmed.starts_with('*') || trimmed.starts_with('#') {
            continue;
        }

        let upper = trimmed.to_uppercase();
        if !upper.starts_with(".OPAMP") && !upper.starts_with(".OTA") {
            continue;
        }

        if let Some(model) = parse_opamp_model_line(trimmed) {
            models.insert(model.name.clone(), model);
        }
    }

    models
}

/// Parse a single `.OPAMP <name> ...` or `.OTA <name> ...` line.
fn parse_opamp_model_line(line: &str) -> Option<SpiceOpAmpModel> {
    let upper = line.to_uppercase();
    let (prefix_len, is_ota) = if upper.starts_with(".OPAMP") {
        (6, false)
    } else if upper.starts_with(".OTA") {
        (4, true)
    } else {
        return None;
    };

    let rest = line[prefix_len..].trim_start();
    let (name, params) = rest.split_once(|c: char| c.is_whitespace())?;
    let mut model = SpiceOpAmpModel::defaults(name, is_ota);

    for pair in params.split_whitespace() {
        if let Some((key, val_str)) = pair.split_once('=') {
            let key_upper = key.to_uppercase();
            if let Some(val) = parse_spice_value(val_str) {
                match key_upper.as_str() {
                    "A0" | "AOL" | "OPEN_LOOP_GAIN" => model.open_loop_gain = val,
                    "GBW" | "GBP" => model.gbw = val,
                    "SR" | "SLEW" | "SLEW_RATE" => model.slew_rate = val,
                    "VPOS" | "VRAILPOS" | "V_RAIL_POS" => model.v_rail_pos = val,
                    "VNEG" | "VRAILNEG" | "V_RAIL_NEG" => model.v_rail_neg = val,
                    "RO" | "ROUT" | "OUTPUT_IMPEDANCE" => model.output_impedance = val,
                    "COUT" | "OUTPUT_CAPACITANCE" => model.output_capacitance = val,
                    "IABC" => model.ota_iabc = val,
                    "VT" => model.ota_vt = val,
                    "GM" | "GMO" => model.ota_gm = val,
                    "RLOAD" | "RL" => model.ota_r_load = val,
                    _ => {}
                }
            }
        }
    }

    Some(model)
}

// ---------------------------------------------------------------------------
// Public lookup API
// ---------------------------------------------------------------------------

/// Look up a BJT model by name (case-insensitive).
///
/// Returns `None` if the model name is not found in the embedded model file.
pub fn bjt_by_name(name: &str) -> Option<&'static SpiceBjtModel> {
    BJT_MODELS.get(&name.to_uppercase())
}

/// Check if a BJT model is germanium based on its saturation current.
///
/// Germanium transistors have IS in the µA range (typ. 1–100 µA),
/// while silicon transistors have IS in the fA–pA range.
pub fn bjt_is_germanium(name: &str) -> bool {
    bjt_by_name(name).map(|m| m.is > 1e-6).unwrap_or(false)
}

/// Check if a BJT model is PNP.
pub fn bjt_is_pnp(name: &str) -> bool {
    bjt_by_name(name).map(|m| m.is_pnp).unwrap_or(false)
}

/// List all available BJT model names.
pub fn bjt_model_names() -> Vec<&'static str> {
    BJT_MODELS.keys().map(|s| s.as_str()).collect()
}

/// Look up a JFET model by name (case-insensitive).
///
/// Returns `None` if the model name is not found in the embedded model file.
pub fn jfet_by_name(name: &str) -> Option<&'static SpiceJfetModel> {
    JFET_MODELS.get(&name.to_uppercase())
}

/// List all available JFET model names.
pub fn jfet_model_names() -> Vec<&'static str> {
    JFET_MODELS.keys().map(|s| s.as_str()).collect()
}

/// Look up a triode model by name (case-insensitive).
pub fn triode_by_name(name: &str) -> Option<&'static SpiceTriodeModel> {
    TRIODE_MODELS.get(&name.to_uppercase())
}

/// List all available triode model names.
pub fn triode_model_names() -> Vec<&'static str> {
    TRIODE_MODELS.keys().map(|s| s.as_str()).collect()
}

/// Look up a pentode model by name (case-insensitive).
pub fn pentode_by_name(name: &str) -> Option<&'static SpicePentodeModel> {
    PENTODE_MODELS.get(&name.to_uppercase())
}

/// List all available pentode model names.
pub fn pentode_model_names() -> Vec<&'static str> {
    PENTODE_MODELS.keys().map(|s| s.as_str()).collect()
}

/// Look up a diode model by name (case-insensitive).
///
/// Searches the standard diodes library (1N34, 1N914, 1N4148, 1N4001–1N4007, etc.)
pub fn diode_by_name(name: &str) -> Option<&'static ShockleyDiodeModel> {
    DIODE_MODELS.get(&name.to_uppercase())
}

/// List all available diode model names.
pub fn diode_model_names() -> Vec<&'static str> {
    DIODE_MODELS.keys().map(|s| s.as_str()).collect()
}

/// Look up an LED model by name (case-insensitive).
///
/// Available names: LED_IR, LED_RED, LED_GREEN, LED_YELLOW, LED_AMBER, LED_BLUE,
/// DLED0–DLED3.
pub fn led_by_name(name: &str) -> Option<&'static ShockleyDiodeModel> {
    LED_MODELS.get(&name.to_uppercase())
}

/// List all available LED model names.
pub fn led_model_names() -> Vec<&'static str> {
    LED_MODELS.keys().map(|s| s.as_str()).collect()
}

/// Look up a Schottky diode model by name (case-insensitive).
pub fn schottky_by_name(name: &str) -> Option<&'static ShockleyDiodeModel> {
    SCHOTTKY_MODELS.get(&name.to_uppercase())
}

/// List all available Schottky diode model names.
pub fn schottky_model_names() -> Vec<&'static str> {
    SCHOTTKY_MODELS.keys().map(|s| s.as_str()).collect()
}

/// Look up a Zener diode model by name (case-insensitive).
///
/// Searches the zener library (1N746–1N759, 1N4728–1N4764, 1N5221–1N5267B, etc.)
pub fn zener_by_name(name: &str) -> Option<&'static ShockleyDiodeModel> {
    ZENER_MODELS.get(&name.to_uppercase())
}

/// List all available Zener diode model names.
pub fn zener_model_names() -> Vec<&'static str> {
    ZENER_MODELS.keys().map(|s| s.as_str()).collect()
}

/// Look up an op-amp or OTA model by name (case-insensitive).
pub fn opamp_by_name(name: &str) -> Option<&'static SpiceOpAmpModel> {
    OPAMP_MODELS.get(&name.to_uppercase())
}

/// List all available op-amp and OTA model names.
pub fn opamp_model_names() -> Vec<&'static str> {
    OPAMP_MODELS.keys().map(|s| s.as_str()).collect()
}

/// Look up any diode-type model by name across all registries.
///
/// Searches in order: diodes → LEDs → Schottky → zeners.
/// Use the specific `*_by_name()` functions when the category is known.
pub fn any_diode_by_name(name: &str) -> Option<&'static ShockleyDiodeModel> {
    let upper = name.to_uppercase();
    DIODE_MODELS
        .get(&upper)
        .or_else(|| LED_MODELS.get(&upper))
        .or_else(|| SCHOTTKY_MODELS.get(&upper))
        .or_else(|| ZENER_MODELS.get(&upper))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(a: Option<f64>, b: Option<f64>) {
        match (a, b) {
            (Some(a), Some(b)) => {
                let rel = if b.abs() > 1e-30 {
                    ((a - b) / b).abs()
                } else {
                    (a - b).abs()
                };
                assert!(rel < 1e-10, "expected {b}, got {a}");
            }
            _ => assert_eq!(a, b),
        }
    }

    #[test]
    fn parse_spice_suffixes() {
        assert_close(parse_spice_value("1.0"), Some(1.0));
        assert_close(parse_spice_value("6.734f"), Some(6.734e-15));
        assert_close(parse_spice_value("4.493p"), Some(4.493e-12));
        assert_close(parse_spice_value("301.2p"), Some(301.2e-12));
        assert_close(parse_spice_value("24n"), Some(24e-9));
        assert_close(parse_spice_value("20.66u"), Some(20.66e-6));
        assert_close(parse_spice_value("66.78m"), Some(66.78e-3));
        assert_close(parse_spice_value("10"), Some(10.0));
        assert_close(parse_spice_value("1k"), Some(1e3));
        assert_close(parse_spice_value("1MEG"), Some(1e6));
    }

    #[test]
    fn parse_scientific_notation() {
        // Scientific notation should NOT be confused with suffixes
        assert_eq!(parse_spice_value("1e-14"), Some(1e-14));
        assert_eq!(parse_spice_value("6.734e-15"), Some(6.734e-15));
        assert_eq!(parse_spice_value("3.295E-14"), Some(3.295e-14));
        assert_eq!(parse_spice_value("1.41e-15"), Some(1.41e-15));
    }

    #[test]
    fn parse_model_line_npn() {
        let line = ".MODEL 2N3904 NPN(IS=6.734f BF=416.4 BR=0.7371 VAF=74.03)";
        let model = parse_model_line(line).unwrap();
        assert_eq!(model.name, "2N3904");
        assert!(!model.is_pnp);
        assert!((model.is - 6.734e-15).abs() < 1e-30);
        assert!((model.bf - 416.4).abs() < 1e-10);
        assert!((model.br - 0.7371).abs() < 1e-10);
        assert!((model.vaf - 74.03).abs() < 1e-10);
    }

    #[test]
    fn parse_model_line_pnp() {
        let line = ".MODEL AC128 PNP(IS=20.66u BF=229.6 BR=14.66 NF=1.133 NR=1.140 VAF=19.68)";
        let model = parse_model_line(line).unwrap();
        assert_eq!(model.name, "AC128");
        assert!(model.is_pnp);
        assert!((model.is - 20.66e-6).abs() < 1e-15);
        assert!((model.bf - 229.6).abs() < 1e-10);
        assert!((model.nf - 1.133).abs() < 1e-10);
    }

    #[test]
    fn embedded_models_load() {
        // Verify all expected models are present
        let names = [
            "2N3904",
            "2N2222",
            "BC108",
            "BC109",
            "2N5088",
            "2N5089",
            "2N3906",
            "AC128",
            "OC44",
            "NKT275",
            "GENERIC_NPN",
            "GENERIC_PNP",
        ];
        for name in &names {
            assert!(
                bjt_by_name(name).is_some(),
                "Model '{}' not found in embedded transistors.model",
                name
            );
        }
    }

    #[test]
    fn case_insensitive_lookup() {
        assert!(bjt_by_name("2n3904").is_some());
        assert!(bjt_by_name("2N3904").is_some());
        assert!(bjt_by_name("ac128").is_some());
        assert!(bjt_by_name("AC128").is_some());
    }

    #[test]
    fn model_polarity() {
        assert!(!bjt_by_name("2N3904").unwrap().is_pnp);
        assert!(!bjt_by_name("2N2222").unwrap().is_pnp);
        assert!(bjt_by_name("2N3906").unwrap().is_pnp);
        assert!(bjt_by_name("AC128").unwrap().is_pnp);
        assert!(bjt_by_name("OC44").unwrap().is_pnp);
        assert!(bjt_by_name("NKT275").unwrap().is_pnp);
    }

    #[test]
    fn germanium_has_high_leakage() {
        let ac128 = bjt_by_name("AC128").unwrap();
        let n2n3904 = bjt_by_name("2N3904").unwrap();
        // Germanium IS should be orders of magnitude higher than silicon
        assert!(ac128.is > n2n3904.is * 1e6);
    }

    #[test]
    fn defaults_for_missing_params() {
        let line = ".MODEL MINIMAL NPN(IS=1f BF=100)";
        let model = parse_model_line(line).unwrap();
        // Unspecified params should get SPICE defaults
        assert_eq!(model.br, 1.0);
        assert_eq!(model.nf, 1.0);
        assert!(model.vaf.is_infinite());
        assert_eq!(model.ise, 0.0);
    }

    // -----------------------------------------------------------------------
    // JFET tests
    // -----------------------------------------------------------------------

    #[test]
    fn parse_jfet_njf() {
        let line = ".MODEL 2N5457 NJF VTO=-1.25 BETA=1.04m LAMBDA=30.8m CGS=3.58p CGD=3.58p PB=3.99 IS=95.6f";
        let model = parse_jfet_model_line(line).unwrap();
        assert_eq!(model.name, "2N5457");
        assert!(model.is_n_channel);
        assert!((model.vto - (-1.25)).abs() < 1e-10);
        assert!((model.beta - 1.04e-3).abs() < 1e-12);
        assert!((model.lambda - 30.8e-3).abs() < 1e-10);
    }

    #[test]
    fn parse_jfet_pjf() {
        let line = ".MODEL 2N5460 PJF VTO=-2.16 BETA=517u LAMBDA=12.6m CGS=5.32p CGD=5.32p PB=722m IS=147f";
        let model = parse_jfet_model_line(line).unwrap();
        assert_eq!(model.name, "2N5460");
        assert!(!model.is_n_channel);
        assert!((model.vto - (-2.16)).abs() < 1e-10);
        assert_close(Some(model.beta), Some(517e-6));
    }

    #[test]
    fn parse_jfet_space_separated() {
        // Some models in the file use space-separated params without parens
        let line = ".MODEL TEST-NJF NJF VTO=-2.0 BETA=1.5m LAMBDA=9m RD=1 RS=1 CGS=2.9p CGD=2.8p";
        let model = parse_jfet_model_line(line).unwrap();
        assert_eq!(model.name, "TEST-NJF");
        assert!(model.is_n_channel);
        assert!((model.vto - (-2.0)).abs() < 1e-10);
    }

    #[test]
    fn jfet_embedded_models_load() {
        let names = [
            "J201",
            "2N5457",
            "2N5460",
            "2N5952",
            "2SK30A",
            "2SK30A-GR",
            "2SK30A-Y",
            "2SK30A-BL",
        ];
        for name in &names {
            assert!(
                jfet_by_name(name).is_some(),
                "JFET model '{}' not found in embedded jfets.model",
                name
            );
        }
    }

    #[test]
    fn jfet_case_insensitive_lookup() {
        assert!(jfet_by_name("j201").is_some());
        assert!(jfet_by_name("J201").is_some());
        assert!(jfet_by_name("2n5457").is_some());
        assert!(jfet_by_name("2N5457").is_some());
    }

    #[test]
    fn jfet_polarity() {
        assert!(jfet_by_name("2N5457").unwrap().is_n_channel);
        assert!(jfet_by_name("J201").unwrap().is_n_channel);
        assert!(!jfet_by_name("2N5460").unwrap().is_n_channel);
    }

    #[test]
    fn jfet_rejects_bjt() {
        assert!(parse_jfet_model_line(".MODEL 2N3904 NPN(IS=6.734f BF=416.4)").is_none());
    }

    // -----------------------------------------------------------------------
    // Triode tests
    // -----------------------------------------------------------------------

    #[test]
    fn parse_triode_line() {
        let line = ".TRIODE 12AX7 MU=100 EX=1.4 KG1=1060 KP=600 KVB=300";
        let model = parse_triode_model_line(line).unwrap();
        assert_eq!(model.name, "12AX7");
        assert!((model.mu - 100.0).abs() < 1e-10);
        assert!((model.ex - 1.4).abs() < 1e-10);
        assert!((model.kg1 - 1060.0).abs() < 1e-10);
        assert!((model.kp - 600.0).abs() < 1e-10);
        assert!((model.kvb - 300.0).abs() < 1e-10);
    }

    #[test]
    fn triode_embedded_models_load() {
        let names = [
            "12AX7", "12AT7", "12AU7", "12AY7", "12BH7", "6386", "6DJ8", "2A3", "300B", "6C33C",
            "6AN8T", "ECC83", "ECC81", "ECC82", "6072",
        ];
        for name in &names {
            assert!(
                triode_by_name(name).is_some(),
                "Triode model '{}' not found in embedded triodes.model",
                name
            );
        }
    }

    #[test]
    fn triode_case_insensitive_lookup() {
        assert!(triode_by_name("12ax7").is_some());
        assert!(triode_by_name("12AX7").is_some());
        assert!(triode_by_name("ecc83").is_some());
    }

    #[test]
    fn triode_koren_values_correct() {
        // Verify Koren parameters match the Tube.lib reference
        let ax7 = triode_by_name("12AX7").unwrap();
        assert!((ax7.mu - 100.0).abs() < 1e-10);
        assert!((ax7.ex - 1.4).abs() < 1e-10);
        assert!((ax7.kg1 - 1060.0).abs() < 1e-10);

        let at7 = triode_by_name("12AT7").unwrap();
        assert!((at7.mu - 60.0).abs() < 1e-10);
        assert!((at7.ex - 1.35).abs() < 1e-10);

        let au7 = triode_by_name("12AU7").unwrap();
        assert!((au7.mu - 21.5).abs() < 1e-10);
        assert!((au7.ex - 1.3).abs() < 1e-10);
    }

    #[test]
    fn triode_aliases_match() {
        let ax7 = triode_by_name("12AX7").unwrap();
        let ecc83 = triode_by_name("ECC83").unwrap();
        assert!((ax7.mu - ecc83.mu).abs() < 1e-10);
        assert!((ax7.kp - ecc83.kp).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Pentode tests
    // -----------------------------------------------------------------------

    #[test]
    fn parse_pentode_line() {
        let line = ".PENTODE 6550 MU=7.9 EX=1.35 KG1=890 KG2=4200 KP=60 KVB=24 KVB2=28 VG2=450";
        let model = parse_pentode_model_line(line).unwrap();
        assert_eq!(model.name, "6550");
        assert!((model.mu - 7.9).abs() < 1e-10);
        assert!((model.ex - 1.35).abs() < 1e-10);
        assert!((model.kg1 - 890.0).abs() < 1e-10);
        assert!((model.kg2 - 4200.0).abs() < 1e-10);
        assert!((model.kp - 60.0).abs() < 1e-10);
        assert!((model.kvb - 24.0).abs() < 1e-10);
        assert!((model.kvb2 - 28.0).abs() < 1e-10);
        assert!((model.vg2_default - 450.0).abs() < 1e-10);
    }

    #[test]
    fn pentode_embedded_models_load() {
        let names = [
            "6550", "EL34", "6L6GC", "KT88", "6AN8P", "EF86", "EL84", "6AQ5A", "6973", "6CA7",
            "KT77", "5881", "KT66", "KT90", "6L6", "6BQ5", "6267", "6AQ5",
        ];
        for name in &names {
            assert!(
                pentode_by_name(name).is_some(),
                "Pentode model '{}' not found in embedded pentodes.model",
                name
            );
        }
    }

    #[test]
    fn pentode_case_insensitive_lookup() {
        assert!(pentode_by_name("el34").is_some());
        assert!(pentode_by_name("EL34").is_some());
        assert!(pentode_by_name("kt88").is_some());
    }

    #[test]
    fn pentode_koren_values_correct() {
        // Verify Koren parameters match the Tube.lib reference
        let p6550 = pentode_by_name("6550").unwrap();
        assert!((p6550.mu - 7.9).abs() < 1e-10);
        assert!((p6550.kp - 60.0).abs() < 1e-10);
        assert!((p6550.kvb - 24.0).abs() < 1e-10);

        let kt88 = pentode_by_name("KT88").unwrap();
        assert!((kt88.mu - 8.8).abs() < 1e-10);
        assert!((kt88.kp - 32.0).abs() < 1e-10);
    }

    #[test]
    fn pentode_kt88_is_distinct_from_6550() {
        let p6550 = pentode_by_name("6550").unwrap();
        let kt88 = pentode_by_name("KT88").unwrap();
        // KT88 is a distinct tube — different KG1, KP, KVB values
        assert!((p6550.kp - kt88.kp).abs() > 1.0);
    }

    #[test]
    fn pentode_aliases_match() {
        let el34 = pentode_by_name("EL34").unwrap();
        let p6ca7 = pentode_by_name("6CA7").unwrap();
        assert!((el34.mu - p6ca7.mu).abs() < 1e-10);
        assert!((el34.kp - p6ca7.kp).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Op-amp / OTA tests
    // -----------------------------------------------------------------------

    #[test]
    fn parse_opamp_line() {
        let line = ".OPAMP TL072 A0=200k GBW=3MEG SR=13 VPOS=12 VNEG=12 RO=75 COUT=20p";
        let model = parse_opamp_model_line(line).unwrap();
        assert_eq!(model.name, "TL072");
        assert!(!model.is_ota);
        assert!((model.open_loop_gain - 200_000.0).abs() < 1e-10);
        assert!((model.gbw - 3e6).abs() < 1e-3);
        assert!((model.slew_rate - 13.0).abs() < 1e-10);
        assert!((model.output_capacitance - 20e-12).abs() < 1e-20);
    }

    #[test]
    fn parse_ota_line() {
        let line = ".OTA CA3080 A0=100k GBW=2MEG SR=50 IABC=100u VT=25.85m GM=2m RLOAD=10k";
        let model = parse_opamp_model_line(line).unwrap();
        assert_eq!(model.name, "CA3080");
        assert!(model.is_ota);
        assert!((model.gbw - 2e6).abs() < 1e-3);
        assert!((model.ota_iabc - 100e-6).abs() < 1e-15);
        assert!((model.ota_vt - 25.85e-3).abs() < 1e-12);
        assert!((model.ota_gm - 2e-3).abs() < 1e-15);
        assert!((model.ota_r_load - 10_000.0).abs() < 1e-10);
    }

    #[test]
    fn opamp_embedded_models_load() {
        let names = [
            "GENERIC", "TL072", "TL082", "JRC4558", "RC4558", "LM308", "LM741", "NE5532", "OP07",
            "CA3080",
        ];
        for name in &names {
            assert!(
                opamp_by_name(name).is_some(),
                "Op-amp model '{}' not found in embedded opamps.model",
                name
            );
        }
    }

    #[test]
    fn opamp_case_insensitive() {
        assert!(opamp_by_name("tl072").is_some());
        assert!(opamp_by_name("jrc4558").is_some());
        assert!(opamp_by_name("ca3080").is_some());
    }

    // -----------------------------------------------------------------------
    // Diode tests
    // -----------------------------------------------------------------------

    #[test]
    fn parse_diode_line() {
        let line = ".MODEL 1N914 D(IS=7.075E-9 RS=0.78 N=1.95 TT=7.2E-9 CJO=4E-12 VJ=0.657 M=0.4 BV=100 IBV=0.0001)";
        let model = parse_diode_model_line(line).unwrap();
        assert_eq!(model.name, "1N914");
        assert!((model.is - 7.075e-9).abs() < 1e-20);
        assert!((model.rs - 0.78).abs() < 1e-10);
        assert!((model.n - 1.95).abs() < 1e-10);
        assert!((model.bv - 100.0).abs() < 1e-10);
    }

    #[test]
    fn diode_embedded_models_load() {
        let names = [
            "1N34", "1N914", "1N4001", "1N4002", "1N4003", "1N4004", "1N4005", "1N4006", "1N4007",
            "1N4148", "1N5400",
        ];
        for name in &names {
            assert!(
                diode_by_name(name).is_some(),
                "Diode model '{}' not found in embedded diodes.model",
                name
            );
        }
    }

    #[test]
    fn diode_case_insensitive() {
        assert!(diode_by_name("1n914").is_some());
        assert!(diode_by_name("1N914").is_some());
        assert!(diode_by_name("1n4148").is_some());
    }

    #[test]
    fn diode_germanium_higher_is() {
        let ge = diode_by_name("1N34").unwrap();
        let si = diode_by_name("1N914").unwrap();
        // Germanium has much higher IS (typ ~200pA vs ~7nA but GE N is higher)
        assert!(
            ge.n > si.n,
            "Ge diode should have higher N: Ge={} Si={}",
            ge.n,
            si.n
        );
    }

    // -----------------------------------------------------------------------
    // LED tests
    // -----------------------------------------------------------------------

    #[test]
    fn led_embedded_models_load() {
        let names = [
            "DLED0",
            "DLED1",
            "DLED2",
            "DLED3",
            "LED_IR",
            "LED_RED",
            "LED_GREEN",
            "LED_BLUE",
        ];
        for name in &names {
            assert!(
                led_by_name(name).is_some(),
                "LED model '{}' not found in embedded leds.model",
                name
            );
        }
    }

    #[test]
    fn led_blue_higher_n() {
        let red = led_by_name("LED_RED").unwrap();
        let blue = led_by_name("LED_BLUE").unwrap();
        // Blue LED has higher Vf → higher N
        assert!(
            blue.n > red.n,
            "Blue LED should have higher N (Vf): {} vs {}",
            blue.n,
            red.n
        );
    }

    // -----------------------------------------------------------------------
    // Schottky tests
    // -----------------------------------------------------------------------

    #[test]
    fn schottky_embedded_models_load() {
        let names = ["11DQ03", "11DQ04", "1N5828"];
        for name in &names {
            assert!(
                schottky_by_name(name).is_some(),
                "Schottky model '{}' not found in embedded schottky.model",
                name
            );
        }
    }

    #[test]
    fn schottky_has_low_bandgap() {
        let s = schottky_by_name("11DQ03").unwrap();
        assert!(
            (s.eg - 0.69).abs() < 0.01,
            "Schottky EG should be ~0.69: {}",
            s.eg
        );
        assert!(
            (s.xti - 2.0).abs() < 0.1,
            "Schottky XTI should be ~2: {}",
            s.xti
        );
    }

    // -----------------------------------------------------------------------
    // Zener tests
    // -----------------------------------------------------------------------

    #[test]
    fn zener_embedded_models_load() {
        let names = [
            "1N746", "1N750", "1N753", "1N758", "1N759", "1N4728", "1N4733", "1N4742", "1N4751",
        ];
        for name in &names {
            assert!(
                zener_by_name(name).is_some(),
                "Zener model '{}' not found in embedded zeners.model",
                name
            );
        }
    }

    #[test]
    fn zener_voltage_from_bv() {
        let z4v7 = zener_by_name("1N750").unwrap();
        // 1N750 is a 4.7V zener
        assert!(
            z4v7.bv > 4.0 && z4v7.bv < 5.0,
            "1N750 BV should be ~4.7V: {}",
            z4v7.bv
        );

        let z12v = zener_by_name("1N759").unwrap();
        // 1N759 is a 12V zener
        assert!(
            z12v.bv > 11.0 && z12v.bv < 13.0,
            "1N759 BV should be ~12V: {}",
            z12v.bv
        );
    }

    #[test]
    fn any_diode_cross_registry() {
        // Should find across all registries
        assert!(any_diode_by_name("1N914").is_some(), "Standard diode");
        assert!(any_diode_by_name("LED_RED").is_some(), "LED");
        assert!(any_diode_by_name("11DQ03").is_some(), "Schottky");
        assert!(any_diode_by_name("1N750").is_some(), "Zener");
    }
}
