//! SPICE `.MODEL` file parser and model registry.
//!
//! Parses standard SPICE `.MODEL` statements from embedded model files
//! and provides lookup by model name. Supports engineering suffixes
//! (f, p, n, u, m, k, MEG) for parameter values.
//!
//! Model files are included at compile time via `include_str!` and parsed
//! into a lazy-initialized registry on first access.

use std::collections::HashMap;
use std::sync::LazyLock;

// ---------------------------------------------------------------------------
// Raw model files embedded at compile time
// ---------------------------------------------------------------------------

static TRANSISTOR_MODELS_SRC: &str = include_str!("../models/transistors.model");
static JFET_MODELS_SRC: &str = include_str!("../models/jfets.model");
static TRIODE_MODELS_SRC: &str = include_str!("../models/triodes.model");
static PENTODE_MODELS_SRC: &str = include_str!("../models/pentodes.model");

// ---------------------------------------------------------------------------
// Parsed model registry (lazy-initialized)
// ---------------------------------------------------------------------------

/// All BJT models parsed from the embedded transistors.model file.
pub static BJT_MODELS: LazyLock<HashMap<String, SpiceBjtModel>> = LazyLock::new(|| {
    parse_bjt_models(TRANSISTOR_MODELS_SRC)
});

/// All JFET models parsed from the embedded jfets.model file.
pub static JFET_MODELS: LazyLock<HashMap<String, SpiceJfetModel>> = LazyLock::new(|| {
    parse_jfet_models(JFET_MODELS_SRC)
});

/// All triode models parsed from the embedded triodes.model file.
pub static TRIODE_MODELS: LazyLock<HashMap<String, SpiceTriodeModel>> = LazyLock::new(|| {
    parse_triode_models(TRIODE_MODELS_SRC)
});

/// All pentode models parsed from the embedded pentodes.model file.
pub static PENTODE_MODELS: LazyLock<HashMap<String, SpicePentodeModel>> = LazyLock::new(|| {
    parse_pentode_models(PENTODE_MODELS_SRC)
});

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
// Parsed SPICE Triode model
// ---------------------------------------------------------------------------

/// A parsed triode tube model with Koren equation parameters.
///
/// Format: `.TRIODE <name> MU= EX= KG1= KP= KVB=`
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
}

// ---------------------------------------------------------------------------
// Parsed SPICE Pentode model
// ---------------------------------------------------------------------------

/// A parsed pentode tube model with screen-referenced Koren equation parameters.
///
/// Format: `.PENTODE <name> MU= EX= KG1= KG2= KP= KVB= KVB2= VG2=`
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
        let params = if type_end < rest.len() { &rest[type_end..] } else { "" };
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

/// Parse a single `.TRIODE <name> MU= EX= KG1= KP= KVB=` line.
fn parse_triode_model_line(line: &str) -> Option<SpiceTriodeModel> {
    // Strip ".TRIODE" prefix
    let rest = line[7..].trim_start();
    let (name, params) = rest.split_once(|c: char| c.is_whitespace())?;

    let mut mu = 100.0;
    let mut ex = 1.4;
    let mut kg1 = 1060.0;
    let mut kp = 600.0;
    let mut kvb = 300.0;

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

/// Parse a single `.PENTODE <name> MU= EX= KG1= KG2= KP= KVB= KVB2= VG2=` line.
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
    })
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

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(a: Option<f64>, b: Option<f64>) {
        match (a, b) {
            (Some(a), Some(b)) => {
                let rel = if b.abs() > 1e-30 { ((a - b) / b).abs() } else { (a - b).abs() };
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
            "2N3904", "2N2222", "BC108", "BC109", "2N5088", "2N5089",
            "2N3906", "AC128", "OC44", "NKT275", "GENERIC_NPN", "GENERIC_PNP",
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
        let names = ["J201", "2N5457", "2N5460", "2N5952", "2SK30A", "2SK30A-GR", "2SK30A-Y", "2SK30A-BL"];
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
            "12AX7", "12AT7", "12AU7", "12AY7", "12BH7", "6386",
            "6DJ8", "2A3", "300B", "6C33C", "6AN8T",
            "ECC83", "ECC81", "ECC82", "6072",
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
            "6550", "EL34", "6L6GC", "KT88", "6AN8P",
            "EF86", "EL84", "6AQ5A", "6973",
            "6CA7", "KT77", "5881", "KT66", "KT90", "6L6",
            "6BQ5", "6267", "6AQ5",
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
}
