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

impl PedalDef {
    /// Returns true if this pedal has an FX send/return loop defined.
    pub fn has_fx_loop(&self) -> bool {
        let has_send = self.nets.iter().any(|n| {
            n.from == Pin::Reserved("fx_send".into())
                || n.to.contains(&Pin::Reserved("fx_send".into()))
        });
        let has_return = self.nets.iter().any(|n| {
            n.from == Pin::Reserved("fx_return".into())
                || n.to.contains(&Pin::Reserved("fx_return".into()))
        });
        has_send && has_return
    }
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
    /// Zener diode with breakdown voltage (V). Common: 3.3, 4.7, 5.1, 6.2, 9.1, 12
    Zener(f64),
    Potentiometer(f64),
    Npn,
    Pnp,
    OpAmp(OpAmpType),
    NJfet(JfetType),
    PJfet(JfetType),
    Photocoupler(PhotocouplerType),
    /// LFO: waveform, timing_r (Ω), timing_c (F) -> f = 1/(2πRC)
    Lfo(LfoWaveformDsl, f64, f64),
    /// Triode vacuum tube
    Triode(TriodeType),
    /// Pentode vacuum tube
    Pentode(PentodeType),
    /// Envelope follower: attack_r (Ω), attack_c (F), release_r (Ω), release_c (F), sensitivity_r (Ω)
    /// Attack τ = attack_r × attack_c, Release τ = release_r × release_c
    /// Sensitivity = sensitivity_r / 10kΩ
    EnvelopeFollower(f64, f64, f64, f64, f64),
    /// N-channel enhancement-mode MOSFET
    Nmos(MosfetType),
    /// P-channel enhancement-mode MOSFET
    Pmos(MosfetType),
    /// BBD (Bucket-Brigade Device) delay line
    Bbd(BbdType),
    // ── Synth-specific component types ──────────────────────────────────
    /// Voltage-Controlled Oscillator IC (CEM3340/AS3340/V3340).
    /// Generates sawtooth, triangle, and pulse waveforms with 1V/Oct tracking.
    Vco(VcoType),
    /// Voltage-Controlled Filter IC (CEM3320/AS3320).
    /// 4-pole lowpass with voltage-controlled cutoff and resonance.
    Vcf(VcfType),
    /// Voltage-Controlled Amplifier IC (SSM2164/V2164).
    /// Exponential-control quad VCA.
    Vca(VcaType),
    /// Comparator IC (LM311, LM393). Open-collector output.
    /// Used in VCO reset circuits, Schmitt triggers, ADSR level detection.
    Comparator(ComparatorType),
    /// Quad bilateral analog switch (CD4066, DG411).
    /// Used for sample-and-hold, gate routing, ADSR switching.
    AnalogSwitch(AnalogSwitchType),
    /// Matched NPN transistor pair/array for exponential converters.
    MatchedNpn(MatchedTransistorType),
    /// Matched PNP transistor pair for exponential converters.
    MatchedPnp(MatchedTransistorType),
    /// Temperature-compensating resistor for exponential converters.
    /// Parameters: nominal resistance (Ω), tempco (ppm/°C).
    Tempco(f64, f64),
}

/// Op-amp types with different characteristics.
/// Each type has distinct slew rate, gain-bandwidth product, and input impedance
/// that affect the tone and response of the circuit.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OpAmpType {
    /// Generic op-amp (defaults to TL072 characteristics)
    #[default]
    Generic,
    /// TL072 - JFET input, fast, clean (slew: 13V/µs, GBW: 3MHz)
    /// Common in: Tube Screamer mods, Klon, modern pedals
    Tl072,
    /// JRC4558D - BJT input, warmer, compressed (slew: 1.7V/µs, GBW: 3MHz)
    /// The classic Tube Screamer op-amp
    Jrc4558,
    /// LM308 - Slow slew rate gives distinctive compression (slew: 0.3V/µs, GBW: 1MHz)
    /// The heart of the Pro-Co RAT
    Lm308,
    /// LM741 - Classic slow op-amp (slew: 0.5V/µs, GBW: 1MHz)
    /// Vintage circuits
    Lm741,
    /// NE5532 - Low noise, high drive (slew: 9V/µs, GBW: 10MHz)
    /// Studio-grade circuits
    Ne5532,
    /// CA3080 - Operational Transconductance Amplifier (OTA)
    /// Current-controlled gain, used in compressors like Dyna Comp
    Ca3080,
    /// RC4558 - Texas Instruments version of 4558 (slew: 1.7V/µs)
    Rc4558,
    /// TL082 - Similar to TL072 but different bias (slew: 13V/µs)
    Tl082,
    /// OP07 - Precision low-offset op-amp (slew: 0.3V/µs, GBW: 0.6MHz)
    Op07,
}

impl OpAmpType {
    /// Slew rate in V/µs - affects high frequency response and distortion character
    pub fn slew_rate(&self) -> f64 {
        match self {
            OpAmpType::Generic | OpAmpType::Tl072 | OpAmpType::Tl082 => 13.0,
            OpAmpType::Jrc4558 | OpAmpType::Rc4558 => 1.7,
            OpAmpType::Lm308 => 0.3,
            OpAmpType::Lm741 => 0.5,
            OpAmpType::Ne5532 => 9.0,
            OpAmpType::Ca3080 => 50.0, // OTA - very fast
            OpAmpType::Op07 => 0.3,
        }
    }

    /// Gain-bandwidth product in Hz - affects frequency response
    pub fn gain_bandwidth(&self) -> f64 {
        match self {
            OpAmpType::Generic | OpAmpType::Tl072 | OpAmpType::Tl082 => 3e6,
            OpAmpType::Jrc4558 | OpAmpType::Rc4558 => 3e6,
            OpAmpType::Lm308 => 1e6,
            OpAmpType::Lm741 => 1e6,
            OpAmpType::Ne5532 => 10e6,
            OpAmpType::Ca3080 => 2e6, // Transconductance-dependent
            OpAmpType::Op07 => 0.6e6,
        }
    }

    /// Maximum supply voltage (total V+ to V-)
    pub fn supply_max(&self) -> f64 {
        match self {
            OpAmpType::Generic | OpAmpType::Tl072 | OpAmpType::Tl082 => 36.0,
            OpAmpType::Jrc4558 | OpAmpType::Rc4558 => 36.0,
            OpAmpType::Lm308 => 36.0,
            OpAmpType::Lm741 => 36.0,
            OpAmpType::Ne5532 => 44.0,
            OpAmpType::Ca3080 => 36.0,
            OpAmpType::Op07 => 44.0,
        }
    }

    /// Whether this is an OTA (Operational Transconductance Amplifier)
    /// OTAs have current output and behave differently
    pub fn is_ota(&self) -> bool {
        matches!(self, OpAmpType::Ca3080)
    }
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
    /// 2N5952 - US phaser JFET (original MXR Phase 90)
    /// Idss: 1.0-5.0 mA (typical 2.4 mA), Vgs(off): -0.5V to -4.0V
    N2n5952,
    /// 2SK30A - Classic phaser JFET (Toshiba), generic mid-grade
    N2sk30a,
    /// 2SK30A-GR - Low Idss grade (0.6-1.4 mA)
    N2sk30aGr,
    /// 2SK30A-Y - Medium Idss grade (1.2-3.0 mA)
    N2sk30aY,
    /// 2SK30A-BL - High Idss grade (2.6-6.5 mA)
    N2sk30aBl,
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
    /// 12AY7 / 6072 - Low-medium gain, original Fender tweed tube
    T12ay7,
}

/// Pentode vacuum tube types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PentodeType {
    /// EF86 / 6267 - Small-signal pentode, Vox AC15/AC30 preamp
    Ef86,
    /// EL84 / 6BQ5 - Power pentode, Vox AC15/AC30 output stage
    El84,
}

/// MOSFET types for enhancement-mode devices used in guitar pedals.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MosfetType {
    /// 2N7000 N-channel MOSFET - common in Fulltone OCD clipping, modern drives
    N2n7000,
    /// IRF520 N-channel power MOSFET - higher current capability
    Irf520,
    /// BS250 P-channel MOSFET - used in some boost circuits
    Bs250,
    /// IRF9520 P-channel power MOSFET
    Irf9520,
}

/// BBD (Bucket-Brigade Device) types for analog delay lines.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BbdType {
    /// MN3207 — 1024-stage, Boss CE-2 chorus
    Mn3207,
    /// MN3007 — 1024-stage low-noise, Boss DM-2 delay
    Mn3007,
    /// MN3005 — 4096-stage long delay, Memory Man
    Mn3005,
}

// ── Synth-specific component type enums ─────────────────────────────────

/// VCO (Voltage-Controlled Oscillator) IC types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VcoType {
    /// CEM3340 — Curtis original. Prophet 5, MemoryMoog, Jupiter 6.
    Cem3340,
    /// AS3340 — Alfa RPAR clone (currently in production, pin-compatible).
    As3340,
    /// V3340 — CoolAudio clone (currently in production, pin-compatible).
    V3340,
}

/// VCF (Voltage-Controlled Filter) IC types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VcfType {
    /// CEM3320 — Curtis 4-pole lowpass. Prophet 5, Oberheim OB-Xa.
    Cem3320,
    /// AS3320 — Alfa RPAR clone (currently in production).
    As3320,
}

/// VCA (Voltage-Controlled Amplifier) IC types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VcaType {
    /// SSM2164 — Quad exponential VCA. Widely used in Eurorack.
    Ssm2164,
    /// V2164 — CoolAudio clone (currently in production, pin-compatible).
    V2164,
}

/// Comparator IC types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComparatorType {
    /// LM311 — Single comparator, open-collector output. VCO reset, Schmitt triggers.
    Lm311,
    /// LM393 — Dual comparator. Precision applications, window comparators.
    Lm393,
}

/// Analog switch IC types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AnalogSwitchType {
    /// CD4066 — Quad bilateral switch (~100Ω on-resistance). S&H, muting, routing.
    Cd4066,
    /// DG411 — Quad SPST analog switch (~25Ω on-resistance). Higher precision.
    Dg411,
}

/// Matched transistor pair/array types for exponential converters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MatchedTransistorType {
    /// SSM2210 — Matched dual NPN. Tight Vbe matching for V/Oct tracking.
    Ssm2210,
    /// CA3046 — 5-NPN transistor array (common substrate). Expo converters.
    Ca3046,
    /// LM394 — Supermatch pair. Ultralow Vbe offset.
    Lm394,
    /// THAT340 — Modern matched quad NPN (THAT Corporation).
    That340,
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

/// Parse zener diode: `zener(5.1)` or `zener(5.1v)`
///
/// The voltage parameter specifies the breakdown voltage in volts.
/// Common values: 3.3, 4.7, 5.1, 5.6, 6.2, 9.1, 12
fn parse_zener(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("zener")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, voltage) = double(input)?;
    // Optional 'v' suffix for clarity
    let (input, _) = opt(tag("v"))(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Zener(voltage)))
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

fn opamp_type(input: &str) -> IResult<&str, OpAmpType> {
    alt((
        value(OpAmpType::Tl072, tag("tl072")),
        value(OpAmpType::Tl082, tag("tl082")),
        value(OpAmpType::Jrc4558, alt((tag("jrc4558"), tag("4558")))),
        value(OpAmpType::Rc4558, tag("rc4558")),
        value(OpAmpType::Lm308, tag("lm308")),
        value(OpAmpType::Lm741, tag("lm741")),
        value(OpAmpType::Ne5532, tag("ne5532")),
        value(OpAmpType::Ca3080, tag("ca3080")),
        value(OpAmpType::Op07, tag("op07")),
    ))(input)
}

fn parse_opamp(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("opamp")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, ot) = opt(opamp_type)(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::OpAmp(ot.unwrap_or(OpAmpType::Generic))))
}

fn jfet_type(input: &str) -> IResult<&str, JfetType> {
    alt((
        value(JfetType::J201, tag("j201")),
        value(JfetType::N2n5457, tag("2n5457")),
        value(JfetType::P2n5460, tag("2n5460")),
        value(JfetType::N2n5952, tag("2n5952")),
        // 2SK30A variants - order matters: specific grades before generic
        value(JfetType::N2sk30aGr, alt((tag("2sk30a_gr"), tag("2sk30-gr")))),
        value(JfetType::N2sk30aY, alt((tag("2sk30a_y"), tag("2sk30-y")))),
        value(JfetType::N2sk30aBl, alt((tag("2sk30a_bl"), tag("2sk30-bl")))),
        value(JfetType::N2sk30a, alt((tag("2sk30a"), tag("2sk30")))),
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
        value(TriodeType::T12ay7, tag("12ay7")),
        value(TriodeType::T12au7, tag("12au7")),
        // European/military designations
        value(TriodeType::T12ax7, tag("ecc83")),
        value(TriodeType::T12at7, tag("ecc81")),
        value(TriodeType::T12au7, tag("ecc82")),
        value(TriodeType::T12ay7, tag("6072")),
    ))(input)
}

fn parse_triode(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("triode")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, tt) = triode_type(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Triode(tt)))
}

fn pentode_type(input: &str) -> IResult<&str, PentodeType> {
    alt((
        value(PentodeType::Ef86, tag("ef86")),
        value(PentodeType::El84, tag("el84")),
        // Alternative designations
        value(PentodeType::Ef86, tag("6267")),
        value(PentodeType::El84, tag("6bq5")),
    ))(input)
}

fn parse_pentode(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("pentode")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, pt) = pentode_type(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Pentode(pt)))
}

fn mosfet_type(input: &str) -> IResult<&str, MosfetType> {
    alt((
        value(MosfetType::N2n7000, tag("2n7000")),
        value(MosfetType::Irf520, tag("irf520")),
        value(MosfetType::Bs250, tag("bs250")),
        value(MosfetType::Irf9520, tag("irf9520")),
    ))(input)
}

fn parse_nmos(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("nmos")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, mt) = mosfet_type(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Nmos(mt)))
}

fn parse_pmos(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("pmos")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, mt) = mosfet_type(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Pmos(mt)))
}

fn bbd_type(input: &str) -> IResult<&str, BbdType> {
    alt((
        value(BbdType::Mn3207, tag("mn3207")),
        value(BbdType::Mn3007, tag("mn3007")),
        value(BbdType::Mn3005, tag("mn3005")),
    ))(input)
}

/// `bbd(mn3207)` — Bucket-brigade device delay line.
fn parse_bbd(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("bbd")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, bt) = bbd_type(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Bbd(bt)))
}

// ── Synth component parsers ─────────────────────────────────────────────

fn vco_type(input: &str) -> IResult<&str, VcoType> {
    alt((
        value(VcoType::Cem3340, tag("cem3340")),
        value(VcoType::As3340, tag("as3340")),
        value(VcoType::V3340, tag("v3340")),
    ))(input)
}

fn parse_vco(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("vco")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, vt) = vco_type(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Vco(vt)))
}

fn vcf_type(input: &str) -> IResult<&str, VcfType> {
    alt((
        value(VcfType::Cem3320, tag("cem3320")),
        value(VcfType::As3320, tag("as3320")),
    ))(input)
}

fn parse_vcf(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("vcf")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, vt) = vcf_type(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Vcf(vt)))
}

fn vca_type(input: &str) -> IResult<&str, VcaType> {
    alt((
        value(VcaType::Ssm2164, tag("ssm2164")),
        value(VcaType::V2164, tag("v2164")),
    ))(input)
}

fn parse_vca(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("vca")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, vt) = vca_type(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Vca(vt)))
}

fn comparator_type(input: &str) -> IResult<&str, ComparatorType> {
    alt((
        value(ComparatorType::Lm311, tag("lm311")),
        value(ComparatorType::Lm393, tag("lm393")),
    ))(input)
}

fn parse_comparator(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("comparator")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, ct) = comparator_type(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Comparator(ct)))
}

fn analog_switch_type(input: &str) -> IResult<&str, AnalogSwitchType> {
    alt((
        value(AnalogSwitchType::Cd4066, tag("cd4066")),
        value(AnalogSwitchType::Dg411, tag("dg411")),
    ))(input)
}

fn parse_analog_switch(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("switch")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, st) = analog_switch_type(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::AnalogSwitch(st)))
}

fn matched_transistor_type(input: &str) -> IResult<&str, MatchedTransistorType> {
    alt((
        value(MatchedTransistorType::Ssm2210, tag("ssm2210")),
        value(MatchedTransistorType::Ca3046, tag("ca3046")),
        value(MatchedTransistorType::Lm394, tag("lm394")),
        value(MatchedTransistorType::That340, tag("that340")),
    ))(input)
}

fn parse_matched_npn(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("matched_npn")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, mt) = matched_transistor_type(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::MatchedNpn(mt)))
}

fn parse_matched_pnp(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("matched_pnp")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, mt) = matched_transistor_type(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::MatchedPnp(mt)))
}

/// `tempco(2k, 3500)` — tempco resistor: nominal_r, ppm/°C
fn parse_tempco(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("tempco")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, resistance) = eng_value(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(',')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, ppm) = double(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Tempco(resistance, ppm)))
}

fn component_kind(input: &str) -> IResult<&str, ComponentKind> {
    alt((
        alt((
            parse_resistor,
            parse_cap,
            parse_inductor,
            parse_diode_pair, // must come before parse_diode
            parse_diode,
            parse_zener, // zener diode with voltage parameter
            parse_pot,
            parse_npn,
            parse_pnp,
        )),
        alt((
            parse_opamp,
            parse_njfet,
            parse_pjfet,
            parse_photocoupler,
            parse_envelope_follower, // must come before parse_lfo (both are long keywords)
            parse_lfo,
            parse_triode,
            parse_pentode,
            parse_nmos,
            parse_pmos,
            parse_bbd,
        )),
        alt((
            parse_vco,
            parse_vcf,
            parse_vca,
            parse_comparator,
            parse_analog_switch,
            parse_matched_npn, // must come before parse_matched_pnp
            parse_matched_pnp,
            parse_tempco,
        )),
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

const RESERVED_NODES: &[&str] = &[
    "in", "out", "gnd", "vcc", "fx_send", "fx_return",
    // Synth-specific CV/Gate nodes
    "gate",       // Note on/off (0V / +5V)
    "cv_pitch",   // Pitch CV (1V/Oct standard)
    "cv_mod",     // Modulation CV (mod wheel, aftertouch)
    "cv_filter",  // Filter cutoff CV
];

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

/// Parse a complete `.pedal` or `.synth` file.
/// Both `pedal "Name" { ... }` and `synth "Name" { ... }` produce the same AST.
pub fn parse_pedal(input: &str) -> IResult<&str, PedalDef> {
    let (input, _) = ws_comments(input)?;
    // Accept both "pedal" and "synth" keywords — same AST, different semantics
    let (input, _) = alt((tag("pedal"), tag("synth")))(input)?;
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
    fn parse_njfet_2n5952() {
        let (_, c) = component_def("J1: njfet(2n5952)").unwrap();
        assert_eq!(c.kind, ComponentKind::NJfet(JfetType::N2n5952));
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
    fn parse_triode_12ay7() {
        let (_, c) = component_def("V1: triode(12ay7)").unwrap();
        assert_eq!(c.id, "V1");
        assert_eq!(c.kind, ComponentKind::Triode(TriodeType::T12ay7));
    }

    #[test]
    fn parse_triode_6072() {
        // 6072 = military designation for 12AY7
        let (_, c) = component_def("V1: triode(6072)").unwrap();
        assert_eq!(c.kind, ComponentKind::Triode(TriodeType::T12ay7));
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

    #[test]
    fn parse_opamp_generic() {
        let (_, c) = component_def("U1: opamp()").unwrap();
        assert_eq!(c.id, "U1");
        assert_eq!(c.kind, ComponentKind::OpAmp(OpAmpType::Generic));
    }

    #[test]
    fn parse_opamp_tl072() {
        let (_, c) = component_def("U1: opamp(tl072)").unwrap();
        assert_eq!(c.kind, ComponentKind::OpAmp(OpAmpType::Tl072));
    }

    #[test]
    fn parse_opamp_jrc4558() {
        let (_, c) = component_def("U1: opamp(jrc4558)").unwrap();
        assert_eq!(c.kind, ComponentKind::OpAmp(OpAmpType::Jrc4558));
    }

    #[test]
    fn parse_opamp_4558_alias() {
        // "4558" should also work as an alias for JRC4558
        let (_, c) = component_def("U1: opamp(4558)").unwrap();
        assert_eq!(c.kind, ComponentKind::OpAmp(OpAmpType::Jrc4558));
    }

    #[test]
    fn parse_opamp_lm308() {
        let (_, c) = component_def("U1: opamp(lm308)").unwrap();
        assert_eq!(c.kind, ComponentKind::OpAmp(OpAmpType::Lm308));
    }

    #[test]
    fn parse_opamp_ca3080() {
        let (_, c) = component_def("U1: opamp(ca3080)").unwrap();
        assert_eq!(c.kind, ComponentKind::OpAmp(OpAmpType::Ca3080));
        // CA3080 is an OTA
        assert!(OpAmpType::Ca3080.is_ota());
    }

    #[test]
    fn opamp_slew_rates() {
        // Verify slew rates are as specified
        assert!((OpAmpType::Lm308.slew_rate() - 0.3).abs() < 0.01);
        assert!((OpAmpType::Tl072.slew_rate() - 13.0).abs() < 0.1);
        assert!((OpAmpType::Jrc4558.slew_rate() - 1.7).abs() < 0.1);
    }

    #[test]
    fn opamp_gain_bandwidth() {
        // Verify GBW products
        assert!((OpAmpType::Lm308.gain_bandwidth() - 1e6).abs() < 1e3);
        assert!((OpAmpType::Ne5532.gain_bandwidth() - 10e6).abs() < 1e3);
    }

    // ── MOSFET parser tests ──────────────────────────────────────────────

    #[test]
    fn parse_nmos_2n7000() {
        let (_, c) = component_def("M1: nmos(2n7000)").unwrap();
        assert_eq!(c.id, "M1");
        assert_eq!(c.kind, ComponentKind::Nmos(MosfetType::N2n7000));
    }

    #[test]
    fn parse_nmos_irf520() {
        let (_, c) = component_def("M1: nmos(irf520)").unwrap();
        assert_eq!(c.kind, ComponentKind::Nmos(MosfetType::Irf520));
    }

    #[test]
    fn parse_pmos_bs250() {
        let (_, c) = component_def("M2: pmos(bs250)").unwrap();
        assert_eq!(c.id, "M2");
        assert_eq!(c.kind, ComponentKind::Pmos(MosfetType::Bs250));
    }

    #[test]
    fn parse_pmos_irf9520() {
        let (_, c) = component_def("M2: pmos(irf9520)").unwrap();
        assert_eq!(c.kind, ComponentKind::Pmos(MosfetType::Irf9520));
    }

    // ── Zener diode parser tests ────────────────────────────────────────

    #[test]
    fn parse_zener_with_voltage_suffix() {
        let (_, c) = component_def("Z1: zener(5.1v)").unwrap();
        assert_eq!(c.id, "Z1");
        if let ComponentKind::Zener(vz) = c.kind {
            assert!((vz - 5.1).abs() < 1e-6);
        } else {
            panic!("expected Zener");
        }
    }

    #[test]
    fn parse_zener_without_suffix() {
        let (_, c) = component_def("Z2: zener(3.3)").unwrap();
        if let ComponentKind::Zener(vz) = c.kind {
            assert!((vz - 3.3).abs() < 1e-6);
        } else {
            panic!("expected Zener");
        }
    }

    #[test]
    fn parse_pedal_with_mosfet() {
        let src = r#"
pedal "MOSFET Drive" {
  components {
    C1: cap(100n)
    R1: resistor(10k)
    M1: nmos(2n7000)
    R2: resistor(1k)
  }
  nets {
    in -> C1.a
    C1.b -> R1.a, M1.drain
    R1.b -> gnd
    M1.source -> R2.a
    R2.b -> gnd
    M1.drain -> out
  }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert_eq!(def.name, "MOSFET Drive");
        assert_eq!(def.components.len(), 4);
        assert!(def.components.iter().any(|c| matches!(c.kind, ComponentKind::Nmos(MosfetType::N2n7000))));
    }

    #[test]
    fn parse_pedal_with_zener() {
        let src = r#"
pedal "Zener Clipper" {
  components {
    C1: cap(100n)
    R1: resistor(10k)
    Z1: zener(5.1v)
  }
  nets {
    in -> C1.a
    C1.b -> R1.a, Z1.a
    R1.b -> gnd
    Z1.b -> gnd
  }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert_eq!(def.name, "Zener Clipper");
        assert_eq!(def.components.len(), 3);
        assert!(def.components.iter().any(|c| matches!(c.kind, ComponentKind::Zener(v) if (v - 5.1).abs() < 1e-6)));
    }

    // ── BBD parser tests ──────────────────────────────────────────────────

    #[test]
    fn parse_bbd_mn3207() {
        let (_, c) = component_def("BBD1: bbd(mn3207)").unwrap();
        assert_eq!(c.id, "BBD1");
        assert_eq!(c.kind, ComponentKind::Bbd(BbdType::Mn3207));
    }

    #[test]
    fn parse_bbd_mn3007() {
        let (_, c) = component_def("BBD1: bbd(mn3007)").unwrap();
        assert_eq!(c.kind, ComponentKind::Bbd(BbdType::Mn3007));
    }

    #[test]
    fn parse_bbd_mn3005() {
        let (_, c) = component_def("BBD1: bbd(mn3005)").unwrap();
        assert_eq!(c.kind, ComponentKind::Bbd(BbdType::Mn3005));
    }

    #[test]
    fn parse_pedal_with_bbd() {
        let src = r#"
pedal "Chorus" {
  components {
    C1: cap(100n)
    R1: resistor(10k)
    BBD1: bbd(mn3207)
    LFO1: lfo(triangle, 100k, 47n)
  }
  nets {
    in -> C1.a
    C1.b -> R1.a, BBD1.in
    R1.b -> gnd
    BBD1.out -> out
    LFO1.out -> BBD1.clock
  }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert_eq!(def.name, "Chorus");
        assert_eq!(def.components.len(), 4);
        assert!(def
            .components
            .iter()
            .any(|c| matches!(c.kind, ComponentKind::Bbd(BbdType::Mn3207))));
    }

    // ── Pentode parser tests ────────────────────────────────────────────

    #[test]
    fn parse_pentode_ef86() {
        let (_, c) = component_def("V1: pentode(ef86)").unwrap();
        assert_eq!(c.id, "V1");
        assert_eq!(c.kind, ComponentKind::Pentode(PentodeType::Ef86));
    }

    #[test]
    fn parse_pentode_el84() {
        let (_, c) = component_def("V1: pentode(el84)").unwrap();
        assert_eq!(c.kind, ComponentKind::Pentode(PentodeType::El84));
    }

    #[test]
    fn parse_pentode_6267() {
        // 6267 = US designation for EF86
        let (_, c) = component_def("V1: pentode(6267)").unwrap();
        assert_eq!(c.kind, ComponentKind::Pentode(PentodeType::Ef86));
    }

    #[test]
    fn parse_pentode_6bq5() {
        // 6BQ5 = US designation for EL84
        let (_, c) = component_def("V1: pentode(6bq5)").unwrap();
        assert_eq!(c.kind, ComponentKind::Pentode(PentodeType::El84));
    }

    #[test]
    fn parse_pedal_with_pentode() {
        let src = r#"
pedal "Vox Preamp" {
  components {
    C1: cap(20n)
    R1: resistor(1M)
    V1: pentode(ef86)
    R2: resistor(220k)
    R3: resistor(2.2k)
    C2: cap(25u)
  }
  nets {
    in -> C1.a
    C1.b -> R1.a, V1.grid
    R1.b -> gnd
    vcc -> R2.a
    R2.b -> V1.plate
    V1.cathode -> R3.a, C2.a
    R3.b -> gnd
    C2.b -> gnd
    V1.plate -> out
  }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert_eq!(def.name, "Vox Preamp");
        assert!(def
            .components
            .iter()
            .any(|c| matches!(c.kind, ComponentKind::Pentode(PentodeType::Ef86))));
    }

    // ── Synth component parser tests ──────────────────────────────────

    #[test]
    fn parse_synth_keyword() {
        let src = r#"
synth "Test Synth" {
  components {
    R1: resistor(10k)
  }
  nets {
    in -> R1.a
    R1.b -> out
  }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert_eq!(def.name, "Test Synth");
    }

    #[test]
    fn parse_vco_cem3340() {
        let (_, c) = component_def("VCO1: vco(cem3340)").unwrap();
        assert_eq!(c.id, "VCO1");
        assert_eq!(c.kind, ComponentKind::Vco(VcoType::Cem3340));
    }

    #[test]
    fn parse_vco_as3340() {
        let (_, c) = component_def("VCO1: vco(as3340)").unwrap();
        assert_eq!(c.kind, ComponentKind::Vco(VcoType::As3340));
    }

    #[test]
    fn parse_vco_v3340() {
        let (_, c) = component_def("VCO1: vco(v3340)").unwrap();
        assert_eq!(c.kind, ComponentKind::Vco(VcoType::V3340));
    }

    #[test]
    fn parse_vcf_cem3320() {
        let (_, c) = component_def("VCF1: vcf(cem3320)").unwrap();
        assert_eq!(c.id, "VCF1");
        assert_eq!(c.kind, ComponentKind::Vcf(VcfType::Cem3320));
    }

    #[test]
    fn parse_vcf_as3320() {
        let (_, c) = component_def("VCF1: vcf(as3320)").unwrap();
        assert_eq!(c.kind, ComponentKind::Vcf(VcfType::As3320));
    }

    #[test]
    fn parse_vca_ssm2164() {
        let (_, c) = component_def("VCA1: vca(ssm2164)").unwrap();
        assert_eq!(c.id, "VCA1");
        assert_eq!(c.kind, ComponentKind::Vca(VcaType::Ssm2164));
    }

    #[test]
    fn parse_vca_v2164() {
        let (_, c) = component_def("VCA1: vca(v2164)").unwrap();
        assert_eq!(c.kind, ComponentKind::Vca(VcaType::V2164));
    }

    #[test]
    fn parse_comparator_lm311() {
        let (_, c) = component_def("U1: comparator(lm311)").unwrap();
        assert_eq!(c.id, "U1");
        assert_eq!(c.kind, ComponentKind::Comparator(ComparatorType::Lm311));
    }

    #[test]
    fn parse_comparator_lm393() {
        let (_, c) = component_def("U1: comparator(lm393)").unwrap();
        assert_eq!(c.kind, ComponentKind::Comparator(ComparatorType::Lm393));
    }

    #[test]
    fn parse_analog_switch_cd4066() {
        let (_, c) = component_def("SW1: switch(cd4066)").unwrap();
        assert_eq!(c.id, "SW1");
        assert_eq!(c.kind, ComponentKind::AnalogSwitch(AnalogSwitchType::Cd4066));
    }

    #[test]
    fn parse_analog_switch_dg411() {
        let (_, c) = component_def("SW1: switch(dg411)").unwrap();
        assert_eq!(c.kind, ComponentKind::AnalogSwitch(AnalogSwitchType::Dg411));
    }

    #[test]
    fn parse_matched_npn_ssm2210() {
        let (_, c) = component_def("QM1: matched_npn(ssm2210)").unwrap();
        assert_eq!(c.id, "QM1");
        assert_eq!(c.kind, ComponentKind::MatchedNpn(MatchedTransistorType::Ssm2210));
    }

    #[test]
    fn parse_matched_npn_ca3046() {
        let (_, c) = component_def("QM1: matched_npn(ca3046)").unwrap();
        assert_eq!(c.kind, ComponentKind::MatchedNpn(MatchedTransistorType::Ca3046));
    }

    #[test]
    fn parse_matched_pnp_lm394() {
        let (_, c) = component_def("QM1: matched_pnp(lm394)").unwrap();
        assert_eq!(c.kind, ComponentKind::MatchedPnp(MatchedTransistorType::Lm394));
    }

    #[test]
    fn parse_tempco() {
        let (_, c) = component_def("RT1: tempco(2k, 3500)").unwrap();
        assert_eq!(c.id, "RT1");
        if let ComponentKind::Tempco(r, ppm) = c.kind {
            assert!((r - 2000.0).abs() < 1e-6);
            assert!((ppm - 3500.0).abs() < 1e-6);
        } else {
            panic!("expected Tempco");
        }
    }

    #[test]
    fn parse_synth_with_cv_gate() {
        let src = r#"
synth "CV Test" {
  components {
    VCO1: vco(as3340)
    R1: resistor(100k)
    VCF1: vcf(as3320)
    R2: resistor(100k)
  }
  nets {
    cv_pitch -> R1.a
    R1.b -> VCO1.cv
    VCO1.saw -> VCF1.in
    cv_filter -> R2.a
    R2.b -> VCF1.cv
    VCF1.out -> out
    gate -> gnd
  }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert_eq!(def.name, "CV Test");
        assert_eq!(def.components.len(), 4);
        // Check that cv_pitch, cv_filter, and gate are valid reserved node names
        let has_reserved = |name: &str| {
            def.nets.iter().any(|n| {
                matches!(&n.from, Pin::Reserved(s) if s == name)
                    || n.to.iter().any(|p| matches!(p, Pin::Reserved(s) if s == name))
            })
        };
        assert!(has_reserved("cv_pitch"));
        assert!(has_reserved("cv_filter"));
        assert!(has_reserved("gate"));
    }
}
