//! DSL parser for `.pedal` circuit definition files.
//!
//! Parses component declarations, net connections, and control mappings
//! into an AST that can drive WDF tree construction or KiCad export.

use nom::{
    branch::alt,
    bytes::complete::{tag, take_while, take_while1},
    character::complete::{char, multispace1, not_line_ending},
    combinator::{map, opt, recognize, value},
    multi::{many0, separated_list1},
    number::complete::double,
    sequence::{delimited, pair, preceded, tuple},
    IResult,
};

// ---------------------------------------------------------------------------
// AST
// ---------------------------------------------------------------------------

/// Rectifier type — determines supply output impedance character.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RectifierType {
    /// Tube rectifier (GZ34, 5U4, 5Y3) — high impedance, significant sag.
    /// Adds ~40–80V drop under load depending on tube type.
    Tube,
    /// Solid-state rectifier (silicon diodes) — very low impedance, stiff supply.
    /// Adds ~1–2V drop under load.
    SolidState,
}

impl Default for RectifierType {
    fn default() -> Self {
        RectifierType::SolidState
    }
}

/// Power supply configuration — models B+ rail behavior under load.
///
/// In real amps, the power supply has finite output impedance from the
/// rectifier forward drop, transformer winding resistance, and filter cap ESR.
/// When the output stage draws current, the B+ voltage sags proportionally.
///
/// Simple form: `supply 9V` — just sets nominal voltage, no sag.
/// Block form: `supply 480V { impedance: 150, filter_cap: 40u, rectifier: tube }`
#[derive(Debug, Clone, PartialEq)]
pub struct SupplyConfig {
    /// Nominal (no-load) supply voltage in volts.
    pub voltage: f64,
    /// Supply output impedance in ohms (rectifier + transformer + ESR).
    /// Higher = more sag. Tube rectifier: 50–200Ω, solid-state: 1–10Ω.
    pub impedance: Option<f64>,
    /// Main filter capacitance in farads — sets sag recovery time.
    /// Larger cap = faster recovery. Typical: 40µF (vintage) to 220µF (modern).
    pub filter_cap: Option<f64>,
    /// Rectifier type — tube (saggy) or solid-state (stiff).
    pub rectifier: RectifierType,
}

impl SupplyConfig {
    /// Create a simple supply config with just a voltage (no sag modeling).
    pub fn voltage_only(voltage: f64) -> Self {
        Self {
            voltage,
            impedance: None,
            filter_cap: None,
            rectifier: RectifierType::default(),
        }
    }

    /// Returns true if this supply has sag parameters configured.
    pub fn has_sag(&self) -> bool {
        self.impedance.is_some()
    }
}

/// Top-level pedal definition.
#[derive(Debug, Clone, PartialEq)]
pub struct PedalDef {
    pub name: String,
    /// Power supply configuration.
    /// If None, defaults to 9V with no sag in the compiler.
    pub supply: Option<SupplyConfig>,
    pub components: Vec<ComponentDef>,
    pub nets: Vec<NetDef>,
    pub controls: Vec<ControlDef>,
    /// Internal trim pots (not user-facing, factory adjustments)
    pub trims: Vec<ControlDef>,
    /// Monitor definitions for real-time metering visualization
    pub monitors: Vec<MonitorDef>,
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
    Npn(BjtType),
    Pnp(BjtType),
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
    /// Generic delay line backed by a ring buffer.
    /// Parameters: min_delay, max_delay, interpolation mode, medium model.
    /// Splits the WDF tree into write and read subtrees coupled by an
    /// implicit one-sample delay — the standard technique for feedback loops.
    DelayLine(f64, f64, crate::elements::Interpolation, crate::elements::Medium),
    /// Read-only tap into a named delay line at a ratio of the base delay.
    /// Parameters: parent delay line component ID, ratio (e.g., 2.0 = 2× base).
    Tap(String, f64),
    /// Neon bulb (NE-2, etc.) - used in relaxation oscillators and optocouplers.
    /// Exhibits negative resistance: conducts when striking voltage reached,
    /// extinguishes when voltage drops below maintaining voltage.
    Neon(NeonType),
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
    // ── Studio equipment component types ─────────────────────────────────
    /// Audio transformer with turns ratio and primary inductance.
    /// Used for impedance matching, isolation, and push-pull drive.
    /// Terminals: pri_a, pri_b, sec_a, sec_b, (optional: pri_ct, sec_ct)
    Transformer(TransformerConfig),
    /// Capacitor with switchable values (for rotary switch frequency selection).
    /// Values are selected by a linked RotarySwitch component.
    CapSwitched(Vec<f64>),
    /// Inductor with switchable values (for multi-tap inductors).
    /// Values are selected by a linked RotarySwitch component.
    InductorSwitched(Vec<f64>),
    /// Resistor with switchable values (for ratio selection networks).
    /// Values are selected by a linked RotarySwitch component.
    ResistorSwitched(Vec<f64>),
    /// Rotary switch that controls one or more switched components.
    /// Links to component IDs that have switchable values.
    RotarySwitch(Vec<String>),
    /// Simple mechanical switch (SPST, SPDT, or n-position rotary).
    /// Unlike AnalogSwitch (CD4066, etc.), this is a passive mechanical element.
    /// Parameter is number of positions (2 = SPDT, 3+ = rotary).
    Switch(usize),
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

/// BJT (Bipolar Junction Transistor) types with Ebers-Moll parameters.
/// Each type has distinct beta (hFE), saturation current, and Early voltage
/// that affect gain, clipping character, and frequency response.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BjtType {
    /// Generic BJT with typical small-signal values
    #[default]
    Generic,

    // ── NPN Silicon ──────────────────────────────────────────────────────
    /// 2N3904 — Workhorse NPN, ubiquitous in audio circuits
    N2n3904,
    /// 2N2222 — Classic NPN switching/amplifier
    N2n2222,
    /// BC108 — British/European small-signal NPN (Vox, Marshall)
    Bc108,
    /// BC109 — Low-noise variant of BC108 (Tone Bender)
    Bc109,
    /// 2N5088 — High-gain NPN for fuzz/distortion (Big Muff)
    N2n5088,
    /// 2N5089 — Ultra-high-gain NPN (Big Muff, boosters)
    N2n5089,

    // ── PNP Silicon ──────────────────────────────────────────────────────
    /// 2N3906 — Standard PNP complement to 2N3904
    N2n3906,

    // ── Germanium (Fuzz Face, vintage) ───────────────────────────────────
    /// AC128 — Classic germanium PNP for Fuzz Face
    /// Low beta (~70), high leakage, temperature-sensitive
    Ac128,
    /// OC44 — Vintage germanium PNP (early Tone Benders)
    /// Very low beta (~50), vintage character
    Oc44,
    /// NKT275 — "Holy grail" germanium PNP for Fuzz Face
    /// Medium-low beta (~85) with sweet spot character
    Nkt275,
}

impl BjtType {
    /// Whether this is a germanium transistor.
    /// Germanium transistors have lower voltage tolerance and higher leakage.
    pub fn is_germanium(&self) -> bool {
        matches!(self, BjtType::Ac128 | BjtType::Oc44 | BjtType::Nkt275)
    }

    /// Whether this is a PNP transistor type.
    pub fn is_pnp(&self) -> bool {
        matches!(
            self,
            BjtType::N2n3906 | BjtType::Ac128 | BjtType::Oc44 | BjtType::Nkt275
        )
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
    /// T4B - Electroluminescent panel + CdS LDR used in LA-2A
    /// EL panel driven by sidechain, LDR in audio path as variable attenuator
    T4b,
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
    /// 12BH7A - Medium-mu dual triode, high current capability
    /// Used in cathode follower stages like LA-2A totem-pole output
    T12bh7,
    /// 6386 - Remote-cutoff (variable-mu) dual triode
    /// THE tube that makes the Fairchild 670 possible.
    /// Unlike regular triodes, mu varies with grid bias (~50 at low bias
    /// to ~5 at high negative bias). This is the gain reduction mechanism.
    /// The smooth, musical compression character comes from this gradual
    /// gain change rather than hard limiting.
    T6386,
}

/// Pentode vacuum tube types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PentodeType {
    /// EF86 / 6267 - Small-signal pentode, Vox AC15/AC30 preamp
    Ef86,
    /// EL84 / 6BQ5 - Power pentode, Vox AC15/AC30 output stage
    El84,
    /// 6AQ5A - Beam power tube, used in Pultec MEQ-5 output stage
    A6aq5a,
    /// 6973 - Beam power tube, 9W output
    /// Used in Fairchild 670 sidechain push-pull output stage.
    /// Similar to EL84 but different pin-out and slightly higher power.
    A6973,
    /// 6L6GC - Beam power tetrode, the American power tube standard.
    /// Used in Fender Twin Reverb, Dumble ODS, and virtually every Fender >40W.
    /// Higher plate dissipation (30W), gradual compression when overdriven.
    /// Aliases: 6L6, 5881 (military), KT66 (British equivalent).
    A6l6gc,
    /// EL34 / 6CA7 - True pentode, the European power tube standard.
    /// Used in Marshall JCM800 and most Marshall amps.
    /// Dominant midrange, earlier breakup, sharper clipping knee than 6L6GC.
    /// Aliases: 6CA7 (American designation), KT77 (drop-in alternative).
    El34,
    /// 6550 - High-power beam tetrode.
    /// Used in Ampeg SVT, some Mesa Boogies, some Dumble builds.
    /// More headroom than 6L6GC, tighter bass.
    /// Aliases: KT88 (British), KT90 (higher dissipation variant).
    A6550,
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

/// Neon bulb types for relaxation oscillators and optocouplers.
/// Neon bulbs exhibit negative resistance behavior:
/// - Off until striking voltage is reached (~90V for NE-2)
/// - Conduct and emit light until voltage drops below maintaining voltage (~60V)
/// - Used in vintage tremolo/vibrato circuits (Fender, Wurlitzer)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum NeonType {
    /// NE-2 — Standard miniature neon indicator
    /// Striking: ~90V, Maintaining: ~60V, Current: 0.3-2mA
    #[default]
    Ne2,
    /// NE-51 — Higher current neon lamp
    /// Striking: ~95V, Maintaining: ~65V, Current: 0.5-3mA
    Ne51,
    /// NE-83 — Lower striking voltage variant
    /// Striking: ~65V, Maintaining: ~50V
    Ne83,
}

impl NeonType {
    /// Striking voltage - voltage at which the bulb ionizes and begins conducting
    pub fn striking_voltage(&self) -> f64 {
        match self {
            NeonType::Ne2 => 90.0,
            NeonType::Ne51 => 95.0,
            NeonType::Ne83 => 65.0,
        }
    }

    /// Maintaining voltage - voltage below which the bulb extinguishes
    pub fn maintaining_voltage(&self) -> f64 {
        match self {
            NeonType::Ne2 => 60.0,
            NeonType::Ne51 => 65.0,
            NeonType::Ne83 => 50.0,
        }
    }

    /// Typical operating current in Amps
    pub fn typical_current(&self) -> f64 {
        match self {
            NeonType::Ne2 => 0.5e-3,   // 0.5mA
            NeonType::Ne51 => 1.0e-3,  // 1mA
            NeonType::Ne83 => 0.3e-3,  // 0.3mA
        }
    }

    /// Whether this neon bulb emits enough light for optocoupler use
    pub fn suitable_for_opto(&self) -> bool {
        // All common neon types work with LDRs
        true
    }
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

// ═══════════════════════════════════════════════════════════════════════════
// Studio Equipment Types
// ═══════════════════════════════════════════════════════════════════════════

/// Transformer winding configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WindingType {
    /// Standard two-terminal winding
    #[default]
    Standard,
    /// Center-tapped winding (adds .ct terminal)
    CenterTap,
    /// Push-pull primary (two halves with center tap for B+)
    PushPull,
}

/// Audio transformer configuration.
///
/// Transformers are modeled with:
/// - Turns ratio (affects impedance transformation by n²)
/// - Primary inductance (determines low-frequency response)
/// - Optional center taps for push-pull configurations
/// - Parasitic elements (DCR, capacitance) for accurate HF response
#[derive(Debug, Clone, PartialEq)]
pub struct TransformerConfig {
    /// Turns ratio (primary:secondary). 1.0 = 1:1, 10.0 = 10:1 step-down
    pub turns_ratio: f64,
    /// Primary winding inductance in Henries
    pub primary_inductance: f64,
    /// Primary winding configuration
    pub primary_type: WindingType,
    /// Secondary winding configuration
    pub secondary_type: WindingType,
    /// DC resistance of primary winding (Ω), default 0
    pub primary_dcr: f64,
    /// DC resistance of secondary winding (Ω), default 0
    pub secondary_dcr: f64,
    /// Parasitic capacitance (F), default 0
    pub capacitance: f64,
    /// Coupling coefficient (0-1), default 0.99 for audio transformers
    pub coupling: f64,
}

impl Default for TransformerConfig {
    fn default() -> Self {
        Self {
            turns_ratio: 1.0,
            primary_inductance: 1.0,
            primary_type: WindingType::Standard,
            secondary_type: WindingType::Standard,
            primary_dcr: 0.0,
            secondary_dcr: 0.0,
            capacitance: 0.0,
            coupling: 0.99,
        }
    }
}

impl TransformerConfig {
    /// Create a simple transformer with turns ratio and inductance.
    pub fn new(turns_ratio: f64, primary_inductance: f64) -> Self {
        Self {
            turns_ratio,
            primary_inductance,
            ..Default::default()
        }
    }

    /// Create a center-tapped secondary transformer.
    pub fn with_center_tap(mut self) -> Self {
        self.secondary_type = WindingType::CenterTap;
        self
    }

    /// Create a push-pull primary transformer.
    pub fn with_push_pull_primary(mut self) -> Self {
        self.primary_type = WindingType::PushPull;
        self
    }

    /// Add DC resistance to windings.
    pub fn with_dcr(mut self, primary: f64, secondary: f64) -> Self {
        self.primary_dcr = primary;
        self.secondary_dcr = secondary;
        self
    }

    /// Add parasitic capacitance.
    pub fn with_capacitance(mut self, cap: f64) -> Self {
        self.capacitance = cap;
        self
    }
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

/// Monitor definition for real-time metering visualization.
///
/// Specifies which circuit nodes/components to tap for VU meters and visualizations.
///
/// # Syntax
/// ```text
/// monitors {
///   V1.plate_current -> "Tube 1" [vu]
///   output -> "Output Level" [ppm]
///   GR.reduction -> "Gain Reduction" [gr]
/// }
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct MonitorDef {
    /// Component to monitor (e.g., "V1", "input", "output", "GR")
    pub component: String,
    /// Property to monitor (e.g., "plate_current", "level", "reduction")
    pub property: String,
    /// Display label for the meter
    pub label: String,
    /// Meter type (VU, PPM, peak, gain reduction)
    pub meter_type: MeterType,
}

/// Meter type for monitor visualization.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MeterType {
    /// VU meter (300ms rise/fall, RMS-responding)
    #[default]
    Vu,
    /// Peak Programme Meter (10ms rise, 1.5s fall)
    Ppm,
    /// True peak with hold
    Peak,
    /// Gain reduction meter (for compressors)
    GainReduction,
    /// Tube glow visualization
    TubeGlow,
    /// Supply sag indicator
    SupplySag,
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
        value(1.0, tag("H")),  // Henries (unit marker, no scaling)
        value(1.0, tag("F")),  // Farads (unit marker, no scaling)
        value(1.0, tag("Ω")),  // Ohms (unit marker, no scaling)
        value(1.0, tag("R")),  // Ohms alternate notation
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
    let (input, _) = ws_comments(input)?;
    let (input, val) = eng_value(input)?;
    let (input, _) = ws_comments(input)?;

    // Optional wattage rating (ignored for WDF, but allows parsing)
    let (input, _wattage) = opt(tuple((
        char(','),
        ws_comments,
        eng_value, // e.g., 2W or 0.5W
        opt(char('W')),
    )))(input)?;

    let (input, _) = ws_comments(input)?;
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
    let (input, _) = ws_comments(input)?;
    let (input, val) = eng_value(input)?;
    let (input, _) = ws_comments(input)?;

    // Optional taper type: log, linear, audio (ignored for WDF, but allows parsing)
    let (input, _taper) = opt(tuple((
        char(','),
        ws_comments,
        alt((tag("log"), tag("linear"), tag("audio"))),
    )))(input)?;

    let (input, _) = ws_comments(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Potentiometer(val)))
}

fn bjt_type(input: &str) -> IResult<&str, BjtType> {
    alt((
        // NPN Silicon
        value(BjtType::N2n3904, tag("2n3904")),
        value(BjtType::N2n2222, tag("2n2222")),
        value(BjtType::Bc108, tag("bc108")),
        value(BjtType::Bc109, tag("bc109")),
        value(BjtType::N2n5088, tag("2n5088")),
        value(BjtType::N2n5089, tag("2n5089")),
        // PNP Silicon
        value(BjtType::N2n3906, tag("2n3906")),
        // Germanium
        value(BjtType::Ac128, tag("ac128")),
        value(BjtType::Oc44, tag("oc44")),
        value(BjtType::Nkt275, tag("nkt275")),
    ))(input)
}

fn parse_npn(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("npn")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, bt) = opt(bjt_type)(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Npn(bt.unwrap_or(BjtType::Generic))))
}

fn parse_pnp(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("pnp")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, bt) = opt(bjt_type)(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Pnp(bt.unwrap_or(BjtType::Generic))))
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
        value(PhotocouplerType::T4b, tag("t4b")),
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
        value(TriodeType::T12bh7, tag("12bh7")),
        // Remote-cutoff (variable-mu) triode for Fairchild 670
        value(TriodeType::T6386, tag("6386")),
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
        alt((
            value(PentodeType::Ef86, tag("ef86")),
            value(PentodeType::El84, tag("el84")),
            value(PentodeType::El34, tag("el34")),
            value(PentodeType::A6aq5a, tag("6aq5a")),
            value(PentodeType::A6aq5a, tag("6aq5")),
            value(PentodeType::A6973, tag("6973")),
            // 6L6GC and aliases — order matters: longer tags first
            value(PentodeType::A6l6gc, tag("6l6gc")),
            value(PentodeType::A6l6gc, tag("6l6")),
            value(PentodeType::A6l6gc, tag("5881")),
        )),
        alt((
            value(PentodeType::A6550, tag("6550")),
            // Alternative / equivalent designations
            value(PentodeType::Ef86, tag("6267")),
            value(PentodeType::El84, tag("6bq5")),
            value(PentodeType::El34, tag("6ca7")),
            value(PentodeType::El34, tag("kt77")),
            value(PentodeType::A6l6gc, tag("kt66")),
            value(PentodeType::A6550, tag("kt88")),
            value(PentodeType::A6550, tag("kt90")),
        )),
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

/// Parse interpolation mode keyword.
fn interpolation_mode(input: &str) -> IResult<&str, crate::elements::Interpolation> {
    alt((
        value(crate::elements::Interpolation::Linear, tag("linear")),
        value(crate::elements::Interpolation::Allpass, tag("allpass")),
        value(crate::elements::Interpolation::Cubic, tag("cubic")),
    ))(input)
}

/// Parse medium type keyword.
fn medium_type(input: &str) -> IResult<&str, crate::elements::Medium> {
    alt((
        value(crate::elements::Medium::None, tag("none")),
        value(crate::elements::Medium::TapeOxide, tag("tape_oxide")),
        value(crate::elements::Medium::BbdLeakage, tag("bbd_leakage")),
        value(crate::elements::Medium::DigitalQuantize, tag("digital_quantize")),
    ))(input)
}

/// `delay_line(1ms, 1200ms)` — Generic delay line.
///
/// Supports optional interpolation mode and medium type:
/// - `delay_line(1ms, 1200ms)` — defaults: allpass interpolation, no medium
/// - `delay_line(1ms, 1200ms, allpass)` — explicit interpolation
/// - `delay_line(1ms, 1200ms, medium: tape_oxide)` — with medium
/// - `delay_line(1ms, 1200ms, allpass, medium: tape_oxide)` — both
fn parse_delay_line(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("delay_line")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, min_delay) = eng_value(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(',')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, max_delay) = eng_value(input)?;
    let (input, _) = ws_comments(input)?;
    // Optional interpolation mode
    let (input, interp) = opt(preceded(
        pair(char(','), ws_comments),
        interpolation_mode,
    ))(input)?;
    let (input, _) = ws_comments(input)?;
    // Optional medium: keyword
    let (input, medium) = opt(preceded(
        pair(char(','), ws_comments),
        preceded(pair(tag("medium:"), ws_comments), medium_type),
    ))(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(')')(input)?;
    let interp = interp.unwrap_or(crate::elements::Interpolation::Allpass);
    let medium = medium.unwrap_or(crate::elements::Medium::None);
    Ok((input, ComponentKind::DelayLine(min_delay, max_delay, interp, medium)))
}

/// `tap(DL1, 2.0)` — Read-only tap into a named delay line.
///
/// Parameters: parent delay line component ID, ratio relative to base delay.
fn parse_tap(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("tap")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, parent_id) = take_while1(|c: char| c.is_alphanumeric() || c == '_')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(',')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, ratio) = double(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Tap(parent_id.to_string(), ratio)))
}

fn neon_type(input: &str) -> IResult<&str, NeonType> {
    alt((
        value(NeonType::Ne2, alt((tag("ne2"), tag("ne-2"), tag("NE2"), tag("NE-2")))),
        value(NeonType::Ne51, alt((tag("ne51"), tag("ne-51"), tag("NE51"), tag("NE-51")))),
        value(NeonType::Ne83, alt((tag("ne83"), tag("ne-83"), tag("NE83"), tag("NE-83")))),
    ))(input)
}

/// `neon()` or `neon(ne2)` — Neon bulb for relaxation oscillators.
/// Used in vintage tremolo circuits (Fender Vibrato, Wurlitzer) paired with LDRs.
fn parse_neon(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("neon")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;
    // Optional type - defaults to NE-2 if empty
    let (input, nt) = opt(neon_type)(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, ComponentKind::Neon(nt.unwrap_or_default())))
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

// ═══════════════════════════════════════════════════════════════════════════
// Studio Equipment Parsers
// ═══════════════════════════════════════════════════════════════════════════

/// Parse winding type modifier: `ct` for center-tap, `pp` for push-pull
/// Parse winding modifiers. Returns (winding_type, is_primary).
/// ct = center-tap secondary, ct_primary = center-tap primary, pp = push-pull
fn winding_modifier(input: &str) -> IResult<&str, WindingType> {
    alt((
        value(WindingType::CenterTap, tag("ct")),
        value(WindingType::PushPull, tag("pp")),
    ))(input)
}

/// Parse ct_primary specifically (center-tapped primary)
fn winding_modifier_primary(input: &str) -> IResult<&str, WindingType> {
    value(WindingType::CenterTap, tag("ct_primary"))(input)
}

/// `transformer(10:1, 2H)` — basic transformer
/// `transformer(1:4, 2H, 75, 200p)` — with positional DCR and parasitic cap
/// `transformer(1:1, 4H, ct)` — center-tapped secondary
/// `transformer(1:1, 4H, pp, ct)` — push-pull primary, center-tapped secondary
/// `transformer(10:1, 10H, 150, 300p, ct_primary)` — center-tapped primary
/// `transformer(10:1, 2H, dcr=75, Cp=200p)` — with named parasitics
///
/// Positional syntax: transformer(ratio, inductance [, dcr] [, cap] [, winding_mod])
/// Named syntax: transformer(ratio, inductance [, dcr=val] [, Cp=val] [, k=val])
fn parse_transformer(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("transformer")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;

    // Parse turns ratio: "10:1" or "1:1" or just a number
    let (input, ratio_num) = double(input)?;
    let (input, ratio) = if let Ok((input, _)) = char::<&str, nom::error::Error<&str>>(':')(input) {
        let (input, denom) = double(input)?;
        (input, ratio_num / denom)
    } else {
        (input, ratio_num)
    };

    let (input, _) = ws_comments(input)?;
    let (input, _) = char(',')(input)?;
    let (input, _) = ws_comments(input)?;

    // Parse primary inductance
    let (input, inductance) = eng_value(input)?;
    let (input, _) = ws_comments(input)?;

    // Parse optional modifiers and named parameters
    let mut config = TransformerConfig::new(ratio, inductance);
    let mut positional_count = 0; // Track positional numeric args after inductance

    // Try to parse additional comma-separated options
    let mut remaining = input;
    loop {
        // Check for comma
        if let Ok((input, _)) = tuple((ws_comments, char::<&str, nom::error::Error<&str>>(','), ws_comments))(remaining) {
            remaining = input;

            // Try ct_primary first (must come before ct to match longer tag)
            if let Ok((input, wt)) = winding_modifier_primary(remaining) {
                config.primary_type = wt;
                remaining = input;
                continue;
            }

            // Try other winding modifiers
            if let Ok((input, wt)) = winding_modifier(remaining) {
                // Determine if this is for primary or secondary based on context
                // pp = push-pull primary, ct = center-tap secondary
                if wt == WindingType::PushPull {
                    config.primary_type = wt;
                } else {
                    // ct defaults to secondary unless already set
                    if config.secondary_type == WindingType::Standard {
                        config.secondary_type = wt;
                    } else {
                        config.primary_type = wt;
                    }
                }
                remaining = input;
                continue;
            }

            // Try named parameters: dcr=value, Cp=value, k=value
            if let Ok((input, _)) = tag::<&str, &str, nom::error::Error<&str>>("dcr")(remaining) {
                let (input, _) = ws_comments(input)?;
                let (input, _) = char('=')(input)?;
                let (input, _) = ws_comments(input)?;
                let (input, dcr) = eng_value(input)?;
                config.primary_dcr = dcr;
                config.secondary_dcr = dcr; // Apply to both by default
                remaining = input;
                continue;
            }

            if let Ok((input, _)) = alt((
                tag::<&str, &str, nom::error::Error<&str>>("Cp"),
                tag("cp"),
            ))(remaining) {
                let (input, _) = ws_comments(input)?;
                let (input, _) = char('=')(input)?;
                let (input, _) = ws_comments(input)?;
                let (input, cap) = eng_value(input)?;
                config.capacitance = cap;
                remaining = input;
                continue;
            }

            if let Ok((input, _)) = tag::<&str, &str, nom::error::Error<&str>>("k")(remaining) {
                let (input, _) = ws_comments(input)?;
                let (input, _) = char('=')(input)?;
                let (input, _) = ws_comments(input)?;
                let (input, k) = double(input)?;
                config.coupling = k;
                remaining = input;
                continue;
            }

            // Try positional numeric values (DCR, then cap)
            if let Ok((input, val)) = eng_value(remaining) {
                match positional_count {
                    0 => {
                        // First positional = DCR
                        config.primary_dcr = val;
                        config.secondary_dcr = val;
                    }
                    1 => {
                        // Second positional = parasitic capacitance
                        config.capacitance = val;
                    }
                    _ => {
                        // Too many positional args, stop
                        break;
                    }
                }
                positional_count += 1;
                remaining = input;
                continue;
            }

            // Unknown parameter, break
            break;
        } else {
            break;
        }
    }

    let (input, _) = ws_comments(remaining)?;
    let (input, _) = char(')')(input)?;

    Ok((input, ComponentKind::Transformer(config)))
}

/// `cap_switched([1.5u, 220n, 68n, 27n])` — capacitor with switchable values
fn parse_cap_switched(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("cap_switched")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;

    // Optional opening bracket
    let (input, has_bracket) = opt(char('['))(input)?;
    let (input, _) = ws_comments(input)?;

    // Parse comma-separated values
    let (input, values) = separated_list1(
        tuple((ws_comments, char(','), ws_comments)),
        eng_value,
    )(input)?;

    let (input, _) = ws_comments(input)?;

    // Closing bracket if we had opening
    let input = if has_bracket.is_some() {
        let (input, _) = char(']')(input)?;
        let (input, _) = ws_comments(input)?;
        input
    } else {
        input
    };

    let (input, _) = char(')')(input)?;

    Ok((input, ComponentKind::CapSwitched(values)))
}

/// `inductor_switched(27m, 47m, 82m, 150m)` — inductor with switchable values (multi-tap)
/// `inductor_switched([27m, 47m, 82m])` — with optional brackets
fn parse_inductor_switched(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("inductor_switched")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;

    // Optional opening bracket
    let (input, has_bracket) = opt(char('['))(input)?;
    let (input, _) = ws_comments(input)?;

    // Parse comma-separated values
    let (input, values) = separated_list1(
        tuple((ws_comments, char(','), ws_comments)),
        eng_value,
    )(input)?;

    let (input, _) = ws_comments(input)?;

    // Closing bracket if we had opening
    let input = if has_bracket.is_some() {
        let (input, _) = char(']')(input)?;
        let (input, _) = ws_comments(input)?;
        input
    } else {
        input
    };

    let (input, _) = char(')')(input)?;

    Ok((input, ComponentKind::InductorSwitched(values)))
}

/// `resistor_switched([12k, 6.8k, 3.9k, 1.5k])` — resistor with switchable values
/// Used for ratio selection networks (1176), feedback networks, etc.
fn parse_resistor_switched(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("resistor_switched")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;

    // Optional opening bracket
    let (input, has_bracket) = opt(char('['))(input)?;
    let (input, _) = ws_comments(input)?;

    // Parse comma-separated values
    let (input, values) = separated_list1(
        tuple((ws_comments, char(','), ws_comments)),
        eng_value,
    )(input)?;

    let (input, _) = ws_comments(input)?;

    // Closing bracket if we had opening
    let input = if has_bracket.is_some() {
        let (input, _) = char(']')(input)?;
        let (input, _) = ws_comments(input)?;
        input
    } else {
        input
    };

    let (input, _) = char(')')(input)?;

    Ok((input, ComponentKind::ResistorSwitched(values)))
}

/// `switch(2)` — simple n-position mechanical switch
/// Unlike AnalogSwitch (CD4066), this is a passive mechanical element.
/// Used for mode selection, bypass, etc.
fn parse_switch(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("switch")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, positions) = double(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(')')(input)?;

    Ok((input, ComponentKind::Switch(positions as usize)))
}

/// `rotary("20Hz", "30Hz", "60Hz", "100Hz")` — rotary switch with position labels
/// `rotary(pos1, pos2, pos3)` — rotary switch with identifier position labels
fn parse_rotary_switch(input: &str) -> IResult<&str, ComponentKind> {
    let (input, _) = tag("rotary")(input)?;
    let (input, _) = char('(')(input)?;
    let (input, _) = ws_comments(input)?;

    // Parse comma-separated position labels (quoted strings or identifiers)
    let (input, labels) = separated_list1(
        tuple((ws_comments, char(','), ws_comments)),
        alt((
            map(quoted_string, String::from),
            map(identifier, String::from),
        )),
    )(input)?;

    let (input, _) = ws_comments(input)?;
    let (input, _) = char(')')(input)?;

    Ok((input, ComponentKind::RotarySwitch(labels)))
}

fn component_kind(input: &str) -> IResult<&str, ComponentKind> {
    alt((
        alt((
            parse_resistor_switched, // must come before parse_resistor
            parse_resistor,
            parse_cap_switched, // must come before parse_cap
            parse_cap,
            parse_inductor_switched, // must come before parse_inductor
            parse_inductor,
            parse_diode_pair, // must come before parse_diode
            parse_diode,
            parse_zener, // zener diode with voltage parameter
            parse_pot,
        )),
        alt((
            parse_npn,
            parse_pnp,
            parse_opamp,
            parse_njfet,
            parse_pjfet,
            parse_photocoupler,
            parse_envelope_follower, // must come before parse_lfo (both are long keywords)
            parse_lfo,
            parse_triode,
            parse_pentode,
        )),
        alt((
            parse_nmos,
            parse_pmos,
            parse_bbd,
            parse_delay_line,
            parse_tap,
            parse_neon,
            parse_vco,
            parse_vcf,
            parse_vca,
            parse_comparator,
        )),
        alt((
            parse_analog_switch,
            parse_matched_npn, // must come before parse_matched_pnp
            parse_matched_pnp,
            parse_tempco,
            parse_transformer,
            parse_rotary_switch,
            parse_switch, // simple n-position switch
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

    // Parse all dot-separated parts: T1.primary.a -> ["primary", "a"]
    let (input, rest_parts) = many0(preceded(char('.'), identifier))(input)?;

    if rest_parts.is_empty() {
        // No dots — reserved node or bare component
        if RESERVED_NODES.contains(&first) {
            Ok((input, Pin::Reserved(first.to_string())))
        } else {
            // Bare component name treated as reserved-style node
            Ok((input, Pin::Reserved(first.to_string())))
        }
    } else {
        // Has dots — component + pin (pin may include sub-parts)
        // T1.primary.a -> component="T1", pin="primary.a"
        let pin_name = rest_parts.join(".");
        Ok((
            input,
            Pin::ComponentPin {
                component: first.to_string(),
                pin: pin_name,
            },
        ))
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

/// Skip a line we can't parse (for forward compatibility)
fn skip_line(input: &str) -> IResult<&str, ()> {
    let (input, _) = ws_comments(input)?;
    // Skip until newline or closing brace
    let (input, _) = take_while(|c: char| c != '\n' && c != '}')(input)?;
    // Consume the newline if present
    let (input, _) = opt(char('\n'))(input)?;
    Ok((input, ()))
}

/// Try to parse a control_def, or skip the line if unparsable
fn control_or_skip(input: &str) -> IResult<&str, Option<ControlDef>> {
    let (input, _) = ws_comments(input)?;

    // Check if we're at closing brace
    if input.starts_with('}') {
        return Err(nom::Err::Error(nom::error::Error::new(input, nom::error::ErrorKind::Char)));
    }

    // Try to parse a control_def
    if let Ok((remaining, ctrl)) = control_def(input) {
        return Ok((remaining, Some(ctrl)));
    }

    // Couldn't parse, skip this line
    let (remaining, _) = skip_line(input)?;
    Ok((remaining, None))
}

fn controls_section(input: &str) -> IResult<&str, Vec<ControlDef>> {
    let (input, _) = ws_comments(input)?;
    let (input, _) = tag("controls")(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('{')(input)?;

    // Parse controls, skipping lines we can't understand
    let (input, maybe_ctrls) = many0(control_or_skip)(input)?;
    let ctrls: Vec<ControlDef> = maybe_ctrls.into_iter().flatten().collect();

    let (input, _) = ws_comments(input)?;
    let (input, _) = char('}')(input)?;
    Ok((input, ctrls))
}

/// Internal trim pots (factory adjustments, not user-facing).
/// Same syntax as controls section, parsed identically.
fn trims_section(input: &str) -> IResult<&str, Vec<ControlDef>> {
    let (input, _) = ws_comments(input)?;
    let (input, _) = tag("trims")(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('{')(input)?;

    // Parse trims, skipping lines we can't understand
    let (input, maybe_trims) = many0(control_or_skip)(input)?;
    let trims: Vec<ControlDef> = maybe_trims.into_iter().flatten().collect();

    let (input, _) = ws_comments(input)?;
    let (input, _) = char('}')(input)?;
    Ok((input, trims))
}

// ---------------------------------------------------------------------------
// Monitors Section
// ---------------------------------------------------------------------------

/// Parse meter type: vu, ppm, peak, gr, glow, sag
fn meter_type(input: &str) -> IResult<&str, MeterType> {
    alt((
        value(MeterType::Vu, alt((tag("vu"), tag("VU")))),
        value(MeterType::Ppm, alt((tag("ppm"), tag("PPM")))),
        value(MeterType::Peak, alt((tag("peak"), tag("PEAK")))),
        value(MeterType::GainReduction, alt((tag("gr"), tag("GR"), tag("gain_reduction")))),
        value(MeterType::TubeGlow, alt((tag("glow"), tag("tube_glow")))),
        value(MeterType::SupplySag, alt((tag("sag"), tag("supply_sag")))),
    ))(input)
}

/// Parse a single monitor definition:
/// `V1.plate_current -> "Tube 1" [vu]`
/// `output -> "Output Level" [ppm]`
fn monitor_def(input: &str) -> IResult<&str, MonitorDef> {
    let (input, _) = ws_comments(input)?;
    let (input, component) = identifier(input)?;
    let (input, _) = ws_comments(input)?;

    // Optional property (defaults to "level")
    let (input, property) = opt(preceded(char('.'), identifier))(input)?;
    let property = property.unwrap_or("level").to_string();

    let (input, _) = ws_comments(input)?;
    let (input, _) = tag("->")(input)?;
    let (input, _) = ws_comments(input)?;

    // Label in quotes
    let (input, label) = quoted_string(input)?;
    let (input, _) = ws_comments(input)?;

    // Meter type in brackets: [vu] or [ppm]
    let (input, _) = char('[')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, mtype) = meter_type(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(']')(input)?;

    Ok((
        input,
        MonitorDef {
            component: component.to_string(),
            property,
            label: label.to_string(),
            meter_type: mtype,
        },
    ))
}

/// Try to parse a monitor_def, or skip the line if we can't understand it.
fn monitor_or_skip(input: &str) -> IResult<&str, Option<MonitorDef>> {
    let (input, _) = ws_comments(input)?;

    // Check if we're at closing brace
    if input.starts_with('}') {
        return Err(nom::Err::Error(nom::error::Error::new(input, nom::error::ErrorKind::Char)));
    }

    // Try to parse a monitor_def
    if let Ok((remaining, mon)) = monitor_def(input) {
        return Ok((remaining, Some(mon)));
    }

    // Couldn't parse, skip this line
    let (remaining, _) = skip_line(input)?;
    Ok((remaining, None))
}

/// Parse the monitors section.
fn monitors_section(input: &str) -> IResult<&str, Vec<MonitorDef>> {
    let (input, _) = ws_comments(input)?;
    let (input, _) = tag("monitors")(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('{')(input)?;

    // Parse monitors, skipping lines we can't understand
    let (input, maybe_mons) = many0(monitor_or_skip)(input)?;
    let mons: Vec<MonitorDef> = maybe_mons.into_iter().flatten().collect();

    let (input, _) = ws_comments(input)?;
    let (input, _) = char('}')(input)?;
    Ok((input, mons))
}

// ---------------------------------------------------------------------------
// Top-level
// ---------------------------------------------------------------------------

/// Parse an optional supply voltage declaration.
///
/// Simple form: `supply 9V` or `supply 250V`
/// Block form:
/// ```text
/// supply 480V {
///     impedance: 150        # ohms
///     filter_cap: 40u       # farads
///     rectifier: tube       # or solid_state
/// }
/// ```
fn supply_section(input: &str) -> IResult<&str, SupplyConfig> {
    let (input, _) = ws_comments(input)?;
    let (input, _) = tag("supply")(input)?;
    let (input, _) = ws_comments(input)?;
    // Parse the voltage value - can be like "9V", "9.0V", "250V", etc.
    // Also support "9v" lowercase
    let (input, voltage) = recognize(pair(
        double,
        opt(alt((char('V'), char('v')))),
    ))(input)?;
    // Extract just the numeric part
    let num_str = voltage.trim_end_matches(|c| c == 'V' || c == 'v');
    let volts = num_str.parse::<f64>().unwrap_or(9.0);

    // Try to parse an optional block with sag parameters
    let (input, block) = opt(supply_block)(input)?;

    let config = if let Some((impedance, filter_cap, rectifier)) = block {
        SupplyConfig {
            voltage: volts,
            impedance,
            filter_cap,
            rectifier: rectifier.unwrap_or_default(),
        }
    } else {
        SupplyConfig::voltage_only(volts)
    };

    Ok((input, config))
}

/// Parse the body of a supply block: `{ impedance: 150, filter_cap: 40u, rectifier: tube }`
fn supply_block(
    input: &str,
) -> IResult<&str, (Option<f64>, Option<f64>, Option<RectifierType>)> {
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('{')(input)?;

    let mut impedance = None;
    let mut filter_cap = None;
    let mut rectifier = None;

    let mut input = input;
    loop {
        let (rest, _) = ws_comments(input)?;
        // Check for closing brace
        if let Ok((rest2, _)) = char::<&str, nom::error::Error<&str>>('}')(rest) {
            input = rest2;
            break;
        }
        // Try parsing each field
        if let Ok((rest2, val)) = supply_field_impedance(rest) {
            impedance = Some(val);
            input = rest2;
        } else if let Ok((rest2, val)) = supply_field_filter_cap(rest) {
            filter_cap = Some(val);
            input = rest2;
        } else if let Ok((rest2, val)) = supply_field_rectifier(rest) {
            rectifier = Some(val);
            input = rest2;
        } else {
            // Unknown field — skip to next line or closing brace
            let (rest2, _) = not_line_ending(rest)?;
            input = rest2;
        }
    }

    Ok((input, (impedance, filter_cap, rectifier)))
}

/// Parse `impedance: 150` (value in ohms).
fn supply_field_impedance(input: &str) -> IResult<&str, f64> {
    let (input, _) = tag("impedance")(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(':')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, val) = eng_value(input)?;
    // Skip optional inline comment
    let (input, _) = opt(preceded(ws_comments, preceded(char('#'), not_line_ending)))(input)?;
    Ok((input, val))
}

/// Parse `filter_cap: 40u` (value in farads).
fn supply_field_filter_cap(input: &str) -> IResult<&str, f64> {
    let (input, _) = tag("filter_cap")(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(':')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, val) = eng_value(input)?;
    // Skip optional inline comment
    let (input, _) = opt(preceded(ws_comments, preceded(char('#'), not_line_ending)))(input)?;
    Ok((input, val))
}

/// Parse `rectifier: tube` or `rectifier: solid_state`.
fn supply_field_rectifier(input: &str) -> IResult<&str, RectifierType> {
    let (input, _) = tag("rectifier")(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char(':')(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, rtype) = alt((
        value(RectifierType::Tube, tag("tube")),
        value(RectifierType::SolidState, tag("solid_state")),
    ))(input)?;
    // Skip optional inline comment
    let (input, _) = opt(preceded(ws_comments, preceded(char('#'), not_line_ending)))(input)?;
    Ok((input, rtype))
}

/// Parse a complete `.pedal` or `.synth` file.
/// Both `pedal "Name" { ... }` and `synth "Name" { ... }` produce the same AST.
pub fn parse_pedal(input: &str) -> IResult<&str, PedalDef> {
    let (input, _) = ws_comments(input)?;
    // Accept both "pedal" and "synth" keywords — same AST, different semantics
    let (input, _) = alt((tag("pedal"), tag("synth"), tag("equipment")))(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, name) = quoted_string(input)?;
    let (input, _) = ws_comments(input)?;
    let (input, _) = char('{')(input)?;

    // Optional supply voltage declaration (before components)
    let (input, supply) = opt(supply_section)(input)?;

    let (input, components) = components_section(input)?;
    let (input, nets) = nets_section(input)?;
    let (input, controls) = opt(controls_section)(input)?;
    let (input, trims) = opt(trims_section)(input)?;
    let (input, monitors) = opt(monitors_section)(input)?;

    let (input, _) = ws_comments(input)?;
    let (input, _) = char('}')(input)?;
    let (input, _) = ws_comments(input)?;

    Ok((
        input,
        PedalDef {
            name: name.to_string(),
            supply,
            components,
            nets,
            controls: controls.unwrap_or_default(),
            trims: trims.unwrap_or_default(),
            monitors: monitors.unwrap_or_default(),
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

    // ── Neon bulb parser tests ─────────────────────────────────────────

    #[test]
    fn parse_neon_default() {
        let (_, c) = component_def("NE1: neon()").unwrap();
        assert_eq!(c.id, "NE1");
        assert_eq!(c.kind, ComponentKind::Neon(NeonType::Ne2));
    }

    #[test]
    fn parse_neon_ne2() {
        let (_, c) = component_def("NE1: neon(ne2)").unwrap();
        assert_eq!(c.kind, ComponentKind::Neon(NeonType::Ne2));
    }

    #[test]
    fn parse_neon_ne2_hyphen() {
        let (_, c) = component_def("NE1: neon(ne-2)").unwrap();
        assert_eq!(c.kind, ComponentKind::Neon(NeonType::Ne2));
    }

    #[test]
    fn parse_neon_ne51() {
        let (_, c) = component_def("NE1: neon(ne51)").unwrap();
        assert_eq!(c.kind, ComponentKind::Neon(NeonType::Ne51));
    }

    #[test]
    fn parse_neon_ne83() {
        let (_, c) = component_def("NE1: neon(ne-83)").unwrap();
        assert_eq!(c.kind, ComponentKind::Neon(NeonType::Ne83));
    }

    #[test]
    fn parse_pedal_with_neon_tremolo() {
        let src = r#"
pedal "Fender Vibrato" {
  components {
    R1: resistor(1M)
    NE1: neon()
    C1: cap(0.1u)
    LDR1: photocoupler(vtl5c3)
    R2: resistor(10k)
  }
  nets {
    in -> R1.a
    R1.b -> LDR1.a
    LDR1.b -> out
    NE1.a -> C1.a
    C1.b -> gnd
    NE1.b -> R2.a
    R2.b -> gnd
  }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert_eq!(def.name, "Fender Vibrato");
        assert!(def
            .components
            .iter()
            .any(|c| matches!(c.kind, ComponentKind::Neon(NeonType::Ne2))));
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

    #[test]
    fn parse_pentode_6l6gc() {
        let (_, c) = component_def("V1: pentode(6l6gc)").unwrap();
        assert_eq!(c.kind, ComponentKind::Pentode(PentodeType::A6l6gc));
    }

    #[test]
    fn parse_pentode_6l6_alias() {
        // 6L6 = original lower-dissipation variant, treated as 6L6GC
        let (_, c) = component_def("V1: pentode(6l6)").unwrap();
        assert_eq!(c.kind, ComponentKind::Pentode(PentodeType::A6l6gc));
    }

    #[test]
    fn parse_pentode_5881_alias() {
        // 5881 = military equivalent of 6L6GC
        let (_, c) = component_def("V1: pentode(5881)").unwrap();
        assert_eq!(c.kind, ComponentKind::Pentode(PentodeType::A6l6gc));
    }

    #[test]
    fn parse_pentode_kt66_alias() {
        // KT66 = British equivalent of 6L6GC
        let (_, c) = component_def("V1: pentode(kt66)").unwrap();
        assert_eq!(c.kind, ComponentKind::Pentode(PentodeType::A6l6gc));
    }

    #[test]
    fn parse_pentode_el34() {
        let (_, c) = component_def("V1: pentode(el34)").unwrap();
        assert_eq!(c.kind, ComponentKind::Pentode(PentodeType::El34));
    }

    #[test]
    fn parse_pentode_6ca7_alias() {
        // 6CA7 = American designation for EL34
        let (_, c) = component_def("V1: pentode(6ca7)").unwrap();
        assert_eq!(c.kind, ComponentKind::Pentode(PentodeType::El34));
    }

    #[test]
    fn parse_pentode_kt77_alias() {
        // KT77 = drop-in alternative for EL34
        let (_, c) = component_def("V1: pentode(kt77)").unwrap();
        assert_eq!(c.kind, ComponentKind::Pentode(PentodeType::El34));
    }

    #[test]
    fn parse_pentode_6550() {
        let (_, c) = component_def("V1: pentode(6550)").unwrap();
        assert_eq!(c.kind, ComponentKind::Pentode(PentodeType::A6550));
    }

    #[test]
    fn parse_pentode_kt88_alias() {
        // KT88 = British equivalent of 6550
        let (_, c) = component_def("V1: pentode(kt88)").unwrap();
        assert_eq!(c.kind, ComponentKind::Pentode(PentodeType::A6550));
    }

    #[test]
    fn parse_pentode_kt90_alias() {
        // KT90 = higher dissipation variant of 6550
        let (_, c) = component_def("V1: pentode(kt90)").unwrap();
        assert_eq!(c.kind, ComponentKind::Pentode(PentodeType::A6550));
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

    // ── Studio Equipment parser tests ──────────────────────────────────

    #[test]
    fn parse_transformer_simple() {
        let (_, c) = component_def("T1: transformer(10:1, 2H)").unwrap();
        assert_eq!(c.id, "T1");
        if let ComponentKind::Transformer(cfg) = c.kind {
            assert!((cfg.turns_ratio - 10.0).abs() < 1e-6);
            assert!((cfg.primary_inductance - 2.0).abs() < 1e-6);
            assert_eq!(cfg.primary_type, WindingType::Standard);
            assert_eq!(cfg.secondary_type, WindingType::Standard);
        } else {
            panic!("expected Transformer");
        }
    }

    #[test]
    fn parse_transformer_with_winding_types() {
        // Parser syntax: pp = push-pull, ct = center-tap
        let (_, c) = component_def("T2: transformer(1.5:1, 4H, pp, ct)").unwrap();
        if let ComponentKind::Transformer(cfg) = c.kind {
            assert!((cfg.turns_ratio - 1.5).abs() < 1e-6);
            assert!((cfg.primary_inductance - 4.0).abs() < 1e-6);
            assert_eq!(cfg.primary_type, WindingType::PushPull);
            assert_eq!(cfg.secondary_type, WindingType::CenterTap);
        } else {
            panic!("expected Transformer");
        }
    }

    #[test]
    fn parse_transformer_with_dcr_and_coupling() {
        // Parser syntax: dcr=value (applies to both), k=coupling
        let (_, c) = component_def("T3: transformer(5:1, 1H, dcr=50, k=0.98)").unwrap();
        if let ComponentKind::Transformer(cfg) = c.kind {
            assert!((cfg.turns_ratio - 5.0).abs() < 1e-6);
            assert!((cfg.primary_dcr - 50.0).abs() < 1e-6);
            assert!((cfg.secondary_dcr - 50.0).abs() < 1e-6); // Both set to same value
            assert!((cfg.coupling - 0.98).abs() < 1e-6);
        } else {
            panic!("expected Transformer");
        }
    }

    #[test]
    fn parse_cap_switched() {
        let (_, c) = component_def("C_lf: cap_switched(27n, 68n, 220n, 1.5u)").unwrap();
        assert_eq!(c.id, "C_lf");
        if let ComponentKind::CapSwitched(values) = c.kind {
            assert_eq!(values.len(), 4);
            assert!((values[0] - 27e-9).abs() < 1e-12);
            assert!((values[1] - 68e-9).abs() < 1e-12);
            assert!((values[2] - 220e-9).abs() < 1e-12);
            assert!((values[3] - 1.5e-6).abs() < 1e-12);
        } else {
            panic!("expected CapSwitched");
        }
    }

    #[test]
    fn parse_inductor_switched() {
        let (_, c) = component_def("L_hf: inductor_switched(27m, 33m, 47m, 68m, 82m, 150m)").unwrap();
        assert_eq!(c.id, "L_hf");
        if let ComponentKind::InductorSwitched(values) = c.kind {
            assert_eq!(values.len(), 6);
            assert!((values[0] - 27e-3).abs() < 1e-6);
            assert!((values[5] - 150e-3).abs() < 1e-6);
        } else {
            panic!("expected InductorSwitched");
        }
    }

    #[test]
    fn parse_rotary_switch() {
        let (_, c) = component_def(r#"SW1: rotary("20Hz", "30Hz", "60Hz", "100Hz")"#).unwrap();
        assert_eq!(c.id, "SW1");
        if let ComponentKind::RotarySwitch(positions) = c.kind {
            assert_eq!(positions.len(), 4);
            assert_eq!(positions[0], "20Hz");
            assert_eq!(positions[3], "100Hz");
        } else {
            panic!("expected RotarySwitch");
        }
    }

    #[test]
    fn parse_studio_equipment_file() {
        let src = r#"
equipment "Test EQ" {
  components {
    T1: transformer(10:1, 2H, pp, ct)
    C_lf: cap_switched(27n, 68n, 220n)
    L_hf: inductor_switched(27m, 47m, 82m)
    SW_freq: rotary("100Hz", "200Hz", "400Hz")
    R1: resistor(10k)
  }
  nets {
    in -> T1.a
    T1.b -> C_lf.a
    C_lf.b -> L_hf.a
    L_hf.b -> R1.a
    R1.b -> out
  }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert_eq!(def.name, "Test EQ");
        assert_eq!(def.components.len(), 5);
        assert!(def.components.iter().any(|c| matches!(c.kind, ComponentKind::Transformer(_))));
        assert!(def.components.iter().any(|c| matches!(c.kind, ComponentKind::CapSwitched(_))));
        assert!(def.components.iter().any(|c| matches!(c.kind, ComponentKind::InductorSwitched(_))));
        assert!(def.components.iter().any(|c| matches!(c.kind, ComponentKind::RotarySwitch(_))));
    }

    // ── Supply voltage parser tests ───────────────────────────────────────

    #[test]
    fn parse_supply_9v() {
        let src = r#"
pedal "9V Pedal" {
    supply 9V
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
        assert_eq!(def.name, "9V Pedal");
        let supply = def.supply.unwrap();
        assert_eq!(supply.voltage, 9.0);
        assert!(!supply.has_sag());
    }

    #[test]
    fn parse_supply_250v_tube_amp() {
        let src = r#"
pedal "Tube Amp" {
    supply 250V
    components {
        V1: triode(12ax7)
        R1: resistor(100k)
    }
    nets {
        in -> V1.grid
        V1.plate -> R1.a
        R1.b -> out
    }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert_eq!(def.name, "Tube Amp");
        assert_eq!(def.supply.unwrap().voltage, 250.0);
    }

    #[test]
    fn parse_supply_lowercase_v() {
        let src = r#"
pedal "Test" {
    supply 12v
    components {
        R1: resistor(10k)
    }
    nets {
        in -> out
    }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert_eq!(def.supply.unwrap().voltage, 12.0);
    }

    #[test]
    fn parse_supply_decimal() {
        let src = r#"
pedal "Test" {
    supply 9.6V
    components {
        R1: resistor(10k)
    }
    nets {
        in -> out
    }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert!((def.supply.unwrap().voltage - 9.6).abs() < 0.01);
    }

    #[test]
    fn parse_no_supply_defaults_none() {
        let src = r#"
pedal "Default" {
    components {
        R1: resistor(10k)
    }
    nets {
        in -> out
    }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert_eq!(def.supply, None);
    }

    #[test]
    fn parse_supply_block_tube_rectifier() {
        let src = r#"
pedal "Tube Amp Sag" {
    supply 480V {
        impedance: 150
        filter_cap: 40u
        rectifier: tube
    }
    components {
        V1: triode(12ax7)
        R1: resistor(100k)
    }
    nets {
        in -> V1.grid
        V1.plate -> R1.a
        R1.b -> out
    }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        let supply = def.supply.unwrap();
        assert_eq!(supply.voltage, 480.0);
        assert!((supply.impedance.unwrap() - 150.0).abs() < 1e-6);
        assert!((supply.filter_cap.unwrap() - 40e-6).abs() < 1e-12);
        assert_eq!(supply.rectifier, RectifierType::Tube);
        assert!(supply.has_sag());
    }

    #[test]
    fn parse_supply_block_solid_state() {
        let src = r#"
pedal "Modern Amp" {
    supply 480V {
        impedance: 5
        filter_cap: 220u
        rectifier: solid_state
    }
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
        let supply = def.supply.unwrap();
        assert_eq!(supply.voltage, 480.0);
        assert!((supply.impedance.unwrap() - 5.0).abs() < 1e-6);
        assert!((supply.filter_cap.unwrap() - 220e-6).abs() < 1e-12);
        assert_eq!(supply.rectifier, RectifierType::SolidState);
    }

    #[test]
    fn parse_supply_block_with_comments() {
        let src = r#"
pedal "Commented Supply" {
    supply 400V {
        impedance: 100    # tube rectifier + transformer resistance
        filter_cap: 47u   # main filter cap
        rectifier: tube    # GZ34
    }
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
        let supply = def.supply.unwrap();
        assert_eq!(supply.voltage, 400.0);
        assert!((supply.impedance.unwrap() - 100.0).abs() < 1e-6);
        assert!((supply.filter_cap.unwrap() - 47e-6).abs() < 1e-12);
        assert_eq!(supply.rectifier, RectifierType::Tube);
    }

    // ── Monitors section tests ────────────────────────────────────────

    #[test]
    fn parse_monitors_section() {
        let src = r#"
pedal "Tube Preamp" {
    supply 250V
    components {
        V1: triode(12ax7)
        R1: resistor(100k)
    }
    nets {
        in -> V1.grid
        V1.plate -> R1.a
        R1.b -> out
    }
    monitors {
        V1.plate_current -> "Tube 1" [vu]
        output -> "Output Level" [ppm]
        input -> "Input Level" [peak]
    }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert_eq!(def.monitors.len(), 3);

        let tube_mon = &def.monitors[0];
        assert_eq!(tube_mon.component, "V1");
        assert_eq!(tube_mon.property, "plate_current");
        assert_eq!(tube_mon.label, "Tube 1");
        assert_eq!(tube_mon.meter_type, MeterType::Vu);

        let output_mon = &def.monitors[1];
        assert_eq!(output_mon.component, "output");
        assert_eq!(output_mon.property, "level");
        assert_eq!(output_mon.meter_type, MeterType::Ppm);

        let input_mon = &def.monitors[2];
        assert_eq!(input_mon.component, "input");
        assert_eq!(input_mon.meter_type, MeterType::Peak);
    }

    #[test]
    fn parse_monitors_gain_reduction() {
        let src = r#"
pedal "Compressor" {
    components {
        R1: resistor(10k)
    }
    nets {
        in -> R1.a
        R1.b -> out
    }
    monitors {
        GR.reduction -> "Gain Reduction" [gr]
        supply.sag -> "Sag" [sag]
    }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert_eq!(def.monitors.len(), 2);
        assert_eq!(def.monitors[0].meter_type, MeterType::GainReduction);
        assert_eq!(def.monitors[1].meter_type, MeterType::SupplySag);
    }

    #[test]
    fn parse_empty_monitors() {
        let src = r#"
pedal "No Monitors" {
    components {
        R1: resistor(10k)
    }
    nets {
        in -> out
    }
}
"#;
        let def = parse_pedal_file(src).unwrap();
        assert!(def.monitors.is_empty());
    }
}
