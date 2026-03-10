//! AST types for parsed `.cab` (speaker cabinet) files.
//!
//! Mirrors the `.pedal` parser's `PedalDef` but describes the electro-mechano-
//! acoustic system of a speaker cabinet: drivers (with Thiele-Small parameters),
//! enclosure geometry, microphone placement, and environmental conditions.

use std::f64::consts::PI;

// ═══════════════════════════════════════════════════════════════════════════
// Top-level definition
// ═══════════════════════════════════════════════════════════════════════════

/// Top-level cabinet definition — the result of parsing a `.cab` file.
#[derive(Debug, Clone)]
pub struct CabDef {
    pub name: String,
    pub drivers: Vec<DriverDef>,
    pub enclosure: EnclosureDef,
    pub crossover: Option<CrossoverDef>,
    pub mics: Vec<MicDef>,
    pub environment: EnvironmentDef,
    pub controls: Vec<CabControlDef>,
    pub monitors: Vec<CabMonitorDef>,
}

// ═══════════════════════════════════════════════════════════════════════════
// Driver
// ═══════════════════════════════════════════════════════════════════════════

/// Loudspeaker driver with Thiele-Small parameters.
#[derive(Debug, Clone)]
pub struct DriverDef {
    /// Named identifier for multi-driver cabs (e.g., "upper_left").
    pub id: Option<String>,
    /// Human-readable model name (e.g., "Celestion Vintage 30").
    pub model: Option<String>,
    /// Preset speaker — loads default T-S params, overridable per-field.
    pub preset: Option<DriverPreset>,

    // ── Thiele-Small primitive parameters (7 independent values) ─────────
    /// DC voice coil resistance (Ω).
    pub re: Option<f64>,
    /// Voice coil inductance (H).
    pub le: Option<f64>,
    /// Force factor (T·m).
    pub bl: Option<f64>,
    /// Moving mass (kg).
    pub mms: Option<f64>,
    /// Suspension compliance (m/N).
    pub cms: Option<f64>,
    /// Mechanical resistance (N·s/m).
    pub rms: Option<f64>,
    /// Effective radiating area (m²).
    pub sd: Option<f64>,

    // ── Derived parameters (computed by validator if absent) ─────────────
    /// Resonant frequency (Hz).
    pub fs: Option<f64>,
    /// Mechanical Q factor.
    pub qms: Option<f64>,
    /// Electrical Q factor.
    pub qes: Option<f64>,
    /// Total Q factor.
    pub qts: Option<f64>,
    /// Equivalent compliance volume (m³).
    pub vas: Option<f64>,
    /// Reference efficiency (ratio, not percent).
    pub eta0: Option<f64>,
    /// Sensitivity (dB SPL, 1W/1m).
    pub spl: Option<f64>,
    /// Maximum linear excursion (m) — from suspension or derived from Bl/Cms.
    pub xmax: Option<f64>,

    // ── Sub-blocks ──────────────────────────────────────────────────────
    pub voice_coil: Option<VoiceCoilDef>,
    pub cone: Option<ConeDef>,
    pub suspension: Option<SuspensionDef>,

    // ── Multi-driver positioning ────────────────────────────────────────
    /// Number of identical copies (e.g., 4 for a 4×12).
    pub copies: Option<usize>,
    /// Positions for copies: [[x, y], ...] in meters from bottom-left.
    pub positions: Vec<[f64; 2]>,
    /// Single-driver position (alternative to positions vec).
    pub position: Option<[f64; 2]>,
}

impl Default for DriverDef {
    fn default() -> Self {
        Self {
            id: None,
            model: None,
            preset: None,
            re: None,
            le: None,
            bl: None,
            mms: None,
            cms: None,
            rms: None,
            sd: None,
            fs: None,
            qms: None,
            qes: None,
            qts: None,
            vas: None,
            eta0: None,
            spl: None,
            xmax: None,
            voice_coil: None,
            cone: None,
            suspension: None,
            copies: None,
            positions: Vec::new(),
            position: None,
        }
    }
}

/// Voice coil sub-block.
#[derive(Debug, Clone)]
pub struct VoiceCoilDef {
    /// Enable semi-inductance model (Le decreases with frequency).
    pub semi_inductance: bool,
    /// Eddy current coefficient for semi-inductance.
    pub ke: f64,
    /// Semi-inductance exponent (0.5 = strong eddy currents, 1.0 = pure inductor).
    pub n: f64,
    /// Thermal modeling parameters.
    pub thermal: Option<ThermalDef>,
}

impl Default for VoiceCoilDef {
    fn default() -> Self {
        Self {
            semi_inductance: false,
            ke: 0.05,
            n: 0.7,
            thermal: None,
        }
    }
}

/// Thermal modeling for voice coil.
#[derive(Debug, Clone)]
pub struct ThermalDef {
    /// Thermal mass (J/°C).
    pub capacity: f64,
    /// Thermal resistance coil → air (°C/W).
    pub r_ambient: f64,
    /// Thermal resistance coil → magnet (°C/W).
    pub r_magnet: f64,
    /// Temperature coefficient of Re (copper ≈ 0.00393 per °C).
    pub tc_re: f64,
    /// Temperature coefficient of Bl (negative, typically -0.001 to -0.002).
    pub tc_bl: f64,
}

/// Cone sub-block.
#[derive(Debug, Clone)]
pub struct ConeDef {
    pub material: ConeMaterial,
    pub dust_cap: DustCapType,
    pub surround: SurroundType,
    /// First breakup mode frequency (Hz). Derived from material + Sd if absent.
    pub breakup_freq: Option<f64>,
    /// Number of breakup modes to model (1–8).
    pub breakup_modes: usize,
    /// Q factor of breakup resonances.
    pub breakup_q: f64,
    /// Breakup amplitude relative to piston band (dB, typically negative).
    pub breakup_level: f64,
}

impl Default for ConeDef {
    fn default() -> Self {
        Self {
            material: ConeMaterial::Paper,
            dust_cap: DustCapType::Paper,
            surround: SurroundType::Cloth,
            breakup_freq: None,
            breakup_modes: 3,
            breakup_q: 4.5,
            breakup_level: -12.0,
        }
    }
}

/// Suspension nonlinearity sub-block.
#[derive(Debug, Clone)]
pub struct SuspensionDef {
    /// Maximum linear excursion (m).
    pub xmax: Option<f64>,
    /// Mechanical limit (m).
    pub xmech: Option<f64>,
    /// Compliance curve shape.
    pub curve: SuspensionCurve,
    /// Hardening exponent (for hardening curve).
    pub hardening_exp: f64,
    /// BL symmetry (1.0 = symmetric, <1.0 = asymmetric).
    pub bl_symmetry: f64,
    /// BL profile shape.
    pub bl_profile: BlProfile,
    /// Suspension creep (0.0 = broken in, 1.0 = brand new).
    pub creep: f64,
}

impl Default for SuspensionDef {
    fn default() -> Self {
        Self {
            xmax: None,
            xmech: None,
            curve: SuspensionCurve::Hardening,
            hardening_exp: 2.0,
            bl_symmetry: 1.0,
            bl_profile: BlProfile::Gaussian,
            creep: 0.0,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Enclosure
// ═══════════════════════════════════════════════════════════════════════════

/// Enclosure (cabinet box) definition.
#[derive(Debug, Clone)]
pub struct EnclosureDef {
    pub enclosure_type: EnclosureType,
    /// Internal air volume (m³). Required for most types.
    pub volume: Option<f64>,

    // ── Open back ───────────────────────────────────────────────────────
    /// Rear panel open area (0.0–1.0 ratio).
    pub open_area: Option<f64>,
    /// Front-to-back internal depth (m).
    pub depth: Option<f64>,

    // ── Bass reflex ─────────────────────────────────────────────────────
    pub port: Option<PortDef>,

    // ── Transmission line ───────────────────────────────────────────────
    pub line_length: Option<f64>,
    pub taper: Option<TaperType>,
    pub stuffing: Option<StuffingLevel>,

    // ── Horn ────────────────────────────────────────────────────────────
    pub throat_area: Option<f64>,
    pub mouth_area: Option<f64>,
    pub horn_length: Option<f64>,
    pub horn_profile: Option<HornProfile>,

    // ── Bandpass ────────────────────────────────────────────────────────
    pub volume_front: Option<f64>,
    pub volume_rear: Option<f64>,

    // ── Sealed ──────────────────────────────────────────────────────────
    /// Air leakage resistance (Ω acoustic). Default 10M (well sealed).
    pub leakage: Option<f64>,

    // ── Sub-blocks ──────────────────────────────────────────────────────
    pub construction: Option<ConstructionDef>,
    pub damping: Option<DampingDef>,
    pub baffle: Option<BaffleDef>,
}

impl Default for EnclosureDef {
    fn default() -> Self {
        Self {
            enclosure_type: EnclosureType::Sealed,
            volume: None,
            open_area: None,
            depth: None,
            port: None,
            line_length: None,
            taper: None,
            stuffing: None,
            throat_area: None,
            mouth_area: None,
            horn_length: None,
            horn_profile: None,
            volume_front: None,
            volume_rear: None,
            leakage: None,
            construction: None,
            damping: None,
            baffle: None,
        }
    }
}

/// Port definition for bass reflex / bandpass enclosures.
#[derive(Debug, Clone)]
pub struct PortDef {
    pub shape: PortShape,
    /// Diameter for round ports (m).
    pub diameter: Option<f64>,
    /// Width for slot ports (m).
    pub width: Option<f64>,
    /// Height for slot ports (m).
    pub height: Option<f64>,
    /// Port tube length (m).
    pub length: f64,
    /// Flare type.
    pub flare: PortFlare,
    // ── Passive radiator (alternative) ──────────────────────────────────
    /// Passive radiator moving mass (kg).
    pub mmp: Option<f64>,
    /// Passive radiator compliance (m/N).
    pub cmp: Option<f64>,
    /// Passive radiator effective area (m²).
    pub sd_pr: Option<f64>,
}

impl Default for PortDef {
    fn default() -> Self {
        Self {
            shape: PortShape::Round,
            diameter: None,
            width: None,
            height: None,
            length: 0.18,
            flare: PortFlare::None,
            mmp: None,
            cmp: None,
            sd_pr: None,
        }
    }
}

/// Cabinet construction details.
#[derive(Debug, Clone)]
pub struct ConstructionDef {
    pub material: PanelMaterial,
    /// Panel thickness (m).
    pub thickness: f64,
    /// External dimensions (m). Estimated from volume if absent.
    pub width: Option<f64>,
    pub height: Option<f64>,
    pub depth: Option<f64>,
    /// Internal bracing.
    pub bracing: Vec<BraceDef>,
    pub joints: JointType,
    pub corners: CornerType,
    pub covering: CoveringType,
    /// Rear panel material override (defaults to same as sides).
    pub rear_panel: Option<PanelMaterial>,
    pub rear_panel_thickness: Option<f64>,
}

impl Default for ConstructionDef {
    fn default() -> Self {
        Self {
            material: PanelMaterial::BirchPly,
            thickness: 0.018,
            width: None,
            height: None,
            depth: None,
            bracing: Vec::new(),
            joints: JointType::Dado,
            corners: CornerType::MetalBracket,
            covering: CoveringType::Tolex,
            rear_panel: None,
            rear_panel_thickness: None,
        }
    }
}

/// A single brace in the cabinet.
#[derive(Debug, Clone)]
pub struct BraceDef {
    pub id: String,
    pub orientation: BraceOrientation,
    /// Position along the perpendicular axis (m).
    pub position: f64,
    /// Second position for cross braces (m).
    pub position2: Option<f64>,
    /// Cross-section width (m).
    pub cs_width: Option<f64>,
    /// Cross-section height (m).
    pub cs_height: Option<f64>,
}

/// Internal damping material.
#[derive(Debug, Clone)]
pub struct DampingDef {
    pub material: DampingMaterial,
    /// Coverage (0.0–1.0 ratio).
    pub coverage: f64,
    /// Thickness of damping material (m).
    pub thickness: f64,
}

impl Default for DampingDef {
    fn default() -> Self {
        Self {
            material: DampingMaterial::None,
            coverage: 0.0,
            thickness: 0.025,
        }
    }
}

/// Baffle (front panel) details.
#[derive(Debug, Clone)]
pub struct BaffleDef {
    /// Single-driver position on baffle [x, y] in meters from bottom-left.
    pub driver_position: Option<[f64; 2]>,
    pub edge: BaffleEdge,
    pub grille: GrilleType,
    /// Gap between grille and baffle face (m).
    pub grille_distance: f64,
    /// Baffle thickness override (m).
    pub thickness: Option<f64>,
}

impl Default for BaffleDef {
    fn default() -> Self {
        Self {
            driver_position: None,
            edge: BaffleEdge::Square,
            grille: GrilleType::Cloth,
            grille_distance: 0.015,
            thickness: None,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Crossover
// ═══════════════════════════════════════════════════════════════════════════

/// Crossover for multi-way cabinets.
#[derive(Debug, Clone)]
pub struct CrossoverDef {
    pub crossover_type: CrossoverType,
    pub topology: CrossoverTopology,
    pub frequency: Option<f64>,
    pub high_driver: Option<String>,
    pub low_driver: Option<String>,
}

// ═══════════════════════════════════════════════════════════════════════════
// Microphone
// ═══════════════════════════════════════════════════════════════════════════

/// Microphone position and model.
#[derive(Debug, Clone)]
pub struct MicDef {
    /// Named identifier (e.g., "close", "room", "rear").
    pub id: String,
    /// Microphone model.
    pub model: MicModel,
    /// Position [x, y, z] in meters relative to target driver cone center.
    /// x: left(-)/right(+), y: down(-)/up(+), z: distance from grille.
    pub position: [f64; 3],
    /// Off-axis angle in degrees (0 = on-axis).
    pub angle: f64,
    /// Target driver ID for multi-driver cabs.
    pub target: Option<String>,
}

// ═══════════════════════════════════════════════════════════════════════════
// Environment
// ═══════════════════════════════════════════════════════════════════════════

/// Environmental conditions affecting acoustic behavior.
#[derive(Debug, Clone)]
pub struct EnvironmentDef {
    /// Temperature in degrees Celsius.
    pub temperature: f64,
    /// Relative humidity (0.0–1.0).
    pub humidity: f64,
    /// Altitude in meters above sea level.
    pub altitude: f64,
}

impl Default for EnvironmentDef {
    fn default() -> Self {
        Self {
            temperature: 20.0,
            humidity: 0.5,
            altitude: 0.0,
        }
    }
}

impl EnvironmentDef {
    /// Speed of sound in m/s at the given temperature.
    pub fn speed_of_sound(&self) -> f64 {
        331.3 * (1.0 + self.temperature / 273.15).sqrt()
    }

    /// Air density in kg/m³ at the given altitude.
    pub fn air_density(&self) -> f64 {
        1.225 * (1.0 - 2.25577e-5 * self.altitude).powf(5.25588)
    }

    /// Atmospheric pressure in Pa at the given altitude.
    pub fn pressure(&self) -> f64 {
        101325.0 * (1.0 - 2.25577e-5 * self.altitude).powf(5.25588)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Controls & Monitors
// ═══════════════════════════════════════════════════════════════════════════

/// A user-exposed control parameter.
#[derive(Debug, Clone)]
pub struct CabControlDef {
    /// Dotted path to the parameter (e.g., "driver.Mms", "mic.close.position.x").
    pub path: String,
    /// Display label.
    pub label: String,
    /// Minimum and maximum values.
    pub range: (f64, f64),
    /// Default value.
    pub default: f64,
}

/// A real-time metering output.
#[derive(Debug, Clone)]
pub struct CabMonitorDef {
    /// Dotted path to the monitored value.
    pub path: String,
    /// Display label.
    pub label: String,
    /// Meter type.
    pub meter_type: CabMeterType,
}

// ═══════════════════════════════════════════════════════════════════════════
// Enums
// ═══════════════════════════════════════════════════════════════════════════

/// Enclosure type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EnclosureType {
    Sealed,
    OpenBack,
    BassReflex,
    TransmissionLine,
    Horn,
    InfiniteBaffle,
    Bandpass,
}

/// Speaker preset — loads default T-S parameters from datasheet values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriverPreset {
    // Guitar speakers
    V30,
    G12mGreenback,
    G12hAnniversary,
    G12t75,
    G12_65,
    G12Blue,
    G12Creamback,
    G12Evh,
    P12r,
    C12n,
    C12q,
    P10r,
    SwampThang,
    ManOWar,
    Cv75,
    Legend1258,
    TexasHeat,
    CannabisRex,
    Ev12l,
    JblD120,
    WbEt65,
    WbRetro30,
    // Bass speakers
    BassliteS2010,
    Kappalite3015,
    Ampeg10,
}

/// Cone material — affects stiffness, damping, and breakup characteristics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ConeMaterial {
    #[default]
    Paper,
    Kevlar,
    Aluminum,
    Ceramic,
    Hemp,
    NeoPaper,
    Pulp,
    CarbonFiber,
}

impl ConeMaterial {
    /// Default first breakup mode frequency for a 12" (30cm) driver.
    /// Scales inversely with effective diameter.
    pub fn default_breakup_freq_12in(&self) -> f64 {
        match self {
            Self::Paper => 4000.0,
            Self::Kevlar => 6000.0,
            Self::Aluminum => 8500.0,
            Self::Ceramic => 10000.0,
            Self::Hemp => 3500.0,
            Self::NeoPaper => 5000.0,
            Self::Pulp => 2500.0,
            Self::CarbonFiber => 7500.0,
        }
    }

    /// Relative stiffness (arbitrary units, paper = 1.0).
    pub fn relative_stiffness(&self) -> f64 {
        match self {
            Self::Paper => 1.0,
            Self::Kevlar => 2.5,
            Self::Aluminum => 4.0,
            Self::Ceramic => 6.0,
            Self::Hemp => 0.9,
            Self::NeoPaper => 1.5,
            Self::Pulp => 0.6,
            Self::CarbonFiber => 3.5,
        }
    }

    /// Internal damping factor (higher = more damped breakup).
    pub fn damping_factor(&self) -> f64 {
        match self {
            Self::Paper => 0.08,
            Self::Kevlar => 0.04,
            Self::Aluminum => 0.01,
            Self::Ceramic => 0.005,
            Self::Hemp => 0.09,
            Self::NeoPaper => 0.05,
            Self::Pulp => 0.12,
            Self::CarbonFiber => 0.03,
        }
    }
}

/// Dust cap type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DustCapType {
    #[default]
    Paper,
    Felt,
    Aluminum,
    Screen,
    PhasePlug,
    None,
}

/// Surround type — affects HF rolloff.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SurroundType {
    #[default]
    Cloth,
    Rubber,
    Foam,
    Accordion,
}

/// Suspension compliance curve shape.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SuspensionCurve {
    Linear,
    #[default]
    Hardening,
    Softening,
}

/// BL(x) profile shape.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BlProfile {
    #[default]
    Gaussian,
    FlatTop,
    Tilted,
}

/// Port shape.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PortShape {
    #[default]
    Round,
    Slot,
    PassiveRadiator,
}

/// Port flare type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PortFlare {
    #[default]
    None,
    Single,
    Double,
}

/// Transmission line taper.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TaperType {
    Constant,
    Exponential,
    Conical,
}

/// Stuffing level for transmission line enclosures.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StuffingLevel {
    None,
    Light,
    Medium,
    Heavy,
}

/// Horn profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HornProfile {
    Conical,
    Exponential,
    Hyperbolic,
    Tractrix,
}

/// Panel / wood material.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PanelMaterial {
    #[default]
    BirchPly,
    Pine,
    Mdf,
    ParticleBoard,
    Poplar,
    MarinePly,
    Oak,
    Mahogany,
    Cedar,
    BambooPly,
}

impl PanelMaterial {
    /// Density in kg/m³.
    pub fn density(&self) -> f64 {
        match self {
            Self::BirchPly => 680.0,
            Self::Pine => 500.0,
            Self::Mdf => 750.0,
            Self::ParticleBoard => 650.0,
            Self::Poplar => 420.0,
            Self::MarinePly => 700.0,
            Self::Oak => 750.0,
            Self::Mahogany => 550.0,
            Self::Cedar => 380.0,
            Self::BambooPly => 700.0,
        }
    }

    /// Young's modulus in Pa.
    pub fn youngs_modulus(&self) -> f64 {
        match self {
            Self::BirchPly => 12.0e9,
            Self::Pine => 9.0e9,
            Self::Mdf => 4.0e9,
            Self::ParticleBoard => 3.0e9,
            Self::Poplar => 8.0e9,
            Self::MarinePly => 13.0e9,
            Self::Oak => 12.0e9,
            Self::Mahogany => 10.0e9,
            Self::Cedar => 6.0e9,
            Self::BambooPly => 14.0e9,
        }
    }

    /// Damping ratio (loss factor).
    pub fn damping_ratio(&self) -> f64 {
        match self {
            Self::BirchPly => 0.025,
            Self::Pine => 0.045,
            Self::Mdf => 0.080,
            Self::ParticleBoard => 0.090,
            Self::Poplar => 0.040,
            Self::MarinePly => 0.020,
            Self::Oak => 0.030,
            Self::Mahogany => 0.035,
            Self::Cedar => 0.055,
            Self::BambooPly => 0.020,
        }
    }

    /// Poisson's ratio (typical for each wood type).
    pub fn poissons_ratio(&self) -> f64 {
        match self {
            Self::Mdf | Self::ParticleBoard => 0.25,
            _ => 0.3,
        }
    }

    /// Flexural rigidity D = E·h³ / (12·(1−ν²)) for a panel of given thickness.
    pub fn flexural_rigidity(&self, thickness: f64) -> f64 {
        let e = self.youngs_modulus();
        let nu = self.poissons_ratio();
        e * thickness.powi(3) / (12.0 * (1.0 - nu * nu))
    }
}

/// Covering material.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CoveringType {
    None,
    #[default]
    Tolex,
    Tweed,
    Vinyl,
    Carpet,
    RatFur,
}

impl CoveringType {
    /// Added surface mass in kg/m².
    pub fn added_mass(&self) -> f64 {
        match self {
            Self::None => 0.0,
            Self::Tolex => 0.8,
            Self::Tweed => 0.4,
            Self::Vinyl => 1.0,
            Self::Carpet => 1.5,
            Self::RatFur => 1.2,
        }
    }

    /// Damping multiplier (1.0 = no added damping).
    pub fn damping_multiplier(&self) -> f64 {
        match self {
            Self::None => 1.0,
            Self::Tolex => 1.3,
            Self::Tweed => 1.1,
            Self::Vinyl => 1.4,
            Self::Carpet => 1.8,
            Self::RatFur => 1.6,
        }
    }
}

/// Internal damping material.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DampingMaterial {
    #[default]
    None,
    PolyesterFill,
    Fiberglass,
    CottonBatting,
    AcousticFoam,
    Wool,
    LongHairWool,
    Dacron,
}

impl DampingMaterial {
    /// Absorption coefficient at mid-frequencies (500–2000 Hz).
    pub fn absorption_coefficient(&self) -> f64 {
        match self {
            Self::None => 0.0,
            Self::PolyesterFill => 0.5,
            Self::Fiberglass => 0.8,
            Self::CottonBatting => 0.3,
            Self::AcousticFoam => 0.6,
            Self::Wool => 0.7,
            Self::LongHairWool => 0.9,
            Self::Dacron => 0.45,
        }
    }
}

/// Joint type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum JointType {
    Butt,
    #[default]
    Dado,
    Rabbet,
    Mitre,
    Finger,
    Dovetail,
}

/// Corner type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CornerType {
    Butt,
    FingerJoint,
    Dovetail,
    #[default]
    MetalBracket,
}

/// Grille type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GrilleType {
    None,
    #[default]
    Cloth,
    MetalPerf,
    ExpandedMetal,
}

/// Brace orientation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BraceOrientation {
    Horizontal,
    Vertical,
    Cross,
}

/// Baffle edge treatment.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BaffleEdge {
    Square,
    /// Rounded edge with radius in meters.
    Rounded(f64),
    /// Chamfered edge with width in meters.
    Chamfered(f64),
}

/// Microphone model.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MicModel {
    #[default]
    Sm57,
    Sm7b,
    Md421,
    E906,
    E609,
    M160,
    R121,
    R101,
    Fathead,
    U87,
    C414,
    Km84,
    C451,
    Re20,
    M88,
    M201,
    Flat,
    Measurement,
    Custom,
}

/// Polar pattern of a microphone.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolarPattern {
    Omni,
    Cardioid,
    Supercardioid,
    Hypercardioid,
    Figure8,
}

/// Crossover type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrossoverType {
    Passive,
    Active,
    None,
}

/// Crossover filter topology (Butterworth order).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrossoverTopology {
    FirstOrder,
    SecondOrder,
    ThirdOrder,
    FourthOrder,
}

/// Monitor meter type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CabMeterType {
    Peak,
    Vu,
    Ppm,
    Thermal,
}

// ═══════════════════════════════════════════════════════════════════════════
// Helper: effective piston radius from Sd
// ═══════════════════════════════════════════════════════════════════════════

/// Compute effective piston radius from radiating area Sd.
pub fn piston_radius(sd: f64) -> f64 {
    (sd / PI).sqrt()
}

// ═══════════════════════════════════════════════════════════════════════════
// Panel mode result
// ═══════════════════════════════════════════════════════════════════════════

/// A single panel vibration mode.
#[derive(Debug, Clone)]
pub struct PanelMode {
    /// Mode indices (m, n).
    pub mode: (usize, usize),
    /// Resonant frequency (Hz).
    pub frequency: f64,
    /// Q factor of the mode.
    pub q: f64,
    /// Relative level in dB (referenced to first mode).
    pub level_db: f64,
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn environment_speed_of_sound() {
        let env = EnvironmentDef::default();
        let c = env.speed_of_sound();
        assert!((c - 343.2).abs() < 0.5, "Speed of sound at 20°C should be ~343 m/s, got {c}");
    }

    #[test]
    fn environment_air_density_sea_level() {
        let env = EnvironmentDef::default();
        let rho = env.air_density();
        assert!((rho - 1.225).abs() < 0.01, "Air density at sea level should be ~1.225 kg/m³, got {rho}");
    }

    #[test]
    fn piston_radius_12in() {
        // A 12" driver has Sd ≈ 531 cm² = 0.0531 m²
        let r = piston_radius(531e-4);
        let diameter_inches = r * 2.0 / 0.0254;
        assert!(
            (diameter_inches - 10.3).abs() < 1.0,
            "Effective diameter for Sd=531cm² should be ~10 inches, got {diameter_inches:.1}"
        );
    }

    #[test]
    fn panel_material_rigidity() {
        let d = PanelMaterial::BirchPly.flexural_rigidity(0.018);
        // D = 12e9 * 0.018³ / (12 * (1 - 0.09)) = 12e9 * 5.832e-6 / 10.92 ≈ 6406
        assert!(d > 5000.0 && d < 8000.0, "Birch ply 18mm rigidity should be ~6400, got {d:.0}");
    }

    #[test]
    fn cone_material_properties() {
        assert!(ConeMaterial::Paper.default_breakup_freq_12in() < ConeMaterial::Aluminum.default_breakup_freq_12in());
        assert!(ConeMaterial::Paper.damping_factor() > ConeMaterial::Aluminum.damping_factor());
    }

    #[test]
    fn covering_properties() {
        assert_eq!(CoveringType::None.added_mass(), 0.0);
        assert!(CoveringType::Carpet.damping_multiplier() > CoveringType::Tweed.damping_multiplier());
    }
}
