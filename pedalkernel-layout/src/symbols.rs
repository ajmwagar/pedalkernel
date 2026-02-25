//! Component symbol definitions.
//!
//! Each component type maps to a vector symbol defined as a series of path
//! commands (move_to, line_to, arc_to, close). These render at any resolution
//! with zero texture memory when drawn with Lyon or a similar path rasterizer.

use pedalkernel::dsl::ComponentKind;
use serde::{Deserialize, Serialize};

/// A symbol identifier string used in the layout JSON.
/// At runtime, the shader/renderer maps this to the actual path commands.
pub fn symbol_for_kind(kind: &ComponentKind) -> String {
    match kind {
        ComponentKind::Resistor(_) => "resistor".into(),
        ComponentKind::Capacitor(_) => "capacitor".into(),
        ComponentKind::Inductor(_) => "inductor".into(),
        ComponentKind::DiodePair(_) => "diode".into(),
        ComponentKind::Diode(_) => "diode".into(),
        ComponentKind::Zener(_) => "zener".into(),
        ComponentKind::Potentiometer(_) => "pot".into(),
        ComponentKind::Npn(_) => "npn_bjt".into(),
        ComponentKind::Pnp(_) => "pnp_bjt".into(),
        ComponentKind::OpAmp(_) => "opamp".into(),
        ComponentKind::NJfet(_) => "njfet".into(),
        ComponentKind::PJfet(_) => "pjfet".into(),
        ComponentKind::Triode(_) => "triode".into(),
        ComponentKind::Pentode(_) => "pentode".into(),
        ComponentKind::Nmos(_) => "nmos".into(),
        ComponentKind::Pmos(_) => "pmos".into(),
        ComponentKind::Transformer(_) => "transformer".into(),
        ComponentKind::Photocoupler(_) => "photocoupler".into(),
        ComponentKind::Lfo(..) => "lfo".into(),
        ComponentKind::EnvelopeFollower(..) => "envelope".into(),
        ComponentKind::Bbd(_) => "ic_chip".into(),
        ComponentKind::DelayLine(..) => "delay".into(),
        ComponentKind::Tap(..) => "tap".into(),
        ComponentKind::Neon(_) => "neon".into(),
        ComponentKind::Vco(_) => "ic_chip".into(),
        ComponentKind::Vcf(_) => "ic_chip".into(),
        ComponentKind::Vca(_) => "ic_chip".into(),
        ComponentKind::Comparator(_) => "ic_chip".into(),
        ComponentKind::AnalogSwitch(_) => "ic_chip".into(),
        ComponentKind::MatchedNpn(_) => "npn_bjt".into(),
        ComponentKind::MatchedPnp(_) => "pnp_bjt".into(),
        ComponentKind::Tempco(..) => "resistor".into(),
        ComponentKind::CapSwitched(_) => "capacitor".into(),
        ComponentKind::InductorSwitched(_) => "inductor".into(),
        ComponentKind::ResistorSwitched(_) => "resistor".into(),
        ComponentKind::RotarySwitch(_) => "switch".into(),
        ComponentKind::Switch(_) => "switch".into(),
    }
}

/// Enumeration of all symbol types used in the layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SymbolType {
    Resistor,
    Capacitor,
    ElectrolyticCap,
    Inductor,
    Diode,
    Zener,
    Triode,
    Pentode,
    NJfet,
    PJfet,
    NpnBjt,
    PnpBjt,
    Nmos,
    Pmos,
    OpAmp,
    Pot,
    Transformer,
    Photocoupler,
    IcChip,
    Ground,
    Supply,
    Speaker,
    Input,
    Output,
    Lfo,
    Neon,
    Switch,
}

/// A path command for vector symbol rendering.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum PathCmd {
    /// Move to (x, y) without drawing.
    MoveTo(f32, f32),
    /// Draw a line to (x, y).
    LineTo(f32, f32),
    /// Draw an arc: center (cx, cy), radius, start_angle, end_angle (radians).
    ArcTo(f32, f32, f32, f32, f32),
    /// Close the current sub-path.
    Close,
}

/// Get the path commands for a given symbol type.
///
/// All symbols are drawn in a normalized coordinate space centered at (0, 0)
/// with a nominal size of ~30×30 units. The renderer scales and translates
/// as needed.
pub fn symbol_paths(sym: SymbolType) -> Vec<PathCmd> {
    match sym {
        SymbolType::Resistor => resistor_path(),
        SymbolType::Capacitor => capacitor_path(),
        SymbolType::ElectrolyticCap => electrolytic_cap_path(),
        SymbolType::Inductor => inductor_path(),
        SymbolType::Diode => diode_path(),
        SymbolType::Zener => zener_path(),
        SymbolType::Triode => triode_path(),
        SymbolType::Pentode => pentode_path(),
        SymbolType::NJfet => njfet_path(),
        SymbolType::PJfet => pjfet_path(),
        SymbolType::NpnBjt => npn_bjt_path(),
        SymbolType::PnpBjt => pnp_bjt_path(),
        SymbolType::Nmos => nmos_path(),
        SymbolType::Pmos => pmos_path(),
        SymbolType::OpAmp => opamp_path(),
        SymbolType::Pot => pot_path(),
        SymbolType::Transformer => transformer_path(),
        SymbolType::Photocoupler => photocoupler_path(),
        SymbolType::IcChip => ic_chip_path(),
        SymbolType::Ground => ground_path(),
        SymbolType::Supply => supply_path(),
        SymbolType::Speaker => speaker_path(),
        SymbolType::Input => input_path(),
        SymbolType::Output => output_path(),
        SymbolType::Lfo => lfo_path(),
        SymbolType::Neon => neon_path(),
        SymbolType::Switch => switch_path(),
    }
}

// ---------------------------------------------------------------------------
// Symbol path definitions
// ---------------------------------------------------------------------------

/// American-style zigzag resistor.
fn resistor_path() -> Vec<PathCmd> {
    vec![
        PathCmd::MoveTo(-15.0, 0.0),
        PathCmd::LineTo(-10.0, 0.0),
        PathCmd::LineTo(-8.0, -5.0),
        PathCmd::LineTo(-4.0, 5.0),
        PathCmd::LineTo(0.0, -5.0),
        PathCmd::LineTo(4.0, 5.0),
        PathCmd::LineTo(8.0, -5.0),
        PathCmd::LineTo(10.0, 0.0),
        PathCmd::LineTo(15.0, 0.0),
    ]
}

/// Non-polarized capacitor: two parallel lines.
fn capacitor_path() -> Vec<PathCmd> {
    vec![
        // Left lead
        PathCmd::MoveTo(-15.0, 0.0),
        PathCmd::LineTo(-3.0, 0.0),
        // Left plate
        PathCmd::MoveTo(-3.0, -8.0),
        PathCmd::LineTo(-3.0, 8.0),
        // Right plate
        PathCmd::MoveTo(3.0, -8.0),
        PathCmd::LineTo(3.0, 8.0),
        // Right lead
        PathCmd::MoveTo(3.0, 0.0),
        PathCmd::LineTo(15.0, 0.0),
    ]
}

/// Electrolytic capacitor: one straight plate, one curved.
fn electrolytic_cap_path() -> Vec<PathCmd> {
    vec![
        PathCmd::MoveTo(-15.0, 0.0),
        PathCmd::LineTo(-3.0, 0.0),
        // Straight plate (positive)
        PathCmd::MoveTo(-3.0, -8.0),
        PathCmd::LineTo(-3.0, 8.0),
        // Curved plate (negative) — approximated with line segments
        PathCmd::MoveTo(3.0, -8.0),
        PathCmd::ArcTo(3.0, 0.0, 8.0, -1.0, 1.0),
        PathCmd::MoveTo(3.0, 0.0),
        PathCmd::LineTo(15.0, 0.0),
        // Plus sign near positive plate
        PathCmd::MoveTo(-8.0, -6.0),
        PathCmd::LineTo(-8.0, -10.0),
        PathCmd::MoveTo(-10.0, -8.0),
        PathCmd::LineTo(-6.0, -8.0),
    ]
}

/// Inductor: series of bumps (coil).
fn inductor_path() -> Vec<PathCmd> {
    vec![
        PathCmd::MoveTo(-15.0, 0.0),
        PathCmd::LineTo(-10.0, 0.0),
        PathCmd::ArcTo(-7.5, 0.0, 2.5, std::f32::consts::PI, 0.0),
        PathCmd::ArcTo(-2.5, 0.0, 2.5, std::f32::consts::PI, 0.0),
        PathCmd::ArcTo(2.5, 0.0, 2.5, std::f32::consts::PI, 0.0),
        PathCmd::ArcTo(7.5, 0.0, 2.5, std::f32::consts::PI, 0.0),
        PathCmd::LineTo(15.0, 0.0),
    ]
}

/// Diode: triangle + line.
fn diode_path() -> Vec<PathCmd> {
    vec![
        PathCmd::MoveTo(-15.0, 0.0),
        PathCmd::LineTo(-5.0, 0.0),
        // Triangle (anode side)
        PathCmd::MoveTo(-5.0, -7.0),
        PathCmd::LineTo(-5.0, 7.0),
        PathCmd::LineTo(5.0, 0.0),
        PathCmd::Close,
        // Cathode bar
        PathCmd::MoveTo(5.0, -7.0),
        PathCmd::LineTo(5.0, 7.0),
        // Lead
        PathCmd::MoveTo(5.0, 0.0),
        PathCmd::LineTo(15.0, 0.0),
    ]
}

/// Zener diode: triangle + bent line.
fn zener_path() -> Vec<PathCmd> {
    vec![
        PathCmd::MoveTo(-15.0, 0.0),
        PathCmd::LineTo(-5.0, 0.0),
        PathCmd::MoveTo(-5.0, -7.0),
        PathCmd::LineTo(-5.0, 7.0),
        PathCmd::LineTo(5.0, 0.0),
        PathCmd::Close,
        // Zener cathode (bent bar)
        PathCmd::MoveTo(3.0, -9.0),
        PathCmd::LineTo(5.0, -7.0),
        PathCmd::LineTo(5.0, 7.0),
        PathCmd::LineTo(7.0, 9.0),
        PathCmd::MoveTo(5.0, 0.0),
        PathCmd::LineTo(15.0, 0.0),
    ]
}

/// Triode: circle with grid, plate, cathode.
fn triode_path() -> Vec<PathCmd> {
    vec![
        // Envelope (circle)
        PathCmd::ArcTo(0.0, 0.0, 12.0, 0.0, std::f32::consts::TAU),
        // Plate (top)
        PathCmd::MoveTo(-6.0, -8.0),
        PathCmd::LineTo(6.0, -8.0),
        PathCmd::MoveTo(0.0, -8.0),
        PathCmd::LineTo(0.0, -15.0),
        // Grid (dashed horizontal line in middle)
        PathCmd::MoveTo(-8.0, 0.0),
        PathCmd::LineTo(-5.0, 0.0),
        PathCmd::MoveTo(-3.0, 0.0),
        PathCmd::LineTo(0.0, 0.0),
        PathCmd::MoveTo(2.0, 0.0),
        PathCmd::LineTo(5.0, 0.0),
        // Grid lead
        PathCmd::MoveTo(-8.0, 0.0),
        PathCmd::LineTo(-15.0, 0.0),
        // Cathode (bottom arc)
        PathCmd::ArcTo(0.0, 8.0, 6.0, std::f32::consts::PI, 0.0),
        PathCmd::MoveTo(0.0, 8.0),
        PathCmd::LineTo(0.0, 15.0),
    ]
}

/// Pentode: circle with grid, screen, plate, cathode.
fn pentode_path() -> Vec<PathCmd> {
    vec![
        // Envelope
        PathCmd::ArcTo(0.0, 0.0, 14.0, 0.0, std::f32::consts::TAU),
        // Plate
        PathCmd::MoveTo(-6.0, -10.0),
        PathCmd::LineTo(6.0, -10.0),
        PathCmd::MoveTo(0.0, -10.0),
        PathCmd::LineTo(0.0, -18.0),
        // Screen grid (dashed, between plate and control grid)
        PathCmd::MoveTo(-6.0, -5.0),
        PathCmd::LineTo(-3.0, -5.0),
        PathCmd::MoveTo(0.0, -5.0),
        PathCmd::LineTo(3.0, -5.0),
        // Screen lead
        PathCmd::MoveTo(6.0, -5.0),
        PathCmd::LineTo(18.0, -5.0),
        // Control grid (dashed)
        PathCmd::MoveTo(-8.0, 2.0),
        PathCmd::LineTo(-5.0, 2.0),
        PathCmd::MoveTo(-3.0, 2.0),
        PathCmd::LineTo(0.0, 2.0),
        PathCmd::MoveTo(2.0, 2.0),
        PathCmd::LineTo(5.0, 2.0),
        // Grid lead
        PathCmd::MoveTo(-8.0, 2.0),
        PathCmd::LineTo(-18.0, 2.0),
        // Cathode
        PathCmd::ArcTo(0.0, 10.0, 6.0, std::f32::consts::PI, 0.0),
        PathCmd::MoveTo(0.0, 10.0),
        PathCmd::LineTo(0.0, 18.0),
    ]
}

/// N-channel JFET.
fn njfet_path() -> Vec<PathCmd> {
    vec![
        // Channel
        PathCmd::MoveTo(0.0, -10.0),
        PathCmd::LineTo(0.0, 10.0),
        // Drain (top)
        PathCmd::MoveTo(0.0, -6.0),
        PathCmd::LineTo(8.0, -6.0),
        PathCmd::LineTo(8.0, -15.0),
        // Source (bottom)
        PathCmd::MoveTo(0.0, 6.0),
        PathCmd::LineTo(8.0, 6.0),
        PathCmd::LineTo(8.0, 15.0),
        // Gate (with arrow pointing in)
        PathCmd::MoveTo(-15.0, 0.0),
        PathCmd::LineTo(0.0, 0.0),
        // Arrow
        PathCmd::MoveTo(-2.0, -3.0),
        PathCmd::LineTo(0.0, 0.0),
        PathCmd::LineTo(-2.0, 3.0),
    ]
}

/// P-channel JFET.
fn pjfet_path() -> Vec<PathCmd> {
    vec![
        PathCmd::MoveTo(0.0, -10.0),
        PathCmd::LineTo(0.0, 10.0),
        PathCmd::MoveTo(0.0, -6.0),
        PathCmd::LineTo(8.0, -6.0),
        PathCmd::LineTo(8.0, -15.0),
        PathCmd::MoveTo(0.0, 6.0),
        PathCmd::LineTo(8.0, 6.0),
        PathCmd::LineTo(8.0, 15.0),
        PathCmd::MoveTo(-15.0, 0.0),
        PathCmd::LineTo(0.0, 0.0),
        // Arrow pointing out
        PathCmd::MoveTo(-4.0, -3.0),
        PathCmd::LineTo(-2.0, 0.0),
        PathCmd::LineTo(-4.0, 3.0),
    ]
}

/// NPN BJT.
fn npn_bjt_path() -> Vec<PathCmd> {
    vec![
        // Base lead
        PathCmd::MoveTo(-15.0, 0.0),
        PathCmd::LineTo(-5.0, 0.0),
        // Base bar
        PathCmd::MoveTo(-5.0, -8.0),
        PathCmd::LineTo(-5.0, 8.0),
        // Collector
        PathCmd::MoveTo(-5.0, -5.0),
        PathCmd::LineTo(5.0, -10.0),
        PathCmd::LineTo(5.0, -15.0),
        // Emitter (with arrow out)
        PathCmd::MoveTo(-5.0, 5.0),
        PathCmd::LineTo(5.0, 10.0),
        PathCmd::LineTo(5.0, 15.0),
        // Arrow on emitter
        PathCmd::MoveTo(2.0, 7.0),
        PathCmd::LineTo(5.0, 10.0),
        PathCmd::LineTo(1.0, 10.0),
    ]
}

/// PNP BJT.
fn pnp_bjt_path() -> Vec<PathCmd> {
    vec![
        PathCmd::MoveTo(-15.0, 0.0),
        PathCmd::LineTo(-5.0, 0.0),
        PathCmd::MoveTo(-5.0, -8.0),
        PathCmd::LineTo(-5.0, 8.0),
        PathCmd::MoveTo(-5.0, -5.0),
        PathCmd::LineTo(5.0, -10.0),
        PathCmd::LineTo(5.0, -15.0),
        PathCmd::MoveTo(-5.0, 5.0),
        PathCmd::LineTo(5.0, 10.0),
        PathCmd::LineTo(5.0, 15.0),
        // Arrow on emitter (pointing in)
        PathCmd::MoveTo(-2.0, 7.0),
        PathCmd::LineTo(-5.0, 5.0),
        PathCmd::LineTo(-2.0, 3.0),
    ]
}

/// N-channel MOSFET.
fn nmos_path() -> Vec<PathCmd> {
    vec![
        // Gate
        PathCmd::MoveTo(-15.0, 0.0),
        PathCmd::LineTo(-6.0, 0.0),
        PathCmd::MoveTo(-6.0, -8.0),
        PathCmd::LineTo(-6.0, 8.0),
        // Channel (three segments)
        PathCmd::MoveTo(-3.0, -8.0),
        PathCmd::LineTo(-3.0, -3.0),
        PathCmd::MoveTo(-3.0, -1.0),
        PathCmd::LineTo(-3.0, 1.0),
        PathCmd::MoveTo(-3.0, 3.0),
        PathCmd::LineTo(-3.0, 8.0),
        // Drain
        PathCmd::MoveTo(-3.0, -6.0),
        PathCmd::LineTo(5.0, -6.0),
        PathCmd::LineTo(5.0, -15.0),
        // Source
        PathCmd::MoveTo(-3.0, 6.0),
        PathCmd::LineTo(5.0, 6.0),
        PathCmd::LineTo(5.0, 15.0),
        // Body connection
        PathCmd::MoveTo(-3.0, 0.0),
        PathCmd::LineTo(5.0, 0.0),
        // Arrow (enhancement mode, pointing in)
        PathCmd::MoveTo(2.0, -3.0),
        PathCmd::LineTo(5.0, 0.0),
        PathCmd::LineTo(2.0, 3.0),
    ]
}

/// P-channel MOSFET.
fn pmos_path() -> Vec<PathCmd> {
    vec![
        PathCmd::MoveTo(-15.0, 0.0),
        PathCmd::LineTo(-6.0, 0.0),
        PathCmd::MoveTo(-6.0, -8.0),
        PathCmd::LineTo(-6.0, 8.0),
        PathCmd::MoveTo(-3.0, -8.0),
        PathCmd::LineTo(-3.0, -3.0),
        PathCmd::MoveTo(-3.0, -1.0),
        PathCmd::LineTo(-3.0, 1.0),
        PathCmd::MoveTo(-3.0, 3.0),
        PathCmd::LineTo(-3.0, 8.0),
        PathCmd::MoveTo(-3.0, -6.0),
        PathCmd::LineTo(5.0, -6.0),
        PathCmd::LineTo(5.0, -15.0),
        PathCmd::MoveTo(-3.0, 6.0),
        PathCmd::LineTo(5.0, 6.0),
        PathCmd::LineTo(5.0, 15.0),
        PathCmd::MoveTo(-3.0, 0.0),
        PathCmd::LineTo(5.0, 0.0),
        // Arrow pointing out
        PathCmd::MoveTo(1.0, -3.0),
        PathCmd::LineTo(-2.0, 0.0),
        PathCmd::LineTo(1.0, 3.0),
    ]
}

/// Op-amp: triangle.
fn opamp_path() -> Vec<PathCmd> {
    vec![
        // Triangle body
        PathCmd::MoveTo(-10.0, -12.0),
        PathCmd::LineTo(-10.0, 12.0),
        PathCmd::LineTo(12.0, 0.0),
        PathCmd::Close,
        // Non-inverting input (+)
        PathCmd::MoveTo(-15.0, -6.0),
        PathCmd::LineTo(-10.0, -6.0),
        PathCmd::MoveTo(-14.0, -8.0),
        PathCmd::LineTo(-14.0, -4.0),
        PathCmd::MoveTo(-16.0, -6.0),
        PathCmd::LineTo(-12.0, -6.0),
        // Inverting input (-)
        PathCmd::MoveTo(-15.0, 6.0),
        PathCmd::LineTo(-10.0, 6.0),
        PathCmd::MoveTo(-16.0, 6.0),
        PathCmd::LineTo(-12.0, 6.0),
        // Output
        PathCmd::MoveTo(12.0, 0.0),
        PathCmd::LineTo(18.0, 0.0),
    ]
}

/// Potentiometer: resistor with arrow wiper.
fn pot_path() -> Vec<PathCmd> {
    vec![
        // Resistor body
        PathCmd::MoveTo(-15.0, 0.0),
        PathCmd::LineTo(-10.0, 0.0),
        PathCmd::LineTo(-8.0, -5.0),
        PathCmd::LineTo(-4.0, 5.0),
        PathCmd::LineTo(0.0, -5.0),
        PathCmd::LineTo(4.0, 5.0),
        PathCmd::LineTo(8.0, -5.0),
        PathCmd::LineTo(10.0, 0.0),
        PathCmd::LineTo(15.0, 0.0),
        // Wiper arrow
        PathCmd::MoveTo(0.0, -10.0),
        PathCmd::LineTo(0.0, -5.0),
        PathCmd::MoveTo(-2.0, -7.0),
        PathCmd::LineTo(0.0, -5.0),
        PathCmd::LineTo(2.0, -7.0),
    ]
}

/// Transformer: two coupled inductors.
fn transformer_path() -> Vec<PathCmd> {
    vec![
        // Primary (left)
        PathCmd::MoveTo(-8.0, -15.0),
        PathCmd::ArcTo(-8.0, -10.0, 3.0, std::f32::consts::FRAC_PI_2, -std::f32::consts::FRAC_PI_2),
        PathCmd::ArcTo(-8.0, -5.0, 3.0, std::f32::consts::FRAC_PI_2, -std::f32::consts::FRAC_PI_2),
        PathCmd::ArcTo(-8.0, 0.0, 3.0, std::f32::consts::FRAC_PI_2, -std::f32::consts::FRAC_PI_2),
        PathCmd::ArcTo(-8.0, 5.0, 3.0, std::f32::consts::FRAC_PI_2, -std::f32::consts::FRAC_PI_2),
        PathCmd::LineTo(-8.0, 15.0),
        // Core lines
        PathCmd::MoveTo(-2.0, -15.0),
        PathCmd::LineTo(-2.0, 15.0),
        PathCmd::MoveTo(2.0, -15.0),
        PathCmd::LineTo(2.0, 15.0),
        // Secondary (right)
        PathCmd::MoveTo(8.0, -15.0),
        PathCmd::ArcTo(8.0, -10.0, 3.0, -std::f32::consts::FRAC_PI_2, std::f32::consts::FRAC_PI_2),
        PathCmd::ArcTo(8.0, -5.0, 3.0, -std::f32::consts::FRAC_PI_2, std::f32::consts::FRAC_PI_2),
        PathCmd::ArcTo(8.0, 0.0, 3.0, -std::f32::consts::FRAC_PI_2, std::f32::consts::FRAC_PI_2),
        PathCmd::ArcTo(8.0, 5.0, 3.0, -std::f32::consts::FRAC_PI_2, std::f32::consts::FRAC_PI_2),
        PathCmd::LineTo(8.0, 15.0),
    ]
}

/// Photocoupler (LED + LDR in box).
fn photocoupler_path() -> Vec<PathCmd> {
    vec![
        // Box
        PathCmd::MoveTo(-10.0, -10.0),
        PathCmd::LineTo(10.0, -10.0),
        PathCmd::LineTo(10.0, 10.0),
        PathCmd::LineTo(-10.0, 10.0),
        PathCmd::Close,
        // LED symbol (left half)
        PathCmd::MoveTo(-8.0, -4.0),
        PathCmd::LineTo(-2.0, 0.0),
        PathCmd::LineTo(-8.0, 4.0),
        PathCmd::Close,
        // Light arrows
        PathCmd::MoveTo(-1.0, -3.0),
        PathCmd::LineTo(2.0, -5.0),
        PathCmd::MoveTo(-1.0, 0.0),
        PathCmd::LineTo(2.0, -2.0),
        // LDR symbol (right half) — zigzag
        PathCmd::MoveTo(4.0, -4.0),
        PathCmd::LineTo(5.0, -2.0),
        PathCmd::LineTo(7.0, -3.0),
        PathCmd::LineTo(5.0, 0.0),
        PathCmd::LineTo(7.0, 1.0),
        PathCmd::LineTo(5.0, 2.0),
        PathCmd::LineTo(4.0, 4.0),
    ]
}

/// Generic IC chip (rectangle with pins).
fn ic_chip_path() -> Vec<PathCmd> {
    vec![
        PathCmd::MoveTo(-10.0, -12.0),
        PathCmd::LineTo(10.0, -12.0),
        PathCmd::LineTo(10.0, 12.0),
        PathCmd::LineTo(-10.0, 12.0),
        PathCmd::Close,
        // Pin 1 dot
        PathCmd::ArcTo(-6.0, -8.0, 1.5, 0.0, std::f32::consts::TAU),
    ]
}

/// Ground symbol: three horizontal lines.
fn ground_path() -> Vec<PathCmd> {
    vec![
        PathCmd::MoveTo(0.0, -5.0),
        PathCmd::LineTo(0.0, 0.0),
        PathCmd::MoveTo(-8.0, 0.0),
        PathCmd::LineTo(8.0, 0.0),
        PathCmd::MoveTo(-5.0, 3.0),
        PathCmd::LineTo(5.0, 3.0),
        PathCmd::MoveTo(-2.0, 6.0),
        PathCmd::LineTo(2.0, 6.0),
    ]
}

/// Supply symbol: upward arrow.
fn supply_path() -> Vec<PathCmd> {
    vec![
        PathCmd::MoveTo(0.0, 5.0),
        PathCmd::LineTo(0.0, -5.0),
        PathCmd::MoveTo(-3.0, -2.0),
        PathCmd::LineTo(0.0, -5.0),
        PathCmd::LineTo(3.0, -2.0),
    ]
}

/// Speaker symbol.
fn speaker_path() -> Vec<PathCmd> {
    vec![
        // Body
        PathCmd::MoveTo(-5.0, -5.0),
        PathCmd::LineTo(-5.0, 5.0),
        PathCmd::LineTo(0.0, 5.0),
        PathCmd::LineTo(8.0, 10.0),
        PathCmd::LineTo(8.0, -10.0),
        PathCmd::LineTo(0.0, -5.0),
        PathCmd::Close,
        // Lead
        PathCmd::MoveTo(-15.0, 0.0),
        PathCmd::LineTo(-5.0, 0.0),
    ]
}

/// Input arrow.
fn input_path() -> Vec<PathCmd> {
    vec![
        PathCmd::MoveTo(-10.0, 0.0),
        PathCmd::LineTo(5.0, 0.0),
        PathCmd::MoveTo(2.0, -4.0),
        PathCmd::LineTo(5.0, 0.0),
        PathCmd::LineTo(2.0, 4.0),
    ]
}

/// Output arrow (larger).
fn output_path() -> Vec<PathCmd> {
    vec![
        PathCmd::MoveTo(-10.0, 0.0),
        PathCmd::LineTo(8.0, 0.0),
        PathCmd::MoveTo(4.0, -5.0),
        PathCmd::LineTo(8.0, 0.0),
        PathCmd::LineTo(4.0, 5.0),
    ]
}

/// LFO symbol (sine wave in box).
fn lfo_path() -> Vec<PathCmd> {
    vec![
        PathCmd::MoveTo(-10.0, -8.0),
        PathCmd::LineTo(10.0, -8.0),
        PathCmd::LineTo(10.0, 8.0),
        PathCmd::LineTo(-10.0, 8.0),
        PathCmd::Close,
        // Sine wave
        PathCmd::MoveTo(-8.0, 0.0),
        PathCmd::ArcTo(-4.0, 0.0, 4.0, std::f32::consts::PI, 0.0),
        PathCmd::ArcTo(4.0, 0.0, 4.0, 0.0, std::f32::consts::PI),
    ]
}

/// Neon bulb.
fn neon_path() -> Vec<PathCmd> {
    vec![
        // Circle (envelope)
        PathCmd::ArcTo(0.0, 0.0, 8.0, 0.0, std::f32::consts::TAU),
        // Two dots (electrodes)
        PathCmd::ArcTo(-3.0, 0.0, 1.5, 0.0, std::f32::consts::TAU),
        PathCmd::ArcTo(3.0, 0.0, 1.5, 0.0, std::f32::consts::TAU),
        // Leads
        PathCmd::MoveTo(-8.0, 0.0),
        PathCmd::LineTo(-15.0, 0.0),
        PathCmd::MoveTo(8.0, 0.0),
        PathCmd::LineTo(15.0, 0.0),
    ]
}

/// Switch symbol.
fn switch_path() -> Vec<PathCmd> {
    vec![
        // Two terminals
        PathCmd::ArcTo(-8.0, 0.0, 2.0, 0.0, std::f32::consts::TAU),
        PathCmd::ArcTo(8.0, 0.0, 2.0, 0.0, std::f32::consts::TAU),
        // Switch arm
        PathCmd::MoveTo(-6.0, 0.0),
        PathCmd::LineTo(6.0, -5.0),
        // Leads
        PathCmd::MoveTo(-15.0, 0.0),
        PathCmd::LineTo(-8.0, 0.0),
        PathCmd::MoveTo(8.0, 0.0),
        PathCmd::LineTo(15.0, 0.0),
    ]
}
