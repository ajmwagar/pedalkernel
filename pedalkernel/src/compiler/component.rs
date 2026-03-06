//! Component trait: compile-time behavior for `ComponentKind`.
//!
//! Centralizes passivity, MNA stamping, pin configs, WDF leaf creation,
//! validation, and KiCad export so that callers in graph.rs, validate.rs,
//! classify.rs, compile.rs, build.rs, and kicad.rs can delegate instead
//! of duplicating match blocks.

use std::collections::HashMap;

use crate::dsl::*;
use crate::elements::{JfetVariableResistor, Photocoupler, PhotocouplerModel};
use crate::tree::MnaSystem;

use super::classify::NonlinearKind;
use super::dyn_node::DynNode;
use super::graph::NodeId;
use super::validate::Severity;

/// Resistance used to model open circuits (infinite R, inactive fork paths).
pub(crate) const OPEN_CIRCUIT_R: f64 = 1_000_000.0;

// ═══════════════════════════════════════════════════════════════════════════
// Types
// ═══════════════════════════════════════════════════════════════════════════

/// Result of stamping a component into an MNA system.
pub(super) enum StampResult {
    /// Stamped conductance directly into G matrix (resistors, tempcos, switched resistors).
    Stamped,
    /// Produces a reactive WDF port (capacitor, inductor, switched cap/inductor).
    Reactive {
        dyn_node: DynNode,
        rp: f64,
    },
    /// Produces a pot entry for dynamic recomputation.
    Pot {
        dyn_node: DynNode,
        initial_conductance: f64,
    },
    /// Not stampable by the trait (transformer needs caller context, or non-passive).
    Skip,
}

/// Pin configuration for validation and graph construction.
pub(super) struct PinConfig {
    /// Valid pin names for this component type.
    pub valid_pins: &'static [&'static str],
    /// Pin aliases: (short, long) pairs that should be unioned in the graph.
    pub aliases: &'static [(&'static str, &'static str)],
}

/// Classification of a circuit edge by electrical behavior.
///
/// The planner uses edge kinds to group components into stages:
/// - Linear + Reactive = passive WDF tree
/// - Nonlinear seeds NR solver stages
/// - Vccs needs MNA off-diagonal stamps
/// - Behavioral breaks the graph (separate stages on each side)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum EdgeKind {
    /// Purely resistive: R, pot sub-R, tempco, switched-R, photocoupler LDR, JFET Vr.
    Linear,
    /// Has state, must be a WDF port: C, L, switched-C, switched-L.
    Reactive,
    /// Needs Newton-Raphson solver: diode, BJT, JFET amplifier, triode, MOSFET, OTA.
    Nonlinear,
    /// Voltage-controlled current source: OTA in linear mode (future).
    Vccs,
    /// Handled outside WDF/MNA: BBD, delay line.
    Behavioral,
}

/// A single edge declared by a component.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct ComponentEdge {
    pub pin_a: &'static str,
    pub pin_b: &'static str,
    pub kind: EdgeKind,
    /// For multi-port NL grouping (e.g., triode grid+plate ports).
    pub port_group: Option<usize>,
}

// ── Control & modulation declarations ─────────────────────────────────────

/// A controllable parameter declared by a component.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct ControlParam {
    /// Parameter name, must match `ControlDef.property` from the DSL.
    pub name: &'static str,
    /// What kind of control target this maps to.
    pub kind: ControlParamKind,
}

/// Classification of a component's controllable parameter.
#[derive(Debug, Clone, PartialEq)]
pub(super) enum ControlParamKind {
    /// Potentiometer position (0–1) — resolved to PotInStage/PotInMultiNlStage/etc.
    PotPosition,
    /// LFO rate (0–1 normalized).
    LfoRate,
    /// LFO depth/amplitude (0–1 normalized).
    LfoDepth,
    /// BBD clock rate (0–1 → delay time).
    BbdClockRate,
    /// BBD feedback amount (0–1).
    BbdFeedback,
    /// Delay line time (0–1 normalized).
    DelayTime,
    /// Delay line feedback (0–1).
    DelayFeedback,
    /// Switch position selector.
    SwitchPosition { num_positions: usize },
    /// Fire a single-sample impulse (drum trigger).
    Trigger,
}

/// Modulation sink: how a component receives LFO/envelope control signals.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct ModulationSink {
    /// Which kind of modulation target this maps to.
    pub target_kind: ModulationSinkKind,
    /// Center/offset voltage for the modulation.
    pub bias: f64,
    /// Modulation amplitude (half-swing from bias).
    pub range: f64,
}

/// Classification of a modulation sink by target type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum ModulationSinkKind {
    JfetVgs,
    PhotocouplerLed,
    TriodeVgk,
    VariMuVgk,
    PentodeVg1k,
    MosfetVgs,
    OtaIabc,
    BbdClock,
    DelaySpeed,
    DelayTime,
}

/// Context provided to `resolve_edges()` so a component can decide its role
/// based on how it's wired in the circuit.
pub(super) struct ResolveContext {
    /// True if the component's control/modulation pin is driven by an LFO,
    /// envelope follower, or DC bias (not in the audio signal path).
    pub control_pin_is_modulated: bool,
    /// True if the component's wiper pin (pots) appears in the netlist.
    pub wiper_connected: bool,
}

/// How a component participates in circuit graph construction.
pub(super) enum GraphRole {
    /// Creates one edge between two named pins.
    Edge {
        pin_a: &'static str,
        pin_b: &'static str,
    },
    /// Creates one edge AND counts as an active element.
    ActiveEdge {
        pin_a: &'static str,
        pin_b: &'static str,
    },
    /// Creates one edge with multi-terminal coupling (tubes).
    /// `edge_pin_a` → `edge_pin_b` is the WDF edge.
    /// `coupled_pins` lists ALL pins that should be coupled for BFS traversal.
    CoupledEdge {
        edge_pin_a: &'static str,
        edge_pin_b: &'static str,
        coupled_pins: &'static [&'static str],
    },
    /// Potentiometer: 2-terminal or 3-terminal depending on wiper usage in nets.
    Pot,
    /// Transformer: complex multi-winding handling (aliases, coupling, center taps).
    Transformer,
    /// No circuit edge, not active (LFO, EnvelopeFollower, BBD, control elements).
    Virtual,
    /// No circuit edge, but counts as active (OpAmp, VCO, VCF, etc.).
    ActiveIc,
}

// ═══════════════════════════════════════════════════════════════════════════
// Component trait
// ═══════════════════════════════════════════════════════════════════════════

/// Compile-time component behavior.
///
/// Requires `Debug` for trait-object `Debug` forwarding.
/// Provides `clone_box`, `as_any`, and `dyn_eq` for trait-object
/// `Clone`, `PartialEq`, and downcasting support.
pub(super) trait Component: std::fmt::Debug {
    // ── Trait Object Support ──────────────────────────────────────────────

    /// Clone into a boxed trait object.
    fn clone_box(&self) -> Box<dyn Component>;

    /// Downcast to `Any` for type-erased equality checks and downcasting.
    fn as_any(&self) -> &dyn std::any::Any;

    /// Dynamic equality: returns `true` if `other` is the same concrete type
    /// and compares equal.
    fn dyn_eq(&self, other: &dyn Component) -> bool;

    // ── Identity ──────────────────────────────────────────────────────────

    /// Human-readable type name (e.g. "resistor", "NPN transistor").
    fn type_tag(&self) -> &'static str;

    // ── Classification ────────────────────────────────────────────────────

    /// Whether this component is passive (collectible by BFS for stage building).
    fn is_passive(&self) -> bool;

    /// Whether this component is a nonlinear element (needs NR solver).
    fn is_nonlinear(&self) -> bool { false }

    /// Whether this component is an active IC (counted toward `num_active`).
    fn is_active_ic(&self) -> bool { false }

    /// Whether this component is a control-only element (no circuit pins).
    fn is_control_only(&self) -> bool { false }

    /// Whether this component's edge impedance changes at runtime.
    /// Stages containing variable edges need scattering matrix recomputation.
    fn is_variable(&self) -> bool { false }

    // ── Pin Interface ─────────────────────────────────────────────────────

    /// Valid pins and aliases for this component type.
    fn pin_config(&self) -> PinConfig;

    /// Extra pins that are valid as modulation targets but may not be in `pin_config`.
    fn modulation_pins(&self) -> &'static [&'static str] { &[] }

    // ── Graph Building ────────────────────────────────────────────────────

    /// How this component participates in circuit graph construction.
    fn graph_role(&self) -> GraphRole;

    /// Declare this component's circuit edges with their electrical classification.
    ///
    /// Returns empty for Virtual/ActiveIc components (no WDF topology participation).
    /// The planner uses these edges to group components into stages by EdgeKind.
    fn edges(&self) -> Vec<ComponentEdge> { vec![] }

    /// Context-dependent resolution: if this component's behavior depends on
    /// how it's wired, return a new edge list reflecting the resolved role.
    ///
    /// Called after graph construction with neighbor connectivity info.
    /// Default: no resolution needed (return None to keep current edges).
    ///
    /// Examples:
    /// - JFET gate → LFO/pot → resolve drain-source as Linear (variable resistor)
    /// - OTA Iabc → envelope → resolve as Vccs (linear mode)
    fn resolve_edges(&self, _neighbors: &ResolveContext) -> Option<Vec<ComponentEdge>> {
        None
    }

    // ── MNA Stamping ──────────────────────────────────────────────────────

    /// Stamp into MNA system. Returns what was produced.
    ///
    /// `comp_id` is the component's string identifier (needed for `DynNode::Pot`).
    /// `n1`/`n2` are MNA node indices (`None` = ground).
    /// Transformers return `Skip` — they need coupled-edge context from the caller.
    fn stamp_mna(
        &self,
        comp_id: &str,
        n1: Option<usize>,
        n2: Option<usize>,
        mna: &mut MnaSystem,
        sample_rate: f64,
    ) -> StampResult;

    // ── WDF Leaf Creation ─────────────────────────────────────────────────

    /// Create a WDF leaf node for this component.
    /// Returns `None` for non-leaf components (diodes, virtual elements, etc.).
    fn make_leaf(&self, _comp_id: &str, _sample_rate: f64) -> Option<DynNode> { None }

    // ── Value Access ──────────────────────────────────────────────────────

    fn resistance(&self) -> Option<f64> { None }
    fn capacitance(&self) -> Option<f64> { None }
    fn inductance(&self) -> Option<f64> { None }

    // ── Validation ────────────────────────────────────────────────────────

    /// Check for suspicious or invalid component values.
    fn validate_values(&self, _comp_id: &str) -> Vec<(Severity, String)> { vec![] }

    // ── Classification (detailed) ────────────────────────────────────────

    /// Classify this component as a nonlinear element for the circuit solver.
    ///
    /// Returns `Some((kind, junction_nodes))` if this is a nonlinear element,
    /// where `junction_nodes` are the circuit nodes where passive elements connect.
    /// Returns `None` for passive, virtual, and active-IC components.
    ///
    /// - 1 junction node: diodes, JFETs, MOSFETs, zeners, OTAs
    /// - 2 junction nodes: BJTs (collector, emitter), triodes/pentodes (plate, cathode)
    fn classify_nonlinear(
        &self,
        _comp_id: &str,
        _node_a: NodeId,
        _node_b: NodeId,
        _gnd_node: NodeId,
        _node_names: &HashMap<String, NodeId>,
    ) -> Option<(NonlinearKind, Vec<NodeId>)> {
        None
    }

    // ── Control Declarations ────────────────────────────────────────────

    /// Declare controllable parameters for this component.
    ///
    /// The compiler matches `ControlDef.property` against each param's `name`
    /// to determine the control target without heuristic net scanning.
    fn controls(&self) -> Vec<ControlParam> { vec![] }

    /// If this component can be a modulation sink (LFO/envelope target),
    /// return the sink description for the given pin.
    ///
    /// Called during LFO/envelope binding to determine bias and range.
    fn modulation_sink(&self, _pin: &str) -> Option<ModulationSink> { None }

    // ── Hardware ──────────────────────────────────────────────────────────

    /// KiCad symbol library reference and designator prefix.
    fn footprint_ref(&self) -> (&'static str, &'static str);
}

// ── Trait-object blanket impls ────────────────────────────────────────────

impl Clone for Box<dyn Component> {
    fn clone(&self) -> Self {
        self.clone_box()
    }
}

impl PartialEq for Box<dyn Component> {
    fn eq(&self, other: &Self) -> bool {
        self.dyn_eq(other.as_ref())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Implementation on ComponentKind
// ═══════════════════════════════════════════════════════════════════════════

impl Component for ComponentKind {
    // ── Trait Object Support ──────────────────────────────────────────────

    fn clone_box(&self) -> Box<dyn Component> {
        Box::new(self.clone())
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn dyn_eq(&self, other: &dyn Component) -> bool {
        other
            .as_any()
            .downcast_ref::<ComponentKind>()
            .map_or(false, |o| self == o)
    }

    // ── Identity ──────────────────────────────────────────────────────────

    fn type_tag(&self) -> &'static str {
        match self {
            ComponentKind::Resistor(_) => "resistor",
            ComponentKind::Capacitor(_) => "capacitor",
            ComponentKind::Inductor(_) => "inductor",
            ComponentKind::DiodePair(_) => "diode pair",
            ComponentKind::Diode(_) => "diode",
            ComponentKind::Zener(_) => "zener diode",
            ComponentKind::Potentiometer(_, _) => "potentiometer",
            ComponentKind::Npn(_) => "NPN transistor",
            ComponentKind::Pnp(_) => "PNP transistor",
            ComponentKind::OpAmp(ot) if ot.is_ota() => "OTA",
            ComponentKind::OpAmp(_) => "op-amp",
            ComponentKind::NJfet(_) => "N-channel JFET",
            ComponentKind::PJfet(_) => "P-channel JFET",
            ComponentKind::Nmos(_) => "N-channel MOSFET",
            ComponentKind::Pmos(_) => "P-channel MOSFET",
            ComponentKind::Photocoupler(_) => "photocoupler",
            ComponentKind::Triode(_) => "triode",
            ComponentKind::VariMu(_) => "variable-mu triode",
            ComponentKind::Pentode(_) => "pentode",
            ComponentKind::Lfo(..) => "LFO",
            ComponentKind::EnvelopeFollower(..) => "envelope follower",
            ComponentKind::Bbd(_) => "BBD delay",
            ComponentKind::DelayLine(..) => "delay line",
            ComponentKind::Tap(..) => "tap",
            ComponentKind::Neon(_) => "neon bulb",
            ComponentKind::Vco(_) => "VCO",
            ComponentKind::Vcf(_) => "VCF",
            ComponentKind::Vca(_) => "VCA",
            ComponentKind::Comparator(_) => "comparator",
            ComponentKind::AnalogSwitch(_) => "analog switch",
            ComponentKind::MatchedNpn(_) => "matched NPN pair",
            ComponentKind::MatchedPnp(_) => "matched PNP pair",
            ComponentKind::Tempco(_, _) => "tempco resistor",
            ComponentKind::Transformer(_) => "transformer",
            ComponentKind::CapSwitched(_) => "switched capacitor",
            ComponentKind::InductorSwitched(_) => "switched inductor",
            ComponentKind::ResistorSwitched(_) => "switched resistor",
            ComponentKind::RotarySwitch(_) => "rotary switch",
            ComponentKind::Switch(_) => "switch",
            ComponentKind::TriggerInput => "trigger_input",
        }
    }

    // ── Classification ────────────────────────────────────────────────────

    fn is_passive(&self) -> bool {
        matches!(
            self,
            ComponentKind::Resistor(_)
                | ComponentKind::Capacitor(_)
                | ComponentKind::Inductor(_)
                | ComponentKind::Potentiometer(_, _)
                | ComponentKind::Tempco(_, _)
                | ComponentKind::ResistorSwitched(_)
                | ComponentKind::CapSwitched(_)
                | ComponentKind::InductorSwitched(_)
                | ComponentKind::Transformer(_)
        )
    }

    fn is_nonlinear(&self) -> bool {
        match self {
            ComponentKind::DiodePair(_)
            | ComponentKind::Diode(_)
            | ComponentKind::Zener(_)
            | ComponentKind::NJfet(_)
            | ComponentKind::PJfet(_)
            | ComponentKind::Npn(_)
            | ComponentKind::Pnp(_)
            | ComponentKind::Triode(_)
            | ComponentKind::Pentode(_)
            | ComponentKind::VariMu(_)
            | ComponentKind::Nmos(_)
            | ComponentKind::Pmos(_) => true,
            ComponentKind::OpAmp(ot) => ot.is_ota(),
            _ => false,
        }
    }

    fn is_active_ic(&self) -> bool {
        match self {
            ComponentKind::OpAmp(ot) => !ot.is_ota(),
            ComponentKind::Vco(_)
            | ComponentKind::Vcf(_)
            | ComponentKind::Vca(_)
            | ComponentKind::Comparator(_)
            | ComponentKind::AnalogSwitch(_)
            | ComponentKind::MatchedNpn(_)
            | ComponentKind::MatchedPnp(_) => true,
            _ => false,
        }
    }

    fn is_control_only(&self) -> bool {
        matches!(
            self,
            ComponentKind::RotarySwitch(_) | ComponentKind::Switch(_) | ComponentKind::TriggerInput
        )
    }

    fn is_variable(&self) -> bool {
        matches!(
            self,
            ComponentKind::Potentiometer(_, _)
                | ComponentKind::NJfet(_)
                | ComponentKind::PJfet(_)
                | ComponentKind::Photocoupler(_)
        )
    }

    fn resolve_edges(&self, ctx: &ResolveContext) -> Option<Vec<ComponentEdge>> {
        match self {
            // JFET with gate modulated → variable resistor (Linear)
            ComponentKind::NJfet(_) | ComponentKind::PJfet(_)
                if ctx.control_pin_is_modulated =>
            {
                Some(vec![ComponentEdge {
                    pin_a: "drain",
                    pin_b: "source",
                    kind: EdgeKind::Linear,
                    port_group: None,
                }])
            }
            // OTA with Iabc modulated → linear VCCS
            ComponentKind::OpAmp(ot) if ot.is_ota() && ctx.control_pin_is_modulated => {
                Some(vec![ComponentEdge {
                    pin_a: "pos",
                    pin_b: "neg",
                    kind: EdgeKind::Vccs,
                    port_group: None,
                }])
            }
            _ => None,
        }
    }

    // ── Pin Interface ─────────────────────────────────────────────────────

    fn pin_config(&self) -> PinConfig {
        match self {
            // 2-terminal passive elements
            ComponentKind::Resistor(_)
            | ComponentKind::Capacitor(_)
            | ComponentKind::Inductor(_)
            | ComponentKind::DiodePair(_)
            | ComponentKind::Diode(_)
            | ComponentKind::Zener(_)
            | ComponentKind::Neon(_)
            | ComponentKind::Tempco(_, _)
            | ComponentKind::CapSwitched(_)
            | ComponentKind::InductorSwitched(_)
            | ComponentKind::ResistorSwitched(_) => PinConfig {
                valid_pins: &["a", "b"],
                aliases: &[],
            },

            ComponentKind::Potentiometer(_, _) => PinConfig {
                valid_pins: &["a", "b", "w", "wiper", "position"],
                aliases: &[],
            },

            // BJT
            ComponentKind::Npn(_) | ComponentKind::Pnp(_) => PinConfig {
                valid_pins: &["base", "collector", "emitter"],
                aliases: &[],
            },
            ComponentKind::MatchedNpn(_) | ComponentKind::MatchedPnp(_) => PinConfig {
                valid_pins: &[
                    "base", "collector", "emitter",
                    "base1", "base2", "collector1", "collector2",
                    "emitter1", "emitter2",
                ],
                aliases: &[],
            },

            // Op-amp
            ComponentKind::OpAmp(_) => PinConfig {
                valid_pins: &["pos", "neg", "out", "output", "vp", "in", "input"],
                aliases: &[("in", "input"), ("out", "output")],
            },

            // JFET
            ComponentKind::NJfet(_) | ComponentKind::PJfet(_) => PinConfig {
                valid_pins: &["gate", "drain", "source", "vgs"],
                aliases: &[],
            },

            // MOSFET
            ComponentKind::Nmos(_) | ComponentKind::Pmos(_) => PinConfig {
                valid_pins: &["gate", "drain", "source", "vgs"],
                aliases: &[],
            },

            // Photocoupler
            ComponentKind::Photocoupler(_) => PinConfig {
                valid_pins: &["a", "b", "led"],
                aliases: &[],
            },

            // Tubes
            ComponentKind::Triode(_) | ComponentKind::VariMu(_) => PinConfig {
                valid_pins: &["plate", "cathode", "grid", "vgk"],
                aliases: &[],
            },
            ComponentKind::Pentode(_) => PinConfig {
                valid_pins: &["plate", "cathode", "g1", "g2", "grid", "screen", "vg1k"],
                aliases: &[],
            },

            // LFO
            ComponentKind::Lfo(..) => PinConfig {
                valid_pins: &["out", "output", "rate"],
                aliases: &[("out", "output")],
            },

            // Envelope follower
            ComponentKind::EnvelopeFollower(..) => PinConfig {
                valid_pins: &["out", "output", "in", "input"],
                aliases: &[("in", "input"), ("out", "output")],
            },

            // BBD
            ComponentKind::Bbd(_) => PinConfig {
                valid_pins: &["in", "input", "out", "output", "clock"],
                aliases: &[("in", "input"), ("out", "output")],
            },

            // Delay line
            ComponentKind::DelayLine(..) => PinConfig {
                valid_pins: &[
                    "input", "in", "output", "out",
                    "rate", "speed_mod", "delay_time", "feedback",
                ],
                aliases: &[("in", "input"), ("out", "output")],
            },

            // Tap
            ComponentKind::Tap(..) => PinConfig {
                valid_pins: &["output", "out"],
                aliases: &[("out", "output")],
            },

            // Transformer
            ComponentKind::Transformer(_) => PinConfig {
                valid_pins: &[
                    "a", "b", "c", "d", "e", "f",
                    "primary.a", "primary.b", "secondary.a", "secondary.b",
                    "tertiary.a", "tertiary.b",
                    "pri.a", "pri.b", "sec.a", "sec.b", "ter.a", "ter.b",
                    "pri_a", "pri_b", "sec_a", "sec_b", "ter_a", "ter_b",
                    "pri_ct", "pri.ct", "sec_ct", "sec.ct", "ct",
                    "primary.ct", "secondary.ct",
                ],
                aliases: &[],
            },

            // Synth ICs
            ComponentKind::Vco(_) => PinConfig {
                valid_pins: &["cv", "saw", "tri", "pulse", "pw", "sync", "out", "output"],
                aliases: &[("out", "output")],
            },
            ComponentKind::Vcf(_) => PinConfig {
                valid_pins: &["in", "input", "out", "output", "cv", "res"],
                aliases: &[("in", "input"), ("out", "output")],
            },
            ComponentKind::Vca(_) => PinConfig {
                valid_pins: &[
                    "in", "input", "out", "output", "cv",
                    "in1", "out1", "cv1", "in2", "out2", "cv2",
                    "in3", "out3", "cv3", "in4", "out4", "cv4",
                ],
                aliases: &[("in", "input"), ("out", "output")],
            },
            ComponentKind::Comparator(_) => PinConfig {
                valid_pins: &["pos", "neg", "out", "output"],
                aliases: &[("out", "output")],
            },
            ComponentKind::AnalogSwitch(_) => PinConfig {
                valid_pins: &[
                    "in1", "out1", "ctrl1", "in2", "out2", "ctrl2",
                    "in3", "out3", "ctrl3", "in4", "out4", "ctrl4",
                ],
                aliases: &[],
            },

            // Control-only elements (no circuit pins)
            ComponentKind::RotarySwitch(_) | ComponentKind::Switch(_) => PinConfig {
                valid_pins: &[],
                aliases: &[],
            },

            // Trigger input: single output pin (unions with `in` node)
            ComponentKind::TriggerInput => PinConfig {
                valid_pins: &["out"],
                aliases: &[],
            },
        }
    }

    fn modulation_pins(&self) -> &'static [&'static str] {
        match self {
            ComponentKind::OpAmp(OpAmpType::Ca3080) => &["iabc"],
            ComponentKind::NJfet(_) | ComponentKind::PJfet(_) => &["vgs", "gate"],
            ComponentKind::Nmos(_) | ComponentKind::Pmos(_) => &["vgs", "gate"],
            ComponentKind::Triode(_) | ComponentKind::VariMu(_) => &["vgk"],
            ComponentKind::Pentode(_) => &["vg1k"],
            ComponentKind::Photocoupler(_) => &["led"],
            ComponentKind::Bbd(_) => &["clock"],
            ComponentKind::DelayLine(..) => &["speed_mod", "delay_time"],
            _ => &[],
        }
    }

    // ── Graph Building ────────────────────────────────────────────────────

    fn graph_role(&self) -> GraphRole {
        match self {
            // 2-terminal elements with a,b pins
            ComponentKind::Resistor(_)
            | ComponentKind::Capacitor(_)
            | ComponentKind::Inductor(_)
            | ComponentKind::DiodePair(_)
            | ComponentKind::Diode(_)
            | ComponentKind::Zener(_)
            | ComponentKind::Photocoupler(_)
            | ComponentKind::Neon(_)
            | ComponentKind::Tempco(_, _)
            | ComponentKind::CapSwitched(_)
            | ComponentKind::InductorSwitched(_)
            | ComponentKind::ResistorSwitched(_) => GraphRole::Edge {
                pin_a: "a",
                pin_b: "b",
            },

            // JFETs and MOSFETs: drain-source edge
            ComponentKind::NJfet(_)
            | ComponentKind::PJfet(_)
            | ComponentKind::Nmos(_)
            | ComponentKind::Pmos(_) => GraphRole::Edge {
                pin_a: "drain",
                pin_b: "source",
            },

            // OTA: pos-neg edge (nonlinear)
            ComponentKind::OpAmp(ot) if ot.is_ota() => GraphRole::Edge {
                pin_a: "pos",
                pin_b: "neg",
            },

            // OpAmp (non-OTA): active IC, no edge
            ComponentKind::OpAmp(_) => GraphRole::ActiveIc,

            // BJT: collector-emitter edge + active
            ComponentKind::Npn(_) | ComponentKind::Pnp(_) => GraphRole::ActiveEdge {
                pin_a: "collector",
                pin_b: "emitter",
            },

            // Triode/VariMu: plate-cathode edge + grid/plate/cathode coupling
            ComponentKind::Triode(_) | ComponentKind::VariMu(_) => GraphRole::CoupledEdge {
                edge_pin_a: "plate",
                edge_pin_b: "cathode",
                coupled_pins: &["plate", "cathode", "grid"],
            },

            // Pentode: plate-cathode edge + 4-pin coupling
            ComponentKind::Pentode(_) => GraphRole::CoupledEdge {
                edge_pin_a: "plate",
                edge_pin_b: "cathode",
                coupled_pins: &["plate", "cathode", "grid", "screen"],
            },

            // Potentiometer: 2-terminal or 3-terminal depending on wiper usage
            ComponentKind::Potentiometer(_, _) => GraphRole::Pot,

            // Transformer: complex multi-winding handling
            ComponentKind::Transformer(_) => GraphRole::Transformer,

            // Virtual: no edge, not active
            ComponentKind::Lfo(..)
            | ComponentKind::EnvelopeFollower(..)
            | ComponentKind::Bbd(_)
            | ComponentKind::DelayLine(..)
            | ComponentKind::Tap(..)
            | ComponentKind::RotarySwitch(_)
            | ComponentKind::Switch(_)
            | ComponentKind::TriggerInput => GraphRole::Virtual,

            // Active ICs: no edge, count as active
            ComponentKind::Vco(_)
            | ComponentKind::Vcf(_)
            | ComponentKind::Vca(_)
            | ComponentKind::Comparator(_)
            | ComponentKind::AnalogSwitch(_)
            | ComponentKind::MatchedNpn(_)
            | ComponentKind::MatchedPnp(_) => GraphRole::ActiveIc,
        }
    }

    fn edges(&self) -> Vec<ComponentEdge> {
        match self {
            // ── Linear edges ─────────────────────────────────────────────
            ComponentKind::Resistor(_)
            | ComponentKind::Tempco(_, _)
            | ComponentKind::ResistorSwitched(_) => vec![ComponentEdge {
                pin_a: "a", pin_b: "b", kind: EdgeKind::Linear, port_group: None,
            }],

            // Photocoupler LDR acts as variable resistor (linear)
            ComponentKind::Photocoupler(_) => vec![ComponentEdge {
                pin_a: "a", pin_b: "b", kind: EdgeKind::Linear, port_group: None,
            }],

            // Pot: linear edge (2-terminal default; 3-term resolved in 2C-3)
            ComponentKind::Potentiometer(_, _) => vec![ComponentEdge {
                pin_a: "a", pin_b: "b", kind: EdgeKind::Linear, port_group: None,
            }],

            // ── Reactive edges ───────────────────────────────────────────
            ComponentKind::Capacitor(_)
            | ComponentKind::CapSwitched(_) => vec![ComponentEdge {
                pin_a: "a", pin_b: "b", kind: EdgeKind::Reactive, port_group: None,
            }],

            ComponentKind::Inductor(_)
            | ComponentKind::InductorSwitched(_) => vec![ComponentEdge {
                pin_a: "a", pin_b: "b", kind: EdgeKind::Reactive, port_group: None,
            }],

            // ── Nonlinear edges ──────────────────────────────────────────
            ComponentKind::Diode(_)
            | ComponentKind::DiodePair(_)
            | ComponentKind::Zener(_)
            | ComponentKind::Neon(_) => vec![ComponentEdge {
                pin_a: "a", pin_b: "b", kind: EdgeKind::Nonlinear, port_group: None,
            }],

            // BJT: collector-emitter is the NL edge
            ComponentKind::Npn(_) | ComponentKind::Pnp(_) => vec![ComponentEdge {
                pin_a: "collector", pin_b: "emitter", kind: EdgeKind::Nonlinear, port_group: None,
            }],

            // JFET: drain-source NL edge (default; resolved to Linear when used as Vr)
            ComponentKind::NJfet(_) | ComponentKind::PJfet(_) => vec![ComponentEdge {
                pin_a: "drain", pin_b: "source", kind: EdgeKind::Nonlinear, port_group: None,
            }],

            // MOSFET: drain-source NL edge
            ComponentKind::Nmos(_) | ComponentKind::Pmos(_) => vec![ComponentEdge {
                pin_a: "drain", pin_b: "source", kind: EdgeKind::Nonlinear, port_group: None,
            }],

            // OTA: pos-neg NL edge
            ComponentKind::OpAmp(ot) if ot.is_ota() => vec![ComponentEdge {
                pin_a: "pos", pin_b: "neg", kind: EdgeKind::Nonlinear, port_group: None,
            }],

            // Triode/VariMu: plate-cathode NL edge
            ComponentKind::Triode(_) | ComponentKind::VariMu(_) => vec![ComponentEdge {
                pin_a: "plate", pin_b: "cathode", kind: EdgeKind::Nonlinear, port_group: None,
            }],

            // Pentode: plate-cathode NL edge
            ComponentKind::Pentode(_) => vec![ComponentEdge {
                pin_a: "plate", pin_b: "cathode", kind: EdgeKind::Nonlinear, port_group: None,
            }],

            // ── Behavioral edges ─────────────────────────────────────────
            ComponentKind::Bbd(_) => vec![ComponentEdge {
                pin_a: "input", pin_b: "output", kind: EdgeKind::Behavioral, port_group: None,
            }],
            ComponentKind::DelayLine(..) => vec![ComponentEdge {
                pin_a: "input", pin_b: "output", kind: EdgeKind::Behavioral, port_group: None,
            }],

            // ── Transformer: reactive (primary inductance) ───────────────
            ComponentKind::Transformer(_) => vec![ComponentEdge {
                pin_a: "a", pin_b: "b", kind: EdgeKind::Reactive, port_group: None,
            }],

            // ── No edges ─────────────────────────────────────────────────
            // Virtual (LFO, Envelope, Tap, switches) and ActiveIc (OpAmp, synth ICs)
            _ => vec![],
        }
    }

    // ── MNA Stamping ──────────────────────────────────────────────────────

    fn stamp_mna(
        &self,
        comp_id: &str,
        n1: Option<usize>,
        n2: Option<usize>,
        mna: &mut MnaSystem,
        sample_rate: f64,
    ) -> StampResult {
        match self {
            ComponentKind::Resistor(r) => {
                mna.stamp_resistor(n1, n2, *r);
                StampResult::Stamped
            }
            ComponentKind::Capacitor(cfg) => {
                let rp = 1.0 / (2.0 * sample_rate * cfg.value);
                StampResult::Reactive {
                    dyn_node: DynNode::Capacitor {
                        capacitance: cfg.value,
                        rp,
                        state: 0.0,
                        last_b: 0.0,
                    },
                    rp,
                }
            }
            ComponentKind::Inductor(l) => {
                let rp = 2.0 * sample_rate * *l;
                StampResult::Reactive {
                    dyn_node: DynNode::Inductor {
                        inductance: *l,
                        rp,
                        state: 0.0,
                    },
                    rp,
                }
            }
            ComponentKind::Potentiometer(max_r, taper) => {
                let initial_pos = 0.5;
                let tapered_pos = taper.apply(initial_pos);
                let r = (tapered_pos * *max_r).max(1.0);
                mna.stamp_resistor(n1, n2, r);
                StampResult::Pot {
                    dyn_node: DynNode::Pot {
                        comp_id: comp_id.to_string(),
                        max_resistance: *max_r,
                        position: initial_pos,
                        taper: *taper,
                        rp: r,
                    },
                    initial_conductance: 1.0 / r,
                }
            }
            ComponentKind::Tempco(r, _ppm) => {
                mna.stamp_resistor(n1, n2, *r);
                StampResult::Stamped
            }
            ComponentKind::ResistorSwitched(values) => {
                if let Some(&r) = values.first() {
                    if r.is_finite() && r > 0.0 {
                        mna.stamp_resistor(n1, n2, r);
                    }
                }
                StampResult::Stamped
            }
            ComponentKind::CapSwitched(values) => {
                if let Some(&c) = values.first() {
                    if c.is_finite() && c > 0.0 {
                        let rp = 1.0 / (2.0 * sample_rate * c);
                        return StampResult::Reactive {
                            dyn_node: DynNode::Capacitor {
                                capacitance: c,
                                rp,
                                state: 0.0,
                                last_b: 0.0,
                            },
                            rp,
                        };
                    }
                }
                StampResult::Skip
            }
            ComponentKind::InductorSwitched(values) => {
                if let Some(&l) = values.first() {
                    if l.is_finite() && l > 0.0 {
                        let rp = 2.0 * sample_rate * l;
                        return StampResult::Reactive {
                            dyn_node: DynNode::Inductor {
                                inductance: l,
                                rp,
                                state: 0.0,
                            },
                            rp,
                        };
                    }
                }
                StampResult::Skip
            }
            // Transformer needs coupled_edge_indices context — handled by caller.
            ComponentKind::Transformer(_) => StampResult::Skip,
            // Non-passive components.
            _ => StampResult::Skip,
        }
    }

    // ── WDF Leaf Creation ─────────────────────────────────────────────────

    fn make_leaf(&self, comp_id: &str, sample_rate: f64) -> Option<DynNode> {
        match self {
            ComponentKind::Resistor(r) if r.is_infinite() => {
                Some(DynNode::Resistor { rp: OPEN_CIRCUIT_R })
            }
            ComponentKind::Resistor(r) => Some(DynNode::Resistor { rp: *r }),
            ComponentKind::Capacitor(cfg) => {
                if cfg.leakage.is_some() || cfg.da.is_some() {
                    let rp = 1.0 / (2.0 * sample_rate * cfg.value);
                    let leakage_decay = if let Some(r_leak) = cfg.leakage {
                        let tau = r_leak * cfg.value;
                        let dt = 1.0 / sample_rate;
                        (-dt / tau).exp()
                    } else {
                        1.0
                    };
                    const DA_TIME_CONSTANT: f64 = 0.5;
                    let da_rate = if cfg.da.is_some() {
                        1.0 / (sample_rate * DA_TIME_CONSTANT)
                    } else {
                        0.0
                    };
                    Some(DynNode::LeakyCapacitor {
                        capacitance: cfg.value,
                        rp,
                        state: 0.0,
                        leakage_decay,
                        da_coef: cfg.da,
                        da_state: 0.0,
                        da_rate,
                    })
                } else {
                    Some(DynNode::Capacitor {
                        capacitance: cfg.value,
                        rp: 1.0 / (2.0 * sample_rate * cfg.value),
                        state: 0.0,
                        last_b: 0.0,
                    })
                }
            }
            ComponentKind::Inductor(l) => Some(DynNode::Inductor {
                inductance: *l,
                rp: 2.0 * sample_rate * *l,
                state: 0.0,
            }),
            ComponentKind::Potentiometer(max_r, taper) => {
                let initial_pos = 0.5;
                let tapered_pos = taper.apply(initial_pos);
                Some(DynNode::Pot {
                    comp_id: comp_id.to_string(),
                    max_resistance: *max_r,
                    position: initial_pos,
                    taper: *taper,
                    rp: (tapered_pos * *max_r).max(1.0),
                })
            }
            ComponentKind::Photocoupler(pt) => {
                let model = match pt {
                    PhotocouplerType::Vtl5c3 => PhotocouplerModel::vtl5c3(),
                    PhotocouplerType::Vtl5c1 => PhotocouplerModel::vtl5c1(),
                    PhotocouplerType::Nsl32 => PhotocouplerModel::nsl32(),
                    PhotocouplerType::T4b => PhotocouplerModel::t4b(),
                };
                Some(DynNode::Photocoupler {
                    comp_id: comp_id.to_string(),
                    inner: Photocoupler::new(model, sample_rate),
                    prev_resistance: 0.0,
                })
            }
            ComponentKind::NJfet(name) | ComponentKind::PJfet(name) => {
                let model = crate::elements::JfetModel::by_name(name);
                Some(DynNode::JfetVr {
                    comp_id: comp_id.to_string(),
                    inner: JfetVariableResistor::new(model),
                    prev_rds: 0.0,
                })
            }
            ComponentKind::Tempco(r, _ppm) => Some(DynNode::Resistor { rp: *r }),
            ComponentKind::Transformer(cfg) => {
                let l_primary = cfg.primary_inductance;
                let n = cfg.turns_ratio;
                let l_secondary = l_primary / (n * n);
                let secondary = Box::new(DynNode::Inductor {
                    inductance: l_secondary,
                    rp: 2.0 * sample_rate * l_secondary,
                    state: 0.0,
                });
                if let Some(n_tertiary) = cfg.tertiary_turns_ratio {
                    let l_tertiary = l_primary / (n_tertiary * n_tertiary);
                    let tertiary = Box::new(DynNode::Inductor {
                        inductance: l_tertiary,
                        rp: 2.0 * sample_rate * l_tertiary,
                        state: 0.0,
                    });
                    let r_sec = secondary.port_resistance();
                    let r_ter = tertiary.port_resistance();
                    let adaptor = crate::tree::RTypeAdaptor::three_winding_transformer(
                        n, n_tertiary, r_sec, r_ter,
                    );
                    Some(DynNode::RType {
                        adaptor,
                        children: vec![secondary, tertiary],
                    })
                } else {
                    let rp_sec = secondary.port_resistance();
                    let rp_prim = n * n * rp_sec;
                    Some(DynNode::Transformer {
                        secondary,
                        turns_ratio: n,
                        rp: rp_prim,
                        b_sec: 0.0,
                    })
                }
            }
            // Non-leaf components (diodes, active ICs, virtual).
            _ => None,
        }
    }

    // ── Value Access ──────────────────────────────────────────────────────

    fn resistance(&self) -> Option<f64> {
        match self {
            ComponentKind::Resistor(r) => Some(*r),
            ComponentKind::Potentiometer(r, _) => Some(*r),
            ComponentKind::Tempco(r, _) => Some(*r),
            ComponentKind::ResistorSwitched(values) => values.first().copied(),
            _ => None,
        }
    }

    fn capacitance(&self) -> Option<f64> {
        match self {
            ComponentKind::Capacitor(cfg) => Some(cfg.value),
            ComponentKind::CapSwitched(values) => values.first().copied(),
            _ => None,
        }
    }

    fn inductance(&self) -> Option<f64> {
        match self {
            ComponentKind::Inductor(l) => Some(*l),
            ComponentKind::InductorSwitched(values) => values.first().copied(),
            _ => None,
        }
    }

    // ── Validation ────────────────────────────────────────────────────────

    fn validate_values(&self, comp_id: &str) -> Vec<(Severity, String)> {
        let mut w = Vec::new();
        match self {
            ComponentKind::Resistor(r) => {
                if *r <= 0.0 {
                    w.push((
                        Severity::Error,
                        format!("Resistor '{}' has non-positive value {:.2} \u{2126}", comp_id, r),
                    ));
                } else if *r < 1.0 {
                    w.push((
                        Severity::Info,
                        format!(
                            "Resistor '{}' is {:.2} \u{2126} \u{2014} extremely low, did you mean {}k?",
                            comp_id, r, r
                        ),
                    ));
                } else if *r > 100e6 {
                    w.push((
                        Severity::Info,
                        format!(
                            "Resistor '{}' is {:.1} M\u{2126} \u{2014} unusually high",
                            comp_id,
                            r / 1e6
                        ),
                    ));
                }
            }
            ComponentKind::Capacitor(cfg) => {
                if cfg.value <= 0.0 {
                    w.push((
                        Severity::Error,
                        format!(
                            "Capacitor '{}' has non-positive value {:.2e} F",
                            comp_id, cfg.value
                        ),
                    ));
                } else if cfg.value > 1.0 {
                    w.push((
                        Severity::Warning,
                        format!(
                            "Capacitor '{}' is {:.2} F \u{2014} over 1 farad, did you forget a unit suffix (n, u, p)?",
                            comp_id, cfg.value
                        ),
                    ));
                }
                if let Some(leak) = cfg.leakage {
                    if leak <= 0.0 {
                        w.push((
                            Severity::Error,
                            format!(
                                "Capacitor '{}' has non-positive leakage resistance {:.2} \u{2126}",
                                comp_id, leak
                            ),
                        ));
                    }
                }
                if let Some(da) = cfg.da {
                    if !(0.0..=1.0).contains(&da) {
                        w.push((
                            Severity::Warning,
                            format!(
                                "Capacitor '{}' has dielectric absorption {:.3} \u{2014} should be 0.0 to 1.0",
                                comp_id, da
                            ),
                        ));
                    }
                }
            }
            ComponentKind::Inductor(l) => {
                if *l <= 0.0 {
                    w.push((
                        Severity::Error,
                        format!(
                            "Inductor '{}' has non-positive value {:.2e} H",
                            comp_id, l
                        ),
                    ));
                }
            }
            ComponentKind::Potentiometer(r, _) => {
                if *r <= 0.0 {
                    w.push((
                        Severity::Error,
                        format!(
                            "Potentiometer '{}' has non-positive max resistance {:.2} \u{2126}",
                            comp_id, r
                        ),
                    ));
                }
            }
            ComponentKind::Zener(v) => {
                if *v <= 0.0 {
                    w.push((
                        Severity::Error,
                        format!(
                            "Zener '{}' has non-positive breakdown voltage {:.2} V",
                            comp_id, v
                        ),
                    ));
                }
            }
            ComponentKind::Lfo(_, timing_r, timing_c) => {
                if *timing_r <= 0.0 || *timing_c <= 0.0 {
                    w.push((
                        Severity::Error,
                        format!(
                            "LFO '{}' has non-positive timing values (R={:.2}, C={:.2e})",
                            comp_id, timing_r, timing_c
                        ),
                    ));
                }
                let freq = 1.0 / (2.0 * std::f64::consts::PI * timing_r * timing_c);
                if freq > 100.0 {
                    w.push((
                        Severity::Warning,
                        format!(
                            "LFO '{}' base frequency is {:.1} Hz \u{2014} unusually fast for modulation",
                            comp_id, freq
                        ),
                    ));
                }
            }
            ComponentKind::EnvelopeFollower(ar, ac, rr, rc, sr) => {
                if *ar <= 0.0 || *ac <= 0.0 || *rr <= 0.0 || *rc <= 0.0 || *sr <= 0.0 {
                    w.push((
                        Severity::Error,
                        format!(
                            "EnvelopeFollower '{}' has non-positive parameter value(s)",
                            comp_id
                        ),
                    ));
                }
            }
            ComponentKind::Switch(n) => {
                if *n < 2 {
                    w.push((
                        Severity::Warning,
                        format!(
                            "Switch '{}' has {} position(s) \u{2014} a switch needs at least 2",
                            comp_id, n
                        ),
                    ));
                }
            }
            _ => {}
        }
        w
    }

    // ── Classification (detailed) ────────────────────────────────────────

    fn classify_nonlinear(
        &self,
        comp_id: &str,
        node_a: NodeId,
        node_b: NodeId,
        gnd_node: NodeId,
        node_names: &HashMap<String, NodeId>,
    ) -> Option<(NonlinearKind, Vec<NodeId>)> {
        let a_is_gnd = node_a == gnd_node;
        let b_is_gnd = node_b == gnd_node;
        // Diodes: junction = non-ground node (fallback to node_a for feedback diodes)
        let diode_jn = if b_is_gnd {
            node_a
        } else if a_is_gnd {
            node_b
        } else {
            node_a
        };
        // Non-diode single-junction: junction = node_b unless at ground
        let other_jn = if b_is_gnd { node_a } else { node_b };

        match self {
            ComponentKind::DiodePair(dt) => {
                Some((NonlinearKind::DiodePair(*dt), vec![diode_jn]))
            }
            ComponentKind::Diode(dt) => {
                Some((NonlinearKind::SingleDiode(*dt), vec![diode_jn]))
            }
            ComponentKind::NJfet(name) => Some((
                NonlinearKind::Jfet {
                    model_name: name.clone(),
                    is_n_channel: true,
                },
                vec![other_jn],
            )),
            ComponentKind::PJfet(name) => Some((
                NonlinearKind::Jfet {
                    model_name: name.clone(),
                    is_n_channel: false,
                },
                vec![other_jn],
            )),
            ComponentKind::Npn(name) => {
                let base_node = node_names
                    .get(&format!("{}.base", comp_id))
                    .copied()
                    .unwrap_or(node_a);
                Some((
                    NonlinearKind::BjtNpn {
                        model_name: name.clone(),
                        base_node,
                    },
                    vec![node_a, node_b],
                ))
            }
            ComponentKind::Pnp(name) => {
                let base_node = node_names
                    .get(&format!("{}.base", comp_id))
                    .copied()
                    .unwrap_or(node_a);
                Some((
                    NonlinearKind::BjtPnp {
                        model_name: name.clone(),
                        base_node,
                    },
                    vec![node_a, node_b],
                ))
            }
            ComponentKind::Triode(name) => {
                let grid_node = node_names
                    .get(&format!("{}.grid", comp_id))
                    .copied();
                Some((
                    NonlinearKind::Triode {
                        model_name: name.clone(),
                        plate_node: node_a,
                        cathode_node: node_b,
                        grid_node,
                        parallel_count: 1,
                        is_vari_mu: false,
                    },
                    vec![node_a, node_b],
                ))
            }
            ComponentKind::VariMu(name) => {
                let grid_node = node_names
                    .get(&format!("{}.grid", comp_id))
                    .copied();
                Some((
                    NonlinearKind::Triode {
                        model_name: name.clone(),
                        plate_node: node_a,
                        cathode_node: node_b,
                        grid_node,
                        parallel_count: 1,
                        is_vari_mu: true,
                    },
                    vec![node_a, node_b],
                ))
            }
            ComponentKind::Pentode(name) => Some((
                NonlinearKind::Pentode {
                    model_name: name.clone(),
                },
                vec![node_a, node_b],
            )),
            ComponentKind::Nmos(mt) => Some((
                NonlinearKind::Mosfet {
                    mosfet_type: *mt,
                    is_n_channel: true,
                },
                vec![other_jn],
            )),
            ComponentKind::Pmos(mt) => Some((
                NonlinearKind::Mosfet {
                    mosfet_type: *mt,
                    is_n_channel: false,
                },
                vec![other_jn],
            )),
            ComponentKind::Zener(vz) => {
                Some((NonlinearKind::Zener { voltage: *vz }, vec![other_jn]))
            }
            ComponentKind::OpAmp(ot) if ot.is_ota() => {
                Some((NonlinearKind::Ota, vec![other_jn]))
            }
            _ => None,
        }
    }

    // ── Control Declarations ────────────────────────────────────────────

    fn controls(&self) -> Vec<ControlParam> {
        match self {
            ComponentKind::Potentiometer(_, _) => vec![
                ControlParam { name: "position", kind: ControlParamKind::PotPosition },
            ],
            ComponentKind::Lfo(..) => vec![
                ControlParam { name: "rate", kind: ControlParamKind::LfoRate },
                ControlParam { name: "depth", kind: ControlParamKind::LfoDepth },
            ],
            ComponentKind::Bbd(_) => vec![
                ControlParam { name: "clock", kind: ControlParamKind::BbdClockRate },
                ControlParam { name: "feedback", kind: ControlParamKind::BbdFeedback },
            ],
            ComponentKind::DelayLine(..) => vec![
                ControlParam { name: "delay_time", kind: ControlParamKind::DelayTime },
                ControlParam { name: "feedback", kind: ControlParamKind::DelayFeedback },
            ],
            ComponentKind::Switch(n) => vec![
                ControlParam { name: "position", kind: ControlParamKind::SwitchPosition { num_positions: *n } },
            ],
            ComponentKind::RotarySwitch(labels) => vec![
                ControlParam { name: "position", kind: ControlParamKind::SwitchPosition { num_positions: labels.len() } },
            ],
            ComponentKind::TriggerInput => vec![
                ControlParam { name: "trigger", kind: ControlParamKind::Trigger },
            ],
            _ => vec![],
        }
    }

    fn modulation_sink(&self, pin: &str) -> Option<ModulationSink> {
        match self {
            ComponentKind::NJfet(_) | ComponentKind::PJfet(_) => match pin {
                "vgs" | "gate" => Some(ModulationSink {
                    target_kind: ModulationSinkKind::JfetVgs,
                    bias: -0.45,
                    range: 0.25,
                }),
                _ => None,
            },
            ComponentKind::Photocoupler(_) => match pin {
                "led" => Some(ModulationSink {
                    target_kind: ModulationSinkKind::PhotocouplerLed,
                    bias: 0.5,
                    range: 0.5,
                }),
                _ => None,
            },
            ComponentKind::Triode(_) => match pin {
                "vgk" => Some(ModulationSink {
                    target_kind: ModulationSinkKind::TriodeVgk,
                    bias: -2.0,
                    range: 2.0,
                }),
                _ => None,
            },
            ComponentKind::VariMu(_) => match pin {
                "vgk" => Some(ModulationSink {
                    target_kind: ModulationSinkKind::VariMuVgk,
                    bias: -2.0,
                    range: 2.0,
                }),
                _ => None,
            },
            ComponentKind::Pentode(_) => match pin {
                "vg1k" => Some(ModulationSink {
                    target_kind: ModulationSinkKind::PentodeVg1k,
                    bias: -2.0,
                    range: 2.0,
                }),
                _ => None,
            },
            ComponentKind::Nmos(_) | ComponentKind::Pmos(_) => match pin {
                "vgs" | "gate" => Some(ModulationSink {
                    target_kind: ModulationSinkKind::MosfetVgs,
                    bias: 3.0,
                    range: 2.0,
                }),
                _ => None,
            },
            ComponentKind::OpAmp(ot) if ot.is_ota() => match pin {
                "iabc" => Some(ModulationSink {
                    target_kind: ModulationSinkKind::OtaIabc,
                    bias: 0.5,
                    range: 0.5,
                }),
                _ => None,
            },
            ComponentKind::Bbd(_) => match pin {
                "clock" => Some(ModulationSink {
                    target_kind: ModulationSinkKind::BbdClock,
                    bias: 0.15,
                    range: 0.10,
                }),
                _ => None,
            },
            ComponentKind::DelayLine(..) => match pin {
                "speed_mod" => Some(ModulationSink {
                    target_kind: ModulationSinkKind::DelaySpeed,
                    bias: 0.0,
                    range: 0.02,
                }),
                "delay_time" => Some(ModulationSink {
                    target_kind: ModulationSinkKind::DelayTime,
                    bias: 0.5,
                    range: 0.5,
                }),
                _ => None,
            },
            _ => None,
        }
    }

    // ── Hardware ──────────────────────────────────────────────────────────

    fn footprint_ref(&self) -> (&'static str, &'static str) {
        match self {
            ComponentKind::Resistor(_) => ("Device:R", "R"),
            ComponentKind::Capacitor(_) => ("Device:C", "C"),
            ComponentKind::Inductor(_) => ("Device:L", "L"),
            ComponentKind::DiodePair(_) => ("Device:D", "D"),
            ComponentKind::Diode(_) => ("Device:D", "D"),
            ComponentKind::Zener(_) => ("Device:D_Zener", "D"),
            ComponentKind::Potentiometer(_, _) => ("Device:R_Potentiometer", "RV"),
            ComponentKind::Npn(_) => ("Device:Q_NPN_BCE", "Q"),
            ComponentKind::Pnp(_) => ("Device:Q_PNP_BCE", "Q"),
            ComponentKind::OpAmp(ot) => {
                let lib = match ot {
                    OpAmpType::Generic => "Amplifier_Operational:TL072",
                    OpAmpType::Tl072 => "Amplifier_Operational:TL072",
                    OpAmpType::Tl082 => "Amplifier_Operational:TL082",
                    OpAmpType::Jrc4558 => "Amplifier_Operational:JRC4558",
                    OpAmpType::Rc4558 => "Amplifier_Operational:RC4558",
                    OpAmpType::Lm308 => "Amplifier_Operational:LM308",
                    OpAmpType::Lm741 => "Amplifier_Operational:LM741",
                    OpAmpType::Ne5532 => "Amplifier_Operational:NE5532",
                    OpAmpType::Ca3080 => "Amplifier_Operational:CA3080",
                    OpAmpType::Op07 => "Amplifier_Operational:OP07",
                };
                (lib, "U")
            }
            ComponentKind::NJfet(_) => ("Device:Q_NJFET_DGS", "J"),
            ComponentKind::PJfet(_) => ("Device:Q_PJFET_DGS", "J"),
            ComponentKind::Photocoupler(_) => ("Isolator:PC817", "OC"),
            ComponentKind::Lfo(..) => ("", "LFO"),
            ComponentKind::Triode(_)
            | ComponentKind::Pentode(_)
            | ComponentKind::VariMu(_) => ("Valve:Triode", "V"),
            ComponentKind::EnvelopeFollower(..) => ("", "ENV"),
            ComponentKind::Nmos(_) => ("Device:Q_NMOS_DGS", "Q"),
            ComponentKind::Pmos(_) => ("Device:Q_PMOS_DGS", "Q"),
            ComponentKind::Bbd(bt) => match bt {
                BbdType::Mn3207 => ("Analog_Delay:MN3207", "IC"),
                BbdType::Mn3007 => ("Analog_Delay:MN3007", "IC"),
                BbdType::Mn3005 => ("Analog_Delay:MN3005", "IC"),
            },
            ComponentKind::DelayLine(..) => ("", "DL"),
            ComponentKind::Tap(..) => ("", "TAP"),
            ComponentKind::Neon(nt) => match nt {
                NeonType::Ne2 => ("Device:Lamp_Neon", "NE"),
                NeonType::Ne51 => ("Device:Lamp_Neon", "NE"),
                NeonType::Ne83 => ("Device:Lamp_Neon", "NE"),
            },
            ComponentKind::Vco(vt) => match vt {
                VcoType::Cem3340 => ("Oscillator:CEM3340", "U"),
                VcoType::As3340 => ("Oscillator:AS3340", "U"),
                VcoType::V3340 => ("Oscillator:V3340", "U"),
            },
            ComponentKind::Vcf(vt) => match vt {
                VcfType::Cem3320 => ("Analog:CEM3320", "U"),
                VcfType::As3320 => ("Analog:AS3320", "U"),
            },
            ComponentKind::Vca(vt) => match vt {
                VcaType::Ssm2164 => ("Analog:SSM2164", "U"),
                VcaType::V2164 => ("Analog:V2164", "U"),
            },
            ComponentKind::Comparator(ct) => match ct {
                ComparatorType::Lm311 => ("Comparator:LM311", "U"),
                ComparatorType::Lm393 => ("Comparator:LM393", "U"),
            },
            ComponentKind::AnalogSwitch(st) => match st {
                AnalogSwitchType::Cd4066 => ("Analog_Switch:CD4066", "U"),
                AnalogSwitchType::Dg411 => ("Analog_Switch:DG411", "U"),
            },
            ComponentKind::MatchedNpn(mt) | ComponentKind::MatchedPnp(mt) => match mt {
                MatchedTransistorType::Ssm2210 => ("Transistor_BJT:SSM2210", "Q"),
                MatchedTransistorType::Ca3046 => ("Transistor_Array:CA3046", "Q"),
                MatchedTransistorType::Lm394 => ("Transistor_BJT:LM394", "Q"),
                MatchedTransistorType::That340 => ("Transistor_BJT:THAT340", "Q"),
            },
            ComponentKind::Tempco(_, _) => ("Device:R", "RT"),
            ComponentKind::Transformer(cfg) => {
                match (cfg.primary_type, cfg.secondary_type) {
                    (WindingType::PushPull, WindingType::Standard) => {
                        ("Transformer:Transformer_1P_1S", "T")
                    }
                    (WindingType::Standard, WindingType::CenterTap) => {
                        ("Transformer:Transformer_1P_1S_CT", "T")
                    }
                    (WindingType::PushPull, WindingType::CenterTap) => {
                        ("Transformer:Transformer_1P_CT_1S_CT", "T")
                    }
                    _ => ("Transformer:Transformer_1P_1S", "T"),
                }
            }
            ComponentKind::CapSwitched(_) => ("Device:C", "C"),
            ComponentKind::InductorSwitched(_) => ("Device:L", "L"),
            ComponentKind::RotarySwitch(ids) => {
                let positions = ids.len().max(2);
                match positions {
                    2..=4 => ("Switch:SW_Rotary4", "SW"),
                    5..=6 => ("Switch:SW_Rotary6", "SW"),
                    7..=8 => ("Switch:SW_Rotary8", "SW"),
                    _ => ("Switch:SW_Rotary12", "SW"),
                }
            }
            ComponentKind::ResistorSwitched(_) => ("Device:R", "R"),
            ComponentKind::Switch(positions) => match positions {
                2 => ("Switch:SW_SPDT", "SW"),
                3 => ("Switch:SW_Rotary4", "SW"),
                _ => ("Switch:SW_Rotary4", "SW"),
            },
            ComponentKind::TriggerInput => ("Connector:TestPoint", "TP"),
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn box_dyn_component_clone() {
        let r: Box<dyn Component> = Box::new(ComponentKind::Resistor(1000.0));
        let r2 = r.clone();
        assert_eq!(r2.type_tag(), "resistor");
        assert_eq!(r2.resistance(), Some(1000.0));
    }

    #[test]
    fn box_dyn_component_eq() {
        let r1: Box<dyn Component> = Box::new(ComponentKind::Resistor(1000.0));
        let r2: Box<dyn Component> = Box::new(ComponentKind::Resistor(1000.0));
        let r3: Box<dyn Component> = Box::new(ComponentKind::Resistor(2200.0));
        let c1: Box<dyn Component> = Box::new(ComponentKind::Capacitor(CapConfig::new(100e-9)));
        assert!(r1.dyn_eq(r2.as_ref()));
        assert!(!r1.dyn_eq(r3.as_ref()));
        assert!(!r1.dyn_eq(c1.as_ref()));
    }

    #[test]
    fn box_dyn_component_debug() {
        let r: Box<dyn Component> = Box::new(ComponentKind::Resistor(4700.0));
        let dbg = format!("{:?}", r);
        assert!(dbg.contains("4700"));
    }

    #[test]
    fn box_dyn_component_downcast() {
        let r: Box<dyn Component> = Box::new(ComponentKind::Resistor(1000.0));
        let any = r.as_any();
        assert!(any.downcast_ref::<ComponentKind>().is_some());
    }

    #[test]
    fn bridge_from_component_kind() {
        use crate::compiler::components;
        let kind = ComponentKind::Resistor(4700.0);
        let boxed: Box<dyn Component> = kind.into();
        assert_eq!(boxed.type_tag(), "resistor");
        assert_eq!(boxed.resistance(), Some(4700.0));
        assert!(boxed.is_passive());
        // Downcast to the concrete struct
        assert!(boxed.as_any().downcast_ref::<components::Resistor>().is_some());
    }

    #[test]
    fn bridge_preserves_behavior() {
        use crate::compiler::components;
        // Test that the bridged struct produces the same make_leaf as ComponentKind
        let kind = ComponentKind::Capacitor(CapConfig::new(100e-9));
        let boxed: Box<dyn Component> = kind.clone().into();
        let leaf_kind = kind.make_leaf("C1", 44100.0);
        let leaf_struct = boxed.make_leaf("C1", 44100.0);
        // Both should produce a Capacitor DynNode
        assert!(leaf_kind.is_some());
        assert!(leaf_struct.is_some());
        let _ = components::Resistor { value: 1.0 }; // suppress unused warning
    }

    #[test]
    fn edges_linear_resistor() {
        let r = ComponentKind::Resistor(1000.0);
        let edges = r.edges();
        assert_eq!(edges.len(), 1);
        assert_eq!(edges[0].kind, EdgeKind::Linear);
        assert_eq!(edges[0].pin_a, "a");
        assert_eq!(edges[0].pin_b, "b");
    }

    #[test]
    fn edges_reactive_capacitor() {
        let c = ComponentKind::Capacitor(CapConfig::new(100e-9));
        let edges = c.edges();
        assert_eq!(edges.len(), 1);
        assert_eq!(edges[0].kind, EdgeKind::Reactive);
    }

    #[test]
    fn edges_nonlinear_diode() {
        let d = ComponentKind::Diode(crate::dsl::DiodeType::Silicon);
        let edges = d.edges();
        assert_eq!(edges.len(), 1);
        assert_eq!(edges[0].kind, EdgeKind::Nonlinear);
    }

    #[test]
    fn edges_nonlinear_bjt() {
        let q = ComponentKind::Npn("2N3904".into());
        let edges = q.edges();
        assert_eq!(edges.len(), 1);
        assert_eq!(edges[0].kind, EdgeKind::Nonlinear);
        assert_eq!(edges[0].pin_a, "collector");
        assert_eq!(edges[0].pin_b, "emitter");
    }

    #[test]
    fn edges_nonlinear_triode() {
        let v = ComponentKind::Triode("12AX7".into());
        let edges = v.edges();
        assert_eq!(edges.len(), 1);
        assert_eq!(edges[0].kind, EdgeKind::Nonlinear);
        assert_eq!(edges[0].pin_a, "plate");
    }

    #[test]
    fn edges_behavioral_bbd() {
        let b = ComponentKind::Bbd(crate::dsl::BbdType::Mn3207);
        let edges = b.edges();
        assert_eq!(edges.len(), 1);
        assert_eq!(edges[0].kind, EdgeKind::Behavioral);
    }

    #[test]
    fn edges_virtual_lfo_empty() {
        let l = ComponentKind::Lfo(crate::dsl::LfoWaveformDsl::Triangle, 100_000.0, 1e-6);
        assert!(l.edges().is_empty());
    }

    #[test]
    fn edges_active_ic_opamp_empty() {
        let o = ComponentKind::OpAmp(crate::dsl::OpAmpType::Tl072);
        assert!(o.edges().is_empty());
    }

    #[test]
    fn edges_ota_nonlinear() {
        let ota = ComponentKind::OpAmp(crate::dsl::OpAmpType::Ca3080);
        let edges = ota.edges();
        assert_eq!(edges.len(), 1);
        assert_eq!(edges[0].kind, EdgeKind::Nonlinear);
        assert_eq!(edges[0].pin_a, "pos");
    }

    #[test]
    fn resolve_jfet_to_linear_when_modulated() {
        let j = ComponentKind::NJfet("2N5457".into());
        // Default: nonlinear
        assert_eq!(j.edges()[0].kind, EdgeKind::Nonlinear);
        // Modulated gate: resolves to linear (variable resistor)
        let ctx = ResolveContext { control_pin_is_modulated: true, wiper_connected: false };
        let resolved = j.resolve_edges(&ctx).unwrap();
        assert_eq!(resolved.len(), 1);
        assert_eq!(resolved[0].kind, EdgeKind::Linear);
        assert_eq!(resolved[0].pin_a, "drain");
    }

    #[test]
    fn resolve_jfet_no_change_without_modulation() {
        let j = ComponentKind::PJfet("2N5460".into());
        let ctx = ResolveContext { control_pin_is_modulated: false, wiper_connected: false };
        assert!(j.resolve_edges(&ctx).is_none());
    }

    #[test]
    fn resolve_ota_to_vccs_when_modulated() {
        let ota = ComponentKind::OpAmp(crate::dsl::OpAmpType::Ca3080);
        // Default: nonlinear
        assert_eq!(ota.edges()[0].kind, EdgeKind::Nonlinear);
        // Modulated Iabc: resolves to VCCS
        let ctx = ResolveContext { control_pin_is_modulated: true, wiper_connected: false };
        let resolved = ota.resolve_edges(&ctx).unwrap();
        assert_eq!(resolved.len(), 1);
        assert_eq!(resolved[0].kind, EdgeKind::Vccs);
    }

    #[test]
    fn resolve_regular_opamp_unchanged() {
        let op = ComponentKind::OpAmp(crate::dsl::OpAmpType::Tl072);
        let ctx = ResolveContext { control_pin_is_modulated: true, wiper_connected: false };
        // Regular opamps don't resolve (no modulation pins)
        assert!(op.resolve_edges(&ctx).is_none());
    }
}
