//! # Component Trait and Supporting Types
//!
//! This module defines [`Component`], the central trait that drives the entire
//! PedalKernel compilation pipeline. Every circuit element — from a simple
//! resistor to a 12AX7 vacuum tube — implements `Component` to declare its
//! electrical behavior, topology, and runtime characteristics.
//!
//! # Design Philosophy
//!
//! **Components declare; the compiler reacts.** The pipeline never pattern-matches
//! on component types. Instead, it queries trait methods to determine how each
//! component participates in the circuit:
//!
//! - **What edges does it create?** ([`Component::edges`]) — determines graph topology
//! - **Is it passive or nonlinear?** ([`Component::is_passive`], [`Component::is_nonlinear`]) — determines stage solver
//! - **What is its output impedance?** ([`Component::output_impedance`]) — determines stage boundaries
//! - **What non-ideal effects does it have?** ([`Component::nonideal_fx`]) — determines post-processing
//! - **How does signal flow through it?** ([`Component::signal_terminals`]) — determines feedback loops
//!
//! This inversion of control means adding a new component type requires implementing
//! one trait — no changes to the graph builder, SPQR decomposer, stage builder,
//! or any other pipeline stage.
//!
//! # How Components Drive the Pipeline
//!
//! ## Stage Splitting via Output Impedance
//!
//! [`OutputImpedance`] controls where the compiler can safely split a circuit into
//! independent processing stages. This is based on Harold Black's theorem:
//! a negative-feedback amplifier's output behaves as a voltage source (zero
//! output impedance), meaning downstream loads cannot affect upstream behavior.
//!
//! When a component returns [`OutputImpedance::VoltageSource`], the compiler knows
//! it can insert a stage boundary after that component's output. Op-amps in
//! negative feedback return `VoltageSource`; passive components return
//! [`OutputImpedance::Finite`].
//!
//! ## Feedback Detection via Signal Terminals
//!
//! [`SignalTerminals`] drives the feedback analysis pass. The compiler uses Tarjan's
//! strongly-connected-component algorithm to find feedback loops. Components with
//! [`SignalTerminals::Amplifier`] define the forward path direction; passive
//! components with [`SignalTerminals::Passive`] can carry signal in either direction
//! and may form the feedback path.
//!
//! ## Post-Processing via Non-Ideal Effects
//!
//! [`NonIdealFx`] allows components to declare physical imperfections from their
//! SPICE models and datasheets. An LM308 op-amp declares its 1 MHz GBW product
//! and 0.3 V/us slew rate; the stage builder translates these into a first-order
//! IIR lowpass and a slew-rate limiter applied after the ideal WDF/MNA solver.
//!
//! # Key Types
//!
//! - [`Component`] — the trait itself (see its documentation for method groups)
//! - [`EdgeKind`] — electrical classification of a circuit edge (Linear, Reactive, Nonlinear, Vcvs, etc.)
//! - [`ComponentEdge`] — a single edge declared by a component
//! - [`GraphRole`] — how a component participates in circuit graph construction
//! - [`OutputImpedance`] — voltage-source vs. finite impedance classification
//! - [`SignalTerminals`] — signal flow directionality for feedback analysis
//! - [`NonIdealFx`] — non-ideal behaviors (GBW, slew rate, rail saturation)
//! - [`StampResult`] — result of stamping a component into an MNA matrix
//! - [`StampContext`] — pin-to-MNA-index resolution for multi-terminal stamping
//! - [`ControlParam`] — controllable parameter declaration (pots, LFO rate, etc.)
//! - [`ModulationSink`] — how a component receives LFO/envelope modulation
//! - [`SolverMethod`] — preferred nonlinear solver (Newton-Raphson, Wright Omega, Ebers-Moll, Gummel-Poon)

use hashbrown::HashMap;

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
pub enum StampResult {
    /// Stamped conductance directly into G matrix (resistors, tempcos, switched resistors).
    Stamped,
    /// Produces a reactive WDF port (capacitor, inductor, switched cap/inductor).
    Reactive { dyn_node: DynNode, rp: f64 },
    /// Produces a pot entry for dynamic recomputation.
    Pot {
        dyn_node: DynNode,
        initial_conductance: f64,
    },
    /// Not stampable by the trait (transformer needs caller context, or non-passive).
    Skip,
}

/// Context for multi-terminal MNA stamping.
///
/// Provides pin-to-MNA-index resolution and pre-allocated vsource indices
/// so components can stamp themselves into the MNA without knowing graph details.
pub struct StampContext<'a> {
    /// Resolve a pin name (e.g., "pos", "neg", "out") to its MNA node index.
    /// Returns `None` for pins connected to ground or supply rails.
    pub pin_to_mna: &'a dyn Fn(&str) -> Option<usize>,
    /// Starting vsource index allocated for this component.
    pub vsrc_base: usize,
    /// Starting internal MNA node index allocated for this component.
    /// Components that declared `mna_internal_node_count() > 0` get indices
    /// `[internal_node_base .. internal_node_base + mna_internal_node_count())`.
    pub internal_node_base: usize,
    /// Sample rate in Hz.
    pub sample_rate: f64,
    /// Capacitor stamps for state-space integration. Components can push
    /// (node_pos, node_neg, capacitance) to add internal compensation caps.
    pub cap_stamps: Option<&'a mut Vec<(Option<usize>, Option<usize>, f64)>>,
}

/// Pin configuration for validation and graph construction.
pub struct PinConfig {
    /// Valid pin names for this component type.
    pub valid_pins: &'static [&'static str],
    /// Pin aliases: (short, long) pairs that should be unioned in the graph.
    pub aliases: &'static [(&'static str, &'static str)],
}

/// Inferred direction for a component pin (used by layout).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinDirection {
    /// Signal enters through this pin (e.g., triode grid).
    Input,
    /// Signal exits through this pin (e.g., triode plate).
    Output,
    /// Pin connects upward toward VCC (e.g., plate load destination).
    Up,
    /// Pin connects downward toward GND (e.g., cathode bias).
    Down,
    /// Direction determined by context (e.g., resistor terminals).
    Bidirectional,
}

/// Component-declared non-ideal behavior applied as post-processing.
///
/// Output impedance classification for stage-splitting decisions.
///
/// Determines where the compiler can safely split the circuit into
/// independent stages. Based on Harold Black's observation: a negative
/// feedback amplifier's output is a voltage source (zero impedance),
/// so downstream loads don't affect upstream behavior.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputImpedance {
    /// Zero output impedance — this node is a voltage source.
    /// Safe to split the circuit after this component's output pin.
    /// Op-amp outputs, unity-gain buffers, ideal voltage sources.
    VoltageSource,
    /// Finite or high output impedance — splitting here changes the circuit.
    /// Resistors, caps, inductors, BJT collectors, JFET drains, diodes.
    Finite,
}

pub use crate::nonideal_fx::NonIdealFx;

/// Signal flow classification for a component's pins.
///
/// Used by feedback analysis to determine which components are coupled
/// through feedback loops. The Component trait drives all decisions —
/// no hardcoded component-type matching in the pipeline.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SignalTerminals {
    /// No directionality — R, C, L, pot. Signal flows either way.
    Passive,
    /// Unidirectional two-port — diode (a→b). Signal has a preferred direction.
    TwoPort {
        input: &'static str,
        output: &'static str,
    },
    /// Amplifier with input, output, and optional control pin.
    /// Op-amp: input=neg, output=out, control=Some(pos).
    /// BJT: input=base, output=collector, control=None.
    /// Tube: input=grid, output=plate, control=None.
    Amplifier {
        input: &'static str,
        output: &'static str,
        control: Option<&'static str>,
    },
}

/// Classification of a circuit edge by electrical behavior.
///
/// The planner uses edge kinds to group components into stages:
/// - Linear + Reactive = passive WDF tree
/// - Nonlinear seeds NR solver stages
/// - Vccs needs MNA off-diagonal stamps
/// - Behavioral breaks the graph (separate stages on each side)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EdgeKind {
    /// Purely resistive: R, pot sub-R, tempco, switched-R, photocoupler LDR, JFET Vr.
    Linear,
    /// Has state, must be a WDF port: C, L, switched-C, switched-L.
    Reactive,
    /// Needs Newton-Raphson solver: diode, BJT, JFET amplifier, triode, MOSFET, OTA.
    Nonlinear,
    /// Voltage-controlled current source: OTA in linear mode (future).
    Vccs,
    /// Voltage-controlled voltage source: op-amp nullor (neg→out edge,
    /// pos resolved via stamp_mna_multi). Forces R-node in SPQR.
    Vcvs,
    /// Handled outside WDF/MNA: BBD, delay line.
    Behavioral,
}

/// A single edge declared by a component.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ComponentEdge {
    pub pin_a: &'static str,
    pub pin_b: &'static str,
    pub kind: EdgeKind,
    /// For multi-port NL grouping (e.g., triode grid+plate ports).
    pub port_group: Option<usize>,
}

// ── Control & modulation declarations ─────────────────────────────────────

/// A controllable parameter declared by a component.
#[derive(Debug, Clone, PartialEq)]
pub struct ControlParam {
    /// Parameter name, must match `ControlDef.property` from the DSL.
    pub name: &'static str,
    /// What kind of control target this maps to.
    pub kind: ControlParamKind,
}

/// Classification of a component's controllable parameter.
#[derive(Debug, Clone, PartialEq)]
pub enum ControlParamKind {
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
pub struct ModulationSink {
    /// Which kind of modulation target this maps to.
    pub target_kind: ModulationSinkKind,
    /// Center/offset voltage for the modulation.
    pub bias: f64,
    /// Modulation amplitude (half-swing from bias).
    pub range: f64,
}

/// Classification of a modulation sink by target type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModulationSinkKind {
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
pub struct ResolveContext {
    /// True if the component's control/modulation pin is driven by an LFO,
    /// envelope follower, or DC bias (not in the audio signal path).
    pub control_pin_is_modulated: bool,
    /// True if the component's wiper pin (pots) appears in the netlist.
    pub wiper_connected: bool,
}

/// How a component participates in circuit graph construction.
pub enum GraphRole {
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
    /// Op-amp / finite-gain VCVS: three pins (pos, neg, out) participate in
    /// the circuit graph as junction nodes. Produces no WDF-tree edge but is
    /// absorbed as a VCVS stamp into the MNA matrix of its containing R-type
    /// adaptor (Werner 2016 nullor, generalised to finite Aol and Ro).
    ///
    /// The three pins are registered in `CircuitGraph::nullor_pins` so the
    /// R-type stage builder can emit one `MnaSystem::stamp_vcvs` per op-amp
    /// with the appropriate datasheet parameters.
    VcvsEdge {
        pin_pos: &'static str,
        pin_neg: &'static str,
        pin_out: &'static str,
    },
}

// ═══════════════════════════════════════════════════════════════════════════
// Solver method hint
// ═══════════════════════════════════════════════════════════════════════════

/// Preferred nonlinear solver method for a component.
///
/// Components that support explicit (closed-form) solvers return
/// `SolverMethod::WrightOmega` from `solver_hint()`.  The compiler build pass
/// uses this hint to select the appropriate root type.
///
/// `NewtonRaphson` is available as an explicit override for debugging or for
/// circuits where the explicit solver needs to be compared against the iterative
/// solver.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SolverMethod {
    /// Newton-Raphson iterative solver (default for all nonlinear devices).
    #[default]
    NewtonRaphson,
    /// Wright Omega explicit solver (closed-form, no outer NR loop).
    /// Faster than NR for diodes; equivalent audio quality.
    WrightOmega,
    /// Simplified Ebers-Moll BJT model (fast alternative to Gummel-Poon).
    ///
    /// Omits Early effect, high-injection knee currents, leakage, and
    /// parasitic resistances. Uses only IS, BF, BR, NF, NR from the model.
    /// Approximately 3× faster per NR iteration than GummelPoon.
    ///
    /// Suitable for clean amplifier stages where GP advanced effects are
    /// negligible at the operating point.
    EbersMoll,
    /// Full Gummel-Poon BJT model (default for BJTs).
    ///
    /// Includes Early effect, high-injection, leakage, and parasitic
    /// resistances. More accurate but more expensive per NR iteration.
    GummelPoon,
}

/// Result of applying DC bias to an active component.
#[derive(Debug, Clone)]
pub enum BiasResult {
    /// Component doesn't use bias (passive elements, diodes).
    NotApplicable,
    /// Bias applied successfully. Contains the computed model parameters.
    Applied {
        /// Positive rail voltage (V above bias point before clipping).
        v_rail_pos: f64,
        /// Negative rail voltage (V below bias point before clipping).
        v_rail_neg: f64,
    },
}

// ═══════════════════════════════════════════════════════════════════════════
// Component trait
// ═══════════════════════════════════════════════════════════════════════════

/// The single source of truth for circuit component behavior.
///
/// Every circuit element in PedalKernel — resistors, capacitors, op-amps,
/// BJTs, vacuum tubes, diodes, potentiometers, transformers, BBD delay lines —
/// implements this trait. The compilation pipeline queries these methods to
/// determine topology, solver strategy, stage boundaries, and runtime behavior
/// without ever pattern-matching on concrete component types.
///
/// # Trait Object Support
///
/// `Component` is used as `Box<dyn Component>` throughout the pipeline. It
/// requires [`Debug`] for trait-object formatting and provides `clone_box`,
/// `as_any`, and `dyn_eq` for trait-object [`Clone`], downcasting, and
/// [`PartialEq`] support.
///
/// # Method Groups
///
/// The trait methods are organized into functional groups:
///
/// ## Identity
/// - [`type_tag`](Component::type_tag) — human-readable name (e.g., `"resistor"`, `"NPN transistor"`)
///
/// ## Ports
/// - [`ports`](Component::ports) — terminal pairs carrying current. A resistor has 1 port `(a, b)`;
///   a BJT has 2 ports `(base-emitter, collector-emitter)`.
///
/// ## Signal Flow
/// - [`signal_terminals`](Component::signal_terminals) — directionality for feedback analysis.
///   Amplifiers declare input/output pins; passive components are bidirectional.
///   The compiler uses Tarjan's SCC algorithm on the directed signal graph to
///   detect feedback loops.
/// - [`output_impedance`](Component::output_impedance) — determines stage split points.
///   [`OutputImpedance::VoltageSource`] (op-amps in feedback) allows safe splitting;
///   [`OutputImpedance::Finite`] (passives, BJT collectors) prevents it.
///   Based on Harold Black's negative feedback theorem (1934).
///
/// ## Non-Idealities
/// - [`nonideal_fx`](Component::nonideal_fx) — declares physical imperfections as
///   [`NonIdealFx`] variants. Values come from SPICE models and datasheets. The
///   stage builder applies these as composable post-processing filters after the
///   ideal WDF/MNA solver.
///
/// ## Classification
/// - [`is_passive`](Component::is_passive) — whether BFS can collect this component into a stage
/// - [`is_nonlinear`](Component::is_nonlinear) — whether this needs a Newton-Raphson solver
/// - [`is_active_ic`](Component::is_active_ic) — whether this counts toward `num_active`
/// - [`is_variable`](Component::is_variable) — whether the edge impedance changes at runtime
///   (triggers scattering matrix recomputation)
/// - Family booleans: [`is_bjt`](Component::is_bjt), [`is_jfet`](Component::is_jfet),
///   [`is_tube`](Component::is_tube), [`is_pot`](Component::is_pot), etc.
///
/// ## Graph Building
/// - [`graph_role`](Component::graph_role) — how this component participates in the circuit graph.
///   [`GraphRole::Edge`] for passives, [`GraphRole::VcvsEdge`] for op-amps,
///   [`GraphRole::Virtual`] for LFOs, [`GraphRole::Pot`] for potentiometers.
/// - [`edges`](Component::edges) — declares circuit edges with [`EdgeKind`] classification.
///   [`EdgeKind::Linear`] and [`EdgeKind::Reactive`] form passive WDF trees;
///   [`EdgeKind::Nonlinear`] seeds NR solver stages; [`EdgeKind::Vcvs`] forces R-node MNA.
/// - [`resolve_edges`](Component::resolve_edges) — context-dependent edge resolution. A JFET
///   with its gate driven by an LFO resolves from `Nonlinear` to `Linear` (variable resistor).
///
/// ## MNA Stamping
/// - [`stamp_mna`](Component::stamp_mna) — stamp into a 2-terminal MNA system (resistors, caps)
/// - [`stamp_mna_multi`](Component::stamp_mna_multi) — stamp with multi-terminal pin resolution
///   (op-amps resolve pos/neg/out pins)
/// - [`mna_vsource_count`](Component::mna_vsource_count) — voltage sources needed (op-amps: 1)
/// - [`mna_internal_node_count`](Component::mna_internal_node_count) — internal MNA nodes (op-amps: 1 for GBW pole)
///
/// ## WDF Leaf Creation
/// - [`make_leaf`](Component::make_leaf) — create a runtime WDF leaf node (`DynNode`).
///   Resistors, capacitors, and inductors return leaf nodes; nonlinear and virtual
///   components return `None`.
///
/// ## Pin Interface
/// - [`pin_config`](Component::pin_config) — valid pin names and aliases
/// - [`pin_direction`](Component::pin_direction) — inferred direction for layout (Input, Output, Up, Down)
///
/// ## Controls
/// - [`controls`](Component::controls) — declares runtime-adjustable parameters ([`ControlParam`]).
///   Pots declare `PotPosition`; LFOs declare `LfoRate` and `LfoDepth`.
/// - [`modulation_sink`](Component::modulation_sink) — how this component receives LFO/envelope
///   control signals, including bias voltage and modulation range.
///
/// ## Validation
/// - [`validate_values`](Component::validate_values) — design rule checks for suspicious or
///   invalid component values (e.g., a 1-ohm resistor, a 1-farad capacitor).
///
/// # Implementing a New Component
///
/// To add a new component type to PedalKernel:
///
/// 1. Create a struct in [`super::components`] (e.g., `MyDevice { model: String }`)
/// 2. Implement `Component` with at minimum:
///    - `type_tag()` — return a human-readable name
///    - `is_passive()` — `true` for R/C/L, `false` for active devices
///    - `pin_config()` — declare valid pin names
///    - `graph_role()` — how it enters the circuit graph
///    - `edges()` — declare edge kinds (Linear, Nonlinear, etc.)
///    - `stamp_mna()` — stamp into MNA matrices (for R-node solving)
///    - `footprint_ref()` — KiCad symbol reference
/// 3. For nonlinear devices, also implement `classify_nonlinear()` and set
///    `is_nonlinear() -> true`
/// 4. For active devices with feedback, implement `output_impedance()` and
///    `signal_terminals()`
/// 5. Add the DSL parser variant in [`crate::dsl`]
///
/// No changes are needed in the graph builder, SPQR decomposer, stage builder,
/// or any other pipeline module.
pub trait Component: std::fmt::Debug {
    // ── Trait Object Support ──────────────────────────────────────────────

    /// Clone into a boxed trait object.
    fn clone_box(&self) -> Box<dyn Component>;

    /// Downcast to `Any` for type-erased equality checks and downcasting.
    fn as_any(&self) -> &dyn std::any::Any;

    /// Downcast to `Any` mutably for in-place mutation (e.g., tolerance tweaks).
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any;

    /// Dynamic equality: returns `true` if `other` is the same concrete type
    /// and compares equal.
    fn dyn_eq(&self, other: &dyn Component) -> bool;

    // ── Identity ──────────────────────────────────────────────────────────

    /// Human-readable type name (e.g. "resistor", "NPN transistor").
    fn type_tag(&self) -> &'static str;

    // ── Solver Hint ───────────────────────────────────────────────────────

    /// Return the preferred nonlinear solver method for this component.
    ///
    /// Returns `None` to accept the global default (Newton-Raphson).
    /// Returns `Some(SolverMethod::WrightOmega)` to request the explicit
    /// closed-form Wright Omega solver (diodes only).
    ///
    /// The build pass uses this to choose between `DiodePairRoot`/`DiodeRoot`
    /// (NR) and `ExplicitDiodePairRoot`/`ExplicitDiodeRoot` (WO).
    fn solver_hint(&self) -> Option<SolverMethod> {
        None
    }

    // ── Ports ─────────────────────────────────────────────────────────────

    /// Terminal pairs carrying current through this component.
    ///
    /// Each port = one graph edge. A resistor has 1 port (a, b).
    /// A BJT has 2 ports (base-emitter, collector-emitter).
    /// A pot has 2 ports (a-wiper, wiper-b).
    /// An op-amp has 1 port (neg-out, pos is voltage-sense).
    fn ports(&self) -> Vec<(&'static str, &'static str)> {
        vec![("a", "b")] // default: 1-port component
    }

    // ── Non-Idealities ────────────────────────────────────────────────────

    /// Non-ideal behaviors for this component (GBW, slew, rails, thermal, etc.).
    ///
    /// Each component declares what non-idealities it has. Values come from
    /// the SPICE model / datasheet lookup. The stage builder attaches them
    /// as post-processing. No pattern matching on component type.
    ///
    /// Default: empty vec (ideal component).
    fn nonideal_fx(&self, _sample_rate: f64) -> Vec<NonIdealFx> {
        Vec::new()
    }

    // ── Signal Flow ──────────────────────────────────────────────────────

    /// Output impedance at this component's output pin.
    ///
    /// `VoltageSource` means the output node voltage is determined entirely
    /// by the component (via feedback), independent of downstream load.
    /// The compiler can safely split the circuit at these nodes.
    ///
    /// `Finite` (default) means splitting here would change impedances
    /// and break the transfer function.
    fn output_impedance(&self) -> OutputImpedance {
        OutputImpedance::Finite
    }

    /// Signal flow classification for this component's pins.
    ///
    /// Drives feedback analysis: amplifier input→output defines the
    /// feedback cycle direction. Passive components have no directionality.
    fn signal_terminals(&self) -> SignalTerminals {
        SignalTerminals::Passive
    }

    // ── Classification ────────────────────────────────────────────────────

    /// Whether this component is passive (collectible by BFS for stage building).
    fn is_passive(&self) -> bool;

    /// Whether this component is a nonlinear element (needs NR solver).
    fn is_nonlinear(&self) -> bool {
        false
    }

    /// Whether this component is an active IC (counted toward `num_active`).
    fn is_active_ic(&self) -> bool {
        false
    }

    /// Whether this component is a control-only element (no circuit pins).
    fn is_control_only(&self) -> bool {
        false
    }

    /// Whether this component's edge impedance changes at runtime.
    /// Stages containing variable edges need scattering matrix recomputation.
    fn is_variable(&self) -> bool {
        false
    }

    // ── Pin Interface ─────────────────────────────────────────────────────

    /// Valid pins and aliases for this component type.
    fn pin_config(&self) -> PinConfig;

    /// Extra pins that are valid as modulation targets but may not be in `pin_config`.
    fn modulation_pins(&self) -> &'static [&'static str] {
        &[]
    }

    // ── Graph Building ────────────────────────────────────────────────────

    /// How this component participates in circuit graph construction.
    fn graph_role(&self) -> GraphRole;

    /// Declare this component's circuit edges with their electrical classification.
    ///
    /// Returns empty for Virtual/ActiveIc components (no WDF topology participation).
    /// The planner uses these edges to group components into stages by EdgeKind.
    fn edges(&self) -> Vec<ComponentEdge> {
        vec![]
    }

    /// Internal signal-path adjacencies: pin pairs that signal can traverse through.
    ///
    /// Used by the validator to check in→out reachability. Default derives pairs
    /// from `edges()`; if edges is empty, fans out from first valid_pin to all others.
    /// Override for multi-terminal or complex components (BJTs, pots, transformers).
    fn signal_adjacencies(&self) -> Vec<(&'static str, &'static str)> {
        let edge_list = self.edges();
        if !edge_list.is_empty() {
            edge_list.iter().map(|e| (e.pin_a, e.pin_b)).collect()
        } else {
            let pins = self.pin_config().valid_pins;
            if pins.len() >= 2 {
                pins[1..].iter().map(|&p| (pins[0], p)).collect()
            } else {
                vec![]
            }
        }
    }

    /// Infer the signal direction for a given pin on this component.
    ///
    /// Used by the layout engine to determine directed graph edges.
    /// Default: `Bidirectional` for all pins.
    fn pin_direction(&self, _pin: &str) -> PinDirection {
        PinDirection::Bidirectional
    }

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

    /// Number of MNA voltage sources this component needs when stamped
    /// into an R-node. Default: 0. Op-amps return 1 (for the VCVS constraint).
    fn mna_vsource_count(&self) -> usize {
        0
    }

    /// Number of internal MNA nodes this component needs.
    /// Default: 0. Op-amps return 1 (internal gain stage node for GBW pole).
    fn mna_internal_node_count(&self) -> usize {
        0
    }

    /// Stamp this component into an MNA system using multi-terminal pin resolution.
    ///
    /// Default implementation resolves pins "a" and "b" and delegates to
    /// `stamp_mna()`. Multi-terminal components (op-amps) override this to
    /// resolve all their pins and stamp directly.
    fn stamp_mna_multi(
        &self,
        comp_id: &str,
        ctx: &mut StampContext,
        mna: &mut MnaSystem,
    ) -> StampResult {
        let n1 = (ctx.pin_to_mna)("a");
        let n2 = (ctx.pin_to_mna)("b");
        self.stamp_mna(comp_id, n1, n2, mna, ctx.sample_rate)
    }

    // ── WDF Leaf Creation ─────────────────────────────────────────────────

    /// Create a WDF leaf node for this component.
    /// Returns `None` for non-leaf components (diodes, virtual elements, etc.).
    fn make_leaf(&self, _comp_id: &str, _sample_rate: f64) -> Option<DynNode> {
        None
    }

    // ── Value Access ──────────────────────────────────────────────────────

    fn resistance(&self) -> Option<f64> {
        None
    }
    fn capacitance(&self) -> Option<f64> {
        None
    }
    fn inductance(&self) -> Option<f64> {
        None
    }

    // ── Validation ────────────────────────────────────────────────────────

    /// Check for suspicious or invalid component values.
    fn validate_values(&self, _comp_id: &str) -> Vec<(Severity, String)> {
        vec![]
    }

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
    fn controls(&self) -> Vec<ControlParam> {
        vec![]
    }

    /// If this component can be a modulation sink (LFO/envelope target),
    /// return the sink description for the given pin.
    ///
    /// Called during LFO/envelope binding to determine bias and range.
    fn modulation_sink(&self, _pin: &str) -> Option<ModulationSink> {
        None
    }

    // ── Hardware ──────────────────────────────────────────────────────────

    /// KiCad symbol library reference and designator prefix.
    fn footprint_ref(&self) -> (&'static str, &'static str);

    // ── Classification (family booleans) ─────────────────────────────────
    // Default: false. Override in concrete structs that belong to each family.

    fn is_modulation_source(&self) -> bool {
        false
    }
    fn is_bjt(&self) -> bool {
        false
    }
    fn is_jfet(&self) -> bool {
        false
    }
    fn is_mosfet(&self) -> bool {
        false
    }
    fn is_tube(&self) -> bool {
        false
    }
    fn is_transformer(&self) -> bool {
        false
    }
    fn is_pot(&self) -> bool {
        false
    }
    fn is_diode_family(&self) -> bool {
        false
    }
    fn is_trigger(&self) -> bool {
        false
    }

    /// Whether this element's input node acts as a summing junction that
    /// should block BFS traversal in the signal flow graph.
    ///
    /// Op-amp neg nodes are summing junctions: the input signal, feedback
    /// network, and interstage coupling all connect to the same node.
    /// Without blocking, BFS from a downstream element can traverse backward
    /// through this element's feedback, across shared passives, and reach
    /// upstream elements — creating false cycle edges.
    ///
    /// Returns true for op-amps (VCVS). Returns false for diodes, BJTs,
    /// tubes, etc. — their input nodes don't create false coupling paths.
    fn feedback_input_is_barrier(&self) -> bool {
        false
    }

    // ── Composite classification ─────────────────────────────────────────

    /// Passive two-terminal element (R, C, L, etc.) — excludes transformers and pots.
    fn is_simple_passive(&self) -> bool {
        self.is_passive() && !self.is_transformer()
    }

    /// Amplifying device: tube, BJT, JFET, MOSFET.
    fn is_gain_device(&self) -> bool {
        false
    }

    // ── Accessor methods (defaults return None) ──────────────────────────

    fn model_name(&self) -> Option<&str> {
        None
    }
    fn op_amp_type(&self) -> Option<crate::dsl::OpAmpType> {
        None
    }
    fn pot_taper(&self) -> Option<crate::dsl::PotTaper> {
        None
    }
    fn diode_type(&self) -> Option<crate::dsl::DiodeType> {
        None
    }
    fn transformer_config(&self) -> Option<&crate::dsl::TransformerConfig> {
        None
    }

    // ── Bias application ────────────────────────────────────────────────

    /// Apply DC bias from a static bias network detected at compile time.
    ///
    /// `bias_voltages` maps pin names (e.g. "pos", "neg", "base", "gate")
    /// to their DC voltage computed from the circuit's resistor divider.
    /// `supply_voltage` is the pedal's supply rail voltage (e.g. 9.0V).
    ///
    /// Each component type interprets bias differently:
    /// - **Op-amp**: sets positive/negative rail limits from bias point
    /// - **BJT**: sets quiescent Ic, Vce from collector/emitter voltages
    /// - **Triode/Pentode**: sets grid bias (Vgk) from grid DC voltage
    /// - **JFET**: sets Vgs from gate bias voltage
    ///
    /// Returns `BiasResult` describing what was applied.
    /// Default: no-op (passive components don't need bias).
    fn apply_bias(
        &self,
        _bias_voltages: &hashbrown::HashMap<String, f64>,
        _supply_voltage: f64,
    ) -> BiasResult {
        BiasResult::NotApplicable
    }

    // ── Layout methods (defaults use type_tag) ──────────────────────────

    /// Symbol identifier for layout rendering (e.g., "resistor", "triode", "opamp").
    fn symbol_name(&self) -> &'static str {
        self.type_tag()
    }

    /// Layout class identifier for placement grouping (e.g., "resistor", "npn", "opamp").
    fn layout_class(&self) -> &'static str {
        self.type_tag()
    }

    /// Human-readable display value for labels (e.g., "4.7kΩ", "100nF").
    fn display_value(&self) -> Option<String> {
        None
    }
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
// Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;
    use crate::compiler::components::*;
    use crate::dsl::CapConfig;

    #[test]
    fn box_dyn_component_clone() {
        let r: Box<dyn Component> = Box::new(Resistor { value: 1000.0 });
        let r2 = r.clone();
        assert_eq!(r2.type_tag(), "resistor");
        assert_eq!(r2.resistance(), Some(1000.0));
    }

    #[test]
    fn box_dyn_component_eq() {
        let r1: Box<dyn Component> = Box::new(Resistor { value: 1000.0 });
        let r2: Box<dyn Component> = Box::new(Resistor { value: 1000.0 });
        let r3: Box<dyn Component> = Box::new(Resistor { value: 2200.0 });
        let c1: Box<dyn Component> = Box::new(Capacitor {
            config: CapConfig::new(100e-9),
        });
        assert!(r1.dyn_eq(r2.as_ref()));
        assert!(!r1.dyn_eq(r3.as_ref()));
        assert!(!r1.dyn_eq(c1.as_ref()));
    }

    #[test]
    fn box_dyn_component_debug() {
        let r: Box<dyn Component> = Box::new(Resistor { value: 4700.0 });
        let dbg = format!("{:?}", r);
        assert!(dbg.contains("4700"));
    }

    #[test]
    fn box_dyn_component_downcast() {
        let r: Box<dyn Component> = Box::new(Resistor { value: 1000.0 });
        let any = r.as_any();
        assert!(any.downcast_ref::<Resistor>().is_some());
    }

    #[test]
    fn edges_linear_resistor() {
        let r = Resistor { value: 1000.0 };
        let edges = r.edges();
        assert_eq!(edges.len(), 1);
        assert_eq!(edges[0].kind, EdgeKind::Linear);
        assert_eq!(edges[0].pin_a, "a");
        assert_eq!(edges[0].pin_b, "b");
    }

    #[test]
    fn edges_reactive_capacitor() {
        let c = Capacitor {
            config: CapConfig::new(100e-9),
        };
        let edges = c.edges();
        assert_eq!(edges.len(), 1);
        assert_eq!(edges[0].kind, EdgeKind::Reactive);
    }

    #[test]
    fn edges_nonlinear_diode() {
        let d = Diode {
            diode_type: crate::dsl::DiodeType::Silicon,
        };
        let edges = d.edges();
        assert_eq!(edges.len(), 1);
        assert_eq!(edges[0].kind, EdgeKind::Nonlinear);
    }

    #[test]
    fn edges_nonlinear_bjt() {
        let q = Npn {
            model: "2N3904".into(),
        };
        let edges = q.edges();
        assert_eq!(edges.len(), 1);
        assert_eq!(edges[0].kind, EdgeKind::Nonlinear);
        assert_eq!(edges[0].pin_a, "collector");
        assert_eq!(edges[0].pin_b, "emitter");
    }

    #[test]
    fn edges_nonlinear_triode() {
        let v = Triode {
            model: "12AX7".into(),
        };
        let edges = v.edges();
        assert_eq!(edges.len(), 1);
        assert_eq!(edges[0].kind, EdgeKind::Nonlinear);
        assert_eq!(edges[0].pin_a, "plate");
    }

    #[test]
    fn edges_behavioral_bbd() {
        let b = Bbd {
            bbd_type: crate::dsl::BbdType::Mn3207,
        };
        let edges = b.edges();
        assert_eq!(edges.len(), 1);
        assert_eq!(edges[0].kind, EdgeKind::Behavioral);
    }

    #[test]
    fn edges_virtual_lfo_empty() {
        let l = Lfo {
            waveform: crate::dsl::LfoWaveformDsl::Triangle,
            timing_r: 100_000.0,
            timing_c: 1e-6,
        };
        assert!(l.edges().is_empty());
    }

    #[test]
    fn edges_active_ic_opamp_vcvs() {
        let o = OpAmp {
            op_type: crate::dsl::OpAmpType::Tl072,
        };
        let edges = o.edges();
        assert_eq!(edges.len(), 1);
        assert_eq!(edges[0].kind, EdgeKind::Vcvs);
        assert_eq!(edges[0].pin_a, "neg");
        assert_eq!(edges[0].pin_b, "out");
    }

    #[test]
    fn edges_ota_nonlinear() {
        let ota = OpAmp {
            op_type: crate::dsl::OpAmpType::Ca3080,
        };
        let edges = ota.edges();
        assert_eq!(edges.len(), 1);
        assert_eq!(edges[0].kind, EdgeKind::Nonlinear);
        assert_eq!(edges[0].pin_a, "pos");
    }

    #[test]
    fn resolve_jfet_to_linear_when_modulated() {
        let j = NJfet {
            model: "2N5457".into(),
        };
        // Default: nonlinear
        assert_eq!(j.edges()[0].kind, EdgeKind::Nonlinear);
        // Modulated gate: resolves to linear (variable resistor)
        let ctx = ResolveContext {
            control_pin_is_modulated: true,
            wiper_connected: false,
        };
        let resolved = j.resolve_edges(&ctx).unwrap();
        assert_eq!(resolved.len(), 1);
        assert_eq!(resolved[0].kind, EdgeKind::Linear);
        assert_eq!(resolved[0].pin_a, "drain");
    }

    #[test]
    fn resolve_jfet_no_change_without_modulation() {
        let j = PJfet {
            model: "2N5460".into(),
        };
        let ctx = ResolveContext {
            control_pin_is_modulated: false,
            wiper_connected: false,
        };
        assert!(j.resolve_edges(&ctx).is_none());
    }

    #[test]
    fn resolve_ota_to_vccs_when_modulated() {
        let ota = OpAmp {
            op_type: crate::dsl::OpAmpType::Ca3080,
        };
        // Default: nonlinear
        assert_eq!(ota.edges()[0].kind, EdgeKind::Nonlinear);
        // Modulated Iabc: resolves to VCCS
        let ctx = ResolveContext {
            control_pin_is_modulated: true,
            wiper_connected: false,
        };
        let resolved = ota.resolve_edges(&ctx).unwrap();
        assert_eq!(resolved.len(), 1);
        assert_eq!(resolved[0].kind, EdgeKind::Vccs);
    }

    #[test]
    fn resolve_regular_opamp_unchanged() {
        let op = OpAmp {
            op_type: crate::dsl::OpAmpType::Tl072,
        };
        let ctx = ResolveContext {
            control_pin_is_modulated: true,
            wiper_connected: false,
        };
        // Regular opamps don't resolve (no modulation pins)
        assert!(op.resolve_edges(&ctx).is_none());
    }
}
