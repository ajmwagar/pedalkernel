//! Component trait: compile-time behavior for `ComponentKind`.
//!
//! Centralizes passivity, MNA stamping, pin configs, WDF leaf creation,
//! validation, and KiCad export so that callers in graph.rs, validate.rs,
//! classify.rs, compile.rs, build.rs, and kicad.rs can delegate instead
//! of duplicating match blocks.

use std::collections::HashMap;

use crate::dsl::*;
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
// Dispatch: ComponentKind → concrete Component struct
// ═══════════════════════════════════════════════════════════════════════════

/// Dispatch a `ComponentKind` to its concrete `Component` struct, calling `f`
/// on a temporary instance.  This is the single match block that replaces 18+
/// per-method match blocks.
fn with_concrete<R>(kind: &ComponentKind, f: impl FnOnce(&dyn Component) -> R) -> R {
    use super::components::*;
    match kind {
        ComponentKind::Resistor(v) => f(&Resistor { value: *v }),
        ComponentKind::Capacitor(cfg) => f(&Capacitor { config: cfg.clone() }),
        ComponentKind::Inductor(v) => f(&Inductor { value: *v }),
        ComponentKind::Potentiometer(r, t) => f(&Potentiometer { max_r: *r, taper: *t }),
        ComponentKind::Tempco(r, p) => f(&Tempco { resistance: *r, ppm: *p }),
        ComponentKind::CapSwitched(v) => f(&CapSwitched { values: v.clone() }),
        ComponentKind::InductorSwitched(v) => f(&InductorSwitched { values: v.clone() }),
        ComponentKind::ResistorSwitched(v) => f(&ResistorSwitched { values: v.clone() }),
        ComponentKind::Diode(dt) => f(&Diode { diode_type: *dt }),
        ComponentKind::DiodePair(dt) => f(&DiodePair { diode_type: *dt }),
        ComponentKind::Zener(v) => f(&Zener { breakdown_voltage: *v }),
        ComponentKind::Neon(nt) => f(&Neon { neon_type: *nt }),
        ComponentKind::Npn(m) => f(&Npn { model: m.clone() }),
        ComponentKind::Pnp(m) => f(&Pnp { model: m.clone() }),
        ComponentKind::NJfet(m) => f(&NJfet { model: m.clone() }),
        ComponentKind::PJfet(m) => f(&PJfet { model: m.clone() }),
        ComponentKind::Nmos(mt) => f(&Nmos { mosfet_type: *mt }),
        ComponentKind::Pmos(mt) => f(&Pmos { mosfet_type: *mt }),
        ComponentKind::Triode(m) => f(&Triode { model: m.clone() }),
        ComponentKind::Pentode(m) => f(&Pentode { model: m.clone() }),
        ComponentKind::VariMu(m) => f(&VariMu { model: m.clone() }),
        ComponentKind::OpAmp(ot) => f(&OpAmp { op_type: *ot }),
        ComponentKind::Vco(vt, freq, wf) => f(&Vco { vco_type: *vt, base_freq: *freq, waveform: *wf }),
        ComponentKind::Vcf(vt) => f(&Vcf { vcf_type: *vt }),
        ComponentKind::Vca(vt) => f(&Vca { vca_type: *vt }),
        ComponentKind::Comparator(ct) => f(&Comparator { comp_type: *ct }),
        ComponentKind::AnalogSwitch(st) => f(&AnalogSwitch { switch_type: *st }),
        ComponentKind::MatchedNpn(mt) => f(&MatchedNpn { matched_type: *mt }),
        ComponentKind::MatchedPnp(mt) => f(&MatchedPnp { matched_type: *mt }),
        ComponentKind::Lfo(w, r, c) => f(&Lfo { waveform: *w, timing_r: *r, timing_c: *c }),
        ComponentKind::EnvelopeFollower(ar, ac, rr, rc, sr) => f(&EnvelopeFollower {
            attack_r: *ar, attack_c: *ac, release_r: *rr, release_c: *rc, sensitivity_r: *sr,
        }),
        ComponentKind::Photocoupler(pt) => f(&PhotocouplerComp { coupler_type: *pt }),
        ComponentKind::Bbd(bt) => f(&Bbd { bbd_type: *bt }),
        ComponentKind::DelayLine(min, max, interp, med) => f(&DelayLineComp {
            min_delay: *min, max_delay: *max, interpolation: *interp, medium: *med,
        }),
        ComponentKind::Tap(id, ratio) => f(&Tap { parent_id: id.clone(), ratio: *ratio }),
        ComponentKind::Transformer(cfg) => f(&TransformerComp { config: cfg.clone() }),
        ComponentKind::RotarySwitch(ids) => f(&RotarySwitch { linked_ids: ids.clone() }),
        ComponentKind::Switch(n) => f(&Switch { positions: *n }),
        ComponentKind::TriggerInput => f(&TriggerInputComp),
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Implementation on ComponentKind — delegates to concrete structs
// ═══════════════════════════════════════════════════════════════════════════

impl Component for ComponentKind {
    // ── Trait Object Support (cannot delegate — operates on ComponentKind identity) ──

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

    // ── All other methods delegate via with_concrete ─────────────────────

    fn type_tag(&self) -> &'static str { with_concrete(self, |c| c.type_tag()) }
    fn is_passive(&self) -> bool { with_concrete(self, |c| c.is_passive()) }
    fn is_nonlinear(&self) -> bool { with_concrete(self, |c| c.is_nonlinear()) }
    fn is_active_ic(&self) -> bool { with_concrete(self, |c| c.is_active_ic()) }
    fn is_control_only(&self) -> bool { with_concrete(self, |c| c.is_control_only()) }
    fn is_variable(&self) -> bool { with_concrete(self, |c| c.is_variable()) }
    fn pin_config(&self) -> PinConfig { with_concrete(self, |c| c.pin_config()) }
    fn modulation_pins(&self) -> &'static [&'static str] { with_concrete(self, |c| c.modulation_pins()) }
    fn graph_role(&self) -> GraphRole { with_concrete(self, |c| c.graph_role()) }
    fn edges(&self) -> Vec<ComponentEdge> { with_concrete(self, |c| c.edges()) }
    fn resolve_edges(&self, ctx: &ResolveContext) -> Option<Vec<ComponentEdge>> {
        with_concrete(self, |c| c.resolve_edges(ctx))
    }
    fn stamp_mna(
        &self,
        comp_id: &str,
        n1: Option<usize>,
        n2: Option<usize>,
        mna: &mut MnaSystem,
        sample_rate: f64,
    ) -> StampResult {
        with_concrete(self, |c| c.stamp_mna(comp_id, n1, n2, mna, sample_rate))
    }
    fn make_leaf(&self, comp_id: &str, sample_rate: f64) -> Option<DynNode> {
        with_concrete(self, |c| c.make_leaf(comp_id, sample_rate))
    }
    fn resistance(&self) -> Option<f64> { with_concrete(self, |c| c.resistance()) }
    fn capacitance(&self) -> Option<f64> { with_concrete(self, |c| c.capacitance()) }
    fn inductance(&self) -> Option<f64> { with_concrete(self, |c| c.inductance()) }
    fn validate_values(&self, comp_id: &str) -> Vec<(Severity, String)> {
        with_concrete(self, |c| c.validate_values(comp_id))
    }
    fn classify_nonlinear(
        &self,
        comp_id: &str,
        node_a: NodeId,
        node_b: NodeId,
        gnd_node: NodeId,
        node_names: &HashMap<String, NodeId>,
    ) -> Option<(NonlinearKind, Vec<NodeId>)> {
        with_concrete(self, |c| c.classify_nonlinear(comp_id, node_a, node_b, gnd_node, node_names))
    }
    fn controls(&self) -> Vec<ControlParam> { with_concrete(self, |c| c.controls()) }
    fn modulation_sink(&self, pin: &str) -> Option<ModulationSink> {
        with_concrete(self, |c| c.modulation_sink(pin))
    }
    fn footprint_ref(&self) -> (&'static str, &'static str) {
        with_concrete(self, |c| c.footprint_ref())
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
