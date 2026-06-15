//! Centralized, **derived** port-classification "rules broker" (Phase 1).
//!
//! # What this is (and is NOT)
//!
//! Stage formation today scatters its boundary decisions across several
//! analyses: `signal_flow` decides feedback vs cascade, `neighbor_roles`
//! infers per-terminal Load/Ref/Signal roles, the modulation-sink machinery
//! (`bind.rs`) knows transducer ports, and `graph` knows the rail sets. Each
//! re-derives a slice of "what does this port DO" from the device models.
//!
//! This module is the single place that owns BOTH:
//!   1. the **detection rules** — [`PortClass::from_component`] derives a port's
//!      class from the *existing model facts* (no new hand-maintained tag), and
//!   2. their **declarative consequences** — [`BoundaryPolicy`] is the whole
//!      rule table in one [`BoundaryPolicy::classify`], and [`Directive`] is the
//!      consequence a later formation phase would apply.
//!
//! **Phase 1 is behaviour-neutral.** Nothing here is wired into formation,
//! `find_flow_groups`, partitioning, or stamping. [`Directive`] outputs are
//! defined but UNUSED by compilation/audio. This mirrors how
//! [`super::neighbor_roles`] landed safely: a pure classification + diagnostic
//! that a *later* arbitration phase will consume.
//!
//! # Derivation provenance (which model fact each `PortClass` comes from)
//!
//! | `PortClass`           | Derived from                                                                 |
//! |-----------------------|------------------------------------------------------------------------------|
//! | [`PortClass::Rail`]   | `graph.gnd_node` / `vcc_node` / `supply_nodes` / `ac_ground_nodes`           |
//! | [`PortClass::Transducer`] | `Component::modulation_sink(pin)` → `ModulationSinkKind` (the `.led`/`.cv`/`.iabc`/`.vgs` ports) |
//! | [`PortClass::ControlInput`] | `Component::signal_terminals()` `Amplifier { input, control }` (grid/base/gate, op-amp pos+neg) |
//! | [`PortClass::Conducting`] | everything else (stamped MNA/WDF terminals: plate, collector, R/C/L pins)  |
//!
//! The forward-vs-back-edge bit consumed by [`BoundaryPolicy::classify`] is the
//! directed, rail-blocked signal flow from [`super::signal_flow`] (Defect B's
//! `directed_signal_distances_from_in`). The broker does not re-derive flow.

use std::collections::HashSet;

use super::component::{Component, EdgeKind, ModulationSinkKind, SignalTerminals};
use super::graph::{CircuitGraph, NodeId};

// ═══════════════════════════════════════════════════════════════════════════
// Coupling domain
// ═══════════════════════════════════════════════════════════════════════════

/// Physical coupling domain of a transducer port.
///
/// Derived from the [`ModulationSinkKind`] of a modulation-sink pin. A
/// transducer port couples two otherwise galvanically-isolated networks
/// through a NON-electrical medium (light, magnetic flux, …) — that medium is
/// the `Domain`. A later formation phase routes such couplings with a
/// one-sample delay across the domain boundary (see [`Directive::RouteDelayed`]
/// / [`BoundaryPolicy::DelayedCoupling`]).
///
/// (The design note `reports/cross-network-coupling-design-2026-06-13.md`
/// sketches a `CouplingDomain { Flux | Light | Thermal | … }`; this is the
/// derived, model-grounded realisation of that idea — every variant maps to a
/// concrete `ModulationSinkKind` the device models already emit.)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Domain {
    /// Optical coupling — photocoupler LED → CdS/LDR cell (`.led`).
    Light,
    /// Control-voltage / current coupling — JFET Vgs, MOSFET Vgs, OTA Iabc,
    /// VCA CV, tube Vgk/Vg1k bias injection. The "control electrode driven by
    /// an external modulation source" family.
    Control,
    /// Clock / sample-rate coupling — BBD clock, delay speed/time.
    Clock,
}

impl Domain {
    /// Derive the coupling domain from a modulation-sink kind.
    pub fn from_sink_kind(kind: ModulationSinkKind) -> Self {
        match kind {
            ModulationSinkKind::PhotocouplerLed => Domain::Light,
            ModulationSinkKind::JfetVgs
            | ModulationSinkKind::MosfetVgs
            | ModulationSinkKind::TriodeVgk
            | ModulationSinkKind::VariMuVgk
            | ModulationSinkKind::PentodeVg1k
            | ModulationSinkKind::OtaIabc
            | ModulationSinkKind::VcaCv
            | ModulationSinkKind::SpringDwell => Domain::Control,
            ModulationSinkKind::BbdClock
            | ModulationSinkKind::DelaySpeed
            | ModulationSinkKind::DelayTime => Domain::Clock,
        }
    }

    /// Human-readable label for diagnostics.
    pub fn label(self) -> &'static str {
        match self {
            Domain::Light => "Light",
            Domain::Control => "Control",
            Domain::Clock => "Clock",
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// PortClass — DERIVED from the device models
// ═══════════════════════════════════════════════════════════════════════════

/// Device-agnostic classification of what a single port (a `component.pin`)
/// *is*, derived entirely from existing model facts — no new annotation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PortClass {
    /// An ordinary current-conducting terminal that gets MNA/WDF-stamped:
    /// a tube plate / cathode, BJT collector / emitter, R/C/L pins, op-amp
    /// output. The default for anything that is not a control / transducer /
    /// rail port.
    Conducting,
    /// A high-impedance signal-control electrode: tube grid, BJT base, FET
    /// gate, op-amp inverting/non-inverting input. Derived from
    /// [`SignalTerminals::Amplifier`] `input`/`control`.
    ControlInput,
    /// A transducer port that couples across a non-electrical [`Domain`]
    /// (photocoupler `.led`, OTA `.iabc`, VCA `.cv`, FET `.vgs`). Derived from
    /// [`Component::modulation_sink`].
    Transducer(Domain),
    /// A supply / ground / AC-ground rail node. Derived from the graph rail
    /// sets. (A *node* property; `from_component` cannot see it, so it is
    /// supplied by [`PortClass::from_pin_in_graph`].)
    Rail,
}

impl PortClass {
    /// Derive the class of `pin` on `comp` from the device model alone.
    ///
    /// This is the component-fact half of the derivation (it cannot see graph
    /// nodes, so it never returns [`PortClass::Rail`] — use
    /// [`PortClass::from_pin_in_graph`] for the rail-aware classification).
    ///
    /// Precedence (most-specific first):
    /// 1. A pin that is a [`Component::modulation_sink`] is a transducer port —
    ///    `.led`, `.iabc`, `.cv`, `.vgs`. (A FET's `vgs`/`gate` IS a transducer
    ///    port when read this way; that is the intended unification — the
    ///    control electrode of an externally-modulated device is a transducer.)
    /// 2. A pin that is the `input` or `control` of an [`SignalTerminals::Amplifier`]
    ///    is a [`PortClass::ControlInput`] — grid / base / gate / op-amp pos+neg.
    /// 3. Otherwise [`PortClass::Conducting`].
    pub fn from_component(comp: &dyn Component, pin: &str) -> Self {
        // (1) Transducer: dedicated modulation-sink port (the .led / .cv /
        //     .iabc / .vgs machinery). Most specific — wins over ControlInput.
        if let Some(sink) = comp.modulation_sink(pin) {
            return PortClass::Transducer(Domain::from_sink_kind(sink.target_kind));
        }
        // (2) ControlInput: amplifier input / control electrode.
        if let SignalTerminals::Amplifier { input, control, .. } = comp.signal_terminals() {
            if pin == input || control == Some(pin) {
                return PortClass::ControlInput;
            }
        }
        // (3) Everything else conducts.
        PortClass::Conducting
    }

    /// Rail-aware classification of `comp.pin` resolved against `graph`.
    ///
    /// Identical to [`PortClass::from_component`] except a pin whose resolved
    /// node is a rail (gnd / vcc / named supply / AC-ground) returns
    /// [`PortClass::Rail`]. The rail check takes precedence because a terminal
    /// tied straight to a rail is electrically a rail port regardless of its
    /// device role.
    pub fn from_pin_in_graph(
        comp: &dyn Component,
        pin: &str,
        node: NodeId,
        graph: &CircuitGraph,
    ) -> Self {
        if node_is_rail(graph, node) {
            return PortClass::Rail;
        }
        Self::from_component(comp, pin)
    }

    /// Human-readable label for diagnostics.
    pub fn label(self) -> &'static str {
        match self {
            PortClass::Conducting => "Conducting",
            PortClass::ControlInput => "ControlInput",
            PortClass::Transducer(_) => "Transducer",
            PortClass::Rail => "Rail",
        }
    }
}

/// True if `node` is a rail node (gnd / vcc / named supply / AC-ground).
///
/// Mirrors `signal_flow::rail_nodes`, but as a single-node predicate so the
/// broker does not allocate the full set per query.
/// True if a component carries inter-sample state/taus: a reactive edge (C / L)
/// or a runtime-variable impedance (pot, photocoupler cell). Derived from the
/// device model's edge kinds + `is_variable`, NOT a hand-maintained tag.
fn component_has_memory(comp: &dyn Component) -> bool {
    comp.is_variable()
        || comp
            .edges()
            .iter()
            .any(|e| matches!(e.kind, EdgeKind::Reactive))
}

/// True if `node` is a rail node (gnd / vcc / named supply / AC-ground).
fn node_is_rail(graph: &CircuitGraph, node: NodeId) -> bool {
    node == graph.gnd_node
        || node == graph.vcc_node
        || graph.supply_nodes.contains(&node)
        || graph.ac_ground_nodes.contains(&node)
}

// ═══════════════════════════════════════════════════════════════════════════
// BoundaryPolicy — the WHOLE rule table in one place
// ═══════════════════════════════════════════════════════════════════════════

/// The boundary policy for an edge between two ports, given the directed
/// signal-flow orientation and component memory.
///
/// This is the single source of truth for "how must an edge between these two
/// kinds of ports be treated at a stage boundary". A later formation phase
/// reads [`BoundaryPolicy::directive`] to decide whether to co-solve the two
/// ends (tight) or cut + route them with a delay (sense / coupling).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoundaryPolicy {
    /// Co-solve: the two ports belong to the same solved system (an ordinary
    /// forward cascade edge, a conducting edge). No boundary cut.
    Tight,
    /// Cut + sense with a one-sample delay: a FEEDBACK edge into a control
    /// electrode (e.g. an output tap rectified back into a sidechain grid).
    /// The sense path is broken to keep the forward solve causal.
    DelayedSense,
    /// Cut + couple across a non-electrical [`Domain`] with a one-sample delay:
    /// a transducer edge (photocoupler LED drive, OTA Iabc CV). The two
    /// networks never share a node; the coupling is delivered next sample.
    DelayedCoupling(Domain),
}

impl BoundaryPolicy {
    /// The whole rule table. `ends` are the two port classes of an edge,
    /// `flow_is_back_edge` is true when the edge runs *against* the directed
    /// signal flow (feedback), and `has_memory` records whether the coupling
    /// element carries state/taus (photocoupler cell, etc.).
    ///
    /// Rules (in priority order):
    /// 1. an edge into a [`PortClass::Transducer`] → [`BoundaryPolicy::DelayedCoupling`].
    /// 2. a BACK-edge into a [`PortClass::ControlInput`] → [`BoundaryPolicy::DelayedSense`].
    /// 3. a FORWARD edge into a [`PortClass::ControlInput`] (cascade) → [`BoundaryPolicy::Tight`].
    /// 4. everything else → [`BoundaryPolicy::Tight`].
    ///
    /// `has_memory` is part of the signature because a future refinement may
    /// route a memoryless transducer differently from a stateful one; in the
    /// Phase-1 table it does not change the outcome (a transducer coupling is
    /// always delayed, with or without its own state).
    pub fn classify(
        ends: (PortClass, PortClass),
        flow_is_back_edge: bool,
        has_memory: bool,
    ) -> Self {
        let _ = has_memory; // reserved; see doc comment.
        let (_from, to) = ends;
        match to {
            // (1) Into a transducer port → delayed cross-domain coupling.
            PortClass::Transducer(domain) => BoundaryPolicy::DelayedCoupling(domain),
            // (2)/(3) Into a control electrode: feedback senses delayed,
            //         forward cascade co-solves.
            PortClass::ControlInput => {
                if flow_is_back_edge {
                    BoundaryPolicy::DelayedSense
                } else {
                    BoundaryPolicy::Tight
                }
            }
            // (4) Conducting / rail sinks co-solve.
            PortClass::Conducting | PortClass::Rail => BoundaryPolicy::Tight,
        }
    }

    /// The declarative consequence a formation consumer applies (Phase 1: all
    /// consumers are absent, so this output is never acted on).
    pub fn directive(&self) -> Directive {
        match self {
            BoundaryPolicy::Tight => Directive::CoSolve,
            BoundaryPolicy::DelayedSense => Directive::NonMergeCut,
            BoundaryPolicy::DelayedCoupling(domain) => Directive::RouteDelayed(*domain),
        }
    }
}

/// Declarative consequence of a [`BoundaryPolicy`] for a future formation pass.
///
/// **Unused by compilation/audio in Phase 1.** Defined so consumers (the next
/// phase: `find_flow_groups` / partition / stamping) have a stable vocabulary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Directive {
    /// Keep both ends in one solved system (no cut).
    CoSolve,
    /// Do not merge the two ends into one group; cut the boundary (the sense
    /// path is delivered one sample late). This is the future "feedback
    /// linearisation" / state-variable break point.
    NonMergeCut,
    /// Cut and route the coupling across the given [`Domain`] with a delay
    /// (the future cross-network transducer stamp).
    RouteDelayed(Domain),
}

// ═══════════════════════════════════════════════════════════════════════════
// Public introspection API (tests + the future arbitration phase)
// ═══════════════════════════════════════════════════════════════════════════

/// One classified port of one component pin in a compiled circuit.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClassifiedPort {
    /// Owning component id (e.g. "PC1").
    pub comp_id: String,
    /// Pin name (e.g. "led", "grid").
    pub pin: String,
    /// Derived class.
    pub class: PortClass,
}

/// One classified edge (a `from_port -> to_port` boundary) in a compiled
/// circuit, with its derived [`BoundaryPolicy`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClassifiedEdge {
    /// `(comp_id, pin)` of the upstream (flow-source) end.
    pub from: (String, String),
    /// `(comp_id, pin)` of the downstream (flow-sink) end.
    pub to: (String, String),
    /// Class of the sink port (the one the edge runs INTO).
    pub to_class: PortClass,
    /// Class of the source port.
    pub from_class: PortClass,
    /// Whether the edge runs against directed signal flow (feedback).
    pub is_back_edge: bool,
    /// The derived policy for this boundary.
    pub policy: BoundaryPolicy,
}

/// Classify EVERY pin of EVERY component in `pedal` into a [`PortClass`].
///
/// Behaviour-neutral introspection entry point: it compiles the netlist into a
/// circuit graph and derives a class for each declared, resolved pin. Pins not
/// present in the netlist (no resolved node) are skipped. Used by the proof
/// diagnostic and the future arbitration phase.
pub fn classify_ports(pedal: &crate::dsl::PedalDef) -> Vec<ClassifiedPort> {
    let graph = CircuitGraph::from_pedal(pedal);
    classify_ports_in_graph(&graph)
}

/// Graph-level [`classify_ports`] (in-crate; the public wrapper compiles a
/// `PedalDef` first).
pub(super) fn classify_ports_in_graph(graph: &CircuitGraph) -> Vec<ClassifiedPort> {
    let mut out = Vec::new();
    for comp in graph.components.iter() {
        let cfg = comp.kind.pin_config();
        for &pin in cfg.valid_pins {
            let key = format!("{}.{}", comp.id, pin);
            let Some(&node) = graph.node_names.get(&key) else {
                continue;
            };
            let class = PortClass::from_pin_in_graph(comp.kind.as_ref(), pin, node, graph);
            out.push(ClassifiedPort {
                comp_id: comp.id.clone(),
                pin: pin.to_string(),
                class,
            });
        }
    }
    out
}

/// Classify the boundary of every component edge in `pedal` into a
/// [`BoundaryPolicy`], oriented by directed signal flow.
///
/// For each edge `(node_a, node_b)` owned by a component, the ends are NAMED
/// (source vs sink) by the directed, rail-blocked signal distance from the
/// circuit input (`signal_flow::directed_signal_distances_from_in`): the
/// lower-distance end is the flow SOURCE. The BACK-edge bit, however, is derived
/// from the output-side passive closure (see [`control_sink_is_back_edge`]) —
/// a control electrode fed from the output is feedback — because in
/// transformer-/tube-coupled circuits the directed-distance walk dead-ends at
/// the magnetic/active coupling and cannot orient most edges. The sink port's
/// class plus the back-edge bit drive [`BoundaryPolicy::classify`].
///
/// Behaviour-neutral: the returned policies are not applied to formation.
pub fn classify_edges(pedal: &crate::dsl::PedalDef) -> Vec<ClassifiedEdge> {
    let graph = CircuitGraph::from_pedal(pedal);
    let dist = super::signal_flow::directed_signal_distances_from_in(&graph);

    // node -> list of (comp_id, pin) declared at that node, for naming ends.
    // The output-side passive closure is the back-edge oracle (see
    // `control_sink_is_back_edge`).
    let out_closure = passive_closure_from(&graph, graph.out_node);

    // Resolve each component pin to its node once.
    let mut out = Vec::new();
    for comp in graph.components.iter() {
        let has_memory = component_has_memory(comp.kind.as_ref());
        for ce in comp.kind.edges() {
            let a_key = format!("{}.{}", comp.id, ce.pin_a);
            let b_key = format!("{}.{}", comp.id, ce.pin_b);
            let (Some(&na), Some(&nb)) = (graph.node_names.get(&a_key), graph.node_names.get(&b_key))
            else {
                continue;
            };

            // Orient by directed signal distance when both ends are reachable
            // (source = lower distance); otherwise keep the declared a->b order.
            // Orientation only names the ends — the back-edge bit below is what
            // drives the policy, and it is orientation-independent (keyed on the
            // SINK control electrode's membership in the output closure).
            let da = dist.get(&na).copied();
            let db = dist.get(&nb).copied();
            let a_first = match (da, db) {
                (Some(da), Some(db)) => da <= db,
                _ => true,
            };
            let (from_pin, from_node, to_pin, to_node) = if a_first {
                (ce.pin_a, na, ce.pin_b, nb)
            } else {
                (ce.pin_b, nb, ce.pin_a, na)
            };

            let from_class =
                PortClass::from_pin_in_graph(comp.kind.as_ref(), from_pin, from_node, &graph);
            let to_class =
                PortClass::from_pin_in_graph(comp.kind.as_ref(), to_pin, to_node, &graph);

            // A control-electrode sink fed from the output side is a back-edge.
            // Check BOTH ends so the orientation choice cannot hide a feedback
            // sink: whichever end is the control input decides.
            let back_to = control_sink_is_back_edge(to_class, to_node, &out_closure);
            let back_from = control_sink_is_back_edge(from_class, from_node, &out_closure);
            // Re-orient so the control-input feedback end is the SINK if needed.
            let (from_pin, from_node, from_class, to_pin, to_node, to_class, back) =
                if back_from && !back_to {
                    (to_pin, to_node, to_class, from_pin, from_node, from_class, true)
                } else {
                    (from_pin, from_node, from_class, to_pin, to_node, to_class, back_to)
                };

            let policy = BoundaryPolicy::classify((from_class, to_class), back, has_memory);

            out.push(ClassifiedEdge {
                from: (comp.id.clone(), from_pin.to_string()),
                to: (comp.id.clone(), to_pin.to_string()),
                to_class,
                from_class,
                is_back_edge: back,
                policy,
            });
        }
    }
    out
}

/// True if an edge into `to_class`/`to_node` is a FEEDBACK (back) edge.
///
/// Derivation: a control electrode that is **driven from the output side** is
/// feedback. We define "output side" as the rail-blocked PASSIVE closure of the
/// circuit `out` node (`out_closure`) — every node a passive network ties to the
/// output without crossing a rail or an active device. A [`PortClass::ControlInput`]
/// whose node lands in that closure is fed back from the output (an op-amp's
/// inverting feedback input, a compressor's sidechain tap off `out`), so the
/// edge into it is a back-edge. A forward cascade grid/base/gate sits OUTSIDE
/// the output's passive closure and is therefore NOT a back-edge.
///
/// This reuses the same rail-blocked passive-walk idiom as
/// `signal_flow`/`neighbor_roles` rather than the directed signal DISTANCE,
/// because in transformer-/tube-coupled circuits the directed distance walk
/// dead-ends at the magnetic/active coupling and cannot orient most edges.
fn control_sink_is_back_edge(
    to_class: PortClass,
    to_node: NodeId,
    out_closure: &HashSet<NodeId>,
) -> bool {
    to_class == PortClass::ControlInput && out_closure.contains(&to_node)
}

/// Classify the boundary along a passive PATH from `out`/a source node into a
/// named control electrode, following the *graph topology* rather than a single
/// component edge. This is the proof-diagnostic helper used to classify
/// multi-hop boundaries such as LA-2A's `out -> ... -> V4.grid` feedback tap or
/// the `V1.plate -> C_c1 -> Gain -> V2.grid` makeup cascade: it resolves the
/// control electrode's node, derives its [`PortClass`], and determines feedback
/// vs forward by whether the control electrode is fed from the output side (see
/// [`control_sink_is_back_edge`]).
///
/// `control_pin_key` is a `"Comp.pin"` net key (e.g. `"V4.grid"`); the upstream
/// key may be a reserved net (`"in"`, `"out"`). Returns `None` if a key
/// resolves to no node.
pub fn classify_control_path(
    pedal: &crate::dsl::PedalDef,
    control_pin_key: &str,
    upstream_pin_key: &str,
) -> Option<ClassifiedEdge> {
    let graph = CircuitGraph::from_pedal(pedal);
    let out_closure = passive_closure_from(&graph, graph.out_node);

    let to_node = *graph.node_names.get(control_pin_key)?;
    let from_node = *graph.node_names.get(upstream_pin_key)?;

    // The SINK key must name a real component pin (it is the classified port).
    let (to_comp, to_pin) = split_pin_key(control_pin_key)?;
    let to_class = class_of_keyed_pin(&graph, &to_comp, &to_pin, to_node)?;

    // The SOURCE key may be a reserved net (`in` / `out` / `gnd`) with no
    // component pin — such a node is a conducting junction. Resolve its class
    // from a component pin when the key has one, else treat it as Conducting
    // (rail nodes still resolve to Rail via the node check).
    let (from_comp, from_pin) = match split_pin_key(upstream_pin_key) {
        Some((c, p)) => (c, p),
        None => (upstream_pin_key.to_string(), String::new()),
    };
    let from_class = if from_pin.is_empty() {
        if node_is_rail(&graph, from_node) {
            PortClass::Rail
        } else {
            PortClass::Conducting
        }
    } else {
        class_of_keyed_pin(&graph, &from_comp, &from_pin, from_node)
            .unwrap_or(PortClass::Conducting)
    };

    let back = control_sink_is_back_edge(to_class, to_node, &out_closure);
    let policy = BoundaryPolicy::classify((from_class, to_class), back, false);

    Some(ClassifiedEdge {
        from: (from_comp, from_pin),
        to: (to_comp, to_pin),
        to_class,
        from_class,
        is_back_edge: back,
        policy,
    })
}

/// Derive the [`PortClass`] of a `Comp.pin` already resolved to `node`.
fn class_of_keyed_pin(
    graph: &CircuitGraph,
    comp_id: &str,
    pin: &str,
    node: NodeId,
) -> Option<PortClass> {
    let comp = graph.components.iter().find(|c| c.id == comp_id)?;
    Some(PortClass::from_pin_in_graph(comp.kind.as_ref(), pin, node, graph))
}

/// Split a `"Comp.pin"` key into `(comp, pin)`.
fn split_pin_key(key: &str) -> Option<(String, String)> {
    let (comp, pin) = key.split_once('.')?;
    Some((comp.to_string(), pin.to_string()))
}

/// Robustness probe: classify every pin of every component for `pedal` and
/// confirm the derivation never panics and yields a class for each resolved
/// pin. Returns the number of pins classified. Used by the corpus smoke gate.
pub fn smoke_classify(pedal: &crate::dsl::PedalDef) -> usize {
    let ports = classify_ports(pedal);
    // Also run the edge classifier so its orientation/flow logic is smoked.
    let _edges = classify_edges(pedal);
    ports.len()
}

/// Passive (non-active, non-rail) closure from a seed node: all nodes reachable
/// by walking only `SignalTerminals::Passive` edges, never expanding through a
/// rail. This is the rail-blocked passive neighbourhood of a node — used to ask
/// "is this control input driven from the output side" (feedback).
fn passive_closure_from(graph: &CircuitGraph, seed: NodeId) -> HashSet<NodeId> {
    use super::component::SignalTerminals;
    let mut visited: HashSet<NodeId> = HashSet::new();
    let mut stack = vec![seed];
    visited.insert(seed);
    while let Some(node) = stack.pop() {
        if node != seed && node_is_rail(graph, node) {
            continue;
        }
        for e in graph.edges.iter() {
            let comp = &graph.components[e.comp_idx];
            if !matches!(comp.kind.signal_terminals(), SignalTerminals::Passive) {
                continue;
            }
            let other = if e.node_a == node {
                e.node_b
            } else if e.node_b == node {
                e.node_a
            } else {
                continue;
            };
            if node_is_rail(graph, other) {
                continue;
            }
            if visited.insert(other) {
                stack.push(other);
            }
        }
    }
    visited
}

/// Set of all distinct [`PortClass`] discriminants seen in a corpus pass —
/// helper for coverage assertions in the proof diagnostic.
pub fn distinct_classes(ports: &[ClassifiedPort]) -> HashSet<&'static str> {
    ports.iter().map(|p| p.class.label()).collect()
}
