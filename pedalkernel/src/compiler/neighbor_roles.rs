//! Connectivity-based neighbour-role inference and compile-time completeness
//! validation for active devices.
//!
//! # What this is (and is NOT)
//!
//! Stage boundaries are currently set by a role-blind topology heuristic that
//! can, for example, evict a tube's plate-load resistor and collapse a
//! multi-stage amp. The durable fix is to let components declare — per terminal
//! — what *neighbours* they need (see [`Component::terminal_requirements`] and
//! [`NeighborRole`]), so a later "arbitration" phase can set boundaries by
//! circuit meaning rather than topology shape.
//!
//! **This module builds only the declaration-driven inference + a completeness
//! diagnostic.** It does NOT change boundary formation, stage ordering, or
//! arbitration — those are a separate later phase that will *consume* these
//! roles. The only runtime-visible behaviour added here is a compile-time
//! completeness error, which is gated to be false-positive-free on the entire
//! working `.pedal` corpus.
//!
//! # Role inference rules (per active terminal, on the compiled netlist)
//!
//! For an active device terminal node, each incident passive (or transformer /
//! active) neighbour edge is classified by a directed, rail-aware walk:
//!
//! - **[`NeighborRole::Load`]** — the neighbour develops the device's output:
//!   a passive path from the far side reaches a supply rail (vcc / named
//!   supply), OR the neighbour edge is a transformer winding, OR the far side
//!   is another active device's terminal acting as the load. Recognised on an
//!   *output* terminal (plate/cathode, collector/emitter, drain/source).
//! - **[`NeighborRole::Signal`]** — a series element whose far side reaches
//!   ANOTHER active device's INPUT terminal (grid / base / gate) by a passive
//!   path. This is the future stage boundary / ordering edge.
//! - **[`NeighborRole::Ref`]** — DC operating-point / AC-grounded support: the
//!   far side reaches ground or an AC-ground (supply rail or large-cap bypass).
//!
//! Inference operates on the netlist and is INDEPENDENT of stage formation.

use std::collections::{HashSet, VecDeque};

use super::component::{Cardinality, Component, EdgeKind, NeighborRole};
use super::graph::{CircuitGraph, NodeId};

/// A completeness error: an active device terminal is missing a `Required`
/// neighbour role.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct CompletenessError {
    /// Component id (e.g. "V1").
    pub comp_id: String,
    /// Human-readable device type tag (e.g. "triode").
    pub type_tag: &'static str,
    /// The missing role.
    pub role: NeighborRole,
    /// One-line fix hint, already device-specialised where useful.
    pub hint: String,
}

impl CompletenessError {
    /// Render as a nice, single-line compile error message.
    ///
    /// DSL `PedalDef` carries no source spans (Pin / ComponentDef / NetDef
    /// have no line/column), so the message names the component + terminal/role
    /// instead of a `file:line` location.
    pub(super) fn message(&self) -> String {
        format!(
            "{} ({}): no {} found — {}",
            self.comp_id,
            self.type_tag,
            self.role.label(),
            self.hint
        )
    }
}

/// Set of NodeIds that are AC-equivalent to ground (gnd + supplies + bypasses).
fn is_ac_ground(graph: &CircuitGraph, node: NodeId) -> bool {
    node == graph.gnd_node || graph.ac_ground_nodes.contains(&node)
}

/// True if `node` is a supply rail (vcc or a named secondary supply).
fn is_supply_rail(graph: &CircuitGraph, node: NodeId) -> bool {
    node == graph.vcc_node || graph.supply_nodes.contains(&node)
}

/// All nodes that are a terminal pin of SOME active (gain / requirement-
/// declaring) device. These are barriers for the rail-aware load walk and the
/// active-input signal walk: a load develops directly off a terminal through
/// passives to a rail — it must not route *through another active device's
/// terminal* (e.g. through a neighbouring tube's plate) to find the rail.
fn active_terminal_nodes(graph: &CircuitGraph) -> HashSet<NodeId> {
    let mut set = HashSet::new();
    for comp in graph.components.iter() {
        if !comp.kind.is_gain_device() && comp.kind.terminal_requirements().is_empty() {
            continue;
        }
        for pin in comp.kind.pin_config().valid_pins {
            let key = format!("{}.{}", comp.id, pin);
            if let Some(&n) = graph.node_names.get(&key) {
                set.insert(n);
            }
        }
    }
    set
}

/// Passive (Linear/Reactive) rail-blocked walk: starting from `start`, can we
/// reach a supply rail without passing through ground / AC-ground and without
/// crossing a nonlinear / VCVS / VCCS / behavioural element OR routing through
/// another active device's terminal node?
///
/// This mirrors the `passive_path_exists` traversal in `spqr_build.rs` but
/// targets the rail *set* rather than a single node, treats ground / AC-ground
/// as dead ends (a resistor to ground is a Ref, not a Load), and treats other
/// active terminals as barriers (`active_terms`).
fn passive_reaches_supply_rail(
    graph: &CircuitGraph,
    start: NodeId,
    active_terms: &HashSet<NodeId>,
) -> bool {
    if is_supply_rail(graph, start) {
        return true;
    }
    let mut queue: VecDeque<NodeId> = VecDeque::new();
    let mut visited: HashSet<NodeId> = HashSet::new();
    visited.insert(start);
    queue.push_back(start);

    while let Some(node) = queue.pop_front() {
        for (eidx, e) in graph.edges.iter().enumerate() {
            let next = if e.node_a == node {
                e.node_b
            } else if e.node_b == node {
                e.node_a
            } else {
                continue;
            };
            // Only walk passive series elements.
            match graph.effective_edge_kind(eidx) {
                EdgeKind::Linear | EdgeKind::Reactive => {}
                EdgeKind::Vcvs
                | EdgeKind::Vccs
                | EdgeKind::Behavioral
                | EdgeKind::Nonlinear => continue,
            }
            if is_supply_rail(graph, next) {
                return true;
            }
            // Ground / AC-ground is a dead end for a load search.
            if next == graph.gnd_node || is_ac_ground(graph, next) {
                continue;
            }
            // Do not route a load through another active device's terminal.
            if next != start && active_terms.contains(&next) {
                continue;
            }
            if visited.insert(next) {
                queue.push_back(next);
            }
        }
    }
    false
}

/// True if `node` is the terminal of *some other* active (gain) device — i.e.
/// the far side of a neighbour is another active stage acting as the load
/// (direct-coupled / active-load topologies).
fn node_is_other_active_terminal(
    graph: &CircuitGraph,
    node: NodeId,
    self_comp_idx: usize,
) -> bool {
    for (idx, comp) in graph.components.iter().enumerate() {
        if idx == self_comp_idx {
            continue;
        }
        if !comp.kind.is_gain_device() && comp.kind.terminal_requirements().is_empty() {
            continue;
        }
        for pin in comp.kind.pin_config().valid_pins {
            let key = format!("{}.{}", comp.id, pin);
            if let Some(&pin_node) = graph.node_names.get(&key) {
                if pin_node == node {
                    return true;
                }
            }
        }
    }
    false
}

/// True if `edge_idx` is a transformer winding edge.
fn edge_is_transformer(graph: &CircuitGraph, edge_idx: usize) -> bool {
    let comp = &graph.components[graph.edges[edge_idx].comp_idx];
    comp.kind.is_transformer()
}

/// Does `term_node` (an output terminal of the active device at `self_comp_idx`)
/// have a Load neighbour?
///
/// A Load is recognised when an incident neighbour:
///   1. is a transformer winding (transformer-coupled / output-transformer load), OR
///   2. directly connects to a supply rail, OR
///   3. is a passive series element whose far side reaches a supply rail
///      (resistor/inductor/choke plate-load to B+), OR
///   4. its far side is another active device's terminal (active / direct-coupled load).
fn terminal_has_load(
    graph: &CircuitGraph,
    self_comp_idx: usize,
    term_node: NodeId,
    active_terms: &HashSet<NodeId>,
) -> bool {
    // The output terminal sitting directly on a supply rail is itself a valid
    // "load on an output terminal" arrangement only for the *other* output
    // (e.g. cathode-follower: plate straight to B+). We treat the plate-on-rail
    // as a degenerate case handled by the sibling terminal carrying the load,
    // so we do not early-return here.
    for (eidx, e) in graph.edges.iter().enumerate() {
        let far = if e.node_a == term_node {
            e.node_b
        } else if e.node_b == term_node {
            e.node_a
        } else {
            continue;
        };
        // Skip the device's own active/nonlinear edge.
        if e.comp_idx == self_comp_idx {
            continue;
        }
        // 1. Transformer winding directly on this terminal → Load.
        if edge_is_transformer(graph, eidx) {
            return true;
        }
        // Only passive series elements develop a resistive/reactive load.
        let kind = graph.effective_edge_kind(eidx);
        let is_passive_edge = matches!(kind, EdgeKind::Linear | EdgeKind::Reactive);

        // 2 + 3. Passive element whose far side reaches a supply rail.
        if is_passive_edge
            && (is_supply_rail(graph, far)
                || passive_reaches_supply_rail(graph, far, active_terms))
        {
            return true;
        }
        // 4. Far side is another active device's terminal → active/direct load.
        if node_is_other_active_terminal(graph, far, self_comp_idx) {
            return true;
        }
    }
    false
}

/// True if this device is a FET (JFET/MOSFET) used as a controlled variable
/// resistor rather than a common-source gain stage. Recognised two ways:
///   1. its own drain–source edge has been RESOLVED to Linear/Reactive
///      (envelope-driven `vgs`, handled by `resolve_components_by`), OR
///   2. its `gate` / `vgs` node is driven by a modulation source's `.out`
///      (LFO or envelope follower) — the netlist unions those pins to one node.
///
/// Such a device is a passive controlled R (e.g. Phase 90 allpass JFETs whose
/// drain feeds an op-amp virtual ground with no drain-load to a rail), so it
/// carries no Load requirement. Key config-tolerance carve-out surfaced by the
/// corpus gate.
fn is_resolved_variable_resistor(graph: &CircuitGraph, comp_idx: usize) -> bool {
    let comp = &graph.components[comp_idx];
    if !(comp.kind.is_jfet() || comp.kind.is_mosfet()) {
        return false;
    }
    // 1. Own edge already resolved to a passive kind.
    for (eidx, e) in graph.edges.iter().enumerate() {
        if e.comp_idx == comp_idx
            && matches!(
                graph.effective_edge_kind(eidx),
                EdgeKind::Linear | EdgeKind::Reactive
            )
        {
            return true;
        }
    }
    // 2. Gate / vgs driven by a modulation source (LFO / envelope follower).
    let mut control_nodes: HashSet<NodeId> = HashSet::new();
    for &pin in &["gate", "vgs"] {
        let key = format!("{}.{}", comp.id, pin);
        if let Some(&n) = graph.node_names.get(&key) {
            control_nodes.insert(n);
        }
    }
    if control_nodes.is_empty() {
        return false;
    }
    for other in graph.components.iter() {
        if !other.kind.is_modulation_source() {
            continue;
        }
        let out_key = format!("{}.out", other.id);
        if let Some(&out_node) = graph.node_names.get(&out_key) {
            if control_nodes.contains(&out_node) {
                return true;
            }
        }
    }
    false
}

/// True if `term_node` is directly connected (one edge) to a supply rail.
fn terminal_on_rail(graph: &CircuitGraph, term_node: NodeId) -> bool {
    if is_supply_rail(graph, term_node) {
        return true;
    }
    for e in graph.edges.iter() {
        let far = if e.node_a == term_node {
            e.node_b
        } else if e.node_b == term_node {
            e.node_a
        } else {
            continue;
        };
        if is_supply_rail(graph, far) {
            return true;
        }
    }
    false
}

/// True if `term_node` develops a follower-style load: a passive series element
/// (resistor / inductor / cap) whose far side is ground / AC-ground. In a
/// cathode / emitter / source follower the output is developed across this
/// element even though it returns to ground.
fn terminal_has_passive_to_ground(
    graph: &CircuitGraph,
    self_comp_idx: usize,
    term_node: NodeId,
) -> bool {
    for (eidx, e) in graph.edges.iter().enumerate() {
        if e.comp_idx == self_comp_idx {
            continue;
        }
        let far = if e.node_a == term_node {
            e.node_b
        } else if e.node_b == term_node {
            e.node_a
        } else {
            continue;
        };
        let is_passive_edge = matches!(
            graph.effective_edge_kind(eidx),
            EdgeKind::Linear | EdgeKind::Reactive
        );
        if is_passive_edge && (far == graph.gnd_node || is_ac_ground(graph, far)) {
            return true;
        }
    }
    false
}

/// Config-tolerance: recognise a follower (cathode / emitter / source) where one
/// output terminal sits on a supply rail and a sibling output terminal develops
/// its load to ground. `load_terms` are the Required-Load terminals (output
/// candidates) that are present in the netlist with their resolved node ids.
fn is_follower_configuration(
    graph: &CircuitGraph,
    self_comp_idx: usize,
    load_terms: &[(&'static str, NodeId)],
) -> bool {
    let any_on_rail = load_terms
        .iter()
        .any(|&(_, n)| terminal_on_rail(graph, n));
    if !any_on_rail {
        return false;
    }
    load_terms
        .iter()
        .any(|&(_, n)| terminal_has_passive_to_ground(graph, self_comp_idx, n))
}

/// Build a device-specialised fix hint for a missing role.
fn load_hint(type_tag: &str) -> String {
    match type_tag {
        "triode" | "variable-mu triode" => {
            "a common-cathode stage needs a plate-load resistor to a supply rail, \
             or wire it as a cathode follower (load on the cathode)."
                .to_string()
        }
        "pentode" => {
            "a pentode stage needs a plate-load (resistor, choke, or output \
             transformer) to a supply rail, or a cathode-follower load."
                .to_string()
        }
        "NPN transistor" | "PNP transistor" => {
            "a common-emitter stage needs a collector-load resistor to a supply \
             rail, or wire it as an emitter follower (load on the emitter)."
                .to_string()
        }
        _ => {
            // JFET / MOSFET and any future gain device.
            "a common-source stage needs a drain-load resistor to a supply rail, \
             or wire it as a source follower (load on the source)."
                .to_string()
        }
    }
}

/// Infer neighbour roles for every active device and report completeness errors
/// for any `Required` role that is missing.
///
/// Config tolerance: a `Load` declared `Required` on more than one terminal of
/// the same device is satisfied if ANY of those terminals carries a Load. This
/// is what lets common-cathode (plate-load) AND cathode-follower (cathode-load)
/// both validate without a false positive.
pub(super) fn check_completeness(graph: &CircuitGraph) -> Vec<CompletenessError> {
    let mut errors = Vec::new();
    let active_terms = active_terminal_nodes(graph);

    for (comp_idx, comp) in graph.components.iter().enumerate() {
        let reqs = comp.kind.terminal_requirements();
        if reqs.is_empty() {
            continue;
        }
        // Config tolerance: a FET resolved to a variable resistor (modulated
        // gate, e.g. phaser allpass JFETs) is a passive controlled R, not a
        // gain stage — it has no Load requirement.
        if is_resolved_variable_resistor(graph, comp_idx) {
            continue;
        }
        let type_tag = comp.kind.type_tag();

        // ── Required Load: satisfied if ANY declared output terminal has one ──
        // Collect the terminals that declare a Required Load, and whether any of
        // them is actually present in the netlist + carries a load.
        let mut load_required = false;
        let mut load_satisfied = false;
        let mut load_terms: Vec<(&'static str, NodeId)> = Vec::new();
        for (term, term_reqs) in &reqs {
            let wants_required_load = term_reqs.iter().any(|r| {
                r.role == NeighborRole::Load && r.card == Cardinality::Required
            });
            if !wants_required_load {
                continue;
            }
            load_required = true;
            let key = format!("{}.{}", comp.id, term);
            if let Some(&node) = graph.node_names.get(&key) {
                load_terms.push((term, node));
                if terminal_has_load(graph, comp_idx, node, &active_terms) {
                    load_satisfied = true;
                    break;
                }
            }
        }
        // Config tolerance: cathode / emitter / source follower — one output
        // terminal on a rail, the sibling output develops its load to ground.
        if load_required && !load_satisfied && is_follower_configuration(graph, comp_idx, &load_terms) {
            load_satisfied = true;
        }
        if load_required && !load_satisfied {
            errors.push(CompletenessError {
                comp_id: comp.id.clone(),
                type_tag,
                role: NeighborRole::Load,
                hint: load_hint(type_tag),
            });
        }

        // Ref / Signal requirements are declared Optional today (config
        // tolerance: valid circuits vary too much to hard-require them yet), so
        // they never produce a completeness error. They are documented in
        // `terminal_requirements()` and consumed by the future arbitration
        // phase. No other Required roles exist at present.
    }

    errors
}

/// Run the completeness pass and return a combined compile error string if any
/// device is incomplete. Returns `Ok(())` when the circuit is complete.
pub(super) fn validate_completeness(graph: &CircuitGraph) -> Result<(), String> {
    let errors = check_completeness(graph);
    if errors.is_empty() {
        return Ok(());
    }
    let mut msg = String::from("circuit completeness error(s):");
    for e in &errors {
        msg.push_str("\n  ");
        msg.push_str(&e.message());
    }
    Err(msg)
}

// ═══════════════════════════════════════════════════════════════════════════
// Public introspection API (for tests + the future arbitration phase)
// ═══════════════════════════════════════════════════════════════════════════

/// An inferred role for one neighbour of one active-device terminal.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InferredNeighbor {
    /// Owning active device id (e.g. "V1").
    pub comp_id: String,
    /// Terminal name (e.g. "plate").
    pub terminal: &'static str,
    /// Neighbour component id across the incident edge (e.g. "R_plate"),
    /// or `None` for a connection straight to a reserved net / rail.
    pub neighbor_id: Option<String>,
    /// Inferred role of the neighbour.
    pub role: NeighborRole,
}

/// Classify the neighbour role of a single incident edge from a terminal node.
fn classify_neighbor(
    graph: &CircuitGraph,
    self_comp_idx: usize,
    far: NodeId,
    edge_idx: usize,
    active_terms: &HashSet<NodeId>,
) -> NeighborRole {
    // Transformer winding or another active device's terminal across the edge
    // is a Load (transformer-coupled / active / direct-coupled load).
    if edge_is_transformer(graph, edge_idx)
        || node_is_other_active_terminal(graph, far, self_comp_idx)
    {
        return NeighborRole::Load;
    }
    // Far side reaches a supply rail through passives → Load.
    if is_supply_rail(graph, far) || passive_reaches_supply_rail(graph, far, active_terms) {
        return NeighborRole::Load;
    }
    // Far side reaches another active device's INPUT terminal → Signal.
    if passive_reaches_active_input(graph, far, self_comp_idx) {
        return NeighborRole::Signal;
    }
    // Otherwise it is an AC-grounded / DC-bias reference.
    NeighborRole::Ref
}

/// Passive walk: does `start` reach the INPUT terminal (grid / base / gate) of
/// some other active device, without crossing a rail or another active edge?
fn passive_reaches_active_input(
    graph: &CircuitGraph,
    start: NodeId,
    self_comp_idx: usize,
) -> bool {
    // Collect input-terminal nodes of every OTHER active device.
    let mut input_nodes: HashSet<NodeId> = HashSet::new();
    for (idx, comp) in graph.components.iter().enumerate() {
        if idx == self_comp_idx {
            continue;
        }
        if !comp.kind.is_gain_device() && comp.kind.terminal_requirements().is_empty() {
            continue;
        }
        for &pin in &["grid", "base", "gate"] {
            let key = format!("{}.{}", comp.id, pin);
            if let Some(&n) = graph.node_names.get(&key) {
                input_nodes.insert(n);
            }
        }
    }
    if input_nodes.is_empty() {
        return false;
    }
    if input_nodes.contains(&start) {
        return true;
    }

    let mut queue: VecDeque<NodeId> = VecDeque::new();
    let mut visited: HashSet<NodeId> = HashSet::new();
    visited.insert(start);
    queue.push_back(start);
    while let Some(node) = queue.pop_front() {
        for (eidx, e) in graph.edges.iter().enumerate() {
            let next = if e.node_a == node {
                e.node_b
            } else if e.node_b == node {
                e.node_a
            } else {
                continue;
            };
            match graph.effective_edge_kind(eidx) {
                EdgeKind::Linear | EdgeKind::Reactive => {}
                _ => continue,
            }
            if input_nodes.contains(&next) {
                return true;
            }
            if next == graph.gnd_node
                || is_supply_rail(graph, next)
                || is_ac_ground(graph, next)
            {
                continue;
            }
            if visited.insert(next) {
                queue.push_back(next);
            }
        }
    }
    false
}

/// Component id owning an edge's far terminal across the given edge.
fn neighbor_comp_id(graph: &CircuitGraph, edge_idx: usize) -> Option<String> {
    let comp = &graph.components[graph.edges[edge_idx].comp_idx];
    Some(comp.id.clone())
}

/// Infer neighbour roles for every active device terminal in `pedal`.
///
/// This is the public, behaviour-neutral introspection entry point. It compiles
/// the netlist into a circuit graph and classifies each active-device terminal's
/// immediate neighbours as [`NeighborRole::Load`] / [`NeighborRole::Ref`] /
/// [`NeighborRole::Signal`]. Used by tests today and by the future boundary
/// arbitration phase.
pub fn infer_neighbor_roles(pedal: &crate::dsl::PedalDef) -> Vec<InferredNeighbor> {
    let graph = CircuitGraph::from_pedal(pedal);
    let active_terms = active_terminal_nodes(&graph);
    let mut out = Vec::new();
    for (comp_idx, comp) in graph.components.iter().enumerate() {
        let reqs = comp.kind.terminal_requirements();
        if reqs.is_empty() {
            continue;
        }
        for (term, _) in &reqs {
            let key = format!("{}.{}", comp.id, term);
            let Some(&term_node) = graph.node_names.get(&key) else {
                continue;
            };
            for (eidx, e) in graph.edges.iter().enumerate() {
                let far = if e.node_a == term_node {
                    e.node_b
                } else if e.node_b == term_node {
                    e.node_a
                } else {
                    continue;
                };
                if e.comp_idx == comp_idx {
                    continue; // skip the device's own edge
                }
                let role = classify_neighbor(&graph, comp_idx, far, eidx, &active_terms);
                out.push(InferredNeighbor {
                    comp_id: comp.id.clone(),
                    terminal: term,
                    neighbor_id: neighbor_comp_id(&graph, eidx),
                    role,
                });
            }
        }
    }
    out
}

/// Public completeness errors for a parsed pedal (returns the rendered
/// messages). `Ok` with an empty vec means the circuit is complete.
pub fn completeness_errors(pedal: &crate::dsl::PedalDef) -> Vec<String> {
    let graph = CircuitGraph::from_pedal(pedal);
    check_completeness(&graph)
        .iter()
        .map(|e| e.message())
        .collect()
}
