//! Boundary-load bindings: compile-time ANALYSIS of what hangs electrically
//! downstream of a stage boundary, separated from the POLICY of what the
//! compiler does about it.
//!
//! WHY: downstream impedance never reflects back into an upstream stage's
//! scattering — stages solve against an open circuit. Proven on the BA283:
//! ngspice with RL=1G reproduces the unloaded WDF to 3–4 decimals (the engine
//! is exact at the topology it simulates; the topology was wrong), and the
//! loaded golden's −16.1 dB THD is Q1/Q2 cutoff clipping from load-current
//! demand the open-circuit topology never delivered. Fusing the trailing
//! `Cout→RL` output network into the solved MNA closed it: LEVEL −0.05 dB,
//! ΔTHD 0.4 dB, tilt 0.79 dB (servo-off default).
//!
//! This module owns the two halves that used to be entangled in
//! `spqr_build::trailing_output_load_group_for_fusion`:
//!
//! * [`analyze_trailing_output_load`] — pure SHAPE analysis of the trailing
//!   output flow group. No device gate, no DC solve, no env check: it answers
//!   "what hangs off this stage's output boundary?", producing a
//!   [`BoundaryLoadModel`].
//! * [`gate_ba283_fusion`] — the POLICY gate deciding whether the analyzed
//!   load may be fused into the upstream stage's general-MNA build. Exactly
//!   the consolidated dc_qpoint seed + servo shape (`apply_bjt_dc_qpoint`):
//!   every NL device in the target is a true 3-terminal BJT, there are at
//!   least two, `solve_bjt_group_dc_qpoint` produces the op-point seed, and
//!   `PK_OUTPUT_FUSE_DISABLE` is not set. Declines carry a static reason
//!   string for the decision table.
//!
//! The outcome of analysis + policy is a [`LoadDisposition`], and every
//! analyzed boundary (fused or declined) is recorded on
//! `CompiledPedal::boundary_loads` as a serializable
//! [`BoundaryLoadBinding`](pedalkernel_rt::processor::BoundaryLoadBinding)
//! (component ids + values; compile-time edge indices never leave the
//! compiler) so dashboards can surface unloaded output boundaries without
//! recompiling.
//!
//! SCOPE (deliberate): these are planner-layer types. The neutral network
//! vocabulary (`LinearMultiportNetwork`, `boundary_math`) stays load-free —
//! bindings and drive policy live at the stage-planning layer, mirroring the
//! `StageBinding`/`ControlTarget` precedent.

use pedalkernel_rt::processor::{
    BoundaryLoadBinding, BoundaryLoadDisposition, BoundaryLoadSummary,
};

use super::boundary_rules::DelayedCutSet;
use super::classify::NonlinearKind;
use super::component::EdgeKind;
use super::graph::{CircuitGraph, NodeId};
use super::signal_flow::FlowGroup;

/// Disposition reason: analysis found no trailing output-load group to fuse
/// (boundary is open, or the boundary shape is not recognized).
pub(super) const REASON_NO_TRAILING_LOAD: &str = "analysis:no-trailing-output-load";

/// Disposition reason: the analyzed shape is recognized (visible in the
/// table) but no fusion policy is validated for it — left open, safe in
/// production (lkf1.3 scope guard). Today: `PassiveNetwork` trailing groups
/// (tone stacks bundled with the output network — tube_screamer, big_muff,
/// proco_rat, blues_driver).
pub(super) const REASON_UNVALIDATED_SHAPE: &str = "gate:unvalidated-shape";

/// Disposition reason: a `PotDividerAtOut` load hangs off a boundary node
/// that is NOT DC-isolated within the target group (some target edge at the
/// node is neither a capacitor nor a grounded fixed resistor). Fusing would
/// shift the compile-time DC op-point the dc_qpoint seed was solved without,
/// so the policy declines.
pub(super) const REASON_DC_COUPLED_POT: &str = "gate:dc-coupled-pot-load";

/// ANALYSIS: what hangs electrically downstream of a stage boundary node.
///
/// Compiler-internal — `edges` are indices into `graph.edges` and are only
/// meaningful during compilation. [`BoundaryLoadModel::summarize`] flattens a
/// model into the serializable [`BoundaryLoadSummary`] stored on
/// `CompiledPedal`.
#[derive(Debug, Clone, PartialEq)]
pub(super) enum BoundaryLoadModel {
    /// Nothing recognized downstream (or analysis declined) — boundary is open.
    #[allow(dead_code)] // emitted by future boundary analyses; summary-side used today
    Unloaded,
    /// Grounded resistive load(s) directly at the boundary node.
    #[allow(dead_code)] // future widening (direct fixed-R loads, no cap)
    GroundedResistive { r_total: f64, edges: Vec<usize> },
    /// One series coupling cap into grounded resistive load(s) — the classic
    /// `Cout→RL` output network (the BA283 shape).
    SeriesCapIntoLoad {
        c: f64,
        r_total: f64,
        edges: Vec<usize>,
    },
    /// General passive network — recognized for table visibility (the
    /// tone-stack-bundled trailing groups of tube_screamer / big_muff /
    /// proco_rat / blues_driver); no fusion policy consumes it
    /// ([`REASON_UNVALIDATED_SHAPE`]).
    PassiveNetwork { edges: Vec<usize> },
    /// One series coupling cap into a POT-bearing grounded output network at
    /// the global `out` (lkf1.3): pot rheostat to ground, or pot divider
    /// whose wiper is the `out` tap (directly or through one series
    /// resistor), plus optional fixed grounded resistors at the load node.
    SeriesCapIntoPotLoad {
        c: f64,
        pot_id: String,
        r_pot: f64,
        /// Parallel fixed grounded resistance at the load node (`None` when
        /// the pot is the only grounded element).
        r_fixed: Option<f64>,
        edges: Vec<usize>,
    },
    /// A 3-terminal pot divider hanging DIRECTLY off the boundary node (no
    /// coupling cap in the trailing group): track top at the boundary node,
    /// wiper is the global `out` tap, track bottom grounded — the
    /// fulltone_ocd `Volume` shape.
    PotDividerAtOut {
        pot_id: String,
        r_pot: f64,
        edges: Vec<usize>,
    },
}

impl BoundaryLoadModel {
    /// All compile-time edge indices of the modeled load network.
    pub(super) fn edges(&self) -> &[usize] {
        match self {
            BoundaryLoadModel::Unloaded => &[],
            BoundaryLoadModel::GroundedResistive { edges, .. } => edges,
            BoundaryLoadModel::SeriesCapIntoLoad { edges, .. } => edges,
            BoundaryLoadModel::PassiveNetwork { edges } => edges,
            BoundaryLoadModel::SeriesCapIntoPotLoad { edges, .. } => edges,
            BoundaryLoadModel::PotDividerAtOut { edges, .. } => edges,
        }
    }

    /// Flatten to the serializable dashboard summary: kind + values +
    /// sorted/deduped component id strings (NOT edge indices).
    pub(super) fn summarize(&self, graph: &CircuitGraph) -> BoundaryLoadSummary {
        let component_ids = |edges: &[usize]| -> Vec<String> {
            let mut ids: Vec<String> = edges
                .iter()
                .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.clone())
                .collect();
            ids.sort();
            ids.dedup();
            ids
        };
        match self {
            BoundaryLoadModel::Unloaded => BoundaryLoadSummary::Unloaded,
            BoundaryLoadModel::GroundedResistive { r_total, edges } => {
                BoundaryLoadSummary::GroundedResistive {
                    r_total: *r_total,
                    component_ids: component_ids(edges),
                }
            }
            BoundaryLoadModel::SeriesCapIntoLoad { c, r_total, edges } => {
                BoundaryLoadSummary::SeriesCapIntoLoad {
                    c: *c,
                    r_total: *r_total,
                    component_ids: component_ids(edges),
                }
            }
            BoundaryLoadModel::PassiveNetwork { edges } => BoundaryLoadSummary::PassiveNetwork {
                component_ids: component_ids(edges),
            },
            BoundaryLoadModel::SeriesCapIntoPotLoad {
                c,
                pot_id,
                r_pot,
                r_fixed,
                edges,
            } => BoundaryLoadSummary::SeriesCapIntoPotLoad {
                c: *c,
                pot_id: pot_id.clone(),
                r_pot: *r_pot,
                r_fixed: *r_fixed,
                component_ids: component_ids(edges),
            },
            BoundaryLoadModel::PotDividerAtOut {
                pot_id,
                r_pot,
                edges,
            } => BoundaryLoadSummary::PotDividerAtOut {
                pot_id: pot_id.clone(),
                r_pot: *r_pot,
                component_ids: component_ids(edges),
            },
        }
    }
}

/// POLICY OUTCOME: what the compiler did about the analyzed load.
#[derive(Debug, Clone, PartialEq)]
pub(super) enum LoadDisposition {
    /// Load edges fused into the upstream stage's solved network; the
    /// standalone downstream stage was consumed (removed).
    FusedUpstream { consumed_group: usize },
    /// Nothing done — boundary left open, with the reason (gate name / env
    /// opt-out / shape mismatch / no load found), for dashboards and triage.
    Unloaded { reason: &'static str },
    /// Transformer-SECONDARY load fused into the transformer's PRIMARY-side
    /// stage (the load edges join the solved MNA that carries the full
    /// transformer skeleton, so the primary feels `n² · Z_secondary`); the
    /// standalone downstream load stage was consumed.
    ReflectedThroughTransformer {
        consumed_group: usize,
        /// `n = N_primary / N_secondary`.
        turns_ratio: f64,
        /// Load resistance as seen from the primary: `n² · r_total`.
        r_reflected: f64,
    },
}

impl LoadDisposition {
    /// The flow-group index consumed by BA283-class output fusion, if any.
    ///
    /// Deliberately `None` for [`LoadDisposition::ReflectedThroughTransformer`]:
    /// the transformer-reflection call site (spqr_build non-feedback arm) owns
    /// its own consumption bookkeeping (skip-set + built-stage removal), while
    /// this accessor feeds only the feedback-arm C1 fusion consumption.
    pub(super) fn fused_group(&self) -> Option<usize> {
        match self {
            LoadDisposition::FusedUpstream { consumed_group } => Some(*consumed_group),
            LoadDisposition::Unloaded { .. } => None,
            LoadDisposition::ReflectedThroughTransformer { .. } => None,
        }
    }

    /// Flatten to the serializable dashboard summary.
    pub(super) fn summarize(&self) -> BoundaryLoadDisposition {
        match self {
            LoadDisposition::FusedUpstream { .. } => BoundaryLoadDisposition::FusedUpstream,
            LoadDisposition::Unloaded { reason } => BoundaryLoadDisposition::Unloaded {
                reason: (*reason).to_string(),
            },
            LoadDisposition::ReflectedThroughTransformer {
                turns_ratio,
                r_reflected,
                ..
            } => BoundaryLoadDisposition::ReflectedThroughTransformer {
                turns_ratio: *turns_ratio,
                r_reflected: *r_reflected,
            },
        }
    }
}

/// Result of [`analyze_trailing_output_load`]: the trailing output flow group,
/// the boundary node the load hangs off, and the load model.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct TrailingOutputLoad {
    /// Index (into the flow-group slice) of the trailing passive output group.
    pub group: usize,
    /// The TARGET-group node the load network attaches to (the coupling cap's
    /// stage-side node) — the upstream stage's output boundary.
    pub boundary_node: NodeId,
    /// What hangs downstream of `boundary_node`.
    pub model: BoundaryLoadModel,
}

/// ANALYSIS ONLY — find the TRAILING OUTPUT GROUP downstream of `target_gi`'s
/// output boundary: the passive flow group carrying the amplifier's load at
/// the global `out`.
///
/// Recognized shapes (per-shape gated predicates; catalog: lkf1.3 fleet
/// sweep 2026-07-07):
///
/// * [`BoundaryLoadModel::SeriesCapIntoLoad`] — EXACTLY one coupling cap
///   bridging a target-group node to the global `out`, plus ≥1 grounded
///   fixed (non-pot) load resistors at `out` — nothing else (the BA283
///   shape; kept byte-identical);
/// * [`BoundaryLoadModel::SeriesCapIntoPotLoad`] — one coupling cap from a
///   target-group node into a load node carrying exactly one pot (rheostat
///   to ground, or divider whose wiper is `out` directly or through one
///   series fixed resistor) plus optional grounded fixed resistors;
/// * [`BoundaryLoadModel::PotDividerAtOut`] — the trailing group IS one
///   3-terminal pot: track top at a target-group node, wiper = `out`, track
///   bottom grounded (the fulltone_ocd `Volume` shape);
/// * [`BoundaryLoadModel::PassiveNetwork`] — fallback VISIBILITY model for a
///   richer passive trailing group at `out` (attached to the target, with at
///   least one grounded shunt element). Policy never fuses it
///   ([`REASON_UNVALIDATED_SHAPE`]).
///
/// Candidate trailing groups are passive (no active edges), have ≥2 edges,
/// touch the global `out`, and contain no broker-cut (Phase 2a
/// `DelayedCutSet`) edges: never fuse across a delayed-coupling boundary.
/// Fusable shapes are searched over ALL candidates before the
/// `PassiveNetwork` fallback so visibility never shadows a fusable load.
///
/// No device-shape gate, no DC solve, no env check here — that is POLICY
/// ([`gate_fusion`]). Returns `None` when the boundary is open or no shape is
/// recognized.
pub(super) fn analyze_trailing_output_load(
    target_gi: usize,
    groups: &[FlowGroup],
    graph: &CircuitGraph,
    cut_edges: &DelayedCutSet,
) -> Option<TrailingOutputLoad> {
    let target = &groups[target_gi];
    let out = graph.out_node;
    if out == graph.gnd_node || out == graph.vcc_node {
        return None;
    }

    // Target node set; the global `out` must NOT already be inside (there must
    // BE a trailing group to fuse).
    let mut target_nodes = std::collections::HashSet::new();
    for &eidx in &target.all_edges() {
        let e = &graph.edges[eidx];
        target_nodes.insert(e.node_a);
        target_nodes.insert(e.node_b);
    }
    if target_nodes.contains(&out) {
        return None;
    }

    // Candidate trailing groups: passive, ≥2 edges, touching the global
    // `out`, no broker-cut edges.
    let candidates: Vec<(usize, Vec<usize>)> = groups
        .iter()
        .enumerate()
        .filter_map(|(src_gi, source)| {
            if src_gi == target_gi || !source.active_edges.is_empty() {
                return None;
            }
            let edges = source.all_edges();
            if edges.len() < 2 {
                return None;
            }
            // Phase 2a: never fuse across a broker-cut boundary.
            if edges.iter().any(|eidx| cut_edges.cuts.contains_key(eidx)) {
                return None;
            }
            let touches_out = edges.iter().any(|&eidx| {
                let e = &graph.edges[eidx];
                e.node_a == out || e.node_b == out
            });
            touches_out.then_some((src_gi, edges))
        })
        .collect();

    // Pass 1: exact fusable shapes.
    for (src_gi, edges) in &candidates {
        if let Some((boundary_node, model)) =
            classify_series_cap_into_fixed_load(edges, &target_nodes, graph)
                .or_else(|| classify_series_cap_into_pot_load(edges, &target_nodes, graph))
                .or_else(|| classify_pot_divider_at_out(edges, &target_nodes, graph))
        {
            return Some(TrailingOutputLoad {
                group: *src_gi,
                boundary_node,
                model,
            });
        }
    }

    // Pass 2: visibility fallback — a richer passive network at `out`.
    for (src_gi, edges) in &candidates {
        if let Some((boundary_node, model)) = classify_passive_network(edges, &target_nodes, graph)
        {
            return Some(TrailingOutputLoad {
                group: *src_gi,
                boundary_node,
                model,
            });
        }
    }
    None
}

/// The classic BA283 `Cout→RL` shape: EXACTLY one coupling cap
/// (target-node → out) plus grounded fixed (non-pot) load resistor(s) at
/// `out`, nothing else. Logic kept byte-identical to the pre-lkf1.3 scan.
fn classify_series_cap_into_fixed_load(
    edges: &[usize],
    target_nodes: &std::collections::HashSet<NodeId>,
    graph: &CircuitGraph,
) -> Option<(NodeId, BoundaryLoadModel)> {
    let out = graph.out_node;
    let is_gnd = |n: NodeId| n == graph.gnd_node || graph.ac_ground_nodes.contains(&n);

    let mut bridge: Option<(NodeId, f64)> = None; // (stage-side node, C)
    let mut load_conductance = 0.0f64;
    let mut n_loads = 0usize;
    for &eidx in edges {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        let touches_out = e.node_a == out || e.node_b == out;
        match graph.effective_edge_kind(eidx) {
            // The output coupling cap: `out` on one side, a target-group
            // node on the other. Exactly one.
            EdgeKind::Reactive
                if comp.kind.capacitance().is_some() && touches_out && bridge.is_none() =>
            {
                let interior = if e.node_a == out { e.node_b } else { e.node_a };
                if target_nodes.contains(&interior) {
                    bridge = Some((interior, comp.kind.capacitance().unwrap_or(0.0)));
                } else {
                    return None;
                }
            }
            // A grounded fixed load resistor at `out` (not a pot).
            EdgeKind::Linear
                if comp.kind.resistance().is_some()
                    && !comp.kind.is_pot()
                    && touches_out
                    && (is_gnd(e.node_a) || is_gnd(e.node_b)) =>
            {
                n_loads += 1;
                if let Some(r) = comp.kind.resistance() {
                    if r > 0.0 {
                        load_conductance += 1.0 / r;
                    }
                }
            }
            _ => {
                return None;
            }
        }
    }
    if n_loads < 1 {
        return None;
    }
    let (boundary_node, c) = bridge?;
    let r_total = if load_conductance > 0.0 {
        1.0 / load_conductance
    } else {
        f64::INFINITY
    };
    Some((
        boundary_node,
        BoundaryLoadModel::SeriesCapIntoLoad {
            c,
            r_total,
            edges: edges.to_vec(),
        },
    ))
}

/// Pot track resistance for the component behind a graph edge, `None` if the
/// component is not a pot.
fn pot_track_resistance(graph: &CircuitGraph, comp_idx: usize) -> Option<f64> {
    let comp = &graph.components[comp_idx];
    comp.kind
        .as_any()
        .downcast_ref::<super::components::Potentiometer>()
        .map(|p| p.max_r)
}

/// True when any edge OUTSIDE `group_edges` touches one of `nodes` — the
/// trailing network's interior must be exclusive to the group, otherwise the
/// one-port load model (and its consumption by fusion) would orphan a
/// neighbor. Applied to the NEW pot shapes only; the BA283 fixed-R scan is
/// kept byte-identical.
fn interior_nodes_shared_outside(
    group_edges: &[usize],
    nodes: &[NodeId],
    graph: &CircuitGraph,
) -> bool {
    let edge_set: std::collections::HashSet<usize> = group_edges.iter().copied().collect();
    graph.edges.iter().enumerate().any(|(eidx, e)| {
        !edge_set.contains(&eidx) && (nodes.contains(&e.node_a) || nodes.contains(&e.node_b))
    })
}

/// `Cout → {pot (+ fixed Rs)}` — one coupling cap from a target-group node
/// into a load node, exactly one pot:
///
/// * rheostat: one pot edge `load_node ↔ gnd` with `load_node == out`
///   (wiper strapped to an end terminal);
/// * divider: two pot half-edges sharing the wiper node `W`, track top at
///   `load_node`, track bottom grounded, with `W == out` (tube_screamer /
///   big_muff sub-shape) or `W → out` through EXACTLY one series fixed
///   resistor (proco_rat / blues_driver sub-shape);
///
/// plus optional grounded fixed resistors at `load_node`, nothing else.
fn classify_series_cap_into_pot_load(
    edges: &[usize],
    target_nodes: &std::collections::HashSet<NodeId>,
    graph: &CircuitGraph,
) -> Option<(NodeId, BoundaryLoadModel)> {
    let out = graph.out_node;
    let is_gnd = |n: NodeId| n == graph.gnd_node || graph.ac_ground_nodes.contains(&n);

    let mut bridge: Option<(NodeId, NodeId, f64)> = None; // (stage-side, load node, C)
    let mut pot_comp: Option<usize> = None;
    let mut pot_edges: Vec<usize> = Vec::new();
    let mut fixed_gnd: Vec<(NodeId, f64)> = Vec::new(); // (hang node, R)
    let mut series_r: Vec<(NodeId, NodeId)> = Vec::new(); // non-grounded fixed Rs
    for &eidx in edges {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        match graph.effective_edge_kind(eidx) {
            EdgeKind::Reactive if comp.kind.capacitance().is_some() && bridge.is_none() => {
                // The coupling cap bridges a target node to the load node.
                let (t, l) = if target_nodes.contains(&e.node_a)
                    && !target_nodes.contains(&e.node_b)
                {
                    (e.node_a, e.node_b)
                } else if target_nodes.contains(&e.node_b) && !target_nodes.contains(&e.node_a) {
                    (e.node_b, e.node_a)
                } else {
                    return None;
                };
                if is_gnd(l) {
                    return None;
                }
                bridge = Some((t, l, comp.kind.capacitance().unwrap_or(0.0)));
            }
            EdgeKind::Linear if comp.kind.is_pot() => {
                match pot_comp {
                    None => pot_comp = Some(e.comp_idx),
                    Some(ci) if ci == e.comp_idx => {}
                    Some(_) => return None, // exactly one pot
                }
                pot_edges.push(eidx);
            }
            EdgeKind::Linear if comp.kind.resistance().is_some() => {
                if is_gnd(e.node_a) || is_gnd(e.node_b) {
                    let hang = if is_gnd(e.node_a) { e.node_b } else { e.node_a };
                    fixed_gnd.push((hang, comp.kind.resistance().unwrap_or(0.0)));
                } else {
                    series_r.push((e.node_a, e.node_b));
                }
            }
            _ => return None,
        }
    }

    let (boundary_node, load_node, c) = bridge?;
    let pot_ci = pot_comp?;
    let pot_id = graph.components[pot_ci].id.clone();
    let r_pot = pot_track_resistance(graph, pot_ci)?;

    // Pot topology: rheostat (1 edge) or divider (2 half-edges).
    let mut interior_nodes: Vec<NodeId> = vec![load_node];
    match pot_edges.len() {
        1 => {
            // Rheostat directly at `out`: load_node == out, pot to ground.
            let e = &graph.edges[pot_edges[0]];
            let ok = load_node == out
                && ((e.node_a == load_node && is_gnd(e.node_b))
                    || (e.node_b == load_node && is_gnd(e.node_a)));
            if !ok || !series_r.is_empty() {
                return None;
            }
        }
        2 => {
            // Divider: half-edges share the wiper node W.
            let e0 = &graph.edges[pot_edges[0]];
            let e1 = &graph.edges[pot_edges[1]];
            let w = [e0.node_a, e0.node_b]
                .into_iter()
                .find(|&n| n == e1.node_a || n == e1.node_b)?;
            let top0 = if e0.node_a == w { e0.node_b } else { e0.node_a };
            let top1 = if e1.node_a == w { e1.node_b } else { e1.node_a };
            // Track top at the load node, track bottom grounded.
            let track_ok =
                (top0 == load_node && is_gnd(top1)) || (top1 == load_node && is_gnd(top0));
            if !track_ok {
                return None;
            }
            if w == out {
                if !series_r.is_empty() {
                    return None;
                }
            } else {
                // Wiper reaches `out` through exactly one series fixed R.
                let [(sa, sb)] = series_r.as_slice() else {
                    return None;
                };
                let series_ok = (*sa == w && *sb == out) || (*sb == w && *sa == out);
                if !series_ok {
                    return None;
                }
            }
            interior_nodes.push(w);
        }
        _ => return None,
    }

    // Fixed grounded resistors must hang off the load node.
    let mut fixed_conductance = 0.0f64;
    for &(hang, r) in &fixed_gnd {
        if hang != load_node {
            return None;
        }
        if r > 0.0 {
            fixed_conductance += 1.0 / r;
        }
    }
    let r_fixed = (fixed_conductance > 0.0).then(|| 1.0 / fixed_conductance);

    // Exclusivity: the trailing network's interior (load node, wiper, out)
    // must not be shared with edges outside the group.
    if !interior_nodes.contains(&out) {
        interior_nodes.push(out);
    }
    if interior_nodes_shared_outside(edges, &interior_nodes, graph) {
        return None;
    }

    Some((
        boundary_node,
        BoundaryLoadModel::SeriesCapIntoPotLoad {
            c,
            pot_id,
            r_pot,
            r_fixed,
            edges: edges.to_vec(),
        },
    ))
}

/// The trailing group IS one 3-terminal pot divider: track top at a
/// target-group node, wiper = the global `out`, track bottom grounded (the
/// fulltone_ocd `Volume` shape — the coupling cap and fixed load live INSIDE
/// the target group there).
fn classify_pot_divider_at_out(
    edges: &[usize],
    target_nodes: &std::collections::HashSet<NodeId>,
    graph: &CircuitGraph,
) -> Option<(NodeId, BoundaryLoadModel)> {
    let out = graph.out_node;
    let is_gnd = |n: NodeId| n == graph.gnd_node || graph.ac_ground_nodes.contains(&n);

    let [e0i, e1i] = edges else {
        return None;
    };
    let (e0, e1) = (&graph.edges[*e0i], &graph.edges[*e1i]);
    if e0.comp_idx != e1.comp_idx {
        return None;
    }
    let comp = &graph.components[e0.comp_idx];
    if !comp.kind.is_pot() {
        return None;
    }
    // Shared node = wiper; must be the global `out`.
    let w = [e0.node_a, e0.node_b]
        .into_iter()
        .find(|&n| n == e1.node_a || n == e1.node_b)?;
    if w != out {
        return None;
    }
    let free0 = if e0.node_a == w { e0.node_b } else { e0.node_a };
    let free1 = if e1.node_a == w { e1.node_b } else { e1.node_a };
    let boundary = if target_nodes.contains(&free0) && is_gnd(free1) {
        free0
    } else if target_nodes.contains(&free1) && is_gnd(free0) {
        free1
    } else {
        return None;
    };
    // Exclusivity on the wiper/out node.
    if interior_nodes_shared_outside(edges, &[out], graph) {
        return None;
    }
    let r_pot = pot_track_resistance(graph, e0.comp_idx)?;
    Some((
        boundary,
        BoundaryLoadModel::PotDividerAtOut {
            pot_id: comp.id.clone(),
            r_pot,
            edges: edges.to_vec(),
        },
    ))
}

/// Fallback VISIBILITY model: a passive trailing group at `out` that attaches
/// to the target group and contains at least one grounded shunt element (a
/// series-only branch to `out` is an open circuit, not a load — e.g. the
/// phase90 `R10 → C_out → out` branch stays unrecognized). Policy never fuses
/// this shape; the row documents WHAT hangs off the boundary.
fn classify_passive_network(
    edges: &[usize],
    target_nodes: &std::collections::HashSet<NodeId>,
    graph: &CircuitGraph,
) -> Option<(NodeId, BoundaryLoadModel)> {
    let is_gnd = |n: NodeId| n == graph.gnd_node || graph.ac_ground_nodes.contains(&n);
    let mut boundary: Option<NodeId> = None;
    let mut has_gnd = false;
    for &eidx in edges {
        let e = &graph.edges[eidx];
        if boundary.is_none() {
            if target_nodes.contains(&e.node_a) {
                boundary = Some(e.node_a);
            } else if target_nodes.contains(&e.node_b) {
                boundary = Some(e.node_b);
            }
        }
        if is_gnd(e.node_a) || is_gnd(e.node_b) {
            has_gnd = true;
        }
    }
    let boundary = boundary?;
    if !has_gnd {
        return None;
    }
    Some((
        boundary,
        BoundaryLoadModel::PassiveNetwork {
            edges: edges.to_vec(),
        },
    ))
}

/// POLICY: may the analyzed trailing load fuse into the upstream stage's
/// general-MNA build? Dispatches per MODEL shape (lkf1.3):
///
/// * `SeriesCapIntoLoad` — the BA283 gate, unchanged;
/// * `SeriesCapIntoPotLoad` — same device-shape + dc_qpoint gate. DC is
///   unaffected by construction exactly like the fixed-R case: the coupling
///   cap blocks DC and every trailing element is grounded, so `v(load) = 0`
///   at DC and the dc_qpoint seed solved WITHOUT the load stays valid. The
///   pot lands in the fused stage as a variable-G element
///   (`stamp_passive_edges` → `VariableResistorCandidate` → `pot_children`),
///   so `set_control` re-stamps the fused G matrix and re-derives scattering
///   + extraction — the same path as any in-stage pot;
/// * `PotDividerAtOut` — same gate PLUS the boundary node must be DC-isolated
///   inside the target group (every target edge at the node is a capacitor or
///   a grounded fixed resistor), otherwise fusing the pot's DC path to ground
///   would shift the op-point the seed was solved without
///   ([`REASON_DC_COUPLED_POT`]);
/// * `PassiveNetwork` (and anything else) — never fused
///   ([`REASON_UNVALIDATED_SHAPE`]): recognized for the decision table only.
pub(super) fn gate_fusion(
    load: &TrailingOutputLoad,
    target: &FlowGroup,
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Result<(), &'static str> {
    match &load.model {
        BoundaryLoadModel::SeriesCapIntoLoad { .. }
        | BoundaryLoadModel::SeriesCapIntoPotLoad { .. } => {
            gate_ba283_fusion(target, graph, supply_voltage)
        }
        BoundaryLoadModel::PotDividerAtOut { .. } => {
            gate_ba283_fusion(target, graph, supply_voltage)?;
            boundary_node_dc_isolated(load.boundary_node, target, graph)
        }
        BoundaryLoadModel::Unloaded
        | BoundaryLoadModel::GroundedResistive { .. }
        | BoundaryLoadModel::PassiveNetwork { .. } => Err(REASON_UNVALIDATED_SHAPE),
    }
}

/// DC-safety check for [`BoundaryLoadModel::PotDividerAtOut`]: every TARGET
/// edge incident on the boundary node must be a capacitor or a fixed (non-pot)
/// resistor to the TRUE ground node (AC-ground rails carry DC, so they do not
/// qualify). Then the node sits at 0 V DC with or without the pot's resistive
/// path to ground, and the fusion cannot shift the dc_qpoint seed.
fn boundary_node_dc_isolated(
    boundary: NodeId,
    target: &FlowGroup,
    graph: &CircuitGraph,
) -> Result<(), &'static str> {
    for &eidx in &target.all_edges() {
        let e = &graph.edges[eidx];
        if e.node_a != boundary && e.node_b != boundary {
            continue;
        }
        let comp = &graph.components[e.comp_idx];
        let other = if e.node_a == boundary {
            e.node_b
        } else {
            e.node_a
        };
        let ok = comp.kind.capacitance().is_some()
            || (comp.kind.resistance().is_some() && !comp.kind.is_pot() && other == graph.gnd_node);
        if !ok {
            return Err(REASON_DC_COUPLED_POT);
        }
    }
    Ok(())
}

/// POLICY gate for BA283-class output-load fusion — gated exactly like the
/// consolidated dc_qpoint seed + servo (`apply_bjt_dc_qpoint`):
///
/// * `PK_OUTPUT_FUSE_DISABLE` is the opt-out escape hatch, mirroring
///   `PK_SERVO_DISABLE`;
/// * the target's NL devices must be ≥2 true 3-terminal BJTs and NOTHING else
///   (the `all_resolved` ∧ `n_bjt ≥ 2` shape);
/// * `solve_bjt_group_dc_qpoint` must actually produce the op-point seed —
///   the same call the general-MNA builder makes before `apply_bjt_dc_qpoint`
///   engages.
///
/// DC is unaffected by the fusion by construction: the coupling cap blocks DC,
/// the DC solve sees the load node only through the grounded resistor
/// (v(out) = 0), and the cap's DC seed charges to the amplifier's output-node
/// voltage exactly like every other coupling cap in `apply_bjt_dc_qpoint`.
///
/// Returns `Err(reason)` when the gate declines; the reason lands in the
/// `CompiledPedal::boundary_loads` decision table.
pub(super) fn gate_ba283_fusion(
    target: &FlowGroup,
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Result<(), &'static str> {
    // Opt-out escape hatch, mirroring PK_SERVO_DISABLE.
    if std::env::var("PK_OUTPUT_FUSE_DISABLE").is_ok() {
        return Err("env:PK_OUTPUT_FUSE_DISABLE");
    }

    // Device shape: every NL device in the target is a true 3-terminal BJT
    // (base != collector) and there are at least two — the exact shape
    // `apply_bjt_dc_qpoint` requires before it engages the consolidated
    // seed + servo (`all_resolved` ∧ `n_bjt >= 2`).
    let mut nl_kinds: Vec<NonlinearKind> = Vec::new();
    let mut seen_comps: std::collections::HashSet<usize> = std::collections::HashSet::new();
    for &eidx in &target.all_edges() {
        if graph.effective_edge_kind(eidx) != EdgeKind::Nonlinear {
            continue;
        }
        let e = &graph.edges[eidx];
        if !seen_comps.insert(e.comp_idx) {
            continue;
        }
        let comp = &graph.components[e.comp_idx];
        let Some((kind, _)) = comp.kind.classify_nonlinear(
            &comp.id,
            e.node_a,
            e.node_b,
            graph.gnd_node,
            &graph.node_names,
        ) else {
            return Err("gate:unclassified-nonlinear");
        };
        let is_three_terminal_bjt = matches!(
            &kind,
            NonlinearKind::BjtNpn { base_node, collector_node, .. }
            | NonlinearKind::BjtPnp { base_node, collector_node, .. }
                if base_node != collector_node
        );
        if !is_three_terminal_bjt {
            return Err("gate:non-bjt-nonlinear");
        }
        nl_kinds.push(kind);
    }
    if nl_kinds.len() < 2 {
        return Err("gate:fewer-than-two-bjts");
    }

    // The dc_qpoint path itself: the compile-time nonlinear DC solve must
    // produce the op-point seed for this group — the same call the
    // general-MNA builder makes before `apply_bjt_dc_qpoint` engages.
    if super::bias::solve_bjt_group_dc_qpoint(&nl_kinds, &target.all_edges(), graph, supply_voltage)
        .is_none()
    {
        return Err("gate:no-dc-qpoint-seed");
    }

    Ok(())
}

/// Result of [`analyze_transformer_secondary_load`]: a transformer whose
/// PRIMARY-side edge lives in the target group and whose SECONDARY winding
/// carries a recognized load network in a *different* flow group.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct TransformerSecondaryLoad {
    /// Index (into the flow-group slice) of the secondary-side load group.
    pub load_group: usize,
    /// Turns ratio `n = N_primary / N_secondary` (10:1 step-down → 10.0).
    pub turns_ratio: f64,
    /// The secondary hot node the load hangs off (`T.c` == the global `out`).
    pub boundary_node: NodeId,
    /// The load resistance as seen from the PRIMARY winding: `n² · r_total`.
    pub r_reflected: f64,
    /// What hangs across the secondary winding.
    pub model: BoundaryLoadModel,
}

/// ANALYSIS ONLY — find a SECONDARY-side load group magnetically downstream of
/// the transformer whose primary edge lives in `target_gi` (the GAP F /
/// LA-2A output-transformer family).
///
/// WHY a dedicated analysis: a transformer contributes only its PRIMARY
/// winding as a conductive `graph.edges` entry — the secondary side couples
/// through `coupled_nodes` (magnetic flux), never an edge. Flow-group
/// formation walks conductive edges only, so a loaded secondary always splits
/// into its own group and compiles as a standalone stage fed through the
/// impedance-blind sample chain: the primary-side solve terminates the
/// transformer with its bare magnetizing branch (an open secondary), and the
/// load stage divides by an arbitrary source resistance. Measured on the CE →
/// 10:1 OT → 1 kΩ probes: +20 dB (collector-fed) / +26 dB (cap-coupled) vs
/// ngspice.
///
/// The recognized shape is deliberately EXACT (the C1 discipline):
/// * the target group holds exactly ONE transformer component — a plain
///   two-port (no tertiary winding), finite positive turns ratio;
/// * its secondary cold end (`T.d`) is grounded and its hot end (`T.c`) IS
///   the global `out` (the output-transformer family: LA-2A `T_out`, Pultec
///   output, the SPICE step-down fixtures) and does not touch the target
///   group;
/// * a different, fully passive flow group hangs off `T.c` consisting of
///   NOTHING but grounded fixed (non-pot) load resistors — no caps, no pots,
///   no actives — and none of its edges are broker-cut (`DelayedCutSet`).
///
/// No device gate, no env check here — that is POLICY
/// ([`gate_transformer_reflection`]). Returns `None` when the shape is not
/// recognized.
pub(super) fn analyze_transformer_secondary_load(
    target_gi: usize,
    groups: &[FlowGroup],
    graph: &CircuitGraph,
    cut_edges: &DelayedCutSet,
) -> Option<TransformerSecondaryLoad> {
    let out = graph.out_node;
    if out == graph.gnd_node || out == graph.vcc_node {
        return None;
    }
    let is_gnd = |n: NodeId| n == graph.gnd_node || graph.ac_ground_nodes.contains(&n);

    // Exactly one transformer component in the target group (its primary edge).
    let target = &groups[target_gi];
    let mut xfmr_comp: Option<usize> = None;
    for &eidx in &target.all_edges() {
        let comp_idx = graph.edges[eidx].comp_idx;
        if graph.components[comp_idx].kind.is_transformer() {
            match xfmr_comp {
                None => xfmr_comp = Some(comp_idx),
                Some(prev) if prev == comp_idx => {}
                Some(_) => return None, // two transformers — ambiguous shape
            }
        }
    }
    let comp_idx = xfmr_comp?;
    let comp = &graph.components[comp_idx];
    let cfg = comp.kind.transformer_config()?;
    if cfg.has_tertiary() {
        return None;
    }
    let n = cfg.turns_ratio;
    if !(n.is_finite() && n > 0.0) {
        return None;
    }

    // Secondary winding terminals: cold end grounded, hot end == global `out`.
    let sec_pos = *graph.node_names.get(&format!("{}.c", comp.id))?;
    let sec_neg = *graph.node_names.get(&format!("{}.d", comp.id))?;
    if !is_gnd(sec_neg) || sec_pos != out || is_gnd(sec_pos) {
        return None;
    }
    // The hot secondary node must live strictly downstream of the target
    // group (no target edge may already touch it).
    for &eidx in &target.all_edges() {
        let e = &graph.edges[eidx];
        if e.node_a == sec_pos || e.node_b == sec_pos {
            return None;
        }
    }

    // The load group: a passive group of NOTHING but grounded fixed (non-pot)
    // load resistors at the secondary hot node.
    for (src_gi, source) in groups.iter().enumerate() {
        if src_gi == target_gi || !source.active_edges.is_empty() {
            continue;
        }
        let edges = source.all_edges();
        if edges.is_empty() {
            continue;
        }
        // Never fuse across a broker-cut (delayed-coupling) boundary.
        if edges.iter().any(|eidx| cut_edges.cuts.contains_key(eidx)) {
            continue;
        }
        let mut load_conductance = 0.0f64;
        let mut shape_ok = true;
        for &eidx in &edges {
            let e = &graph.edges[eidx];
            let load_comp = &graph.components[e.comp_idx];
            let touches_sec = e.node_a == sec_pos || e.node_b == sec_pos;
            match graph.effective_edge_kind(eidx) {
                EdgeKind::Linear
                    if load_comp.kind.resistance().is_some()
                        && !load_comp.kind.is_pot()
                        && touches_sec
                        && (is_gnd(e.node_a) || is_gnd(e.node_b)) =>
                {
                    if let Some(r) = load_comp.kind.resistance() {
                        if r > 0.0 {
                            load_conductance += 1.0 / r;
                        }
                    }
                }
                _ => {
                    shape_ok = false;
                }
            }
            if !shape_ok {
                break;
            }
        }
        if shape_ok && load_conductance > 0.0 {
            let r_total = 1.0 / load_conductance;
            return Some(TransformerSecondaryLoad {
                load_group: src_gi,
                turns_ratio: n,
                boundary_node: sec_pos,
                r_reflected: n * n * r_total,
                model: BoundaryLoadModel::GroundedResistive { r_total, edges },
            });
        }
    }
    None
}

/// POLICY gate for transformer-secondary load reflection.
///
/// The reflection lowers the fused (transformer + load) group via
/// `build_passive_rtype_stage`, whose MNA carries the full transformer
/// skeleton (DCR + leakage + magnetizing/JA core + ideal turns-ratio stamp) —
/// the same machinery that matches the ngspice coupled-inductor fixtures to
/// ~−90 dB when transformer and load share a stage. That builder is
/// passive-only, so the target group must be fully passive (a transformer
/// claimed INTO an active device's group — e.g. an OT primary in a collector
/// branch — is out of scope here and stays declined-with-reason for the
/// dashboard).
///
/// * `PK_XFMR_REFLECT_DISABLE` is the opt-out escape hatch, mirroring
///   `PK_OUTPUT_FUSE_DISABLE`.
pub(super) fn gate_transformer_reflection(
    target: &FlowGroup,
    graph: &CircuitGraph,
) -> Result<(), &'static str> {
    // Opt-out escape hatch, mirroring PK_OUTPUT_FUSE_DISABLE.
    if std::env::var("PK_XFMR_REFLECT_DISABLE").is_ok() {
        return Err("env:PK_XFMR_REFLECT_DISABLE");
    }
    if !target.active_edges.is_empty() {
        return Err("gate:transformer-stage-not-passive");
    }
    for &eidx in &target.all_edges() {
        if !graph.components[graph.edges[eidx].comp_idx]
            .kind
            .is_passive()
        {
            return Err("gate:transformer-stage-not-passive");
        }
    }
    Ok(())
}

/// A boundary-load decision recorded during stage building, before final
/// stage ordering is known. `stage_key` is the stage's minimum stable
/// component id — the same deterministic identity the stage sort uses — and
/// is resolved to a final `CompiledPedal::stages` index by
/// [`resolve_boundary_load_bindings`].
pub(super) struct PendingBoundaryLoad {
    pub stage_key: String,
    pub boundary_node: NodeId,
    pub model: BoundaryLoadSummary,
    pub disposition: BoundaryLoadDisposition,
}

/// Resolve pending boundary-load records against the FINAL stage order.
///
/// `stage_min_comp_id[i]` must be the minimum stable component id of
/// `stages[i]` after the last permutation sort (spqr_build keeps that table
/// aligned through both sorts). Unresolvable keys (stage consumed after
/// recording — not expected in practice) keep the record with
/// `upstream_stage == usize::MAX` rather than dropping the analysis.
pub(super) fn resolve_boundary_load_bindings(
    pending: Vec<PendingBoundaryLoad>,
    stage_min_comp_id: &[String],
) -> Vec<BoundaryLoadBinding> {
    pending
        .into_iter()
        .map(|p| BoundaryLoadBinding {
            upstream_stage: stage_min_comp_id
                .iter()
                .position(|id| *id == p.stage_key)
                .unwrap_or(usize::MAX),
            boundary_node: p.boundary_node,
            model: p.model,
            disposition: p.disposition,
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// BA283-shaped fixture: two DC-coupled NPN common-emitter stages — Q1
    /// self-biased via collector→base shunt feedback (Rf, a cycle through the
    /// active device, so the pair lands in ONE feedback flow group), Q2
    /// DC-coupled from Q1's collector — with the classic trailing output
    /// network `Cout → {RL ‖ gnd}` at the global `out`.
    const TWO_BJT_FB_AMP: &str = r#"
        pedal "two bjt fb amp" { supply 9V
            components {
                Cin:  cap(10u, electrolytic)
                Rpd:  resistor(1M)
                Rc1:  resistor(33k)
                Q1:   npn(2n3904)
                Rf:   resistor(470k)
                Rc2:  resistor(5.6k)
                Q2:   npn(2n3904)
                Re2:  resistor(1k)
                Cout: cap(10u, electrolytic)
                RL:   resistor(10k)
            }
            nets {
                in -> Cin.a, Rpd.a
                Rpd.b -> gnd
                Cin.b -> Q1.base, Rf.b
                vcc -> Rc1.a
                Rc1.b -> Q1.collector, Rf.a, Q2.base
                Q1.emitter -> gnd
                vcc -> Rc2.a
                Rc2.b -> Q2.collector, Cout.a
                Q2.emitter -> Re2.a
                Re2.b -> gnd
                Cout.b -> RL.a, out
                RL.b -> gnd
            }
        }"#;

    /// Same output network, but the target group holds ONE BJT (shunt
    /// collector→base feedback) — fails the ≥2-BJT policy gate.
    const ONE_BJT_FB_AMP: &str = r#"
        pedal "one bjt fb amp" { supply 9V
            components {
                Cin:  cap(10u, electrolytic)
                Rpd:  resistor(1M)
                Rc1:  resistor(33k)
                Q1:   npn(2n3904)
                Re1:  resistor(1k)
                Rf:   resistor(470k)
                Cout: cap(10u, electrolytic)
                RL:   resistor(10k)
            }
            nets {
                in -> Cin.a, Rpd.a
                Rpd.b -> gnd
                Cin.b -> Q1.base, Rf.b
                vcc -> Rc1.a
                Rc1.b -> Q1.collector, Rf.a, Cout.a
                Q1.emitter -> Re1.a
                Re1.b -> gnd
                Cout.b -> RL.a, out
                RL.b -> gnd
            }
        }"#;

    /// Trailing group holds a POT RHEOSTAT at `out` instead of a fixed load
    /// resistor (wiper strapped to the grounded end) — the simplest
    /// pot-bearing load shape (lkf1.3).
    const POT_LOAD_AMP: &str = r#"
        pedal "pot load amp" { supply 9V
            components {
                Cin:  cap(10u, electrolytic)
                Rpd:  resistor(1M)
                Rc1:  resistor(33k)
                Q1:   npn(2n3904)
                Rf:   resistor(470k)
                Rc2:  resistor(5.6k)
                Q2:   npn(2n3904)
                Re2:  resistor(1k)
                Cout: cap(10u, electrolytic)
                RL:   pot(10k)
            }
            nets {
                in -> Cin.a, Rpd.a
                Rpd.b -> gnd
                Cin.b -> Q1.base, Rf.b
                vcc -> Rc1.a
                Rc1.b -> Q1.collector, Rf.a, Q2.base
                Q1.emitter -> gnd
                vcc -> Rc2.a
                Rc2.b -> Q2.collector, Cout.a
                Q2.emitter -> Re2.a
                Re2.b -> gnd
                Cout.b -> RL.a, out
                RL.w -> gnd
                RL.b -> gnd
            }
        }"#;

    /// TWO_BJT amp + `Cout → {Rload ∥ VOL divider}`, wiper = the global
    /// `out` (the tube_screamer / big_muff / fulltone_ocd output family on a
    /// BA283-class target). The `{Cout, Rload}` sub-network cross-reactive-
    /// merges INTO the feedback group (it does not touch `out`), leaving the
    /// pot as its own trailing divider group — the `PotDividerAtOut` shape
    /// with a cap-isolated (DC-safe) boundary node.
    const POT_DIVIDER_LOAD_AMP: &str = r#"
        pedal "pot divider load amp" { supply 9V
            components {
                Cin:   cap(10u, electrolytic)
                Rpd:   resistor(1M)
                Rc1:   resistor(33k)
                Q1:    npn(2n3904)
                Rf:    resistor(470k)
                Rc2:   resistor(5.6k)
                Q2:    npn(2n3904)
                Re2:   resistor(1k)
                Cout:  cap(10u, electrolytic)
                Rload: resistor(100k)
                VOL:   pot(100k)
            }
            nets {
                in -> Cin.a, Rpd.a
                Rpd.b -> gnd
                Cin.b -> Q1.base, Rf.b
                vcc -> Rc1.a
                Rc1.b -> Q1.collector, Rf.a, Q2.base
                Q1.emitter -> gnd
                vcc -> Rc2.a
                Rc2.b -> Q2.collector, Cout.a
                Q2.emitter -> Re2.a
                Re2.b -> gnd
                Cout.b -> Rload.a, VOL.a
                Rload.b -> gnd
                VOL.w -> out
                VOL.b -> gnd
            }
            controls {
                VOL.position -> "Volume" [1.0, 0.0] = 0.5
            }
        }"#;

    /// TWO_BJT amp + `Cout → {Rload ∥ VOL divider}`, wiper → series R → `out`
    /// (the proco_rat / blues_driver output family). The whole trailing
    /// network touches `out`, so it stays ONE passive group —
    /// `SeriesCapIntoPotLoad` with the series wiper resistor.
    const POT_WIPER_SERIES_R_LOAD_AMP: &str = r#"
        pedal "pot wiper series r load amp" { supply 9V
            components {
                Cin:   cap(10u, electrolytic)
                Rpd:   resistor(1M)
                Rc1:   resistor(33k)
                Q1:    npn(2n3904)
                Rf:    resistor(470k)
                Rc2:   resistor(5.6k)
                Q2:    npn(2n3904)
                Re2:   resistor(1k)
                Cout:  cap(10u, electrolytic)
                Rload: resistor(100k)
                VOL:   pot(100k)
                Rw:    resistor(1k)
            }
            nets {
                in -> Cin.a, Rpd.a
                Rpd.b -> gnd
                Cin.b -> Q1.base, Rf.b
                vcc -> Rc1.a
                Rc1.b -> Q1.collector, Rf.a, Q2.base
                Q1.emitter -> gnd
                vcc -> Rc2.a
                Rc2.b -> Q2.collector, Cout.a
                Q2.emitter -> Re2.a
                Re2.b -> gnd
                Cout.b -> Rload.a, VOL.a
                Rload.b -> gnd
                VOL.w -> Rw.a
                VOL.b -> gnd
                Rw.b -> out
            }
            controls {
                VOL.position -> "Volume" [1.0, 0.0] = 0.5
            }
        }"#;

    /// A pot divider hanging DIRECTLY off Q2's collector — the boundary node
    /// carries DC (Rc2 to VCC + the collector itself), so the
    /// `PotDividerAtOut` POLICY must decline with
    /// [`REASON_DC_COUPLED_POT`] even though the ANALYSIS recognizes the
    /// shape.
    const POT_DIVIDER_DC_COUPLED_AMP: &str = r#"
        pedal "pot divider dc coupled amp" { supply 9V
            components {
                Cin:  cap(10u, electrolytic)
                Rpd:  resistor(1M)
                Rc1:  resistor(33k)
                Q1:   npn(2n3904)
                Rf:   resistor(470k)
                Rc2:  resistor(5.6k)
                Q2:   npn(2n3904)
                Re2:  resistor(1k)
                VOL:  pot(100k)
            }
            nets {
                in -> Cin.a, Rpd.a
                Rpd.b -> gnd
                Cin.b -> Q1.base, Rf.b
                vcc -> Rc1.a
                Rc1.b -> Q1.collector, Rf.a, Q2.base
                Q1.emitter -> gnd
                vcc -> Rc2.a
                Rc2.b -> Q2.collector, VOL.a
                Q2.emitter -> Re2.a
                Re2.b -> gnd
                VOL.w -> out
                VOL.b -> gnd
            }
            controls {
                VOL.position -> "Volume" [1.0, 0.0] = 0.5
            }
        }"#;

    /// Reproduce the production grouping prelude (compile_via_spqr Step 1)
    /// up to the point the fusion call site sees: graph, merged flow groups,
    /// and the broker cut set.
    fn groups_for(src: &str) -> (CircuitGraph, Vec<FlowGroup>, DelayedCutSet) {
        let pedal = crate::dsl::parse_pedal_file(src).expect("parse failed");
        let graph = CircuitGraph::from_pedal(&pedal);
        let cut_edges = super::super::boundary_rules::delayed_cut_edges(&graph);
        let active_set: std::collections::HashSet<usize> =
            graph.active_edge_indices.iter().copied().collect();
        let all_edges: Vec<usize> = (0..graph.edges.len())
            .filter(|i| !active_set.contains(i))
            .collect();
        let mut groups = super::super::signal_flow::find_flow_groups(&all_edges, &graph);
        super::super::spqr_build::merge_cross_reactive_groups_into_active_groups(
            &mut groups,
            &graph,
            &cut_edges,
        );
        super::super::spqr_build::merge_input_coupling_into_active_groups(
            &mut groups,
            &graph,
            &cut_edges,
        );
        (graph, groups, cut_edges)
    }

    fn feedback_group_index(groups: &[FlowGroup]) -> usize {
        groups
            .iter()
            .position(|g| g.has_feedback())
            .expect("fixture should produce a feedback group")
    }

    fn edge_index_of(graph: &CircuitGraph, comp_id: &str) -> usize {
        graph
            .edges
            .iter()
            .position(|e| graph.components[e.comp_idx].id == comp_id)
            .unwrap_or_else(|| panic!("no edge for component {comp_id}"))
    }

    #[test]
    fn ba283_shape_analyzes_to_series_cap_into_load() {
        let (graph, groups, cut_edges) = groups_for(TWO_BJT_FB_AMP);
        let target_gi = feedback_group_index(&groups);

        let load = analyze_trailing_output_load(target_gi, &groups, &graph, &cut_edges)
            .expect("BA283 shape should analyze to a trailing output load");

        // Boundary node = the coupling cap's stage-side node (Q2 collector).
        let nout = *graph
            .node_names
            .get("Q2.collector")
            .expect("Q2.collector node");
        assert_eq!(load.boundary_node, nout);

        match &load.model {
            BoundaryLoadModel::SeriesCapIntoLoad { c, r_total, edges } => {
                assert!((c - 10e-6).abs() < 1e-9, "Cout = 10u, got {c}");
                assert!((r_total - 10e3).abs() < 1e-6, "RL = 10k, got {r_total}");
                assert_eq!(edges.len(), 2, "trailing group = {{Cout, RL}}");
            }
            other => panic!("expected SeriesCapIntoLoad, got {other:?}"),
        }

        // Serializable summary carries component ids, not edge indices.
        match load.model.summarize(&graph) {
            BoundaryLoadSummary::SeriesCapIntoLoad { component_ids, .. } => {
                assert_eq!(component_ids, vec!["Cout".to_string(), "RL".to_string()]);
            }
            other => panic!("expected SeriesCapIntoLoad summary, got {other:?}"),
        }
    }

    #[test]
    fn ba283_shape_passes_policy_gate() {
        let (graph, groups, _cut_edges) = groups_for(TWO_BJT_FB_AMP);
        let target_gi = feedback_group_index(&groups);
        assert_eq!(
            gate_ba283_fusion(&groups[target_gi], &graph, 9.0),
            Ok(()),
            "2-BJT DC-feedback target with a dc_qpoint seed should pass the gate"
        );
    }

    /// lkf1.3: the pot RHEOSTAT load (wiper strapped to ground) is now a
    /// recognized fusable shape. (Pre-lkf1.3 this fixture locked the DECLINE;
    /// the widening flipped it by design.)
    #[test]
    fn pot_rheostat_load_analyzes_and_passes_gate() {
        let (graph, groups, cut_edges) = groups_for(POT_LOAD_AMP);
        let target_gi = feedback_group_index(&groups);
        let load = analyze_trailing_output_load(target_gi, &groups, &graph, &cut_edges)
            .expect("pot rheostat at out should analyze to SeriesCapIntoPotLoad");

        let nout = *graph
            .node_names
            .get("Q2.collector")
            .expect("Q2.collector node");
        assert_eq!(load.boundary_node, nout);
        match &load.model {
            BoundaryLoadModel::SeriesCapIntoPotLoad {
                c,
                pot_id,
                r_pot,
                r_fixed,
                edges,
            } => {
                assert!((c - 10e-6).abs() < 1e-9, "Cout = 10u, got {c}");
                assert_eq!(pot_id, "RL");
                assert!((r_pot - 10e3).abs() < 1e-6, "RL track = 10k, got {r_pot}");
                assert_eq!(*r_fixed, None, "the pot is the only grounded element");
                assert_eq!(
                    edges.len(),
                    2,
                    "trailing group = {{Cout, RL rheostat edge}}"
                );
            }
            other => panic!("expected SeriesCapIntoPotLoad, got {other:?}"),
        }
        // POLICY: a 2-BJT dc_qpoint target may fuse the pot load (DC-safe by
        // construction: the coupling cap blocks DC).
        assert_eq!(
            gate_fusion(&load, &groups[target_gi], &graph, 9.0),
            Ok(()),
            "pot-bearing trailing group should pass the fusion gate on a BA283-class target"
        );
    }

    /// lkf1.3: pot DIVIDER load, wiper = the global `out` (tube_screamer /
    /// big_muff / fulltone_ocd family), plus a fixed 100k pulldown.
    #[test]
    fn pot_divider_load_analyzes_and_passes_gate() {
        let (graph, groups, cut_edges) = groups_for(POT_DIVIDER_LOAD_AMP);
        let target_gi = feedback_group_index(&groups);
        let load = analyze_trailing_output_load(target_gi, &groups, &graph, &cut_edges)
            .expect("pot divider at out should analyze to SeriesCapIntoPotLoad");

        match &load.model {
            BoundaryLoadModel::SeriesCapIntoPotLoad {
                c,
                pot_id,
                r_pot,
                r_fixed,
                edges,
            } => {
                assert!((c - 10e-6).abs() < 1e-9);
                assert_eq!(pot_id, "VOL");
                assert!((r_pot - 100e3).abs() < 1e-6);
                let rf = r_fixed.expect("Rload = 100k pulldown should be modeled");
                assert!((rf - 100e3).abs() < 1e-3, "r_fixed = 100k, got {rf}");
                assert_eq!(
                    edges.len(),
                    4,
                    "trailing group = {{Cout, Rload, VOL aw, VOL wb}}"
                );
            }
            other => panic!("expected SeriesCapIntoPotLoad, got {other:?}"),
        }
        assert_eq!(gate_fusion(&load, &groups[target_gi], &graph, 9.0), Ok(()));

        // Serializable summary carries the pot id + component ids.
        match load.model.summarize(&graph) {
            BoundaryLoadSummary::SeriesCapIntoPotLoad {
                pot_id,
                component_ids,
                ..
            } => {
                assert_eq!(pot_id, "VOL");
                assert_eq!(
                    component_ids,
                    vec!["Cout".to_string(), "Rload".to_string(), "VOL".to_string()]
                );
            }
            other => panic!("expected SeriesCapIntoPotLoad summary, got {other:?}"),
        }
    }

    /// lkf1.3: pot divider whose wiper reaches `out` through one series fixed
    /// resistor (proco_rat / blues_driver family).
    #[test]
    fn pot_wiper_series_r_load_analyzes_and_passes_gate() {
        let (graph, groups, cut_edges) = groups_for(POT_WIPER_SERIES_R_LOAD_AMP);
        let target_gi = feedback_group_index(&groups);
        let load = analyze_trailing_output_load(target_gi, &groups, &graph, &cut_edges)
            .expect("wiper→series-R→out should analyze to SeriesCapIntoPotLoad");
        match &load.model {
            BoundaryLoadModel::SeriesCapIntoPotLoad { pot_id, edges, .. } => {
                assert_eq!(pot_id, "VOL");
                assert_eq!(
                    edges.len(),
                    5,
                    "trailing group = {{Cout, Rload, VOL aw, VOL wb, Rw}}"
                );
            }
            other => panic!("expected SeriesCapIntoPotLoad, got {other:?}"),
        }
        assert_eq!(gate_fusion(&load, &groups[target_gi], &graph, 9.0), Ok(()));
    }

    /// lkf1.3 negative: a pot divider hanging DIRECTLY off a DC-carrying node
    /// (Q2's collector, no coupling cap). ANALYSIS recognizes the
    /// `PotDividerAtOut` shape; POLICY must decline — fusing the pot's DC
    /// path to ground would shift the op-point the dc_qpoint seed was solved
    /// without.
    #[test]
    fn dc_coupled_pot_divider_analyzes_but_policy_declines() {
        let (graph, groups, cut_edges) = groups_for(POT_DIVIDER_DC_COUPLED_AMP);
        let target_gi = feedback_group_index(&groups);
        let load = analyze_trailing_output_load(target_gi, &groups, &graph, &cut_edges)
            .expect("pot divider directly at the collector should analyze");
        match &load.model {
            BoundaryLoadModel::PotDividerAtOut { pot_id, r_pot, .. } => {
                assert_eq!(pot_id, "VOL");
                assert!((r_pot - 100e3).abs() < 1e-6);
            }
            other => panic!("expected PotDividerAtOut, got {other:?}"),
        }
        let nout = *graph
            .node_names
            .get("Q2.collector")
            .expect("Q2.collector node");
        assert_eq!(load.boundary_node, nout);
        assert_eq!(
            gate_fusion(&load, &groups[target_gi], &graph, 9.0),
            Err(REASON_DC_COUPLED_POT),
            "a DC-carrying boundary node must decline PotDividerAtOut fusion"
        );
    }

    #[test]
    fn broker_cut_edge_declines_analysis() {
        let (graph, groups, mut cut_edges) = groups_for(TWO_BJT_FB_AMP);
        let target_gi = feedback_group_index(&groups);

        // Sanity: without the cut, the shape analyzes.
        assert!(analyze_trailing_output_load(target_gi, &groups, &graph, &cut_edges).is_some());

        // Synthesize a broker cut on the coupling cap edge — never fuse
        // across a delayed-coupling boundary.
        cut_edges.cuts.insert(
            edge_index_of(&graph, "Cout"),
            super::super::boundary_rules::Directive::NonMergeCut,
        );
        assert_eq!(
            analyze_trailing_output_load(target_gi, &groups, &graph, &cut_edges),
            None,
            "a broker-cut edge in the trailing group must decline analysis"
        );
    }

    #[test]
    fn single_bjt_target_declines_gate_with_reason() {
        let (graph, groups, cut_edges) = groups_for(ONE_BJT_FB_AMP);
        let target_gi = feedback_group_index(&groups);

        // The trailing output network itself is the recognized shape...
        assert!(
            analyze_trailing_output_load(target_gi, &groups, &graph, &cut_edges).is_some(),
            "analysis is shape-only; the single-BJT fixture still has Cout->RL"
        );
        // ...but the policy gate declines: not the >=2-BJT dc_qpoint path.
        let reason = gate_ba283_fusion(&groups[target_gi], &graph, 9.0)
            .expect_err("single-BJT target must not pass the BA283 fusion gate");
        assert!(
            reason.starts_with("gate:"),
            "decline reason should name the gate, got {reason:?}"
        );
        assert_eq!(reason, "gate:fewer-than-two-bjts");
    }

    #[test]
    fn compiled_two_bjt_pedal_records_fused_boundary_load() {
        // skip_blockwise forces the feedback group down the general-MNA
        // (dc_qpoint) path where the fusion call site lives; skip_k_tables
        // keeps the debug compile fast.
        let pedal = crate::dsl::parse_pedal_file(TWO_BJT_FB_AMP).expect("parse failed");
        let options = super::super::compile::CompileOptions {
            skip_blockwise: true,
            skip_k_tables: true,
            ..Default::default()
        };
        let compiled =
            super::super::spqr_build::compile_via_spqr_with_options(&pedal, 48_000.0, options)
                .expect("compile failed");

        assert_eq!(
            compiled.boundary_loads.len(),
            1,
            "one analyzed output boundary expected, got {:?}",
            compiled.boundary_loads
        );
        let binding = &compiled.boundary_loads[0];
        assert_eq!(
            binding.disposition,
            pedalkernel_rt::processor::BoundaryLoadDisposition::FusedUpstream
        );
        match &binding.model {
            BoundaryLoadSummary::SeriesCapIntoLoad {
                c,
                r_total,
                component_ids,
            } => {
                assert!((c - 10e-6).abs() < 1e-9);
                assert!((r_total - 10e3).abs() < 1e-6);
                assert_eq!(component_ids, &["Cout".to_string(), "RL".to_string()]);
            }
            other => panic!("expected SeriesCapIntoLoad summary, got {other:?}"),
        }
        // The upstream stage resolved to the fused general-MNA stage.
        assert!(
            binding.upstream_stage < compiled.stages.len(),
            "upstream_stage should resolve, got {}",
            binding.upstream_stage
        );
        assert!(
            matches!(
                compiled.stages[binding.upstream_stage],
                super::super::compiled::Stage::MultiNl(_)
            ),
            "the fused stage is the multi-BJT general-MNA stage"
        );
        // Boundary node = the coupling cap's stage-side node (Q2 collector).
        assert_eq!(binding.boundary_node, graph_node(&pedal, "Q2.collector"));
    }

    #[test]
    fn compiled_single_bjt_pedal_records_unloaded_reason() {
        let pedal = crate::dsl::parse_pedal_file(ONE_BJT_FB_AMP).expect("parse failed");
        let options = super::super::compile::CompileOptions {
            skip_blockwise: true,
            skip_k_tables: true,
            ..Default::default()
        };
        let compiled =
            super::super::spqr_build::compile_via_spqr_with_options(&pedal, 48_000.0, options)
                .expect("compile failed");

        assert_eq!(compiled.boundary_loads.len(), 1);
        match &compiled.boundary_loads[0].disposition {
            pedalkernel_rt::processor::BoundaryLoadDisposition::Unloaded { reason } => {
                assert!(
                    reason.starts_with("gate:"),
                    "single-BJT decline should carry a gate reason, got {reason:?}"
                );
            }
            other => panic!("expected Unloaded disposition, got {other:?}"),
        }
    }

    // ── Fleet reality locks (lkf1.3): the widened analysis against the REAL
    // example pedals. These document CURRENT REALITY — when routing or the
    // policy gate changes, update the locks honestly. ──────────────────────

    fn example_source(rel: &str) -> String {
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("examples/pedals")
            .join(rel);
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path:?}: {e}"))
    }

    fn feedback_group_containing<'g>(
        groups: &'g [FlowGroup],
        graph: &CircuitGraph,
        comp_id: &str,
    ) -> usize {
        groups
            .iter()
            .position(|g| {
                g.has_feedback()
                    && g.all_edges()
                        .iter()
                        .any(|&e| graph.components[graph.edges[e].comp_idx].id == comp_id)
            })
            .unwrap_or_else(|| panic!("no feedback group containing {comp_id}"))
    }

    /// fulltone_ocd: the `Volume` pot IS the trailing group (its coupling cap
    /// `C5` + fixed load `R7` cross-reactive-merged into the MOSFET clipper
    /// group), wiper wired directly to `out` — the real-world
    /// `PotDividerAtOut` instance. The boundary node is cap-isolated (the
    /// dc-safety check passes), but the target group carries MOSFETs, so the
    /// device-shape gate declines: analysis-recognized, policy-declined,
    /// visible in the table, safe in production.
    #[test]
    fn fulltone_ocd_volume_analyzes_pot_divider_at_out_but_gate_declines() {
        let src = example_source("overdrive/fulltone_ocd.pedal");
        let (graph, groups, cut_edges) = groups_for(&src);
        let target_gi = feedback_group_containing(&groups, &graph, "M1");

        let load = analyze_trailing_output_load(target_gi, &groups, &graph, &cut_edges)
            .expect("OCD Volume divider should be recognized (lkf1.3)");
        match &load.model {
            BoundaryLoadModel::PotDividerAtOut { pot_id, r_pot, .. } => {
                assert_eq!(pot_id, "Volume");
                assert!((r_pot - 100e3).abs() < 1e-6, "Volume = 100k, got {r_pot}");
            }
            other => panic!("expected PotDividerAtOut, got {other:?}"),
        }

        // The boundary node (C5.b) is DC-isolated inside the target group —
        // the dc-safety half of the policy would pass...
        assert_eq!(
            boundary_node_dc_isolated(load.boundary_node, &groups[target_gi], &graph),
            Ok(()),
            "OCD boundary node is cap-isolated (C5) with a grounded fixed R (R7)"
        );
        // ...but the device-shape gate declines (MOSFET clippers, not BJTs).
        assert_eq!(
            gate_fusion(&load, &groups[target_gi], &graph, 9.0),
            Err("gate:non-bjt-nonlinear"),
            "OCD target is a MOSFET group — current reality is policy-declined"
        );
    }

    /// tube_screamer: the trailing group bundles the whole Tone stack with
    /// the Level pot — recognized as the `PassiveNetwork` VISIBILITY model
    /// (what hangs off the boundary is now in the table), never fused
    /// (`gate:unvalidated-shape`).
    #[test]
    fn tube_screamer_output_network_analyzes_passive_network_visibility() {
        let src = example_source("overdrive/tube_screamer.pedal");
        let (graph, groups, cut_edges) = groups_for(&src);
        let target_gi = feedback_group_containing(&groups, &graph, "U2");

        let load = analyze_trailing_output_load(target_gi, &groups, &graph, &cut_edges)
            .expect("tube_screamer tone/level network should be visible (lkf1.3)");
        match load.model.summarize(&graph) {
            BoundaryLoadSummary::PassiveNetwork { component_ids } => {
                assert!(
                    component_ids.contains(&"Level".to_string())
                        && component_ids.contains(&"Tone".to_string()),
                    "the visibility model should name the tone/level network, got {component_ids:?}"
                );
            }
            other => panic!("expected PassiveNetwork summary, got {other:?}"),
        }
        assert_eq!(
            gate_fusion(&load, &groups[target_gi], &graph, 9.0),
            Err(REASON_UNVALIDATED_SHAPE),
            "PassiveNetwork is visibility-only — no fusion policy is validated for it"
        );
    }

    /// proco_rat (live WSL/NaN triage case): same PassiveNetwork visibility —
    /// the trailing group bundles the Filter pot + output network.
    #[test]
    fn proco_rat_output_network_analyzes_passive_network_visibility() {
        let src = example_source("distortion/proco_rat.pedal");
        let (graph, groups, cut_edges) = groups_for(&src);
        let target_gi = feedback_group_containing(&groups, &graph, "U1");

        let load = analyze_trailing_output_load(target_gi, &groups, &graph, &cut_edges)
            .expect("proco_rat filter/volume network should be visible (lkf1.3)");
        match load.model.summarize(&graph) {
            BoundaryLoadSummary::PassiveNetwork { component_ids } => {
                assert!(
                    component_ids.contains(&"Volume".to_string()),
                    "expected the Volume pot in the visibility model, got {component_ids:?}"
                );
            }
            other => panic!("expected PassiveNetwork summary, got {other:?}"),
        }
        assert_eq!(
            gate_fusion(&load, &groups[target_gi], &graph, 9.0),
            Err(REASON_UNVALIDATED_SHAPE)
        );
    }

    /// phase90: the output branch is `U6.out → R10 → C_out → out` with NO
    /// grounded element — an OPEN series branch, not a load. Analysis must
    /// keep returning `None` (the boundary is genuinely unloaded).
    #[test]
    fn phase90_series_output_branch_stays_unrecognized() {
        let src = example_source("phaser/phase90.pedal");
        let (graph, groups, cut_edges) = groups_for(&src);
        let target_gi = feedback_group_containing(&groups, &graph, "U6");
        assert_eq!(
            analyze_trailing_output_load(target_gi, &groups, &graph, &cut_edges),
            None,
            "a series-only branch to out is an open circuit, not a load"
        );
    }

    // ── Compiled-pedal locks for the fused pot load (lkf1.3) ──────────────

    fn compile_fixture(src: &str) -> super::super::compiled::CompiledPedal {
        let pedal = crate::dsl::parse_pedal_file(src).expect("parse failed");
        let options = super::super::compile::CompileOptions {
            skip_blockwise: true,
            skip_k_tables: true,
            ..Default::default()
        };
        super::super::spqr_build::compile_via_spqr_with_options(&pedal, 48_000.0, options)
            .expect("compile failed")
    }

    /// The divider fixture compiles with the pot FUSED into the multi-BJT
    /// general-MNA stage: one FusedUpstream row, the fused stage owns the pot
    /// as an in-stage variable resistor (control target
    /// `PotInMultiNlStage`), the standalone trailing stage is consumed, and
    /// NO WiperDivider is synthesized for the pot (the divider is solved
    /// in-network — a WiperDivider would double-apply the position).
    #[test]
    fn compiled_pot_divider_pedal_fuses_pot_as_variable_g() {
        let compiled = compile_fixture(POT_DIVIDER_LOAD_AMP);

        assert_eq!(
            compiled.boundary_loads.len(),
            1,
            "one analyzed output boundary expected, got {:?}",
            compiled.boundary_loads
        );
        let binding = &compiled.boundary_loads[0];
        assert_eq!(
            binding.disposition,
            pedalkernel_rt::processor::BoundaryLoadDisposition::FusedUpstream
        );
        match &binding.model {
            BoundaryLoadSummary::SeriesCapIntoPotLoad {
                pot_id,
                component_ids,
                ..
            } => {
                assert_eq!(pot_id, "VOL");
                assert_eq!(
                    component_ids,
                    &["Cout".to_string(), "Rload".to_string(), "VOL".to_string()]
                );
            }
            other => panic!("expected SeriesCapIntoPotLoad summary, got {other:?}"),
        }
        assert!(
            matches!(
                compiled.stages[binding.upstream_stage],
                super::super::compiled::Stage::MultiNl(_)
            ),
            "the fused stage is the multi-BJT general-MNA stage"
        );

        // The pot is owned by EXACTLY the fused stage (standalone trailing
        // stage consumed — the load isn't applied twice).
        let owners: Vec<usize> = compiled
            .stages
            .iter()
            .enumerate()
            .filter_map(|(idx, s)| s.control_target_for_pot(idx, "VOL").map(|_| idx))
            .collect();
        assert_eq!(
            owners,
            vec![binding.upstream_stage],
            "VOL must be owned by the fused stage only"
        );

        // The control binding re-stamps the fused stage like any in-stage pot.
        let ctrl = compiled
            .controls
            .iter()
            .find(|c| c.component_id == "VOL")
            .expect("Volume control binding should exist");
        assert!(
            matches!(
                ctrl.target,
                super::super::compiled::ControlTarget::PotInMultiNlStage(..)
            ),
            "fused pot control target should be PotInMultiNlStage, got {:?}",
            ctrl.target
        );

        // No WiperDivider for the fused pot — the wiper node (`out`) is an
        // MNA node of the fused stage; extraction happens at the wiper, so a
        // post-stage divider would DOUBLE-APPLY the position.
        assert!(
            compiled
                .wiper_dividers
                .iter()
                .all(|w| w.pot_comp_id != "VOL"),
            "no WiperDivider must be synthesized for an in-MNA fused pot"
        );
    }

    /// CONTROL FIDELITY (lkf1.3 gate 3): the fused pot responds to
    /// `set_control` — sweeping the Volume control re-stamps the fused
    /// stage's G matrix (scattering + extraction re-derived), the output
    /// level moves MONOTONICALLY over a wide range, and the DC operating
    /// point stays stable at every position (silence in → ~0 out).
    #[test]
    fn fused_pot_divider_volume_sweeps_monotonically_with_stable_dc_op() {
        use pedalkernel_rt::PedalProcessor as _;
        const SR: f64 = 48_000.0;
        let mut proc = compile_fixture(POT_DIVIDER_LOAD_AMP);

        // Fused sanity (same shape as the lock above).
        assert_eq!(
            proc.boundary_loads[0].disposition,
            pedalkernel_rt::processor::BoundaryLoadDisposition::FusedUpstream
        );

        let mut rms_at = Vec::new();
        for &v in &[0.1, 0.3, 0.5, 0.7, 0.9] {
            // Immediate path: bypasses the smoother, re-stamps the fused G
            // matrix and re-derives scattering + extraction right away.
            proc.set_control_immediate("Volume", v);

            // DC settle on silence: the op must stay stable across
            // re-stamps (the coupling cap blocks DC; v(out) → 0). A re-stamp
            // kicks a REAL cap re-equilibration transient (τ ≈ Cout × network
            // R ≈ 0.25 s), so give it 16k samples before measuring.
            let mut dc_acc = 0.0f64;
            for i in 0..16_000 {
                let y = proc.process(0.0);
                assert!(y.is_finite(), "non-finite output at Volume={v} (i={i})");
                if i >= 15_000 {
                    dc_acc += y;
                }
            }
            let dc = dc_acc / 1000.0;
            assert!(
                dc.abs() < 1e-4,
                "DC op should be stable after re-stamp at Volume={v}: dc={dc}"
            );

            // Small-signal RMS at 440 Hz.
            let n = 4800;
            let mut acc = 0.0;
            for s in 0..n {
                let x = 0.01 * (2.0 * core::f64::consts::PI * 440.0 * s as f64 / SR).sin();
                let y = proc.process(x);
                assert!(y.is_finite());
                acc += y * y;
            }
            rms_at.push((v, (acc / n as f64).sqrt()));
        }

        // Monotonic increasing (control range [1.0, 0.0]: higher app value →
        // wiper toward the track top → louder), with real authority.
        for w in rms_at.windows(2) {
            let (v0, r0) = w[0];
            let (v1, r1) = w[1];
            assert!(
                r1 > r0 * 1.15,
                "fused pot sweep must be monotonic: Volume={v0} → rms={r0:.6}, \
                 Volume={v1} → rms={r1:.6} (full sweep: {rms_at:?})"
            );
        }
        let (_, lo) = rms_at.first().unwrap();
        let (_, hi) = rms_at.last().unwrap();
        assert!(
            hi / lo > 3.0,
            "the volume pot should have wide authority over the fused stage: \
             lo={lo:.6} hi={hi:.6}"
        );
    }

    /// Fleet diagnostic (ignored; lkf1.3 data-first tool): dump per-group
    /// shape data — group composition, trailing-load analysis, and the fusion
    /// gate verdict — for the example pedals whose output boundaries were
    /// open at the C2 sweep. This is the group-level view the compiled
    /// `boundary_loads` table cannot show (it built the lkf1.3 shape catalog;
    /// keep it for the lkf1.5 blockwise-recording work). Run with
    /// `cargo test -p pedalkernel --lib boundary_load::tests::dump_fleet -- --nocapture --ignored`
    #[test]
    #[ignore]
    fn dump_fleet_group_shapes() {
        let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("examples/pedals");
        for rel in [
            "distortion/proco_rat.pedal",
            "fuzz/big_muff.pedal",
            "modulation/boss_ce2.pedal",
            "overdrive/blues_driver.pedal",
            "overdrive/fulltone_ocd.pedal",
            "overdrive/klon_centaur.pedal",
            "overdrive/tube_screamer.pedal",
            "phaser/phase90.pedal",
        ] {
            let src = std::fs::read_to_string(root.join(rel)).expect("read pedal");
            let (graph, groups, cut_edges) = groups_for(&src);
            let node_name = |n: NodeId| -> String {
                graph
                    .node_names
                    .iter()
                    .filter(|(_, &id)| id == n)
                    .map(|(name, _)| name.clone())
                    .collect::<Vec<_>>()
                    .join("/")
            };
            println!("\n===== {rel} (out={}) =====", node_name(graph.out_node));
            for (gi, g) in groups.iter().enumerate() {
                let mut ids: Vec<String> = g
                    .all_edges()
                    .iter()
                    .map(|&e| graph.components[graph.edges[e].comp_idx].id.clone())
                    .collect();
                ids.sort();
                ids.dedup();
                let touches_out = g.all_edges().iter().any(|&e| {
                    graph.edges[e].node_a == graph.out_node
                        || graph.edges[e].node_b == graph.out_node
                });
                println!(
                    "  group {gi}: fb={} active={} out={} comps={:?}",
                    g.has_feedback(),
                    g.active_edges.len(),
                    touches_out,
                    ids
                );
                if g.has_feedback() {
                    let analysis = analyze_trailing_output_load(gi, &groups, &graph, &cut_edges);
                    let gate = gate_ba283_fusion(g, &graph, 9.0);
                    println!("    -> analysis={analysis:?} gate={gate:?}");
                }
            }
        }
    }

    /// Node id lookup through a fresh graph build (test-only convenience for
    /// asserting boundary nodes against compiled pedals — node ids are
    /// deterministic per netlist).
    fn graph_node(pedal: &crate::dsl::PedalDef, name: &str) -> NodeId {
        let graph = CircuitGraph::from_pedal(pedal);
        *graph
            .node_names
            .get(name)
            .unwrap_or_else(|| panic!("no node named {name}"))
    }

    // ────────────────────────────────────────────────────────────────────
    // Transformer-secondary load reflection (GAP F / lkf1.4)
    // ────────────────────────────────────────────────────────────────────

    /// The LA-2A `T_out` shape (GAP F positive fixture): CE amp cap-coupled
    /// into a 10:1 output transformer whose grounded-secondary hot end is the
    /// global `out` with a 1 kΩ load. The transformer + coupling cap form
    /// their own all-passive flow group; {RL} splits into a separate group
    /// (the windings couple magnetically, not conductively).
    ///
    /// ngspice truth for this exact circuit (coupled inductors k=0.99,
    /// 1 kHz): transformer-stage transfer 0.1021×; before the reflection the
    /// chain measured +26 dB high.
    const XFMR_OUT_AMP: &str = r#"
        pedal "xfmr out amp" { supply 9V
            components {
                Cin:  cap(10u, electrolytic)
                Rb1:  resistor(100k)
                Rb2:  resistor(22k)
                Q1:   npn(2n3904)
                Rc:   resistor(4.7k)
                Re:   resistor(1k)
                Ce:   cap(100u, electrolytic)
                Cout: cap(10u, electrolytic)
                T1:   transformer(10:1, 2H)
                RL:   resistor(1k)
            }
            nets {
                in -> Cin.a
                Cin.b -> Q1.base, Rb1.b, Rb2.a
                vcc -> Rb1.a, Rc.a
                Rc.b -> Q1.collector, Cout.a
                Q1.emitter -> Re.a, Ce.a
                Re.b -> gnd
                Ce.b -> gnd
                Rb2.b -> gnd
                Cout.b -> T1.a
                T1.b -> gnd
                T1.c -> out, RL.a
                T1.d -> gnd
                RL.b -> gnd
            }
        }"#;

    /// Collector-fed output transformer (single-ended output stage): the
    /// primary edge is CLAIMED into the BJT's group, so the transformer group
    /// is NOT all-passive — the shape analyzes but the policy gate declines
    /// (reflecting into an active stage's tree is out of this bead's scope).
    const XFMR_COLLECTOR_AMP: &str = r#"
        pedal "xfmr collector amp" { supply 9V
            components {
                Cin: cap(10u, electrolytic)
                Rb1: resistor(100k)
                Rb2: resistor(22k)
                Q1:  npn(2n3904)
                Re:  resistor(1k)
                Ce:  cap(100u, electrolytic)
                T1:  transformer(10:1, 2H)
                RL:  resistor(1k)
            }
            nets {
                in -> Cin.a
                Cin.b -> Q1.base, Rb1.b, Rb2.a
                vcc -> Rb1.a, T1.a
                T1.b -> Q1.collector
                Q1.emitter -> Re.a, Ce.a
                Re.b -> gnd
                Ce.b -> gnd
                Rb2.b -> gnd
                T1.c -> out, RL.a
                T1.d -> gnd
                RL.b -> gnd
            }
        }"#;

    /// Same as the positive fixture but the load is a POT — not the fixed
    /// grounded-resistor load shape; analysis must decline.
    const XFMR_POT_LOAD: &str = r#"
        pedal "xfmr pot load" { supply 9V
            components {
                Cin:  cap(10u, electrolytic)
                Rb1:  resistor(100k)
                Rb2:  resistor(22k)
                Q1:   npn(2n3904)
                Rc:   resistor(4.7k)
                Re:   resistor(1k)
                Ce:   cap(100u, electrolytic)
                Cout: cap(10u, electrolytic)
                T1:   transformer(10:1, 2H)
                RL:   pot(10k)
            }
            nets {
                in -> Cin.a
                Cin.b -> Q1.base, Rb1.b, Rb2.a
                vcc -> Rb1.a, Rc.a
                Rc.b -> Q1.collector, Cout.a
                Q1.emitter -> Re.a, Ce.a
                Re.b -> gnd
                Ce.b -> gnd
                Rb2.b -> gnd
                Cout.b -> T1.a
                T1.b -> gnd
                T1.c -> out, RL.a
                RL.w -> gnd
                RL.b -> gnd
            }
        }"#;

    /// Mid-chain transformer: the secondary hot end feeds a downstream RC
    /// network, NOT the global `out` — outside the output-transformer family;
    /// analysis must decline.
    const XFMR_MID_CHAIN: &str = r#"
        pedal "xfmr mid chain" { supply 9V
            components {
                Cin:  cap(10u, electrolytic)
                Rb1:  resistor(100k)
                Rb2:  resistor(22k)
                Q1:   npn(2n3904)
                Rc:   resistor(4.7k)
                Re:   resistor(1k)
                Ce:   cap(100u, electrolytic)
                Cout: cap(10u, electrolytic)
                T1:   transformer(10:1, 2H)
                Rs:   resistor(1k)
                RL:   resistor(10k)
            }
            nets {
                in -> Cin.a
                Cin.b -> Q1.base, Rb1.b, Rb2.a
                vcc -> Rb1.a, Rc.a
                Rc.b -> Q1.collector, Cout.a
                Q1.emitter -> Re.a, Ce.a
                Re.b -> gnd
                Ce.b -> gnd
                Rb2.b -> gnd
                Cout.b -> T1.a
                T1.b -> gnd
                T1.c -> Rs.a
                T1.d -> gnd
                Rs.b -> RL.a, out
                RL.b -> gnd
            }
        }"#;

    /// Index of the flow group holding the transformer's primary edge.
    fn transformer_group_index(groups: &[FlowGroup], graph: &CircuitGraph) -> usize {
        groups
            .iter()
            .position(|g| {
                g.all_edges().iter().any(|&eidx| {
                    graph.components[graph.edges[eidx].comp_idx]
                        .kind
                        .is_transformer()
                })
            })
            .expect("fixture should have a transformer-owning group")
    }

    #[test]
    fn xfmr_out_amp_analyzes_to_secondary_load() {
        let (graph, groups, cut_edges) = groups_for(XFMR_OUT_AMP);
        let target_gi = transformer_group_index(&groups, &graph);

        let load = analyze_transformer_secondary_load(target_gi, &groups, &graph, &cut_edges)
            .expect("output-transformer shape should analyze to a secondary load");

        assert_eq!(load.turns_ratio, 10.0, "10:1 → n = 10");
        assert!(
            (load.r_reflected - 100_000.0).abs() < 1e-6,
            "n²·RL = 100·1k, got {}",
            load.r_reflected
        );
        // Boundary node = the secondary hot end == global `out`.
        assert_eq!(load.boundary_node, graph.out_node);
        assert_eq!(
            *graph.node_names.get("T1.c").expect("T1.c node"),
            graph.out_node
        );
        match &load.model {
            BoundaryLoadModel::GroundedResistive { r_total, edges } => {
                assert!((r_total - 1_000.0).abs() < 1e-9, "RL = 1k, got {r_total}");
                assert_eq!(edges.len(), 1, "load group = {{RL}}");
            }
            other => panic!("expected GroundedResistive, got {other:?}"),
        }
        // The load group is a different group than the transformer group.
        assert_ne!(load.load_group, target_gi);

        // Passive transformer group passes the policy gate.
        assert_eq!(
            gate_transformer_reflection(&groups[target_gi], &graph),
            Ok(())
        );
    }

    #[test]
    fn collector_fed_transformer_declines_gate_with_reason() {
        let (graph, groups, cut_edges) = groups_for(XFMR_COLLECTOR_AMP);
        let target_gi = transformer_group_index(&groups, &graph);

        // The shape itself analyzes (transformer + grounded-R load at out)...
        assert!(
            analyze_transformer_secondary_load(target_gi, &groups, &graph, &cut_edges).is_some(),
            "analysis is shape-only; the collector-fed fixture still has T1 + RL"
        );
        // ...but the transformer lives in the BJT's (active) group — declined.
        assert_eq!(
            gate_transformer_reflection(&groups[target_gi], &graph),
            Err("gate:transformer-stage-not-passive")
        );
    }

    #[test]
    fn pot_load_declines_transformer_analysis() {
        let (graph, groups, cut_edges) = groups_for(XFMR_POT_LOAD);
        let target_gi = transformer_group_index(&groups, &graph);
        assert_eq!(
            analyze_transformer_secondary_load(target_gi, &groups, &graph, &cut_edges),
            None,
            "a pot across the secondary is not the fixed grounded-R load shape"
        );
    }

    #[test]
    fn mid_chain_secondary_declines_transformer_analysis() {
        let (graph, groups, cut_edges) = groups_for(XFMR_MID_CHAIN);
        let target_gi = transformer_group_index(&groups, &graph);
        assert_eq!(
            analyze_transformer_secondary_load(target_gi, &groups, &graph, &cut_edges),
            None,
            "a secondary feeding a mid-chain network (hot end != out) must decline"
        );
    }

    #[test]
    fn broker_cut_load_edge_declines_transformer_analysis() {
        let (graph, groups, mut cut_edges) = groups_for(XFMR_OUT_AMP);
        let target_gi = transformer_group_index(&groups, &graph);

        // Sanity: without the cut, the shape analyzes.
        assert!(
            analyze_transformer_secondary_load(target_gi, &groups, &graph, &cut_edges).is_some()
        );

        // Synthesize a broker cut on the load edge — never fuse across a
        // delayed-coupling boundary.
        cut_edges.cuts.insert(
            edge_index_of(&graph, "RL"),
            super::super::boundary_rules::Directive::NonMergeCut,
        );
        assert_eq!(
            analyze_transformer_secondary_load(target_gi, &groups, &graph, &cut_edges),
            None
        );
    }

    #[test]
    fn compiled_xfmr_out_amp_reflects_and_consumes_load_stage() {
        let pedal = crate::dsl::parse_pedal_file(XFMR_OUT_AMP).expect("parse failed");
        let options = super::super::compile::CompileOptions {
            skip_k_tables: true,
            ..Default::default()
        };
        let compiled =
            super::super::spqr_build::compile_via_spqr_with_options(&pedal, 48_000.0, options)
                .expect("compile failed");

        assert_eq!(
            compiled.boundary_loads.len(),
            1,
            "one analyzed transformer boundary expected, got {:?}",
            compiled.boundary_loads
        );
        let binding = &compiled.boundary_loads[0];
        match &binding.disposition {
            pedalkernel_rt::processor::BoundaryLoadDisposition::ReflectedThroughTransformer {
                turns_ratio,
                r_reflected,
            } => {
                assert_eq!(*turns_ratio, 10.0);
                assert!((r_reflected - 100_000.0).abs() < 1e-6);
            }
            other => panic!("expected ReflectedThroughTransformer, got {other:?}"),
        }
        match &binding.model {
            BoundaryLoadSummary::GroundedResistive {
                r_total,
                component_ids,
            } => {
                assert!((r_total - 1_000.0).abs() < 1e-9);
                assert_eq!(component_ids, &["RL".to_string()]);
            }
            other => panic!("expected GroundedResistive summary, got {other:?}"),
        }
        assert_eq!(binding.boundary_node, graph_node(&pedal, "T1.c"));
        // The upstream stage resolved to the fused primary-side WDF stage.
        assert!(
            binding.upstream_stage < compiled.stages.len(),
            "upstream_stage should resolve, got {}",
            binding.upstream_stage
        );
        assert!(matches!(
            compiled.stages[binding.upstream_stage],
            super::super::compiled::Stage::Wdf(_)
        ));
        // De-duplication: the standalone {RL} stage was consumed — amp stage +
        // fused transformer stage only.
        assert_eq!(
            compiled.stages.len(),
            2,
            "standalone RL load stage must be consumed by the reflection"
        );
    }

    #[test]
    fn compiled_collector_fed_records_declined_reason() {
        let pedal = crate::dsl::parse_pedal_file(XFMR_COLLECTOR_AMP).expect("parse failed");
        let options = super::super::compile::CompileOptions {
            skip_k_tables: true,
            ..Default::default()
        };
        let compiled =
            super::super::spqr_build::compile_via_spqr_with_options(&pedal, 48_000.0, options)
                .expect("compile failed");

        assert_eq!(compiled.boundary_loads.len(), 1);
        match &compiled.boundary_loads[0].disposition {
            pedalkernel_rt::processor::BoundaryLoadDisposition::Unloaded { reason } => {
                assert_eq!(reason, "gate:transformer-stage-not-passive");
            }
            other => panic!("expected declined Unloaded disposition, got {other:?}"),
        }
    }
}
