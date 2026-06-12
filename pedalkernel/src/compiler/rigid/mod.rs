//! Rigid stage strategy selection and building.
//!
//! Scans a Rigid stage's edges once via `StageStats`, then applies
//! decision rules to select the cheapest correct runtime strategy.
//! All decisions driven by `Component` trait queries — no topology
//! pattern matching.
//!
//! Cost ordering (cheapest first):
//! 1. **Iir** — all linear (±VCVS) → biquad from MNA + NonIdealFx. O(1)/sample.
//! 2. **StateSpace** — linear, >2nd order → Schur complement. O(N)/sample.
//! 3. **General** — has NL → MNA scattering + Component::solver_hint(). O(N²)/sample.

mod general;
mod iir;
mod mna_builder;
mod opamp_root; // TODO: remove once all references in old pipeline are gone
mod state_space;

// ═══════════════════════════════════════════════════════════════════════════
// Re-exports
// ═══════════════════════════════════════════════════════════════════════════

pub(super) use self::general::{
    build_general_mna_from_edges, build_general_mna_from_edges_with_hints,
    build_general_mna_from_edges_with_supply, build_opamp_nl_feedback,
};
pub(super) use self::opamp_root::{extract_opamp_config, make_opamp_root, OpAmpConfig};

use super::component::EdgeKind;
use super::dyn_node::DynNode;
use super::graph::{CircuitGraph, NodeId};
use super::signal_flow::FlowGroup;
use super::spqr_build::BuiltStage;
use super::stage::{IirPotBinding, IirPotRole, IirStage, OpAmpWdfAdaptor, RootKind, WdfStage};
use crate::oversampling::{Oversampler, OversamplingFactor};

// ═══════════════════════════════════════════════════════════════════════════
// StageStats — one-pass Component trait scan
// ═══════════════════════════════════════════════════════════════════════════

/// Component trait answers collected from a single pass over a stage's edges.
///
/// Built once, then used by `optimization()` to select the runtime strategy.
/// Tracks edge kind counts and remembers singular edges for fast lookup.
#[derive(Debug, Clone)]
pub(super) struct StageStats {
    pub vcvs_count: usize,
    pub nl_count: usize,
    pub reactive_count: usize,
    pub linear_count: usize,
    pub total: usize,
    /// The VCVS edge index (valid when `vcvs_count == 1`).
    pub vcvs_edge: Option<usize>,
    /// The first NL edge index (valid when `nl_count >= 1`).
    pub nl_edge: Option<usize>,
}

impl StageStats {
    /// Scan edges once, querying `effective_edge_kind()` on each.
    pub fn from_edges(edge_indices: &[usize], graph: &CircuitGraph) -> Self {
        let mut stats = Self {
            vcvs_count: 0,
            nl_count: 0,
            reactive_count: 0,
            linear_count: 0,
            total: edge_indices.len(),
            vcvs_edge: None,
            nl_edge: None,
        };
        // Count unique components per edge kind (multi-port components
        // have multiple edges but count as one device).
        let mut seen_nl: std::collections::HashSet<usize> = std::collections::HashSet::new();
        let mut seen_vcvs: std::collections::HashSet<usize> = std::collections::HashSet::new();
        for &eidx in edge_indices {
            let comp_idx = graph.edges[eidx].comp_idx;
            match graph.effective_edge_kind(eidx) {
                EdgeKind::Vcvs => {
                    if seen_vcvs.insert(comp_idx) {
                        stats.vcvs_count += 1;
                        stats.vcvs_edge = Some(eidx);
                    }
                }
                EdgeKind::Nonlinear => {
                    if seen_nl.insert(comp_idx) {
                        stats.nl_count += 1;
                        if stats.nl_edge.is_none() {
                            stats.nl_edge = Some(eidx);
                        }
                    }
                }
                EdgeKind::Reactive => stats.reactive_count += 1,
                EdgeKind::Linear => stats.linear_count += 1,
                EdgeKind::Vccs | EdgeKind::Behavioral => {}
            }
        }
        stats
    }

    /// Is this a purely passive stage (no VCVS, no NL)?
    pub fn is_all_linear(&self) -> bool {
        self.vcvs_count == 0 && self.nl_count == 0
    }

    /// Does this stage have exactly one VCVS and no nonlinear elements?
    pub fn is_single_vcvs_linear(&self) -> bool {
        self.vcvs_count == 1 && self.nl_count == 0
    }
}

/// Check if all nonlinear edges in a group are 1-port NL devices suitable
/// for the WDF explicit diode solver (Wright Omega or NR single-port).
///
/// Returns true if: at least one NL edge exists, AND all NL edges are
/// single-port nonlinear (diode family). Multi-port NL devices (BJT, JFET,
/// tube) require general MNA and return false.
///
/// Uses port_semantic() to identify NL edges, then checks is_diode_family()
/// for solver compatibility. This is the correct check because the WDF
/// opamp+diode feedback path specifically creates DiodePair/SingleDiode roots.
fn group_nl_edges_are_single_port_solvable(edge_indices: &[usize], graph: &CircuitGraph) -> bool {
    let mut found_nonlinear = false;

    for &edge_idx in edge_indices {
        if graph.effective_edge_kind(edge_idx) != EdgeKind::Nonlinear {
            continue;
        }

        found_nonlinear = true;
        let comp = &graph.components[graph.edges[edge_idx].comp_idx];
        // Must be a 1-port NL device (diode family) for the WDF solver.
        // Multi-port NL (BJT, JFET, tube) can't use this path.
        if !comp.kind.is_diode_family() {
            return false;
        }
    }

    found_nonlinear
}

// ═══════════════════════════════════════════════════════════════════════════
// RigidOptimization — strategy selection
// ═══════════════════════════════════════════════════════════════════════════

/// Runtime strategy for a Rigid stage, selected at compile time.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum RigidOptimization {
    /// All linear, no VCVS → biquad from MNA. O(1)/sample.
    /// Covers passive bridges, 808 bridged-T resonator, pot dividers.
    Iir,
    /// Single VCVS, linear feedback → Black's formula (H = A/(1+Aβ)).
    /// Closed-form gain from Rf/Ri, GBW/slew/rails from NonIdealFx.
    /// Named after Harold Black's 1927 negative feedback amplifier theorem.
    /// O(1)/sample with WDF two-port scattering (Zi input, Zf feedback).
    BlackFeedback,
    /// Linear but IIR-incompatible (>2nd order) → state-space. O(N)/sample.
    StateSpace,
    /// Has NL elements → MNA scattering + Component::solver_hint(). O(N²)/sample.
    General,
}

/// Select the cheapest correct strategy for a Rigid stage.
///
/// Decision rules (applied in cost order):
/// 1. NL present → General (needs NR/WO solver, Component decides which)
/// 2. Single VCVS, no NL → BlackFeedback (closed-form Rf/Ri + NonIdealFx)
/// 3. No VCVS, linear → IIR from MNA (passive bridges, resonators)
/// 4. Fallback → General
pub(super) fn classify_rigid(
    stats: &StageStats,
    graph: &CircuitGraph,
    _group: Option<&FlowGroup>,
) -> RigidOptimization {
    // Rule 1: any NL element forces general MNA + solver
    if stats.nl_count > 0 {
        return RigidOptimization::General;
    }

    // Rule 2: single VCVS, no NL → check feedback path topology.
    // If reactive elements are in ground shunts with resistive feedback (808 bridged-T),
    // IIR handles the resonance correctly via extract_feedback_r.
    // Otherwise, Black's formula for closed-form Rf/Ri gain.
    //
    // SAFETY: Even though nl_count == 0, check for ControlledConductance ports
    // (pots, photocouplers) that couple to reactive nodes. If found, IIR lowering
    // is unsafe because the controlled element's time-varying impedance interacts
    // with the reactive state in ways that a static biquad can't capture.
    if stats.is_single_vcvs_linear() {
        if let Some(g) = _group {
            let all_edges = g.all_edges();

            // Coupling safety check: if a ControlledConductance port shares a
            // node with a reactive element, we must NOT lower to static IIR.
            if super::coupling::has_nl_reactive_coupling(&all_edges, graph) {
                return RigidOptimization::General;
            }

            let is_reactive = |eidx: usize| -> bool {
                let comp = &graph.components[graph.edges[eidx].comp_idx];
                comp.kind.capacitance().is_some() || comp.kind.inductance().is_some()
            };

            // Any reactive element in the rigid group → IIR to capture
            // frequency response. Covers:
            //   - 808 bridged-T: caps to ground, resistive feedback
            //   - MFB/Rauch: cap in feedback (C from neg to out)
            //   - Sallen-Key (when opamp group includes filter caps)
            // Without this, BlackFeedback computes only DC gain = Rf/Ri,
            // losing the filter's poles and zeros.
            let has_any_reactive = all_edges.iter().any(|&eidx| is_reactive(eidx));
            if has_any_reactive {
                return RigidOptimization::Iir;
            }
        }
        return RigidOptimization::BlackFeedback;
    }

    // Rule 3: all linear, no VCVS → IIR biquad from MNA
    // Reactive elements give biquad dynamics; purely resistive gives DC gain.
    // Same coupling safety check applies.
    if stats.nl_count == 0 {
        if let Some(g) = _group {
            let all_edges = g.all_edges();
            if super::coupling::has_nl_reactive_coupling(&all_edges, graph) {
                return RigidOptimization::General;
            }
        }
        return RigidOptimization::Iir;
    }

    // Rule 4: fallback
    RigidOptimization::General
}

/// Determine if a single-VCVS stage is inverting or non-inverting.
///
/// Inverting: pos pin is at ground (or AC ground).
/// Non-inverting: pos pin carries signal (not ground).
/// Used by general.rs for NL+VCVS stages (TS, RAT pattern).
pub(super) fn is_inverting_topology(stats: &StageStats, graph: &CircuitGraph) -> bool {
    let vcvs_idx = match stats.vcvs_edge {
        Some(idx) => idx,
        None => return true,
    };
    let edge = &graph.edges[vcvs_idx];

    for rec in &graph.nullor_pins {
        if rec.comp_idx == edge.comp_idx {
            // Inverting if V+ is at AC ground: GND, AC ground node, or a
            // bias point that reaches GND through a capacitor (voltage
            // divider bias networks are AC ground by definition).
            if rec.pos_node == graph.gnd_node || graph.ac_ground_nodes.contains(&rec.pos_node) {
                return true;
            }

            // Check if pos_node is a bias point: BFS from pos through
            // passive edges (excluding neg/out to avoid traversing the
            // feedback network). If pos reaches ANY rail → it's biased
            // to a fixed DC point → inverting.
            let mut blocked = std::collections::HashSet::new();
            blocked.insert(rec.neg_node);
            blocked.insert(rec.out_node);

            // Output nodes of OTHER op-amps are driven signal sources
            // (DC- or R-coupled cascades feed pos from an upstream stage's
            // output). Treat them like `in`: signal, and opaque — never
            // traverse through into the upstream feedback network.
            let upstream_outs: std::collections::HashSet<usize> = graph
                .nullor_pins
                .iter()
                .filter(|r| r.comp_idx != edge.comp_idx)
                .map(|r| r.out_node)
                .collect();

            let mut visited = std::collections::HashSet::new();
            let mut queue = std::collections::VecDeque::new();
            visited.insert(rec.pos_node);
            queue.push_back(rec.pos_node);

            let mut reaches_rail = false;
            let mut reaches_signal = false;
            while let Some(node) = queue.pop_front() {
                for e in &graph.edges {
                    let comp = &graph.components[e.comp_idx];
                    if !comp.kind.is_passive() {
                        continue;
                    }
                    let next = if e.node_a == node && !visited.contains(&e.node_b) {
                        Some(e.node_b)
                    } else if e.node_b == node && !visited.contains(&e.node_a) {
                        Some(e.node_a)
                    } else {
                        None
                    };
                    if let Some(n) = next {
                        if blocked.contains(&n) {
                            continue;
                        }
                        if n == graph.in_node {
                            reaches_signal = true;
                        }
                        if upstream_outs.contains(&n) {
                            // Upstream op-amp output: signal source. Don't
                            // walk through it (same as rails).
                            reaches_signal = true;
                            continue;
                        }
                        if n == graph.gnd_node
                            || n == graph.vcc_node
                            || graph.supply_nodes.contains(&n)
                        {
                            reaches_rail = true;
                            // Don't break — keep searching for in_node
                            continue;
                        }
                        visited.insert(n);
                        queue.push_back(n);
                    }
                }
            }

            // If pos reaches circuit input → non-inverting (signal at pos).
            // If pos reaches only rails → inverting (pos is biased).
            if reaches_signal {
                return false;
            }
            return reaches_rail;
        }
    }

    true // Default: inverting
}

/// Collect NonIdealFx from all components in a stage's edge set.
///
/// Each component declares its own non-idealities via the Component trait.
/// The builder collects them all and attaches to the stage as post-processing.
/// Deduplicates by comp_idx (multi-port components have multiple edges).
fn collect_nonideal_fx(
    edge_indices: &[usize],
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Vec<super::component::NonIdealFx> {
    let mut seen = std::collections::HashSet::new();
    let mut fx = Vec::new();
    for &eidx in edge_indices {
        let comp_idx = graph.edges[eidx].comp_idx;
        if seen.insert(comp_idx) {
            fx.extend(graph.components[comp_idx].kind.nonideal_fx(sample_rate));
        }
    }
    fx
}

/// Extract pot bindings from a classified FlowGroup for IIR runtime recomputation.
///
/// Scans ALL edges in the group (feedback, ground shunt, pendant, active) for
/// pots. Each pot gets an IirPotBinding so the IIR stage can recompute biquad
/// coefficients when any pot changes.
///
/// Previously only scanned feedback_edges — missed Resonance pots (ground shunt)
/// and Cutoff pots (input path / pendant).
fn extract_pot_bindings(
    group: &FlowGroup,
    _edge_indices: &[usize],
    graph: &CircuitGraph,
) -> Vec<IirPotBinding> {
    let mut bindings = Vec::new();
    let all_edges = group.all_edges();

    // Ri: sum of resistance on pendant_edges (input coupling path)
    let ri: f64 = group
        .pendant_edges
        .iter()
        .filter_map(|&eidx| {
            graph.components[graph.edges[eidx].comp_idx]
                .kind
                .resistance()
        })
        .sum();
    let ri = if ri <= 0.0 { 1.0 } else { ri }; // Safety: avoid div-by-zero
    let vcvs_pins = group.active_edges.iter().find_map(|&eidx| {
        let comp_idx = graph.edges[eidx].comp_idx;
        graph
            .nullor_pins
            .iter()
            .find(|rec| rec.comp_idx == comp_idx)
    });
    let feedback_r = vcvs_pins
        .map(|pins| {
            all_edges
                .iter()
                .filter_map(|&eidx| {
                    let e = &graph.edges[eidx];
                    let spans_feedback = (e.node_a == pins.neg_node && e.node_b == pins.out_node)
                        || (e.node_a == pins.out_node && e.node_b == pins.neg_node);
                    spans_feedback.then(|| graph.components[e.comp_idx].kind.resistance())?
                })
                .sum::<f64>()
        })
        .unwrap_or(0.0);
    let non_inverting = !is_inverting_topology(&StageStats::from_edges(&all_edges, graph), graph);

    // Scan ALL edges for pots — any pot in the group affects the IIR
    let mut fixed_r: f64 = 0.0;
    let mut seen_comp: std::collections::HashSet<usize> = std::collections::HashSet::new();
    for &eidx in &all_edges {
        let comp_idx = graph.edges[eidx].comp_idx;
        if !seen_comp.insert(comp_idx) {
            continue; // skip duplicate edges for same component
        }
        let comp = &graph.components[comp_idx];
        if let Some(pot) = comp
            .kind
            .as_any()
            .downcast_ref::<crate::compiler::components::Potentiometer>()
        {
            let role = vcvs_pins
                .map(|pins| {
                    classify_iir_pot_role(
                        eidx,
                        comp_idx,
                        pins.neg_node,
                        pins.out_node,
                        &all_edges,
                        graph,
                    )
                })
                .unwrap_or(IirPotRole::Generic);
            let fixed_series_r = if role == IirPotRole::GroundLeg {
                vcvs_pins
                    .and_then(|pins| {
                        ground_leg_fixed_resistance(eidx, pins.neg_node, &all_edges, graph)
                    })
                    .unwrap_or(0.0)
            } else {
                0.0
            };
            bindings.push(IirPotBinding {
                comp_id: comp.id.clone(),
                max_r: pot.max_r,
                fixed_series_r,
                ri,
                position: 0.5, // default
                role,
                feedback_r,
                non_inverting,
            });
        } else if let Some(r) = comp.kind.resistance() {
            // Only count non-pot feedback resistors as fixed_r
            if group.feedback_edges.contains(&eidx) {
                fixed_r += r;
            }
        }
    }

    // Set fixed_series_r on all pot bindings
    for b in &mut bindings {
        if b.role != IirPotRole::GroundLeg {
            b.fixed_series_r = fixed_r;
        }
    }

    bindings
}

fn classify_iir_pot_role(
    pot_edge_idx: usize,
    pot_comp_idx: usize,
    neg: NodeId,
    out: NodeId,
    group_edges: &[usize],
    graph: &CircuitGraph,
) -> IirPotRole {
    let e = &graph.edges[pot_edge_idx];
    let spans_feedback =
        (e.node_a == neg && e.node_b == out) || (e.node_a == out && e.node_b == neg);
    if spans_feedback {
        return IirPotRole::Feedback;
    }

    if ground_leg_fixed_resistance(pot_edge_idx, neg, group_edges, graph).is_some() {
        let touches_ground = |node| {
            node == graph.gnd_node
                || graph.ac_ground_nodes.contains(&node)
                || graph.supply_nodes.contains(&node)
        };
        let has_ground_end = group_edges.iter().any(|&eidx| {
            graph.edges[eidx].comp_idx == pot_comp_idx
                && (touches_ground(graph.edges[eidx].node_a)
                    || touches_ground(graph.edges[eidx].node_b))
        });
        if has_ground_end {
            return IirPotRole::GroundLeg;
        }
    }

    IirPotRole::Generic
}

fn ground_leg_fixed_resistance(
    pot_edge_idx: usize,
    neg: NodeId,
    group_edges: &[usize],
    graph: &CircuitGraph,
) -> Option<f64> {
    let pot_edge = &graph.edges[pot_edge_idx];
    let touches_ground = |node| {
        node == graph.gnd_node
            || graph.ac_ground_nodes.contains(&node)
            || graph.supply_nodes.contains(&node)
    };
    let start = if touches_ground(pot_edge.node_a) {
        pot_edge.node_b
    } else if touches_ground(pot_edge.node_b) {
        pot_edge.node_a
    } else {
        return None;
    };

    let mut stack = vec![(start, 0.0f64)];
    let mut visited = std::collections::HashSet::new();
    while let Some((node, r_sum)) = stack.pop() {
        if !visited.insert(node) {
            continue;
        }
        if node == neg {
            return Some(r_sum);
        }

        for &eidx in group_edges {
            if eidx == pot_edge_idx {
                continue;
            }
            let e = &graph.edges[eidx];
            let next = if e.node_a == node {
                e.node_b
            } else if e.node_b == node {
                e.node_a
            } else {
                continue;
            };
            let comp = &graph.components[e.comp_idx];
            if comp.kind.is_pot() {
                continue;
            }
            if let Some(r) = comp.kind.resistance() {
                stack.push((next, r_sum + r));
            }
        }
    }

    None
}

/// Find a pot in the feedback edges and create a WDF leaf for it.
///
/// Returns `(pot_comp_id, pot_dyn_node, fixed_series_r, parallel_r)` if found.
/// The pot leaf is needed in the WDF tree so `set_pot` can find it and
/// `notify_pot_changed` can read its resistance to recompute OpAmpRoot gain.
fn find_feedback_pot(
    group: &FlowGroup,
    graph: &CircuitGraph,
) -> Option<(String, DynNode, f64, Option<f64>)> {
    let mut pot_id = None;
    let mut pot_leaf = None;
    let mut fixed_r = 0.0f64;
    let mut parallel_r_candidates: Vec<f64> = Vec::new();

    for &eidx in &group.feedback_edges {
        let comp = &graph.components[graph.edges[eidx].comp_idx];
        if comp.kind.is_pot() {
            if pot_id.is_none() {
                pot_id = Some(comp.id.clone());
                pot_leaf = comp.kind.make_leaf(&comp.id, 48000.0);
            }
        } else if let Some(r) = comp.kind.resistance() {
            fixed_r += r;
        }
    }

    // No heuristic for series vs parallel — sum all non-pot resistors
    // in feedback as fixed_series_r. The OpAmpRoot gain formula handles
    // the effective Rf = pot_r + fixed_series_r correctly for series topology.
    // Parallel Rf (if present) is captured separately by the topology.
    let parallel_r: Option<f64> = None;

    pot_id.map(|id| {
        let leaf = pot_leaf.unwrap_or_else(|| {
            DynNode::Leaf(crate::compiler::wdf_leaf::LeafKind::Pot(
                crate::compiler::wdf_leaf::WdfPot {
                    comp_id: id.clone(),
                    max_resistance: 100_000.0,
                    position: 0.5,
                    taper: crate::dsl::PotTaper::B,
                    rp: 50_000.0,
                    last_a: 0.0,
                    complement: false,
                },
            ))
        });
        (id, leaf, fixed_r, parallel_r)
    })
}

// ═══════════════════════════════════════════════════════════════════════════
// Rigid stage dispatcher
// ═══════════════════════════════════════════════════════════════════════════

/// Build a runnable stage from a Rigid SpqrStage.
///
/// Uses `StageStats` + `classify_rigid()` to select the cheapest strategy,
/// then constructs the appropriate stage type.
/// Build a runnable stage from a classified FlowGroup.
///
/// Uses `StageStats` + `classify_rigid()` to select the cheapest strategy.
/// Edge classification from FlowGroup flows directly to builders.
pub(super) fn build_rigid(
    edge_indices: Vec<usize>,
    _boundary_nodes: Vec<NodeId>,
    _pendant_trees: Vec<(DynNode, NodeId)>,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<BuiltStage, String> {
    build_rigid_from_group(edge_indices, graph, sample_rate, None, 9.0, None, true)
}

pub(super) fn build_rigid_without_iir(
    edge_indices: Vec<usize>,
    _boundary_nodes: Vec<NodeId>,
    _pendant_trees: Vec<(DynNode, NodeId)>,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<BuiltStage, String> {
    build_rigid_from_group(edge_indices, graph, sample_rate, None, 9.0, None, false)
}

/// Build from a FlowGroup with full classification.
pub(super) fn build_rigid_from_group(
    edge_indices: Vec<usize>,
    graph: &CircuitGraph,
    sample_rate: f64,
    group: Option<&FlowGroup>,
    supply_voltage: f64,
    bias_v_max: Option<(f64, f64)>,
    allow_iir: bool,
) -> Result<BuiltStage, String> {
    build_rigid_from_group_with_hints(
        edge_indices,
        graph,
        sample_rate,
        group,
        supply_voltage,
        bias_v_max,
        allow_iir,
        &[],
    )
}

/// Build from a FlowGroup with full classification and optional init hints.
pub(super) fn build_rigid_from_group_with_hints(
    edge_indices: Vec<usize>,
    graph: &CircuitGraph,
    sample_rate: f64,
    group: Option<&FlowGroup>,
    supply_voltage: f64,
    bias_v_max: Option<(f64, f64)>,
    allow_iir: bool,
    init_hints: &[crate::dsl::InitHint],
) -> Result<BuiltStage, String> {
    let stats = StageStats::from_edges(&edge_indices, graph);
    let optimization = classify_rigid(&stats, graph, group);

    #[cfg(test)]
    eprintln!(
        "  rigid: {:?} (vcvs={}, nl={}, linear={}, reactive={}, edges={})",
        optimization,
        stats.vcvs_count,
        stats.nl_count,
        stats.linear_count,
        stats.reactive_count,
        edge_indices.len()
    );

    // ── Single-pass fallthrough: cheapest → most expensive ──────────────
    // Each strategy attempts to build a stage. On failure (Err or degenerate
    // result), fall through to the next. No loops, no recovery.
    //
    // Order: BlackFeedback → IIR → StateSpace → General (WDF)
    // classify_rigid picks the entry point; we fall through from there.

    let try_black_feedback = matches!(optimization, RigidOptimization::BlackFeedback);
    let try_iir = allow_iir
        && matches!(
            optimization,
            RigidOptimization::BlackFeedback | RigidOptimization::Iir
        );

    // ── BlackFeedback: O(1)/sample, resistive feedback only ──────────
    if try_black_feedback {
        if let Some(g) = group {
            let inverting = is_inverting_topology(&stats, graph);
            let config = opamp_root::extract_opamp_config(g, inverting, graph)?;

            // Validate: BlackFeedback needs non-zero Rf. If Rf=0 (all-reactive
            // feedback like a coupling cap), this topology needs frequency-
            // dependent processing — fall through to IIR/WDF.
            if config.rf > 0.0 {
                let fx = collect_nonideal_fx(&edge_indices, graph, sample_rate);
                let mut stage = super::stage::BlackFeedbackStage::new(
                    config.rf,
                    config.ri,
                    inverting,
                    &fx,
                    sample_rate,
                );
                // Apply bias-derived asymmetric rail limits directly
                if let Some((pos, neg)) = bias_v_max {
                    stage.set_v_rails(pos, neg);
                } else {
                    let v = (supply_voltage / 2.0 - 1.5).max(0.5);
                    stage.set_v_rails(v, v);
                }

                for &eidx in &g.feedback_edges {
                    let comp = &graph.components[graph.edges[eidx].comp_idx];
                    if comp.kind.is_pot() {
                        if let Some(max_r) = comp.kind.resistance() {
                            let taper = comp.kind.pot_taper().unwrap_or(crate::dsl::PotTaper::B);
                            stage.pot_comp_id = Some(comp.id.clone());
                            stage.pot_fixed_r = (config.rf - max_r).max(0.0);
                            stage.pot_max_r = max_r;
                            stage.pot_taper = taper;
                            stage.set_rf((stage.pot_fixed_r + taper.apply(0.5) * max_r).max(1.0));
                            break;
                        }
                    }
                }

                return Ok(BuiltStage::BlackFeedback(stage));
            }
            // Rf=0 → fall through to IIR
            #[cfg(test)]
            eprintln!("  BlackFeedback: Rf=0 (reactive feedback), falling through to IIR");
        }
    }

    if stats.vcvs_count == 1 && stats.nl_count == 0 {
        if let Some(g) = group {
            if let Some(stage) = try_build_linear_feedback_wdf_adaptor(
                g,
                &stats,
                graph,
                sample_rate,
                supply_voltage,
                bias_v_max,
            ) {
                return Ok(BuiltStage::Wdf(stage));
            }
        }
    }

    // ── IIR: O(1)/sample, ≤2 reactive elements ──────────────────────
    if try_iir || (allow_iir && try_black_feedback) {
        let pendant_trees = Vec::new();

        // Skip rail-only groups (no signal nodes)
        let has_signal_node = edge_indices.iter().any(|&eidx| {
            let e = &graph.edges[eidx];
            let a_rail = e.node_a == graph.gnd_node || graph.supply_nodes.contains(&e.node_a);
            let b_rail = e.node_b == graph.gnd_node || graph.supply_nodes.contains(&e.node_b);
            !a_rail || !b_rail
        });
        if !has_signal_node {
            let iir =
                super::stage::IirData::new(vec![1.0, 0.0, 0.0], vec![1.0, 0.0, 0.0], sample_rate);
            return Ok(BuiltStage::Iir(IirStage::new(iir)));
        }

        if let Ok(built_iir) =
            iir::build_iir_stage(&edge_indices, &pendant_trees, graph, sample_rate)
        {
            // Validate: check for degenerate transfer function (all-zero numerator)
            let iir_data = built_iir.data;
            let has_signal = iir_data.b_coeffs.iter().any(|&b| b.abs() > 1e-15);
            if has_signal {
                let mut stage = IirStage::new(iir_data);
                stage.bind_physical_one_ports(&built_iir.reactive_one_ports);
                stage.bind_ports(built_iir.input_node_id, built_iir.output_node_id);
                let mut fx = collect_nonideal_fx(&edge_indices, graph, sample_rate);
                // Patch RailSaturation with bias-derived v_max or supply voltage.
                // NonIdealFx uses symmetric v_max; OpAmpRoot gets asymmetric rails.
                let actual_v_max = bias_v_max
                    .map(|(pos, neg)| pos.min(neg))
                    .unwrap_or_else(|| (supply_voltage / 2.0 - 1.5).max(0.5));
                for f in &mut fx {
                    if let super::component::NonIdealFx::RailSaturation { v_max } = f {
                        *v_max = actual_v_max;
                    }
                }
                if !fx.is_empty() {
                    stage.set_nonideal_fx(fx, sample_rate);
                }
                if let Some(g) = group {
                    stage.pot_bindings = extract_pot_bindings(g, &edge_indices, graph);

                    // Build biquad lookup table for pot-controlled stages.
                    // Each pot becomes its own dimension (ganged pots sharing a
                    // control label are handled at the BiValve/set_pot level).
                    if !stage.pot_bindings.is_empty() {
                        let labels: Vec<String> = stage
                            .pot_bindings
                            .iter()
                            .map(|b| b.comp_id.clone())
                            .collect();
                        let table_steps = if labels.len() <= 2 { 32 } else { 16 };
                        stage.biquad_table = iir::build_biquad_table(
                            &edge_indices,
                            &pendant_trees,
                            graph,
                            sample_rate,
                            &labels,
                            table_steps,
                        );
                        #[cfg(test)]
                        if let Some(ref t) = stage.biquad_table {
                            eprintln!(
                                "  BiquadTable: {}D, {} steps, {} entries, {:.1}KB",
                                t.dim_labels.len(),
                                t.steps,
                                t.coeffs.len() / 5,
                                t.coeffs.len() as f64 * 8.0 / 1024.0,
                            );
                        }
                    }
                }
                return Ok(BuiltStage::Iir(stage));
            }
            #[cfg(test)]
            eprintln!("  IIR: degenerate b=[0,0,0], falling through");
        }

        // IIR failed → try the MNA-derived state-space form. This is the
        // active-linear path for VCVS filters whose transfer cannot be reduced
        // to the current biquad extractor.
        if let Ok(ss) = state_space::build_state_space_stage(
            &edge_indices,
            &pendant_trees,
            graph,
            sample_rate,
            supply_voltage,
        ) {
            return Ok(BuiltStage::StateSpace(ss));
        }
    }

    // When IIR synthesis is disabled, passive rigid RC/RLC networks still need
    // a cheap compiled form. StateSpace is the nearest non-IIR fallback for
    // linear, non-VCVS subgraphs; active feedback groups continue below into
    // the WDF feedback adaptor cases.
    if !allow_iir && stats.vcvs_count == 0 {
        let pendant_trees = Vec::new();
        if let Ok(ss) = state_space::build_state_space_stage(
            &edge_indices,
            &pendant_trees,
            graph,
            sample_rate,
            supply_voltage,
        ) {
            return Ok(BuiltStage::StateSpace(ss));
        }
    }

    // ── Original classification-specific paths ───────────────────────
    match optimization {
        RigidOptimization::BlackFeedback | RigidOptimization::Iir => {
            // BlackFeedback and IIR both fell through — use WDF.
            // Build an OpAmp WDF stage that handles reactive feedback
            // naturally through wave scattering.
            if stats.vcvs_count == 1 && stats.nl_count == 1 {
                if let Some(g) = group {
                    return build_opamp_nl_feedback(
                        g,
                        &stats,
                        graph,
                        sample_rate,
                        supply_voltage,
                        bias_v_max,
                    )
                    .map(BuiltStage::Wdf);
                }
            }
            if stats.vcvs_count == 1 && stats.nl_count == 0 {
                if let Some(g) = group {
                    if let Some(stage) = try_build_linear_feedback_wdf_adaptor(
                        g,
                        &stats,
                        graph,
                        sample_rate,
                        supply_voltage,
                        bias_v_max,
                    ) {
                        return Ok(BuiltStage::Wdf(stage));
                    }

                    let inverting = is_inverting_topology(&stats, graph);
                    let config = opamp_root::extract_opamp_config(g, inverting, graph)?;
                    let mut fx = collect_nonideal_fx(&edge_indices, graph, sample_rate);
                    // Patch RailSaturation with bias-derived v_max or supply voltage.
                    // NonIdealFx uses symmetric v_max; OpAmpRoot gets asymmetric rails.
                    let actual_v_max = bias_v_max
                        .map(|(pos, neg)| pos.min(neg))
                        .unwrap_or_else(|| (supply_voltage / 2.0 - 1.5).max(0.5));
                    for f in &mut fx {
                        if let super::component::NonIdealFx::RailSaturation { v_max } = f {
                            *v_max = actual_v_max;
                        }
                    }
                    let stage = super::stage::BlackFeedbackStage::new(
                        config.rf,
                        config.ri,
                        inverting,
                        &fx,
                        sample_rate,
                    );
                    return Ok(BuiltStage::BlackFeedback(stage));
                }
            }
            // No VCVS or no group — unity passthrough. In no-IIR diagnostic
            // mode, report the miss instead of silently creating an IIR.
            if allow_iir {
                let iir = super::stage::IirData::new(
                    vec![1.0, 0.0, 0.0],
                    vec![1.0, 0.0, 0.0],
                    sample_rate,
                );
                Ok(BuiltStage::Iir(IirStage::new(iir)))
            } else {
                Err(format!(
                    "IIR disabled: no WDF/StateSpace fallback for rigid group (vcvs={}, nl={}, linear={}, reactive={})",
                    stats.vcvs_count, stats.nl_count, stats.linear_count, stats.reactive_count
                ))
            }
        }
        RigidOptimization::StateSpace => {
            let pendant_trees = Vec::new(); // Empty — input coupling handled by SPQR passive stages
            state_space::build_state_space_stage(
                &edge_indices,
                &pendant_trees,
                graph,
                sample_rate,
                supply_voltage,
            )
            .map(BuiltStage::StateSpace)
        }
        RigidOptimization::General => {
            // VCVS + NL with FlowGroup → op-amp drives NL root.
            // For multiple NL edges (e.g. SD-1's two separate antiparallel diodes),
            // build_opamp_nl_feedback detects antiparallel pairs and synthesizes
            // a DiodePair root from individual diode components.
            if let Some(g) = group {
                if stats.vcvs_count == 1
                    && stats.nl_count > 0
                    && group_nl_edges_are_single_port_solvable(&edge_indices, graph)
                {
                    return build_opamp_nl_feedback(
                        g,
                        &stats,
                        graph,
                        sample_rate,
                        supply_voltage,
                        bias_v_max,
                    )
                    .map(BuiltStage::Wdf);
                }
            }

            // NL present → General MNA + NR solver.
            // Works with or without FlowGroup — uses raw edge indices.
            if stats.nl_count > 0 {
                build_general_mna_from_edges_with_hints(
                    &edge_indices,
                    graph,
                    sample_rate,
                    init_hints,
                )
                .map(BuiltStage::MultiNl)
            } else {
                Err(format!(
                    "General: no NL elements (vcvs={}, nl={}, linear={}, reactive={})",
                    stats.vcvs_count, stats.nl_count, stats.linear_count, stats.reactive_count
                ))
            }
        }
    }
}

/// Build a WDF op-amp constraint adaptor for linear active tone stages whose
/// feedback impedance contains reactive or split-pot elements.
///
/// Heuristic: inverting VCVS, one input resistor into the inverting node, and
/// a feedback network from inverting node to op-amp output containing a cap
/// plus a three-terminal pot. This covers Klon/Goldenrod-style active treble
/// shelves without lowering them to scalar BlackFeedback.
fn try_build_linear_feedback_wdf_adaptor(
    group: &FlowGroup,
    stats: &StageStats,
    graph: &CircuitGraph,
    sample_rate: f64,
    supply_voltage: f64,
    bias_v_max: Option<(f64, f64)>,
) -> Option<WdfStage> {
    if !stats.is_single_vcvs_linear() || !is_inverting_topology(stats, graph) {
        return None;
    }

    let vcvs_edge = stats.vcvs_edge?;
    let vcvs_comp_idx = graph.edges[vcvs_edge].comp_idx;
    let pins = graph
        .nullor_pins
        .iter()
        .find(|p| p.comp_idx == vcvs_comp_idx)?;
    let neg = pins.neg_node;
    let out = pins.out_node;

    let all_edges = group.all_edges();
    let is_ground = |node: NodeId| {
        node == graph.gnd_node
            || graph.supply_nodes.contains(&node)
            || graph.ac_ground_nodes.contains(&node)
    };
    let other = |eidx: usize, node: NodeId| {
        let e = &graph.edges[eidx];
        if e.node_a == node {
            Some(e.node_b)
        } else if e.node_b == node {
            Some(e.node_a)
        } else {
            None
        }
    };
    let spans = |eidx: usize, a: NodeId, b: NodeId| {
        let e = &graph.edges[eidx];
        (e.node_a == a && e.node_b == b) || (e.node_a == b && e.node_b == a)
    };

    let input_edge = all_edges.iter().copied().find(|&eidx| {
        if graph.effective_edge_kind(eidx) != EdgeKind::Linear {
            return false;
        }
        let comp = &graph.components[graph.edges[eidx].comp_idx];
        comp.kind.resistance().is_some()
            && other(eidx, neg).is_some_and(|n| n != out && !is_ground(n))
    })?;
    let input_r = graph.components[graph.edges[input_edge].comp_idx]
        .kind
        .resistance()?;

    let fb_r_edge = all_edges.iter().copied().find(|&eidx| {
        graph.effective_edge_kind(eidx) == EdgeKind::Linear
            && spans(eidx, neg, out)
            && graph.components[graph.edges[eidx].comp_idx]
                .kind
                .resistance()
                .is_some()
    });

    let cap_edge = all_edges.iter().copied().find(|&eidx| {
        graph.effective_edge_kind(eidx) == EdgeKind::Reactive
            && other(eidx, neg).is_some()
            && graph.components[graph.edges[eidx].comp_idx]
                .kind
                .capacitance()
                .is_some()
    })?;
    let cap_to = other(cap_edge, neg)?;
    let cap_comp = &graph.components[graph.edges[cap_edge].comp_idx];
    let cap = cap_comp.kind.capacitance()?;

    let pot_comp_idx = all_edges.iter().find_map(|&eidx| {
        let comp_idx = graph.edges[eidx].comp_idx;
        let count = all_edges
            .iter()
            .filter(|&&other_eidx| graph.edges[other_eidx].comp_idx == comp_idx)
            .count();
        (count >= 2 && graph.components[comp_idx].kind.is_pot()).then_some(comp_idx)
    })?;
    let pot_comp = &graph.components[pot_comp_idx];
    let pot_id = pot_comp.id.as_str();
    let pot_a = graph.node_names.get(&format!("{pot_id}.a")).copied()?;
    let pot_w = graph.node_names.get(&format!("{pot_id}.w")).copied()?;
    let pot_b = graph.node_names.get(&format!("{pot_id}.b")).copied()?;
    if pot_a != cap_to || pot_b != out {
        return None;
    }
    let pot = pot_comp
        .kind
        .as_any()
        .downcast_ref::<super::components::Potentiometer>()?;
    let pot_max = pot.max_r;
    let pot_taper = pot.taper;

    let shelf_edge = all_edges.iter().copied().find(|&eidx| {
        graph.effective_edge_kind(eidx) == EdgeKind::Linear
            && spans(eidx, pot_w, out)
            && graph.edges[eidx].comp_idx != pot_comp_idx
            && graph.components[graph.edges[eidx].comp_idx]
                .kind
                .resistance()
                .is_some()
    })?;
    let shelf_comp = &graph.components[graph.edges[shelf_edge].comp_idx];
    let shelf_r = shelf_comp.kind.resistance()?;

    let zi = DynNode::VoltageSource(0.0, input_r);
    let cap_rp = 1.0 / (2.0 * sample_rate * cap);
    let cap_node = DynNode::Capacitor(Some(cap_comp.id.clone()), cap, cap_rp);
    let pot_aw = DynNode::Pot(format!("{pot_id}__aw"), pot_max, 0.5, pot_taper);
    let pot_wb = DynNode::Pot(format!("{pot_id}__wb"), pot_max, 0.5, pot_taper);
    let shelf = DynNode::Resistor(Some(shelf_comp.id.clone()), shelf_r);
    let shelf_parallel = DynNode::Parallel(Box::new(shelf), Box::new(pot_wb));
    let reactive_branch = DynNode::Series(
        Box::new(DynNode::Series(Box::new(cap_node), Box::new(pot_aw))),
        Box::new(shelf_parallel),
    );
    let zf = if let Some(eidx) = fb_r_edge {
        let comp = &graph.components[graph.edges[eidx].comp_idx];
        let r = comp.kind.resistance()?;
        DynNode::Parallel(
            Box::new(DynNode::Resistor(Some(comp.id.clone()), r)),
            Box::new(reactive_branch),
        )
    } else {
        reactive_branch
    };

    let config = opamp_root::extract_opamp_config(group, true, graph).ok()?;
    let mut opamp = opamp_root::make_opamp_root(&config, sample_rate, supply_voltage, bias_v_max);
    opamp.set_gbw_gain((zf.port_resistance() / input_r).abs().max(1.0));

    let input_node = other(input_edge, neg)?;
    let mut stage = WdfStage::new(
        DynNode::VoltageSource(0.0, 1.0),
        RootKind::Passthrough,
        Oversampler::new(OversamplingFactor::X1),
    );
    stage.injection_node_id = input_node;
    stage.output_node_id = out;
    stage.opamp_wdf_adaptor = Some(OpAmpWdfAdaptor::new(
        zi,
        zf,
        true,
        opamp,
        Some(pot_id.to_string()),
    ));
    Some(stage)
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════
