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

pub(super) use self::general::{build_general_mna, build_general_mna_from_edges, build_opamp_nl_feedback};
// Legacy re-exports for old pipeline — will be removed with opamp_analysis.rs
pub(super) use self::opamp_root::{
    extract_opamp_config, make_opamp_root, build_opamp_root, OpAmpConfig,
};

use super::component::EdgeKind;
use super::dyn_node::DynNode;
use super::signal_flow::FlowGroup;
use super::graph::{CircuitGraph, NodeId};
use super::spqr_build::BuiltStage;
use super::stage::{IirPotBinding, IirStage};

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
    if stats.is_single_vcvs_linear() {
        if let Some(g) = _group {
            let is_reactive = |eidx: usize| -> bool {
                let comp = &graph.components[graph.edges[eidx].comp_idx];
                comp.kind.capacitance().is_some() || comp.kind.inductance().is_some()
            };
            let has_reactive_ground = g.ground_shunt_edges.iter().any(|&eidx| is_reactive(eidx));
            let feedback_purely_resistive = g.feedback_edges.iter().all(|&eidx| !is_reactive(eidx));
            if has_reactive_ground && feedback_purely_resistive {
                // 808-style: caps to ground create resonance, feedback is resistive.
                // IIR with extract_feedback_r handles this correctly.
                return RigidOptimization::Iir;
            }
        }
        return RigidOptimization::BlackFeedback;
    }

    // Rule 3: all linear, no VCVS → IIR biquad from MNA
    // Reactive elements give biquad dynamics; purely resistive gives DC gain.
    if stats.nl_count == 0 {
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
            if rec.pos_node == graph.gnd_node
                || graph.ac_ground_nodes.contains(&rec.pos_node)
            {
                return true;
            }

            // Check if pos_node is a bias point: BFS from pos through
            // passive edges (excluding neg/out to avoid traversing the
            // feedback network). If pos reaches ANY rail → it's biased
            // to a fixed DC point → inverting.
            let mut blocked = std::collections::HashSet::new();
            blocked.insert(rec.neg_node);
            blocked.insert(rec.out_node);

            let mut visited = std::collections::HashSet::new();
            let mut queue = std::collections::VecDeque::new();
            visited.insert(rec.pos_node);
            queue.push_back(rec.pos_node);

            let mut reaches_rail = false;
            let mut reaches_signal = false;
            while let Some(node) = queue.pop_front() {
                for e in &graph.edges {
                    let comp = &graph.components[e.comp_idx];
                    if !comp.kind.is_passive() { continue; }
                    let next = if e.node_a == node && !visited.contains(&e.node_b) {
                        Some(e.node_b)
                    } else if e.node_b == node && !visited.contains(&e.node_a) {
                        Some(e.node_a)
                    } else {
                        None
                    };
                    if let Some(n) = next {
                        if blocked.contains(&n) { continue; }
                        if n == graph.in_node {
                            reaches_signal = true;
                        }
                        if n == graph.gnd_node || n == graph.vcc_node
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
/// For each pot in the feedback path, computes:
/// - `ri`: sum of resistance on pendant_edges (input coupling)
/// - `fixed_series_r`: sum of non-pot resistance on feedback_edges
/// - `max_r`: pot's maximum resistance
///
/// No heap allocations at runtime — all values are scalars stored at compile time.
fn extract_pot_bindings(
    group: &FlowGroup,
    _edge_indices: &[usize],
    graph: &CircuitGraph,
) -> Vec<IirPotBinding> {
    let mut bindings = Vec::new();

    // Ri: sum of resistance on pendant_edges (input coupling path)
    let ri: f64 = group
        .pendant_edges
        .iter()
        .filter_map(|&eidx| graph.components[graph.edges[eidx].comp_idx].kind.resistance())
        .sum();
    let ri = if ri <= 0.0 { 1.0 } else { ri }; // Safety: avoid div-by-zero

    // Scan feedback edges for pots
    let mut fixed_r: f64 = 0.0;
    for &eidx in &group.feedback_edges {
        let comp = &graph.components[graph.edges[eidx].comp_idx];
        if let Some(pot) = comp.kind.as_any().downcast_ref::<crate::compiler::components::Potentiometer>() {
            bindings.push(IirPotBinding {
                comp_id: comp.id.clone(),
                max_r: pot.max_r,
                fixed_series_r: 0.0, // filled in below
                ri,
                position: 0.5, // default
            });
        } else if let Some(r) = comp.kind.resistance() {
            fixed_r += r;
        }
    }

    // Set fixed_series_r on all pot bindings
    for b in &mut bindings {
        b.fixed_series_r = fixed_r;
    }

    bindings
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
            DynNode::Leaf(Box::new(crate::compiler::wdf_leaf::WdfPot {
                comp_id: id.clone(),
                max_resistance: 100_000.0,
                position: 0.5,
                taper: crate::dsl::PotTaper::B,
                rp: 50_000.0,
                last_a: 0.0,
                complement: false,
            }))
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
    build_rigid_from_group(edge_indices, graph, sample_rate, None)
}

/// Build from a FlowGroup with full classification.
pub(super) fn build_rigid_from_group(
    edge_indices: Vec<usize>,
    graph: &CircuitGraph,
    sample_rate: f64,
    group: Option<&FlowGroup>,
) -> Result<BuiltStage, String> {
    let stats = StageStats::from_edges(&edge_indices, graph);
    let optimization = classify_rigid(&stats, graph, group);

    #[cfg(test)]
    eprintln!("  rigid: {:?} (vcvs={}, nl={}, linear={}, reactive={}, edges={})",
        optimization, stats.vcvs_count, stats.nl_count, stats.linear_count, stats.reactive_count, edge_indices.len());

    // ── Single-pass fallthrough: cheapest → most expensive ──────────────
    // Each strategy attempts to build a stage. On failure (Err or degenerate
    // result), fall through to the next. No loops, no recovery.
    //
    // Order: BlackFeedback → IIR → StateSpace → General (WDF)
    // classify_rigid picks the entry point; we fall through from there.

    let try_black_feedback = matches!(optimization, RigidOptimization::BlackFeedback);
    let try_iir = matches!(optimization, RigidOptimization::BlackFeedback | RigidOptimization::Iir);

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
                    config.rf, config.ri, inverting, &fx, sample_rate,
                );

                for &eidx in &g.all_edges() {
                    let comp = &graph.components[graph.edges[eidx].comp_idx];
                    if comp.kind.is_pot() {
                        if let Some(max_r) = comp.kind.resistance() {
                            stage.pot_comp_id = Some(comp.id.clone());
                            stage.pot_max_r = max_r;
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

    // ── IIR: O(1)/sample, ≤2 reactive elements ──────────────────────
    if try_iir || try_black_feedback {
        let pendant_trees = Vec::new();

        // Skip rail-only groups (no signal nodes)
        let has_signal_node = edge_indices.iter().any(|&eidx| {
            let e = &graph.edges[eidx];
            let a_rail = e.node_a == graph.gnd_node || graph.supply_nodes.contains(&e.node_a);
            let b_rail = e.node_b == graph.gnd_node || graph.supply_nodes.contains(&e.node_b);
            !a_rail || !b_rail
        });
        if !has_signal_node {
            let iir = super::stage::IirData::new(vec![1.0, 0.0, 0.0], vec![1.0, 0.0, 0.0], sample_rate);
            return Ok(BuiltStage::Iir(IirStage::new(iir)));
        }

        if let Ok(iir_data) = iir::build_iir_stage(&edge_indices, &pendant_trees, graph, sample_rate) {
            // Validate: check for degenerate transfer function (all-zero numerator)
            let has_signal = iir_data.b_coeffs.iter().any(|&b| b.abs() > 1e-15);
            if has_signal {
                let mut stage = IirStage::new(iir_data);
                let fx = collect_nonideal_fx(&edge_indices, graph, sample_rate);
                if !fx.is_empty() {
                    stage.set_nonideal_fx(fx, sample_rate);
                }
                if let Some(g) = group {
                    stage.pot_bindings = extract_pot_bindings(g, &edge_indices, graph);
                }
                return Ok(BuiltStage::Iir(stage));
            }
            #[cfg(test)]
            eprintln!("  IIR: degenerate b=[0,0,0], falling through");
        }

        // IIR failed → try StateSpace (only for non-VCVS stages).
        // VCVS stages with reactive feedback should fall through to WDF
        // where the op-amp + cap feedback is handled by wave scattering.
        if stats.vcvs_count == 0 {
            let supply_voltage = 9.0;
            if let Ok(ss) = state_space::build_state_space_stage(
                &edge_indices, &pendant_trees, graph, sample_rate, supply_voltage,
            ) {
                return Ok(BuiltStage::StateSpace(ss));
            }
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
                    return build_opamp_nl_feedback(g, &stats, graph, sample_rate)
                        .map(BuiltStage::Wdf);
                }
            }
            if stats.vcvs_count == 1 && stats.nl_count == 0 {
                // Linear VCVS + reactive feedback (tone stages).
                // TODO: build OpAmpWdfAdaptor with Zf tree from feedback edges.
                // For now, use BlackFeedback with unity passthrough — audio
                // passes through but tone pot has no effect.
                if let Some(g) = group {
                    let inverting = is_inverting_topology(&stats, graph);
                    let config = opamp_root::extract_opamp_config(g, inverting, graph)?;
                    let fx = collect_nonideal_fx(&edge_indices, graph, sample_rate);
                    let stage = super::stage::BlackFeedbackStage::new(
                        config.rf, config.ri, inverting, &fx, sample_rate,
                    );
                    return Ok(BuiltStage::BlackFeedback(stage));
                }
            }
            // No VCVS or no group — unity passthrough
            let iir = super::stage::IirData::new(vec![1.0, 0.0, 0.0], vec![1.0, 0.0, 0.0], sample_rate);
            Ok(BuiltStage::Iir(IirStage::new(iir)))
        }
        RigidOptimization::StateSpace => {
            let pendant_trees = Vec::new(); // TODO: extract from group
            let supply_voltage = 9.0; // TODO: pass from PedalDef
            state_space::build_state_space_stage(
                &edge_indices, &pendant_trees, graph, sample_rate, supply_voltage,
            )
            .map(BuiltStage::StateSpace)
        }
        RigidOptimization::General => {
            // VCVS + NL with FlowGroup → op-amp drives NL root
            if let Some(g) = group {
                if stats.vcvs_count == 1 && stats.nl_count > 0 {
                    return build_opamp_nl_feedback(g, &stats, graph, sample_rate)
                        .map(BuiltStage::Wdf);
                }
            }

            // NL present → General MNA + NR solver.
            // Works with or without FlowGroup — uses raw edge indices.
            if stats.nl_count > 0 {
                build_general_mna_from_edges(&edge_indices, graph, sample_rate)
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

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

