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
            return rec.pos_node == graph.gnd_node
                || graph.ac_ground_nodes.contains(&rec.pos_node);
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

    match optimization {
        RigidOptimization::BlackFeedback => {
            // Black's feedback formula: closed-form gain from Rf/Ri.
            // Requires FlowGroup classification (feedback_edges → Rf, pendant_edges → Ri).
            if let Some(g) = group {
                let inverting = is_inverting_topology(&stats, graph);
                let config = opamp_root::extract_opamp_config(g, inverting, graph)?;
                let opamp = opamp_root::make_opamp_root(&config, sample_rate);

                // Build WDF tree from pendant edges (input coupling)
                let tree = general::build_pendant_tree(&g.pendant_edges, graph, sample_rate);

                let oversampler = crate::oversampling::Oversampler::new(
                    crate::oversampling::OversamplingFactor::X1,
                );
                let root = super::stage::RootKind::OpAmp(opamp);
                Ok(BuiltStage::Wdf(super::stage::WdfStage::new(tree, root, oversampler)))
            } else {
                // No FlowGroup → can't classify Rf/Ri. Fall back to IIR.
                // This path is rare (only from build_spqr_stage without signal flow).
                let pendant_trees = Vec::new();
                match iir::build_iir_stage(&edge_indices, &pendant_trees, graph, sample_rate) {
                    Ok(iir_data) => Ok(BuiltStage::Iir(IirStage::new(iir_data))),
                    Err(e) => Err(format!("BlackFeedback without FlowGroup: IIR fallback failed: {e}")),
                }
            }
        }
        RigidOptimization::Iir => {
            let pendant_trees = Vec::new();
            // Try IIR (≤2 states). If too many states, use StateSpace.
            match iir::build_iir_stage(&edge_indices, &pendant_trees, graph, sample_rate) {
                Ok(iir_data) => {
                    let mut stage = IirStage::new(iir_data);
                    // Collect NonIdealFx from any op-amp components in this stage.
                    let fx = collect_nonideal_fx(&edge_indices, graph, sample_rate);
                    if !fx.is_empty() {
                        stage.set_nonideal_fx(fx, sample_rate);
                    }
                    // Extract pot bindings for runtime coefficient recomputation.
                    if let Some(g) = group {
                        stage.pot_bindings = extract_pot_bindings(g, &edge_indices, graph);
                    }
                    Ok(BuiltStage::Iir(stage))
                }
                Err(_) => {
                    // IIR failed (>2 states or other issue) → StateSpace
                    let supply_voltage = 9.0; // TODO: pass from PedalDef
                    state_space::build_state_space_stage(
                        &edge_indices, &pendant_trees, graph, sample_rate, supply_voltage,
                    )
                    .map(BuiltStage::StateSpace)
                }
            }
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
            // VCVS + NL with FlowGroup → op-amp drives NL root (TS/RAT/Klon)
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

