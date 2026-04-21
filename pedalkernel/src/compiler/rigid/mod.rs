//! Rigid stage strategy selection and building.
//!
//! Scans a Rigid stage's edges once via `StageStats`, then applies
//! decision rules to select the cheapest correct runtime strategy.
//! All decisions driven by `Component` trait queries — no topology
//! pattern matching.
//!
//! Cost ordering (cheapest first):
//! 1. **Iir** — all linear → biquad from MNA. O(1)/sample.
//! 2. **OpAmpRoot** — simple VCVS feedback → gain + GBW + slew. O(1)/sample.
//! 3. **StateSpace** — VCVS + reactive, no NL → Schur complement. O(N)/sample.
//! 4. **General** — has NL → MNA scattering + solver from Component. O(N²)/sample.

mod general;
mod iir;
mod mna_builder;
mod opamp_root;
mod state_space;

// ═══════════════════════════════════════════════════════════════════════════
// Re-exports
// ═══════════════════════════════════════════════════════════════════════════

pub(super) use self::general::{build_general_mna, build_opamp_nl_feedback};
pub(super) use self::opamp_root::{
    extract_opamp_config, make_opamp_root, build_opamp_root, OpAmpConfig,
};

use super::component::EdgeKind;
use super::dyn_node::DynNode;
use super::signal_flow::FlowGroup;
use super::graph::{CircuitGraph, NodeId};
use super::spqr_build::BuiltStage;
use super::stage::{IirStage, RootKind, StateSpaceStage, WdfStage};
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

// ═══════════════════════════════════════════════════════════════════════════
// RigidOptimization — strategy selection
// ═══════════════════════════════════════════════════════════════════════════

/// Runtime strategy for a Rigid stage, selected at compile time.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum RigidOptimization {
    /// All linear (with or without VCVS) → biquad from MNA. O(1)/sample.
    /// Covers passive bridges AND active linear circuits (808 bridged-T).
    Iir,
    /// Single VCVS, resistive feedback → gain + GBW + slew. O(1)/sample.
    OpAmpRoot { inverting: bool },
    /// Linear but IIR-incompatible (>2nd order) → state-space. O(N)/sample.
    StateSpace,
    /// Has NL elements → MNA scattering + Component::solver_hint(). O(N²)/sample.
    General,
}

/// Select the cheapest correct strategy for a Rigid stage.
///
/// Decision rules (applied in cost order):
/// 1. NL present → General (needs NR/WO solver, Component decides which)
/// 2. Single VCVS, no reactive, no NL → OpAmpRoot (pure gain)
/// 3. All linear (no NL) → Iir (biquad from MNA, covers passive + VCVS)
/// 4. Fallback → General
pub(super) fn classify_rigid(
    stats: &StageStats,
    graph: &CircuitGraph,
    group: Option<&FlowGroup>,
) -> RigidOptimization {
    // Rule 1: any NL element forces general MNA + solver
    if stats.nl_count > 0 {
        return RigidOptimization::General;
    }

    // Rule 2: single VCVS, reactive ground shunts, resistive feedback → IIR
    // Resonator circuits (808 bridged-T): caps to GND create resonance,
    // feedback path is purely resistive. IIR captures the dynamics.
    if stats.is_single_vcvs_linear() {
        if let Some(g) = group {
            let is_reactive = |eidx: usize| -> bool {
                let comp = &graph.components[graph.edges[eidx].comp_idx];
                comp.kind.capacitance().is_some() || comp.kind.inductance().is_some()
            };
            let has_reactive_ground = g.ground_shunt_edges.iter().any(|&eidx| is_reactive(eidx));
            let feedback_purely_resistive = g.feedback_edges.iter().all(|&eidx| !is_reactive(eidx));
            if has_reactive_ground && feedback_purely_resistive {
                return RigidOptimization::Iir;
            }
        }
    }

    // Rule 3: single VCVS, no NL → OpAmpRoot
    // Reactive elements in feedback (e.g., 100pF HF rolloff cap) are
    // absorbed by the OpAmpRoot's GBW model.
    if stats.is_single_vcvs_linear() {
        let inverting = is_inverting_topology(stats, graph);
        return RigidOptimization::OpAmpRoot { inverting };
    }

    // Rule 3: all linear with reactive elements → IIR from MNA
    // This covers both passive bridges AND active linear circuits
    // (808 bridged-T with VCVS + caps). VCVS is linear.
    // Must have at least 1 reactive element for biquad coefficients.
    if stats.nl_count == 0 && stats.reactive_count > 0 {
        return RigidOptimization::Iir;
    }

    // Rule 3b: all linear, purely resistive → passthrough (no dynamics)
    // These stages are just resistive dividers or loads.
    if stats.nl_count == 0 && stats.reactive_count == 0 && stats.vcvs_count == 0 {
        return RigidOptimization::Iir; // Will produce trivial biquad (DC gain)
        // TODO: could optimize to just a gain stage
    }

    // Rule 4: fallback
    RigidOptimization::General
}

/// Determine if a single-VCVS stage is inverting or non-inverting.
///
/// Inverting: pos pin is at ground (or AC ground).
/// Non-inverting: pos pin carries signal (not ground).
pub(super) fn is_inverting_topology(stats: &StageStats, graph: &CircuitGraph) -> bool {
    let vcvs_idx = match stats.vcvs_edge {
        Some(idx) => idx,
        None => return true,
    };
    let edge = &graph.edges[vcvs_idx];

    // Find the NullorPinRecord for this op-amp component
    for rec in &graph.nullor_pins {
        if rec.comp_idx == edge.comp_idx {
            // Inverting if pos is ground or AC-ground (bypassed supply rail)
            return rec.pos_node == graph.gnd_node
                || graph.ac_ground_nodes.contains(&rec.pos_node);
        }
    }

    true // Default: inverting
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

    match optimization {
        RigidOptimization::OpAmpRoot { inverting } => {
            if let Some(g) = group {
                build_opamp_root(g, inverting, graph, sample_rate).map(BuiltStage::Wdf)
            } else {
                Err("OpAmpRoot needs classified FlowGroup".to_string())
            }
        }
        RigidOptimization::Iir => {
            let pendant_trees = Vec::new();
            // Try IIR (≤2 states). If too many states, use StateSpace.
            match iir::build_iir_stage(&edge_indices, &pendant_trees, graph, sample_rate) {
                Ok(iir) => Ok(BuiltStage::Iir(IirStage::new(iir))),
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
            if let Some(g) = group {
                if stats.vcvs_count == 1 && stats.nl_count > 0 {
                    build_opamp_nl_feedback(g, &stats, graph, sample_rate)
                        .map(BuiltStage::Wdf)
                } else if stats.nl_count > 0 {
                    // NL case (with or without VCVS): BJTs, coupled diodes, etc.
                    // For multi-VCVS + NL (blues), the op-amps are stamped as
                    // nullors in the MNA and become part of the scattering matrix.
                    build_general_mna(g, graph, sample_rate)
                        .map(BuiltStage::MultiNl)
                } else {
                    Err(format!(
                        "General MNA not yet implemented (vcvs={}, nl={}, linear={}, reactive={})",
                        stats.vcvs_count, stats.nl_count, stats.linear_count, stats.reactive_count
                    ))
                }
            } else {
                Err(format!(
                    "General MNA not yet implemented (vcvs={}, nl={}, linear={}, reactive={})",
                    stats.vcvs_count, stats.nl_count, stats.linear_count, stats.reactive_count
                ))
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

