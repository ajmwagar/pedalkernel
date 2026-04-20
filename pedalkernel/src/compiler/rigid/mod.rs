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
mod opamp_root;
mod state_space;

// ═══════════════════════════════════════════════════════════════════════════
// Re-exports
// ═══════════════════════════════════════════════════════════════════════════

pub(super) use self::general::build_opamp_nl_feedback;
pub(super) use self::opamp_root::{
    extract_opamp_config, make_opamp_root, build_opamp_root, OpAmpConfig,
};

use super::component::EdgeKind;
use super::dyn_node::DynNode;
use super::graph::{CircuitGraph, NodeId};
use super::stage::{RootKind, WdfStage};
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
        for &eidx in edge_indices {
            match graph.effective_edge_kind(eidx) {
                EdgeKind::Vcvs => {
                    stats.vcvs_count += 1;
                    stats.vcvs_edge = Some(eidx);
                }
                EdgeKind::Nonlinear => {
                    stats.nl_count += 1;
                    if stats.nl_edge.is_none() {
                        stats.nl_edge = Some(eidx);
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
    /// All linear, no VCVS → biquad from MNA eigenvalues. O(1)/sample.
    Iir,
    /// Single VCVS, resistive feedback → gain + GBW + slew. O(1)/sample.
    OpAmpRoot { inverting: bool },
    /// VCVS + reactive feedback, no NL → state-space. O(N)/sample.
    StateSpace,
    /// Has NL elements → MNA scattering + Component::solver_hint(). O(N²)/sample.
    General,
}

/// Select the cheapest correct strategy for a Rigid stage.
///
/// Decision rules (applied in cost order):
/// 1. NL present → General (needs NR/WO solver, Component decides which)
/// 2. No VCVS, no NL → Iir (pure passive rigid, e.g. Wheatstone bridge)
/// 3. Single VCVS, all linear, no reactive → OpAmpRoot
/// 4. VCVS + reactive → StateSpace
/// 5. Fallback → General
pub(super) fn classify_rigid(stats: &StageStats, graph: &CircuitGraph) -> RigidOptimization {
    // Rule 1: any NL element forces general MNA + solver
    if stats.nl_count > 0 {
        return RigidOptimization::General;
    }

    // Rule 2: pure passive rigid → IIR
    if stats.is_all_linear() {
        return RigidOptimization::Iir;
    }

    // Rule 3: single VCVS, resistive-only feedback → OpAmpRoot
    if stats.is_single_vcvs_linear() && stats.reactive_count == 0 {
        let inverting = is_inverting_topology(stats, graph);
        return RigidOptimization::OpAmpRoot { inverting };
    }

    // Rule 4: VCVS + reactive (caps/inductors in feedback) → StateSpace
    if stats.vcvs_count >= 1 && stats.reactive_count > 0 && stats.nl_count == 0 {
        return RigidOptimization::StateSpace;
    }

    // Rule 5: fallback
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
pub(super) fn build_rigid(
    edge_indices: Vec<usize>,
    _boundary_nodes: Vec<NodeId>,
    pendant_trees: Vec<(DynNode, NodeId)>,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<WdfStage, String> {
    let stats = StageStats::from_edges(&edge_indices, graph);
    let optimization = classify_rigid(&stats, graph);

    match optimization {
        RigidOptimization::OpAmpRoot { inverting } => {
            build_opamp_root(&edge_indices, &pendant_trees, inverting, graph, sample_rate)
        }
        RigidOptimization::Iir => {
            iir::build_iir_stage()
        }
        RigidOptimization::StateSpace => {
            state_space::build_state_space_stage()
        }
        RigidOptimization::General => {
            // Common case: VCVS + NL (TS/RAT/Klon clipping stages).
            // OpAmpRoot handles gain+GBW+slew, NL root handles clipping.
            if stats.vcvs_count == 1 && stats.nl_count > 0 {
                build_opamp_nl_feedback(
                    &edge_indices,
                    &pendant_trees,
                    &stats,
                    graph,
                    sample_rate,
                )
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

