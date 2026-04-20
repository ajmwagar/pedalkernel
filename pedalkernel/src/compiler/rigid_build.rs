//! Rigid stage strategy selection.
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

use super::component::EdgeKind;
use super::graph::CircuitGraph;

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
fn is_inverting_topology(stats: &StageStats, graph: &CircuitGraph) -> bool {
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
// Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    fn make_graph_all_edges(pedal_src: &str) -> (CircuitGraph, Vec<usize>) {
        let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse failed");
        let graph = CircuitGraph::from_pedal(&pedal);
        let active_set: std::collections::HashSet<usize> =
            graph.active_edge_indices.iter().copied().collect();
        let all_edges: Vec<usize> = (0..graph.edges.len())
            .filter(|i| !active_set.contains(i))
            .collect();
        (graph, all_edges)
    }

    fn make_graph_passive(pedal_src: &str) -> (CircuitGraph, Vec<usize>) {
        let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse failed");
        let graph = CircuitGraph::from_pedal(&pedal);
        let active_set: std::collections::HashSet<usize> =
            graph.active_edge_indices.iter().copied().collect();
        let passive_edges: Vec<usize> = (0..graph.edges.len())
            .filter(|i| !active_set.contains(i))
            .filter(|&i| graph.components[graph.edges[i].comp_idx].kind.is_passive())
            .collect();
        (graph, passive_edges)
    }

    #[test]
    fn stats_passive_bridge_is_iir() {
        let (graph, edges) = make_graph_passive(r#"
            pedal "test" { supply 9V
                components {
                    R1: resistor(10k)  R2: resistor(10k)
                    R3: resistor(10k)  R4: resistor(10k)
                    R5: resistor(10k)
                }
                nets {
                    in -> R1.a, R3.a
                    R1.b -> R2.a, R5.a
                    R3.b -> R4.a, R5.b
                    R2.b -> out  R4.b -> out
                }
                controls {}
            }"#);
        let stats = StageStats::from_edges(&edges, &graph);
        assert!(stats.is_all_linear());
        assert_eq!(stats.linear_count, 5);
        assert_eq!(classify_rigid(&stats, &graph), RigidOptimization::Iir);
    }

    #[test]
    fn stats_inverting_opamp_is_opamproot() {
        let (graph, edges) = make_graph_all_edges(r#"
            pedal "test" { supply 9V
                components {
                    R1: resistor(10k)
                    Rf: resistor(100k)
                    U1: opamp(tl072)
                }
                nets {
                    in -> R1.a
                    R1.b -> U1.neg
                    Rf.a -> U1.neg
                    Rf.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> out
                }
                controls {}
            }"#);
        let stats = StageStats::from_edges(&edges, &graph);
        assert_eq!(stats.vcvs_count, 1);
        assert_eq!(stats.nl_count, 0);
        assert_eq!(stats.linear_count, 2);
        assert_eq!(
            classify_rigid(&stats, &graph),
            RigidOptimization::OpAmpRoot { inverting: true }
        );
    }

    #[test]
    fn stats_noninverting_opamp_is_opamproot() {
        let (graph, edges) = make_graph_all_edges(r#"
            pedal "test" { supply 9V
                components {
                    R1: resistor(10k)
                    Rf: resistor(100k)
                    U1: opamp(tl072)
                }
                nets {
                    in -> U1.pos
                    U1.neg -> R1.a
                    R1.b -> gnd
                    U1.neg -> Rf.a
                    Rf.b -> U1.out
                    U1.out -> out
                }
                controls {}
            }"#);
        let stats = StageStats::from_edges(&edges, &graph);
        assert_eq!(
            classify_rigid(&stats, &graph),
            RigidOptimization::OpAmpRoot { inverting: false }
        );
    }

    #[test]
    fn stats_opamp_with_feedback_cap_is_statesp() {
        let (graph, edges) = make_graph_all_edges(r#"
            pedal "test" { supply 9V
                components {
                    R1: resistor(10k)
                    Rf: resistor(100k)
                    Cf: cap(100p)
                    U1: opamp(tl072)
                }
                nets {
                    in -> R1.a
                    R1.b -> U1.neg
                    Rf.a -> U1.neg
                    Rf.b -> U1.out
                    Cf.a -> U1.neg
                    Cf.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> out
                }
                controls {}
            }"#);
        let stats = StageStats::from_edges(&edges, &graph);
        assert_eq!(stats.vcvs_count, 1);
        assert_eq!(stats.reactive_count, 1);
        assert_eq!(classify_rigid(&stats, &graph), RigidOptimization::StateSpace);
    }

    #[test]
    fn stats_diode_in_rigid_is_general() {
        let (graph, edges) = make_graph_all_edges(r#"
            pedal "test" { supply 9V
                components {
                    R1: resistor(10k)
                    D1: diode(silicon)
                    Rf: resistor(100k)
                    U1: opamp(tl072)
                }
                nets {
                    in -> R1.a
                    R1.b -> U1.neg
                    D1.a -> U1.neg
                    D1.b -> U1.out
                    Rf.a -> U1.neg
                    Rf.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> out
                }
                controls {}
            }"#);
        let stats = StageStats::from_edges(&edges, &graph);
        assert!(stats.nl_count > 0, "Should detect diode as NL");
        assert_eq!(classify_rigid(&stats, &graph), RigidOptimization::General);
    }

    #[test]
    fn stats_counts_are_correct() {
        let (graph, edges) = make_graph_all_edges(r#"
            pedal "test" { supply 9V
                components {
                    R1: resistor(10k)
                    C1: cap(100n)
                    D1: diode(silicon)
                    U1: opamp(tl072)
                }
                nets {
                    in -> R1.a
                    R1.b -> C1.a
                    C1.b -> U1.neg
                    D1.a -> U1.neg
                    D1.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> out
                }
                controls {}
            }"#);
        let stats = StageStats::from_edges(&edges, &graph);
        assert_eq!(stats.total, 4, "R + C + D + U1");
        assert_eq!(stats.linear_count, 1, "R1");
        assert_eq!(stats.reactive_count, 1, "C1");
        assert_eq!(stats.nl_count, 1, "D1");
        assert_eq!(stats.vcvs_count, 1, "U1");
    }
}
