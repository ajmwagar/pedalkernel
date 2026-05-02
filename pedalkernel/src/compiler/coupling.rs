//! Coupling barrier analysis for optimization legality.
//!
//! Before lowering a subgraph to IIR or BlackFeedback, the compiler must
//! verify that no nonlinear or control-dependent port is electrically coupled
//! to internal reactive/feedback nodes. These functions query the CircuitGraph
//! using port semantics from the Component trait.

use super::component::PortSemantic;
use super::graph::{CircuitGraph, NodeId};
use hashbrown::HashSet;

/// Get the port semantic for a circuit graph edge.
///
/// Uses the component's edges() to find the matching pin pair,
/// then queries port_semantic(). Falls back to the default
/// (derived from is_nonlinear/is_variable/capacitance/inductance).
fn edge_port_semantic(eidx: usize, graph: &CircuitGraph) -> PortSemantic {
    let edge = &graph.edges[eidx];
    let comp = &graph.components[edge.comp_idx];

    // Find the pin pair for this edge from the component's declared edges.
    // Most components have a single edge, so the first match works.
    let comp_edges = comp.kind.edges();
    if let Some(ce) = comp_edges.first() {
        comp.kind.port_semantic(ce.pin_a, ce.pin_b)
    } else {
        // Fallback: use default (no pin info available)
        comp.kind.port_semantic("", "")
    }
}

/// Returns true if any edge in the subgraph has a nonlinear or
/// control-dependent port — an optimization barrier.
pub(super) fn has_coupling_barrier(edge_indices: &[usize], graph: &CircuitGraph) -> bool {
    edge_indices.iter().any(|&eidx| {
        let sem = edge_port_semantic(eidx, graph);
        matches!(
            sem,
            PortSemantic::Nonlinear | PortSemantic::ControlledConductance
        )
    })
}

/// Returns true if any edge in the subgraph has reactive state (C, L).
pub(super) fn has_reactive_state(edge_indices: &[usize], graph: &CircuitGraph) -> bool {
    edge_indices.iter().any(|&eidx| {
        let sem = edge_port_semantic(eidx, graph);
        matches!(sem, PortSemantic::Reactive)
    })
}

/// Returns true if a nonlinear/controlled edge shares a circuit node with
/// a reactive edge in the same subgraph.
///
/// This is the key coupling test: if a BJT's collector shares a node with
/// a capacitor in a bridged-T, the subgraph has NL-reactive coupling and
/// MUST NOT be lowered to IIR (which would lose the nonlinear interaction).
pub(super) fn has_nl_reactive_coupling(edge_indices: &[usize], graph: &CircuitGraph) -> bool {
    let mut reactive_nodes: HashSet<NodeId> = HashSet::new();
    let mut nl_nodes: HashSet<NodeId> = HashSet::new();

    for &eidx in edge_indices {
        let edge = &graph.edges[eidx];
        let sem = edge_port_semantic(eidx, graph);
        match sem {
            PortSemantic::Reactive => {
                reactive_nodes.insert(edge.node_a);
                reactive_nodes.insert(edge.node_b);
            }
            PortSemantic::Nonlinear | PortSemantic::ControlledConductance => {
                nl_nodes.insert(edge.node_a);
                nl_nodes.insert(edge.node_b);
            }
            _ => {}
        }
    }

    !reactive_nodes.is_disjoint(&nl_nodes)
}
