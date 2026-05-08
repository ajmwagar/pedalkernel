//! State-space rigid stage builder.
//!
//! Builds an MNA system from the rigid stage's edges, stamps all
//! components (resistors, caps, VCVS), and calls
//! `MnaSystem::build_state_space_matrices()` to get (A, b, c, d).
//! O(N²)/sample at runtime where N = number of states.
//!
//! Covers linear circuits with 3+ reactive elements that can't reduce
//! to a biquad: Klon Centaur stages, BB Preamp, complex active filters.

use super::super::dyn_node::DynNode;
use super::super::graph::{CircuitGraph, NodeId};
use super::super::stage::{StateSpaceData, StateSpaceStage};
use super::mna_builder::build_mna;

/// Build a state-space stage from a linear rigid R-node with 3+ reactive elements.
///
/// 1. Builds MNA via shared `build_mna()` (node collection + stamping)
/// 2. Calls `build_state_space_matrices()` to get discrete-time matrices
/// 3. Returns StateSpaceStage
pub(in crate::compiler) fn build_state_space_stage(
    edge_indices: &[usize],
    pendant_trees: &[(DynNode, NodeId)],
    graph: &CircuitGraph,
    sample_rate: f64,
    supply_voltage: f64,
) -> Result<StateSpaceStage, String> {
    let built = build_mna(edge_indices, pendant_trees, graph, sample_rate)?;

    if built.cap_stamps.is_empty() {
        return Err("StateSpace: no reactive elements found".to_string());
    }

    let (a_d, b_d, c_out, n_states, d_feedthrough) = built.mna.build_state_space_matrices(
        &built.cap_stamps,
        built.vs_idx,
        built.output_mna,
        None,
        sample_rate,
    );

    if n_states == 0 {
        return Err("StateSpace: zero states after matrix construction".to_string());
    }

    let ss = StateSpaceData {
        x: vec![0.0; n_states],
        a_matrix: a_d,
        b_vector: b_d,
        c_vector: c_out,
        n_states,
        cap_stamps: built.cap_stamps,
        vs_idx: built.vs_idx,
        output_pos: built.output_mna,
        output_neg: None,
        sample_rate,
        d_feedthrough,
        prev_output: 0.0,
        pot_stamps: Vec::new(),
    };

    Ok(StateSpaceStage::new(ss, supply_voltage))
}
