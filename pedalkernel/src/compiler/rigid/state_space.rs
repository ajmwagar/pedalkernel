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
use super::super::stage::{StateSpaceData, StateSpacePotBinding, StateSpaceStage};
use super::mna_builder::build_mna;
use pedalkernel_rt::boundary_math::{MnaNodeId, MnaPortTerminals};

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

    if built.reactive_one_ports.is_empty() {
        return Err("StateSpace: no reactive elements found".to_string());
    }

    let (a_d, b_d, c_out, n_states, d_feedthrough) = built.mna.build_state_space_matrices(
        &built.reactive_one_ports,
        built.vs_idx,
        built.output_mna,
        None,
        sample_rate,
    );

    if n_states == 0 {
        return Err("StateSpace: zero states after matrix construction".to_string());
    }

    let node_to_mna = |node: NodeId| -> Option<usize> {
        if node == graph.gnd_node
            || graph.supply_nodes.contains(&node)
            || graph.ac_ground_nodes.contains(&node)
        {
            None
        } else {
            built.node_set.iter().position(|&n| n == node)
        }
    };

    let mut pot_bindings = Vec::new();
    for &eidx in edge_indices {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        if !comp.kind.is_pot() {
            continue;
        }

        let comp_edge_count = edge_indices
            .iter()
            .filter(|&&other| graph.edges[other].comp_idx == e.comp_idx)
            .count();
        if comp_edge_count != 1 {
            continue;
        }

        let Some(pot) = comp
            .kind
            .as_any()
            .downcast_ref::<super::super::components::Potentiometer>()
        else {
            continue;
        };
        let position = 0.5;
        let conductance = 1.0 / (pot.taper.apply(position) * pot.max_r).max(1.0);
        pot_bindings.push(StateSpacePotBinding {
            comp_id: comp.id.clone(),
            max_r: pot.max_r,
            taper: pot.taper,
            position,
            terminals: MnaPortTerminals::maybe_differential(
                node_to_mna(e.node_a).map(MnaNodeId::new),
                node_to_mna(e.node_b).map(MnaNodeId::new),
            ),
            conductance,
        });
    }

    let ss = StateSpaceData {
        x: vec![0.0; n_states],
        a_matrix: a_d,
        b_vector: b_d,
        c_vector: c_out,
        n_states,
        reactive_one_ports: built.reactive_one_ports,
        vs_idx: built.vs_idx,
        output_pos: built.output_mna,
        output_neg: None,
        sample_rate,
        d_feedthrough,
        prev_output: 0.0,
        variable_resistors: Vec::new(),
    };

    let mut stage = StateSpaceStage::new(ss, supply_voltage);
    stage.pot_bindings = pot_bindings;
    if !stage.pot_bindings.is_empty() {
        stage.recompute_mna = Some(built.mna);
    }

    Ok(stage)
}
