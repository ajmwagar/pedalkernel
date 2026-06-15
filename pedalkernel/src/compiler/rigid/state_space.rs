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

    // Track which pot components have already been bound so a 3-terminal pot
    // (two edges) is handled exactly once on its first edge.
    let mut bound_pots: std::collections::HashSet<usize> = std::collections::HashSet::new();
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

        let Some(pot) = comp
            .kind
            .as_any()
            .downcast_ref::<super::super::components::Potentiometer>()
        else {
            continue;
        };

        match comp_edge_count {
            // ── 2-terminal rheostat (a→b, no wiper) ────────────────────────
            // One edge: a single variable resistor spanning the two terminals.
            // `build_mna` stamped it via the `is_pot()` → `stamp_mna` path
            // (r = taper.apply(0.5)·max_r).max(1.0)), so the binding baseline
            // matches that exact conductance for an exact delta update.
            1 => {
                let position = 0.5;
                let r = (pot.taper.apply(position) * pot.max_r).max(1.0);
                pot_bindings.push(StateSpacePotBinding {
                    comp_id: comp.id.clone(),
                    max_r: pot.max_r,
                    taper: pot.taper,
                    position,
                    terminals: MnaPortTerminals::maybe_differential(
                        node_to_mna(e.node_a).map(MnaNodeId::new),
                        node_to_mna(e.node_b).map(MnaNodeId::new),
                    ),
                    conductance: 1.0 / r,
                    complement: false,
                });
            }
            // ── 3-terminal wiper divider (a→w, w→b) ────────────────────────
            // GAP (a): two edges. `build_mna` stamped BOTH segments via
            // `stamp_mna_multi`: r_aw = (taper.apply(0.5)·max_r).max(1.0) and
            // r_wb = ((1−taper.apply(0.5))·max_r).max(1.0). Bind each segment
            // as its own pot leaf with the synthetic `__aw`/`__wb` ids (the
            // same convention WdfStage/SerialDelayedFeedbackStage use). At
            // runtime `set_pot` drives BOTH segments with the same `pos`; the
            // `__wb` binding carries `complement: true`, so it re-derives its
            // (1−taper.apply(pos)) resistance and the two segments always sum to
            // max_r and track the wiper together.
            2 => {
                if !bound_pots.insert(e.comp_idx) {
                    continue;
                }
                // Resolve the wiper node so each segment's terminals can be
                // identified independently (a→w vs w→b).
                let w_node = graph
                    .node_names
                    .get(&format!("{}.w", comp.id))
                    .or_else(|| graph.node_names.get(&format!("{}.wiper", comp.id)))
                    .copied();
                let a_node = graph.node_names.get(&format!("{}.a", comp.id)).copied();
                let b_node = graph.node_names.get(&format!("{}.b", comp.id)).copied();
                let (Some(w_node), Some(a_node), Some(b_node)) = (w_node, a_node, b_node) else {
                    // Cannot identify the three terminals by name — fall back to
                    // leaving the pot unbound (frozen at its stamped baseline)
                    // rather than mis-binding segments.
                    continue;
                };

                // a→w segment: R_aw = pos·max_R (tracks `position` directly).
                let r_aw = (pot.taper.apply(0.5) * pot.max_r).max(1.0);
                pot_bindings.push(StateSpacePotBinding {
                    comp_id: format!("{}__aw", comp.id),
                    max_r: pot.max_r,
                    taper: pot.taper,
                    position: 0.5,
                    terminals: MnaPortTerminals::maybe_differential(
                        node_to_mna(a_node).map(MnaNodeId::new),
                        node_to_mna(w_node).map(MnaNodeId::new),
                    ),
                    conductance: 1.0 / r_aw,
                    complement: false,
                });

                // w→b segment: R_wb = (1−taper.apply(pos))·max_R. Baseline
                // matches the `stamp_mna_multi` complement form
                // (1−taper.apply(0.5)) so the first delta is exact. The
                // `complement: true` flag makes `set_pot` track 1−taper(pos).
                let r_wb = ((1.0 - pot.taper.apply(0.5)) * pot.max_r).max(1.0);
                pot_bindings.push(StateSpacePotBinding {
                    comp_id: format!("{}__wb", comp.id),
                    max_r: pot.max_r,
                    taper: pot.taper,
                    position: 0.5,
                    terminals: MnaPortTerminals::maybe_differential(
                        node_to_mna(w_node).map(MnaNodeId::new),
                        node_to_mna(b_node).map(MnaNodeId::new),
                    ),
                    conductance: 1.0 / r_wb,
                    complement: true,
                });
            }
            _ => continue,
        }
    }

    let input_node_id = built.injection_node_id();
    let output_node_id = built.output_node_id();

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
    stage.bind_ports(input_node_id, output_node_id);
    stage.pot_bindings = pot_bindings;
    if !stage.pot_bindings.is_empty() {
        stage.recompute_mna = Some(built.mna);
    }

    Ok(stage)
}
