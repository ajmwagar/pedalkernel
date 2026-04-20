//! State-space rigid stage builder.
//!
//! Builds an MNA system from the rigid stage's edges, stamps all
//! components (resistors, caps, VCVS), and calls
//! `MnaSystem::build_state_space_matrices()` to get (A, b, c, d).
//! O(N²)/sample at runtime where N = number of states.
//!
//! Covers linear circuits with 3+ reactive elements that can't reduce
//! to a biquad: Klon Centaur stages, BB Preamp, complex active filters.

use super::super::component::{EdgeKind, StampContext};
use super::super::dyn_node::DynNode;
use super::super::graph::{CircuitGraph, NodeId};
use super::super::stage::{StateSpaceData, StateSpaceStage};
use crate::tree::MnaSystem;

/// Build a state-space stage from a linear rigid R-node with 3+ reactive elements.
///
/// Follows the same MNA construction pattern as `build_iir_stage()`:
/// 1. Collect unique circuit nodes from edges
/// 2. Create MNA system, stamp each component
/// 3. Call `build_state_space_matrices()` to get discrete-time matrices
/// 4. Return StateSpaceStage
pub(in crate::compiler) fn build_state_space_stage(
    edge_indices: &[usize],
    pendant_trees: &[(DynNode, NodeId)],
    graph: &CircuitGraph,
    sample_rate: f64,
    supply_voltage: f64,
) -> Result<StateSpaceStage, String> {
    // ── Step 1: Collect unique MNA nodes ─────────────────────────────
    let mut node_set: Vec<NodeId> = Vec::new();
    let mut add_node = |node: NodeId| {
        if node == graph.gnd_node || graph.supply_nodes.contains(&node) {
            return;
        }
        if !node_set.contains(&node) {
            node_set.push(node);
        }
    };

    for &eidx in edge_indices {
        let e = &graph.edges[eidx];
        add_node(e.node_a);
        add_node(e.node_b);
    }
    for (_, attach) in pendant_trees {
        add_node(*attach);
    }

    // Also add nodes from VCVS pos pin
    for rec in &graph.nullor_pins {
        let is_in_stage = edge_indices
            .iter()
            .any(|&eidx| graph.edges[eidx].comp_idx == rec.comp_idx);
        if is_in_stage {
            add_node(rec.pos_node);
            add_node(rec.neg_node);
            add_node(rec.out_node);
        }
    }

    let num_nodes = node_set.len();
    if num_nodes == 0 {
        return Err("StateSpace: no circuit nodes found".to_string());
    }

    let node_to_mna = |node: NodeId| -> Option<usize> {
        if node == graph.gnd_node || graph.supply_nodes.contains(&node) {
            None
        } else {
            node_set.iter().position(|&n| n == node)
        }
    };

    // ── Step 2: Count voltage sources ────────────────────────────────
    let mut num_vsources: usize = 0;
    let mut comp_vsrc_base: Vec<(usize, usize)> = Vec::new();

    for &eidx in edge_indices {
        let comp = &graph.components[graph.edges[eidx].comp_idx];
        let count = comp.kind.mna_vsource_count();
        if count > 0 {
            comp_vsrc_base.push((graph.edges[eidx].comp_idx, num_vsources));
            num_vsources += count;
        }
    }

    // Input voltage source (last)
    let vs_idx = num_vsources;
    num_vsources += 1;

    // ── Step 3: Build MNA and stamp components ──────────────────────
    let mut mna = MnaSystem::new(num_nodes, num_vsources);
    let mut cap_stamps: Vec<(Option<usize>, Option<usize>, f64)> = Vec::new();

    for &eidx in edge_indices {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        let n1 = node_to_mna(e.node_a);
        let n2 = node_to_mna(e.node_b);
        let edge_kind = graph.effective_edge_kind(eidx);

        match edge_kind {
            EdgeKind::Linear => {
                if let Some(r) = comp.kind.resistance() {
                    mna.stamp_resistor(n1, n2, r);
                }
            }
            EdgeKind::Reactive => {
                if let Some(c) = comp.kind.capacitance() {
                    cap_stamps.push((n1, n2, c));
                }
                if let Some(l) = comp.kind.inductance() {
                    cap_stamps.push((n1, n2, -l)); // Negative = inductor convention
                }
            }
            EdgeKind::Vcvs => {
                let vsrc_base = comp_vsrc_base
                    .iter()
                    .find(|&&(ci, _)| ci == e.comp_idx)
                    .map(|&(_, base)| base)
                    .unwrap_or(0);

                let pin_fn = |pin: &str| -> Option<usize> {
                    let key = format!("{}.{}", comp.id, pin);
                    let node = graph.node_names.get(&key)?;
                    node_to_mna(*node)
                };
                let mut ctx = StampContext {
                    pin_to_mna: &pin_fn,
                    vsrc_base,
                    internal_node_base: 0,
                    sample_rate,
                    cap_stamps: None,
                };
                comp.kind.stamp_mna_multi(&comp.id, &mut ctx, &mut mna);
            }
            _ => {} // Skip NL, Vccs, Behavioral
        }
    }

    // Pendant WDF trees: stamp as port resistance at attachment node
    for (tree, attach_node) in pendant_trees {
        let n = node_to_mna(*attach_node);
        let rp = tree.port_resistance();
        if rp > 0.0 {
            mna.stamp_resistor(n, None, rp);
        }
    }

    if cap_stamps.is_empty() {
        return Err("StateSpace: no reactive elements found".to_string());
    }

    // Input voltage source
    let injection_mna = node_to_mna(graph.in_node);
    mna.stamp_voltage_source(injection_mna, None, vs_idx);

    // Output node
    let output_node = graph
        .nullor_pins
        .iter()
        .find(|rec| {
            edge_indices
                .iter()
                .any(|&eidx| graph.edges[eidx].comp_idx == rec.comp_idx)
        })
        .map(|rec| rec.out_node)
        .unwrap_or(graph.out_node);
    let out_mna = node_to_mna(output_node);

    // ── Step 4: Build state-space matrices ──────────────────────────
    let (a_d, b_d, c_out, n_states, d_feedthrough) =
        mna.build_state_space_matrices(&cap_stamps, vs_idx, out_mna, None, sample_rate);

    if n_states == 0 {
        return Err("StateSpace: zero states after matrix construction".to_string());
    }

    let ss = StateSpaceData {
        x: vec![0.0; n_states],
        a_matrix: a_d,
        b_vector: b_d,
        c_vector: c_out,
        n_states,
        cap_stamps,
        vs_idx,
        output_pos: out_mna,
        output_neg: None,
        sample_rate,
        d_feedthrough,
        prev_output: 0.0,
        pot_stamps: Vec::new(),
    };

    Ok(StateSpaceStage::new(ss, supply_voltage))
}
