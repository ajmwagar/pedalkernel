//! IIR rigid stage builder.
//!
//! Builds an MNA system from the rigid stage's edges, stamps all
//! components (resistors, caps, VCVS), and calls `MnaSystem::build_iir()`
//! to get biquad coefficients. O(1)/sample at runtime.
//!
//! Covers both passive bridges AND active linear circuits (808 bridged-T
//! with op-amp VCVS + capacitors). VCVS is a linear constraint.

use super::super::component::{EdgeKind, StampContext};
use super::super::dyn_node::DynNode;
use super::super::graph::{CircuitGraph, NodeId};
use super::super::stage::IirData;
use crate::tree::MnaSystem;

/// Build a biquad IIR from a linear rigid R-node.
///
/// 1. Collects unique circuit nodes from edges
/// 2. Creates MNA system, stamps each component
/// 3. Calls `build_iir()` to get (b, a) biquad coefficients
/// 4. Returns IirData (caller wraps in IirStage)
pub(in crate::compiler) fn build_iir_stage(
    edge_indices: &[usize],
    pendant_trees: &[(DynNode, NodeId)],
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<IirData, String> {
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

    // Also add nodes from VCVS pos pin (might be at ground/bias, but add anyway)
    for rec in &graph.nullor_pins {
        let is_in_stage = edge_indices.iter().any(|&eidx| {
            graph.edges[eidx].comp_idx == rec.comp_idx
        });
        if is_in_stage {
            add_node(rec.pos_node);
            add_node(rec.neg_node);
            add_node(rec.out_node);
        }
    }

    let num_nodes = node_set.len();
    if num_nodes == 0 {
        return Err("IIR: no circuit nodes found".to_string());
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
    let mut comp_vsrc_base: Vec<(usize, usize)> = Vec::new(); // (comp_idx, vsrc_base)

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
                // Inductors: stamp as conductance (1/sL → bilinear)
                if let Some(l) = comp.kind.inductance() {
                    cap_stamps.push((n1, n2, -l)); // Negative = inductor convention
                }
            }
            EdgeKind::Vcvs => {
                // Stamp via Component::stamp_mna_multi
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
                    vsrc_base: vsrc_base,
                    internal_node_base: 0,
                    sample_rate,
                    cap_stamps: Some(&mut cap_stamps),
                };
                comp.kind.stamp_mna_multi(&comp.id, &mut ctx, &mut mna);
            }
            _ => {} // Skip NL, Vccs, Behavioral (shouldn't be in IIR stage)
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

    // Input voltage source — find injection node (first pendant or first edge node)
    let injection_node = pendant_trees
        .first()
        .map(|(_, n)| *n)
        .unwrap_or_else(|| graph.edges[edge_indices[0]].node_a);
    let injection_mna = node_to_mna(injection_node);
    mna.stamp_voltage_source(injection_mna, None, vs_idx);

    // Output node — typically the VCVS out node or last edge node
    let output_node = graph.nullor_pins.iter()
        .find(|rec| edge_indices.iter().any(|&eidx| graph.edges[eidx].comp_idx == rec.comp_idx))
        .map(|rec| rec.out_node)
        .unwrap_or(graph.out_node);
    let out_mna = node_to_mna(output_node);

    // ── Step 4: Build IIR biquad ────────────────────────────────────
    let (b_coeffs, a_coeffs) = mna
        .build_iir(&cap_stamps, vs_idx, out_mna, None, sample_rate, None)
        .ok_or("IIR: build_iir failed — circuit may not reduce to biquad")?;

    Ok(IirData::new(b_coeffs, a_coeffs, sample_rate))
}
