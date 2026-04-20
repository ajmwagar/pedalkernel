//! Shared MNA system construction for rigid stage builders.
//! Used by iir.rs and state_space.rs to avoid duplicating node collection,
//! component stamping, and voltage source injection logic.

use super::super::component::{EdgeKind, StampContext};
use super::super::dyn_node::DynNode;
use super::super::graph::{CircuitGraph, NodeId};
use crate::tree::MnaSystem;

/// Result of building an MNA system from a set of edges.
pub(super) struct BuiltMna {
    pub mna: MnaSystem,
    pub cap_stamps: Vec<(Option<usize>, Option<usize>, f64)>,
    pub node_set: Vec<NodeId>,
    pub vs_idx: usize,
    pub injection_mna: Option<usize>,
    pub output_mna: Option<usize>,
}

/// Build an MNA system from a set of edges.
///
/// Handles:
/// 1. Node collection (skipping GND/supply)
/// 2. Voltage source counting (VCVS + input VS)
/// 3. Component stamping (resistors, caps/inductors, VCVS)
/// 4. Pendant tree port resistance stamping
/// 5. Input voltage source injection at `graph.in_node`
/// 6. Output node resolution (VCVS out or `graph.out_node`)
pub(super) fn build_mna(
    edge_indices: &[usize],
    pendant_trees: &[(DynNode, NodeId)],
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<BuiltMna, String> {
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
        return Err("MNA: no circuit nodes found".to_string());
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

    // Input voltage source at injection node
    let injection_mna = node_to_mna(graph.in_node);
    mna.stamp_voltage_source(injection_mna, None, vs_idx);

    // Output node: VCVS out node (if present) or graph.out_node
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
    let output_mna = node_to_mna(output_node);

    Ok(BuiltMna {
        mna,
        cap_stamps,
        node_set,
        vs_idx,
        injection_mna,
        output_mna,
    })
}
