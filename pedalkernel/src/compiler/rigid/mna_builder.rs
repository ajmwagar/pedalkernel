//! Shared MNA system construction for rigid stage builders.
//! Used by iir.rs and state_space.rs to avoid duplicating node collection,
//! component stamping, and voltage source injection logic.

use super::super::component::{EdgeKind, StampContext};
use super::super::dyn_node::DynNode;
use super::super::graph::{CircuitGraph, NodeId};
use crate::tree::MnaSystem;
use pedalkernel_rt::boundary_math::{
    MnaNodeId, MnaOnePort, MnaPortTerminals, OnePort, OnePortKind,
};
use std::collections::{HashMap, VecDeque};

/// Result of building an MNA system from a set of edges.
pub(super) struct BuiltMna {
    pub mna: MnaSystem,
    pub reactive_one_ports: Vec<MnaOnePort>,
    pub node_set: Vec<NodeId>,
    pub vs_idx: usize,
    pub injection_mna: Option<usize>,
    pub output_mna: Option<usize>,
}

impl BuiltMna {
    pub(super) fn injection_node_id(&self) -> Option<usize> {
        self.injection_mna
            .and_then(|idx| self.node_set.get(idx).copied())
    }

    pub(super) fn output_node_id(&self) -> Option<usize> {
        self.output_mna
            .and_then(|idx| self.node_set.get(idx).copied())
    }
}

/// Build an MNA system from a set of edges.
///
/// Handles:
/// 1. Node collection (skipping GND/supply)
/// 2. Voltage source counting (VCVS + input VS)
/// 3. Component stamping (resistors, caps/inductors, VCVS)
/// 4. Pendant tree port resistance stamping
/// 5. Input voltage source injection at `graph.in_node`
/// 6. Output node resolution (`graph.out_node` when present, otherwise VCVS out)
pub(super) fn build_mna(
    edge_indices: &[usize],
    pendant_trees: &[(DynNode, NodeId)],
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<BuiltMna, String> {
    // ── Step 1: Collect unique MNA nodes ─────────────────────────────
    let mut node_set: Vec<NodeId> = Vec::new();
    let mut add_node = |node: NodeId| {
        if node == graph.gnd_node
            || graph.supply_nodes.contains(&node)
            || graph.ac_ground_nodes.contains(&node)
        {
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
        if graph.effective_edge_kind(eidx) == EdgeKind::Vccs {
            let comp = &graph.components[e.comp_idx];
            let out_key = format!("{}.out", comp.id);
            if let Some(&out_node) = graph.node_names.get(&out_key) {
                add_node(out_node);
            }
        }
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
        if node == graph.gnd_node
            || graph.supply_nodes.contains(&node)
            || graph.ac_ground_nodes.contains(&node)
        {
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
    let mut reactive_one_ports: Vec<MnaOnePort> = Vec::new();

    // Track multi-port components to avoid double-stamping.
    // Components with >1 port use stamp_mna_multi() once.
    let mut stamped_multi: std::collections::HashSet<usize> = std::collections::HashSet::new();

    for &eidx in edge_indices {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        let n1 = node_to_mna(e.node_a);
        let n2 = node_to_mna(e.node_b);
        let edge_kind = graph.effective_edge_kind(eidx);

        // Multi-port components (pots with 2 edges): stamp once via stamp_mna_multi.
        // ONLY use multi-port path when the component actually has multiple edges
        // in this stage. Pots used with just a/b terminals (no wiper) have 1 edge
        // and should use the normal resistance() path — stamp_mna_multi would
        // incorrectly stamp two resistors to ground via the missing wiper node.
        let comp_edge_count = edge_indices
            .iter()
            .filter(|&&ei| graph.edges[ei].comp_idx == e.comp_idx)
            .count();
        if comp.kind.ports().len() > 1 && edge_kind == EdgeKind::Linear && comp_edge_count > 1 {
            if stamped_multi.insert(e.comp_idx) {
                let pin_fn = |pin: &str| -> Option<usize> {
                    let key = format!("{}.{}", comp.id, pin);
                    let node = graph.node_names.get(&key)?;
                    node_to_mna(*node)
                };
                let mut ctx = StampContext {
                    pin_to_mna: &pin_fn,
                    vsrc_base: 0,
                    internal_node_base: 0,
                    sample_rate,
                    reactive_one_ports: None,
                };
                comp.kind.stamp_mna_multi(&comp.id, &mut ctx, &mut mna);
            }
            continue;
        }

        match edge_kind {
            EdgeKind::Linear => {
                if comp.kind.is_pot() {
                    comp.kind.stamp_mna(&comp.id, n1, n2, &mut mna, sample_rate);
                } else if let Some(r) = comp.kind.resistance() {
                    mna.stamp_resistor(n1, n2, r);
                }
            }
            EdgeKind::Reactive => {
                if let Some(c) = comp.kind.capacitance() {
                    if n1.is_some() || n2.is_some() {
                        reactive_one_ports.push(OnePort::new(
                            MnaPortTerminals::maybe_differential(
                                n1.map(MnaNodeId::new),
                                n2.map(MnaNodeId::new),
                            ),
                            OnePortKind::Capacitor(c),
                        ));
                    }
                }
                if let Some(l) = comp.kind.inductance() {
                    if n1.is_some() || n2.is_some() {
                        reactive_one_ports.push(OnePort::new(
                            MnaPortTerminals::maybe_differential(
                                n1.map(MnaNodeId::new),
                                n2.map(MnaNodeId::new),
                            ),
                            OnePortKind::Inductor(l),
                        ));
                    }
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
                    reactive_one_ports: None,
                };
                comp.kind.stamp_mna_multi(&comp.id, &mut ctx, &mut mna);
            }
            EdgeKind::Vccs => {
                let pin_fn = |pin: &str| -> Option<usize> {
                    let key = format!("{}.{}", comp.id, pin);
                    let node = graph.node_names.get(&key)?;
                    node_to_mna(*node)
                };
                let gm = linear_ota_transconductance(comp.kind.op_amp_type());
                mna.stamp_vccs(pin_fn("out"), None, pin_fn("pos"), pin_fn("neg"), gm);
            }
            _ => {} // Skip NL, Behavioral
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

    // Mid-chain terminal derivation (GAP b): a passive group sitting between
    // two other stages touches neither the global `in` nor `out` node, and has
    // no nullor of its own. The injection/output resolvers below then fall to
    // None → a zero c-vector → silence. Derive the group's true boundary ports
    // the same way `build_passive_rtype_stage` does (via `compute_group_terminals`
    // + signal-flow orientation) so a mid-chain StateSpace group binds real
    // injection/output ports. Used only as a last-resort fallback, so circuits
    // whose group contains the global in/out (the extraction suite) are
    // unaffected.
    let is_signal_ref = |n: NodeId| -> bool {
        n == graph.gnd_node || graph.supply_nodes.contains(&n) || graph.ac_ground_nodes.contains(&n)
    };
    let mid_chain_terminals = || -> Option<(Option<usize>, Option<usize>)> {
        let terminals = crate::compiler::spqr_build::compute_group_terminals(
            edge_indices,
            graph,
            &[graph.in_node, graph.out_node],
        );
        // Pick output: prefer the global out, else the first non-ref boundary
        // that isn't the chosen input; pick input symmetrically.
        let mut output_node = terminals
            .iter()
            .copied()
            .find(|&t| t == graph.out_node)
            .or_else(|| {
                terminals
                    .iter()
                    .copied()
                    .find(|&t| !is_signal_ref(t) && t != graph.in_node)
            })?;
        let mut input_node = terminals
            .iter()
            .copied()
            .find(|&t| t == graph.in_node)
            .or_else(|| {
                terminals
                    .iter()
                    .copied()
                    .find(|&t| !is_signal_ref(t) && t != output_node)
            })?;
        // Orient interior groups by rail-blocked signal-flow distance from
        // `in`: the upstream terminal drives, the downstream one extracts.
        // Swap only on a proven inversion (mirrors build_passive_rtype_stage).
        if output_node != graph.out_node && input_node != graph.in_node {
            let d_in = crate::compiler::signal_flow::bfs_distances_from_in_node(graph);
            if let (Some(&d_input), Some(&d_output)) =
                (d_in.get(&input_node), d_in.get(&output_node))
            {
                if d_output < d_input {
                    core::mem::swap(&mut input_node, &mut output_node);
                }
            }
        }
        Some((node_to_mna(input_node), node_to_mna(output_node)))
    };

    // Nullor (op-amp) MNA nodes present in this stage. The op-amp is stamped
    // into the B/C/D voltage-source blocks (`stamp_vcvs`), NOT the G matrix, so
    // the signal-path BFS below never crosses the device — reachability to a
    // nullor node means "wired to the op-amp's pins through passives".
    let stage_nullor_mnas: Vec<usize> = graph
        .nullor_pins
        .iter()
        .filter(|rec| {
            edge_indices
                .iter()
                .any(|&eidx| graph.edges[eidx].comp_idx == rec.comp_idx)
        })
        .flat_map(|rec| [rec.pos_node, rec.neg_node, rec.out_node])
        .filter_map(node_to_mna)
        .collect();

    // Reactive coupling edges (capacitors / inductors). These are stamped as
    // `reactive_one_ports`, NOT into `g_matrix`, but a coupling cap IS a valid AC
    // signal path (Sallen-Key HPF: `in → C1 → C2 → op-amp.pos`), so the
    // reachability BFS below must cross them too. Collect the MNA node pairs.
    let reactive_edges: Vec<(usize, usize)> = reactive_one_ports
        .iter()
        .filter_map(|op| match (op.terminals.pos, op.terminals.neg) {
            (Some(p), Some(q)) => Some((p.get(), q.get())),
            _ => None,
        })
        .filter(|&(p, q)| p < num_nodes && q < num_nodes && p != q)
        .collect();

    // Signal-path reachability from the nullor nodes: BFS over nonzero
    // off-diagonal conductances in `mna.g_matrix` PLUS reactive coupling edges.
    // Used to reject an injection candidate that sits on an ISLAND disconnected
    // from the op-amp (uberdrive: the global `in` node R1.a/R2.a dead-ends at R2,
    // never touching IC2's nullor — resistively OR reactively). The op-amp itself
    // is stamped into B/C/D (`stamp_vcvs`), never the G matrix, so the BFS never
    // crosses the device. A stage with NO nullor is always "reachable" (the guard
    // is a no-op → passive/single-stage groups keep their exact prior injection).
    let g_reachable_from_nullor = |candidate: usize| -> bool {
        if stage_nullor_mnas.is_empty() {
            return true;
        }
        let n = num_nodes;
        let mut visited = vec![false; n];
        let mut queue: VecDeque<usize> = VecDeque::new();
        for &start in &stage_nullor_mnas {
            if start < n && !visited[start] {
                visited[start] = true;
                queue.push_back(start);
            }
        }
        while let Some(i) = queue.pop_front() {
            if i == candidate {
                return true;
            }
            for j in 0..n {
                if j != i && !visited[j] && mna.g_matrix[i * n + j].abs() > 0.0 {
                    visited[j] = true;
                    queue.push_back(j);
                }
            }
            for &(p, q) in &reactive_edges {
                let other = if p == i {
                    Some(q)
                } else if q == i {
                    Some(p)
                } else {
                    None
                };
                if let Some(o) = other {
                    if !visited[o] {
                        visited[o] = true;
                        queue.push_back(o);
                    }
                }
            }
        }
        candidate < n && visited[candidate]
    };

    // Mid-chain active stage rescue: pick the group's boundary terminal that IS
    // conductively wired to the op-amp (G-reachable from a nullor node), that is
    // not the group's output terminal, and that is most upstream by signal-flow
    // distance. For uberdrive's IC2 tone stage the terminals are
    // [in-island, 14, IC1.out→R8, …, follower-out]; only the true upstream
    // boundary (IC1.out→R8) reaches the nullor, so it wins — while the global-`in`
    // island (dead-ended at R2) and the downstream follower node are rejected.
    let mid_chain_reachable_input = || -> Option<usize> {
        if stage_nullor_mnas.is_empty() {
            return None;
        }
        let out_terminal = mid_chain_terminals().and_then(|(_, out)| out);
        let d_in = crate::compiler::signal_flow::bfs_distances_from_in_node(graph);
        let terminals = crate::compiler::spqr_build::compute_group_terminals(
            edge_indices,
            graph,
            &[graph.in_node, graph.out_node],
        );
        terminals
            .iter()
            .copied()
            .filter(|&t| !is_signal_ref(t))
            .filter_map(|t| node_to_mna(t).map(|idx| (t, idx)))
            .filter(|&(_, idx)| Some(idx) != out_terminal)
            .filter(|&(_, idx)| g_reachable_from_nullor(idx))
            // Most upstream (smallest distance from `in`) drives; ties broken by
            // node id for determinism.
            .min_by_key(|&(t, _)| (d_in.get(&t).copied().unwrap_or(usize::MAX), t))
            .map(|(_, idx)| idx)
    };

    // Input voltage source at injection node.
    // Prefer graph.in_node if it's in this stage's MNA AND conductively reaches
    // this stage's op-amp (a stage without a nullor passes this unconditionally).
    // Otherwise fall back to the nearest reachable stage input, the reachable
    // upstream boundary terminal (rescues a mid-chain active stage whose only
    // claim to the global `in` is a dead-end island — uberdrive's IC2 tone
    // stage), the VCVS neg node, then the raw mid-chain input as a last resort.
    let injection_mna = node_to_mna(graph.in_node)
        .filter(|&idx| g_reachable_from_nullor(idx))
        .or_else(|| {
            nearest_stage_input_node(&node_set, graph)
                .and_then(node_to_mna)
                .filter(|&idx| g_reachable_from_nullor(idx))
        })
        .or_else(mid_chain_reachable_input)
        .or_else(|| {
            graph
                .nullor_pins
                .iter()
                .find(|rec| {
                    edge_indices
                        .iter()
                        .any(|&eidx| graph.edges[eidx].comp_idx == rec.comp_idx)
                })
                .and_then(|rec| node_to_mna(rec.neg_node))
        })
        .or_else(|| mid_chain_terminals().and_then(|(inj, _)| inj));
    mna.stamp_voltage_source(injection_mna, None, vs_idx);

    // Output node: if this rigid group contains the circuit's named `out`
    // terminal, sample that node. Otherwise fall back to the active device's
    // output node, then to the mid-chain downstream terminal.
    //
    // This matters for module/pedal output pads: `U1.out -> R -> out -> R -> gnd`
    // must extract at `out`, not at `U1.out`, or the pad only loads the circuit
    // and never attenuates the returned signal.
    let output_mna = node_to_mna(graph.out_node)
        .or_else(|| {
            graph
                .nullor_pins
                .iter()
                .find(|rec| {
                    edge_indices
                        .iter()
                        .any(|&eidx| graph.edges[eidx].comp_idx == rec.comp_idx)
                })
                .and_then(|rec| node_to_mna(rec.out_node))
        })
        .or_else(|| mid_chain_terminals().and_then(|(_, out)| out));

    Ok(BuiltMna {
        mna,
        reactive_one_ports,
        node_set,
        vs_idx,
        injection_mna,
        output_mna,
    })
}

fn linear_ota_transconductance(op_type: Option<crate::dsl::OpAmpType>) -> f64 {
    op_type
        .and_then(|op_type| crate::model_lookup::ota_gm_from_type(&op_type))
        .unwrap_or(0.0)
}

fn nearest_stage_input_node(node_set: &[NodeId], graph: &CircuitGraph) -> Option<NodeId> {
    let mut dist: HashMap<NodeId, usize> = HashMap::new();
    let mut queue = VecDeque::new();
    dist.insert(graph.in_node, 0);
    queue.push_back(graph.in_node);

    while let Some(node) = queue.pop_front() {
        let next_dist = dist[&node] + 1;
        for (eidx, edge) in graph.edges.iter().enumerate() {
            if graph.effective_edge_kind(eidx) == EdgeKind::Vcvs {
                continue;
            }
            let next = if edge.node_a == node {
                edge.node_b
            } else if edge.node_b == node {
                edge.node_a
            } else {
                continue;
            };
            if next == graph.gnd_node
                || graph.supply_nodes.contains(&next)
                || graph.ac_ground_nodes.contains(&next)
            {
                continue;
            }
            if dist.insert(next, next_dist).is_none() {
                queue.push_back(next);
            }
        }
    }

    node_set
        .iter()
        .filter_map(|node| dist.get(node).map(|distance| (*distance, *node)))
        .min_by_key(|(distance, node)| (*distance, *node))
        .map(|(_, node)| node)
}
