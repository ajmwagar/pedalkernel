//! Pass 1: Unified element classification.
//!
//! Single pass over all edges classifies each as passive or nonlinear.
//! Per-component classification is delegated to the `Component::classify_nonlinear()`
//! trait method. Cross-component concerns (parallel triode merging, anti-parallel
//! diode synthesis) are handled as post-passes here.

use hashbrown::HashMap;
use std::collections::HashSet;

use crate::dsl::*;

use super::graph::{CircuitGraph, NodeId};

// ═══════════════════════════════════════════════════════════════════════════
// Nonlinear element kind
// ═══════════════════════════════════════════════════════════════════════════

/// The kind of nonlinear element, with element-specific data.
#[derive(Clone)]
pub(super) enum NonlinearKind {
    DiodePair(DiodeType),
    SingleDiode(DiodeType),
    Jfet {
        model_name: String,
        is_n_channel: bool,
    },
    BjtNpn {
        model_name: String,
        base_node: NodeId,
        collector_node: NodeId,
        emitter_node: NodeId,
    },
    BjtPnp {
        model_name: String,
        base_node: NodeId,
        collector_node: NodeId,
        emitter_node: NodeId,
    },
    Triode {
        model_name: String,
        plate_node: NodeId,
        cathode_node: NodeId,
        grid_node: Option<NodeId>,
        parallel_count: usize,
        is_vari_mu: bool,
    },
    Pentode {
        model_name: String,
        plate_node: NodeId,
        cathode_node: NodeId,
        grid_node: Option<NodeId>,
    },
    Mosfet {
        mosfet_type: MosfetType,
        is_n_channel: bool,
    },
    Zener {
        voltage: f64,
    },
    Ota,
}

// ═══════════════════════════════════════════════════════════════════════════
// Classified element
// ═══════════════════════════════════════════════════════════════════════════

/// A classified nonlinear element with its graph position.
pub(super) struct NonlinearElement {
    /// Edge index in the circuit graph.
    pub(super) edge_idx: usize,
    /// What kind of nonlinear element this is.
    pub(super) kind: NonlinearKind,
    /// Junction node(s) where passive elements connect.
    /// - 1 node for simple elements (diode, JFET, MOSFET, zener, OTA)
    /// - 2 nodes for 3-terminal elements (BJT: collector+emitter, triode/pentode: plate+cathode)
    pub(super) junction_nodes: Vec<NodeId>,
    /// BFS distance from input (for ordering).
    pub(super) distance: usize,
}

/// Result of the classification pass.
pub(super) struct ClassifiedCircuit {
    /// Nonlinear elements, sorted by BFS distance from input.
    pub(super) nonlinear_elements: Vec<NonlinearElement>,
    /// All nonlinear edge indices (for exclusion from passive collection).
    pub(super) all_nonlinear_edge_indices: Vec<usize>,
    /// Sidechain edge set (from partitioning).
    pub(super) sidechain_edge_set: HashSet<usize>,
    /// BFS distances from in_node.
    pub(super) dist_from_in: HashMap<NodeId, usize>,
    /// BFS distances from out_node.
    pub(super) dist_from_out: HashMap<NodeId, usize>,
    /// Whether the circuit contains germanium diodes.
    pub(super) has_germanium: bool,
}

// ═══════════════════════════════════════════════════════════════════════════
// Classification
// ═══════════════════════════════════════════════════════════════════════════

/// Perform unified classification of all circuit elements.
///
/// Delegates per-component classification to `Component::classify_nonlinear()`.
/// Then applies cross-component merging passes:
/// 1. Parallel triode merging (same plate+cathode → single element with parallel_count)
/// 2. Anti-parallel diode synthesis (two opposite single diodes → one DiodePair)
pub(super) fn classify_circuit(graph: &CircuitGraph, pedal: &PedalDef) -> ClassifiedCircuit {
    // ── Sidechain partitioning ─────────────────────────────────────────
    let sidechain_edge_set: HashSet<usize> = {
        let mut set = HashSet::new();
        for sc_info in &pedal.sidechains {
            if let (Some(&tap), Some(&cv)) = (
                graph.node_names.get(&sc_info.tap_node),
                graph.node_names.get(&sc_info.cv_node),
            ) {
                if let Some(partition) = graph.partition_sidechain(tap, cv) {
                    set.extend(partition.sidechain_edge_indices);
                }
            }
        }
        set
    };

    // ── BFS from in_node / out_node ─────────────────────────────────────
    let dist_from_in = bfs_distances(graph, graph.in_node);
    let dist_from_out = bfs_distances(graph, graph.out_node);

    // ── Classify all edges via Component trait ──────────────────────────
    let mut elements: Vec<NonlinearElement> = Vec::new();
    let mut all_nonlinear_edge_indices: Vec<usize> = Vec::new();
    let mut has_germanium = false;

    for (edge_idx, e) in graph.edges.iter().enumerate() {
        let comp = &graph.components[e.comp_idx];
        let classified = comp.kind.classify_nonlinear(
            &comp.id,
            e.node_a,
            e.node_b,
            graph.gnd_node,
            &graph.node_names,
        );

        if let Some((kind, junction_nodes)) = classified {
            all_nonlinear_edge_indices.push(edge_idx);

            // Track germanium diodes
            match &kind {
                NonlinearKind::SingleDiode(dt) | NonlinearKind::DiodePair(dt) => {
                    if *dt == DiodeType::Germanium {
                        has_germanium = true;
                    }
                }
                _ => {}
            }

            // Sort key: base_node for BJTs, cathode for triodes, first junction otherwise
            let sort_node = match &kind {
                NonlinearKind::BjtNpn { base_node, .. }
                | NonlinearKind::BjtPnp { base_node, .. } => *base_node,
                NonlinearKind::Triode { cathode_node, .. } => *cathode_node,
                _ => junction_nodes[0],
            };
            let distance = dist_from_in.get(&sort_node).copied().unwrap_or(usize::MAX);

            elements.push(NonlinearElement {
                edge_idx,
                kind,
                junction_nodes,
                distance,
            });
        }
    }

    // ── Post-pass 1: Merge parallel triodes ─────────────────────────────
    // Group triodes by (plate, cathode). Keep one representative with
    // parallel_count = group size, remove duplicates.
    {
        let mut triode_groups: HashMap<(NodeId, NodeId), Vec<usize>> = HashMap::new();
        for (i, elem) in elements.iter().enumerate() {
            if let NonlinearKind::Triode {
                plate_node,
                cathode_node,
                ..
            } = &elem.kind
            {
                triode_groups
                    .entry((*plate_node, *cathode_node))
                    .or_default()
                    .push(i);
            }
        }
        let mut to_remove: HashSet<usize> = HashSet::new();
        for group in triode_groups.values() {
            if group.len() > 1 {
                let rep = group[0];
                if let NonlinearKind::Triode {
                    ref mut parallel_count,
                    ..
                } = elements[rep].kind
                {
                    *parallel_count = group.len();
                }
                for &idx in &group[1..] {
                    to_remove.insert(idx);
                }
            }
        }
        let mut remove_vec: Vec<usize> = to_remove.into_iter().collect();
        remove_vec.sort_unstable_by(|a, b| b.cmp(a));
        for idx in remove_vec {
            elements.remove(idx);
        }
    }

    // ── Post-pass 2: Anti-parallel diode pair synthesis ─────────────────
    // Two SingleDiode elements sharing the same two edge nodes (opposite
    // orientation) → merge into one DiodePair (e.g. Klon's MA856 pair).
    {
        let mut merged_indices: HashSet<usize> = HashSet::new();
        let mut i = 0;
        while i < elements.len() {
            if merged_indices.contains(&i) {
                i += 1;
                continue;
            }
            let dt_i = match &elements[i].kind {
                NonlinearKind::SingleDiode(dt) => *dt,
                _ => {
                    i += 1;
                    continue;
                }
            };
            let ei = &graph.edges[elements[i].edge_idx];
            let nodes_i = (ei.node_a.min(ei.node_b), ei.node_a.max(ei.node_b));

            for j in (i + 1)..elements.len() {
                if merged_indices.contains(&j) {
                    continue;
                }
                let dt_j = match &elements[j].kind {
                    NonlinearKind::SingleDiode(dt) => *dt,
                    _ => continue,
                };
                if dt_i != dt_j {
                    continue;
                }
                let ej = &graph.edges[elements[j].edge_idx];
                let nodes_j = (ej.node_a.min(ej.node_b), ej.node_a.max(ej.node_b));
                if nodes_i == nodes_j {
                    elements[i].kind = NonlinearKind::DiodePair(dt_i);
                    merged_indices.insert(j);
                    break;
                }
            }
            i += 1;
        }
        let mut to_remove: Vec<usize> = merged_indices.into_iter().collect();
        to_remove.sort_unstable_by(|a, b| b.cmp(a));
        for idx in to_remove {
            elements.remove(idx);
        }
    }

    // Sort by distance (with edge_idx tiebreaker for determinism).
    elements.sort_by_key(|e| (e.distance, e.edge_idx));

    ClassifiedCircuit {
        nonlinear_elements: elements,
        all_nonlinear_edge_indices,
        sidechain_edge_set,
        dist_from_in,
        dist_from_out,
        has_germanium,
    }
}

/// BFS from a single node, returning distances to all reachable nodes.
fn bfs_distances(graph: &CircuitGraph, start: NodeId) -> HashMap<NodeId, usize> {
    let mut adj: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
    for e in &graph.edges {
        adj.entry(e.node_a).or_default().push(e.node_b);
        adj.entry(e.node_b).or_default().push(e.node_a);
    }
    let mut dist: HashMap<NodeId, usize> = HashMap::new();
    let mut queue = std::collections::VecDeque::new();
    dist.insert(start, 0);
    queue.push_back(start);
    while let Some(n) = queue.pop_front() {
        let d = dist[&n];
        if let Some(neighbors) = adj.get(&n) {
            for &nb in neighbors {
                dist.entry(nb).or_insert_with(|| {
                    queue.push_back(nb);
                    d + 1
                });
            }
        }
    }
    dist
}
