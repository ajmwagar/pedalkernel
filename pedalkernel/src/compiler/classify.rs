//! Pass 1: Unified element classification.
//!
//! Single pass over all edges classifies each as passive or nonlinear.
//! Replaces the per-type `find_diodes()`, `find_jfets()`, `find_bjts()`,
//! `find_triodes()`, `find_pentodes()`, `find_mosfets()`, `find_zeners()`,
//! `find_otas()` methods that each independently performed BFS.

use std::collections::{HashMap, HashSet};

use crate::dsl::*;

use super::graph::{CircuitGraph, NodeId};

// ═══════════════════════════════════════════════════════════════════════════
// Nonlinear element kind
// ═══════════════════════════════════════════════════════════════════════════

/// The kind of nonlinear element, with element-specific data.
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
    },
    BjtPnp {
        model_name: String,
        base_node: NodeId,
    },
    Triode {
        model_name: String,
        plate_node: NodeId,
        cathode_node: NodeId,
        parallel_count: usize,
        is_vari_mu: bool,
    },
    Pentode {
        model_name: String,
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
    /// - 1 node for simple elements (diode, JFET, pentode, MOSFET, zener, OTA)
    /// - 2 nodes for 3-terminal elements (BJT: collector+emitter, triode: plate+cathode)
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
/// Single pass over all edges. For nonlinear elements, records:
/// - Edge index, element kind, junction node(s)
/// - Sorts by BFS distance from input (for cascaded stage ordering)
pub(super) fn classify_circuit(
    graph: &CircuitGraph,
    pedal: &PedalDef,
) -> ClassifiedCircuit {
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

    // ── BFS from in_node ───────────────────────────────────────────────
    let dist_from_in = bfs_distances(graph, graph.in_node);
    let dist_from_out = bfs_distances(graph, graph.out_node);

    // ── Collect raw triodes for parallel merging ───────────────────────
    let mut raw_triodes: Vec<(usize, String, NodeId, NodeId, bool)> = Vec::new();
    for (edge_idx, e) in graph.edges.iter().enumerate() {
        let comp = &graph.components[e.comp_idx];
        match &comp.kind {
            ComponentKind::Triode(name) => {
                raw_triodes.push((edge_idx, name.clone(), e.node_a, e.node_b, false));
            }
            ComponentKind::VariMu(name) => {
                raw_triodes.push((edge_idx, name.clone(), e.node_a, e.node_b, true));
            }
            _ => {}
        }
    }

    // Group triodes by (plate, cathode) to detect parallel tubes.
    let mut triode_groups: HashMap<(NodeId, NodeId), Vec<(usize, String, bool)>> = HashMap::new();
    for (edge_idx, name, plate, cathode, is_vm) in &raw_triodes {
        triode_groups
            .entry((*plate, *cathode))
            .or_default()
            .push((*edge_idx, name.clone(), *is_vm));
    }

    // All raw triode edge indices (including parallel duplicates).
    let all_triode_edges: HashSet<usize> = raw_triodes.iter().map(|(idx, _, _, _, _)| *idx).collect();

    // ── Single pass: classify all edges ────────────────────────────────
    let mut elements: Vec<NonlinearElement> = Vec::new();
    let mut all_nonlinear_edge_indices: Vec<usize> = Vec::new();
    let mut has_germanium = false;

    // Track which triode groups we've already added (by representative plate+cathode).
    let mut processed_triode_groups: HashSet<(NodeId, NodeId)> = HashSet::new();

    for (edge_idx, e) in graph.edges.iter().enumerate() {
        let comp = &graph.components[e.comp_idx];

        let a_is_gnd = e.node_a == graph.gnd_node;
        let b_is_gnd = e.node_b == graph.gnd_node;

        // Diodes: feedback diodes (neither node at gnd) use node_a as junction.
        let diode_junction = |node_a: NodeId, node_b: NodeId| -> NodeId {
            if b_is_gnd { node_a } else if a_is_gnd { node_b } else { node_a }
        };
        // JFETs, pentodes, MOSFETs, zeners, OTAs: use node_b unless node_b is gnd.
        let nondiode_junction = |node_a: NodeId, node_b: NodeId| -> NodeId {
            if b_is_gnd { node_a } else { node_b }
        };

        let classified = match &comp.kind {
            // ── Diodes ─────────────────────────────────────────────
            ComponentKind::DiodePair(dt) => {
                if *dt == DiodeType::Germanium {
                    has_germanium = true;
                }
                let junction = diode_junction(e.node_a, e.node_b);
                Some((
                    NonlinearKind::DiodePair(*dt),
                    vec![junction],
                ))
            }
            ComponentKind::Diode(dt) => {
                if *dt == DiodeType::Germanium {
                    has_germanium = true;
                }
                let junction = diode_junction(e.node_a, e.node_b);
                Some((
                    NonlinearKind::SingleDiode(*dt),
                    vec![junction],
                ))
            }

            // ── JFETs ──────────────────────────────────────────────
            ComponentKind::NJfet(name) => {
                let junction = nondiode_junction(e.node_a, e.node_b);
                Some((
                    NonlinearKind::Jfet {
                        model_name: name.clone(),
                        is_n_channel: true,
                    },
                    vec![junction],
                ))
            }
            ComponentKind::PJfet(name) => {
                let junction = nondiode_junction(e.node_a, e.node_b);
                Some((
                    NonlinearKind::Jfet {
                        model_name: name.clone(),
                        is_n_channel: false,
                    },
                    vec![junction],
                ))
            }

            // ── BJTs ───────────────────────────────────────────────
            ComponentKind::Npn(name) => {
                let base_node = graph
                    .node_names
                    .get(&format!("{}.base", comp.id))
                    .copied()
                    .unwrap_or(e.node_a);
                Some((
                    NonlinearKind::BjtNpn {
                        model_name: name.clone(),
                        base_node,
                    },
                    vec![e.node_a, e.node_b], // collector, emitter
                ))
            }
            ComponentKind::Pnp(name) => {
                let base_node = graph
                    .node_names
                    .get(&format!("{}.base", comp.id))
                    .copied()
                    .unwrap_or(e.node_a);
                Some((
                    NonlinearKind::BjtPnp {
                        model_name: name.clone(),
                        base_node,
                    },
                    vec![e.node_a, e.node_b], // collector, emitter
                ))
            }

            // ── Triodes (merged parallel) ──────────────────────────
            ComponentKind::Triode(_) | ComponentKind::VariMu(_) => {
                let key = (e.node_a, e.node_b);
                if processed_triode_groups.contains(&key) {
                    // Already processed this group — just record edge index.
                    all_nonlinear_edge_indices.push(edge_idx);
                    None
                } else if let Some(group) = triode_groups.get(&key) {
                    processed_triode_groups.insert(key);
                    let (_, ref rep_name, rep_is_vari_mu) = group[0];
                    Some((
                        NonlinearKind::Triode {
                            model_name: rep_name.clone(),
                            plate_node: e.node_a,
                            cathode_node: e.node_b,
                            parallel_count: group.len(),
                            is_vari_mu: rep_is_vari_mu,
                        },
                        vec![e.node_a, e.node_b], // plate, cathode
                    ))
                } else {
                    None
                }
            }

            // ── Pentodes ───────────────────────────────────────────
            ComponentKind::Pentode(name) => {
                let junction = nondiode_junction(e.node_a, e.node_b);
                Some((
                    NonlinearKind::Pentode {
                        model_name: name.clone(),
                    },
                    vec![junction],
                ))
            }

            // ── MOSFETs ────────────────────────────────────────────
            ComponentKind::Nmos(mt) => {
                let junction = nondiode_junction(e.node_a, e.node_b);
                Some((
                    NonlinearKind::Mosfet {
                        mosfet_type: *mt,
                        is_n_channel: true,
                    },
                    vec![junction],
                ))
            }
            ComponentKind::Pmos(mt) => {
                let junction = nondiode_junction(e.node_a, e.node_b);
                Some((
                    NonlinearKind::Mosfet {
                        mosfet_type: *mt,
                        is_n_channel: false,
                    },
                    vec![junction],
                ))
            }

            // ── Zeners ─────────────────────────────────────────────
            ComponentKind::Zener(vz) => {
                let junction = nondiode_junction(e.node_a, e.node_b);
                Some((
                    NonlinearKind::Zener { voltage: *vz },
                    vec![junction],
                ))
            }

            // ── OTAs (CA3080 etc.) ─────────────────────────────────
            ComponentKind::OpAmp(ot) if ot.is_ota() => {
                let junction = nondiode_junction(e.node_a, e.node_b);
                Some((NonlinearKind::Ota, vec![junction]))
            }

            _ => None,
        };

        if let Some((kind, junction_nodes)) = classified {
            all_nonlinear_edge_indices.push(edge_idx);

            // Sort key: use base_node distance for BJTs (cascade ordering),
            // junction_node distance for others.
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
        } else if all_triode_edges.contains(&edge_idx)
            && !all_nonlinear_edge_indices.contains(&edge_idx)
        {
            // Parallel triode duplicate edge — already counted above in the
            // `processed_triode_groups` branch, but make sure it's in the index list.
            all_nonlinear_edge_indices.push(edge_idx);
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
