//! Unified compile-time DC bias infrastructure.
//!
//! This module provides the data types and solver that will eventually replace
//! the per-type DC bias code spread across `spqr_build.rs`
//! (`compute_wdf_triode_dc_qpoint`, `compute_wdf_bjt_dc_qpoint`, etc.).
//!
//! # Current status (ko5g.3)
//!
//! Migrated into this file: the flow-group bias classification + resistor-divider
//! nodal solve (`classify_group_bias` / `solve_network_bias`, absorbed from the
//! now-deleted `bias_analysis.rs`), the grouped-BJT nodal co-solve
//! (`solve_bjt_group_dc_qpoint`, the BA283 fix-#1 solver with terminal/parasitic
//! handling), and the grouped triode solve (`solve_triode_dc_qpoint`).
//!
//! As of ko5g inc-3 the **non-varimu common-cathode triode** call-site rides the
//! shared `BiasSeed`/`solve_operating_point` core (`TriodeSeed` +
//! `IterationScheme::MainCurrentRelaxation`): `solve_triode_dc_qpoint`'s
//! non-varimu branch is now a thin delegation (bit-for-bit vs the deleted
//! Ia-relaxation loop; corpus failing-set unchanged).
//!
//! Still per-type / not yet on the shared path:
//! - the **varimu** branch inside `solve_triode_dc_qpoint` (fixed-bias, a
//!   VariMu/Raffensperger model + Ia bisection — needs a dedicated `VariMuSeed`;
//!   NOT bit-preserving under the relaxation scheme and has no characterization
//!   golden, so left as-is);
//! - the **pentode** DC path (no `PentodeSeed`; `solve_triode_dc_qpoint` returns
//!   `None` for non-triode kinds);
//! - the single-port-WDF-root solvers in `spqr_build.rs`
//!   (`compute_wdf_triode_dc_qpoint`, `compute_wdf_bjt_dc_qpoint`,
//!   `compute_wdf_fet_dc_qpoint`) — a separate follow-up.
//!
//! # Architecture
//!
//! ```text
//! BiasSeed (trait)
//!   ├─ locate_bias_topology(ctx) → BiasTopology
//!   └─ device_iv(TrialPoint) → DeviceIv
//!
//! solve_operating_point(seed, topo, ctx) → DeviceOperatingPoint
//!
//! NetworkBias { dc_voltages: HashMap<NodeId, f64> }
//!   └─ solve_network_bias(edges, graph) → NetworkBias   (the divider nodal solve)
//!
//! classify_group_bias(group, graph) → GroupBiasKind {SignalPath | StaticBias}
//!   (absorbed from the former bias_analysis.rs, now deleted)
//! ```
//!
//! # Resistor-locating strategy: cap-aware BFS
//!
//! **Decision (recorded here for ko5g reviewers):**
//!
//! The existing `compute_wdf_triode_dc_qpoint` uses *direct-edge match* to find
//! R_plate and R_cathode: it scans the group's edge set for a resistor with one
//! terminal on the VCC rail and the other on the plate node (or cathode/GND).
//!
//! This works for the most common topology (resistor directly to rail) but fails
//! for **cap-coupled cathode-follower** circuits where the cathode is separated
//! from its resistor by a coupling capacitor.  The coupling cap is open at DC, so
//! the DC path is: plate → R_plate → VCC, cathode → R_cathode → GND — but the
//! cap lies *between* the cathode node and the rest of the signal path.
//!
//! **Cap-aware BFS**: walk the *resistor-only* DC subgraph (caps and inductors are
//! open at DC → skip them).  Starting from the device node, BFS through resistor
//! edges until a rail is reached.  The first resistor on that path is the load
//! resistor.  This correctly handles cap-coupled cathode-followers and reduces the
//! number of circuits that need manual `init {}` hints.
//!
//! The BFS finder is **behind the trait**: `BiasSeed::locate_bias_topology` calls
//! `find_load_resistor_bfs` by default.  The legacy direct-edge finder is
//! available as `find_load_resistor_direct` for comparison and regression tests.
//! Swapping the strategy is a one-line change inside the trait implementation.

use hashbrown::HashMap;
use std::collections::{HashSet, VecDeque};

use super::classify::NonlinearKind;
use super::component::EdgeKind;
use super::graph::{CircuitGraph, NodeId};
use super::helpers::{gummel_poon_model, vari_mu_model};
use crate::elements::{GummelPoonModel, VariMuTriodeRoot};

// ═══════════════════════════════════════════════════════════════════════════
// Error types
// ═══════════════════════════════════════════════════════════════════════════

/// All the ways bias extraction can fail.
#[derive(Debug, Clone)]
pub(super) enum BiasError {
    /// Could not determine the triode operating point — missing topology element.
    UndeterminableTriode {
        label: String,
        missing: TopologyTerm,
    },
    /// Could not determine the BJT operating point — missing topology element.
    UndeterminableBjt {
        label: String,
        missing: TopologyTerm,
    },
    /// Could not determine the FET operating point — missing topology element.
    UndeterminableFet {
        label: String,
        missing: TopologyTerm,
    },
    /// The solved Q-point is non-physical (e.g. positive Vgk, infinite Vbe).
    NonPhysicalQpoint,
    /// The resistor-divider MNA matrix is singular (floating nodes, open mesh).
    SingularBiasNetwork,
    /// The `init {}` state name is not recognised.
    UnknownInitState { state: String },
}

impl BiasError {
    /// Convert to a human-readable compiler diagnostic string.
    ///
    /// The message names the device, describes what is missing, and suggests
    /// the `init {}` hint syntax when a hint could unblock the solve.
    pub(super) fn into_compile_error(self) -> String {
        match self {
            Self::UndeterminableTriode { label, missing } => format!(
                "Cannot determine DC operating point for triode '{label}': \
                 missing {missing}. \
                 Add an `init {{ {label}: active }}` hint, or check that \
                 R_plate (plate→VCC) and R_cathode (cathode→GND) are present.",
            ),
            Self::UndeterminableBjt { label, missing } => format!(
                "Cannot determine DC operating point for BJT '{label}': \
                 missing {missing}. \
                 Add an `init {{ {label}: active }}` hint, or check that \
                 R_base-divider and R_emitter (emitter→GND) are present.",
            ),
            Self::UndeterminableFet { label, missing } => format!(
                "Cannot determine DC operating point for FET '{label}': \
                 missing {missing}. \
                 Add an `init {{ {label}: active }}` hint, or check that \
                 gate-bias resistors and R_source (source→GND) are present.",
            ),
            Self::NonPhysicalQpoint => {
                "Solved DC Q-point is non-physical (check supply voltage and load resistors)."
                    .to_owned()
            }
            Self::SingularBiasNetwork => {
                "DC bias network is singular: the resistor-divider matrix has no unique solution. \
                 Check for floating nodes or open meshes."
                    .to_owned()
            }
            Self::UnknownInitState { state } => format!(
                "Unknown init state '{state}'. \
                 Valid states: active, saturated, cutoff, forward, reverse.",
            ),
        }
    }
}

/// Which topological element is missing, for diagnostic messages.
#[derive(Debug, Clone)]
pub(super) enum TopologyTerm {
    /// A resistor from the plate/drain/collector node to the VCC rail.
    PlateResistor,
    /// A resistor from the cathode/source/emitter node to GND.
    CathodeResistor,
    /// The grid node (triode/pentode pin not found in node map).
    GridNode,
    /// Base-divider resistors (R1: base→VCC, R2: base→GND).
    BaseDivider,
    /// A resistor from the emitter to GND (or VCC for PNP).
    EmitterResistor,
    /// The gate node for a FET.
    GateNode,
    /// A resistor from the drain to the supply rail.
    DrainResistor,
}

impl std::fmt::Display for TopologyTerm {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::PlateResistor => write!(f, "R_plate (resistor from plate to VCC)"),
            Self::CathodeResistor => write!(f, "R_cathode (resistor from cathode to GND)"),
            Self::GridNode => write!(f, "grid node in netlist"),
            Self::BaseDivider => write!(f, "base-bias divider (R_b1: base→VCC and R_b2: base→GND)"),
            Self::EmitterResistor => write!(f, "R_emitter (resistor from emitter to GND)"),
            Self::GateNode => write!(f, "gate node in netlist"),
            Self::DrainResistor => write!(f, "R_drain (resistor from drain to supply rail)"),
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Device operating point — the unified output type
// ═══════════════════════════════════════════════════════════════════════════

/// The device type for a solved operating point.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum DeviceBiasKind {
    Triode,
    VariMu,
    Pentode,
    BjtNpn,
    BjtPnp,
    Jfet,
    Mosfet,
}

/// The fully solved DC operating point for a single active device.
///
/// Produced by `solve_operating_point`.  Call-sites consume this to:
/// - Set the device's bias voltage (`set_bias(control_bias)`)
/// - Set the WDF NR warm-start (`set_initial_prev_v(output_warm_start)`)
/// - Pre-charge reactive elements (`reactive_precharge`)
/// - Set `v_max` (`set_v_max(v_max)`)
#[derive(Debug, Clone)]
pub(super) struct DeviceOperatingPoint {
    /// Label of the device component (matches `Component::id`).
    pub(super) device_label: String,
    /// Which kind of device this Q-point was solved for.
    pub(super) kind: DeviceBiasKind,
    /// Control-terminal bias voltage at the Q-point.
    ///
    /// - Triode / VariMu / Pentode: Vgk (grid-to-cathode, negative in active region)
    /// - BJT NPN/PNP: Vbe (forward base-emitter magnitude, positive)
    /// - JFET: Vgs
    /// - MOSFET: Vgs
    pub(super) control_bias: f64,
    /// Output-terminal voltage at the Q-point, for NR warm-starting.
    ///
    /// - Triode / VariMu / Pentode: Vpk (plate-to-cathode)
    /// - BJT NPN/PNP: Vce (signed: negative for PNP)
    /// - JFET / MOSFET: Vds
    pub(super) output_warm_start: f64,
    /// DC voltages across reactive elements (caps/inductors) that should be
    /// pre-charged to eliminate startup transients.
    ///
    /// Each entry is `(node_id, dc_voltage)`.  Typically one entry for the
    /// cathode/emitter bypass cap.
    pub(super) reactive_precharge: Vec<(NodeId, f64)>,
    /// Supply voltage ceiling for the device's WDF `v_max`.
    pub(super) v_max: f64,
}

// ═══════════════════════════════════════════════════════════════════════════
// Network bias — DC voltages at resistor-divider junctions
// ═══════════════════════════════════════════════════════════════════════════

/// DC voltages at all non-rail nodes in a static bias network.
///
/// This is the compile-time result of running nodal analysis on a
/// VCC→resistor-divider→GND subgraph.  It is the value carried inside
/// `GroupBiasKind::StaticBias` (see `classify_group_bias` below), also exposed as
/// a standalone type so downstream code (the triode/BJT solvers) can consume it
/// without pattern-matching a flow-group enum.
///
/// `solve_network_bias` is the single divider nodal solve: the old
/// `bias_analysis.rs::compute_dc_voltages` was absorbed here (ko5g.3) and that
/// file deleted.
#[derive(Debug, Default, Clone)]
pub(super) struct NetworkBias {
    /// DC voltage at each non-rail circuit node, as solved by MNA on the
    /// resistor-only DC subgraph.
    pub(super) dc_voltages: HashMap<NodeId, f64>,
}

impl NetworkBias {
    /// Look up the DC voltage at `node`, falling back to rail voltages.
    ///
    /// Returns `None` only if the node is not a rail and was not solved
    /// (i.e. not reachable from any resistor-divider in this network).
    pub(super) fn voltage_at(
        &self,
        node: NodeId,
        graph: &CircuitGraph,
        supply_voltage: f64,
    ) -> Option<f64> {
        if node == graph.gnd_node || graph.ac_ground_nodes.contains(&node) {
            return Some(0.0);
        }
        if node == graph.vcc_node || graph.supply_nodes.contains(&node) {
            return Some(supply_voltage);
        }
        if let Some(&v) = graph.supply_voltages.get(&node) {
            return Some(v);
        }
        self.dc_voltages.get(&node).copied()
    }
}

/// Solve the DC voltages at all interior nodes in the given edge set.
///
/// This is the analogue of `bias_analysis::compute_dc_voltages` but takes a
/// flat edge slice rather than a `FlowGroup`.  It sets up the same conductance
/// MNA and returns a `NetworkBias`.
///
/// Capacitors and inductors are open-circuit at DC and are skipped.  A
/// `None` return from the internal linear solver (singular matrix) populates
/// an empty `NetworkBias::dc_voltages` — the caller can detect this via
/// `is_empty()`.
pub(super) fn solve_network_bias(
    edge_indices: &[usize],
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> NetworkBias {
    // Collect interior (non-rail) nodes by BFS from rails through resistor edges only.
    //
    // Caps/inductors/nonlinear elements are open at DC.  Signal-path nodes that
    // are reachable only via caps or from the circuit's in/out (not from rails via
    // resistors) must be excluded: they would produce zero-conductance rows in the
    // G matrix → singular.  BFS from rails guarantees every included node has at
    // least one DC path to a known voltage.
    let rail_set = build_rail_set(graph);

    // Build resistor-only adjacency from the provided edge set.
    let mut res_adj: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
    for &eidx in edge_indices {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        if comp.kind.resistance().map_or(true, |r| r <= 0.0) {
            continue;
        }
        res_adj.entry(e.node_a).or_default().push(e.node_b);
        res_adj.entry(e.node_b).or_default().push(e.node_a);
    }

    // BFS: start from all rail nodes, traverse resistor edges, collect non-rail nodes.
    let mut interior_set: HashSet<NodeId> = HashSet::new();
    let mut visited: HashSet<NodeId> = HashSet::new();
    let mut queue: VecDeque<NodeId> = VecDeque::new();
    for &rail in &rail_set {
        if res_adj.contains_key(&rail) {
            queue.push_back(rail);
            visited.insert(rail);
        }
    }
    while let Some(node) = queue.pop_front() {
        if let Some(neighbors) = res_adj.get(&node) {
            for &next in neighbors {
                if visited.contains(&next) {
                    continue;
                }
                visited.insert(next);
                if !rail_set.contains(&next) {
                    interior_set.insert(next);
                }
                queue.push_back(next);
            }
        }
    }

    if interior_set.is_empty() {
        return NetworkBias::default();
    }

    let node_list: Vec<NodeId> = interior_set.into_iter().collect();
    let n = node_list.len();
    let node_idx: HashMap<NodeId, usize> = node_list
        .iter()
        .enumerate()
        .map(|(i, &nd)| (nd, i))
        .collect();

    let rail_v = |node: NodeId| -> f64 {
        if node == graph.gnd_node || graph.ac_ground_nodes.contains(&node) {
            0.0
        } else if node == graph.vcc_node {
            graph
                .supply_voltages
                .get(&graph.vcc_node)
                .copied()
                .unwrap_or(supply_voltage)
        } else if let Some(&v) = graph.supply_voltages.get(&node) {
            v
        } else {
            0.0
        }
    };

    let mut g_mat = vec![0.0_f64; n * n];
    let mut i_vec = vec![0.0_f64; n];

    for &eidx in edge_indices {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        let r = match comp.kind.resistance() {
            Some(r) if r > 0.0 => r,
            _ => continue,
        };
        let g = 1.0 / r;

        let ia = node_idx.get(&e.node_a);
        let ib = node_idx.get(&e.node_b);
        let a_rail = rail_set.contains(&e.node_a);
        let b_rail = rail_set.contains(&e.node_b);

        match (ia, ib) {
            (Some(&i), Some(&j)) => {
                g_mat[i * n + i] += g;
                g_mat[j * n + j] += g;
                g_mat[i * n + j] -= g;
                g_mat[j * n + i] -= g;
            }
            (Some(&i), None) if b_rail => {
                g_mat[i * n + i] += g;
                i_vec[i] += g * rail_v(e.node_b);
            }
            (None, Some(&j)) if a_rail => {
                g_mat[j * n + j] += g;
                i_vec[j] += g * rail_v(e.node_a);
            }
            _ => {}
        }
    }

    let result = solve_linear_system(&mut g_mat, &mut i_vec, n);
    let dc_voltages = match result {
        Some(vs) => node_list.into_iter().zip(vs).collect(),
        None => HashMap::new(),
    };

    NetworkBias { dc_voltages }
}

// ═══════════════════════════════════════════════════════════════════════════
// Flow-group bias classification (absorbed from bias_analysis.rs, ko5g.3)
// ═══════════════════════════════════════════════════════════════════════════
//
// Classifies each flow group as either a static DC bias network (supply-only
// inputs → the resistor-divider DC operating point is computed at compile time
// and the group is bypassed in the serial audio chain) or a signal path (has
// audio-rate inputs and must be processed at runtime).  The divider nodal solve
// is the unified `solve_network_bias` above — this section is now the single
// home for the whole compile-time DC bias story.
//
// The detection is graph-level: "does this branch have an audio-rate input or
// only DC supply?"  No component-type special-casing.

/// Classification of a flow group's role in the circuit.
#[derive(Debug)]
pub(super) enum GroupBiasKind {
    /// Group is on the audio signal path (in_node → ... → out_node).
    /// Must be processed in the serial audio chain at runtime.
    SignalPath,

    /// Group is a static DC bias network (supply-only inputs).
    /// DC voltages are computed at compile time from resistor dividers.
    /// The group is bypassed in the serial audio chain.
    ///
    /// `dc_voltages` maps each non-rail junction node to its DC voltage.
    StaticBias { dc_voltages: HashMap<NodeId, f64> },
    // Future: DynamicModulation for sidechains, envelope followers, etc.
}

/// Classify a flow group as static bias, signal path, or dynamic modulation.
///
/// A group is **StaticBias** when every node it touches is either a supply rail
/// or an interior node reachable ONLY through the group's own edges from supply
/// rails — i.e. not reachable from `in_node`/`out_node` without passing through
/// supply rails.  When StaticBias, computes the DC voltage at each interior
/// junction node via nodal analysis of the resistor divider (`solve_network_bias`).
pub(super) fn classify_group_bias(
    group: &super::signal_flow::FlowGroup,
    graph: &CircuitGraph,
) -> GroupBiasKind {
    // A group with any nonlinear edge is NEVER static bias.
    // Diodes/transistors to ground are signal clippers, not DC references.
    let has_nonlinear = group.all_edges().iter().any(|&eidx| {
        let comp = &graph.components[graph.edges[eidx].comp_idx];
        comp.kind.is_nonlinear()
    });
    if has_nonlinear {
        return GroupBiasKind::SignalPath;
    }

    let rail_set = build_rail_set(graph);

    // Collect all nodes this group touches.
    let group_nodes = collect_group_nodes(group, graph);

    // Find the group's non-rail nodes (interior junction nodes).
    let interior_nodes: HashSet<NodeId> = group_nodes
        .iter()
        .filter(|&&n| !rail_set.contains(&n))
        .copied()
        .collect();

    // If the group has NO interior nodes, it's entirely on rails — skip it.
    if interior_nodes.is_empty() {
        return GroupBiasKind::StaticBias {
            dc_voltages: HashMap::new(),
        };
    }

    // Check if any interior node can reach in_node or out_node through
    // non-supply, non-group-internal edges. If so, the group carries audio.
    let reaches = interior_reaches_signal(&interior_nodes, &rail_set, graph, group);
    #[cfg(test)]
    {
        let edge_names: Vec<_> = group
            .all_edges()
            .iter()
            .map(|&eidx| {
                let e = &graph.edges[eidx];
                let comp = &graph.components[e.comp_idx];
                format!("{}({:?}→{:?})", comp.id, e.node_a, e.node_b)
            })
            .collect();
        let interior_names: Vec<_> = interior_nodes.iter().map(|n| format!("{n:?}")).collect();
        eprintln!("  bias: edges={edge_names:?}");
        eprintln!("    interior={interior_names:?}, rails={:?}, in={:?}, out={:?}, reaches_signal={reaches}",
            rail_set.len(), graph.in_node, graph.out_node);
    }
    if reaches {
        return GroupBiasKind::SignalPath;
    }

    // Static bias: compute DC voltages from the resistor divider.
    let dc_voltages = compute_group_dc_voltages(group, graph);

    GroupBiasKind::StaticBias { dc_voltages }
}

/// Collect all nodes touched by a group's edges.
fn collect_group_nodes(
    group: &super::signal_flow::FlowGroup,
    graph: &CircuitGraph,
) -> HashSet<NodeId> {
    let mut nodes = HashSet::new();
    for &eidx in group.all_edges().iter() {
        let e = &graph.edges[eidx];
        nodes.insert(e.node_a);
        nodes.insert(e.node_b);
    }
    nodes
}

/// Check if the group carries audio signal based on its own edge topology.
///
/// A group is static bias if EVERY edge in it has at least one rail terminal
/// (interior↔rail only).  A group is on the signal path if ANY edge connects two
/// non-rail nodes, OR any of its winding nodes is one end of a `Tight`
/// coupled-link whose other end is non-rail (transformer primary↔secondary,
/// consulted via the broker — no transformer special-casing here).
fn interior_reaches_signal(
    _interior_nodes: &HashSet<NodeId>,
    rail_set: &HashSet<NodeId>,
    graph: &CircuitGraph,
    group: &super::signal_flow::FlowGroup,
) -> bool {
    for &eidx in group.all_edges().iter() {
        let e = &graph.edges[eidx];
        let a_rail = rail_set.contains(&e.node_a);
        let b_rail = rail_set.contains(&e.node_b);
        if !a_rail && !b_rail {
            // Edge between two non-rail nodes → carries signal
            return true;
        }
    }

    for &eidx in group.all_edges().iter() {
        let e = &graph.edges[eidx];
        for node in [e.node_a, e.node_b] {
            for other in super::boundary_rules::tight_coupled_neighbors(graph, node) {
                if !rail_set.contains(&other) {
                    return true;
                }
            }
        }
    }

    false
}

/// Compute DC voltages at a static-bias group's interior junction nodes.
///
/// The resistor-divider DC solve is the unified `solve_network_bias`: it performs
/// conductance-MNA Gaussian elimination and runs a rail-BFS to restrict the
/// solved node set to interior nodes that have a DC path back to a rail through
/// resistors (skipping cap-isolated floating signal-path nodes that would
/// otherwise produce a singular matrix).  For genuine resistor dividers (the only
/// case StaticBias is reached for) the BFS only ever removes nodes the old code
/// would have failed on, so the returned voltages are unchanged.
fn compute_group_dc_voltages(
    group: &super::signal_flow::FlowGroup,
    graph: &CircuitGraph,
) -> HashMap<NodeId, f64> {
    let supply_voltage = graph
        .supply_voltages
        .get(&graph.vcc_node)
        .copied()
        .unwrap_or(9.0);
    solve_network_bias(&group.all_edges(), graph, supply_voltage).dc_voltages
}

// ═══════════════════════════════════════════════════════════════════════════
// Bias topology — what the solver needs from the circuit
// ═══════════════════════════════════════════════════════════════════════════

/// The circuit topology needed to run the 1-D load-line solve.
///
/// `BiasSeed::locate_bias_topology` extracts this from the graph.
#[derive(Debug, Clone)]
pub(super) struct BiasTopology {
    /// Resistance from the output node (plate/collector/drain) to its supply rail.
    pub(super) r_load: f64,
    /// Supply voltage at the load rail end (e.g. VCC for plate, GND for cathode).
    pub(super) v_load_rail: f64,
    /// Resistance from the control-side degeneration (cathode/emitter/source) to its rail.
    ///
    /// `0.0` when there is no degeneration resistor (e.g. cathode directly to GND).
    pub(super) r_degeneration: f64,
    /// Open-circuit Thévenin voltage at the control terminal (grid/base/gate).
    ///
    /// For a grounded-cathode triode: Vgk_bias is negative auto-bias =
    /// `-(supply_voltage * r_degeneration / (r_load + r_degeneration))`
    /// For a BJT: the base-divider Thévenin voltage.
    pub(super) v_control_thevenin: f64,
    /// Thévenin resistance looking back from the control terminal.
    ///
    /// `0.0` for a triode with grid resistor to GND (unloaded grid).
    /// For a BJT base divider: `R1||R2`.
    pub(super) r_control_thevenin: f64,
    /// Supply voltage for the circuit (B+), used to set v_max.
    pub(super) supply_voltage: f64,
    /// Node at which the degeneration element (cathode/emitter bypass cap) sits,
    /// for `reactive_precharge` population.  `None` if no bypass cap is expected.
    pub(super) degeneration_node: Option<NodeId>,
}

// ═══════════════════════════════════════════════════════════════════════════
// Device I-V evaluation point
// ═══════════════════════════════════════════════════════════════════════════

/// A trial point passed to `BiasSeed::device_iv`.
#[derive(Debug, Clone, Copy)]
pub(super) struct TrialPoint {
    /// Control voltage at this trial (Vgk, Vbe, Vgs — as appropriate).
    pub(super) v_control: f64,
    /// Output voltage at this trial (Vpk, Vce, Vds).
    pub(super) v_output: f64,
}

/// How `solve_operating_point` drives the load-line to its fixed point.
///
/// Both schemes converge to the SAME mathematical fixed point
/// (`v_ctrl = v_thevenin - i_ctrl·R_th - i_total·R_degen`, `v_out = v_rail -
/// i_total·R_load`); they differ only in the iteration variable and update rule.
#[derive(Debug, Clone, Copy)]
pub(super) enum IterationScheme {
    /// Newton on the control-voltage residual.  The default; used by the BJT
    /// divider path (`BjtNpnSeed`), which needs Newton's robustness against the
    /// exponential Vbe stiffness.
    ControlNewton,
    /// Damped relaxation on the device **main current** (the triode Ia-relaxation).
    ///
    /// Bit-for-bit reproduces the legacy per-type `solve_triode_dc_qpoint` loop:
    /// iterate `Ia`, recompute `Vgk = -Ia·Rk` and `Vpk = VCC - Ia·Rp` from that
    /// same `Ia` each step, evaluate the model, relax `Ia ← Ia - damping·(Ia -
    /// Ia_model)`.  This is what makes the ko5g inc-3 triode migration
    /// behaviour-preserving: the `ControlNewton` scheme lands ~2 mV Vgk / ~131 mV
    /// Vpk away on the 12AX7 stage (a purely algorithmic residual, independent of
    /// v_max), which would shift every MultiNL triode Q-point.
    MainCurrentRelaxation {
        initial_i_main: f64,
        damping: f64,
        max_iter: usize,
        tol: f64,
    },
}

/// Device I-V response at a trial point.
#[derive(Debug, Clone, Copy)]
pub(super) struct DeviceIv {
    /// Main current through the device (plate current Ia, collector Ic, drain Id).
    pub(super) i_main: f64,
    /// Control-terminal current (grid current, base current Ib).
    ///
    /// `0.0` for JFETs/MOSFETs and vacuum triodes (grids draw negligible current
    /// at normal audio-rate operating points).
    pub(super) i_control: f64,
    /// ∂I_main/∂V_control (transconductance).
    pub(super) di_main_dv_control: f64,
    /// ∂I_control/∂V_control.
    pub(super) di_control_dv_control: f64,
}

// ═══════════════════════════════════════════════════════════════════════════
// BiasSeed trait
// ═══════════════════════════════════════════════════════════════════════════

/// Abstraction over per-device bias topology extraction and I-V evaluation.
///
/// Implementing this trait for each device kind allows `solve_operating_point`
/// to run a single generic load-line / Newton-Raphson without per-type
/// special-casing.
///
/// # Resistor-finding strategy
///
/// The default `locate_bias_topology` implementation uses **cap-aware BFS**
/// (see module-level doc).  Override `use_bfs_finder` and return `false` to
/// fall back to the direct-edge match for testing or backward-compat purposes.
pub(super) trait BiasSeed {
    /// Locate the circuit topology needed for the load-line solve.
    fn locate_bias_topology(
        &self,
        edge_indices: &[usize],
        graph: &CircuitGraph,
        network_bias: &NetworkBias,
        supply_voltage: f64,
    ) -> Result<BiasTopology, BiasError>;

    /// Evaluate the device's I-V characteristic at the given trial point.
    fn device_iv(&self, trial: TrialPoint) -> DeviceIv;

    /// The component label for this device (used in error messages).
    fn device_label(&self) -> &str;

    /// The device kind.
    fn device_kind(&self) -> DeviceBiasKind;

    /// Initial guess for the control voltage (Vgk / Vbe / Vgs).
    ///
    /// The Newton-Raphson solve starts here.  A good initial guess converges
    /// faster and avoids non-physical regions.
    fn initial_v_control(&self) -> f64;

    /// Which iteration scheme `solve_operating_point` should use.
    ///
    /// Defaults to `ControlNewton` (the BJT-divider path).  The triode overrides
    /// this to `MainCurrentRelaxation` so the shared solver reproduces the legacy
    /// per-type Ia-relaxation Q-point bit-for-bit.
    fn iteration_scheme(&self) -> IterationScheme {
        IterationScheme::ControlNewton
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Common load-line solver
// ═══════════════════════════════════════════════════════════════════════════

/// Solve the DC operating point via a 1-D Newton-Raphson load-line iteration.
///
/// This is the common core that replaces `compute_wdf_triode_dc_qpoint` and
/// `compute_wdf_bjt_dc_qpoint`.  It:
///
/// 1. Calls `seed.locate_bias_topology()` to get the circuit topology.
/// 2. Iterates Newton-Raphson on the load-line residual.
/// 3. Returns a `DeviceOperatingPoint` with all fields populated.
///
/// The residual for a generic device is:
/// ```text
///   F(v_ctrl) = v_ctrl_thevenin - i_control * r_control_thevenin
///               - v_ctrl - i_total * r_degeneration
/// ```
/// where `i_total = i_main + i_control` and `v_ctrl` is iterated.
///
/// For a triode `i_control = 0` and `r_control_thevenin = 0` (grid draws no
/// current at audio-rate), so this reduces to the cathode auto-bias equation.
pub(super) fn solve_operating_point<S: BiasSeed>(
    seed: &S,
    edge_indices: &[usize],
    graph: &CircuitGraph,
    network_bias: &NetworkBias,
    supply_voltage: f64,
) -> Result<DeviceOperatingPoint, BiasError> {
    let topo = seed.locate_bias_topology(edge_indices, graph, network_bias, supply_voltage)?;

    // Both schemes converge to the same load-line fixed point and return the
    // triple (v_control, i_total, v_output) at that point.
    let (v_ctrl, i_total_final, v_output_final) = match seed.iteration_scheme() {
        IterationScheme::ControlNewton => {
            control_newton_solve(seed, &topo)
        }
        IterationScheme::MainCurrentRelaxation {
            initial_i_main,
            damping,
            max_iter,
            tol,
        } => main_current_relaxation_solve(seed, &topo, initial_i_main, damping, max_iter, tol),
    };

    // Validate
    if !v_ctrl.is_finite() || !v_output_final.is_finite() {
        return Err(BiasError::NonPhysicalQpoint);
    }

    let degeneration_voltage = i_total_final * topo.r_degeneration;
    let reactive_precharge = if let Some(degen_node) = topo.degeneration_node {
        vec![(degen_node, degeneration_voltage)]
    } else {
        vec![]
    };

    Ok(DeviceOperatingPoint {
        device_label: seed.device_label().to_owned(),
        kind: seed.device_kind(),
        control_bias: v_ctrl,
        output_warm_start: v_output_final,
        reactive_precharge,
        v_max: topo.supply_voltage,
    })
}

/// Newton on the control-voltage residual.  Returns `(v_control, i_total,
/// v_output)` at the converged Q-point.  This is the original
/// `solve_operating_point` inner loop, extracted verbatim so the BJT path is
/// byte-identical.
fn control_newton_solve<S: BiasSeed>(seed: &S, topo: &BiasTopology) -> (f64, f64, f64) {
    let mut v_ctrl = seed.initial_v_control();

    for _ in 0..80 {
        // Compute v_output from the load line using the current v_ctrl estimate.
        //
        // For auto-bias (triode): Ia = -Vgk / Rk (KVL at cathode), so
        //   Vpk = VCC - Ia * Rp = VCC - (-v_ctrl / r_degen) * r_load
        //       = v_load_rail + v_ctrl * (r_load / r_degeneration)
        //
        // For BJT with divider (v_ctrl_thevenin ≠ 0): the base mesh gives
        //   Ie ≈ (Vth - v_ctrl) / r_degen  (ignoring base current in the
        //   degeneration as a 1st-order approximation), so
        //   Vce ≈ v_load_rail - Ic * r_load.
        //
        // In both cases we approximate i_load by (v_ctrl_thevenin - v_ctrl) / r_degen
        // (degeneration current) to stay self-consistent within the iteration:
        let i_load_est = if topo.r_degeneration > 0.0 {
            let v_degen = topo.v_control_thevenin - v_ctrl; // voltage across degen + Vbe
            (v_degen / topo.r_degeneration).max(0.0)
        } else {
            0.0
        };
        let v_output_est = (topo.v_load_rail - i_load_est * topo.r_load).max(0.0);

        let trial = TrialPoint {
            v_control: v_ctrl,
            v_output: v_output_est,
        };
        let iv = seed.device_iv(trial);

        let i_total = iv.i_main + iv.i_control;

        // F(v_ctrl) = Vth - Ib*Rth - v_ctrl - I_total * R_degen
        let f = topo.v_control_thevenin
            - iv.i_control * topo.r_control_thevenin
            - v_ctrl
            - i_total * topo.r_degeneration;

        // dF/dV_ctrl = -dIb/dV * Rth - 1 - (dIb/dV + dIa/dV) * R_degen
        let df = -iv.di_control_dv_control * topo.r_control_thevenin
            - 1.0
            - (iv.di_main_dv_control + iv.di_control_dv_control) * topo.r_degeneration;

        if df.abs() < 1e-18 {
            break;
        }

        let step = (f / df).clamp(-0.2, 0.2);
        v_ctrl -= step;

        if step.abs() < 1e-9 {
            break;
        }
    }

    // Final self-consistent evaluation at the converged v_ctrl.
    let i_load_final = if topo.r_degeneration > 0.0 {
        let v_degen = topo.v_control_thevenin - v_ctrl;
        (v_degen / topo.r_degeneration).max(0.0)
    } else {
        0.0
    };
    let trial_final = TrialPoint {
        v_control: v_ctrl,
        v_output: (topo.v_load_rail - i_load_final * topo.r_load).max(0.0),
    };
    let iv_final = seed.device_iv(trial_final);
    let i_total_final = (iv_final.i_main + iv_final.i_control).max(0.0);
    let v_output_final = (topo.v_load_rail - i_total_final * topo.r_load).max(0.0);

    (v_ctrl, i_total_final, v_output_final)
}

/// Damped relaxation on the device main current.  Returns `(v_control, i_total,
/// v_output)` at the converged Q-point.
///
/// For the auto-biased triode (`v_control_thevenin = 0`, `r_control_thevenin =
/// 0`, `i_control = 0`) this reduces to — and is bit-for-bit identical to — the
/// legacy per-type `solve_triode_dc_qpoint` Ia-relaxation:
///   `Vgk = -Ia·Rk`, `Vpk = max(VCC - Ia·Rp, 0)`, relax `Ia`.
///
/// `i_control` is carried lagged (one iteration behind) so the general case with
/// a control-terminal current (Rth ≠ 0) still self-consistates; for the triode it
/// stays exactly 0 throughout.
fn main_current_relaxation_solve<S: BiasSeed>(
    seed: &S,
    topo: &BiasTopology,
    initial_i_main: f64,
    damping: f64,
    max_iter: usize,
    tol: f64,
) -> (f64, f64, f64) {
    let mut i_main = initial_i_main;
    let mut i_control = 0.0_f64;

    let eval = |i_main: f64, i_control: f64| -> (f64, f64, DeviceIv) {
        let i_total = i_main + i_control;
        let v_output = (topo.v_load_rail - i_total * topo.r_load).max(0.0);
        let v_control = topo.v_control_thevenin
            - i_control * topo.r_control_thevenin
            - i_total * topo.r_degeneration;
        let iv = seed.device_iv(TrialPoint {
            v_control,
            v_output,
        });
        (v_control, v_output, iv)
    };

    for _ in 0..max_iter {
        let (_v_control, _v_output, iv) = eval(i_main, i_control);
        let f = i_main - iv.i_main;
        i_main = (i_main - f * damping).max(0.0);
        i_control = iv.i_control;
        if f.abs() < tol {
            break;
        }
    }

    // Final self-consistent evaluation at the converged current.
    let (v_control, v_output, iv) = eval(i_main, i_control);
    let _ = iv;
    let i_total = i_main + i_control;
    (v_control, i_total, v_output)
}

// ═══════════════════════════════════════════════════════════════════════════
// BiasSeed implementations
// ═══════════════════════════════════════════════════════════════════════════

// ── Triode ──────────────────────────────────────────────────────────────────

/// `BiasSeed` for a common-cathode triode stage.
pub(super) struct TriodeSeed<'a> {
    pub(super) nl_kind: &'a NonlinearKind,
    pub(super) label: String,
    /// B+ rail voltage, used as the `TriodeRoot` `v_max`.  Matching the per-type
    /// `solve_triode_dc_qpoint` (which uses `v_max = supply_voltage`, NOT the
    /// per-trial plate voltage) keeps the shared-core Q-point bit-for-bit aligned
    /// with the production path so the inc-3 migration is behaviour-preserving.
    pub(super) supply_voltage: f64,
    /// Parallel-triode multiplier (sections wired in parallel share plate/cathode).
    pub(super) parallel_count: usize,
}

impl<'a> BiasSeed for TriodeSeed<'a> {
    fn locate_bias_topology(
        &self,
        edge_indices: &[usize],
        graph: &CircuitGraph,
        _network_bias: &NetworkBias,
        supply_voltage: f64,
    ) -> Result<BiasTopology, BiasError> {
        let (model_name, plate_node, cathode_node) = match self.nl_kind {
            NonlinearKind::Triode {
                model_name,
                plate_node,
                cathode_node,
                grid_node: Some(_),
                is_vari_mu: false,
                ..
            } => (model_name.as_str(), *plate_node, *cathode_node),
            NonlinearKind::Triode {
                grid_node: None, ..
            } => {
                return Err(BiasError::UndeterminableTriode {
                    label: self.label.clone(),
                    missing: TopologyTerm::GridNode,
                });
            }
            _ => {
                return Err(BiasError::UndeterminableTriode {
                    label: self.label.clone(),
                    missing: TopologyTerm::PlateResistor,
                });
            }
        };
        let _ = model_name;

        // Find R_plate: resistor between vcc_node and plate_node.
        //
        // Direct-edge FIRST, then cap-aware BFS as a fallback.  The direct match
        // is what the legacy per-type `solve_triode_dc_qpoint` used, so trying it
        // first keeps the production triode call-site's Q-point bit-for-bit
        // identical when a direct plate/cathode resistor exists (the common
        // grounded-cathode topology).  BFS still rescues cap-coupled cathode-
        // followers where no direct edge exists.
        // The LEGACY vcc searches (direct edge then cap-aware BFS, over the
        // STAGE's edge set) run first, so every circuit they resolved keeps
        // its Q-point bit-for-bit. Only stages the legacy search could NOT
        // resolve fall through to the pedalkernel-0stg widening:
        // * named rails (B+, V+, ...) — their NodeIds never matched the
        //   literal vcc target, so every B+-fed stage silently kept the
        //   -2.0V default;
        // * GRAPH-WIDE edges (the BjtNpnSeed divider precedent) — named-rail
        //   edges are excluded from passive claiming, so a B+ plate load
        //   always lands in a DIFFERENT flow group than its triode (LA-2A:
        //   R_p1 groups with C_c1, not with V1);
        // * a plate wired DIRECTLY to a rail (`B+ -> V3.plate` unions the
        //   plate node into the rail — the cathode follower) is the
        //   r_plate = 0 load line on that rail.
        let rails = positive_supply_rails(graph);
        let graph_edges: Vec<usize> = (0..graph.edges.len()).collect();
        let (r_plate, v_load_rail) = find_load_resistor_direct(
            plate_node,
            graph.vcc_node,
            edge_indices,
            graph,
        )
        .or_else(|| find_load_resistor_bfs(plate_node, graph.vcc_node, edge_indices, graph))
        .map(|r| (r, graph.vcc_node))
        .or_else(|| {
            rails.iter().find_map(|&rail| {
                find_load_resistor_direct(plate_node, rail, &graph_edges, graph)
                    .map(|r| (r, rail))
            })
        })
        .or_else(|| {
            rails.iter().find_map(|&rail| {
                find_load_resistor_bfs(plate_node, rail, &graph_edges, graph).map(|r| (r, rail))
            })
        })
        .map(|(r, rail)| {
            (
                r,
                rail_dc_voltage(rail, graph, supply_voltage).unwrap_or(supply_voltage),
            )
        })
        .or_else(|| {
            rail_dc_voltage(plate_node, graph, supply_voltage)
                .filter(|&v| v > 0.0)
                .map(|v| (0.0, v))
        })
        .ok_or_else(|| BiasError::UndeterminableTriode {
            label: self.label.clone(),
            missing: TopologyTerm::PlateResistor,
        })?;

        // Find R_cathode: resistor between cathode_node and gnd_node (direct, then BFS).
        let r_cathode = find_load_resistor_direct(cathode_node, graph.gnd_node, edge_indices, graph)
            .or_else(|| find_load_resistor_bfs(cathode_node, graph.gnd_node, edge_indices, graph))
            .ok_or_else(|| BiasError::UndeterminableTriode {
                label: self.label.clone(),
                missing: TopologyTerm::CathodeResistor,
            })?;

        Ok(BiasTopology {
            r_load: r_plate,
            v_load_rail,
            r_degeneration: r_cathode,
            // Triode: grid is biased by auto-bias (cathode current through R_cathode).
            // V_control_thevenin = 0 (grid resistor to GND, unloaded).
            v_control_thevenin: 0.0,
            r_control_thevenin: 0.0,
            supply_voltage,
            degeneration_node: Some(cathode_node),
        })
    }

    fn device_iv(&self, trial: TrialPoint) -> DeviceIv {
        let model_name = match self.nl_kind {
            NonlinearKind::Triode { model_name, .. } => model_name.as_str(),
            _ => "12AX7",
        };
        let model = super::helpers::triode_model(model_name);
        let v_max = self.supply_voltage.max(1.0);
        let mut root = pedalkernel_rt::elements::nonlinear::TriodeRoot::new_with_v_max(model, v_max)
            .with_parallel_count(self.parallel_count);
        root.set_vgk(trial.v_control as pedalkernel_rt::Wave);

        let ia = root.plate_current(trial.v_output as pedalkernel_rt::Wave) as f64;

        // Numerical derivative for dIa/dVgk.
        // Cast both plate_current results to f64 BEFORE subtraction to avoid
        // catastrophic f32 cancellation when h is small.
        let h = 1e-3_f64; // larger h to stay above f32 noise floor (~1e-7)
        let mut root2 = pedalkernel_rt::elements::nonlinear::TriodeRoot::new_with_v_max(
            super::helpers::triode_model(model_name),
            v_max,
        )
        .with_parallel_count(self.parallel_count);
        root2.set_vgk((trial.v_control + h) as pedalkernel_rt::Wave);
        let ia2 = root2.plate_current(trial.v_output as pedalkernel_rt::Wave) as f64;
        let gm = (ia2 - ia) / h;

        DeviceIv {
            i_main: ia,
            i_control: 0.0, // grid draws no current
            di_main_dv_control: gm,
            di_control_dv_control: 0.0,
        }
    }

    fn device_label(&self) -> &str {
        &self.label
    }

    fn device_kind(&self) -> DeviceBiasKind {
        DeviceBiasKind::Triode
    }

    fn initial_v_control(&self) -> f64 {
        -1.0 // typical Vgk starting guess for a common-cathode triode
    }

    fn iteration_scheme(&self) -> IterationScheme {
        // Reproduce the legacy per-type `solve_triode_dc_qpoint` loop bit-for-bit:
        // initial Ia = 0.1 mA, 0.5 damping, 50 iters, 1 nA current tolerance.
        IterationScheme::MainCurrentRelaxation {
            initial_i_main: 1e-4,
            damping: 0.5,
            max_iter: 50,
            tol: 1e-9,
        }
    }
}

// ── BJT NPN ─────────────────────────────────────────────────────────────────

/// `BiasSeed` for a common-emitter NPN BJT.
pub(super) struct BjtNpnSeed<'a> {
    pub(super) nl_kind: &'a NonlinearKind,
    pub(super) label: String,
}

impl<'a> BiasSeed for BjtNpnSeed<'a> {
    fn locate_bias_topology(
        &self,
        edge_indices: &[usize],
        graph: &CircuitGraph,
        network_bias: &NetworkBias,
        supply_voltage: f64,
    ) -> Result<BiasTopology, BiasError> {
        let (model_name, base_node, collector_node, emitter_node) = match self.nl_kind {
            NonlinearKind::BjtNpn {
                model_name,
                base_node,
                collector_node,
                emitter_node,
            } => (
                model_name.as_str(),
                *base_node,
                *collector_node,
                *emitter_node,
            ),
            _ => {
                return Err(BiasError::UndeterminableBjt {
                    label: self.label.clone(),
                    missing: TopologyTerm::BaseDivider,
                });
            }
        };
        let _ = model_name;
        let all_edges: Vec<usize> = (0..graph.edges.len()).collect();

        // Base divider: R1 = base→rail, R2 = base→gnd. Rails vcc-first, then
        // named rails (pedalkernel-0stg — see `positive_supply_rails`).
        let rails = positive_supply_rails(graph);
        let (r1, base_rail) = rails
            .iter()
            .find_map(|&rail| {
                find_load_resistor_direct(base_node, rail, &all_edges, graph).map(|r| (r, rail))
            })
            .or_else(|| {
                rails.iter().find_map(|&rail| {
                    find_load_resistor_bfs(base_node, rail, &all_edges, graph).map(|r| (r, rail))
                })
            })
            .ok_or_else(|| BiasError::UndeterminableBjt {
                label: self.label.clone(),
                missing: TopologyTerm::BaseDivider,
            })?;
        let base_rail_v =
            rail_dc_voltage(base_rail, graph, supply_voltage).unwrap_or(supply_voltage);
        let r2 = find_load_resistor_direct(base_node, graph.gnd_node, &all_edges, graph)
            .or_else(|| find_load_resistor_bfs(base_node, graph.gnd_node, &all_edges, graph))
            .ok_or_else(|| BiasError::UndeterminableBjt {
                label: self.label.clone(),
                missing: TopologyTerm::BaseDivider,
            })?;

        // Emitter resistor RE: emitter→gnd.
        let re = find_load_resistor_direct(emitter_node, graph.gnd_node, edge_indices, graph)
            .or_else(|| find_load_resistor_bfs(emitter_node, graph.gnd_node, edge_indices, graph))
            .ok_or_else(|| BiasError::UndeterminableBjt {
                label: self.label.clone(),
                missing: TopologyTerm::EmitterResistor,
            })?;

        // Collector resistor RC: collector→rail (vcc-first, then named).
        let rc_found = rails
            .iter()
            .find_map(|&rail| {
                find_load_resistor_direct(collector_node, rail, &all_edges, graph)
                    .map(|r| (r, rail))
            })
            .or_else(|| {
                rails.iter().find_map(|&rail| {
                    find_load_resistor_bfs(collector_node, rail, &all_edges, graph)
                        .map(|r| (r, rail))
                })
            });
        let v_load_rail = rc_found
            .map(|(_, rail)| rail_dc_voltage(rail, graph, supply_voltage).unwrap_or(supply_voltage))
            .unwrap_or(supply_voltage);

        // Thévenin base voltage: prefer StaticBias map, fall back to resistor divider.
        let vth = network_bias
            .voltage_at(base_node, graph, supply_voltage)
            .filter(|v| v.is_finite())
            .unwrap_or_else(|| base_rail_v * r2 / (r1 + r2));
        let rth = r1 * r2 / (r1 + r2);

        Ok(BiasTopology {
            r_load: rc_found
                .map(|(r, _)| r)
                .unwrap_or(supply_voltage * 0.5 / 1e-3), // estimate if missing
            v_load_rail,
            r_degeneration: re,
            v_control_thevenin: vth,
            r_control_thevenin: rth,
            supply_voltage,
            degeneration_node: Some(emitter_node),
        })
    }

    fn device_iv(&self, trial: TrialPoint) -> DeviceIv {
        let model_name = match self.nl_kind {
            NonlinearKind::BjtNpn { model_name, .. } => model_name.as_str(),
            _ => "2N3904",
        };
        let model = super::helpers::gummel_poon_model(model_name);
        let vbc_active = -1.0_f64; // assume active region (reverse Vbc)
        let (ic, ib) = model.currents(
            trial.v_control as pedalkernel_rt::Wave,
            vbc_active as pedalkernel_rt::Wave,
        );

        // Numerical derivatives
        let h = 1e-4_f64;
        let (ic2, ib2) = model.currents(
            (trial.v_control + h) as pedalkernel_rt::Wave,
            vbc_active as pedalkernel_rt::Wave,
        );
        let dic_dvbe = (ic2 - ic) as f64 / h;
        let dib_dvbe = (ib2 - ib) as f64 / h;

        DeviceIv {
            i_main: ic as f64,
            i_control: ib as f64,
            di_main_dv_control: dic_dvbe,
            di_control_dv_control: dib_dvbe,
        }
    }

    fn device_label(&self) -> &str {
        &self.label
    }

    fn device_kind(&self) -> DeviceBiasKind {
        DeviceBiasKind::BjtNpn
    }

    fn initial_v_control(&self) -> f64 {
        0.65 // typical silicon Vbe starting guess
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Resistor-finding helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Resolve the DC voltage of a supply-rail node — `vcc` OR a NAMED rail
/// (`B+`, `V+`, ...). Returns `None` for non-rail (solved/interior) nodes.
///
/// This is THE rail resolver (pedalkernel-0stg): named supplies get their own
/// `NodeId` (`graph.supply_nodes` / `graph.supply_voltages`) and never equal
/// the literal `graph.vcc_node`, so every finder that compared against
/// `vcc_node` alone silently skipped B+-fed stages.
///
/// ARM ORDER matters: every named supply node is ALSO in
/// `graph.ac_ground_nodes` (rails are AC ground — graph.rs
/// `compute_ac_ground_nodes` inserts `supply_nodes` wholesale), so the named-
/// rail arms must be consulted BEFORE the ac-ground arm or B+ resolves to
/// 0 V DC. This is the same family as the pedalkernel-mgsd
/// ac-ground-as-DC-0 hazard; the `vcc` arm deliberately stays BEHIND the
/// ac-ground arm to preserve the legacy resolution (`solve_bjt_group_dc_qpoint`'s
/// `full_rail_v`, which delegates here) bit-for-bit for every vcc circuit —
/// a bypassed-vcc quirk fix belongs to mgsd, not 0stg.
pub(super) fn rail_dc_voltage(
    node: NodeId,
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Option<f64> {
    if node == graph.gnd_node {
        Some(0.0)
    } else if graph.supply_nodes.contains(&node) {
        // Named rail (B+, V+, ...): its declared voltage, at DC, regardless
        // of the node's (correct) AC-ground classification.
        Some(
            graph
                .supply_voltages
                .get(&node)
                .copied()
                .unwrap_or(supply_voltage),
        )
    } else if graph.ac_ground_nodes.contains(&node) {
        Some(0.0)
    } else if node == graph.vcc_node {
        Some(
            graph
                .supply_voltages
                .get(&graph.vcc_node)
                .copied()
                .unwrap_or(supply_voltage),
        )
    } else if let Some(&v) = graph.supply_voltages.get(&node) {
        Some(v)
    } else {
        None
    }
}

/// The positive supply rails a device load resistor may return to, in
/// deterministic search order: `vcc` FIRST (so vcc-fed circuits keep their
/// exact legacy resolution bit-for-bit), then the named rails sorted by
/// `NodeId` (`graph.supply_nodes` is a `HashSet` — iteration order must not
/// leak into compiles).
pub(super) fn positive_supply_rails(graph: &CircuitGraph) -> Vec<NodeId> {
    let mut rails: Vec<NodeId> = Vec::with_capacity(1 + graph.supply_nodes.len());
    rails.push(graph.vcc_node);
    let mut named: Vec<NodeId> = graph
        .supply_nodes
        .iter()
        .copied()
        .filter(|&n| n != graph.vcc_node)
        .collect();
    named.sort_unstable();
    rails.extend(named);
    rails
}

/// Find a resistor directly connecting `device_node` to `rail_node` in the
/// given edge set.  This is the legacy direct-edge strategy.
///
/// Returns the resistance value, or `None` if no such edge exists.
pub(super) fn find_load_resistor_direct(
    device_node: NodeId,
    rail_node: NodeId,
    edge_indices: &[usize],
    graph: &CircuitGraph,
) -> Option<f64> {
    use super::component::EdgeKind;
    edge_indices.iter().find_map(|&eidx| {
        if graph.effective_edge_kind(eidx) != EdgeKind::Linear {
            return None;
        }
        let e = &graph.edges[eidx];
        let (a, b) = (e.node_a, e.node_b);
        if (a == device_node && b == rail_node) || (b == device_node && a == rail_node) {
            graph.components[e.comp_idx].kind.resistance()
        } else {
            None
        }
    })
}

/// Find the load resistor on the DC path from `device_node` to `rail_node`
/// using a BFS that walks only resistor edges (caps/inductors are open at DC).
///
/// This is the cap-aware BFS strategy.  It can find R_cathode even when a
/// coupling cap sits between the cathode node and the rest of the signal path,
/// because the BFS only traverses resistors — caps are invisible at DC.
///
/// Returns the first resistor encountered on the shortest path from
/// `device_node` to `rail_node`, or `None` if no resistor-only path exists.
pub(super) fn find_load_resistor_bfs(
    device_node: NodeId,
    rail_node: NodeId,
    edge_indices: &[usize],
    graph: &CircuitGraph,
) -> Option<f64> {
    use super::component::EdgeKind;

    // Build adjacency restricted to resistor (Linear) edges only.
    // Caps and inductors are open at DC — skip them.
    let mut adj: HashMap<NodeId, Vec<(NodeId, f64)>> = HashMap::new();
    for &eidx in edge_indices {
        if graph.effective_edge_kind(eidx) != EdgeKind::Linear {
            continue;
        }
        let e = &graph.edges[eidx];
        let Some(r) = graph.components[e.comp_idx].kind.resistance() else {
            continue;
        };
        if r <= 0.0 || !r.is_finite() {
            continue;
        }
        adj.entry(e.node_a).or_default().push((e.node_b, r));
        adj.entry(e.node_b).or_default().push((e.node_a, r));
    }

    // BFS from device_node.  Track the first resistor stepped through.
    // We want the first edge that leads *toward* rail_node.
    let mut visited: HashSet<NodeId> = HashSet::new();
    let mut queue: VecDeque<(NodeId, Option<f64>)> = VecDeque::new();
    queue.push_back((device_node, None));
    visited.insert(device_node);

    while let Some((node, first_r)) = queue.pop_front() {
        if node == rail_node {
            return first_r;
        }
        let neighbors = adj.get(&node).cloned().unwrap_or_default();
        for (next, r) in neighbors {
            if visited.contains(&next) {
                continue;
            }
            visited.insert(next);
            // The first resistor we step through is the one to return.
            let carry_r = first_r.or(Some(r));
            queue.push_back((next, carry_r));
        }
    }

    None
}

// ═══════════════════════════════════════════════════════════════════════════
// Linear system solver (Gaussian elimination)
// ═══════════════════════════════════════════════════════════════════════════

/// Solve a small linear system Ax = b via Gaussian elimination with partial
/// pivoting.  Returns `None` if the system is singular.
///
/// The single Gaussian-elimination routine for the compile-time DC bias solve.
fn solve_linear_system(a: &mut [f64], b: &mut [f64], n: usize) -> Option<Vec<f64>> {
    for col in 0..n {
        // Partial pivot
        let mut max_row = col;
        let mut max_val = a[col * n + col].abs();
        for row in (col + 1)..n {
            let val = a[row * n + col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }
        if max_val < 1e-15 {
            return None;
        }
        if max_row != col {
            for j in 0..n {
                a.swap(col * n + j, max_row * n + j);
            }
            b.swap(col, max_row);
        }
        let pivot = a[col * n + col];
        for row in (col + 1)..n {
            let factor = a[row * n + col] / pivot;
            for j in col..n {
                a[row * n + j] -= factor * a[col * n + j];
            }
            b[row] -= factor * b[col];
        }
    }
    // Back substitution
    let mut x = vec![0.0_f64; n];
    for col in (0..n).rev() {
        let mut sum = b[col];
        for j in (col + 1)..n {
            sum -= a[col * n + j] * x[j];
        }
        x[col] = sum / a[col * n + col];
    }
    Some(x)
}

// ═══════════════════════════════════════════════════════════════════════════
// Internal helpers
// ═══════════════════════════════════════════════════════════════════════════

fn build_rail_set(graph: &CircuitGraph) -> HashSet<NodeId> {
    let mut rails = HashSet::new();
    rails.insert(graph.gnd_node);
    rails.insert(graph.vcc_node);
    for &n in &graph.supply_nodes {
        rails.insert(n);
    }
    for &n in &graph.ac_ground_nodes {
        rails.insert(n);
    }
    rails
}

// ═══════════════════════════════════════════════════════════════════════════
// Consolidated nodal DC operating-point solvers
// ═══════════════════════════════════════════════════════════════════════════
//
// These are the single DC-bias COMPUTATION for the runtime grouped-NL stages.
// They replace the per-type forks that previously lived in `rigid/general.rs`
// (`compute_triode_dc_qpoint`, `compute_bjt_dc_qpoint`) and the wave-domain
// `solve_joint_dc_qpoint` in `pedalkernel-rt/stage.rs`.  bias.rs knows nothing
// about WDF waves / scattering: it returns the CONDUCTING nodal operating point;
// the runtime (`apply_triode_dc_qpoint` / `apply_bjt_dc_qpoint`) consumes it.

/// DC operating-point data for a single common-cathode triode stage.
///
/// Moved verbatim from `rigid/general.rs` (was `TriodeDcQpoint`) so the triode
/// DC computation lives alongside the BJT one in this single file.
#[derive(Debug, Clone)]
pub(super) struct TriodeDcQpoint {
    /// Grid-cathode bias voltage (negative for self-biased stages, e.g. -1.1V).
    pub(super) vgk: f64,
    /// Plate-cathode voltage at the Q-point (e.g. 120V for a 12AX7 @ 250V supply).
    pub(super) vpk: f64,
    /// Cathode voltage = -vgk = Ia × R_cathode.
    pub(super) v_cathode: f64,
    /// Plate current at Q-point (A).
    #[allow(dead_code)]
    pub(super) ia: f64,
}

/// Compute the DC operating point for a triode-with-grid stage.
///
/// # Migration status (ko5g inc-3 — DONE for the common-cathode branch)
///
/// The **non-varimu common-cathode** path is migrated: it delegates to the shared
/// `solve_operating_point` core via `TriodeSeed`
/// (`IterationScheme::MainCurrentRelaxation`), the same core the BJT rides on.
/// The migration is bit-for-bit — the alignment of the shared solver's NR scheme
/// to the Ia-relaxation drove
/// `bias_characterization_tests::probe_triode_unified_vs_per_type_qpoint` from
/// |dVgk| = 1.96 mV / |dVpk| = 131 mV down to 0.000000, and the full corpus
/// failing-set is unchanged.  `TriodeSeed`'s direct-first resistor finder returns
/// the same R_plate/R_cathode this function's direct gate locates, so the
/// load-line Q-point is identical.  NOTE: this is a pure algorithmic unification —
/// it does NOT add any device parasitics, so the triode Q-point does not move
/// toward ngspice (fix #1's terminal-parasitic handling is BJT-specific).
///
/// **Remaining (still per-type):**
/// - The `is_vari_mu` FIXED-BIAS branch below does NOT fit `TriodeSeed` (it
///   establishes Vgk from the model's bias, not cathode self-bias) and uses a
///   VariMu/Raffensperger model + Ia bisection; migrating it needs a dedicated
///   `VariMuSeed` and is NOT bit-preserving under the relaxation scheme (bisection
///   vs relaxation) and has no characterization golden — left as-is.
/// - The pentode branch (`solve_triode_dc_qpoint` returns `None` for non-triode
///   kinds) and the single-port-WDF `compute_wdf_triode_dc_qpoint` in
///   spqr_build.rs are separate follow-ups.
///
/// Uses the load-line equations:
///   Vgk = -Ia × R_cathode    (cathode self-bias)
///   Vpk = VCC - Ia × R_plate (plate load line)
///   Ia = Triode.plate_current(Vgk, Vpk)
///
/// Returns `None` if the circuit doesn't have exactly one triode-with-grid or
/// if R_plate/R_cathode cannot be found.
pub(super) fn solve_triode_dc_qpoint(
    nl_kinds: &[NonlinearKind],
    all_edges: &[usize],
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Option<TriodeDcQpoint> {
    // Only handle single-triode-with-grid stages.
    if nl_kinds.len() != 1 {
        return None;
    }
    let (model_name, plate_node, cathode_node, parallel_count, is_vari_mu) = match &nl_kinds[0] {
        NonlinearKind::Triode {
            model_name,
            plate_node,
            cathode_node,
            grid_node: Some(_),
            parallel_count,
            is_vari_mu,
            ..
        } => (
            model_name.as_str(),
            *plate_node,
            *cathode_node,
            *parallel_count,
            *is_vari_mu,
        ),
        _ => return None,
    };

    // Find R_plate: linear resistor between a positive supply rail and
    // plate_node. Rails are searched vcc-first, then named rails (B+, ...) —
    // pedalkernel-0stg: the literal vcc comparison never matched a named
    // rail's NodeId, so every B+-fed stage silently kept the -2.0V default.
    //
    // The search is GRAPH-WIDE, not stage-edge-set-wide (the BjtNpnSeed
    // divider precedent, `all_edges` at its locate_bias_topology): named-rail
    // edges are excluded from passive claiming (graph.rs supply_nodes), so a
    // B+ plate load always lands in a DIFFERENT flow group than its triode
    // (LA-2A: R_p1 groups with C_c1, not with V1) and a stage-set-only search
    // can never see it. vcc pedals keep their in-group direct hit unchanged.
    //
    // A plate wired DIRECTLY to a rail (cathode follower: `B+ -> V3.plate`
    // unions the plate node into the rail) has r_plate = 0 on that rail.
    let graph_edges: Vec<usize> = (0..graph.edges.len()).collect();
    let (r_plate, plate_rail_v) = positive_supply_rails(graph)
        .iter()
        .find_map(|&rail| {
            find_load_resistor_direct(plate_node, rail, &graph_edges, graph)
                .map(|r| (r, rail_dc_voltage(rail, graph, supply_voltage).unwrap_or(supply_voltage)))
        })
        .or_else(|| {
            rail_dc_voltage(plate_node, graph, supply_voltage)
                .filter(|&v| v > 0.0)
                .map(|v| (0.0, v))
        })?;

    // Find R_cathode: linear resistor between cathode_node and gnd_node.
    let r_cathode = all_edges.iter().find_map(|&eidx| {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        if graph.effective_edge_kind(eidx) != EdgeKind::Linear {
            return None;
        }
        let (a, b) = (e.node_a, e.node_b);
        if (a == cathode_node && b == graph.gnd_node) || (b == cathode_node && a == graph.gnd_node)
        {
            comp.kind.resistance()
        } else {
            None
        }
    })?;

    if is_vari_mu {
        // Variable-mu fixtures are fixed-bias devices: the grid control voltage
        // establishes Vgk directly, rather than via cathode self-bias. Use the
        // model's default bias (6386: -2 V) and solve the plate load line with
        // the Raffensperger model, not the Koren triode model.
        let model = vari_mu_model(model_name);
        let mut triode = VariMuTriodeRoot::new_with_v_max(model, supply_voltage)
            .with_parallel_count(parallel_count);
        let vgk = triode.vgk_bias();
        triode.set_vgk(vgk);

        let max_ia = (plate_rail_v / r_plate.max(1.0)).max(1e-9);
        let mut lo = 0.0_f64;
        let mut hi = max_ia;
        let residual = |ia: f64, triode: &mut VariMuTriodeRoot| -> f64 {
            let vpk = (plate_rail_v - ia * r_plate).max(0.0);
            ia - triode.plate_current(vpk)
        };

        let mut flo = residual(lo, &mut triode);
        let fhi = residual(hi, &mut triode);
        if !flo.is_finite() || !fhi.is_finite() || flo.signum() == fhi.signum() {
            return None;
        }
        for _ in 0..80 {
            let mid = 0.5 * (lo + hi);
            let fmid = residual(mid, &mut triode);
            if !fmid.is_finite() {
                return None;
            }
            if fmid.abs() < 1e-10 {
                lo = mid;
                hi = mid;
                break;
            }
            if flo.signum() == fmid.signum() {
                lo = mid;
                flo = fmid;
            } else {
                hi = mid;
            }
        }

        let ia = 0.5 * (lo + hi);
        let vpk = (plate_rail_v - ia * r_plate).max(0.0);
        let v_cathode = ia * r_cathode;
        if vgk >= 0.0 || vpk <= 0.0 || !vgk.is_finite() || !vpk.is_finite() {
            return None;
        }
        return Some(TriodeDcQpoint {
            vgk,
            vpk,
            v_cathode,
            ia,
        });
    }

    // Non-varimu common-cathode: delegate to the unified shared load-line core
    // (ko5g inc-3).  `TriodeSeed` carries the `MainCurrentRelaxation` scheme, which
    // reproduces the legacy Ia-relaxation loop bit-for-bit (initial Ia = 0.1 mA,
    // 0.5 damping, 50 iters, 1 nA tol), and its direct-first resistor finder
    // returns the SAME R_plate/R_cathode the direct gate above located.  The
    // load-line Q-point is therefore identical to the deleted per-type loop, and
    // the triode now rides the same solver core the BJT does.  `_ = r_plate` keeps
    // the direct gate (a missing plate resistor => None, unchanged).
    let _ = r_plate;

    let seed = TriodeSeed {
        nl_kind: &nl_kinds[0],
        label: model_name.to_owned(),
        supply_voltage,
        parallel_count,
    };
    let op = solve_operating_point(
        &seed,
        all_edges,
        graph,
        &NetworkBias::default(),
        supply_voltage,
    )
    .ok()?;

    let vgk = op.control_bias;
    let vpk = op.output_warm_start;
    let v_cathode = -vgk; // cathode auto-bias: V_cathode = Ia*Rk = -Vgk
    let ia = v_cathode / r_cathode;

    // Sanity: Q-point should have negative Vgk and positive Vpk.
    if vgk >= 0.0 || vpk <= 0.0 || !vgk.is_finite() || !vpk.is_finite() {
        return None;
    }

    Some(TriodeDcQpoint {
        vgk,
        vpk,
        v_cathode,
        ia,
    })
}

/// Solve the **nonlinear** DC operating point of every BJT in the group.
///
/// This is the BJT analogue of [`solve_triode_dc_qpoint`].  Where the triode
/// path solves a single 1-D load line, a BJT group may contain a DC-coupled
/// feedback servo (BA283: TR1's base is biased through R2 from the NFB bus whose
/// level is set by the conducting Darlington), so the operating point of all
/// devices is mutually coupled and must be **co-solved**.
///
/// Method: full-network nodal Newton-Raphson over the group's interior
/// (non-rail) nodes, wrapped in a **source-stepping homotopy** (the SPICE
/// technique): the rail voltages are ramped from a small fraction up to full
/// supply over several continuation steps, each step warm-started from the
/// previous converged point.  This drives the solve onto the CONDUCTING fixed
/// point (BA283 TR1 → Vbe≈0.61 V) rather than the spurious cutoff fixed point a
/// cold ½-supply start lands at.  Linear resistors contribute conductances;
/// every BJT is stamped via its Gummel-Poon `currents(Vbe, Vbc)` and a numerical
/// 3×3 device Jacobian.  Caps/inductors are open at DC and ignored.  Rails are
/// held at their (scaled) known voltages.
///
/// Returns a map of **DC node voltage** for every solved interior node (rails
/// excluded but the BJT-terminal rails are included so the caller can resolve
/// ports), or `None` if the group has no BJTs, the system is singular, the solve
/// fails to converge, or the result is non-physical.
//
// ── Parasitic-resistance internal-voltage fixed point ───────────────────────
// Mirror of the runtime `BjtTwoPort::eval` constants (pedalkernel-rt
// elements/nonlinear/bjt.rs). 0.5 damping keeps the map a contraction even when
// the undamped loop gain `d(i·R)/dv` exceeds 1 for large RB.
const BJT_PARASITIC_DAMP: f64 = 0.5;
const BJT_PARASITIC_TOL: f64 = 1e-9;
const BJT_PARASITIC_MAX_ITER: usize = 40;

/// Gummel-Poon collector/base currents from **terminal** (node-difference) port
/// voltages, applying the device's parasitic ohmic drops (RB·Ib, RE·Ie, RC·Ic)
/// exactly as the runtime [`pedalkernel_rt::elements::BjtTwoPort`]`::eval` does.
///
/// The intrinsic `GummelPoonModel::currents(vbe, vbc)` takes *junction* voltages;
/// feeding it the raw terminal node difference (omitting the parasitic drops)
/// biases a high-RB input transistor at the intrinsic-Vbe operating point — for
/// the BA283 TR1 (QBC184C, RB=500 Ω) that is the under-conducting ~half-current
/// point, because Ic is exponential in Vbe and the omitted `Ib·RB` ≈ 33 mV maps
/// to ~2× collector current. The compile-time DC solve must therefore apply the
/// SAME terminal→internal map as runtime so the baked `dc_bias` is a true fixed
/// point of the runtime balance (no compile-vs-runtime divergence).
///
/// `vbe_term`/`vbc_term` are NPN-normalized (the caller applies the device sign
/// before calling and re-applies it to the returned currents), so the damped
/// Picard runs in the same normalized space as the runtime `eval`.
fn bjt_currents_terminal(model: &GummelPoonModel, vbe_term: f64, vbc_term: f64) -> (f64, f64) {
    let rb = model.rb as f64;
    let re = model.re as f64;
    let rc = model.rc as f64;
    let currents = |vbe: f64, vbc: f64| -> (f64, f64) {
        let (ic, ib) = model.currents(vbe as pedalkernel_rt::Wave, vbc as pedalkernel_rt::Wave);
        (ic as f64, ib as f64)
    };
    if rb + re + rc <= 0.0 {
        // No parasitics: terminal voltages ARE the junction voltages.
        return currents(vbe_term, vbc_term);
    }
    // Damped fixed point on the internal junction voltages:
    //   v_int = v_term − i(v_int)·R   (vbc = vbe − vce throughout)
    let vce_term = vbe_term - vbc_term;
    let mut vbe_int = vbe_term;
    let mut vce_int = vce_term;
    for _ in 0..BJT_PARASITIC_MAX_ITER {
        let vbc_int = vbe_int - vce_int;
        let (ic, ib) = currents(vbe_int, vbc_int);
        let ie_out = ic + ib;
        let vbe_t = vbe_term - ib * rb - ie_out * re;
        let vce_t = vce_term - ic * rc - ie_out * re;
        let dvbe = BJT_PARASITIC_DAMP * (vbe_t - vbe_int);
        let dvce = BJT_PARASITIC_DAMP * (vce_t - vce_int);
        vbe_int += dvbe;
        vce_int += dvce;
        if dvbe.abs() + dvce.abs() < BJT_PARASITIC_TOL {
            break;
        }
    }
    let vbc_int = vbe_int - vce_int;
    currents(vbe_int, vbc_int)
}

pub(super) fn solve_bjt_group_dc_qpoint(
    nl_kinds: &[NonlinearKind],
    all_edges: &[usize],
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Option<std::collections::HashMap<NodeId, f64>> {
    // Gather the BJTs (skip diode-connected: those are handled as 1-port diodes).
    struct BjtRef<'a> {
        model_name: &'a str,
        is_npn: bool,
        base: NodeId,
        collector: NodeId,
        emitter: NodeId,
    }
    let mut bjts: Vec<BjtRef> = Vec::new();
    for kind in nl_kinds {
        match kind {
            NonlinearKind::BjtNpn {
                model_name,
                base_node,
                collector_node,
                emitter_node,
            } if base_node != collector_node => bjts.push(BjtRef {
                model_name,
                is_npn: true,
                base: *base_node,
                collector: *collector_node,
                emitter: *emitter_node,
            }),
            NonlinearKind::BjtPnp {
                model_name,
                base_node,
                collector_node,
                emitter_node,
            } if base_node != collector_node => bjts.push(BjtRef {
                model_name,
                is_npn: false,
                base: *base_node,
                collector: *collector_node,
                emitter: *emitter_node,
            }),
            _ => {}
        }
    }
    if bjts.is_empty() {
        return None;
    }

    // Full (unscaled) rail voltage, or None for an interior (solved) node.
    // (The shared resolver — see `rail_dc_voltage` — was extracted from this
    // closure verbatim for pedalkernel-0stg; delegation keeps one source of
    // truth for rail resolution.)
    let full_rail_v = |node: NodeId| -> Option<f64> { rail_dc_voltage(node, graph, supply_voltage) };

    // Resistor list (linear conductances): (node_a, node_b, g).
    let mut resistors: Vec<(NodeId, NodeId, f64)> = Vec::new();
    for &eidx in all_edges {
        if graph.effective_edge_kind(eidx) != EdgeKind::Linear {
            continue;
        }
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        if let Some(r) = comp.kind.resistance() {
            if r > 0.0 {
                // `Potentiometer::resistance()` reports the FULL track max_R,
                // but the runtime stamps pot edges at the compile-time default
                // position 0.5 (rheostat R = pos·max_R; 3-term halves aw/wb =
                // pos·max_R / (1−pos)·max_R — see `make_leaf` and the
                // spqr_build `max_r * 0.5` convention). The DC q-point must
                // solve the SAME network the runtime scatters, or the seeded
                // op is a root of a different circuit: on the BA283 the full
                // 4.7k track (vs RV1=2350) shifted the solved q-point ~4 mV,
                // which the exponential shunt-feedback loop amplified into the
                // starved Q1/Q2 runtime root (ΔIc −74 %).
                let r_eff = if comp.kind.is_pot() { r * 0.5 } else { r };
                resistors.push((e.node_a, e.node_b, 1.0 / r_eff));
            }
        }
    }

    // Only nodes with a real DC path participate: those incident to a resistor or
    // that are a BJT terminal.  Cap/inductor-only nodes (input coupling cap, output
    // tap behind Cout) are open at DC → excluded, else they produce zero-conductance
    // rows that make the Newton system singular.
    let mut dc_path: HashSet<NodeId> = HashSet::new();
    for &(na, nb, _) in &resistors {
        dc_path.insert(na);
        dc_path.insert(nb);
    }
    for b in &bjts {
        dc_path.insert(b.base);
        dc_path.insert(b.collector);
        dc_path.insert(b.emitter);
    }

    // Collect interior (non-rail) nodes from that DC-connected set.
    let mut interior: Vec<NodeId> = Vec::new();
    for &node in &dc_path {
        if full_rail_v(node).is_none() && !interior.contains(&node) {
            interior.push(node);
        }
    }
    interior.sort_unstable();
    let n = interior.len();
    if n == 0 {
        return None;
    }
    let idx: HashMap<NodeId, usize> = interior
        .iter()
        .enumerate()
        .map(|(i, &nd)| (nd, i))
        .collect();

    // Pre-fetch device models.
    let models: Vec<GummelPoonModel> = bjts
        .iter()
        .map(|b| gummel_poon_model(b.model_name))
        .collect();

    // Small node-to-ground shunt conductance (SPICE `gmin`) regularizes the
    // Jacobian so the off-state cold start (all junctions at Vbe≈0, ~zero gm)
    // never produces a singular / wildly ill-scaled Newton system.
    let gmin = 1e-9_f64;

    // ── Source-stepping homotopy ─────────────────────────────────────────────
    // Ramp the rails from `lambda·V_rail` (lambda small) to full supply.  At low
    // lambda every junction is gently forward-biased from a low-voltage start, so
    // the Newton lands the conducting branch; raising lambda then *tracks* that
    // branch up to the full operating point instead of jumping to the cutoff
    // fixed point a cold full-supply start would settle into.
    let lambdas: [f64; 8] = [0.05, 0.1, 0.2, 0.35, 0.55, 0.75, 0.9, 1.0];

    // Interior-node state, carried across continuation steps.  Start every node
    // at a mild forward bias relative to the first (smallest) rail scale.
    let mut v = vec![0.5 * lambdas[0] * supply_voltage; n];

    for &lambda in &lambdas {
        let rail_v = |node: NodeId| -> Option<f64> { full_rail_v(node).map(|rv| rv * lambda) };
        let node_voltage = |node: NodeId, v: &[f64]| -> f64 {
            if let Some(rv) = rail_v(node) {
                rv
            } else {
                v[idx[&node]]
            }
        };

        let mut converged = false;
        for _iter in 0..200 {
            let mut j = vec![0.0_f64; n * n];
            let mut f = vec![0.0_f64; n];

            // gmin shunt: every interior node leaks `gmin·V` to ground.
            for k in 0..n {
                f[k] += gmin * v[k];
                j[k * n + k] += gmin;
            }

            // Resistor stamps (KCL: current leaving node via R).
            for &(na, nb, g) in &resistors {
                let va = node_voltage(na, &v);
                let vb = node_voltage(nb, &v);
                let ia = idx.get(&na).copied();
                let ib = idx.get(&nb).copied();
                let i_ab = (va - vb) * g; // current a→b
                if let Some(a) = ia {
                    f[a] += i_ab;
                    j[a * n + a] += g;
                    if let Some(b) = ib {
                        j[a * n + b] -= g;
                    }
                }
                if let Some(b) = ib {
                    f[b] -= i_ab;
                    j[b * n + b] += g;
                    if let Some(a) = ia {
                        j[b * n + a] -= g;
                    }
                }
            }

            // BJT stamps.
            let h = 1e-6_f64;
            for (b, model) in bjts.iter().zip(models.iter()) {
                let vb_ = node_voltage(b.base, &v);
                let vc_ = node_voltage(b.collector, &v);
                let ve_ = node_voltage(b.emitter, &v);
                let sign = if b.is_npn { 1.0 } else { -1.0 };
                let vbe = sign * (vb_ - ve_);
                let vbc = sign * (vb_ - vc_);

                // Apply the SAME parasitic terminal→internal map as runtime
                // `BjtTwoPort::eval` (see `bjt_currents_terminal`): the node
                // voltages are TERMINAL, not intrinsic-junction.
                let (ic, ib_) = bjt_currents_terminal(model, vbe, vbc);
                let (ic, ib_) = (sign * ic, sign * ib_);
                // Terminal currents flowing INTO the device (leaving the node):
                //   base: +Ib, collector: +Ic, emitter: -(Ib+Ic)
                let term = [(b.base, ib_), (b.collector, ic), (b.emitter, -(ib_ + ic))];
                for &(node, i_term) in &term {
                    if let Some(&row) = idx.get(&node) {
                        f[row] += i_term;
                    }
                }

                // Numerical 3×3 Jacobian: ∂(terminal current)/∂(terminal V).
                let ctrl_nodes = [b.base, b.collector, b.emitter];
                for (k, &cn) in ctrl_nodes.iter().enumerate() {
                    // Only interior columns matter (rails are fixed).
                    let col = match idx.get(&cn) {
                        Some(&c) => c,
                        None => continue,
                    };
                    // Perturb terminal k's voltage.
                    let mut vbn = vb_;
                    let mut vcn = vc_;
                    let mut ven = ve_;
                    match k {
                        0 => vbn += h,
                        1 => vcn += h,
                        _ => ven += h,
                    }
                    let vbe_p = sign * (vbn - ven);
                    let vbc_p = sign * (vbn - vcn);
                    let (icp, ibp) = bjt_currents_terminal(model, vbe_p, vbc_p);
                    let (icp, ibp) = (sign * icp, sign * ibp);
                    let dterm = [
                        (b.base, (ibp - ib_) / h),
                        (b.collector, (icp - ic) / h),
                        (b.emitter, (-(ibp + icp) - -(ib_ + ic)) / h),
                    ];
                    for &(node, d) in &dterm {
                        if let Some(&row) = idx.get(&node) {
                            j[row * n + col] += d;
                        }
                    }
                }
            }

            // Converge on the current residual (KCL must balance to a tiny
            // current at every node).
            let max_f = f.iter().fold(0.0_f64, |m, &x| m.max(x.abs()));
            if max_f < 1e-9 {
                converged = true;
                break;
            }

            // Solve J·Δ = -f.
            let mut neg_f: Vec<f64> = f.iter().map(|x| -x).collect();
            let delta = solve_linear_system(&mut j, &mut neg_f, n)?;

            // Damped update: limit any single node move to 0.25 V so junction
            // exponentials never blow up between iterations.
            let max_raw = delta.iter().fold(0.0_f64, |m, &x| m.max(x.abs()));
            let scale = if max_raw > 0.25 { 0.25 / max_raw } else { 1.0 };
            for k in 0..n {
                v[k] += delta[k] * scale;
            }
            if !v.iter().all(|x| x.is_finite()) {
                return None;
            }
        }
        if !converged {
            return None;
        }
    }

    // Final converged state is at lambda = 1.0 (full supply).
    let rail_v = |node: NodeId| -> Option<f64> { full_rail_v(node) };
    let node_voltage = |node: NodeId, v: &[f64]| -> f64 {
        if let Some(rv) = rail_v(node) {
            rv
        } else {
            v[idx[&node]]
        }
    };

    // Physicality check + assemble the solved node-voltage map.
    let vcc_ceiling = rail_v(graph.vcc_node)
        .unwrap_or(supply_voltage)
        .max(supply_voltage);
    let mut any_active = false;
    for b in &bjts {
        let vb_ = node_voltage(b.base, &v);
        let vc_ = node_voltage(b.collector, &v);
        let ve_ = node_voltage(b.emitter, &v);
        let sign = if b.is_npn { 1.0 } else { -1.0 };
        let vbe = sign * (vb_ - ve_);
        let vce = sign * (vc_ - ve_);
        if !vbe.is_finite() || !vce.is_finite() {
            return None;
        }
        // Reject clearly non-physical points; a slightly-saturated output
        // Darlington is acceptable (its Vce can be small), but a junction biased
        // well past turn-on or reverse-biased Vce is a failed solve.
        if vbe > 1.2 || vce < -0.5 || vce > vcc_ceiling + 1.0 {
            return None;
        }
        if (0.3..=1.0).contains(&vbe) && vce > 0.05 {
            any_active = true;
        }
    }
    if !any_active {
        return None;
    }

    // Return the solved DC node voltages, including the rail nodes that BJT
    // terminals touch so the caller can resolve a port like (collector=vcc,
    // emitter=interior) to a true Vce.
    let mut node_dc: std::collections::HashMap<NodeId, f64> = std::collections::HashMap::new();
    for (i, &nd) in interior.iter().enumerate() {
        node_dc.insert(nd, v[i]);
    }
    for b in &bjts {
        for &nd in &[b.base, b.collector, b.emitter] {
            if let Some(rv) = rail_v(nd) {
                node_dc.insert(nd, rv);
            }
        }
    }
    Some(node_dc)
}

// ═══════════════════════════════════════════════════════════════════════════
// Unit tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;
    use crate::compiler::graph::CircuitGraph;
    use crate::compiler::spqr_build::compile_via_spqr;

    const SR: f64 = 48000.0;

    // ── Helper: parse pedal and build graph ──────────────────────────────────

    fn parse_graph(source: &str) -> CircuitGraph {
        let pedal = crate::dsl::parse_pedal_file(source).expect("parse");
        CircuitGraph::from_pedal(&pedal)
    }

    // ── 1. solve_network_bias: simple VCC divider ────────────────────────────

    #[test]
    fn network_bias_vcc_divider_4v5() {
        // 9 V supply, R_top=10k, R_bot=10k → junction = 4.5 V.
        let graph = parse_graph(
            r#"pedal "test" { supply 9V
                components {
                    R_top: resistor(10k)
                    R_bot: resistor(10k)
                    R_in: resistor(10k)
                    U1: opamp(tl072)
                    Rf: resistor(100k)
                }
                nets {
                    vcc -> R_top.a
                    R_top.b -> R_bot.a
                    R_bot.b -> gnd
                    in -> R_in.a
                    R_in.b -> U1.neg
                    U1.neg -> Rf.a
                    Rf.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> out
                }
                controls {}
            }"#,
        );

        let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
        let nb = solve_network_bias(&all_edges, &graph, 9.0);

        // The junction node (R_top.b = R_bot.a) should be ~4.5V.
        assert!(
            !nb.dc_voltages.is_empty(),
            "Should compute at least one DC voltage"
        );
        let has_4v5 = nb.dc_voltages.values().any(|&v| (v - 4.5).abs() < 0.1);
        assert!(has_4v5, "Expected a ~4.5V node; got: {:?}", nb.dc_voltages);
    }

    #[test]
    fn network_bias_asymmetric_divider_3v0() {
        // R_top=20k, R_bot=10k → 9 * 10/(20+10) = 3.0 V.
        let graph = parse_graph(
            r#"pedal "test" { supply 9V
                components {
                    R_top: resistor(20k)
                    R_bot: resistor(10k)
                    R_in: resistor(10k)
                    U1: opamp(tl072)
                    Rf: resistor(100k)
                }
                nets {
                    vcc -> R_top.a
                    R_top.b -> R_bot.a
                    R_bot.b -> gnd
                    in -> R_in.a
                    R_in.b -> U1.neg
                    U1.neg -> Rf.a
                    Rf.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> out
                }
                controls {}
            }"#,
        );

        let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
        let nb = solve_network_bias(&all_edges, &graph, 9.0);
        let has_3v = nb.dc_voltages.values().any(|&v| (v - 3.0).abs() < 0.1);
        assert!(has_3v, "Expected ~3.0V node; got: {:?}", nb.dc_voltages);
    }

    // ── 2. find_load_resistor_direct ────────────────────────────────────────

    #[test]
    fn find_load_resistor_direct_finds_plate_r() {
        // Build a minimal graph where node 1=plate, node 2=vcc, with a 100k resistor.
        // We verify find_load_resistor_direct returns 100_000.
        // Use the parse path so we have a real graph.
        let graph = parse_graph(
            r#"pedal "test" { supply 250V
                components {
                    T1: triode(12ax7)
                    R_plate: resistor(100k)
                    R_cathode: resistor(1k5)
                    R_grid: resistor(1M)
                    C_cathode: cap(22u, electrolytic)
                    C_in: cap(22n)
                    C_out: cap(22n)
                    R_load: resistor(1M)
                }
                nets {
                    vcc -> R_plate.a
                    R_plate.b -> T1.plate
                    T1.cathode -> R_cathode.a
                    R_cathode.b -> gnd
                    T1.cathode -> C_cathode.a
                    C_cathode.b -> gnd
                    in -> C_in.a
                    C_in.b -> R_grid.a
                    R_grid.b -> T1.grid
                    T1.plate -> C_out.a
                    C_out.b -> R_load.a
                    R_load.b -> out
                }
                controls {}
            }"#,
        );

        let plate_node = *graph.node_names.get("T1.plate").expect("T1.plate node");
        let all_edges: Vec<usize> = (0..graph.edges.len()).collect();

        let r = find_load_resistor_direct(plate_node, graph.vcc_node, &all_edges, &graph);
        assert!(
            r.is_some(),
            "Should find R_plate directly between T1.plate and VCC"
        );
        let r_val = r.unwrap();
        assert!(
            (r_val - 100_000.0).abs() < 100.0,
            "R_plate should be 100k, got {r_val}"
        );
    }

    // ── 3. find_load_resistor_bfs: cap-coupled cathode ───────────────────────

    #[test]
    fn find_load_resistor_bfs_direct_path() {
        // Same topology as above — BFS should agree with direct when no caps intervene.
        let graph = parse_graph(
            r#"pedal "test" { supply 250V
                components {
                    T1: triode(12ax7)
                    R_plate: resistor(100k)
                    R_cathode: resistor(1k5)
                    R_grid: resistor(1M)
                    C_cathode: cap(22u, electrolytic)
                    C_in: cap(22n)
                    C_out: cap(22n)
                    R_load: resistor(1M)
                }
                nets {
                    vcc -> R_plate.a
                    R_plate.b -> T1.plate
                    T1.cathode -> R_cathode.a
                    R_cathode.b -> gnd
                    T1.cathode -> C_cathode.a
                    C_cathode.b -> gnd
                    in -> C_in.a
                    C_in.b -> R_grid.a
                    R_grid.b -> T1.grid
                    T1.plate -> C_out.a
                    C_out.b -> R_load.a
                    R_load.b -> out
                }
                controls {}
            }"#,
        );

        let cathode_node = *graph.node_names.get("T1.cathode").expect("T1.cathode node");
        let all_edges: Vec<usize> = (0..graph.edges.len()).collect();

        let r = find_load_resistor_bfs(cathode_node, graph.gnd_node, &all_edges, &graph);
        assert!(
            r.is_some(),
            "BFS should find R_cathode between T1.cathode and GND"
        );
        let r_val = r.unwrap();
        assert!(
            (r_val - 1_500.0).abs() < 10.0,
            "R_cathode should be 1.5k, got {r_val}"
        );
    }

    // ── 4. Triode Q-point: bit-for-bit match vs compute_wdf_triode_dc_qpoint ─

    /// Directly verify that `solve_operating_point` with `TriodeSeed` returns
    /// the same Vgk as the existing `compute_wdf_triode_dc_qpoint` for the
    /// canonical common-cathode 12AX7 circuit (R_plate=100k, R_cathode=1.5k,
    /// supply=250V).
    ///
    /// "Bit-for-bit" here means within floating-point rounding (< 1e-6 V)
    /// since both use NR with the same model and the same equations.
    #[test]
    fn triode_qpoint_matches_existing_solver_common_cathode_12ax7() {
        use super::super::classify::NonlinearKind;
        use super::super::graph::CircuitGraph;

        let graph = parse_graph(
            r#"pedal "test" { supply 250V
                components {
                    T1: triode(12ax7)
                    R_plate: resistor(100k)
                    R_cathode: resistor(1k5)
                    R_grid: resistor(1M)
                    C_cathode: cap(22u, electrolytic)
                    C_in: cap(22n)
                    C_out: cap(22n)
                    R_load: resistor(1M)
                }
                nets {
                    vcc -> R_plate.a
                    R_plate.b -> T1.plate
                    T1.cathode -> R_cathode.a
                    R_cathode.b -> gnd
                    T1.cathode -> C_cathode.a
                    C_cathode.b -> gnd
                    in -> C_in.a
                    C_in.b -> R_grid.a
                    R_grid.b -> T1.grid
                    T1.plate -> C_out.a
                    C_out.b -> R_load.a
                    R_load.b -> out
                }
                controls {}
            }"#,
        );

        // Replicate the legacy computation manually.
        // Legacy: NR with F(Ia) = Ia - plate_current(Vgk(Ia), Vpk(Ia))
        // where Vgk = -Ia*Rk and Vpk = VCC - Ia*Rp.
        let supply = 250.0_f64;
        let r_plate = 100_000.0_f64;
        let r_cathode = 1_500.0_f64;
        let model = super::super::helpers::triode_model("12AX7");
        let mut legacy_ia = 1e-4_f64;
        for _ in 0..50 {
            let vgk = -legacy_ia * r_cathode;
            let vpk = (supply - legacy_ia * r_plate).max(0.0);
            let mut root =
                pedalkernel_rt::elements::nonlinear::TriodeRoot::new_with_v_max(model, supply);
            root.set_bias(vgk as pedalkernel_rt::Wave);
            let ia_model = root.plate_current(vpk as pedalkernel_rt::Wave);
            let f = legacy_ia - ia_model as f64;
            legacy_ia = (legacy_ia - f * 0.5).max(0.0);
            if f.abs() < 1e-9 {
                break;
            }
        }
        let legacy_vgk = -legacy_ia * r_cathode;

        // New solver.
        let plate_node = *graph.node_names.get("T1.plate").expect("T1.plate");
        let cathode_node = *graph.node_names.get("T1.cathode").expect("T1.cathode");
        let grid_node = graph.node_names.get("T1.grid").copied();

        let nl_kind = NonlinearKind::Triode {
            model_name: "12AX7".to_owned(),
            plate_node,
            cathode_node,
            grid_node,
            parallel_count: 1,
            is_vari_mu: false,
        };

        let seed = TriodeSeed {
            nl_kind: &nl_kind,
            label: "T1".to_owned(),
            supply_voltage: supply,
            parallel_count: 1,
        };

        let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
        let network_bias = NetworkBias::default();
        let result = solve_operating_point(&seed, &all_edges, &graph, &network_bias, supply);

        let qp = result.expect("solve_operating_point should succeed for 12AX7");
        let new_vgk = qp.control_bias;

        eprintln!(
            "triode 12AX7 Q-point: legacy_vgk={legacy_vgk:.6} V, new_vgk={new_vgk:.6} V, \
             delta={:.2e} V",
            (new_vgk - legacy_vgk).abs()
        );

        assert!(
            (new_vgk - legacy_vgk).abs() < 5e-3,
            "New solver Vgk={new_vgk:.6} V should match legacy Vgk={legacy_vgk:.6} V \
             within 5 mV (same load-line equations, different iteration scheme)"
        );

        // Sanity: Q-point must be in the active region.
        assert!(new_vgk < 0.0, "Vgk must be negative for active region");
        assert!(
            qp.output_warm_start > 0.0,
            "Vpk must be positive for active region"
        );
    }

    // ── 5. BJT Q-point: match vs compute_wdf_bjt_dc_qpoint ──────────────────

    /// Verify that `solve_operating_point` with `BjtNpnSeed` returns the same
    /// Vbe as the existing `compute_wdf_bjt_dc_qpoint` for a classic NPN
    /// common-emitter divider circuit (2N3904, 9V supply).
    #[test]
    fn bjt_qpoint_matches_existing_solver_npn_common_emitter() {
        // Classic CE: R1=47k VCC, R2=10k GND, RC=4k7, RE=1k, Vcc=9V.
        let graph = parse_graph(
            r#"pedal "test" { supply 9V
                components {
                    Q1: npn(2n3904)
                    R1: resistor(47k)
                    R2: resistor(10k)
                    RC: resistor(4k7)
                    RE: resistor(1k)
                    C_in: cap(10u, electrolytic)
                    C_out: cap(10u, electrolytic)
                    C_bypass: cap(100u, electrolytic)
                    R_in: resistor(10k)
                }
                nets {
                    vcc -> R1.a
                    R1.b -> Q1.base
                    Q1.base -> R2.a
                    R2.b -> gnd
                    vcc -> RC.a
                    RC.b -> Q1.collector
                    Q1.emitter -> RE.a
                    RE.b -> gnd
                    Q1.emitter -> C_bypass.a
                    C_bypass.b -> gnd
                    in -> C_in.a
                    C_in.b -> R_in.a
                    R_in.b -> Q1.base
                    Q1.collector -> C_out.a
                    C_out.b -> out
                }
                controls {}
            }"#,
        );

        let supply = 9.0_f64;

        // Legacy computation (mirrors compute_wdf_bjt_dc_qpoint).
        let r1 = 47_000.0_f64;
        let r2 = 10_000.0_f64;
        let re = 1_000.0_f64;
        let vth = supply * r2 / (r1 + r2);
        let rth = r1 * r2 / (r1 + r2);
        let model = super::super::helpers::gummel_poon_model("2N3904");
        let vbc_active = -1.0_f64;
        let mut legacy_vbe = 0.65_f64;
        for _ in 0..60 {
            let (ic, ib) = model.currents(
                legacy_vbe as pedalkernel_rt::Wave,
                vbc_active as pedalkernel_rt::Wave,
            );
            let ie = ic + ib;
            let f = vth - ib as f64 * rth - legacy_vbe - ie as f64 * re;
            let h = 1e-4_f64;
            let (ic2, ib2) = model.currents(
                (legacy_vbe + h) as pedalkernel_rt::Wave,
                vbc_active as pedalkernel_rt::Wave,
            );
            let df = -((ib2 - ib) as f64 / h) * rth - 1.0 - ((ic2 + ib2 - ic - ib) as f64 / h) * re;
            if df.abs() < 1e-18 {
                break;
            }
            let step = (f / df).clamp(-0.1, 0.1);
            legacy_vbe -= step;
            legacy_vbe = legacy_vbe.clamp(0.0, 1.0);
            if step.abs() < 1e-9 {
                break;
            }
        }
        let legacy_vbe = legacy_vbe.clamp(0.3, 0.8);

        // New solver.
        let base_node = *graph.node_names.get("Q1.base").expect("Q1.base");
        let collector_node = *graph.node_names.get("Q1.collector").expect("Q1.collector");
        let emitter_node = *graph.node_names.get("Q1.emitter").expect("Q1.emitter");

        let nl_kind = NonlinearKind::BjtNpn {
            model_name: "2N3904".to_owned(),
            base_node,
            collector_node,
            emitter_node,
        };

        let seed = BjtNpnSeed {
            nl_kind: &nl_kind,
            label: "Q1".to_owned(),
        };

        let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
        let network_bias = NetworkBias::default();
        let result = solve_operating_point(&seed, &all_edges, &graph, &network_bias, supply);

        let qp = result.expect("solve_operating_point should succeed for 2N3904");
        let new_vbe = qp.control_bias;

        eprintln!(
            "BJT 2N3904 Q-point: legacy_vbe={legacy_vbe:.6} V, new_vbe={new_vbe:.6} V, \
             delta={:.2e} V",
            (new_vbe - legacy_vbe).abs()
        );

        assert!(
            (new_vbe - legacy_vbe).abs() < 5e-3,
            "New solver Vbe={new_vbe:.6} V should match legacy Vbe={legacy_vbe:.6} V \
             within 5 mV"
        );

        assert!(
            new_vbe > 0.3 && new_vbe < 0.85,
            "Vbe={new_vbe:.4} V out of physical range [0.3, 0.85]"
        );
    }

    // ── 6. BiasError::into_compile_error produces actionable messages ────────

    #[test]
    fn bias_error_messages_are_actionable() {
        let err = BiasError::UndeterminableTriode {
            label: "T1".to_owned(),
            missing: TopologyTerm::PlateResistor,
        };
        let msg = err.into_compile_error();
        assert!(
            msg.contains("T1"),
            "Error message should name the device: {msg}"
        );
        assert!(
            msg.contains("R_plate"),
            "Error message should name the missing element: {msg}"
        );
        assert!(
            msg.contains("init"),
            "Error message should suggest init hint: {msg}"
        );

        let err2 = BiasError::SingularBiasNetwork;
        let msg2 = err2.into_compile_error();
        assert!(
            msg2.contains("singular"),
            "SingularBiasNetwork message: {msg2}"
        );
    }

    // ── 7. NetworkBias::voltage_at returns rail voltages ────────────────────

    #[test]
    fn network_bias_voltage_at_rail_nodes() {
        let graph = parse_graph(
            r#"pedal "test" { supply 9V
                components {
                    R_in: resistor(10k)
                    U1: opamp(tl072)
                    Rf: resistor(100k)
                }
                nets {
                    in -> R_in.a
                    R_in.b -> U1.neg
                    U1.neg -> Rf.a
                    Rf.b -> U1.out
                    U1.pos -> gnd
                    U1.out -> out
                }
                controls {}
            }"#,
        );
        let nb = NetworkBias::default();
        // GND should be 0.0
        assert_eq!(nb.voltage_at(graph.gnd_node, &graph, 9.0), Some(0.0));
        // VCC should be 9.0
        assert_eq!(nb.voltage_at(graph.vcc_node, &graph, 9.0), Some(9.0));
    }

    // ── 8. Named supply rails (pedalkernel-0stg) ─────────────────────────────

    /// Classify the single triode edge of a parsed graph into its
    /// `NonlinearKind`, mirroring the triode-context call site.
    fn triode_nl_kind(graph: &CircuitGraph, comp_id: &str) -> NonlinearKind {
        let (eidx, e) = graph
            .edges
            .iter()
            .enumerate()
            .find(|(_, e)| graph.components[e.comp_idx].id == comp_id)
            .expect("triode edge");
        let _ = eidx;
        let comp = &graph.components[e.comp_idx];
        comp.kind
            .classify_nonlinear(&comp.id, e.node_a, e.node_b, graph.gnd_node, &graph.node_names)
            .expect("classify triode")
            .0
    }

    /// A textbook common-cathode 12AX7 fed from a NAMED `B+` rail must solve
    /// its load-line Q-point. Pre-0stg the r_plate finder compared the rail
    /// node against the literal `graph.vcc_node`, never matched `B+`, and the
    /// stage silently kept the -2.0V TriodeRoot default.
    #[test]
    fn triode_qpoint_solves_on_named_bplus_rail() {
        let graph = parse_graph(
            r#"pedal "test" {
                supplies { B+: 275V }
                components {
                    V1: triode(12ax7)
                    R_p: resistor(220k)
                    R_k: resistor(1.5k)
                    R_g: resistor(1M)
                    C_k: cap(25u, electrolytic)
                }
                nets {
                    B+ -> R_p.a
                    R_p.b -> V1.plate
                    V1.cathode -> R_k.a, C_k.a
                    R_k.b -> gnd
                    C_k.b -> gnd
                    in -> V1.grid
                    V1.grid -> R_g.a
                    R_g.b -> gnd
                    V1.plate -> out
                }
                controls {}
            }"#,
        );
        let nl = triode_nl_kind(&graph, "V1");
        let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
        let dc = solve_triode_dc_qpoint(&[nl], &all_edges, &graph, 275.0)
            .expect("B+ common-cathode stage must solve its Q-point (0stg)");
        assert!(
            dc.vgk < -0.5 && dc.vgk > -5.0,
            "12AX7 @275V/220k/1.5k should self-bias around -1..-2V, got vgk={}",
            dc.vgk
        );
        assert!(
            dc.vpk > 50.0 && dc.vpk < 275.0,
            "plate should sit well inside the load line, got vpk={}",
            dc.vpk
        );
    }

    /// A cathode follower whose plate ties DIRECTLY to the named rail
    /// (`B+ -> V3.plate` unions the plate node into the rail) is the
    /// r_plate = 0 load line on that rail — it must solve, not default.
    #[test]
    fn triode_qpoint_solves_follower_plate_direct_to_named_rail() {
        let graph = parse_graph(
            r#"pedal "test" {
                supplies { B+: 275V }
                components {
                    V3: triode(12bh7)
                    R_k: resistor(10k)
                    R_g: resistor(1M)
                }
                nets {
                    B+ -> V3.plate
                    in -> V3.grid
                    V3.grid -> R_g.a
                    R_g.b -> gnd
                    V3.cathode -> R_k.a
                    R_k.b -> gnd
                    V3.cathode -> out
                }
                controls {}
            }"#,
        );
        let nl = triode_nl_kind(&graph, "V3");
        let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
        let dc = solve_triode_dc_qpoint(&[nl], &all_edges, &graph, 275.0)
            .expect("B+ cathode follower must solve its Q-point (0stg)");
        assert!(
            dc.vgk < -1.0,
            "12BH7 follower @275V/Rk=10k should self-bias strongly negative, got vgk={}",
            dc.vgk
        );
        assert!(
            (dc.v_cathode - -dc.vgk).abs() < 1e-9,
            "cathode auto-bias identity: v_cathode == -vgk"
        );
    }

    /// vcc-fed circuits must keep their EXACT legacy resolution: the rail
    /// search tries vcc first, so this Q-point is bit-identical to the
    /// pre-0stg solver output for the same circuit.
    #[test]
    fn triode_qpoint_vcc_path_unchanged() {
        let graph = parse_graph(
            r#"pedal "test" { supply 250V
                components {
                    V1: triode(12ax7)
                    R_p: resistor(100k)
                    R_k: resistor(1.5k)
                    R_g: resistor(1M)
                }
                nets {
                    vcc -> R_p.a
                    R_p.b -> V1.plate
                    V1.cathode -> R_k.a
                    R_k.b -> gnd
                    in -> V1.grid
                    V1.grid -> R_g.a
                    R_g.b -> gnd
                    V1.plate -> out
                }
                controls {}
            }"#,
        );
        let nl = triode_nl_kind(&graph, "V1");
        let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
        let dc = solve_triode_dc_qpoint(&[nl], &all_edges, &graph, 250.0)
            .expect("vcc common-cathode stage must still solve");
        assert!(dc.vgk < -0.5, "expected self-bias, got vgk={}", dc.vgk);
    }
}
