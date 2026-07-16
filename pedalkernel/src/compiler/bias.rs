//! Unified compile-time DC bias infrastructure.
//!
//! This module provides the data types and solver that replace the per-type
//! DC bias code formerly spread across `spqr_build.rs`, `blockwise.rs` and
//! `rigid/general.rs` (triode + BJT collapsed; FET/pentode still pending).
//!
//! # Current status (ko5g.4)
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
//! Ia-relaxation loop; corpus failing-set unchanged).  ko5g.3 folded the
//! single-port-WDF triode copy in ([`solve_wdf_triode_dc_qpoint`]).
//!
//! As of ko5g.4 ALL FOUR BJT bias paths live here:
//! - grouped nodal co-solve — [`solve_bjt_group_dc_qpoint`] (unchanged);
//! - single-port WDF load line — [`solve_wdf_bjt_dc_qpoint`] (`BjtSeed` +
//!   `ControlNewton`, replaces `spqr_build.rs::compute_wdf_bjt_dc_qpoint`
//!   bit-for-bit);
//! - blockwise raw base-voltage read — [`solve_blockwise_bjt_base_bias`],
//!   with the group co-solve fallback [`solve_blockwise_bjt_group_qpoint`]
//!   (pedalkernel-y9hz: the ko5g.2 audit's S9 unconditional-conduction
//!   `default_vbe` is DELETED — undeterminable now warns + stays at cutoff);
//! - grouped-MNA NR warm-start seed — [`bjt_model_conduction_seed`] (replaces
//!   the `initial_v` physics defaults in `rigid/general.rs`).
//!
//! As of ko5g.5 the **pentode** has its FIRST DC solver (none existed — every
//! pentode rode the `PentodeRoot` −8.0 V default): [`PentodeSeed`] +
//! [`solve_wdf_pentode_dc_qpoint`] / [`solve_pentode_dc_qpoint`] ride the
//! shared core with the new `IterationScheme::MainCurrentBisection`
//! (unconditionally convergent — the triode's damped relaxation diverges for
//! high-g_m power tubes).  Includes the screen (Vg2) divider resolution, the
//! shared-cathode-Rk co-solve, the inductive plate DC path (OT primary at its
//! DCR — the first pedalkernel-bix9 landing), and the grounded-cathode
//! FIXED-BIAS deferral (the ko5g.2 la2a-V5 safeguard: warn + keep default).
//!
//! Still per-type / not yet on the shared path:
//! - the **varimu** branch inside `solve_triode_dc_qpoint` (fixed-bias, a
//!   VariMu/Raffensperger model + Ia bisection — needs a dedicated `VariMuSeed`;
//!   NOT bit-preserving under the relaxation scheme and has no characterization
//!   golden, so left as-is);
//! - the single-port-WDF **FET** solver in `spqr_build.rs`
//!   (`compute_wdf_fet_dc_qpoint`) — ko5g.6;
//! - the `rigid/general.rs::compute_dc_bias` ±0.65 V BJT port clamp (a linear
//!   dc-bias floor, adjacent to but distinct from the seeds above — flagged
//!   for ko5g.8).
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
    /// Could not determine the pentode operating point — missing topology element.
    UndeterminablePentode {
        label: String,
        missing: TopologyTerm,
    },
    /// The pentode's cathode is wired DIRECTLY to GND: a fixed-bias topology.
    ///
    /// The control-grid bias of a fixed-bias stage comes from an external
    /// (negative) bias supply that the netlist does not model, so it CANNOT be
    /// determined from topology — and trusting the grid-leak's Vg1k = 0 would
    /// slam the tube to maximum conduction (the ko5g.2 audit's la2a-V5 time
    /// bomb).  The caller must keep the model default and warn until an
    /// explicit bias hint exists (pedalkernel-ko5g.7).
    FixedBiasPentode { label: String },
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
            Self::UndeterminablePentode { label, missing } => format!(
                "Cannot determine DC operating point for pentode '{label}': \
                 missing {missing}. \
                 Check that the plate has a DC path to B+ (load resistor, or an \
                 output-transformer primary whose center tap / far end is tied \
                 to B+) and that R_cathode (cathode→GND) is present.",
            ),
            Self::FixedBiasPentode { label } => format!(
                "Pentode '{label}' has its cathode wired directly to GND \
                 (fixed-bias topology): the grid bias comes from an external \
                 bias supply the netlist does not model, so it cannot be \
                 determined from topology. Add a cathode resistor for \
                 self-bias, or an explicit grid-bias hint \
                 (`init {{ {label}: ... }}`, pedalkernel-ko5g.7) once hints \
                 support tube grid voltages.",
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
    /// A DC-conducting path (resistor, inductor, or transformer-primary
    /// winding at its DCR) from the plate to a positive supply rail.
    PlateDcPath,
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
            Self::PlateDcPath => write!(
                f,
                "DC path from plate to a positive rail (load resistor, or OT \
                 primary with its center tap / far end on B+)"
            ),
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
    // Legacy rail-voltage arm order (byte-identity for the flow-group
    // StaticBias classification): AC-ground resolves to 0 V BEFORE named
    // supplies — the pedalkernel-mgsd family quirk, preserved here.
    let legacy_rail_v = |node: NodeId| -> f64 {
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
    solve_network_bias_with_rails(
        edge_indices,
        graph,
        build_rail_set(graph),
        &legacy_rail_v,
    )
}

/// [`solve_network_bias`] with an explicit rail set + rail-voltage resolver.
///
/// The default rail set ([`build_rail_set`]) counts every AC-ground node as a
/// 0 V rail — correct for the flow-group StaticBias classification it was
/// built for, but WRONG for resolving a bypassed screen-grid node (the
/// pedalkernel-mgsd family: a screen dropper's junction has a ≥10 µF bypass
/// cap, so it IS AC ground, yet sits at ≈B+ at DC).  The pentode screen
/// resolver passes the TRUE-rail set (`gnd` + declared supplies only) AND a
/// supply-first `rail_v` (every named supply is itself AC-ground, so the
/// legacy ac-ground-first arm order would stamp B+ as 0 V) so bypassed
/// interior nodes are solved, not pinned to 0 V.
fn solve_network_bias_with_rails(
    edge_indices: &[usize],
    graph: &CircuitGraph,
    rail_set: HashSet<NodeId>,
    rail_v: &dyn Fn(NodeId) -> f64,
) -> NetworkBias {
    // Collect interior (non-rail) nodes by BFS from rails through resistor edges only.
    //
    // Caps/inductors/nonlinear elements are open at DC.  Signal-path nodes that
    // are reachable only via caps or from the circuit's in/out (not from rails via
    // resistors) must be excluded: they would produce zero-conductance rows in the
    // G matrix → singular.  BFS from rails guarantees every included node has at
    // least one DC path to a known voltage.

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
    /// divider path (`BjtSeed`), which needs Newton's robustness against the
    /// exponential Vbe stiffness.
    ///
    /// The parameters are the knobs the legacy per-type loops disagreed on.
    /// The trait default ([`BiasSeed::iteration_scheme`]) carries the constants
    /// of the deleted `spqr_build.rs::compute_wdf_bjt_dc_qpoint` loop (ko5g.4:
    /// 60 iterations, ±0.1 V step clamp, per-iteration `v_control` clamp to
    /// [0, 1] V) so the single production Newton call-site is reproduced
    /// bit-for-bit.
    ControlNewton {
        max_iter: usize,
        /// Symmetric NR step clamp (V).
        step_clamp: f64,
        /// Optional per-iteration clamp applied to `v_control` AFTER stepping.
        v_control_clamp: Option<(f64, f64)>,
    },
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
    /// Bisection on the device **main-current** residual
    /// `F(i) = i - i_model(v_ctrl(i), v_out(i))` over `[0, i_ceiling]`.
    ///
    /// `F` is monotone increasing for the self-bias load line (`F' = 1 +
    /// n·R_k·g_m·(…) > 0`), so bisection ALWAYS converges — unlike the damped
    /// relaxation, whose iteration map has slope `0.5·(1 − R_degen·g_m)` and
    /// DIVERGES for high-transconductance power pentodes (`R_k·g_m > 3`, e.g.
    /// an EL34 with Rk = 470 Ω).  Added for the pentode (ko5g.5); a candidate
    /// for the vari-mu migration (ko5g.6), whose legacy branch already
    /// bisects.
    ///
    /// `i_ceiling` bounds the search when the load line cannot (`r_load = 0`,
    /// the transformer-coupled plate): pick a value comfortably above any
    /// audio-tube plate current (1 A default at the pentode seed).
    MainCurrentBisection {
        i_ceiling: f64,
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
    /// ∂(I_main + I_control)/∂V_control — the TOTAL-current derivative, which is
    /// what the degeneration term of the Newton Jacobian actually needs
    /// (`dF/dV = -dI_ctrl·R_th - 1 - dI_total·R_degen`).
    ///
    /// ko5g.4 NOTE (bit-identity): the deleted `compute_wdf_bjt_dc_qpoint` loop
    /// computed this as `(ic2 + ib2 - ie) / h` — total currents summed BEFORE
    /// the finite-difference division.  Carrying the total derivative as one
    /// field (instead of `di_main + di_control` summed in the solver) preserves
    /// that floating-point grouping exactly.  For devices with `i_control = 0`
    /// (triode, FET) this is simply the transconductance g_m.
    pub(super) di_total_dv_control: f64,
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
    /// Defaults to `ControlNewton` with the deleted
    /// `spqr_build.rs::compute_wdf_bjt_dc_qpoint` loop constants (ko5g.4), so
    /// the BJT-divider path reproduces the legacy per-type Newton bit-for-bit.
    /// The triode overrides this to `MainCurrentRelaxation` so the shared
    /// solver reproduces the legacy Ia-relaxation Q-point bit-for-bit.
    fn iteration_scheme(&self) -> IterationScheme {
        IterationScheme::ControlNewton {
            max_iter: 60,
            step_clamp: 0.1,
            v_control_clamp: Some((0.0, 1.0)),
        }
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
    solve_located_operating_point(seed, &topo)
}

/// Run the iteration scheme against an already-located [`BiasTopology`].
///
/// Split out of [`solve_operating_point`] (ko5g.3) so call-sites that also
/// need topology values themselves (e.g. `R_cathode` for the plate-current
/// readback in `solve_wdf_triode_dc_qpoint`) can locate once and solve once.
pub(super) fn solve_located_operating_point<S: BiasSeed>(
    seed: &S,
    topo: &BiasTopology,
) -> Result<DeviceOperatingPoint, BiasError> {
    // Both schemes converge to the same load-line fixed point and return the
    // triple (v_control, i_total, v_output) at that point.
    let (v_ctrl, i_total_final, v_output_final) = match seed.iteration_scheme() {
        IterationScheme::ControlNewton {
            max_iter,
            step_clamp,
            v_control_clamp,
        } => control_newton_solve(seed, topo, max_iter, step_clamp, v_control_clamp),
        IterationScheme::MainCurrentRelaxation {
            initial_i_main,
            damping,
            max_iter,
            tol,
        } => main_current_relaxation_solve(seed, topo, initial_i_main, damping, max_iter, tol),
        IterationScheme::MainCurrentBisection {
            i_ceiling,
            max_iter,
            tol,
        } => main_current_bisection_solve(seed, topo, i_ceiling, max_iter, tol),
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
/// v_output)` at the converged Q-point.
///
/// With the trait-default parameters (`max_iter = 60`, `step_clamp = 0.1`,
/// `v_control_clamp = [0, 1]`) and a `DeviceIv` whose derivatives use the
/// legacy floating-point grouping (see [`DeviceIv::di_total_dv_control`]),
/// this loop reproduces the deleted `spqr_build.rs::compute_wdf_bjt_dc_qpoint`
/// Newton bit-for-bit (pinned by
/// `wdf_bjt_qpoint_bit_reproduces_deleted_loop`).
fn control_newton_solve<S: BiasSeed>(
    seed: &S,
    topo: &BiasTopology,
    max_iter: usize,
    step_clamp: f64,
    v_control_clamp: Option<(f64, f64)>,
) -> (f64, f64, f64) {
    let mut v_ctrl = seed.initial_v_control();

    for _ in 0..max_iter {
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

        // dF/dV_ctrl = -dIb/dV * Rth - 1 - dI_total/dV * R_degen
        let df = -iv.di_control_dv_control * topo.r_control_thevenin
            - 1.0
            - iv.di_total_dv_control * topo.r_degeneration;

        if df.abs() < 1e-18 {
            break;
        }

        let step = (f / df).clamp(-step_clamp, step_clamp);
        v_ctrl -= step;
        if let Some((lo, hi)) = v_control_clamp {
            v_ctrl = v_ctrl.clamp(lo, hi);
        }

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

/// Bisection on the device main-current residual.  Returns `(v_control,
/// i_total, v_output)` at the converged Q-point.
///
/// The residual `F(i) = i - i_main_model(v_ctrl(i), v_out(i))` is evaluated
/// on the same load-line relations the relaxation uses (`v_out = v_rail -
/// i·r_load`, `v_ctrl = v_th - i·r_degen`; control current treated as zero —
/// tube grids draw none at the operating point).  `F(0) ≤ 0` (the un-biased
/// device conducts) and `F(hi) > 0` once `v_ctrl(hi)` drives the device
/// toward cutoff, so a sign change is bracketed by doubling `hi` from the
/// load-line ceiling (or `i_ceiling` when `r_load = 0`).
fn main_current_bisection_solve<S: BiasSeed>(
    seed: &S,
    topo: &BiasTopology,
    i_ceiling: f64,
    max_iter: usize,
    tol: f64,
) -> (f64, f64, f64) {
    let eval = |i: f64| -> (f64, f64, f64) {
        let v_output = (topo.v_load_rail - i * topo.r_load).max(0.0);
        let v_control = topo.v_control_thevenin - i * topo.r_degeneration;
        let iv = seed.device_iv(TrialPoint {
            v_control,
            v_output,
        });
        (v_control, v_output, i - iv.i_main)
    };

    let mut lo = 0.0_f64;
    let mut hi = if topo.r_load > 0.0 {
        (topo.v_load_rail / topo.r_load).max(1e-9)
    } else {
        i_ceiling.max(1e-9)
    };

    let (_, _, mut f_lo) = eval(lo);
    let (_, _, mut f_hi) = eval(hi);
    // Grow the bracket if the ceiling still conducts more than it draws.
    let mut grow = 0;
    while f_hi <= 0.0 && grow < 8 && f_hi.is_finite() {
        hi *= 2.0;
        f_hi = eval(hi).2;
        grow += 1;
    }
    if !f_lo.is_finite() || !f_hi.is_finite() || f_lo.signum() == f_hi.signum() {
        // No bracketed root — return a non-finite marker; the caller's
        // validation maps it to `BiasError::NonPhysicalQpoint`.
        return (f64::NAN, f64::NAN, f64::NAN);
    }

    for _ in 0..max_iter {
        let mid = 0.5 * (lo + hi);
        let (_, _, f_mid) = eval(mid);
        if !f_mid.is_finite() {
            return (f64::NAN, f64::NAN, f64::NAN);
        }
        if f_mid.abs() < tol {
            lo = mid;
            hi = mid;
            break;
        }
        if f_lo.signum() == f_mid.signum() {
            lo = mid;
            f_lo = f_mid;
        } else {
            hi = mid;
        }
    }

    let i_main = 0.5 * (lo + hi);
    let (v_control, v_output, _) = eval(i_main);
    (v_control, i_main, v_output)
}

// ═══════════════════════════════════════════════════════════════════════════
// BiasSeed implementations
// ═══════════════════════════════════════════════════════════════════════════

// ── Triode ──────────────────────────────────────────────────────────────────

/// Which resistor-finder chain `TriodeSeed::locate_bias_topology` runs (ko5g.3).
///
/// The two production call-sites historically used DIFFERENT finder breadths,
/// and this refactor is byte-identity-gated, so the breadth is preserved
/// per-flavor rather than silently widened:
///
/// - [`DirectThenBfs`](Self::DirectThenBfs) — the grouped MNA path
///   (`rigid/general.rs` → `solve_triode_dc_qpoint`): direct-edge match first,
///   then the cap-aware BFS fallback, at every arm.
/// - [`DirectOnly`](Self::DirectOnly) — the single-port WDF path
///   (`spqr_build.rs` → `solve_wdf_triode_dc_qpoint`): the deleted
///   `compute_wdf_triode_dc_qpoint` copy never had a BFS fallback; keeping it
///   direct-only keeps every WDF Q-point (solve OR fail-to-default) identical.
///
/// Both flavors share the SAME arm order (pedalkernel-0stg): vcc in the stage
/// edge set → named rails (B+, ...) GRAPH-WIDE (named-rail edges are excluded
/// from passive claiming so a B+ plate load always lands in a different flow
/// group than its triode) → plate wired DIRECTLY to a rail (cathode follower,
/// r_plate = 0). Unifying the breadths is a deliberate future physics change,
/// not part of this refactor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum TriodeFinderFlavor {
    DirectThenBfs,
    DirectOnly,
}

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
    /// Which resistor-finder chain to run (per-call-site legacy breadth).
    pub(super) flavor: TriodeFinderFlavor,
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
        // Direct-edge FIRST, then (DirectThenBfs flavor only) cap-aware BFS as
        // a fallback.  The direct match is what the legacy per-type
        // `solve_triode_dc_qpoint` used, so trying it first keeps the
        // production triode call-site's Q-point bit-for-bit identical when a
        // direct plate/cathode resistor exists (the common grounded-cathode
        // topology).  BFS still rescues cap-coupled cathode-followers where no
        // direct edge exists.  The single-port WDF flavor (`DirectOnly`) never
        // had the BFS arms — see `TriodeFinderFlavor`.
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
        //
        // NOTE (pedalkernel-bix9): these finders treat inductive one-ports
        // (transformer windings, chokes) as OPEN at DC, but a magnetizing
        // inductance is a DC SHORT through its winding DCR — a
        // cathode/plate-loaded output transformer shifts the true op-point
        // (pultec V2: cold -13.27 V vs true -0.89 V). Now that BOTH triode
        // call-sites route through this single locate, the bix9 fix (walk
        // winding/inductor edges at their DCR) lands HERE + in
        // `find_load_resistor_direct`/`find_load_resistor_bfs` once.
        let use_bfs = self.flavor == TriodeFinderFlavor::DirectThenBfs;
        let rails = positive_supply_rails(graph);
        let graph_edges: Vec<usize> = (0..graph.edges.len()).collect();
        let (r_plate, v_load_rail) = find_load_resistor_direct(
            plate_node,
            graph.vcc_node,
            edge_indices,
            graph,
        )
        .or_else(|| {
            use_bfs
                .then(|| find_load_resistor_bfs(plate_node, graph.vcc_node, edge_indices, graph))
                .flatten()
        })
        .map(|r| (r, graph.vcc_node))
        .or_else(|| {
            rails.iter().find_map(|&rail| {
                find_load_resistor_direct(plate_node, rail, &graph_edges, graph)
                    .map(|r| (r, rail))
            })
        })
        .or_else(|| {
            use_bfs
                .then(|| {
                    rails.iter().find_map(|&rail| {
                        find_load_resistor_bfs(plate_node, rail, &graph_edges, graph)
                            .map(|r| (r, rail))
                    })
                })
                .flatten()
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

        // Find R_cathode: resistor between cathode_node and gnd_node (direct,
        // then BFS on the DirectThenBfs flavor).
        let r_cathode = find_load_resistor_direct(cathode_node, graph.gnd_node, edge_indices, graph)
            .or_else(|| {
                use_bfs
                    .then(|| {
                        find_load_resistor_bfs(cathode_node, graph.gnd_node, edge_indices, graph)
                    })
                    .flatten()
            })
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
            // i_control ≡ 0, so the total-current derivative IS g_m.
            di_total_dv_control: gm,
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

// ── BJT ─────────────────────────────────────────────────────────────────────

/// Which resistor-finder chain `BjtSeed::locate_bias_topology` runs (ko5g.4).
///
/// Exactly like [`TriodeFinderFlavor`], the historical BJT bias copies used
/// DIFFERENT finder breadths, and this refactor is byte-identity-gated, so the
/// breadth is preserved per-flavor rather than silently widened:
///
/// - [`WdfStageDirect`](Self::WdfStageDirect) — the single-port WDF path
///   (`spqr_build.rs` → [`solve_wdf_bjt_dc_qpoint`]): the deleted
///   `compute_wdf_bjt_dc_qpoint` searched the STAGE edge set only, with
///   direct-edge matches only (no cap-aware BFS in any arm); R1 across
///   `positive_supply_rails` (vcc first, then named — pedalkernel-0stg); RC
///   against the LITERAL `graph.vcc_node` only; the Thévenin base voltage
///   prefers the spqr `StaticBias` node-voltage map (legacy
///   `node_dc_voltage` arm order); PNP is handled by the magnitude mirror
///   `v_drive = |supply| - Vth|`.
/// - [`DividerBfs`](Self::DividerBfs) — the ko5g.1 `BjtNpnSeed` breadth
///   (currently exercised by characterization tests only, NO production
///   call-site): GRAPH-WIDE R1/R2/RC search, direct-edge first then cap-aware
///   BFS at every arm, RC across all rails with an estimate fallback, Thévenin
///   base voltage from [`NetworkBias::voltage_at`]. NPN only.
///
/// LEGACY GAPS — FIXED here in the pedalkernel-y9hz batch:
/// - **pedalkernel-6ou7** (FIXED): the RE/RC finder rails are keyed on device
///   polarity in `locate_wdf_stage_direct` — NPN keeps the legacy GND/vcc
///   arms bit-for-bit; the PNP mirror searches RE→positive-rails (then GND)
///   and RC→GND (then vcc).
/// - **pedalkernel-129p** (FIXED, both halves): the WDF post-clamp floor
///   ([`WDF_BJT_VBE_CLAMP`]) and the group solver's `any_active` window are
///   MODEL-AWARE via [`bjt_nominal_conduction_vbe`] — germanium (Vbe_on ≈
///   0.11-0.2 V) conducts below the silicon floor; silicon keeps the legacy
///   0.3 V floor bit-for-bit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum BjtFinderFlavor {
    WdfStageDirect,
    DividerBfs,
}

/// Post-solve Vbe clamp of the deleted `compute_wdf_bjt_dc_qpoint` (ko5g.4):
/// "A silicon BJT in conduction sits ~0.55-0.75 V; clamp defensively."
/// pedalkernel-129p (y9hz batch): the FLOOR is now model-aware at the use
/// site — it relaxes to `0.5 · bjt_nominal_conduction_vbe(model)` when that
/// is lower (germanium), keeping silicon at this legacy 0.3 V bit-for-bit.
pub(super) const WDF_BJT_VBE_CLAMP: (f64, f64) = (0.3, 0.8);

/// `BiasSeed` for a common-emitter BJT (NPN, plus PNP via the magnitude mirror
/// on the `WdfStageDirect` flavor).
pub(super) struct BjtSeed<'a> {
    pub(super) nl_kind: &'a NonlinearKind,
    pub(super) label: String,
    /// Which resistor-finder chain to run (per-call-site legacy breadth).
    pub(super) flavor: BjtFinderFlavor,
    /// The spqr `StaticBias` node-voltage map, used by the `WdfStageDirect`
    /// flavor's Thévenin-base-voltage preference (legacy `node_dc_voltage`
    /// arm order — which differs from [`NetworkBias::voltage_at`]: it
    /// consults `supply_voltages` BEFORE the ac-ground arm).  `None` behaves
    /// as an empty map.
    pub(super) wdf_bias_node_voltages: Option<&'a std::collections::BTreeMap<NodeId, f64>>,
}

impl<'a> BjtSeed<'a> {
    fn nl_terminals(&self) -> Option<(&str, NodeId, NodeId, NodeId, bool)> {
        match self.nl_kind {
            NonlinearKind::BjtNpn {
                model_name,
                base_node,
                collector_node,
                emitter_node,
            } => Some((
                model_name.as_str(),
                *base_node,
                *collector_node,
                *emitter_node,
                false,
            )),
            NonlinearKind::BjtPnp {
                model_name,
                base_node,
                collector_node,
                emitter_node,
            } => Some((
                model_name.as_str(),
                *base_node,
                *collector_node,
                *emitter_node,
                true,
            )),
            _ => None,
        }
    }

    /// The deleted WDF copy's locate: STAGE edge set, direct-edge matches only.
    fn locate_wdf_stage_direct(
        &self,
        edge_indices: &[usize],
        graph: &CircuitGraph,
        supply_voltage: f64,
    ) -> Result<BiasTopology, BiasError> {
        let undet = |missing: TopologyTerm| BiasError::UndeterminableBjt {
            label: self.label.clone(),
            missing,
        };
        let (_model_name, base_node, _collector_node, emitter_node, is_pnp) = self
            .nl_terminals()
            .ok_or_else(|| undet(TopologyTerm::BaseDivider))?;

        // Base divider: R1 = base→rail, R2 = base→gnd. Rails vcc-first, then
        // named rails with their actual voltages (pedalkernel-0stg).
        let (r1, base_rail_v) = positive_supply_rails(graph)
            .iter()
            .find_map(|&rail| {
                find_load_resistor_direct(base_node, rail, edge_indices, graph).map(|r| {
                    (
                        r,
                        rail_dc_voltage(rail, graph, supply_voltage).unwrap_or(supply_voltage),
                    )
                })
            })
            .ok_or_else(|| undet(TopologyTerm::BaseDivider))?;
        let r2 = find_load_resistor_direct(base_node, graph.gnd_node, edge_indices, graph)
            .ok_or_else(|| undet(TopologyTerm::BaseDivider))?;
        if r1 + r2 <= 0.0 {
            return Err(undet(TopologyTerm::BaseDivider));
        }

        // Prefer the StaticBias-map base voltage (matches blockwise's source) for
        // the open-circuit divider voltage; fall back to the resistor divider.
        static EMPTY_BIAS_MAP: std::collections::BTreeMap<NodeId, f64> =
            std::collections::BTreeMap::new();
        let bias_map = self.wdf_bias_node_voltages.unwrap_or(&EMPTY_BIAS_MAP);
        let vth = node_dc_voltage(base_node, bias_map, graph)
            .filter(|v| v.is_finite())
            .unwrap_or(base_rail_v * r2 / (r1 + r2));
        let rth = r1 * r2 / (r1 + r2);

        // Emitter resistor RE.  Required for the load-line solve; a
        // degeneration resistor is what makes the raw divider voltage an
        // over-bias.  pedalkernel-6ou7 FIX (pedalkernel-y9hz batch): the rail
        // RE returns to is keyed on device polarity — NPN degenerates to GND
        // (legacy arm, byte-identical); a classic PNP common-emitter mirror
        // returns RE to VCC/a positive rail (searched rails-first, with the
        // GND arm kept as a fallback for flipped "positive-ground" PNP decks
        // whose emitters sit near ground).
        let re = if is_pnp {
            positive_supply_rails(graph)
                .iter()
                .find_map(|&rail| {
                    find_load_resistor_direct(emitter_node, rail, edge_indices, graph)
                })
                .or_else(|| {
                    find_load_resistor_direct(emitter_node, graph.gnd_node, edge_indices, graph)
                })
        } else {
            find_load_resistor_direct(emitter_node, graph.gnd_node, edge_indices, graph)
        }
        .ok_or_else(|| undet(TopologyTerm::EmitterResistor))?;

        // PNP common-emitter is the mirror image: the emitter sits at VCC, the
        // divider biases the base below it, so the loop voltage that
        // forward-biases the (emitter-base) junction is (VCC - Vth) and RE
        // returns to VCC.  Working in the device's own (positive-forward) sign
        // convention, the magnitude equations are identical with
        // `v_drive = |rail - Vth|`.
        let v_drive = if is_pnp {
            (supply_voltage.abs() - vth).abs()
        } else {
            vth
        };

        Ok(BiasTopology {
            // The WDF entry computes its own legacy Vce warm-start (RC lookup
            // is vcc-literal AND optional there — see
            // `solve_wdf_bjt_dc_qpoint`), so the load-line fields are inert
            // placeholders: r_load = 0 makes the shared final-eval
            // v_output = v_load_rail, finite by construction.
            r_load: 0.0,
            v_load_rail: supply_voltage.abs(),
            r_degeneration: re,
            v_control_thevenin: v_drive,
            r_control_thevenin: rth,
            supply_voltage,
            degeneration_node: Some(emitter_node),
        })
    }

    /// The ko5g.1 graph-wide direct-then-BFS locate (NPN only; no production
    /// call-site — kept per-flavor for the characterization suite and as the
    /// candidate breadth for the ko5g.8-era unification).
    fn locate_divider_bfs(
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

        // Emitter resistor RE: emitter→gnd (pedalkernel-6ou7 applies here too).
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
}

impl<'a> BiasSeed for BjtSeed<'a> {
    fn locate_bias_topology(
        &self,
        edge_indices: &[usize],
        graph: &CircuitGraph,
        network_bias: &NetworkBias,
        supply_voltage: f64,
    ) -> Result<BiasTopology, BiasError> {
        match self.flavor {
            BjtFinderFlavor::WdfStageDirect => {
                self.locate_wdf_stage_direct(edge_indices, graph, supply_voltage)
            }
            BjtFinderFlavor::DividerBfs => {
                self.locate_divider_bfs(edge_indices, graph, network_bias, supply_voltage)
            }
        }
    }

    fn device_iv(&self, trial: TrialPoint) -> DeviceIv {
        let model_name = match self.nl_kind {
            NonlinearKind::BjtNpn { model_name, .. }
            | NonlinearKind::BjtPnp { model_name, .. } => model_name.as_str(),
            _ => "2N3904",
        };
        let model = super::helpers::gummel_poon_model(model_name);
        // Active-region collector reverse bias: vbc < 0 → exp term negligible.
        // A small fixed reverse bias keeps base_charge / transport in the
        // active region.  PNP runs in the device's own positive-forward
        // convention (the topology's `v_drive` mirror), so the model call is
        // polarity-agnostic here — exactly the deleted copy's convention.
        let vbc_active = -1.0_f64;
        let (ic, ib) = model.currents(
            trial.v_control as pedalkernel_rt::Wave,
            vbc_active as pedalkernel_rt::Wave,
        );
        let ie = ic + ib;

        // Numerical derivatives, in the deleted loop's exact grouping (the
        // total-current derivative sums the perturbed currents BEFORE the
        // finite-difference division — see `DeviceIv::di_total_dv_control`).
        let h = 1e-4_f64;
        let (ic2, ib2) = model.currents(
            (trial.v_control + h) as pedalkernel_rt::Wave,
            vbc_active as pedalkernel_rt::Wave,
        );
        let dib_dvbe = (ib2 - ib) as f64 / h;
        let die_dvbe = (ic2 + ib2 - ie) as f64 / h;

        DeviceIv {
            i_main: ic as f64,
            i_control: ib as f64,
            di_total_dv_control: die_dvbe,
            di_control_dv_control: dib_dvbe,
        }
    }

    fn device_label(&self) -> &str {
        &self.label
    }

    fn device_kind(&self) -> DeviceBiasKind {
        match self.nl_kind {
            NonlinearKind::BjtPnp { .. } => DeviceBiasKind::BjtPnp,
            _ => DeviceBiasKind::BjtNpn,
        }
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

/// DC voltage of `node` from the spqr `StaticBias` node-voltage map, with the
/// LEGACY rail arm order (moved verbatim from `spqr_build.rs`, ko5g.4): gnd →
/// declared supply voltages → ac-ground → solved map.
///
/// NOTE this order differs from [`NetworkBias::voltage_at`], which consults
/// the ac-ground arm before ANY supply arm (the pedalkernel-mgsd family):
/// here a node that is both a declared supply and AC-ground resolves to its
/// supply voltage, there it resolves to 0 V.  Both orders are preserved
/// because each is load-bearing for its historical call-sites; unifying them
/// is mgsd's business, not this refactor's.
pub(super) fn node_dc_voltage(
    node: NodeId,
    bias_node_voltages: &std::collections::BTreeMap<NodeId, f64>,
    graph: &CircuitGraph,
) -> Option<f64> {
    if node == graph.gnd_node {
        return Some(0.0);
    }
    if let Some(&v) = graph.supply_voltages.get(&node) {
        return Some(v);
    }
    if graph.ac_ground_nodes.contains(&node) {
        return Some(0.0);
    }
    bias_node_voltages.get(&node).copied()
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

/// Why a triode Q-point solve produced no operating point (ko5g.3).
///
/// Splits the old opaque `None` into the two cases the ko5g epic cares about:
///
/// - [`NotApplicable`](Self::NotApplicable): the stage is not a solvable
///   single-triode-with-grid shape at all — pentode groups (ko5g.5), vari-mu on
///   the WDF path (ko5g.6), strapped/grid-less triodes, multi-NL groups. These
///   have no triode load line to solve and stay SILENT.
/// - [`Undeterminable`](Self::Undeterminable): a REAL triode gain stage whose
///   bias could not be determined from topology (e.g. the tweed 5e3 cathodyne
///   phase inverter, per the ko5g.2 audit). The caller keeps its legacy
///   fallback (WDF path: the -2.0 V `TriodeRoot` default; MNA path: the linear
///   `dc_bias` superposition) and must WARN LOUDLY via
///   [`warn_if_undeterminable`](Self::warn_if_undeterminable). The fail-loud
///   flip (warn → `CompileError`) is ko5g.8, gated on the blast-radius audit.
#[derive(Debug, Clone)]
pub(super) enum TriodeQpointSkip {
    /// Not a single common-cathode triode-with-grid — nothing to warn about.
    NotApplicable,
    /// A triode gain stage rode into its silent legacy fallback.
    Undeterminable(BiasError),
}

impl TriodeQpointSkip {
    /// ko5g.3 warn-not-error: print a loud stderr warning when a real triode
    /// stage keeps its legacy fallback. `fallback` names what the caller
    /// actually does (the two call-sites default differently).
    pub(super) fn warn_if_undeterminable(&self, fallback: &str) {
        if let Self::Undeterminable(err) = self {
            eprintln!(
                "  [bias] WARNING: {} {fallback} (warn-not-error, pedalkernel-ko5g.3; \
                 this becomes a compile error under pedalkernel-ko5g.8)",
                err.clone().into_compile_error()
            );
        }
    }
}

/// Best-effort instance name for the triode that owns `plate_node`: the
/// `<comp>.plate` pin name if one maps to the node (→ "V2"), else the
/// lexicographically-first node alias, else the model name. Used for warnings
/// (where it must be a valid `init {}` hint target) + the
/// `PK_BIAS_QPOINT_DEBUG` table — never affects the solve.
pub(super) fn triode_instance_label(
    graph: &CircuitGraph,
    plate_node: NodeId,
    model_name: &str,
) -> String {
    let mut names: Vec<&str> = graph
        .node_names
        .iter()
        .filter(|(_, &n)| n == plate_node)
        .map(|(k, _)| k.as_str())
        .collect();
    names.sort_unstable();
    names
        .iter()
        .find(|k| k.ends_with(".plate"))
        .map(|k| k.trim_end_matches(".plate").to_owned())
        .or_else(|| names.first().map(|k| (*k).to_owned()))
        .unwrap_or_else(|| model_name.to_owned())
}

/// Display form for the `PK_BIAS_QPOINT_DEBUG` table: "V2 (12au7)".
fn triode_display_label(graph: &CircuitGraph, plate_node: NodeId, model_name: &str) -> String {
    let inst = triode_instance_label(graph, plate_node, model_name);
    if inst == model_name {
        inst
    } else {
        format!("{inst} ({model_name})")
    }
}

/// `PK_BIAS_QPOINT_DEBUG=1`: dump every triode Q-point solve (or its failure)
/// to stderr — the per-instance Vgk table used to gate bias refactors.
fn qpoint_debug(path: &str, label: &str, supply_voltage: f64, out: &Result<TriodeDcQpoint, TriodeQpointSkip>) {
    if std::env::var("PK_BIAS_QPOINT_DEBUG").is_err() {
        return;
    }
    match out {
        Ok(dc) => eprintln!(
            "[bias-qpoint] path={path} triode {label}: vgk={:.6} vpk={:.6} v_cathode={:.6} ia={:.9} supply={supply_voltage}",
            dc.vgk, dc.vpk, dc.v_cathode, dc.ia
        ),
        Err(TriodeQpointSkip::Undeterminable(err)) => eprintln!(
            "[bias-qpoint] path={path} triode {label}: UNDETERMINABLE ({err:?}) supply={supply_voltage}"
        ),
        Err(TriodeQpointSkip::NotApplicable) => {}
    }
}

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
/// - The pentode branch (`solve_triode_dc_qpoint` returns `NotApplicable` for
///   non-triode kinds) is a separate follow-up (ko5g.5).  The single-port-WDF
///   path is MIGRATED (ko5g.3): `spqr_build.rs`'s duplicate
///   `compute_wdf_triode_dc_qpoint` was deleted and that call-site now rides
///   [`solve_wdf_triode_dc_qpoint`] below (same seed/core, legacy
///   `DirectOnly` finder flavor).
///
/// Uses the load-line equations:
///   Vgk = -Ia × R_cathode    (cathode self-bias)
///   Vpk = VCC - Ia × R_plate (plate load line)
///   Ia = Triode.plate_current(Vgk, Vpk)
///
/// Returns `Err(TriodeQpointSkip::NotApplicable)` when the group isn't exactly
/// one triode-with-grid (multi-NL groups, pentodes, strapped triodes), and
/// `Err(TriodeQpointSkip::Undeterminable)` when it IS one but R_plate/R_cathode
/// cannot be found or the solve lands non-physical — the ko5g.3 warn-not-error
/// split (the caller warns and keeps its legacy fallback; ko5g.8 flips to a
/// compile error).
pub(super) fn solve_triode_dc_qpoint(
    nl_kinds: &[NonlinearKind],
    all_edges: &[usize],
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Result<TriodeDcQpoint, TriodeQpointSkip> {
    let out = solve_triode_dc_qpoint_inner(nl_kinds, all_edges, graph, supply_voltage);
    if let Some(NonlinearKind::Triode {
        model_name,
        plate_node,
        ..
    }) = nl_kinds.first()
    {
        qpoint_debug(
            "mna",
            &triode_display_label(graph, *plate_node, model_name),
            supply_voltage,
            &out,
        );
    }
    out
}

fn solve_triode_dc_qpoint_inner(
    nl_kinds: &[NonlinearKind],
    all_edges: &[usize],
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Result<TriodeDcQpoint, TriodeQpointSkip> {
    // Only handle single-triode-with-grid stages.
    if nl_kinds.len() != 1 {
        return Err(TriodeQpointSkip::NotApplicable);
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
        _ => return Err(TriodeQpointSkip::NotApplicable),
    };
    let label = triode_instance_label(graph, plate_node, model_name);
    let undet = |missing: TopologyTerm, label: &str| {
        TriodeQpointSkip::Undeterminable(BiasError::UndeterminableTriode {
            label: label.to_owned(),
            missing,
        })
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
        })
        .ok_or_else(|| undet(TopologyTerm::PlateResistor, &label))?;

    // Find R_cathode: linear resistor between cathode_node and gnd_node.
    let r_cathode = find_load_resistor_direct(cathode_node, graph.gnd_node, all_edges, graph)
        .ok_or_else(|| undet(TopologyTerm::CathodeResistor, &label))?;

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
            return Err(TriodeQpointSkip::Undeterminable(
                BiasError::NonPhysicalQpoint,
            ));
        }
        for _ in 0..80 {
            let mid = 0.5 * (lo + hi);
            let fmid = residual(mid, &mut triode);
            if !fmid.is_finite() {
                return Err(TriodeQpointSkip::Undeterminable(
                    BiasError::NonPhysicalQpoint,
                ));
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
            return Err(TriodeQpointSkip::Undeterminable(
                BiasError::NonPhysicalQpoint,
            ));
        }
        return Ok(TriodeDcQpoint {
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
        label: label.clone(),
        supply_voltage,
        parallel_count,
        flavor: TriodeFinderFlavor::DirectThenBfs,
    };
    let op = solve_operating_point(
        &seed,
        all_edges,
        graph,
        &NetworkBias::default(),
        supply_voltage,
    )
    .map_err(TriodeQpointSkip::Undeterminable)?;

    let vgk = op.control_bias;
    let vpk = op.output_warm_start;
    let v_cathode = -vgk; // cathode auto-bias: V_cathode = Ia*Rk = -Vgk
    let ia = v_cathode / r_cathode;

    // Sanity: Q-point should have negative Vgk and positive Vpk.
    if vgk >= 0.0 || vpk <= 0.0 || !vgk.is_finite() || !vpk.is_finite() {
        return Err(TriodeQpointSkip::Undeterminable(
            BiasError::NonPhysicalQpoint,
        ));
    }

    Ok(TriodeDcQpoint {
        vgk,
        vpk,
        v_cathode,
        ia,
    })
}

/// Single-port-WDF triode DC Q-point (ko5g.3) — replaces the deleted
/// `spqr_build.rs::compute_wdf_triode_dc_qpoint` duplicate. Rides the SAME
/// `TriodeSeed` locate + `MainCurrentRelaxation` core as the grouped MNA path
/// above, with the legacy WDF call-site semantics preserved exactly:
///
/// - **`TriodeFinderFlavor::DirectOnly`** — the deleted copy had no cap-aware
///   BFS fallback in any finder arm.
/// - **`parallel_count` NOT applied** (divergence, preserved): the deleted copy
///   solved the load line for a SINGLE section even though the runtime root is
///   built `.with_parallel_count(n)` (`build.rs`). Fixing that shifts every
///   parallel-section WDF Q-point and belongs with its own golden re-baseline.
/// - **No `vpk > 0` sanity** (divergence, preserved): the deleted copy accepted
///   a rail-saturated `vpk == 0` solve; only `vgk < 0` (finite) is required.
/// - **No vari-mu branch**: vari-mu is fixed-bias; on the WDF path the
///   `VariMuRoot` model default stands until ko5g.6 — `NotApplicable`, silent.
pub(super) fn solve_wdf_triode_dc_qpoint(
    nl_kind: &NonlinearKind,
    edge_indices: &[usize],
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Result<TriodeDcQpoint, TriodeQpointSkip> {
    let out = solve_wdf_triode_dc_qpoint_inner(nl_kind, edge_indices, graph, supply_voltage);
    if let NonlinearKind::Triode {
        model_name,
        plate_node,
        ..
    } = nl_kind
    {
        qpoint_debug(
            "wdf",
            &triode_display_label(graph, *plate_node, model_name),
            supply_voltage,
            &out,
        );
    }
    out
}

fn solve_wdf_triode_dc_qpoint_inner(
    nl_kind: &NonlinearKind,
    edge_indices: &[usize],
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Result<TriodeDcQpoint, TriodeQpointSkip> {
    let (model_name, plate_node) = match nl_kind {
        NonlinearKind::Triode {
            model_name,
            plate_node,
            grid_node: Some(_),
            is_vari_mu: false,
            ..
        } => (model_name.as_str(), *plate_node),
        // Strapped (grid-less) triodes, vari-mu (ko5g.6), pentodes (ko5g.5),
        // non-tube kinds: no load-line Q-point on this path — silent skip.
        _ => return Err(TriodeQpointSkip::NotApplicable),
    };
    let label = triode_instance_label(graph, plate_node, model_name);

    let seed = TriodeSeed {
        nl_kind,
        label: label.clone(),
        supply_voltage,
        // LEGACY-PRESERVING: see the fn doc — the deleted WDF copy never
        // applied the parallel-section multiplier.
        parallel_count: 1,
        flavor: TriodeFinderFlavor::DirectOnly,
    };
    let network_bias = NetworkBias::default();
    let topo = seed
        .locate_bias_topology(edge_indices, graph, &network_bias, supply_voltage)
        .map_err(TriodeQpointSkip::Undeterminable)?;
    let op = solve_located_operating_point(&seed, &topo)
        .map_err(TriodeQpointSkip::Undeterminable)?;

    let vgk = op.control_bias;
    let vpk = op.output_warm_start;
    let v_cathode = -vgk; // cathode auto-bias: V_cathode = Ia*Rk = -Vgk

    // Legacy WDF sanity: negative, finite Vgk only (no vpk > 0 requirement).
    if vgk >= 0.0 || !vgk.is_finite() || !v_cathode.is_finite() {
        return Err(TriodeQpointSkip::Undeterminable(
            BiasError::NonPhysicalQpoint,
        ));
    }

    Ok(TriodeDcQpoint {
        vgk,
        vpk,
        v_cathode,
        ia: v_cathode / topo.r_degeneration,
    })
}

// ─────────────────────────────────────────────────────────────────────────────
// Pentode bias entries (ko5g.5 — the first pentode DC solver; no legacy copy
// existed, so unlike the triode/BJT migrations there are NO finder flavors to
// preserve: both call-sites share one finder breadth, graph-wide direct-then-
// BFS, plus the inductive-DC-path arms the tube fleet needs)
// ─────────────────────────────────────────────────────────────────────────────

/// Why a pentode Q-point solve produced no operating point (ko5g.5 — the
/// exact [`TriodeQpointSkip`] pattern).
///
/// - [`NotApplicable`](Self::NotApplicable): not a single pentode-with-grid —
///   silent (multi-NL pentode groups, grid-less pentodes, non-pentode kinds).
/// - [`Undeterminable`](Self::Undeterminable): a REAL pentode stage that keeps
///   its legacy fallback (the `PentodeRoot` −8.0 V `vg1k_bias` default and the
///   model-default `vg2k`) — the caller must WARN LOUDLY via
///   [`warn_if_undeterminable`](Self::warn_if_undeterminable).  This includes
///   the [`BiasError::FixedBiasPentode`] guard: a grounded-cathode (fixed-bias)
///   pentode DEFERS to the default rather than trusting the topology's
///   Vg1k = 0 hot point (the audit's la2a-V5 safeguard).  The fail-loud flip
///   is ko5g.8.
#[derive(Debug, Clone)]
pub(super) enum PentodeQpointSkip {
    /// Not a single pentode-with-grid — nothing to warn about.
    NotApplicable,
    /// A pentode stage rode into its legacy default.
    Undeterminable(BiasError),
}

impl PentodeQpointSkip {
    /// ko5g.5 warn-not-error: print a loud stderr warning when a real pentode
    /// stage keeps its legacy default.
    pub(super) fn warn_if_undeterminable(&self, fallback: &str) {
        if let Self::Undeterminable(err) = self {
            eprintln!(
                "  [bias] WARNING: {} {fallback} (warn-not-error, pedalkernel-ko5g.5; \
                 this becomes a compile error under pedalkernel-ko5g.8)",
                err.clone().into_compile_error()
            );
        }
    }
}

/// DC operating-point data for a single pentode stage.
#[derive(Debug, Clone)]
pub(super) struct PentodeDcQpoint {
    /// Control-grid bias voltage (negative for self-biased stages).
    pub(super) vg1k: f64,
    /// Plate-cathode voltage at the Q-point (≈ B+ for OT-coupled plates).
    pub(super) vpk: f64,
    /// Cathode voltage = −vg1k = n_sharing × Ia × R_cathode.
    pub(super) v_cathode: f64,
    /// Per-tube plate current at the Q-point (A).
    pub(super) ia: f64,
    /// Screen-grid DC voltage resolved from the circuit (divider/dropper at
    /// DC — NO screen-current drop, the Koren model draws none).  `None` when
    /// the netlist never wires the screen pin: the model-default `vg2k`
    /// stands (it IS the declared operating point of such fixtures).
    pub(super) vg2k: Option<f64>,
}

/// `PK_BIAS_QPOINT_DEBUG=1`: dump every pentode Q-point solve (or its failure).
fn pentode_qpoint_debug(
    path: &str,
    label: &str,
    supply_voltage: f64,
    out: &Result<PentodeDcQpoint, PentodeQpointSkip>,
) {
    if std::env::var("PK_BIAS_QPOINT_DEBUG").is_err() {
        return;
    }
    match out {
        Ok(dc) => eprintln!(
            "[bias-qpoint] path={path} pentode {label}: vg1k={:.6} vpk={:.6} v_cathode={:.6} ia={:.9} vg2k={} supply={supply_voltage}",
            dc.vg1k,
            dc.vpk,
            dc.v_cathode,
            dc.ia,
            dc.vg2k
                .map(|v| format!("{v:.6}"))
                .unwrap_or_else(|| "model-default".to_owned()),
        ),
        Err(PentodeQpointSkip::Undeterminable(err)) => eprintln!(
            "[bias-qpoint] path={path} pentode {label}: UNDETERMINABLE ({err:?}) supply={supply_voltage}"
        ),
        Err(PentodeQpointSkip::NotApplicable) => {}
    }
}

/// `BiasSeed` for a self-biased pentode power/driver stage.
///
/// The screen voltage is resolved BEFORE the seed is built
/// ([`resolve_pentode_screen_dc`]) and carried as a plain field: `Vg2k` is an
/// external parameter of the Koren pentode plate equation, not a solved port.
pub(super) struct PentodeSeed<'a> {
    pub(super) nl_kind: &'a NonlinearKind,
    pub(super) label: String,
    pub(super) supply_voltage: f64,
    /// Effective screen-grid voltage for the plate-current evaluation
    /// (resolved from the circuit, or the model default when unwired).
    pub(super) vg2k: f64,
    /// Number of tubes sharing the cathode resistor (push-pull pairs share
    /// one Rk: the DC drop is `n × Ia × Rk`, folded in as
    /// `r_degeneration = n·Rk`).
    pub(super) cathode_sharing: usize,
}

impl<'a> BiasSeed for PentodeSeed<'a> {
    fn locate_bias_topology(
        &self,
        edge_indices: &[usize],
        graph: &CircuitGraph,
        _network_bias: &NetworkBias,
        supply_voltage: f64,
    ) -> Result<BiasTopology, BiasError> {
        let (plate_node, cathode_node) = match self.nl_kind {
            NonlinearKind::Pentode {
                plate_node,
                cathode_node,
                grid_node: Some(_),
                ..
            } => (*plate_node, *cathode_node),
            _ => {
                return Err(BiasError::UndeterminablePentode {
                    label: self.label.clone(),
                    missing: TopologyTerm::GridNode,
                });
            }
        };

        // ── Cathode first: the fixed-bias guard (la2a-V5 safeguard) ────────
        //
        // A cathode wired DIRECTLY to GND is a fixed-bias topology — the grid
        // bias comes from an external supply the netlist does not model.
        // Trusting the grid leak's Vg1k = 0 would slam the tube to maximum
        // conduction, an uncommanded behaviour flip in every circuit that
        // rode the −8 V default (ko5g.2 audit).  Defer to the default, loudly.
        if cathode_node == graph.gnd_node {
            return Err(BiasError::FixedBiasPentode {
                label: self.label.clone(),
            });
        }

        // ── Plate DC path ──────────────────────────────────────────────────
        //
        // Arm order (most-specific first):
        //  1. direct plate→rail resistor, stage edges then graph-wide
        //     (named-rail edges land in other flow groups — pedalkernel-0stg);
        //  2. cap-aware BFS, graph-wide;
        //  3. plate node IS a rail (r_plate = 0);
        //  4. inductive DC walk: resistors + inductors + transformer-primary
        //     edges at their DC resistance (pedalkernel-bix9 — an OT primary
        //     is a DC SHORT at its winding DCR, not an open);
        //  5. OT-primary center tap on a rail (push-pull halves: the primary
        //     a–b edge connects the two PLATES; B+ enters at the ct, which has
        //     no graph edge — resolve through `transformer_info` + the ct
        //     node aliases).
        let graph_edges: Vec<usize> = (0..graph.edges.len()).collect();
        let rails = positive_supply_rails(graph);
        let (r_plate, v_plate_rail) = rails
            .iter()
            .find_map(|&rail| {
                find_load_resistor_direct(plate_node, rail, edge_indices, graph)
                    .or_else(|| find_load_resistor_direct(plate_node, rail, &graph_edges, graph))
                    .map(|r| (r, rail))
            })
            .or_else(|| {
                rails.iter().find_map(|&rail| {
                    find_load_resistor_bfs(plate_node, rail, &graph_edges, graph)
                        .map(|r| (r, rail))
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
            .or_else(|| find_plate_dc_path_inductive(plate_node, graph, supply_voltage))
            .or_else(|| find_ot_primary_ct_rail(plate_node, graph, supply_voltage))
            .ok_or_else(|| BiasError::UndeterminablePentode {
                label: self.label.clone(),
                missing: TopologyTerm::PlateDcPath,
            })?;

        // ── Cathode resistor ───────────────────────────────────────────────
        let r_cathode = find_load_resistor_direct(cathode_node, graph.gnd_node, edge_indices, graph)
            .or_else(|| {
                find_load_resistor_direct(cathode_node, graph.gnd_node, &graph_edges, graph)
            })
            .or_else(|| find_load_resistor_bfs(cathode_node, graph.gnd_node, &graph_edges, graph))
            .ok_or_else(|| BiasError::UndeterminablePentode {
                label: self.label.clone(),
                missing: TopologyTerm::CathodeResistor,
            })?;

        Ok(BiasTopology {
            r_load: r_plate,
            v_load_rail: v_plate_rail,
            // Shared-cathode co-solve: n tubes push their (matched) plate
            // currents through ONE Rk, so the per-tube load line sees n·Rk.
            r_degeneration: r_cathode * self.cathode_sharing as f64,
            v_control_thevenin: 0.0,
            r_control_thevenin: 0.0,
            supply_voltage,
            degeneration_node: Some(cathode_node),
        })
    }

    fn device_iv(&self, trial: TrialPoint) -> DeviceIv {
        let model_name = match self.nl_kind {
            NonlinearKind::Pentode { model_name, .. } => model_name.as_str(),
            _ => "EL34",
        };
        let model = super::helpers::pentode_model(model_name);
        let v_max = self.supply_voltage.max(1.0);
        let mut root = pedalkernel_rt::elements::nonlinear::PentodeRoot::new_with_v_max(
            model,
            v_max as pedalkernel_rt::Wave,
        );
        root.set_vg2k(self.vg2k as pedalkernel_rt::Wave);
        root.set_vg1k(trial.v_control as pedalkernel_rt::Wave);
        let ia = root.plate_current(trial.v_output as pedalkernel_rt::Wave) as f64;

        // Numerical transconductance (matches the TriodeSeed convention).
        let h = 1e-3_f64;
        root.set_vg1k((trial.v_control + h) as pedalkernel_rt::Wave);
        let ia2 = root.plate_current(trial.v_output as pedalkernel_rt::Wave) as f64;
        let gm = (ia2 - ia) / h;

        DeviceIv {
            i_main: ia,
            i_control: 0.0, // control grid draws no current at the Q-point
            di_total_dv_control: gm,
            di_control_dv_control: 0.0,
        }
    }

    fn device_label(&self) -> &str {
        &self.label
    }

    fn device_kind(&self) -> DeviceBiasKind {
        DeviceBiasKind::Pentode
    }

    fn initial_v_control(&self) -> f64 {
        -8.0 // unused by the bisection scheme; the model default, for symmetry
    }

    fn iteration_scheme(&self) -> IterationScheme {
        // Bisection, NOT the triode's damped relaxation: power pentodes have
        // g_m up to ~11 mS, and with Rk = 470 Ω the relaxation map's slope
        // `0.5·(1 − Rk·g_m)` exceeds 1 in magnitude — it diverges exactly on
        // the fleet's EL34 stage.  The main-current residual is monotone, so
        // bisection is unconditionally convergent.
        IterationScheme::MainCurrentBisection {
            i_ceiling: 1.0,
            max_iter: 80,
            tol: 1e-9,
        }
    }
}

/// Walk the DC-conducting subgraph from the plate to a positive rail,
/// traversing resistors (at R), inductors (at 0 Ω — the DSL models no
/// inductor DCR), and transformer PRIMARY edges (at `primary_dcr`).
/// Capacitors are open; nonlinear devices are not walked.
///
/// Returns `(series_resistance, rail_dc_voltage)` for the first (BFS-shortest)
/// rail reached with a positive DC voltage.
fn find_plate_dc_path_inductive(
    plate_node: NodeId,
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Option<(f64, f64)> {
    // DC series resistance of an edge, or None when open/not-walkable at DC.
    let edge_dc_r = |eidx: usize| -> Option<f64> {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        match graph.effective_edge_kind(eidx) {
            EdgeKind::Linear => comp.kind.resistance().filter(|r| *r > 0.0 && r.is_finite()),
            EdgeKind::Reactive => {
                if comp.kind.capacitance().is_some() {
                    None // open at DC
                } else if let Some(cfg) = comp.kind.transformer_config() {
                    // The transformer's single graph edge is the primary a–b.
                    Some(cfg.primary_dcr.max(0.0))
                } else if comp.kind.inductance().is_some() {
                    Some(0.0) // ideal inductor: DC short
                } else {
                    None
                }
            }
            _ => None,
        }
    };

    let mut adj: HashMap<NodeId, Vec<(NodeId, f64)>> = HashMap::new();
    for eidx in 0..graph.edges.len() {
        let Some(r) = edge_dc_r(eidx) else { continue };
        let e = &graph.edges[eidx];
        adj.entry(e.node_a).or_default().push((e.node_b, r));
        adj.entry(e.node_b).or_default().push((e.node_a, r));
    }

    let mut visited: HashSet<NodeId> = HashSet::new();
    let mut queue: VecDeque<(NodeId, f64)> = VecDeque::new();
    visited.insert(plate_node);
    queue.push_back((plate_node, 0.0));
    while let Some((node, acc_r)) = queue.pop_front() {
        if node != plate_node {
            if let Some(v) = rail_dc_voltage(node, graph, supply_voltage) {
                if v > 0.0 {
                    return Some((acc_r, v));
                }
                continue; // gnd / 0 V rail: not a plate supply, stop here
            }
        }
        for &(next, r) in adj.get(&node).into_iter().flatten() {
            if visited.insert(next) {
                queue.push_back((next, acc_r + r));
            }
        }
    }
    None
}

/// Push-pull OT primary: the transformer's only graph edge connects the two
/// primary ENDS (the plates); B+ enters at the center tap, which never gets a
/// graph edge.  When the plate is a primary-winding node and the ct node
/// aliases resolve to a positive rail, the plate's DC load is half the
/// primary winding at its DCR.
fn find_ot_primary_ct_rail(
    plate_node: NodeId,
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Option<(f64, f64)> {
    let info = graph.transformer_info.get(&plate_node)?;
    if info.is_secondary {
        return None;
    }
    let comp = &graph.components[info.comp_idx];
    let cfg = comp.kind.transformer_config()?;
    let ct_node = ["ct", "pri.ct", "primary.ct", "pri_ct", "primary_ct"]
        .iter()
        .find_map(|suffix| graph.node_names.get(&format!("{}.{suffix}", comp.id)))
        .copied()?;
    let v = rail_dc_voltage(ct_node, graph, supply_voltage).filter(|&v| v > 0.0)?;
    Some((0.5 * cfg.primary_dcr.max(0.0), v))
}

/// Count the tubes (components with a `plate` pin) whose `.cathode` pin sits
/// on `cathode_node` — the shared-Rk co-solve multiplier.  Diode `.cathode`
/// pins are excluded by the plate-pin requirement.
fn count_cathode_sharing_tubes(graph: &CircuitGraph, cathode_node: NodeId) -> usize {
    let mut sharing: HashSet<&str> = HashSet::new();
    for (name, &node) in &graph.node_names {
        if node != cathode_node {
            continue;
        }
        let Some(comp_id) = name.strip_suffix(".cathode") else {
            continue;
        };
        let Some(comp) = graph.components.iter().find(|c| c.id == comp_id) else {
            continue;
        };
        if comp.kind.pin_config().valid_pins.contains(&"plate") {
            sharing.insert(comp_id);
        }
    }
    sharing.len().max(1)
}

/// Resolve the screen-grid (g2) DC voltage from the circuit.
///
/// - `None`: the netlist never wires a `g2`/`screen` pin — the model-default
///   `vg2k` stands (it IS the declared op of screen-unwired fixtures).
/// - Screen on a rail: the rail's DC voltage.
/// - Otherwise: the resistor-divider nodal solve with the TRUE-rail set (a
///   bypassed screen node is AC-ground but sits at ≈B+ at DC — mgsd family).
///   A pure series dropper resolves to the feeding rail's voltage: the Koren
///   model draws no screen current, so there is no Ig2·R drop to model.
pub(super) fn resolve_pentode_screen_dc(
    label: &str,
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Option<f64> {
    let screen_node = ["g2", "screen"]
        .iter()
        .find_map(|pin| graph.node_names.get(&format!("{label}.{pin}")))
        .copied()?;

    if screen_node == graph.gnd_node {
        return Some(0.0);
    }
    if graph.supply_nodes.contains(&screen_node) || screen_node == graph.vcc_node {
        return Some(
            graph
                .supply_voltages
                .get(&screen_node)
                .copied()
                .unwrap_or(supply_voltage),
        );
    }
    if let Some(&v) = graph.supply_voltages.get(&screen_node) {
        return Some(v);
    }

    let mut true_rails: HashSet<NodeId> = HashSet::new();
    true_rails.insert(graph.gnd_node);
    true_rails.insert(graph.vcc_node);
    true_rails.extend(graph.supply_nodes.iter().copied());
    // Supply-FIRST rail voltages: every named supply is itself AC-ground, so
    // the legacy ac-ground-first arm order would stamp B+ as 0 V (mgsd).
    let true_rail_v = |node: NodeId| -> f64 {
        if node == graph.gnd_node {
            0.0
        } else if let Some(&v) = graph.supply_voltages.get(&node) {
            v
        } else if node == graph.vcc_node {
            supply_voltage
        } else {
            0.0
        }
    };
    let graph_edges: Vec<usize> = (0..graph.edges.len()).collect();
    let nb = solve_network_bias_with_rails(&graph_edges, graph, true_rails, &true_rail_v);
    nb.dc_voltages.get(&screen_node).copied()
}

/// Compute the DC operating point for a single self-biased pentode stage —
/// the grouped-MNA entry (`rigid/general.rs`), `nl_kinds.len() == 1`.
///
/// Returns `Err(NotApplicable)` for anything that is not exactly one
/// pentode-with-grid, `Err(Undeterminable)` when the topology cannot be
/// resolved (incl. the grounded-cathode fixed-bias guard) — the caller warns
/// and keeps the legacy defaults (warn-not-error until ko5g.8).
pub(super) fn solve_pentode_dc_qpoint(
    nl_kinds: &[NonlinearKind],
    all_edges: &[usize],
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Result<PentodeDcQpoint, PentodeQpointSkip> {
    if nl_kinds.len() != 1 {
        return Err(PentodeQpointSkip::NotApplicable);
    }
    let out = solve_pentode_dc_qpoint_inner(&nl_kinds[0], all_edges, graph, supply_voltage);
    if let NonlinearKind::Pentode {
        model_name,
        plate_node,
        ..
    } = &nl_kinds[0]
    {
        pentode_qpoint_debug(
            "mna",
            &triode_display_label(graph, *plate_node, model_name),
            supply_voltage,
            &out,
        );
    }
    out
}

/// Single-port-WDF pentode DC Q-point — seeds `PentodeRoot::set_bias` /
/// `set_vg2k` at the `spqr_build.rs` call-site.  Same finder breadth and
/// solver as the MNA entry (no legacy copy existed to flavor-preserve).
pub(super) fn solve_wdf_pentode_dc_qpoint(
    nl_kind: &NonlinearKind,
    edge_indices: &[usize],
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Result<PentodeDcQpoint, PentodeQpointSkip> {
    let out = solve_pentode_dc_qpoint_inner(nl_kind, edge_indices, graph, supply_voltage);
    if let NonlinearKind::Pentode {
        model_name,
        plate_node,
        ..
    } = nl_kind
    {
        pentode_qpoint_debug(
            "wdf",
            &triode_display_label(graph, *plate_node, model_name),
            supply_voltage,
            &out,
        );
    }
    out
}

fn solve_pentode_dc_qpoint_inner(
    nl_kind: &NonlinearKind,
    edge_indices: &[usize],
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Result<PentodeDcQpoint, PentodeQpointSkip> {
    let (model_name, plate_node, cathode_node) = match nl_kind {
        NonlinearKind::Pentode {
            model_name,
            plate_node,
            cathode_node,
            grid_node: Some(_),
        } => (model_name.as_str(), *plate_node, *cathode_node),
        // Grid-less pentodes have no Vg1k to solve; non-pentode kinds are not
        // ours — silent skip either way.
        _ => return Err(PentodeQpointSkip::NotApplicable),
    };
    let label = triode_instance_label(graph, plate_node, model_name);

    // Locate FIRST: the topology errors (grounded-cathode fixed-bias guard,
    // missing plate DC path) carry the actionable message and must not be
    // masked by a screen-resolution failure.
    let model = super::helpers::pentode_model(model_name);
    let mut seed = PentodeSeed {
        nl_kind,
        label: label.clone(),
        supply_voltage,
        vg2k: model.vg2_default as f64, // placeholder until the screen resolves
        cathode_sharing: count_cathode_sharing_tubes(graph, cathode_node),
    };
    let network_bias = NetworkBias::default();
    let topo = seed
        .locate_bias_topology(edge_indices, graph, &network_bias, supply_voltage)
        .map_err(PentodeQpointSkip::Undeterminable)?;

    // Screen: resolved from the circuit, model default when unwired.
    let vg2k_resolved = resolve_pentode_screen_dc(&label, graph, supply_voltage);
    let vg2k_eff = vg2k_resolved.unwrap_or(model.vg2_default as f64);
    if !(vg2k_eff > 0.0) {
        // Screen resolved to 0 V (or worse): the tube cannot conduct — a
        // netlist bug (screen strapped to GND?), not a solvable op.
        return Err(PentodeQpointSkip::Undeterminable(
            BiasError::NonPhysicalQpoint,
        ));
    }
    seed.vg2k = vg2k_eff;

    let op = solve_located_operating_point(&seed, &topo)
        .map_err(PentodeQpointSkip::Undeterminable)?;

    let vg1k = op.control_bias;
    let vpk = op.output_warm_start;
    let v_cathode = -vg1k; // self-bias: V_k = n·Ia·Rk = -Vg1k
    // r_degeneration = n·Rk, so v_cathode / r_degeneration is the PER-TUBE Ia.
    let ia = if topo.r_degeneration > 0.0 {
        v_cathode / topo.r_degeneration
    } else {
        0.0
    };

    // Sanity: self-biased pentode must land at negative Vg1k, positive Vpk.
    if vg1k >= 0.0 || vpk <= 0.0 || !vg1k.is_finite() || !vpk.is_finite() || !ia.is_finite() {
        return Err(PentodeQpointSkip::Undeterminable(
            BiasError::NonPhysicalQpoint,
        ));
    }

    Ok(PentodeDcQpoint {
        vg1k,
        vpk,
        v_cathode,
        ia,
        vg2k: vg2k_resolved,
    })
}

// ─────────────────────────────────────────────────────────────────────────────
// Single-device BJT bias entries (ko5g.4 — the three collapsed copies)
// ─────────────────────────────────────────────────────────────────────────────

/// Why a single-device BJT bias resolution produced no value (ko5g.4 — the
/// exact [`TriodeQpointSkip`] pattern from ko5g.3).
///
/// - [`NotApplicable`](Self::NotApplicable): the element is not a BJT at all —
///   nothing to warn about, silent.
/// - [`Undeterminable`](Self::Undeterminable): a REAL BJT stage whose bias
///   could not be determined.  The caller keeps the cutoff fallback
///   (`BjtRoot`'s vbe_bias = 0 default — both the WDF and, since
///   pedalkernel-y9hz, the blockwise path) and must WARN LOUDLY via
///   [`warn_if_undeterminable`](Self::warn_if_undeterminable).  The fail-loud
///   flip (warn → `CompileError`) is ko5g.8.
#[derive(Debug, Clone)]
pub(super) enum BjtQpointSkip {
    /// Not a BJT — nothing to warn about.
    NotApplicable,
    /// A BJT stage rode into its legacy fallback.
    Undeterminable(BiasError),
}

impl BjtQpointSkip {
    /// ko5g.4 warn-not-error: print a loud stderr warning when a real BJT
    /// stage keeps its legacy fallback. `fallback` names what the caller
    /// actually does (the call-sites default differently).
    pub(super) fn warn_if_undeterminable(&self, fallback: &str) {
        if let Self::Undeterminable(err) = self {
            eprintln!(
                "  [bias] WARNING: {} {fallback} (warn-not-error, pedalkernel-ko5g.4; \
                 this becomes a compile error under pedalkernel-ko5g.8)",
                err.clone().into_compile_error()
            );
        }
    }
}

/// Best-effort instance name for the BJT that owns `base_node`: the
/// `<comp>.base` pin name if one maps to the node (→ "Q2"), else the
/// lexicographically-first node alias, else the model name. Used for warnings
/// (where it must be a valid `init {}` hint target) + the
/// `PK_BIAS_QPOINT_DEBUG` table — never affects the solve.
pub(super) fn bjt_instance_label(
    graph: &CircuitGraph,
    base_node: NodeId,
    model_name: &str,
) -> String {
    let mut names: Vec<&str> = graph
        .node_names
        .iter()
        .filter(|(_, &n)| n == base_node)
        .map(|(k, _)| k.as_str())
        .collect();
    names.sort_unstable();
    names
        .iter()
        .find(|k| k.ends_with(".base"))
        .map(|k| k.trim_end_matches(".base").to_owned())
        .or_else(|| names.first().map(|k| (*k).to_owned()))
        .unwrap_or_else(|| model_name.to_owned())
}

/// Display form for the `PK_BIAS_QPOINT_DEBUG` table: "Q2 (2n3904)".
fn bjt_display_label(graph: &CircuitGraph, base_node: NodeId, model_name: &str) -> String {
    let inst = bjt_instance_label(graph, base_node, model_name);
    if inst == model_name {
        inst
    } else {
        format!("{inst} ({model_name})")
    }
}

/// `PK_BIAS_QPOINT_DEBUG=1`: dump every single-device BJT Q-point solve (or
/// its failure) to stderr — the per-instance Vbe/Vce table used to gate bias
/// refactors (gate 2 of ko5g.4).
fn bjt_qpoint_debug(
    path: &str,
    label: &str,
    supply_voltage: f64,
    out: &Result<BjtDcQpoint, BjtQpointSkip>,
) {
    if std::env::var("PK_BIAS_QPOINT_DEBUG").is_err() {
        return;
    }
    match out {
        Ok(dc) => eprintln!(
            "[bias-qpoint] path={path} bjt {label}: vbe={:.6} vce={:.6} v_emitter={:.6} supply={supply_voltage}",
            dc.vbe, dc.vce, dc.v_emitter
        ),
        Err(BjtQpointSkip::Undeterminable(err)) => eprintln!(
            "[bias-qpoint] path={path} bjt {label}: UNDETERMINABLE ({err:?}) supply={supply_voltage}"
        ),
        Err(BjtQpointSkip::NotApplicable) => {}
    }
}

/// DC Q-point data for a single common-emitter BJT stage (moved verbatim from
/// `spqr_build.rs`, ko5g.4).
#[derive(Debug, Clone)]
pub(super) struct BjtDcQpoint {
    /// Forward base-emitter voltage at the operating point (magnitude, fed to
    /// `BjtRoot::set_bias`; the root applies PNP sign internally).
    pub(super) vbe: f64,
    /// Collector-emitter voltage at the operating point, signed for PNP, fed to
    /// `BjtRoot::set_initial_prev_v` as the NR warm-start.
    pub(super) vce: f64,
    /// Emitter-resistor DC drop (|Ie·RE|), to pre-charge the emitter bypass cap.
    pub(super) v_emitter: f64,
    /// Emitter degeneration resistance R_E (Ω) from the located bias topology;
    /// 0.0 when the emitter ties directly to its rail. Feeds the
    /// unbypassed-emitter-degeneration control divider (pedalkernel-2alk).
    /// Blockwise-path Q-points report 0.0 (the divider is a one-port
    /// `BjtRoot`/`WdfStage` mechanism; blocks carry their own emitter leg).
    pub(super) r_degeneration: f64,
    /// Small-signal transconductance dIc/dVbe (S) at the solved Q-point
    /// (central difference on the Gummel-Poon model). Feeds the
    /// unbypassed-emitter-degeneration control divider (pedalkernel-2alk).
    /// Blockwise-path Q-points report 0.0 (divider never engages there).
    pub(super) gm: f64,
    /// Collector load resistance R_C (Ω) — the direct collector→rail
    /// resistor used for the Vce warm-start; 0.0 when not found (half-rail
    /// Vce fallback). Feeds the unbypassed-emitter-degeneration control
    /// divider (pedalkernel-2alk), which targets the physical stage gain
    /// gm·R_C/(1+gm·R_E·(β+1)/β). Blockwise-path Q-points report 0.0.
    pub(super) r_load: f64,
}

/// Single-port-WDF BJT DC Q-point (ko5g.4) — replaces the deleted
/// `spqr_build.rs::compute_wdf_bjt_dc_qpoint` copy. Rides the SAME
/// `BjtSeed` locate + `ControlNewton` core as the (test-only) divider path,
/// with the legacy WDF call-site semantics preserved exactly:
///
/// - **`BjtFinderFlavor::WdfStageDirect`** — stage-edge-set, direct-edge-only
///   finders; RC vcc-literal; StaticBias-map Thévenin preference (legacy
///   `node_dc_voltage` arm order); the PNP magnitude mirror.  See
///   [`BjtFinderFlavor`].
/// - **Newton loop** — the trait-default `ControlNewton` parameters ARE the
///   deleted loop's (60 iters, ±0.1 step clamp, [0,1] V per-iter clamp,
///   0.65 V start, h = 1e-4, Vbc = -1 V), and `BjtSeed::device_iv` carries the
///   deleted loop's exact finite-difference grouping — bit-for-bit, pinned by
///   `wdf_bjt_qpoint_bit_reproduces_deleted_loop`.
/// - **Post-solve** — reject `vbe <= 0`; clamp to [`WDF_BJT_VBE_CLAMP`] with
///   a MODEL-AWARE floor (pedalkernel-129p fixed: germanium conducts below
///   the silicon 0.3 V floor; silicon keeps it bit-for-bit); Vce from a
///   polarity-keyed stage-set RC lookup (pedalkernel-6ou7 fixed: NPN
///   vcc-literal as before, PNP collector→GND first) with the half-rail
///   fallback, clamped to [0, vcc], PNP-signed; `v_emitter = |Ie·RE|`.
///
/// Returns `Err(BjtQpointSkip::NotApplicable)` when the element is not a BJT,
/// `Err(BjtQpointSkip::Undeterminable)` when it IS one but the base divider /
/// emitter resistor cannot be located or the solve lands non-physical — the
/// warn-not-error split (the caller warns and keeps its legacy cutoff
/// fallback; ko5g.8 flips to a compile error).
pub(super) fn solve_wdf_bjt_dc_qpoint(
    nl_kind: &NonlinearKind,
    edge_indices: &[usize],
    graph: &CircuitGraph,
    bias_node_voltages: &std::collections::BTreeMap<NodeId, f64>,
    supply_voltage: f64,
) -> Result<BjtDcQpoint, BjtQpointSkip> {
    let out = solve_wdf_bjt_dc_qpoint_inner(
        nl_kind,
        edge_indices,
        graph,
        bias_node_voltages,
        supply_voltage,
    );
    if let NonlinearKind::BjtNpn {
        model_name,
        base_node,
        ..
    }
    | NonlinearKind::BjtPnp {
        model_name,
        base_node,
        ..
    } = nl_kind
    {
        bjt_qpoint_debug(
            "wdf",
            &bjt_display_label(graph, *base_node, model_name),
            supply_voltage,
            &out,
        );
    }
    out
}

fn solve_wdf_bjt_dc_qpoint_inner(
    nl_kind: &NonlinearKind,
    edge_indices: &[usize],
    graph: &CircuitGraph,
    bias_node_voltages: &std::collections::BTreeMap<NodeId, f64>,
    supply_voltage: f64,
) -> Result<BjtDcQpoint, BjtQpointSkip> {
    let (model_name, base_node, collector_node, is_pnp) = match nl_kind {
        NonlinearKind::BjtNpn {
            model_name,
            base_node,
            collector_node,
            ..
        } => (model_name.as_str(), *base_node, *collector_node, false),
        NonlinearKind::BjtPnp {
            model_name,
            base_node,
            collector_node,
            ..
        } => (model_name.as_str(), *base_node, *collector_node, true),
        // Not a BJT: no base-loop Q-point on this path — silent skip.
        _ => return Err(BjtQpointSkip::NotApplicable),
    };
    let label = bjt_instance_label(graph, base_node, model_name);

    let seed = BjtSeed {
        nl_kind,
        label: label.clone(),
        flavor: BjtFinderFlavor::WdfStageDirect,
        wdf_bias_node_voltages: Some(bias_node_voltages),
    };
    let network_bias = NetworkBias::default();
    let topo = seed
        .locate_bias_topology(edge_indices, graph, &network_bias, supply_voltage)
        .map_err(BjtQpointSkip::Undeterminable)?;
    let op = solve_located_operating_point(&seed, &topo)
        .map_err(BjtQpointSkip::Undeterminable)?;

    // Legacy WDF sanity: a non-positive solved Vbe means the base loop drove
    // the junction out of conduction — treat as no Q-point.
    let vbe = op.control_bias;
    if !vbe.is_finite() || vbe <= 0.0 {
        return Err(BjtQpointSkip::Undeterminable(BiasError::NonPhysicalQpoint));
    }
    // A silicon BJT in conduction sits ~0.55-0.75 V; clamp defensively.
    // pedalkernel-129p FIX (WDF half, pedalkernel-y9hz batch): the floor is
    // MODEL-AWARE — a germanium device (AC128/OC44: Vbe_on ≈ 0.11-0.2 V)
    // genuinely conducts BELOW the silicon floor, and forcing it up to 0.3 V
    // over-biased every Ge WDF stage. The floor relaxes to half the model's
    // nominal 1 mA conduction voltage when that is lower; silicon models
    // (Vbe_on ≈ 0.65 → half ≈ 0.33 > 0.3) keep the legacy 0.3 V floor
    // bit-for-bit.
    let model = super::helpers::gummel_poon_model(model_name);
    let (clamp_lo, clamp_hi) = WDF_BJT_VBE_CLAMP;
    let clamp_lo = clamp_lo.min(0.5 * bjt_nominal_conduction_vbe(&model));
    let vbe = vbe.clamp(clamp_lo, clamp_hi);

    // Collector-emitter operating-point voltage, for the BjtRoot warm-start.
    // RC = collector→rail.  Vce = |VCC - Ic·RC - Ie·RE| (active region).  Used
    // to pre-seed `prev_v` so the WDF/NR solve cold-starts AT the Q-point
    // instead of 0 V, eliminating the bias-settling startup transient (ngspice
    // runs a `.op` before `.tran`).  Falls back to half-rail when RC is absent.
    let vbc_active = -1.0_f64;
    let (ic, ib) = model.currents(
        vbe as pedalkernel_rt::Wave,
        vbc_active as pedalkernel_rt::Wave,
    );
    let ie = ic + ib;
    let re = topo.r_degeneration;
    // pedalkernel-6ou7 FIX (second half): the RC rail is polarity-keyed too —
    // NPN keeps the legacy vcc-LITERAL lookup bit-for-bit; the PNP mirror's
    // load returns to GND (collector pulls toward ground), with the vcc arm
    // kept as a fallback for flipped positive-ground decks.
    let rc = if is_pnp {
        find_load_resistor_direct(collector_node, graph.gnd_node, edge_indices, graph).or_else(
            || find_load_resistor_direct(collector_node, graph.vcc_node, edge_indices, graph),
        )
    } else {
        find_load_resistor_direct(collector_node, graph.vcc_node, edge_indices, graph)
    };
    let vcc = supply_voltage.abs();
    let vce = match rc {
        Some(rc) => (vcc - ic * rc - ie * re).clamp(0.0, vcc),
        None => vcc * 0.5,
    };
    let vce = if is_pnp { -vce } else { vce };

    // Emitter DC voltage = Ie·RE (NPN: positive above gnd).  Used to pre-charge
    // the emitter bypass cap so its RE·CE time-constant transient does not
    // appear at startup.  For PNP the emitter returns to VCC; the cap across RE
    // still charges to the |Ie·RE| drop, so we report the magnitude.
    let v_emitter = (ie * re).abs();

    // Small-signal transconductance gm = dIc/dVbe at the Q-point, by central
    // difference on the same model evaluation used for Ic above (mirrors the
    // solver's own finite-difference style).  Feeds the unbypassed-emitter
    // degeneration control divider (pedalkernel-2alk); NOT used by the DC
    // seed itself, so the solved vbe/vce stay bit-for-bit.
    let gm_h = 1e-4;
    let (ic_hi, _) = model.currents(
        (vbe + gm_h) as pedalkernel_rt::Wave,
        vbc_active as pedalkernel_rt::Wave,
    );
    let (ic_lo, _) = model.currents(
        (vbe - gm_h) as pedalkernel_rt::Wave,
        vbc_active as pedalkernel_rt::Wave,
    );
    let gm = ((ic_hi - ic_lo) as f64 / (2.0 * gm_h)).max(0.0);

    Ok(BjtDcQpoint {
        vbe,
        vce,
        v_emitter,
        r_degeneration: re,
        gm,
        r_load: rc.unwrap_or(0.0),
    })
}

/// Blockwise-path BJT base bias (ko5g.4) — replaces the copy that lived at
/// `blockwise.rs::lower_block_stages` (the ko5g.2 audit's S9 site).
///
/// DIVERGENCE (preserved): unlike the WDF load-line solve above, this flavor
/// never solved anything — it reads the RAW `StaticBias` base-node DC voltage
/// of the first NL edge whose `<comp>.base` pin is in the map and caps it at
/// 0.8 V (`v_base.min(0.8)` — no lower clamp, no emitter-degeneration
/// correction, no PNP sign handling).  Collapsing it here makes the divergence
/// visible and greppable; making it a real load-line solve is a deliberate
/// physics change for a later bead, not this refactor.
///
/// `Err(Undeterminable)` when no NL edge's base voltage is in the map — the
/// caller falls through to the REAL group co-solve
/// [`solve_blockwise_bjt_group_qpoint`] (pedalkernel-y9hz; the unconditional
/// 0.6 V conduction default that used to fire here is DELETED).
pub(super) fn solve_blockwise_bjt_base_bias(
    nl_edge_indices: &[usize],
    graph: &CircuitGraph,
    bias_node_voltages: &std::collections::BTreeMap<NodeId, f64>,
) -> Result<f64, BjtQpointSkip> {
    let base_bias = nl_edge_indices.iter().find_map(|&eidx| {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        let base_key = format!("{}.base", comp.id);
        let base_node = graph.node_names.get(&base_key)?;
        bias_node_voltages.get(base_node).copied()
    });

    // Label for warnings/debug: the first NL component in the block.
    let label = nl_edge_indices
        .first()
        .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.clone())
        .unwrap_or_else(|| "BJT".to_owned());

    let out = match base_bias {
        Some(v_base) => Ok(v_base.min(0.8)),
        None => Err(BjtQpointSkip::Undeterminable(BiasError::UndeterminableBjt {
            label: label.clone(),
            missing: TopologyTerm::BaseDivider,
        })),
    };
    if std::env::var("PK_BIAS_QPOINT_DEBUG").is_ok() {
        match &out {
            Ok(vbe) => eprintln!(
                "[bias-qpoint] path=blockwise bjt {label}: vbe={vbe:.6} (StaticBias map)"
            ),
            Err(BjtQpointSkip::Undeterminable(err)) => {
                eprintln!("[bias-qpoint] path=blockwise bjt {label}: UNDETERMINABLE ({err:?})")
            }
            Err(BjtQpointSkip::NotApplicable) => {}
        }
    }
    out
}

// DELETED (pedalkernel-y9hz, USER DIRECTIVE): `bjt_unconditional_default_vbe`
// — the ko5g.2 audit's S9, the anonymous `default_vbe` ko5g.4 named for
// exactly this one-line removal. The blockwise call-site now falls through to
// [`solve_blockwise_bjt_group_qpoint`] (real solve) and, when THAT is
// undeterminable, keeps the `BjtRoot` cutoff default — never a fabricated
// conduction point. `bias_tests::unconditional_conduction_default_is_deleted`
// is the greppable proof.

/// Model-derived BJT conduction seed (ko5g.4) — replaces the copy that lived
/// in `rigid/general.rs` (`initial_v` physics-based defaults for
/// `BjtTwoPort` groups; the bead's "general.rs:1205" path).
///
/// Returns the SIGNED `(vbe, vce)` NR warm-start pair for one device:
/// `vbe = nf·Vt·ln(1 mA / Is)` — the junction voltage at a nominal 1 mA
/// collector current — clamped to [0.1, 0.8] V, and `vce = supply/2`
/// (half-rail active region), both negated for PNP.
///
/// DIVERGENCE (preserved): this flavor is a MODEL-only seed — no topology at
/// all (not even the divider) — because it only warm-starts the grouped-MNA
/// NR whose DC excitation (`dc_bias`) is computed elsewhere.  It can never
/// fail, so it has no `Undeterminable` arm.
pub(super) fn bjt_model_conduction_seed(
    model: &GummelPoonModel,
    supply_voltage: f64,
    is_pnp: bool,
) -> (f64, f64) {
    let sign = if is_pnp { -1.0 } else { 1.0 };
    (
        sign * bjt_nominal_conduction_vbe(model),
        sign * supply_voltage * 0.5,
    )
}

/// Nominal conduction Vbe for a BJT model: the junction voltage at a nominal
/// 1 mA collector current, `nf·Vt·ln(1 mA / Is)`, clamped to [0.1, 0.8] V.
/// Silicon (Is ≈ 1e-14) ≈ 0.65 V; germanium (Is ≈ 2e-5, AC128) ≈ 0.11 V.
///
/// Shared by [`bjt_model_conduction_seed`] (byte-identical extraction) and
/// the MODEL-AWARE physicality gates (pedalkernel-129p, y9hz batch): any
/// fixed "conducting Vbe" window tuned for silicon rejects germanium
/// operating points that genuinely conduct at 0.12-0.25 V.
pub(super) fn bjt_nominal_conduction_vbe(model: &GummelPoonModel) -> f64 {
    (model.nf * model.vt * (1.0e-3_f64 / model.is).ln()).clamp(0.1, 0.8)
}

/// Blockwise-path REAL BJT bias solve (pedalkernel-y9hz, USER DIRECTIVE).
///
/// This replaces the deleted `bjt_unconditional_default_vbe` — the ko5g.2
/// audit's S9, the engine's most aggressive silent default: when the raw
/// `StaticBias` base-voltage read missed, the blockwise call-site FORCED the
/// device into nominal conduction (0.6 V) with no topology evidence at all.
///
/// The real solve: run [`solve_bjt_group_dc_qpoint`] — the full-network
/// source-stepping-homotopy Newton the general-MNA path seeds from (the BA283
/// fix-#1 solver) — over the WHOLE blockwise plan's edges (all blocks +
/// coupling), so DC-coupled feedback bias loops that CROSS block boundaries
/// (fuzz-face family, si_fb_amp's Rf servo) are co-solved, then read the
/// target block's BJT terminals out of the solved node-voltage map.
///
/// Returns the device-normalized Q-point:
/// - `vbe` — forward base-emitter magnitude (≥ 0; cutoff reads ≈ 0), fed to
///   `BjtRoot::set_bias` which applies PNP sign internally;
/// - `vce` — RAW `V(collector) − V(emitter)` (negative for a conducting PNP),
///   fed to `set_initial_prev_v` exactly like the WDF path's PNP-signed
///   warm-start;
/// - `v_emitter` — always 0.0 here: the blockwise call-site does not
///   pre-charge emitter bypass caps (blocks carry their own reactive state).
///
/// `Err(NotApplicable)` when the target block holds no BJT;
/// `Err(Undeterminable)` when the group solve fails — the caller warns loudly
/// and keeps the `BjtRoot` cutoff default (vbe_bias = 0), the solver's proper
/// fallback semantics. NEVER a fabricated conduction default.
pub(super) fn solve_blockwise_bjt_group_qpoint(
    target_nl_edges: &[usize],
    plan_all_edges: &[usize],
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Result<BjtDcQpoint, BjtQpointSkip> {
    // Identify the target block's BJT (first BJT-classified NL component).
    let target = target_nl_edges.iter().find_map(|&eidx| {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        let (kind, _) = comp.kind.classify_nonlinear(
            &comp.id,
            e.node_a,
            e.node_b,
            graph.gnd_node,
            &graph.node_names,
        )?;
        match kind {
            NonlinearKind::BjtNpn {
                model_name,
                base_node,
                collector_node,
                emitter_node,
            } => Some((model_name, base_node, collector_node, emitter_node, false)),
            NonlinearKind::BjtPnp {
                model_name,
                base_node,
                collector_node,
                emitter_node,
            } => Some((model_name, base_node, collector_node, emitter_node, true)),
            _ => None,
        }
    });
    let Some((model_name, base_node, collector_node, emitter_node, is_pnp)) = target else {
        return Err(BjtQpointSkip::NotApplicable);
    };
    let label = bjt_instance_label(graph, base_node, &model_name);
    let undet = || {
        BjtQpointSkip::Undeterminable(BiasError::UndeterminableBjt {
            label: label.clone(),
            missing: TopologyTerm::BaseDivider,
        })
    };

    // Classify every NL device across the plan (dedup by component — BJTs
    // contribute two graph edges) for the group co-solve.
    let mut nl_kinds: Vec<NonlinearKind> = Vec::new();
    let mut seen: HashSet<usize> = HashSet::new();
    for &eidx in plan_all_edges {
        if graph.effective_edge_kind(eidx) != EdgeKind::Nonlinear {
            continue;
        }
        let e = &graph.edges[eidx];
        if !seen.insert(e.comp_idx) {
            continue;
        }
        let comp = &graph.components[e.comp_idx];
        if let Some((kind, _)) = comp.kind.classify_nonlinear(
            &comp.id,
            e.node_a,
            e.node_b,
            graph.gnd_node,
            &graph.node_names,
        ) {
            nl_kinds.push(kind);
        }
    }

    let node_dc = solve_bjt_group_dc_qpoint(&nl_kinds, plan_all_edges, graph, supply_voltage)
        .ok_or_else(undet)?;
    let (Some(&vb), Some(&vc), Some(&ve)) = (
        node_dc.get(&base_node),
        node_dc.get(&collector_node),
        node_dc.get(&emitter_node),
    ) else {
        return Err(undet());
    };

    let sign = if is_pnp { -1.0 } else { 1.0 };
    let out = BjtDcQpoint {
        // Forward magnitude; a (true) cutoff op reads as ~0 → the root stays
        // at cutoff, matching the solved circuit instead of forcing 0.6 V.
        vbe: (sign * (vb - ve)).max(0.0),
        // Raw collector-emitter voltage: positive for conducting NPN,
        // negative for conducting PNP — the same signed convention the WDF
        // path hands `set_initial_prev_v`.
        vce: vc - ve,
        v_emitter: 0.0,
        // Blockwise blocks model the emitter leg inside their K-method
        // rungs; the one-port control divider never engages on this path.
        r_degeneration: 0.0,
        gm: 0.0,
        r_load: 0.0,
    };
    if std::env::var("PK_BIAS_QPOINT_DEBUG").is_ok() {
        eprintln!(
            "[bias-qpoint] path=blockwise-group bjt {}: vbe={:.6} vce={:.6} supply={supply_voltage}",
            bjt_display_label(graph, base_node, &model_name),
            out.vbe,
            out.vce
        );
    }
    Ok(out)
}

/// DC-closure edge set for a BJT group's operating-point solve
/// (pedalkernel-onu2, RCA GAPs 2a+4a on pedalkernel-a5ho).
///
/// [`solve_bjt_group_dc_qpoint`] stamps only the resistive edges it is HANDED.
/// The general-MNA caller (`rigid/general.rs` Step 10) historically passed the
/// flow-GROUP's own edges, so any part of a device's DC bias network that lives
/// in another flow group was invisible:
///
/// - sunflower: Q2's collector chain (`R3→BIAS→SUNDIAL→R4→rail`) is in the
///   DOWNSTREAM group, so the collector floated on `gmin` and the homotopy
///   converged a saturated artifact (vbe 0.192 / vce 0.0036 V vs the real
///   |vce| ≈ 4 V that the linear seed had already found);
/// - uberdrive / LGSM: the `vref` divider (`R100/R101`) is in another group and
///   `vref` hits no [`rail_dc_voltage`] arm, so base and vref floated to ~0 V
///   and the solve "converged" with the follower in cutoff.
///
/// This returns `group_edges` plus every DC-CONDUCTING edge of the WHOLE graph
/// transitively reachable from the group's BJT terminals — the same
/// whole-network visibility the blockwise caller already gets by passing the
/// full plan's edges (see [`solve_blockwise_bjt_group_qpoint`]). Traversal uses
/// exactly the solver's own resistor filter (`EdgeKind::Linear` +
/// `resistance() > 0`; caps/inductors/NL edges are not crossed) and TERMINATES
/// at rail nodes ([`rail_dc_voltage`] `Some`) — a rail pins the potential, so
/// networks on its far side cannot influence this group's DC.
///
/// The second return value is the set of **vref-class** nodes the traversal
/// crossed (see [`vref_class_node`]): nodes the ac-ground classification would
/// pin to 0 V DC even though their true DC level is set by their resistive
/// divider (the SD-1/TS-808 `vref` idiom — big bypass cap + ≥3 signal edges,
/// `graph.rs compute_ac_ground_nodes`). The caller hands these to
/// [`solve_bjt_group_dc_qpoint_with_overrides`] so the solve treats them as
/// INTERIOR (divider-solved) instead of a 0 V rail — otherwise a follower base
/// biased `base → R → vref` reads 0 V and the solve "converges" in cutoff
/// (the bias.rs:4064 quiescent-dead acceptance) despite a healthy real circuit.
///
/// Determinism / byte-identity: the group's edges pass through IN ORDER and
/// added edges are appended SORTED by edge index, so a group whose bias network
/// is fully group-local (BA283, fuzz_face_pnp, the legends fleet) hands the
/// solver the exact legacy list and reproduces the legacy Newton sequence
/// bit-for-bit.
pub(super) fn bjt_dc_closure_edges(
    nl_kinds: &[NonlinearKind],
    group_edges: &[usize],
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> (Vec<usize>, HashSet<NodeId>) {
    // Seed: the terminals of the same devices `solve_bjt_group_dc_qpoint`
    // stamps (NPN/PNP with base ≠ collector; diode-connected BJTs excluded).
    let mut reached: HashSet<NodeId> = HashSet::new();
    for kind in nl_kinds {
        match kind {
            NonlinearKind::BjtNpn {
                base_node,
                collector_node,
                emitter_node,
                ..
            }
            | NonlinearKind::BjtPnp {
                base_node,
                collector_node,
                emitter_node,
                ..
            } if base_node != collector_node => {
                reached.insert(*base_node);
                reached.insert(*collector_node);
                reached.insert(*emitter_node);
            }
            _ => {}
        }
    }
    if reached.is_empty() {
        return (group_edges.to_vec(), HashSet::new());
    }

    // A rail terminates DC traversal — EXCEPT vref-class nodes, whose "rail"
    // status (ac-ground ⇒ 0 V) is an AC artifact; their DC comes from the
    // divider network the traversal must keep following.
    let is_rail = |nd: NodeId| {
        rail_dc_voltage(nd, graph, supply_voltage).is_some() && !vref_class_node(nd, graph)
    };

    // Fixpoint sweep over the whole graph's DC-conducting edges. An edge joins
    // the closure when it touches a reached NON-RAIL node (expansion never
    // crosses a rail); both its endpoints then become reached.
    let mut member: HashSet<usize> = HashSet::new();
    loop {
        let mut grew = false;
        for eidx in 0..graph.edges.len() {
            if member.contains(&eidx) {
                continue;
            }
            if graph.effective_edge_kind(eidx) != EdgeKind::Linear {
                continue;
            }
            let e = &graph.edges[eidx];
            let comp = &graph.components[e.comp_idx];
            let Some(r) = comp.kind.resistance() else {
                continue;
            };
            if r <= 0.0 {
                continue;
            }
            let via_a = reached.contains(&e.node_a) && !is_rail(e.node_a);
            let via_b = reached.contains(&e.node_b) && !is_rail(e.node_b);
            if !(via_a || via_b) {
                continue;
            }
            member.insert(eidx);
            grew |= reached.insert(e.node_a);
            grew |= reached.insert(e.node_b);
        }
        if !grew {
            break;
        }
    }

    // vref-class nodes actually reached by the resistive traversal: these must
    // be solved as interior nodes, not read as 0 V rails.
    let overrides: HashSet<NodeId> = reached
        .iter()
        .copied()
        .filter(|&nd| vref_class_node(nd, graph))
        .collect();

    let in_group: HashSet<usize> = group_edges.iter().copied().collect();
    let mut added: Vec<usize> = member
        .into_iter()
        .filter(|eidx| !in_group.contains(eidx))
        .collect();
    added.sort_unstable();

    let mut out = group_edges.to_vec();
    out.extend(added);
    (out, overrides)
}

/// "vref-class" node (pedalkernel-onu2): classified AC-ground by the
/// bypassed-bias-divider arm of `graph.rs compute_ac_ground_nodes` (large cap
/// to ground + ≥3 signal edges — the SD-1/TS-808 `vref` idiom), yet NOT an
/// actual DC rail: no declared supply voltage, not `gnd`/`vcc`. At DC the
/// bypass cap is open and the node's true level is set by its resistive
/// divider; resolving it through [`rail_dc_voltage`]'s ac-ground arm (0 V) is
/// an AC-classification artifact that starves any DC bias solve consulting it.
pub(super) fn vref_class_node(node: NodeId, graph: &CircuitGraph) -> bool {
    node != graph.gnd_node
        && node != graph.vcc_node
        && !graph.supply_nodes.contains(&node)
        && !graph.supply_voltages.contains_key(&node)
        && graph.ac_ground_nodes.contains(&node)
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
    // Legacy entry: no interior overrides. The blockwise caller and the unit
    // tests go through here and keep the exact pre-onu2 rail resolution.
    solve_bjt_group_dc_qpoint_with_overrides(
        nl_kinds,
        all_edges,
        graph,
        supply_voltage,
        &HashSet::new(),
    )
}

/// [`solve_bjt_group_dc_qpoint`] with an **interior-override** set
/// (pedalkernel-onu2): nodes in `interior_overrides` are solved as interior
/// unknowns even when [`rail_dc_voltage`] would resolve them (the vref-class
/// ac-ground-artifact case — see [`bjt_dc_closure_edges`] /
/// [`vref_class_node`]). An empty set reproduces the legacy solve bit-for-bit.
pub(super) fn solve_bjt_group_dc_qpoint_with_overrides(
    nl_kinds: &[NonlinearKind],
    all_edges: &[usize],
    graph: &CircuitGraph,
    supply_voltage: f64,
    interior_overrides: &HashSet<NodeId>,
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
    // truth for rail resolution.) Interior-override nodes (vref-class,
    // pedalkernel-onu2) are forced interior so their divider sets their DC.
    let full_rail_v = |node: NodeId| -> Option<f64> {
        if interior_overrides.contains(&node) {
            None
        } else {
            rail_dc_voltage(node, graph, supply_voltage)
        }
    };

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
    let base_lambdas: [f64; 8] = [0.05, 0.1, 0.2, 0.35, 0.55, 0.75, 0.9, 1.0];

    // pedalkernel-129p (y9hz batch): the ladder density is MODEL-AWARE.
    // Branch TRACKING requires each rail step to stay within the Newton basin
    // of the previous rung's root. The legacy 8-rung ladder steps the rails
    // by up to ~1.4 V — fine for silicon (Vbe_on ≈ 0.65 V, junction scale
    // 60 mV: intermediate roots are separated by far more than a step), but a
    // GERMANIUM junction (AC128 Vbe_on ≈ 0.11 V, nf·Vt ≈ 33 mV) is already in
    // mA-scale conduction one coarse step past its root, so the continuation
    // CONVERGES onto a saturated/inverse artifact branch and rides it to a
    // non-physical "root" at full supply (measured: fuzz_face_pnp, Q1 vbc ≈
    // +1.05 V forward, collector 0.93 V ABOVE the rail — a point ngspice's
    // `.op` never lands). When any device in the group is germanium-class
    // (Vbe_on < 0.3 V), build a DENSE linear ladder whose rail step is half
    // the smallest conduction voltage (~160 rungs at 9 V — compile-time
    // trivial for these small groups). All-silicon groups keep the exact
    // legacy ladder (and start) bit-for-bit.
    let min_vbe_on = models
        .iter()
        .map(bjt_nominal_conduction_vbe)
        .fold(f64::INFINITY, f64::min);
    let lambdas: Vec<f64> = if min_vbe_on < 0.3 && supply_voltage.abs() > 0.0 {
        let d_lambda = (0.5 * min_vbe_on / supply_voltage.abs()).min(0.05);
        let steps = (1.0 / d_lambda).ceil() as usize;
        (1..=steps).map(|k| (k as f64) * d_lambda).map(|l| l.min(1.0)).collect()
    } else {
        base_lambdas.to_vec()
    };

    // Interior-node state, carried across continuation steps.  Start every node
    // at a mild forward bias relative to the first (smallest) rail scale.
    let mut v = vec![0.5 * lambdas[0] * supply_voltage; n];

    // SPICE-style iterate limiting bracket (pedalkernel-y9hz): at DC the
    // network's node voltages live within the rail hull; a Newton ITERATE has
    // no such guarantee — on a floating high-Is (germanium) cluster whose only
    // ground conductance is `gmin`, one bad step on the reverse-leakage
    // plateau (Jacobian row ≈ gmin) threw an interior node to −27 V, and the
    // 0.25 V step clamp then needed >100 iterations to crawl back (measured:
    // fuzz_face_pnp, NO CONVERGENCE at lambda=0.1). Clamp each iterate to the
    // rail hull ± 1 V — every true root is strictly inside, so this only
    // shortens wild excursions, it cannot change which root is found.
    let (v_lo, v_hi) = {
        let mut lo = 0.0_f64;
        let mut hi = 0.0_f64;
        for &node in &dc_path {
            if let Some(rv) = full_rail_v(node) {
                lo = lo.min(rv);
                hi = hi.max(rv);
            }
        }
        lo = lo.min(supply_voltage.min(0.0));
        hi = hi.max(supply_voltage.max(0.0));
        (lo - 1.0, hi + 1.0)
    };

    // Adaptive source-step refinement (pedalkernel-y9hz / 129p): the fixed
    // ladder's 2× steps move the rails by up to ~1.4 V per step — fine for
    // silicon (junction scale 60 mV, negligible current below ~0.5 V), but a
    // GERMANIUM junction (Is ≈ 20 µA, nf·Vt ≈ 33 mV) already conducts mA-scale
    // current a couple hundred mV past its previous root, so the warm start
    // lands outside the Newton basin and the step fails (measured:
    // fuzz_face_pnp stalls at lambda 0.1→0.2). Standard SPICE practice: on a
    // failed source step, RESTORE the last converged state and BISECT the
    // step, giving up only below a minimum step size / attempt budget. A run
    // whose every ladder step converges (BA283, muff — the silicon fleet)
    // executes the exact same Newton sequence as before, bit-for-bit.
    let mut pending: std::collections::VecDeque<f64> = lambdas.iter().copied().collect();
    let mut lambda_done = 0.0_f64;
    let mut attempts = 0usize;
    // Per-iteration Newton trace (pedalkernel-y9hz diagnostic — this is how
    // the −27 V excursion and the Ge branch-jump were localized).
    let trace = std::env::var("PK_BIAS_GROUP_TRACE").is_ok();
    while let Some(lambda) = pending.pop_front() {
        attempts += 1;
        if attempts > 1024 {
            if std::env::var("PK_BIAS_QPOINT_DEBUG").is_ok() {
                eprintln!(
                    "[bias-qpoint] path=mna-group: source-step attempt budget exhausted \
                     at lambda={lambda}"
                );
            }
            return None;
        }
        let v_checkpoint = v.clone();
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
            if trace {
                eprintln!(
                    "[bias-group-trace] lambda={lambda} iter={_iter} max_f={max_f:.3e} v={v:.4?}"
                );
            }
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
                v[k] = (v[k] + delta[k] * scale).clamp(v_lo, v_hi);
            }
            if !v.iter().all(|x| x.is_finite()) {
                break; // treated as a failed step below (state restored)
            }
        }
        if converged && v.iter().all(|x| x.is_finite()) {
            lambda_done = lambda;
            continue;
        }
        // Failed step: restore the last converged state and bisect.
        v = v_checkpoint;
        let step = lambda - lambda_done;
        if step < 1e-3 {
            if std::env::var("PK_BIAS_QPOINT_DEBUG").is_ok() {
                eprintln!(
                    "[bias-qpoint] path=mna-group: NO CONVERGENCE — source step \
                     {lambda_done}→{lambda} unresolvable below minimum step \
                     (n={n} interior nodes, {} BJTs)",
                    bjts.len()
                );
            }
            return None;
        }
        pending.push_front(lambda);
        pending.push_front(lambda_done + 0.5 * step);
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
    for (b, model) in bjts.iter().zip(models.iter()) {
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
            if std::env::var("PK_BIAS_QPOINT_DEBUG").is_ok() {
                eprintln!(
                    "[bias-qpoint] path=mna-group bjt {}: NON-PHYSICAL reject \
                     vbe={vbe:.6} vce={vce:.6} ceiling={vcc_ceiling:.3}",
                    bjt_display_label(graph, b.base, b.model_name),
                );
            }
            return None;
        }
        // pedalkernel-129p FIX (group half, y9hz batch): the "conducting"
        // floor is MODEL-AWARE. The old fixed `0.3 ≤ vbe` window was tuned
        // for silicon (Vbe_on ≈ 0.65, half ≈ 0.33 → the 0.3 floor is
        // preserved bit-for-bit via the `min`) and rejected every germanium
        // operating point (AC128 conducts at Vbe ≈ 0.12-0.25 V).
        let active_floor = 0.3_f64.min(0.5 * bjt_nominal_conduction_vbe(model));
        if (active_floor..=1.0).contains(&vbe) && vce > 0.05 {
            any_active = true;
        }
    }
    // pedalkernel-129p FIX (y9hz batch): a CONVERGED all-cutoff solution is a
    // real operating point, not a failed solve. The `any_active` reject was a
    // blunt proxy for "did the homotopy escape the spurious cutoff basin" —
    // but the source-stepping homotopy above IS the mechanism that escapes
    // it, and some circuits are genuinely quiescent-dead at DC (validated:
    // fuzz_face_pnp's BJT cluster has NO resistive path to ground, and
    // ngspice `.op` with a grounded input + reltol=1e-6 lands every node at
    // VCC with both AC128s at Vbe ≈ -0.3 µV / Ic ≈ 0.3 nA — nodeset at a
    // conducting guess collapses back to the same root). Rejecting that op
    // silently substituted a fabricated conduction seed downstream. Accept
    // it, LOUDLY, so bias dashboards compare the true fixed point.
    if !any_active {
        eprintln!(
            "  [bias] WARNING: BJT group DC solve converged with ALL {} device(s) in \
             cutoff — accepting the quiescent-dead operating point (pedalkernel-129p; \
             was silently rejected before pedalkernel-y9hz).",
            bjts.len()
        );
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

    // `PK_BIAS_QPOINT_DEBUG=1`: per-instance grouped op-point table (probe
    // output only — the solve above is unaffected; ko5g.4 gate 2).
    if std::env::var("PK_BIAS_QPOINT_DEBUG").is_ok() {
        for b in &bjts {
            let vb_ = node_voltage(b.base, &v);
            let vc_ = node_voltage(b.collector, &v);
            let ve_ = node_voltage(b.emitter, &v);
            let sign = if b.is_npn { 1.0 } else { -1.0 };
            eprintln!(
                "[bias-qpoint] path=mna-group bjt {}: vbe={:.6} vce={:.6} supply={supply_voltage}",
                bjt_display_label(graph, b.base, b.model_name),
                sign * (vb_ - ve_),
                sign * (vc_ - ve_),
            );
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
            flavor: TriodeFinderFlavor::DirectThenBfs,
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

    /// Verify that `solve_operating_point` with `BjtSeed` returns the same
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

        let seed = BjtSeed {
            nl_kind: &nl_kind,
            label: "Q1".to_owned(),
            flavor: BjtFinderFlavor::DividerBfs,
            wdf_bias_node_voltages: None,
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

    // ── 5b. ko5g.4: single-device BJT collapse — per-flavor behavior pins ────

    /// VERBATIM copy of the deleted `spqr_build.rs::compute_wdf_bjt_dc_qpoint`
    /// (ko5g.4), kept as the bit-reproduction reference for
    /// `wdf_bjt_qpoint_bit_reproduces_deleted_loop`.  Do not "clean up" — the
    /// exact expression grouping is the thing under test.
    #[allow(clippy::too_many_arguments)]
    fn deleted_compute_wdf_bjt_dc_qpoint(
        nl_kind: &NonlinearKind,
        edge_indices: &[usize],
        graph: &CircuitGraph,
        bias_node_voltages: &std::collections::BTreeMap<super::super::graph::NodeId, f64>,
        supply_voltage: f64,
    ) -> Option<(f64, f64, f64)> {
        use super::super::component::EdgeKind;
        let (model_name, base_node, _collector_node, emitter_node, is_pnp) = match nl_kind {
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
                false,
            ),
            NonlinearKind::BjtPnp {
                model_name,
                base_node,
                collector_node,
                emitter_node,
            } => (
                model_name.as_str(),
                *base_node,
                *collector_node,
                *emitter_node,
                true,
            ),
            _ => return None,
        };

        let find_r_to_rail = |node: super::super::graph::NodeId,
                              rail: super::super::graph::NodeId|
         -> Option<f64> {
            edge_indices.iter().find_map(|&eidx| {
                if graph.effective_edge_kind(eidx) != EdgeKind::Linear {
                    return None;
                }
                let e = &graph.edges[eidx];
                let (a, b) = (e.node_a, e.node_b);
                if (a == node && b == rail) || (b == node && a == rail) {
                    graph.components[e.comp_idx].kind.resistance()
                } else {
                    None
                }
            })
        };

        let (r1, base_rail_v) = positive_supply_rails(graph).iter().find_map(|&rail| {
            find_r_to_rail(base_node, rail).map(|r| {
                (
                    r,
                    rail_dc_voltage(rail, graph, supply_voltage).unwrap_or(supply_voltage),
                )
            })
        })?;
        let r2 = find_r_to_rail(base_node, graph.gnd_node)?;
        if r1 + r2 <= 0.0 {
            return None;
        }

        let vth = node_dc_voltage(base_node, bias_node_voltages, graph)
            .filter(|v| v.is_finite())
            .unwrap_or(base_rail_v * r2 / (r1 + r2));
        let rth = r1 * r2 / (r1 + r2);

        let re = find_r_to_rail(emitter_node, graph.gnd_node)?;

        let v_drive = if is_pnp {
            (supply_voltage.abs() - vth).abs()
        } else {
            vth
        };

        let model = super::super::helpers::gummel_poon_model(model_name);
        let vbc_active = -1.0_f64;
        let mut vbe = 0.65_f64;
        for _ in 0..60 {
            let (ic, ib) = model.currents(
                vbe as pedalkernel_rt::Wave,
                vbc_active as pedalkernel_rt::Wave,
            );
            let (ic, ib) = (ic, ib);
            let ie = ic + ib;
            let f = v_drive - ib * rth - vbe - ie * re;
            let h = 1e-4;
            let (ic2, ib2) = model.currents(
                (vbe + h) as pedalkernel_rt::Wave,
                vbc_active as pedalkernel_rt::Wave,
            );
            let (ic2, ib2) = (ic2, ib2);
            let df = -((ib2 - ib) / h) * rth - 1.0 - ((ic2 + ib2 - ie) / h) * re;
            if df.abs() < 1e-18 {
                break;
            }
            let step = (f / df).clamp(-0.1, 0.1);
            vbe -= step;
            vbe = vbe.clamp(0.0, 1.0);
            if step.abs() < 1e-9 {
                break;
            }
        }

        if !vbe.is_finite() || vbe <= 0.0 {
            return None;
        }
        let vbe = vbe.clamp(0.3, 0.8);

        let (ic, ib) = model.currents(
            vbe as pedalkernel_rt::Wave,
            vbc_active as pedalkernel_rt::Wave,
        );
        let ie = ic + ib;
        let rc = find_r_to_rail(_collector_node, graph.vcc_node);
        let vcc = supply_voltage.abs();
        let vce = match rc {
            Some(rc) => (vcc - ic * rc - ie * re).clamp(0.0, vcc),
            None => vcc * 0.5,
        };
        let vce = if is_pnp { -vce } else { vce };
        let v_emitter = (ie * re).abs();

        Some((vbe, vce, v_emitter))
    }

    /// Classify the single BJT edge of a parsed graph into its
    /// `NonlinearKind`, mirroring the WDF stage-builder call site.
    fn bjt_nl_kind(graph: &CircuitGraph, comp_id: &str, is_pnp: bool) -> NonlinearKind {
        let e = graph
            .edges
            .iter()
            .find(|e| graph.components[e.comp_idx].id == comp_id)
            .expect("bjt edge");
        let comp = &graph.components[e.comp_idx];
        let nl = comp
            .kind
            .classify_nonlinear(&comp.id, e.node_a, e.node_b, graph.gnd_node, &graph.node_names)
            .expect("classify bjt")
            .0;
        match (&nl, is_pnp) {
            (NonlinearKind::BjtNpn { .. }, false) | (NonlinearKind::BjtPnp { .. }, true) => nl,
            _ => panic!("fixture {comp_id} classified as unexpected NonlinearKind"),
        }
    }

    /// Gate-4 pin: the unified WDF entry bit-reproduces the deleted
    /// `compute_wdf_bjt_dc_qpoint` — NPN divider CE, with and without a
    /// StaticBias-map preference, and the PNP magnitude mirror.
    #[test]
    fn wdf_bjt_qpoint_bit_reproduces_deleted_loop() {
        let npn = parse_graph(
            r#"pedal "test" { supply 9V
                components {
                    Q1: npn(2n3904)
                    R1: resistor(47k)
                    R2: resistor(10k)
                    RC: resistor(4k7)
                    RE: resistor(1k)
                    C_in: cap(10u, electrolytic)
                    C_out: cap(10u, electrolytic)
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
                    in -> C_in.a
                    C_in.b -> Q1.base
                    Q1.collector -> C_out.a
                    C_out.b -> out
                }
                controls {}
            }"#,
        );
        // Classic PNP mirror with RE returned to GND so the legacy finder
        // resolves it (the RE→VCC case is pinned separately below).  The
        // divider is mirrored (base sits NEAR VCC: R1 small to vcc, R2 large
        // to gnd) so `v_drive = |VCC - Vth|` lands a real active-region point.
        let pnp = parse_graph(
            r#"pedal "test" { supply 9V
                components {
                    Q1: pnp(2n3906)
                    R1: resistor(10k)
                    R2: resistor(47k)
                    RC: resistor(4k7)
                    RE: resistor(1k)
                    C_in: cap(10u, electrolytic)
                    C_out: cap(10u, electrolytic)
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
                    in -> C_in.a
                    C_in.b -> Q1.base
                    Q1.collector -> C_out.a
                    C_out.b -> out
                }
                controls {}
            }"#,
        );

        for (graph, is_pnp, with_map) in [
            (&npn, false, false),
            (&npn, false, true),
            (&pnp, true, false),
        ] {
            let nl = bjt_nl_kind(graph, "Q1", is_pnp);
            let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
            let mut bias_map = std::collections::BTreeMap::new();
            if with_map {
                // A StaticBias solve would land near but not exactly on the
                // unloaded divider voltage — pin the preference arm.
                let base = *graph.node_names.get("Q1.base").unwrap();
                bias_map.insert(base, 1.52_f64);
            }

            let legacy =
                deleted_compute_wdf_bjt_dc_qpoint(&nl, &all_edges, graph, &bias_map, 9.0)
                    .expect("legacy loop must solve this divider CE");
            let new = solve_wdf_bjt_dc_qpoint(&nl, &all_edges, graph, &bias_map, 9.0)
                .expect("unified WDF entry must solve this divider CE");

            eprintln!(
                "wdf bjt bit-repro (pnp={is_pnp} map={with_map}): legacy=({:.15}, {:.15}, {:.15}) new=({:.15}, {:.15}, {:.15})",
                legacy.0, legacy.1, legacy.2, new.vbe, new.vce, new.v_emitter
            );
            assert!(
                (new.vbe - legacy.0).abs() < 1e-12,
                "vbe diverged: legacy={:.17} new={:.17}",
                legacy.0,
                new.vbe
            );
            assert!(
                (new.vce - legacy.1).abs() < 1e-12,
                "vce diverged: legacy={:.17} new={:.17}",
                legacy.1,
                new.vce
            );
            assert!(
                (new.v_emitter - legacy.2).abs() < 1e-12,
                "v_emitter diverged: legacy={:.17} new={:.17}",
                legacy.2,
                new.v_emitter
            );
            if is_pnp {
                assert!(new.vce < 0.0, "PNP Vce must be signed negative");
            }
        }
    }

    /// pedalkernel-6ou7 FIXED (y9hz batch): a classic PNP CE whose emitter
    /// resistor returns to VCC now SEEDS on the WDF path — the RE/RC finder
    /// rails are keyed on device polarity in `BjtSeed::locate_wdf_stage_direct`
    /// (one place, post-ko5g.4). The deleted copy returned a silent `None`
    /// here (pinned by ko5g.4 as `..fails_identically_6ou7`); this test now
    /// pins the FIX: divider 47k/470k → Vth ≈ 8.18 V, v_drive ≈ 0.82 V,
    /// RE = 2.2 k → an active-region point with a few-tens-of-µA emitter
    /// current, PNP-signed Vce, and a real |Ie·RE| emitter drop.
    #[test]
    fn wdf_bjt_pnp_re_to_vcc_seeds_6ou7() {
        let graph = parse_graph(
            r#"pedal "test" { supply 9V
                components {
                    Q1: pnp(2n3906)
                    R1: resistor(47k)
                    Rb2: resistor(470k)
                    R2: resistor(10k)
                    R3: resistor(2.2k)
                    C1: cap(100n)
                }
                nets {
                    in -> C1.a
                    C1.b -> R1.a, Q1.base
                    R1.b -> vcc
                    Q1.base -> Rb2.a
                    Rb2.b -> gnd
                    gnd -> R2.a
                    R2.b -> Q1.collector
                    Q1.emitter -> R3.a
                    R3.b -> vcc
                    Q1.collector -> out
                }
                controls {}
            }"#,
        );
        let nl = bjt_nl_kind(&graph, "Q1", true);
        let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
        let bias_map = std::collections::BTreeMap::new();

        let legacy = deleted_compute_wdf_bjt_dc_qpoint(&nl, &all_edges, &graph, &bias_map, 9.0);
        assert!(
            legacy.is_none(),
            "legacy copy silently skipped PNP RE→VCC (the 6ou7 bug this fixes)"
        );

        let dc = solve_wdf_bjt_dc_qpoint(&nl, &all_edges, &graph, &bias_map, 9.0)
            .expect("PNP RE→VCC must seed after the 6ou7 fix");
        // Silicon PNP in conduction: solved forward Vbe in the clamp window,
        // not pinned AT either clamp edge (a real solve, not a clamp artifact).
        assert!(
            dc.vbe > 0.3 && dc.vbe < 0.8,
            "solved Vbe should be a real conduction point, got {}",
            dc.vbe
        );
        // Vce PNP-signed negative, inside the rail span, and NOT the half-rail
        // fallback — RC (10k collector→gnd) must be found by the PNP arm.
        assert!(
            dc.vce < 0.0 && dc.vce > -9.0,
            "PNP Vce must be signed negative within the rail span, got {}",
            dc.vce
        );
        assert!(
            (dc.vce - (-4.5)).abs() > 0.2,
            "Vce ≈ -4.5 V means the RC finder fell back to half-rail — the \
             PNP collector→GND arm must resolve RC, got {}",
            dc.vce
        );
        // |Ie·RE| — the divider (Vth ≈ 8.18 V, v_drive ≈ 0.82 V) drops most
        // of the drive across RE ≈ 2.2 k.
        assert!(
            dc.v_emitter > 0.05 && dc.v_emitter < 0.82,
            "emitter drop should be a real Ie·RE, got {}",
            dc.v_emitter
        );
    }

    /// pedalkernel-129p (group half): a germanium PNP pair whose true DC root
    /// is QUIESCENT-DEAD must SOLVE, landing the same all-at-VCC leakage root
    /// ngspice's `.op` finds. This is the fuzz_face_pnp validate deck: its
    /// BJT cluster has NO resistive path to ground (R1 is a gnd→vcc bleed),
    /// so ngspice (grounded input, reltol=1e-6) puts every node at 9 V with
    /// both AC128s at Vbe ≈ −0.3 µV — and a nodeset at a conducting guess
    /// collapses back to the same root. Pre-y9hz the solver (a) branch-jumped
    /// onto a saturated artifact root (Q1 vbc ≈ +1.05 V, collector above the
    /// rail) because the source-stepping ladder was silicon-coarse, and
    /// (b) would have rejected the true root anyway via the `any_active`
    /// gate. Both fixed: dense Ge ladder + accepted quiescent-dead op.
    #[test]
    fn bjt_group_ge_quiescent_dead_root_accepted() {
        let graph = parse_graph(
            r#"pedal "test" { supply 9V
                components {
                    C1: cap(2.2u)
                    R1: resistor(33k)
                    R2: resistor(8.2k)
                    Q1: pnp(ac128)
                    R3: resistor(470)
                    R4: resistor(100k)
                    Q2: pnp(ac128)
                    C2: cap(10u)
                    RL: resistor(10k)
                }
                nets {
                    in -> C1.a
                    C1.b -> Q1.base
                    vcc -> Q1.emitter
                    gnd -> R1.a
                    R1.b -> vcc
                    Q2.collector -> R2.a
                    R2.b -> Q1.base
                    Q1.collector -> R4.a
                    R4.b -> Q2.base
                    vcc -> R3.a
                    R3.b -> Q2.emitter
                    Q2.collector -> C2.a
                    C2.b -> RL.a, out
                    RL.b -> gnd
                }
                controls {}
            }"#,
        );
        let nl_q1 = bjt_nl_kind(&graph, "Q1", true);
        let nl_q2 = bjt_nl_kind(&graph, "Q2", true);
        let all_edges: Vec<usize> = (0..graph.edges.len()).collect();

        let node_dc = solve_bjt_group_dc_qpoint(&[nl_q1, nl_q2], &all_edges, &graph, 9.0)
            .expect("Ge quiescent-dead group must solve (pedalkernel-129p)");

        // Every BJT-cluster node sits at the rail (within a few mV of gmin
        // leakage), matching ngspice's grounded-input .op.
        for pin in ["Q1.base", "Q1.collector", "Q2.base", "Q2.collector", "Q2.emitter"] {
            let node = *graph.node_names.get(pin).unwrap();
            let v = node_dc.get(&node).copied().unwrap_or(f64::NAN);
            assert!(
                (v - 9.0).abs() < 0.01,
                "{pin} should sit at the 9 V rail (quiescent-dead root), got {v}"
            );
        }
    }

    /// pedalkernel-129p (group half, conducting case): a germanium PNP CE
    /// whose true operating point conducts at Vbe ≈ 0.1-0.2 V — BELOW the old
    /// silicon-tuned `0.3 ≤ vbe` activity floor — must solve and be counted
    /// ACTIVE (no quiescent-dead warning path needed; the model-aware floor
    /// is 0.5·Vbe_on(AC128) ≈ 0.057 V).
    #[test]
    fn bjt_group_ge_conducting_point_below_silicon_floor_accepted() {
        let graph = parse_graph(
            r#"pedal "test" { supply 9V
                components {
                    Q1: pnp(ac128)
                    R1: resistor(47k)
                    Rb2: resistor(470k)
                    R2: resistor(10k)
                    R3: resistor(2.2k)
                    C1: cap(100n)
                }
                nets {
                    in -> C1.a
                    C1.b -> R1.a, Q1.base
                    R1.b -> vcc
                    Q1.base -> Rb2.a
                    Rb2.b -> gnd
                    gnd -> R2.a
                    R2.b -> Q1.collector
                    Q1.emitter -> R3.a
                    R3.b -> vcc
                    Q1.collector -> out
                }
                controls {}
            }"#,
        );
        let nl = bjt_nl_kind(&graph, "Q1", true);
        let all_edges: Vec<usize> = (0..graph.edges.len()).collect();

        let node_dc = solve_bjt_group_dc_qpoint(&[nl], &all_edges, &graph, 9.0)
            .expect("conducting Ge PNP CE must solve (pedalkernel-129p)");

        let vb = node_dc[graph.node_names.get("Q1.base").unwrap()];
        let ve = node_dc[graph.node_names.get("Q1.emitter").unwrap()];
        let vc = node_dc[graph.node_names.get("Q1.collector").unwrap()];
        let vbe = ve - vb; // PNP-normalized forward magnitude
        let vce = ve - vc;
        assert!(
            (0.05..0.3).contains(&vbe),
            "Ge PNP should conduct BELOW the old 0.3 V silicon floor, got vbe={vbe}"
        );
        assert!(
            vce > 1.0,
            "conducting CE should hold a real collector swing, got vce={vce}"
        );
    }

    /// Gate-4 pin: the two BJT finder flavors keep their historical breadths.
    /// A base divider reachable only THROUGH another resistor resolves under
    /// the graph-wide/BFS `DividerBfs` flavor but not under the stage-set
    /// direct-only `WdfStageDirect` flavor.
    #[test]
    fn bjt_finder_flavor_breadth_divergence_pinned() {
        let graph = parse_graph(
            r#"pedal "test" { supply 9V
                components {
                    Q1: npn(2n3904)
                    R_stop: resistor(1k)
                    R1: resistor(47k)
                    R2: resistor(10k)
                    RC: resistor(4k7)
                    RE: resistor(1k)
                }
                nets {
                    vcc -> R1.a
                    R1.b -> R_stop.a
                    R_stop.b -> Q1.base
                    Q1.base -> R2.a
                    R2.b -> gnd
                    vcc -> RC.a
                    RC.b -> Q1.collector
                    Q1.emitter -> RE.a
                    RE.b -> gnd
                    in -> Q1.base
                    Q1.collector -> out
                }
                controls {}
            }"#,
        );
        let nl = bjt_nl_kind(&graph, "Q1", false);
        let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
        let network_bias = NetworkBias::default();

        let wdf_seed = BjtSeed {
            nl_kind: &nl,
            label: "Q1".to_owned(),
            flavor: BjtFinderFlavor::WdfStageDirect,
            wdf_bias_node_voltages: None,
        };
        assert!(
            wdf_seed
                .locate_bias_topology(&all_edges, &graph, &network_bias, 9.0)
                .is_err(),
            "WdfStageDirect must NOT see the base divider through R_stop (direct-only)"
        );

        let bfs_seed = BjtSeed {
            nl_kind: &nl,
            label: "Q1".to_owned(),
            flavor: BjtFinderFlavor::DividerBfs,
            wdf_bias_node_voltages: None,
        };
        let topo = bfs_seed
            .locate_bias_topology(&all_edges, &graph, &network_bias, 9.0)
            .expect("DividerBfs must locate the divider through R_stop");
        assert!(
            topo.r_degeneration > 0.0,
            "BFS flavor should find RE too, got {topo:?}"
        );
    }

    /// Gate-4 pin: the blockwise flavor reads the RAW StaticBias base voltage
    /// (capped at 0.8 V, no lower clamp, no load line) and fails loudly —
    /// never silently — when the map has nothing for the base node.
    #[test]
    fn blockwise_bjt_base_bias_flavor_pinned() {
        let graph = parse_graph(
            r#"pedal "test" { supply 9V
                components {
                    Q1: npn(2n3904)
                    R1: resistor(47k)
                    R2: resistor(10k)
                    RE: resistor(1k)
                }
                nets {
                    vcc -> R1.a
                    R1.b -> Q1.base
                    Q1.base -> R2.a
                    R2.b -> gnd
                    Q1.emitter -> RE.a
                    RE.b -> gnd
                    in -> Q1.base
                    Q1.collector -> out
                }
                controls {}
            }"#,
        );
        let base_node = *graph.node_names.get("Q1.base").unwrap();
        let nl_edges: Vec<usize> = graph
            .edges
            .iter()
            .enumerate()
            .filter(|(_, e)| graph.components[e.comp_idx].id == "Q1")
            .map(|(i, _)| i)
            .collect();
        assert!(!nl_edges.is_empty(), "fixture must expose Q1 edges");

        // In-map: raw value passes through…
        let mut bias_map = std::collections::BTreeMap::new();
        bias_map.insert(base_node, 0.72_f64);
        let vbe = solve_blockwise_bjt_base_bias(&nl_edges, &graph, &bias_map)
            .expect("base voltage in map must resolve");
        assert_eq!(vbe, 0.72, "raw StaticBias voltage passes through");

        // …capped at 0.8 V above (and NOT floored below — legacy semantics).
        bias_map.insert(base_node, 1.53_f64);
        let vbe = solve_blockwise_bjt_base_bias(&nl_edges, &graph, &bias_map).unwrap();
        assert_eq!(vbe, 0.8, "over-bias caps at 0.8 V");
        bias_map.insert(base_node, 0.05_f64);
        let vbe = solve_blockwise_bjt_base_bias(&nl_edges, &graph, &bias_map).unwrap();
        assert_eq!(vbe, 0.05, "no lower clamp (legacy: cutoff-level reads pass)");

        // Map miss → loud Undeterminable (the caller then runs the REAL group
        // co-solve; the unconditional conduction default is DELETED — see
        // `unconditional_conduction_default_is_deleted`).
        bias_map.clear();
        let miss = solve_blockwise_bjt_base_bias(&nl_edges, &graph, &bias_map);
        assert!(
            matches!(
                miss,
                Err(BjtQpointSkip::Undeterminable(
                    BiasError::UndeterminableBjt { .. }
                ))
            ),
            "map miss must be Undeterminable (S9 flavor no longer silent), got {miss:?}"
        );
    }

    /// pedalkernel-y9hz gate-4 proof: the unconditional-conduction flavor
    /// (`bjt_unconditional_default_vbe`, the ko5g.2 audit's S9) is DELETED
    /// from the codebase — grep-level, source-of-truth check over the two
    /// files that ever defined or called it. The needle is assembled at
    /// runtime so this test's own source cannot satisfy it.
    #[test]
    fn unconditional_conduction_default_is_deleted() {
        let needle: String = ["bjt_unconditional", "_default_vbe"].concat();
        for (name, src) in [
            ("bias.rs", include_str!("bias.rs")),
            ("blockwise.rs", include_str!("blockwise.rs")),
        ] {
            let defines_or_calls = src
                .lines()
                .filter(|l| {
                    let t = l.trim_start();
                    !t.starts_with("//") && !t.starts_with("///") && !t.starts_with("//!")
                })
                .any(|l| l.contains(&needle));
            assert!(
                !defines_or_calls,
                "{name} still defines or calls `{needle}` — the S9 \
                 unconditional conduction default must stay deleted \
                 (pedalkernel-y9hz USER DIRECTIVE)"
            );
        }
    }

    /// Gate-4 pin: the grouped-MNA model-derived warm-start seed reproduces
    /// the deleted `rigid/general.rs` expression exactly (Vbe at 1 mA clamped
    /// to [0.1, 0.8], half-rail Vce, PNP-negated).
    #[test]
    fn bjt_model_conduction_seed_matches_deleted_general_expr() {
        for (model_name, supply, is_pnp) in [
            ("2N3904", 9.0, false),
            ("2N3906", 9.0, true),
            ("AC128", 9.0, true), // germanium: high Is → low Vbe, exercises the clamp floor region
            ("2N3904", 250.0, false),
        ] {
            let model = super::super::helpers::gummel_poon_model(model_name);
            // The deleted expression, verbatim.
            let sign = if is_pnp { -1.0 } else { 1.0 };
            let legacy_vbe_raw = model.nf * model.vt * (1.0e-3_f64 / model.is).ln();
            let legacy_vbe = sign * legacy_vbe_raw.clamp(0.1, 0.8);
            let legacy_vce = sign * supply * 0.5;

            let (vbe, vce) = bjt_model_conduction_seed(&model, supply, is_pnp);
            assert_eq!(
                vbe, legacy_vbe,
                "{model_name} pnp={is_pnp}: vbe seed must be bit-identical"
            );
            assert_eq!(
                vce, legacy_vce,
                "{model_name} pnp={is_pnp}: vce seed must be bit-identical"
            );
        }
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

    // ── ko5g.3: the unified single-port-WDF entry ────────────────────────────

    const VCC_CC_12AX7: &str = r#"pedal "test" { supply 250V
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
    }"#;

    /// The WDF entry reproduces the DELETED `spqr_build.rs`
    /// `compute_wdf_triode_dc_qpoint` loop bit-for-bit on the canonical vcc
    /// grounded-cathode stage, and lands the SAME Q-point as the grouped MNA
    /// entry (both resolve the same direct R_plate/R_cathode here).
    #[test]
    fn wdf_triode_qpoint_reproduces_legacy_loop_and_matches_mna() {
        let graph = parse_graph(VCC_CC_12AX7);
        let nl = triode_nl_kind(&graph, "V1");
        let all_edges: Vec<usize> = (0..graph.edges.len()).collect();

        // The deleted spqr_build loop, replicated verbatim (Rp=100k, Rk=1.5k).
        let supply = 250.0_f64;
        let (r_plate, r_cathode) = (100_000.0_f64, 1_500.0_f64);
        let model = super::super::helpers::triode_model("12ax7");
        let mut triode =
            pedalkernel_rt::elements::nonlinear::TriodeRoot::new_with_v_max(model, supply);
        let mut ia = 1e-4_f64;
        for _ in 0..50 {
            let vgk = -ia * r_cathode;
            let vpk = (supply - ia * r_plate).max(0.0);
            triode.set_vgk(vgk);
            let ia_model = triode.plate_current(vpk);
            let f = ia - ia_model;
            ia = (ia - f * 0.5).max(0.0);
            if f.abs() < 1e-9 {
                break;
            }
        }
        let legacy_vgk = -ia * r_cathode;
        let legacy_v_cathode = ia * r_cathode;

        let wdf = solve_wdf_triode_dc_qpoint(&nl, &all_edges, &graph, supply)
            .expect("WDF entry must solve the vcc grounded-cathode stage");
        assert!(
            (wdf.vgk - legacy_vgk).abs() < 1e-12,
            "WDF vgk={} must reproduce the deleted loop's vgk={}",
            wdf.vgk,
            legacy_vgk
        );
        assert!(
            (wdf.v_cathode - legacy_v_cathode).abs() < 1e-12,
            "WDF v_cathode={} vs legacy {}",
            wdf.v_cathode,
            legacy_v_cathode
        );

        let mna = solve_triode_dc_qpoint(&[nl], &all_edges, &graph, supply)
            .expect("MNA entry must also solve");
        assert!(
            (wdf.vgk - mna.vgk).abs() < 1e-12 && (wdf.vpk - mna.vpk).abs() < 1e-12,
            "WDF and MNA entries share the solver core: wdf=({}, {}) mna=({}, {})",
            wdf.vgk,
            wdf.vpk,
            mna.vgk,
            mna.vpk
        );
    }

    /// B+-railed common-cathode (the 0stg named-rail arm) solves on the WDF
    /// entry too — graph-wide named-rail direct search, no BFS needed.
    #[test]
    fn wdf_triode_qpoint_solves_on_named_bplus_rail() {
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
        let wdf = solve_wdf_triode_dc_qpoint(&nl, &all_edges, &graph, 275.0)
            .expect("B+ common-cathode must solve on the WDF entry (0stg)");
        let mna = solve_triode_dc_qpoint(&[nl], &all_edges, &graph, 275.0)
            .expect("MNA entry must also solve");
        assert!(
            (wdf.vgk - mna.vgk).abs() < 1e-12,
            "same direct topology → same Q-point: wdf={} mna={}",
            wdf.vgk,
            mna.vgk
        );
        assert!(wdf.vgk < -0.5 && wdf.vgk > -5.0, "vgk={}", wdf.vgk);
    }

    /// A cathode follower whose plate ties DIRECTLY to the named rail solves
    /// as the r_plate = 0 load line on the WDF entry.
    #[test]
    fn wdf_triode_qpoint_solves_follower_plate_direct_to_rail() {
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
        let wdf = solve_wdf_triode_dc_qpoint(&nl, &all_edges, &graph, 275.0)
            .expect("B+ follower must solve on the WDF entry");
        assert!(wdf.vgk < -1.0, "12BH7 follower should self-bias strongly negative, got {}", wdf.vgk);
        assert!(
            (wdf.v_cathode - -wdf.vgk).abs() < 1e-9,
            "cathode auto-bias identity: v_cathode == -vgk"
        );
    }

    /// Undeterminable bias (no cathode-to-gnd resistor reachable by the
    /// legacy DIRECT-only WDF finder) is a WARN case: `Undeterminable`, not
    /// `NotApplicable`.  The SAME fixture still locates under the MNA
    /// `DirectThenBfs` flavor — pinning the preserved per-call-site
    /// divergence (the WDF path never had the BFS fallback).
    #[test]
    fn wdf_triode_qpoint_undeterminable_is_warned_not_silent() {
        // Cathode reaches gnd only through TWO series resistors: the direct
        // finder fails, the cap-aware BFS succeeds.
        let graph = parse_graph(
            r#"pedal "test" { supply 250V
                components {
                    V1: triode(12ax7)
                    R_p: resistor(100k)
                    R_k1: resistor(750)
                    R_k2: resistor(750)
                    R_g: resistor(1M)
                }
                nets {
                    vcc -> R_p.a
                    R_p.b -> V1.plate
                    V1.cathode -> R_k1.a
                    R_k1.b -> R_k2.a
                    R_k2.b -> gnd
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

        let err = solve_wdf_triode_dc_qpoint(&nl, &all_edges, &graph, 250.0)
            .expect_err("series-Rk must be undeterminable on the DIRECT-only WDF flavor");
        match err {
            TriodeQpointSkip::Undeterminable(BiasError::UndeterminableTriode {
                ref missing,
                ..
            }) => {
                assert!(
                    matches!(missing, TopologyTerm::CathodeResistor),
                    "should name the missing cathode resistor, got {missing:?}"
                );
            }
            other => panic!("expected Undeterminable(UndeterminableTriode), got {other:?}"),
        }

        // The MNA flavor's BFS arm CAN locate through the series pair — the
        // preserved breadth divergence between the two call-sites.
        let seed = TriodeSeed {
            nl_kind: &nl,
            label: "V1".to_owned(),
            supply_voltage: 250.0,
            parallel_count: 1,
            flavor: TriodeFinderFlavor::DirectThenBfs,
        };
        let topo = seed
            .locate_bias_topology(&all_edges, &graph, &NetworkBias::default(), 250.0)
            .expect("DirectThenBfs must locate via the BFS fallback");
        assert!(
            (topo.r_degeneration - 750.0).abs() < 1e-9,
            "BFS returns the FIRST resistor stepped through (750Ω), got {}",
            topo.r_degeneration
        );
    }

    /// Vari-mu and strapped (grid-less) triodes are NotApplicable on the WDF
    /// entry — silent skip, no warning (ko5g.6 owns vari-mu).
    #[test]
    fn wdf_triode_qpoint_not_applicable_for_varimu_and_strapped() {
        let graph = parse_graph(VCC_CC_12AX7);
        let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
        let plate_node = *graph.node_names.get("V1.plate").expect("plate");
        let cathode_node = *graph.node_names.get("V1.cathode").expect("cathode");
        let grid_node = graph.node_names.get("V1.grid").copied();

        let varimu = NonlinearKind::Triode {
            model_name: "6386".to_owned(),
            plate_node,
            cathode_node,
            grid_node,
            parallel_count: 1,
            is_vari_mu: true,
        };
        assert!(matches!(
            solve_wdf_triode_dc_qpoint(&varimu, &all_edges, &graph, 250.0),
            Err(TriodeQpointSkip::NotApplicable)
        ));

        let strapped = NonlinearKind::Triode {
            model_name: "12ax7".to_owned(),
            plate_node,
            cathode_node,
            grid_node: None,
            parallel_count: 1,
            is_vari_mu: false,
        };
        assert!(matches!(
            solve_wdf_triode_dc_qpoint(&strapped, &all_edges, &graph, 250.0),
            Err(TriodeQpointSkip::NotApplicable)
        ));
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Pentode Q-point (ko5g.5)
    // ═══════════════════════════════════════════════════════════════════════

    fn pentode_nl_kind(graph: &CircuitGraph, comp_id: &str) -> NonlinearKind {
        let e = graph
            .edges
            .iter()
            .find(|e| graph.components[e.comp_idx].id == comp_id)
            .expect("pentode edge");
        let comp = &graph.components[e.comp_idx];
        comp.kind
            .classify_nonlinear(&comp.id, e.node_a, e.node_b, graph.gnd_node, &graph.node_names)
            .expect("classify pentode")
            .0
    }

    fn all_graph_edges(graph: &CircuitGraph) -> Vec<usize> {
        (0..graph.edges.len()).collect()
    }

    /// Gate-4: self-biased Rk + resistive plate load + screen dropper — the
    /// single-ended EL34 shape.  The solved op must be datasheet-plausible
    /// (EL34 SE class-A at 450 V: Vg1 ≈ −30 ± 30 %, Ia tens of mA) and the
    /// screen must resolve through the dropper to the feeding rail (no screen
    /// current in the Koren model → no Ig2·R drop).
    #[test]
    fn pentode_qpoint_self_biased_rk_resistive_load() {
        let graph = parse_graph(
            r#"pedal "test" { supply 450V
                components {
                    V1: pentode(el34)
                    R_p: resistor(3.5k)
                    R_k: resistor(470)
                    R_g: resistor(220k)
                    R_scr: resistor(1k)
                    C_k: cap(100u, electrolytic)
                    C_in: cap(100n)
                }
                nets {
                    in -> C_in.a
                    C_in.b -> V1.grid, R_g.a
                    R_g.b -> gnd
                    vcc -> R_p.a
                    R_p.b -> V1.plate
                    vcc -> R_scr.a
                    R_scr.b -> V1.g2
                    V1.cathode -> R_k.a, C_k.a
                    R_k.b -> gnd
                    C_k.b -> gnd
                    V1.plate -> out
                }
                controls {}
            }"#,
        );
        let nl = pentode_nl_kind(&graph, "V1");
        let all_edges = all_graph_edges(&graph);
        let dc = solve_wdf_pentode_dc_qpoint(&nl, &all_edges, &graph, 450.0)
            .expect("self-biased EL34 must solve");

        assert!(
            (-40.0..=-20.0).contains(&dc.vg1k),
            "EL34 Vg1k {:.3} outside the datasheet-plausible −40..−20 V band",
            dc.vg1k
        );
        assert!(
            (0.04..=0.09).contains(&dc.ia),
            "EL34 Ia {:.4} A outside the plausible 40-90 mA band",
            dc.ia
        );
        assert!(
            (150.0..=300.0).contains(&dc.vpk),
            "EL34 Vpk {:.1} V outside the load-line band",
            dc.vpk
        );
        // Self-bias consistency: Vk = Ia·Rk = −Vg1k.
        assert!((dc.v_cathode - dc.ia * 470.0).abs() < 1e-6);
        assert_eq!(
            dc.vg2k,
            Some(450.0),
            "screen dropper resolves to the feeding rail (no Ig2 in the model)"
        );
    }

    /// Gate-4: the screen resolver must survive the mgsd trap — a bypassed
    /// screen node (≥10 µF cap → AC-ground classified) still resolves to ≈B+
    /// at DC, and a named B+ rail (itself AC-ground) must not read as 0 V.
    #[test]
    fn pentode_qpoint_screen_dropper_resolution_survives_ac_ground() {
        let graph = parse_graph(
            r#"pedal "test" {
                supplies { B+: 330V }
                components {
                    V1: pentode(6v6gt)
                    R_p: resistor(5k)
                    R_k: resistor(250)
                    R_g: resistor(220k)
                    R_scr: resistor(470)
                    C_scr: cap(22u, electrolytic)
                    C_k: cap(25u, electrolytic)
                }
                nets {
                    in -> V1.grid, R_g.a
                    R_g.b -> gnd
                    B+ -> R_p.a
                    R_p.b -> V1.plate
                    B+ -> R_scr.a
                    R_scr.b -> V1.screen, C_scr.a
                    C_scr.b -> gnd
                    V1.cathode -> R_k.a, C_k.a
                    R_k.b -> gnd
                    C_k.b -> gnd
                    V1.plate -> out
                }
                controls {}
            }"#,
        );
        // The actual mgsd trap: the named B+ rail is ITSELF AC-ground
        // classified (graph.rs inserts supply nodes wholesale), so a
        // rail-voltage resolver with the legacy ac-ground-first arm order
        // stamps B+ as 0 V and the whole dropper solves to 0.
        let bplus = graph
            .supply_nodes
            .iter()
            .copied()
            .find(|n| graph.ac_ground_nodes.contains(n))
            .expect("fixture must have an AC-ground-classified named rail");
        assert!(graph.supply_voltages.get(&bplus).copied() == Some(330.0));
        let v = resolve_pentode_screen_dc("V1", &graph, 330.0);
        assert_eq!(
            v,
            Some(330.0),
            "bypassed screen dropper must resolve to B+ at DC, not the \
             AC-ground 0 V (mgsd arm order)"
        );
    }

    /// Gate-4 (THE ko5g.2 la2a-V5 safeguard): a grounded-cathode pentode is a
    /// FIXED-BIAS topology — the solver must DEFER (loud Undeterminable with
    /// the hint-shaped message), never trust the topology's Vg1k = 0 hot
    /// point.
    #[test]
    fn pentode_qpoint_grounded_cathode_defers_loudly() {
        let graph = parse_graph(
            r#"pedal "test" {
                supplies { B+: 275V }
                components {
                    V5: pentode(6aq5a)
                    R_g: resistor(220k)
                    R_scr: resistor(1k)
                    R_p: resistor(10k)
                }
                nets {
                    in -> V5.grid, R_g.a
                    R_g.b -> gnd
                    B+ -> R_p.a
                    R_p.b -> V5.plate
                    B+ -> R_scr.a
                    R_scr.b -> V5.g2
                    V5.cathode -> gnd
                    V5.plate -> out
                }
                controls {}
            }"#,
        );
        let nl = pentode_nl_kind(&graph, "V5");
        let all_edges = all_graph_edges(&graph);
        let out = solve_wdf_pentode_dc_qpoint(&nl, &all_edges, &graph, 275.0);
        let Err(PentodeQpointSkip::Undeterminable(err)) = out else {
            panic!("grounded-cathode pentode must DEFER, got {out:?}");
        };
        assert!(
            matches!(err, BiasError::FixedBiasPentode { ref label } if label == "V5"),
            "expected FixedBiasPentode for V5, got {err:?}"
        );
        let msg = err.into_compile_error();
        assert!(
            msg.contains("cathode resistor") && msg.contains("init { V5:"),
            "deferral message must name both escape hatches, got: {msg}"
        );
    }

    /// Gate-4: push-pull OT primary with its center tap on B+ — the plate DC
    /// path resolves through the winding at DCR (bix9), and the SHARED
    /// cathode resistor is co-solved (r_degeneration = 2·Rk, so
    /// Vk = 2·Ia·Rk).
    #[test]
    fn pentode_qpoint_ot_primary_ct_on_rail_solves_shared_cathode() {
        let graph = parse_graph(
            r#"pedal "test" {
                supplies { B+: 330V }
                components {
                    V3a: pentode(6v6gt)
                    V3b: pentode(6v6gt)
                    R_ga: resistor(220k)
                    R_gb: resistor(220k)
                    R_k: resistor(250)
                    C_k: cap(25u, electrolytic)
                    R_scr: resistor(470)
                    OT: transformer(30:1, 8H, pp)
                    R_spk: resistor(8)
                }
                nets {
                    in -> V3a.grid, R_ga.a
                    R_ga.b -> gnd
                    gnd -> V3b.grid, R_gb.a
                    R_gb.b -> gnd
                    V3a.cathode -> R_k.a, C_k.a
                    V3b.cathode -> R_k.a
                    R_k.b -> gnd
                    C_k.b -> gnd
                    B+ -> R_scr.a
                    R_scr.b -> V3a.screen, V3b.screen
                    B+ -> OT.pri.ct
                    V3a.plate -> OT.pri.a
                    V3b.plate -> OT.pri.b
                    OT.sec.a -> R_spk.a, out
                    OT.sec.b -> gnd
                    R_spk.b -> gnd
                }
                controls {}
            }"#,
        );
        let nl = pentode_nl_kind(&graph, "V3a");
        let all_edges = all_graph_edges(&graph);
        let dc = solve_wdf_pentode_dc_qpoint(&nl, &all_edges, &graph, 330.0)
            .expect("OT-primary-with-ct pentode must solve");

        // r_load = winding DCR (unmodeled → 0) ⇒ plate sits at B+.
        assert_eq!(dc.vpk, 330.0, "OT-coupled plate sits at B+ at DC");
        // Shared cathode: Vk = 2·Ia·Rk (per-tube Ia reported).
        assert!(
            (dc.v_cathode - 2.0 * dc.ia * 250.0).abs() < 1e-6,
            "shared-Rk co-solve: Vk {:.4} != 2·Ia·Rk {:.4}",
            dc.v_cathode,
            2.0 * dc.ia * 250.0
        );
        assert!(
            (-25.0..=-8.0).contains(&dc.vg1k),
            "6V6 push-pull Vg1k {:.3} outside the plausible −25..−8 V band \
             (datasheet cathode bias ≈ −15 V at 330 V)",
            dc.vg1k
        );
        assert!(
            (0.015..=0.05).contains(&dc.ia),
            "6V6 Ia {:.4} A outside the plausible 15-50 mA band",
            dc.ia
        );
        assert_eq!(dc.vg2k, Some(330.0), "shared screen dropper → B+");
    }

    /// Gate-4: an OT primary whose center tap is NOT wired to any rail (the
    /// push_pull_6l6 fixture's shape) must fall back LOUDLY — never silently
    /// mis-solve through the transformer.
    #[test]
    fn pentode_qpoint_ot_primary_floating_ct_falls_back_loudly() {
        let graph = parse_graph(
            r#"pedal "test" { supply 400V
                components {
                    V1: pentode(6l6gc)
                    V2: pentode(6l6gc)
                    R_g1: resistor(220k)
                    R_g2: resistor(220k)
                    R_k: resistor(250)
                    T1: transformer(25:1, 10H, pp)
                    RL: resistor(8)
                }
                nets {
                    in -> V1.g1, R_g1.a
                    R_g1.b -> gnd
                    gnd -> V2.g1, R_g2.a
                    R_g2.b -> gnd
                    V1.cathode -> R_k.a
                    V2.cathode -> R_k.a
                    R_k.b -> gnd
                    V1.plate -> T1.a
                    V2.plate -> T1.b
                    T1.c -> RL.a, out
                    T1.d -> gnd
                    RL.b -> gnd
                }
                controls {}
            }"#,
        );
        let nl = pentode_nl_kind(&graph, "V1");
        let all_edges = all_graph_edges(&graph);
        let out = solve_wdf_pentode_dc_qpoint(&nl, &all_edges, &graph, 400.0);
        assert!(
            matches!(
                out,
                Err(PentodeQpointSkip::Undeterminable(
                    BiasError::UndeterminablePentode {
                        missing: TopologyTerm::PlateDcPath,
                        ..
                    }
                ))
            ),
            "floating-ct OT primary must be a LOUD PlateDcPath fallback, got {out:?}"
        );
    }

    /// Gate-4: a two-terminal transformer primary between B+ and the plate
    /// (the la2a EL_drive shape, degenerate series inductor) is a DC short —
    /// the inductive walk resolves the plate to B+ through the winding.
    #[test]
    fn pentode_qpoint_two_terminal_inductive_primary_reaches_rail() {
        let graph = parse_graph(
            r#"pedal "test" {
                supplies { B+: 275V }
                components {
                    V5: pentode(6aq5a)
                    R_g: resistor(220k)
                    R_k: resistor(470)
                    R_scr: resistor(1k)
                    EL_drive: transformer(1:2, 8H)
                }
                nets {
                    in -> V5.grid, R_g.a
                    R_g.b -> gnd
                    B+ -> R_scr.a
                    R_scr.b -> V5.g2
                    B+ -> EL_drive.a
                    V5.plate -> EL_drive.b
                    V5.cathode -> R_k.a
                    R_k.b -> gnd
                    EL_drive.c -> out
                    EL_drive.d -> gnd
                }
                controls {}
            }"#,
        );
        let nl = pentode_nl_kind(&graph, "V5");
        let all_edges = all_graph_edges(&graph);
        let dc = solve_wdf_pentode_dc_qpoint(&nl, &all_edges, &graph, 275.0)
            .expect("plate through a 2-terminal primary to B+ must solve");
        assert_eq!(dc.vpk, 275.0, "inductive plate path → plate at B+ at DC");
        assert!(dc.vg1k < 0.0 && dc.ia > 0.0);
        assert_eq!(dc.vg2k, Some(275.0));
    }

    /// Gate-4: a netlist that never wires the screen pin keeps the MODEL
    /// default `vg2k` (`PentodeDcQpoint::vg2k = None`) — the declared op of
    /// the screen-unwired test fixtures.
    #[test]
    fn pentode_qpoint_unwired_screen_keeps_model_default() {
        let graph = parse_graph(
            r#"pedal "test" { supply 300V
                components {
                    V1: pentode(el84)
                    R_p: resistor(100k)
                    R_k: resistor(470)
                    R_g: resistor(1M)
                }
                nets {
                    in -> V1.grid, R_g.a
                    R_g.b -> gnd
                    vcc -> R_p.a
                    R_p.b -> V1.plate
                    V1.cathode -> R_k.a
                    R_k.b -> gnd
                    V1.plate -> out
                }
                controls {}
            }"#,
        );
        let nl = pentode_nl_kind(&graph, "V1");
        let all_edges = all_graph_edges(&graph);
        let dc = solve_wdf_pentode_dc_qpoint(&nl, &all_edges, &graph, 300.0)
            .expect("EL84 with resistive load must solve");
        assert_eq!(
            dc.vg2k, None,
            "unwired screen → None → the model default stands at the call-site"
        );
        assert!(dc.vg1k < 0.0 && dc.vpk > 0.0);
    }

    /// Gate-4: non-pentode kinds and grid-less pentodes are NotApplicable
    /// (silent), and a missing cathode resistor is a loud Undeterminable.
    #[test]
    fn pentode_qpoint_not_applicable_and_missing_rk() {
        let graph = parse_graph(
            r#"pedal "test" { supply 300V
                components {
                    V1: pentode(el84)
                    R_p: resistor(100k)
                    R_g: resistor(1M)
                    C_k: cap(100u, electrolytic)
                }
                nets {
                    in -> V1.grid, R_g.a
                    R_g.b -> gnd
                    vcc -> R_p.a
                    R_p.b -> V1.plate
                    V1.cathode -> C_k.a
                    C_k.b -> gnd
                    V1.plate -> out
                }
                controls {}
            }"#,
        );
        let nl = pentode_nl_kind(&graph, "V1");
        let all_edges = all_graph_edges(&graph);

        // Cathode floats behind a cap (no Rk anywhere): loud Undeterminable.
        let out = solve_wdf_pentode_dc_qpoint(&nl, &all_edges, &graph, 300.0);
        assert!(
            matches!(
                out,
                Err(PentodeQpointSkip::Undeterminable(
                    BiasError::UndeterminablePentode {
                        missing: TopologyTerm::CathodeResistor,
                        ..
                    }
                ))
            ),
            "cap-isolated cathode without Rk must warn, got {out:?}"
        );

        // Grid-less pentode: silent NotApplicable.
        let (plate_node, cathode_node) = match &nl {
            NonlinearKind::Pentode {
                plate_node,
                cathode_node,
                ..
            } => (*plate_node, *cathode_node),
            _ => unreachable!(),
        };
        let strapped = NonlinearKind::Pentode {
            model_name: "el84".to_owned(),
            plate_node,
            cathode_node,
            grid_node: None,
        };
        assert!(matches!(
            solve_wdf_pentode_dc_qpoint(&strapped, &all_edges, &graph, 300.0),
            Err(PentodeQpointSkip::NotApplicable)
        ));

        // Non-pentode kind through the grouped entry: silent NotApplicable.
        let triode = NonlinearKind::Triode {
            model_name: "12ax7".to_owned(),
            plate_node,
            cathode_node,
            grid_node: None,
            parallel_count: 1,
            is_vari_mu: false,
        };
        assert!(matches!(
            solve_pentode_dc_qpoint(&[triode], &all_edges, &graph, 300.0),
            Err(PentodeQpointSkip::NotApplicable)
        ));
    }

    // ── pedalkernel-onu2: DC-closure for the general-MNA BJT group solve ─────

    /// An NPN emitter follower whose base bias divider (`vref`, bypassed by a
    /// large cap → ac-ground-classified) lives in ANOTHER flow group — the
    /// uberdrive / LGSM (SD-1 / TS-808) input-buffer shape (RCA GAP 4a on
    /// pedalkernel-a5ho). Pre-onu2 the group-local DC solve saw neither the
    /// divider (other group) nor a real `vref` voltage (ac-ground arm reads
    /// 0 V), converged the follower in CUTOFF, and the stage was silent from
    /// sample 0. With the DC-closure edge set + vref-class interior override +
    /// cross-group q-point seed the follower must pass AC at roughly unity.
    ///
    /// The extra `R5`/`C4` and `RL→gnd` legs make `vref` hit the ≥3-signal-edge
    /// arm of `compute_ac_ground_nodes` (graph.rs) so the test exercises the
    /// vref-class override, not just the plain-interior closure.
    #[test]
    fn follower_with_cross_group_vref_divider_passes_signal() {
        let src = r#"pedal "follower_probe" {
  supply 9V
  components {
    R100: resistor(33k)
    R101: resistor(33k)
    C101: cap(47u, electrolytic)
    R2: resistor(10k)
    C1: cap(47n)
    R3: resistor(470k)
    Q1: npn(2n5088)
    R4: resistor(10k)
    C2: cap(1u, electrolytic)
    RL: resistor(100k)
    R5: resistor(100k)
    C4: cap(18n)
    R6: resistor(100k)
    C5: cap(18n)
  }
  nets {
    vcc -> R100.a
    R100.b -> R101.a, C101.a, vref
    R101.b -> gnd
    C101.b -> gnd
    in -> R2.a
    R2.b -> C1.a
    C1.b -> Q1.base, R3.a
    R3.b -> vref
    vcc -> Q1.collector
    Q1.emitter -> R4.a, C2.a
    R4.b -> gnd
    C2.b -> RL.a, out
    RL.b -> gnd
    R5.a -> vref
    R5.b -> C4.a
    C4.b -> gnd
    R6.a -> vref
    R6.b -> C5.a
    C5.b -> gnd
  }
}"#;
        let pedal = crate::dsl::parse_pedal_file(src).expect("parse follower probe");
        let graph = CircuitGraph::from_pedal(&pedal);

        // The vref net must actually be ac-ground-classified (the premise that
        // makes this the vref-CLASS case) and vref-class per the new predicate.
        let vref_node = *graph.node_names.get("vref").expect("vref node exists");
        assert!(
            graph.ac_ground_nodes.contains(&vref_node),
            "test premise: vref must be ac-ground-classified (bypassed divider)"
        );
        assert!(
            vref_class_node(vref_node, &graph),
            "vref must be vref-class (ac-ground artifact, no declared DC)"
        );

        use pedalkernel_rt::PedalProcessor as _;
        let mut proc = compile_via_spqr(&pedal, SR).expect("compile follower probe");
        let n = (SR as usize) / 2;
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            let x = 0.05 * (2.0 * core::f64::consts::PI * 1000.0 * i as f64 / SR).sin();
            out.push(proc.process(x));
        }
        // Steady-state window (transient + coupling caps settled).
        let tail = &out[n / 2..];
        let rms = (tail.iter().map(|v| v * v).sum::<f64>() / tail.len() as f64).sqrt();
        let mean = tail.iter().sum::<f64>() / tail.len() as f64;
        // Input RMS = 0.05/√2 ≈ 0.0354; an emitter follower is ~unity. The
        // cutoff-starved artifact read ~0 here. Loose band: alive and
        // follower-scaled, not a precision gate.
        assert!(
            rms > 0.015,
            "cross-group-vref follower must pass AC (steady-state rms {rms:.6} ≤ 0.015 → \
             quiescent-dead; pre-onu2 artifact was ~0)"
        );
        assert!(
            rms < 0.1,
            "follower output should stay follower-scaled (rms {rms:.6} ≥ 0.1)"
        );
        assert!(
            mean.abs() < 0.5,
            "AC-coupled output should carry no large DC offset (mean {mean:.6})"
        );
    }
}
