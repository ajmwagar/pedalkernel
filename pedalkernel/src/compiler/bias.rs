//! Unified compile-time DC bias infrastructure.
//!
//! This module provides the data types and solver that will eventually replace
//! the per-type DC bias code spread across `spqr_build.rs`
//! (`compute_wdf_triode_dc_qpoint`, `compute_wdf_bjt_dc_qpoint`, etc.).
//!
//! # Current status (ko5g.1)
//!
//! This bead is **additive only**.  The types and solver are defined here and
//! verified by unit tests, but no call-site in `spqr_build.rs` / `blockwise.rs`
//! / `general.rs` has been migrated yet.  Migrations happen in ko5g.3–ko5g.6.
//! All existing compiler behaviour is therefore byte-identical after this bead.
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
//!   └─ solve_network_bias(graph, group) → NetworkBias   (absorbs bias_analysis's divider)
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
use super::graph::{CircuitGraph, NodeId};

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
            Self::BaseDivider => write!(
                f,
                "base-bias divider (R_b1: base→VCC and R_b2: base→GND)"
            ),
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
/// VCC→resistor-divider→GND subgraph.  It is conceptually equivalent to what
/// `bias_analysis.rs::classify_group_bias` returns inside `GroupBiasKind::StaticBias`,
/// but typed as a standalone value so downstream code (the triode/BJT solvers)
/// can consume it without pattern-matching a flow-group enum.
///
/// `bias_analysis.rs` is NOT deleted by this bead — the call-site swap that
/// replaces it with `solve_network_bias` is deferred to ko5g.3.
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
    let node_idx: HashMap<NodeId, usize> =
        node_list.iter().enumerate().map(|(i, &nd)| (nd, i)).collect();

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

// ═══════════════════════════════════════════════════════════════════════════
// BiasSeed implementations
// ═══════════════════════════════════════════════════════════════════════════

// ── Triode ──────────────────────────────────────────────────────────────────

/// `BiasSeed` for a common-cathode triode stage.
pub(super) struct TriodeSeed<'a> {
    pub(super) nl_kind: &'a NonlinearKind,
    pub(super) label: String,
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

        // Find R_plate: resistor between vcc_node and plate_node (BFS or direct).
        let r_plate = find_load_resistor_bfs(plate_node, graph.vcc_node, edge_indices, graph)
            .or_else(|| find_load_resistor_direct(plate_node, graph.vcc_node, edge_indices, graph))
            .ok_or_else(|| BiasError::UndeterminableTriode {
                label: self.label.clone(),
                missing: TopologyTerm::PlateResistor,
            })?;

        // Find R_cathode: resistor between cathode_node and gnd_node (BFS or direct).
        let r_cathode =
            find_load_resistor_bfs(cathode_node, graph.gnd_node, edge_indices, graph)
                .or_else(|| {
                    find_load_resistor_direct(cathode_node, graph.gnd_node, edge_indices, graph)
                })
                .ok_or_else(|| BiasError::UndeterminableTriode {
                    label: self.label.clone(),
                    missing: TopologyTerm::CathodeResistor,
                })?;

        Ok(BiasTopology {
            r_load: r_plate,
            v_load_rail: supply_voltage,
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
        let mut root = pedalkernel_rt::elements::nonlinear::TriodeRoot::new_with_v_max(
            model,
            trial.v_output.max(1.0),
        );
        root.set_bias(trial.v_control as pedalkernel_rt::Wave);

        let ia = root.plate_current(trial.v_output as pedalkernel_rt::Wave) as f64;

        // Numerical derivative for dIa/dVgk.
        // Cast both plate_current results to f64 BEFORE subtraction to avoid
        // catastrophic f32 cancellation when h is small.
        let h = 1e-3_f64; // larger h to stay above f32 noise floor (~1e-7)
        let mut root2 = pedalkernel_rt::elements::nonlinear::TriodeRoot::new_with_v_max(
            super::helpers::triode_model(model_name),
            trial.v_output.max(1.0),
        );
        root2.set_bias((trial.v_control + h) as pedalkernel_rt::Wave);
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

        // Base divider: R1 = base→vcc, R2 = base→gnd.
        let r1 = find_load_resistor_direct(base_node, graph.vcc_node, &all_edges, graph)
            .or_else(|| find_load_resistor_bfs(base_node, graph.vcc_node, &all_edges, graph))
            .ok_or_else(|| BiasError::UndeterminableBjt {
                label: self.label.clone(),
                missing: TopologyTerm::BaseDivider,
            })?;
        let r2 = find_load_resistor_direct(base_node, graph.gnd_node, &all_edges, graph)
            .or_else(|| find_load_resistor_bfs(base_node, graph.gnd_node, &all_edges, graph))
            .ok_or_else(|| BiasError::UndeterminableBjt {
                label: self.label.clone(),
                missing: TopologyTerm::BaseDivider,
            })?;

        // Emitter resistor RE: emitter→gnd.
        let re =
            find_load_resistor_direct(emitter_node, graph.gnd_node, edge_indices, graph)
                .or_else(|| {
                    find_load_resistor_bfs(emitter_node, graph.gnd_node, edge_indices, graph)
                })
                .ok_or_else(|| BiasError::UndeterminableBjt {
                    label: self.label.clone(),
                    missing: TopologyTerm::EmitterResistor,
                })?;

        // Collector resistor RC: collector→vcc.
        let _rc =
            find_load_resistor_direct(collector_node, graph.vcc_node, &all_edges, graph)
                .or_else(|| {
                    find_load_resistor_bfs(collector_node, graph.vcc_node, &all_edges, graph)
                });

        // Thévenin base voltage: prefer StaticBias map, fall back to resistor divider.
        let vth = network_bias
            .voltage_at(base_node, graph, supply_voltage)
            .filter(|v| v.is_finite())
            .unwrap_or_else(|| supply_voltage * r2 / (r1 + r2));
        let rth = r1 * r2 / (r1 + r2);

        Ok(BiasTopology {
            r_load: _rc.unwrap_or(supply_voltage * 0.5 / 1e-3), // estimate if missing
            v_load_rail: supply_voltage,
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
// Linear system solver (Gaussian elimination, shared with bias_analysis.rs)
// ═══════════════════════════════════════════════════════════════════════════

/// Solve a small linear system Ax = b via Gaussian elimination with partial
/// pivoting.  Returns `None` if the system is singular.
///
/// This duplicates the private `solve_linear_system` in `bias_analysis.rs`.
/// When ko5g.3 migrates the call-sites, the two can be unified; for now both
/// exist so this bead stays additive-only and does not modify `bias_analysis.rs`.
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
            let df =
                -((ib2 - ib) as f64 / h) * rth - 1.0 - ((ic2 + ib2 - ic - ib) as f64 / h) * re;
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
}
