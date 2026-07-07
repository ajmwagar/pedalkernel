//! Boundary-load bindings: compile-time ANALYSIS of what hangs electrically
//! downstream of a stage boundary, separated from the POLICY of what the
//! compiler does about it.
//!
//! WHY: downstream impedance never reflects back into an upstream stage's
//! scattering — stages solve against an open circuit. Proven on the BA283:
//! ngspice with RL=1G reproduces the unloaded WDF to 3–4 decimals (the engine
//! is exact at the topology it simulates; the topology was wrong), and the
//! loaded golden's −16.1 dB THD is Q1/Q2 cutoff clipping from load-current
//! demand the open-circuit topology never delivered. Fusing the trailing
//! `Cout→RL` output network into the solved MNA closed it: LEVEL −0.05 dB,
//! ΔTHD 0.4 dB, tilt 0.79 dB (servo-off default).
//!
//! This module owns the two halves that used to be entangled in
//! `spqr_build::trailing_output_load_group_for_fusion`:
//!
//! * [`analyze_trailing_output_load`] — pure SHAPE analysis of the trailing
//!   output flow group. No device gate, no DC solve, no env check: it answers
//!   "what hangs off this stage's output boundary?", producing a
//!   [`BoundaryLoadModel`].
//! * [`gate_ba283_fusion`] — the POLICY gate deciding whether the analyzed
//!   load may be fused into the upstream stage's general-MNA build. Exactly
//!   the consolidated dc_qpoint seed + servo shape (`apply_bjt_dc_qpoint`):
//!   every NL device in the target is a true 3-terminal BJT, there are at
//!   least two, `solve_bjt_group_dc_qpoint` produces the op-point seed, and
//!   `PK_OUTPUT_FUSE_DISABLE` is not set. Declines carry a static reason
//!   string for the decision table.
//!
//! The outcome of analysis + policy is a [`LoadDisposition`], and every
//! analyzed boundary (fused or declined) is recorded on
//! `CompiledPedal::boundary_loads` as a serializable
//! [`BoundaryLoadBinding`](pedalkernel_rt::processor::BoundaryLoadBinding)
//! (component ids + values; compile-time edge indices never leave the
//! compiler) so dashboards can surface unloaded output boundaries without
//! recompiling.
//!
//! SCOPE (deliberate): these are planner-layer types. The neutral network
//! vocabulary (`LinearMultiportNetwork`, `boundary_math`) stays load-free —
//! bindings and drive policy live at the stage-planning layer, mirroring the
//! `StageBinding`/`ControlTarget` precedent.

use pedalkernel_rt::processor::{
    BoundaryLoadBinding, BoundaryLoadDisposition, BoundaryLoadSummary,
};

use super::boundary_rules::DelayedCutSet;
use super::classify::NonlinearKind;
use super::component::EdgeKind;
use super::graph::{CircuitGraph, NodeId};
use super::signal_flow::FlowGroup;

/// Disposition reason: analysis found no trailing output-load group to fuse
/// (boundary is open, or the boundary shape is not recognized).
pub(super) const REASON_NO_TRAILING_LOAD: &str = "analysis:no-trailing-output-load";

/// ANALYSIS: what hangs electrically downstream of a stage boundary node.
///
/// Compiler-internal — `edges` are indices into `graph.edges` and are only
/// meaningful during compilation. [`BoundaryLoadModel::summarize`] flattens a
/// model into the serializable [`BoundaryLoadSummary`] stored on
/// `CompiledPedal`.
#[derive(Debug, Clone, PartialEq)]
pub(super) enum BoundaryLoadModel {
    /// Nothing recognized downstream (or analysis declined) — boundary is open.
    #[allow(dead_code)] // emitted by future boundary analyses; summary-side used today
    Unloaded,
    /// Grounded resistive load(s) directly at the boundary node.
    #[allow(dead_code)] // future widening (pot loads at `out`, lkf1.3)
    GroundedResistive { r_total: f64, edges: Vec<usize> },
    /// One series coupling cap into grounded resistive load(s) — the classic
    /// `Cout→RL` output network (the BA283 shape).
    SeriesCapIntoLoad {
        c: f64,
        r_total: f64,
        edges: Vec<usize>,
    },
    /// General passive network (future widening; analysis may emit it, no
    /// policy consumes it yet).
    #[allow(dead_code)]
    PassiveNetwork { edges: Vec<usize> },
}

impl BoundaryLoadModel {
    /// All compile-time edge indices of the modeled load network.
    pub(super) fn edges(&self) -> &[usize] {
        match self {
            BoundaryLoadModel::Unloaded => &[],
            BoundaryLoadModel::GroundedResistive { edges, .. } => edges,
            BoundaryLoadModel::SeriesCapIntoLoad { edges, .. } => edges,
            BoundaryLoadModel::PassiveNetwork { edges } => edges,
        }
    }

    /// Flatten to the serializable dashboard summary: kind + values +
    /// sorted/deduped component id strings (NOT edge indices).
    pub(super) fn summarize(&self, graph: &CircuitGraph) -> BoundaryLoadSummary {
        let component_ids = |edges: &[usize]| -> Vec<String> {
            let mut ids: Vec<String> = edges
                .iter()
                .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.clone())
                .collect();
            ids.sort();
            ids.dedup();
            ids
        };
        match self {
            BoundaryLoadModel::Unloaded => BoundaryLoadSummary::Unloaded,
            BoundaryLoadModel::GroundedResistive { r_total, edges } => {
                BoundaryLoadSummary::GroundedResistive {
                    r_total: *r_total,
                    component_ids: component_ids(edges),
                }
            }
            BoundaryLoadModel::SeriesCapIntoLoad { c, r_total, edges } => {
                BoundaryLoadSummary::SeriesCapIntoLoad {
                    c: *c,
                    r_total: *r_total,
                    component_ids: component_ids(edges),
                }
            }
            BoundaryLoadModel::PassiveNetwork { edges } => BoundaryLoadSummary::PassiveNetwork {
                component_ids: component_ids(edges),
            },
        }
    }
}

/// POLICY OUTCOME: what the compiler did about the analyzed load.
#[derive(Debug, Clone, PartialEq)]
pub(super) enum LoadDisposition {
    /// Load edges fused into the upstream stage's solved network; the
    /// standalone downstream stage was consumed (removed).
    FusedUpstream { consumed_group: usize },
    /// Nothing done — boundary left open, with the reason (gate name / env
    /// opt-out / shape mismatch / no load found), for dashboards and triage.
    Unloaded { reason: &'static str },
}

impl LoadDisposition {
    /// The flow-group index consumed by fusion, if any.
    pub(super) fn fused_group(&self) -> Option<usize> {
        match self {
            LoadDisposition::FusedUpstream { consumed_group } => Some(*consumed_group),
            LoadDisposition::Unloaded { .. } => None,
        }
    }

    /// Flatten to the serializable dashboard summary.
    pub(super) fn summarize(&self) -> BoundaryLoadDisposition {
        match self {
            LoadDisposition::FusedUpstream { .. } => BoundaryLoadDisposition::FusedUpstream,
            LoadDisposition::Unloaded { reason } => BoundaryLoadDisposition::Unloaded {
                reason: (*reason).to_string(),
            },
        }
    }
}

/// Result of [`analyze_trailing_output_load`]: the trailing output flow group,
/// the boundary node the load hangs off, and the load model.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct TrailingOutputLoad {
    /// Index (into the flow-group slice) of the trailing passive output group.
    pub group: usize,
    /// The TARGET-group node the load network attaches to (the coupling cap's
    /// stage-side node) — the upstream stage's output boundary.
    pub boundary_node: NodeId,
    /// What hangs downstream of `boundary_node`.
    pub model: BoundaryLoadModel,
}

/// ANALYSIS ONLY — find the TRAILING OUTPUT GROUP downstream of `target_gi`'s
/// output boundary: the passive `{Cout, RL}` flow group carrying the
/// amplifier's load.
///
/// The trailing group must be EXACTLY: one coupling cap bridging a
/// target-group node to the global `out`, plus ≥1 grounded fixed (non-pot)
/// load resistors at `out` — nothing else (no pots, no actives, no second
/// cap), and no broker-cut (Phase 2a `DelayedCutSet`) edges: never fuse across
/// a delayed-coupling boundary.
///
/// No device-shape gate, no DC solve, no env check here — that is POLICY
/// ([`gate_ba283_fusion`]). Returns `None` when the boundary is open or the
/// shape is not recognized.
pub(super) fn analyze_trailing_output_load(
    target_gi: usize,
    groups: &[FlowGroup],
    graph: &CircuitGraph,
    cut_edges: &DelayedCutSet,
) -> Option<TrailingOutputLoad> {
    let target = &groups[target_gi];
    let out = graph.out_node;
    if out == graph.gnd_node || out == graph.vcc_node {
        return None;
    }
    let is_gnd = |n: NodeId| n == graph.gnd_node || graph.ac_ground_nodes.contains(&n);

    // Target node set; the global `out` must NOT already be inside (there must
    // BE a trailing group to fuse).
    let mut target_nodes = std::collections::HashSet::new();
    for &eidx in &target.all_edges() {
        let e = &graph.edges[eidx];
        target_nodes.insert(e.node_a);
        target_nodes.insert(e.node_b);
    }
    if target_nodes.contains(&out) {
        return None;
    }

    // The trailing output group: a passive group that is EXACTLY one coupling
    // cap (target-node -> out) plus grounded fixed load resistor(s) at out.
    for (src_gi, source) in groups.iter().enumerate() {
        if src_gi == target_gi || !source.active_edges.is_empty() {
            continue;
        }
        let edges = source.all_edges();
        if edges.len() < 2 {
            continue;
        }
        // Phase 2a: never fuse across a broker-cut boundary.
        if edges.iter().any(|eidx| cut_edges.cuts.contains_key(eidx)) {
            continue;
        }
        let mut bridge: Option<(NodeId, f64)> = None; // (stage-side node, C)
        let mut load_conductance = 0.0f64;
        let mut n_loads = 0usize;
        let mut shape_ok = true;
        for &eidx in &edges {
            let e = &graph.edges[eidx];
            let comp = &graph.components[e.comp_idx];
            let touches_out = e.node_a == out || e.node_b == out;
            match graph.effective_edge_kind(eidx) {
                // The output coupling cap: `out` on one side, a target-group
                // node on the other. Exactly one.
                EdgeKind::Reactive
                    if comp.kind.capacitance().is_some() && touches_out && bridge.is_none() =>
                {
                    let interior = if e.node_a == out { e.node_b } else { e.node_a };
                    if target_nodes.contains(&interior) {
                        bridge = Some((interior, comp.kind.capacitance().unwrap_or(0.0)));
                    } else {
                        shape_ok = false;
                    }
                }
                // A grounded fixed load resistor at `out` (not a pot).
                EdgeKind::Linear
                    if comp.kind.resistance().is_some()
                        && !comp.kind.is_pot()
                        && touches_out
                        && (is_gnd(e.node_a) || is_gnd(e.node_b)) =>
                {
                    n_loads += 1;
                    if let Some(r) = comp.kind.resistance() {
                        if r > 0.0 {
                            load_conductance += 1.0 / r;
                        }
                    }
                }
                _ => {
                    shape_ok = false;
                }
            }
            if !shape_ok {
                break;
            }
        }
        if shape_ok && n_loads >= 1 {
            if let Some((boundary_node, c)) = bridge {
                let r_total = if load_conductance > 0.0 {
                    1.0 / load_conductance
                } else {
                    f64::INFINITY
                };
                return Some(TrailingOutputLoad {
                    group: src_gi,
                    boundary_node,
                    model: BoundaryLoadModel::SeriesCapIntoLoad { c, r_total, edges },
                });
            }
        }
    }
    None
}

/// POLICY gate for BA283-class output-load fusion — gated exactly like the
/// consolidated dc_qpoint seed + servo (`apply_bjt_dc_qpoint`):
///
/// * `PK_OUTPUT_FUSE_DISABLE` is the opt-out escape hatch, mirroring
///   `PK_SERVO_DISABLE`;
/// * the target's NL devices must be ≥2 true 3-terminal BJTs and NOTHING else
///   (the `all_resolved` ∧ `n_bjt ≥ 2` shape);
/// * `solve_bjt_group_dc_qpoint` must actually produce the op-point seed —
///   the same call the general-MNA builder makes before `apply_bjt_dc_qpoint`
///   engages.
///
/// DC is unaffected by the fusion by construction: the coupling cap blocks DC,
/// the DC solve sees the load node only through the grounded resistor
/// (v(out) = 0), and the cap's DC seed charges to the amplifier's output-node
/// voltage exactly like every other coupling cap in `apply_bjt_dc_qpoint`.
///
/// Returns `Err(reason)` when the gate declines; the reason lands in the
/// `CompiledPedal::boundary_loads` decision table.
pub(super) fn gate_ba283_fusion(
    target: &FlowGroup,
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Result<(), &'static str> {
    // Opt-out escape hatch, mirroring PK_SERVO_DISABLE.
    if std::env::var("PK_OUTPUT_FUSE_DISABLE").is_ok() {
        return Err("env:PK_OUTPUT_FUSE_DISABLE");
    }

    // Device shape: every NL device in the target is a true 3-terminal BJT
    // (base != collector) and there are at least two — the exact shape
    // `apply_bjt_dc_qpoint` requires before it engages the consolidated
    // seed + servo (`all_resolved` ∧ `n_bjt >= 2`).
    let mut nl_kinds: Vec<NonlinearKind> = Vec::new();
    let mut seen_comps: std::collections::HashSet<usize> = std::collections::HashSet::new();
    for &eidx in &target.all_edges() {
        if graph.effective_edge_kind(eidx) != EdgeKind::Nonlinear {
            continue;
        }
        let e = &graph.edges[eidx];
        if !seen_comps.insert(e.comp_idx) {
            continue;
        }
        let comp = &graph.components[e.comp_idx];
        let Some((kind, _)) = comp.kind.classify_nonlinear(
            &comp.id,
            e.node_a,
            e.node_b,
            graph.gnd_node,
            &graph.node_names,
        ) else {
            return Err("gate:unclassified-nonlinear");
        };
        let is_three_terminal_bjt = matches!(
            &kind,
            NonlinearKind::BjtNpn { base_node, collector_node, .. }
            | NonlinearKind::BjtPnp { base_node, collector_node, .. }
                if base_node != collector_node
        );
        if !is_three_terminal_bjt {
            return Err("gate:non-bjt-nonlinear");
        }
        nl_kinds.push(kind);
    }
    if nl_kinds.len() < 2 {
        return Err("gate:fewer-than-two-bjts");
    }

    // The dc_qpoint path itself: the compile-time nonlinear DC solve must
    // produce the op-point seed for this group — the same call the
    // general-MNA builder makes before `apply_bjt_dc_qpoint` engages.
    if super::bias::solve_bjt_group_dc_qpoint(&nl_kinds, &target.all_edges(), graph, supply_voltage)
        .is_none()
    {
        return Err("gate:no-dc-qpoint-seed");
    }

    Ok(())
}

/// A boundary-load decision recorded during stage building, before final
/// stage ordering is known. `stage_key` is the stage's minimum stable
/// component id — the same deterministic identity the stage sort uses — and
/// is resolved to a final `CompiledPedal::stages` index by
/// [`resolve_boundary_load_bindings`].
pub(super) struct PendingBoundaryLoad {
    pub stage_key: String,
    pub boundary_node: NodeId,
    pub model: BoundaryLoadSummary,
    pub disposition: BoundaryLoadDisposition,
}

/// Resolve pending boundary-load records against the FINAL stage order.
///
/// `stage_min_comp_id[i]` must be the minimum stable component id of
/// `stages[i]` after the last permutation sort (spqr_build keeps that table
/// aligned through both sorts). Unresolvable keys (stage consumed after
/// recording — not expected in practice) keep the record with
/// `upstream_stage == usize::MAX` rather than dropping the analysis.
pub(super) fn resolve_boundary_load_bindings(
    pending: Vec<PendingBoundaryLoad>,
    stage_min_comp_id: &[String],
) -> Vec<BoundaryLoadBinding> {
    pending
        .into_iter()
        .map(|p| BoundaryLoadBinding {
            upstream_stage: stage_min_comp_id
                .iter()
                .position(|id| *id == p.stage_key)
                .unwrap_or(usize::MAX),
            boundary_node: p.boundary_node,
            model: p.model,
            disposition: p.disposition,
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// BA283-shaped fixture: two DC-coupled NPN common-emitter stages — Q1
    /// self-biased via collector→base shunt feedback (Rf, a cycle through the
    /// active device, so the pair lands in ONE feedback flow group), Q2
    /// DC-coupled from Q1's collector — with the classic trailing output
    /// network `Cout → {RL ‖ gnd}` at the global `out`.
    const TWO_BJT_FB_AMP: &str = r#"
        pedal "two bjt fb amp" { supply 9V
            components {
                Cin:  cap(10u, electrolytic)
                Rpd:  resistor(1M)
                Rc1:  resistor(33k)
                Q1:   npn(2n3904)
                Rf:   resistor(470k)
                Rc2:  resistor(5.6k)
                Q2:   npn(2n3904)
                Re2:  resistor(1k)
                Cout: cap(10u, electrolytic)
                RL:   resistor(10k)
            }
            nets {
                in -> Cin.a, Rpd.a
                Rpd.b -> gnd
                Cin.b -> Q1.base, Rf.b
                vcc -> Rc1.a
                Rc1.b -> Q1.collector, Rf.a, Q2.base
                Q1.emitter -> gnd
                vcc -> Rc2.a
                Rc2.b -> Q2.collector, Cout.a
                Q2.emitter -> Re2.a
                Re2.b -> gnd
                Cout.b -> RL.a, out
                RL.b -> gnd
            }
        }"#;

    /// Same output network, but the target group holds ONE BJT (shunt
    /// collector→base feedback) — fails the ≥2-BJT policy gate.
    const ONE_BJT_FB_AMP: &str = r#"
        pedal "one bjt fb amp" { supply 9V
            components {
                Cin:  cap(10u, electrolytic)
                Rpd:  resistor(1M)
                Rc1:  resistor(33k)
                Q1:   npn(2n3904)
                Re1:  resistor(1k)
                Rf:   resistor(470k)
                Cout: cap(10u, electrolytic)
                RL:   resistor(10k)
            }
            nets {
                in -> Cin.a, Rpd.a
                Rpd.b -> gnd
                Cin.b -> Q1.base, Rf.b
                vcc -> Rc1.a
                Rc1.b -> Q1.collector, Rf.a, Cout.a
                Q1.emitter -> Re1.a
                Re1.b -> gnd
                Cout.b -> RL.a, out
                RL.b -> gnd
            }
        }"#;

    /// Trailing group holds a POT at `out` instead of a fixed load resistor —
    /// analysis must decline the shape (pot loads are lkf1.3 scope).
    const POT_LOAD_AMP: &str = r#"
        pedal "pot load amp" { supply 9V
            components {
                Cin:  cap(10u, electrolytic)
                Rpd:  resistor(1M)
                Rc1:  resistor(33k)
                Q1:   npn(2n3904)
                Rf:   resistor(470k)
                Rc2:  resistor(5.6k)
                Q2:   npn(2n3904)
                Re2:  resistor(1k)
                Cout: cap(10u, electrolytic)
                RL:   pot(10k)
            }
            nets {
                in -> Cin.a, Rpd.a
                Rpd.b -> gnd
                Cin.b -> Q1.base, Rf.b
                vcc -> Rc1.a
                Rc1.b -> Q1.collector, Rf.a, Q2.base
                Q1.emitter -> gnd
                vcc -> Rc2.a
                Rc2.b -> Q2.collector, Cout.a
                Q2.emitter -> Re2.a
                Re2.b -> gnd
                Cout.b -> RL.a, out
                RL.w -> gnd
                RL.b -> gnd
            }
        }"#;

    /// Reproduce the production grouping prelude (compile_via_spqr Step 1)
    /// up to the point the fusion call site sees: graph, merged flow groups,
    /// and the broker cut set.
    fn groups_for(src: &str) -> (CircuitGraph, Vec<FlowGroup>, DelayedCutSet) {
        let pedal = crate::dsl::parse_pedal_file(src).expect("parse failed");
        let graph = CircuitGraph::from_pedal(&pedal);
        let cut_edges = super::super::boundary_rules::delayed_cut_edges(&graph);
        let active_set: std::collections::HashSet<usize> =
            graph.active_edge_indices.iter().copied().collect();
        let all_edges: Vec<usize> = (0..graph.edges.len())
            .filter(|i| !active_set.contains(i))
            .collect();
        let mut groups = super::super::signal_flow::find_flow_groups(&all_edges, &graph);
        super::super::spqr_build::merge_cross_reactive_groups_into_active_groups(
            &mut groups,
            &graph,
            &cut_edges,
        );
        super::super::spqr_build::merge_input_coupling_into_active_groups(
            &mut groups,
            &graph,
            &cut_edges,
        );
        (graph, groups, cut_edges)
    }

    fn feedback_group_index(groups: &[FlowGroup]) -> usize {
        groups
            .iter()
            .position(|g| g.has_feedback())
            .expect("fixture should produce a feedback group")
    }

    fn edge_index_of(graph: &CircuitGraph, comp_id: &str) -> usize {
        graph
            .edges
            .iter()
            .position(|e| graph.components[e.comp_idx].id == comp_id)
            .unwrap_or_else(|| panic!("no edge for component {comp_id}"))
    }

    #[test]
    fn ba283_shape_analyzes_to_series_cap_into_load() {
        let (graph, groups, cut_edges) = groups_for(TWO_BJT_FB_AMP);
        let target_gi = feedback_group_index(&groups);

        let load = analyze_trailing_output_load(target_gi, &groups, &graph, &cut_edges)
            .expect("BA283 shape should analyze to a trailing output load");

        // Boundary node = the coupling cap's stage-side node (Q2 collector).
        let nout = *graph
            .node_names
            .get("Q2.collector")
            .expect("Q2.collector node");
        assert_eq!(load.boundary_node, nout);

        match &load.model {
            BoundaryLoadModel::SeriesCapIntoLoad { c, r_total, edges } => {
                assert!((c - 10e-6).abs() < 1e-9, "Cout = 10u, got {c}");
                assert!((r_total - 10e3).abs() < 1e-6, "RL = 10k, got {r_total}");
                assert_eq!(edges.len(), 2, "trailing group = {{Cout, RL}}");
            }
            other => panic!("expected SeriesCapIntoLoad, got {other:?}"),
        }

        // Serializable summary carries component ids, not edge indices.
        match load.model.summarize(&graph) {
            BoundaryLoadSummary::SeriesCapIntoLoad { component_ids, .. } => {
                assert_eq!(component_ids, vec!["Cout".to_string(), "RL".to_string()]);
            }
            other => panic!("expected SeriesCapIntoLoad summary, got {other:?}"),
        }
    }

    #[test]
    fn ba283_shape_passes_policy_gate() {
        let (graph, groups, _cut_edges) = groups_for(TWO_BJT_FB_AMP);
        let target_gi = feedback_group_index(&groups);
        assert_eq!(
            gate_ba283_fusion(&groups[target_gi], &graph, 9.0),
            Ok(()),
            "2-BJT DC-feedback target with a dc_qpoint seed should pass the gate"
        );
    }

    #[test]
    fn pot_in_load_group_declines_analysis() {
        let (graph, groups, cut_edges) = groups_for(POT_LOAD_AMP);
        let target_gi = feedback_group_index(&groups);
        assert_eq!(
            analyze_trailing_output_load(target_gi, &groups, &graph, &cut_edges),
            None,
            "a pot in the trailing group is not the fixed-load shape"
        );
    }

    #[test]
    fn broker_cut_edge_declines_analysis() {
        let (graph, groups, mut cut_edges) = groups_for(TWO_BJT_FB_AMP);
        let target_gi = feedback_group_index(&groups);

        // Sanity: without the cut, the shape analyzes.
        assert!(analyze_trailing_output_load(target_gi, &groups, &graph, &cut_edges).is_some());

        // Synthesize a broker cut on the coupling cap edge — never fuse
        // across a delayed-coupling boundary.
        cut_edges.cuts.insert(
            edge_index_of(&graph, "Cout"),
            super::super::boundary_rules::Directive::NonMergeCut,
        );
        assert_eq!(
            analyze_trailing_output_load(target_gi, &groups, &graph, &cut_edges),
            None,
            "a broker-cut edge in the trailing group must decline analysis"
        );
    }

    #[test]
    fn single_bjt_target_declines_gate_with_reason() {
        let (graph, groups, cut_edges) = groups_for(ONE_BJT_FB_AMP);
        let target_gi = feedback_group_index(&groups);

        // The trailing output network itself is the recognized shape...
        assert!(
            analyze_trailing_output_load(target_gi, &groups, &graph, &cut_edges).is_some(),
            "analysis is shape-only; the single-BJT fixture still has Cout->RL"
        );
        // ...but the policy gate declines: not the >=2-BJT dc_qpoint path.
        let reason = gate_ba283_fusion(&groups[target_gi], &graph, 9.0)
            .expect_err("single-BJT target must not pass the BA283 fusion gate");
        assert!(
            reason.starts_with("gate:"),
            "decline reason should name the gate, got {reason:?}"
        );
        assert_eq!(reason, "gate:fewer-than-two-bjts");
    }

    #[test]
    fn compiled_two_bjt_pedal_records_fused_boundary_load() {
        // skip_blockwise forces the feedback group down the general-MNA
        // (dc_qpoint) path where the fusion call site lives; skip_k_tables
        // keeps the debug compile fast.
        let pedal = crate::dsl::parse_pedal_file(TWO_BJT_FB_AMP).expect("parse failed");
        let options = super::super::compile::CompileOptions {
            skip_blockwise: true,
            skip_k_tables: true,
            ..Default::default()
        };
        let compiled =
            super::super::spqr_build::compile_via_spqr_with_options(&pedal, 48_000.0, options)
                .expect("compile failed");

        assert_eq!(
            compiled.boundary_loads.len(),
            1,
            "one analyzed output boundary expected, got {:?}",
            compiled.boundary_loads
        );
        let binding = &compiled.boundary_loads[0];
        assert_eq!(
            binding.disposition,
            pedalkernel_rt::processor::BoundaryLoadDisposition::FusedUpstream
        );
        match &binding.model {
            BoundaryLoadSummary::SeriesCapIntoLoad {
                c,
                r_total,
                component_ids,
            } => {
                assert!((c - 10e-6).abs() < 1e-9);
                assert!((r_total - 10e3).abs() < 1e-6);
                assert_eq!(component_ids, &["Cout".to_string(), "RL".to_string()]);
            }
            other => panic!("expected SeriesCapIntoLoad summary, got {other:?}"),
        }
        // The upstream stage resolved to the fused general-MNA stage.
        assert!(
            binding.upstream_stage < compiled.stages.len(),
            "upstream_stage should resolve, got {}",
            binding.upstream_stage
        );
        assert!(
            matches!(
                compiled.stages[binding.upstream_stage],
                super::super::compiled::Stage::MultiNl(_)
            ),
            "the fused stage is the multi-BJT general-MNA stage"
        );
        // Boundary node = the coupling cap's stage-side node (Q2 collector).
        assert_eq!(binding.boundary_node, graph_node(&pedal, "Q2.collector"));
    }

    #[test]
    fn compiled_single_bjt_pedal_records_unloaded_reason() {
        let pedal = crate::dsl::parse_pedal_file(ONE_BJT_FB_AMP).expect("parse failed");
        let options = super::super::compile::CompileOptions {
            skip_blockwise: true,
            skip_k_tables: true,
            ..Default::default()
        };
        let compiled =
            super::super::spqr_build::compile_via_spqr_with_options(&pedal, 48_000.0, options)
                .expect("compile failed");

        assert_eq!(compiled.boundary_loads.len(), 1);
        match &compiled.boundary_loads[0].disposition {
            pedalkernel_rt::processor::BoundaryLoadDisposition::Unloaded { reason } => {
                assert!(
                    reason.starts_with("gate:"),
                    "single-BJT decline should carry a gate reason, got {reason:?}"
                );
            }
            other => panic!("expected Unloaded disposition, got {other:?}"),
        }
    }

    /// Node id lookup through a fresh graph build (test-only convenience for
    /// asserting boundary nodes against compiled pedals — node ids are
    /// deterministic per netlist).
    fn graph_node(pedal: &crate::dsl::PedalDef, name: &str) -> NodeId {
        let graph = CircuitGraph::from_pedal(pedal);
        *graph
            .node_names
            .get(name)
            .unwrap_or_else(|| panic!("no node named {name}"))
    }
}
