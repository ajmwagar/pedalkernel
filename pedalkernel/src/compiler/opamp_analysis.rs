//! Pass 2: Op-amp feedback topology analysis.
//!
//! Detects op-amp feedback loops and classifies them:
//! - Unity-gain buffers (neg=out) → paired with JFET stages for all-pass
//! - Inverting amplifiers (Rf/Ri) → dedicated op-amp WDF stage
//! - Non-inverting amplifiers (1+Rf/Ri) → dedicated op-amp WDF stage
//!
//! This is genuinely topology-specific analysis that cannot be generalized
//! into the unified pipeline. The detection logic lives in `graph.rs` as
//! `find_opamp_feedback_loops()`.

use std::collections::{HashMap, HashSet};

use crate::dsl::*;
use crate::elements::*;
use crate::elements::FeedbackConfig;

use super::dyn_node::DynNode;
use super::graph::{
    CircuitGraph, NodeId, OpAmpFeedbackInfo, OpAmpFeedbackKind,
};
use super::stage::WdfStage;

/// Result of op-amp analysis.
pub(super) struct OpAmpAnalysis {
    /// Detected feedback loops (unity-gain, inverting, non-inverting).
    pub(super) feedback_loops: Vec<OpAmpFeedbackInfo>,
    /// IDs of op-amps that have feedback (excludes standalone op-amps).
    pub(super) feedback_opamp_ids: HashSet<String>,
    /// IDs of unity-gain op-amps (for JFET pairing).
    #[allow(dead_code)]
    pub(super) unity_gain_opamp_ids: HashSet<String>,
}

/// Run op-amp feedback analysis on the circuit.
pub(super) fn analyze_opamps(graph: &CircuitGraph, pedal: &PedalDef) -> OpAmpAnalysis {
    let feedback_loops = graph.find_opamp_feedback_loops(pedal);

    let feedback_opamp_ids: HashSet<String> = feedback_loops
        .iter()
        .map(|info| info.comp_id.clone())
        .collect();

    let unity_gain_opamp_ids: HashSet<String> = feedback_loops
        .iter()
        .filter(|info| matches!(info.feedback_kind, OpAmpFeedbackKind::UnityGain))
        .map(|info| info.comp_id.clone())
        .collect();

    OpAmpAnalysis {
        feedback_loops,
        feedback_opamp_ids,
        unity_gain_opamp_ids,
    }
}

/// Info for an op-amp root that should be paired with a DiodePair/SingleDiode stage.
///
/// When an inverting op-amp has feedback diodes AND shares a junction with the
/// diode NL element, the opamp gain is applied as `feedback_opamp` on the
/// DiodePair's WdfStage rather than creating a standalone OpAmp WdfStage.
pub(super) struct DiodePairedOpAmp {
    /// Neg node of the opamp (for matching to DiodePair junction).
    pub(super) neg_node: NodeId,
    /// Out node of the opamp.
    pub(super) out_node: NodeId,
    /// Configured OpAmpRoot (gain, GBW, slew — NO soft_clip).
    pub(super) opamp_root: OpAmpRoot,
    /// Feedback pot ID (if any) for runtime gain updates.
    pub(super) feedback_pot_id: Option<String>,
}

/// Build op-amp gain stages from feedback loop analysis.
///
/// Creates WDF stages for inverting and non-inverting op-amps.
/// Sets `FeedbackConfig` on the OpAmpRoot and `feedback_pot_id` on the stage
/// so the stage self-manages gain updates when feedback pots change.
///
/// Returns:
/// - `Vec<WdfStage>`: standalone op-amp stages (gain-only, no shared NL junction)
/// - `Vec<DiodePairedOpAmp>`: op-amps that should be paired with DiodePair stages
pub(super) fn build_opamp_feedback_stages(
    analysis: &OpAmpAnalysis,
    pedal: &PedalDef,
    graph: &CircuitGraph,
    _existing_stage_count: usize,
    sample_rate: f64,
    oversampling: crate::oversampling::OversamplingFactor,
    skip_feedback_tree: &HashSet<String>,
    nl_junction_nodes: &HashSet<NodeId>,
) -> (Vec<WdfStage>, Vec<DiodePairedOpAmp>) {
    use super::stage::RootKind;

    let mut stages = Vec::new();
    let mut diode_paired = Vec::new();

    for info in &analysis.feedback_loops {
        match &info.feedback_kind {
            OpAmpFeedbackKind::UnityGain => {
                // Unity-gain op-amps are paired with JFET stages for all-pass filters.
                // Skip here — handled in plan.rs during JFET stage creation.
            }
            OpAmpFeedbackKind::Inverting {
                rf,
                ri,
                feedback_diode,
                rf_pot,
            } => {
                let mut feedback_pot_id = None;

                let model = OpAmpModel::from_opamp_type(&info.opamp_type);
                let gain = rf / ri;
                let mut root = OpAmpRoot::new_inverting(model, gain);
                root.set_sample_rate(sample_rate);

                // If there's a pot in the Rf feedback path, configure the root
                // to self-manage gain from pot resistance.
                if let Some((pot_id, _max_pot_r, fixed_series_r, parallel_fixed_r)) = rf_pot {
                    root.set_feedback_config(FeedbackConfig {
                        pot_comp_id: pot_id.clone(),
                        other_leg_r: *ri,
                        fixed_series_r: *fixed_series_r,
                        parallel_r: *parallel_fixed_r,
                        pot_is_feedback: true,
                        is_inverting: true,
                    });
                    feedback_pot_id = Some(pot_id.clone());
                }

                let default_supply = 9.0_f64;
                let v_max = (default_supply / 2.0 - 1.5).max(0.5);
                root.set_v_max(v_max);

                // Skip feedback tree if opamp output shares junction with NL elements.
                // The NL stage handles the full passive set including feedback components.
                let skip_tree = skip_feedback_tree.contains(&info.comp_id);

                // Pair this opamp with the DiodePair/SingleDiode stage when:
                // - feedback_diode: diodes ARE in feedback (Tube Screamer, Klon)
                // - output IS diode junction: diodes shunt opamp output to ground (RAT)
                // In both cases the NR solver does accurate clipping; opamp just provides gain.
                // Don't set soft_clip — the NR solver handles it.
                let output_is_nl_junction = nl_junction_nodes.contains(&info.out_node);
                if skip_tree && (feedback_diode.is_some() || output_is_nl_junction) {
                    diode_paired.push(DiodePairedOpAmp {
                        neg_node: info.neg_node,
                        out_node: info.out_node,
                        opamp_root: root,
                        feedback_pot_id,
                    });
                    continue;
                }

                if let Some(diode_type) = feedback_diode {
                    let diode_vf = match diode_type {
                        DiodeType::Silicon => 0.6,
                        DiodeType::Germanium => 0.3,
                        DiodeType::Led => 1.6,
                    };
                    root.set_soft_clip(diode_vf);
                }
                let (tree, fb_pot_from_tree) =
                    if skip_tree {
                        (None, None)
                    } else {
                        match build_feedback_tree(info, graph, pedal, sample_rate) {
                            Some((t, pot_id)) => (Some(t), pot_id),
                            None => (None, None),
                        }
                    };
                // Track whether graph_reduce built the tree (has reactive elements).
                // If so, skip balance_vs_impedance — the VS must stay low-Rp so
                // the capacitor's frequency-dependent scattering isn't swamped.
                let has_complex_fb_tree = tree.is_some();
                let (tree, fb_pot_from_tree) = match (tree, fb_pot_from_tree) {
                    (Some(t), pot_id) => (t, pot_id),
                    _ => {
                        // Fallback: existing bare VS + optional pot
                        let vs = DynNode::VoltageSource(0.0, 10_000.0);
                        let tree = if let Some((
                            pot_id,
                            max_pot_r,
                            _fixed_series_r,
                            _parallel_fixed_r,
                        )) = rf_pot
                        {
                            let taper = lookup_pot_taper(pedal, pot_id);
                            let initial_pos = 0.5;
                            let pot = DynNode::Pot(
                                pot_id.clone(),
                                *max_pot_r,
                                initial_pos,
                                taper,
                            );
                            DynNode::Series(Box::new(vs), Box::new(pot))
                        } else {
                            vs
                        };
                        (tree, rf_pot.as_ref().map(|(id, ..)| id.clone()))
                    }
                };
                // Tree-discovered pot takes priority
                let has_tree_pot = fb_pot_from_tree.is_some();
                if has_tree_pot {
                    feedback_pot_id = fb_pot_from_tree;
                }

                let mut stage = WdfStage {
                    tree,
                    root: RootKind::OpAmp(root),
                    compensation: 1.0,
                    oversampler: crate::oversampling::Oversampler::new(oversampling),
                    base_diode_model: None,
                    paired_opamp: None,
                    allpass_feedback: None,
                    allpass_direct: None,
                    dc_block: None,
                    is_source_follower: false,
                    prev_source_voltage: 0.0,
                    signal_flow_distance: 0,
                    transformer_gain: 1.0,
                    injection_node_id: usize::MAX,
                    output_node_id: usize::MAX,
                    is_trigger_voice: false,
                    is_feedforward: false,
                    sample_counter: 0,
                    root_comp_id: String::new(),
                    feedback_pot_id,
                    output_probe: None,
                    feedback_opamp: None,
                    vcc_injection_coeff: 0.0,
                    vcc_dc_ramp: 0,
                    tone_feedback: None,
                };
                // Don't balance VS impedance for complex feedback trees — their
                // VS must remain low-Rp so the cap/pot frequency shaping works.
                if !has_complex_fb_tree {
                    stage.balance_vs_impedance();
                }
                stages.push(stage);
            }
            OpAmpFeedbackKind::NonInverting { rf, ri, rf_pot, ri_pot } => {
                let mut feedback_pot_id = None;

                let model = OpAmpModel::from_opamp_type(&info.opamp_type);
                let gain = 1.0 + (rf / ri);
                let mut root = OpAmpRoot::new_non_inverting(model, gain);
                root.set_sample_rate(sample_rate);

                // Rf pot in feedback path
                if let Some((pot_id, _max_pot_r, fixed_series_r, parallel_fixed_r)) = rf_pot {
                    root.set_feedback_config(FeedbackConfig {
                        pot_comp_id: pot_id.clone(),
                        other_leg_r: *ri,
                        fixed_series_r: *fixed_series_r,
                        parallel_r: *parallel_fixed_r,
                        pot_is_feedback: true,
                        is_inverting: false,
                    });
                    feedback_pot_id = Some(pot_id.clone());
                }
                // Ri pot in ground leg (overrides Rf pot if both present — unlikely)
                if let Some((pot_id, _max_pot_r, fixed_series_r, _parallel_fixed_r)) = ri_pot {
                    root.set_feedback_config(FeedbackConfig {
                        pot_comp_id: pot_id.clone(),
                        other_leg_r: *rf,
                        fixed_series_r: *fixed_series_r,
                        parallel_r: None,
                        pot_is_feedback: false,
                        is_inverting: false,
                    });
                    feedback_pot_id = Some(pot_id.clone());
                }

                let default_supply = 9.0_f64;
                let v_max = (default_supply / 2.0 - 1.5).max(0.5);
                root.set_v_max(v_max);

                let skip_tree = skip_feedback_tree.contains(&info.comp_id);
                let (tree, fb_pot_from_tree) =
                    if skip_tree {
                        (None, None)
                    } else {
                        match build_feedback_tree(info, graph, pedal, sample_rate) {
                            Some((t, pot_id)) => (Some(t), pot_id),
                            None => (None, None),
                        }
                    };
                let has_complex_fb_tree = tree.is_some();
                let (tree, fb_pot_from_tree) = match (tree, fb_pot_from_tree) {
                    (Some(t), pot_id) => (t, pot_id),
                    _ => {
                        // Fallback: existing bare VS + optional pot
                        let vs = DynNode::VoltageSource(0.0, 10_000.0);
                        let active_pot = rf_pot.as_ref().or(ri_pot.as_ref());
                        let tree = if let Some((
                            pot_id,
                            max_pot_r,
                            _fixed_series_r,
                            _parallel_fixed_r,
                        )) = active_pot
                        {
                            let taper = lookup_pot_taper(pedal, pot_id);
                            let initial_pos = 0.5;
                            let pot = DynNode::Pot(
                                pot_id.clone(),
                                *max_pot_r,
                                initial_pos,
                                taper,
                            );
                            DynNode::Series(Box::new(vs), Box::new(pot))
                        } else {
                            vs
                        };
                        let pot_id = rf_pot
                            .as_ref()
                            .or(ri_pot.as_ref())
                            .map(|(id, ..)| id.clone());
                        (tree, pot_id)
                    }
                };
                // Tree-discovered pot takes priority
                let has_tree_pot = fb_pot_from_tree.is_some();
                if has_tree_pot {
                    feedback_pot_id = fb_pot_from_tree;
                }

                let mut stage = WdfStage {
                    tree,
                    root: RootKind::OpAmp(root),
                    compensation: 1.0,
                    oversampler: crate::oversampling::Oversampler::new(oversampling),
                    base_diode_model: None,
                    paired_opamp: None,
                    allpass_feedback: None,
                    allpass_direct: None,
                    dc_block: None,
                    is_source_follower: false,
                    prev_source_voltage: 0.0,
                    signal_flow_distance: 0,
                    transformer_gain: 1.0,
                    injection_node_id: usize::MAX,
                    output_node_id: usize::MAX,
                    is_trigger_voice: false,
                    is_feedforward: false,
                    sample_counter: 0,
                    root_comp_id: String::new(),
                    feedback_pot_id,
                    output_probe: None,
                    feedback_opamp: None,
                    vcc_injection_coeff: 0.0,
                    vcc_dc_ramp: 0,
                    tone_feedback: None,
                };
                if !has_complex_fb_tree {
                    stage.balance_vs_impedance();
                }
                stages.push(stage);
            }
            OpAmpFeedbackKind::AllpassJfet { .. } => {
                // Handled during JFET stage building in build_stages().
            }
            OpAmpFeedbackKind::Allpass { .. } => {
                // Paired with JFET stage via paired_opamp in build_stages().
            }
        }
    }

    (stages, diode_paired)
}

/// Build a queue of unity-gain op-amp roots for pairing with JFET stages.
pub(super) fn build_unity_gain_queue(analysis: &OpAmpAnalysis, sample_rate: f64) -> Vec<OpAmpRoot> {
    analysis
        .feedback_loops
        .iter()
        .filter(|info| matches!(info.feedback_kind, OpAmpFeedbackKind::UnityGain))
        .map(|info| {
            let model = OpAmpModel::from_opamp_type(&info.opamp_type);
            let mut opamp = OpAmpRoot::new(model);
            opamp.set_sample_rate(sample_rate);
            opamp
        })
        .collect()
}

/// Build a map from JFET component ID → (rf, cf) for AllpassJfet feedback topologies.
pub(super) fn build_allpass_jfet_map(analysis: &OpAmpAnalysis) -> HashMap<String, (f64, f64)> {
    let mut map = HashMap::new();
    for info in &analysis.feedback_loops {
        if let OpAmpFeedbackKind::AllpassJfet {
            rf,
            cf,
            ref jfet_id,
        } = info.feedback_kind
        {
            map.insert(jfet_id.clone(), (rf, cf));
        }
    }
    map
}

/// Info for an all-pass opamp to be paired with a JFET stage.
pub(super) struct AllpassPairing {
    /// Phase-shifting capacitor value in Farads.
    pub(super) cap: f64,
    /// Sample rate for IIR coefficient computation.
    pub(super) sample_rate: f64,
}

/// Build a map from JFET component ID → AllpassPairing for Allpass feedback topologies.
/// The AllpassDirect IIR computes H(s) = (1-sRC)/(1+sRC) using the JFET's Rds and C_ap.
pub(super) fn build_allpass_queue(
    analysis: &OpAmpAnalysis,
    sample_rate: f64,
) -> HashMap<String, AllpassPairing> {
    let mut map = HashMap::new();
    for info in &analysis.feedback_loops {
        if let OpAmpFeedbackKind::Allpass {
            ref jfet_id,
            cap,
            ..
        } = info.feedback_kind
        {
            map.insert(jfet_id.clone(), AllpassPairing {
                cap,
                sample_rate,
            });
        }
    }
    map
}

/// Build standalone op-amp stages (no feedback detected).
pub(super) fn build_standalone_opamp_stages(
    pedal: &PedalDef,
    feedback_opamp_ids: &HashSet<String>,
    sample_rate: f64,
) -> Vec<super::compiled::OpAmpStage> {
    let mut opamp_stages = Vec::new();
    for comp in &pedal.components {
        if let Some(ot) = comp.kind.op_amp_type() {
            if !ot.is_ota() && !feedback_opamp_ids.contains(&comp.id) {
                let model = OpAmpModel::from_opamp_type(&ot);
                let mut opamp = OpAmpRoot::new(model);
                opamp.set_sample_rate(sample_rate);
                opamp_stages.push(super::compiled::OpAmpStage {
                    opamp,
                    comp_id: comp.id.clone(),
                });
            }
        }
    }
    opamp_stages
}

/// Try to build a WDF tree from the feedback network via SP reduction.
/// Returns (tree, feedback_pot_id) or None if the network is too simple
/// or SP reduction fails.
fn build_feedback_tree(
    info: &OpAmpFeedbackInfo,
    graph: &CircuitGraph,
    pedal: &PedalDef,
    sample_rate: f64,
) -> Option<(DynNode, Option<String>)> {
    if info.feedback_comp_ids.len() <= 1 {
        return None; // Single resistor — use existing simple path
    }

    // Step 1: Collect feedback edge indices from graph
    let feedback_set: HashSet<&str> = info.feedback_comp_ids.iter().map(|s| s.as_str()).collect();
    let feedback_edges: Vec<usize> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(_, e)| feedback_set.contains(graph.components[e.comp_idx].id.as_str()))
        .map(|(idx, _)| idx)
        .collect();

    if feedback_edges.len() < 2 {
        return None;
    }

    // Step 2: Find terminal nodes (boundary of feedback subgraph)
    // Border nodes: appear in feedback edges AND non-feedback edges
    let fb_nodes: HashSet<NodeId> = feedback_edges
        .iter()
        .flat_map(|&idx| {
            let e = &graph.edges[idx];
            [e.node_a, e.node_b]
        })
        .collect();

    let mut border_nodes: Vec<NodeId> = fb_nodes
        .iter()
        .filter(|&&n| {
            graph.edges.iter().any(|e| {
                (e.node_a == n || e.node_b == n)
                    && !feedback_set.contains(graph.components[e.comp_idx].id.as_str())
            })
        })
        .copied()
        .collect();
    border_nodes.sort();
    border_nodes.dedup();

    if border_nodes.len() != 2 {
        return None; // Not a clean 2-terminal feedback network
    }

    // Step 3: Reduce the feedback network to a 2-terminal DynNode tree.
    // Don't embed VS as an ExtraEdge (it'd end up in Parallel with feedback
    // resistors sharing the same border nodes, making the VS invisible).
    // Instead, build the pure passive tree and wrap it in Series(VS, tree).
    let extra = vec![];
    let terminals = vec![border_nodes[0], border_nodes[1]];

    let fb_tree = super::graph::graph_reduce(
        &feedback_edges, &extra, &terminals,
        graph, sample_rate, &std::collections::HashMap::new(), |n| n,
        None,
    ).ok()?.0;

    // Wrap in Series(VS, feedback_tree) — VS drives the feedback impedance
    let vs = DynNode::VoltageSource(0.0, 1.0);
    let tree = DynNode::Series(Box::new(vs), Box::new(fb_tree));

    // Step 6: Find feedback pot ID (if any pot in the network)
    let feedback_pot_id = info
        .feedback_comp_ids
        .iter()
        .find(|id| {
            let base_id = id
                .strip_suffix("__aw")
                .or_else(|| id.strip_suffix("__wb"))
                .unwrap_or(id);
            pedal
                .components
                .iter()
                .any(|c| c.id == base_id && c.kind.pot_taper().is_some())
        })
        .map(|id| {
            id.strip_suffix("__aw")
                .or_else(|| id.strip_suffix("__wb"))
                .unwrap_or(id)
                .to_string()
        });

    Some((tree, feedback_pot_id))
}

/// Look up the taper of a potentiometer from the pedal definition.
fn lookup_pot_taper(pedal: &PedalDef, pot_id: &str) -> PotTaper {
    pedal
        .components
        .iter()
        .find(|c| c.id == pot_id)
        .and_then(|c| c.kind.pot_taper())
        .unwrap_or(PotTaper::B)
}
