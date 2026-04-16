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
use crate::elements::FeedbackConfig;
use crate::elements::*;

use super::components::PhotocouplerComp;
use super::dyn_node::DynNode;
use super::graph::{CircuitGraph, NodeId, OpAmpFeedbackInfo, OpAmpFeedbackKind};
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
///
/// **NULLOR REFACTOR (Phase 3b):** This function now returns an empty
/// analysis. All op-amps are handled via the unified nullor-in-R-type
/// pipeline — each op-amp contributes a `stamp_vcvs` to its stage's MNA
/// matrix (see `build_rtype_stage::InStageNullor`). Topology-specific
/// classification is gone; the scattering matrix handles every feedback
/// topology automatically.
///
/// Arguments are kept for the moment to minimise churn at call sites.
/// They will be removed along with the entire module in Phase 7.
#[allow(unused_variables)]
pub(super) fn analyze_opamps(
    graph: &CircuitGraph,
    pedal: &PedalDef,
    pre_classified: &[OpAmpFeedbackInfo],
    skip_ids: &HashSet<String>,
) -> OpAmpAnalysis {
    OpAmpAnalysis {
        feedback_loops: Vec::new(),
        feedback_opamp_ids: HashSet::new(),
        unity_gain_opamp_ids: HashSet::new(),
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
/// - `HashSet<usize>`: edge indices consumed by opamp feedback stages (for global claiming)
pub(super) fn build_opamp_feedback_stages(
    analysis: &OpAmpAnalysis,
    pedal: &PedalDef,
    graph: &CircuitGraph,
    _existing_stage_count: usize,
    sample_rate: f64,
    oversampling: crate::oversampling::OversamplingFactor,
    skip_feedback_tree: &HashSet<String>,
    nl_junction_nodes: &HashSet<NodeId>,
) -> (Vec<WdfStage>, Vec<DiodePairedOpAmp>, HashSet<usize>) {
    use super::stage::RootKind;

    let mut stages = Vec::new();
    let mut diode_paired = Vec::new();
    let mut consumed_edges: HashSet<usize> = HashSet::new();

    // Helper: map feedback_comp_ids to edge indices.
    let comp_ids_to_edge_indices = |comp_ids: &[String]| -> Vec<usize> {
        let id_set: HashSet<&str> = comp_ids.iter().map(|s| s.as_str()).collect();
        graph
            .edges
            .iter()
            .enumerate()
            .filter(|(_, e)| id_set.contains(graph.components[e.comp_idx].id.as_str()))
            .map(|(idx, _)| idx)
            .collect()
    };

    for info in &analysis.feedback_loops {
        match &info.feedback_kind {
            OpAmpFeedbackKind::UnityGain => {
                // Unity-gain op-amps are paired with JFET stages for all-pass filters.
                // Skip here — handled in plan.rs during JFET stage creation.
            }
            OpAmpFeedbackKind::BridgedTResonator { r1, r2, c1, c2, rf } => {
                let model = OpAmpModel::from_opamp_type(&info.opamp_type);
                let gain = rf / r1;
                let mut root = OpAmpRoot::new_inverting(model, gain);
                root.set_sample_rate(sample_rate);

                let default_supply = 9.0_f64;
                let v_max = (default_supply / 2.0 - 1.5).max(0.5);
                root.set_v_max(v_max);

                let res_fb =
                    super::stage::ResonatorFeedback::new(*r1, *r2, *c1, *c2, *rf, sample_rate);

                // Minimal tree — the IIR bypasses WDF output, so the tree
                // is only needed for pot state tracking (none for bridged-T).
                let vs = DynNode::VoltageSource(0.0, 10_000.0);

                let mut stage = WdfStage {
                    injection_node_id: info.neg_node,
                    output_node_id: info.out_node,
                    resonator_feedback: Some(res_fb),
                    ..WdfStage::new(
                        vs,
                        RootKind::OpAmp(root),
                        crate::oversampling::Oversampler::new(oversampling),
                    )
                };
                stage.balance_vs_impedance();
                // BridgedTResonator always claims its feedback edges.
                consumed_edges.extend(comp_ids_to_edge_indices(&info.feedback_comp_ids));
                stages.push(stage);
            }
            OpAmpFeedbackKind::Inverting {
                rf,
                ri,
                feedback_diode,
                rf_pot,
                ri_pot,
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

                // Ri pot in input path — only use if rf_pot was NOT found.
                // Same guard as NonInverting: circuitous ground paths can find
                // downstream pots that aren't part of the gain equation.
                if rf_pot.is_none() {
                    if let Some((pot_id, max_pot_r, fixed_series_r)) = ri_pot {
                        root.set_feedback_config(FeedbackConfig {
                            pot_comp_id: pot_id.clone(),
                            other_leg_r: *rf,
                            fixed_series_r: *fixed_series_r,
                            parallel_r: None,
                            pot_is_feedback: false,
                            is_inverting: true,
                        });
                        feedback_pot_id = Some(pot_id.clone());
                    }
                }

                let default_supply = 9.0_f64;
                let v_max = (default_supply / 2.0 - 1.5).max(0.5);
                root.set_v_max(v_max);

                // Skip feedback tree if opamp output shares junction with NL elements.
                // The NL stage handles the full passive set including feedback components.
                let skip_tree = skip_feedback_tree.contains(&info.comp_id);

                // Pair this opamp with the diode stage when the opamp output IS a
                // diode junction (shunt diodes to ground, e.g. RAT). The NR solver
                // handles clipping; opamp just provides gain.
                //
                // Do NOT pair for feedback diodes (Screamer, Klon) — feedback diodes
                // are modeled by OpAmpRoot soft_clip and the Drive pot must remain in
                // the opamp's own feedback tree / MNA adaptor to respond to control
                // changes. The DiodePairedOpAmp path loses pot responsiveness.
                let output_is_nl_junction = nl_junction_nodes.contains(&info.out_node);
                if skip_tree && output_is_nl_junction {
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
                        DiodeType::Schottky => 0.3,
                    };
                    root.set_soft_clip(diode_vf);
                }
                // ── Try WDF constraint adaptor for Inverting with reactive feedback ──
                // Uses V_neg = V+ closed-form scattering. Gain emerges from Zf/Zi
                // impedance ratios — caps and pots handled naturally via WDF state.
                let has_reactive_fb = !skip_tree
                    && info.input_photocoupler_ids.is_empty()
                    && info.feedback_comp_ids.iter().any(|id| {
                        pedal
                            .components
                            .iter()
                            .any(|c| c.id == *id && c.kind.capacitance().is_some())
                            || is_pot_component(id, pedal)
                    });
                #[cfg(test)]
                eprintln!(
                    "[OPAMP_WDF] {} has_reactive_fb={} skip_tree={}",
                    info.comp_id, has_reactive_fb, skip_tree
                );
                if has_reactive_fb {
                    let wdf_result = build_opamp_wdf_adaptor(info, graph, pedal, sample_rate, *ri);
                    #[cfg(test)]
                    eprintln!(
                        "[OPAMP_WDF] {} build_opamp_wdf_adaptor → {}",
                        info.comp_id,
                        wdf_result.is_some()
                    );
                    if let Some((zi_tree, zf_tree, wdf_pot_id, wdf_edges)) = wdf_result {
                        if let Some(ref pid) = wdf_pot_id {
                            feedback_pot_id = Some(pid.clone());
                        }
                        // Set GBW gain from the DC gain for bandwidth rolloff
                        let gain = rf / ri;
                        root.set_gain(1.0); // WDF adaptor handles gain
                        root.set_gbw_gain(gain);
                        root.set_sample_rate(sample_rate);

                        let default_supply = 9.0_f64;
                        let v_max = (default_supply / 2.0 - 1.5).max(0.5);
                        root.set_v_max(v_max);

                        let wdf_adaptor = super::stage::OpAmpWdfAdaptor {
                            zi: zi_tree,
                            zf: zf_tree,
                            is_inverting: true,
                            opamp: root.clone(),
                            gbw_state: 0.0,
                            prev_out: 0.0,
                            feedback_pot_id: feedback_pot_id.clone(),
                        };

                        let vs = DynNode::VoltageSource(0.0, 10_000.0);
                        let mut stage = WdfStage {
                            feedback_pot_id,
                            opamp_wdf_adaptor: Some(wdf_adaptor),
                            ..WdfStage::new(
                                vs,
                                RootKind::OpAmp(root),
                                crate::oversampling::Oversampler::new(oversampling),
                            )
                        };
                        // Populate input-path photocouplers for inverting opamps (e.g. Mu-Tron).
                        // Must happen regardless of which path (MNA or fallback) is taken.
                        if !info.input_photocoupler_ids.is_empty() {
                            for pc_id in &info.input_photocoupler_ids {
                                if let Some(pc_comp) =
                                    pedal.components.iter().find(|c| c.id == *pc_id)
                                {
                                    if let Some(pc_type) =
                                        pc_comp.kind.as_any().downcast_ref::<PhotocouplerComp>()
                                    {
                                        use crate::dsl::PhotocouplerType;
                                        let model = match pc_type.coupler_type {
                                            PhotocouplerType::Vtl5c3 => PhotocouplerModel::vtl5c3(),
                                            PhotocouplerType::Vtl5c1 => PhotocouplerModel::vtl5c1(),
                                            PhotocouplerType::Nsl32 => PhotocouplerModel::nsl32(),
                                            PhotocouplerType::T4b => PhotocouplerModel::t4b(),
                                        };
                                        stage.input_photocouplers.push(
                                            super::stage::InputPhotocoupler {
                                                comp_id: pc_id.clone(),
                                                element: Photocoupler::new(model, sample_rate),
                                                fixed_series_r: info.input_fixed_r,
                                                dc_rf: *rf,
                                            },
                                        );
                                    }
                                }
                            }
                        }
                        consumed_edges.extend(comp_ids_to_edge_indices(&info.feedback_comp_ids));
                        consumed_edges.extend(comp_ids_to_edge_indices(&info.ground_leg_comp_ids));
                        consumed_edges.extend(comp_ids_to_edge_indices(&info.input_comp_ids));
                        stages.push(stage);
                        continue;
                    }
                }

                // Fallback: standard tree path (purely resistive feedback or adaptor failed)
                let (tree, fb_pot_from_tree, fb_tree_edges) = if skip_tree {
                    (None, None, vec![])
                } else {
                    match build_feedback_tree(info, graph, pedal, sample_rate) {
                        Some((t, pot_id, edges)) => (Some(t), pot_id, edges),
                        None => (None, None, vec![]),
                    }
                };
                // Track whether graph_reduce built the tree (has reactive elements).
                // If so, skip balance_vs_impedance — the VS must stay low-Rp so
                // the capacitor's frequency-dependent scattering isn't swamped.
                let has_complex_fb_tree = tree.is_some();
                if !skip_tree {
                    consumed_edges.extend(fb_tree_edges.iter().copied());
                }
                let (tree, fb_pot_from_tree) = match (tree, fb_pot_from_tree) {
                    (Some(t), pot_id) => (t, pot_id),
                    _ => {
                        // Fallback: existing bare VS + optional pot(s)
                        let vs = DynNode::VoltageSource(0.0, 10_000.0);
                        let mut tree =
                            if let Some((pot_id, max_pot_r, _fixed_series_r, _parallel_fixed_r)) =
                                rf_pot
                            {
                                let taper = lookup_pot_taper(pedal, pot_id);
                                let initial_pos = 0.5;
                                let pot =
                                    DynNode::Pot(pot_id.clone(), *max_pot_r, initial_pos, taper);
                                DynNode::Series(Box::new(vs), Box::new(pot))
                            } else {
                                vs
                            };
                        let mut fb_pot = rf_pot.as_ref().map(|(id, ..)| id.clone());

                        // Also add ri_pot to tree so pot binding and
                        // notify_pot_changed can read its resistance.
                        if let Some((pot_id, max_pot_r, _fixed_series_r)) = ri_pot {
                            let taper = lookup_pot_taper(pedal, pot_id);
                            let initial_pos = 0.5;
                            let pot = DynNode::Pot(pot_id.clone(), *max_pot_r, initial_pos, taper);
                            tree = DynNode::Series(Box::new(tree), Box::new(pot));
                            if fb_pot.is_none() {
                                fb_pot = Some(pot_id.clone());
                            }
                        }

                        (tree, fb_pot)
                    }
                };
                // Tree-discovered pot takes priority
                let has_tree_pot = fb_pot_from_tree.is_some();
                if has_tree_pot {
                    feedback_pot_id = fb_pot_from_tree;
                }

                let mut stage = WdfStage {
                    feedback_pot_id,
                    ..WdfStage::new(
                        tree,
                        RootKind::OpAmp(root),
                        crate::oversampling::Oversampler::new(oversampling),
                    )
                };

                // Create input-path photocoupler elements if present.
                if !info.input_photocoupler_ids.is_empty() {
                    for pc_id in &info.input_photocoupler_ids {
                        // Find the photocoupler component to get its model.
                        if let Some(pc_comp) = pedal.components.iter().find(|c| c.id == *pc_id) {
                            if let Some(pc_type) =
                                pc_comp.kind.as_any().downcast_ref::<PhotocouplerComp>()
                            {
                                use crate::dsl::PhotocouplerType;
                                let model = match pc_type.coupler_type {
                                    PhotocouplerType::Vtl5c3 => PhotocouplerModel::vtl5c3(),
                                    PhotocouplerType::Vtl5c1 => PhotocouplerModel::vtl5c1(),
                                    PhotocouplerType::Nsl32 => PhotocouplerModel::nsl32(),
                                    PhotocouplerType::T4b => PhotocouplerModel::t4b(),
                                };
                                stage
                                    .input_photocouplers
                                    .push(super::stage::InputPhotocoupler {
                                        comp_id: pc_id.clone(),
                                        element: Photocoupler::new(model, sample_rate),
                                        fixed_series_r: info.input_fixed_r,
                                        dc_rf: *rf,
                                    });
                            }
                        }
                    }
                }

                // Don't balance VS impedance for complex feedback trees — their
                // VS must remain low-Rp so the cap/pot frequency shaping works.
                if !has_complex_fb_tree {
                    stage.balance_vs_impedance();
                }
                // Claim feedback edges when:
                // - NOT skip_tree (normal inverting opamp, standalone stage), OR
                // - skip_tree but no feedback diode (neg_is_barrier: neg used as BFS
                //   barrier so NL BFS can't claim these edges; claim them here instead).
                // Skip claiming when skip_tree AND feedback_diode exists (Tube Screamer,
                // Bluesbreaker) — the NL BFS must traverse through neg to incorporate them.
                if !skip_tree || feedback_diode.is_none() {
                    consumed_edges.extend(comp_ids_to_edge_indices(&info.feedback_comp_ids));
                }
                stages.push(stage);
            }
            OpAmpFeedbackKind::NonInverting {
                rf,
                ri,
                feedback_diode,
                rf_pot,
                ri_pot,
            } => {
                let mut feedback_pot_id = None;

                #[cfg(test)]
                eprintln!("[OPAMP_ANALYSIS] NonInverting comp={} rf={rf:.1} ri={ri:.1} rf_pot={rf_pot:?} ri_pot={ri_pot:?}",
                    info.comp_id);

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
                // Ri pot in ground leg — only use if rf_pot was NOT found.
                // When rf_pot exists, the ground-leg path search can find spurious pots
                // via circuitous routes (neg → Rf → out → coupling_cap → ... → pot → gnd).
                // These are downstream tone/filter pots, not gain-determining ground-leg pots.
                if rf_pot.is_none() {
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
                }

                let default_supply = 9.0_f64;
                let v_max = (default_supply / 2.0 - 1.5).max(0.5);
                root.set_v_max(v_max);

                let skip_tree = skip_feedback_tree.contains(&info.comp_id);

                // Pair with diode stage only for shunt diodes (output IS diode junction).
                // Feedback diodes use OpAmpRoot soft_clip instead — preserves pot binding.
                let output_is_nl_junction = nl_junction_nodes.contains(&info.out_node);
                if skip_tree && output_is_nl_junction {
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
                        DiodeType::Schottky => 0.3,
                    };
                    root.set_soft_clip(diode_vf);
                }
                // ── Try WDF constraint adaptor for NonInverting ──
                let has_reactive_ni = !skip_tree
                    && (info.feedback_comp_ids.iter().any(|id| {
                        pedal
                            .components
                            .iter()
                            .any(|c| c.id == *id && c.kind.capacitance().is_some())
                            || is_pot_component(id, pedal)
                    }) || info.ground_leg_comp_ids.iter().any(|id| {
                        pedal
                            .components
                            .iter()
                            .any(|c| c.id == *id && c.kind.capacitance().is_some())
                            || is_pot_component(id, pedal)
                    }));
                #[cfg(test)]
                eprintln!(
                    "[OPAMP_WDF_NI] {} has_reactive_ni={}",
                    info.comp_id, has_reactive_ni
                );
                if has_reactive_ni {
                    if let Some((zi_tree, zf_tree, wdf_pot_id, wdf_edges)) =
                        build_opamp_wdf_adaptor_ni(info, graph, pedal, sample_rate)
                    {
                        if let Some(ref pid) = wdf_pot_id {
                            feedback_pot_id = Some(pid.clone());
                        }
                        let gain = 1.0 + rf / ri;
                        root.set_gain(1.0);
                        root.set_gbw_gain(gain);

                        let wdf_adaptor = super::stage::OpAmpWdfAdaptor {
                            zi: zi_tree,
                            zf: zf_tree,
                            is_inverting: false,
                            opamp: root.clone(),
                            gbw_state: 0.0,
                            prev_out: 0.0,
                            feedback_pot_id: feedback_pot_id.clone(),
                        };

                        let vs = DynNode::VoltageSource(0.0, 10_000.0);
                        let stage = WdfStage {
                            feedback_pot_id,
                            opamp_wdf_adaptor: Some(wdf_adaptor),
                            ..WdfStage::new(
                                vs,
                                RootKind::OpAmp(root),
                                crate::oversampling::Oversampler::new(oversampling),
                            )
                        };
                        consumed_edges.extend(comp_ids_to_edge_indices(&info.feedback_comp_ids));
                        consumed_edges.extend(comp_ids_to_edge_indices(&info.ground_leg_comp_ids));
                        stages.push(stage);
                        continue;
                    }
                }

                // Try 3-port adaptor when ground leg has reactive components
                let has_reactive_gnd = !skip_tree
                    && !info.ground_leg_comp_ids.is_empty()
                    && info.ground_leg_comp_ids.iter().any(|id| {
                        pedal.components.iter().any(|c| {
                            c.id == *id
                                && (c.kind.capacitance().is_some()
                                    || c.kind.as_any().downcast_ref::<Inductor>().is_some())
                        })
                    });

                let three_port = if has_reactive_gnd {
                    #[cfg(test)]
                    eprintln!("[OPAMP_NI] Attempting 3-port adaptor for {}", info.comp_id);
                    build_3port_opamp(info, graph, pedal, sample_rate)
                } else {
                    None
                };

                if let Some((zf_tree, zg_tree, adaptor, zf_pot_id, recompute_data)) = three_port {
                    // 3-port adaptor built successfully
                    if let Some(ref pot_id) = zf_pot_id {
                        feedback_pot_id = Some(pot_id.clone());
                    }
                    // The 3-port scattering matrix handles port impedance relationships.
                    // The VCVS gain must be 1.0 to avoid applying gain twice (once in the
                    // scattering and once in OpAmpRoot::process). The true closed-loop gain
                    // is stored in gbw_gain for GBW rolloff calculation only.
                    let true_gain = root.gain();
                    root.set_gain(1.0);
                    root.set_gbw_gain(true_gain);
                    // Dummy tree (holds pot state for notify_pot_changed)
                    let vs = DynNode::VoltageSource(0.0, 10_000.0);

                    let stage = WdfStage {
                        feedback_pot_id,
                        zf_child: Some(zf_tree),
                        zg_child: Some(zg_tree),
                        opamp_adaptor: Some(adaptor),
                        opamp_recompute: Some(recompute_data),
                        ..WdfStage::new(
                            vs,
                            RootKind::OpAmp(root),
                            crate::oversampling::Oversampler::new(oversampling),
                        )
                    };
                    // Don't balance VS — the 3-port adaptor handles impedance matching
                    consumed_edges.extend(comp_ids_to_edge_indices(&info.feedback_comp_ids));
                    consumed_edges.extend(comp_ids_to_edge_indices(&info.ground_leg_comp_ids));
                    stages.push(stage);
                    continue;
                }

                // Fallback: standard 2-port path (no reactive ground leg)
                let (tree, fb_pot_from_tree, fb_tree_edges_ni) = if skip_tree {
                    (None, None, vec![])
                } else {
                    match build_feedback_tree(info, graph, pedal, sample_rate) {
                        Some((t, pot_id, edges)) => (Some(t), pot_id, edges),
                        None => (None, None, vec![]),
                    }
                };
                let has_complex_fb_tree = tree.is_some();
                if !skip_tree {
                    consumed_edges.extend(fb_tree_edges_ni.iter().copied());
                }
                let (tree, fb_pot_from_tree) = match (tree, fb_pot_from_tree) {
                    (Some(t), pot_id) => (t, pot_id),
                    _ => {
                        // Fallback: existing bare VS + optional pot
                        let vs = DynNode::VoltageSource(0.0, 10_000.0);
                        let active_pot = rf_pot.as_ref().or(ri_pot.as_ref());
                        let tree =
                            if let Some((pot_id, max_pot_r, _fixed_series_r, _parallel_fixed_r)) =
                                active_pot
                            {
                                let taper = lookup_pot_taper(pedal, pot_id);
                                let initial_pos = 0.5;
                                let pot =
                                    DynNode::Pot(pot_id.clone(), *max_pot_r, initial_pos, taper);
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
                    feedback_pot_id,
                    ..WdfStage::new(
                        tree,
                        RootKind::OpAmp(root),
                        crate::oversampling::Oversampler::new(oversampling),
                    )
                };
                if !has_complex_fb_tree {
                    stage.balance_vs_impedance();
                }
                // Claim feedback edges (Zf) and ground-leg edges (Zg).
                if !skip_tree || feedback_diode.is_none() {
                    consumed_edges.extend(comp_ids_to_edge_indices(&info.feedback_comp_ids));
                    consumed_edges.extend(comp_ids_to_edge_indices(&info.ground_leg_comp_ids));
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

    (stages, diode_paired, consumed_edges)
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
            ref jfet_id, cap, ..
        } = info.feedback_kind
        {
            map.insert(jfet_id.clone(), AllpassPairing { cap, sample_rate });
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
/// Returns (tree, feedback_pot_id, feedback_edge_indices) or None if the
/// network is too simple or SP reduction fails.
fn build_feedback_tree(
    info: &OpAmpFeedbackInfo,
    graph: &CircuitGraph,
    pedal: &PedalDef,
    sample_rate: f64,
) -> Option<(DynNode, Option<String>, Vec<usize>)> {
    if info.feedback_comp_ids.len() <= 1 {
        #[cfg(test)]
        eprintln!(
            "[BUILD_FB_TREE] {} skipped: only {} feedback comps",
            info.comp_id,
            info.feedback_comp_ids.len()
        );
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

    #[cfg(test)]
    eprintln!(
        "[BUILD_FB_TREE] {} feedback_comp_ids={:?} feedback_set={:?} feedback_edges={}",
        info.comp_id,
        info.feedback_comp_ids,
        feedback_set,
        feedback_edges.len()
    );

    if feedback_edges.len() < 2 {
        #[cfg(test)]
        eprintln!(
            "[BUILD_FB_TREE] {} skipped: only {} feedback edges",
            info.comp_id,
            feedback_edges.len()
        );
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

    #[cfg(test)]
    eprintln!(
        "[BUILD_FB_TREE] {} border_nodes={:?} (need exactly 2)",
        info.comp_id, border_nodes
    );

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
        &feedback_edges,
        &extra,
        &terminals,
        graph,
        sample_rate,
        &std::collections::HashMap::new(),
        |n| n,
        None,
    )
    .ok()?
    .0;

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

    Some((tree, feedback_pot_id, feedback_edges))
}

/// Build a 3-port WDF adaptor for a NonInverting opamp with reactive ground leg.
///
/// Port layout:
///   Port 0 (Zf): feedback subtree (neg → out)
///   Port 1 (Zg): ground-leg subtree (neg → gnd)
///   Port 2 (adapted): opamp root port
///
/// Returns (zf_tree, zg_tree, adaptor, zf_pot_id, recompute_data) or None if either tree fails.
fn build_3port_opamp(
    info: &OpAmpFeedbackInfo,
    graph: &CircuitGraph,
    _pedal: &PedalDef,
    sample_rate: f64,
) -> Option<(
    DynNode,
    DynNode,
    crate::tree::RTypeAdaptor,
    Option<String>,
    super::stage::OpAmpRecomputeData,
)> {
    use crate::tree::{MnaSystem, RTypeAdaptor, WdfPort};

    // Build Zf tree (feedback: neg → out)
    let fb_set: HashSet<&str> = info.feedback_comp_ids.iter().map(|s| s.as_str()).collect();
    let fb_edges: Vec<usize> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(_, e)| fb_set.contains(graph.components[e.comp_idx].id.as_str()))
        .map(|(i, _)| i)
        .collect();

    if fb_edges.len() < 2 {
        return None;
    }

    // Find border nodes for Zf
    let fb_nodes: HashSet<NodeId> = fb_edges
        .iter()
        .flat_map(|&i| {
            let e = &graph.edges[i];
            [e.node_a, e.node_b]
        })
        .collect();
    let mut fb_border: Vec<NodeId> = fb_nodes
        .iter()
        .filter(|&&n| {
            graph.edges.iter().any(|e| {
                (e.node_a == n || e.node_b == n)
                    && !fb_set.contains(graph.components[e.comp_idx].id.as_str())
            })
        })
        .copied()
        .collect();
    fb_border.sort();
    fb_border.dedup();

    #[cfg(test)]
    eprintln!("[3PORT] Zf border={:?} edges={}", fb_border, fb_edges.len());

    if fb_border.len() != 2 {
        return None;
    }

    let zf_tree = super::graph::graph_reduce(
        &fb_edges,
        &[],
        &fb_border,
        graph,
        sample_rate,
        &std::collections::HashMap::new(),
        |n| n,
        None,
    )
    .ok()?
    .0;

    // Build Zg tree (ground leg: neg → gnd)
    let gl_set: HashSet<&str> = info
        .ground_leg_comp_ids
        .iter()
        .map(|s| s.as_str())
        .collect();
    let gl_edges: Vec<usize> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(_, e)| gl_set.contains(graph.components[e.comp_idx].id.as_str()))
        .map(|(i, _)| i)
        .collect();

    if gl_edges.len() < 2 {
        return None;
    }

    let gl_nodes: HashSet<NodeId> = gl_edges
        .iter()
        .flat_map(|&i| {
            let e = &graph.edges[i];
            [e.node_a, e.node_b]
        })
        .collect();
    let mut gl_border: Vec<NodeId> = gl_nodes
        .iter()
        .filter(|&&n| {
            graph.edges.iter().any(|e| {
                (e.node_a == n || e.node_b == n)
                    && !gl_set.contains(graph.components[e.comp_idx].id.as_str())
            })
        })
        .copied()
        .collect();
    gl_border.sort();
    gl_border.dedup();

    #[cfg(test)]
    eprintln!("[3PORT] Zg border={:?} edges={}", gl_border, gl_edges.len());

    if gl_border.len() != 2 {
        return None;
    }

    let zg_tree = super::graph::graph_reduce(
        &gl_edges,
        &[],
        &gl_border,
        graph,
        sample_rate,
        &std::collections::HashMap::new(),
        |n| n,
        None,
    )
    .ok()?
    .0;

    let r_f = zf_tree.port_resistance();
    let r_g = zg_tree.port_resistance();
    // Adapted port: parallel combination for optimal reflection-free adaptation
    let r_adapted = (r_f * r_g) / (r_f + r_g);

    #[cfg(test)]
    eprintln!(
        "[3PORT] Rf={:.1} Rg={:.1} R_adapted={:.1}",
        r_f, r_g, r_adapted
    );

    // MNA: 2 nodes (neg=0, out=1). gnd = None (reference).
    // No component stamps — the WDF subtrees handle internal structure.
    // derive_scattering_matrix_general stamps port Thévenin resistances.
    let mna = MnaSystem::new(2, 0);

    let ports = vec![
        WdfPort {
            node_pos: Some(1),
            node_neg: Some(0),
            resistance: r_f,
        }, // Zf: out→neg
        WdfPort {
            node_pos: Some(0),
            node_neg: None,
            resistance: r_g,
        }, // Zg: neg→gnd
        WdfPort {
            node_pos: Some(1),
            node_neg: None,
            resistance: r_adapted,
        }, // adapted: out→gnd
    ];

    let scattering = mna.derive_scattering_matrix_general(&ports);

    #[cfg(test)]
    {
        eprintln!("[3PORT] S matrix:");
        for i in 0..3 {
            eprintln!(
                "  [{:.4} {:.4} {:.4}]",
                scattering[i * 3],
                scattering[i * 3 + 1],
                scattering[i * 3 + 2]
            );
        }
    }

    let adaptor = RTypeAdaptor::new(scattering, &[r_f, r_g, r_adapted]);

    // Find Zf feedback pot ID
    let zf_pot_id = info.feedback_comp_ids.iter().find_map(|id| {
        let base = id
            .strip_suffix("__aw")
            .or_else(|| id.strip_suffix("__wb"))
            .unwrap_or(id);
        if _pedal
            .components
            .iter()
            .any(|c| c.id == base && c.kind.pot_taper().is_some())
        {
            Some(base.to_string())
        } else {
            None
        }
    });

    // Recompute data for pot-driven scattering updates.
    // Port pairs match the 3-port layout: (pos, neg) for Zf, Zg, adapted.
    let port_pairs = vec![
        (Some(1usize), Some(0usize)), // Zf: out→neg (MNA indices in 2-node system)
        (Some(0usize), None),         // Zg: neg→gnd
        (Some(1usize), None),         // adapted: out→gnd
    ];
    let port_resistances = vec![r_f, r_g, r_adapted];
    let recompute_data = super::stage::OpAmpRecomputeData {
        mna,
        port_pairs,
        port_resistances,
    };

    Some((zf_tree, zg_tree, adaptor, zf_pot_id, recompute_data))
}

/// Get the base pot ID by stripping __aw/__wb suffixes.
fn pot_base_id(comp_id: &str) -> &str {
    comp_id
        .strip_suffix("__aw")
        .or_else(|| comp_id.strip_suffix("__wb"))
        .unwrap_or(comp_id)
}

/// Look up pot info for a component — handles both full pots and split halves (__aw/__wb).
/// Returns (taper, max_resistance) if this is a pot or split pot half.
fn lookup_pot_info(comp_id: &str, pedal: &PedalDef) -> Option<(PotTaper, f64)> {
    // Direct match: component itself has pot_taper
    let pedal_comp = pedal.components.iter().find(|c| c.id == comp_id);
    if let Some(pc) = pedal_comp {
        if let Some(taper) = pc.kind.pot_taper() {
            let max_r = pc.kind.resistance().map(|r| r * 2.0).unwrap_or(100_000.0);
            return Some((taper, max_r));
        }
    }
    // Split pot half: strip __aw/__wb and look up base pot
    let base = pot_base_id(comp_id);
    if base == comp_id {
        return None;
    } // No suffix stripped → not a split pot
    let base_comp = pedal.components.iter().find(|c| c.id == base)?;
    let taper = base_comp.kind.pot_taper()?;
    let max_r = base_comp
        .kind
        .resistance()
        .map(|r| r * 2.0)
        .unwrap_or(100_000.0);
    Some((taper, max_r))
}

/// Check if a component is a pot (including split halves).
fn is_pot_component(comp_id: &str, pedal: &PedalDef) -> bool {
    lookup_pot_info(comp_id, pedal).is_some()
}

/// Build an MNA-based opamp adaptor for Inverting opamps with reactive feedback.
///
/// Stamps fixed resistors into MNA as conductances. Each cap/inductor/pot becomes
/// a WDF leaf child port. The scattering matrix captures frequency-dependent
/// impedance relationships that the scalar OpAmpRoot gain cannot model.
///
/// When `info.input_comp_ids` is non-empty, R_in (input→neg) is added as an
/// additional VoltageSource port so the full inverting topology (Rf/Ri) is
/// captured in the scattering matrix. Signal enters via this VoltageSource child
/// each sample — the stage must set its voltage to `input` before scatter_up.
///
/// Returns (children, adaptor, recompute_data, feedback_pot_id, input_child_idx).
/// `input_child_idx` is `Some(i)` when an input VoltageSource child was added at
/// index `i` in `children`. The caller must drive this child each sample.
fn build_opamp_adaptor(
    info: &OpAmpFeedbackInfo,
    graph: &CircuitGraph,
    pedal: &PedalDef,
    sample_rate: f64,
) -> Option<(
    Vec<DynNode>,
    crate::tree::RTypeAdaptor,
    super::stage::OpAmpRecomputeData,
    Option<String>,
    Option<usize>,
)> {
    use crate::tree::{MnaSystem, RTypeAdaptor, WdfPort};
    use std::collections::HashMap;

    // Collect ALL feedback + ground-leg + input component IDs
    let all_ids: HashSet<&str> = info
        .feedback_comp_ids
        .iter()
        .chain(info.ground_leg_comp_ids.iter())
        .chain(info.input_comp_ids.iter())
        .map(|s| s.as_str())
        .collect();

    let edges: Vec<usize> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(_, e)| all_ids.contains(graph.components[e.comp_idx].id.as_str()))
        .map(|(i, _)| i)
        .collect();

    if edges.is_empty() {
        return None;
    }

    // Map nodes to MNA indices (gnd = None/reference)
    let gnd = graph.gnd_node;
    let mut node_set: HashSet<NodeId> = HashSet::new();
    for &i in &edges {
        let e = &graph.edges[i];
        if e.node_a != gnd {
            node_set.insert(e.node_a);
        }
        if e.node_b != gnd {
            node_set.insert(e.node_b);
        }
    }
    if info.neg_node != gnd {
        node_set.insert(info.neg_node);
    }
    if info.out_node != gnd {
        node_set.insert(info.out_node);
    }
    // Include input node so it participates in the MNA system.
    if !info.input_comp_ids.is_empty() && info.input_node != gnd {
        node_set.insert(info.input_node);
    }

    let nodes: Vec<NodeId> = node_set.into_iter().collect();
    let node_map: HashMap<NodeId, usize> = nodes.iter().enumerate().map(|(i, &n)| (n, i)).collect();
    let n_nodes = nodes.len();

    let to_mna = |n: NodeId| -> Option<usize> {
        if n == gnd {
            None
        } else {
            node_map.get(&n).copied()
        }
    };

    // Build MNA and WDF children
    let mut mna = MnaSystem::new(n_nodes, 0);
    let mut children: Vec<DynNode> = Vec::new();
    let mut child_ports: Vec<WdfPort> = Vec::new();
    let mut feedback_pot_id: Option<String> = None;
    let mut processed: HashSet<usize> = HashSet::new();

    // Separate input edges from feedback/ground-leg edges so they are handled differently.
    let input_ids_set: HashSet<&str> = info.input_comp_ids.iter().map(|s| s.as_str()).collect();

    for &edge_idx in &edges {
        if !processed.insert(edge_idx) {
            continue;
        }
        let e = &graph.edges[edge_idx];
        let comp = &graph.components[e.comp_idx];

        // Input-path edges: stamp into MNA as conductances (fixed resistors only).
        // They will be driven by the VoltageSource port added below.
        if input_ids_set.contains(comp.id.as_str()) {
            if let Some(res) = comp.kind.resistance() {
                let na = to_mna(e.node_a);
                let nb = to_mna(e.node_b);
                mna.stamp_resistor(na, nb, res);
            }
            continue;
        }

        #[cfg(test)]
        {
            let is_pot = lookup_pot_info(&comp.id, pedal).is_some();
            let is_cap = comp.kind.capacitance().is_some();
            let is_res = comp.kind.resistance().is_some();
            eprintln!(
                "[MNA_EDGE] id={} pot={} cap={} res={} na={} nb={}",
                comp.id, is_pot, is_cap, is_res, e.node_a, e.node_b
            );
        }
        let na = to_mna(e.node_a);
        let nb = to_mna(e.node_b);

        if let Some(cap) = comp.kind.capacitance() {
            let rp = 1.0 / (2.0 * sample_rate * cap);
            children.push(DynNode::Capacitor(Some(comp.id.clone()), cap, rp));
            child_ports.push(WdfPort {
                node_pos: na,
                node_neg: nb,
                resistance: rp,
            });
        } else if let Some((taper, max_r)) = lookup_pot_info(&comp.id, pedal) {
            // Pot (or split pot half) → WDF child port only.
            // NOT stamped into MNA — derive_scattering_matrix_general adds
            // port Thévenin resistance. Stamping G too would double-count.
            let pos = taper.apply(0.5);
            let r = (pos * max_r).max(1.0);
            children.push(DynNode::Pot(comp.id.clone(), max_r, 0.5, taper));
            child_ports.push(WdfPort {
                node_pos: na,
                node_neg: nb,
                resistance: r,
            });
            if feedback_pot_id.is_none() {
                let base_id = pot_base_id(&comp.id);
                feedback_pot_id = Some(base_id.to_string());
            }
        } else if let Some(res) = comp.kind.resistance() {
            mna.stamp_resistor(na, nb, res);
        }
        // TODO: inductor support
    }

    if children.is_empty() {
        return None; // No reactive/variable elements — scalar gain path handles this
    }

    // Add R_in as a VoltageSource port when input path is known.
    // The port connects input_node → neg_node. The WDF convention for a VS:
    //   reflected b = 2*V, port resistance = Ri.
    // We pick Ri from the input comp edges (use the first resistor found).
    let input_child_idx: Option<usize> = if !info.input_comp_ids.is_empty() {
        // Compute R_in from the graph edges in the input path.
        let ri_val: f64 = info
            .input_comp_ids
            .iter()
            .filter_map(|id| {
                graph
                    .edges
                    .iter()
                    .find(|e| graph.components[e.comp_idx].id == *id)
                    .and_then(|e| graph.components[e.comp_idx].kind.resistance())
            })
            .sum::<f64>()
            .max(1.0);
        let inp_mna = to_mna(info.input_node);
        let neg_mna = to_mna(info.neg_node);
        let idx = children.len();
        children.push(DynNode::VoltageSource(0.0, ri_val));
        child_ports.push(WdfPort {
            node_pos: inp_mna,
            node_neg: neg_mna,
            resistance: ri_val,
        });
        #[cfg(test)]
        eprintln!(
            "[MNA_INPUT_PORT] R_in={:.1} inp_node={:?} neg_node={:?} idx={}",
            ri_val, inp_mna, neg_mna, idx
        );
        Some(idx)
    } else {
        None
    };

    // Adapted port: out→gnd (the opamp output node)
    let out_mna = to_mna(info.out_node);
    let r_adapted = {
        let sum_inv: f64 = child_ports.iter().map(|p| 1.0 / p.resistance).sum();
        if sum_inv > 0.0 {
            1.0 / sum_inv
        } else {
            1000.0
        }
    };

    // All ports: children first, adapted last
    let mut all_ports = child_ports;
    all_ports.push(WdfPort {
        node_pos: out_mna,
        node_neg: None,
        resistance: r_adapted,
    });

    let all_r: Vec<f64> = all_ports.iter().map(|p| p.resistance).collect();
    let port_pairs: Vec<(Option<usize>, Option<usize>)> =
        all_ports.iter().map(|p| (p.node_pos, p.node_neg)).collect();

    #[cfg(test)]
    {
        eprintln!(
            "[MNA_OPAMP] {} n_nodes={} n_children={} R_adapted={:.1} input_child={:?}",
            info.comp_id,
            n_nodes,
            children.len(),
            r_adapted,
            input_child_idx,
        );
    }

    let scattering = mna.derive_scattering_matrix_general(&all_ports);
    let adaptor = RTypeAdaptor::new(scattering, &all_r);

    let recompute_data = super::stage::OpAmpRecomputeData {
        mna,
        port_pairs: port_pairs.clone(),
        port_resistances: all_r,
    };

    Some((
        children,
        adaptor,
        recompute_data,
        feedback_pot_id,
        input_child_idx,
    ))
}

/// Build a WDF constraint adaptor for an inverting opamp with reactive feedback.
///
/// Returns (zi_tree, zf_tree, feedback_pot_id, consumed_edge_indices) or None
/// if the subtrees can't be built (e.g., not enough edges, SP decomposition fails).
///
/// Zi = input network (from input node to neg node).
/// Zf = feedback network (from neg node to output node).
fn build_opamp_wdf_adaptor(
    info: &OpAmpFeedbackInfo,
    graph: &CircuitGraph,
    pedal: &PedalDef,
    sample_rate: f64,
    ri_fallback: f64,
) -> Option<(DynNode, DynNode, Option<String>, Vec<usize>)> {
    // ── Build Zf (feedback: neg → out) ──
    let fb_set: HashSet<&str> = info.feedback_comp_ids.iter().map(|s| s.as_str()).collect();
    let fb_edges: Vec<usize> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(_, e)| fb_set.contains(graph.components[e.comp_idx].id.as_str()))
        .map(|(i, _)| i)
        .collect();

    #[cfg(test)]
    eprintln!(
        "[WDF_OPAMP_BUILD] {} fb_comp_ids={:?} fb_edges={} input_comp_ids={:?}",
        info.comp_id,
        info.feedback_comp_ids,
        fb_edges.len(),
        info.input_comp_ids
    );

    if fb_edges.is_empty() {
        #[cfg(test)]
        eprintln!("[WDF_OPAMP_BUILD] {} BAIL: no feedback edges", info.comp_id);
        return None;
    }

    // Find border nodes for Zf
    let fb_nodes: HashSet<NodeId> = fb_edges
        .iter()
        .flat_map(|&i| {
            let e = &graph.edges[i];
            [e.node_a, e.node_b]
        })
        .collect();
    let mut fb_border: Vec<NodeId> = fb_nodes
        .iter()
        .filter(|&&n| {
            graph.edges.iter().any(|e| {
                (e.node_a == n || e.node_b == n)
                    && !fb_set.contains(graph.components[e.comp_idx].id.as_str())
            })
        })
        .copied()
        .collect();
    fb_border.sort();
    fb_border.dedup();

    #[cfg(test)]
    eprintln!(
        "[WDF_OPAMP_BUILD] {} Zf fb_border={:?}",
        info.comp_id, fb_border
    );
    if fb_border.len() != 2 {
        #[cfg(test)]
        eprintln!(
            "[WDF_OPAMP_BUILD] {} BAIL: Zf border len={} (need 2)",
            info.comp_id,
            fb_border.len()
        );
        return None;
    }

    let zf_tree = super::graph::graph_reduce(
        &fb_edges,
        &[],
        &fb_border,
        graph,
        sample_rate,
        &std::collections::HashMap::new(),
        |n| n,
        None,
    )
    .ok()?
    .0;

    #[cfg(test)]
    eprintln!(
        "[WDF_OPAMP_BUILD] {} Zf tree:\n{}",
        info.comp_id,
        dump_tree(&zf_tree, 1)
    );

    // ── Build Zi (input: input_node → neg) ──
    let input_set: HashSet<&str> = info.input_comp_ids.iter().map(|s| s.as_str()).collect();
    let input_edges: Vec<usize> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(_, e)| input_set.contains(graph.components[e.comp_idx].id.as_str()))
        .map(|(i, _)| i)
        .collect();

    // For the Zi tree: if there's only one input component (typical: just Ri),
    // create a simple resistor leaf. Otherwise, try graph_reduce.
    let zi_tree = if input_edges.len() == 1 {
        let e = &graph.edges[input_edges[0]];
        let comp = &graph.components[e.comp_idx];
        if let Some(res) = comp.kind.resistance() {
            // Zi = Series(VS, Ri) — voltage source drives through input resistor
            let vs = DynNode::VoltageSource(0.0, res);
            vs
        } else {
            return None;
        }
    } else if input_edges.len() > 1 {
        // Multi-component input path — try graph_reduce
        let input_nodes: HashSet<NodeId> = input_edges
            .iter()
            .flat_map(|&i| {
                let e = &graph.edges[i];
                [e.node_a, e.node_b]
            })
            .collect();
        let mut input_border: Vec<NodeId> = input_nodes
            .iter()
            .filter(|&&n| {
                graph.edges.iter().any(|e| {
                    (e.node_a == n || e.node_b == n)
                        && !input_set.contains(graph.components[e.comp_idx].id.as_str())
                })
            })
            .copied()
            .collect();
        input_border.sort();
        input_border.dedup();

        if input_border.len() != 2 {
            return None;
        }

        let zi_reduced = super::graph::graph_reduce(
            &input_edges,
            &[],
            &input_border,
            graph,
            sample_rate,
            &std::collections::HashMap::new(),
            |n| n,
            None,
        )
        .ok()?
        .0;

        // Wrap in Series(VS, zi_reduced) for signal injection
        let rp = zi_reduced.port_resistance();
        let vs = DynNode::VoltageSource(0.0, rp);
        DynNode::Series(Box::new(vs), Box::new(zi_reduced))
    } else {
        // No input edges identified by opamp analysis — use ri_fallback as VS port resistance.
        // This handles cases where the input resistor wasn't tracked (e.g., Klon U4).
        #[cfg(test)]
        eprintln!(
            "[WDF_OPAMP_BUILD] {} no input edges, using ri_fallback={:.1}",
            info.comp_id, ri_fallback
        );
        DynNode::VoltageSource(0.0, ri_fallback)
    };

    // Find feedback pot ID
    let feedback_pot_id = info.feedback_comp_ids.iter().find_map(|id| {
        let base = id
            .strip_suffix("__aw")
            .or_else(|| id.strip_suffix("__wb"))
            .unwrap_or(id);
        if pedal
            .components
            .iter()
            .any(|c| c.id == base && c.kind.pot_taper().is_some())
        {
            Some(base.to_string())
        } else {
            None
        }
    });

    let mut consumed = fb_edges;
    consumed.extend(input_edges);

    #[cfg(test)]
    eprintln!(
        "[WDF_OPAMP] {} built: Zi R={:.1}, Zf R={:.1}, pot={:?}, edges={}",
        info.comp_id,
        zi_tree.port_resistance(),
        zf_tree.port_resistance(),
        feedback_pot_id,
        consumed.len(),
    );

    Some((zi_tree, zf_tree, feedback_pot_id, consumed))
}

/// Build a WDF constraint adaptor for a non-inverting opamp.
///
/// Zi = ground leg (neg → gnd through Rg).
/// Zf = feedback network (neg → out through Rf, caps, pots).
fn build_opamp_wdf_adaptor_ni(
    info: &OpAmpFeedbackInfo,
    graph: &CircuitGraph,
    pedal: &PedalDef,
    sample_rate: f64,
) -> Option<(DynNode, DynNode, Option<String>, Vec<usize>)> {
    // ── Build Zf (feedback: neg → out) ──
    let fb_set: HashSet<&str> = info.feedback_comp_ids.iter().map(|s| s.as_str()).collect();
    let fb_edges: Vec<usize> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(_, e)| fb_set.contains(graph.components[e.comp_idx].id.as_str()))
        .map(|(i, _)| i)
        .collect();

    #[cfg(test)]
    eprintln!(
        "[WDF_NI_BUILD] {} fb_edges={} gl_comp_ids={:?}",
        info.comp_id,
        fb_edges.len(),
        info.ground_leg_comp_ids
    );

    if fb_edges.is_empty() {
        return None;
    }

    let fb_nodes: HashSet<NodeId> = fb_edges
        .iter()
        .flat_map(|&i| {
            let e = &graph.edges[i];
            [e.node_a, e.node_b]
        })
        .collect();
    let mut fb_border: Vec<NodeId> = fb_nodes
        .iter()
        .filter(|&&n| {
            graph.edges.iter().any(|e| {
                (e.node_a == n || e.node_b == n)
                    && !fb_set.contains(graph.components[e.comp_idx].id.as_str())
            })
        })
        .copied()
        .collect();
    fb_border.sort();
    fb_border.dedup();

    if fb_border.len() != 2 {
        #[cfg(test)]
        eprintln!(
            "[WDF_NI_BUILD] {} Zf border={:?} (need 2)",
            info.comp_id, fb_border
        );
        return None;
    }

    let zf_tree = super::graph::graph_reduce(
        &fb_edges,
        &[],
        &fb_border,
        graph,
        sample_rate,
        &std::collections::HashMap::new(),
        |n| n,
        None,
    )
    .ok()?
    .0;

    // ── Build Zi (ground leg: neg → gnd) ──
    let gl_set: HashSet<&str> = info
        .ground_leg_comp_ids
        .iter()
        .map(|s| s.as_str())
        .collect();
    let gl_edges: Vec<usize> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(_, e)| gl_set.contains(graph.components[e.comp_idx].id.as_str()))
        .map(|(i, _)| i)
        .collect();

    let zi_tree = if gl_edges.len() == 1 {
        // Single ground-leg component (typical: just Rg)
        let e = &graph.edges[gl_edges[0]];
        let comp = &graph.components[e.comp_idx];
        if let Some(res) = comp.kind.resistance() {
            DynNode::Resistor(Some(comp.id.clone()), res)
        } else if let Some(cap) = comp.kind.capacitance() {
            let rp = 1.0 / (2.0 * sample_rate * cap);
            DynNode::Capacitor(Some(comp.id.clone()), cap, rp)
        } else {
            return None;
        }
    } else if gl_edges.len() > 1 {
        let gl_nodes: HashSet<NodeId> = gl_edges
            .iter()
            .flat_map(|&i| {
                let e = &graph.edges[i];
                [e.node_a, e.node_b]
            })
            .collect();
        let mut gl_border: Vec<NodeId> = gl_nodes
            .iter()
            .filter(|&&n| {
                graph.edges.iter().any(|e| {
                    (e.node_a == n || e.node_b == n)
                        && !gl_set.contains(graph.components[e.comp_idx].id.as_str())
                })
            })
            .copied()
            .collect();
        gl_border.sort();
        gl_border.dedup();

        if gl_border.len() != 2 {
            return None;
        }

        super::graph::graph_reduce(
            &gl_edges,
            &[],
            &gl_border,
            graph,
            sample_rate,
            &std::collections::HashMap::new(),
            |n| n,
            None,
        )
        .ok()?
        .0
    } else {
        // No ground-leg edges — can't build Zi
        return None;
    };

    // Find feedback pot ID
    let feedback_pot_id = info
        .feedback_comp_ids
        .iter()
        .chain(info.ground_leg_comp_ids.iter())
        .find_map(|id| {
            let base = id
                .strip_suffix("__aw")
                .or_else(|| id.strip_suffix("__wb"))
                .unwrap_or(id);
            if pedal
                .components
                .iter()
                .any(|c| c.id == base && c.kind.pot_taper().is_some())
            {
                Some(base.to_string())
            } else {
                None
            }
        });

    let mut consumed = fb_edges;
    consumed.extend(gl_edges);

    #[cfg(test)]
    eprintln!(
        "[WDF_NI_BUILD] {} built: Zi R={:.1}, Zf R={:.1}, pot={:?}",
        info.comp_id,
        zi_tree.port_resistance(),
        zf_tree.port_resistance(),
        feedback_pot_id
    );

    Some((zi_tree, zf_tree, feedback_pot_id, consumed))
}

/// Quick recursive tree dumper for diagnostics.
fn dump_tree(node: &DynNode, depth: usize) -> String {
    let indent = "  ".repeat(depth);
    match node {
        DynNode::Leaf(leaf) => {
            let id = leaf.comp_id().unwrap_or("?");
            format!(
                "{}Leaf({} '{}', R={:.1})",
                indent,
                leaf.type_tag(),
                id,
                leaf.port_resistance()
            )
        }
        DynNode::Binary {
            kind,
            left,
            right,
            gamma,
            ..
        } => {
            let kind_str = match kind {
                super::dyn_node::BinaryKind::Series => "Series",
                super::dyn_node::BinaryKind::Parallel => "Parallel",
            };
            format!(
                "{}{kind_str}(γ={gamma:.4}, R={:.1})\n{}\n{}",
                indent,
                node.port_resistance(),
                dump_tree(left, depth + 1),
                dump_tree(right, depth + 1)
            )
        }
        _ => format!("{}Other(R={:.1})", indent, node.port_resistance()),
    }
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
