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

use super::component::Component;
use super::graph::{CircuitGraph, OpAmpFeedbackInfo, OpAmpFeedbackKind};
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

/// Build op-amp gain stages from feedback loop analysis.
///
/// Creates WDF stages for inverting and non-inverting op-amps.
/// Sets `FeedbackConfig` on the OpAmpRoot and `feedback_pot_id` on the stage
/// so the stage self-manages gain updates when feedback pots change.
pub(super) fn build_opamp_feedback_stages(
    analysis: &OpAmpAnalysis,
    pedal: &PedalDef,
    _existing_stage_count: usize,
    sample_rate: f64,
    oversampling: crate::oversampling::OversamplingFactor,
) -> Vec<WdfStage> {
    use super::dyn_node::DynNode;
    use super::stage::RootKind;

    let mut stages = Vec::new();

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

                if let Some(diode_type) = feedback_diode {
                    let diode_vf = match diode_type {
                        DiodeType::Silicon => 0.6,
                        DiodeType::Germanium => 0.3,
                        DiodeType::Led => 1.6,
                    };
                    root.set_soft_clip(diode_vf);
                }

                let vs = DynNode::VoltageSource {
                    voltage: 0.0,
                    rp: 10_000.0,
                };
                let tree = if let Some((pot_id, max_pot_r, _fixed_series_r, _parallel_fixed_r)) = rf_pot {
                    let taper = lookup_pot_taper(pedal, pot_id);
                    let initial_pos = 0.5;
                    let tapered = taper.apply(initial_pos);
                    let pot_rp = (tapered * max_pot_r).max(1.0);
                    let pot = DynNode::Pot {
                        comp_id: pot_id.clone(),
                        max_resistance: *max_pot_r,
                        position: initial_pos,
                        taper,
                        rp: pot_rp,
                    };
                    let r_vs = vs.port_resistance();
                    let r_pot = pot.port_resistance();
                    let rp = r_vs + r_pot;
                    DynNode::Series {
                        gamma: r_vs / rp,
                        left: Box::new(vs),
                        right: Box::new(pot),
                        rp,
                        b1: 0.0,
                        b2: 0.0,
                    }
                } else {
                    vs
                };

                stages.push(WdfStage {
                    tree,
                    root: RootKind::OpAmp(root),
                    compensation: 1.0,
                    oversampler: crate::oversampling::Oversampler::new(oversampling),
                    base_diode_model: None,
                    paired_opamp: None,
                    allpass_feedback: None,
                    dc_block: None,
                    is_source_follower: false,
                    prev_source_voltage: 0.0,
                    signal_flow_distance: 0,
                    transformer_gain: 1.0,
                    injection_node_id: usize::MAX,
                    output_node_id: usize::MAX,
                    is_trigger_voice: false,
                    sample_counter: 0,
                    root_comp_id: String::new(),
                    is_supply_driven: false,
                    feedback_pot_id,
                });
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

                let vs = DynNode::VoltageSource {
                    voltage: 0.0,
                    rp: 10_000.0,
                };
                // Include feedback pot in tree so binding finds it via has_pot().
                let active_pot = rf_pot.as_ref().or(ri_pot.as_ref());
                let tree = if let Some((pot_id, max_pot_r, _fixed_series_r, _parallel_fixed_r)) = active_pot {
                    let taper = lookup_pot_taper(pedal, pot_id);
                    let initial_pos = 0.5;
                    let tapered = taper.apply(initial_pos);
                    let pot_rp = (tapered * max_pot_r).max(1.0);
                    let pot = DynNode::Pot {
                        comp_id: pot_id.clone(),
                        max_resistance: *max_pot_r,
                        position: initial_pos,
                        taper,
                        rp: pot_rp,
                    };
                    let r_vs = vs.port_resistance();
                    let r_pot = pot.port_resistance();
                    let rp = r_vs + r_pot;
                    DynNode::Series {
                        gamma: r_vs / rp,
                        left: Box::new(vs),
                        right: Box::new(pot),
                        rp,
                        b1: 0.0,
                        b2: 0.0,
                    }
                } else {
                    vs
                };

                stages.push(WdfStage {
                    tree,
                    root: RootKind::OpAmp(root),
                    compensation: 1.0,
                    oversampler: crate::oversampling::Oversampler::new(oversampling),
                    base_diode_model: None,
                    paired_opamp: None,
                    allpass_feedback: None,
                    dc_block: None,
                    is_source_follower: false,
                    prev_source_voltage: 0.0,
                    signal_flow_distance: 0,
                    transformer_gain: 1.0,
                    injection_node_id: usize::MAX,
                    output_node_id: usize::MAX,
                    is_trigger_voice: false,
                    sample_counter: 0,
                    root_comp_id: String::new(),
                    is_supply_driven: false,
                    feedback_pot_id,
                });
            }
            OpAmpFeedbackKind::AllpassJfet { .. } => {
                // Handled during JFET stage building in build_stages().
            }
        }
    }

    stages
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

/// Look up the taper of a potentiometer from the pedal definition.
fn lookup_pot_taper(pedal: &PedalDef, pot_id: &str) -> PotTaper {
    pedal
        .components
        .iter()
        .find(|c| c.id == pot_id)
        .and_then(|c| c.kind.pot_taper())
        .unwrap_or(PotTaper::B)
}
