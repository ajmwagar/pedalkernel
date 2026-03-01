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

use super::graph::{CircuitGraph, NodeId, OpAmpFeedbackInfo, OpAmpFeedbackKind};
use super::stage::WdfStage;

/// Result of op-amp analysis.
pub(super) struct OpAmpAnalysis {
    /// Detected feedback loops (unity-gain, inverting, non-inverting).
    pub(super) feedback_loops: Vec<OpAmpFeedbackInfo>,
    /// IDs of op-amps that have feedback (excludes standalone op-amps).
    pub(super) feedback_opamp_ids: HashSet<String>,
    /// IDs of unity-gain op-amps (for JFET pairing).
    pub(super) unity_gain_opamp_ids: HashSet<String>,
    /// Map from pot component ID → gain modulation info:
    /// (stage_idx, ri, fixed_series_r, max_pot_r, parallel_fixed_r, is_inverting)
    pub(super) pot_map: HashMap<String, (usize, f64, f64, f64, Option<f64>, bool)>,
}

/// Run op-amp feedback analysis on the circuit.
pub(super) fn analyze_opamps(
    graph: &CircuitGraph,
    pedal: &PedalDef,
) -> OpAmpAnalysis {
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
        pot_map: HashMap::new(), // Populated during stage building (build.rs)
    }
}

/// Build op-amp gain stages from feedback loop analysis.
///
/// Creates WDF stages for inverting and non-inverting op-amps.
/// Returns the stages and updates the pot_map for runtime gain modulation.
pub(super) fn build_opamp_feedback_stages(
    analysis: &mut OpAmpAnalysis,
    existing_stage_count: usize,
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
                let stage_idx = existing_stage_count + stages.len();
                if let Some((pot_id, max_pot_r, fixed_series_r, parallel_fixed_r)) = rf_pot {
                    analysis.pot_map.insert(
                        pot_id.clone(),
                        (stage_idx, *ri, *fixed_series_r, *max_pot_r, *parallel_fixed_r, true),
                    );
                }

                let model = OpAmpModel::from_opamp_type(&info.opamp_type);
                let gain = rf / ri;
                let mut root = OpAmpRoot::new_inverting(model, gain);
                root.set_sample_rate(sample_rate);

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

                let tree = DynNode::VoltageSource {
                    voltage: 0.0,
                    rp: 10_000.0,
                };

                stages.push(WdfStage {
                    tree,
                    root: RootKind::OpAmp(root),
                    compensation: 1.0,
                    oversampler: crate::oversampling::Oversampler::new(oversampling),
                    base_diode_model: None,
                    paired_opamp: None,
                    dc_block: None,
                    is_source_follower: false,
                    prev_source_voltage: 0.0,
                });
            }
            OpAmpFeedbackKind::NonInverting { rf, ri, rf_pot } => {
                let stage_idx = existing_stage_count + stages.len();
                if let Some((pot_id, max_pot_r, fixed_series_r, parallel_fixed_r)) = rf_pot {
                    analysis.pot_map.insert(
                        pot_id.clone(),
                        (stage_idx, *ri, *fixed_series_r, *max_pot_r, *parallel_fixed_r, false),
                    );
                }

                let model = OpAmpModel::from_opamp_type(&info.opamp_type);
                let gain = 1.0 + (rf / ri);
                let mut root = OpAmpRoot::new_non_inverting(model, gain);
                root.set_sample_rate(sample_rate);

                let default_supply = 9.0_f64;
                let v_max = (default_supply / 2.0 - 1.5).max(0.5);
                root.set_v_max(v_max);

                let tree = DynNode::VoltageSource {
                    voltage: 0.0,
                    rp: 10_000.0,
                };

                stages.push(WdfStage {
                    tree,
                    root: RootKind::OpAmp(root),
                    compensation: 1.0,
                    oversampler: crate::oversampling::Oversampler::new(oversampling),
                    base_diode_model: None,
                    paired_opamp: None,
                    dc_block: None,
                    is_source_follower: false,
                    prev_source_voltage: 0.0,
                });
            }
        }
    }

    stages
}

/// Build a queue of unity-gain op-amp roots for pairing with JFET stages.
pub(super) fn build_unity_gain_queue(
    analysis: &OpAmpAnalysis,
    sample_rate: f64,
) -> Vec<OpAmpRoot> {
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

/// Build standalone op-amp stages (no feedback detected).
pub(super) fn build_standalone_opamp_stages(
    pedal: &PedalDef,
    feedback_opamp_ids: &HashSet<String>,
    sample_rate: f64,
) -> Vec<super::compiled::OpAmpStage> {
    let mut opamp_stages = Vec::new();
    for comp in &pedal.components {
        if let ComponentKind::OpAmp(ot) = &comp.kind {
            if !ot.is_ota() && !feedback_opamp_ids.contains(&comp.id) {
                let model = OpAmpModel::from_opamp_type(ot);
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
