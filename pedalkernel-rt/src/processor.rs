//! Compiled pedal processor: the runtime audio processing chain.

extern crate alloc;
use alloc::{
    format,
    string::{String, ToString},
    sync::Arc,
    vec::Vec,
};

use crate::boundary_math::{
    push_differential_voltage_drives_for_ports, BoundaryIncidentDrive, PortVoltage,
};
use crate::elements::*;
use crate::loading::{ImpedanceModel, InterstageLoading};
use crate::metering::{MetricsAccumulator, MetricsRingBuffer, UiMetrics};
use crate::oversampling::OversamplingFactor;
use crate::thermal::ThermalModel;
use crate::{PedalProcessor, Wave};
use hashbrown::HashMap;

use crate::stage::{
    IirStage, MultiNlStage, NlDeviceGroupKind, NlDeviceKind, PushPullStage, RootKind,
    SerialDelayedFeedbackStage, SidechainProcessor, StateSpaceStage, SubcircuitProcessor, WdfStage,
};

/// A single processing stage in the compiled pedal.
///
/// Owns its data directly — no index indirection. The `stages` vec
/// on [`CompiledPedal`] holds these in processing order.
#[allow(clippy::large_enum_variant)] // boxing WdfStage would require 58+ callsite changes; deferred to tech-debt
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Stage {
    Wdf(WdfStage),
    MultiNl(MultiNlStage),
    Iir(IirStage),
    StateSpace(StateSpaceStage),
    BlackFeedback(crate::stage::BlackFeedbackStage),
    #[cfg_attr(feature = "serde", serde(alias = "BlockwiseKMethod"))]
    Blockwise(crate::stage::BlockwiseStage),
    KMethod {
        block: crate::stage::KMethodBlock,
        ports: Vec<(crate::stage::OwnedPortRole, PortBinding)>,
    },
    SerialDelayedFeedback(SerialDelayedFeedbackStage),
}

impl Clone for Stage {
    fn clone(&self) -> Self {
        match self {
            Stage::Blockwise(stage) => Stage::Blockwise(stage.clone()),
            Stage::KMethod { block, ports } => Stage::KMethod {
                block: block.clone(),
                ports: ports.clone(),
            },
            _ => panic!("only blockwise recursive stages are cloneable"),
        }
    }
}

impl core::fmt::Debug for Stage {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Stage::Wdf(_) => write!(f, "Stage::Wdf(..)"),
            Stage::MultiNl(_) => write!(f, "Stage::MultiNl(..)"),
            Stage::Iir(_) => write!(f, "Stage::Iir(..)"),
            Stage::StateSpace(_) => write!(f, "Stage::StateSpace(..)"),
            Stage::BlackFeedback(_) => write!(f, "Stage::BlackFeedback(..)"),
            Stage::Blockwise(_) => write!(f, "Stage::Blockwise(..)"),
            Stage::KMethod { .. } => write!(f, "Stage::KMethod(..)"),
            Stage::SerialDelayedFeedback(_) => {
                write!(f, "Stage::SerialDelayedFeedback(..)")
            }
        }
    }
}

impl Stage {
    /// Whether this stage is excluded from the serial audio chain
    /// (static bias networks, K-method blocks driven via route plans, ...).
    pub fn bypass_serial(&self) -> bool {
        match self {
            Stage::Wdf(w) => w.bypass_serial,
            Stage::MultiNl(m) => m.bypass_serial,
            Stage::Iir(i) => i.bypass_serial,
            Stage::StateSpace(s) => s.bypass_serial,
            Stage::BlackFeedback(b) => b.bypass_serial,
            Stage::Blockwise(k) => k.bypass_serial,
            Stage::KMethod { .. } => true,
            Stage::SerialDelayedFeedback(s) => s.bypass_serial,
        }
    }

    pub fn k_method_ports(&self) -> &[(crate::stage::OwnedPortRole, PortBinding)] {
        match self {
            Stage::KMethod { ports, .. } => ports.as_slice(),
            _ => &[],
        }
    }

    // ── Stage-graph diagnostic accessors ──────────────────────────────────
    //
    // These surface the compiled stage structure for machine-readable export
    // (see `CompiledPedal::stage_graph_json`). They are read-only and do not
    // affect processing.

    /// Stable variant name used as the stage-graph node `kind`.
    pub fn graph_kind(&self) -> &'static str {
        match self {
            Stage::Wdf(_) => "Wdf",
            Stage::MultiNl(_) => "MultiNl",
            Stage::Iir(_) => "Iir",
            Stage::StateSpace(_) => "StateSpace",
            Stage::BlackFeedback(_) => "BlackFeedback",
            Stage::Blockwise(_) => "Blockwise",
            Stage::KMethod { .. } => "KMethod",
            Stage::SerialDelayedFeedback(_) => "SerialDelayedFeedback",
        }
    }

    /// Human-readable label describing the stage's contents.
    ///
    /// For WDF/MultiNl stages this surfaces the per-stage `debug_label`
    /// (component names) in debug builds; in release builds it falls back to
    /// a structural descriptor (root kind, NL device names) so the label is
    /// always populated.
    pub fn graph_label(&self) -> alloc::string::String {
        use alloc::string::ToString;
        match self {
            Stage::Wdf(wdf) => {
                #[cfg(debug_assertions)]
                if !wdf.debug_label.is_empty() {
                    return wdf.debug_label.clone();
                }
                format!("WDF root={}", wdf.root.kind_name())
            }
            Stage::MultiNl(mnl) => {
                #[cfg(debug_assertions)]
                if !mnl.debug_label.is_empty() {
                    return mnl.debug_label.clone();
                }
                let mut names = alloc::vec::Vec::new();
                if let Some(ref dg) = mnl.device_groups {
                    for g in &dg.groups {
                        names.push(g.debug_name());
                    }
                } else {
                    for d in &mnl.nl_devices {
                        names.push(d.debug_name());
                    }
                }
                format!("MultiNL [{}]", names.join(","))
            }
            Stage::Iir(_) => "IIR biquad".to_string(),
            Stage::StateSpace(_) => "StateSpace LTI".to_string(),
            Stage::BlackFeedback(_) => "BlackFeedback".to_string(),
            Stage::Blockwise(k) => k.debug_label(),
            Stage::KMethod { .. } => "KMethod child".to_string(),
            Stage::SerialDelayedFeedback(s) => {
                format!("SerialDelayedFeedback ({} rungs)", s.stages.len())
            }
        }
    }

    /// Signal-flow distance (BFS distance from input) used to order stages.
    /// Returns `None` for stages that do not carry a flow distance.
    pub fn flow_order(&self) -> Option<usize> {
        match self {
            Stage::Wdf(w) => Some(w.signal_flow_distance),
            Stage::MultiNl(m) => Some(m.signal_flow_distance),
            Stage::Iir(i) => Some(i.signal_flow_distance),
            Stage::StateSpace(s) => Some(s.signal_flow_distance),
            Stage::BlackFeedback(b) => Some(b.signal_flow_distance),
            Stage::Blockwise(k) => Some(k.signal_flow_distance),
            Stage::SerialDelayedFeedback(s) => Some(s.signal_flow_distance),
            Stage::KMethod { .. } => None,
        }
    }

    /// Number of feedback ports this stage exposes.
    ///
    /// - Blockwise: explicit internal feedback drives (`feedback_port_map`).
    /// - MultiNl: 1 when an in-loop feedback op-amp is present.
    /// - Wdf: 1 when a paired op-amp / all-pass feedback path is present.
    /// - SerialDelayedFeedback: 1 (inherently a delayed feedback loop).
    pub fn feedback_port_count(&self) -> usize {
        match self {
            Stage::Blockwise(k) => k.feedback_port_map.len(),
            Stage::MultiNl(m) => usize::from(m.feedback_opamp.is_some()),
            Stage::Wdf(w) => usize::from(
                w.paired_opamp.is_some()
                    || w.allpass_feedback.is_some()
                    || w.allpass_direct.is_some(),
            ),
            Stage::SerialDelayedFeedback(_) => 1,
            _ => 0,
        }
    }

    /// Number of nonlinear roots/devices co-solved in this stage.
    ///
    /// - Wdf: 1 when the WDF root is nonlinear, else 0.
    /// - MultiNl: number of co-solved NL *devices* — the cross-coupled device
    ///   groups when grouped (e.g. 3 BJTs → 3), else the raw NL port count
    ///   (`n_nl`). This is the key signal for whether N transistors are fused
    ///   into one stage or split across stages.
    /// - Blockwise: number of NL sub-stage blocks.
    pub fn nl_root_count(&self) -> usize {
        match self {
            Stage::Wdf(w) => usize::from(w.root.is_nonlinear()),
            Stage::MultiNl(m) => match &m.device_groups {
                Some(dg) => dg.groups.len(),
                None => m.n_nl,
            },
            Stage::Blockwise(k) => k.sub_stages.len(),
            Stage::SerialDelayedFeedback(s) => s.stages.len(),
            _ => 0,
        }
    }

    /// Raw nonlinear *port* count (pre-grouping).
    ///
    /// For MultiNl this is `n_nl` (e.g. 6 for three 2-port BJTs); for grouped
    /// stages it differs from [`nl_root_count`](Self::nl_root_count), which
    /// counts physical devices. Surfaced separately so diagnostics can see both
    /// the device count and the solver's port dimension.
    pub fn nl_port_count(&self) -> usize {
        match self {
            Stage::Wdf(w) => usize::from(w.root.is_nonlinear()),
            Stage::MultiNl(m) => m.n_nl,
            Stage::Blockwise(k) => k.sub_stages.len(),
            Stage::SerialDelayedFeedback(s) => s.stages.len(),
            _ => 0,
        }
    }

    pub fn owned_port_ids(&self) -> Vec<usize> {
        let mut ids = Vec::new();
        for (_, binding) in self.k_method_ports() {
            if !ids.contains(&binding.local_port) {
                ids.push(binding.local_port);
            }
        }
        ids
    }

    pub fn is_empty(&self) -> bool {
        self.k_method_ports().is_empty()
    }

    /// Declarative input bindings exposed by this stage.
    ///
    /// This reports current boundary metadata only. It does not imply a
    /// processing strategy; containers can build route tables from these ids.
    pub fn ins(&self) -> Vec<PortBinding> {
        match self {
            Stage::Wdf(wdf) => wdf
                .boundary_bindings
                .iter()
                .enumerate()
                .filter_map(|(idx, binding)| {
                    matches!(
                        binding.direction,
                        crate::stage::WdfBoundaryDirection::Input
                            | crate::stage::WdfBoundaryDirection::Control
                    )
                    .then_some(PortBinding::new(BindingId::new(binding.node_id), idx))
                })
                .collect(),
            Stage::Blockwise(bkm) => bkm.ins(),
            Stage::KMethod { ports, .. } => ports.iter().map(|(_, binding)| *binding).collect(),
            Stage::Iir(iir) => iir.ins(),
            Stage::StateSpace(ss) => ss.ins(),
            _ => Vec::new(),
        }
    }

    /// Declarative output bindings exposed by this stage.
    pub fn outs(&self) -> Vec<PortBinding> {
        match self {
            Stage::Wdf(wdf) => wdf
                .boundary_bindings
                .iter()
                .enumerate()
                .filter_map(|(idx, binding)| {
                    (binding.direction == crate::stage::WdfBoundaryDirection::Output)
                        .then_some(PortBinding::new(BindingId::new(binding.node_id), idx))
                })
                .collect(),
            Stage::Blockwise(bkm) => bkm.outs(),
            Stage::KMethod { ports, .. } => ports.iter().map(|(_, binding)| *binding).collect(),
            Stage::Iir(iir) => iir.outs(),
            Stage::StateSpace(ss) => ss.outs(),
            _ => Vec::new(),
        }
    }

    /// Return the runtime target for a pot owned by this stage.
    ///
    /// This is the control-binding capability check. Keep stage-specific pot
    /// discovery here so compiler-side binding cannot recognize a pot without
    /// the matching runtime mutation path living next to it.
    pub fn control_target_for_pot(&self, stage_idx: usize, comp_id: &str) -> Option<ControlTarget> {
        let aw_id = format!("{comp_id}__aw");
        let wb_id = format!("{comp_id}__wb");

        match self {
            Stage::Wdf(wdf) => {
                let in_wdf = wdf.has_pot(comp_id) || wdf.has_pot(&aw_id) || wdf.has_pot(&wb_id);
                let is_feedback_pot = wdf.feedback_pot_id.as_deref() == Some(comp_id);
                let is_feedback_ri_pot = wdf.feedback_ri_pot_id.as_deref() == Some(comp_id);

                if in_wdf || is_feedback_pot || is_feedback_ri_pot {
                    Some(ControlTarget::PotInStage(stage_idx))
                } else {
                    None
                }
            }
            Stage::MultiNl(mnl) => {
                for child in &mnl.pot_children {
                    if child.get_pot_position(comp_id).is_some()
                        || child.get_pot_position(&aw_id).is_some()
                        || child.get_pot_position(&wb_id).is_some()
                    {
                        return Some(ControlTarget::PotInMultiNlStage(stage_idx, 0));
                    }
                }
                None
            }
            Stage::Iir(iir) => iir
                .has_pot(comp_id)
                .then_some(ControlTarget::PotInIirStage(stage_idx)),
            Stage::BlackFeedback(bf) => bf
                .has_pot(comp_id)
                .then_some(ControlTarget::PotInBlackFeedbackStage(stage_idx)),
            Stage::Blockwise(bkm) => {
                for block in bkm.k_method_blocks() {
                    if block.tree.get_pot_position(comp_id).is_some()
                        || block.tree.get_pot_position(&aw_id).is_some()
                        || block.tree.get_pot_position(&wb_id).is_some()
                    {
                        return Some(ControlTarget::PotInStage(stage_idx));
                    }
                }
                if bkm.has_coupling_pot(comp_id) {
                    return Some(ControlTarget::PotInStage(stage_idx));
                }
                None
            }
            Stage::SerialDelayedFeedback(serial) => serial
                .has_pot(comp_id)
                .then_some(ControlTarget::PotInStage(stage_idx)),
            Stage::StateSpace(ss) => ss
                .has_pot(comp_id)
                .then_some(ControlTarget::PotInStage(stage_idx)),
            Stage::KMethod { .. } => None,
        }
    }

    /// Apply a normalized pot value to this stage if it owns `comp_id`.
    ///
    /// Returns true when the stage accepted the control. Each stage handles
    /// its own recompute/update contract here.
    pub fn set_control_pot(&mut self, comp_id: &str, value: crate::Wave) -> bool {
        match self {
            Stage::Wdf(wdf) => {
                let changed = wdf.set_pot(comp_id, value);
                if changed {
                    wdf.flush_recompute();
                }
                changed
            }
            Stage::Iir(iir) => {
                if iir.has_pot(comp_id) {
                    iir.set_pot(comp_id, value);
                    true
                } else {
                    false
                }
            }
            Stage::MultiNl(mnl) => {
                let changed = mnl.set_pot(comp_id, value);
                if changed {
                    mnl.flush_recompute();
                }
                changed
            }
            Stage::BlackFeedback(bf) => {
                if bf.has_pot(comp_id) {
                    bf.set_pot(comp_id, value);
                    bf.update_ri_from_pot(comp_id, value);
                    true
                } else {
                    false
                }
            }
            Stage::Blockwise(bkm) => bkm.set_pot(comp_id, value),
            Stage::SerialDelayedFeedback(serial) => serial.set_pot(comp_id, value),
            Stage::StateSpace(ss) => {
                if ss.has_pot(comp_id) {
                    ss.set_pot(comp_id, value);
                    true
                } else {
                    false
                }
            }
            Stage::KMethod { .. } => false,
        }
    }

    /// Get a reference to the inner WdfStage, if this is a Wdf variant.
    pub fn as_wdf(&self) -> Option<&WdfStage> {
        if let Stage::Wdf(w) = self {
            Some(w)
        } else {
            None
        }
    }
    /// Get a mutable reference to the inner WdfStage, if this is a Wdf variant.
    pub fn as_wdf_mut(&mut self) -> Option<&mut WdfStage> {
        if let Stage::Wdf(w) = self {
            Some(w)
        } else {
            None
        }
    }
    /// Get a reference to the inner MultiNlStage, if this is a MultiNl variant.
    pub fn as_multi_nl(&self) -> Option<&MultiNlStage> {
        if let Stage::MultiNl(m) = self {
            Some(m)
        } else {
            None
        }
    }
    /// Get a mutable reference to the inner MultiNlStage, if this is a MultiNl variant.
    pub fn as_multi_nl_mut(&mut self) -> Option<&mut MultiNlStage> {
        if let Stage::MultiNl(m) = self {
            Some(m)
        } else {
            None
        }
    }
    /// Get a reference to the inner IirStage, if this is an Iir variant.
    pub fn as_iir(&self) -> Option<&IirStage> {
        if let Stage::Iir(i) = self {
            Some(i)
        } else {
            None
        }
    }
    /// Get a mutable reference to the inner IirStage.
    pub fn as_iir_mut(&mut self) -> Option<&mut IirStage> {
        if let Stage::Iir(i) = self {
            Some(i)
        } else {
            None
        }
    }
    /// Get a reference to the inner StateSpaceStage.
    pub fn as_state_space(&self) -> Option<&StateSpaceStage> {
        if let Stage::StateSpace(s) = self {
            Some(s)
        } else {
            None
        }
    }
    /// Get a mutable reference to the inner StateSpaceStage.
    pub fn as_state_space_mut(&mut self) -> Option<&mut StateSpaceStage> {
        if let Stage::StateSpace(s) = self {
            Some(s)
        } else {
            None
        }
    }
    /// Get a reference to the inner BlackFeedbackStage.
    pub fn as_black_feedback(&self) -> Option<&crate::stage::BlackFeedbackStage> {
        if let Stage::BlackFeedback(b) = self {
            Some(b)
        } else {
            None
        }
    }
    /// Get a mutable reference to the inner BlackFeedbackStage.
    pub fn as_black_feedback_mut(&mut self) -> Option<&mut crate::stage::BlackFeedbackStage> {
        if let Stage::BlackFeedback(b) = self {
            Some(b)
        } else {
            None
        }
    }
}

/// Legacy alias — being migrated to `Stage`.
pub type StageRef = Stage;

/// Return type for [`CompiledPedal::multi_nl_debug_info`]:
/// `(stage_index, n_nl, scattering_flat, adapted_resistance, dc_bias)`.
pub type MultiNlDebugInfo = (
    usize,
    usize,
    Vec<crate::Wave>,
    crate::Wave,
    Vec<crate::Wave>,
);

#[cfg(test)]
mod tests {
    use alloc::boxed::Box;
    use alloc::string::ToString;
    use alloc::vec;

    use super::{ControlTarget, Stage};
    use crate::boundary_math::{MnaNodeId, OnePort, OnePortKind, PortTerminals};
    use crate::dyn_node::DynNode;
    use crate::oversampling::{Oversampler, OversamplingFactor};
    use crate::pot_taper::PotTaper;
    use crate::route::{BindingId, PortBinding};
    use crate::stage::{
        BlackFeedbackStage, IirData, IirPotBinding, IirStage, RootKind, StateSpaceData,
        StateSpaceStage, WdfBoundaryBinding, WdfBoundaryDirection, WdfStage,
    };

    #[test]
    fn stage_exposes_declarative_wdf_boundary_bindings() {
        let mut wdf = WdfStage::new(
            DynNode::VoltageSource(0.0, 1.0),
            RootKind::ShortCircuit,
            Oversampler::new(OversamplingFactor::X1),
        );
        wdf.boundary_bindings = vec![
            WdfBoundaryBinding {
                label: "base_left".to_string(),
                node_id: 10,
                direction: WdfBoundaryDirection::Input,
            },
            WdfBoundaryBinding {
                label: "collector_left".to_string(),
                node_id: 20,
                direction: WdfBoundaryDirection::Output,
            },
            WdfBoundaryBinding {
                label: "emitter_tail".to_string(),
                node_id: 30,
                direction: WdfBoundaryDirection::Control,
            },
        ];
        let stage = Stage::Wdf(wdf);

        let ins = stage.ins();
        let outs = stage.outs();

        assert_eq!(ins.len(), 2);
        assert_eq!(ins[0].binding_id.get(), 10);
        assert_eq!(ins[0].local_port, 0);
        assert_eq!(ins[1].binding_id.get(), 30);
        assert_eq!(ins[1].local_port, 2);
        assert_eq!(outs.len(), 1);
        assert_eq!(outs[0].binding_id.get(), 20);
        assert_eq!(outs[0].local_port, 1);
    }

    #[test]
    fn route_is_only_binding_id_data() {
        let route = crate::route::Route::new(
            crate::route::BindingId::new(20),
            crate::route::BindingId::new(10),
        );

        assert_eq!(route.from.get(), 20);
        assert_eq!(route.to.get(), 10);
    }

    #[test]
    fn stage_owned_wdf_pot_discovery_and_mutation() {
        let tree = DynNode::Series(
            Box::new(DynNode::VoltageSource(0.0, 1.0)),
            Box::new(DynNode::Pot(
                "Tone".to_string(),
                100_000.0,
                0.5,
                PotTaper::B,
            )),
        );
        let wdf = WdfStage::new(
            tree,
            RootKind::ShortCircuit,
            Oversampler::new(OversamplingFactor::X1),
        );
        let mut stage = Stage::Wdf(wdf);

        let target = stage.control_target_for_pot(2, "Tone");
        assert!(matches!(target, Some(ControlTarget::PotInStage(2))));
        assert!(stage.control_target_for_pot(2, "Drive").is_none());

        assert!(stage.set_control_pot("Tone", 0.8));
        assert_eq!(
            stage.as_wdf().unwrap().tree.get_pot_position("Tone"),
            Some(0.8)
        );
        assert!(!stage.set_control_pot("Drive", 0.5));
    }

    #[test]
    fn stage_owned_black_feedback_pot_discovery_and_mutation() {
        let mut bf = BlackFeedbackStage::new_test(100_000.0, 10_000.0, true, 48_000.0);
        bf.pot_comp_id = Some("Drive".to_string());
        bf.pot_max_r = 100_000.0;
        let mut stage = Stage::BlackFeedback(bf);

        let target = stage.control_target_for_pot(3, "Drive");
        assert!(matches!(
            target,
            Some(ControlTarget::PotInBlackFeedbackStage(3))
        ));
        assert!(stage.control_target_for_pot(3, "Tone").is_none());

        assert!(stage.set_control_pot("Drive", 1.0));
        let high_gain = stage.as_black_feedback().unwrap().gain().abs();

        assert!(stage.set_control_pot("Drive", 0.1));
        let low_gain = stage.as_black_feedback().unwrap().gain().abs();

        assert!(high_gain > low_gain * 3.0);
        assert!(!stage.set_control_pot("Tone", 0.5));
    }

    #[test]
    fn stage_owned_iir_pot_discovery_and_mutation() {
        let iir = IirData::new(
            alloc::vec![1.0, 0.0, 0.0],
            alloc::vec![1.0, 0.0, 0.0],
            48_000.0,
        );
        let mut iir_stage = IirStage::new(iir);
        iir_stage.pot_bindings.push(IirPotBinding {
            comp_id: "Gain".to_string(),
            max_r: 100_000.0,
            fixed_series_r: 0.0,
            ri: 10_000.0,
            position: 0.0,
            role: crate::stage::IirPotRole::Generic,
            feedback_r: 0.0,
            non_inverting: false,
        });
        let mut stage = Stage::Iir(iir_stage);

        let target = stage.control_target_for_pot(1, "Gain");
        assert!(matches!(target, Some(ControlTarget::PotInIirStage(1))));
        assert!(stage.control_target_for_pot(1, "Tone").is_none());

        assert!(stage.set_control_pot("Gain", 0.8));
        assert_eq!(stage.as_iir().unwrap().pot_bindings[0].position, 0.8);
        assert!((stage.as_iir().unwrap().iir.b_coeffs[0] + 8.0).abs() < 1e-9);
        assert!(!stage.set_control_pot("Tone", 0.5));
    }

    #[test]
    fn iir_stage_binds_physical_one_port_state_map() {
        let iir = IirData::new(
            alloc::vec![0.1, 0.0, -0.1],
            alloc::vec![1.0, -0.5, 0.25],
            48_000.0,
        );
        let mut iir_stage = IirStage::new(iir);
        let reactive_one_ports = alloc::vec![
            OnePort::new(
                PortTerminals::single_ended(MnaNodeId::new(0)),
                OnePortKind::Capacitor(68e-9),
            ),
            OnePort::new(
                PortTerminals::single_ended(MnaNodeId::new(1)),
                OnePortKind::Capacitor(68e-9),
            ),
        ];

        iir_stage.bind_physical_one_ports(&reactive_one_ports);

        assert_eq!(iir_stage.one_port_states().len(), 2);
        assert_eq!(iir_stage.state_map.physical_state_count, 2);
        assert_eq!(iir_stage.state_map.transformed_state_count, 2);
        assert_eq!(iir_stage.state_map.folded_or_eliminated_count, 0);
        assert!(iir_stage
            .state_map
            .physical_one_ports
            .iter()
            .all(|one_port| matches!(one_port.spec.kind, OnePortKind::Capacitor(_))));
    }

    #[test]
    fn iir_stage_exposes_declarative_route_bindings() {
        let iir = IirData::new(
            alloc::vec![1.0, 0.0, 0.0],
            alloc::vec![1.0, 0.0, 0.0],
            48_000.0,
        );
        let mut iir_stage = IirStage::new(iir);
        iir_stage.bind_ports(Some(10), Some(20));
        let stage = Stage::Iir(iir_stage);

        assert_eq!(
            stage.ins(),
            alloc::vec![PortBinding::new(BindingId::new(10), 0)]
        );
        assert_eq!(
            stage.outs(),
            alloc::vec![PortBinding::new(BindingId::new(20), 1)]
        );
    }

    #[test]
    fn state_space_stage_binds_physical_one_port_state_map() {
        let reactive_one_ports = alloc::vec![
            OnePort::new(
                PortTerminals::single_ended(MnaNodeId::new(0)),
                OnePortKind::Capacitor(100e-9),
            ),
            OnePort::new(
                PortTerminals::single_ended(MnaNodeId::new(1)),
                OnePortKind::Capacitor(47e-9),
            ),
        ];
        let ss = StateSpaceData {
            x: alloc::vec![0.0, 0.0],
            a_matrix: alloc::vec![0.5, 0.0, 0.0, 0.25],
            b_vector: alloc::vec![1.0, 0.0],
            c_vector: alloc::vec![0.0, 1.0],
            n_states: 2,
            reactive_one_ports,
            vs_idx: 0,
            output_pos: Some(1),
            output_neg: None,
            sample_rate: 48_000.0,
            d_feedthrough: 0.0,
            prev_output: 0.0,
            variable_resistors: alloc::vec![],
        };

        let stage = StateSpaceStage::new(ss, 9.0);

        assert_eq!(stage.one_port_states().len(), 2);
        assert_eq!(stage.state_map.physical_state_count, 2);
        assert_eq!(stage.state_map.transformed_state_count, 2);
        assert_eq!(stage.state_map.folded_or_eliminated_count, 0);
        assert!(stage
            .state_map
            .physical_one_ports
            .iter()
            .all(|one_port| matches!(one_port.spec.kind, OnePortKind::Capacitor(_))));
    }

    /// Regression lock for the state-space OUTPUT extraction when the output
    /// node is ALGEBRAIC (resistive / VCVS-driven, no capacitor on it).
    ///
    /// An algebraic output node's voltage is a linear function of BOTH the
    /// input AND the dynamic (capacitor) state vector:
    ///   v_out_alg = (G_aa⁻¹·b_a)·u  +  (−G_aa⁻¹·G_ac)·x
    /// `build_state_space_matrices` must fold the state part into `c_vector`
    /// (NOT collapse it entirely into the direct feedthrough `d`). If the
    /// state coupling were dropped, the algebraic output would be a flat
    /// feedthrough `y = d·u` and the circuit's frequency shaping would vanish.
    ///
    /// Circuit: VS(0) → R_in(10k) → pos(1) ; C(10nF): pos(1) → gnd ;
    /// unity buffer VCVS pos=1, neg=out, out=2. Output node = 2 (ALGEBRAIC).
    /// node1 is a first-order RC low-pass of the input (corner ~1.6 kHz),
    /// buffered to node2. The output MUST roll off with frequency: low-freq
    /// gain ≈ 1, high-freq gain ≪ 1. A non-trivial `c_vector` is the only way
    /// that state dependence reaches the algebraic output.
    #[test]
    fn state_space_algebraic_output_retains_state_coupling() {
        use crate::tree::MnaSystem;

        let fs = 48_000.0;
        let mut mna = MnaSystem::new(3, 2);
        mna.stamp_voltage_source(Some(0), None, 0);
        mna.stamp_resistor(Some(0), Some(1), 10_000.0); // R_in
                                                        // unity buffer: pos=1, neg=out=2 (op-amp follower)
        mna.stamp_vcvs(Some(1), Some(2), Some(2), None, 200_000.0, 75.0, 1);

        // Capacitor on the (algebraic-feeding) pos node -> RC low-pass.
        let reactive_one_ports = alloc::vec![OnePort::new(
            PortTerminals::single_ended(MnaNodeId::new(1)),
            OnePortKind::Capacitor(10e-9),
        )];

        // Output node 2 is the VCVS output: ALGEBRAIC (no cap on it).
        let (a_d, b_d, c_out, n_states, d_ft) =
            mna.build_state_space_matrices(&reactive_one_ports, 0, Some(2), None, fs);

        assert!(n_states >= 1, "expected >=1 dynamic state, got {n_states}");

        // The decisive invariant: the algebraic output's state coupling must
        // survive in c_vector. If the bug were present, c_vector would be all
        // zero and the entire response would live in the constant feedthrough.
        let c_norm: crate::Wave = c_out.iter().map(|v| v.abs()).sum();
        assert!(
            c_norm > 1e-6,
            "algebraic-output c_vector collapsed to ~0 ({c_out:?}); state \
             coupling was dropped into feedthrough d={d_ft}"
        );

        // Behavioural check: measure peak gain at a low and a high frequency.
        let measure = |freq: crate::Wave| -> crate::Wave {
            let n = (fs * 0.5) as usize;
            let mut x = alloc::vec![0.0; n_states];
            let mut work = alloc::vec![0.0; n_states];
            let mut peak = 0.0 as crate::Wave;
            for i in 0..n {
                let u = crate::math::sin(2.0 * crate::math::PI * freq * i as crate::Wave / fs);
                for r in 0..n_states {
                    let mut v = b_d[r] * u;
                    for cc in 0..n_states {
                        v += a_d[r * n_states + cc] * x[cc];
                    }
                    work[r] = v;
                }
                let mut y = d_ft * u;
                for r in 0..n_states {
                    // trapezoidal output (matches bilinear midpoint)
                    y += c_out[r] * (work[r] + x[r]) * 0.5;
                }
                x.copy_from_slice(&work);
                if i > n * 3 / 4 {
                    peak = peak.max(y.abs());
                }
            }
            peak
        };

        let g_low = measure(100.0);
        let g_high = measure(15_000.0);
        // Low-freq passes (~unity), high-freq is rolled off by the RC.
        assert!(
            g_low > 0.7,
            "low-frequency gain {g_low:.4} should be near unity"
        );
        assert!(
            g_high < 0.5 * g_low,
            "high-frequency gain {g_high:.4} should roll off below low \
             gain {g_low:.4}; a flat feedthrough would NOT roll off"
        );
    }

    #[test]
    fn state_space_stage_exposes_declarative_route_bindings() {
        let ss = StateSpaceData {
            x: alloc::vec![0.0],
            a_matrix: alloc::vec![0.5],
            b_vector: alloc::vec![1.0],
            c_vector: alloc::vec![1.0],
            n_states: 1,
            reactive_one_ports: alloc::vec![],
            vs_idx: 0,
            output_pos: Some(0),
            output_neg: None,
            sample_rate: 48_000.0,
            d_feedthrough: 0.0,
            prev_output: 0.0,
            variable_resistors: alloc::vec![],
        };
        let mut ss_stage = StateSpaceStage::new(ss, 9.0);
        ss_stage.bind_ports(Some(11), Some(21));
        let stage = Stage::StateSpace(ss_stage);

        assert_eq!(
            stage.ins(),
            alloc::vec![PortBinding::new(BindingId::new(11), 0)]
        );
        assert_eq!(
            stage.outs(),
            alloc::vec![PortBinding::new(BindingId::new(21), 1)]
        );
    }

    #[test]
    fn lti_state_map_reports_folded_or_eliminated_physical_states() {
        let iir = IirData::new(alloc::vec![0.1, 0.0], alloc::vec![1.0, -0.5], 48_000.0);
        let mut iir_stage = IirStage::new(iir);
        let reactive_one_ports = alloc::vec![
            OnePort::new(
                PortTerminals::single_ended(MnaNodeId::new(0)),
                OnePortKind::Capacitor(68e-9),
            ),
            OnePort::new(
                PortTerminals::single_ended(MnaNodeId::new(1)),
                OnePortKind::Capacitor(68e-9),
            ),
        ];

        iir_stage.bind_physical_one_ports(&reactive_one_ports);

        assert_eq!(iir_stage.state_map.physical_state_count, 2);
        assert_eq!(iir_stage.state_map.transformed_state_count, 1);
        assert_eq!(iir_stage.state_map.folded_or_eliminated_count, 1);
    }
}

#[cfg(feature = "debug-trace")]
use core::sync::atomic::{AtomicU64, Ordering};

/// Count of traced pipeline samples (non-zero only).
#[cfg(feature = "debug-trace")]
static TRACE_COUNT_PIPELINE: AtomicU64 = AtomicU64::new(0);

/// Max pipeline samples to trace (non-zero only).
#[cfg(feature = "debug-trace")]
const MAX_TRACE_PIPELINE: u64 = 5;
#[cfg(feature = "debug-trace")]
static LATE_TRACE_COUNT: AtomicU64 = AtomicU64::new(0);

// ═══════════════════════════════════════════════════════════════════════════
// Compiled pedal
// ═══════════════════════════════════════════════════════════════════════════

/// Pre-computed mirror pot IDs (avoids format!() allocation on RT thread).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MirrorPot {
    pub id: String,
    pub id_aw: String,
    pub id_wb: String,
}

/// Named voltage port binding: maps a port name to a circuit node for
/// audio-rate signal injection/extraction. No impedance recompute.
///
/// Input ports inject voltage at a circuit node via `node_signals`.
/// Output ports extract voltage from a circuit node after processing.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct NamedPortBinding {
    /// Port name (e.g., "audio_in", "cv_cutoff", "env_out").
    pub name: alloc::string::String,
    /// Input or Output.
    pub direction: crate::PortDirection,
    /// Dense index into `port_values` array.
    pub index: usize,
    /// Circuit graph node ID for injection/extraction.
    pub node_id: usize,
    /// Default value when no external signal is connected (volts).
    pub default_value: crate::Wave,
    /// Stage index that contains this port's VS leaf.
    /// Set at compile time so runtime skips stages that don't have this port.
    /// usize::MAX = not yet resolved (inject into all stages as fallback).
    pub stage_idx: usize,
}

/// Compiler-synthesized INTERNAL delayed port (Phase 2b) — NOT user-overridable.
///
/// Carries a solved boundary-node value across a one-sample (z⁻¹) gap, so a
/// de-fused sub-network (e.g. the LA-2A detector) can read its taps one sample
/// late and solve as its own sub-network with NO intra-sample ordering
/// dependency. This is the GENERAL realization of the `cv_delayed` cross-sample
/// precedent (`SidechainProcessor`): the value at `source_node_id` is WRITTEN to
/// `prev_value` at end-of-sample, and INJECTED at `consumer_node_id` (via
/// `node_signals`) at the START of the next sample.
///
/// * `gain` scales the injected value (a superposition transfer weight from the
///   detector resistive mix; `1.0` for a pass-through carry).
/// * `consumer_node_id == usize::MAX` means "carry only" — the solved value is
///   stored in `prev_value` for a later phase to consume (Phase 3 reads the
///   detector `EL_drive` carry to drive the photocoupler LED), but nothing is
///   injected this phase.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct InternalPortBinding {
    /// Circuit graph node whose SOLVED value is captured at end-of-sample.
    pub source_node_id: usize,
    /// Circuit graph node where `prev_value * gain` is injected at the start of
    /// the NEXT sample. `usize::MAX` = carry-only (store, do not inject).
    pub consumer_node_id: usize,
    /// Superposition / carry gain applied to the injected value.
    pub gain: crate::Wave,
    /// The z⁻¹ register: last sample's solved value at `source_node_id`.
    pub prev_value: crate::Wave,
}

/// Compiler-synthesized DETECTOR → photocoupler-LED coupling (Phase 3) — the
/// actual gain-reduction loop closure of a cross-network feedback detector
/// (the LA-2A `EL_drive.b -> PC1.led` light path).
///
/// This is the optical, non-electrical half of the de-fused detector: it reads
/// the detector's solved EL-drive value (carried one sample late by the
/// `internal_ports[carry_idx]` z⁻¹ register, Phase 2b), rectifies + normalizes
/// it to a 0..1 LED drive, and pushes it into the T4B photocoupler's two-rate
/// cell (`set_led_drive`), which darkens the LDR shunt leaf in the FORWARD
/// gain-reduction divider — louder program -> brighter EL panel -> lower cell
/// resistance -> more attenuation. The one-sample (z⁻¹) delay (reusing the 2b
/// carry) breaks the otherwise-instantaneous feedback loop, exactly the
/// `cv_delayed` precedent.
///
/// NARROW: only synthesized for a true delayed feedback detector (LA-2A opto
/// leveler). Non-detector circuits leave this `None`, so the
/// envelope-follower → LED modulation path (`ModulationTarget::PhotocouplerLed`,
/// used by `opto_leveler.pedal`) is entirely untouched.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DetectorLedCoupling {
    /// Index into `internal_ports` of the EL-drive carry (its `prev_value` is
    /// last sample's solved detector output — the z⁻¹ closure).
    pub carry_idx: usize,
    /// The photocoupler (LDR) component id to drive (e.g. `PC1`).
    pub comp_id: String,
    /// Stage index that owns the photocoupler LDR leaf in the forward divider.
    pub stage_idx: usize,
    /// Normalization gain: `led = (|el_drive| * scale).clamp(0, 1)`. Chosen so a
    /// loud-program EL-drive maps near full illumination.
    pub scale: crate::Wave,
}

pub use crate::routing::{
    BindingId, BkmVsRouteBinding, GraphRoutedBkm, PortBinding, Route, StageRouteDebug,
    StageRoutePlan,
};

/// Which physical half of a 3-terminal pot a stage binding drives.
///
/// A 3-terminal pot is split at compile time into two leaves, `{id}__aw`
/// (a→wiper) and `{id}__wb` (wiper→b). Each owning stage binding records
/// which half it re-stamps so dispatch can route the correctly-formatted
/// leaf id and the (optionally complemented) position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PotHalf {
    /// The pot is not split in this stage — re-stamp the base component id.
    Whole,
    /// The a→wiper half (`{id}__aw`).
    Aw,
    /// The wiper→b half (`{id}__wb`).
    Wb,
}

/// One coordinated stage update for a control.
///
/// A single logical control (one host parameter) can own several stage
/// bindings when its pot STRADDLES a stage boundary — e.g. an LA-2A makeup
/// pot whose two halves land in a WDF gain-reduction stage and a MultiNL
/// tube-grid stage, with a wiper-divider tap at the boundary. Each binding
/// re-stamps one stage (or applies one wiper divider) with the correct half
/// and complement; `CompiledPedal::set_control` fans a single position out to
/// every binding in the set.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StageBinding {
    /// Which stage / element this binding updates, and how to reach it.
    pub target: ControlTarget,
    /// Which physical half of a split 3-terminal pot this binding drives.
    pub half: PotHalf,
    /// Invert the position (`1.0 - value`) for this binding. Used for the
    /// ground-touching half of a split pot so the two halves sum to `max_R`.
    pub complement: bool,
}

/// Control binding: maps a knob label to a parameter in the processing chain.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ControlBinding {
    pub label: String,
    /// Primary / representative target (kept for back-compat with the many
    /// readers of `.target`; for a multi-target pot it is the first stage).
    pub target: ControlTarget,
    /// Coordinated stage-update set. ONE logical control = ONE host
    /// parameter, but N coordinated stage updates. Empty for legacy /
    /// non-pot bindings, in which case dispatch falls back to `target` +
    /// the comp-id broadcast. When populated, dispatch iterates this set so
    /// a pot straddling a stage boundary updates EVERY owning stage (and any
    /// wiper-divider at the boundary).
    #[cfg_attr(feature = "serde", serde(default))]
    pub targets: Vec<StageBinding>,
    pub component_id: String,
    #[allow(dead_code)]
    pub max_resistance: crate::Wave,
    /// Pot taper curve — applied once before splitting into aw/wb halves
    /// so that aw + wb = max_R always (split halves use Linear taper internally).
    pub taper: crate::pot_taper::PotTaper,
    /// Range mapping: (min, max).  Input 0→min, 1→max.
    /// Default (0.0, 1.0) is identity; (1.0, 0.0) inverts.
    pub range: (crate::Wave, crate::Wave),
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ControlTarget {
    /// Modify a pot in a specific WDF stage.
    PotInStage(usize),
    /// Modify a pot in an IIR stage (recomputes dc_gain / biquad coefficients).
    PotInIirStage(usize),
    /// Modify a pot inside a multi-NL stage (R-type adaptor approach).
    /// (multi_nl_stage_idx, passive_child_idx)
    PotInMultiNlStage(usize, usize),
    /// Apply an inter-stage wiper-divider ratio after the given stage index.
    ///
    /// This is the cross-stage pot path: a 3-terminal pot whose wiper feeds a
    /// downstream high-impedance load (e.g. a tube grid / BJT base in a LATER
    /// stage) divides the signal by its position at the boundary. Used as one
    /// of a multi-target [`StageBinding`] set alongside the per-stage leaf
    /// re-stamps so the divided wiper voltage actually reaches the load. The
    /// matching [`WiperDivider`] entry (keyed by component id) carries the
    /// live position; this target marks that a divider exists for the control.
    WiperDivider { after_stage_idx: usize },
    /// Modify the feedback pot in a BlackFeedback stage (recomputes Rf → gain).
    PotInBlackFeedbackStage(usize),
    /// Forward a control change to a sidechain sub-circuit.
    /// The sidechain's own CompiledPedal handles the pot lookup and
    /// scattering matrix recomputation internally.
    SidechainControl(usize),
    /// Forward a control change to a subcircuit's inner processor.
    SubcircuitControl { subcircuit_idx: usize },
    /// Modify an LFO's rate (index into lfos vector).
    LfoRate(usize),
    /// Modify an LFO's depth/amplitude (index into lfos vector).
    LfoDepth(usize),
    /// Modify a delay line's delay time (normalized 0–1).
    DelayTime(usize),
    /// Modify a delay line's feedback amount (0–1).
    DelayFeedback(usize),
    /// Modify a BBD delay line's clock rate (normalized 0–1 → delay time).
    BbdClockRate(usize),
    /// Modify a BBD delay line's feedback amount (0–1).
    BbdFeedback(usize),
    /// Modify a spring reverb's dwell/drive input trim (normalized 0–1).
    SpringDwell(usize),
    /// Modify a spring reverb's decay → RT60 authority (normalized 0–1).
    SpringDecay(usize),
    /// Modify a spring reverb's damping LPF cutoff (normalized 0–1).
    SpringDamping(usize),
    /// Modify a spring reverb's wet/dry mix (normalized 0–1).
    SpringMix(usize),
    /// Modify a switch position for fork() routing.
    /// Contains: (switch_id, num_positions)
    SwitchPosition {
        switch_id: String,
        num_positions: usize,
    },
    /// Fire a trigger impulse (index into triggers vec).
    Trigger(usize),
}

/// Inter-stage wiper divider for output Level/Volume pots.
///
/// Applied between stages in the serial audio chain. The WDF tree handles
/// the impedance of each pot half; this routing layer applies the voltage
/// division: `signal *= position` (after taper).
///
/// This is the correct architecture for pot wipers that span stage boundaries:
/// the pot halves may be in different WDF tree branches, but the wiper voltage
/// is a simple ratio computed from the pot position.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WiperDivider {
    /// Apply the divider AFTER processing this stage index.
    pub after_stage_idx: usize,
    /// Component ID of the pot (for smoother matching).
    pub pot_comp_id: String,
    /// Current position (0.0–1.0, after taper). Updated by smoother.
    pub position: crate::Wave,
    /// Pot taper for mapping raw position to effective position.
    pub taper: crate::pot_taper::PotTaper,
}

/// Runtime state for a single-sample impulse source (drum trigger).
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TriggerState {
    pub amplitude: crate::Wave,
    pub countdown: u32,
    /// True on the sample when this trigger fired (before tick() clears countdown).
    /// Used by VCA envelope gating which runs after tick().
    pub active_this_sample: bool,
    /// Index of the WDF stage this trigger injects into (None = global input).
    pub target_stage: Option<usize>,
    /// Circuit graph node ID where this trigger injects its impulse.
    /// `usize::MAX` means use global input (backward compat for single-trigger circuits).
    pub injection_node: usize,
}

impl TriggerState {
    pub fn new(amplitude: crate::Wave) -> Self {
        Self {
            amplitude,
            countdown: 0,
            active_this_sample: false,
            target_stage: None,
            injection_node: usize::MAX,
        }
    }
    pub fn fire(&mut self) {
        self.countdown = 1;
    }
    /// Returns the impulse value for this sample (amplitude or 0.0).
    /// Also sets `active_this_sample` so downstream consumers (VCA envelopes)
    /// can detect the trigger even after countdown is decremented.
    pub fn tick(&mut self) -> crate::Wave {
        if self.countdown > 0 {
            self.countdown -= 1;
            self.active_this_sample = true;
            self.amplitude
        } else {
            self.active_this_sample = false;
            0.0
        }
    }
}

// PotEffect enum removed — all pot effects are now handled by component-driven
// updates: OpAmpRoot::set_feedback_pot_r(), MultiNlStage::update_bias_from_pot(),
// and bbd_mix_pot_id on CompiledPedal.

/// Modulation target for LFOs and envelope followers.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ModulationTarget {
    /// Modulate a JFET's Vgs.
    JfetVgs { stage_idx: usize },
    /// Modulate ALL JFET stages' Vgs together (for phasers).
    AllJfetVgs,
    /// Modulate a JFET variable-resistor *leaf* inside a stage's WDF tree.
    /// Modulated JFETs compile to `jfet_vr` leaves (EdgeKind::Linear), not
    /// stage roots, so root-targeting `set_jfet_vgs()` cannot reach them.
    JfetVrVgs { stage_idx: usize, comp_id: String },
    /// Modulate a Photocoupler's LED drive.
    PhotocouplerLed { stage_idx: usize, comp_id: String },
    /// Modulate a Triode's Vgk (grid-cathode bias).
    TriodeVgk { stage_idx: usize },
    /// Modulate a Pentode's Vg1k (control grid bias).
    PentodeVg1k { stage_idx: usize },
    /// Modulate a Variable-mu triode's Vgk (grid-cathode bias).
    VariMuVgk { stage_idx: usize },
    /// Modulate a MOSFET's Vgs.
    MosfetVgs { stage_idx: usize },
    /// Modulate an OTA's bias current (for VCA/compressor) — WDF tree root.
    OtaIabc { stage_idx: usize },
    /// Modulate a linearized OTA's gm in the R-type MNA (for VCA/compressor).
    /// The OTA is stamped as a VCCS in the MNA; gm changes trigger S-matrix recompute.
    OtaIabcLinear { multi_nl_idx: usize },
    /// Modulate a BBD's clock frequency (for chorus/flanger).
    BbdClock { bbd_idx: usize },
    /// Modulate a VCA's control-voltage port (compressors/tremolo).
    /// Drives `Vca::set_cv_normalized` on `vcas[vca_idx]` (declaration-order
    /// index contract from the compiler's `vca_lowering` pass): 0 = unity
    /// gain, 1 = full attenuation (SSM2164-class dB-linear law).
    VcaCv { vca_idx: usize },
    /// Modulate an op-amp's non-inverting input voltage.
    #[allow(dead_code)]
    OpAmpVp { opamp_idx: usize },
    /// Modulate a delay line's speed (for wow/flutter).
    /// The modulation value is added to the speed factor (1.0 = no change).
    DelaySpeed { delay_idx: usize },
    /// Modulate a delay line's delay time (normalized 0–1).
    DelayTime { delay_idx: usize },
    /// Modulate a spring reverb's dwell/drive input trim (CV → normalized 0–1).
    SpringDwell { spring_idx: usize },
}

/// Delay line binding in a compiled pedal.
///
/// Holds the generic delay line and metadata about its taps.
/// The delay line is processed separately from the WDF tree:
/// write once per sample, then read from each tap.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DelayLineBinding {
    /// The ring-buffer delay line.
    pub delay_line: crate::elements::DelayLine,
    /// Tap ratios for multi-tap reading (e.g., [1.0, 2.0, 4.0] for RE-201).
    /// Each tap reads from the buffer at `base_delay * ratio`.
    pub taps: Vec<crate::Wave>,
    /// Wet/dry mix (0.0 = fully dry, 1.0 = fully wet). Per-instance (spec §6 —
    /// no global mix). Defaults to 0.5 (the historical hard-coded 50/50) when
    /// no mix pot is wired, so unbound delays behave as before.
    pub wet_mix: crate::Wave,
    /// Component ID for debugging and control binding.
    #[allow(dead_code)]
    pub comp_id: String,
}

/// Spring reverb binding in a compiled pedal.
///
/// Holds the dispersive [`crate::elements::SpringTank`] and its per-instance
/// wet/dry mix (spec §6 — no global mix field; learns from the BBD's shared
/// `bbd_wet_mix` being the wrong pattern). Bridged across the behavioral gap
/// in the processor tick loop like the BBD/delay-line inserts.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SpringBinding {
    /// The spring-reverb tank (dispersive allpass network).
    pub tank: crate::elements::SpringTank,
    /// Wet/dry mix (0.0 = fully dry, 1.0 = fully wet). Per-instance — default
    /// 0.33 (a tasteful spring blend) when no mix pot is wired.
    pub wet_mix: crate::Wave,
    /// Component ID for debugging and control binding.
    #[allow(dead_code)]
    pub comp_id: String,
}

/// VCO runtime binding — generates audio-rate waveforms at a circuit node.
///
/// A VCO is a **generator** [`crate::processor::Stage`]-free DSP block (spec
/// §4, N=0 audio inputs): it owns no WDF/MNA scattering and injects its output
/// into `node_signals` each sample, where downstream stages read it. Pitch is
/// an **audio-rate CV input port** (spec §3): the block reads the voltage at
/// `cv_node_id` from `node_signals` and maps it to frequency at 1 V/oct via
/// `Vco::set_cv_pitch` — a float update to `phase_inc`, NO scattering recompute.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VcoBinding {
    pub vco: crate::elements::Vco,
    /// Which waveform output to use (saw/tri/pulse).
    pub waveform: crate::elements::VcoWaveform,
    /// Circuit node where VCO output is injected.
    pub output_node_id: usize,
    /// Circuit node whose voltage drives 1 V/oct pitch CV. `usize::MAX` when no
    /// CV pin is wired — the VCO then runs at its declared base frequency.
    pub cv_node_id: usize,
    /// Component ID for debugging.
    #[allow(dead_code)]
    pub comp_id: String,
}

/// VCA runtime binding — amplitude modulation controlled by trigger envelope.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VcaBinding {
    pub vca: crate::elements::Vca,
    /// ADSR envelope that gates the VCA.
    pub envelope: crate::elements::AdsrEnvelope,
    /// Index into triggers vec (which trigger gates this VCA).
    pub trigger_idx: Option<usize>,
    /// Circuit node to read audio input from.
    pub input_node_id: usize,
    /// Circuit node to write amplitude-modulated output to.
    pub output_node_id: usize,
    /// Component ID for debugging.
    #[allow(dead_code)]
    pub comp_id: String,
}

/// Op-amp stage for unity-gain buffers and gain stages.
///
/// Models the op-amp as a voltage-controlled voltage source (VCVS).
/// The output voltage follows: Vout = Aol * (Vp - Vm)
/// For unity-gain buffers (neg tied to out), Vout ≈ Vp.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OpAmpStage {
    /// The op-amp root element with Newton-Raphson solver.
    pub opamp: OpAmpRoot,
    /// Component ID for debugging.
    #[allow(dead_code)]
    pub comp_id: String,
}

/// LFO binding in a compiled pedal.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LfoBinding {
    pub lfo: crate::elements::Lfo,
    pub target: ModulationTarget,
    /// Bias offset for the modulation (e.g., Vgs center point).
    pub bias: crate::Wave,
    /// Modulation range (amplitude).
    pub range: crate::Wave,
    /// Base frequency from RC timing: f = 1/(2πRC).
    pub base_freq: crate::Wave,
    /// LFO component ID (for debugging and future control binding).
    #[allow(dead_code)]
    pub lfo_id: String,
}

/// Signal source feeding an envelope follower's detector input.
///
/// Resolved at compile time from the follower's `EF.in -> <node>` net
/// (see `resolve_envelope_tap` in the compiler's bind pass). Historically
/// every envelope follower read the global pedal input regardless of its
/// netlist tap (audit gap G1); `StageOutput` makes feed-forward taps on
/// interior nodes and feedback taps on the circuit output real.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum EnvelopeTapSource {
    /// Detector reads the raw pedal input (ticked before any stage runs).
    #[default]
    GlobalInput,
    /// Detector reads the serial-chain output of `stages[idx]`, ticked
    /// inside the stage loop right after that stage produces its sample.
    /// Feed-forward taps (tap stage before the modulated gain stage) thus
    /// modulate the CURRENT sample; feedback taps (tap stage at/after the
    /// gain stage) take effect on the NEXT sample — an inherent one-sample
    /// delay, same convention as `SidechainProcessor::cv_delayed`.
    StageOutput(usize),
}

/// Envelope follower binding in a compiled pedal.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct EnvelopeBinding {
    pub envelope: crate::elements::EnvelopeFollower,
    pub target: ModulationTarget,
    /// Bias offset for the modulation.
    pub bias: crate::Wave,
    /// Modulation range (amplitude).
    pub range: crate::Wave,
    /// Envelope follower component ID.
    #[allow(dead_code)]
    pub env_id: String,
    /// Where the detector taps its signal (compile-time resolved).
    #[cfg_attr(feature = "serde", serde(default))]
    pub tap: EnvelopeTapSource,
}

/// Smoothed parameter for zipper-free pot control.
///
/// Uses a one-pole low-pass filter to smoothly interpolate from current
/// to target value. This eliminates clicks/pops when knobs are turned.
///
/// Smoothing time is ~10ms (coefficient ~0.9995 at 48kHz).
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SmoothedParam {
    /// Target value (set immediately by user input)
    pub target: crate::Wave,
    /// Current smoothed value (approaches target over time)
    pub current: crate::Wave,
    /// Smoothing coefficient (0.9995 = ~10ms at 48kHz)
    pub coef: crate::Wave,
    /// Control index in the controls vec (for updating the actual control)
    pub control_idx: usize,
}

impl SmoothedParam {
    /// Create a new smoothed parameter.
    pub fn new(initial: crate::Wave, control_idx: usize, sample_rate: crate::Wave) -> Self {
        // Smoothing time constant ~10ms = 0.010 seconds
        // coef = exp(-1 / (tau * fs)) ≈ exp(-1 / (0.010 * 48000)) ≈ 0.9979
        let tau = 0.010; // 10ms smoothing
        let coef = crate::math::exp(-1.0 / (tau * sample_rate));
        Self {
            target: initial,
            current: initial,
            coef,
            control_idx,
        }
    }

    /// Advance the smoother by one sample. Returns true if value changed.
    #[inline]
    pub fn advance(&mut self) -> bool {
        self.current = self.coef * self.current + (1.0 - self.coef) * self.target;
        // Snap to target when asymptotically close (avoids infinite tail).
        if (self.current - self.target).abs() < 1e-6 {
            self.current = self.target;
        }
        // Always return true — caller checks is_settled() before calling us,
        // so if we're here the pot needs updating.
        true
    }

    /// Set the target value (called from set_control).
    pub fn set_target(&mut self, value: crate::Wave) {
        self.target = value;
    }

    /// Check if we're close enough to target to stop smoothing.
    pub fn is_settled(&self) -> bool {
        (self.current - self.target).abs() < 0.0001
    }

    /// Update smoothing coefficient for new sample rate.
    pub fn set_sample_rate(&mut self, sample_rate: crate::Wave) {
        let tau = 0.010;
        self.coef = crate::math::exp(-1.0 / (tau * sample_rate));
    }
}

/// Device-aware rail saturation model.
///
/// Real rail saturation depends on the active device's output stage topology.
/// Instead of a generic `tanh` waveshaper, this models the saturation shape
/// based on the dominant active device type in the circuit.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum RailSaturation {
    /// No active devices — passive clipping only (no rail saturation).
    None,
    /// Op-amp output stage: symmetric saturation with slight crossover
    /// distortion near zero. Output stage is push-pull emitter follower
    /// that saturates symmetrically against both rails.
    /// `output_swing_ratio` is Vout_max / Vsupply (typically 0.85–0.95).
    OpAmp { output_swing_ratio: crate::Wave },
    /// BJT common-emitter: asymmetric saturation.  The collector saturates
    /// hard (~0.2V from rail) when driven into saturation, but cuts off
    /// softly when the base drive is removed.  NPN saturates at positive
    /// rail (hard), cuts off toward negative rail (soft).
    Bjt { vce_sat: crate::Wave },
    /// JFET/MOSFET: soft saturation onset due to square-law I-V curve.
    /// The drain current pinches off gradually as Vds approaches Vgs-Vth.
    Fet,
    /// Triode/pentode: grid conduction (hard clip) at positive swing,
    /// plate current cutoff (soft) at negative swing.
    /// `mu` is the amplification factor (affects saturation sharpness).
    Tube { mu: crate::Wave },
}

impl RailSaturation {
    /// Apply rail saturation to a signal given the supply headroom.
    ///
    /// `headroom` is `supply_voltage / 9.0` (normalized to 9V reference).
    #[inline]
    fn process(&self, signal: crate::Wave, headroom: crate::Wave) -> crate::Wave {
        match self {
            RailSaturation::None => signal,
            RailSaturation::OpAmp { output_swing_ratio } => {
                // Op-amp output stage: symmetric clipping at ±(swing_ratio * headroom).
                // Uses a quintic polynomial for smoother knee than tanh, matching
                // the push-pull emitter-follower output stage characteristic.
                let ceiling = output_swing_ratio * headroom;
                let x = signal / ceiling;
                let ax = x.abs();
                if ax <= 0.75 {
                    // Linear region: no saturation
                    signal
                } else if ax >= 1.5 {
                    // Hard saturation
                    ceiling * x.signum()
                } else {
                    // Knee: quintic ease from 0.75 to ceiling
                    // Normalized to [0, 1] within knee region
                    let t = (ax - 0.75) / 0.75; // 0..1
                    let t2 = t * t;
                    let t3 = t2 * t;
                    // Hermite-like: starts at 0.75, ends at 1.0, smooth at both ends
                    let compressed = 0.75 + 0.25 * (3.0 * t2 - 2.0 * t3);
                    (ceiling * compressed).copysign(signal)
                }
            }
            RailSaturation::Bjt { vce_sat } => {
                // BJT asymmetric saturation.
                // Positive rail (collector saturation): hard knee at Vcc - Vce_sat
                // Negative rail (cutoff): soft exponential tail
                let pos_ceiling = headroom * (1.0 - vce_sat / (9.0 * headroom));
                let neg_ceiling = headroom;
                if signal > 0.0 {
                    // Hard saturation toward positive rail (collector saturation)
                    let x = signal / pos_ceiling;
                    if x <= 0.7 {
                        signal
                    } else if x >= 1.5 {
                        pos_ceiling
                    } else {
                        let t = (x - 0.7) / 0.8;
                        let t2 = t * t;
                        pos_ceiling * (0.7 + 0.3 * (2.0 * t - t2))
                    }
                } else {
                    // Soft cutoff toward negative rail (transistor cutoff)
                    let x = -signal / neg_ceiling;
                    if x <= 0.8 {
                        signal
                    } else {
                        let over = x - 0.8;
                        // Exponential compression: gradual cutoff
                        -(neg_ceiling * (0.8 + 0.2 * (1.0 - crate::math::exp(-over * 5.0))))
                    }
                }
            }
            RailSaturation::Fet => {
                // JFET/MOSFET: soft saturation from square-law characteristic.
                // The onset is gentler than BJT — current gradually pinches off.
                let ceiling = headroom;
                let x = signal / ceiling;
                let ax = x.abs();
                if ax <= 0.85 {
                    signal
                } else {
                    // Square-law-like soft compression
                    let over = ax - 0.85;
                    let compressed = 0.85 + 0.15 * (1.0 - 1.0 / (1.0 + over * 3.0));
                    (ceiling * compressed).copysign(signal)
                }
            }
            RailSaturation::Tube { mu } => {
                // Triode: asymmetric saturation.
                // Positive swing → grid conduction (hard clipping, like a diode)
                // Negative swing → plate current cutoff (soft, gradual)
                // Higher mu = sharper saturation knee
                let ceiling = headroom;
                let sharpness = (mu / 100.0).clamp(0.5, 3.0);
                if signal > 0.0 {
                    // Grid conduction — hard clip (grid draws current)
                    let x = signal / ceiling;
                    if x <= 0.6 {
                        signal
                    } else if x >= 2.0 {
                        ceiling
                    } else {
                        let t = (x - 0.6) / 1.4;
                        let curve = 1.0 - crate::math::exp(-t * 2.5 * sharpness);
                        ceiling * (0.6 + 0.4 * curve)
                    }
                } else {
                    // Plate cutoff — soft, gradual
                    let x = -signal / ceiling;
                    if x <= 0.9 {
                        signal
                    } else {
                        let over = x - 0.9;
                        let compressed = 0.9 + 0.1 * over / (0.1 + over);
                        -(ceiling * compressed)
                    }
                }
            }
        }
    }
}

/// A pedal processor compiled from a `.pedal` file's netlist.
///
/// Each `.pedal` file produces a unique processor with its own WDF tree topology,
/// component values, and diode models — no hardcoded processor selection.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CompiledPedal {
    /// All processing stages in signal-flow order. One vec, no index indirection.
    pub stages: Vec<Stage>,
    /// Runtime execution plan derived from stage-owned boundary bindings.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub stage_route_plan: StageRoutePlan,
    /// Push-pull differential stages (e.g., Fairchild 670 gain cell).
    /// These are processed after regular stages.
    pub push_pull_stages: Vec<PushPullStage>,
    pub pre_gain: crate::Wave,
    /// Auto-calibrated output gain scalar (1.0 = no calibration).
    pub output_gain: crate::Wave,
    pub rail_saturation: RailSaturation,
    /// Oversampler for rail saturation to anti-alias harmonics generated
    /// by the nonlinear clipping at supply rails.
    pub rail_sat_oversampler: crate::oversampling::Oversampler,
    pub sample_rate: crate::Wave,
    pub controls: Vec<ControlBinding>,
    pub gain_range: (crate::Wave, crate::Wave),
    /// Supply voltage in volts (default 9.0).
    pub supply_voltage: crate::Wave,
    /// LFO modulators.
    pub lfos: Vec<LfoBinding>,
    /// Envelope follower modulators.
    pub envelopes: Vec<EnvelopeBinding>,
    /// BBD delay lines for delay/chorus/flanger effects.
    pub bbds: Vec<BbdDelayLine>,
    /// Generic delay lines (tape echo, digital delay, Karplus-Strong, etc.).
    pub delay_lines: Vec<DelayLineBinding>,
    /// Spring reverb tanks (dispersive electromechanical reverb islands).
    pub springs: Vec<SpringBinding>,
    /// VCO audio-rate oscillator bindings (inject audio at circuit nodes).
    pub vcos: Vec<VcoBinding>,
    /// VCA amplitude modulation bindings (scale audio by envelope).
    pub vcas: Vec<VcaBinding>,
    /// Thermal model for temperature-dependent behavior.
    /// When present, modulates diode Is and BJT gain over time.
    pub thermal: Option<ThermalModel>,
    /// Tolerance engine seed (stored for diagnostics).
    pub tolerance_seed: u64,
    /// Oversampling factor used for this pedal's nonlinear stages.
    pub oversampling: OversamplingFactor,
    /// Op-amp stages for unity-gain buffers and gain stages.
    /// Each op-amp is modeled as a VCVS with the OpAmpRoot element.
    pub opamp_stages: Vec<OpAmpStage>,
    /// Power supply sag model.
    /// When present, computes instantaneous B+ droop based on signal current draw
    /// and feeds the sagged voltage into `set_supply_voltage()` each sample.
    pub power_supply: Option<PowerSupply>,
    // TODO: DebugStats lives in pedalkernel crate, not rt. Re-enable when DebugStats is ported.
    // #[cfg(debug_assertions)]
    // pub debug_stats: Option<alloc::sync::Arc<crate::debug::DebugStats>>,
    /// Metrics accumulator for real-time UI visualization.
    /// Accumulates per-sample data and reduces to `UiMetrics` every block.
    pub metrics_accumulator: Option<MetricsAccumulator>,
    /// Ring buffer for sending metrics to the UI thread (shared via Arc).
    /// Skipped for serde: AtomicUsize inside MetricsRingBuffer is not serializable,
    /// and the ring buffer is a runtime-only construct (not needed on M7 firmware).
    #[cfg_attr(feature = "serde", serde(skip))]
    pub metrics_buffer: Option<Arc<MetricsRingBuffer>>,
    /// Live runtime diagnostics ring (IPC Phase B, `diag` feature).
    ///
    /// When attached via [`CompiledPedal::attach_diag_ring`], the processor
    /// builds one [`crate::diag_ring::RuntimeFrame`] per metering block — NR
    /// convergence aggregate, per-BJT operating points (`Vbe`/`Vce`/`Ic`/`Ib`/
    /// `gm`), and per-stage levels — and pushes it lock-free into a ring laid
    /// over the mmap'd IPC file. Off by default; only built when attached.
    #[cfg(feature = "diag")]
    #[cfg_attr(feature = "serde", serde(skip))]
    pub diag_ring: Option<crate::diag_ring::RuntimeRing>,
    /// Input loading model — models source impedance interaction at circuit input.
    /// Applies DC attenuation and frequency-dependent rolloff based on the
    /// source (e.g., guitar pickup, upstream pedal) driving this circuit.
    pub input_loading: Option<InterstageLoading>,
    /// Output loading model — models load impedance interaction at circuit output.
    /// Applies DC attenuation and frequency-dependent rolloff based on what
    /// this circuit is driving (e.g., downstream pedal, amp input).
    pub output_loading: Option<InterstageLoading>,
    /// Output DC blocker — first-order HPF at ~5 Hz to remove subsonic drift
    /// from integrator DC tails and opamp offset accumulation.
    /// Format: (a1, b0, y_prev, x_prev) for IIR highpass.
    pub output_dc_block: Option<(crate::Wave, crate::Wave, crate::Wave, crate::Wave)>,
    /// Sidechain processors for feedback compression loops.
    /// Each sidechain taps audio, extracts an envelope, and modulates
    /// the push-pull grid bias. Multiple sidechains are supported
    /// (e.g., one per channel in a stereo compressor like the 670).
    pub sidechains: Vec<SidechainProcessor>,
    /// Subcircuit processors (for equipment with subcircuit blocks).
    /// When non-empty, `process()` delegates to `process_subcircuits()` and
    /// the normal WDF stage pipeline is bypassed.
    pub subcircuit_processors: Vec<SubcircuitProcessor>,
    /// Routing steps defining signal flow between subcircuits (topological order).
    pub subcircuit_routing: Vec<crate::subcircuit::RoutingStep>,
    /// Index of the subcircuit whose output is the equipment output.
    pub subcircuit_output_idx: Option<usize>,
    /// Per-subcircuit output buffer (indexed by subcircuit_idx).
    pub subcircuit_outputs: Vec<crate::Wave>,
    /// Smoothed parameters for zipper-free pot control.
    /// One per pot in the circuit that needs smoothing.
    pub pot_smoothers: Vec<SmoothedParam>,
    /// Inter-stage wiper dividers for output Level/Volume pots.
    /// Applied in the serial stage chain after the specified stage index.
    pub wiper_dividers: Vec<WiperDivider>,
    /// Mirrored pot mappings: source_comp_id → list of mirrored pots.
    /// When a source pot is updated, each mirror gets position = 1.0 - source.
    pub pot_mirrors: HashMap<String, Vec<MirrorPot>>,
    /// Base (unmodulated) grid bias for push-pull stages.
    /// Stored so sidechain CV can be subtracted without accumulation.
    pub base_grid_bias: crate::Wave,
    /// Sample counter for throttling multi-NL scattering recomputes.
    /// When a pot inside a multi-NL R-type adaptor changes, the O(n³) matrix
    /// inversion is deferred and flushed every 32 samples (~0.7 ms at 48 kHz).
    pub multi_nl_recompute_counter: u32,
    // (stage_order removed — `stages` vec IS the order)
    /// Node-based signal routing buffer for parallel-path topologies.
    /// Maps circuit graph node IDs to signal values. When non-empty, multi-NL
    /// stages read from their injection_node_id and write to their output_node_id,
    /// enabling correct parallel channel processing (e.g., 670 sidechain).
    pub node_signals: Vec<(usize, crate::Wave)>,
    /// BBD wet/dry mix (0.0 = fully dry, 1.0 = fully wet).
    /// Controlled by "Blend"/"Mix" knobs on BBD delay pedals.
    pub bbd_wet_mix: crate::Wave,
    /// Component ID of the pot controlling BBD wet/dry mix.
    /// When set, the pot position is read directly after pot updates.
    pub bbd_mix_pot_id: Option<String>,
    /// Trigger impulse sources for drum/percussion circuits.
    pub triggers: Vec<TriggerState>,
    /// Original passive component values for reset support.
    /// Maps comp_id -> (kind_str, original_value).
    /// Skipped for serde: `&'static str` cannot be deserialized from owned data.
    /// Reconstructed on the M7 target if needed.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub original_passive_values: HashMap<String, (&'static str, crate::Wave)>,

    /// Named voltage port bindings (audio I/O, CV, gates, envelope outputs).
    /// Ports inject/extract voltage at circuit nodes at audio rate without
    /// impedance recompute. Declared in the .pedal `ports {}` section.
    pub ports: Vec<NamedPortBinding>,
    /// Dense port value buffer — indexed by NamedPortBinding::index.
    /// Input ports: written by external code via set_port() before process().
    /// Output ports: written by process(), read via get_port() after.
    pub port_values: Vec<crate::Wave>,
    /// Compiler-synthesized INTERNAL delayed ports (Phase 2b) — the z⁻¹ carry
    /// table for de-fused sub-networks (e.g. the LA-2A detector taps + EL_drive).
    /// Not user-overridable; written/read entirely inside `process()`.
    #[cfg_attr(feature = "serde", serde(default))]
    pub internal_ports: Vec<InternalPortBinding>,
    /// Compiler-synthesized DETECTOR → photocoupler-LED coupling (Phase 3) — the
    /// gain-reduction loop closure for a cross-network feedback detector
    /// (LA-2A). `None` for every other circuit (envelope-follower opto levelers
    /// drive the LED via `ModulationTarget::PhotocouplerLed`, unchanged).
    #[cfg_attr(feature = "serde", serde(default))]
    pub detector_led_coupling: Option<DetectorLedCoupling>,
    /// Auto-init flag: cache_all_vs_pointers runs once on first process() call.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub initialized: bool,
}

/// Gain-like control labels.
#[allow(dead_code)]
pub fn is_gain_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "drive" | "gain" | "fuzz" | "sustain" | "distortion" | "sensitivity"
    )
}

/// Level-like control labels.
#[allow(dead_code)]
pub fn is_level_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "level" | "volume" | "output"
    )
}

/// Rate-like control labels (for LFO).
#[allow(dead_code)]
pub fn is_rate_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "rate" | "speed" | "tempo"
    )
}

/// Depth-like control labels (for LFO modulation amount).
#[allow(dead_code)]
pub fn is_depth_label(label: &str) -> bool {
    matches!(label.to_ascii_lowercase().as_str(), "depth" | "width")
}

/// Delay time control labels.
#[allow(dead_code)]
pub fn is_delay_time_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "time" | "delay" | "repeat_rate" | "echo"
    )
}

/// Delay feedback/intensity control labels.
#[allow(dead_code)]
pub fn is_delay_feedback_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "feedback" | "repeats" | "intensity" | "regeneration"
    )
}

/// Wet/dry mix control labels.
pub fn is_mix_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "mix" | "blend" | "wet" | "dry/wet" | "wet/dry"
    )
}

impl CompiledPedal {
    fn normalized_control_name(input: &str) -> alloc::string::String {
        input
            .chars()
            .filter(|c| c.is_ascii_alphanumeric())
            .map(|c| c.to_ascii_lowercase())
            .collect()
    }

    fn control_label_matches_port(label: &str, port_name: &str) -> bool {
        let label = Self::normalized_control_name(label);
        if label.is_empty() {
            return false;
        }

        let port = Self::normalized_control_name(port_name);
        if port == label {
            return true;
        }

        port.strip_prefix("cv")
            .is_some_and(|suffix| suffix == label)
            || port
                .strip_suffix("cv")
                .is_some_and(|prefix| prefix == label)
    }

    fn set_matching_input_port_control(&mut self, label: &str, value: crate::Wave) -> bool {
        let mut found = false;
        for port in &self.ports {
            if port.direction == crate::PortDirection::Input
                && port.index < self.port_values.len()
                && Self::control_label_matches_port(label, &port.name)
            {
                self.port_values[port.index] = value;
                found = true;
            }
        }
        found
    }

    /// Set the supply voltage and update all voltage-dependent models.
    ///
    /// This affects:
    /// - Headroom for rail saturation modeling
    /// - Op-amp output swing (v_max)
    /// - Triode/pentode plate voltage limits (v_max = B+)
    /// - BJT collector-emitter voltage limits (v_max = Vcc)
    ///
    /// For single-supply pedals biased at Vsupply/2, the op-amp can swing
    /// from about 1.5V to (Vsupply - 1.5V), giving v_max ≈ (Vsupply/2) - 1.5V.
    ///
    /// For tube circuits, the plate voltage can swing from 0V to B+ (supply).
    /// For BJT circuits, Vce can swing from ~0V to Vcc (supply).
    pub fn set_supply_voltage(&mut self, voltage: crate::Wave) {
        let prev_voltage = self.supply_voltage;
        // Support negative supplies (PNP positive-ground, e.g. -9V Rangemaster)
        self.supply_voltage = if voltage >= 0.0 {
            voltage.clamp(5.0, 500.0)
        } else {
            voltage.clamp(-500.0, -5.0)
        };

        // Calculate op-amp v_max from supply voltage.
        // For single-supply biased at Vsupply/2: v_max = (Vsupply/2) - saturation_margin
        // Saturation margin is typically 1.0-1.5V for most op-amps.
        // For high voltages (tube gear), the margin stays ~1.5V.
        let saturation_margin = 1.5;
        let abs_voltage = voltage.abs();
        let opamp_v_max = (abs_voltage / 2.0 - saturation_margin).max(1.0);

        // For tubes and transistors, v_max is the magnitude of the supply voltage
        // (plate/collector can swing from 0V to |B+/Vcc|)
        let tube_v_max = abs_voltage;
        let bjt_v_max = abs_voltage;

        // Propagate v_max to standalone op-amp stages
        for stage in &mut self.opamp_stages {
            stage.opamp.set_v_max(opamp_v_max);
        }

        // Propagate v_max to push-pull stages
        for pp in &mut self.push_pull_stages {
            pp.push_root.set_v_max(tube_v_max);
            pp.pull_root.set_v_max(tube_v_max);
        }

        // Propagate v_max to all stages (WDF roots + multi-NL devices).
        for stage in &mut self.stages {
            match stage {
                Stage::Wdf(wdf) => match &mut wdf.root {
                    RootKind::OpAmp(op) => op.set_v_max(opamp_v_max),
                    RootKind::Triode(t) => t.set_v_max(tube_v_max),
                    RootKind::VariMu(t) => t.set_v_max(tube_v_max),
                    RootKind::Pentode(p) => p.set_v_max(tube_v_max),
                    _ => {}
                },
                Stage::MultiNl(mnl) => {
                    for device in &mut mnl.nl_devices {
                        match device {
                            NlDeviceKind::Triode(t) => t.set_v_max(tube_v_max),
                            NlDeviceKind::Pentode(p) => p.set_v_max(tube_v_max),
                            NlDeviceKind::VariMu(t) => t.set_v_max(tube_v_max),
                            NlDeviceKind::Diode(_)
                            | NlDeviceKind::DiodePair(_)
                            | NlDeviceKind::ExplicitDiode(_)
                            | NlDeviceKind::ExplicitDiodePair(_)
                            | NlDeviceKind::Jfet(_)
                            | NlDeviceKind::Ota(_) => {}
                        }
                    }
                    if let Some(ref mut dg) = mnl.device_groups {
                        for group in &mut dg.groups {
                            match group {
                                NlDeviceGroupKind::VariMuThreePort(t) => t.set_v_max(tube_v_max),
                                NlDeviceGroupKind::TriodeThreePort(t) => t.set_v_max(tube_v_max),
                                NlDeviceGroupKind::PentodeThreePort(p) => p.set_v_max(tube_v_max),
                                NlDeviceGroupKind::BjtTwoPort(b) => b.set_v_max(bjt_v_max),
                                NlDeviceGroupKind::EbersMollTwoPort(e) => e.set_v_max(bjt_v_max),
                                NlDeviceGroupKind::SinglePort(d) => match d {
                                    NlDeviceKind::Triode(t) => t.set_v_max(tube_v_max),
                                    NlDeviceKind::Pentode(p) => p.set_v_max(tube_v_max),
                                    NlDeviceKind::VariMu(t) => t.set_v_max(tube_v_max),
                                    NlDeviceKind::Diode(_)
                                    | NlDeviceKind::DiodePair(_)
                                    | NlDeviceKind::ExplicitDiode(_)
                                    | NlDeviceKind::ExplicitDiodePair(_)
                                    | NlDeviceKind::Jfet(_)
                                    | NlDeviceKind::Ota(_) => {}
                                },
                            }
                        }
                    }
                    mnl.update_supply_voltage(voltage);
                }
                _ => {}
            }
        }

        // Update single-NL WDF stage VCC bias.
        if prev_voltage.abs() > 0.0 {
            let scale = self.supply_voltage / prev_voltage;
            for stage in &mut self.stages {
                if let Stage::Wdf(wdf) = stage {
                    if wdf.vcc_injection_coeff != 0.0 {
                        wdf.vcc_injection_coeff *= scale;
                    }
                }
            }
        }
    }

    /// Resolve and cache runtime-only state for all stages.
    ///
    /// Must be called once after construction (or deserialization) and before
    /// the first `process()` call. This:
    /// - Caches raw pointers to VS leaves (eliminates per-sample tree walking)
    /// - Precomputes K-table inverse scales (eliminates per-sample division)
    /// - Precomputes StateSpace v_rail (eliminates per-sample division)
    pub fn cache_all_vs_pointers(&mut self) {
        self.stage_route_plan = StageRoutePlan::from_compiled_parts(&self.ports, &self.stages);
        for stage_idx in 0..self.stages.len() {
            match &mut self.stages[stage_idx] {
                Stage::Wdf(ref mut wdf) => {
                    // Cache VS pointers
                    let port_names: alloc::vec::Vec<alloc::string::String> = self
                        .ports
                        .iter()
                        .filter(|p| {
                            p.direction == crate::PortDirection::Input && p.stage_idx == stage_idx
                        })
                        .map(|p| p.name.clone())
                        .collect();
                    let name_refs: alloc::vec::Vec<&str> =
                        port_names.iter().map(|s| s.as_str()).collect();
                    wdf.cache_vs_pointers(&name_refs);
                    // Precompute K-table inverse scales
                    if let Some(ref mut kt) = wdf.k_table {
                        kt.precompute_scales();
                    }
                }
                Stage::StateSpace(ref mut ss) => {
                    ss.recompute_v_rail();
                }
                Stage::Blockwise(ref mut bkm) => {
                    // Resolve port name → port_values index (once, not per-sample)
                    bkm.port_index_cache = bkm
                        .vs_port_map
                        .iter()
                        .map(|(name, _)| {
                            self.ports
                                .iter()
                                .find(|p| &p.name == name)
                                .map(|p| p.index)
                                .unwrap_or(usize::MAX)
                        })
                        .collect();
                    bkm.solve_dc_operating_point();
                }
                Stage::SerialDelayedFeedback(ref mut serial) => {
                    for wdf in &mut serial.stages {
                        if let Some(ref mut kt) = wdf.k_table {
                            kt.precompute_scales();
                        }
                    }
                }
                _ => {}
            }
        }
    }

    fn process_stage_route_plan(&mut self) -> Option<crate::Wave> {
        let route = self.stage_route_plan.primary_bkm.clone()?;
        let mut boundary_outputs = alloc::vec::Vec::new();
        for drive in &route.boundary_drives {
            let mut input = 0.0 as crate::Wave;
            for port in drive.positive_processor_ports() {
                let port_idx = port.get();
                if port_idx < self.port_values.len() {
                    input += self.port_values[port_idx];
                }
            }
            for port in drive.negative_processor_ports() {
                let port_idx = port.get();
                if port_idx < self.port_values.len() {
                    input -= self.port_values[port_idx];
                }
            }

            if let Some(Stage::Wdf(wdf)) = self.stages.get_mut(drive.source_stage_idx) {
                let out = wdf.process(input);
                if out.is_finite() {
                    boundary_outputs.push((drive.clone(), out));
                }
            }
        }

        let Stage::Blockwise(bkm_stage) = self.stages.get_mut(route.stage_idx)? else {
            return None;
        };
        let n_vs = bkm_stage.vs_port_map.len();
        let mut vs_signals = alloc::vec![0.0 as crate::Wave; n_vs];
        for binding in &route.vs_bindings {
            let port_idx = binding.processor_port().get();
            if binding.signal_index < vs_signals.len() && port_idx < self.port_values.len() {
                vs_signals[binding.signal_index] = self.port_values[port_idx];
            }
        }

        let mut boundary_drives: alloc::vec::Vec<BoundaryIncidentDrive> = alloc::vec::Vec::new();
        for (drive, out) in boundary_outputs {
            let positive_ports: alloc::vec::Vec<_> =
                drive.positive_target_scattering_ports().collect();
            let negative_ports: alloc::vec::Vec<_> =
                drive.negative_target_scattering_ports().collect();
            push_differential_voltage_drives_for_ports(
                &mut boundary_drives,
                &positive_ports,
                &negative_ports,
                PortVoltage::new(out),
            );
        }

        let output = bkm_stage.process_with_boundary_drives(&boundary_drives, &vs_signals);
        let output = self.condition_output_signal(output);
        for port_idx in route.output_port_indices {
            if port_idx < self.port_values.len() {
                self.port_values[port_idx] = output;
            }
        }
        Some(output)
    }

    fn condition_output_signal(&mut self, mut signal: crate::Wave) -> crate::Wave {
        if let Some(ref mut loading) = self.output_loading {
            signal = loading.process(signal);
        }
        if let Some((a1, b0, ref mut y_prev, ref mut x_prev)) = self.output_dc_block {
            let x = signal;
            let y = b0 * (x - *x_prev) + a1 * *y_prev;
            *x_prev = x;
            *y_prev = crate::stage::flush_denormal(y);
            signal = *y_prev;
        }
        if !signal.is_finite() {
            signal = 0.0;
        }
        signal * self.output_gain
    }

    /// Get the tolerance seed for this pedal unit (for diagnostics/UI).
    pub fn tolerance_seed(&self) -> u64 {
        self.tolerance_seed
    }

    /// Get the oversampling factor used for this pedal.
    pub fn oversampling(&self) -> OversamplingFactor {
        self.oversampling
    }

    /// Get the number of WDF stages and op-amp stages.
    /// Returns (stage_count, opamp_count).
    pub fn wdf_element_counts(&self) -> (u32, u32) {
        let wdf_count = self
            .stages
            .iter()
            .filter(|s| matches!(s, Stage::Wdf(_)))
            .count() as u32;
        let stage_count = wdf_count + self.push_pull_stages.len() as u32;
        (stage_count, self.opamp_stages.len() as u32)
    }

    /// Debug: return opamp gains and feedback pot IDs for all stages.
    pub fn opamp_debug_info(&self) -> Vec<(usize, crate::Wave, Option<String>)> {
        self.stages
            .iter()
            .enumerate()
            .filter_map(|(i, s)| {
                if let Stage::Wdf(wdf) = s {
                    wdf.opamp_gain().map(|g| {
                        let pot_id = wdf.feedback_pot_id().map(|s| s.to_string());
                        (i, g, pot_id)
                    })
                } else {
                    None
                }
            })
            .collect()
    }

    /// Debug: return multi-NL stage scattering info.
    pub fn multi_nl_debug_info(&self) -> Vec<MultiNlDebugInfo> {
        self.stages
            .iter()
            .enumerate()
            .filter_map(|(i, s)| {
                if let Stage::MultiNl(mnl) = s {
                    Some((
                        i,
                        mnl.n_nl,
                        mnl.scattering.s_nl_adapted.clone(),
                        mnl.compensation,
                        mnl.nl_port_resistances.clone(),
                    ))
                } else {
                    None
                }
            })
            .collect()
    }

    /// Get the supply voltage.
    pub fn supply_voltage(&self) -> crate::Wave {
        self.supply_voltage
    }

    /// Get the input impedance of this pedal (Ω).
    ///
    /// Returns the WDF port resistance seen looking into the input.
    /// This is the impedance of the first WDF stage's tree.
    /// If the pedal has no stages, returns high impedance (1MΩ).
    pub fn input_impedance(&self) -> crate::Wave {
        self.stages
            .iter()
            .find_map(|s| {
                if let Stage::Wdf(wdf) = s {
                    Some(wdf.tree.port_resistance())
                } else {
                    None
                }
            })
            .unwrap_or(1_000_000.0) // High-Z default (doesn't load source)
    }

    /// Get the output impedance of this pedal (Ω).
    ///
    /// Returns the WDF port resistance seen looking back into the output.
    /// This is the impedance of the last WDF stage's tree.
    /// If the pedal has no stages, returns low impedance (1kΩ).
    pub fn output_impedance(&self) -> crate::Wave {
        self.stages
            .iter()
            .rev()
            .find_map(|s| {
                if let Stage::Wdf(wdf) = s {
                    Some(wdf.tree.port_resistance())
                } else {
                    None
                }
            })
            .unwrap_or(1_000.0) // Low-Z default (can drive loads)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Inter-Plugin Impedance API
    // ═══════════════════════════════════════════════════════════════════════════

    /// Set the source impedance driving this circuit's input.
    ///
    /// This models the electrical interaction between whatever is driving this
    /// circuit (guitar pickup, upstream pedal, preamp output) and the circuit's
    /// input stage. The interaction produces:
    ///
    /// - **DC attenuation**: voltage divider formed by source and load impedance
    /// - **Frequency-dependent rolloff**: combined source/load capacitance creates
    ///   a low-pass filter that rolls off high frequencies
    ///
    /// # Arguments
    ///
    /// * `resistance` - Source resistance in Ohms (e.g., 7000 for guitar pickup)
    /// * `capacitance` - Source capacitance in Farads (e.g., 500e-12 for cable)
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Guitar pickup through 15ft cable
    /// pedal.set_source_impedance(7000.0, 500e-12);
    ///
    /// // Buffered pedal output
    /// pedal.set_source_impedance(1000.0, 50e-12);
    /// ```
    pub fn set_source_impedance(&mut self, resistance: crate::Wave, capacitance: crate::Wave) {
        let source = ImpedanceModel {
            resistance,
            capacitance,
        };

        // Use this circuit's input impedance as the load
        let load = ImpedanceModel {
            resistance: self.input_impedance(),
            capacitance: 50e-12, // Typical input capacitance
        };

        self.input_loading = Some(InterstageLoading::new(source, load, self.sample_rate));
    }

    /// Set the load impedance this circuit drives.
    ///
    /// This models the electrical interaction between this circuit's output stage
    /// and whatever it's driving (downstream pedal, amp input, mixer). The
    /// interaction produces:
    ///
    /// - **DC attenuation**: if the load impedance is low relative to output impedance
    /// - **Frequency-dependent rolloff**: if significant capacitance is present
    ///
    /// # Arguments
    ///
    /// * `resistance` - Load resistance in Ohms (e.g., 1_000_000 for tube amp)
    /// * `capacitance` - Load capacitance in Farads (e.g., 50e-12 for Miller cap)
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Driving a tube amp input
    /// pedal.set_load_impedance(1_000_000.0, 50e-12);
    ///
    /// // Driving a Fuzz Face (low impedance!)
    /// pedal.set_load_impedance(10_000.0, 50e-12);
    /// ```
    pub fn set_load_impedance(&mut self, resistance: crate::Wave, capacitance: crate::Wave) {
        // Use this circuit's output impedance as the source
        let source = ImpedanceModel {
            resistance: self.output_impedance(),
            capacitance: 30e-12, // Typical output capacitance
        };

        let load = ImpedanceModel {
            resistance,
            capacitance,
        };

        self.output_loading = Some(InterstageLoading::new(source, load, self.sample_rate));
    }

    /// Clear source impedance modeling (use ideal voltage source).
    pub fn clear_source_impedance(&mut self) {
        self.input_loading = None;
    }

    /// Clear load impedance modeling (use ideal open-circuit load).
    pub fn clear_load_impedance(&mut self) {
        self.output_loading = None;
    }

    /// Check if source impedance modeling is active.
    pub fn has_source_impedance(&self) -> bool {
        self.input_loading.is_some()
    }

    /// Check if load impedance modeling is active.
    pub fn has_load_impedance(&self) -> bool {
        self.output_loading.is_some()
    }

    // TODO: DebugStats lives in pedalkernel crate, not rt. Re-enable when ported.
    // #[cfg(debug_assertions)]
    // pub fn set_debug_stats(&mut self, stats: alloc::sync::Arc<crate::debug::DebugStats>) { ... }
    // #[cfg(debug_assertions)]
    // pub fn debug_stats(&self) -> Option<&alloc::sync::Arc<crate::debug::DebugStats>> { ... }

    /// Enable or disable debug mode.
    #[cfg(debug_assertions)]
    pub fn set_debug_enabled(&self, _enabled: bool) {
        // TODO: DebugStats not yet in rt crate
    }

    /// Enable metering for real-time UI visualization.
    ///
    /// Creates a metrics accumulator and ring buffer. Call this before starting
    /// audio processing, then retrieve the ring buffer with `metrics_buffer()`
    /// to read metrics in the UI thread.
    ///
    /// `block_size`: Number of samples between metric reductions (typically 128 or 256).
    pub fn enable_metering(&mut self, block_size: usize) {
        let buffer = Arc::new(MetricsRingBuffer::new(16)); // ~16 frames buffered
        self.metrics_accumulator = Some(MetricsAccumulator::new(block_size));
        self.metrics_buffer = Some(buffer);

        // Set nominal supply voltage for sag calculation
        if let Some(ref mut acc) = self.metrics_accumulator {
            acc.set_nominal_supply(self.supply_voltage as f32);
        }

        for subcircuit in &mut self.subcircuit_processors {
            subcircuit.circuit.enable_metering(block_size);
        }
    }

    /// Get a reference to the metrics ring buffer for UI thread reading.
    ///
    /// Returns `None` if metering is not enabled.
    pub fn metrics_buffer(&self) -> Option<Arc<MetricsRingBuffer>> {
        self.metrics_buffer.clone()
    }

    /// Attach a live runtime diagnostics ring (IPC Phase B) over a caller-owned
    /// byte region (the runtime-ring section of an mmap'd IPC file).
    ///
    /// The processor will, once per metering block, build a
    /// [`crate::diag_ring::RuntimeFrame`] (NR convergence + per-BJT op-points +
    /// per-stage levels) and push it lock-free into the ring. Requires metering
    /// to be enabled (the per-block cadence + level accumulators come from it).
    ///
    /// # Safety
    /// `ptr` must be valid and writable for [`crate::diag_ring::region_len`]`
    /// (capacity)` bytes for as long as this processor processes audio, and must
    /// not be aliased by any other writer. The std host (`diag_ipc`) holds the
    /// mmap that owns this region's lifetime.
    #[cfg(feature = "diag")]
    pub unsafe fn attach_diag_ring(&mut self, ptr: *mut u8, capacity: usize) {
        self.diag_ring = Some(crate::diag_ring::RuntimeRing::init(ptr, capacity));
    }

    /// Read the latest metrics (convenience method for the UI thread).
    ///
    /// Returns default metrics if metering is not enabled.
    pub fn read_metrics(&self) -> UiMetrics {
        let mut metrics = self
            .metrics_buffer
            .as_ref()
            .map(|b| b.read_latest())
            .unwrap_or_default();

        if metrics.nr_solve_count == 0 {
            for (stage_idx, stage) in self.stages.iter().enumerate() {
                let Stage::Blockwise(blockwise) = stage else {
                    continue;
                };
                let diag = blockwise.solve_diagnostics();
                if diag.iterations == 0 {
                    continue;
                }
                metrics.nr_solve_count = metrics.nr_solve_count.saturating_add(1);
                metrics.nr_total_iterations =
                    metrics.nr_total_iterations.saturating_add(diag.iterations);
                metrics.nr_max_iterations = metrics
                    .nr_max_iterations
                    .max(diag.iterations.min(u16::MAX as u32) as u16);
                if !diag.converged || diag.linear_solve_failed {
                    metrics.nr_nonconverged_count = metrics.nr_nonconverged_count.saturating_add(1);
                }
                if diag.residual.is_finite() {
                    metrics.nr_max_residual = metrics.nr_max_residual.max(diag.residual as f32);
                }
                if stage_idx < crate::metering::MAX_STAGES {
                    metrics.stage_nr_iterations[stage_idx] = metrics.stage_nr_iterations[stage_idx]
                        .max(diag.iterations.min(u16::MAX as u32) as u16);
                    if diag.residual.is_finite() {
                        metrics.stage_nr_residual[stage_idx] =
                            metrics.stage_nr_residual[stage_idx].max(diag.residual as f32);
                    }
                    if !diag.converged || diag.linear_solve_failed {
                        metrics.stage_nr_nonconverged[stage_idx] =
                            metrics.stage_nr_nonconverged[stage_idx].saturating_add(1);
                    }
                    metrics.stage_count = metrics
                        .stage_count
                        .max((stage_idx + 1).min(crate::metering::MAX_STAGES) as u8);
                }
            }
        }

        for subcircuit in &self.subcircuit_processors {
            let child = subcircuit.circuit.read_metrics();
            metrics.nr_solve_count = metrics.nr_solve_count.saturating_add(child.nr_solve_count);
            metrics.nr_total_iterations = metrics
                .nr_total_iterations
                .saturating_add(child.nr_total_iterations);
            metrics.nr_max_iterations = metrics.nr_max_iterations.max(child.nr_max_iterations);
            metrics.nr_nonconverged_count = metrics
                .nr_nonconverged_count
                .saturating_add(child.nr_nonconverged_count);
            metrics.nr_max_residual = metrics.nr_max_residual.max(child.nr_max_residual);
            metrics.nonfinite_count = metrics
                .nonfinite_count
                .saturating_add(child.nonfinite_count);
            metrics.blowup_count = metrics.blowup_count.saturating_add(child.blowup_count);
            metrics.max_abs_sample = metrics.max_abs_sample.max(child.max_abs_sample);
            metrics.stage_count = metrics.stage_count.max(child.stage_count);
            for i in 0..crate::metering::MAX_STAGES {
                metrics.stage_levels[i] = metrics.stage_levels[i].max(child.stage_levels[i]);
                metrics.stage_nr_iterations[i] =
                    metrics.stage_nr_iterations[i].max(child.stage_nr_iterations[i]);
                metrics.stage_nr_residual[i] =
                    metrics.stage_nr_residual[i].max(child.stage_nr_residual[i]);
                metrics.stage_nr_nonconverged[i] =
                    metrics.stage_nr_nonconverged[i].saturating_add(child.stage_nr_nonconverged[i]);
            }
        }

        metrics
    }

    /// List all editable passive components across all stages.
    pub fn list_editable_components(&self) -> Vec<(String, &'static str, crate::Wave)> {
        let mut result = Vec::new();
        for stage in &self.stages {
            match stage {
                Stage::Wdf(wdf) => {
                    result.extend(wdf.tree.list_editable_leaves());
                }
                Stage::MultiNl(mnl) => {
                    result.extend(mnl.pot_children.iter().flat_map(|child| {
                        child
                            .list_editable_leaves()
                            .into_iter()
                            .filter(|(_, kind, _)| *kind == "pot")
                    }));
                }
                _ => {}
            }
        }
        for stage in &self.push_pull_stages {
            result.extend(stage.push_tree.list_editable_leaves());
            result.extend(stage.pull_tree.list_editable_leaves());
        }
        result
    }

    /// Set a passive component's value by comp_id across all stages.
    /// Automatically determines component type from original_passive_values.
    /// After setting, recomputes all affected WDF tree coefficients.
    pub fn set_passive(&mut self, comp_id: &str, value: crate::Wave) -> bool {
        let kind = match self.original_passive_values.get(comp_id) {
            Some((k, _)) => *k,
            None => return false,
        };
        let sample_rate = self.sample_rate;
        let mut found = false;

        // Search all stages
        for stage in &mut self.stages {
            match stage {
                Stage::Wdf(wdf) => {
                    let hit = match kind {
                        "resistor" => wdf.tree.set_resistor(comp_id, value),
                        "capacitor" => wdf.tree.set_capacitor(comp_id, value, sample_rate),
                        "inductor" => wdf.tree.set_inductor(comp_id, value, sample_rate),
                        _ => false,
                    };
                    if hit {
                        wdf.tree.recompute();
                        found = true;
                    }
                }
                Stage::MultiNl(mnl) => {
                    if kind == "pot" && mnl.set_pot(comp_id, value) {
                        found = true;
                    }
                }
                _ => {}
            }
        }

        // Search push-pull stages
        for stage in &mut self.push_pull_stages {
            let hit_push = match kind {
                "resistor" => stage.push_tree.set_resistor(comp_id, value),
                "capacitor" => stage.push_tree.set_capacitor(comp_id, value, sample_rate),
                "inductor" => stage.push_tree.set_inductor(comp_id, value, sample_rate),
                _ => false,
            };
            let hit_pull = match kind {
                "resistor" => stage.pull_tree.set_resistor(comp_id, value),
                "capacitor" => stage.pull_tree.set_capacitor(comp_id, value, sample_rate),
                "inductor" => stage.pull_tree.set_inductor(comp_id, value, sample_rate),
                _ => false,
            };
            if hit_push {
                stage.push_tree.recompute();
                found = true;
            }
            if hit_pull {
                stage.pull_tree.recompute();
                found = true;
            }
        }

        found
    }

    /// Reset a passive component to its original value.
    pub fn reset_passive(&mut self, comp_id: &str) -> bool {
        if let Some(&(_, original_value)) = self.original_passive_values.get(comp_id) {
            self.set_passive(comp_id, original_value)
        } else {
            false
        }
    }

    /// Populate original_passive_values from all stages' current editable leaves.
    /// Should be called once after compilation is complete.
    pub fn snapshot_original_values(&mut self) {
        for (id, kind, value) in self.list_editable_components() {
            self.original_passive_values
                .entry(id)
                .or_insert((kind, value));
        }
    }

    /// Set a control by its label (e.g., "Drive", "Level", "Rate").
    /// Resolve a control label to its binding indices. Call once at init,
    /// then use [`set_control_indexed`] at audio rate to skip string matching.
    ///
    /// Returns a vec of control indices that match the label (there may be
    /// multiple bindings for the same label, e.g., ganged pots).
    pub fn resolve_control(&self, label: &str) -> Vec<usize> {
        self.controls
            .iter()
            .enumerate()
            .filter(|(_, c)| c.label.eq_ignore_ascii_case(label))
            .map(|(i, _)| i)
            .collect()
    }

    /// Set a control by pre-resolved index. No string matching — O(1).
    ///
    /// Use [`resolve_control`] at init to get the index, then call this
    /// at audio rate for envelope-modulated parameters.
    pub fn set_control_indexed(&mut self, control_idx: usize, value: crate::Wave) {
        if control_idx >= self.controls.len() {
            return;
        }
        let value = value.clamp(0.0, 1.0);
        let (r0, r1) = self.controls[control_idx].range;
        let value = (r0 + value * (r1 - r0)).clamp(0.0, 1.0);
        match &self.controls[control_idx].target {
            ControlTarget::PotInStage(_)
            | ControlTarget::PotInIirStage(_)
            | ControlTarget::PotInMultiNlStage(_, _)
            | ControlTarget::PotInBlackFeedbackStage(_)
            | ControlTarget::WiperDivider { .. } => {
                if let Some(smoother) = self
                    .pot_smoothers
                    .iter_mut()
                    .find(|s| s.control_idx == control_idx)
                {
                    smoother.set_target(value);
                }
            }
            _ => {
                // For non-pot targets, fall back to the full set_control path
                let label = self.controls[control_idx].label.clone();
                self.set_control(&label, value);
            }
        }
    }

    pub fn set_control(&mut self, label: &str, value: crate::Wave) {
        let value = value.clamp(0.0, 1.0);
        let mut found = false;
        for i in 0..self.controls.len() {
            if !self.controls[i].label.eq_ignore_ascii_case(label) {
                continue;
            }
            found = true;
            // Apply range mapping only. Taper is applied by each leaf/stage
            // internally — WdfPot applies taper in set_control, IIR/BF
            // apply it when computing gain/coefficients.
            let (r0, r1) = self.controls[i].range;
            let value = (r0 + value * (r1 - r0)).clamp(0.0, 1.0);
            match &self.controls[i].target {
                ControlTarget::PotInStage(_)
                | ControlTarget::PotInIirStage(_)
                | ControlTarget::PotInMultiNlStage(_, _)
                | ControlTarget::PotInBlackFeedbackStage(_)
                | ControlTarget::WiperDivider { .. } => {
                    if let Some(smoother) =
                        self.pot_smoothers.iter_mut().find(|s| s.control_idx == i)
                    {
                        smoother.set_target(value);
                    } else {
                        // Fallback: no smoother (shouldn't happen), update immediately.
                        // `value` is already tapered+ranged from above.
                        let comp_id = self.controls[i].component_id.clone();
                        for stage in &mut self.stages {
                            stage.set_control_pot(&comp_id, value);
                        }
                        for wd in &mut self.wiper_dividers {
                            if wd.pot_comp_id == comp_id {
                                wd.position = wd.taper.apply(value);
                            }
                        }
                        if self.bbd_mix_pot_id.as_deref() == Some(&*comp_id) {
                            self.bbd_wet_mix = value;
                        }
                        self.apply_pot_mirrors(&comp_id, value);
                    }
                }
                ControlTarget::SidechainControl(sc_idx) => {
                    let sc_idx = *sc_idx;
                    if let Some(sc) = self.sidechains.get_mut(sc_idx) {
                        sc.set_control(label, value);
                    }
                }
                ControlTarget::SubcircuitControl { subcircuit_idx } => {
                    let sc_idx = *subcircuit_idx;
                    if let Some(sc) = self.subcircuit_processors.get_mut(sc_idx) {
                        sc.set_control(label, value);
                    }
                }
                ControlTarget::LfoRate(lfo_idx) => {
                    let lfo_idx = *lfo_idx;
                    if let Some(binding) = self.lfos.get_mut(lfo_idx) {
                        // Scale rate around base_freq: 0.1x to 10x (100x range)
                        // pot=0 -> 0.1x, pot=0.5 -> 1x, pot=1 -> 10x
                        let scale =
                            0.1 as crate::Wave * crate::math::powf(100.0 as crate::Wave, value);
                        let rate = binding.base_freq * scale;
                        binding.lfo.set_rate(rate);
                    }
                }
                ControlTarget::LfoDepth(lfo_idx) => {
                    let lfo_idx = *lfo_idx;
                    if let Some(binding) = self.lfos.get_mut(lfo_idx) {
                        binding.lfo.set_depth(value);
                    }
                }
                ControlTarget::DelayTime(delay_idx) => {
                    let delay_idx = *delay_idx;
                    if let Some(dl) = self.delay_lines.get_mut(delay_idx) {
                        dl.delay_line.set_delay_normalized(value);
                    }
                }
                ControlTarget::DelayFeedback(delay_idx) => {
                    let delay_idx = *delay_idx;
                    if let Some(dl) = self.delay_lines.get_mut(delay_idx) {
                        dl.delay_line.set_feedback(value * 0.95); // Scale to max 0.95
                    }
                }
                ControlTarget::BbdClockRate(bbd_idx) => {
                    let bbd_idx = *bbd_idx;
                    if let Some(bbd) = self.bbds.get_mut(bbd_idx) {
                        bbd.set_delay_normalized(value);
                    }
                }
                ControlTarget::BbdFeedback(bbd_idx) => {
                    let bbd_idx = *bbd_idx;
                    if let Some(bbd) = self.bbds.get_mut(bbd_idx) {
                        bbd.set_feedback(value * 0.95);
                    }
                }
                ControlTarget::SpringDwell(spring_idx) => {
                    let spring_idx = *spring_idx;
                    if let Some(s) = self.springs.get_mut(spring_idx) {
                        s.tank.set_dwell_normalized(value);
                    }
                }
                ControlTarget::SpringDecay(spring_idx) => {
                    let spring_idx = *spring_idx;
                    if let Some(s) = self.springs.get_mut(spring_idx) {
                        s.tank.set_decay_normalized(value);
                    }
                }
                ControlTarget::SpringDamping(spring_idx) => {
                    let spring_idx = *spring_idx;
                    if let Some(s) = self.springs.get_mut(spring_idx) {
                        s.tank.set_damping_normalized(value);
                    }
                }
                ControlTarget::SpringMix(spring_idx) => {
                    let spring_idx = *spring_idx;
                    if let Some(s) = self.springs.get_mut(spring_idx) {
                        s.wet_mix = value.clamp(0.0, 1.0);
                    }
                }
                ControlTarget::SwitchPosition {
                    switch_id,
                    num_positions,
                } => {
                    // Convert normalized 0-1 value to discrete position index
                    // value=0 → pos=0, value=1 → pos=num_positions-1
                    let num_positions = *num_positions;
                    let position = if num_positions <= 1 {
                        0
                    } else {
                        ((value * (num_positions as crate::Wave - 0.001)) as usize)
                            .min(num_positions - 1)
                    };
                    // Update all stages' switched resistors
                    for stage in &mut self.stages {
                        if let Stage::Wdf(wdf) = stage {
                            wdf.tree.set_switch_position(switch_id, position);
                            wdf.tree.recompute();
                        }
                    }
                }
                ControlTarget::Trigger(idx) => {
                    if value > 0.5 {
                        if let Some(trig) = self.triggers.get_mut(*idx) {
                            trig.fire();
                        }
                    }
                }
            }
        }

        if !found {
            found = self.set_matching_input_port_control(label, value);
        }

        if !found {
            // Forward unmatched controls to sidechain sub-circuits.
            // Sidechain controls (e.g., "Lat Time Constant") are compiled into
            // the sidechain's own CompiledPedal and handled there.
            for sc in &mut self.sidechains {
                sc.set_control(label, value);
            }
        }
    }

    pub fn set_control_immediate(&mut self, label: &str, value: crate::Wave) {
        let value = value.clamp(0.0, 1.0);
        for i in 0..self.controls.len() {
            if !self.controls[i].label.eq_ignore_ascii_case(label) {
                continue;
            }
            let (r0, r1) = self.controls[i].range;
            let value = (r0 + value * (r1 - r0)).clamp(0.0, 1.0);
            match &self.controls[i].target {
                ControlTarget::PotInStage(_)
                | ControlTarget::PotInIirStage(_)
                | ControlTarget::PotInMultiNlStage(_, _)
                | ControlTarget::PotInBlackFeedbackStage(_)
                | ControlTarget::WiperDivider { .. } => {
                    if let Some(smoother) =
                        self.pot_smoothers.iter_mut().find(|s| s.control_idx == i)
                    {
                        smoother.current = value;
                        smoother.target = value;
                    }
                    if !self.fan_out_pot_targets(i, value) {
                        let comp_id = self.controls[i].component_id.clone();
                        for stage in &mut self.stages {
                            stage.set_control_pot(&comp_id, value);
                        }
                        for wd in &mut self.wiper_dividers {
                            if wd.pot_comp_id == comp_id {
                                wd.position = wd.taper.apply(value);
                            }
                        }
                        if self.bbd_mix_pot_id.as_deref() == Some(&*comp_id) {
                            self.bbd_wet_mix = value;
                        }
                        self.apply_pot_mirrors(&comp_id, value);
                    }
                }
                _ => self.set_control(label, value),
            }
        }
    }

    /// Handle MIDI note-on by firing all trigger inputs.
    pub fn note_on(&mut self, note: u8, _velocity: u8) {
        #[cfg(all(debug_assertions, feature = "std"))]
        std::eprintln!(
            "[CompiledPedal] note_on({}) → firing ALL {} triggers",
            note,
            self.triggers.len()
        );
        for trig in &mut self.triggers {
            trig.fire();
        }
    }

    /// Fan a single pot position out to EVERY coordinated stage binding.
    ///
    /// This is the multi-target dispatch path: a control whose pot straddles a
    /// stage boundary owns several [`StageBinding`]s (one per stage the pot
    /// touches, plus a wiper-divider at the boundary). Each binding re-stamps
    /// its stage with the correctly-formatted half id (`{id}__aw` / `{id}__wb`
    /// / base id) and the (optionally complemented) position, so a 3-terminal
    /// pot spanning a WDF→MultiNL boundary updates both halves coherently and
    /// the wiper-divider delivers the divided voltage to the downstream load.
    ///
    /// Returns true if any binding was applied. When the control has no
    /// explicit `targets` set (legacy / non-pot bindings) the caller falls
    /// back to the comp-id broadcast.
    fn fan_out_pot_targets(&mut self, ctrl_idx: usize, value: crate::Wave) -> bool {
        if self.controls[ctrl_idx].targets.is_empty() {
            return false;
        }
        let comp_id = self.controls[ctrl_idx].component_id.clone();
        // Clone the small target set to release the borrow on self.controls.
        let targets = self.controls[ctrl_idx].targets.clone();
        for sb in &targets {
            let pos = if sb.complement { 1.0 - value } else { value };
            let leaf_id = match sb.half {
                PotHalf::Whole => comp_id.clone(),
                PotHalf::Aw => format!("{comp_id}__aw"),
                PotHalf::Wb => format!("{comp_id}__wb"),
            };
            match &sb.target {
                ControlTarget::PotInStage(idx)
                | ControlTarget::PotInIirStage(idx)
                | ControlTarget::PotInBlackFeedbackStage(idx) => {
                    if let Some(stage) = self.stages.get_mut(*idx) {
                        stage.set_control_pot(&leaf_id, pos);
                    }
                }
                ControlTarget::PotInMultiNlStage(idx, _) => {
                    if let Some(stage) = self.stages.get_mut(*idx) {
                        stage.set_control_pot(&leaf_id, pos);
                    }
                }
                ControlTarget::WiperDivider { .. } => {
                    for wd in &mut self.wiper_dividers {
                        if wd.pot_comp_id == comp_id {
                            wd.position = wd.taper.apply(pos);
                        }
                    }
                }
                // A coordinated set only ever holds pot/divider targets; any
                // other kind is dispatched via its single-target path.
                _ => {}
            }
        }
        if self.bbd_mix_pot_id.as_deref() == Some(&*comp_id) {
            self.bbd_wet_mix = value;
        }
        self.apply_pot_mirrors(&comp_id, value);
        true
    }

    /// Update mirrored pots (position = 1.0 - source) across all stages.
    fn apply_pot_mirrors(&mut self, comp_id: &str, value: crate::Wave) {
        // Collect mirror info to avoid borrow conflict with self.stages.
        let mirror_info: Option<Vec<(String, String, String)>> =
            self.pot_mirrors.get(comp_id).map(|mirrors| {
                mirrors
                    .iter()
                    .map(|mp| (mp.id.clone(), mp.id_aw.clone(), mp.id_wb.clone()))
                    .collect()
            });
        if let Some(mirrors) = mirror_info {
            let inv = 1.0 - value;
            for (id, _, _) in &mirrors {
                for stage in &mut self.stages {
                    stage.set_control_pot(id, inv);
                }
            }
        }
    }

    /// Advance all pot smoothers and update WDF trees where values have changed.
    ///
    /// Call this once per sample (or per block for efficiency) to smoothly
    /// interpolate pot values toward their targets. This eliminates zipper
    /// noise and clicks when knobs are turned.
    ///
    /// Multi-NL scattering recomputes are throttled to every 32 samples
    /// (~0.7 ms at 48 kHz) to avoid the O(n³) matrix inversion cost on
    /// every sample.  The pot resistance in the DynNode is still updated
    /// per-sample so the passive port impedance tracks smoothly.
    #[inline]
    fn advance_smoothers(&mut self) {
        for si in 0..self.pot_smoothers.len() {
            if self.pot_smoothers[si].is_settled() {
                continue;
            }
            if self.pot_smoothers[si].advance() {
                let ctrl_idx = self.pot_smoothers[si].control_idx;
                let value = self.pot_smoothers[si].current;

                // Multi-target path: fan out to every coordinated stage binding
                // (per-stage half/complement + wiper divider). Falls back to the
                // comp-id broadcast for legacy/non-explicit bindings.
                if !self.fan_out_pot_targets(ctrl_idx, value) {
                    let comp_id = self.controls[ctrl_idx].component_id.clone();

                    // Update all stages — each stage ignores pots it doesn't own.
                    // Complement halves automatically use 1-value via the complement flag.
                    for stage in &mut self.stages {
                        stage.set_control_pot(&comp_id, value);
                    }

                    // Update wiper dividers
                    for wd in &mut self.wiper_dividers {
                        if wd.pot_comp_id == comp_id {
                            wd.position = wd.taper.apply(value);
                        }
                    }

                    if self.bbd_mix_pot_id.as_deref() == Some(&*comp_id) {
                        self.bbd_wet_mix = value;
                    }
                    self.apply_pot_mirrors(&comp_id, value);
                }
            }
        }

        // Per-sample flush for table-based WDF stages (O(1) lookup).
        for stage in &mut self.stages {
            if let Stage::Wdf(wdf) = stage {
                if wdf.has_interp_table() {
                    wdf.flush_recompute();
                }
            }
        }

        // Throttle expensive scattering recomputes to every 32 samples.
        // Pot resistances in DynNodes are updated per-sample above;
        // the O(n³) matrix re-derivation is batched here.
        self.multi_nl_recompute_counter += 1;
        if self.multi_nl_recompute_counter >= 32 {
            self.multi_nl_recompute_counter = 0;
            for stage in &mut self.stages {
                match stage {
                    Stage::MultiNl(mnl) => mnl.flush_recompute(),
                    Stage::Wdf(wdf) => wdf.flush_recompute(),
                    _ => {}
                }
            }
        }
    }

    /// Debug dump: print complete pedal structure for debugging.
    ///
    /// Number of WDF stages (for debug reporting).
    pub fn debug_stage_count(&self) -> usize {
        self.stages.len()
    }
    /// Number of push-pull stages (for debug reporting).
    pub fn debug_push_pull_count(&self) -> usize {
        self.push_pull_stages.len()
    }
    /// Number of multi-NL stages (for debug reporting).
    pub fn debug_multi_nl_count(&self) -> usize {
        self.stages
            .iter()
            .filter(|s| matches!(s, Stage::MultiNl(_)))
            .count()
    }

    /// Shows gain structure, all WDF stages with their trees, and control bindings.
    /// Build a read-only compile-time diagnostics snapshot of every `MultiNl`
    /// stage (MNA scattering matrix, per-NL-port `gm`/companion, adapted port
    /// resistances, extraction coefficients, VS injection) plus a coarse stage
    /// listing. Derived on demand; behavior-preserving.
    ///
    /// `pedal_name`/`source_path` are set by the caller (the std host knows the
    /// source path; the rt crate does not).
    pub fn diag_snapshot(&self, pedal_name: &str, source_path: &str) -> crate::diag::DiagSnapshot {
        let stage_kinds: Vec<String> = self
            .stages
            .iter()
            .map(|stage| {
                match stage {
                    Stage::Wdf(_) => "Wdf",
                    Stage::MultiNl(_) => "MultiNl",
                    Stage::Iir(_) => "Iir",
                    Stage::StateSpace(_) => "StateSpace",
                    Stage::BlackFeedback(_) => "BlackFeedback",
                    Stage::Blockwise(_) => "Blockwise",
                    Stage::KMethod { .. } => "KMethod",
                    Stage::SerialDelayedFeedback(_) => "SerialDelayedFeedback",
                }
                .to_string()
            })
            .collect();

        let multinl: Vec<crate::diag::MnaStageSnapshot> = self
            .stages
            .iter()
            .enumerate()
            .filter_map(|(i, stage)| match stage {
                Stage::MultiNl(mnl) => Some(mnl.mna_snapshot(i)),
                _ => None,
            })
            .collect();

        crate::diag::DiagSnapshot {
            pedal_name: pedal_name.to_string(),
            source_path: source_path.to_string(),
            sample_rate: self.sample_rate,
            stage_count: self.stages.len(),
            stage_kinds,
            multinl,
        }
    }

    pub fn debug_dump(&self) -> String {
        let mut s = String::new();
        s.push_str("═══════════════════════════════════════════════════════════════════════════\n");
        s.push_str("CompiledPedal Debug Dump\n");
        s.push_str(
            "═══════════════════════════════════════════════════════════════════════════\n\n",
        );

        s.push_str(&format!("Sample Rate: {} Hz\n", self.sample_rate));
        s.push_str(&format!("Supply Voltage: {:.1}V\n", self.supply_voltage));
        s.push_str(&format!(
            "Pre-Gain: {:.6} ({:.2} dB)\n",
            self.pre_gain,
            20.0 * crate::math::log10(self.pre_gain)
        ));
        s.push_str(&format!(
            "Gain Range: ({:.6}, {:.6})\n",
            self.gain_range.0, self.gain_range.1
        ));
        s.push_str(&format!("Oversampling: {:?}\n\n", self.oversampling));

        s.push_str(&format!("Stages: {} total\n", self.stages.len()));
        s.push_str("───────────────────────────────────────────────────────────────────────────\n");
        for (i, stage) in self.stages.iter().enumerate() {
            match stage {
                Stage::Wdf(wdf) => {
                    s.push_str(&format!("\n[Stage {} (WDF)]\n", i));
                    s.push_str(&wdf.debug_dump());
                    s.push('\n');
                }
                Stage::MultiNl(mnl) => {
                    s.push_str(&format!("\n[Stage {} (MultiNL)] {}\n", i, mnl.debug_dump()));
                }
                Stage::Iir(_) => {
                    s.push_str(&format!("\n[Stage {} (IIR)]\n", i));
                }
                Stage::StateSpace(_) => {
                    s.push_str(&format!("\n[Stage {} (StateSpace)]\n", i));
                }
                Stage::BlackFeedback(_) => {
                    s.push_str(&format!("\n[Stage {} (BlackFeedback)]\n", i));
                }
                Stage::Blockwise(k) => {
                    s.push_str(&format!(
                        "\n[Stage {} (Blockwise)] {}\n",
                        i,
                        k.debug_label()
                    ));
                }
                Stage::KMethod { .. } => {
                    s.push_str(&format!("\n[Stage {} (KMethod child)]\n", i));
                }
                Stage::SerialDelayedFeedback(serial) => {
                    s.push_str(&format!(
                        "\n[Stage {} (SerialDelayedFeedback)] {} WDF rungs, gain={:+.3}\n",
                        i,
                        serial.stages.len(),
                        serial.feedback_gain
                    ));
                }
            }
        }

        if !self.push_pull_stages.is_empty() {
            s.push_str(&format!(
                "\nPush-Pull Stages: {}\n",
                self.push_pull_stages.len()
            ));
            s.push_str(
                "───────────────────────────────────────────────────────────────────────────\n",
            );
            for (i, pp) in self.push_pull_stages.iter().enumerate() {
                s.push_str(&format!("  [{}] {}\n", i, pp.debug_dump()));
            }
        }

        if !self.delay_lines.is_empty() {
            s.push_str(&format!("\nDelay Lines: {}\n", self.delay_lines.len()));
            s.push_str(
                "───────────────────────────────────────────────────────────────────────────\n",
            );
            for (i, dl) in self.delay_lines.iter().enumerate() {
                s.push_str(&format!(
                    "  [{}] {} - {} taps\n",
                    i,
                    dl.comp_id,
                    dl.taps.len()
                ));
            }
        }

        if !self.controls.is_empty() {
            s.push_str("\nControls:\n");
            s.push_str(
                "───────────────────────────────────────────────────────────────────────────\n",
            );
            for ctrl in &self.controls {
                s.push_str(&format!("  {} -> {:?}\n", ctrl.label, ctrl.target));
            }
        }

        if !self.vcos.is_empty() {
            s.push_str(&format!("\nVCO Bindings: {}\n", self.vcos.len()));
            s.push_str(
                "───────────────────────────────────────────────────────────────────────────\n",
            );
            for (i, vco) in self.vcos.iter().enumerate() {
                s.push_str(&format!(
                    "  [{}] {} - freq={:.1}Hz, waveform={:?}, out_node={}\n",
                    i,
                    vco.comp_id,
                    vco.vco.frequency(),
                    vco.waveform,
                    vco.output_node_id
                ));
            }
        }

        if !self.vcas.is_empty() {
            s.push_str(&format!("\nVCA Bindings: {}\n", self.vcas.len()));
            s.push_str(
                "───────────────────────────────────────────────────────────────────────────\n",
            );
            for (i, vca) in self.vcas.iter().enumerate() {
                let trig = match vca.trigger_idx {
                    Some(idx) => format!("trigger {idx}"),
                    None => "none".to_string(),
                };
                s.push_str(&format!(
                    "  [{}] {} - in_node={}, out_node={}, trigger={}\n",
                    i, vca.comp_id, vca.input_node_id, vca.output_node_id, trig
                ));
            }
        }

        if !self.triggers.is_empty() {
            s.push_str(&format!("\nTriggers: {}\n", self.triggers.len()));
            s.push_str(
                "───────────────────────────────────────────────────────────────────────────\n",
            );
            for (i, trig) in self.triggers.iter().enumerate() {
                let target = match trig.target_stage {
                    Some(idx) => format!("stage {idx}"),
                    None => "global".to_string(),
                };
                let inj = if trig.injection_node != usize::MAX {
                    format!(", inj_node={}", trig.injection_node)
                } else {
                    String::new()
                };
                s.push_str(&format!(
                    "  [{}] amp={:.1}V, target={}{}\n",
                    i, trig.amplitude, target, inj
                ));
            }
        }

        s.push_str(&format!("\nStages: {} total\n", self.stages.len()));

        s
    }

    /// Machine-readable structured dump of the compiled stage structure.
    ///
    /// This is the structured counterpart to [`debug_dump`](Self::debug_dump):
    /// instead of prose it emits a JSON `StageGraph` so tooling (the tracing
    /// IDE, structure diagnostics) can reason about the compiled pipeline.
    ///
    /// Schema:
    /// ```json
    /// {
    ///   "schema": "pedalkernel.stage_graph/1",
    ///   "sample_rate": 48000.0,
    ///   "supply_voltage": 12.0,
    ///   "stage_count": 3,
    ///   "nodes": [
    ///     { "id": 0, "kind": "MultiNl", "label": "...",
    ///       "flow_order": 1, "feedback_ports": 0, "nl_root_count": 3,
    ///       "nl_port_count": 6, "bypass_serial": false }
    ///   ],
    ///   "edges": [
    ///     { "from": 0, "to": 1, "kind": "Flow" }
    ///   ]
    /// }
    /// ```
    ///
    /// Edges (Phase 1b) are inter-stage `Flow` edges derived from the
    /// flow-distance ordering of audio-path (non-`bypass_serial`) stages.
    pub fn stage_graph_json(&self) -> alloc::string::String {
        use alloc::string::String;

        // ── nodes ────────────────────────────────────────────────────────
        let mut nodes = String::new();
        for (i, stage) in self.stages.iter().enumerate() {
            if i > 0 {
                nodes.push(',');
            }
            let flow = match stage.flow_order() {
                Some(d) => format!("{d}"),
                None => String::from("null"),
            };
            nodes.push_str(&format!(
                "\n    {{\"id\": {}, \"kind\": \"{}\", \"label\": \"{}\", \
                 \"flow_order\": {}, \"feedback_ports\": {}, \"nl_root_count\": {}, \
                 \"nl_port_count\": {}, \"bypass_serial\": {}}}",
                i,
                stage.graph_kind(),
                json_escape(&stage.graph_label()),
                flow,
                stage.feedback_port_count(),
                stage.nl_root_count(),
                stage.nl_port_count(),
                stage.bypass_serial(),
            ));
        }

        // ── edges (Phase 1b: inter-stage Flow from flow-distance order) ────
        //
        // Order the audio-path stages (skip bypass_serial bias networks and
        // KMethod children, which have no flow distance) by flow_order, then
        // connect consecutive stages. Ties keep compiled (vector) order.
        let mut audio: alloc::vec::Vec<(usize, usize)> = alloc::vec::Vec::new();
        for (i, stage) in self.stages.iter().enumerate() {
            if stage.bypass_serial() {
                continue;
            }
            if let Some(d) = stage.flow_order() {
                audio.push((d, i));
            }
        }
        audio.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.cmp(&b.1)));

        let mut edges = String::new();
        for w in audio.windows(2) {
            if !edges.is_empty() {
                edges.push(',');
            }
            edges.push_str(&format!(
                "\n    {{\"from\": {}, \"to\": {}, \"kind\": \"Flow\"}}",
                w[0].1, w[1].1
            ));
        }

        format!(
            "{{\n  \"schema\": \"pedalkernel.stage_graph/1\",\n  \
             \"sample_rate\": {},\n  \"supply_voltage\": {},\n  \
             \"stage_count\": {},\n  \"nodes\": [{}\n  ],\n  \
             \"edges\": [{}\n  ]\n}}\n",
            self.sample_rate,
            self.supply_voltage,
            self.stages.len(),
            nodes,
            edges,
        )
    }
}

/// Escape a string for embedding in a JSON string literal.
fn json_escape(input: &str) -> alloc::string::String {
    let mut out = alloc::string::String::with_capacity(input.len());
    for c in input.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => {
                out.push_str(&format!("\\u{:04x}", c as u32));
            }
            c => out.push(c),
        }
    }
    out
}

impl CompiledPedal {
    /// Process one sample through the subcircuit routing graph.
    ///
    /// Called when `subcircuit_processors` is non-empty. Iterates the
    /// topologically-sorted routing steps, resolving signal sources and
    /// forwarding samples to each subcircuit's inner processor.
    fn process_subcircuits(&mut self, input: crate::Wave) -> crate::Wave {
        // Reset per-sample output buffer
        for v in &mut self.subcircuit_outputs {
            *v = 0.0;
        }

        // Walk routing steps in topological order (stored as indices to avoid clone)
        for step_idx in 0..self.subcircuit_routing.len() {
            let (sc_idx, sig) = match &self.subcircuit_routing[step_idx] {
                crate::subcircuit::RoutingStep::ProcessSubcircuit {
                    subcircuit_idx,
                    input_source,
                } => {
                    let idx = *subcircuit_idx;
                    let sig =
                        Self::resolve_signal_static(input, input_source, &self.subcircuit_outputs);
                    (idx, sig)
                }
            };
            let output = self.subcircuit_processors[sc_idx].process(sig);
            self.subcircuit_outputs[sc_idx] = output;
        }

        // Return the designated output subcircuit's signal
        if let Some(out_idx) = self.subcircuit_output_idx {
            self.subcircuit_outputs.get(out_idx).copied().unwrap_or(0.0)
        } else {
            0.0
        }
    }

    /// Resolve a `SignalSource` to a concrete `crate::Wave` sample value.
    ///
    /// Static version (no `&self`) so it can be called while `subcircuit_processors`
    /// is borrowed mutably.
    fn resolve_signal_static(
        input: crate::Wave,
        source: &crate::subcircuit::SignalSource,
        outputs: &[crate::Wave],
    ) -> crate::Wave {
        use crate::subcircuit::SignalSource;
        match source {
            SignalSource::EquipmentInput => input,
            SignalSource::SubcircuitOutput(idx) => outputs.get(*idx).copied().unwrap_or(0.0),
            SignalSource::Sum(sources) => sources
                .iter()
                .map(|s| Self::resolve_signal_static(input, s, outputs))
                .sum(),
        }
    }

    /// Route one envelope follower's output to its modulation target.
    ///
    /// Takes disjoint field borrows (not `&mut self`) so it can be called
    /// from inside `process()`'s serial stage loop while the envelope
    /// binding itself is mutably borrowed. `env_out` is the raw follower
    /// output (used by `DelaySpeed`, which scales it by `range` without the
    /// bias); `modulation` is `bias + env_out * range`.
    #[allow(clippy::too_many_arguments)]
    fn route_envelope_modulation(
        stages: &mut [Stage],
        bbds: &mut [BbdDelayLine],
        opamp_stages: &mut [OpAmpStage],
        delay_lines: &mut [DelayLineBinding],
        vcas: &mut [VcaBinding],
        springs: &mut [SpringBinding],
        target: &ModulationTarget,
        modulation: crate::Wave,
        env_out: crate::Wave,
        range: crate::Wave,
    ) {
        match target {
            ModulationTarget::JfetVgs { stage_idx } => {
                if let Some(Stage::Wdf(wdf)) = stages.get_mut(*stage_idx) {
                    wdf.set_jfet_vgs(modulation);
                }
            }
            ModulationTarget::AllJfetVgs => {
                for stage in stages.iter_mut() {
                    if let Stage::Wdf(wdf) = stage {
                        if matches!(&wdf.root, RootKind::Jfet(_) | RootKind::JfetVr(_)) {
                            wdf.set_jfet_vgs(modulation);
                        }
                    }
                }
            }
            ModulationTarget::JfetVrVgs { stage_idx, comp_id } => {
                if let Some(Stage::Wdf(wdf)) = stages.get_mut(*stage_idx) {
                    wdf.set_jfet_vr_vgs(comp_id, modulation);
                }
            }
            ModulationTarget::PhotocouplerLed { stage_idx, comp_id } => {
                if let Some(Stage::Wdf(wdf)) = stages.get_mut(*stage_idx) {
                    let led = modulation.clamp(0.0, 1.0);
                    // Leaf-positioned XOR input-path (audit gap G3): a
                    // photocoupler that compiled as a leaf (its netlist
                    // position) must NOT also be driven as an op-amp
                    // input-path series gain — that path inverts the GR
                    // direction for shunt CdS cells.
                    if !wdf.set_photocoupler_led(comp_id, led) {
                        wdf.set_input_photocoupler_led(comp_id, led);
                    }
                }
            }
            ModulationTarget::TriodeVgk { stage_idx } => {
                if let Some(Stage::Wdf(wdf)) = stages.get_mut(*stage_idx) {
                    wdf.set_triode_vgk(modulation);
                }
            }
            ModulationTarget::PentodeVg1k { stage_idx } => {
                if let Some(Stage::Wdf(wdf)) = stages.get_mut(*stage_idx) {
                    wdf.set_pentode_vg1k(modulation);
                }
            }
            ModulationTarget::VariMuVgk { stage_idx } => {
                if let Some(Stage::Wdf(wdf)) = stages.get_mut(*stage_idx) {
                    wdf.set_vari_mu_vgk(modulation);
                }
            }
            ModulationTarget::MosfetVgs { stage_idx } => {
                if let Some(Stage::Wdf(wdf)) = stages.get_mut(*stage_idx) {
                    wdf.set_mosfet_vgs(modulation);
                }
            }
            ModulationTarget::OtaIabc { stage_idx } => {
                if let Some(Stage::Wdf(wdf)) = stages.get_mut(*stage_idx) {
                    let gain = (1.0 - modulation).clamp(0.0, 1.0);
                    wdf.set_ota_gain(gain);
                }
            }
            ModulationTarget::OtaIabcLinear { multi_nl_idx } => {
                if let Some(Stage::MultiNl(mnl)) = stages.get_mut(*multi_nl_idx) {
                    let gain = (1.0 - modulation).clamp(0.0, 1.0);
                    mnl.set_ota_gain_linear(gain);
                }
            }
            ModulationTarget::BbdClock { bbd_idx } => {
                if let Some(bbd) = bbds.get_mut(*bbd_idx) {
                    bbd.set_delay_normalized(modulation.clamp(0.0, 1.0));
                }
            }
            ModulationTarget::VcaCv { vca_idx } => {
                if let Some(vca_binding) = vcas.get_mut(*vca_idx) {
                    // Detector silence (modulation 0) = unity gain; louder
                    // program = more attenuation, per the SSM2164-class
                    // dB-linear law in Vca::set_cv_normalized.
                    vca_binding
                        .vca
                        .set_cv_normalized(modulation.clamp(0.0, 1.0));
                }
            }
            ModulationTarget::OpAmpVp { opamp_idx } => {
                if let Some(opamp_stage) = opamp_stages.get_mut(*opamp_idx) {
                    opamp_stage.opamp.set_vp(modulation);
                }
            }
            ModulationTarget::DelaySpeed { delay_idx } => {
                if let Some(dl) = delay_lines.get_mut(*delay_idx) {
                    dl.delay_line.add_speed_mod(env_out * range);
                }
            }
            ModulationTarget::DelayTime { delay_idx } => {
                if let Some(dl) = delay_lines.get_mut(*delay_idx) {
                    dl.delay_line
                        .set_delay_normalized(modulation.clamp(0.0, 1.0));
                }
            }
            ModulationTarget::SpringDwell { spring_idx } => {
                if let Some(s) = springs.get_mut(*spring_idx) {
                    s.tank.set_dwell_normalized(modulation.clamp(0.0, 1.0));
                }
            }
        }
    }
}

impl PedalProcessor for CompiledPedal {
    fn process(&mut self, input: Wave) -> Wave {
        let input = input as crate::Wave;
        // Auto-init on first call.
        if !self.initialized {
            self.cache_all_vs_pointers();
            self.initialized = true;
        }

        // Write input to the first input port value. This ensures BKM stages
        // (which read from port_values) see the serial chain signal.
        // process_ports() writes all port values before calling process(),
        // so this only matters for the process(input) path.
        if let Some(first_in) = self
            .ports
            .iter()
            .find(|p| p.direction == crate::PortDirection::Input)
        {
            if first_in.index < self.port_values.len() {
                self.port_values[first_in.index] = input;
            }
        }

        // Subcircuit routing mode: process inter-subcircuit signal graph.
        // When subcircuits are present, the normal WDF stage pipeline is bypassed.
        if !self.subcircuit_processors.is_empty() {
            return self.process_subcircuits(input) as Wave;
        }

        // Fire any pending triggers.
        // Triggers with explicit injection nodes route through node_signals
        // for per-voice WDF stage routing. Triggers without (injection_node == MAX)
        // sum into a global impulse that replaces the input signal.
        self.node_signals.clear();
        let mut global_trigger: crate::Wave = 0.0;
        for trigger in &mut self.triggers {
            let impulse = trigger.tick();
            if impulse != 0.0 {
                if trigger.injection_node != usize::MAX {
                    self.node_signals.push((trigger.injection_node, impulse));
                } else {
                    global_trigger += impulse;
                }
            }
        }
        let input = if global_trigger != 0.0 {
            global_trigger
        } else {
            input
        };

        // Inject input port voltages into node_signals.
        // Input ports are voltage sources at circuit nodes — set by external
        // code via set_port() before this process() call. Audio rate, no
        // impedance recompute. Stages with matching injection_node_id will
        // read these values during their processing.
        for port in &self.ports {
            if port.direction == crate::PortDirection::Input && port.node_id != usize::MAX {
                self.node_signals
                    .push((port.node_id, self.port_values[port.index]));
            }
        }

        // INTERNAL delayed ports (Phase 2b): the z⁻¹ READ. Inject each carried
        // value (last sample's solved value at `source_node_id`) at its consumer
        // node, scaled by the superposition/carry gain. Because the value is from
        // LAST sample, the consuming sub-network has no intra-sample ordering
        // dependency on its driver — this is the cross-sample (cv_delayed-style)
        // persistence that lets the de-fused detector solve as its own network.
        for ip in &self.internal_ports {
            if ip.consumer_node_id != usize::MAX {
                self.node_signals
                    .push((ip.consumer_node_id, ip.prev_value * ip.gain));
            }
        }

        // Phase 3 — CLOSE THE GR LOOP. Drive the photocoupler LED from the
        // detector's solved EL-drive value, carried one sample late (the z⁻¹
        // closure that breaks the otherwise-instantaneous feedback loop). The
        // T4B cell darkens the FORWARD shunt leaf -> downward gain reduction.
        // This runs BEFORE the WDF stages so the cell resistance is current
        // when the forward divider solves this sample.
        if let Some(coupling) = self.detector_led_coupling.clone() {
            if let Some(ip) = self.internal_ports.get(coupling.carry_idx) {
                // Rectify (full-wave) + normalize the carried EL-drive node
                // value to a 0..1 LED drive. The T4B set_led_drive applies the
                // physical attack/release of the EL panel + CdS cell.
                let led = (ip.prev_value.abs() * coupling.scale).clamp(0.0, 1.0);
                match self.stages.get_mut(coupling.stage_idx) {
                    // Shunt CdS cell that compiled as a WDF leaf.
                    Some(Stage::Wdf(wdf)) => {
                        // Leaf-positioned (the shunt cell) — same XOR fallback
                        // as route_envelope_modulation; the LED never drives an
                        // input-path series gain for a shunt CdS divider.
                        if !wdf.set_photocoupler_led(&coupling.comp_id, led) {
                            wdf.set_input_photocoupler_led(&coupling.comp_id, led);
                        }
                    }
                    // Shunt CdS cell fused into the MNA stage with the makeup
                    // tube V1 (the LA-2A faithful case: stage label
                    // `C_in,...,PC1,...,V1`) — the cell is an MNA G-matrix
                    // conductance, driven via the MultiNlStage setter.
                    Some(Stage::MultiNl(mnl)) => {
                        mnl.set_photocoupler_led(&coupling.comp_id, led);
                    }
                    _ => {}
                }
            }
        }

        // Advance pot smoothers — smoothly interpolate pot values toward targets.
        // This eliminates zipper noise and clicks when knobs are turned.
        self.advance_smoothers();

        // Tick thermal model — temperature changes are very slow (updated
        // every ~1000 samples internally), so the overhead is negligible.
        // The thermal state modulates diode Is and n_vt for each stage.
        if let Some(ref mut thermal) = self.thermal {
            let state = *thermal.tick();
            for stage in &mut self.stages {
                if let Stage::Wdf(wdf) = stage {
                    wdf.apply_thermal(&state);
                }
            }
        }

        // Tick all LFOs and route their outputs to targets.
        for binding in &mut self.lfos {
            let lfo_out = binding.lfo.tick();
            let modulation = binding.bias + lfo_out * binding.range;

            match &binding.target {
                ModulationTarget::JfetVgs { stage_idx } => {
                    if let Some(Stage::Wdf(wdf)) = self.stages.get_mut(*stage_idx) {
                        wdf.set_jfet_vgs(modulation);
                    }
                }
                ModulationTarget::AllJfetVgs => {
                    // Modulate ALL JFET stages with the same value (for phasers)
                    for stage in &mut self.stages {
                        if let Stage::Wdf(wdf) = stage {
                            if matches!(&wdf.root, RootKind::Jfet(_) | RootKind::JfetVr(_)) {
                                wdf.set_jfet_vgs(modulation);
                            }
                        }
                    }
                }
                ModulationTarget::JfetVrVgs { stage_idx, comp_id } => {
                    if let Some(Stage::Wdf(wdf)) = self.stages.get_mut(*stage_idx) {
                        wdf.set_jfet_vr_vgs(comp_id, modulation);
                    }
                }
                ModulationTarget::PhotocouplerLed { stage_idx, comp_id } => {
                    if let Some(Stage::Wdf(wdf)) = self.stages.get_mut(*stage_idx) {
                        let led = modulation.clamp(0.0, 1.0);
                        // Leaf-positioned XOR input-path (audit gap G3) —
                        // see route_envelope_modulation.
                        if !wdf.set_photocoupler_led(comp_id, led) {
                            wdf.set_input_photocoupler_led(comp_id, led);
                        }
                    }
                }
                ModulationTarget::TriodeVgk { stage_idx } => {
                    if let Some(Stage::Wdf(wdf)) = self.stages.get_mut(*stage_idx) {
                        wdf.set_triode_vgk(modulation);
                    }
                }
                ModulationTarget::PentodeVg1k { stage_idx } => {
                    if let Some(Stage::Wdf(wdf)) = self.stages.get_mut(*stage_idx) {
                        wdf.set_pentode_vg1k(modulation);
                    }
                }
                ModulationTarget::VariMuVgk { stage_idx } => {
                    if let Some(Stage::Wdf(wdf)) = self.stages.get_mut(*stage_idx) {
                        wdf.set_vari_mu_vgk(modulation);
                    }
                }
                ModulationTarget::MosfetVgs { stage_idx } => {
                    if let Some(Stage::Wdf(wdf)) = self.stages.get_mut(*stage_idx) {
                        wdf.set_mosfet_vgs(modulation);
                    }
                }
                ModulationTarget::OtaIabc { stage_idx } => {
                    if let Some(Stage::Wdf(wdf)) = self.stages.get_mut(*stage_idx) {
                        wdf.set_ota_gain(modulation.clamp(0.0, 1.0));
                    }
                }
                ModulationTarget::OtaIabcLinear { multi_nl_idx } => {
                    if let Some(Stage::MultiNl(mnl)) = self.stages.get_mut(*multi_nl_idx) {
                        mnl.set_ota_gain_linear(modulation.clamp(0.0, 1.0));
                    }
                }
                ModulationTarget::BbdClock { bbd_idx } => {
                    if let Some(bbd) = self.bbds.get_mut(*bbd_idx) {
                        // LFO modulates delay time: modulation = normalized 0-1
                        bbd.set_delay_normalized(modulation.clamp(0.0, 1.0));
                    }
                }
                ModulationTarget::VcaCv { vca_idx } => {
                    if let Some(vca_binding) = self.vcas.get_mut(*vca_idx) {
                        // LFO-driven VCA CV (tremolo): normalized 0..1 onto
                        // the SSM2164-class dB-linear law.
                        vca_binding
                            .vca
                            .set_cv_normalized(modulation.clamp(0.0, 1.0));
                    }
                }
                ModulationTarget::OpAmpVp { opamp_idx } => {
                    if let Some(opamp_stage) = self.opamp_stages.get_mut(*opamp_idx) {
                        // LFO modulates op-amp's non-inverting input
                        opamp_stage.opamp.set_vp(modulation);
                    }
                }
                ModulationTarget::DelaySpeed { delay_idx } => {
                    if let Some(dl) = self.delay_lines.get_mut(*delay_idx) {
                        // LFO modulates tape speed: modulation is additive offset
                        // from 1.0 (center). Typical wow: ±0.02, flutter: ±0.005.
                        dl.delay_line.add_speed_mod(lfo_out * binding.range);
                    }
                }
                ModulationTarget::DelayTime { delay_idx } => {
                    if let Some(dl) = self.delay_lines.get_mut(*delay_idx) {
                        // LFO modulates delay time (normalized 0–1).
                        dl.delay_line
                            .set_delay_normalized(modulation.clamp(0.0, 1.0));
                    }
                }
                ModulationTarget::SpringDwell { spring_idx } => {
                    if let Some(s) = self.springs.get_mut(*spring_idx) {
                        s.tank.set_dwell_normalized(modulation.clamp(0.0, 1.0));
                    }
                }
            }
        }

        // Tick envelope followers and route their outputs to targets.
        // GlobalInput-tapped detectors read the raw pedal input here, before
        // any stage runs. Stage-tapped detectors (EnvelopeTapSource::
        // StageOutput) tick inside the serial stage loop instead, fed by
        // their tap stage's fresh output — except when a stage route plan
        // bypasses the serial loop, in which case they fall back to the
        // global input (the pre-tap legacy behavior).
        let route_plan_active = self.stage_route_plan.primary_bkm.is_some();
        for binding in &mut self.envelopes {
            match binding.tap {
                EnvelopeTapSource::GlobalInput => {}
                EnvelopeTapSource::StageOutput(_) if route_plan_active => {}
                EnvelopeTapSource::StageOutput(_) => continue,
            }
            let env_out = binding.envelope.process(input);
            let modulation = binding.bias + env_out * binding.range;
            Self::route_envelope_modulation(
                &mut self.stages,
                &mut self.bbds,
                &mut self.opamp_stages,
                &mut self.delay_lines,
                &mut self.vcas,
                &mut self.springs,
                &binding.target,
                modulation,
                env_out,
                binding.range,
            );
        }

        // Tick VCOs — generate audio-rate waveforms and inject into node_signals.
        // This happens before WDF stages so the VCO audio is available as input.
        for vco_binding in &mut self.vcos {
            // Audio-rate CV-port path (spec §3): read the 1 V/oct pitch CV node
            // voltage from node_signals (set by input-port injection above or an
            // upstream stage) and map it to frequency. This is a float update to
            // phase_inc — NO scattering recompute. When no CV pin is wired the
            // VCO holds its declared base frequency.
            if vco_binding.cv_node_id != usize::MAX {
                let cv: crate::Wave = self
                    .node_signals
                    .iter()
                    .rev()
                    .filter(|(nid, _)| *nid == vco_binding.cv_node_id)
                    .map(|(_, v)| *v)
                    .sum();
                vco_binding.vco.set_cv_pitch(cv);
            }
            let (saw, tri, pulse) = vco_binding.vco.tick();
            let sample = match vco_binding.waveform {
                crate::elements::VcoWaveform::Saw => saw,
                crate::elements::VcoWaveform::Triangle => tri,
                crate::elements::VcoWaveform::Pulse => pulse,
            };
            self.node_signals.push((vco_binding.output_node_id, sample));
        }

        // TODO: DebugStats not yet in rt crate
        // #[cfg(debug_assertions)]
        // if let Some(ref stats) = self.debug_stats {
        //     stats.record_input(input);
        // }

        // Power supply sag: compute instantaneous current draw from signal
        // level and update the supply voltage.  The power supply model tracks
        // the filter cap charge/discharge dynamics, producing B+ droop under
        // load that recovers at a rate set by the filter capacitance.
        if let Some(ref mut psu) = self.power_supply {
            // Estimate current draw from the signal amplitude.
            // In a tube amp, plate current ≈ signal voltage / plate load resistance.
            // We use a nominal 100kΩ load impedance for the normalized estimate.
            let current_draw = input.abs() * self.pre_gain / 100_000.0;
            let sagged_voltage = psu.tick(current_draw);
            // Propagate the sagged voltage to all voltage-dependent elements.
            // This is the same mechanism as set_supply_voltage() but avoids
            // the clamp so we can track the dynamic sag precisely.
            let prev_supply_voltage = self.supply_voltage;
            self.supply_voltage = sagged_voltage;
            // Update v_max for all root elements
            let saturation_margin = 1.5;
            let abs_sagged = sagged_voltage.abs();
            let opamp_v_max = (abs_sagged / 2.0 - saturation_margin).max(1.0);
            let tube_v_max = abs_sagged;
            for stage in &mut self.opamp_stages {
                stage.opamp.set_v_max(opamp_v_max);
            }
            for stage in &mut self.stages {
                match stage {
                    Stage::Wdf(wdf) => match &mut wdf.root {
                        RootKind::OpAmp(op) => op.set_v_max(opamp_v_max),
                        RootKind::Triode(t) => t.set_v_max(tube_v_max),
                        RootKind::Pentode(p) => p.set_v_max(tube_v_max),
                        _ => {}
                    },
                    Stage::MultiNl(mnl) => {
                        mnl.update_supply_voltage(sagged_voltage);
                    }
                    _ => {}
                }
            }
            for pp in &mut self.push_pull_stages {
                pp.push_root.set_v_max(tube_v_max);
                pp.pull_root.set_v_max(tube_v_max);
            }
            // Update single-NL WDF stage VCC bias for supply sag.
            if prev_supply_voltage.abs() > 0.0 {
                let scale = sagged_voltage / prev_supply_voltage;
                for stage in &mut self.stages {
                    if let Stage::Wdf(wdf) = stage {
                        if wdf.vcc_injection_coeff != 0.0 {
                            wdf.vcc_injection_coeff *= scale;
                        }
                    }
                }
            }
        }

        // Headroom models the supply rail ceiling.  In real circuits, gain
        // is set by resistor ratios (feedback/input) and is independent of
        // supply voltage.  What changes with voltage is how far the active
        // element can swing before hitting the rail — i.e. the clipping
        // ceiling.  So headroom only scales the soft limiter, not pre_gain
        // or output_gain.
        let headroom = self.supply_voltage.abs() / 9.0;

        // Apply pre-gain ONCE at the input.  This models the initial active
        // gain element (op-amp or transistor) that drives signal into the
        // first processing stage.
        let mut signal = input * self.pre_gain;

        #[cfg(feature = "debug-trace")]
        let trace_on = if input.abs() > 1e-10 {
            let n = TRACE_COUNT_PIPELINE.fetch_add(1, Ordering::Relaxed);
            if n < MAX_TRACE_PIPELINE {
                std::eprintln!(
                    "[PIPE n={n}] input={input:.6e} pre_gain={:.6} signal={signal:.6e} supply={:.1}V stages={} pp={}",
                    self.pre_gain, self.supply_voltage, self.stages.len(), self.push_pull_stages.len()
                );
                true
            } else {
                false
            }
        } else {
            false
        };

        // Apply input loading from upstream source impedance.
        // Models the voltage divider and frequency-dependent rolloff created by
        // the source impedance (guitar pickup, upstream pedal) driving this
        // circuit's input impedance.
        if let Some(ref mut loading) = self.input_loading {
            signal = loading.process(signal);
        }

        if let Some(output) = self.process_stage_route_plan() {
            return output as Wave;
        }

        // Process all stages in topological (signal-flow) order.
        // WDF, multi-NL, and coupled BJT stages are interleaved by their
        // signal_flow_distance so earlier stages process first regardless of type.
        // WDF clipping stages get inter-stage re-amplification.
        //
        // Multi-NL stages use node-based routing: each stage reads from its
        // injection_node_id and writes to its output_node_id. This correctly
        // handles parallel-path topologies (e.g., 670 dual-channel sidechain)
        // where two stages at the same level should receive the same input,
        // not chain serially.
        let mut prev_was_clipping = false;
        let num_stages = self.stages.len();
        let mut stage_levels = [0.0 as crate::Wave; crate::metering::MAX_STAGES];
        // Phase-B diag: reduced metrics stashed at block-complete, consumed after
        // the metering accumulator borrow ends to build the runtime-ring frame.
        #[cfg(feature = "diag")]
        let mut diag_block_metrics: Option<UiMetrics> = None;
        let mut stage_solver_iterations = [0u32; crate::metering::MAX_STAGES];
        let mut stage_solver_residual = [0.0 as crate::Wave; crate::metering::MAX_STAGES];
        let mut stage_solver_converged = [true; crate::metering::MAX_STAGES];
        let mut stage_solver_active = [false; crate::metering::MAX_STAGES];
        let mut wdf_stage_counter = 0usize;

        // Node routing: trigger impulses were already pushed to node_signals above.

        for stage_idx in 0..self.stages.len() {
            // Determine variant type to dispatch correctly.
            let is_wdf = matches!(&self.stages[stage_idx], Stage::Wdf(_));
            let is_mnl = matches!(&self.stages[stage_idx], Stage::MultiNl(_));
            let is_iir = matches!(&self.stages[stage_idx], Stage::Iir(_));
            let is_ss = matches!(&self.stages[stage_idx], Stage::StateSpace(_));
            let is_bf = matches!(&self.stages[stage_idx], Stage::BlackFeedback(_));
            let is_blockwise = matches!(&self.stages[stage_idx], Stage::Blockwise(_));
            let is_serial_fb = matches!(&self.stages[stage_idx], Stage::SerialDelayedFeedback(_));

            // Check bypass_serial: static bias networks process for metering
            // but don't overwrite the serial audio signal.
            let bypass_serial = self.stages[stage_idx].bypass_serial();

            if bypass_serial {
                // Process for metering but don't touch `signal`
                match &mut self.stages[stage_idx] {
                    Stage::Wdf(w) => {
                        let _ = w.process(0.0);
                    }
                    Stage::MultiNl(_) => {
                        // Control/bias nonlinear groups can be split out of
                        // the audio path. Do not run them as independent
                        // audio-rate solvers; their derived effect is owned by
                        // the target stage (for example blockwise cutoff current).
                    }
                    Stage::Iir(i) => {
                        let _ = i.process(0.0);
                    }
                    Stage::StateSpace(s) => {
                        let _ = s.process(0.0);
                    }
                    Stage::BlackFeedback(b) => {
                        let _ = b.process(0.0);
                    }
                    Stage::Blockwise(k) => {
                        let _ = k.process(&[]);
                    }
                    Stage::KMethod { .. } => {}
                    Stage::SerialDelayedFeedback(s) => {
                        let _ = s.process(0.0);
                    }
                }
                continue;
            }

            // This stage's own output sample, fed to envelope followers whose
            // detector taps resolve to this stage (EnvelopeTapSource::
            // StageOutput). Mirrors `signal` except for feedforward WDF
            // stages, where `signal` is the additive blend.
            let mut stage_tap_output: crate::Wave = 0.0;

            // Parallel-branch convergence summation (F13b): this WDF stage is a
            // pure resistive mixer driven by >=2 upstream op-amp outputs. It does
            // NOT run the WDF solver — it reads each driver's voltage from
            // node_signals and sums by the precomputed superposition gains
            // (O(N) multiply-adds, no per-sample matrix work).
            if is_wdf {
                let is_convergence = matches!(
                    &self.stages[stage_idx],
                    Stage::Wdf(w) if w.convergence.is_some()
                );
                if is_convergence {
                    let (out_value, out_node) = {
                        let cs = match &self.stages[stage_idx] {
                            Stage::Wdf(w) => w.convergence.as_ref().unwrap(),
                            _ => unreachable!(),
                        };
                        let node_signals = &self.node_signals;
                        let value = cs.evaluate(|node_id| {
                            node_signals
                                .iter()
                                .rev()
                                .filter(|(nid, _)| *nid == node_id)
                                .map(|(_, v)| *v)
                                .sum()
                        });
                        (value, cs.output_node_id)
                    };
                    let out_value = if out_value.is_finite() {
                        out_value
                    } else {
                        0.0
                    };
                    if out_node != usize::MAX {
                        self.node_signals.push((out_node, out_value));
                    }
                    signal = out_value;
                    if stage_idx < crate::metering::MAX_STAGES {
                        stage_levels[stage_idx] = out_value;
                    }
                    wdf_stage_counter += 1;
                    continue;
                }

                let stage = if let Stage::Wdf(w) = &mut self.stages[stage_idx] {
                    w
                } else {
                    unreachable!()
                };

                // Inject named port voltages into this stage's WDF tree.
                // Only ports whose stage_idx matches this stage are applied.
                // Uses cached VS pointers for zero-cost writes (no tree walk).
                for port in &self.ports {
                    if port.direction == crate::PortDirection::Input && port.stage_idx == stage_idx
                    {
                        stage.set_port_voltage(&port.name, self.port_values[port.index]);
                    }
                }

                // Node-based routing for per-voice trigger stages:
                // Only trigger voice stages read exclusively from node_signals.
                // Regular stages (triodes, BJTs, etc.) use serial chain even
                // if they have injection_node_id set for other purposes.
                let stage_input = if stage.is_trigger_voice {
                    let inj_node = stage.injection_node_id;
                    let impulse: crate::Wave = self
                        .node_signals
                        .iter()
                        .rev()
                        .filter(|(nid, _)| *nid == inj_node)
                        .map(|(_, v)| *v)
                        .sum();
                    // Activate voice on first trigger impulse.
                    if impulse != 0.0 {
                        stage.voice_active = true;
                    }
                    // Skip processing entirely until first trigger fires.
                    // This prevents unfired voices from contributing VCC
                    // bias to the output sum.
                    if !stage.voice_active {
                        wdf_stage_counter += 1;
                        continue;
                    }
                    impulse
                } else if stage.is_feedforward {
                    // Feedforward stages read from upstream node_signals.
                    // Fall back to serial `signal` if no upstream stage
                    // wrote to this node (e.g. unity follower buffers).
                    let inj_node = stage.injection_node_id;
                    self.node_signals
                        .iter()
                        .rev()
                        .find(|(nid, _)| *nid == inj_node)
                        .map(|(_, v)| *v)
                        .unwrap_or(signal)
                } else {
                    // Re-amplify only after the *previous* stage clipped.
                    if prev_was_clipping {
                        signal *= self.pre_gain;
                    }
                    signal
                };
                prev_was_clipping = !stage.is_feedforward && stage.root.is_clipping_stage();

                if stage.has_paired_opamp() {
                    stage.set_paired_opamp_vp(stage_input);
                }

                #[cfg(feature = "debug-trace")]
                let pre_stage = stage_input;
                let stage_output = stage.process(stage_input);

                // Guard against NaN from NR solver divergence.
                let stage_output = if stage_output.is_finite() {
                    stage_output
                } else {
                    static NAN_COUNT: core::sync::atomic::AtomicU32 =
                        core::sync::atomic::AtomicU32::new(0);
                    let _n = NAN_COUNT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                    #[cfg(feature = "std")]
                    let n = _n;
                    #[cfg(feature = "std")]
                    if n < 10 || n % 48000 == 0 {
                        std::eprintln!(
                            "NaN/Inf in WDF stage {} ({}): input={:.6e} output={:.6e}",
                            wdf_stage_counter,
                            stage.root_comp_id,
                            stage_input,
                            stage_output,
                        );
                    }
                    0.0
                };
                stage_tap_output = stage_output;

                // Write output to stage's output node for junction summing
                // (only for trigger voice stages).
                if stage.is_feedforward {
                    // Feedforward: additive blend into serial chain
                    signal += stage_output;
                } else if stage.is_trigger_voice && stage.output_node_id != usize::MAX {
                    if let Some(entry) = self
                        .node_signals
                        .iter_mut()
                        .find(|(nid, _)| *nid == stage.output_node_id)
                    {
                        entry.1 += stage_output;
                    } else {
                        self.node_signals.push((stage.output_node_id, stage_output));
                    }
                    // Use accumulated value at output node as the serial chain signal.
                    signal = self
                        .node_signals
                        .iter()
                        .rev()
                        .find(|(nid, _)| *nid == stage.output_node_id)
                        .map(|(_, v)| *v)
                        .unwrap_or(stage_output);
                } else {
                    signal = stage_output;
                    // Write to node_signals for downstream feedforward stages
                    if stage.output_node_id != usize::MAX {
                        self.node_signals.push((stage.output_node_id, stage_output));
                    }
                }

                #[cfg(feature = "debug-trace")]
                if trace_on {
                    std::eprintln!(
                            "  [WDF {wdf_stage_counter}] root={} in={pre_stage:.6e} out={stage_output:.6e} rp={:.1}",
                            stage.root_comp_id,
                            stage.tree.port_resistance()
                        );
                }
                if stage_idx < crate::metering::MAX_STAGES {
                    stage_levels[stage_idx] = stage_output;
                }

                // TODO: DebugStats not yet in rt crate
                // #[cfg(debug_assertions)]
                // if let Some(ref stats) = self.debug_stats {
                //     stats.record_stage_level(wdf_stage_counter, stage_output);
                // }
                wdf_stage_counter += 1;
            } else if is_mnl {
                prev_was_clipping = false;
                let mnl = if let Stage::MultiNl(m) = &mut self.stages[stage_idx] {
                    m
                } else {
                    unreachable!()
                };
                let mnl_input = signal;

                #[cfg(feature = "debug-trace")]
                let pre = mnl_input;
                let mnl_output = mnl.process(mnl_input);
                let mnl_output = if mnl_output.is_finite() {
                    mnl_output
                } else {
                    static NAN_COUNT: core::sync::atomic::AtomicU32 =
                        core::sync::atomic::AtomicU32::new(0);
                    let _n = NAN_COUNT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                    #[cfg(feature = "std")]
                    let n = _n;
                    #[cfg(feature = "std")]
                    if n < 10 || n % 48000 == 0 {
                        let devices: String = if let Some(ref dg) = mnl.device_groups {
                            dg.groups
                                .iter()
                                .map(|g| g.debug_name())
                                .collect::<Vec<_>>()
                                .join(",")
                        } else {
                            mnl.nl_devices
                                .iter()
                                .map(|d| d.debug_name())
                                .collect::<Vec<_>>()
                                .join(",")
                        };
                        std::eprintln!(
                                "NaN/Inf in MultiNL stage {} [{}]: input={:.6e} output={:.6e} v_prev={:.4?}",
                                stage_idx, devices, mnl_input, mnl_output, &mnl.v_prev,
                            );
                    }
                    0.0
                };

                let out_node = mnl.output_node_id;
                self.node_signals.push((out_node, mnl_output));
                signal = mnl_output;
                if stage_idx < crate::metering::MAX_STAGES {
                    stage_levels[stage_idx] = mnl_output;
                }

                #[cfg(feature = "debug-trace")]
                if trace_on {
                    let mnl = if let Stage::MultiNl(m) = &self.stages[stage_idx] {
                        m
                    } else {
                        unreachable!()
                    };
                    let devices: String = if let Some(ref dg) = mnl.device_groups {
                        dg.groups
                            .iter()
                            .map(|g| g.debug_name())
                            .collect::<Vec<_>>()
                            .join(",")
                    } else {
                        mnl.nl_devices
                            .iter()
                            .map(|d| d.debug_name())
                            .collect::<Vec<_>>()
                            .join(",")
                    };
                    std::eprintln!(
                            "  [MNL {stage_idx}] [{devices}] in={pre:.6e} out={signal:.6e} n_nl={} out_port={} xfmr={:.2} inj={} out={}",
                            mnl.n_nl, mnl.output_port, mnl.transformer_gain,
                            mnl.injection_node_id, mnl.output_node_id,
                        );
                    std::eprintln!(
                        "    s_nl_adapted={:.6?} v_prev={:.4?}",
                        &mnl.scattering.s_nl_adapted,
                        &mnl.v_prev,
                    );
                }
            } else if is_iir {
                prev_was_clipping = false;
                let iir_stage = if let Stage::Iir(s) = &mut self.stages[stage_idx] {
                    s
                } else {
                    unreachable!()
                };
                signal = iir_stage.process(signal);
                if stage_idx < crate::metering::MAX_STAGES {
                    stage_levels[stage_idx] = signal;
                }
            } else if is_ss {
                prev_was_clipping = false;
                // F13b: parallel-branch state-space stages read their drive from
                // node_signals[input_node_id] so sibling branches all see the same
                // source rather than cascading down the serial chain.
                let ss_in_node = match &self.stages[stage_idx] {
                    Stage::StateSpace(s) => s.input_node_id,
                    _ => usize::MAX,
                };
                let ss_input = if ss_in_node != usize::MAX {
                    self.node_signals
                        .iter()
                        .rev()
                        .find(|(nid, _)| *nid == ss_in_node)
                        .map(|(_, v)| *v)
                        .unwrap_or(signal)
                } else {
                    signal
                };
                let ss_stage = if let Stage::StateSpace(s) = &mut self.stages[stage_idx] {
                    s
                } else {
                    unreachable!()
                };
                let ss_output = ss_stage.process(ss_input);
                let ss_out_node = ss_stage.output_node_id;
                // Parallel branches publish to the bus but must NOT clobber the
                // serial chain (the convergence stage owns the serial output).
                if ss_out_node != usize::MAX {
                    self.node_signals.push((ss_out_node, ss_output));
                } else {
                    signal = ss_output;
                }
                if stage_idx < crate::metering::MAX_STAGES {
                    stage_levels[stage_idx] = ss_output;
                }
            } else if is_bf {
                prev_was_clipping = false;
                let bf_stage = if let Stage::BlackFeedback(s) = &mut self.stages[stage_idx] {
                    s
                } else {
                    unreachable!()
                };
                signal = bf_stage.process(signal);
                // Write to node_signals for downstream feedforward stages
                if bf_stage.output_node_id != usize::MAX {
                    self.node_signals.push((bf_stage.output_node_id, signal));
                }
                if stage_idx < crate::metering::MAX_STAGES {
                    stage_levels[stage_idx] = signal;
                }
            } else if is_blockwise {
                prev_was_clipping = false;
                let first_input_name = self
                    .ports
                    .iter()
                    .find(|p| p.direction == crate::PortDirection::Input)
                    .map(|p| p.name.clone());
                let refreshed_port_index_cache =
                    if let Stage::Blockwise(stage) = &self.stages[stage_idx] {
                        (stage.port_index_cache.len() != stage.vs_port_map.len()).then(|| {
                            stage
                                .vs_port_map
                                .iter()
                                .map(|(name, _)| {
                                    self.ports
                                        .iter()
                                        .find(|p| &p.name == name)
                                        .map(|p| p.index)
                                        .unwrap_or(usize::MAX)
                                })
                                .collect::<alloc::vec::Vec<_>>()
                        })
                    } else {
                        None
                    };
                let bkm_stage = if let Stage::Blockwise(s) = &mut self.stages[stage_idx] {
                    s
                } else {
                    unreachable!()
                };
                if let Some(cache) = refreshed_port_index_cache {
                    bkm_stage.port_index_cache = cache;
                }
                // Build VS signals from port values.
                let n_vs = bkm_stage.vs_port_map.len();
                let mut vs_signals = alloc::vec![0.0 as crate::Wave; n_vs];
                for (i, port_idx) in bkm_stage.port_index_cache.iter().enumerate() {
                    let port_name = &bkm_stage.vs_port_map[i].0;
                    vs_signals[i] = if first_input_name.as_ref() == Some(port_name) {
                        signal
                    } else if *port_idx < self.port_values.len() {
                        self.port_values[*port_idx]
                    } else {
                        0.0
                    };
                }
                let serial_input = if bkm_stage
                    .vs_port_map
                    .iter()
                    .any(|(name, _)| first_input_name.as_ref() == Some(name))
                {
                    0.0
                } else {
                    signal
                };
                signal = bkm_stage.process_with_serial_input(serial_input, &vs_signals);
                if stage_idx < crate::metering::MAX_STAGES {
                    let diag = bkm_stage.solve_diagnostics();
                    stage_solver_iterations[stage_idx] = diag.iterations;
                    stage_solver_residual[stage_idx] = diag.residual;
                    stage_solver_converged[stage_idx] = diag.converged && !diag.linear_solve_failed;
                    stage_solver_active[stage_idx] = diag.iterations > 0;
                }
                if stage_idx < crate::metering::MAX_STAGES {
                    stage_levels[stage_idx] = signal;
                }
            } else if is_serial_fb {
                prev_was_clipping = false;
                let serial_stage =
                    if let Stage::SerialDelayedFeedback(s) = &mut self.stages[stage_idx] {
                        s
                    } else {
                        unreachable!()
                    };
                signal = serial_stage.process(signal);
                if stage_idx < crate::metering::MAX_STAGES {
                    stage_levels[stage_idx] = signal;
                }
            }

            if !is_wdf {
                // All non-WDF serial stages write their output straight to
                // `signal`, so the tap sample is just the updated chain value.
                stage_tap_output = signal;
            }

            // Apply output wiper dividers after the stage that contains them.
            // The WDF stage still owns the passive filtering/loading behavior;
            // this routing layer only applies the explicit wiper ratio. Using
            // the pre-stage signal here discards any other controls in the same
            // passive group, e.g. RAT Filter + Volume.
            for wd in &self.wiper_dividers {
                if wd.after_stage_idx == stage_idx {
                    signal *= wd.position;
                    // The wiper belongs to this stage, so detector taps on its
                    // output (e.g. an 1176 feedback detector tapping the
                    // output pot wiper) see the post-wiper level too.
                    stage_tap_output *= wd.position;
                }
            }

            // Tick envelope followers whose detector taps this stage's output
            // with the fresh sample. Taps on stages BEFORE their modulated
            // gain element (feed-forward) affect the current sample (the gain
            // stage hasn't run yet); taps at/after it (feedback) take effect
            // next sample — a one-sample-delayed feedback loop, the same
            // convention as SidechainProcessor::cv_delayed.
            for binding in &mut self.envelopes {
                if binding.tap != EnvelopeTapSource::StageOutput(stage_idx) {
                    continue;
                }
                let env_out = binding.envelope.process(stage_tap_output);
                let modulation = binding.bias + env_out * binding.range;
                Self::route_envelope_modulation(
                    &mut self.stages,
                    &mut self.bbds,
                    &mut self.opamp_stages,
                    &mut self.delay_lines,
                    &mut self.vcas,
                    &mut self.springs,
                    &binding.target,
                    modulation,
                    env_out,
                    binding.range,
                );
            }
        }

        #[cfg(feature = "debug-trace")]
        if trace_on {
            let wdf_count = self
                .stages
                .iter()
                .filter(|s| matches!(s, Stage::Wdf(_)))
                .count();
            let mnl_count = self
                .stages
                .iter()
                .filter(|s| matches!(s, Stage::MultiNl(_)))
                .count();
            std::eprintln!(
                "  [PRE-PP] signal={signal:.6e} n_pp={} n_sc={} stages={}(wdf={}, mnl={})",
                self.push_pull_stages.len(),
                self.sidechains.len(),
                self.stages.len(),
                wdf_count,
                mnl_count
            );
        }

        // Late-stage periodic trace: every 4410 samples (0.1s at 44.1kHz)
        #[cfg(feature = "debug-trace")]
        {
            let n = LATE_TRACE_COUNT.fetch_add(1, Ordering::Relaxed);
            if n > 500 && n % 4800 == 0 {
                let pre_pp = signal;
                std::eprintln!(
                    "[LATE n={}] input={:.6e} pre_pp={:.6e} signal_so_far={:.6e}",
                    n,
                    input,
                    pre_pp,
                    signal
                );
            }
        }

        // Tick VCAs — apply amplitude modulation after WDF stages. Two modes:
        // - Trigger-gated (trigger_idx Some): the ADSR owns the gain (synth
        //   voices / drum machines).
        // - CV-driven (trigger_idx None, the vca_lowering compiler pass): the
        //   gain is owned by ModulationTarget::VcaCv routing (envelope
        //   follower or LFO on the cv pin) and must NOT be overwritten by the
        //   idle ADSR (which would force gain 0 = silence).
        for vca_binding in &mut self.vcas {
            // Gate envelope from trigger: only call gate_on() when the trigger
            // fires. Do NOT call gate_off() on subsequent samples — let the ADSR
            // run its full cycle (attack→decay→sustain). For percussive use
            // (sustain=0), the envelope naturally decays to zero.
            if let Some(trig_idx) = vca_binding.trigger_idx {
                if let Some(trig) = self.triggers.get(trig_idx) {
                    if trig.active_this_sample {
                        vca_binding.envelope.gate_on();
                    }
                }
                let env_val = vca_binding.envelope.tick();
                vca_binding.vca.set_gain(env_val);
            }

            // Read audio from input node (sum all signals at that node)
            let vca_input = self
                .node_signals
                .iter()
                .rev()
                .filter(|(nid, _)| *nid == vca_binding.input_node_id)
                .map(|(_, v)| *v)
                .sum::<crate::Wave>();

            // If no node signal found, use the serial chain signal
            let vca_input = if vca_input == 0.0 && vca_binding.input_node_id == usize::MAX {
                signal
            } else {
                vca_input
            };

            let output = vca_binding.vca.process(vca_input);
            self.node_signals.push((vca_binding.output_node_id, output));

            // Feed VCA output into the serial signal chain
            signal = output;
        }

        // If VCOs exist but no VCA or compiled audio stage consumed their output,
        // sum VCO contributions directly into the signal chain. Do not do this
        // for VCO→filter circuits: their stages already read the VCO through
        // named source ports, and summing here creates a dry bypass path.
        if !self.vcos.is_empty() && self.vcas.is_empty() && self.stages.is_empty() {
            for vco_binding in &self.vcos {
                if let Some((_, val)) = self
                    .node_signals
                    .iter()
                    .rev()
                    .find(|(nid, _)| *nid == vco_binding.output_node_id)
                {
                    signal += *val;
                }
            }
        }

        // Process through push-pull stages (differential tube amplifiers).
        // These model circuits like the Fairchild 670 where push and pull
        // triode halves process the signal simultaneously with opposite phase,
        // and the output is the differential plate voltage through a CT transformer.
        //
        // When sidechains are present, the CV from the previous sample modulates
        // the grid bias: Vgrid = base_bias - cv_delayed. This implements
        // variable-mu compression — higher CV = more negative bias = less gain.
        //
        // For stereo equipment (multiple identical push-pull stages), the stages
        // represent parallel channels (e.g., Fairchild 670 Lat/Vert). Only
        // process the first stage to avoid incorrect cascading.
        let pp_active = if self.push_pull_stages.len() > 1
            && self.push_pull_stages.windows(2).all(|w| {
                w[0].turns_ratio == w[1].turns_ratio && w[0].compensation == w[1].compensation
            }) {
            1
        } else {
            self.push_pull_stages.len()
        };

        if !self.sidechains.is_empty() {
            let total_cv: crate::Wave = self.sidechains.iter().map(|sc| sc.cv_delayed).sum();
            for pp_stage in self.push_pull_stages[..pp_active].iter_mut() {
                pp_stage.grid_bias = self.base_grid_bias - total_cv;
                signal = pp_stage.process(signal);
            }
        } else {
            for pp_stage in self.push_pull_stages[..pp_active].iter_mut() {
                signal = pp_stage.process(signal);
            }
        }

        // Process sidechains: tap the audio output and compute CV for next sample.
        // The 1-sample delay is inherent and correct for discrete-time feedback.
        for sc in self.sidechains.iter_mut() {
            let cv = sc.process(signal);
            #[cfg(feature = "debug-trace")]
            if trace_on {
                std::eprintln!(
                    "  [SC] tap={signal:.6e} cv_out={cv:.6e} cv_prev={:.6e}",
                    sc.cv_delayed
                );
            }
            sc.cv_delayed = cv;
        }

        // Process through op-amp stages.
        // Each op-amp is modeled as a VCVS (voltage-controlled voltage source).
        // For unity-gain buffers (most common), the op-amp passes the signal
        // through with high input impedance and low output impedance.
        // The slew rate limiting is built into OpAmpRoot.
        for opamp_stage in &mut self.opamp_stages {
            // Set the input signal as Vp (non-inverting input)
            opamp_stage.opamp.set_vp(signal);

            // WDF formulation: incident wave a comes from the network.
            // For a unity-gain buffer where we want v_out ≈ vp:
            //   a = 2 * vp (the voltage wave from the input)
            //   b = 2 * v_out - a (reflected wave)
            //   v_out = (a + b) / 2
            //
            // By passing a = 2*signal, the solver has the correct reference
            // for the desired output and can properly compute the current
            // needed to drive any load (represented by Rp).
            let a = 2.0 * signal;
            let b = opamp_stage.opamp.process(a, 10_000.0);
            signal = (a + b) / 2.0; // Convert wave pair to voltage
        }

        // Guard against NaN/Inf from NL solver divergence — prevents
        // permanent state corruption in the oversampler's IIR filters.
        if !signal.is_finite() {
            if let Some(ref mut acc) = self.metrics_accumulator {
                acc.record_signal_fault(signal);
            }
            signal = 0.0;
        }
        signal = self
            .rail_sat_oversampler
            .process(signal, |s| self.rail_saturation.process(s, headroom));

        // Process through BBD delay lines (wet signal mixed with dry).
        for bbd in &mut self.bbds {
            let wet = bbd.process(signal);
            let mix = self.bbd_wet_mix;
            signal = signal * (1.0 - mix) + wet * mix;
        }

        // Process through spring reverb tanks (wet signal mixed with dry).
        // Per-instance mix (spec §6 — no global mix field, unlike bbd_wet_mix).
        for spring in &mut self.springs {
            let wet = spring.tank.process(signal);
            let mix = spring.wet_mix;
            signal = signal * (1.0 - mix) + wet * mix;
        }

        // Process through generic delay lines.
        // Each delay line writes the current signal, then reads from all taps.
        // Tap outputs are mixed equally and blended with the dry signal.
        for dl_binding in &mut self.delay_lines {
            // Reset speed modulation for this sample (LFOs will have already added offsets)
            dl_binding.delay_line.write(signal);

            // Read all taps and sum into the OUTPUT wet signal. The taps are
            // the playback heads (RE-201 heads at 1x/2x/3x); summing them is
            // the multi-head output mix.
            let num_taps = dl_binding.taps.len();
            if num_taps > 0 {
                let mut wet = 0.0;
                let mut base = 0.0;
                for (tap_idx, &ratio) in dl_binding.taps.iter().enumerate() {
                    let tap = dl_binding.delay_line.read_at_ratio(ratio, tap_idx);
                    wet += tap;
                    if tap_idx == 0 {
                        base = tap; // head 1 (ratio 1.0) — the regeneration tap
                    }
                }
                wet /= num_taps as crate::Wave;

                // Per-instance wet/dry mix (spec §6). Defaults to 0.5 — the
                // historical 50/50 — when no mix pot is wired.
                let mix = dl_binding.wet_mix;
                signal = signal * (1.0 - mix) + wet * mix;

                // Regeneration recirculates the BASE tape loop (head 1), not
                // the multi-head output sum: Intensity returns the loop signal
                // into the record path at the base delay. Feeding back the
                // head-average would change the loop's effective delay and
                // decay with tap count; the loop is the tape itself.
                dl_binding.delay_line.set_feedback_sample(base);
            }

            // Reset speed mod accumulator for next sample
            dl_binding.delay_line.reset_speed_mod();
        }

        // Apply output loading from downstream load impedance.
        // Models the voltage divider and frequency-dependent rolloff created by
        // this circuit's output impedance driving the load (downstream pedal,
        // amp input).
        if let Some(ref mut loading) = self.output_loading {
            signal = loading.process(signal);
        }

        // Output DC blocker — remove subsonic drift from integrator DC tails.
        if let Some((a1, b0, ref mut y_prev, ref mut x_prev)) = self.output_dc_block {
            let x = signal;
            let y = b0 * (x - *x_prev) + a1 * *y_prev;
            *x_prev = x;
            *y_prev = crate::stage::flush_denormal(y);
            signal = *y_prev;
        }

        // Final NaN guard — prevent corrupted audio from reaching the output.
        if !signal.is_finite() {
            if let Some(ref mut acc) = self.metrics_accumulator {
                acc.record_signal_fault(signal);
            }
            signal = 0.0;
        }
        let output = signal * self.output_gain;

        #[cfg(feature = "debug-trace")]
        if trace_on {
            std::eprintln!("  [OUT] output={output:.6e}");
        }

        // TODO: DebugStats not yet in rt crate
        // #[cfg(debug_assertions)]
        // if let Some(ref stats) = self.debug_stats {
        //     stats.record_output(output);
        // }

        // ═══════════════════════════════════════════════════════════════════════
        // Metering: accumulate per-sample, reduce per-block, write to ring buffer
        // ═══════════════════════════════════════════════════════════════════════
        if let Some(ref mut acc) = self.metrics_accumulator {
            // Accumulate input/output levels
            acc.accumulate_levels(input, output);

            // Accumulate per-stage signal levels for circuit view visualization
            for i in 0..num_stages.min(crate::metering::MAX_STAGES) {
                acc.accumulate_stage(i, stage_levels[i]);
                if stage_solver_active[i] {
                    acc.record_stage_solver(
                        i,
                        stage_solver_iterations[i],
                        stage_solver_residual[i],
                        stage_solver_converged[i],
                    );
                }
            }

            // Record tube state from WDF stages (plate current for glow shaders)
            let mut tube_idx = 0;
            for stage in &self.stages {
                if let Stage::Wdf(wdf) = stage {
                    match &wdf.root {
                        RootKind::Triode(t) => {
                            let vpk = (self.supply_voltage * 0.6) as f32;
                            let ip_ma = (t.plate_current(vpk as crate::Wave) * 1000.0) as f32;
                            acc.record_tube(tube_idx, ip_ma, vpk);
                            tube_idx += 1;
                        }
                        RootKind::Pentode(p) => {
                            let vpk = (self.supply_voltage * 0.6) as f32;
                            let ip_ma = (p.plate_current(vpk as crate::Wave) * 1000.0) as f32;
                            acc.record_tube(tube_idx, ip_ma, vpk);
                            tube_idx += 1;
                        }
                        _ => {}
                    }
                }
            }
            // Record push-pull triode tubes
            for pp in &self.push_pull_stages {
                let vpk = (self.supply_voltage * 0.6) as f32;
                let ip_push = (pp.push_root.plate_current(vpk as crate::Wave) * 1000.0) as f32;
                acc.record_tube(tube_idx, ip_push, vpk);
                tube_idx += 1;
                let ip_pull = (pp.pull_root.plate_current(vpk as crate::Wave) * 1000.0) as f32;
                acc.record_tube(tube_idx, ip_pull, vpk);
                tube_idx += 1;
            }

            // Record LFO phase (first LFO for tremolo indicator)
            if let Some(binding) = self.lfos.first() {
                acc.record_lfo_phase(binding.lfo.phase() as f32);
            }

            // Record supply voltage (for sag visualization when we add dynamic sag)
            acc.record_supply(self.supply_voltage as f32);

            // Check if block is complete — reduce and write to ring buffer
            if acc.is_block_complete() {
                let metrics = acc.reduce();
                if let Some(ref buffer) = self.metrics_buffer {
                    buffer.write(metrics);
                }
                // Stash the reduced metrics for the Phase-B diag frame, built
                // after the `acc` borrow ends (needs &self.stages + &mut ring).
                #[cfg(feature = "diag")]
                {
                    diag_block_metrics = Some(metrics);
                }
            }
        }

        // ── Diagnostics runtime ring (IPC Phase B, `diag` feature) ───────────
        // When a block completed AND a ring is attached, build one RuntimeFrame
        // (NR convergence from the grouped-NR SolverStats snapshot + per-BJT
        // operating points + per-stage levels) and push it lock-free.
        #[cfg(feature = "diag")]
        if let (Some(metrics), Some(ring)) =
            (diag_block_metrics, self.diag_ring.as_mut())
        {
            let mut frame = crate::diag_ring::RuntimeFrame {
                block_counter: metrics.block_counter as u64,
                ..Default::default()
            };

            // NR convergence: prefer the thread-local grouped-NR SolverStats
            // (the MultiNl path); fall back to the metering aggregate.
            let ss = crate::elements::nonlinear::solver::solver_stats_snapshot();
            if ss.solves > 0 {
                frame.nr_solves = ss.solves;
                frame.nr_total_iterations = ss.total_iterations;
                frame.nr_max_iterations = ss.max_iterations;
                frame.nr_iter_cap_hits = ss.iter_cap_hits as u32;
                frame.nr_max_residual = ss.max_residual as f32;
            } else {
                frame.nr_solves = metrics.nr_solve_count as u64;
                frame.nr_total_iterations = metrics.nr_total_iterations as u64;
                frame.nr_max_iterations = metrics.nr_max_iterations as u32;
                frame.nr_iter_cap_hits = metrics.nr_nonconverged_count as u32;
                frame.nr_max_residual = metrics.nr_max_residual;
            }
            crate::elements::nonlinear::solver::reset_solver_stats();

            // Levels: dB → linear RMS (input/output) + per-stage absolute level.
            let db2lin = |db: f32| if db <= -200.0 { 0.0 } else { 10f32.powf(db / 20.0) };
            frame.input_rms = db2lin(metrics.input_rms_db);
            frame.output_rms = db2lin(metrics.output_rms_db);
            let n_lvl = num_stages.min(crate::diag_ring::MAX_STAGE_LEVELS);
            for (dst, src) in frame.stage_levels[..n_lvl]
                .iter_mut()
                .zip(stage_levels[..n_lvl].iter())
            {
                *dst = *src as f32;
            }
            frame.n_stage_levels = n_lvl as u32;

            // Per-BJT/NL operating points from every MultiNl stage.
            let mut n_op = 0usize;
            for (si, stage) in self.stages.iter().enumerate() {
                if n_op >= crate::diag_ring::MAX_OP_RECORDS {
                    break;
                }
                if let Stage::MultiNl(mnl) = stage {
                    n_op += mnl.runtime_op_points(si, &mut frame.op_records[n_op..]);
                }
            }
            frame.n_op_records = n_op as u32;

            ring.push(&frame);
        }

        // Extract output port values from the processed signal.
        for port in &self.ports {
            if port.direction == crate::PortDirection::Output {
                // Primary audio output uses the serial chain result.
                // Other outputs read from node_signals (e.g., envelope taps).
                self.port_values[port.index] = self
                    .node_signals
                    .iter()
                    .rev()
                    .find(|(nid, _)| *nid == port.node_id)
                    .map(|(_, v)| *v)
                    .unwrap_or(output);
            }
        }

        // INTERNAL delayed ports (Phase 2b): the z⁻¹ WRITE. After the full sample
        // has solved, capture the value at each `source_node_id` from
        // `node_signals` (rev-find = the last writer, same convention as output
        // ports) into `prev_value`, where it waits one sample for the consumer.
        // For the detector this captures the mixed tap value at the front node
        // AND the solved `EL_drive` node (carry-only, consumer = MAX) so Phase 3
        // can drive the photocoupler LED from it.
        if !self.internal_ports.is_empty() {
            for idx in 0..self.internal_ports.len() {
                let src = self.internal_ports[idx].source_node_id;
                if let Some(&(_, v)) = self.node_signals.iter().rev().find(|(nid, _)| *nid == src) {
                    self.internal_ports[idx].prev_value = v;
                }
            }
        }

        output as Wave
    }

    fn set_sample_rate(&mut self, rate: crate::Wave) {
        self.sample_rate = rate;
        for stage in &mut self.stages {
            match stage {
                Stage::Wdf(wdf) => {
                    wdf.tree.update_sample_rate(rate);
                    wdf.tree.recompute();
                }
                Stage::MultiNl(mnl) => {
                    mnl.reset();
                }
                _ => {}
            }
        }
        for binding in &mut self.lfos {
            binding.lfo.set_sample_rate(rate);
        }
        for binding in &mut self.envelopes {
            binding.envelope.set_sample_rate(rate);
        }
        for bbd in &mut self.bbds {
            bbd.set_sample_rate(rate);
        }
        for dl_binding in &mut self.delay_lines {
            dl_binding.delay_line.set_sample_rate(rate);
        }
        for spring in &mut self.springs {
            spring.tank.set_sample_rate(rate);
        }
        if let Some(ref mut thermal) = self.thermal {
            thermal.set_sample_rate(rate);
        }
        for opamp_stage in &mut self.opamp_stages {
            opamp_stage.opamp.set_sample_rate(rate);
        }
        if let Some(ref mut psu) = self.power_supply {
            psu.set_sample_rate(rate);
        }
        for smoother in &mut self.pot_smoothers {
            smoother.set_sample_rate(rate);
        }
    }

    fn reset(&mut self) {
        for stage in &mut self.stages {
            match stage {
                Stage::Wdf(wdf) => wdf.reset(),
                Stage::MultiNl(mnl) => mnl.reset(),
                Stage::SerialDelayedFeedback(serial) => serial.reset(),
                // IIR, StateSpace, BlackFeedback don't have reset()
                _ => {}
            }
        }
        for binding in &mut self.lfos {
            binding.lfo.reset();
        }
        for binding in &mut self.envelopes {
            binding.envelope.reset();
        }
        for bbd in &mut self.bbds {
            bbd.reset();
        }
        for dl_binding in &mut self.delay_lines {
            dl_binding.delay_line.reset();
        }
        for spring in &mut self.springs {
            spring.tank.reset();
        }
        for opamp_stage in &mut self.opamp_stages {
            opamp_stage.opamp.reset();
        }
        if let Some(ref mut thermal) = self.thermal {
            thermal.reset();
        }
        if let Some(ref mut psu) = self.power_supply {
            psu.reset();
        }
        for sc in &mut self.sidechains {
            sc.reset();
        }
        self.rail_sat_oversampler.reset();
        if let Some((_, _, ref mut y_prev, ref mut x_prev)) = self.output_dc_block {
            *y_prev = 0.0;
            *x_prev = 0.0;
        }
    }

    fn set_control(&mut self, label: &str, value: crate::Wave) {
        self.set_control(label, value);
    }

    fn set_supply_voltage(&mut self, voltage: crate::Wave) {
        self.set_supply_voltage(voltage);
    }

    fn list_editable_components(&self) -> Vec<(String, &'static str, crate::Wave)> {
        self.list_editable_components()
    }

    fn set_passive(&mut self, comp_id: &str, value: crate::Wave) -> bool {
        self.set_passive(comp_id, value)
    }

    fn reset_passive(&mut self, comp_id: &str) -> bool {
        self.reset_passive(comp_id)
    }

    fn control_debug_info(&self) -> Vec<(String, crate::Wave, crate::Wave)> {
        let mut out = Vec::new();

        // Stage type map: {S0} WDF, {S1} IIR, etc.
        for (si, stage) in self.stages.iter().enumerate() {
            let type_name = match stage {
                Stage::Wdf(w) => {
                    if w.feedback_opamp.is_some() {
                        "WDF+OpAmp"
                    } else {
                        "WDF"
                    }
                }
                Stage::MultiNl(_) => "MultiNL",
                Stage::Iir(_) => "IIR",
                Stage::StateSpace(_) => "StateSpace",
                Stage::BlackFeedback(_) => "BlackFB",
                Stage::Blockwise(_) => "Blockwise",
                Stage::KMethod { .. } => "KMethod",
                Stage::SerialDelayedFeedback(_) => "SerialFB",
            };
            out.push((format!("{{S{si}}} {type_name}"), 0.0, 0.0));
        }

        // Named controls with smoother state
        for (i, ctrl) in self.controls.iter().enumerate() {
            let smoothed = self
                .pot_smoothers
                .iter()
                .find(|s| s.control_idx == i)
                .map(|s| s.current)
                .unwrap_or(0.0);
            let target = self
                .pot_smoothers
                .iter()
                .find(|s| s.control_idx == i)
                .map(|s| s.target)
                .unwrap_or(0.0);
            out.push((ctrl.label.clone(), target, smoothed));
        }

        // All pot leaves from stages (includes ganged halves like __aw/__wb)
        let mut seen = hashbrown::HashSet::new();
        for (si, stage) in self.stages.iter().enumerate() {
            match stage {
                Stage::Wdf(wdf) => {
                    wdf.tree.for_each_leaf(&mut |leaf| {
                        if leaf.type_tag() == "pot" {
                            if let Some(id) = leaf.comp_id() {
                                if seen.insert(format!("s{}:{}", si, id)) {
                                    let pos = leaf.pot_position().unwrap_or(0.0);
                                    let r = leaf.port_resistance();
                                    out.push((format!("[{}] {}", si, id), pos, r));
                                }
                            }
                        }
                    });
                }
                Stage::MultiNl(mnl) => {
                    for child in &mnl.pot_children {
                        child.for_each_leaf(&mut |leaf| {
                            if leaf.type_tag() == "pot" {
                                if let Some(id) = leaf.comp_id() {
                                    if seen.insert(format!("m{}:{}", si, id)) {
                                        let pos = leaf.pot_position().unwrap_or(0.0);
                                        let r = leaf.port_resistance();
                                        out.push((format!("[m{}] {}", si, id), pos, r));
                                    }
                                }
                            }
                        });
                    }
                }
                Stage::Blockwise(bkm) => {
                    for (bi, block) in bkm.k_method_blocks().enumerate() {
                        block.tree.for_each_leaf(&mut |leaf| {
                            if leaf.type_tag() == "pot" {
                                if let Some(id) = leaf.comp_id() {
                                    if seen.insert(format!("b{}:{}:{}", si, bi, id)) {
                                        let pos = leaf.pot_position().unwrap_or(0.0);
                                        let r = leaf.port_resistance();
                                        out.push((format!("[b{}.{}] {}", si, bi, id), pos, r));
                                    }
                                }
                            }
                        });
                    }
                }
                _ => {}
            }
        }

        out
    }

    fn resolve_port(&self, name: &str) -> Option<usize> {
        self.ports
            .iter()
            .find(|p| p.name.eq_ignore_ascii_case(name))
            .map(|p| p.index)
    }

    fn port_count(&self) -> usize {
        self.ports.len()
    }

    fn process_ports(&mut self, ports: &mut [Wave]) {
        // Copy input port values from caller's slice
        for port in &self.ports {
            if port.direction == crate::PortDirection::Input && port.index < ports.len() {
                self.port_values[port.index] = ports[port.index] as crate::Wave;
            }
        }

        // Run the main process with input from the first input port (backward compat)
        let input = self
            .ports
            .iter()
            .find(|p| p.direction == crate::PortDirection::Input)
            .map(|p| ports.get(p.index).copied().unwrap_or(0.0 as Wave))
            .unwrap_or(0.0 as Wave);
        let output = self.process(input);

        // Copy output port values to caller's slice
        for port in &self.ports {
            if port.direction == crate::PortDirection::Output && port.index < ports.len() {
                ports[port.index] = self.port_values[port.index] as Wave;
            }
        }
        // Also write the serial chain output to the first output port
        if let Some(out_port) = self
            .ports
            .iter()
            .find(|p| p.direction == crate::PortDirection::Output)
        {
            if out_port.index < ports.len() {
                ports[out_port.index] = output;
            }
        }
        if let Some(audio_out) = self
            .ports
            .iter()
            .find(|p| p.name.eq_ignore_ascii_case("audio_out"))
        {
            if audio_out.index < ports.len() {
                ports[audio_out.index] = output;
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Precompute extraction — lives in pedalkernel crate (compiler-only).
// TODO: Re-enable when precompute module is ported to rt.
// pub fn extract_precomputed_from_compiled(...) { ... }
