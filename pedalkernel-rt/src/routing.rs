//! Runtime routing derived from stage-owned boundary bindings.
//!
//! `StageRoutePlan` is executable: it distills stage `ins()`/`outs()` metadata
//! into stable indices the audio processor can use without rediscovering
//! topology on the realtime path.

use alloc::string::String;
use alloc::vec::Vec;

use crate::boundary_math::{ProcessorPortId, ScatteringPortId};
use crate::processor::{NamedPortBinding, Stage};
pub use crate::route::{BindingId, PortBinding, Route};

/// Executable routing distilled from stage-owned boundary bindings.
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StageRoutePlan {
    pub primary_bkm: Option<GraphRoutedBkm>,
}

#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GraphRoutedBkm {
    pub stage_idx: usize,
    pub vs_bindings: Vec<BkmVsRouteBinding>,
    pub output_port_indices: Vec<usize>,
    pub boundary_drives: Vec<BkmBoundaryDrive>,
}

#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BkmVsRouteBinding {
    pub signal_index: usize,
    pub port_index: usize,
    pub coupling_port_index: usize,
    pub port_name: String,
}

impl BkmVsRouteBinding {
    pub fn new(
        signal_index: usize,
        processor_port: ProcessorPortId,
        scattering_port: ScatteringPortId,
        port_name: String,
    ) -> Self {
        Self {
            signal_index,
            port_index: processor_port.get(),
            coupling_port_index: scattering_port.get(),
            port_name,
        }
    }

    pub const fn processor_port(&self) -> ProcessorPortId {
        ProcessorPortId::new(self.port_index)
    }

    pub const fn scattering_port(&self) -> ScatteringPortId {
        ScatteringPortId::new(self.coupling_port_index)
    }
}

#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BkmBoundaryDrive {
    pub source_stage_idx: usize,
    pub positive_input_port_indices: Vec<usize>,
    pub negative_input_port_indices: Vec<usize>,
    #[cfg_attr(feature = "serde", serde(default))]
    pub positive_input_port_names: Vec<String>,
    #[cfg_attr(feature = "serde", serde(default))]
    pub negative_input_port_names: Vec<String>,
    pub target_coupling_port_indices: Vec<usize>,
    #[cfg_attr(feature = "serde", serde(default))]
    pub positive_target_coupling_port_indices: Vec<usize>,
    #[cfg_attr(feature = "serde", serde(default))]
    pub negative_target_coupling_port_indices: Vec<usize>,
    pub label: String,
}

impl BkmBoundaryDrive {
    pub fn positive_processor_ports(&self) -> impl Iterator<Item = ProcessorPortId> + '_ {
        self.positive_input_port_indices
            .iter()
            .copied()
            .map(ProcessorPortId::new)
    }

    pub fn negative_processor_ports(&self) -> impl Iterator<Item = ProcessorPortId> + '_ {
        self.negative_input_port_indices
            .iter()
            .copied()
            .map(ProcessorPortId::new)
    }

    pub fn target_scattering_ports(&self) -> impl Iterator<Item = ScatteringPortId> + '_ {
        self.target_coupling_port_indices
            .iter()
            .copied()
            .map(ScatteringPortId::new)
    }

    pub fn positive_target_scattering_ports(&self) -> impl Iterator<Item = ScatteringPortId> + '_ {
        self.positive_target_coupling_port_indices
            .iter()
            .copied()
            .map(ScatteringPortId::new)
    }

    pub fn negative_target_scattering_ports(&self) -> impl Iterator<Item = ScatteringPortId> + '_ {
        self.negative_target_coupling_port_indices
            .iter()
            .copied()
            .map(ScatteringPortId::new)
    }
}

#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StageRouteDebug {
    pub primary_bkm_stage_idx: Option<usize>,
    pub primary_bkm_vs_bindings: Vec<String>,
    pub primary_bkm_output_ports: Vec<usize>,
    pub primary_bkm_boundary_drives: Vec<String>,
}

impl StageRoutePlan {
    pub fn from_compiled_parts(ports: &[NamedPortBinding], stages: &[Stage]) -> Self {
        let Some((bkm_stage_idx, Stage::Blockwise(bkm))) = stages
            .iter()
            .enumerate()
            .find(|(_, stage)| matches!(stage, Stage::Blockwise(_)))
        else {
            return Self::default();
        };

        let bkm_stage = &stages[bkm_stage_idx];
        let boundary_drives = Self::bkm_boundary_drives(ports, stages, bkm_stage, bkm);
        let consumed_port_indices = boundary_drives
            .iter()
            .flat_map(|drive| {
                drive
                    .positive_input_port_indices
                    .iter()
                    .chain(drive.negative_input_port_indices.iter())
            })
            .copied()
            .collect::<Vec<_>>();

        let mut vs_bindings = Vec::new();
        for (signal_index, (name, coupling_port_index)) in bkm.vs_port_map.iter().enumerate() {
            if let Some(port) = ports.iter().find(|port| {
                port.direction == crate::PortDirection::Input
                    && port.name.eq_ignore_ascii_case(name)
                    && !consumed_port_indices.contains(&port.index)
            }) {
                vs_bindings.push(BkmVsRouteBinding::new(
                    signal_index,
                    ProcessorPortId::new(port.index),
                    ScatteringPortId::new(*coupling_port_index),
                    port.name.clone(),
                ));
            }
        }

        let mut output_port_indices = Vec::new();
        for port in ports {
            if port.node_id == usize::MAX {
                continue;
            }
            let touches_bkm = Self::stage_has_binding_for_node(bkm_stage, port.node_id);
            if port.direction == crate::PortDirection::Output && touches_bkm {
                output_port_indices.push(port.index);
            }
        }

        if boundary_drives.is_empty() || vs_bindings.is_empty() || output_port_indices.is_empty() {
            return Self::default();
        }

        Self {
            primary_bkm: Some(GraphRoutedBkm {
                stage_idx: bkm_stage_idx,
                vs_bindings,
                output_port_indices,
                boundary_drives,
            }),
        }
    }

    pub fn debug(&self) -> StageRouteDebug {
        let primary_bkm_stage_idx = self.primary_bkm.as_ref().map(|route| route.stage_idx);
        let primary_bkm_vs_bindings = self
            .primary_bkm
            .as_ref()
            .map(|route| {
                route
                    .vs_bindings
                    .iter()
                    .map(|binding| {
                        alloc::format!(
                            "{}@vs{}->port{}",
                            binding.port_name,
                            binding.signal_index,
                            binding.coupling_port_index
                        )
                    })
                    .collect()
            })
            .unwrap_or_default();
        let primary_bkm_output_ports = self
            .primary_bkm
            .as_ref()
            .map(|route| route.output_port_indices.clone())
            .unwrap_or_default();
        let primary_bkm_boundary_drives = self
            .primary_bkm
            .as_ref()
            .map(|route| {
                route
                    .boundary_drives
                    .iter()
                    .map(|drive| {
                        alloc::format!(
                            "{}@stage{} +{:?} -{:?} -> {:?}",
                            drive.label,
                            drive.source_stage_idx,
                            drive.positive_input_port_names,
                            drive.negative_input_port_names,
                            drive.target_coupling_port_indices
                        )
                    })
                    .collect()
            })
            .unwrap_or_default();
        StageRouteDebug {
            primary_bkm_stage_idx,
            primary_bkm_vs_bindings,
            primary_bkm_output_ports,
            primary_bkm_boundary_drives,
        }
    }

    fn bkm_boundary_drives(
        ports: &[NamedPortBinding],
        stages: &[Stage],
        bkm_stage: &Stage,
        bkm: &crate::stage::BlockwiseStage,
    ) -> Vec<BkmBoundaryDrive> {
        let mut drives = Vec::new();
        for (source_stage_idx, stage) in stages.iter().enumerate() {
            let Stage::Wdf(wdf) = stage else {
                continue;
            };

            let has_input_pair_boundaries = wdf
                .boundary_bindings
                .iter()
                .any(|binding| binding.label == "base_left")
                && wdf
                    .boundary_bindings
                    .iter()
                    .any(|binding| binding.label == "collector_left")
                && wdf
                    .boundary_bindings
                    .iter()
                    .any(|binding| binding.label == "collector_right");
            if !has_input_pair_boundaries {
                continue;
            }

            let base_left = wdf
                .boundary_bindings
                .iter()
                .find(|binding| binding.label == "base_left")
                .map(|binding| binding.node_id);
            let base_right = wdf
                .boundary_bindings
                .iter()
                .find(|binding| binding.label == "base_right")
                .map(|binding| binding.node_id);
            let collector_nodes = wdf
                .boundary_bindings
                .iter()
                .filter(|binding| {
                    binding.label == "collector_left" || binding.label == "collector_right"
                })
                .map(|binding| binding.node_id)
                .collect::<Vec<_>>();
            let collector_left = wdf
                .boundary_bindings
                .iter()
                .find(|binding| binding.label == "collector_left")
                .map(|binding| binding.node_id);
            let collector_right = wdf
                .boundary_bindings
                .iter()
                .find(|binding| binding.label == "collector_right")
                .map(|binding| binding.node_id);

            let mut positive_input_port_indices =
                Self::external_ports_reaching_node(ports, bkm, base_left);
            let mut negative_input_port_indices =
                Self::external_ports_reaching_node(ports, bkm, base_right);
            positive_input_port_indices.sort_unstable();
            positive_input_port_indices.dedup();
            negative_input_port_indices.sort_unstable();
            negative_input_port_indices.dedup();
            if positive_input_port_indices.is_empty() && negative_input_port_indices.is_empty() {
                continue;
            }

            let mut target_coupling_port_indices = Vec::new();
            for node in collector_nodes {
                target_coupling_port_indices
                    .extend(Self::stage_port_indices_for_node(bkm_stage, node));
            }
            target_coupling_port_indices.sort_unstable();
            target_coupling_port_indices.dedup();
            if target_coupling_port_indices.is_empty() {
                continue;
            }
            let mut positive_target_coupling_port_indices = collector_left
                .map(|node| Self::stage_port_indices_for_node(bkm_stage, node))
                .unwrap_or_default();
            let mut negative_target_coupling_port_indices = collector_right
                .map(|node| Self::stage_port_indices_for_node(bkm_stage, node))
                .unwrap_or_default();
            positive_target_coupling_port_indices.sort_unstable();
            positive_target_coupling_port_indices.dedup();
            negative_target_coupling_port_indices.sort_unstable();
            negative_target_coupling_port_indices.dedup();

            drives.push(BkmBoundaryDrive {
                source_stage_idx,
                positive_input_port_names: Self::port_names_for_indices(
                    ports,
                    &positive_input_port_indices,
                ),
                negative_input_port_names: Self::port_names_for_indices(
                    ports,
                    &negative_input_port_indices,
                ),
                positive_input_port_indices,
                negative_input_port_indices,
                target_coupling_port_indices,
                positive_target_coupling_port_indices,
                negative_target_coupling_port_indices,
                label: wdf.root_comp_id.clone(),
            });
        }
        drives
    }

    fn external_ports_reaching_node(
        ports: &[NamedPortBinding],
        bkm: &crate::stage::BlockwiseStage,
        node: Option<usize>,
    ) -> Vec<usize> {
        let Some(node) = node else {
            return Vec::new();
        };
        let mut adjacency = Vec::new();
        for element in &bkm.coupling_elements {
            if let (Some(a), Some(b)) = (element.graph_node_a, element.graph_node_b) {
                adjacency.push((a, b));
            }
        }
        for passive in &bkm.coupling_passives {
            if let Some(terminals) = bkm.coupling_port_graph_terminals(passive.port_idx) {
                let (a, b) = terminals.as_tuple();
                let (Some(a), Some(b)) = (a, b) else {
                    continue;
                };
                adjacency.push((a, b));
            }
        }

        let mut out = Vec::new();
        for port in ports {
            if port.direction != crate::PortDirection::Input {
                continue;
            }
            if !Self::is_audio_boundary_drive_port(&port.name) {
                continue;
            }
            if Self::nodes_connected_by_coupling(port.node_id, node, &adjacency) {
                out.push(port.index);
            }
        }
        out
    }

    fn is_audio_boundary_drive_port(name: &str) -> bool {
        let name = name.to_ascii_lowercase();
        (name.contains("audio") || name.contains("vco"))
            && !name.starts_with("cv_")
            && !name.contains("gate")
    }

    fn port_names_for_indices(ports: &[NamedPortBinding], indices: &[usize]) -> Vec<String> {
        indices
            .iter()
            .filter_map(|index| {
                ports
                    .iter()
                    .find(|port| port.index == *index)
                    .map(|port| port.name.clone())
            })
            .collect()
    }

    fn stage_has_binding_for_node(stage: &Stage, node_id: usize) -> bool {
        let binding_id = BindingId::new(node_id);
        stage
            .ins()
            .into_iter()
            .chain(stage.outs())
            .any(|binding| binding.binding_id == binding_id)
    }

    fn stage_port_indices_for_node(stage: &Stage, node_id: usize) -> Vec<usize> {
        let binding_id = BindingId::new(node_id);
        let mut indices = stage
            .ins()
            .into_iter()
            .chain(stage.outs())
            .filter_map(|binding| (binding.binding_id == binding_id).then_some(binding.local_port))
            .collect::<Vec<_>>();
        indices.sort_unstable();
        indices.dedup();
        indices
    }

    fn nodes_connected_by_coupling(start: usize, target: usize, edges: &[(usize, usize)]) -> bool {
        if start == target {
            return true;
        }
        let mut stack = Vec::new();
        let mut seen = Vec::new();
        stack.push(start);
        while let Some(node) = stack.pop() {
            if node == target {
                return true;
            }
            if seen.contains(&node) {
                continue;
            }
            seen.push(node);
            for &(a, b) in edges {
                if a == node && !seen.contains(&b) {
                    stack.push(b);
                } else if b == node && !seen.contains(&a) {
                    stack.push(a);
                }
            }
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use alloc::string::ToString;
    use alloc::vec;

    use super::*;
    use crate::boundary_math::{
        CircuitMappedPort, ExtractionProbe, GraphNodeId, MnaNodeId, PortSpec, RuntimeState,
        WdfPortTerminals,
    };
    use crate::dyn_node::DynNode;
    use crate::oversampling::{Oversampler, OversamplingFactor};
    use crate::processor::NamedPortBinding;
    use crate::stage::{
        BlockwiseSolveMode, BlockwiseStage, CoupledSolveScratch, RootKind, WdfBoundaryBinding,
        WdfBoundaryDirection, WdfStage,
    };

    fn mapped_port(terminals: WdfPortTerminals, resistance: crate::Wave) -> CircuitMappedPort {
        CircuitMappedPort::new(
            terminals.map(GraphNodeId::new),
            PortSpec::new(terminals.map(MnaNodeId::new), resistance),
        )
    }

    fn empty_bkm_with_port(node_id: usize) -> BlockwiseStage {
        BlockwiseStage {
            coupling_s: vec![0.0],
            coupling_n_mna: 0,
            coupling_ports: vec![mapped_port(WdfPortTerminals::single_ended(node_id), 1.0)],
            sub_stages: vec![],
            coupling_elements: vec![],
            coupling_passives: vec![],
            coupling_one_ports: vec![],
            coupling_runtime_state: RuntimeState::new(),
            coupling_passive_by_port: vec![],
            coupling_vcvss: vec![],
            n_ports: 1,
            output_block: 0,
            output_port_index: Some(0),
            supply_voltage: 9.0,
            vs_port_map: vec![("audio_in".to_string(), 0)],
            cutoff_cv_port: None,
            shared_diode_cutoff_pot: None,
            feedback_port_map: vec![],
            output_extraction: ExtractionProbe::default(),
            compensation: 1.0,
            oversampler: Oversampler::new(OversamplingFactor::X1),
            signal_flow_distance: 0,
            bypass_serial: false,
            solve_mode: BlockwiseSolveMode::Cascade,
            diode_ladder_core: None,
            b_warm: vec![0.0],
            work_b: vec![0.0],
            work_a: vec![0.0],
            coupled_scratch: CoupledSolveScratch::default(),
            port_index_cache: vec![],
        }
    }

    #[test]
    fn bkm_voltage_ports_use_generic_binding_routes_until_boundary_drive_is_needed() {
        let mut source = WdfStage::new(
            DynNode::VoltageSource(0.0, 1.0),
            RootKind::ShortCircuit,
            Oversampler::new(OversamplingFactor::X1),
        );
        source.boundary_bindings = vec![WdfBoundaryBinding {
            label: "source_out".to_string(),
            node_id: 42,
            direction: WdfBoundaryDirection::Output,
        }];

        let ports = vec![
            NamedPortBinding {
                name: "audio_in".to_string(),
                direction: crate::PortDirection::Input,
                index: 0,
                node_id: 42,
                default_value: 0.0,
                stage_idx: 1,
            },
            NamedPortBinding {
                name: "audio_out".to_string(),
                direction: crate::PortDirection::Output,
                index: 1,
                node_id: 42,
                default_value: 0.0,
                stage_idx: 1,
            },
        ];
        let stages = vec![
            Stage::Wdf(source),
            Stage::Blockwise(empty_bkm_with_port(42)),
        ];

        assert!(
            stages[0]
                .outs()
                .iter()
                .any(|binding| binding.binding_id.get() == 42),
            "source stage should expose the shared graph node as an output binding"
        );
        assert!(
            stages[1]
                .ins()
                .iter()
                .any(|binding| binding.binding_id.get() == 42),
            "BKM stage should expose the shared graph node as an input binding"
        );
        let plan = StageRoutePlan::from_compiled_parts(&ports, &stages);
        assert!(
            plan.primary_bkm.is_none(),
            "ordinary BKM voltage mapping is not an executable graph-routed BKM handoff"
        );
    }
}
