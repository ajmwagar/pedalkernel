//! Runtime routing derived from compiler-emitted stage graph metadata.
//!
//! `StageGraph` is descriptive: it records emitted stages, their boundary
//! ports, and original circuit-node adjacency. `StageRoutePlan` is executable:
//! it distills that graph into stable indices the audio processor can use
//! without rediscovering topology on the realtime path.

use alloc::string::String;
use alloc::vec::Vec;

use crate::boundary_math::{ProcessorPortId, ScatteringPortId};
use crate::processor::{NamedPortBinding, Stage};
pub use crate::route::{BindingId, PortBinding, Route};

/// Direction/role of one compiled-stage boundary port.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum StageGraphPortDirection {
    Input,
    Output,
    Control,
    Bidirectional,
}

/// One named boundary on an emitted DSP stage, tied back to the original
/// circuit graph node when known.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StageGraphPort {
    pub label: String,
    pub node_id: usize,
    pub direction: StageGraphPortDirection,
}

/// One emitted DSP stage in the compiled stage graph.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StageGraphNode {
    pub stage_idx: usize,
    pub kind: String,
    pub label: String,
    pub component_ids: Vec<String>,
    pub ports: Vec<StageGraphPort>,
}

/// Electrical adjacency between two emitted stage ports through one circuit
/// node. This is metadata for validation/routing; it is not the solver.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StageGraphConnection {
    pub node_id: usize,
    pub from_stage: usize,
    pub from_port: usize,
    pub to_stage: usize,
    pub to_port: usize,
}

/// Full emitted-stage connectivity graph.
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StageGraph {
    pub stages: Vec<StageGraphNode>,
    pub connections: Vec<StageGraphConnection>,
}

/// Executable routing distilled from `StageGraph`.
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StageRoutePlan {
    pub primary_bkm: Option<GraphRoutedBkm>,
    pub connections: Vec<StageRouteConnection>,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum StageRouteEndpointKind {
    Stage,
    ExternalInput,
    ExternalOutput,
}

impl Default for StageRouteEndpointKind {
    fn default() -> Self {
        Self::Stage
    }
}

#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StageRouteEndpoint {
    pub kind: StageRouteEndpointKind,
    pub graph_stage_index: usize,
    pub stage_idx: usize,
    pub port_idx: usize,
}

#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StageRouteConnection {
    pub node_id: usize,
    pub from: StageRouteEndpoint,
    pub to: StageRouteEndpoint,
}

#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StageRouteDebug {
    pub connection_count: usize,
    pub primary_bkm_stage_idx: Option<usize>,
    pub primary_bkm_vs_bindings: Vec<String>,
    pub primary_bkm_output_ports: Vec<usize>,
    pub primary_bkm_boundary_drives: Vec<String>,
}

impl StageRoutePlan {
    pub fn from_compiled_parts(
        graph: &StageGraph,
        ports: &[NamedPortBinding],
        stages: &[Stage],
    ) -> Self {
        let connections = Self::route_connections(graph);
        let Some(bkm_graph_idx) = graph
            .stages
            .iter()
            .position(|stage| stage.kind == "Blockwise" || stage.kind == "BlockwiseKMethod")
        else {
            return Self {
                connections,
                ..Self::default()
            };
        };
        let bkm_stage_idx = graph.stages[bkm_graph_idx].stage_idx;
        if bkm_stage_idx == usize::MAX {
            return Self {
                connections,
                ..Self::default()
            };
        }

        let Some(Stage::Blockwise(bkm)) = stages.get(bkm_stage_idx) else {
            return Self {
                connections,
                ..Self::default()
            };
        };

        let boundary_drives = Self::bkm_boundary_drives(graph, ports, stages, bkm_graph_idx, bkm);
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
            let touches_bkm = graph.connections.iter().any(|connection| {
                connection.node_id == port.node_id
                    && (connection.from_stage == bkm_graph_idx
                        || connection.to_stage == bkm_graph_idx)
            }) || graph.stages[bkm_graph_idx]
                .ports
                .iter()
                .any(|graph_port| graph_port.node_id == port.node_id);
            if port.direction == crate::PortDirection::Output && touches_bkm {
                output_port_indices.push(port.index);
            }
        }

        if vs_bindings.is_empty() || output_port_indices.is_empty() {
            return Self {
                connections,
                ..Self::default()
            };
        }

        Self {
            primary_bkm: Some(GraphRoutedBkm {
                stage_idx: bkm_stage_idx,
                vs_bindings,
                output_port_indices,
                boundary_drives,
            }),
            connections,
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
            connection_count: self.connections.len(),
            primary_bkm_stage_idx,
            primary_bkm_vs_bindings,
            primary_bkm_output_ports,
            primary_bkm_boundary_drives,
        }
    }

    fn bkm_boundary_drives(
        graph: &StageGraph,
        ports: &[NamedPortBinding],
        stages: &[Stage],
        bkm_graph_idx: usize,
        bkm: &crate::stage::BlockwiseStage,
    ) -> Vec<BkmBoundaryDrive> {
        let mut drives = Vec::new();
        for graph_stage in &graph.stages {
            if graph_stage.kind != "Wdf" || graph_stage.stage_idx == usize::MAX {
                continue;
            }

            let has_input_pair_boundaries = graph_stage
                .ports
                .iter()
                .any(|port| port.label == "base_left")
                && graph_stage
                    .ports
                    .iter()
                    .any(|port| port.label == "collector_left")
                && graph_stage
                    .ports
                    .iter()
                    .any(|port| port.label == "collector_right");
            if !has_input_pair_boundaries {
                continue;
            }
            if !matches!(stages.get(graph_stage.stage_idx), Some(Stage::Wdf(_))) {
                continue;
            }

            let base_left = graph_stage
                .ports
                .iter()
                .find(|port| port.label == "base_left")
                .map(|port| port.node_id);
            let base_right = graph_stage
                .ports
                .iter()
                .find(|port| port.label == "base_right")
                .map(|port| port.node_id);
            let collector_nodes = graph_stage
                .ports
                .iter()
                .filter(|port| port.label == "collector_left" || port.label == "collector_right")
                .map(|port| port.node_id)
                .collect::<Vec<_>>();
            let collector_left = graph_stage
                .ports
                .iter()
                .find(|port| port.label == "collector_left")
                .map(|port| port.node_id);
            let collector_right = graph_stage
                .ports
                .iter()
                .find(|port| port.label == "collector_right")
                .map(|port| port.node_id);

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
                let connected_to_bkm = graph.connections.iter().any(|connection| {
                    connection.node_id == node
                        && (connection.from_stage == bkm_graph_idx
                            || connection.to_stage == bkm_graph_idx)
                }) || graph.stages[bkm_graph_idx]
                    .ports
                    .iter()
                    .any(|port| port.node_id == node);
                if !connected_to_bkm {
                    continue;
                }
                for port_idx in 0..bkm.coupling_ports.len() {
                    let Some(terminals) = bkm.coupling_port_graph_terminals(port_idx) else {
                        continue;
                    };
                    let (node_pos, node_neg) = terminals.as_tuple();
                    if node_pos == Some(node) || node_neg == Some(node) {
                        target_coupling_port_indices.push(port_idx);
                    }
                }
            }
            target_coupling_port_indices.sort_unstable();
            target_coupling_port_indices.dedup();
            if target_coupling_port_indices.is_empty() {
                continue;
            }
            let mut positive_target_coupling_port_indices =
                Self::bkm_coupling_ports_for_node(bkm, collector_left);
            let mut negative_target_coupling_port_indices =
                Self::bkm_coupling_ports_for_node(bkm, collector_right);
            positive_target_coupling_port_indices.sort_unstable();
            positive_target_coupling_port_indices.dedup();
            negative_target_coupling_port_indices.sort_unstable();
            negative_target_coupling_port_indices.dedup();

            drives.push(BkmBoundaryDrive {
                source_stage_idx: graph_stage.stage_idx,
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
                label: graph_stage.label.clone(),
            });
        }
        drives
    }

    fn bkm_coupling_ports_for_node(
        bkm: &crate::stage::BlockwiseStage,
        node: Option<usize>,
    ) -> Vec<usize> {
        let Some(node) = node else {
            return Vec::new();
        };
        bkm.coupling_ports
            .iter()
            .enumerate()
            .filter_map(|(port_idx, port)| {
                let terminals = port.graph.raw();
                let (node_pos, node_neg) = terminals.as_tuple();
                (node_pos == Some(node) || node_neg == Some(node)).then_some(port_idx)
            })
            .collect()
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

    fn endpoint_kind(stage: &StageGraphNode) -> StageRouteEndpointKind {
        if stage.kind == "ExternalPort" {
            if stage
                .ports
                .first()
                .is_some_and(|port| port.direction == StageGraphPortDirection::Output)
            {
                StageRouteEndpointKind::ExternalInput
            } else {
                StageRouteEndpointKind::ExternalOutput
            }
        } else {
            StageRouteEndpointKind::Stage
        }
    }

    fn route_connections(graph: &StageGraph) -> Vec<StageRouteConnection> {
        graph
            .connections
            .iter()
            .map(|connection| {
                let from_stage = graph.stages.get(connection.from_stage);
                let to_stage = graph.stages.get(connection.to_stage);
                StageRouteConnection {
                    node_id: connection.node_id,
                    from: StageRouteEndpoint {
                        kind: from_stage.map(Self::endpoint_kind).unwrap_or_default(),
                        graph_stage_index: connection.from_stage,
                        stage_idx: from_stage
                            .map(|stage| stage.stage_idx)
                            .unwrap_or(usize::MAX),
                        port_idx: connection.from_port,
                    },
                    to: StageRouteEndpoint {
                        kind: to_stage.map(Self::endpoint_kind).unwrap_or_default(),
                        graph_stage_index: connection.to_stage,
                        stage_idx: to_stage.map(|stage| stage.stage_idx).unwrap_or(usize::MAX),
                        port_idx: connection.to_port,
                    },
                }
            })
            .collect()
    }
}
