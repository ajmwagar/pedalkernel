//! Runtime routing derived from compiler-emitted stage graph metadata.
//!
//! `StageGraph` is descriptive: it records emitted stages, their boundary
//! ports, and original circuit-node adjacency. `StageRoutePlan` is executable:
//! it distills that graph into stable indices the audio processor can use
//! without rediscovering topology on the realtime path.

use alloc::string::String;
use alloc::vec::Vec;

use crate::processor::{PortBinding, Stage};

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
}

#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BkmVsRouteBinding {
    pub signal_index: usize,
    pub port_index: usize,
    pub coupling_port_index: usize,
    pub port_name: String,
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
}

impl StageRoutePlan {
    pub fn from_compiled_parts(
        graph: &StageGraph,
        ports: &[PortBinding],
        stages: &[Stage],
    ) -> Self {
        let connections = Self::route_connections(graph);
        let Some(bkm_graph_idx) = graph
            .stages
            .iter()
            .position(|stage| stage.kind == "BlockwiseKMethod")
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

        let Some(Stage::BlockwiseKMethod(bkm)) = stages.get(bkm_stage_idx) else {
            return Self {
                connections,
                ..Self::default()
            };
        };

        let mut vs_bindings = Vec::new();
        for (signal_index, (name, coupling_port_index)) in bkm.vs_port_map.iter().enumerate() {
            if let Some(port) = ports.iter().find(|port| {
                port.direction == crate::PortDirection::Input
                    && port.name.eq_ignore_ascii_case(name)
            }) {
                vs_bindings.push(BkmVsRouteBinding {
                    signal_index,
                    port_index: port.index,
                    coupling_port_index: *coupling_port_index,
                    port_name: port.name.clone(),
                });
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
        StageRouteDebug {
            connection_count: self.connections.len(),
            primary_bkm_stage_idx,
            primary_bkm_vs_bindings,
            primary_bkm_output_ports,
        }
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
