//! Blockwise decomposition of large nonlinear circuits.
//!
//! Detects when a monolithic multi-NL system can be split into smaller
//! chained blocks with sparse inter-block coupling. Uses the SPQR tree
//! structure directly:
//!
//! 1. Find P-nodes containing NL edges → those are the blocks
//! 2. Note each P-node's endpoints → block port nodes
//! 3. Assign sibling Q-leaves to blocks by which port nodes they touch
//! 4. Validate: ≥2 blocks, each has reactive state, coupling is NL-free

use super::component::EdgeKind;
use super::graph::{CircuitGraph, NodeId};
use super::spqr::{spqr_decompose, spqr_to_stages, SpqrNode};
use super::spqr_build::BuiltStage;
use pedalkernel_rt::boundary_math::{
    CircuitMappedPort, ExtractionProbe, GraphNodeId, MnaNodeId, MnaPortTerminals, OnePort,
    OnePortKind, PortSpec, PortTerminals, RuntimeOnePort, RuntimeState, ScatteringPortId,
    WdfPortTerminals,
};
use pedalkernel_rt::processor::Stage;
use pedalkernel_rt::route::{BindingId, PortBinding};
use pedalkernel_rt::stage::{KMethodBlock, OwnedPortRole};
use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};

fn push_coupling_port(
    ports: &mut Vec<CircuitMappedPort>,
    graph_terminals: WdfPortTerminals,
    mna_terminals: WdfPortTerminals,
    resistance: f64,
) -> usize {
    let port_idx = ports.len();
    ports.push(CircuitMappedPort::new(
        graph_terminals.map(GraphNodeId::new),
        PortSpec::new(mna_terminals.map(MnaNodeId::new), resistance),
    ));
    port_idx
}

fn bindings_for_coupling_port(ports: &[CircuitMappedPort], port_idx: usize) -> Vec<PortBinding> {
    let Some(terminals) = ports.get(port_idx).map(|port| port.graph.raw()) else {
        return Vec::new();
    };
    let (pos, neg) = terminals.as_tuple();
    pos.into_iter()
        .chain(neg)
        .map(|node| PortBinding::new(BindingId::new(node), port_idx))
        .collect()
}

fn push_k_method_port(
    sub_stages: &mut [Stage],
    block_idx: usize,
    role: OwnedPortRole,
    ports: &[CircuitMappedPort],
    port_idx: usize,
) {
    let Some(Stage::KMethod {
        ports: stage_ports, ..
    }) = sub_stages.get_mut(block_idx)
    else {
        return;
    };
    stage_ports.extend(
        bindings_for_coupling_port(ports, port_idx)
            .into_iter()
            .map(|binding| (role, binding)),
    );
}

fn k_method_block(stage: &Stage) -> Option<&KMethodBlock> {
    match stage {
        Stage::KMethod { block, .. } => Some(block),
        _ => None,
    }
}

fn k_method_ports(stage: &Stage) -> Option<&[(OwnedPortRole, PortBinding)]> {
    match stage {
        Stage::KMethod { ports, .. } => Some(ports.as_slice()),
        _ => None,
    }
}

/// Compiler-recognized topology for a blockwise nonlinear unit.
///
/// Keep these generic graph shapes rather than pedal-specific names. The
/// builder can then choose a specialized realtime primitive without baking
/// TB303/EMS/etc. knowledge into the compiler.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum BlockTopology {
    Generic,
    DifferentialDiodeRung {
        left_comp_idx: usize,
        right_comp_idx: usize,
        reactive_edge: usize,
    },
    InputDifferentialPair {
        left_comp_idx: usize,
        right_comp_idx: usize,
        shared_emitter: NodeId,
    },
}

/// A block in the blockwise decomposition — one NL device + its local state.
#[derive(Debug)]
pub struct Block {
    /// NL edge indices (the nonlinear device junctions).
    pub nl_edges: Vec<usize>,
    /// Linear edge indices interior to this block.
    pub linear_edges: Vec<usize>,
    /// Reactive edge indices interior to this block.
    pub reactive_edges: Vec<usize>,
    /// Port nodes — where this block connects to the coupling network.
    pub port_nodes: Vec<NodeId>,
    /// Recognized block topology.
    pub topology: BlockTopology,
}

/// Why an edge belongs to the residual coupling network.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum CouplingEdgeRole {
    /// Structural path between two nonlinear blocks.
    InterBlock,
    /// Unclaimed passive/control edge left after block-local ownership.
    Residual,
    /// Input-side edge or short chain that must drive the coupling network.
    BoundaryInput,
    /// Output-side load/probe edge that must remain in the coupling network.
    BoundaryOutput,
}

/// A typed residual coupling edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct CouplingEdge {
    pub edge_idx: usize,
    pub role: CouplingEdgeRole,
}

/// Neutral coupling-network description produced by blockwise analysis.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct CouplingNetwork {
    pub edges: Vec<CouplingEdge>,
    pub boundary_nodes: Vec<NodeId>,
}

impl CouplingNetwork {
    fn new(boundary_nodes: Vec<NodeId>) -> Self {
        Self {
            edges: Vec::new(),
            boundary_nodes,
        }
    }

    fn push_edge(&mut self, edge_idx: usize, role: CouplingEdgeRole) -> bool {
        if self.contains_edge(edge_idx) {
            return false;
        }
        self.edges.push(CouplingEdge { edge_idx, role });
        true
    }

    pub(super) fn contains_edge(&self, edge_idx: usize) -> bool {
        self.edges.iter().any(|edge| edge.edge_idx == edge_idx)
    }

    pub(super) fn is_empty(&self) -> bool {
        self.edges.is_empty()
    }

    pub(super) fn len(&self) -> usize {
        self.edges.len()
    }

    pub(super) fn edge_indices(&self) -> Vec<usize> {
        self.edges.iter().map(|edge| edge.edge_idx).collect()
    }

    pub(super) fn iter_edge_indices(&self) -> impl Iterator<Item = usize> + '_ {
        self.edges.iter().map(|edge| edge.edge_idx)
    }
}

/// Runtime formulation selected for one structural block.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum BlockFormulation {
    DifferentialDiodeRungWdf,
    InputDifferentialPairWdf,
    SpqrSubgraph,
}

/// Runtime formulation selected for the residual coupling network.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum CouplingFormulation {
    None,
    Serial,
    SerialDelayedFeedback,
    BlockwiseCoupled { coupled_newton: bool },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct FormulationSelection {
    pub blocks: Vec<BlockFormulation>,
    pub coupling: CouplingFormulation,
}

pub(super) fn select_formulations(
    plan: &BlockwisePlan,
    force_serial: bool,
    force_serial_feedback_gain: f64,
    coupled_newton: bool,
) -> FormulationSelection {
    let blocks = plan
        .blocks
        .iter()
        .map(|block| match block.topology {
            BlockTopology::DifferentialDiodeRung { .. } => {
                BlockFormulation::DifferentialDiodeRungWdf
            }
            BlockTopology::InputDifferentialPair { .. } => {
                BlockFormulation::InputDifferentialPairWdf
            }
            BlockTopology::Generic => BlockFormulation::SpqrSubgraph,
        })
        .collect();

    let coupling = if plan.coupling.is_empty() {
        CouplingFormulation::None
    } else if force_serial {
        if force_serial_feedback_gain.abs() > 0.0 {
            CouplingFormulation::SerialDelayedFeedback
        } else {
            CouplingFormulation::Serial
        }
    } else {
        CouplingFormulation::BlockwiseCoupled { coupled_newton }
    };

    FormulationSelection { blocks, coupling }
}

struct LoweredBlockStages {
    stages: Vec<BuiltStage>,
    stage_plan_blocks: Vec<usize>,
}

fn lower_block_stages(
    plan: &BlockwisePlan,
    selection: &FormulationSelection,
    graph: &CircuitGraph,
    terminals: &[NodeId],
    sample_rate: f64,
    bias_node_voltages: &BTreeMap<NodeId, f64>,
    supply_voltage: f64,
    disable_iir: bool,
    init_hints: &[crate::dsl::InitHint],
) -> Option<LoweredBlockStages> {
    let mut stages = Vec::new();
    let mut stage_plan_blocks = Vec::new();

    for (bi, block) in plan.blocks.iter().enumerate() {
        let block_edges = block.all_edges();
        if block_edges.is_empty() {
            continue;
        }

        if matches!(
            selection.blocks.get(bi),
            Some(BlockFormulation::DifferentialDiodeRungWdf)
        ) {
            if let Some(mut wdf) =
                build_differential_diode_rung_stage(block, graph, sample_rate, supply_voltage)
            {
                #[cfg(test)]
                eprintln!(
                    "  Block {bi}: {} edges -> 1 stage, topology={:?}",
                    block_edges.len(),
                    block.topology
                );
                wdf.k_table = super::k_method::generate_k_table(&mut wdf);
                stages.push(BuiltStage::Wdf(wdf));
                stage_plan_blocks.push(bi);
                continue;
            }
        }

        if matches!(
            selection.blocks.get(bi),
            Some(BlockFormulation::InputDifferentialPairWdf)
        ) {
            if let Some(mut wdf) = build_input_differential_pair_stage(block, graph, supply_voltage)
            {
                #[cfg(test)]
                eprintln!(
                    "  Block {bi}: {} edges -> 1 stage, topology={:?}",
                    block_edges.len(),
                    block.topology
                );
                wdf.k_table = super::k_method::generate_k_table(&mut wdf);
                stages.push(BuiltStage::Wdf(wdf));
                stage_plan_blocks.push(bi);
                continue;
            }
        }

        let block_terminals =
            super::spqr_build::compute_group_terminals(&block_edges, graph, terminals);
        let spqr_tree = spqr_decompose(&block_edges, &block_terminals, graph, graph.gnd_node);
        let spqr_stages = spqr_to_stages(&spqr_tree, graph, sample_rate);

        #[cfg(test)]
        {
            let class = super::spqr::classify_sp_subtree(&spqr_tree, graph);
            eprintln!(
                "  Block {bi}: {} edges -> {} stages, class={:?}",
                block_edges.len(),
                spqr_stages.len(),
                class,
            );
        }

        for (si, stage) in spqr_stages.into_iter().enumerate() {
            eprintln!("  [blockwise] block {bi} stage {si}: building...");
            let mut built = super::spqr_build::build_spqr_stage_with_options(
                stage,
                graph,
                sample_rate,
                disable_iir,
                init_hints,
                supply_voltage,
                bias_node_voltages,
            )
            .ok()?;
            eprintln!("  [blockwise] block {bi} stage {si}: built");

            if let BuiltStage::Wdf(ref mut wdf) = built {
                if wdf.output_probe.is_none() {
                    if let Some(&reactive_edge) = block.reactive_edges.first() {
                        let comp = &graph.components[graph.edges[reactive_edge].comp_idx];
                        wdf.output_probe = Some(comp.id.clone());
                    }
                }
            }

            if let BuiltStage::Wdf(ref mut wdf) = built {
                if let pedalkernel_rt::stage::RootKind::Bjt(ref mut bjt) = wdf.root {
                    // ko5g.4: the raw StaticBias base-voltage read (and the
                    // formerly-anonymous `default_vbe` fallback) live in
                    // bias.rs — see `solve_blockwise_bjt_base_bias` /
                    // `bjt_unconditional_default_vbe` for the preserved
                    // legacy semantics (v_base.min(0.8), no load line).
                    match super::bias::solve_blockwise_bjt_base_bias(
                        &block.nl_edges,
                        graph,
                        bias_node_voltages,
                    ) {
                        Ok(vbe) => {
                            bjt.set_bias(vbe);
                            #[cfg(test)]
                            eprintln!("  Block {bi}: BJT bias = {vbe:.3}V (from circuit)");
                        }
                        Err(skip) => {
                            // ko5g.4 warn-not-error: this call-site's legacy
                            // fallback is the UNCONDITIONAL conduction-forcing
                            // default (the ko5g.2 audit's S9) — byte-identical,
                            // but no longer silent.
                            skip.warn_if_undeterminable(
                                "Blockwise BJT stage FORCES the unconditional \
                                 nominal-conduction default \
                                 (bias::bjt_unconditional_default_vbe).",
                            );
                            let default_vbe =
                                super::bias::bjt_unconditional_default_vbe(supply_voltage);
                            bjt.set_bias(default_vbe);
                            if std::env::var("PK_BIAS_QPOINT_DEBUG").is_ok() {
                                eprintln!(
                                    "[bias-qpoint] path=blockwise bjt block{bi}: DEFAULT \
                                     vbe={default_vbe:.6} supply={supply_voltage}"
                                );
                            }
                            #[cfg(test)]
                            eprintln!("  Block {bi}: BJT bias = {default_vbe:.1}V (default)");
                        }
                    }

                    bjt.set_v_max(supply_voltage.abs().max(1.0));
                }
            }

            stages.push(built);
            stage_plan_blocks.push(bi);
        }
    }

    Some(LoweredBlockStages {
        stages,
        stage_plan_blocks,
    })
}

/// External electrical boundary of a compiler-recognized differential rung.
///
/// A diode-connected BJT rung is not a one-port element. The top pair is the
/// two shorted base/collector nodes, and the bottom pair is the two emitter
/// nodes bridged by the rung capacitor. Adjacent rungs share these boundaries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct DifferentialRungPorts {
    pub top_left: NodeId,
    pub top_right: NodeId,
    pub bottom_left: NodeId,
    pub bottom_right: NodeId,
}

impl Block {
    /// All edge indices in this block (NL + linear + reactive).
    pub fn all_edges(&self) -> Vec<usize> {
        let mut edges = Vec::with_capacity(
            self.nl_edges.len() + self.linear_edges.len() + self.reactive_edges.len(),
        );
        edges.extend(&self.nl_edges);
        edges.extend(&self.linear_edges);
        edges.extend(&self.reactive_edges);
        edges
    }
}

/// Result of blockwise decomposition analysis.
#[derive(Debug)]
pub struct BlockwisePlan {
    /// The chained blocks, in signal-flow order.
    pub blocks: Vec<Block>,
    /// Typed residual coupling network.
    pub(super) coupling: CouplingNetwork,
    /// Port nodes shared between blocks and coupling network.
    pub port_nodes: Vec<NodeId>,
}

impl BlockwisePlan {
    pub fn num_blocks(&self) -> usize {
        self.blocks.len()
    }

    fn add_coupling_edge(&mut self, edge_idx: usize, role: CouplingEdgeRole) -> bool {
        self.coupling.push_edge(edge_idx, role)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Step 1: Find NL P-nodes in the SPQR tree
// ═══════════════════════════════════════════════════════════════════════════

/// An NL block found in the SPQR tree: a P-node whose children are
/// NL Q-leaves from the same component.
#[derive(Clone)]
struct NlBlock {
    /// The NL edge indices (from Q-leaf children of the P-node).
    nl_edges: Vec<usize>,
    /// All nodes touched by this block's NL edges.
    nodes: Vec<NodeId>,
    /// Component index of the NL device.
    comp_idx: usize,
}

fn find_root(parent: &mut [usize], idx: usize) -> usize {
    if parent[idx] != idx {
        let root = find_root(parent, parent[idx]);
        parent[idx] = root;
    }
    parent[idx]
}

fn union_roots(parent: &mut [usize], a: usize, b: usize) {
    let ra = find_root(parent, a);
    let rb = find_root(parent, b);
    if ra != rb {
        parent[rb] = ra;
    }
}

fn merge_differential_reactive_rungs(
    nl_blocks: Vec<NlBlock>,
    edge_indices: &[usize],
    graph: &CircuitGraph,
) -> (Vec<NlBlock>, Vec<Vec<usize>>, HashSet<usize>) {
    if nl_blocks.len() < 2 {
        return (nl_blocks, Vec::new(), HashSet::new());
    }

    let mut output_node_to_block: HashMap<NodeId, usize> = HashMap::new();
    for (bi, block) in nl_blocks.iter().enumerate() {
        if let Some((_, emitter)) = diode_connected_bjt_nodes(block.comp_idx, graph) {
            output_node_to_block.insert(emitter, bi);
        }
    }

    let mut parent: Vec<usize> = (0..nl_blocks.len()).collect();
    let mut rung_reactive_edges = Vec::new();

    for &eidx in edge_indices {
        if graph.effective_edge_kind(eidx) != EdgeKind::Reactive {
            continue;
        }
        let edge = &graph.edges[eidx];
        let Some(&a_block) = output_node_to_block.get(&edge.node_a) else {
            continue;
        };
        let Some(&b_block) = output_node_to_block.get(&edge.node_b) else {
            continue;
        };
        if a_block == b_block {
            continue;
        }
        union_roots(&mut parent, a_block, b_block);
        rung_reactive_edges.push((eidx, a_block, b_block));
    }

    if rung_reactive_edges.is_empty() {
        let preassigned = vec![Vec::new(); nl_blocks.len()];
        return (nl_blocks, preassigned, HashSet::new());
    }

    let mut root_to_new: HashMap<usize, usize> = HashMap::new();
    let mut merged = Vec::<NlBlock>::new();
    let mut old_to_new = vec![0usize; nl_blocks.len()];

    for (old_idx, block) in nl_blocks.iter().enumerate() {
        let root = find_root(&mut parent, old_idx);
        let new_idx = *root_to_new.entry(root).or_insert_with(|| {
            let idx = merged.len();
            merged.push(NlBlock {
                nl_edges: Vec::new(),
                nodes: Vec::new(),
                comp_idx: block.comp_idx,
            });
            idx
        });
        old_to_new[old_idx] = new_idx;
        merged[new_idx]
            .nl_edges
            .extend(block.nl_edges.iter().copied());
        for &node in &block.nodes {
            if !merged[new_idx].nodes.contains(&node) {
                merged[new_idx].nodes.push(node);
            }
        }
    }

    let mut preassigned_reactive = vec![Vec::new(); merged.len()];
    let mut preassigned_set = HashSet::new();
    for (eidx, a_block, _) in rung_reactive_edges {
        let new_idx = old_to_new[a_block];
        preassigned_reactive[new_idx].push(eidx);
        preassigned_set.insert(eidx);
        let edge = &graph.edges[eidx];
        for node in [edge.node_a, edge.node_b] {
            if !merged[new_idx].nodes.contains(&node) {
                merged[new_idx].nodes.push(node);
            }
        }
    }

    #[cfg(test)]
    if merged.len() != nl_blocks.len() {
        eprintln!(
            "  [blockwise] merged differential reactive rungs: {} NL blocks → {} rung blocks",
            nl_blocks.len(),
            merged.len()
        );
    }

    (merged, preassigned_reactive, preassigned_set)
}

fn merge_shared_emitter_input_pairs(nl_blocks: Vec<NlBlock>, graph: &CircuitGraph) -> Vec<NlBlock> {
    if nl_blocks.len() < 2 {
        return nl_blocks;
    }

    let mut parent: Vec<usize> = (0..nl_blocks.len()).collect();
    for a in 0..nl_blocks.len() {
        let Some(a_terms) = bjt_terminal_nodes(nl_blocks[a].comp_idx, graph) else {
            continue;
        };
        // Skip fully-diode-connected BJTs — those belong to DifferentialDiodeRung merging.
        // Mixed pairs (one active + one diode-connected, shared emitter) are valid
        // InputDifferentialPair candidates: merge them here so classify_block_topology
        // sees both transistors in a single 2-NL block.
        for b in (a + 1)..nl_blocks.len() {
            let Some(b_terms) = bjt_terminal_nodes(nl_blocks[b].comp_idx, graph) else {
                continue;
            };
            let a_diode_connected = a_terms.base == a_terms.collector;
            let b_diode_connected = b_terms.base == b_terms.collector;
            // Skip fully-diode-connected pairs — those are ladder rungs, handled elsewhere.
            if a_diode_connected && b_diode_connected {
                continue;
            }
            // Require shared emitter with distinct base/collector signal paths.
            if a_terms.emitter == b_terms.emitter
                && a_terms.base != b_terms.base
                && a_terms.collector != b_terms.collector
            {
                union_roots(&mut parent, a, b);
            }
        }
    }

    let mut root_to_new: HashMap<usize, usize> = HashMap::new();
    let mut merged = Vec::<NlBlock>::new();
    for (old_idx, block) in nl_blocks.iter().enumerate() {
        let root = find_root(&mut parent, old_idx);
        let new_idx = *root_to_new.entry(root).or_insert_with(|| {
            let idx = merged.len();
            merged.push(NlBlock {
                nl_edges: Vec::new(),
                nodes: Vec::new(),
                comp_idx: block.comp_idx,
            });
            idx
        });
        merged[new_idx]
            .nl_edges
            .extend(block.nl_edges.iter().copied());
        for &node in &block.nodes {
            if !merged[new_idx].nodes.contains(&node) {
                merged[new_idx].nodes.push(node);
            }
        }
    }

    #[cfg(test)]
    if merged.len() != nl_blocks.len() {
        eprintln!(
            "  [blockwise] merged shared-emitter input pairs: {} NL blocks → {} blocks",
            nl_blocks.len(),
            merged.len()
        );
    }

    merged
}

fn component_pin_node(graph: &CircuitGraph, comp_id: &str, pin: &str) -> Option<NodeId> {
    graph.node_names.get(&format!("{comp_id}.{pin}")).copied()
}

fn nl_block_direction_nodes(block: &NlBlock, graph: &CircuitGraph) -> Option<(NodeId, NodeId)> {
    let comp = &graph.components[block.comp_idx];
    match comp.kind.signal_terminals() {
        super::component::SignalTerminals::Amplifier { input, output, .. } => {
            let input_node = component_pin_node(graph, &comp.id, input)?;
            let output_node = component_pin_node(graph, &comp.id, output)?;
            let common_node = comp
                .kind
                .pin_config()
                .valid_pins
                .iter()
                .find(|&&p| p != input && p != output)
                .and_then(|&pin| component_pin_node(graph, &comp.id, pin));

            // Diode-connected transistor ladder rungs short the declared
            // amplifier input/output pins. In a differential diode ladder the
            // signal current enters at the common terminal and propagates to
            // the shorted base/collector node of the next rung, so the
            // compiler must order these as emitter -> base/collector. This is
            // a topology rule, not a TB303 naming rule.
            if input_node == output_node {
                let common_node = common_node?;
                if block.nodes.contains(&input_node) && block.nodes.contains(&common_node) {
                    return Some((common_node, input_node));
                }
            }

            if block.nodes.contains(&input_node) && block.nodes.contains(&output_node) {
                Some((input_node, output_node))
            } else {
                None
            }
        }
        super::component::SignalTerminals::TwoPort { input, output } => {
            let input_node = component_pin_node(graph, &comp.id, input)?;
            let output_node = component_pin_node(graph, &comp.id, output)?;
            if block.nodes.contains(&input_node) && block.nodes.contains(&output_node) {
                Some((input_node, output_node))
            } else {
                None
            }
        }
        super::component::SignalTerminals::Passive => None,
    }
}

fn diode_connected_bjt_nodes(comp_idx: usize, graph: &CircuitGraph) -> Option<(NodeId, NodeId)> {
    let terms = bjt_terminal_nodes(comp_idx, graph)?;
    (terms.base == terms.collector).then_some((terms.base, terms.emitter))
}

#[derive(Debug, Clone, Copy)]
struct BjtTerminalNodes {
    base: NodeId,
    collector: NodeId,
    emitter: NodeId,
}

fn bjt_terminal_nodes(comp_idx: usize, graph: &CircuitGraph) -> Option<BjtTerminalNodes> {
    let comp = &graph.components[comp_idx];
    let eidx = graph
        .edges
        .iter()
        .position(|edge| edge.comp_idx == comp_idx)?;
    let edge = &graph.edges[eidx];
    let (kind, _) = comp.kind.classify_nonlinear(
        &comp.id,
        edge.node_a,
        edge.node_b,
        graph.gnd_node,
        &graph.node_names,
    )?;

    match kind {
        super::classify::NonlinearKind::BjtNpn {
            base_node,
            collector_node,
            emitter_node,
            ..
        }
        | super::classify::NonlinearKind::BjtPnp {
            base_node,
            collector_node,
            emitter_node,
            ..
        } => Some(BjtTerminalNodes {
            base: base_node,
            collector: collector_node,
            emitter: emitter_node,
        }),
        _ => None,
    }
}

fn classify_block_topology(
    nl_edges: &[usize],
    reactive_edges: &[usize],
    graph: &CircuitGraph,
) -> BlockTopology {
    let mut comp_indices: Vec<usize> = nl_edges
        .iter()
        .map(|&eidx| graph.edges[eidx].comp_idx)
        .collect();
    comp_indices.sort_unstable();
    comp_indices.dedup();

    if comp_indices.len() != 2 {
        return BlockTopology::Generic;
    }

    if let (Some(left), Some(right)) = (
        bjt_terminal_nodes(comp_indices[0], graph),
        bjt_terminal_nodes(comp_indices[1], graph),
    ) {
        let left_diode_connected = left.base == left.collector;
        let right_diode_connected = right.base == right.collector;
        // Both-active pair (fully differential) or mixed active+diode-connected pair:
        // as long as the two BJTs share an emitter node and have distinct base/collector
        // signal paths, this is an InputDifferentialPair. The diode-connected side's
        // base == collector is legal — it acts as a current-mirror reference whose
        // base and collector are tied together.
        if !left_diode_connected || !right_diode_connected {
            if left.emitter == right.emitter
                && left.base != right.base
                && left.collector != right.collector
            {
                return BlockTopology::InputDifferentialPair {
                    left_comp_idx: comp_indices[0],
                    right_comp_idx: comp_indices[1],
                    shared_emitter: left.emitter,
                };
            }
        }
    }

    let Some((_, left_emitter)) = diode_connected_bjt_nodes(comp_indices[0], graph) else {
        return BlockTopology::Generic;
    };
    let Some((_, right_emitter)) = diode_connected_bjt_nodes(comp_indices[1], graph) else {
        return BlockTopology::Generic;
    };

    for &eidx in reactive_edges {
        let edge = &graph.edges[eidx];
        let connects_emitters = (edge.node_a == left_emitter && edge.node_b == right_emitter)
            || (edge.node_a == right_emitter && edge.node_b == left_emitter);
        if connects_emitters {
            return BlockTopology::DifferentialDiodeRung {
                left_comp_idx: comp_indices[0],
                right_comp_idx: comp_indices[1],
                reactive_edge: eidx,
            };
        }
    }

    BlockTopology::Generic
}

pub(super) fn differential_rung_ports(
    block: &Block,
    graph: &CircuitGraph,
) -> Option<DifferentialRungPorts> {
    let BlockTopology::DifferentialDiodeRung {
        left_comp_idx,
        right_comp_idx,
        ..
    } = block.topology
    else {
        return None;
    };

    let (left_top, left_emitter) = diode_connected_bjt_nodes(left_comp_idx, graph)?;
    let (right_top, right_emitter) = diode_connected_bjt_nodes(right_comp_idx, graph)?;
    Some(DifferentialRungPorts {
        top_left: left_top,
        top_right: right_top,
        bottom_left: left_emitter,
        bottom_right: right_emitter,
    })
}

fn differential_rung_port_nodes(block: &Block, graph: &CircuitGraph) -> Option<(NodeId, NodeId)> {
    let ports = differential_rung_ports(block, graph)?;
    Some((ports.bottom_left, ports.bottom_right))
}

fn differential_rung_bottom_to_top_order(
    blocks: &[Block],
    graph: &CircuitGraph,
) -> Option<Vec<usize>> {
    if blocks.is_empty() {
        return None;
    }

    let ports: Vec<DifferentialRungPorts> = blocks
        .iter()
        .map(|block| differential_rung_ports(block, graph))
        .collect::<Option<Vec<_>>>()?;

    let top_pairs: HashSet<(NodeId, NodeId)> = ports
        .iter()
        .map(|port| (port.top_left, port.top_right))
        .collect();
    let mut starts: Vec<usize> = ports
        .iter()
        .enumerate()
        .filter_map(|(idx, port)| {
            let bottom = (port.bottom_left, port.bottom_right);
            (!top_pairs.contains(&bottom)).then_some(idx)
        })
        .collect();
    if starts.len() != 1 {
        return None;
    }

    let mut order = Vec::with_capacity(blocks.len());
    let mut used = HashSet::new();
    let mut current = starts.remove(0);
    loop {
        order.push(current);
        used.insert(current);
        let current_top = (ports[current].top_left, ports[current].top_right);
        let next = ports.iter().enumerate().find_map(|(idx, port)| {
            let bottom = (port.bottom_left, port.bottom_right);
            (!used.contains(&idx) && bottom == current_top).then_some(idx)
        });
        let Some(next) = next else {
            break;
        };
        current = next;
    }

    (order.len() == blocks.len()).then_some(order)
}

fn bjt_model_name_for_comp(comp_idx: usize, graph: &CircuitGraph) -> Option<String> {
    let comp = &graph.components[comp_idx];
    let eidx = graph
        .edges
        .iter()
        .position(|edge| edge.comp_idx == comp_idx)?;
    let edge = &graph.edges[eidx];
    let (kind, _) = comp.kind.classify_nonlinear(
        &comp.id,
        edge.node_a,
        edge.node_b,
        graph.gnd_node,
        &graph.node_names,
    )?;

    match kind {
        super::classify::NonlinearKind::BjtNpn { model_name, .. }
        | super::classify::NonlinearKind::BjtPnp { model_name, .. } => Some(model_name),
        _ => None,
    }
}

fn build_differential_diode_rung_stage(
    block: &Block,
    graph: &CircuitGraph,
    sample_rate: f64,
    supply_voltage: f64,
) -> Option<pedalkernel_rt::stage::WdfStage> {
    let BlockTopology::DifferentialDiodeRung {
        left_comp_idx,
        reactive_edge,
        ..
    } = block.topology
    else {
        return None;
    };

    let cap_comp = &graph.components[graph.edges[reactive_edge].comp_idx];
    let capacitance = cap_comp.kind.capacitance()?;
    let cap_rp = 1.0 / (2.0 * sample_rate * capacitance);
    let cap = super::dyn_node::DynNode::Capacitor(Some(cap_comp.id.clone()), capacitance, cap_rp);
    let tree = super::spqr_build::with_voltage_source_rp(cap, 1_000.0);

    let model_name = bjt_model_name_for_comp(left_comp_idx, graph)?;
    let model = super::helpers::gummel_poon_model(&model_name);

    let bias_resistance = block
        .linear_edges
        .iter()
        .filter_map(|&eidx| {
            graph.components[graph.edges[eidx].comp_idx]
                .kind
                .resistance()
        })
        .find(|r| *r > 0.0)
        .unwrap_or(100_000.0);
    let i_tail = ((supply_voltage - 0.6).max(0.1) / bias_resistance).max(1.0e-9);
    let root = pedalkernel_rt::stage::RootKind::DiffPair(
        pedalkernel_rt::elements::DiffPairRoot::from_gummel_poon(&model, i_tail),
    );
    let oversampler = pedalkernel_rt::oversampling::Oversampler::new(
        pedalkernel_rt::oversampling::OversamplingFactor::X1,
    );

    let mut stage = pedalkernel_rt::stage::WdfStage::new(tree, root, oversampler);
    stage.output_probe = Some(cap_comp.id.clone());
    stage.root_comp_id = graph.components[left_comp_idx].id.clone();
    Some(stage)
}

fn build_input_differential_pair_stage(
    block: &Block,
    graph: &CircuitGraph,
    supply_voltage: f64,
) -> Option<pedalkernel_rt::stage::WdfStage> {
    let BlockTopology::InputDifferentialPair {
        left_comp_idx,
        right_comp_idx,
        shared_emitter,
    } = block.topology
    else {
        return None;
    };

    let left = bjt_terminal_nodes(left_comp_idx, graph)?;
    let right = bjt_terminal_nodes(right_comp_idx, graph)?;
    let model_name = bjt_model_name_for_comp(left_comp_idx, graph)?;
    let model = super::helpers::gummel_poon_model(&model_name);
    let i_tail = (supply_voltage.max(1.0) / 100_000.0).max(1.0e-9);
    let root = pedalkernel_rt::stage::RootKind::DiffPair(
        pedalkernel_rt::elements::DiffPairRoot::from_gummel_poon(&model, i_tail),
    );
    let oversampler = pedalkernel_rt::oversampling::Oversampler::new(
        pedalkernel_rt::oversampling::OversamplingFactor::X1,
    );
    let mut stage = pedalkernel_rt::stage::WdfStage::new(
        pedalkernel_rt::dyn_node::DynNode::VoltageSource(0.0, 1_000.0),
        root,
        oversampler,
    );
    stage.root_comp_id = graph.components[left_comp_idx].id.clone();
    stage.bypass_serial = true;
    #[cfg(debug_assertions)]
    {
        stage.debug_label = format!(
            "{}/{} input differential pair",
            graph.components[left_comp_idx].id, graph.components[right_comp_idx].id
        );
    }
    stage.boundary_bindings = vec![
        pedalkernel_rt::stage::WdfBoundaryBinding {
            label: "base_left".into(),
            node_id: left.base,
            direction: pedalkernel_rt::stage::WdfBoundaryDirection::Input,
        },
        pedalkernel_rt::stage::WdfBoundaryBinding {
            label: "base_right".into(),
            node_id: right.base,
            direction: pedalkernel_rt::stage::WdfBoundaryDirection::Input,
        },
        pedalkernel_rt::stage::WdfBoundaryBinding {
            label: "collector_left".into(),
            node_id: left.collector,
            direction: pedalkernel_rt::stage::WdfBoundaryDirection::Output,
        },
        pedalkernel_rt::stage::WdfBoundaryBinding {
            label: "collector_right".into(),
            node_id: right.collector,
            direction: pedalkernel_rt::stage::WdfBoundaryDirection::Output,
        },
        pedalkernel_rt::stage::WdfBoundaryBinding {
            label: "emitter_tail".into(),
            node_id: shared_emitter,
            direction: pedalkernel_rt::stage::WdfBoundaryDirection::Control,
        },
    ];
    Some(stage)
}

fn build_diode_ladder_core(
    blocks: &[Block],
    graph: &CircuitGraph,
    sample_rate: f64,
    supply_voltage: f64,
) -> Option<pedalkernel_rt::stage::DiodeLadderCore> {
    if blocks.is_empty()
        || !blocks
            .iter()
            .all(|block| matches!(block.topology, BlockTopology::DifferentialDiodeRung { .. }))
    {
        return None;
    }

    let mut cap_values = Vec::with_capacity(blocks.len());
    let mut first_left_comp_idx = None;
    let mut first_bias_resistance = None;
    for block in blocks {
        let BlockTopology::DifferentialDiodeRung {
            left_comp_idx,
            reactive_edge,
            ..
        } = block.topology
        else {
            return None;
        };
        first_left_comp_idx.get_or_insert(left_comp_idx);
        let cap_comp = &graph.components[graph.edges[reactive_edge].comp_idx];
        cap_values.push(cap_comp.kind.capacitance()?);
        if first_bias_resistance.is_none() {
            first_bias_resistance = block
                .linear_edges
                .iter()
                .filter_map(|&eidx| {
                    graph.components[graph.edges[eidx].comp_idx]
                        .kind
                        .resistance()
                })
                .find(|r| *r > 0.0);
        }
    }

    let model_name = bjt_model_name_for_comp(first_left_comp_idx?, graph)?;
    let model = super::helpers::gummel_poon_model(&model_name);
    let alpha = model.bf / (model.bf + 1.0);
    let n_vt = model.nf * model.vt;
    let cutoff_bias_resistance = first_bias_resistance.unwrap_or(100_000.0);
    let cutoff_max_resistance = graph
        .components
        .iter()
        .find_map(|comp| {
            let lower = comp.id.to_ascii_lowercase();
            if !lower.contains("cutoff") {
                return None;
            }
            comp.kind
                .as_any()
                .downcast_ref::<super::components::Potentiometer>()
                .map(|pot| pot.max_r)
        })
        .unwrap_or(100_000.0);
    let r_min = cutoff_bias_resistance.max(1.0);
    let r_max = (cutoff_bias_resistance + cutoff_max_resistance)
        .max(r_min * 1.01)
        .max(1.0);
    let v_min = 0.0;
    let v_max = (supply_voltage + 5.0).max(supply_voltage * 1.01).max(1.0);
    let tail_current_table = generate_diode_ladder_tail_current_table(
        blocks.len(),
        n_vt,
        model.is,
        r_min,
        r_max,
        v_min,
        v_max,
    );
    let i_tail_bias = tail_current_table.lookup_2d(
        (cutoff_bias_resistance + cutoff_max_resistance * 0.5).clamp(r_min, r_max),
        supply_voltage.clamp(v_min, v_max),
    );
    let i_tail_max = tail_current_table
        .lookup_2d(r_min, v_max)
        .max(i_tail_bias * 1.01);
    let tanh_table = super::k_method::generate_differential_ladder_tanh_table(n_vt);

    Some(pedalkernel_rt::stage::DiodeLadderCore::new(
        tanh_table,
        cap_values,
        sample_rate,
        alpha,
        n_vt,
        tail_current_table,
        i_tail_bias,
        i_tail_max,
        cutoff_bias_resistance,
    ))
}

fn generate_diode_ladder_tail_current_table(
    diode_count: usize,
    n_vt: f64,
    diode_is: f64,
    r_min: f64,
    r_max: f64,
    v_min: f64,
    v_max: f64,
) -> pedalkernel_rt::stage::KTable {
    let steps = 64;
    let diode_count = diode_count.max(1) as f64;
    let n_vt = n_vt.max(1.0e-6);
    let diode_is = diode_is.max(1.0e-18);
    let mut entries = Vec::with_capacity(steps * steps);

    for iv in 0..steps {
        let v_t = iv as f64 / (steps - 1) as f64;
        let available_voltage = v_min + v_t * (v_max - v_min);
        for ir in 0..steps {
            let r_t = ir as f64 / (steps - 1) as f64;
            let series_resistance = r_min + r_t * (r_max - r_min);
            entries.push(solve_diode_ladder_tail_current(
                diode_count,
                n_vt,
                diode_is,
                available_voltage,
                series_resistance,
            ) as pedalkernel_rt::Wave);
        }
    }

    let mut table = pedalkernel_rt::stage::KTable {
        dims: 2,
        b_min: r_min,
        b_max: r_max,
        ctrl_min: v_min,
        ctrl_max: v_max,
        steps,
        entries,
        inv_b_scale: 0.0,
        inv_c_scale: 0.0,
    };
    table.precompute_scales();
    table
}

fn solve_diode_ladder_tail_current(
    diode_count: f64,
    n_vt: f64,
    diode_is: f64,
    available_voltage: f64,
    series_resistance: f64,
) -> f64 {
    if available_voltage <= 0.0 {
        return 1.0e-9;
    }

    let r = series_resistance.max(1.0);
    let max_ohmic_current = (available_voltage / r).max(1.0e-9);
    let mut lo = 1.0e-9_f64.min(max_ohmic_current);
    let mut hi = max_ohmic_current.max(lo * 1.01);

    // Compile-time DC operating-point analysis for the shared-current ladder.
    // Runtime receives this as a table and only interpolates it.
    for _ in 0..64 {
        let mid = 0.5 * (lo + hi);
        let diode_current = (mid * 0.5).max(1.0e-18);
        let diode_drop = diode_count * n_vt * (1.0 + diode_current / diode_is).ln();
        let used_voltage = mid * r + diode_drop;
        if used_voltage > available_voltage {
            hi = mid;
        } else {
            lo = mid;
        }
    }

    (0.5 * (lo + hi)).max(1.0e-9)
}

pub(super) fn block_coupling_port_node(
    nl_edges: &[usize],
    port_nodes: &[NodeId],
    graph: &CircuitGraph,
) -> Option<NodeId> {
    let comp_idx = nl_edges.first().map(|&eidx| graph.edges[eidx].comp_idx)?;
    let comp = &graph.components[comp_idx];
    let (input, output) = match comp.kind.signal_terminals() {
        super::component::SignalTerminals::Amplifier { input, output, .. }
        | super::component::SignalTerminals::TwoPort { input, output } => (input, output),
        super::component::SignalTerminals::Passive => return None,
    };

    let input_node = component_pin_node(graph, &comp.id, input)?;
    let output_node = component_pin_node(graph, &comp.id, output)?;

    // For diode-connected transistors, input and output pins are shorted.
    // This is still the driven side of the one-port; the common pin
    // (emitter/source/cathode) is the cascade output and must not be used as
    // the coupling matrix port.
    if input_node == output_node && port_nodes.contains(&input_node) {
        return Some(input_node);
    }

    if port_nodes.contains(&input_node) {
        Some(input_node)
    } else {
        None
    }
}

fn block_cascade_output_node(
    nl_edges: &[usize],
    port_nodes: &[NodeId],
    graph: &CircuitGraph,
) -> Option<NodeId> {
    let comp_idx = nl_edges.first().map(|&eidx| graph.edges[eidx].comp_idx)?;
    let comp = &graph.components[comp_idx];
    let (input, output, is_amplifier) = match comp.kind.signal_terminals() {
        super::component::SignalTerminals::Amplifier { input, output, .. } => (input, output, true),
        super::component::SignalTerminals::TwoPort { input, output } => (input, output, false),
        super::component::SignalTerminals::Passive => return None,
    };

    let input_node = component_pin_node(graph, &comp.id, input)?;
    let output_node = component_pin_node(graph, &comp.id, output)?;

    if is_amplifier && input_node == output_node {
        return comp
            .kind
            .pin_config()
            .valid_pins
            .iter()
            .find(|&&p| p != input && p != output)
            .and_then(|&pin| component_pin_node(graph, &comp.id, pin))
            .filter(|node| port_nodes.contains(node));
    }

    if port_nodes.contains(&output_node) {
        Some(output_node)
    } else {
        None
    }
}

fn signal_order_indices(nl_blocks: &[NlBlock], graph: &CircuitGraph) -> Option<Vec<usize>> {
    let dirs: Vec<Option<(NodeId, NodeId)>> = nl_blocks
        .iter()
        .map(|block| nl_block_direction_nodes(block, graph))
        .collect();
    if dirs.iter().any(Option::is_none) {
        return None;
    }
    let dirs: Vec<(NodeId, NodeId)> = dirs.into_iter().map(Option::unwrap).collect();

    let downstream_nodes: HashSet<NodeId> =
        dirs.iter().map(|(_, downstream)| *downstream).collect();
    let mut start_candidates: Vec<usize> = dirs
        .iter()
        .enumerate()
        .filter_map(|(i, (upstream, _))| (!downstream_nodes.contains(upstream)).then_some(i))
        .collect();

    if start_candidates.is_empty() {
        return None;
    }

    start_candidates.sort_by_key(|&i| {
        let (upstream, _) = dirs[i];
        let touches_input =
            nl_blocks[i].nodes.contains(&graph.in_node) || upstream == graph.in_node;
        (!touches_input, i)
    });

    let mut ordered = Vec::with_capacity(nl_blocks.len());
    let mut used = HashSet::new();
    let mut current = start_candidates[0];

    loop {
        ordered.push(current);
        used.insert(current);

        let (_, downstream) = dirs[current];
        let mut next: Vec<usize> = dirs
            .iter()
            .enumerate()
            .filter_map(|(i, (upstream, _))| {
                (!used.contains(&i) && *upstream == downstream).then_some(i)
            })
            .collect();
        if next.is_empty() {
            break;
        }
        next.sort_unstable();
        current = next[0];
    }

    if ordered.len() == nl_blocks.len() {
        Some(ordered)
    } else {
        None
    }
}

/// Recursively extract all NL Q-leaf edge indices from a subtree.
/// Skips P-nodes that are all-NL-same-comp (those are found by
/// find_nl_blocks pattern 1 and should not be broken up).
fn extract_nl_q_leaves(node: &SpqrNode, graph: &CircuitGraph, out: &mut Vec<usize>) {
    match node {
        SpqrNode::Q { edge_idx, .. } => {
            if graph.effective_edge_kind(*edge_idx) == EdgeKind::Nonlinear {
                out.push(*edge_idx);
            }
        }
        SpqrNode::P { children, .. } => {
            // Check if this is an all-NL-same-comp P-node (pattern 1 candidate)
            let all_nl_same = {
                let mut ci = None;
                let mut ok = true;
                for c in children {
                    if let SpqrNode::Q { edge_idx, .. } = c {
                        if graph.effective_edge_kind(*edge_idx) == EdgeKind::Nonlinear {
                            let c2 = graph.edges[*edge_idx].comp_idx;
                            if let Some(prev) = ci {
                                if prev != c2 {
                                    ok = false;
                                }
                            } else {
                                ci = Some(c2);
                            }
                        } else {
                            ok = false;
                        }
                    } else {
                        ok = false;
                    }
                }
                ok && ci.is_some()
            };
            if !all_nl_same {
                // Mixed P-node (pendant wrapper) — extract NL Q-leaves
                for c in children {
                    extract_nl_q_leaves(c, graph, out);
                }
            }
            // all-NL-same-comp → skip, handled by find_nl_blocks pattern 1
        }
        _ => {} // S/R nodes are structural, don't descend
    }
}

/// Scan children for NL Q-leaves, grouping by component. Recursively
/// looks through P-node pendant wrappers at any depth. Then recurse
/// into non-Q children that weren't consumed.
fn collect_nl_q_children(children: &[SpqrNode], graph: &CircuitGraph, blocks: &mut Vec<NlBlock>) {
    let mut comp_groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    let mut consumed: Vec<usize> = Vec::new(); // indices of children fully consumed

    for (i, child) in children.iter().enumerate() {
        let mut nl_leaves = Vec::new();
        extract_nl_q_leaves(child, graph, &mut nl_leaves);
        if !nl_leaves.is_empty() {
            consumed.push(i);
            for eidx in nl_leaves {
                let ci = graph.edges[eidx].comp_idx;
                comp_groups.entry(ci).or_default().push(eidx);
            }
        }
    }

    for (ci, nl_edges) in comp_groups {
        if nl_edges.is_empty() {
            continue;
        }
        let mut all_nodes: Vec<NodeId> = Vec::new();
        for &eidx in &nl_edges {
            let e = &graph.edges[eidx];
            if !all_nodes.contains(&e.node_a) {
                all_nodes.push(e.node_a);
            }
            if !all_nodes.contains(&e.node_b) {
                all_nodes.push(e.node_b);
            }
        }
        if all_nodes.is_empty() {
            continue;
        }
        blocks.push(NlBlock {
            nl_edges,
            nodes: all_nodes,
            comp_idx: ci,
        });
    }

    // Recurse into children that weren't consumed
    for (i, child) in children.iter().enumerate() {
        if consumed.contains(&i) {
            continue;
        }
        match child {
            SpqrNode::Q { .. } => {} // passive Q, not relevant
            _ => find_nl_blocks(child, graph, blocks),
        }
    }
}

/// Walk the SPQR tree and collect NL blocks.
///
/// Two patterns:
/// 1. **P-node**: children are parallel NL edges from the same component
///    (diode-connected BJT: base=collector → same endpoints)
/// 2. **S-node siblings**: Q-leaf NL edges from the same component that
///    are siblings in an S-node (CE BJT: B-E and C-E share emitter node
///    but have different other endpoints)
fn find_nl_blocks(node: &SpqrNode, graph: &CircuitGraph, blocks: &mut Vec<NlBlock>) {
    match node {
        SpqrNode::P {
            children,
            endpoints,
        } => {
            // Pattern 1: P-node with all-NL children from same component
            let mut nl_edges = Vec::new();
            let mut comp_idx = None;
            let mut all_nl_same_comp = true;

            for child in children {
                if let SpqrNode::Q { edge_idx, .. } = child {
                    if graph.effective_edge_kind(*edge_idx) == EdgeKind::Nonlinear {
                        let ci = graph.edges[*edge_idx].comp_idx;
                        if let Some(existing) = comp_idx {
                            if existing != ci {
                                all_nl_same_comp = false;
                            }
                        } else {
                            comp_idx = Some(ci);
                        }
                        nl_edges.push(*edge_idx);
                    } else {
                        all_nl_same_comp = false;
                    }
                } else {
                    all_nl_same_comp = false;
                }
            }

            if all_nl_same_comp && !nl_edges.is_empty() {
                if let Some(ci) = comp_idx {
                    blocks.push(NlBlock {
                        nl_edges,
                        nodes: vec![endpoints.0, endpoints.1],
                        comp_idx: ci,
                    });
                    return;
                }
            }

            // Fallback: mixed P-node. Scan Q children for NL edges
            // (same as S-node pattern), then recurse into non-Q children.
            collect_nl_q_children(children, graph, blocks);
        }

        SpqrNode::S { children, .. } | SpqrNode::R { children, .. } => {
            // Pattern 2: group NL Q-leaf siblings by comp_idx,
            // then recurse into non-Q children.
            collect_nl_q_children(children, graph, blocks);
        }

        SpqrNode::Q { .. } => {} // Single leaf — not a block
    }
}

fn add_missing_nl_component_blocks(
    edge_indices: &[usize],
    graph: &CircuitGraph,
    blocks: &mut Vec<NlBlock>,
) {
    let covered_edges: HashSet<usize> = blocks
        .iter()
        .flat_map(|block| block.nl_edges.iter().copied())
        .collect();
    let mut by_comp: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for &eidx in edge_indices {
        if graph.effective_edge_kind(eidx) != EdgeKind::Nonlinear || covered_edges.contains(&eidx) {
            continue;
        }
        by_comp
            .entry(graph.edges[eidx].comp_idx)
            .or_default()
            .push(eidx);
    }

    for (comp_idx, nl_edges) in by_comp {
        let mut nodes = Vec::new();
        for &eidx in &nl_edges {
            let edge = &graph.edges[eidx];
            for node in [edge.node_a, edge.node_b] {
                if !nodes.contains(&node) {
                    nodes.push(node);
                }
            }
        }
        if nodes.is_empty() {
            continue;
        }
        #[cfg(test)]
        eprintln!(
            "  [blockwise] synthesized missing NL block for {} ({} edges)",
            graph.components[comp_idx].id,
            nl_edges.len()
        );
        blocks.push(NlBlock {
            nl_edges,
            nodes,
            comp_idx,
        });
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Step 2: Collect sibling Q-leaves (non-NL edges)
// ═══════════════════════════════════════════════════════════════════════════

/// Collect all Q-leaf edge indices that are NOT part of any NL block.
fn collect_sibling_edges(node: &SpqrNode, nl_edge_set: &HashSet<usize>, siblings: &mut Vec<usize>) {
    match node {
        SpqrNode::Q { edge_idx, .. } => {
            if !nl_edge_set.contains(edge_idx) {
                siblings.push(*edge_idx);
            }
        }
        SpqrNode::S { children, .. } | SpqrNode::P { children, .. } => {
            // Check if this P-node is an NL block (all children are NL from same comp)
            // If so, skip — its edges are already in nl_edge_set
            let is_nl_p = matches!(node, SpqrNode::P { .. })
                && children.iter().all(
                    |c| matches!(c, SpqrNode::Q { edge_idx, .. } if nl_edge_set.contains(edge_idx)),
                );
            if !is_nl_p {
                for child in children {
                    collect_sibling_edges(child, nl_edge_set, siblings);
                }
            }
        }
        SpqrNode::R {
            children,
            edge_indices,
            ..
        } => {
            // R-node edges that aren't NL
            for &eidx in edge_indices {
                if !nl_edge_set.contains(&eidx) {
                    siblings.push(eidx);
                }
            }
            for child in children {
                collect_sibling_edges(child, nl_edge_set, siblings);
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Step 3: Assign siblings to blocks by port node adjacency
// ═══════════════════════════════════════════════════════════════════════════

// (assign_sibling replaced by iterative node_to_block expansion in analyze_blockwise)

// ═══════════════════════════════════════════════════════════════════════════
// Step 4: Validation
// ═══════════════════════════════════════════════════════════════════════════

fn validate_plan(plan: &BlockwisePlan, graph: &CircuitGraph) -> bool {
    if plan.blocks.len() < 2 {
        return false;
    }
    let has_reactive_state = plan
        .blocks
        .iter()
        .any(|block| !block.reactive_edges.is_empty())
        || plan
            .coupling
            .iter_edge_indices()
            .any(|eidx| graph.effective_edge_kind(eidx) == EdgeKind::Reactive);
    if !has_reactive_state {
        return false;
    }
    for block in &plan.blocks {
        if block.nl_edges.is_empty() {
            return false;
        }
    }
    // Coupling must not contain NL
    for eidx in plan.coupling.iter_edge_indices() {
        if graph.effective_edge_kind(eidx) == EdgeKind::Nonlinear {
            return false;
        }
    }
    true
}

// ═══════════════════════════════════════════════════════════════════════════
// Public entry point: analyze
// ═══════════════════════════════════════════════════════════════════════════

/// Analyze a set of edges for blockwise decomposition using the SPQR tree.
///
/// Returns `Some(BlockwisePlan)` if the circuit splits into ≥2 NL blocks
/// with sparse coupling. Returns `None` otherwise.
pub(super) fn analyze_blockwise(
    edge_indices: &[usize],
    graph: &CircuitGraph,
) -> Option<BlockwisePlan> {
    // Build SPQR tree for these edges.
    // Terminals may be graph.in_node/out_node (which might be port nodes
    // outside this subgraph). The SPQR series chain walk handles this safely
    // via visited-set cycle detection — no infinite loop even when terminals
    // are absent from the subgraph.
    let terminals = vec![graph.in_node, graph.out_node];
    eprintln!(
        "  [blockwise] analyze: SPQR decompose ({} edges, terminals={terminals:?})...",
        edge_indices.len()
    );
    let tree = spqr_decompose(edge_indices, &terminals, graph, graph.gnd_node);
    eprintln!("  [blockwise] analyze: SPQR done, finding NL blocks...");

    // Step 1: find NL blocks (P-nodes or S-node siblings)
    let mut nl_blocks = Vec::new();
    find_nl_blocks(&tree, graph, &mut nl_blocks);
    add_missing_nl_component_blocks(edge_indices, graph, &mut nl_blocks);

    eprintln!("  [blockwise] analyze: found {} NL blocks", nl_blocks.len());

    if nl_blocks.len() < 2 {
        return None;
    }

    let nl_blocks = merge_shared_emitter_input_pairs(nl_blocks, graph);
    let (nl_blocks, preassigned_reactive, preassigned_reactive_set) =
        merge_differential_reactive_rungs(nl_blocks, edge_indices, graph);

    // Build NL edge set and port node map
    let nl_edge_set: HashSet<usize> = nl_blocks
        .iter()
        .flat_map(|b| b.nl_edges.iter().copied())
        .collect();

    let block_port_nodes: Vec<(usize, Vec<NodeId>)> = nl_blocks
        .iter()
        .enumerate()
        .map(|(i, b)| (i, b.nodes.clone()))
        .collect();

    let excluded: HashSet<NodeId> = {
        let mut s = HashSet::new();
        s.insert(graph.gnd_node);
        s.insert(graph.vcc_node);
        for &n in &graph.supply_nodes {
            s.insert(n);
        }
        for &n in &graph.ac_ground_nodes {
            s.insert(n);
        }
        s
    };
    let external_terminal_nodes: HashSet<NodeId> = graph
        .node_names
        .iter()
        .filter_map(|(name, &node)| {
            let is_reserved = matches!(name.as_str(), "in" | "out" | "gnd" | "vcc");
            (!is_reserved && !name.contains('.')).then_some(node)
        })
        .collect();

    // Step 2: collect non-NL edges — everything that isn't in an NL block
    let sibling_edges: Vec<usize> = edge_indices
        .iter()
        .filter(|e| !nl_edge_set.contains(e) && !preassigned_reactive_set.contains(e))
        .copied()
        .collect();

    #[cfg(test)]
    eprintln!(
        "  siblings: {} edges (of {} total, {} NL)",
        sibling_edges.len(),
        edge_indices.len(),
        nl_edge_set.len()
    );

    // Step 3: assign siblings to blocks.
    //
    // Node classification:
    // - A port node appearing in exactly 1 block → owned by that block
    // - A port node appearing in 2+ blocks → boundary (coupling interface)
    //   Neither block owns it. Edges here are assigned by their OTHER endpoint.
    // - Ground/supply → excluded (not owned, not boundary)
    //
    // Then iteratively expand: edges where one endpoint is owned and the
    // other is unclaimed → assign to the owning block, claim the other node.

    // Find boundary nodes (shared between blocks)
    let mut node_block_count: HashMap<NodeId, HashSet<usize>> = HashMap::new();
    for (bi, nodes) in &block_port_nodes {
        for &node in nodes {
            if excluded.contains(&node) {
                continue;
            }
            node_block_count.entry(node).or_default().insert(*bi);
        }
    }
    let boundary_nodes: HashSet<NodeId> = node_block_count
        .iter()
        .filter(|(_, blocks)| blocks.len() > 1)
        .map(|(&node, _)| node)
        .collect();

    // Seed node_to_block with non-boundary port nodes
    let mut node_to_block: HashMap<NodeId, usize> = HashMap::new();
    for (bi, nodes) in &block_port_nodes {
        for &node in nodes {
            if excluded.contains(&node) || boundary_nodes.contains(&node) {
                continue;
            }
            node_to_block.entry(node).or_insert(*bi);
        }
    }

    let mut block_linear: Vec<Vec<usize>> = vec![Vec::new(); nl_blocks.len()];
    let mut block_reactive: Vec<Vec<usize>> = if preassigned_reactive.len() == nl_blocks.len() {
        preassigned_reactive
    } else {
        vec![Vec::new(); nl_blocks.len()]
    };
    #[cfg(test)]
    eprintln!("  node_to_block seed: {:?}", node_to_block);

    let mut coupling_edges = Vec::new();

    // Pre-classify: find edges on paths connecting two different NL blocks.
    // These are coupling by definition — the greedy expansion must not
    // claim them for any single block. Without this, multi-hop feedback
    // paths (e.g., Resonance → R_fb_limit between Q4 and Q1) get greedily
    // absorbed into the first block they touch.
    //
    // Algorithm: BFS from each NL block's port nodes through sibling edges.
    // If a path reaches another block's port node, all edges on that path
    // are coupling.
    let coupling_set: HashSet<usize> = {
        // Build adjacency for sibling edges only
        let mut adj: HashMap<NodeId, Vec<(usize, NodeId)>> = HashMap::new();
        for &eidx in &sibling_edges {
            let e = &graph.edges[eidx];
            adj.entry(e.node_a).or_default().push((eidx, e.node_b));
            adj.entry(e.node_b).or_default().push((eidx, e.node_a));
        }

        // Map: node → which block(s) own it (from NL port nodes)
        let mut node_blocks: HashMap<NodeId, Vec<usize>> = HashMap::new();
        for (bi, nodes) in &block_port_nodes {
            for &n in nodes {
                if !excluded.contains(&n) {
                    node_blocks.entry(n).or_default().push(*bi);
                }
            }
        }

        let mut inter_block_edges: HashSet<usize> = HashSet::new();

        // BFS from each block's port nodes through sibling edges
        for (bi, nodes) in &block_port_nodes {
            for &start in nodes {
                if excluded.contains(&start) {
                    continue;
                }
                // BFS to find paths to other blocks
                let mut visited: HashSet<NodeId> = HashSet::new();
                let mut queue: Vec<(NodeId, Vec<usize>)> = vec![(start, Vec::new())];
                visited.insert(start);

                while let Some((node, path)) = queue.pop() {
                    // Did we reach a different block's port node?
                    if let Some(blocks) = node_blocks.get(&node) {
                        if blocks.iter().any(|b| b != bi) && !path.is_empty() {
                            // This path connects two blocks → all edges are coupling
                            inter_block_edges.extend(path.iter());
                            continue; // don't expand further from this node
                        }
                    }

                    if let Some(neighbors) = adj.get(&node) {
                        for &(eidx, next) in neighbors {
                            if visited.contains(&next) {
                                continue;
                            }
                            if excluded.contains(&next) {
                                continue;
                            }
                            visited.insert(next);
                            let mut new_path = path.clone();
                            new_path.push(eidx);
                            queue.push((next, new_path));
                        }
                    }
                }
            }
        }

        #[cfg(test)]
        if !inter_block_edges.is_empty() {
            let names: Vec<&str> = inter_block_edges
                .iter()
                .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
                .collect();
            eprintln!("  inter-block coupling detected: {:?}", names);
        }

        inter_block_edges
    };
    eprintln!(
        "  [blockwise] analyze: inter-block coupling done ({} edges)",
        coupling_set.len()
    );

    // Step 3: Pendant-chain assignment.
    //
    // Each block claims only edges reachable from its NL device's
    // COMMON terminal (emitter/source/cathode) via pendant chains
    // to ground/rail. This produces identical blocks for symmetric
    // cascades — the "repeatable unit" — and pushes contextual
    // edges (bias resistors, input/output networks) to coupling.
    //
    // Algorithm per block:
    //   1. Find the NL device's common terminal node
    //   2. BFS through sibling edges from that node
    //   3. Claim edges that terminate at excluded nodes (GND/VCC/rail)
    //   4. Stop at boundary nodes — don't cross into other blocks

    let sibling_set: HashSet<usize> = sibling_edges.iter().copied().collect();
    let assigned_set: HashSet<usize> = coupling_set.clone(); // start with coupling pre-excluded

    // Build adjacency for sibling edges
    let mut sib_adj: HashMap<NodeId, Vec<(usize, NodeId)>> = HashMap::new();
    for &eidx in &sibling_edges {
        if coupling_set.contains(&eidx) {
            continue;
        }
        let e = &graph.edges[eidx];
        sib_adj.entry(e.node_a).or_default().push((eidx, e.node_b));
        sib_adj.entry(e.node_b).or_default().push((eidx, e.node_a));
    }

    // Build map: node → which block(s) it belongs to (from NL port nodes)
    let mut node_owner: HashMap<NodeId, Vec<usize>> = HashMap::new();
    for (bi, nodes) in &block_port_nodes {
        for &n in nodes {
            if !excluded.contains(&n) {
                node_owner.entry(n).or_default().push(*bi);
            }
        }
    }

    // For each block, find its common terminal and BFS for pendant chains
    let mut claimed: HashSet<usize> = coupling_set.clone();

    for (bi, nl_block) in nl_blocks.iter().enumerate() {
        // Find common terminal (emitter/source/cathode)
        let comp = &graph.components[nl_block.comp_idx];
        let common_node = match comp.kind.signal_terminals() {
            super::component::SignalTerminals::Amplifier { input, output, .. } => {
                let common_pin = comp
                    .kind
                    .pin_config()
                    .valid_pins
                    .iter()
                    .find(|&&p| p != input && p != output);
                common_pin.and_then(|&pin| {
                    let key = format!("{}.{}", comp.id, pin);
                    graph.node_names.get(&key).copied()
                })
            }
            super::component::SignalTerminals::TwoPort { output, .. } => {
                let key = format!("{}.{}", comp.id, output);
                graph.node_names.get(&key).copied()
            }
            super::component::SignalTerminals::Passive => None,
        };

        let start_node = match common_node {
            Some(n) => n,
            None => continue, // can't determine common terminal
        };

        // BFS from common terminal through sibling edges
        let mut visited: HashSet<NodeId> = HashSet::new();
        let mut queue: Vec<NodeId> = vec![start_node];
        visited.insert(start_node);

        while let Some(node) = queue.pop() {
            if let Some(neighbors) = sib_adj.get(&node) {
                for &(eidx, next) in neighbors {
                    if claimed.contains(&eidx) {
                        continue;
                    }
                    if visited.contains(&next) {
                        continue;
                    }

                    // Claim this edge if it terminates at ground
                    // OR leads to a non-boundary interior node.
                    // Don't claim VCC/supply-connected edges at boundary
                    // nodes — those are bias networks for the next stage.
                    let terminates_at_gnd =
                        next == graph.gnd_node || graph.ac_ground_nodes.contains(&next);
                    let terminates_at_vcc =
                        next == graph.vcc_node || graph.supply_nodes.contains(&next);
                    // Stop at nodes owned by other blocks, circuit terminals,
                    // or shared boundaries. The BFS is allowed to START at a
                    // boundary (the emitter IS the cascade junction) but
                    // shouldn't cross INTO other blocks or the I/O interface.
                    let next_owned_by_other = node_owner
                        .get(&next)
                        .map_or(false, |owners| owners.iter().any(|&o| o != bi));
                    let next_is_terminal = next == graph.in_node
                        || next == graph.out_node
                        || external_terminal_nodes.contains(&next);
                    let is_boundary =
                        boundary_nodes.contains(&next) || next_owned_by_other || next_is_terminal;

                    if terminates_at_gnd {
                        // Edge to ground → pendant, claim for this block
                        claimed.insert(eidx);
                        match graph.effective_edge_kind(eidx) {
                            EdgeKind::Linear => block_linear[bi].push(eidx),
                            EdgeKind::Reactive => block_reactive[bi].push(eidx),
                            _ => {}
                        }
                        #[cfg(test)]
                        eprintln!(
                            "    assign edge {eidx} {}({:?}) → block {bi} (pendant to gnd)",
                            graph.components[graph.edges[eidx].comp_idx].id,
                            graph.effective_edge_kind(eidx)
                        );
                    } else if terminates_at_vcc {
                        // VCC-connected at boundary → coupling (bias for next stage)
                        // VCC-connected at non-boundary → claim (only this block's bias)
                        if !boundary_nodes.contains(&node) {
                            claimed.insert(eidx);
                            match graph.effective_edge_kind(eidx) {
                                EdgeKind::Linear => block_linear[bi].push(eidx),
                                EdgeKind::Reactive => block_reactive[bi].push(eidx),
                                _ => {}
                            }
                            #[cfg(test)]
                            eprintln!("    assign edge {eidx} {}({:?}) → block {bi} (pendant to vcc, non-boundary)",
                                graph.components[graph.edges[eidx].comp_idx].id,
                                graph.effective_edge_kind(eidx));
                        }
                        // else: leave unclaimed → becomes coupling
                    } else if !is_boundary {
                        // Interior node (not shared) → claim and continue BFS
                        visited.insert(next);
                        claimed.insert(eidx);
                        match graph.effective_edge_kind(eidx) {
                            EdgeKind::Linear => block_linear[bi].push(eidx),
                            EdgeKind::Reactive => block_reactive[bi].push(eidx),
                            _ => {}
                        }
                        queue.push(next);
                        #[cfg(test)]
                        eprintln!(
                            "    assign edge {eidx} {}({:?}) → block {bi} (pendant interior)",
                            graph.components[graph.edges[eidx].comp_idx].id,
                            graph.effective_edge_kind(eidx)
                        );
                    }
                    // If boundary → don't claim, don't cross. Edge stays unassigned.
                }
            }
        }
    }

    // Inter-block coupling (from BFS pre-classification) + unclaimed → coupling
    for &eidx in &sibling_edges {
        if coupling_set.contains(&eidx) && !coupling_edges.contains(&eidx) {
            coupling_edges.push(eidx);
        }
    }
    for &eidx in &sibling_edges {
        if !claimed.contains(&eidx) {
            coupling_edges.push(eidx);
            #[cfg(test)]
            eprintln!(
                "    coupling: edge {eidx} {}({:?})",
                graph.components[graph.edges[eidx].comp_idx].id,
                graph.effective_edge_kind(eidx)
            );
        }
    }

    let block_order: Vec<usize> =
        signal_order_indices(&nl_blocks, graph).unwrap_or_else(|| (0..nl_blocks.len()).collect());

    #[cfg(test)]
    {
        let order_names: Vec<&str> = block_order
            .iter()
            .map(|&i| graph.components[nl_blocks[i].comp_idx].id.as_str())
            .collect();
        eprintln!("  block signal order: {:?}", order_names);
    }

    // Build the plan
    let mut blocks = Vec::new();
    for i in block_order {
        let nl_block = &nl_blocks[i];
        let linear_edges = std::mem::take(&mut block_linear[i]);
        let reactive_edges = std::mem::take(&mut block_reactive[i]);
        let topology = classify_block_topology(&nl_block.nl_edges, &reactive_edges, graph);
        blocks.push(Block {
            nl_edges: nl_block.nl_edges.clone(),
            linear_edges,
            reactive_edges,
            port_nodes: nl_block.nodes.clone(),
            topology,
        });
    }

    let differential_prefix_len = blocks
        .iter()
        .take_while(|block| matches!(block.topology, BlockTopology::DifferentialDiodeRung { .. }))
        .count();
    if differential_prefix_len > 1 {
        let order =
            differential_rung_bottom_to_top_order(&blocks[..differential_prefix_len], graph);
        if let Some(order) = order {
            let mut old_blocks = blocks.into_iter().map(Some).collect::<Vec<_>>();
            let mut reordered = Vec::with_capacity(old_blocks.len());
            for idx in order {
                if let Some(block) = old_blocks.get_mut(idx).and_then(Option::take) {
                    reordered.push(block);
                }
            }
            for block in old_blocks.into_iter().flatten() {
                reordered.push(block);
            }
            blocks = reordered;
        }
        #[cfg(test)]
        {
            let order_names: Vec<&str> = blocks
                .iter()
                .filter_map(|block| {
                    block
                        .nl_edges
                        .first()
                        .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
                })
                .collect();
            eprintln!("  differential rung bottom-to-top order: {:?}", order_names);
        }
    }

    let all_port_nodes: Vec<NodeId> = blocks
        .iter()
        .flat_map(|b| b.port_nodes.iter().copied())
        .collect::<HashSet<_>>()
        .into_iter()
        .collect();

    let mut coupling = CouplingNetwork::new(all_port_nodes.clone());
    for &edge_idx in &coupling_edges {
        let role = if coupling_set.contains(&edge_idx) {
            CouplingEdgeRole::InterBlock
        } else {
            CouplingEdgeRole::Residual
        };
        coupling.push_edge(edge_idx, role);
    }

    let plan = BlockwisePlan {
        blocks,
        coupling,
        port_nodes: all_port_nodes,
    };

    if validate_plan(&plan, graph) {
        Some(plan)
    } else {
        #[cfg(test)]
        {
            for (i, b) in plan.blocks.iter().enumerate() {
                eprintln!(
                    "  validate fail: block {i}: {} NL, {} reactive, {} linear",
                    b.nl_edges.len(),
                    b.reactive_edges.len(),
                    b.linear_edges.len()
                );
            }
        }
        None
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Public entry point: build
// ═══════════════════════════════════════════════════════════════════════════

/// Try blockwise decomposition and build. Returns `Some(stages)` if the
/// circuit decomposes, `None` to fall through to monolithic.
pub(super) fn try_build_blockwise(
    edge_indices: &[usize],
    graph: &CircuitGraph,
    terminals: &[NodeId],
    sample_rate: f64,
    bias_node_voltages: &std::collections::BTreeMap<NodeId, f64>,
    supply_voltage: f64,
    port_defs: &[crate::dsl::PortDef],
    force_serial: bool,
    force_serial_feedback_gain: f64,
    disable_iir: bool,
    coupled_newton: bool,
    init_hints: &[crate::dsl::InitHint],
) -> Option<Vec<BuiltStage>> {
    let mut plan = analyze_blockwise(edge_indices, graph)?;

    #[cfg(test)]
    eprintln!(
        "  Blockwise: {} blocks, {} coupling edges",
        plan.num_blocks(),
        plan.coupling.len()
    );

    let mut early_block_boundary_nodes: HashSet<NodeId> = HashSet::new();
    for block in &plan.blocks {
        if let Some(ports) = differential_rung_ports(block, graph) {
            early_block_boundary_nodes.extend([
                ports.top_left,
                ports.top_right,
                ports.bottom_left,
                ports.bottom_right,
            ]);
        } else {
            early_block_boundary_nodes.extend(block.port_nodes.iter().copied());
        }
    }
    let early_external_terminal_nodes: HashSet<NodeId> = port_defs
        .iter()
        .filter_map(|port_def| graph.node_names.get(&port_def.name).copied())
        .collect();
    let mut early_extra_coupling = Vec::new();
    let mut early_extra_coupling_roles: HashMap<usize, CouplingEdgeRole> = HashMap::new();
    for port_def in port_defs {
        if port_def.direction != pedalkernel_rt::PortDirection::Input {
            continue;
        }
        let Some(&port_node) = graph.node_names.get(&port_def.name) else {
            continue;
        };

        let mut queue: VecDeque<(NodeId, Vec<usize>)> = VecDeque::new();
        let mut visited: HashSet<NodeId> = HashSet::new();
        queue.push_back((port_node, Vec::new()));
        visited.insert(port_node);
        while let Some((node, path)) = queue.pop_front() {
            if path.len() >= 4 {
                continue;
            }
            for (eidx, edge) in graph.edges.iter().enumerate() {
                if early_extra_coupling.contains(&eidx) || path.contains(&eidx) {
                    continue;
                }
                if !matches!(
                    graph.effective_edge_kind(eidx),
                    EdgeKind::Linear | EdgeKind::Reactive
                ) {
                    continue;
                }
                let next = if edge.node_a == node {
                    edge.node_b
                } else if edge.node_b == node {
                    edge.node_a
                } else {
                    continue;
                };

                let mut next_path = path.clone();
                next_path.push(eidx);
                if early_block_boundary_nodes.contains(&next) {
                    for path_eidx in next_path {
                        if !plan.coupling.contains_edge(path_eidx)
                            && !early_extra_coupling.contains(&path_eidx)
                        {
                            early_extra_coupling.push(path_eidx);
                            early_extra_coupling_roles
                                .insert(path_eidx, CouplingEdgeRole::BoundaryInput);
                        }
                    }
                    continue;
                }

                let next_is_external = next == graph.out_node
                    || early_external_terminal_nodes.contains(&next)
                    || next == graph.gnd_node
                    || graph.ac_ground_nodes.contains(&next)
                    || next == graph.vcc_node
                    || graph.supply_nodes.contains(&next);
                if !next_is_external && visited.insert(next) {
                    queue.push_back((next, next_path));
                }
            }
        }
    }

    let mut early_output_boundary_nodes: HashSet<NodeId> = HashSet::new();
    early_output_boundary_nodes.insert(graph.out_node);
    for port_def in port_defs {
        if port_def.direction != pedalkernel_rt::PortDirection::Output {
            continue;
        }
        if let Some(&port_node) = graph.node_names.get(&port_def.name) {
            early_output_boundary_nodes.insert(port_node);
        }
    }
    for (eidx, edge) in graph.edges.iter().enumerate() {
        if plan.coupling.contains_edge(eidx) || early_extra_coupling.contains(&eidx) {
            continue;
        }
        if !matches!(graph.effective_edge_kind(eidx), EdgeKind::Linear) {
            continue;
        }
        let Some(other_node) = (if early_output_boundary_nodes.contains(&edge.node_a) {
            Some(edge.node_b)
        } else if early_output_boundary_nodes.contains(&edge.node_b) {
            Some(edge.node_a)
        } else {
            None
        }) else {
            continue;
        };
        if other_node == graph.gnd_node || graph.ac_ground_nodes.contains(&other_node) {
            early_extra_coupling.push(eidx);
            early_extra_coupling_roles.insert(eidx, CouplingEdgeRole::BoundaryOutput);
        }
    }

    if !early_extra_coupling.is_empty() {
        let early_set: HashSet<usize> = early_extra_coupling.iter().copied().collect();
        for block in &mut plan.blocks {
            block.linear_edges.retain(|eidx| !early_set.contains(eidx));
            block
                .reactive_edges
                .retain(|eidx| !early_set.contains(eidx));
        }
        for eidx in early_extra_coupling {
            let role = early_extra_coupling_roles
                .get(&eidx)
                .copied()
                .unwrap_or(CouplingEdgeRole::Residual);
            if plan.add_coupling_edge(eidx, role) {
                #[cfg(test)]
                eprintln!(
                    "  [blockwise] preclaimed boundary edge {} into coupling",
                    graph.components[graph.edges[eidx].comp_idx].id
                );
            }
        }
    }

    let formulation_selection = select_formulations(
        &plan,
        force_serial,
        force_serial_feedback_gain,
        coupled_newton,
    );

    let LoweredBlockStages {
        stages: mut all_stages,
        stage_plan_blocks: mut all_stage_plan_blocks,
    } = lower_block_stages(
        &plan,
        &formulation_selection,
        graph,
        terminals,
        sample_rate,
        bias_node_voltages,
        supply_voltage,
        disable_iir,
        init_hints,
    )?;

    // ── Build coupling scattering matrix + package as BlockwiseStage ────────
    //
    // Instead of building coupling edges as a separate passive WDF stage
    // (which has no feedback loop), we build a coupling scattering matrix
    // that connects all blocks' ports. The BlockwiseStage uses
    // wave-domain Newton iteration to solve the algebraic feedback loop
    // each sample — no unit delay, correct resonance tracking.
    if !plan.coupling.is_empty() && !all_stages.is_empty() {
        let mut coupling_edges = plan.coupling.edge_indices();
        let mut block_boundary_nodes: HashSet<NodeId> = HashSet::new();
        for block in &plan.blocks {
            if let Some(ports) = differential_rung_ports(block, graph) {
                block_boundary_nodes.extend([
                    ports.top_left,
                    ports.top_right,
                    ports.bottom_left,
                    ports.bottom_right,
                ]);
            } else {
                block_boundary_nodes.extend(block.port_nodes.iter().copied());
            }
        }
        let external_terminal_nodes: HashSet<NodeId> = port_defs
            .iter()
            .filter_map(|port_def| graph.node_names.get(&port_def.name).copied())
            .collect();

        for port_def in port_defs {
            if port_def.direction != pedalkernel_rt::PortDirection::Input {
                continue;
            }
            let Some(&port_node) = graph.node_names.get(&port_def.name) else {
                continue;
            };
            for (eidx, edge) in graph.edges.iter().enumerate() {
                if coupling_edges.contains(&eidx) {
                    continue;
                }
                if !matches!(
                    graph.effective_edge_kind(eidx),
                    EdgeKind::Linear | EdgeKind::Reactive
                ) {
                    continue;
                }
                let other_node = if edge.node_a == port_node {
                    edge.node_b
                } else if edge.node_b == port_node {
                    edge.node_a
                } else {
                    continue;
                };
                if block_boundary_nodes.contains(&other_node) {
                    coupling_edges.push(eidx);
                    #[cfg(test)]
                    eprintln!(
                        "  [blockwise] pulled input boundary edge {} into coupling for port {}",
                        graph.components[edge.comp_idx].id, port_def.name
                    );
                }
            }

            let mut queue: VecDeque<(NodeId, Vec<usize>)> = VecDeque::new();
            let mut visited: HashSet<NodeId> = HashSet::new();
            queue.push_back((port_node, Vec::new()));
            visited.insert(port_node);

            while let Some((node, path)) = queue.pop_front() {
                if path.len() >= 4 {
                    continue;
                }
                for (eidx, edge) in graph.edges.iter().enumerate() {
                    if path.contains(&eidx) {
                        continue;
                    }
                    if !matches!(
                        graph.effective_edge_kind(eidx),
                        EdgeKind::Linear | EdgeKind::Reactive
                    ) {
                        continue;
                    }
                    let next = if edge.node_a == node {
                        edge.node_b
                    } else if edge.node_b == node {
                        edge.node_a
                    } else {
                        continue;
                    };

                    let mut next_path = path.clone();
                    next_path.push(eidx);
                    if block_boundary_nodes.contains(&next) {
                        for path_eidx in next_path {
                            if !coupling_edges.contains(&path_eidx) {
                                coupling_edges.push(path_eidx);
                                #[cfg(test)]
                                eprintln!(
                                    "  [blockwise] pulled input boundary chain edge {} into coupling for port {}",
                                    graph.components[graph.edges[path_eidx].comp_idx].id,
                                    port_def.name
                                );
                            }
                        }
                        continue;
                    }

                    let next_is_external = next == graph.out_node
                        || external_terminal_nodes.contains(&next)
                        || next == graph.gnd_node
                        || graph.ac_ground_nodes.contains(&next)
                        || next == graph.vcc_node
                        || graph.supply_nodes.contains(&next);
                    if !next_is_external && visited.insert(next) {
                        queue.push_back((next, next_path));
                    }
                }
            }
        }

        let mut output_boundary_nodes: HashSet<NodeId> = HashSet::new();
        output_boundary_nodes.insert(graph.out_node);
        for port_def in port_defs {
            if port_def.direction != pedalkernel_rt::PortDirection::Output {
                continue;
            }
            if let Some(&port_node) = graph.node_names.get(&port_def.name) {
                output_boundary_nodes.insert(port_node);
            }
        }
        for (eidx, edge) in graph.edges.iter().enumerate() {
            if coupling_edges.contains(&eidx) {
                continue;
            }
            if !matches!(
                graph.effective_edge_kind(eidx),
                EdgeKind::Linear | EdgeKind::Reactive
            ) {
                continue;
            }
            let Some(other_node) = (if output_boundary_nodes.contains(&edge.node_a) {
                Some(edge.node_b)
            } else if output_boundary_nodes.contains(&edge.node_b) {
                Some(edge.node_a)
            } else {
                None
            }) else {
                continue;
            };
            if other_node == graph.gnd_node || graph.ac_ground_nodes.contains(&other_node) {
                coupling_edges.push(eidx);
                #[cfg(test)]
                eprintln!(
                    "  [blockwise] pulled output boundary load {} into coupling",
                    graph.components[edge.comp_idx].id
                );
            }
        }

        let coupling_names: Vec<&str> = coupling_edges
            .iter()
            .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
            .collect();
        eprintln!(
            "  [blockwise] coupling: {} edges {:?}",
            coupling_edges.len(),
            coupling_names
        );

        // Extract KMethodBlocks from the built WDF stages.
        //
        // Each block's VS Rp must match the source impedance driving it:
        // - Block 0: driven by R_in from external input → VS Rp from port
        //   impedance (e.g. 10kΩ). Set by with_voltage_source() default.
        // - Blocks 1..N: driven by previous diode's output → VS Rp = 1/gm
        //   where gm = I_bias / Vt. The bias current comes from R_bias to VCC.
        //
        // Without this, VS Rp = 10kΩ for all blocks → gamma ≈ 0.97 → cap
        // can't filter → each stage adds gain → 12× total gain explosion.

        // Compute a fallback 1/gm from bias: I = (V_supply - V_be) / R_bias.
        // Individual blocks below prefer their own bias leg because cascaded
        // diode ladders often use different bias resistors per rung.
        let vt = pedalkernel_rt::SPICE_VT_27C; // thermal voltage at ngspice default 27°C
        let v_be = 0.6; // typical forward bias
                        // Find the typical R_bias value from coupling edges
        let r_bias_value = coupling_edges
            .iter()
            .find_map(|&eidx| {
                let comp = &graph.components[graph.edges[eidx].comp_idx];
                if comp.id.contains("R_bias") || comp.id.contains("r_bias") {
                    comp.kind.resistance()
                } else {
                    None
                }
            })
            .unwrap_or(100_000.0);
        let i_bias = (supply_voltage - v_be).max(0.1) / r_bias_value;
        let gm = i_bias / vt;
        let r_source_cascade = (1.0 / gm).clamp(10.0, 10_000.0); // 1/gm, clamped

        #[cfg(test)]
        eprintln!(
            "  cascade source impedance: R_bias={r_bias_value:.0}Ω, I_bias={i_bias:.1e}A, \
                   gm={gm:.4}S, 1/gm={r_source_cascade:.1}Ω"
        );

        let first_block_source_r = port_defs
            .iter()
            .find(|port| port.direction == pedalkernel_rt::PortDirection::Input)
            .and_then(|port| port.impedance)
            .unwrap_or(10_000.0);

        let has_differential_rung_blocks = plan
            .blocks
            .iter()
            .any(|block| matches!(block.topology, BlockTopology::DifferentialDiodeRung { .. }));
        let mut sub_stages: Vec<Stage> = Vec::new();
        let mut plan_block_to_k_block = vec![None; plan.blocks.len()];
        for (stage_idx, built) in all_stages.iter_mut().enumerate() {
            let bi = all_stage_plan_blocks
                .get(stage_idx)
                .copied()
                .unwrap_or(stage_idx);
            if let BuiltStage::Wdf(ref mut wdf) = built {
                if has_differential_rung_blocks
                    && !matches!(
                        plan.blocks.get(bi).map(|block| &block.topology),
                        Some(BlockTopology::DifferentialDiodeRung { .. })
                    )
                {
                    #[cfg(test)]
                    {
                        if matches!(
                            plan.blocks.get(bi).map(|block| &block.topology),
                            Some(BlockTopology::InputDifferentialPair { .. })
                        ) {
                            eprintln!(
                                "  block {bi}: input differential pair kept as a WDF boundary block outside rung blockwise stage"
                            );
                        } else {
                            eprintln!(
                                "  block {bi}: auxiliary memoryless/control WDF omitted from differential-rung blockwise blocks"
                            );
                        }
                    }
                    continue;
                }
                let block_port_node = plan.blocks.get(bi).and_then(|block| {
                    block_coupling_port_node(&block.nl_edges, &block.port_nodes, graph)
                });
                let block_bias_resistance = block_port_node
                    .and_then(|port_node| {
                        coupling_edges.iter().find_map(|&eidx| {
                            let e = &graph.edges[eidx];
                            let comp = &graph.components[e.comp_idx];
                            let touches_port = e.node_a == port_node || e.node_b == port_node;
                            let touches_supply = e.node_a == graph.vcc_node
                                || e.node_b == graph.vcc_node
                                || graph.supply_nodes.contains(&e.node_a)
                                || graph.supply_nodes.contains(&e.node_b);
                            if touches_port && touches_supply {
                                comp.kind.resistance()
                            } else {
                                None
                            }
                        })
                    })
                    .unwrap_or(r_bias_value);
                // Block 0 is driven through the declared input impedance.
                // Downstream rungs are driven by the previous diode follower,
                // whose small-signal output impedance is set by that rung's
                // own DC bias current.
                let block_source_r = if bi == 0 {
                    first_block_source_r
                } else if matches!(
                    plan.blocks.get(bi).map(|block| &block.topology),
                    Some(BlockTopology::DifferentialDiodeRung { .. })
                ) {
                    // Differential diode rungs are not emitter followers in a
                    // buffered cascade. Their local cross-capacitor WDF was
                    // built with the rung primitive's own source impedance;
                    // do not replace it with the old single-ended 1/gm guess.
                    1_000.0
                } else {
                    match &wdf.root {
                        pedalkernel_rt::stage::RootKind::ExplicitSingleDiode(root) => root
                            .model
                            .dynamic_resistance_from_sources(&[(
                                supply_voltage,
                                block_bias_resistance,
                            )])
                            .clamp(10.0, 10_000.0),
                        _ => {
                            let i_bias = (supply_voltage - v_be).max(0.1) / block_bias_resistance;
                            let gm = i_bias / vt;
                            (1.0 / gm).clamp(10.0, 10_000.0)
                        }
                    }
                };
                wdf.tree.set_vs_port_resistance(block_source_r);
                let diode_cutoff = match (&wdf.root, plan.blocks.get(bi).map(|b| &b.topology)) {
                    (pedalkernel_rt::stage::RootKind::ExplicitSingleDiode(_), _)
                    | (
                        pedalkernel_rt::stage::RootKind::DiffPair(_),
                        Some(BlockTopology::DifferentialDiodeRung { .. }),
                    ) => Some(pedalkernel_rt::stage::DiodeCutoffCalibration {
                        bias_voltage: supply_voltage,
                        bias_resistance: block_bias_resistance,
                        cv_resistance: None,
                        min_rp: 10.0,
                        max_rp: 100_000.0,
                    }),
                    _ => None,
                };

                // Generate K-table with the correct adapted tree impedance.
                // A diode-connected ladder block needs a bias-voltage axis:
                // the local reactive tree removes DC from b_tree, but the
                // nonlinear diode's small-signal conductance is set by the
                // raw coupling/cascade voltage.
                wdf.k_table = match &wdf.root {
                    pedalkernel_rt::stage::RootKind::ExplicitSingleDiode(root) => {
                        Some(super::k_method::generate_biased_single_diode_k_table(
                            root.model,
                            wdf.tree.port_resistance(),
                        ))
                    }
                    _ => super::k_method::generate_k_table(wdf),
                };

                if let Some(ref k_table) = wdf.k_table {
                    let vbe_bias = match &wdf.root {
                        pedalkernel_rt::stage::RootKind::Bjt(bjt) => bjt.vbe_bias(),
                        pedalkernel_rt::stage::RootKind::DiffPair(dp) => dp.i_tail_bias(),
                        _ => 0.6,
                    };
                    // dc_offset initialized to the quiescent cascade voltage.
                    // Tracked at runtime via 1-pole LPF to extract the AC
                    // component for the K-table ctrl axis.
                    let mut dc_tree = wdf.tree.clone();
                    let mut dc_runtime_state = wdf.runtime_state.clone();
                    dc_tree.set_voltage(0.0);
                    let b0 = dc_tree.reflected_with_state(&mut dc_runtime_state);
                    let mut dc_kt = k_table.clone();
                    dc_kt.precompute_scales();
                    let a0 = if dc_kt.dims == 1 {
                        dc_kt.lookup_1d(b0)
                    } else {
                        dc_kt.lookup_2d(b0, 0.0)
                    };
                    dc_tree.set_incident_with_state(a0, &mut dc_runtime_state);
                    let dc_offset = wdf
                        .output_probe
                        .as_ref()
                        .and_then(|probe| dc_tree.leaf_voltage_with_state(probe, &dc_runtime_state))
                        .unwrap_or((a0 + b0) / 2.0);

                    #[cfg(test)]
                    eprintln!(
                        "  block {bi}: VS Rp={:.1}Ω, tree Rp={:.1}Ω",
                        block_source_r,
                        wdf.tree.port_resistance()
                    );
                    let root_polarity = match &wdf.root {
                        pedalkernel_rt::stage::RootKind::DiodePair(_)
                        | pedalkernel_rt::stage::RootKind::SingleDiode(_)
                        | pedalkernel_rt::stage::RootKind::ExplicitDiodePair(_)
                        | pedalkernel_rt::stage::RootKind::ExplicitSingleDiode(_)
                        | pedalkernel_rt::stage::RootKind::Zener(_) => -1.0,
                        _ => 1.0,
                    };
                    let source_polarity = if wdf.negate_vs {
                        -root_polarity
                    } else {
                        root_polarity
                    };
                    let k_table_control_polarity = match &wdf.root {
                        // The biased diode K-table axis is physical diode bias,
                        // not the voltage-source orientation used to drive the
                        // WDF tree.
                        pedalkernel_rt::stage::RootKind::ExplicitSingleDiode(_) => 1.0,
                        _ => source_polarity,
                    };
                    let control_from_drive =
                        !matches!(&wdf.root, pedalkernel_rt::stage::RootKind::DiffPair(_));
                    // The cascade tap is at the passive subtree (cap voltage),
                    // not the root port (diode residual). This is determined
                    // by the circuit: the cascade node (Q.emitter) is where
                    // the cap connects, which is the right child of the
                    // Series(VS, Cap) adaptor.
                    let k_block_idx = sub_stages.len();
                    if bi < plan_block_to_k_block.len() {
                        plan_block_to_k_block[bi] = Some(k_block_idx);
                    }
                    sub_stages.push(Stage::KMethod {
                        block: pedalkernel_rt::stage::KMethodBlock {
                            tree: wdf.tree.clone(),
                            runtime_state: wdf.runtime_state.clone(),
                            k_table: k_table.clone(),
                            explicit_diode_root: match &wdf.root {
                                pedalkernel_rt::stage::RootKind::ExplicitSingleDiode(root) => {
                                    Some(*root)
                                }
                                _ => None,
                            },
                            nominal_vs_rp: block_source_r,
                            diode_cutoff,
                            rp: wdf.tree.port_resistance(),
                            vbe_bias,
                            dc_offset,
                            cascade_from_passive: true,
                            cascade_probe_id: wdf.output_probe.clone(),
                            source_polarity,
                            k_table_control_polarity,
                            shared_diode_bias_voltage: None,
                            control_from_drive,
                        },
                        ports: Vec::new(),
                    });
                }
            }
        }

        match formulation_selection.coupling {
            CouplingFormulation::SerialDelayedFeedback => {
                let mut rung_stages = Vec::with_capacity(all_stages.len());
                for built in all_stages {
                    match built {
                        BuiltStage::Wdf(wdf) => rung_stages.push(wdf),
                        other => return Some(vec![other]),
                    }
                }
                eprintln!(
                    "  [blockwise] force_serial: wrapping {} rung stages with z^-1 feedback gain {force_serial_feedback_gain:+.3}",
                    rung_stages.len()
                );
                return Some(vec![BuiltStage::SerialDelayedFeedback(
                    pedalkernel_rt::stage::SerialDelayedFeedbackStage::new(
                        rung_stages,
                        force_serial_feedback_gain,
                    ),
                )]);
            }
            CouplingFormulation::Serial => {
                eprintln!(
                "  [blockwise] force_serial: returning {} rung stages without coupled blockwise solve",
                all_stages.len()
            );
                return Some(all_stages);
            }
            CouplingFormulation::None | CouplingFormulation::BlockwiseCoupled { .. } => {}
        }

        let n_blocks = sub_stages.len();
        if n_blocks < 2 {
            // Not enough K-table blocks to preserve the coupling network.
            // Returning separate stages here is only valid for true cascades;
            // for coupled ladders it drops the residual network entirely.
            eprintln!(
                "  [blockwise] only {n_blocks} K-table blocks, falling back to monolithic compile"
            );
            return None;
        }

        // ── Build MNA for coupling network ──────────────────────────────
        // Collect unique nodes from coupling edges.
        //
        // GND and AC ground are true ground references (MNA ground node).
        // VCC and supply nodes are NOT ground — they are DC voltage sources
        // that bias the circuit. They get included in the MNA as nodes,
        // and a VS port is added to inject the supply voltage.
        let mut node_set: Vec<NodeId> = Vec::new();
        let ground_rails: HashSet<NodeId> = {
            let mut r = HashSet::new();
            r.insert(graph.gnd_node);
            r.extend(&graph.ac_ground_nodes);
            r
        };
        // Track which supply nodes appear in the coupling (need VS ports)
        let mut supply_nodes_in_coupling: Vec<(NodeId, f64)> = Vec::new();

        for &eidx in &coupling_edges {
            let e = &graph.edges[eidx];
            for &node in &[e.node_a, e.node_b] {
                if ground_rails.contains(&node) {
                    continue; // true ground — MNA reference
                }
                if !node_set.contains(&node) {
                    node_set.push(node);
                }
                // Track supply nodes that need VS ports
                if node == graph.vcc_node || graph.supply_nodes.contains(&node) {
                    if !supply_nodes_in_coupling.iter().any(|(n, _)| *n == node) {
                        supply_nodes_in_coupling.push((node, supply_voltage));
                    }
                }
            }
        }
        let mut coupling_vcvs_comps: Vec<usize> = Vec::new();
        for &eidx in &coupling_edges {
            if graph.effective_edge_kind(eidx) != EdgeKind::Vcvs {
                continue;
            }
            let comp_idx = graph.edges[eidx].comp_idx;
            if !coupling_vcvs_comps.contains(&comp_idx) {
                coupling_vcvs_comps.push(comp_idx);
            }
        }
        for &comp_idx in &coupling_vcvs_comps {
            if let Some(pins) = graph
                .nullor_pins
                .iter()
                .find(|pins| pins.comp_idx == comp_idx)
            {
                for node in [pins.pos_node, pins.neg_node, pins.out_node] {
                    if ground_rails.contains(&node) {
                        continue;
                    }
                    if !node_set.contains(&node) {
                        node_set.push(node);
                    }
                }
            }
        }
        let coupling_touch_count = |node: NodeId| -> usize {
            coupling_edges
                .iter()
                .filter(|&&eidx| {
                    let e = &graph.edges[eidx];
                    e.node_a == node || e.node_b == node
                })
                .count()
        };

        for block in &plan.blocks {
            if let Some(ports) = differential_rung_ports(block, graph) {
                for node in [
                    ports.top_left,
                    ports.top_right,
                    ports.bottom_left,
                    ports.bottom_right,
                ] {
                    if ground_rails.contains(&node) {
                        continue;
                    }
                    if node == graph.vcc_node || graph.supply_nodes.contains(&node) {
                        continue;
                    }
                    if !node_set.contains(&node) {
                        node_set.push(node);
                    }
                }
                continue;
            }
            for &node in &block.port_nodes {
                if ground_rails.contains(&node) {
                    continue;
                }
                if node == graph.vcc_node || graph.supply_nodes.contains(&node) {
                    continue;
                }
                if coupling_touch_count(node) == 0 {
                    continue;
                }
                if !node_set.contains(&node) {
                    node_set.push(node);
                }
            }
        }
        let node_to_mna: HashMap<NodeId, usize> =
            node_set.iter().enumerate().map(|(i, &n)| (n, i)).collect();
        let n_mna = node_set.len();

        eprintln!("  [blockwise] coupling MNA: {n_mna} nodes, {n_blocks} sub-stage ports");

        let mut vcvs_vsource_base: HashMap<usize, usize> = HashMap::new();
        let mut n_vcvs_sources = 0usize;
        for &comp_idx in &coupling_vcvs_comps {
            let count = graph.components[comp_idx].kind.mna_vsource_count();
            if count > 0 {
                vcvs_vsource_base.insert(comp_idx, n_vcvs_sources);
                n_vcvs_sources += count;
            }
        }

        // Create MNA system and stamp coupling linear elements.
        let mut mna = pedalkernel_rt::tree::MnaSystem::new(n_mna, n_vcvs_sources);
        let output_feedback_node = plan
            .blocks
            .last()
            .and_then(|block| block_cascade_output_node(&block.nl_edges, &block.port_nodes, graph));

        let mut coupling_elements = Vec::new();
        let mut coupling_passive_specs: Vec<(
            String,
            Option<usize>,
            Option<usize>,
            Option<usize>,
            Option<usize>,
            f64,
        )> = Vec::new();
        let mut coupling_vcvss = Vec::new();
        for &comp_idx in &coupling_vcvs_comps {
            let comp = &graph.components[comp_idx];
            let Some(pins) = graph
                .nullor_pins
                .iter()
                .find(|pins| pins.comp_idx == comp_idx)
            else {
                continue;
            };
            let pin_to_mna = |pin: &str| -> Option<usize> {
                let node = match pin {
                    "pos" => pins.pos_node,
                    "neg" => pins.neg_node,
                    "out" => pins.out_node,
                    _ => return None,
                };
                if ground_rails.contains(&node) {
                    None
                } else {
                    node_to_mna.get(&node).copied()
                }
            };
            let vsource_index = *vcvs_vsource_base.get(&comp_idx).unwrap_or(&0);
            let mut ctx = super::component::StampContext {
                pin_to_mna: &pin_to_mna,
                vsrc_base: vsource_index,
                internal_node_base: 0,
                sample_rate,
                reactive_one_ports: None,
            };
            comp.kind.stamp_mna_multi(&comp.id, &mut ctx, &mut mna);

            let model = comp
                .kind
                .op_amp_type()
                .map(|op_type| crate::model_lookup::opamp_model_from_type(&op_type));
            if let Some(model) = model {
                coupling_vcvss.push(pedalkernel_rt::stage::CouplingVcvs {
                    pos: pin_to_mna("pos"),
                    neg: pin_to_mna("neg"),
                    out_pos: pin_to_mna("out"),
                    out_neg: None,
                    gain: model.open_loop_gain,
                    output_resistance: model.output_impedance,
                    vsource_index,
                });
            }
        }
        let mut node_incidence: HashMap<NodeId, usize> = HashMap::new();
        for edge in &graph.edges {
            *node_incidence.entry(edge.node_a).or_default() += 1;
            *node_incidence.entry(edge.node_b).or_default() += 1;
        }
        let is_floating_terminal = |node: NodeId| -> bool {
            !ground_rails.contains(&node)
                && node != graph.in_node
                && node != graph.out_node
                && node != graph.vcc_node
                && !graph.supply_nodes.contains(&node)
                && !graph.ac_ground_nodes.contains(&node)
                && node_incidence.get(&node).copied().unwrap_or(0) <= 1
        };

        for &eidx in &coupling_edges {
            let e = &graph.edges[eidx];
            let comp = &graph.components[e.comp_idx];
            if graph.effective_edge_kind(eidx) == EdgeKind::Vcvs {
                continue;
            }
            if graph.effective_edge_kind(eidx) != EdgeKind::Linear {
                if graph.effective_edge_kind(eidx) == EdgeKind::Reactive {
                    if let Some(capacitance) = comp.kind.capacitance() {
                        let n1 = if ground_rails.contains(&e.node_a) {
                            None
                        } else {
                            node_to_mna.get(&e.node_a).copied()
                        };
                        let n2 = if ground_rails.contains(&e.node_b) {
                            None
                        } else {
                            node_to_mna.get(&e.node_b).copied()
                        };
                        let graph_n1 = (!ground_rails.contains(&e.node_a)).then_some(e.node_a);
                        let graph_n2 = (!ground_rails.contains(&e.node_b)).then_some(e.node_b);
                        coupling_passive_specs.push((
                            comp.id.clone(),
                            n1,
                            n2,
                            graph_n1,
                            graph_n2,
                            capacitance,
                        ));
                        continue;
                    }
                }
                #[cfg(test)]
                eprintln!(
                    "    skip non-resistive coupling edge {eidx} {}({:?})",
                    comp.id,
                    graph.effective_edge_kind(eidx)
                );
                continue;
            }
            let Some(r) = comp.kind.resistance() else {
                #[cfg(test)]
                eprintln!(
                    "    skip coupling edge {eidx} {} without resistance",
                    comp.id
                );
                continue;
            };
            if comp.kind.is_pot()
                && (is_floating_terminal(e.node_a) || is_floating_terminal(e.node_b))
            {
                #[cfg(test)]
                eprintln!(
                    "    skip floating pot coupling edge {eidx} {} node {:?}->{:?}",
                    comp.id, e.node_a, e.node_b
                );
                continue;
            }
            let n1 = if ground_rails.contains(&e.node_a) {
                None
            } else {
                node_to_mna.get(&e.node_a).copied()
            };
            let n2 = if ground_rails.contains(&e.node_b) {
                None
            } else {
                node_to_mna.get(&e.node_b).copied()
            };
            mna.stamp_resistor(n1, n2, r);

            let pot_meta = comp
                .kind
                .as_any()
                .downcast_ref::<super::components::Potentiometer>()
                .map(|p| (p.max_r, p.taper));
            let invert_control = pot_meta.is_some()
                && (output_feedback_node
                    .map(|node| e.node_a == node || e.node_b == node)
                    .unwrap_or(false)
                    || (n1.is_some() && n2.is_some()));
            coupling_elements.push(pedalkernel_rt::stage::CouplingElement {
                comp_id: comp.id.clone(),
                node_a: n1,
                node_b: n2,
                graph_node_a: (!ground_rails.contains(&e.node_a)).then_some(e.node_a),
                graph_node_b: (!ground_rails.contains(&e.node_b)).then_some(e.node_b),
                resistance: r,
                pot_max_resistance: pot_meta.map(|(max_r, _)| max_r),
                taper: pot_meta
                    .map(|(_, taper)| taper)
                    .unwrap_or(pedalkernel_rt::pot_taper::PotTaper::B),
                invert_control,
            });
        }

        // GMIN regularization (prevent singular matrix)
        for i in 0..n_mna {
            mna.stamp_resistor(Some(i), None, 1e9);
        }

        // ── Define WDF ports ────────────────────────────────────────────
        // One port per block (at the block's coupling node) + one adapted VS port
        let mut ports = Vec::new();

        // Sub-stage ports: find each block's UNIQUE coupling node.
        //
        // For cascaded diode-connected BJTs (base=collector), adjacent blocks
        // share a node (Q1.emitter = Q2.base). Naively picking the first
        // port_node in the MNA can assign the same node to two blocks,
        // collapsing them in the scattering matrix.
        //
        // Heuristic: pick the port_node with the MOST coupling edges touching
        // it. This selects the base/collector node (where R_bias connects)
        // rather than the cascade junction.
        let mut used_ports: HashSet<NodeId> = HashSet::new();
        for (bi, block) in plan.blocks.iter().enumerate() {
            let Some(kbi) = plan_block_to_k_block.get(bi).and_then(|idx| *idx) else {
                #[cfg(test)]
                eprintln!(
                    "    block {bi}: no K-table stage; nonlinear memoryless/control block omitted from blockwise coupling ports"
                );
                continue;
            };
            if let Some(rung_ports) = differential_rung_ports(block, graph) {
                let rp = k_method_block(&sub_stages[kbi])
                    .map(|block| block.nominal_vs_rp)
                    .unwrap_or(r_source_cascade);
                for (label, node, role) in [
                    (
                        "diff_pos",
                        rung_ports.bottom_left,
                        OwnedPortRole::DifferentialPositive,
                    ),
                    (
                        "diff_neg",
                        rung_ports.bottom_right,
                        OwnedPortRole::DifferentialNegative,
                    ),
                ] {
                    let port_idx = push_coupling_port(
                        &mut ports,
                        WdfPortTerminals::single_ended(node),
                        WdfPortTerminals::maybe_single_ended(node_to_mna.get(&node).copied()),
                        rp,
                    );
                    push_k_method_port(&mut sub_stages, kbi, role, &ports, port_idx);
                    #[cfg(test)]
                    eprintln!(
                        "    block {bi}: {label}=Some({node}) → mna_node={:?}, scattering_port={port_idx}",
                        node_to_mna.get(&node)
                    );
                }
                if let (Some(&left_idx), Some(&right_idx)) = (
                    node_to_mna.get(&rung_ports.top_left),
                    node_to_mna.get(&rung_ports.top_right),
                ) {
                    let port_idx = push_coupling_port(
                        &mut ports,
                        WdfPortTerminals::differential(rung_ports.top_left, rung_ports.top_right),
                        WdfPortTerminals::differential(left_idx, right_idx),
                        rp,
                    );
                    push_k_method_port(
                        &mut sub_stages,
                        kbi,
                        OwnedPortRole::DifferentialOutput,
                        &ports,
                        port_idx,
                    );
                    #[cfg(test)]
                    eprintln!(
                        "    block {bi}: diff_out=Some(({},{})) → mna_node=Some(({left_idx},{right_idx})), scattering_port={port_idx}",
                        rung_ports.top_left, rung_ports.top_right
                    );
                }
                continue;
            }
            let preferred_node =
                block_coupling_port_node(&block.nl_edges, &block.port_nodes, graph).filter(|pn| {
                    node_to_mna.contains_key(pn)
                        && !used_ports.contains(pn)
                        && coupling_touch_count(*pn) > 0
                });

            let best_node = preferred_node.or_else(|| {
                block
                    .port_nodes
                    .iter()
                    .filter(|pn| {
                        node_to_mna.contains_key(pn)
                            && !used_ports.contains(pn)
                            && coupling_touch_count(**pn) > 0
                    })
                    .max_by_key(|&&pn| coupling_touch_count(pn))
                    .copied()
            });

            if let Some(pn) = best_node {
                let mna_idx = node_to_mna[&pn];
                let rp = k_method_block(&sub_stages[kbi])
                    .map(|block| block.nominal_vs_rp)
                    .unwrap_or(r_source_cascade);
                let port_idx = push_coupling_port(
                    &mut ports,
                    WdfPortTerminals::single_ended(pn),
                    WdfPortTerminals::single_ended(mna_idx),
                    rp,
                );
                push_k_method_port(
                    &mut sub_stages,
                    kbi,
                    OwnedPortRole::Primary,
                    &ports,
                    port_idx,
                );
                used_ports.insert(pn);
                #[cfg(test)]
                eprintln!("    block {bi}: port_node=Some({pn}) → mna_node=Some({mna_idx})");
            } else {
                // Block has no unique node in coupling — use dummy
                let port_idx = push_coupling_port(
                    &mut ports,
                    WdfPortTerminals::grounded(),
                    WdfPortTerminals::grounded(),
                    1000.0,
                );
                push_k_method_port(
                    &mut sub_stages,
                    kbi,
                    OwnedPortRole::Primary,
                    &ports,
                    port_idx,
                );
                #[cfg(test)]
                eprintln!("    block {bi}: port_node=None (no unique coupling node)");
            }
        }

        let use_coupled_solve = n_blocks >= 2
            && sub_stages.iter().all(|sub_stage| {
                let Some(ports) = k_method_ports(sub_stage) else {
                    return false;
                };
                ports
                    .iter()
                    .any(|(role, _)| *role == OwnedPortRole::DifferentialPositive)
                    && ports
                        .iter()
                        .any(|(role, _)| *role == OwnedPortRole::DifferentialNegative)
                    && ports
                        .iter()
                        .any(|(role, _)| *role == OwnedPortRole::DifferentialOutput)
            });
        let mut feedback_port_map: Vec<(usize, usize)> = Vec::new();
        let output_block_idx = n_blocks.saturating_sub(1);
        for (bi, block) in plan.blocks.iter().enumerate() {
            if use_coupled_solve {
                break;
            }
            if bi != output_block_idx {
                continue;
            }
            let Some(output_node) =
                block_cascade_output_node(&block.nl_edges, &block.port_nodes, graph)
            else {
                continue;
            };
            if used_ports.contains(&output_node) {
                continue;
            }
            if !coupling_edges.iter().any(|&eidx| {
                let e = &graph.edges[eidx];
                e.node_a == output_node || e.node_b == output_node
            }) {
                continue;
            }
            let Some(&mna_idx) = node_to_mna.get(&output_node) else {
                continue;
            };
            let rp = sub_stages
                .get(bi)
                .and_then(k_method_block)
                .map(|block| block.rp)
                .unwrap_or(r_source_cascade);
            let scattering_idx = push_coupling_port(
                &mut ports,
                WdfPortTerminals::single_ended(output_node),
                WdfPortTerminals::single_ended(mna_idx),
                rp,
            );
            feedback_port_map.push((bi, scattering_idx));
            used_ports.insert(output_node);
            #[cfg(test)]
            eprintln!(
                "    feedback port block {bi}: node={output_node} → mna_node=Some({mna_idx}), scattering_port={scattering_idx}"
            );
        }

        // ── VS ports: one per input port that connects through a coupling edge ──
        //
        // The .pedal declares input ports (audio_in, cv_cutoff, etc.) with
        // net connections to components. If a port's component is a coupling
        // edge, we add a VS port at that edge's external node in the coupling
        // MNA. The scattering matrix distributes the port voltage through
        // the resistor network. At runtime, writing to the VS port is O(1)
        // — no recompute, just update b[vs_port] before the matrix multiply.
        let mut vs_port_map: Vec<(String, usize)> = Vec::new(); // (port_name, scattering_port_idx)

        // Collect all coupling edge nodes for quick lookup
        let coupling_edge_set: HashSet<usize> = coupling_edges.iter().copied().collect();

        for port_def in port_defs {
            if port_def.direction != pedalkernel_rt::PortDirection::Input {
                continue;
            }
            // Resolve port name to graph node
            let port_node = graph.node_names.get(&port_def.name).copied();
            let port_node = match port_node {
                Some(n) => n,
                None => continue,
            };

            // Find the coupling edge connected to this port node.
            // The VS goes at the port node's end of the edge (the "external"
            // side that faces outside the coupling network).
            let edge_and_node = coupling_edges.iter().find_map(|&eidx| {
                let e = &graph.edges[eidx];
                // The port node connects to one end of a coupling edge.
                // The VS port goes at the port node itself (the external side).
                if e.node_a == port_node || e.node_b == port_node {
                    // Find the port node in MNA. If it's a rail, use the other end.
                    let mna_node = node_to_mna.get(&port_node).copied();
                    if mna_node.is_some() {
                        return Some(mna_node);
                    }
                    // Port node is a rail/external — use the other end
                    let other = if e.node_a == port_node {
                        e.node_b
                    } else {
                        e.node_a
                    };
                    Some(node_to_mna.get(&other).copied())
                } else {
                    None
                }
            });

            if let Some(Some(mna_idx)) = edge_and_node {
                let rp = port_def.impedance.unwrap_or(1.0);
                let scattering_idx = push_coupling_port(
                    &mut ports,
                    WdfPortTerminals::single_ended(port_node),
                    WdfPortTerminals::single_ended(mna_idx),
                    rp,
                );
                vs_port_map.push((port_def.name.clone(), scattering_idx));
                #[cfg(test)]
                eprintln!("    VS port '{}': mna_node={mna_idx}, Rp={rp:.0}Ω, scattering_port={scattering_idx}",
                    port_def.name);
            }
        }

        // Fallback: if no audio input port was found, add a default VS at graph.in_node
        if vs_port_map.is_empty() {
            let vs_node = node_to_mna.get(&graph.in_node).copied();
            let scattering_idx = push_coupling_port(
                &mut ports,
                WdfPortTerminals::single_ended(graph.in_node),
                WdfPortTerminals::maybe_single_ended(vs_node),
                1.0,
            );
            vs_port_map.push(("audio_in".to_string(), scattering_idx));
        }

        // Supply VS ports: VCC and other supply nodes that appear in coupling.
        // These establish DC bias current through R_bias → diodes.
        // Without them, R_bias grounds the sub-stage ports → diodes off → dead filter.
        for (supply_node, voltage) in &supply_nodes_in_coupling {
            if let Some(&mna_idx) = node_to_mna.get(supply_node) {
                let scattering_idx = push_coupling_port(
                    &mut ports,
                    WdfPortTerminals::single_ended(*supply_node),
                    WdfPortTerminals::single_ended(mna_idx),
                    1.0,
                );
                let name = format!("_supply_{}", supply_node);
                vs_port_map.push((name, scattering_idx));
                #[cfg(test)]
                eprintln!("    Supply VS: node={supply_node}, {voltage}V, scattering_port={scattering_idx}");
            }
        }

        let mut coupling_passives = Vec::new();
        let mut coupling_one_ports = Vec::new();
        let mut coupling_runtime_state = RuntimeState::new();
        for (comp_id, node_a, node_b, graph_node_a, graph_node_b, capacitance) in
            coupling_passive_specs
        {
            let rp = 1.0 / (2.0 * sample_rate * capacitance);
            let scattering_idx = push_coupling_port(
                &mut ports,
                WdfPortTerminals::maybe_differential(graph_node_a, graph_node_b),
                WdfPortTerminals::maybe_differential(node_a, node_b),
                rp,
            );
            let kind = OnePortKind::Capacitor(capacitance);
            let state_slot = coupling_runtime_state.allocate_one_port(kind);
            let one_port_idx = coupling_one_ports.len();
            coupling_one_ports.push(RuntimeOnePort::new(
                OnePort::new(
                    PortTerminals::single_ended(ScatteringPortId::new(scattering_idx)),
                    kind,
                ),
                state_slot,
            ));
            coupling_passives.push(pedalkernel_rt::stage::CouplingPassive {
                comp_id: comp_id.clone(),
                port_idx: scattering_idx,
                one_port_idx,
            });
            #[cfg(test)]
            eprintln!(
                "    passive coupling cap {comp_id}: node={node_a:?}->{node_b:?}, C={capacitance:.3e}, Rp={rp:.1}Ω, scattering_port={scattering_idx}"
            );
        }

        let output_probe_terminals = node_to_mna
            .get(&graph.out_node)
            .copied()
            .map(MnaNodeId::new)
            .map(MnaPortTerminals::single_ended);
        let output_probe_coeffs = output_probe_terminals
            .map(|terminals| {
                let (out_pos, out_neg) = terminals.raw().as_tuple();
                #[cfg(test)]
                eprintln!(
                    "    output extraction: graph.out_node={} → mna_node={out_pos:?}",
                    graph.out_node
                );
                let wdf_ports: Vec<_> = ports
                    .iter()
                    .copied()
                    .map(CircuitMappedPort::to_wdf_port)
                    .collect();
                mna.derive_node_extraction_coeffs(&wdf_ports, out_pos, out_neg)
            })
            .unwrap_or_default();
        let output_extraction = ExtractionProbe::new(output_probe_coeffs, output_probe_terminals);
        let output_port_index = None;

        let n_ports = ports.len();
        eprintln!("  [blockwise] coupling scattering: {n_ports} ports");

        // Derive scattering matrix
        let wdf_ports: Vec<_> = ports
            .iter()
            .copied()
            .map(CircuitMappedPort::to_wdf_port)
            .collect();
        let scattering = mna.derive_scattering_matrix_general(&wdf_ports);

        // Validate scattering matrix
        let all_finite = scattering.iter().all(|&v| v.is_finite());
        if !all_finite || scattering.len() != n_ports * n_ports {
            eprintln!("  [blockwise] WARNING: coupling scattering invalid, falling back");
            eprintln!("  [blockwise] done: {} total stages", all_stages.len());
            return Some(all_stages);
        }

        #[cfg(test)]
        {
            eprintln!("  [blockwise] scattering matrix ({n_ports}x{n_ports}):");
            for i in 0..n_ports {
                let row: Vec<f64> = (0..n_ports).map(|j| scattering[i * n_ports + j]).collect();
                eprintln!("    row {i}: {row:.4?}");
            }
        }

        let first_block_port_node = plan
            .blocks
            .first()
            .and_then(|block| block_coupling_port_node(&block.nl_edges, &block.port_nodes, graph));
        let all_explicit_diode_blocks = sub_stages.len() >= 2
            && sub_stages
                .iter()
                .filter_map(k_method_block)
                .all(|b| b.explicit_diode_root.is_some());
        let shared_current_ladder = all_explicit_diode_blocks || use_coupled_solve;
        let shared_diode_cutoff_pot = if shared_current_ladder {
            first_block_port_node
                .and_then(|port_node| {
                    coupling_edges.iter().find_map(|&eidx| {
                        let e = &graph.edges[eidx];
                        let comp = &graph.components[e.comp_idx];
                        let is_pot = comp
                            .kind
                            .as_any()
                            .downcast_ref::<super::components::Potentiometer>()
                            .is_some();
                        let touches_first_port = e.node_a == port_node || e.node_b == port_node;
                        let touches_feedback_output = output_feedback_node
                            .map(|node| e.node_a == node || e.node_b == node)
                            .unwrap_or(false);

                        if is_pot && touches_first_port && !touches_feedback_output {
                            Some(comp.id.clone())
                        } else {
                            None
                        }
                    })
                })
                .or_else(|| {
                    coupling_edges.iter().find_map(|&eidx| {
                        let comp = &graph.components[graph.edges[eidx].comp_idx];
                        let is_pot = comp
                            .kind
                            .as_any()
                            .downcast_ref::<super::components::Potentiometer>()
                            .is_some();
                        if is_pot && comp.id.to_ascii_lowercase().contains("cutoff") {
                            Some(comp.id.clone())
                        } else {
                            None
                        }
                    })
                })
                .or_else(|| {
                    graph.components.iter().find_map(|comp| {
                        let is_pot = comp
                            .kind
                            .as_any()
                            .downcast_ref::<super::components::Potentiometer>()
                            .is_some();
                        if is_pot && comp.id.to_ascii_lowercase().contains("cutoff") {
                            Some(comp.id.clone())
                        } else {
                            None
                        }
                    })
                })
        } else {
            None
        };
        let cutoff_cv_port = if shared_current_ladder {
            first_block_port_node
                .and_then(|port_node| {
                    port_defs.iter().find_map(|port_def| {
                        if port_def.direction != pedalkernel_rt::PortDirection::Input {
                            return None;
                        }
                        let lower_name = port_def.name.to_ascii_lowercase();
                        if !lower_name.contains("cv") || lower_name.contains("resonance") {
                            return None;
                        }
                        let external_port_node = graph.node_names.get(&port_def.name).copied()?;
                        let drives_first_port = coupling_edges.iter().any(|&eidx| {
                            let e = &graph.edges[eidx];
                            (e.node_a == external_port_node && e.node_b == port_node)
                                || (e.node_b == external_port_node && e.node_a == port_node)
                        });
                        if drives_first_port {
                            Some(port_def.name.clone())
                        } else {
                            None
                        }
                    })
                })
                .or_else(|| {
                    port_defs.iter().find_map(|port_def| {
                        if port_def.direction == pedalkernel_rt::PortDirection::Input {
                            let lower_name = port_def.name.to_ascii_lowercase();
                            if lower_name.contains("cutoff") && lower_name.contains("cv") {
                                return Some(port_def.name.clone());
                            }
                        }
                        None
                    })
                })
        } else {
            None
        };

        if let Some(ref comp_id) = shared_diode_cutoff_pot {
            let already_tracked = coupling_elements
                .iter()
                .any(|element| element.comp_id == *comp_id);
            if !already_tracked {
                if let Some(comp) = graph.components.iter().find(|comp| comp.id == *comp_id) {
                    if let Some(pot) = comp
                        .kind
                        .as_any()
                        .downcast_ref::<super::components::Potentiometer>()
                    {
                        coupling_elements.push(pedalkernel_rt::stage::CouplingElement {
                            comp_id: comp.id.clone(),
                            node_a: None,
                            node_b: None,
                            graph_node_a: None,
                            graph_node_b: None,
                            resistance: (pot.max_r * 0.5).max(1.0e-3),
                            pot_max_resistance: Some(pot.max_r),
                            taper: pot.taper,
                            invert_control: false,
                        });
                    }
                }
            }
        }

        if let Some(ref cutoff_name) = cutoff_cv_port {
            if !vs_port_map.iter().any(|(name, _)| name == cutoff_name) {
                // The TB-303 cutoff CV can enter through an exponential
                // converter outside the ladder coupling network. Keep it in
                // the blockwise source map as a control-only input so the shared
                // tail-current axis updates without stamping a synthetic
                // voltage source into the ladder scattering matrix.
                vs_port_map.push((cutoff_name.clone(), n_ports));
            }
        }

        // TODO: Compute per-block source impedance from circuit topology.
        //
        // The VS Rp = 1Ω (default) makes the source near-ideal, which
        // prevents the RC filter from working (gamma ≈ 0). The correct
        // source impedance comes from:
        // - Block 0: R_in from the DSL port impedance declaration
        // - Blocks 1..N: previous emitter follower output Z (~1/gm)
        //
        // The coupling scattering diagonal S[i][i] gives the TOTAL
        // Thevenin impedance including DC bias (R_bias → VCC), which
        // is wrong for AC source impedance. Need AC-only analysis.
        //
        // For now: use port impedance from DSL if available, else keep
        // default VS Rp = 1Ω. The DSL extension `audio_in: input(10k)`
        // is implemented and ready to use.

        // ── Package as BlockwiseStage ────────────────────────────────────
        let output_block = n_blocks - 1;
        // Keep the specialized diode ladder core disabled by default. The real
        // Blockwise path must use the coupled block solve and coupling matrix; the
        // core shortcut bypasses resonance/feedback coupling and is only a
        // future explicit optimization target.
        let diode_ladder_core = None;
        let mut coupling_passive_by_port = vec![None; n_ports];
        for (passive_idx, passive) in coupling_passives.iter().enumerate() {
            if passive.port_idx < n_ports {
                coupling_passive_by_port[passive.port_idx] = Some(passive_idx);
            }
        }
        let bkm = pedalkernel_rt::stage::BlockwiseStage {
            coupling_s: scattering,
            coupling_n_mna: n_mna,
            coupling_ports: ports,
            sub_stages,
            coupling_elements,
            coupling_passives,
            coupling_one_ports,
            coupling_runtime_state,
            coupling_passive_by_port,
            coupling_vcvss,
            n_ports,
            output_block,
            output_port_index,
            output_extraction,
            supply_voltage,
            vs_port_map,
            cutoff_cv_port,
            shared_diode_cutoff_pot,
            feedback_port_map,
            compensation: 1.0,
            oversampler: pedalkernel_rt::oversampling::Oversampler::new(
                pedalkernel_rt::oversampling::OversamplingFactor::X1,
            ),
            // Blockwise stage should process after input/source impedance groups
            // but before output coupling (C_out). The feedback flow distance
            // formula inflates distance for ladders, so keep this near the
            // front without outrunning the input port conditioning stage.
            signal_flow_distance: 1,
            bypass_serial: false,
            solve_mode: match formulation_selection.coupling {
                CouplingFormulation::BlockwiseCoupled { coupled_newton }
                    if use_coupled_solve && coupled_newton =>
                {
                    pedalkernel_rt::stage::BlockwiseSolveMode::CoupledNewton
                }
                CouplingFormulation::BlockwiseCoupled { .. } if use_coupled_solve => {
                    pedalkernel_rt::stage::BlockwiseSolveMode::CoupledFixedPoint
                }
                _ => pedalkernel_rt::stage::BlockwiseSolveMode::Cascade,
            },
            diode_ladder_core,
            b_warm: vec![0.0; n_ports],
            work_b: vec![0.0; n_ports],
            work_a: vec![0.0; n_ports],
            coupled_scratch: pedalkernel_rt::stage::CoupledSolveScratch::new(n_ports),
            port_index_cache: Vec::new(),
        };

        eprintln!(
            "  [blockwise] built BlockwiseStage: {} blocks, {} ports",
            bkm.block_count(),
            bkm.n_ports
        );

        // Replace the rung-local stages with the single blockwise stage, but keep
        // explicit multi-boundary WDF blocks such as input differential pairs.
        let mut boundary_wdf_stages = Vec::new();
        for (stage, bi) in all_stages.drain(..).zip(all_stage_plan_blocks.drain(..)) {
            let keep_boundary_wdf = matches!(
                plan.blocks.get(bi).map(|block| &block.topology),
                Some(BlockTopology::InputDifferentialPair { .. })
            ) && matches!(&stage, BuiltStage::Wdf(wdf) if !wdf.boundary_bindings.is_empty());
            if keep_boundary_wdf {
                boundary_wdf_stages.push(stage);
            }
        }
        all_stages = boundary_wdf_stages;
        all_stages.push(BuiltStage::Blockwise(bkm));
    }

    eprintln!("  [blockwise] done: {} total stages", all_stages.len());
    Some(all_stages)
}
