//! Phase 1: Directed graph construction from the `.pedal` netlist.
//!
//! Builds a [`LayoutGraph`] where each component is a node and each net
//! creates edges between the components it connects. Edge direction is
//! inferred from component type and pin function (e.g., triode grid = input,
//! plate = output).

use pedalkernel::dsl::{ComponentDef, ComponentKind, NetDef, PedalDef, Pin};
use std::collections::{HashMap, HashSet};

// ---------------------------------------------------------------------------
// Pin direction inference
// ---------------------------------------------------------------------------

/// Inferred direction for a component pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinDirection {
    /// Signal enters through this pin (e.g., triode grid).
    Input,
    /// Signal exits through this pin (e.g., triode plate).
    Output,
    /// Pin connects upward toward VCC (e.g., plate load destination).
    Up,
    /// Pin connects downward toward GND (e.g., cathode bias).
    Down,
    /// Direction determined by context (e.g., resistor terminals).
    Bidirectional,
}

/// Infer the signal direction for a given pin on a component.
pub fn infer_pin_direction(kind: &ComponentKind, pin_name: &str) -> PinDirection {
    match kind {
        // Triode: grid=in, plate=out, cathode=down (bias hangs below)
        ComponentKind::Triode(_) => match pin_name {
            "grid" => PinDirection::Input,
            "plate" => PinDirection::Output,
            "cathode" => PinDirection::Down,
            _ => PinDirection::Bidirectional,
        },
        // Pentode: grid=in, plate=out, screen=up (to supply), cathode=down
        ComponentKind::Pentode(_) => match pin_name {
            "grid" => PinDirection::Input,
            "plate" => PinDirection::Output,
            "screen" => PinDirection::Up,
            "cathode" => PinDirection::Down,
            _ => PinDirection::Bidirectional,
        },
        // Transformer: primary=input side, secondary=output side
        ComponentKind::Transformer(_) => {
            if pin_name.starts_with("pri") {
                PinDirection::Input
            } else if pin_name.starts_with("sec") {
                PinDirection::Output
            } else {
                PinDirection::Bidirectional
            }
        }
        // Potentiometer: wiper=output
        ComponentKind::Potentiometer(_) => match pin_name {
            "wiper" | "b" => PinDirection::Output,
            "a" => PinDirection::Input,
            _ => PinDirection::Bidirectional,
        },
        // Op-amp: pos/neg=input, out=output
        ComponentKind::OpAmp(_) => match pin_name {
            "pos" | "neg" => PinDirection::Input,
            "out" => PinDirection::Output,
            _ => PinDirection::Bidirectional,
        },
        // BJTs: base=input, collector=output, emitter=down
        ComponentKind::Npn(_) | ComponentKind::Pnp(_) => match pin_name {
            "base" => PinDirection::Input,
            "collector" => PinDirection::Output,
            "emitter" => PinDirection::Down,
            _ => PinDirection::Bidirectional,
        },
        // JFETs: gate=input, drain=output, source=down
        ComponentKind::NJfet(_) | ComponentKind::PJfet(_) => match pin_name {
            "gate" => PinDirection::Input,
            "drain" => PinDirection::Output,
            "source" => PinDirection::Down,
            _ => PinDirection::Bidirectional,
        },
        // MOSFETs: gate=input, drain=output, source=down
        ComponentKind::Nmos(_) | ComponentKind::Pmos(_) => match pin_name {
            "gate" => PinDirection::Input,
            "drain" => PinDirection::Output,
            "source" => PinDirection::Down,
            _ => PinDirection::Bidirectional,
        },
        // Diodes: anode=input, cathode=output (current flow direction)
        ComponentKind::Diode(_) | ComponentKind::DiodePair(_) | ComponentKind::Zener(_) => {
            match pin_name {
                "a" => PinDirection::Input,
                "b" => PinDirection::Output,
                _ => PinDirection::Bidirectional,
            }
        }
        // Two-terminal passives: direction from context
        ComponentKind::Resistor(_)
        | ComponentKind::Capacitor(_)
        | ComponentKind::Inductor(_) => PinDirection::Bidirectional,
        // Everything else: bidirectional
        _ => PinDirection::Bidirectional,
    }
}

// ---------------------------------------------------------------------------
// Layout graph
// ---------------------------------------------------------------------------

/// A node in the layout graph (one per component + special anchor nodes).
#[derive(Debug, Clone)]
pub struct LayoutNode {
    /// Index into `LayoutGraph::nodes`.
    pub id: usize,
    /// Component ID (or `"__in"`, `"__out"`, `"__vcc"`, `"__gnd"` for anchors).
    pub comp_id: String,
    /// Component definition (None for anchor nodes).
    pub comp: Option<ComponentDef>,
    /// Whether this is an anchor node (`in`, `out`, `vcc`, `gnd`).
    pub is_anchor: bool,
}

/// A directed edge between two layout nodes.
#[derive(Debug, Clone)]
pub struct LayoutEdge {
    pub from: usize,
    pub to: usize,
    /// Net name (if any).
    pub net_name: Option<String>,
    /// Whether this edge is on the primary signal path.
    pub signal_path: bool,
    /// Whether this is a supply connection (to vcc/gnd).
    pub is_supply: bool,
    /// Whether this is a feedback edge (back-edge in topological sort).
    pub is_feedback: bool,
}

/// Directed graph built from the `.pedal` netlist for layout purposes.
#[derive(Debug)]
pub struct LayoutGraph {
    pub nodes: Vec<LayoutNode>,
    pub edges: Vec<LayoutEdge>,
    /// Map from component ID to node index.
    pub id_to_node: HashMap<String, usize>,
    /// Index of the `in` anchor node.
    pub in_node: usize,
    /// Index of the `out` anchor node.
    pub out_node: usize,
    /// Index of the `vcc` anchor node.
    pub vcc_node: usize,
    /// Index of the `gnd` anchor node.
    pub gnd_node: usize,
    /// Merged net groups: each group is a set of pins that are connected.
    pub net_groups: Vec<NetGroup>,
    /// Monitor component IDs from the pedal definition.
    pub monitor_ids: Vec<String>,
}

/// A group of pins that are all connected (one electrical net).
#[derive(Debug, Clone)]
pub struct NetGroup {
    /// Net index (0-based).
    pub index: usize,
    /// Optional net name (from reserved pins or named nodes).
    pub name: Option<String>,
    /// All pins in this net.
    pub pins: Vec<Pin>,
    /// Component IDs that this net touches.
    pub component_ids: Vec<String>,
}

impl LayoutGraph {
    /// Build a layout graph from a parsed pedal definition.
    pub fn from_pedal(pedal: &PedalDef) -> Self {
        let mut nodes = Vec::new();
        let mut id_to_node = HashMap::new();

        // Create anchor nodes for in, out, vcc, gnd
        let anchors = ["__in", "__out", "__vcc", "__gnd"];
        for name in &anchors {
            let id = nodes.len();
            id_to_node.insert(name.to_string(), id);
            nodes.push(LayoutNode {
                id,
                comp_id: name.to_string(),
                comp: None,
                is_anchor: true,
            });
        }
        let in_node = id_to_node["__in"];
        let out_node = id_to_node["__out"];
        let vcc_node = id_to_node["__vcc"];
        let gnd_node = id_to_node["__gnd"];

        // Create a node for each component
        for comp in &pedal.components {
            let id = nodes.len();
            id_to_node.insert(comp.id.clone(), id);
            nodes.push(LayoutNode {
                id,
                comp_id: comp.id.clone(),
                comp: Some(comp.clone()),
                is_anchor: false,
            });
        }

        // Build merged net groups using union-find on pin connectivity
        let net_groups = build_net_groups(&pedal.nets);

        // Build edges from net groups
        let edges = build_edges(&net_groups, &nodes, &id_to_node, pedal);

        // Collect monitor IDs
        let monitor_ids = pedal.monitors.iter().map(|m| m.component.clone()).collect();

        LayoutGraph {
            nodes,
            edges,
            id_to_node,
            in_node,
            out_node,
            vcc_node,
            gnd_node,
            net_groups,
            monitor_ids,
        }
    }

    /// Get all outgoing edges from a node.
    pub fn outgoing(&self, node_id: usize) -> Vec<&LayoutEdge> {
        self.edges.iter().filter(|e| e.from == node_id).collect()
    }

    /// Get all incoming edges to a node.
    pub fn incoming(&self, node_id: usize) -> Vec<&LayoutEdge> {
        self.edges.iter().filter(|e| e.to == node_id).collect()
    }

    /// Get all neighbors (both directions) of a node.
    pub fn neighbors(&self, node_id: usize) -> HashSet<usize> {
        let mut result = HashSet::new();
        for e in &self.edges {
            if e.from == node_id {
                result.insert(e.to);
            }
            if e.to == node_id {
                result.insert(e.from);
            }
        }
        result
    }

    /// Check if a node connects to vcc (directly or through supply components).
    pub fn connects_to_vcc(&self, node_id: usize) -> bool {
        self.edges
            .iter()
            .any(|e| (e.from == node_id && e.to == self.vcc_node) || (e.to == node_id && e.from == self.vcc_node))
    }

    /// Check if a node connects to gnd (directly or through ground components).
    pub fn connects_to_gnd(&self, node_id: usize) -> bool {
        self.edges
            .iter()
            .any(|e| (e.from == node_id && e.to == self.gnd_node) || (e.to == node_id && e.from == self.gnd_node))
    }

    /// Get the component kind for a node, if it has one.
    pub fn node_kind(&self, node_id: usize) -> Option<&ComponentKind> {
        self.nodes[node_id].comp.as_ref().map(|c| &c.kind)
    }

    /// Check if a node is an active device (tube, transistor, op-amp).
    pub fn is_active_device(&self, node_id: usize) -> bool {
        matches!(
            self.node_kind(node_id),
            Some(
                ComponentKind::Triode(_)
                    | ComponentKind::Pentode(_)
                    | ComponentKind::Npn(_)
                    | ComponentKind::Pnp(_)
                    | ComponentKind::NJfet(_)
                    | ComponentKind::PJfet(_)
                    | ComponentKind::Nmos(_)
                    | ComponentKind::Pmos(_)
                    | ComponentKind::OpAmp(_)
            )
        )
    }

    /// Check if a component is a passive (R, C, L).
    pub fn is_passive(&self, node_id: usize) -> bool {
        matches!(
            self.node_kind(node_id),
            Some(
                ComponentKind::Resistor(_)
                    | ComponentKind::Capacitor(_)
                    | ComponentKind::Inductor(_)
            )
        )
    }
}

// ---------------------------------------------------------------------------
// Net group construction
// ---------------------------------------------------------------------------

/// Map a reserved pin name to the corresponding anchor node ID.
fn reserved_to_anchor(name: &str) -> Option<&'static str> {
    match name {
        "in" => Some("__in"),
        "out" => Some("__out"),
        "vcc" => Some("__vcc"),
        "gnd" => Some("__gnd"),
        _ => None,
    }
}

/// Extract the component ID from a pin reference.
fn pin_component_id(pin: &Pin) -> Option<String> {
    match pin {
        Pin::Reserved(name) => reserved_to_anchor(name).map(|s| s.to_string()),
        Pin::ComponentPin { component, .. } => Some(component.clone()),
        // For Fork, use the switch component as the main reference
        Pin::Fork { switch, .. } => Some(switch.clone()),
    }
}

fn build_net_groups(nets: &[NetDef]) -> Vec<NetGroup> {
    // Merge nets that share pins (same logic as kicad.rs build_net_map)
    let mut groups: Vec<Vec<Pin>> = Vec::new();

    for net in nets {
        let mut all_pins: Vec<Pin> = vec![net.from.clone()];
        all_pins.extend(net.to.iter().cloned());

        let mut merge_indices: Vec<usize> = Vec::new();
        for (i, group) in groups.iter().enumerate() {
            if all_pins.iter().any(|p| group.contains(p)) {
                merge_indices.push(i);
            }
        }

        if merge_indices.is_empty() {
            groups.push(all_pins);
        } else {
            merge_indices.sort();
            let target = merge_indices[0];
            for &idx in merge_indices.iter().skip(1).rev() {
                let g = groups.remove(idx);
                groups[target].extend(g);
            }
            for p in all_pins {
                if !groups[target].contains(&p) {
                    groups[target].push(p);
                }
            }
        }
    }

    groups
        .into_iter()
        .enumerate()
        .map(|(i, pins)| {
            let name = pins.iter().find_map(|p| {
                if let Pin::Reserved(n) = p {
                    Some(n.clone())
                } else {
                    None
                }
            });
            let mut comp_ids: Vec<String> = pins.iter().filter_map(|p| pin_component_id(p)).collect();
            comp_ids.sort();
            comp_ids.dedup();
            NetGroup {
                index: i,
                name,
                pins,
                component_ids: comp_ids,
            }
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Edge construction
// ---------------------------------------------------------------------------

fn build_edges(
    net_groups: &[NetGroup],
    nodes: &[LayoutNode],
    id_to_node: &HashMap<String, usize>,
    pedal: &PedalDef,
) -> Vec<LayoutEdge> {
    let mut edges = Vec::new();
    let mut seen_pairs: HashSet<(usize, usize)> = HashSet::new();

    // For each net group, create edges between all component pairs in the net
    for ng in net_groups {
        let node_ids: Vec<usize> = ng
            .component_ids
            .iter()
            .filter_map(|cid| id_to_node.get(cid).copied())
            .collect();

        let is_supply_net = ng.name.as_deref() == Some("vcc") || ng.name.as_deref() == Some("gnd");
        let is_signal = ng.name.as_deref() == Some("in") || ng.name.as_deref() == Some("out");

        // Create directed edges based on pin direction inference
        for &from_id in &node_ids {
            for &to_id in &node_ids {
                if from_id == to_id {
                    continue;
                }

                // Determine direction from pin roles
                let direction = infer_edge_direction(
                    from_id, to_id, ng, nodes, pedal,
                );

                match direction {
                    EdgeDir::Forward => {
                        if seen_pairs.insert((from_id, to_id)) {
                            edges.push(LayoutEdge {
                                from: from_id,
                                to: to_id,
                                net_name: ng.name.clone(),
                                signal_path: is_signal,
                                is_supply: is_supply_net,
                                is_feedback: false,
                            });
                        }
                    }
                    EdgeDir::Backward => {
                        if seen_pairs.insert((to_id, from_id)) {
                            edges.push(LayoutEdge {
                                from: to_id,
                                to: from_id,
                                net_name: ng.name.clone(),
                                signal_path: is_signal,
                                is_supply: is_supply_net,
                                is_feedback: false,
                            });
                        }
                    }
                    EdgeDir::Both => {
                        // For bidirectional, pick the lower-id → higher-id direction
                        let (a, b) = if from_id < to_id {
                            (from_id, to_id)
                        } else {
                            (to_id, from_id)
                        };
                        if seen_pairs.insert((a, b)) {
                            edges.push(LayoutEdge {
                                from: a,
                                to: b,
                                net_name: ng.name.clone(),
                                signal_path: is_signal,
                                is_supply: is_supply_net,
                                is_feedback: false,
                            });
                        }
                    }
                }
            }
        }
    }

    // Mark feedback edges (back-edges in topological order)
    mark_feedback_edges(&mut edges, nodes.len(), id_to_node.get("__in").copied());

    edges
}

enum EdgeDir {
    Forward,
    Backward,
    Both,
}

fn infer_edge_direction(
    from_id: usize,
    to_id: usize,
    ng: &NetGroup,
    nodes: &[LayoutNode],
    _pedal: &PedalDef,
) -> EdgeDir {
    let from_node = &nodes[from_id];
    let to_node = &nodes[to_id];

    // Check pin directions for from_node in this net
    let from_dir = get_pin_direction_in_net(from_node, ng);
    let to_dir = get_pin_direction_in_net(to_node, ng);

    match (from_dir, to_dir) {
        (PinDirection::Output, PinDirection::Input) => EdgeDir::Forward,
        (PinDirection::Input, PinDirection::Output) => EdgeDir::Backward,
        (PinDirection::Output, _) => EdgeDir::Forward,
        (_, PinDirection::Input) => EdgeDir::Forward,
        (PinDirection::Input, _) => EdgeDir::Backward,
        (_, PinDirection::Output) => EdgeDir::Backward,
        _ => EdgeDir::Both,
    }
}

fn get_pin_direction_in_net(node: &LayoutNode, ng: &NetGroup) -> PinDirection {
    if node.is_anchor {
        return match node.comp_id.as_str() {
            "__in" => PinDirection::Output, // 'in' anchor sends signal out
            "__out" => PinDirection::Input,  // 'out' anchor receives signal
            "__vcc" => PinDirection::Up,
            "__gnd" => PinDirection::Down,
            _ => PinDirection::Bidirectional,
        };
    }

    let comp = match &node.comp {
        Some(c) => c,
        None => return PinDirection::Bidirectional,
    };

    // Find which pin of this component appears in the net
    for pin in &ng.pins {
        if let Pin::ComponentPin { component, pin: pin_name } = pin {
            if *component == node.comp_id {
                return infer_pin_direction(&comp.kind, pin_name);
            }
        }
    }

    PinDirection::Bidirectional
}

/// Mark back-edges in the graph (feedback loops) using DFS from the input.
fn mark_feedback_edges(edges: &mut [LayoutEdge], num_nodes: usize, start: Option<usize>) {
    let start = match start {
        Some(s) => s,
        None => return,
    };

    // Build adjacency list
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); num_nodes];
    for (i, e) in edges.iter().enumerate() {
        adj[e.from].push(i);
    }

    // DFS to find back-edges
    let mut visited = vec![false; num_nodes];
    let mut in_stack = vec![false; num_nodes];
    let mut stack = vec![(start, 0usize)];
    visited[start] = true;
    in_stack[start] = true;

    while let Some((node, edge_idx)) = stack.last_mut() {
        let node = *node;
        if *edge_idx >= adj[node].len() {
            in_stack[node] = false;
            stack.pop();
            continue;
        }
        let ei = adj[node][*edge_idx];
        *edge_idx += 1;
        let target = edges[ei].to;
        if in_stack[target] {
            // Back-edge found — mark as feedback
            edges[ei].is_feedback = true;
        } else if !visited[target] {
            visited[target] = true;
            in_stack[target] = true;
            stack.push((target, 0));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pedalkernel::dsl::*;

    fn simple_pedal() -> PedalDef {
        PedalDef {
            name: "Test".into(),
            supplies: vec![],
            components: vec![
                ComponentDef { id: "R1".into(), kind: ComponentKind::Resistor(4700.0) },
                ComponentDef { id: "C1".into(), kind: ComponentKind::Capacitor(100e-9) },
            ],
            nets: vec![
                NetDef {
                    from: Pin::Reserved("in".into()),
                    to: vec![Pin::ComponentPin { component: "C1".into(), pin: "a".into() }],
                },
                NetDef {
                    from: Pin::ComponentPin { component: "C1".into(), pin: "b".into() },
                    to: vec![Pin::ComponentPin { component: "R1".into(), pin: "a".into() }],
                },
                NetDef {
                    from: Pin::ComponentPin { component: "R1".into(), pin: "b".into() },
                    to: vec![Pin::Reserved("out".into())],
                },
            ],
            controls: vec![],
            trims: vec![],
            monitors: vec![],
        }
    }

    #[test]
    fn graph_has_correct_node_count() {
        let g = LayoutGraph::from_pedal(&simple_pedal());
        // 4 anchors + 2 components = 6
        assert_eq!(g.nodes.len(), 6);
    }

    #[test]
    fn graph_has_edges() {
        let g = LayoutGraph::from_pedal(&simple_pedal());
        assert!(!g.edges.is_empty());
    }

    #[test]
    fn pin_direction_triode() {
        assert_eq!(
            infer_pin_direction(&ComponentKind::Triode(TriodeType::T12ax7), "grid"),
            PinDirection::Input
        );
        assert_eq!(
            infer_pin_direction(&ComponentKind::Triode(TriodeType::T12ax7), "plate"),
            PinDirection::Output
        );
        assert_eq!(
            infer_pin_direction(&ComponentKind::Triode(TriodeType::T12ax7), "cathode"),
            PinDirection::Down
        );
    }

    #[test]
    fn pin_direction_opamp() {
        assert_eq!(
            infer_pin_direction(&ComponentKind::OpAmp(OpAmpType::Jrc4558), "pos"),
            PinDirection::Input
        );
        assert_eq!(
            infer_pin_direction(&ComponentKind::OpAmp(OpAmpType::Jrc4558), "out"),
            PinDirection::Output
        );
    }
}
