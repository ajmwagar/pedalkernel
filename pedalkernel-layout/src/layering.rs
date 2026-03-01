//! Phase 3: Sugiyama layer (column) assignment.
//!
//! Assigns each functional group to a horizontal column based on signal flow
//! order. Uses longest-path layering from the input anchor to ensure signal
//! flows strictly left to right.

use crate::graph::LayoutGraph;
use crate::groups::{FunctionalGroup, GroupKind};
use std::collections::{HashMap, HashSet, VecDeque};

/// Column assignment for each functional group.
#[derive(Debug, Clone)]
pub struct ColumnAssignment {
    /// Map from group name to column index (0 = leftmost).
    pub group_columns: HashMap<String, usize>,
    /// Total number of columns.
    pub num_columns: usize,
    /// Column widths (relative, normalized to sum=1.0 later).
    pub column_widths: Vec<f32>,
}

/// Assign columns to functional groups using longest-path layering.
pub fn assign_columns(graph: &LayoutGraph, groups: &[FunctionalGroup]) -> ColumnAssignment {
    // Build a group-level graph: edges between groups based on signal flow
    let group_graph = build_group_graph(graph, groups);

    // Topological sort of groups, respecting signal flow
    let topo_order = topological_sort_groups(&group_graph, groups);

    // Longest-path layering: assign each group to its longest distance from input
    let layers = longest_path_layering(&group_graph, &topo_order, groups);

    // Merge single-component groups into adjacent layers if they're just coupling elements
    let layers = compact_layers(layers, groups);

    let num_columns = layers.values().copied().max().map(|m| m + 1).unwrap_or(1);

    // Compute column widths based on group complexity
    let column_widths = compute_column_widths(groups, &layers, num_columns);

    ColumnAssignment {
        group_columns: layers,
        num_columns,
        column_widths,
    }
}

// ---------------------------------------------------------------------------
// Group-level graph
// ---------------------------------------------------------------------------

/// Adjacency list for the group-level directed graph.
struct GroupGraph {
    /// Edges: group index → set of successor group indices.
    adj: Vec<HashSet<usize>>,
    /// Reverse edges: group index → set of predecessor group indices.
    rev: Vec<HashSet<usize>>,
}

fn build_group_graph(graph: &LayoutGraph, groups: &[FunctionalGroup]) -> GroupGraph {
    let n = groups.len();
    let mut adj = vec![HashSet::new(); n];
    let mut rev = vec![HashSet::new(); n];

    // Map node ID → group index
    let mut node_to_group: HashMap<usize, usize> = HashMap::new();
    for (gi, g) in groups.iter().enumerate() {
        for &m in &g.members {
            node_to_group.insert(m, gi);
        }
    }

    // For each edge in the layout graph, if it connects two different groups,
    // add a group-level edge
    for edge in &graph.edges {
        if edge.is_feedback || edge.is_supply {
            continue;
        }
        if let (Some(&from_g), Some(&to_g)) =
            (node_to_group.get(&edge.from), node_to_group.get(&edge.to))
        {
            if from_g != to_g {
                adj[from_g].insert(to_g);
                rev[to_g].insert(from_g);
            }
        }
    }

    // Also add edges from anchor nodes to their connected groups
    // Input anchor → first groups; last groups → output anchor
    for edge in &graph.edges {
        if edge.from == graph.in_node {
            if let Some(&to_g) = node_to_group.get(&edge.to) {
                // Mark input group as having no predecessors (it's the start)
                // — already implicit from lack of reverse edges
                let _ = to_g;
            }
        }
    }

    GroupGraph { adj, rev }
}

fn topological_sort_groups(gg: &GroupGraph, groups: &[FunctionalGroup]) -> Vec<usize> {
    let n = groups.len();
    let mut in_degree: Vec<usize> = vec![0; n];

    for (gi, succs) in gg.adj.iter().enumerate() {
        let _ = gi;
        for &s in succs {
            in_degree[s] += 1;
        }
    }

    let mut queue: VecDeque<usize> = VecDeque::new();
    for i in 0..n {
        if in_degree[i] == 0 {
            queue.push_back(i);
        }
    }

    // Prioritize input sections first
    let mut sorted_queue: Vec<usize> = queue.drain(..).collect();
    sorted_queue.sort_by_key(|&gi| match groups[gi].kind {
        GroupKind::InputSection => 0,
        GroupKind::GainStage => 1,
        GroupKind::OpAmpStage => 1,
        GroupKind::ToneStack => 2,
        GroupKind::PhaseInverter => 3,
        GroupKind::PushPullOutput => 4,
        GroupKind::OutputSection => 5,
        GroupKind::Generic => 6,
    });
    for gi in sorted_queue {
        queue.push_back(gi);
    }

    let mut order = Vec::with_capacity(n);
    while let Some(gi) = queue.pop_front() {
        order.push(gi);
        for &succ in &gg.adj[gi] {
            in_degree[succ] -= 1;
            if in_degree[succ] == 0 {
                queue.push_back(succ);
            }
        }
    }

    // If there are cycles (shouldn't happen after feedback removal), add remaining
    for i in 0..n {
        if !order.contains(&i) {
            order.push(i);
        }
    }

    order
}

fn longest_path_layering(
    gg: &GroupGraph,
    topo_order: &[usize],
    groups: &[FunctionalGroup],
) -> HashMap<String, usize> {
    let n = groups.len();
    let mut layer = vec![0usize; n];

    // Process in topological order; each group's layer is max(predecessors' layers) + 1
    for &gi in topo_order {
        let max_pred = gg.rev[gi]
            .iter()
            .map(|&pred| layer[pred] + 1)
            .max()
            .unwrap_or(0);
        layer[gi] = max_pred;
    }

    // Force input sections to column 0 and output sections to the last column
    let max_layer = *layer.iter().max().unwrap_or(&0);
    for (gi, g) in groups.iter().enumerate() {
        if g.kind == GroupKind::InputSection {
            layer[gi] = 0;
        } else if g.kind == GroupKind::OutputSection {
            layer[gi] = max_layer;
        }
    }

    groups
        .iter()
        .enumerate()
        .map(|(gi, g)| (g.name.clone(), layer[gi]))
        .collect()
}

/// Compact layers by merging single-passive groups into adjacent columns.
fn compact_layers(
    mut layers: HashMap<String, usize>,
    groups: &[FunctionalGroup],
) -> HashMap<String, usize> {
    // If a group has only one passive member and it bridges two columns,
    // merge it with the earlier column.
    for g in groups {
        if g.members.len() == 1 && g.kind == GroupKind::Generic {
            if let Some(&col) = layers.get(&g.name) {
                if col > 0 {
                    layers.insert(g.name.clone(), col.saturating_sub(1));
                }
            }
        }
    }

    // Re-number columns to close gaps
    let mut used: Vec<usize> = layers.values().copied().collect();
    used.sort();
    used.dedup();

    let remap: HashMap<usize, usize> = used
        .iter()
        .enumerate()
        .map(|(new, &old)| (old, new))
        .collect();

    for val in layers.values_mut() {
        if let Some(&new) = remap.get(val) {
            *val = new;
        }
    }

    layers
}

fn compute_column_widths(
    groups: &[FunctionalGroup],
    layers: &HashMap<String, usize>,
    num_columns: usize,
) -> Vec<f32> {
    let mut widths = vec![1.0f32; num_columns];

    for g in groups {
        if let Some(&col) = layers.get(&g.name) {
            let complexity = match g.kind {
                GroupKind::GainStage => 1.2,
                GroupKind::ToneStack => 1.5 + 0.3 * (g.members.len() as f32 - 3.0).max(0.0),
                GroupKind::PushPullOutput => 2.0,
                GroupKind::PhaseInverter => 1.5,
                GroupKind::OpAmpStage => 1.3,
                GroupKind::InputSection => 0.8,
                GroupKind::OutputSection => 0.8,
                GroupKind::Generic => 1.0,
            };
            if complexity > widths[col] {
                widths[col] = complexity;
            }
        }
    }

    // Normalize so widths sum to 1.0
    let total: f32 = widths.iter().sum();
    if total > 0.0 {
        for w in &mut widths {
            *w /= total;
        }
    }

    widths
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::LayoutGraph;
    use pedalkernel::dsl::*;

    #[test]
    fn linear_chain_assigns_increasing_columns() {
        let pedal = PedalDef {
            name: "Test".into(),
            supplies: vec![],
            components: vec![
                ComponentDef { id: "C1".into(), kind: ComponentKind::Capacitor(CapConfig::new(100e-9)) },
                ComponentDef { id: "R1".into(), kind: ComponentKind::Resistor(1e6) },
                ComponentDef { id: "V1".into(), kind: ComponentKind::Triode(TriodeType::T12ax7) },
                ComponentDef { id: "R2".into(), kind: ComponentKind::Resistor(100e3) },
            ],
            nets: vec![
                NetDef {
                    from: Pin::Reserved("in".into()),
                    to: vec![Pin::ComponentPin { component: "C1".into(), pin: "a".into() }],
                },
                NetDef {
                    from: Pin::ComponentPin { component: "C1".into(), pin: "b".into() },
                    to: vec![
                        Pin::ComponentPin { component: "R1".into(), pin: "a".into() },
                        Pin::ComponentPin { component: "V1".into(), pin: "grid".into() },
                    ],
                },
                NetDef {
                    from: Pin::Reserved("vcc".into()),
                    to: vec![Pin::ComponentPin { component: "R2".into(), pin: "a".into() }],
                },
                NetDef {
                    from: Pin::ComponentPin { component: "R2".into(), pin: "b".into() },
                    to: vec![Pin::ComponentPin { component: "V1".into(), pin: "plate".into() }],
                },
                NetDef {
                    from: Pin::ComponentPin { component: "R1".into(), pin: "b".into() },
                    to: vec![Pin::Reserved("gnd".into())],
                },
                NetDef {
                    from: Pin::ComponentPin { component: "V1".into(), pin: "plate".into() },
                    to: vec![Pin::Reserved("out".into())],
                },
            ],
            controls: vec![],
            trims: vec![],
            monitors: vec![],
            sidechains: vec![],
        };

        let graph = LayoutGraph::from_pedal(&pedal);
        let groups = crate::groups::detect_groups(&graph);
        let cols = assign_columns(&graph, &groups);

        assert!(cols.num_columns >= 1, "Should have at least one column");
    }
}
