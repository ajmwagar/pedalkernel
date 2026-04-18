//! Phase 4: Vertical placement within columns.
//!
//! Places components within each column following electrical convention:
//! - VCC rail at top (5% from top)
//! - Supply-connected components (plate loads) extend upward
//! - Signal path on the horizontal center line (50%)
//! - Ground-connected components (cathode bias) extend downward
//! - GND rail at bottom (5% from bottom)

use crate::graph::LayoutGraph;
use crate::groups::{FunctionalGroup, GroupKind};
use crate::layering::ColumnAssignment;
use crate::symbols::symbol_for_kind;
use crate::types::*;
use pedalkernel::compiler::Component;

/// Vertical position constants (as fraction of total height).
const VCC_RAIL_Y: f32 = 0.05;
const SUPPLY_COMPONENT_Y: f32 = 0.20;
const SIGNAL_PATH_Y: f32 = 0.50;
const BIAS_COMPONENT_Y: f32 = 0.75;
const GND_RAIL_Y: f32 = 0.95;

/// For push-pull / phase inverter: split center line.
const PUSH_Y: f32 = 0.35;
const PULL_Y: f32 = 0.65;

/// Minimum spacing between components (in layout units).
const MIN_COMPONENT_SPACING: f32 = 40.0;

/// Place all components and produce the initial layout.
pub fn place_components(
    graph: &LayoutGraph,
    groups: &[FunctionalGroup],
    columns: &ColumnAssignment,
    width: f32,
    height: f32,
) -> Layout {
    let mut placed = Vec::new();
    let mut layout_groups = Vec::new();

    // Compute column x-positions from widths
    let col_positions = compute_column_positions(&columns.column_widths, width);

    // Place each group
    for group in groups {
        let col = columns.group_columns.get(&group.name).copied().unwrap_or(0);
        let (col_x, col_w) = col_positions[col];
        let col_center_x = col_x + col_w / 2.0;

        let group_components = place_group(graph, group, col_center_x, col_w, height);
        let group_bounds = compute_group_bounds(&group_components, col_x, col_w, height);

        layout_groups.push(Group {
            name: group.name.clone(),
            label: group.label.clone(),
            bounds: group_bounds,
        });

        placed.extend(group_components);
    }

    // Build signal path order
    let signal_path_order = compute_signal_path_order(&placed, graph);

    Layout {
        version: 1,
        bounds: Bounds { width, height },
        components: placed,
        wires: Vec::new(), // Filled in Phase 5
        groups: layout_groups,
        supply_rails: vec![
            SupplyRail {
                name: "vcc".into(),
                y: VCC_RAIL_Y * height,
            },
            SupplyRail {
                name: "gnd".into(),
                y: GND_RAIL_Y * height,
            },
        ],
        signal_path_order,
    }
}

// ---------------------------------------------------------------------------
// Column position computation
// ---------------------------------------------------------------------------

/// Returns (x_start, width) for each column.
fn compute_column_positions(widths: &[f32], total_width: f32) -> Vec<(f32, f32)> {
    let margin = total_width * 0.05; // 5% margin on each side
    let usable = total_width - 2.0 * margin;
    let mut positions = Vec::with_capacity(widths.len());
    let mut x = margin;

    for &w in widths {
        let col_w = usable * w;
        positions.push((x, col_w));
        x += col_w;
    }

    positions
}

// ---------------------------------------------------------------------------
// Group placement
// ---------------------------------------------------------------------------

fn place_group(
    graph: &LayoutGraph,
    group: &FunctionalGroup,
    center_x: f32,
    col_width: f32,
    height: f32,
) -> Vec<PlacedComponent> {
    match group.kind {
        GroupKind::GainStage => place_gain_stage(graph, group, center_x, col_width, height),
        GroupKind::OpAmpStage => place_opamp_stage(graph, group, center_x, col_width, height),
        GroupKind::ToneStack => place_tone_stack(graph, group, center_x, col_width, height),
        GroupKind::PushPullOutput => place_push_pull(graph, group, center_x, col_width, height),
        GroupKind::PhaseInverter => place_phase_inverter(graph, group, center_x, col_width, height),
        _ => place_generic(graph, group, center_x, col_width, height),
    }
}

fn place_gain_stage(
    graph: &LayoutGraph,
    group: &FunctionalGroup,
    center_x: f32,
    _col_width: f32,
    height: f32,
) -> Vec<PlacedComponent> {
    let mut placed = Vec::new();
    let signal_y = SIGNAL_PATH_Y * height;
    let supply_y = SUPPLY_COMPONENT_Y * height;
    let bias_y = BIAS_COMPONENT_Y * height;

    let mut supply_offset = 0.0f32;
    let mut bias_offset = 0.0f32;

    for &member_id in &group.members {
        let node = &graph.nodes[member_id];
        let comp = match &node.comp {
            Some(c) => c,
            None => continue,
        };

        let (x, y, orientation) = if Some(member_id) == group.primary_device {
            // Active device at center of signal path
            (center_x, signal_y, 0)
        } else if graph.connects_to_vcc(member_id) {
            // Supply component above
            supply_offset += MIN_COMPONENT_SPACING;
            (
                center_x,
                supply_y + supply_offset - MIN_COMPONENT_SPACING,
                90,
            )
        } else if graph.connects_to_gnd(member_id) {
            // Bias component below
            bias_offset += MIN_COMPONENT_SPACING;
            (center_x, bias_y + bias_offset - MIN_COMPONENT_SPACING, 90)
        } else {
            // Coupling component on signal path (offset left)
            (center_x - MIN_COMPONENT_SPACING, signal_y, 0)
        };

        let monitor_index = graph.monitor_ids.iter().position(|id| *id == node.comp_id);

        placed.push(PlacedComponent {
            name: node.comp_id.clone(),
            kind: kind_to_string(comp.kind.as_ref()),
            x,
            y,
            orientation,
            symbol: symbol_for_kind(comp.kind.as_ref()),
            group: group.name.clone(),
            label: value_label(comp.kind.as_ref()),
            monitor_index,
        });
    }

    placed
}

fn place_opamp_stage(
    graph: &LayoutGraph,
    group: &FunctionalGroup,
    center_x: f32,
    col_width: f32,
    height: f32,
) -> Vec<PlacedComponent> {
    let mut placed = Vec::new();
    let signal_y = SIGNAL_PATH_Y * height;

    let mut offset_x = -col_width * 0.3;

    for &member_id in &group.members {
        let node = &graph.nodes[member_id];
        let comp = match &node.comp {
            Some(c) => c,
            None => continue,
        };

        let (x, y, orientation) = if Some(member_id) == group.primary_device {
            // Op-amp at center
            (center_x, signal_y, 0)
        } else if comp.kind.is_diode_family() {
            // Clipping diodes above signal path
            (center_x, signal_y - MIN_COMPONENT_SPACING, 0)
        } else {
            // Feedback components arranged around the op-amp
            offset_x += MIN_COMPONENT_SPACING;
            (
                center_x + offset_x,
                signal_y - MIN_COMPONENT_SPACING * 0.5,
                0,
            )
        };

        let monitor_index = graph.monitor_ids.iter().position(|id| *id == node.comp_id);

        placed.push(PlacedComponent {
            name: node.comp_id.clone(),
            kind: kind_to_string(comp.kind.as_ref()),
            x,
            y,
            orientation,
            symbol: symbol_for_kind(comp.kind.as_ref()),
            group: group.name.clone(),
            label: value_label(comp.kind.as_ref()),
            monitor_index,
        });
    }

    placed
}

fn place_tone_stack(
    graph: &LayoutGraph,
    group: &FunctionalGroup,
    center_x: f32,
    col_width: f32,
    height: f32,
) -> Vec<PlacedComponent> {
    let mut placed = Vec::new();
    let signal_y = SIGNAL_PATH_Y * height;

    let count = group.members.len() as f32;
    let spacing = ((col_width * 0.7) / count.max(1.0)).max(MIN_COMPONENT_SPACING);

    for (i, &member_id) in group.members.iter().enumerate() {
        let node = &graph.nodes[member_id];
        let comp = match &node.comp {
            Some(c) => c,
            None => continue,
        };

        let is_pot = comp.kind.is_pot();
        let x = center_x + (i as f32 - count / 2.0) * spacing;
        let y = if is_pot {
            signal_y
        } else if graph.connects_to_gnd(member_id) {
            signal_y + MIN_COMPONENT_SPACING
        } else {
            signal_y - MIN_COMPONENT_SPACING * 0.5
        };

        let monitor_index = graph.monitor_ids.iter().position(|id| *id == node.comp_id);

        placed.push(PlacedComponent {
            name: node.comp_id.clone(),
            kind: kind_to_string(comp.kind.as_ref()),
            x,
            y,
            orientation: if is_pot { 0 } else { 90 },
            symbol: symbol_for_kind(comp.kind.as_ref()),
            group: group.name.clone(),
            label: value_label(comp.kind.as_ref()),
            monitor_index,
        });
    }

    placed
}

fn place_push_pull(
    graph: &LayoutGraph,
    group: &FunctionalGroup,
    center_x: f32,
    _col_width: f32,
    height: f32,
) -> Vec<PlacedComponent> {
    let mut placed = Vec::new();
    let push_y = PUSH_Y * height;
    let pull_y = PULL_Y * height;
    let signal_y = SIGNAL_PATH_Y * height;

    let mut pentode_count = 0;

    for &member_id in &group.members {
        let node = &graph.nodes[member_id];
        let comp = match &node.comp {
            Some(c) => c,
            None => continue,
        };

        let (x, y, orientation) = if comp.kind.layout_class() == "pentode" {
            pentode_count += 1;
            if pentode_count % 2 == 1 {
                (center_x, push_y, 0)
            } else {
                (center_x, pull_y, 0)
            }
        } else if comp.kind.is_transformer() {
            (center_x + MIN_COMPONENT_SPACING * 2.0, signal_y, 0)
        } else {
            // Bias components
            let y = if graph.connects_to_vcc(member_id) {
                SUPPLY_COMPONENT_Y * height
            } else {
                BIAS_COMPONENT_Y * height
            };
            (center_x, y, 90)
        };

        let monitor_index = graph.monitor_ids.iter().position(|id| *id == node.comp_id);

        placed.push(PlacedComponent {
            name: node.comp_id.clone(),
            kind: kind_to_string(comp.kind.as_ref()),
            x,
            y,
            orientation,
            symbol: symbol_for_kind(comp.kind.as_ref()),
            group: group.name.clone(),
            label: value_label(comp.kind.as_ref()),
            monitor_index,
        });
    }

    placed
}

fn place_phase_inverter(
    graph: &LayoutGraph,
    group: &FunctionalGroup,
    center_x: f32,
    _col_width: f32,
    height: f32,
) -> Vec<PlacedComponent> {
    // Phase inverter is similar to push-pull but with triodes
    place_push_pull(graph, group, center_x, _col_width, height)
}

fn place_generic(
    graph: &LayoutGraph,
    group: &FunctionalGroup,
    center_x: f32,
    col_width: f32,
    height: f32,
) -> Vec<PlacedComponent> {
    let mut placed = Vec::new();
    let signal_y = SIGNAL_PATH_Y * height;
    let count = group.members.len() as f32;
    let spacing = ((col_width * 0.6) / count.max(1.0)).max(MIN_COMPONENT_SPACING);

    for (i, &member_id) in group.members.iter().enumerate() {
        let node = &graph.nodes[member_id];
        let comp = match &node.comp {
            Some(c) => c,
            None => continue,
        };

        let x = center_x + (i as f32 - count / 2.0) * spacing;
        let y = if graph.connects_to_vcc(member_id) {
            SUPPLY_COMPONENT_Y * height
        } else if graph.connects_to_gnd(member_id) {
            BIAS_COMPONENT_Y * height
        } else {
            signal_y
        };

        let monitor_index = graph.monitor_ids.iter().position(|id| *id == node.comp_id);

        placed.push(PlacedComponent {
            name: node.comp_id.clone(),
            kind: kind_to_string(comp.kind.as_ref()),
            x,
            y,
            orientation: 0,
            symbol: symbol_for_kind(comp.kind.as_ref()),
            group: group.name.clone(),
            label: value_label(comp.kind.as_ref()),
            monitor_index,
        });
    }

    placed
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn compute_group_bounds(
    components: &[PlacedComponent],
    col_x: f32,
    col_w: f32,
    height: f32,
) -> GroupBounds {
    if components.is_empty() {
        return GroupBounds {
            x: col_x,
            y: 0.0,
            w: col_w,
            h: height,
        };
    }

    let min_x = components.iter().map(|c| c.x).fold(f32::INFINITY, f32::min);
    let max_x = components
        .iter()
        .map(|c| c.x)
        .fold(f32::NEG_INFINITY, f32::max);
    let min_y = components.iter().map(|c| c.y).fold(f32::INFINITY, f32::min);
    let max_y = components
        .iter()
        .map(|c| c.y)
        .fold(f32::NEG_INFINITY, f32::max);

    let padding = MIN_COMPONENT_SPACING * 0.5;
    GroupBounds {
        x: min_x - padding,
        y: min_y - padding,
        w: (max_x - min_x) + 2.0 * padding,
        h: (max_y - min_y) + 2.0 * padding,
    }
}

fn compute_signal_path_order(placed: &[PlacedComponent], graph: &LayoutGraph) -> Vec<usize> {
    // BFS from graph.in_node following non-feedback, non-supply edges
    // to determine signal-flow ordering.
    let mut visited = std::collections::BTreeSet::new();
    let mut queue = std::collections::VecDeque::new();
    let mut bfs_order: Vec<String> = Vec::new();

    visited.insert(graph.in_node);
    queue.push_back(graph.in_node);

    while let Some(node_id) = queue.pop_front() {
        let comp_id = &graph.nodes[node_id].comp_id;
        if !graph.nodes[node_id].is_anchor {
            bfs_order.push(comp_id.clone());
        }

        // Follow outgoing non-feedback, non-supply edges
        for edge in &graph.edges {
            let neighbor = if edge.from == node_id {
                edge.to
            } else if edge.to == node_id {
                edge.from
            } else {
                continue;
            };

            if edge.is_feedback || edge.is_supply {
                continue;
            }
            if visited.contains(&neighbor) {
                continue;
            }
            visited.insert(neighbor);
            queue.push_back(neighbor);
        }
    }

    // Map BFS comp_id order → placed component indices
    let name_to_placed: std::collections::BTreeMap<&str, usize> = placed
        .iter()
        .enumerate()
        .map(|(i, c)| (c.name.as_str(), i))
        .collect();

    let mut ordered: Vec<usize> = Vec::new();
    let mut used = std::collections::BTreeSet::new();

    for comp_id in &bfs_order {
        if let Some(&idx) = name_to_placed.get(comp_id.as_str()) {
            if used.insert(idx) {
                ordered.push(idx);
            }
        }
    }

    // Append any unreached components at the end (sorted by x as fallback)
    let mut remaining: Vec<usize> = (0..placed.len()).filter(|i| !used.contains(i)).collect();
    remaining.sort_by(|&a, &b| {
        placed[a]
            .x
            .partial_cmp(&placed[b].x)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    ordered.extend(remaining);

    ordered
}

fn kind_to_string(kind: &dyn Component) -> String {
    kind.layout_class().to_string()
}

fn value_label(kind: &dyn Component) -> Option<String> {
    kind.display_value()
}
