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
use pedalkernel::dsl::ComponentKind;


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
            (center_x, supply_y + supply_offset - MIN_COMPONENT_SPACING, 90)
        } else if graph.connects_to_gnd(member_id) {
            // Bias component below
            bias_offset += MIN_COMPONENT_SPACING;
            (center_x, bias_y + bias_offset - MIN_COMPONENT_SPACING, 90)
        } else {
            // Coupling component on signal path (offset left)
            (center_x - MIN_COMPONENT_SPACING, signal_y, 0)
        };

        let monitor_index = graph
            .monitor_ids
            .iter()
            .position(|id| *id == node.comp_id);

        placed.push(PlacedComponent {
            name: node.comp_id.clone(),
            kind: kind_to_string(&comp.kind),
            x,
            y,
            orientation,
            symbol: symbol_for_kind(&comp.kind),
            group: group.name.clone(),
            label: value_label(&comp.kind),
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
        } else if matches!(&comp.kind, ComponentKind::Diode(_) | ComponentKind::DiodePair(_)) {
            // Clipping diodes above signal path
            (center_x, signal_y - MIN_COMPONENT_SPACING, 0)
        } else {
            // Feedback components arranged around the op-amp
            offset_x += MIN_COMPONENT_SPACING;
            (center_x + offset_x, signal_y - MIN_COMPONENT_SPACING * 0.5, 0)
        };

        let monitor_index = graph
            .monitor_ids
            .iter()
            .position(|id| *id == node.comp_id);

        placed.push(PlacedComponent {
            name: node.comp_id.clone(),
            kind: kind_to_string(&comp.kind),
            x,
            y,
            orientation,
            symbol: symbol_for_kind(&comp.kind),
            group: group.name.clone(),
            label: value_label(&comp.kind),
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
    let spacing = (col_width * 0.7).min(count * MIN_COMPONENT_SPACING) / count.max(1.0);

    for (i, &member_id) in group.members.iter().enumerate() {
        let node = &graph.nodes[member_id];
        let comp = match &node.comp {
            Some(c) => c,
            None => continue,
        };

        let is_pot = matches!(comp.kind, ComponentKind::Potentiometer(_));
        let x = center_x + (i as f32 - count / 2.0) * spacing;
        let y = if is_pot {
            signal_y
        } else if graph.connects_to_gnd(member_id) {
            signal_y + MIN_COMPONENT_SPACING
        } else {
            signal_y - MIN_COMPONENT_SPACING * 0.5
        };

        let monitor_index = graph
            .monitor_ids
            .iter()
            .position(|id| *id == node.comp_id);

        placed.push(PlacedComponent {
            name: node.comp_id.clone(),
            kind: kind_to_string(&comp.kind),
            x,
            y,
            orientation: if is_pot { 0 } else { 90 },
            symbol: symbol_for_kind(&comp.kind),
            group: group.name.clone(),
            label: value_label(&comp.kind),
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

        let (x, y, orientation) = if matches!(comp.kind, ComponentKind::Pentode(_)) {
            pentode_count += 1;
            if pentode_count % 2 == 1 {
                (center_x, push_y, 0)
            } else {
                (center_x, pull_y, 0)
            }
        } else if matches!(comp.kind, ComponentKind::Transformer(_)) {
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

        let monitor_index = graph
            .monitor_ids
            .iter()
            .position(|id| *id == node.comp_id);

        placed.push(PlacedComponent {
            name: node.comp_id.clone(),
            kind: kind_to_string(&comp.kind),
            x,
            y,
            orientation,
            symbol: symbol_for_kind(&comp.kind),
            group: group.name.clone(),
            label: value_label(&comp.kind),
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
    let spacing = (col_width * 0.6).min(count * MIN_COMPONENT_SPACING) / count.max(1.0);

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

        let monitor_index = graph
            .monitor_ids
            .iter()
            .position(|id| *id == node.comp_id);

        placed.push(PlacedComponent {
            name: node.comp_id.clone(),
            kind: kind_to_string(&comp.kind),
            x,
            y,
            orientation: 0,
            symbol: symbol_for_kind(&comp.kind),
            group: group.name.clone(),
            label: value_label(&comp.kind),
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
    let max_x = components.iter().map(|c| c.x).fold(f32::NEG_INFINITY, f32::max);
    let min_y = components.iter().map(|c| c.y).fold(f32::INFINITY, f32::min);
    let max_y = components.iter().map(|c| c.y).fold(f32::NEG_INFINITY, f32::max);

    let padding = MIN_COMPONENT_SPACING * 0.5;
    GroupBounds {
        x: min_x - padding,
        y: min_y - padding,
        w: (max_x - min_x) + 2.0 * padding,
        h: (max_y - min_y) + 2.0 * padding,
    }
}

fn compute_signal_path_order(placed: &[PlacedComponent], _graph: &LayoutGraph) -> Vec<usize> {
    // Sort by x position (left to right = signal flow)
    let mut indices: Vec<usize> = (0..placed.len()).collect();
    indices.sort_by(|&a, &b| placed[a].x.partial_cmp(&placed[b].x).unwrap_or(std::cmp::Ordering::Equal));
    indices
}

fn kind_to_string(kind: &ComponentKind) -> String {
    match kind {
        ComponentKind::Resistor(_) => "resistor".into(),
        ComponentKind::Capacitor(_) => "capacitor".into(),
        ComponentKind::Inductor(_) => "inductor".into(),
        ComponentKind::DiodePair(_) => "diode_pair".into(),
        ComponentKind::Diode(_) => "diode".into(),
        ComponentKind::Zener(_) => "zener".into(),
        ComponentKind::Potentiometer(_) => "pot".into(),
        ComponentKind::Npn(_) => "npn".into(),
        ComponentKind::Pnp(_) => "pnp".into(),
        ComponentKind::OpAmp(_) => "opamp".into(),
        ComponentKind::NJfet(_) => "njfet".into(),
        ComponentKind::PJfet(_) => "pjfet".into(),
        ComponentKind::Triode(_) => "triode".into(),
        ComponentKind::Pentode(_) => "pentode".into(),
        ComponentKind::Nmos(_) => "nmos".into(),
        ComponentKind::Pmos(_) => "pmos".into(),
        ComponentKind::Transformer(_) => "transformer".into(),
        ComponentKind::Photocoupler(_) => "photocoupler".into(),
        ComponentKind::Lfo(..) => "lfo".into(),
        ComponentKind::EnvelopeFollower(..) => "envelope".into(),
        ComponentKind::Bbd(_) => "bbd".into(),
        ComponentKind::DelayLine(..) => "delay".into(),
        ComponentKind::Tap(..) => "tap".into(),
        ComponentKind::Neon(_) => "neon".into(),
        ComponentKind::Vco(_) => "vco".into(),
        ComponentKind::Vcf(_) => "vcf".into(),
        ComponentKind::Vca(_) => "vca".into(),
        ComponentKind::Comparator(_) => "comparator".into(),
        ComponentKind::AnalogSwitch(_) => "analog_switch".into(),
        ComponentKind::MatchedNpn(_) => "matched_npn".into(),
        ComponentKind::MatchedPnp(_) => "matched_pnp".into(),
        ComponentKind::Tempco(..) => "tempco".into(),
        ComponentKind::CapSwitched(_) => "cap_switched".into(),
        ComponentKind::InductorSwitched(_) => "inductor_switched".into(),
        ComponentKind::ResistorSwitched(_) => "resistor_switched".into(),
        ComponentKind::RotarySwitch(_) => "rotary_switch".into(),
        ComponentKind::Switch(_) => "switch".into(),
    }
}

fn value_label(kind: &ComponentKind) -> Option<String> {
    use pedalkernel::kicad::format_eng;
    match kind {
        ComponentKind::Resistor(v) => Some(format_eng(*v, "Ω")),
        ComponentKind::Capacitor(v) => Some(format_eng(v.value, "F")),
        ComponentKind::Inductor(v) => Some(format_eng(*v, "H")),
        ComponentKind::Potentiometer(v) => Some(format_eng(*v, "Ω")),
        ComponentKind::Zener(v) => Some(format!("{v}V")),
        _ => None,
    }
}
