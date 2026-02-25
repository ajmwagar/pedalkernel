//! Phase 6: Aesthetic optimization pass.
//!
//! After initial placement and routing, run optimization passes to improve
//! readability: grid snapping, overlap removal, crossing minimization, and
//! symmetry balancing.

use crate::types::*;

/// Grid size for snapping (in layout units).
const GRID_SIZE: f32 = 10.0;

/// Minimum clearance between component bounding boxes.
const MIN_CLEARANCE: f32 = 30.0;

/// Component bounding box size estimate (half-width/height).
const COMPONENT_HALF_SIZE: f32 = 15.0;

/// Run all optimization passes on the layout.
pub fn optimize_layout(layout: &mut Layout) {
    snap_to_grid(layout);
    resolve_overlaps(layout);
    minimize_crossings(layout);
    center_groups(layout);
}

/// Snap all component positions to the nearest grid point.
fn snap_to_grid(layout: &mut Layout) {
    for comp in &mut layout.components {
        comp.x = (comp.x / GRID_SIZE).round() * GRID_SIZE;
        comp.y = (comp.y / GRID_SIZE).round() * GRID_SIZE;
    }
}

/// Push apart overlapping components.
fn resolve_overlaps(layout: &mut Layout) {
    // Simple iterative repulsion: for each pair of components, if they
    // overlap, push them apart along the axis of least overlap.
    for _ in 0..10 {
        let mut any_overlap = false;

        for i in 0..layout.components.len() {
            for j in (i + 1)..layout.components.len() {
                let dx = layout.components[j].x - layout.components[i].x;
                let dy = layout.components[j].y - layout.components[i].y;

                let overlap_x = MIN_CLEARANCE - dx.abs();
                let overlap_y = MIN_CLEARANCE - dy.abs();

                if overlap_x > 0.0 && overlap_y > 0.0 {
                    any_overlap = true;

                    if overlap_x < overlap_y {
                        // Push apart horizontally
                        let push = overlap_x / 2.0 + 1.0;
                        if dx >= 0.0 {
                            layout.components[i].x -= push;
                            layout.components[j].x += push;
                        } else {
                            layout.components[i].x += push;
                            layout.components[j].x -= push;
                        }
                    } else {
                        // Push apart vertically
                        let push = overlap_y / 2.0 + 1.0;
                        if dy >= 0.0 {
                            layout.components[i].y -= push;
                            layout.components[j].y += push;
                        } else {
                            layout.components[i].y += push;
                            layout.components[j].y -= push;
                        }
                    }
                }
            }
        }

        if !any_overlap {
            break;
        }
    }

    // Re-snap after overlap resolution
    snap_to_grid(layout);
}

/// Minimize wire crossings using barycenter heuristic.
///
/// For each column, reorder components to minimize crossings with
/// adjacent columns. This is a simplified version of the Sugiyama
/// crossing minimization step.
fn minimize_crossings(layout: &mut Layout) {
    // Group components by their group name
    let mut group_members: std::collections::HashMap<String, Vec<usize>> =
        std::collections::HashMap::new();
    for (i, comp) in layout.components.iter().enumerate() {
        group_members
            .entry(comp.group.clone())
            .or_default()
            .push(i);
    }

    // For each group, sort members by their connected wire positions
    // (barycenter of connected component positions)
    for (_group_name, members) in &group_members {
        if members.len() <= 1 {
            continue;
        }

        // Compute barycenter for each member based on connected wires
        let mut barycenters: Vec<(usize, f32)> = members
            .iter()
            .map(|&idx| {
                let comp_name = &layout.components[idx].name;
                let connected_y: Vec<f32> = layout
                    .wires
                    .iter()
                    .filter(|w| {
                        w.points.first().map(|p| {
                            (p[0] - layout.components[idx].x).abs() < COMPONENT_HALF_SIZE
                                && (p[1] - layout.components[idx].y).abs() < COMPONENT_HALF_SIZE
                        }).unwrap_or(false)
                            || w.points.last().map(|p| {
                                (p[0] - layout.components[idx].x).abs() < COMPONENT_HALF_SIZE
                                    && (p[1] - layout.components[idx].y).abs() < COMPONENT_HALF_SIZE
                            }).unwrap_or(false)
                    })
                    .flat_map(|w| w.points.iter().map(|p| p[1]))
                    .collect();

                let bc = if connected_y.is_empty() {
                    layout.components[idx].y
                } else {
                    connected_y.iter().sum::<f32>() / connected_y.len() as f32
                };

                let _ = comp_name; // used for filtering above
                (idx, bc)
            })
            .collect();

        barycenters.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        // Reorder y-positions to match barycenter order
        let mut y_positions: Vec<f32> = barycenters.iter().map(|&(idx, _)| layout.components[idx].y).collect();
        y_positions.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        for (rank, &(idx, _)) in barycenters.iter().enumerate() {
            if rank < y_positions.len() {
                layout.components[idx].y = y_positions[rank];
            }
        }
    }
}

/// Center each group's components within the group bounding box.
fn center_groups(layout: &mut Layout) {
    // Re-compute group bounds from current component positions
    for group in &mut layout.groups {
        let members: Vec<usize> = layout
            .components
            .iter()
            .enumerate()
            .filter(|(_, c)| c.group == group.name)
            .map(|(i, _)| i)
            .collect();

        if members.is_empty() {
            continue;
        }

        let min_x = members.iter().map(|&i| layout.components[i].x).fold(f32::INFINITY, f32::min);
        let max_x = members.iter().map(|&i| layout.components[i].x).fold(f32::NEG_INFINITY, f32::max);
        let min_y = members.iter().map(|&i| layout.components[i].y).fold(f32::INFINITY, f32::min);
        let max_y = members.iter().map(|&i| layout.components[i].y).fold(f32::NEG_INFINITY, f32::max);

        let padding = COMPONENT_HALF_SIZE;
        group.bounds = GroupBounds {
            x: min_x - padding,
            y: min_y - padding,
            w: (max_x - min_x) + 2.0 * padding,
            h: (max_y - min_y) + 2.0 * padding,
        };
    }
}

/// Compute a layout quality score (higher is better).
///
/// Used for hill-climbing optimization and testing.
pub fn layout_score(layout: &Layout) -> f32 {
    let crossing_penalty = count_wire_crossings(layout) as f32 * -10.0;
    let alignment_bonus = count_grid_aligned(layout) as f32 * 1.0;
    let overlap_penalty = count_overlaps(layout) as f32 * -100.0;
    let symmetry_bonus = measure_vertical_symmetry(layout) * 5.0;
    let wire_length_penalty = total_wire_length(layout) * -0.01;
    let signal_flow_bonus = if signal_flows_left_to_right(layout) { 50.0 } else { 0.0 };

    crossing_penalty
        + alignment_bonus
        + overlap_penalty
        + symmetry_bonus
        + wire_length_penalty
        + signal_flow_bonus
}

fn count_wire_crossings(layout: &Layout) -> usize {
    let mut crossings = 0;
    for i in 0..layout.wires.len() {
        for j in (i + 1)..layout.wires.len() {
            // Check each segment pair for crossings
            let wire_a = &layout.wires[i];
            let wire_b = &layout.wires[j];
            for seg_a in wire_a.points.windows(2) {
                for seg_b in wire_b.points.windows(2) {
                    if segments_cross(seg_a[0], seg_a[1], seg_b[0], seg_b[1]) {
                        crossings += 1;
                    }
                }
            }
        }
    }
    crossings
}

fn segments_cross(a1: [f32; 2], a2: [f32; 2], b1: [f32; 2], b2: [f32; 2]) -> bool {
    // Simple check for orthogonal segment crossing:
    // One segment is horizontal and one is vertical
    let a_horiz = (a1[1] - a2[1]).abs() < 0.5;
    let b_horiz = (b1[1] - b2[1]).abs() < 0.5;

    if a_horiz == b_horiz {
        return false; // Parallel segments don't cross (for Manhattan routing)
    }

    let (h, v) = if a_horiz { ((a1, a2), (b1, b2)) } else { ((b1, b2), (a1, a2)) };

    let (h_min_x, h_max_x) = minmax(h.0[0], h.1[0]);
    let h_y = h.0[1];
    let (v_min_y, v_max_y) = minmax(v.0[1], v.1[1]);
    let v_x = v.0[0];

    v_x > h_min_x && v_x < h_max_x && h_y > v_min_y && h_y < v_max_y
}

fn minmax(a: f32, b: f32) -> (f32, f32) {
    if a <= b { (a, b) } else { (b, a) }
}

fn count_grid_aligned(layout: &Layout) -> usize {
    layout
        .components
        .iter()
        .filter(|c| {
            (c.x % GRID_SIZE).abs() < 0.5 && (c.y % GRID_SIZE).abs() < 0.5
        })
        .count()
}

fn count_overlaps(layout: &Layout) -> usize {
    let mut overlaps = 0;
    for i in 0..layout.components.len() {
        for j in (i + 1)..layout.components.len() {
            let dx = (layout.components[i].x - layout.components[j].x).abs();
            let dy = (layout.components[i].y - layout.components[j].y).abs();
            if dx < MIN_CLEARANCE && dy < MIN_CLEARANCE {
                overlaps += 1;
            }
        }
    }
    overlaps
}

fn measure_vertical_symmetry(layout: &Layout) -> f32 {
    if layout.components.is_empty() {
        return 0.0;
    }

    let center_y = layout.bounds.height / 2.0;
    let deviations: f32 = layout
        .components
        .iter()
        .map(|c| (c.y - center_y).abs())
        .sum();
    let max_deviation = layout.components.len() as f32 * center_y;

    if max_deviation > 0.0 {
        1.0 - (deviations / max_deviation)
    } else {
        1.0
    }
}

fn total_wire_length(layout: &Layout) -> f32 {
    layout
        .wires
        .iter()
        .flat_map(|w| {
            w.points.windows(2).map(|seg| {
                let dx = seg[1][0] - seg[0][0];
                let dy = seg[1][1] - seg[0][1];
                (dx * dx + dy * dy).sqrt()
            })
        })
        .sum()
}

fn signal_flows_left_to_right(layout: &Layout) -> bool {
    // Check that signal-path wires generally go left to right
    let signal_wires: Vec<_> = layout.wires.iter().filter(|w| w.signal_path).collect();
    if signal_wires.is_empty() {
        return true;
    }

    let left_to_right_count = signal_wires
        .iter()
        .filter(|w| {
            if let (Some(first), Some(last)) = (w.points.first(), w.points.last()) {
                first[0] <= last[0]
            } else {
                true
            }
        })
        .count();

    left_to_right_count * 2 >= signal_wires.len()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn segments_cross_perpendicular() {
        // Horizontal (0,5)-(10,5) crossing vertical (5,0)-(5,10)
        assert!(segments_cross([0.0, 5.0], [10.0, 5.0], [5.0, 0.0], [5.0, 10.0]));
    }

    #[test]
    fn segments_no_cross_parallel() {
        // Two horizontal segments
        assert!(!segments_cross([0.0, 5.0], [10.0, 5.0], [0.0, 7.0], [10.0, 7.0]));
    }

    #[test]
    fn segments_no_cross_miss() {
        // Horizontal and vertical that don't intersect
        assert!(!segments_cross([0.0, 5.0], [3.0, 5.0], [5.0, 0.0], [5.0, 10.0]));
    }
}
