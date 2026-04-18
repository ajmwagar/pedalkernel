//! Phase 5: Wire routing (orthogonal Manhattan paths).
//!
//! Routes wires between component pins using orthogonal (Manhattan) paths.
//! Signal path wires run horizontally on the center line. Supply wires run
//! vertically to the rails. Feedback wires route along the bottom.

use crate::graph::LayoutGraph;
use crate::types::*;
use std::collections::{BTreeMap, BTreeSet};

/// Radius for obstacle avoidance — if an L-route corner is within this
/// distance of a component center, switch to a Z-route.
const OBSTACLE_RADIUS: f32 = 25.0;

/// Route all wires in the layout based on the graph connectivity.
pub fn route_wires(layout: &mut Layout, graph: &LayoutGraph) {
    let mut wires = Vec::new();

    // Build a map from component name to placed position
    let comp_positions: BTreeMap<&str, (f32, f32)> = layout
        .components
        .iter()
        .map(|c| (c.name.as_str(), (c.x, c.y)))
        .collect();

    // Build a map from component name to index in layout.components
    let comp_indices: BTreeMap<&str, usize> = layout
        .components
        .iter()
        .enumerate()
        .map(|(i, c)| (c.name.as_str(), i))
        .collect();

    // Collect all component centers as obstacles for wire routing
    let obstacles: Vec<(f32, f32)> = layout.components.iter().map(|c| (c.x, c.y)).collect();

    // Collect feedback component pairs for feedback wire detection
    let feedback_comps: BTreeSet<(usize, usize)> = graph
        .edges
        .iter()
        .filter(|e| e.is_feedback)
        .map(|e| (e.from, e.to))
        .collect();

    // GND rail Y for feedback wire routing below signal path
    let gnd_rail_y = layout
        .supply_rails
        .iter()
        .find(|r| r.name == "gnd")
        .map(|r| r.y)
        .unwrap_or(layout.bounds.height);
    let feedback_y = gnd_rail_y - 20.0;

    // For each net group, route wires between the components
    for ng in &graph.net_groups {
        let net_name = ng
            .name
            .clone()
            .unwrap_or_else(|| format!("net_{}", ng.index));

        let is_supply_net = ng.name.as_deref() == Some("vcc") || ng.name.as_deref() == Some("gnd");

        // Check if this net contains a feedback edge
        let is_feedback_net = ng.component_ids.iter().any(|cid_a| {
            let node_a = graph.id_to_node.get(cid_a).copied();
            ng.component_ids.iter().any(|cid_b| {
                let node_b = graph.id_to_node.get(cid_b).copied();
                if let (Some(a), Some(b)) = (node_a, node_b) {
                    feedback_comps.contains(&(a, b)) || feedback_comps.contains(&(b, a))
                } else {
                    false
                }
            })
        });

        // Collect positioned components in this net
        let mut positions: Vec<(String, f32, f32)> = Vec::new();
        for cid in &ng.component_ids {
            // Map anchor IDs
            let lookup_name = match cid.as_str() {
                "__in" | "__out" | "__vcc" | "__gnd" => continue, // Skip anchors
                name => name,
            };
            if let Some(&(x, y)) = comp_positions.get(lookup_name) {
                positions.push((lookup_name.to_string(), x, y));
            }
        }

        if positions.len() < 2 {
            // Handle supply connections as stub wires
            if is_supply_net && positions.len() == 1 {
                let (name, x, y) = &positions[0];
                let rail_y = if ng.name.as_deref() == Some("vcc") {
                    layout
                        .supply_rails
                        .iter()
                        .find(|r| r.name == "vcc")
                        .map(|r| r.y)
                        .unwrap_or(0.0)
                } else {
                    layout
                        .supply_rails
                        .iter()
                        .find(|r| r.name == "gnd")
                        .map(|r| r.y)
                        .unwrap_or(layout.bounds.height)
                };

                wires.push(Wire {
                    net: net_name,
                    points: vec![[*x, *y], [*x, rail_y]],
                    signal_path: false,
                    monitor_index_start: comp_indices
                        .get(name.as_str())
                        .and_then(|&i| layout.components[i].monitor_index),
                    monitor_index_end: None,
                });
            }
            continue;
        }

        // Sort positions by x (left to right for signal flow)
        positions.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        // Route wires between consecutive components in the net
        for window in positions.windows(2) {
            let (name_a, x_a, y_a) = &window[0];
            let (name_b, x_b, y_b) = &window[1];

            let start = Point::new(*x_a, *y_a);
            let end = Point::new(*x_b, *y_b);

            let points = if is_feedback_net {
                // Feedback wires route below signal path
                route_wire_feedback(start, end, feedback_y)
            } else {
                route_wire_with_obstacles(start, end, &obstacles)
            };

            let is_signal = !is_supply_net
                && ng.name.as_deref() != Some("vcc")
                && ng.name.as_deref() != Some("gnd");

            let monitor_start = comp_indices
                .get(name_a.as_str())
                .and_then(|&i| layout.components[i].monitor_index);
            let monitor_end = comp_indices
                .get(name_b.as_str())
                .and_then(|&i| layout.components[i].monitor_index);

            wires.push(Wire {
                net: net_name.clone(),
                points: points.into_iter().map(|p| [p.x, p.y]).collect(),
                signal_path: is_signal,
                monitor_index_start: monitor_start,
                monitor_index_end: monitor_end,
            });
        }
    }

    layout.wires = wires;
}

/// Route a single wire between two points using orthogonal (Manhattan) routing.
///
/// - If same Y: straight horizontal line
/// - If same X: straight vertical line
/// - Otherwise: L-route (horizontal then vertical)
fn route_wire(start: Point, end: Point) -> Vec<Point> {
    const EPSILON: f32 = 0.5;

    if (start.y - end.y).abs() < EPSILON {
        // Same horizontal level: straight line
        vec![start, end]
    } else if (start.x - end.x).abs() < EPSILON {
        // Same vertical level: straight line
        vec![start, end]
    } else {
        // L-route: horizontal first, then vertical
        let mid = Point::new(end.x, start.y);
        vec![start, mid, end]
    }
}

/// Route a wire with obstacle avoidance. Uses an L-route if the corner is
/// clear of components, otherwise falls back to a Z-route through the midpoint.
fn route_wire_with_obstacles(start: Point, end: Point, obstacles: &[(f32, f32)]) -> Vec<Point> {
    let basic = route_wire(start, end);

    // Only L-routes (3 points) need obstacle checking
    if basic.len() != 3 {
        return basic;
    }

    let corner = &basic[1];
    let corner_blocked = obstacles.iter().any(|&(ox, oy)| {
        // Don't count start/end positions as obstacles
        let is_endpoint = ((ox - start.x).abs() < 1.0 && (oy - start.y).abs() < 1.0)
            || ((ox - end.x).abs() < 1.0 && (oy - end.y).abs() < 1.0);
        if is_endpoint {
            return false;
        }
        let dx = corner.x - ox;
        let dy = corner.y - oy;
        (dx * dx + dy * dy).sqrt() < OBSTACLE_RADIUS
    });

    if corner_blocked {
        // Z-route through the midpoint X
        let mid_x = (start.x + end.x) / 2.0;
        route_wire_z(start, end, mid_x)
    } else {
        basic
    }
}

/// Route a wire with a Z-route (horizontal, vertical jog, horizontal) to
/// avoid obstacles. Used when an L-route would cross a component.
fn route_wire_z(start: Point, end: Point, jog_x: f32) -> Vec<Point> {
    vec![
        start,
        Point::new(jog_x, start.y),
        Point::new(jog_x, end.y),
        end,
    ]
}

/// Route a feedback wire below the signal path.
fn route_wire_feedback(start: Point, end: Point, bottom_y: f32) -> Vec<Point> {
    vec![
        start,
        Point::new(start.x, bottom_y),
        Point::new(end.x, bottom_y),
        end,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_horizontal() {
        let pts = route_wire(Point::new(0.0, 100.0), Point::new(200.0, 100.0));
        assert_eq!(pts.len(), 2);
    }

    #[test]
    fn straight_vertical() {
        let pts = route_wire(Point::new(100.0, 0.0), Point::new(100.0, 200.0));
        assert_eq!(pts.len(), 2);
    }

    #[test]
    fn l_route() {
        let pts = route_wire(Point::new(0.0, 0.0), Point::new(200.0, 100.0));
        assert_eq!(pts.len(), 3);
        assert_eq!(pts[1].x, 200.0);
        assert_eq!(pts[1].y, 0.0);
    }
}
