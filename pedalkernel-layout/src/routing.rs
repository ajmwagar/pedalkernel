//! Phase 5: Wire routing (orthogonal Manhattan paths).
//!
//! Routes wires between component pins using orthogonal (Manhattan) paths.
//! Signal path wires run horizontally on the center line. Supply wires run
//! vertically to the rails. Feedback wires route along the bottom.

use crate::graph::LayoutGraph;
use crate::types::*;
use std::collections::HashMap;

/// Route all wires in the layout based on the graph connectivity.
pub fn route_wires(layout: &mut Layout, graph: &LayoutGraph) {
    let mut wires = Vec::new();

    // Build a map from component name to placed position
    let comp_positions: HashMap<&str, (f32, f32)> = layout
        .components
        .iter()
        .map(|c| (c.name.as_str(), (c.x, c.y)))
        .collect();

    // Build a map from component name to index in layout.components
    let comp_indices: HashMap<&str, usize> = layout
        .components
        .iter()
        .enumerate()
        .map(|(i, c)| (c.name.as_str(), i))
        .collect();

    // For each net group, route wires between the components
    for ng in &graph.net_groups {
        let net_name = ng
            .name
            .clone()
            .unwrap_or_else(|| format!("net_{}", ng.index));

        let is_supply_net = ng.name.as_deref() == Some("vcc") || ng.name.as_deref() == Some("gnd");

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
                    layout.supply_rails.iter().find(|r| r.name == "vcc").map(|r| r.y).unwrap_or(0.0)
                } else {
                    layout.supply_rails.iter().find(|r| r.name == "gnd").map(|r| r.y).unwrap_or(layout.bounds.height)
                };

                wires.push(Wire {
                    net: net_name,
                    points: vec![[*x, *y], [*x, rail_y]],
                    signal_path: false,
                    monitor_index_start: comp_indices.get(name.as_str()).and_then(|&i| layout.components[i].monitor_index),
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

            let points = route_wire(
                Point::new(*x_a, *y_a),
                Point::new(*x_b, *y_b),
            );

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

/// Route a wire with a Z-route (horizontal, vertical jog, horizontal) to
/// avoid obstacles. Used when an L-route would cross a component.
#[allow(dead_code)]
fn route_wire_z(start: Point, end: Point, jog_x: f32) -> Vec<Point> {
    vec![
        start,
        Point::new(jog_x, start.y),
        Point::new(jog_x, end.y),
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
