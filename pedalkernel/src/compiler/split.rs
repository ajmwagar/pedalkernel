//! FX loop split: partition a PedalDef at fx_send/fx_return boundaries.

use std::collections::HashMap;

use crate::dsl::*;
use crate::PedalProcessor;

use super::compile::compile_pedal;
use super::compiled::CompiledPedal;

/// Extract the primary node name from a Pin for graph connectivity purposes.
fn pin_node_name(pin: &Pin) -> String {
    match pin {
        Pin::Reserved(n) => n.clone(),
        Pin::ComponentPin { component, .. } => component.clone(),
        // For Fork, we use the switch component as the node
        Pin::Fork { switch, .. } => switch.clone(),
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// FX loop split — partition a PedalDef at fx_send/fx_return
// ═══════════════════════════════════════════════════════════════════════════

/// Split a pedal definition at its `fx_send`/`fx_return` boundary into
/// pre-section and post-section `PedalDef`s.
///
/// Returns `None` if the pedal has no FX loop (no `fx_send`/`fx_return`).
pub fn split_pedal_def(pedal: &PedalDef) -> Option<(PedalDef, PedalDef)> {
    if !pedal.has_fx_loop() {
        return None;
    }

    // Collect all node names that each component touches.
    let mut component_nodes: HashMap<String, Vec<String>> = HashMap::new();
    for net in &pedal.nets {
        let node_name = pin_node_name(&net.from);
        for pin in &net.to {
            let to_name = pin_node_name(pin);
            component_nodes
                .entry(node_name.clone())
                .or_default()
                .push(to_name.clone());
            component_nodes
                .entry(to_name)
                .or_default()
                .push(node_name.clone());
        }
    }

    // BFS from "in" — collect all component IDs reachable WITHOUT crossing fx_send→fx_return.
    // Stop at fx_send (include it as a boundary but don't cross to fx_return).
    let pre_components = bfs_collect_components(&component_nodes, "in", "fx_send");
    let post_components = bfs_collect_components(&component_nodes, "fx_return", "out");

    // Build pre PedalDef: filter to pre components, replace fx_send→out
    let pre_def = build_half(pedal, &pre_components, "fx_send", "out", "pre");
    // Build post PedalDef: filter to post components, replace fx_return→in
    let post_def = build_half(pedal, &post_components, "fx_return", "in", "post");

    Some((pre_def, post_def))
}

/// BFS from `start_node` through component adjacency, collecting component IDs.
/// Stops at `stop_node` (does not cross it to reach further components).
fn bfs_collect_components(
    adj: &HashMap<String, Vec<String>>,
    start_node: &str,
    stop_node: &str,
) -> std::collections::HashSet<String> {
    use std::collections::{HashSet, VecDeque};
    let mut visited = HashSet::new();
    let mut queue = VecDeque::new();
    queue.push_back(start_node.to_string());
    visited.insert(start_node.to_string());
    // Always include gnd and vcc — they're shared by all stages
    visited.insert("gnd".to_string());
    visited.insert("vcc".to_string());

    while let Some(node) = queue.pop_front() {
        if node == stop_node {
            // Include the stop node itself but don't traverse its neighbors
            continue;
        }
        if let Some(neighbors) = adj.get(&node) {
            for nb in neighbors {
                if !visited.contains(nb) {
                    visited.insert(nb.clone());
                    queue.push_back(nb.clone());
                }
            }
        }
    }
    visited
}

/// Build one half of a split pedal definition.
fn build_half(
    pedal: &PedalDef,
    component_ids: &std::collections::HashSet<String>,
    old_node: &str,
    new_node: &str,
    suffix: &str,
) -> PedalDef {
    // Filter components to those in this half
    let components: Vec<ComponentDef> = pedal
        .components
        .iter()
        .filter(|c| component_ids.contains(&c.id))
        .cloned()
        .collect();

    // Rename the boundary node and filter nets
    let rename = |pin: &Pin| -> Pin {
        match pin {
            Pin::Reserved(n) if n == old_node => Pin::Reserved(new_node.to_string()),
            // Drop the other fx node — it doesn't exist in this half
            Pin::Reserved(n) if (n == "fx_send" || n == "fx_return") && n != old_node => {
                return pin.clone(); // will be filtered out
            }
            _ => pin.clone(),
        }
    };

    let nets: Vec<NetDef> = pedal
        .nets
        .iter()
        .filter_map(|net| {
            let from = rename(&net.from);
            let to: Vec<Pin> = net.to.iter().map(&rename).collect();

            // A pin is "relevant" if it references a component in this half
            // or is a standard reserved node (in, out, gnd, vcc, or a supply rail).
            let relevant = |p: &Pin| match p {
                Pin::Reserved(n) => {
                    n == "in" || n == "out" || n == "gnd" || n == "vcc" || n == new_node
                        || pedal.is_supply_rail(n)
                }
                Pin::ComponentPin { component, .. } => component_ids.contains(component),
                Pin::Fork { switch, destinations } => {
                    component_ids.contains(switch)
                        || destinations.iter().any(|d| match d {
                            Pin::ComponentPin { component, .. } => component_ids.contains(component),
                            _ => false,
                        })
                }
            };

            // A pin "belongs" to this half — only component pins in our set.
            let belongs = |p: &Pin| match p {
                Pin::Reserved(_) => true,
                Pin::ComponentPin { component, .. } => component_ids.contains(component),
                Pin::Fork { switch, .. } => component_ids.contains(switch),
            };

            // Keep this net only if at least one component pin belongs to this half.
            // This avoids orphan nets like `C6.b -> out` when C6 is in the other half.
            let all_pins = std::iter::once(&from).chain(to.iter());
            let has_local_component = all_pins.clone().any(|p| {
                matches!(p, Pin::ComponentPin { component, .. } if component_ids.contains(component))
            });

            if has_local_component {
                let filtered_to: Vec<Pin> = to.into_iter().filter(|p| belongs(p)).collect();
                if !filtered_to.is_empty() && relevant(&from) {
                    Some(NetDef {
                        from,
                        to: filtered_to,
                    })
                } else {
                    None
                }
            } else {
                None
            }
        })
        .collect();

    // Filter controls to those referencing components in this half
    let controls: Vec<ControlDef> = pedal
        .controls
        .iter()
        .filter(|c| component_ids.contains(&c.component))
        .cloned()
        .collect();

    // Filter trims to those referencing components in this half
    let trims: Vec<ControlDef> = pedal
        .trims
        .iter()
        .filter(|c| component_ids.contains(&c.component))
        .cloned()
        .collect();

    let monitors: Vec<MonitorDef> = pedal
        .monitors
        .iter()
        .filter(|m| component_ids.contains(&m.component) || m.component == "input" || m.component == "output")
        .cloned()
        .collect();

    PedalDef {
        name: format!("{} ({})", pedal.name, suffix),
        supplies: pedal.supplies.clone(),
        components,
        nets,
        controls,
        trims,
        monitors,
        sidechains: vec![],
    }
}

/// A split compiled pedal — pre-section and post-section with an FX loop between them.
pub struct SplitCompiledPedal {
    pub pre: CompiledPedal,
    pub post: CompiledPedal,
}

impl SplitCompiledPedal {
    /// Process with no FX loop — direct connection from pre to post.
    pub fn process(&mut self, input: f64) -> f64 {
        let send = self.pre.process(input);
        self.post.process(send)
    }

    /// Process the pre-section only, returning the FX send signal.
    pub fn process_pre(&mut self, input: f64) -> f64 {
        self.pre.process(input)
    }

    /// Process the post-section only, taking the FX return signal.
    pub fn process_post(&mut self, fx_return: f64) -> f64 {
        self.post.process(fx_return)
    }

    pub fn set_sample_rate(&mut self, rate: f64) {
        self.pre.set_sample_rate(rate);
        self.post.set_sample_rate(rate);
    }

    pub fn reset(&mut self) {
        self.pre.reset();
        self.post.reset();
    }

    pub fn set_control(&mut self, label: &str, value: f64) {
        // Try both halves — controls are partitioned but the user shouldn't
        // need to know which half a control belongs to.
        self.pre.set_control(label, value);
        self.post.set_control(label, value);
    }

    /// Get the input impedance of the split pedal (Ω).
    ///
    /// Returns the input impedance of the pre-section.
    pub fn input_impedance(&self) -> f64 {
        self.pre.input_impedance()
    }

    /// Get the output impedance of the split pedal (Ω).
    ///
    /// Returns the output impedance of the post-section.
    pub fn output_impedance(&self) -> f64 {
        self.post.output_impedance()
    }
}

/// Compile a pedal with an FX loop, splitting at `fx_send`/`fx_return`.
///
/// Returns `Ok(SplitCompiledPedal)` if the pedal has send/return nodes.
/// Returns `Err` if the pedal has no FX loop or compilation fails.
pub fn compile_split_pedal(
    pedal: &PedalDef,
    sample_rate: f64,
) -> Result<SplitCompiledPedal, String> {
    let (pre_def, post_def) =
        split_pedal_def(pedal).ok_or_else(|| "Pedal has no fx_send/fx_return nodes".to_string())?;

    let pre = compile_pedal(&pre_def, sample_rate)
        .map_err(|e| format!("Failed to compile pre-section: {e}"))?;
    let post = compile_pedal(&post_def, sample_rate)
        .map_err(|e| format!("Failed to compile post-section: {e}"))?;

    Ok(SplitCompiledPedal { pre, post })
}

