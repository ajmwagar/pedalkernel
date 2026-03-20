//! Pass 1.5: Component self-classification via topology context.
//!
//! Provides `TopologyContext` so components can inspect their neighborhood
//! and self-classify their circuit topology. This allows components (especially
//! op-amps) to declare "I am a unity-gain buffer" or "I am an all-pass filter"
//! without the monolithic `find_opamp_feedback_loops()` doing all the work.
//!
//! Over time, each topology migrates into its component's `classify_topology()`
//! implementation, shrinking the monolith.

use std::collections::{HashMap, HashSet, VecDeque};

use crate::dsl::*;

use super::components::{Capacitor, Inductor, PhotocouplerComp, Potentiometer, Resistor};
use super::graph::{OpAmpFeedbackInfo, OpAmpFeedbackKind};

// ═══════════════════════════════════════════════════════════════════════════
// Resolved topology (shared across all classify_topology calls)
// ═══════════════════════════════════════════════════════════════════════════

/// Pre-resolved circuit pin topology for the entire netlist.
///
/// Built once per compilation from `PedalDef` nets using union-find.
/// Shared (by reference) across all `TopologyContext` instances.
pub(super) struct ResolvedTopology<'a> {
    /// Pin key (e.g., "U1.neg") → resolved node ID.
    pub(super) pin_to_node: HashMap<String, usize>,
    /// Ground node ID (resolved).
    pub(super) gnd_node: usize,
    /// AC-ground equivalent nodes (supply rails + large bypass cap nodes).
    pub(super) ac_ground_nodes: HashSet<usize>,
    /// Resistor/pot adjacency: node → [(neighbor, resistance, comp_id, is_pot, max_r)].
    pub(super) resistor_adj: HashMap<usize, Vec<(usize, f64, String, bool, f64)>>,
    /// Ground-leg adjacency: like resistor_adj but includes caps/inductors as 0Ω passthrough.
    /// Used by NonInverting detection to traverse R+C series paths (e.g., RAT R4→C5→gnd).
    pub(super) ground_leg_adj: HashMap<usize, Vec<(usize, f64, String, bool, f64)>>,
    /// Diode map: (node_a, node_b) → DiodeType. Both orderings stored.
    pub(super) diode_map: HashMap<(usize, usize), DiodeType>,
    /// Passive adjacency: node → [(neighbor, comp_id)]. Includes R, C, L, pots.
    pub(super) passive_adj: HashMap<usize, Vec<(usize, String)>>,
    /// Diode adjacency: node → [(neighbor, DiodeType)].
    pub(super) diode_adj: HashMap<usize, Vec<(usize, DiodeType)>>,
    /// Reference to the pedal definition.
    pub(super) pedal: &'a PedalDef,
}

impl<'a> ResolvedTopology<'a> {
    /// Build the resolved topology from a PedalDef.
    ///
    /// Performs union-find pin resolution (same algorithm as `find_opamp_feedback_loops`)
    /// and builds resistor adjacency maps for path queries.
    pub(super) fn from_pedal(pedal: &'a PedalDef) -> Self {
        let mut uf = UnionFind::new();
        let mut pin_ids: HashMap<String, usize> = HashMap::new();
        let mut next_id = 0usize;

        let mut get_id = |key: &str, uf: &mut UnionFind| -> usize {
            if let Some(&id) = pin_ids.get(key) {
                id
            } else {
                let id = next_id;
                next_id += 1;
                pin_ids.insert(key.to_string(), id);
                uf.ensure(id);
                id
            }
        };

        // Create special nodes.
        for name in &["in", "out", "gnd", "vcc"] {
            get_id(name, &mut uf);
        }
        for supply in &pedal.supplies {
            get_id(&supply.name, &mut uf);
        }

        // Union nets.
        for net in &pedal.nets {
            let from_id = get_id(&pin_key(&net.from), &mut uf);
            for to_pin in &net.to {
                let to_id = get_id(&pin_key(to_pin), &mut uf);
                uf.union(from_id, to_id);
            }
        }

        let gnd_node = uf.find(*pin_ids.get("gnd").unwrap_or(&0));

        // Build AC-ground set.
        let mut ac_ground_nodes: HashSet<usize> = HashSet::new();
        for supply in &pedal.supplies {
            if let Some(&raw_id) = pin_ids.get(&supply.name) {
                let resolved = uf.find(raw_id);
                if resolved != gnd_node {
                    ac_ground_nodes.insert(resolved);
                }
            }
        }
        // Large bypass caps (≥10µF) create AC ground at the non-ground terminal.
        for comp in &pedal.components {
            if let Some(cap) = comp.kind.as_any().downcast_ref::<Capacitor>() {
                if cap.config.value < 10e-6 {
                    continue;
                }
                let pa = format!("{}.a", comp.id);
                let pb = format!("{}.b", comp.id);
                if let (Some(&a_id), Some(&b_id)) = (pin_ids.get(&pa), pin_ids.get(&pb)) {
                    let a_node = uf.find(a_id);
                    let b_node = uf.find(b_id);
                    if a_node == gnd_node {
                        ac_ground_nodes.insert(b_node);
                    } else if b_node == gnd_node {
                        ac_ground_nodes.insert(a_node);
                    }
                }
            }
        }

        // Build resistor adjacency map.
        let mut resistor_adj: HashMap<usize, Vec<(usize, f64, String, bool, f64)>> =
            HashMap::new();
        for comp in &pedal.components {
            let (resistance, is_pot, max_r) =
                if let Some(pot) = comp.kind.as_any().downcast_ref::<Potentiometer>() {
                    (Some(pot.max_r * 0.5), true, pot.max_r)
                } else if let Some(r) = comp.kind.as_any().downcast_ref::<Resistor>() {
                    (Some(r.value), false, r.value)
                } else {
                    (None, false, 0.0)
                };

            if let Some(r) = resistance {
                let pin_a_key = format!("{}.a", comp.id);
                let pin_b_key = format!("{}.b", comp.id);
                if let (Some(&a_id), Some(&b_id)) =
                    (pin_ids.get(&pin_a_key), pin_ids.get(&pin_b_key))
                {
                    let a_node = uf.find(a_id);
                    let b_node = uf.find(b_id);

                    // 3-terminal pot: emit two half-segments (a→w, w→b).
                    if is_pot {
                        let wiper_key = format!("{}.w", comp.id);
                        let wiper_key_long = format!("{}.wiper", comp.id);
                        let wiper_id = pin_ids
                            .get(&wiper_key)
                            .or_else(|| pin_ids.get(&wiper_key_long));
                        if let Some(&w_id) = wiper_id {
                            let w_node = uf.find(w_id);
                            let aw_id = format!("{}__aw", comp.id);
                            let wb_id = format!("{}__wb", comp.id);
                            resistor_adj
                                .entry(a_node)
                                .or_default()
                                .push((w_node, max_r * 0.5, aw_id.clone(), true, max_r));
                            resistor_adj
                                .entry(w_node)
                                .or_default()
                                .push((a_node, max_r * 0.5, aw_id, true, max_r));
                            resistor_adj
                                .entry(w_node)
                                .or_default()
                                .push((b_node, max_r * 0.5, wb_id.clone(), true, max_r));
                            resistor_adj
                                .entry(b_node)
                                .or_default()
                                .push((w_node, max_r * 0.5, wb_id, true, max_r));
                            continue;
                        }
                    }

                    // 2-terminal.
                    resistor_adj
                        .entry(a_node)
                        .or_default()
                        .push((b_node, r, comp.id.clone(), is_pot, max_r));
                    resistor_adj
                        .entry(b_node)
                        .or_default()
                        .push((a_node, r, comp.id.clone(), is_pot, max_r));
                }
            }
        }

        // Ground-leg adjacency: resistor_adj + caps/inductors as 0Ω passthrough.
        let mut ground_leg_adj = resistor_adj.clone();
        for comp in &pedal.components {
            if comp.kind.as_any().downcast_ref::<Capacitor>().is_some()
                || comp.kind.as_any().downcast_ref::<Inductor>().is_some()
            {
                let pa = format!("{}.a", comp.id);
                let pb = format!("{}.b", comp.id);
                if let (Some(&a_id), Some(&b_id)) = (pin_ids.get(&pa), pin_ids.get(&pb)) {
                    let a_node = uf.find(a_id);
                    let b_node = uf.find(b_id);
                    ground_leg_adj
                        .entry(a_node)
                        .or_default()
                        .push((b_node, 0.0, comp.id.clone(), false, 0.0));
                    ground_leg_adj
                        .entry(b_node)
                        .or_default()
                        .push((a_node, 0.0, comp.id.clone(), false, 0.0));
                }
            }
        }

        // Diode map and adjacency.
        let mut diode_map: HashMap<(usize, usize), DiodeType> = HashMap::new();
        let mut diode_adj: HashMap<usize, Vec<(usize, DiodeType)>> = HashMap::new();
        for comp in &pedal.components {
            let dt = match comp.kind.diode_type() {
                Some(dt) => dt,
                None => continue,
            };
            let key_a = format!("{}.a", comp.id);
            let key_b = format!("{}.b", comp.id);
            if let (Some(&id_a), Some(&id_b)) = (pin_ids.get(&key_a), pin_ids.get(&key_b)) {
                let na = uf.find(id_a);
                let nb = uf.find(id_b);
                diode_map.insert((na, nb), dt);
                diode_map.insert((nb, na), dt);
                diode_adj.entry(na).or_default().push((nb, dt));
                diode_adj.entry(nb).or_default().push((na, dt));
            }
        }

        // Passive adjacency: R, C, L, pots.
        let mut passive_adj: HashMap<usize, Vec<(usize, String)>> = HashMap::new();
        for comp in &pedal.components {
            let is_simple_passive = comp.kind.as_any().downcast_ref::<Resistor>().is_some()
                || comp.kind.as_any().downcast_ref::<Capacitor>().is_some()
                || comp.kind.as_any().downcast_ref::<Inductor>().is_some()
                || comp.kind.as_any().downcast_ref::<PhotocouplerComp>().is_some();
            if is_simple_passive {
                let pa = format!("{}.a", comp.id);
                let pb = format!("{}.b", comp.id);
                if let (Some(&a_id), Some(&b_id)) = (pin_ids.get(&pa), pin_ids.get(&pb)) {
                    let a_node = uf.find(a_id);
                    let b_node = uf.find(b_id);
                    passive_adj
                        .entry(a_node)
                        .or_default()
                        .push((b_node, comp.id.clone()));
                    passive_adj
                        .entry(b_node)
                        .or_default()
                        .push((a_node, comp.id.clone()));
                }
            }
            if let Some(_pot) = comp.kind.as_any().downcast_ref::<Potentiometer>() {
                let pa = format!("{}.a", comp.id);
                let pb = format!("{}.b", comp.id);
                let pw = format!("{}.w", comp.id);
                let pw_long = format!("{}.wiper", comp.id);
                let wiper_id = pin_ids.get(&pw).or_else(|| pin_ids.get(&pw_long));
                if let Some(&w_id) = wiper_id {
                    if let Some(&a_id) = pin_ids.get(&pa) {
                        let a_node = uf.find(a_id);
                        let w_node = uf.find(w_id);
                        let aw_id = format!("{}__aw", comp.id);
                        passive_adj
                            .entry(a_node)
                            .or_default()
                            .push((w_node, aw_id.clone()));
                        passive_adj
                            .entry(w_node)
                            .or_default()
                            .push((a_node, aw_id));
                    }
                    if let Some(&b_id) = pin_ids.get(&pb) {
                        let b_node = uf.find(b_id);
                        let w_node = uf.find(w_id);
                        let wb_id = format!("{}__wb", comp.id);
                        passive_adj
                            .entry(w_node)
                            .or_default()
                            .push((b_node, wb_id.clone()));
                        passive_adj
                            .entry(b_node)
                            .or_default()
                            .push((w_node, wb_id));
                    }
                } else {
                    if let (Some(&a_id), Some(&b_id)) = (pin_ids.get(&pa), pin_ids.get(&pb)) {
                        let a_node = uf.find(a_id);
                        let b_node = uf.find(b_id);
                        passive_adj
                            .entry(a_node)
                            .or_default()
                            .push((b_node, comp.id.clone()));
                        passive_adj
                            .entry(b_node)
                            .or_default()
                            .push((a_node, comp.id.clone()));
                    }
                }
            }
        }

        // Resolve all pin keys to final node IDs.
        let pin_to_node: HashMap<String, usize> = pin_ids
            .iter()
            .map(|(key, &raw_id)| (key.clone(), uf.find(raw_id)))
            .collect();

        Self {
            pin_to_node,
            gnd_node,
            ac_ground_nodes,
            resistor_adj,
            ground_leg_adj,
            diode_map,
            passive_adj,
            diode_adj,
            pedal,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// TopologyContext (per-component)
// ═══════════════════════════════════════════════════════════════════════════

/// Topology context for a single component's self-classification.
///
/// Provides helper methods to inspect the circuit neighborhood:
/// pin nodes, grounding, resistive paths, and neighboring components.
pub(super) struct TopologyContext<'a> {
    /// This component's pin name → resolved node ID.
    pub pin_nodes: HashMap<&'a str, usize>,
    /// Reference to the shared resolved topology.
    resolved: &'a ResolvedTopology<'a>,
}

impl<'a> TopologyContext<'a> {
    /// Build a TopologyContext for a component with the given ID.
    pub(super) fn for_component(
        resolved: &'a ResolvedTopology<'a>,
        comp_id: &str,
        pin_names: &[&'a str],
    ) -> Self {
        let mut pin_nodes = HashMap::new();
        for &pin in pin_names {
            let key = format!("{}.{}", comp_id, pin);
            if let Some(&node) = resolved.pin_to_node.get(&key) {
                pin_nodes.insert(pin, node);
            }
        }
        Self {
            pin_nodes,
            resolved,
        }
    }

    /// Get the resolved node for a pin of this component.
    pub fn pin_node(&self, pin: &str) -> Option<usize> {
        self.pin_nodes.get(pin).copied()
    }

    /// True if `node` is the ground node.
    pub fn is_grounded(&self, node: usize) -> bool {
        node == self.resolved.gnd_node
    }

    /// True if `node` is ground or AC-ground equivalent (supply rail, bypass cap).
    pub fn is_ac_grounded(&self, node: usize) -> bool {
        node == self.resolved.gnd_node || self.resolved.ac_ground_nodes.contains(&node)
    }

    /// Find a resistive path between two nodes.
    ///
    /// Returns `(effective_resistance, component_ids, pot_info)`.
    pub fn find_resistive_path(
        &self,
        from: usize,
        to: usize,
    ) -> Option<(f64, Vec<String>, Option<(String, f64, f64, Option<f64>)>)> {
        super::graph::find_path_through_adj(from, to, &self.resolved.resistor_adj)
    }

    /// Find the input resistor at `neg_node` (connected to neg but not to `out_node` or `gnd`).
    pub fn find_input_resistor(&self, neg_node: usize, out_node: usize) -> Option<f64> {
        if let Some(neighbors) = self.resolved.resistor_adj.get(&neg_node) {
            for &(next, r, _, _, _) in neighbors {
                // Skip Rf (connects neg to out)
                if next == out_node {
                    continue;
                }
                // Skip ground leg resistors
                if self.is_ac_grounded(next) {
                    continue;
                }
                return Some(r);
            }
        }
        None
    }

    /// Find the input resistor at `neg_node`, also returning pot info if it's a pot.
    /// Returns `(ri, Option<(pot_comp_id, max_r, fixed_series_r)>)`.
    /// `fixed_series_r` is the sum of fixed resistors in series with the pot in the input path.
    pub fn find_input_resistor_with_pot(
        &self, neg_node: usize, out_node: usize,
    ) -> Option<(f64, Option<(String, f64, f64)>)> {
        if let Some(neighbors) = self.resolved.resistor_adj.get(&neg_node) {
            for &(next, r, ref id, is_pot, max_r) in neighbors {
                if next == out_node { continue; }
                if self.is_ac_grounded(next) { continue; }
                if is_pot {
                    // Found a pot in the input path. Check for fixed series R beyond it.
                    let mut fixed_r = 0.0;
                    if let Some(beyond) = self.resolved.resistor_adj.get(&next) {
                        for &(nn, rr, _, ip, _) in beyond {
                            if nn == neg_node { continue; }
                            if !ip { fixed_r += rr; }
                        }
                    }
                    // Strip __aw/__wb suffix to get the original pot comp_id
                    let pot_id = id.strip_suffix("__aw")
                        .or_else(|| id.strip_suffix("__wb"))
                        .unwrap_or(id)
                        .to_string();
                    return Some((r + fixed_r, Some((pot_id, max_r, fixed_r))));
                }
                return Some((r, None));
            }
        }
        None
    }

    /// Find a JFET component with a pin at the given node.
    ///
    /// Returns the component ID if found.
    pub fn find_jfet_at(&self, node: usize) -> Option<String> {
        for comp in &self.resolved.pedal.components {
            if !comp.kind.is_jfet() {
                continue;
            }
            for pin in &["source", "drain"] {
                let key = format!("{}.{}", comp.id, pin);
                if self.resolved.pin_to_node.get(&key) == Some(&node) {
                    return Some(comp.id.clone());
                }
            }
        }
        None
    }

    /// Find a JFET whose drain pin is at the given node.
    ///
    /// Returns the component ID if found.
    pub fn find_jfet_drain_at(&self, node: usize) -> Option<String> {
        for comp in &self.resolved.pedal.components {
            if !comp.kind.is_jfet() {
                continue;
            }
            let key = format!("{}.drain", comp.id);
            if self.resolved.pin_to_node.get(&key) == Some(&node) {
                return Some(comp.id.clone());
            }
        }
        None
    }

    /// Find a capacitor with a pin at the given node.
    ///
    /// Returns the capacitance value if found.
    pub fn find_cap_at(&self, node: usize) -> Option<f64> {
        for comp in &self.resolved.pedal.components {
            if let Some(cap) = comp.kind.as_any().downcast_ref::<Capacitor>() {
                let pa = format!("{}.a", comp.id);
                let pb = format!("{}.b", comp.id);
                if self.resolved.pin_to_node.get(&pa) == Some(&node)
                    || self.resolved.pin_to_node.get(&pb) == Some(&node)
                {
                    return Some(cap.config.value);
                }
            }
        }
        None
    }

    /// Find a capacitor spanning between two nodes.
    ///
    /// Returns the capacitance value if found.
    pub fn find_cap_between(&self, node_a: usize, node_b: usize) -> Option<f64> {
        for comp in &self.resolved.pedal.components {
            if let Some(cap) = comp.kind.as_any().downcast_ref::<Capacitor>() {
                let pa = format!("{}.a", comp.id);
                let pb = format!("{}.b", comp.id);
                if let (Some(&a), Some(&b)) = (
                    self.resolved.pin_to_node.get(&pa),
                    self.resolved.pin_to_node.get(&pb),
                ) {
                    if (a == node_a && b == node_b) || (a == node_b && b == node_a) {
                        return Some(cap.config.value);
                    }
                }
            }
        }
        None
    }

    /// Find a ground-leg path from `from` to `to` through resistors AND caps/inductors.
    ///
    /// Like `find_resistive_path` but caps/inductors are treated as 0Ω passthrough,
    /// so R→C→gnd series paths are discoverable (e.g., RAT's R4→C5→gnd).
    pub fn find_ground_leg_path(
        &self,
        from: usize,
        to: usize,
    ) -> Option<(f64, Vec<String>, Option<(String, f64, f64, Option<f64>)>)> {
        super::graph::find_path_through_adj(from, to, &self.resolved.ground_leg_adj)
    }

    /// Find feedback diodes between two nodes.
    ///
    /// Searches direct connections, 1-hop through passives, and chained diodes.
    /// Matches the logic in `find_opamp_feedback_loops` for Tube Screamer/Klon/Bluesbreaker.
    pub fn find_feedback_diode(&self, node_a: usize, node_b: usize) -> Option<DiodeType> {
        // Direct: diode spans (node_a, node_b)
        if let Some(&dt) = self.resolved.diode_map.get(&(node_a, node_b)) {
            return Some(dt);
        }
        // 1-hop from node_a through passive: diode at (intermediate, node_b)
        if let Some(neighbors) = self.resolved.passive_adj.get(&node_a) {
            for &(mid, _) in neighbors {
                if let Some(&dt) = self.resolved.diode_map.get(&(mid, node_b)) {
                    return Some(dt);
                }
                // 2-hop: passive → diode → diode → node_b
                if let Some(diode_neighbors) = self.resolved.diode_adj.get(&mid) {
                    for &(mid2, dt) in diode_neighbors {
                        if self.resolved.diode_map.get(&(mid2, node_b)).is_some() {
                            return Some(dt);
                        }
                    }
                }
            }
        }
        // 1-hop from node_b through passive: diode at (node_a, intermediate)
        if let Some(neighbors) = self.resolved.passive_adj.get(&node_b) {
            for &(mid, _) in neighbors {
                if let Some(&dt) = self.resolved.diode_map.get(&(node_a, mid)) {
                    return Some(dt);
                }
                // 2-hop: node_a → diode → diode → passive → node_b
                if let Some(diode_neighbors) = self.resolved.diode_adj.get(&mid) {
                    for &(mid2, dt) in diode_neighbors {
                        if self.resolved.diode_map.get(&(node_a, mid2)).is_some() {
                            return Some(dt);
                        }
                    }
                }
            }
        }
        // Direct chain: node_a → diode → diode → node_b
        if let Some(diode_neighbors) = self.resolved.diode_adj.get(&node_a) {
            for &(mid, dt) in diode_neighbors {
                if self.resolved.diode_map.get(&(mid, node_b)).is_some() {
                    return Some(dt);
                }
            }
        }
        None
    }

    /// Collect input-path photocouplers at neg_node for inverting opamps.
    ///
    /// BFS backward from neg_node through passive_adj (which now includes
    /// photocouplers), excluding the feedback direction (out_node) and ground.
    /// Returns (photocoupler_ids, input_fixed_r).
    pub fn collect_input_photocouplers(
        &self,
        neg_node: usize,
        out_node: usize,
        default_ri: f64,
    ) -> (Vec<String>, f64) {
        use std::collections::{HashSet, VecDeque};

        let mut input_pc_ids = Vec::new();
        let mut visited = HashSet::new();
        visited.insert(neg_node);
        visited.insert(out_node);
        visited.insert(self.resolved.gnd_node);
        for &ag in &self.resolved.ac_ground_nodes {
            visited.insert(ag);
        }

        // Collect opamp pin nodes as barriers
        let mut opamp_pins = HashSet::new();
        for comp in &self.resolved.pedal.components {
            if comp.kind.op_amp_type().is_some() {
                for suffix in &["neg", "out", "pos"] {
                    let key = format!("{}.{}", comp.id, suffix);
                    if let Some(&node) = self.resolved.pin_to_node.get(&key) {
                        opamp_pins.insert(node);
                    }
                }
            }
        }

        let mut queue = VecDeque::new();
        queue.push_back(neg_node);
        let mut fixed_r_sum = 0.0_f64;

        while let Some(node) = queue.pop_front() {
            if let Some(neighbors) = self.resolved.passive_adj.get(&node) {
                for (next, comp_id) in neighbors {
                    // Check if this component is a photocoupler BEFORE visited check —
                    // photocouplers often terminate at ground, which is pre-visited.
                    let is_pc = self.resolved.pedal.components.iter().any(|c| {
                        c.id == *comp_id
                            && c.kind.as_any().downcast_ref::<PhotocouplerComp>().is_some()
                    });
                    if is_pc {
                        input_pc_ids.push(comp_id.clone());
                        continue; // Don't traverse through photocoupler
                    }
                    if !visited.insert(*next) {
                        continue;
                    }
                    // Accumulate fixed resistance from resistor_adj
                    if let Some(neighbors_r) = self.resolved.resistor_adj.get(&node) {
                        for &(n, r, ref id, _, _) in neighbors_r {
                            if n == *next && id == comp_id {
                                fixed_r_sum += r;
                            }
                        }
                    }
                    // Don't expand past opamp pins
                    if !opamp_pins.contains(next) {
                        queue.push_back(*next);
                    }
                }
            }
        }

        if input_pc_ids.is_empty() {
            (Vec::new(), 0.0)
        } else {
            (input_pc_ids, fixed_r_sum.max(100.0))
        }
    }

    /// Collect feedback component IDs between two nodes via passive BFS.
    ///
    /// Used to build `feedback_comp_ids` for the OpAmpFeedbackInfo.
    pub fn collect_passive_comps_between(&self, start: usize, end: usize) -> Vec<String> {
        use std::collections::VecDeque;

        // Build passive adjacency for this query.
        let mut passive_adj: HashMap<usize, Vec<(usize, String)>> = HashMap::new();
        for comp in &self.resolved.pedal.components {
            let is_passive = comp.kind.as_any().downcast_ref::<Resistor>().is_some()
                || comp.kind.as_any().downcast_ref::<Capacitor>().is_some()
                || comp.kind.as_any().downcast_ref::<Inductor>().is_some()
                || comp.kind.as_any().downcast_ref::<PhotocouplerComp>().is_some();
            if !is_passive {
                if let Some(_pot) = comp.kind.as_any().downcast_ref::<Potentiometer>() {
                    // Handle 3-terminal pots.
                    let pa = format!("{}.a", comp.id);
                    let pb = format!("{}.b", comp.id);
                    let pw = format!("{}.w", comp.id);
                    let pw_long = format!("{}.wiper", comp.id);
                    let wiper_node = self
                        .resolved
                        .pin_to_node
                        .get(&pw)
                        .or_else(|| self.resolved.pin_to_node.get(&pw_long));
                    if let Some(&w_node) = wiper_node {
                        if let Some(&a_node) = self.resolved.pin_to_node.get(&pa) {
                            let aw_id = format!("{}__aw", comp.id);
                            passive_adj
                                .entry(a_node)
                                .or_default()
                                .push((w_node, aw_id.clone()));
                            passive_adj
                                .entry(w_node)
                                .or_default()
                                .push((a_node, aw_id));
                        }
                        if let Some(&b_node) = self.resolved.pin_to_node.get(&pb) {
                            let wb_id = format!("{}__wb", comp.id);
                            passive_adj
                                .entry(w_node)
                                .or_default()
                                .push((b_node, wb_id.clone()));
                            passive_adj
                                .entry(b_node)
                                .or_default()
                                .push((w_node, wb_id));
                        }
                    } else {
                        // 2-terminal pot.
                        if let (Some(&a_node), Some(&b_node)) = (
                            self.resolved.pin_to_node.get(&pa),
                            self.resolved.pin_to_node.get(&pb),
                        ) {
                            passive_adj
                                .entry(a_node)
                                .or_default()
                                .push((b_node, comp.id.clone()));
                            passive_adj
                                .entry(b_node)
                                .or_default()
                                .push((a_node, comp.id.clone()));
                        }
                    }
                }
                continue;
            }
            let pa = format!("{}.a", comp.id);
            let pb = format!("{}.b", comp.id);
            if let (Some(&a_node), Some(&b_node)) = (
                self.resolved.pin_to_node.get(&pa),
                self.resolved.pin_to_node.get(&pb),
            ) {
                passive_adj
                    .entry(a_node)
                    .or_default()
                    .push((b_node, comp.id.clone()));
                passive_adj
                    .entry(b_node)
                    .or_default()
                    .push((a_node, comp.id.clone()));
            }
        }

        // Barriers: ground, AC ground, opamp pins (except start/end).
        let mut barriers: HashSet<usize> = HashSet::new();
        barriers.insert(self.resolved.gnd_node);
        for &ag in &self.resolved.ac_ground_nodes {
            barriers.insert(ag);
        }
        // Add all opamp pin nodes as barriers.
        for comp in &self.resolved.pedal.components {
            if comp.kind.op_amp_type().is_some() {
                for suffix in &["neg", "out", "pos"] {
                    let key = format!("{}.{}", comp.id, suffix);
                    if let Some(&node) = self.resolved.pin_to_node.get(&key) {
                        barriers.insert(node);
                    }
                }
            }
        }
        barriers.remove(&start);
        barriers.remove(&end);

        // BFS forward.
        let mut visited: HashSet<usize> = HashSet::new();
        let mut parent_edges: HashMap<usize, Vec<(usize, String)>> = HashMap::new();
        let mut queue = VecDeque::new();
        visited.insert(start);
        queue.push_back(start);
        while let Some(node) = queue.pop_front() {
            if let Some(neighbors) = passive_adj.get(&node) {
                for (next, comp_id) in neighbors {
                    parent_edges
                        .entry(*next)
                        .or_default()
                        .push((node, comp_id.clone()));
                    if visited.insert(*next) {
                        if *next != end && !barriers.contains(next) {
                            queue.push_back(*next);
                        }
                    }
                }
            }
        }
        if !visited.contains(&end) {
            return Vec::new();
        }

        // BFS backward from end.
        let mut result_comps: HashSet<String> = HashSet::new();
        let mut back_visited: HashSet<usize> = HashSet::new();
        let mut back_queue = VecDeque::new();
        back_visited.insert(end);
        back_queue.push_back(end);
        while let Some(node) = back_queue.pop_front() {
            if let Some(parents) = parent_edges.get(&node) {
                for (prev, comp_id) in parents {
                    result_comps.insert(comp_id.clone());
                    if back_visited.insert(*prev) && *prev != start {
                        back_queue.push_back(*prev);
                    }
                }
            }
        }
        result_comps.into_iter().collect()
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Pass 1.5: classify_topologies
// ═══════════════════════════════════════════════════════════════════════════

/// Short-range BFS from `start` to `end` through `adj`, excluding `exclude`
/// node, limited to `max_hops` edges.  Used to find direct ground legs
/// (e.g. RAT neg→R4→C5→gnd = 2 hops) without traversing the entire signal
/// chain (e.g. Klon neg→R4→R2→R3→C2→…→gnd = many hops).
fn has_short_path_excluding(
    start: usize,
    end: usize,
    adj: &HashMap<usize, Vec<(usize, f64, String, bool, f64)>>,
    exclude: usize,
    max_hops: usize,
) -> bool {
    if start == end {
        return true;
    }
    if start == exclude || max_hops == 0 {
        return false;
    }
    let mut visited = HashSet::new();
    visited.insert(start);
    visited.insert(exclude);
    // BFS with depth tracking: (node, depth)
    let mut queue: VecDeque<(usize, usize)> = VecDeque::new();
    queue.push_back((start, 0));
    while let Some((node, depth)) = queue.pop_front() {
        if let Some(neighbors) = adj.get(&node) {
            for (next, _, _, _, _) in neighbors {
                if *next == end {
                    return true;
                }
                if depth + 1 < max_hops && visited.insert(*next) {
                    queue.push_back((*next, depth + 1));
                }
            }
        }
    }
    false
}

/// Returns a list of `OpAmpFeedbackInfo` for components that self-classified,
/// and a set of component IDs that were classified (to be skipped by the
/// monolithic `find_opamp_feedback_loops`).
pub(super) fn classify_topologies(
    pedal: &PedalDef,
    _sample_rate: f64,
) -> (Vec<OpAmpFeedbackInfo>, HashSet<String>) {
    let resolved = ResolvedTopology::from_pedal(pedal);
    let mut results = Vec::new();
    let mut classified_ids = HashSet::new();

    for comp in &pedal.components {
        // Only classify opamps for now (Phase B).
        let opamp_type = match comp.kind.op_amp_type() {
            Some(ot) if !ot.is_ota() => ot,
            _ => continue,
        };

        let ctx = TopologyContext::for_component(
            &resolved,
            &comp.id,
            &["neg", "pos", "out"],
        );

        if let Some(info) = classify_opamp(&ctx, &comp.id, opamp_type) {
            classified_ids.insert(comp.id.clone());
            results.push(info);
        }
    }

    (results, classified_ids)
}

/// Classify an op-amp's topology from its neighborhood context.
///
/// Detects:
/// - **Unity-gain buffer**: neg == out (voltage follower)
/// - **Allpass (gain-of-2)**: JFET at pos, Rf/Ri from neg to out, cap at pos
/// - **AllpassJfet (inverting)**: JFET drain at neg, Rf||Cf from neg to out
fn classify_opamp(
    ctx: &TopologyContext,
    comp_id: &str,
    opamp_type: OpAmpType,
) -> Option<OpAmpFeedbackInfo> {
    let neg_node = ctx.pin_node("neg")?;
    let pos_node = ctx.pin_node("pos")?;
    let out_node = ctx.pin_node("out")?;

    // ── Unity-gain buffer ────────────────────────────────────────────────
    if neg_node == out_node {
        return Some(OpAmpFeedbackInfo {
            comp_id: comp_id.to_string(),
            opamp_type,
            feedback_kind: OpAmpFeedbackKind::UnityGain,
            neg_node,
            pos_node,
            out_node,
            feedback_comp_ids: Vec::new(),
            input_photocoupler_ids: Vec::new(),
            input_fixed_r: 0.0,
        });
    }

    // Need Rf (feedback resistor from neg to out) for remaining topologies.
    let (rf, _rf_comps, rf_pot) = ctx.find_resistive_path(neg_node, out_node)?;
    let all_fb_comps = ctx.collect_passive_comps_between(neg_node, out_node);

    // ── Allpass (gain-of-2 style): JFET at pos ──────────────────────────
    // Phase 90 topology: Vin → R_ap → neg, R_fb → out, JFET+C → pos.
    if let Some(jfet_id) = ctx.find_jfet_at(pos_node) {
        if let Some(ri) = ctx.find_input_resistor(neg_node, out_node) {
            if let Some(cap) = ctx.find_cap_at(pos_node) {
                // Collect Ri comp ID too.
                let mut fb_comps = all_fb_comps.clone();
                if let Some(neighbors) = ctx.resolved.resistor_adj.get(&neg_node) {
                    for &(next, _, ref id, _, _) in neighbors {
                        if next == out_node || ctx.is_ac_grounded(next) {
                            continue;
                        }
                        if !fb_comps.contains(id) {
                            fb_comps.push(id.clone());
                        }
                        break;
                    }
                }
                return Some(OpAmpFeedbackInfo {
                    comp_id: comp_id.to_string(),
                    opamp_type,
                    feedback_kind: OpAmpFeedbackKind::Allpass {
                        rf,
                        ri,
                        jfet_id,
                        cap,
                    },
                    neg_node,
                    pos_node,
                    out_node,
                    feedback_comp_ids: fb_comps,
                    input_photocoupler_ids: Vec::new(),
                    input_fixed_r: 0.0,
                });
            }
        }
    }

    // ── AllpassJfet (inverting): JFET drain at neg, Cf from neg to out ───
    if let Some(jfet_id) = ctx.find_jfet_drain_at(neg_node) {
        if let Some(cf) = ctx.find_cap_between(neg_node, out_node) {
            return Some(OpAmpFeedbackInfo {
                comp_id: comp_id.to_string(),
                opamp_type,
                feedback_kind: OpAmpFeedbackKind::AllpassJfet {
                    rf,
                    cf,
                    jfet_id,
                },
                neg_node,
                pos_node,
                out_node,
                feedback_comp_ids: all_fb_comps,
                input_photocoupler_ids: Vec::new(),
                input_fixed_r: 0.0,
            });
        }
    }

    // ── Inverting amplifier ──────────────────────────────────────────────
    // pos grounded (directly or via resistive path to gnd/AC-gnd).
    //
    // IMPORTANT: If neg also has a ground leg (R→C→gnd or R→gnd), this
    // is a non-inverting topology with DC bias at pos (e.g. RAT: pos has
    // R3→R1→gnd bias path, but neg has R4→C5→gnd ground leg).
    // Only classify as Inverting when neg does NOT have a ground leg.
    let pos_grounded = ctx.is_ac_grounded(pos_node)
        || ctx.find_resistive_path(pos_node, ctx.resolved.gnd_node).is_some()
        || ctx
            .resolved
            .ac_ground_nodes
            .iter()
            .any(|&ag| ctx.find_resistive_path(pos_node, ag).is_some());
    // Check if neg has an INDEPENDENT ground leg (one that doesn't share
    // components with the feedback path out→neg).  This distinguishes
    // non-inverting (RAT: neg→R4→C5→gnd) from inverting (Tube Screamer:
    // neg→gnd path only exists through feedback/signal components).
    // Check for ground leg that doesn't pass through out_node.
    // This uses a custom BFS that excludes out_node from traversal,
    // so only direct neg→gnd paths (like RAT's R4→C5→gnd) are found.
    let neg_has_independent_ground_leg = {
        let check = |target: usize| -> bool {
            has_short_path_excluding(neg_node, target, &ctx.resolved.ground_leg_adj, out_node, 4)
        };
        check(ctx.resolved.gnd_node)
            || ctx
                .resolved
                .ac_ground_nodes
                .iter()
                .any(|&ag| check(ag))
    };
    if pos_grounded && !neg_has_independent_ground_leg {
        if let Some((ri, ri_pot_info)) = ctx.find_input_resistor_with_pot(neg_node, out_node) {
            let feedback_diode = ctx.find_feedback_diode(neg_node, out_node);

            // ── Shelving EQ detection (Klon Centaur style) ────────────────────
            // Before falling through to plain Inverting: check whether the feedback
            // network contains a cap AND a pot whose terminal "b" lands at out_node.
            // Topology: neg → C_tone → Treble.a ... Treble.b → out (in parallel with R_fb).
            // The pot "b" terminal at out distinguishes tone-control (shelving EQ)
            // from gain-control (terminal "b" at ground, never at out).
            // No diodes: diodes mean soft-clipper, not shelving EQ.
            if feedback_diode.is_none() {
                let cap_in_fb = all_fb_comps.iter().find_map(|id| {
                    ctx.resolved.pedal.components.iter()
                        .find(|c| c.id == *id)
                        .and_then(|c| c.kind.capacitance())
                });
                // A pot's "b" terminal is at out_node when the resistor_adj entry at
                // out_node contains a "__wb" entry whose comp_id is in all_fb_comps.
                let pot_at_out = if let Some(out_neighbors) = ctx.resolved.resistor_adj.get(&out_node) {
                    out_neighbors.iter().find_map(|(_next, _r, id, is_pot, max_r)| {
                        if !is_pot { return None; }
                        let wb_id = id;
                        if !wb_id.ends_with("__wb") { return None; }
                        if !all_fb_comps.contains(wb_id) { return None; }
                        let base = wb_id.strip_suffix("__wb")?;
                        Some((base.to_string(), *max_r))
                    })
                } else {
                    None
                };
                if let (Some(c_tone), Some((shelf_pot_id, max_pot_r))) = (cap_in_fb, pot_at_out) {
                    // Shelf resistor: in all_fb_comps, a non-pot resistor whose value ≠ rf.
                    let r_shelf = all_fb_comps.iter().find_map(|id| {
                        if let Some(neighbors) = ctx.resolved.resistor_adj.get(&neg_node) {
                            // Look in resistor_adj for this comp_id; we need to find by ID.
                            let _ = neighbors; // just a hint
                        }
                        // Search resistor_adj for an entry with this comp_id that is not a pot
                        // and whose resistance differs from rf.
                        for neighbors in ctx.resolved.resistor_adj.values() {
                            for &(_, r, ref cid, is_pot, _) in neighbors {
                                if cid == id && !is_pot && (r - rf).abs() > 1.0 {
                                    return Some(r);
                                }
                            }
                        }
                        None
                    }).unwrap_or(0.0);

                    let (input_pc_ids, input_fixed_r) = ctx.collect_input_photocouplers(neg_node, out_node, ri);
                    return Some(OpAmpFeedbackInfo {
                        comp_id: comp_id.to_string(),
                        opamp_type,
                        feedback_kind: OpAmpFeedbackKind::InvertingShelving {
                            rf,
                            ri,
                            c_tone,
                            r_shelf,
                            pot_id: shelf_pot_id,
                            max_pot_r,
                        },
                        neg_node,
                        pos_node,
                        out_node,
                        feedback_comp_ids: all_fb_comps,
                        input_photocoupler_ids: input_pc_ids,
                        input_fixed_r,
                    });
                }
            }

            // Collect input-path photocouplers at neg_node.
            let (input_pc_ids, input_fixed_r) = ctx.collect_input_photocouplers(neg_node, out_node, ri);

            return Some(OpAmpFeedbackInfo {
                comp_id: comp_id.to_string(),
                opamp_type,
                feedback_kind: OpAmpFeedbackKind::Inverting {
                    rf,
                    ri,
                    feedback_diode,
                    rf_pot,
                    ri_pot: ri_pot_info,
                },
                neg_node,
                pos_node,
                out_node,
                feedback_comp_ids: all_fb_comps,
                input_photocoupler_ids: input_pc_ids,
                input_fixed_r,
            });
        }
    }

    // ── All-pass check with JFET at pos (before NonInverting) ────────────
    // Moved above NonInverting so JFET-at-pos circuits aren't misclassified.
    // (Already handled above in the Allpass section.)

    // ── Non-inverting amplifier ──────────────────────────────────────────
    // Ri path from neg to ground (or AC ground). Pos is not grounded.
    // Use ground_leg_path to traverse R+C series paths (e.g., RAT R4→C5→gnd).
    let ni_gnd_node = if ctx.find_ground_leg_path(neg_node, ctx.resolved.gnd_node).is_some() {
        Some(ctx.resolved.gnd_node)
    } else {
        ctx.resolved
            .ac_ground_nodes
            .iter()
            .find(|&&ag| ctx.find_ground_leg_path(neg_node, ag).is_some())
            .copied()
    };
    if let Some(gnd_target) = ni_gnd_node {
        // Prefer pure-resistive path for ri/pot extraction (avoids bypass caps
        // inflating parallel conductance — e.g., Klon C_gnd creating 0Ω shortcut).
        // Fall back to ground_leg_path only when no resistive path exists.
        let (ri, _ri_comps, ri_pot) =
            if let Some(rp) = ctx.find_resistive_path(neg_node, gnd_target) {
                rp
            } else {
                ctx.find_ground_leg_path(neg_node, gnd_target)?
            };
        // Collect ground-leg passive components too.
        let gnd_leg_comps = ctx.collect_passive_comps_between(neg_node, gnd_target);
        let mut fb_comps = all_fb_comps.clone();
        for c in gnd_leg_comps {
            if !fb_comps.contains(&c) {
                fb_comps.push(c);
            }
        }
        return Some(OpAmpFeedbackInfo {
            comp_id: comp_id.to_string(),
            opamp_type,
            feedback_kind: OpAmpFeedbackKind::NonInverting {
                rf,
                ri,
                feedback_diode: None,
                rf_pot,
                ri_pot,
            },
            neg_node,
            pos_node,
            out_node,
            feedback_comp_ids: fb_comps,
            input_photocoupler_ids: Vec::new(),
            input_fixed_r: 0.0,
        });
    }

    // Unrecognized topology — let the monolith deal with it.
    None
}

// ═══════════════════════════════════════════════════════════════════════════
// Local UnionFind (same algorithm as graph.rs, avoids making that one pub)
// ═══════════════════════════════════════════════════════════════════════════

struct UnionFind {
    parent: Vec<usize>,
}

impl UnionFind {
    fn new() -> Self {
        Self {
            parent: Vec::new(),
        }
    }
    fn ensure(&mut self, id: usize) {
        while self.parent.len() <= id {
            let n = self.parent.len();
            self.parent.push(n);
        }
    }
    fn find(&mut self, mut x: usize) -> usize {
        while self.parent[x] != x {
            self.parent[x] = self.parent[self.parent[x]];
            x = self.parent[x];
        }
        x
    }
    fn union(&mut self, a: usize, b: usize) {
        let ra = self.find(a);
        let rb = self.find(b);
        if ra != rb {
            self.parent[rb] = ra;
        }
    }
}

/// Convert a DSL Pin to a string key for union-find resolution.
fn pin_key(pin: &Pin) -> String {
    match pin {
        Pin::Reserved(s) => s.clone(),
        Pin::ComponentPin { component, pin } => format!("{}.{}", component, pin),
        Pin::Fork { switch, .. } => format!("__fork_{}", switch),
    }
}
