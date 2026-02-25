//! Circuit graph construction, series-parallel decomposition, and WDF tree building.

use std::collections::{HashMap, HashSet};

use crate::dsl::*;
use crate::elements::{Photocoupler, PhotocouplerModel};

use super::dyn_node::DynNode;

// ═══════════════════════════════════════════════════════════════════════════
// Circuit graph
// ═══════════════════════════════════════════════════════════════════════════

pub(super) type NodeId = usize;

pub(super) struct GraphEdge {
    pub(super) comp_idx: usize,
    pub(super) node_a: NodeId,
    pub(super) node_b: NodeId,
}

pub(super) struct CircuitGraph {
    /// All two-terminal elements as edges.
    pub(super) edges: Vec<GraphEdge>,
    /// Component definitions (indexed by comp_idx).
    pub(super) components: Vec<ComponentDef>,
    /// Special nodes.
    pub(super) in_node: NodeId,
    pub(super) out_node: NodeId,
    pub(super) gnd_node: NodeId,
    /// Number of active elements found (opamps + transistors).
    #[allow(dead_code)]
    pub(super) num_active: usize,
    /// Edge indices for virtual bridge edges through active elements.
    /// These exist for BFS traversal but are not passive WDF tree elements.
    pub(super) active_edge_indices: Vec<usize>,
}

/// Simple union-find for grouping connected pins into circuit nodes.
struct UnionFind {
    parent: Vec<usize>,
}

impl UnionFind {
    fn new() -> Self {
        Self { parent: Vec::new() }
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

fn pin_key(pin: &Pin) -> String {
    match pin {
        Pin::Reserved(s) => s.clone(),
        Pin::ComponentPin { component, pin } => format!("{}.{}", component, pin),
    }
}

impl CircuitGraph {
    pub(super) fn from_pedal(pedal: &PedalDef) -> Self {
        // Pre-scan: identify pots that use .w (wiper) pin in nets.
        // A pot with .w is a 3-terminal pot: lug A, wiper, lug B.
        // Must be done before the get_id closure borrows pin_ids.
        let pots_with_wiper: HashSet<String> = {
            let mut set = HashSet::new();
            for net in &pedal.nets {
                let check_pin = |p: &Pin, s: &mut HashSet<String>| {
                    if let Pin::ComponentPin { component, pin } = p {
                        if pin == "w" {
                            s.insert(component.clone());
                        }
                    }
                };
                check_pin(&net.from, &mut set);
                for p in &net.to {
                    check_pin(p, &mut set);
                }
            }
            set
        };

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

        // Ensure reserved nodes exist.
        for name in &["in", "out", "gnd", "vcc"] {
            get_id(name, &mut uf);
        }

        // Union connected pins.
        for net in &pedal.nets {
            let from_id = get_id(&pin_key(&net.from), &mut uf);
            for to_pin in &net.to {
                let to_id = get_id(&pin_key(to_pin), &mut uf);
                uf.union(from_id, to_id);
            }
        }

        // Build edges for two-terminal components.
        let mut edges = Vec::new();
        let mut num_active = 0usize;
        let mut deferred_3term: Vec<(usize, String)> = Vec::new();

        for (idx, comp) in pedal.components.iter().enumerate() {
            match &comp.kind {
                ComponentKind::Potentiometer(_) => {
                    if pots_with_wiper.contains(&comp.id) {
                        // 3-terminal pot — defer to after loop
                        deferred_3term.push((idx, comp.id.clone()));
                    } else {
                        // 2-terminal pot — existing behavior
                        let key_a = format!("{}.a", comp.id);
                        let key_b = format!("{}.b", comp.id);
                        let id_a = get_id(&key_a, &mut uf);
                        let id_b = get_id(&key_b, &mut uf);
                        let node_a = uf.find(id_a);
                        let node_b = uf.find(id_b);
                        edges.push(GraphEdge {
                            comp_idx: idx,
                            node_a,
                            node_b,
                        });
                    }
                }
                ComponentKind::Resistor(_)
                | ComponentKind::Capacitor(_)
                | ComponentKind::Inductor(_)
                | ComponentKind::DiodePair(_)
                | ComponentKind::Diode(_)
                | ComponentKind::Zener(_)
                | ComponentKind::Photocoupler(_)
                | ComponentKind::Neon(_) => {
                    let key_a = format!("{}.a", comp.id);
                    let key_b = format!("{}.b", comp.id);
                    let id_a = get_id(&key_a, &mut uf);
                    let id_b = get_id(&key_b, &mut uf);
                    let node_a = uf.find(id_a);
                    let node_b = uf.find(id_b);
                    edges.push(GraphEdge {
                        comp_idx: idx,
                        node_a,
                        node_b,
                    });
                }
                ComponentKind::NJfet(_) | ComponentKind::PJfet(_) => {
                    // JFET: drain-source path is the WDF edge (like a diode).
                    // Gate is external control, not part of WDF tree.
                    let key_d = format!("{}.drain", comp.id);
                    let key_s = format!("{}.source", comp.id);
                    let id_d = get_id(&key_d, &mut uf);
                    let id_s = get_id(&key_s, &mut uf);
                    let node_d = uf.find(id_d);
                    let node_s = uf.find(id_s);
                    edges.push(GraphEdge {
                        comp_idx: idx,
                        node_a: node_d,
                        node_b: node_s,
                    });
                }
                ComponentKind::Triode(_) => {
                    // Triode: plate-cathode path is the WDF edge (like JFET drain-source).
                    // Grid is external control, not part of WDF tree.
                    let key_p = format!("{}.plate", comp.id);
                    let key_k = format!("{}.cathode", comp.id);
                    let id_p = get_id(&key_p, &mut uf);
                    let id_k = get_id(&key_k, &mut uf);
                    let node_p = uf.find(id_p);
                    let node_k = uf.find(id_k);
                    edges.push(GraphEdge {
                        comp_idx: idx,
                        node_a: node_p,
                        node_b: node_k,
                    });
                }
                ComponentKind::Pentode(_) => {
                    // Pentode: plate-cathode path is the WDF edge.
                    // Control grid (g1) and screen grid (g2) are external control.
                    let key_p = format!("{}.plate", comp.id);
                    let key_k = format!("{}.cathode", comp.id);
                    let id_p = get_id(&key_p, &mut uf);
                    let id_k = get_id(&key_k, &mut uf);
                    let node_p = uf.find(id_p);
                    let node_k = uf.find(id_k);
                    edges.push(GraphEdge {
                        comp_idx: idx,
                        node_a: node_p,
                        node_b: node_k,
                    });
                }
                ComponentKind::Nmos(_) | ComponentKind::Pmos(_) => {
                    // MOSFET: drain-source path is the WDF edge (like a JFET).
                    // Gate is external control, not part of WDF tree.
                    let key_d = format!("{}.drain", comp.id);
                    let key_s = format!("{}.source", comp.id);
                    let id_d = get_id(&key_d, &mut uf);
                    let id_s = get_id(&key_s, &mut uf);
                    let node_d = uf.find(id_d);
                    let node_s = uf.find(id_s);
                    edges.push(GraphEdge {
                        comp_idx: idx,
                        node_a: node_d,
                        node_b: node_s,
                    });
                }
                ComponentKind::Npn(_) | ComponentKind::Pnp(_) => {
                    num_active += 1;
                }
                ComponentKind::OpAmp(ot) if ot.is_ota() => {
                    // OTAs are nonlinear elements (current-controlled gain).
                    // Their pos/neg pins define the WDF edge (like a diode).
                    // Use pos/neg if available, otherwise fall back to generic 2-terminal.
                    let key_a = format!("{}.pos", comp.id);
                    let key_b = format!("{}.neg", comp.id);
                    let id_a = get_id(&key_a, &mut uf);
                    let id_b = get_id(&key_b, &mut uf);
                    let node_a = uf.find(id_a);
                    let node_b = uf.find(id_b);
                    edges.push(GraphEdge {
                        comp_idx: idx,
                        node_a,
                        node_b,
                    });
                }
                ComponentKind::OpAmp(_) => {
                    num_active += 1;
                }
                ComponentKind::Lfo(..) | ComponentKind::EnvelopeFollower(..) => {
                    // LFO and EnvelopeFollower are virtual modulation sources, not physical
                    // circuit elements. They connect via .out pin to modulation targets.
                }
                ComponentKind::Bbd(_) => {
                    // BBDs are handled as delay line processors, not WDF elements.
                    // They connect via .in/.out pins but are processed separately.
                }
                ComponentKind::DelayLine(..) => {
                    // Delay lines split the WDF tree into write/read subtrees.
                    // The compiler processes them separately as ring buffer processors.
                    // Connect via .input/.output pins; taps provide additional read ports.
                }
                ComponentKind::Tap(..) => {
                    // Taps are read-only ports into a parent delay line.
                    // They don't have WDF edges — they provide output from the delay buffer.
                }
                // ── Synth ICs ──────────────────────────────────────────
                // These are complex ICs with internal behavior. They are NOT
                // part of the WDF tree — they generate/process signals
                // independently and connect via their pin nodes.
                ComponentKind::Vco(_) => {
                    // VCO generates audio internally. Not a WDF element.
                    // Connects via .saw/.tri/.pulse output pins and .cv input.
                    num_active += 1;
                }
                ComponentKind::Vcf(_) => {
                    // VCF processes audio through internal OTA stages.
                    // .in/.out are signal path; .cv/.res are control.
                    num_active += 1;
                }
                ComponentKind::Vca(_) => {
                    // VCA is a gain-control element (exponential CV).
                    // .in/.out are signal path; .cv is control.
                    num_active += 1;
                }
                ComponentKind::Comparator(_) => {
                    // Binary output element. Not a WDF nonlinear root.
                    // .pos/.neg are inputs; .out is open-collector output.
                    num_active += 1;
                }
                ComponentKind::AnalogSwitch(_) => {
                    // Bilateral switch: low-R path when ctrl is high.
                    // Each channel: .in1/.out1/.ctrl1 through .in4/.out4/.ctrl4.
                    num_active += 1;
                }
                ComponentKind::MatchedNpn(_) | ComponentKind::MatchedPnp(_) => {
                    // Matched pairs: same as regular BJTs but with tighter Vbe matching.
                    num_active += 1;
                }
                ComponentKind::Tempco(_, _) => {
                    // Temperature-compensating resistor (2-terminal, like regular R).
                    let key_a = format!("{}.a", comp.id);
                    let key_b = format!("{}.b", comp.id);
                    let id_a = get_id(&key_a, &mut uf);
                    let id_b = get_id(&key_b, &mut uf);
                    let node_a = uf.find(id_a);
                    let node_b = uf.find(id_b);
                    edges.push(GraphEdge {
                        comp_idx: idx,
                        node_a,
                        node_b,
                    });
                }
                // ── Studio Equipment: Switched Components ────────────────
                // These represent physically switched component values (rotary selectors).
                // At compile time, we select a specific value based on control state.
                // For WDF, they behave as their base type (C or L).
                ComponentKind::CapSwitched(_values) => {
                    // Use first value as default; control system will select correct value
                    // during pre-compilation of WDF trees for each switch combination.
                    let key_a = format!("{}.a", comp.id);
                    let key_b = format!("{}.b", comp.id);
                    let id_a = get_id(&key_a, &mut uf);
                    let id_b = get_id(&key_b, &mut uf);
                    let node_a = uf.find(id_a);
                    let node_b = uf.find(id_b);
                    edges.push(GraphEdge {
                        comp_idx: idx,
                        node_a,
                        node_b,
                    });
                }
                ComponentKind::InductorSwitched(_values) => {
                    // Same as CapSwitched — 2-terminal element with selectable value.
                    let key_a = format!("{}.a", comp.id);
                    let key_b = format!("{}.b", comp.id);
                    let id_a = get_id(&key_a, &mut uf);
                    let id_b = get_id(&key_b, &mut uf);
                    let node_a = uf.find(id_a);
                    let node_b = uf.find(id_b);
                    edges.push(GraphEdge {
                        comp_idx: idx,
                        node_a,
                        node_b,
                    });
                }
                ComponentKind::Transformer(_cfg) => {
                    // Transformer: primary winding is a-b, secondary is c-d.
                    // For WDF, we model the ideal transformer as a 2-port adaptor
                    // with the turns ratio. The primary side is the WDF edge.
                    // NOTE: Full transformer WDF model needs implementation in tree.rs.
                    let key_a = format!("{}.a", comp.id);
                    let key_b = format!("{}.b", comp.id);
                    let id_a = get_id(&key_a, &mut uf);
                    let id_b = get_id(&key_b, &mut uf);
                    let node_a = uf.find(id_a);
                    let node_b = uf.find(id_b);
                    edges.push(GraphEdge {
                        comp_idx: idx,
                        node_a,
                        node_b,
                    });
                }
                ComponentKind::RotarySwitch(_) => {
                    // Rotary switch is a control element, not a circuit element.
                    // It determines which value to use for switched components.
                    // No WDF edge — it's handled by the control system.
                }
                ComponentKind::ResistorSwitched(_values) => {
                    // Same as CapSwitched — 2-terminal element with selectable value.
                    // Used for ratio selection networks (1176), feedback networks, etc.
                    let key_a = format!("{}.a", comp.id);
                    let key_b = format!("{}.b", comp.id);
                    let id_a = get_id(&key_a, &mut uf);
                    let id_b = get_id(&key_b, &mut uf);
                    let node_a = uf.find(id_a);
                    let node_b = uf.find(id_b);
                    edges.push(GraphEdge {
                        comp_idx: idx,
                        node_a,
                        node_b,
                    });
                }
                ComponentKind::Switch(_) => {
                    // Simple mechanical switch is a control element.
                    // It's used to select circuit topology (e.g., limit/compress mode).
                    // No WDF edge — handled by the control system.
                }
            }
        }

        // Process deferred 3-terminal pots. Each becomes two synthetic
        // Potentiometer components sharing the wiper node:
        //   {id}__aw: a → wiper (R = position * max_R)
        //   {id}__wb: wiper → b (R = (1 - position) * max_R)
        let mut extra_components: Vec<ComponentDef> = Vec::new();
        for (_original_idx, pot_id) in &deferred_3term {
            let max_r = match &pedal.components[*_original_idx].kind {
                ComponentKind::Potentiometer(r) => *r,
                _ => unreachable!(),
            };

            // Synthetic upper-half: a → wiper
            let aw_idx = pedal.components.len() + extra_components.len();
            extra_components.push(ComponentDef {
                id: format!("{pot_id}__aw"),
                kind: ComponentKind::Potentiometer(max_r),
            });

            // Synthetic lower-half: wiper → b
            let wb_idx = pedal.components.len() + extra_components.len();
            extra_components.push(ComponentDef {
                id: format!("{pot_id}__wb"),
                kind: ComponentKind::Potentiometer(max_r),
            });

            let key_a = format!("{pot_id}.a");
            let key_w = format!("{pot_id}.w");
            let key_b = format!("{pot_id}.b");
            let id_a = get_id(&key_a, &mut uf);
            let id_w = get_id(&key_w, &mut uf);
            let id_b = get_id(&key_b, &mut uf);

            edges.push(GraphEdge {
                comp_idx: aw_idx,
                node_a: uf.find(id_a),
                node_b: uf.find(id_w),
            });
            edges.push(GraphEdge {
                comp_idx: wb_idx,
                node_a: uf.find(id_w),
                node_b: uf.find(id_b),
            });
        }

        // Create virtual bridge edges for active elements (OpAmp, Npn, Pnp).
        // This ensures BFS can traverse through them for distance computation
        // and voltage source injection picks a proper connected node.
        let mut active_edge_indices = Vec::new();
        for comp in &pedal.components {
            let pin_order: &[&str] = match &comp.kind {
                ComponentKind::OpAmp(_) => {
                    // OpAmps use either 3-pin (pos/neg/out) or 2-pin (in/out) form.
                    if pin_ids.contains_key(&format!("{}.pos", comp.id)) {
                        &["pos", "neg", "out"]
                    } else {
                        &["in", "out"]
                    }
                }
                ComponentKind::Npn(_) | ComponentKind::Pnp(_) => &["base", "collector", "emitter"],
                _ => continue,
            };

            // Collect resolved node IDs for each pin that exists in the netlist.
            let mut pin_nodes: Vec<NodeId> = Vec::new();
            for pin_name in pin_order {
                let key = format!("{}.{}", comp.id, pin_name);
                if let Some(&raw_id) = pin_ids.get(&key) {
                    pin_nodes.push(uf.find(raw_id));
                }
            }

            // Chain consecutive pin pairs as virtual bridge edges.
            for pair in pin_nodes.windows(2) {
                if pair[0] != pair[1] {
                    active_edge_indices.push(edges.len());
                    edges.push(GraphEdge {
                        comp_idx: 0, // placeholder — active edges are excluded from WDF
                        node_a: pair[0],
                        node_b: pair[1],
                    });
                }
            }
        }

        let in_node = uf.find(*pin_ids.get("in").unwrap());
        let out_node = uf.find(*pin_ids.get("out").unwrap());
        let gnd_node = uf.find(*pin_ids.get("gnd").unwrap());

        // Append synthetic 3-terminal pot halves to the component list.
        let mut components = pedal.components.clone();
        components.extend(extra_components);

        CircuitGraph {
            edges,
            components,
            in_node,
            out_node,
            gnd_node,
            num_active,
            active_edge_indices,
        }
    }

    /// Find diode edges, ordered by topological distance from `in`.
    pub(super) fn find_diodes(&self) -> Vec<(usize, DiodeInfo)> {
        // BFS from in_node to compute distances.
        let mut adj: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
        for e in &self.edges {
            adj.entry(e.node_a).or_default().push(e.node_b);
            adj.entry(e.node_b).or_default().push(e.node_a);
        }
        let mut dist: HashMap<NodeId, usize> = HashMap::new();
        let mut queue = std::collections::VecDeque::new();
        dist.insert(self.in_node, 0);
        queue.push_back(self.in_node);
        while let Some(n) = queue.pop_front() {
            let d = dist[&n];
            if let Some(neighbors) = adj.get(&n) {
                for &nb in neighbors {
                    if let std::collections::hash_map::Entry::Vacant(e) = dist.entry(nb) {
                        e.insert(d + 1);
                        queue.push_back(nb);
                    }
                }
            }
        }

        let mut diodes: Vec<(usize, DiodeInfo)> = Vec::new();
        for (edge_idx, e) in self.edges.iter().enumerate() {
            let comp = &self.components[e.comp_idx];
            let info = match &comp.kind {
                ComponentKind::DiodePair(dt) => Some(DiodeInfo {
                    diode_type: *dt,
                    is_pair: true,
                    junction_node: if e.node_b == self.gnd_node {
                        e.node_a
                    } else {
                        e.node_b
                    },
                    ground_node: if e.node_b == self.gnd_node {
                        e.node_b
                    } else {
                        e.node_a
                    },
                }),
                ComponentKind::Diode(dt) => Some(DiodeInfo {
                    diode_type: *dt,
                    is_pair: false,
                    junction_node: if e.node_b == self.gnd_node {
                        e.node_a
                    } else {
                        e.node_b
                    },
                    ground_node: if e.node_b == self.gnd_node {
                        e.node_b
                    } else {
                        e.node_a
                    },
                }),
                _ => None,
            };
            if let Some(info) = info {
                diodes.push((edge_idx, info));
            }
        }

        // Sort by distance of junction node from input.
        diodes
            .sort_by_key(|(_, info)| dist.get(&info.junction_node).copied().unwrap_or(usize::MAX));
        diodes
    }

    /// Collect edge indices of passive elements directly connected to a junction node,
    /// excluding diode edges and edges on the direct output path.
    pub(super) fn elements_at_junction(
        &self,
        junction: NodeId,
        diode_edge_indices: &[usize],
        active_edge_indices: &[usize],
    ) -> Vec<usize> {
        self.edges
            .iter()
            .enumerate()
            .filter(|(idx, e)| {
                if diode_edge_indices.contains(idx) {
                    return false;
                }
                if active_edge_indices.contains(idx) {
                    return false;
                }
                // Must touch the junction.
                if e.node_a != junction && e.node_b != junction {
                    return false;
                }
                // Skip elements going directly to output (those become output attenuation).
                let other = if e.node_a == junction {
                    e.node_b
                } else {
                    e.node_a
                };
                if other == self.out_node {
                    return false;
                }
                true
            })
            .map(|(idx, _)| idx)
            .collect()
    }

    /// Find JFET edges, ordered by topological distance from `in`.
    pub(super) fn find_jfets(&self) -> Vec<(usize, JfetInfo)> {
        // BFS from in_node to compute distances.
        let mut adj: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
        for e in &self.edges {
            adj.entry(e.node_a).or_default().push(e.node_b);
            adj.entry(e.node_b).or_default().push(e.node_a);
        }
        let mut dist: HashMap<NodeId, usize> = HashMap::new();
        let mut queue = std::collections::VecDeque::new();
        dist.insert(self.in_node, 0);
        queue.push_back(self.in_node);
        while let Some(n) = queue.pop_front() {
            let d = dist[&n];
            if let Some(neighbors) = adj.get(&n) {
                for &nb in neighbors {
                    if let std::collections::hash_map::Entry::Vacant(e) = dist.entry(nb) {
                        e.insert(d + 1);
                        queue.push_back(nb);
                    }
                }
            }
        }

        let mut jfets: Vec<(usize, JfetInfo)> = Vec::new();
        for (edge_idx, e) in self.edges.iter().enumerate() {
            let comp = &self.components[e.comp_idx];
            let info = match &comp.kind {
                ComponentKind::NJfet(jt) => Some(JfetInfo {
                    jfet_type: *jt,
                    is_n_channel: true,
                    junction_node: if e.node_b == self.gnd_node {
                        e.node_a
                    } else {
                        e.node_b
                    },
                    ground_node: if e.node_b == self.gnd_node {
                        e.node_b
                    } else {
                        e.node_a
                    },
                }),
                ComponentKind::PJfet(jt) => Some(JfetInfo {
                    jfet_type: *jt,
                    is_n_channel: false,
                    junction_node: if e.node_b == self.gnd_node {
                        e.node_a
                    } else {
                        e.node_b
                    },
                    ground_node: if e.node_b == self.gnd_node {
                        e.node_b
                    } else {
                        e.node_a
                    },
                }),
                _ => None,
            };
            if let Some(info) = info {
                jfets.push((edge_idx, info));
            }
        }

        // Sort by distance of junction node from input.
        jfets.sort_by_key(|(_, info)| dist.get(&info.junction_node).copied().unwrap_or(usize::MAX));
        jfets
    }

    /// Find BJT edges, ordered by topological distance from `in`.
    pub(super) fn find_bjts(&self) -> Vec<(usize, BjtInfo)> {
        // BFS from in_node to compute distances.
        let mut adj: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
        for e in &self.edges {
            adj.entry(e.node_a).or_default().push(e.node_b);
            adj.entry(e.node_b).or_default().push(e.node_a);
        }
        let mut dist: HashMap<NodeId, usize> = HashMap::new();
        let mut queue = std::collections::VecDeque::new();
        dist.insert(self.in_node, 0);
        queue.push_back(self.in_node);
        while let Some(n) = queue.pop_front() {
            let d = dist[&n];
            if let Some(neighbors) = adj.get(&n) {
                for &nb in neighbors {
                    if let std::collections::hash_map::Entry::Vacant(e) = dist.entry(nb) {
                        e.insert(d + 1);
                        queue.push_back(nb);
                    }
                }
            }
        }

        let mut bjts: Vec<(usize, BjtInfo)> = Vec::new();
        for (edge_idx, e) in self.edges.iter().enumerate() {
            let comp = &self.components[e.comp_idx];
            let info = match &comp.kind {
                ComponentKind::Npn(bt) => Some(BjtInfo {
                    bjt_type: *bt,
                    is_npn: true,
                    junction_node: if e.node_b == self.gnd_node {
                        e.node_a
                    } else {
                        e.node_b
                    },
                    ground_node: if e.node_b == self.gnd_node {
                        e.node_b
                    } else {
                        e.node_a
                    },
                }),
                ComponentKind::Pnp(bt) => Some(BjtInfo {
                    bjt_type: *bt,
                    is_npn: false,
                    junction_node: if e.node_b == self.gnd_node {
                        e.node_a
                    } else {
                        e.node_b
                    },
                    ground_node: if e.node_b == self.gnd_node {
                        e.node_b
                    } else {
                        e.node_a
                    },
                }),
                _ => None,
            };
            if let Some(info) = info {
                bjts.push((edge_idx, info));
            }
        }

        // Sort by distance of junction node from input.
        bjts.sort_by_key(|(_, info)| dist.get(&info.junction_node).copied().unwrap_or(usize::MAX));
        bjts
    }

    /// Find triode edges, ordered by topological distance from `in`.
    pub(super) fn find_triodes(&self) -> Vec<(usize, TriodeInfo)> {
        // BFS from in_node to compute distances.
        let mut adj: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
        for e in &self.edges {
            adj.entry(e.node_a).or_default().push(e.node_b);
            adj.entry(e.node_b).or_default().push(e.node_a);
        }
        let mut dist: HashMap<NodeId, usize> = HashMap::new();
        let mut queue = std::collections::VecDeque::new();
        dist.insert(self.in_node, 0);
        queue.push_back(self.in_node);
        while let Some(n) = queue.pop_front() {
            let d = dist[&n];
            if let Some(neighbors) = adj.get(&n) {
                for &nb in neighbors {
                    if let std::collections::hash_map::Entry::Vacant(e) = dist.entry(nb) {
                        e.insert(d + 1);
                        queue.push_back(nb);
                    }
                }
            }
        }

        let mut triodes: Vec<(usize, TriodeInfo)> = Vec::new();
        for (edge_idx, e) in self.edges.iter().enumerate() {
            let comp = &self.components[e.comp_idx];
            if let ComponentKind::Triode(tt) = &comp.kind {
                triodes.push((
                    edge_idx,
                    TriodeInfo {
                        triode_type: *tt,
                        junction_node: if e.node_b == self.gnd_node {
                            e.node_a
                        } else {
                            e.node_b
                        },
                        ground_node: if e.node_b == self.gnd_node {
                            e.node_b
                        } else {
                            e.node_a
                        },
                    },
                ));
            }
        }

        // Sort by distance of junction node from input.
        triodes
            .sort_by_key(|(_, info)| dist.get(&info.junction_node).copied().unwrap_or(usize::MAX));
        triodes
    }

    /// Find pentode edges, ordered by topological distance from `in`.
    pub(super) fn find_pentodes(&self) -> Vec<(usize, PentodeInfo)> {
        let mut adj: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
        for e in &self.edges {
            adj.entry(e.node_a).or_default().push(e.node_b);
            adj.entry(e.node_b).or_default().push(e.node_a);
        }
        let mut dist: HashMap<NodeId, usize> = HashMap::new();
        let mut queue = std::collections::VecDeque::new();
        dist.insert(self.in_node, 0);
        queue.push_back(self.in_node);
        while let Some(n) = queue.pop_front() {
            let d = dist[&n];
            if let Some(neighbors) = adj.get(&n) {
                for &nb in neighbors {
                    if let std::collections::hash_map::Entry::Vacant(e) = dist.entry(nb) {
                        e.insert(d + 1);
                        queue.push_back(nb);
                    }
                }
            }
        }

        let mut pentodes: Vec<(usize, PentodeInfo)> = Vec::new();
        for (edge_idx, e) in self.edges.iter().enumerate() {
            let comp = &self.components[e.comp_idx];
            if let ComponentKind::Pentode(pt) = &comp.kind {
                pentodes.push((
                    edge_idx,
                    PentodeInfo {
                        pentode_type: *pt,
                        junction_node: if e.node_b == self.gnd_node {
                            e.node_a
                        } else {
                            e.node_b
                        },
                        ground_node: if e.node_b == self.gnd_node {
                            e.node_b
                        } else {
                            e.node_a
                        },
                    },
                ));
            }
        }

        pentodes
            .sort_by_key(|(_, info)| dist.get(&info.junction_node).copied().unwrap_or(usize::MAX));
        pentodes
    }

    /// Find MOSFET edges, ordered by topological distance from `in`.
    pub(super) fn find_mosfets(&self) -> Vec<(usize, MosfetInfo)> {
        let mut adj: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
        for e in &self.edges {
            adj.entry(e.node_a).or_default().push(e.node_b);
            adj.entry(e.node_b).or_default().push(e.node_a);
        }
        let mut dist: HashMap<NodeId, usize> = HashMap::new();
        let mut queue = std::collections::VecDeque::new();
        dist.insert(self.in_node, 0);
        queue.push_back(self.in_node);
        while let Some(n) = queue.pop_front() {
            let d = dist[&n];
            if let Some(neighbors) = adj.get(&n) {
                for &nb in neighbors {
                    if let std::collections::hash_map::Entry::Vacant(e) = dist.entry(nb) {
                        e.insert(d + 1);
                        queue.push_back(nb);
                    }
                }
            }
        }

        let mut mosfets: Vec<(usize, MosfetInfo)> = Vec::new();
        for (edge_idx, e) in self.edges.iter().enumerate() {
            let comp = &self.components[e.comp_idx];
            let info = match &comp.kind {
                ComponentKind::Nmos(mt) => Some(MosfetInfo {
                    mosfet_type: *mt,
                    is_n_channel: true,
                    junction_node: if e.node_b == self.gnd_node {
                        e.node_a
                    } else {
                        e.node_b
                    },
                    ground_node: if e.node_b == self.gnd_node {
                        e.node_b
                    } else {
                        e.node_a
                    },
                }),
                ComponentKind::Pmos(mt) => Some(MosfetInfo {
                    mosfet_type: *mt,
                    is_n_channel: false,
                    junction_node: if e.node_b == self.gnd_node {
                        e.node_a
                    } else {
                        e.node_b
                    },
                    ground_node: if e.node_b == self.gnd_node {
                        e.node_b
                    } else {
                        e.node_a
                    },
                }),
                _ => None,
            };
            if let Some(info) = info {
                mosfets.push((edge_idx, info));
            }
        }

        mosfets
            .sort_by_key(|(_, info)| dist.get(&info.junction_node).copied().unwrap_or(usize::MAX));
        mosfets
    }

    /// Find Zener diode edges, ordered by topological distance from `in`.
    pub(super) fn find_zeners(&self) -> Vec<(usize, ZenerInfo)> {
        let mut adj: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
        for e in &self.edges {
            adj.entry(e.node_a).or_default().push(e.node_b);
            adj.entry(e.node_b).or_default().push(e.node_a);
        }
        let mut dist: HashMap<NodeId, usize> = HashMap::new();
        let mut queue = std::collections::VecDeque::new();
        dist.insert(self.in_node, 0);
        queue.push_back(self.in_node);
        while let Some(n) = queue.pop_front() {
            let d = dist[&n];
            if let Some(neighbors) = adj.get(&n) {
                for &nb in neighbors {
                    if let std::collections::hash_map::Entry::Vacant(e) = dist.entry(nb) {
                        e.insert(d + 1);
                        queue.push_back(nb);
                    }
                }
            }
        }

        let mut zeners: Vec<(usize, ZenerInfo)> = Vec::new();
        for (edge_idx, e) in self.edges.iter().enumerate() {
            let comp = &self.components[e.comp_idx];
            if let ComponentKind::Zener(vz) = &comp.kind {
                zeners.push((
                    edge_idx,
                    ZenerInfo {
                        voltage: *vz,
                        junction_node: if e.node_b == self.gnd_node {
                            e.node_a
                        } else {
                            e.node_b
                        },
                        ground_node: if e.node_b == self.gnd_node {
                            e.node_b
                        } else {
                            e.node_a
                        },
                    },
                ));
            }
        }

        zeners
            .sort_by_key(|(_, info)| dist.get(&info.junction_node).copied().unwrap_or(usize::MAX));
        zeners
    }

    /// Find OTA (CA3080) edges, ordered by topological distance from `in`.
    pub(super) fn find_otas(&self) -> Vec<(usize, OtaInfo)> {
        let mut adj: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
        for e in &self.edges {
            adj.entry(e.node_a).or_default().push(e.node_b);
            adj.entry(e.node_b).or_default().push(e.node_a);
        }
        let mut dist: HashMap<NodeId, usize> = HashMap::new();
        let mut queue = std::collections::VecDeque::new();
        dist.insert(self.in_node, 0);
        queue.push_back(self.in_node);
        while let Some(n) = queue.pop_front() {
            let d = dist[&n];
            if let Some(neighbors) = adj.get(&n) {
                for &nb in neighbors {
                    if let std::collections::hash_map::Entry::Vacant(e) = dist.entry(nb) {
                        e.insert(d + 1);
                        queue.push_back(nb);
                    }
                }
            }
        }

        let mut otas: Vec<(usize, OtaInfo)> = Vec::new();
        for (edge_idx, e) in self.edges.iter().enumerate() {
            let comp = &self.components[e.comp_idx];
            if let ComponentKind::OpAmp(ot) = &comp.kind {
                if ot.is_ota() {
                    otas.push((
                        edge_idx,
                        OtaInfo {
                            junction_node: if e.node_b == self.gnd_node {
                                e.node_a
                            } else {
                                e.node_b
                            },
                            ground_node: if e.node_b == self.gnd_node {
                                e.node_b
                            } else {
                                e.node_a
                            },
                        },
                    ));
                }
            }
        }

        otas.sort_by_key(|(_, info)| {
            dist.get(&info.junction_node).copied().unwrap_or(usize::MAX)
        });
        otas
    }

}

pub(super) struct DiodeInfo {
    pub(super) diode_type: DiodeType,
    pub(super) is_pair: bool,
    pub(super) junction_node: NodeId,
    #[allow(dead_code)]
    pub(super) ground_node: NodeId,
}

pub(super) struct JfetInfo {
    pub(super) jfet_type: JfetType,
    pub(super) is_n_channel: bool,
    pub(super) junction_node: NodeId,
    #[allow(dead_code)]
    pub(super) ground_node: NodeId,
}

pub(super) struct BjtInfo {
    pub(super) bjt_type: BjtType,
    pub(super) is_npn: bool,
    /// The collector node (where the WDF tree connects).
    pub(super) junction_node: NodeId,
    #[allow(dead_code)]
    pub(super) ground_node: NodeId,
}

pub(super) struct TriodeInfo {
    pub(super) triode_type: TriodeType,
    pub(super) junction_node: NodeId,
    #[allow(dead_code)]
    pub(super) ground_node: NodeId,
}

pub(super) struct PentodeInfo {
    pub(super) pentode_type: PentodeType,
    pub(super) junction_node: NodeId,
    #[allow(dead_code)]
    pub(super) ground_node: NodeId,
}

pub(super) struct MosfetInfo {
    pub(super) mosfet_type: MosfetType,
    pub(super) is_n_channel: bool,
    pub(super) junction_node: NodeId,
    #[allow(dead_code)]
    pub(super) ground_node: NodeId,
}

pub(super) struct ZenerInfo {
    pub(super) voltage: f64,
    pub(super) junction_node: NodeId,
    #[allow(dead_code)]
    pub(super) ground_node: NodeId,
}

pub(super) struct OtaInfo {
    pub(super) junction_node: NodeId,
    #[allow(dead_code)]
    pub(super) ground_node: NodeId,
}

// ═══════════════════════════════════════════════════════════════════════════
// Series-parallel decomposition
// ═══════════════════════════════════════════════════════════════════════════

pub(super) enum SpTree {
    Leaf(usize), // component index
    Series(Box<SpTree>, Box<SpTree>),
    Parallel(Box<SpTree>, Box<SpTree>),
}

/// Reduce a set of edges into a single SP tree.
///
/// `edges`: (node_a, node_b, tree) — the initial edges.
/// `terminals`: nodes that must not be eliminated by series reduction.
pub(super) fn sp_reduce(
    mut edges: Vec<(NodeId, NodeId, SpTree)>,
    terminals: &[NodeId],
) -> Result<SpTree, String> {
    // Pre-process: redirect dead-end nodes to the first terminal.
    loop {
        let mut changed = false;
        let mut degree: HashMap<NodeId, Vec<usize>> = HashMap::new();
        for (idx, (a, b, _)) in edges.iter().enumerate() {
            degree.entry(*a).or_default().push(idx);
            degree.entry(*b).or_default().push(idx);
        }
        for (node, idxs) in &degree {
            if terminals.contains(node) || idxs.len() != 1 {
                continue;
            }
            // Dead-end: redirect this edge's dead side to terminals[0].
            let eidx = idxs[0];
            if edges[eidx].0 == *node {
                edges[eidx].0 = terminals[0];
            } else {
                edges[eidx].1 = terminals[0];
            }
            // Remove self-loops.
            if edges[eidx].0 == edges[eidx].1 {
                edges.remove(eidx);
            }
            changed = true;
            break;
        }
        if !changed {
            break;
        }
    }

    // SP reduction loop.
    loop {
        if edges.is_empty() {
            return Err("empty network".into());
        }
        if edges.len() == 1 {
            return Ok(edges.remove(0).2);
        }

        let mut changed = false;

        // Parallel reduction: edges with same endpoints.
        'par: for i in 0..edges.len() {
            for j in (i + 1)..edges.len() {
                let same = (edges[i].0 == edges[j].0 && edges[i].1 == edges[j].1)
                    || (edges[i].0 == edges[j].1 && edges[i].1 == edges[j].0);
                if same {
                    let (_, _, tree_j) = edges.remove(j);
                    let tree_i = std::mem::replace(&mut edges[i].2, SpTree::Leaf(0));
                    edges[i].2 = SpTree::Parallel(Box::new(tree_i), Box::new(tree_j));
                    changed = true;
                    break 'par;
                }
            }
        }
        if changed {
            continue;
        }

        // Series reduction: non-terminal nodes with degree 2.
        let mut degree: HashMap<NodeId, Vec<usize>> = HashMap::new();
        for (idx, (a, b, _)) in edges.iter().enumerate() {
            degree.entry(*a).or_default().push(idx);
            degree.entry(*b).or_default().push(idx);
        }
        for (node, idxs) in &degree {
            if terminals.contains(node) || idxs.len() != 2 {
                continue;
            }
            let i1 = idxs[0];
            let i2 = idxs[1];
            let other1 = if edges[i1].0 == *node {
                edges[i1].1
            } else {
                edges[i1].0
            };
            let other2 = if edges[i2].0 == *node {
                edges[i2].1
            } else {
                edges[i2].0
            };
            let (lo, hi) = if i1 < i2 { (i1, i2) } else { (i2, i1) };
            let (_, _, tree_hi) = edges.remove(hi);
            let (_, _, tree_lo) = edges.remove(lo);
            edges.push((
                other1,
                other2,
                SpTree::Series(Box::new(tree_lo), Box::new(tree_hi)),
            ));
            changed = true;
            break;
        }
        if changed {
            continue;
        }

        return Err(format!(
            "circuit is not series-parallel ({} edges remain)",
            edges.len()
        ));
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SP tree → DynNode conversion
// ═══════════════════════════════════════════════════════════════════════════

#[allow(dead_code)]
pub(super) fn sp_to_dyn(tree: &SpTree, components: &[ComponentDef], sample_rate: f64) -> DynNode {
    match tree {
        SpTree::Leaf(idx) => make_leaf(&components[*idx], sample_rate),
        SpTree::Series(left, right) => {
            let l = sp_to_dyn(left, components, sample_rate);
            let r = sp_to_dyn(right, components, sample_rate);
            let r1 = l.port_resistance();
            let r2 = r.port_resistance();
            let rp = r1 + r2;
            DynNode::Series {
                left: Box::new(l),
                right: Box::new(r),
                rp,
                gamma: r1 / rp,
                b1: 0.0,
                b2: 0.0,
            }
        }
        SpTree::Parallel(left, right) => {
            let l = sp_to_dyn(left, components, sample_rate);
            let r = sp_to_dyn(right, components, sample_rate);
            let r1 = l.port_resistance();
            let r2 = r.port_resistance();
            let rp = r1 * r2 / (r1 + r2);
            DynNode::Parallel {
                left: Box::new(l),
                right: Box::new(r),
                rp,
                gamma: r2 / (r1 + r2),
                b1: 0.0,
                b2: 0.0,
            }
        }
    }
}

pub(super) fn make_leaf(comp: &ComponentDef, sample_rate: f64) -> DynNode {
    match &comp.kind {
        ComponentKind::Resistor(r) => DynNode::Resistor { rp: *r },
        ComponentKind::Capacitor(c) => DynNode::Capacitor {
            capacitance: *c,
            rp: 1.0 / (2.0 * sample_rate * *c),
            state: 0.0,
        },
        ComponentKind::Inductor(l) => DynNode::Inductor {
            inductance: *l,
            rp: 2.0 * sample_rate * *l,
            state: 0.0,
        },
        ComponentKind::Potentiometer(max_r) => DynNode::Pot {
            comp_id: comp.id.clone(),
            max_resistance: *max_r,
            position: 0.5,
            rp: 0.5 * *max_r,
        },
        ComponentKind::Photocoupler(pt) => {
            let model = match pt {
                PhotocouplerType::Vtl5c3 => PhotocouplerModel::vtl5c3(),
                PhotocouplerType::Vtl5c1 => PhotocouplerModel::vtl5c1(),
                PhotocouplerType::Nsl32 => PhotocouplerModel::nsl32(),
                PhotocouplerType::T4b => PhotocouplerModel::t4b(),
            };
            DynNode::Photocoupler {
                comp_id: comp.id.clone(),
                inner: Photocoupler::new(model, sample_rate),
            }
        }
        // Tempco resistor: modeled as a standard resistor (nominal value).
        // Temperature compensation handled by the thermal model separately.
        ComponentKind::Tempco(r, _ppm) => DynNode::Resistor { rp: *r },
        // Diodes shouldn't appear as leaves (they're roots), but handle gracefully.
        _ => DynNode::Resistor { rp: 1000.0 },
    }
}
