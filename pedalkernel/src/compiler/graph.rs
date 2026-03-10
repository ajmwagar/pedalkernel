//! Circuit graph construction, series-parallel decomposition, and WDF tree building.

use std::collections::{BTreeMap, HashMap, HashSet};

use crate::dsl::*;
use super::component::{GraphRole, ResolveContext};
use super::components::*;
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
    /// The Vcc power rail node. Used as a BFS boundary (edges collected
    /// but traversal does not continue through it).
    pub(super) vcc_node: NodeId,
    /// Supply rail nodes (vcc + named supplies like B+, A_bal, etc.).
    /// Edges to these nodes are excluded from passive element collection
    /// because supply voltages are injected as tube bias parameters, not
    /// as part of the WDF tree.
    pub(super) supply_nodes: HashSet<NodeId>,
    /// Map from supply node ID → nominal voltage (volts).
    /// Used by push-pull builders to create CathodeBiasSource leaves.
    pub(super) supply_voltages: HashMap<NodeId, f64>,
    /// Number of active elements found (opamps + transistors).
    #[allow(dead_code)]
    pub(super) num_active: usize,
    /// Edge indices for virtual bridge edges through active elements.
    /// These exist for BFS traversal but are not passive WDF tree elements.
    pub(super) active_edge_indices: Vec<usize>,
    /// Fork path information: maps component index to ForkPathInfo.
    /// Only contains entries for synthetic fork path components.
    pub(super) fork_paths: HashMap<usize, ForkPathInfo>,
    /// Map from net/pin names to resolved NodeIds.
    /// Used for looking up named nodes like "A_node_ch_out" for sidechain routing.
    pub(super) node_names: HashMap<String, NodeId>,
    /// Magnetic/amplifying coupling: when one pin node of a multi-port device is
    /// reached during sidechain BFS, all other pin nodes should also be reachable.
    /// This covers transformers (primary↔secondary) and tubes (grid↔plate↔cathode).
    /// Maps each pin node → all other pin nodes of the same device.
    pub(super) coupled_nodes: HashMap<NodeId, Vec<NodeId>>,
    /// Transformer winding info: maps winding pin nodes to their transformer
    /// component index, whether they are secondary-side, and the turns ratio.
    /// Used by build.rs to apply inter-stage voltage gain across transformers.
    pub(super) transformer_info: HashMap<NodeId, TransformerNodeInfo>,
    /// Post-resolution edge classifications. Maps edge index → resolved EdgeKind.
    /// Only populated for edges whose kind changed during resolution.
    /// If an edge index is absent, use the component's default edges().
    pub(super) resolved_edge_kinds: HashMap<usize, super::component::EdgeKind>,
    /// Trigger components whose `.out` pin was NOT unioned to `in_node`.
    /// These triggers have explicit net connections and inject at their own
    /// resolved node IDs. Used by compile.rs to build per-voice WDF stages.
    pub(super) trigger_nodes: Vec<(String, NodeId)>,
}

/// Identifies which transformer winding a node belongs to.
#[derive(Clone, Debug)]
pub(super) struct TransformerNodeInfo {
    pub(super) comp_idx: usize,
    pub(super) is_secondary: bool,
    pub(super) turns_ratio: f64,
}

/// Result of partitioning sidechain edges from audio edges.
pub(super) struct SidechainPartition {
    /// Edge indices that belong to the sidechain path.
    pub(super) sidechain_edge_indices: HashSet<usize>,
    /// The node where audio is tapped for the sidechain.
    #[allow(dead_code)]
    pub(super) tap_node: NodeId,
    /// The node where the sidechain CV feeds back.
    #[allow(dead_code)]
    pub(super) cv_node: NodeId,
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
        // Fork destinations are handled specially in graph building
        Pin::Fork { switch, .. } => format!("__fork_{}", switch),
    }
}

/// Information about a synthetic fork path component.
/// Created when a `fork()` construct is encountered in nets.
#[derive(Clone)]
pub(super) struct ForkPathInfo {
    /// The switch component that controls this fork
    pub(super) switch_id: String,
    /// Which path index this represents (0, 1, 2, ...)
    pub(super) path_index: usize,
    /// Total number of paths in this fork
    pub(super) num_paths: usize,
}

/// Expand all Pin::Fork constructs into synthetic fork path components and modified nets.
/// Returns: (expanded_nets, fork_path_components)
fn expand_forks(
    nets: &[NetDef],
    _component_base_idx: usize,
) -> (Vec<NetDef>, Vec<(ComponentDef, ForkPathInfo)>) {
    let mut expanded_nets = Vec::new();
    let mut fork_components: Vec<(ComponentDef, ForkPathInfo)> = Vec::new();
    let mut fork_counter = 0usize;

    for net in nets {
        let mut new_to: Vec<Pin> = Vec::new();

        for dest in &net.to {
            match dest {
                Pin::Fork {
                    switch,
                    destinations,
                } => {
                    // Create a synthetic fork path component for each destination
                    let num_paths = destinations.len();
                    for (path_idx, dest_pin) in destinations.iter().enumerate() {
                        // Synthetic component ID: __fork_<counter>_path_<idx>
                        let comp_id = format!("__fork_{}_path_{}", fork_counter, path_idx);

                        // Create a synthetic resistor component (value will be set at runtime)
                        // Use a small resistance for active path, large for inactive
                        let comp = ComponentDef {
                            id: comp_id.clone(),
                            kind: Box::new(Resistor { value: 1.0 }), // Placeholder - actual value set by SwitchedResistor
                        };

                        let info = ForkPathInfo {
                            switch_id: switch.clone(),
                            path_index: path_idx,
                            num_paths,
                        };

                        fork_components.push((comp, info));

                        // Connect source → fork_path.a (done by adding fork_path.a to new_to)
                        new_to.push(Pin::ComponentPin {
                            component: comp_id.clone(),
                            pin: "a".to_string(),
                        });

                        // Connect fork_path.b → destination (new net)
                        expanded_nets.push(NetDef {
                            from: Pin::ComponentPin {
                                component: comp_id,
                                pin: "b".to_string(),
                            },
                            to: vec![dest_pin.clone()],
                        });
                    }
                    fork_counter += 1;
                }
                _ => {
                    // Regular pin - keep as-is
                    new_to.push(dest.clone());
                }
            }
        }

        // Add the original net with fork destinations replaced by fork path components
        if !new_to.is_empty() {
            expanded_nets.push(NetDef {
                from: net.from.clone(),
                to: new_to,
            });
        }
    }

    (expanded_nets, fork_components)
}

impl CircuitGraph {
    pub(super) fn from_pedal(pedal: &PedalDef) -> Self {
        // Expand fork() constructs into synthetic fork path components
        let (expanded_nets, fork_components) = expand_forks(&pedal.nets, pedal.components.len());

        // Build combined component list: original + fork paths
        let mut all_components: Vec<ComponentDef> = pedal.components.clone();
        let mut fork_paths: HashMap<usize, ForkPathInfo> = HashMap::new();

        for (comp, info) in fork_components {
            let comp_idx = all_components.len();
            fork_paths.insert(comp_idx, info);
            all_components.push(comp);
        }

        // Pre-scan: identify pots that use a wiper pin in nets.
        // A pot with a wiper pin is a 3-terminal pot: lug A, wiper, lug B.
        // Must be done before the get_id closure borrows pin_ids.
        const WIPER_PIN_NAMES: &[&str] = &["w", "wiper"];
        let pots_with_wiper: HashSet<String> = {
            let mut set = HashSet::new();
            for net in &expanded_nets {
                let check_pin = |p: &Pin, s: &mut HashSet<String>| {
                    if let Pin::ComponentPin { component, pin } = p {
                        if WIPER_PIN_NAMES.contains(&pin.as_str()) {
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
        // Also create nodes for named supply rails (e.g., "V+", "V-", "B+").
        for supply in &pedal.supplies {
            get_id(&supply.name, &mut uf);
        }

        // Union connected pins (using expanded nets).
        for net in &expanded_nets {
            let from_id = get_id(&pin_key(&net.from), &mut uf);
            for to_pin in &net.to {
                let to_id = get_id(&pin_key(to_pin), &mut uf);
                uf.union(from_id, to_id);
            }
        }

        // Stereo I/O aliasing for equipment circuits.
        // When 'in'/'out' have no net connections but 'in_L'/'out_L' exist, alias them
        // so that BFS from in_node can reach the circuit components.
        {
            let has_reserved = |nets: &[NetDef], name: &str| -> bool {
                nets.iter().any(|net| {
                    let check = |p: &Pin| matches!(p, Pin::Reserved(s) if s == name);
                    check(&net.from) || net.to.iter().any(check)
                })
            };
            if !has_reserved(&expanded_nets, "in") && has_reserved(&expanded_nets, "in_L") {
                let in_id = get_id("in", &mut uf);
                let in_l_id = get_id("in_L", &mut uf);
                uf.union(in_id, in_l_id);
            }
            if !has_reserved(&expanded_nets, "out") && has_reserved(&expanded_nets, "out_L") {
                let out_id = get_id("out", &mut uf);
                let out_l_id = get_id("out_L", &mut uf);
                uf.union(out_id, out_l_id);
            }
        }

        // Auto-union trigger_input out pin with in_node when `in` has no explicit nets
        // AND the trigger's .out pin has no explicit net connections.
        // When triggers have explicit nets (e.g., T_C.out -> R_C_trig.a in multi-voice
        // synth circuits), keep them separate so each trigger injects at its own node.
        {
            let in_has_nets = expanded_nets.iter().any(|net| {
                let check = |p: &Pin| matches!(p, Pin::Reserved(s) if s == "in");
                check(&net.from) || net.to.iter().any(check)
            });
            if !in_has_nets {
                for comp in &all_components {
                    if comp.kind.is_trigger() {
                        // Check if this trigger's .out pin appears in any net.
                        let trigger_has_nets = expanded_nets.iter().any(|net| {
                            let check = |p: &Pin| {
                                matches!(p, Pin::ComponentPin { component, pin }
                                    if component == &comp.id && pin == "out")
                            };
                            check(&net.from) || net.to.iter().any(check)
                        });
                        if !trigger_has_nets {
                            let trigger_out = format!("{}.out", comp.id);
                            let trig_id = get_id(&trigger_out, &mut uf);
                            let in_id = get_id("in", &mut uf);
                            uf.union(trig_id, in_id);
                        }
                    }
                }
            }
        }

        // Build edges for two-terminal components.
        let mut edges = Vec::new();
        let mut num_active = 0usize;
        let mut deferred_3term: Vec<(usize, String)> = Vec::new();
        let mut coupled_nodes: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
        let mut transformer_info: HashMap<NodeId, TransformerNodeInfo> = HashMap::new();

        for (idx, comp) in all_components.iter().enumerate() {
            match comp.kind.graph_role() {
                GraphRole::Edge { pin_a, pin_b } => {
                    let key_a = format!("{}.{}", comp.id, pin_a);
                    let key_b = format!("{}.{}", comp.id, pin_b);
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
                GraphRole::ActiveEdge { pin_a, pin_b } => {
                    let key_a = format!("{}.{}", comp.id, pin_a);
                    let key_b = format!("{}.{}", comp.id, pin_b);
                    let id_a = get_id(&key_a, &mut uf);
                    let id_b = get_id(&key_b, &mut uf);
                    let node_a = uf.find(id_a);
                    let node_b = uf.find(id_b);
                    edges.push(GraphEdge {
                        comp_idx: idx,
                        node_a,
                        node_b,
                    });
                    num_active += 1;
                }
                GraphRole::CoupledEdge {
                    edge_pin_a,
                    edge_pin_b,
                    coupled_pins,
                } => {
                    // Resolve all coupled pins through union-find.
                    let coupled_node_ids: Vec<NodeId> = coupled_pins
                        .iter()
                        .map(|pin| {
                            let key = format!("{}.{}", comp.id, pin);
                            let id = get_id(&key, &mut uf);
                            uf.find(id)
                        })
                        .collect();

                    // Find edge pin nodes from the resolved set.
                    let edge_a_pos = coupled_pins
                        .iter()
                        .position(|&p| p == edge_pin_a)
                        .unwrap();
                    let edge_b_pos = coupled_pins
                        .iter()
                        .position(|&p| p == edge_pin_b)
                        .unwrap();
                    let node_a = coupled_node_ids[edge_a_pos];
                    let node_b = coupled_node_ids[edge_b_pos];

                    edges.push(GraphEdge {
                        comp_idx: idx,
                        node_a,
                        node_b,
                    });

                    // Record coupling for sidechain BFS traversal.
                    for &n in &coupled_node_ids {
                        let others: Vec<_> = coupled_node_ids
                            .iter()
                            .copied()
                            .filter(|&x| x != n)
                            .collect();
                        coupled_nodes.entry(n).or_default().extend(others);
                    }
                }
                GraphRole::Pot => {
                    if pots_with_wiper.contains(&comp.id) {
                        deferred_3term.push((idx, comp.id.clone()));
                    } else {
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
                GraphRole::Transformer => {
                    // Complex multi-winding handling: aliases, coupling, center taps.
                    let cfg = comp.kind.transformer_config()
                        .expect("Transformer GraphRole requires transformer_config()");

                    // Winding pin aliases: union shorthand (.a/.b) with explicit
                    // (.primary.a/.primary.b) and abbreviated (.pri.a/.pri.b).
                    let windings: &[(&str, &str, &[&str])] = &[
                        ("a", "b", &["primary", "pri"]),
                        ("c", "d", &["secondary", "sec"]),
                    ];
                    for &(short_a, short_b, prefixes) in windings {
                        let id_short_a =
                            get_id(&format!("{}.{}", comp.id, short_a), &mut uf);
                        let id_short_b =
                            get_id(&format!("{}.{}", comp.id, short_b), &mut uf);
                        for prefix in prefixes {
                            let alias_a =
                                get_id(&format!("{}.{}.a", comp.id, prefix), &mut uf);
                            let alias_b =
                                get_id(&format!("{}.{}.b", comp.id, prefix), &mut uf);
                            uf.union(id_short_a, alias_a);
                            uf.union(id_short_b, alias_b);
                        }
                    }

                    // Tertiary winding aliases (only if transformer has third winding)
                    if cfg.has_tertiary() {
                        let id_e = get_id(&format!("{}.e", comp.id), &mut uf);
                        let id_f = get_id(&format!("{}.f", comp.id), &mut uf);
                        for prefix in &["tertiary", "ter"] {
                            let alias_a =
                                get_id(&format!("{}.{}.a", comp.id, prefix), &mut uf);
                            let alias_b =
                                get_id(&format!("{}.{}.b", comp.id, prefix), &mut uf);
                            uf.union(id_e, alias_a);
                            uf.union(id_f, alias_b);
                        }
                    }

                    let id_a = get_id(&format!("{}.a", comp.id), &mut uf);
                    let id_b = get_id(&format!("{}.b", comp.id), &mut uf);
                    let id_c = get_id(&format!("{}.c", comp.id), &mut uf);
                    let id_d = get_id(&format!("{}.d", comp.id), &mut uf);

                    let node_a = uf.find(id_a);
                    let node_b = uf.find(id_b);
                    edges.push(GraphEdge {
                        comp_idx: idx,
                        node_a,
                        node_b,
                    });

                    // Record transformer winding coupling for sidechain BFS.
                    let node_c = uf.find(id_c);
                    let node_d = uf.find(id_d);
                    let mut all_winding_nodes = vec![node_a, node_b, node_c, node_d];
                    if cfg.has_tertiary() {
                        let ter_e = get_id(&format!("{}.e", comp.id), &mut uf);
                        let ter_f = get_id(&format!("{}.f", comp.id), &mut uf);
                        all_winding_nodes.push(uf.find(ter_e));
                        all_winding_nodes.push(uf.find(ter_f));
                    }
                    if matches!(
                        cfg.primary_type,
                        WindingType::CenterTap | WindingType::PushPull
                    ) {
                        let ct_id =
                            get_id(&format!("{}.primary.ct", comp.id), &mut uf);
                        let ct_abbr =
                            get_id(&format!("{}.pri.ct", comp.id), &mut uf);
                        let ct_short =
                            get_id(&format!("{}.ct", comp.id), &mut uf);
                        uf.union(ct_id, ct_abbr);
                        uf.union(ct_id, ct_short);
                        all_winding_nodes.push(uf.find(ct_id));
                    }
                    all_winding_nodes.sort();
                    all_winding_nodes.dedup();
                    for &n in &all_winding_nodes {
                        let others: Vec<NodeId> = all_winding_nodes
                            .iter()
                            .copied()
                            .filter(|&x| x != n)
                            .collect();
                        coupled_nodes.entry(n).or_default().extend(others);
                    }

                    // Record transformer winding info for inter-stage voltage gain.
                    let n = cfg.turns_ratio;
                    for node in [node_a, node_b] {
                        transformer_info.insert(
                            node,
                            TransformerNodeInfo {
                                comp_idx: idx,
                                is_secondary: false,
                                turns_ratio: n,
                            },
                        );
                    }
                    for node in [node_c, node_d] {
                        transformer_info.insert(
                            node,
                            TransformerNodeInfo {
                                comp_idx: idx,
                                is_secondary: true,
                                turns_ratio: n,
                            },
                        );
                    }
                }
                GraphRole::Virtual => {}
                GraphRole::ActiveIc => {
                    num_active += 1;
                }
            }
        }

        // Process deferred 3-terminal pots. Each becomes two synthetic
        // Potentiometer components sharing the wiper node:
        //   {id}__aw: a → wiper (R = position * max_R)
        //   {id}__wb: wiper → b (R = (1 - position) * max_R)
        let mut extra_components: Vec<ComponentDef> = Vec::new();
        for (_original_idx, pot_id) in &deferred_3term {
            let pot_comp = pedal.components[*_original_idx].kind.as_any()
                .downcast_ref::<Potentiometer>()
                .expect("deferred 3-term component must be a Potentiometer");
            let (max_r, taper) = (pot_comp.max_r, pot_comp.taper);

            // Synthetic upper-half: a → wiper
            let aw_idx = pedal.components.len() + extra_components.len();
            extra_components.push(ComponentDef {
                id: format!("{pot_id}__aw"),
                kind: Box::new(Potentiometer { max_r, taper }),
            });

            // Synthetic lower-half: wiper → b
            let wb_idx = pedal.components.len() + extra_components.len();
            extra_components.push(ComponentDef {
                id: format!("{pot_id}__wb"),
                kind: Box::new(Potentiometer { max_r, taper }),
            });

            let key_a = format!("{pot_id}.a");
            let key_b = format!("{pot_id}.b");
            let id_a = get_id(&key_a, &mut uf);
            let id_b = get_id(&key_b, &mut uf);

            // Alias all wiper pin variants so nets using any name
            // (e.g. ".w" or ".wiper") resolve to the same UF group.
            let id_w = get_id(&format!("{pot_id}.{}", WIPER_PIN_NAMES[0]), &mut uf);
            for &alias in &WIPER_PIN_NAMES[1..] {
                let id_alias = get_id(&format!("{pot_id}.{alias}"), &mut uf);
                uf.union(id_w, id_alias);
            }

            let resolved_a = uf.find(id_a);
            let resolved_w = uf.find(id_w);
            let resolved_b = uf.find(id_b);
            edges.push(GraphEdge {
                comp_idx: aw_idx,
                node_a: resolved_a,
                node_b: resolved_w,
            });
            edges.push(GraphEdge {
                comp_idx: wb_idx,
                node_a: resolved_w,
                node_b: resolved_b,
            });
        }

        // Union pin aliases (e.g. .in↔.input, .out↔.output) so pedal
        // authors can use either spelling interchangeably.
        {
            for comp in &pedal.components {
                for &(short, long) in comp.kind.pin_config().aliases {
                    let id_short = get_id(&format!("{}.{}", comp.id, short), &mut uf);
                    let id_long = get_id(&format!("{}.{}", comp.id, long), &mut uf);
                    uf.union(id_short, id_long);
                }
            }
        }

        // Create virtual bridge edges for active elements (OpAmp, Npn, Pnp).
        // This ensures BFS can traverse through them for distance computation
        // and voltage source injection picks a proper connected node.
        let mut active_edge_indices = Vec::new();
        for comp in &pedal.components {
            let pin_order: &[&str] = if comp.kind.op_amp_type().is_some() {
                // OpAmps use either 3-pin (pos/neg/out) or 2-pin (in/out) form.
                if pin_ids.contains_key(&format!("{}.pos", comp.id)) {
                    &["pos", "neg", "out"]
                } else {
                    &["in", "out"]
                }
            } else if comp.kind.is_bjt() {
                &["base", "collector", "emitter"]
            } else {
                continue;
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

        // Re-resolve all edge node IDs through the final union-find state.
        // This is necessary because transformer pin aliasing (and potentially
        // other late unions) can merge nodes AFTER edges were stored.
        // Without this, edges may reference stale union-find roots.
        for edge in &mut edges {
            edge.node_a = uf.find(edge.node_a);
            edge.node_b = uf.find(edge.node_b);
        }

        // Re-resolve transformer_info and coupled_nodes through final UF state.
        // These were recorded during component processing, before all unions
        // were complete. Without re-resolution, transformer secondary nodes
        // may not match the nodes used by downstream passive edges.
        let transformer_info: HashMap<NodeId, TransformerNodeInfo> = transformer_info
            .into_iter()
            .map(|(node, info)| (uf.find(node), info))
            .collect();
        let coupled_nodes: HashMap<NodeId, Vec<NodeId>> = coupled_nodes
            .into_iter()
            .map(|(node, others)| {
                (uf.find(node), others.into_iter().map(|n| uf.find(n)).collect())
            })
            .collect();

        let in_node = uf.find(*pin_ids.get("in").unwrap());
        let out_node = uf.find(*pin_ids.get("out").unwrap());
        let gnd_node = uf.find(*pin_ids.get("gnd").unwrap());

        // Collect secondary supply rail nodes from named supplies.
        // Edges to these nodes are excluded from passive element collection
        // because they represent bias voltages that create non-SP subgraphs.
        // NOTE: vcc and gnd are NOT excluded — they are valid WDF terminations
        // (voltage source for plate loads, ground for cathode resistors).
        let vcc_node = uf.find(*pin_ids.get("vcc").unwrap());
        let mut supply_nodes: HashSet<NodeId> = HashSet::new();
        let mut supply_voltages: HashMap<NodeId, f64> = HashMap::new();
        for supply in &pedal.supplies {
            if let Some(&raw_id) = pin_ids.get(&supply.name) {
                let resolved = uf.find(raw_id);
                // Skip vcc and gnd — they're standard WDF terminations
                if resolved != vcc_node && resolved != gnd_node {
                    supply_nodes.insert(resolved);
                    supply_voltages.insert(resolved, supply.config.voltage);
                }
            }
        }

        // Append synthetic 3-terminal pot halves to the component list.
        // Start from all_components (which includes fork paths) instead of pedal.components.
        let mut components = all_components;
        components.extend(extra_components);

        // Build node_names map: resolve all pin names through union-find.
        // This allows looking up named nodes like "A_node_ch_out" for sidechain routing.
        let mut node_names: HashMap<String, NodeId> = HashMap::new();
        for (name, &raw_id) in &pin_ids {
            node_names.insert(name.clone(), uf.find(raw_id));
        }

        // Collect trigger components whose .out pin was NOT unioned to in_node.
        // These triggers have explicit net connections and need per-voice stages.
        let trigger_nodes: Vec<(String, NodeId)> = components
            .iter()
            .filter(|c| c.kind.is_trigger())
            .filter_map(|c| {
                let trigger_out = format!("{}.out", c.id);
                if let Some(&raw_id) = pin_ids.get(&trigger_out) {
                    let resolved = uf.find(raw_id);
                    if resolved != in_node {
                        Some((c.id.clone(), resolved))
                    } else {
                        None
                    }
                } else {
                    None
                }
            })
            .collect();

        CircuitGraph {
            edges,
            components,
            in_node,
            out_node,
            gnd_node,
            vcc_node,
            supply_nodes,
            supply_voltages,
            num_active,
            active_edge_indices,
            fork_paths,
            node_names,
            coupled_nodes,
            transformer_info,
            resolved_edge_kinds: HashMap::new(),
            trigger_nodes,
        }
    }

    /// Get the effective edge kind for a given edge index.
    ///
    /// Returns the resolved kind if the edge was resolved during wiring resolution,
    /// otherwise falls back to the component's default edges().
    pub(super) fn effective_edge_kind(&self, edge_idx: usize) -> super::component::EdgeKind {
        if let Some(&kind) = self.resolved_edge_kinds.get(&edge_idx) {
            return kind;
        }
        let comp = &self.components[self.edges[edge_idx].comp_idx];
        let edges = comp.kind.edges();
        // Most components have exactly one edge; return its kind.
        edges.first().map(|e| e.kind).unwrap_or(super::component::EdgeKind::Linear)
    }

    /// Partition edges into sidechain and audio paths.
    ///
    /// BFS from `tap_node` through all edges, collecting edges that lead
    /// toward `cv_node`. The sidechain path branches off at the tap node
    /// and ends at the CV node. All edges on this path are "sidechain edges"
    /// that should be built into a `SidechainProcessor` instead of the
    /// main audio WDF chain.
    ///
    /// Uses bidirectional BFS intersection: nodes reachable from tap_node
    /// intersected with nodes reachable from cv_node, both stopping at
    /// circuit boundary nodes (in, out, gnd, vcc, supply rails). This
    /// isolates the sidechain subgraph without relying on component names.
    pub(super) fn partition_sidechain(
        &self,
        tap_node: NodeId,
        cv_node: NodeId,
    ) -> Option<SidechainPartition> {
        // Build adjacency list: node → [(edge_idx, neighbor_node)]
        let mut adj: HashMap<NodeId, Vec<(usize, NodeId)>> = HashMap::new();
        for (idx, e) in self.edges.iter().enumerate() {
            adj.entry(e.node_a).or_default().push((idx, e.node_b));
            adj.entry(e.node_b).or_default().push((idx, e.node_a));
        }

        // Hard boundary nodes: the BFS stops here completely.
        // Audio I/O nodes are hard boundaries — they truly separate
        // the sidechain from the audio input/output path.
        // For stereo circuits, include both L and R output/input nodes
        // to prevent cross-channel leakage through decode matrices.
        let mut boundary: HashSet<NodeId> = HashSet::new();
        boundary.insert(self.in_node);
        boundary.insert(self.out_node);
        for suffix in &["in_R", "out_R", "in_L", "out_L"] {
            if let Some(&n) = self.node_names.get(*suffix) {
                boundary.insert(n);
            }
        }

        // Soft boundaries: reachable (included in node set) but NOT traversed.
        // This includes gnd and all supply rails (vcc, vcc_sc, bias rails).
        // Both audio path and sidechain components connect to gnd/supply, so
        // these are shared infrastructure, not path boundaries. Making them
        // "soft" means BFS discovers edges TO them but doesn't follow edges
        // FROM them (which would leak into the entire circuit).
        let mut supply_boundary: HashSet<NodeId> = HashSet::new();
        supply_boundary.insert(self.gnd_node);
        if let Some(&vcc) = self.node_names.get("vcc") {
            supply_boundary.insert(vcc);
        }
        supply_boundary.extend(&self.supply_nodes);

        // Build filtered coupled_nodes that excludes tube coupling for
        // tubes whose grid terminal is at cv_node (or tap_node).
        //
        // In variable-mu compressors (Fairchild 670), the sidechain CV node
        // is wired directly to the push-pull tube grids. Union-find merges
        // cv_node with all tube grid nodes. Without this filter, the BFS
        // from cv would traverse grid→plate→cathode via tube coupling,
        // absorbing the entire audio path into the sidechain partition.
        //
        // The fix: remove tube coupling entries for cv_node and its coupled
        // partners (the plate/cathode nodes of tubes whose grid is at cv).
        // Sidechain tubes (whose grids are at different nodes) keep their
        // coupling. The BFS can still traverse plate↔cathode through the
        // regular graph edge (tube WDF edge), so audio-path traversal is
        // unaffected — only the grid↔plate shortcut is disabled.
        let filtered_coupled = {
            let mut fc = self.coupled_nodes.clone();
            let mut exclude: HashSet<NodeId> = HashSet::new();
            if self.coupled_nodes.contains_key(&cv_node) {
                exclude.insert(cv_node);
                if let Some(coupled) = self.coupled_nodes.get(&cv_node) {
                    exclude.extend(coupled);
                }
            }
            if !exclude.is_empty() {
                for node in &exclude {
                    fc.remove(node);
                }
                for (_, coupled_list) in fc.iter_mut() {
                    coupled_list.retain(|n| !exclude.contains(n));
                }
            }
            fc
        };

        // BFS from tap_node: find all nodes reachable without crossing
        // cv_node or any hard boundary. Supply/gnd nodes are reachable but not
        // traversed. Transformer windings allow crossing magnetic isolation.
        let reachable_from_tap = Self::bfs_reachable(
            tap_node,
            cv_node,
            &boundary,
            &supply_boundary,
            &filtered_coupled,
            &adj,
        );

        // BFS from cv_node: same rules, opposite direction.
        let reachable_from_cv = Self::bfs_reachable(
            cv_node,
            tap_node,
            &boundary,
            &supply_boundary,
            &filtered_coupled,
            &adj,
        );

        // Sidechain nodes = intersection of both reachable sets.
        // Nodes reachable from BOTH tap and cv (without crossing boundaries)
        // are exclusively sidechain components. Audio-path nodes are only
        // reachable from one side because boundaries block the other direction.
        let mut sidechain_nodes: HashSet<NodeId> = reachable_from_tap
            .intersection(&reachable_from_cv)
            .copied()
            .collect();

        // Include the boundary nodes themselves (tap and cv)
        sidechain_nodes.insert(tap_node);
        sidechain_nodes.insert(cv_node);

        // Collect edges where both endpoints are sidechain nodes.
        let sidechain_edges: HashSet<usize> = self
            .edges
            .iter()
            .enumerate()
            .filter(|(_, e)| {
                sidechain_nodes.contains(&e.node_a) && sidechain_nodes.contains(&e.node_b)
            })
            .map(|(i, _)| i)
            .collect();

        if sidechain_edges.is_empty() {
            return None;
        }

        Some(SidechainPartition {
            sidechain_edge_indices: sidechain_edges,
            tap_node,
            cv_node,
        })
    }

    /// BFS from `start`, collecting all reachable nodes.
    ///
    /// - Stops completely at `opposite` and hard `boundary` nodes (in/out only).
    /// - Supply/gnd nodes (`supply_boundary`) are reachable and included in the
    ///   result, but the BFS does NOT traverse through them to their neighbors.
    /// - Transformer magnetic coupling (`xfmr_coupled`): when a transformer winding
    ///   pin is reached, all other winding pins of that transformer are also seeded
    ///   into the BFS. This allows traversal across galvanic isolation boundaries.
    ///
    /// The start node itself is NOT included in the result (it's a boundary).
    fn bfs_reachable(
        start: NodeId,
        opposite: NodeId,
        boundary: &HashSet<NodeId>,
        supply_boundary: &HashSet<NodeId>,
        xfmr_coupled: &HashMap<NodeId, Vec<NodeId>>,
        adj: &HashMap<NodeId, Vec<(usize, NodeId)>>,
    ) -> HashSet<NodeId> {
        let mut visited: HashSet<NodeId> = HashSet::new();
        let mut queue = std::collections::VecDeque::new();
        visited.insert(start);

        // Helper: try to enqueue a node (returns true if newly visited)
        let try_enqueue =
            |node: NodeId,
             visited: &mut HashSet<NodeId>,
             queue: &mut std::collections::VecDeque<NodeId>| {
                if node == start || node == opposite || boundary.contains(&node) {
                    return;
                }
                if visited.insert(node) {
                    // Supply/gnd nodes are reachable but not traversed
                    if !supply_boundary.contains(&node) {
                        queue.push_back(node);
                    }
                }
            };

        // Seed: follow all edges from start
        if let Some(neighbors) = adj.get(&start) {
            for &(_, neighbor) in neighbors {
                try_enqueue(neighbor, &mut visited, &mut queue);
            }
        }
        // Also seed transformer-coupled nodes of start
        if let Some(coupled) = xfmr_coupled.get(&start) {
            for &c in coupled {
                try_enqueue(c, &mut visited, &mut queue);
            }
        }

        while let Some(node) = queue.pop_front() {
            // Follow graph edges
            if let Some(neighbors) = adj.get(&node) {
                for &(_, neighbor) in neighbors {
                    try_enqueue(neighbor, &mut visited, &mut queue);
                }
            }
            // Follow transformer magnetic coupling
            if let Some(coupled) = xfmr_coupled.get(&node) {
                for &c in coupled {
                    try_enqueue(c, &mut visited, &mut queue);
                }
            }
        }

        // Remove start from the result — it's a boundary, not a sidechain-internal node
        visited.remove(&start);
        visited
    }

    /// Collect edge indices of passive elements directly connected to a junction node,
    /// excluding diode edges, edges on the direct output path, and edges to supply rails.
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
                let other = if e.node_a == junction {
                    e.node_b
                } else {
                    e.node_a
                };
                // Skip elements going directly to output (those become output attenuation).
                if other == self.out_node {
                    return false;
                }
                // Skip elements whose far node is a supply rail (vcc, B+, bias voltages).
                // Supply voltages are injected as tube bias parameters, not as WDF tree nodes.
                if self.supply_nodes.contains(&other) {
                    return false;
                }
                true
            })
            .map(|(idx, _)| idx)
            .collect()
    }

    /// Find triode edges, ordered by topological distance from `in`.
    /// Parallel tubes sharing both plate and cathode nodes are merged into
    /// a single entry with `parallel_count > 1`.
    /// Returns (merged_triodes, all_triode_edge_indices) where the second vec
    /// contains ALL triode edge indices including parallel duplicates.
    #[cfg(test)]
    pub(super) fn find_triodes(&self) -> (Vec<(usize, TriodeInfo)>, Vec<usize>) {
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

        // Collect all triode edges (including variable-mu triodes).
        let mut raw_triodes: Vec<(usize, String, NodeId, NodeId, bool)> = Vec::new();
        for (edge_idx, e) in self.edges.iter().enumerate() {
            let comp = &self.components[e.comp_idx];
            if let Some(triode) = comp.kind.as_any().downcast_ref::<Triode>() {
                raw_triodes.push((edge_idx, triode.model.clone(), e.node_a, e.node_b, false));
            } else if let Some(varimu) = comp.kind.as_any().downcast_ref::<VariMu>() {
                raw_triodes.push((edge_idx, varimu.model.clone(), e.node_a, e.node_b, true));
            }
        }

        // Group by (plate_node, cathode_node) to detect parallel tubes.
        // Tubes sharing both nodes are electrically identical and should be
        // modeled as a single tube with scaled plate current.
        let mut groups: HashMap<(NodeId, NodeId), Vec<(usize, String, bool)>> = HashMap::new();
        for (edge_idx, name, plate, cathode, is_vm) in &raw_triodes {
            groups
                .entry((*plate, *cathode))
                .or_default()
                .push((*edge_idx, name.clone(), *is_vm));
        }

        let mut triodes: Vec<(usize, TriodeInfo)> = Vec::new();
        for ((plate_node, cathode_node), group) in &groups {
            // Use the first edge as the representative; store parallel count.
            let (rep_edge_idx, ref rep_name, rep_is_vari_mu) = group[0];
            triodes.push((
                rep_edge_idx,
                TriodeInfo {
                    model_name: rep_name.clone(),
                    plate_node: *plate_node,
                    cathode_node: *cathode_node,
                    junction_node: *cathode_node,
                    ground_node: self.gnd_node,
                    parallel_count: group.len(),
                    is_vari_mu: rep_is_vari_mu,
                },
            ));
        }

        // Collect all triode edge indices (including parallel duplicates).
        let all_triode_edges: Vec<usize> =
            raw_triodes.iter().map(|(idx, _, _, _, _)| *idx).collect();

        // Sort by distance of junction node from input, with edge_idx as tiebreaker
        // for deterministic ordering when distances are equal.
        triodes.sort_by_key(|(edge_idx, info)| {
            (
                dist.get(&info.junction_node).copied().unwrap_or(usize::MAX),
                *edge_idx,
            )
        });
        (triodes, all_triode_edges)
    }

    /// Detect push-pull triode pairs connected through center-tapped transformers.
    /// For the 670, push triodes connect to primary.a and pull triodes connect
    /// to primary.b of a CT transformer.
    ///
    /// Returns pairs of (push_triode_idx, pull_triode_idx, transformer_edge_idx, turns_ratio)
    /// where the indices refer to positions in the merged triodes list.
    pub(super) fn find_push_pull_triode_pairs(
        &self,
        triodes: &[(usize, TriodeInfo)],
        nonlinear_edge_indices: &[usize],
    ) -> (Vec<PushPullPairInfo>, HashSet<usize>) {
        // Find CT transformers (primary_type = CenterTap or PushPull).
        let mut ct_transformers: Vec<(usize, &TransformerConfig)> = Vec::new();
        for (edge_idx, e) in self.edges.iter().enumerate() {
            let comp = &self.components[e.comp_idx];
            if let Some(cfg) = comp.kind.transformer_config() {
                if matches!(
                    cfg.primary_type,
                    WindingType::CenterTap | WindingType::PushPull
                ) {
                    ct_transformers.push((edge_idx, cfg));
                }
            }
        }

        let mut pairs = Vec::new();
        let mut pp_transformer_edges: HashSet<usize> = HashSet::new();
        let mut used_triodes: HashSet<usize> = HashSet::new();

        for (xfmr_edge_idx, cfg) in &ct_transformers {
            let xfmr_edge = &self.edges[*xfmr_edge_idx];
            let primary_a_node = xfmr_edge.node_a;
            let primary_b_node = xfmr_edge.node_b;

            // BFS through passives only: exclude nonlinear edges (triodes, diodes,
            // JFETs, BJTs), active edges (op-amps, other transformers), and the
            // transformer edge itself.
            let mut exclude: Vec<usize> = nonlinear_edge_indices.to_vec();
            exclude.push(*xfmr_edge_idx);

            let nodes_near_a =
                self.bfs_through_passives(primary_a_node, &exclude, &self.active_edge_indices);
            let nodes_near_b =
                self.bfs_through_passives(primary_b_node, &exclude, &self.active_edge_indices);

            // Find triode groups whose plate is reachable from either transformer end.
            // In the 670, both transformer ends reach both triode plates through a
            // shared balance node (A_bal). We collect all reachable triode groups
            // and pair them — push/pull phase assignment is arbitrary (doesn't affect
            // the differential output, only inverts polarity).
            let mut candidates: Vec<usize> = Vec::new();
            for (i, (_, info)) in triodes.iter().enumerate() {
                if used_triodes.contains(&i) {
                    continue;
                }
                if nodes_near_a.contains(&info.plate_node)
                    || nodes_near_b.contains(&info.plate_node)
                {
                    candidates.push(i);
                }
            }

            // Need at least 2 triode groups for a push-pull pair.
            if candidates.len() >= 2 {
                let push = candidates[0];
                let pull = candidates[1];
                used_triodes.insert(push);
                used_triodes.insert(pull);
                pp_transformer_edges.insert(*xfmr_edge_idx);
                pairs.push(PushPullPairInfo {
                    push_triode_idx: push,
                    pull_triode_idx: pull,
                    transformer_edge_idx: *xfmr_edge_idx,
                    turns_ratio: cfg.turns_ratio,
                });
            }
        }

        (pairs, pp_transformer_edges)
    }

    /// Find Standard-type transformers that are functionally push-pull.
    ///
    /// A Standard-type transformer (like T_sc_out in the 670) is functionally
    /// push-pull if its primary nodes are reachable from both halves of a
    /// push-pull triode pair. These must be skipped during BFS just like
    /// explicit CT/PP transformers.
    pub(super) fn find_pp_driven_transformer_edges(
        &self,
        push_pull_pairs: &[PushPullPairInfo],
        triodes: &[(usize, TriodeInfo)],
        nonlinear_edge_indices: &[usize],
    ) -> HashSet<usize> {
        let mut pp_driven: HashSet<usize> = HashSet::new();

        // Collect plate nodes from all push-pull paired triodes.
        let mut pp_plate_nodes: Vec<(NodeId, NodeId)> = Vec::new(); // (push_plate, pull_plate)
        for pair in push_pull_pairs {
            let push_plate = triodes[pair.push_triode_idx].1.plate_node;
            let pull_plate = triodes[pair.pull_triode_idx].1.plate_node;
            pp_plate_nodes.push((push_plate, pull_plate));
        }

        // Find Standard-type transformer edges.
        let standard_xfmr_edges: Vec<usize> = self
            .edges
            .iter()
            .enumerate()
            .filter(|(_, e)| {
                self.components[e.comp_idx].kind.transformer_config()
                    .map_or(false, |cfg| matches!(cfg.primary_type, WindingType::Standard))
            })
            .map(|(idx, _)| idx)
            .collect();

        for &xfmr_idx in &standard_xfmr_edges {
            let xfmr_edge = &self.edges[xfmr_idx];
            let xfmr_nodes = [xfmr_edge.node_a, xfmr_edge.node_b];

            // For each push-pull pair, BFS from each plate node through passives
            // (excluding NL edges and the transformer itself).
            for &(push_plate, pull_plate) in &pp_plate_nodes {
                let mut exclude: Vec<usize> = nonlinear_edge_indices.to_vec();
                exclude.push(xfmr_idx);

                let push_reachable =
                    self.bfs_through_passives(push_plate, &exclude, &self.active_edge_indices);
                let pull_reachable =
                    self.bfs_through_passives(pull_plate, &exclude, &self.active_edge_indices);

                // If both push and pull plates can reach a transformer primary node,
                // this transformer is functionally push-pull.
                let push_reaches = xfmr_nodes.iter().any(|n| push_reachable.contains(n));
                let pull_reaches = xfmr_nodes.iter().any(|n| pull_reachable.contains(n));
                if push_reaches && pull_reaches {
                    pp_driven.insert(xfmr_idx);
                }
            }
        }

        pp_driven
    }

    /// BFS through passive edges only, excluding specified nonlinear and active edges.
    fn bfs_through_passives(
        &self,
        start: NodeId,
        exclude_indices: &[usize],
        active_indices: &[usize],
    ) -> HashSet<NodeId> {
        let mut reachable = HashSet::new();
        let mut queue = std::collections::VecDeque::new();
        reachable.insert(start);
        queue.push_back(start);

        while let Some(node) = queue.pop_front() {
            for (idx, e) in self.edges.iter().enumerate() {
                if exclude_indices.contains(&idx) || active_indices.contains(&idx) {
                    continue;
                }
                let neighbor = if e.node_a == node {
                    Some(e.node_b)
                } else if e.node_b == node {
                    Some(e.node_a)
                } else {
                    None
                };
                if let Some(n) = neighbor {
                    if reachable.insert(n) {
                        queue.push_back(n);
                    }
                }
            }
        }

        reachable
    }

    /// BFS through passive edges starting from `start`, returning **edge indices**.
    ///
    /// Boundary rules:
    /// - Excludes nonlinear and active edges (same as `bfs_through_passives`).
    /// - Stops at transformer edges (don't cross magnetic isolation).
    /// - Stops at `gnd_node` and `vcc_node`: the edge TO them is collected but
    ///   BFS does not continue THROUGH them (they are global hubs).
    /// - Named supply nodes (A_bal, etc.): when `include_supply_adjacent` is
    ///   false, edges to supply nodes are skipped entirely. When true, the edge
    ///   IS collected (it's a plate load or cathode bias path) but BFS does NOT
    ///   continue through the supply node (it's treated as a boundary).
    pub(super) fn bfs_passive_edges(
        &self,
        start: NodeId,
        nonlinear_indices: &[usize],
        active_indices: &[usize],
        include_supply_adjacent: bool,
        skip_out_node: bool,
        pp_transformer_edges: &HashSet<usize>,
        barrier_nodes: &HashSet<NodeId>,
    ) -> Vec<usize> {
        let mut visited_nodes: HashSet<NodeId> = HashSet::new();
        let mut collected_edges: Vec<usize> = Vec::new();
        let mut queue = std::collections::VecDeque::new();
        visited_nodes.insert(start);
        queue.push_back(start);

        while let Some(node) = queue.pop_front() {
            for (idx, e) in self.edges.iter().enumerate() {
                // Skip nonlinear and active edges.
                if nonlinear_indices.contains(&idx) || active_indices.contains(&idx) {
                    continue;
                }
                // Determine neighbor through this edge.
                let neighbor = if e.node_a == node {
                    Some(e.node_b)
                } else if e.node_b == node {
                    Some(e.node_a)
                } else {
                    None
                };
                let Some(n) = neighbor else { continue };
                // Skip push-pull transformer edges (magnetic isolation boundary).
                // Non-PP transformers are kept so they can be built into WDF subtrees.
                if self.components[e.comp_idx].kind.is_transformer() {
                    if pp_transformer_edges.contains(&idx) {
                        continue;
                    }
                }
                // Skip edges to output node (those become output attenuation).
                if skip_out_node && n == self.out_node {
                    continue;
                }
                // Barrier nodes: collect the edge but don't traverse further.
                // Used to stop BFS at active bridge nodes (opamp/transistor pins).
                if barrier_nodes.contains(&n) {
                    if !collected_edges.contains(&idx) {
                        collected_edges.push(idx);
                    }
                    continue;
                }
                // Named supply rail handling (supply_nodes excludes vcc and gnd —
                // those are standard WDF terminations handled below).
                if self.supply_nodes.contains(&n) {
                    if include_supply_adjacent {
                        // Collect this edge (plate load / cathode bias path) but
                        // don't BFS further — supply node is a boundary.
                        if visited_nodes.insert(n) {
                            collected_edges.push(idx);
                        }
                    }
                    continue;
                }
                // Ground and VCC are "star" nodes — multiple edges from
                // different circuit branches terminate there.  Always
                // collect edges to them (pots, cathode Rs, etc.) even
                // if gnd/vcc was already reached via another path.
                if n == self.gnd_node || n == self.vcc_node {
                    if !collected_edges.contains(&idx) {
                        collected_edges.push(idx);
                    }
                } else if visited_nodes.insert(n) {
                    collected_edges.push(idx);
                    queue.push_back(n);
                } else if !collected_edges.contains(&idx) {
                    // Parallel edge: same nodes, different component.
                    // Must collect (e.g., C4 and C5 share nodes in SCREAMER).
                    collected_edges.push(idx);
                }
            }
        }

        collected_edges
    }

    /// Find op-amp feedback loops and calculate closed-loop gain.
    ///
    /// Detects:
    /// - **Unity-gain buffers**: neg and out pins resolve to the same node
    /// - **Inverting amplifiers**: pos to ground, Rf from neg to out, Ri from input to neg
    /// - **Non-inverting amplifiers**: signal to pos, Rf from neg to out, Ri from neg to ground
    ///
    /// Returns a list of `OpAmpFeedbackInfo` ordered by topological distance
    /// from the input node.
    pub(super) fn find_opamp_feedback_loops(&self, pedal: &PedalDef) -> Vec<OpAmpFeedbackInfo> {
        // Build a pin → resolved node map from the netlist.
        // We need to re-resolve pins because CircuitGraph doesn't store the
        // full pin_ids map (only edges and resolved nodes).
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

        for name in &["in", "out", "gnd", "vcc"] {
            get_id(name, &mut uf);
        }
        // Also create nodes for named supply rails.
        for supply in &pedal.supplies {
            get_id(&supply.name, &mut uf);
        }
        for net in &pedal.nets {
            let from_id = get_id(&pin_key(&net.from), &mut uf);
            for to_pin in &net.to {
                let to_id = get_id(&pin_key(to_pin), &mut uf);
                uf.union(from_id, to_id);
            }
        }

        // Get resolved node IDs for special nodes
        let _in_node_resolved = uf.find(*pin_ids.get("in").unwrap_or(&0));
        let gnd_node_resolved = uf.find(*pin_ids.get("gnd").unwrap_or(&0));

        // Build set of AC-ground-equivalent nodes:
        // 1. Named supply nodes (V-, B+, etc.)
        // 2. Nodes bypassed to ground through a large capacitor (virtual ground bias points)
        let mut ac_ground_nodes: HashSet<usize> = HashSet::new();

        for supply in &pedal.supplies {
            if let Some(&raw_id) = pin_ids.get(&supply.name) {
                let resolved = uf.find(raw_id);
                if resolved != gnd_node_resolved {
                    ac_ground_nodes.insert(resolved);
                }
            }
        }

        for comp in &pedal.components {
            if let Some(cap) = comp.kind.as_any().downcast_ref::<Capacitor>() {
                if cap.config.value < 10e-6 {
                    continue; // Skip caps < 10µF — only large bypass caps indicate virtual ground
                }
                let pa = format!("{}.a", comp.id);
                let pb = format!("{}.b", comp.id);
                if let (Some(&a_id), Some(&b_id)) = (pin_ids.get(&pa), pin_ids.get(&pb)) {
                    let a_node = uf.find(a_id);
                    let b_node = uf.find(b_id);
                    if a_node == gnd_node_resolved {
                        ac_ground_nodes.insert(b_node);
                    } else if b_node == gnd_node_resolved {
                        ac_ground_nodes.insert(a_node);
                    }
                }
            }
        }

        let is_ac_ground = |node: usize| -> bool {
            node == gnd_node_resolved || ac_ground_nodes.contains(&node)
        };

        // Build a map of component → (pin_a_node, pin_b_node, resistance, is_pot, max_r)
        // for resistors and pots. Pots use default position (0.5) for initial gain calc.
        struct ResistorInfo {
            id: String,
            node_a: usize,
            node_b: usize,
            resistance: f64,
            is_pot: bool,
            max_r: f64, // For pots: max resistance; for resistors: same as resistance
        }
        let mut resistor_nodes: Vec<ResistorInfo> = Vec::new();
        for comp in &pedal.components {
            let (resistance, is_pot, max_r) = if let Some(pot) = comp.kind.as_any().downcast_ref::<Potentiometer>() {
                // Use default position (0.5) for initial gain calculation
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

                    // 3-terminal pot: emit two half-segments (a→w, w→b)
                    // matching the synthetic __aw/__wb edge IDs from graph construction
                    if is_pot {
                        let wiper_key = format!("{}.w", comp.id);
                        let wiper_key_long = format!("{}.wiper", comp.id);
                        let wiper_id = pin_ids.get(&wiper_key).or_else(|| pin_ids.get(&wiper_key_long));
                        if let Some(&w_id) = wiper_id {
                            let w_node = uf.find(w_id);
                            resistor_nodes.push(ResistorInfo {
                                id: format!("{}__aw", comp.id),
                                node_a: a_node,
                                node_b: w_node,
                                resistance: max_r * 0.5,
                                is_pot: true,
                                max_r,
                            });
                            resistor_nodes.push(ResistorInfo {
                                id: format!("{}__wb", comp.id),
                                node_a: w_node,
                                node_b: b_node,
                                resistance: max_r * 0.5,
                                is_pot: true,
                                max_r,
                            });
                            continue;
                        }
                    }

                    // 2-terminal (resistor or pot without wiper): existing behavior
                    resistor_nodes.push(ResistorInfo {
                        id: comp.id.clone(),
                        node_a: a_node,
                        node_b: b_node,
                        resistance: r,
                        is_pot,
                        max_r,
                    });
                }
            }
        }

        // Helper: find the input resistor connected to neg_node
        // For cascaded op-amps, Ri may connect to a previous stage's output, not the global input
        let find_input_resistor =
            |neg_node: usize, out_node: usize, gnd_node: usize| -> Option<f64> {
                for info in &resistor_nodes {
                    // Skip the feedback resistor (connects neg to out)
                    if (info.node_a == neg_node && info.node_b == out_node)
                        || (info.node_a == out_node && info.node_b == neg_node)
                    {
                        continue;
                    }
                    // Skip resistors to ground (those are Ri for non-inverting topology)
                    if (info.node_a == neg_node && info.node_b == gnd_node)
                        || (info.node_a == gnd_node && info.node_b == neg_node)
                    {
                        continue;
                    }
                    // Find any other resistor connected to neg_node - this is Ri
                    if info.node_a == neg_node || info.node_b == neg_node {
                        return Some(info.resistance);
                    }
                }
                None
            };

        // Helper: find resistive path between two nodes
        // Handles series resistances (sum) and parallel paths (1/R = 1/R1 + 1/R2)
        // Returns (effective_resistance, component_ids, pot_info)
        // pot_info: Option<(pot_comp_id, pot_max_r, fixed_series_r, parallel_fixed_r)>
        // The return type includes parallel_fixed_r for pots in parallel with fixed resistors
        let find_resistive_path =
            |start: usize,
             end: usize|
             -> Option<(f64, Vec<String>, Option<(String, f64, f64, Option<f64>)>)> {
                if start == end {
                    return None; // Same node, no resistance
                }

                // Build adjacency for resistor-only edges
                // Store (next_node, resistance, comp_id, is_pot, max_r)
                let mut resistor_adj: HashMap<usize, Vec<(usize, f64, String, bool, f64)>> =
                    HashMap::new();
                for info in &resistor_nodes {
                    resistor_adj.entry(info.node_a).or_default().push((
                        info.node_b,
                        info.resistance,
                        info.id.clone(),
                        info.is_pot,
                        info.max_r,
                    ));
                    resistor_adj.entry(info.node_b).or_default().push((
                        info.node_a,
                        info.resistance,
                        info.id.clone(),
                        info.is_pot,
                        info.max_r,
                    ));
                }

                // Find all simple paths from start to end using DFS
                // Track: (resistance, comp_ids, pot_info: Option<(id, max_r, fixed_series_r)>)
                // Note: pot_info here is 3 elements; we add parallel_fixed_r at the end
                #[derive(Clone)]
                struct PathState {
                    node: usize,
                    path_r: f64,
                    path_comps: Vec<String>,
                    pot_info: Option<(String, f64)>, // (pot_id, pot_max_r)
                    fixed_r: f64,                    // Sum of non-pot resistors in path
                    visited: HashSet<usize>,
                }

                // Intermediate paths have 3-element pot_info (without parallel_fixed_r)
                let mut all_paths: Vec<(f64, Vec<String>, Option<(String, f64, f64)>)> = Vec::new();
                let mut stack: Vec<PathState> = Vec::new();
                let mut visited_start = HashSet::new();
                visited_start.insert(start);
                stack.push(PathState {
                    node: start,
                    path_r: 0.0,
                    path_comps: Vec::new(),
                    pot_info: None,
                    fixed_r: 0.0,
                    visited: visited_start,
                });

                while let Some(state) = stack.pop() {
                    if state.node == end {
                        let pot_info = state.pot_info.map(|(id, max_r)| (id, max_r, state.fixed_r));
                        all_paths.push((state.path_r, state.path_comps, pot_info));
                        continue;
                    }

                    if let Some(neighbors) = resistor_adj.get(&state.node) {
                        for (next_node, r, comp_id, is_pot, max_r) in neighbors {
                            if !state.visited.contains(next_node) {
                                let mut new_visited = state.visited.clone();
                                new_visited.insert(*next_node);
                                let mut new_comps = state.path_comps.clone();
                                new_comps.push(comp_id.clone());

                                // Track pot info and fixed series resistance
                                let (new_pot_info, new_fixed_r) = if *is_pot {
                                    // This is a pot - record it (only track first pot in path)
                                    let pot_info =
                                        state.pot_info.clone().or(Some((comp_id.clone(), *max_r)));
                                    (pot_info, state.fixed_r)
                                } else {
                                    // Fixed resistor - add to fixed_r
                                    (state.pot_info.clone(), state.fixed_r + r)
                                };

                                // Series: add resistances
                                stack.push(PathState {
                                    node: *next_node,
                                    path_r: state.path_r + r,
                                    path_comps: new_comps,
                                    pot_info: new_pot_info,
                                    fixed_r: new_fixed_r,
                                    visited: new_visited,
                                });
                            }
                        }
                    }
                }

                if all_paths.is_empty() {
                    return None;
                }

                // If single path, use it directly (no parallel fixed resistance)
                if all_paths.len() == 1 {
                    let (r, comps, pot_info) = all_paths.into_iter().next().unwrap();
                    // Convert pot_info to 4-element tuple with None for parallel_fixed_r
                    let pot_info_4 =
                        pot_info.map(|(id, max_r, fixed_series)| (id, max_r, fixed_series, None));
                    return Some((r, comps, pot_info_4));
                }

                // Parallel paths: 1/R_total = 1/R1 + 1/R2 + ...
                // For pots in parallel paths, track pot info AND the parallel fixed resistance
                // (the resistance of paths that don't include the pot)
                let mut conductance_sum = 0.0;
                let mut all_comps: Vec<String> = Vec::new();
                let mut first_pot_info: Option<(String, f64, f64)> = None;
                let mut fixed_paths_conductance = 0.0; // Conductance of paths without the pot

                for (r, comps, pot_info) in &all_paths {
                    if *r > 0.0 {
                        conductance_sum += 1.0 / r;

                        if pot_info.is_some() {
                            // This path has a pot
                            if first_pot_info.is_none() {
                                first_pot_info = pot_info.clone();
                            }
                        } else {
                            // This is a fixed path (no pot) - track its conductance
                            fixed_paths_conductance += 1.0 / r;
                        }
                    }
                    for c in comps {
                        if !all_comps.contains(c) {
                            all_comps.push(c.clone());
                        }
                    }
                }

                // If there's a pot AND fixed parallel paths, include the parallel fixed R
                // in the pot info as a 4th element (parallel_fixed_r)
                let final_pot_info = if let Some((id, max_r, fixed_series)) = first_pot_info {
                    if fixed_paths_conductance > 0.0 {
                        // There are fixed paths in parallel with the pot
                        // parallel_fixed_r = 1 / fixed_paths_conductance
                        let parallel_fixed_r = 1.0 / fixed_paths_conductance;
                        Some((id, max_r, fixed_series, Some(parallel_fixed_r)))
                    } else {
                        Some((id, max_r, fixed_series, None))
                    }
                } else {
                    None
                };

                if conductance_sum > 0.0 {
                    Some((1.0 / conductance_sum, all_comps, final_pot_info))
                } else {
                    None
                }
            };

        // Build a map of diodes: (node_a, node_b) -> DiodeType
        // Use the SAME UnionFind and pin_ids we just built to compute node IDs.
        let mut feedback_diodes: HashMap<(usize, usize), DiodeType> = HashMap::new();
        for comp in &pedal.components {
            let diode_type = match comp.kind.diode_type() {
                Some(dt) => dt,
                None => continue,
            };

            let key_a = format!("{}.a", comp.id);
            let key_b = format!("{}.b", comp.id);

            if let (Some(&id_a), Some(&id_b)) = (pin_ids.get(&key_a), pin_ids.get(&key_b)) {
                let na = uf.find(id_a);
                let nb = uf.find(id_b);
                // Store both orderings so lookup is order-independent
                feedback_diodes.insert((na, nb), diode_type);
                feedback_diodes.insert((nb, na), diode_type);
            }
        }

        // Passive adjacency map: node → [(neighbor, comp_id)]
        // Includes resistors, capacitors, inductors, and pots (all 2-terminal passive edges).
        // Used by BFS to collect ALL passive components in an op-amp feedback network.
        let mut passive_adj: HashMap<usize, Vec<(usize, String)>> = HashMap::new();
        for comp in &pedal.components {
            let is_passive = comp.kind.as_any().downcast_ref::<Resistor>().is_some()
                || comp.kind.as_any().downcast_ref::<Capacitor>().is_some()
                || comp.kind.as_any().downcast_ref::<Inductor>().is_some();
            if is_passive {
                let pa = format!("{}.a", comp.id);
                let pb = format!("{}.b", comp.id);
                if let (Some(&a_id), Some(&b_id)) = (pin_ids.get(&pa), pin_ids.get(&pb)) {
                    let a_node = uf.find(a_id);
                    let b_node = uf.find(b_id);
                    passive_adj.entry(a_node).or_default().push((b_node, comp.id.clone()));
                    passive_adj.entry(b_node).or_default().push((a_node, comp.id.clone()));
                }
            }
            if let Some(_pot) = comp.kind.as_any().downcast_ref::<Potentiometer>() {
                // 3-terminal pot: edges a-w and w-b
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
                        passive_adj.entry(a_node).or_default().push((w_node, aw_id.clone()));
                        passive_adj.entry(w_node).or_default().push((a_node, aw_id));
                    }
                    if let Some(&b_id) = pin_ids.get(&pb) {
                        let b_node = uf.find(b_id);
                        let w_node = uf.find(w_id);
                        let wb_id = format!("{}__wb", comp.id);
                        passive_adj.entry(b_node).or_default().push((w_node, wb_id.clone()));
                        passive_adj.entry(w_node).or_default().push((b_node, wb_id));
                    }
                } else {
                    // 2-terminal pot (no wiper)
                    if let (Some(&a_id), Some(&b_id)) = (pin_ids.get(&pa), pin_ids.get(&pb)) {
                        let a_node = uf.find(a_id);
                        let b_node = uf.find(b_id);
                        passive_adj.entry(a_node).or_default().push((b_node, comp.id.clone()));
                        passive_adj.entry(b_node).or_default().push((a_node, comp.id.clone()));
                    }
                }
            }
        }

        // Helper: find diodes between two nodes (for feedback diode detection).
        // Checks direct match first, then 1-hop through passive adjacency.
        // Covers Klon topology: neg → R6 → D1/D2 → out.
        let find_feedback_diode = |node_a: usize, node_b: usize| -> Option<DiodeType> {
            // Direct: diode spans (neg, out)
            if let Some(&dt) = feedback_diodes.get(&(node_a, node_b)) {
                return Some(dt);
            }
            // 1-hop from node_a: diode at (intermediate, node_b)
            if let Some(neighbors) = passive_adj.get(&node_a) {
                for &(mid, _) in neighbors {
                    if let Some(&dt) = feedback_diodes.get(&(mid, node_b)) {
                        return Some(dt);
                    }
                }
            }
            // 1-hop from node_b: diode at (node_a, intermediate)
            if let Some(neighbors) = passive_adj.get(&node_b) {
                for &(mid, _) in neighbors {
                    if let Some(&dt) = feedback_diodes.get(&(node_a, mid)) {
                        return Some(dt);
                    }
                }
            }
            None
        };

        // Collect all op-amp pin nodes as BFS barriers (prevents crossing stage boundaries).
        let mut opamp_pin_nodes: HashSet<usize> = HashSet::new();
        for c in &pedal.components {
            if c.kind.op_amp_type().is_some() {
                for suffix in &["neg", "out", "pos"] {
                    let key = format!("{}.{}", c.id, suffix);
                    if let Some(&id) = pin_ids.get(&key) {
                        opamp_pin_nodes.insert(uf.find(id));
                    }
                }
            }
        }

        // BFS from `start` through passive edges; collect IDs of all components on
        // paths that reach `end`. Barriers prevent the BFS from escaping the local
        // feedback network into the signal path or power supply.
        let collect_feedback_comps = |start: usize, end: usize, extra_barriers: &HashSet<usize>| -> Vec<String> {
            use std::collections::VecDeque;
            // Barrier nodes: ground, AC ground, op-amp pins (except start/end),
            // global in/out nodes.
            let mut barriers: HashSet<usize> = HashSet::new();
            barriers.insert(gnd_node_resolved);
            for &ag in &ac_ground_nodes {
                barriers.insert(ag);
            }
            for &op in &opamp_pin_nodes {
                barriers.insert(op);
            }
            for &eb in extra_barriers {
                barriers.insert(eb);
            }
            // Start and end are NOT barriers (we must traverse through them).
            barriers.remove(&start);
            barriers.remove(&end);

            // BFS forward from start, stopping at barriers and not expanding past end.
            let mut visited: HashSet<usize> = HashSet::new();
            let mut parent_edges: HashMap<usize, Vec<(usize, String)>> = HashMap::new();
            let mut queue = VecDeque::new();
            visited.insert(start);
            queue.push_back(start);
            while let Some(node) = queue.pop_front() {
                if let Some(neighbors) = passive_adj.get(&node) {
                    for (next, comp_id) in neighbors {
                        parent_edges.entry(*next).or_default().push((node, comp_id.clone()));
                        if visited.insert(*next) {
                            // Don't expand past end node or through barrier nodes,
                            // but DO record the edge to collect the component.
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
            // BFS backward from end to collect all components on paths from start.
            // Don't expand past start — otherwise we'd follow input-path edges
            // (e.g., R_in, C_in) that happen to connect at neg_node.
            let mut result_comps: HashSet<String> = HashSet::new();
            let mut back_visited: HashSet<usize> = HashSet::new();
            let mut back_queue = VecDeque::new();
            back_visited.insert(end);
            back_queue.push_back(end);
            while let Some(node) = back_queue.pop_front() {
                if let Some(parents) = parent_edges.get(&node) {
                    for (prev, comp_id) in parents {
                        result_comps.insert(comp_id.clone());
                        if back_visited.insert(*prev) {
                            // Don't expand past start node — edges beyond start
                            // are input/output path components, not feedback.
                            if *prev != start {
                                back_queue.push_back(*prev);
                            }
                        }
                    }
                }
            }
            result_comps.into_iter().collect()
        };

        // BFS distances for ordering.
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

        let mut results: Vec<OpAmpFeedbackInfo> = Vec::new();

        for comp in &pedal.components {
            let opamp_type = match comp.kind.op_amp_type() {
                Some(ot) if !ot.is_ota() => ot,
                _ => continue,
            };

            // Check for 3-pin form (pos/neg/out).
            let neg_key = format!("{}.neg", comp.id);
            let out_key = format!("{}.out", comp.id);
            let pos_key = format!("{}.pos", comp.id);

            let neg_id = pin_ids.get(&neg_key).copied();
            let out_id = pin_ids.get(&out_key).copied();
            let pos_id = pin_ids.get(&pos_key).copied();

            if let (Some(neg_raw), Some(out_raw), Some(pos_raw)) = (neg_id, out_id, pos_id) {
                let neg_node = uf.find(neg_raw);
                let out_node = uf.find(out_raw);
                let pos_node = uf.find(pos_raw);

                // Unity-gain buffer: neg and out resolve to the same node.
                if neg_node == out_node {
                    results.push(OpAmpFeedbackInfo {
                        comp_id: comp.id.clone(),
                        opamp_type,
                        feedback_kind: OpAmpFeedbackKind::UnityGain,
                        neg_node,
                        pos_node,
                        out_node,
                        feedback_comp_ids: Vec::new(),
                    });
                    continue;
                }

                // Look for feedback resistor path (Rf: neg to out)
                // Use resistive path finding to handle series/parallel combinations
                // Returns (rf_value, component_ids, pot_info)
                if let Some((rf, rf_comps, rf_pot)) = find_resistive_path(neg_node, out_node) {
                    // BFS through ALL passive edges (R, C, L, pot) from neg to out
                    // to collect the complete feedback network for edge exclusion.
                    let no_extra = HashSet::new();
                    let all_fb_comps = collect_feedback_comps(neg_node, out_node, &no_extra);

                    // Check for inverting topology: pos connected to ground (or AC ground)
                    if is_ac_ground(pos_node) {
                        // Inverting: look for Ri connected to neg (from any input source)
                        // For cascaded op-amps, Ri may connect to a previous stage's output
                        if let Some(ri) = find_input_resistor(neg_node, out_node, gnd_node_resolved)
                        {
                            // Check for feedback diodes (Tube Screamer style soft clipping)
                            let feedback_diode = find_feedback_diode(neg_node, out_node);
                            results.push(OpAmpFeedbackInfo {
                                comp_id: comp.id.clone(),
                                opamp_type,
                                feedback_kind: OpAmpFeedbackKind::Inverting {
                                    rf,
                                    ri,
                                    feedback_diode,
                                    rf_pot,
                                },
                                neg_node,
                                pos_node,
                                out_node,
                                feedback_comp_ids: all_fb_comps,
                            });
                            continue;
                        }
                    }

                    // Check for non-inverting topology: pos connected to input (or signal path)
                    // Non-inverting: look for Ri path from neg to ground (or AC ground)
                    let ni_gnd_node = if find_resistive_path(neg_node, gnd_node_resolved).is_some() {
                        Some(gnd_node_resolved)
                    } else {
                        ac_ground_nodes.iter().find(|&&ag| find_resistive_path(neg_node, ag).is_some()).copied()
                    };
                    if let Some(gnd_target) = ni_gnd_node {
                        let (ri, _ri_comps, ri_pot) = find_resistive_path(neg_node, gnd_target).unwrap();
                        // Collect ground-leg passive components too
                        let gnd_leg_comps = collect_feedback_comps(neg_node, gnd_target, &no_extra);
                        let mut fb_comps = all_fb_comps.clone();
                        for c in gnd_leg_comps {
                            if !fb_comps.contains(&c) {
                                fb_comps.push(c);
                            }
                        }
                        results.push(OpAmpFeedbackInfo {
                            comp_id: comp.id.clone(),
                            opamp_type,
                            feedback_kind: OpAmpFeedbackKind::NonInverting { rf, ri, rf_pot, ri_pot },
                            neg_node,
                            pos_node,
                            out_node,
                            feedback_comp_ids: fb_comps,
                        });
                        continue;
                    }

                    // Check for AllpassJfet: JFET drain at neg with R||C feedback (Phase 90).
                    let mut cf_val = None;
                    for c in &pedal.components {
                        if let Some(cap) = c.kind.as_any().downcast_ref::<Capacitor>() {
                            let pa = format!("{}.a", c.id);
                            let pb = format!("{}.b", c.id);
                            if let (Some(&a), Some(&b)) =
                                (pin_ids.get(&pa), pin_ids.get(&pb))
                            {
                                let an = uf.find(a);
                                let bn = uf.find(b);
                                if (an == neg_node && bn == out_node)
                                    || (an == out_node && bn == neg_node)
                                {
                                    cf_val = Some(cap.config.value);
                                    break;
                                }
                            }
                        }
                    }
                    let mut jfet_comp_id = None;
                    for c in &pedal.components {
                        if c.kind.is_jfet() {
                            let dk = format!("{}.drain", c.id);
                            if let Some(&did) = pin_ids.get(&dk) {
                                if uf.find(did) == neg_node {
                                    jfet_comp_id = Some(c.id.clone());
                                    break;
                                }
                            }
                        }
                    }
                    if let (Some(cf), Some(jfet_id)) = (cf_val, jfet_comp_id) {
                        results.push(OpAmpFeedbackInfo {
                            comp_id: comp.id.clone(),
                            opamp_type,
                            feedback_kind: OpAmpFeedbackKind::AllpassJfet {
                                rf,
                                cf,
                                jfet_id,
                            },
                            neg_node,
                            pos_node,
                            out_node,
                            feedback_comp_ids: all_fb_comps.clone(),
                        });
                        continue;
                    }

                    // Rf found but no Ri - could be unity-gain buffer through resistor
                    // or more complex topology. Skip for now.
                }
            }
        }

        // Sort by topological distance from input (using pos_node as reference
        // since that's where the signal enters the op-amp).
        results.sort_by_key(|info| dist.get(&info.pos_node).copied().unwrap_or(usize::MAX));

        results
    }
}

/// Push-pull pair detected via center-tapped transformer.
pub(super) struct PushPullPairInfo {
    /// Index into the merged triodes list for the push half.
    pub(super) push_triode_idx: usize,
    /// Index into the merged triodes list for the pull half.
    pub(super) pull_triode_idx: usize,
    /// Edge index of the CT transformer connecting them.
    pub(super) transformer_edge_idx: usize,
    /// Turns ratio of the output transformer (primary:secondary).
    pub(super) turns_ratio: f64,
}

#[allow(dead_code)]
pub(super) struct TriodeInfo {
    pub(super) model_name: String,
    /// Plate node - connected to plate load resistor and output
    pub(super) plate_node: NodeId,
    /// Cathode node - connected to cathode resistor and bypass cap
    pub(super) cathode_node: NodeId,
    /// Legacy junction_node - kept for compatibility, equals cathode_node
    pub(super) junction_node: NodeId,
    pub(super) ground_node: NodeId,
    /// Number of parallel tubes sharing the same plate and cathode nodes.
    /// Default is 1. When > 1, the tube model scales plate current by N.
    pub(super) parallel_count: usize,
    /// True if this is a variable-mu triode (Raffensperger model).
    pub(super) is_vari_mu: bool,
}

/// Op-amp feedback loop information.
///
/// Detected when an op-amp's `neg` and `out` pins resolve to the same node
/// (unity-gain buffer / voltage follower), or when they connect through a
/// feedback resistor network.
pub(super) struct OpAmpFeedbackInfo {
    /// Component ID of the op-amp (e.g., "U1").
    pub(super) comp_id: String,
    /// Op-amp type for model selection.
    pub(super) opamp_type: OpAmpType,
    /// The kind of feedback topology detected.
    pub(super) feedback_kind: OpAmpFeedbackKind,
    /// Node where the inverting input meets the passive network.
    /// For unity-gain buffers, this is the output/neg node.
    #[allow(dead_code)]
    pub(super) neg_node: NodeId,
    /// Non-inverting input node (signal reference or bias).
    pub(super) pos_node: NodeId,
    /// Op-amp output node (resolved graph NodeId).
    pub(super) out_node: NodeId,
    /// Component IDs of resistors/pots in the feedback path (Rf and Ri).
    /// Used to exclude these edges from MultiNl passive BFS.
    pub(super) feedback_comp_ids: Vec<String>,
}

/// The feedback topology of an op-amp.
#[derive(Debug, Clone)]
pub(super) enum OpAmpFeedbackKind {
    /// Direct connection: neg tied to out (voltage follower).
    /// Closed-loop gain = 1.0.
    UnityGain,
    /// Inverting amplifier: pos tied to ground, neg connected through Ri to input
    /// and through Rf to output. Closed-loop gain = Rf/Ri.
    Inverting {
        /// Feedback resistor value (neg to out)
        rf: f64,
        /// Input resistor value (input to neg)
        ri: f64,
        /// Diode type in feedback loop (if any) for soft clipping.
        /// Tube Screamer style: diodes in parallel with Rf create soft clipping.
        feedback_diode: Option<DiodeType>,
        /// Potentiometer info for runtime gain modulation.
        /// (comp_id, max_resistance, fixed_series_resistance, parallel_fixed_resistance)
        /// parallel_fixed_r is the resistance of paths in parallel with the pot (e.g., TS R4)
        rf_pot: Option<(String, f64, f64, Option<f64>)>,
    },
    /// Non-inverting amplifier: pos connected to input, neg connected through Ri
    /// to ground and through Rf to output. Closed-loop gain = 1 + Rf/Ri.
    NonInverting {
        /// Feedback resistor value (neg to out)
        rf: f64,
        /// Ground resistor value (neg to gnd)
        ri: f64,
        /// Potentiometer info for Rf path (runtime gain modulation).
        /// (comp_id, max_resistance, fixed_series_resistance, parallel_fixed_resistance)
        rf_pot: Option<(String, f64, f64, Option<f64>)>,
        /// Potentiometer info for Ri path (ground leg, e.g. Tumnus Gain).
        /// (comp_id, max_resistance, fixed_series_resistance, parallel_fixed_resistance)
        ri_pot: Option<(String, f64, f64, Option<f64>)>,
    },
    /// JFET drain at neg with R||C feedback to out (Phase 90 inverting all-pass).
    /// Gain = -Z_fb/Z_in where Z_in = R_ap + R_jfet and Z_fb = Rf || Cf.
    AllpassJfet {
        /// Feedback resistor value (neg to out)
        rf: f64,
        /// Feedback capacitor value (neg to out)
        cf: f64,
        /// Component ID of the JFET whose drain connects to neg
        jfet_id: String,
    },
}

// ═══════════════════════════════════════════════════════════════════════════
// Graph-based SP reduction (builds DynNode directly)
// ═══════════════════════════════════════════════════════════════════════════

/// Extra edge for graph_reduce — pre-built DynNode for synthetic components
/// (voltage sources, virtual edges, cathode bias sources).
pub(super) struct ExtraEdge {
    pub node_a: NodeId,
    pub node_b: NodeId,
    pub tree: DynNode,
}

/// Internal working edge for graph_reduce.
struct WdfEdge {
    node_a: NodeId,
    node_b: NodeId,
    tree: DynNode,
}

/// Eliminate degree-1 non-terminal nodes by redirecting their single edge
/// to terminals[0]. Removes resulting self-loops.
fn eliminate_dead_ends(edges: &mut Vec<WdfEdge>, terminals: &[NodeId]) -> bool {
    let mut any_changed = false;
    loop {
        let mut changed = false;
        let mut degree: HashMap<NodeId, Vec<usize>> = HashMap::new();
        for (idx, e) in edges.iter().enumerate() {
            degree.entry(e.node_a).or_default().push(idx);
            degree.entry(e.node_b).or_default().push(idx);
        }
        let mut sorted: Vec<_> = degree.keys().copied().collect();
        sorted.sort();
        for node in sorted {
            let idxs = &degree[&node];
            if terminals.contains(&node) || idxs.len() != 1 {
                continue;
            }
            let eidx = idxs[0];
            if edges[eidx].node_a == node {
                edges[eidx].node_a = terminals[0];
            } else {
                edges[eidx].node_b = terminals[0];
            }
            if edges[eidx].node_a == edges[eidx].node_b {
                edges.remove(eidx);
            }
            changed = true;
            any_changed = true;
            break;
        }
        if !changed {
            break;
        }
    }
    any_changed
}

/// Reduce circuit edges to a single DynNode WDF tree.
///
/// Builds DynNode directly during reduction (no SpTree intermediate).
/// Dead-end elimination runs every iteration, fixing the push_pull bug
/// when callers include transformer pin nodes in `terminals`.
///
/// - `edge_indices` — passive edge indices from graph
/// - `extra_edges` — synthetic DynNodes (VS, virtual edges, supply sources)
/// - `terminals` — protected nodes (NL junction, VS source, transformer pins)
/// - `leaf_overrides` — comp_idx → custom DynNode (CathodeBiasSource for push-pull)
/// - `remap` — node canonicalization (supply → gnd for AC equivalence)
/// - `output_node` — if set, track which leaf component connects to this node
///   during series reduction. Returns its comp_id so the stage can extract
///   voltage at the output node after the WDF down-sweep.
pub(super) fn graph_reduce(
    edge_indices: &[usize],
    extra_edges: &[ExtraEdge],
    terminals: &[NodeId],
    graph: &CircuitGraph,
    sample_rate: f64,
    leaf_overrides: &HashMap<usize, DynNode>,
    remap: impl Fn(NodeId) -> NodeId,
    output_node: Option<NodeId>,
) -> Result<(DynNode, Option<String>), String> {
    // 1. Build WdfEdge list from graph edges.
    let mut edges: Vec<WdfEdge> = Vec::new();

    for &eidx in edge_indices {
        let e = &graph.edges[eidx];
        let na = remap(e.node_a);
        let nb = remap(e.node_b);
        // Skip self-loops (both endpoints mapped to same node).
        if na == nb {
            continue;
        }

        let tree = if let Some(override_node) = leaf_overrides.get(&e.comp_idx) {
            override_node.clone()
        } else {
            make_leaf(
                e.comp_idx,
                &graph.components[e.comp_idx],
                graph.fork_paths.get(&e.comp_idx),
                sample_rate,
            )
        };

        edges.push(WdfEdge {
            node_a: na,
            node_b: nb,
            tree,
        });
    }

    // 2. Add extra edges (VS, virtual edges, etc.).
    for extra in extra_edges {
        let na = remap(extra.node_a);
        let nb = remap(extra.node_b);
        if na == nb {
            continue;
        }
        edges.push(WdfEdge {
            node_a: na,
            node_b: nb,
            tree: extra.tree.clone(),
        });
    }

    // Pre-process: eliminate dead-end nodes once at the start
    // (same as old sp_reduce — NOT per-iteration, which incorrectly
    // reduces non-SP circuits like the RAT D1 diode stage).
    eliminate_dead_ends(&mut edges, terminals);

    // 3. Main reduction loop.
    //
    // Output node tracking: when a series reduction collapses output_node,
    // we identify the ground-side leaf component. Its comp_id is the output
    // probe — voltage at this leaf after the WDF down-sweep equals V_out.
    let output_node = output_node.map(|n| remap(n));
    let mut output_probe_comp_id: Option<String> = None;
    // Track which edge (by vec index) contains the output probe subtree.
    // Updated when edges are merged so it follows the probe through reductions.
    let mut output_probe_edge: Option<usize> = None;

    loop {
        if edges.is_empty() {
            return Err("empty network".into());
        }
        if edges.len() == 1 {
            return Ok((edges.remove(0).tree, output_probe_comp_id));
        }

        let mut changed = false;

        // b. Parallel reduction: edges with same endpoints.
        'par: for i in 0..edges.len() {
            for j in (i + 1)..edges.len() {
                let same = (edges[i].node_a == edges[j].node_a
                    && edges[i].node_b == edges[j].node_b)
                    || (edges[i].node_a == edges[j].node_b
                        && edges[i].node_b == edges[j].node_a);
                if same {
                    // Track output probe edge through parallel merge
                    let probe_in_j = output_probe_edge == Some(j);
                    let probe_in_i = output_probe_edge == Some(i);
                    let WdfEdge { tree: tree_j, .. } = edges.remove(j);
                    // After remove(j), indices >= j shift down
                    if let Some(ref mut ope) = output_probe_edge {
                        if *ope > j { *ope -= 1; }
                    }
                    let tree_i = std::mem::replace(
                        &mut edges[i].tree,
                        DynNode::Resistor { rp: 1.0, last_a: 0.0 },
                    );
                    let r1 = tree_i.port_resistance();
                    let r2 = tree_j.port_resistance();
                    let rp = r1 * r2 / (r1 + r2);
                    edges[i].tree = DynNode::Parallel {
                        left: Box::new(tree_i),
                        right: Box::new(tree_j),
                        rp,
                        gamma: r2 / (r1 + r2),
                        b1: 0.0,
                        b2: 0.0,
                    };
                    // Output probe stays in edges[i] (merged result)
                    if probe_in_i || probe_in_j {
                        output_probe_edge = Some(i);
                    }
                    changed = true;
                    break 'par;
                }
            }
        }
        if changed {
            continue;
        }

        // c. Series reduction: non-terminal nodes with degree 2.
        let mut degree: HashMap<NodeId, Vec<usize>> = HashMap::new();
        for (idx, e) in edges.iter().enumerate() {
            degree.entry(e.node_a).or_default().push(idx);
            degree.entry(e.node_b).or_default().push(idx);
        }
        let mut sorted_nodes: Vec<_> = degree.keys().copied().collect();
        sorted_nodes.sort();
        for node in sorted_nodes {
            let idxs = &degree[&node];
            if terminals.contains(&node) || idxs.len() != 2 {
                continue;
            }
            let i1 = idxs[0];
            let i2 = idxs[1];
            let other1 = if edges[i1].node_a == node {
                edges[i1].node_b
            } else {
                edges[i1].node_a
            };
            let other2 = if edges[i2].node_a == node {
                edges[i2].node_b
            } else {
                edges[i2].node_a
            };

            // Output node tracking: when collapsing output_node, identify
            // the ground-side leaf. Its voltage = V_out after the down-sweep.
            if output_node == Some(node) && output_probe_comp_id.is_none() {
                // Determine which edge connects to ground (the "ground side").
                // The ground-side edge's leaf comp_id is the output probe.
                let gnd_side_idx = if terminals.contains(&other2)
                    || other2 == graph.gnd_node
                {
                    i2
                } else {
                    i1
                };
                output_probe_comp_id = edges[gnd_side_idx].tree.leaf_comp_id();
            }

            let probe_in_lo = output_probe_edge == Some(if i1 < i2 { i1 } else { i2 });
            let probe_in_hi = output_probe_edge == Some(if i1 < i2 { i2 } else { i1 });

            let (lo, hi) = if i1 < i2 { (i1, i2) } else { (i2, i1) };
            let WdfEdge { tree: tree_hi, .. } = edges.remove(hi);
            let WdfEdge { tree: tree_lo, .. } = edges.remove(lo);
            let r1 = tree_lo.port_resistance();
            let r2 = tree_hi.port_resistance();
            let rp = r1 + r2;
            let new_idx = edges.len();
            edges.push(WdfEdge {
                node_a: other1,
                node_b: other2,
                tree: DynNode::Series {
                    left: Box::new(tree_lo),
                    right: Box::new(tree_hi),
                    rp,
                    gamma: r1 / rp,
                    b1: 0.0,
                    b2: 0.0,
                },
            });

            // Track: merged edge inherits output probe, or this IS the output collapse
            if probe_in_lo || probe_in_hi || output_node == Some(node) {
                output_probe_edge = Some(new_idx);
            }
            // Adjust existing probe index for removed edges
            if let Some(ref mut ope) = output_probe_edge {
                if *ope != new_idx {
                    if *ope > hi { *ope -= 2; }
                    else if *ope > lo { *ope -= 1; }
                }
            }

            changed = true;
            break;
        }
        if changed {
            continue;
        }

        // d. Last resort: series/parallel reductions may create new dead-end
        //    nodes (degree 1, non-terminal). Run dead-end elimination and
        //    retry. This handles pendant branches that only become dead ends
        //    after earlier reductions (e.g. output chain collapses into one
        //    edge to ground, leaving gnd with degree 1).
        if eliminate_dead_ends(&mut edges, terminals) {
            continue;
        }

        return Err(format!(
            "circuit is not series-parallel ({} edges remain)",
            edges.len()
        ));
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Greedy series-parallel decomposition
// ═══════════════════════════════════════════════════════════════════════════

/// Result of greedy SP decomposition for a circuit stage.
///
/// Extracts maximum SP-reducible pendant subtrees from a circuit's passive
/// network, leaving only non-reducible "bridging" edges for the MNA builder.
/// Junction nodes (NL terminals, VS injection) form the R-type core.
pub(super) struct DecomposedCircuit {
    /// SP-reduced subtrees, each becoming a single port on the R-type adaptor.
    pub wdf_subtrees: Vec<WdfSubtreePort>,
    /// Residual edge indices that must be stamped into MNA (bridging resistors).
    pub residual_edges: Vec<usize>,
}

/// A single SP-reduced subtree that feeds one port of the R-type adaptor.
pub(super) struct WdfSubtreePort {
    /// The WDF subtree (already a DynNode, built directly by graph_reduce).
    pub tree: DynNode,
    /// The junction node where this subtree attaches (R-type side).
    pub attachment_node: NodeId,
    /// The far terminal (ground/supply reference node).
    pub far_node: NodeId,
}

/// Greedily decompose a passive network into SP-reducible subtrees and residual.
///
/// Junction nodes are the "core" of the R-type adaptor — typically NL element
/// terminals and the VS injection node. The algorithm:
///
/// 1. Map supply/ground nodes to canonical ground
/// 2. Build adjacency from passive edges
/// 3. Find connected components of internal (non-junction) nodes
/// 4. Components touching exactly 1 junction node → pendant → try SP reduce
/// 5. Components touching 2+ junction nodes → bridging → residual
/// 6. Direct junction-to-junction/ground edges → residual
///
/// The resulting `DecomposedCircuit` has subtrees for SP-reducible parts and
/// residual edges for MNA stamping. If `residual_edges` is empty, the entire
/// circuit is SP-reducible and no MNA builder is needed.
pub(super) fn sp_decompose(
    passive_edge_indices: &[usize],
    junction_nodes: &[NodeId],
    graph: &CircuitGraph,
    sample_rate: f64,
) -> DecomposedCircuit {
    // Canonical ground: gnd + all supply nodes map to the same reference.
    let ground = graph.gnd_node;
    let effective = |node: NodeId| -> NodeId {
        if node == graph.vcc_node || graph.supply_nodes.contains(&node) {
            ground
        } else {
            node
        }
    };

    let junction_set: HashSet<NodeId> = junction_nodes.iter().map(|&n| effective(n)).collect();
    let is_junction_or_ground =
        |node: NodeId| -> bool { node == ground || junction_set.contains(&node) };

    // Build adjacency with effective (ground-mapped) nodes.
    // Each entry is (edge_index, other_effective_node).
    struct EdgeInfo {
        edge_idx: usize,
        eff_a: NodeId,
        eff_b: NodeId,
    }
    let edge_infos: Vec<EdgeInfo> = passive_edge_indices
        .iter()
        .map(|&eidx| {
            let e = &graph.edges[eidx];
            EdgeInfo {
                edge_idx: eidx,
                eff_a: effective(e.node_a),
                eff_b: effective(e.node_b),
            }
        })
        .collect();

    // Classify edges and find internal nodes.
    // An "internal" node is any node that is NOT a junction node and NOT ground.
    let mut internal_nodes: HashSet<NodeId> = HashSet::new();
    let mut self_loops: Vec<usize> = Vec::new(); // edges where both endpoints map to same node

    for ei in &edge_infos {
        if ei.eff_a == ei.eff_b {
            self_loops.push(ei.edge_idx);
            continue;
        }
        if !is_junction_or_ground(ei.eff_a) {
            internal_nodes.insert(ei.eff_a);
        }
        if !is_junction_or_ground(ei.eff_b) {
            internal_nodes.insert(ei.eff_b);
        }
    }

    // Build adjacency for internal nodes only (for component discovery).
    // adj[internal_node] = [(edge_idx, neighbor_node)]
    // neighbor_node can be internal, junction, or ground.
    // BTreeMap ensures deterministic iteration order.
    let mut adj: BTreeMap<NodeId, Vec<(usize, NodeId)>> = BTreeMap::new();
    for ei in &edge_infos {
        if ei.eff_a == ei.eff_b {
            continue; // skip self-loops
        }
        if internal_nodes.contains(&ei.eff_a) {
            adj.entry(ei.eff_a)
                .or_default()
                .push((ei.edge_idx, ei.eff_b));
        }
        if internal_nodes.contains(&ei.eff_b) {
            adj.entry(ei.eff_b)
                .or_default()
                .push((ei.edge_idx, ei.eff_a));
        }
    }

    // Find connected components of internal nodes via BFS.
    // For each component, track: which internal nodes, which edges, which junction nodes it borders.
    struct InternalComponent {
        edges: Vec<usize>,
        bordering_junctions: Vec<NodeId>,
    }

    let mut visited: HashSet<NodeId> = HashSet::new();
    let mut components: Vec<InternalComponent> = Vec::new();

    let mut sorted_internal: Vec<NodeId> = internal_nodes.iter().copied().collect();
    sorted_internal.sort();

    for &start in &sorted_internal {
        if visited.contains(&start) {
            continue;
        }
        let mut comp = InternalComponent {
            edges: Vec::new(),
            bordering_junctions: Vec::new(),
        };
        let mut queue = std::collections::VecDeque::new();
        let mut comp_nodes: HashSet<NodeId> = HashSet::new();
        visited.insert(start);
        comp_nodes.insert(start);
        queue.push_back(start);

        while let Some(node) = queue.pop_front() {
            if let Some(neighbors) = adj.get(&node) {
                for &(eidx, neighbor) in neighbors {
                    if is_junction_or_ground(neighbor) {
                        // This edge borders a junction/ground — record it.
                        if neighbor != ground
                            && !comp.bordering_junctions.contains(&neighbor)
                        {
                            comp.bordering_junctions.push(neighbor);
                        }
                    } else if visited.insert(neighbor) {
                        comp_nodes.insert(neighbor);
                        queue.push_back(neighbor);
                    }
                }
            }
        }

        // Collect all edges that have at least one endpoint in this component's internal nodes.
        let mut comp_edge_set: HashSet<usize> = HashSet::new();
        for ei in &edge_infos {
            if ei.eff_a == ei.eff_b {
                continue;
            }
            let a_in = comp_nodes.contains(&ei.eff_a);
            let b_in = comp_nodes.contains(&ei.eff_b);
            if a_in || b_in {
                comp_edge_set.insert(ei.edge_idx);
                // Also track junction borders from edges.
                if a_in
                    && is_junction_or_ground(ei.eff_b)
                    && ei.eff_b != ground
                    && !comp.bordering_junctions.contains(&ei.eff_b)
                {
                    comp.bordering_junctions.push(ei.eff_b);
                }
                if b_in
                    && is_junction_or_ground(ei.eff_a)
                    && ei.eff_a != ground
                    && !comp.bordering_junctions.contains(&ei.eff_a)
                {
                    comp.bordering_junctions.push(ei.eff_a);
                }
            }
        }
        comp.edges = comp_edge_set.into_iter().collect();
        comp.edges.sort(); // deterministic order
        comp.bordering_junctions.sort(); // deterministic order
        components.push(comp);
    }

    // Also collect "direct" edges: both endpoints are junction/ground (no internal nodes).
    let mut direct_edges: Vec<usize> = Vec::new();
    for ei in &edge_infos {
        if ei.eff_a == ei.eff_b {
            continue;
        }
        if is_junction_or_ground(ei.eff_a) && is_junction_or_ground(ei.eff_b) {
            direct_edges.push(ei.edge_idx);
        }
    }

    // Now classify components and try SP-reduce pendants.
    let mut wdf_subtrees: Vec<WdfSubtreePort> = Vec::new();
    let mut residual_edges: Vec<usize> = Vec::new();

    // Direct junction-to-junction edges always go to residual.
    residual_edges.extend(&direct_edges);

    // Self-loop edges (both endpoints map to same effective node) are dead — skip.

    for comp in &components {
        if comp.edges.is_empty() {
            continue;
        }

        if comp.bordering_junctions.len() == 1 {
            // Pendant: hangs off exactly one junction node → try graph_reduce.

            let &junction = comp.bordering_junctions.iter().next().unwrap();
            let terminals = vec![junction, ground];
            let remap = |n: NodeId| -> NodeId { effective(n) };

            match graph_reduce(
                &comp.edges,
                &[],
                &terminals,
                graph,
                sample_rate,
                &HashMap::new(),
                remap,
                None,
            ) {
                Ok((tree, _)) => {
                    wdf_subtrees.push(WdfSubtreePort {
                        tree,
                        attachment_node: junction,
                        far_node: ground,
                    });
                }
                Err(_) => {
                    // Not SP-reducible even as pendant — leave for MNA.
                    residual_edges.extend(&comp.edges);
                }
            }
        } else if comp.bordering_junctions.is_empty() {
            // Component only touches ground, no junction nodes.
            // This is an isolated sub-network — treat as residual (might be
            // a grounding network that the MNA needs for regularization).
            residual_edges.extend(&comp.edges);
        } else {
            // Bridging: connects 2+ junction nodes → must go to MNA.
            residual_edges.extend(&comp.edges);
        }
    }

    residual_edges.sort();
    residual_edges.dedup();

    DecomposedCircuit {
        wdf_subtrees,
        residual_edges,
    }
}

/// Active resistance for fork paths (effectively a short circuit)
const FORK_R_ACTIVE: f64 = 1.0;
/// Inactive resistance for fork paths (effectively open circuit)
const FORK_R_INACTIVE: f64 = 1_000_000.0;

pub(super) fn make_leaf(
    _comp_idx: usize,
    comp: &ComponentDef,
    fork_info: Option<&ForkPathInfo>,
    sample_rate: f64,
) -> DynNode {
    // Check if this is a fork path component
    if let Some(info) = fork_info {
        // Fork path: create a SwitchedResistor
        // Default to position 0 (first path active)
        let is_active = info.path_index == 0;
        return DynNode::SwitchedResistor {
            switch_id: info.switch_id.clone(),
            path_index: info.path_index,
            num_paths: info.num_paths,
            r_active: FORK_R_ACTIVE,
            r_inactive: FORK_R_INACTIVE,
            position: 0,
            rp: if is_active {
                FORK_R_ACTIVE
            } else {
                FORK_R_INACTIVE
            },
        };
    }

    // Delegate to Component trait; fallback for non-leaf types (diodes, etc.)
    comp.kind
        .make_leaf(&comp.id, sample_rate)
        .unwrap_or(DynNode::Resistor { rp: 1000.0, last_a: 0.0 })
}

// ═══════════════════════════════════════════════════════════════════════════
// Post-construction wiring resolution
// ═══════════════════════════════════════════════════════════════════════════

/// Resolve context-dependent component edge kinds based on wiring.
///
/// Scans the pedal's nets for modulation connections (LFO/envelope → control pins)
/// and calls `resolve_edges()` on each modulated component. Results are stored in
/// `graph.resolved_edge_kinds` which maps edge index → resolved EdgeKind.
///
/// This generalizes `detect_lfo_controlled_jfets()` and
/// `detect_envelope_controlled_otas()` in compile.rs.
pub(super) fn resolve_components(graph: &mut CircuitGraph, pedal: &PedalDef) {
    // Collect modulator component IDs (LFO, EnvelopeFollower).
    let modulator_ids: HashSet<&str> = pedal
        .components
        .iter()
        .filter(|c| c.kind.is_modulation_source())
        .map(|c| c.id.as_str())
        .collect();

    if modulator_ids.is_empty() {
        return;
    }

    // Scan nets to find which components have modulated control pins.
    let mut modulated_components: HashSet<String> = HashSet::new();

    for net in &pedal.nets {
        let from_is_modulator = match &net.from {
            Pin::ComponentPin { component, pin } => {
                modulator_ids.contains(component.as_str()) && pin == "out"
            }
            _ => false,
        };
        if !from_is_modulator {
            continue;
        }
        for to_pin in &net.to {
            if let Pin::ComponentPin { component, pin } = to_pin {
                let comp_def = pedal.components.iter().find(|c| c.id == *component);
                if let Some(comp_def) = comp_def {
                    if comp_def.kind.modulation_pins().contains(&pin.as_str()) {
                        modulated_components.insert(component.clone());
                    }
                }
            }
        }
    }

    if modulated_components.is_empty() {
        return;
    }

    // For each graph edge whose component is modulated, resolve its edge kind.
    for (edge_idx, edge) in graph.edges.iter().enumerate() {
        let comp = &graph.components[edge.comp_idx];

        if !modulated_components.contains(&comp.id) {
            continue;
        }

        let ctx = ResolveContext {
            control_pin_is_modulated: true,
            wiper_connected: false,
        };

        if let Some(resolved_edges) = comp.kind.resolve_edges(&ctx) {
            // Apply the first resolved edge's kind to this graph edge.
            // Works because JFET/OTA each produce a single circuit edge.
            if let Some(first) = resolved_edges.first() {
                graph.resolved_edge_kinds.insert(edge_idx, first.kind);
            }
        }
    }
}
