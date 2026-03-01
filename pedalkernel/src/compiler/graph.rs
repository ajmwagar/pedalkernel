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
    /// Supply rail nodes (vcc + named supplies like B+, A_bal, etc.).
    /// Edges to these nodes are excluded from passive element collection
    /// because supply voltages are injected as tube bias parameters, not
    /// as part of the WDF tree.
    pub(super) supply_nodes: HashSet<NodeId>,
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
    component_base_idx: usize,
) -> (Vec<NetDef>, Vec<(ComponentDef, ForkPathInfo)>) {
    let mut expanded_nets = Vec::new();
    let mut fork_components: Vec<(ComponentDef, ForkPathInfo)> = Vec::new();
    let mut fork_counter = 0usize;

    for net in nets {
        let mut new_to: Vec<Pin> = Vec::new();

        for dest in &net.to {
            match dest {
                Pin::Fork { switch, destinations } => {
                    // Create a synthetic fork path component for each destination
                    let num_paths = destinations.len();
                    for (path_idx, dest_pin) in destinations.iter().enumerate() {
                        // Synthetic component ID: __fork_<counter>_path_<idx>
                        let comp_id = format!("__fork_{}_path_{}", fork_counter, path_idx);

                        // Create a synthetic resistor component (value will be set at runtime)
                        // Use a small resistance for active path, large for inactive
                        let comp = ComponentDef {
                            id: comp_id.clone(),
                            kind: ComponentKind::Resistor(1.0), // Placeholder - actual value set by SwitchedResistor
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
        let (expanded_nets, fork_components) =
            expand_forks(&pedal.nets, pedal.components.len());

        // Build combined component list: original + fork paths
        let mut all_components: Vec<ComponentDef> = pedal.components.clone();
        let mut fork_paths: HashMap<usize, ForkPathInfo> = HashMap::new();

        for (comp, info) in fork_components {
            let comp_idx = all_components.len();
            fork_paths.insert(comp_idx, info);
            all_components.push(comp);
        }

        // Pre-scan: identify pots that use .w (wiper) pin in nets.
        // A pot with .w is a 3-terminal pot: lug A, wiper, lug B.
        // Must be done before the get_id closure borrows pin_ids.
        let pots_with_wiper: HashSet<String> = {
            let mut set = HashSet::new();
            for net in &expanded_nets {
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

        // Build edges for two-terminal components.
        let mut edges = Vec::new();
        let mut num_active = 0usize;
        let mut deferred_3term: Vec<(usize, String)> = Vec::new();

        for (idx, comp) in all_components.iter().enumerate() {
            match &comp.kind {
                ComponentKind::Potentiometer(_, _) => {
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
                ComponentKind::Transformer(cfg) => {
                    // Transformer: primary winding is a-b, secondary is c-d.
                    // For WDF, we model the ideal transformer as a 2-port adaptor
                    // with the turns ratio. The primary side is the WDF edge.
                    //
                    // Pin naming: the DSL supports both shorthand (.a/.b) and
                    // explicit winding names (.primary.a/.primary.b). Union both
                    // so that either convention works in netlists.
                    let key_a = format!("{}.a", comp.id);
                    let key_b = format!("{}.b", comp.id);
                    let key_pri_a = format!("{}.primary.a", comp.id);
                    let key_pri_b = format!("{}.primary.b", comp.id);
                    let id_a = get_id(&key_a, &mut uf);
                    let id_b = get_id(&key_b, &mut uf);
                    // Union shorthand pins with explicit winding pins
                    let id_pri_a = get_id(&key_pri_a, &mut uf);
                    let id_pri_b = get_id(&key_pri_b, &mut uf);
                    uf.union(id_a, id_pri_a);
                    uf.union(id_b, id_pri_b);
                    // Also union secondary pins: .c/.d with .secondary.a/.secondary.b
                    let key_c = format!("{}.c", comp.id);
                    let key_d = format!("{}.d", comp.id);
                    let key_sec_a = format!("{}.secondary.a", comp.id);
                    let key_sec_b = format!("{}.secondary.b", comp.id);
                    let id_c = get_id(&key_c, &mut uf);
                    let id_d = get_id(&key_d, &mut uf);
                    let id_sec_a = get_id(&key_sec_a, &mut uf);
                    let id_sec_b = get_id(&key_sec_b, &mut uf);
                    uf.union(id_c, id_sec_a);
                    uf.union(id_d, id_sec_b);

                    // Register tertiary pins if the transformer has a third winding.
                    // .e/.f shorthand, .tertiary.a/.tertiary.b explicit names.
                    if cfg.has_tertiary() {
                        let key_e = format!("{}.e", comp.id);
                        let key_f = format!("{}.f", comp.id);
                        let key_ter_a = format!("{}.tertiary.a", comp.id);
                        let key_ter_b = format!("{}.tertiary.b", comp.id);
                        let id_e = get_id(&key_e, &mut uf);
                        let id_f = get_id(&key_f, &mut uf);
                        let id_ter_a = get_id(&key_ter_a, &mut uf);
                        let id_ter_b = get_id(&key_ter_b, &mut uf);
                        uf.union(id_e, id_ter_a);
                        uf.union(id_f, id_ter_b);
                    }

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
            let (max_r, taper) = match &pedal.components[*_original_idx].kind {
                ComponentKind::Potentiometer(r, t) => (*r, *t),
                _ => unreachable!(),
            };

            // Synthetic upper-half: a → wiper
            let aw_idx = pedal.components.len() + extra_components.len();
            extra_components.push(ComponentDef {
                id: format!("{pot_id}__aw"),
                kind: ComponentKind::Potentiometer(max_r, taper),
            });

            // Synthetic lower-half: wiper → b
            let wb_idx = pedal.components.len() + extra_components.len();
            extra_components.push(ComponentDef {
                id: format!("{pot_id}__wb"),
                kind: ComponentKind::Potentiometer(max_r, taper),
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

        // Re-resolve all edge node IDs through the final union-find state.
        // This is necessary because transformer pin aliasing (and potentially
        // other late unions) can merge nodes AFTER edges were stored.
        // Without this, edges may reference stale union-find roots.
        for edge in &mut edges {
            edge.node_a = uf.find(edge.node_a);
            edge.node_b = uf.find(edge.node_b);
        }

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
        for supply in &pedal.supplies {
            if let Some(&raw_id) = pin_ids.get(&supply.name) {
                let resolved = uf.find(raw_id);
                // Skip vcc and gnd — they're standard WDF terminations
                if resolved != vcc_node && resolved != gnd_node {
                    supply_nodes.insert(resolved);
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

        CircuitGraph {
            edges,
            components,
            in_node,
            out_node,
            gnd_node,
            supply_nodes,
            num_active,
            active_edge_indices,
            fork_paths,
            node_names,
        }
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

        // Boundary nodes: the BFS stops at these. They represent circuit
        // boundaries that separate the sidechain from the audio path.
        let mut boundary: HashSet<NodeId> = HashSet::new();
        boundary.insert(self.in_node);
        boundary.insert(self.out_node);
        boundary.insert(self.gnd_node);
        if let Some(&vcc) = self.node_names.get("vcc") {
            boundary.insert(vcc);
        }
        boundary.extend(&self.supply_nodes);

        // BFS from tap_node: find all nodes reachable without crossing
        // cv_node or any boundary node.
        let reachable_from_tap = Self::bfs_reachable(tap_node, cv_node, &boundary, &adj);

        // BFS from cv_node: find all nodes reachable without crossing
        // tap_node or any boundary node.
        let reachable_from_cv = Self::bfs_reachable(cv_node, tap_node, &boundary, &adj);

        eprintln!("[partition] reachable_from_tap: {} nodes, reachable_from_cv: {} nodes",
            reachable_from_tap.len(), reachable_from_cv.len());
        eprintln!("[partition] boundary has {} nodes (in={}, out={}, gnd={})",
            boundary.len(), self.in_node, self.out_node, self.gnd_node);
        eprintln!("[partition] tap={}, cv={}", tap_node, cv_node);

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
    /// Stops at `opposite` (the other sidechain boundary) and at all `boundary` nodes.
    /// The start node itself is NOT included in the result (it's a boundary).
    fn bfs_reachable(
        start: NodeId,
        opposite: NodeId,
        boundary: &HashSet<NodeId>,
        adj: &HashMap<NodeId, Vec<(usize, NodeId)>>,
    ) -> HashSet<NodeId> {
        let mut visited: HashSet<NodeId> = HashSet::new();
        let mut queue = std::collections::VecDeque::new();
        visited.insert(start);

        // Seed: follow all edges from start, except to boundary or opposite
        if let Some(neighbors) = adj.get(&start) {
            for &(_, neighbor) in neighbors {
                if neighbor == opposite || boundary.contains(&neighbor) {
                    continue;
                }
                if visited.insert(neighbor) {
                    queue.push_back(neighbor);
                }
            }
        }

        while let Some(node) = queue.pop_front() {
            if let Some(neighbors) = adj.get(&node) {
                for &(_, neighbor) in neighbors {
                    if neighbor == start || neighbor == opposite || boundary.contains(&neighbor) {
                        continue;
                    }
                    if visited.insert(neighbor) {
                        queue.push_back(neighbor);
                    }
                }
            }
        }

        // Remove start from the result — it's a boundary, not a sidechain-internal node
        visited.remove(&start);
        visited
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

            // Check if one terminal is connected to ground.
            // Diodes connected to ground create "hard clipping" stages.
            // Diodes in feedback loops (neither terminal at ground) create
            // "soft clipping" that should be modeled within the op-amp.
            let a_is_gnd = e.node_a == self.gnd_node;
            let b_is_gnd = e.node_b == self.gnd_node;
            let has_gnd = a_is_gnd || b_is_gnd;

            let info = match &comp.kind {
                ComponentKind::DiodePair(dt) => Some(DiodeInfo {
                    diode_type: *dt,
                    is_pair: true,
                    // For ground-connected diodes, use the non-ground node as junction
                    // For feedback diodes, use node_a as junction (arbitrary but consistent)
                    junction_node: if has_gnd {
                        if b_is_gnd { e.node_a } else { e.node_b }
                    } else {
                        e.node_a // feedback diode: junction at first node
                    },
                    ground_node: if has_gnd {
                        if b_is_gnd { e.node_b } else { e.node_a }
                    } else {
                        e.node_b // feedback diode: "ground" at second node
                    },
                }),
                ComponentKind::Diode(dt) => Some(DiodeInfo {
                    diode_type: *dt,
                    is_pair: false,
                    junction_node: if has_gnd {
                        if b_is_gnd { e.node_a } else { e.node_b }
                    } else {
                        e.node_a
                    },
                    ground_node: if has_gnd {
                        if b_is_gnd { e.node_b } else { e.node_a }
                    } else {
                        e.node_b
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
                ComponentKind::NJfet(name) => Some(JfetInfo {
                    model_name: name.clone(),
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
                ComponentKind::PJfet(name) => Some(JfetInfo {
                    model_name: name.clone(),
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
                ComponentKind::Npn(name) => Some(BjtInfo {
                    model_name: name.clone(),
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
                    // TODO: Properly determine pin nodes from netlist
                    base_node: e.node_a,
                    emitter_node: self.gnd_node,
                    collector_node: e.node_b,
                }),
                ComponentKind::Pnp(name) => Some(BjtInfo {
                    model_name: name.clone(),
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
                    // TODO: Properly determine pin nodes from netlist
                    base_node: e.node_a,
                    emitter_node: self.gnd_node,
                    collector_node: e.node_b,
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
    /// Parallel tubes sharing both plate and cathode nodes are merged into
    /// a single entry with `parallel_count > 1`.
    /// Returns (merged_triodes, all_triode_edge_indices) where the second vec
    /// contains ALL triode edge indices including parallel duplicates.
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

        // Collect all triode edges.
        let mut raw_triodes: Vec<(usize, String, NodeId, NodeId)> = Vec::new();
        for (edge_idx, e) in self.edges.iter().enumerate() {
            let comp = &self.components[e.comp_idx];
            if let ComponentKind::Triode(name) = &comp.kind {
                raw_triodes.push((edge_idx, name.clone(), e.node_a, e.node_b));
            }
        }

        // Group by (plate_node, cathode_node) to detect parallel tubes.
        // Tubes sharing both nodes are electrically identical and should be
        // modeled as a single tube with scaled plate current.
        let mut groups: HashMap<(NodeId, NodeId), Vec<(usize, String)>> = HashMap::new();
        for (edge_idx, name, plate, cathode) in &raw_triodes {
            groups.entry((*plate, *cathode)).or_default().push((*edge_idx, name.clone()));
        }

        let mut triodes: Vec<(usize, TriodeInfo)> = Vec::new();
        for ((plate_node, cathode_node), group) in &groups {
            // Use the first edge as the representative; store parallel count.
            let (rep_edge_idx, ref rep_name) = group[0];
            triodes.push((
                rep_edge_idx,
                TriodeInfo {
                    model_name: rep_name.clone(),
                    plate_node: *plate_node,
                    cathode_node: *cathode_node,
                    junction_node: *cathode_node,
                    ground_node: self.gnd_node,
                    parallel_count: group.len(),
                },
            ));
        }

        // Collect all triode edge indices (including parallel duplicates).
        let all_triode_edges: Vec<usize> = raw_triodes.iter().map(|(idx, _, _, _)| *idx).collect();

        // Sort by distance of junction node from input, with edge_idx as tiebreaker
        // for deterministic ordering when distances are equal.
        triodes.sort_by_key(|(edge_idx, info)| {
            (dist.get(&info.junction_node).copied().unwrap_or(usize::MAX), *edge_idx)
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
    ) -> Vec<PushPullPairInfo> {
        // Find CT transformers (primary_type = CenterTap or PushPull).
        let mut ct_transformers: Vec<(usize, &TransformerConfig)> = Vec::new();
        for (edge_idx, e) in self.edges.iter().enumerate() {
            let comp = &self.components[e.comp_idx];
            if let ComponentKind::Transformer(cfg) = &comp.kind {
                if matches!(cfg.primary_type, WindingType::CenterTap | WindingType::PushPull) {
                    ct_transformers.push((edge_idx, cfg));
                }
            }
        }

        let mut pairs = Vec::new();
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

            let nodes_near_a = self.bfs_through_passives(
                primary_a_node,
                &exclude,
                &self.active_edge_indices,
            );
            let nodes_near_b = self.bfs_through_passives(
                primary_b_node,
                &exclude,
                &self.active_edge_indices,
            );

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
                pairs.push(PushPullPairInfo {
                    push_triode_idx: push,
                    pull_triode_idx: pull,
                    transformer_edge_idx: *xfmr_edge_idx,
                    turns_ratio: cfg.turns_ratio,
                });
            }
        }

        pairs
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
            if let ComponentKind::Pentode(name) = &comp.kind {
                pentodes.push((
                    edge_idx,
                    PentodeInfo {
                        model_name: name.clone(),
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

    /// Find op-amp feedback loops and calculate closed-loop gain.
    ///
    /// Detects:
    /// - **Unity-gain buffers**: neg and out pins resolve to the same node
    /// - **Inverting amplifiers**: pos to ground, Rf from neg to out, Ri from input to neg
    /// - **Non-inverting amplifiers**: signal to pos, Rf from neg to out, Ri from neg to ground
    ///
    /// Returns a list of `OpAmpFeedbackInfo` ordered by topological distance
    /// from the input node.
    pub(super) fn find_opamp_feedback_loops(
        &self,
        pedal: &PedalDef,
    ) -> Vec<OpAmpFeedbackInfo> {
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
        let in_node_resolved = uf.find(*pin_ids.get("in").unwrap_or(&0));
        let gnd_node_resolved = uf.find(*pin_ids.get("gnd").unwrap_or(&0));

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
            let (resistance, is_pot, max_r) = match &comp.kind {
                ComponentKind::Resistor(r) => (Some(*r), false, *r),
                ComponentKind::Potentiometer(max_r, _) => {
                    // Use default position (0.5) for initial gain calculation
                    (Some(*max_r * 0.5), true, *max_r)
                }
                _ => (None, false, 0.0),
            };

            if let Some(r) = resistance {
                let pin_a_key = format!("{}.a", comp.id);
                let pin_b_key = format!("{}.b", comp.id);
                if let (Some(&a_id), Some(&b_id)) = (pin_ids.get(&pin_a_key), pin_ids.get(&pin_b_key)) {
                    let a_node = uf.find(a_id);
                    let b_node = uf.find(b_id);
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
        let find_input_resistor = |neg_node: usize, out_node: usize, gnd_node: usize| -> Option<f64> {
            for info in &resistor_nodes {
                // Skip the feedback resistor (connects neg to out)
                if (info.node_a == neg_node && info.node_b == out_node) || (info.node_a == out_node && info.node_b == neg_node) {
                    continue;
                }
                // Skip resistors to ground (those are Ri for non-inverting topology)
                if (info.node_a == neg_node && info.node_b == gnd_node) || (info.node_a == gnd_node && info.node_b == neg_node) {
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
        let find_resistive_path = |start: usize, end: usize| -> Option<(f64, Vec<String>, Option<(String, f64, f64, Option<f64>)>)> {
            if start == end {
                return None; // Same node, no resistance
            }

            // Build adjacency for resistor-only edges
            // Store (next_node, resistance, comp_id, is_pot, max_r)
            let mut resistor_adj: HashMap<usize, Vec<(usize, f64, String, bool, f64)>> = HashMap::new();
            for info in &resistor_nodes {
                resistor_adj.entry(info.node_a).or_default().push((info.node_b, info.resistance, info.id.clone(), info.is_pot, info.max_r));
                resistor_adj.entry(info.node_b).or_default().push((info.node_a, info.resistance, info.id.clone(), info.is_pot, info.max_r));
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
                fixed_r: f64, // Sum of non-pot resistors in path
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
                                let pot_info = state.pot_info.clone().or(Some((comp_id.clone(), *max_r)));
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
                let pot_info_4 = pot_info.map(|(id, max_r, fixed_series)| (id, max_r, fixed_series, None));
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
            let diode_type = match &comp.kind {
                ComponentKind::Diode(dt) | ComponentKind::DiodePair(dt) => *dt,
                _ => continue,
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

        // Helper: find diodes between two nodes (for feedback diode detection)
        let find_feedback_diode = |node_a: usize, node_b: usize| -> Option<DiodeType> {
            feedback_diodes.get(&(node_a, node_b)).copied()
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
            let opamp_type = match &comp.kind {
                ComponentKind::OpAmp(ot) if !ot.is_ota() => *ot,
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
                    });
                    continue;
                }

                // Look for feedback resistor path (Rf: neg to out)
                // Use resistive path finding to handle series/parallel combinations
                // Returns (rf_value, component_ids, pot_info)
                if let Some((rf, _rf_comps, rf_pot)) = find_resistive_path(neg_node, out_node) {
                    // Check for inverting topology: pos connected to ground
                    if pos_node == gnd_node_resolved {
                        // Inverting: look for Ri connected to neg (from any input source)
                        // For cascaded op-amps, Ri may connect to a previous stage's output
                        if let Some(ri) = find_input_resistor(neg_node, out_node, gnd_node_resolved) {
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
                            });
                            continue;
                        }
                    }

                    // Check for non-inverting topology: pos connected to input (or signal path)
                    // Non-inverting: look for Ri path from neg to ground
                    if let Some((ri, _ri_comps, _ri_pot)) = find_resistive_path(neg_node, gnd_node_resolved) {
                        results.push(OpAmpFeedbackInfo {
                            comp_id: comp.id.clone(),
                            opamp_type,
                            feedback_kind: OpAmpFeedbackKind::NonInverting { rf, ri, rf_pot },
                            neg_node,
                            pos_node,
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
        results.sort_by_key(|info| {
            dist.get(&info.pos_node).copied().unwrap_or(usize::MAX)
        });

        results
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
    pub(super) model_name: String,
    pub(super) is_n_channel: bool,
    pub(super) junction_node: NodeId,
    #[allow(dead_code)]
    pub(super) ground_node: NodeId,
}

pub(super) struct BjtInfo {
    pub(super) model_name: String,
    pub(super) is_npn: bool,
    /// The collector node (where the WDF tree connects).
    pub(super) junction_node: NodeId,
    pub(super) ground_node: NodeId,
    /// Base node for detecting complementary pairs.
    pub(super) base_node: NodeId,
    /// Emitter node for push-pull detection.
    pub(super) emitter_node: NodeId,
    /// Collector node for push-pull detection.
    pub(super) collector_node: NodeId,
}

/// Type of push-pull configuration.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum PushPullType {
    /// Shared collector (output node) — typical for Class AB output stages.
    /// Both transistors connect to the same output through their collectors.
    SharedCollector,
    /// Shared emitter — differential pair or long-tailed pair.
    /// Both transistors share an emitter connection (often through a resistor).
    SharedEmitter,
}

/// A detected push-pull transistor pair.
///
/// Push-pull pairs exhibit crossover distortion when the bias current is low
/// enough that there's a "dead zone" where neither transistor conducts.
/// The width of this dead zone depends on Vbe (≈0.6V for silicon, ≈0.2V for
/// germanium) and the bias current.
#[allow(dead_code)]
#[derive(Debug, Clone)]
pub(super) struct PushPullPair {
    /// Index of the NPN transistor in the BJT list.
    pub(super) npn_idx: usize,
    /// Index of the PNP transistor in the BJT list.
    pub(super) pnp_idx: usize,
    /// The shared node where crossover occurs.
    pub(super) shared_node: NodeId,
    /// Type of push-pull configuration.
    pub(super) pair_type: PushPullType,
}

/// A detected push-pull tube pair (for Class AB tube output stages).
#[allow(dead_code)]
#[derive(Debug, Clone)]
pub(super) struct TubePushPullPair {
    /// Index of first triode in the triode list.
    pub(super) triode_a_idx: usize,
    /// Index of second triode in the triode list.
    pub(super) triode_b_idx: usize,
    /// The shared cathode node.
    pub(super) shared_cathode: NodeId,
}

/// Push-pull pair detected via center-tapped transformer.
pub(super) struct PushPullPairInfo {
    /// Index into the merged triodes list for the push half.
    pub(super) push_triode_idx: usize,
    /// Index into the merged triodes list for the pull half.
    pub(super) pull_triode_idx: usize,
    /// Edge index of the CT transformer connecting them.
    #[allow(dead_code)]
    pub(super) transformer_edge_idx: usize,
    /// Turns ratio of the output transformer (primary:secondary).
    pub(super) turns_ratio: f64,
}

pub(super) struct TriodeInfo {
    pub(super) model_name: String,
    /// Plate node - connected to plate load resistor and output
    pub(super) plate_node: NodeId,
    /// Cathode node - connected to cathode resistor and bypass cap
    pub(super) cathode_node: NodeId,
    /// Legacy junction_node - kept for compatibility, equals cathode_node
    pub(super) junction_node: NodeId,
    #[allow(dead_code)]
    pub(super) ground_node: NodeId,
    /// Number of parallel tubes sharing the same plate and cathode nodes.
    /// Default is 1. When > 1, the tube model scales plate current by N.
    pub(super) parallel_count: usize,
}

pub(super) struct PentodeInfo {
    pub(super) model_name: String,
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
    pub(super) neg_node: NodeId,
    /// Non-inverting input node (signal reference or bias).
    pub(super) pos_node: NodeId,
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
        /// Potentiometer info for runtime gain modulation.
        /// (comp_id, max_resistance, fixed_series_resistance, parallel_fixed_resistance)
        rf_pot: Option<(String, f64, f64, Option<f64>)>,
    },
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
        // Sort nodes for deterministic iteration order
        let mut sorted_nodes: Vec<_> = degree.keys().copied().collect();
        sorted_nodes.sort();
        for node in sorted_nodes {
            let idxs = &degree[&node];
            if terminals.contains(&node) || idxs.len() != 1 {
                continue;
            }
            // Dead-end: redirect this edge's dead side to terminals[0].
            let eidx = idxs[0];
            if edges[eidx].0 == node {
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
        // Sort nodes for deterministic iteration order
        let mut sorted_nodes: Vec<_> = degree.keys().copied().collect();
        sorted_nodes.sort();
        for node in sorted_nodes {
            let idxs = &degree[&node];
            if terminals.contains(&node) || idxs.len() != 2 {
                continue;
            }
            let i1 = idxs[0];
            let i2 = idxs[1];
            let other1 = if edges[i1].0 == node {
                edges[i1].1
            } else {
                edges[i1].0
            };
            let other2 = if edges[i2].0 == node {
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
pub(super) fn sp_to_dyn(
    tree: &SpTree,
    components: &[ComponentDef],
    fork_paths: &HashMap<usize, ForkPathInfo>,
    sample_rate: f64,
) -> DynNode {
    match tree {
        SpTree::Leaf(idx) => make_leaf(*idx, &components[*idx], fork_paths.get(idx), sample_rate),
        SpTree::Series(left, right) => {
            let l = sp_to_dyn(left, components, fork_paths, sample_rate);
            let r = sp_to_dyn(right, components, fork_paths, sample_rate);
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
            let l = sp_to_dyn(left, components, fork_paths, sample_rate);
            let r = sp_to_dyn(right, components, fork_paths, sample_rate);
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
            rp: if is_active { FORK_R_ACTIVE } else { FORK_R_INACTIVE },
        };
    }

    // Regular component handling
    match &comp.kind {
        // Handle inf (infinite) resistor as very high resistance (open circuit)
        ComponentKind::Resistor(r) if r.is_infinite() => DynNode::Resistor { rp: FORK_R_INACTIVE },
        ComponentKind::Resistor(r) => DynNode::Resistor { rp: *r },
        ComponentKind::Capacitor(cfg) => {
            // Use LeakyCapacitor if leakage or DA is specified
            if cfg.leakage.is_some() || cfg.da.is_some() {
                let rp = 1.0 / (2.0 * sample_rate * cfg.value);

                // Calculate leakage decay factor per sample
                // decay = exp(-dt / tau) where tau = R_leak * C and dt = 1/fs
                // decay = exp(-1 / (fs * R_leak * C))
                let leakage_decay = if let Some(r_leak) = cfg.leakage {
                    let tau = r_leak * cfg.value; // RC time constant in seconds
                    let dt = 1.0 / sample_rate;
                    (-dt / tau).exp()
                } else {
                    1.0 // No decay
                };

                // Calculate DA rate (time constant ~0.5s)
                const DA_TIME_CONSTANT: f64 = 0.5;
                let da_rate = if cfg.da.is_some() {
                    1.0 / (sample_rate * DA_TIME_CONSTANT)
                } else {
                    0.0
                };

                DynNode::LeakyCapacitor {
                    capacitance: cfg.value,
                    rp,
                    state: 0.0,
                    leakage_decay,
                    da_coef: cfg.da,
                    da_state: 0.0,
                    da_rate,
                }
            } else {
                // Use simple Capacitor for ideal caps (faster hot path)
                DynNode::Capacitor {
                    capacitance: cfg.value,
                    rp: 1.0 / (2.0 * sample_rate * cfg.value),
                    state: 0.0,
                    last_b: 0.0,
                }
            }
        }
        ComponentKind::Inductor(l) => DynNode::Inductor {
            inductance: *l,
            rp: 2.0 * sample_rate * *l,
            state: 0.0,
        },
        ComponentKind::Potentiometer(max_r, taper) => {
            // Apply taper for initial position (0.5)
            let initial_pos = 0.5;
            let tapered_pos = taper.apply(initial_pos);
            DynNode::Pot {
                comp_id: comp.id.clone(),
                max_resistance: *max_r,
                position: initial_pos,
                taper: *taper,
                rp: (tapered_pos * *max_r).max(1.0),
            }
        }
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
        // Transformer: create a transformer adaptor with secondary load stub.
        //
        // The transformer connects primary and secondary windings.
        // For now, we model the secondary as a simple resistive load (the output impedance).
        // Full transformer modeling requires building the secondary subtree separately.
        //
        // The turns ratio determines voltage/current transformation:
        // - V_primary / V_secondary = turns_ratio
        // - I_secondary / I_primary = turns_ratio (power conservation)
        // - Z_primary = turns_ratio² × Z_secondary
        ComponentKind::Transformer(cfg) => {
            let l_primary = cfg.primary_inductance;
            let n = cfg.turns_ratio; // Primary:Secondary ratio

            // Secondary inductance scales by 1/n²
            let l_secondary = l_primary / (n * n);

            // Secondary stub (inductor representing magnetizing inductance)
            let secondary = Box::new(DynNode::Inductor {
                inductance: l_secondary,
                rp: 2.0 * sample_rate * l_secondary,
                state: 0.0,
            });

            if let Some(n_tertiary) = cfg.tertiary_turns_ratio {
                // 3-winding transformer → R-type adaptor
                // Ports: 0=secondary, 1=tertiary, 2=primary (adapted, reflection-free)
                let l_tertiary = l_primary / (n_tertiary * n_tertiary);
                let tertiary = Box::new(DynNode::Inductor {
                    inductance: l_tertiary,
                    rp: 2.0 * sample_rate * l_tertiary,
                    state: 0.0,
                });

                let r_sec = secondary.port_resistance();
                let r_ter = tertiary.port_resistance();
                let adaptor = crate::tree::RTypeAdaptor::three_winding_transformer(
                    n, n_tertiary, r_sec, r_ter,
                );

                DynNode::RType {
                    adaptor,
                    children: vec![secondary, tertiary],
                }
            } else {
                // Standard 2-winding transformer
                let rp_sec = secondary.port_resistance();
                let rp_prim = n * n * rp_sec;

                DynNode::Transformer {
                    secondary,
                    turns_ratio: n,
                    rp: rp_prim,
                    b_sec: 0.0,
                }
            }
        }
        // Diodes shouldn't appear as leaves (they're roots), but handle gracefully.
        _ => DynNode::Resistor { rp: 1000.0 },
    }
}
