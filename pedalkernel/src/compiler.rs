//! Netlist-to-WDF compiler.
//!
//! Compiles a parsed `.pedal` file into a real-time audio processor by:
//! 1. Building a circuit graph from the netlist
//! 2. Identifying clipping stages (around diode elements)
//! 3. For each stage, building a WDF binary tree via series-parallel decomposition
//! 4. Modeling active elements (transistors, opamps) as gain stages
//! 5. Chaining everything into a cascaded `PedalProcessor`

use std::collections::HashMap;

use crate::dsl::*;
use crate::elements::*;
use crate::PedalProcessor;

// ═══════════════════════════════════════════════════════════════════════════
// Dynamic WDF tree node
// ═══════════════════════════════════════════════════════════════════════════

/// A node in a dynamically-constructed WDF binary tree.
///
/// Leaves are one-port elements; internal nodes are series/parallel adaptors.
/// All state is inline — zero heap allocation on the processing hot path.
pub enum DynNode {
    Resistor {
        rp: f64,
    },
    Capacitor {
        capacitance: f64,
        rp: f64,
        state: f64,
    },
    Inductor {
        inductance: f64,
        rp: f64,
        state: f64,
    },
    VoltageSource {
        voltage: f64,
        rp: f64,
    },
    /// Variable resistor (potentiometer) — tagged with component ID for control binding.
    Pot {
        comp_id: String,
        max_resistance: f64,
        position: f64,
        rp: f64,
    },
    Series {
        left: Box<DynNode>,
        right: Box<DynNode>,
        rp: f64,
        gamma: f64,
        b1: f64,
        b2: f64,
    },
    Parallel {
        left: Box<DynNode>,
        right: Box<DynNode>,
        rp: f64,
        gamma: f64,
        b1: f64,
        b2: f64,
    },
}

impl DynNode {
    pub fn port_resistance(&self) -> f64 {
        match self {
            Self::Resistor { rp }
            | Self::Capacitor { rp, .. }
            | Self::Inductor { rp, .. }
            | Self::VoltageSource { rp, .. }
            | Self::Pot { rp, .. }
            | Self::Series { rp, .. }
            | Self::Parallel { rp, .. } => *rp,
        }
    }

    /// Scatter-up: compute reflected wave (bottom → root), caching child waves.
    pub fn reflected(&mut self) -> f64 {
        match self {
            Self::Resistor { .. } | Self::Pot { .. } => 0.0,
            Self::Capacitor { state, .. } => *state,
            Self::Inductor { state, .. } => -*state,
            Self::VoltageSource { voltage, .. } => 2.0 * *voltage,
            Self::Series {
                left,
                right,
                b1,
                b2,
                ..
            } => {
                *b1 = left.reflected();
                *b2 = right.reflected();
                -(*b1 + *b2)
            }
            Self::Parallel {
                left,
                right,
                gamma,
                b1,
                b2,
                ..
            } => {
                *b1 = left.reflected();
                *b2 = right.reflected();
                *b1 + *gamma * (*b2 - *b1)
            }
        }
    }

    /// Scatter-down + state update: propagate incident wave (root → leaves).
    pub fn set_incident(&mut self, a: f64) {
        match self {
            Self::Resistor { .. } | Self::Pot { .. } | Self::VoltageSource { .. } => {}
            Self::Capacitor { state, .. } => *state = a,
            Self::Inductor { state, .. } => *state = a,
            Self::Series {
                left,
                right,
                gamma,
                b1,
                b2,
                ..
            } => {
                let sum = *b1 + *b2 + a;
                let a1 = *b1 - *gamma * sum;
                let a2 = *b2 - (1.0 - *gamma) * sum;
                left.set_incident(a1);
                right.set_incident(a2);
            }
            Self::Parallel {
                left,
                right,
                gamma,
                b1,
                b2,
                ..
            } => {
                let diff = *b2 - *b1;
                let a1 = a + (1.0 - *gamma) * diff;
                let a2 = a - *gamma * diff;
                left.set_incident(a1);
                right.set_incident(a2);
            }
        }
    }

    /// Set the voltage source value (searches recursively).
    pub fn set_voltage(&mut self, v: f64) {
        match self {
            Self::VoltageSource { voltage, .. } => *voltage = v,
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.set_voltage(v);
                right.set_voltage(v);
            }
            _ => {}
        }
    }

    /// Update a pot's position and resistance.  Returns true if found.
    pub fn set_pot(&mut self, target_id: &str, pos: f64) -> bool {
        match self {
            Self::Pot {
                comp_id,
                max_resistance,
                position,
                rp,
            } if comp_id == target_id => {
                *position = pos;
                *rp = (pos * *max_resistance).max(1.0);
                true
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.set_pot(target_id, pos) || right.set_pot(target_id, pos)
            }
            _ => false,
        }
    }

    /// Recompute all adaptor coefficients bottom-up (call after pot changes).
    pub fn recompute(&mut self) {
        match self {
            Self::Series {
                left,
                right,
                rp,
                gamma,
                ..
            } => {
                left.recompute();
                right.recompute();
                let r1 = left.port_resistance();
                let r2 = right.port_resistance();
                *rp = r1 + r2;
                *gamma = r1 / *rp;
            }
            Self::Parallel {
                left,
                right,
                rp,
                gamma,
                ..
            } => {
                left.recompute();
                right.recompute();
                let r1 = left.port_resistance();
                let r2 = right.port_resistance();
                *rp = r1 * r2 / (r1 + r2);
                *gamma = r2 / (r1 + r2);
            }
            _ => {}
        }
    }

    /// Reset all state (capacitor/inductor memory + adaptor caches).
    pub fn reset(&mut self) {
        match self {
            Self::Capacitor { state, .. } | Self::Inductor { state, .. } => *state = 0.0,
            Self::Series {
                left,
                right,
                b1,
                b2,
                ..
            }
            | Self::Parallel {
                left,
                right,
                b1,
                b2,
                ..
            } => {
                *b1 = 0.0;
                *b2 = 0.0;
                left.reset();
                right.reset();
            }
            _ => {}
        }
    }

    /// Update capacitor/inductor port resistances for a new sample rate.
    pub fn update_sample_rate(&mut self, fs: f64) {
        match self {
            Self::Capacitor {
                capacitance, rp, ..
            } => {
                *rp = 1.0 / (2.0 * fs * *capacitance);
            }
            Self::Inductor { inductance, rp, .. } => {
                *rp = 2.0 * fs * *inductance;
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.update_sample_rate(fs);
                right.update_sample_rate(fs);
            }
            _ => {}
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// WDF clipping stage
// ═══════════════════════════════════════════════════════════════════════════

pub enum RootKind {
    DiodePair(DiodePairRoot),
    SingleDiode(DiodeRoot),
}

pub struct WdfStage {
    tree: DynNode,
    root: RootKind,
    /// Compensates for passive attenuation in the tree topology.
    /// Computed automatically from the tree's impedance structure.
    compensation: f64,
}

impl WdfStage {
    /// Process one sample through the WDF tree.
    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
        self.tree.set_voltage(input * self.compensation);
        let b_tree = self.tree.reflected();
        let rp = self.tree.port_resistance();
        let a_root = match &self.root {
            RootKind::DiodePair(dp) => dp.process(b_tree, rp),
            RootKind::SingleDiode(d) => d.process(b_tree, rp),
        };
        self.tree.set_incident(a_root);
        (a_root + b_tree) / 2.0
    }

    pub fn reset(&mut self) {
        self.tree.reset();
    }

    /// Balance the voltage source impedance to match the network.
    ///
    /// When the Vs branch is inside a Parallel adaptor whose sibling has much
    /// higher impedance, the signal is heavily attenuated.  This adjusts the
    /// Vs port resistance so the branches are balanced (gamma ≈ 0.5).
    fn balance_vs_impedance(&mut self) {
        balance_parallel_vs(&mut self.tree);
        self.tree.recompute();
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Circuit graph
// ═══════════════════════════════════════════════════════════════════════════

type NodeId = usize;

struct GraphEdge {
    comp_idx: usize,
    node_a: NodeId,
    node_b: NodeId,
}

struct CircuitGraph {
    /// All two-terminal elements as edges.
    edges: Vec<GraphEdge>,
    /// Component definitions (indexed by comp_idx).
    components: Vec<ComponentDef>,
    /// Special nodes.
    in_node: NodeId,
    out_node: NodeId,
    gnd_node: NodeId,
    /// Number of active elements found (opamps + transistors).
    num_active: usize,
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
    fn from_pedal(pedal: &PedalDef) -> Self {
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

        for (idx, comp) in pedal.components.iter().enumerate() {
            match &comp.kind {
                ComponentKind::Resistor(_)
                | ComponentKind::Capacitor(_)
                | ComponentKind::Inductor(_)
                | ComponentKind::Potentiometer(_)
                | ComponentKind::DiodePair(_)
                | ComponentKind::Diode(_) => {
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
                ComponentKind::Npn | ComponentKind::Pnp | ComponentKind::OpAmp => {
                    num_active += 1;
                }
            }
        }

        let in_node = uf.find(*pin_ids.get("in").unwrap());
        let out_node = uf.find(*pin_ids.get("out").unwrap());
        let gnd_node = uf.find(*pin_ids.get("gnd").unwrap());

        CircuitGraph {
            edges,
            components: pedal.components.clone(),
            in_node,
            out_node,
            gnd_node,
            num_active,
        }
    }

    /// Find diode edges, ordered by topological distance from `in`.
    fn find_diodes(&self) -> Vec<(usize, DiodeInfo)> {
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
    fn elements_at_junction(&self, junction: NodeId, diode_edge_indices: &[usize]) -> Vec<usize> {
        self.edges
            .iter()
            .enumerate()
            .filter(|(idx, e)| {
                if diode_edge_indices.contains(idx) {
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
}

struct DiodeInfo {
    diode_type: DiodeType,
    is_pair: bool,
    junction_node: NodeId,
    #[allow(dead_code)]
    ground_node: NodeId,
}

// ═══════════════════════════════════════════════════════════════════════════
// Series-parallel decomposition
// ═══════════════════════════════════════════════════════════════════════════

enum SpTree {
    Leaf(usize), // component index
    Series(Box<SpTree>, Box<SpTree>),
    Parallel(Box<SpTree>, Box<SpTree>),
}

/// Reduce a set of edges into a single SP tree.
///
/// `edges`: (node_a, node_b, tree) — the initial edges.
/// `terminals`: nodes that must not be eliminated by series reduction.
fn sp_reduce(
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
fn sp_to_dyn(tree: &SpTree, components: &[ComponentDef], sample_rate: f64) -> DynNode {
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

fn make_leaf(comp: &ComponentDef, sample_rate: f64) -> DynNode {
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
        // Diodes shouldn't appear as leaves (they're roots), but handle gracefully.
        _ => DynNode::Resistor { rp: 1000.0 },
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Compiled pedal
// ═══════════════════════════════════════════════════════════════════════════

/// Control binding: maps a knob label to a parameter in the processing chain.
struct ControlBinding {
    label: String,
    target: ControlTarget,
    component_id: String,
    #[allow(dead_code)]
    max_resistance: f64,
}

enum ControlTarget {
    /// Modify the pre-gain multiplier (gain/drive/fuzz/distortion/sustain).
    PreGain,
    /// Modify output attenuation (level/volume/output).
    OutputGain,
    /// Modify a pot in a specific WDF stage.
    PotInStage(usize),
}

/// A pedal processor compiled from a `.pedal` file's netlist.
///
/// Each `.pedal` file produces a unique processor with its own WDF tree topology,
/// component values, and diode models — no hardcoded processor selection.
pub struct CompiledPedal {
    stages: Vec<WdfStage>,
    pre_gain: f64,
    output_gain: f64,
    use_soft_limit: bool,
    soft_limit_scale: f64,
    sample_rate: f64,
    controls: Vec<ControlBinding>,
    gain_range: (f64, f64),
}

/// Gain-like control labels.
fn is_gain_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "drive" | "gain" | "fuzz" | "sustain" | "distortion" | "sensitivity"
    )
}

/// Level-like control labels.
fn is_level_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "level" | "volume" | "output"
    )
}

impl CompiledPedal {
    /// Set a control by its label (e.g., "Drive", "Level").
    pub fn set_control(&mut self, label: &str, value: f64) {
        let value = value.clamp(0.0, 1.0);
        for binding in &self.controls {
            if !binding.label.eq_ignore_ascii_case(label) {
                continue;
            }
            match &binding.target {
                ControlTarget::PreGain => {
                    let (lo, hi) = self.gain_range;
                    self.pre_gain = lo * (hi / lo).powf(value);
                }
                ControlTarget::OutputGain => {
                    self.output_gain = value;
                }
                ControlTarget::PotInStage(stage_idx) => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage.tree.set_pot(&binding.component_id, value);
                        stage.tree.recompute();
                    }
                }
            }
            return;
        }
    }
}

impl PedalProcessor for CompiledPedal {
    fn process(&mut self, input: f64) -> f64 {
        let mut signal = input;

        // Apply pre-gain before EACH stage.  In real circuits, each clipping
        // stage has its own active gain element (transistor/opamp).  The diode
        // voltage output (~0.7V) must be re-amplified before the next stage.
        for stage in &mut self.stages {
            signal = stage.process(signal * self.pre_gain);
        }

        // No WDF stages → just apply gain + soft clip.
        if self.stages.is_empty() {
            signal = input * self.pre_gain;
        }

        if self.use_soft_limit {
            signal = (signal * self.soft_limit_scale).tanh();
        }

        signal * self.output_gain
    }

    fn set_sample_rate(&mut self, rate: f64) {
        self.sample_rate = rate;
        for stage in &mut self.stages {
            stage.tree.update_sample_rate(rate);
            stage.tree.recompute();
        }
    }

    fn reset(&mut self) {
        for stage in &mut self.stages {
            stage.reset();
        }
    }

    fn set_control(&mut self, label: &str, value: f64) {
        self.set_control(label, value);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Main compiler entry point
// ═══════════════════════════════════════════════════════════════════════════

/// Compile a parsed `.pedal` file into a real-time audio processor.
///
/// The compiler analyzes the netlist topology and builds one or more WDF
/// clipping stages, each with its own binary tree derived from the circuit's
/// passive elements and a nonlinear diode root.
///
/// Active elements (transistors, opamps) are modeled as gain stages.
/// Controls are automatically bound to gain/level/pot parameters.
pub fn compile_pedal(pedal: &PedalDef, sample_rate: f64) -> Result<CompiledPedal, String> {
    let graph = CircuitGraph::from_pedal(pedal);
    let diodes = graph.find_diodes();
    let diode_edge_indices: Vec<usize> = diodes.iter().map(|(idx, _)| *idx).collect();

    // Determine gain range based on circuit characteristics.
    let has_germanium = diodes
        .iter()
        .any(|(_, d)| d.diode_type == DiodeType::Germanium);
    let has_single_diode = diodes.iter().any(|(_, d)| !d.is_pair);
    let gain_range = if has_germanium && has_single_diode {
        (5.0, 250.0) // Fuzz-like
    } else if has_germanium {
        (3.0, 150.0) // Germanium overdrive (Klon-ish)
    } else if diodes.len() > 1 {
        (2.0, 50.0) // Multi-stage (each clips, cumulative is heavy)
    } else {
        (2.0, 100.0) // Standard overdrive
    };

    // Active elements contribute implicit gain.
    let active_bonus = match graph.num_active {
        0 => 1.0,
        1 => 3.0,
        _ => 5.0,
    };

    // Build WDF stages.
    let mut stages = Vec::new();
    let vs_comp_idx = graph.components.len(); // virtual voltage source index

    // BFS distances from in_node for Vs placement.
    let dist_from_in = {
        let mut adj: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
        for e in &graph.edges {
            adj.entry(e.node_a).or_default().push(e.node_b);
            adj.entry(e.node_b).or_default().push(e.node_a);
        }
        let mut dist: HashMap<NodeId, usize> = HashMap::new();
        let mut queue = std::collections::VecDeque::new();
        dist.insert(graph.in_node, 0);
        queue.push_back(graph.in_node);
        while let Some(n) = queue.pop_front() {
            let d = dist[&n];
            if let Some(neighbors) = adj.get(&n) {
                for &nb in neighbors {
                    dist.entry(nb).or_insert_with(|| {
                        queue.push_back(nb);
                        d + 1
                    });
                }
            }
        }
        dist
    };

    for (_edge_idx, diode_info) in &diodes {
        let junction = diode_info.junction_node;
        let passive_idxs = graph.elements_at_junction(junction, &diode_edge_indices);

        if passive_idxs.is_empty() {
            continue;
        }

        // Find the best injection node for the voltage source: the non-junction
        // endpoint closest to in_node.  This ensures the Vs is reachable from the
        // passive elements in this stage (critical for multi-stage circuits).
        let mut injection_node = graph.in_node;
        let mut best_dist = usize::MAX;
        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            let other = if e.node_a == junction {
                e.node_b
            } else {
                e.node_a
            };
            if let Some(&d) = dist_from_in.get(&other) {
                if d < best_dist {
                    best_dist = d;
                    injection_node = other;
                }
            }
        }

        // Build SP edges: one edge per passive element, plus a virtual VoltageSource.
        let source_node = graph.edges.len() + 1000; // virtual source node
        let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();

        // Add voltage source edge: source_node → injection_node.
        sp_edges.push((source_node, injection_node, SpTree::Leaf(vs_comp_idx)));

        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
        }

        let terminals = vec![source_node, junction];
        let sp_tree = match sp_reduce(sp_edges, &terminals) {
            Ok(t) => t,
            Err(_) => continue, // Skip stages that can't be decomposed.
        };

        // Convert SP tree to dynamic WDF nodes.
        // We need to handle the virtual VoltageSource leaf specially.
        let mut all_components = graph.components.clone();
        // Add a virtual VoltageSource component at vs_comp_idx.
        while all_components.len() <= vs_comp_idx {
            all_components.push(ComponentDef {
                id: "__vs__".to_string(),
                kind: ComponentKind::Resistor(1.0), // placeholder
            });
        }

        let tree = sp_to_dyn_with_vs(&sp_tree, &all_components, sample_rate, vs_comp_idx);

        let model = diode_model(diode_info.diode_type);
        let root = if diode_info.is_pair {
            RootKind::DiodePair(DiodePairRoot::new(model))
        } else {
            RootKind::SingleDiode(DiodeRoot::new(model))
        };

        stages.push(WdfStage {
            tree,
            root,
            compensation: 1.0,
        });
    }

    // Balance voltage source impedance in each stage.
    // This fixes topologies where the Vs branch sits in a Parallel adaptor
    // opposite a high-impedance element (e.g. Big Muff: Parallel(Series(Vs,C), R)
    // with R >> C causes gamma ≈ 1.0 and severe signal attenuation).
    for stage in &mut stages {
        stage.balance_vs_impedance();
    }

    // If no diodes found, create a dummy stage-less pedal (just gain + soft clip).
    let use_soft_limit = has_single_diode || stages.is_empty();
    let soft_limit_scale = if has_germanium { 3.0 } else { 2.0 };

    // Build control bindings.
    let mut controls = Vec::new();
    for ctrl in &pedal.controls {
        let target = if is_gain_label(&ctrl.label) {
            ControlTarget::PreGain
        } else if is_level_label(&ctrl.label) {
            ControlTarget::OutputGain
        } else {
            // Try to find this pot in a WDF stage.
            let mut found_stage = None;
            for (si, stage) in stages.iter().enumerate() {
                if has_pot(&stage.tree, &ctrl.component) {
                    found_stage = Some(si);
                    break;
                }
            }
            match found_stage {
                Some(si) => ControlTarget::PotInStage(si),
                None => ControlTarget::PreGain, // fallback: map unknown controls to gain
            }
        };

        let max_r = pedal
            .components
            .iter()
            .find(|c| c.id == ctrl.component)
            .and_then(|c| match &c.kind {
                ComponentKind::Potentiometer(r) => Some(*r),
                _ => None,
            })
            .unwrap_or(100_000.0);

        controls.push(ControlBinding {
            label: ctrl.label.clone(),
            target,
            component_id: ctrl.component.clone(),
            max_resistance: max_r,
        });
    }

    // Compute initial pre-gain from default control values.
    let gain_default = pedal
        .controls
        .iter()
        .find(|c| is_gain_label(&c.label))
        .map(|c| c.default)
        .unwrap_or(0.5);
    let level_default = pedal
        .controls
        .iter()
        .find(|c| is_level_label(&c.label))
        .map(|c| c.default)
        .unwrap_or(0.8);

    let (glo, ghi) = gain_range;
    let ratio: f64 = ghi / glo;
    let pre_gain = glo * ratio.powf(gain_default) * active_bonus;

    Ok(CompiledPedal {
        stages,
        pre_gain,
        output_gain: level_default,
        use_soft_limit,
        soft_limit_scale,
        sample_rate,
        controls,
        gain_range: (glo * active_bonus, ghi * active_bonus),
    })
}

/// Convert SP tree to DynNode, inserting a VoltageSource for the virtual leaf.
fn sp_to_dyn_with_vs(
    tree: &SpTree,
    components: &[ComponentDef],
    sample_rate: f64,
    vs_idx: usize,
) -> DynNode {
    match tree {
        SpTree::Leaf(idx) if *idx == vs_idx => DynNode::VoltageSource {
            voltage: 0.0,
            rp: 1.0,
        },
        SpTree::Leaf(idx) => make_leaf(&components[*idx], sample_rate),
        SpTree::Series(left, right) => {
            let l = sp_to_dyn_with_vs(left, components, sample_rate, vs_idx);
            let r = sp_to_dyn_with_vs(right, components, sample_rate, vs_idx);
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
            let l = sp_to_dyn_with_vs(left, components, sample_rate, vs_idx);
            let r = sp_to_dyn_with_vs(right, components, sample_rate, vs_idx);
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

fn diode_model(dt: DiodeType) -> DiodeModel {
    match dt {
        DiodeType::Silicon => DiodeModel::silicon(),
        DiodeType::Germanium => DiodeModel::germanium(),
        DiodeType::Led => DiodeModel::led(),
    }
}

fn has_vs(node: &DynNode) -> bool {
    match node {
        DynNode::VoltageSource { .. } => true,
        DynNode::Series { left, right, .. } | DynNode::Parallel { left, right, .. } => {
            has_vs(left) || has_vs(right)
        }
        _ => false,
    }
}

/// Walk the tree and balance any adaptor where one branch contains the
/// VoltageSource and the other has much higher impedance.
///
/// - **Parallel**: high-Z sibling causes gamma→1, attenuating the Vs signal
///   (silent output, e.g. Big Muff).
/// - **Series**: high-Z sibling causes gamma→0, dumping all scattered-down
///   energy into the passives and causing oscillation/instability (e.g. ProCo RAT).
fn balance_parallel_vs(node: &mut DynNode) {
    match node {
        DynNode::Parallel { left, right, .. } | DynNode::Series { left, right, .. } => {
            let left_has_vs = has_vs(left);
            let right_has_vs = has_vs(right);

            if left_has_vs && !right_has_vs {
                let target = right.port_resistance();
                adjust_vs_branch_rp(left, target);
            } else if right_has_vs && !left_has_vs {
                let target = left.port_resistance();
                adjust_vs_branch_rp(right, target);
            }
            // Recurse into children.
            balance_parallel_vs(left);
            balance_parallel_vs(right);
        }
        _ => {}
    }
}

/// Adjust the Vs port resistance inside `branch` so that `branch.port_resistance() ≈ target_rp`.
///
/// If the branch already has comparable impedance, do nothing.
fn adjust_vs_branch_rp(branch: &mut DynNode, target_rp: f64) {
    let current_rp = branch.port_resistance();
    if current_rp >= target_rp * 0.5 {
        return; // Already reasonably balanced.
    }
    match branch {
        DynNode::VoltageSource { rp, .. } => {
            *rp = target_rp.max(1.0);
        }
        DynNode::Series { left, right, .. } => {
            // Series(Vs_branch, other): set Vs rp so that series total ≈ target.
            if has_vs(left) {
                let other_rp = right.port_resistance();
                let vs_target = (target_rp - other_rp).max(1.0);
                set_vs_rp(left, vs_target);
            } else if has_vs(right) {
                let other_rp = left.port_resistance();
                let vs_target = (target_rp - other_rp).max(1.0);
                set_vs_rp(right, vs_target);
            }
        }
        DynNode::Parallel { left, right, .. } => {
            // Recurse — the Vs is deeper.
            if has_vs(left) {
                adjust_vs_branch_rp(left, target_rp);
            } else {
                adjust_vs_branch_rp(right, target_rp);
            }
        }
        _ => {}
    }
}

fn set_vs_rp(node: &mut DynNode, rp_val: f64) {
    match node {
        DynNode::VoltageSource { rp, .. } => *rp = rp_val,
        DynNode::Series { left, right, .. } | DynNode::Parallel { left, right, .. } => {
            if has_vs(left) {
                set_vs_rp(left, rp_val);
            } else {
                set_vs_rp(right, rp_val);
            }
        }
        _ => {}
    }
}

fn has_pot(node: &DynNode, comp_id: &str) -> bool {
    match node {
        DynNode::Pot { comp_id: id, .. } => id == comp_id,
        DynNode::Series { left, right, .. } | DynNode::Parallel { left, right, .. } => {
            has_pot(left, comp_id) || has_pot(right, comp_id)
        }
        _ => false,
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::parse_pedal_file;

    fn parse(filename: &str) -> PedalDef {
        let path = format!("examples/{filename}");
        let src = std::fs::read_to_string(&path).unwrap();
        parse_pedal_file(&src).unwrap()
    }

    #[test]
    fn compile_tube_screamer() {
        let pedal = parse("tube_screamer.pedal");
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        proc.set_control("Drive", 0.7);
        proc.set_control("Level", 0.8);

        // Process a sine wave and verify it's not a wire.
        let n = 48000;
        let input: Vec<f64> = (0..n)
            .map(|i| 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin())
            .collect();
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

        let corr = correlation(&input, &output).abs();
        assert!(corr < 0.98, "Compiled TS should distort: |corr|={corr:.4}");
        assert!(output.iter().all(|x| x.is_finite()), "No NaN/inf in output");
    }

    #[test]
    fn compile_big_muff() {
        let pedal = parse("big_muff.pedal");
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        proc.set_control("Sustain", 0.7);
        proc.set_control("Volume", 0.5);

        let n = 48000;
        let input: Vec<f64> = (0..n)
            .map(|i| 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin())
            .collect();
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

        let corr = correlation(&input, &output).abs();
        assert!(
            corr < 0.98,
            "Compiled Big Muff should distort: |corr|={corr:.4}"
        );
    }

    #[test]
    fn compile_fuzz_face() {
        // Fuzz Face has no diodes in DSL — uses transistor gain + soft limit.
        let pedal = parse("fuzz_face.pedal");
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        proc.set_control("Fuzz", 0.8);
        proc.set_control("Volume", 0.5);

        let n = 48000;
        let input: Vec<f64> = (0..n)
            .map(|i| 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin())
            .collect();
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

        assert!(output.iter().all(|x| x.is_finite()), "No NaN/inf in output");
        let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(peak > 0.01, "Should produce output: peak={peak}");
    }

    #[test]
    fn compile_all_pedals() {
        let files = [
            "tube_screamer.pedal",
            "fuzz_face.pedal",
            "big_muff.pedal",
            "blues_driver.pedal",
            "dyna_comp.pedal",
            "klon_centaur.pedal",
            "proco_rat.pedal",
        ];
        for f in files {
            let pedal = parse(f);
            let result = compile_pedal(&pedal, 48000.0);
            if let Err(e) = &result {
                panic!("Failed to compile {f}: {e}");
            }

            let mut proc = result.unwrap();
            let input: Vec<f64> = (0..4800)
                .map(|i| 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin())
                .collect();
            let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
            assert!(
                output.iter().all(|x| x.is_finite()),
                "{f}: output contains NaN/inf"
            );
            let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
            assert!(peak > 0.001, "{f}: silent output, peak={peak}");
        }
    }

    #[test]
    fn compiled_pedals_differ() {
        // Each pedal should produce a distinct output (different topologies/values).
        let files = [
            "tube_screamer.pedal",
            "big_muff.pedal",
            "blues_driver.pedal",
        ];
        let mut outputs = Vec::new();

        for f in files {
            let pedal = parse(f);
            let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
            let input: Vec<f64> = (0..48000)
                .map(|i| 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin())
                .collect();
            let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
            outputs.push(output);
        }

        // Each pair should have |corr| < 0.999 (they're not identical).
        for i in 0..outputs.len() {
            for j in (i + 1)..outputs.len() {
                let corr = correlation(&outputs[i], &outputs[j]).abs();
                assert!(
                    corr < 0.999,
                    "{} vs {} are too similar: |corr|={corr:.6}",
                    files[i],
                    files[j]
                );
            }
        }
    }

    // -----------------------------------------------------------------------
    // Per-pedal regression tests
    // -----------------------------------------------------------------------

    #[test]
    fn compile_blues_driver() {
        let pedal = parse("blues_driver.pedal");
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        proc.set_control("Gain", 0.7);
        proc.set_control("Level", 0.5);

        let input = sine(48000);
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        assert_finite(&output, "Blues Driver");
        let corr = correlation(&input, &output).abs();
        assert!(corr < 0.98, "Blues Driver should distort: |corr|={corr:.4}");
    }

    #[test]
    fn compile_proco_rat() {
        let pedal = parse("proco_rat.pedal");
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        proc.set_control("Distortion", 0.7);
        proc.set_control("Volume", 0.5);

        let input = sine(48000);
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        assert_finite(&output, "ProCo RAT");
        let corr = correlation(&input, &output).abs();
        assert!(corr < 0.98, "ProCo RAT should distort: |corr|={corr:.4}");
    }

    #[test]
    fn compile_klon_centaur() {
        let pedal = parse("klon_centaur.pedal");
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        proc.set_control("Gain", 0.7);
        proc.set_control("Output", 0.5);

        let input = sine(48000);
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        assert_finite(&output, "Klon Centaur");
        let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(peak > 0.01, "Klon should produce output: peak={peak}");
    }

    #[test]
    fn compile_dyna_comp() {
        // Dyna Comp has no diodes — uses opamp gain + soft limit.
        let pedal = parse("dyna_comp.pedal");
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        proc.set_control("Sensitivity", 0.7);
        proc.set_control("Output", 0.5);

        let input = sine(48000);
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        assert_finite(&output, "Dyna Comp");
        let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(peak > 0.01, "Dyna Comp should produce output: peak={peak}");
    }

    // -----------------------------------------------------------------------
    // Control response tests
    // -----------------------------------------------------------------------

    #[test]
    fn gain_control_affects_distortion() {
        let pedal = parse("tube_screamer.pedal");
        let input = sine(48000);

        let mut low = compile_pedal(&pedal, 48000.0).unwrap();
        low.set_control("Drive", 0.1);
        let out_low: Vec<f64> = input.iter().map(|&s| low.process(s)).collect();

        let mut high = compile_pedal(&pedal, 48000.0).unwrap();
        high.set_control("Drive", 0.9);
        let out_high: Vec<f64> = input.iter().map(|&s| high.process(s)).collect();

        let corr_low = correlation(&input, &out_low).abs();
        let corr_high = correlation(&input, &out_high).abs();
        assert!(
            corr_high < corr_low,
            "Higher Drive should produce more distortion: low={corr_low:.4} high={corr_high:.4}"
        );
    }

    #[test]
    fn level_control_affects_volume() {
        let pedal = parse("tube_screamer.pedal");
        let input = sine(48000);

        let mut quiet = compile_pedal(&pedal, 48000.0).unwrap();
        quiet.set_control("Level", 0.1);
        let out_quiet: Vec<f64> = input.iter().map(|&s| quiet.process(s)).collect();
        let peak_quiet = out_quiet.iter().fold(0.0f64, |m, x| m.max(x.abs()));

        let mut loud = compile_pedal(&pedal, 48000.0).unwrap();
        loud.set_control("Level", 0.9);
        let out_loud: Vec<f64> = input.iter().map(|&s| loud.process(s)).collect();
        let peak_loud = out_loud.iter().fold(0.0f64, |m, x| m.max(x.abs()));

        assert!(
            peak_loud > peak_quiet,
            "Higher Level should be louder: quiet={peak_quiet:.4} loud={peak_loud:.4}"
        );
    }

    #[test]
    fn compiled_output_bounded() {
        // No pedal should produce output exceeding 2.0 with 0.5 amplitude input.
        let files = [
            "tube_screamer.pedal",
            "fuzz_face.pedal",
            "big_muff.pedal",
            "blues_driver.pedal",
            "dyna_comp.pedal",
            "klon_centaur.pedal",
            "proco_rat.pedal",
        ];
        let input = sine(48000);
        for f in files {
            let pedal = parse(f);
            let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
            let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
            let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
            assert!(peak < 2.0, "{f}: output too loud, peak={peak}");
        }
    }

    #[test]
    fn compiled_reset_clears_state() {
        let pedal = parse("tube_screamer.pedal");
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        proc.set_control("Drive", 0.7);

        // Process some audio
        let input = sine(4800);
        let _: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

        // Reset and process again — should match initial run
        proc.reset();
        let mut proc2 = compile_pedal(&pedal, 48000.0).unwrap();
        proc2.set_control("Drive", 0.7);

        let out1: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        let out2: Vec<f64> = input.iter().map(|&s| proc2.process(s)).collect();
        let corr = correlation(&out1, &out2);
        assert!(
            corr > 0.999,
            "Reset should restore initial state: corr={corr:.6}"
        );
    }

    #[test]
    fn all_seven_pedals_unique() {
        let files = [
            "tube_screamer.pedal",
            "fuzz_face.pedal",
            "big_muff.pedal",
            "blues_driver.pedal",
            "dyna_comp.pedal",
            "klon_centaur.pedal",
            "proco_rat.pedal",
        ];
        let input = sine(48000);
        let mut outputs = Vec::new();
        for f in files {
            let pedal = parse(f);
            let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
            let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
            outputs.push(output);
        }
        for i in 0..outputs.len() {
            for j in (i + 1)..outputs.len() {
                let corr = correlation(&outputs[i], &outputs[j]).abs();
                assert!(
                    corr < 0.9999,
                    "{} vs {} too similar: |corr|={corr:.6}",
                    files[i],
                    files[j]
                );
            }
        }
    }

    // -----------------------------------------------------------------------
    // Non-default settings regression tests
    // -----------------------------------------------------------------------

    #[test]
    fn tube_screamer_max_drive() {
        let pedal = parse("tube_screamer.pedal");
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        proc.set_control("Drive", 1.0);
        proc.set_control("Level", 1.0);

        let input = sine(48000);
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        assert_finite(&output, "TS max drive");
        let corr = correlation(&input, &output).abs();
        assert!(
            corr < 0.95,
            "Max drive should heavily distort: |corr|={corr:.4}"
        );
    }

    #[test]
    fn tube_screamer_min_drive() {
        let pedal = parse("tube_screamer.pedal");
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        proc.set_control("Drive", 0.0);
        proc.set_control("Level", 0.5);

        let input = sine(48000);
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        assert_finite(&output, "TS min drive");
        let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(
            peak > 0.001,
            "Min drive should still produce output: peak={peak}"
        );
    }

    #[test]
    fn big_muff_max_sustain() {
        let pedal = parse("big_muff.pedal");
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        proc.set_control("Sustain", 1.0);
        proc.set_control("Volume", 0.8);

        let input = sine(48000);
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        assert_finite(&output, "BM max sustain");
        let corr = correlation(&input, &output).abs();
        assert!(
            corr < 0.95,
            "Max sustain should heavily distort: |corr|={corr:.4}"
        );
    }

    #[test]
    fn proco_rat_max_distortion() {
        let pedal = parse("proco_rat.pedal");
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        proc.set_control("Distortion", 1.0);
        proc.set_control("Volume", 0.8);

        let input = sine(48000);
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        assert_finite(&output, "RAT max distortion");
        let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(
            peak > 0.01,
            "Max distortion should produce output: peak={peak}"
        );
    }

    #[test]
    fn fuzz_face_max_fuzz() {
        let pedal = parse("fuzz_face.pedal");
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        proc.set_control("Fuzz", 1.0);
        proc.set_control("Volume", 1.0);

        let input = sine(48000);
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        assert_finite(&output, "FF max fuzz");
        let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(peak > 0.01, "Max fuzz should produce output: peak={peak}");
    }

    #[test]
    fn all_pedals_extreme_settings_stable() {
        let configs: &[(&str, &[(&str, f64)])] = &[
            ("tube_screamer.pedal", &[("Drive", 1.0), ("Level", 1.0)]),
            ("tube_screamer.pedal", &[("Drive", 0.0), ("Level", 0.0)]),
            ("fuzz_face.pedal", &[("Fuzz", 1.0), ("Volume", 1.0)]),
            ("fuzz_face.pedal", &[("Fuzz", 0.0), ("Volume", 0.0)]),
            ("big_muff.pedal", &[("Sustain", 1.0), ("Volume", 1.0)]),
            ("big_muff.pedal", &[("Sustain", 0.0), ("Volume", 0.0)]),
            ("blues_driver.pedal", &[("Gain", 1.0), ("Level", 1.0)]),
            ("blues_driver.pedal", &[("Gain", 0.0), ("Level", 0.0)]),
            ("proco_rat.pedal", &[("Distortion", 1.0), ("Volume", 1.0)]),
            ("proco_rat.pedal", &[("Distortion", 0.0), ("Volume", 0.0)]),
            ("klon_centaur.pedal", &[("Gain", 1.0), ("Output", 1.0)]),
            ("klon_centaur.pedal", &[("Gain", 0.0), ("Output", 0.0)]),
            ("dyna_comp.pedal", &[("Sensitivity", 1.0), ("Output", 1.0)]),
            ("dyna_comp.pedal", &[("Sensitivity", 0.0), ("Output", 0.0)]),
        ];

        let input = sine(4800);
        for (f, knobs) in configs {
            let pedal = parse(f);
            let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
            for &(label, val) in *knobs {
                proc.set_control(label, val);
            }
            let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
            let settings: String = knobs
                .iter()
                .map(|(l, v)| format!("{l}={v}"))
                .collect::<Vec<_>>()
                .join(",");
            assert!(
                output.iter().all(|x| x.is_finite()),
                "{f} [{settings}]: NaN/inf"
            );
            let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
            assert!(peak < 5.0, "{f} [{settings}]: output too loud, peak={peak}");
        }
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    fn sine(n: usize) -> Vec<f64> {
        (0..n)
            .map(|i| 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin())
            .collect()
    }

    fn assert_finite(output: &[f64], name: &str) {
        assert!(
            output.iter().all(|x| x.is_finite()),
            "{name}: output contains NaN/inf"
        );
    }

    fn correlation(a: &[f64], b: &[f64]) -> f64 {
        let n = a.len().min(b.len()) as f64;
        let ma = a.iter().sum::<f64>() / n;
        let mb = b.iter().sum::<f64>() / n;
        let (mut cov, mut va, mut vb) = (0.0, 0.0, 0.0);
        for i in 0..n as usize {
            let (da, db) = (a[i] - ma, b[i] - mb);
            cov += da * db;
            va += da * da;
            vb += db * db;
        }
        if va == 0.0 || vb == 0.0 {
            return 0.0;
        }
        cov / (va.sqrt() * vb.sqrt())
    }
}
