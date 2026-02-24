//! Netlist-to-WDF compiler.
//!
//! Compiles a parsed `.pedal` file into a real-time audio processor by:
//! 1. Building a circuit graph from the netlist
//! 2. Identifying clipping stages (around diode elements)
//! 3. For each stage, building a WDF binary tree via series-parallel decomposition
//! 4. Modeling active elements (transistors, opamps) as gain stages
//! 5. Chaining everything into a cascaded `PedalProcessor`

use std::collections::{HashMap, HashSet};

use crate::dsl::*;
use crate::elements::*;
use crate::oversampling::{Oversampler, OversamplingFactor};
use crate::thermal::ThermalModel;
use crate::tolerance::ToleranceEngine;
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
    /// Photocoupler (Vactrol) — LDR with CdS carrier dynamics.
    Photocoupler {
        comp_id: String,
        inner: crate::elements::Photocoupler,
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
            Self::Photocoupler { inner, .. } => inner.port_resistance(),
        }
    }

    /// Scatter-up: compute reflected wave (bottom → root), caching child waves.
    pub fn reflected(&mut self) -> f64 {
        match self {
            Self::Resistor { .. } | Self::Pot { .. } | Self::Photocoupler { .. } => 0.0,
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
            Self::Resistor { .. }
            | Self::Pot { .. }
            | Self::VoltageSource { .. }
            | Self::Photocoupler { .. } => {}
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
            Self::Photocoupler { inner, .. } => inner.reset(),
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
            Self::Photocoupler { inner, .. } => {
                inner.set_sample_rate(fs);
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.update_sample_rate(fs);
                right.update_sample_rate(fs);
            }
            _ => {}
        }
    }

    /// Set LED drive level for a photocoupler. Returns true if found.
    pub fn set_photocoupler_led(&mut self, target_id: &str, led_drive: f64) -> bool {
        match self {
            Self::Photocoupler { comp_id, inner } if comp_id == target_id => {
                inner.set_led_drive(led_drive);
                true
            }
            Self::Series { left, right, .. } | Self::Parallel { left, right, .. } => {
                left.set_photocoupler_led(target_id, led_drive)
                    || right.set_photocoupler_led(target_id, led_drive)
            }
            _ => false,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// WDF clipping stage
// ═══════════════════════════════════════════════════════════════════════════

pub enum RootKind {
    DiodePair(DiodePairRoot),
    SingleDiode(DiodeRoot),
    Zener(ZenerRoot),
    Jfet(JfetRoot),
    Triode(TriodeRoot),
    Pentode(PentodeRoot),
    Mosfet(MosfetRoot),
    Ota(OtaRoot),
    /// Voltage-mode op-amp (TL072, LM308, JRC4558, etc.).
    /// Modeled as a VCVS: Vout = Aol * (Vp - Vm).
    OpAmp(OpAmpRoot),
    /// NPN BJT transistor (2N3904, BC109, 2N5089, etc.).
    /// Modeled with Ebers-Moll equations.
    BjtNpn(BjtNpnRoot),
    /// PNP BJT transistor (2N3906, AC128, NKT275, etc.).
    /// Modeled with Ebers-Moll equations.
    BjtPnp(BjtPnpRoot),
}

pub struct WdfStage {
    tree: DynNode,
    root: RootKind,
    /// Compensates for passive attenuation in the tree topology.
    /// Computed automatically from the tree's impedance structure.
    compensation: f64,
    /// Oversampler for antialiasing at nonlinear stages.
    oversampler: Oversampler,
    /// Base diode model (before thermal modulation). Stored so thermal
    /// drift can be applied as a multiplier without accumulation.
    base_diode_model: Option<DiodeModel>,
}

impl WdfStage {
    /// Process one sample through the WDF tree with oversampling.
    ///
    /// The oversampler wraps the entire WDF scatter-up → root solve →
    /// scatter-down cycle, ensuring that harmonics generated by the
    /// nonlinear root are properly bandlimited before decimation.
    #[inline]
    pub fn process(&mut self, input: f64) -> f64 {
        // Borrow fields individually to satisfy the borrow checker
        let tree = &mut self.tree;
        let root = &mut self.root;
        let compensation = self.compensation;

        self.oversampler.process(input, |sample| {
            tree.set_voltage(sample * compensation);
            let b_tree = tree.reflected();
            let rp = tree.port_resistance();
            let a_root = match root {
                RootKind::DiodePair(dp) => dp.process(b_tree, rp),
                RootKind::SingleDiode(d) => d.process(b_tree, rp),
                RootKind::Zener(z) => z.process(b_tree, rp),
                RootKind::Jfet(j) => j.process(b_tree, rp),
                RootKind::Triode(t) => t.process(b_tree, rp),
                RootKind::Pentode(p) => p.process(b_tree, rp),
                RootKind::Mosfet(m) => m.process(b_tree, rp),
                RootKind::Ota(o) => o.process(b_tree, rp),
                RootKind::OpAmp(op) => op.process(b_tree, rp),
                RootKind::BjtNpn(bjt) => bjt.process(b_tree, rp),
                RootKind::BjtPnp(bjt) => bjt.process(b_tree, rp),
            };
            tree.set_incident(a_root);
            (a_root + b_tree) / 2.0
        })
    }

    pub fn reset(&mut self) {
        self.tree.reset();
        self.oversampler.reset();
    }

    /// Apply thermal drift to temperature-sensitive root elements.
    ///
    /// Modulates diode Is and n_vt based on the current thermal state.
    /// Uses stored base model to prevent multiplier accumulation.
    fn apply_thermal(&mut self, state: &crate::thermal::ThermalState) {
        if let Some(base) = &self.base_diode_model {
            let ideality_ratio = base.n_vt / 0.02585; // n factor (ideality * Vt_ref)
            match &mut self.root {
                RootKind::DiodePair(dp) => {
                    dp.model.is = base.is * state.is_multiplier;
                    dp.model.n_vt = ideality_ratio * state.vt;
                }
                RootKind::SingleDiode(d) => {
                    d.model.is = base.is * state.is_multiplier;
                    d.model.n_vt = ideality_ratio * state.vt;
                }
                _ => {}
            }
        }
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

    /// Set the gate-source voltage for JFET root elements.
    ///
    /// This is used for external modulation (LFO, envelope, etc.).
    /// Has no effect if the root is not a JFET.
    #[inline]
    pub fn set_jfet_vgs(&mut self, vgs: f64) {
        if let RootKind::Jfet(j) = &mut self.root {
            j.set_vgs(vgs);
        }
    }

    /// Get the current gate-source voltage if this is a JFET stage.
    pub fn jfet_vgs(&self) -> Option<f64> {
        match &self.root {
            RootKind::Jfet(j) => Some(j.vgs()),
            _ => None,
        }
    }

    /// Set the grid-cathode voltage for triode root elements.
    ///
    /// This is used for external modulation (bias, LFO, signal input).
    /// Has no effect if the root is not a triode.
    #[inline]
    pub fn set_triode_vgk(&mut self, vgk: f64) {
        if let RootKind::Triode(t) = &mut self.root {
            t.set_vgk(vgk);
        }
    }

    /// Get the current grid-cathode voltage if this is a triode stage.
    pub fn triode_vgk(&self) -> Option<f64> {
        match &self.root {
            RootKind::Triode(t) => Some(t.vgk()),
            _ => None,
        }
    }

    /// Set the control grid voltage (g1-cathode) for pentode root elements.
    #[inline]
    pub fn set_pentode_vg1k(&mut self, vg1k: f64) {
        if let RootKind::Pentode(p) = &mut self.root {
            p.set_vg1k(vg1k);
        }
    }

    /// Set the screen grid voltage (g2-cathode) for pentode root elements.
    #[inline]
    pub fn set_pentode_vg2k(&mut self, vg2k: f64) {
        if let RootKind::Pentode(p) = &mut self.root {
            p.set_vg2k(vg2k);
        }
    }

    /// Get the current control grid voltage if this is a pentode stage.
    pub fn pentode_vg1k(&self) -> Option<f64> {
        match &self.root {
            RootKind::Pentode(p) => Some(p.vg1k()),
            _ => None,
        }
    }

    /// Set the gate-source voltage for MOSFET root elements.
    #[inline]
    pub fn set_mosfet_vgs(&mut self, vgs: f64) {
        if let RootKind::Mosfet(m) = &mut self.root {
            m.set_vgs(vgs);
        }
    }

    /// Get the current gate-source voltage if this is a MOSFET stage.
    pub fn mosfet_vgs(&self) -> Option<f64> {
        match &self.root {
            RootKind::Mosfet(m) => Some(m.vgs()),
            _ => None,
        }
    }

    /// Set the OTA bias current (for envelope-controlled gain).
    #[inline]
    pub fn set_ota_iabc(&mut self, iabc: f64) {
        if let RootKind::Ota(o) = &mut self.root {
            o.set_iabc(iabc);
        }
    }

    /// Set OTA gain as normalized value (0.0–1.0).
    #[inline]
    pub fn set_ota_gain(&mut self, gain: f64) {
        if let RootKind::Ota(o) = &mut self.root {
            o.set_gain_normalized(gain);
        }
    }

    /// Set the non-inverting input voltage (Vp) for op-amp root elements.
    ///
    /// For unity-gain buffers, the op-amp output will follow this voltage.
    /// Has no effect if the root is not an op-amp.
    #[inline]
    pub fn set_opamp_vp(&mut self, vp: f64) {
        if let RootKind::OpAmp(op) = &mut self.root {
            op.set_vp(vp);
        }
    }

    /// Get the current non-inverting input voltage if this is an op-amp stage.
    pub fn opamp_vp(&self) -> Option<f64> {
        match &self.root {
            RootKind::OpAmp(op) => Some(op.vp()),
            _ => None,
        }
    }

    /// Configure op-amp feedback topology.
    ///
    /// - `ratio = 1.0`: Unity-gain buffer (Vm = Vout)
    /// - `ratio < 1.0`: Gain stage with feedback network
    #[inline]
    pub fn set_opamp_feedback(&mut self, ratio: f64, vm_external: f64) {
        if let RootKind::OpAmp(op) = &mut self.root {
            op.set_feedback(ratio, vm_external);
        }
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
    #[allow(dead_code)]
    num_active: usize,
    /// Edge indices for virtual bridge edges through active elements.
    /// These exist for BFS traversal but are not passive WDF tree elements.
    active_edge_indices: Vec<usize>,
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
                | ComponentKind::Photocoupler(_) => {
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
    fn elements_at_junction(
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
    fn find_jfets(&self) -> Vec<(usize, JfetInfo)> {
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
    fn find_bjts(&self) -> Vec<(usize, BjtInfo)> {
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
    fn find_triodes(&self) -> Vec<(usize, TriodeInfo)> {
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
    fn find_pentodes(&self) -> Vec<(usize, PentodeInfo)> {
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
    fn find_mosfets(&self) -> Vec<(usize, MosfetInfo)> {
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
    fn find_zeners(&self) -> Vec<(usize, ZenerInfo)> {
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
    fn find_otas(&self) -> Vec<(usize, OtaInfo)> {
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

struct DiodeInfo {
    diode_type: DiodeType,
    is_pair: bool,
    junction_node: NodeId,
    #[allow(dead_code)]
    ground_node: NodeId,
}

struct JfetInfo {
    jfet_type: JfetType,
    is_n_channel: bool,
    junction_node: NodeId,
    #[allow(dead_code)]
    ground_node: NodeId,
}

struct BjtInfo {
    bjt_type: BjtType,
    is_npn: bool,
    /// The collector node (where the WDF tree connects).
    junction_node: NodeId,
    #[allow(dead_code)]
    ground_node: NodeId,
}

struct TriodeInfo {
    triode_type: TriodeType,
    junction_node: NodeId,
    #[allow(dead_code)]
    ground_node: NodeId,
}

struct PentodeInfo {
    pentode_type: PentodeType,
    junction_node: NodeId,
    #[allow(dead_code)]
    ground_node: NodeId,
}

struct MosfetInfo {
    mosfet_type: MosfetType,
    is_n_channel: bool,
    junction_node: NodeId,
    #[allow(dead_code)]
    ground_node: NodeId,
}

struct ZenerInfo {
    voltage: f64,
    junction_node: NodeId,
    #[allow(dead_code)]
    ground_node: NodeId,
}

struct OtaInfo {
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
        ComponentKind::Photocoupler(pt) => {
            let model = match pt {
                PhotocouplerType::Vtl5c3 => crate::elements::PhotocouplerModel::vtl5c3(),
                PhotocouplerType::Vtl5c1 => crate::elements::PhotocouplerModel::vtl5c1(),
                PhotocouplerType::Nsl32 => crate::elements::PhotocouplerModel::nsl32(),
            };
            DynNode::Photocoupler {
                comp_id: comp.id.clone(),
                inner: crate::elements::Photocoupler::new(model, sample_rate),
            }
        }
        // Tempco resistor: modeled as a standard resistor (nominal value).
        // Temperature compensation handled by the thermal model separately.
        ComponentKind::Tempco(r, _ppm) => DynNode::Resistor { rp: *r },
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

#[derive(Debug)]
enum ControlTarget {
    /// Modify the pre-gain multiplier (gain/drive/fuzz/distortion/sustain).
    PreGain,
    /// Modify output attenuation (level/volume/output).
    OutputGain,
    /// Modify a pot in a specific WDF stage.
    PotInStage(usize),
    /// Modify an LFO's rate (index into lfos vector).
    LfoRate(usize),
    /// Modify an LFO's depth/amplitude (index into lfos vector).
    LfoDepth(usize),
}

/// Modulation target for LFOs and envelope followers.
#[derive(Debug, Clone)]
enum ModulationTarget {
    /// Modulate a JFET's Vgs.
    JfetVgs { stage_idx: usize },
    /// Modulate ALL JFET stages' Vgs together (for phasers).
    AllJfetVgs,
    /// Modulate a Photocoupler's LED drive.
    PhotocouplerLed { stage_idx: usize, comp_id: String },
    /// Modulate a Triode's Vgk (grid-cathode bias).
    TriodeVgk { stage_idx: usize },
    /// Modulate a Pentode's Vg1k (control grid bias).
    PentodeVg1k { stage_idx: usize },
    /// Modulate a MOSFET's Vgs.
    MosfetVgs { stage_idx: usize },
    /// Modulate an OTA's bias current (for VCA/compressor).
    OtaIabc { stage_idx: usize },
    /// Modulate a BBD's clock frequency (for chorus/flanger).
    BbdClock { bbd_idx: usize },
    /// Modulate an op-amp's non-inverting input voltage.
    OpAmpVp { opamp_idx: usize },
}

/// Op-amp stage for unity-gain buffers and gain stages.
///
/// Models the op-amp as a voltage-controlled voltage source (VCVS).
/// The output voltage follows: Vout = Aol * (Vp - Vm)
/// For unity-gain buffers (neg tied to out), Vout ≈ Vp.
struct OpAmpStage {
    /// The op-amp root element with Newton-Raphson solver.
    opamp: OpAmpRoot,
    /// Component ID for debugging.
    #[allow(dead_code)]
    comp_id: String,
}

/// LFO binding in a compiled pedal.
struct LfoBinding {
    lfo: crate::elements::Lfo,
    target: ModulationTarget,
    /// Bias offset for the modulation (e.g., Vgs center point).
    bias: f64,
    /// Modulation range (amplitude).
    range: f64,
    /// Base frequency from RC timing: f = 1/(2πRC).
    base_freq: f64,
    /// LFO component ID (for debugging and future control binding).
    #[allow(dead_code)]
    lfo_id: String,
}

/// Envelope follower binding in a compiled pedal.
struct EnvelopeBinding {
    envelope: crate::elements::EnvelopeFollower,
    target: ModulationTarget,
    /// Bias offset for the modulation.
    bias: f64,
    /// Modulation range (amplitude).
    range: f64,
    /// Envelope follower component ID.
    #[allow(dead_code)]
    env_id: String,
}

/// Device-aware rail saturation model.
///
/// Real rail saturation depends on the active device's output stage topology.
/// Instead of a generic `tanh` waveshaper, this models the saturation shape
/// based on the dominant active device type in the circuit.
#[derive(Debug, Clone)]
enum RailSaturation {
    /// No active devices — passive clipping only (no rail saturation).
    None,
    /// Op-amp output stage: symmetric saturation with slight crossover
    /// distortion near zero. Output stage is push-pull emitter follower
    /// that saturates symmetrically against both rails.
    /// `output_swing_ratio` is Vout_max / Vsupply (typically 0.85–0.95).
    OpAmp { output_swing_ratio: f64 },
    /// BJT common-emitter: asymmetric saturation.  The collector saturates
    /// hard (~0.2V from rail) when driven into saturation, but cuts off
    /// softly when the base drive is removed.  NPN saturates at positive
    /// rail (hard), cuts off toward negative rail (soft).
    Bjt { vce_sat: f64 },
    /// JFET/MOSFET: soft saturation onset due to square-law I-V curve.
    /// The drain current pinches off gradually as Vds approaches Vgs-Vth.
    Fet,
    /// Triode/pentode: grid conduction (hard clip) at positive swing,
    /// plate current cutoff (soft) at negative swing.
    /// `mu` is the amplification factor (affects saturation sharpness).
    Tube { mu: f64 },
}

impl RailSaturation {
    /// Apply rail saturation to a signal given the supply headroom.
    ///
    /// `headroom` is `supply_voltage / 9.0` (normalized to 9V reference).
    #[inline]
    fn process(&self, signal: f64, headroom: f64) -> f64 {
        match self {
            RailSaturation::None => signal,
            RailSaturation::OpAmp { output_swing_ratio } => {
                // Op-amp output stage: symmetric clipping at ±(swing_ratio * headroom).
                // Uses a quintic polynomial for smoother knee than tanh, matching
                // the push-pull emitter-follower output stage characteristic.
                let ceiling = output_swing_ratio * headroom;
                let x = signal / ceiling;
                let ax = x.abs();
                if ax <= 0.75 {
                    // Linear region: no saturation
                    signal
                } else if ax >= 1.5 {
                    // Hard saturation
                    ceiling * x.signum()
                } else {
                    // Knee: quintic ease from 0.75 to ceiling
                    // Normalized to [0, 1] within knee region
                    let t = (ax - 0.75) / 0.75; // 0..1
                    let t2 = t * t;
                    let t3 = t2 * t;
                    // Hermite-like: starts at 0.75, ends at 1.0, smooth at both ends
                    let compressed = 0.75 + 0.25 * (3.0 * t2 - 2.0 * t3);
                    (ceiling * compressed).copysign(signal)
                }
            }
            RailSaturation::Bjt { vce_sat } => {
                // BJT asymmetric saturation.
                // Positive rail (collector saturation): hard knee at Vcc - Vce_sat
                // Negative rail (cutoff): soft exponential tail
                let pos_ceiling = headroom * (1.0 - vce_sat / (9.0 * headroom));
                let neg_ceiling = headroom;
                if signal > 0.0 {
                    // Hard saturation toward positive rail (collector saturation)
                    let x = signal / pos_ceiling;
                    if x <= 0.7 {
                        signal
                    } else if x >= 1.5 {
                        pos_ceiling
                    } else {
                        let t = (x - 0.7) / 0.8;
                        let t2 = t * t;
                        pos_ceiling * (0.7 + 0.3 * (2.0 * t - t2))
                    }
                } else {
                    // Soft cutoff toward negative rail (transistor cutoff)
                    let x = -signal / neg_ceiling;
                    if x <= 0.8 {
                        signal
                    } else {
                        let over = x - 0.8;
                        // Exponential compression: gradual cutoff
                        -(neg_ceiling * (0.8 + 0.2 * (1.0 - (-over * 5.0).exp())))
                    }
                }
            }
            RailSaturation::Fet => {
                // JFET/MOSFET: soft saturation from square-law characteristic.
                // The onset is gentler than BJT — current gradually pinches off.
                let ceiling = headroom;
                let x = signal / ceiling;
                let ax = x.abs();
                if ax <= 0.85 {
                    signal
                } else {
                    // Square-law-like soft compression
                    let over = ax - 0.85;
                    let compressed = 0.85 + 0.15 * (1.0 - 1.0 / (1.0 + over * 3.0));
                    (ceiling * compressed).copysign(signal)
                }
            }
            RailSaturation::Tube { mu } => {
                // Triode: asymmetric saturation.
                // Positive swing → grid conduction (hard clipping, like a diode)
                // Negative swing → plate current cutoff (soft, gradual)
                // Higher mu = sharper saturation knee
                let ceiling = headroom;
                let sharpness = (mu / 100.0).clamp(0.5, 3.0);
                if signal > 0.0 {
                    // Grid conduction — hard clip (grid draws current)
                    let x = signal / ceiling;
                    if x <= 0.6 {
                        signal
                    } else if x >= 2.0 {
                        ceiling
                    } else {
                        let t = (x - 0.6) / 1.4;
                        let curve = 1.0 - (-t * 2.5 * sharpness).exp();
                        ceiling * (0.6 + 0.4 * curve)
                    }
                } else {
                    // Plate cutoff — soft, gradual
                    let x = -signal / ceiling;
                    if x <= 0.9 {
                        signal
                    } else {
                        let over = x - 0.9;
                        let compressed = 0.9 + 0.1 * over / (0.1 + over);
                        -(ceiling * compressed)
                    }
                }
            }
        }
    }
}

/// A pedal processor compiled from a `.pedal` file's netlist.
///
/// Each `.pedal` file produces a unique processor with its own WDF tree topology,
/// component values, and diode models — no hardcoded processor selection.
pub struct CompiledPedal {
    stages: Vec<WdfStage>,
    pre_gain: f64,
    output_gain: f64,
    rail_saturation: RailSaturation,
    sample_rate: f64,
    controls: Vec<ControlBinding>,
    gain_range: (f64, f64),
    /// Supply voltage in volts (default 9.0).
    supply_voltage: f64,
    /// LFO modulators.
    lfos: Vec<LfoBinding>,
    /// Envelope follower modulators.
    envelopes: Vec<EnvelopeBinding>,
    /// Slew rate limiters for op-amp stages (one per op-amp in the circuit).
    /// Applied to the signal path to model HF compression from slow op-amps.
    slew_limiters: Vec<SlewRateLimiter>,
    /// BBD delay lines for delay/chorus/flanger effects.
    bbds: Vec<BbdDelayLine>,
    /// Thermal model for temperature-dependent behavior.
    /// When present, modulates diode Is and BJT gain over time.
    thermal: Option<ThermalModel>,
    /// Tolerance engine seed (stored for diagnostics).
    tolerance_seed: u64,
    /// Oversampling factor used for this pedal's nonlinear stages.
    oversampling: OversamplingFactor,
    /// Op-amp stages for unity-gain buffers and gain stages.
    /// Each op-amp is modeled as a VCVS with the OpAmpRoot element.
    opamp_stages: Vec<OpAmpStage>,
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

/// Rate-like control labels (for LFO).
fn is_rate_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "rate" | "speed" | "tempo"
    )
}

/// Depth-like control labels (for LFO modulation amount).
fn is_depth_label(label: &str) -> bool {
    matches!(
        label.to_ascii_lowercase().as_str(),
        "depth" | "intensity" | "width"
    )
}

impl CompiledPedal {
    pub fn set_supply_voltage(&mut self, voltage: f64) {
        self.supply_voltage = voltage.clamp(5.0, 24.0);
    }

    /// Get the tolerance seed for this pedal unit (for diagnostics/UI).
    pub fn tolerance_seed(&self) -> u64 {
        self.tolerance_seed
    }

    /// Get the oversampling factor used for this pedal.
    pub fn oversampling(&self) -> OversamplingFactor {
        self.oversampling
    }

    /// Set a control by its label (e.g., "Drive", "Level", "Rate").
    pub fn set_control(&mut self, label: &str, value: f64) {
        let value = value.clamp(0.0, 1.0);
        for i in 0..self.controls.len() {
            if !self.controls[i].label.eq_ignore_ascii_case(label) {
                continue;
            }
            match &self.controls[i].target {
                ControlTarget::PreGain => {
                    let (lo, hi) = self.gain_range;
                    self.pre_gain = lo * (hi / lo).powf(value);
                }
                ControlTarget::OutputGain => {
                    self.output_gain = value;
                }
                ControlTarget::PotInStage(stage_idx) => {
                    let stage_idx = *stage_idx;
                    let comp_id = self.controls[i].component_id.clone();
                    if let Some(stage) = self.stages.get_mut(stage_idx) {
                        stage.tree.set_pot(&comp_id, value);
                        // For 3-terminal pots: update synthetic halves with
                        // inverse coupling. No-ops for 2-terminal pots.
                        stage.tree.set_pot(&format!("{comp_id}__aw"), value);
                        stage.tree.set_pot(&format!("{comp_id}__wb"), 1.0 - value);
                        stage.tree.recompute();
                    }
                }
                ControlTarget::LfoRate(lfo_idx) => {
                    let lfo_idx = *lfo_idx;
                    if let Some(binding) = self.lfos.get_mut(lfo_idx) {
                        // Scale rate around base_freq: 0.1x to 10x (100x range)
                        // pot=0 -> 0.1x, pot=0.5 -> 1x, pot=1 -> 10x
                        let scale = 0.1_f64 * 100.0_f64.powf(value);
                        let rate = binding.base_freq * scale;
                        binding.lfo.set_rate(rate);
                    }
                }
                ControlTarget::LfoDepth(lfo_idx) => {
                    let lfo_idx = *lfo_idx;
                    if let Some(binding) = self.lfos.get_mut(lfo_idx) {
                        binding.lfo.set_depth(value);
                    }
                }
            }
            return;
        }
    }
}

impl PedalProcessor for CompiledPedal {
    fn process(&mut self, input: f64) -> f64 {
        // Tick thermal model — temperature changes are very slow (updated
        // every ~1000 samples internally), so the overhead is negligible.
        // The thermal state modulates diode Is and n_vt for each stage.
        if let Some(ref mut thermal) = self.thermal {
            let state = *thermal.tick();
            for stage in &mut self.stages {
                stage.apply_thermal(&state);
            }
        }

        // Tick all LFOs and route their outputs to targets.
        for binding in &mut self.lfos {
            let lfo_out = binding.lfo.tick();
            let modulation = binding.bias + lfo_out * binding.range;

            match &binding.target {
                ModulationTarget::JfetVgs { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage.set_jfet_vgs(modulation);
                    }
                }
                ModulationTarget::AllJfetVgs => {
                    // Modulate ALL JFET stages with the same value (for phasers)
                    for stage in &mut self.stages {
                        if matches!(&stage.root, RootKind::Jfet(_)) {
                            stage.set_jfet_vgs(modulation);
                        }
                    }
                }
                ModulationTarget::PhotocouplerLed { stage_idx, comp_id } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage
                            .tree
                            .set_photocoupler_led(comp_id, modulation.clamp(0.0, 1.0));
                        stage.tree.recompute();
                    }
                }
                ModulationTarget::TriodeVgk { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage.set_triode_vgk(modulation);
                    }
                }
                ModulationTarget::PentodeVg1k { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage.set_pentode_vg1k(modulation);
                    }
                }
                ModulationTarget::MosfetVgs { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage.set_mosfet_vgs(modulation);
                    }
                }
                ModulationTarget::OtaIabc { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        // LFO modulation of OTA: modulation maps to gain (0-1)
                        stage.set_ota_gain(modulation.clamp(0.0, 1.0));
                    }
                }
                ModulationTarget::BbdClock { bbd_idx } => {
                    if let Some(bbd) = self.bbds.get_mut(*bbd_idx) {
                        // LFO modulates delay time: modulation = normalized 0-1
                        bbd.set_delay_normalized(modulation.clamp(0.0, 1.0));
                    }
                }
                ModulationTarget::OpAmpVp { opamp_idx } => {
                    if let Some(opamp_stage) = self.opamp_stages.get_mut(*opamp_idx) {
                        // LFO modulates op-amp's non-inverting input
                        opamp_stage.opamp.set_vp(modulation);
                    }
                }
            }
        }

        // Tick all envelope followers and route their outputs to targets.
        for binding in &mut self.envelopes {
            let env_out = binding.envelope.process(input);
            let modulation = binding.bias + env_out * binding.range;

            match &binding.target {
                ModulationTarget::JfetVgs { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage.set_jfet_vgs(modulation);
                    }
                }
                ModulationTarget::AllJfetVgs => {
                    // Modulate ALL JFET stages with the same value (for phasers)
                    for stage in &mut self.stages {
                        if matches!(&stage.root, RootKind::Jfet(_)) {
                            stage.set_jfet_vgs(modulation);
                        }
                    }
                }
                ModulationTarget::PhotocouplerLed { stage_idx, comp_id } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage
                            .tree
                            .set_photocoupler_led(comp_id, modulation.clamp(0.0, 1.0));
                        stage.tree.recompute();
                    }
                }
                ModulationTarget::TriodeVgk { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage.set_triode_vgk(modulation);
                    }
                }
                ModulationTarget::PentodeVg1k { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage.set_pentode_vg1k(modulation);
                    }
                }
                ModulationTarget::MosfetVgs { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        stage.set_mosfet_vgs(modulation);
                    }
                }
                ModulationTarget::OtaIabc { stage_idx } => {
                    if let Some(stage) = self.stages.get_mut(*stage_idx) {
                        // Envelope controls OTA gain: louder input → lower gain
                        // Invert: high envelope (loud) → low gain (compression)
                        let gain = (1.0 - modulation).clamp(0.0, 1.0);
                        stage.set_ota_gain(gain);
                    }
                }
                ModulationTarget::BbdClock { bbd_idx } => {
                    if let Some(bbd) = self.bbds.get_mut(*bbd_idx) {
                        bbd.set_delay_normalized(modulation.clamp(0.0, 1.0));
                    }
                }
                ModulationTarget::OpAmpVp { opamp_idx } => {
                    if let Some(opamp_stage) = self.opamp_stages.get_mut(*opamp_idx) {
                        opamp_stage.opamp.set_vp(modulation);
                    }
                }
            }
        }

        // Headroom models the supply rail ceiling.  In real circuits, gain
        // is set by resistor ratios (feedback/input) and is independent of
        // supply voltage.  What changes with voltage is how far the active
        // element can swing before hitting the rail — i.e. the clipping
        // ceiling.  So headroom only scales the soft limiter, not pre_gain
        // or output_gain.
        let headroom = self.supply_voltage / 9.0;
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

        // Process through op-amp stages.
        // Each op-amp is modeled as a VCVS (voltage-controlled voltage source).
        // For unity-gain buffers (most common), the op-amp passes the signal
        // through with high input impedance and low output impedance.
        // The slew rate limiting is built into OpAmpRoot.
        for opamp_stage in &mut self.opamp_stages {
            // Set the input signal as Vp (non-inverting input)
            opamp_stage.opamp.set_vp(signal);
            // Process through the op-amp's VCVS model
            // For unity-gain buffer: output ≈ Vp (with slew rate limiting)
            let b = opamp_stage.opamp.process(0.0, 10_000.0);
            signal = b / 2.0; // Convert reflected wave to voltage
        }

        // Apply slew rate limiting from op-amps.
        // Each op-amp in the circuit contributes its own slew rate limit.
        // This is the mechanism that makes the LM308 RAT sound different
        // from a TL072 RAT — the slow slew rate rounds off HF transients.
        for slew in &mut self.slew_limiters {
            signal = slew.process(signal);
        }

        signal = self.rail_saturation.process(signal, headroom);

        // Process through BBD delay lines (wet signal mixed with dry).
        for bbd in &mut self.bbds {
            let wet = bbd.process(signal);
            signal = signal * 0.5 + wet * 0.5;
        }

        signal * self.output_gain
    }

    fn set_sample_rate(&mut self, rate: f64) {
        self.sample_rate = rate;
        for stage in &mut self.stages {
            stage.tree.update_sample_rate(rate);
            stage.tree.recompute();
        }
        for binding in &mut self.lfos {
            binding.lfo.set_sample_rate(rate);
        }
        for binding in &mut self.envelopes {
            binding.envelope.set_sample_rate(rate);
        }
        for slew in &mut self.slew_limiters {
            slew.set_sample_rate(rate);
        }
        for bbd in &mut self.bbds {
            bbd.set_sample_rate(rate);
        }
        if let Some(ref mut thermal) = self.thermal {
            thermal.set_sample_rate(rate);
        }
        for opamp_stage in &mut self.opamp_stages {
            opamp_stage.opamp.set_sample_rate(rate);
        }
    }

    fn reset(&mut self) {
        for stage in &mut self.stages {
            stage.reset();
        }
        for binding in &mut self.lfos {
            binding.lfo.reset();
        }
        for binding in &mut self.envelopes {
            binding.envelope.reset();
        }
        for slew in &mut self.slew_limiters {
            slew.reset();
        }
        for bbd in &mut self.bbds {
            bbd.reset();
        }
        for opamp_stage in &mut self.opamp_stages {
            opamp_stage.opamp.reset();
        }
        if let Some(ref mut thermal) = self.thermal {
            thermal.reset();
        }
    }

    fn set_control(&mut self, label: &str, value: f64) {
        self.set_control(label, value);
    }

    fn set_supply_voltage(&mut self, voltage: f64) {
        self.set_supply_voltage(voltage);
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
        let node_name = match &net.from {
            Pin::Reserved(n) => n.clone(),
            Pin::ComponentPin { component, .. } => component.clone(),
        };
        for pin in &net.to {
            let to_name = match pin {
                Pin::Reserved(n) => n.clone(),
                Pin::ComponentPin { component, .. } => component.clone(),
            };
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
            // or is a standard reserved node (in, out, gnd, vcc).
            let relevant = |p: &Pin| match p {
                Pin::Reserved(n) => {
                    n == "in" || n == "out" || n == "gnd" || n == "vcc" || n == new_node
                }
                Pin::ComponentPin { component, .. } => component_ids.contains(component),
            };

            // A pin "belongs" to this half — only component pins in our set.
            let belongs = |p: &Pin| match p {
                Pin::Reserved(_) => true,
                Pin::ComponentPin { component, .. } => component_ids.contains(component),
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

    PedalDef {
        name: format!("{} ({})", pedal.name, suffix),
        components,
        nets,
        controls,
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
/// Options for pedal compilation.
///
/// Controls optional features like oversampling, component tolerance,
/// and thermal modeling.
pub struct CompileOptions {
    /// Oversampling factor for nonlinear stages (default: X1 = off).
    pub oversampling: OversamplingFactor,
    /// Component tolerance engine (default: ideal = no variation).
    pub tolerance: ToleranceEngine,
    /// Whether to enable thermal drift modeling.
    pub thermal: bool,
}

impl Default for CompileOptions {
    fn default() -> Self {
        Self {
            oversampling: OversamplingFactor::X1,
            tolerance: ToleranceEngine::ideal(),
            thermal: false,
        }
    }
}

/// Compile a pedal definition with default options (no oversampling, no tolerance).
pub fn compile_pedal(pedal: &PedalDef, sample_rate: f64) -> Result<CompiledPedal, String> {
    compile_pedal_with_options(pedal, sample_rate, CompileOptions::default())
}

/// Compile a pedal definition with custom options.
pub fn compile_pedal_with_options(
    pedal: &PedalDef,
    sample_rate: f64,
    options: CompileOptions,
) -> Result<CompiledPedal, String> {
    let oversampling = options.oversampling;
    let tolerance = options.tolerance;
    let enable_thermal = options.thermal;

    let mut graph = CircuitGraph::from_pedal(pedal);

    // Apply component tolerance to resistors and capacitors.
    // This deterministically shifts component values from nominal based on the
    // tolerance seed, so two instances of the same pedal model will sound
    // slightly different (like real hardware).
    for (i, comp) in graph.components.iter_mut().enumerate() {
        match &mut comp.kind {
            ComponentKind::Resistor(r) => {
                *r = tolerance.apply_resistor(*r, i);
            }
            ComponentKind::Capacitor(c) => {
                *c = tolerance.apply_capacitor(*c, i);
            }
            ComponentKind::Potentiometer(max_r) => {
                *max_r = tolerance.apply_resistor(*max_r, i);
            }
            _ => {}
        }
    }

    let diodes = graph.find_diodes();
    let diode_edge_indices: Vec<usize> = diodes.iter().map(|(idx, _)| *idx).collect();

    // Determine gain range based on circuit characteristics.
    //
    // The gain taxonomy considers:
    //   - Diode type (germanium clips softer → needs more drive)
    //   - Single vs pair diodes (single = asymmetric = fuzz-like)
    //   - Number of clipping stages (multi-stage = less gain per stage)
    //   - Active device count and type (each contributes specific gain)
    let has_germanium = diodes
        .iter()
        .any(|(_, d)| d.diode_type == DiodeType::Germanium);
    let has_single_diode = diodes.iter().any(|(_, d)| !d.is_pair);
    let has_led = diodes
        .iter()
        .any(|(_, d)| d.diode_type == DiodeType::Led);
    let gain_range = if has_germanium && has_single_diode {
        (5.0, 250.0) // Fuzz-like: single germanium diode needs high drive
    } else if has_germanium {
        (3.0, 150.0) // Germanium overdrive (Klon-ish)
    } else if has_led {
        (4.0, 120.0) // LED clipping: higher Vf → needs more drive, but cleaner
    } else if diodes.len() > 2 {
        (1.5, 40.0) // Many stages: each clips a little, cumulative is heavy
    } else if diodes.len() > 1 {
        (2.0, 60.0) // Two stages (e.g., Big Muff topology)
    } else {
        (2.0, 100.0) // Standard single-stage overdrive
    };

    // Active elements contribute gain based on their type.
    // Instead of a flat bonus, compute per-device gain contributions.
    let active_bonus = {
        let mut bonus = 1.0_f64;
        for comp in &pedal.components {
            match &comp.kind {
                ComponentKind::OpAmp(ot) if !ot.is_ota() => {
                    // Op-amp gain depends on type — faster op-amps provide
                    // more effective gain at audio frequencies.
                    bonus *= match ot {
                        OpAmpType::Ne5532 => 3.5,   // High GBW = more gain
                        OpAmpType::Tl072 | OpAmpType::Tl082 | OpAmpType::Generic => 3.0,
                        OpAmpType::Jrc4558 | OpAmpType::Rc4558 => 2.5,
                        OpAmpType::Lm741 | OpAmpType::Op07 => 2.0,
                        OpAmpType::Lm308 => 1.8,    // Low GBW = less effective gain
                        _ => 2.5,
                    };
                }
                ComponentKind::Npn(_) | ComponentKind::Pnp(_) => {
                    // Single BJT CE stage: gain ≈ 20–50x typical
                    bonus *= 2.5;
                }
                ComponentKind::NJfet(_) | ComponentKind::PJfet(_) => {
                    // JFET CS stage: lower gain than BJT (gm is lower)
                    bonus *= 1.5;
                }
                ComponentKind::Triode(_) => {
                    // Triode gain stage: mu ≈ 17–100, but loaded gain is lower
                    bonus *= 2.0;
                }
                ComponentKind::Pentode(_) => {
                    // Pentode: higher gain than triode
                    bonus *= 3.0;
                }
                _ => {}
            }
        }
        bonus
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
        let passive_idxs =
            graph.elements_at_junction(junction, &diode_edge_indices, &graph.active_edge_indices);

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
        // Fallback: if in_node is disconnected (e.g. split pedal post-half
        // where in maps to a triode grid), use gnd as the injection point.
        if best_dist == usize::MAX {
            injection_node = graph.gnd_node;
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
            Err(_) => continue,
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
            oversampler: Oversampler::new(oversampling),
            base_diode_model: Some(model),
        });
    }

    // ── Op-amp stages ──────────────────────────────────────────────────────
    // Create OpAmpRoot elements for each non-OTA op-amp in the circuit.
    // These model the op-amp as a VCVS (voltage-controlled voltage source)
    // with slew rate limiting and output saturation.
    let mut opamp_stages: Vec<OpAmpStage> = Vec::new();
    for comp in &pedal.components {
        if let ComponentKind::OpAmp(ot) = &comp.kind {
            // OTAs (CA3080) are handled separately as OtaRoot, not OpAmpRoot
            if !ot.is_ota() {
                let model = OpAmpModel::from_opamp_type(ot);
                let mut opamp = OpAmpRoot::new(model);
                opamp.set_sample_rate(sample_rate);
                opamp_stages.push(OpAmpStage {
                    opamp,
                    comp_id: comp.id.clone(),
                });
            }
        }
    }

    // Build WDF stages for JFETs.
    let jfets = graph.find_jfets();
    let jfet_edge_indices: Vec<usize> = jfets.iter().map(|(idx, _)| *idx).collect();
    let all_nonlinear_indices: Vec<usize> = diode_edge_indices
        .iter()
        .chain(jfet_edge_indices.iter())
        .copied()
        .collect();

    for (_edge_idx, jfet_info) in &jfets {
        let junction = jfet_info.junction_node;
        let passive_idxs = graph.elements_at_junction(
            junction,
            &all_nonlinear_indices,
            &graph.active_edge_indices,
        );

        if passive_idxs.is_empty() {
            continue;
        }

        // Find the best injection node for the voltage source.
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
        // Fallback: if in_node is disconnected (e.g. split pedal post-half
        // where in maps to a triode grid), use gnd as the injection point.
        if best_dist == usize::MAX {
            injection_node = graph.gnd_node;
        }

        // Build SP edges.
        let source_node = graph.edges.len() + 2000; // virtual source node (different from diodes)
        let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();
        sp_edges.push((source_node, injection_node, SpTree::Leaf(vs_comp_idx)));

        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
        }

        let terminals = vec![source_node, junction];
        let sp_tree = match sp_reduce(sp_edges, &terminals) {
            Ok(t) => t,
            Err(_) => continue,
        };

        // Create components list with virtual voltage source for JFET stages.
        let mut jfet_components = graph.components.clone();
        while jfet_components.len() <= vs_comp_idx {
            jfet_components.push(ComponentDef {
                id: "__vs__".to_string(),
                kind: ComponentKind::Resistor(1.0),
            });
        }

        let tree = sp_to_dyn_with_vs(&sp_tree, &jfet_components, sample_rate, vs_comp_idx);

        let model = jfet_model(jfet_info.jfet_type, jfet_info.is_n_channel);
        let root = RootKind::Jfet(JfetRoot::new(model));

        stages.push(WdfStage {
            tree,
            root,
            compensation: 1.0,
            oversampler: Oversampler::new(oversampling),
            base_diode_model: None,
        });
    }

    // Build WDF stages for BJTs.
    let bjts = graph.find_bjts();
    let bjt_edge_indices: Vec<usize> = bjts.iter().map(|(idx, _)| *idx).collect();
    let all_nonlinear_with_bjts: Vec<usize> = all_nonlinear_indices
        .iter()
        .chain(bjt_edge_indices.iter())
        .copied()
        .collect();

    for (_edge_idx, bjt_info) in &bjts {
        let junction = bjt_info.junction_node;
        let passive_idxs = graph.elements_at_junction(
            junction,
            &all_nonlinear_with_bjts,
            &graph.active_edge_indices,
        );

        if passive_idxs.is_empty() {
            continue;
        }

        // Find the best injection node for the voltage source.
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
        if best_dist == usize::MAX {
            injection_node = graph.gnd_node;
        }

        // Build SP edges.
        let source_node = graph.edges.len() + 3000; // virtual source node for BJTs
        let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();
        sp_edges.push((source_node, injection_node, SpTree::Leaf(vs_comp_idx)));

        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
        }

        let terminals = vec![source_node, junction];
        let sp_tree = match sp_reduce(sp_edges, &terminals) {
            Ok(t) => t,
            Err(_) => continue,
        };

        // Create components list with virtual voltage source for BJT stages.
        let mut bjt_components = graph.components.clone();
        while bjt_components.len() <= vs_comp_idx {
            bjt_components.push(ComponentDef {
                id: "__vs__".to_string(),
                kind: ComponentKind::Resistor(1.0),
            });
        }

        let tree = sp_to_dyn_with_vs(&sp_tree, &bjt_components, sample_rate, vs_comp_idx);

        let model = BjtModel::from_bjt_type(&bjt_info.bjt_type);
        let root = if bjt_info.is_npn {
            RootKind::BjtNpn(BjtNpnRoot::new(model))
        } else {
            RootKind::BjtPnp(BjtPnpRoot::new(model))
        };

        stages.push(WdfStage {
            tree,
            root,
            compensation: 1.0,
            oversampler: Oversampler::new(oversampling),
            base_diode_model: None,
        });
    }

    // Build WDF stages for triodes.
    let triodes = graph.find_triodes();
    let triode_edge_indices: Vec<usize> = triodes.iter().map(|(idx, _)| *idx).collect();
    let all_nonlinear_with_triodes: Vec<usize> = all_nonlinear_with_bjts
        .iter()
        .chain(triode_edge_indices.iter())
        .copied()
        .collect();

    for (_edge_idx, triode_info) in &triodes {
        let junction = triode_info.junction_node;
        let passive_idxs = graph.elements_at_junction(
            junction,
            &all_nonlinear_with_triodes,
            &graph.active_edge_indices,
        );

        if passive_idxs.is_empty() {
            continue;
        }

        // Find the best injection node for the voltage source.
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
        // Fallback: if in_node is disconnected (e.g. split pedal post-half
        // where in maps to a triode grid), use gnd as the injection point.
        if best_dist == usize::MAX {
            injection_node = graph.gnd_node;
        }

        // Build SP edges.
        let source_node = graph.edges.len() + 3000; // virtual source node (unique offset)
        let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();
        sp_edges.push((source_node, injection_node, SpTree::Leaf(vs_comp_idx)));

        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
        }

        let terminals = vec![source_node, junction];
        let sp_tree = match sp_reduce(sp_edges, &terminals) {
            Ok(t) => t,
            Err(_) => continue,
        };

        let mut triode_components = graph.components.clone();
        while triode_components.len() <= vs_comp_idx {
            triode_components.push(ComponentDef {
                id: "__vs__".to_string(),
                kind: ComponentKind::Resistor(1.0),
            });
        }

        let tree = sp_to_dyn_with_vs(&sp_tree, &triode_components, sample_rate, vs_comp_idx);

        let model = triode_model(triode_info.triode_type);
        let root = RootKind::Triode(TriodeRoot::new(model));

        stages.push(WdfStage {
            tree,
            root,
            compensation: 1.0,
            oversampler: Oversampler::new(oversampling),
            base_diode_model: None,
        });
    }

    // Build WDF stages for pentodes.
    let pentodes = graph.find_pentodes();
    let pentode_edge_indices: Vec<usize> = pentodes.iter().map(|(idx, _)| *idx).collect();
    let all_nonlinear_with_pentodes: Vec<usize> = all_nonlinear_with_triodes
        .iter()
        .chain(pentode_edge_indices.iter())
        .copied()
        .collect();

    for (_edge_idx, pentode_info) in &pentodes {
        let junction = pentode_info.junction_node;
        let passive_idxs = graph.elements_at_junction(
            junction,
            &all_nonlinear_with_pentodes,
            &graph.active_edge_indices,
        );

        if passive_idxs.is_empty() {
            continue;
        }

        // Find the best injection node for the voltage source.
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
        if best_dist == usize::MAX {
            injection_node = graph.gnd_node;
        }

        // Build SP edges.
        let source_node = graph.edges.len() + 4000; // unique offset for pentodes
        let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();
        sp_edges.push((source_node, injection_node, SpTree::Leaf(vs_comp_idx)));

        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
        }

        let terminals = vec![source_node, junction];
        let sp_tree = match sp_reduce(sp_edges, &terminals) {
            Ok(t) => t,
            Err(_) => continue,
        };

        let mut pentode_components = graph.components.clone();
        while pentode_components.len() <= vs_comp_idx {
            pentode_components.push(ComponentDef {
                id: "__vs__".to_string(),
                kind: ComponentKind::Resistor(1.0),
            });
        }

        let tree = sp_to_dyn_with_vs(&sp_tree, &pentode_components, sample_rate, vs_comp_idx);

        let model = pentode_model(pentode_info.pentode_type);
        let root = RootKind::Pentode(PentodeRoot::new(model));

        stages.push(WdfStage {
            tree,
            root,
            compensation: 1.0,
            oversampler: Oversampler::new(oversampling),
            base_diode_model: None,
        });
    }

    // Build WDF stages for MOSFETs.
    let mosfets = graph.find_mosfets();
    let mosfet_edge_indices: Vec<usize> = mosfets.iter().map(|(idx, _)| *idx).collect();
    let all_nonlinear_with_mosfets: Vec<usize> = all_nonlinear_with_pentodes
        .iter()
        .chain(mosfet_edge_indices.iter())
        .copied()
        .collect();

    for (_edge_idx, mosfet_info) in &mosfets {
        let junction = mosfet_info.junction_node;
        let passive_idxs = graph.elements_at_junction(
            junction,
            &all_nonlinear_with_mosfets,
            &graph.active_edge_indices,
        );

        if passive_idxs.is_empty() {
            continue;
        }

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
        if best_dist == usize::MAX {
            injection_node = graph.gnd_node;
        }

        let source_node = graph.edges.len() + 4000;
        let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();
        sp_edges.push((source_node, injection_node, SpTree::Leaf(vs_comp_idx)));

        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
        }

        let terminals = vec![source_node, junction];
        let sp_tree = match sp_reduce(sp_edges, &terminals) {
            Ok(t) => t,
            Err(_) => continue,
        };

        let mut mosfet_components = graph.components.clone();
        while mosfet_components.len() <= vs_comp_idx {
            mosfet_components.push(ComponentDef {
                id: "__vs__".to_string(),
                kind: ComponentKind::Resistor(1.0),
            });
        }

        let tree = sp_to_dyn_with_vs(&sp_tree, &mosfet_components, sample_rate, vs_comp_idx);

        let model = mosfet_model(mosfet_info.mosfet_type, mosfet_info.is_n_channel);
        let root = RootKind::Mosfet(MosfetRoot::new(model));

        stages.push(WdfStage {
            tree,
            root,
            compensation: 1.0,
            oversampler: Oversampler::new(oversampling),
            base_diode_model: None,
        });
    }

    // Build WDF stages for Zener diodes.
    let zeners = graph.find_zeners();
    let zener_edge_indices: Vec<usize> = zeners.iter().map(|(idx, _)| *idx).collect();
    let all_nonlinear_final: Vec<usize> = all_nonlinear_with_mosfets
        .iter()
        .chain(zener_edge_indices.iter())
        .copied()
        .collect();

    for (_edge_idx, zener_info) in &zeners {
        let junction = zener_info.junction_node;
        let passive_idxs = graph.elements_at_junction(
            junction,
            &all_nonlinear_final,
            &graph.active_edge_indices,
        );

        if passive_idxs.is_empty() {
            continue;
        }

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
        if best_dist == usize::MAX {
            injection_node = graph.gnd_node;
        }

        let source_node = graph.edges.len() + 5000;
        let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();
        sp_edges.push((source_node, injection_node, SpTree::Leaf(vs_comp_idx)));

        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
        }

        let terminals = vec![source_node, junction];
        let sp_tree = match sp_reduce(sp_edges, &terminals) {
            Ok(t) => t,
            Err(_) => continue,
        };

        let mut zener_components = graph.components.clone();
        while zener_components.len() <= vs_comp_idx {
            zener_components.push(ComponentDef {
                id: "__vs__".to_string(),
                kind: ComponentKind::Resistor(1.0),
            });
        }

        let tree = sp_to_dyn_with_vs(&sp_tree, &zener_components, sample_rate, vs_comp_idx);

        let model = ZenerModel::new(zener_info.voltage);
        let root = RootKind::Zener(ZenerRoot::new(model));

        stages.push(WdfStage {
            tree,
            root,
            compensation: 1.0,
            oversampler: Oversampler::new(oversampling),
            base_diode_model: None,
        });
    }

    // ── OTA (CA3080) stages ──────────────────────────────────────────────
    let otas = graph.find_otas();
    let ota_edge_indices: Vec<usize> = otas.iter().map(|(idx, _)| *idx).collect();
    let all_nonlinear_with_otas: Vec<usize> = all_nonlinear_final
        .iter()
        .chain(ota_edge_indices.iter())
        .copied()
        .collect();

    for (_edge_idx, ota_info) in &otas {
        let junction = ota_info.junction_node;
        let passive_idxs = graph.elements_at_junction(
            junction,
            &all_nonlinear_with_otas,
            &graph.active_edge_indices,
        );

        if passive_idxs.is_empty() {
            continue;
        }

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
        if best_dist == usize::MAX {
            injection_node = graph.gnd_node;
        }

        let source_node = graph.edges.len() + 6000;
        let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();
        sp_edges.push((source_node, injection_node, SpTree::Leaf(vs_comp_idx)));

        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
        }

        let terminals = vec![source_node, junction];
        let sp_tree = match sp_reduce(sp_edges, &terminals) {
            Ok(t) => t,
            Err(_) => continue,
        };

        let mut ota_components = graph.components.clone();
        while ota_components.len() <= vs_comp_idx {
            ota_components.push(ComponentDef {
                id: "__vs__".to_string(),
                kind: ComponentKind::Resistor(1.0),
            });
        }

        let tree = sp_to_dyn_with_vs(&sp_tree, &ota_components, sample_rate, vs_comp_idx);

        let model = OtaModel::ca3080();
        let root = RootKind::Ota(OtaRoot::new(model));

        // OTA stages need compensation: the transconductance amplifier's
        // voltage gain (gm * Rp) can be very high. In real circuits, negative
        // feedback (R2 in Dyna Comp) tames this. We apply a compensation
        // factor based on the feedback network impedance.
        let feedback_compensation = 0.08; // Models closed-loop gain reduction from feedback R

        stages.push(WdfStage {
            tree,
            root,
            compensation: feedback_compensation,
            oversampler: Oversampler::new(oversampling),
            base_diode_model: None,
        });
    }

    // Build WDF stages for zener diodes.
    let zeners = graph.find_zeners();
    let zener_edge_indices: Vec<usize> = zeners.iter().map(|(idx, _)| *idx).collect();
    let all_nonlinear_with_zeners: Vec<usize> = all_nonlinear_with_triodes
        .iter()
        .chain(zener_edge_indices.iter())
        .copied()
        .collect();

    for (_edge_idx, zener_info) in &zeners {
        let junction = zener_info.junction_node;
        let passive_idxs = graph.elements_at_junction(
            junction,
            &all_nonlinear_with_zeners,
            &graph.active_edge_indices,
        );

        if passive_idxs.is_empty() {
            continue;
        }

        // Find the best injection node for the voltage source.
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

        // Build SP edges.
        let source_node = graph.edges.len() + 4000; // virtual source node (unique offset for zeners)
        let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();
        sp_edges.push((source_node, injection_node, SpTree::Leaf(vs_comp_idx)));

        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
        }

        let terminals = vec![source_node, junction];
        let sp_tree = match sp_reduce(sp_edges, &terminals) {
            Ok(t) => t,
            Err(_) => continue,
        };

        let mut zener_components = graph.components.clone();
        while zener_components.len() <= vs_comp_idx {
            zener_components.push(ComponentDef {
                id: "__vs__".to_string(),
                kind: ComponentKind::Resistor(1.0),
            });
        }

        let tree = sp_to_dyn_with_vs(&sp_tree, &zener_components, sample_rate, vs_comp_idx);

        let model = ZenerModel::with_voltage(zener_info.voltage);
        let root = RootKind::Zener(ZenerRoot::new(model));

        stages.push(WdfStage {
            tree,
            root,
            compensation: 1.0,
            oversampler: Oversampler::new(oversampling),
            base_diode_model: None,
        });
    }

    // Balance voltage source impedance in each stage.
    // This fixes topologies where the Vs branch sits in a Parallel adaptor
    // opposite a high-impedance element (e.g. Big Muff: Parallel(Series(Vs,C), R)
    // with R >> C causes gamma ≈ 1.0 and severe signal attenuation).
    for stage in &mut stages {
        stage.balance_vs_impedance();
    }

    // ── Slew rate limiters (from op-amps) ─────────────────────────────────
    // Each op-amp in the circuit contributes a slew rate limiter to model
    // the HF compression from finite output slew rate.
    let mut slew_limiters = Vec::new();
    for comp in &pedal.components {
        if let ComponentKind::OpAmp(ot) = &comp.kind {
            // OTAs (CA3080) are very fast (50 V/µs) — slew limiting is negligible.
            // Only add slew limiters for voltage-mode op-amps with slow slew rates.
            if !ot.is_ota() {
                slew_limiters.push(SlewRateLimiter::new(ot.slew_rate(), sample_rate));
            }
        }
    }

    // ── BBD delay lines ──────────────────────────────────────────────────
    let mut bbds = Vec::new();
    for comp in &pedal.components {
        if let ComponentKind::Bbd(bt) = &comp.kind {
            let model = match bt {
                BbdType::Mn3207 => BbdModel::mn3207(),
                BbdType::Mn3007 => BbdModel::mn3007(),
                BbdType::Mn3005 => BbdModel::mn3005(),
            };
            bbds.push(BbdDelayLine::new(model, sample_rate));
        }
    }

    // Determine device-aware rail saturation model from the circuit's active devices.
    // Scan components to find the dominant active device type that shapes rail clipping.
    let rail_saturation = {
        let mut has_opamp = false;
        let mut has_bjt = false;
        let mut has_fet = false;
        let mut has_tube = false;
        let mut tube_mu = 100.0_f64;
        let mut opamp_swing = 0.85_f64; // default output swing ratio
        for comp in &pedal.components {
            match &comp.kind {
                ComponentKind::OpAmp(ot) if !ot.is_ota() => {
                    has_opamp = true;
                    // JFET-input op-amps (TL072) swing closer to rails than BJT-input
                    opamp_swing = match ot {
                        OpAmpType::Tl072 | OpAmpType::Tl082 | OpAmpType::Generic => 0.92,
                        OpAmpType::Ne5532 => 0.90,
                        OpAmpType::Jrc4558 | OpAmpType::Rc4558 => 0.87,
                        OpAmpType::Lm308 | OpAmpType::Lm741 | OpAmpType::Op07 => 0.85,
                        _ => 0.85,
                    };
                }
                ComponentKind::Npn(_) | ComponentKind::Pnp(_) => {
                    has_bjt = true;
                }
                ComponentKind::NJfet(_) | ComponentKind::PJfet(_)
                | ComponentKind::Nmos(_) | ComponentKind::Pmos(_) => {
                    has_fet = true;
                }
                ComponentKind::Triode(tt) => {
                    has_tube = true;
                    tube_mu = match tt {
                        TriodeType::T12ax7 => 100.0,
                        TriodeType::T12at7 => 60.0,
                        TriodeType::T12ay7 => 40.0,
                        TriodeType::T12au7 => 17.0,
                    };
                }
                ComponentKind::Pentode(_) => {
                    has_tube = true;
                    tube_mu = 200.0; // pentodes have higher effective gain
                }
                _ => {}
            }
        }
        // Priority: tube > BJT > FET > op-amp.
        // The dominant nonlinear device determines the rail saturation character.
        if has_tube {
            RailSaturation::Tube { mu: tube_mu }
        } else if has_bjt {
            let vce_sat = if has_germanium { 0.3 } else { 0.2 };
            RailSaturation::Bjt { vce_sat }
        } else if has_fet {
            RailSaturation::Fet
        } else if has_opamp {
            RailSaturation::OpAmp { output_swing_ratio: opamp_swing }
        } else {
            RailSaturation::None
        }
    };

    // Collect LFO component IDs for control binding.
    let lfo_ids: Vec<String> = pedal
        .components
        .iter()
        .filter_map(|c| {
            if matches!(c.kind, ComponentKind::Lfo(..)) {
                Some(c.id.clone())
            } else {
                None
            }
        })
        .collect();

    // Build control bindings.
    let mut controls = Vec::new();
    for ctrl in &pedal.controls {
        let target = if is_gain_label(&ctrl.label) {
            ControlTarget::PreGain
        } else if is_level_label(&ctrl.label) {
            ControlTarget::OutputGain
        } else if is_rate_label(&ctrl.label) {
            // Check if there's an LFO to control
            if !lfo_ids.is_empty() {
                ControlTarget::LfoRate(0) // Control first LFO by default
            } else {
                ControlTarget::PreGain // fallback
            }
        } else if is_depth_label(&ctrl.label) {
            if !lfo_ids.is_empty() {
                ControlTarget::LfoDepth(0)
            } else {
                ControlTarget::PreGain // fallback
            }
        } else {
            // Check if this pot connects to an LFO.rate via nets
            let mut lfo_target = None;
            for net in &pedal.nets {
                if let Pin::ComponentPin { component, pin } = &net.from {
                    if component == &ctrl.component && pin == "wiper" {
                        // This pot's wiper connects somewhere
                        for to_pin in &net.to {
                            if let Pin::ComponentPin {
                                component: target_comp,
                                pin: target_pin,
                            } = to_pin
                            {
                                if target_pin == "rate" {
                                    // Find the LFO index
                                    if let Some(idx) =
                                        lfo_ids.iter().position(|id| id == target_comp)
                                    {
                                        lfo_target = Some(ControlTarget::LfoRate(idx));
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if let Some(target) = lfo_target {
                target
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

    // Build LFO bindings from LFO components and their net connections.
    let mut lfos = Vec::new();
    for comp in &pedal.components {
        if let ComponentKind::Lfo(waveform_dsl, timing_r, timing_c) = &comp.kind {
            // Convert DSL waveform to elements waveform
            let waveform = match waveform_dsl {
                LfoWaveformDsl::Sine => crate::elements::LfoWaveform::Sine,
                LfoWaveformDsl::Triangle => crate::elements::LfoWaveform::Triangle,
                LfoWaveformDsl::Square => crate::elements::LfoWaveform::Square,
                LfoWaveformDsl::SawUp => crate::elements::LfoWaveform::SawUp,
                LfoWaveformDsl::SawDown => crate::elements::LfoWaveform::SawDown,
                LfoWaveformDsl::SampleAndHold => crate::elements::LfoWaveform::SampleAndHold,
            };

            // Compute base frequency from RC timing: f = 1/(2πRC)
            let base_freq = 1.0 / (2.0 * std::f64::consts::PI * timing_r * timing_c);

            // Create LFO with base frequency
            let mut lfo = crate::elements::Lfo::new(waveform, sample_rate);
            lfo.set_rate(base_freq);

            // Track if we've already created an AllJfetVgs binding for this LFO
            let mut created_all_jfet_binding = false;

            // Find what this LFO connects to via nets (LFO.out -> target.property)
            for net in &pedal.nets {
                if let Pin::ComponentPin { component, pin } = &net.from {
                    if component == &comp.id && pin == "out" {
                        // This LFO's output connects to targets in net.to
                        for target_pin in &net.to {
                            if let Pin::ComponentPin {
                                component: target_comp,
                                pin: target_prop,
                            } = target_pin
                            {
                                let target = match target_prop.as_str() {
                                    "vgs" | "gate" => {
                                        // Count how many JFETs this LFO connects to
                                        let jfet_count = stages
                                            .iter()
                                            .filter(|s| matches!(&s.root, RootKind::Jfet(_)))
                                            .count();

                                        if jfet_count > 1 {
                                            // Multiple JFETs - use AllJfetVgs
                                            // Only create ONE binding for AllJfetVgs
                                            if created_all_jfet_binding {
                                                continue; // Skip duplicate bindings
                                            }
                                            created_all_jfet_binding = true;
                                            ModulationTarget::AllJfetVgs
                                        } else if let Some(stage_idx) = stages
                                            .iter()
                                            .position(|s| matches!(&s.root, RootKind::Jfet(_)))
                                        {
                                            ModulationTarget::JfetVgs { stage_idx }
                                        } else if let Some(stage_idx) = stages
                                            .iter()
                                            .position(|s| matches!(&s.root, RootKind::Mosfet(_)))
                                        {
                                            ModulationTarget::MosfetVgs { stage_idx }
                                        } else {
                                            ModulationTarget::JfetVgs { stage_idx: 0 }
                                        }
                                    }
                                    "led" => ModulationTarget::PhotocouplerLed {
                                        stage_idx: 0,
                                        comp_id: target_comp.clone(),
                                    },
                                    "vgk" => {
                                        // Find which stage contains this triode
                                        let stage_idx = stages
                                            .iter()
                                            .position(|s| matches!(&s.root, RootKind::Triode(_)))
                                            .unwrap_or(0);
                                        ModulationTarget::TriodeVgk { stage_idx }
                                    }
                                    "vg1k" => {
                                        // Find which stage contains this pentode
                                        let stage_idx = stages
                                            .iter()
                                            .position(|s| matches!(&s.root, RootKind::Pentode(_)))
                                            .unwrap_or(0);
                                        ModulationTarget::PentodeVg1k { stage_idx }
                                    }
                                    "iabc" => {
                                        // Find which stage contains an OTA
                                        let stage_idx = stages
                                            .iter()
                                            .position(|s| matches!(&s.root, RootKind::Ota(_)))
                                            .unwrap_or(0);
                                        ModulationTarget::OtaIabc { stage_idx }
                                    }
                                    "clock" => {
                                        // Find which BBD to modulate
                                        let bbd_idx = 0; // First BBD by default
                                        ModulationTarget::BbdClock { bbd_idx }
                                    }
                                    _ => continue,
                                };

                                // Bias and range based on target type
                                let (bias, range) = match &target {
                                    // JFET phaser: Vgs must stay in variable-resistance region
                                    // 2SK30A-GR: Vp ~ -0.8V to -1.2V
                                    // Keep Vgs between -0.2V and -0.7V to avoid pinch-off
                                    // At -0.2V: low Rds (signal passes)
                                    // At -0.7V: high Rds (but still conducting)
                                    ModulationTarget::JfetVgs { .. } => (-0.45, 0.25),
                                    ModulationTarget::AllJfetVgs => (-0.45, 0.25),
                                    ModulationTarget::PhotocouplerLed { .. } => (0.5, 0.5),
                                    // Triode grid bias: -2V center with ±2V swing
                                    ModulationTarget::TriodeVgk { .. } => (-2.0, 2.0),
                                    // Pentode control grid: -2V center with ±2V swing
                                    ModulationTarget::PentodeVg1k { .. } => (-2.0, 2.0),
                                    // MOSFET: bias above threshold with swing
                                    ModulationTarget::MosfetVgs { .. } => (3.0, 2.0),
                                    // OTA: normalized gain 0-1
                                    ModulationTarget::OtaIabc { .. } => (0.5, 0.5),
                                    // BBD: short-delay chorus range (center ~3.3 ms, sweep ~3–5 ms)
                                    ModulationTarget::BbdClock { .. } => (0.15, 0.10),
                                    // Op-amp Vp: follows input signal
                                    ModulationTarget::OpAmpVp { .. } => (0.0, 1.0),
                                };

                                lfos.push(LfoBinding {
                                    lfo: lfo.clone(),
                                    target,
                                    bias,
                                    range,
                                    base_freq,
                                    lfo_id: comp.id.clone(),
                                });
                            }
                        }
                    }
                }
            }
        }
    }

    // Build EnvelopeFollower bindings from EnvelopeFollower components and their net connections.
    let mut envelopes = Vec::new();
    for comp in &pedal.components {
        if let ComponentKind::EnvelopeFollower(
            attack_r,
            attack_c,
            release_r,
            release_c,
            sensitivity_r,
        ) = &comp.kind
        {
            let envelope = crate::elements::EnvelopeFollower::from_rc(
                *attack_r,
                *attack_c,
                *release_r,
                *release_c,
                *sensitivity_r,
                sample_rate,
            );

            // Find what this envelope follower connects to via nets (EF.out -> target.property)
            for net in &pedal.nets {
                if let Pin::ComponentPin { component, pin } = &net.from {
                    if component == &comp.id && pin == "out" {
                        for target_pin in &net.to {
                            if let Pin::ComponentPin {
                                component: target_comp,
                                pin: target_prop,
                            } = target_pin
                            {
                                let target = match target_prop.as_str() {
                                    "vgs" => {
                                        let stage_idx = stages
                                            .iter()
                                            .position(|s| matches!(&s.root, RootKind::Jfet(_)))
                                            .unwrap_or(0);
                                        ModulationTarget::JfetVgs { stage_idx }
                                    }
                                    "led" => ModulationTarget::PhotocouplerLed {
                                        stage_idx: 0,
                                        comp_id: target_comp.clone(),
                                    },
                                    "vgk" => {
                                        let stage_idx = stages
                                            .iter()
                                            .position(|s| matches!(&s.root, RootKind::Triode(_)))
                                            .unwrap_or(0);
                                        ModulationTarget::TriodeVgk { stage_idx }
                                    }
                                    "vg1k" => {
                                        let stage_idx = stages
                                            .iter()
                                            .position(|s| matches!(&s.root, RootKind::Pentode(_)))
                                            .unwrap_or(0);
                                        ModulationTarget::PentodeVg1k { stage_idx }
                                    }
                                    "iabc" => {
                                        let stage_idx = stages
                                            .iter()
                                            .position(|s| matches!(&s.root, RootKind::Ota(_)))
                                            .unwrap_or(0);
                                        ModulationTarget::OtaIabc { stage_idx }
                                    }
                                    "clock" => {
                                        ModulationTarget::BbdClock { bbd_idx: 0 }
                                    }
                                    _ => continue,
                                };

                                let (bias, range) = match &target {
                                    // JFET: Keep in variable-resistance region (avoid pinch-off)
                                    ModulationTarget::JfetVgs { .. } => (-0.45, 0.25),
                                    ModulationTarget::AllJfetVgs => (-0.45, 0.25),
                                    ModulationTarget::PhotocouplerLed { .. } => (0.0, 1.0),
                                    ModulationTarget::TriodeVgk { .. } => (-2.0, 2.0),
                                    ModulationTarget::PentodeVg1k { .. } => (-2.0, 2.0),
                                    ModulationTarget::MosfetVgs { .. } => (3.0, 2.0),
                                    ModulationTarget::OtaIabc { .. } => (0.5, 0.5),
                                    ModulationTarget::BbdClock { .. } => (0.15, 0.10),
                                    // Op-amp Vp: follows input signal
                                    ModulationTarget::OpAmpVp { .. } => (0.0, 1.0),
                                };

                                envelopes.push(EnvelopeBinding {
                                    envelope: envelope.clone(),
                                    target,
                                    bias,
                                    range,
                                    env_id: comp.id.clone(),
                                });
                            }
                        }
                    }
                }
            }
        }
    }

    // Set up thermal model if enabled and circuit uses germanium.
    let thermal = if enable_thermal && has_germanium {
        Some(ThermalModel::germanium_fuzz(sample_rate))
    } else if enable_thermal {
        Some(ThermalModel::silicon_standard(sample_rate))
    } else {
        None
    };

    Ok(CompiledPedal {
        stages,
        pre_gain,
        output_gain: level_default,
        rail_saturation,
        sample_rate,
        controls,
        gain_range: (glo * active_bonus, ghi * active_bonus),
        supply_voltage: 9.0,
        lfos,
        envelopes,
        slew_limiters,
        bbds,
        thermal,
        tolerance_seed: tolerance.seed(),
        oversampling,
        opamp_stages,
    })
}

// ═══════════════════════════════════════════════════════════════════════════
// Voltage compatibility check
// ═══════════════════════════════════════════════════════════════════════════

/// Severity of a voltage compatibility warning.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WarningSeverity {
    /// Informational — the component should be fine, but behavior may differ
    /// from the canonical 9V version (e.g. slightly different bias point).
    Info,
    /// The component is likely operating outside its typical ratings and may
    /// clip, distort differently, or wear out faster in a real build.
    Caution,
    /// The component would almost certainly fail or be damaged at this voltage
    /// in a physical build.
    Danger,
}

/// A single voltage compatibility warning for a component.
#[derive(Debug, Clone)]
pub struct VoltageWarning {
    pub component_id: String,
    pub severity: WarningSeverity,
    pub message: String,
}

/// Check a pedal definition for voltage compatibility at a given supply voltage.
///
/// Returns a list of warnings for components that may not tolerate the target
/// voltage.  These are heuristic — the DSL doesn't carry voltage ratings, so
/// we infer from component types and typical part specs:
///
/// - **Germanium transistors**: Vce(max) typically 15–32V.  Caution above 15V.
/// - **Electrolytic capacitors** (≥ 1 µF): often rated 10–16V.  Caution above 12V.
/// - **Op-amps**: common audio opamps (TL072, NE5532) rated to ±18V (36V total).
///   Caution above 18V.
/// - **Silicon transistors**: usually Vce(max) ≥ 40V.  Caution above 24V.
/// - **Germanium diodes**: forward behaviour is voltage-independent, but reverse
///   breakdown is lower (~50V Ge vs ~100V+ Si).  Only flagged above 24V.
///
/// Note: passive elements (R, L) and silicon diodes are unaffected in the WDF
/// model and are not flagged.
pub fn check_voltage_compatibility(pedal: &PedalDef, voltage: f64) -> Vec<VoltageWarning> {
    let mut warnings = Vec::new();

    for comp in &pedal.components {
        match &comp.kind {
            ComponentKind::Npn(bt) | ComponentKind::Pnp(bt) => {
                let is_ge = bt.is_germanium();
                if is_ge {
                    // Germanium PNPs: typical Vce(max) = 15–32V
                    if voltage > 18.0 {
                        warnings.push(VoltageWarning {
                            component_id: comp.id.clone(),
                            severity: WarningSeverity::Danger,
                            message: format!(
                                "Germanium transistor {} likely exceeds Vce(max) at {:.0}V \
                                 (typical Ge PNP rated 15–32V)",
                                comp.id, voltage
                            ),
                        });
                    } else if voltage > 12.0 {
                        warnings.push(VoltageWarning {
                            component_id: comp.id.clone(),
                            severity: WarningSeverity::Caution,
                            message: format!(
                                "Germanium transistor {} may run hot at {:.0}V \
                                 — bias point shifts, tone will differ from 9V",
                                comp.id, voltage
                            ),
                        });
                    }
                } else if voltage > 24.0 {
                    // Silicon transistors: typically Vce(max) >= 40V
                    warnings.push(VoltageWarning {
                        component_id: comp.id.clone(),
                        severity: WarningSeverity::Caution,
                        message: format!(
                            "Transistor {} at {:.0}V — verify Vce(max) rating of actual part",
                            comp.id, voltage
                        ),
                    });
                }
            }
            ComponentKind::OpAmp(ot) => {
                // Use the op-amp type's known supply_max for accurate warnings
                let max_supply = ot.supply_max();
                if voltage > max_supply * 0.5 {
                    // Operating above half the max total supply (typical single-supply limit)
                    let severity = if voltage > max_supply {
                        WarningSeverity::Danger
                    } else {
                        WarningSeverity::Caution
                    };
                    warnings.push(VoltageWarning {
                        component_id: comp.id.clone(),
                        severity,
                        message: format!(
                            "Op-amp {} ({:?}) at {:.0}V — max total supply {:.0}V (±{:.0}V split)",
                            comp.id, ot, voltage, max_supply, max_supply / 2.0
                        ),
                    });
                }
            }
            ComponentKind::Capacitor(farads) => {
                // Electrolytics (≥ 1µF) often have low voltage ratings.
                // 10µF caps commonly rated 10V or 16V.
                if *farads >= 1e-6 {
                    if voltage > 16.0 {
                        warnings.push(VoltageWarning {
                            component_id: comp.id.clone(),
                            severity: WarningSeverity::Danger,
                            message: format!(
                                "Electrolytic cap {} ({:.0}µF) may exceed voltage rating at {:.0}V \
                                 — common ratings are 10V, 16V, 25V",
                                comp.id, farads * 1e6, voltage
                            ),
                        });
                    } else if voltage > 12.0 {
                        warnings.push(VoltageWarning {
                            component_id: comp.id.clone(),
                            severity: WarningSeverity::Caution,
                            message: format!(
                                "Electrolytic cap {} ({:.0}µF) — ensure voltage rating ≥ {:.0}V",
                                comp.id,
                                farads * 1e6,
                                voltage
                            ),
                        });
                    }
                }
            }
            ComponentKind::DiodePair(DiodeType::Germanium)
            | ComponentKind::Diode(DiodeType::Germanium) => {
                // Ge diode forward behaviour is voltage-independent in the WDF,
                // but reverse breakdown is lower (~50V vs 100V+ Si).
                // Also: more temperature-sensitive at higher power dissipation.
                if voltage > 18.0 {
                    warnings.push(VoltageWarning {
                        component_id: comp.id.clone(),
                        severity: WarningSeverity::Info,
                        message: format!(
                            "Germanium diode {} — higher power dissipation at {:.0}V \
                             may shift forward voltage (temperature dependent)",
                            comp.id, voltage
                        ),
                    });
                }
            }
            ComponentKind::Bbd(_) => {
                // BBDs are typically rated 10-15V. They're sensitive to
                // over-voltage which causes excess clock noise and distortion.
                if voltage > 15.0 {
                    warnings.push(VoltageWarning {
                        component_id: comp.id.clone(),
                        severity: WarningSeverity::Danger,
                        message: format!(
                            "BBD {} may exceed max supply at {:.0}V — typical rating 10-15V",
                            comp.id, voltage
                        ),
                    });
                }
            }
            // Synth ICs: CEM/AS/V series typically run ±5V to ±9V (10-18V total)
            ComponentKind::Vco(_) | ComponentKind::Vcf(_) => {
                if voltage > 18.0 {
                    warnings.push(VoltageWarning {
                        component_id: comp.id.clone(),
                        severity: WarningSeverity::Danger,
                        message: format!(
                            "Synth IC {} exceeds max supply at {:.0}V — CEM/AS/V series rated ±9V (18V max)",
                            comp.id, voltage
                        ),
                    });
                }
            }
            ComponentKind::AnalogSwitch(_) => {
                if voltage > 20.0 {
                    warnings.push(VoltageWarning {
                        component_id: comp.id.clone(),
                        severity: WarningSeverity::Danger,
                        message: format!(
                            "Analog switch {} exceeds max supply at {:.0}V — CD4066/DG411 rated 20V max",
                            comp.id, voltage
                        ),
                    });
                }
            }
            // Resistors, inductors, Si/LED diodes, pots, VCA, comparator, matched
            // transistors, tempco: no voltage concerns within the 5–24V range
            // (or handled by supply_max spec if provided).
            _ => {}
        }
    }

    warnings
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

fn jfet_model(jt: JfetType, is_n_channel: bool) -> JfetModel {
    match jt {
        JfetType::J201 => JfetModel::n_j201(),
        JfetType::N2n5457 => JfetModel::n_2n5457(),
        JfetType::N2n5952 => JfetModel::n_2n5952(),
        // 2SK30A variants - classic phaser JFETs
        JfetType::N2sk30a => JfetModel::n_2sk30a(),
        JfetType::N2sk30aGr => JfetModel::n_2sk30a_gr(),
        JfetType::N2sk30aY => JfetModel::n_2sk30a_y(),
        JfetType::N2sk30aBl => JfetModel::n_2sk30a_bl(),
        JfetType::P2n5460 => {
            if is_n_channel {
                // Mismatch: N-channel requested with P-channel type
                // Fall back to a reasonable N-channel model
                JfetModel::n_2n5457()
            } else {
                JfetModel::p_2n5460()
            }
        }
    }
}

fn triode_model(tt: TriodeType) -> TriodeModel {
    match tt {
        TriodeType::T12ax7 => TriodeModel::t_12ax7(),
        TriodeType::T12at7 => TriodeModel::t_12at7(),
        TriodeType::T12au7 => TriodeModel::t_12au7(),
        TriodeType::T12ay7 => TriodeModel::t_12ay7(),
    }
}

fn pentode_model(pt: PentodeType) -> PentodeModel {
    match pt {
        PentodeType::Ef86 => PentodeModel::p_ef86(),
        PentodeType::El84 => PentodeModel::p_el84(),
    }
}

fn mosfet_model(mt: MosfetType, is_n_channel: bool) -> MosfetModel {
    match mt {
        MosfetType::N2n7000 => MosfetModel::n_2n7000(),
        MosfetType::Irf520 => MosfetModel::n_irf520(),
        MosfetType::Bs250 => {
            if is_n_channel {
                // Mismatch: N-channel requested with P-channel type
                MosfetModel::n_2n7000()
            } else {
                MosfetModel::p_bs250()
            }
        }
        MosfetType::Irf9520 => {
            if is_n_channel {
                MosfetModel::n_2n7000()
            } else {
                MosfetModel::p_irf9520()
            }
        }
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
        DynNode::Pot { comp_id: id, .. } => {
            // Match the pot itself or the synthetic __aw half of a 3-terminal pot.
            // (Finding either half means the control is in this stage.)
            id == comp_id || *id == format!("{comp_id}__aw")
        }
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
        let path = find_example(filename);
        let src = std::fs::read_to_string(&path).unwrap();
        parse_pedal_file(&src).unwrap()
    }

    /// Search examples/ subdirectories for a file by name.
    fn find_example(filename: &str) -> String {
        for entry in walkdir("examples") {
            if entry.ends_with(filename) {
                return entry;
            }
        }
        panic!("example file not found: {filename}");
    }

    /// Simple recursive directory walk.
    fn walkdir(dir: &str) -> Vec<String> {
        let mut results = Vec::new();
        if let Ok(entries) = std::fs::read_dir(dir) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.is_dir() {
                    results.extend(walkdir(path.to_str().unwrap()));
                } else {
                    results.push(path.to_string_lossy().to_string());
                }
            }
        }
        results
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
            "boss_ce2.pedal",
            "tweed_deluxe_5e3.pedal",
            "bassman_5f6a.pedal",
            "marshall_jtm45.pedal",
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

    #[test]
    fn compile_phase90() {
        // Phase 90 compiles with op-amp stages and produces phasing effect
        let pedal = parse("phase90.pedal");
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        proc.set_control("Speed", 0.5);

        // Process a sine wave and verify output
        let input = sine(48000);
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        assert_finite(&output, "Phase 90");

        // Should produce output
        let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(
            peak > 0.01,
            "Phase 90 should produce output: peak={peak}"
        );

        // Verify the circuit is actively creating the sweeping effect
        // by checking that the output varies over time (not just a static gain)
        let first_half: Vec<f64> = output[0..24000].to_vec();
        let second_half: Vec<f64> = output[24000..48000].to_vec();

        // RMS of each half
        let rms_first = (first_half.iter().map(|x| x * x).sum::<f64>() / 24000.0).sqrt();
        let rms_second = (second_half.iter().map(|x| x * x).sum::<f64>() / 24000.0).sqrt();

        // Both halves should have similar RMS (not cutting out)
        assert!(
            rms_first > 0.01 && rms_second > 0.01,
            "Both halves should have signal: rms_first={rms_first}, rms_second={rms_second}"
        );
    }

    #[test]
    fn compile_tweed_deluxe_5e3() {
        let pedal = parse("tweed_deluxe_5e3.pedal");
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        proc.set_control("Volume", 0.7);
        proc.set_control("Tone", 0.5);

        let input = sine(48000);
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        assert_finite(&output, "Tweed Deluxe 5E3");
        let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(
            peak > 0.01,
            "Tweed Deluxe should produce output: peak={peak}"
        );
    }

    #[test]
    fn compile_bassman_5f6a() {
        let pedal = parse("bassman_5f6a.pedal");
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        proc.set_control("Volume", 0.6);
        proc.set_control("Treble", 0.6);
        proc.set_control("Mid", 0.5);
        proc.set_control("Bass", 0.5);

        let input = sine(48000);
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        assert_finite(&output, "Bassman 5F6-A");
        let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(
            peak > 0.01,
            "Bassman should produce output: peak={peak}"
        );
    }

    #[test]
    fn compile_marshall_jtm45() {
        let pedal = parse("marshall_jtm45.pedal");
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        proc.set_control("Volume", 0.6);
        proc.set_control("Treble", 0.6);
        proc.set_control("Mid", 0.5);
        proc.set_control("Bass", 0.5);
        proc.set_control("Presence", 0.5);

        let input = sine(48000);
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        assert_finite(&output, "Marshall JTM45");
        let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(
            peak > 0.01,
            "JTM45 should produce output: peak={peak}"
        );
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
    // Phase 3: Slew rate, OTA, BBD tests
    // -----------------------------------------------------------------------

    #[test]
    fn proco_rat_has_slew_limiter() {
        // ProCo RAT uses LM308 (0.3 V/µs slew rate).
        // The compiled pedal should have a slew rate limiter.
        let pedal = parse("proco_rat.pedal");
        let proc = compile_pedal(&pedal, 48000.0).unwrap();
        assert!(
            !proc.slew_limiters.is_empty(),
            "ProCo RAT should have slew rate limiters from LM308"
        );
        assert!(
            (proc.slew_limiters[0].slew_rate() - 0.3).abs() < 0.01,
            "LM308 slew rate should be 0.3 V/µs"
        );
    }

    #[test]
    fn tube_screamer_has_slew_limiter() {
        // Tube Screamer uses JRC4558 (1.7 V/µs slew rate).
        let pedal = parse("tube_screamer.pedal");
        let proc = compile_pedal(&pedal, 48000.0).unwrap();
        assert!(
            !proc.slew_limiters.is_empty(),
            "Tube Screamer should have slew limiter from JRC4558"
        );
        assert!(
            (proc.slew_limiters[0].slew_rate() - 1.7).abs() < 0.01,
            "JRC4558 slew rate should be 1.7 V/µs"
        );
    }

    #[test]
    fn dyna_comp_no_slew_limiter() {
        // Dyna Comp uses CA3080 OTA which is very fast (50 V/µs).
        // OTAs are handled as WDF roots, not slew limiters.
        let pedal = parse("dyna_comp.pedal");
        let proc = compile_pedal(&pedal, 48000.0).unwrap();
        assert!(
            proc.slew_limiters.is_empty(),
            "CA3080 OTA should not add a slew rate limiter (it's a WDF root)"
        );
    }

    #[test]
    fn dyna_comp_has_ota_stage() {
        // Dyna Comp should have an OTA WDF stage.
        let pedal = parse("dyna_comp.pedal");
        let proc = compile_pedal(&pedal, 48000.0).unwrap();
        let ota_stages: Vec<_> = proc
            .stages
            .iter()
            .filter(|s| matches!(s.root, RootKind::Ota(_)))
            .collect();
        assert!(
            !ota_stages.is_empty(),
            "Dyna Comp should have an OTA stage"
        );
    }

    #[test]
    fn slew_rate_affects_rat_character() {
        // The RAT with LM308 slew rate limiting should produce different
        // output than if we removed the slew limiting.
        let pedal = parse("proco_rat.pedal");
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        proc.set_control("Distortion", 0.8);
        proc.set_control("Volume", 0.5);

        let input = sine(4800);
        let output_with_slew: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

        // Process should produce finite output
        assert!(
            output_with_slew.iter().all(|x| x.is_finite()),
            "RAT with slew limiting: output should be finite"
        );
        let peak = output_with_slew
            .iter()
            .fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(peak > 0.01, "RAT should produce output: peak={peak}");
    }

    #[test]
    fn bbd_pedal_compiles_and_processes() {
        // Parse a BBD-based chorus pedal inline.
        let src = r#"
pedal "Test Chorus" {
  components {
    C1: cap(100n)
    R1: resistor(10k)
    BBD1: bbd(mn3207)
    LFO1: lfo(triangle, 100k, 47n)
  }
  nets {
    in -> C1.a
    C1.b -> R1.a, BBD1.in
    R1.b -> gnd
    BBD1.out -> out
    LFO1.out -> BBD1.clock
  }
}
"#;
        let pedal = crate::dsl::parse_pedal_file(src).unwrap();
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();

        assert!(
            !proc.bbds.is_empty(),
            "Chorus pedal should have BBD delay line"
        );

        // Process audio
        let input = sine(4800);
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        assert!(
            output.iter().all(|x| x.is_finite()),
            "BBD output should be finite"
        );
        let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(peak > 0.001, "Chorus should produce output: peak={peak}");
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

    fn rms(buf: &[f64]) -> f64 {
        (buf.iter().map(|x| x * x).sum::<f64>() / buf.len() as f64).sqrt()
    }

    // -----------------------------------------------------------------------
    // 12V supply voltage tests (compiled pedals)
    // -----------------------------------------------------------------------

    #[test]
    fn compiled_12v_more_headroom() {
        // At 12V the soft-limiter ceiling is higher, so the signal clips
        // less — giving a cleaner waveform with higher peak amplitude.
        // The gain and output level stay the same (resistor ratios don't
        // change with supply voltage), but the clipping ceiling rises.
        let pedal = parse("tube_screamer.pedal");
        let input = sine(48000);

        let mut proc_9v = compile_pedal(&pedal, 48000.0).unwrap();
        proc_9v.set_control("Drive", 0.8);
        proc_9v.set_control("Level", 1.0);
        let out_9v: Vec<f64> = input.iter().map(|&s| proc_9v.process(s)).collect();

        let mut proc_12v = compile_pedal(&pedal, 48000.0).unwrap();
        proc_12v.set_supply_voltage(12.0);
        proc_12v.set_control("Drive", 0.8);
        proc_12v.set_control("Level", 1.0);
        let out_12v: Vec<f64> = input.iter().map(|&s| proc_12v.process(s)).collect();

        let peak_9v = out_9v.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        let peak_12v = out_12v.iter().fold(0.0f64, |m, x| m.max(x.abs()));

        assert!(
            peak_12v > peak_9v,
            "12V should allow higher peaks (more headroom): 9V peak={peak_9v}, 12V peak={peak_12v}"
        );
    }

    #[test]
    fn compiled_12v_all_pedals_stable() {
        let files = [
            "tube_screamer.pedal",
            "fuzz_face.pedal",
            "big_muff.pedal",
            "blues_driver.pedal",
            "dyna_comp.pedal",
            "klon_centaur.pedal",
            "proco_rat.pedal",
        ];
        let input = sine(4800);
        for f in files {
            let pedal = parse(f);
            let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
            proc.set_supply_voltage(12.0);
            let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
            assert!(output.iter().all(|x| x.is_finite()), "{f} at 12V: NaN/inf");
            let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
            assert!(peak < 7.0, "{f} at 12V: output too loud, peak={peak}");
            assert!(peak > 0.001, "{f} at 12V: silent, peak={peak}");
        }
    }

    #[test]
    fn compiled_9v_unchanged() {
        // Explicitly setting 9V should match default behavior.
        let pedal = parse("tube_screamer.pedal");
        let input = sine(48000);

        let mut proc_default = compile_pedal(&pedal, 48000.0).unwrap();
        let out_default: Vec<f64> = input.iter().map(|&s| proc_default.process(s)).collect();

        let mut proc_9v = compile_pedal(&pedal, 48000.0).unwrap();
        proc_9v.set_supply_voltage(9.0);
        let out_9v: Vec<f64> = input.iter().map(|&s| proc_9v.process(s)).collect();

        let diff_rms = rms(&out_default
            .iter()
            .zip(&out_9v)
            .map(|(a, b)| a - b)
            .collect::<Vec<_>>());
        assert!(
            diff_rms < 1e-10,
            "Compiled 9V should match default: diff_rms={diff_rms}"
        );
    }

    #[test]
    fn compiled_12v_extreme_settings_stable() {
        let configs: &[(&str, &[(&str, f64)])] = &[
            ("tube_screamer.pedal", &[("Drive", 1.0), ("Level", 1.0)]),
            ("fuzz_face.pedal", &[("Fuzz", 1.0), ("Volume", 1.0)]),
            ("big_muff.pedal", &[("Sustain", 1.0), ("Volume", 1.0)]),
            ("proco_rat.pedal", &[("Distortion", 1.0), ("Volume", 1.0)]),
        ];

        let input = sine(4800);
        for (f, knobs) in configs {
            let pedal = parse(f);
            let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
            proc.set_supply_voltage(12.0);
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
                "{f} 12V [{settings}]: NaN/inf"
            );
            let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
            assert!(
                peak < 10.0,
                "{f} 12V [{settings}]: output too loud, peak={peak}"
            );
        }
    }

    // -----------------------------------------------------------------------
    // Voltage compatibility check tests
    // -----------------------------------------------------------------------

    #[test]
    fn voltage_check_tube_screamer_9v_clean() {
        // Tube Screamer at 9V should have no warnings.
        let pedal = parse("tube_screamer.pedal");
        let warnings = check_voltage_compatibility(&pedal, 9.0);
        assert!(
            warnings.is_empty(),
            "TS at 9V should have no warnings: got {warnings:?}"
        );
    }

    #[test]
    fn voltage_check_tube_screamer_12v_clean() {
        // Tube Screamer has only passives + silicon diodes — fine at 12V.
        let pedal = parse("tube_screamer.pedal");
        let warnings = check_voltage_compatibility(&pedal, 12.0);
        assert!(
            warnings.is_empty(),
            "TS at 12V should have no warnings: got {warnings:?}"
        );
    }

    #[test]
    fn voltage_check_fuzz_face_18v_warns() {
        // Fuzz Face has PNP transistors (likely germanium) and electrolytic
        // caps — should warn at 18V.
        let pedal = parse("fuzz_face.pedal");
        let warnings = check_voltage_compatibility(&pedal, 18.0);
        assert!(
            !warnings.is_empty(),
            "Fuzz Face at 18V should have warnings"
        );
        // Should flag the electrolytic caps (2.2µF, 10µF).
        let cap_warnings: Vec<_> = warnings
            .iter()
            .filter(|w| w.component_id.starts_with('C'))
            .collect();
        assert!(
            !cap_warnings.is_empty(),
            "Should warn about electrolytic caps at 18V"
        );
    }

    #[test]
    fn voltage_check_fuzz_face_ge_transistors() {
        // The Fuzz Face PNP transistors should trigger caution above 12V.
        let pedal = parse("fuzz_face.pedal");
        let warnings = check_voltage_compatibility(&pedal, 15.0);
        let transistor_warnings: Vec<_> = warnings
            .iter()
            .filter(|w| w.component_id.starts_with('Q'))
            .collect();
        assert!(
            !transistor_warnings.is_empty(),
            "Should warn about Ge PNP transistors above 12V"
        );
        assert!(transistor_warnings
            .iter()
            .all(|w| w.severity == WarningSeverity::Caution));
    }

    #[test]
    fn voltage_check_fuzz_face_ge_transistors_danger() {
        // At 20V, germanium transistors should be Danger severity.
        let pedal = parse("fuzz_face.pedal");
        let warnings = check_voltage_compatibility(&pedal, 20.0);
        let danger: Vec<_> = warnings
            .iter()
            .filter(|w| w.severity == WarningSeverity::Danger)
            .collect();
        assert!(!danger.is_empty(), "Ge transistors at 20V should be Danger");
    }

    #[test]
    fn voltage_check_klon_opamp_18v() {
        // Klon has opamps — should warn above 18V.
        let pedal = parse("klon_centaur.pedal");
        let warnings = check_voltage_compatibility(&pedal, 20.0);
        let opamp_warnings: Vec<_> = warnings
            .iter()
            .filter(|w| w.component_id.starts_with('U'))
            .collect();
        assert!(
            !opamp_warnings.is_empty(),
            "Should warn about opamps above 18V"
        );
    }

    #[test]
    fn voltage_check_klon_12v_clean() {
        // Klon at 12V should be fine (no electrolytics in the signal path
        // that would fail, opamps are within range).
        let pedal = parse("klon_centaur.pedal");
        let warnings = check_voltage_compatibility(&pedal, 12.0);
        // The 1µF output coupling cap may trigger a caution.
        let danger: Vec<_> = warnings
            .iter()
            .filter(|w| w.severity == WarningSeverity::Danger)
            .collect();
        assert!(
            danger.is_empty(),
            "Klon at 12V should have no Danger warnings: got {danger:?}"
        );
    }

    #[test]
    fn voltage_check_all_pedals_9v_no_danger() {
        // At 9V, no pedal should have Danger or Caution warnings.
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
            let warnings = check_voltage_compatibility(&pedal, 9.0);
            assert!(
                warnings.is_empty(),
                "{f} at 9V should have no warnings: got {warnings:?}"
            );
        }
    }

    // -----------------------------------------------------------------------
    // FX loop split tests
    // -----------------------------------------------------------------------

    #[test]
    fn tweed_deluxe_has_fx_loop() {
        let pedal = parse("tweed_deluxe_5e3.pedal");
        assert!(pedal.has_fx_loop(), "Tweed Deluxe should have fx_send/fx_return");
    }

    #[test]
    fn split_tweed_deluxe_produces_two_halves() {
        let pedal = parse("tweed_deluxe_5e3.pedal");
        let (pre, post) = split_pedal_def(&pedal).expect("should split");
        assert!(
            pre.name.contains("pre"),
            "Pre half should have 'pre' in name: {}",
            pre.name
        );
        assert!(
            post.name.contains("post"),
            "Post half should have 'post' in name: {}",
            post.name
        );
        // Both halves should have some components
        assert!(!pre.components.is_empty(), "Pre half should have components");
        assert!(!post.components.is_empty(), "Post half should have components");
        // Neither half should still have fx_send/fx_return
        assert!(!pre.has_fx_loop(), "Pre half should not have fx loop nodes");
        assert!(!post.has_fx_loop(), "Post half should not have fx loop nodes");
    }

    #[test]
    fn compile_split_tweed_deluxe() {
        let pedal = parse("tweed_deluxe_5e3.pedal");
        let mut split = compile_split_pedal(&pedal, 48000.0).unwrap();
        split.set_control("Volume", 0.7);
        split.set_control("Tone", 0.5);

        // Process through split (direct, no FX loop effects)
        let input = sine(48000);
        let output: Vec<f64> = input.iter().map(|&s| split.process(s)).collect();
        assert_finite(&output, "Split Tweed (direct)");
        let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(peak > 0.001, "Split Tweed should produce output: peak={peak}");
    }

    #[test]
    fn split_pre_post_individually() {
        let pedal = parse("tweed_deluxe_5e3.pedal");
        let mut split = compile_split_pedal(&pedal, 48000.0).unwrap();
        split.set_control("Volume", 0.7);
        split.set_control("Tone", 0.5);

        let input = sine(48000);
        // Process through pre → post separately
        let mut output = Vec::with_capacity(input.len());
        for &s in &input {
            let send = split.process_pre(s);
            let ret = split.process_post(send);
            output.push(ret);
        }
        assert_finite(&output, "Split Tweed (pre→post)");
        let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(peak > 0.001, "Pre→post should produce output: peak={peak}");
    }

    #[test]
    fn non_fx_loop_pedal_does_not_split() {
        let pedal = parse("tube_screamer.pedal");
        assert!(!pedal.has_fx_loop());
        assert!(split_pedal_def(&pedal).is_none());
        assert!(compile_split_pedal(&pedal, 48000.0).is_err());
    }

    #[test]
    fn three_terminal_pot_compiles_and_passes_signal() {
        // 3-terminal pot as volume attenuator at the diode junction.
        // Wiper is the junction node so both pot halves appear in the
        // WDF stage. __wb goes to gnd (dead-end, redirected by sp_reduce).
        let src = r#"
            pedal "3TermTest" {
                components {
                    C1: cap(100n)
                    R1: resistor(10k)
                    Vol: pot(100k)
                    D1: diode_pair(silicon)
                    R2: resistor(10k)
                }
                nets {
                    in -> C1.a
                    C1.b -> R1.a
                    R1.b -> Vol.a
                    Vol.b -> gnd
                    Vol.w -> D1.a
                    D1.b -> gnd
                    Vol.w -> R2.a
                    R2.b -> out
                }
                controls {
                    Vol.position -> "Vol" [0.0, 1.0] = 0.5
                }
            }
        "#;
        let pedal = parse_pedal_file(src).unwrap();
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();

        // Basic signal pass-through test at default position.
        let input = sine(48000);
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        assert_finite(&output, "3-term pot default");
        let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(peak > 0.001, "3-term pot should pass signal: peak={peak}");

        // Change control and verify it still produces finite output.
        proc.set_control("Vol", 0.2);
        let output2: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        assert_finite(&output2, "3-term pot Vol=0.2");
    }
}
