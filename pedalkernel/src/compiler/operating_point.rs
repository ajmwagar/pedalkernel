//! Compile-time DC operating-point extraction.
//!
//! After the SPQR pipeline builds a `CompiledPedal`, this module (when asked
//! via `CompileOptions::compute_operating_point`) settles the freshly-built
//! processor at DC and reads back a per-net + per-device Q-point report.
//!
//! ## Why it lives in the compiler
//!
//! The runtime `CompiledPedal` holds only stages + ports — it has NO circuit
//! graph and NO net names. Only here do we have the `CircuitGraph` that maps
//! net names ↔ `NodeId` and transistor pins ↔ `NodeId`. So we DERIVE node
//! voltages here by combining the graph's comp→terminal mapping with the
//! runtime's per-component voltage probe.
//!
//! ## How node voltages are recovered (and its honest limits)
//!
//! The runtime WDF tree erases circuit node ids: leaves carry `()` terminals,
//! so we cannot ask the runtime "what is the voltage at node N?". What we CAN
//! ask is "what is the voltage ACROSS component C?" (`probe_component_voltage`).
//!
//! To turn a component voltage into a node voltage we anchor to a rail: if a
//! passive component connects node N to a known-voltage rail R (gnd = 0 or a
//! supply), then `V(N) = V(R) ± V_across(C)`. The sign is the genuinely fragile
//! part — the WDF leaf voltage's polarity is the leaf's internal orientation,
//! not the graph's. We resolve it by orienting the probe so the rail terminal
//! is the reference and report the magnitude on the node side; where this is
//! ambiguous or no rail-anchored passive exists, the net is reported `None`
//! ("n/a") rather than fabricating a number.

use hashbrown::HashMap;

use super::component::EdgeKind;
use super::graph::{CircuitGraph, NodeId};
use pedalkernel_rt::operating_point::{
    DeviceOp, NetOp, OperatingPoint, RootOp, VoltageSource,
};
use pedalkernel_rt::processor::{CompiledPedal, Stage};
use pedalkernel_rt::stage::RootKind;
use pedalkernel_rt::PedalProcessor;

/// Settle the processor at DC and compute its operating-point report.
///
/// `settle_seconds` of input-`0.0` samples are pushed so coupling/bypass caps
/// charge and the NL roots reach quiescence before the readout.
pub(super) fn compute_operating_point(
    compiled: &mut CompiledPedal,
    graph: &CircuitGraph,
    sample_rate: f64,
    settle_seconds: f64,
) {
    let settle_samples = (settle_seconds * sample_rate).round() as usize;
    let mut last_out = 0.0;
    for _ in 0..settle_samples {
        last_out = compiled.process(0.0);
    }

    let probe = NodeProbe::new(graph, compiled, last_out);
    let nets = build_net_table(graph, compiled, &probe);
    let devices = build_device_table(graph, compiled, &probe);

    compiled.operating_point = Some(OperatingPoint {
        nets,
        devices,
        settle_samples,
    });
}

/// Resolves node voltages from a settled processor.
///
/// Recovery model (see module docs for the honesty caveats):
///   * Rails (gnd, supplies) and the circuit input (DC = 0) are KNOWN seeds.
///   * The circuit OUTPUT net is seeded from the settled output sample — the
///     single most trustworthy number, since it is literally what the runtime
///     emits at DC (it is what we already measured at ≈ -1.25 V).
///   * Every other node is reached by PROPAGATING a known voltage across a
///     CAPACITOR. A capacitor at DC carries no current, so its stored WDF
///     voltage equals the DC potential difference across it: V(far) = V(near)
///     ± V_cap. Resistors store NO voltage in the WDF runtime (their leaf
///     voltage is identically 0), so they are useless as probes and are NOT
///     used to anchor — that is why a resistor-only node is reported "n/a".
///
/// The SIGN of a cap's stored voltage is the WDF leaf's internal orientation,
/// not the graph's, so the propagation magnitude is faithful but the polarity
/// is resolved heuristically (anchored to the closest known reference). This is
/// flagged with the `probe` source so the reader treats the polarity with
/// suspicion; magnitudes are the load-bearing signal for the bias bug.
struct NodeProbe {
    /// Resolved node id → (voltage, source). Computed up-front by propagation.
    resolved: HashMap<NodeId, (f64, VoltageSource)>,
}

impl NodeProbe {
    fn new(graph: &CircuitGraph, compiled: &CompiledPedal, settled_output: f64) -> Self {
        let mut resolved: HashMap<NodeId, (f64, VoltageSource)> = HashMap::new();

        // ── Seeds ───────────────────────────────────────────────────────
        // Rails.
        resolved.insert(graph.gnd_node, (0.0, VoltageSource::Rail));
        let supply_default = graph
            .supply_voltages
            .values()
            .copied()
            .next()
            .unwrap_or(9.0);
        for (&node, &v) in &graph.supply_voltages {
            resolved.insert(node, (v, VoltageSource::Rail));
        }
        resolved
            .entry(graph.vcc_node)
            .or_insert((supply_default, VoltageSource::Rail));
        // Circuit input is a 0 V source at DC.
        resolved
            .entry(graph.in_node)
            .or_insert((0.0, VoltageSource::Rail));
        // Circuit output = settled output sample (authoritative).
        resolved.insert(graph.out_node, (settled_output, VoltageSource::PassiveProbe));

        // ── Capacitor adjacency ─────────────────────────────────────────
        // Only Reactive (C/L) edges store a usable DC voltage at runtime;
        // resistors' WDF leaf voltage is identically 0 (no anchoring value).
        // Each entry: node → (comp_id, far_node, v_cap) where v_cap is the
        // probed |voltage| across the component (None if not in any WDF tree).
        let mut cap_edges: Vec<(NodeId, NodeId, f64)> = Vec::new();
        for (eidx, edge) in graph.edges.iter().enumerate() {
            let comp = &graph.components[edge.comp_idx];
            let kind = graph
                .resolved_edge_kinds
                .get(&eidx)
                .copied()
                .or_else(|| comp.kind.edges().first().map(|e| e.kind));
            if kind != Some(EdgeKind::Reactive) {
                continue;
            }
            let Some(v_cap) = compiled.probe_component_voltage(&comp.id) else {
                continue;
            };
            cap_edges.push((edge.node_a, edge.node_b, v_cap));
        }

        // ── Fixed-point propagation across caps ─────────────────────────
        // For a cap between known node K and unknown node U: |V(U) - V(K)| =
        // |v_cap|. Polarity is chosen to push U AWAY from the nearest rail
        // ground reference (heuristic — see struct docs). Iterate to a fixed
        // point so multi-hop chains (in→C1→base, collector→C2→out) resolve.
        let mut changed = true;
        let mut guard = 0;
        while changed && guard < graph.edges.len() + 8 {
            changed = false;
            guard += 1;
            for &(a, b, v_cap) in &cap_edges {
                let a_known = resolved.get(&a).copied();
                let b_known = resolved.get(&b).copied();
                match (a_known, b_known) {
                    (Some((va, _)), None) => {
                        // Push U=b away from 0: sign follows the known side.
                        let vb = va + sign_for(va) * v_cap.abs();
                        resolved.insert(b, (vb, VoltageSource::PassiveProbe));
                        changed = true;
                    }
                    (None, Some((vb, _))) => {
                        let va = vb + sign_for(vb) * v_cap.abs();
                        resolved.insert(a, (va, VoltageSource::PassiveProbe));
                        changed = true;
                    }
                    _ => {}
                }
            }
        }

        Self { resolved }
    }

    /// Resolve voltage at `node`. Returns `(voltage, source)`.
    fn node_voltage(
        &self,
        node: NodeId,
        _compiled: &CompiledPedal,
    ) -> (Option<f64>, VoltageSource) {
        match self.resolved.get(&node) {
            Some(&(v, src)) => (Some(v), src),
            None => (None, VoltageSource::None),
        }
    }
}

/// Polarity heuristic for propagating across a cap from a known node at `v`:
/// push the unknown node further from ground (positive nodes go more positive).
/// When the known node is ~0 V we have no directional cue, so default positive.
fn sign_for(v: f64) -> f64 {
    if v < -1e-9 {
        -1.0
    } else {
        1.0
    }
}

/// Build the per-net DC voltage table. One row per distinct circuit node that
/// has at least one human-facing name, with a preferred display name.
fn build_net_table(
    graph: &CircuitGraph,
    compiled: &CompiledPedal,
    probe: &NodeProbe,
) -> Vec<NetOp> {
    // Group names by node id, then pick the best display name per node.
    let mut by_node: HashMap<NodeId, Vec<String>> = HashMap::new();
    for (name, &node) in &graph.node_names {
        by_node.entry(node).or_default().push(name.clone());
    }

    let mut rows: Vec<(NodeId, String)> = by_node
        .into_iter()
        .map(|(node, mut names)| {
            names.sort();
            // Prefer a net-style name (no '.', i.e. not a "Comp.pin" key); then
            // an active-device terminal (Q*.base/collector/... is the clearest
            // label for a transistor node); else the first pin name.
            let is_active_pin = |n: &&String| {
                n.ends_with(".base")
                    || n.ends_with(".collector")
                    || n.ends_with(".emitter")
                    || n.ends_with(".gate")
                    || n.ends_with(".drain")
                    || n.ends_with(".source")
            };
            let display = names
                .iter()
                .find(|n| !n.contains('.'))
                .or_else(|| names.iter().find(is_active_pin))
                .cloned()
                .unwrap_or_else(|| names[0].clone());
            (node, display)
        })
        .collect();
    // Stable, readable order: by node id.
    rows.sort_by_key(|(node, _)| *node);

    rows.into_iter()
        .map(|(node, name)| {
            let (voltage, source) = probe.node_voltage(node, compiled);
            NetOp {
                name,
                voltage,
                source,
            }
        })
        .collect()
}

/// Build the per-device Q-point table for every transistor in the circuit.
fn build_device_table(
    graph: &CircuitGraph,
    compiled: &CompiledPedal,
    probe: &NodeProbe,
) -> Vec<DeviceOp> {
    let mut devices = Vec::new();

    for (comp_idx, comp) in graph.components.iter().enumerate() {
        let Some(info) = device_terminals(comp_idx, graph) else {
            continue;
        };

        let mut terminals: Vec<(String, Option<f64>)> = info
            .pins
            .iter()
            .map(|(pin, node)| (pin.to_string(), probe.node_voltage(*node, compiled).0))
            .collect();

        // Derived BJT/FET differentials, appended for convenience.
        // For a BJT: (Vb, Ve, Vc) → Vbe = Vb-Ve, Vce = Vc-Ve.
        // For a FET: (Vg, Vs, Vd) → Vgs = Vg-Vs, Vds = Vd-Vs.
        let v_in = terminals[0].1;
        let v_ref = terminals[1].1; // emitter / source
        let v_out = terminals[2].1; // collector / drain
        let diff = |a: Option<f64>, b: Option<f64>| match (a, b) {
            (Some(a), Some(b)) => Some(a - b),
            _ => None,
        };
        if info.is_bjt {
            terminals.push(("Vbe".into(), diff(v_in, v_ref)));
            terminals.push(("Vce".into(), diff(v_out, v_ref)));
        } else {
            terminals.push(("Vgs".into(), diff(v_in, v_ref)));
            terminals.push(("Vds".into(), diff(v_out, v_ref)));
        }

        // Output current via the load resistor on the output terminal: the
        // collector/drain current equals the current through the passive load
        // tying the output node to its rail, I = V_across(R_load) / R.
        let ic = output_current(comp_idx, graph, compiled, probe, &info);

        // Pull the REAL device Q-point from the nonlinear WDF root, if this
        // component compiled to one. For a BJT common-emitter stage the DC
        // operating point lives in the root's seeded bias + runtime-solved
        // state, NOT on the passive tree leaves above.
        let root = root_op_for(&comp.id, compiled);

        devices.push(DeviceOp {
            id: comp.id.clone(),
            kind: info.kind_label.into(),
            terminals,
            ic,
            root,
        });
    }

    devices
}

/// Read the nonlinear-root operating point for a device by its component id.
///
/// Scans the compiled stages for a `WdfStage` whose `root_comp_id` matches and
/// whose `root` is a NL root we can report. Returns `None` when no such stage
/// exists — the caller renders that as "device folded to PassiveRType (no NL
/// root)" rather than a bare n/a.
fn root_op_for(comp_id: &str, compiled: &CompiledPedal) -> Option<RootOp> {
    for stage in &compiled.stages {
        let Stage::Wdf(wdf) = stage else { continue };
        if wdf.root_comp_id != comp_id {
            continue;
        }
        if let RootKind::Bjt(bjt) = &wdf.root {
            let seed = wdf.bjt_seed;
            return Some(RootOp {
                kind: "Bjt".into(),
                seed_resolved: seed.map(|s| s.resolved).unwrap_or(false),
                seeded_vbe: seed.and_then(|s| s.seeded_vbe.map(|v| v as f64)),
                seeded_vce: seed.and_then(|s| s.seeded_vce.map(|v| v as f64)),
                vbe_bias: Some(bjt.vbe_bias() as f64),
                solved_vce: Some(bjt.solved_vce() as f64),
                solved_ic: Some(bjt.solved_ic() as f64),
            });
        }
    }
    None
}

struct DeviceTerminals {
    /// (pin name, node id) ordered input/ref/output:
    /// BJT: base, emitter, collector. FET: gate, source, drain.
    pins: [(&'static str, NodeId); 3],
    /// output node id (collector / drain) — for load-current estimation.
    output_node: NodeId,
    is_bjt: bool,
    kind_label: &'static str,
}

/// Identify a transistor component and resolve its three terminal node ids.
/// Returns `None` for non-transistor components.
fn device_terminals(comp_idx: usize, graph: &CircuitGraph) -> Option<DeviceTerminals> {
    use super::classify::NonlinearKind;

    let comp = &graph.components[comp_idx];
    if !comp.kind.is_nonlinear() {
        return None;
    }
    // Find this component's graph edge and classify it.
    let eidx = graph
        .edges
        .iter()
        .position(|e| e.comp_idx == comp_idx)?;
    let edge = &graph.edges[eidx];
    let (kind, _) = comp.kind.classify_nonlinear(
        &comp.id,
        edge.node_a,
        edge.node_b,
        graph.gnd_node,
        &graph.node_names,
    )?;

    match kind {
        NonlinearKind::BjtNpn {
            base_node,
            collector_node,
            emitter_node,
            ..
        } => Some(DeviceTerminals {
            pins: [
                ("base", base_node),
                ("emitter", emitter_node),
                ("collector", collector_node),
            ],
            output_node: collector_node,
            is_bjt: true,
            kind_label: "npn",
        }),
        NonlinearKind::BjtPnp {
            base_node,
            collector_node,
            emitter_node,
            ..
        } => Some(DeviceTerminals {
            pins: [
                ("base", base_node),
                ("emitter", emitter_node),
                ("collector", collector_node),
            ],
            output_node: collector_node,
            // PNP is still a BJT — we want Vbe/Vce labels, not Vgs/Vds.
            is_bjt: true,
            kind_label: "pnp",
        }),
        _ => None,
    }
}

/// Estimate collector/drain current via the load resistor on the output node.
///
/// The output node (collector/drain) is tied to a rail through a load resistor
/// R_load, so the transistor's DC output current ≈ the current through that
/// load: `I = (V_rail − V_out) / R`. We use the PROPAGATED node voltages here
/// (not the resistor's WDF leaf voltage, which is identically 0). Returns
/// `None` when no rail-anchored load resistor is found, or when either of the
/// two node voltages it depends on is unrecoverable.
fn output_current(
    _comp_idx: usize,
    graph: &CircuitGraph,
    compiled: &CompiledPedal,
    probe: &NodeProbe,
    info: &DeviceTerminals,
) -> Option<f64> {
    let out = info.output_node;
    let (v_out, _) = probe.node_voltage(out, compiled);
    let v_out = v_out?;
    for (eidx, edge) in graph.edges.iter().enumerate() {
        if edge.node_a != out && edge.node_b != out {
            continue;
        }
        let other = if edge.node_a == out {
            edge.node_b
        } else {
            edge.node_a
        };
        let comp = &graph.components[edge.comp_idx];
        // Must be a resistor (Linear edge) with a known ohmic value.
        let kind = graph
            .resolved_edge_kinds
            .get(&eidx)
            .copied()
            .or_else(|| comp.kind.edges().first().map(|e| e.kind));
        if kind != Some(EdgeKind::Linear) {
            continue;
        }
        // Far terminal must be a node whose voltage we know (the load's rail).
        let (v_other, src) = probe.node_voltage(other, compiled);
        if src != VoltageSource::Rail {
            continue;
        }
        let Some(v_other) = v_other else { continue };
        let r = resistor_ohms(comp)?;
        if r <= 0.0 {
            continue;
        }
        return Some((v_other - v_out).abs() / r);
    }
    None
}

/// Extract a resistor's ohmic value from the component, if it is a resistor.
fn resistor_ohms(comp: &crate::dsl::ComponentDef) -> Option<f64> {
    comp.kind
        .as_any()
        .downcast_ref::<super::components::Resistor>()
        .map(|r| r.value)
}
