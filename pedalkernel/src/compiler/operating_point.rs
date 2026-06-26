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

/// Amplitude of the diagnostic settle stimulus (volts of input). Tiny enough
/// to stay deep in the small-signal regime around the Q-point, large enough to
/// keep the NL solve OFF its clamps (see below).
const SETTLE_STIM_AMPLITUDE: f64 = 1e-3;
/// Frequency of the diagnostic settle stimulus (Hz). Low so a final cycle is
/// many samples wide (≈ a clean Q-point average) yet completes within settle.
const SETTLE_STIM_FREQ_HZ: f64 = 100.0;

/// Settle the processor and compute its operating-point report.
///
/// ## Why a tiny sine, not pure DC
///
/// The original harness settled with pure-`0.0` input. That is faithful for the
/// passive net table, but it MISREADS the BJT NL root's Q-point: the root solves
/// incrementally around its seeded bias, and at EXACTLY DC=0 the Newton
/// warm-start (`v0 = prev_v`) keeps the AC port voltage pinned at the negative
/// clamp it hits during the initial cap-charge transient. `solved_vce()` =
/// `vce_bias + prev_v` then reads the rail (e.g. −9 V) instead of the true
/// Q-point (≈ +2.92 V), even though the audio path — which always sees a real,
/// nonzero signal — tracks correctly.
///
/// So we settle the diagnostic the way the working audio path runs it: with a
/// very small low-frequency sine (`SETTLE_STIM_AMPLITUDE` @ `SETTLE_STIM_FREQ_HZ`).
/// The root then tracks off its clamp, and we read the Q-point as the AVERAGE of
/// the root's solved Vce/Ic over the FINAL stimulus cycle (the sine is symmetric
/// about its zero, so the cycle mean ≈ the DC operating point). The passive net
/// table is still trustworthy: at this amplitude the cap voltages differ from
/// their DC values by ~1e-3 V, negligible for the bias report.
///
/// This is a DIAGNOSTIC-ONLY change. After settling we `reset()` the processor
/// so it returns to its audio-ready quiescent state (the caller hands the same
/// processor on for audio use).
pub(super) fn compute_operating_point(
    compiled: &mut CompiledPedal,
    graph: &CircuitGraph,
    sample_rate: f64,
    settle_seconds: f64,
) {
    let settle_samples = (settle_seconds * sample_rate).round() as usize;
    // Samples in one stimulus cycle (clamped to at least 1 and to the whole
    // settle window, so very short settles or odd sample rates stay sane).
    let cycle_samples = ((sample_rate / SETTLE_STIM_FREQ_HZ).round() as usize)
        .max(1)
        .min(settle_samples.max(1));
    // The final cycle over which we average the root Q-point.
    let avg_start = settle_samples.saturating_sub(cycle_samples);

    // Per-root-component accumulated (sum_vce, sum_ic, count) over the final
    // cycle, so the reported Q-point is the cycle MEAN (≈ DC operating point).
    let mut root_avg: HashMap<String, (f64, f64, u64)> = HashMap::new();

    // Start the settle from the clean, seeded quiescent state. Without this the
    // NL root's NR warm-start (`prev_v`) can be left pinned at an alternate
    // fixed point from a prior run/transient, which a tiny stimulus cannot
    // escape — `reset()` restores each BJT root to `prev_v = initial_prev_v`
    // (= 0, the seeded Q-point), so the solve tracks from the true operating
    // point outward.
    compiled.reset();

    let two_pi = core::f64::consts::PI * 2.0;
    let mut last_out = 0.0;
    for n in 0..settle_samples {
        let phase = two_pi * (n as f64) * SETTLE_STIM_FREQ_HZ / sample_rate;
        let stim = SETTLE_STIM_AMPLITUDE * phase.sin();
        last_out = compiled.process(stim);

        // Accumulate the root Q-point over the final stimulus cycle only.
        if n >= avg_start {
            accumulate_root_qpoints(compiled, &mut root_avg);
        }
    }

    let probe = NodeProbe::new(graph, compiled, last_out);
    let nets = build_net_table(graph, compiled, &probe);
    let devices = build_device_table(graph, compiled, &probe, &root_avg);

    compiled.operating_point = Some(OperatingPoint {
        nets,
        devices,
        settle_samples,
    });

    // Restore the processor to its quiescent, audio-ready state — the diagnostic
    // settle must not leave the handed-on processor mid-transient.
    compiled.reset();
}

/// Scan the compiled stages for BJT NL roots and add this sample's solved
/// Vce/Ic to the running per-component average accumulator.
fn accumulate_root_qpoints(
    compiled: &CompiledPedal,
    root_avg: &mut HashMap<String, (f64, f64, u64)>,
) {
    for stage in &compiled.stages {
        match stage {
            Stage::Wdf(wdf) => {
                if let RootKind::Bjt(bjt) = &wdf.root {
                    let entry = root_avg
                        .entry(wdf.root_comp_id.clone())
                        .or_insert((0.0, 0.0, 0));
                    entry.0 += bjt.solved_vce() as f64;
                    entry.1 += bjt.solved_ic() as f64;
                    entry.2 += 1;
                }
            }
            // BJTs co-solved inside a MultiNl stage (feedback/self-bias cores).
            Stage::MultiNl(multi) => {
                for (id, _vbe, vce, ic) in multi.bjt_two_port_qpoints() {
                    if id.is_empty() {
                        continue;
                    }
                    let entry = root_avg.entry(id).or_insert((0.0, 0.0, 0));
                    entry.0 += vce;
                    entry.1 += ic;
                    entry.2 += 1;
                }
            }
            _ => {}
        }
    }
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
    root_avg: &HashMap<String, (f64, f64, u64)>,
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
        let root = root_op_for(&comp.id, compiled, root_avg);

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
fn root_op_for(
    comp_id: &str,
    compiled: &CompiledPedal,
    root_avg: &HashMap<String, (f64, f64, u64)>,
) -> Option<RootOp> {
    for stage in &compiled.stages {
        let Stage::Wdf(wdf) = stage else { continue };
        if wdf.root_comp_id != comp_id {
            continue;
        }
        if let RootKind::Bjt(bjt) = &wdf.root {
            let seed = wdf.bjt_seed;
            // Prefer the final-cycle AVERAGE of the solved Q-point (faithful to
            // the audio path); fall back to the instantaneous read only if the
            // settle window collected no samples for this root.
            let (solved_vce, solved_ic) = match root_avg.get(comp_id) {
                Some(&(sum_vce, sum_ic, count)) if count > 0 => {
                    (sum_vce / count as f64, sum_ic / count as f64)
                }
                _ => (bjt.solved_vce() as f64, bjt.solved_ic() as f64),
            };
            return Some(RootOp {
                kind: "Bjt".into(),
                seed_resolved: seed.map(|s| s.resolved).unwrap_or(false),
                seeded_vbe: seed.and_then(|s| s.seeded_vbe.map(|v| v as f64)),
                seeded_vce: seed.and_then(|s| s.seeded_vce.map(|v| v as f64)),
                vbe_bias: Some(bjt.vbe_bias() as f64),
                solved_vce: Some(solved_vce),
                solved_ic: Some(solved_ic),
            });
        }
    }

    // A BJT in a feedback/self-bias circuit does NOT compile to a WDF BjtRoot —
    // it is co-solved inside a MultiNl stage as a `BjtTwoPort`. Read the real
    // co-solved Q-point (Vbe/Vce/Ic) from the settled stage. Prefer the
    // final-cycle AVERAGE collected during settle (mirrors the WDF-root path);
    // fall back to the instantaneous read if no samples were averaged.
    for stage in &compiled.stages {
        let Stage::MultiNl(multi) = stage else {
            continue;
        };
        for (id, vbe, vce, ic) in multi.bjt_two_port_qpoints() {
            if id != comp_id {
                continue;
            }
            let (solved_vce, solved_ic) = match root_avg.get(comp_id) {
                Some(&(sum_vce, sum_ic, count)) if count > 0 => {
                    (sum_vce / count as f64, sum_ic / count as f64)
                }
                _ => (vce, ic),
            };
            return Some(RootOp {
                kind: "Bjt 2-port (MultiNl co-solve)".into(),
                // The co-solve carries no separate compile-time seed record;
                // the operating point IS the runtime-solved state.
                seed_resolved: true,
                seeded_vbe: None,
                seeded_vce: None,
                vbe_bias: Some(vbe),
                solved_vce: Some(solved_vce),
                solved_ic: Some(solved_ic),
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
