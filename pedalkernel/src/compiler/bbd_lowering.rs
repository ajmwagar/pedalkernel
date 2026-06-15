//! BBD lowering: turn `bbd()` components into runtime `BbdDelayLine`s plus
//! their control and LFO bindings (bug-hunt census #1).
//!
//! The `Bbd` component is `GraphRole::Virtual` with a `Behavioral` edge — it
//! contributes no electrical edges, so the circuit graph is galvanically
//! split at `BBD.in`/`BBD.out`. The SPQR pipeline previously compiled the
//! surrounding passives into a single stage spanning that gap (output probe
//! reads exact 0) and left `CompiledPedal.bbds` empty, so every BBD pedal
//! was silent. This module:
//!
//! 1. builds one `BbdDelayLine` per `bbd()` component (`bbds[idx]` order =
//!    component declaration order — the index contract used by
//!    `ControlTarget::BbdClockRate/BbdFeedback` and
//!    `ModulationTarget::BbdClock`),
//! 2. splits flow groups into galvanically-connected clusters so each side
//!    of the behavioral gap becomes its own serial stage (the runtime then
//!    applies the BBD wet/dry mix after the stage chain),
//! 3. binds pots wired to `BBD.clock` / `BBD.out` to clock-rate / feedback
//!    control targets, detects wet/dry mix pots, and
//! 4. binds LFOs that drive `BBD.clock` (directly or through a resistive
//!    divider) as `ModulationTarget::BbdClock` modulators, including their
//!    Rate/Depth pots.
//!
//! Placement note: control/LFO resolution like (3)/(4) has a natural home in
//! `bind.rs` (which still carries equivalent-but-dead legacy helpers from the
//! removed 6-pass pipeline), but that module is owned by parallel work, so
//! the SPQR-specific BBD lowering lives here as its own pass.

use hashbrown::HashMap;
use std::collections::HashSet;

use crate::dsl::{BbdType, LfoWaveformDsl, PedalDef, Pin};

use super::compiled::{CompiledPedal, ControlBinding, ControlTarget, LfoBinding, ModulationTarget};
use super::components::{Bbd, Lfo};
use super::dsp_block::{BlockIo, BlockPort, DspBlock, PortRole};
use super::graph::CircuitGraph;
use super::signal_flow::FlowGroup;
use pedalkernel_rt::elements::{BbdDelayLine, BbdModel, Modulator};

/// The BBD [`DspBlock`]: lowers `bbd()` components to runtime `BbdDelayLine`s
/// (`compiled.bbds`) plus their control/LFO bindings. The type-specific guts
/// live as `pub(super)` helpers in this module.
pub(super) struct BbdBlock;

impl DspBlock for BbdBlock {
    fn handles(&self, type_tag: &str) -> bool {
        type_tag == "BBD delay"
    }

    fn component_ids(&self, pedal: &PedalDef) -> Vec<String> {
        bbd_component_ids(pedal)
    }

    fn io(&self, pedal: &PedalDef, graph: &CircuitGraph) -> Vec<(String, BlockIo)> {
        bbd_io(pedal, graph)
    }

    fn bind_runtime(
        &self,
        pedal: &PedalDef,
        compiled: &mut CompiledPedal,
        sample_rate: f64,
    ) -> Result<(), String> {
        compiled.bbds = build_bbd_delay_lines(pedal, sample_rate);
        bind_bbd_runtime(pedal, compiled, sample_rate);
        Ok(())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// (1) BbdDelayLine construction
// ═══════════════════════════════════════════════════════════════════════════

/// All `bbd()` component IDs in declaration order. `bbds[i]` corresponds to
/// `bbd_component_ids(pedal)[i]` — the index every BBD control/modulation
/// target refers to.
pub(super) fn bbd_component_ids(pedal: &PedalDef) -> Vec<String> {
    pedal
        .components
        .iter()
        .filter(|c| c.kind.as_any().downcast_ref::<Bbd>().is_some())
        .map(|c| c.id.clone())
        .collect()
}

/// Build one `BbdDelayLine` per `bbd()` component (declaration order).
pub(super) fn build_bbd_delay_lines(pedal: &PedalDef, sample_rate: f64) -> Vec<BbdDelayLine> {
    pedal
        .components
        .iter()
        .filter_map(|c| {
            let bbd = c.kind.as_any().downcast_ref::<Bbd>()?;
            let model = match bbd.bbd_type {
                BbdType::Mn3207 => BbdModel::mn3207(),
                BbdType::Mn3007 => BbdModel::mn3007(),
                BbdType::Mn3005 => BbdModel::mn3005(),
            };
            Some(BbdDelayLine::new(model, sample_rate))
        })
        .collect()
}

/// Typed port signature (spec §2) of each `bbd()`: 1 Audio input (the BBD `in`
/// node), 1 Audio output (the BBD `out` node), in component declaration order.
///
/// These nodes are the behavioral stage boundaries: the netlist is galvanically
/// cut there, so each one must be treated like a global terminal when computing
/// group terminals — otherwise the dangling side of a passive group has no port
/// and compiles into a stage whose probe reads 0. `all_boundary_nodes` flattens
/// inputs-then-outputs, recovering exactly the `[in_node, out_node]` set the
/// previous flat `bbd_boundary_nodes` produced (`in`/`input` and `out`/`output`
/// are pin aliases that union to one node each).
pub(super) fn bbd_io(pedal: &PedalDef, graph: &CircuitGraph) -> Vec<(String, BlockIo)> {
    let resolve = |id: &str, pins: &[&'static str]| -> Option<BlockPort> {
        for &pin in pins {
            if let Some(&node) = graph.node_names.get(&format!("{id}.{pin}")) {
                return Some(BlockPort {
                    node,
                    pin,
                    role: PortRole::Audio,
                });
            }
        }
        None
    };
    bbd_component_ids(pedal)
        .into_iter()
        .map(|id| {
            let inputs = resolve(&id, &["in", "input"]).into_iter().collect();
            let outputs = resolve(&id, &["out", "output"]).into_iter().collect();
            (id, BlockIo { inputs, outputs })
        })
        .collect()
}

// ═══════════════════════════════════════════════════════════════════════════
// (2) Group splitting at behavioral gaps
// ═══════════════════════════════════════════════════════════════════════════

/// Split flow groups into galvanically-connected clusters.
///
/// `find_flow_groups` can merge edges on both sides of a BBD's behavioral gap
/// into one group (they remain connected through ground). A WDF/MNA stage
/// built across the gap has no transfer path from its source to its probe and
/// outputs exact 0. Splitting yields one serial stage per cluster; flow
/// distances computed afterwards order the input-side cluster before the
/// output-side cluster, and the runtime BBD wet path bridges them.
///
/// Connectivity deliberately ignores rails (gnd/vcc/supplies/AC grounds) —
/// two networks that only share ground are not a signal path.
pub(super) fn split_groups_at_behavioral_gaps(groups: &mut Vec<FlowGroup>, graph: &CircuitGraph) {
    let rails = super::signal_flow::rail_nodes(graph);

    let mut result: Vec<FlowGroup> = Vec::with_capacity(groups.len());
    for group in groups.iter() {
        let edges = group.all_edges();
        if edges.len() <= 1 {
            result.push(group.clone());
            continue;
        }

        // Union-find over the group's edges: edges sharing a non-rail node
        // are in the same galvanic cluster.
        let mut parent: Vec<usize> = (0..edges.len()).collect();
        fn find(parent: &mut Vec<usize>, mut i: usize) -> usize {
            while parent[i] != i {
                parent[i] = parent[parent[i]];
                i = parent[i];
            }
            i
        }
        let mut node_owner: HashMap<usize, usize> = HashMap::new();
        for (i, &eidx) in edges.iter().enumerate() {
            let e = &graph.edges[eidx];
            for node in [e.node_a, e.node_b] {
                if rails.contains(&node) {
                    continue;
                }
                if let Some(&owner) = node_owner.get(&node) {
                    let (ra, rb) = (find(&mut parent, owner), find(&mut parent, i));
                    parent[ra] = rb;
                } else {
                    node_owner.insert(node, i);
                }
            }
        }

        // Bucket edge positions by cluster root, preserving order.
        let mut clusters: Vec<(usize, Vec<usize>)> = Vec::new();
        for i in 0..edges.len() {
            let root = find(&mut parent, i);
            match clusters.iter_mut().find(|(r, _)| *r == root) {
                Some((_, v)) => v.push(i),
                None => clusters.push((root, vec![i])),
            }
        }

        if clusters.len() <= 1 {
            result.push(group.clone());
            continue;
        }

        // Map cluster membership back through the group's edge categories.
        for (_, members) in &clusters {
            let member_edges: HashSet<usize> = members.iter().map(|&i| edges[i]).collect();
            let filter = |v: &Vec<usize>| -> Vec<usize> {
                v.iter()
                    .copied()
                    .filter(|e| member_edges.contains(e))
                    .collect()
            };
            result.push(FlowGroup {
                active_edges: filter(&group.active_edges),
                feedback_edges: filter(&group.feedback_edges),
                pendant_edges: filter(&group.pendant_edges),
                ground_shunt_edges: filter(&group.ground_shunt_edges),
            });
        }
    }

    *groups = result;
}

// ═══════════════════════════════════════════════════════════════════════════
// (3)+(4) Control and LFO binding
// ═══════════════════════════════════════════════════════════════════════════

/// Bind BBD controls and clock LFOs onto a compiled pedal.
///
/// Must run after `spqr_control::bind_controls` (it only adds/retargets
/// bindings, never reorders existing ones — pot smoother indices stay valid).
pub(super) fn bind_bbd_runtime(pedal: &PedalDef, compiled: &mut CompiledPedal, sample_rate: f64) {
    let bbd_ids = bbd_component_ids(pedal);
    if bbd_ids.is_empty() {
        return;
    }
    let bbd_id_to_idx: HashMap<&str, usize> = bbd_ids
        .iter()
        .enumerate()
        .map(|(i, id)| (id.as_str(), i))
        .collect();

    // ── LFOs driving BBD clocks ─────────────────────────────────────────
    // `LFO.out -> BBD.clock`, possibly through a resistive divider
    // (e.g. CE-2: LFO1.out -> Depth -> R7 -> BBD1.clock).
    let mut lfo_binding_idx: HashMap<String, usize> = HashMap::new(); // lfo comp id -> lfos[] idx
    for comp in &pedal.components {
        let Some(lfo) = comp.kind.as_any().downcast_ref::<Lfo>() else {
            continue;
        };
        for net in &pedal.nets {
            let Pin::ComponentPin { component, pin } = &net.from else {
                continue;
            };
            if component != &comp.id || (pin != "out" && pin != "output") {
                continue;
            }
            for to_pin in &net.to {
                let Pin::ComponentPin {
                    component: tcomp,
                    pin: tpin,
                } = to_pin
                else {
                    continue;
                };
                let mut visited = HashSet::new();
                let Some(bbd_idx) =
                    resolve_bbd_clock(tcomp, tpin, pedal, &bbd_id_to_idx, &mut visited)
                else {
                    continue;
                };
                if lfo_binding_idx.contains_key(&comp.id) {
                    continue; // one clock binding per LFO is enough
                }
                // Bias/range from the BBD's declared modulation sink.
                let (bias, range) = pedal
                    .components
                    .iter()
                    .find(|c| c.id == bbd_ids[bbd_idx])
                    .and_then(|c| c.kind.modulation_sink("clock"))
                    .map(|s| (s.bias, s.range))
                    .unwrap_or((0.15, 0.10));

                let waveform = match lfo.waveform {
                    LfoWaveformDsl::Sine => crate::elements::LfoWaveform::Sine,
                    LfoWaveformDsl::Triangle => crate::elements::LfoWaveform::Triangle,
                    LfoWaveformDsl::Square => crate::elements::LfoWaveform::Square,
                    LfoWaveformDsl::SawUp => crate::elements::LfoWaveform::SawUp,
                    LfoWaveformDsl::SawDown => crate::elements::LfoWaveform::SawDown,
                    LfoWaveformDsl::SampleAndHold => crate::elements::LfoWaveform::SampleAndHold,
                };
                let base_freq = 1.0 / (2.0 * std::f64::consts::PI * lfo.timing_r * lfo.timing_c);
                let mut osc = crate::elements::Lfo::new(waveform, sample_rate);
                osc.set_rate(base_freq);

                lfo_binding_idx.insert(comp.id.clone(), compiled.lfos.len());
                compiled.lfos.push(LfoBinding {
                    lfo: osc,
                    target: ModulationTarget::BbdClock { bbd_idx },
                    bias,
                    range,
                    base_freq,
                    lfo_id: comp.id.clone(),
                });
            }
        }
    }

    // ── Controls ────────────────────────────────────────────────────────
    for ctrl in &pedal.controls {
        // Direct declarations on the BBD itself (`BBD1.clock -> "Time"`).
        if let Some(&idx) = bbd_id_to_idx.get(ctrl.component.as_str()) {
            let target = match ctrl.property.as_str() {
                "clock" => Some(ControlTarget::BbdClockRate(idx)),
                "feedback" => Some(ControlTarget::BbdFeedback(idx)),
                _ => None,
            };
            if let Some(target) = target {
                upsert_control(compiled, pedal, ctrl, target);
            }
            continue;
        }

        // Pot-routed controls.
        let Some(pot) = pedal
            .components
            .iter()
            .find(|c| c.id == ctrl.component && c.kind.is_pot())
        else {
            continue;
        };

        if let Some(target) = scan_pot_target(&pot.id, pedal, &bbd_id_to_idx, &lfo_binding_idx) {
            upsert_control(compiled, pedal, ctrl, target);
            continue;
        }

        // Wet/dry mix pot: reaches a BBD output through passives. The pot
        // keeps its stage binding (it is electrically in the mix network);
        // the runtime additionally mirrors its position into bbd_wet_mix.
        if compiled.bbd_mix_pot_id.is_none()
            && pot_reaches_bbd_output(&pot.id, pedal, &bbd_id_to_idx)
        {
            compiled.bbd_mix_pot_id = Some(pot.id.clone());
            compiled.bbd_wet_mix = ctrl.default.clamp(0.0, 1.0);
        }
    }
}

/// Retarget an existing binding for (label, component) or append a new one.
///
/// Retargeting in place keeps `controls` indices stable so pot smoothers
/// created by `bind_controls` stay valid.
fn upsert_control(
    compiled: &mut CompiledPedal,
    pedal: &PedalDef,
    ctrl: &crate::dsl::ControlDef,
    target: ControlTarget,
) {
    if let Some(existing) = compiled
        .controls
        .iter_mut()
        .find(|b| b.label == ctrl.label && b.component_id == ctrl.component)
    {
        existing.target = target;
        // Drop any stale coordinated pot/divider targets — this control is
        // now a BBD modulation target, not a multi-stage pot.
        existing.targets.clear();
    } else {
        let comp = pedal.components.iter().find(|c| c.id == ctrl.component);
        let max_r = comp.and_then(|c| c.kind.resistance()).unwrap_or(100_000.0);
        let taper = comp
            .and_then(|c| c.kind.pot_taper())
            .unwrap_or(crate::dsl::PotTaper::B);
        compiled.controls.push(ControlBinding {
            label: ctrl.label.clone(),
            target,
            targets: Vec::new(),
            component_id: ctrl.component.clone(),
            max_resistance: max_r,
            taper,
            range: ctrl.range,
        });
    }
    // (Re-)apply the declared default so the new target receives it —
    // bind_controls applied defaults before this pass retargeted/added the
    // binding. Non-pot targets have no smoother; set_control is direct.
    compiled.set_control(&ctrl.label, ctrl.default);
}

/// Resolve a pot to a BBD/LFO control target by scanning nets.
///
/// Forward: `Pot.pin -> BBD.clock` ⇒ clock rate; `Pot.pin -> LFO.rate` ⇒
/// LFO rate. Reverse: `BBD.out -> Pot.pin` ⇒ feedback; `LFO.out -> Pot.pin`
/// ⇒ LFO depth. Mirrors the legacy `scan_pot_nets` priority (net-declared
/// modulation routing wins over physical stage placement).
fn scan_pot_target(
    pot_id: &str,
    pedal: &PedalDef,
    bbd_id_to_idx: &HashMap<&str, usize>,
    lfo_binding_idx: &HashMap<String, usize>,
) -> Option<ControlTarget> {
    let pot_pins = ["wiper", "w", "a", "b"];
    for net in &pedal.nets {
        // Forward: Pot.pin -> Target.modpin
        if let Pin::ComponentPin { component, pin } = &net.from {
            if component == pot_id && pot_pins.contains(&pin.as_str()) {
                for to_pin in &net.to {
                    let Pin::ComponentPin {
                        component: tcomp,
                        pin: tpin,
                    } = to_pin
                    else {
                        continue;
                    };
                    if tpin == "clock" {
                        if let Some(&idx) = bbd_id_to_idx.get(tcomp.as_str()) {
                            return Some(ControlTarget::BbdClockRate(idx));
                        }
                    }
                    if tpin == "rate" {
                        if let Some(&idx) = lfo_binding_idx.get(tcomp.as_str()) {
                            return Some(ControlTarget::LfoRate(idx));
                        }
                    }
                }
            }
        }
        // Reverse: Source.out -> Pot.pin
        for to_pin in &net.to {
            let Pin::ComponentPin { component, pin } = to_pin else {
                continue;
            };
            if component != pot_id || !pot_pins.contains(&pin.as_str()) {
                continue;
            }
            if let Pin::ComponentPin {
                component: scomp,
                pin: spin,
            } = &net.from
            {
                if spin == "out" || spin == "output" {
                    if let Some(&idx) = bbd_id_to_idx.get(scomp.as_str()) {
                        return Some(ControlTarget::BbdFeedback(idx));
                    }
                    if let Some(&idx) = lfo_binding_idx.get(scomp.as_str()) {
                        return Some(ControlTarget::LfoDepth(idx));
                    }
                }
            }
        }
    }
    None
}

/// Follow a net endpoint through resistive elements (resistors/pots) until a
/// BBD `clock` pin is reached. Returns the BBD index.
fn resolve_bbd_clock(
    comp_id: &str,
    pin: &str,
    pedal: &PedalDef,
    bbd_id_to_idx: &HashMap<&str, usize>,
    visited: &mut HashSet<String>,
) -> Option<usize> {
    if !visited.insert(format!("{comp_id}:{pin}")) {
        return None;
    }
    if pin == "clock" {
        if let Some(&idx) = bbd_id_to_idx.get(comp_id) {
            return Some(idx);
        }
    }

    let comp = pedal.components.iter().find(|c| c.id == comp_id)?;
    let other_pins: Vec<&str> = if comp.kind.is_pot() {
        match pin {
            "a" => vec!["b", "wiper", "w"],
            "b" | "wiper" | "w" => vec!["a"],
            _ => return None,
        }
    } else if comp.kind.type_tag() == "resistor" {
        match pin {
            "a" => vec!["b"],
            "b" => vec!["a"],
            _ => return None,
        }
    } else {
        return None;
    };

    for other in other_pins {
        for net in &pedal.nets {
            let all: Vec<&Pin> = std::iter::once(&net.from).chain(net.to.iter()).collect();
            let touches = all.iter().any(|p| {
                matches!(p, Pin::ComponentPin { component, pin: pp }
                    if component == comp_id && pp == other)
            });
            if !touches {
                continue;
            }
            for p in &all {
                if let Pin::ComponentPin {
                    component: next,
                    pin: next_pin,
                } = p
                {
                    if next != comp_id {
                        if let Some(idx) =
                            resolve_bbd_clock(next, next_pin, pedal, bbd_id_to_idx, visited)
                        {
                            return Some(idx);
                        }
                    }
                }
            }
        }
    }
    None
}

/// BFS through passive 2-terminal elements (R/C) from a pot's terminals to a
/// BBD `out` pin — detects wet/dry mix pots (mirrors the legacy
/// `pot_reaches_bbd_output`).
fn pot_reaches_bbd_output(
    pot_id: &str,
    pedal: &PedalDef,
    bbd_id_to_idx: &HashMap<&str, usize>,
) -> bool {
    let pot_pins = ["a", "b", "wiper", "w"];
    let mut visited: HashSet<String> = pot_pins.iter().map(|pp| format!("{pot_id}:{pp}")).collect();

    let mut queue: Vec<(String, String)> = Vec::new();
    for net in &pedal.nets {
        let all: Vec<&Pin> = std::iter::once(&net.from).chain(net.to.iter()).collect();
        let pot_in_net = all.iter().any(|p| {
            matches!(p, Pin::ComponentPin { component, pin }
                if component == pot_id && pot_pins.contains(&pin.as_str()))
        });
        if !pot_in_net {
            continue;
        }
        for p in &all {
            if let Pin::ComponentPin { component, pin } = p {
                if component.as_str() != pot_id && visited.insert(format!("{component}:{pin}")) {
                    queue.push((component.clone(), pin.clone()));
                }
            }
        }
    }

    while let Some((comp_id, pin)) = queue.pop() {
        if (pin == "out" || pin == "output") && bbd_id_to_idx.contains_key(comp_id.as_str()) {
            return true;
        }
        let comp = pedal.components.iter().find(|c| c.id == comp_id);
        let other_pin = match comp {
            Some(c) if c.kind.type_tag() == "resistor" || c.kind.type_tag() == "capacitor" => {
                match pin.as_str() {
                    "a" => "b",
                    "b" => "a",
                    _ => continue,
                }
            }
            _ => continue,
        };
        if !visited.insert(format!("{comp_id}:{other_pin}")) {
            continue;
        }
        for net in &pedal.nets {
            let all: Vec<&Pin> = std::iter::once(&net.from).chain(net.to.iter()).collect();
            let touches = all.iter().any(|p| {
                matches!(p, Pin::ComponentPin { component, pin: pp }
                    if component == &comp_id && pp == other_pin)
            });
            if !touches {
                continue;
            }
            for p in &all {
                if let Pin::ComponentPin {
                    component: next,
                    pin: next_pin,
                } = p
                {
                    if next.as_str() != comp_id.as_str()
                        && visited.insert(format!("{next}:{next_pin}"))
                    {
                        queue.push((next.clone(), next_pin.clone()));
                    }
                }
            }
        }
    }
    false
}
