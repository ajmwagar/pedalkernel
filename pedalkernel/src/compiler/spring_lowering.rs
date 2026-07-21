//! Spring-reverb lowering: turn `spring()` components into runtime
//! `SpringBinding`s (a dispersive [`SpringTank`]) plus their dwell/decay/
//! damping/mix controls — the fourth [`DspBlock`]
//! (reports/spring-reverb-design-2026-06-13.md).
//!
//! `spring()` is `GraphRole::Virtual` with a single `Behavioral` in→out edge:
//! it contributes no electrical edges, so the circuit graph is galvanically
//! cut at `RV.in`/`RV.out`, exactly like `bbd()`/`delay_line()` (1-in/1-out
//! inserts — the precedent this module mirrors). The driver stage and the
//! recovery stage compile into separate serial WDF stages; the runtime spring
//! tank bridges the gap in the processor tick loop.
//!
//! Per the design defaults:
//!   * **Per-instance** state and mix (spec §6, OQ5) — no global mix field;
//!     each `SpringBinding` owns its `wet_mix` (learning from the BBD's shared
//!     `bbd_wet_mix` being the wrong pattern).
//!   * dwell → drive trim / decay → RT60 authority / damping → loop LPF / mix →
//!     wet-dry, all bindable by pot or CV via the generalized control path.
//!   * a `spring()` whose audio pins cannot bridge the gap is a **compile
//!     error** naming the component (architecture debt §4 — never ship a
//!     silently-bypassed circuit). The mandatory-lowering gate handles an
//!     unwired `spring()` like an unwired `vco()`.
//!
//! As with the other blocks, `springs[i]` corresponds to
//! `spring_component_ids(pedal)[i]` (declaration order = the index contract the
//! `SpringDwell`/`SpringDecay`/`SpringDamping`/`SpringMix` targets refer to).

use hashbrown::HashMap;

use crate::dsl::{PedalDef, Pin, SpringTankType};

use super::compiled::{CompiledPedal, ControlBinding, ControlTarget};
use super::components::Spring;
use super::dsp_block::{behavioral_island_error, BlockIo, BlockPort, DspBlock, PortRole};
use super::graph::CircuitGraph;
use pedalkernel_rt::elements::{SpringModel, SpringTank};
use pedalkernel_rt::processor::SpringBinding;

/// Pin names a `spring()` accepts on its audio input / output side.
const SPRING_IN_PINS: &[&str] = &["in", "input"];
const SPRING_OUT_PINS: &[&str] = &["out", "output"];
/// Pot terminals a routed control may connect through.
const POT_PINS: &[&str] = &["wiper", "w", "a", "b"];

/// The spring-reverb [`DspBlock`]: lowers `spring()` components to runtime
/// `SpringBinding`s (`compiled.springs`) plus their control bindings.
pub(super) struct SpringBlock;

impl DspBlock for SpringBlock {
    fn handles(&self, type_tag: &str) -> bool {
        type_tag == "spring reverb"
    }

    fn component_ids(&self, pedal: &PedalDef) -> Vec<String> {
        spring_component_ids(pedal)
    }

    fn io(&self, pedal: &PedalDef, graph: &CircuitGraph) -> Vec<(String, BlockIo)> {
        spring_io(pedal, graph)
    }

    fn bind_runtime(
        &self,
        pedal: &PedalDef,
        compiled: &mut CompiledPedal,
        sample_rate: f64,
    ) -> Result<(), String> {
        compiled.springs = build_spring_tanks(pedal, sample_rate)?;
        bind_spring_runtime(pedal, compiled);
        Ok(())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// (1) Component enumeration + runtime SpringBinding construction
// ═══════════════════════════════════════════════════════════════════════════

/// All `spring()` component IDs in declaration order. `springs[i]` corresponds
/// to `spring_component_ids(pedal)[i]`.
pub(super) fn spring_component_ids(pedal: &PedalDef) -> Vec<String> {
    pedal
        .components
        .iter()
        .filter(|c| c.kind.as_any().downcast_ref::<Spring>().is_some())
        .map(|c| c.id.clone())
        .collect()
}

/// Map a DSL tank type to the runtime model.
fn rt_model(t: SpringTankType) -> SpringModel {
    match t {
        SpringTankType::Type4 => SpringModel::type4(),
        SpringTankType::Type8 => SpringModel::type8(),
        SpringTankType::Type9 => SpringModel::type9(),
        SpringTankType::Re201Tank => SpringModel::re201_tank(),
    }
}

/// True if any net references `comp_id.<pin>` for one of `pins`.
fn pin_wired(pedal: &PedalDef, comp_id: &str, pins: &[&str]) -> bool {
    pedal.nets.iter().any(|net| {
        std::iter::once(&net.from).chain(net.to.iter()).any(|p| {
            matches!(p, Pin::ComponentPin { component, pin }
                if component == comp_id && pins.contains(&pin.as_str()))
        })
    })
}

/// Build one `SpringBinding` per `spring()` component (declaration order).
///
/// Errors (never ship silent — architecture debt §4): a `spring()` whose audio
/// `in`/`out` pins are not both wired into the netlist cannot bridge the
/// galvanic gap and would silently produce nothing — a compile error naming
/// the component (mirrors `vco_lowering`'s unplaceable-island rejection).
pub(super) fn build_spring_tanks(
    pedal: &PedalDef,
    sample_rate: f64,
) -> Result<Vec<SpringBinding>, String> {
    let mut springs = Vec::new();
    for comp in &pedal.components {
        let Some(spring) = comp.kind.as_any().downcast_ref::<Spring>() else {
            continue;
        };

        let in_wired = pin_wired(pedal, &comp.id, SPRING_IN_PINS);
        let out_wired = pin_wired(pedal, &comp.id, SPRING_OUT_PINS);
        if !in_wired || !out_wired {
            return Err(behavioral_island_error(
                &comp.id,
                "spring()",
                "its audio in/out pins are not both wired into the netlist, so the \
                 tank cannot bridge the galvanic gap and would silently produce nothing",
            ));
        }

        let tank = SpringTank::new(rt_model(spring.tank_type), sample_rate as crate::Wave);
        springs.push(SpringBinding {
            tank,
            // Per-instance default wet/dry (spec §6): 0.33 — a tasteful spring
            // blend when no mix pot/control is wired. Overridden below by a
            // declared `mix` control if present.
            wet_mix: 0.33,
            comp_id: comp.id.clone(),
        });
    }
    Ok(springs)
}

// ═══════════════════════════════════════════════════════════════════════════
// (2) Port signature (spec §2) — 1 Audio in, 1 Audio out
// ═══════════════════════════════════════════════════════════════════════════

/// Resolve `id.pin` (trying each candidate pin name) to a graph node.
fn resolve_port(
    graph: &CircuitGraph,
    id: &str,
    pins: &[&'static str],
    role: PortRole,
) -> Option<BlockPort> {
    for &pin in pins {
        if let Some(&node) = graph.node_names.get(&format!("{id}.{pin}")) {
            return Some(BlockPort { node, pin, role });
        }
    }
    None
}

/// Typed port signature (spec §2) of each `spring()`: 1 Audio input (the `in`
/// node), 1 Audio output (the `out` node), in component declaration order.
///
/// These nodes are the behavioral stage boundaries: the netlist is galvanically
/// cut at the spring, so each must be a SPQR terminal — otherwise the dangling
/// side of a passive group has no port and its stage probes 0.
/// `all_boundary_nodes` flattens inputs-then-outputs to recover the set.
pub(super) fn spring_io(pedal: &PedalDef, graph: &CircuitGraph) -> Vec<(String, BlockIo)> {
    spring_component_ids(pedal)
        .into_iter()
        .map(|id| {
            let inputs = resolve_port(graph, &id, SPRING_IN_PINS, PortRole::Audio)
                .into_iter()
                .collect();
            let outputs = resolve_port(graph, &id, SPRING_OUT_PINS, PortRole::Audio)
                .into_iter()
                .collect();
            (id, BlockIo { inputs, outputs })
        })
        .collect()
}

// ═══════════════════════════════════════════════════════════════════════════
// (3) Control binding (per-instance, no global state)
// ═══════════════════════════════════════════════════════════════════════════

/// Bind spring controls (dwell / decay / damping / mix) onto a compiled pedal.
///
/// Must run after `spqr_control::bind_controls` (it only adds/retargets
/// bindings, never reorders existing ones — pot smoother indices stay valid),
/// mirroring `bbd_lowering::bind_bbd_runtime` / `delay_lowering`.
pub(super) fn bind_spring_runtime(pedal: &PedalDef, compiled: &mut CompiledPedal) {
    let ids = spring_component_ids(pedal);
    if ids.is_empty() {
        return;
    }
    let id_to_idx: HashMap<&str, usize> = ids
        .iter()
        .enumerate()
        .map(|(i, id)| (id.as_str(), i))
        .collect();

    for ctrl in &pedal.controls {
        // Direct declarations on the spring itself (`RV1.decay -> "Decay"`).
        if let Some(&idx) = id_to_idx.get(ctrl.component.as_str()) {
            if let Some(target) = property_target(&ctrl.property, idx) {
                upsert_control(compiled, pedal, ctrl, target);
            }
            continue;
        }

        // Pot-routed controls: a pot whose terminal nets reach a spring control
        // pin (`Pot.wiper -> RV.decay` / `-> RV.dwell` / ...).
        let Some(pot) = pedal
            .components
            .iter()
            .find(|c| c.id == ctrl.component && c.kind.is_pot())
        else {
            continue;
        };
        if let Some(target) = scan_pot_target(&pot.id, pedal, &id_to_idx) {
            upsert_control(compiled, pedal, ctrl, target);
        }
    }
}

/// Map a spring control property name to its target at instance `idx`.
fn property_target(property: &str, idx: usize) -> Option<ControlTarget> {
    match property {
        "dwell" => Some(ControlTarget::SpringDwell(idx)),
        "decay" => Some(ControlTarget::SpringDecay(idx)),
        "damping" => Some(ControlTarget::SpringDamping(idx)),
        "mix" => Some(ControlTarget::SpringMix(idx)),
        _ => None,
    }
}

/// Resolve a pot to a spring control target by scanning nets
/// (`Pot.term -> RV.<prop>`). Mirrors `delay_lowering::scan_pot_target`.
fn scan_pot_target(
    pot_id: &str,
    pedal: &PedalDef,
    id_to_idx: &HashMap<&str, usize>,
) -> Option<ControlTarget> {
    for net in &pedal.nets {
        let Pin::ComponentPin { component, pin } = &net.from else {
            continue;
        };
        if component != pot_id || !POT_PINS.contains(&pin.as_str()) {
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
            if let Some(&idx) = id_to_idx.get(tcomp.as_str()) {
                if let Some(target) = property_target(tpin, idx) {
                    return Some(target);
                }
            }
        }
    }
    None
}

/// Retarget an existing binding for (label, component) or append a new one.
///
/// Retargeting in place keeps `controls` indices stable so pot smoothers
/// created by `bind_controls` stay valid (mirrors the other blocks). The mix
/// control additionally seeds the binding's per-instance `wet_mix` default.
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
        // Drop stale coordinated pot/divider targets — now a spring target.
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
            max_resistance: max_r as crate::Wave,
            taper,
            range: (ctrl.range.0 as crate::Wave, ctrl.range.1 as crate::Wave),
        });
    }
    // (Re-)apply the declared default so the new target receives it.
    compiled.set_control(&ctrl.label, ctrl.default as crate::Wave);
}
