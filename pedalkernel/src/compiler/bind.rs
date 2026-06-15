//! Pass 5: Binding & assembly.
//!
//! Moves LFO binding, envelope follower binding, control target resolution,
//! sidechain construction, and peripheral construction from compile.rs.

use hashbrown::HashMap;
use std::collections::HashSet;

use crate::dsl::*;
use crate::elements::*;

use super::compiled::*;
use super::component::ControlParamKind;
use super::graph::CircuitGraph;
use super::helpers::has_pot;
use super::stage::{MultiNlStage, RootKind, SidechainProcessor, WdfStage};

// ═══════════════════════════════════════════════════════════════════════════
// Control binding
// ═══════════════════════════════════════════════════════════════════════════

/// Build control bindings for all user controls.
///
/// Returns `(controls, bbd_mix_pot_id)` where `bbd_mix_pot_id` is the component
/// ID of a pot controlling BBD wet/dry mix (if detected from topology).
pub(super) fn build_controls(
    pedal: &PedalDef,
    stages: &[WdfStage],
    multi_nl_stages: &[MultiNlStage],
    lfo_ids: &[String],
    delay_id_to_idx: &HashMap<String, usize>,
    _delay_lines_empty: bool,
    sidechain_comp_ids: &HashMap<String, usize>,
    bbd_id_to_idx: &HashMap<String, usize>,
    trigger_id_to_idx: &HashMap<String, usize>,
) -> (Vec<ControlBinding>, Option<String>) {
    let mut controls = Vec::new();
    let mut bbd_mix_pot_id: Option<String> = None;

    for ctrl in &pedal.controls {
        // Look up the component and find the declared control parameter matching
        // the DSL property name (e.g., "position", "rate", "clock").
        let comp = pedal.components.iter().find(|c| c.id == ctrl.component);
        let param_kind = comp.and_then(|c| {
            c.kind
                .controls()
                .into_iter()
                .find(|p| p.name == ctrl.property)
                .map(|p| p.kind)
        });

        let target = match param_kind {
            Some(ControlParamKind::SwitchPosition { num_positions }) => {
                ControlTarget::SwitchPosition {
                    switch_id: ctrl.component.clone(),
                    num_positions,
                }
            }
            Some(ControlParamKind::LfoRate) => {
                let idx = lfo_ids
                    .iter()
                    .position(|id| id == &ctrl.component)
                    .unwrap_or(0);
                ControlTarget::LfoRate(idx)
            }
            Some(ControlParamKind::LfoDepth) => {
                let idx = lfo_ids
                    .iter()
                    .position(|id| id == &ctrl.component)
                    .unwrap_or(0);
                ControlTarget::LfoDepth(idx)
            }
            Some(ControlParamKind::BbdClockRate) => {
                let idx = bbd_id_to_idx.get(&ctrl.component).copied().unwrap_or(0);
                ControlTarget::BbdClockRate(idx)
            }
            Some(ControlParamKind::BbdFeedback) => {
                let idx = bbd_id_to_idx.get(&ctrl.component).copied().unwrap_or(0);
                ControlTarget::BbdFeedback(idx)
            }
            Some(ControlParamKind::DelayTime) => {
                let idx = delay_id_to_idx.get(&ctrl.component).copied().unwrap_or(0);
                ControlTarget::DelayTime(idx)
            }
            Some(ControlParamKind::DelayFeedback) => {
                let idx = delay_id_to_idx.get(&ctrl.component).copied().unwrap_or(0);
                ControlTarget::DelayFeedback(idx)
            }
            Some(ControlParamKind::Trigger) => {
                let idx = trigger_id_to_idx.get(&ctrl.component).copied().unwrap_or(0);
                ControlTarget::Trigger(idx)
            }
            // Spring controls are bound (retargeted to the correct per-instance
            // index) by the spring_lowering DspBlock pass; this legacy resolver
            // maps to instance 0 as a placeholder the later pass overrides.
            Some(ControlParamKind::SpringDwell) => ControlTarget::SpringDwell(0),
            Some(ControlParamKind::SpringDecay) => ControlTarget::SpringDecay(0),
            Some(ControlParamKind::SpringDamping) => ControlTarget::SpringDamping(0),
            Some(ControlParamKind::SpringMix) => ControlTarget::SpringMix(0),
            Some(ControlParamKind::PotPosition) => {
                let (target, is_bbd_mix) = resolve_pot_target(
                    &ctrl.component,
                    pedal,
                    stages,
                    multi_nl_stages,
                    sidechain_comp_ids,
                    lfo_ids,
                    bbd_id_to_idx,
                    delay_id_to_idx,
                );
                if is_bbd_mix {
                    bbd_mix_pot_id = Some(ctrl.component.clone());
                }
                target
            }
            None => {
                eprintln!(
                    "WARNING: no declared control '{}' on component '{}' ({}) — falling back",
                    ctrl.property, ctrl.component, ctrl.label
                );
                ControlTarget::PotInStage(0)
            }
        };

        let max_r = comp.and_then(|c| c.kind.resistance()).unwrap_or(100_000.0);

        let taper = comp.and_then(|c| c.kind.pot_taper()).unwrap_or(PotTaper::B);

        controls.push(ControlBinding {
            label: ctrl.label.clone(),
            target,
            targets: Vec::new(),
            component_id: ctrl.component.clone(),
            max_resistance: max_r,
            taper,
            range: ctrl.range,
        });
    }

    (controls, bbd_mix_pot_id)
}

/// Resolve a pot-position control to its target.
///
/// Returns `(target, is_bbd_mix)` where `is_bbd_mix` is true if this pot
/// controls BBD wet/dry mix.
///
/// Priority:
///  1.  Net scan: pot wired to LFO/BBD/delay (modulation routing wins over physical placement)
///  2.  Pot in WDF stage tree (op-amp feedback pots found here naturally)
///  3.  Pot in multi-NL stage (BJT bias pots found here naturally)
///  4.  Sidechain pot
///  5.  BBD mix detection (BFS through passives)
///  6.  PassiveRType children
///  7.  Fallback: NotInAudioPath warning
fn resolve_pot_target(
    pot_id: &str,
    pedal: &PedalDef,
    stages: &[WdfStage],
    multi_nl_stages: &[MultiNlStage],
    sidechain_comp_ids: &HashMap<String, usize>,
    lfo_ids: &[String],
    bbd_id_to_idx: &HashMap<String, usize>,
    delay_id_to_idx: &HashMap<String, usize>,
) -> (ControlTarget, bool) {
    // 1. Net scan: pot wired to LFO/BBD/delay targets.
    // This runs early because pots with modulation routing (e.g. Speed.wiper -> LFO1.rate)
    // may also be stamped into WDF stages via their other lugs (Speed.a -> vcc, etc.).
    // The explicit modulation connection takes priority over physical placement.
    if let Some(target) = scan_pot_nets(pot_id, pedal, lfo_ids, bbd_id_to_idx, delay_id_to_idx) {
        return (target, false);
    }

    // 2. Pot in WDF stage (tree, opamp children, PassiveRType children)
    for (si, stage) in stages.iter().enumerate() {
        if stage.has_pot(pot_id) {
            return (ControlTarget::PotInStage(si), false);
        }
    }

    // 3. Pot in multi-NL stage (includes BJT bias pots, output volume pots)
    for (mi, mnl) in multi_nl_stages.iter().enumerate() {
        for (pi, child) in mnl.pot_children.iter().enumerate() {
            if has_pot(child, pot_id) {
                return (ControlTarget::PotInMultiNlStage(mi, pi), false);
            }
        }
    }

    // 4. Sidechain pot
    let sc_idx = sidechain_comp_ids
        .get(pot_id)
        .or_else(|| sidechain_comp_ids.get(&format!("{pot_id}__aw")))
        .or_else(|| sidechain_comp_ids.get(&format!("{pot_id}__wb")));
    if let Some(&idx) = sc_idx {
        return (ControlTarget::SidechainControl(idx), false);
    }

    // 5. BBD mix detection via topology (pot reaches BBD output through passives)
    if !bbd_id_to_idx.is_empty() && pot_reaches_bbd_output(pot_id, pedal, bbd_id_to_idx) {
        // Find the stage containing this mix pot
        for (si, stage) in stages.iter().enumerate() {
            if let RootKind::PassiveRType { children, .. } = &stage.root {
                if children.iter().any(|c| has_pot(c, pot_id)) {
                    return (ControlTarget::PotInStage(si), true);
                }
            }
            if has_pot(&stage.tree, pot_id) {
                return (ControlTarget::PotInStage(si), true);
            }
        }
        return (ControlTarget::PotInStage(0), true);
    }

    // 6. PassiveRType children — already covered by step 2's stage.has_pot()

    // 7. Fallback
    eprintln!("WARNING: pot '{pot_id}' not bound to any audio stage — control will have no effect");
    (ControlTarget::PotInStage(0), false)
}

/// Scan nets for pot pins wired to LFO/BBD/delay modulation targets.
///
/// Handles both forward (Pot.wiper -> LFO.rate) and reverse (BBD.out -> Pot.a)
/// net directions.
fn scan_pot_nets(
    pot_id: &str,
    pedal: &PedalDef,
    lfo_ids: &[String],
    bbd_id_to_idx: &HashMap<String, usize>,
    delay_id_to_idx: &HashMap<String, usize>,
) -> Option<ControlTarget> {
    let pot_pins = ["wiper", "w", "b", "a"];
    for net in &pedal.nets {
        // Forward: Pot.pin -> Target.modpin
        if let Pin::ComponentPin { component, pin } = &net.from {
            if component == pot_id && pot_pins.contains(&pin.as_str()) {
                for to_pin in &net.to {
                    if let Pin::ComponentPin {
                        component: target_comp,
                        pin: target_pin,
                    } = to_pin
                    {
                        if target_pin == "rate" {
                            if let Some(idx) = lfo_ids.iter().position(|id| id == target_comp) {
                                return Some(ControlTarget::LfoRate(idx));
                            }
                        }
                        if target_pin == "clock" {
                            if let Some(&idx) = bbd_id_to_idx.get(target_comp.as_str()) {
                                return Some(ControlTarget::BbdClockRate(idx));
                            }
                        }
                        if target_pin == "delay_time" {
                            if let Some(&idx) = delay_id_to_idx.get(target_comp.as_str()) {
                                return Some(ControlTarget::DelayTime(idx));
                            }
                        }
                        if target_pin == "feedback" {
                            if let Some(&idx) = delay_id_to_idx.get(target_comp.as_str()) {
                                return Some(ControlTarget::DelayFeedback(idx));
                            }
                        }
                    }
                }
            }
        }
        // Reverse: Target.out -> Pot.pin
        for to_pin in &net.to {
            if let Pin::ComponentPin { component, pin } = to_pin {
                if component == pot_id && pot_pins.contains(&pin.as_str()) {
                    if let Pin::ComponentPin {
                        component: src_comp,
                        pin: src_pin,
                    } = &net.from
                    {
                        if src_pin == "out" {
                            if let Some(&idx) = bbd_id_to_idx.get(src_comp.as_str()) {
                                return Some(ControlTarget::BbdFeedback(idx));
                            }
                            if let Some(idx) = lfo_ids.iter().position(|id| id == src_comp) {
                                return Some(ControlTarget::LfoDepth(idx));
                            }
                        }
                    }
                }
            }
        }
    }
    None
}

// ═══════════════════════════════════════════════════════════════════════════
// LFO binding
// ═══════════════════════════════════════════════════════════════════════════

/// Build LFO bindings from LFO components and their net connections.
pub(super) fn build_lfo_bindings(
    pedal: &PedalDef,
    stages: &[WdfStage],
    multi_nl_stages: &[MultiNlStage],
    delay_id_to_idx: &HashMap<String, usize>,
    sample_rate: f64,
) -> Vec<LfoBinding> {
    let mut lfos = Vec::new();

    for comp in &pedal.components {
        if let Some(lfo) = comp
            .kind
            .as_any()
            .downcast_ref::<crate::compiler::components::Lfo>()
        {
            let waveform = match lfo.waveform {
                LfoWaveformDsl::Sine => crate::elements::LfoWaveform::Sine,
                LfoWaveformDsl::Triangle => crate::elements::LfoWaveform::Triangle,
                LfoWaveformDsl::Square => crate::elements::LfoWaveform::Square,
                LfoWaveformDsl::SawUp => crate::elements::LfoWaveform::SawUp,
                LfoWaveformDsl::SawDown => crate::elements::LfoWaveform::SawDown,
                LfoWaveformDsl::SampleAndHold => crate::elements::LfoWaveform::SampleAndHold,
            };

            let base_freq = 1.0 / (2.0 * std::f64::consts::PI * lfo.timing_r * lfo.timing_c);
            let mut lfo = crate::elements::Lfo::new(waveform, sample_rate);
            lfo.set_rate(base_freq);

            let mut created_all_jfet_binding = false;

            for net in &pedal.nets {
                if let Pin::ComponentPin { component, pin } = &net.from {
                    if component == &comp.id && pin == "out" {
                        for target_pin in &net.to {
                            if let Pin::ComponentPin {
                                component: target_comp,
                                pin: target_prop,
                            } = target_pin
                            {
                                // Direct resolution first.
                                let resolved = resolve_modulation_target(
                                    target_prop,
                                    target_comp,
                                    pedal,
                                    stages,
                                    multi_nl_stages,
                                    delay_id_to_idx,
                                    &mut created_all_jfet_binding,
                                )
                                .or_else(|| {
                                    // Trace through resistive path for indirect connections.
                                    let mut visited = HashSet::new();
                                    let (final_comp, final_pin) = trace_through_resistive_path(
                                        target_comp,
                                        target_prop,
                                        pedal,
                                        &mut visited,
                                    )?;
                                    resolve_modulation_target(
                                        &final_pin,
                                        &final_comp,
                                        pedal,
                                        stages,
                                        multi_nl_stages,
                                        delay_id_to_idx,
                                        &mut created_all_jfet_binding,
                                    )
                                });

                                if let Some((target, bias, range)) = resolved {
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
    }

    lfos
}

// ═══════════════════════════════════════════════════════════════════════════
// Envelope follower binding
// ═══════════════════════════════════════════════════════════════════════════

/// Build envelope follower bindings.
pub(super) fn build_envelope_bindings(
    pedal: &PedalDef,
    stages: &[WdfStage],
    multi_nl_stages: &[MultiNlStage],
    delay_id_to_idx: &HashMap<String, usize>,
    sample_rate: f64,
) -> Vec<EnvelopeBinding> {
    let mut envelopes = Vec::new();

    for comp in &pedal.components {
        if let Some(ef) = comp
            .kind
            .as_any()
            .downcast_ref::<crate::compiler::components::EnvelopeFollower>()
        {
            let envelope = crate::elements::EnvelopeFollower::from_rc(
                ef.attack_r,
                ef.attack_c,
                ef.release_r,
                ef.release_c,
                ef.sensitivity_r,
                sample_rate,
            );

            let mut unused_flag = false;

            for net in &pedal.nets {
                if let Pin::ComponentPin { component, pin } = &net.from {
                    if component == &comp.id && pin == "out" {
                        for target_pin in &net.to {
                            if let Pin::ComponentPin {
                                component: target_comp,
                                pin: target_prop,
                            } = target_pin
                            {
                                if let Some((target, bias, range)) = resolve_modulation_target(
                                    target_prop,
                                    target_comp,
                                    pedal,
                                    stages,
                                    multi_nl_stages,
                                    delay_id_to_idx,
                                    &mut unused_flag,
                                ) {
                                    envelopes.push(EnvelopeBinding {
                                        envelope: envelope.clone(),
                                        target,
                                        bias,
                                        range,
                                        env_id: comp.id.clone(),
                                        // Legacy 6-pass builder (no runtime
                                        // Stage list): global-input tap.
                                        tap: EnvelopeTapSource::default(),
                                    });
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    envelopes
}

/// Resolve an envelope follower's detector tap (`EF.in -> <node>`) to a
/// runtime signal source.
///
/// Historically the runtime fed EVERY envelope follower the global pedal
/// input, ignoring the `.in` net entirely (audit gap G1). This resolves the
/// tap at compile time with the following rule, applied to the electrical
/// node the `EF.in` pin sits on:
///
/// 1. **No `.in` net** → [`EnvelopeTapSource::GlobalInput`] (legacy
///    behavior, also the safe default).
/// 2. Walk outward from the tap node through PASSIVE two-terminal-ish
///    components only (resistors, capacitors, inductors, pots and their
///    switched/tempco variants), level by level, never expanding through
///    supply rails or ground. At each level (level 0 = the tap node
///    itself):
///    - if the reserved `in` node is reached → `GlobalInput` (the tap is
///      input-coupled, e.g. `EF.in -> C_in.b`; checked before `out` so a
///      degenerate node touching both resolves to the legacy source);
///    - if the reserved `out` node is reached →
///      [`EnvelopeTapSource::StageOutput`] of the LAST serial stage (the
///      tap is output-coupled — a feedback detector, e.g. an 1176 rev D
///      tapping the output pot wiper).
///
///    Active devices (transistors, tubes, op-amps, transformers, ...) are
///    deliberate walk barriers: a tap behind an amplifier is NOT the same
///    signal as the node on the amplifier's other side.
/// 3. Otherwise (an interior tap node that reaches neither `in` nor `out`
///    through passives): the first stage of the serial chain that owns a
///    component touched by the walk (tap-node neighbors first) →
///    `StageOutput` of that stage — i.e. the tap is approximated by the
///    output of the stage the tapped network compiled into.
/// 4. If nothing matches → `GlobalInput`.
///
/// The runtime feeds `StageOutput(i)` detectors stage `i`'s post-wiper
/// serial output each sample (see the serial loop in
/// `CompiledPedal::process`): feed-forward taps modulate the same sample,
/// feedback taps the next (one-sample delay).
pub(super) fn resolve_envelope_tap(
    pedal: &PedalDef,
    stages: &[Stage],
    ef_id: &str,
) -> EnvelopeTapSource {
    use std::collections::VecDeque;

    // Pin → set key. Reserved nodes are namespaced so a component named
    // "in" can't collide with the reserved input node.
    let pin_key = |p: &Pin| -> Option<String> {
        match p {
            Pin::Reserved(n) => Some(format!("@{n}")),
            Pin::ComponentPin { component, pin } => Some(format!("{component}.{pin}")),
            Pin::Fork { .. } | Pin::SubcircuitPort { .. } => None,
        }
    };
    // Rails/ground must never join the node closure: half the circuit
    // touches ground, and expanding through it would merge everything.
    let is_rail_key = |key: &str| matches!(key.strip_prefix('@'), Some(n) if n == "gnd" || n == "vcc" || pedal.is_supply_rail(n));

    let passive_pins = |comp_id: &str| -> Option<Vec<&'static str>> {
        let comp = pedal.components.iter().find(|c| c.id == comp_id)?;
        if comp.kind.is_pot() {
            return Some(vec!["a", "b", "w", "wiper"]);
        }
        match comp.kind.type_tag() {
            "resistor" | "capacitor" | "inductor" | "tempco resistor" | "switched resistor"
            | "switched capacitor" | "switched inductor" => Some(vec!["a", "b"]),
            _ => None,
        }
    };

    let last_serial_stage = stages.iter().rposition(|s| !s.bypass_serial());

    let mut visited: HashSet<String> = HashSet::new();
    // Components touched by the walk, in BFS order (tap-node neighbors
    // first) — fallback candidates for rule 3.
    let mut touched_components: Vec<String> = Vec::new();
    // Current BFS level's frontier of pin keys.
    let mut frontier: Vec<String> = vec![format!("{ef_id}.in")];
    visited.insert(frontier[0].clone());

    while !frontier.is_empty() {
        // Expand the frontier to its full electrical node closure: any net
        // sharing a (non-rail) pin with the closure contributes all its
        // pins. Iterate to fixpoint since nets chain through named nodes.
        let mut closure: Vec<String> = frontier.clone();
        let mut closure_set: HashSet<String> = closure.iter().cloned().collect();
        loop {
            let mut grew = false;
            for net in &pedal.nets {
                let keys: Vec<String> = std::iter::once(&net.from)
                    .chain(net.to.iter())
                    .filter_map(&pin_key)
                    .collect();
                if !keys
                    .iter()
                    .any(|k| closure_set.contains(k) && !is_rail_key(k))
                {
                    continue;
                }
                for k in keys {
                    if !is_rail_key(&k) && closure_set.insert(k.clone()) {
                        closure.push(k);
                        grew = true;
                    }
                }
            }
            if !grew {
                break;
            }
        }

        // Reserved-node checks for this level. `in` first: a node touching
        // both resolves to the legacy global-input source.
        if closure_set.contains("@in") {
            return EnvelopeTapSource::GlobalInput;
        }
        if closure_set.contains("@out") {
            return match last_serial_stage {
                Some(idx) => EnvelopeTapSource::StageOutput(idx),
                None => EnvelopeTapSource::GlobalInput,
            };
        }

        // Cross passive components to their other pins → next level.
        let mut next: Vec<String> = Vec::new();
        let mut queue: VecDeque<String> = closure.into();
        while let Some(key) = queue.pop_front() {
            let Some((comp_id, pin)) = key.rsplit_once('.') else {
                continue;
            };
            if comp_id == ef_id {
                continue;
            }
            visited.insert(key.clone());
            if !touched_components.iter().any(|c| c == comp_id) {
                touched_components.push(comp_id.to_string());
            }
            let Some(pins) = passive_pins(comp_id) else {
                continue;
            };
            for other in pins {
                if other == pin {
                    continue;
                }
                let other_key = format!("{comp_id}.{other}");
                if visited.insert(other_key.clone()) {
                    next.push(other_key);
                }
            }
        }
        frontier = next;
    }

    // Rule 3 fallback: first serial stage owning a touched component.
    for comp_id in &touched_components {
        if let Some(idx) = stages.iter().position(
            |s| matches!(s, Stage::Wdf(w) if !w.bypass_serial && w.contains_component(comp_id)),
        ) {
            return EnvelopeTapSource::StageOutput(idx);
        }
    }

    EnvelopeTapSource::GlobalInput
}

/// Build envelope-follower → JFET bindings for the SPQR pipeline.
///
/// The SPQR pipeline never calls the legacy [`build_envelope_bindings`] (it
/// expects the removed 6-pass pipeline's bare `WdfStage` list), so
/// `CompiledPedal::envelopes` stayed empty and `EF.out -> J.vgs` nets were
/// inert (audit gap G2). This builder works on the runtime [`Stage`] list.
///
/// A gate-modulated JFET compiles to a `jfet_vr` LEAF (`resolve_edges()`
/// reclassifies the drain–source edge as `EdgeKind::Linear`), not a stage
/// root, so the binding targets the leaf by component id
/// (`ModulationTarget::JfetVrVgs`). Bias/range come from the component's
/// `modulation_sink()` — the same scaling the LFO path uses, mapping the
/// envelope's [0,1] output onto a negative Vgs swing toward pinch-off.
///
/// Photocoupler LED sinks (audit gap G3) are bound here too: the envelope
/// drives the LED of a photocoupler that compiled as a leaf (WDF tree or
/// PassiveRType MNA child — its netlist position), or, when no leaf exists,
/// as an op-amp input-path gain element. The classification is exclusive
/// (leaf-positioned XOR input-path) so a shunt CdS cell is never doubly
/// modeled as a series transmission gain.
///
/// Scope: JFET vgs/gate and photocoupler LED sinks. Other sink kinds
/// (VCA cv, ...) are separate audit gaps (G4) and keep their current
/// behavior.
pub(super) fn build_envelope_jfet_bindings(
    pedal: &PedalDef,
    stages: &[Stage],
    sample_rate: f64,
) -> Vec<EnvelopeBinding> {
    use super::component::ModulationSinkKind;

    let mut envelopes = Vec::new();

    for comp in &pedal.components {
        let Some(ef) = comp
            .kind
            .as_any()
            .downcast_ref::<crate::compiler::components::EnvelopeFollower>()
        else {
            continue;
        };
        let envelope = crate::elements::EnvelopeFollower::from_rc(
            ef.attack_r,
            ef.attack_c,
            ef.release_r,
            ef.release_c,
            ef.sensitivity_r,
            sample_rate,
        );
        // Detector tap: resolve the `EF.in -> <node>` net to its runtime
        // signal source (global input vs. a specific stage's output).
        let tap = resolve_envelope_tap(pedal, stages, &comp.id);

        for net in &pedal.nets {
            let Pin::ComponentPin { component, pin } = &net.from else {
                continue;
            };
            if component != &comp.id || pin != "out" {
                continue;
            }
            for target_pin in &net.to {
                let Pin::ComponentPin {
                    component: target_comp,
                    pin: target_prop,
                } = target_pin
                else {
                    continue;
                };
                let Some(sink) = pedal
                    .components
                    .iter()
                    .find(|c| &c.id == target_comp)
                    .and_then(|c| c.kind.modulation_sink(target_prop))
                else {
                    continue;
                };
                match sink.target_kind {
                    ModulationSinkKind::JfetVgs => {
                        let Some(stage_idx) = stages.iter().position(
                            |s| matches!(s, Stage::Wdf(w) if w.contains_jfet_vr(target_comp)),
                        ) else {
                            continue;
                        };
                        envelopes.push(EnvelopeBinding {
                            envelope: envelope.clone(),
                            target: ModulationTarget::JfetVrVgs {
                                stage_idx,
                                comp_id: target_comp.clone(),
                            },
                            bias: sink.bias,
                            range: sink.range,
                            env_id: comp.id.clone(),
                            tap,
                        });
                    }
                    ModulationSinkKind::PhotocouplerLed => {
                        // Leaf-positioned XOR input-path (audit gap G3):
                        // prefer the stage holding the photocoupler at its
                        // netlist position (WDF tree leaf or PassiveRType MNA
                        // child); only fall back to an op-amp stage that
                        // models it as an input-path gain element when no
                        // leaf exists anywhere. Skip (no binding) if the
                        // photocoupler didn't compile into any stage.
                        let stage_idx = stages
                            .iter()
                            .position(
                                |s| matches!(s, Stage::Wdf(w) if w.contains_photocoupler(target_comp)),
                            )
                            .or_else(|| {
                                stages.iter().position(|s| {
                                    matches!(s, Stage::Wdf(w) if w
                                        .input_photocouplers
                                        .iter()
                                        .any(|pc| pc.comp_id == *target_comp))
                                })
                            });
                        let Some(stage_idx) = stage_idx else {
                            continue;
                        };
                        // The sink's bias/range (0.5/0.5) map a BIPOLAR LFO
                        // onto LED drive [0, 1]; the envelope output is
                        // already unipolar [0, 1] and detector silence must
                        // mean LED dark, so pass it through unscaled.
                        envelopes.push(EnvelopeBinding {
                            envelope: envelope.clone(),
                            target: ModulationTarget::PhotocouplerLed {
                                stage_idx,
                                comp_id: target_comp.clone(),
                            },
                            bias: 0.0,
                            range: 1.0,
                            env_id: comp.id.clone(),
                            tap,
                        });
                    }
                    _ => continue,
                }
            }
        }
    }

    envelopes
}

// ═══════════════════════════════════════════════════════════════════════════
// Sidechain construction
// ═══════════════════════════════════════════════════════════════════════════

/// Build sidechain processors.
pub(super) fn build_sidechains(
    pedal: &PedalDef,
    graph: &CircuitGraph,
    sample_rate: f64,
) -> (Vec<SidechainProcessor>, HashMap<String, usize>) {
    let mut processors = Vec::new();
    let mut comp_to_sc: HashMap<String, usize> = HashMap::new();

    for sc_info in &pedal.sidechains {
        let tap = match graph.node_names.get(&sc_info.tap_node) {
            Some(&n) => n,
            None => {
                eprintln!("[sidechain] tap node '{}' not found", sc_info.tap_node);
                continue;
            }
        };
        let cv = match graph.node_names.get(&sc_info.cv_node) {
            Some(&n) => n,
            None => {
                eprintln!("[sidechain] cv node '{}' not found", sc_info.cv_node);
                continue;
            }
        };

        let partition = match graph.partition_sidechain(tap, cv) {
            Some(p) => p,
            None => {
                eprintln!(
                    "[sidechain] partition returned None for tap={} cv={}",
                    sc_info.tap_node, sc_info.cv_node
                );
                continue;
            }
        };

        let sc_idx = processors.len(); // Index this sidechain will have if it compiles OK
        let mut sc_component_ids: HashSet<String> = HashSet::new();
        for &edge_idx in &partition.sidechain_edge_indices {
            let comp = &graph.components[graph.edges[edge_idx].comp_idx];
            let id = comp.id.clone();
            sc_component_ids.insert(id.clone());
            comp_to_sc.insert(id.clone(), sc_idx);
            // Also insert the base pot name for Baxandall-decomposed pots.
            // Graph construction decomposes pots into __aw/__wb edges, but the
            // PedalDef still uses the original name. Both must be present so
            // extract_sidechain_def can match components AND nets.
            if let Some(base) = id.strip_suffix("__aw").or_else(|| id.strip_suffix("__wb")) {
                sc_component_ids.insert(base.to_string());
                comp_to_sc.insert(base.to_string(), sc_idx);
            }
        }

        eprintln!(
            "[sidechain] tap={} cv={}",
            sc_info.tap_node, sc_info.cv_node
        );
        eprintln!(
            "[sidechain] {} edges, {} components",
            partition.sidechain_edge_indices.len(),
            sc_component_ids.len()
        );
        let mut sorted: Vec<_> = sc_component_ids.iter().cloned().collect();
        sorted.sort();
        eprintln!("[sidechain] {}", sorted.join(", "));

        let sc_def = extract_sidechain_def(
            pedal,
            &sc_component_ids,
            &sc_info.tap_node,
            &sc_info.cv_node,
        );

        eprintln!(
            "[sidechain] sub-def: {} comps, {} nets, {} controls, {} trims",
            sc_def.components.len(),
            sc_def.nets.len(),
            sc_def.controls.len(),
            sc_def.trims.len()
        );

        // Compile the sidechain as individual stages. Multi-stage amplifier
        // cascades need per-stage NR solves for correct gain accumulation.
        // Only tightly-coupled junctions (bridge rectifiers) are collapsed.
        let sc_options = super::compile::CompileOptions {
            collapse_nl: false,
            ..Default::default()
        };
        match super::compile::compile_pedal_with_options(&sc_def, sample_rate, sc_options) {
            Ok(compiled) => {
                eprintln!(
                    "[sidechain] compiled OK: {} stages, {} push-pull, {} multi-nl",
                    compiled.debug_stage_count(),
                    compiled.debug_push_pull_count(),
                    compiled.debug_multi_nl_count(),
                );
                eprintln!("[sidechain] debug_dump:\n{}", compiled.debug_dump());
                processors.push(SidechainProcessor {
                    circuit: compiled,
                    cv_delayed: 0.0,
                });
            }
            Err(e) => {
                eprintln!("[sidechain] FAILED: {e}");
            }
        }
    }

    (processors, comp_to_sc)
}

// ═══════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Resolve a target pin and component to a `(ModulationTarget, bias, range)`.
///
/// Uses the component's `modulation_sink()` for target kind + bias/range,
/// then resolves stage indices from the compiled stage list.
///
/// `created_all_jfet_binding` guards multi-JFET AllJfetVgs (LFO only).
fn resolve_modulation_target(
    target_pin: &str,
    target_comp: &str,
    pedal: &PedalDef,
    stages: &[WdfStage],
    multi_nl_stages: &[super::stage::MultiNlStage],
    delay_id_to_idx: &HashMap<String, usize>,
    created_all_jfet_binding: &mut bool,
) -> Option<(ModulationTarget, f64, f64)> {
    use super::component::ModulationSinkKind;

    // Look up the component and ask for its modulation sink.
    let comp_kind = pedal
        .components
        .iter()
        .find(|c| c.id == target_comp)
        .map(|c| &c.kind);
    let sink = comp_kind?.modulation_sink(target_pin)?;

    let target = match sink.target_kind {
        ModulationSinkKind::JfetVgs => {
            // Leaf resolution first: a JFET whose gate is modulated compiles
            // to a `jfet_vr` LEAF (EdgeKind::Linear) inside a stage tree, not
            // a stage root — root-targeting set_jfet_vgs() would silently
            // no-op. Find the stage whose tree contains this exact component.
            let tc = target_comp;
            if let Some(stage_idx) = stages.iter().position(|s| s.contains_jfet_vr(tc)) {
                return Some((
                    ModulationTarget::JfetVrVgs {
                        stage_idx,
                        comp_id: tc.to_string(),
                    },
                    sink.bias,
                    sink.range,
                ));
            }
            let jfet_count = stages
                .iter()
                .filter(|s| matches!(&s.root, RootKind::Jfet(_) | RootKind::JfetVr(_)))
                .count();
            if jfet_count > 1 {
                if *created_all_jfet_binding {
                    return None;
                }
                *created_all_jfet_binding = true;
                ModulationTarget::AllJfetVgs
            } else if let Some(stage_idx) = stages
                .iter()
                .position(|s| matches!(&s.root, RootKind::Jfet(_) | RootKind::JfetVr(_)))
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
        ModulationSinkKind::PhotocouplerLed => {
            // Find the stage containing this photocoupler. Classification is
            // leaf-positioned XOR op-amp-input-path (audit gap G3): prefer the
            // stage holding the photocoupler as a LEAF (WDF tree or
            // PassiveRType MNA child — its netlist position), and only fall
            // back to a stage that models it as an op-amp input-path gain
            // element when no leaf exists anywhere.
            let tc = target_comp;
            let stage_idx = stages
                .iter()
                .position(|s| s.contains_photocoupler(tc))
                .or_else(|| {
                    stages
                        .iter()
                        .position(|s| s.input_photocouplers.iter().any(|pc| pc.comp_id == tc))
                })
                .unwrap_or(0);
            ModulationTarget::PhotocouplerLed {
                stage_idx,
                comp_id: tc.to_string(),
            }
        }
        ModulationSinkKind::TriodeVgk => {
            let stage_idx = stages
                .iter()
                .position(|s| matches!(&s.root, RootKind::Triode(_)))
                .unwrap_or(0);
            ModulationTarget::TriodeVgk { stage_idx }
        }
        ModulationSinkKind::VariMuVgk => {
            if let Some(stage_idx) = stages
                .iter()
                .position(|s| matches!(&s.root, RootKind::VariMu(_)))
            {
                ModulationTarget::VariMuVgk { stage_idx }
            } else {
                let stage_idx = stages
                    .iter()
                    .position(|s| matches!(&s.root, RootKind::Triode(_)))
                    .unwrap_or(0);
                ModulationTarget::TriodeVgk { stage_idx }
            }
        }
        ModulationSinkKind::PentodeVg1k => {
            let stage_idx = stages
                .iter()
                .position(|s| matches!(&s.root, RootKind::Pentode(_)))
                .unwrap_or(0);
            ModulationTarget::PentodeVg1k { stage_idx }
        }
        ModulationSinkKind::MosfetVgs => {
            let stage_idx = stages
                .iter()
                .position(|s| matches!(&s.root, RootKind::Mosfet(_)))
                .unwrap_or(0);
            ModulationTarget::MosfetVgs { stage_idx }
        }
        ModulationSinkKind::OtaIabc => {
            if let Some(multi_nl_idx) = multi_nl_stages.iter().position(|s| s.has_linearized_ota())
            {
                ModulationTarget::OtaIabcLinear { multi_nl_idx }
            } else {
                let stage_idx = stages
                    .iter()
                    .position(|s| matches!(&s.root, RootKind::Ota(_)))
                    .unwrap_or(0);
                ModulationTarget::OtaIabc { stage_idx }
            }
        }
        ModulationSinkKind::BbdClock => ModulationTarget::BbdClock { bbd_idx: 0 },
        ModulationSinkKind::VcaCv => {
            // vcas[i] is declaration-ordered (vca_lowering index contract).
            let vca_idx = pedal
                .components
                .iter()
                .filter(|c| {
                    c.kind
                        .as_any()
                        .downcast_ref::<super::components::Vca>()
                        .is_some()
                })
                .position(|c| c.id == target_comp)
                .unwrap_or(0);
            ModulationTarget::VcaCv { vca_idx }
        }
        ModulationSinkKind::DelaySpeed => {
            let delay_idx = delay_id_to_idx.get(target_comp).copied().unwrap_or(0);
            ModulationTarget::DelaySpeed { delay_idx }
        }
        ModulationSinkKind::DelayTime => {
            let delay_idx = delay_id_to_idx.get(target_comp).copied().unwrap_or(0);
            ModulationTarget::DelayTime { delay_idx }
        }
        ModulationSinkKind::SpringDwell => {
            // Spring dwell CV is bound by the spring_lowering pass (per-instance
            // index contract); this legacy resolver maps to instance 0 as a
            // safe default if ever reached.
            ModulationTarget::SpringDwell { spring_idx: 0 }
        }
    };

    Some((target, sink.bias, sink.range))
}

/// Trace through resistive paths (pots, resistors) to find modulation targets.
///
/// Uses `modulation_sink()` to recognize target pins instead of a hardcoded list.
fn trace_through_resistive_path(
    start_comp: &str,
    start_pin: &str,
    pedal: &PedalDef,
    visited: &mut HashSet<String>,
) -> Option<(String, String)> {
    let key = format!("{start_comp}:{start_pin}");
    if visited.contains(&key) {
        return None;
    }
    visited.insert(key);

    let comp = pedal.components.iter().find(|c| c.id == start_comp)?;

    // If this component has a modulation sink on this pin, we found our target.
    if comp.kind.modulation_sink(start_pin).is_some() {
        return Some((start_comp.to_string(), start_pin.to_string()));
    }

    // Only trace through resistive elements (pots, resistors).
    let other_pins: Vec<&str> = if comp.kind.is_pot() {
        match start_pin {
            "a" => vec!["b", "wiper"],
            "b" | "wiper" => vec!["a"],
            _ => return None,
        }
    } else if comp.kind.type_tag() == "resistor" {
        match start_pin {
            "a" => vec!["b"],
            "b" => vec!["a"],
            _ => return None,
        }
    } else {
        return None;
    };

    for other_pin in other_pins {
        for net in &pedal.nets {
            if let Pin::ComponentPin { component, pin } = &net.from {
                if component == start_comp && pin == other_pin {
                    for target in &net.to {
                        if let Pin::ComponentPin {
                            component: next_comp,
                            pin: next_pin,
                        } = target
                        {
                            if let Some(result) =
                                trace_through_resistive_path(next_comp, next_pin, pedal, visited)
                            {
                                return Some(result);
                            }
                        }
                    }
                }
            }
            for src_target in &net.to {
                if let Pin::ComponentPin { component, pin } = src_target {
                    if component == start_comp && pin == other_pin {
                        if let Pin::ComponentPin {
                            component: src_comp,
                            pin: src_pin,
                        } = &net.from
                        {
                            if let Some(result) =
                                trace_through_resistive_path(src_comp, src_pin, pedal, visited)
                            {
                                return Some(result);
                            }
                        }
                    }
                }
            }
        }
    }

    None
}

/// Extract a sub-PedalDef for a sidechain sub-circuit.
fn extract_sidechain_def(
    pedal: &PedalDef,
    component_ids: &HashSet<String>,
    tap_node: &str,
    cv_node: &str,
) -> PedalDef {
    // Match components whose ID is in the sidechain, OR whose Baxandall-
    // decomposed variants (__aw/__wb) are. Graph construction decomposes
    // pots into __aw/__wb edges, so component_ids contains decomposed names
    // while pedal.components has the original pot name.
    let components: Vec<ComponentDef> = pedal
        .components
        .iter()
        .filter(|c| {
            component_ids.contains(&c.id)
                || component_ids.contains(&format!("{}__aw", c.id))
                || component_ids.contains(&format!("{}__wb", c.id))
        })
        .cloned()
        .collect();

    let rename = |pin: &Pin| -> Pin {
        match pin {
            Pin::Reserved(n) if n == tap_node => Pin::Reserved("in".to_string()),
            Pin::Reserved(n) if n == cv_node => Pin::Reserved("out".to_string()),
            _ => pin.clone(),
        }
    };

    // Collect internal sidechain node names: Reserved pin names that appear
    // in nets alongside sidechain component pins. These are internal nodes
    // (e.g., node_rect_pos, node_rect_neg in a bridge rectifier) that must
    // be preserved so shared connections aren't lost.
    let sc_internal_nodes: HashSet<String> = {
        let mut nodes = HashSet::new();
        for net in &pedal.nets {
            let all_pins: Vec<&Pin> = std::iter::once(&net.from).chain(net.to.iter()).collect();
            let has_sc_component = all_pins.iter().any(|p| {
                matches!(p, Pin::ComponentPin { component, .. } if component_ids.contains(component))
            });
            if has_sc_component {
                for p in &all_pins {
                    if let Pin::Reserved(n) = p {
                        // Don't include standard pins or supply rails — only internal nodes.
                        if n != tap_node
                            && n != cv_node
                            && n != "gnd"
                            && n != "vcc"
                            && n != "in"
                            && n != "out"
                            && !pedal.is_supply_rail(n)
                        {
                            nodes.insert(n.clone());
                        }
                    }
                }
            }
        }
        nodes
    };

    let nets: Vec<NetDef> = pedal
        .nets
        .iter()
        .filter_map(|net| {
            let from = rename(&net.from);
            let to: Vec<Pin> = net.to.iter().map(&rename).collect();

            let belongs = |p: &Pin| match p {
                Pin::Reserved(n) => {
                    n == "in" || n == "out" || n == "gnd" || n == "vcc"
                        || pedal.is_supply_rail(n)
                        || sc_internal_nodes.contains(n)
                }
                Pin::ComponentPin { component, .. } => component_ids.contains(component),
                Pin::Fork { switch, .. } => component_ids.contains(switch),
                Pin::SubcircuitPort { .. } => false,
            };

            let has_local_component = std::iter::once(&from).chain(to.iter()).any(|p| {
                matches!(p, Pin::ComponentPin { component, .. } if component_ids.contains(component))
            });

            if has_local_component {
                let filtered_to: Vec<Pin> = to.into_iter().filter(|p| belongs(p)).collect();
                if !filtered_to.is_empty() && belongs(&from) {
                    Some(NetDef { from, to: filtered_to })
                } else {
                    None
                }
            } else {
                None
            }
        })
        .collect();

    // Match controls whose component is in the sidechain. Pot components
    // may be decomposed into __aw/__wb variants during graph construction
    // (Baxandall decomposition), so also check for decomposed names.
    let comp_in_sc = |comp: &str| -> bool {
        component_ids.contains(comp)
            || component_ids.contains(&format!("{comp}__aw"))
            || component_ids.contains(&format!("{comp}__wb"))
    };

    let controls: Vec<ControlDef> = pedal
        .controls
        .iter()
        .filter(|c| comp_in_sc(&c.component))
        .cloned()
        .collect();

    let trims: Vec<ControlDef> = pedal
        .trims
        .iter()
        .filter(|c| comp_in_sc(&c.component))
        .cloned()
        .collect();

    PedalDef {
        name: format!("{} (sidechain)", pedal.name),
        subtitle: None,
        supplies: pedal.supplies.clone(),
        components,
        nets,
        controls,
        trims,
        monitors: vec![],
        sidechains: vec![],
        mirrors: hashbrown::HashMap::new(),
        calibrate: false,
        subcircuits: vec![],
        ports: vec![],
        init_hints: vec![],
    }
}

/// Check whether a pot reaches a BBD output through passive elements (resistors, caps).
/// Used to detect wet/dry mix pots that aren't directly wired to the BBD.
fn pot_reaches_bbd_output(
    pot_id: &str,
    pedal: &PedalDef,
    bbd_ids: &HashMap<String, usize>,
) -> bool {
    let mut visited: HashSet<String> = HashSet::new();
    let pot_pins = ["a", "b", "wiper"];

    // Mark pot's own pins as visited to prevent looping back.
    for pp in &pot_pins {
        visited.insert(format!("{pot_id}:{pp}"));
    }

    // Seed BFS with all component pins directly connected to the pot's terminals.
    let mut queue: Vec<(String, String)> = Vec::new();
    for net in &pedal.nets {
        let all: Vec<&Pin> = std::iter::once(&net.from).chain(net.to.iter()).collect();
        let pot_in_net = all.iter().any(|p| {
            matches!(p, Pin::ComponentPin { component, pin }
                if component == pot_id && pot_pins.contains(&pin.as_str()))
        });
        if pot_in_net {
            for p in &all {
                if let Pin::ComponentPin { component, pin } = p {
                    if component.as_str() != pot_id {
                        let key = format!("{component}:{pin}");
                        if visited.insert(key) {
                            queue.push((component.clone(), pin.clone()));
                        }
                    }
                }
            }
        }
    }

    // BFS through passive 2-terminal elements.
    while let Some((comp_id, pin)) = queue.pop() {
        // Found BBD output?
        if pin == "out" && bbd_ids.contains_key(comp_id.as_str()) {
            return true;
        }

        // Only trace through resistors and capacitors.
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

        let other_key = format!("{comp_id}:{other_pin}");
        if !visited.insert(other_key) {
            continue;
        }

        // Find net neighbors of comp_id.other_pin.
        for net in &pedal.nets {
            let all: Vec<&Pin> = std::iter::once(&net.from).chain(net.to.iter()).collect();
            let has_pin = all.iter().any(|p| {
                matches!(p, Pin::ComponentPin { component, pin: p }
                    if component == &comp_id && p == other_pin)
            });
            if has_pin {
                for p in &all {
                    if let Pin::ComponentPin {
                        component: next,
                        pin: next_pin,
                    } = p
                    {
                        if next.as_str() != comp_id.as_str() {
                            let key = format!("{next}:{next_pin}");
                            if visited.insert(key) {
                                queue.push((next.clone(), next_pin.clone()));
                            }
                        }
                    }
                }
            }
        }
    }

    false
}

// Label classification helpers are imported from compiled.rs via `use super::compiled::*`.
