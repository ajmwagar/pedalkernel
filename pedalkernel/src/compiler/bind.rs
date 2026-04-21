//! Pass 5: Binding & assembly.
//!
//! Moves LFO binding, envelope follower binding, control target resolution,
//! sidechain construction, and peripheral construction from compile.rs.

use std::collections::{HashMap, HashSet};

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
            component_id: ctrl.component.clone(),
            component_id_aw: format!("{}__aw", ctrl.component),
            component_id_wb: format!("{}__wb", ctrl.component),
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

/// Temporary diagnostic: trace resolve_pot_target for a specific pedal.
#[allow(dead_code)]
fn trace_pot_binding(
    pot_id: &str,
    _pedal: &PedalDef,
    stages: &[WdfStage],
    multi_nl_stages: &[MultiNlStage],
) {
    eprintln!("[bind-trace] pot '{pot_id}' in pedal '{}':", _pedal.name);
    for (si, stage) in stages.iter().enumerate() {
        if has_pot(&stage.tree, pot_id) {
            eprintln!("  -> step 2: PotInStage({si})");
            return;
        }
    }
    for (mi, mnl) in multi_nl_stages.iter().enumerate() {
        for (pi, child) in mnl.pot_children.iter().enumerate() {
            if has_pot(child, pot_id) {
                eprintln!("  -> step 3: PotInMultiNlStage({mi}, {pi})");
                return;
            }
        }
    }
    eprintln!("  -> steps 4+: later fallback");
}

/// Detect output volume pots: 3-terminal pot with wiper reaching `out` and
/// lug B connected to `gnd`.
///
/// Handles both direct (pot.w → out) and 1-hop (pot.w → R.a, R.b → out)
/// wiring patterns.
#[allow(dead_code)]
fn is_output_volume_pot(pot_id: &str, pedal: &PedalDef) -> bool {
    let wiper_pins: &[&str] = &["w", "wiper"];
    let out_names: &[&str] = &["out", "output"];
    let gnd_names: &[&str] = &["gnd", "ground"];

    // Helper: check if a pin matches a component pin.
    let is_comp_pin = |p: &Pin, comp: &str, pin_name: &str| -> bool {
        matches!(p, Pin::ComponentPin { component, pin } if component == comp && pin == pin_name)
    };
    let _is_comp_pin_any = |p: &Pin, comp: &str, pin_names: &[&str]| -> bool {
        matches!(p, Pin::ComponentPin { component, pin }
            if component == comp && pin_names.contains(&pin.as_str()))
    };
    let is_reserved_any = |p: &Pin, names: &[&str]| -> bool {
        matches!(p, Pin::Reserved(name) if names.contains(&name.as_str()))
    };

    // Collect all pins on the same net as a given pin.
    let net_peers = |comp: &str, pin_name: &str| -> Vec<Pin> {
        let mut peers = Vec::new();
        for net in &pedal.nets {
            let in_from = is_comp_pin(&net.from, comp, pin_name);
            let in_to = net.to.iter().any(|p| is_comp_pin(p, comp, pin_name));
            if in_from || in_to {
                // Collect all other pins in this net
                if !is_comp_pin(&net.from, comp, pin_name) {
                    peers.push(net.from.clone());
                }
                for p in &net.to {
                    if !is_comp_pin(p, comp, pin_name) {
                        peers.push(p.clone());
                    }
                }
            }
        }
        peers
    };

    // Check pot.b → gnd
    let b_peers = net_peers(pot_id, "b");
    let b_to_gnd = b_peers.iter().any(|p| is_reserved_any(p, gnd_names));
    if !b_to_gnd {
        return false;
    }

    // Collect peers of pot wiper
    let mut wiper_peers = Vec::new();
    for wp in wiper_pins {
        wiper_peers.extend(net_peers(pot_id, wp));
    }
    if wiper_peers.is_empty() {
        return false;
    }

    // Direct: pot.w on same net as "out"
    if wiper_peers.iter().any(|p| is_reserved_any(p, out_names)) {
        return true;
    }

    // 1-hop: pot.w → Resistor.pin, and Resistor.other_pin → out
    for peer in &wiper_peers {
        if let Pin::ComponentPin {
            component: neighbor_comp,
            pin: neighbor_pin,
        } = peer
        {
            if neighbor_comp == pot_id {
                continue;
            }
            // Check if the neighbor component is a resistor
            let is_resistor = pedal
                .components
                .iter()
                .any(|c| c.id == *neighbor_comp && c.kind.type_tag() == "resistor");
            if !is_resistor {
                continue;
            }
            // Find the neighbor's other pin name
            let other_pin = match neighbor_pin.as_str() {
                "a" => "b",
                "b" => "a",
                _ => continue,
            };
            // Check if the other pin's net contains "out"
            let other_peers = net_peers(neighbor_comp, other_pin);
            if other_peers.iter().any(|p| is_reserved_any(p, out_names)) {
                return true;
            }
        }
    }

    false
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
            // Find the stage containing this photocoupler — either in the WDF tree
            // or as an input-path photocoupler on an opamp stage.
            let tc = target_comp;
            let stage_idx = stages
                .iter()
                .position(|s| {
                    s.tree
                        .find_leaf(&|leaf| {
                            if leaf.comp_id() == Some(tc) {
                                Some(())
                            } else {
                                None
                            }
                        })
                        .is_some()
                        || s.input_photocouplers.iter().any(|pc| pc.comp_id == tc)
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
        ModulationSinkKind::DelaySpeed => {
            let delay_idx = delay_id_to_idx.get(target_comp).copied().unwrap_or(0);
            ModulationTarget::DelaySpeed { delay_idx }
        }
        ModulationSinkKind::DelayTime => {
            let delay_idx = delay_id_to_idx.get(target_comp).copied().unwrap_or(0);
            ModulationTarget::DelayTime { delay_idx }
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
        mirrors: std::collections::HashMap::new(),
        calibrate: false,
        subcircuits: vec![],
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
