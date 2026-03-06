//! Pass 5: Binding & assembly.
//!
//! Moves LFO binding, envelope follower binding, control target resolution,
//! sidechain construction, and peripheral construction from compile.rs.

use std::collections::{HashMap, HashSet};

use crate::dsl::*;
use crate::elements::*;

use super::bjt_bias_analysis::BjtBiasPotInfo;
use super::compiled::*;
use super::component::{Component, ControlParamKind};
use super::graph::CircuitGraph;
use super::helpers::has_pot;
use super::stage::{MultiNlStage, RootKind, SidechainProcessor, WdfStage};

// ═══════════════════════════════════════════════════════════════════════════
// Control binding
// ═══════════════════════════════════════════════════════════════════════════

/// Build control bindings for all user controls.
///
/// Returns `(controls, pot_effects)` where `pot_effects` maps component IDs
/// to behavioral side-effects (op-amp gain formula, BBD mix) that are applied
/// alongside the physical PotInStage update.
pub(super) fn build_controls(
    pedal: &PedalDef,
    stages: &[WdfStage],
    multi_nl_stages: &[MultiNlStage],
    opamp_pot_map: &HashMap<String, (usize, f64, f64, f64, Option<f64>, bool)>,
    bjt_bias_pot_map: &HashMap<String, BjtBiasPotInfo>,
    lfo_ids: &[String],
    delay_id_to_idx: &HashMap<String, usize>,
    _delay_lines_empty: bool,
    sidechain_comp_ids: &HashMap<String, usize>,
    bbd_id_to_idx: &HashMap<String, usize>,
    trigger_id_to_idx: &HashMap<String, usize>,
) -> (Vec<ControlBinding>, HashMap<String, Vec<PotEffect>>) {
    let mut controls = Vec::new();
    let mut pot_effects: HashMap<String, Vec<PotEffect>> = HashMap::new();

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
                let idx = lfo_ids.iter().position(|id| id == &ctrl.component).unwrap_or(0);
                ControlTarget::LfoRate(idx)
            }
            Some(ControlParamKind::LfoDepth) => {
                let idx = lfo_ids.iter().position(|id| id == &ctrl.component).unwrap_or(0);
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
                resolve_pot_target(
                    &ctrl.component,
                    pedal,
                    stages,
                    multi_nl_stages,
                    opamp_pot_map,
                    bjt_bias_pot_map,
                    sidechain_comp_ids,
                    lfo_ids,
                    bbd_id_to_idx,
                    delay_id_to_idx,
                    &mut pot_effects,
                )
            }
            None => {
                eprintln!(
                    "WARNING: no declared control '{}' on component '{}' ({}) — falling back",
                    ctrl.property, ctrl.component, ctrl.label
                );
                ControlTarget::PotInStage(0)
            }
        };

        let max_r = comp
            .and_then(|c| c.kind.resistance())
            .unwrap_or(100_000.0);

        controls.push(ControlBinding {
            label: ctrl.label.clone(),
            target,
            component_id: ctrl.component.clone(),
            max_resistance: max_r,
            range: ctrl.range,
        });
    }

    (controls, pot_effects)
}

/// Resolve a pot-position control to its target.
///
/// Priority:
///  1. Op-amp feedback pot
///  2. BJT bias pot
///  3. Pot in WDF stage tree
///  4. Pot in multi-NL stage
///  5. Sidechain pot
///  6. Net scan: pot wired to LFO/BBD/delay
///  7. BBD mix detection (BFS through passives)
///  8. PassiveRType children
///  9. Fallback warning
fn resolve_pot_target(
    pot_id: &str,
    pedal: &PedalDef,
    stages: &[WdfStage],
    multi_nl_stages: &[MultiNlStage],
    opamp_pot_map: &HashMap<String, (usize, f64, f64, f64, Option<f64>, bool)>,
    bjt_bias_pot_map: &HashMap<String, BjtBiasPotInfo>,
    sidechain_comp_ids: &HashMap<String, usize>,
    lfo_ids: &[String],
    bbd_id_to_idx: &HashMap<String, usize>,
    delay_id_to_idx: &HashMap<String, usize>,
    pot_effects: &mut HashMap<String, Vec<PotEffect>>,
) -> ControlTarget {
    // 1. Op-amp feedback pot
    if let Some(&(stage_idx, ri, fixed_series_r, max_pot_r, parallel_fixed_r, is_inverting)) =
        opamp_pot_map.get(pot_id)
    {
        pot_effects
            .entry(pot_id.to_string())
            .or_default()
            .push(PotEffect::OpAmpGain {
                stage_idx,
                ri,
                fixed_series_r,
                max_pot_r,
                parallel_fixed_r,
                is_inverting,
            });
        return ControlTarget::PotInStage(stage_idx);
    }

    // 2. BJT bias pot
    if let Some(info) = bjt_bias_pot_map.get(pot_id) {
        return ControlTarget::BjtBias {
            max_pot_r: info.max_pot_r,
            taper: info.taper,
        };
    }

    // 3. Pot in WDF stage tree
    for (si, stage) in stages.iter().enumerate() {
        if has_pot(&stage.tree, pot_id) {
            return ControlTarget::PotInStage(si);
        }
    }

    // 4. Pot in multi-NL stage
    for (mi, mnl) in multi_nl_stages.iter().enumerate() {
        for (pi, child) in mnl.pot_children.iter().enumerate() {
            if has_pot(child, pot_id) {
                return ControlTarget::PotInMultiNlStage(mi, pi);
            }
        }
    }

    // 5. Sidechain pot
    let sc_idx = sidechain_comp_ids
        .get(pot_id)
        .or_else(|| sidechain_comp_ids.get(&format!("{pot_id}__aw")))
        .or_else(|| sidechain_comp_ids.get(&format!("{pot_id}__wb")));
    if let Some(&idx) = sc_idx {
        return ControlTarget::SidechainControl(idx);
    }

    // 6. Net scan: pot wired to LFO/BBD/delay targets
    if let Some(target) = scan_pot_nets(pot_id, pedal, lfo_ids, bbd_id_to_idx, delay_id_to_idx) {
        return target;
    }

    // 7. BBD mix detection via topology
    if !bbd_id_to_idx.is_empty() && pot_reaches_bbd_output(pot_id, pedal, bbd_id_to_idx) {
        pot_effects
            .entry(pot_id.to_string())
            .or_default()
            .push(PotEffect::BbdMix);
        // Find the stage containing this mix pot
        for (si, stage) in stages.iter().enumerate() {
            if let RootKind::PassiveRType { children, .. } = &stage.root {
                if children.iter().any(|c| has_pot(c, pot_id)) {
                    return ControlTarget::PotInStage(si);
                }
            }
            if has_pot(&stage.tree, pot_id) {
                return ControlTarget::PotInStage(si);
            }
        }
        return ControlTarget::PotInStage(0);
    }

    // 8. PassiveRType children (2-terminal pots in MNA)
    for (si, stage) in stages.iter().enumerate() {
        if let RootKind::PassiveRType { children, .. } = &stage.root {
            if children.iter().any(|c| has_pot(c, pot_id)) {
                return ControlTarget::PotInStage(si);
            }
        }
    }

    // 9. Fallback
    eprintln!(
        "WARNING: pot '{pot_id}' not bound to any audio stage — control will have no effect"
    );
    ControlTarget::PotInStage(0)
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
    let pot_pins = ["wiper", "b", "a"];
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
        if let ComponentKind::Lfo(waveform_dsl, timing_r, timing_c) = &comp.kind {
            let waveform = match waveform_dsl {
                LfoWaveformDsl::Sine => crate::elements::LfoWaveform::Sine,
                LfoWaveformDsl::Triangle => crate::elements::LfoWaveform::Triangle,
                LfoWaveformDsl::Square => crate::elements::LfoWaveform::Square,
                LfoWaveformDsl::SawUp => crate::elements::LfoWaveform::SawUp,
                LfoWaveformDsl::SawDown => crate::elements::LfoWaveform::SawDown,
                LfoWaveformDsl::SampleAndHold => crate::elements::LfoWaveform::SampleAndHold,
            };

            let base_freq = 1.0 / (2.0 * std::f64::consts::PI * timing_r * timing_c);
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

    eprintln!(
        "[sidechain] {} sidechain definitions",
        pedal.sidechains.len()
    );
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
    let comp_kind = pedal.components.iter().find(|c| c.id == target_comp).map(|c| &c.kind);
    let sink = comp_kind?.modulation_sink(target_pin)?;

    let target = match sink.target_kind {
        ModulationSinkKind::JfetVgs => {
            let jfet_count = stages
                .iter()
                .filter(|s| matches!(&s.root, RootKind::Jfet(_) | RootKind::JfetVr(_)))
                .count();
            if jfet_count > 1 {
                if *created_all_jfet_binding { return None; }
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
        ModulationSinkKind::PhotocouplerLed => ModulationTarget::PhotocouplerLed {
            stage_idx: 0,
            comp_id: target_comp.to_string(),
        },
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
            if let Some(multi_nl_idx) = multi_nl_stages
                .iter()
                .position(|s| s.has_linearized_ota())
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
    let other_pins: Vec<&str> = match &comp.kind {
        ComponentKind::Potentiometer(_, _) => match start_pin {
            "a" => vec!["b", "wiper"],
            "b" | "wiper" => vec!["a"],
            _ => return None,
        },
        ComponentKind::Resistor(_) => match start_pin {
            "a" => vec!["b"],
            "b" => vec!["a"],
            _ => return None,
        },
        _ => return None,
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
        let other_pin = match comp.map(|c| &c.kind) {
            Some(ComponentKind::Resistor(_)) | Some(ComponentKind::Capacitor(_)) => {
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
