//! Pass 5: Binding & assembly.
//!
//! Moves LFO binding, envelope follower binding, control target resolution,
//! sidechain construction, and peripheral construction from compile.rs.

use std::collections::{HashMap, HashSet};

use crate::dsl::*;
use crate::elements::*;

use super::compiled::*;
use super::graph::CircuitGraph;
use super::helpers::has_pot;
use super::stage::{RootKind, SidechainProcessor, WdfStage};

// ═══════════════════════════════════════════════════════════════════════════
// Control binding
// ═══════════════════════════════════════════════════════════════════════════

/// Build control bindings for all user controls.
pub(super) fn build_controls(
    pedal: &PedalDef,
    stages: &[WdfStage],
    opamp_pot_map: &HashMap<String, (usize, f64, f64, f64, Option<f64>, bool)>,
    lfo_ids: &[String],
    delay_id_to_idx: &HashMap<String, usize>,
    delay_lines_empty: bool,
) -> Vec<ControlBinding> {
    let mut controls = Vec::new();

    for ctrl in &pedal.controls {
        // Check for Switch/RotarySwitch component.
        let switch_info = pedal.components.iter().find_map(|c| {
            if c.id == ctrl.component {
                match &c.kind {
                    ComponentKind::Switch(positions) => Some((c.id.clone(), *positions)),
                    ComponentKind::RotarySwitch(labels) => Some((c.id.clone(), labels.len())),
                    _ => None,
                }
            } else {
                None
            }
        });

        let target = if let Some((switch_id, num_positions)) = switch_info {
            ControlTarget::SwitchPosition { switch_id, num_positions }
        } else if let Some(&(stage_idx, ri, fixed_series_r, max_pot_r, parallel_fixed_r, is_inverting)) = opamp_pot_map.get(&ctrl.component) {
            ControlTarget::OpAmpGain {
                stage_idx,
                ri,
                fixed_series_r,
                max_pot_r,
                parallel_fixed_r,
                is_inverting,
            }
        } else if is_gain_label(&ctrl.label) {
            ControlTarget::PreGain
        } else if is_level_label(&ctrl.label) {
            ControlTarget::OutputGain
        } else if is_rate_label(&ctrl.label) {
            if !lfo_ids.is_empty() {
                ControlTarget::LfoRate(0)
            } else {
                ControlTarget::PreGain
            }
        } else if is_depth_label(&ctrl.label) {
            if !lfo_ids.is_empty() {
                ControlTarget::LfoDepth(0)
            } else {
                ControlTarget::PreGain
            }
        } else if is_delay_time_label(&ctrl.label) && !delay_lines_empty {
            ControlTarget::DelayTime(0)
        } else if is_delay_feedback_label(&ctrl.label) && !delay_lines_empty {
            ControlTarget::DelayFeedback(0)
        } else {
            // Check if this pot connects to an LFO.rate via nets.
            let mut lfo_target = None;
            for net in &pedal.nets {
                if let Pin::ComponentPin { component, pin } = &net.from {
                    if component == &ctrl.component && pin == "wiper" {
                        for to_pin in &net.to {
                            if let Pin::ComponentPin {
                                component: target_comp,
                                pin: target_pin,
                            } = to_pin
                            {
                                if target_pin == "rate" {
                                    if let Some(idx) = lfo_ids.iter().position(|id| id == target_comp) {
                                        lfo_target = Some(ControlTarget::LfoRate(idx));
                                        break;
                                    }
                                }
                                if target_pin == "delay_time" {
                                    if let Some(&idx) = delay_id_to_idx.get(target_comp.as_str()) {
                                        lfo_target = Some(ControlTarget::DelayTime(idx));
                                        break;
                                    }
                                }
                                if target_pin == "feedback" {
                                    if let Some(&idx) = delay_id_to_idx.get(target_comp.as_str()) {
                                        lfo_target = Some(ControlTarget::DelayFeedback(idx));
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
                let mut found_stage = None;
                for (si, stage) in stages.iter().enumerate() {
                    if has_pot(&stage.tree, &ctrl.component) {
                        found_stage = Some(si);
                        break;
                    }
                }
                match found_stage {
                    Some(si) => ControlTarget::PotInStage(si),
                    None => ControlTarget::PreGain,
                }
            }
        };

        let max_r = pedal
            .components
            .iter()
            .find(|c| c.id == ctrl.component)
            .and_then(|c| match &c.kind {
                ComponentKind::Potentiometer(r, _) => Some(*r),
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

    controls
}

// ═══════════════════════════════════════════════════════════════════════════
// LFO binding
// ═══════════════════════════════════════════════════════════════════════════

/// Build LFO bindings from LFO components and their net connections.
pub(super) fn build_lfo_bindings(
    pedal: &PedalDef,
    stages: &[WdfStage],
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
                                let target = match target_prop.as_str() {
                                    "vgs" | "gate" => {
                                        let jfet_count = stages
                                            .iter()
                                            .filter(|s| matches!(&s.root, RootKind::Jfet(_)))
                                            .count();

                                        if jfet_count > 1 {
                                            if created_all_jfet_binding { continue; }
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
                                    "clock" => ModulationTarget::BbdClock { bbd_idx: 0 },
                                    "speed_mod" => {
                                        let delay_idx = delay_id_to_idx
                                            .get(target_comp.as_str())
                                            .copied()
                                            .unwrap_or(0);
                                        ModulationTarget::DelaySpeed { delay_idx }
                                    }
                                    "delay_time" => {
                                        let delay_idx = delay_id_to_idx
                                            .get(target_comp.as_str())
                                            .copied()
                                            .unwrap_or(0);
                                        ModulationTarget::DelayTime { delay_idx }
                                    }
                                    _ => {
                                        // Try tracing through resistive path.
                                        let mut visited = HashSet::new();
                                        if let Some((final_comp, final_pin)) =
                                            trace_through_resistive_path(
                                                target_comp,
                                                target_prop,
                                                pedal,
                                                &mut visited,
                                            )
                                        {
                                            match final_pin.as_str() {
                                                "clock" => ModulationTarget::BbdClock { bbd_idx: 0 },
                                                "vgs" | "gate" => {
                                                    let jfet_count = stages
                                                        .iter()
                                                        .filter(|s| matches!(&s.root, RootKind::Jfet(_)))
                                                        .count();
                                                    if jfet_count > 1 {
                                                        if created_all_jfet_binding { continue; }
                                                        created_all_jfet_binding = true;
                                                        ModulationTarget::AllJfetVgs
                                                    } else if let Some(stage_idx) = stages
                                                        .iter()
                                                        .position(|s| matches!(&s.root, RootKind::Jfet(_)))
                                                    {
                                                        ModulationTarget::JfetVgs { stage_idx }
                                                    } else {
                                                        ModulationTarget::JfetVgs { stage_idx: 0 }
                                                    }
                                                }
                                                "led" => ModulationTarget::PhotocouplerLed {
                                                    stage_idx: 0,
                                                    comp_id: final_comp,
                                                },
                                                "speed_mod" => {
                                                    let delay_idx = delay_id_to_idx
                                                        .get(final_comp.as_str())
                                                        .copied()
                                                        .unwrap_or(0);
                                                    ModulationTarget::DelaySpeed { delay_idx }
                                                }
                                                "delay_time" => {
                                                    let delay_idx = delay_id_to_idx
                                                        .get(final_comp.as_str())
                                                        .copied()
                                                        .unwrap_or(0);
                                                    ModulationTarget::DelayTime { delay_idx }
                                                }
                                                _ => continue,
                                            }
                                        } else {
                                            continue;
                                        }
                                    }
                                };

                                let (bias, range) = modulation_bias_range(&target);

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

    lfos
}

// ═══════════════════════════════════════════════════════════════════════════
// Envelope follower binding
// ═══════════════════════════════════════════════════════════════════════════

/// Build envelope follower bindings.
pub(super) fn build_envelope_bindings(
    pedal: &PedalDef,
    stages: &[WdfStage],
    delay_id_to_idx: &HashMap<String, usize>,
    sample_rate: f64,
) -> Vec<EnvelopeBinding> {
    let mut envelopes = Vec::new();

    for comp in &pedal.components {
        if let ComponentKind::EnvelopeFollower(attack_r, attack_c, release_r, release_c, sensitivity_r) = &comp.kind {
            let envelope = crate::elements::EnvelopeFollower::from_rc(
                *attack_r, *attack_c, *release_r, *release_c, *sensitivity_r, sample_rate,
            );

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
                                    "clock" => ModulationTarget::BbdClock { bbd_idx: 0 },
                                    "speed_mod" => {
                                        let delay_idx = delay_id_to_idx
                                            .get(target_comp.as_str())
                                            .copied()
                                            .unwrap_or(0);
                                        ModulationTarget::DelaySpeed { delay_idx }
                                    }
                                    "delay_time" => {
                                        let delay_idx = delay_id_to_idx
                                            .get(target_comp.as_str())
                                            .copied()
                                            .unwrap_or(0);
                                        ModulationTarget::DelayTime { delay_idx }
                                    }
                                    _ => continue,
                                };

                                let (bias, range) = modulation_bias_range(&target);

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
) -> Vec<SidechainProcessor> {
    let mut processors = Vec::new();

    eprintln!("[sidechain] {} sidechain definitions", pedal.sidechains.len());
    for sc_info in &pedal.sidechains {
        let tap = match graph.node_names.get(&sc_info.tap_node) {
            Some(&n) => n,
            None => { eprintln!("[sidechain] tap node '{}' not found", sc_info.tap_node); continue; },
        };
        let cv = match graph.node_names.get(&sc_info.cv_node) {
            Some(&n) => n,
            None => { eprintln!("[sidechain] cv node '{}' not found", sc_info.cv_node); continue; },
        };

        let partition = match graph.partition_sidechain(tap, cv) {
            Some(p) => p,
            None => { eprintln!("[sidechain] partition returned None for tap={} cv={}", sc_info.tap_node, sc_info.cv_node); continue; },
        };

        let mut sc_component_ids: HashSet<String> = HashSet::new();
        for &edge_idx in &partition.sidechain_edge_indices {
            let comp = &graph.components[graph.edges[edge_idx].comp_idx];
            sc_component_ids.insert(comp.id.clone());
        }

        eprintln!("[sidechain] tap={} cv={}", sc_info.tap_node, sc_info.cv_node);
        eprintln!("[sidechain] {} edges, {} components", partition.sidechain_edge_indices.len(), sc_component_ids.len());
        let mut sorted: Vec<_> = sc_component_ids.iter().cloned().collect();
        sorted.sort();
        eprintln!("[sidechain] {}", sorted.join(", "));

        let sc_def = extract_sidechain_def(
            pedal,
            &sc_component_ids,
            &sc_info.tap_node,
            &sc_info.cv_node,
        );

        eprintln!("[sidechain] sub-def: {} comps, {} nets, {} controls, {} trims",
            sc_def.components.len(), sc_def.nets.len(), sc_def.controls.len(), sc_def.trims.len());

        match super::compile::compile_pedal(&sc_def, sample_rate) {
            Ok(compiled) => {
                eprintln!("[sidechain] compiled OK: {} stages, {} push-pull",
                    compiled.debug_stage_count(), compiled.debug_push_pull_count());
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

    processors
}

// ═══════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Return (bias, range) for a modulation target.
fn modulation_bias_range(target: &ModulationTarget) -> (f64, f64) {
    match target {
        ModulationTarget::JfetVgs { .. } => (-0.45, 0.25),
        ModulationTarget::AllJfetVgs => (-0.45, 0.25),
        ModulationTarget::PhotocouplerLed { .. } => (0.5, 0.5),
        ModulationTarget::TriodeVgk { .. } => (-2.0, 2.0),
        ModulationTarget::VariMuVgk { .. } => (-2.0, 2.0),
        ModulationTarget::PentodeVg1k { .. } => (-2.0, 2.0),
        ModulationTarget::MosfetVgs { .. } => (3.0, 2.0),
        ModulationTarget::OtaIabc { .. } => (0.5, 0.5),
        ModulationTarget::BbdClock { .. } => (0.15, 0.10),
        ModulationTarget::OpAmpVp { .. } => (0.0, 1.0),
        ModulationTarget::DelaySpeed { .. } => (0.0, 0.02),
        ModulationTarget::DelayTime { .. } => (0.5, 0.5),
    }
}

/// Trace through resistive paths (pots, resistors) to find modulation targets.
fn trace_through_resistive_path<'a>(
    start_comp: &str,
    start_pin: &str,
    pedal: &'a PedalDef,
    visited: &mut HashSet<String>,
) -> Option<(String, String)> {
    let key = format!("{}:{}", start_comp, start_pin);
    if visited.contains(&key) {
        return None;
    }
    visited.insert(key);

    let comp_kind = pedal
        .components
        .iter()
        .find(|c| c.id == start_comp)
        .map(|c| &c.kind);

    let is_resistive = matches!(
        comp_kind,
        Some(ComponentKind::Potentiometer(_, _)) | Some(ComponentKind::Resistor(_))
    );
    if !is_resistive {
        let recognized_targets = ["clock", "vgs", "gate", "led", "vgk", "vg1k", "iabc", "speed_mod", "delay_time"];
        if recognized_targets.contains(&start_pin) {
            return Some((start_comp.to_string(), start_pin.to_string()));
        }
        return None;
    }

    let other_pins: Vec<&str> = match comp_kind {
        Some(ComponentKind::Potentiometer(_, _)) => {
            match start_pin {
                "a" => vec!["b", "wiper"],
                "b" | "wiper" => vec!["a"],
                _ => vec![],
            }
        }
        Some(ComponentKind::Resistor(_)) => {
            match start_pin {
                "a" => vec!["b"],
                "b" => vec!["a"],
                _ => vec![],
            }
        }
        _ => vec![],
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
    let components: Vec<ComponentDef> = pedal
        .components
        .iter()
        .filter(|c| component_ids.contains(&c.id))
        .cloned()
        .collect();

    let rename = |pin: &Pin| -> Pin {
        match pin {
            Pin::Reserved(n) if n == tap_node => Pin::Reserved("in".to_string()),
            Pin::Reserved(n) if n == cv_node => Pin::Reserved("out".to_string()),
            _ => pin.clone(),
        }
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

    let controls: Vec<ControlDef> = pedal
        .controls
        .iter()
        .filter(|c| component_ids.contains(&c.component))
        .cloned()
        .collect();

    let trims: Vec<ControlDef> = pedal
        .trims
        .iter()
        .filter(|c| component_ids.contains(&c.component))
        .cloned()
        .collect();

    PedalDef {
        name: format!("{} (sidechain)", pedal.name),
        supplies: pedal.supplies.clone(),
        components,
        nets,
        controls,
        trims,
        monitors: vec![],
        sidechains: vec![],
    }
}

// Label classification helpers are imported from compiled.rs via `use super::compiled::*`.
