//! Main compiler entry point: PedalDef -> CompiledPedal.
//!
//! Orchestrates the 6-pass compilation pipeline:
//! - Pass 0: Graph construction (CircuitGraph::from_pedal)
//! - Pass 1: Element classification (classify.rs)
//! - Pass 2: Op-amp analysis (opamp_analysis.rs)
//! - Pass 3: Stage planning (plan.rs)
//! - Pass 4: Tree building (build.rs)
//! - Pass 5: Binding & assembly (bind.rs)

use std::collections::HashMap;

use crate::dsl::*;
use crate::elements::*;
use crate::oversampling::{Oversampler, OversamplingFactor};
use crate::thermal::ThermalModel;
use crate::tolerance::ToleranceEngine;

use super::compiled::*;
use super::dyn_node::DynNode;
use super::graph::*;
use super::helpers::*;
use super::stage::{RootKind, WdfStage};

// ═══════════════════════════════════════════════════════════════════════════
// Passive circuit gain helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Compute the voltage divider gain for a resistor-only circuit.
fn compute_resistor_divider_gain(graph: &CircuitGraph) -> f64 {
    let r_series = find_resistance_between(graph, graph.in_node, graph.out_node);
    let r_shunt = find_resistance_between(graph, graph.out_node, graph.gnd_node);

    match (r_series, r_shunt) {
        (Some(rs), Some(rsh)) => rsh / (rs + rsh),
        _ => 1.0,
    }
}

/// Find total resistance between two nodes via BFS through resistive elements.
fn find_resistance_between(graph: &CircuitGraph, from: NodeId, to: NodeId) -> Option<f64> {
    if from == to {
        return Some(0.0);
    }
    let mut visited = std::collections::HashSet::new();
    let mut queue = std::collections::VecDeque::new();
    visited.insert(from);
    queue.push_back((from, 0.0));

    while let Some((node, r_so_far)) = queue.pop_front() {
        for (_, e) in graph.edges.iter().enumerate() {
            let (other, matches) = if e.node_a == node {
                (e.node_b, true)
            } else if e.node_b == node {
                (e.node_a, true)
            } else {
                (0, false)
            };
            if !matches {
                continue;
            }

            if let ComponentKind::Resistor(r) = &graph.components[e.comp_idx].kind {
                if other == to {
                    return Some(r_so_far + r);
                }
                if visited.insert(other) {
                    queue.push_back((other, r_so_far + r));
                }
            }
        }
    }
    None
}

/// Filter topology type for simple RC/RL circuits.
enum FilterTopology {
    RcLowpass { a1: f64, b0: f64 },
    RcHighpass { a1: f64, b0: f64 },
    RlLowpass { a1: f64, b0: f64 },
}

/// Detect simple RC/RL filter topology and compute filter parameters.
fn detect_rc_filter_topology(graph: &CircuitGraph, sample_rate: f64) -> Option<FilterTopology> {
    let resistors: Vec<_> = graph.edges.iter()
        .filter(|e| matches!(graph.components[e.comp_idx].kind, ComponentKind::Resistor(_)))
        .collect();
    let capacitors: Vec<_> = graph.edges.iter()
        .filter(|e| matches!(graph.components[e.comp_idx].kind, ComponentKind::Capacitor(_)))
        .collect();
    let inductors: Vec<_> = graph.edges.iter()
        .filter(|e| matches!(graph.components[e.comp_idx].kind, ComponentKind::Inductor(_)))
        .collect();

    // RL lowpass: 1 R, 1 L, 0 C
    if resistors.len() == 1 && inductors.len() == 1 && capacitors.is_empty() {
        let r_edge = resistors[0];
        let l_edge = inductors[0];
        let r = match &graph.components[r_edge.comp_idx].kind { ComponentKind::Resistor(r) => *r, _ => return None };
        let l = match &graph.components[l_edge.comp_idx].kind { ComponentKind::Inductor(l) => *l, _ => return None };

        let l_connects_in_out = (l_edge.node_a == graph.in_node && l_edge.node_b == graph.out_node)
            || (l_edge.node_a == graph.out_node && l_edge.node_b == graph.in_node);
        let r_connects_out_gnd = (r_edge.node_a == graph.out_node && r_edge.node_b == graph.gnd_node)
            || (r_edge.node_a == graph.gnd_node && r_edge.node_b == graph.out_node);

        if l_connects_in_out && r_connects_out_gnd {
            let tau = l / r;
            let alpha = 2.0 * sample_rate * tau;
            let a1 = (1.0 - alpha) / (1.0 + alpha);
            let b0 = 1.0 / (1.0 + alpha);
            return Some(FilterTopology::RlLowpass { a1, b0 });
        }
    }

    // RC filters: 1 R, 1 C
    if resistors.len() != 1 || capacitors.len() != 1 { return None; }
    let r_edge = resistors[0];
    let c_edge = capacitors[0];
    let r = match &graph.components[r_edge.comp_idx].kind { ComponentKind::Resistor(r) => *r, _ => return None };
    let c = match &graph.components[c_edge.comp_idx].kind { ComponentKind::Capacitor(cfg) => cfg.value, _ => return None };

    let r_in_out = (r_edge.node_a == graph.in_node && r_edge.node_b == graph.out_node)
        || (r_edge.node_a == graph.out_node && r_edge.node_b == graph.in_node);
    let c_out_gnd = (c_edge.node_a == graph.out_node && c_edge.node_b == graph.gnd_node)
        || (c_edge.node_a == graph.gnd_node && c_edge.node_b == graph.out_node);

    if r_in_out && c_out_gnd {
        let omega_rc = 2.0 * sample_rate * r * c;
        let a1 = (omega_rc - 1.0) / (omega_rc + 1.0);
        let b0 = 1.0 / (omega_rc + 1.0);
        return Some(FilterTopology::RcLowpass { a1, b0 });
    }

    let c_in_out = (c_edge.node_a == graph.in_node && c_edge.node_b == graph.out_node)
        || (c_edge.node_a == graph.out_node && c_edge.node_b == graph.in_node);
    let r_out_gnd = (r_edge.node_a == graph.out_node && r_edge.node_b == graph.gnd_node)
        || (r_edge.node_a == graph.gnd_node && r_edge.node_b == graph.out_node);

    if c_in_out && r_out_gnd {
        let omega_rc = 2.0 * sample_rate * r * c;
        let a1 = (omega_rc - 1.0) / (omega_rc + 1.0);
        let b0 = omega_rc / (omega_rc + 1.0);
        return Some(FilterTopology::RcHighpass { a1, b0 });
    }

    None
}

/// DC simulation to compute gain correction for passive WDF circuits.
fn compute_passive_compensation(tree: &DynNode) -> f64 {
    let mut tree_copy = tree.clone();
    let rp = tree_copy.port_resistance();
    if rp <= 0.0 { return 1.0; }
    tree_copy.set_voltage(1.0);
    let b = tree_copy.reflected();
    tree_copy.set_incident(-b);
    let v_out = (-b + tree_copy.reflected()) / 2.0;
    if v_out.abs() < 1e-12 { return 1.0; }
    (1.0 / v_out.abs()).min(10.0)
}

// ═══════════════════════════════════════════════════════════════════════════
// Compile options
// ═══════════════════════════════════════════════════════════════════════════

/// Options for pedal compilation.
pub struct CompileOptions {
    pub oversampling: OversamplingFactor,
    pub tolerance: ToleranceEngine,
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

// ═══════════════════════════════════════════════════════════════════════════
// Main compilation pipeline
// ═══════════════════════════════════════════════════════════════════════════

/// Compile a pedal definition with default options.
pub fn compile_pedal(pedal: &PedalDef, sample_rate: f64) -> Result<CompiledPedal, String> {
    compile_pedal_with_options(pedal, sample_rate, CompileOptions::default())
}

/// Compile a pedal definition with custom options.
///
/// Orchestrates the 6-pass compilation pipeline:
/// 1. Graph construction
/// 2. Element classification
/// 3. Op-amp analysis
/// 4. Stage planning
/// 5. Tree building
/// 6. Binding & assembly
pub fn compile_pedal_with_options(
    pedal: &PedalDef,
    sample_rate: f64,
    options: CompileOptions,
) -> Result<CompiledPedal, String> {
    let oversampling = options.oversampling;
    let tolerance = options.tolerance;
    let enable_thermal = options.thermal;

    // ══ Pass 0: Graph construction ════════════════════════════════════
    let mut graph = CircuitGraph::from_pedal(pedal);

    // Apply component tolerance.
    for (i, comp) in graph.components.iter_mut().enumerate() {
        match &mut comp.kind {
            ComponentKind::Resistor(r) => { *r = tolerance.apply_resistor(*r, i); }
            ComponentKind::Capacitor(cfg) => { cfg.value = tolerance.apply_capacitor(cfg.value, i); }
            ComponentKind::Potentiometer(max_r, _) => { *max_r = tolerance.apply_resistor(*max_r, i); }
            _ => {}
        }
    }

    // ══ Pass 1: Element classification ════════════════════════════════
    let classified = super::classify::classify_circuit(&graph, pedal);

    // ══ Pass 2: Op-amp analysis ═══════════════════════════════════════
    let mut opamp_analysis = super::opamp_analysis::analyze_opamps(&graph, pedal);
    let opamp_feedback_gain = 1.0_f64;

    // Build op-amp feedback stages (inverting, non-inverting).
    let mut stages: Vec<WdfStage> = Vec::new();
    let opamp_feedback_stages = super::opamp_analysis::build_opamp_feedback_stages(
        &mut opamp_analysis,
        stages.len(),
        sample_rate,
        oversampling,
    );
    stages.extend(opamp_feedback_stages);

    // Build standalone op-amp stages (no feedback).
    let opamp_stages = super::opamp_analysis::build_standalone_opamp_stages(
        pedal,
        &opamp_analysis.feedback_opamp_ids,
        sample_rate,
    );

    // ══ Pass 3: Stage planning ════════════════════════════════════════
    let (stage_plans, push_pull_plans) = super::plan::plan_stages(&classified, &graph, sample_rate);

    // ══ Pass 4: Tree building ═════════════════════════════════════════
    // Build nonlinear WDF stages from plans.
    let nonlinear_stages = super::build::build_stages(
        &stage_plans,
        &classified,
        &graph,
        &opamp_analysis,
        sample_rate,
        oversampling,
    );
    stages.extend(nonlinear_stages);

    // Build push-pull stages.
    let push_pull_stages = super::build::build_push_pull_stages(
        &push_pull_plans,
        &classified,
        &graph,
        sample_rate,
        oversampling,
    );

    // ══ Passive-only fallback ═════════════════════════════════════════
    let mut passive_attenuation = 1.0;

    if stages.is_empty() {
        let has_reactive = pedal.components.iter().any(|c| {
            matches!(c.kind, ComponentKind::Capacitor(_) | ComponentKind::Inductor(_))
        });

        if has_reactive {
            let rc_filter = detect_rc_filter_topology(&graph, sample_rate);

            if let Some(filter) = rc_filter {
                let tree = DynNode::VoltageSource { voltage: 0.0, rp: 1.0 };
                let root = match filter {
                    FilterTopology::RcLowpass { a1, b0, .. } => RootKind::IirLowpass { a1, b0, y_prev: 0.0, x_prev: 0.0 },
                    FilterTopology::RcHighpass { a1, b0, .. } => RootKind::IirHighpass { a1, b0, y_prev: 0.0, x_prev: 0.0 },
                    FilterTopology::RlLowpass { a1, b0, .. } => RootKind::IirLowpass { a1, b0, y_prev: 0.0, x_prev: 0.0 },
                };
                stages.push(WdfStage {
                    tree, root,
                    compensation: 1.0,
                    oversampler: Oversampler::new(oversampling),
                    base_diode_model: None, paired_opamp: None, dc_block: None,
                    is_source_follower: false, prev_source_voltage: 0.0,
                });
            } else {
                // Fall back to WDF for complex passive circuits.
                let vs_comp_idx = graph.components.len();
                let passive_edges: Vec<usize> = graph.edges.iter().enumerate()
                    .filter(|(_, e)| matches!(
                        graph.components[e.comp_idx].kind,
                        ComponentKind::Resistor(_) | ComponentKind::Capacitor(_)
                            | ComponentKind::Inductor(_) | ComponentKind::Potentiometer(_, _)
                    ))
                    .map(|(i, _)| i)
                    .collect();

                if !passive_edges.is_empty() {
                    let source_node = graph.edges.len() + 1000;
                    let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();
                    sp_edges.push((source_node, graph.in_node, SpTree::Leaf(vs_comp_idx)));
                    for &eidx in &passive_edges {
                        let e = &graph.edges[eidx];
                        sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
                    }
                    let terminals = vec![source_node, graph.gnd_node];
                    if let Ok(sp_tree) = sp_reduce(sp_edges, &terminals) {
                        let mut all_components = graph.components.clone();
                        while all_components.len() <= vs_comp_idx {
                            all_components.push(ComponentDef {
                                id: "__vs__".to_string(),
                                kind: ComponentKind::Resistor(1.0),
                            });
                        }
                        let tree = sp_to_dyn_with_vs(&sp_tree, &all_components, &graph.fork_paths, sample_rate, vs_comp_idx);
                        let compensation = compute_passive_compensation(&tree);
                        stages.push(WdfStage {
                            tree,
                            root: RootKind::ShortCircuit,
                            compensation,
                            oversampler: Oversampler::new(oversampling),
                            base_diode_model: None, paired_opamp: None, dc_block: None,
                            is_source_follower: false, prev_source_voltage: 0.0,
                        });
                    }
                }
            }
        } else {
            passive_attenuation = compute_resistor_divider_gain(&graph);
        }
    }

    // ══ Transformer gain ══════════════════════════════════════════════
    let mut transformer_gain = 1.0;
    for comp in &pedal.components {
        if let ComponentKind::Transformer(cfg) = &comp.kind {
            let pin_matches = |p: &Pin, comp_id: &str, pin_name: &str| -> bool {
                matches!(p, Pin::ComponentPin { component, pin } if component == comp_id && pin == pin_name)
            };
            let is_input_pin = |p: &Pin| -> bool { matches!(p, Pin::Reserved(name) if name == "in") };
            let is_output_pin = |p: &Pin| -> bool { matches!(p, Pin::Reserved(name) if name == "out") };

            let output_from_secondary = pedal.nets.iter().any(|net| {
                let has_secondary = pin_matches(&net.from, &comp.id, "c") || pin_matches(&net.from, &comp.id, "d");
                let has_secondary_to = net.to.iter().any(|p| pin_matches(p, &comp.id, "c") || pin_matches(p, &comp.id, "d"));
                let has_output = is_output_pin(&net.from) || net.to.iter().any(|p| is_output_pin(p));
                (has_secondary || has_secondary_to) && has_output
            });
            let input_to_primary = pedal.nets.iter().any(|net| {
                let has_primary = pin_matches(&net.from, &comp.id, "a") || pin_matches(&net.from, &comp.id, "b");
                let has_primary_to = net.to.iter().any(|p| pin_matches(p, &comp.id, "a") || pin_matches(p, &comp.id, "b"));
                let has_input = is_input_pin(&net.from) || net.to.iter().any(|p| is_input_pin(p));
                (has_primary || has_primary_to) && has_input
            });

            if input_to_primary && output_from_secondary {
                transformer_gain *= 1.0 / cfg.turns_ratio;
            }
        }
    }

    // ══ Slew rate limiters ════════════════════════════════════════════
    let mut slew_limiters = Vec::new();
    for comp in &pedal.components {
        if let ComponentKind::OpAmp(ot) = &comp.kind {
            if !ot.is_ota() {
                slew_limiters.push(SlewRateLimiter::new(ot.slew_rate(), sample_rate));
            }
        }
    }

    // ══ BBD delay lines ═══════════════════════════════════════════════
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

    // ══ Generic delay lines ═══════════════════════════════════════════
    let mut delay_lines: Vec<DelayLineBinding> = Vec::new();
    let mut delay_id_to_idx: HashMap<String, usize> = HashMap::new();

    for comp in &pedal.components {
        if let ComponentKind::DelayLine(min_delay, max_delay, interp, medium) = &comp.kind {
            let idx = delay_lines.len();
            delay_id_to_idx.insert(comp.id.clone(), idx);
            let mut dl = crate::elements::DelayLine::new(*min_delay, *max_delay, sample_rate, *interp);
            dl.set_medium(*medium);
            delay_lines.push(DelayLineBinding {
                delay_line: dl,
                taps: vec![1.0],
                comp_id: comp.id.clone(),
            });
        }
    }

    for comp in &pedal.components {
        if let ComponentKind::Tap(parent_id, ratio) = &comp.kind {
            if let Some(&dl_idx) = delay_id_to_idx.get(parent_id) {
                delay_lines[dl_idx].taps.push(*ratio);
            }
        }
    }

    for dl_binding in &mut delay_lines {
        if dl_binding.delay_line.medium() != crate::elements::Medium::None {
            let taps = dl_binding.taps.clone();
            dl_binding.delay_line.configure_zones_from_taps(&taps, None);
        }
    }

    // ══ Rail saturation model ═════════════════════════════════════════
    let rail_saturation = {
        let mut has_opamp = false;
        let mut has_bjt = false;
        let mut has_fet = false;
        let mut has_tube = false;
        let mut tube_mu = 100.0_f64;
        let mut opamp_swing = 0.85_f64;
        for comp in &pedal.components {
            match &comp.kind {
                ComponentKind::OpAmp(ot) if !ot.is_ota() => {
                    has_opamp = true;
                    opamp_swing = match ot {
                        OpAmpType::Tl072 | OpAmpType::Tl082 | OpAmpType::Generic => 0.92,
                        OpAmpType::Ne5532 => 0.90,
                        OpAmpType::Jrc4558 | OpAmpType::Rc4558 => 0.87,
                        OpAmpType::Lm308 | OpAmpType::Lm741 | OpAmpType::Op07 => 0.85,
                        _ => 0.85,
                    };
                }
                ComponentKind::Npn(_) | ComponentKind::Pnp(_) => { has_bjt = true; }
                ComponentKind::NJfet(_) | ComponentKind::PJfet(_)
                | ComponentKind::Nmos(_) | ComponentKind::Pmos(_) => { has_fet = true; }
                ComponentKind::Triode(name) => {
                    has_tube = true;
                    tube_mu = TriodeModel::try_by_name(name).map(|m| m.mu).unwrap_or(100.0);
                }
                ComponentKind::VariMu(_) => { has_tube = true; tube_mu = 35.0; }
                ComponentKind::Pentode(_) => { has_tube = true; tube_mu = 200.0; }
                _ => {}
            }
        }
        let has_source_follower = stages.iter().any(|s| s.is_source_follower);
        if has_tube {
            RailSaturation::Tube { mu: tube_mu }
        } else if has_bjt {
            let vce_sat = if classified.has_germanium { 0.3 } else { 0.2 };
            RailSaturation::Bjt { vce_sat }
        } else if has_fet && !has_source_follower {
            RailSaturation::Fet
        } else if has_opamp {
            RailSaturation::OpAmp { output_swing_ratio: opamp_swing }
        } else {
            RailSaturation::None
        }
    };

    // ══ Pass 5: Binding & assembly ════════════════════════════════════
    let lfo_ids: Vec<String> = pedal.components.iter()
        .filter_map(|c| if matches!(c.kind, ComponentKind::Lfo(..)) { Some(c.id.clone()) } else { None })
        .collect();

    let controls = super::bind::build_controls(
        pedal,
        &stages,
        &opamp_analysis.pot_map,
        &lfo_ids,
        &delay_id_to_idx,
        delay_lines.is_empty(),
    );

    let level_default = pedal.controls.iter()
        .find(|c| is_level_label(&c.label))
        .map(|c| c.default)
        .unwrap_or(1.0);

    let physical_gain = opamp_feedback_gain * passive_attenuation * transformer_gain;
    let (pre_gain, output_gain, gain_range_final) = (physical_gain, level_default, (1.0, 1.0));

    let lfos = super::bind::build_lfo_bindings(pedal, &stages, &delay_id_to_idx, sample_rate);
    let envelopes = super::bind::build_envelope_bindings(pedal, &stages, &delay_id_to_idx, sample_rate);

    // Thermal model.
    let thermal = if enable_thermal && classified.has_germanium {
        Some(ThermalModel::germanium_fuzz(sample_rate))
    } else if enable_thermal {
        Some(ThermalModel::silicon_standard(sample_rate))
    } else {
        None
    };

    // Power supply.
    let primary_supply = pedal.supplies.first().map(|s| &s.config);
    let supply_voltage = primary_supply.map_or(9.0, |s| s.voltage);

    let power_supply = primary_supply
        .filter(|s| s.has_sag())
        .map(|s| {
            crate::elements::PowerSupply::new(
                s.voltage, s.impedance.unwrap_or(0.0), s.filter_cap.unwrap_or(100e-6),
                s.rectifier, sample_rate,
            )
        });

    // Sidechain construction.
    let sidechains = super::bind::build_sidechains(pedal, &graph, sample_rate);

    let base_grid_bias = push_pull_stages.first().map_or(-2.0, |pp| pp.grid_bias);

    // Build pot smoothers.
    let pot_smoothers: Vec<SmoothedParam> = controls.iter().enumerate()
        .filter_map(|(i, ctrl)| {
            if matches!(ctrl.target, ControlTarget::PotInStage(_)) {
                Some(SmoothedParam::new(0.5, i, sample_rate))
            } else {
                None
            }
        })
        .collect();

    // ══ Assembly ══════════════════════════════════════════════════════
    let mut compiled = CompiledPedal {
        stages,
        push_pull_stages,
        pre_gain,
        output_gain,
        rail_saturation,
        sample_rate,
        controls,
        gain_range: gain_range_final,
        supply_voltage: 9.0,
        lfos,
        envelopes,
        slew_limiters,
        bbds,
        delay_lines,
        thermal,
        tolerance_seed: tolerance.seed(),
        oversampling,
        opamp_stages,
        power_supply,
        #[cfg(debug_assertions)]
        debug_stats: None,
        metrics_accumulator: None,
        metrics_buffer: None,
        input_loading: None,
        output_loading: None,
        sidechains,
        pot_smoothers,
        base_grid_bias,
    };

    let initial_voltage = match &compiled.power_supply {
        Some(psu) => psu.steady_state_voltage(),
        None => supply_voltage,
    };
    compiled.set_supply_voltage(initial_voltage);

    Ok(compiled)
}
