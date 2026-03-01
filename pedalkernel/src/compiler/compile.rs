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


// ═══════════════════════════════════════════════════════════════════════════
// Passive WDF stage builder
// ═══════════════════════════════════════════════════════════════════════════

/// Build a WDF stage for a passive-only circuit.
///
/// Uses output-rooted tree decomposition: the load element (connected from
/// out_node to gnd) is placed at the root, and the remaining elements + VS
/// form the tree body. This gives correct filter behavior via the standard
/// WDF output extraction: V_out = (a_root + b_tree) / 2.
///
/// For simple topologies (1R + 1C, or 1R + 1L):
/// - RC lowpass (R in→out, C out→gnd): tree=Series(VS,R), root=CapacitorRoot
/// - RC highpass (C in→out, R out→gnd): tree=Series(VS,C), root=ResistiveTermination
/// - RL lowpass (L in→out, R out→gnd): tree=Series(VS,L), root=ResistiveTermination
/// - RL highpass (R in→out, L out→gnd): tree=Series(VS,R), root=InductorRoot
///
/// For complex circuits: falls back to ShortCircuit with all elements in tree.
fn build_passive_wdf_stage(
    graph: &CircuitGraph,
    sample_rate: f64,
    oversampling: OversamplingFactor,
) -> Option<WdfStage> {
    let vs_comp_idx = graph.components.len();
    let passive_edges: Vec<usize> = graph.edges.iter().enumerate()
        .filter(|(_, e)| matches!(
            graph.components[e.comp_idx].kind,
            ComponentKind::Resistor(_) | ComponentKind::Capacitor(_)
                | ComponentKind::Inductor(_) | ComponentKind::Potentiometer(_, _)
        ))
        .map(|(i, _)| i)
        .collect();

    if passive_edges.is_empty() {
        return None;
    }

    // Try output-rooted decomposition for simple 2-element circuits.
    if let Some(stage) = build_output_rooted_stage(graph, &passive_edges, sample_rate, oversampling, vs_comp_idx) {
        return Some(stage);
    }

    // Fallback: full tree with ShortCircuit root (all elements in tree, gnd at root).
    let source_node = graph.edges.len() + 1000;
    let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();
    sp_edges.push((source_node, graph.in_node, SpTree::Leaf(vs_comp_idx)));
    for &eidx in &passive_edges {
        let e = &graph.edges[eidx];
        sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
    }
    let terminals = vec![source_node, graph.gnd_node];
    let sp_tree = sp_reduce(sp_edges, &terminals).ok()?;
    let mut all_components = graph.components.clone();
    while all_components.len() <= vs_comp_idx {
        all_components.push(ComponentDef {
            id: "__vs__".to_string(),
            kind: ComponentKind::Resistor(1.0),
        });
    }
    let tree = sp_to_dyn_with_vs(&sp_tree, &all_components, &graph.fork_paths, sample_rate, vs_comp_idx);
    let mut stage = WdfStage {
        tree,
        root: RootKind::ShortCircuit,
        compensation: 1.0,
        oversampler: Oversampler::new(oversampling),
        base_diode_model: None, paired_opamp: None, dc_block: None,
        is_source_follower: false, prev_source_voltage: 0.0,
    };
    stage.balance_vs_impedance();
    Some(stage)
}

/// Build an output-rooted WDF stage for simple 2-element passive circuits.
///
/// The load element (connected out→gnd) is placed at the root. The source
/// element + VS form the tree. VS impedance is set to the load's port
/// resistance to maintain correct circuit time constants.
fn build_output_rooted_stage(
    graph: &CircuitGraph,
    passive_edges: &[usize],
    sample_rate: f64,
    oversampling: OversamplingFactor,
    vs_comp_idx: usize,
) -> Option<WdfStage> {
    if passive_edges.len() != 2 {
        return None;
    }

    // Find the load edge (connects out_node to gnd_node) and source edge.
    let mut load_idx = None;
    let mut source_idx = None;
    for &eidx in passive_edges {
        let e = &graph.edges[eidx];
        let connects_out_gnd = (e.node_a == graph.out_node && e.node_b == graph.gnd_node)
            || (e.node_a == graph.gnd_node && e.node_b == graph.out_node);
        if connects_out_gnd {
            load_idx = Some(eidx);
        } else {
            source_idx = Some(eidx);
        }
    }
    let load_eidx = load_idx?;
    let source_eidx = source_idx?;

    // Verify source edge connects in→out (through the circuit).
    let source_edge = &graph.edges[source_eidx];
    let connects_in_out = (source_edge.node_a == graph.in_node && source_edge.node_b == graph.out_node)
        || (source_edge.node_a == graph.out_node && source_edge.node_b == graph.in_node);
    if !connects_in_out {
        return None;
    }

    let load_comp = &graph.components[graph.edges[load_eidx].comp_idx];
    let source_comp = &graph.components[source_edge.comp_idx];

    // Determine root kind based on load element type.
    let (root, load_rp) = match &load_comp.kind {
        ComponentKind::Resistor(r) => (RootKind::ResistiveTermination, *r),
        ComponentKind::Capacitor(cfg) => {
            let rp = 1.0 / (2.0 * sample_rate * cfg.value);
            (
                RootKind::CapacitorRoot {
                    capacitance: cfg.value,
                    rp,
                    state: 0.0,
                },
                rp,
            )
        }
        ComponentKind::Inductor(l) => {
            let rp = 2.0 * sample_rate * *l;
            (
                RootKind::InductorRoot {
                    inductance: *l,
                    rp,
                    state: 0.0,
                },
                rp,
            )
        }
        _ => return None,
    };

    // Build tree: Series(VS, source_element), with VS rp set to load_rp.
    // This ensures the tree's port resistance ≈ 2 * load_rp, giving correct
    // scattering and time constants.
    let source_node = graph.edges.len() + 1000;
    let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();
    sp_edges.push((source_node, graph.in_node, SpTree::Leaf(vs_comp_idx)));
    sp_edges.push((source_edge.node_a, source_edge.node_b, SpTree::Leaf(source_edge.comp_idx)));
    let terminals = vec![source_node, graph.out_node];
    let sp_tree = sp_reduce(sp_edges, &terminals).ok()?;

    // Build component list with VS having load-matched impedance.
    let mut all_components = graph.components.clone();
    while all_components.len() <= vs_comp_idx {
        all_components.push(ComponentDef {
            id: "__vs__".to_string(),
            kind: ComponentKind::Resistor(load_rp.max(1.0)),
        });
    }

    let mut tree = sp_to_dyn_with_vs(&sp_tree, &all_components, &graph.fork_paths, sample_rate, vs_comp_idx);

    // Set VS impedance to half the load impedance for correct time constants.
    // In WDF, the root port termination implicitly contributes impedance equal
    // to the tree's port resistance. Total effective R = VS_rp + tree_rp ≈ 2*VS_rp.
    // Setting VS_rp = load_rp/2 gives total ≈ load_rp, matching the actual circuit.
    let vs_rp = (load_rp / 2.0).max(1.0);
    set_vs_impedance(&mut tree, vs_rp);
    tree.recompute();

    // Verify source element is in the tree (not just VS).
    let has_source = match &source_comp.kind {
        ComponentKind::Capacitor(_) | ComponentKind::Inductor(_) | ComponentKind::Resistor(_) => true,
        _ => false,
    };
    if !has_source {
        return None;
    }

    Some(WdfStage {
        tree,
        root,
        compensation: 1.0,
        oversampler: Oversampler::new(oversampling),
        base_diode_model: None, paired_opamp: None, dc_block: None,
        is_source_follower: false, prev_source_voltage: 0.0,
    })
}

/// Set the voltage source impedance within a tree.
fn set_vs_impedance(node: &mut DynNode, target_rp: f64) {
    match node {
        DynNode::VoltageSource { rp, .. } => *rp = target_rp,
        DynNode::Series { left, right, .. } | DynNode::Parallel { left, right, .. } => {
            set_vs_impedance(left, target_rp);
            set_vs_impedance(right, target_rp);
        }
        _ => {}
    }
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
            if let Some(stage) = build_passive_wdf_stage(&graph, sample_rate, oversampling) {
                stages.push(stage);
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
        pot_mirrors: {
            // Build reverse mapping: source_id → [mirrored_ids]
            let mut m: std::collections::HashMap<String, Vec<String>> = std::collections::HashMap::new();
            for (mirrored, source) in &pedal.mirrors {
                m.entry(source.clone()).or_default().push(mirrored.clone());
            }
            m
        },
        base_grid_bias,
    };

    let initial_voltage = match &compiled.power_supply {
        Some(psu) => psu.steady_state_voltage(),
        None => supply_voltage,
    };
    compiled.set_supply_voltage(initial_voltage);

    Ok(compiled)
}
