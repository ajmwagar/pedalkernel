//! Main compiler entry point: PedalDef -> CompiledPedal.

use std::collections::{HashMap, HashSet};

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
// Main compiler entry point
// ═══════════════════════════════════════════════════════════════════════════

/// Compile a parsed `.pedal` file into a real-time audio processor.
///
/// The compiler analyzes the netlist topology and builds one or more WDF
/// clipping stages, each with its own binary tree derived from the circuit's
/// passive elements and a nonlinear diode root.
///
/// Active elements (transistors, opamps) are modeled as gain stages.
/// Controls are automatically bound to gain/level/pot parameters.
/// Options for pedal compilation.
///
/// Controls optional features like oversampling, component tolerance,
/// and thermal modeling.
pub struct CompileOptions {
    /// Oversampling factor for nonlinear stages (default: X1 = off).
    pub oversampling: OversamplingFactor,
    /// Component tolerance engine (default: ideal = no variation).
    pub tolerance: ToleranceEngine,
    /// Whether to enable thermal drift modeling.
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

/// Compile a pedal definition with default options (no oversampling, no tolerance).
pub fn compile_pedal(pedal: &PedalDef, sample_rate: f64) -> Result<CompiledPedal, String> {
    compile_pedal_with_options(pedal, sample_rate, CompileOptions::default())
}

/// Compile a pedal definition with custom options.
pub fn compile_pedal_with_options(
    pedal: &PedalDef,
    sample_rate: f64,
    options: CompileOptions,
) -> Result<CompiledPedal, String> {
    let oversampling = options.oversampling;
    let tolerance = options.tolerance;
    let enable_thermal = options.thermal;

    let mut graph = CircuitGraph::from_pedal(pedal);

    // Apply component tolerance to resistors and capacitors.
    // This deterministically shifts component values from nominal based on the
    // tolerance seed, so two instances of the same pedal model will sound
    // slightly different (like real hardware).
    for (i, comp) in graph.components.iter_mut().enumerate() {
        match &mut comp.kind {
            ComponentKind::Resistor(r) => {
                *r = tolerance.apply_resistor(*r, i);
            }
            ComponentKind::Capacitor(cfg) => {
                cfg.value = tolerance.apply_capacitor(cfg.value, i);
            }
            ComponentKind::Potentiometer(max_r) => {
                *max_r = tolerance.apply_resistor(*max_r, i);
            }
            _ => {}
        }
    }

    let diodes = graph.find_diodes();
    let diode_edge_indices: Vec<usize> = diodes.iter().map(|(idx, _)| *idx).collect();

    // Determine gain range based on circuit characteristics.
    //
    // The gain taxonomy considers:
    //   - Diode type (germanium clips softer → needs more drive)
    //   - Single vs pair diodes (single = asymmetric = fuzz-like)
    //   - Number of clipping stages (multi-stage = less gain per stage)
    //   - Active device count and type (each contributes specific gain)
    let has_germanium = diodes
        .iter()
        .any(|(_, d)| d.diode_type == DiodeType::Germanium);
    let has_single_diode = diodes.iter().any(|(_, d)| !d.is_pair);
    let has_led = diodes
        .iter()
        .any(|(_, d)| d.diode_type == DiodeType::Led);
    // Check what types of active elements the circuit has.
    let has_bjts = pedal
        .components
        .iter()
        .any(|c| matches!(&c.kind, ComponentKind::Npn(_) | ComponentKind::Pnp(_)));
    let has_tubes = pedal
        .components
        .iter()
        .any(|c| matches!(&c.kind, ComponentKind::Triode(_) | ComponentKind::Pentode(_)));

    let gain_range = if diodes.is_empty() && !has_bjts && !has_tubes {
        // Effects circuits (phasers, chorus, compressors) using only JFETs
        // and/or op-amps — no clipping or gain stages.  JFETs act as variable
        // resistors, op-amps as buffers.  Signal should pass at unity gain.
        // NOTE: output is currently attenuated because op-amp feedback loops
        // aren't modeled inside the WDF tree.  The fix is integrating op-amp
        // feedback into the tree, not inflating the gain range.
        (1.0, 1.0)
    } else if diodes.is_empty() {
        // Active gain stages (BJTs, triodes, pentodes) without diode clipping.
        // The active elements clip through device saturation. They need drive,
        // but less than diode circuits since the active element IS the clipper.
        (2.0, 30.0)
    } else if has_germanium && has_single_diode {
        (5.0, 250.0) // Fuzz-like: single germanium diode needs high drive
    } else if has_germanium {
        (3.0, 150.0) // Germanium overdrive (Klon-ish)
    } else if has_led {
        (4.0, 120.0) // LED clipping: higher Vf → needs more drive, but cleaner
    } else if diodes.len() > 2 {
        (1.5, 40.0) // Many stages: each clips a little, cumulative is heavy
    } else if diodes.len() > 1 {
        (2.0, 60.0) // Two stages (e.g., Big Muff topology)
    } else {
        (2.0, 100.0) // Standard single-stage overdrive
    };

    // Active elements contribute gain based on their type.
    //
    // IMPORTANT: Use the maximum single-element bonus, NOT the product.
    // Each active element already has its own processing stage:
    //   - Op-amps → OpAmpStage (separate processing loop)
    //   - JFETs → WDF stages with JfetRoot
    //   - BJTs → WDF stages with BjtRoot
    //   - Triodes → WDF stages with TriodeRoot
    //   - Pentodes → WDF stages with PentodeRoot
    //
    // The active_bonus only models the gain that drives signal INTO diode
    // clipping stages. Using a product (bonus *= N per element) causes
    // exponential gain growth: 4 op-amps → 2^4 = 16x, 4 JFETs → 1.5^4 ≈ 5x,
    // combined = 81x — catastrophically wrong for unity-gain circuits like
    // phasers, buffers, and all-pass filters.
    //
    // When there are no diode stages, active_bonus = 1.0 because the active
    // elements' own WDF/opamp stages handle gain correctly.
    let active_bonus = if diodes.is_empty() {
        // No diode clipping: all active elements have their own stages.
        1.0
    } else {
        // Find the strongest active gain element that drives a clipping stage.
        // Use max (not product) since pre_gain is applied once at the input
        // (and again only between consecutive clipping stages).
        let mut max_bonus = 1.0_f64;
        for comp in &pedal.components {
            let bonus = match &comp.kind {
                ComponentKind::OpAmp(ot) if !ot.is_ota() => {
                    match ot {
                        OpAmpType::Ne5532 => 3.5,
                        OpAmpType::Tl072 | OpAmpType::Tl082 | OpAmpType::Generic => 3.0,
                        OpAmpType::Jrc4558 | OpAmpType::Rc4558 => 2.5,
                        OpAmpType::Lm741 | OpAmpType::Op07 => 2.0,
                        OpAmpType::Lm308 => 1.8,
                        _ => 2.5,
                    }
                }
                ComponentKind::Npn(_) | ComponentKind::Pnp(_) => 2.5,
                ComponentKind::NJfet(_) | ComponentKind::PJfet(_) => 1.5,
                ComponentKind::Triode(_) => 2.0,
                ComponentKind::Pentode(_) => 3.0,
                _ => 1.0,
            };
            max_bonus = max_bonus.max(bonus);
        }
        max_bonus
    };

    // Build WDF stages.
    let mut stages = Vec::new();
    let vs_comp_idx = graph.components.len(); // virtual voltage source index

    // BFS distances from in_node for Vs placement.
    let dist_from_in = {
        let mut adj: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
        for e in &graph.edges {
            adj.entry(e.node_a).or_default().push(e.node_b);
            adj.entry(e.node_b).or_default().push(e.node_a);
        }
        let mut dist: HashMap<NodeId, usize> = HashMap::new();
        let mut queue = std::collections::VecDeque::new();
        dist.insert(graph.in_node, 0);
        queue.push_back(graph.in_node);
        while let Some(n) = queue.pop_front() {
            let d = dist[&n];
            if let Some(neighbors) = adj.get(&n) {
                for &nb in neighbors {
                    dist.entry(nb).or_insert_with(|| {
                        queue.push_back(nb);
                        d + 1
                    });
                }
            }
        }
        dist
    };

    for (_edge_idx, diode_info) in &diodes {
        let junction = diode_info.junction_node;
        let passive_idxs =
            graph.elements_at_junction(junction, &diode_edge_indices, &graph.active_edge_indices);

        if passive_idxs.is_empty() {
            continue;
        }

        // Find the best injection node for the voltage source: the non-junction
        // endpoint closest to in_node.  This ensures the Vs is reachable from the
        // passive elements in this stage (critical for multi-stage circuits).
        let mut injection_node = graph.in_node;
        let mut best_dist = usize::MAX;
        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            let other = if e.node_a == junction {
                e.node_b
            } else {
                e.node_a
            };
            if let Some(&d) = dist_from_in.get(&other) {
                if d < best_dist {
                    best_dist = d;
                    injection_node = other;
                }
            }
        }
        // Fallback: if in_node is disconnected (e.g. split pedal post-half
        // where in maps to a triode grid), use gnd as the injection point.
        if best_dist == usize::MAX {
            injection_node = graph.gnd_node;
        }

        // Build SP edges: one edge per passive element, plus a virtual VoltageSource.
        let source_node = graph.edges.len() + 1000; // virtual source node
        let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();

        // Add voltage source edge: source_node → injection_node.
        sp_edges.push((source_node, injection_node, SpTree::Leaf(vs_comp_idx)));

        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
        }

        let terminals = vec![source_node, junction];
        let sp_tree = match sp_reduce(sp_edges, &terminals) {
            Ok(t) => t,
            Err(_) => continue,
        };

        // Convert SP tree to dynamic WDF nodes.
        // We need to handle the virtual VoltageSource leaf specially.
        let mut all_components = graph.components.clone();
        // Add a virtual VoltageSource component at vs_comp_idx.
        while all_components.len() <= vs_comp_idx {
            all_components.push(ComponentDef {
                id: "__vs__".to_string(),
                kind: ComponentKind::Resistor(1.0), // placeholder
            });
        }

        let tree = sp_to_dyn_with_vs(&sp_tree, &all_components, &graph.fork_paths, sample_rate, vs_comp_idx);

        let model = diode_model(diode_info.diode_type);
        let root = if diode_info.is_pair {
            RootKind::DiodePair(DiodePairRoot::new(model))
        } else {
            RootKind::SingleDiode(DiodeRoot::new(model))
        };

        stages.push(WdfStage {
            tree,
            root,
            compensation: 1.0,
            oversampler: Oversampler::new(oversampling),
            base_diode_model: Some(model),
            paired_opamp: None,
        });
    }

    // ── Op-amp feedback detection ───────────────────────────────────────────
    // Detect op-amp feedback topologies and calculate closed-loop gain.
    // - Unity-gain: neg=out, gain = 1.0 (paired with JFET stages for all-pass)
    // - Inverting: gain = Rf/Ri (creates dedicated op-amp WDF stage)
    // - Non-inverting: gain = 1 + Rf/Ri (creates dedicated op-amp WDF stage)
    let feedback_loops = graph.find_opamp_feedback_loops(pedal);
    let feedback_opamp_ids: HashSet<String> = feedback_loops
        .iter()
        .map(|info| info.comp_id.clone())
        .collect();

    // Track which op-amps are unity-gain (for JFET pairing) vs gain stages
    let unity_gain_opamp_ids: HashSet<String> = feedback_loops
        .iter()
        .filter(|info| matches!(info.feedback_kind, OpAmpFeedbackKind::UnityGain))
        .map(|info| info.comp_id.clone())
        .collect();

    // No longer need opamp_feedback_gain - Phase 2 uses WDF roots directly
    let opamp_feedback_gain = 1.0_f64;

    // ── Op-amp gain stages (Inverting/Non-Inverting) ───────────────────────
    // Create WDF stages for op-amps with detected gain configurations.
    // These stages apply the closed-loop gain through the WDF scattering.
    //
    // Also build a map from pot component IDs to their gain modulation info:
    // (stage_idx, ri, fixed_series_r, max_pot_r, parallel_fixed_r, is_inverting)
    let mut opamp_pot_map: HashMap<String, (usize, f64, f64, f64, Option<f64>, bool)> = HashMap::new();

    for info in &feedback_loops {
        match &info.feedback_kind {
            OpAmpFeedbackKind::UnityGain => {
                // Unity-gain op-amps are paired with JFET stages for all-pass filters
                // Skip here - handled in JFET stage creation below
            }
            OpAmpFeedbackKind::Inverting { rf, ri, feedback_diode, rf_pot } => {
                // Track pot info for runtime gain modulation BEFORE creating stage
                // (so we have the correct stage index)
                let stage_idx = stages.len();
                if let Some((pot_id, max_pot_r, fixed_series_r, parallel_fixed_r)) = rf_pot {
                    opamp_pot_map.insert(pot_id.clone(), (stage_idx, *ri, *fixed_series_r, *max_pot_r, *parallel_fixed_r, true));
                }

                // Create an inverting op-amp WDF stage
                let model = OpAmpModel::from_opamp_type(&info.opamp_type);
                let gain = rf / ri;
                let mut root = OpAmpInvertingRoot::new(model, gain);
                root.set_sample_rate(sample_rate);

                // Set v_max based on supply voltage (single-supply biased at Vcc/2)
                // Use default 9V supply, will be updated when supply_voltage is known
                let default_supply = 9.0_f64;
                let v_max = (default_supply / 2.0 - 1.5).max(0.5);
                root.set_v_max(v_max);

                // Enable soft clipping if feedback diodes are present
                // This models Tube Screamer style soft clipping via feedback diodes
                if let Some(diode_type) = feedback_diode {
                    let diode_vf = match diode_type {
                        DiodeType::Silicon => 0.6, // 1N4148, 1N914
                        DiodeType::Germanium => 0.3, // 1N34A, OA91
                        DiodeType::Led => 1.6, // Red LED
                    };
                    root.set_soft_clip(diode_vf);
                }

                // Create a tree with voltage source for signal injection.
                // The voltage source models the input signal, and the resistor
                // represents the load impedance. The op-amp root drives the output.
                let tree = DynNode::VoltageSource { voltage: 0.0, rp: 10_000.0 };

                stages.push(WdfStage {
                    tree,
                    root: RootKind::OpAmpInverting(root),
                    compensation: 1.0,
                    oversampler: Oversampler::new(oversampling),
                    base_diode_model: None,
                    paired_opamp: None,
                });
            }
            OpAmpFeedbackKind::NonInverting { rf, ri, rf_pot } => {
                // Track pot info for runtime gain modulation BEFORE creating stage
                let stage_idx = stages.len();
                if let Some((pot_id, max_pot_r, fixed_series_r, parallel_fixed_r)) = rf_pot {
                    opamp_pot_map.insert(pot_id.clone(), (stage_idx, *ri, *fixed_series_r, *max_pot_r, *parallel_fixed_r, false));
                }

                // Create a non-inverting op-amp WDF stage
                let model = OpAmpModel::from_opamp_type(&info.opamp_type);
                let gain = 1.0 + (rf / ri);
                let mut root = OpAmpNonInvertingRoot::new(model, gain);
                root.set_sample_rate(sample_rate);

                // Set v_max based on supply voltage (single-supply biased at Vcc/2)
                // Use default 9V supply, will be updated when supply_voltage is known
                let default_supply = 9.0_f64;
                let v_max = (default_supply / 2.0 - 1.5).max(0.5);
                root.set_v_max(v_max);

                // Create a tree with voltage source for signal injection.
                // For non-inverting, the input goes to Vp (set in stage.process),
                // but we still need a voltage source for the WDF to work.
                let tree = DynNode::VoltageSource { voltage: 0.0, rp: 10_000.0 };

                stages.push(WdfStage {
                    tree,
                    root: RootKind::OpAmpNonInverting(root),
                    compensation: 1.0,
                    oversampler: Oversampler::new(oversampling),
                    base_diode_model: None,
                    paired_opamp: None,
                });
            }
        }
    }

    // ── Op-amp stages (standalone, no feedback) ────────────────────────────
    // Create OpAmpRoot elements for each non-OTA op-amp in the circuit
    // that doesn't have detected feedback loops.
    let mut opamp_stages: Vec<OpAmpStage> = Vec::new();
    for comp in &pedal.components {
        if let ComponentKind::OpAmp(ot) = &comp.kind {
            // OTAs (CA3080) are handled separately as OtaRoot, not OpAmpRoot
            if !ot.is_ota() && !feedback_opamp_ids.contains(&comp.id) {
                let model = OpAmpModel::from_opamp_type(ot);
                let mut opamp = OpAmpRoot::new(model);
                opamp.set_sample_rate(sample_rate);
                opamp_stages.push(OpAmpStage {
                    opamp,
                    comp_id: comp.id.clone(),
                });
            }
        }
    }

    // Build a queue of unity-gain feedback op-amps to pair with JFET stages.
    // Each feedback op-amp will be attached to the next JFET stage
    // in the signal chain (ordered by topological distance from input).
    // Only unity-gain op-amps are paired with JFET stages (for all-pass filters).
    let mut feedback_opamp_queue: Vec<OpAmpRoot> = feedback_loops
        .iter()
        .filter(|info| matches!(info.feedback_kind, OpAmpFeedbackKind::UnityGain))
        .map(|info| {
            let model = OpAmpModel::from_opamp_type(&info.opamp_type);
            let mut opamp = OpAmpRoot::new(model);
            opamp.set_sample_rate(sample_rate);
            opamp
        })
        .collect();

    // Build WDF stages for JFETs.
    let jfets = graph.find_jfets();
    let jfet_edge_indices: Vec<usize> = jfets.iter().map(|(idx, _)| *idx).collect();
    let all_nonlinear_indices: Vec<usize> = diode_edge_indices
        .iter()
        .chain(jfet_edge_indices.iter())
        .copied()
        .collect();

    for (_edge_idx, jfet_info) in &jfets {
        let junction = jfet_info.junction_node;
        let passive_idxs = graph.elements_at_junction(
            junction,
            &all_nonlinear_indices,
            &graph.active_edge_indices,
        );

        if passive_idxs.is_empty() {
            continue;
        }

        // Find the best injection node for the voltage source.
        let mut injection_node = graph.in_node;
        let mut best_dist = usize::MAX;
        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            let other = if e.node_a == junction {
                e.node_b
            } else {
                e.node_a
            };
            if let Some(&d) = dist_from_in.get(&other) {
                if d < best_dist {
                    best_dist = d;
                    injection_node = other;
                }
            }
        }
        // Fallback: if in_node is disconnected (e.g. split pedal post-half
        // where in maps to a triode grid), use gnd as the injection point.
        if best_dist == usize::MAX {
            injection_node = graph.gnd_node;
        }

        // Build SP edges.
        let source_node = graph.edges.len() + 2000; // virtual source node (different from diodes)
        let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();
        sp_edges.push((source_node, injection_node, SpTree::Leaf(vs_comp_idx)));

        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
        }

        let terminals = vec![source_node, junction];
        let sp_tree = match sp_reduce(sp_edges, &terminals) {
            Ok(t) => t,
            Err(_) => continue,
        };

        // Create components list with virtual voltage source for JFET stages.
        let mut jfet_components = graph.components.clone();
        while jfet_components.len() <= vs_comp_idx {
            jfet_components.push(ComponentDef {
                id: "__vs__".to_string(),
                kind: ComponentKind::Resistor(1.0),
            });
        }

        let tree = sp_to_dyn_with_vs(&sp_tree, &jfet_components, &graph.fork_paths, sample_rate, vs_comp_idx);

        let model = jfet_model(jfet_info.jfet_type, jfet_info.is_n_channel);
        let root = RootKind::Jfet(JfetRoot::new(model));

        // Pair this JFET stage with a feedback op-amp if available.
        // In all-pass circuits (Phase 90), each JFET stage is followed
        // by a unity-gain op-amp buffer that maintains signal level.
        let paired_opamp = if !feedback_opamp_queue.is_empty() {
            Some(feedback_opamp_queue.remove(0))
        } else {
            None
        };

        stages.push(WdfStage {
            tree,
            root,
            compensation: 1.0,
            oversampler: Oversampler::new(oversampling),
            base_diode_model: None,
            paired_opamp,
        });
    }

    // Build WDF stages for BJTs.
    let bjts = graph.find_bjts();
    let bjt_edge_indices: Vec<usize> = bjts.iter().map(|(idx, _)| *idx).collect();
    let all_nonlinear_with_bjts: Vec<usize> = all_nonlinear_indices
        .iter()
        .chain(bjt_edge_indices.iter())
        .copied()
        .collect();

    for (_edge_idx, bjt_info) in &bjts {
        let junction = bjt_info.junction_node;
        let passive_idxs = graph.elements_at_junction(
            junction,
            &all_nonlinear_with_bjts,
            &graph.active_edge_indices,
        );

        if passive_idxs.is_empty() {
            continue;
        }

        // Find the best injection node for the voltage source.
        let mut injection_node = graph.in_node;
        let mut best_dist = usize::MAX;
        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            let other = if e.node_a == junction {
                e.node_b
            } else {
                e.node_a
            };
            if let Some(&d) = dist_from_in.get(&other) {
                if d < best_dist {
                    best_dist = d;
                    injection_node = other;
                }
            }
        }
        if best_dist == usize::MAX {
            injection_node = graph.gnd_node;
        }

        // Build SP edges.
        let source_node = graph.edges.len() + 3000; // virtual source node for BJTs
        let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();
        sp_edges.push((source_node, injection_node, SpTree::Leaf(vs_comp_idx)));

        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
        }

        let terminals = vec![source_node, junction];
        let sp_tree = match sp_reduce(sp_edges, &terminals) {
            Ok(t) => t,
            Err(_) => continue,
        };

        // Create components list with virtual voltage source for BJT stages.
        let mut bjt_components = graph.components.clone();
        while bjt_components.len() <= vs_comp_idx {
            bjt_components.push(ComponentDef {
                id: "__vs__".to_string(),
                kind: ComponentKind::Resistor(1.0),
            });
        }

        let tree = sp_to_dyn_with_vs(&sp_tree, &bjt_components, &graph.fork_paths, sample_rate, vs_comp_idx);

        let model = BjtModel::from_bjt_type(&bjt_info.bjt_type);
        let root = if bjt_info.is_npn {
            RootKind::BjtNpn(BjtNpnRoot::new(model))
        } else {
            RootKind::BjtPnp(BjtPnpRoot::new(model))
        };

        stages.push(WdfStage {
            tree,
            root,
            compensation: 1.0,
            oversampler: Oversampler::new(oversampling),
            base_diode_model: None,
            paired_opamp: None,
        });
    }

    // Build WDF stages for triodes.
    let triodes = graph.find_triodes();
    let triode_edge_indices: Vec<usize> = triodes.iter().map(|(idx, _)| *idx).collect();
    let all_nonlinear_with_triodes: Vec<usize> = all_nonlinear_with_bjts
        .iter()
        .chain(triode_edge_indices.iter())
        .copied()
        .collect();

    for (_edge_idx, triode_info) in &triodes {
        let junction = triode_info.junction_node;
        let passive_idxs = graph.elements_at_junction(
            junction,
            &all_nonlinear_with_triodes,
            &graph.active_edge_indices,
        );

        if passive_idxs.is_empty() {
            continue;
        }

        // Find the best injection node for the voltage source.
        let mut injection_node = graph.in_node;
        let mut best_dist = usize::MAX;
        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            let other = if e.node_a == junction {
                e.node_b
            } else {
                e.node_a
            };
            if let Some(&d) = dist_from_in.get(&other) {
                if d < best_dist {
                    best_dist = d;
                    injection_node = other;
                }
            }
        }
        // Fallback: if in_node is disconnected (e.g. split pedal post-half
        // where in maps to a triode grid), use gnd as the injection point.
        if best_dist == usize::MAX {
            injection_node = graph.gnd_node;
        }

        // Build SP edges.
        let source_node = graph.edges.len() + 3000; // virtual source node (unique offset)
        let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();
        sp_edges.push((source_node, injection_node, SpTree::Leaf(vs_comp_idx)));

        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
        }

        let terminals = vec![source_node, junction];
        let sp_tree = match sp_reduce(sp_edges, &terminals) {
            Ok(t) => t,
            Err(_) => continue,
        };

        let mut triode_components = graph.components.clone();
        while triode_components.len() <= vs_comp_idx {
            triode_components.push(ComponentDef {
                id: "__vs__".to_string(),
                kind: ComponentKind::Resistor(1.0),
            });
        }

        let tree = sp_to_dyn_with_vs(&sp_tree, &triode_components, &graph.fork_paths, sample_rate, vs_comp_idx);

        let model = triode_model(triode_info.triode_type);
        let root = RootKind::Triode(TriodeRoot::new(model));

        // Scale compensation based on triode mu (voltage gain ≈ mu × R_load/(R_load+r_p))
        // Normalize to 12AX7 (mu=100) as reference. Lower-mu tubes have proportionally
        // lower voltage gain, which is a defining characteristic of each tube type.
        let mu_compensation = model.mu / 100.0;

        stages.push(WdfStage {
            tree,
            root,
            compensation: mu_compensation,
            oversampler: Oversampler::new(oversampling),
            base_diode_model: None,
            paired_opamp: None,
        });
    }

    // Build WDF stages for pentodes.
    let pentodes = graph.find_pentodes();
    let pentode_edge_indices: Vec<usize> = pentodes.iter().map(|(idx, _)| *idx).collect();
    let all_nonlinear_with_pentodes: Vec<usize> = all_nonlinear_with_triodes
        .iter()
        .chain(pentode_edge_indices.iter())
        .copied()
        .collect();

    for (_edge_idx, pentode_info) in &pentodes {
        let junction = pentode_info.junction_node;
        let passive_idxs = graph.elements_at_junction(
            junction,
            &all_nonlinear_with_pentodes,
            &graph.active_edge_indices,
        );

        if passive_idxs.is_empty() {
            continue;
        }

        // Find the best injection node for the voltage source.
        let mut injection_node = graph.in_node;
        let mut best_dist = usize::MAX;
        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            let other = if e.node_a == junction {
                e.node_b
            } else {
                e.node_a
            };
            if let Some(&d) = dist_from_in.get(&other) {
                if d < best_dist {
                    best_dist = d;
                    injection_node = other;
                }
            }
        }
        if best_dist == usize::MAX {
            injection_node = graph.gnd_node;
        }

        // Build SP edges.
        let source_node = graph.edges.len() + 4000; // unique offset for pentodes
        let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();
        sp_edges.push((source_node, injection_node, SpTree::Leaf(vs_comp_idx)));

        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
        }

        let terminals = vec![source_node, junction];
        let sp_tree = match sp_reduce(sp_edges, &terminals) {
            Ok(t) => t,
            Err(_) => continue,
        };

        let mut pentode_components = graph.components.clone();
        while pentode_components.len() <= vs_comp_idx {
            pentode_components.push(ComponentDef {
                id: "__vs__".to_string(),
                kind: ComponentKind::Resistor(1.0),
            });
        }

        let tree = sp_to_dyn_with_vs(&sp_tree, &pentode_components, &graph.fork_paths, sample_rate, vs_comp_idx);

        let model = pentode_model(pentode_info.pentode_type);
        let root = RootKind::Pentode(PentodeRoot::new(model));

        // Scale compensation based on pentode mu. Pentodes have higher mu than triodes
        // but lower effective gain due to screen grid shielding. Normalize to mu=200
        // as a reference (typical for power pentodes like EL34, 6L6).
        let mu_compensation = model.mu / 200.0;

        stages.push(WdfStage {
            tree,
            root,
            compensation: mu_compensation,
            oversampler: Oversampler::new(oversampling),
            base_diode_model: None,
            paired_opamp: None,
        });
    }

    // Build WDF stages for MOSFETs.
    let mosfets = graph.find_mosfets();
    let mosfet_edge_indices: Vec<usize> = mosfets.iter().map(|(idx, _)| *idx).collect();
    let all_nonlinear_with_mosfets: Vec<usize> = all_nonlinear_with_pentodes
        .iter()
        .chain(mosfet_edge_indices.iter())
        .copied()
        .collect();

    for (_edge_idx, mosfet_info) in &mosfets {
        let junction = mosfet_info.junction_node;
        let passive_idxs = graph.elements_at_junction(
            junction,
            &all_nonlinear_with_mosfets,
            &graph.active_edge_indices,
        );

        if passive_idxs.is_empty() {
            continue;
        }

        let mut injection_node = graph.in_node;
        let mut best_dist = usize::MAX;
        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            let other = if e.node_a == junction {
                e.node_b
            } else {
                e.node_a
            };
            if let Some(&d) = dist_from_in.get(&other) {
                if d < best_dist {
                    best_dist = d;
                    injection_node = other;
                }
            }
        }
        if best_dist == usize::MAX {
            injection_node = graph.gnd_node;
        }

        let source_node = graph.edges.len() + 4000;
        let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();
        sp_edges.push((source_node, injection_node, SpTree::Leaf(vs_comp_idx)));

        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
        }

        let terminals = vec![source_node, junction];
        let sp_tree = match sp_reduce(sp_edges, &terminals) {
            Ok(t) => t,
            Err(_) => continue,
        };

        let mut mosfet_components = graph.components.clone();
        while mosfet_components.len() <= vs_comp_idx {
            mosfet_components.push(ComponentDef {
                id: "__vs__".to_string(),
                kind: ComponentKind::Resistor(1.0),
            });
        }

        let tree = sp_to_dyn_with_vs(&sp_tree, &mosfet_components, &graph.fork_paths, sample_rate, vs_comp_idx);

        let model = mosfet_model(mosfet_info.mosfet_type, mosfet_info.is_n_channel);
        let root = RootKind::Mosfet(MosfetRoot::new(model));

        stages.push(WdfStage {
            tree,
            root,
            compensation: 1.0,
            oversampler: Oversampler::new(oversampling),
            base_diode_model: None,
            paired_opamp: None,
        });
    }

    // Build WDF stages for Zener diodes.
    let zeners = graph.find_zeners();
    let zener_edge_indices: Vec<usize> = zeners.iter().map(|(idx, _)| *idx).collect();
    let all_nonlinear_final: Vec<usize> = all_nonlinear_with_mosfets
        .iter()
        .chain(zener_edge_indices.iter())
        .copied()
        .collect();

    for (_edge_idx, zener_info) in &zeners {
        let junction = zener_info.junction_node;
        let passive_idxs = graph.elements_at_junction(
            junction,
            &all_nonlinear_final,
            &graph.active_edge_indices,
        );

        if passive_idxs.is_empty() {
            continue;
        }

        let mut injection_node = graph.in_node;
        let mut best_dist = usize::MAX;
        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            let other = if e.node_a == junction {
                e.node_b
            } else {
                e.node_a
            };
            if let Some(&d) = dist_from_in.get(&other) {
                if d < best_dist {
                    best_dist = d;
                    injection_node = other;
                }
            }
        }
        if best_dist == usize::MAX {
            injection_node = graph.gnd_node;
        }

        let source_node = graph.edges.len() + 5000;
        let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();
        sp_edges.push((source_node, injection_node, SpTree::Leaf(vs_comp_idx)));

        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
        }

        let terminals = vec![source_node, junction];
        let sp_tree = match sp_reduce(sp_edges, &terminals) {
            Ok(t) => t,
            Err(_) => continue,
        };

        let mut zener_components = graph.components.clone();
        while zener_components.len() <= vs_comp_idx {
            zener_components.push(ComponentDef {
                id: "__vs__".to_string(),
                kind: ComponentKind::Resistor(1.0),
            });
        }

        let tree = sp_to_dyn_with_vs(&sp_tree, &zener_components, &graph.fork_paths, sample_rate, vs_comp_idx);

        let model = ZenerModel::new(zener_info.voltage);
        let root = RootKind::Zener(ZenerRoot::new(model));

        stages.push(WdfStage {
            tree,
            root,
            compensation: 1.0,
            oversampler: Oversampler::new(oversampling),
            base_diode_model: None,
            paired_opamp: None,
        });
    }

    // ── OTA (CA3080) stages ──────────────────────────────────────────────
    let otas = graph.find_otas();
    let ota_edge_indices: Vec<usize> = otas.iter().map(|(idx, _)| *idx).collect();
    let all_nonlinear_with_otas: Vec<usize> = all_nonlinear_final
        .iter()
        .chain(ota_edge_indices.iter())
        .copied()
        .collect();

    for (_edge_idx, ota_info) in &otas {
        let junction = ota_info.junction_node;
        let passive_idxs = graph.elements_at_junction(
            junction,
            &all_nonlinear_with_otas,
            &graph.active_edge_indices,
        );

        if passive_idxs.is_empty() {
            continue;
        }

        let mut injection_node = graph.in_node;
        let mut best_dist = usize::MAX;
        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            let other = if e.node_a == junction {
                e.node_b
            } else {
                e.node_a
            };
            if let Some(&d) = dist_from_in.get(&other) {
                if d < best_dist {
                    best_dist = d;
                    injection_node = other;
                }
            }
        }
        if best_dist == usize::MAX {
            injection_node = graph.gnd_node;
        }

        let source_node = graph.edges.len() + 6000;
        let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();
        sp_edges.push((source_node, injection_node, SpTree::Leaf(vs_comp_idx)));

        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
        }

        let terminals = vec![source_node, junction];
        let sp_tree = match sp_reduce(sp_edges, &terminals) {
            Ok(t) => t,
            Err(_) => continue,
        };

        let mut ota_components = graph.components.clone();
        while ota_components.len() <= vs_comp_idx {
            ota_components.push(ComponentDef {
                id: "__vs__".to_string(),
                kind: ComponentKind::Resistor(1.0),
            });
        }

        let tree = sp_to_dyn_with_vs(&sp_tree, &ota_components, &graph.fork_paths, sample_rate, vs_comp_idx);

        let model = OtaModel::ca3080();
        let root = RootKind::Ota(OtaRoot::new(model));

        // OTA stages need compensation: the transconductance amplifier's
        // voltage gain (gm * Rp) can be very high. In real circuits, negative
        // feedback (R2 in Dyna Comp) tames this. We apply a compensation
        // factor based on the feedback network impedance.
        let feedback_compensation = 0.08; // Models closed-loop gain reduction from feedback R

        stages.push(WdfStage {
            tree,
            root,
            compensation: feedback_compensation,
            oversampler: Oversampler::new(oversampling),
            base_diode_model: None,
            paired_opamp: None,
        });
    }

    // Build WDF stages for zener diodes.
    let zeners = graph.find_zeners();
    let zener_edge_indices: Vec<usize> = zeners.iter().map(|(idx, _)| *idx).collect();
    let all_nonlinear_with_zeners: Vec<usize> = all_nonlinear_with_triodes
        .iter()
        .chain(zener_edge_indices.iter())
        .copied()
        .collect();

    for (_edge_idx, zener_info) in &zeners {
        let junction = zener_info.junction_node;
        let passive_idxs = graph.elements_at_junction(
            junction,
            &all_nonlinear_with_zeners,
            &graph.active_edge_indices,
        );

        if passive_idxs.is_empty() {
            continue;
        }

        // Find the best injection node for the voltage source.
        let mut injection_node = graph.in_node;
        let mut best_dist = usize::MAX;
        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            let other = if e.node_a == junction {
                e.node_b
            } else {
                e.node_a
            };
            if let Some(&d) = dist_from_in.get(&other) {
                if d < best_dist {
                    best_dist = d;
                    injection_node = other;
                }
            }
        }

        // Build SP edges.
        let source_node = graph.edges.len() + 4000; // virtual source node (unique offset for zeners)
        let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();
        sp_edges.push((source_node, injection_node, SpTree::Leaf(vs_comp_idx)));

        for &eidx in &passive_idxs {
            let e = &graph.edges[eidx];
            sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
        }

        let terminals = vec![source_node, junction];
        let sp_tree = match sp_reduce(sp_edges, &terminals) {
            Ok(t) => t,
            Err(_) => continue,
        };

        let mut zener_components = graph.components.clone();
        while zener_components.len() <= vs_comp_idx {
            zener_components.push(ComponentDef {
                id: "__vs__".to_string(),
                kind: ComponentKind::Resistor(1.0),
            });
        }

        let tree = sp_to_dyn_with_vs(&sp_tree, &zener_components, &graph.fork_paths, sample_rate, vs_comp_idx);

        let model = ZenerModel::with_voltage(zener_info.voltage);
        let root = RootKind::Zener(ZenerRoot::new(model));

        stages.push(WdfStage {
            tree,
            root,
            compensation: 1.0,
            oversampler: Oversampler::new(oversampling),
            base_diode_model: None,
            paired_opamp: None,
        });
    }

    // Balance voltage source impedance in each stage.
    // This fixes topologies where the Vs branch sits in a Parallel adaptor
    // opposite a high-impedance element (e.g. Big Muff: Parallel(Series(Vs,C), R)
    // with R >> C causes gamma ≈ 1.0 and severe signal attenuation).
    for stage in &mut stages {
        stage.balance_vs_impedance();
    }

    // ── Passive-only stage for circuits without nonlinear elements ─────────
    // If the circuit has reactive elements (capacitors, inductors) but no
    // nonlinear elements, we need a WDF stage to model the filtering.
    // Without this, capacitor high-pass behavior and leakage wouldn't work.
    if stages.is_empty() {
        let has_reactive = pedal.components.iter().any(|c| {
            matches!(c.kind, ComponentKind::Capacitor(_) | ComponentKind::Inductor(_))
        });

        if has_reactive {
            // Find all passive edges (resistors, capacitors, inductors)
            let passive_edges: Vec<usize> = graph
                .edges
                .iter()
                .enumerate()
                .filter(|(_, e)| {
                    let kind = &graph.components[e.comp_idx].kind;
                    matches!(
                        kind,
                        ComponentKind::Resistor(_)
                            | ComponentKind::Capacitor(_)
                            | ComponentKind::Inductor(_)
                            | ComponentKind::Potentiometer(_)
                    )
                })
                .map(|(i, _)| i)
                .collect();

            if !passive_edges.is_empty() {
                // Build SP edges for all passive elements
                let source_node = graph.edges.len() + 1000;
                let mut sp_edges: Vec<(NodeId, NodeId, SpTree)> = Vec::new();

                // Add voltage source edge: source_node → in_node
                sp_edges.push((source_node, graph.in_node, SpTree::Leaf(vs_comp_idx)));

                for &eidx in &passive_edges {
                    let e = &graph.edges[eidx];
                    sp_edges.push((e.node_a, e.node_b, SpTree::Leaf(e.comp_idx)));
                }

                // Use out_node as the junction (where we measure the output)
                let terminals = vec![source_node, graph.out_node];
                if let Ok(sp_tree) = sp_reduce(sp_edges, &terminals) {
                    let mut all_components = graph.components.clone();
                    while all_components.len() <= vs_comp_idx {
                        all_components.push(ComponentDef {
                            id: "__vs__".to_string(),
                            kind: ComponentKind::Resistor(1.0),
                        });
                    }

                    let tree = sp_to_dyn_with_vs(
                        &sp_tree,
                        &all_components,
                        &graph.fork_paths,
                        sample_rate,
                        vs_comp_idx,
                    );

                    // Use a passthrough root for passive circuits.
                    // The tree processes the signal normally and the root
                    // reflects the incident wave unchanged (open circuit).
                    stages.push(WdfStage {
                        tree,
                        root: RootKind::Passthrough,
                        compensation: 1.0,
                        oversampler: Oversampler::new(oversampling),
                        base_diode_model: None,
                        paired_opamp: None,
                    });

                    // Balance the passive stage too
                    if let Some(stage) = stages.last_mut() {
                        stage.balance_vs_impedance();
                    }
                }
            }
        }
    }

    // ── Slew rate limiters (from op-amps) ─────────────────────────────────
    // Each op-amp in the circuit contributes a slew rate limiter to model
    // the HF compression from finite output slew rate.
    let mut slew_limiters = Vec::new();
    for comp in &pedal.components {
        if let ComponentKind::OpAmp(ot) = &comp.kind {
            // OTAs (CA3080) are very fast (50 V/µs) — slew limiting is negligible.
            // Only add slew limiters for voltage-mode op-amps with slow slew rates.
            if !ot.is_ota() {
                slew_limiters.push(SlewRateLimiter::new(ot.slew_rate(), sample_rate));
            }
        }
    }

    // ── BBD delay lines ──────────────────────────────────────────────────
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

    // ── Generic delay lines ─────────────────────────────────────────────
    // Collect delay_line() components and their associated tap() elements.
    // Build a map from delay line component ID → index for tap resolution.
    let mut delay_lines: Vec<DelayLineBinding> = Vec::new();
    let mut delay_id_to_idx: std::collections::HashMap<String, usize> = std::collections::HashMap::new();

    for comp in &pedal.components {
        if let ComponentKind::DelayLine(min_delay, max_delay, interp, medium) = &comp.kind {
            let idx = delay_lines.len();
            delay_id_to_idx.insert(comp.id.clone(), idx);
            let mut dl = crate::elements::DelayLine::new(
                *min_delay,
                *max_delay,
                sample_rate,
                *interp,
            );
            dl.set_medium(*medium);
            delay_lines.push(DelayLineBinding {
                delay_line: dl,
                taps: vec![1.0], // Default: single tap at 1× base delay
                comp_id: comp.id.clone(),
            });
        }
    }

    // Resolve tap() components → attach to parent delay lines.
    for comp in &pedal.components {
        if let ComponentKind::Tap(parent_id, ratio) = &comp.kind {
            if let Some(&dl_idx) = delay_id_to_idx.get(parent_id) {
                delay_lines[dl_idx].taps.push(*ratio);
            }
            // Silently ignore taps that reference unknown delay lines
            // (the DSL validator should catch this).
        }
    }

    // Auto-configure medium zones from tap positions.
    // Each tap creates a zone boundary for incremental medium processing.
    for dl_binding in &mut delay_lines {
        if dl_binding.delay_line.medium() != crate::elements::Medium::None {
            let taps = dl_binding.taps.clone();
            dl_binding.delay_line.configure_zones_from_taps(&taps, None);
        }
    }

    // Determine device-aware rail saturation model from the circuit's active devices.
    // Scan components to find the dominant active device type that shapes rail clipping.
    let rail_saturation = {
        let mut has_opamp = false;
        let mut has_bjt = false;
        let mut has_fet = false;
        let mut has_tube = false;
        let mut tube_mu = 100.0_f64;
        let mut opamp_swing = 0.85_f64; // default output swing ratio
        for comp in &pedal.components {
            match &comp.kind {
                ComponentKind::OpAmp(ot) if !ot.is_ota() => {
                    has_opamp = true;
                    // JFET-input op-amps (TL072) swing closer to rails than BJT-input
                    opamp_swing = match ot {
                        OpAmpType::Tl072 | OpAmpType::Tl082 | OpAmpType::Generic => 0.92,
                        OpAmpType::Ne5532 => 0.90,
                        OpAmpType::Jrc4558 | OpAmpType::Rc4558 => 0.87,
                        OpAmpType::Lm308 | OpAmpType::Lm741 | OpAmpType::Op07 => 0.85,
                        _ => 0.85,
                    };
                }
                ComponentKind::Npn(_) | ComponentKind::Pnp(_) => {
                    has_bjt = true;
                }
                ComponentKind::NJfet(_) | ComponentKind::PJfet(_)
                | ComponentKind::Nmos(_) | ComponentKind::Pmos(_) => {
                    has_fet = true;
                }
                ComponentKind::Triode(tt) => {
                    has_tube = true;
                    tube_mu = match tt {
                        TriodeType::T12ax7 => 100.0,
                        TriodeType::T12at7 => 60.0,
                        TriodeType::T12ay7 => 40.0,
                        TriodeType::T12au7 => 17.0,
                        TriodeType::T12bh7 => 17.0, // Similar mu to 12AU7, higher current
                        TriodeType::T6386 => 40.0,  // Variable-mu: 5-50, nominal 40
                    };
                }
                ComponentKind::Pentode(_) => {
                    has_tube = true;
                    tube_mu = 200.0; // pentodes have higher effective gain
                }
                _ => {}
            }
        }
        // Priority: tube > BJT > FET > op-amp.
        // The dominant nonlinear device determines the rail saturation character.
        if has_tube {
            RailSaturation::Tube { mu: tube_mu }
        } else if has_bjt {
            let vce_sat = if has_germanium { 0.3 } else { 0.2 };
            RailSaturation::Bjt { vce_sat }
        } else if has_fet {
            RailSaturation::Fet
        } else if has_opamp {
            RailSaturation::OpAmp { output_swing_ratio: opamp_swing }
        } else {
            RailSaturation::None
        }
    };

    // Collect LFO component IDs for control binding.
    let lfo_ids: Vec<String> = pedal
        .components
        .iter()
        .filter_map(|c| {
            if matches!(c.kind, ComponentKind::Lfo(..)) {
                Some(c.id.clone())
            } else {
                None
            }
        })
        .collect();

    // Build control bindings.
    let mut controls = Vec::new();
    for ctrl in &pedal.controls {
        // First check if this control targets a Switch/RotarySwitch component
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
            // This pot is in an op-amp feedback path - use runtime gain modulation
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
            // Check if there's an LFO to control
            if !lfo_ids.is_empty() {
                ControlTarget::LfoRate(0) // Control first LFO by default
            } else {
                ControlTarget::PreGain // fallback
            }
        } else if is_depth_label(&ctrl.label) {
            if !lfo_ids.is_empty() {
                ControlTarget::LfoDepth(0)
            } else {
                ControlTarget::PreGain // fallback
            }
        } else if is_delay_time_label(&ctrl.label) && !delay_lines.is_empty() {
            ControlTarget::DelayTime(0) // Control first delay line by default
        } else if is_delay_feedback_label(&ctrl.label) && !delay_lines.is_empty() {
            ControlTarget::DelayFeedback(0)
        } else {
            // Check if this pot connects to an LFO.rate via nets
            let mut lfo_target = None;
            for net in &pedal.nets {
                if let Pin::ComponentPin { component, pin } = &net.from {
                    if component == &ctrl.component && pin == "wiper" {
                        // This pot's wiper connects somewhere
                        for to_pin in &net.to {
                            if let Pin::ComponentPin {
                                component: target_comp,
                                pin: target_pin,
                            } = to_pin
                            {
                                if target_pin == "rate" {
                                    // Find the LFO index
                                    if let Some(idx) =
                                        lfo_ids.iter().position(|id| id == target_comp)
                                    {
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
                // Try to find this pot in a WDF stage.
                let mut found_stage = None;
                for (si, stage) in stages.iter().enumerate() {
                    if has_pot(&stage.tree, &ctrl.component) {
                        found_stage = Some(si);
                        break;
                    }
                }
                match found_stage {
                    Some(si) => ControlTarget::PotInStage(si),
                    None => ControlTarget::PreGain, // fallback: map unknown controls to gain
                }
            }
        };

        let max_r = pedal
            .components
            .iter()
            .find(|c| c.id == ctrl.component)
            .and_then(|c| match &c.kind {
                ComponentKind::Potentiometer(r) => Some(*r),
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

    // Compute initial pre-gain from default control values.
    let gain_default = pedal
        .controls
        .iter()
        .find(|c| is_gain_label(&c.label))
        .map(|c| c.default)
        .unwrap_or(0.5);
    let level_default = pedal
        .controls
        .iter()
        .find(|c| is_level_label(&c.label))
        .map(|c| c.default)
        .unwrap_or(0.8);

    let (glo, ghi) = gain_range;
    let ratio: f64 = ghi / glo;
    // Apply op-amp feedback gain (from detected inverting/non-inverting topologies)
    // This is the actual closed-loop gain from Rf/Ri ratios.
    let pre_gain = glo * ratio.powf(gain_default) * active_bonus * opamp_feedback_gain;

    // Helper: trace through resistive paths (pots, resistors) to find modulation targets.
    // Returns (target_component, target_property) if found.
    fn trace_through_resistive_path<'a>(
        start_comp: &str,
        start_pin: &str,
        pedal: &'a PedalDef,
        visited: &mut HashSet<String>,
    ) -> Option<(String, String)> {
        // Mark this component as visited to avoid cycles
        let key = format!("{}:{}", start_comp, start_pin);
        if visited.contains(&key) {
            return None;
        }
        visited.insert(key);

        // Find the component type
        let comp_kind = pedal
            .components
            .iter()
            .find(|c| c.id == start_comp)
            .map(|c| &c.kind);

        // Only trace through resistive elements (pots, resistors)
        let is_resistive = matches!(
            comp_kind,
            Some(ComponentKind::Potentiometer(_)) | Some(ComponentKind::Resistor(_))
        );
        if !is_resistive {
            // This is a non-resistive component - check if it's a modulation target
            let recognized_targets = ["clock", "vgs", "gate", "led", "vgk", "vg1k", "iabc", "speed_mod", "delay_time"];
            if recognized_targets.contains(&start_pin) {
                return Some((start_comp.to_string(), start_pin.to_string()));
            }
            return None;
        }

        // Find connections from other pins of this resistive element
        // For pots: a, b, wiper; for resistors: a, b
        let other_pins: Vec<&str> = match comp_kind {
            Some(ComponentKind::Potentiometer(_)) => {
                // Pot pins: a (ccw), b (wiper/output), wiper
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

        // Search nets for connections from other pins
        for other_pin in other_pins {
            for net in &pedal.nets {
                // Check if this net originates from our component's other pin
                if let Pin::ComponentPin { component, pin } = &net.from {
                    if component == start_comp && pin == other_pin {
                        // Follow all targets
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
                // Also check if this component.pin appears in the targets
                for src_target in &net.to {
                    if let Pin::ComponentPin { component, pin } = src_target {
                        if component == start_comp && pin == other_pin {
                            // This pin is a target - check the source
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

    // Build LFO bindings from LFO components and their net connections.
    let mut lfos = Vec::new();
    for comp in &pedal.components {
        if let ComponentKind::Lfo(waveform_dsl, timing_r, timing_c) = &comp.kind {
            // Convert DSL waveform to elements waveform
            let waveform = match waveform_dsl {
                LfoWaveformDsl::Sine => crate::elements::LfoWaveform::Sine,
                LfoWaveformDsl::Triangle => crate::elements::LfoWaveform::Triangle,
                LfoWaveformDsl::Square => crate::elements::LfoWaveform::Square,
                LfoWaveformDsl::SawUp => crate::elements::LfoWaveform::SawUp,
                LfoWaveformDsl::SawDown => crate::elements::LfoWaveform::SawDown,
                LfoWaveformDsl::SampleAndHold => crate::elements::LfoWaveform::SampleAndHold,
            };

            // Compute base frequency from RC timing: f = 1/(2πRC)
            let base_freq = 1.0 / (2.0 * std::f64::consts::PI * timing_r * timing_c);

            // Create LFO with base frequency
            let mut lfo = crate::elements::Lfo::new(waveform, sample_rate);
            lfo.set_rate(base_freq);

            // Track if we've already created an AllJfetVgs binding for this LFO
            let mut created_all_jfet_binding = false;

            // Find what this LFO connects to via nets (LFO.out -> target.property)
            for net in &pedal.nets {
                if let Pin::ComponentPin { component, pin } = &net.from {
                    if component == &comp.id && pin == "out" {
                        // This LFO's output connects to targets in net.to
                        for target_pin in &net.to {
                            if let Pin::ComponentPin {
                                component: target_comp,
                                pin: target_prop,
                            } = target_pin
                            {
                                let target = match target_prop.as_str() {
                                    "vgs" | "gate" => {
                                        // Count how many JFETs this LFO connects to
                                        let jfet_count = stages
                                            .iter()
                                            .filter(|s| matches!(&s.root, RootKind::Jfet(_)))
                                            .count();

                                        if jfet_count > 1 {
                                            // Multiple JFETs - use AllJfetVgs
                                            // Only create ONE binding for AllJfetVgs
                                            if created_all_jfet_binding {
                                                continue; // Skip duplicate bindings
                                            }
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
                                        // Find which stage contains this triode
                                        let stage_idx = stages
                                            .iter()
                                            .position(|s| matches!(&s.root, RootKind::Triode(_)))
                                            .unwrap_or(0);
                                        ModulationTarget::TriodeVgk { stage_idx }
                                    }
                                    "vg1k" => {
                                        // Find which stage contains this pentode
                                        let stage_idx = stages
                                            .iter()
                                            .position(|s| matches!(&s.root, RootKind::Pentode(_)))
                                            .unwrap_or(0);
                                        ModulationTarget::PentodeVg1k { stage_idx }
                                    }
                                    "iabc" => {
                                        // Find which stage contains an OTA
                                        let stage_idx = stages
                                            .iter()
                                            .position(|s| matches!(&s.root, RootKind::Ota(_)))
                                            .unwrap_or(0);
                                        ModulationTarget::OtaIabc { stage_idx }
                                    }
                                    "clock" => {
                                        // Find which BBD to modulate
                                        let bbd_idx = 0; // First BBD by default
                                        ModulationTarget::BbdClock { bbd_idx }
                                    }
                                    "speed_mod" => {
                                        // Find which delay line to modulate
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
                                        // Try tracing through resistive path to find target
                                        let mut visited = HashSet::new();
                                        if let Some((final_comp, final_pin)) =
                                            trace_through_resistive_path(
                                                target_comp,
                                                target_prop,
                                                pedal,
                                                &mut visited,
                                            )
                                        {
                                            // Found a target through resistive path
                                            match final_pin.as_str() {
                                                "clock" => {
                                                    ModulationTarget::BbdClock { bbd_idx: 0 }
                                                }
                                                "vgs" | "gate" => {
                                                    let jfet_count = stages
                                                        .iter()
                                                        .filter(|s| {
                                                            matches!(&s.root, RootKind::Jfet(_))
                                                        })
                                                        .count();
                                                    if jfet_count > 1 {
                                                        if created_all_jfet_binding {
                                                            continue;
                                                        }
                                                        created_all_jfet_binding = true;
                                                        ModulationTarget::AllJfetVgs
                                                    } else if let Some(stage_idx) = stages
                                                        .iter()
                                                        .position(|s| {
                                                            matches!(&s.root, RootKind::Jfet(_))
                                                        })
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

                                // Bias and range based on target type
                                let (bias, range) = match &target {
                                    // JFET phaser: Vgs must stay in variable-resistance region
                                    // 2SK30A-GR: Vp ~ -0.8V to -1.2V
                                    // Keep Vgs between -0.2V and -0.7V to avoid pinch-off
                                    // At -0.2V: low Rds (signal passes)
                                    // At -0.7V: high Rds (but still conducting)
                                    ModulationTarget::JfetVgs { .. } => (-0.45, 0.25),
                                    ModulationTarget::AllJfetVgs => (-0.45, 0.25),
                                    ModulationTarget::PhotocouplerLed { .. } => (0.5, 0.5),
                                    // Triode grid bias: -2V center with ±2V swing
                                    ModulationTarget::TriodeVgk { .. } => (-2.0, 2.0),
                                    // Pentode control grid: -2V center with ±2V swing
                                    ModulationTarget::PentodeVg1k { .. } => (-2.0, 2.0),
                                    // MOSFET: bias above threshold with swing
                                    ModulationTarget::MosfetVgs { .. } => (3.0, 2.0),
                                    // OTA: normalized gain 0-1
                                    ModulationTarget::OtaIabc { .. } => (0.5, 0.5),
                                    // BBD: short-delay chorus range (center ~3.3 ms, sweep ~3–5 ms)
                                    ModulationTarget::BbdClock { .. } => (0.15, 0.10),
                                    // Op-amp Vp: follows input signal
                                    ModulationTarget::OpAmpVp { .. } => (0.0, 1.0),
                                    // Delay speed: wow ±2%, flutter ±0.5%
                                    ModulationTarget::DelaySpeed { .. } => (0.0, 0.02),
                                    // Delay time: center 0.5, full sweep
                                    ModulationTarget::DelayTime { .. } => (0.5, 0.5),
                                };

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

    // Build EnvelopeFollower bindings from EnvelopeFollower components and their net connections.
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

            // Find what this envelope follower connects to via nets (EF.out -> target.property)
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
                                        let stage_idx = stages
                                            .iter()
                                            .position(|s| matches!(&s.root, RootKind::Triode(_)))
                                            .unwrap_or(0);
                                        ModulationTarget::TriodeVgk { stage_idx }
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
                                    "clock" => {
                                        ModulationTarget::BbdClock { bbd_idx: 0 }
                                    }
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

                                let (bias, range) = match &target {
                                    // JFET: Keep in variable-resistance region (avoid pinch-off)
                                    ModulationTarget::JfetVgs { .. } => (-0.45, 0.25),
                                    ModulationTarget::AllJfetVgs => (-0.45, 0.25),
                                    ModulationTarget::PhotocouplerLed { .. } => (0.0, 1.0),
                                    ModulationTarget::TriodeVgk { .. } => (-2.0, 2.0),
                                    ModulationTarget::PentodeVg1k { .. } => (-2.0, 2.0),
                                    ModulationTarget::MosfetVgs { .. } => (3.0, 2.0),
                                    ModulationTarget::OtaIabc { .. } => (0.5, 0.5),
                                    ModulationTarget::BbdClock { .. } => (0.15, 0.10),
                                    // Op-amp Vp: follows input signal
                                    ModulationTarget::OpAmpVp { .. } => (0.0, 1.0),
                                    ModulationTarget::DelaySpeed { .. } => (0.0, 0.02),
                                    ModulationTarget::DelayTime { .. } => (0.5, 0.5),
                                };

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

    // Set up thermal model if enabled and circuit uses germanium.
    let thermal = if enable_thermal && has_germanium {
        Some(ThermalModel::germanium_fuzz(sample_rate))
    } else if enable_thermal {
        Some(ThermalModel::silicon_standard(sample_rate))
    } else {
        None
    };

    // Use primary supply voltage from .pedal file, defaulting to 9V for typical pedals
    // For multi-rail circuits, use the first (primary) supply.
    let primary_supply = pedal.supplies.first().map(|s| &s.config);
    let supply_voltage = primary_supply.map_or(9.0, |s| s.voltage);

    // Build power supply sag model if impedance parameters are specified
    let power_supply = primary_supply
        .filter(|s| s.has_sag())
        .map(|s| {
            crate::elements::PowerSupply::new(
                s.voltage,
                s.impedance.unwrap_or(0.0),
                s.filter_cap.unwrap_or(100e-6),
                s.rectifier,
                sample_rate,
            )
        });

    let mut compiled = CompiledPedal {
        stages,
        pre_gain,
        output_gain: level_default,
        rail_saturation,
        sample_rate,
        controls,
        gain_range: (glo * active_bonus, ghi * active_bonus),
        supply_voltage: 9.0, // Will be updated by set_supply_voltage
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
    };

    // Apply supply voltage - this propagates v_max to all op-amp stages.
    // When a power supply sag model is present, use its steady-state output
    // voltage (which accounts for the rectifier static drop) so the initial
    // v_max matches what the PSU will actually produce from the first sample.
    let initial_voltage = match &compiled.power_supply {
        Some(psu) => psu.steady_state_voltage(),
        None => supply_voltage,
    };
    compiled.set_supply_voltage(initial_voltage);

    Ok(compiled)
}

