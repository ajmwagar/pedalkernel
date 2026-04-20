//! IIR rigid stage builder.
//!
//! Builds an MNA system from the rigid stage's edges, stamps all
//! components (resistors, caps, VCVS), and calls `MnaSystem::build_iir()`
//! to get biquad coefficients. O(1)/sample at runtime.
//!
//! Covers both passive bridges AND active linear circuits (808 bridged-T
//! with op-amp VCVS + capacitors). VCVS is a linear constraint.

use super::super::component::{EdgeKind, StampContext};
use super::super::dyn_node::DynNode;
use super::super::graph::{CircuitGraph, NodeId};
use super::super::stage::IirData;
use crate::tree::MnaSystem;

/// Build a biquad IIR from a linear rigid R-node.
///
/// 1. Collects unique circuit nodes from edges
/// 2. Creates MNA system, stamps each component
/// 3. Calls `build_iir()` to get (b, a) biquad coefficients
/// 4. Returns IirData (caller wraps in IirStage)
pub(in crate::compiler) fn build_iir_stage(
    edge_indices: &[usize],
    pendant_trees: &[(DynNode, NodeId)],
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<IirData, String> {
    // ── Step 1: Collect unique MNA nodes ─────────────────────────────
    let mut node_set: Vec<NodeId> = Vec::new();
    let mut add_node = |node: NodeId| {
        if node == graph.gnd_node || graph.supply_nodes.contains(&node) {
            return;
        }
        if !node_set.contains(&node) {
            node_set.push(node);
        }
    };

    for &eidx in edge_indices {
        let e = &graph.edges[eidx];
        add_node(e.node_a);
        add_node(e.node_b);
    }
    for (_, attach) in pendant_trees {
        add_node(*attach);
    }

    // Also add nodes from VCVS pos pin (might be at ground/bias, but add anyway)
    for rec in &graph.nullor_pins {
        let is_in_stage = edge_indices.iter().any(|&eidx| {
            graph.edges[eidx].comp_idx == rec.comp_idx
        });
        if is_in_stage {
            add_node(rec.pos_node);
            add_node(rec.neg_node);
            add_node(rec.out_node);
        }
    }

    let num_nodes = node_set.len();
    if num_nodes == 0 {
        return Err("IIR: no circuit nodes found".to_string());
    }

    let node_to_mna = |node: NodeId| -> Option<usize> {
        if node == graph.gnd_node || graph.supply_nodes.contains(&node) {
            None
        } else {
            node_set.iter().position(|&n| n == node)
        }
    };

    // ── Step 2: Count voltage sources ────────────────────────────────
    let mut num_vsources: usize = 0;
    let mut comp_vsrc_base: Vec<(usize, usize)> = Vec::new(); // (comp_idx, vsrc_base)

    for &eidx in edge_indices {
        let comp = &graph.components[graph.edges[eidx].comp_idx];
        let count = comp.kind.mna_vsource_count();
        if count > 0 {
            comp_vsrc_base.push((graph.edges[eidx].comp_idx, num_vsources));
            num_vsources += count;
        }
    }

    // Input voltage source (last)
    let vs_idx = num_vsources;
    num_vsources += 1;

    // ── Step 3: Build MNA and stamp components ──────────────────────
    let mut mna = MnaSystem::new(num_nodes, num_vsources);
    let mut cap_stamps: Vec<(Option<usize>, Option<usize>, f64)> = Vec::new();

    for &eidx in edge_indices {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        let n1 = node_to_mna(e.node_a);
        let n2 = node_to_mna(e.node_b);
        let edge_kind = graph.effective_edge_kind(eidx);

        match edge_kind {
            EdgeKind::Linear => {
                if let Some(r) = comp.kind.resistance() {
                    mna.stamp_resistor(n1, n2, r);
                }
            }
            EdgeKind::Reactive => {
                if let Some(c) = comp.kind.capacitance() {
                    cap_stamps.push((n1, n2, c));
                }
                // Inductors: stamp as conductance (1/sL → bilinear)
                if let Some(l) = comp.kind.inductance() {
                    cap_stamps.push((n1, n2, -l)); // Negative = inductor convention
                }
            }
            EdgeKind::Vcvs => {
                // Stamp via Component::stamp_mna_multi
                let vsrc_base = comp_vsrc_base
                    .iter()
                    .find(|&&(ci, _)| ci == e.comp_idx)
                    .map(|&(_, base)| base)
                    .unwrap_or(0);

                let pin_fn = |pin: &str| -> Option<usize> {
                    let key = format!("{}.{}", comp.id, pin);
                    let node = graph.node_names.get(&key)?;
                    node_to_mna(*node)
                };
                // cap_stamps: None — don't collect op-amp internal caps (GBW modeling).
                // The circuit's physical caps (C1, C2) are already collected from
                // Reactive edges. The op-amp output cap would add a spurious state.
                let mut ctx = StampContext {
                    pin_to_mna: &pin_fn,
                    vsrc_base: vsrc_base,
                    internal_node_base: 0,
                    sample_rate,
                    cap_stamps: None,
                };
                comp.kind.stamp_mna_multi(&comp.id, &mut ctx, &mut mna);
            }
            _ => {} // Skip NL, Vccs, Behavioral (shouldn't be in IIR stage)
        }
    }

    // Pendant WDF trees: stamp as port resistance at attachment node
    for (tree, attach_node) in pendant_trees {
        let n = node_to_mna(*attach_node);
        let rp = tree.port_resistance();
        if rp > 0.0 {
            mna.stamp_resistor(n, None, rp);
        }
    }

    // Input voltage source — inject at graph.in_node (signal entry point)
    let injection_mna = node_to_mna(graph.in_node);
    mna.stamp_voltage_source(injection_mna, None, vs_idx);

    // Output node — typically the VCVS out node or last edge node
    let output_node = graph.nullor_pins.iter()
        .find(|rec| edge_indices.iter().any(|&eidx| graph.edges[eidx].comp_idx == rec.comp_idx))
        .map(|rec| rec.out_node)
        .unwrap_or(graph.out_node);
    let out_mna = node_to_mna(output_node);

    // ── No caps → pure resistive (DC gain only) ──────────────────
    if cap_stamps.is_empty() {
        // Trivial biquad: just a gain. b=[gain,0,0], a=[1,0,0].
        // The gain comes from the resistive divider in the MNA.
        // For now, return a unity passthrough biquad.
        return Ok(IirData::new(vec![1.0, 0.0, 0.0], vec![1.0, 0.0, 0.0], sample_rate));
    }

    // ── Step 4: Extract feedback_r for VCVS circuits ──────────────
    // For bridged-T and similar VCVS oscillators, build_iir needs
    // (Rf, R_crit, f0) to compute biquad coefficients directly.
    // Classification uses graph structure: edges touching neg/out nodes.
    let feedback_r = extract_feedback_r(edge_indices, graph);

    // ── Step 5: Build IIR biquad ────────────────────────────────────
    match mna.build_iir(&cap_stamps, vs_idx, out_mna, None, sample_rate, feedback_r.as_ref().map(|&(rf, rc, f0)| (rf, rc, f0))) {
        Some((b_coeffs, a_coeffs)) => {
            let mut iir = IirData::new(b_coeffs, a_coeffs, sample_rate);
            // Store component values for pot recomputation
            if let Some((rf, r_crit, _)) = feedback_r {
                iir.r_fb = rf;
                iir.r_crit = r_crit;
            }
            Ok(iir)
        }
        None => Err("IIR: build_iir failed — circuit may need StateSpace".to_string()),
    }
}

/// Extract feedback parameters (Rf, R_crit, f0) from a VCVS rigid stage.
///
/// Classifies edges by their relationship to the VCVS neg/out nodes:
/// - Linear edge spanning neg→out → **Rf** (feedback resistor)
/// - Linear edge touching neg OR out (not both) → **series R** (T-network)
/// - Reactive edge touching GND → **shunt C**
///
/// Returns `None` if the circuit doesn't have the right structure.
fn extract_feedback_r(
    edge_indices: &[usize],
    graph: &CircuitGraph,
) -> Option<(f64, f64, f64)> {
    // Find the VCVS edge → get neg and out nodes
    let vcvs_rec = graph.nullor_pins.iter().find(|rec| {
        edge_indices.iter().any(|&eidx| graph.edges[eidx].comp_idx == rec.comp_idx)
    })?;
    let neg = vcvs_rec.neg_node;
    let out = vcvs_rec.out_node;

    let mut rf = 0.0f64;
    let mut r_series: Vec<f64> = Vec::new();
    let mut c_shunt: Vec<f64> = Vec::new();

    let is_ground = |node: NodeId| -> bool {
        node == graph.gnd_node
            || graph.ac_ground_nodes.contains(&node)
            || graph.supply_nodes.contains(&node)
    };

    for &eidx in edge_indices {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        let edge_kind = graph.effective_edge_kind(eidx);

        let touches_neg = e.node_a == neg || e.node_b == neg;
        let touches_out = e.node_a == out || e.node_b == out;
        let touches_gnd = is_ground(e.node_a) || is_ground(e.node_b);

        match edge_kind {
            EdgeKind::Linear => {
                if let Some(r) = comp.kind.resistance() {
                    if touches_neg && touches_out {
                        // Feedback R: spans neg→out directly
                        rf += r;
                    } else if (touches_neg || touches_out) && !touches_gnd {
                        // Series R: part of T-network (touches one side, not ground)
                        r_series.push(r);
                    }
                    // Bias/load resistors touching GND are ignored
                }
            }
            EdgeKind::Reactive => {
                if let Some(c) = comp.kind.capacitance() {
                    if touches_gnd {
                        // Shunt cap to ground
                        c_shunt.push(c);
                    }
                }
            }
            _ => {}
        }
    }

    // Need at least: Rf, 2 series R, 2 shunt C for bridged-T
    if rf <= 0.0 || r_series.len() < 2 || c_shunt.len() < 2 {
        return None;
    }

    // f0 = 1 / (2π √(R1·R2·C1·C2))
    let r_product: f64 = r_series.iter().take(2).product();
    let c_product: f64 = c_shunt.iter().take(2).product();
    let f0 = 1.0 / (2.0 * std::f64::consts::PI * (r_product * c_product).sqrt());

    // R_crit = R1 + R2 + R1·C1/C2
    let r_crit = r_series[0] + r_series[1] + r_series[0] * c_shunt[0] / c_shunt[1];

    Some((rf, r_crit, f0))
}
