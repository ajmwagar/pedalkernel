//! IIR rigid stage builder.
//!
//! Builds an MNA system from the rigid stage's edges, stamps all
//! components (resistors, caps, VCVS), and calls `MnaSystem::build_iir()`
//! to get biquad coefficients. O(1)/sample at runtime.
//!
//! Covers both passive bridges AND active linear circuits (808 bridged-T
//! with op-amp VCVS + capacitors). VCVS is a linear constraint.

use super::super::component::EdgeKind;
use super::super::dyn_node::DynNode;
use super::super::graph::{CircuitGraph, NodeId};
use super::super::stage::IirData;
use super::mna_builder::build_mna;

struct FeedbackParams {
    rf: f64,
    r_crit: f64,
    f0: f64,
    r_series: [f64; 2],
    c_shunt: [f64; 2],
}

/// Build a biquad IIR from a linear rigid R-node.
///
/// 1. Builds MNA via shared `build_mna()` (node collection + stamping)
/// 2. Calls `build_iir()` to get (b, a) biquad coefficients
/// 3. Falls back to state-space biquad extraction for ≤2nd order
/// 4. Returns IirData (caller wraps in IirStage)
pub(in crate::compiler) fn build_iir_stage(
    edge_indices: &[usize],
    pendant_trees: &[(DynNode, NodeId)],
    graph: &CircuitGraph,
    sample_rate: f64,
) -> Result<IirData, String> {
    let built = build_mna(edge_indices, pendant_trees, graph, sample_rate)?;
    let mna = built.mna;
    let cap_stamps = built.cap_stamps;
    let vs_idx = built.vs_idx;
    let out_mna = built.output_mna;

    // ── No caps → pure resistive network (DC gain only) ──────────
    // Compute the actual gain from MNA instead of assuming passthrough.
    // This handles pot voltage dividers, attenuator pads, etc.
    if cap_stamps.is_empty() {
        let dc_gain = mna.dc_gain(vs_idx, out_mna);
        #[cfg(test)]
        eprintln!("IIR dc_gain={dc_gain:.4} vs_idx={vs_idx} out_mna={out_mna:?}");
        return Ok(IirData::new(
            vec![dc_gain, 0.0, 0.0],
            vec![1.0, 0.0, 0.0],
            sample_rate,
        ));
    }

    // ── Extract feedback_r for VCVS circuits ──────────────────────
    let feedback = extract_feedback_r(edge_indices, graph);

    // ── Build IIR biquad ─────────────────────────────────────────
    if let Some((b_coeffs, a_coeffs)) = mna.build_iir(
        &cap_stamps,
        vs_idx,
        out_mna,
        None,
        sample_rate,
        feedback
            .as_ref()
            .map(|params| (params.rf, params.r_crit, params.f0)),
    ) {
        let mut iir = IirData::new(b_coeffs, a_coeffs, sample_rate);
        if let Some(params) = feedback {
            iir.r_fb = params.rf;
            iir.r_crit = params.r_crit;
            iir.r_series_base = params.r_series;
            iir.c_shunt_base = params.c_shunt;
            iir.r_series_product = params.r_series[0] * params.r_series[1];
            iir.c_shunt_product = params.c_shunt[0] * params.c_shunt[1];
        }
        return Ok(iir);
    }

    // Fallback: state-space reduction → extract biquad if 2nd order
    let (a_d, b_d, c_out, n_states, d_feedthrough) =
        mna.build_state_space_matrices(&cap_stamps, vs_idx, out_mna, None, sample_rate);

    if n_states <= 2 && n_states > 0 {
        if n_states == 1 {
            let a_val = a_d[0];
            let b0 = d_feedthrough;
            let b1 = c_out[0] * b_d[0] - d_feedthrough * a_val;
            return Ok(IirData::new(vec![b0, b1], vec![1.0, -a_val], sample_rate));
        } else {
            let a11 = a_d[0];
            let a12 = a_d[1];
            let a21 = a_d[2];
            let a22 = a_d[3];
            let da1 = -(a11 + a22);
            let da2 = a11 * a22 - a12 * a21;
            let num_z1 = c_out[0] * b_d[0] + c_out[1] * b_d[1];
            let num_z0 = c_out[0] * (-a22 * b_d[0] + a12 * b_d[1])
                + c_out[1] * (a21 * b_d[0] - a11 * b_d[1]);
            let b0 = d_feedthrough;
            let b1 = d_feedthrough * da1 + num_z1;
            let b2 = d_feedthrough * da2 + num_z0;
            return Ok(IirData::new(
                vec![b0, b1, b2],
                vec![1.0, da1, da2],
                sample_rate,
            ));
        }
    }

    Err(format!(
        "IIR: circuit has {} states (need ≤2 for biquad)",
        n_states
    ))
}

/// Extract feedback parameters (Rf, R_crit, f0) from a VCVS rigid stage.
///
/// Classifies edges by their relationship to the VCVS neg/out nodes:
/// - Linear edge spanning neg→out → **Rf** (feedback resistor)
/// - Linear edge touching neg OR out (not both) → **series R** (T-network)
/// - Reactive edge touching GND → **shunt C**
///
/// Returns `None` if the circuit doesn't have the right structure.
fn extract_feedback_r(edge_indices: &[usize], graph: &CircuitGraph) -> Option<FeedbackParams> {
    // Find the VCVS edge → get neg and out nodes
    let vcvs_rec = graph.nullor_pins.iter().find(|rec| {
        edge_indices
            .iter()
            .any(|&eidx| graph.edges[eidx].comp_idx == rec.comp_idx)
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
                        rf += r;
                    } else if (touches_neg || touches_out) && !touches_gnd {
                        r_series.push(r);
                    }
                }
            }
            EdgeKind::Reactive => {
                if let Some(c) = comp.kind.capacitance() {
                    if touches_gnd {
                        c_shunt.push(c);
                    }
                }
            }
            _ => {}
        }
    }

    if rf <= 0.0 || r_series.len() < 2 || c_shunt.len() < 2 {
        return None;
    }

    let r_product: f64 = r_series.iter().take(2).product();
    let c_product: f64 = c_shunt.iter().take(2).product();
    let f0 = 1.0 / (2.0 * std::f64::consts::PI * (r_product * c_product).sqrt());

    let r_crit = r_series[0] + r_series[1] + r_series[0] * c_shunt[0] / c_shunt[1];

    Some(FeedbackParams {
        rf,
        r_crit,
        f0,
        r_series: [r_series[0], r_series[1]],
        c_shunt: [c_shunt[0], c_shunt[1]],
    })
}
