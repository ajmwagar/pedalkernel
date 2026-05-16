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
        // b_d contains two packed vectors: b_plus[0..n] and b_minus[n..2n]
        // b_plus couples to u[n+1], b_minus couples to u[n].
        // H(z) = c·(zI-A)⁻¹·(b_plus·z + b_minus) + d
        let has_two_vectors = b_d.len() >= 2 * n_states;
        let bp = &b_d[..n_states];
        let bm = if has_two_vectors {
            &b_d[n_states..2 * n_states]
        } else {
            bp
        };

        if n_states == 1 {
            let a_val = a_d[0];
            // H(z) = c0·(z - a)⁻¹·(bp0·z + bm0) + d
            // = c0·(bp0·z + bm0)/(z - a) + d·(z - a)/(z - a)
            // = [c0·bp0·z + c0·bm0 + d·z - d·a] / (z - a)
            // = [(c0·bp0 + d)·z + (c0·bm0 - d·a)] / (z - a)
            let b0 = c_out[0] * bp[0] + d_feedthrough;
            let b1 = c_out[0] * bm[0] - d_feedthrough * a_val;
            return Ok(IirData::new(vec![b0, b1], vec![1.0, -a_val], sample_rate));
        } else {
            let a11 = a_d[0];
            let a12 = a_d[1];
            let a21 = a_d[2];
            let a22 = a_d[3];
            let da1 = -(a11 + a22);
            let da2 = a11 * a22 - a12 * a21;

            // H(z) = c·adj(zI-A)·(bp·z + bm)/det(zI-A) + d
            // adj(zI-A) = [z-a22, a12; a21, z-a11]
            //
            // c·adj·bp = z·(c0·bp0+c1·bp1) + (-c0·a22·bp0+c0·a12·bp1+c1·a21·bp0-c1·a11·bp1)
            //          = z·num_z1_p + num_z0_p
            // c·adj·bm = z·(c0·bm0+c1·bm1) + (-c0·a22·bm0+c0·a12·bm1+c1·a21·bm0-c1·a11·bm1)
            //          = z·num_z1_m + num_z0_m
            //
            // Numerator = z·(z·num_z1_p + num_z0_p) + (z·num_z1_m + num_z0_m) + d·det
            //           = z²·num_z1_p + z·(num_z0_p + num_z1_m) + num_z0_m + d·(z²+da1·z+da2)
            let num_z1_p = c_out[0] * bp[0] + c_out[1] * bp[1];
            let num_z0_p =
                c_out[0] * (-a22 * bp[0] + a12 * bp[1]) + c_out[1] * (a21 * bp[0] - a11 * bp[1]);
            let num_z1_m = c_out[0] * bm[0] + c_out[1] * bm[1];
            let num_z0_m =
                c_out[0] * (-a22 * bm[0] + a12 * bm[1]) + c_out[1] * (a21 * bm[0] - a11 * bm[1]);

            let b0 = num_z1_p + d_feedthrough;
            let b1 = num_z0_p + num_z1_m + d_feedthrough * da1;
            let b2 = num_z0_m + d_feedthrough * da2;
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

/// Build a precomputed biquad coefficient table by sweeping pot positions.
///
/// For each point on an N-dimensional grid (one dim per independent control),
/// re-stamps the MNA with the pot's resistance at that position and extracts
/// biquad coefficients. Returns a flat table of [b0,b1,b2,a1,a2] entries.
///
/// `control_labels`: unique control labels that map to pots in this stage.
/// `steps`: grid resolution per dimension (e.g. 64).
///
/// Table entry index for positions (d0, d1, ...): d0 + d1*steps + d2*steps² + ...
#[cfg(feature = "biquad-table")]
pub(in crate::compiler) fn build_biquad_table(
    edge_indices: &[usize],
    pendant_trees: &[(DynNode, NodeId)],
    graph: &CircuitGraph,
    sample_rate: f64,
    control_labels: &[String],
    steps: usize,
) -> Option<super::super::stage::BiquadTable> {
    use super::super::stage::BiquadTable;

    if control_labels.is_empty() || steps < 2 {
        return None;
    }

    let n_dims = control_labels.len();
    let total_entries = steps.checked_pow(n_dims as u32)?;
    let mut coeffs = Vec::with_capacity(total_entries * 5);

    // Identify pot edges: for each control label, find the pot component edges
    // and their MNA node pairs. We'll re-stamp these at each grid point.
    struct PotInfo {
        edge_idx: usize,
        comp_idx: usize,
        max_r: f64,
        dim: usize, // which control dimension this pot belongs to
    }

    let mut pot_infos: Vec<PotInfo> = Vec::new();
    for &eidx in edge_indices {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        if !comp.kind.is_pot() {
            continue;
        }
        // Match by component ID (control_labels contains comp_ids)
        for (di, comp_id) in control_labels.iter().enumerate() {
            if comp.id == *comp_id {
                if let Some(pot) = comp
                    .kind
                    .as_any()
                    .downcast_ref::<super::super::components::Potentiometer>()
                {
                    pot_infos.push(PotInfo {
                        edge_idx: eidx,
                        comp_idx: e.comp_idx,
                        max_r: pot.max_r,
                        dim: di,
                    });
                }
                break;
            }
        }
    }

    if pot_infos.is_empty() {
        return None;
    }

    #[cfg(test)]
    eprintln!(
        "  BiquadTable builder: {} pots found: {:?}",
        pot_infos.len(),
        pot_infos
            .iter()
            .map(|p| format!(
                "dim{}={} (max_r={:.0})",
                p.dim, graph.components[p.comp_idx].id, p.max_r
            ))
            .collect::<Vec<_>>()
    );

    // Iterate over all grid points
    for flat_idx in 0..total_entries {
        // Decode flat index → per-dimension step indices
        let mut dim_positions = vec![0.5f64; n_dims];
        let mut remaining = flat_idx;
        for d in 0..n_dims {
            let step = remaining % steps;
            remaining /= steps;
            dim_positions[d] = step as f64 / (steps - 1) as f64;
        }

        // Build MNA with pot resistances at these positions
        // We need to rebuild the full MNA because pot resistance affects G matrix
        // Use a temporary graph override approach: modify pot resistance, build, restore
        //
        // Actually, since we can't modify the graph, we rebuild the MNA each time
        // but override the pot stamps. The cleanest way: build MNA normally,
        // then adjust the G matrix entries for each pot.

        let built = match build_mna(edge_indices, pendant_trees, graph, sample_rate) {
            Ok(b) => b,
            Err(_) => return None,
        };
        let mut mna = built.mna;
        let cap_stamps = &built.cap_stamps;
        let vs_idx = built.vs_idx;
        let out_mna = built.output_mna;

        // Collect MNA node indices for pot edges
        let node_to_mna = |node: NodeId| -> Option<usize> {
            if node == graph.gnd_node || graph.supply_nodes.contains(&node) {
                None
            } else {
                built.node_set.iter().position(|&n| n == node)
            }
        };

        // Adjust G matrix: remove default pot conductance, add position-dependent
        for pot in &pot_infos {
            let e = &graph.edges[pot.edge_idx];
            let n1 = node_to_mna(e.node_a);
            let n2 = node_to_mna(e.node_b);

            // Delta conductance: new - old
            let old_g = 1.0 / pot.max_r;
            let pos = dim_positions[pot.dim];
            let new_r = (pos * pot.max_r).max(1.0);
            let new_g = 1.0 / new_r;
            let delta_g = new_g - old_g;

            // Apply delta to G matrix (same pattern as stamp_resistor)
            if let Some(p) = n1 {
                mna.g_matrix[p * mna.num_nodes + p] += delta_g;
                if let Some(q) = n2 {
                    mna.g_matrix[p * mna.num_nodes + q] -= delta_g;
                }
            }
            if let Some(q) = n2 {
                mna.g_matrix[q * mna.num_nodes + q] += delta_g;
                if let Some(p) = n1 {
                    mna.g_matrix[q * mna.num_nodes + p] -= delta_g;
                }
            }
        }

        // Extract biquad from modified MNA
        let feedback = extract_feedback_r(edge_indices, graph);
        let biquad = mna.build_iir(
            cap_stamps,
            vs_idx,
            out_mna,
            None,
            sample_rate,
            feedback.as_ref().map(|p| (p.rf, p.r_crit, p.f0)),
        );

        if let Some((b, a)) = biquad {
            coeffs.push(b.get(0).copied().unwrap_or(0.0));
            coeffs.push(b.get(1).copied().unwrap_or(0.0));
            coeffs.push(b.get(2).copied().unwrap_or(0.0));
            coeffs.push(a.get(1).copied().unwrap_or(0.0));
            coeffs.push(a.get(2).copied().unwrap_or(0.0));
        } else {
            // Fallback: try state-space
            let (a_d, b_d, c_out, n_states, d_ft) =
                mna.build_state_space_matrices(cap_stamps, vs_idx, out_mna, None, sample_rate);
            if n_states <= 2 && n_states > 0 {
                let has_two = b_d.len() >= 2 * n_states;
                let bp = &b_d[..n_states];
                let bm = if has_two {
                    &b_d[n_states..2 * n_states]
                } else {
                    bp
                };

                if n_states == 1 {
                    let a_val = a_d[0];
                    let b0 = c_out[0] * bp[0] + d_ft;
                    let b1 = c_out[0] * bm[0] - d_ft * a_val;
                    coeffs.extend_from_slice(&[b0, b1, 0.0, -a_val, 0.0]);
                } else {
                    let a11 = a_d[0];
                    let a12 = a_d[1];
                    let a21 = a_d[2];
                    let a22 = a_d[3];
                    let da1 = -(a11 + a22);
                    let da2 = a11 * a22 - a12 * a21;
                    let nz1p = c_out[0] * bp[0] + c_out[1] * bp[1];
                    let nz0p = c_out[0] * (-a22 * bp[0] + a12 * bp[1])
                        + c_out[1] * (a21 * bp[0] - a11 * bp[1]);
                    let nz1m = c_out[0] * bm[0] + c_out[1] * bm[1];
                    let nz0m = c_out[0] * (-a22 * bm[0] + a12 * bm[1])
                        + c_out[1] * (a21 * bm[0] - a11 * bm[1]);
                    let b0 = nz1p + d_ft;
                    let b1 = nz0p + nz1m + d_ft * da1;
                    let b2 = nz0m + d_ft * da2;
                    coeffs.extend_from_slice(&[b0, b1, b2, da1, da2]);
                }
            } else {
                // Can't extract — fill with passthrough
                coeffs.extend_from_slice(&[1.0, 0.0, 0.0, 0.0, 0.0]);
            }
        }
    }

    Some(BiquadTable {
        steps,
        dim_labels: control_labels.to_vec(),
        coeffs,
    })
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
