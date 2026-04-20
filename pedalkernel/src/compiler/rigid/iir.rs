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

    // ── Step 4: Build IIR biquad ────────────────────────────────────
    // Try direct IIR first (works for oscillators with feedback_r)
    if let Some((b_coeffs, a_coeffs)) =
        mna.build_iir(&cap_stamps, vs_idx, out_mna, None, sample_rate, None)
    {
        return Ok(IirData::new(b_coeffs, a_coeffs, sample_rate));
    }

    // Fallback: reduce to state-space, then extract biquad if 2nd order
    let (a_d, b_d, c_out, n_states, d_feedthrough) =
        mna.build_state_space_matrices(&cap_stamps, vs_idx, out_mna, None, sample_rate);

    if n_states == 2 {
        // 2×2 state-space → biquad directly
        // A = [[a11,a12],[a21,a22]], characteristic poly: z² - tr(A)z + det(A)
        let a11 = a_d[0];
        let a12 = a_d[1];
        let a21 = a_d[2];
        let a22 = a_d[3];
        let tr_a = a11 + a22;
        let det_a = a11 * a22 - a12 * a21;

        // Denominator: 1, -tr(A), det(A)
        let da1 = -tr_a;
        let da2 = det_a;

        // Numerator from transfer function:
        // H(z) = c·(zI-A)⁻¹·b + d
        // For z⁻¹ form: b0 = d + c·b, b1 = c·A·b - d·tr(A), b2 = d·det(A) - c·adj(A)·b·??
        // Simpler: evaluate H(z) at z=0,1,-1 and solve for b0,b1,b2
        // H(z) = (b0 + b1/z + b2/z²) / (1 + a1/z + a2/z²)
        //       = (b0·z² + b1·z + b2) / (z² + a1·z + a2)
        //
        // Direct: b0 = d_feedthrough
        //         and state-space → transfer function via matrix ops
        let cb0 = c_out[0] * b_d[0] + c_out[1] * b_d[1]; // c·b
        // c·A·b
        let ab0 = a11 * b_d[0] + a12 * b_d[1];
        let ab1 = a21 * b_d[0] + a22 * b_d[1];
        let cab = c_out[0] * ab0 + c_out[1] * ab1;

        let b0 = d_feedthrough + cb0;
        let b1 = cab + d_feedthrough * da1;
        let b2 = d_feedthrough * da2
            + c_out[0] * (a11 * ab0 + a12 * ab1)
            + c_out[1] * (a21 * ab0 + a22 * ab1)
            - cb0 * tr_a;

        // Hmm, this is getting complicated. Let me use the simpler approach:
        // Just wrap in StateSpaceData and let that process. But the user wants IIR...
        //
        // Actually, the simplest correct approach: use the state-space process
        // but wrap it in IirData format. The 2×2 state-space IS equivalent
        // to a biquad. Let me just use evaluation at 3 points.

        // Evaluate H(z) at z=1 (DC), z=-1 (Nyquist), z=e^(jπ/2) (quarter rate)
        // to get 3 equations for b0, b1, b2.
        // Actually, for a proper z-transform extraction:
        //
        // From y = c·x + d·u with x = A·x_prev + b·u:
        // Y(z) = c·(zI-A)⁻¹·b·U(z) + d·U(z)
        // H(z) = c·(zI-A)⁻¹·b + d
        //
        // (zI-A)⁻¹ = adj(zI-A) / det(zI-A)
        // det(zI-A) = z² - tr(A)·z + det(A)
        // adj(zI-A) = [[z-a22, a12], [a21, z-a11]]
        //
        // c·adj(zI-A)·b = c0·((z-a22)·b0 + a12·b1) + c1·(a21·b0 + (z-a11)·b1)
        //               = (c0·b0 + c1·b1)·z + (c1·a21·b0 - c0·a22·b0 + c0·a12·b1 - c1·a11·b1)
        //
        // So numerator = d·(z² + a1·z + a2) + (c0·b0+c1·b1)·z + (c1·a21·b0 - c0·a22·b0 + c0·a12·b1 - c1·a11·b1)

        let num_z1 = c_out[0] * b_d[0] + c_out[1] * b_d[1]; // coefficient of z
        let num_z0 = c_out[0] * (-a22 * b_d[0] + a12 * b_d[1])
            + c_out[1] * (a21 * b_d[0] - a11 * b_d[1]); // constant term

        // H(z) = [d·z² + (d·a1 + num_z1)·z + (d·a2 + num_z0)] / [z² + a1·z + a2]
        let b0_iir = d_feedthrough;
        let b1_iir = d_feedthrough * da1 + num_z1;
        let b2_iir = d_feedthrough * da2 + num_z0;

        #[cfg(test)]
        eprintln!(
            "IIR biquad: b=[{b0_iir:.6e}, {b1_iir:.6e}, {b2_iir:.6e}] a=[1, {da1:.6e}, {da2:.6e}] n_states={n_states} d={d_feedthrough:.6e}"
        );
        #[cfg(test)]
        eprintln!(
            "  A=[[{a11:.6e},{a12:.6e}],[{a21:.6e},{a22:.6e}]] b=[{:.6e},{:.6e}] c=[{:.6e},{:.6e}]",
            b_d[0], b_d[1], c_out[0], c_out[1]
        );

        return Ok(IirData::new(
            vec![b0_iir, b1_iir, b2_iir],
            vec![1.0, da1, da2],
            sample_rate,
        ));
    }

    Err(format!(
        "IIR: circuit has {} states (need ≤2 for biquad). Consider StateSpace.",
        n_states
    ))
}
