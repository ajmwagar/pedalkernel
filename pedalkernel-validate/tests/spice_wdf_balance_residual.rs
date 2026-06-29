//! 1:1 SPICE-vs-WDF MultiNl **balance-residual** diagnostic.
//!
//! The grouped Newton-Raphson enforces, per NL port i:
//!     a[i] = known_a[i] + Σ_j s_nl[i][j]·b[j]
//! where a[i]=v[i]+Rp[i]·i(v), b[i]=v[i]−Rp[i]·i(v), i = device port current,
//! and  known_a[i] = dc_bias[i] + Σ_k s_nl_passive[i][k]·b_passive[k]  (input=0).
//!
//! At the TRUE operating point this must hold exactly. We plug SPICE's `.op`
//! into the WDF's own S/Rp/dc_bias (+ the DC-settled cap states) and read the
//! residual per port. We also dump the WDF's OWN settled op-point (v_prev) and
//! check the residual there — that MUST be ~0 (sanity on the computation). A
//! large residual at SPICE's point with ~0 at the WDF's point means the WDF
//! converged to a different operating point than physics; the per-port residual
//! and the v_spice-vs-v_wdf gap localize exactly where.

use pedalkernel::compiler::{compile_pedal_with_options, CompileOptions};
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use pedalkernel_rt::elements::nonlinear::solver::NlDeviceGroupIv;
use pedalkernel_rt::processor::Stage;

const SR: f64 = 96_000.0;

#[test]
fn spice_wdf_balance_residual() {
    let source = include_str!("../circuits/active/xref_feedback_amp.pedal");
    let def = parse_pedal_file(source).expect("parse xref_feedback_amp.pedal");
    let opts = CompileOptions {
        skip_blockwise: true,
        ..CompileOptions::default()
    };
    let mut proc = compile_pedal_with_options(&def, SR, opts).expect("compile");

    let mnl_idx = proc
        .stages
        .iter()
        .position(|st| matches!(st, Stage::MultiNl(_)))
        .expect("expected a MultiNl stage");

    // Settle to DC steady state (input = 0) so the cap ports reach b_passive_DC.
    for _ in 0..8000 {
        proc.process(0.0);
    }

    let mnl = match &proc.stages[mnl_idx] {
        Stage::MultiNl(m) => m,
        _ => unreachable!(),
    };
    let n = mnl.n_nl;
    let n_passive = mnl.scattering.s_nl_passive.len() / n;
    let rp = &mnl.nl_port_resistances;
    let s = &mnl.scattering.s_nl;
    let s_pass = &mnl.scattering.s_nl_passive;
    let dc_bias = &mnl.dc_bias;
    let dg = mnl.device_groups.as_ref().expect("device_groups");

    // b_passive at the settled DC state.
    let mut b_passive = vec![0.0f64; n_passive];
    for (k, op) in mnl.passive_one_ports.iter().enumerate() {
        b_passive[k] = op.wdf_reflected(&mnl.passive_runtime_state);
    }
    // known_a[i] = dc_bias[i] + Σ_k s_nl_passive[i][k]·b_passive[k]   (input=0)
    let known_a: Vec<f64> = (0..n)
        .map(|i| {
            dc_bias[i]
                + (0..n_passive)
                    .map(|k| s_pass[i * n_passive + k] * b_passive[k])
                    .sum::<f64>()
        })
        .collect();

    // Per-port residual at a given set of port voltages.
    let residual_at = |v: &[f64]| -> (Vec<f64>, Vec<f64>) {
        let mut i_dev = vec![0.0f64; n];
        for (g, group) in dg.groups.iter().enumerate() {
            let giv = group.as_group_iv();
            let np = giv.n_ports();
            let off = dg.offsets[g];
            let mut cur = vec![0.0f64; np];
            let mut jac = vec![0.0f64; np * np];
            giv.eval(&v[off..off + np], &mut cur, &mut jac);
            for p in 0..np {
                i_dev[off + p] = cur[p];
            }
        }
        let a: Vec<f64> = (0..n).map(|i| v[i] + rp[i] * i_dev[i]).collect();
        let b: Vec<f64> = (0..n).map(|i| v[i] - rp[i] * i_dev[i]).collect();
        let r: Vec<f64> = (0..n)
            .map(|i| known_a[i] + (0..n).map(|j| s[i * n + j] * b[j]).sum::<f64>() - a[i])
            .collect();
        (r, i_dev)
    };

    // SPICE .op (original xref_feedback_amp, caps OPEN at DC):
    //   nb1=3.155869 nc1=2.528017 ne1=2.459444 nout=13.84149 ne2=1.849139
    let (nb1, nc1, ne1, nout, ne2) = (3.155869, 2.528017, 2.459444, 13.841490, 1.849139);
    let v_spice = [nb1 - ne1, nc1 - ne1, nc1 - ne2, nout - ne2];
    let v_wdf = mnl.v_prev.clone();

    eprintln!("\n========== SPICE vs WDF balance residual (xref_feedback_amp) ==========");
    eprintln!("n_nl={n}  n_passive={n_passive}");
    eprintln!("Rp           = {rp:.2?}");
    eprintln!("dc_bias      = {dc_bias:.4?}");
    eprintln!("b_passive_DC = {b_passive:.4?}");
    eprintln!("known_a(full)= {known_a:.4?}   (= dc_bias + Σ s_nl_passive·b_passive)");
    eprintln!("s_nl:");
    for i in 0..n {
        eprintln!("   row{i}: {:+.4?}", &s[i * n..i * n + n]);
    }
    // Rosetta stone: dump every port's (pos,neg) MNA node ids. NL ports 0..3 map
    // to known SPICE nodes (nb1/nc1/ne1/ne2/nout), so the passive ports' node ids
    // decode to SPICE cap voltages.
    if let Some(rd) = &mnl.recompute_data {
        eprintln!("\nport_node_pairs (pos,neg) per port:");
        for (i, p) in rd.port_node_pairs.iter().enumerate() {
            eprintln!("  port {i}: ({:?}, {:?})", p.pos, p.neg);
        }
    } else {
        eprintln!("\n(recompute_data = None)");
    }
    eprintln!("passive_one_ports terminals:");
    for (k, op) in mnl.passive_one_ports.iter().enumerate() {
        eprintln!("  passive {k}: ({:?}, {:?})", op.spec.terminals.pos, op.spec.terminals.neg);
    }

    eprintln!("\nv_SPICE (Vbe1,Vce1,Vbe2,Vce2) = {v_spice:.5?}");
    eprintln!("v_WDF   (settled v_prev)      = {v_wdf:.5?}");

    let (r_wdf, i_wdf) = residual_at(&v_wdf);
    eprintln!("\n--- residual at the WDF's OWN settled op-point (sanity; must be ~0) ---");
    eprintln!("  i(v_wdf) = {i_wdf:?}");
    for i in 0..n {
        eprintln!("  port {i}:  R = {:+.4e}", r_wdf[i]);
    }

    let (r_sp, i_sp) = residual_at(&v_spice);
    eprintln!("\n--- residual at SPICE's TRUE op-point (the test) ---");
    eprintln!("  i(v_spice) = {i_sp:?}");
    let mut max_r = 0.0f64;
    for i in 0..n {
        max_r = max_r.max(r_sp[i].abs());
        eprintln!("  port {i}:  R = {:+.4e}", r_sp[i]);
    }
    eprintln!("\nmax |residual at SPICE op-point| = {max_r:.4e}");
}
