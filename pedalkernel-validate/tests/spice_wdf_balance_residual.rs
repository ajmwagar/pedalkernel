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

#[test]
fn falsify_node_merge_rf_sweep() {
    // Prior (suspect) claim: Q1.base == Q2.collector, i.e. Rf is shorted/merged.
    // If TRUE, changing Rf 33k->100k is inert (Rf absent from the network).
    // If Rf is a real resistor, dc_bias/S entries tied to that node move with Rf.
    let src33 = include_str!("../circuits/active/xref_feedback_amp.pedal");
    let src100 = src33.replace("33k", "100k");
    assert_ne!(src33, src100, "Rf 33k must be present to replace");
    let dump = |src: &str| -> (Vec<f64>, Vec<f64>) {
        let def = parse_pedal_file(src).expect("parse");
        let opts = CompileOptions { skip_blockwise: true, ..CompileOptions::default() };
        let p = compile_pedal_with_options(&def, SR, opts).expect("compile");
        let m = p.stages.iter().find_map(|st| match st { Stage::MultiNl(m) => Some(m), _ => None }).expect("mnl");
        (m.dc_bias.clone(), m.scattering.s_nl.clone())
    };
    let (db33, s33) = dump(src33);
    let (db100, s100) = dump(&src100);
    eprintln!("\n=== Rf falsification: 33k vs 100k ===");
    eprintln!("dc_bias  @33k  = {db33:.4?}");
    eprintln!("dc_bias  @100k = {db100:.4?}");
    eprintln!("s_nl[0][3] (port0<->port3 coupling): 33k={:.6}  100k={:.6}", s33[3], s100[3]);
    eprintln!("s_nl[3][0]:                          33k={:.6}  100k={:.6}", s33[12], s100[12]);
    let dmax = db33.iter().zip(&db100).map(|(a,b)| (a-b).abs()).fold(0.0f64, f64::max);
    let smax = s33.iter().zip(&s100).map(|(a,b)| (a-b).abs()).fold(0.0f64, f64::max);
    eprintln!("max|Δdc_bias|={dmax:.4}  max|Δs_nl|={smax:.6}");
    eprintln!("(both ~0 => Rf NOT in network => node-merge claim TRUE; nonzero => Rf real => claim FALSE)");
}

#[test]
fn stage_shape_feedback_dc() {
    let source = include_str!("../circuits/active/xref_feedback_dc.pedal");
    let def = parse_pedal_file(source).expect("parse");
    let opts = CompileOptions { skip_blockwise: true, ..CompileOptions::default() };
    let p = compile_pedal_with_options(&def, SR, opts).expect("compile");
    eprintln!("\n=== xref_feedback_dc stages (skip_blockwise) ===");
    for (i, st) in p.stages.iter().enumerate() {
        let kind = match st {
            Stage::MultiNl(m) => format!("MultiNl(n_nl={}, n_passive={})", m.n_nl, m.scattering.s_nl_passive.len()/m.n_nl.max(1)),
            Stage::Wdf(_) => "Wdf".into(),
            Stage::Iir(_) => "Iir".into(),
            Stage::StateSpace(_) => "StateSpace".into(),
            Stage::Blockwise(_) => "Blockwise".into(),
            _ => "other".into(),
        };
        eprintln!("  stage {i}: {kind}");
    }
}

#[test]
fn measure_gain_xref() {
    let source = include_str!("../circuits/active/xref_feedback_amp.pedal");
    let def = parse_pedal_file(source).expect("parse");
    let opts = CompileOptions { skip_blockwise: true, ..CompileOptions::default() };
    let mut proc = compile_pedal_with_options(&def, SR, opts).expect("compile");
    for _ in 0..8000 { proc.process(0.0); }
    let (f, amp, n) = (1000.0f64, 0.01f64, 19200usize);
    let (mut si, mut so) = (0.0f64, 0.0f64);
    for k in 0..n {
        let t = k as f64 / SR;
        let x = amp * (2.0 * std::f64::consts::PI * f * t).sin();
        let y = proc.process(x);
        if k >= n/2 { si += x*x; so += y*y; }
    }
    let rin = (si/(n/2) as f64).sqrt();
    let rout = (so/(n/2) as f64).sqrt();
    eprintln!("\n=== xref WDF gain (VBC_MAX removed) ===");
    eprintln!("rms_in={rin:.6} rms_out={rout:.6}  gain = {:.2} dB   (SPICE AC ~ +18.7 dB)", 20.0*(rout/rin).log10());
}

#[test]
fn clean_spice_substituted_residual() {
    let source = include_str!("../circuits/active/xref_feedback_amp.pedal");
    let def = parse_pedal_file(source).expect("parse");
    let opts = CompileOptions { skip_blockwise: true, ..CompileOptions::default() };
    let proc = compile_pedal_with_options(&def, SR, opts).expect("compile");
    let mnl = proc.stages.iter().find_map(|st| match st { Stage::MultiNl(m) => Some(m), _ => None }).expect("mnl");
    let n = mnl.n_nl;
    let n_passive = mnl.scattering.s_nl_passive.len() / n;
    let rp = &mnl.nl_port_resistances;
    let s = &mnl.scattering.s_nl;
    let s_pass = &mnl.scattering.s_nl_passive;
    let dc_bias = &mnl.dc_bias;

    // VERIFIED SPICE .op + measured port currents (ngspice on the runnable deck)
    let (nb1, nc1, ne1, nout, ne2) = (3.155869, 2.528017, 2.459444, 13.841490, 1.849139);
    let v_spice = [nb1 - ne1, nc1 - ne1, nc1 - ne2, nout - ne2];
    let i_spice = [3.238068e-4, 2.135638e-3, 1.156071e-5, 1.837578e-3]; // Ib1,Ic1,Ib2,Ic2

    let (v_cf, v_cout, v_ce1, v_ce2) = (nout - nb1, nout, ne1, ne2); // a=b=v at DC

    let pnp = mnl.recompute_data.as_ref().map(|r| r.port_node_pairs.clone());
    let mut b_passive = vec![0.0f64; n_passive];
    for (k, op) in mnl.passive_one_ports.iter().enumerate() {
        let f = if let pedalkernel_rt::boundary_math::OnePortKind::Capacitor(f) = &op.spec.kind { *f as f64 } else { -1.0 };
        let pos = pnp.as_ref().and_then(|p| p.get(n + k)).and_then(|t| t.pos);
        let (label, val) = if f > 50e-6 { if pos == Some(1) { ("CE1(ne1)", v_ce1) } else { ("CE2(ne2)", v_ce2) } }
            else if f > 1e-6 { ("Cout", v_cout) } else { ("Cf", v_cf) };
        b_passive[k] = val;
        eprintln!("passive {k}: C={f:.3e}F pos={pos:?} -> {label} = {val:.4} V", );
    }

    let a_phys: Vec<f64> = (0..n).map(|i| v_spice[i] + rp[i] * i_spice[i]).collect();
    let b_phys: Vec<f64> = (0..n).map(|i| v_spice[i] - rp[i] * i_spice[i]).collect();
    let cap_term: Vec<f64> = (0..n).map(|i| (0..n_passive).map(|k| s_pass[i * n_passive + k] * b_passive[k]).sum()).collect();
    let nl_term: Vec<f64> = (0..n).map(|i| (0..n).map(|j| s[i * n + j] * b_phys[j]).sum()).collect();
    let r: Vec<f64> = (0..n).map(|i| a_phys[i] - dc_bias[i] - cap_term[i] - nl_term[i]).collect();

    let lbl = ["Q1.Vbe", "Q1.Vce", "Q2.Vbe", "Q2.Vce"];
    eprintln!("\n=== CLEAN SPICE-substituted residual (no companion, SPICE caps) ===");
    eprintln!("Rp       = {rp:.1?}");
    eprintln!("a_phys   = {a_phys:.4?}");
    eprintln!("dc_bias  = {dc_bias:.4?}");
    eprintln!("cap_term = {cap_term:.4?}   (Σ s_nl_passive·b_passive)");
    eprintln!("nl_term  = {nl_term:.4?}   (Σ s_nl·b_phys)");
    for i in 0..n { eprintln!("  port {i} {:8}: r = {:+.4} V", lbl[i], r[i]); }
    eprintln!("max|r| = {:.4} V", r.iter().cloned().fold(0.0f64, |a, x| a.max(x.abs())));
}

#[test]
fn check3_init_at_spice_op_point_and_step() {
    let source = include_str!("../circuits/active/xref_feedback_amp.pedal");
    let def = parse_pedal_file(source).expect("parse");
    let opts = CompileOptions { skip_blockwise: true, ..CompileOptions::default() };
    let mut proc = compile_pedal_with_options(&def, SR, opts).expect("compile");
    let idx = proc.stages.iter().position(|s| matches!(s, Stage::MultiNl(_))).unwrap();

    let (nb1, nc1, ne1, nout, ne2) = (3.155869, 2.528017, 2.459444, 13.841490, 1.849139);
    let v_spice = [nb1 - ne1, nc1 - ne1, nc1 - ne2, nout - ne2];
    let (v_cf, v_cout, v_ce1, v_ce2) = (nout - nb1, nout, ne1, ne2);

    {
        let mnl = match &mut proc.stages[idx] { Stage::MultiNl(m) => m, _ => unreachable!() };
        let n = mnl.n_nl;
        let pnp = mnl.recompute_data.as_ref().map(|r| r.port_node_pairs.clone());
        let np = mnl.passive_one_ports.len();
        let mut capval = vec![0.0f64; np];
        for k in 0..np {
            let f = if let pedalkernel_rt::boundary_math::OnePortKind::Capacitor(f) = &mnl.passive_one_ports[k].spec.kind { *f as f64 } else { -1.0 };
            let pos = pnp.as_ref().and_then(|p| p.get(n + k)).and_then(|t| t.pos);
            capval[k] = if f > 50e-6 { if pos == Some(1) { v_ce1 } else { v_ce2 } } else if f > 1e-6 { v_cout } else { v_cf };
        }
        for k in 0..np {
            mnl.passive_one_ports[k].wdf_set_incident(capval[k], &mut mnl.passive_runtime_state);
        }
        mnl.v_prev = v_spice.to_vec();
    }

    eprintln!("\n=== Check #3: seed at SPICE op-point (NL + caps), then step ===");
    eprintln!("SPICE physical = [0.6964, 0.0686, 0.6789, 11.9924]  (Q1 sat / Q2 active)");
    eprintln!("mirror (wrong) = [0.47,   17.27,  0.73,   0.086]    (Q1 cutoff / Q2 sat)");
    let v0 = match &proc.stages[idx] { Stage::MultiNl(m) => m.v_prev.clone(), _ => unreachable!() };
    eprintln!("seeded  step    0: v_prev = {v0:.4?}");
    let milestones = [1usize, 2, 5, 20, 100, 1000, 8000];
    let mut done = 0;
    for &m in &milestones {
        while done < m { proc.process(0.0); done += 1; }
        let v = match &proc.stages[idx] { Stage::MultiNl(mm) => mm.v_prev.clone(), _ => unreachable!() };
        eprintln!("after   step {done:5}: v_prev = {v:.4?}");
    }
}

#[test]
fn cout_sweep_dc_bias() {
    let base = include_str!("../circuits/active/xref_feedback_amp.pedal");
    eprintln!("\n=== Cout sweep: does the output cap's WDF-short starve Q2.Vce's VCC injection? ===");
    eprintln!("(dc_bias port order: [Q1.Vbe, Q1.Vce, Q2.Vbe, Q2.Vce]; physics wants Q2.Vce ~ +12)");
    for (label, repl) in [
        ("Cout=10u (orig)", "Cout: cap(10u, electrolytic)"),
        ("Cout=100n",       "Cout: cap(100n)"),
        ("Cout=1n",         "Cout: cap(1n)"),
        ("Cout=1p",         "Cout: cap(1p)"),
    ] {
        let src = base.replace("Cout: cap(10u, electrolytic)", repl);
        let def = parse_pedal_file(&src).expect("parse");
        let opts = CompileOptions { skip_blockwise: true, ..CompileOptions::default() };
        let p = compile_pedal_with_options(&def, SR, opts).expect("compile");
        match p.stages.iter().find_map(|s| match s { Stage::MultiNl(m) => Some(m), _ => None }) {
            Some(m) => eprintln!("  {label:16}: dc_bias = {:.3?}  n_passive={}", m.dc_bias, m.scattering.s_nl_passive.len()/m.n_nl),
            None => eprintln!("  {label:16}: (no MultiNl stage)"),
        }
    }
}

#[test]
fn ce_sweep_and_passive_coupling() {
    let base = include_str!("../circuits/active/xref_feedback_amp.pedal");
    let comp = |src: &str| {
        let def = parse_pedal_file(src).expect("parse");
        let opts = CompileOptions { skip_blockwise: true, ..CompileOptions::default() };
        compile_pedal_with_options(&def, SR, opts).expect("compile")
    };
    // baseline: dump Rp + full s_nl_passive (rows=NL ports, cols=passive caps)
    let p = comp(base);
    let m = p.stages.iter().find_map(|s| match s { Stage::MultiNl(m) => Some(m), _ => None }).unwrap();
    let n = m.n_nl; let np = m.scattering.s_nl_passive.len()/n;
    eprintln!("\n=== baseline Rp + s_nl_passive ===");
    eprintln!("Rp = {:.1?}", m.nl_port_resistances);
    eprintln!("dc_bias = {:.3?}", m.dc_bias);
    let lbl=["Q1.Vbe","Q1.Vce","Q2.Vbe","Q2.Vce"];
    for i in 0..n { eprintln!("  s_nl_passive[{}]: {:+.4?}", lbl[i], &m.scattering.s_nl_passive[i*np..i*np+np]); }
    eprintln!("(passive col order = compile order of caps)");

    eprintln!("\n=== C/E (emitter bypass) sweeps — watch dc_bias[Q2.Vce]=idx3 ===");
    for (label, find, repl) in [
        ("CE2 220u->1p", "CE2:  cap(220u, electrolytic)", "CE2:  cap(1p)"),
        ("CE1 220u->1p", "CE1:  cap(220u, electrolytic)", "CE1:  cap(1p)"),
        ("RL 10k->100meg","RL:   resistor(10k)", "RL:   resistor(10M)"),
        ("RC2 4k7->47",   "RC2:  resistor(4k7)", "RC2:  resistor(1k)"),
    ] {
        let src = base.replace(find, repl);
        let changed = src != base;
        let pp = comp(&src);
        match pp.stages.iter().find_map(|s| match s { Stage::MultiNl(m) => Some(m), _ => None }) {
            Some(mm) => eprintln!("  {label:16} (applied={changed}): dc_bias = {:.3?}", mm.dc_bias),
            None => eprintln!("  {label:16}: no MultiNl"),
        }
    }
}

#[test]
fn collector_rp_mechanism() {
    let base = include_str!("../circuits/active/xref_feedback_amp.pedal");
    let comp = |src: &str| -> (Vec<f64>, Vec<f64>) {
        let def = parse_pedal_file(src).expect("parse");
        let opts = CompileOptions { skip_blockwise: true, ..CompileOptions::default() };
        let p = compile_pedal_with_options(&def, SR, opts).expect("compile");
        let m = p.stages.iter().find_map(|s| match s { Stage::MultiNl(m) => Some(m), _ => None }).unwrap();
        (m.nl_port_resistances.clone(), m.dc_bias.clone())
    };
    let mk = |reps: &[(&str, &str)]| { let mut s = base.to_string(); for (a, b) in reps { s = s.replace(a, b); } s };
    let ce1  = ("CE1:  cap(220u, electrolytic)", "CE1:  cap(1p)");
    let ce2  = ("CE2:  cap(220u, electrolytic)", "CE2:  cap(1p)");
    let cout = ("Cout: cap(10u, electrolytic)", "Cout: cap(1p)");
    let cf   = ("cap(100p)", "cap(1p)");
    let cases: Vec<(&str, String)> = vec![
        ("baseline", base.to_string()),
        ("Cout->1p", mk(&[cout])),
        ("Cf->1p", mk(&[cf])),
        ("Cout+Cf->1p", mk(&[cout, cf])),
        ("CE2->1p", mk(&[ce2])),
        ("CE1+CE2->1p", mk(&[ce1, ce2])),
        ("CE1+CE2+Cout+Cf->1p", mk(&[ce1, ce2, cout, cf])),
    ];
    eprintln!("\n=== Rp + dc_bias vs cap-short removal (SPICE wants Q2.Vce: dc_bias~12, Rp~? ) ===");
    eprintln!("Q1.Vce (port1) sits at 0.069V sat;  Q2.Vce (port3) at 11.99V active");
    for (label, src) in &cases {
        let (rp, dc) = comp(src);
        eprintln!("  {label:22}  Rp1={:8.1}  Rp3={:8.1}   dcb1={:8.3}  dcb3={:8.3}", rp[1], rp[3], dc[1], dc[3]);
    }
}

#[test]
fn output_load_mechanism() {
    let base = include_str!("../circuits/active/xref_feedback_amp.pedal");
    let comp = |src: &str| -> (Vec<f64>, Vec<f64>) {
        let def = parse_pedal_file(src).expect("parse");
        let opts = CompileOptions { skip_blockwise: true, ..CompileOptions::default() };
        let p = compile_pedal_with_options(&def, SR, opts).expect("compile");
        let m = p.stages.iter().find_map(|s| match s { Stage::MultiNl(m) => Some(m), _ => None }).unwrap();
        (m.nl_port_resistances.clone(), m.dc_bias.clone())
    };
    let mk = |reps: &[(&str, &str)]| { let mut s = base.to_string(); for (a, b) in reps { s = s.replace(a, b); } s };
    let ce  = [("CE1:  cap(220u, electrolytic)", "CE1:  cap(1p)"), ("CE2:  cap(220u, electrolytic)", "CE2:  cap(1p)")];
    let rl  = ("RL:   resistor(10k)", "RL:   resistor(10M)");
    let rc2 = ("RC2:  resistor(4k7)", "RC2:  resistor(1k)");
    let cases: Vec<(&str, String)> = vec![
        ("baseline",                base.to_string()),
        ("RL->100meg",              mk(&[rl])),
        ("RL->100meg +CE1,2->1p",   mk(&[rl, ce[0], ce[1]])),
        ("RC2->470",                mk(&[rc2])),
        ("RC2->470 +CE1,2->1p",     mk(&[rc2, ce[0], ce[1]])),
        ("RL->100meg +RC2->470 +CE1,2->1p", mk(&[rl, rc2, ce[0], ce[1]])),
    ];
    eprintln!("\n=== output-load / collector-R effect on Q2.Vce (port 3) — does dcb3 reach ~12 & Rp3 ~ Rp1? ===");
    for (label, src) in &cases {
        let (rp, dc) = comp(src);
        eprintln!("  {label:34}  Rp1={:8.1}  Rp3={:8.1}   dcb1={:8.3}  dcb3={:8.3}", rp[1], rp[3], dc[1], dc[3]);
    }
}

#[test]
fn rc1_rc2_node_routing() {
    let base = include_str!("../circuits/active/xref_feedback_amp.pedal");
    let comp = |src: &str| -> Vec<f64> {
        let def = parse_pedal_file(src).expect("parse");
        let opts = CompileOptions { skip_blockwise: true, ..CompileOptions::default() };
        let p = compile_pedal_with_options(&def, SR, opts).expect("compile");
        p.stages.iter().find_map(|s| match s { Stage::MultiNl(m) => Some(m.dc_bias.clone()), _ => None }).unwrap()
    };
    eprintln!("\n=== which collector resistor moves which Vce port's dc_bias? ===");
    eprintln!("RC1 feeds nc1 (Q1 collector). RC2 feeds nout (Q2 collector).");
    eprintln!("baseline       dc_bias[Q1.Vce, Q2.Vce] = [{:.3}, {:.3}]", comp(base)[1], comp(base)[3]);
    let rc1 = comp(&base.replace("RC1:  resistor(10k)", "RC1:  resistor(1k)"));
    eprintln!("RC1 10k->1k    dc_bias[Q1.Vce, Q2.Vce] = [{:.3}, {:.3}]", rc1[1], rc1[3]);
    let rc2 = comp(&base.replace("RC2:  resistor(4k7)", "RC2:  resistor(1k)"));
    eprintln!("RC2 4k7->1k    dc_bias[Q1.Vce, Q2.Vce] = [{:.3}, {:.3}]", rc2[1], rc2[3]);
}

#[test]
fn falsify_shared_node_swap() {
    let base = include_str!("../circuits/active/xref_feedback_amp.pedal");
    // Split nc1: interpose Rlink(100) so Q1.collector and Q2.base are DISTINCT nodes.
    let split = base
        .replace("RC1:  resistor(10k)", "RC1:  resistor(10k)\n    Rlink: resistor(100)")
        .replace("RC1.b -> Q1.collector, Q2.base",
                 "RC1.b -> Q1.collector\n    Rlink.a -> Q1.collector\n    Rlink.b -> Q2.base");
    assert_ne!(split, base, "split must apply");
    let dcb = |src: &str| -> Option<Vec<f64>> {
        let def = parse_pedal_file(src).ok()?;
        let opts = CompileOptions { skip_blockwise: true, ..CompileOptions::default() };
        let p = compile_pedal_with_options(&def, SR, opts).ok()?;
        p.stages.iter().find_map(|s| match s { Stage::MultiNl(m) => Some(m.dc_bias.clone()), _ => None })
    };
    eprintln!("\n=== Falsify shared-node swap: does splitting nc1 kill the RC1/RC2 swap? ===");
    for (label, b) in [("SHARED nc1 (orig)", base.to_string()), ("SPLIT nc1 (Q1.C|Q2.B distinct)", split.clone())] {
        let d0 = dcb(&b);
        let d1 = dcb(&b.replace("RC1:  resistor(10k)", "RC1:  resistor(1k)"));
        let d2 = dcb(&b.replace("RC2:  resistor(4k7)", "RC2:  resistor(1k)"));
        match (d0, d1, d2) {
            (Some(d0), Some(d1), Some(d2)) => {
                eprintln!("\n{label}:");
                eprintln!("  baseline dc_bias[Q1.Vce,Q2.Vce]=[{:7.3},{:7.3}]", d0[1], d0[3]);
                eprintln!("  RC1->1k  dc_bias[Q1.Vce,Q2.Vce]=[{:7.3},{:7.3}]  <- RC1 is Q1's pull-up", d1[1], d1[3]);
                eprintln!("  RC2->1k  dc_bias[Q1.Vce,Q2.Vce]=[{:7.3},{:7.3}]  <- RC2 is Q2's pull-up", d2[1], d2[3]);
            }
            _ => eprintln!("{label}: NO MultiNl (topology changed too much)"),
        }
    }
}

#[test]
fn falsify_shared_node_swap_v2() {
    let base = include_str!("../circuits/active/xref_feedback_amp.pedal");
    let split = base
        .replace("RC1:  resistor(10k)", "RC1:  resistor(10k)\n    Rlink: resistor(100)")
        .replace("RC1.b -> Q1.collector, Q2.base",
                 "RC1.b -> Q1.collector\n    Rlink.a -> Q1.collector\n    Rlink.b -> Q2.base");
    let dcb = |s2: &str| -> Option<Vec<f64>> {
        let d = parse_pedal_file(s2).ok()?;
        let opts = CompileOptions { skip_blockwise: true, ..CompileOptions::default() };
        let p = compile_pedal_with_options(&d, SR, opts).ok()?;
        p.stages.iter().find_map(|s| match s { Stage::MultiNl(m) => Some(m.dc_bias.clone()), _ => None })
    };
    let info = |src: &str, label: &str| {
        let def = parse_pedal_file(src).expect("parse");
        let opts = CompileOptions { skip_blockwise: true, ..CompileOptions::default() };
        let p = compile_pedal_with_options(&def, SR, opts).expect("compile");
        let stages: Vec<String> = p.stages.iter().map(|s| match s {
            Stage::MultiNl(m) => format!("MultiNl(n_nl={})", m.n_nl),
            Stage::Wdf(_) => "Wdf".into(), Stage::Iir(_) => "Iir".into(),
            Stage::StateSpace(_) => "StateSpace".into(), Stage::Blockwise(_) => "Blockwise".into(), _ => "other".into(),
        }).collect();
        eprintln!("\n{label}: stages={stages:?}");
        eprintln!("  baseline dc_bias = {:?}", dcb(src));
        eprintln!("  RC1->1k  dc_bias = {:?}", dcb(&src.replace("RC1:  resistor(10k)", "RC1:  resistor(1k)")));
        eprintln!("  RC2->1k  dc_bias = {:?}", dcb(&src.replace("RC2:  resistor(4k7)", "RC2:  resistor(1k)")));
    };
    eprintln!("\n=== Falsify shared-node swap (v2, full dump) ===");
    info(base, "SHARED nc1");
    info(&split, "SPLIT nc1");
}

#[test]
fn clean_residual_robust() {
    let source = include_str!("../circuits/active/xref_feedback_amp.pedal");
    let def = parse_pedal_file(source).expect("parse");
    let opts = CompileOptions { skip_blockwise: true, ..CompileOptions::default() };
    let proc = compile_pedal_with_options(&def, SR, opts).expect("compile");
    let mnl = proc.stages.iter().find_map(|st| match st { Stage::MultiNl(m) => Some(m), _ => None }).unwrap();
    let n = mnl.n_nl;
    let n_passive = mnl.scattering.s_nl_passive.len() / n;
    let rp = &mnl.nl_port_resistances;
    let s = &mnl.scattering.s_nl;
    let s_pass = &mnl.scattering.s_nl_passive;
    let dc_bias = &mnl.dc_bias;
    let dg = mnl.device_groups.as_ref().unwrap();
    let pnp = mnl.recompute_data.as_ref().unwrap().port_node_pairs.clone();
    // MNA idx -> SPICE node voltage (PK9XU1 node_set + ngspice .op)
    // 0=nout 1=ne2 2=nc1 3=ne1 4=nb1 5=out
    let vmap = |idx: Option<usize>| -> f64 { match idx {
        Some(0)=>13.841490, Some(1)=>1.849139, Some(2)=>2.528017, Some(3)=>2.459444, Some(4)=>3.155869, _=>0.0,
    }};
    let v_spice: Vec<f64> = (0..n).map(|i| vmap(pnp[i].pos) - vmap(pnp[i].neg)).collect();
    let mut i_dev = vec![0.0; n];
    for (g, group) in dg.groups.iter().enumerate() {
        let giv = group.as_group_iv(); let np = giv.n_ports(); let off = dg.offsets[g];
        let mut cur = vec![0.0; np]; let mut jac = vec![0.0; np*np];
        giv.eval(&v_spice[off..off+np], &mut cur, &mut jac);
        for p in 0..np { i_dev[off+p] = cur[p]; }
    }
    let b_passive: Vec<f64> = (0..n_passive).map(|k| vmap(pnp[n+k].pos) - vmap(pnp[n+k].neg)).collect();
    let a: Vec<f64> = (0..n).map(|i| v_spice[i] + rp[i]*i_dev[i]).collect();
    let b: Vec<f64> = (0..n).map(|i| v_spice[i] - rp[i]*i_dev[i]).collect();
    let cap: Vec<f64> = (0..n).map(|i| (0..n_passive).map(|k| s_pass[i*n_passive+k]*b_passive[k]).sum()).collect();
    let nlt: Vec<f64> = (0..n).map(|i| (0..n).map(|j| s[i*n+j]*b[j]).sum()).collect();
    let r: Vec<f64> = (0..n).map(|i| a[i] - dc_bias[i] - cap[i] - nlt[i]).collect();
    eprintln!("\n=== ROBUST residual (terminals -> v, no port-order guess) ===");
    eprintln!("v_spice  = {v_spice:.4?}");
    eprintln!("i_dev    = {i_dev:?}");
    eprintln!("a_phys   = {a:.4?}");
    eprintln!("dc_bias  = {dc_bias:.4?}");
    for i in 0..n { eprintln!("  port {i} (mna {:?},{:?}): r = {:+.4} V", pnp[i].pos, pnp[i].neg, r[i]); }
    eprintln!("max|r| = {:.4} V", r.iter().cloned().fold(0.0f64,|m,x|m.max(x.abs())));
}

#[test]
fn check3_robust() {
    let source = include_str!("../circuits/active/xref_feedback_amp.pedal");
    let def = parse_pedal_file(source).expect("parse");
    let opts = CompileOptions { skip_blockwise: true, ..CompileOptions::default() };
    let mut proc = compile_pedal_with_options(&def, SR, opts).expect("compile");
    let idx = proc.stages.iter().position(|s| matches!(s, Stage::MultiNl(_))).unwrap();
    let vmap = |i: Option<usize>| -> f64 { match i {
        Some(0)=>13.841490,Some(1)=>1.849139,Some(2)=>2.528017,Some(3)=>2.459444,Some(4)=>3.155869,_=>0.0 }};
    let v_spice: Vec<f64>;
    {
        let mnl = match &mut proc.stages[idx] { Stage::MultiNl(m)=>m,_=>unreachable!() };
        let n = mnl.n_nl;
        let pnp = mnl.recompute_data.as_ref().unwrap().port_node_pairs.clone();
        v_spice = (0..n).map(|i| vmap(pnp[i].pos)-vmap(pnp[i].neg)).collect();
        let np = mnl.passive_one_ports.len();
        for k in 0..np {
            let bk = vmap(pnp[n+k].pos) - vmap(pnp[n+k].neg);
            mnl.passive_one_ports[k].wdf_set_incident(bk, &mut mnl.passive_runtime_state);
        }
        mnl.v_prev = v_spice.clone();
    }
    eprintln!("\n=== Check #3 ROBUST: seed TRUE op-point (correct order), step ===");
    eprintln!("seeded v_prev = {v_spice:.4?}  (Q2.Vbe,Q2.Vce,Q1.Vbe,Q1.Vce) [Q2 active, Q1 sat]");
    let mut done = 0;
    for &m in &[1usize,2,5,20,100,1000,8000] {
        while done < m { proc.process(0.0); done += 1; }
        let v = match &proc.stages[idx] { Stage::MultiNl(mm)=>mm.v_prev.clone(),_=>unreachable!() };
        eprintln!("after {done:5}: v_prev = {v:.4?}");
    }
}

#[test]
fn residual_validation_and_breakdown() {
    let source = include_str!("../circuits/active/xref_feedback_amp.pedal");
    let def = parse_pedal_file(source).expect("parse");
    let opts = CompileOptions { skip_blockwise: true, ..CompileOptions::default() };
    let mut proc = compile_pedal_with_options(&def, SR, opts).expect("compile");
    let idx = proc.stages.iter().position(|s| matches!(s, Stage::MultiNl(_))).unwrap();
    for _ in 0..8000 { proc.process(0.0); }
    let mnl = match &proc.stages[idx] { Stage::MultiNl(m)=>m,_=>unreachable!() };
    let n = mnl.n_nl; let n_passive = mnl.scattering.s_nl_passive.len()/n;
    let rp=&mnl.nl_port_resistances; let s=&mnl.scattering.s_nl; let s_pass=&mnl.scattering.s_nl_passive;
    let dc_bias=&mnl.dc_bias; let dg=mnl.device_groups.as_ref().unwrap();
    let pnp = mnl.recompute_data.as_ref().unwrap().port_node_pairs.clone();
    let vmap = |i: Option<usize>| -> f64 { match i {
        Some(0)=>13.841490,Some(1)=>1.849139,Some(2)=>2.528017,Some(3)=>2.459444,Some(4)=>3.155869,_=>0.0 }};
    let resid = |v: &[f64], bp: &[f64]| -> (Vec<f64>,Vec<f64>,Vec<f64>,Vec<f64>) {
        let mut id = vec![0.0;n];
        for (g,grp) in dg.groups.iter().enumerate() { let giv=grp.as_group_iv(); let np=giv.n_ports(); let off=dg.offsets[g];
            let mut c=vec![0.0;np]; let mut j=vec![0.0;np*np]; giv.eval(&v[off..off+np],&mut c,&mut j); for p in 0..np {id[off+p]=c[p];}}
        let a:Vec<f64>=(0..n).map(|i| v[i]+rp[i]*id[i]).collect();
        let b:Vec<f64>=(0..n).map(|i| v[i]-rp[i]*id[i]).collect();
        let cap:Vec<f64>=(0..n).map(|i| (0..n_passive).map(|k| s_pass[i*n_passive+k]*bp[k]).sum()).collect();
        let nlt:Vec<f64>=(0..n).map(|i| (0..n).map(|j| s[i*n+j]*b[j]).sum()).collect();
        let r:Vec<f64>=(0..n).map(|i| a[i]-dc_bias[i]-cap[i]-nlt[i]).collect();
        (a,cap,nlt,r)
    };
    // 1. validation: residual at the WDF's OWN settled fixed point -> must be ~0
    let v_wdf = mnl.v_prev.clone();
    let bp_wdf:Vec<f64>=(0..n_passive).map(|k| mnl.passive_one_ports[k].wdf_reflected(&mnl.passive_runtime_state)).collect();
    let (_,_,_,r_wdf)=resid(&v_wdf,&bp_wdf);
    eprintln!("\n=== VALIDATION: residual at WDF's own settled fixed point (must be ~0) ===");
    eprintln!("  v_wdf={v_wdf:.4?}  max|r|={:.4}", r_wdf.iter().cloned().fold(0.0f64,|m,x|m.max(x.abs())));
    eprintln!("  r_wdf={r_wdf:.4?}");
    // 2. SPICE op-point, full breakdown
    let v_sp:Vec<f64>=(0..n).map(|i| vmap(pnp[i].pos)-vmap(pnp[i].neg)).collect();
    let bp_sp:Vec<f64>=(0..n_passive).map(|k| vmap(pnp[n+k].pos)-vmap(pnp[n+k].neg)).collect();
    let (a,cap,nlt,r)=resid(&v_sp,&bp_sp);
    eprintln!("\n=== SPICE op-point breakdown (port order [Q2.Vbe,Q2.Vce,Q1.Vbe,Q1.Vce]) ===");
    eprintln!("  b_passive(caps) = {bp_sp:.4?}");
    for i in 0..n { eprintln!("  port{i}: a={:+.3} dc_bias={:+.3} cap={:+.3} nl={:+.3}  r={:+.3}", a[i],dc_bias[i],cap[i],nlt[i],r[i]); }
}

#[test]
fn wdf_supply_continuation() {
    let source = include_str!("../circuits/active/xref_feedback_amp.pedal");
    let def = parse_pedal_file(source).expect("parse");
    let mk = || compile_pedal_with_options(&def, SR, CompileOptions{skip_blockwise:true,..CompileOptions::default()}).expect("compile");
    let idx; let orig: Vec<f64>;
    { let p = mk(); idx = p.stages.iter().position(|s| matches!(s, Stage::MultiNl(_))).unwrap();
      orig = match &p.stages[idx] { Stage::MultiNl(m)=>m.dc_bias.clone(), _=>unreachable!() }; }
    // (A) cold start
    let mut pa = mk();
    for _ in 0..30000 { pa.process(0.0); }
    let v_cold = match &pa.stages[idx] { Stage::MultiNl(m)=>m.v_prev.clone(), _=>unreachable!() };
    // (B) supply ramp: dc_bias 0 -> full over a slow ramp, carrying solution forward
    let mut pb = mk();
    let ramp = 150_000usize;
    for k in 0..ramp {
        let alpha = (k as f64 / ramp as f64).min(1.0);
        if let Stage::MultiNl(m) = &mut pb.stages[idx] { for (d,o) in m.dc_bias.iter_mut().zip(&orig) { *d = o*alpha; } }
        pb.process(0.0);
    }
    if let Stage::MultiNl(m) = &mut pb.stages[idx] { m.dc_bias = orig.clone(); }
    for _ in 0..30000 { pb.process(0.0); }
    let v_ramp = match &pb.stages[idx] { Stage::MultiNl(m)=>m.v_prev.clone(), _=>unreachable!() };
    eprintln!("\n=== Supply continuation (port order [Q2.Vbe,Q2.Vce,Q1.Vbe,Q1.Vce]; SPICE active: Q2.Vce~12) ===");
    eprintln!("cold-start: v_prev={v_cold:.3?}  Q2.Vce={:.2}", v_cold[1]);
    eprintln!("ramped:     v_prev={v_ramp:.3?}  Q2.Vce={:.2}", v_ramp[1]);
}

#[test]
fn proper_vcc_ramp_validation() {
    let source = include_str!("../circuits/active/xref_feedback_amp.pedal");
    let def = parse_pedal_file(source).expect("parse");
    let mut proc = compile_pedal_with_options(&def, SR, CompileOptions{skip_blockwise:true,..CompileOptions::default()}).expect("compile");
    let idx = proc.stages.iter().position(|s| matches!(s, Stage::MultiNl(_))).unwrap();
    let read = |p: &dyn std::any::Any| {}; let _ = read;
    // cold-start control
    let mut pc = compile_pedal_with_options(&def, SR, CompileOptions{skip_blockwise:true,..CompileOptions::default()}).expect("compile");
    for _ in 0..30000 { pc.process(0.0); }
    let v_cold = match &pc.stages[idx] { Stage::MultiNl(m)=>m.v_prev.clone(),_=>unreachable!() };
    // proper ramp: drop to 5V, settle, ramp 5->24 slowly (scales dc_bias AND device v_max)
    proc.set_supply_voltage(0.1);
    for _ in 0..8000 { proc.process(0.0); }
    let v5 = match &proc.stages[idx] { Stage::MultiNl(m)=>m.v_prev.clone(),_=>unreachable!() };
    let ramp = 300_000usize;
    for k in 0..ramp {
        let v = 0.1 + (k as f64 / ramp as f64) * (24.0 - 0.1);
        proc.set_supply_voltage(v);
        proc.process(0.0);
    }
    proc.set_supply_voltage(24.0);
    for _ in 0..30000 { proc.process(0.0); }
    let vr = match &proc.stages[idx] { Stage::MultiNl(m)=>m.v_prev.clone(),_=>unreachable!() };
    eprintln!("\n=== Proper VCC ramp validation (port [Q2.Vbe,Q2.Vce,Q1.Vbe,Q1.Vce]; SPICE active Q2.Vce~12) ===");
    eprintln!("cold-start:    v_prev={v_cold:.3?}  Q2.Vce={:.2}", v_cold[1]);
    eprintln!("settled @5V:   v_prev={v5:.3?}  Q2.Vce={:.2}", v5[1]);
    eprintln!("after 5->24V:  v_prev={vr:.3?}  Q2.Vce={:.2}", vr[1]);
}


#[test]
fn real_circuits_multinl_check() {
    for path in ["circuits/active/fuzz_face_pnp.pedal", "circuits/active/push_pull_6l6.pedal"] {
        let src = match std::fs::read_to_string(path) { Ok(s)=>s, Err(e)=>{ eprintln!("{path}: READ ERR {e}"); continue; } };
        let def = match parse_pedal_file(&src) { Ok(d)=>d, Err(e)=>{ eprintln!("{path}: PARSE ERR {e}"); continue; } };
        let opts = CompileOptions { skip_blockwise: true, ..CompileOptions::default() };
        let mut proc = match compile_pedal_with_options(&def, SR, opts) { Ok(p)=>p, Err(e)=>{ eprintln!("{path}: COMPILE ERR {e}"); continue; } };
        let shape: Vec<String> = proc.stages.iter().map(|s| match s {
            Stage::MultiNl(m)=>format!("MultiNl(n_nl={})",m.n_nl), Stage::Wdf(_)=>"Wdf".into(), Stage::Blockwise(_)=>"Blockwise".into(), _=>"other".into() }).collect();
        eprintln!("\n{path}\n  stages: {shape:?}");
        if let Some(idx) = proc.stages.iter().position(|s| matches!(s, Stage::MultiNl(_))) {
            for _ in 0..60000 { proc.process(0.0); }
            let v = match &proc.stages[idx] { Stage::MultiNl(m)=>m.v_prev.clone(), _=>unreachable!() };
            eprintln!("  WDF MultiNl settled v_prev = {v:.3?}");
        }
    }
}

#[test]
fn fuzz_face_default_path() {
    let src = std::fs::read_to_string("circuits/active/fuzz_face_pnp.pedal").expect("read");
    let def = parse_pedal_file(&src).expect("parse");
    for (label, skip) in [("DEFAULT (blockwise on)", false), ("skip_blockwise", true)] {
        let opts = CompileOptions { skip_blockwise: skip, ..CompileOptions::default() };
        let mut proc = compile_pedal_with_options(&def, SR, opts).expect("compile");
        let shape: Vec<String> = proc.stages.iter().map(|s| match s {
            Stage::MultiNl(m)=>format!("MultiNl(n_nl={})",m.n_nl), Stage::Wdf(_)=>"Wdf".into(),
            Stage::Blockwise(_)=>"Blockwise".into(), Stage::KMethod{..}=>"KMethod".into(), _=>"other".into() }).collect();
        for _ in 0..40000 { proc.process(0.0); }
        eprintln!("\n{label}: stages={shape:?}");
    }
}

#[test]
fn corpus_crash_check() {
    let paths = [
        "circuits/compressor/la2a.pedal", "circuits/eq/pultec_eqp1a.pedal",
        "circuits/compressor/fet_leveler.pedal", "circuits/canonical/bjt_diff_pair.pedal",
        "circuits/active/push_pull_6l6.pedal", "circuits/active/fuzz_face_pnp.pedal",
        "circuits/active/xref_feedback_amp.pedal",
    ];
    for p in paths {
        let src = match std::fs::read_to_string(p) { Ok(s)=>s, Err(_)=>{ eprintln!("{p}: (missing)"); continue; } };
        let r = std::panic::catch_unwind(|| {
            let def = parse_pedal_file(&src).ok()?;
            let opts = CompileOptions { ..CompileOptions::default() };
            let p2 = compile_pedal_with_options(&def, SR, opts).ok()?;
            Some(p2.stages.iter().filter(|s| matches!(s, Stage::MultiNl(_))).count())
        });
        match r {
            Ok(Some(n)) => eprintln!("{p}: OK ({n} MultiNl stages)"),
            Ok(None) => eprintln!("{p}: compile/parse returned None"),
            Err(_) => eprintln!("{p}: !!! PANIC"),
        }
    }
}
