//! BA283 per-device base-current precision — our Gummel-Poon vs ngspice.
//!
//! The BA283 bias residual is dominated by Ib precision: the BC184C runs
//! β≈2–6 here (Ib is ISE/NE recombination-dominated, not transport), and a
//! ~1–2 µA Ib error is amplified by the 56k (Rfb) / 68k (R3) shunt-feedback
//! network into a ~40 mV base-node offset → the starved Q1/Q2 root.  Closing
//! ΔIc < 10 % therefore requires per-device |ΔIb| < ~0.2 µA **at ngspice's own
//! operating-point voltages** — a pure device-model comparison with the
//! network factored out.
//!
//! # Reference data provenance
//!
//! `spice-circuits/active/neve1073_ba283.spice` + `VIN v_in 0 DC 0` (the deck
//! is multi-stable floating without it), ngspice `.op` at TEMP = TNOM = 27 °C,
//! `set numdgt=12`:
//!
//! ```text
//! print n1 nb1 nd ne1 ne2  @qX[ib] @qX[ic] @qX[vbe] @qX[vbc]
//! ```
//!
//! `@q[vbe]`/`@q[vbc]` are ngspice's **internal junction** voltages (inside
//! RB/RE/RC); node voltages give the **terminal** voltages.  Both layers are
//! checked: layer 1 isolates the junction equations, layer 2 additionally
//! exercises the RB/RE/RC terminal→junction decomposition in
//! `BjtTwoPort::eval`.

use pedalkernel::model_lookup::bjt_model_by_name;
use pedalkernel_rt::elements::nonlinear::BjtTwoPort;
use pedalkernel_rt::elements::solver::NlDeviceGroupIv;

/// One device row of the ngspice `.op` reference.
struct NgDevice {
    name: &'static str,
    model: &'static str,
    /// Internal junction voltages (ngspice `@q[vbe]`, `@q[vbc]`), V.
    vbe_int: f64,
    vbc_int: f64,
    /// Terminal voltages from node prints, V.
    vb: f64,
    vc: f64,
    ve: f64,
    /// ngspice terminal currents, A.
    ib: f64,
    ic: f64,
}

/// ngspice 44 `.op` of neve1073_ba283.spice, VIN grounded, TEMP=TNOM=27.
const NG_OP: [NgDevice; 3] = [
    NgDevice {
        name: "Q3 (TR1 input)",
        model: "BC184C",
        vbe_int: 0.6086984124979,
        vbc_int: -4.74434365048,
        vb: 1.030801940602,  // nb1
        vc: 5.742598078885,  // n1
        ve: 0.3891352092513, // ne1
        ib: 6.561236227367e-05,
        ic: 2.586670139218e-04,
    },
    NgDevice {
        name: "Q1 (TR2 driver)",
        model: "BC184C",
        vbe_int: 0.5352257669389,
        vbc_int: -18.2622949629,
        vb: 5.742598078885, // n1
        vc: 24.0,           // vcc
        ve: 5.202445758519, // ne2
        ib: 9.829710694514e-06,
        ic: 1.909043528619e-05,
    },
    NgDevice {
        name: "Q2 (TR3 output)",
        model: "2N3055",
        vbe_int: 0.4063058213889,
        vbc_int: -18.7975565523,
        vb: 5.202445758519, // ne2
        vc: 24.0,           // vcc
        ve: 4.796136157429, // nd
        ib: 1.260601440336e-06,
        ic: 3.676440820219e-05,
    },
];

/// Per-device |ΔIb| gate (A): the network amplifies ~1 µA of Ib error into the
/// ~40 mV starved-root offset; 0.2 µA keeps the base-node offset < ~10 mV.
const IB_GATE_A: f64 = 0.2e-6;
/// Per-device relative ΔIc gate at fixed voltages (the model itself).
const IC_REL_GATE: f64 = 0.01;

#[test]
fn ba283_ib_matches_ngspice_at_op_voltages() {
    let mut fail = false;

    eprintln!("── Layer 1: junction equations at ngspice INTERNAL (vbe', vbc') ──");
    eprintln!(
        "{:<16} {:>12} {:>12} {:>9} | {:>12} {:>12} {:>8}",
        "device", "ng ib µA", "our ib µA", "Δib µA", "ng ic µA", "our ic µA", "Δic %"
    );
    for d in &NG_OP {
        let model = bjt_model_by_name(d.model);
        let (ic, ib) = model.currents(d.vbe_int, d.vbc_int);
        let dib = ib - d.ib;
        let dic_rel = (ic - d.ic) / d.ic;
        eprintln!(
            "{:<16} {:>12.5} {:>12.5} {:>9.4} | {:>12.5} {:>12.5} {:>8.3}",
            d.name,
            d.ib * 1e6,
            ib * 1e6,
            dib * 1e6,
            d.ic * 1e6,
            ic * 1e6,
            dic_rel * 100.0
        );
        if dib.abs() > IB_GATE_A || dic_rel.abs() > IC_REL_GATE {
            fail = true;
        }
    }

    eprintln!("── Layer 2: BjtTwoPort::eval at TERMINAL (vbe, vce) — RB/RE/RC path ──");
    eprintln!(
        "{:<16} {:>12} {:>12} {:>9} | {:>12} {:>12} {:>8}",
        "device", "ng ib µA", "our ib µA", "Δib µA", "ng ic µA", "our ic µA", "Δic %"
    );
    for d in &NG_OP {
        let model = bjt_model_by_name(d.model);
        let bjt = BjtTwoPort::new(model);
        let vbe_ext = d.vb - d.ve;
        let vce_ext = d.vc - d.ve;
        let mut currents = [0.0f64; 2];
        let mut jac = [0.0f64; 4];
        bjt.eval(&[vbe_ext, vce_ext], &mut currents, &mut jac);
        let (ib, ic) = (currents[0], currents[1]);
        let dib = ib - d.ib;
        let dic_rel = (ic - d.ic) / d.ic;
        eprintln!(
            "{:<16} {:>12.5} {:>12.5} {:>9.4} | {:>12.5} {:>12.5} {:>8.3}",
            d.name,
            d.ib * 1e6,
            ib * 1e6,
            dib * 1e6,
            d.ic * 1e6,
            ic * 1e6,
            dic_rel * 100.0
        );
        if dib.abs() > IB_GATE_A || dic_rel.abs() > IC_REL_GATE {
            fail = true;
        }
    }

    assert!(
        !fail,
        "per-device Ib/Ic vs ngspice out of gate (|Δib| < {:.2} µA, |Δic| < {} %) — see table",
        IB_GATE_A * 1e6,
        IC_REL_GATE * 100.0
    );
}

/// Layer 3 — deck-KCL audit of OUR settled root.
///
/// With the device model now ngspice-exact, any difference between our settled
/// DC root and ngspice's can only come from the compiled NETWORK.  This probe
/// settles the compiled BA283 cold (as `bias_accuracy` MATCH does), reads the
/// per-device op, reconstructs the deck node voltages from the emitter ladder,
/// and evaluates the DECK's own KCL (R3=68k, R4=1.2k, R6=18k, R7=47, RV1=2350,
/// Rfb=56k, VCC=24) at every DC node.  Our root satisfies OUR equations exactly,
/// so any deck-KCL violation printed here IS the compiled-network-vs-deck delta,
/// localized to a node.
#[test]
fn ba283_our_root_deck_kcl_audit() {
    use pedalkernel::compiler::compile_pedal;
    use pedalkernel::dsl::parse_pedal_file;
    use pedalkernel::PedalProcessor;
    use pedalkernel_rt::processor::Stage;
    use pedalkernel_validate::pro_pedal::load_pro_pedal_sub;
    use pedalkernel_validate::skip_if_missing;

    let source = skip_if_missing!(
        load_pro_pedal_sub("pedals/500/1073/sub/ba283.pedal"),
        "pedals/500/1073/sub/ba283.pedal"
    );
    let def = parse_pedal_file(&source).expect("parse BA283");
    let mut proc = compile_pedal(&def, 48_000.0).expect("compile BA283");
    proc.set_control("Gain", 0.5);
    let q3_vbe = |proc: &pedalkernel::compiler::CompiledPedal| -> f64 {
        for st in proc.stages.iter() {
            if let Stage::MultiNl(mnl) = st {
                for d in mnl.device_op_points() {
                    if d.ref_name == "Q3" {
                        return d.v_be;
                    }
                }
            }
        }
        f64::NAN
    };
    // Pre-settle: is the COLD-compiled state a fixed point of its own balance?
    for st in proc.stages.iter() {
        if let Stage::MultiNl(mnl) = st {
            if mnl.device_groups.is_none() {
                continue;
            }
            let ports = mnl.op_seed_residual();
            let max_r = ports.iter().map(|p| p.residual.abs()).fold(0.0f64, f64::max);
            eprintln!("  F(cold dc_qpoint seed) per-port:");
            for (i, p) in ports.iter().enumerate() {
                eprintln!(
                    "    port {i}: a={:+.4} dc_bias={:+.4} cap={:+.4} nl={:+.4}  r={:+.4} V",
                    p.a, p.dc_bias, p.cap, p.nl, p.residual
                );
            }
            eprintln!("  max|F(cold)| = {max_r:.4} V");
        }
    }
    eprintln!("  settle trajectory Q3.vbe (seed target 0.64167):");
    for i in 0..48_000 {
        proc.process(0.0);
        if [0usize, 1, 3, 10, 30, 100, 300, 1_000, 3_000, 10_000, 30_000, 47_999]
            .contains(&i)
        {
            eprintln!("    sample {i:>6}: vbe(Q3) = {:.5}", q3_vbe(&proc));
        }
    }

    // Collect our settled per-device op (terminal vbe/vce) keyed by ref name.
    let mut ops: std::collections::BTreeMap<String, (f64, f64, f64)> = Default::default();
    for st in proc.stages.iter() {
        if let Stage::MultiNl(mnl) = st {
            if let Some(qv) = &mnl.dc_qpoint_v {
                eprintln!(
                    "  compile-time dc_qpoint_v (port order): {:?}",
                    qv.iter().map(|v| (v * 1e4).round() / 1e4).collect::<Vec<_>>()
                );
            }
            for d in mnl.device_op_points() {
                if !d.ref_name.is_empty() {
                    ops.insert(d.ref_name.clone(), (d.v_be, d.v_ce, d.i_c));
                }
            }
        }
    }
    let get = |k: &str| *ops.get(k).unwrap_or_else(|| panic!("no op for {k}"));
    let (vbe3, vce3, ic3_rt) = get("Q3");
    let (vbe1, vce1, ic1_rt) = get("Q1");
    let (vbe2, vce2, ic2_rt) = get("Q2");

    // Device currents from the (ngspice-exact) model at OUR root.
    let ib_ic = |model: &str, vbe: f64, vce: f64| -> (f64, f64) {
        let bjt = BjtTwoPort::new(bjt_model_by_name(model));
        let mut c = [0.0f64; 2];
        let mut j = [0.0f64; 4];
        bjt.eval(&[vbe, vce], &mut c, &mut j);
        (c[0], c[1])
    };
    let (ib3, ic3) = ib_ic("BC184C", vbe3, vce3);
    let (ib1, ic1) = ib_ic("BC184C", vbe1, vce1);
    let (ib2, ic2) = ib_ic("2N3055", vbe2, vce2);
    eprintln!("── Layer 3: our settled root, deck-KCL audit ──");
    eprintln!(
        "  Q3 vbe={vbe3:.5} vce={vce3:.4} ic_rt={:.2}µA ic_model={:.2}µA ib={:.2}µA",
        ic3_rt * 1e6,
        ic3 * 1e6,
        ib3 * 1e6
    );
    eprintln!(
        "  Q1 vbe={vbe1:.5} vce={vce1:.4} ic_rt={:.2}µA ic_model={:.2}µA ib={:.2}µA",
        ic1_rt * 1e6,
        ic1 * 1e6,
        ib1 * 1e6
    );
    eprintln!(
        "  Q2 vbe={vbe2:.5} vce={vce2:.4} ic_rt={:.2}µA ic_model={:.2}µA ib={:.2}µA",
        ic2_rt * 1e6,
        ic2 * 1e6,
        ib2 * 1e6
    );

    // Reconstruct deck nodes from the emitter ladder.
    let vcc = 24.0;
    let ie3 = ib3 + ic3;
    let ie1 = ib1 + ic1;
    let ie2 = ib2 + ic2;
    let ne1 = ie3 * 1_200.0;
    let nb1 = ne1 + vbe3;
    let n1 = ne1 + vce3;
    let ne2 = n1 - vbe1; // Q1 base = n1
    let nd = ne2 - vbe2; // Q2 base = ne2
    let nfb = nd - ie2 * (47.0 + 2_350.0); // Cout blocks DC ⇒ I(R7)=Ie2
    eprintln!(
        "  nodes: nb1={nb1:.4} n1={n1:.4} ne1={ne1:.4} ne2={ne2:.4} nd={nd:.4} nfb={nfb:.4}"
    );
    eprintln!(
        "  cross-checks: vce1 vs 24−ne2: {:.4} vs {:.4}   vce2 vs 24−nd: {:.4} vs {:.4}",
        vce1,
        vcc - ne2,
        vce2,
        vcc - nd
    );

    // Deck KCL residuals (µA). Positive = current surplus into the node.
    let r_n1 = (vcc - n1) / 68_000.0 - ic3 - ib1;
    let r_nb1 = (nfb - nb1) / 56_000.0 - ib3;
    let r_ne2 = ie1 - ib2 - (ne2 - nfb) / 18_000.0;
    let r_nfb = ie2 + (ne2 - nfb) / 18_000.0 - (nfb - nb1) / 56_000.0;
    eprintln!("  deck-KCL residuals (µA, + = surplus into node):");
    eprintln!("    n1  (R3 vs Ic3+Ib1)      : {:+9.3}", r_n1 * 1e6);
    eprintln!("    nb1 (Rfb vs Ib3)         : {:+9.3}", r_nb1 * 1e6);
    eprintln!("    ne2 (Ie1 vs Ib2+R6)      : {:+9.3}", r_ne2 * 1e6);
    eprintln!("    nfb (R7+R6 in vs Rfb out): {:+9.3}", r_nfb * 1e6);
}

/// Layer 4 — standalone deck DC solve with OUR device model.
///
/// Newton-solves the deck's 6 DC nodes (nb1, n1, ne1, ne2, nd, nfb) using
/// `BjtTwoPort::eval` terminal currents.  With the deck's RV1 = 2350 Ω (pot at
/// 0.5) the root must land on ngspice's op — proving model+network agreement.
/// With RV1 = 4700 Ω (the FULL pot track, which `Potentiometer::resistance()`
/// reports to `solve_bjt_group_dc_qpoint`) the root shows where the compiler's
/// DC q-point actually sits.
#[test]
fn ba283_deck_dc_solve_pot_sensitivity() {
    let solve = |rv1: f64| -> [f64; 6] {
        let q_bc = BjtTwoPort::new(bjt_model_by_name("BC184C"));
        let q_3055 = BjtTwoPort::new(bjt_model_by_name("2N3055"));
        let dev = |bjt: &BjtTwoPort, vbe: f64, vce: f64| -> (f64, f64) {
            let mut c = [0.0f64; 2];
            let mut j = [0.0f64; 4];
            bjt.eval(&[vbe, vce], &mut c, &mut j);
            (c[0], c[1]) // (ib, ic)
        };
        // v = [nb1, n1, ne1, ne2, nd, nfb]
        let kcl = |v: &[f64; 6]| -> [f64; 6] {
            let [nb1, n1, ne1, ne2, nd, nfb] = *v;
            let (ib3, ic3) = dev(&q_bc, nb1 - ne1, n1 - ne1);
            let (ib1, ic1) = dev(&q_bc, n1 - ne2, 24.0 - ne2);
            let (ib2, ic2) = dev(&q_3055, ne2 - nd, 24.0 - nd);
            let rfbk = 47.0 + rv1;
            [
                (nfb - nb1) / 56_000.0 - ib3,
                (24.0 - n1) / 68_000.0 - ic3 - ib1,
                (ib3 + ic3) - ne1 / 1_200.0,
                (ib1 + ic1) - ib2 - (ne2 - nfb) / 18_000.0,
                (ib2 + ic2) - (nd - nfb) / rfbk,
                (ne2 - nfb) / 18_000.0 + (nd - nfb) / rfbk - (nfb - nb1) / 56_000.0,
            ]
        };
        // Start at ngspice's op; NR with numeric Jacobian + step damping.
        let mut v = [1.0308, 5.7426, 0.3891, 5.2024, 4.7961, 4.7051];
        for _ in 0..200 {
            let f = kcl(&v);
            let maxf = f.iter().fold(0.0f64, |m, x| m.max(x.abs()));
            if maxf < 1e-12 {
                break;
            }
            let h = 1e-8;
            let mut jac = [[0.0f64; 6]; 6];
            for c in 0..6 {
                let mut vp = v;
                vp[c] += h;
                let fp = kcl(&vp);
                for r in 0..6 {
                    jac[r][c] = (fp[r] - f[r]) / h;
                }
            }
            // Gaussian elimination, solve jac·d = -f.
            let mut a = jac;
            let mut b: [f64; 6] = core::array::from_fn(|i| -f[i]);
            for p in 0..6 {
                let mut best = p;
                for r in p + 1..6 {
                    if a[r][p].abs() > a[best][p].abs() {
                        best = r;
                    }
                }
                a.swap(p, best);
                b.swap(p, best);
                let piv = a[p][p];
                for r in p + 1..6 {
                    let m = a[r][p] / piv;
                    for c in p..6 {
                        a[r][c] -= m * a[p][c];
                    }
                    b[r] -= m * b[p];
                }
            }
            let mut d = [0.0f64; 6];
            for r in (0..6).rev() {
                let mut s = b[r];
                for c in r + 1..6 {
                    s -= a[r][c] * d[c];
                }
                d[r] = s / a[r][r];
            }
            let maxd = d.iter().fold(0.0f64, |m, x| m.max(x.abs()));
            let scale = if maxd > 0.25 { 0.25 / maxd } else { 1.0 };
            for k in 0..6 {
                v[k] += d[k] * scale;
            }
        }
        v
    };

    for (label, rv1) in [("deck RV1=2350 (pot@0.5)", 2350.0), ("FULL track 4700", 4700.0)] {
        let v = solve(rv1);
        let [nb1, n1, ne1, ne2, nd, nfb] = v;
        eprintln!("── Layer 4: deck DC root with {label} ──");
        eprintln!("  nb1={nb1:.4} n1={n1:.4} ne1={ne1:.4} ne2={ne2:.4} nd={nd:.4} nfb={nfb:.4}");
        eprintln!(
            "  Q3 vbe={:.5} vce={:.4}   Q1 vbe={:.5} vce={:.4}   Q2 vbe={:.5} vce={:.4}",
            nb1 - ne1,
            n1 - ne1,
            n1 - ne2,
            24.0 - ne2,
            ne2 - nd,
            24.0 - nd
        );
    }
    // Gate: with the deck's 2350 the root must be ngspice's op (terminal Vbe/Vce).
    let v = solve(2350.0);
    let ng = [1.030807, 5.742574, 0.389139, 5.202455, 4.796174, 4.705127];
    for (i, (ours, ngv)) in v.iter().zip(ng.iter()).enumerate() {
        assert!(
            (ours - ngv).abs() < 2e-3,
            "node {i}: deck-DC root {ours:.5} vs ngspice {ngv:.5} — model/network mismatch"
        );
    }
}

/// Localization aid: same layer-1 diff swept over candidate thermal voltages.
/// Not a gate — prints which Vt the ngspice build actually uses.
#[test]
fn ba283_ib_vt_sweep() {
    // (label, Vt at 27 °C = 300.15 K)
    let candidates = [
        ("current   0.02585    (25 °C)", 0.02585),
        ("spice3f5  0.02586419 (k=1.3806226e-23, q=1.6021918e-19)", 0.025864186),
        ("CODATA18  0.02586493 (k=1.380649e-23,  q=1.602176634e-19)", 0.025864931),
    ];
    for (label, vt) in candidates {
        eprintln!("Vt = {label}");
        for d in &NG_OP {
            let mut model = bjt_model_by_name(d.model);
            model.vt = vt;
            let (ic, ib) = model.currents(d.vbe_int, d.vbc_int);
            eprintln!(
                "  {:<16} Δib = {:>8.4} µA   Δic = {:>7.3} %",
                d.name,
                (ib - d.ib) * 1e6,
                (ic - d.ic) / d.ic * 100.0
            );
        }
    }
}
