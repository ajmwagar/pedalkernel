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
