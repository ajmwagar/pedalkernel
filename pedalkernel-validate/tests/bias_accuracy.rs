//! DC operating-point (bias) accuracy dashboard.
//!
//! The missing **primary** validation axis for active circuits: a per-circuit,
//! per-device comparison of our WDF DC bias against ngspice's `.op`. This is the
//! gate for compile-time-DC-solve work — bias-right ⇒ audio-follows
//! (`docs/dc-operating-point-solve.md`).
//!
//! Two sub-metrics (they answer different questions):
//!
//!   1. **MATCH** (Layer B): per-device ΔVbe / ΔVce / ΔIc between OUR settled DC
//!      bias and ngspice's `.op`. Large MATCH + small residual ⇒ solver/root bug.
//!
//!   2. **RESIDUAL** (Layer A): `F(ngspice_op)` — re-pin each BJT's warm-start at
//!      ngspice's `.op` and read the four-term DC-balance residual via
//!      `MultiNlStage::op_seed_residual`. Large residual ⇒ formulation bug.
//!
//! Run the dashboard:
//! ```text
//! cargo test -p pedalkernel-validate --test bias_accuracy bias_accuracy_dashboard -- --nocapture
//! ```
//!
//! Regenerate the ngspice-derived goldens (`golden/op/<circuit>.json`):
//! ```text
//! PK_BIAS_BOOTSTRAP=1 cargo test -p pedalkernel-validate --test bias_accuracy bias_accuracy_dashboard -- --nocapture
//! ```
//!
//! # Engine bias we read
//!
//! We read the engine's **settled** DC bias: compile the pedal, then process ~1 s
//! of silence so the coupling/bypass caps reach DC steady state, then read each
//! device's `(Vbe, Vce, Ic)` from `MultiNlStage::device_op_points()`. This is the
//! representative operating point the audio path actually rides on (the same
//! warm-up the audio metrics use). The compile-time cold-solve seed is a *worse*
//! representative — it is exactly the point the upcoming DC-solve work replaces.
//!
//! # How to add a circuit
//!
//! Append a [`Circuit`] to `CIRCUITS`:
//!   * `name`      — golden file stem (`golden/op/<name>.json`).
//!   * `deck`      — ngspice deck under `spice-circuits/` (the `.op` golden source).
//!   * `pedal`     — `Pedal::Public("circuits/…")` for a public `.pedal`, or
//!                   `Pedal::Pro("pedals/…")` for a private pedalkernel-pro pedal
//!                   (the test SKIPS that circuit when the pro repo is absent).
//!   * `gain`      — optional control to set before settling.
//!
//! The circuit must compile to a `MultiNlStage` with grouped BJT devices (two or
//! more coupled transistors). Single-BJT stages that compile to a `Wdf` root, and
//! tube/triode stages whose ngspice model is a sub-circuit (no `show` device
//! row), are reported as `NO DATA` until the node-voltage extraction path is
//! added — derive `Vgk`/`Vpk` from the deck's node voltages in the snapshot
//! (`SpiceOpSnapshot::nodes`) rather than the `show` device table.

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor; // brings `process` into scope for CompiledPedal
use pedalkernel_rt::processor::Stage;
use pedalkernel_rt::stage::NlDeviceGroupKind;
use pedalkernel_validate::metrics::op_point::{self, DeviceBias, OpPassCriteria, PortResidual};
use pedalkernel_validate::pro_pedal::load_pro_pedal_sub;
use pedalkernel_validate::report::render_bias_accuracy;
use pedalkernel_validate::spice::{SpiceConfig, SpiceOpSnapshot, SpiceRunner};
use std::path::PathBuf;

const SR: f64 = 48_000.0;
const SETTLE_SAMPLES: usize = 48_000; // ~1 s of silence to settle coupling caps.

enum Pedal {
    /// Public `.pedal` under `pedalkernel-validate/circuits/`.
    Public(&'static str),
    /// Private pedalkernel-pro `.pedal` (skipped when the pro repo is absent).
    Pro(&'static str),
}

struct Circuit {
    name: &'static str,
    deck: &'static str,
    pedal: Pedal,
    gain: Option<f64>,
    /// Map from a `.pedal` node NAME (a pin like `Q3.base`, `Cin.a`, or a reserved
    /// node like `in`/`out`) to the ngspice net NAME the golden's node-voltage map
    /// is keyed by. Used to emit the `op { nodes { … } }` sub-block so the reactive
    /// (capacitor) ports are seeded to ngspice's DC node voltages — completing the
    /// FULL operating-point seed. Each cap only fires when BOTH its terminals are
    /// covered here; leaving an entry out leaves that cap at its cold state (and
    /// contaminates the residual — the very bug this map exists to fix). Empty ⇒
    /// caps sit COLD (the old, buggy behavior) and the residual is NOT trustworthy.
    node_map: &'static [(&'static str, &'static str)],
}

/// The wired circuits. BA283 first (the known answer), then a fuzz face and the
/// silicon two-stage feedback amp (the public BA283 proxy). All three compile to
/// the grouped multi-BJT co-solve (`MultiNlStage` with BJT device groups). See the
/// module docs for how to add more. NOTE: a circuit only yields device-level data
/// when it reaches the grouped-NR path — single-BJT stages (`npn_common_emitter`)
/// and uncoupled/K-method stages (`bjt_diff_pair`) compile to a `Wdf`/Blockwise
/// stage with no device groups and read `NO DATA` here.
const CIRCUITS: &[Circuit] = &[
    Circuit {
        name: "neve1073_ba283",
        deck: "active/neve1073_ba283",
        pedal: Pedal::Pro("pedals/500/1073/sub/ba283.pedal"),
        gain: Some(0.5),
        // BA283 caps (Cin, Cmil, Ccmp, Cfb, Cout) — every cap terminal maps to an
        // ngspice net. Mirrors NGSPICE_NODES in tests/neve1073_spice.rs.
        node_map: &[
            ("Cin.a", "v_in"),
            ("Cin.b", "ni"),
            ("Q3.base", "nb1"),
            ("Q3.collector", "n1"),
            ("Q3.emitter", "ne1"),
            ("Rcmp.b", "nrc"),
            ("Q1.emitter", "ne2"),
            ("Q2.emitter", "nd"),
            ("RU1.b", "nfb"),
            ("Cout.b", "v_out"),
        ],
    },
    Circuit {
        name: "fuzz_face_pnp",
        deck: "active/fuzz_face_pnp",
        pedal: Pedal::Public("circuits/active/fuzz_face_pnp.pedal"),
        gain: None,
        // Caps C1 (in→Q1.base) and C2 (Q2.collector→out).
        node_map: &[
            ("C1.a", "v_in"),
            ("C1.b", "net2"),
            ("C2.a", "net6"),
            ("C2.b", "v_out"),
        ],
    },
    Circuit {
        name: "si_fb_amp",
        deck: "active/si_fb_amp",
        pedal: Pedal::Public("circuits/active/si_fb_amp.pedal"),
        gain: None,
        // Caps Cin (in→nb1) and Cout (nout→out).
        node_map: &[
            ("Cin.a", "v_in"),
            ("Cin.b", "nb1"),
            ("Cout.a", "nout"),
            ("Cout.b", "v_out"),
        ],
    },
];

fn manifest() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn deck_path(deck: &str) -> PathBuf {
    manifest().join("spice-circuits").join(format!("{deck}.spice"))
}

fn golden_path(name: &str) -> PathBuf {
    manifest().join("golden").join("op").join(format!("{name}.json"))
}

/// Load the ngspice-derived `.op` golden, generating it from the deck if missing
/// (or when `PK_BIAS_BOOTSTRAP=1`). The golden is NEVER WDF-bootstrapped.
fn ngspice_op(circuit: &Circuit) -> Option<SpiceOpSnapshot> {
    let gp = golden_path(circuit.name);
    let force = std::env::var("PK_BIAS_BOOTSTRAP").map(|v| v == "1").unwrap_or(false);
    if gp.exists() && !force {
        let s = std::fs::read_to_string(&gp).ok()?;
        return serde_json::from_str(&s).ok();
    }
    // Generate from ngspice (the golden source).
    let dp = deck_path(circuit.deck);
    if !dp.exists() {
        eprintln!("  deck missing: {dp:?}");
        return None;
    }
    let runner = SpiceRunner::new(SpiceConfig { sample_rate: SR as u32, oversample: 4 });
    match runner.operating_point(&dp) {
        Ok(snap) => {
            std::fs::create_dir_all(gp.parent().unwrap()).ok();
            if let Ok(json) = serde_json::to_string_pretty(&snap) {
                std::fs::write(&gp, json).ok();
                eprintln!("  wrote golden {gp:?}");
            }
            Some(snap)
        }
        Err(e) => {
            eprintln!("  ngspice .op failed for {}: {e}", circuit.name);
            None
        }
    }
}

/// Load a circuit's `.pedal` source. Returns `None` for a Pro pedal that is
/// absent (so the circuit is skipped, not failed).
fn load_source(circuit: &Circuit) -> Option<String> {
    match &circuit.pedal {
        Pedal::Public(rel) => {
            let p = manifest().join(rel);
            Some(std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("read {p:?}: {e}")))
        }
        Pedal::Pro(rel) => match load_pro_pedal_sub(rel) {
            Some(s) => Some(s),
            None => {
                eprintln!("  SKIP {} — pro pedal absent ({rel})", circuit.name);
                None
            }
        },
    }
}

/// Compile a `.pedal` source and set the gain control (no silence processing).
fn compile_only(circuit: &Circuit, source: &str) -> pedalkernel::compiler::CompiledPedal {
    let def = parse_pedal_file(source).unwrap_or_else(|e| panic!("parse {}: {e}", circuit.name));
    let mut proc =
        compile_pedal(&def, SR).unwrap_or_else(|e| panic!("compile {}: {e}", circuit.name));
    if let Some(g) = circuit.gain {
        proc.set_control("Gain", g);
    }
    proc
}

/// Compile, set gain, and settle ~1 s of silence so the coupling/bypass caps
/// reach the engine's OWN DC steady state. Used for the MATCH metric (where the
/// engine actually sits after warm-up).
fn compile_settle(circuit: &Circuit, source: &str) -> pedalkernel::compiler::CompiledPedal {
    let mut proc = compile_only(circuit, source);
    for _ in 0..SETTLE_SAMPLES {
        proc.process(0.0);
    }
    proc
}

/// Inject an `op { Q1: { vbe, vce } … nodes { … } }` block (ngspice's `.op`)
/// before the pedal's closing brace. This seeds:
///   * every device's NR warm-start (`vbe`/`vce`) at ngspice's operating point,
///     AND
///   * every reactive (capacitor) port's stored DC voltage, via the `nodes { }`
///     sub-block, so the caps sit at `V(node_a) − V(node_b)` from ngspice's `.op`
///     NODE voltages instead of their COLD compile-time state.
///
/// Seeding the caps is what makes the residual honest: `F(ngspice_op)` MUST be
/// evaluated with the reactive state AT ngspice's op, not at the engine's own
/// cold-solve basin. Omitting `nodes { }` (the historical bug) left every cap
/// cold, which under-reported the residual and mis-classified the BA283 as
/// Layer B when the true, cap-seeded residual is Layer A (Q3 Vbe dominant).
///
/// The `nodes { }` entries are keyed by `.pedal` node NAME (from `circuit.node_map`)
/// with values looked up by ngspice net name in `snap.nodes`. Signs are ngspice's
/// junction-magnitude convention; the DSL/op-seed path applies the per-device PNP
/// sign internally.
fn inject_op_block(source: &str, snap: &SpiceOpSnapshot, circuit: &Circuit) -> String {
    let mut block = String::from("\n  op {\n");
    for d in &snap.devices {
        block.push_str(&format!(
            "    {}: {{ vbe: {}, vce: {} }}\n",
            d.device, d.vbe, d.vce
        ));
    }
    // Cap/node seed: emit `nodes { <pedal_name>: <ngspice_node_voltage> }` so the
    // reactive ports are seeded consistent with ngspice's op — the fix for the
    // cold-cap residual contamination. Only names whose ngspice net is present in
    // the golden's node map are emitted.
    if !circuit.node_map.is_empty() {
        block.push_str("    nodes {\n");
        for (pedal_name, spice_net) in circuit.node_map {
            match snap.nodes.get(*spice_net) {
                Some(v) => block.push_str(&format!("      {pedal_name}: {v}\n")),
                None => eprintln!(
                    "  [{}] node_map: ngspice net {spice_net:?} (for {pedal_name:?}) \
                     absent from golden — cap seed incomplete",
                    circuit.name
                ),
            }
        }
        block.push_str("    }\n");
    }
    block.push_str("  }\n");
    let cut = source.rfind('}').expect("pedal has a closing brace");
    let mut out = String::with_capacity(source.len() + block.len());
    out.push_str(&source[..cut]);
    out.push_str(&block);
    out.push_str(&source[cut..]);
    out
}

/// Read OUR settled per-device bias from every MultiNl stage.
fn wdf_device_bias(proc: &pedalkernel::compiler::CompiledPedal) -> Vec<DeviceBias> {
    let mut out = Vec::new();
    for st in proc.stages.iter() {
        if let Stage::MultiNl(mnl) = st {
            for d in mnl.device_op_points() {
                if d.ref_name.is_empty() {
                    continue;
                }
                out.push(DeviceBias {
                    reference: d.ref_name,
                    vbe: d.v_be,
                    vce: d.v_ce,
                    ic: d.i_c,
                });
            }
        }
    }
    out
}

/// Re-pin each BJT warm-start at ngspice's `.op` and read the per-port DC-balance
/// residual `F(ngspice_op)` — the formulation probe (Layer A). The op{} block
/// already set v_prev + cap state at compile; this re-pin is an explicit,
/// idempotent restatement of the seed before reading `op_seed_residual`.
fn residual_at_ngspice_op(
    proc: &mut pedalkernel::compiler::CompiledPedal,
    snap: &SpiceOpSnapshot,
) -> Vec<PortResidual> {
    let mut out = Vec::new();
    for st in proc.stages.iter_mut() {
        let Stage::MultiNl(mnl) = st else { continue };
        let Some(dg) = mnl.device_groups.as_ref() else { continue };
        let mut reseed: Vec<(usize, f64, f64)> = Vec::new();
        for (g, group) in dg.groups.iter().enumerate() {
            let NlDeviceGroupKind::BjtTwoPort(bjt) = group else { continue };
            let label = dg
                .identities
                .get(g)
                .map(|id| id.ref_name.to_uppercase())
                .unwrap_or_default();
            let Some(dev) = snap.devices.iter().find(|d| d.device.to_uppercase() == label) else {
                continue;
            };
            let sign = if bjt.is_pnp { -1.0 } else { 1.0 };
            reseed.push((dg.offsets[g], sign * dev.vbe, sign * dev.vce));
        }
        if reseed.is_empty() {
            continue;
        }
        for (off, vbe, vce) in reseed {
            for (port, v) in [(off, vbe), (off + 1, vce)] {
                if port < mnl.n_nl {
                    mnl.v_prev[port] = v as pedalkernel_rt::Wave;
                }
            }
        }
        for p in mnl.op_seed_residual() {
            out.push(PortResidual {
                residual: p.residual,
                residual_full: p.residual_full,
            });
        }
    }
    out
}

#[test]
fn bias_accuracy_dashboard() {
    let criteria = OpPassCriteria::default();
    let mut results = Vec::new();
    let mut ba283: Option<op_point::OpPointResult> = None;

    for circuit in CIRCUITS {
        let Some(snap) = ngspice_op(circuit) else {
            eprintln!("  skip {} (no ngspice golden)", circuit.name);
            continue;
        };
        let Some(source) = load_source(circuit) else {
            continue; // Pro pedal absent.
        };

        // MATCH: the engine's OWN cold-solve settled bias (no seed).
        let match_proc = compile_settle(circuit, &source);
        let ours = wdf_device_bias(&match_proc);

        // RESIDUAL: op{}-seed each device at ngspice's .op AND seed the caps via
        // the `nodes { }` sub-block. At compile the engine initializes v_prev AND
        // the reactive (cap) state consistent with that seed, so F(ngspice_op) is
        // read IMMEDIATELY — no silence settle (which would walk the caps off the
        // seed and contaminate the `cap` term). Seeding the caps (not leaving them
        // COLD) is what makes the residual honest.
        let seeded = inject_op_block(&source, &snap, circuit);
        let mut res_proc = compile_only(circuit, &seeded);
        let residuals = residual_at_ngspice_op(&mut res_proc, &snap);
        let res = op_point::evaluate(circuit.name, &ours, &snap.devices, &residuals, &criteria);
        if circuit.name == "neve1073_ba283" {
            ba283 = Some(res.clone());
        }
        results.push(res);
    }

    if results.is_empty() {
        eprintln!("  no circuits evaluated (ngspice/pro absent) — skipping");
        return;
    }

    let table = render_bias_accuracy(&results, &criteria);
    println!("{table}");

    // ── BA283 verdict gate (the known circuit) ───────────────────────────────
    // BA283 is the calibration circuit: we KNOW its bias is wrong (TR1 starves),
    // so the dashboard must (a) pair all three transistors against ngspice, (b)
    // find a clearly-off MATCH, and (c) emit a decisive Layer-A/Layer-B verdict.
    //
    // With the FULL operating-point seed (device ports AND caps, via the `op {
    // nodes { … } }` sub-block) the residual is HONEST: F(ngspice_op) ≈ 0.615 V,
    // dominated by Q3's Vbe — ngspice's `.op` is genuinely NOT a fixed point of
    // our DC equations. That is **Layer A** (DC-bias formulation), matching the
    // repo's BA283 note. The old dashboard omitted `nodes { }`, so the caps sat
    // COLD, the residual read a contaminated ≈ 0.04 V, and it MIS-classified the
    // circuit as Layer B. The gate below now locks in Layer A.
    if let Some(r) = ba283 {
        println!(
            "\nBA283 verdict: {}\n  max|resid|={:.4}V (full={:.2e}V, {} ports)  \
             max ΔVbe={:.4}V  ΔVce={:.3}V  ΔIc={:.1}%",
            r.verdict.label(),
            r.max_abs_residual,
            r.max_abs_residual_full,
            r.n_residual_ports,
            r.max_abs_d_vbe,
            r.max_abs_d_vce,
            r.max_abs_d_ic_pct,
        );
        let layer = match r.verdict {
            op_point::Verdict::FormulationBug => "Layer A (formulation / DC-bias math)",
            op_point::Verdict::FormulationOkSolverOff => "Layer B (solver / root-selection)",
            op_point::Verdict::BiasOk => "bias correct",
            op_point::Verdict::NoData => "NO DATA",
        };
        println!("  ⇒ localized to: {layer}\n");

        // The metric must have paired all three BJTs and produced residual ports.
        assert_eq!(r.devices.len(), 3, "BA283 should pair Q1/Q2/Q3 vs ngspice");
        assert_eq!(r.n_residual_ports, 6, "BA283 has 3 BJTs × 2 ports");
        // The BA283 MATCH is CLOSED (cold-path DC solve lands AND holds
        // ngspice's op exactly, cdce13b4): ΔVbe = 0.0000 V, ΔIc = 0.0 %. This
        // used to assert the opposite ("MATCH should be clearly off", the old
        // TR1-starved state) — that canary predated the closure and had gone
        // stale. LOCK the closed state with the same bounds as the permanent
        // ba283_ib_diagnostic gates (Vbe < 2 mV, Ic < 2 %).
        assert!(
            r.max_abs_d_vbe < 0.002,
            "BA283 DC bias regressed: max ΔVbe={:.4}V (gate: < 2 mV; the closed \
             state reads 0.0000 V)",
            r.max_abs_d_vbe
        );
        assert!(
            r.max_abs_d_ic_pct < 2.0,
            "BA283 DC bias regressed: max ΔIc={:.1}% (gate: < 2 %; the closed \
             state reads 0.0 %)",
            r.max_abs_d_ic_pct
        );
        // Part 2 (stiff-cap fold branch, `apply_cap_seed` self-consistency fix):
        // the cap-seeded residual is now the TRUE formulation residual ≈ 0.04 V,
        // NOT the old 0.615 V. The 0.615 V was an ARTIFACT: the caps were seeded
        // HOT to ngspice's node voltages, but `dc_bias` was still derived against
        // the compiler's own DC cap solve (`dc_qpoint_passive_b`), whose input
        // coupling cap Cin sat COLD at 0 V instead of its real −1.03 V. That
        // inconsistency injected ~0.6 V of spurious `known_a` residual and stopped
        // the grouped-NR from converging at the seeded op. Re-pointing
        // `dc_qpoint_passive_b` at the seeded (ngspice) cap waves and re-running
        // the wave-domain Norton inversion makes `F(ngspice_op) ≈ 0` — exactly the
        // Part-2 directive (fix the `dc_bias` source so the op is a fixed point),
        // using only `i(v*)` (no small-signal Jacobians). Guard the low band so a
        // regression that RE-introduces the inconsistency (residual jumps back to
        // ~0.6 V) or a runaway is caught.
        assert!(
            r.max_abs_residual < 0.15,
            "BA283 cap-seeded residual should be the TRUE formulation residual \
             ≈ 0.04 V after the Part-2 seed self-consistency fix, got {:.4} V \
             — a value near 0.6 V means the dc_bias↔cap-seed inconsistency regressed",
            r.max_abs_residual
        );
        // With the seed self-consistent AND the solver landing/holding ngspice's
        // op exactly (cdce13b4), the verdict is BIAS OK — both the formulation
        // and the root are correct. (The old `FormulationOkSolverOff` lock was
        // the intermediate state before the cold-path solve closed the root.)
        assert_eq!(
            r.verdict,
            op_point::Verdict::BiasOk,
            "BA283 should read BIAS OK (closed DC op); cap-seeded residual \
             {:.4} V, max ΔVbe {:.4} V",
            r.max_abs_residual,
            r.max_abs_d_vbe,
        );
    } else {
        eprintln!("  (BA283 pro pedal absent — verdict gate skipped)");
    }
}
