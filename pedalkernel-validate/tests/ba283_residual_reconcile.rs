//! BA283 residual reconciliation — DIAGNOSTIC (measure only, no fix).
//!
//! # The puzzle
//!
//! Two probes report the four-term WDF DC-balance residual `F(ngspice_op)` at the
//! BA283's ngspice operating point, and they DISAGREED by ~0.6 V at TR1.Vce:
//!
//!   * the bias-accuracy dashboard (`bias_accuracy.rs`) read **0.027 V**, and
//!   * the seed-and-hold test (`neve1073_spice::ba283_seed_and_hold`) read
//!     **0.62 V** — worst port TR1.Vce (Q3 collector).
//!
//! Both call the IDENTICAL `MultiNlStage::op_seed_residual` against the IDENTICAL
//! engine-derived `dc_bias`. So where does the 0.6 V come from?
//!
//! # The answer (measured here)
//!
//! The two probes SEED DIFFERENT TR1 base–emitter voltages, because they read two
//! different ngspice "Vbe" definitions for the same `.op`:
//!
//!   * the dashboard's golden (`golden/op/neve1073_ba283.json`) is parsed from the
//!     ngspice `show` device table, which reports the **intrinsic junction**
//!     voltage  →  Q3.Vbe = **0.6087 V**;
//!   * the hold test seeds the **terminal node-difference** `v(nb1) − v(ne1)` —
//!     the voltage that actually appears across the WDF port  →  Q3.Vbe =
//!     **0.6417 V**.
//!
//! They differ by exactly TR1's base-resistance drop: `Ib·RB = 65.6 µA · 500 Ω =
//! 0.033 V` (the QBC184C model has RB = 500 Ω, and at this op `Ib(Q3) = 65.6 µA`).
//! Re-derived fresh from `spice-circuits/active/neve1073_ba283.spice` (+1 MΩ input
//! pulldown) via `/opt/homebrew/bin/ngspice`:
//!
//! ```text
//!   show q3:  vbe 0.608698  ic 258.7 µA  ib 65.6 µA   (intrinsic)
//!   nodes:    v(nb1) 1.0308  v(ne1) 0.3891  → terminal Vbe 0.6417, Vce 5.3535
//!   0.608698 + 65.6µA·500Ω + 324µA·0.5Ω = 0.64166 V  ✓ = terminal
//! ```
//!
//! Because BJT collector current is exponential in Vbe, the 0.033 V seed gap maps
//! to ~2× TR1 collector current (122 µA at the intrinsic seed vs 261 µA at the
//! terminal seed — and the terminal value MATCHES ngspice's true Ic = 258.7 µA).
//! The residual term that moves is `a = Vce + Rp·i_device`; `dc_bias` and `cap`
//! are byte-identical between the two seeds. `Rp(port5) ≈ 4479 Ω`, so
//! `Δa = 4479 · (261 − 122) µA = 0.62 V` — the entire gap.
//!
//! # Verdict
//!
//! * The 0.027-vs-0.62 split is a **probe artifact** (intrinsic vs terminal Vbe
//!   seed), NOT a compile-vs-runtime equation divergence.
//! * Measured at the IDENTICAL, physically-correct **terminal** op-point, the
//!   TRUE residual is **~0.62 V at TR1.Vce** — it does NOT collapse to 0, so this
//!   is a **real formulation residual, not pure root-selection**.
//! * The divergent term is the **DC-bias / Norton source term**: the engine's
//!   compile-time DC solve (`solve_bjt_group_dc_qpoint` → `apply_dc_qpoint_seed`)
//!   biases TR1 at the under-conducting (~half-current, intrinsic-junction) point,
//!   so ngspice's true op (TR1 fully conducting, 258 µA) is NOT a fixed point of
//!   the engine's runtime balance.
//!
//! Run: `cargo test -p pedalkernel-validate --test ba283_residual_reconcile \
//!       -- --nocapture`

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel_rt::processor::Stage;
use pedalkernel_validate::pro_pedal::load_pro_pedal_sub;
use pedalkernel_validate::skip_if_missing;

const SR: f64 = 48_000.0;
const PRO_PATH: &str = "pedals/500/1073/sub/ba283.pedal";

/// ngspice `.op` seeds, in `.pedal` device labels (Q3=TR1, Q1=TR2, Q2=TR3).
/// `(label, vbe, vce)`.
///
/// INTRINSIC = ngspice `show` device-table junction voltages (what the dashboard
/// golden stores). TERMINAL = node-difference voltages `v(node_a) − v(node_b)`
/// (the actual WDF port voltage; what the hold test seeds). Both are the SAME
/// ngspice `.op`; they differ only by the per-device parasitic IR drops.
const SEED_INTRINSIC: &[(&str, f64, f64)] = &[
    ("Q3", 0.608699, 5.353019),
    ("Q1", 0.535194, 18.797494),
    ("Q2", 0.406277, 19.203777),
];
const SEED_TERMINAL: &[(&str, f64, f64)] = &[
    ("Q3", 0.641667, 5.353463),
    ("Q1", 0.540152, 18.797600),
    ("Q2", 0.406310, 19.203900),
];

/// Inject an `op { Q: { vbe, vce } }` block (device ports only — no `nodes{}`, so
/// caps stay at the engine's homotopy precharge; we showed the cap state moves the
/// residual by < 0.003 V, so it is not the variable under study).
fn inject_op_block(source: &str, seed: &[(&str, f64, f64)]) -> String {
    let mut block = String::from("\n  op {\n");
    for (label, vbe, vce) in seed {
        block.push_str(&format!("    {label}: {{ vbe: {vbe}, vce: {vce} }}\n"));
    }
    block.push_str("  }\n");
    let cut = source.rfind('}').expect("pedal has a closing brace");
    let mut out = String::with_capacity(source.len() + block.len());
    out.push_str(&source[..cut]);
    out.push_str(&block);
    out.push_str(&source[cut..]);
    out
}

/// Compile the seeded BA283 and read `op_seed_residual` from the (single) grouped
/// MultiNl stage. Returns `(per_port_residual, per_port_i_device, per_port_a,
/// per_port_dc_bias, per_port_cap, per_port_nl)`.
#[allow(clippy::type_complexity)]
fn measure(source: &str, seed: &[(&str, f64, f64)]) -> Vec<(f64, f64, f64, f64, f64, f64)> {
    let seeded = inject_op_block(source, seed);
    let def = parse_pedal_file(&seeded).expect("parse seeded BA283");
    let mut proc = compile_pedal(&def, SR).expect("compile seeded BA283");
    proc.set_control("Gain", 0.5);
    let mut out = Vec::new();
    for st in proc.stages.iter() {
        let Stage::MultiNl(mnl) = st else { continue };
        if mnl.device_groups.is_none() {
            continue;
        }
        for p in mnl.op_seed_residual() {
            out.push((p.residual, p.i_device, p.a, p.dc_bias, p.cap, p.nl));
        }
    }
    out
}

fn max_abs(ports: &[(f64, f64, f64, f64, f64, f64)]) -> (usize, f64) {
    ports
        .iter()
        .enumerate()
        .map(|(i, p)| (i, p.0.abs()))
        .fold((0, 0.0), |acc, x| if x.1 > acc.1 { x } else { acc })
}

#[test]
fn ba283_residual_reconcile_intrinsic_vs_terminal() {
    let source = skip_if_missing!(load_pro_pedal_sub(PRO_PATH), PRO_PATH);

    let intr = measure(&source, SEED_INTRINSIC);
    let term = measure(&source, SEED_TERMINAL);
    assert_eq!(intr.len(), term.len(), "same port count for both seeds");
    assert_eq!(intr.len(), 6, "BA283 = 3 BJTs × 2 ports");

    // Port → device.port map (offsets from apply_op_seed): 0=Q2.Vbe 1=Q2.Vce
    // 2=Q1.Vbe 3=Q1.Vce 4=Q3(TR1).Vbe 5=Q3(TR1).Vce.
    let names = ["Q2.Vbe", "Q2.Vce", "Q1.Vbe", "Q1.Vce", "TR1(Q3).Vbe", "TR1(Q3).Vce"];

    eprintln!("\n  ═══ BA283 residual reconciliation: INTRINSIC vs TERMINAL Vbe seed ═══");
    eprintln!(
        "  (both call the IDENTICAL op_seed_residual against the IDENTICAL engine dc_bias)\n"
    );
    eprintln!(
        "  {:<13} | {:>10} | {:>10} | {:>9} | {:>10} {:>10}",
        "port", "r_intr V", "r_term V", "Δ V", "i_dev_intr", "i_dev_term"
    );
    eprintln!("  {}", "-".repeat(74));
    for i in 0..intr.len() {
        let (ri, ii, ..) = intr[i];
        let (rt, it, ..) = term[i];
        eprintln!(
            "  {:<13} | {:>+10.4} | {:>+10.4} | {:>+9.4} | {:>10.3e} {:>10.3e}",
            names[i], ri, rt, rt - ri, ii, it
        );
    }

    let (wi, mi) = max_abs(&intr);
    let (wt, mt) = max_abs(&term);
    eprintln!(
        "\n  max|residual|  INTRINSIC seed = {:.4} V at {}",
        mi, names[wi]
    );
    eprintln!("  max|residual|  TERMINAL  seed = {:.4} V at {}", mt, names[wt]);

    // ── TR1.Vce (port 5) term decomposition at BOTH seeds ────────────────────
    let p5i = intr[5];
    let p5t = term[5];
    eprintln!("\n  TR1(Q3).Vce term decomposition  r = a − dc_bias − cap − nl:");
    eprintln!(
        "    INTRINSIC: a={:+.4} dc_bias={:+.4} cap={:+.4} nl={:+.4}  i_dev={:+.3e}  r={:+.4}",
        p5i.2, p5i.3, p5i.4, p5i.5, p5i.1, p5i.0
    );
    eprintln!(
        "    TERMINAL : a={:+.4} dc_bias={:+.4} cap={:+.4} nl={:+.4}  i_dev={:+.3e}  r={:+.4}",
        p5t.2, p5t.3, p5t.4, p5t.5, p5t.1, p5t.0
    );
    let rp = if (p5t.1 - p5i.1).abs() > 1e-12 {
        (p5t.2 - p5i.2) / (p5t.1 - p5i.1)
    } else {
        0.0
    };
    eprintln!(
        "    → dc_bias & cap are IDENTICAL; only a moves: Δa = Rp·Δi_dev = {:.1}Ω · ({:+.3e} A) = {:+.4} V",
        rp,
        p5t.1 - p5i.1,
        p5t.2 - p5i.2
    );

    eprintln!("\n  ═══ VERDICT ═══");
    eprintln!(
        "  0.027-vs-0.62 = PROBE ARTIFACT (intrinsic 0.6087 V vs terminal 0.6417 V TR1.Vbe seed;"
    );
    eprintln!(
        "  gap = Ib·RB = 65.6µA·500Ω = 0.033 V). At the IDENTICAL terminal op the TRUE residual is"
    );
    eprintln!(
        "  ~{:.2} V at TR1.Vce ⇒ a REAL FORMULATION residual in the dc_bias/Norton source term,",
        mt
    );
    eprintln!("  NOT pure root-selection. (The engine biases TR1 at the under-conducting point.)\n");

    // ── Assertions: the artifact is reproducible and localized ───────────────
    // 1. The intrinsic seed (the dashboard's golden) yields a small residual.
    assert!(
        mi < 0.10,
        "INTRINSIC-seed residual should be small (was {mi:.4} V) — this is the dashboard's 0.027 path"
    );
    // 2. The terminal seed (the true WDF port op-point) yields ~0.6 V at TR1.Vce.
    assert!(
        mt > 0.50,
        "TERMINAL-seed residual should be ~0.6 V (was {mt:.4} V) — the hold test's 0.62 path"
    );
    assert_eq!(wt, 5, "worst terminal-seed residual must localize to TR1(Q3).Vce");
    // 3. The gap is carried entirely by `a` (device current), not dc_bias/cap.
    let dc_bias_moved = (p5t.3 - p5i.3).abs();
    let cap_moved = (p5t.4 - p5i.4).abs();
    assert!(
        dc_bias_moved < 1e-6 && cap_moved < 1e-6,
        "dc_bias and cap must be identical across seeds (moved {dc_bias_moved:.2e}/{cap_moved:.2e})"
    );
    // 4. The terminal seed roughly DOUBLES TR1 collector current vs the intrinsic
    //    seed (the exponential Vbe sensitivity that drives the whole gap).
    assert!(
        p5t.1 > 1.8 * p5i.1,
        "terminal seed should ~2× TR1 collector current ({:.3e} vs {:.3e})",
        p5t.1,
        p5i.1
    );
}
