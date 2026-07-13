// ═══════════════════════════════════════════════════════════════════════════
// Compile-time DC bias characterization (ko5g migration goldens)
//
// These tests PIN the current compile-time DC operating points that the unified
// bias infrastructure (`bias.rs`) produces, so the ko5g migration (absorb
// `bias_analysis.rs`; migrate the triode solver into the shared
// `BiasSeed`/`solve_operating_point` path) is provably behaviour-preserving —
// or, where it moves, moves TOWARD ngspice deliberately and visibly.
//
// Three device families are pinned (one per bias solver mechanism):
//   (a) BJT group   — `solve_bjt_group_dc_qpoint` (the migrated fix-#1 solver
//                      with terminal/parasitic handling; the same solver the
//                      BA283 rides on). Guards the BJT nodal co-solve.
//   (b) Triode      — `solve_triode_dc_qpoint` (the NOT-yet-unified per-type
//                      solver). Pins Vgk/Vpk/Vcathode/Ia for a 12AX7 common-
//                      cathode stage; the inc-3 migration must keep these green
//                      or move them toward ngspice.
//   (c) Static bias — `solve_network_bias` (absorbs `bias_analysis`'s resistor-
//                      divider nodal solve). Pins an interior divider node DC.
//
// The BA283 TR1≈258 µA / Vce≈5.35 fix-#1 result is guarded end-to-end by
// `pedalkernel-validate`'s `bias_accuracy` / `ba283_residual_reconcile` (pro
// pedal). The ko5g increments here do NOT touch BJT physics, so that number is
// preserved by construction; (a) additionally guards the BJT co-solve mechanism
// on a public synthetic stage.
// ═══════════════════════════════════════════════════════════════════════════

use super::classify::NonlinearKind;
use super::graph::CircuitGraph;

const TOL: f64 = 1e-3;

fn graph_of(source: &str) -> CircuitGraph {
    let pedal = crate::dsl::parse_pedal_file(source).expect("parse");
    CircuitGraph::from_pedal(&pedal)
}

fn node(graph: &CircuitGraph, name: &str) -> super::graph::NodeId {
    *graph
        .node_names
        .get(name)
        .unwrap_or_else(|| panic!("node {name} not found; have {:?}", graph.node_names.keys()))
}

// ─────────────────────────────────────────────────────────────────────────────
// (b) TRIODE — 12AX7 common-cathode @ 250 V.  Pins the per-type triode solver.
// ─────────────────────────────────────────────────────────────────────────────

const TRIODE_12AX7: &str = r#"
pedal "12AX7 Common Cathode" {
  supply 250V { impedance: 50, filter_cap: 47u, rectifier: solid_state }
  components {
    C_in: cap(22n)
    R_grid: resistor(1M)
    V1: triode(12ax7)
    R_plate: resistor(100k)
    R_cathode: resistor(1.5k)
    C_cathode: cap(25u)
    C_out: cap(22n)
    R_load: resistor(1M)
  }
  nets {
    in -> C_in.a
    C_in.b -> R_grid.a, V1.grid
    R_grid.b -> gnd
    vcc -> R_plate.a
    R_plate.b -> V1.plate
    V1.cathode -> R_cathode.a, C_cathode.a
    R_cathode.b -> gnd
    C_cathode.b -> gnd
    V1.plate -> C_out.a
    C_out.b -> R_load.a, out
    R_load.b -> gnd
  }
}"#;

fn triode_12ax7_qpoint() -> super::bias::TriodeDcQpoint {
    let graph = graph_of(TRIODE_12AX7);
    let nl = NonlinearKind::Triode {
        model_name: "12ax7".to_string(),
        plate_node: node(&graph, "V1.plate"),
        cathode_node: node(&graph, "V1.cathode"),
        grid_node: Some(node(&graph, "V1.grid")),
        parallel_count: 1,
        is_vari_mu: false,
    };
    let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
    super::bias::solve_triode_dc_qpoint(&[nl], &all_edges, &graph, 250.0)
        .expect("12AX7 common-cathode should have a determinable Q-point")
}

#[test]
fn characterize_triode_12ax7_qpoint() {
    let qp = triode_12ax7_qpoint();
    eprintln!(
        "TRIODE 12AX7 @250V: Vgk={:.6} Vpk={:.6} Vcathode={:.6} Ia={:.9}",
        qp.vgk, qp.vpk, qp.v_cathode, qp.ia
    );

    // GOLDEN — pinned from the current `solve_triode_dc_qpoint` output.
    // (captured 2026-06-30 on 804f91e6). The inc-3 triode migration must keep
    // these OR move them toward ngspice with a documented rationale.
    //
    // ngspice reference (spice-circuits/nonlinear/common_cathode_12ax7.spice,
    // /opt/homebrew/bin/ngspice, ngspice-45.2):  a bare `.op` on the Koren
    // behavioural-source triode deck is DEGENERATE — it reports v_cathode≈0 with
    // ~1.3 mA plate current (cathode KCL grossly violated) and gmin/source
    // stepping fails.  Our compile-time solve is KCL-consistent by construction
    // (Ia·Rk = Vcathode = -Vgk = 1.038 V; Vpk = 250 - Ia·Rp = 180.8 V).  The
    // trustworthy toward-ngspice reference for triodes is therefore a SETTLED
    // `.tran` extraction (as `pedalkernel-validate/tests/bias_accuracy.rs` does),
    // NOT `.op`; the inc-3 assertion of "moved toward ngspice" must use that.
    assert!((qp.vgk - (-1.037_841)).abs() < TOL, "Vgk={}", qp.vgk);
    assert!((qp.vpk - 180.810_599).abs() < 5e-2, "Vpk={}", qp.vpk);
    assert!((qp.v_cathode - 1.037_841).abs() < TOL, "Vc={}", qp.v_cathode);
    assert!((qp.ia - 691.894e-6).abs() < 1e-6, "Ia={}", qp.ia);

    // Physical sanity: self-biased common-cathode stage sits in the active
    // region (negative grid bias, plate below B+, positive current).
    assert!(qp.vgk < 0.0 && qp.vgk > -5.0);
    assert!(qp.vpk > 0.0 && qp.vpk < 250.0);
    assert!(qp.ia > 0.0);
}

// ─────────────────────────────────────────────────────────────────────────────
// (b') TRIODE unification probe — does the shared `solve_operating_point` /
//      `TriodeSeed` path reproduce the per-type `solve_triode_dc_qpoint`?  This
//      is the ko5g inc-3 migration gate: if the unified path lands the same
//      Q-point, the per-type triode NR loop can delegate to it (unifying triodes
//      onto the same solver core the BJT uses).
// ─────────────────────────────────────────────────────────────────────────────

#[test]
fn probe_triode_unified_vs_per_type_qpoint() {
    let graph = graph_of(TRIODE_12AX7);
    let nl = NonlinearKind::Triode {
        model_name: "12ax7".to_string(),
        plate_node: node(&graph, "V1.plate"),
        cathode_node: node(&graph, "V1.cathode"),
        grid_node: Some(node(&graph, "V1.grid")),
        parallel_count: 1,
        is_vari_mu: false,
    };
    let all_edges: Vec<usize> = (0..graph.edges.len()).collect();

    // Per-type solver (current production path).
    let per_type = super::bias::solve_triode_dc_qpoint(&[nl.clone()], &all_edges, &graph, 250.0)
        .expect("per-type Q-point");

    // Unified shared-core path via the BiasSeed trait.
    let seed = super::bias::TriodeSeed {
        nl_kind: &nl,
        label: "V1".to_string(),
        supply_voltage: 250.0,
        parallel_count: 1,
        flavor: super::bias::TriodeFinderFlavor::DirectThenBfs,
    };
    let nb = super::bias::NetworkBias::default();
    let unified = super::bias::solve_operating_point(&seed, &all_edges, &graph, &nb, 250.0)
        .expect("unified Q-point");

    eprintln!(
        "TRIODE unify: per-type Vgk={:.6} Vpk={:.6} | unified Vgk(control)={:.6} Vpk(warm)={:.6}",
        per_type.vgk, per_type.vpk, unified.control_bias, unified.output_warm_start
    );

    // The shared solver mathematically shares the SAME fixed point
    // (v_ctrl = -Ia*Rk, v_out = VCC - Ia*Rp).  This probe records how close the
    // two implementations land, so the inc-3 migration decision is evidence-based.
    //
    // HISTORY (804f91e6): before the ko5g inc-3 NR-scheme alignment the shared
    // `solve_operating_point` v_ctrl-Newton (with an `i_load_est` degeneration
    // approximation) landed |dVgk| = 1.96 mV / |dVpk| = 131 mV off the per-type
    // Ia-relaxation — a purely ALGORITHMIC residual (independent of v_max).
    //
    // LANDED (ko5g inc-3): `TriodeSeed` now selects
    // `IterationScheme::MainCurrentRelaxation`, which reproduces the per-type
    // Ia-relaxation loop bit-for-bit, so the two land IDENTICAL (dVgk = dVpk = 0).
    // The production `solve_triode_dc_qpoint` non-varimu branch delegates to this
    // shared core; the flip is bit-preserving (corpus failing-set unchanged).
    let dvgk = (per_type.vgk - unified.control_bias).abs();
    let dvpk = (per_type.vpk - unified.output_warm_start).abs();
    eprintln!("TRIODE unify: |dVgk|={dvgk:.6} V  |dVpk|={dvpk:.6} V");

    assert!(unified.control_bias < 0.0, "unified Vgk should be negative");
    assert!(unified.output_warm_start > 0.0 && unified.output_warm_start < 250.0);
    // The shared core is now bit-identical to the per-type solver (the migration
    // is behaviour-preserving).  Pin that: the two must agree to fp rounding.
    assert!(dvgk < 1e-9, "per-type vs unified Vgk must be bit-identical: {dvgk}");
    assert!(dvpk < 1e-9, "per-type vs unified Vpk must be bit-identical: {dvpk}");
}

// ─────────────────────────────────────────────────────────────────────────────
// (a) BJT — synthetic 2N3904 common-emitter, divider-biased.  Pins the unified
//     BJT nodal co-solve `solve_bjt_group_dc_qpoint` (the BA283 solver).
// ─────────────────────────────────────────────────────────────────────────────

const BJT_CE_2N3904: &str = r#"
pedal "NPN CE" {
  supply 9V
  components {
    C_in: cap(1u)
    R1: resistor(47k)
    R2: resistor(10k)
    Q1: npn(2n3904)
    R_c: resistor(2.2k)
    R_e: resistor(1k)
    C_e: cap(47u)
    C_out: cap(1u)
    R_load: resistor(100k)
  }
  nets {
    in -> C_in.a
    C_in.b -> R1.b, R2.a, Q1.base
    vcc -> R1.a, R_c.a
    R2.b -> gnd
    R_c.b -> Q1.collector, C_out.a
    Q1.emitter -> R_e.a, C_e.a
    R_e.b -> gnd
    C_e.b -> gnd
    C_out.b -> R_load.a, out
    R_load.b -> gnd
  }
}"#;

#[test]
fn characterize_bjt_ce_2n3904_qpoint() {
    let graph = graph_of(BJT_CE_2N3904);
    let nl = NonlinearKind::BjtNpn {
        model_name: "2n3904".to_string(),
        base_node: node(&graph, "Q1.base"),
        collector_node: node(&graph, "Q1.collector"),
        emitter_node: node(&graph, "Q1.emitter"),
    };
    let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
    let node_dc = super::bias::solve_bjt_group_dc_qpoint(&[nl], &all_edges, &graph, 9.0)
        .expect("NPN CE stage should have a determinable Q-point");

    let vb = node_dc[&node(&graph, "Q1.base")];
    let vc = node_dc[&node(&graph, "Q1.collector")];
    let ve = node_dc[&node(&graph, "Q1.emitter")];
    let vbe = vb - ve;
    let vce = vc - ve;
    eprintln!("BJT 2N3904 CE @9V: Vb={vb:.6} Vc={vc:.6} Ve={ve:.6} Vbe={vbe:.6} Vce={vce:.6}");

    // GOLDEN — pinned from the current `solve_bjt_group_dc_qpoint` (captured
    // 2026-06-30 on 804f91e6). This is the same solver + terminal/parasitic
    // handling that lands the BA283 TR1 fix-#1 conducting Q-point.
    assert!((vbe - 0.660_166).abs() < 5e-3, "Vbe={vbe}");
    assert!((vce - 6.240_190).abs() < 5e-2, "Vce={vce}");
    assert!(vce > 0.5 && vce < 9.0, "Vce out of active region: {vce}");
    // Conducting fixed point (NOT the spurious cutoff root): Vbe near a silicon
    // forward drop, emitter lifted off ground.
    assert!(vbe > 0.55 && vbe < 0.72, "Vbe not a silicon forward drop: {vbe}");
    assert!(ve > 0.3, "emitter should be lifted (conducting): Ve={ve}");
}

// ─────────────────────────────────────────────────────────────────────────────
// (c) STATIC BIAS — pure VCC resistor divider.  Pins `solve_network_bias`, the
//     divider nodal solve that absorbs `bias_analysis::compute_dc_voltages`.
// ─────────────────────────────────────────────────────────────────────────────

const DIVIDER: &str = r#"
pedal "divider" {
  supply 9V
  components {
    R_v1: resistor(10k)
    R_v2: resistor(20k)
    C_v: cap(47u, electrolytic)
    R_in: resistor(10k)
    U1: opamp(tl072)
    Rf: resistor(100k)
  }
  nets {
    vcc -> R_v1.a
    R_v1.b -> R_v2.a, C_v.a
    R_v2.b -> gnd
    C_v.b -> gnd
    in -> R_in.a
    R_in.b -> U1.neg
    U1.neg -> Rf.a
    Rf.b -> U1.out
    U1.pos -> gnd
    U1.out -> out
  }
}"#;

#[test]
fn characterize_static_divider_voltage() {
    let graph = graph_of(DIVIDER);
    let all_edges: Vec<usize> = (0..graph.edges.len()).collect();
    let nb = super::bias::solve_network_bias(&all_edges, &graph, 9.0);
    let vb = nb.dc_voltages[&node(&graph, "R_v1.b")];
    eprintln!("DIVIDER @9V: v(R_v1.b)={vb:.6}");
    // 9 V * 20k / (10k + 20k) = 6.0 V.
    assert!((vb - 6.0).abs() < TOL, "divider node={vb}");
}
