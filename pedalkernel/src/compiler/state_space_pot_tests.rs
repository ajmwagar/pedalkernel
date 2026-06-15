// ═══════════════════════════════════════════════════════════════════════════
// Direct unit tests for the rigid StateSpace stage builder's pot + mid-chain
// port handling (GAP a + GAP b).
//
// WHY a direct builder test (not a compiled .pedal fixture):
//   `classify_rigid` never returns `RigidOptimization::StateSpace` for a passive
//   group — an all-linear, no-VCVS group always classifies to `Iir` (Rule 3),
//   and the all-passive Rigid / PassiveWdf branches both try
//   `build_passive_rtype_stage` FIRST, which succeeds for every passive pot
//   group we could construct (verified empirically: a mid-chain bridged-T
//   resonator with R/L/C + a 3-terminal pot still compiles to a WDF
//   PassiveRType stage, both with and without `disable_iir`). So no real circuit
//   routes a pot into `build_state_space_stage` today; these paths are latent
//   (as CLAUDE.md notes). To make the fix falsifiable we drive the builder
//   directly on a hand-selected mid-chain passive group containing BOTH a
//   2-terminal rheostat pot and a 3-terminal wiper-divider pot, then assert:
//     (GAP a) both pot kinds bind, and each sweeps the output MONOTONICALLY
//             via `set_pot` -> `recompute_mna` -> `process`;
//     (GAP b) the mid-chain group binds real injection/output ports and is
//             NOT silent (non-zero output for a non-zero input).
// ═══════════════════════════════════════════════════════════════════════════

use super::graph::CircuitGraph;
use super::rigid::build_state_space_stage;

const SR: f64 = 48000.0;

/// Build a CircuitGraph and the set of passive edge indices that form a
/// mid-chain group (everything except the two buffer resistors that touch the
/// global `in`/`out` pads). The middle network has 3 reactive elements (so it
/// classifies as a 3+-reactive StateSpace candidate) plus two pots.
fn midchain_graph() -> (CircuitGraph, Vec<usize>) {
    // in --Rin--> n1 --[ passive network: Rheo (2-term pot a->b),
    //                    C1, C2, C3, R1, R2,
    //                    Lvl (3-term pot a/w/b) ]--> n_out --Rout--> out
    //
    // The middle group touches neither `in` nor `out`, so GAP (b)'s mid-chain
    // terminal derivation must kick in to bind injection/output.
    let src = r#"
pedal "MidChain SS Pots" {
  components {
    Rin: resistor(1k)
    Rheo: pot(20k, b)
    R1: resistor(10k)
    R2: resistor(12k)
    C1: cap(10n)
    C2: cap(22n)
    C3: cap(4.7n)
    Lvl: pot(50k, b)
    Rout: resistor(1k)
  }
  nets {
    in -> Rin.a
    Rin.b -> Rheo.a
    Rheo.b -> R1.a, C1.a
    R1.b -> R2.a, C2.a
    R2.b -> C3.a, Lvl.a
    C1.b -> gnd
    C2.b -> gnd
    C3.b -> gnd
    Lvl.b -> gnd
    Lvl.w -> Rout.a
    Rout.b -> out
  }
  controls {
    Rheo.position -> "Rheo" [0.0, 1.0] = 0.5
    Lvl.position  -> "Lvl"  [0.0, 1.0] = 0.5
  }
}
"#;
    let pedal = crate::dsl::parse_pedal_file(src).expect("parse");
    let graph = CircuitGraph::from_pedal(&pedal);

    // Mid-chain edges = all passive edges EXCEPT the two buffer resistors that
    // touch the global in/out pads. Selecting these as the StateSpace group
    // forces the mid-chain (no global in/out terminal) condition.
    let mid: Vec<usize> = (0..graph.edges.len())
        .filter(|&i| {
            let id = graph.components[graph.edges[i].comp_idx].id.as_str();
            id != "Rin" && id != "Rout"
        })
        .collect();
    (graph, mid)
}

/// Run a settled-state peak measurement of a 440Hz sine through the stage.
fn settled_peak(stage: &mut pedalkernel_rt::stage::StateSpaceStage) -> f64 {
    let freq = 440.0;
    let amp = 0.1;
    for s in 0..2000 {
        let x = amp * (std::f64::consts::TAU * freq * s as f64 / SR).sin();
        stage.process(x);
    }
    let mut peak = 0.0f64;
    let n = (4.0 * SR / freq) as usize;
    for s in 0..n {
        let x = amp * (std::f64::consts::TAU * freq * (2000 + s) as f64 / SR).sin();
        peak = peak.max(stage.process(x).abs());
    }
    peak
}

#[test]
fn state_space_binds_both_pot_kinds_and_midchain_ports() {
    let (graph, mid) = midchain_graph();
    let stage =
        build_state_space_stage(&mid, &[], &graph, SR, 9.0).expect("build StateSpace stage");

    // GAP (b): mid-chain injection + output ports must both bind (not None),
    // otherwise the c-vector is zero and the stage is silent.
    assert!(
        stage.input_binding.is_some(),
        "GAP b: mid-chain injection port should bind"
    );
    assert!(
        stage.output_binding.is_some(),
        "GAP b: mid-chain output port should bind"
    );

    // GAP (a): the 2-terminal rheostat binds once; the 3-terminal wiper pot
    // binds as TWO segment leaves (__aw + __wb).
    let ids: Vec<&str> = stage
        .pot_bindings
        .iter()
        .map(|b| b.comp_id.as_str())
        .collect();
    assert!(
        ids.contains(&"Rheo"),
        "GAP a: 2-terminal pot 'Rheo' should bind (got {ids:?})"
    );
    assert!(
        ids.contains(&"Lvl__aw") && ids.contains(&"Lvl__wb"),
        "GAP a: 3-terminal pot 'Lvl' should bind as __aw + __wb (got {ids:?})"
    );
    assert!(
        stage.has_pot("Rheo") && stage.has_pot("Lvl"),
        "has_pot must report both pots by their public ids"
    );

    // recompute_mna must be present so set_pot can re-stamp.
    assert!(
        stage.recompute_mna.is_some(),
        "recompute_mna must be cached for pot re-stamping"
    );
}

#[test]
fn state_space_midchain_is_not_silent() {
    let (graph, mid) = midchain_graph();
    let mut stage =
        build_state_space_stage(&mid, &[], &graph, SR, 9.0).expect("build StateSpace stage");
    let peak = settled_peak(&mut stage);
    assert!(
        peak.is_finite() && peak > 1e-4,
        "GAP b: mid-chain StateSpace stage must produce output, got peak={peak}"
    );
}

#[test]
fn state_space_two_terminal_pot_sweeps_monotonically() {
    // Rheo is an a->b series rheostat feeding an RC ladder to the Lvl divider.
    // Increasing its resistance raises the series impedance, so the output peak
    // must decrease monotonically as position rises.
    let (graph, mid) = midchain_graph();
    let mut prev = f64::INFINITY;
    let mut peaks = Vec::new();
    for &pos in &[0.05, 0.25, 0.5, 0.75, 0.95] {
        let mut stage = build_state_space_stage(&mid, &[], &graph, SR, 9.0).expect("build");
        // Pin Lvl at full so only Rheo moves.
        stage.set_pot("Lvl", 1.0);
        stage.set_pot("Rheo", pos);
        let peak = settled_peak(&mut stage);
        peaks.push((pos, peak));
        assert!(
            peak <= prev * (1.0 + 1e-6),
            "Rheo sweep not monotonic (decreasing): {peaks:?}"
        );
        prev = peak;
    }
    // And the sweep must actually MOVE the output (not frozen). The signal here
    // is small (RC ladder + series pot), so compare as a RATIO, not absolute.
    let first = peaks.first().unwrap().1;
    let last = peaks.last().unwrap().1;
    assert!(
        first > 1e-9 && (first - last) / first > 0.1,
        "Rheo pot is frozen — sweep produced <10% change: {peaks:?}"
    );
}

#[test]
fn state_space_three_terminal_pot_sweeps_monotonically() {
    // Lvl is a wiper divider: a <- network, w -> out, b -> gnd. The output is
    // the wiper voltage. R_aw = pos·max_r (a→w), R_wb = (1−pos)·max_r (w→gnd).
    // As `pos` rises the wiper moves toward `b` (ground): R_wb shrinks and R_aw
    // grows, so a smaller fraction reaches the wiper → output decreases
    // monotonically. A monotonic, large-range sweep is only possible if BOTH
    // the __aw and __wb segments re-stamp together (GAP a).
    let (graph, mid) = midchain_graph();
    let mut prev = f64::INFINITY;
    let mut peaks = Vec::new();
    for &pos in &[0.05, 0.25, 0.5, 0.75, 0.95] {
        let mut stage = build_state_space_stage(&mid, &[], &graph, SR, 9.0).expect("build");
        // Pin Rheo low (low series R) so the divider sees a stable source.
        stage.set_pot("Rheo", 0.05);
        stage.set_pot("Lvl", pos);
        let peak = settled_peak(&mut stage);
        peaks.push((pos, peak));
        assert!(
            peak <= prev * (1.0 + 1e-6),
            "Lvl wiper sweep not monotonic (decreasing): {peaks:?}"
        );
        prev = peak;
    }
    let first = peaks.first().unwrap().1;
    let last = peaks.last().unwrap().1;
    // Strong, full-range divider action (here ~19x) proves both segments track.
    assert!(
        first > 1e-9 && (first - last) / first > 0.5,
        "Lvl wiper pot is frozen / single-segment — weak sweep: {peaks:?}"
    );
}
