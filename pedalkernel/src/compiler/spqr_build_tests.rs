// Tests extracted from spqr_build.rs — stage building + end-to-end pipeline.

use super::compiled::Stage;
use super::graph::CircuitGraph;
use super::spqr::{spqr_decompose, spqr_to_stages, SpqrStage};
use super::spqr_build::*;
use crate::PedalProcessor;

fn make_graph_all_edges(pedal_src: &str) -> (CircuitGraph, Vec<usize>) {
    let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&pedal);
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();
    (graph, all_edges)
}

fn make_graph_passive(pedal_src: &str) -> (CircuitGraph, Vec<usize>) {
    let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&pedal);
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let passive_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .filter(|&i| graph.components[graph.edges[i].comp_idx].kind.is_passive())
        .collect();
    (graph, passive_edges)
}

#[test]
fn spqr_diode_clipper_produces_audio() {
    let (graph, edges) = make_graph_all_edges(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(4.7k)  D1: diode(silicon) }
            nets { in -> R1.a  R1.b -> D1.a  D1.b -> gnd }
            controls {}
        }"#,
    );
    let spqr = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    let spqr_stages = spqr_to_stages(&spqr, &graph, 48000.0);

    assert_eq!(spqr_stages.len(), 1);
    let mut stage = build_spqr_stage(spqr_stages.into_iter().next().unwrap(), &graph, 48000.0)
        .expect("Should build NlWdf stage")
        .into_wdf();

    // DC test: 5V input should clip to ~0.6V (silicon diode forward voltage)
    let dc_out = stage.process(5.0);
    assert!(dc_out < 1.5, "5V DC should clip to <1.5V, got {dc_out:.4}V");
    assert!(
        dc_out > 0.1,
        "5V DC should produce output, got {dc_out:.4}V"
    );

    // Negative DC: single diode doesn't clip reverse bias → passes through
    let neg_out = stage.process(-5.0);
    assert!(
        neg_out.abs() > 2.0,
        "Reverse bias should pass through, got {neg_out:.4}V"
    );

    // Sine wave: positive peaks clipped, negative peaks pass through
    let mut pos_peak = 0.0f64;
    let mut neg_peak = 0.0f64;
    for i in 0..960 {
        let input = 5.0 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin();
        let output = stage.process(input);
        pos_peak = pos_peak.max(output);
        neg_peak = neg_peak.min(output);
    }

    // SingleDiode clips forward (positive) direction only
    assert!(
        pos_peak < 1.5,
        "Forward bias should clip: pos_peak={pos_peak:.3}V"
    );
    assert!(
        neg_peak < -1.0,
        "Reverse bias should pass: neg_peak={neg_peak:.3}V"
    );
}

#[test]
fn spqr_passive_rc_produces_audio() {
    // End-to-end: RC lowpass should pass DC through.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  C1: cap(100n) }
            nets { in -> R1.a  R1.b -> C1.a, out  C1.b -> gnd }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    // Process step input — capacitor should charge (lowpass)
    let mut output = 0.0;
    for _ in 0..480 {
        output = compiled.process(1.0);
    }

    // After 10ms with RC = 1ms, cap should be mostly charged
    assert!(
        output.abs() > 0.001,
        "RC lowpass should pass DC, got {output:.6}"
    );
}

#[test]
fn spqr_inverting_opamp_gain() {
    // Inverting amp: R1=10k, Rf=100k → gain = -10
    // Goes through full compile_via_spqr pipeline (signal flow groups include VCVS).
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                Rf: resistor(100k)
                U1: opamp(tl072)
            }
            nets {
                in -> R1.a
                R1.b -> U1.neg
                Rf.a -> U1.neg
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    // Small signal: 0.01V input → should get ~0.1V output (gain=10, inverted)
    for _ in 0..20 {
        compiled.process(0.01);
    }
    let output = compiled.process(0.01);
    let gain_measured = output.abs() / 0.01;

    assert!(
        gain_measured > 5.0,
        "Inverting gain should be ~10, got {gain_measured:.2}"
    );
    assert!(
        gain_measured < 15.0,
        "Inverting gain should be ~10, got {gain_measured:.2}"
    );
}

#[test]
fn compile_via_spqr_diode_clipper_end_to_end() {
    use crate::PedalProcessor;

    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test_clipper" { supply 9V
            components { R1: resistor(4.7k)  D1: diode(silicon) }
            nets { in -> R1.a  R1.b -> D1.a  D1.b -> gnd }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("Should compile via SPQR");

    // Process through PedalProcessor trait
    let dc_out = compiled.process(5.0);
    assert!(
        dc_out.abs() < 2.0,
        "Diode should clip 5V input, got {dc_out:.4}"
    );

    // Process sine
    let mut peak = 0.0f64;
    for i in 0..480 {
        let input = 1.0 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin();
        let output = compiled.process(input);
        peak = peak.max(output.abs());
    }
    assert!(peak > 0.01, "Should produce output: {peak:.6}");
}

#[test]
fn compile_via_spqr_inverting_opamp_end_to_end() {
    use crate::PedalProcessor;

    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test_inv" { supply 9V
            components {
                R1: resistor(10k)
                Rf: resistor(100k)
                U1: opamp(tl072)
            }
            nets {
                in -> R1.a
                R1.b -> U1.neg
                Rf.a -> U1.neg
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled =
        compile_via_spqr(&pedal, 48000.0).expect("Should compile inverting amp via SPQR");

    // Settle
    for _ in 0..10 {
        compiled.process(0.1);
    }
    let output = compiled.process(0.1);
    let gain = output.abs() / 0.1;
    assert!(
        gain > 5.0 && gain < 15.0,
        "Inverting gain should be ~10, got {gain:.2}"
    );
}

#[test]
fn compile_via_spqr_opamp_diode_feedback() {
    // TS-style: R1 → opamp(neg) ← Rf ← diode ← opamp(out)
    // OpAmpRoot pre-amplifies, diode clips
    use crate::PedalProcessor;

    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test_ts" { supply 9V
            components {
                R1: resistor(4.7k)
                D1: diode(silicon)
                Rf: resistor(51k)
                U1: opamp(jrc4558)
            }
            nets {
                in -> R1.a
                R1.b -> U1.neg
                D1.a -> U1.neg
                D1.b -> U1.out
                Rf.a -> U1.neg
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled =
        compile_via_spqr(&pedal, 48000.0).expect("Should compile TS-style circuit via SPQR");

    // Small signal should be amplified (gain ≈ Rf/Ri ≈ 51k/4.7k ≈ 10.8)
    for _ in 0..20 {
        compiled.process(0.01);
    }
    let output = compiled.process(0.01);
    assert!(
        output.abs() > 0.02,
        "Should amplify small signal, got {output:.6}"
    );

    // Large signal should clip (diode limits output)
    for _ in 0..20 {
        compiled.process(0.5);
    }
    let loud_output = compiled.process(0.5);
    // Gain of ~11 would give 5.5V, but diode clips at ~0.6V
    assert!(
        loud_output.abs() < 3.0,
        "Diode should clip large signal, got {loud_output:.4}V"
    );
}

#[test]
fn compile_via_spqr_808_kick() {
    // 808 bass drum: bridged-T resonator with op-amp
    // Should classify as Iir (VCVS + reactive, all linear)
    use crate::PedalProcessor;

    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "808 BD" { supply 9V
            components {
                U1: opamp(tl072)
                Rb1: resistor(100k)
                Rb2: resistor(100k)
                R1: resistor(150k)
                R2: resistor(150k)
                C1: cap(8.2n)
                C2: cap(8.2n)
                R_fb: resistor(470k)
                R_trig: resistor(100k)
                R_out: resistor(10k)
            }
            nets {
                vcc -> Rb1.a
                Rb1.b -> Rb2.a, U1.pos
                Rb2.b -> gnd
                U1.neg -> R1.a, C1.a
                R1.b -> R2.a, C2.a
                R2.b -> U1.out
                C1.b -> gnd
                C2.b -> gnd
                U1.neg -> R_fb.a
                R_fb.b -> U1.out
                in -> R_trig.a
                R_trig.b -> R1.b
                U1.out -> R_out.a
                R_out.b -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("Should compile 808 kick via SPQR");

    // Warmup
    for _ in 0..480 {
        compiled.process(0.0);
    }

    // Fire impulse and collect output
    let n_samples = 48000;
    let mut output = Vec::with_capacity(n_samples);
    for i in 0..n_samples {
        let input = if i == 0 { 1.0 } else { 0.0 };
        output.push(compiled.process(input));
    }

    let peak = output.iter().map(|s| s.abs()).fold(0.0_f64, f64::max);
    assert!(
        peak > 0.001,
        "808 kick should produce output, peak={peak:.6}"
    );

    // Count zero crossings in first 500ms to estimate frequency
    let analysis = 24000; // 500ms
    let zc: u32 = (1..analysis.min(output.len()))
        .filter(|&i| output[i] * output[i - 1] < 0.0)
        .count() as u32;
    let freq = zc as f64 / 2.0 / 0.5;

    // 808 kick should resonate around 130Hz (±30%)
    eprintln!("808 kick: peak={peak:.4}, freq={freq:.1}Hz");
    assert!(
        freq > 90.0 && freq < 170.0,
        "808 kick frequency should be ~130Hz, got {freq:.1}Hz"
    );
}

#[test]
fn compile_via_spqr_808_kick_v2_bjt_sweep_compiles() {
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "808 Kick v2" {
            supply 9V
            components {
                U1: opamp(tl072)
                Rb1: resistor(100k)
                Rb2: resistor(100k)
                R1: resistor(150k)
                R2: resistor(150k)
                C1: cap(8.2n)
                C2: cap(8.2n)
                R_fb: resistor(470k)
                R_trig: resistor(1k)
                Q_sweep: npn(2n3904)
                R_base: resistor(47k)
                C_env: cap(100n)
                R_env: resistor(100k)
                Decay: pot(500k, b)
                U2: opamp(tl072)
                Ri_out: resistor(10k)
                Rf_out: resistor(470k)
                R_click: resistor(470k)
                C_click: cap(100p)
            }
            nets {
                vcc -> Rb1.a
                Rb1.b -> Rb2.a, U1.pos
                Rb2.b -> gnd
                U1.neg -> R1.a, C1.a
                R1.b -> R2.a, C2.a
                R2.b -> U1.out
                C1.b -> gnd
                C2.b -> gnd
                U1.neg -> R_fb.a
                R_fb.b -> U1.out
                U1.neg -> Q_sweep.collector
                Q_sweep.emitter -> R1.b
                in -> C_env.a
                C_env.b -> R_base.a
                R_base.b -> Q_sweep.base
                Q_sweep.base -> R_env.a
                R_env.b -> gnd
                in -> R_trig.a
                R_trig.b -> R1.b
                R1.b -> Decay.a
                Decay.b -> gnd
                in -> R_click.a
                R_click.b -> C_click.a
                C_click.b -> Ri_out.b
                U1.out -> Ri_out.a
                Ri_out.b -> U2.neg
                U2.neg -> Rf_out.a
                Rf_out.b -> U2.out
                U2.pos -> gnd
                U2.out -> out
            }
            controls {
                Decay.position -> "Decay" [0.0, 1.0] = 1.0
            }
        }"#,
    )
    .expect("parse");

    let compiled =
        compile_via_spqr(&pedal, 48000.0).expect("808 Kick v2 with BJT sweep should compile");

    assert!(
        compiled
            .stages
            .iter()
            .any(|stage| matches!(stage, Stage::MultiNl(_))),
        "Q_sweep BJT should route through MultiNlStage, not a WDF nonlinear root"
    );
    assert!(
        compiled.stages.len() > 1,
        "BJT sweep must not collapse the full 808 kick into a single generic stage"
    );

    let serial_stages: Vec<&Stage> = compiled
        .stages
        .iter()
        .filter(|stage| match stage {
            Stage::Wdf(stage) => !stage.bypass_serial,
            Stage::MultiNl(stage) => !stage.bypass_serial,
            Stage::Iir(stage) => !stage.bypass_serial,
            Stage::StateSpace(stage) => !stage.bypass_serial,
            Stage::BlackFeedback(stage) => !stage.bypass_serial,
        })
        .collect();
    assert!(
        matches!(serial_stages.first(), Some(Stage::Iir(_))),
        "U1 bridged-T resonator should be first serial stage; got {:?}",
        compiled.stages
    );
    assert!(
        matches!(serial_stages.last(), Some(Stage::BlackFeedback(_))),
        "U2 output amplifier should be last serial stage; got {:?}",
        compiled.stages
    );
}

#[test]
fn try_all_legends_pedals() {
    let pedal_dir = std::path::Path::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../pedalkernel-pro/pedals/legends"
    ));
    if !pedal_dir.exists() {
        eprintln!("Skipping: legends directory not found");
        return;
    }
    let mut pass = 0;
    let mut fail = 0;
    for entry in std::fs::read_dir(pedal_dir).unwrap() {
        let path = entry.unwrap().path();
        if path.extension().map_or(true, |e| e != "pedal") {
            continue;
        }
        let name = path.file_stem().unwrap().to_string_lossy().to_string();
        let source = std::fs::read_to_string(&path).unwrap();
        let pedal = match crate::dsl::parse_pedal_file(&source) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("✗ {name}: parse error: {e}");
                fail += 1;
                continue;
            }
        };
        match compile_via_spqr(&pedal, 48000.0) {
            Ok(_) => {
                eprintln!("✓ {name}");
                pass += 1;
            }
            Err(e) => {
                eprintln!("✗ {name}: {e}");
                fail += 1;
            }
        }
    }
    eprintln!("\n{pass} passed, {fail} failed");
    // Don't assert — just report. We're tracking coverage.
}

// ═══════════════════════════════════════════════════════════════════════════
// Audio output regression tests — each legend pedal must produce non-zero output
// ═══════════════════════════════════════════════════════════════════════════

fn load_legend(name: &str) -> crate::dsl::PedalDef {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/{}.pedal",
        env!("CARGO_MANIFEST_DIR"),
        name
    );
    let source = std::fs::read_to_string(&path).unwrap_or_else(|_| panic!("Can't read {path}"));
    crate::dsl::parse_pedal_file(&source).unwrap_or_else(|e| panic!("{name}: parse error: {e}"))
}

fn assert_produces_audio(name: &str, pedal: &crate::dsl::PedalDef) {
    let mut compiled =
        compile_via_spqr(pedal, 48000.0).unwrap_or_else(|e| panic!("{name}: compile error: {e}"));

    eprintln!(
        "{name}: wdf={}, iir={}, ss={}, mnl={}, order={:?}",
        compiled.stages.len(),
        compiled
            .stages
            .iter()
            .filter(|s| matches!(s, Stage::Iir(_)))
            .count(),
        compiled
            .stages
            .iter()
            .filter(|s| matches!(s, Stage::StateSpace(_)))
            .count(),
        compiled
            .stages
            .iter()
            .filter(|s| matches!(s, Stage::MultiNl(_)))
            .count(),
        compiled.stages
    );

    // Settle DC
    for _ in 0..2000 {
        compiled.process(0.0);
    }

    // Feed guitar-level 440Hz signal
    let mut peak = 0.0f64;
    for i in 0..4800 {
        let input = 0.05 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin();
        let output = compiled.process(input);
        peak = peak.max(output.abs());
    }

    eprintln!("{name}: peak={peak:.6}V");
    assert!(peak.is_finite(), "{name}: output is NaN/Inf");
    assert!(
        peak > 0.001,
        "{name}: ZERO OUTPUT regression — peak={peak:.6}V"
    );
}

#[test]
fn legend_goldenrod_produces_audio() {
    let pedal = load_legend("goldenrod");
    assert_produces_audio("goldenrod", &pedal);
}

#[test]
fn legend_ratking_produces_audio() {
    let pedal = load_legend("ratking");
    assert_produces_audio("ratking", &pedal);
}

#[test]
fn legend_sd1_produces_audio() {
    let pedal = load_legend("sd1");
    assert_produces_audio("sd1", &pedal);
}

#[test]
fn legend_screamer_produces_audio() {
    let pedal = load_legend("screamer");
    assert_produces_audio("screamer", &pedal);
}

#[test]
fn legend_muff_produces_audio() {
    let pedal = load_legend("muff");
    assert_produces_audio("muff", &pedal);
}

#[test]
fn legend_blues_produces_audio() {
    let pedal = load_legend("blues");
    assert_produces_audio("blues", &pedal);
}

#[test]
fn legend_fizz_produces_audio() {
    let pedal = load_legend("fizz");
    assert_produces_audio("fizz", &pedal);
}

#[test]
fn legend_rangemaster_produces_audio() {
    let pedal = load_legend("rangemaster");
    assert_produces_audio("rangemaster", &pedal);
}

#[test]
fn legend_shrine_produces_audio() {
    let pedal = load_legend("shrine");
    assert_produces_audio("shrine", &pedal);
}

/// Diagnostic: measure frequency response to find HPF behavior.
/// Runs the full pipeline at two frequencies and compares levels.
#[test]
fn diagnose_ratking_hpf() {
    let pedal = load_legend("ratking_non_invert_v1a");

    eprintln!("\nRATKING HPF diagnosis:");

    // Measure at 200Hz (should be loud for a distortion pedal)
    let mut compiled_lo = compile_via_spqr(&pedal, 48000.0).expect("compile");
    for _ in 0..2000 {
        compiled_lo.process(0.0);
    }
    let mut peak_lo = 0.0f64;
    for s in 0..4800 {
        let input = 0.05 * (2.0 * std::f64::consts::PI * 200.0 * s as f64 / 48000.0).sin();
        peak_lo = peak_lo.max(compiled_lo.process(input).abs());
    }

    // Measure at 5kHz
    let mut compiled_hi = compile_via_spqr(&pedal, 48000.0).expect("compile");
    for _ in 0..2000 {
        compiled_hi.process(0.0);
    }
    let mut peak_hi = 0.0f64;
    for s in 0..4800 {
        let input = 0.05 * (2.0 * std::f64::consts::PI * 5000.0 * s as f64 / 48000.0).sin();
        peak_hi = peak_hi.max(compiled_hi.process(input).abs());
    }

    let ratio = if peak_lo > 1e-10 {
        peak_hi / peak_lo
    } else {
        f64::INFINITY
    };
    eprintln!("  200Hz peak={peak_lo:.6}V, 5kHz peak={peak_hi:.6}V, ratio={ratio:.1}x");
    eprintln!("  stages: {:?}", compiled_lo.stages);
    for (i, s) in compiled_lo.stages.iter().enumerate() {
        if let Stage::Wdf(wdf) = s {
            eprintln!(
                "  [{i}] Wdf: rp={:.0} comp={}",
                wdf.tree.port_resistance(),
                wdf.root_comp_id
            );
        } else {
            eprintln!("  [{i}] {:?}", s);
        }
    }

    // A distortion pedal should NOT have 10x more output at 5kHz vs 200Hz
    assert!(
        ratio < 10.0,
        "HPF detected: 5kHz/200Hz ratio={ratio:.1}x (should be <10x for a distortion pedal)"
    );
    // Both frequencies should produce audible output
    assert!(peak_lo > 0.001, "200Hz should produce output: {peak_lo:.6}");
    assert!(peak_hi > 0.001, "5kHz should produce output: {peak_hi:.6}");
}

/// Same HPF test for screamer
#[test]
fn diagnose_screamer_hpf() {
    let pedal = load_legend("screamer");
    let mut compiled_lo = compile_via_spqr(&pedal, 48000.0).expect("compile");
    for _ in 0..2000 {
        compiled_lo.process(0.0);
    }
    let mut peak_lo = 0.0f64;
    for s in 0..4800 {
        let input = 0.05 * (2.0 * std::f64::consts::PI * 200.0 * s as f64 / 48000.0).sin();
        peak_lo = peak_lo.max(compiled_lo.process(input).abs());
    }
    let mut compiled_hi = compile_via_spqr(&pedal, 48000.0).expect("compile");
    for _ in 0..2000 {
        compiled_hi.process(0.0);
    }
    let mut peak_hi = 0.0f64;
    for s in 0..4800 {
        let input = 0.05 * (2.0 * std::f64::consts::PI * 5000.0 * s as f64 / 48000.0).sin();
        peak_hi = peak_hi.max(compiled_hi.process(input).abs());
    }
    let ratio = if peak_lo > 1e-10 {
        peak_hi / peak_lo
    } else {
        f64::INFINITY
    };
    eprintln!("SCREAMER: 200Hz={peak_lo:.6}, 5kHz={peak_hi:.6}, ratio={ratio:.1}x");
    assert!(ratio < 10.0, "HPF detected: ratio={ratio:.1}x");
    assert!(peak_lo > 0.001, "200Hz should produce output: {peak_lo:.6}");
}

// ═══════════════════════════════════════════════════════════════════════════
// compute_group_terminals: verify correct boundary node detection
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn compute_group_terminals_t_junction_after_opamp() {
    // Circuit: in → R_in → U1.neg [Rf feedback] → U1.out → R_tone → (C_tone→gnd, R_out→out)
    // The tone stack group (R_tone, C_tone, R_out) should have 2 terminals:
    // - U1.out (entry from previous stage, voltage source barrier)
    // - out (exit to circuit output)
    // The junction node (R_tone.b = C_tone.a = R_out.a) is INTERNAL — not a terminal.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                R_tone: resistor(10k)
                C_tone: cap(22n)
                R_out: resistor(10k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> R_tone.a
                R_tone.b -> C_tone.a
                C_tone.b -> gnd
                R_tone.b -> R_out.a
                R_out.b -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let graph = super::graph::CircuitGraph::from_pedal(&pedal);
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    // Find the tone stack edges
    let tone_edges: Vec<usize> = all_edges
        .iter()
        .copied()
        .filter(|&eidx| {
            let comp = &graph.components[graph.edges[eidx].comp_idx];
            comp.id == "R_tone" || comp.id == "C_tone" || comp.id == "R_out"
        })
        .collect();

    eprintln!("Tone stack edges:");
    for &eidx in &tone_edges {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        eprintln!(
            "  {} ({:?}): node {} → node {}",
            comp.id,
            graph.effective_edge_kind(eidx),
            e.node_a,
            e.node_b
        );
    }
    eprintln!(
        "  gnd={}, in={}, out={}",
        graph.gnd_node, graph.in_node, graph.out_node
    );

    let global_terminals = vec![graph.in_node, graph.out_node];
    let terminals =
        super::spqr_build::compute_group_terminals(&tone_edges, &graph, &global_terminals);

    eprintln!("Computed terminals: {:?}", terminals);

    // Should have exactly 2 non-GND terminals:
    // 1. The node shared with U1.out (entry)
    // 2. The out node (exit)
    // GND may be included for ground-shunt detection.
    let non_gnd_terminals: Vec<_> = terminals.iter().filter(|&&n| n != graph.gnd_node).collect();

    assert!(
        non_gnd_terminals.len() <= 2,
        "Should have ≤2 non-GND terminals (entry + exit), got {:?}",
        terminals
    );

    // Now verify SPQR with these terminals produces PassiveWdf, not Rigid
    let spqr_tree = super::spqr::spqr_decompose(&tone_edges, &terminals, &graph, graph.gnd_node);
    let stages = super::spqr::spqr_to_stages(&spqr_tree, &graph, 48000.0);
    eprintln!("SPQR stages with computed terminals:");
    for (i, s) in stages.iter().enumerate() {
        eprintln!("  [{i}]: {s:?}");
    }
    let has_rigid = stages
        .iter()
        .any(|s| matches!(s, super::spqr::SpqrStage::Rigid { .. }));
    assert!(
        !has_rigid,
        "T-junction with correct terminals should NOT produce Rigid stage"
    );
}

#[test]
fn compute_group_terminals_simple_series() {
    // Simple series chain: in → R1 → R2 → out
    // Terminals should be [in, out]
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  R2: resistor(10k) }
            nets { in -> R1.a  R1.b -> R2.a  R2.b -> out }
            controls {}
        }"#,
    )
    .expect("parse");

    let graph = super::graph::CircuitGraph::from_pedal(&pedal);
    let edges: Vec<usize> = (0..graph.edges.len())
        .filter(|&i| !graph.active_edge_indices.contains(&i))
        .collect();

    let global_terminals = vec![graph.in_node, graph.out_node];
    let terminals = super::spqr_build::compute_group_terminals(&edges, &graph, &global_terminals);

    eprintln!("Series chain terminals: {:?}", terminals);
    assert!(terminals.contains(&graph.in_node), "Should include in_node");
    assert!(
        terminals.contains(&graph.out_node),
        "Should include out_node"
    );
}

#[test]
fn spqr_noninverting_opamp_gain() {
    // Non-inverting amp: R1=10k, Rf=100k → gain = 1 + 100k/10k = 11
    // Goes through full compile_via_spqr pipeline (signal flow groups include VCVS).
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                Rf: resistor(100k)
                U1: opamp(tl072)
            }
            nets {
                in -> U1.pos
                U1.neg -> R1.a
                R1.b -> gnd
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.out -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    for _ in 0..20 {
        compiled.process(0.01);
    }
    let output = compiled.process(0.01);
    let gain_measured = output.abs() / 0.01;

    assert!(
        gain_measured > 6.0,
        "Non-inverting gain should be ~11, got {gain_measured:.2}"
    );
    assert!(
        gain_measured < 16.0,
        "Non-inverting gain should be ~11, got {gain_measured:.2}"
    );
}

#[test]
fn diagnose_ratking() {
    let source = std::fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../pedalkernel-pro/pedals/legends/ratking.pedal"
    ))
    .expect("read ratking.pedal");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");

    let graph = super::graph::CircuitGraph::from_pedal(&pedal);
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    eprintln!("\nRATKING diagnosis:");
    eprintln!("  Total edges: {}", all_edges.len());
    for &eidx in &all_edges {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        let ek = graph.effective_edge_kind(eidx);
        eprintln!(
            "    edge {eidx}: {} ({:?}) nodes {}→{}",
            comp.id, ek, e.node_a, e.node_b
        );
    }

    let terminals = vec![graph.in_node, graph.out_node];
    let spqr_tree = spqr_decompose(&all_edges, &terminals, &graph, graph.gnd_node);
    eprintln!("\n  SPQR tree: {spqr_tree:?}");
    let spqr_stages = spqr_to_stages(&spqr_tree, &graph, 48000.0);

    eprintln!("\n  SPQR stages: {}", spqr_stages.len());
    for (i, s) in spqr_stages.iter().enumerate() {
        eprintln!("    stage {i}: {s:?}");
    }

    // Also check what classify_rigid says
    for (i, s) in spqr_stages.iter().enumerate() {
        if let SpqrStage::Rigid { edge_indices, .. } = s {
            let stats = super::rigid::StageStats::from_edges(edge_indices, &graph);
            let opt = super::rigid::classify_rigid(&stats, &graph, None);
            eprintln!(
                "    stage {i} rigid: {:?} (stats: vcvs={}, nl={}, linear={}, reactive={})",
                opt, stats.vcvs_count, stats.nl_count, stats.linear_count, stats.reactive_count
            );
        }
    }
}

#[test]
fn compile_via_spqr_state_space_active_filter() {
    // An active filter with 3 caps that can't reduce to biquad.
    // Should compile via StateSpace and produce filtered output.
    use crate::PedalProcessor;

    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                C1: cap(100n)
                C2: cap(47n)
                C3: cap(22n)
            }
            nets {
                in -> R1.a
                R1.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                C1.a -> U1.neg
                C1.b -> gnd
                C2.a -> U1.out
                C2.b -> gnd
                C3.a -> U1.neg
                C3.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled =
        compile_via_spqr(&pedal, 48000.0).expect("Should compile active filter via StateSpace");

    // Should produce filtered output (not silence)
    for _ in 0..100 {
        compiled.process(0.1);
    }
    let output = compiled.process(0.1);
    assert!(
        output.abs() > 0.001,
        "StateSpace should produce output, got {output:.6}"
    );
}

#[test]
fn compile_via_spqr_single_bjt_stage() {
    // Simple common-emitter BJT amp: should compile via General MNA
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                C_in: cap(1u)
                R_b: resistor(100k)
                Q1: npn(2n3904)
                R_c: resistor(10k)
                R_e: resistor(1k)
                C_out: cap(10u)
            }
            nets {
                in -> C_in.a
                C_in.b -> R_b.a, Q1.base
                R_b.b -> vcc
                Q1.collector -> R_c.a
                R_c.b -> vcc
                Q1.emitter -> R_e.a
                R_e.b -> gnd
                Q1.collector -> C_out.a
                C_out.b -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("Should compile BJT stage");

    // Should produce amplified output
    for _ in 0..100 {
        compiled.process(0.01);
    }
    let output = compiled.process(0.01);
    assert!(
        output.abs() > 0.0001,
        "BJT should produce output: {output:.6}"
    );
}
