// Tests extracted from spqr_build.rs — stage building + end-to-end pipeline.

use super::graph::CircuitGraph;
use super::spqr::{spqr_decompose, spqr_to_stages, SpqrStage};
use super::spqr_build::*;

fn make_graph_all_edges(
    pedal_src: &str,
) -> (CircuitGraph, Vec<usize>) {
    let pedal = crate::dsl::parse_pedal_file(pedal_src).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&pedal);
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();
    (graph, all_edges)
}

fn make_graph_passive(
    pedal_src: &str,
) -> (CircuitGraph, Vec<usize>) {
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
    let (graph, edges) = make_graph_all_edges(r#"
        pedal "test" { supply 9V
            components { R1: resistor(4.7k)  D1: diode(silicon) }
            nets { in -> R1.a  R1.b -> D1.a  D1.b -> gnd }
            controls {}
        }"#);
    let spqr = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    let spqr_stages = spqr_to_stages(&spqr, &graph, 48000.0);

    assert_eq!(spqr_stages.len(), 1);
    let mut stage = build_spqr_stage(
        spqr_stages.into_iter().next().unwrap(),
        &graph,
        48000.0,
    )
    .expect("Should build NlWdf stage")
    .into_wdf();

    // DC test: 5V input should clip to ~0.6V (silicon diode forward voltage)
    let dc_out = stage.process(5.0);
    assert!(
        dc_out < 1.5,
        "5V DC should clip to <1.5V, got {dc_out:.4}V"
    );
    assert!(dc_out > 0.1, "5V DC should produce output, got {dc_out:.4}V");

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
        let input =
            5.0 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin();
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
    let (graph, edges) = make_graph_passive(r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  C1: cap(100n) }
            nets { in -> R1.a  R1.b -> C1.a  C1.b -> gnd }
            controls {}
        }"#);
    let spqr = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    let spqr_stages = spqr_to_stages(&spqr, &graph, 48000.0);

    assert_eq!(spqr_stages.len(), 1);
    let mut stage = build_spqr_stage(
        spqr_stages.into_iter().next().unwrap(),
        &graph,
        48000.0,
    )
    .expect("Should build PassiveWdf stage")
    .into_wdf();

    // Process step input — capacitor should charge (lowpass)
    let mut output = 0.0;
    for _ in 0..480 {
        output = stage.process(1.0);
    }

    // After 10ms with RC = 1ms, cap should be mostly charged
    assert!(output.abs() > 0.001, "RC lowpass should pass DC, got {output:.6}");
}

#[test]
fn spqr_inverting_opamp_gain() {
    // Inverting amp: R1=10k, Rf=100k → gain = -10
    let (graph, edges) = make_graph_all_edges(r#"
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
        }"#);
    let spqr = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    let spqr_stages = spqr_to_stages(&spqr, &graph, 48000.0);

    assert_eq!(spqr_stages.len(), 1);
    let mut stage = build_spqr_stage(
        spqr_stages.into_iter().next().unwrap(),
        &graph,
        48000.0,
    )
    .expect("Should build OpAmpRoot stage")
    .into_wdf();

    // Small signal: 0.1V input → should get ~1.0V output (gain=10, inverted)
    // Let the stage settle for a few samples
    for _ in 0..10 {
        stage.process(0.1);
    }
    let output = stage.process(0.1);
    let gain_measured = output.abs() / 0.1;

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

    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test_clipper" { supply 9V
            components { R1: resistor(4.7k)  D1: diode(silicon) }
            nets { in -> R1.a  R1.b -> D1.a  D1.b -> gnd }
            controls {}
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0)
        .expect("Should compile via SPQR");

    // Process through PedalProcessor trait
    let dc_out = compiled.process(5.0);
    assert!(
        dc_out.abs() < 2.0,
        "Diode should clip 5V input, got {dc_out:.4}"
    );

    // Process sine
    let mut peak = 0.0f64;
    for i in 0..480 {
        let input =
            1.0 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin();
        let output = compiled.process(input);
        peak = peak.max(output.abs());
    }
    assert!(peak > 0.01, "Should produce output: {peak:.6}");
}

#[test]
fn compile_via_spqr_inverting_opamp_end_to_end() {
    use crate::PedalProcessor;

    let pedal = crate::dsl::parse_pedal_file(r#"
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
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0)
        .expect("Should compile inverting amp via SPQR");

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

    let pedal = crate::dsl::parse_pedal_file(r#"
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
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0)
        .expect("Should compile TS-style circuit via SPQR");

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

    let pedal = crate::dsl::parse_pedal_file(r#"
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
        }"#)
    .expect("parse");

    // Debug: show feedback groups
    {
        use super::feedback::find_feedback_groups;
        let graph = super::graph::CircuitGraph::from_pedal(&pedal);
        let active_set: std::collections::HashSet<usize> =
            graph.active_edge_indices.iter().copied().collect();
        let all_edges: Vec<usize> = (0..graph.edges.len())
            .filter(|i| !active_set.contains(i))
            .collect();
        let groups = find_feedback_groups(&all_edges, &graph);
        for (i, g) in groups.iter().enumerate() {
            let names: Vec<String> = g.all_edges().iter()
                .map(|&eidx| {
                    let comp = &graph.components[graph.edges[eidx].comp_idx];
                    format!("{}({:?})", comp.id, graph.effective_edge_kind(eidx))
                })
                .collect();
            eprintln!("808 group {i}: active={} fb={} pendant={} gnd={} :: {:?}",
                g.active_edges.len(), g.feedback_edges.len(),
                g.pendant_edges.len(), g.ground_shunt_edges.len(), names);
        }
    }

    let mut compiled = compile_via_spqr(&pedal, 48000.0)
        .expect("Should compile 808 kick via SPQR");

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
    assert!(peak > 0.001, "808 kick should produce output, peak={peak:.6}");

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
fn try_all_legends_pedals() {
    let pedal_dir = std::path::Path::new(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../pedalkernel-pro/pedals/legends")
    );
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

#[test]
fn spqr_noninverting_opamp_gain() {
    // Non-inverting amp: R1=10k, Rf=100k → gain = 1 + 100k/10k = 11
    let (graph, edges) = make_graph_all_edges(r#"
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
        }"#);
    let spqr = spqr_decompose(
        &edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );
    let spqr_stages = spqr_to_stages(&spqr, &graph, 48000.0);

    assert_eq!(spqr_stages.len(), 1);
    let mut stage = build_spqr_stage(
        spqr_stages.into_iter().next().unwrap(),
        &graph,
        48000.0,
    )
    .expect("Should build OpAmpRoot stage")
    .into_wdf();

    for _ in 0..10 {
        stage.process(0.1);
    }
    let output = stage.process(0.1);
    let gain_measured = output.abs() / 0.1;

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
    let source = std::fs::read_to_string(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../pedalkernel-pro/pedals/legends/ratking.pedal")
    ).expect("read ratking.pedal");
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
        eprintln!("    edge {eidx}: {} ({:?}) nodes {}→{}", comp.id, ek, e.node_a, e.node_b);
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
            eprintln!("    stage {i} rigid: {:?} (stats: vcvs={}, nl={}, linear={}, reactive={})",
                opt, stats.vcvs_count, stats.nl_count, stats.linear_count, stats.reactive_count);
        }
    }
}
