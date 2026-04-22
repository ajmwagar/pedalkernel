// TDD tests for the zero-output regression.
//
// Traces exactly where signal dies in the compilation pipeline.
// Each test isolates one layer: grouping → SPQR → stage type → audio.

use super::spqr_build::compile_via_spqr;
use super::spqr::*;
use super::graph::CircuitGraph;
use crate::PedalProcessor;

fn load_legend(name: &str) -> crate::dsl::PedalDef {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/{}.pedal",
        env!("CARGO_MANIFEST_DIR"),
        name
    );
    let source = std::fs::read_to_string(&path)
        .unwrap_or_else(|_| panic!("Can't read {path}"));
    crate::dsl::parse_pedal_file(&source)
        .unwrap_or_else(|e| panic!("{name}: parse error: {e}"))
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 1: What stage types does each group produce?
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_no_iir_with_zero_b() {
    // The screamer should NOT have any IIR stage with b=[0,0,0].
    // If it does, that stage kills signal.
    let pedal = load_legend("screamer");
    let compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    for (i, stage) in compiled.iir_stages.iter().enumerate() {
        let dc = stage.iir.dc_gain();
        let all_zero = stage.iir.b_coeffs.iter().all(|&b| b.abs() < 1e-30);
        eprintln!("  iir[{i}]: dc_gain={dc:.4}, b={:?}, a={:?}",
            stage.iir.b_coeffs, stage.iir.a_coeffs);
        assert!(
            !all_zero,
            "IIR stage {i} has b=[0,0,0] — this kills signal. dc_gain={dc:.6}"
        );
    }
}

#[test]
fn sd1_no_iir_with_zero_b() {
    let pedal = load_legend("sd1");
    let compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    for (i, stage) in compiled.iir_stages.iter().enumerate() {
        let all_zero = stage.iir.b_coeffs.iter().all(|&b| b.abs() < 1e-30);
        eprintln!("  iir[{i}]: b={:?}", stage.iir.b_coeffs);
        assert!(!all_zero, "SD1 IIR stage {i} has b=[0,0,0]");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 2: Passive sub-groups — SPQR should produce PassiveWdf, not Rigid
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_passive_groups_are_not_rigid() {
    // After signal flow grouping, passive sub-groups should go through
    // SPQR and produce PassiveWdf stages (WDF tree), not Rigid stages
    // that go to IIR with b=[0,0,0].
    let pedal = load_legend("screamer");
    let graph = CircuitGraph::from_pedal(&pedal);
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    let groups = super::signal_flow::find_flow_groups(&all_edges, &graph);

    for (gi, group) in groups.iter().enumerate() {
        // Only check passive-only groups (no active edges)
        if !group.active_edges.is_empty() {
            continue;
        }
        let edges = group.all_edges();
        if edges.is_empty() {
            continue;
        }

        let edge_names: Vec<String> = edges.iter().map(|&eidx| {
            graph.components[graph.edges[eidx].comp_idx].id.clone()
        }).collect();

        // Compute group terminals (same as compile_via_spqr does)
        let global_terminals = vec![graph.in_node, graph.out_node];
        let terminals = super::spqr_build::compute_group_terminals(
            &edges, &graph, &global_terminals,
        );

        let tree = spqr_decompose(&edges, &terminals, &graph, graph.gnd_node);
        let stages = spqr_to_stages(&tree, &graph, 48000.0);

        let has_rigid = stages.iter().any(|s| matches!(s, SpqrStage::Rigid { .. }));

        eprintln!("group {gi}: {:?} → {} stages, rigid={}",
            edge_names,
            stages.len(),
            has_rigid,
        );
        for (si, s) in stages.iter().enumerate() {
            eprintln!("  [{si}]: {s:?}");
        }

        if has_rigid {
            // Show why it's rigid — what terminals did we compute?
            eprintln!("  terminals: {:?}", terminals);
            eprintln!("  FAIL: passive group should be SP-reducible, got Rigid");
        }

        assert!(
            !has_rigid,
            "Passive group {gi} ({:?}) should not be Rigid",
            edge_names,
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 3: Minimal reproduction — simplest circuit that fails
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn minimal_bias_network_not_rigid() {
    // The screamer's input network has bias resistors (R_v1, R_v2) from
    // signal node to VCC/GND. These create a T-junction that should
    // SP-reduce with pendant extraction.
    //
    // Simplified: Cin → R_in → junction → (R_bias to VCC, R_out to out)
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                Cin: cap(47n)
                R_in: resistor(10k)
                R_bias: resistor(510k)
                R_out: resistor(10k)
            }
            nets {
                in -> Cin.a
                Cin.b -> R_in.a
                R_in.b -> R_bias.a
                R_bias.b -> vcc
                R_in.b -> R_out.a
                R_out.b -> out
            }
            controls {}
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    eprintln!("Bias network: wdf={}, iir={}, ss={}, order={:?}",
        compiled.stages.len(), compiled.iir_stages.len(),
        compiled.state_space_stages.len(), compiled.stage_order);

    // Check no IIR with zero b
    for (i, stage) in compiled.iir_stages.iter().enumerate() {
        let all_zero = stage.iir.b_coeffs.iter().all(|&b| b.abs() < 1e-30);
        eprintln!("  iir[{i}]: b={:?}", stage.iir.b_coeffs);
        assert!(!all_zero, "IIR stage {i} has b=[0,0,0]");
    }

    // Should produce output
    for _ in 0..2000 { compiled.process(0.0); }
    let mut peak = 0.0f64;
    for s in 0..4800 {
        let input = 0.05 * (2.0 * std::f64::consts::PI * 440.0 * s as f64 / 48000.0).sin();
        peak = peak.max(compiled.process(input).abs());
    }
    eprintln!("Bias network: peak={peak:.6}");
    assert!(peak > 0.001, "Bias network should pass signal: {peak:.6}");
}

#[test]
fn minimal_coupling_cap_passes_signal() {
    // A single coupling cap between two stages should pass AC signal.
    // If it becomes a standalone WDF stage with wrong root, it blocks.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                C1: cap(1u)
                R2: resistor(10k)
            }
            nets {
                in -> R1.a
                R1.b -> C1.a
                C1.b -> R2.a
                R2.b -> out
            }
            controls {}
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    eprintln!("Coupling cap: wdf={}, iir={}, order={:?}",
        compiled.stages.len(), compiled.iir_stages.len(), compiled.stage_order);

    for _ in 0..2000 { compiled.process(0.0); }
    let mut peak = 0.0f64;
    for s in 0..4800 {
        let input = 0.05 * (2.0 * std::f64::consts::PI * 440.0 * s as f64 / 48000.0).sin();
        peak = peak.max(compiled.process(input).abs());
    }
    eprintln!("Coupling cap: peak={peak:.6}");
    assert!(peak > 0.001, "R-C-R chain should pass signal: {peak:.6}");
}

#[test]
fn minimal_opamp_with_input_network_produces_audio() {
    // The simplest circuit that reproduces the regression:
    // Cin → R_in → opamp → out
    // Cin + R_in form a passive group that must pass signal.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                Cin: cap(47n)
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
            }
            nets {
                in -> Cin.a
                Cin.b -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    eprintln!("Cin+R+opamp: wdf={}, iir={}, ss={}, order={:?}",
        compiled.stages.len(), compiled.iir_stages.len(),
        compiled.state_space_stages.len(), compiled.stage_order);

    for _ in 0..2000 { compiled.process(0.0); }
    let mut peak = 0.0f64;
    for s in 0..4800 {
        let input = 0.05 * (2.0 * std::f64::consts::PI * 440.0 * s as f64 / 48000.0).sin();
        peak = peak.max(compiled.process(input).abs());
    }
    eprintln!("Cin+R+opamp: peak={peak:.6}");
    assert!(peak > 0.01, "Cin→R→opamp should produce output: {peak:.6}");
}

#[test]
fn screamer_clipping_stage_has_gain() {
    // The screamer's clipping stage (General: vcvs=1, nl=1) should AMPLIFY,
    // not attenuate by 5000x. The build_opamp_nl_feedback function creates
    // an OpAmpRoot from Rf/Ri — if Ri is wrong, gain is wrong.
    let pedal = load_legend("screamer");
    let graph = CircuitGraph::from_pedal(&pedal);
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    let groups = super::signal_flow::find_flow_groups(&all_edges, &graph);

    // Find the clipping group (has VCVS + NL)
    let clip_group = groups.iter().find(|g| {
        g.active_edges.iter().any(|&eidx| {
            graph.effective_edge_kind(eidx) == super::component::EdgeKind::Nonlinear
        }) && g.active_edges.iter().any(|&eidx| {
            graph.effective_edge_kind(eidx) == super::component::EdgeKind::Vcvs
        })
    });

    if let Some(group) = clip_group {
        let active_names: Vec<String> = group.active_edges.iter().map(|&eidx| {
            format!("{}({:?})", graph.components[graph.edges[eidx].comp_idx].id,
                graph.effective_edge_kind(eidx))
        }).collect();
        let feedback_names: Vec<String> = group.feedback_edges.iter().map(|&eidx| {
            let comp = &graph.components[graph.edges[eidx].comp_idx];
            format!("{}(R={:?})", comp.id, comp.kind.resistance())
        }).collect();
        let pendant_names: Vec<String> = group.pendant_edges.iter().map(|&eidx| {
            let comp = &graph.components[graph.edges[eidx].comp_idx];
            format!("{}(R={:?})", comp.id, comp.kind.resistance())
        }).collect();
        let ground_names: Vec<String> = group.ground_shunt_edges.iter().map(|&eidx| {
            graph.components[graph.edges[eidx].comp_idx].id.clone()
        }).collect();

        eprintln!("Screamer clipping group:");
        eprintln!("  active: {:?}", active_names);
        eprintln!("  feedback: {:?}", feedback_names);
        eprintln!("  pendant: {:?}", pendant_names);
        eprintln!("  ground_shunt: {:?}", ground_names);

        // Pendant edges are Ri (input coupling). Should have resistance.
        let ri: f64 = group.pendant_edges.iter()
            .filter_map(|&eidx| graph.components[graph.edges[eidx].comp_idx].kind.resistance())
            .sum();
        let rf: f64 = group.feedback_edges.iter()
            .filter_map(|&eidx| graph.components[graph.edges[eidx].comp_idx].kind.resistance())
            .sum();

        eprintln!("  Ri={ri:.0}, Rf={rf:.0}, gain={:.1}", rf / ri.max(1.0));

        assert!(ri > 0.0, "Screamer clipping stage needs Ri > 0 for gain calculation");
        assert!(rf > 0.0, "Screamer clipping stage needs Rf > 0 for gain calculation");
        let gain = rf / ri;
        assert!(gain > 1.0, "Screamer clipping stage should have gain > 1, got {gain:.2}");
    } else {
        panic!("No clipping group found in screamer");
    }
}

#[test]
fn pot_with_output_passives_produces_audio() {
    // Volume pot + output coupling cap + load resistors.
    // This is the screamer's Level group pattern.
    // The pot creates 2 edges (aw, wb) which may prevent SP reduction.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                Level: pot(100k, a)
                C_out: cap(10u)
                R_out: resistor(10k)
            }
            nets {
                in -> R_in.a
                R_in.b -> Level.a
                Level.w -> C_out.a
                C_out.b -> R_out.a
                R_out.b -> out
                Level.b -> gnd
            }
            controls { Level.position -> "Level" [1.0, 0.0] = 0.5 }
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    eprintln!("Pot+output: wdf={}, iir={}, ss={}, order={:?}",
        compiled.stages.len(), compiled.iir_stages.len(),
        compiled.state_space_stages.len(), compiled.stage_order);

    // Check no IIR with zero b
    for (i, stage) in compiled.iir_stages.iter().enumerate() {
        let all_zero = stage.iir.b_coeffs.iter().all(|&b| b.abs() < 1e-30);
        eprintln!("  iir[{i}]: b={:?}", stage.iir.b_coeffs);
        assert!(!all_zero, "IIR stage {i} has b=[0,0,0]");
    }

    for _ in 0..2000 { compiled.process(0.0); }
    let mut peak = 0.0f64;
    for s in 0..4800 {
        let input = 0.05 * (2.0 * std::f64::consts::PI * 440.0 * s as f64 / 48000.0).sin();
        peak = peak.max(compiled.process(input).abs());
    }
    eprintln!("Pot+output: peak={peak:.6}");
    assert!(peak > 0.001, "Pot+output chain should pass signal: {peak:.6}");
}

#[test]
fn opamp_into_tone_stack_into_volume_produces_audio() {
    // The full screamer-like cascade: opamp → tone → volume → out.
    // Each passive sub-group must pass signal for the cascade to work.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                R_tone: resistor(10k)
                C_tone: cap(22n)
                Level: pot(100k, a)
                C_out: cap(10u)
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
                R_tone.b -> Level.a
                Level.w -> C_out.a
                C_out.b -> out
                Level.b -> gnd
            }
            controls { Level.position -> "Level" [1.0, 0.0] = 0.5 }
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    // Debug: show all edges in each group
    {
        let graph = CircuitGraph::from_pedal(&pedal);
        let active_set: std::collections::HashSet<usize> =
            graph.active_edge_indices.iter().copied().collect();
        let all_edges: Vec<usize> = (0..graph.edges.len())
            .filter(|i| !active_set.contains(i))
            .collect();
        let groups = super::signal_flow::find_flow_groups(&all_edges, &graph);
        for (gi, g) in groups.iter().enumerate() {
            if !g.active_edges.is_empty() { continue; }
            let edges = g.all_edges();
            for &eidx in &edges {
                let e = &graph.edges[eidx];
                let comp = &graph.components[e.comp_idx];
                eprintln!("  g{gi} edge: {} nodes {}→{} (gnd={})",
                    comp.id, e.node_a, e.node_b, graph.gnd_node);
            }
        }
    }

    // Remove the detailed edge debug output for cleanliness
    eprintln!("Opamp+tone+vol: wdf={}, iir={}, ss={}, order={:?}",
        compiled.stages.len(), compiled.iir_stages.len(),
        compiled.state_space_stages.len(), compiled.stage_order);

    for _ in 0..2000 { compiled.process(0.0); }
    let mut peak = 0.0f64;
    for s in 0..4800 {
        let input = 0.05 * (2.0 * std::f64::consts::PI * 440.0 * s as f64 / 48000.0).sin();
        peak = peak.max(compiled.process(input).abs());
    }
    eprintln!("Opamp+tone+vol: peak={peak:.6}");
    assert!(peak > 0.01, "Full cascade should produce output: {peak:.6}");
}
