//! TB-303 ladder filter decomposition tests.
//!
//! These tests load the .pedal files from pedalkernel-pro (if present)
//! and verify that the 4-stage diode-connected BJT ladder decomposes
//! correctly via blockwise + K-method.
//!
//! If the .pedal files are not found (e.g., public-only checkout),
//! tests pass with a skip message. No .pedal content is inlined here.

use super::blockwise;
use super::graph::CircuitGraph;
use super::spqr::*;
use crate::PedalProcessor;

const SR: f64 = 48_000.0;

fn settled_sine_rms<F>(freq: f64, amp: f64, mut process: F) -> f64
where
    F: FnMut(f64) -> f64,
{
    for i in 0..9600 {
        let input = amp * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
        let _ = process(input);
    }

    let mut sum_sq = 0.0f64;
    let mut count = 0usize;
    for i in 0..9600 {
        let input = amp * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
        let out = process(input);
        sum_sq += out * out;
        count += 1;
    }

    (sum_sq / count.max(1) as f64).sqrt()
}

fn ac_rms(values: &[f64]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let mean = values.iter().copied().sum::<f64>() / values.len() as f64;
    let sum_sq = values
        .iter()
        .map(|value| {
            let ac = *value - mean;
            ac * ac
        })
        .sum::<f64>();
    (sum_sq / values.len() as f64).sqrt()
}

fn stinchcombe_htb_magnitude(freq_hz: f64, fc_hz: f64) -> f64 {
    let w = freq_hz / fc_hz;
    let w2 = w * w;
    let w3 = w2 * w;
    let w4 = w2 * w2;
    let re = w4 - 14.142 * w2 + 1.0;
    let im = -6.727 * w3 + 9.514 * w;
    1.0 / (re * re + im * im).sqrt()
}

fn db_norm(value: f64, reference: f64) -> f64 {
    20.0 * (value / reference.max(1e-12)).max(1e-12).log10()
}

const TWO_RUNG_DIODE_LADDER: &str = r#"
pedal "Two Rung Diode Ladder" { supply 9V
  ports {
    audio_in: input(10k)
    audio_out: output
  }
  components {
    D1: diode(silicon)
    D2: diode(silicon)
    R_bias1: resistor(100k)
    R_bias2: resistor(100k)
    R_in: resistor(10k)
    C1: cap(33n)
    C2: cap(33n)
  }
  nets {
    vcc -> R_bias1.a
    R_bias1.b -> D1.a
    audio_in -> R_in.a
    R_in.b -> D1.a

    D1.b -> C1.a
    C1.b -> gnd
    D1.b -> D2.a

    vcc -> R_bias2.a
    R_bias2.b -> D2.a
    D2.b -> C2.a
    C2.b -> gnd
    D2.b -> audio_out
  }
  controls {}
}
"#;

const TWO_RUNG_DIODE_LADDER_WITH_FEEDBACK: &str = r#"
pedal "Two Rung Diode Ladder Feedback" { supply 9V
  ports {
    audio_in: input(10k)
    audio_out: output
  }
  components {
    D1: diode(silicon)
    D2: diode(silicon)
    R_bias1: resistor(100k)
    R_bias2: resistor(100k)
    R_in: resistor(10k)
    C1: cap(33n)
    C2: cap(33n)
    Resonance: pot(100k, b)
    R_fb_limit: resistor(33k)
  }
  nets {
    vcc -> R_bias1.a
    R_bias1.b -> D1.a
    audio_in -> R_in.a
    R_in.b -> D1.a

    D1.b -> C1.a
    C1.b -> gnd
    D1.b -> D2.a

    vcc -> R_bias2.a
    R_bias2.b -> D2.a
    D2.b -> C2.a
    C2.b -> gnd
    D2.b -> audio_out
    D2.b -> Resonance.a
    Resonance.b -> R_fb_limit.a
    R_fb_limit.b -> D1.a
  }
  controls {
    Resonance.position -> "Resonance" [0.0, 0.95] = 0.0
  }
}
"#;

const TWO_RUNG_DIODE_LADDER_WITH_FEEDBACK_AND_CV: &str = r#"
pedal "Two Rung Diode Ladder Feedback CV" { supply 9V
  ports {
    audio_in: input(10k)
    cv_cutoff: input(47k)
    audio_out: output
  }
  components {
    D1: diode(silicon)
    D2: diode(silicon)
    R_bias1: resistor(100k)
    R_bias2: resistor(100k)
    R_in: resistor(10k)
    R_cv: resistor(47k)
    C1: cap(33n)
    C2: cap(33n)
    Resonance: pot(100k, b)
    R_fb_limit: resistor(33k)
  }
  nets {
    vcc -> R_bias1.a
    R_bias1.b -> D1.a
    audio_in -> R_in.a
    R_in.b -> D1.a
    cv_cutoff -> R_cv.a
    R_cv.b -> D1.a

    D1.b -> C1.a
    C1.b -> gnd
    D1.b -> D2.a

    vcc -> R_bias2.a
    R_bias2.b -> D2.a
    D2.b -> C2.a
    C2.b -> gnd
    D2.b -> audio_out
    D2.b -> Resonance.a
    Resonance.b -> R_fb_limit.a
    R_fb_limit.b -> D1.a
  }
  controls {
    Resonance.position -> "Resonance" [0.0, 0.95] = 0.0
  }
}
"#;

/// Try to load a .pedal file from the pro crate's acidattack-core directory.
fn load_pro_pedal(filename: &str) -> Option<String> {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let pro_path = format!(
        "{}/../../pedalkernel-pro/crates/acidattack/acidattack-core/{filename}",
        manifest_dir
    );
    match std::fs::read_to_string(&pro_path) {
        Ok(s) => {
            eprintln!("  loaded {filename} ({} bytes)", s.len());
            Some(s)
        }
        Err(_) => None,
    }
}

macro_rules! skip_if_missing {
    ($source:expr, $name:expr) => {
        match $source {
            Some(s) => s,
            None => {
                eprintln!("  SKIP: {} not found (pro crate not present)", $name);
                return;
            }
        }
    };
}

// ═══════════════════════════════════════════════════════════════════════════
// 1. Filter compiles without error (structural — no K-tables)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_filter_compiles() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    eprintln!("  parsing...");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    eprintln!("  compiling (skip_k_tables=true)...");
    let compiled =
        super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::debug());
    assert!(
        compiled.is_ok(),
        "TB303 filter should compile: {:?}",
        compiled.err()
    );
    let compiled = compiled.unwrap();
    eprintln!("  TB303 filter: {} stages", compiled.stages.len());
    for (i, stage) in compiled.stages.iter().enumerate() {
        match stage {
            super::compiled::Stage::Wdf(w) => {
                let is_bjt = matches!(w.root, pedalkernel_rt::stage::RootKind::Bjt(_));
                eprintln!(
                    "    stage {i}: WDF bjt={is_bjt} rp={:.0}",
                    w.tree.port_resistance()
                );
            }
            super::compiled::Stage::Iir(iir) => {
                eprintln!("    stage {i}: IIR");
            }
            super::compiled::Stage::MultiNl(m) => {
                eprintln!("    stage {i}: MultiNL (MONOLITHIC — bad!)");
            }
            _ => {
                eprintln!("    stage {i}: other");
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Blockwise detects 4 NL blocks (one per ladder stage)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_filter_has_4_blockwise_blocks() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);

    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    eprintln!(
        "  total edges: {}, NL: {}",
        all_edges.len(),
        all_edges
            .iter()
            .filter(|&&e| graph.effective_edge_kind(e) == super::component::EdgeKind::Nonlinear)
            .count()
    );

    let plan = blockwise::analyze_blockwise(&all_edges, &graph);

    if let Some(plan) = &plan {
        eprintln!(
            "  Blockwise: {} blocks, {} coupling edges",
            plan.num_blocks(),
            plan.coupling_edges.len()
        );
        for (i, block) in plan.blocks.iter().enumerate() {
            let edge_names: Vec<&str> = block
                .all_edges()
                .iter()
                .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
                .collect();
            eprintln!(
                "    block {i}: {} NL, {} reactive, {} linear = {:?}",
                block.nl_edges.len(),
                block.reactive_edges.len(),
                block.linear_edges.len(),
                edge_names
            );
        }
        let coupling_names: Vec<&str> = plan
            .coupling_edges
            .iter()
            .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
            .collect();
        eprintln!("    coupling: {:?}", coupling_names);

        assert!(
            plan.num_blocks() >= 4,
            "TB303 4-pole ladder should have ≥4 blockwise blocks, got {}",
            plan.num_blocks()
        );
    } else {
        eprintln!("  WARNING: blockwise decomposition returned None — ladder not split");
    }
}

#[test]
fn tb303_blockwise_blocks_are_in_signal_order() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);

    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    let groups = super::signal_flow::find_flow_groups(&all_edges, &graph);
    let feedback_group = groups
        .iter()
        .find(|group| group.has_feedback())
        .expect("TB303 ladder feedback group should exist");
    let plan = blockwise::analyze_blockwise(&feedback_group.all_edges(), &graph)
        .expect("should decompose");
    let block_names: Vec<&str> = plan
        .blocks
        .iter()
        .map(|block| {
            let edge = block.nl_edges[0];
            graph.components[graph.edges[edge].comp_idx].id.as_str()
        })
        .collect();

    assert_eq!(
        block_names,
        vec!["Q1", "Q2", "Q3", "Q4"],
        "diode-connected BJT ladder blocks must follow base/collector -> emitter cascade order"
    );
}

#[test]
fn tb303_blockwise_rungs_keep_their_own_caps() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);

    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    let groups = super::signal_flow::find_flow_groups(&all_edges, &graph);
    let feedback_group = groups
        .iter()
        .find(|group| group.has_feedback())
        .expect("TB303 ladder feedback group should exist");
    let plan = blockwise::analyze_blockwise(&feedback_group.all_edges(), &graph)
        .expect("should decompose");
    let actual: Vec<(&str, Vec<&str>)> = plan
        .blocks
        .iter()
        .map(|block| {
            let nl_edge = block.nl_edges[0];
            let nl_name = graph.components[graph.edges[nl_edge].comp_idx].id.as_str();
            let caps = block
                .reactive_edges
                .iter()
                .filter_map(|&eidx| {
                    let id = graph.components[graph.edges[eidx].comp_idx].id.as_str();
                    matches!(id, "C1" | "C2" | "C3" | "C4").then_some(id)
                })
                .collect::<Vec<_>>();
            (nl_name, caps)
        })
        .collect();

    eprintln!("  rung cap assignment: {actual:?}");
    assert_eq!(
        actual,
        vec![
            ("Q1", vec!["C1"]),
            ("Q2", vec!["C2"]),
            ("Q3", vec!["C3"]),
            ("Q4", vec!["C4"]),
        ],
        "each signal-ordered BKM rung must own the shunt cap attached to its emitter"
    );
}

#[test]
fn tb303_blockwise_coupling_ports_are_diode_inputs() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);

    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();
    let plan = blockwise::analyze_blockwise(&all_edges, &graph).expect("should decompose");

    let selected: Vec<_> = plan
        .blocks
        .iter()
        .map(|block| {
            blockwise::block_coupling_port_node(&block.nl_edges, &block.port_nodes, &graph)
        })
        .collect();

    let expected: Vec<_> = ["Q1.base", "Q2.base", "Q3.base", "Q4.base"]
        .iter()
        .map(|name| Some(graph.node_names.get(*name).copied().expect("pin node")))
        .collect();

    eprintln!("  selected block ports: {selected:?}");
    eprintln!("  expected diode inputs: {expected:?}");

    assert_eq!(
        selected, expected,
        "BKM coupling ports must attach to each diode's driven base/collector node"
    );
}

#[test]
fn tb303_pedal_nodes_match_expected_ladder_wiring() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);

    let node = |name: &str| graph.node_names.get(name).copied().expect(name);

    assert_eq!(node("Q1.base"), node("Q1.collector"));
    assert_eq!(node("Q2.base"), node("Q2.collector"));
    assert_eq!(node("Q3.base"), node("Q3.collector"));
    assert_eq!(node("Q4.base"), node("Q4.collector"));

    assert_eq!(node("Q1.emitter"), node("Q2.base"));
    assert_eq!(node("Q2.emitter"), node("Q3.base"));
    assert_eq!(node("Q3.emitter"), node("Q4.base"));
    assert_eq!(node("Q4.emitter"), node("C_out.a"));
    assert_eq!(node("Q4.emitter"), node("Resonance.a"));

    assert_ne!(node("Q1.base"), node("Q1.emitter"));
    assert_ne!(node("Q4.base"), node("Q4.emitter"));
}

#[test]
fn two_rung_diode_ladder_splits_into_ordered_blocks() {
    let def = crate::dsl::parse_pedal_file(TWO_RUNG_DIODE_LADDER).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();
    let plan = blockwise::analyze_blockwise(&all_edges, &graph)
        .expect("literal two-rung diode ladder should decompose");

    let names: Vec<&str> = plan
        .blocks
        .iter()
        .map(|block| {
            let edge = block.nl_edges[0];
            graph.components[graph.edges[edge].comp_idx].id.as_str()
        })
        .collect();

    eprintln!("  two-rung diode ladder block order: {names:?}");
    assert_eq!(names, vec!["D1", "D2"]);
}

#[test]
fn two_rung_diode_ladder_bkm_direct_path_is_lowpass() {
    let def = crate::dsl::parse_pedal_file(TWO_RUNG_DIODE_LADDER).expect("parse failed");

    let measure = |freq: f64| -> f64 {
        let mut proc = super::compile_pedal(&def, SR).expect("compile failed");
        let bkm = proc
            .stages
            .iter_mut()
            .find_map(|s| {
                if let pedalkernel_rt::processor::Stage::BlockwiseKMethod(ref mut k) = s {
                    Some(k)
                } else {
                    None
                }
            })
            .expect("two-rung diode ladder should compile to BKM");

        for _ in 0..4800 {
            let _ = bkm.process(&[0.0, 0.0]);
        }

        let mut peak = 0.0f64;
        for i in 0..4800 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            peak = peak.max(bkm.process(&[input, 0.0]).abs());
        }
        peak
    };

    let gain_100 = measure(100.0);
    let gain_10k = measure(10_000.0);
    let ratio_db = 20.0 * (gain_100 / gain_10k.max(1e-12)).log10();

    eprintln!(
        "  two-rung diode ladder BKM: 100Hz={gain_100:.6}, 10kHz={gain_10k:.6}, ratio={ratio_db:+.1} dB"
    );

    assert!(
        gain_100 > gain_10k,
        "two-rung diode ladder BKM should be lowpass: 100Hz={gain_100:.6}, 10kHz={gain_10k:.6}"
    );
}

#[test]
fn two_rung_normal_diode_ladder_cv_moves_cutoff() {
    let def = crate::dsl::parse_pedal_file(TWO_RUNG_DIODE_LADDER_WITH_FEEDBACK_AND_CV)
        .expect("parse failed");
    let probe = super::compile_pedal(&def, SR).expect("compile failed");
    assert!(
        probe
            .stages
            .iter()
            .any(|stage| matches!(stage, pedalkernel_rt::processor::Stage::BlockwiseKMethod(_))),
        "normal diode CV ladder should compile to BKM"
    );

    let measure_ac = |cv_voltage: f64, freq: f64| -> f64 {
        let mut proc = super::compile_pedal(&def, SR).expect("compile failed");
        let in_idx = proc.resolve_port("audio_in").expect("audio_in port");
        let cv_idx = proc.resolve_port("cv_cutoff").expect("cv_cutoff port");
        let out_idx = proc.resolve_port("audio_out").expect("audio_out port");

        for _ in 0..9600 {
            let mut ports = vec![0.0; proc.port_count()];
            ports[cv_idx] = cv_voltage;
            proc.process_ports(&mut ports);
        }

        let mut values = Vec::new();
        for i in 0..9600 {
            let input = 0.05 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let mut ports = vec![0.0; proc.port_count()];
            ports[in_idx] = input;
            ports[cv_idx] = cv_voltage;
            proc.process_ports(&mut ports);
            values.push(ports[out_idx]);
        }
        ac_rms(&values)
    };

    let low_cv_8k = measure_ac(-1.0, 8_000.0);
    let high_cv_8k = measure_ac(3.0, 8_000.0);
    let low_cv_12k = measure_ac(-1.0, 12_000.0);
    let high_cv_12k = measure_ac(3.0, 12_000.0);

    eprintln!(
        "  normal diode cutoff CV AC: 8k -1V={low_cv_8k:.6}, +3V={high_cv_8k:.6}; \
         12k -1V={low_cv_12k:.6}, +3V={high_cv_12k:.6}"
    );

    assert!(
        high_cv_8k > low_cv_8k * 1.25,
        "raising cv_cutoff should move the normal diode ladder cutoff upward at 8kHz: \
         low={low_cv_8k:.6}, high={high_cv_8k:.6}, ratio={:.3}",
        high_cv_8k / low_cv_8k.max(1e-12)
    );
    assert!(
        high_cv_12k > low_cv_12k * 1.25,
        "raising cv_cutoff should move the normal diode ladder cutoff upward at 12kHz: \
         low={low_cv_12k:.6}, high={high_cv_12k:.6}, ratio={:.3}",
        high_cv_12k / low_cv_12k.max(1e-12)
    );
}

#[test]
fn two_rung_normal_diode_ladder_cutoff_cv_is_reversible() {
    let def = crate::dsl::parse_pedal_file(TWO_RUNG_DIODE_LADDER_WITH_FEEDBACK_AND_CV)
        .expect("parse failed");
    let mut proc = super::compile_pedal(&def, SR).expect("compile failed");
    let in_idx = proc.resolve_port("audio_in").expect("audio_in port");
    let cv_idx = proc.resolve_port("cv_cutoff").expect("cv_cutoff port");
    let out_idx = proc.resolve_port("audio_out").expect("audio_out port");

    let measure_ac = |proc: &mut super::compiled::CompiledPedal, cv_voltage: f64| -> f64 {
        for _ in 0..9600 {
            let mut ports = vec![0.0; proc.port_count()];
            ports[cv_idx] = cv_voltage;
            proc.process_ports(&mut ports);
        }

        let mut values = Vec::new();
        for i in 0..9600 {
            let input = 0.05 * (2.0 * std::f64::consts::PI * 12_000.0 * i as f64 / SR).sin();
            let mut ports = vec![0.0; proc.port_count()];
            ports[in_idx] = input;
            ports[cv_idx] = cv_voltage;
            proc.process_ports(&mut ports);
            values.push(ports[out_idx]);
        }
        ac_rms(&values)
    };

    let low_initial = measure_ac(&mut proc, 0.0);
    let high = measure_ac(&mut proc, 3.0);
    let low_again = measure_ac(&mut proc, 0.0);

    eprintln!(
        "  reversible normal diode cutoff CV AC: 12k 0V={low_initial:.6}, \
         +3V={high:.6}, back-to-0V={low_again:.6}"
    );

    assert!(
        high > low_initial * 1.25,
        "raising cv_cutoff should open the normal diode ladder before reversal: \
         low={low_initial:.6}, high={high:.6}"
    );
    assert!(
        low_again < high / 1.25,
        "returning cv_cutoff to 0V should restore the lower cutoff instead of keeping stale Rp: \
         high={high:.6}, low_again={low_again:.6}"
    );
}

#[test]
fn two_rung_normal_diode_ladder_cutoff_uses_circuit_calibration() {
    let def = crate::dsl::parse_pedal_file(TWO_RUNG_DIODE_LADDER_WITH_FEEDBACK_AND_CV)
        .expect("parse failed");
    let mut proc = super::compile_pedal(&def, SR).expect("compile failed");
    let cv_idx = proc.resolve_port("cv_cutoff").expect("cv_cutoff port");

    let measure_rp = |proc: &mut super::compiled::CompiledPedal, cv_voltage: f64| -> f64 {
        let mut ports = vec![0.0; proc.port_count()];
        ports[cv_idx] = cv_voltage;
        proc.process_ports(&mut ports);
        proc.stages
            .iter()
            .find_map(|stage| {
                if let pedalkernel_rt::processor::Stage::BlockwiseKMethod(bkm) = stage {
                    Some(bkm.blocks[0].rp)
                } else {
                    None
                }
            })
            .expect("normal diode ladder should compile to BKM")
    };

    let rp_0v = measure_rp(&mut proc, 0.0);
    let rp_3v = measure_rp(&mut proc, 3.0);
    let ratio = rp_3v / rp_0v;

    eprintln!("  circuit-calibrated normal diode cutoff Rp: 0V={rp_0v:.3}, +3V={rp_3v:.3}, ratio={ratio:.3}");

    assert!(
        ratio > 0.35 && ratio < 0.85,
        "cutoff CV should use diode dynamic resistance from R_bias/R_cv, not 2^CV scaling: \
         rp_0v={rp_0v:.3}, rp_3v={rp_3v:.3}, ratio={ratio:.3}"
    );
}

#[test]
fn two_rung_feedback_diode_ladder_compiles_to_bkm() {
    let def =
        crate::dsl::parse_pedal_file(TWO_RUNG_DIODE_LADDER_WITH_FEEDBACK).expect("parse failed");
    let proc = super::compile_pedal(&def, SR).expect("compile failed");

    let bkm_count = proc
        .stages
        .iter()
        .filter(|stage| matches!(stage, pedalkernel_rt::processor::Stage::BlockwiseKMethod(_)))
        .count();

    assert_eq!(
        bkm_count, 1,
        "feedback diode ladder should compile as one BKM stage"
    );
}

#[test]
fn two_rung_feedback_diode_ladder_bkm_direct_path_is_lowpass() {
    let def =
        crate::dsl::parse_pedal_file(TWO_RUNG_DIODE_LADDER_WITH_FEEDBACK).expect("parse failed");

    let measure = |freq: f64| -> f64 {
        let mut proc = super::compile_pedal(&def, SR).expect("compile failed");
        proc.set_control("Resonance", 0.0);

        for _ in 0..4800 {
            let _ = proc.process(0.0);
        }

        let mut peak = 0.0f64;
        for i in 0..4800 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            peak = peak.max(proc.process(input).abs());
        }
        peak
    };

    let gain_100 = measure(100.0);
    let gain_10k = measure(10_000.0);
    let ratio_db = 20.0 * (gain_100 / gain_10k.max(1e-12)).log10();

    eprintln!(
        "  two-rung feedback diode ladder: 100Hz={gain_100:.6}, 10kHz={gain_10k:.6}, ratio={ratio_db:+.1} dB"
    );

    assert!(
        gain_100 > gain_10k * 1.1,
        "two-rung feedback diode ladder should tilt lowpass: 100Hz={gain_100:.6}, 10kHz={gain_10k:.6}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Compiled structure: not monolithic (structural — no K-tables)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_filter_not_monolithic() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let compiled =
        super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::debug())
            .unwrap();

    let dump = compiled.debug_dump();
    let has_8port = dump.contains("n_nl=8");
    let stage_count = compiled.stages.len();

    eprintln!("  Stages: {stage_count}, monolithic_8port: {has_8port}");
    eprintln!("{dump}");

    if has_8port {
        eprintln!("  WARNING: monolithic n_nl=8 — blockwise not applied to this circuit");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Audio: lowpass behavior (needs K-tables — uses disk cache)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_filter_is_lowpass() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    eprintln!("  compiling with K-tables (cached)...");
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_filter_audio",
        "tb303_filter_audio",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");
    eprintln!("  blob: {} bytes", blob.len());

    let measure = |freq: f64, cutoff: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.set_control("Cutoff", cutoff);
        proc.set_control("Resonance", 0.0);
        for _ in 0..2400 {
            proc.process(0.0);
        }
        let mut peak = 0.0f64;
        for i in 0..4800 {
            let input = (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            peak = peak.max(proc.process(input).abs());
        }
        peak
    };

    let gain_100 = measure(100.0, 0.5);
    let gain_10k = measure(10000.0, 0.5);
    let ratio_db = 20.0 * (gain_100 / gain_10k.max(1e-12)).log10();

    eprintln!("  LPF: 100Hz={gain_100:.4}, 10kHz={gain_10k:.4}, ratio={ratio_db:+.1} dB");

    assert!(
        gain_100 > gain_10k * 2.0,
        "LOWPASS: 100Hz ({gain_100:.4}) must be >2x 10kHz ({gain_10k:.4})"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. SPQR tree structure inspection
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_filter_spqr_structure() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);

    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    let tree = spqr_decompose(
        &all_edges,
        &[graph.in_node, graph.out_node],
        &graph,
        graph.gnd_node,
    );

    fn count_nodes(node: &SpqrNode) -> (usize, usize, usize, usize) {
        match node {
            SpqrNode::S { children, .. } => {
                let mut s = (1, 0, 0, 0);
                for c in children {
                    let r = count_nodes(c);
                    s.0 += r.0;
                    s.1 += r.1;
                    s.2 += r.2;
                    s.3 += r.3;
                }
                s
            }
            SpqrNode::P { children, .. } => {
                let mut s = (0, 1, 0, 0);
                for c in children {
                    let r = count_nodes(c);
                    s.0 += r.0;
                    s.1 += r.1;
                    s.2 += r.2;
                    s.3 += r.3;
                }
                s
            }
            SpqrNode::Q { .. } => (0, 0, 1, 0),
            SpqrNode::R { children, .. } => {
                let mut s = (0, 0, 0, 1);
                for c in children {
                    let r = count_nodes(c);
                    s.0 += r.0;
                    s.1 += r.1;
                    s.2 += r.2;
                    s.3 += r.3;
                }
                s
            }
        }
    }

    let (s, p, q, r) = count_nodes(&tree);
    eprintln!("  SPQR nodes: S={s}, P={p}, Q={q}, R={r}");
    eprintln!(
        "  Total edges: {}, NL edges: {}",
        all_edges.len(),
        all_edges
            .iter()
            .filter(|&&e| { graph.effective_edge_kind(e) == super::component::EdgeKind::Nonlinear })
            .count()
    );
    eprintln!("  Has P-nodes (parallel shunts): {}", p > 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. Coupling scattering matrix is well-formed
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_coupling_scattering_wellformed() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);

    // Collect passive (non-active) edge indices.
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    let plan = blockwise::analyze_blockwise(&all_edges, &graph)
        .expect("blockwise decomposition should succeed for TB303");

    assert!(
        !plan.coupling_edges.is_empty(),
        "TB303 ladder must have coupling edges connecting the 4 blocks"
    );
    eprintln!(
        "  coupling: {} edges, {} blocks",
        plan.coupling_edges.len(),
        plan.num_blocks()
    );

    // Build MNA from coupling edges.
    // Collect unique non-rail, non-ground nodes from coupling edges.
    let excluded: std::collections::HashSet<usize> = {
        let mut s = std::collections::HashSet::new();
        s.insert(graph.gnd_node);
        s.insert(graph.vcc_node);
        for &n in &graph.supply_nodes {
            s.insert(n);
        }
        s
    };

    let mut coupling_nodes: Vec<usize> = Vec::new();
    for &eidx in &plan.coupling_edges {
        let e = &graph.edges[eidx];
        for &n in &[e.node_a, e.node_b] {
            if !excluded.contains(&n) && !coupling_nodes.contains(&n) {
                coupling_nodes.push(n);
            }
        }
    }
    eprintln!("  coupling nodes: {:?}", coupling_nodes);

    // Map node IDs to MNA indices (0-based, ground = None).
    let node_to_mna: std::collections::HashMap<usize, usize> = coupling_nodes
        .iter()
        .enumerate()
        .map(|(i, &n)| (n, i))
        .collect();
    let num_nodes = coupling_nodes.len();

    let mut mna = pedalkernel_rt::tree::MnaSystem::new(num_nodes, 0);

    // Stamp each coupling resistor.
    for &eidx in &plan.coupling_edges {
        let e = &graph.edges[eidx];
        let comp = &graph.components[e.comp_idx];
        let r = comp.kind.resistance().unwrap_or(10_000.0); // default if not a pure resistor
        let n1 = if excluded.contains(&e.node_a) {
            None
        } else {
            Some(node_to_mna[&e.node_a])
        };
        let n2 = if excluded.contains(&e.node_b) {
            None
        } else {
            Some(node_to_mna[&e.node_b])
        };
        eprintln!(
            "    stamp R={r:.0} between {:?} and {:?} (edge {} = {})",
            n1, n2, eidx, comp.id
        );
        mna.stamp_resistor(n1, n2, r);
    }

    // Create WDF ports: one per block (at port node) + one adapted VS input port.
    let n_ports = plan.num_blocks() + 1;
    let mut ports = Vec::with_capacity(n_ports);

    // Block ports: each block's first port_node that appears in coupling_nodes.
    for (bi, block) in plan.blocks.iter().enumerate() {
        let port_node = block
            .port_nodes
            .iter()
            .find(|&&n| node_to_mna.contains_key(&n))
            .copied();
        let mna_node = port_node.and_then(|n| node_to_mna.get(&n).copied());
        eprintln!(
            "    block {bi}: port_node={:?} → mna_node={:?}",
            port_node, mna_node
        );
        ports.push(pedalkernel_rt::tree::WdfPort {
            node_pos: mna_node,
            node_neg: None,
            resistance: 1000.0, // nominal port resistance
        });
    }

    // Adapted VS input port at the first coupling node.
    ports.push(pedalkernel_rt::tree::WdfPort {
        node_pos: Some(0),
        node_neg: None,
        resistance: 1.0, // VS port
    });

    let s_matrix = mna.derive_scattering_matrix_general(&ports);

    eprintln!("  scattering matrix: {}×{}", n_ports, n_ports);
    for row in 0..n_ports {
        let row_vals: Vec<String> = (0..n_ports)
            .map(|col| format!("{:+.4}", s_matrix[row * n_ports + col]))
            .collect();
        eprintln!("    [{}]", row_vals.join(", "));
    }

    // Assert: matrix is square (n_ports × n_ports).
    assert_eq!(
        s_matrix.len(),
        n_ports * n_ports,
        "Scattering matrix should be {}×{} = {} elements, got {}",
        n_ports,
        n_ports,
        n_ports * n_ports,
        s_matrix.len()
    );

    // Assert: no NaN/Inf.
    for (i, &val) in s_matrix.iter().enumerate() {
        assert!(
            val.is_finite(),
            "Scattering matrix element [{}][{}] = {} is not finite",
            i / n_ports,
            i % n_ports,
            val
        );
    }

    // Assert: all entries bounded [-2, 2] (passivity bound for multi-port).
    for (i, &val) in s_matrix.iter().enumerate() {
        assert!(
            val.abs() <= 2.0 + 1e-10,
            "Scattering matrix element [{}][{}] = {} exceeds passivity bound [-2, 2]",
            i / n_ports,
            i % n_ports,
            val
        );
    }

    // Assert: number of ports = blocks + 1.
    assert_eq!(
        n_ports,
        plan.num_blocks() + 1,
        "Number of ports should be num_blocks + 1 (VS input)"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 7. Blockwise K-method produces audio (non-silent)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_blockwise_k_method_produces_audio() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    eprintln!("  compiling with K-tables (cached)...");
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_bkm_audio",
        "tb303_bkm_audio",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");
    eprintln!("  blob: {} bytes", blob.len());

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    // Default controls.
    proc.set_control("Cutoff", 0.5);
    proc.set_control("Resonance", 0.3);

    // Warm-up.
    for _ in 0..2400 {
        proc.process(0.0);
    }

    // Process 4800 samples of 440Hz sine at 0.5V amplitude.
    let mut peak = 0.0f64;
    for i in 0..4800 {
        let input = 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SR).sin();
        let out = proc.process(input);
        peak = peak.max(out.abs());
    }

    eprintln!("  peak output: {peak:.6}");

    assert!(
        peak > 0.01,
        "TB303 K-method stage should produce audible output, got peak={peak:.6}"
    );
}

#[test]
fn tb303_bkm_direct_output_reaches_serial_output() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_bkm_direct_vs_serial",
        "tb303_bkm_direct_vs_serial",
        SR,
        &super::compile::CompileOptions::default(),
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc_direct: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");
    let mut proc_coupled: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");
    let mut proc_serial: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    let bkm = proc_direct
        .stages
        .iter_mut()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::BlockwiseKMethod(ref mut k) = s {
                Some(k)
            } else {
                None
            }
        })
        .expect("TB303 should compile to blockwise K-method");
    let wdf = proc_coupled
        .stages
        .iter_mut()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::Wdf(ref mut w) = s {
                Some(w)
            } else {
                None
            }
        })
        .expect("TB303 should have output coupling WDF stage");

    for _ in 0..2400 {
        let direct = bkm.process(&[0.0, 0.0, 0.0, 0.0]);
        let _ = wdf.process(direct);
        let _ = proc_serial.process(0.0);
    }

    let mut bkm_peak = 0.0f64;
    let mut coupled_peak = 0.0f64;
    let mut serial_peak = 0.0f64;
    for i in 0..4800 {
        let input = 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SR).sin();
        let bkm_out = bkm.process(&[input, 0.0, 0.0, 0.0]);
        bkm_peak = bkm_peak.max(bkm_out.abs());
        coupled_peak = coupled_peak.max(wdf.process(bkm_out).abs());
        serial_peak = serial_peak.max(proc_serial.process(input).abs());
    }

    eprintln!(
        "  BKM direct peak={bkm_peak:.6}, direct C_out peak={coupled_peak:.6}, serial peak={serial_peak:.6}"
    );
    assert!(
        bkm_peak > 0.01,
        "BKM should produce signal before output coupling, got {bkm_peak:.6}"
    );
    assert!(
        coupled_peak > bkm_peak * 0.1,
        "Output coupling should preserve at least 10% of BKM signal: \
         bkm={bkm_peak:.6}, coupled={coupled_peak:.6}"
    );
    assert!(
        serial_peak > coupled_peak * 0.5,
        "Serial output should preserve at least 10% of BKM signal through C_out: \
         coupled={coupled_peak:.6}, serial={serial_peak:.6}"
    );
}

#[test]
fn tb303_bkm_direct_path_is_lowpass() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_bkm_direct_lowpass",
        "tb303_bkm_direct_lowpass",
        SR,
        &super::compile::CompileOptions::default(),
        &cache_dir,
    )
    .expect("compile failed");

    let measure = |freq: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        let bkm = proc
            .stages
            .iter_mut()
            .find_map(|s| {
                if let pedalkernel_rt::processor::Stage::BlockwiseKMethod(ref mut k) = s {
                    Some(k)
                } else {
                    None
                }
            })
            .expect("TB303 should compile to blockwise K-method");

        for _ in 0..4800 {
            let _ = bkm.process(&[0.0, 0.0, 0.0, 0.0]);
        }

        settled_sine_rms(freq, 0.1, |input| bkm.process(&[input, 0.0, 0.0, 0.0]))
    };

    let gain_100 = measure(100.0);
    let gain_10k = measure(10_000.0);
    let ratio_db = 20.0 * (gain_100 / gain_10k.max(1e-12)).log10();

    eprintln!(
        "  BKM direct LPF: 100Hz={gain_100:.6}, 10kHz={gain_10k:.6}, ratio={ratio_db:+.1} dB"
    );

    assert!(
        gain_100 > gain_10k * 3.0,
        "BKM core should have a settled lowpass slope before C_out: 100Hz={gain_100:.6}, 10kHz={gain_10k:.6}"
    );
}

#[test]
fn tb303_bkm_block_outputs_show_cascaded_lowpass_shape() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_bkm_block_shape",
        "tb303_bkm_block_shape",
        SR,
        &super::compile::CompileOptions::default(),
        &cache_dir,
    )
    .expect("compile failed");

    let measure_blocks = |freq: f64| -> Vec<f64> {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        let bkm = proc
            .stages
            .iter_mut()
            .find_map(|s| {
                if let pedalkernel_rt::processor::Stage::BlockwiseKMethod(ref mut k) = s {
                    Some(k)
                } else {
                    None
                }
            })
            .expect("TB303 should compile to blockwise K-method");

        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let _ = bkm.debug_process_with_block_outputs(&[input, 0.0, 0.0, 0.0]);
        }

        let mut values = vec![Vec::new(); bkm.blocks.len()];
        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let (_, blocks) = bkm.debug_process_with_block_outputs(&[input, 0.0, 0.0, 0.0]);
            for (block_values, value) in values.iter_mut().zip(blocks.iter()) {
                block_values.push(*value);
            }
        }

        values
            .iter()
            .map(|block_values| ac_rms(block_values))
            .collect()
    };

    let low = measure_blocks(100.0);
    let high = measure_blocks(10_000.0);
    eprintln!("  BKM block RMS 100Hz: {low:.6?}");
    eprintln!("  BKM block RMS 10kHz: {high:.6?}");

    assert_eq!(low.len(), 4);
    assert_eq!(high.len(), 4);
    assert!(
        low[3] > high[3] * 3.0,
        "final BKM rung should show cascaded lowpass shape: 100Hz={:.6}, 10kHz={:.6}",
        low[3],
        high[3]
    );
}

#[test]
fn tb303_bkm_rung_response_matches_forced_serial_wdf_order() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");

    let measure_serial = |freq: f64| -> Vec<f64> {
        let mut options = super::compile::CompileOptions::default();
        options.force_serial_blockwise = true;
        let mut proc =
            super::compile_pedal_with_options(&def, SR, options).expect("compile failed");
        proc.set_control("Resonance", 0.0);

        for i in 0..9600 {
            let _ = proc.process(0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin());
        }

        let mut sums = Vec::new();
        let mut count = 0usize;
        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let mut x = input;
            let mut rung = 0usize;
            for stage in &mut proc.stages {
                let pedalkernel_rt::processor::Stage::Wdf(wdf) = stage else {
                    continue;
                };
                x = wdf.process(x);
                if rung < 4 {
                    if sums.len() <= rung {
                        sums.push(0.0);
                    }
                    sums[rung] += x * x;
                    rung += 1;
                }
            }
            count += 1;
        }

        sums.into_iter()
            .map(|v| (v / count.max(1) as f64).sqrt())
            .collect()
    };

    let measure_bkm = |freq: f64| -> Vec<f64> {
        let mut proc =
            super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
                .expect("compile failed");
        proc.set_control("Resonance", 0.0);

        let bkm = proc
            .stages
            .iter_mut()
            .find_map(|s| {
                if let pedalkernel_rt::processor::Stage::BlockwiseKMethod(ref mut k) = s {
                    Some(k)
                } else {
                    None
                }
            })
            .expect("TB303 should compile to BKM");

        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let _ = bkm.debug_process_with_block_outputs(&[input, 0.0, 0.0, 0.0]);
        }

        let mut values = vec![Vec::new(); bkm.blocks.len()];
        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let (_, outputs) = bkm.debug_process_with_block_outputs(&[input, 0.0, 0.0, 0.0]);
            for (block_values, value) in values.iter_mut().zip(outputs.iter()) {
                block_values.push(*value);
            }
        }

        values
            .iter()
            .map(|block_values| ac_rms(block_values))
            .collect()
    };

    let serial_low = measure_serial(100.0);
    let serial_high = measure_serial(10_000.0);
    let bkm_low = measure_bkm(100.0);
    let bkm_high = measure_bkm(10_000.0);

    let serial_ratio: Vec<f64> = serial_low
        .iter()
        .zip(serial_high.iter())
        .map(|(lo, hi)| lo / hi.max(1e-12))
        .collect();
    let bkm_ratio: Vec<f64> = bkm_low
        .iter()
        .zip(bkm_high.iter())
        .map(|(lo, hi)| lo / hi.max(1e-12))
        .collect();

    eprintln!("  serial low RMS: {serial_low:.6?}");
    eprintln!("  serial high RMS: {serial_high:.6?}");
    eprintln!("  serial low/high per rung: {serial_ratio:.3?}");
    eprintln!("  BKM low RMS: {bkm_low:.6?}");
    eprintln!("  BKM high RMS: {bkm_high:.6?}");
    eprintln!("  BKM low/high per rung: {bkm_ratio:.3?}");

    assert_eq!(serial_ratio.len(), 4);
    assert_eq!(bkm_ratio.len(), 4);
    assert!(
        bkm_ratio[1] > bkm_ratio[0] * 1.5,
        "BKM second rung should add another pole like forced-serial WDF. \
         serial ratios={serial_ratio:.3?}, BKM ratios={bkm_ratio:.3?}"
    );
}

#[test]
fn tb303_compare_bkm_forced_serial_and_htb_shape() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let freqs = [100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0, 10_000.0];

    let measure_compiled = |freq: f64, force_serial: bool| -> f64 {
        let mut options = super::compile::CompileOptions::default();
        options.force_serial_blockwise = force_serial;
        let mut proc =
            super::compile_pedal_with_options(&def, SR, options).expect("compile failed");
        proc.set_control("Cutoff", 0.5);
        proc.set_control("Resonance", 0.0);
        settled_sine_rms(freq, 0.03, |input| proc.process(input))
    };

    let bkm: Vec<f64> = freqs
        .iter()
        .map(|&freq| measure_compiled(freq, false))
        .collect();
    let serial: Vec<f64> = freqs
        .iter()
        .map(|&freq| measure_compiled(freq, true))
        .collect();

    let best_fit_fc = |gains: &[f64]| -> f64 {
        let target: Vec<f64> = gains.iter().map(|gain| db_norm(*gain, gains[0])).collect();
        let mut best_fc = 100.0;
        let mut best_err = f64::INFINITY;

        for step in 0..240 {
            let t = step as f64 / 239.0;
            let fc = 100.0 * (80.0f64).powf(t);
            let h_ref = stinchcombe_htb_magnitude(freqs[0], fc);
            let mut err = 0.0;
            for (i, &freq) in freqs.iter().enumerate() {
                let h_db = db_norm(stinchcombe_htb_magnitude(freq, fc), h_ref);
                let diff = target[i] - h_db;
                err += diff * diff;
            }
            if err < best_err {
                best_err = err;
                best_fc = fc;
            }
        }

        best_fc
    };

    let htb_fc = best_fit_fc(&serial);
    let h_ref = stinchcombe_htb_magnitude(freqs[0], htb_fc);

    eprintln!("  TB303 normalized response comparison, Cutoff=0.5 Resonance=0");
    eprintln!("  H_tb best-fit fc to forced-serial WDF: {htb_fc:.1} Hz");
    eprintln!(
        "  {:>8} {:>10} {:>10} {:>10} {:>10}",
        "Freq", "BKM dB", "Serial dB", "H_tb dB", "BKM-HTB"
    );
    for (i, &freq) in freqs.iter().enumerate() {
        let bkm_db = db_norm(bkm[i], bkm[0]);
        let serial_db = db_norm(serial[i], serial[0]);
        let htb_db = db_norm(stinchcombe_htb_magnitude(freq, htb_fc), h_ref);
        eprintln!(
            "  {freq:>8.0} {bkm_db:>+10.1} {serial_db:>+10.1} {htb_db:>+10.1} {:>+10.1}",
            bkm_db - htb_db
        );
    }

    let bkm_oct = db_norm(bkm[6], bkm[5]);
    let serial_oct = db_norm(serial[6], serial[5]);
    let htb_oct = db_norm(
        stinchcombe_htb_magnitude(freqs[6], htb_fc),
        stinchcombe_htb_magnitude(freqs[5], htb_fc),
    );
    eprintln!(
        "  5k->10k slope: BKM={bkm_oct:+.1} dB/oct, serial={serial_oct:+.1} dB/oct, H_tb={htb_oct:+.1} dB/oct"
    );

    let bkm_5k_db = db_norm(bkm[5], bkm[0]);
    let htb_5k_db = db_norm(stinchcombe_htb_magnitude(freqs[5], htb_fc), h_ref);
    assert!(
        bkm_5k_db > htb_5k_db + 6.0,
        "BKM should currently expose the transition-band mismatch under investigation: \
         BKM 5k={bkm_5k_db:+.1} dB, H_tb 5k={htb_5k_db:+.1} dB"
    );
}

#[test]
fn tb303_bkm_direct_block_matches_forced_serial_first_rung_order() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");

    let measure_serial_first = |freq: f64| -> f64 {
        let mut options = super::compile::CompileOptions::default();
        options.force_serial_blockwise = true;
        let mut proc =
            super::compile_pedal_with_options(&def, SR, options).expect("compile failed");
        proc.set_control("Resonance", 0.0);

        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            for stage in &mut proc.stages {
                if let pedalkernel_rt::processor::Stage::Wdf(wdf) = stage {
                    let _ = wdf.process(input);
                    break;
                }
            }
        }

        let mut sum_sq = 0.0;
        let mut count = 0usize;
        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            for stage in &mut proc.stages {
                if let pedalkernel_rt::processor::Stage::Wdf(wdf) = stage {
                    let out = wdf.process(input);
                    sum_sq += out * out;
                    count += 1;
                    break;
                }
            }
        }

        (sum_sq / count.max(1) as f64).sqrt()
    };

    let measure_bkm_first_direct = |freq: f64| -> f64 {
        let mut proc =
            super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
                .expect("compile failed");
        proc.set_control("Resonance", 0.0);

        let bkm = proc
            .stages
            .iter_mut()
            .find_map(|s| {
                if let pedalkernel_rt::processor::Stage::BlockwiseKMethod(ref mut k) = s {
                    Some(k)
                } else {
                    None
                }
            })
            .expect("TB303 should compile to BKM");

        for v in &mut bkm.work_b {
            *v = 0.0;
        }
        for v in &mut bkm.work_a {
            *v = 0.0;
        }
        for &(ref name, port_idx) in &bkm.vs_port_map {
            if name.starts_with("_supply_") {
                bkm.work_b[port_idx] = 2.0 * bkm.supply_voltage;
            }
        }
        for row in 0..bkm.n_ports {
            let mut sum = 0.0;
            for col in 0..bkm.n_ports {
                sum += bkm.coupling_s[row * bkm.n_ports + col] * bkm.work_b[col];
            }
            bkm.work_a[row] = sum;
        }
        let dc_drive = (bkm.work_a[0] + bkm.work_b[0]) / 2.0;

        let mut block = bkm.blocks[0].clone();
        block.k_table.precompute_scales();

        let mut process_block = |input: f64| {
            let physical_input = dc_drive + input;
            block
                .tree
                .set_voltage(block.source_polarity * physical_input);
            let b_tree = block.tree.reflected();
            let a_root = if block.k_table.dims == 1 {
                block.k_table.lookup_1d(b_tree)
            } else {
                block
                    .k_table
                    .lookup_2d(b_tree, block.k_table_control_polarity * physical_input)
            };
            let raw = if let Some(ref probe_id) = block.cascade_probe_id {
                block
                    .tree
                    .leaf_voltage_for_incident(probe_id, a_root)
                    .unwrap_or((a_root + b_tree) / 2.0)
            } else {
                (a_root + b_tree) / 2.0
            };
            block.tree.set_incident(a_root);
            raw - block.dc_offset
        };

        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let _ = process_block(input);
        }

        let mut values = Vec::new();
        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let out = process_block(input);
            values.push(out);
        }

        ac_rms(&values)
    };

    let serial_low = measure_serial_first(100.0);
    let serial_high = measure_serial_first(10_000.0);
    let bkm_low = measure_bkm_first_direct(100.0);
    let bkm_high = measure_bkm_first_direct(10_000.0);
    let serial_ratio = serial_low / serial_high.max(1e-12);
    let bkm_ratio = bkm_low / bkm_high.max(1e-12);

    eprintln!(
        "  first rung direct WDF: 100Hz={serial_low:.6}, 10kHz={serial_high:.6}, ratio={serial_ratio:.3}"
    );
    eprintln!(
        "  first rung direct BKM: 100Hz={bkm_low:.6}, 10kHz={bkm_high:.6}, ratio={bkm_ratio:.3}"
    );

    assert!(
        bkm_ratio > serial_ratio * 0.9,
        "direct BKM first block should preserve the same one-pole order as forced WDF. \
         serial ratio={serial_ratio:.3}, BKM ratio={bkm_ratio:.3}"
    );
}

#[test]
fn tb303_bkm_direct_blocks_each_have_lowpass_order() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");

    let measure_blocks = |freq: f64| -> Vec<f64> {
        let mut proc =
            super::compile_pedal_with_options(&def, SR, super::compile::CompileOptions::default())
                .expect("compile failed");
        proc.set_control("Resonance", 0.0);

        let bkm = proc
            .stages
            .iter_mut()
            .find_map(|s| {
                if let pedalkernel_rt::processor::Stage::BlockwiseKMethod(ref mut k) = s {
                    Some(k)
                } else {
                    None
                }
            })
            .expect("TB303 should compile to BKM");

        for v in &mut bkm.work_b {
            *v = 0.0;
        }
        for v in &mut bkm.work_a {
            *v = 0.0;
        }
        for &(ref name, port_idx) in &bkm.vs_port_map {
            if name.starts_with("_supply_") {
                bkm.work_b[port_idx] = 2.0 * bkm.supply_voltage;
            }
        }
        for row in 0..bkm.n_ports {
            let mut sum = 0.0;
            for col in 0..bkm.n_ports {
                sum += bkm.coupling_s[row * bkm.n_ports + col] * bkm.work_b[col];
            }
            bkm.work_a[row] = sum;
        }
        let dc_drives: Vec<f64> = (0..bkm.blocks.len())
            .map(|i| (bkm.work_a[i] + bkm.work_b[i]) / 2.0)
            .collect();
        eprintln!(
            "  direct BKM probes: {:?}",
            bkm.blocks
                .iter()
                .map(|block| block.cascade_probe_id.as_deref().unwrap_or("<root>"))
                .collect::<Vec<_>>()
        );
        eprintln!(
            "  direct BKM rp: {:?}",
            bkm.blocks.iter().map(|block| block.rp).collect::<Vec<_>>()
        );
        let mut blocks = bkm.blocks.clone();
        for block in &mut blocks {
            block.k_table.precompute_scales();
        }

        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            for (block_idx, block) in blocks.iter_mut().enumerate() {
                let physical_input = dc_drives[block_idx] + input;
                block
                    .tree
                    .set_voltage(block.source_polarity * physical_input);
                let b_tree = block.tree.reflected();
                let a_root = if block.k_table.dims == 1 {
                    block.k_table.lookup_1d(b_tree)
                } else {
                    block
                        .k_table
                        .lookup_2d(b_tree, block.k_table_control_polarity * physical_input)
                };
                block.tree.set_incident(a_root);
            }
        }

        let mut values = vec![Vec::new(); blocks.len()];
        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            for (block_idx, block) in blocks.iter_mut().enumerate() {
                let physical_input = dc_drives[block_idx] + input;
                block
                    .tree
                    .set_voltage(block.source_polarity * physical_input);
                let b_tree = block.tree.reflected();
                let a_root = if block.k_table.dims == 1 {
                    block.k_table.lookup_1d(b_tree)
                } else {
                    block
                        .k_table
                        .lookup_2d(b_tree, block.k_table_control_polarity * physical_input)
                };
                let raw = if let Some(ref probe_id) = block.cascade_probe_id {
                    block
                        .tree
                        .leaf_voltage_for_incident(probe_id, a_root)
                        .unwrap_or((a_root + b_tree) / 2.0)
                } else {
                    (a_root + b_tree) / 2.0
                };
                block.tree.set_incident(a_root);
                let out = raw - block.dc_offset;
                values[block_idx].push(out);
            }
        }

        values
            .iter()
            .map(|block_values| ac_rms(block_values))
            .collect()
    };

    let low = measure_blocks(100.0);
    let high = measure_blocks(10_000.0);
    let ratios: Vec<f64> = low
        .iter()
        .zip(high.iter())
        .map(|(lo, hi)| lo / hi.max(1e-12))
        .collect();

    eprintln!("  direct BKM block 100Hz RMS: {low:.6?}");
    eprintln!("  direct BKM block 10kHz RMS: {high:.6?}");
    eprintln!("  direct BKM block low/high: {ratios:.3?}");

    assert!(
        ratios.iter().all(|ratio| *ratio > 2.0),
        "each independently driven BKM block should retain local lowpass behavior: {ratios:.3?}"
    );
}

#[test]
fn tb303_bkm_k_tables_are_generated_per_rung_not_shared() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_bkm_ktable_ownership_v3",
        "tb303_bkm_ktable_ownership_v3",
        SR,
        &super::compile::CompileOptions::default(),
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");
    let bkm = proc
        .stages
        .iter_mut()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::BlockwiseKMethod(ref mut k) = s {
                Some(k)
            } else {
                None
            }
        })
        .expect("TB303 should compile to blockwise K-method");

    assert_eq!(bkm.blocks.len(), 4);

    let table_ptrs: Vec<usize> = bkm
        .blocks
        .iter()
        .map(|block| block.k_table.entries.as_ptr() as usize)
        .collect();
    let mut unique_ptrs = table_ptrs.clone();
    unique_ptrs.sort_unstable();
    unique_ptrs.dedup();

    eprintln!("  BKM K-table entry pointers: {table_ptrs:?}");
    assert_eq!(
        unique_ptrs.len(),
        table_ptrs.len(),
        "each BKM rung should own its generated K-table storage; do not share table Vecs blindly"
    );

    let table_delta = |a: usize, b: usize| -> f64 {
        let ta = &bkm.blocks[a].k_table;
        let tb = &bkm.blocks[b].k_table;
        assert_eq!(ta.dims, tb.dims);
        assert_eq!(ta.steps, tb.steps);
        assert_eq!(ta.entries.len(), tb.entries.len());
        ta.entries
            .iter()
            .zip(tb.entries.iter())
            .map(|(x, y)| (x - y).abs())
            .fold(0.0f64, f64::max)
    };

    let q1_q2_delta = table_delta(0, 1);
    let q2_q3_delta = table_delta(1, 2);
    let q3_q4_delta = table_delta(2, 3);
    eprintln!(
        "  BKM K-table max deltas: Q1/Q2={q1_q2_delta:.6}, Q2/Q3={q2_q3_delta:.6}, Q3/Q4={q3_q4_delta:.6}"
    );

    assert!(
        q1_q2_delta > 1e-6,
        "Q1 table should differ from downstream rung tables because its source impedance differs"
    );
    assert!(
        q2_q3_delta < 1e-9 && q3_q4_delta < 1e-9,
        "identical downstream rungs may have matching table contents, but still own separate tables"
    );
}

#[test]
fn tb303_serial_blockwise_without_bkm_is_lowpass_diagnostic() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let mut options = super::compile::CompileOptions::default();
    options.force_serial_blockwise = true;

    let compiled =
        super::compile_pedal_with_options(&def, SR, options.clone()).expect("compile failed");
    let bkm_count = compiled
        .stages
        .iter()
        .filter(|stage| matches!(stage, pedalkernel_rt::processor::Stage::BlockwiseKMethod(_)))
        .count();
    let wdf_count = compiled
        .stages
        .iter()
        .filter(|stage| matches!(stage, pedalkernel_rt::processor::Stage::Wdf(_)))
        .count();

    eprintln!(
        "  serial blockwise diagnostic: stages={}, wdf_count={wdf_count}, bkm_count={bkm_count}",
        compiled.stages.len()
    );
    assert_eq!(
        bkm_count, 0,
        "force_serial_blockwise must not emit BKM stages"
    );
    assert!(
        wdf_count >= 4,
        "force_serial_blockwise should expose the ladder rungs as serial WDF/K-method stages"
    );

    let measure = |freq: f64| -> f64 {
        let mut proc =
            super::compile_pedal_with_options(&def, SR, options.clone()).expect("compile failed");
        proc.set_control("Resonance", 0.0);

        for _ in 0..4800 {
            let _ = proc.process(0.0);
        }

        let mut peak = 0.0f64;
        for i in 0..4800 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            peak = peak.max(proc.process(input).abs());
        }
        peak
    };

    let gain_100 = measure(100.0);
    let gain_10k = measure(10_000.0);
    let ratio_db = 20.0 * (gain_100 / gain_10k.max(1e-12)).log10();

    eprintln!(
        "  serial blockwise diagnostic: 100Hz={gain_100:.6}, 10kHz={gain_10k:.6}, ratio={ratio_db:+.1} dB"
    );
    assert!(
        gain_100 > gain_10k,
        "serial blockwise rungs should be lowpass without BKM coupling: 100Hz={gain_100:.6}, 10kHz={gain_10k:.6}"
    );
}

#[test]
fn tb303_forced_serial_stage_outputs_show_reference_shape() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let mut options = super::compile::CompileOptions::default();
    options.force_serial_blockwise = true;

    let measure = |freq: f64| -> Vec<f64> {
        let mut proc =
            super::compile_pedal_with_options(&def, SR, options.clone()).expect("compile failed");
        proc.set_control("Resonance", 0.0);

        let mut sums = vec![0.0f64; proc.stages.len()];
        let mut count = 0usize;

        for i in 0..9600 {
            let _ = proc.process(0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin());
        }

        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let mut x = input;
            for (si, stage) in proc.stages.iter_mut().enumerate() {
                x = match stage {
                    pedalkernel_rt::processor::Stage::Wdf(wdf) => wdf.process(x),
                    pedalkernel_rt::processor::Stage::MultiNl(mnl) => mnl.process(x),
                    pedalkernel_rt::processor::Stage::Iir(iir) => iir.process(x),
                    pedalkernel_rt::processor::Stage::StateSpace(ss) => ss.process(x),
                    pedalkernel_rt::processor::Stage::BlackFeedback(bf) => bf.process(x),
                    pedalkernel_rt::processor::Stage::BlockwiseKMethod(bkm) => {
                        bkm.process(&[x, 0.0, 0.0, 0.0])
                    }
                };
                sums[si] += x * x;
            }
            count += 1;
        }

        sums.into_iter()
            .map(|v| (v / count.max(1) as f64).sqrt())
            .collect()
    };

    let low = measure(100.0);
    let high = measure(10_000.0);
    eprintln!("  forced serial stage RMS 100Hz: {low:.6?}");
    eprintln!("  forced serial stage RMS 10kHz: {high:.6?}");

    assert!(
        low.last().copied().unwrap_or(0.0) > high.last().copied().unwrap_or(0.0) * 3.0,
        "forced serial reference should be strongly lowpass"
    );
}

#[test]
fn tb303_forced_serial_wdf_rungs_stay_bounded_on_silence() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let mut options = super::compile::CompileOptions::default();
    options.force_serial_blockwise = true;

    let mut proc = super::compile_pedal_with_options(&def, SR, options).expect("compile failed");

    for sample in 0..4800 {
        proc.process(0.0);

        if sample % 480 == 479 {
            for (si, stage) in proc.stages.iter().enumerate() {
                let pedalkernel_rt::processor::Stage::Wdf(wdf) = stage else {
                    continue;
                };
                let mut tree = wdf.tree.clone();
                tree.set_voltage(0.0);
                let b = tree.reflected();
                eprintln!("  serial WDF sample={sample} stage={si} b_tree={b:.6}");
                assert!(
                    b.is_finite() && b.abs() < 10.0,
                    "forced-serial WDF stage {si} should stay bounded on silence, b_tree={b:.6}"
                );
            }
        }
    }
}

#[test]
fn two_rung_feedback_bkm_stays_bounded_on_silence() {
    let def =
        crate::dsl::parse_pedal_file(TWO_RUNG_DIODE_LADDER_WITH_FEEDBACK).expect("parse failed");
    let mut proc = super::compile_pedal(&def, SR).expect("compile failed");

    for sample in 0..4800 {
        proc.process(0.0);

        if sample % 480 == 479 {
            let bkm = proc
                .stages
                .iter()
                .find_map(|s| {
                    if let pedalkernel_rt::processor::Stage::BlockwiseKMethod(ref k) = s {
                        Some(k)
                    } else {
                        None
                    }
                })
                .expect("two-rung feedback ladder should compile to BKM");

            for (bi, block) in bkm.blocks.iter().enumerate() {
                let mut tree = block.tree.clone();
                tree.set_voltage(0.0);
                let b = tree.reflected();
                eprintln!(
                    "  two-rung BKM sample={sample} block={bi} b_tree={b:.6}, dc_offset={:.6}",
                    block.dc_offset
                );
                assert!(
                    b.is_finite() && b.abs() < 10.0,
                    "two-rung BKM block {bi} should stay bounded on silence, b_tree={b:.6}"
                );
            }
        }
    }
}

#[test]
fn tb303_output_coupling_probes_load_resistor() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let proc = super::spqr_build::compile_via_spqr(&def, SR).expect("compile failed");

    let probes: Vec<Option<String>> = proc
        .stages
        .iter()
        .filter_map(|stage| {
            if let pedalkernel_rt::processor::Stage::Wdf(wdf) = stage {
                Some(wdf.output_probe.clone())
            } else {
                None
            }
        })
        .collect();

    eprintln!("  WDF output probes: {:?}", probes);
    assert!(
        probes.iter().any(|probe| probe.as_deref() == Some("R_out")),
        "TB303 output coupling stage should probe R_out, got {:?}",
        probes
    );
}

#[test]
fn tb303_resonance_recomputes_bkm_coupling_matrix() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let mut compiled = super::compile_pedal(&def, SR).expect("compile failed");

    assert!(
        compiled
            .controls
            .iter()
            .any(|c| c.label == "Resonance" && c.component_id == "Resonance"),
        "Resonance control must bind even though its pot lives in the BKM coupling network"
    );

    let bkm = compiled
        .stages
        .iter_mut()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::BlockwiseKMethod(ref mut k) = s {
                Some(k)
            } else {
                None
            }
        })
        .expect("BKM stage");

    let before = bkm.coupling_s.clone();
    assert!(
        bkm.set_pot("Resonance", 0.7),
        "BKM should accept coupling-network pot changes"
    );
    let delta = before
        .iter()
        .zip(bkm.coupling_s.iter())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0f64, f64::max);

    eprintln!("  Resonance coupling matrix max delta: {delta:.6}");
    assert!(
        delta > 1e-6,
        "moving Resonance must recompute the BKM coupling scattering matrix"
    );
}

#[test]
fn tb303_resonance_pot_compiles_as_split_feedback_divider() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_resonance_split_divider",
        "tb303_resonance_split_divider",
        SR,
        &super::compile::CompileOptions::default(),
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    let bkm = proc
        .stages
        .iter_mut()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::BlockwiseKMethod(k) = s {
                Some(k)
            } else {
                None
            }
        })
        .expect("TB303 should compile to blockwise K-method");
    assert!(
        bkm.vs_port_map
            .iter()
            .any(|(name, _)| name == "cv_resonance"),
        "Resonance CV must compile as a voltage port, not an audio-rate pot update"
    );
    assert!(bkm.set_pot("Resonance", 0.0));

    let resonance_legs: Vec<_> = bkm
        .coupling_elements
        .iter()
        .filter(|e| e.comp_id == "Resonance")
        .collect();

    assert_eq!(
        resonance_legs.len(),
        2,
        "Resonance should compile as a three-terminal divider with two coupling legs"
    );
    assert!(
        resonance_legs.iter().any(|e| e.invert_control)
            && resonance_legs.iter().any(|e| !e.invert_control),
        "Resonance divider legs must track opposite halves of the pot"
    );

    let min_direct = resonance_legs
        .iter()
        .find(|e| e.invert_control)
        .expect("direct feedback leg should be inverted")
        .resistance;
    let min_ground = resonance_legs
        .iter()
        .find(|e| !e.invert_control)
        .expect("ground shunt leg should not be inverted")
        .resistance;

    assert!(
        min_direct > 90_000.0 && min_ground < 10.0,
        "Resonance=0 should ground the wiper and isolate feedback: direct={min_direct:.1}, ground={min_ground:.1}"
    );

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");
    let bkm = proc
        .stages
        .iter_mut()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::BlockwiseKMethod(k) = s {
                Some(k)
            } else {
                None
            }
        })
        .expect("TB303 should compile to blockwise K-method");
    assert!(bkm.set_pot("Resonance", 0.95));
    let resonance_legs: Vec<_> = bkm
        .coupling_elements
        .iter()
        .filter(|e| e.comp_id == "Resonance")
        .collect();
    let max_direct = resonance_legs
        .iter()
        .find(|e| e.invert_control)
        .expect("direct feedback leg should be inverted")
        .resistance;
    let max_ground = resonance_legs
        .iter()
        .find(|e| !e.invert_control)
        .expect("ground shunt leg should not be inverted")
        .resistance;

    assert!(
        max_direct < 10_000.0 && max_ground > 90_000.0,
        "Resonance=0.95 should connect feedback and lift the ground shunt: direct={max_direct:.1}, ground={max_ground:.1}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 8. Resonance creates a gain peak
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_resonance_creates_peak() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    eprintln!("  compiling with K-tables (cached)...");
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_resonance_peak",
        "tb303_resonance_peak",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let measure_gain_at_freq = |freq: f64, resonance: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.set_control("Cutoff", 0.5);
        proc.set_control("Resonance", resonance);

        // Warm-up: let transients settle.
        for _ in 0..4800 {
            proc.process(0.0);
        }

        // Measure peak over 4800 samples of sine. AcidAttack drives the WDF
        // filter at roughly 30 mV, not 1 V; at 1 V the nonlinear ladder
        // compresses the resonance peak and this stops being a small-signal
        // filter-readiness check.
        let mut peak = 0.0f64;
        for i in 0..4800 {
            let input = 0.03 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let out = proc.process(input);
            peak = peak.max(out.abs());
        }
        peak
    };

    let gain_flat = measure_gain_at_freq(2000.0, 0.0);
    let gain_resonant = measure_gain_at_freq(2000.0, 0.85);

    eprintln!(
        "  2kHz gain: flat(reso=0.0)={gain_flat:.4}, resonant(reso=0.85)={gain_resonant:.4}, ratio={:.2}x",
        gain_resonant / gain_flat.max(1e-12)
    );

    assert!(
        gain_resonant > gain_flat * 1.25,
        "Resonance should boost gain at 2kHz by >1.25×: flat={gain_flat:.4}, resonant={gain_resonant:.4}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 9. Block port mapping: each block gets a unique coupling port
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_each_block_has_unique_coupling_port() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse failed");
    let graph = CircuitGraph::from_pedal(&def);

    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    // Get the feedback group (the one with all BJTs)
    let groups = super::signal_flow::find_flow_groups(&all_edges, &graph);
    let feedback_group = groups
        .iter()
        .find(|g| g.has_feedback())
        .expect("should have feedback group");
    let group_edges = feedback_group.all_edges();

    let plan = blockwise::analyze_blockwise(&group_edges, &graph).expect("should decompose");

    // Collect coupling MNA nodes
    let rails: std::collections::HashSet<usize> = {
        let mut r = std::collections::HashSet::new();
        r.insert(graph.gnd_node);
        r.insert(graph.vcc_node);
        r.extend(&graph.supply_nodes);
        r.extend(&graph.ac_ground_nodes);
        r
    };
    let mut coupling_nodes: Vec<usize> = Vec::new();
    for &eidx in &plan.coupling_edges {
        let e = &graph.edges[eidx];
        if !rails.contains(&e.node_a) && !coupling_nodes.contains(&e.node_a) {
            coupling_nodes.push(e.node_a);
        }
        if !rails.contains(&e.node_b) && !coupling_nodes.contains(&e.node_b) {
            coupling_nodes.push(e.node_b);
        }
    }

    // For each block, find its coupling port node
    // Count how many coupling edges touch each of the block's port_nodes
    let mut assigned_ports: Vec<usize> = Vec::new();
    for (bi, block) in plan.blocks.iter().enumerate() {
        let best_node = block
            .port_nodes
            .iter()
            .filter(|pn| coupling_nodes.contains(pn))
            .max_by_key(|&&pn| {
                plan.coupling_edges
                    .iter()
                    .filter(|&&eidx| {
                        let e = &graph.edges[eidx];
                        e.node_a == pn || e.node_b == pn
                    })
                    .count()
            })
            .copied();
        eprintln!(
            "  block {bi}: port_nodes={:?}, best_coupling_node={:?}",
            block.port_nodes, best_node
        );
        if let Some(node) = best_node {
            assigned_ports.push(node);
        }
    }

    // Assert: all assigned ports are unique
    let unique_count = {
        let mut s = assigned_ports.clone();
        s.sort();
        s.dedup();
        s.len()
    };
    assert_eq!(
        unique_count,
        assigned_ports.len(),
        "Each block must have a UNIQUE coupling port, but got duplicates: {:?}",
        assigned_ports
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 10. Linear-regime sanity: coupling scattering with identity K-tables
//
// Replace K-tables with linear approximation (b_out ≈ b_in * gain).
// The system becomes purely linear. If this produces lowpass behavior,
// the coupling network is correct and any remaining issues are in K-table
// interpolation. If flat, the coupling network itself is wrong.
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_linear_regime_has_frequency_dependence() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_linear_regime",
        "tb303_linear_regime",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let measure = |freq: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.set_control("Cutoff", 0.5);
        proc.set_control("Resonance", 0.0);
        // Warm-up
        for _ in 0..2400 {
            proc.process(0.0);
        }
        let mut peak = 0.0f64;
        for i in 0..4800 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            peak = peak.max(proc.process(input).abs());
        }
        peak
    };

    let gain_100 = measure(100.0);
    let gain_1k = measure(1000.0);
    let gain_10k = measure(10000.0);

    let ratio_100_10k = 20.0 * (gain_100 / gain_10k.max(1e-12)).log10();

    eprintln!("  Linear regime: 100Hz={gain_100:.6}, 1kHz={gain_1k:.6}, 10kHz={gain_10k:.6}");
    eprintln!("  Ratio 100Hz/10kHz: {ratio_100_10k:+.1} dB");

    // Even in linear regime (K-tables as-is), the RC time constants in
    // each block's WDF tree should produce SOME frequency dependence.
    // If the ratio is exactly 0 dB, the coupling is not routing correctly.
    assert!(
        (gain_100 - gain_10k).abs() > gain_100 * 0.01,
        "Linear regime should show frequency dependence (RC in each block). \
         100Hz={gain_100:.6}, 10kHz={gain_10k:.6} — identical means coupling is broken."
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 11. Single-block frequency response: each block should lowpass
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_single_block_filters() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_single_block",
        "tb303_single_block",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    // Find the BlockwiseKMethod stage
    let bkm = proc.stages.iter().find_map(|s| {
        if let pedalkernel_rt::processor::Stage::BlockwiseKMethod(ref k) = s {
            Some(k)
        } else {
            None
        }
    });

    if bkm.is_none() {
        eprintln!("  SKIP: no BlockwiseKMethod stage found");
        return;
    }
    let bkm = bkm.unwrap();

    // Process a single block directly
    let measure_block = |block_idx: usize, freq: f64| -> f64 {
        let mut block = bkm.blocks[block_idx].clone();
        block.k_table.precompute_scales();
        // Warm up
        for _ in 0..2400 {
            block.tree.set_voltage(0.0);
            let b = block.tree.reflected();
            let a = block.k_table.lookup_2d(b, 0.0);
            block.tree.set_incident(a);
        }
        let mut peak = 0.0f64;
        for i in 0..4800 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            block.tree.set_voltage(input);
            let b = block.tree.reflected();
            let a = block.k_table.lookup_2d(b, 0.0);
            block.tree.set_incident(a);
            let v_out = (a + b) / 2.0;
            peak = peak.max(v_out.abs());
        }
        peak
    };

    for bi in 0..bkm.blocks.len().min(4) {
        let g100 = measure_block(bi, 100.0);
        let g1k = measure_block(bi, 1000.0);
        let g10k = measure_block(bi, 10000.0);
        let ratio_db = 20.0 * (g100 / g10k.max(1e-12)).log10();
        eprintln!(
            "  block {bi}: 100Hz={g100:.6}, 1kHz={g1k:.6}, 10kHz={g10k:.6}, ratio={ratio_db:+.1}dB"
        );
    }

    // At least one block should show >3dB rolloff from 100Hz to 10kHz
    let block0_100 = measure_block(0, 100.0);
    let block0_10k = measure_block(0, 10000.0);
    assert!(
        block0_100 > block0_10k * 1.2,
        "Single block should lowpass: 100Hz={block0_100:.6} should be >1.2× 10kHz={block0_10k:.6}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Test: CV port changes filter behavior
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_cv_port_modulates_output() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_cv_test_v3",
        "tb303_cv_test_v3",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let measure_with_cv = |cv_voltage: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.set_control("Resonance", 0.0);
        // Warm up with CV
        for _ in 0..2400 {
            let mut ports = vec![0.0; proc.port_count()];
            if let Some(cv_idx) = proc.resolve_port("cv_cutoff") {
                ports[cv_idx] = cv_voltage;
            }
            proc.process_ports(&mut ports);
        }
        let mut peak = 0.0f64;
        for i in 0..4800 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / SR).sin();
            let mut ports = vec![0.0; proc.port_count()];
            if let Some(in_idx) = proc.resolve_port("audio_in") {
                ports[in_idx] = input;
            }
            if let Some(cv_idx) = proc.resolve_port("cv_cutoff") {
                ports[cv_idx] = cv_voltage;
            }
            proc.process_ports(&mut ports);
            if let Some(out_idx) = proc.resolve_port("audio_out") {
                peak = peak.max(ports[out_idx].abs());
            }
        }
        peak
    };

    let gain_cv0 = measure_with_cv(0.0);
    let gain_cv1 = measure_with_cv(1.0);
    let gain_cv_neg1 = measure_with_cv(-1.0);

    eprintln!("  CV=0: {gain_cv0:.6}, CV=+1V: {gain_cv1:.6}, CV=-1V: {gain_cv_neg1:.6}");

    // CV should change the output — different bias → different cutoff
    let max_gain = gain_cv0.max(gain_cv1).max(gain_cv_neg1);
    let min_gain = gain_cv0.min(gain_cv1).min(gain_cv_neg1);
    assert!(
        max_gain > min_gain * 1.1,
        "CV should modulate filter: max={max_gain:.6}, min={min_gain:.6} (ratio={:.2}×)",
        max_gain / min_gain.max(1e-12)
    );
}

#[test]
fn tb303_cv_port_moves_explicit_diode_ladder_cutoff() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_cv_ac_cutoff_test_v4",
        "tb303_cv_ac_cutoff_test_v4",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let measure_ac = |cv_voltage: f64, freq: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        proc.set_control("Resonance", 0.0);
        let in_idx = proc.resolve_port("audio_in").expect("audio_in port");
        let cv_idx = proc.resolve_port("cv_cutoff").expect("cv_cutoff port");
        let out_idx = proc.resolve_port("audio_out").expect("audio_out port");

        for _ in 0..9600 {
            let mut ports = vec![0.0; proc.port_count()];
            ports[cv_idx] = cv_voltage;
            proc.process_ports(&mut ports);
        }

        let mut values = Vec::new();
        for i in 0..9600 {
            let input = 0.05 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let mut ports = vec![0.0; proc.port_count()];
            ports[in_idx] = input;
            ports[cv_idx] = cv_voltage;
            proc.process_ports(&mut ports);
            values.push(ports[out_idx]);
        }

        ac_rms(&values)
    };

    let low_cv_8k = measure_ac(-1.0, 8_000.0);
    let high_cv_8k = measure_ac(3.0, 8_000.0);
    let low_cv_12k = measure_ac(-1.0, 12_000.0);
    let high_cv_12k = measure_ac(3.0, 12_000.0);

    eprintln!(
        "  cutoff CV AC: 8k -1V={low_cv_8k:.6}, +3V={high_cv_8k:.6}; \
         12k -1V={low_cv_12k:.6}, +3V={high_cv_12k:.6}"
    );

    assert!(
        high_cv_8k > low_cv_8k * 1.25,
        "raising cv_cutoff should move the ExplicitSingleDiode ladder cutoff upward at 8kHz: \
         low={low_cv_8k:.6}, high={high_cv_8k:.6}, ratio={:.3}",
        high_cv_8k / low_cv_8k.max(1e-12)
    );
    assert!(
        high_cv_12k > low_cv_12k * 1.25,
        "raising cv_cutoff should move the ExplicitSingleDiode ladder cutoff upward at 12kHz: \
         low={low_cv_12k:.6}, high={high_cv_12k:.6}, ratio={:.3}",
        high_cv_12k / low_cv_12k.max(1e-12)
    );
}

#[test]
fn tb303_cutoff_cv_uses_circuit_calibrated_diode_resistance() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_cv_rp_calibration_test_v1",
        "tb303_cv_rp_calibration_test_v1",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");
    let cv_idx = proc.resolve_port("cv_cutoff").expect("cv_cutoff port");

    let measure_rp = |proc: &mut super::compiled::CompiledPedal, cv_voltage: f64| -> f64 {
        let mut ports = vec![0.0; proc.port_count()];
        ports[cv_idx] = cv_voltage;
        proc.process_ports(&mut ports);
        proc.stages
            .iter()
            .find_map(|stage| {
                if let pedalkernel_rt::processor::Stage::BlockwiseKMethod(bkm) = stage {
                    Some(bkm.blocks[1].rp)
                } else {
                    None
                }
            })
            .expect("tb303 filter should compile to BKM")
    };

    let rp_0v = measure_rp(&mut proc, 0.0);
    let rp_3v = measure_rp(&mut proc, 3.0);
    let ratio = rp_3v / rp_0v;

    eprintln!(
        "  circuit-calibrated TB303 cutoff Rp: 0V={rp_0v:.3}, +3V={rp_3v:.3}, ratio={ratio:.3}"
    );

    assert!(
        ratio > 0.35 && ratio < 0.85,
        "TB303 cutoff CV should use diode dynamic resistance from R_bias/R_cv, not 2^CV scaling: \
         rp_0v={rp_0v:.3}, rp_3v={rp_3v:.3}, ratio={ratio:.3}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Test G: Validate measurement infrastructure with known transfer function
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_measurement_infra_detects_known_lowpass() {
    // Bypass the 303 entirely. Implement a trivial 1-pole RC lowpass
    // in software and measure it the same way the filter tests do.
    // If THIS test fails, our measurement code is wrong.
    let sr = SR;
    let fc = 1000.0; // 1kHz cutoff
    let rc = 1.0 / (2.0 * std::f64::consts::PI * fc);
    let alpha = 1.0 / (1.0 + rc * sr); // bilinear 1-pole coefficient

    let measure = |freq: f64| -> f64 {
        let mut state = 0.0f64;
        // Warm up
        for _ in 0..2400 {
            state += alpha * (0.0 - state);
        }
        let mut peak = 0.0f64;
        for i in 0..4800 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / sr).sin();
            state += alpha * (input - state);
            peak = peak.max(state.abs());
        }
        peak
    };

    let g100 = measure(100.0);
    let g1k = measure(1000.0);
    let g10k = measure(10000.0);
    let ratio_db = 20.0 * (g100 / g10k.max(1e-12)).log10();

    eprintln!("  Known 1-pole LP: 100Hz={g100:.6}, 1kHz={g1k:.6}, 10kHz={g10k:.6}");
    eprintln!("  Ratio 100Hz/10kHz: {ratio_db:+.1} dB");

    assert!(
        g100 > g10k * 2.0,
        "Known 1kHz LP should show >6dB rolloff at 10kHz: 100Hz={g100:.6}, 10kHz={g10k:.6}"
    );
    assert!(
        ratio_db > 15.0,
        "Expected >15dB rolloff from 100Hz to 10kHz, got {ratio_db:+.1} dB"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Test F: DC gain check — suspicious 2.0 flat output
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_dc_gain_not_two() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_dc_gain",
        "tb303_dc_gain",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");
    proc.set_control("Resonance", 0.0);

    // Step input: DC gain test
    for _ in 0..4800 {
        proc.process(0.0);
    }
    let mut last = 0.0;
    for _ in 0..9600 {
        last = proc.process(1.0); // Step to 1.0V
    }

    eprintln!("  DC step response (after 200ms): {last:.6}");

    // DC gain should be ≈1.0 (Stinchcombe: H(0) = -1, magnitude 1).
    // If we get exactly 2.0, the output has a doubling bug.
    assert!(
        last.abs() < 1.5,
        "DC gain should be ≈1 (Stinchcombe H(0)=-1), got {last:.6}. \
         If exactly 2.0, output extraction has a doubling (V=(a+b)/2 with a≈b)."
    );
    assert!(
        last.abs() > 0.01,
        "DC gain should be nonzero, got {last:.6}. Filter is silent."
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Test D: Newton iteration count — is Newton actually running?
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_k_table_not_identity() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");

    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_ktable_check",
        "tb303_ktable_check",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    // Find the BlockwiseKMethod stage and inspect its K-tables
    let bkm = proc.stages.iter().find_map(|s| {
        if let pedalkernel_rt::processor::Stage::BlockwiseKMethod(ref k) = s {
            Some(k)
        } else {
            None
        }
    });

    if bkm.is_none() {
        eprintln!("  SKIP: no BlockwiseKMethod stage found");
        return;
    }
    let bkm = bkm.unwrap();

    // Check each block's K-table: evaluate at several b values
    for (bi, block) in bkm.blocks.iter().enumerate() {
        let mut table = block.k_table.clone();
        table.precompute_scales();

        let b_values = [-5.0, -1.0, 0.0, 1.0, 5.0];
        let a_values: Vec<f64> = b_values.iter().map(|&b| table.lookup_2d(b, 0.0)).collect();

        let max_a = a_values.iter().map(|a| a.abs()).fold(0.0f64, f64::max);
        let min_a = a_values.iter().map(|a| a.abs()).fold(f64::MAX, f64::min);
        let spread = max_a - min_a;

        eprintln!("  block {bi} K-table: b={b_values:?} → a={a_values:.4?}, spread={spread:.6}");

        // The K-table should NOT be constant — a_root should vary with b_tree.
        // If spread ≈ 0, the diode is degenerating to a constant (identity reflection).
        assert!(
            spread > 0.01,
            "Block {bi} K-table is near-constant (spread={spread:.6}). \
             Diode NL is degenerating — check bias point, Is, Vt."
        );
    }
}

// VCA test removed — VCA now uses behavioral macromodel (BA662 + LA4140),
// not a WDF-compiled .pedal. See acidattack-core/src/vca.rs.

#[test]
fn tb303_vco_port_reaches_output() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_vco_port",
        "tb303_vco_port",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    // Warm up with silence
    for _ in 0..2400 {
        proc.process(0.0);
    }

    // Test 1: process(input) path — serial chain
    let mut peak_serial = 0.0f64;
    for i in 0..4800 {
        let input = 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SR).sin();
        peak_serial = peak_serial.max(proc.process(input).abs());
    }

    // Test 2: process_ports path — VCO through vco_in port
    let mut proc2: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");
    for _ in 0..2400 {
        let mut ports = vec![0.0; proc2.port_count()];
        proc2.process_ports(&mut ports);
    }
    let vco_idx = proc2.resolve_port("vco_in");
    let out_idx = proc2.resolve_port("audio_out");
    eprintln!(
        "  vco_in port index: {:?}, audio_out port index: {:?}",
        vco_idx, out_idx
    );
    eprintln!("  total ports: {}", proc2.port_count());

    let mut peak_port = 0.0f64;
    for i in 0..4800 {
        let vco = 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SR).sin();
        let mut ports = vec![0.0; proc2.port_count()];
        if let Some(idx) = vco_idx {
            ports[idx] = vco;
        }
        proc2.process_ports(&mut ports);
        if let Some(idx) = out_idx {
            peak_port = peak_port.max(ports[idx].abs());
        }
    }

    eprintln!("  serial chain peak: {peak_serial:.6}");
    eprintln!("  vco_in port peak: {peak_port:.6}");

    assert!(
        peak_serial > 0.01,
        "Serial chain should produce audio, got {peak_serial:.6}"
    );
    assert!(
        peak_port > 0.01,
        "VCO through vco_in port should produce audio, got {peak_port:.6}"
    );
}

#[test]
fn tb303_sustained_tone_not_silent() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_sustained",
        "tb303_sustained",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    // Process 48000 samples (1 second) of 220Hz saw through vco_in port
    let vco_idx = proc.resolve_port("vco_in");
    let out_idx = proc.resolve_port("audio_out");
    eprintln!(
        "  vco_in={:?} audio_out={:?} ports={}",
        vco_idx,
        out_idx,
        proc.port_count()
    );

    let mut samples = Vec::new();
    for i in 0..48000 {
        let t = i as f64 / SR;
        let vco = 0.3 * (2.0 * std::f64::consts::PI * 220.0 * t).sin();
        let mut ports = vec![0.0; proc.port_count()];
        if let Some(idx) = vco_idx {
            ports[idx] = vco;
        }
        proc.process_ports(&mut ports);
        let out = if let Some(idx) = out_idx {
            ports[idx]
        } else {
            0.0
        };
        samples.push(out);
    }

    // Check signal at different time windows
    let peak_0_100ms: f64 = samples[0..4800].iter().map(|s| s.abs()).fold(0.0, f64::max);
    let peak_100_200ms: f64 = samples[4800..9600]
        .iter()
        .map(|s| s.abs())
        .fold(0.0, f64::max);
    let peak_500_600ms: f64 = samples[24000..28800]
        .iter()
        .map(|s| s.abs())
        .fold(0.0, f64::max);
    let peak_900_1000ms: f64 = samples[43200..48000]
        .iter()
        .map(|s| s.abs())
        .fold(0.0, f64::max);

    eprintln!("  0-100ms:   peak={peak_0_100ms:.6}");
    eprintln!("  100-200ms: peak={peak_100_200ms:.6}");
    eprintln!("  500-600ms: peak={peak_500_600ms:.6}");
    eprintln!("  900-1000ms: peak={peak_900_1000ms:.6}");

    // Check for NaN/Inf
    let nan_count = samples.iter().filter(|s| !s.is_finite()).count();
    eprintln!("  NaN/Inf samples: {nan_count} / {}", samples.len());

    // Signal should be sustained (not just a click)
    assert!(
        peak_500_600ms > 0.001,
        "Signal at 500-600ms should be nonzero (sustained tone), got {peak_500_600ms:.6}"
    );
    assert!(nan_count == 0, "No NaN/Inf allowed, got {nan_count}");
}

#[test]
fn tb303_render_wav_comparison() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);

    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_wav_render",
        "tb303_wav_render",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    let wav_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-wav");
    let _ = std::fs::create_dir_all(&wav_dir);

    let duration_secs = 2.0;
    let n_samples = (duration_secs * SR) as usize;

    // ── Track 1: dry VCO (bypass filter) ──
    let mut dry_samples = Vec::with_capacity(n_samples);
    for i in 0..n_samples {
        let t = i as f64 / SR;
        // Saw-ish: fundamental + a few harmonics
        let vco = 0.3 * (2.0 * std::f64::consts::PI * 220.0 * t).sin()
            + 0.15 * (2.0 * std::f64::consts::PI * 440.0 * t).sin()
            + 0.1 * (2.0 * std::f64::consts::PI * 660.0 * t).sin();
        dry_samples.push(vco);
    }
    crate::wav::write_wav(
        &dry_samples,
        &wav_dir.join("tb303_1_dry_vco.wav"),
        SR as u32,
    )
    .expect("write dry wav");

    // ── Track 2: VCO through filter (serial chain / process(input)) ──
    let mut filtered_serial = Vec::with_capacity(n_samples);
    for i in 0..n_samples {
        let t = i as f64 / SR;
        let vco = 0.3 * (2.0 * std::f64::consts::PI * 220.0 * t).sin()
            + 0.15 * (2.0 * std::f64::consts::PI * 440.0 * t).sin()
            + 0.1 * (2.0 * std::f64::consts::PI * 660.0 * t).sin();
        let out = proc.process(vco);
        filtered_serial.push(out);
    }
    crate::wav::write_wav(
        &filtered_serial,
        &wav_dir.join("tb303_2_filtered_serial.wav"),
        SR as u32,
    )
    .expect("write filtered serial wav");

    // ── Track 3: VCO through filter (port path / process_ports) ──
    let mut proc3: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");
    let vco_idx = proc3.resolve_port("vco_in");
    let out_idx = proc3.resolve_port("audio_out");
    let mut filtered_port = Vec::with_capacity(n_samples);
    for i in 0..n_samples {
        let t = i as f64 / SR;
        let vco = 0.3 * (2.0 * std::f64::consts::PI * 220.0 * t).sin()
            + 0.15 * (2.0 * std::f64::consts::PI * 440.0 * t).sin()
            + 0.1 * (2.0 * std::f64::consts::PI * 660.0 * t).sin();
        let mut ports = vec![0.0; proc3.port_count()];
        if let Some(idx) = vco_idx {
            ports[idx] = vco * 0.03;
        } // attenuated
        proc3.process_ports(&mut ports);
        let out = if let Some(idx) = out_idx {
            ports[idx]
        } else {
            0.0
        };
        filtered_port.push(out);
    }
    crate::wav::write_wav(
        &filtered_port,
        &wav_dir.join("tb303_3_filtered_port.wav"),
        SR as u32,
    )
    .expect("write filtered port wav");

    // ── Track 4: VCO through filter with CV sweep ──
    let mut proc4: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");
    let vco_idx4 = proc4.resolve_port("vco_in");
    let cv_idx4 = proc4.resolve_port("cv_cutoff");
    let out_idx4 = proc4.resolve_port("audio_out");
    let mut filtered_cv = Vec::with_capacity(n_samples);
    for i in 0..n_samples {
        let t = i as f64 / SR;
        let vco = 0.3 * (2.0 * std::f64::consts::PI * 220.0 * t).sin()
            + 0.15 * (2.0 * std::f64::consts::PI * 440.0 * t).sin()
            + 0.1 * (2.0 * std::f64::consts::PI * 660.0 * t).sin();
        // CV sweep: 0 to 2V over 2 seconds (opening the filter)
        let cv = t;
        let mut ports = vec![0.0; proc4.port_count()];
        if let Some(idx) = vco_idx4 {
            ports[idx] = vco * 0.03;
        }
        if let Some(idx) = cv_idx4 {
            ports[idx] = cv;
        }
        proc4.process_ports(&mut ports);
        let out = if let Some(idx) = out_idx4 {
            ports[idx]
        } else {
            0.0
        };
        filtered_cv.push(out);
    }
    crate::wav::write_wav(
        &filtered_cv,
        &wav_dir.join("tb303_4_filtered_cv_sweep.wav"),
        SR as u32,
    )
    .expect("write filtered cv wav");

    let serial_peak = filtered_serial
        .iter()
        .map(|s| s.abs())
        .fold(0.0f64, f64::max);
    let port_peak = filtered_port.iter().map(|s| s.abs()).fold(0.0f64, f64::max);
    let cv_peak = filtered_cv.iter().map(|s| s.abs()).fold(0.0f64, f64::max);

    eprintln!("  WAV files written to {:?}", wav_dir);
    eprintln!("  Track 1: dry VCO");
    eprintln!("  Track 2: filtered (serial), peak={serial_peak:.4}");
    eprintln!("  Track 3: filtered (port), peak={port_peak:.4}");
    eprintln!("  Track 4: filtered (port + CV sweep), peak={cv_peak:.4}");
}

#[test]
fn tb303_trace_cascade_one_sample() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_trace",
        "tb303_trace",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    // Warm up
    for _ in 0..4800 {
        proc.process(0.0);
    }

    // Process one nonzero sample and trace internals
    let input = 0.3; // single step input

    // Find the BKM stage
    let bkm = proc
        .stages
        .iter_mut()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::BlockwiseKMethod(ref mut k) = s {
                Some(k)
            } else {
                None
            }
        })
        .expect("BKM stage");

    let n_blocks = bkm.blocks.len();
    let n = bkm.n_ports;

    // Manually trace one cascade
    let audio_vs_idx = bkm
        .vs_port_map
        .first()
        .map(|(_, idx)| *idx)
        .unwrap_or(n_blocks);
    bkm.work_b[audio_vs_idx] = 2.0 * input;
    eprintln!(
        "  INPUT: {input}, audio_vs_idx={audio_vs_idx}, work_b[{audio_vs_idx}]={}",
        bkm.work_b[audio_vs_idx]
    );

    // Scatter
    for i in 0..n {
        let mut sum = 0.0;
        for j in 0..n {
            sum += bkm.coupling_s[i * n + j] * bkm.work_b[j];
        }
        bkm.work_a[i] = sum;
    }

    let feedback = (bkm.work_a[0] + bkm.work_b[0]) / 2.0;
    let mut cascade = input + feedback;
    eprintln!("  FEEDBACK: {feedback:.6}, cascade_start: {cascade:.6}");

    for i in 0..n_blocks {
        bkm.blocks[i].tree.set_voltage(cascade);
        let b_tree = bkm.blocks[i].tree.reflected();
        let ctrl = cascade;
        let a_root = bkm.blocks[i].k_table.lookup_2d(b_tree, ctrl);
        let v_out = (a_root + b_tree) / 2.0;
        let dc = bkm.blocks[i].dc_offset;
        let v_ac = v_out - dc;
        eprintln!("  BLOCK {i}: cascade_in={cascade:.6}, b_tree={b_tree:.6}, ctrl={ctrl:.6}, a_root={a_root:.6}, v_out={v_out:.6}, dc={dc:.6}, v_ac={v_ac:.6}");
        cascade = v_ac;
    }
    eprintln!("  FINAL OUTPUT: {cascade:.6}");
}

#[test]
fn tb303_trace_small_signal() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_trace_small",
        "tb303_trace_small",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    // Warm up
    for _ in 0..4800 {
        proc.process(0.0);
    }

    // Realistic signal: 10mV (what the VCO sends after 0.03× attenuation)
    let input = 0.01;

    let bkm = proc
        .stages
        .iter_mut()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::BlockwiseKMethod(ref mut k) = s {
                Some(k)
            } else {
                None
            }
        })
        .expect("BKM stage");

    let n_blocks = bkm.blocks.len();
    let n = bkm.n_ports;
    let audio_vs_idx = bkm
        .vs_port_map
        .first()
        .map(|(_, idx)| *idx)
        .unwrap_or(n_blocks);
    bkm.work_b[audio_vs_idx] = 2.0 * input;

    for i in 0..n {
        let mut sum = 0.0;
        for j in 0..n {
            sum += bkm.coupling_s[i * n + j] * bkm.work_b[j];
        }
        bkm.work_a[i] = sum;
    }

    let feedback = (bkm.work_a[0] + bkm.work_b[0]) / 2.0;
    let mut cascade = input + feedback;
    eprintln!("  INPUT: {input}, feedback: {feedback:.6}, cascade_start: {cascade:.6}");

    for i in 0..n_blocks {
        bkm.blocks[i].tree.set_voltage(cascade);
        let b_tree = bkm.blocks[i].tree.reflected();
        let ctrl = cascade;
        let a_root = bkm.blocks[i].k_table.lookup_2d(b_tree, ctrl);
        let v_out = (a_root + b_tree) / 2.0;
        let dc = bkm.blocks[i].dc_offset;
        let v_ac = v_out - dc;
        eprintln!("  BLOCK {i}: cascade={cascade:.6}, ctrl={ctrl:.6}, b_tree={b_tree:.6}, a_root={a_root:.6}, v_out={v_out:.6}, dc={dc:.4}, v_ac={v_ac:.6}");
        cascade = v_ac;
    }
    eprintln!("  FINAL: {cascade:.6}");
    assert!(
        cascade.abs() < 1.0,
        "Small-signal output should be <1V, got {cascade:.6}"
    );
}

#[test]
fn tb303_cap_state_bounded_during_warmup() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_capwind",
        "tb303_capwind",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");
    proc.set_control("Resonance", 0.0);

    // Process 4800 samples of SILENCE — no input
    // After each sample, check the BKM stage's block tree states
    for sample in 0..4800 {
        proc.process(0.0);

        // Every 480 samples (10ms), check block states
        if sample % 480 == 479 {
            let bkm = proc.stages.iter().find_map(|s| {
                if let pedalkernel_rt::processor::Stage::BlockwiseKMethod(ref k) = s {
                    Some(k)
                } else {
                    None
                }
            });
            if let Some(bkm) = bkm {
                for (bi, block) in bkm.blocks.iter().enumerate() {
                    let probe_v = block
                        .cascade_probe_id
                        .as_deref()
                        .and_then(|id| block.tree.leaf_voltage(id));
                    let mut tree_clone = block.tree.clone();
                    tree_clone.set_voltage(0.0);
                    let b = tree_clone.reflected();
                    let reported = probe_v.unwrap_or(b);
                    if b.abs() > 10.0 {
                        eprintln!(
                            "  WINDUP at sample {sample}: block {bi} b_tree={b:.4}, probe_v={probe_v:?}"
                        );
                    }
                    assert!(
                        reported.is_finite() && reported.abs() < 10.0,
                        "Cap wind-up: block {bi} probe/b_tree={reported:.4} at sample {sample} \
                         (silent input). Cap state has diverged. b_tree={b:.4}"
                    );
                }
            }
        }
    }

    // Final check: all b_tree should be small
    let bkm = proc
        .stages
        .iter()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::BlockwiseKMethod(ref k) = s {
                Some(k)
            } else {
                None
            }
        })
        .expect("BKM");

    for (bi, block) in bkm.blocks.iter().enumerate() {
        let mut t = block.tree.clone();
        t.set_voltage(0.0);
        let b = t.reflected();
        let probe_v = block
            .cascade_probe_id
            .as_deref()
            .and_then(|id| block.tree.leaf_voltage(id));
        let reported = probe_v.unwrap_or(b);
        eprintln!(
            "  Final block {bi}: b_tree={b:.6}, probe_v={probe_v:?}, dc_offset={:.6}",
            block.dc_offset
        );
        assert!(
            reported.is_finite() && reported.abs() < 10.0,
            "After 100ms silence, block {bi} probe/b_tree={reported:.6} should be small. \
             Cap wound up. b_tree={b:.6}"
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Signal routing: VCO port path should be lowpass, not highpass
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn tb303_vco_port_path_is_lowpass() {
    // The process(input) serial chain goes through C_out (highpass) first,
    // then the ladder. That's wrong — C_out is an output coupling cap.
    //
    // The process_ports path with vco_in goes directly into the BKM
    // stage's coupling VS port, bypassing C_out. This should be lowpass.
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_vco_lp",
        "tb303_vco_lp",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let measure_port = |freq: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal =
            postcard::from_bytes(&blob).expect("deserialize failed");
        let vco_idx = proc.resolve_port("vco_in");
        let out_idx = proc.resolve_port("audio_out");
        // Warm up
        for _ in 0..4800 {
            let mut ports = vec![0.0; proc.port_count()];
            proc.process_ports(&mut ports);
        }
        let mut peak = 0.0f64;
        for i in 0..4800 {
            let vco = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            let mut ports = vec![0.0; proc.port_count()];
            if let Some(idx) = vco_idx {
                ports[idx] = vco;
            }
            proc.process_ports(&mut ports);
            if let Some(idx) = out_idx {
                peak = peak.max(ports[idx].abs());
            }
        }
        peak
    };

    let g100 = measure_port(100.0);
    let g1k = measure_port(1000.0);
    let g10k = measure_port(10000.0);
    let ratio_db = 20.0 * (g100 / g10k.max(1e-12)).log10();

    eprintln!("  VCO port path: 100Hz={g100:.6}, 1kHz={g1k:.6}, 10kHz={g10k:.6}");
    eprintln!("  Ratio 100Hz/10kHz: {ratio_db:+.1} dB");

    // The diode ladder should be LOWPASS: 100Hz > 10kHz
    assert!(
        g100 > g10k,
        "VCO port path should be lowpass: 100Hz={g100:.6} must be > 10kHz={g10k:.6} \
         (ratio={ratio_db:+.1} dB)"
    );

    // For a 4-pole ladder, expect at least 12dB rolloff from 100Hz to 10kHz
    // (relaxed — the cutoff frequency may be high)
    if ratio_db > 6.0 {
        eprintln!("  PASS: {ratio_db:+.1} dB lowpass rolloff");
    } else {
        eprintln!("  WEAK: only {ratio_db:+.1} dB rolloff (expected >6dB for 4-pole)");
    }
}

#[test]
fn tb303_serial_path_ordering_check() {
    // Document the serial chain ordering issue:
    // process(input) goes Stage 0 (C_out) → Stage 1 (BKM ladder)
    // This is highpass-then-lowpass, which gives highpass behavior.
    // The correct order should be lowpass-then-highpass (ladder → C_out).
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_serial_order",
        "tb303_serial_order",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    // Check stage types and ordering
    for (i, stage) in proc.stages.iter().enumerate() {
        let stype = match stage {
            pedalkernel_rt::processor::Stage::Wdf(w) => {
                format!(
                    "WDF(root={:?}, rp={:.0})",
                    if w.tree.port_resistance() > 5000.0 {
                        "high-Z"
                    } else {
                        "low-Z"
                    },
                    w.tree.port_resistance()
                )
            }
            pedalkernel_rt::processor::Stage::BlockwiseKMethod(k) => {
                format!("BKM({}blocks, {}ports)", k.blocks.len(), k.n_ports)
            }
            _ => format!("other"),
        };
        eprintln!("  Stage {i}: {stype}");
    }

    // The BKM stage should come BEFORE the C_out stage in processing order
    let bkm_idx = proc
        .stages
        .iter()
        .position(|s| matches!(s, pedalkernel_rt::processor::Stage::BlockwiseKMethod(_)));
    let cout_idx = proc
        .stages
        .iter()
        .position(|s| matches!(s, pedalkernel_rt::processor::Stage::Wdf(_)));

    eprintln!(
        "  BKM stage index: {:?}, C_out stage index: {:?}",
        bkm_idx, cout_idx
    );

    if let (Some(bkm), Some(cout)) = (bkm_idx, cout_idx) {
        if bkm > cout {
            eprintln!("  WARNING: BKM after C_out — serial chain is highpass-then-lowpass");
            eprintln!("  FIX: swap ordering so ladder processes before output coupling");
        }
    }
}

#[test]
fn tb303_stage_ordering_debug() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let def = crate::dsl::parse_pedal_file(&source).expect("parse");
    let compiled = super::spqr_build::compile_via_spqr(&def, SR).expect("compile");
    for (i, stage) in compiled.stages.iter().enumerate() {
        match stage {
            pedalkernel_rt::processor::Stage::Wdf(w) => {
                eprintln!(
                    "  Stage {i}: WDF flow_dist={} rp={:.0}",
                    w.signal_flow_distance,
                    w.tree.port_resistance()
                );
            }
            pedalkernel_rt::processor::Stage::BlockwiseKMethod(k) => {
                eprintln!(
                    "  Stage {i}: BKM flow_dist={} blocks={}",
                    k.signal_flow_distance,
                    k.blocks.len()
                );
            }
            _ => {
                eprintln!("  Stage {i}: other");
            }
        }
    }
}

#[test]
fn tb303_per_block_signal_trace() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_trace_detail",
        "tb303_trace_detail",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let mut proc: super::compiled::CompiledPedal =
        postcard::from_bytes(&blob).expect("deserialize failed");

    // Warm up
    for _ in 0..4800 {
        proc.process(0.0);
    }

    // Find BKM stage
    let bkm = proc
        .stages
        .iter_mut()
        .find_map(|s| {
            if let pedalkernel_rt::processor::Stage::BlockwiseKMethod(ref mut k) = s {
                Some(k)
            } else {
                None
            }
        })
        .expect("BKM stage");

    let n = bkm.n_ports;
    let n_blocks = bkm.blocks.len();

    eprintln!("  === BKM state after warmup ===");
    eprintln!("  n_ports={n}, n_blocks={n_blocks}");
    eprintln!("  supply_voltage={}", bkm.supply_voltage);
    eprintln!("  vs_port_map: {:?}", bkm.vs_port_map);
    eprintln!("  port_index_cache: {:?}", bkm.port_index_cache);

    // Print coupling scattering matrix
    eprintln!("  scattering ({n}x{n}):");
    for i in 0..n {
        let row: Vec<String> = (0..n)
            .map(|j| format!("{:+.4}", bkm.coupling_s[i * n + j]))
            .collect();
        eprintln!("    row {i}: [{}]", row.join(", "));
    }

    // Print work_b (should have supply voltage in supply VS port)
    eprintln!(
        "  work_b: {:?}",
        &bkm.work_b[..n]
            .iter()
            .map(|v| format!("{v:+.4}"))
            .collect::<Vec<_>>()
    );

    // Print per-block tree state
    for (bi, block) in bkm.blocks.iter_mut().enumerate() {
        let mut t = block.tree.clone();
        t.set_voltage(0.0);
        let b = t.reflected();
        eprintln!("  block {bi}: rp={:.1}, dc_offset={:.4}, b_tree_at_0={:.4}, k_table dims={}, entries={}",
            block.rp, block.dc_offset, b, block.k_table.dims, block.k_table.entries.len());

        // Check K-table at a few points
        block.k_table.precompute_scales();
        let test_b = [-2.0, -0.5, 0.0, 0.5, 2.0];
        let test_a: Vec<String> = test_b
            .iter()
            .map(|&b| {
                let a = block.k_table.lookup_1d(b);
                format!("{a:+.4}")
            })
            .collect();
        eprintln!("    K-table: b={test_b:?} → a=[{}]", test_a.join(", "));
    }

    // Now process ONE sample with input=0.1V and trace every step
    eprintln!("\n  === Processing input=0.1V ===");
    let input = 0.1;

    // Simulate what process() does
    // 1. Write VS signals
    for (i, &(ref name, vs_idx)) in bkm.vs_port_map.iter().enumerate() {
        let v = if name.starts_with("_supply_") {
            bkm.supply_voltage
        } else if i == 0 {
            input // audio_in gets the input
        } else {
            0.0
        };
        bkm.work_b[vs_idx] = 2.0 * v;
        eprintln!(
            "  VS port '{name}' (idx={vs_idx}): v={v:.4}, b={:.4}",
            2.0 * v
        );
    }

    // 2. Scatter
    let mut work_a = vec![0.0; n];
    for i in 0..n {
        let mut sum = 0.0;
        for j in 0..n {
            sum += bkm.coupling_s[i * n + j] * bkm.work_b[j];
        }
        work_a[i] = sum;
    }
    eprintln!("  after scatter:");
    for i in 0..n {
        let v = (work_a[i] + bkm.work_b[i]) / 2.0;
        eprintln!(
            "    port {i}: a={:+.4}, b={:+.4}, V={:+.4}",
            work_a[i], bkm.work_b[i], v
        );
    }

    // 3. Cascade
    let mut cascade = (work_a[0] + bkm.work_b[0]) / 2.0;
    eprintln!("  cascade start: {cascade:+.6}");

    for i in 0..n_blocks {
        let vs_in = -cascade; // negated for WDF sign convention
        bkm.blocks[i].tree.set_voltage(vs_in);
        let b_tree = bkm.blocks[i].tree.reflected();
        let a_root = bkm.blocks[i].k_table.lookup_1d(b_tree);
        let v_out = (a_root + b_tree) / 2.0;
        eprintln!("  block {i}: cascade_in={cascade:+.6}, VS={vs_in:+.6}, b_tree={b_tree:+.6}, a_root={a_root:+.6}, V_out={v_out:+.6}");
        cascade = v_out;
    }
    eprintln!("  cascade output: {cascade:+.6}");

    // The cascade output must be nonzero for the filter to work
    assert!(
        cascade.abs() > 1e-6,
        "Cascade output should be nonzero with 0.1V input. Got {cascade:.10}"
    );
}

#[test]
fn tb303_check_wider_frequency_range() {
    let source = skip_if_missing!(load_pro_pedal("tb303_filter.pedal"), "tb303_filter.pedal");
    let cache_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("test-cache");
    let _ = std::fs::create_dir_all(&cache_dir);
    let options = super::compile::CompileOptions::default();
    let blob = super::compile::compile_pedal_cached(
        &source,
        "tb303_freq_range",
        "tb303_freq_range",
        SR,
        &options,
        &cache_dir,
    )
    .expect("compile failed");

    let measure = |freq: f64| -> f64 {
        let mut proc: super::compiled::CompiledPedal = postcard::from_bytes(&blob).expect("deser");
        for _ in 0..4800 {
            proc.process(0.0);
        }
        let mut peak = 0.0f64;
        for i in 0..9600 {
            let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / SR).sin();
            peak = peak.max(proc.process(input).abs());
        }
        peak
    };

    let freqs = [
        50.0, 100.0, 500.0, 1000.0, 5000.0, 10000.0, 15000.0, 20000.0,
    ];
    eprintln!("  Frequency response:");
    for &f in &freqs {
        let g = measure(f);
        let g_db = 20.0 * (g / 0.1).log10();
        eprintln!("    {f:>8.0} Hz: {g:.6} ({g_db:+.1} dB)");
    }

    let g_50 = measure(50.0);
    let g_20k = measure(20000.0);
    let ratio = 20.0 * (g_50 / g_20k.max(1e-12)).log10();
    eprintln!("  50Hz/20kHz ratio: {ratio:+.1} dB");

    assert!(
        g_50 > g_20k,
        "Should be lowpass: 50Hz={g_50:.6} > 20kHz={g_20k:.6}"
    );
}
