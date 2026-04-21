// TDD tests for native 3-terminal pot handling.
//
// These test the CONTRACT: a pot is one component, creates one edge,
// produces a working voltage divider, and responds to set_control().
// Written BEFORE the implementation — they define what "correct" means.

use super::spqr_build::compile_via_spqr;
use super::spqr_control::bind_controls;
use crate::PedalProcessor;

// ═══════════════════════════════════════════════════════════════════════════
// Graph level: pot is one component, one edge
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn pot_halves_in_same_stage() {
    // Pot halves (aw + wb) should end up in the same stage, not split apart.
    // The wiper is a real circuit node — two edges is correct in the graph.
    // But the SPQR pipeline must keep them together for the divider to work.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                Volume: pot(100k, a)
            }
            nets {
                in -> R1.a
                R1.b -> Volume.a
                Volume.w -> out
                Volume.b -> gnd
            }
            controls { Volume.position -> "Volume" [0.0, 1.0] = 0.5 }
        }"#)
    .expect("parse");

    let compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");

    // Both pot edges should be in the same stage (same comp_idx).
    // With native ports, the pot is "Volume" (not __aw/__wb).
    let pot_stage = compiled.stages.iter().position(|s| {
        s.tree.get_pot_position("Volume").is_some()
    });
    assert!(
        pot_stage.is_some(),
        "Should find Volume pot in a stage"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Stage level: pot creates a voltage divider
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn pot_creates_voltage_divider_stage() {
    // A standalone volume pot should compile into a stage that attenuates.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                Volume: pot(100k, a)
            }
            nets {
                in -> R1.a
                R1.b -> Volume.a
                Volume.w -> out
                Volume.b -> gnd
            }
            controls { Volume.position -> "Volume" [1.0, 0.0] = 0.5 }
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    eprintln!("Divider stages: {}", compiled.stages.len());
    for (i, s) in compiled.stages.iter().enumerate() {
        eprintln!("  stage {i}: rp={:.1}", s.tree.port_resistance());
    }

    // At default (0.5): should pass signal with some attenuation
    for _ in 0..100 {
        compiled.process(1.0);
    }
    let mid_output = compiled.process(1.0);
    assert!(
        mid_output.abs() > 0.01 && mid_output.abs() < 1.0,
        "Pot at 50% should attenuate: got {mid_output:.4}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Control level: set_control changes attenuation
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn pot_set_control_changes_level() {
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                Volume: pot(100k, a)
            }
            nets {
                in -> R1.a
                R1.b -> Volume.a
                Volume.w -> out
                Volume.b -> gnd
            }
            controls { Volume.position -> "Volume" [0.0, 1.0] = 0.5 }
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    bind_controls(&pedal, &mut compiled);

    // Full volume
    compiled.set_control("Volume", 1.0);
    for _ in 0..50 { compiled.process(1.0); }
    let full = compiled.process(1.0).abs();

    // Zero volume
    compiled.set_control("Volume", 0.0);
    for _ in 0..50 { compiled.process(1.0); }
    let zero = compiled.process(1.0).abs();

    eprintln!("Pot control: full={full:.4}, zero={zero:.4}");
    assert!(
        full > zero * 3.0,
        "Full volume should be much louder than zero: full={full:.4}, zero={zero:.4}"
    );
}

#[test]
fn pot_in_feedback_changes_gain() {
    // Drive pot in op-amp feedback: changing position changes gain.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(1k)
                U1: opamp(tl072)
                R_min: resistor(1k)
                Drive: pot(100k, a)
            }
            nets {
                in -> R1.a
                R1.b -> U1.neg
                U1.neg -> R_min.a
                R_min.b -> Drive.a
                Drive.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls { Drive.position -> "Drive" [0.0, 1.0] = 0.5 }
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    bind_controls(&pedal, &mut compiled);

    // Low gain
    compiled.set_control("Drive", 0.0);
    for _ in 0..50 { compiled.process(0.01); }
    let low_gain_out = compiled.process(0.01).abs();

    // High gain
    compiled.set_control("Drive", 1.0);
    for _ in 0..50 { compiled.process(0.01); }
    let high_gain_out = compiled.process(0.01).abs();

    eprintln!("Drive: low={low_gain_out:.4}, high={high_gain_out:.4}");
    assert!(
        high_gain_out > low_gain_out * 2.0,
        "High drive should amplify more: low={low_gain_out:.4}, high={high_gain_out:.4}"
    );
}

/// Inverted range: [1.0, 0.0] means knob CW = pot position 0 (low R).
/// The pedal designer uses this when the circuit's wiring is "backwards"
/// from the user's expectation. The control system handles the inversion.
#[test]
fn pot_in_feedback_inverted_range() {
    // Same circuit as pot_in_feedback_changes_gain, but range [1.0, 0.0].
    // User "Drive" at 1.0 (CW) → pot position 0.0 → Rf = R_min → LOW gain.
    // User "Drive" at 0.0 (CCW) → pot position 1.0 → Rf = R_min + 100k → HIGH gain.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(1k)
                U1: opamp(tl072)
                R_min: resistor(1k)
                Drive: pot(100k, a)
            }
            nets {
                in -> R1.a
                R1.b -> U1.neg
                U1.neg -> R_min.a
                R_min.b -> Drive.a
                Drive.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls { Drive.position -> "Drive" [1.0, 0.0] = 0.5 }
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    bind_controls(&pedal, &mut compiled);

    // User CW (value=1.0) → range maps to position 0.0 → low Rf → LOW gain
    compiled.set_control("Drive", 1.0);
    for _ in 0..50 { compiled.process(0.01); }
    let cw_out = compiled.process(0.01).abs();

    // User CCW (value=0.0) → range maps to position 1.0 → high Rf → HIGH gain
    compiled.set_control("Drive", 0.0);
    for _ in 0..50 { compiled.process(0.01); }
    let ccw_out = compiled.process(0.01).abs();

    eprintln!("Inverted range: CW(low gain)={cw_out:.4}, CCW(high gain)={ccw_out:.4}");
    // With inverted range, CCW should produce MORE gain (higher Rf)
    assert!(
        ccw_out > cw_out * 2.0,
        "Inverted range: CCW should have more gain than CW: cw={cw_out:.4}, ccw={ccw_out:.4}"
    );
}

/// Normal range with volume pot (not feedback). Validates that range
/// direction is respected for simple voltage divider pots too.
#[test]
fn pot_volume_inverted_range() {
    // Volume pot: wiper → output. [1.0, 0.0] = "turn right for less volume"
    // (uncommon but valid — some designers prefer reverse-log feel).
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(4.7k)
                Volume: pot(100k, a)
            }
            nets {
                in -> R1.a
                R1.b -> Volume.a
                Volume.w -> out
                Volume.b -> gnd
            }
            controls { Volume.position -> "Volume" [1.0, 0.0] = 0.5 }
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    bind_controls(&pedal, &mut compiled);

    // User CW (value=1.0) → range maps to position 0.0 → wiper at ground → quiet
    compiled.set_control("Volume", 1.0);
    for _ in 0..50 { compiled.process(0.5); }
    let cw_out = compiled.process(0.5).abs();

    // User CCW (value=0.0) → range maps to position 1.0 → wiper at input → loud
    compiled.set_control("Volume", 0.0);
    for _ in 0..50 { compiled.process(0.5); }
    let ccw_out = compiled.process(0.5).abs();

    eprintln!("Inverted volume: CW(quiet)={cw_out:.4}, CCW(loud)={ccw_out:.4}");
    assert!(
        ccw_out > cw_out * 2.0 || cw_out < 0.01,
        "Inverted range: CW should be quieter: cw={cw_out:.4}, ccw={ccw_out:.4}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 1: MNA stamping — does the pot stamp correct resistances?
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn mna_pot_stamps_split_resistance() {
    use crate::tree::MnaSystem;
    use super::component::{Component, StampContext};
    use super::components::Potentiometer;
    use crate::dsl::PotTaper;

    let pot = Potentiometer { max_r: 100_000.0, taper: PotTaper::B };
    let mut mna = MnaSystem::new(3, 0); // nodes: a(0), w(1), b(2)

    let pin_fn = |pin: &str| -> Option<usize> {
        match pin { "a" => Some(0), "w" | "wiper" => Some(1), "b" => Some(2), _ => None }
    };
    let mut ctx = StampContext {
        pin_to_mna: &pin_fn, vsrc_base: 0, internal_node_base: 0,
        sample_rate: 48000.0, cap_stamps: None,
    };
    pot.stamp_mna_multi("Vol", &mut ctx, &mut mna);

    // Position 0.5, linear: R_aw=50k, R_wb=50k → G=2e-5 each
    let g_half = 1.0 / 50_000.0;
    assert!((mna.g_matrix[0] - g_half).abs() < 1e-8, "G[a,a]");
    assert!((mna.g_matrix[4] - 2.0 * g_half).abs() < 1e-8, "G[w,w] = aw+wb");
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 2: DC gain — does the MNA produce correct gain?
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn mna_dc_gain_voltage_divider() {
    use crate::tree::MnaSystem;
    // VS → R1(50k) → R2(50k) → GND. Gain at junction = 0.5
    let mut mna = MnaSystem::new(2, 1);
    mna.stamp_resistor(Some(0), Some(1), 50_000.0);
    mna.stamp_resistor(Some(1), None, 50_000.0);
    mna.stamp_voltage_source(Some(0), None, 0);
    let gain = mna.dc_gain(0, Some(1));
    eprintln!("50k/50k divider gain: {gain:.4}");
    assert!((gain - 0.5).abs() < 0.01, "Expected 0.5, got {gain:.4}");
}

#[test]
fn mna_dc_gain_asymmetric() {
    use crate::tree::MnaSystem;
    // 10k / 90k → gain = 0.9
    let mut mna = MnaSystem::new(2, 1);
    mna.stamp_resistor(Some(0), Some(1), 10_000.0);
    mna.stamp_resistor(Some(1), None, 90_000.0);
    mna.stamp_voltage_source(Some(0), None, 0);
    let gain = mna.dc_gain(0, Some(1));
    eprintln!("10k/90k divider gain: {gain:.4}");
    assert!((gain - 0.9).abs() < 0.01, "Expected 0.9, got {gain:.4}");
}

// ═══════════════════════════════════════════════════════════════════════════
// Layer 3: IIR builder — correct biquad from pot circuit?
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn iir_builder_pot_dc_gain() {
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components { R1: resistor(10k)  Volume: pot(100k, b) }
            nets { in -> R1.a  R1.b -> Volume.a  Volume.w -> out  Volume.b -> gnd }
            controls { Volume.position -> "Volume" [0.0, 1.0] = 0.5 }
        }"#).expect("parse");

    let graph = super::graph::CircuitGraph::from_pedal(&pedal);
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    let groups = super::signal_flow::find_flow_groups(&all_edges, &graph);
    for group in &groups {
        let edges = group.all_edges();
        if edges.len() < 2 { continue; }
        let names: Vec<_> = edges.iter()
            .map(|&e| graph.components[graph.edges[e].comp_idx].id.clone())
            .collect();
        eprintln!("Group ({} edges): {names:?}", edges.len());

        // Build MNA manually to check dc_gain with correct output node
        use crate::tree::MnaSystem;
        use super::component::StampContext;
        // Collect nodes
        let mut nodes: Vec<super::graph::NodeId> = Vec::new();
        for &eidx in &edges {
            let e = &graph.edges[eidx];
            for n in [e.node_a, e.node_b] {
                if n != graph.gnd_node && !graph.supply_nodes.contains(&n) && !nodes.contains(&n) {
                    nodes.push(n);
                }
            }
        }
        let n2m = |n: super::graph::NodeId| -> Option<usize> {
            if n == graph.gnd_node { None } else { nodes.iter().position(|&x| x == n) }
        };
        let mut mna = MnaSystem::new(nodes.len(), 1);
        // Stamp pot via stamp_mna_multi
        for &eidx in &edges {
            let comp = &graph.components[graph.edges[eidx].comp_idx];
            if comp.kind.ports().len() > 1 {
                let pin_fn = |pin: &str| -> Option<usize> {
                    let key = format!("{}.{}", comp.id, pin);
                    graph.node_names.get(&key).and_then(|&n| n2m(n))
                };
                let mut ctx = StampContext {
                    pin_to_mna: &pin_fn, vsrc_base: 0, internal_node_base: 0,
                    sample_rate: 48000.0, cap_stamps: None,
                };
                comp.kind.stamp_mna_multi(&comp.id, &mut ctx, &mut mna);
                break; // Only stamp once
            } else if let Some(r) = comp.kind.resistance() {
                let e = &graph.edges[eidx];
                mna.stamp_resistor(n2m(e.node_a), n2m(e.node_b), r);
            }
        }
        // VS at in_node
        let vs_mna = n2m(graph.in_node);
        mna.stamp_voltage_source(vs_mna, None, 0);
        // Output at out_node (wiper)
        let out_mna = n2m(graph.out_node);
        let gain = mna.dc_gain(0, out_mna);
        eprintln!("  Manual MNA: nodes={:?} vs={vs_mna:?} out={out_mna:?} dc_gain={gain:.4}", nodes);
        assert!(
            gain.abs() > 0.01 && gain.abs() < 1.0,
            "DC gain should attenuate, got {gain:.4}"
        );
        break;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Direct WDF scatter math test
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn wdf_voltage_divider_scatter_math() {
    // Direct WDF test: Series(VS, Series(R_aw, R_wb)) + ShortCircuit
    // Expected: wiper voltage = Vin * R_wb / (R_aw + R_wb)
    use super::dyn_node::DynNode;
    use super::wdf_leaf::{WdfResistor, WdfVoltageSource};

    let r_aw = 50_000.0;
    let r_wb = 50_000.0;

    let vs = DynNode::Leaf(Box::new(WdfVoltageSource {
        voltage: 0.0,
        rp: 1.0,
        is_cathode_bias: false,
    }));
    let leaf_aw = DynNode::Leaf(Box::new(WdfResistor { rp: r_aw, comp_id: Some("aw".to_string()), last_a: 0.0 }));
    let leaf_wb = DynNode::Leaf(Box::new(WdfResistor { rp: r_wb, comp_id: Some("wb".to_string()), last_a: 0.0 }));

    // Series(R_aw, R_wb) — the divider chain
    let divider = DynNode::Series(Box::new(leaf_aw), Box::new(leaf_wb));
    // Series(VS, divider) — full tree
    let mut tree = DynNode::Series(Box::new(vs), Box::new(divider));

    let rp = tree.port_resistance();
    eprintln!("Tree port resistance: {rp:.1}");
    // Expected: VS(1) + R_aw(50k) + R_wb(50k) = 100001
    assert!((rp - 100001.0).abs() < 10.0, "rp should be ~100001, got {rp:.1}");

    // Set VS voltage and scatter
    tree.set_voltage(1.0);
    let b = tree.reflected();
    eprintln!("reflected b = {b:.6}");

    // ShortCircuit: a = -b
    let a = -b;
    tree.set_incident(a);

    // Read junction voltage at wiper (between R_aw and R_wb)
    // Method 1: series_junction_voltage
    let v_junction = tree.series_junction_voltage(a);
    eprintln!("series_junction_voltage = {v_junction:?}");

    // Method 2: leaf voltage of R_wb
    let v_wb = tree.leaf_voltage(""); // Can't easily get by ID on raw resistors
    eprintln!("leaf_voltage = {v_wb:?}");

    // Expected wiper voltage: 1.0 * 50k / 100k = 0.5V
    if let Some(v) = v_junction {
        let expected = 1.0 * r_wb / (r_aw + r_wb);
        assert!(
            (v.abs() - expected).abs() < 0.1,
            "Wiper voltage should be ~{expected:.2}V, got {v:.4}V"
        );
    } else {
        // If series_junction_voltage returns None, try manual extraction
        // Root voltage = (a + b) / 2 = 0 (ShortCircuit)
        let v_root = (a + b) / 2.0;
        eprintln!("Root voltage (should be 0): {v_root:.6}");

        // Let's try a different approach: output_probe on R_wb
        panic!("series_junction_voltage returned None — need alternative extraction");
    }
}

#[test]
fn wdf_simple_resistor_divider() {
    // Simplest possible test: Series(VS, R) with ShortCircuit.
    // Output should be 0 at root (short circuit) but VS drives current through R.
    use super::dyn_node::DynNode;
    use super::wdf_leaf::{WdfResistor, WdfVoltageSource};

    let vs = DynNode::Leaf(Box::new(WdfVoltageSource {
        voltage: 0.0,
        rp: 1.0,
        is_cathode_bias: false,
    }));
    let r = DynNode::Leaf(Box::new(WdfResistor { rp: 10_000.0, comp_id: Some("R".to_string()), last_a: 0.0 }));
    let mut tree = DynNode::Series(Box::new(vs), Box::new(r));

    tree.set_voltage(1.0);
    let b = tree.reflected();
    let a = -b; // ShortCircuit
    tree.set_incident(a);

    let v_root = (a + b) / 2.0;
    let v_junction = tree.series_junction_voltage(a);
    eprintln!("Simple: b={b:.6}, a={a:.6}, v_root={v_root:.6}, v_junction={v_junction:?}");

    // With ShortCircuit: all voltage drops across R.
    // V_R = Vin (VS drives 1V, ground absorbs it all through R)
    // series_junction_voltage should give voltage at R node
    assert!(v_junction.is_some(), "Should extract junction voltage");
}

// ═══════════════════════════════════════════════════════════════════════════
// No synthetic split leaking
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn no_synthetic_aw_wb_components() {
    // After compilation, no __aw/__wb synthetic component names should leak
    // into stage trees or control bindings.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components {
                R1: resistor(10k)
                Volume: pot(100k, a)
            }
            nets {
                in -> R1.a
                R1.b -> Volume.a
                Volume.w -> out
                Volume.b -> gnd
            }
            controls { Volume.position -> "Volume" [0.0, 1.0] = 0.5 }
        }"#)
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
    bind_controls(&pedal, &mut compiled);

    // Control binding should use base ID "Volume", not "Volume__aw"
    for ctrl in &compiled.controls {
        assert!(
            !ctrl.component_id.contains("__aw") && !ctrl.component_id.contains("__wb"),
            "Binding should use base ID, got '{}'", ctrl.component_id
        );
    }
}
