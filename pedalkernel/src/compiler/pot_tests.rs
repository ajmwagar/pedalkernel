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
fn pot_is_single_edge_in_graph() {
    // A 3-terminal pot should produce ONE graph edge, not two synthetic ones.
    let pedal = crate::dsl::parse_pedal_file(r#"
        pedal "test" { supply 9V
            components { Volume: pot(100k, a) }
            nets { in -> Volume.a  Volume.w -> out  Volume.b -> gnd }
            controls { Volume.position -> "Volume" [1.0, 0.0] = 0.5 }
        }"#)
    .expect("parse");

    let graph = super::graph::CircuitGraph::from_pedal(&pedal);
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let signal_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();

    // Should have exactly 1 edge for the pot (not 2 synthetic __aw/__wb)
    let pot_edges: Vec<_> = signal_edges
        .iter()
        .filter(|&&eidx| {
            let comp = &graph.components[graph.edges[eidx].comp_idx];
            comp.id == "Volume" || comp.id.starts_with("Volume__")
        })
        .collect();

    assert_eq!(
        pot_edges.len(), 1,
        "3-terminal pot should be 1 edge, not {} (IDs: {:?})",
        pot_edges.len(),
        pot_edges.iter().map(|&&eidx| &graph.components[graph.edges[eidx].comp_idx].id).collect::<Vec<_>>()
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
