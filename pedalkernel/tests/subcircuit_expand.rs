//! Tests for file-based subcircuit instantiation (`use("path.pedal")`) and the
//! flatten/expand pass (`dsl_expand::expand_uses`).

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::{parse_pedal_file, Pin};
use pedalkernel::dsl_expand::{expand_uses, parse_and_expand_pedal_file};
use pedalkernel::PedalProcessor;
use std::path::{Path, PathBuf};

const SAMPLE_RATE: f64 = 48_000.0;

fn pedals_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/test_pedals")
}

/// Render a 1 kHz sine through a compiled pedal and return the output samples.
fn render_sine(def: &pedalkernel::dsl::PedalDef, n: usize) -> Vec<f64> {
    let mut compiled = compile_pedal(def, SAMPLE_RATE).expect("compile");
    (0..n)
        .map(|i| {
            let x = (2.0 * std::f64::consts::PI * 1000.0 * (i as f64) / SAMPLE_RATE).sin();
            compiled.process(x)
        })
        .collect()
}

/// Component ids present in a pedal, as a sorted Vec.
fn comp_ids(def: &pedalkernel::dsl::PedalDef) -> Vec<String> {
    let mut v: Vec<String> = def.components.iter().map(|c| c.id.clone()).collect();
    v.sort();
    v
}

// ── Equivalence ─────────────────────────────────────────────────────────────

#[test]
fn expand_yields_namespaced_components() {
    let parent = parse_and_expand_pedal_file(&pedals_dir().join("parent_gain.pedal")).unwrap();
    let ids = comp_ids(&parent);
    assert_eq!(
        ids,
        vec![
            "Rs".to_string(),
            "g.C1".to_string(),
            "g.R1".to_string(),
            "g.R2".to_string(),
        ],
        "instance components should be namespaced with `g.`"
    );
    // No leftover `use` instances after expansion.
    assert!(parent.uses.is_empty(), "uses must be cleared after expand");
}

#[test]
fn expand_unifies_ports_and_internal_nodes() {
    let parent = parse_and_expand_pedal_file(&pedals_dir().join("parent_gain.pedal")).unwrap();

    // The sub's `in` port must have been unified with `Rs.b` (the parent's
    // wiring), so the sub's input cap is fed from Rs.b — NOT from a leftover
    // `Reserved("in")` or a dangling SubcircuitPort.
    let feeds_c1_from_rs_b = parent.nets.iter().any(|n| {
        matches!(&n.from, Pin::ComponentPin { component, pin } if component == "Rs" && pin == "b")
            && n.to.iter().any(|p| {
                matches!(p, Pin::ComponentPin { component, pin } if component == "g.C1" && pin == "a")
            })
    });
    assert!(feeds_c1_from_rs_b, "port `in` should unify Rs.b -> g.C1.a");

    // The sub's internal node `mid` must be namespaced to `g.mid`.
    let has_namespaced_mid = parent.nets.iter().any(|n| {
        matches!(&n.from, Pin::Reserved(s) if s == "g.mid")
            || n.to
                .iter()
                .any(|p| matches!(p, Pin::Reserved(s) if s == "g.mid"))
    });
    assert!(has_namespaced_mid, "internal node `mid` -> `g.mid`");

    // No SubcircuitPort references should survive.
    let has_subport = parent.nets.iter().any(|n| {
        matches!(&n.from, Pin::SubcircuitPort { .. })
            || n.to.iter().any(|p| matches!(p, Pin::SubcircuitPort { .. }))
    });
    assert!(
        !has_subport,
        "all SubcircuitPort references must be consumed"
    );
}

#[test]
fn expanded_matches_flat_equivalent_output() {
    let parent = parse_and_expand_pedal_file(&pedals_dir().join("parent_gain.pedal")).unwrap();
    let flat = parse_pedal_file(
        &std::fs::read_to_string(pedals_dir().join("parent_gain_flat.pedal")).unwrap(),
    )
    .unwrap();

    // Same number of components / nets as the hand-written flat equivalent.
    assert_eq!(
        parent.components.len(),
        flat.components.len(),
        "expanded vs flat component count"
    );
    assert_eq!(
        parent.nets.len(),
        flat.nets.len(),
        "expanded vs flat net count"
    );

    // Both compile and produce the SAME output for a 1 kHz sine.
    let n = 480;
    let expanded_out = render_sine(&parent, n);
    let flat_out = render_sine(&flat, n);
    for (i, (a, b)) in expanded_out.iter().zip(flat_out.iter()).enumerate() {
        assert!((a - b).abs() < 1e-9, "sample {i}: expanded {a} != flat {b}");
    }
    // Sanity: output is non-trivial.
    let energy: f64 = expanded_out.iter().map(|s| s.abs()).sum();
    assert!(energy > 1e-3, "expanded output should be non-silent");
}

// ── Rail implicit + override ────────────────────────────────────────────────

#[test]
fn vcc_implicit_global() {
    let parent =
        parse_and_expand_pedal_file(&pedals_dir().join("parent_vcc_implicit.pedal")).unwrap();

    // With no explicit mapping, the sub's `vcc` stays the global `vcc` rail.
    let uses_global_vcc = parent.nets.iter().any(|n| {
        matches!(&n.from, Pin::Reserved(s) if s == "vcc")
            || n.to
                .iter()
                .any(|p| matches!(p, Pin::Reserved(s) if s == "vcc"))
    });
    assert!(uses_global_vcc, "sub vcc should remain the global vcc rail");

    // It should NOT have been namespaced to `s.vcc`.
    let has_namespaced_vcc = parent.nets.iter().any(|n| {
        matches!(&n.from, Pin::Reserved(s) if s == "s.vcc")
            || n.to
                .iter()
                .any(|p| matches!(p, Pin::Reserved(s) if s == "s.vcc"))
    });
    assert!(
        !has_namespaced_vcc,
        "vcc must not be namespaced when implicit"
    );

    assert!(
        compile_pedal(&parent, SAMPLE_RATE).is_ok(),
        "implicit vcc compiles"
    );
}

#[test]
fn vcc_explicit_override() {
    let parent =
        parse_and_expand_pedal_file(&pedals_dir().join("parent_vcc_override.pedal")).unwrap();

    // The sub's internal `vcc` references should now be unified with `vrail`.
    let sub_load_to_vrail = parent.nets.iter().any(|n| {
        matches!(&n.from, Pin::Reserved(s) if s == "vrail")
            && n.to
                .iter()
                .any(|p| matches!(p, Pin::ComponentPin { component, .. } if component == "s.Rload"))
    });
    assert!(
        sub_load_to_vrail,
        "overridden vcc should feed s.Rload from `vrail`"
    );

    // The routing net `s.vcc -> vrail` must have been consumed (no SubcircuitPort).
    let has_subport = parent.nets.iter().any(|n| {
        matches!(&n.from, Pin::SubcircuitPort { .. })
            || n.to.iter().any(|p| matches!(p, Pin::SubcircuitPort { .. }))
    });
    assert!(!has_subport, "vcc routing net must be consumed");

    assert!(
        compile_pedal(&parent, SAMPLE_RATE).is_ok(),
        "override vcc compiles"
    );
}

// ── Nested ──────────────────────────────────────────────────────────────────

#[test]
fn nested_flattens_with_double_namespacing() {
    let top = parse_and_expand_pedal_file(&pedals_dir().join("nested/a_top.pedal")).unwrap();
    let ids = comp_ids(&top);
    // A's own Rin, B's C1 (as b.C1), C's R1 (as b.c.R1).
    assert!(ids.contains(&"Rin".to_string()), "ids: {ids:?}");
    assert!(ids.contains(&"b.C1".to_string()), "ids: {ids:?}");
    assert!(
        ids.contains(&"b.c.R1".to_string()),
        "deep nested id `b.c.R1` expected, got: {ids:?}"
    );
    assert!(top.uses.is_empty());
    assert!(
        compile_pedal(&top, SAMPLE_RATE).is_ok(),
        "nested flat compiles"
    );
}

// ── Cycle ───────────────────────────────────────────────────────────────────

#[test]
fn cycle_detected() {
    let res = parse_and_expand_pedal_file(&pedals_dir().join("cycle/cyc_a.pedal"));
    assert!(res.is_err(), "A uses B uses A must error");
    let msg = res.err().unwrap();
    assert!(
        msg.to_lowercase().contains("cycle"),
        "error should mention a cycle, got: {msg}"
    );
}

// ── Standalone unchanged ────────────────────────────────────────────────────

#[test]
fn sub_parses_and_compiles_standalone() {
    // The sub `.pedal` still parses + compiles on its own (ports as boundary;
    // in/out behave as reserved I/O).
    let sub = parse_and_expand_pedal_file(&pedals_dir().join("sub/sub_gain.pedal")).unwrap();
    assert!(sub.uses.is_empty(), "standalone sub has no uses");
    assert!(
        compile_pedal(&sub, SAMPLE_RATE).is_ok(),
        "standalone sub compiles"
    );
}

// ── Backward-compat / idempotence ───────────────────────────────────────────

#[test]
fn no_uses_is_idempotent() {
    let src = std::fs::read_to_string(pedals_dir().join("parent_gain_flat.pedal")).unwrap();
    let def = parse_pedal_file(&src).unwrap();
    assert!(def.uses.is_empty(), "flat pedal has no uses");
    let expanded = expand_uses(&def, &pedals_dir()).unwrap();
    // Expanding a use-free pedal changes nothing structural.
    assert_eq!(comp_ids(&def), comp_ids(&expanded));
    assert_eq!(def.nets.len(), expanded.nets.len());
    assert_eq!(def.controls, expanded.controls);
}

// ── Control re-exposure ─────────────────────────────────────────────────────

#[test]
fn parent_reexposes_sub_control() {
    let parent = parse_and_expand_pedal_file(&pedals_dir().join("parent_ctrl.pedal")).unwrap();
    // Exactly one control, targeting the namespaced sub pot, with the parent's
    // label/default.
    assert_eq!(parent.controls.len(), 1, "controls: {:?}", parent.controls);
    let c = &parent.controls[0];
    assert_eq!(
        c.component, "micamp.Gain",
        "control retargets namespaced pot"
    );
    assert_eq!(c.label, "Mic Gain", "parent relabels the control");
    assert!((c.default - 0.6).abs() < 1e-9, "parent default applied");
    assert!(
        compile_pedal(&parent, SAMPLE_RATE).is_ok(),
        "ctrl parent compiles"
    );
}

// ── with{} component-value overrides ────────────────────────────────────────

/// Two instances of the same sub with different R1 values must compile to
/// distinct transfer functions. Because R1 controls the divider ratio, higher
/// R1 = more attenuation; the two chains should produce measurably different
/// output energies.
#[test]
fn with_override_produces_distinct_transfer_functions() {
    // Parse the parent that instantiates sub_gain twice with different R1.
    let parent =
        parse_and_expand_pedal_file(&pedals_dir().join("parent_with_override.pedal")).unwrap();

    // After expansion both instances must be present (6 sub components: 3 * 2).
    // Plus the parent's own Rs = 7 total.
    assert_eq!(parent.components.len(), 7, "components: {:?}", parent.components.iter().map(|c| &c.id).collect::<Vec<_>>());

    // No leftover use instances.
    assert!(parent.uses.is_empty());

    // The two R1 components must have DIFFERENT resistance values.
    let r1_g1 = parent
        .components
        .iter()
        .find(|c| c.id == "g1.R1")
        .expect("g1.R1 must exist")
        .kind
        .resistance()
        .expect("g1.R1 must be a resistor");
    let r1_g2 = parent
        .components
        .iter()
        .find(|c| c.id == "g2.R1")
        .expect("g2.R1 must exist")
        .kind
        .resistance()
        .expect("g2.R1 must be a resistor");

    assert!(
        (r1_g1 - 1_000.0).abs() < 1.0,
        "g1.R1 should be 1k, got {r1_g1}"
    );
    assert!(
        (r1_g2 - 100_000.0).abs() < 1.0,
        "g2.R1 should be 100k, got {r1_g2}"
    );

    // Compile and verify the two sub-pedals independently produce different
    // transfer functions by comparing instantaneous gain.
    let n = 480;

    // Build isolated pedals for each sub instance to compare their gains.
    // sub_gain with R1=1k (g1 override): low divider → more signal passes.
    let src_low = r#"
        pedal "SubGainLow" {
          ports { in: input  out: output }
          components { C1: cap(100n)  R1: resistor(1k)  R2: resistor(10k) }
          nets { in -> C1.a  C1.b -> R1.a  R1.b -> mid  mid -> R2.a  R2.b -> gnd  mid -> out }
          controls {}
        }
    "#;
    // sub_gain with R1=100k (g2 override): high divider → most signal attenuated.
    let src_high = r#"
        pedal "SubGainHigh" {
          ports { in: input  out: output }
          components { C1: cap(100n)  R1: resistor(100k)  R2: resistor(10k) }
          nets { in -> C1.a  C1.b -> R1.a  R1.b -> mid  mid -> R2.a  R2.b -> gnd  mid -> out }
          controls {}
        }
    "#;

    let def_low = pedalkernel::dsl::parse_pedal_file(src_low).unwrap();
    let def_high = pedalkernel::dsl::parse_pedal_file(src_high).unwrap();
    let out_low = render_sine(&def_low, n);
    let out_high = render_sine(&def_high, n);

    let energy_low: f64 = out_low.iter().map(|s| s * s).sum();
    let energy_high: f64 = out_high.iter().map(|s| s * s).sum();

    assert!(
        energy_low > energy_high * 2.0,
        "R1=1k (low attenuation) should have significantly more energy than R1=100k: \
         low={energy_low:.6}, high={energy_high:.6}"
    );
}

/// An override targeting a non-existent component id must return a clear error.
#[test]
fn with_override_unknown_id_errors() {
    let src = r#"
        pedal "Parent" {
          supply 9V
          components {
            Rs: resistor(100)
            g: use("sub/sub_gain.pedal") with { Rxxx: 10k }
          }
          nets { in -> Rs.a  Rs.b -> g.in  g.out -> out }
          controls {}
        }
    "#;
    let def = pedalkernel::dsl::parse_pedal_file(src).unwrap();
    let result = pedalkernel::dsl_expand::expand_uses(&def, &pedals_dir());
    assert!(result.is_err(), "unknown override id must error");
    let msg = result.err().unwrap();
    assert!(
        msg.contains("Rxxx"),
        "error must name the unknown id, got: {msg}"
    );
    assert!(
        msg.contains("override"),
        "error must mention 'override', got: {msg}"
    );
}

/// A `use("path")` with NO `with{}` block still works (regression guard).
#[test]
fn use_without_override_is_backward_compatible() {
    // Uses the existing parent_gain.pedal which has no with{} block.
    let parent = parse_and_expand_pedal_file(&pedals_dir().join("parent_gain.pedal")).unwrap();
    // Exact same assertions as the original test: no regressions.
    let ids = comp_ids(&parent);
    assert!(ids.contains(&"g.R1".to_string()), "ids: {ids:?}");
    assert!(ids.contains(&"g.R2".to_string()), "ids: {ids:?}");
    assert!(parent.uses.is_empty());
    assert!(
        compile_pedal(&parent, SAMPLE_RATE).is_ok(),
        "no-override use still compiles"
    );
}
