//! `PK_XFMR_REFLECT_DISABLE` opt-out for the transformer-secondary load
//! reflection (mirrors `PK_OUTPUT_FUSE_DISABLE`).
//!
//! ISOLATION NOTE: this file holds exactly ONE test because it mutates the
//! process environment — the harness runs tests within a binary concurrently,
//! and any concurrent compile in this process would see the variable.

use pedalkernel::dsl::parse_pedal_file;

/// Same fixture as `xfmr_secondary_reflection.rs`.
const BUF_XFMR_OUT: &str = r#"
pedal "buffer xfmr out" {
  supply 9V
  components {
    U1: opamp(tl072)
    Cout: cap(10u, electrolytic)
    T1: transformer(10:1, 2H)
    RL: resistor(1k)
  }
  nets {
    in -> U1.pos
    U1.out -> U1.neg, Cout.a
    Cout.b -> T1.a
    T1.b -> gnd
    T1.c -> out, RL.a
    T1.d -> gnd
    RL.b -> gnd
  }
}
"#;

#[test]
fn env_opt_out_declines_reflection_with_reason() {
    let def = parse_pedal_file(BUF_XFMR_OUT).expect("parse");

    std::env::set_var("PK_XFMR_REFLECT_DISABLE", "1");
    let disabled = pedalkernel::compiler::compile_pedal(&def, 48_000.0).expect("compile");
    std::env::remove_var("PK_XFMR_REFLECT_DISABLE");

    // The shape still analyzes — the row records the env decline.
    assert_eq!(
        disabled.boundary_loads.len(),
        1,
        "expected the analyzed-but-declined row, got {:?}",
        disabled.boundary_loads
    );
    match &disabled.boundary_loads[0].disposition {
        pedalkernel_rt::processor::BoundaryLoadDisposition::Unloaded { reason } => {
            assert_eq!(reason, "env:PK_XFMR_REFLECT_DISABLE");
        }
        other => panic!("expected env-declined Unloaded disposition, got {other:?}"),
    }

    // And the reflection re-engages without the variable.
    let enabled = pedalkernel::compiler::compile_pedal(&def, 48_000.0).expect("compile");
    assert!(
        enabled.boundary_loads.iter().any(|b| matches!(
            b.disposition,
            pedalkernel_rt::processor::BoundaryLoadDisposition::ReflectedThroughTransformer { .. }
        )),
        "without the env opt-out the reflection must engage, got {:?}",
        enabled.boundary_loads
    );
    // The disabled compile keeps the standalone load stage; the enabled one
    // consumes it.
    assert_eq!(
        disabled.stages.len(),
        enabled.stages.len() + 1,
        "opt-out must keep the standalone secondary-load stage"
    );
}
