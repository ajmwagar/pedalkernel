//! Parser tests for the optional `init { ... }` DSL block.
//!
//! WHY: The `init` block lets authors declare asymmetric NL device starting states,
//! enabling free-running oscillators (BJT astable multivibrators) to escape the
//! symmetric DC saddle that traps the homotopy pre-convergence.
//!
//! These tests verify the parser in isolation — no compilation or audio processing.

use pedalkernel::dsl::parse_pedal_file;

// ── Sub-task 2 parser tests ───────────────────────────────────────────────────

/// WHY: Confirm a minimal pedal with `init { Q1: saturated }` parses correctly and
/// the hint is captured in `PedalDef.init_hints`. Without this, the homotopy override
/// has no data to act on.
#[test]
fn parser_accepts_init_block() {
    let src = r#"
pedal "Init Test" {
  supply 9V
  components {
    Q1: npn(2n3904)
    R1: resistor(10k)
  }
  nets {
    vcc -> R1.a
    R1.b -> Q1.collector
    Q1.base -> in
    Q1.emitter -> gnd
    Q1.collector -> out
  }
  init {
    Q1: saturated
  }
}
"#;
    let pedal = parse_pedal_file(src).expect("should parse successfully");
    assert_eq!(pedal.init_hints.len(), 1, "expected 1 init hint");
    assert_eq!(pedal.init_hints[0].device_label, "Q1");
    let pedalkernel::dsl::InitState::Named(name) = &pedal.init_hints[0].state else {
        panic!(
            "expected a Named init state, got: {:?}",
            pedal.init_hints[0].state
        );
    };
    assert_eq!(name, "saturated", "expected 'saturated' named state");
}

/// WHY: Unknown named states should cause a parse error that mentions the bad value.
/// Without this guard, a typo like `saturatd` would silently become a no-op and
/// the author would have no idea why their oscillator still doesn't work.
#[test]
fn parser_rejects_unknown_init_state() {
    let src = r#"
pedal "Init Test" {
  supply 9V
  components {
    Q1: npn(2n3904)
    R1: resistor(10k)
  }
  nets {
    vcc -> R1.a
    R1.b -> Q1.collector
    Q1.base -> in
    Q1.emitter -> gnd
    Q1.collector -> out
  }
  init {
    Q1: foo
  }
}
"#;
    let result = parse_pedal_file(src);
    assert!(
        result.is_err(),
        "expected parse error for unknown init state 'foo', got: {result:?}"
    );
}

/// WHY: Existing .pedals without an `init` block must parse identically to before.
/// Any regression here would silently break all existing pedals.
#[test]
fn parser_no_init_block_is_empty_vec() {
    // A representative minimal pedal with no init block.
    let src = r#"
pedal "No Init" {
  supply 9V
  components {
    Q1: npn(2n3904)
    R1: resistor(10k)
  }
  nets {
    vcc -> R1.a
    R1.b -> Q1.collector
    Q1.base -> in
    Q1.emitter -> gnd
    Q1.collector -> out
  }
}
"#;
    let pedal = parse_pedal_file(src).expect("should parse successfully");
    assert!(
        pedal.init_hints.is_empty(),
        "expected empty init_hints for pedal without init block, got {:?}",
        pedal.init_hints
    );
}

/// WHY: Multiple hints in one init block must all be captured.
#[test]
fn parser_accepts_multiple_init_hints() {
    let src = r#"
pedal "Multi Init" {
  supply 9V
  components {
    Q1: npn(2n3904)
    Q2: npn(2n3904)
    R1: resistor(10k)
    R2: resistor(10k)
  }
  nets {
    vcc -> R1.a
    R1.b -> Q1.collector
    vcc -> R2.a
    R2.b -> Q2.collector
    Q1.base -> in
    Q1.emitter -> gnd
    Q2.base -> gnd
    Q2.emitter -> gnd
    Q1.collector -> out
    Q2.collector -> out
  }
  init {
    Q1: saturated
    Q2: cutoff
  }
}
"#;
    let pedal = parse_pedal_file(src).expect("should parse successfully");
    assert_eq!(pedal.init_hints.len(), 2, "expected 2 init hints");
    assert_eq!(pedal.init_hints[0].device_label, "Q1");
    assert_eq!(pedal.init_hints[1].device_label, "Q2");
    assert!(
        matches!(
            &pedal.init_hints[0].state,
            pedalkernel::dsl::InitState::Named(n) if n == "saturated"
        ),
        "Q1 should be saturated"
    );
    assert!(
        matches!(
            &pedal.init_hints[1].state,
            pedalkernel::dsl::InitState::Named(n) if n == "cutoff"
        ),
        "Q2 should be cutoff"
    );
}

/// WHY: All valid named states must be accepted. If a state name is added to the
/// resolver but not the parser (or vice versa), the mismatch would cause silent
/// failures at compile time.
#[test]
fn parser_accepts_all_named_states() {
    for state_name in &["saturated", "cutoff", "active", "forward", "reverse"] {
        let src = format!(
            r#"
pedal "State Test" {{
  supply 9V
  components {{
    Q1: npn(2n3904)
    R1: resistor(10k)
  }}
  nets {{
    vcc -> R1.a
    R1.b -> Q1.collector
    Q1.base -> in
    Q1.emitter -> gnd
    Q1.collector -> out
  }}
  init {{
    Q1: {state_name}
  }}
}}
"#
        );
        let result = parse_pedal_file(&src);
        assert!(
            result.is_ok(),
            "named state '{state_name}' should be accepted, got: {result:?}"
        );
        let pedal = result.unwrap();
        assert_eq!(pedal.init_hints.len(), 1);
        assert!(
            matches!(&pedal.init_hints[0].state, pedalkernel::dsl::InitState::Named(n) if n == state_name),
            "state should be Named({state_name})"
        );
    }
}
