//! Integration tests for per-terminal neighbour-role inference and the
//! compile-time completeness diagnostic.
//!
//! These live in an integration-test binary (not `--lib` unit tests) so they
//! build under `--no-default-features` without pulling jack-sys.
//!
//! Coverage:
//! - (corpus gate) every committed working `.pedal` compiles WITHOUT a
//!   completeness error — the make-or-break false-positive gate.
//! - (a) role inference on a real 12AX7 common-cathode stage.
//! - (b) a cathode-follower triode compiles clean (config tolerance).
//! - (c) a malformed triode with NO load on any terminal trips the error.
//! - (d) a transformer-coupled plate compiles clean (winding = Load).

use std::path::{Path, PathBuf};

use pedalkernel::compiler::neighbor_roles::{completeness_errors, infer_neighbor_roles};
use pedalkernel::compiler::{compile_pedal, NeighborRole};
use pedalkernel::dsl::parse_pedal_file;

const SR: f64 = 48_000.0;

// ═══════════════════════════════════════════════════════════════════════════
// SAFETY GATE: zero false positives on the working corpus
// ═══════════════════════════════════════════════════════════════════════════

fn collect_pedal_files(root: &Path, out: &mut Vec<PathBuf>) {
    if let Ok(entries) = std::fs::read_dir(root) {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                collect_pedal_files(&path, out);
            } else if path.extension().map_or(false, |e| e == "pedal") {
                out.push(path);
            }
        }
    }
}

#[test]
fn corpus_has_zero_completeness_false_positives() {
    // CWD when tests run is the crate root: pedalkernel/.
    let mut files = Vec::new();
    collect_pedal_files(Path::new("examples"), &mut files);
    collect_pedal_files(Path::new("../pedalkernel-validate/circuits"), &mut files);

    assert!(
        !files.is_empty(),
        "no .pedal corpus files found — wrong CWD?"
    );

    let mut offenders: Vec<String> = Vec::new();
    let mut checked = 0usize;

    for path in &files {
        let src = std::fs::read_to_string(path).unwrap();
        let pedal = match parse_pedal_file(&src) {
            Ok(p) => p,
            // Parse failures are not this pass's concern (and are exercised
            // elsewhere); skip them so we only gate the completeness check.
            Err(_) => continue,
        };
        checked += 1;

        // Direct, isolated completeness probe (independent of full compile so a
        // failure here is unambiguously a completeness false positive).
        let errs = completeness_errors(&pedal);
        if !errs.is_empty() {
            offenders.push(format!("{}: {:?}", path.display(), errs));
        }

        // Also confirm the full compile path does not regress this corpus with
        // a NEW completeness-origin error. (Some circuits legitimately fail to
        // compile for unrelated reasons — only flag completeness errors.)
        if let Err(e) = compile_pedal(&pedal, SR) {
            if e.contains("completeness error") {
                offenders.push(format!("{} (via compile): {}", path.display(), e));
            }
        }
    }

    assert!(
        offenders.is_empty(),
        "completeness check produced {} false positive(s) on {} working circuits:\n{}",
        offenders.len(),
        checked,
        offenders.join("\n")
    );

    eprintln!("corpus completeness gate: {checked} working .pedal files clean");
}

// ═══════════════════════════════════════════════════════════════════════════
// (a) Role inference on a real 12AX7 common-cathode stage
// ═══════════════════════════════════════════════════════════════════════════

const COMMON_CATHODE_12AX7: &str = r#"
pedal "12AX7 Common Cathode" {
  supply 250V {
    impedance: 50
    filter_cap: 47u
  }
  components {
    C_in: cap(22n)
    R_grid: resistor(1M)
    V1: triode(12ax7)
    R_plate: resistor(100k)
    R_cathode: resistor(1.5k)
    C_cathode: cap(25u)
    C_out: cap(22n)
    R_load: resistor(1M)
  }
  nets {
    in -> C_in.a
    C_in.b -> R_grid.a, V1.grid
    R_grid.b -> gnd
    vcc -> R_plate.a
    R_plate.b -> V1.plate
    V1.cathode -> R_cathode.a, C_cathode.a
    R_cathode.b -> gnd
    C_cathode.b -> gnd
    V1.plate -> C_out.a
    C_out.b -> R_load.a, out
    R_load.b -> gnd
  }
}
"#;

#[test]
fn role_inference_common_cathode_12ax7() {
    let pedal = parse_pedal_file(COMMON_CATHODE_12AX7).expect("parse");
    let roles = infer_neighbor_roles(&pedal);

    let role_of = |neighbor: &str| -> Option<NeighborRole> {
        roles
            .iter()
            .find(|r| r.comp_id == "V1" && r.neighbor_id.as_deref() == Some(neighbor))
            .map(|r| r.role)
    };

    // Plate-load resistor to the 250V rail → Load.
    assert_eq!(
        role_of("R_plate"),
        Some(NeighborRole::Load),
        "R_plate should be a Load; roles = {roles:?}"
    );
    // Grid-leak resistor to gnd → Ref.
    assert_eq!(
        role_of("R_grid"),
        Some(NeighborRole::Ref),
        "R_grid should be a Ref; roles = {roles:?}"
    );
    // Cathode bias resistor + bypass cap to gnd → Ref.
    assert_eq!(role_of("R_cathode"), Some(NeighborRole::Ref));
    assert_eq!(role_of("C_cathode"), Some(NeighborRole::Ref));
    // Output coupling cap on the plate, far side → out/load (no other active
    // input downstream) → not a Load, not a Signal-to-active → Ref-ish.
    // Input coupling cap on the grid path is the series Signal to... in (source),
    // so we only assert the clearly-load/ref ones above plus completeness clean.
    assert!(
        completeness_errors(&pedal).is_empty(),
        "common-cathode must be complete: {:?}",
        completeness_errors(&pedal)
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// (b) Cathode follower — load on the cathode, plate straight to B+
// ═══════════════════════════════════════════════════════════════════════════

const CATHODE_FOLLOWER: &str = r#"
pedal "12AX7 Cathode Follower" {
  supply 250V {
    impedance: 50
  }
  components {
    C_in: cap(22n)
    R_grid: resistor(1M)
    V1: triode(12ax7)
    R_cathode: resistor(100k)
    C_out: cap(1u)
    R_load: resistor(1M)
  }
  nets {
    in -> C_in.a
    C_in.b -> R_grid.a, V1.grid
    R_grid.b -> gnd
    vcc -> V1.plate
    V1.cathode -> R_cathode.a, C_out.a
    R_cathode.b -> gnd
    C_out.b -> R_load.a, out
    R_load.b -> gnd
  }
}
"#;

#[test]
fn cathode_follower_compiles_clean() {
    let pedal = parse_pedal_file(CATHODE_FOLLOWER).expect("parse");
    // Plate straight to B+, load on the cathode — must NOT trip the Load check.
    // This is the core deliverable (b): a cathode follower compiles clean.
    let errs = completeness_errors(&pedal);
    assert!(
        errs.is_empty(),
        "cathode follower is a valid config and must not error: {errs:?}"
    );
    // And the full compile path accepts it (no completeness error).
    match compile_pedal(&pedal, SR) {
        Ok(_) => {}
        Err(e) => assert!(
            !e.contains("completeness error"),
            "cathode follower must not produce a completeness error: {e}"
        ),
    }
    // Inference still classifies the cathode network (here a bias R to ground,
    // electrically a Ref) — the follower-load semantics is a device-level
    // interpretation applied by the completeness check, not the per-edge view.
    let roles = infer_neighbor_roles(&pedal);
    assert!(
        roles.iter().any(|r| r.comp_id == "V1" && r.terminal == "cathode"),
        "cathode neighbours should be classified; roles = {roles:?}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// (c) Malformed: triode with NO load on any terminal → completeness error
// ═══════════════════════════════════════════════════════════════════════════

const NO_LOAD_TRIODE: &str = r#"
pedal "Broken Triode" {
  supply 250V {}
  components {
    C_in: cap(22n)
    R_grid: resistor(1M)
    V1: triode(12ax7)
    R_cathode: resistor(1.5k)
    C_out: cap(22n)
    R_load: resistor(1M)
  }
  nets {
    in -> C_in.a
    C_in.b -> R_grid.a, V1.grid
    R_grid.b -> gnd
    # plate floats to the output only — no path to a supply rail anywhere
    V1.plate -> C_out.a
    C_out.b -> R_load.a, out
    R_load.b -> gnd
    V1.cathode -> R_cathode.a
    R_cathode.b -> gnd
  }
}
"#;

#[test]
fn no_load_triode_trips_completeness_error() {
    let pedal = parse_pedal_file(NO_LOAD_TRIODE).expect("parse");
    let errs = completeness_errors(&pedal);
    assert!(
        !errs.is_empty(),
        "a triode with no load on any terminal must produce a completeness error"
    );
    let msg = errs.join("\n");
    assert!(msg.contains("V1"), "error must name the component: {msg}");
    assert!(msg.contains("triode"), "error must name the device type: {msg}");
    assert!(msg.contains("no Load found"), "error must name the role: {msg}");
    assert!(
        msg.contains("cathode follower"),
        "error must give the fix hint: {msg}"
    );

    // And the full compile path must reject it too.
    match compile_pedal(&pedal, SR) {
        Ok(_) => panic!("malformed circuit must not compile"),
        Err(compile_err) => assert!(
            compile_err.contains("completeness error"),
            "compile error should be the completeness error: {compile_err}"
        ),
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// (d) Transformer-coupled plate — winding recognised as Load
// ═══════════════════════════════════════════════════════════════════════════

const TRANSFORMER_COUPLED: &str = r#"
pedal "Transformer-Coupled Output Stage" {
  supply 250V {
    impedance: 50
  }
  components {
    C_in: cap(22n)
    R_grid: resistor(1M)
    V1: triode(12ax7)
    OT: transformer(4:1, 10H)
    R_cathode: resistor(1.5k)
    C_cathode: cap(25u)
  }
  nets {
    in -> C_in.a
    C_in.b -> R_grid.a, V1.grid
    R_grid.b -> gnd
    # plate develops its output across the primary winding (which returns to B+)
    V1.plate -> OT.a
    OT.b -> vcc
    OT.c -> out
    OT.d -> gnd
    V1.cathode -> R_cathode.a, C_cathode.a
    R_cathode.b -> gnd
    C_cathode.b -> gnd
  }
}
"#;

#[test]
fn transformer_coupled_plate_compiles_clean() {
    let pedal = parse_pedal_file(TRANSFORMER_COUPLED).expect("parse");
    let errs = completeness_errors(&pedal);
    assert!(
        errs.is_empty(),
        "transformer winding on the plate is a valid Load and must not error: {errs:?}"
    );
    let roles = infer_neighbor_roles(&pedal);
    let plate_load = roles.iter().any(|r| {
        r.comp_id == "V1"
            && r.terminal == "plate"
            && r.neighbor_id.as_deref() == Some("OT")
            && r.role == NeighborRole::Load
    });
    assert!(
        plate_load,
        "transformer primary on the plate should be a Load; roles = {roles:?}"
    );
}
