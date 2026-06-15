//! Proof diagnostic for the DERIVED port-classification "rules broker"
//! (`compiler/boundary_rules.rs`, Phase 1).
//!
//! These live in an integration-test binary (not `--lib` unit tests) so they
//! build under `--no-default-features` without pulling jack-sys, exactly like
//! `tests/neighbor_roles.rs`.
//!
//! The deliverable this binary proves is that the broker's classification is
//! DERIVED CORRECTLY FROM THE DEVICE MODELS — it asserts NOTHING about audio
//! behaviour (the broker is wired into no formation/solving in this phase).
//!
//! Coverage:
//!  - (LA-2A) the three landmark boundaries of the faithful LA-2A:
//!      * `EL_drive.b -> PC1.led`            -> DelayedCoupling(Light)
//!      * `out -> ... -> V4.grid` (feedback) -> DelayedSense
//!      * `V1.plate -> C_c1 -> Gain -> V2.grid` (cascade) -> Tight
//!  - (forward cascades) op-amp pedal + 2-tube amp: forward control-input
//!    edges classify Tight (no spurious DelayedSense).
//!  - (corpus smoke) every committed `.pedal` classifies every pin without a
//!    panic and yields a class for each — the robustness gate, mirroring the
//!    neighbor-roles corpus gate.

use std::path::{Path, PathBuf};

use pedalkernel::compiler::boundary_rules::{
    classify_control_path, classify_edges, classify_ports, smoke_classify, BoundaryPolicy, Domain,
    PortClass,
};
use pedalkernel::dsl::parse_pedal_file;

// ───────────────────────── helpers ─────────────────────────

fn read_example(rel: &str) -> String {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples").join(rel);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

fn read_validate_circuit(rel: &str) -> String {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../pedalkernel-validate/circuits")
        .join(rel);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

// ═══════════════════════════════════════════════════════════════════════════
// LA-2A: the three landmark boundary classifications (the deliverable)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn la2a_el_drive_to_led_is_delayed_coupling_light() {
    let src = read_example("outboard/compressor/la2a.pedal");
    let pedal = parse_pedal_file(&src).expect("parse la2a");

    // `EL_drive.b -> PC1.led.a`: the photocoupler LED is a transducer port.
    // Derived from PC1's modulation_sink("led") == PhotocouplerLed -> Light.
    let edge = classify_control_path(&pedal, "PC1.led", "EL_drive.b")
        .or_else(|| classify_control_path(&pedal, "PC1.led.a", "EL_drive.b"))
        .expect("PC1.led path must resolve");

    assert_eq!(
        edge.to_class,
        PortClass::Transducer(Domain::Light),
        "PC1.led must classify Transducer(Light), got {:?}",
        edge.to_class
    );
    assert_eq!(
        edge.policy,
        BoundaryPolicy::DelayedCoupling(Domain::Light),
        "edge INTO the LED transducer must be DelayedCoupling(Light), got {:?}",
        edge.policy
    );
}

#[test]
fn la2a_feedback_tap_into_sidechain_grid_is_delayed_sense() {
    let src = read_example("outboard/compressor/la2a.pedal");
    let pedal = parse_pedal_file(&src).expect("parse la2a");

    // The sidechain detector taps the OUTPUT and feeds it BACK into the
    // 12AX7 sidechain amp grid (V4.grid). Relative to directed signal flow
    // from `in`, V4.grid sits upstream of `out` -> this is a feedback (back)
    // edge into a control electrode -> DelayedSense.
    let edge =
        classify_control_path(&pedal, "V4.grid", "out").expect("V4.grid <- out path must resolve");

    assert_eq!(
        edge.to_class,
        PortClass::ControlInput,
        "V4.grid is a triode grid -> ControlInput, got {:?}",
        edge.to_class
    );
    assert!(
        edge.is_back_edge,
        "out -> V4.grid is a feedback tap; must be a back-edge"
    );
    assert_eq!(
        edge.policy,
        BoundaryPolicy::DelayedSense,
        "feedback into a control electrode must be DelayedSense, got {:?}",
        edge.policy
    );
}

#[test]
fn la2a_makeup_cascade_into_v2_grid_is_tight() {
    let src = read_example("outboard/compressor/la2a.pedal");
    let pedal = parse_pedal_file(&src).expect("parse la2a");

    // The makeup cascade `V1.plate -> C_c1 -> Gain -> V2.grid` is a normal
    // FORWARD path into a control electrode -> Tight (co-solve).
    let edge = classify_control_path(&pedal, "V2.grid", "V1.plate")
        .expect("V2.grid <- V1.plate path must resolve");

    assert_eq!(
        edge.to_class,
        PortClass::ControlInput,
        "V2.grid is a triode grid -> ControlInput, got {:?}",
        edge.to_class
    );
    assert!(
        !edge.is_back_edge,
        "V1.plate -> ... -> V2.grid is a forward cascade, not feedback"
    );
    assert_eq!(
        edge.policy,
        BoundaryPolicy::Tight,
        "forward cascade into a control electrode must be Tight, got {:?}",
        edge.policy
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Forward control-input edges classify Tight (no spurious DelayedSense)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn opamp_pedal_forward_input_is_tight_no_spurious_sense() {
    // Non-inverting op-amp: `in -> U1.pos`. U1.pos is the op-amp control
    // (non-inverting) input -> ControlInput. The input drive is FORWARD, so it
    // must be Tight, never DelayedSense. (The U1.neg feedback divider is a
    // separate, legitimately-back edge and is NOT asserted here.)
    let src = read_validate_circuit("opamp/noninverting_gain10.pedal");
    let pedal = parse_pedal_file(&src).expect("parse noninverting_gain10");

    let edge =
        classify_control_path(&pedal, "U1.pos", "in").expect("U1.pos <- in path must resolve");
    assert_eq!(
        edge.to_class,
        PortClass::ControlInput,
        "op-amp pos input -> ControlInput, got {:?}",
        edge.to_class
    );
    assert!(
        !edge.is_back_edge,
        "in -> U1.pos is the forward signal drive, not feedback"
    );
    assert_eq!(
        edge.policy,
        BoundaryPolicy::Tight,
        "forward drive into op-amp input must be Tight, got {:?}",
        edge.policy
    );
}

#[test]
fn two_tube_cascade_forward_grids_are_tight() {
    // Tweed Deluxe 5E3: a two-12AX7 cascade. Both stage inputs are fed
    // FORWARD; neither grid edge may be DelayedSense.
    let src = read_example("amps/tweed_deluxe_5e3.pedal");
    let pedal = parse_pedal_file(&src).expect("parse tweed_deluxe_5e3");

    // V1a.grid fed from `in` (forward).
    let e1 =
        classify_control_path(&pedal, "V1a.grid", "in").expect("V1a.grid <- in path must resolve");
    assert_eq!(e1.to_class, PortClass::ControlInput, "V1a.grid ControlInput");
    assert!(!e1.is_back_edge, "in -> V1a.grid is forward");
    assert_eq!(
        e1.policy,
        BoundaryPolicy::Tight,
        "forward drive into V1a.grid must be Tight, got {:?}",
        e1.policy
    );

    // V1b.grid fed from the V1a plate (forward cascade).
    let e2 = classify_control_path(&pedal, "V1b.grid", "V1a.plate")
        .expect("V1b.grid <- V1a.plate path must resolve");
    assert_eq!(e2.to_class, PortClass::ControlInput, "V1b.grid ControlInput");
    assert!(!e2.is_back_edge, "V1a.plate -> V1b.grid is forward cascade");
    assert_eq!(
        e2.policy,
        BoundaryPolicy::Tight,
        "forward cascade into V1b.grid must be Tight, got {:?}",
        e2.policy
    );
}

#[test]
fn diode_clipper_has_no_control_inputs_and_no_spurious_sense() {
    // A plain diode clipper has only conducting/rail ports and diode two-ports —
    // no ControlInput electrodes — so NO edge may ever be DelayedSense.
    let src = read_validate_circuit("nonlinear/diode_clipper.pedal");
    let pedal = parse_pedal_file(&src).expect("parse diode_clipper");

    let ports = classify_ports(&pedal);
    assert!(
        ports.iter().all(|p| p.class != PortClass::ControlInput),
        "a passive diode clipper has no control electrodes"
    );

    let edges = classify_edges(&pedal);
    assert!(
        edges
            .iter()
            .all(|e| e.policy != BoundaryPolicy::DelayedSense),
        "no edge in a diode clipper may classify DelayedSense"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// LA-2A transducer-port presence (derivation sanity on the real file)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn la2a_has_exactly_one_light_transducer_port() {
    let src = read_example("outboard/compressor/la2a.pedal");
    let pedal = parse_pedal_file(&src).expect("parse la2a");
    let ports = classify_ports(&pedal);

    let light_ports: Vec<_> = ports
        .iter()
        .filter(|p| p.class == PortClass::Transducer(Domain::Light))
        .collect();
    assert_eq!(
        light_ports.len(),
        1,
        "LA-2A has exactly one optical transducer port (PC1.led), got {:?}",
        light_ports
    );
    assert_eq!(light_ports[0].comp_id, "PC1");
    assert_eq!(light_ports[0].pin, "led");
}

// ═══════════════════════════════════════════════════════════════════════════
// CORPUS SMOKE: classify never panics, yields a class for every pin
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
fn corpus_classification_is_total_and_panic_free() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut files = Vec::new();
    collect_pedal_files(&root.join("examples"), &mut files);
    collect_pedal_files(&root.join("../pedalkernel-validate/circuits"), &mut files);

    assert!(!files.is_empty(), "no .pedal corpus files found — wrong CWD?");

    let mut checked = 0usize;
    let mut total_pins = 0usize;
    let mut empty: Vec<String> = Vec::new();

    for path in &files {
        let src = std::fs::read_to_string(path).unwrap();
        let pedal = match parse_pedal_file(&src) {
            Ok(p) => p,
            // Parse failures are not this gate's concern (exercised elsewhere).
            Err(_) => continue,
        };
        checked += 1;

        // `smoke_classify` runs BOTH classify_ports and classify_edges; if
        // either panics the corpus gate fails here.
        let n = smoke_classify(&pedal);
        total_pins += n;

        // A circuit with at least one component net should classify >0 pins.
        // (Pins absent from the netlist are skipped; an empty result means a
        // total derivation miss, which is the robustness failure we gate.)
        let ports = classify_ports(&pedal);
        if ports.is_empty() && !pedal.components.is_empty() {
            empty.push(path.display().to_string());
        }
    }

    assert!(
        empty.is_empty(),
        "{} circuit(s) yielded ZERO classified ports despite having components:\n{}",
        empty.len(),
        empty.join("\n")
    );

    eprintln!(
        "boundary-rules corpus smoke: {checked} .pedal files, {total_pins} pins classified, no panics"
    );
}
