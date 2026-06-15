//! Regression gate for the SPQR all-passive-drop bug.
//!
//! BUG (fixed): an all-passive component group that reduced to a rigid
//! (non-series-parallel) R-node was SILENTLY DROPPED by SPQR lowering —
//! `spqr_to_dyn_node` returns `None` for an R-node, the `AllPassive` arm of
//! `collect_stages` warn-and-dropped the whole stage, and every pot control
//! declared inside that stage (e.g. tape `Tone`, blues_driver `Tone`+`Level`,
//! space_echo EQ) vanished. The symptom was a unity passthrough with dead
//! knobs. The fix (spqr_build.rs) rebuilds such groups as WDF PassiveRType
//! stages with terminal-derived ports and live pot leaves.
//!
//! This gate compiles EVERY `examples/**/*.pedal` and asserts that every
//! declared `*.position` (potentiometer) control binds in the compiled
//! processor. A dropped all-passive stage is exactly what un-binds a pot
//! control, so this locks the fix against regression across the whole example
//! corpus. (Non-pot controls — delay time/feedback, LFO rate, etc. — are not
//! checked here: they bind through other targets and are out of scope.)

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use std::path::{Path, PathBuf};

const SAMPLE_RATE: f64 = 48000.0;

/// KNOWN PRE-EXISTING unbound pot controls, present on the clean baseline
/// BEFORE the all-passive-drop fix and NOT revived by it (they reach a
/// different lowering path — the pot is absorbed into / straddles a nonlinear
/// or active group, not the all-passive-produced-zero-stages case this fix
/// targets). They are out of scope for the narrow SPQR fix and are tracked
/// separately. The fix's job is to ensure the gate's *non-allowlisted* set
/// only shrinks; this allowlist must NEVER grow.
///   (relative example path, control label)
const KNOWN_UNBOUND: &[(&str, &str)] = &[
    // 3-terminal opto/grid wiper straddling the GR + tube-grid stages
    // (same family as the CLAUDE.md LA-2A "Gain pot frozen" topology gap).
    ("outboard/compressor/la2a.pedal", "Limit/Compress"),
    // Fuzz Face: the Fuzz pot is part of the cross-coupled BJT feedback
    // group, absorbed into the nonlinear stage rather than an all-passive R.
    ("pedals/fuzz/fuzz_face.pedal", "Fuzz"),
    // Klon Centaur: Treble tone control reaches the active gain-recovery
    // network, not an all-passive R-node.
    ("pedals/overdrive/klon_centaur.pedal", "Treble"),
];

fn collect_pedals(dir: &Path, out: &mut Vec<PathBuf>) {
    if let Ok(entries) = std::fs::read_dir(dir) {
        for entry in entries.flatten() {
            let p = entry.path();
            if p.is_dir() {
                collect_pedals(&p, out);
            } else if p.extension().map_or(false, |e| e == "pedal") {
                out.push(p);
            }
        }
    }
}

#[test]
fn every_declared_pot_control_binds_in_all_examples() {
    let examples_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
    let mut pedals = Vec::new();
    collect_pedals(&examples_dir, &mut pedals);
    pedals.sort();
    assert!(
        !pedals.is_empty(),
        "no example .pedal files found under {}",
        examples_dir.display()
    );

    let mut failures: Vec<String> = Vec::new();
    let mut checked_controls = 0usize;

    for path in &pedals {
        let rel = path
            .strip_prefix(&examples_dir)
            .unwrap_or(path)
            .to_string_lossy()
            .to_string();
        let src = match std::fs::read_to_string(path) {
            Ok(s) => s,
            Err(e) => {
                failures.push(format!("{rel}: read failed: {e}"));
                continue;
            }
        };
        let pedal = match parse_pedal_file(&src) {
            Ok(p) => p,
            Err(e) => {
                // Parse errors are not this gate's concern, but record them so
                // a corpus regression is visible.
                failures.push(format!("{rel}: parse failed: {e}"));
                continue;
            }
        };

        // Declared pot controls: a `<Pot>.position -> "Label"` mapping.
        let declared_pots: Vec<&str> = pedal
            .controls
            .iter()
            .filter(|c| c.property.eq_ignore_ascii_case("position"))
            .map(|c| c.label.as_str())
            .collect();
        if declared_pots.is_empty() {
            continue;
        }

        let proc = match compile_pedal(&pedal, SAMPLE_RATE) {
            Ok(p) => p,
            Err(e) => {
                failures.push(format!("{rel}: compile failed: {e}"));
                continue;
            }
        };
        let bound: Vec<String> = proc.controls.iter().map(|c| c.label.clone()).collect();

        for label in declared_pots {
            checked_controls += 1;
            let is_bound = bound.iter().any(|b| b.eq_ignore_ascii_case(label));
            let is_known = KNOWN_UNBOUND
                .iter()
                .any(|(p, l)| *p == rel && l.eq_ignore_ascii_case(label));
            if !is_bound && !is_known {
                failures.push(format!(
                    "{rel}: declared pot control '{label}' is UNBOUND (all-passive \
                     stage dropped?). bound controls = {bound:?}"
                ));
            }
            if is_bound && is_known {
                // The allowlist must stay honest: if a previously-broken
                // control starts binding, REMOVE it from KNOWN_UNBOUND.
                failures.push(format!(
                    "{rel}: control '{label}' is in KNOWN_UNBOUND but now BINDS — \
                     remove it from the allowlist"
                ));
            }
        }
    }

    eprintln!(
        "all_passive_drop_gate: scanned {} example pedals, checked {} declared pot controls",
        pedals.len(),
        checked_controls
    );

    assert!(
        failures.is_empty(),
        "SPQR all-passive-drop regression — declared pot controls failed to bind:\n  {}",
        failures.join("\n  ")
    );
}
