//! Simple structural checks for compiled stages to ensure nonlinear roots exist.

use pedalkernel::compiler::{compile_pedal, CompiledPedal};
use pedalkernel::dsl::parse_pedal_file;
use std::path::{Path, PathBuf};

fn load_example(filename: &str) -> PathBuf {
    let examples_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
    find_file_recursive(&examples_dir, filename)
        .unwrap_or_else(|| panic!("example file not found: {filename}"))
}

fn find_file_recursive(dir: &Path, filename: &str) -> Option<PathBuf> {
    if let Ok(entries) = std::fs::read_dir(dir) {
        for entry in entries.flatten() {
            let p = entry.path();
            if p.is_dir() {
                if let Some(found) = find_file_recursive(&p, filename) {
                    return Some(found);
                }
            } else if p.file_name().map_or(false, |n| n == filename) {
                return Some(p);
            }
        }
    }
    None
}

fn compile_example(filename: &str) -> CompiledPedal {
    let path = load_example(filename);
    let src = std::fs::read_to_string(&path).unwrap();
    let pedal = parse_pedal_file(&src).expect("parse example pedal");
    compile_pedal(&pedal, 48_000.0).expect("compile example pedal")
}

#[test]
fn rat_contains_diode_stage() {
    let compiled = compile_example("proco_rat.pedal");
    let dump = compiled.debug_dump();
    assert!(
        dump.contains("DiodePair"),
        "RAT should contain a diode stage; stage dump:\n{dump}"
    );
}

#[test]
fn klon_contains_diode_stage() {
    let compiled = compile_example("klon_centaur.pedal");
    let dump = compiled.debug_dump();
    assert!(
        dump.contains("DiodePair"),
        "Klon should contain a diode stage; stage dump:\n{dump}"
    );
}

#[test]
fn memory_man_controls_bound_correctly() {
    let compiled = compile_example("memory_man.pedal");
    let dump = compiled.debug_dump();
    // Delay pot should control LFO rate (which modulates BBD clock)
    assert!(
        dump.contains("Delay -> LfoRate(0)"),
        "Memory Man Delay pot should be bound to LfoRate; controls:\n{dump}"
    );
    // Feedback pot should control BBD feedback
    assert!(
        dump.contains("Feedback -> BbdFeedback(0)"),
        "Memory Man Feedback pot should be bound to BbdFeedback; controls:\n{dump}"
    );
    // Blend pot should control BBD wet/dry mix
    assert!(
        dump.contains("Blend -> BbdMix"),
        "Memory Man Blend pot should be bound to BbdMix; controls:\n{dump}"
    );
}

#[test]
fn dm2_controls_bound_correctly() {
    let compiled = compile_example("boss_dm2.pedal");
    let dump = compiled.debug_dump();
    // Time pot directly drives BBD clock
    assert!(
        dump.contains("Time -> BbdClockRate(0)"),
        "DM-2 Time pot should be bound to BbdClockRate; controls:\n{dump}"
    );
    // Repeats pot should control BBD feedback
    assert!(
        dump.contains("Repeats -> BbdFeedback(0)"),
        "DM-2 Repeats pot should be bound to BbdFeedback; controls:\n{dump}"
    );
    // Mix pot should control BBD wet/dry mix
    assert!(
        dump.contains("Mix -> BbdMix"),
        "DM-2 Mix pot should be bound to BbdMix; controls:\n{dump}"
    );
}

#[test]
fn ce2_rate_controls_lfo() {
    let compiled = compile_example("boss_ce2.pedal");
    let dump = compiled.debug_dump();
    // Rate pot should control LFO rate
    assert!(
        dump.contains("Rate -> LfoRate(0)"),
        "CE-2 Rate pot should be bound to LfoRate; controls:\n{dump}"
    );
}
