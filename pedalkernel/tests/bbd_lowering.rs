//! BBD lowering tests (bug-hunt census #1).
//!
//! `bbd()` components must be lowered into `CompiledPedal.bbds` so the
//! runtime BBD wet/dry path (processor.rs `process()`) actually runs.
//! Before the fix, `bbds` was always empty and every BBD pedal compiled
//! to exact silence (the Behavioral edge splits the circuit graph and
//! nothing bridged the gap).

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::compiler::{compile_pedal, CompiledPedal};
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use std::path::Path;

fn compile_test_pedal(filename: &str) -> CompiledPedal {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/test_pedals")
        .join(filename);
    let src = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("failed to read {}: {e}", path.display()));
    let pedal = parse_pedal_file(&src).expect("parse failed");
    compile_pedal(&pedal, SAMPLE_RATE).expect("compile failed")
}

fn compile_example(filename: &str) -> CompiledPedal {
    let examples_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
    let path = find_recursive(&examples_dir, filename)
        .unwrap_or_else(|| panic!("example not found: {filename}"));
    let src = std::fs::read_to_string(&path).unwrap();
    let pedal = parse_pedal_file(&src).expect("parse failed");
    compile_pedal(&pedal, SAMPLE_RATE).expect("compile failed")
}

fn find_recursive(dir: &Path, filename: &str) -> Option<std::path::PathBuf> {
    for entry in std::fs::read_dir(dir).ok()?.flatten() {
        let p = entry.path();
        if p.is_dir() {
            if let Some(found) = find_recursive(&p, filename) {
                return Some(found);
            }
        } else if p.file_name().is_some_and(|n| n == filename) {
            return Some(p);
        }
    }
    None
}

fn process_all(proc: &mut CompiledPedal, input: &[f64]) -> Vec<f64> {
    input.iter().map(|&s| proc.process(s)).collect()
}

/// A short 440 Hz burst followed by silence — for measuring delayed energy.
fn burst_then_silence(burst_secs: f64, silence_secs: f64) -> Vec<f64> {
    let mut input = sine(440.0, burst_secs, SAMPLE_RATE);
    input.extend(vec![0.0; (silence_secs * SAMPLE_RATE) as usize]);
    input
}

// ---------------------------------------------------------------------------
// (1) Smallest red fixture: bbd_chorus.pedal
// ---------------------------------------------------------------------------

#[test]
fn bbd_chorus_lowers_bbds() {
    let compiled = compile_test_pedal("bbd_chorus.pedal");
    assert_eq!(
        compiled.bbds.len(),
        1,
        "bbd_chorus.pedal declares one bbd(mn3207); CompiledPedal.bbds must \
         contain one BbdDelayLine (was always empty before lowering)"
    );
    assert_eq!(
        compiled.bbds[0].model.num_stages, 1024,
        "mn3207 is a 1024-stage device"
    );
}

#[test]
fn bbd_chorus_produces_nonsilent_output() {
    let mut compiled = compile_test_pedal("bbd_chorus.pedal");
    let input = sine(440.0, 0.5, SAMPLE_RATE);
    let output = process_all(&mut compiled, &input);

    assert!(output.iter().all(|x| x.is_finite()), "NaN/inf in output");
    let p = peak(&output);
    eprintln!("bbd_chorus: peak={p:.6} rms={:.6}", rms(&output));
    assert!(
        p > 1e-4,
        "bbd_chorus must produce audio (was exact silence before BBD lowering): peak={p:.8}"
    );
}

// ---------------------------------------------------------------------------
// (2) boss_dm2: output exists AND contains a genuinely delayed component
// ---------------------------------------------------------------------------

#[test]
fn boss_dm2_produces_delayed_output() {
    let mut compiled = compile_example("boss_dm2.pedal");
    assert!(
        !compiled.bbds.is_empty(),
        "boss_dm2 declares BBD1: bbd(mn3005) — bbds must be lowered"
    );

    // 100 ms burst, then 1 s of silence.
    let burst_secs = 0.1;
    let input = burst_then_silence(burst_secs, 1.0);
    let output = process_all(&mut compiled, &input);
    assert!(output.iter().all(|x| x.is_finite()), "NaN/inf in output");

    let burst_end = (burst_secs * SAMPLE_RATE) as usize;
    let p = peak(&output);
    assert!(p > 1e-4, "boss_dm2 must produce audio: peak={p:.8}");

    // Any dry/leakage path dies with the input (give it 10 ms of grace).
    // A real delay line stores the burst and replays it later: there must be
    // energy in the 20–200 ms post-burst window well above the silent floor.
    let grace = (0.010 * SAMPLE_RATE) as usize;
    let win_lo = burst_end + (0.020 * SAMPLE_RATE) as usize;
    let win_hi = burst_end + (0.200 * SAMPLE_RATE) as usize;
    let delayed_rms = rms(&output[win_lo..win_hi]);

    // Silent baseline: the very end of the buffer, long after repeats decay
    // (still nonzero — BBDs inject noise — so compare against it with margin).
    let tail_rms = rms(&output[output.len() - 4800..]);

    // Measured delay characteristics: first post-grace sample where the
    // envelope rises above 10x the tail floor.
    let env = rms_envelope(&output, 0.002, SAMPLE_RATE);
    let onset = (burst_end + grace..output.len())
        .find(|&i| env[i] > (10.0 * tail_rms).max(1e-6))
        .map(|i| (i - burst_end) as f64 / SAMPLE_RATE * 1000.0);
    eprintln!(
        "boss_dm2: peak={p:.5} delayed_rms(20-200ms)={delayed_rms:.6} \
         tail_rms={tail_rms:.6} first-echo-onset={onset:?} ms"
    );

    assert!(
        delayed_rms > 1e-5,
        "boss_dm2: no delayed energy in 20-200 ms post-burst window \
         (delayed_rms={delayed_rms:.8}) — BBD wet path not running"
    );
    assert!(
        delayed_rms > 3.0 * tail_rms,
        "boss_dm2: post-burst window ({delayed_rms:.8}) should clearly exceed \
         the silent tail floor ({tail_rms:.8})"
    );
}

// ---------------------------------------------------------------------------
// (3) ehx_memory_man: wet path vs dry-leak baseline
// ---------------------------------------------------------------------------

#[test]
fn ehx_memory_man_wet_path_carries_energy() {
    let mut compiled = compile_test_pedal("ehx_memory_man.pedal");
    assert!(
        !compiled.bbds.is_empty(),
        "ehx_memory_man declares BBD1: bbd(mn3005) — bbds must be lowered"
    );
    eprintln!(
        "ehx_memory_man: bbds={} wet_mix={}",
        compiled.bbds.len(),
        compiled.bbd_wet_mix
    );

    // Steady-state spectrum/level with the BBD wet path active.
    let input = sine(440.0, 1.0, SAMPLE_RATE);
    let wet_out = process_all(&mut compiled, &input);
    assert!(wet_out.iter().all(|x| x.is_finite()), "NaN/inf in output");
    let wet_rms = rms(&wet_out[9600..]);
    let wet_440 = goertzel_power(&wet_out[9600..], SAMPLE_RATE, 440.0);

    // Dry-leak baseline: same circuit with the wet path muted.
    let mut dry_proc = compile_test_pedal("ehx_memory_man.pedal");
    dry_proc.bbd_wet_mix = 0.0;
    let dry_out = process_all(&mut dry_proc, &input);
    let dry_rms = rms(&dry_out[9600..]);
    let dry_440 = goertzel_power(&dry_out[9600..], SAMPLE_RATE, 440.0);

    eprintln!(
        "ehx_memory_man: wet rms={wet_rms:.6} p440={wet_440:.3e} | \
         dry-only rms={dry_rms:.6} p440={dry_440:.3e}"
    );

    // Conservative: the pedal must produce signal, and mixing in the BBD wet
    // path must actually change the output relative to the dry-leak baseline.
    assert!(
        wet_rms > 1e-5,
        "ehx_memory_man should produce signal: rms={wet_rms:.8}"
    );
    let diff: f64 = wet_out
        .iter()
        .zip(&dry_out)
        .map(|(w, d)| (w - d) * (w - d))
        .sum::<f64>()
        / wet_out.len() as f64;
    let diff_rms = diff.sqrt();
    eprintln!("ehx_memory_man: wet-vs-dry diff rms={diff_rms:.6}");
    assert!(
        diff_rms > 1e-6,
        "BBD wet mix must contribute to the output (diff_rms={diff_rms:.8} vs dry-leak baseline)"
    );
}

// ---------------------------------------------------------------------------
// Census coverage: remaining BBD pedals are lowered and audible
// ---------------------------------------------------------------------------

#[test]
fn all_census_bbd_pedals_lower_and_produce_output() {
    let cases: [(&str, bool); 4] = [
        ("boss_dm2.pedal", false),
        ("memory_man.pedal", false),
        ("boss_ce2.pedal", false),
        ("walrus_slo.pedal", true),
    ];
    for (name, is_test_pedal) in cases {
        let mut compiled = if is_test_pedal {
            compile_test_pedal(name)
        } else {
            compile_example(name)
        };
        assert!(!compiled.bbds.is_empty(), "{name}: bbds not lowered");
        let input = sine(440.0, 0.5, SAMPLE_RATE);
        let output = process_all(&mut compiled, &input);
        assert!(output.iter().all(|x| x.is_finite()), "{name}: NaN/inf");
        let p = peak(&output);
        eprintln!("{name}: bbds={} peak={p:.6}", compiled.bbds.len());
        assert!(p > 1e-4, "{name}: silent output (peak={p:.8})");
    }
}
