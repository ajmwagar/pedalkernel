//! Engine determinism: reset/rewind semantics and sample-rate invariance.

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::compiler::{compile_pedal, CompileOptions};
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::oversampling::OversamplingFactor;
use pedalkernel::PedalProcessor;
use std::path::Path;

#[test]
fn clip_silicon_reset_reproduces_output() {
    let src = read_test_pedal("clip_silicon.pedal");
    let pedal = parse_pedal_file(&src).expect("parse clip pedal");
    let mut proc = compile_pedal(&pedal, SAMPLE_RATE).expect("compile clip");

    let input = sine_at(660.0, 0.4, 0.1, SAMPLE_RATE);

    let first_run: Vec<f64> = input.iter().map(|&x| proc.process(x)).collect();
    proc.reset();
    let after_reset: Vec<f64> = input.iter().map(|&x| proc.process(x)).collect();

    assert_eq!(first_run.len(), after_reset.len());

    let diff: Vec<f64> = first_run
        .iter()
        .zip(&after_reset)
        .map(|(a, b)| a - b)
        .collect();
    let diff_rms = rms(&diff);
    assert!(
        diff_rms < 1e-10,
        "Reset should reproduce first run: diff_rms={diff_rms:e}"
    );
}

#[test]
fn triode_stage_is_sample_rate_consistent() {
    let duration = 0.2;
    let input48 = sine_at(523.25, 0.25, duration, 48_000.0);
    let input96 = sine_at(523.25, 0.25, duration, 96_000.0);

    // Use X1 oversampling so we compare raw sample rates, not oversampled rates
    let opts_48 = CompileOptions {
        oversampling: OversamplingFactor::X1,
        ..CompileOptions::default()
    };
    let opts_96 = CompileOptions {
        oversampling: OversamplingFactor::X1,
        ..CompileOptions::default()
    };
    let out48 =
        compile_test_pedal_with_options("triode_clean.pedal", &input48, 48_000.0, &[], opts_48);
    let out96 =
        compile_test_pedal_with_options("triode_clean.pedal", &input96, 96_000.0, &[], opts_96);
    let out96_down = decimate(&out96, 2);

    assert_eq!(out48.len(), out96_down.len());
    assert!(
        out48.iter().all(|x| x.is_finite()),
        "triode 48k output contains NaN/Inf"
    );
    assert!(
        out96_down.iter().all(|x| x.is_finite()),
        "triode 96k output contains NaN/Inf"
    );

    let corr = correlation(&out48, &out96_down).abs();
    assert!(
        corr > 0.985,
        "Downsampled 96k output should correlate: corr={corr:.6}"
    );

    let diff: Vec<f64> = out48.iter().zip(&out96_down).map(|(a, b)| a - b).collect();
    let diff_rms = rms(&diff);
    assert!(
        diff_rms < 0.01,
        "Sample-rate mismatch too large: rms={diff_rms}"
    );
}

/// Compile dyna_comp and fingerprint its output for a level-stepped sine,
/// hashing the exact bit pattern of every sample (FNV-1a) so any numeric
/// divergence between compiles is caught.
fn dyna_comp_fingerprint() -> u64 {
    let path =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/pedals/compressor/dyna_comp.pedal");
    let src = std::fs::read_to_string(&path).expect("read dyna_comp.pedal");
    let pedal = parse_pedal_file(&src).expect("parse dyna_comp");
    let mut proc = compile_pedal(&pedal, SAMPLE_RATE).expect("compile dyna_comp");

    // 0.5 s of 220 Hz: quiet (0.05) for the first half, loud (0.4) after,
    // so both the linear region and the compressing region are exercised.
    let n = (0.5 * SAMPLE_RATE) as usize;
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325; // FNV-1a offset basis
    for i in 0..n {
        let t = i as f64 / SAMPLE_RATE;
        let amp = if t < 0.25 { 0.05 } else { 0.4 };
        let x = amp * (2.0 * std::f64::consts::PI * 220.0 * t).sin();
        let y = proc.process(x);
        for byte in y.to_bits().to_le_bytes() {
            hash ^= byte as u64;
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3); // FNV-1a prime
        }
    }
    hash
}

#[test]
fn dyna_comp_recompile_is_deterministic_in_process() {
    let fingerprints: Vec<u64> = (0..8).map(|_| dyna_comp_fingerprint()).collect();
    let first = fingerprints[0];
    assert!(
        fingerprints.iter().all(|&f| f == first),
        "dyna_comp recompiles diverge within one process: {fingerprints:016x?}"
    );
}

#[test]
fn dyna_comp_compile_is_deterministic_across_processes() {
    const CHILD_ENV: &str = "PK_DETERMINISM_CHILD";
    const MARKER: &str = "FINGERPRINT=";

    if std::env::var(CHILD_ENV).is_ok() {
        // Child: compile once, report the fingerprint, and return.
        println!("{MARKER}{:016x}", dyna_comp_fingerprint());
        return;
    }

    // Parent: spawn 8 fresh processes and require a single distinct fingerprint.
    let exe = std::env::current_exe().expect("current_exe");
    let fingerprints: Vec<u64> = (0..8)
        .map(|i| {
            let out = std::process::Command::new(&exe)
                .args([
                    "dyna_comp_compile_is_deterministic_across_processes",
                    "--exact",
                    "--nocapture",
                ])
                .env(CHILD_ENV, "1")
                .output()
                .expect("failed to spawn determinism child");
            let stdout = String::from_utf8_lossy(&out.stdout);
            stdout
                .lines()
                .find_map(|l| l.trim().strip_prefix(MARKER))
                .and_then(|v| u64::from_str_radix(v, 16).ok())
                .unwrap_or_else(|| {
                    panic!(
                        "child {i}: no fingerprint marker in stdout:\n{stdout}\nstderr:\n{}",
                        String::from_utf8_lossy(&out.stderr)
                    )
                })
        })
        .collect();

    let mut distinct = fingerprints.clone();
    distinct.sort_unstable();
    distinct.dedup();
    assert!(
        distinct.len() == 1,
        "dyna_comp compiles nondeterministically across processes: \
         {} distinct fingerprints over 8 children: {fingerprints:016x?}",
        distinct.len()
    );
}

/// Compile the cem3340_vco generator and fingerprint 0.25 s of its output
/// (FNV-1a over every sample's bit pattern). A VCO is the first generator
/// DspBlock; this guards that the N=0 source path + PolyBLEP body recompile
/// bit-identically (spec §8 / §9 determinism).
fn cem3340_vco_fingerprint() -> u64 {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/synths/cem3340_vco.pedal");
    let src = std::fs::read_to_string(&path).expect("read cem3340_vco.pedal");
    let pedal = parse_pedal_file(&src).expect("parse cem3340_vco");
    let mut proc = compile_pedal(&pedal, SAMPLE_RATE).expect("compile cem3340_vco");

    let n = (0.25 * SAMPLE_RATE) as usize;
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325; // FNV-1a offset basis
    for _ in 0..n {
        let y = proc.process(0.0);
        for byte in y.to_bits().to_le_bytes() {
            hash ^= byte as u64;
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3); // FNV-1a prime
        }
    }
    hash
}

#[test]
fn cem3340_vco_recompile_is_deterministic_in_process() {
    let fingerprints: Vec<u64> = (0..8).map(|_| cem3340_vco_fingerprint()).collect();
    let first = fingerprints[0];
    assert!(
        fingerprints.iter().all(|&f| f == first),
        "cem3340_vco recompiles diverge within one process: {fingerprints:016x?}"
    );
}

fn decimate(buf: &[f64], factor: usize) -> Vec<f64> {
    buf.iter().step_by(factor).copied().collect()
}

fn read_test_pedal(filename: &str) -> String {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/test_pedals")
        .join(filename);
    std::fs::read_to_string(&path).expect("read test pedal")
}
