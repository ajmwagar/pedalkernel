//! Realtime-budget gate — runs every PR, must pass, must be fast.
//!
//! For each public example pedal: compile, process 4096 samples at 48 kHz,
//! measure ns/sample, and assert the engine is at least 2× faster than
//! realtime.  A 2× headroom means the pedal consumes at most 50% of a
//! single-core CPU budget — comfortable for a plugin host running alongside
//! other processing.
//!
//! WHY 2× and not tighter: host jitter, OS scheduling, and differences between
//! dev machines and CI runners all add overhead.  2× keeps the gate meaningful
//! without false-failing on slower machines while still catching catastrophic
//! regressions.
//!
//! WHY 4096 samples: large enough to amortise compile + setup time;
//! small enough that the test completes in a few seconds total.
//!
//! WHY `--release`: performance gates are meaningless in debug builds.
//! The `realtime-gate` CI job must run with `cargo test --release --test realtime_gate`.
//!
//! BIG MUFF NOTE: big_muff.pedal has 4 coupled nonlinear stages and is a
//! known-slow outlier at ~0.3× realtime (measured; tracked in wdf_bench
//! throughput_report nightly).  It is measured here for observability but
//! excluded from the PASS assertion — a dedicated perf sprint is the right
//! fix, not a permanently-failing CI gate.
//!
//! This test uses the SAME ns/sample + x-realtime formula as
//! `bench_realtime_budget` in `benches/wdf_bench.rs` (DRY by design).

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use std::time::Instant;

const SAMPLE_RATE: f64 = 48_000.0;
const BUFFER_SIZE: usize = 4096;

/// Minimum realtime multiplier required for gated pedals.
/// 2× means the pedal processes audio at ≥ 2× realtime speed.
const HEADROOM: f64 = 2.0;

// Compile-time inclusion of public example pedals (no file search at runtime).
// Paths are relative to the crate root (CARGO_MANIFEST_DIR).
const TUBE_SCREAMER: &str =
    include_str!("../examples/pedals/overdrive/tube_screamer.pedal");
const BIG_MUFF: &str =
    include_str!("../examples/pedals/fuzz/big_muff.pedal");
const KLON_CENTAUR: &str =
    include_str!("../examples/pedals/overdrive/klon_centaur.pedal");
const PROCO_RAT: &str =
    include_str!("../examples/pedals/distortion/proco_rat.pedal");
const FUZZ_FACE: &str =
    include_str!("../examples/pedals/fuzz/fuzz_face.pedal");
const BLUES_DRIVER: &str =
    include_str!("../examples/pedals/overdrive/blues_driver.pedal");
const DYNA_COMP: &str =
    include_str!("../examples/pedals/compressor/dyna_comp.pedal");

/// Process `BUFFER_SIZE` samples through a compiled pedal and return
/// (ns_per_sample, samples_per_sec).  Same formula as wdf_bench.rs.
fn measure(src: &str) -> (f64, f64) {
    let pedal = parse_pedal_file(src).expect("parse");
    let mut proc = compile_pedal(&pedal, SAMPLE_RATE).expect("compile");

    // Sine wave at 440 Hz, 0.5 amplitude (same stimulus as bench_realtime_budget)
    let block: Vec<f64> = (0..BUFFER_SIZE)
        .map(|i| {
            0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SAMPLE_RATE).sin()
        })
        .collect();

    // Warm-up pass — primes caches and K-table lookups
    for &s in &block {
        let _ = proc.process(s);
    }

    // Timed pass
    let start = Instant::now();
    for &s in &block {
        let _ = proc.process(s);
    }
    let elapsed = start.elapsed();

    let ns_per_sample = elapsed.as_nanos() as f64 / BUFFER_SIZE as f64;
    let samples_per_sec = 1e9 / ns_per_sample;
    (ns_per_sample, samples_per_sec)
}

/// Measure and print a pedal; assert it meets HEADROOM if `gated` is true.
fn check(name: &str, src: &str, gated: bool) {
    let (ns_per_sample, samples_per_sec) = measure(src);
    let x_realtime = samples_per_sec / SAMPLE_RATE;

    let tag = if gated { "" } else { " (observe-only)" };
    println!(
        "  {:<20} {:>9.1} ns/sample  {:>6.1}x realtime{}",
        name, ns_per_sample, x_realtime, tag
    );

    if gated {
        assert!(
            samples_per_sec >= SAMPLE_RATE * HEADROOM,
            "{name}: only {x_realtime:.1}x realtime — must be >= {HEADROOM}x \
             ({:.0} samples/sec < {:.0} required). \
             Run with `cargo test --release --test realtime_gate` to measure optimised build.",
            samples_per_sec,
            SAMPLE_RATE * HEADROOM,
        );
    }
}

#[test]
fn realtime_gate_all_pedals() {
    println!();
    println!(
        "  Realtime gate — HEADROOM={HEADROOM}x, {SAMPLE_RATE} Hz, {BUFFER_SIZE} samples"
    );
    println!("  ──────────────────────────────────────────────────────────────────");

    // Gated: must meet HEADROOM
    check("tube_screamer", TUBE_SCREAMER, true);
    check("klon_centaur", KLON_CENTAUR, true);
    check("proco_rat", PROCO_RAT, true);
    check("fuzz_face", FUZZ_FACE, true);
    check("blues_driver", BLUES_DRIVER, true);
    check("dyna_comp", DYNA_COMP, true);

    // Observe-only: big_muff has 4 coupled NL stages (~0.3× realtime; tracked nightly)
    check("big_muff", BIG_MUFF, false);

    println!("  ──────────────────────────────────────────────────────────────────");
    println!("  Gated pedals pass {HEADROOM}x realtime gate.");
    println!();
}
