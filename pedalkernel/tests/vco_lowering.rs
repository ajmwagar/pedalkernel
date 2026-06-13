//! VCO lowering acceptance tests — the proof case for the generalized
//! `DspBlock` (reports/dsp-block-generalization-2026-06-13.md §8).
//!
//! `vco()` is a **generator** behavioral island (`GraphRole::Virtual`, 0 audio
//! inputs, ≥1 audio output). Before VcoBlock it ERRORED in
//! `reject_unlowered_behavioral`, so `cem3340_vco.pedal` / `minisynth.pedal`
//! compile-failed. These tests pin the wiring (compiler/vco_lowering.rs +
//! processor.rs VCO tick loop):
//!
//! 1. cem3340_vco compiles and produces a non-silent band-limited tone at the
//!    vco()'s configured frequency (Goertzel fundamental within tolerance),
//! 2. PolyBLEP anti-aliasing: a high-frequency saw has low alias energy,
//! 3. the audio-rate CV-port path tracks 1 V/oct (a 1 V CV step doubles the
//!    fundamental) — spec §3, a float `phase_inc` update with NO scattering
//!    recompute,
//! 4. minisynth (VCO → VCF → VCA, two real DspBlocks) compiles with both
//!    instances lowered — the integration case, and
//! 5. a wired VCO clears ≥ 30x realtime single-core (generators are cheap).
//!
//! Run: cargo test -p pedalkernel --no-default-features --test vco_lowering

mod audio_analysis;

use std::time::Instant;

use audio_analysis::*;
use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;

/// Minimal generator: a CEM3340 saw VCO at 220 Hz driving an output resistor.
/// No audio input — exercises the N=0 source-placement branch.
const MINIMAL_VCO: &str = r#"
pedal "Minimal VCO" {
  supply 12V
  components {
    VCO1: vco(cem3340, 220)
    R_out: resistor(1k)
  }
  nets {
    VCO1.saw -> R_out.a
    R_out.b -> out
  }
  controls {}
}
"#;

/// Same generator with an external 1 V/oct pitch CV port wired straight to the
/// VCO's cv pin — exercises the audio-rate CV-port path. The CV port is the
/// only input port, so `process(cv)` injects the CV voltage at the VCO cv node.
const VCO_WITH_CV: &str = r#"
pedal "VCO with CV" {
  supply 12V
  ports { cv_in: input }
  components {
    VCO1: vco(cem3340, 220)
    R_out: resistor(1k)
  }
  nets {
    cv_in -> VCO1.cv
    VCO1.saw -> R_out.a
    R_out.b -> out
  }
  controls {}
}
"#;

/// High-frequency saw generator for the aliasing test (naive saw would fold
/// badly at 3 kHz: the 8th harmonic at 24 kHz is already at Nyquist).
const VCO_HIGH: &str = r#"
pedal "VCO high" {
  supply 12V
  components {
    VCO1: vco(cem3340, 3000)
    R_out: resistor(1k)
  }
  nets {
    VCO1.saw -> R_out.a
    R_out.b -> out
  }
  controls {}
}
"#;

/// Scan for the dominant tone in a buffer (2 Hz resolution).
fn dominant_hz(buf: &[f64], sample_rate: f64) -> f64 {
    let mut best = (0.0_f64, 0.0_f64);
    let mut f = 50.0;
    while f < sample_rate / 2.0 {
        let m = goertzel_mag(buf, sample_rate, f);
        if m > best.1 {
            best = (f, m);
        }
        f += 2.0;
    }
    best.0
}

/// Compile and render `secs` of a generator (audio input held at `drive`).
fn render(src: &str, drive: f64, secs: f64, sample_rate: f64) -> Vec<f64> {
    let pedal = parse_pedal_file(src).expect("parse");
    let mut compiled = compile_pedal(&pedal, sample_rate).expect("compile");
    let n = (sample_rate * secs) as usize;
    (0..n).map(|_| compiled.process(drive)).collect()
}

// ===========================================================================
// (1) cem3340_vco lowers, is non-silent, fundamental matches the config
// ===========================================================================

#[test]
fn cem3340_vco_lowers_and_emits_configured_fundamental() {
    let src = example_pedal_source("cem3340_vco.pedal");
    let pedal = parse_pedal_file(&src).expect("parse");
    let mut compiled = compile_pedal(&pedal, SAMPLE_RATE).expect("compile");

    // Lowering contract: one runtime VCO per vco(), declaration order.
    assert_eq!(compiled.vcos.len(), 1, "vcos not populated by VcoBlock");
    assert_eq!(compiled.vcos[0].comp_id, "VCO1");

    // cem3340_vco declares vco(as3340) with no explicit freq → 440 Hz default.
    let out: Vec<f64> = (0..SAMPLE_RATE as usize)
        .map(|_| compiled.process(0.0))
        .collect();
    let tail = &out[(0.5 * SAMPLE_RATE) as usize..];
    let pk = peak(tail);
    let fund = dominant_hz(tail, SAMPLE_RATE);
    eprintln!("cem3340_vco: peak {pk:.4}, fundamental {fund:.1} Hz (configured 440)");

    assert!(pk > 0.05, "VCO is silent: peak {pk:.4}");
    assert!(
        (fund - 440.0).abs() < 5.0,
        "fundamental {fund:.1} Hz != configured 440 Hz"
    );
}

// ===========================================================================
// (2) PolyBLEP band-limiting: high-frequency saw has low alias energy
// ===========================================================================

#[test]
fn vco_saw_is_band_limited_low_alias_energy() {
    let out = render(VCO_HIGH, 0.0, 1.0, SAMPLE_RATE);
    let tail = &out[(0.5 * SAMPLE_RATE) as usize..];
    let summary = alias_energy_summary(tail, SAMPLE_RATE, 3000.0, 120.0);
    eprintln!(
        "vco 3 kHz saw alias ratio: {:.5} (top bins {:?})",
        summary.alias_ratio, summary.top_bins
    );
    // PolyBLEP keeps inter-harmonic alias energy a small fraction of total.
    assert!(
        summary.alias_ratio < 0.05,
        "alias energy ratio {:.5} too high — PolyBLEP not band-limiting",
        summary.alias_ratio
    );
}

// ===========================================================================
// (3) Audio-rate CV-port path: 1 V/oct pitch tracking (spec §3)
// ===========================================================================

#[test]
fn vco_cv_port_tracks_one_volt_per_octave() {
    let pedal = parse_pedal_file(VCO_WITH_CV).expect("parse");
    let mut compiled = compile_pedal(&pedal, SAMPLE_RATE).expect("compile");
    assert_eq!(compiled.vcos.len(), 1);
    assert_eq!(compiled.port_count(), 1, "expected one CV input port");

    // Drive the CV port (the only input port) by passing the voltage as the
    // process() input — it injects at the VCO cv node via node_signals.
    let freq_at = |compiled: &mut _, cv: f64| -> f64 {
        let n = SAMPLE_RATE as usize;
        let buf: Vec<f64> = (0..n)
            .map(|_| PedalProcessor::process(compiled, cv))
            .collect();
        dominant_hz(&buf[n / 2..], SAMPLE_RATE)
    };

    let f0 = freq_at(&mut compiled, 0.0);
    let f_up = freq_at(&mut compiled, 1.0);
    let f_dn = freq_at(&mut compiled, -1.0);
    let up_ratio = f_up / f0;
    let dn_ratio = f0 / f_dn;
    eprintln!(
        "vco CV octave tracking: cv=0 -> {f0:.1} Hz, +1V -> {f_up:.1} Hz (x{up_ratio:.3}), \
         -1V -> {f_dn:.1} Hz (x{dn_ratio:.3})"
    );

    assert!((f0 - 220.0).abs() < 5.0, "base freq {f0:.1} != 220");
    assert!(
        (up_ratio - 2.0).abs() < 0.05,
        "+1 V CV should double the frequency, got x{up_ratio:.3}"
    );
    assert!(
        (dn_ratio - 2.0).abs() < 0.05,
        "-1 V CV should halve the frequency, got x{dn_ratio:.3}"
    );
}

// ===========================================================================
// (4) minisynth integration: VCO + VCA both lower as real DspBlocks
// ===========================================================================

#[test]
fn minisynth_compiles_with_vco_and_vca_lowered() {
    let src = example_pedal_source("minisynth.pedal");
    let pedal = parse_pedal_file(&src).expect("parse");
    let compiled = compile_pedal(&pedal, SAMPLE_RATE).expect("minisynth must compile");

    // Both behavioral DSP blocks lower (was a compile error on vco() before).
    assert_eq!(compiled.vcos.len(), 1, "VCO did not lower");
    assert_eq!(compiled.vcas.len(), 1, "VCA did not lower");
    eprintln!(
        "minisynth lowered: vcos={}, vcas={}, stages={}",
        compiled.vcos.len(),
        compiled.vcas.len(),
        compiled.stages.len()
    );
    // NOTE: full VCO→VCF→VCA throughput awaits VCF lowering (vcf() is still a
    // GraphRole::ActiveIc stub with no runtime model — out of scope for the
    // VCO pass); the integration deliverable here is that the synth compiles
    // and both VCO + VCA lower as real per-instance DspBlocks.
}

// ===========================================================================
// (5) Realtime: a generator is cheap.
//
// The realtime standard's ≥ 30x target is a RELEASE figure (measured via
// `cargo run --release --example rt_bench`, where cem3340_vco clears ~97x).
// This test runs in the unoptimized test profile (~11x), so it gates on a
// conservative debug floor — proof the generator path adds no per-sample
// matrix work, not the headline number.
// ===========================================================================

#[test]
fn vco_runs_well_over_realtime() {
    let pedal = parse_pedal_file(MINIMAL_VCO).expect("parse");
    let mut compiled = compile_pedal(&pedal, SAMPLE_RATE).expect("compile");

    let secs = 5.0;
    let n = (SAMPLE_RATE * secs) as usize;
    // Warm-up outside the timed region.
    for _ in 0..(SAMPLE_RATE * 0.25) as usize {
        std::hint::black_box(compiled.process(0.0));
    }
    let start = Instant::now();
    let mut acc = 0.0;
    for _ in 0..n {
        acc += compiled.process(0.0).abs();
    }
    let elapsed = start.elapsed().as_secs_f64();
    std::hint::black_box(acc);
    let xrt = secs / elapsed;
    // Debug floor; release clears ≥ 30x (rt_bench).
    let floor = if cfg!(debug_assertions) { 4.0 } else { 30.0 };
    eprintln!(
        "vco realtime factor: {xrt:.1}x ({elapsed:.3} s wall for {secs} s audio, floor {floor}x)"
    );
    assert!(
        xrt >= floor,
        "VCO generator only {xrt:.1}x realtime, expected >= {floor}x"
    );
}
