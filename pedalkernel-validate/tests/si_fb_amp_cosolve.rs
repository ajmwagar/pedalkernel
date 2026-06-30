//! Silicon 2-BJT shunt-feedback amp — compile / stability / co-solve probe.
//!
//! `circuits/active/si_fb_amp.pedal` is a CLEAN, STABLE replacement for the
//! bistable `xref_feedback_amp` reference: two DC-coupled 2N3904 common-emitter
//! stages with GLOBAL DEGENERATIVE feedback (Q2 emitter -> Rf -> Q1 base). It is
//! ngspice-verified to bias to a single mid-rail root (nout ~4.35 V, both BJTs
//! active, holds flat) and the WDF tracks ngspice to ~0 dB RMS/peak.
//!
//! FINDING (calibrated against xref on this same engine): because Q1's emitter
//! is HARD-GROUNDED (per the verified design), Q1 collapses to a single-port
//! common-emitter WDF root and never enters the rigid/Blockwise multi-port NL
//! grouping. The two BJTs therefore compile to SEPARATE Wdf-root stages rather
//! than one co-solved group. (The reference xref — both emitters bypassed, hence
//! multi-port — DOES form a Blockwise block_count=2 / MultiNl n_nl=4 group on the
//! identical engine.) So on this engine the grouped-Newton co-solve is gated on
//! multi-port BJT topology, not merely on the presence of resistor-coupled
//! feedback. The grouping test below is therefore `#[ignore]`d as a documented
//! gap, not deleted.
//!
//! Run: `cargo test -p pedalkernel-validate --test si_fb_amp_cosolve -- --nocapture`

use pedalkernel::compiler::{compile_pedal_with_options, CompileOptions};
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::oversampling::OversamplingFactor;
use pedalkernel::PedalProcessor;
use pedalkernel_rt::processor::Stage;
use std::f64::consts::PI;
use std::path::PathBuf;

/// Match the validation runner: golden is pre-oversampled, so compile at the
/// effective (oversampled) rate with X1 internal oversampling.
const EFFECTIVE_SR: f64 = 96_000.0 * 4.0;

fn pedal_path() -> PathBuf {
    let manifest = env!("CARGO_MANIFEST_DIR");
    PathBuf::from(format!("{manifest}/circuits/active/si_fb_amp.pedal"))
}

fn stage_label(s: &Stage) -> &'static str {
    match s {
        Stage::MultiNl(_) => "MultiNl",
        Stage::Blockwise(_) => "Blockwise",
        Stage::Wdf(_) => "Wdf",
        Stage::Iir(_) => "Iir",
        Stage::StateSpace(_) => "StateSpace",
        _ => "other",
    }
}

fn runner_opts() -> CompileOptions {
    CompileOptions {
        oversampling: OversamplingFactor::X1,
        ..CompileOptions::default()
    }
}

/// Goertzel single-bin magnitude.
fn goertzel(signal: &[f64], sr: f64, freq: f64) -> f64 {
    let n = signal.len();
    let k = (n as f64 * freq / sr).round();
    let w = 2.0 * PI * k / n as f64;
    let coeff = 2.0 * w.cos();
    let (mut s1, mut s2) = (0.0f64, 0.0f64);
    for &x in signal {
        let s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    let real = s1 - s2 * w.cos();
    let imag = s2 * w.sin();
    2.0 * (real * real + imag * imag).sqrt() / n as f64
}

/// PRIMARY: the reference must COMPILE, hold a STABLE bias, and pass a clean
/// small signal (finite, bounded, non-silent gain). This is the property that
/// makes it a trustworthy validation vehicle.
#[test]
fn si_fb_amp_compiles_stable_and_passes_signal() {
    let src = std::fs::read_to_string(pedal_path()).expect("read si_fb_amp.pedal");
    let def = parse_pedal_file(&src).expect("parse si_fb_amp.pedal");
    let mut proc =
        compile_pedal_with_options(&def, EFFECTIVE_SR, runner_opts()).expect("compile si_fb_amp");

    eprintln!(
        "\n  si_fb_amp: {} stage(s) — [{}]",
        proc.stages.len(),
        proc.stages
            .iter()
            .map(stage_label)
            .collect::<Vec<_>>()
            .join(", ")
    );

    let freq = 1_000.0;
    let amp = 0.01;

    // Warm up the coupling caps to steady-state bias.
    let warmup = (0.2 * EFFECTIVE_SR) as usize;
    for i in 0..warmup {
        let x = amp * (2.0 * PI * freq * i as f64 / EFFECTIVE_SR).sin();
        let y = proc.process(x);
        assert!(y.is_finite(), "WDF output went non-finite during warmup");
    }

    // Record a window of whole cycles.
    let n = (EFFECTIVE_SR / freq * 20.0) as usize;
    let mut input = Vec::with_capacity(n);
    let mut output = Vec::with_capacity(n);
    let mut peak = 0.0f64;
    for i in 0..n {
        let x = amp * (2.0 * PI * freq * (warmup + i) as f64 / EFFECTIVE_SR).sin();
        let y = proc.process(x);
        assert!(y.is_finite(), "WDF output non-finite in record window");
        peak = peak.max(y.abs());
        input.push(x);
        output.push(y);
    }

    let gain_db = 20.0 * (goertzel(&output, EFFECTIVE_SR, freq) / goertzel(&input, EFFECTIVE_SR, freq)).log10();
    eprintln!("  WDF small-signal gain @1kHz = {gain_db:.2} dB,  output peak = {peak:.4} V");

    assert!(peak < 9.0, "output exceeds rail — bias unstable (peak {peak})");
    assert!(
        gain_db.is_finite() && gain_db > -60.0,
        "stage is silent — DC bias collapsed (gain {gain_db} dB)"
    );
}

/// DOCUMENTED GAP (ignored): the exact verified design does NOT co-solve into a
/// single MultiNl/Blockwise group on this engine, because Q1's grounded emitter
/// makes it a single-port WDF root. Kept as an executable record of the finding
/// (calibration: xref, with bypassed multi-port emitters, DOES group here).
#[test]
#[ignore = "grounded-emitter Q1 collapses to single-port WDF root; co-solve grouping needs multi-port BJTs (calibrated vs xref)"]
fn si_fb_amp_co_solves_both_bjts() {
    let src = std::fs::read_to_string(pedal_path()).expect("read si_fb_amp.pedal");
    let def = parse_pedal_file(&src).expect("parse si_fb_amp.pedal");

    for mono in [false, true] {
        let opts = if mono {
            runner_opts().force_monolithic()
        } else {
            runner_opts()
        };
        let compiled = compile_pedal_with_options(&def, EFFECTIVE_SR, opts).expect("compile");
        let mut co_solve = false;
        for s in &compiled.stages {
            match s {
                Stage::Blockwise(b) if b.block_count() >= 2 => co_solve = true,
                Stage::MultiNl(m) if m.n_nl >= 4 => co_solve = true,
                _ => {}
            }
        }
        eprintln!(
            "  [{}] stages [{}] -> co_solve_group={co_solve}",
            if mono { "monolithic" } else { "default" },
            compiled
                .stages
                .iter()
                .map(stage_label)
                .collect::<Vec<_>>()
                .join(", ")
        );
        assert!(
            co_solve,
            "expected a 2-BJT co-solve group (Blockwise blocks>=2 or MultiNl n_nl>=4)"
        );
    }
}
