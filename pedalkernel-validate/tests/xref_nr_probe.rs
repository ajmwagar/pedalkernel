//! TEMPORARY probe (pedalkernel-0c2p adaptation-floor): grouped-NR iteration
//! counts + clamp/cap stats + gain for the xref multi-BJT MultiNl path, so the
//! Rp adaptation-floor experiment can be judged on CONVERGENCE not just gain.
//!
//! Run per mode:
//!   PK_PIN_NL_RP=off  cargo test -p pedalkernel-validate --test xref_nr_probe -- --nocapture --test-threads=1
//!   PK_PIN_NL_RP=pin  ...
//!   PK_PIN_NL_RP=     ... (floor, default)

use pedalkernel::compiler::{compile_pedal_with_options, CompileOptions};
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use pedalkernel_rt::elements::nonlinear::{reset_solver_stats, solver_stats_snapshot};
use std::f64::consts::PI;
use std::path::PathBuf;

const SR: f64 = 96_000.0;

fn pedal_path_named(name: &str) -> PathBuf {
    let manifest = env!("CARGO_MANIFEST_DIR");
    PathBuf::from(format!("{manifest}/circuits/active/{name}"))
}

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

fn probe(label: &str, source: &str, freq: f64, amp: f64) {
    let def = parse_pedal_file(source).expect("parse");
    let opts = CompileOptions {
        skip_blockwise: true,
        ..CompileOptions::default()
    };
    let mut proc =
        compile_pedal_with_options(&def, SR, opts).expect("compile (skip_blockwise)");

    let warmup = (0.2 * SR) as usize;
    for i in 0..warmup {
        let x = amp * (2.0 * PI * freq * i as f64 / SR).sin();
        proc.process(x);
    }

    // Measure NR stats over the record window only.
    reset_solver_stats();
    let cycles = (freq * 0.05).max(10.0) as usize;
    let spc = (SR / freq).max(2.0);
    let n = ((cycles as f64) * spc) as usize;
    let mut input = Vec::with_capacity(n);
    let mut output = Vec::with_capacity(n);
    for i in 0..n {
        let x = amp * (2.0 * PI * freq * (warmup + i) as f64 / SR).sin();
        let y = proc.process(x);
        input.push(x);
        output.push(y);
    }
    let s = solver_stats_snapshot();
    let in_mag = goertzel(&input, SR, freq);
    let out_mag = goertzel(&output, SR, freq);
    let gain_db = 20.0 * (out_mag / in_mag).log10();

    let avg = if s.solves > 0 {
        s.total_iterations as f64 / s.solves as f64
    } else {
        0.0
    };
    eprintln!(
        "  {label:<28} gain={gain_db:>8.2}dB | solves={:<6} avg_iter={avg:>5.2} max_iter={:<3} cap_hits={:<6} clamp_hits={:<6} max_resid={:.3e}",
        s.solves, s.max_iterations, s.iter_cap_hits, s.clamp_hits, s.max_residual
    );
}

#[test]
fn xref_nr_probe_multinl() {
    let mode = std::env::var("PK_PIN_NL_RP").unwrap_or_default();
    eprintln!("\n  ═══ XREF NR probe (MultiNl path) — PK_PIN_NL_RP='{mode}' ═══");
    let amp = 0.01;
    let freq = 1_000.0;

    let nofb = std::fs::read_to_string(pedal_path_named("xref_twostage_nofb.pedal")).unwrap();
    probe("two-stage NOFB", &nofb, freq, amp);

    let fb_base = std::fs::read_to_string(pedal_path_named("xref_feedback_amp.pedal")).unwrap();
    for rf in ["15k", "22k", "33k", "47k"] {
        let src = fb_base.replace("Rf:   resistor(33k)", &format!("Rf:   resistor({rf})"));
        probe(&format!("feedback Rf={rf}"), &src, freq, amp);
    }
}
