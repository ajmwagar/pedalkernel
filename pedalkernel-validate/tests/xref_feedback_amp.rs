//! CROSS-REFERENCE EXPERIMENT (pedalkernel-0c2p)
//!
//! Goal: decide whether the BA283 small-signal-gain bug is an ENGINE bug (the
//! WDF mishandles discrete global feedback loops) or a BA283-TOPOLOGY bug (the
//! BA283 `.pedal` mis-models that specific circuit).
//!
//! Method: a MINIMAL, textbook, trustworthy 2-BJT shunt-feedback amplifier,
//! built FAITHFULLY in BOTH a public SPICE deck
//! (`spice-circuits/active/xref_feedback_amp.spice`) and a public WDF `.pedal`
//! (`circuits/active/xref_feedback_amp.pedal`) — identical nodes / values /
//! 2N3904 model. This test drives the WDF pedal with small sines at 100 Hz /
//! 1 kHz / 10 kHz / 100 kHz, measures closed-loop gain (out/in fundamental, dB)
//! by Goertzel, and ALSO varies the feedback resistor Rf (15k/22k/33k/47k) to
//! probe the monotonicity-vs-feedback signature. SPICE `.ac` numbers are
//! generated separately by `tools/xref_spice_ac.sh` and pasted into the verdict.
//!
//! No engine code is changed. This is measurement only.
//!
//! Run: `cargo test -p pedalkernel-validate --test xref_feedback_amp -- --nocapture`

use pedalkernel::compiler::{compile_pedal, compile_pedal_with_options, CompileOptions};
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use std::f64::consts::PI;
use std::path::PathBuf;

const SR: f64 = 96_000.0;
const FREQS: [f64; 4] = [100.0, 1_000.0, 10_000.0, 100_000.0];

fn pedal_path() -> PathBuf {
    pedal_path_named("xref_feedback_amp.pedal")
}

fn pedal_path_named(name: &str) -> PathBuf {
    let manifest = env!("CARGO_MANIFEST_DIR");
    PathBuf::from(format!("{manifest}/circuits/active/{name}"))
}

/// Goertzel single-bin magnitude at `freq` over `signal` sampled at `sr`.
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

/// Build the pedal source with a substituted feedback resistor value.
fn pedal_source_with_rf(rf: &str) -> String {
    let base = std::fs::read_to_string(pedal_path()).expect("read xref pedal");
    // Replace exactly the Rf component declaration value.
    base.replace(
        "Rf:   resistor(33k)",
        &format!("Rf:   resistor({rf})"),
    )
}

/// Measure closed-loop gain (dB) of the WDF pedal at `freq` with input
/// amplitude `amp`. Warms up, then records an integer number of cycles.
fn wdf_gain_db(source: &str, freq: f64, amp: f64) -> f64 {
    wdf_gain_db_opt(source, freq, amp, false)
}

/// As [`wdf_gain_db`] but optionally forces `skip_blockwise` so the multi-BJT
/// group co-solves on the SAME MultiNl/rigid path the BA283 uses (instead of
/// being split into chained Blockwise NL blocks).
fn wdf_gain_db_opt(source: &str, freq: f64, amp: f64, skip_blockwise: bool) -> f64 {
    let def = parse_pedal_file(source).expect("parse xref pedal");
    let mut proc = if skip_blockwise {
        let opts = CompileOptions {
            skip_blockwise: true,
            ..CompileOptions::default()
        };
        compile_pedal_with_options(&def, SR, opts).expect("compile xref pedal (skip_blockwise)")
    } else {
        compile_pedal(&def, SR).expect("compile xref pedal")
    };

    // Choose a record window of whole cycles, long enough for bin resolution.
    let cycles = (freq * 0.05).max(10.0) as usize; // ~50 ms or >=10 cycles
    let samples_per_cycle = (SR / freq).max(2.0);
    let n = ((cycles as f64) * samples_per_cycle) as usize;

    // Warm up: settle coupling caps / DC ramp. Use several time constants.
    let warmup = (0.2 * SR) as usize; // 200 ms
    for i in 0..warmup {
        let x = amp * (2.0 * PI * freq * i as f64 / SR).sin();
        proc.process(x);
    }

    let mut input = Vec::with_capacity(n);
    let mut output = Vec::with_capacity(n);
    for i in 0..n {
        let x = amp * (2.0 * PI * freq * (warmup + i) as f64 / SR).sin();
        let y = proc.process(x);
        input.push(x);
        output.push(y);
    }

    let in_mag = goertzel(&input, SR, freq);
    let out_mag = goertzel(&output, SR, freq);
    20.0 * (out_mag / in_mag).log10()
}

/// CONTROL: measure several reference pedals through the SAME harness so the
/// feedback-amp result can be attributed. We want to know:
///   * single CE (existing npn_common_emitter.pedal) — does the basic engine
///     BJT gain path produce sensible gain at all?
///   * two-stage NO-feedback — do two DC-coupled coupled BJTs work without a
///     global feedback loop?
///   * the feedback amp — silence?
///   * single-BJT collector->base feedback (different engine path).
#[test]
fn xref_wdf_controls() {
    let amp = 0.01;
    let cases = [
        ("single CE (no fb)        ", "npn_common_emitter.pedal"),
        ("two-stage (NO global fb) ", "xref_twostage_nofb.pedal"),
        ("two-stage SHUNT FEEDBACK ", "xref_feedback_amp.pedal"),
        ("single-BJT C->B feedback ", "xref_single_bjt_fb.pedal"),
    ];
    eprintln!("\n  ═══ XREF controls — WDF gain (dB) @ 1 kHz, amp={amp} ═══");
    eprintln!("  {:<26} {:>10}", "circuit", "gain(dB)");
    for (label, file) in cases {
        let source = std::fs::read_to_string(pedal_path_named(file))
            .unwrap_or_else(|e| panic!("read {file}: {e}"));
        let g = wdf_gain_db(&source, 1_000.0, amp);
        eprintln!("  {label:<26} {g:>10.2}");
    }
    eprintln!(
        "  (a working CE stage is ~+20..+40 dB open-loop; ~0 dB or -inf means the\n   \
         stage is NOT passing signal — DC bias likely collapsed in the WDF.)"
    );
}

/// BONUS path: single-BJT collector->base shunt feedback across frequency.
/// This stage takes the single-BJT WDF-root path (not Blockwise/MultiNl), so it
/// tells us whether the feedback bug is path-specific or general.
#[test]
fn xref_wdf_single_bjt_fb_vs_freq() {
    let amp = 0.01;
    let src = std::fs::read_to_string(pedal_path_named("xref_single_bjt_fb.pedal")).unwrap();
    eprintln!("\n  ═══ XREF single-BJT C->B feedback — WDF gain vs freq (Rf=220k) ═══");
    eprintln!("  {:>10}   {:>10}", "freq (Hz)", "gain (dB)");
    for &f in &FREQS {
        let g = wdf_gain_db(&src, f, amp);
        eprintln!("  {f:>10.0}   {g:>10.2}");
    }
}

/// MultiNl-path measurement: force `skip_blockwise` so the two coupled BJTs
/// co-solve as ONE grouped-NR (MultiNl/rigid) group — the SAME engine path the
/// BA283 takes — instead of being split into chained Blockwise NL blocks.
/// This is the faithful cross-reference of the BA283 feedback path.
#[test]
fn xref_wdf_multinl_path() {
    let amp = 0.01;
    eprintln!(
        "\n  ═══ XREF — WDF gain (dB) on the MultiNl path (skip_blockwise) ═══"
    );

    // No-feedback control on the MultiNl path.
    {
        let src = std::fs::read_to_string(pedal_path_named("xref_twostage_nofb.pedal")).unwrap();
        eprintln!("  two-stage NO global fb (MultiNl):");
        eprintln!("    {:>10}   {:>10}", "freq (Hz)", "gain (dB)");
        for &f in &FREQS {
            let g = wdf_gain_db_opt(&src, f, amp, true);
            eprintln!("    {f:>10.0}   {g:>10.2}");
        }
    }

    // Feedback amp on the MultiNl path, swept vs Rf.
    eprintln!("\n  two-stage SHUNT FEEDBACK (MultiNl) — gain (dB) vs feedback Rf:");
    eprintln!(
        "  {:>6} | {:>9} {:>9} {:>9} {:>9}",
        "Rf", "100 Hz", "1 kHz", "10 kHz", "100 kHz"
    );
    let rfs = ["15k", "22k", "33k", "47k"];
    let mut g1k = Vec::new();
    for rf in rfs {
        let src = pedal_source_with_rf(rf);
        let mut row = [0.0f64; 4];
        for (i, &f) in FREQS.iter().enumerate() {
            row[i] = wdf_gain_db_opt(&src, f, amp, true);
        }
        eprintln!(
            "  {rf:>6} | {:>9.2} {:>9.2} {:>9.2} {:>9.2}",
            row[0], row[1], row[2], row[3]
        );
        g1k.push(row[1]);
    }
    let monotonic = g1k.windows(2).all(|w| w[1] >= w[0]);
    eprintln!(
        "\n  MultiNl 1 kHz gain vs Rf monotonic-increasing? {monotonic}  ({:?})",
        g1k.iter().map(|g| format!("{g:.2}")).collect::<Vec<_>>()
    );
}

/// PRIMARY measurement: gain vs frequency at the DESIGN feedback (Rf=33k).
#[test]
fn xref_wdf_gain_vs_freq_design() {
    let source = pedal_source_with_rf("33k");
    let amp = 0.01; // small signal — verified linear below

    eprintln!("\n  ═══ XREF feedback amp — WDF gain vs frequency (Rf=33k, design) ═══");
    eprintln!("  {:>10}   {:>10}", "freq (Hz)", "gain (dB)");
    for &f in &FREQS {
        let g = wdf_gain_db(&source, f, amp);
        eprintln!("  {f:>10.0}   {g:>10.2}");
    }

    // Sanity: output must not be silence at 1 kHz.
    let g1k = wdf_gain_db(&source, 1_000.0, amp);
    assert!(
        g1k.is_finite(),
        "WDF gain at 1 kHz is not finite — compilation/processing failed"
    );
}

/// Linearity check: gain must be (near) amplitude-independent for small signals.
#[test]
fn xref_wdf_linearity() {
    let source = pedal_source_with_rf("33k");
    eprintln!("\n  ═══ XREF feedback amp — linearity check (Rf=33k, 1 kHz) ═══");
    let mut prev: Option<f64> = None;
    for &amp in &[0.002, 0.005, 0.01, 0.02] {
        let g = wdf_gain_db(&source, 1_000.0, amp);
        eprintln!("  amp={amp:.3}  gain={g:.3} dB");
        if let Some(p) = prev {
            assert!(
                (g - p).abs() < 0.5,
                "gain shifts >0.5 dB with amplitude ({p:.3}->{g:.3}) — not linear at these amps"
            );
        }
        prev = Some(g);
    }
}

/// FEEDBACK-MONOTONICITY signature: gain vs Rf at each frequency.
#[test]
fn xref_wdf_gain_vs_feedback() {
    let amp = 0.01;
    let rfs = ["15k", "22k", "33k", "47k"];

    eprintln!("\n  ═══ XREF feedback amp — WDF gain (dB) vs feedback Rf ═══");
    eprintln!(
        "  {:>6} | {:>9} {:>9} {:>9} {:>9}",
        "Rf", "100 Hz", "1 kHz", "10 kHz", "100 kHz"
    );
    let mut table: Vec<(String, [f64; 4])> = Vec::new();
    for rf in rfs {
        let source = pedal_source_with_rf(rf);
        let mut row = [0.0f64; 4];
        for (i, &f) in FREQS.iter().enumerate() {
            row[i] = wdf_gain_db(&source, f, amp);
        }
        eprintln!(
            "  {rf:>6} | {:>9.2} {:>9.2} {:>9.2} {:>9.2}",
            row[0], row[1], row[2], row[3]
        );
        table.push((rf.to_string(), row));
    }

    // Report monotonicity-vs-feedback at 1 kHz (index 1). SPICE design is
    // monotonic-increasing with Rf. This is logged for the verdict; not gated.
    let g1k: Vec<f64> = table.iter().map(|(_, r)| r[1]).collect();
    let monotonic = g1k.windows(2).all(|w| w[1] >= w[0]);
    eprintln!(
        "\n  1 kHz gain vs Rf monotonic-increasing? {monotonic}  ({:?})",
        g1k.iter().map(|g| format!("{g:.2}")).collect::<Vec<_>>()
    );
}

/// ISOLATED steady-state trace dump for the 2-BJT cascade on the MultiNl path.
/// Run with: cargo test -p pedalkernel-validate --features debug-trace \
///   --test xref_feedback_amp xref_dump_twostage_multinl -- --nocapture
/// Set PK_TRACE_SKIP to the warmup count so the trace fires at steady state.
#[test]
fn xref_dump_twostage_multinl() {
    let src = std::fs::read_to_string(pedal_path_named("xref_twostage_nofb.pedal")).unwrap();
    let amp = 0.01;
    let freq = 1_000.0;
    let g = wdf_gain_db_opt(&src, freq, amp, true);
    eprintln!("\n  TWOSTAGE NOFB (MultiNl) gain @1kHz = {g:.3} dB");
}

/// Same dump for the WORKING single-BJT feedback amp (WDF-root path).
#[test]
fn xref_dump_single_bjt() {
    let src = std::fs::read_to_string(pedal_path_named("xref_single_bjt_fb.pedal")).unwrap();
    let amp = 0.01;
    let freq = 1_000.0;
    let g = wdf_gain_db_opt(&src, freq, amp, false);
    eprintln!("\n  SINGLE-BJT FB gain @1kHz = {g:.3} dB");
}

/// DEFAULT-path dump for the 2-BJT cascade (no skip_blockwise → the path the
/// controls test exercises, which gave -123 dB).
#[test]
fn xref_dump_twostage_default() {
    let src = std::fs::read_to_string(pedal_path_named("xref_twostage_nofb.pedal")).unwrap();
    let g = wdf_gain_db_opt(&src, 1_000.0, 0.01, false);
    eprintln!("\n  TWOSTAGE NOFB (default path) gain @1kHz = {g:.3} dB");
}

/// Dump the SHUNT-FEEDBACK amp on the MultiNl path (the BA283 path: the global
/// feedback loop forces the two BJTs into a single grouped-NR MultiNl group).
#[test]
fn xref_dump_feedback_multinl() {
    let src = pedal_source_with_rf("33k");
    let g = wdf_gain_db_opt(&src, 1_000.0, 0.01, true);
    eprintln!("\n  FEEDBACK AMP (MultiNl, Rf=33k) gain @1kHz = {g:.3} dB");
}
