//! Product-candidate tape channel smoke tests.
//!
//! These are intentionally WDF-only. The SPICE decks live next to the circuits
//! and can generate ngspice goldens once ngspice is available, but the first
//! gate for these product cores is simpler: parse, compile, pass signal, and
//! expose a live Drive control without relying on the known compressor/pedal
//! gap areas.

use std::fs;
use std::path::PathBuf;

use pedalkernel::compiler::{compile_pedal, CompiledPedal};
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;

const SR: f64 = 48_000.0;
const PI: f64 = std::f64::consts::PI;

#[derive(Clone, Copy)]
struct Candidate {
    name: &'static str,
    pedal: &'static str,
    spice: &'static str,
}

const CANDIDATES: &[Candidate] = &[
    Candidate {
        name: "Swiss Tape Channel",
        pedal: "product/swiss_tape_channel.pedal",
        spice: "product/swiss_tape_channel.spice",
    },
    Candidate {
        name: "American Tube Tape Channel",
        pedal: "product/american_tube_tape_channel.pedal",
        spice: "product/american_tube_tape_channel.spice",
    },
];

fn manifest() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn load_pedal(rel: &str) -> String {
    fs::read_to_string(manifest().join("circuits").join(rel))
        .unwrap_or_else(|e| panic!("read {rel}: {e}"))
}

fn compile(rel: &str) -> CompiledPedal {
    let src = load_pedal(rel);
    let def = parse_pedal_file(&src).unwrap_or_else(|e| panic!("parse {rel}: {e}"));
    compile_pedal(&def, SR).unwrap_or_else(|e| panic!("compile {rel}: {e}"))
}

fn sine_run(rel: &str, freq: f64, amp: f64, controls: &[(&str, f64)]) -> Vec<f64> {
    let mut proc = compile(rel);
    for &(label, value) in controls {
        proc.set_control(label, value);
    }

    let n = SR as usize;
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let x = amp * (2.0 * PI * freq * i as f64 / SR).sin();
        out.push(proc.process(x));
    }
    out
}

fn rms(xs: &[f64]) -> f64 {
    (xs.iter().map(|x| x * x).sum::<f64>() / xs.len().max(1) as f64).sqrt()
}

fn peak(xs: &[f64]) -> f64 {
    xs.iter().map(|x| x.abs()).fold(0.0, f64::max)
}

fn goertzel(xs: &[f64], freq: f64) -> f64 {
    let n = xs.len() as f64;
    let k = (0.5 + n * freq / SR).floor();
    let omega = 2.0 * PI * k / n;
    let coeff = 2.0 * omega.cos();
    let (mut s1, mut s2) = (0.0, 0.0);
    for &x in xs {
        let s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    (s1 * s1 + s2 * s2 - coeff * s1 * s2).max(0.0).sqrt()
}

fn thd(xs: &[f64], freq: f64) -> f64 {
    let fund = goertzel(xs, freq).max(1e-18);
    let mut harmonics = 0.0;
    for k in 2..=8 {
        let h = goertzel(xs, freq * k as f64);
        harmonics += h * h;
    }
    harmonics.sqrt() / fund
}

#[test]
fn product_tape_candidates_have_matching_spice_decks() {
    for c in CANDIDATES {
        let pedal = manifest().join("circuits").join(c.pedal);
        let spice = manifest().join("spice-circuits").join(c.spice);
        assert!(
            pedal.exists(),
            "{} pedal missing: {}",
            c.name,
            pedal.display()
        );
        assert!(
            spice.exists(),
            "{} SPICE deck missing: {}",
            c.name,
            spice.display()
        );

        let deck = fs::read_to_string(&spice).expect("read SPICE deck");
        assert!(
            deck.contains("Bhead"),
            "{} SPICE deck must include the J-A tape-head behavioural source",
            c.name
        );
        assert!(
            deck.contains("v_out"),
            "{} SPICE deck must expose harness output node v_out",
            c.name
        );
    }
}

#[test]
fn product_tape_candidates_compile_and_pass_signal() {
    for c in CANDIDATES {
        let out = sine_run(
            c.pedal,
            1_000.0,
            0.02,
            &[
                ("Drive", 0.35),
                ("Bias", 0.5),
                ("Tone", 0.6),
                ("Output", 0.75),
            ],
        );
        let settled = &out[SR as usize / 2..];
        assert!(
            settled.iter().all(|x| x.is_finite()),
            "{} output contains non-finite samples",
            c.name
        );
        let r = rms(settled);
        let p = peak(settled);
        eprintln!("{} default: rms={r:.6}, peak={p:.6}", c.name);
        assert!(r > 1e-5, "{} should pass audible signal, rms={r}", c.name);
        assert!(p < 50.0, "{} output runaway, peak={p}", c.name);
    }
}

#[test]
fn product_tape_candidates_drive_changes_harmonic_color() {
    for c in CANDIDATES {
        let clean = sine_run(
            c.pedal,
            120.0,
            0.03,
            &[
                ("Drive", 0.05),
                ("Bias", 0.5),
                ("Tone", 0.6),
                ("Output", 0.75),
            ],
        );
        let hot = sine_run(
            c.pedal,
            120.0,
            0.03,
            &[
                ("Drive", 0.95),
                ("Bias", 0.5),
                ("Tone", 0.6),
                ("Output", 0.75),
            ],
        );
        let clean_tail = &clean[SR as usize / 2..];
        let hot_tail = &hot[SR as usize / 2..];
        let clean_thd = thd(clean_tail, 120.0);
        let hot_thd = thd(hot_tail, 120.0);
        let clean_rms = rms(clean_tail);
        let hot_rms = rms(hot_tail);
        eprintln!(
            "{} drive sweep: clean_thd={clean_thd:.5}, hot_thd={hot_thd:.5}, clean_rms={clean_rms:.6}, hot_rms={hot_rms:.6}",
            c.name
        );

        let thd_delta = (hot_thd - clean_thd).abs();
        let rms_delta_db = 20.0 * (hot_rms.max(1e-12) / clean_rms.max(1e-12)).log10();
        assert!(
            thd_delta > 0.001 || rms_delta_db.abs() > 1.0,
            "{} Drive should materially change level or harmonic color",
            c.name
        );
    }
}
