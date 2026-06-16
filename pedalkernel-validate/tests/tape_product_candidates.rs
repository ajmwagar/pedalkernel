//! Product-candidate tape channel smoke tests.
//!
//! These are intentionally WDF-only. The SPICE decks live next to the circuits
//! and can generate ngspice goldens once ngspice is available, but the first
//! gate for these product cores is simpler: parse, compile, pass signal, and
//! expose a live shared control surface without relying on the known
//! compressor/pedal gap areas.

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
    defaults: [(&'static str, f64); 4],
}

const CANDIDATES: &[Candidate] = &[
    Candidate {
        name: "Swiss Tape Channel",
        pedal: "product/swiss_tape_channel.pedal",
        spice: "product/swiss_tape_channel.spice",
        defaults: [
            ("Drive", 0.35),
            ("Bias", 0.50),
            ("Tone", 0.60),
            ("Output", 0.70),
        ],
    },
    Candidate {
        name: "American Tube Tape Channel",
        pedal: "product/american_tube_tape_channel.pedal",
        spice: "product/american_tube_tape_channel.spice",
        defaults: [
            ("Drive", 0.35),
            ("Bias", 0.55),
            ("Tone", 0.60),
            ("Output", 0.75),
        ],
    },
];

const PANEL_CONTROLS: &[(&str, &str)] = &[
    ("Drive", "Drive"),
    ("Bias", "Bias"),
    ("Tone", "Tone"),
    ("Output", "Output"),
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
        proc.set_control_immediate(label, value);
    }

    let n = SR as usize;
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let x = amp * (2.0 * PI * freq * i as f64 / SR).sin();
        out.push(proc.process(x));
    }
    out
}

fn controls_for(c: Candidate, overrides: &[(&'static str, f64)]) -> Vec<(&'static str, f64)> {
    let mut controls = c.defaults.to_vec();
    for &(label, value) in overrides {
        let Some((_, slot)) = controls
            .iter_mut()
            .find(|(existing, _)| existing.eq_ignore_ascii_case(label))
        else {
            panic!("{} has no default for control {label}", c.name);
        };
        *slot = value;
    }
    controls
}

fn candidate_run(c: Candidate, freq: f64, amp: f64, overrides: &[(&'static str, f64)]) -> Vec<f64> {
    let controls = controls_for(c, overrides);
    sine_run(c.pedal, freq, amp, &controls)
}

fn settled(xs: &[f64]) -> &[f64] {
    &xs[SR as usize / 2..]
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

fn db_ratio(a: f64, b: f64) -> f64 {
    20.0 * (a.max(1e-12) / b.max(1e-12)).log10()
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
fn product_tape_candidates_controls_are_runtime_bound() {
    for c in CANDIDATES {
        let proc = compile(c.pedal);
        assert_eq!(
            proc.controls.len(),
            PANEL_CONTROLS.len(),
            "{} should expose exactly the shared product controls",
            c.name
        );

        for &(label, component_id) in PANEL_CONTROLS {
            let bindings = proc.resolve_control(label);
            assert_eq!(
                bindings.len(),
                1,
                "{} / {label} should have one runtime binding",
                c.name
            );

            let binding = &proc.controls[bindings[0]];
            assert_eq!(
                binding.component_id, component_id,
                "{} / {label} bound to unexpected component",
                c.name
            );

            eprintln!(
                "{} control {label}: component={}, target={:?}, fanout={}",
                c.name,
                binding.component_id,
                binding.target,
                binding.targets.len().max(1)
            );
        }
    }
}

#[test]
fn product_tape_candidates_compile_and_pass_signal() {
    for c in CANDIDATES {
        let out = candidate_run(*c, 1_000.0, 0.02, &[]);
        let settled = settled(&out);
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
        let clean = candidate_run(*c, 120.0, 0.03, &[("Drive", 0.05)]);
        let hot = candidate_run(*c, 120.0, 0.03, &[("Drive", 0.95)]);
        let clean_tail = settled(&clean);
        let hot_tail = settled(&hot);
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

#[test]
fn product_tape_candidates_panel_controls_have_audio_authority() {
    for c in CANDIDATES {
        let output_low = candidate_run(*c, 1_000.0, 0.02, &[("Output", 0.15)]);
        let output_high = candidate_run(*c, 1_000.0, 0.02, &[("Output", 0.95)]);
        let output_delta_db = db_ratio(rms(settled(&output_high)), rms(settled(&output_low)));

        let tone_dark = candidate_run(*c, 8_000.0, 0.02, &[("Tone", 0.05)]);
        let tone_bright = candidate_run(*c, 8_000.0, 0.02, &[("Tone", 0.95)]);
        let tone_delta_db = db_ratio(rms(settled(&tone_bright)), rms(settled(&tone_dark))).abs();

        let bias_low = candidate_run(*c, 120.0, 0.03, &[("Drive", 0.80), ("Bias", 0.05)]);
        let bias_high = candidate_run(*c, 120.0, 0.03, &[("Drive", 0.80), ("Bias", 0.95)]);
        let bias_low_tail = settled(&bias_low);
        let bias_high_tail = settled(&bias_high);
        let bias_thd_delta = (thd(bias_high_tail, 120.0) - thd(bias_low_tail, 120.0)).abs();
        let bias_level_delta_db = db_ratio(rms(bias_high_tail), rms(bias_low_tail)).abs();

        eprintln!(
            "{} panel authority: output_delta_db={output_delta_db:.2}, tone_delta_db={tone_delta_db:.2}, bias_thd_delta={bias_thd_delta:.5}, bias_level_delta_db={bias_level_delta_db:.2}",
            c.name
        );

        assert!(
            output_delta_db > 8.0,
            "{} Output should provide material trim range",
            c.name
        );
        assert!(
            tone_delta_db > 1.0,
            "{} Tone should materially change 8 kHz level",
            c.name
        );
        assert!(
            bias_thd_delta > 0.0005 || bias_level_delta_db > 0.25,
            "{} Bias should materially change tape-head operating color",
            c.name
        );
    }
}
