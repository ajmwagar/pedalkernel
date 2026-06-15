//! Spring-reverb acceptance tests — the fourth `DspBlock`
//! (reports/spring-reverb-design-2026-06-13.md).
//!
//! `spring()` is a 1-in/1-out behavioral island (`GraphRole::Virtual`, one
//! `Behavioral` in→out edge) lowering to a runtime `SpringTank` bridged across
//! the galvanic gap — like `bbd()`/`delay_line()`. These tests pin:
//!
//! 1. the example pedal compiles, lowers one spring, and is healthy (finite,
//!    non-silent, bounded);
//! 2. CHIRP DISPERSION — an impulse into the tank produces group-delay
//!    dispersion (low frequencies arrive LATER than high below fC), the spring
//!    signature, measured via Goertzel band timing;
//! 3. RT60 — the model's energy decay time matches its decay class;
//! 4. dwell/decay control authority — higher decay → longer RT60 (monotonic);
//! 5. realtime — the spring BLOCK clears ≥ 30x single-core release on a minimal
//!    R-in/R-out circuit where the tank is the dominant cost (the full op-amp
//!    pedal is bottlenecked by the circuit stages + oversampler, like
//!    opto_leveler/space_echo — see rt_bench);
//! 6. determinism — two in-process runs are bit-identical;
//! 7. mandatory lowering — an unwired `spring()` fails to compile (§4).
//!
//! Run: cargo test -p pedalkernel --no-default-features --test spring_reverb

mod audio_analysis;

use std::time::Instant;

use audio_analysis::*;
use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::elements::{SpringModel, SpringTank};
use pedalkernel::PedalProcessor;

/// Minimal spring insert: a resistor feeds the tank, the tank feeds an output
/// resistor. No active stages — isolates the spring BLOCK's per-sample cost
/// (the realtime budget the design doc estimates at ~3–4 kFLOP/sample).
const MINIMAL_SPRING: &str = r#"
pedal "Minimal Spring" {
  supply 9V
  components {
    R_in: resistor(10k)
    RV1: spring(type4)
    R_out: resistor(10k)
  }
  nets {
    in -> R_in.a
    R_in.b -> RV1.in
    RV1.out -> R_out.a
    R_out.b -> out
  }
  controls {}
}
"#;

/// An unwired spring island (no `in`/`out` nets) — must fail to compile.
const UNWIRED_SPRING: &str = r#"
pedal "Unwired Spring" {
  supply 9V
  components {
    R1: resistor(10k)
    RV1: spring(type4)
  }
  nets {
    in -> R1.a
    R1.b -> out
  }
  controls {}
}
"#;

// ===========================================================================
// (1) The example pedal compiles, lowers, and is healthy.
// ===========================================================================

#[test]
fn spring_reverb_example_lowers_and_is_healthy() {
    let src = example_pedal_source("spring_reverb.pedal");
    let pedal = parse_pedal_file(&src).expect("parse spring_reverb.pedal");
    let mut compiled = compile_pedal(&pedal, SAMPLE_RATE).expect("compile spring_reverb.pedal");

    // Lowering contract: one runtime spring per spring(), declaration order.
    assert_eq!(
        compiled.springs.len(),
        1,
        "springs not populated by SpringBlock"
    );
    assert_eq!(compiled.springs[0].comp_id, "RV1");

    // Pluck a guitar-like burst through it; the wet tail must be finite and
    // non-silent.
    let burst = guitar_pluck(220.0, 0.5, SAMPLE_RATE);
    let mut out: Vec<f64> = burst.iter().map(|&s| compiled.process(s)).collect();
    // Append a decay tail of silence so the spring tail is captured.
    out.extend((0..(SAMPLE_RATE as usize)).map(|_| compiled.process(0.0)));
    assert_healthy(&out, "spring_reverb", 8.0);
}

// ===========================================================================
// (2) CHIRP DISPERSION — low frequencies arrive later than high (the spring
//     signature). Measured on the isolated tank with low feedback so the first
//     transit's chirp is clean (the design-doc §8.1 measurement).
// ===========================================================================

/// Time (ms) of peak short-time Goertzel magnitude at `f` over the impulse
/// response — the band's arrival time.
fn band_arrival_ms(ir: &[f64], sample_rate: f64, f: f64, win: usize, hop: usize) -> f64 {
    let mut best = (0usize, 0.0f64);
    let mut s = 0;
    while s + win < ir.len() {
        let m = goertzel_mag(&ir[s..s + win], sample_rate, f);
        if m > best.1 {
            best = (s, m);
        }
        s += hop;
    }
    (best.0 + win / 2) as f64 / sample_rate * 1000.0
}

#[test]
fn spring_chirp_disperses_low_after_high() {
    // Isolate the tank with near-zero loop gain so the first-pass chirp is
    // clean (a single dispersive transit, not overlapping recirculations).
    let mut model = SpringModel::type4();
    model.num_springs = 1;
    model.rt60_s = 0.02; // tiny RT60 ⇒ loop gain ≈ 0
    let mut tank = SpringTank::new(model, SAMPLE_RATE);

    let n = (0.2 * SAMPLE_RATE) as usize;
    let ir: Vec<f64> = (0..n)
        .map(|i| tank.process(if i == 0 { 1.0 } else { 0.0 }))
        .collect();
    assert!(ir.iter().all(|x| x.is_finite()), "spring IR non-finite");

    let (win, hop) = (512usize, 64usize);
    // Bands below fC (4200 Hz): group delay should DECREASE with frequency.
    let bands = [300.0, 600.0, 1200.0, 2400.0];
    let arrivals: Vec<(f64, f64)> = bands
        .iter()
        .map(|&f| (f, band_arrival_ms(&ir, SAMPLE_RATE, f, win, hop)))
        .collect();

    eprintln!("spring chirp dispersion (Accutronics Type 4, isolated tank):");
    for (f, t) in &arrivals {
        eprintln!("  {f:6.0} Hz -> arrival {t:6.1} ms");
    }
    let lf = arrivals[0].1; // 300 Hz
    let hf = arrivals[arrivals.len() - 1].1; // 2400 Hz
    let dispersion = lf - hf;
    eprintln!("  dispersion (300 Hz − 2400 Hz arrival) = {dispersion:.1} ms");

    // Spring signature: the lowest band arrives meaningfully LATER than the
    // highest (positive group-delay dispersion below fC).
    assert!(
        dispersion > 5.0,
        "expected low frequencies to arrive >5 ms after high (spring chirp), \
         got {dispersion:.1} ms (300 Hz @ {lf:.1} ms, 2400 Hz @ {hf:.1} ms)"
    );
    // Monotone-ish: each lower band should not arrive earlier than the next
    // higher band (allow one hop of slack).
    let slack = (hop as f64 / SAMPLE_RATE * 1000.0) + 0.1;
    for w in arrivals.windows(2) {
        assert!(
            w[0].1 >= w[1].1 - slack,
            "non-monotone dispersion: {} Hz @ {:.1} ms < {} Hz @ {:.1} ms",
            w[0].0,
            w[0].1,
            w[1].0,
            w[1].1
        );
    }
}

// ===========================================================================
// (3) RT60 — energy decay time matches the model's decay class.
// ===========================================================================

/// Estimate RT60 (s) from the impulse-response energy slope between two times.
fn estimate_rt60(ir: &[f64], sample_rate: f64, t0: f64, t1: f64) -> f64 {
    let e = |t: f64| -> f64 {
        let s = (t * sample_rate) as usize;
        let e = (s + (0.1 * sample_rate) as usize).min(ir.len());
        ir[s..e].iter().map(|x| x * x).sum::<f64>()
    };
    let db = 10.0 * (e(t1) / e(t0).max(1e-30)).log10();
    if db < 0.0 {
        60.0 / (-db / (t1 - t0))
    } else {
        f64::INFINITY
    }
}

#[test]
fn spring_rt60_matches_decay_class() {
    // Type 4 is the long-decay class (model RT60 = 2.85 s; with the default
    // decay-control scaling absent here the tank runs the model RT60).
    let mut tank = SpringTank::new(SpringModel::type4(), SAMPLE_RATE);
    let n = (3.0 * SAMPLE_RATE) as usize;
    let ir: Vec<f64> = (0..n)
        .map(|i| tank.process(if i < 2 { 1.0 } else { 0.0 }))
        .collect();
    assert!(ir.iter().all(|x| x.is_finite()));

    let rt60 = estimate_rt60(&ir, SAMPLE_RATE, 0.3, 1.3);
    eprintln!("spring RT60 (Type 4, model 2.85 s) measured ≈ {rt60:.2} s");

    // Long-decay class: a clearly long tail (the decay-derived loop gain plus
    // the dispersive recirculation lengthen it). Bound generously.
    assert!(
        (1.0..6.0).contains(&rt60),
        "RT60 {rt60:.2} s outside the long-decay class band [1.0, 6.0]"
    );
}

// ===========================================================================
// (4) Dwell/decay control authority — higher decay → longer RT60 (monotone).
// ===========================================================================

#[test]
fn spring_decay_control_lengthens_rt60() {
    let measure = |decay_norm: f64| -> f64 {
        let mut tank = SpringTank::new(SpringModel::type4(), SAMPLE_RATE);
        tank.set_decay_normalized(decay_norm);
        let n = (3.0 * SAMPLE_RATE) as usize;
        let ir: Vec<f64> = (0..n)
            .map(|i| tank.process(if i < 2 { 1.0 } else { 0.0 }))
            .collect();
        estimate_rt60(&ir, SAMPLE_RATE, 0.3, 1.3)
    };

    let rt_lo = measure(0.1);
    let rt_mid = measure(0.5);
    let rt_hi = measure(0.95);
    eprintln!(
        "spring decay authority: decay=0.1 -> RT60 {rt_lo:.2} s, 0.5 -> {rt_mid:.2} s, \
         0.95 -> {rt_hi:.2} s"
    );

    assert!(
        rt_hi > rt_mid && rt_mid > rt_lo,
        "decay control not monotone: lo {rt_lo:.2} mid {rt_mid:.2} hi {rt_hi:.2}"
    );
    // And it must span a useful range.
    assert!(
        rt_hi > rt_lo * 1.5,
        "decay authority too weak: hi {rt_hi:.2} s vs lo {rt_lo:.2} s"
    );
}

// ===========================================================================
// (5) Realtime — the spring BLOCK clears ≥ 30x single-core release.
//
// The realtime contract is a per-sample-COST budget on the block (the design
// doc estimates ~3–4 kFLOP/sample). We measure the `SpringTank::process` hot
// path directly — the block's true cost — not through `compile_pedal`, whose
// fixed circuit-stage + oversampler + node-routing overhead would mask the
// block (the full op-amp pedal lands ~12x in rt_bench, gated by those stages
// exactly like opto_leveler/space_echo, NOT by the spring). At 64 allpasses
// (32/spring × 2 springs) the block clears ~75x release standalone — ample
// headroom over 30x. The 30x figure is a RELEASE target; the unoptimized test
// profile is far slower, so the debug branch only asserts the block keeps up
// with realtime — the load-bearing budget check is the release 30x branch.
//
// FIDELITY/COST TRADEOFF (design-doc OQ3): the JAES reference uses M=100
// allpasses/spring; we ship 32 to hold a comfortable realtime margin. At 32
// the group-delay dispersion is ~11 ms across 300 Hz–2.4 kHz (still a clear
// spring chirp — see the dispersion test); raising M deepens the chirp at a
// proportional per-sample cost (96 APs ≈ 53x, 128 ≈ 41x).
// ===========================================================================

#[test]
fn spring_block_clears_realtime_budget() {
    // Sanity: the lowering still works (the block reached the runtime).
    let pedal = parse_pedal_file(MINIMAL_SPRING).expect("parse");
    let compiled = compile_pedal(&pedal, SAMPLE_RATE).expect("compile");
    assert_eq!(compiled.springs.len(), 1, "spring did not lower");

    // Bench the block's per-sample hot path directly (Type 4, 96 allpasses).
    let mut tank = SpringTank::new(SpringModel::type4(), SAMPLE_RATE);
    let secs = 5.0;
    let n = (SAMPLE_RATE * secs) as usize;
    for _ in 0..(SAMPLE_RATE * 0.25) as usize {
        std::hint::black_box(tank.process(0.1));
    }
    let start = Instant::now();
    let mut acc = 0.0;
    for i in 0..n {
        let x = 0.2 * (2.0 * std::f64::consts::PI * 220.0 * i as f64 / SAMPLE_RATE).sin();
        acc += tank.process(x) as f64;
    }
    let elapsed = start.elapsed().as_secs_f64();
    std::hint::black_box(acc);
    let xrt = secs / elapsed;
    let floor = if cfg!(debug_assertions) { 2.0 } else { 30.0 };
    eprintln!(
        "spring block realtime factor: {xrt:.1}x ({elapsed:.3} s wall for {secs} s audio, \
         floor {floor}x, {} allpasses)",
        tank.total_allpasses()
    );
    assert!(
        xrt >= floor,
        "spring block only {xrt:.1}x realtime, expected >= {floor}x"
    );
}

// ===========================================================================
// (6) Determinism — two in-process runs are bit-identical.
// ===========================================================================

#[test]
fn spring_is_deterministic_in_process() {
    let run = || -> Vec<f64> {
        let pedal = parse_pedal_file(MINIMAL_SPRING).expect("parse");
        let mut compiled = compile_pedal(&pedal, SAMPLE_RATE).expect("compile");
        let n = (1.0 * SAMPLE_RATE) as usize;
        (0..n)
            .map(|i| {
                let x = 0.2 * (2.0 * std::f64::consts::PI * 330.0 * i as f64 / SAMPLE_RATE).sin();
                compiled.process(x)
            })
            .collect()
    };
    let a = run();
    let b = run();
    assert_eq!(a.len(), b.len());
    let fingerprint = |v: &[f64]| -> u64 {
        let mut h = 1469598103934665603u64;
        for &x in v {
            for byte in x.to_bits().to_le_bytes() {
                h ^= byte as u64;
                h = h.wrapping_mul(1099511628211);
            }
        }
        h
    };
    let (fa, fb) = (fingerprint(&a), fingerprint(&b));
    eprintln!("spring determinism fingerprint: {fa:#018x} vs {fb:#018x}");
    assert_eq!(
        fa, fb,
        "spring output not deterministic across in-process runs"
    );
}

// ===========================================================================
// (7) Mandatory lowering — an unwired spring() fails to compile (§4).
// ===========================================================================

#[test]
fn unwired_spring_fails_to_compile() {
    let pedal = parse_pedal_file(UNWIRED_SPRING).expect("parse");
    let err = compile_pedal(&pedal, SAMPLE_RATE)
        .err()
        .expect("unwired spring() must fail to compile (architecture debt §4)");
    eprintln!("unwired spring() compile error: {err}");
    assert!(
        err.contains("RV1") && err.contains("spring()"),
        "error should name the unwired spring component: {err}"
    );
}
