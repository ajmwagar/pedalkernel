//! Transformer-secondary load reflection (GAP F / pedalkernel-lkf1.4).
//!
//! A transformer's PRIMARY-side stage never felt a load on its SECONDARY:
//! the windings couple magnetically (`coupled_nodes`), not conductively, so a
//! loaded secondary always split into its own flow group and the primary-side
//! solve terminated the transformer with an OPEN secondary while the
//! standalone load stage divided by an arbitrary source resistance. The
//! boundary-load reflection fuses the secondary load edges into the
//! primary-side `PassiveRType` MNA (full transformer skeleton with real
//! secondary nodes), so the primary feels `n²·R` through the ideal
//! turns-ratio stamp.
//!
//! LEVEL ORACLE (ngspice 45.2, coupled inductors, 2026-07-07): for
//! `V_ideal → Cout(10µ) → L1(2H) ⊗ k=0.99 ⊗ L2(20mH) → RL(1kΩ)` at 1 kHz,
//! `|V(out)/V(in)| = 0.1021` (deck: Cout v_in np 10u / L1 np 0 2H /
//! L2 v_out 0 20m / K1 L1 L2 0.99 / RL v_out 0 1k; .tran 2u 0.1, p-p over
//! the last 20 ms). Before the reflection the same chain measured ≈ 0.28×
//! (+8.8 dB high); with it the WDF measures 0.102× (< 0.1 dB from ngspice).

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;

/// Op-amp unity follower driving the LA-2A `T_out` shape: cap-coupled 10:1
/// step-down output transformer, grounded secondary, 1 kΩ load at `out`.
/// The follower isolates the transformer + load transfer (the thing under
/// test) from upstream-stage gain accuracy.
const BUF_XFMR_OUT: &str = r#"
pedal "buffer xfmr out" {
  supply 9V
  components {
    U1: opamp(tl072)
    Cout: cap(10u, electrolytic)
    T1: transformer(10:1, 2H)
    RL: resistor(1k)
  }
  nets {
    in -> U1.pos
    U1.out -> U1.neg, Cout.a
    Cout.b -> T1.a
    T1.b -> gnd
    T1.c -> out, RL.a
    T1.d -> gnd
    RL.b -> gnd
  }
}
"#;

const SAMPLE_RATE: f64 = 48_000.0;

/// Steady-state 1 kHz peak-to-peak gain (last 20 of 100 cycles).
fn gain_at_1k(proc: &mut impl FnMut(f64) -> f64, amplitude: f64) -> f64 {
    let freq = 1_000.0;
    let spc = (SAMPLE_RATE / freq) as usize;
    let (mut max_out, mut min_out) = (0.0f64, 0.0f64);
    for i in 0..(100 * spc) {
        let t = i as f64 / SAMPLE_RATE;
        let input = amplitude * (2.0 * std::f64::consts::PI * freq * t).sin();
        let output = proc(input);
        if i >= 80 * spc {
            max_out = max_out.max(output);
            min_out = min_out.min(output);
        }
    }
    (max_out - min_out) / (2.0 * amplitude)
}

/// LEVEL vs ngspice: the reflected transformer + load transfer must match the
/// coupled-inductor oracle (0.1021× at 1 kHz) — the pre-fix chain measured
/// ≈ 0.28× (the load stage divided by an arbitrary 10 kΩ source resistance
/// and the primary solved against an open secondary).
#[test]
fn reflected_transformer_matches_ngspice_level() {
    let def = parse_pedal_file(BUF_XFMR_OUT).expect("parse");
    let compiled = compile_pedal(&def, SAMPLE_RATE).expect("compile");

    // Structural: the reflection fired and recorded its row.
    let row = compiled
        .boundary_loads
        .iter()
        .find(|b| {
            matches!(
                b.disposition,
                pedalkernel_rt::processor::BoundaryLoadDisposition::ReflectedThroughTransformer { .. }
            )
        })
        .unwrap_or_else(|| {
            panic!(
                "expected a ReflectedThroughTransformer boundary-load row, got {:?}",
                compiled.boundary_loads
            )
        });
    match &row.disposition {
        pedalkernel_rt::processor::BoundaryLoadDisposition::ReflectedThroughTransformer {
            turns_ratio,
            r_reflected,
        } => {
            assert_eq!(*turns_ratio, 10.0);
            assert!((r_reflected - 100_000.0).abs() < 1e-6);
        }
        _ => unreachable!(),
    }

    let mut proc = compiled;
    let gain = gain_at_1k(&mut |x| proc.process(x), 0.01);
    let gain_db = 20.0 * gain.log10();
    let oracle_db = 20.0 * 0.1021f64.log10();
    eprintln!(
        "buffer→10:1 OT→1k @1kHz: WDF {gain:.4}x ({gain_db:+.2} dB) vs ngspice 0.1021x \
         ({oracle_db:+.2} dB)"
    );
    assert!(
        (gain_db - oracle_db).abs() < 1.0,
        "reflected transformer transfer must match the ngspice coupled-inductor \
         oracle within 1 dB: WDF {gain_db:+.2} dB vs oracle {oracle_db:+.2} dB"
    );
}
