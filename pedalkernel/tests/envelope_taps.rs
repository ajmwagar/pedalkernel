//! Envelope follower detector TAP tests (audit gap G1).
//!
//! The DSL wires a detector's input explicitly (`EF1.in -> <node>`), but the
//! runtime historically fed EVERY envelope follower the global pedal input,
//! so feed-forward and feedback detector topologies measured identically.
//! These tests pin down the fix: the tap net must select the signal the
//! detector actually sees.
//!
//! Fixture: a minimal 1176-style JFET shunt compressor — series R into a
//! J201 shunt (gate driven by EF1), then a non-inverting op-amp makeup
//! stage (~x11) — in two variants that differ ONLY in the `EF1.in` net:
//!
//!   * INPUT tap   (`EF1.in -> C_in.b`): feed-forward detector, reads the
//!     program before any gain reduction or makeup.
//!   * OUTPUT tap  (`EF1.in -> C_out.b`, the `out` node): feedback detector
//!     (1176 rev D style), reads the post-makeup output, so detector drive
//!     includes makeup gain minus the gain reduction itself.
//!
//! With the tap honored, the two variants must produce measurably different
//! static gain curves. Before the fix both detectors read the global input
//! and the curves are identical — the difference assertion is the RED test.

mod audio_analysis;

use audio_analysis::*;

/// Minimal JFET-shunt compressor; `{tap}` is the node the detector reads.
///
/// GR core: 3.3k series into a J201 shunt (Rds(on) ≈ 920 Ω, |VTO| 0.69 V —
/// large Rds swing over the envelope sink's Vgs range). Makeup: ideal
/// non-inverting op-amp, 1 + 100k/10k = x11 (~+20.8 dB), cap-coupled in and
/// out (the vca_bus_comp-proven pattern: cap straight into `pos` with a
/// bias return).
fn shunt_compressor_src(tap: &str) -> String {
    format!(
        r#"
pedal "Envelope Tap Fixture" {{
  supply 9V
  components {{
    C_in: cap(100n)
    R_s: resistor(3.3k)
    J1: njfet(j201)
    R_gb: resistor(1M)
    C_c: cap(100n)
    R_b: resistor(1M)
    U1: opamp(tl072)
    R_g: resistor(10k)
    R_f: resistor(100k)
    C_out: cap(100n)
    R_l: resistor(100k)
    EF1: envelope_follower(10k, 1u, 100k, 10u, 20k)
  }}
  nets {{
    in -> C_in.a
    C_in.b -> R_s.a
    R_s.b -> J1.drain, C_c.a
    J1.source -> gnd
    J1.gate -> R_gb.a
    R_gb.b -> gnd
    C_c.b -> R_b.a, U1.pos
    R_b.b -> gnd
    U1.neg -> R_g.a
    R_g.b -> gnd
    R_f.a -> U1.neg
    R_f.b -> U1.out
    U1.out -> C_out.a
    C_out.b -> R_l.a, out
    R_l.b -> gnd
    EF1.in -> {tap}
    EF1.out -> J1.vgs
  }}
  controls {{}}
}}
"#
    )
}

/// Static gain curve (input dB → gain dB) over a -40..0 dB sweep at 1 kHz.
fn gain_curve(src: &str) -> Vec<(f64, f64)> {
    let levels: Vec<f64> = (0..=8).map(|i| -40.0 + 5.0 * i as f64).collect();
    static_gain_curve(
        || pedal_processor(src, SAMPLE_RATE, &[]),
        &levels,
        1_000.0,
        SAMPLE_RATE,
    )
    .into_iter()
    .map(|(in_db, out_db)| (in_db, out_db - in_db))
    .collect()
}

/// [G1] The same circuit with the detector tapping the circuit INPUT vs the
/// circuit OUTPUT (post-makeup) must measure differently: the output tap
/// sees makeup gain minus the gain reduction itself (feedback detector), so
/// at equal input level its detector drive — and therefore its gain — must
/// diverge from the feed-forward variant by > 0.5 dB somewhere on the
/// sweep. Today both detectors read the global pedal input, so the curves
/// are bit-identical and this test is RED.
#[test]
fn input_and_output_taps_produce_different_gain_curves() {
    let input_tap_src = shunt_compressor_src("C_in.b");
    let output_tap_src = shunt_compressor_src("C_out.b");

    let ff_curve = gain_curve(&input_tap_src);
    let fb_curve = gain_curve(&output_tap_src);

    let mut max_delta = 0.0f64;
    for (&(in_db, ff_gain), &(_, fb_gain)) in ff_curve.iter().zip(fb_curve.iter()) {
        let delta = (ff_gain - fb_gain).abs();
        max_delta = max_delta.max(delta);
        eprintln!(
            "tap curves: in {in_db:6.1} dB -> input-tap gain {ff_gain:8.3} dB, \
             output-tap gain {fb_gain:8.3} dB (delta {delta:6.3} dB)"
        );
        assert!(
            ff_gain.is_finite() && fb_gain.is_finite(),
            "non-finite gain at {in_db} dB input"
        );
    }
    eprintln!("max input-tap vs output-tap gain delta: {max_delta:.3} dB");

    assert!(
        max_delta > 0.5,
        "input-tapped and output-tapped detectors measure identically \
         (max gain delta {max_delta:.3} dB, expected > 0.5 dB) — \
         envelope follower tap nets are being ignored"
    );
}

/// Sanity for the fixture itself: the feed-forward (input-tap) variant must
/// actually compress — otherwise the difference test above could pass green
/// for the wrong reason (e.g. one variant silent). Uses the same thresholds
/// style as outboard_acceptance but on the minimal fixture.
#[test]
fn input_tap_fixture_compresses() {
    let src = shunt_compressor_src("C_in.b");
    let curve = gain_curve(&src);
    for &(in_db, gain_db) in &curve {
        eprintln!("input-tap fixture: in {in_db:6.1} dB -> gain {gain_db:8.3} dB");
    }
    let quiet_gain = curve.first().unwrap().1;
    let loud_gain = curve.last().unwrap().1;
    let gr = quiet_gain - loud_gain;
    eprintln!("input-tap fixture GR (-40 vs 0 dB): {gr:.3} dB");
    assert!(
        gr > 1.0,
        "fixture does not compress: GR {gr:.3} dB between -40 and 0 dB input, expected > 1 dB"
    );
}
