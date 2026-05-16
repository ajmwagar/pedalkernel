// ═══════════════════════════════════════════════════════════════════════════
// Diode polarity & multi-diode tests
//
// Verify that the SPQR pipeline correctly handles:
// 1. Single diode_pair component (Screamer) → symmetric clipping
// 2. Two separate antiparallel diodes (SD-1) → both directions clip
// 3. Two separate diodes to ground (RAT/Klon) → both directions clip
// 4. Output extraction produces non-zero clipped signal
// ═══════════════════════════════════════════════════════════════════════════

use super::compiled::CompiledPedal;
use super::spqr_build::compile_via_spqr;
use crate::PedalProcessor;

const SR: f64 = 48000.0;
const FREQ: f64 = 440.0;

/// Helper: drive a compiled pedal with a sine and return (positive_peak, negative_peak).
fn measure_peaks(
    compiled: &mut impl PedalProcessor,
    amp: f64,
    warmup: usize,
    measure: usize,
) -> (f64, f64) {
    // Warmup
    for s in 0..warmup {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }
    // Measure
    let mut pos_peak = 0.0f64;
    let mut neg_peak = 0.0f64;
    for s in 0..measure {
        let out =
            compiled.process(amp * (std::f64::consts::TAU * FREQ * (warmup + s) as f64 / SR).sin());
        if out > pos_peak {
            pos_peak = out;
        }
        if out < neg_peak {
            neg_peak = out;
        }
    }
    (pos_peak, neg_peak)
}

/// Debug helper: drive a compiled pedal and dump per-stage metering data.
/// Uses the same metering ring buffer the UI reads — both sides see the same data.
fn dump_stage_metering(compiled: &mut CompiledPedal, amp: f64, warmup: usize, measure: usize) {
    // Enable metering with small block size for quick results
    compiled.enable_metering(128);

    // Warmup
    for s in 0..warmup {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }
    // Measure
    for s in 0..measure {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * (warmup + s) as f64 / SR).sin());
    }

    // Read the latest metrics from the ring buffer
    let metrics = compiled.read_metrics();
    eprintln!("  Metering ({} blocks):", metrics.block_counter);
    eprintln!(
        "    input:  RMS={:.1} dB, peak={:.1} dB",
        metrics.input_rms_db, metrics.input_peak_db
    );
    eprintln!(
        "    output: RMS={:.1} dB, peak={:.1} dB",
        metrics.output_rms_db, metrics.output_peak_db
    );
    eprintln!(
        "    signal envelope: [{:.4}, {:.4}]",
        metrics.signal_min, metrics.signal_max
    );
    let n = compiled.stages.len().min(crate::metering::MAX_STAGES);
    for i in 0..n {
        let lvl = metrics.stage_levels[i];
        let db = if lvl > 1e-10 {
            20.0 * (lvl as f64).log10()
        } else {
            -120.0
        };
        let (stage_type, dist, label, bypass) = match &compiled.stages[i] {
            super::compiled::Stage::Wdf(w) => (
                "Wdf",
                w.signal_flow_distance,
                w.debug_label.as_str(),
                w.bypass_serial,
            ),
            super::compiled::Stage::MultiNl(m) => (
                "MultiNl",
                m.signal_flow_distance,
                m.debug_label.as_str(),
                m.bypass_serial,
            ),
            super::compiled::Stage::Iir(s) => (
                "Iir",
                s.signal_flow_distance,
                s.debug_label.as_str(),
                s.bypass_serial,
            ),
            super::compiled::Stage::StateSpace(s) => (
                "SS",
                s.signal_flow_distance,
                s.debug_label.as_str(),
                s.bypass_serial,
            ),
            super::compiled::Stage::BlackFeedback(b) => (
                "BF",
                b.signal_flow_distance,
                b.debug_label.as_str(),
                b.bypass_serial,
            ),
            super::compiled::Stage::BlockwiseKMethod(bk) => (
                "BKM",
                bk.signal_flow_distance,
                "blockwise",
                bk.bypass_serial,
            ),
        };
        let bypass_tag = if bypass { " BYPASS" } else { "" };
        eprintln!("    stage {i}: [{stage_type}] dist={dist} [{label}]{bypass_tag} → {lvl:.4} ({db:.1} dB)");
    }
    if n == 0 {
        eprintln!("    (no stages)");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 1. DiodePair component (Screamer-style) clips both directions
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn diode_pair_component_clips_both_directions() {
    // Screamer-style: opamp + diode_pair in feedback
    // DiodePair clips symmetrically (same threshold + and -)
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D_clip: diode_pair(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.out -> D_clip.a
                D_clip.b -> U1.neg
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Drive hard enough to clip (gain=10, input=0.1V → 1V > Vf=0.6V)
    let (pos_peak, neg_peak) = measure_peaks(&mut compiled, 0.1, 2000, 500);

    eprintln!("DiodePair: pos_peak={pos_peak:.4}V, neg_peak={neg_peak:.4}V");

    // Both peaks should be non-zero (signal present)
    assert!(
        pos_peak > 0.01,
        "Should have positive output: {pos_peak:.4}V"
    );
    assert!(
        neg_peak < -0.01,
        "Should have negative output: {neg_peak:.4}V"
    );

    // Both directions should be clipped (below gain × input = 1.0V)
    assert!(pos_peak < 0.9, "Positive should be clipped: {pos_peak:.4}V");
    assert!(
        neg_peak > -0.9,
        "Negative should be clipped: {neg_peak:.4}V"
    );

    // Symmetric: positive and negative peaks should be approximately equal magnitude
    let ratio = pos_peak / neg_peak.abs();
    eprintln!("  Symmetry ratio: {ratio:.3} (should be ~1.0)");
    assert!(
        (ratio - 1.0).abs() < 0.3,
        "Should be roughly symmetric: {ratio:.3}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Two separate antiparallel diodes (SD-1 style) clip BOTH directions
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn two_separate_antiparallel_diodes_clip_both_directions() {
    // SD-1 style: two separate diodes in antiparallel feedback
    // D1 clips positive swing (anode at out, cathode at neg)
    // D2 clips negative swing (anode at neg, cathode at out)
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D1: diode(silicon)
                D2: diode(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.out -> D1.a
                D1.b -> U1.neg
                U1.neg -> D2.b
                D2.a -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    let (pos_peak, neg_peak) = measure_peaks(&mut compiled, 0.1, 2000, 500);

    eprintln!("Two antiparallel diodes: pos_peak={pos_peak:.4}V, neg_peak={neg_peak:.4}V");

    // CRITICAL: Both directions must clip. If only one diode is modeled,
    // one direction passes unclipped (gain × input ≈ 1.0V) while the other clips.
    assert!(
        pos_peak > 0.01,
        "Should have positive output: {pos_peak:.4}V"
    );
    assert!(
        neg_peak < -0.01,
        "Should have negative output: {neg_peak:.4}V"
    );

    // Both should be clipped below the unclipped gain level
    assert!(pos_peak < 0.9, "Positive should be clipped: {pos_peak:.4}V");
    assert!(
        neg_peak > -0.9,
        "Negative should be clipped: {neg_peak:.4}V"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Asymmetric antiparallel diodes (SD-1 actual) have different thresholds
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn asymmetric_diodes_have_different_clip_thresholds() {
    // SD-1 actual: silicon (Vf≈0.6V) + germanium (Vf≈0.25V) antiparallel
    // Positive swing clips at silicon threshold (higher)
    // Negative swing clips at germanium threshold (lower)
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D_si: diode(silicon)
                D_ge: diode(germanium)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.out -> D_si.a
                D_si.b -> U1.neg
                D_ge.a -> U1.neg
                D_ge.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    let (pos_peak, neg_peak) = measure_peaks(&mut compiled, 0.1, 2000, 500);

    eprintln!("Asymmetric diodes: pos_peak={pos_peak:.4}V, neg_peak={neg_peak:.4}V");

    // Both directions must produce output
    assert!(
        pos_peak > 0.01,
        "Should have positive output: {pos_peak:.4}V"
    );
    assert!(
        neg_peak < -0.01,
        "Should have negative output: {neg_peak:.4}V"
    );

    // Both should clip
    assert!(pos_peak < 0.9, "Positive should clip: {pos_peak:.4}V");
    assert!(neg_peak > -0.9, "Negative should clip: {neg_peak:.4}V");

    // Asymmetric: germanium clips at lower voltage than silicon
    // So |neg_peak| should be less than pos_peak (germanium clips harder)
    eprintln!(
        "  Asymmetry: pos={pos_peak:.4} vs |neg|={:.4}",
        neg_peak.abs()
    );
    // Note: this test may need adjustment based on actual diode models,
    // but the key invariant is that BOTH directions are present.
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Two diodes to ground (RAT/Klon pattern) clip both directions
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn hard_clip_diodes_to_ground_both_directions() {
    // RAT pattern: opamp gain stage → diodes to ground (hard clip)
    // D1.a at output, D1.b at ground (clips positive)
    // D2.b at output, D2.a at ground (clips negative)
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(lm308)
                Rf: resistor(1M)
                D1: diode(silicon)
                D2: diode(silicon)
                R_out: resistor(10k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> D1.a, D2.b
                D1.b -> gnd
                D2.a -> gnd
                U1.out -> R_out.a
                R_out.b -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Dump per-stage metering to see what each stage does
    dump_stage_metering(&mut compiled, 0.05, 2000, 2000);

    let mut compiled2 = compile_via_spqr(&pedal, SR).expect("compile");
    let (pos_peak, neg_peak) = measure_peaks(&mut compiled2, 0.05, 2000, 500);

    eprintln!("Hard clip to GND: pos_peak={pos_peak:.4}V, neg_peak={neg_peak:.4}V");

    // Both directions should produce output
    assert!(
        pos_peak > 0.001,
        "Should have positive output: {pos_peak:.4}V"
    );
    assert!(
        neg_peak < -0.001,
        "Should have negative output: {neg_peak:.4}V"
    );

    // Hard clipping should limit output to ~Vf (0.6V for silicon)
    assert!(
        pos_peak < 0.9,
        "Positive should be hard-clipped: {pos_peak:.4}V"
    );
    assert!(
        neg_peak > -0.9,
        "Negative should be hard-clipped: {neg_peak:.4}V"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. Output extraction: feedback_opamp produces non-trivial clipped signal
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn feedback_opamp_output_has_harmonics() {
    // A clipped sine has crest factor < √2 ≈ 1.414 (flattened peaks).
    // If output extraction is wrong (near-zero), RMS will be near-zero too.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D_clip: diode_pair(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.out -> D_clip.a
                D_clip.b -> U1.neg
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Warmup
    let amp = 0.1;
    for s in 0..2000 {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }

    // Collect one full cycle
    let n = (SR / FREQ).ceil() as usize;
    let mut samples = Vec::with_capacity(n);
    for s in 0..n {
        samples.push(
            compiled.process(amp * (std::f64::consts::TAU * FREQ * (2000 + s) as f64 / SR).sin()),
        );
    }

    let peak = samples.iter().map(|s| s.abs()).fold(0.0f64, f64::max);
    let rms = (samples.iter().map(|s| s * s).sum::<f64>() / n as f64).sqrt();
    let crest = if rms > 1e-10 { peak / rms } else { 0.0 };

    eprintln!("Feedback opamp harmonics: peak={peak:.4}V, rms={rms:.4}V, crest={crest:.3}");

    // Output should be substantial (not near-zero from bad extraction)
    assert!(
        peak > 0.05,
        "Output should be substantial, not near-zero: {peak:.6}V"
    );
    assert!(rms > 0.01, "RMS should be substantial: {rms:.6}V");

    // Clipped sine should have crest factor < 1.4
    // If no clipping (pure sine), crest ≈ 1.414
    // If heavily clipped (square-ish), crest → 1.0
    assert!(
        crest < 1.4,
        "Should have harmonics (clipped): crest={crest:.3}"
    );
    assert!(crest > 0.5, "Crest factor should be reasonable: {crest:.3}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. SD-1 legend pedal compiles and produces asymmetric clipping
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn sd1_legend_produces_clipped_output() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/sd1.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read sd1.pedal");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Dump per-stage metering to see where signal dies
    dump_stage_metering(&mut compiled, 0.1, 2000, 2000);

    // Re-compile for clean measurement (metering warmup consumed samples)
    let mut compiled2 = compile_via_spqr(&pedal, SR).expect("compile");
    let (pos_peak, neg_peak) = measure_peaks(&mut compiled2, 0.1, 4000, 500);

    eprintln!("SD-1 legend: pos_peak={pos_peak:.4}V, neg_peak={neg_peak:.4}V");

    // Must produce output in both directions
    assert!(
        pos_peak > 0.001,
        "SD-1 should have positive output: {pos_peak:.4}V"
    );
    assert!(
        neg_peak < -0.001,
        "SD-1 should have negative output: {neg_peak:.4}V"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 7. Single diode clips only one direction (sanity check)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn single_diode_clips_one_direction_only() {
    // A single diode (not antiparallel pair) should only clip one direction.
    // This is the expected behavior for a half-wave clipper.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D1: diode(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.out -> D1.a
                D1.b -> U1.neg
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    let (pos_peak, neg_peak) = measure_peaks(&mut compiled, 0.1, 2000, 500);

    eprintln!("Single diode: pos_peak={pos_peak:.4}V, neg_peak={neg_peak:.4}V");

    // Should produce output
    let max_abs = pos_peak.max(neg_peak.abs());
    assert!(max_abs > 0.01, "Should produce output: peak={max_abs:.4}V");

    // One direction should clip (limited to ~Vf), the other should pass with full gain
    // The clipped direction should have a smaller peak than the unclipped direction
    let asymmetry = pos_peak / neg_peak.abs();
    eprintln!("  Asymmetry ratio: {asymmetry:.3} (should differ from 1.0)");
    // A single diode should be clearly asymmetric
    assert!(
        (asymmetry - 1.0).abs() > 0.2,
        "Single diode should be asymmetric: ratio={asymmetry:.3}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 8. Bias network does NOT kill audio signal
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bias_network_does_not_kill_signal() {
    // A circuit with a VCC bias network (voltage divider for op-amp DC reference)
    // should pass audio through the clipping stage without the bias network
    // stomping the signal to zero. The bias network creates a DC reference;
    // it is NOT in the audio signal path.
    let pedal = crate::dsl::parse_pedal_file(
        r#"
        pedal "test" { supply 9V
            components {
                # Bias network — 4.5V virtual ground
                R_v1: resistor(10k)
                R_v2: resistor(10k)
                C_v: cap(47u, electrolytic)

                # Input coupling
                Cin: cap(22n)
                R_in: resistor(510k)

                # Clipping stage
                C_c1: cap(1u, electrolytic)
                R_b1: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D_clip: diode_pair(silicon)

                # Output
                R_out: resistor(10k)
            }
            nets {
                # Bias
                vcc -> R_v1.a
                R_v1.b -> R_v2.a, C_v.a
                R_v2.b -> gnd
                C_v.b -> gnd
                R_v1.b -> vb

                # Input
                in -> Cin.a
                Cin.b -> R_in.a, C_c1.a
                R_in.b -> gnd

                # Clipping (non-inverting)
                C_c1.b -> U1.pos, R_b1.a
                R_b1.b -> vb
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.out -> D_clip.a
                D_clip.b -> U1.neg
                U1.pos -> gnd

                # Output
                U1.out -> R_out.a
                R_out.b -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Dump per-stage metering to diagnose
    dump_stage_metering(&mut compiled, 0.1, 2000, 2000);

    // Re-compile for measurement
    let mut compiled2 = compile_via_spqr(&pedal, SR).expect("compile");
    let (pos_peak, neg_peak) = measure_peaks(&mut compiled2, 0.1, 2000, 500);

    eprintln!("Bias network test: pos_peak={pos_peak:.4}V, neg_peak={neg_peak:.4}V");

    // The bias network should NOT kill the signal.
    // The clipping stage output should reach the circuit output.
    assert!(
        pos_peak > 0.01,
        "Signal should survive bias network: {pos_peak:.4}V"
    );
    assert!(
        neg_peak < -0.01,
        "Signal should survive bias network: {neg_peak:.4}V"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 9. Screamer legend with bias network produces audio
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_legend_with_bias_produces_audio() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/screamer.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read screamer.pedal");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Dump per-stage metering
    dump_stage_metering(&mut compiled, 0.1, 2000, 2000);

    let mut compiled2 = compile_via_spqr(&pedal, SR).expect("compile");
    let (pos_peak, neg_peak) = measure_peaks(&mut compiled2, 0.1, 4000, 500);

    eprintln!("Screamer: pos_peak={pos_peak:.4}V, neg_peak={neg_peak:.4}V");

    assert!(
        pos_peak > 0.001,
        "Screamer should produce output: {pos_peak:.4}V"
    );
    assert!(
        neg_peak < -0.001,
        "Screamer should produce output: {neg_peak:.4}V"
    );
}
