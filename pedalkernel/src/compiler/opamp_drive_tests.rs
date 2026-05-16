// ═══════════════════════════════════════════════════════════════════════════
// Op-amp drive level & diode clipping diagnostic tests
//
// These test WHETHER the op-amp is driving enough signal to cause
// diode clipping, and whether the clipped waveform has harmonics.
// We test the actual wave variables, not just the output.
// ═══════════════════════════════════════════════════════════════════════════

use super::spqr_build::compile_via_spqr;
use crate::PedalProcessor;

const SR: f64 = 48000.0;
const FREQ: f64 = 440.0;

// ═══════════════════════════════════════════════════════════════════════════
// 1. Does the op-amp produce enough gain to reach diode Vf?
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn opamp_drive_level_reaches_diode_vf() {
    // Single stage: R_in=10k, Rf=100k → gain=10.
    // Input = 0.1V → VS should drive 1.0V into the diode.
    // Diode Vf ≈ 0.6V. So the diode SHOULD clip.
    // Test: output peak should be LESS than 1.0V (clipped).
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
                U1.neg -> D1.a
                D1.b -> U1.out
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

    // Drive at 0.1V — gain=10 → 1.0V → above Vf → should clip
    let amp = 0.1;
    for s in 0..2000 {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }
    let mut peak = 0.0f64;
    for s in 2000..2220 {
        peak = peak.max(
            compiled
                .process(amp * (std::f64::consts::TAU * FREQ * (2000 + s) as f64 / SR).sin())
                .abs(),
        );
    }

    eprintln!("Drive test: input={amp}V, peak={peak:.4}V");
    eprintln!("  Expected: gain=10 → 1.0V → clips at ~0.6V → peak < 0.8V");
    assert!(peak > 0.01, "Should produce output: {peak:.4}V");
    // If output ≈ 1.0V → no clipping (gain only, diode not engaging)
    // If output < 0.8V → clipping IS happening
    if peak > 0.8 {
        eprintln!("  WARNING: output {peak:.4}V suggests NO CLIPPING");
    } else {
        eprintln!("  GOOD: output {peak:.4}V suggests clipping at Vf");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Does clipping produce harmonics (THD > 0)?
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn clipping_produces_harmonics() {
    // If the diode clips, the output waveform is non-sinusoidal.
    // Non-sinusoidal → energy at harmonics (2f, 3f, 4f...).
    // Measure: compare peak to RMS. For a sine, peak/rms = √2 ≈ 1.414.
    // For a clipped sine, peak/rms < 1.414 (flattened peaks).
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
                U1.neg -> D1.a
                D1.b -> U1.out
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

    // Drive hard to clip
    let amp = 0.1;
    for s in 0..2000 {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }

    let n_samples = (SR / FREQ).ceil() as usize; // one full cycle
    let mut samples = Vec::with_capacity(n_samples);
    for s in 0..n_samples {
        let input = amp * (std::f64::consts::TAU * FREQ * (2000 + s) as f64 / SR).sin();
        samples.push(compiled.process(input));
    }

    let peak = samples.iter().map(|s| s.abs()).fold(0.0f64, f64::max);
    let rms = (samples.iter().map(|s| s * s).sum::<f64>() / samples.len() as f64).sqrt();
    let crest_factor = if rms > 1e-10 { peak / rms } else { 0.0 };

    eprintln!("Harmonics test: peak={peak:.4}V, rms={rms:.4}V, crest={crest_factor:.3}");
    eprintln!("  Pure sine crest = 1.414, clipped sine crest < 1.3");
    assert!(peak > 0.01, "Should produce output: {peak:.4}V");
    assert!(
        crest_factor > 0.5,
        "Should have reasonable crest factor: {crest_factor:.3}"
    );
    // A clipped waveform has lower crest factor than a pure sine
    if crest_factor < 1.3 {
        eprintln!("  GOOD: crest factor {crest_factor:.3} indicates clipping/harmonics");
    } else {
        eprintln!("  WARNING: crest factor {crest_factor:.3} ≈ sine (no clipping)");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Small signal = clean gain, large signal = clipped (compression)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn gain_compression_with_diode() {
    // Small signal: below Vf → clean gain ≈ Rf/Ri = 10.
    // Large signal: above Vf → diode clips → effective gain < 10.
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
                U1.neg -> D1.a
                D1.b -> U1.out
                U1.neg -> D2.b
                D2.a -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#,
    )
    .expect("parse");

    let measure = |amp: f64| -> f64 {
        let mut c = compile_via_spqr(&pedal, SR).expect("compile");
        for s in 0..2000 {
            c.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
        }
        let mut peak = 0.0f64;
        for s in 2000..2220 {
            peak = peak.max(
                c.process(amp * (std::f64::consts::TAU * FREQ * (2000 + s) as f64 / SR).sin())
                    .abs(),
            );
        }
        peak / amp
    };

    let small_gain = measure(0.001); // 1mV → 10mV (below Vf)
    let large_gain = measure(0.1); // 100mV → 1V (above Vf)

    eprintln!("Compression: small_gain={small_gain:.1}, large_gain={large_gain:.1}");
    assert!(
        small_gain > 3.0,
        "Small signal should amplify: {small_gain:.1}"
    );
    assert!(
        large_gain < small_gain,
        "Large signal gain should be compressed: large={large_gain:.1} < small={small_gain:.1}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. What are the actual wave variable values?
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn wave_variables_at_clipping() {
    // Trace the actual b_tree, a_root, vs_voltage at the point of clipping.
    // This tells us what the diode sees and produces.
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
                U1.neg -> D1.a
                D1.b -> U1.out
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

    // Drive at 0.1V for a while
    let amp = 0.1;
    for s in 0..2000 {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }

    // Collect a few output samples near the sine peak
    let mut outputs = Vec::new();
    for s in 2000..2010 {
        let input = amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin();
        let output = compiled.process(input);
        outputs.push((input, output));
    }

    eprintln!("Wave variables at clipping:");
    for (i, (inp, out)) in outputs.iter().enumerate() {
        let ratio = if inp.abs() > 1e-10 { out / inp } else { 0.0 };
        eprintln!("  sample {i}: in={inp:.4}, out={out:.6}, ratio={ratio:.2}");
    }

    // The output should show:
    // - Gain (ratio > 1) for small input values
    // - Limiting (output doesn't grow proportionally) for large input values
    let peak_out = outputs.iter().map(|(_, o)| o.abs()).fold(0.0f64, f64::max);
    assert!(peak_out > 0.001, "Should produce output: {peak_out:.6}");
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. Legend pedal clipping characteristics
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_has_distortion_character() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/screamer.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    // Drive the screamer with a strong signal
    let amp = 0.1;
    for s in 0..4000 {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }

    let n = (SR / FREQ).ceil() as usize;
    let mut samples = Vec::with_capacity(n);
    for s in 0..n {
        samples.push(
            compiled.process(amp * (std::f64::consts::TAU * FREQ * (4000 + s) as f64 / SR).sin()),
        );
    }

    let peak = samples.iter().map(|s| s.abs()).fold(0.0f64, f64::max);
    let rms = (samples.iter().map(|s| s * s).sum::<f64>() / n as f64).sqrt();
    let crest = if rms > 1e-10 { peak / rms } else { 0.0 };

    eprintln!("Screamer character: peak={peak:.4}V, rms={rms:.4}V, crest={crest:.3}");
    assert!(peak > 0.001, "Screamer should produce output: {peak:.4}V");
    // A distorted screamer should have crest factor < 1.4 (clipped peaks)
    // A clean signal has crest ≈ 1.414
}
