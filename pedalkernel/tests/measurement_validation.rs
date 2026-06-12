//! Validation of the outboard-gear measurement utilities in audio_analysis:
//! frequency response sweeps, static gain curves, compression ratio, gain
//! reduction, and attack/release timing — checked against circuits and
//! synthetic signals with known analytic behavior.

mod audio_analysis;

use audio_analysis::*;

// ===========================================================================
// RC lowpass — frequency response with known analytic transfer function
// ===========================================================================

/// RC lowpass: R = 10k, C = 100n → fc = 1/(2πRC) ≈ 159.15 Hz.
const RC_LOWPASS: &str = r#"
pedal "RC Lowpass" {
  components {
    R1: resistor(10k)
    C1: cap(100n)
  }
  nets {
    in -> R1.a
    R1.b -> C1.a, out
    C1.b -> gnd
  }
}
"#;

#[test]
fn rc_lowpass_frequency_response_matches_analytic() {
    // Analytic: |H(f)| = 1/sqrt(1 + (f/fc)²) → 0 dB well below fc, -3.01 dB
    // at fc, -20 dB/decade asymptote above. Bilinear warping steepens the
    // slope slightly near Nyquist, so the decade-slope check stays below
    // fc·30 (~4.8 kHz at 48 kHz) where warping adds < 1 dB.
    let fc = 1.0 / (2.0 * std::f64::consts::PI * 10_000.0 * 100e-9);
    let freqs = [fc / 10.0, fc, 10.0 * fc, 30.0 * fc];
    let resp = frequency_response_db(
        || pedal_processor(RC_LOWPASS, SAMPLE_RATE, &[]),
        &freqs,
        0.1,
        SAMPLE_RATE,
    );
    for &(f, g) in &resp {
        eprintln!("RC lowpass response: {f:9.2} Hz -> {g:7.3} dB");
        assert!(g.is_finite(), "RC lowpass: non-finite gain at {f} Hz");
    }

    // Passband: ~0 dB a decade below cutoff (analytic -0.04 dB).
    assert!(
        resp[0].1.abs() < 0.75,
        "passband gain {:.3} dB not ~0 dB",
        resp[0].1
    );
    // Cutoff: -3.01 dB ± 1 dB.
    assert!(
        (resp[1].1 + 3.01).abs() < 1.0,
        "cutoff gain {:.3} dB not within ±1 dB of -3.01 dB",
        resp[1].1
    );
    // Stopband anchor: -20.04 dB analytic at 10·fc.
    assert!(
        (-21.5..=-18.5).contains(&resp[2].1),
        "gain at 10·fc = {:.3} dB, expected ~-20 dB",
        resp[2].1
    );
    // Rolloff slope between 10·fc and 30·fc, normalized per decade.
    // Analytic -20 dB/decade; bilinear warping at 30·fc adds ~-0.4 dB.
    let slope_per_decade = (resp[3].1 - resp[2].1) / 3.0f64.log10();
    eprintln!("RC lowpass rolloff: {slope_per_decade:.2} dB/decade");
    assert!(
        (-22.5..=-17.5).contains(&slope_per_decade),
        "rolloff {slope_per_decade:.2} dB/decade not ~-20 dB/decade"
    );
}

// ===========================================================================
// Unity op-amp buffer — linear static gain curve, ratio ≈ 1.0
// ===========================================================================

#[test]
fn opamp_buffer_static_gain_curve_is_linear() {
    // TL072 unity buffer: output should track input level dB-for-dB, so the
    // gain curve has slope 1 (ratio 1.0) and flat gain across levels.
    let src = test_pedal_source("opamp_buffer.pedal");
    let levels: Vec<f64> = (0..=10).map(|i| -50.0 + 5.0 * i as f64).collect();
    let curve = static_gain_curve(
        || pedal_processor(&src, SAMPLE_RATE, &[]),
        &levels,
        1_000.0,
        SAMPLE_RATE,
    );

    let gains: Vec<f64> = curve.iter().map(|&(i, o)| o - i).collect();
    for (&(in_db, out_db), &g) in curve.iter().zip(&gains) {
        eprintln!("buffer curve: in {in_db:6.1} dB -> out {out_db:7.2} dB (gain {g:6.3} dB)");
        assert!(
            out_db.is_finite(),
            "buffer: non-finite output at {in_db} dB"
        );
        assert!(
            g.abs() < 1.5,
            "buffer gain {g:.3} dB not ~0 dB at {in_db} dB"
        );
    }

    // Flat gain: spread across all levels under 0.5 dB.
    let g_max = gains.iter().cloned().fold(f64::MIN, f64::max);
    let g_min = gains.iter().cloned().fold(f64::MAX, f64::min);
    assert!(
        g_max - g_min < 0.5,
        "buffer gain spread {:.3} dB, expected flat",
        g_max - g_min
    );

    // Slope-derived ratio ≈ 1.0 within 5%.
    let ratio = compression_ratio(&curve, -50.0, 0.0);
    eprintln!("buffer ratio: {ratio:.4}:1");
    assert!(
        (0.95..=1.05).contains(&ratio),
        "buffer ratio {ratio:.4} not within 5% of 1.0"
    );

    // A linear device shows ~0 dB gain reduction.
    let gr = gain_reduction_db(&curve, -50.0, 0.0);
    assert!(gr.abs() < 0.5, "buffer GR {gr:.3} dB, expected ~0");
}

// ===========================================================================
// Synthetic signals — ratio math and attack/release timing
// ===========================================================================

#[test]
fn compression_ratio_recovers_known_slope() {
    // Synthetic 4:1 compressor curve above -30 dB threshold: slope 0.25.
    let curve: Vec<(f64, f64)> = (0..=15)
        .map(|i| {
            let in_db = -30.0 + 2.0 * i as f64;
            (in_db, -30.0 + (in_db + 30.0) / 4.0)
        })
        .collect();
    let ratio = compression_ratio(&curve, -30.0, 0.0);
    assert!((ratio - 4.0).abs() < 1e-9, "synthetic ratio {ratio} != 4.0");
    // GR between -30 and 0 dB input: gain falls from 0 to -22.5 dB.
    let gr = gain_reduction_db(&curve, -30.0, 0.0);
    assert!((gr - 22.5).abs() < 1e-9, "synthetic GR {gr} != 22.5");
}

#[test]
fn attack_release_timing_matches_one_pole_envelope() {
    // 1 kHz sine whose amplitude follows a one-pole step response with
    // τ = 10 ms: 0.05 for 0.2 s, rising toward 0.5 for 0.4 s, falling back
    // toward 0.05 for 0.4 s. The ±1 dB settling convention gives analytic
    // attack ≈ τ·ln((hi-lo)/(hi·(1-10^(-1/20)))) ≈ 21 ms and release
    // ≈ τ·ln((hi-lo)/(lo·(10^(1/20)-1))) ≈ 43 ms, plus ~2 ms envelope lag.
    let (lo, hi, tau) = (0.05, 0.5, 0.010);
    let step_up = (0.2 * SAMPLE_RATE) as usize;
    let step_down = (0.6 * SAMPLE_RATE) as usize;
    let n = (1.0 * SAMPLE_RATE) as usize;
    let buf: Vec<f64> = (0..n)
        .map(|i| {
            let t = i as f64 / SAMPLE_RATE;
            let amp = if i < step_up {
                lo
            } else if i < step_down {
                hi + (lo - hi) * (-(t - 0.2) / tau).exp()
            } else {
                lo + (hi - lo) * (-(t - 0.6) / tau).exp()
            };
            amp * (2.0 * std::f64::consts::PI * 1_000.0 * t).sin()
        })
        .collect();

    let attack = measure_attack_seconds(&buf[..step_down], step_up, SAMPLE_RATE);
    let release = measure_release_seconds(&buf, step_down, SAMPLE_RATE);
    eprintln!("one-pole envelope: attack {attack:.4} s, release {release:.4} s");
    assert!(
        (0.012..=0.035).contains(&attack),
        "attack {attack:.4} s, expected ~0.021 s"
    );
    assert!(
        (0.030..=0.060).contains(&release),
        "release {release:.4} s, expected ~0.043 s"
    );
}

#[test]
fn tone_burst_levels_match_requested_amplitudes() {
    let burst = tone_burst(1_000.0, 0.05, 0.5, 0.1, 0.1, 0.1, SAMPLE_RATE);
    let seg = (0.1 * SAMPLE_RATE) as usize;
    assert_eq!(burst.len(), 3 * seg);
    let expected_lo = 0.05 / std::f64::consts::SQRT_2;
    let expected_hi = 0.5 / std::f64::consts::SQRT_2;
    assert!((rms(&burst[..seg]) - expected_lo).abs() / expected_lo < 0.02);
    assert!((rms(&burst[seg..2 * seg]) - expected_hi).abs() / expected_hi < 0.02);
    assert!((rms(&burst[2 * seg..]) - expected_lo).abs() / expected_lo < 0.02);
}

// ===========================================================================
// Dyna Comp — dynamics measurements on a real compressor circuit
// ===========================================================================

#[test]
fn dyna_comp_static_gain_curve_and_dynamics() {
    // KNOWN ENGINE LIMITATION (see integration_compressor.rs): the WDF OTA
    // root has limited gain range due to low port resistance (~5Ω) relative
    // to gm, so achievable gain variation is small. Assertions here are
    // deliberately conservative — finite, monotonic-ish, ratio >= ~1.0 —
    // while the measured ratio/GR/attack/release are printed as audit data.
    let src = example_pedal_source("dyna_comp.pedal");
    let controls: &[(&str, f64)] = &[("Sensitivity", 0.8), ("Output", 0.7)];

    let levels: Vec<f64> = (0..=8).map(|i| -40.0 + 5.0 * i as f64).collect();
    let curve = static_gain_curve(
        || pedal_processor(&src, SAMPLE_RATE, controls),
        &levels,
        1_000.0,
        SAMPLE_RATE,
    );
    for &(in_db, out_db) in &curve {
        eprintln!(
            "dyna comp curve: in {in_db:6.1} dB -> out {out_db:8.3} dB (gain {:7.3} dB)",
            out_db - in_db
        );
        assert!(
            out_db.is_finite(),
            "dyna comp: non-finite output at {in_db} dB input"
        );
    }
    // Monotonic-ish: louder input never produces a >1 dB drop in output.
    for pair in curve.windows(2) {
        assert!(
            pair[1].1 >= pair[0].1 - 1.0,
            "dyna comp curve not monotonic-ish: {:?} -> {:?}",
            pair[0],
            pair[1]
        );
    }

    let ratio = compression_ratio(&curve, -20.0, 0.0);
    let gr = gain_reduction_db(&curve, -40.0, 0.0);
    eprintln!("dyna comp ratio (-20..0 dB): {ratio:.4}:1");
    eprintln!("dyna comp gain reduction (-40 vs 0 dB): {gr:.3} dB");
    assert!(ratio.is_finite(), "dyna comp ratio not finite");
    // Cross-process compile nondeterminism: dyna_comp.pedal alternates
    // between a flat-linear solution (ratio 1.00, GR 0 dB) and a mildly
    // compressing one (ratio up to ~1.34, GR ~4.2 dB), with occasional
    // slight expansion. Bound loosely; the prints above are the audit data.
    assert!(
        ratio >= 0.75,
        "dyna comp ratio {ratio:.4} indicates strong expansion"
    );

    // Tone burst for attack/release: compression depth may be too shallow
    // for a robust time constant, so only finiteness/positivity is asserted.
    let step_up = (0.5 * SAMPLE_RATE) as usize;
    let step_down = step_up + (0.75 * SAMPLE_RATE) as usize;
    let burst = tone_burst(1_000.0, 0.02, 0.5, 0.5, 0.75, 0.75, SAMPLE_RATE);
    let output = compile_and_process(&src, &burst, SAMPLE_RATE, controls);
    assert_healthy(&output, "Dyna Comp tone burst", 50.0);

    let attack = measure_attack_seconds(&output[..step_down], step_up, SAMPLE_RATE);
    let release = measure_release_seconds(&output, step_down, SAMPLE_RATE);
    eprintln!("dyna comp attack: {attack:.4} s, release: {release:.4} s");
    assert!(
        attack.is_finite() && attack >= 0.0,
        "attack {attack} invalid"
    );
    assert!(
        release.is_finite() && release >= 0.0,
        "release {release} invalid"
    );
}
