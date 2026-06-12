//! Acceptance tests for the rack gear examples (`examples/rack/`) —
//! Roland SDD-320 Dimension D-style BBD chorus.
//!
//! Measurement conventions follow `tests/bbd_lowering.rs` /
//! `tests/eurorack_examples.rs`: compile the example, drive it with a
//! deterministic stimulus, print the measured numbers, and assert only
//! what measures true. The wet (BBD) contribution is isolated by running
//! the same circuit twice — once as compiled, once with `bbd_wet_mix = 0`
//! — and subtracting.

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::compiler::{compile_pedal, CompiledPedal};
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::elements::Modulator;
use pedalkernel::PedalProcessor;

/// Compile examples/rack/dimension_d.pedal at the shared test sample rate.
fn compile_dimension_d() -> CompiledPedal {
    let src = example_pedal_source("dimension_d.pedal");
    let pedal = parse_pedal_file(&src).expect("dimension_d.pedal: parse failed");
    compile_pedal(&pedal, SAMPLE_RATE).expect("dimension_d.pedal: compile failed")
}

fn process_all(proc: &mut CompiledPedal, input: &[f64]) -> Vec<f64> {
    input.iter().map(|&s| proc.process(s)).collect()
}

/// Deterministic full-band pseudo-noise (xorshift32), amplitude `amp`.
/// Aperiodic stimulus so cross-correlation has a single unambiguous peak
/// (a sine burst would alias at multiples of its period).
fn pseudo_noise(duration_secs: f64, amp: f64, sample_rate: f64) -> Vec<f64> {
    let n = (duration_secs * sample_rate) as usize;
    let mut state: u32 = 0x2545_F491;
    (0..n)
        .map(|_| {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            amp * (state as f64 / u32::MAX as f64 * 2.0 - 1.0)
        })
        .collect()
}

/// Cross-correlate `wet[start..start+len]` against `reference` over lags in
/// [lo_ms, hi_ms], returning (best_lag_ms, normalized_peak).
fn correlation_delay_ms(
    reference: &[f64],
    wet: &[f64],
    start: usize,
    len: usize,
    lo_ms: f64,
    hi_ms: f64,
    sample_rate: f64,
) -> (f64, f64) {
    let lo = (lo_ms * 1e-3 * sample_rate) as usize;
    let hi = (hi_ms * 1e-3 * sample_rate) as usize;
    let r = &reference[start..start + len];
    let r_energy = energy(r).max(1e-18);
    let mut best = (lo_ms, f64::MIN);
    for lag in lo..=hi {
        let w = &wet[start + lag..start + lag + len];
        let dot: f64 = r.iter().zip(w).map(|(a, b)| a * b).sum();
        let score = dot / (r_energy * energy(w)).sqrt().max(1e-18);
        if score > best.1 {
            best = (lag as f64 / sample_rate * 1000.0, score);
        }
    }
    best
}

// ===========================================================================
// (1) Compiles, BBD lowered, healthy output
// ===========================================================================

#[test]
fn dimension_d_compiles_and_is_healthy() {
    let mut compiled = compile_dimension_d();

    assert_eq!(
        compiled.bbds.len(),
        1,
        "dimension_d declares BBD1: bbd(mn3007) — must lower into CompiledPedal.bbds"
    );
    assert_eq!(
        compiled.bbds[0].model.num_stages, 1024,
        "MN3007 is a 1024-stage device"
    );
    assert_eq!(
        compiled.lfos.len(),
        1,
        "LFO1.out -> BBD1.clock must bind exactly one clock LFO"
    );
    let lfo = &compiled.lfos[0];
    eprintln!(
        "dimension_d: lfo base_freq={:.4} Hz bias={:.3} range={:.3}",
        lfo.base_freq, lfo.bias, lfo.range
    );
    // lfo(triangle, 636k, 1u): f = 1/(2*pi*0.636) = 0.2503 Hz (Mode 1/2 rate)
    assert!(
        (lfo.base_freq - 0.25).abs() < 0.01,
        "LFO base rate should be ~0.25 Hz (measured {:.4})",
        lfo.base_freq
    );

    let input = sine(440.0, 1.0, SAMPLE_RATE);
    let output = process_all(&mut compiled, &input);
    eprintln!(
        "dimension_d: peak={:.6} rms={:.6} wet_mix={}",
        peak(&output),
        rms(&output),
        compiled.bbd_wet_mix
    );
    assert_healthy(&output, "Dimension D steady sine", 5.0);
}

// ===========================================================================
// (2) Delay time: wet contribution sits in the SDD-320's 5-12 ms window
// ===========================================================================

#[test]
fn dimension_d_delay_sweep_in_window() {
    // Continuous deterministic pseudo-noise for 13 s (three 0.25 Hz LFO
    // periods + settle). The BBD wet contribution is isolated by
    // subtracting a dry-only run (bbd_wet_mix = 0); a sliding 0.2 s
    // cross-correlation window then tracks the instantaneous wet delay.
    // The delay-vs-time series IS the LFO sweep: its range must sit in
    // the SDD-320's 5-12 ms mode window and its dominant rate at the
    // 0.25 Hz Mode-1/2 LFO frequency.
    let input = pseudo_noise(13.0, 0.5, SAMPLE_RATE);

    let mut wet_proc = compile_dimension_d();
    let wet_out = process_all(&mut wet_proc, &input);
    let mut dry_proc = compile_dimension_d();
    dry_proc.bbd_wet_mix = 0.0;
    let dry_out = process_all(&mut dry_proc, &input);
    assert!(wet_out.iter().all(|x| x.is_finite()), "NaN/inf in output");

    let wet_only: Vec<f64> = wet_out.iter().zip(&dry_out).map(|(w, d)| w - d).collect();
    let wet_rms = rms(&wet_only);
    assert!(
        wet_rms > 1e-4,
        "BBD wet path must contribute to the output (wet-only rms={wet_rms:.8})"
    );

    // Track delay every 0.25 s from t=1 s to t=12.75 s (lags 1-25 ms).
    let win = (0.2 * SAMPLE_RATE) as usize;
    let hop = (0.25 * SAMPLE_RATE) as usize;
    let mut delays_ms = Vec::new();
    let mut start = SAMPLE_RATE as usize;
    while start + win + (0.025 * SAMPLE_RATE) as usize + 1 < wet_only.len() {
        let (d, score) =
            correlation_delay_ms(&dry_out, &wet_only, start, win, 1.0, 25.0, SAMPLE_RATE);
        assert!(
            score > 0.05,
            "weak correlation peak ({score:.3}) at t={:.2} s — wet path lost",
            start as f64 / SAMPLE_RATE
        );
        delays_ms.push(d);
        start += hop;
    }
    let d_min = delays_ms.iter().cloned().fold(f64::MAX, f64::min);
    let d_max = delays_ms.iter().cloned().fold(f64::MIN, f64::max);
    let d_mean = delays_ms.iter().sum::<f64>() / delays_ms.len() as f64;

    // Sweep rate: Goertzel on the (mean-removed) delay series, which is
    // sampled at 4 Hz. Scan 0.05-1.0 Hz for the dominant component.
    let series: Vec<f64> = delays_ms.iter().map(|d| d - d_mean).collect();
    let mut sweep_rate = (0.0, 0.0);
    let mut f = 0.05;
    while f <= 1.0 + 1e-9 {
        let p = goertzel_power(&series, 4.0, f);
        if p > sweep_rate.1 {
            sweep_rate = (f, p);
        }
        f += 0.0125;
    }

    eprintln!(
        "dimension_d: delay sweep {d_min:.2} -> {d_max:.2} ms (center {d_mean:.2} ms) \
         over {} windows; sweep rate = {:.3} Hz (design 0.25 Hz triangle, \
         engine clock sink => ~5.7 -> 9.1 ms)",
        delays_ms.len(),
        sweep_rate.0
    );

    assert!(
        (5.0..=12.0).contains(&d_min) && (5.0..=12.0).contains(&d_max),
        "delay sweep {d_min:.2}..{d_max:.2} ms outside the SDD-320 5-12 ms mode window"
    );
    assert!(
        d_max - d_min > 1.0,
        "LFO must sweep the delay by >1 ms (measured {:.2} ms span)",
        d_max - d_min
    );
    assert!(
        (sweep_rate.0 - 0.25).abs() <= 0.05,
        "delay sweep rate {:.3} Hz != 0.25 Hz Mode-1/2 LFO",
        sweep_rate.0
    );
}

// ===========================================================================
// (3) Modulation: triangle LFO sweep is present and at the design rate
// ===========================================================================

#[test]
fn dimension_d_modulation_rate_mode2() {
    // Steady 440 Hz sine for 12 s (three 0.25 Hz LFO periods). The swept
    // BBD delay comb-filters the wet+dry sum, producing amplitude
    // modulation at the LFO rate (and its low harmonics, since the
    // triangle sweep retraces the same delays each half-cycle).
    let mut compiled = compile_dimension_d();
    let input = sine(440.0, 10.0, SAMPLE_RATE);
    let output = process_all(&mut compiled, &input);
    assert!(output.iter().all(|x| x.is_finite()), "NaN/inf in output");

    // Skip the first 2 s (delay-line fill + compander settle), analyze
    // exactly two 0.25 Hz LFO periods (8 s) so the Goertzel bins at the
    // LFO rate and its harmonics are leakage-free.
    let settled = &output[2 * SAMPLE_RATE as usize..];
    let modulated = has_amplitude_modulation(settled, 16);
    let rate = detect_modulation_rate(settled, SAMPLE_RATE);
    eprintln!(
        "dimension_d Mode 2: detected modulation rate = {rate:.2} Hz \
         (design LFO 0.25 Hz; comb retrace puts energy at low harmonics), \
         has_amplitude_modulation = {modulated}"
    );

    assert!(
        modulated,
        "steady sine through the swept BBD must show amplitude modulation"
    );
    // Accept the LFO fundamental or its low harmonics (the triangle sweep
    // retraces the same comb response within each period, concentrating
    // envelope energy at small multiples of 0.25 Hz).
    let plausible = [0.25, 0.5, 0.75, 1.0]
        .iter()
        .any(|&f| (rate - f).abs() <= 0.06);
    assert!(
        plausible,
        "detected modulation rate {rate:.2} Hz is not the 0.25 Hz design \
         rate or a low harmonic of it"
    );
}

#[test]
fn dimension_d_mode_control_doubles_lfo_rate() {
    // "Mode" is the LFO rate control: 0.50 -> 1x (0.25 Hz, Modes 1/2),
    // 0.65 -> 2x (0.5 Hz, Modes 3/4). Verify the binding actually doubles
    // the oscillator rate.
    let mut m2 = compile_dimension_d();
    m2.set_control("Mode", 0.5);
    let rate_m2 = m2.lfos[0].lfo.rate();
    let mut m3 = compile_dimension_d();
    m3.set_control("Mode", 0.65);
    let rate_m3 = m3.lfos[0].lfo.rate();
    eprintln!(
        "dimension_d: Mode=0.50 -> {rate_m2:.4} Hz, Mode=0.65 -> {rate_m3:.4} Hz \
         (design: 0.25 / 0.5 Hz)"
    );
    assert!(
        (rate_m2 - 0.25).abs() < 0.01,
        "Mode 0.50 should give the 0.25 Hz Mode-1/2 rate (got {rate_m2:.4})"
    );
    assert!(
        (rate_m3 / rate_m2 - 2.0).abs() < 0.05,
        "Mode 0.65 should double the LFO rate (got {rate_m3:.4} vs {rate_m2:.4})"
    );
}

// ===========================================================================
// (4) Chorus character: wet+dry spectrum differs from dry-only
// ===========================================================================

#[test]
fn dimension_d_chorus_changes_spectrum() {
    let input = sine(440.0, 2.0, SAMPLE_RATE);

    let mut wet_proc = compile_dimension_d();
    let wet_out = process_all(&mut wet_proc, &input);
    let mut dry_proc = compile_dimension_d();
    dry_proc.bbd_wet_mix = 0.0;
    let dry_out = process_all(&mut dry_proc, &input);

    let settle = (0.2 * SAMPLE_RATE) as usize;
    let dist = spectral_distance(&wet_out[settle..], &dry_out[settle..], SAMPLE_RATE, 50.0);
    let diff_rms = {
        let d: f64 = wet_out[settle..]
            .iter()
            .zip(&dry_out[settle..])
            .map(|(w, d)| (w - d) * (w - d))
            .sum::<f64>()
            / (wet_out.len() - settle) as f64;
        d.sqrt()
    };
    eprintln!(
        "dimension_d: wet+dry vs dry-only spectral distance = {dist:.3} dB, \
         time-domain diff rms = {diff_rms:.6}"
    );
    assert!(
        diff_rms > 1e-5,
        "mixing in the BBD wet path must change the output (diff_rms={diff_rms:.8})"
    );
    assert!(
        dist > 0.1,
        "wet+dry spectrum should measurably differ from dry-only (distance={dist:.4} dB)"
    );
}

// ===========================================================================
// Engine-gated: stereo Dimension matrix (documented, not yet expressible)
// ===========================================================================

/// The real SDD-320 is two of these channels sharing ONE triangle LFO:
/// channel A's MN3101 clock takes it straight, channel B's takes it
/// INVERTED, and each channel's wet feeds the OPPOSITE output inverted
/// through a highpass filter (plus a slight dry bass boost). A .pedal file
/// compiles to a single in -> out processor: there is no second output bus
/// to cross-feed into and no LFO polarity-inversion pin, so the build in
/// examples/rack/dimension_d.pedal is the same-side (channel A) path only.
///
/// When stereo bus support lands, this test should compile a two-channel
/// build, drive both with the same sine, and assert the two wet delay
/// sweeps move in anti-phase (delay-vs-time slopes of opposite sign) with
/// the measured mono numbers as the per-channel baseline: delay sweep
/// 5.85 -> 9.06 ms (center 7.34 ms) at a measured 0.21 Hz (design 0.25 Hz).
#[test]
#[ignore = "engine-gated: stereo bus + inverted-LFO routing not yet supported; \
            mono channel measures 5.85 -> 9.06 ms sweep (center 7.34 ms) at \
            0.21 Hz (see dimension_d_delay_sweep_in_window)"]
fn dimension_d_stereo_antiphase_sweep() {
    unimplemented!("requires stereo output bus and an LFO inversion pin");
}
