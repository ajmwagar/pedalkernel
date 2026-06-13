//! Acceptance tests for the rack gear examples (`examples/rack/`) —
//! Roland SDD-320 Dimension D-style BBD chorus and Roland RE-201
//! Space Echo-style tape delay.
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

// ███████████████████████████████████████████████████████████████████████████
// Roland RE-201 Space Echo (tape section) — examples/rack/space_echo.pedal
// ███████████████████████████████████████████████████████████████████████████

/// Compile examples/rack/space_echo.pedal at the shared test sample rate.
fn compile_space_echo() -> CompiledPedal {
    let src = example_pedal_source("space_echo.pedal");
    let pedal = parse_pedal_file(&src).expect("space_echo.pedal: parse failed");
    compile_pedal(&pedal, SAMPLE_RATE).expect("space_echo.pedal: compile failed")
}

/// Stimulus for echo-timing tests: `settle_secs` of silence (preamp bias
/// settle), a `burst_secs` pseudo-noise burst at `amp`, then silence to
/// `total_secs`. Returns (samples, burst_start_sample).
fn settle_burst_silence(
    settle_secs: f64,
    burst_secs: f64,
    total_secs: f64,
    amp: f64,
) -> (Vec<f64>, usize) {
    let n_settle = (settle_secs * SAMPLE_RATE) as usize;
    let burst = pseudo_noise(burst_secs, amp, SAMPLE_RATE);
    let n_total = (total_secs * SAMPLE_RATE) as usize;
    let mut input = vec![0.0; n_settle];
    input.extend_from_slice(&burst);
    input.resize(n_total, 0.0);
    (input, n_settle)
}

/// Detect echo events in `output` from `start` on: short-window RMS
/// envelope (5 ms window, 1 ms hop); contiguous regions above
/// `rel_threshold` x global envelope peak become events at their envelope
/// maximum. Regions closer than 25 ms merge. Returns (time_secs, peak_env)
/// per event, time measured from sample 0.
fn echo_events(output: &[f64], start: usize, rel_threshold: f64) -> Vec<(f64, f64)> {
    let win = (0.005 * SAMPLE_RATE) as usize;
    let hop = (0.001 * SAMPLE_RATE) as usize;
    let mut env: Vec<(usize, f64)> = Vec::new();
    let mut pos = start;
    while pos + win <= output.len() {
        env.push((pos, rms(&output[pos..pos + win])));
        pos += hop;
    }
    let peak_env = env.iter().map(|&(_, e)| e).fold(0.0f64, f64::max);
    let thr = peak_env * rel_threshold;
    let gap_hops = 25; // 25 ms @ 1 ms hop
    let mut events: Vec<(f64, f64)> = Vec::new();
    let mut current: Option<(usize, f64)> = None; // (best_pos, best_env)
    let mut hops_below = gap_hops;
    for &(p, e) in &env {
        if e > thr {
            hops_below = 0;
            current = match current {
                Some((bp, be)) if be >= e => Some((bp, be)),
                _ => Some((p, e)),
            };
        } else if current.is_some() {
            hops_below += 1;
            if hops_below >= gap_hops {
                let (bp, be) = current.take().unwrap();
                events.push((bp as f64 / SAMPLE_RATE, be));
            }
        }
    }
    if let Some((bp, be)) = current {
        events.push((bp as f64 / SAMPLE_RATE, be));
    }
    events
}

/// Head-1 delay (seconds) the engine produces for a "Repeat Rate" pot value:
/// log interpolation between the delay line's 67 ms / 200 ms range.
fn expected_head1_delay(repeat_rate: f64) -> f64 {
    let (lo, hi) = (0.067f64, 0.200f64);
    (lo.ln() + repeat_rate * (hi.ln() - lo.ln())).exp()
}

/// Apply the panel laws straight onto the runtime delay line.
///
/// ENGINE GAP (see space_echo_panel_controls_engine_gated): the SPQR
/// pipeline never binds pot -> DL.delay_time / DL.feedback nets —
/// spqr_control::bind_controls only binds pots that live inside compiled
/// stages, and the legacy bind.rs handlers for ControlTarget::DelayTime/
/// DelayFeedback are dead code — so `set_control("Repeat Rate"/
/// "Intensity", ..)` is a no-op. These tests therefore drive the public
/// runtime API directly, with the exact mappings the bindings would use:
/// `set_delay_normalized(pot)` (processor.rs ControlTarget::DelayTime)
/// and `set_feedback(pot * 0.95)` (ControlTarget::DelayFeedback).
fn set_tape_loop(proc: &mut CompiledPedal, repeat_rate: f64, intensity: f64) {
    let dl = &mut proc.delay_lines[0].delay_line;
    dl.set_delay_normalized(repeat_rate);
    dl.set_feedback(intensity * 0.95);
}

// ===========================================================================
// (1) Compiles, delay line lowered, healthy output
// ===========================================================================

#[test]
fn space_echo_compiles_and_is_healthy() {
    let mut compiled = compile_space_echo();

    assert_eq!(
        compiled.delay_lines.len(),
        1,
        "space_echo declares DL1: delay_line(...) — must lower into CompiledPedal.delay_lines"
    );
    let dl = &compiled.delay_lines[0];
    eprintln!(
        "space_echo: delay range {:.3}-{:.3} s, lowered taps = {:?} \
         (T2/T3 head taps are engine-gated, see space_echo_head_ratios_2x_3x)",
        dl.delay_line.min_delay_sec(),
        dl.delay_line.max_delay_sec(),
        dl.taps
    );
    assert!(
        (dl.delay_line.min_delay_sec() - 0.067).abs() < 1e-6
            && (dl.delay_line.max_delay_sec() - 0.200).abs() < 1e-6,
        "head-1 base delay range must be 67-200 ms"
    );

    // ENGINE GAPS, demonstrated (not asserted, so this test starts passing
    // the moment they are fixed):
    //   - LFO_wow.out -> DL1.speed_mod binds no LFO: the SPQR pipeline only
    //     creates LfoBindings for BBD clock targets (bbd_lowering.rs
    //     bind_bbd_runtime); bind.rs build_lfo_bindings is never called.
    //   - "Repeat Rate"/"Intensity" pots on DL pins bind no control:
    //     set_control is a no-op on the delay line.
    let t_before = compiled.delay_lines[0].delay_line.delay_time();
    compiled.set_control("Repeat Rate", 0.0);
    let t_after = compiled.delay_lines[0].delay_line.delay_time();
    eprintln!(
        "space_echo: lfo bindings = {} (design: 1 wow LFO at 4.97 Hz); \
         set_control(\"Repeat Rate\", 0.0): delay {:.4} -> {:.4} s \
         (design: -> 0.067 s)",
        compiled.lfos.len(),
        t_before,
        t_after
    );

    let mut proc = compile_space_echo();
    set_tape_loop(&mut proc, 0.5, 0.35);
    let input = sine(440.0, 2.0, SAMPLE_RATE);
    let output = process_all(&mut proc, &input);
    eprintln!(
        "space_echo: peak={:.6} rms={:.6}",
        peak(&output),
        rms(&output)
    );
    assert_healthy(&output, "Space Echo steady sine", 5.0);
}

// ===========================================================================
// (2) Echo timing: head-1 arrival and feedback-recirculation spacing
// ===========================================================================

#[test]
fn space_echo_echo_timing_and_recirculation_ratios() {
    // 20 ms noise burst, then silence. With Intensity at 0.6 the repeats
    // recirculate through the loop, arriving at D, 2D, 3D, ... after the
    // dry burst. NOTE: these 1:2:3 arrivals are head-1 RECIRCULATIONS —
    // the real unit's heads 2/3 would put first-pass energy at the same
    // 2x/3x positions, but tap lowering is engine-gated (see
    // space_echo_head_ratios_2x_3x). The spacing arithmetic measured here
    // is the same.
    let mut proc = compile_space_echo();
    set_tape_loop(&mut proc, 0.5, 0.6);
    let (input, burst_start) = settle_burst_silence(0.5, 0.020, 1.5, 0.3);
    let output = process_all(&mut proc, &input);
    assert!(output.iter().all(|x| x.is_finite()), "NaN/inf in output");

    let events = echo_events(&output, burst_start, 3e-3);
    eprintln!("space_echo: events (t from input start, env peak):");
    for (t, e) in &events {
        eprintln!("  t={:.4} s  env={:.6}", t, e);
    }
    assert!(
        events.len() >= 4,
        "expected dry burst + >=3 recirculating echoes, got {} events",
        events.len()
    );
    let t0 = events[0].0; // dry burst
    let arrivals: Vec<f64> = events[1..].iter().map(|(t, _)| t - t0).collect();
    let expected = expected_head1_delay(0.5);
    eprintln!(
        "space_echo: echo arrivals after dry = {:?} s (head-1 design {:.4} s); \
         ratios vs first = {:?}",
        arrivals
            .iter()
            .map(|a| (a * 1e4).round() / 1e4)
            .collect::<Vec<_>>(),
        expected,
        arrivals
            .iter()
            .map(|a| ((a / arrivals[0]) * 100.0).round() / 100.0)
            .collect::<Vec<_>>()
    );
    assert!(
        (arrivals[0] - expected).abs() / expected < 0.10,
        "first echo at {:.4} s, design head-1 delay {:.4} s (>10% off)",
        arrivals[0],
        expected
    );
    let r2 = arrivals[1] / arrivals[0];
    let r3 = arrivals[2] / arrivals[0];
    assert!(
        (r2 - 2.0).abs() < 0.2,
        "second arrival / first = {r2:.3}, expected ~2.0 +/-10%"
    );
    assert!(
        (r3 - 3.0).abs() < 0.3,
        "third arrival / first = {r3:.3}, expected ~3.0 +/-10%"
    );
}

// ===========================================================================
// (3) Repeat Rate authority: motor speed shifts the echo arrival
// ===========================================================================

#[test]
fn space_echo_repeat_rate_authority() {
    // Intensity 0 isolates the first head pass. Repeat Rate 0.0 vs 1.0
    // spans the 67-200 ms head-1 range (log law) — arrival ratio design
    // 2.99. Applied via the runtime API (set_tape_loop): the panel pot
    // binding itself is engine-gated.
    let mut arrivals = Vec::new();
    for &rr in &[0.0, 1.0] {
        let mut proc = compile_space_echo();
        set_tape_loop(&mut proc, rr, 0.0);
        let (input, burst_start) = settle_burst_silence(0.5, 0.020, 1.2, 0.3);
        let output = process_all(&mut proc, &input);
        let events = echo_events(&output, burst_start, 3e-3);
        assert!(
            events.len() >= 2,
            "Repeat Rate {rr}: expected dry + head-1 echo, got {} events",
            events.len()
        );
        let arrival = events[1].0 - events[0].0;
        eprintln!(
            "space_echo: Repeat Rate {rr} -> head-1 echo at {:.4} s (design {:.4} s)",
            arrival,
            expected_head1_delay(rr)
        );
        arrivals.push(arrival);
    }
    let ratio = arrivals[1] / arrivals[0];
    eprintln!("space_echo: Repeat Rate authority = {ratio:.3}x (design 2.99x)");
    assert!(
        ratio > 1.5,
        "Repeat Rate must shift the echo arrival by >1.5x (measured {ratio:.3}x)"
    );
}

// ===========================================================================
// (4) Intensity: more regeneration = more audible repeats
// ===========================================================================

#[test]
fn space_echo_intensity_regeneration() {
    // Count echo events above -60 dB (rel. to the dry burst envelope) in
    // 3.5 s. fb = Intensity * 0.95 per pass: 0.25 -> ~4 repeats, 0.75 ->
    // a long tail.
    let mut counts = Vec::new();
    for &intensity in &[0.25, 0.75] {
        let mut proc = compile_space_echo();
        set_tape_loop(&mut proc, 0.3, intensity);
        let (input, burst_start) = settle_burst_silence(0.5, 0.020, 4.0, 0.3);
        let output = process_all(&mut proc, &input);
        let events = echo_events(&output, burst_start, 1e-3);
        let echoes = events.len().saturating_sub(1); // minus the dry burst
        eprintln!("space_echo: Intensity {intensity} -> {echoes} echoes above -60 dB");
        counts.push(echoes);
    }
    assert!(
        counts[1] > counts[0],
        "higher Intensity must yield more repeats ({} vs {})",
        counts[1],
        counts[0]
    );
}

// ===========================================================================
// (5) Intensity at max: repeats sustain long after the input stops
// ===========================================================================

#[test]
fn space_echo_max_intensity_sustains() {
    // Engine clamps loop gain to 0.95 (processor.rs set_control scales
    // Intensity x0.95; DelayLine::set_feedback clamps at 0.99), so true
    // runaway self-oscillation — and the tape compression that tames it
    // in the real unit (Intensity past ~2 o'clock) — is engine-gated.
    // What measures true: at Intensity 1.0 the tail decays only ~0.45 dB
    // per 116 ms pass, so it is still strong seconds after the input
    // stops, while moderate Intensity leaves silence.
    let run = |intensity: f64| -> (f64, f64) {
        let mut proc = compile_space_echo();
        set_tape_loop(&mut proc, 0.5, intensity);
        let (input, burst_start) = settle_burst_silence(0.5, 0.020, 4.5, 0.3);
        let output = process_all(&mut proc, &input);
        assert!(output.iter().all(|x| x.is_finite()), "NaN/inf in output");
        let w = |a: f64, b: f64| {
            let lo = burst_start + (a * SAMPLE_RATE) as usize;
            let hi = burst_start + (b * SAMPLE_RATE) as usize;
            rms(&output[lo..hi])
        };
        (w(0.3, 0.8), w(3.0, 3.5)) // (early tail, late tail)
    };

    let (early_hi, late_hi) = run(1.0);
    let (early_lo, late_lo) = run(0.3);
    eprintln!(
        "space_echo: Intensity 1.0 tail rms early={early_hi:.6} late={late_hi:.6} \
         (late/early = {:.3}); Intensity 0.3 late={late_lo:.8}",
        late_hi / early_hi.max(1e-18)
    );
    assert!(
        late_hi > 0.05 * early_hi,
        "Intensity max: tail must sustain 3 s after input stops \
         (late {late_hi:.6} vs early {early_hi:.6})"
    );
    assert!(
        late_hi > 10.0 * late_lo.max(1e-12),
        "max-Intensity tail must dwarf the moderate-Intensity tail \
         ({late_hi:.6} vs {late_lo:.8})"
    );
    assert!(
        late_hi < 10.0 * early_lo.max(early_hi),
        "loop gain is clamped < 1 — tail must not blow up (late {late_hi:.6})"
    );
}

// ===========================================================================
// (6) Wow: speed_mod LFO puts measurable modulation on a steady tone
// ===========================================================================

#[test]
fn space_echo_wow_modulates_steady_tone() {
    // 110 Hz steady sine: the +/-2% wow on a ~116 ms delay sweeps the
    // wet path by ~+/-2.3 ms — about half a comb period at 110 Hz — so
    // the wet+dry sum is amplitude-modulated at the wow rate.
    //
    // The wow is injected per sample through the public runtime API
    // (DelayLine::add_speed_mod) with EXACTLY the values the declared
    // binding would deliver — `LFO_wow: lfo(sine, 32k, 1u)` at 4.974 Hz
    // into the speed_mod sink's range of 0.02 — because the compiler
    // never creates the LfoBinding (see
    // space_echo_wow_lfo_binding_engine_gated). A no-wow control run
    // isolates the effect.
    let wow_hz = 4.974;
    let wow_range = 0.02; // DelayLineComp speed_mod ModulationSink range
    let input = sine(110.0, 7.0, SAMPLE_RATE);

    let run = |wow_depth: f64| -> Vec<f64> {
        let mut proc = compile_space_echo();
        set_tape_loop(&mut proc, 0.5, 0.35);
        // F15 NOTE: the wow LFO now binds for real (DelaySpeed) and would
        // double-apply on top of this manual injection (and run in the no-wow
        // control too). Disable the bound LFO's depth so this manual injection
        // is the sole wow source, preserving the original single-source premise
        // of this measurement. The bound path is verified separately in
        // space_echo_wow_lfo_binding_engine_gated.
        if !proc.lfos.is_empty() {
            proc.lfos[0].range = 0.0;
        }
        input
            .iter()
            .enumerate()
            .map(|(i, &s)| {
                let t = i as f64 / SAMPLE_RATE;
                let wow = wow_depth * (2.0 * std::f64::consts::PI * wow_hz * t).sin();
                proc.delay_lines[0].delay_line.add_speed_mod(wow);
                proc.process(s)
            })
            .collect()
    };

    let depth_of = |output: &[f64]| -> f64 {
        let settled = &output[(1.0 * SAMPLE_RATE) as usize..];
        let mut env = rms_envelope(settled, 0.010, SAMPLE_RATE);
        env.sort_by(|a, b| a.partial_cmp(b).unwrap());
        // 5th/95th percentile depth — robust against onset/settle tails.
        let lo = env[env.len() / 20];
        let hi = env[env.len() - 1 - env.len() / 20];
        (hi - lo) / hi.max(1e-18)
    };

    let wet = run(wow_range);
    assert!(wet.iter().all(|x| x.is_finite()), "NaN/inf in output");
    let dry = run(0.0);
    let (depth_wow, depth_still) = (depth_of(&wet), depth_of(&dry));
    let rate = detect_modulation_rate(&wet[(1.0 * SAMPLE_RATE) as usize..], SAMPLE_RATE);
    eprintln!(
        "space_echo: wow envelope depth = {:.1}% (no-wow control {:.1}%), \
         detected modulation rate = {rate:.2} Hz (design wow {wow_hz} Hz; \
         comb retrace can land at low harmonics)",
        depth_wow * 100.0,
        depth_still * 100.0
    );
    assert!(
        depth_wow > 0.05,
        "wow must modulate the steady tone by >5% (measured {:.1}%)",
        depth_wow * 100.0
    );
    assert!(
        depth_wow > 2.0 * depth_still,
        "modulation must come from the wow injection, not residue \
         ({:.1}% vs {:.1}% without wow)",
        depth_wow * 100.0,
        depth_still * 100.0
    );
    assert!(
        (2.0..=16.0).contains(&rate),
        "detected modulation rate {rate:.2} Hz outside the wow band \
         (design 4.97 Hz fundamental)"
    );
}

// ===========================================================================
// Engine-gated: playback heads 2/3 (taps parse, but are not lowered)
// ===========================================================================

/// The RE-201's three playback heads sit evenly along the tape path, so
/// one record pass arrives at 1x / 2x / 3x the head-1 delay. The DSL
/// expresses this as `T2: tap(DL1, 2.0)` / `T3: tap(DL1, 3.0)`, and the
/// runtime DelayLineBinding.taps supports multi-tap reads — but the
/// compiler hard-codes `taps: vec![1.0]` (spqr_build.rs,
/// build_delay_line_bindings) and never lowers `tap()` components, so
/// only head 1 sounds. The tape_oxide medium is similarly inert: the
/// compiler never calls DelayLine::configure_zones_from_taps, leaving the
/// zone list empty.
///
/// When tap lowering lands, this test should drive a 20 ms burst with
/// Intensity = 0 (no recirculation) and assert THREE first-pass arrivals
/// with tap-2/tap-1 ratio 2.0 +/-10% and tap-3/tap-1 ratio 3.0 +/-10%.
/// Measured today (Intensity = 0, Repeat Rate = 0.5): exactly ONE echo at
/// ~0.116 s and silence at the 2x/3x positions — see
/// space_echo_repeat_rate_authority's event dump.
/// PROMOTED by F15 (delay_lowering DspBlock): `T2: tap(DL1, 2.0)` /
/// `T3: tap(DL1, 3.0)` now lower into `DelayLineBinding.taps = [1.0, 2.0, 3.0]`
/// (was hard-coded `[1.0]`), so the runtime reads + sums all three playback
/// heads. With Intensity = 0 (no recirculation) a single burst arrives THREE
/// times — at the head-1 delay D, then 2D and 3D — exactly the RE-201's evenly
/// spaced heads. Baseline: Repeat Rate 0.5 → head-1 ~0.116 s; arrivals at
/// 1x/2x/3x +/-10%.
#[test]
fn space_echo_head_ratios_2x_3x() {
    let mut proc = compile_space_echo();
    assert_eq!(
        proc.delay_lines[0].taps,
        vec![1.0, 2.0, 3.0],
        "tap(DL1,2.0)/tap(DL1,3.0) must lower into the head ratios"
    );
    // Intensity 0 isolates the first record pass: no recirculation, so the
    // only echoes are the three playback heads. Repeat Rate 0.5.
    proc.set_control("Repeat Rate", 0.5);
    proc.set_control("Intensity", 0.0);

    let (input, burst_start) = settle_burst_silence(0.5, 0.020, 1.5, 0.3);
    let output = process_all(&mut proc, &input);
    assert!(output.iter().all(|x| x.is_finite()), "NaN/inf in output");

    let events = echo_events(&output, burst_start, 3e-3);
    eprintln!("space_echo head ratios: events (t, env):");
    for (t, e) in &events {
        eprintln!("  t={:.4} s  env={:.6}", t, e);
    }
    assert!(
        events.len() >= 4,
        "expected dry burst + 3 head echoes (1x/2x/3x), got {} events",
        events.len()
    );
    let t0 = events[0].0;
    let arrivals: Vec<f64> = events[1..4].iter().map(|(t, _)| t - t0).collect();
    let d1 = expected_head1_delay(0.5);
    eprintln!(
        "space_echo head ratios: arrivals after dry = {:?} s (head-1 design {:.4} s); \
         ratios = {:?}",
        arrivals,
        d1,
        arrivals.iter().map(|a| a / arrivals[0]).collect::<Vec<_>>()
    );
    assert!(
        (arrivals[0] - d1).abs() / d1 < 0.10,
        "head-1 arrival {:.4} s, design {:.4} s (>10% off)",
        arrivals[0],
        d1
    );
    let r2 = arrivals[1] / arrivals[0];
    let r3 = arrivals[2] / arrivals[0];
    assert!(
        (r2 - 2.0).abs() < 0.2,
        "head-2/head-1 ratio = {r2:.3}, expected ~2.0 +/-10%"
    );
    assert!(
        (r3 - 3.0).abs() < 0.3,
        "head-3/head-1 ratio = {r3:.3}, expected ~3.0 +/-10%"
    );
}

// ===========================================================================
// Engine-gated: panel pots and the wow LFO never bind to the delay line
// ===========================================================================

/// `RepeatRate.wiper -> DL1.delay_time` and `Intensity.wiper ->
/// DL1.feedback` follow the documented DSL pattern (docs/dsl.md, and
/// tests/control_binding.rs pot_to_delay_time/pot_to_delay_feedback), but
/// the SPQR pipeline — now the only pipeline (compile.rs: "the legacy
/// 6-pass pipeline has been removed") — only binds pots that live inside
/// compiled stages (spqr_control.rs find_pot_bindings) plus BBD targets
/// (bbd_lowering.rs). bind.rs's ControlTarget::DelayTime/DelayFeedback
/// resolution is dead code, so set_control("Repeat Rate"/"Intensity") is
/// a no-op: the delay stays at its construction midpoint (measured
/// 0.1335 s = (67+200)/2 ms) and feedback stays 0. The four delay tests
/// in tests/control_binding.rs fail at HEAD for the same reason.
///
/// When the bindings land, this test should drive bursts at
/// set_control("Repeat Rate", 0.0 / 1.0) and assert echo arrivals at
/// 67 ms / 200 ms, then set_control("Intensity", 0.75) and assert a
/// repeat tail — the measured DSP-level numbers in
/// space_echo_repeat_rate_authority / space_echo_intensity_regeneration
/// are the acceptance baseline.
/// PROMOTED by F15: `RepeatRate.wiper -> DL1.delay_time` now binds to
/// `ControlTarget::DelayTime(0)` and `Intensity.wiper -> DL1.feedback` to
/// `ControlTarget::DelayFeedback(0)` (delay_lowering DspBlock). So
/// `set_control("Repeat Rate", 0.0/1.0)` drives the head-1 delay across the
/// 67-200 ms motor-speed range (log law, ~2.99x authority) and
/// `set_control("Intensity", ..)` opens the regeneration loop. The four
/// `tests/control_binding.rs` delay tests are the structural counterpart of
/// this behavioral one.
#[test]
fn space_echo_panel_controls_engine_gated() {
    // Repeat Rate authority: 0.0 -> 67 ms, 1.0 -> 200 ms head-1 (Intensity 0
    // isolates the first head pass; the burst's first head echo IS head 1).
    let mut arrivals = Vec::new();
    for &rr in &[0.0, 1.0] {
        let mut proc = compile_space_echo();
        proc.set_control("Intensity", 0.0);
        proc.set_control("Repeat Rate", rr);
        let (input, burst_start) = settle_burst_silence(0.5, 0.020, 1.2, 0.3);
        let output = process_all(&mut proc, &input);
        let events = echo_events(&output, burst_start, 3e-3);
        assert!(
            events.len() >= 2,
            "Repeat Rate {rr}: expected dry + head-1 echo, got {} events",
            events.len()
        );
        let arrival = events[1].0 - events[0].0;
        eprintln!(
            "space_echo panel: set_control(Repeat Rate, {rr}) -> head-1 echo at {:.4} s \
             (design {:.4} s)",
            arrival,
            expected_head1_delay(rr)
        );
        arrivals.push(arrival);
    }
    let ratio = arrivals[1] / arrivals[0];
    eprintln!("space_echo panel: Repeat Rate authority = {ratio:.3}x (design 2.99x)");
    assert!(
        ratio > 1.5,
        "set_control(Repeat Rate) must shift the echo arrival by >1.5x (measured {ratio:.3}x)"
    );

    // Intensity authority: higher Intensity = more audible repeats.
    let mut counts = Vec::new();
    for &intensity in &[0.0, 0.75] {
        let mut proc = compile_space_echo();
        proc.set_control("Repeat Rate", 0.3);
        proc.set_control("Intensity", intensity);
        let (input, burst_start) = settle_burst_silence(0.5, 0.020, 4.0, 0.3);
        let output = process_all(&mut proc, &input);
        let events = echo_events(&output, burst_start, 1e-3);
        let echoes = events.len().saturating_sub(1);
        eprintln!("space_echo panel: set_control(Intensity, {intensity}) -> {echoes} echoes");
        counts.push(echoes);
    }
    assert!(
        counts[1] > counts[0],
        "set_control(Intensity) must add repeats ({} vs {})",
        counts[1],
        counts[0]
    );
}

/// `LFO_wow.out -> DL1.speed_mod` is the documented wow/flutter pattern
/// (docs/dsl.md "Delay + Tap example"), and validate.rs accepts speed_mod
/// as a modulation target — but the SPQR pipeline only creates
/// LfoBindings for BBD clock targets (bbd_lowering.rs bind_bbd_runtime);
/// bind.rs build_lfo_bindings (which resolves DelaySpeed/DelayTime sinks)
/// is never called. Compiled result: CompiledPedal.lfos is empty and the
/// tape runs wow-free.
///
/// When the binding lands, space_echo_wow_modulates_steady_tone's
/// numbers are the acceptance baseline (it injects the identical
/// modulation — 4.974 Hz sine x 0.02 speed_mod range — through the
/// public DelayLine::add_speed_mod API): envelope depth > 5% on a 110 Hz
/// tone and a detected wow-band modulation rate.
/// PROMOTED by F15: `LFO_wow.out -> DL1.speed_mod` now binds to
/// `ModulationTarget::DelaySpeed { delay_idx: 0 }` (delay_lowering DspBlock),
/// at the declared 4.974 Hz (`lfo(sine, 32k, 1u)`) into the speed_mod sink's
/// 0.02 range — capstan wow. The runtime ticks the LFO each sample and calls
/// `add_speed_mod(lfo_out * 0.02)`, so the compiled tape wobbles WITHOUT any
/// manual injection. Control run: zero the binding's range. Baseline: >5%
/// envelope depth on a 110 Hz tone, detected wow-band rate.
#[test]
fn space_echo_wow_lfo_binding_engine_gated() {
    // Structural: the wow LFO binds to the delay's speed_mod (was: lfos empty).
    let compiled = compile_space_echo();
    assert_eq!(
        compiled.lfos.len(),
        1,
        "LFO_wow.out -> DL1.speed_mod must bind one LFO (was engine-gated: 0)"
    );
    assert!(
        matches!(
            compiled.lfos[0].target,
            pedalkernel_rt::processor::ModulationTarget::DelaySpeed { delay_idx: 0 }
        ),
        "wow LFO must target DelaySpeed {{ delay_idx: 0 }}, got {:?}",
        compiled.lfos[0].target
    );
    assert!(
        (compiled.lfos[0].base_freq - 4.974).abs() < 0.05,
        "wow LFO base freq {:.3} Hz, design 4.974 Hz",
        compiled.lfos[0].base_freq
    );

    // Behavioral: the bound wow modulates a steady 110 Hz tone. Control run
    // zeroes the binding's modulation range (kills the wow without touching
    // the rest of the path).
    let input = sine(110.0, 7.0, SAMPLE_RATE);
    let run = |wow_on: bool| -> Vec<f64> {
        let mut proc = compile_space_echo();
        proc.set_control("Repeat Rate", 0.5);
        proc.set_control("Intensity", 0.35);
        if !wow_on {
            proc.lfos[0].range = 0.0; // disable the wow depth
        }
        process_all(&mut proc, &input)
    };

    let depth_of = |output: &[f64]| -> f64 {
        let settled = &output[(1.0 * SAMPLE_RATE) as usize..];
        let mut env = rms_envelope(settled, 0.010, SAMPLE_RATE);
        env.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let lo = env[env.len() / 20];
        let hi = env[env.len() - 1 - env.len() / 20];
        (hi - lo) / hi.max(1e-18)
    };

    let wet = run(true);
    assert!(wet.iter().all(|x| x.is_finite()), "NaN/inf in output");
    let dry = run(false);
    let (depth_wow, depth_still) = (depth_of(&wet), depth_of(&dry));
    let rate = detect_modulation_rate(&wet[(1.0 * SAMPLE_RATE) as usize..], SAMPLE_RATE);
    eprintln!(
        "space_echo wow binding: depth = {:.1}% (wow off {:.1}%), detected rate = {rate:.2} Hz \
         (design 4.97 Hz)",
        depth_wow * 100.0,
        depth_still * 100.0
    );
    assert!(
        depth_wow > 0.05,
        "bound wow must modulate the steady tone by >5% (measured {:.1}%)",
        depth_wow * 100.0
    );
    assert!(
        depth_wow > 2.0 * depth_still,
        "modulation must come from the bound wow, not the static multi-head comb \
         ({:.1}% vs {:.1}% with wow disabled)",
        depth_wow * 100.0,
        depth_still * 100.0
    );
    assert!(
        (2.0..=16.0).contains(&rate),
        "detected modulation rate {rate:.2} Hz outside the wow band (design 4.97 Hz)"
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
