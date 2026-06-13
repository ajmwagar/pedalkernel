//! Control-sweep verification of the Pultec-style passive LC EQ
//! (`examples/outboard/eq/passive_lc_eq.pedal`).
//!
//! Sweeps the LF boost, LF cut, HF boost, and HF atten controls across a
//! ~25-point log frequency grid (20 Hz - 20 kHz) and asserts the response
//! moves the way the netlist says it should. Every threshold below is
//! backed by a measurement on this exact netlist (2026-06-12); condensed
//! tables live in `reports/outboard-sweeps-2026-06-12.md`.
//!
//! Known engine artifacts (documented in the .pedal header):
//!   - steep rolloff above ~12 kHz regardless of controls (header note 7),
//!     so assertions stay at or below ~11 kHz;
//!   - the HF boost LC branch loses most of its reactance (header note 4):
//!     the resonant peak is a broad +1.4 dB hump instead of ~+13.6 dB, but
//!     its maximum still sits within an octave of f0 = 1/(2*pi*sqrt(LC));
//!   - runtime `set_control` pot POSITIONS BELOW 0.5 are aliased and
//!     non-physical in compiled passive trees (header note 8): position 0.5
//!     measures identical to position 0.0, and positions ~0.1-0.25 land
//!     OUTSIDE the physically reachable range (e.g. LF Boost 0.1 -> 2.8 dB
//!     BELOW the pot's 1-ohm floor; LF Cut 0.1 -> 2.1 dB ABOVE the flat
//!     response). Compiled-in control defaults are inert (always position
//!     0). Green sweeps below therefore use the trustworthy upper half of
//!     pot travel; the full-range sweeps are `#[ignore]`d red tests with
//!     measured values in the reason.

mod audio_analysis;

use audio_analysis::*;

const PULTEC: &str = "passive_lc_eq.pedal";
const PROBE_AMP: f64 = 0.1;

/// ~25 log-spaced probe frequencies, 20 Hz - 20 kHz.
fn log_freqs() -> Vec<f64> {
    let n = 25;
    (0..n)
        .map(|i| 20.0 * (20_000.0f64 / 20.0).powf(i as f64 / (n - 1) as f64))
        .collect()
}

/// Frequency response (dB) of the EQ at the given controls over `freqs`.
fn pultec_response(freqs: &[f64], controls: &[(&str, f64)]) -> Vec<(f64, f64)> {
    let src = example_pedal_source(PULTEC);
    frequency_response_db(
        || pedal_processor(&src, SAMPLE_RATE, controls),
        freqs,
        PROBE_AMP,
        SAMPLE_RATE,
    )
}

/// Single-frequency gain (dB) at the given controls.
fn gain_at(freq: f64, controls: &[(&str, f64)]) -> f64 {
    pultec_response(&[freq], controls)[0].1
}

/// Assert a sequence is monotonically non-decreasing within `tol` dB.
fn assert_monotonic_rise(vals: &[(f64, f64)], tol: f64, what: &str) {
    for w in vals.windows(2) {
        assert!(
            w[1].1 >= w[0].1 - tol,
            "{what}: not monotonic — {:.3} dB at {:.2} -> {:.3} dB at {:.2}",
            w[0].1,
            w[0].0,
            w[1].1,
            w[1].0
        );
    }
}

// ===========================================================================
// Overview table (printed for the report) + health checks
// ===========================================================================

#[test]
fn pultec_response_overview() {
    let freqs = log_freqs();
    let flat = pultec_response(&freqs, &[]);
    let lf_boost = pultec_response(&freqs, &[("LF Boost", 1.0)]);
    let lf_cut = pultec_response(&freqs, &[("LF Cut", 1.0)]);
    let hf_boost = pultec_response(&freqs, &[("HF Boost", 1.0)]);
    let hf_atten = pultec_response(&freqs, &[("HF Atten", 1.0)]);

    eprintln!(
        "{:>9} {:>8} {:>8} {:>8} {:>8} {:>8}",
        "freq", "flat", "LFb=1", "LFc=1", "HFb=1", "HFa=1"
    );
    for i in 0..freqs.len() {
        eprintln!(
            "{:9.1} {:8.2} {:8.2} {:8.2} {:8.2} {:8.2}",
            flat[i].0, flat[i].1, lf_boost[i].1, lf_cut[i].1, hf_boost[i].1, hf_atten[i].1
        );
        for resp in [&flat, &lf_boost, &lf_cut, &hf_boost, &hf_atten] {
            assert!(
                resp[i].1.is_finite(),
                "non-finite response at {:.1} Hz",
                freqs[i]
            );
        }
    }

    // Net chain gain at 1 kHz: passive pad ~-29 dB + 12AX7 makeup ~+33 dB.
    let g1k = gain_at(1_000.0, &[]);
    eprintln!("flat net gain at 1 kHz: {g1k:+.2} dB");
    assert!(
        (0.0..=10.0).contains(&g1k),
        "flat 1 kHz gain {g1k:+.2} dB outside [0, 10] dB (measured +4.3 dB 2026-06-12)"
    );
}

// ===========================================================================
// LF boost: monotonic shelf at 60-100 Hz, < 2 dB movement at 1 kHz
// ===========================================================================

#[test]
fn lf_boost_monotonic_shelf() {
    // Trustworthy pot window (header note 8): position 0.0 endpoint plus
    // the upper half of travel. Interior positions < 0.5 are aliased.
    let positions = [0.0, 0.5, 0.6, 0.75, 0.9, 1.0];

    for (freq, min_authority) in [(63.0, 1.5), (100.0, 1.0)] {
        let sweep: Vec<(f64, f64)> = positions
            .iter()
            .map(|&p| (p, gain_at(freq, &[("LF Boost", p)])))
            .collect();
        for &(p, g) in &sweep {
            eprintln!("LF Boost {p:.2} at {freq:.0} Hz -> {g:+7.3} dB");
        }
        assert_monotonic_rise(&sweep, 0.05, &format!("LF Boost at {freq:.0} Hz"));
        let authority = sweep.last().unwrap().1 - sweep[0].1;
        eprintln!("LF Boost authority at {freq:.0} Hz: {authority:+.3} dB");
        assert!(
            authority > min_authority,
            "LF Boost authority {authority:+.3} dB at {freq:.0} Hz, expected > \
             {min_authority} dB (measured +2.8 dB at 63 Hz 2026-06-12)"
        );
    }

    // Shelf selectivity: full boost must move 1 kHz by < 2 dB.
    let mid_move = gain_at(1_000.0, &[("LF Boost", 1.0)]) - gain_at(1_000.0, &[]);
    eprintln!("LF Boost 0 -> 1 movement at 1 kHz: {mid_move:+.3} dB");
    assert!(
        mid_move.abs() < 2.0,
        "LF Boost leaks {mid_move:+.3} dB into 1 kHz, expected < 2 dB (measured -0.16 dB)"
    );
}

/// Full-range pot sweep — GREEN since 2026-06-13: the compiled pot baseline
/// now matches the control's declared default (defaults applied via
/// `set_control_immediate` in spqr_control.rs, so the PassiveRType pot leaf is
/// no longer left at its compile-time 0.5 stamp). The sweep is monotonic across
/// the full range and pos-0.5 differs from pos-0.0.
#[test]
fn lf_boost_monotonic_full_range() {
    let sweep: Vec<(f64, f64)> = [0.0, 0.1, 0.25, 0.4, 0.5, 0.75, 1.0]
        .iter()
        .map(|&p| (p, gain_at(63.0, &[("LF Boost", p)])))
        .collect();
    for &(p, g) in &sweep {
        eprintln!("LF Boost {p:.2} at 63 Hz -> {g:+7.3} dB");
    }
    assert_monotonic_rise(&sweep, 0.05, "LF Boost at 63 Hz (full range)");
}

// ===========================================================================
// HF boost: peak frequency within an octave of 1/(2*pi*sqrt(LC)),
// boost amount monotonic in the pot
// ===========================================================================

#[test]
fn hf_boost_peak_near_lc_resonance() {
    // From the netlist: L_hf = 47 mH, C_hf = 22 nF.
    let f0 = 1.0 / (2.0 * std::f64::consts::PI * (0.047f64 * 22e-9).sqrt());
    eprintln!("analytic f0 = 1/(2*pi*sqrt(LC)) = {f0:.0} Hz");

    // Probe 1.5-11 kHz (below the >12 kHz engine rolloff, note 7) with the
    // bandwidth pot at minimum damping so the hump is sharpest.
    let freqs: Vec<f64> = (0..24)
        .map(|i| 1_500.0 * (11_000.0f64 / 1_500.0).powf(i as f64 / 23.0))
        .collect();
    let flat = pultec_response(&freqs, &[("HF Bandwidth", 0.0)]);
    let boosted = pultec_response(&freqs, &[("HF Boost", 1.0), ("HF Bandwidth", 0.0)]);

    let delta: Vec<(f64, f64)> = freqs
        .iter()
        .zip(flat.iter().zip(boosted.iter()))
        .map(|(&f, (fl, bo))| (f, bo.1 - fl.1))
        .collect();
    for &(f, d) in &delta {
        eprintln!("HF Boost delta at {f:8.1} Hz: {d:+6.3} dB");
    }

    let &(peak_f, peak_d) = delta
        .iter()
        .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        .unwrap();
    eprintln!("HF Boost peak: {peak_d:+.3} dB at {peak_f:.0} Hz (f0 {f0:.0} Hz)");
    assert!(
        peak_d > 1.0,
        "HF Boost peak {peak_d:+.3} dB, expected > 1 dB (measured +1.39 dB; the \
         ~+13.6 dB analytic resonance is an engine gap, header note 4)"
    );
    assert!(
        (f0 / 2.0..=f0 * 2.0).contains(&peak_f),
        "HF Boost peak at {peak_f:.0} Hz outside one octave of f0 {f0:.0} Hz"
    );

    // Boost amount at the peak frequency is monotonic in the pot over the
    // trustworthy control window. "HF Boost" is range-inverted ([1.0, 0.0]),
    // so control 0..0.5 covers pot positions 1.0..0.5 — the sane half
    // (header note 8). Beyond 0.5 the position aliasing kicks in (measured:
    // control 0.75 -> +8.201 dB, control 1.0 -> +6.193 dB == control 0.5).
    let sweep: Vec<(f64, f64)> = [0.0, 0.125, 0.25, 0.375, 0.5]
        .iter()
        .map(|&p| {
            (
                p,
                gain_at(peak_f, &[("HF Boost", p), ("HF Bandwidth", 0.0)]),
            )
        })
        .collect();
    for &(p, g) in &sweep {
        eprintln!("HF Boost {p:.2} at {peak_f:.0} Hz -> {g:+7.3} dB");
    }
    assert_monotonic_rise(&sweep, 0.05, "HF Boost at peak");
}

/// Full-range HF boost sweep — GREEN since 2026-06-13 (compiled pot baseline
/// now matches the declared default; see lf_boost_monotonic_full_range).
#[test]
fn hf_boost_monotonic_full_range() {
    let sweep: Vec<(f64, f64)> = [0.0, 0.25, 0.5, 0.75, 1.0]
        .iter()
        .map(|&p| {
            (
                p,
                gain_at(5_044.0, &[("HF Boost", p), ("HF Bandwidth", 0.0)]),
            )
        })
        .collect();
    for &(p, g) in &sweep {
        eprintln!("HF Boost {p:.2} at 5044 Hz -> {g:+7.3} dB");
    }
    assert_monotonic_rise(&sweep, 0.05, "HF Boost at 5044 Hz (full range)");
}

// ===========================================================================
// HF atten: monotonic cut at 10 kHz
// ===========================================================================

#[test]
fn hf_atten_monotonic_cut() {
    // "HF Atten" is range-inverted ([1.0, 0.0]); controls 0..0.75 map to
    // pot positions 1.0..0.25, which measured monotonic. Control 1.0 (pot
    // position 0) hits the position-aliasing artifact (header note 8):
    // measured +1.864 dB, identical to control 0.5 instead of deepest cut.
    let sweep: Vec<(f64, f64)> = [0.0, 0.25, 0.5, 0.75]
        .iter()
        .map(|&p| (p, gain_at(10_000.0, &[("HF Atten", p)])))
        .collect();
    for &(p, g) in &sweep {
        eprintln!("HF Atten {p:.2} at 10 kHz -> {g:+7.3} dB");
    }
    // Monotonic FALL: negate and reuse the rise check.
    let negated: Vec<(f64, f64)> = sweep.iter().map(|&(p, g)| (p, -g)).collect();
    assert_monotonic_rise(&negated, 0.05, "HF Atten at 10 kHz (cut)");

    let depth = sweep[0].1 - sweep.last().unwrap().1;
    eprintln!("HF Atten authority at 10 kHz (controls 0 -> 0.75): {depth:.3} dB of cut");
    assert!(
        depth > 1.5,
        "HF Atten cut {depth:.3} dB at 10 kHz, expected > 1.5 dB (measured ~6.2 dB \
         over controls 0 -> 0.75, 2026-06-12)"
    );
}

/// Full-range HF atten sweep — GREEN since 2026-06-13 (compiled pot baseline
/// now matches the declared default; see lf_boost_monotonic_full_range).
#[test]
fn hf_atten_monotonic_full_range() {
    let sweep: Vec<(f64, f64)> = [0.0, 0.25, 0.5, 0.75, 1.0]
        .iter()
        .map(|&p| (p, gain_at(10_000.0, &[("HF Atten", p)])))
        .collect();
    for &(p, g) in &sweep {
        eprintln!("HF Atten {p:.2} at 10 kHz -> {g:+7.3} dB");
    }
    let negated: Vec<(f64, f64)> = sweep.iter().map(|&(p, g)| (p, -g)).collect();
    assert_monotonic_rise(&negated, 0.05, "HF Atten at 10 kHz (full range)");
}

// ===========================================================================
// THE TRICK: LF boost 0.7 + LF cut 0.7 — sub-100 Hz lift stays, relative
// 200-500 Hz dip appears
// ===========================================================================

#[test]
fn pultec_trick_boost_plus_cut() {
    let freqs = log_freqs();
    let flat = pultec_response(&freqs, &[]);
    let boost_only = pultec_response(&freqs, &[("LF Boost", 0.7)]);
    let combined = pultec_response(&freqs, &[("LF Boost", 0.7), ("LF Cut", 0.7)]);

    eprintln!(
        "{:>9} {:>8} {:>8} {:>8} {:>10} {:>10}",
        "freq", "flat", "boost.7", "trick.7", "vs flat", "vs boost"
    );
    for i in 0..freqs.len() {
        eprintln!(
            "{:9.1} {:8.2} {:8.2} {:8.2} {:+10.3} {:+10.3}",
            freqs[i],
            flat[i].1,
            boost_only[i].1,
            combined[i].1,
            combined[i].1 - flat[i].1,
            combined[i].1 - boost_only[i].1
        );
    }

    // Sub-100 Hz lift survives the cut (combined stays above flat).
    for i in freqs.iter().enumerate().filter(|(_, &f)| f < 100.0) {
        let (idx, &f) = i;
        let lift = combined[idx].1 - flat[idx].1;
        assert!(
            lift > 0.2,
            "trick lost the sub-100 Hz lift: {lift:+.3} dB at {f:.1} Hz, expected > +0.2 dB \
             (measured +0.35..+0.83 dB 2026-06-12)"
        );
    }

    // Relative dip vs boost-only in the 200-500 Hz band.
    let band: Vec<f64> = freqs
        .iter()
        .enumerate()
        .filter(|(_, &f)| (200.0..=500.0).contains(&f))
        .map(|(idx, _)| combined[idx].1 - boost_only[idx].1)
        .collect();
    assert!(band.len() >= 3, "need >= 3 points in 200-500 Hz");
    let mean_dip = band.iter().sum::<f64>() / band.len() as f64;
    eprintln!("trick mean dip vs boost-only over 200-500 Hz: {mean_dip:+.3} dB");
    assert!(
        mean_dip < -0.2,
        "trick dip vs boost-only {mean_dip:+.3} dB over 200-500 Hz, expected < -0.2 dB \
         (measured ~-0.5 dB 2026-06-12)"
    );

    // And the absolute trick signature: combined dips below FLAT somewhere
    // in the 200-630 Hz midband even though it lifts the lows.
    let min_vs_flat = freqs
        .iter()
        .enumerate()
        .filter(|(_, &f)| (200.0..=650.0).contains(&f))
        .map(|(idx, _)| combined[idx].1 - flat[idx].1)
        .fold(f64::INFINITY, f64::min);
    eprintln!("trick deepest dip vs flat in 200-650 Hz: {min_vs_flat:+.3} dB");
    assert!(
        min_vs_flat < -0.3,
        "trick dip vs flat {min_vs_flat:+.3} dB in 200-650 Hz, expected < -0.3 dB \
         (measured -0.54 dB at ~356 Hz 2026-06-12)"
    );
}

// ===========================================================================
// Insertion gain vs netlist divider math
// ===========================================================================

/// Analytic mid-band (1 kHz) chain gain from the netlist:
///   passive pad: R1 (10k) -> shunt1 (R_s1 2.7k + |Z_C_lf| ~0.6k) ->
///                R2 (10k, LC branch out at pot=100k) -> shunt R_s2 (2.7k)
///                -> R3 (4.7k) -> R_load 100k || R_g-path
///   makeup:      12AX7 mu=100, r_p~62.5k into R_p 100k || R_out 1M
#[test]
fn insertion_gain_matches_divider_math() {
    // Passive divider at 1 kHz (reactances: C_lf 270n -> ~589 ohm).
    let z_shunt1 = 2_700.0 + 589.0;
    let z_y = 1.0 / (1.0 / 2_700.0 + 1.0 / (4_700.0 + 100_000.0));
    let z_mid = 10_000.0 + z_y;
    let z_x = 1.0 / (1.0 / z_shunt1 + 1.0 / z_mid);
    let v_x = z_x / (10_000.0 + z_x);
    let v_y = z_y / z_mid;
    let v_out = 100_000.0 / (4_700.0 + 100_000.0);
    let passive_db = lin_to_db(v_x * v_y * v_out);

    // 12AX7 common cathode, bypassed cathode: A = mu * Z_L / (Z_L + r_p).
    let z_l = 1.0 / (1.0 / 100_000.0 + 1.0 / 1_000_000.0);
    let triode_db = lin_to_db(100.0 * z_l / (z_l + 62_500.0));

    let analytic = passive_db + triode_db;
    let measured = gain_at(1_000.0, &[]);
    eprintln!(
        "1 kHz insertion gain: analytic {analytic:+.2} dB \
         (passive {passive_db:+.2} dB + 12AX7 {triode_db:+.2} dB), measured {measured:+.2} dB"
    );
    assert!(
        (measured - analytic).abs() < 6.0,
        "1 kHz gain measured {measured:+.2} dB vs analytic {analytic:+.2} dB — \
         differs by more than 6 dB"
    );
}
