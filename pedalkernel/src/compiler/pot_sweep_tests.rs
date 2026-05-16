// TDD tests for pot sweep behavior across all legend pedals.
//
// Each pot control should measurably change either:
// - **Gain**: output level changes when the pot sweeps (Volume, Level, Drive, Boost)
// - **Spectral**: frequency balance changes (Tone, Filter, Treble, Bass)
//
// Tests verify that sweeping from 0.0 to 1.0 produces a measurable difference
// in the output. A pot that does nothing indicates a binding or routing failure.

use super::spqr_build::compile_via_spqr;
use crate::PedalProcessor;

fn load_legend(name: &str) -> crate::dsl::PedalDef {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/{}.pedal",
        env!("CARGO_MANIFEST_DIR"),
        name
    );
    let source = std::fs::read_to_string(&path).unwrap_or_else(|_| panic!("Can't read {path}"));
    crate::dsl::parse_pedal_file(&source).unwrap_or_else(|e| panic!("{name}: parse error: {e}"))
}

/// Measure peak output at a given control setting.
/// Feeds 440Hz guitar-level signal, returns peak absolute output.
fn measure_peak(pedal: &crate::dsl::PedalDef, control: &str, value: f64) -> f64 {
    let mut compiled = compile_via_spqr(pedal, 48000.0).expect("compile");
    compiled.set_control(control, value);
    // Settle with control applied
    for _ in 0..2000 {
        compiled.process(0.0);
    }
    let mut peak = 0.0f64;
    for s in 0..4800 {
        let input = 0.05 * (2.0 * std::f64::consts::PI * 440.0 * s as f64 / 48000.0).sin();
        peak = peak.max(compiled.process(input).abs());
    }
    peak
}

/// VST path applies an input impedance model before processing. Exercise that
/// integration because it can change boundary loading without changing bindings.
fn measure_peak_with_source_z(pedal: &crate::dsl::PedalDef, control: &str, value: f64) -> f64 {
    let mut compiled = compile_via_spqr(pedal, 48000.0).expect("compile");
    compiled.set_source_impedance(7_000.0, 500e-12);
    compiled.set_control(control, value);
    for _ in 0..2000 {
        compiled.process(0.0);
    }
    let mut peak = 0.0f64;
    for s in 0..4800 {
        let input = 0.05 * (2.0 * std::f64::consts::PI * 440.0 * s as f64 / 48000.0).sin();
        peak = peak.max(compiled.process(input).abs());
    }
    peak
}

/// Measure spectral balance: ratio of high-frequency energy to low-frequency energy.
/// Returns (peak_lo, peak_hi) at 200Hz and 5kHz.
fn measure_spectrum(pedal: &crate::dsl::PedalDef, control: &str, value: f64) -> (f64, f64) {
    // 200Hz
    let mut lo = compile_via_spqr(pedal, 48000.0).expect("compile");
    lo.set_control(control, value);
    for _ in 0..2000 {
        lo.process(0.0);
    }
    let mut peak_lo = 0.0f64;
    for s in 0..4800 {
        let input = 0.05 * (2.0 * std::f64::consts::PI * 200.0 * s as f64 / 48000.0).sin();
        peak_lo = peak_lo.max(lo.process(input).abs());
    }

    // 5kHz
    let mut hi = compile_via_spqr(pedal, 48000.0).expect("compile");
    hi.set_control(control, value);
    for _ in 0..2000 {
        hi.process(0.0);
    }
    let mut peak_hi = 0.0f64;
    for s in 0..4800 {
        let input = 0.05 * (2.0 * std::f64::consts::PI * 5000.0 * s as f64 / 48000.0).sin();
        peak_hi = peak_hi.max(hi.process(input).abs());
    }

    (peak_lo, peak_hi)
}

/// Assert that sweeping a control changes the output gain.
fn assert_gain_changes(pedal: &crate::dsl::PedalDef, pedal_name: &str, control: &str) {
    let low = measure_peak(pedal, control, 0.0);
    let high = measure_peak(pedal, control, 1.0);

    eprintln!("{pedal_name}/{control}: low={low:.4}, high={high:.4}");

    // At least one setting should produce output
    let has_output = low > 0.001 || high > 0.001;
    assert!(
        has_output,
        "{pedal_name}/{control}: neither setting produces output (low={low:.6}, high={high:.6})"
    );

    // The two settings should differ (pot actually does something)
    let max_val = low.max(high);
    let min_val = low.min(high);
    let ratio = if min_val > 1e-10 {
        max_val / min_val
    } else if max_val > 0.001 {
        100.0
    } else {
        1.0
    };
    assert!(
        ratio > 1.5 || (low - high).abs() > 0.001,
        "{pedal_name}/{control}: pot sweep has no effect (low={low:.4}, high={high:.4}, ratio={ratio:.1}x)"
    );
}

fn assert_vst_gain_changes(pedal: &crate::dsl::PedalDef, pedal_name: &str, control: &str) {
    let low = measure_peak_with_source_z(pedal, control, 0.0);
    let high = measure_peak_with_source_z(pedal, control, 1.0);

    eprintln!("{pedal_name}/{control} with source Z: low={low:.4}, high={high:.4}");

    let has_output = low > 0.001 || high > 0.001;
    assert!(
        has_output,
        "{pedal_name}/{control} with source Z: neither setting produces output (low={low:.6}, high={high:.6})"
    );

    let max_val = low.max(high);
    let min_val = low.min(high);
    let ratio = if min_val > 1e-10 {
        max_val / min_val
    } else if max_val > 0.001 {
        100.0
    } else {
        1.0
    };
    assert!(
        ratio > 1.5 || (low - high).abs() > 0.001,
        "{pedal_name}/{control} with source Z: pot sweep has no effect (low={low:.4}, high={high:.4}, ratio={ratio:.1}x)"
    );
}

/// Assert that sweeping a control changes the spectral balance.
fn assert_spectrum_changes(pedal: &crate::dsl::PedalDef, pedal_name: &str, control: &str) {
    let (lo_0, hi_0) = measure_spectrum(pedal, control, 0.0);
    let (lo_1, hi_1) = measure_spectrum(pedal, control, 1.0);

    let ratio_0 = if lo_0 > 1e-10 { hi_0 / lo_0 } else { 0.0 };
    let ratio_1 = if lo_1 > 1e-10 { hi_1 / lo_1 } else { 0.0 };

    eprintln!("{pedal_name}/{control}: @0: lo={lo_0:.4} hi={hi_0:.4} ratio={ratio_0:.2} | @1: lo={lo_1:.4} hi={hi_1:.4} ratio={ratio_1:.2}");

    // At least one setting should produce output
    let has_output = lo_0 > 0.001 || hi_0 > 0.001 || lo_1 > 0.001 || hi_1 > 0.001;
    assert!(
        has_output,
        "{pedal_name}/{control}: no output at any setting"
    );

    // The spectral balance should change
    let ratio_diff = (ratio_0 - ratio_1).abs();
    let level_diff = (lo_0 - lo_1).abs() + (hi_0 - hi_1).abs();
    assert!(
        ratio_diff > 0.1 || level_diff > 0.001,
        "{pedal_name}/{control}: tone pot has no spectral effect (ratio_diff={ratio_diff:.2}, level_diff={level_diff:.4})"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// RATKING (v1a — the VST version)
// Controls: Distortion [0,1], Filter [0,1], Volume [1,0]
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn ratking_v1a_distortion_changes_gain() {
    let pedal = load_legend("ratking_non_invert_v1a");
    assert_gain_changes(&pedal, "ratking_v1a", "Distortion");
}

#[test]
fn ratking_v1a_filter_changes_spectrum() {
    let pedal = load_legend("ratking_non_invert_v1a");
    assert_spectrum_changes(&pedal, "ratking_v1a", "Filter");
}

#[test]
fn ratking_v1a_volume_changes_gain() {
    let pedal = load_legend("ratking_non_invert_v1a");
    assert_gain_changes(&pedal, "ratking_v1a", "Volume");
}

#[test]
fn ratking_v1a_vst_distortion_and_volume_survive_source_impedance() {
    let pedal = load_legend("ratking_non_invert_v1a");
    assert_vst_gain_changes(&pedal, "ratking_v1a", "Distortion");
    assert_vst_gain_changes(&pedal, "ratking_v1a", "Volume");
}

// ═══════════════════════════════════════════════════════════════════════════
// SCREAMER (TS-808)
// Controls: Drive [0,1], Tone [0,1], Level [1,0]
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_drive_changes_gain() {
    let pedal = load_legend("screamer");
    assert_gain_changes(&pedal, "screamer", "Drive");
}

#[test]
fn screamer_tone_changes_spectrum() {
    let pedal = load_legend("screamer");
    assert_spectrum_changes(&pedal, "screamer", "Tone");
}

#[test]
fn screamer_level_changes_gain() {
    let pedal = load_legend("screamer");
    assert_gain_changes(&pedal, "screamer", "Level");
}

// ═══════════════════════════════════════════════════════════════════════════
// SD-1
// Controls: Drive [0,1], Tone [0,1], Level [1,0]
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn sd1_drive_changes_gain() {
    let pedal = load_legend("sd1");
    assert_gain_changes(&pedal, "sd1", "Drive");
}

#[test]
fn sd1_tone_changes_spectrum() {
    let pedal = load_legend("sd1");
    assert_spectrum_changes(&pedal, "sd1", "Tone");
}

#[test]
fn sd1_level_changes_gain() {
    let pedal = load_legend("sd1");
    assert_gain_changes(&pedal, "sd1", "Level");
}

// ═══════════════════════════════════════════════════════════════════════════
// BLUES (Marshall Bluesbreaker)
// Controls: Drive [1,0], Tone [0,1], Volume [1,0]
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn blues_drive_changes_gain() {
    let pedal = load_legend("blues");
    assert_gain_changes(&pedal, "blues", "Drive");
}

#[test]
fn blues_tone_changes_spectrum() {
    let pedal = load_legend("blues");
    assert_spectrum_changes(&pedal, "blues", "Tone");
}

#[test]
fn blues_volume_changes_gain() {
    let pedal = load_legend("blues");
    assert_gain_changes(&pedal, "blues", "Volume");
}

// ═══════════════════════════════════════════════════════════════════════════
// MUFF (Big Muff Pi)
// Controls: Sustain [0,1], Tone [0,1], Volume [1,0]
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn muff_sustain_changes_gain() {
    let pedal = load_legend("muff");
    assert_gain_changes(&pedal, "muff", "Sustain");
}

#[test]
fn muff_tone_changes_spectrum() {
    let pedal = load_legend("muff");
    assert_spectrum_changes(&pedal, "muff", "Tone");
}

#[test]
fn muff_volume_changes_gain() {
    let pedal = load_legend("muff");
    assert_gain_changes(&pedal, "muff", "Volume");
}

// ═══════════════════════════════════════════════════════════════════════════
// FIZZ (Fuzz Face)
// Controls: Fuzz [0,1], Volume [1,0]
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn fizz_fuzz_changes_gain() {
    let pedal = load_legend("fizz");
    assert_gain_changes(&pedal, "fizz", "Fuzz");
}

#[test]
fn fizz_volume_changes_gain() {
    let pedal = load_legend("fizz");
    assert_gain_changes(&pedal, "fizz", "Volume");
}

// ═══════════════════════════════════════════════════════════════════════════
// RANGEMASTER
// Controls: Boost [0,1]
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn rangemaster_boost_changes_gain() {
    let pedal = load_legend("rangemaster");
    assert_gain_changes(&pedal, "rangemaster", "Boost");
}

// ═══════════════════════════════════════════════════════════════════════════
// GOLDENROD (Klon Centaur)
// Controls: Gain [1,0]+[0,1] (dual gang), Treble [1,0], Output [1,0]
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn goldenrod_gain_changes_gain() {
    let pedal = load_legend("goldenrod");
    assert_gain_changes(&pedal, "goldenrod", "Gain");
}

#[test]
fn goldenrod_treble_changes_spectrum() {
    let pedal = load_legend("goldenrod");
    assert_spectrum_changes(&pedal, "goldenrod", "Treble");
}

#[test]
fn goldenrod_output_changes_gain() {
    let pedal = load_legend("goldenrod");
    assert_gain_changes(&pedal, "goldenrod", "Output");
}

#[test]
fn goldenrod_vst_gain_and_output_survive_source_impedance() {
    let pedal = load_legend("goldenrod");
    assert_vst_gain_changes(&pedal, "goldenrod", "Gain");
    assert_vst_gain_changes(&pedal, "goldenrod", "Output");
}

// ═══════════════════════════════════════════════════════════════════════════
// SHRINE (Fender 6G15 Spring Reverb)
// Controls: Dwell [0,1], Tone [0,1], Mix [0,1], Decay [0,0.85]
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn shrine_dwell_changes_gain() {
    let pedal = load_legend("shrine");
    assert_gain_changes(&pedal, "shrine", "Dwell");
}

#[test]
fn shrine_tone_changes_spectrum() {
    let pedal = load_legend("shrine");
    assert_spectrum_changes(&pedal, "shrine", "Tone");
}

#[test]
fn shrine_mix_changes_gain() {
    let pedal = load_legend("shrine");
    assert_gain_changes(&pedal, "shrine", "Mix");
}
