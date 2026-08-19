// TDD tests for pot sweep behavior across all legend pedals.
//
// Each pot control should measurably change either:
// - **Gain**: output level changes when the pot sweeps (Volume, Level, Drive, Boost)
// - **Spectral**: frequency balance changes (Tone, Filter, Treble, Bass)
//
// Tests verify that sweeping from 0.0 to 1.0 produces a measurable difference
// in the output. A pot that does nothing indicates a binding or routing failure.

use super::spqr_build::compile_via_spqr;
use super::test_support::load_legend_source;
use super::{compile_pedal_with_options, CompileOptions};
use crate::PedalProcessor;
use std::collections::HashSet;

fn load_legend(name: &str) -> crate::dsl::PedalDef {
    let source = load_legend_source(name)
        .unwrap_or_else(|| panic!("Can't read pedals/legends/{name}.pedal"));
    crate::dsl::parse_pedal_file(&source).unwrap_or_else(|e| panic!("{name}: parse error: {e}"))
}

fn measure_default_rms_peak(mut compiled: pedalkernel_rt::processor::CompiledPedal) -> (f64, f64) {
    for i in 0..24_000 {
        let input = 0.05 * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 48000.0).sin();
        compiled.process(input);
    }
    let mut sum_sq = 0.0;
    let mut peak = 0.0f64;
    for i in 0..4_800 {
        let input =
            0.05 * (2.0 * std::f64::consts::PI * 1000.0 * (24_000 + i) as f64 / 48000.0).sin();
        let out = compiled.process(input);
        sum_sq += out * out;
        peak = peak.max(out.abs());
    }
    ((sum_sq / 4_800.0).sqrt(), peak)
}

#[test]
fn legend_default_output_calibration_uses_bound_controls() {
    let mut rms_values = Vec::new();
    for name in ["ratking_non_invert_v1a", "goldenrod", "screamer", "sd1"] {
        let pedal = load_legend(name);
        let compiled = compile_via_spqr(&pedal, 48000.0).expect("compile");
        let pre_gain = compiled.pre_gain;
        let output_gain = compiled.output_gain;
        let stages = compiled.stages.len();
        let (rms, peak) = measure_default_rms_peak(compiled);
        eprintln!(
            "{name}: calibrate={} stages={} pre_gain={pre_gain:.6} output_gain={output_gain:.6} rms={rms:.6} peak={peak:.6}",
            pedal.calibrate, stages,
        );
        assert!(
            peak < 0.5,
            "{name}: calibrated default output is too hot: peak={peak:.6}, rms={rms:.6}, output_gain={output_gain:.6}"
        );
        rms_values.push((name, rms));
    }

    let min_rms = rms_values
        .iter()
        .map(|(_, rms)| *rms)
        .fold(f64::INFINITY, f64::min);
    let max_rms = rms_values
        .iter()
        .map(|(_, rms)| *rms)
        .fold(0.0f64, f64::max);
    assert!(
        max_rms / min_rms < 5.0,
        "calibrated default outputs are not comparable: {rms_values:?}"
    );
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

fn measure_peak_skip_blockwise(pedal: &crate::dsl::PedalDef, control: &str, value: f64) -> f64 {
    let opts = CompileOptions {
        skip_blockwise: true,
        skip_k_tables: true,
        ..Default::default()
    };
    let mut compiled = compile_pedal_with_options(pedal, 48000.0, opts).expect("compile");
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

fn assert_skip_blockwise_gain_changes(
    pedal: &crate::dsl::PedalDef,
    pedal_name: &str,
    control: &str,
) {
    let low = measure_peak_skip_blockwise(pedal, control, 0.0);
    let high = measure_peak_skip_blockwise(pedal, control, 1.0);
    let diff = (low - high).abs();
    let ratio = low.max(high) / low.min(high).max(1e-10);
    eprintln!(
        "{pedal_name}/{control} skip_blockwise: low={low:.4}, high={high:.4}, ratio={ratio:.1}x"
    );
    assert!(
        diff > 0.001 || ratio > 1.5,
        "{pedal_name}/{control} lost feedback control when blockwise is skipped (low={low:.4}, high={high:.4})"
    );
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

fn measure_bound_stage_spectrum(
    pedal: &crate::dsl::PedalDef,
    control: &str,
    value: f64,
) -> (f64, f64) {
    let measure = |freq: f64| -> f64 {
        let mut compiled = compile_via_spqr(pedal, 48000.0).expect("compile");
        compiled.set_control(control, value);
        for _ in 0..3000 {
            compiled.process(0.0);
        }

        let stage_idx = compiled
            .controls
            .iter()
            .find(|c| c.label.eq_ignore_ascii_case(control))
            .and_then(|c| match c.target {
                super::compiled::ControlTarget::PotInStage(idx) => Some(idx),
                super::compiled::ControlTarget::PotInIirStage(idx) => Some(idx),
                super::compiled::ControlTarget::PotInBlackFeedbackStage(idx) => Some(idx),
                _ => None,
            })
            .expect("control should bind to a stage");

        let amp = 0.05;
        let mut peak = 0.0f64;
        for s in 0..4800 {
            let input = amp * (2.0 * std::f64::consts::PI * freq * s as f64 / 48000.0).sin();
            let out = match &mut compiled.stages[stage_idx] {
                super::compiled::Stage::Wdf(w) => w.process(input),
                super::compiled::Stage::Iir(i) => i.process(input),
                super::compiled::Stage::BlackFeedback(b) => b.process(input),
                _ => panic!("unsupported stage type for spectral control test"),
            };
            peak = peak.max(out.abs());
        }
        peak
    };

    (measure(200.0), measure(5000.0))
}

fn assert_bound_stage_spectrum_changes(
    pedal: &crate::dsl::PedalDef,
    pedal_name: &str,
    control: &str,
) {
    let (lo_0, hi_0) = measure_bound_stage_spectrum(pedal, control, 0.0);
    let (lo_1, hi_1) = measure_bound_stage_spectrum(pedal, control, 1.0);

    let ratio_0 = if lo_0 > 1e-10 { hi_0 / lo_0 } else { 0.0 };
    let ratio_1 = if lo_1 > 1e-10 { hi_1 / lo_1 } else { 0.0 };
    eprintln!("{pedal_name}/{control} bound stage: @0: lo={lo_0:.6} hi={hi_0:.6} ratio={ratio_0:.3} | @1: lo={lo_1:.6} hi={hi_1:.6} ratio={ratio_1:.3}");

    let has_output = lo_0 > 0.0001 || hi_0 > 0.0001 || lo_1 > 0.0001 || hi_1 > 0.0001;
    assert!(
        has_output,
        "{pedal_name}/{control}: bound tone stage has no output"
    );

    let ratio_diff = (ratio_0 - ratio_1).abs();
    let level_diff = (lo_0 - lo_1).abs() + (hi_0 - hi_1).abs();
    assert!(
        ratio_diff > 0.02 || level_diff > 0.0005,
        "{pedal_name}/{control}: bound tone stage has no spectral effect (ratio_diff={ratio_diff:.3}, level_diff={level_diff:.6})"
    );
}

fn settle_control(proc: &mut super::compiled::CompiledPedal) {
    for _ in 0..6000 {
        proc.process(0.0);
    }
}

fn smoothed_values_for_control(proc: &super::compiled::CompiledPedal, label: &str) -> Vec<f64> {
    proc.resolve_control(label)
        .into_iter()
        .map(|control_idx| {
            proc.pot_smoothers
                .iter()
                .find(|s| s.control_idx == control_idx)
                .unwrap_or_else(|| panic!("missing smoother for {label} binding {control_idx}"))
                .current
        })
        .collect()
}

fn debug_leaf_values_for_component(
    proc: &super::compiled::CompiledPedal,
    component_id: &str,
) -> Vec<(f64, f64)> {
    proc.control_debug_info()
        .into_iter()
        .filter_map(|(name, position, resistance)| {
            (name.starts_with('[') && name.ends_with(component_id))
                .then_some((position, resistance))
        })
        .collect()
}

#[test]
fn core_legend_12_user_pots_have_runtime_bindings() {
    let cases: &[(&str, &str, &[(&str, usize)])] = &[
        (
            "ratking_v1a",
            "ratking_non_invert_v1a",
            &[("Distortion", 1), ("Filter", 1), ("Volume", 1)],
        ),
        (
            "screamer",
            "screamer",
            &[("Drive", 1), ("Tone", 2), ("Level", 1)],
        ),
        ("sd1", "sd1", &[("Drive", 1), ("Tone", 2), ("Level", 1)]),
        (
            "goldenrod",
            "goldenrod",
            &[("Gain", 3), ("Treble", 1), ("Output", 1)],
        ),
    ];

    let mut user_pot_count = 0;
    let mut runtime_binding_count = 0;

    for (display_name, legend_name, expected_controls) in cases {
        let pedal = load_legend(legend_name);
        let declared_labels: HashSet<&str> = pedal
            .controls
            .iter()
            .map(|control| control.label.as_str())
            .collect();

        let expected_labels: HashSet<&str> =
            expected_controls.iter().map(|(label, _)| *label).collect();
        assert_eq!(
            declared_labels, expected_labels,
            "{display_name}: declared controls changed"
        );

        let mut proc = compile_via_spqr(&pedal, 48_000.0)
            .unwrap_or_else(|e| panic!("{display_name}: compile failed: {e}"));

        for (label, expected_bindings) in *expected_controls {
            user_pot_count += 1;

            let binding_indices = proc.resolve_control(label);
            runtime_binding_count += binding_indices.len();
            assert_eq!(
                binding_indices.len(),
                *expected_bindings,
                "{display_name}/{label}: wrong runtime binding count"
            );

            proc.set_control(label, 0.13);
            settle_control(&mut proc);
            let low_values = smoothed_values_for_control(&proc, label);
            let low_leaf_values: Vec<_> = pedal
                .controls
                .iter()
                .filter(|control| control.label == *label)
                .map(|control| {
                    (
                        control.component.as_str(),
                        debug_leaf_values_for_component(&proc, &control.component),
                    )
                })
                .collect();

            proc.set_control(label, 0.87);
            settle_control(&mut proc);
            let high_values = smoothed_values_for_control(&proc, label);
            let high_leaf_values: Vec<_> = pedal
                .controls
                .iter()
                .filter(|control| control.label == *label)
                .map(|control| {
                    (
                        control.component.as_str(),
                        debug_leaf_values_for_component(&proc, &control.component),
                    )
                })
                .collect();

            assert_eq!(
                low_values.len(),
                high_values.len(),
                "{display_name}/{label}: binding count changed during sweep"
            );

            let changed = low_values
                .iter()
                .zip(&high_values)
                .any(|(low, high)| (low - high).abs() > 0.05);
            eprintln!(
                "{display_name}/{label}: bindings={} low={low_values:?} high={high_values:?}",
                binding_indices.len()
            );
            assert!(
                changed,
                "{display_name}/{label}: set_control did not move runtime smoother state"
            );

            for ((component, low_leaf), (_, high_leaf)) in
                low_leaf_values.iter().zip(&high_leaf_values)
            {
                if low_leaf.is_empty() && high_leaf.is_empty() {
                    continue;
                }
                assert_eq!(
                    low_leaf.len(),
                    high_leaf.len(),
                    "{display_name}/{label}/{component}: debug leaf count changed"
                );
                let leaf_changed =
                    low_leaf
                        .iter()
                        .zip(high_leaf)
                        .any(|((low_pos, low_r), (high_pos, high_r))| {
                            (low_pos - high_pos).abs() > 0.05 || (low_r - high_r).abs() > 1.0
                        });
                eprintln!(
                    "{display_name}/{label}/{component}: leaf low={low_leaf:?} high={high_leaf:?}"
                );
                assert!(
                    leaf_changed,
                    "{display_name}/{label}/{component}: stage leaf did not move"
                );
            }
        }
    }

    assert_eq!(user_pot_count, 12, "expected 12 user-facing pots");
    assert_eq!(
        runtime_binding_count, 16,
        "expected split passive controls to bind every runtime stage owner"
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

#[test]
fn ratking_v1a_distortion_survives_skip_blockwise() {
    let pedal = load_legend("ratking_non_invert_v1a");
    assert_skip_blockwise_gain_changes(&pedal, "ratking_v1a", "Distortion");
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
fn screamer_drive_survives_skip_blockwise() {
    let pedal = load_legend("screamer");
    assert_skip_blockwise_gain_changes(&pedal, "screamer", "Drive");
}

#[test]
fn screamer_tone_changes_spectrum() {
    let pedal = load_legend("screamer");
    assert_bound_stage_spectrum_changes(&pedal, "screamer", "Tone");
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
    assert_bound_stage_spectrum_changes(&pedal, "sd1", "Tone");
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
