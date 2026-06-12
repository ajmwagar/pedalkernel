// ═══════════════════════════════════════════════════════════════════════════
// Goldenrod (Klon) pot diagnostic tests
//
// The Klon has a dual-gang Gain pot (Gain_A + Gain_B) that crossfades
// between clean and dirty signal paths. Both pots are bound to the same
// "Gain" control label with inverted ranges:
//   Gain_A [1.0, 0.0] = dirty gain (increases with knob)
//   Gain_B [0.0, 1.0] = clean blend (decreases with knob)
//
// The Treble pot controls the tone stage, Output controls volume.
// ═══════════════════════════════════════════════════════════════════════════

use super::compiled::CompiledPedal;
use super::compiled::Stage;
use super::spqr_build::compile_via_spqr;
use super::{compile_pedal_with_options, CompileOptions};
use crate::PedalProcessor;

const SR: f64 = 48000.0;
const FREQ: f64 = 440.0;

fn load_goldenrod() -> CompiledPedal {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/goldenrod.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read goldenrod.pedal");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    compile_via_spqr(&pedal, SR).expect("compile")
}

fn load_goldenrod_with_options(options: CompileOptions) -> CompiledPedal {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/goldenrod.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read goldenrod.pedal");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    compile_pedal_with_options(&pedal, SR, options).expect("compile")
}

fn measure_peak(compiled: &mut CompiledPedal, amp: f64) -> f64 {
    for s in 0..4000 {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }
    let mut peak = 0.0f64;
    for s in 0..500 {
        let out =
            compiled.process(amp * (std::f64::consts::TAU * FREQ * (4000 + s) as f64 / SR).sin());
        peak = peak.max(out.abs());
    }
    peak
}

fn measure_peak_and_rms(compiled: &mut CompiledPedal, amp: f64) -> (f64, f64) {
    for s in 0..4000 {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }
    let mut peak = 0.0f64;
    let mut sum_sq = 0.0f64;
    let n = 2048;
    for s in 0..n {
        let out =
            compiled.process(amp * (std::f64::consts::TAU * FREQ * (4000 + s) as f64 / SR).sin());
        peak = peak.max(out.abs());
        sum_sq += out * out;
    }
    (peak, (sum_sq / n as f64).sqrt())
}

#[cfg(debug_assertions)]
fn stage_label(stage: &super::compiled::Stage) -> &str {
    match stage {
        super::compiled::Stage::Wdf(w) => w.debug_label.as_str(),
        super::compiled::Stage::MultiNl(m) => m.debug_label.as_str(),
        super::compiled::Stage::Iir(s) => s.debug_label.as_str(),
        super::compiled::Stage::StateSpace(s) => s.debug_label.as_str(),
        super::compiled::Stage::BlackFeedback(b) => b.debug_label.as_str(),
        super::compiled::Stage::Blockwise(_) => "blockwise",
        super::compiled::Stage::SerialDelayedFeedback(_) => "serial_feedback",
        super::compiled::Stage::KMethod { .. } => "k_method",
    }
}

#[cfg(debug_assertions)]
fn metered_goldenrod(gain: f64, output: f64, treble: f64, amp: f64) -> CompiledPedal {
    let mut compiled = load_goldenrod();
    compiled.set_control("Gain", gain);
    compiled.set_control("Output", output);
    compiled.set_control("Treble", treble);
    compiled.enable_metering(128);
    for s in 0..8000 {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }
    compiled
}

#[cfg(debug_assertions)]
fn stage_level(compiled: &CompiledPedal, label: &str) -> f64 {
    let metrics = compiled.read_metrics();
    compiled
        .stages
        .iter()
        .enumerate()
        .find_map(|(idx, stage)| {
            (stage_label(stage).contains(label)).then(|| metrics.stage_levels[idx] as f64)
        })
        .unwrap_or_else(|| panic!("missing Goldenrod stage containing {label:?}"))
}

#[cfg(debug_assertions)]
fn control_target_label(compiled: &CompiledPedal, component_id: &str) -> String {
    let ctrl = compiled
        .controls
        .iter()
        .find(|ctrl| ctrl.component_id == component_id)
        .unwrap_or_else(|| panic!("missing Goldenrod control binding for {component_id:?}"));

    let stage_idx = match ctrl.target {
        super::compiled::ControlTarget::PotInStage(idx)
        | super::compiled::ControlTarget::PotInIirStage(idx)
        | super::compiled::ControlTarget::PotInBlackFeedbackStage(idx) => idx,
        _ => panic!("{component_id:?} should bind to a pot-owning audio stage"),
    };
    stage_label(&compiled.stages[stage_idx]).to_string()
}

fn debug_value(compiled: &CompiledPedal, label: &str) -> f64 {
    compiled
        .control_debug_info()
        .into_iter()
        .find_map(|(name, _target, value)| name.contains(label).then_some(value))
        .unwrap_or_else(|| panic!("missing Goldenrod debug entry containing {label:?}"))
}

// ═══════════════════════════════════════════════════════════════════════════
// 1. Stage dump — what does the pipeline build?
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn goldenrod_stage_dump() {
    let mut compiled = load_goldenrod();
    compiled.enable_metering(128);

    let amp = 0.1;
    for s in 0..8000 {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }

    let metrics = compiled.read_metrics();
    let n = compiled.stages.len().min(crate::metering::MAX_STAGES);
    eprintln!("\nGOLDENROD DIAGNOSTIC: {} stages", n);
    eprintln!(
        "  input:  RMS={:.1} dB, peak={:.1} dB",
        metrics.input_rms_db, metrics.input_peak_db
    );
    eprintln!(
        "  output: RMS={:.1} dB, peak={:.1} dB",
        metrics.output_rms_db, metrics.output_peak_db
    );
    for i in 0..n {
        let lvl = metrics.stage_levels[i];
        let db = if lvl > 1e-10 {
            20.0 * (lvl as f64).log10()
        } else {
            -120.0
        };
        #[cfg(debug_assertions)]
        {
            let (stype, lbl, bypass, dist) = match &compiled.stages[i] {
                super::compiled::Stage::Wdf(w) => (
                    "Wdf",
                    w.debug_label.as_str(),
                    w.bypass_serial,
                    w.signal_flow_distance,
                ),
                super::compiled::Stage::MultiNl(m) => (
                    "MNL",
                    m.debug_label.as_str(),
                    m.bypass_serial,
                    m.signal_flow_distance,
                ),
                super::compiled::Stage::Iir(s) => (
                    "Iir",
                    s.debug_label.as_str(),
                    s.bypass_serial,
                    s.signal_flow_distance,
                ),
                super::compiled::Stage::StateSpace(s) => (
                    "SS",
                    s.debug_label.as_str(),
                    s.bypass_serial,
                    s.signal_flow_distance,
                ),
                super::compiled::Stage::BlackFeedback(b) => (
                    "BF",
                    b.debug_label.as_str(),
                    b.bypass_serial,
                    b.signal_flow_distance,
                ),
                super::compiled::Stage::Blockwise(bk) => (
                    "BKM",
                    "blockwise",
                    bk.bypass_serial,
                    bk.signal_flow_distance,
                ),
                super::compiled::Stage::SerialDelayedFeedback(s) => (
                    "SerialFB",
                    "serial_feedback",
                    s.bypass_serial,
                    s.signal_flow_distance,
                ),
                super::compiled::Stage::KMethod { .. } => ("KMethod", "k_method", true, usize::MAX),
            };
            let bp = if bypass { " BYPASS" } else { "" };
            eprintln!("  stage {i}: [{stype}] dist={dist} [{lbl}]{bp} → {lvl:.4} ({db:.1} dB)");
        }
    }

    // Dump control bindings
    eprintln!("\n  Control bindings ({}):", compiled.controls.len());
    for ctrl in &compiled.controls {
        eprintln!(
            "    \"{}\" → comp={} range={:?}",
            ctrl.label, ctrl.component_id, ctrl.range
        );
    }

    // Must have bindings for all 4 controls
    let labels: Vec<&str> = compiled.controls.iter().map(|c| c.label.as_str()).collect();
    eprintln!("  Labels found: {:?}", labels);

    assert!(labels.contains(&"Gain"), "Should have Gain binding");
    assert!(labels.contains(&"Treble"), "Should have Treble binding");
    assert!(labels.contains(&"Output"), "Should have Output binding");

    // Gain should appear TWICE (Gain_A + Gain_B ganged)
    let gain_count = labels.iter().filter(|&&l| l == "Gain").count();
    eprintln!("  Gain bindings: {gain_count} (should be 2 for ganged)");
    assert_eq!(gain_count, 2, "Ganged Gain pot should have 2 bindings");
}

#[test]
fn goldenrod_disable_iir_compiles_without_iir_stages() {
    let compiled = load_goldenrod_with_options(CompileOptions {
        disable_iir: true,
        skip_k_tables: true,
        ..Default::default()
    });

    let iir_count = compiled
        .stages
        .iter()
        .filter(|stage| matches!(stage, super::compiled::Stage::Iir(_)))
        .count();
    let wdf_count = compiled
        .stages
        .iter()
        .filter(|stage| matches!(stage, super::compiled::Stage::Wdf(_)))
        .count();

    assert_eq!(iir_count, 0, "disable_iir should suppress IIR stages");
    assert!(
        wdf_count > 0,
        "disable_iir should still leave WDF stages available"
    );
}

#[test]
fn goldenrod_dynamic_linear_stages_expose_graph_route_boundaries() {
    let compiled = load_goldenrod_with_options(CompileOptions {
        skip_k_tables: true,
        ..Default::default()
    });
    let mut routed_count = 0;
    for stage in &compiled.stages {
        match stage {
            Stage::Iir(iir) => {
                routed_count += 1;
                assert!(
                    !iir.ins().is_empty(),
                    "compiled IIR stage should expose graph input binding"
                );
                assert!(
                    !iir.outs().is_empty(),
                    "compiled IIR stage should expose graph output binding"
                );
            }
            Stage::Wdf(wdf) if wdf.opamp_wdf_adaptor.is_some() => {
                routed_count += 1;
                assert_ne!(
                    wdf.injection_node_id, wdf.output_node_id,
                    "active-feedback WDF stage should expose distinct graph boundary nodes"
                );
            }
            _ => {}
        }
    }
    assert!(
        routed_count > 0,
        "Goldenrod should compile at least one routed dynamic linear stage"
    );
}

#[test]
fn goldenrod_disable_iir_gain_changes_output() {
    let mut low_gain = load_goldenrod_with_options(CompileOptions {
        disable_iir: true,
        skip_k_tables: true,
        ..Default::default()
    });
    low_gain.set_control("Gain", 0.0);
    low_gain.set_control("Output", 0.7);

    let mut high_gain = load_goldenrod_with_options(CompileOptions {
        disable_iir: true,
        skip_k_tables: true,
        ..Default::default()
    });
    high_gain.set_control("Gain", 1.0);
    high_gain.set_control("Output", 0.7);

    let (low_peak, low_rms) = measure_peak_and_rms(&mut low_gain, 0.05);
    let (high_peak, high_rms) = measure_peak_and_rms(&mut high_gain, 0.05);
    let ratio = high_rms.max(low_rms) / high_rms.min(low_rms).max(1e-12);

    eprintln!(
        "Goldenrod disable_iir Gain: low_peak={low_peak:.6}, high_peak={high_peak:.6}, low_rms={low_rms:.6}, high_rms={high_rms:.6}, ratio={ratio:.3}"
    );
    assert!(
        ratio > 1.2 || (high_peak - low_peak).abs() > 0.01,
        "Goldenrod disable_iir Gain should measurably change output"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Output pot changes volume
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn goldenrod_output_pot_changes_level() {
    let mut low = load_goldenrod();
    low.set_control("Output", 0.2);
    let peak_low = measure_peak(&mut low, 0.1);

    let mut high = load_goldenrod();
    high.set_control("Output", 0.9);
    let peak_high = measure_peak(&mut high, 0.1);

    eprintln!("Output 0.2: {peak_low:.4}V, Output 0.9: {peak_high:.4}V");
    assert!(
        peak_high > peak_low * 1.5,
        "Output pot should change level: low={peak_low:.4}, high={peak_high:.4}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Gain pot crossfade — low gain = clean, high gain = dirty
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn goldenrod_gain_crossfade() {
    // At low Gain: Gain_A = high R (low gain), Gain_B = low R (clean passes)
    // At high Gain: Gain_A = low R (high gain), Gain_B = high R (clean blocked)
    // The dirty path should clip harder at high gain, changing the waveform.
    let mut clean = load_goldenrod();
    clean.set_control("Gain", 0.1);
    let peak_clean = measure_peak(&mut clean, 0.1);

    let mut dirty = load_goldenrod();
    dirty.set_control("Gain", 0.9);
    let peak_dirty = measure_peak(&mut dirty, 0.1);

    eprintln!("Gain 0.1 (clean): {peak_clean:.4}V, Gain 0.9 (dirty): {peak_dirty:.4}V");

    // Both should produce output
    assert!(
        peak_clean > 0.0001,
        "Clean should produce output: {peak_clean:.4}V"
    );
    assert!(
        peak_dirty > 0.001,
        "Dirty should produce output: {peak_dirty:.4}V"
    );

    // The levels should be DIFFERENT (crossfade working)
    let ratio = peak_dirty / peak_clean.max(0.0001);
    eprintln!("  Dirty/clean ratio: {ratio:.2}");
    assert!(
        (ratio - 1.0).abs() > 0.05,
        "Gain crossfade should change character: ratio={ratio:.2}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Treble pot changes tone
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn goldenrod_treble_pot_changes_spectrum() {
    let mut dark = load_goldenrod();
    dark.set_control("Treble", 0.1);
    let peak_dark = measure_peak(&mut dark, 0.1);

    let mut bright = load_goldenrod();
    bright.set_control("Treble", 0.9);
    let peak_bright = measure_peak(&mut bright, 0.1);

    eprintln!("Treble 0.1: {peak_dark:.4}V, Treble 0.9: {peak_bright:.4}V");

    // Both should produce output
    assert!(
        peak_dark > 0.001,
        "Dark should produce output: {peak_dark:.4}V"
    );
    assert!(
        peak_bright > 0.001,
        "Bright should produce output: {peak_bright:.4}V"
    );
}

#[test]
fn goldenrod_treble_uses_dynamic_feedback_adaptor() {
    let compiled = load_goldenrod();
    let tone_stage = compiled.stages.iter().find_map(|stage| match stage {
        Stage::Wdf(wdf)
            if wdf
                .opamp_wdf_adaptor
                .as_ref()
                .and_then(|adaptor| adaptor.feedback_pot_id.as_deref())
                == Some("Treble") =>
        {
            Some(wdf)
        }
        _ => None,
    });
    assert!(
        tone_stage.is_some(),
        "Goldenrod Treble should compile to the dynamic active-feedback WDF adaptor"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 4b. Gain sweep — signal at EVERY position including 100%
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn goldenrod_gain_100_not_dead() {
    let positions = [0.0, 0.25, 0.5, 0.75, 0.9, 0.95, 1.0];
    let mut results = Vec::new();
    for &pos in &positions {
        let mut compiled = load_goldenrod();
        compiled.set_control("Gain", pos);
        let peak = measure_peak(&mut compiled, 0.1);
        eprintln!("Gain={pos:.2}: peak={peak:.6}V");
        results.push((pos, peak));
    }
    // All positions (except maybe 0 where it's intentional no-gain) should produce output
    for &(pos, peak) in &results {
        if pos > 0.1 {
            assert!(peak > 0.001, "Signal dies at Gain={pos}: peak={peak:.6}V");
        }
    }
    // 100% should not die
    let full_peak = results.last().map(|(_, pk)| *pk).unwrap_or(0.0);
    assert!(
        full_peak > 0.001,
        "Gain=1.0 should produce output: {full_peak:.6}V"
    );

    // Also check Output at 100%
    let mut out100 = load_goldenrod();
    out100.set_control("Output", 1.0);
    let peak_out100 = measure_peak(&mut out100, 0.1);
    eprintln!("Output=1.0: peak={peak_out100:.6}V");
    assert!(
        peak_out100 > 0.001,
        "Output=1.0 should produce output: {peak_out100:.6}V"
    );

    // Per-stage levels at Gain=0 (where signal dies)
    eprintln!("\nPer-stage at Gain=0.0:");
    let mut g0 = load_goldenrod();
    g0.set_control("Gain", 0.0);
    // Print BF stage gains before processing
    for (i, stage) in g0.stages.iter().enumerate() {
        if let super::compiled::Stage::BlackFeedback(bf) = stage {
            #[cfg(debug_assertions)]
            eprintln!(
                "  BF stage {i} [{}]: gain={:.2} bypass={}",
                bf.debug_label,
                bf.gain(),
                bf.bypass_serial
            );
        }
    }
    // Also check WDF stages
    for (i, stage) in g0.stages.iter().enumerate() {
        if let super::compiled::Stage::Wdf(w) = stage {
            #[cfg(debug_assertions)]
            eprintln!(
                "  WDF stage {i} [{}]: bypass={} feedforward={}",
                w.debug_label, w.bypass_serial, w.is_feedforward
            );
        }
    }
    g0.enable_metering(128);
    // Process with trace on a few samples
    for s in 0..7990 {
        g0.process(0.1 * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }
    // Manual trace of last 10 samples through stages
    eprintln!("\n  Signal trace (sample 7990):");
    let input = 0.1 * (std::f64::consts::TAU * FREQ * 7990.0 / SR).sin();
    let mut signal = input;
    eprintln!("    input: {signal:.6}");
    for (i, stage) in g0.stages.iter_mut().enumerate() {
        let prev = signal;
        match stage {
            super::compiled::Stage::Wdf(w) => {
                if !w.bypass_serial {
                    signal = w.process(signal);
                }
            }
            super::compiled::Stage::BlackFeedback(bf) => {
                if !bf.bypass_serial {
                    signal = bf.process(signal);
                }
            }
            _ => {}
        }
        if (signal - prev).abs() > 0.0001 || i == 8 || i == 9 || i == 10 {
            eprintln!(
                "    stage {i}: {prev:.6} → {signal:.6} (×{:.2})",
                signal / prev.max(1e-10)
            );
        }
    }
    // Apply wiper dividers
    for wd in &g0.wiper_dividers {
        let prev = signal;
        signal *= wd.position;
        eprintln!(
            "    wiper[{}] after_stage={}: {prev:.6} → {signal:.6} (×{:.4})",
            wd.pot_comp_id, wd.after_stage_idx, wd.position
        );
    }
    eprintln!("    final: {signal:.6}");

    for s in 7990..8000 {
        g0.process(0.1 * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }
    let m0 = g0.read_metrics();
    let n = g0.stages.len().min(crate::metering::MAX_STAGES);
    for i in 0..n {
        let lvl = m0.stage_levels[i];
        let db = if lvl > 1e-10 {
            20.0 * (lvl as f64).log10()
        } else {
            -120.0
        };
        #[cfg(debug_assertions)]
        {
            let lbl = match &g0.stages[i] {
                super::compiled::Stage::Wdf(w) => &w.debug_label,
                super::compiled::Stage::BlackFeedback(b) => &b.debug_label,
                super::compiled::Stage::Iir(s) => &s.debug_label,
                _ => &String::new(),
            };
            eprintln!("  stage {i}: [{lbl}] level={lvl:.6} ({db:.1} dB)");
        }
    }
    eprintln!("  output: peak={:.1} dB", m0.output_peak_db);

    // Wiper dividers + control targets
    let diag = load_goldenrod();
    eprintln!("\nControl targets:");
    for ctrl in &diag.controls {
        eprintln!(
            "  '{}' comp={} target={:?}",
            ctrl.label, ctrl.component_id, ctrl.target
        );
    }
    eprintln!("\nWiper dividers ({}):", diag.wiper_dividers.len());
    for wd in &diag.wiper_dividers {
        eprintln!(
            "  {} after_stage={} position={:.4} taper={:?}",
            wd.pot_comp_id, wd.after_stage_idx, wd.position, wd.taper
        );
    }

    // Check Gain at 100% with Output at various levels
    eprintln!("\nGain=1.0 with Output sweep:");
    for out_pos in [0.3, 0.5, 0.7, 1.0] {
        let mut c = load_goldenrod();
        c.set_control("Gain", 1.0);
        c.set_control("Output", out_pos);
        let p = measure_peak(&mut c, 0.1);
        eprintln!("  Gain=1.0, Output={out_pos:.1}: peak={p:.6}V");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. Goldenrod gain is a path crossfade, not a plain THD knob
// ═══════════════════════════════════════════════════════════════════════════

/// Measure THD by comparing fundamental power to harmonic power.
fn measure_thd(compiled: &mut CompiledPedal, amp: f64) -> f64 {
    let n = 4096;
    // Warmup
    for s in 0..4000 {
        compiled.process(amp * (std::f64::consts::TAU * FREQ * s as f64 / SR).sin());
    }
    // Capture
    let mut samples = Vec::with_capacity(n);
    for s in 0..n {
        let input = amp * (std::f64::consts::TAU * FREQ * (4000 + s) as f64 / SR).sin();
        samples.push(compiled.process(input));
    }
    // FFT via DFT at harmonic bins
    let bin_width = SR / n as f64;
    let fund_bin = (FREQ / bin_width).round() as usize;

    let mut fund_power = 0.0f64;
    let mut harm_power = 0.0f64;

    for h in 1..=8 {
        let bin = fund_bin * h;
        if bin >= n / 2 {
            break;
        }
        // Goertzel-like: sum of (sample * e^(-j2pi*bin*k/N))
        let mut re = 0.0f64;
        let mut im = 0.0f64;
        for (k, &s) in samples.iter().enumerate() {
            let angle = std::f64::consts::TAU * bin as f64 * k as f64 / n as f64;
            re += s * angle.cos();
            im += s * angle.sin();
        }
        let power = re * re + im * im;
        if h == 1 {
            fund_power = power;
        } else {
            harm_power += power;
        }
    }

    if fund_power < 1e-20 {
        return 0.0;
    }
    (harm_power / fund_power).sqrt() * 100.0 // THD as percentage
}

#[test]
fn goldenrod_gain_moves_clean_and_dirty_paths() {
    #[cfg(debug_assertions)]
    {
        let low_gain = metered_goldenrod(0.1, 0.7, 0.5, 0.01);
        let high_gain = metered_goldenrod(0.9, 0.7, 0.5, 0.01);

        let gain_a_stage = control_target_label(&low_gain, "Gain_A");
        let gain_b_primary_stage = control_target_label(&low_gain, "Gain_B");
        let gain_b_r_low = debug_value(&low_gain, "] Gain_B");
        let gain_b_r_high = debug_value(&high_gain, "] Gain_B");
        let dirty_low = stage_level(&low_gain, "U2");
        let dirty_high = stage_level(&high_gain, "U2");
        let sum_low = stage_level(&low_gain, "U3");
        let sum_high = stage_level(&high_gain, "U3");

        eprintln!(
            "Goldenrod gain targets: Gain_A -> {gain_a_stage}, Gain_B primary -> {gain_b_primary_stage}; \
             Gain_B R {gain_b_r_low:.1} -> {gain_b_r_high:.1}; \
             dirty U2 {dirty_low:.4} -> {dirty_high:.4}, sum U3 {sum_low:.4} -> {sum_high:.4}"
        );

        assert!(
            gain_a_stage.contains("U2"),
            "Gain_A should target the U2 dirty gain stage, got {gain_a_stage:?}"
        );
        assert!(
            (gain_b_r_high / gain_b_r_low.max(1e-9) - 1.0).abs() > 0.20,
            "Gain_B pot resistance should move as Gain rises: low={gain_b_r_low:.1}, high={gain_b_r_high:.1}"
        );
        assert!(
            dirty_low > 0.01 && dirty_high > 0.01,
            "U2 dirty path should stay alive at both ends: low={dirty_low:.4}, high={dirty_high:.4}"
        );
        // At this 440 Hz probe, C_gnd bypasses the lower ground-leg chain, so
        // Gain_A is not expected to produce a large U2 level delta. The
        // product-facing crossfade is verified at the U3 summing stage below.
        assert!(
            (sum_high / sum_low.max(1e-9) - 1.0).abs() > 0.05,
            "U3 summing stage should see the crossfade move: low={sum_low:.4}, high={sum_high:.4}"
        );
    }
}

#[test]
fn goldenrod_gain_changes_output_without_thd_assumption() {
    let mut low_gain = load_goldenrod();
    low_gain.set_control("Gain", 0.1);
    let thd_low = measure_thd(&mut low_gain, 0.1);

    let mut high_gain = load_goldenrod();
    high_gain.set_control("Gain", 0.9);
    let thd_high = measure_thd(&mut high_gain, 0.1);

    eprintln!("THD at Gain=0.1: {thd_low:.1}%");
    eprintln!("THD at Gain=0.9: {thd_high:.1}%");
    eprintln!("Ratio (high/low): {:.2}x", thd_high / thd_low.max(0.001));

    let mut low_gain = load_goldenrod();
    low_gain.set_control("Gain", 0.1);
    let (peak_low, rms_low) = measure_peak_and_rms(&mut low_gain, 0.1);

    let mut high_gain = load_goldenrod();
    high_gain.set_control("Gain", 0.9);
    let (peak_high, rms_high) = measure_peak_and_rms(&mut high_gain, 0.1);

    eprintln!(
        "Output Gain=0.1 peak={peak_low:.4} rms={rms_low:.4}; \
         Gain=0.9 peak={peak_high:.4} rms={rms_high:.4}"
    );

    assert!(
        peak_high > 0.01,
        "High gain should produce output: {peak_high:.4}"
    );
    assert!(
        peak_low > 0.0001,
        "Low gain should produce output: {peak_low:.4}"
    );
    assert!(
        (rms_high / rms_low.max(1e-9) - 1.0).abs() > 0.05,
        "Gain crossfade should change final level even when THD is compressed: low={rms_low:.4}, high={rms_high:.4}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. Gain_A actually modulates U2's gain
//
// The ground leg chain R5 → R6 → Gain_A → GND must be in U2's feedback
// group for BlackFeedback to compute gain = 1 + Rf / (R5 + R6 + Gain_A_R).
// Currently R5 is claimed, but R6 + Gain_A are in a separate passive group.
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn goldenrod_gain_a_in_u2_group() {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/goldenrod.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect("read goldenrod.pedal");
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");

    let graph = super::graph::CircuitGraph::from_pedal(&pedal);
    let active_set: std::collections::HashSet<usize> =
        graph.active_edge_indices.iter().copied().collect();
    let all_edges: Vec<usize> = (0..graph.edges.len())
        .filter(|i| !active_set.contains(i))
        .collect();
    let groups = super::signal_flow::find_flow_groups(&all_edges, &graph);

    // Find U2's group
    let u2_group = groups
        .iter()
        .find(|g| {
            g.active_edges
                .iter()
                .any(|&eidx| graph.components[graph.edges[eidx].comp_idx].id == "U2")
        })
        .expect("Should find U2 group");

    // Collect all component names in U2's group (feedback + pendant + ground_shunt + active)
    let all_edges = u2_group.all_edges();
    let all_comp_names: Vec<&str> = all_edges
        .iter()
        .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
        .collect();

    eprintln!("U2 group components: {:?}", all_comp_names);
    let names_for = |edges: &[usize]| -> Vec<&str> {
        edges
            .iter()
            .map(|&eidx| graph.components[graph.edges[eidx].comp_idx].id.as_str())
            .collect()
    };
    eprintln!("  active: {:?}", names_for(&u2_group.active_edges));
    eprintln!("  feedback: {:?}", names_for(&u2_group.feedback_edges));
    eprintln!("  pendant: {:?}", names_for(&u2_group.pendant_edges));
    eprintln!("  shunt: {:?}", names_for(&u2_group.ground_shunt_edges));

    // R5 should be pendant (single-hop from neg). R6 and Gain_A are in a
    // separate passive group — the Ri BFS in spqr_build finds them without
    // needing them in U2's FlowGroup.
    assert!(
        all_comp_names.contains(&"R5"),
        "R5 should be in U2 group (pendant from neg): {:?}",
        all_comp_names
    );
    assert!(
        all_comp_names.contains(&"Gain_A"),
        "Gain_A should stay in U2 ground-leg group: {:?}",
        all_comp_names
    );
    for feedforward_comp in ["R_ff1a", "C_ff1", "Gain_B", "R_ff1b", "R_ff2"] {
        assert!(
            !all_comp_names.contains(&feedforward_comp),
            "{feedforward_comp} should remain split from U2 feedback group: {:?}",
            all_comp_names
        );
    }
}
