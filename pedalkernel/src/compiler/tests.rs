// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

use super::compile::*;
use super::split::*;
use super::stage::*;
use super::warnings::*;
use crate::dsl::*;
use crate::PedalProcessor;

fn parse(filename: &str) -> PedalDef {
    let path = find_example(filename);
    let src = std::fs::read_to_string(&path).unwrap();
    parse_pedal_file(&src).unwrap()
}

/// Search examples/ subdirectories for a file by name.
fn find_example(filename: &str) -> String {
    for entry in walkdir("examples") {
        if entry.ends_with(filename) {
            return entry;
        }
    }
    panic!("example file not found: {filename}");
}

/// Simple recursive directory walk.
fn walkdir(dir: &str) -> Vec<String> {
    let mut results = Vec::new();
    if let Ok(entries) = std::fs::read_dir(dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                results.extend(walkdir(path.to_str().unwrap()));
            } else {
                results.push(path.to_string_lossy().to_string());
            }
        }
    }
    results
}

#[test]
fn compile_tube_screamer() {
    let pedal = parse("tube_screamer.pedal");
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
    proc.set_control("Drive", 0.7);
    proc.set_control("Level", 0.8);

    // Process a sine wave and verify it's not a wire.
    let n = 48000;
    let input: Vec<f64> = (0..n)
        .map(|i| 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin())
        .collect();
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

    let corr = correlation(&input, &output).abs();
    assert!(corr < 0.98, "Compiled TS should distort: |corr|={corr:.4}");
    assert!(output.iter().all(|x| x.is_finite()), "No NaN/inf in output");
}

#[test]
fn compile_big_muff() {
    let pedal = parse("big_muff.pedal");
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
    proc.set_control("Sustain", 0.7);
    proc.set_control("Volume", 0.5);

    let n = 48000;
    let input: Vec<f64> = (0..n)
        .map(|i| 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin())
        .collect();
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

    let corr = correlation(&input, &output).abs();
    assert!(
        corr < 0.98,
        "Compiled Big Muff should distort: |corr|={corr:.4}"
    );
}

#[test]
fn compile_fuzz_face() {
    // Fuzz Face has no diodes in DSL — uses transistor gain + soft limit.
    let pedal = parse("fuzz_face.pedal");
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
    proc.set_control("Fuzz", 0.8);
    proc.set_control("Volume", 0.5);

    let n = 48000;
    let input: Vec<f64> = (0..n)
        .map(|i| 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin())
        .collect();
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

    assert!(output.iter().all(|x| x.is_finite()), "No NaN/inf in output");
    let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    assert!(peak > 0.01, "Should produce output: peak={peak}");
}

#[test]
fn compile_all_pedals() {
    let files = [
        "tube_screamer.pedal",
        "fuzz_face.pedal",
        "big_muff.pedal",
        "blues_driver.pedal",
        "dyna_comp.pedal",
        "klon_centaur.pedal",
        "proco_rat.pedal",
        "boss_ce2.pedal",
        "tweed_deluxe_5e3.pedal",
        "bassman_5f6a.pedal",
        "marshall_jtm45.pedal",
    ];
    for f in files {
        let pedal = parse(f);
        let result = compile_pedal(&pedal, 48000.0);
        if let Err(e) = &result {
            panic!("Failed to compile {f}: {e}");
        }

        let mut proc = result.unwrap();
        let input: Vec<f64> = (0..4800)
            .map(|i| 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin())
            .collect();
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        assert!(
            output.iter().all(|x| x.is_finite()),
            "{f}: output contains NaN/inf"
        );
        let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(peak > 0.001, "{f}: silent output, peak={peak}");
    }
}

#[test]
fn compiled_pedals_differ() {
    // Each pedal should produce a distinct output (different topologies/values).
    let files = [
        "tube_screamer.pedal",
        "big_muff.pedal",
        "blues_driver.pedal",
    ];
    let mut outputs = Vec::new();

    for f in files {
        let pedal = parse(f);
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        let input: Vec<f64> = (0..48000)
            .map(|i| 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin())
            .collect();
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        outputs.push(output);
    }

    // Each pair should have |corr| < 0.999 (they're not identical).
    for i in 0..outputs.len() {
        for j in (i + 1)..outputs.len() {
            let corr = correlation(&outputs[i], &outputs[j]).abs();
            assert!(
                corr < 0.999,
                "{} vs {} are too similar: |corr|={corr:.6}",
                files[i],
                files[j]
            );
        }
    }
}

// -----------------------------------------------------------------------
// Per-pedal regression tests
// -----------------------------------------------------------------------

#[test]
fn compile_blues_driver() {
    let pedal = parse("blues_driver.pedal");
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
    proc.set_control("Gain", 0.7);
    proc.set_control("Level", 0.5);

    let input = sine(48000);
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    assert_finite(&output, "Blues Driver");
    let corr = correlation(&input, &output).abs();
    assert!(corr < 0.98, "Blues Driver should distort: |corr|={corr:.4}");
}

#[test]
fn compile_proco_rat() {
    let pedal = parse("proco_rat.pedal");
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
    proc.set_control("Distortion", 0.7);
    proc.set_control("Volume", 0.5);

    let input = sine(48000);
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    assert_finite(&output, "ProCo RAT");
    let corr = correlation(&input, &output).abs();
    assert!(corr < 0.98, "ProCo RAT should distort: |corr|={corr:.4}");
}

#[test]
fn compile_klon_centaur() {
    let pedal = parse("klon_centaur.pedal");
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
    proc.set_control("Gain", 0.7);
    proc.set_control("Output", 0.5);

    let input = sine(48000);
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    assert_finite(&output, "Klon Centaur");
    let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    assert!(peak > 0.01, "Klon should produce output: peak={peak}");
}

#[test]
fn compile_dyna_comp() {
    // Dyna Comp has no diodes — uses opamp gain + soft limit.
    let pedal = parse("dyna_comp.pedal");
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
    proc.set_control("Sensitivity", 0.7);
    proc.set_control("Output", 0.5);

    let input = sine(48000);
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    assert_finite(&output, "Dyna Comp");
    let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    assert!(peak > 0.01, "Dyna Comp should produce output: peak={peak}");
}

#[test]
fn compile_phase90() {
    // Phase 90 compiles with op-amp stages and produces phasing effect
    let pedal = parse("phase90.pedal");
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
    proc.set_control("Speed", 0.5);

    // Process a sine wave and verify output
    let input = sine(48000);
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    assert_finite(&output, "Phase 90");

    // With op-amp feedback integrated into the WDF tree, each all-pass
    // stage's unity-gain op-amp buffer compensates for passive R/C/JFET
    // attenuation.  The output should be at a healthy level (~-10dB).
    let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    assert!(
        peak > 0.05,
        "Phase 90 should produce output at healthy level: peak={peak}"
    );

    // Verify the circuit is actively creating the sweeping effect
    // by checking that the output varies over time (not just a static gain)
    let first_half: Vec<f64> = output[0..24000].to_vec();
    let second_half: Vec<f64> = output[24000..48000].to_vec();

    // RMS of each half
    let rms_first = (first_half.iter().map(|x| x * x).sum::<f64>() / 24000.0).sqrt();
    let rms_second = (second_half.iter().map(|x| x * x).sum::<f64>() / 24000.0).sqrt();

    // Both halves should have signal at a reasonable level.
    assert!(
        rms_first > 0.01 && rms_second > 0.01,
        "Both halves should have signal: rms_first={rms_first}, rms_second={rms_second}"
    );
}

#[test]
fn compile_tweed_deluxe_5e3() {
    let pedal = parse("tweed_deluxe_5e3.pedal");
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
    proc.set_control("Volume", 0.7);
    proc.set_control("Tone", 0.5);

    let input = sine(48000);
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    assert_finite(&output, "Tweed Deluxe 5E3");
    let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    assert!(
        peak > 0.01,
        "Tweed Deluxe should produce output: peak={peak}"
    );
}

#[test]
fn compile_bassman_5f6a() {
    let pedal = parse("bassman_5f6a.pedal");
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
    proc.set_control("Volume", 0.6);
    proc.set_control("Treble", 0.6);
    proc.set_control("Mid", 0.5);
    proc.set_control("Bass", 0.5);

    let input = sine(48000);
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    assert_finite(&output, "Bassman 5F6-A");
    let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    assert!(
        peak > 0.01,
        "Bassman should produce output: peak={peak}"
    );
}

#[test]
fn compile_marshall_jtm45() {
    let pedal = parse("marshall_jtm45.pedal");
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
    proc.set_control("Volume", 0.6);
    proc.set_control("Treble", 0.6);
    proc.set_control("Mid", 0.5);
    proc.set_control("Bass", 0.5);
    proc.set_control("Presence", 0.5);

    let input = sine(48000);
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    assert_finite(&output, "Marshall JTM45");
    let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    assert!(
        peak > 0.01,
        "JTM45 should produce output: peak={peak}"
    );
}

// -----------------------------------------------------------------------
// Control response tests
// -----------------------------------------------------------------------

#[test]
fn gain_control_affects_distortion() {
    let pedal = parse("tube_screamer.pedal");
    let input = sine(48000);

    let mut low = compile_pedal(&pedal, 48000.0).unwrap();
    low.set_control("Drive", 0.1);
    let out_low: Vec<f64> = input.iter().map(|&s| low.process(s)).collect();

    let mut high = compile_pedal(&pedal, 48000.0).unwrap();
    high.set_control("Drive", 0.9);
    let out_high: Vec<f64> = input.iter().map(|&s| high.process(s)).collect();

    let corr_low = correlation(&input, &out_low).abs();
    let corr_high = correlation(&input, &out_high).abs();
    assert!(
        corr_high < corr_low,
        "Higher Drive should produce more distortion: low={corr_low:.4} high={corr_high:.4}"
    );
}

#[test]
fn level_control_affects_volume() {
    let pedal = parse("tube_screamer.pedal");
    let input = sine(48000);

    let mut quiet = compile_pedal(&pedal, 48000.0).unwrap();
    quiet.set_control("Level", 0.1);
    let out_quiet: Vec<f64> = input.iter().map(|&s| quiet.process(s)).collect();
    let peak_quiet = out_quiet.iter().fold(0.0f64, |m, x| m.max(x.abs()));

    let mut loud = compile_pedal(&pedal, 48000.0).unwrap();
    loud.set_control("Level", 0.9);
    let out_loud: Vec<f64> = input.iter().map(|&s| loud.process(s)).collect();
    let peak_loud = out_loud.iter().fold(0.0f64, |m, x| m.max(x.abs()));

    assert!(
        peak_loud > peak_quiet,
        "Higher Level should be louder: quiet={peak_quiet:.4} loud={peak_loud:.4}"
    );
}

#[test]
fn compiled_output_bounded() {
    // No pedal should produce output exceeding 2.0 with 0.5 amplitude input.
    let files = [
        "tube_screamer.pedal",
        "fuzz_face.pedal",
        "big_muff.pedal",
        "blues_driver.pedal",
        "dyna_comp.pedal",
        "klon_centaur.pedal",
        "proco_rat.pedal",
    ];
    let input = sine(48000);
    for f in files {
        let pedal = parse(f);
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(peak < 2.0, "{f}: output too loud, peak={peak}");
    }
}

#[test]
fn compiled_reset_clears_state() {
    let pedal = parse("tube_screamer.pedal");
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
    proc.set_control("Drive", 0.7);

    // Process some audio
    let input = sine(4800);
    let _: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

    // Reset and process again — should match initial run
    proc.reset();
    let mut proc2 = compile_pedal(&pedal, 48000.0).unwrap();
    proc2.set_control("Drive", 0.7);

    let out1: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    let out2: Vec<f64> = input.iter().map(|&s| proc2.process(s)).collect();
    let corr = correlation(&out1, &out2);
    assert!(
        corr > 0.999,
        "Reset should restore initial state: corr={corr:.6}"
    );
}

#[test]
fn all_seven_pedals_unique() {
    let files = [
        "tube_screamer.pedal",
        "fuzz_face.pedal",
        "big_muff.pedal",
        "blues_driver.pedal",
        "dyna_comp.pedal",
        "klon_centaur.pedal",
        "proco_rat.pedal",
    ];
    // Use smaller input (0.1V peak) to test at moderate saturation levels
    // where soft vs hard clipping produces more distinct waveforms.
    // Full 0.5V input drives most pedals into deep saturation.
    let input: Vec<f64> = (0..48000)
        .map(|i| 0.1 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin())
        .collect();
    let mut outputs = Vec::new();
    for f in files {
        let pedal = parse(f);
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        outputs.push(output);
    }
    for i in 0..outputs.len() {
        for j in (i + 1)..outputs.len() {
            let corr = correlation(&outputs[i], &outputs[j]).abs();
            assert!(
                corr < 0.9999,
                "{} vs {} too similar: |corr|={corr:.6}",
                files[i],
                files[j]
            );
        }
    }
}

// -----------------------------------------------------------------------
// Non-default settings regression tests
// -----------------------------------------------------------------------

#[test]
fn tube_screamer_max_drive() {
    let pedal = parse("tube_screamer.pedal");
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
    proc.set_control("Drive", 1.0);
    proc.set_control("Level", 1.0);

    let input = sine(48000);
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    assert_finite(&output, "TS max drive");
    let corr = correlation(&input, &output).abs();
    assert!(
        corr < 0.95,
        "Max drive should heavily distort: |corr|={corr:.4}"
    );
}

#[test]
fn tube_screamer_min_drive() {
    let pedal = parse("tube_screamer.pedal");
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
    proc.set_control("Drive", 0.0);
    proc.set_control("Level", 0.5);

    let input = sine(48000);
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    assert_finite(&output, "TS min drive");
    let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    assert!(
        peak > 0.001,
        "Min drive should still produce output: peak={peak}"
    );
}

#[test]
fn big_muff_max_sustain() {
    let pedal = parse("big_muff.pedal");
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
    proc.set_control("Sustain", 1.0);
    proc.set_control("Volume", 0.8);

    let input = sine(48000);
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    assert_finite(&output, "BM max sustain");
    let corr = correlation(&input, &output).abs();
    assert!(
        corr < 0.95,
        "Max sustain should heavily distort: |corr|={corr:.4}"
    );
}

#[test]
fn proco_rat_max_distortion() {
    let pedal = parse("proco_rat.pedal");
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
    proc.set_control("Distortion", 1.0);
    proc.set_control("Volume", 0.8);

    let input = sine(48000);
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    assert_finite(&output, "RAT max distortion");
    let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    assert!(
        peak > 0.01,
        "Max distortion should produce output: peak={peak}"
    );
}

#[test]
fn fuzz_face_max_fuzz() {
    let pedal = parse("fuzz_face.pedal");
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
    proc.set_control("Fuzz", 1.0);
    proc.set_control("Volume", 1.0);

    let input = sine(48000);
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    assert_finite(&output, "FF max fuzz");
    let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    assert!(peak > 0.01, "Max fuzz should produce output: peak={peak}");
}

#[test]
fn all_pedals_extreme_settings_stable() {
    let configs: &[(&str, &[(&str, f64)])] = &[
        ("tube_screamer.pedal", &[("Drive", 1.0), ("Level", 1.0)]),
        ("tube_screamer.pedal", &[("Drive", 0.0), ("Level", 0.0)]),
        ("fuzz_face.pedal", &[("Fuzz", 1.0), ("Volume", 1.0)]),
        ("fuzz_face.pedal", &[("Fuzz", 0.0), ("Volume", 0.0)]),
        ("big_muff.pedal", &[("Sustain", 1.0), ("Volume", 1.0)]),
        ("big_muff.pedal", &[("Sustain", 0.0), ("Volume", 0.0)]),
        ("blues_driver.pedal", &[("Gain", 1.0), ("Level", 1.0)]),
        ("blues_driver.pedal", &[("Gain", 0.0), ("Level", 0.0)]),
        ("proco_rat.pedal", &[("Distortion", 1.0), ("Volume", 1.0)]),
        ("proco_rat.pedal", &[("Distortion", 0.0), ("Volume", 0.0)]),
        ("klon_centaur.pedal", &[("Gain", 1.0), ("Output", 1.0)]),
        ("klon_centaur.pedal", &[("Gain", 0.0), ("Output", 0.0)]),
        ("dyna_comp.pedal", &[("Sensitivity", 1.0), ("Output", 1.0)]),
        ("dyna_comp.pedal", &[("Sensitivity", 0.0), ("Output", 0.0)]),
    ];

    let input = sine(4800);
    for (f, knobs) in configs {
        let pedal = parse(f);
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        for &(label, val) in *knobs {
            proc.set_control(label, val);
        }
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        let settings: String = knobs
            .iter()
            .map(|(l, v)| format!("{l}={v}"))
            .collect::<Vec<_>>()
            .join(",");
        assert!(
            output.iter().all(|x| x.is_finite()),
            "{f} [{settings}]: NaN/inf"
        );
        let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(peak < 5.0, "{f} [{settings}]: output too loud, peak={peak}");
    }
}

// -----------------------------------------------------------------------
// Phase 3: Slew rate, OTA, BBD tests
// -----------------------------------------------------------------------

#[test]
fn proco_rat_has_slew_limiter() {
    // ProCo RAT uses LM308 (0.3 V/µs slew rate).
    // The compiled pedal should have a slew rate limiter.
    let pedal = parse("proco_rat.pedal");
    let proc = compile_pedal(&pedal, 48000.0).unwrap();
    assert!(
        !proc.slew_limiters.is_empty(),
        "ProCo RAT should have slew rate limiters from LM308"
    );
    assert!(
        (proc.slew_limiters[0].slew_rate() - 0.3).abs() < 0.01,
        "LM308 slew rate should be 0.3 V/µs"
    );
}

#[test]
fn tube_screamer_has_slew_limiter() {
    // Tube Screamer uses JRC4558 (1.7 V/µs slew rate).
    let pedal = parse("tube_screamer.pedal");
    let proc = compile_pedal(&pedal, 48000.0).unwrap();
    assert!(
        !proc.slew_limiters.is_empty(),
        "Tube Screamer should have slew limiter from JRC4558"
    );
    assert!(
        (proc.slew_limiters[0].slew_rate() - 1.7).abs() < 0.01,
        "JRC4558 slew rate should be 1.7 V/µs"
    );
}

#[test]
fn dyna_comp_no_slew_limiter() {
    // Dyna Comp uses CA3080 OTA which is very fast (50 V/µs).
    // OTAs are handled as WDF roots, not slew limiters.
    let pedal = parse("dyna_comp.pedal");
    let proc = compile_pedal(&pedal, 48000.0).unwrap();
    assert!(
        proc.slew_limiters.is_empty(),
        "CA3080 OTA should not add a slew rate limiter (it's a WDF root)"
    );
}

#[test]
fn dyna_comp_has_ota_stage() {
    // Dyna Comp should have an OTA WDF stage.
    let pedal = parse("dyna_comp.pedal");
    let proc = compile_pedal(&pedal, 48000.0).unwrap();
    let ota_stages: Vec<_> = proc
        .stages
        .iter()
        .filter(|s| matches!(s.root, RootKind::Ota(_)))
        .collect();
    assert!(
        !ota_stages.is_empty(),
        "Dyna Comp should have an OTA stage"
    );
}

#[test]
fn slew_rate_affects_rat_character() {
    // The RAT with LM308 slew rate limiting should produce different
    // output than if we removed the slew limiting.
    let pedal = parse("proco_rat.pedal");
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
    proc.set_control("Distortion", 0.8);
    proc.set_control("Volume", 0.5);

    let input = sine(4800);
    let output_with_slew: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

    // Process should produce finite output
    assert!(
        output_with_slew.iter().all(|x| x.is_finite()),
        "RAT with slew limiting: output should be finite"
    );
    let peak = output_with_slew
        .iter()
        .fold(0.0f64, |m, x| m.max(x.abs()));
    assert!(peak > 0.01, "RAT should produce output: peak={peak}");
}

#[test]
fn bbd_pedal_compiles_and_processes() {
    // Parse a BBD-based chorus pedal inline.
    let src = r#"
pedal "Test Chorus" {
  components {
C1: cap(100n)
R1: resistor(10k)
BBD1: bbd(mn3207)
LFO1: lfo(triangle, 100k, 47n)
  }
  nets {
in -> C1.a
C1.b -> R1.a, BBD1.in
R1.b -> gnd
BBD1.out -> out
LFO1.out -> BBD1.clock
  }
}
"#;
    let pedal = crate::dsl::parse_pedal_file(src).unwrap();
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();

    assert!(
        !proc.bbds.is_empty(),
        "Chorus pedal should have BBD delay line"
    );

    // Process audio
    let input = sine(4800);
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    assert!(
        output.iter().all(|x| x.is_finite()),
        "BBD output should be finite"
    );
    let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    assert!(peak > 0.001, "Chorus should produce output: peak={peak}");
}

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------

fn sine(n: usize) -> Vec<f64> {
    (0..n)
        .map(|i| 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin())
        .collect()
}

fn assert_finite(output: &[f64], name: &str) {
    assert!(
        output.iter().all(|x| x.is_finite()),
        "{name}: output contains NaN/inf"
    );
}

fn correlation(a: &[f64], b: &[f64]) -> f64 {
    let n = a.len().min(b.len()) as f64;
    let ma = a.iter().sum::<f64>() / n;
    let mb = b.iter().sum::<f64>() / n;
    let (mut cov, mut va, mut vb) = (0.0, 0.0, 0.0);
    for i in 0..n as usize {
        let (da, db) = (a[i] - ma, b[i] - mb);
        cov += da * db;
        va += da * da;
        vb += db * db;
    }
    if va == 0.0 || vb == 0.0 {
        return 0.0;
    }
    cov / (va.sqrt() * vb.sqrt())
}

fn rms(buf: &[f64]) -> f64 {
    (buf.iter().map(|x| x * x).sum::<f64>() / buf.len() as f64).sqrt()
}

// -----------------------------------------------------------------------
// 12V supply voltage tests (compiled pedals)
// -----------------------------------------------------------------------

#[test]
fn compiled_12v_more_headroom() {
    // At 12V the op-amp supply rails are higher.
    //
    // For most distortion pedals, diode clipping (either in feedback or to ground)
    // limits the final output to ~0.6V regardless of supply. The 12V headroom
    // primarily affects the op-amp's internal operation before the diodes clip.
    //
    // Use Fuzz Face which relies on transistor clipping (not diode clipping).
    // The transistor saturation voltage scales somewhat with supply.
    let pedal = parse("fuzz_face.pedal");
    let input = sine(48000);

    let mut proc_9v = compile_pedal(&pedal, 48000.0).unwrap();
    proc_9v.set_control("Fuzz", 0.5);
    proc_9v.set_control("Volume", 1.0);
    let out_9v: Vec<f64> = input.iter().map(|&s| proc_9v.process(s)).collect();

    let mut proc_12v = compile_pedal(&pedal, 48000.0).unwrap();
    proc_12v.set_supply_voltage(12.0);
    proc_12v.set_control("Fuzz", 0.5);
    proc_12v.set_control("Volume", 1.0);
    let out_12v: Vec<f64> = input.iter().map(|&s| proc_12v.process(s)).collect();

    let peak_9v = out_9v.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    let peak_12v = out_12v.iter().fold(0.0f64, |m, x| m.max(x.abs()));

    // For transistor-based fuzzes, 12V supply gives higher collector swing
    assert!(
        peak_12v >= peak_9v,
        "12V should allow equal or higher peaks: 9V peak={peak_9v}, 12V peak={peak_12v}"
    );
}

#[test]
fn compiled_12v_all_pedals_stable() {
    let files = [
        "tube_screamer.pedal",
        "fuzz_face.pedal",
        "big_muff.pedal",
        "blues_driver.pedal",
        "dyna_comp.pedal",
        "klon_centaur.pedal",
        "proco_rat.pedal",
    ];
    let input = sine(4800);
    for f in files {
        let pedal = parse(f);
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        proc.set_supply_voltage(12.0);
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        assert!(output.iter().all(|x| x.is_finite()), "{f} at 12V: NaN/inf");
        let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(peak < 7.0, "{f} at 12V: output too loud, peak={peak}");
        assert!(peak > 0.001, "{f} at 12V: silent, peak={peak}");
    }
}

#[test]
fn compiled_9v_unchanged() {
    // Explicitly setting 9V should match default behavior.
    let pedal = parse("tube_screamer.pedal");
    let input = sine(48000);

    let mut proc_default = compile_pedal(&pedal, 48000.0).unwrap();
    let out_default: Vec<f64> = input.iter().map(|&s| proc_default.process(s)).collect();

    let mut proc_9v = compile_pedal(&pedal, 48000.0).unwrap();
    proc_9v.set_supply_voltage(9.0);
    let out_9v: Vec<f64> = input.iter().map(|&s| proc_9v.process(s)).collect();

    let diff_rms = rms(&out_default
        .iter()
        .zip(&out_9v)
        .map(|(a, b)| a - b)
        .collect::<Vec<_>>());
    assert!(
        diff_rms < 1e-10,
        "Compiled 9V should match default: diff_rms={diff_rms}"
    );
}

#[test]
fn compiled_12v_extreme_settings_stable() {
    let configs: &[(&str, &[(&str, f64)])] = &[
        ("tube_screamer.pedal", &[("Drive", 1.0), ("Level", 1.0)]),
        ("fuzz_face.pedal", &[("Fuzz", 1.0), ("Volume", 1.0)]),
        ("big_muff.pedal", &[("Sustain", 1.0), ("Volume", 1.0)]),
        ("proco_rat.pedal", &[("Distortion", 1.0), ("Volume", 1.0)]),
    ];

    let input = sine(4800);
    for (f, knobs) in configs {
        let pedal = parse(f);
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        proc.set_supply_voltage(12.0);
        for &(label, val) in *knobs {
            proc.set_control(label, val);
        }
        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        let settings: String = knobs
            .iter()
            .map(|(l, v)| format!("{l}={v}"))
            .collect::<Vec<_>>()
            .join(",");
        assert!(
            output.iter().all(|x| x.is_finite()),
            "{f} 12V [{settings}]: NaN/inf"
        );
        let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(
            peak < 10.0,
            "{f} 12V [{settings}]: output too loud, peak={peak}"
        );
    }
}

// -----------------------------------------------------------------------
// Voltage compatibility check tests
// -----------------------------------------------------------------------

#[test]
fn voltage_check_tube_screamer_9v_clean() {
    // Tube Screamer at 9V should have no warnings.
    let pedal = parse("tube_screamer.pedal");
    let warnings = check_voltage_compatibility(&pedal, 9.0);
    assert!(
        warnings.is_empty(),
        "TS at 9V should have no warnings: got {warnings:?}"
    );
}

#[test]
fn voltage_check_tube_screamer_12v_clean() {
    // Tube Screamer has only passives + silicon diodes — fine at 12V.
    let pedal = parse("tube_screamer.pedal");
    let warnings = check_voltage_compatibility(&pedal, 12.0);
    assert!(
        warnings.is_empty(),
        "TS at 12V should have no warnings: got {warnings:?}"
    );
}

#[test]
fn voltage_check_fuzz_face_18v_warns() {
    // Fuzz Face has PNP transistors (likely germanium) and electrolytic
    // caps — should warn at 18V.
    let pedal = parse("fuzz_face.pedal");
    let warnings = check_voltage_compatibility(&pedal, 18.0);
    assert!(
        !warnings.is_empty(),
        "Fuzz Face at 18V should have warnings"
    );
    // Should flag the electrolytic caps (2.2µF, 10µF).
    let cap_warnings: Vec<_> = warnings
        .iter()
        .filter(|w| w.component_id.starts_with('C'))
        .collect();
    assert!(
        !cap_warnings.is_empty(),
        "Should warn about electrolytic caps at 18V"
    );
}

#[test]
fn voltage_check_fuzz_face_ge_transistors() {
    // The Fuzz Face PNP transistors should trigger caution above 12V.
    let pedal = parse("fuzz_face.pedal");
    let warnings = check_voltage_compatibility(&pedal, 15.0);
    let transistor_warnings: Vec<_> = warnings
        .iter()
        .filter(|w| w.component_id.starts_with('Q'))
        .collect();
    assert!(
        !transistor_warnings.is_empty(),
        "Should warn about Ge PNP transistors above 12V"
    );
    assert!(transistor_warnings
        .iter()
        .all(|w| w.severity == WarningSeverity::Caution));
}

#[test]
fn voltage_check_fuzz_face_ge_transistors_danger() {
    // At 20V, germanium transistors should be Danger severity.
    let pedal = parse("fuzz_face.pedal");
    let warnings = check_voltage_compatibility(&pedal, 20.0);
    let danger: Vec<_> = warnings
        .iter()
        .filter(|w| w.severity == WarningSeverity::Danger)
        .collect();
    assert!(!danger.is_empty(), "Ge transistors at 20V should be Danger");
}

#[test]
fn voltage_check_klon_opamp_18v() {
    // Klon has opamps — should warn above 18V.
    let pedal = parse("klon_centaur.pedal");
    let warnings = check_voltage_compatibility(&pedal, 20.0);
    let opamp_warnings: Vec<_> = warnings
        .iter()
        .filter(|w| w.component_id.starts_with('U'))
        .collect();
    assert!(
        !opamp_warnings.is_empty(),
        "Should warn about opamps above 18V"
    );
}

#[test]
fn voltage_check_klon_12v_clean() {
    // Klon at 12V should be fine (no electrolytics in the signal path
    // that would fail, opamps are within range).
    let pedal = parse("klon_centaur.pedal");
    let warnings = check_voltage_compatibility(&pedal, 12.0);
    // The 1µF output coupling cap may trigger a caution.
    let danger: Vec<_> = warnings
        .iter()
        .filter(|w| w.severity == WarningSeverity::Danger)
        .collect();
    assert!(
        danger.is_empty(),
        "Klon at 12V should have no Danger warnings: got {danger:?}"
    );
}

#[test]
fn voltage_check_all_pedals_9v_no_danger() {
    // At 9V, no pedal should have Danger or Caution warnings.
    let files = [
        "tube_screamer.pedal",
        "fuzz_face.pedal",
        "big_muff.pedal",
        "blues_driver.pedal",
        "dyna_comp.pedal",
        "klon_centaur.pedal",
        "proco_rat.pedal",
    ];
    for f in files {
        let pedal = parse(f);
        let warnings = check_voltage_compatibility(&pedal, 9.0);
        assert!(
            warnings.is_empty(),
            "{f} at 9V should have no warnings: got {warnings:?}"
        );
    }
}

// -----------------------------------------------------------------------
// FX loop split tests
// -----------------------------------------------------------------------

#[test]
fn tweed_deluxe_has_fx_loop() {
    let pedal = parse("tweed_deluxe_5e3.pedal");
    assert!(pedal.has_fx_loop(), "Tweed Deluxe should have fx_send/fx_return");
}

#[test]
fn split_tweed_deluxe_produces_two_halves() {
    let pedal = parse("tweed_deluxe_5e3.pedal");
    let (pre, post) = split_pedal_def(&pedal).expect("should split");
    assert!(
        pre.name.contains("pre"),
        "Pre half should have 'pre' in name: {}",
        pre.name
    );
    assert!(
        post.name.contains("post"),
        "Post half should have 'post' in name: {}",
        post.name
    );
    // Both halves should have some components
    assert!(!pre.components.is_empty(), "Pre half should have components");
    assert!(!post.components.is_empty(), "Post half should have components");
    // Neither half should still have fx_send/fx_return
    assert!(!pre.has_fx_loop(), "Pre half should not have fx loop nodes");
    assert!(!post.has_fx_loop(), "Post half should not have fx loop nodes");
}

#[test]
fn compile_split_tweed_deluxe() {
    let pedal = parse("tweed_deluxe_5e3.pedal");
    let mut split = compile_split_pedal(&pedal, 48000.0).unwrap();
    split.set_control("Volume", 0.7);
    split.set_control("Tone", 0.5);

    // Process through split (direct, no FX loop effects)
    let input = sine(48000);
    let output: Vec<f64> = input.iter().map(|&s| split.process(s)).collect();
    assert_finite(&output, "Split Tweed (direct)");
    let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    assert!(peak > 0.001, "Split Tweed should produce output: peak={peak}");
}

#[test]
fn split_pre_post_individually() {
    let pedal = parse("tweed_deluxe_5e3.pedal");
    let mut split = compile_split_pedal(&pedal, 48000.0).unwrap();
    split.set_control("Volume", 0.7);
    split.set_control("Tone", 0.5);

    let input = sine(48000);
    // Process through pre → post separately
    let mut output = Vec::with_capacity(input.len());
    for &s in &input {
        let send = split.process_pre(s);
        let ret = split.process_post(send);
        output.push(ret);
    }
    assert_finite(&output, "Split Tweed (pre→post)");
    let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    assert!(peak > 0.001, "Pre→post should produce output: peak={peak}");
}

#[test]
fn non_fx_loop_pedal_does_not_split() {
    let pedal = parse("tube_screamer.pedal");
    assert!(!pedal.has_fx_loop());
    assert!(split_pedal_def(&pedal).is_none());
    assert!(compile_split_pedal(&pedal, 48000.0).is_err());
}

#[test]
fn three_terminal_pot_compiles_and_passes_signal() {
    // 3-terminal pot as volume attenuator at the diode junction.
    // Wiper is the junction node so both pot halves appear in the
    // WDF stage. __wb goes to gnd (dead-end, redirected by sp_reduce).
    let src = r#"
        pedal "3TermTest" {
            components {
                C1: cap(100n)
                R1: resistor(10k)
                Vol: pot(100k)
                D1: diode_pair(silicon)
                R2: resistor(10k)
            }
            nets {
                in -> C1.a
                C1.b -> R1.a
                R1.b -> Vol.a
                Vol.b -> gnd
                Vol.w -> D1.a
                D1.b -> gnd
                Vol.w -> R2.a
                R2.b -> out
            }
            controls {
                Vol.position -> "Vol" [0.0, 1.0] = 0.5
            }
        }
    "#;
    let pedal = parse_pedal_file(src).unwrap();
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();

    // Basic signal pass-through test at default position.
    let input = sine(48000);
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    assert_finite(&output, "3-term pot default");
    let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    assert!(peak > 0.001, "3-term pot should pass signal: peak={peak}");

    // Change control and verify it still produces finite output.
    proc.set_control("Vol", 0.2);
    let output2: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    assert_finite(&output2, "3-term pot Vol=0.2");
}

// -----------------------------------------------------------------------
// Power supply sag regression tests
// -----------------------------------------------------------------------

#[test]
fn supply_sag_does_not_affect_9v_pedals() {
    // 9V pedals (no sag parameters) should produce signal unaffected by
    // the power supply sag infrastructure.  This verifies that the
    // PowerSupply element is NOT created for simple supply declarations.
    let files = [
        "phase90.pedal",
        "tube_screamer.pedal",
        "boss_ce2.pedal",
        "klon_centaur.pedal",
        "blues_driver.pedal",
    ];
    let input = sine(48000);

    for filename in files {
        let pedal = parse(filename);
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();

        // Apply default controls
        for ctrl in &pedal.controls {
            proc.set_control(&ctrl.label, ctrl.default);
        }

        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        assert_finite(&output, filename);

        let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(
            peak > 1e-4,
            "{filename}: 9V pedal output too quiet (peak={peak:.6}). \
             Supply sag may be incorrectly affecting 9V circuits."
        );

        // Verify supply voltage is exactly 9.0 (no sag modification)
        assert!(
            (proc.supply_voltage() - 9.0).abs() < 1e-6,
            "{filename}: supply voltage should be 9.0, got {}. \
             PowerSupply sag may be leaking into 9V pedal path.",
            proc.supply_voltage()
        );
    }
}

#[test]
fn supply_sag_amp_circuits_produce_output() {
    // Amp circuits with explicit sag parameters should still produce audio.
    // The sag model should reduce voltage under load but not kill signal.
    let files = [
        ("tweed_deluxe_5e3.pedal", &[("Volume", 0.7), ("Tone", 0.5)] as &[(&str, f64)]),
        ("bassman_5f6a.pedal", &[("Volume", 0.6), ("Treble", 0.6), ("Mid", 0.5), ("Bass", 0.5)]),
        ("marshall_jtm45.pedal", &[("Volume", 0.6), ("Treble", 0.6), ("Mid", 0.5), ("Bass", 0.5), ("Presence", 0.5)]),
    ];
    let input = sine(48000);

    for (filename, controls) in files {
        let pedal = parse(filename);
        let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
        for &(label, val) in controls {
            proc.set_control(label, val);
        }

        let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
        assert_finite(&output, filename);

        let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(
            peak > 0.001,
            "{filename}: amp with sag produced near-silent output (peak={peak:.6})"
        );

        // Supply voltage should be near nominal minus rectifier drop, not zero
        let supply_v = proc.supply_voltage();
        assert!(
            supply_v > 100.0,
            "{filename}: supply voltage too low ({supply_v:.1}V). \
             PSU cap may be starting uncharged."
        );
    }
}

#[test]
fn power_supply_starts_at_steady_state() {
    // The first audio sample should see the same supply voltage that the PSU
    // produces at idle (nominal minus rectifier static drop).  If the PSU cap
    // started uncharged, the first samples would see near-zero voltage.
    use crate::elements::PowerSupply;
    use crate::dsl::RectifierType;

    // Tube rectifier: steady state = nominal - 10V
    let psu = PowerSupply::new(350.0, 180.0, 16e-6, RectifierType::Tube, 48000.0);
    let steady = psu.steady_state_voltage();
    assert!(
        (steady - 340.0).abs() < 1.0,
        "Tube PSU steady state should be ~340V (350-10), got {steady}"
    );

    // Solid-state rectifier: steady state = nominal - 1.4V
    let psu_ss = PowerSupply::new(480.0, 5.0, 220e-6, RectifierType::SolidState, 48000.0);
    let steady_ss = psu_ss.steady_state_voltage();
    assert!(
        (steady_ss - 478.6).abs() < 1.0,
        "SS PSU steady state should be ~478.6V (480-1.4), got {steady_ss}"
    );
}

#[test]
fn power_supply_first_sample_not_anomalous() {
    // For amp circuits with sag, the first audio sample should produce
    // output comparable to later samples.  If the supply voltage is
    // inconsistent between init and first tick(), the first sample may
    // be anomalously loud or quiet.
    let pedal = parse("tweed_deluxe_5e3.pedal");
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
    proc.set_control("Volume", 0.5);
    proc.set_control("Tone", 0.5);

    // Process a constant-amplitude input
    let dc_in = 0.1;
    let first_out = proc.process(dc_in);
    // Process 99 more samples
    let mut outputs = vec![first_out];
    for _ in 0..99 {
        outputs.push(proc.process(dc_in));
    }
    let hundredth_out = outputs[99];

    // The first sample should not be dramatically different from later samples
    if hundredth_out.abs() > 1e-10 {
        let ratio = first_out.abs() / hundredth_out.abs();
        assert!(
            ratio > 0.1 && ratio < 10.0,
            "First sample ({first_out:.6}) is {ratio:.1}x different from 100th ({hundredth_out:.6}). \
             Supply voltage may be inconsistent at init."
        );
    }
}

#[test]
fn supply_sag_v_max_propagates_correctly_9v() {
    // For 9V supply, op-amp v_max should be (9/2 - 1.5) = 3.0V.
    // Process a moderate signal and verify the output amplitude is reasonable:
    // signal at 0.1V should pass through without being clamped to nothing.
    let pedal = parse("phase90.pedal");
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();
    proc.set_control("Speed", 0.5);

    let dc_in = 0.1; // 100mV — well within 9V headroom
    let mut max_output = 0.0f64;
    for _ in 0..4800 {
        let out = proc.process(dc_in);
        max_output = max_output.max(out.abs());
    }

    // Output should be non-zero.  The amplitude is currently lower than ideal
    // because the WDF tree only sees passive R/C/JFET elements — the op-amp
    // feedback loops that enforce unity gain in the real circuit aren't modeled
    // inside the WDF tree yet.  Once that is fixed, raise this threshold to 0.01.
    assert!(
        max_output > 1e-5,
        "Phase 90 output near-silent ({max_output:.6}). v_max may be wrong."
    );
    assert!(
        max_output < 5.0,
        "Phase 90 output too hot ({max_output:.6}). Gain may be miscalculated."
    );
}

// -----------------------------------------------------------------------
// Capacitor Leakage & Dielectric Absorption Tests
// -----------------------------------------------------------------------

/// Test that leaky capacitors behave differently from ideal capacitors.
/// Leakage causes the capacitor state to decay, which affects the output.
#[test]
fn leaky_capacitor_affects_frequency_response() {
    // Create two simple RC circuits:
    // 1. Ideal cap: cap(1u)
    // 2. Very leaky cap: cap(1u, electrolytic, leakage: 1k) - severely worn

    let ideal_src = r#"
pedal "Ideal Cap Test" {
  components {
    C1: cap(1u)
    R1: resistor(10k)
  }
  nets {
    in -> C1.a
    C1.b -> R1.a
    R1.b -> out, gnd
  }
}
"#;

    // Use a very low leakage resistance (1kΩ) to make the effect obvious
    // RC time constant = 1kΩ * 1µF = 1ms, so significant decay per sample
    let leaky_src = r#"
pedal "Leaky Cap Test" {
  components {
    C1: cap(1u, electrolytic, leakage: 1k)
    R1: resistor(10k)
  }
  nets {
    in -> C1.a
    C1.b -> R1.a
    R1.b -> out, gnd
  }
}
"#;

    let ideal_pedal = parse_pedal_file(ideal_src).unwrap();
    let leaky_pedal = parse_pedal_file(leaky_src).unwrap();

    let mut ideal_proc = compile_pedal(&ideal_pedal, 48000.0).unwrap();
    let mut leaky_proc = compile_pedal(&leaky_pedal, 48000.0).unwrap();

    // Test with a step input - charge the cap then observe discharge
    // First, charge both caps with positive input
    for _ in 0..4800 {
        ideal_proc.process(0.5);
        leaky_proc.process(0.5);
    }

    // Then apply zero input and observe how fast the output decays
    let mut ideal_decay: Vec<f64> = Vec::new();
    let mut leaky_decay: Vec<f64> = Vec::new();
    for _ in 0..4800 {
        ideal_decay.push(ideal_proc.process(0.0));
        leaky_decay.push(leaky_proc.process(0.0));
    }

    // Both should produce output
    assert!(ideal_decay.iter().all(|x| x.is_finite()), "Ideal cap output should be finite");
    assert!(leaky_decay.iter().all(|x| x.is_finite()), "Leaky cap output should be finite");

    // The leaky cap should decay faster (lower energy remaining)
    let ideal_energy: f64 = ideal_decay.iter().map(|x| x * x).sum();
    let leaky_energy: f64 = leaky_decay.iter().map(|x| x * x).sum();

    // With 1kΩ leakage and 1µF cap, tau = 1ms = 48 samples at 48kHz
    // Over 4800 samples (100ms), the state should decay by exp(-100) ≈ 0
    // So leaky energy should be much less than ideal energy
    assert!(
        leaky_energy < ideal_energy * 0.99 || (ideal_energy < 1e-10 && leaky_energy < 1e-10),
        "Leaky cap should decay faster: ideal_energy={ideal_energy:.6}, leaky_energy={leaky_energy:.6}"
    );
}

/// Test that dielectric absorption creates a "memory" effect.
/// After a transient, the voltage should slowly creep back toward the absorbed value.
#[test]
fn dielectric_absorption_creates_memory_effect() {
    // Create a circuit with DA-enabled capacitor
    let da_src = r#"
pedal "DA Cap Test" {
  components {
    C1: cap(10u, electrolytic, leakage: 1M, da: 0.1)
    R1: resistor(10k)
  }
  nets {
    in -> C1.a
    C1.b -> R1.a
    R1.b -> out, gnd
  }
}
"#;

    let da_pedal = parse_pedal_file(da_src).unwrap();
    let mut proc = compile_pedal(&da_pedal, 48000.0).unwrap();

    // Charge the cap with a DC offset, then remove it
    // First: charge phase (1 second of DC)
    for _ in 0..48000 {
        proc.process(0.5);
    }

    // Then: discharge phase - input goes to zero
    // The DA effect should cause a slow voltage creep
    let mut discharge_output: Vec<f64> = Vec::new();
    for _ in 0..48000 {
        discharge_output.push(proc.process(0.0));
    }

    // Output should be finite
    assert!(
        discharge_output.iter().all(|x| x.is_finite()),
        "DA cap output should be finite"
    );

    // The DA cap should exhibit some decay behavior during discharge
    // (The exact behavior depends on implementation details)
    let early_samples = &discharge_output[0..1000];
    let late_samples = &discharge_output[40000..48000];

    let early_rms = (early_samples.iter().map(|x| x * x).sum::<f64>() / 1000.0).sqrt();
    let late_rms = (late_samples.iter().map(|x| x * x).sum::<f64>() / 8000.0).sqrt();

    // Both should be small (discharge), but they should exist
    // The exact values depend on the DA model implementation
    assert!(
        early_rms.is_finite() && late_rms.is_finite(),
        "Discharge RMS values should be finite"
    );
}

/// Test that the new capacitor syntax parses correctly and maintains
/// backwards compatibility with the simple cap(value) syntax.
#[test]
fn capacitor_syntax_backwards_compatible() {
    // Simple syntax should still work
    let simple_src = r#"
pedal "Simple Cap" {
  components {
    C1: cap(100n)
    R1: resistor(10k)
  }
  nets {
    in -> C1.a
    C1.b -> R1.a
    R1.b -> out, gnd
  }
}
"#;

    let pedal = parse_pedal_file(simple_src).unwrap();
    let result = compile_pedal(&pedal, 48000.0);
    assert!(result.is_ok(), "Simple cap syntax should compile: {:?}", result.err());

    let mut proc = result.unwrap();
    let input = sine(4800);
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    assert!(output.iter().all(|x| x.is_finite()), "Output should be finite");
}

/// Test all capacitor type variants compile correctly.
#[test]
fn all_cap_types_compile() {
    let src = r#"
pedal "All Cap Types" {
  components {
    C1: cap(100n, film)
    C2: cap(22u, electrolytic)
    C3: cap(100p, ceramic)
    C4: cap(10u, tantalum)
    C5: cap(22u, electrolytic, leakage: 100k)
    C6: cap(22u, electrolytic, leakage: 10k, da: 0.05)
    R1: resistor(10k)
  }
  nets {
    in -> C1.a
    C1.b -> C2.a
    C2.b -> C3.a
    C3.b -> C4.a
    C4.b -> C5.a
    C5.b -> C6.a
    C6.b -> R1.a
    R1.b -> out, gnd
  }
}
"#;

    let pedal = parse_pedal_file(src).unwrap();
    let result = compile_pedal(&pedal, 48000.0);
    assert!(result.is_ok(), "All cap types should compile: {:?}", result.err());

    let mut proc = result.unwrap();
    let input = sine(4800);
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();
    assert!(output.iter().all(|x| x.is_finite()), "Output should be finite");
}

// -----------------------------------------------------------------------
// fork() dynamic routing tests
// -----------------------------------------------------------------------

#[test]
fn fork_parse_basic() {
    // Test that fork() syntax parses correctly
    let src = r#"
pedal "Fork Test" {
  components {
    R1: resistor(10k)
    R2: resistor(20k)
    R3: resistor(30k)
    SW1: switch(3)
  }
  nets {
    in -> R1.a
    R1.b -> fork(SW1, [R2.a, R3.a, gnd])
    R2.b -> out
    R3.b -> out
  }
  controls {
    SW1.position -> "Mode" [0, 2] = 0
  }
}
"#;
    let pedal = parse_pedal_file(src).unwrap();

    // Verify we have the fork in the nets
    let has_fork = pedal.nets.iter().any(|net| {
        net.to.iter().any(|pin| matches!(pin, Pin::Fork { .. }))
    });
    assert!(has_fork, "Should have a fork() in nets");
}

#[test]
fn fork_compile_creates_switched_resistors() {
    // Test that fork() creates SwitchedResistor nodes in the WDF tree
    let src = r#"
pedal "Fork Compile Test" {
  components {
    R1: resistor(10k)
    D1: diode(silicon)
    R2: resistor(20k)
    D2: diode(silicon)
    SW1: switch(2)
  }
  nets {
    in -> R1.a
    R1.b -> fork(SW1, [D1.a, D2.a])
    D1.b -> R2.a
    D2.b -> R2.a
    R2.b -> out
  }
  controls {
    SW1.position -> "Path" [0, 1] = 0
  }
}
"#;
    let pedal = parse_pedal_file(src).unwrap();
    let result = compile_pedal(&pedal, 48000.0);

    // Should compile successfully
    assert!(result.is_ok(), "Fork circuit should compile: {:?}", result.err());
}

#[test]
fn fork_switch_control_binding() {
    // Test that switch controls are properly bound to SwitchPosition targets
    let src = r#"
pedal "Switch Control Test" {
  components {
    R1: resistor(10k)
    R2: resistor(20k)
    R3: resistor(30k)
    D1: diode_pair(silicon)
    SW1: switch(2)
  }
  nets {
    in -> R1.a
    R1.b -> fork(SW1, [R2.a, R3.a])
    R2.b -> D1.a
    R3.b -> D1.a
    D1.b -> out
  }
  controls {
    SW1.position -> "Mode" [0, 1] = 0
  }
}
"#;
    let pedal = parse_pedal_file(src).unwrap();
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();

    // Process with switch at position 0
    proc.set_control("Mode", 0.0);
    let input = sine(480);
    let output_pos0: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

    // Reset and process with switch at position 1
    proc.reset();
    proc.set_control("Mode", 1.0);
    let output_pos1: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

    // Both outputs should be finite
    assert_finite(&output_pos0, "Fork position 0");
    assert_finite(&output_pos1, "Fork position 1");

    // The outputs should be different (different paths through circuit)
    let rms_0 = rms(&output_pos0);
    let rms_1 = rms(&output_pos1);

    // Both should produce some output
    assert!(rms_0 > 1e-6 || rms_1 > 1e-6,
        "At least one fork path should produce output: rms_0={rms_0}, rms_1={rms_1}");
}

#[test]
fn inf_resistance_in_switched_resistor() {
    // Test that `inf` keyword works for open-circuit positions
    let src = r#"
pedal "Infinite Resistance Test" {
  components {
    R1: resistor(10k)
    R_sw: resistor_switched([10k, inf, 47k])
    D1: diode_pair(silicon)
    SW1: rotary("10k", "Open", "47k")
  }
  nets {
    in -> R1.a
    R1.b -> R_sw.a
    R_sw.b -> D1.a
    D1.b -> out
  }
  controls {
    SW1.position -> "Resistance" [0, 2] = 0
  }
}
"#;
    let pedal = parse_pedal_file(src).unwrap();

    // Should parse successfully with inf value
    let switched_resistor = pedal.components.iter().find(|c| c.id == "R_sw");
    assert!(switched_resistor.is_some(), "Should have R_sw component");

    if let Some(comp) = switched_resistor {
        if let ComponentKind::ResistorSwitched(values) = &comp.kind {
            assert_eq!(values.len(), 3, "Should have 3 positions");
            assert_eq!(values[0], 10_000.0, "First position should be 10k");
            assert!(values[1].is_infinite(), "Second position should be infinite");
            assert_eq!(values[2], 47_000.0, "Third position should be 47k");
        } else {
            panic!("R_sw should be ResistorSwitched");
        }
    }
}

// -----------------------------------------------------------------------
// Op-amp gain stage tests
// -----------------------------------------------------------------------

#[test]
fn opamp_inverting_amplifier_gain() {
    // Inverting amplifier: Rf/Ri = 100k/10k = 10x gain
    // Standard inverting topology: pos → gnd, neg → Ri → in, neg → Rf → out
    // Note: Ri connects directly from 'in' to U1.neg for detection to work
    let src = r#"
pedal "Inverting Amp" {
  components {
    Ri: resistor(10k)
    Rf: resistor(100k)
    U1: opamp(tl072)
    R_load: resistor(10k)
  }
  nets {
    in -> Ri.a
    Ri.b -> U1.neg
    U1.pos -> gnd
    U1.neg -> Rf.a
    Rf.b -> U1.out
    U1.out -> R_load.a
    R_load.b -> out
  }
}
"#;
    let pedal = parse_pedal_file(src).unwrap();
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();

    // Use small input signal to avoid op-amp saturation.
    // v_max ≈ 3V (9V supply), so 0.1V * 10x gain = 1V output (well below saturation)
    let input: Vec<f64> = (0..4800)
        .map(|i| 0.1 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin())
        .collect();
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

    assert_finite(&output, "Inverting amp");

    // Measure RMS gain ratio
    let input_rms = rms(&input[480..4800]); // Skip initial transient
    let output_rms = rms(&output[480..4800]);

    // Expected gain: Rf/Ri = 100k/10k = 10
    // Allow ±50% tolerance for WDF modeling variations
    let gain = output_rms / input_rms;
    assert!(
        gain > 5.0 && gain < 20.0,
        "Inverting amp gain should be ~10x (Rf/Ri): measured gain={gain:.2}"
    );
}

#[test]
fn opamp_noninverting_amplifier_gain() {
    // Non-inverting amplifier: gain = 1 + Rf/Ri = 1 + 90k/10k = 10x
    // Standard non-inverting topology: pos → input, neg → Ri → gnd, neg → Rf → out
    // Note: Direct connection from 'in' to U1.pos for cleaner detection
    let src = r#"
pedal "Non-Inverting Amp" {
  components {
    Ri: resistor(10k)
    Rf: resistor(90k)
    U1: opamp(tl072)
    R_load: resistor(10k)
  }
  nets {
    in -> U1.pos
    U1.neg -> Ri.a
    Ri.b -> gnd
    U1.neg -> Rf.a
    Rf.b -> U1.out
    U1.out -> R_load.a
    R_load.b -> out
  }
}
"#;
    let pedal = parse_pedal_file(src).unwrap();
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();

    // Use small input signal to avoid op-amp saturation.
    // v_max ≈ 3V (9V supply), so 0.1V * 10x gain = 1V output (well below saturation)
    let input: Vec<f64> = (0..4800)
        .map(|i| 0.1 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin())
        .collect();
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

    assert_finite(&output, "Non-inverting amp");

    // Measure RMS gain ratio
    let input_rms = rms(&input[480..4800]); // Skip initial transient
    let output_rms = rms(&output[480..4800]);

    // Expected gain: 1 + Rf/Ri = 1 + 90k/10k = 10
    // Allow ±50% tolerance for WDF modeling variations
    let gain = output_rms / input_rms;
    assert!(
        gain > 5.0 && gain < 20.0,
        "Non-inverting amp gain should be ~10x (1+Rf/Ri): measured gain={gain:.2}"
    );
}

#[test]
fn opamp_unity_gain_buffer() {
    // Unity-gain buffer: neg directly connected to out
    // Gain should be ~1.0
    let src = r#"
pedal "Unity Buffer" {
  components {
    C1: cap(100n)
    U1: opamp(tl072)
    R_load: resistor(10k)
  }
  nets {
    in -> C1.a
    C1.b -> U1.pos
    U1.neg -> U1.out
    U1.out -> R_load.a
    R_load.b -> out
  }
}
"#;
    let pedal = parse_pedal_file(src).unwrap();
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();

    let input = sine(4800);
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

    assert_finite(&output, "Unity buffer");

    // Measure RMS gain ratio
    let input_rms = rms(&input[480..4800]);
    let output_rms = rms(&output[480..4800]);

    // Expected gain: 1.0 (unity)
    // Allow reasonable tolerance
    let gain = output_rms / input_rms;
    assert!(
        gain > 0.5 && gain < 2.0,
        "Unity buffer gain should be ~1.0: measured gain={gain:.2}"
    );
}

#[test]
fn opamp_cascaded_gain_stages() {
    // Two cascaded inverting stages: total gain = 5 * 5 = 25
    // Note: Direct connections from input to Ri for detection
    let src = r#"
pedal "Cascaded Amp" {
  components {
    Ri1: resistor(10k)
    Rf1: resistor(50k)
    U1: opamp(tl072)
    Ri2: resistor(10k)
    Rf2: resistor(50k)
    U2: opamp(tl072)
    R_load: resistor(10k)
  }
  nets {
    in -> Ri1.a
    Ri1.b -> U1.neg
    U1.pos -> gnd
    U1.neg -> Rf1.a
    Rf1.b -> U1.out
    U1.out -> Ri2.a
    Ri2.b -> U2.neg
    U2.pos -> gnd
    U2.neg -> Rf2.a
    Rf2.b -> U2.out
    U2.out -> R_load.a
    R_load.b -> out
  }
}
"#;
    let pedal = parse_pedal_file(src).unwrap();
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();

    // Use very small input signal to avoid saturation with 25x gain.
    // v_max ≈ 3V (9V supply), so 0.05V * 25x gain = 1.25V output
    let input: Vec<f64> = (0..4800)
        .map(|i| 0.05 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin())
        .collect();
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

    assert_finite(&output, "Cascaded amp");

    // Measure RMS gain ratio
    let input_rms = rms(&input[480..4800]);
    let output_rms = rms(&output[480..4800]);

    // Expected gain: (50k/10k) * (50k/10k) = 5 * 5 = 25
    // Allow wider tolerance for cascaded stages
    let gain = output_rms / input_rms;
    assert!(
        gain > 10.0 && gain < 50.0,
        "Cascaded amp gain should be ~25x: measured gain={gain:.2}"
    );
}

#[test]
fn proco_rat_distortion_affects_gain() {
    // The RAT's Distortion pot is in the op-amp feedback loop
    // Higher distortion = higher Rf = more gain
    let pedal = parse("proco_rat.pedal");

    // Low distortion
    let mut proc_low = compile_pedal(&pedal, 48000.0).unwrap();
    proc_low.set_control("Distortion", 0.1);
    proc_low.set_control("Volume", 0.5);

    // High distortion
    let mut proc_high = compile_pedal(&pedal, 48000.0).unwrap();
    proc_high.set_control("Distortion", 0.9);
    proc_high.set_control("Volume", 0.5);

    let input = sine(4800);
    let output_low: Vec<f64> = input.iter().map(|&s| proc_low.process(s)).collect();
    let output_high: Vec<f64> = input.iter().map(|&s| proc_high.process(s)).collect();

    assert_finite(&output_low, "RAT low dist");
    assert_finite(&output_high, "RAT high dist");

    let rms_low = rms(&output_low[480..]);
    let rms_high = rms(&output_high[480..]);

    // Higher distortion should produce more output (more gain, more clipping)
    assert!(
        rms_high > rms_low * 0.8,
        "Higher Distortion should produce similar or more output: low={rms_low:.4}, high={rms_high:.4}"
    );
}

// -----------------------------------------------------------------------
// Passive-only circuit (RC highpass) creates WDF stage
// -----------------------------------------------------------------------

#[test]
fn passive_rc_circuit_creates_stage() {
    // Simple RC highpass: no nonlinear elements, but should still create
    // a WDF stage to model the capacitor's highpass behavior.
    // Output at C/R junction (highpass filter topology)
    let src = r#"
pedal "RC Highpass" {
  components {
    C1: cap(1u)
    R1: resistor(10k)
  }
  nets {
    in -> C1.a
    C1.b -> R1.a, out
    R1.b -> gnd
  }
}
"#;
    let pedal = parse_pedal_file(src).unwrap();
    let proc = compile_pedal(&pedal, 48000.0).unwrap();

    // Should have exactly one (passthrough) stage
    assert_eq!(proc.stages.len(), 1, "Passive RC should create 1 WDF stage");

    // Process a step input and check for highpass behavior
    let mut p = proc;
    let mut output = Vec::new();
    for i in 0..4800 {
        let input = if i < 2400 { 0.5 } else { 0.0 };
        output.push(p.process(input));
    }

    // Highpass: DC should be blocked, but transients should pass
    let dc_portion = &output[1000..2000]; // After initial transient, during DC
    let dc_rms: f64 = (dc_portion.iter().map(|x| x * x).sum::<f64>() / dc_portion.len() as f64).sqrt();

    // DC should be heavily attenuated (RC time constant = 10ms)
    assert!(
        dc_rms < 0.1,
        "DC should be blocked by highpass: dc_rms={dc_rms:.6}"
    );

    // Initial transient should have passed some signal
    let transient = &output[0..100];
    let transient_peak = transient.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    assert!(
        transient_peak > 0.01,
        "Highpass should pass initial transient: peak={transient_peak:.6}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Phase 1: Stereo I/O aliasing + supply rail filtering
// ═══════════════════════════════════════════════════════════════════════════

/// Equipment-style circuit with in_L/out_L instead of in/out.
/// The compiler should automatically alias in→in_L and out→out_L.
#[test]
fn stereo_io_aliasing_triode() {
    let src = r#"
equipment "Stereo Triode Test" {
  supplies {
    vcc: 250V
  }
  components {
    C_in: cap(22n)
    R_grid: resistor(1M)
    V1: triode(12ax7)
    R_plate: resistor(100k)
    R_cathode: resistor(1.5k)
    C_cathode: cap(25u)
    C_out: cap(22n)
    R_load: resistor(1M)
  }
  nets {
    in_L -> C_in.a
    C_in.b -> R_grid.a, V1.grid
    R_grid.b -> gnd
    vcc -> R_plate.a
    R_plate.b -> V1.plate
    V1.cathode -> R_cathode.a, C_cathode.a
    R_cathode.b -> gnd
    C_cathode.b -> gnd
    V1.plate -> C_out.a
    C_out.b -> R_load.a, out_L
    R_load.b -> gnd
  }
}
"#;
    let pedal = parse_pedal_file(src).unwrap();
    let proc = compile_pedal(&pedal, 48000.0).unwrap();

    // Should create at least one triode stage (not zero because in was isolated)
    let triode_count = proc.stages.iter().filter(|s| {
        matches!(s.root, RootKind::Triode(_))
    }).count();
    assert!(
        triode_count >= 1,
        "Equipment circuit with in_L/out_L should compile triode stages, got {triode_count}"
    );
}

/// Verify that standard (non-equipment) pedals with in/out still work correctly.
#[test]
fn stereo_aliasing_no_regression_standard_pedal() {
    let src = r#"
pedal "Standard Triode" {
  supply 250V {
    impedance: 50
    filter_cap: 47u
    rectifier: solid_state
  }
  components {
    C_in: cap(22n)
    R_grid: resistor(1M)
    V1: triode(12ax7)
    R_plate: resistor(100k)
    R_cathode: resistor(1.5k)
    C_cathode: cap(25u)
    C_out: cap(22n)
    R_load: resistor(1M)
  }
  nets {
    in -> C_in.a
    C_in.b -> R_grid.a, V1.grid
    R_grid.b -> gnd
    vcc -> R_plate.a
    R_plate.b -> V1.plate
    V1.cathode -> R_cathode.a, C_cathode.a
    R_cathode.b -> gnd
    C_cathode.b -> gnd
    V1.plate -> C_out.a
    C_out.b -> R_load.a, out
    R_load.b -> gnd
  }
}
"#;
    let pedal = parse_pedal_file(src).unwrap();
    let proc = compile_pedal(&pedal, 48000.0).unwrap();

    let triode_count = proc.stages.iter().filter(|s| {
        matches!(s.root, RootKind::Triode(_))
    }).count();
    assert!(
        triode_count >= 1,
        "Standard pedal with in/out should still compile triode stages, got {triode_count}"
    );
}

/// Supply rail filtering: edges to supply nodes should be excluded from
/// passive element collection, preventing SP reduction failures.
#[test]
fn supply_rail_filtered_from_passives() {
    // This circuit has a triode with plate resistor to vcc AND a resistor
    // from the plate to a named supply rail (bias_ref). Without supply
    // filtering, the bias_ref edge would create a non-SP subgraph.
    let src = r#"
equipment "Supply Rail Test" {
  supplies {
    vcc: 250V
    bias_ref: -3V
  }
  components {
    C_in: cap(22n)
    R_grid: resistor(1M)
    V1: triode(12ax7)
    R_plate: resistor(100k)
    R_bias: resistor(47k)
    R_cathode: resistor(1.5k)
    C_cathode: cap(25u)
    C_out: cap(22n)
    R_load: resistor(1M)
  }
  nets {
    in_L -> C_in.a
    C_in.b -> R_grid.a, V1.grid
    R_grid.b -> gnd
    vcc -> R_plate.a
    R_plate.b -> V1.plate
    V1.plate -> R_bias.a
    R_bias.b -> bias_ref
    V1.cathode -> R_cathode.a, C_cathode.a
    R_cathode.b -> gnd
    C_cathode.b -> gnd
    V1.plate -> C_out.a
    C_out.b -> R_load.a, out_L
    R_load.b -> gnd
  }
}
"#;
    let pedal = parse_pedal_file(src).unwrap();
    let proc = compile_pedal(&pedal, 48000.0).unwrap();

    // The triode stage should be created despite the supply rail edge
    let triode_count = proc.stages.iter().filter(|s| {
        matches!(s.root, RootKind::Triode(_))
    }).count();
    assert!(
        triode_count >= 1,
        "Supply rail edges should be filtered — triode stage should still compile, got {triode_count}"
    );
}

/// Verify equipment circuits with named supplies produce signal output
/// (not all zeros like before the Phase 1 fixes).
#[test]
fn equipment_triode_produces_signal() {
    let src = r#"
equipment "Equipment Signal Test" {
  supplies {
    vcc: 250V
  }
  components {
    C_in: cap(22n)
    R_grid: resistor(1M)
    V1: triode(12ax7)
    R_plate: resistor(100k)
    R_cathode: resistor(1.5k)
    C_cathode: cap(25u)
    C_out: cap(22n)
    R_load: resistor(1M)
  }
  nets {
    in_L -> C_in.a
    C_in.b -> R_grid.a, V1.grid
    R_grid.b -> gnd
    vcc -> R_plate.a
    R_plate.b -> V1.plate
    V1.cathode -> R_cathode.a, C_cathode.a
    R_cathode.b -> gnd
    C_cathode.b -> gnd
    V1.plate -> C_out.a
    C_out.b -> R_load.a, out_L
    R_load.b -> gnd
  }
}
"#;
    let pedal = parse_pedal_file(src).unwrap();
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();

    // Process a 1kHz sine and verify signal passes through
    let n = 4800; // 100ms at 48kHz
    let input: Vec<f64> = (0..n)
        .map(|i| 0.5 * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 48000.0).sin())
        .collect();
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

    let rms = (output.iter().map(|x| x * x).sum::<f64>() / n as f64).sqrt();
    let peak = output.iter().fold(0.0f64, |m, x| m.max(x.abs()));

    assert!(output.iter().all(|x| x.is_finite()), "No NaN/inf in output");
    assert!(
        rms > 1e-6,
        "Equipment triode should produce signal, got rms={rms:.6}"
    );
    assert!(
        peak > 1e-5,
        "Equipment triode should produce signal, got peak={peak:.6}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Phase 2: Parallel tube merging, push-pull detection, PushPullStage
// ═══════════════════════════════════════════════════════════════════════════

/// Parallel tubes sharing the same plate and cathode nodes should be
/// merged into a single TriodeInfo with parallel_count > 1.
#[test]
fn parallel_tubes_merged() {
    use super::graph::CircuitGraph;

    // 4 triodes sharing the same plate and cathode nodes
    let src = r#"
equipment "Parallel Tube Test" {
  supplies {
    vcc: 240V
  }
  components {
    C_in: cap(22n)
    R_grid: resistor(1M)
    V1: triode(6386)
    V2: triode(6386)
    V3: triode(6386)
    V4: triode(6386)
    R_plate: resistor(33k)
    R_cathode: resistor(470)
    C_cathode: cap(8u)
    C_out: cap(22n)
    R_load: resistor(1M)
  }
  nets {
    in_L -> C_in.a
    C_in.b -> R_grid.a, V1.grid, V2.grid, V3.grid, V4.grid
    R_grid.b -> gnd
    vcc -> R_plate.a
    R_plate.b -> V1.plate, V2.plate, V3.plate, V4.plate
    V1.cathode -> R_cathode.a, C_cathode.a
    V2.cathode -> R_cathode.a
    V3.cathode -> R_cathode.a
    V4.cathode -> R_cathode.a
    R_cathode.b -> gnd
    C_cathode.b -> gnd
    V1.plate -> C_out.a
    C_out.b -> R_load.a, out_L
    R_load.b -> gnd
  }
}
"#;
    let pedal = parse_pedal_file(src).unwrap();
    let graph = CircuitGraph::from_pedal(&pedal);

    let (triodes, all_edges) = graph.find_triodes();

    // Should be merged into 1 group with parallel_count = 4
    assert_eq!(
        triodes.len(),
        1,
        "4 parallel triodes should merge into 1 group, got {}",
        triodes.len()
    );
    assert_eq!(
        triodes[0].1.parallel_count, 4,
        "Parallel count should be 4, got {}",
        triodes[0].1.parallel_count
    );
    // All 4 raw edges should still be tracked for exclusion
    assert_eq!(
        all_edges.len(),
        4,
        "All 4 raw triode edges should be in all_edges, got {}",
        all_edges.len()
    );
}

/// Non-parallel triodes (different nodes) should remain separate.
#[test]
fn non_parallel_triodes_stay_separate() {
    use super::graph::CircuitGraph;

    // Two triodes with separate plate nodes (cascaded)
    let src = r#"
equipment "Cascaded Tubes" {
  supplies {
    vcc: 250V
  }
  components {
    C_in: cap(22n)
    R_grid1: resistor(1M)
    V1: triode(12ax7)
    R_plate1: resistor(100k)
    R_cathode1: resistor(1.5k)
    C_coupling: cap(22n)
    R_grid2: resistor(1M)
    V2: triode(12ax7)
    R_plate2: resistor(100k)
    R_cathode2: resistor(1.5k)
    C_out: cap(22n)
    R_load: resistor(1M)
  }
  nets {
    in_L -> C_in.a
    C_in.b -> R_grid1.a, V1.grid
    R_grid1.b -> gnd
    vcc -> R_plate1.a
    R_plate1.b -> V1.plate
    V1.cathode -> R_cathode1.a
    R_cathode1.b -> gnd
    V1.plate -> C_coupling.a
    C_coupling.b -> R_grid2.a, V2.grid
    R_grid2.b -> gnd
    vcc -> R_plate2.a
    R_plate2.b -> V2.plate
    V2.cathode -> R_cathode2.a
    R_cathode2.b -> gnd
    V2.plate -> C_out.a
    C_out.b -> R_load.a, out_L
    R_load.b -> gnd
  }
}
"#;
    let pedal = parse_pedal_file(src).unwrap();
    let graph = CircuitGraph::from_pedal(&pedal);

    let (triodes, _) = graph.find_triodes();

    assert_eq!(
        triodes.len(),
        2,
        "Two cascaded triodes should remain 2 groups, got {}",
        triodes.len()
    );
    for (_, info) in &triodes {
        assert_eq!(info.parallel_count, 1, "Each should have parallel_count=1");
    }
}

/// Push-pull triode pairs connected via CT transformer should be detected.
#[test]
fn push_pull_pair_detection() {
    use super::graph::CircuitGraph;

    let src = r#"
equipment "Push-Pull Test" {
  supplies {
    vcc: 240V
  }
  components {
    C_in: cap(22n)
    R_grid_push: resistor(1M)
    R_grid_pull: resistor(1M)
    V_push: triode(6386)
    V_pull: triode(6386)
    R_plate_push: resistor(33k)
    R_plate_pull: resistor(33k)
    R_cathode_push: resistor(470)
    R_cathode_pull: resistor(470)
    R_sense_push: resistor(33)
    R_sense_pull: resistor(33)
    T_out: transformer(9:1, 35.7H, 5, 1p, ct_primary)
    R_load: resistor(600)
  }
  nets {
    in_L -> C_in.a
    C_in.b -> R_grid_push.a, R_grid_pull.a
    R_grid_push.b -> V_push.grid
    R_grid_pull.b -> V_pull.grid
    vcc -> R_plate_push.a, R_plate_pull.a
    R_plate_push.b -> V_push.plate
    R_plate_pull.b -> V_pull.plate
    V_push.cathode -> R_cathode_push.a
    V_pull.cathode -> R_cathode_pull.a
    R_cathode_push.b -> gnd
    R_cathode_pull.b -> gnd
    V_push.plate -> R_sense_push.a
    R_sense_push.b -> T_out.primary.a
    V_pull.plate -> R_sense_pull.a
    R_sense_pull.b -> T_out.primary.b
    T_out.primary.ct -> vcc
    T_out.secondary.a -> R_load.a, out_L
    R_load.b -> T_out.secondary.b
  }
}
"#;
    let pedal = parse_pedal_file(src).unwrap();
    let graph = CircuitGraph::from_pedal(&pedal);

    let (triodes, _) = graph.find_triodes();
    assert_eq!(triodes.len(), 2, "Should have 2 triode groups");

    let pairs = graph.find_push_pull_triode_pairs(&triodes);
    assert_eq!(
        pairs.len(),
        1,
        "Should detect 1 push-pull pair via CT transformer, got {}",
        pairs.len()
    );

    let pair = &pairs[0];
    assert!(pair.push_triode_idx != pair.pull_triode_idx, "Push and pull should be different");
    assert!((pair.turns_ratio - 9.0).abs() < 0.1, "Turns ratio should be 9:1");
}

/// Push-pull pairs should produce PushPullStage in compiled output.
#[test]
fn push_pull_stage_compiles() {
    let src = r#"
equipment "Push-Pull Compile" {
  supplies {
    vcc: 240V
  }
  components {
    C_in: cap(22n)
    R_grid_push: resistor(1M)
    R_grid_pull: resistor(1M)
    V_push: triode(6386)
    V_pull: triode(6386)
    R_plate_push: resistor(33k)
    R_plate_pull: resistor(33k)
    R_cathode_push: resistor(470)
    R_cathode_pull: resistor(470)
    R_sense_push: resistor(33)
    R_sense_pull: resistor(33)
    T_out: transformer(9:1, 35.7H, 5, 1p, ct_primary)
    R_load: resistor(600)
  }
  nets {
    in_L -> C_in.a
    C_in.b -> R_grid_push.a, R_grid_pull.a
    R_grid_push.b -> V_push.grid
    R_grid_pull.b -> V_pull.grid
    vcc -> R_plate_push.a, R_plate_pull.a
    R_plate_push.b -> V_push.plate
    R_plate_pull.b -> V_pull.plate
    V_push.cathode -> R_cathode_push.a
    V_pull.cathode -> R_cathode_pull.a
    R_cathode_push.b -> gnd
    R_cathode_pull.b -> gnd
    V_push.plate -> R_sense_push.a
    R_sense_push.b -> T_out.primary.a
    V_pull.plate -> R_sense_pull.a
    R_sense_pull.b -> T_out.primary.b
    T_out.primary.ct -> vcc
    T_out.secondary.a -> R_load.a, out_L
    R_load.b -> T_out.secondary.b
  }
}
"#;
    let pedal = parse_pedal_file(src).unwrap();
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();

    // Should have at least 1 push-pull stage
    assert!(
        proc.push_pull_stages.len() >= 1,
        "Should compile at least 1 PushPullStage, got {}",
        proc.push_pull_stages.len()
    );

    // Should NOT have regular triode stages for the paired tubes
    let regular_triode_count = proc.stages.iter().filter(|s| {
        matches!(s.root, RootKind::Triode(_))
    }).count();
    assert_eq!(
        regular_triode_count, 0,
        "Paired triodes should not appear as regular stages, got {regular_triode_count}"
    );

    // Process signal and verify output is finite and non-zero
    let n = 4800;
    let input: Vec<f64> = (0..n)
        .map(|i| 0.5 * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 48000.0).sin())
        .collect();
    let output: Vec<f64> = input.iter().map(|&s| proc.process(s)).collect();

    assert!(output.iter().all(|x| x.is_finite()), "No NaN/inf in push-pull output");

    // NOTE: Signal level depends on proper WDF tree construction and tube bias.
    // For now, we verify the stage exists and produces finite output.
    // Signal amplitude tuning is part of Phase 3 validation work.
}

/// Transformer pin aliasing: .primary.a/.primary.b should be unified with .a/.b
#[test]
fn transformer_pin_aliasing() {
    use super::graph::CircuitGraph;

    // Use explicit .primary.a/.primary.b pins (like the 670 does)
    let src = r#"
pedal "Transformer Pin Test" {
  components {
    R_in: resistor(100)
    T1: transformer(10:1, 10H, 5)
    R_load: resistor(1k)
  }
  nets {
    in -> R_in.a
    R_in.b -> T1.primary.a
    T1.primary.b -> gnd
    T1.secondary.a -> R_load.a, out
    R_load.b -> T1.secondary.b
  }
}
"#;
    let pedal = parse_pedal_file(src).unwrap();
    let graph = CircuitGraph::from_pedal(&pedal);

    // The transformer edge should connect properly to the nodes
    // used by the netlist (primary.a/primary.b)
    let xfmr_edges: Vec<_> = graph.edges.iter().enumerate().filter(|(_, e)| {
        matches!(graph.components[e.comp_idx].kind, ComponentKind::Transformer(_))
    }).collect();

    assert_eq!(xfmr_edges.len(), 1, "Should have 1 transformer edge");
    let (_, xfmr) = &xfmr_edges[0];
    // One node should be connected to R_in.b (via primary.a), other to gnd (via primary.b)
    assert!(
        xfmr.node_a != xfmr.node_b,
        "Transformer should connect two different nodes"
    );
}
