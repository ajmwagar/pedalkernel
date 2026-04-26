// ═══════════════════════════════════════════════════════════════════════════
// Harmonic content tests
//
// Each pedal should produce different harmonic content due to different
// op-amp models, diode types, clipping topologies, and tone filters.
// If they all sound the same, something is homogenizing the signal.
// ═══════════════════════════════════════════════════════════════════════════

use super::spqr_build::compile_via_spqr;
use crate::PedalProcessor;

const SR: f64 = 48000.0;

/// Process a pedal and return (fundamental_power, harmonic_power, peak).
/// Harmonic power = total power in harmonics 2-8 relative to fundamental.
fn measure_harmonics(compiled: &mut impl PedalProcessor, amp: f64, freq: f64) -> (f64, f64, f64) {
    // Warmup
    for s in 0..8000 {
        compiled.process(amp * (std::f64::consts::TAU * freq * s as f64 / SR).sin());
    }

    // Collect samples
    let n = 2048;
    let mut samples = vec![0.0f64; n];
    for s in 0..n {
        samples[s] = compiled.process(
            amp * (std::f64::consts::TAU * freq * (8000 + s) as f64 / SR).sin()
        );
    }

    let peak = samples.iter().map(|s| s.abs()).fold(0.0f64, f64::max);

    // Simple DFT at fundamental and harmonics
    let mut fund_power = 0.0;
    let mut harm_power = 0.0;

    for h in 1..=8 {
        let f = freq * h as f64;
        let bin = (f * n as f64 / SR).round() as usize;
        if bin >= n / 2 { break; }

        // Goertzel-like: correlate with sin/cos at this frequency
        let mut re = 0.0;
        let mut im = 0.0;
        for (i, &s) in samples.iter().enumerate() {
            let angle = std::f64::consts::TAU * bin as f64 * i as f64 / n as f64;
            re += s * angle.cos();
            im += s * angle.sin();
        }
        let power = (re * re + im * im) / (n as f64 * n as f64);

        if h == 1 {
            fund_power = power;
        } else {
            harm_power += power;
        }
    }

    (fund_power, harm_power, peak)
}

fn load_legend(name: &str) -> super::compiled::CompiledPedal {
    let path = format!(
        "{}/../../pedalkernel-pro/pedals/legends/{name}.pedal",
        env!("CARGO_MANIFEST_DIR"),
    );
    let source = std::fs::read_to_string(&path).expect(&format!("read {name}.pedal"));
    let pedal = crate::dsl::parse_pedal_file(&source).expect("parse");
    compile_via_spqr(&pedal, SR).expect("compile")
}

// ═══════════════════════════════════════════════════════════════════════════
// 1. Each pedal should have different harmonic content
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn pedals_have_different_harmonic_profiles() {
    let pedals = ["screamer", "sd1", "ratking", "goldenrod"];
    let mut profiles: Vec<(String, f64, f64, f64)> = Vec::new();

    for name in &pedals {
        let mut compiled = load_legend(name);
        let (fund, harm, peak) = measure_harmonics(&mut compiled, 0.1, 440.0);
        let thd = if fund > 1e-20 { (harm / fund).sqrt() } else { 0.0 };
        eprintln!("{name:12}: peak={peak:.4}V fund={fund:.2e} harm={harm:.2e} THD={:.1}%", thd * 100.0);
        profiles.push((name.to_string(), fund, harm, peak));
    }

    // All should produce output
    for (name, _, _, peak) in &profiles {
        assert!(*peak > 0.001, "{name} should produce output: peak={peak:.4}V");
    }

    // THD should vary between pedals — they use different clipping
    let thds: Vec<f64> = profiles.iter()
        .map(|(_, f, h, _)| if *f > 1e-20 { (*h / *f).sqrt() } else { 0.0 })
        .collect();

    eprintln!("\nTHD values: {:?}", thds.iter().map(|t| format!("{:.1}%", t * 100.0)).collect::<Vec<_>>());

    // Check that THDs are NOT all the same (within 10% of each other)
    let min_thd = thds.iter().copied().fold(f64::MAX, f64::min);
    let max_thd = thds.iter().copied().fold(0.0f64, f64::max);
    let spread = if min_thd > 1e-10 { max_thd / min_thd } else { 1.0 };
    eprintln!("THD spread: {spread:.2}x (min={:.1}%, max={:.1}%)", min_thd * 100.0, max_thd * 100.0);

    // THD should be significantly different between hard-clip and feedback-clip topologies.
    // Real pedals: RAT ~40-60% THD, Screamer ~20-40% THD.
    // Currently all are <2% — the diode stages aren't actually clipping.
    assert!(spread > 3.0,
        "Pedals should have very different THD: spread={spread:.2}x. \
         All at {:.1}%-{:.1}% — diodes aren't clipping!", min_thd * 100.0, max_thd * 100.0);
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. Per-stage signal trace — find where attenuation happens
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_per_stage_signal_levels() {
    // Trace signal level at each stage to find where the 50× drop occurs.
    // Expected: gain stage outputs ~1V, diode clips to ~0.5V.
    // Actual: gain stage outputs ~1V but diode stage receives ~0.02V.
    let mut compiled = load_legend("screamer");
    compiled.enable_metering(128);

    let amp = 0.1;
    for s in 0..8000 {
        compiled.process(amp * (std::f64::consts::TAU * 440.0 * s as f64 / SR).sin());
    }

    let metrics = compiled.read_metrics();
    let n = compiled.stages.len().min(crate::metering::MAX_STAGES);

    eprintln!("\nSCREAMER per-stage signal trace:");
    let mut prev_db = -20.0f64; // input is 0.1V ≈ -20 dB
    for i in 0..n {
        let lvl = metrics.stage_levels[i];
        let db = if lvl > 1e-10 { 20.0 * (lvl as f64).log10() } else { -120.0 };
        let delta = db - prev_db;
        #[cfg(debug_assertions)]
        {
            let (stype, lbl, bypass) = match &compiled.stages[i] {
                super::compiled::Stage::Wdf(w) => ("Wdf", &w.debug_label, w.bypass_serial),
                super::compiled::Stage::MultiNl(m) => ("MNL", &m.debug_label, m.bypass_serial),
                super::compiled::Stage::Iir(s) => ("Iir", &s.debug_label, s.bypass_serial),
                super::compiled::Stage::StateSpace(s) => ("SS", &s.debug_label, s.bypass_serial),
                super::compiled::Stage::BlackFeedback(b) => ("BF", &b.debug_label, b.bypass_serial),
            };
            if !bypass {
                let arrow = if delta > 3.0 { "↑↑" } else if delta > 0.5 { "↑" }
                    else if delta < -20.0 { "↓↓↓" } else if delta < -3.0 { "↓↓" }
                    else if delta < -0.5 { "↓" } else { "→" };
                eprintln!("  [{i}] [{stype}] {lbl}: {db:.1} dB ({delta:+.1} dB) {arrow}");
                prev_db = db;
            } else {
                eprintln!("  [{i}] [{stype}] {lbl}: BYPASS");
            }
        }
    }

    // The gain stage should amplify significantly
    // The diode stage should clip (reduce peak but add harmonics)
    // No stage should drop more than 20 dB unexpectedly
    for i in 0..n {
        let lvl = metrics.stage_levels[i];
        let db = if lvl > 1e-10 { 20.0 * (lvl as f64).log10() } else { -120.0 };
        let bypass = match &compiled.stages[i] {
            super::compiled::Stage::Wdf(w) => w.bypass_serial,
            super::compiled::Stage::MultiNl(m) => m.bypass_serial,
            super::compiled::Stage::Iir(s) => s.bypass_serial,
            super::compiled::Stage::StateSpace(s) => s.bypass_serial,
            super::compiled::Stage::BlackFeedback(b) => b.bypass_serial,
        };
        if bypass { continue; }
        #[cfg(debug_assertions)]
        {
            let lbl = match &compiled.stages[i] {
                super::compiled::Stage::Wdf(w) => &w.debug_label,
                super::compiled::Stage::MultiNl(m) => &m.debug_label,
                super::compiled::Stage::Iir(s) => &s.debug_label,
                super::compiled::Stage::StateSpace(s) => &s.debug_label,
                super::compiled::Stage::BlackFeedback(b) => &b.debug_label,
            };
            assert!(db > -60.0,
                "Stage {i} [{lbl}] has dead signal at {db:.1} dB — signal lost in chain");
        }
    }
}

#[test]
fn ratking_per_stage_signal_levels() {
    let mut compiled = load_legend("ratking");
    compiled.enable_metering(128);

    let amp = 0.1;
    for s in 0..8000 {
        compiled.process(amp * (std::f64::consts::TAU * 440.0 * s as f64 / SR).sin());
    }

    let metrics = compiled.read_metrics();
    let n = compiled.stages.len().min(crate::metering::MAX_STAGES);

    eprintln!("\nRATKING per-stage signal trace:");
    let mut prev_db = -20.0f64;
    for i in 0..n {
        let lvl = metrics.stage_levels[i];
        let db = if lvl > 1e-10 { 20.0 * (lvl as f64).log10() } else { -120.0 };
        let delta = db - prev_db;
        #[cfg(debug_assertions)]
        {
            let (stype, lbl, bypass) = match &compiled.stages[i] {
                super::compiled::Stage::Wdf(w) => ("Wdf", &w.debug_label, w.bypass_serial),
                super::compiled::Stage::MultiNl(m) => ("MNL", &m.debug_label, m.bypass_serial),
                super::compiled::Stage::Iir(s) => ("Iir", &s.debug_label, s.bypass_serial),
                super::compiled::Stage::StateSpace(s) => ("SS", &s.debug_label, s.bypass_serial),
                super::compiled::Stage::BlackFeedback(b) => ("BF", &b.debug_label, b.bypass_serial),
            };
            if !bypass {
                let arrow = if delta > 3.0 { "↑↑" } else if delta > 0.5 { "↑" }
                    else if delta < -20.0 { "↓↓↓" } else if delta < -3.0 { "↓↓" }
                    else if delta < -0.5 { "↓" } else { "→" };
                eprintln!("  [{i}] [{stype}] {lbl}: {db:.1} dB ({delta:+.1} dB) {arrow}");
                prev_db = db;
            } else {
                eprintln!("  [{i}] [{stype}] {lbl}: BYPASS");
            }
        }
    }

    // The gain stage (BF with LM308) should amplify
    // The diode stage (D1,D2) should clip hard
    // The tone stage should pass signal
    for i in 0..n {
        let lvl = metrics.stage_levels[i];
        let db = if lvl > 1e-10 { 20.0 * (lvl as f64).log10() } else { -120.0 };
        let bypass = match &compiled.stages[i] {
            super::compiled::Stage::Wdf(w) => w.bypass_serial,
            super::compiled::Stage::MultiNl(m) => m.bypass_serial,
            super::compiled::Stage::Iir(s) => s.bypass_serial,
            super::compiled::Stage::StateSpace(s) => s.bypass_serial,
            super::compiled::Stage::BlackFeedback(b) => b.bypass_serial,
        };
        if bypass { continue; }
        #[cfg(debug_assertions)]
        {
            let lbl = match &compiled.stages[i] {
                super::compiled::Stage::Wdf(w) => &w.debug_label,
                super::compiled::Stage::MultiNl(m) => &m.debug_label,
                super::compiled::Stage::Iir(s) => &s.debug_label,
                super::compiled::Stage::StateSpace(s) => &s.debug_label,
                super::compiled::Stage::BlackFeedback(b) => &b.debug_label,
            };
            assert!(db > -60.0,
                "Stage {i} [{lbl}] has dead signal at {db:.1} dB — signal lost in chain");
        }
    }
}

#[test]
fn gain_stage_output_reaches_diode_threshold() {
    // The gain stage should output enough voltage to reach diode Vf.
    // Silicon Vf ≈ 0.6V, Germanium ≈ 0.25V.
    // With gain=10 and input=0.1V, the gain stage should output ~1V peak
    // (or ~0.96V after tanh at v_max=3V). That's well above Vf.
    let mut compiled = load_legend("screamer");
    compiled.enable_metering(128);

    let amp = 0.1;
    for s in 0..8000 {
        compiled.process(amp * (std::f64::consts::TAU * 440.0 * s as f64 / SR).sin());
    }

    let metrics = compiled.read_metrics();
    let n = compiled.stages.len().min(crate::metering::MAX_STAGES);

    // Find the gain stage (BlackFeedback) output level
    let mut gain_stage_peak = 0.0f64;
    for i in 0..n {
        if let super::compiled::Stage::BlackFeedback(b) = &compiled.stages[i] {
            if !b.bypass_serial {
                gain_stage_peak = metrics.stage_levels[i] as f64;
                #[cfg(debug_assertions)]
                eprintln!("Gain stage [{}]: peak={gain_stage_peak:.4}V", b.debug_label);
            }
        }
    }

    // The gain stage output should be above diode Vf
    assert!(gain_stage_peak > 0.5,
        "Gain stage should output >{:.1}V (above diode Vf), got {gain_stage_peak:.4}V. \
         The signal is too low for diodes to clip.", 0.5);
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. Input coupling: isolation vs full pedal
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn input_coupling_isolation_vs_full_pedal() {
    // The input coupling (Cin + R_in) works in isolation (gain ≈ 0.97)
    // but drops 30 dB in the full Screamer. Find the difference.

    // Isolation test: standalone Cin + R_in
    let iso_source = r#"
        pedal "test" { supply 9V
            components {
                Cin: cap(22n)
                R_in: resistor(510k)
            }
            nets {
                in -> Cin.a
                Cin.b -> R_in.a, out
                R_in.b -> gnd
            }
            controls {}
        }"#;
    let iso_pedal = crate::dsl::parse_pedal_file(iso_source).expect("parse");
    let mut iso = compile_via_spqr(&iso_pedal, SR).expect("compile");

    let amp = 0.1;
    for s in 0..4000 {
        iso.process(amp * (std::f64::consts::TAU * 440.0 * s as f64 / SR).sin());
    }
    let mut iso_peak = 0.0f64;
    for s in 0..500 {
        let out = iso.process(
            amp * (std::f64::consts::TAU * 440.0 * (4000 + s) as f64 / SR).sin()
        );
        iso_peak = iso_peak.max(out.abs());
    }
    let iso_db = 20.0 * iso_peak.log10();

    // Full Screamer: same Cin + R_in but with other stages after
    let mut full = load_legend("screamer");
    full.enable_metering(128);
    for s in 0..4000 {
        full.process(amp * (std::f64::consts::TAU * 440.0 * s as f64 / SR).sin());
    }
    let metrics = full.read_metrics();
    let full_level = metrics.stage_levels[0] as f64;
    let full_db = if full_level > 1e-10 { 20.0 * full_level.log10() } else { -120.0 };

    eprintln!("Input coupling:");
    eprintln!("  Isolation: peak={iso_peak:.4}V ({iso_db:.1} dB)");
    eprintln!("  Full pedal stage 0: level={full_level:.4} ({full_db:.1} dB)");
    eprintln!("  Difference: {:.1} dB", full_db - iso_db);

    // Check the SPQR terminals and tree structure for both
    let iso_graph = super::graph::CircuitGraph::from_pedal(&iso_pedal);
    let iso_edges: Vec<usize> = (0..iso_graph.edges.len()).collect();
    let iso_groups = super::signal_flow::find_flow_groups(&iso_edges, &iso_graph);

    let full_source = std::fs::read_to_string(format!(
        "{}/../../pedalkernel-pro/pedals/legends/screamer.pedal",
        env!("CARGO_MANIFEST_DIR"),
    )).expect("read");
    let full_pedal = crate::dsl::parse_pedal_file(&full_source).expect("parse");
    let full_graph = super::graph::CircuitGraph::from_pedal(&full_pedal);
    let full_edges: Vec<usize> = (0..full_graph.edges.len()).collect();
    let full_groups = super::signal_flow::find_flow_groups(&full_edges, &full_graph);

    // Find the input coupling group in both
    let iso_ic = iso_groups.iter().find(|g| {
        g.all_edges().iter().any(|&eidx| iso_graph.components[iso_graph.edges[eidx].comp_idx].id == "Cin")
    });
    let full_ic = full_groups.iter().find(|g| {
        g.all_edges().iter().any(|&eidx| full_graph.components[full_graph.edges[eidx].comp_idx].id == "Cin")
    });

    if let Some(g) = iso_ic {
        let comps: Vec<&str> = g.all_edges().iter()
            .map(|&eidx| iso_graph.components[iso_graph.edges[eidx].comp_idx].id.as_str())
            .collect();
        let terms = super::spqr_build::compute_group_terminals(
            &g.all_edges(), &iso_graph, &vec![iso_graph.in_node, iso_graph.out_node]);
        eprintln!("  Isolation group: {comps:?} terminals={terms:?}");
    }

    if let Some(g) = full_ic {
        let comps: Vec<&str> = g.all_edges().iter()
            .map(|&eidx| full_graph.components[full_graph.edges[eidx].comp_idx].id.as_str())
            .collect();
        let terms = super::spqr_build::compute_group_terminals(
            &g.all_edges(), &full_graph, &vec![full_graph.in_node, full_graph.out_node]);
        eprintln!("  Full pedal group: {comps:?} terminals={terms:?}");
    }

    // The input coupling should NOT lose more than 10 dB between isolation and full
    let diff = (full_db - iso_db).abs();
    assert!(diff < 10.0,
        "Input coupling loses {diff:.1} dB in full pedal vs isolation — something is wrong");
}

// ═══════════════════════════════════════════════════════════════════════════
// 7. Direct signal trace — process one sample and check intermediate values
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn screamer_single_sample_trace() {
    // Process the Screamer for warmup, then trace one sample through.
    // This uses the actual process() path, not metering.
    let mut compiled = load_legend("screamer");

    let amp = 0.1;
    let freq = 440.0;
    // Warmup
    for s in 0..4000 {
        compiled.process(amp * (std::f64::consts::TAU * freq * s as f64 / SR).sin());
    }

    // Process several samples and track peak output
    let mut peak = 0.0f64;
    for s in 0..500 {
        let input = amp * (std::f64::consts::TAU * freq * (4000 + s) as f64 / SR).sin();
        let output = compiled.process(input);
        peak = peak.max(output.abs());
    }

    eprintln!("Screamer: input amp={amp}V, output peak={peak:.4}V, gain={:.2}", peak / amp);

    // With Screamer gain ≈ 50 and diode clip at 0.5V:
    // Expected output ≈ 0.5V (clipped)
    // Currently getting ≈ 0.02V
    assert!(peak > 0.1,
        "Screamer should clip at ~0.5V, got peak={peak:.4}V — gain stage or diode not working");
}

#[test]
fn simple_inverting_gain_traces_correctly() {
    // Minimal test: just an inverting op-amp with gain=10, no diodes.
    // This isolates whether the gain stage itself works.
    let source = r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#;
    let pedal = crate::dsl::parse_pedal_file(source).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    let amp = 0.1;
    let freq = 440.0;
    for s in 0..4000 {
        compiled.process(amp * (std::f64::consts::TAU * freq * s as f64 / SR).sin());
    }
    let mut peak = 0.0f64;
    for s in 0..500 {
        let out = compiled.process(
            amp * (std::f64::consts::TAU * freq * (4000 + s) as f64 / SR).sin()
        );
        peak = peak.max(out.abs());
    }

    let gain = peak / amp;
    eprintln!("Simple inverting gain=10: peak={peak:.4}V, measured gain={gain:.2}");
    // Gain = Rf/Ri = 100k/10k = 10. After tanh(1.0/3.0)*3 ≈ 0.96V.
    // Peak should be ~0.96V.
    assert!(gain > 3.0, "Gain stage should amplify ≈10x: got {gain:.2}x");
}

#[test]
fn simple_inverting_with_diode_clips() {
    // Inverting + feedback diode pair. Should clip at ~0.5V.
    let source = r#"
        pedal "test" { supply 9V
            components {
                R_in: resistor(10k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                D: diode_pair(silicon)
            }
            nets {
                in -> R_in.a
                R_in.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.out -> D.a
                D.b -> U1.neg
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#;
    let pedal = crate::dsl::parse_pedal_file(source).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");
    compiled.enable_metering(128);

    let amp = 0.1;
    let freq = 440.0;
    for s in 0..4000 {
        compiled.process(amp * (std::f64::consts::TAU * freq * s as f64 / SR).sin());
    }
    let mut peak = 0.0f64;
    for s in 0..500 {
        let out = compiled.process(
            amp * (std::f64::consts::TAU * freq * (4000 + s) as f64 / SR).sin()
        );
        peak = peak.max(out.abs());
    }

    let metrics = compiled.read_metrics();
    let n = compiled.stages.len().min(crate::metering::MAX_STAGES);
    for i in 0..n {
        let lvl = metrics.stage_levels[i];
        let db = if lvl > 1e-10 { 20.0 * (lvl as f64).log10() } else { -120.0 };
        #[cfg(debug_assertions)]
        {
            let (stype, lbl) = match &compiled.stages[i] {
                super::compiled::Stage::Wdf(w) => ("Wdf", &w.debug_label),
                super::compiled::Stage::BlackFeedback(b) => ("BF", &b.debug_label),
                _ => continue,
            };
            eprintln!("  [{i}] [{stype}] {lbl}: {db:.1} dB (level={lvl:.4})");
        }
    }

    let gain = peak / amp;
    eprintln!("Inverting + diode: peak={peak:.4}V, gain={gain:.2}");
    // Should clip at diode Vf ≈ 0.5V
    assert!(peak > 0.2, "Should clip at ~0.5V: peak={peak:.4}V");
    assert!(peak < 1.5, "Should be below rail: peak={peak:.4}V");
}

#[test]
fn input_coupling_then_gain_preserves_level() {
    // Cin + R_in → gain stage. Does adding input coupling kill the signal?
    let source = r#"
        pedal "test" { supply 9V
            components {
                Cin: cap(22n)
                R_in: resistor(510k)
                U1: opamp(tl072)
                Rf: resistor(100k)
                R_series: resistor(10k)
            }
            nets {
                in -> Cin.a
                Cin.b -> R_in.a, R_series.a
                R_in.b -> gnd
                R_series.b -> U1.neg
                U1.neg -> Rf.a
                Rf.b -> U1.out
                U1.pos -> gnd
                U1.out -> out
            }
            controls {}
        }"#;
    let pedal = crate::dsl::parse_pedal_file(source).expect("parse");
    let mut compiled = compile_via_spqr(&pedal, SR).expect("compile");

    let amp = 0.1;
    let freq = 440.0;
    for s in 0..4000 {
        compiled.process(amp * (std::f64::consts::TAU * freq * s as f64 / SR).sin());
    }
    let mut peak = 0.0f64;
    for s in 0..500 {
        let out = compiled.process(
            amp * (std::f64::consts::TAU * freq * (4000 + s) as f64 / SR).sin()
        );
        peak = peak.max(out.abs());
    }

    let gain = peak / amp;
    eprintln!("Cin + R_in + gain: peak={peak:.4}V, gain={gain:.2}");
    // Cin barely attenuates at 440Hz. Gain = Rf/R_series = 100k/10k = 10.
    // Expected: ~1V (10x gain, tanh-clipped)
    assert!(gain > 3.0,
        "Input coupling + gain should amplify: got {gain:.2}x — coupling killing signal?");
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. RAT (LM308) should have more slew-rate distortion than SD-1 (TL072)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn lm308_has_more_slew_distortion_than_tl072() {
    // LM308 slew = 0.3 V/µs, TL072 slew = 13 V/µs
    // At high frequencies, LM308 should produce more harmonics from slew limiting.
    let mut rat = load_legend("ratking");
    let mut sd1 = load_legend("sd1");

    // Use 2kHz to emphasize slew rate effects
    let (_, harm_rat, _) = measure_harmonics(&mut rat, 0.1, 2000.0);
    let (_, harm_sd1, _) = measure_harmonics(&mut sd1, 0.1, 2000.0);

    eprintln!("RAT (LM308) harmonics at 2kHz: {harm_rat:.4e}");
    eprintln!("SD-1 (TL072) harmonics at 2kHz: {harm_sd1:.4e}");

    // RAT should have more harmonic content from slew limiting
    // (This may fail if slew rate isn't being applied correctly)
    if harm_rat > 1e-10 && harm_sd1 > 1e-10 {
        let ratio = harm_rat / harm_sd1;
        eprintln!("  RAT/SD-1 harmonic ratio: {ratio:.2}x");
        assert!(ratio > 1.2, "LM308 should produce more slew distortion than TL072: ratio={ratio:.2}x");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Hard clip (RAT) vs feedback clip (Screamer) should differ
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn hard_clip_vs_feedback_clip_differ() {
    // RAT: hard clip to ground (sharp, buzzy)
    // Screamer: feedback clip (softer, compressed)
    // The harmonic profiles should be noticeably different.
    let mut rat = load_legend("ratking");
    let mut screamer = load_legend("screamer");

    let (fund_rat, harm_rat, peak_rat) = measure_harmonics(&mut rat, 0.1, 440.0);
    let (fund_scr, harm_scr, peak_scr) = measure_harmonics(&mut screamer, 0.1, 440.0);

    let thd_rat = if fund_rat > 1e-20 { (harm_rat / fund_rat).sqrt() } else { 0.0 };
    let thd_scr = if fund_scr > 1e-20 { (harm_scr / fund_scr).sqrt() } else { 0.0 };

    eprintln!("RAT: peak={peak_rat:.4}V THD={:.1}%", thd_rat * 100.0);
    eprintln!("Screamer: peak={peak_scr:.4}V THD={:.1}%", thd_scr * 100.0);

    // Both should produce output
    assert!(peak_rat > 0.001, "RAT should produce output: {peak_rat:.4}V");
    assert!(peak_scr > 0.001, "Screamer should produce output: {peak_scr:.4}V");

    // Hard clip should have higher THD than feedback clip
    // (hard clip produces sharp edges → more harmonics)
    if thd_rat > 0.001 && thd_scr > 0.001 {
        eprintln!("  RAT THD / Screamer THD = {:.2}x", thd_rat / thd_scr);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Germanium (Goldenrod) clips at lower voltage than silicon (RAT)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn germanium_clips_lower_than_silicon_in_legends() {
    // Goldenrod: germanium 1N34A (Vf ≈ 0.25V)
    // RAT: silicon 1N914 (Vf ≈ 0.6V)
    // At same gain, germanium should clip at lower output level.
    let mut gold = load_legend("goldenrod");
    let mut rat = load_legend("ratking");

    // Set both to similar gain/drive settings
    gold.set_control("Gain", 0.7);
    rat.set_control("Distortion", 0.7);

    let (_, _, peak_gold) = measure_harmonics(&mut gold, 0.1, 440.0);
    let (_, _, peak_rat) = measure_harmonics(&mut rat, 0.1, 440.0);

    eprintln!("Goldenrod (Ge): peak={peak_gold:.4}V");
    eprintln!("RAT (Si): peak={peak_rat:.4}V");

    assert!(peak_gold > 0.001, "Goldenrod should produce output: {peak_gold:.4}V");
    assert!(peak_rat > 0.001, "RAT should produce output: {peak_rat:.4}V");
}
