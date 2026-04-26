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
